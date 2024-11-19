#include <stdint.h>
#include <stdio.h>
#include <pico/bootrom.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/multicore.h>
#include <pico/unique_id.h>
#include <hardware/pio.h>
#include <bsp/board.h>
#include <tusb.h>
#include "pio_jtag.h"
#include "led.h"
#include "cmd.h"
#include "get_serial.h"
#include "git.h"

#include "dirtyJtagConfig.h"

//#define DEBUG

#define MTU_SIZE 1500

// round up the buffer size to the next multiple of 64 bytes
#define BUFFER_SIZE ((MTU_SIZE + 63) & ~63)

#ifdef DEBUG
#define dprintf(...) printf(__VA_ARGS__)
#else
#define dprintf(...) (void)(__VA_ARGS__)
#endif

void init_pins()
{
  bi_decl(bi_4pins_with_names(PIN_TCK, "TCK", PIN_TDI, "TDI", PIN_TDO, "TDO", PIN_TMS, "TMS"));
  bi_decl(bi_1pin_with_name(PIN_RST, "SRST#"));
}

pio_jtag_inst_t jtag = {
  .pio = pio0,
  .sm = 0
};
pio_a5clk_inst_t a5clk = {
  .pio = pio1,
  .sm = 0
};

static char whoami[512];

const char *djtag_whoami()
{
  return whoami;
}

static void fatal_error()
{
  printf ("Rebooting to bootloader...\n");
  // this is fatal - wait 100ms then reboot to bootloader
  sleep_ms(100);
  reset_usb_boot(0, 0);
  while (1) asm volatile ("wfe");
}

void djtag_init()
{
  init_pins();
  init_jtag(&jtag, 1000, PIN_TCK, PIN_TDI, PIN_TDO, PIN_TMS, PIN_RST);
  init_a5clk(&a5clk, 1000, PIN_A5_CLK);
}

// in ethernet.c
int eth_init(uint64_t);

// callback for updating the ID part
// connected (-1: err), hostname, MAC, server listening, client connected
// if connected==-1. "hostname" is the error
static uint64_t usbserial = 0;

static char *ipconfig_ptr;
static const char *hostname = "???", *macaddr = "???";

void notify_ip_config(int link, const char *mac, const char *host, const char *srv, const char *cli)
{
  if (link != -1) {
    if (mac) macaddr = mac;
    if (hostname) hostname = host;
  }
  if (! ipconfig_ptr)
    return;
  char *p = ipconfig_ptr;
  static int last_link = -1;
  static const char *last_srv = 0, *last_cli = 0;
  if (link == -1)
    sprintf (p, "eth error: %s", host);
  else {
    p += sprintf (p, "eth: link ");
    if (! link)
      sprintf (p, "down");
    else {
      p += sprintf (p, "up; ");
      if (! srv)
        sprintf (p, "waiting for IP...");
      else if (! cli)
        sprintf (p, "listening on %s...", srv);
      else 
        sprintf (p, "serving on %s; connected to %s", srv, cli);
    }
  }
  // avoid duplicate messages
  if ((link != last_link) || (srv != last_srv) || (cli != last_cli))
    puts(ipconfig_ptr);
  last_link = link; last_srv = srv; last_cli = cli;
}

// buffers
typedef struct buffer_s
{
  volatile enum {
    UNUSED = 0, RECEIVING, RECEIVED,
      READY_TO_EXECUTE = RECEIVED,
    EXECUTING, EXECUTED,
      READY_TO_SEND = EXECUTED, 
    SENDING, SENT,
      DONE = SENT
  } state;
  // need a resp_pos (next position to send)
  // a cmd_pos is not needed, since cmd_n works as cmd_pos
  volatile unsigned cmd_n, resp_n, resp_pos;
  uint8_t cmd[BUFFER_SIZE], resp[BUFFER_SIZE];
} buffer_t;
buffer_t usbbuf, ethbuf;

// receive tasks, usb and ethernet
// they return -1 on no data, bufidx on got data
static void usb_rx_task();
static void eth_rx_task();
// run the jtag task in a specific buffer
static void run_jtag_task(buffer_t*);
// check if any task completed
static void check_jtag_tasks();
// respond tasks, usb and ethernet
static void usb_tx_task();
static void eth_tx_task();

// core1 runs the jtag tasks
static void core1_entry();

//-----------------------------------------------------------------------------

int main()
{
  board_init();
  led_init(PIN_LED);
  usb_serial_init();
  tusb_init();
  stdio_uart_init_full(uart1, 115200, 8, -1); // enable stdio over uart1 on pin_tx=gp8, no rx

  // get the Pico's serial ID
  pico_unique_board_id_t uID;
  pico_get_unique_board_id(&uID);
  usbserial = \
    ((uint64_t)uID.id[0] << 56) | ((uint64_t)uID.id[1] << 48) | 
    ((uint64_t)uID.id[2] << 40) | ((uint64_t)uID.id[3] << 32) | 
    ((uint64_t)uID.id[4] << 24) | ((uint64_t)uID.id[5] << 16) | 
    ((uint64_t)uID.id[6] <<  8) | ((uint64_t)uID.id[7] <<  0);
  eth_init(usbserial);

  // initialize the header
  snprintf (whoami, sizeof(whoami), 
            "DirtyJTAG2-pico %s %s%s %s [custom]\n"
            "  with: A5clk, driven_RST#, pincfg_setall\n"
            "USB serial: %016llX; MAC: %s\n"
            "Label: %04X; Hostname: %s\n", 
            git_Branch, git_Describe, git_AnyUncommittedChanges?"(dirty)":"", git_Remote,
            usbserial, macaddr,
            (uint32_t)(usbserial >> 8) & 0xFFFF, hostname);
  ipconfig_ptr = whoami + strlen(whoami);

  // printf the header
  printf ("\n\n----\n%s", whoami);

  multicore_launch_core1(core1_entry);
  while (1) {
    // see if we have anything incoming
    if (usbbuf.state < READY_TO_EXECUTE)
      usb_rx_task();
    if (ethbuf.state < READY_TO_EXECUTE)
      eth_rx_task();
    // kick off received tasks
    if (usbbuf.state == READY_TO_EXECUTE) // usb_rx_task() might update this
      run_jtag_task(&usbbuf);
    if (ethbuf.state == READY_TO_EXECUTE) // eth_rx_task() might update this
      run_jtag_task(&ethbuf);
    // advance either buffer's state if it advanced
    check_jtag_tasks();
    // send the response, if any 
    if (usbbuf.state > READY_TO_EXECUTE) // check_jtag_tasks() might update this
      usb_tx_task();
    if (ethbuf.state > READY_TO_EXECUTE) // check_jtag_tasks() might update this
      eth_tx_task();
  }
}

// Core 1 runs the JTAG tasks:
// - receive ID of buffer to execute 
// - execute and update buffer
// - send ID of buffer that executed
void core1_entry() {
  djtag_init();
  while (1) {
    buffer_t *crtbuf = (buffer_t*)multicore_fifo_pop_blocking();
    assert (crtbuf->state == EXECUTING);
    crtbuf->resp_n = cmd_execute(&jtag, (crtbuf==&usbbuf)?'U':'E', crtbuf->cmd, crtbuf->cmd_n, crtbuf->resp);
    multicore_fifo_push_blocking((uint32_t)crtbuf);
  }
}

// USB receive task
// since the EP size is 64, and we might want more, we expect packets of size 64 until we get one that isn't
// of size 64; in practical terms, this means that the client must pad the data by at least 1 byte if the intended
// sequence length is exactly 64*N
void usb_rx_task()
{
  // do nothing unless the usb buffer is either UNUSED or RECEIVING
  if ((usbbuf.state != UNUSED) && (usbbuf.state != RECEIVING))
    return;
  // run the TinyUSB Device task *unless* we already have available data
  int n = tud_vendor_available();
  if (n < 1) {
    tud_task();
    n = tud_vendor_available();
  }
  // if we haven't got any data, we have nothing left to do
  if (n < 1)
    return;
  // reset the cmd pointer, if we're just entering the RECEIVING state
  if (usbbuf.state != RECEIVING) {
    usbbuf.cmd_n = 0;
    usbbuf.state = RECEIVING;
  }
  n = tud_vendor_read(usbbuf.cmd + usbbuf.cmd_n, 64);
  if (n < 1)
    return;
  // update the read pointer
  usbbuf.cmd_n += n;
  dprintf (" U< %u(%u)\n", n, usbbuf.cmd_n); // <{received}({total_received})
  // if we got a max-size packet, stay in receive mode - more will follow
  if (n == 64)
    return;
  // if we got a fractional packet, we're done - ready to execute
  usbbuf.state = READY_TO_EXECUTE;
# ifdef DEBUG
  printf ("usbbuf ready to execute (cmd_sz=%u)\n", usbbuf.cmd_n);
# endif
}

// USB transmit task
// no need to do crazy size-related stuff here, since the host knows exactly how many bytes it wants
void usb_tx_task()
{
  // if we have nothing to send back, just mark the buffer as available
  if ((usbbuf.state == EXECUTED) && (! usbbuf.resp_n)) {
    // usbbuf.state = DONE;
    usbbuf.state = UNUSED;
    return;
  }
  // do nothing unless the usb buffer is either READY_TO_SEND or SENDING
  if ((usbbuf.state != READY_TO_SEND) && (usbbuf.state != SENDING))
    return;
  // enqueue as much data as we can (if any)
  unsigned avl = tud_vendor_write_available();
usb_send_more:
  // if we can't send any data, we have nothing left to do *for now*
  if (avl < 1)
    return;
  // reset the response pointer, if we're just entering the SENDING state
  if (usbbuf.state != SENDING) {
    usbbuf.resp_pos = 0;
    usbbuf.state = SENDING;
  }
  unsigned n0 = usbbuf.resp_n - usbbuf.resp_pos;
  unsigned n = n0;
  if (n > avl)
    n = avl;
  // enqueue any data, if we could
  dprintf (" U> %u(%u)\n", n, n0 - n); // >{transmitted}({left})
  tud_vendor_write(usbbuf.resp + usbbuf.resp_pos, n);
  tud_vendor_flush();
  // run the TinyUSB device task
  tud_task();
  // update output pointer (and the state, if we're done)
  usbbuf.resp_pos += n;
  if (usbbuf.resp_pos == usbbuf.resp_n) {
    // usbbuf.state = DONE;
    usbbuf.state = UNUSED;
  }
  // keep sending, if we can
  else {
    avl = tud_vendor_write_available();
    goto usb_send_more;
  }
}

// ETH receive task
void eth_task();
int fetch_eth_data(char *dest);
void eth_rx_task()
{
  // do nothing unless the eth buffer is either UNUSED or RECEIVING
  if ((ethbuf.state != UNUSED) && (ethbuf.state != RECEIVING))
    return;
  // reset the cmd pointer, if we're just entering the RECEIVING state
  if (ethbuf.state != RECEIVING) {
    ethbuf.cmd_n = 0;
    ethbuf.state = RECEIVING; // transitory!
  }
  // run the Ethernet task
  eth_task();
  // grab data from Ethernet
  int n = fetch_eth_data(ethbuf.cmd);
  if (n < 1)
    return;
  // set the read pointer
  ethbuf.cmd_n = n;
  dprintf (" E< %u\n", n); // <{received}
  // we're done - ready to execute
  ethbuf.state = READY_TO_EXECUTE;
# ifdef DEBUG
  printf ("ethbuf ready to execute (cmd_sz=%u)\n", ethbuf.cmd_n);
# endif
}

// ETH transmit task
int submit_eth_data(const char *src, unsigned n);
int eth_sending_data();
void eth_tx_task()
{
  // for ethernet, a zero-length response still needs to be sent, per protocol
  // (which is purely ping-pong)
  // do nothing unless the eth buffer is READY_TO_SEND or SENDING
  if ((ethbuf.state != READY_TO_SEND) && (ethbuf.state != SENDING))
    return;
  // reset the response pointer, if we're just entering the SENDING state
  if (ethbuf.state != SENDING) {
    ethbuf.resp_pos = 0;
    ethbuf.state = SENDING; // transitory!
  }
  // we can be in two states - either we need to send, or we need to wait
  // for Ethernet FSM to advance until the data is sent
  if (! ethbuf.resp_pos) {
    // set the output data
    dprintf (" E> %u\n", ethbuf.resp_n); // >{transmitted}
    submit_eth_data(ethbuf.resp, ethbuf.resp_n);
    // run the Ethernet task
    eth_task();
    // mark the output pointer
    // note that we *can* send a zero-byte reply, so storing resp_n won't do
    ethbuf.resp_pos = ethbuf.resp_n | 1;
  }
  else {
    eth_task();
    if (! eth_sending_data()) {
      ethbuf.state = UNUSED;
      return;
    }
  }
}

// run the jtag task in a specific buffer
void run_jtag_task(buffer_t *buf)
{
  // if the buffer isn't READY_TO_EXECUTE state, nothing to do
  if (buf->state != READY_TO_EXECUTE) // == RECEIVED
    return;
  // do nothing unless the JTAG core's available
  if (! multicore_fifo_wready())
    return;
  buf->state = EXECUTING;
  dprintf ((buf == &usbbuf) ? " Ux\n" : " Ex\n");

  multicore_fifo_push_blocking((uint32_t)buf);
}

// run the jtag task in a specific buffer
void check_jtag_tasks()
{
  // do nothing unless the JTAG core finished a job
  while (multicore_fifo_rvalid()) {
    buffer_t *buf = (buffer_t*)multicore_fifo_pop_blocking();
    dprintf (" %c=%u\n", (buf==&usbbuf)?'U':'E', buf->resp_n); // >{transmitted}
    buf->resp_pos = 0;
    buf->state = EXECUTED; // == READY_TO_SEND
  }
}

// this is to work around the fact that tinyUSB does not handle setup request automatically
// Hence this boiler plate code
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
  if (stage != CONTROL_STAGE_SETUP)
    return true;
  return false;
}

