#include <stdint.h>
#include <stdio.h>
#include <pico/bootrom.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/multicore.h>
#include <pico/unique_id.h>
#include <hardware/pio.h>
#include <hardware/spi.h>
#include <hardware/clocks.h>
#include <bsp/board.h>
#include <tusb.h>
#include "pio_jtag.h"
#include "utils.h"
#include "cmd.h"
#include "git.h"
#include "a5pins.h"
#include "adc.h"
#include "ethernet.h"
#include "misc.h"

#include "config.h"

//#define DEBUG

#ifdef DEBUG
#define dprintf(...) printf(__VA_ARGS__)
#else
#define dprintf(...) (void)(__VA_ARGS__)
#endif

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
  printf("Rebooting to bootloader...\n");
  // this is fatal - wait 100ms then reboot to bootloader
  sleep_ms(100);
  reset_usb_boot(0, 0);
  while (1) asm volatile ("wfe");
}

void djtag_init()
{
  // declare and initialize the pins on the Pico
  jtag_pins_init();
  a5clk_pin_init();
  a5_pico_pins_init();
  // declare the Ethernet pins
  eth_pins_init();
  // initialize the JTAG and the A5 clock generator
  init_jtag(&jtag, 1000, PIN_TCK, PIN_TDI, PIN_TDO, PIN_TMS, PIN_RST);
  init_a5clk(&a5clk, 1000, PIN_A5_CLK);
  // set the state varibles used to sync ETH and ADC
  adc_busy = 0; eth_busy = 0; 
# ifdef SPI_PARANOIA
  adc_ss = 1;   eth_ss = 1;
# endif
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
buffer_t usbbuf, tcpbuf, udpbuf;

// receive tasks, usb and ethernet
// they return -1 on no data, bufidx on got data
static void usb_rx_task();
static void tcp_rx_task();
static void udp_rx_task();
// run the jtag task in a specific buffer
static void run_jtag_task(buffer_t*);
// check if any task completed
static void check_jtag_tasks();
// respond tasks, usb and ethernet
static void usb_tx_task();
static void tcp_tx_task();
static void udp_tx_task();

// core1 runs the jtag tasks
static void core1_entry();

static const char *location_of(void *ptr);

// initialize the whoami description
static uint64_t board_id;
static void whoami_init();

// in usb.c (here, 'cause we can't justify an usb.h just for this)
int usb_init(uint64_t);

//-----------------------------------------------------------------------------

int main()
{
  board_init(); 
  // configure the LED and the debug output
  led_init(PIN_LED);
  stdio_uart_init_full(UART_DBG, 115200, PIN_DBGTX, -1);
  // configure the signals directly connected to the Pico
  djtag_init();
  // get the Pico's serial ID
  pico_unique_board_id_t uID;
  pico_get_unique_board_id(&uID);
  board_id = __builtin_bswap64(*(uint64_t*)&uID);
  board_id = (board_id >> 8) | (board_id << 56);
  // configure USB and the Ethernet chip (if present)
  usb_init(board_id);
  eth_init(board_id);
  // configure the IO expander (if present) and the signals connected to it
  iox_init();
  if (iox_spi_speed > 0)
    a5_iox_pins_init();
  // configure the ADC (if present)
  adc_busy = 0; eth_busy = 0;
  adc_init();
  // generate the whoami description
  whoami_init();

  // printf the header
  printf("\n----\n%s----\n", whoami);
  // print the initial TILESEL/CLKSRC
  iox_debug();
  if (board_has_ethernet()) {
    // magic call that re-prints the last state (and updates whoami)
    notify_ip_config(-1,(const char*)-1, 0);
    // register the TCP debug service
    if (tcpsrv_init())
      notify_ip_config(-1,"failed to register the TCP debug service", 0);
    // register the UDP debug service
    if (udpsrv_init())
      notify_ip_config(-1,"failed to register the TCP debug service", 0);
    // register the firmware update (tftp) service
    if (fwupd_init())
      notify_ip_config(-1,"failed to register the firmware update service", 0);
  }

  multicore_launch_core1(core1_entry);
  while (1) {
    // see if we have anything incoming
    if (usbbuf.state < READY_TO_EXECUTE)
      usb_rx_task();
    if (tcpbuf.state < READY_TO_EXECUTE)
      tcp_rx_task();
    if (udpbuf.state < READY_TO_EXECUTE)
      udp_rx_task();
    // kick off received tasks
    if (usbbuf.state == READY_TO_EXECUTE) // usb_rx_task() might update this
      run_jtag_task(&usbbuf);
    if (tcpbuf.state == READY_TO_EXECUTE) // tcp_rx_task() might update this
      run_jtag_task(&tcpbuf);
    if (udpbuf.state == READY_TO_EXECUTE) // udp_rx_task() might update this
      run_jtag_task(&udpbuf);
    // advance either buffer's state if it advanced
    check_jtag_tasks();
    // send the response, if any 
    if (usbbuf.state > READY_TO_EXECUTE) // check_jtag_tasks() might update this
      usb_tx_task();
    if (tcpbuf.state > READY_TO_EXECUTE) // check_jtag_tasks() might update this
      tcp_tx_task();
    if (udpbuf.state > READY_TO_EXECUTE) // check_jtag_tasks() might update this
      udp_tx_task();
  }
}

// Core 1 runs the JTAG tasks:
// - receive ID of buffer to execute 
// - execute and update buffer
// - send ID of buffer that executed

void core1_entry() {
  while (1) {
    buffer_t *crtbuf = (buffer_t*)multicore_fifo_pop_blocking();
    assert (crtbuf->state == EXECUTING);
    crtbuf->resp_n = cmd_execute(&jtag, (crtbuf==&usbbuf)?'U':(crtbuf==&tcpbuf)?'T':'D', crtbuf->cmd, crtbuf->cmd_n, crtbuf->resp);
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
  dprintf(" U< %u(%u)\n", n, usbbuf.cmd_n); // <{received}({total_received})
  // if we got a max-size packet, stay in receive mode - more will follow
  if (n == 64)
    return;
  // if we got a fractional packet, we're done - ready to execute
  usbbuf.state = READY_TO_EXECUTE;
# ifdef DEBUG
  printf("usbbuf ready to execute (cmd_sz=%u)\n", usbbuf.cmd_n);
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
  dprintf(" U> %u(%u)\n", n, n0 - n); // >{transmitted}({left})
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

// ETH TCP receive task
void tcp_rx_task()
{
  // do nothing unless the eth buffer is either UNUSED or RECEIVING
  if ((tcpbuf.state != UNUSED) && (tcpbuf.state != RECEIVING))
    return;
  // reset the cmd pointer, if we're just entering the RECEIVING state
  if (tcpbuf.state != RECEIVING) {
    tcpbuf.cmd_n = 0;
    tcpbuf.state = RECEIVING; // transitory!
  }
  // run the Ethernet task
  eth_task();
  // grab data from Ethernet
  int n = tcpsrv_fetch(tcpbuf.cmd);
  if (n < 1)
    return;
  // set the read pointer
  tcpbuf.cmd_n = n;
  dprintf(" T< %u\n", n); // <{received}
  // we're done - ready to execute
  tcpbuf.state = READY_TO_EXECUTE;
# ifdef DEBUG
  printf("ethbuf ready to execute (cmd_sz=%u)\n", tcpbuf.cmd_n);
# endif
}

// ETH TCP transmit task
void tcp_tx_task()
{
  // for ethernet, a zero-length response still needs to be sent, per protocol
  // (which is purely ping-pong)
  // do nothing unless the eth buffer is READY_TO_SEND or SENDING
  if ((tcpbuf.state != READY_TO_SEND) && (tcpbuf.state != SENDING))
    return;
  // reset the response pointer, if we're just entering the SENDING state
  if (tcpbuf.state != SENDING) {
    tcpbuf.resp_pos = 0;
    tcpbuf.state = SENDING; // transitory!
  }
  // we can be in two states - either we need to send, or we need to wait
  // for Ethernet FSM to advance until the data is sent
  if (! tcpbuf.resp_pos) {
    // set the output data
    dprintf(" T> %u\n", tcpbuf.resp_n); // >{transmitted}
    tcpsrv_submit(tcpbuf.resp, tcpbuf.resp_n);
    // run the Ethernet task
    eth_task();
    // mark the output pointer
    // note that we *can* send a zero-byte reply, so storing resp_n won't do
    tcpbuf.resp_pos = tcpbuf.resp_n | 1;
  }
  else {
    eth_task();
    if (! is_tcpsrv_sending()) {
      tcpbuf.state = UNUSED;
      return;
    }
  }
}

// ETH UDP receive task
void udp_rx_task()
{
  // do nothing unless the eth buffer is either UNUSED or RECEIVING
  if ((udpbuf.state != UNUSED) && (udpbuf.state != RECEIVING))
    return;
  // reset the cmd pointer, if we're just entering the RECEIVING state
  if (udpbuf.state != RECEIVING) {
    udpbuf.cmd_n = 0;
    udpbuf.state = RECEIVING; // transitory!
  }
  // run the Ethernet task
  eth_task();
  // grab data from Ethernet
  int n = udpsrv_fetch(udpbuf.cmd);
  if (n < 1)
    return;
  // set the read pointer
  udpbuf.cmd_n = n;
  dprintf(" D< %u\n", n); // <{received}
  // we're done - ready to execute
  udpbuf.state = READY_TO_EXECUTE;
# ifdef DEBUG
  printf("udpbuf ready to execute (cmd_sz=%u)\n", udpbuf.cmd_n);
# endif
}

// ETH UDP transmit task
void udp_tx_task()
{
  // for ethernet, a zero-length response still needs to be sent, per protocol
  // (which is purely ping-pong)
  // do nothing unless the eth buffer is READY_TO_SEND or SENDING
  if ((udpbuf.state != READY_TO_SEND) && (udpbuf.state != SENDING))
    return;
  // reset the response pointer, if we're just entering the SENDING state
  if (udpbuf.state != SENDING) {
    udpbuf.resp_pos = 0;
    udpbuf.state = SENDING; // transitory!
  }
  // we can be in two states - either we need to send, or we need to wait
  // for Ethernet FSM to advance until the data is sent
  if (! udpbuf.resp_pos) {
    // set the output data
    dprintf(" D> %u\n", udpbuf.resp_n); // >{transmitted}
    udpsrv_submit(udpbuf.resp, udpbuf.resp_n);
    // run the Ethernet task
    eth_task();
    // mark the output pointer
    // note that we *can* send a zero-byte reply, so storing resp_n won't do
    udpbuf.resp_pos = udpbuf.resp_n | 1;
  }
  else {
    eth_task();
    if (! is_udpsrv_sending()) {
      udpbuf.state = UNUSED;
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
  dprintf((buf == &usbbuf) ? " Ux\n" : (buf == &tcpbuf) ? " Tx\n" : " Dx\n");

  multicore_fifo_push_blocking((uint32_t)buf);
}

// run the jtag task in a specific buffer
void check_jtag_tasks()
{
  // do nothing unless the JTAG core finished a job
  while (multicore_fifo_rvalid()) {
    buffer_t *buf = (buffer_t*)multicore_fifo_pop_blocking();
    dprintf(" %c=%u\n", (buf==&usbbuf)?'U':(buf==&tcpbuf)?'T':'D', buf->resp_n); // >{transmitted}
    buf->resp_pos = 0;
    buf->state = EXECUTED; // == READY_TO_SEND
  }
}

const char *location_of(void *ptr)
{
  if ((uint32_t)ptr < 0x10000000)
    return "ROM";
  if ((uint32_t)ptr < 0x20000000)
    return "flash";
  if ((uint32_t)ptr < 0x30000000)
    return "RAM";
  return "???";
}

void whoami_init()
{
  char *p = whoami;
  ipconfig_ptr = 0;
  unsigned sysclk_khz = (clock_get_hz(clk_sys) + 500)/ 1000;
  // print the version
  p += sprintf(p, "equal1 JTAG (%s, %s%s %s) caps=0x%08X\n",
               git_Branch, git_Describe, git_AnyUncommittedChanges?"(dirty)":"",
                git_Remote,
               IMPLEMENTED_CAPS);
  // print what we're running on
  p += sprintf(p, "  running from %s on %s (%s@%u.%uMHz), host board %s\n",
               location_of(main),
               eth_spi_speed ? WIZCHIP_BOARD_NAME : RASPBERRY_BOARD_NAME,
               CHIP_NAME,
               (sysclk_khz + 50)/1000, (sysclk_khz + 50)%1000/100, 
               (adc_spi_speed && iox_spi_speed) ? "DDv2+" :
               iox_spi_speed ? "DDv1" : "unknown");
  // print the USB serial
  p += sprintf(p, "USB serial: %016llX\n", board_id);
  // print the IO expander config
  int f = (iox_spi_speed + 500) / 1000;
  if (f > 0)
    p += sprintf(p, "IOX: " IOX_NAME ", spi%c@%u.%03uMHz, SS#@GP%u\n",
                 (SPI_IOX == spi0)?'0':'1', f / 1000, f % 1000, PIN_IOX_SSn);
  // print the ADC config
  f = (adc_spi_speed + 500) / 1000;
  if (f > 0)
    p += sprintf(p, "ADC: " ADC_NAME ", spi%c@%u.%03uMHz, SS#@GP%u; device address %c\n",
                 (SPI_ADC == spi0)?'0':'1', f / 1000, f % 1000, PIN_ADC_SSn,
                 '0'+adc_addr);
  // print the Ethernet config
  if (board_has_ethernet ()) {
    p += sprintf(p, "Ethernet: %s, spi%c@%s, SS#@GP%u; MAC address %s, hostname %s\n",
                 ethstr(ETHSTR_CHIP), (SPI_ETH == spi0)?'0':'1', ethstr(ETHSTR_SPIFREQ), PIN_ETH_CSn,
                 ethstr(ETHSTR_MAC), ethstr(ETHSTR_HOSTNAME));
    ipconfig_ptr = p;
  }
}
