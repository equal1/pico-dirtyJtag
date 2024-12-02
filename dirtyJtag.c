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
#include "led.h"
#include "cmd.h"
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

unsigned iox_spi_speed;
int iox_check();
void iox_init()
{
  // configure CS#
  gpio_init(PIN_IOX_SSn);
  gpio_put(PIN_IOX_SSn, 1); // initially de-selected
  gpio_set_dir(PIN_IOX_SSn, GPIO_OUT);
  bi_decl(bi_1pin_with_name(PIN_IOX_SSn, "IOX_SS#"));
  // configure SPI itself - mode 0
  spi_init(SPI_IOX, FREQ_IOX_KHZ * 1000);
  spi_set_format (SPI_IOX, 8, 0, 0, 0);
  // configure the SPI pins
  gpio_set_function(PIN_IOX_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_IOX_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_IOX_MISO, GPIO_FUNC_SPI);
  bi_decl(bi_3pins_with_func(PIN_IOX_MISO, PIN_IOX_MOSI, PIN_IOX_SCK, GPIO_FUNC_SPI));
  iox_spi_speed = spi_get_baudrate(SPI_IOX);
  if (iox_check()) {
    //printf ("IOX does NOT work at %u.%uMHz!\n", (iox_spi_speed+500)/1000000, ((iox_spi_speed+500)%1000000)/1000);
    iox_spi_speed = 0;
  }
}

int adc_spi_speed, adc_addr;
int adc_probe();
void adc_init()
{
  // configure CS#
  gpio_init(PIN_ADC_SSn);
  gpio_put(PIN_ADC_SSn, 1); // initially de-selected
  gpio_set_dir(PIN_ADC_SSn, GPIO_OUT);
  bi_decl(bi_1pin_with_name(PIN_ADC_SSn, "ADC_SS#"));
  // configure SPI itself - mode 0
  spi_init(SPI_ADC, FREQ_ADC_KHZ * 1000);
  spi_set_format (SPI_ADC, 8, 0, 0, 0);
  // configure the SPI pins
  gpio_set_function(PIN_ADC_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_ADC_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_ADC_MISO, GPIO_FUNC_SPI);
  bi_decl(bi_3pins_with_func(PIN_ADC_MISO, PIN_ADC_MOSI, PIN_ADC_SCK, GPIO_FUNC_SPI));
  adc_spi_speed = spi_get_baudrate(SPI_ADC);
  adc_addr = -1;
  if ((adc_addr = adc_probe()) < 0) {
    //printf ("ADC does NOT work at %u.%uMHz!\n", (adc_spi_speed+500)/1000000, ((adc_spi_speed+500)%1000000)/1000);
    adc_spi_speed = 0;
  }
}

int a5_pico_pins_init();
int a5_iox_pins_init();

// in ethernet.c
int eth_spi_speed;
int eth_init(uint64_t);
// in usb.c
int usb_init(uint64_t);

static uint64_t board_id = 0;

static char *ipconfig_ptr;
static const char *hostname = "?", *macaddr = "?";

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

static const char *location_of(void *ptr)
{
  if ((uint32_t)ptr < 0x10000000)
    return "ROM";
  if ((uint32_t)ptr < 0x20000000)
    return "flash";
  if ((uint32_t)ptr < 0x30000000)
    return "RAM";
  return "???";
}

//-----------------------------------------------------------------------------

int main()
{
  board_init();
  led_init(PIN_LED);
  stdio_uart_init_full(uart1, 115200, 8, -1); // enable stdio over uart1 on pin_tx=gp8, no rx
  adc_init();
  iox_init();

  // get the Pico's serial ID
  pico_unique_board_id_t uID;
  pico_get_unique_board_id(&uID);
  board_id = __builtin_bswap64(*(uint64_t*)&uID);
  board_id = (board_id >> 8) | (board_id << 56);
  usb_init(board_id);
  eth_init(board_id);

  // configure the rest of the a5 pins
  a5_pico_pins_init();
  a5_iox_pins_init();
  unsigned sysclk_khz = (clock_get_hz(clk_sys) + 500)/ 1000;
  // initialize the header
  sprintf (whoami,
            "equal1 JTAG (%s, %s%s %s)\n",
            git_Branch, git_Describe, git_AnyUncommittedChanges?"|dirty":"",
             git_Remote);
  sprintf(whoami + strlen(whoami),
          "  running from %s on %s @%u.%uMHz, host board %s\n",
          location_of(main),
#         if PICO_RP2040
          eth_spi_speed ? "W5500_EVB_Pico" : "Raspberry_Pico",
#         elif PICO_RP2350
          eth_spi_speed ? "W5500_EVB_Pico2" : "Raspberry_Pico2",
#         else
          "???",
#         endif
          (sysclk_khz + 50)/1000, (sysclk_khz + 50)%1000/100, 
          (adc_spi_speed && iox_spi_speed) ? "DDv2+" :
          iox_spi_speed ? "DDv1" : "unknown");
  if (iox_spi_speed > 0)
    sprintf(whoami + strlen(whoami),
            "IOX: " IOX_NAME ", spi%c@%u.%03uMHz, SS#@GP%u\n",
            (SPI_IOX == spi0)? '0' : '1',
            (iox_spi_speed+500)/1000000, ((iox_spi_speed+500)%1000000)/1000,
            PIN_IOX_SSn);
  if (adc_addr >= 0)
    sprintf(whoami + strlen(whoami),
            "ADC: " ADC_NAME ", spi%c@%u.%03uMHz, SS#@GP%u; device address %u\n",
            (SPI_ADC == spi0)? '0' : '1',
            (adc_spi_speed+500)/1000000, ((adc_spi_speed+500)%1000000)/1000,
            PIN_ADC_SSn, adc_addr);
  sprintf(whoami + strlen(whoami),
             "USB serial: %016llX (label: %04X)\n",
            board_id, (uint32_t)board_id & 0xFFFF);
  if (eth_spi_speed > 0)
    sprintf(whoami + strlen(whoami),
            "Ethernet: " ETH_NAME ", spi%c@%u.%03uMHz, SS#@GP%u; MAC address %s, hostname %s\n",
            (SPI_ETH == spi0)? '0' : '1',
            (eth_spi_speed+500)/1000000, ((adc_spi_speed+500)%1000000)/1000,
            PIN_ETH_CSn, macaddr, hostname);
  ipconfig_ptr = whoami + strlen(whoami);

  // printf the header
  printf ("\n----\n%s----\n", whoami);
  void iox_debug();
  iox_debug();

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

