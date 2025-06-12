#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <hardware/dma.h>
#include <wizchip_conf.h>
#include <w5500.h>
#include <socket.h>
#include <dhcp.h>

#include "config.h"
#include "ethernet.h"
#include "pico-w5500/Ethernet/wizchip_conf.h"
#include "utils.h"

// enable for some paranoid checks throughout the code
//#define PARANOID

//============================================================================

// Ethernet state

enum eth_state {
  ETH_NOT_INIT = -3,
  ETH_NOT_PRESENT,
  ETH_ERROR,
  ETH_NO_LINK = 0,
  ETH_WAIT_FOR_IP,
  ETH_HAVE_IP
};

struct eth_state_s {
  // state
  short state; // enum eth_state; don't use char, since that's unsigned (!!)
  short have_link; // -1 for error, or bool
  char have_ethernet; // bool
  char dhcp_running; //bool
  // timers
  char dhcp_timer_running, error_timer_running; // bool'ss
  repeating_timer_t dhcp_timer, error_timer;
  // connected chip
  unsigned spi_freq;
  // network configuration
  wiz_NetInfo net;
  struct {
    char chip_name[8]; //  (6) "W????"
    char spifreq[12];  //  (9) "?.???MHz" / "???KHz"
    char hostname[24]; // (23) "pico?-????????????????"
    char mac[20];      // (18) "??:??:??:??:??:??"
    char ip[16];       // (16) "???.???.???.???"
    char gateway[16];
    char subnet[16];
    char dns[16];
    char network[20];  // (20) "???.???.???.???/???"
    char error[20];
    // up to 1 tcp client at any time
    char tcpcli[24];   // (22) "???.???.???.???:?????"
  } txt;
  // sockets
  uint16_t used_socket_map;
  uint16_t running_socket_map;
  struct {
    link_callback_t on_link_up, on_link_down;
    socket_callback_t process;
    // service-specific structure
    struct dbgsvc_s *svc;
  } socket_callback[8];
};

static struct eth_state_s eth = {
  .state = ETH_NOT_INIT, // not initialized at all
  .txt = {
    .chip_name = "???",
    .spifreq = "?.???MHz",
    .hostname = "???",
    .mac = "??:??:??:??:??:??",
    .ip = "\0",
    .gateway = "\0",
    .subnet = "\0",
    .dns = "\0",
    .network = "\0",
    .error = "\0",
    .tcpcli = "\0"
  }
};

//============================================================================

// W5500 chip operations

volatile char eth_busy;
#ifdef SPI_PARANOIA
volatile char eth_ss;
#endif
int eth_spi_speed;

static int eth_set_spi_speed(unsigned speed);

// reset the chip
// return 1 if the chip was detected
static int w5500_reset();

// load the current network settings into the chip
static int w5500_update();

// assert CS#
void w5500_select();
// de-assert CS#
void w5500_deselect();

// read single byte
uint8_t w5500_read8();
// write single byte
void w5500_write8(uint8_t tx_data);

#ifdef W5500_USE_BLOCK
// read byte sequence
void w5500_block_read(uint8_t *pBuf, uint16_t len);
// write byte sequence
void w5500_block_write(uint8_t *pBuf, uint16_t len);

#ifdef W5500_USE_BLOCK_DMA
// initialize the DMA channels for the block read/write
void w5500_dma_init();
#endif
#endif

static void set_eth_client(const uint8_t *ip, uint16_t port);

//----------------------------------------------------------------------------

// IRQs don't seem to help if the code is written in the main-loop fashion

#if 0
void w5500_on_irq(uint gpio, uint32_t events);

typedef void (*socket_handler_t)(unsigned, unsigned);
static void w5500_config_socket_irq(unsigned socket, socket_handler_t handler);
#endif

//----------------------------------------------------------------------------

// DHCP event handlers

static void on_dhcp_assign();
static void on_dhcp_update();
static void on_dhcp_conflict();

//----------------------------------------------------------------------------

// update the text descriptions of the connection parameters; also, load those
// parameters into the chip
static void eth_config_update();

// stop all running Ethernet services
void eth_stop_services();
// start all running Ethernet services
void eth_start_services();
// restart all running Ethernet services
void eth_restart_services();

// error timer callback
bool err_every_100ms(repeating_timer_t *rt);

//============================================================================

void eth_pins_init()
{
  bi_decl(bi_1pin_with_name(PIN_ETH_CSn, "W5500_CS#"));
  bi_decl(bi_1pin_with_name(PIN_ETH_RSTn, "W5500_RST#"));
  bi_decl(bi_1pin_with_name(PIN_ETH_INTn, "W5500_INT#"));
  bi_decl(bi_3pins_with_func(PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_SCK,
                             GPIO_FUNC_SPI));
  eth.state = ETH_NOT_INIT;
}

//----------------------------------------------------------------------------

static void _assert_eth_cs(const char*);
static void _deassert_eth_cs(const char*);
static void _check_eth_state(const char*);
#define assert_eth_cs() _assert_eth_cs(__FUNCTION__)
#define deassert_eth_cs() _deassert_eth_cs(__FUNCTION__)
#define check_eth_state() _check_eth_state(__FUNCTION__)

// all of these update the state, so if they're used for what they do,
//  the state needs to be changed immediately afterwards
static void eth_lost_link(void);
static void eth_got_link(void);
static void eth_got_error(const char *msg);

int eth_init(uint64_t board_id)
{
  // don't reinit if chip isn't present
  if (eth.state >= ETH_NO_LINK )
    return 0;
  
  // derive a MAC address from the unique board ID
  uint64_t macaddr = (board_id & 0xf0ffffffffff) | 0x020000000000;
  eth.net.mac[0] = (uint8_t)((macaddr >> 40) & 0xFF);
  eth.net.mac[1] = (uint8_t)((macaddr >> 32) & 0xFF);
  eth.net.mac[2] = (uint8_t)((macaddr >> 24) & 0xFF);
  eth.net.mac[3] = (uint8_t)((macaddr >> 16) & 0xFF);
  eth.net.mac[4] = (uint8_t)((macaddr >> 8 ) & 0xFF);
  eth.net.mac[5] = (uint8_t)((macaddr >> 0 ) & 0xFF);

# ifdef W5500_USE_BLOCK_DMA
  // initialize the DMA config
  w5500_dma_init();
# endif

  // initialize the pins and the core
  // - configure CS#
  gpio_init(PIN_ETH_CSn);
  gpio_put(PIN_ETH_CSn, 1); // initially de-selected
  gpio_set_dir(PIN_ETH_CSn, GPIO_OUT);
  deassert_eth_cs();
  // - configure RST#
  gpio_init(PIN_ETH_RSTn);
  gpio_put(PIN_ETH_RSTn, 0); // initially, in reset
  gpio_set_dir(PIN_ETH_RSTn, GPIO_OUT);
  gpio_put(PIN_ETH_RSTn, 0);
  // - configure INT#
  gpio_init(PIN_ETH_INTn);
  gpio_set_dir(PIN_ETH_INTn, GPIO_IN);
# if 0
  // don't attempt to use the IRQ line
  gpio_set_irq_enabled_with_callback(PIN_ETH_INTn, GPIO_IRQ_EDGE_FALL, true,
                                     w5500_on_irq);
# endif
  // - configure SPI itself - mode 0
  spi_init(SPI_ETH, FREQ_ETH_KHZ * 1000);
  spi_set_format (SPI_ETH, 8, 0, 0, 0);
  // - configure the SPI pins
  gpio_set_function(PIN_ETH_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_ETH_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_ETH_MISO, GPIO_FUNC_SPI);

  // set the WizChip callbacks
  reg_wizchip_cs_cbfunc(w5500_select, w5500_deselect);
  reg_wizchip_spi_cbfunc(w5500_read8, w5500_write8);
# ifdef W5500_USE_BLOCK
  // this seems to degrade performance, regardless of whether we use DMAs or not
  reg_wizchip_spiburst_cbfunc(w5500_block_read, w5500_block_write);
# endif

  // set the DHCP callbacks
  reg_dhcp_cbfunc(on_dhcp_assign, on_dhcp_update, on_dhcp_conflict);

  // detect/configure the SPI speed
  // try to find the maximum frequency at which the chip would talk to us
  eth.spi_freq = FREQ_ETH_KHZ * 1000;
  while (! eth_set_spi_speed(eth.spi_freq)) {
    // if we're >= 10MHz, decrease by 1MHz
    if (eth.spi_freq >= 10000000)
      eth.spi_freq -= 1000000;
    // if we're >= 1MHz, decrease by 0.5MHz
    else if (eth.spi_freq >= 1000000)
      eth.spi_freq -= 500000;
    // if we can't connect even at 1MHz, give up
    else {
      eth.spi_freq = 0;
      break;
    }
  }
  eth_spi_speed = eth.spi_freq; // replicate here - used by claim_spi_for_eth()
  if (! eth.spi_freq) {
    // register the error
    notify_ip_config(-1, "W5500 not found", 0);
    eth.state = ETH_NOT_PRESENT;
    return 0;
  }
  // here: we're good, we do have an Ethernet chip

  // format the hostname for the DHCP client
  // (which appends the last 3 bytes from the pico ID)
  sprintf(HOST_NAME, BOARD_NAME "-%010llX", board_id >> 24);

  // grab the chip name
  ctlwizchip(CW_GET_ID, eth.txt.chip_name);
  // format the MAC address
  sprintf(eth.txt.mac, "%02X:%02X:%02X:%02X:%02X:%02X",
            eth.net.mac[0], eth.net.mac[1], eth.net.mac[2],
            eth.net.mac[3], eth.net.mac[4], eth.net.mac[5]);
  // format the hostname
  sprintf(eth.txt.hostname, BOARD_NAME"-%016llX", board_id);
  // format the SPI frequency
  if (eth.spi_freq >= 1000000)
    sprintf(eth.txt.spifreq, "%u.%03uMHz",
             (eth.spi_freq + 500) / 1000000, 
             ((eth.spi_freq + 500) % 1000000) / 1000);
  else
    sprintf(eth.txt.spifreq, "%uKHz",
             (eth.spi_freq + 500) / 1000);

  eth.have_link = 0;
  eth_lost_link();
  // check if the link is actually off
  ctlwizchip(CW_GET_PHYLINK, &eth.have_link);
  // if we just saw the link go up, say so
  notify_ip_config(eth.have_link, 0, 0);

  return 0;
}

//----------------------------------------------------------------------------

int board_has_ethernet()
{
  if (eth.state == ETH_NOT_PRESENT)
    return 0;
  if (eth.state == ETH_NOT_INIT)
    return -1;
  return 1;
}

int is_ethernet_connected()
{
  return eth.state == ETH_HAVE_IP;
}

int have_tcp_client_connected()
{
  return eth.txt.tcpcli[0];
}

void set_eth_client(const uint8_t *ip, uint16_t port)
{
  // clear client if !IP || !port
  if ((! ip) || (! port) || (! *(uint32_t*)ip)) {
    eth.txt.tcpcli[0] = 0;
    notify_ip_config(eth.have_link, eth.txt.ip, 0);
  }
  else {
    sprintf(eth.txt.tcpcli, "%u.%u.%u.%u:%u",
            ip[0], ip[1], ip[2], ip[3], port);
    notify_ip_config(eth.have_link, eth.txt.ip, eth.txt.tcpcli);
  }
}

const char *ethstr(int s)
{
  const char *str;
  switch (s) {
  case ETHSTR_CHIP:
    str = eth.txt.chip_name; break;
  case ETHSTR_SPIFREQ:
    str = eth.txt.spifreq; break;
  case ETHSTR_MAC:
    str = eth.txt.mac; break;
  case ETHSTR_HOSTNAME:
    str = eth.txt.hostname; break;
  case ETHSTR_IP:
    str = eth.txt.ip; break;
  case ETHSTR_NETWORK:
    str = eth.txt.network; break;
  case ETHSTR_GATEWAY:
    str = eth.txt.gateway; break;
  case ETHSTR_SUBNET:
    str = eth.txt.subnet; break;
  case ETHSTR_DNS:
    str = eth.txt.dns; break;
  case ETHSTR_TCPCLI:
    str = eth.txt.tcpcli; break;
  case ETHSTR_ERROR:
    str = eth.txt.error; break;
  default:
    str = 0;
  }
  return str[0] ? str : 0;
}

//============================================================================

void eth_task()
{
  // do nothing before initialization, or on non-ethernet boards
  if (eth.state < ETH_ERROR)
    return;

  // check the link state
  int had_no_link = ! eth.have_link;
  ctlwizchip(CW_GET_PHYLINK, &eth.have_link);
  // if we just saw the link go up, say so
  if (had_no_link && eth.have_link)
    notify_ip_config(1, 0, 0);

  // if there's no link
  if (! eth.have_link) {
    // if it's the same as last time, we have nothing to do
    if (eth.state == ETH_NO_LINK)
      return;
    // otherwise, handle the situation
    eth_lost_link();
    // reset the Ethernet chip, just in case
    w5500_reset();
    eth.state == ETH_NO_LINK;
    return;
  }
  // here: the link is up

  // if the link just went up, start the DHCP server
  // note, this changes the state to WAIT_FOR_IP
  if (eth.state == ETH_NO_LINK)
    eth_got_link();

  // if we're waiting for an IP address, don't budge until/unless we either
  //  fail, or we get an IP address
  if (eth.state == ETH_WAIT_FOR_IP) {
    switch (DHCP_run()) {
    // once we got an address, keep going
    // note, the state is changed by the relevant callback
    case DHCP_IP_ASSIGN:
    case DHCP_IP_CHANGED:
    case DHCP_IP_LEASED:
      break;
    // if we failed, enter error mode
    // note, this almost certainly means a DHCP timeout; an IP conflict already
    //  changed state to error
    case DHCP_FAILED:
      printf("eth: DHCP timeout");
      eth_got_error("DHCP timeout");
      return;
    // ignore any other state
    default:
      return;
    }
  }

  // execute all registered services, running on sockets that don't have a
  // transient state
  if (eth.state == ETH_HAVE_IP) {
    unsigned todo = eth.running_socket_map;
    unsigned socket = 0;
    while (todo) {
      if (todo & 1) {
        // if the socket is in a transient state, do nothing this time
        unsigned sstate = getSn_SR(socket);
        if (sstate == SOCK_SYNSENT)   goto next_socket;
        if (sstate == SOCK_SYNRECV)   goto next_socket;
        if (sstate == SOCK_FIN_WAIT)  goto next_socket;
        if (sstate == SOCK_CLOSING)   goto next_socket;
        if (sstate == SOCK_TIME_WAIT) goto next_socket;
        if (sstate == SOCK_LAST_ACK)  goto next_socket;
        // if the socket is in CLOSE_WAITING state, close it
        if (sstate == SOCK_CLOSE_WAIT) {
          //printf("eth: socket %u in CLOSE_WAIT - closing\n", socket);
          close(socket);
          goto next_socket;
        }
        // call the service process() callback
        if (eth.socket_callback[socket].process)
          eth.socket_callback[socket].process(socket, sstate, eth.socket_callback[socket].svc);
      }
    next_socket:
      ++socket;
      todo >>= 1;
    }
  }
}

//----------------------------------------------------------------------------

void eth_lost_link()
{
  // stop any running services
  eth_stop_services();

  // stop the DHCP client (including its timer)
  if (eth.dhcp_timer_running) {
    cancel_repeating_timer(&eth.dhcp_timer);
    eth.dhcp_timer_running = 0;
  }
  if (eth.dhcp_running) {
    DHCP_stop();
    eth.dhcp_running = 0;
  }
  // stop the error timer (if it was running)
  if (eth.error_timer_running) {
    cancel_repeating_timer(&eth.error_timer);
    eth.error_timer_running = 0;
  }

  // reset the DHCP-provided settings and update chip and the descriptions
  *(uint32_t*)eth.net.ip = 0;  // IP address
  *(uint32_t*)eth.net.gw = 0;  // gatweay IP address
  *(uint32_t*)eth.net.sn = 0;  // subnet mask
  *(uint32_t*)eth.net.dns = 0; // DNS server
  eth.net.dhcp = NETINFO_DHCP;
  eth_config_update();

  // turn off the LED
  set_led(LED_ETHERNET, 0);

  // change state to NO_LINK
  eth.state = ETH_NO_LINK;
}

//----------------------------------------------------------------------------

static uint8_t dhcp_buf[548];

static bool dhcp_every_1s(repeating_timer_t*);

void eth_got_link()
{
  // start the DHCP client (including its timer)
  if (! eth.dhcp_running) {
    w5500_update(); // seems required, to set the SHAR as it was?
    DHCP_init(SOCKET_DHCP, dhcp_buf);
    eth.dhcp_running = 1;
  }
  if (! eth.dhcp_timer_running) {
    eth.dhcp_timer_running = 1;
    add_repeating_timer_ms (1000, dhcp_every_1s, 0, &eth.dhcp_timer);
  }
  eth.state = ETH_WAIT_FOR_IP;
}

//----------------------------------------------------------------------------

static void eth_got_error(const char *msg)
{
  eth.have_link = -1;
  // act as if we had lost the link
  eth_lost_link();
  // record the error
  strcpy(eth.txt.error, msg);
  // start flashing the LED
  if (! eth.error_timer_running) {
    eth.error_timer_running = 1;
    add_repeating_timer_ms (100, err_every_100ms, 0, &eth.error_timer);
  }
  eth.state = ETH_ERROR;
}

///============================================================================

// register a service on a socket
int eth_register_service(int socket, struct dbgsvc_s *svc,
                         link_callback_t on_link_up, 
                         link_callback_t on_link_down,
                         socket_callback_t process)
{
  // return error, if the service is already registered
  if (eth.used_socket_map & (1 << socket))
    return -1;
  // register the 3 callbacks
  eth.socket_callback[socket].on_link_up   = on_link_up;
  eth.socket_callback[socket].on_link_down = on_link_down;
  eth.socket_callback[socket].process      = process;
  eth.socket_callback[socket].svc          = svc;
  eth.used_socket_map |= 1 << socket;
  // patch in the tcp_set_cli pointer in the service structure
  // (the target is static in this file)
  svc->tcp_set_cli = set_eth_client;
  // if the state of ethernet is up, with IP, start the service
  // (if not, the init fn will be called once the link is up and
  //  the IP, assigned)
  if ((eth.state == ETH_HAVE_IP) && on_link_up) {
    eth.socket_callback[socket].on_link_up(socket, eth.socket_callback[socket].svc);
    eth.running_socket_map |= 1 << socket;
  }
  // return success
  return 0;
}

//----------------------------------------------------------------------------

// stop all running Ethernet services
void eth_stop_services()
{
  unsigned todo = eth.running_socket_map;
  unsigned socket = 0;
  while (todo) {
    if ((todo & 1) && eth.socket_callback[socket].on_link_down) {
      eth.socket_callback[socket].on_link_down(socket, eth.socket_callback[socket].svc);
      eth.running_socket_map &= ~(1 << socket);
    }
    ++socket;
    todo >>= 1;
  }
}

//----------------------------------------------------------------------------

// start all Ethernet services that aren't running already
void eth_start_services()
{
  unsigned todo = eth.used_socket_map & ~eth.running_socket_map;
  unsigned socket = 0;
  while (todo) {
    if ((todo & 1) && eth.socket_callback[socket].on_link_up) {
      eth.socket_callback[socket].on_link_up(socket, eth.socket_callback[socket].svc);
      eth.running_socket_map |= 1 << socket;
    }
    ++socket;
    todo >>= 1;
  }
}

//----------------------------------------------------------------------------

// restart all running Ethernet services
void eth_restart_services()
{
  eth_stop_services();
  eth_start_services();
}

//============================================================================

// timer that runs while DHCP is running
bool dhcp_every_1s(repeating_timer_t *rt)
{
  (void)rt;
  if (eth.dhcp_running) {
    DHCP_time_handler();
    // toggle the LED every second while waiting for DHCP
    if (eth.state < ETH_HAVE_IP)
      toggle_led(LED_ETHERNET);
  }
  return true;
}

//----------------------------------------------------------------------------

// DHCP address assigned
void on_dhcp_assign()
{
  // grab the IP address, the gateway and the subnet mask
  getIPfromDHCP(eth.net.ip);
  getGWfromDHCP(eth.net.gw);
  getSNfromDHCP(eth.net.sn);
  getDNSfromDHCP(eth.net.dns);
  eth.net.dhcp = NETINFO_DHCP;
  // update the chip and the text things
  eth_config_update();
  // print what just happened
# if 0
  printf("dhcp: got IP %s; gateway %s, subnet %s (network: %s); DNS server: %s\n",
          eth.txt.ip, eth.txt.gateway, eth.txt.subnet, eth.txt.network, eth.txt.dns);
# else
  // don't print both the newtork and the gateway
  printf("dhcp: got IP %s; gateway %s, network: %s, DNS server: %s\n",
          eth.txt.ip, eth.txt.gateway, eth.txt.network, eth.txt.dns);
# endif
  // switch state to GOT_IP
  eth.state = ETH_HAVE_IP;
  // start any registered services
  eth_start_services();
}

//----------------------------------------------------------------------------

// DHCP address changed
void on_dhcp_update(void)
{
  printf("eth/dhcp: address changed; restarting network...\n");
  // stop any running services
  eth_stop_services();
  // do the same as if an IP address had just been assigned
  on_dhcp_assign();
}

//----------------------------------------------------------------------------

// DHCP IP conflict
void on_dhcp_conflict(void)
{
  printf("eth/dhcp: IP conflict");
  eth_got_error("DHCP conflict");
}

//============================================================================

// timer that runs while we have an ethernet error
// (specifically, DHCP timeout or DHCP conflict)
bool err_every_100ms(repeating_timer_t *rt)
{
  (void)rt;
  if (eth.state == ETH_ERROR)
    toggle_led(LED_ETHERNET);
  return true;
}

//============================================================================

// update everything possible in the eth structure and in the chip
void eth_config_update()
{
  // load the settings into the W5500
  w5500_update();

  // format the DHCP settings as text
  if (! *(uint32_t*)eth.net.ip) {
    strcpy (eth.txt.ip, "?.?.?.?");
    strcpy (eth.txt.gateway, "?.?.?.?");
    strcpy (eth.txt.subnet, "?.?.?.?");
    strcpy (eth.txt.dns, "?.?.?.?");
    strcpy (eth.txt.network, "?.?.?.?/?");
    notify_ip_config(eth.have_link, 0, 0);
  }
  else {
    sprintf(eth.txt.ip, "%u.%u.%u.%u",
             eth.net.ip[0], eth.net.ip[1], eth.net.ip[2], eth.net.ip[3]);
    sprintf(eth.txt.gateway, "%u.%u.%u.%u", 
             eth.net.gw[0], eth.net.gw[1], eth.net.gw[2], eth.net.gw[3]);
    sprintf(eth.txt.subnet, "%u.%u.%u.%u",
             eth.net.sn[0], eth.net.sn[1], eth.net.sn[2], eth.net.sn[3]);
    sprintf(eth.txt.dns, "%u.%u.%u.%u",
             eth.net.dns[0], eth.net.dns[1], eth.net.dns[2], eth.net.dns[3]);
    // compute the eth.net address
    unsigned netaddr_bits;
    uint32_t network = ((uint32_t)eth.net.gw[0] << 24) | ((uint32_t)eth.net.gw[1] << 16) | 
                       ((uint32_t)eth.net.gw[2] <<  8) | ((uint32_t)eth.net.gw[3] <<  0);
    uint32_t subnet = ((uint32_t)eth.net.sn[0] << 24) | ((uint32_t)eth.net.sn[1] << 16) | 
                      ((uint32_t)eth.net.sn[2] <<  8) | ((uint32_t)eth.net.sn[3] <<  0);
    netaddr_bits = 0;
    while (subnet & (1 << 31)) {
      ++netaddr_bits;
      subnet <<= 1;
    }
    network &= ~((1 << (32 - netaddr_bits)) - 1);
    sprintf(eth.txt.network, "%u.%u.%u.%u/%u",
             (network >> 24) & 0xFF, (network >> 16)& 0xFF, 
             (network >>  8) & 0xFF, (network >>  0)& 0xFF,
             netaddr_bits);
    notify_ip_config(eth.have_link, eth.txt.ip, 0);
  }
}

//============================================================================

// socket buffer sizes, TX respectively RX
// alloc 4k for the debug ports, 2k for the firmware update one,
// 2k for the reminder (currently unused)
static const uint8_t w5500_buf_sizes[2][8] = {
    { 4, 4, 2, 1, 1, 1, 1, 1 }, 
    { 4, 4, 2, 1, 1, 1, 1, 1 }
};

// reset/initialize the Ethernet chip
int w5500_reset()
{
  // pulse reset
  gpio_put(PIN_ETH_RSTn, 0); sleep_ms(2);
  gpio_put(PIN_ETH_RSTn, 1); sleep_ms(2);
  // initialize the chip
  ctlwizchip (CW_RESET_WIZCHIP, 0); 
  sleep_ms(2);
  ctlwizchip (CW_INIT_WIZCHIP, (void*)w5500_buf_sizes);
  sleep_ms(2);
  ctlwizchip (CW_RESET_PHY, 0);
  sleep_ms(2);
  // sanity-check the chip
  eth.have_ethernet = (getVERSIONR() == 0x04) ? 1 : 0;
  return eth.have_ethernet;
}

int eth_set_spi_speed(unsigned speed) {
# if 0
  printf("W5500: attempting SPI at %u.%03uMHz...\n", 
          (eth_spi_speed+500)/1000000, ((eth_spi_speed+500)%1000000/1000) );
# endif
  // re-configure the SPI speed
  spi_set_baudrate(SPI_ETH, speed);
  eth_spi_speed = spi_get_baudrate(SPI_ETH);
  // re-reset the chip
  return w5500_reset();
}

//----------------------------------------------------------------------------

// load the network settings into the chip
int w5500_update()
{
  ctlnetwork(CN_SET_NETINFO, &eth.net);
}

//----------------------------------------------------------------------------

void _assert_eth_cs(const char *fn)
{
  (void)fn;
# ifdef SPI_PARANOIA
  if (! eth_ss)
    printf("P: in %s(): ETH CS# already asserted!!!\n", fn);
  else {
    if (! eth_busy)
      printf("P: in %s(): ETH CS# asserting without eth_busy!!!\n", fn);
    if (! adc_busy)
      printf("P: in %s(): ETH CS# asserting while adc_busy!!!\n", fn);
    if (! adc_ss)
      printf("P: in %s(): asserting ETH CS# while the ADC CS# is asserted!!!\n", fn);
  }
# endif
  gpio_put(PIN_ETH_CSn, 0);
# ifdef SPI_PARANOIA
  eth_ss = 0;
# endif
}

void _deassert_eth_cs(const char *fn)
{
  (void)fn;
# ifdef SPI_PARANOIA
  if (eth_ss)
    printf("P: in %s(): ETH CS# not yet asserted!!!\n", fn);
# endif
  gpio_put(PIN_ETH_CSn, 1);
# ifdef SPI_PARANOIA
  eth_ss = 1;
# endif
}

void _check_eth_state(const char *fn)
{
  (void)fn;
# ifdef PARANOID
  if (! eth_busy)
    printf("P: %s() called while !eth_busy\n", fn);
  if (adc_busy)
    printf("P: %s() called while adc_busy\n", fn);
  if (eth_ss)
    printf("P: %s() called while W5500 CS# not yet active\n", fn);
  if (! adc_ss)
    printf("P: %s() called while ADC CS# is already active\n", fn);
# endif
}

void w5500_select()
{
  claim_spi_for_eth();
  assert_eth_cs();
}

void w5500_deselect()
{
  deassert_eth_cs();
  release_eth_spi();
}

//----------------------------------------------------------------------------

uint8_t w5500_read8()
{
  uint8_t rx_data = 0;
  uint8_t tx_data = 0xFF;
  check_eth_state();
  spi_read_blocking(SPI_ETH, tx_data, &rx_data, 1);
  return rx_data;
}

void w5500_write8(uint8_t tx_data)
{
  check_eth_state();
  spi_write_blocking(SPI_ETH, &tx_data, 1);
}

//----------------------------------------------------------------------------

# ifdef W5500_USE_BLOCK

# ifdef W5500_USE_BLOCK_DMA
static uint w5500_dma_tx, w5500_dma_rx;
static dma_channel_config w5500_dma_tx_cfg, w5500_dma_rx_cfg;

void w5500_dma_init()
{
  w5500_dma_tx = dma_claim_unused_channel(true);
  w5500_dma_tx_cfg = dma_channel_get_default_config(w5500_dma_tx);
  channel_config_set_transfer_data_size(&w5500_dma_tx_cfg, DMA_SIZE_8);
  channel_config_set_dreq(&w5500_dma_tx_cfg, spi_get_dreq(SPI_ETH, true));
  channel_config_set_write_increment(&w5500_dma_tx_cfg, false);
  w5500_dma_rx = dma_claim_unused_channel(true);
  w5500_dma_rx_cfg = dma_channel_get_default_config(w5500_dma_rx);
  channel_config_set_transfer_data_size(&w5500_dma_rx_cfg, DMA_SIZE_8);
  channel_config_set_dreq(&w5500_dma_rx_cfg, spi_get_dreq(SPI_ETH, false));
  channel_config_set_read_increment(&w5500_dma_rx_cfg, false);
}
# endif

//----------------------------------------------------------------------------

void w5500_block_read(uint8_t *pBuf, uint16_t len)
{
  check_eth_state();
# ifndef W5500_USE_BLOCK_DMA
  spi_read_blocking(SPI_ETH, 0xFF, pBuf, len);
# else
  // use spi_read_blocking() for small blocks
  if (len <= 16) {
    spi_read_blocking(SPI_ETH, 0xFF, pBuf, len);
    return;
  }
  // otherwise, use the DMA
  static const uint8_t dummy_data = 0xFF;
  channel_config_set_read_increment(&w5500_dma_tx_cfg, false);
  dma_channel_configure(w5500_dma_tx, &w5500_dma_tx_cfg,
                        &spi_get_hw(SPI_ETH)->dr, // write address
                        &dummy_data,                // read address
                        len,                        // element count (each element is of size transfer_data_size)
                        false);                     // don't start yet
  channel_config_set_write_increment(&w5500_dma_rx_cfg, true);
  dma_channel_configure(w5500_dma_rx, &w5500_dma_rx_cfg,
                        pBuf,                       // write address
                        &spi_get_hw(SPI_ETH)->dr, // read address
                        len,                        // element count (each element is of size transfer_data_size)
                        false);                     // don't start yet
  dma_start_channel_mask((1u << w5500_dma_tx) | (1u << w5500_dma_rx));
  dma_channel_wait_for_finish_blocking(w5500_dma_rx);
# endif
}

//----------------------------------------------------------------------------

void w5500_block_write(uint8_t *pBuf, uint16_t len)
{
  check_eth_state();
# ifndef W5500_USE_BLOCK_DMA
  spi_write_blocking(SPI_ETH, pBuf, len);
# else
  // use spi_write_blocking() for small blocks
  if (len <= 16) {
    spi_write_blocking(SPI_ETH, pBuf, len);
    return;
  }
  // otherwise, use the DMA
  static uint8_t dummy_data;
  channel_config_set_read_increment(&w5500_dma_tx_cfg, true);
  dma_channel_configure(w5500_dma_tx, &w5500_dma_tx_cfg,
                        &spi_get_hw(SPI_ETH)->dr, // write address
                        pBuf,                       // read address
                        len,                        // element count (each element is of size transfer_data_size)
                        false);                     // don't start yet
  channel_config_set_write_increment(&w5500_dma_rx_cfg, false);
  dma_channel_configure(w5500_dma_rx, &w5500_dma_rx_cfg,
                        &dummy_data,                // write address
                        &spi_get_hw(SPI_ETH)->dr, // read address
                        len,                        // element count (each element is of size transfer_data_size)
                        false);                     // don't start yet
  dma_start_channel_mask((1u << w5500_dma_tx) | (1u << w5500_dma_rx));
  dma_channel_wait_for_finish_blocking(w5500_dma_rx);
# endif
}

#endif // # ifdef W5500_USE_BLOCK

//============================================================================

const char *ssstr(int s)
{
  static char sstxt[8];
  switch (s) {
  case SOCK_CLOSED: return "SOCK_CLOSED";
  case SOCK_INIT: return "SOCK_INIT";
  case SOCK_LISTEN: return "SOCK_LISTEN";
  case SOCK_SYNSENT: return "SOCK_SYNSENT";
  case SOCK_SYNRECV: return "SOCK_SYNRECV";
  case SOCK_ESTABLISHED: return "SOCK_ESTABLISHED";
  case SOCK_FIN_WAIT: return "SOCK_FIN_WAIT";
  case SOCK_CLOSING: return "SOCK_CLOSING";
  case SOCK_TIME_WAIT: return "SOCK_TIME_WAIT";
  case SOCK_CLOSE_WAIT: return "SOCK_CLOSE_WAIT";
  case SOCK_LAST_ACK: return "SOCK_LAST_ACK";
  default: sprintf(sstxt,"0x%02X", s); return sstxt;
  }
}

//============================================================================

// W5500 IRQ logic - neither useful nor needed

#if 0

static uint16_t socket_irq_enabled = 0;
static uint8_t socket_intmask[8], socket_interrupt[8];
static socket_handler_t socket_handlers[8];

void w5500_config_socket_irq(unsigned socket, socket_handler_t handler)
{
  uint16_t skt_en;
  if (handler) {
    // enable all sources for the socket
    socket_intmask[socket] = (SIK_CONNECTED | SIK_DISCONNECTED | SIK_RECEIVED | SIK_TIMEOUT); // except SendOK
    ctlsocket(socket, CS_SET_INTMASK, &socket_intmask[socket]);
    // enable the socket interrupt
    socket_irq_enabled |= (1 << socket) << 8;
    ctlwizchip(CW_SET_INTRMASK, &socket_irq_enabled);
    // assign the handler
    socket_handlers[socket] = handler;
  } else {
    // remove the handler
    socket_handlers[socket] = 0;
    // disable the socket interrupt
    socket_irq_enabled &= ~((1 << socket) << 8);
    ctlwizchip(CW_SET_INTRMASK, &socket_irq_enabled);
    // disable all events for the socket
    socket_intmask[socket] = 0;
    ctlsocket(socket, CS_SET_INTMASK, &socket_intmask[socket]);
  }
}

void w5500_on_irq(uint gpio, uint32_t events)
{
  // get the chip interrupts
  uint16_t cwirq;
  ctlwizchip(CW_GET_INTERRUPT, &cwirq);
# if 0
  printf("w5500_irq: %c%c%c%c%c%c%c%c %c%c\n",
    (cwirq & IK_SOCK_7) ? '7' : '-',
    (cwirq & IK_SOCK_6) ? '6' : '-',
    (cwirq & IK_SOCK_5) ? '5' : '-',
    (cwirq & IK_SOCK_4) ? '4' : '-',
    (cwirq & IK_SOCK_3) ? '3' : '-',
    (cwirq & IK_SOCK_2) ? '2' : '-',
    (cwirq & IK_SOCK_1) ? '1' : '-',
    (cwirq & IK_SOCK_0) ? '0' : '-',
    (cwirq & IK_DEST_UNREACH) ? 'U' : '-',
    (cwirq & IK_WOL) ? 'W' : '-'
  );
# endif
  // check all sockets irqs
  for (int i = 0; i < 8; ++i) {
    if (cwirq & (1<<(8+i)))
      ctlsocket(i, CS_GET_INTERRUPT, &socket_interrupt[i]);
    else
      socket_interrupt[i] = 0;
  }
  // foreach socket for which we have a handler, call the handler
  // for those that don't, print the interrupt
  for (int i = 0; i < 8; ++i) {
    unsigned irqs = socket_interrupt[i];
    if (irqs) {
      if (socket_handlers[i])
        (socket_handlers[i])(i, irqs);
      else {
#       if 0
        printf("S%c: %c%c%c%c\n", '0'+i, 
                (irqs & SIK_CONNECTED) ? 'C': '-',
                (irqs & SIK_DISCONNECTED) ? 'D': '-',
                (irqs & SIK_RECEIVED) ? 'R': '-',
                (irqs & SIK_SENT) ? 'S': '-',
                (irqs & SIK_TIMEOUT) ? 'T': '-');
#       endif
        // don't even think about clearing interrupts
        //ctlsocket(i, CS_CLR_INTERRUPT, &socket_interrupt[i]);
      }
    }
  }
  // don't even think about clearing interrupts
  //cwirq &= IK_SOCK_ALL;
  //ctlwizchip(CW_CLR_INTERRUPT, &cwirq);
}

#endif
