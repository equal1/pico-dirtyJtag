#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <hardware/dma.h>

#include "pico-w5500/Ethernet/socket.h"
#include "pico-w5500/Ethernet/wizchip_conf.h"
#include "pico-w5500/Internet/DHCP/dhcp.h"
#include "wizchip_conf.h"
#include "w5500.h"
#include "socket.h"
#include "dhcp.h"
#include "led.h"

#include "dirtyJtagConfig.h"

// MAC address (implemented in LwIP port)
uint8_t mac[6];

// socket allocation
#define SOCKET_DHCP 0
#define SOCKET_DBGSRV 1
//#define SOCKET_HTTP 2 // maybe later

// debug port
#define PORT_DBGSRV 8901

// data buffer
#define ETH_BUFFER_SIZE 2048
static uint8_t dhcp_buf[ETH_BUFFER_SIZE];
static struct {
  uint8_t buf[ETH_BUFFER_SIZE];
  uint32_t size;
  uint32_t sender_crc32, receiver_crc32;
} dbgsrv_in, dbgsrv_out;

// interface setup
static wiz_NetInfo network = {
  .dhcp = NETINFO_DHCP
};

// Ethernet FSM
static enum {
  ETH_FATAL = -1,
  ETH_NONE = 0,
  ETH_NO_LINK,
  ETH_HAVE_LINK, // transient
  ETH_WAIT_DHCP,
  ETH_HAVE_DHCP, // transient
  ETH_READY,
  ETH_HAVE_SOCKET,
  ETH_LISTENING,
  ETH_CONNECTING,
  ETH_CONNECTED,
  ETH_RECEIVING,
  ETH_JTAG_RUNNING, // wait for the JTAG to produce response data
  ETH_WAITING_REPLY, // got a packet that expects an answer
  ETH_SENDING_REPLY,
  ETH_DISCONNECTING
} eth_status;

uint8_t eth_link_state = 0xff;

// text stuff
static char eth_macaddr[18];
static char eth_err[80];
static char eth_own_ip[] = "111.111.111.111";
static char eth_listen_port[] = "111.111.111.111:12345", 
            eth_connected_port[] = "222.222.222.222:23456";
static char eth_hostname[] = "pico-0123456789abcdef";

static void w5500_spi_init();
static void w5500_dma_init();

// main callback, updates the ID part
// connected (-1: err), MAC, hostname, server listening, client connected
// if connected==-1. "hostname" is the error
void notify_ip_config(int, const char*, const char*, const char*, const char*);

static void eth_crc32_init();
void eth_crc32_start(const char *buf, unsigned len, uint32_t *destination);
static void eth_crc32_wait();

static int client_uses_crc32;

extern int eth_spi_speed;
static void w5500_on_irq(uint, uint32_t);
static void w5500_select();
static void w5500_deselect();
static uint8_t w5500_read();
static void w5500_write(uint8_t);
static void w5500_dma_init();
static void w5500_block_read(uint8_t*, uint16_t);
static void w5500_block_write(uint8_t*, uint16_t);
static int w5500_set_spi_speed(unsigned speed);

int eth_init(uint64_t uid)
{
  // configure the MAC address
  uint64_t macaddr = (uid & 0xf0ffffffffff) | 0x020000000000;
  network.mac[0] = (uint8_t)((macaddr >> 40) & 0xFF);
  network.mac[1] = (uint8_t)((macaddr >> 32) & 0xFF);
  network.mac[2] = (uint8_t)((macaddr >> 24) & 0xFF);
  network.mac[3] = (uint8_t)((macaddr >> 16) & 0xFF);
  network.mac[4] = (uint8_t)((macaddr >> 8 ) & 0xFF);
  network.mac[5] = (uint8_t)((macaddr >> 0 ) & 0xFF);
  sprintf (HOST_NAME, "pico-%010llX", uid>>24);
  sprintf (eth_hostname, "pico-%016llX", uid);
  sprintf (eth_macaddr, "%02X:%02X:%02X:%02X:%02X:%02X",
           network.mac[0], network.mac[1], network.mac[2],
           network.mac[3], network.mac[4], network.mac[5]);
  eth_status = ETH_NONE;
  // initial configuration of the W5500 interface
  w5500_dma_init();
  w5500_spi_init();

  // initialize the Ethernet chip itself
  reg_wizchip_cs_cbfunc(w5500_select, w5500_deselect); // chip select control
  reg_wizchip_spi_cbfunc(w5500_read, w5500_write); // receive/send
  reg_wizchip_spiburst_cbfunc(w5500_block_read, w5500_block_write); // receive/send block
  // grab the chip name (can't fail)
	ctlwizchip (CW_GET_ID, dhcp_buf);
  // try to find the maximum frequency at which the chip would talk to us
  while (! w5500_set_spi_speed(eth_spi_speed)) {
    // if we're >= 10MHz, decrease by 1MHz
    if (eth_spi_speed >= 10000000)
      eth_spi_speed -= 1000000;
    // if we're >= 1MHz, decrease by 0.5MHz
    else if (eth_spi_speed >= 1000000)
      eth_spi_speed -= 500000;
    // if we can't connect even at 1MHz, give up
    else {
      eth_spi_speed = 0;
      break;
    }
  }
  if (! eth_spi_speed) {
    notify_ip_config(0, 0, 0, 0, 0);
    eth_status = ETH_FATAL;
    return -1;
  }
  // grab the initial PHY link
  ctlwizchip (CW_GET_PHYLINK, (void *)&eth_link_state);
  // initialize crc32 support
  eth_crc32_init();
  // pass the MAC and the hostname to whoami()
  notify_ip_config(0, eth_macaddr, eth_hostname, 0, 0);
  eth_status = ETH_NO_LINK;
  // only keep the LED on when we have the IP connection up
  set_led(0);
  return 0;
}

static int w5500_init();
static void network_init();

typedef void (*socket_handler_t)(unsigned, unsigned);
static void w5500_config_socket_irq(unsigned socket, socket_handler_t handler);

// DHCP event handlers
static repeating_timer_t dhcp_timer;
static bool every_1s(repeating_timer_t*);

// error notification task
static repeating_timer_t error_timer;
static bool every_100ms(repeating_timer_t*);

#if 0
// don't mess with IRQs!!! - mircea
// (data gets lost if you do)
static void on_dhcp_irq(unsigned socket, unsigned irqs) {
  printf ("DHCP: %c%c%c%c\n",
          (irqs & SIK_CONNECTED) ? 'C': '-',
          (irqs & SIK_DISCONNECTED) ? 'D': '-',
          (irqs & SIK_RECEIVED) ? 'R': '-',
          (irqs & SIK_SENT) ? 'S': '-',
          (irqs & SIK_TIMEOUT) ? 'T': '-');
  //ctlsocket(socket, CS_CLR_INTERRUPT, &irqs);
}
#endif

static void on_dhcp_assign();
static void on_dhcp_update();
static void on_dhcp_conflict();

// check if the link is still up
static int check_link();

// check for transient socket states
static int is_socket_state_transient(int ss)
{
  if (ss == SOCK_SYNSENT ) return 1;
  if (ss == SOCK_SYNRECV ) return 1;
  if (ss == SOCK_FIN_WAIT ) return 1;
  if (ss == SOCK_CLOSING ) return 1;
  if (ss == SOCK_TIME_WAIT ) return 1;
  if (ss == SOCK_LAST_ACK ) return 1;
  return 0;
}

#define RESEND_RESPONSE_MAGIC 0x137f5aa5
#define RESEND_COMMAND_MAGIC  0x7f5aa513

void eth_task() {
  int tmp;
  int sock_state = -1;

  if (! eth_spi_speed)
    return;

  switch (eth_status) {
  // if ethernet failed, give up
  case ETH_FATAL:
    return;

  // this should never happen
  case ETH_NONE:
    if (eth_spi_speed) {
      strcpy (eth_err, "W5500 init failed");
    fatal_error:
      eth_status = ETH_FATAL;
      add_repeating_timer_ms (100, every_100ms, 0, &error_timer);
      printf ("eth: %s\n", eth_err);
      notify_ip_config(-1, eth_macaddr, eth_err, 0, 0);
      return;
    }
    network_init();
    eth_status = ETH_NO_LINK;
    notify_ip_config(0, eth_macaddr, eth_hostname, 0, 0);
    // fall-through

  case ETH_NO_LINK:
    // nothing left to do, if the phy us off
    if(! check_link())
      return;
    // fall-through

  case ETH_HAVE_LINK:
    // start the DHCP FSM
    //printf ("eth: requesting IP address...\n");
    add_repeating_timer_ms (1000, every_1s, 0, &dhcp_timer);
    // no messing with IRQs, thank you very much!
    // w5500_config_socket_irq(SOCKET_DHCP, on_dhcp_irq);
		DHCP_init(SOCKET_DHCP, dhcp_buf);
		reg_dhcp_cbfunc(on_dhcp_assign, on_dhcp_update, on_dhcp_conflict);
    eth_status = ETH_WAIT_DHCP;
    // fall-through
  
  case ETH_WAIT_DHCP:
    // make sure we still have a link
    if(! check_link())
      return;
    // run the DHCP FSM
		switch (DHCP_run()) {
		case DHCP_IP_ASSIGN:
		case DHCP_IP_CHANGED:
		case DHCP_IP_LEASED:
      set_led(1);
      eth_status = ETH_READY;
			break;
		case DHCP_FAILED:
			cancel_repeating_timer (&dhcp_timer);
      strcpy (eth_err, "cannot get IP address");
      goto fatal_error;
			return;
		default:
			return;
		}
    //printf ("eth: got IP %s\n", eth_own_ip);
    // fall-through

  case ETH_READY:
    if(! check_link())
      return;
    // socket state should totally be closed here
    sock_state = getSn_SR(SOCKET_DBGSRV);
    if (is_socket_state_transient(sock_state))
      return;
    if (sock_state != SOCK_CLOSED) {
      strcpy(eth_err, "in ETH_READY, sock_state not CLOSED");
      goto fatal_error;
    }
    if(socket(SOCKET_DBGSRV, Sn_MR_TCP, PORT_DBGSRV, 0x00) != SOCKET_DBGSRV) {
      strcpy(eth_err, "in ETH_READY, socket() failed");
      goto fatal_error;
    }
    //printf("eth: opened server socket\n");
    sprintf (eth_listen_port, "%s:%u", eth_own_ip, PORT_DBGSRV);
    eth_status = ETH_HAVE_SOCKET;
    // fall-through
  
  case ETH_HAVE_SOCKET:
    if(! check_link())
      return;
    // socket state should be open here
    sock_state = getSn_SR(SOCKET_DBGSRV);
    if (is_socket_state_transient(sock_state))
      return;
    if (sock_state != SOCK_INIT) {
      // at high SPI speeds, we might end up with transitory states in the socket state - 
      // - ignore these altogether
      return;
    }
    if(listen(SOCKET_DBGSRV) != SOCK_OK) {
      // at high SPI speeds, we might end up with transitory states in the socket state - 
      // - ignore these altogether
      return;
    }
    //printf("eth: listening on %s:%u\n", eth_own_ip, PORT_DBGSRV);
    notify_ip_config(1, eth_macaddr, eth_hostname, eth_listen_port, 0);
    eth_status = ETH_LISTENING;
    // fall-through

  case ETH_LISTENING:
    if(! check_link())
      return;
    // this can go to either SOCK_CLOSED (on timeout) or to SOCK_ESTABLISHED (on connected)
    // or it can stay in SOCK_LISTEN
    sock_state = getSn_SR(SOCKET_DBGSRV);
    if (is_socket_state_transient(sock_state))
      return;
    if (sock_state == SOCK_CLOSED) {
      puts ("eth: socket timeout, disconnected");
      eth_status = ETH_READY;
      return;
    }
    if (sock_state == SOCK_LISTEN)
      return;
    eth_status = ETH_CONNECTING;
    // fall-through

  case ETH_CONNECTING:
    if(! check_link())
      return;
    // socket state should be ESTABLISHED here
    // but technically you could also have CLOSED (on timeout) or CLOSE_WAIT (on closed connection)
    sock_state = getSn_SR(SOCKET_DBGSRV);
    if (is_socket_state_transient(sock_state))
      return;
    if (sock_state == SOCK_CLOSED) {
      puts ("eth: socket timeout, disconnected");
      eth_status = ETH_READY;
      return;
    }
    if (sock_state == SOCK_CLOSE_WAIT) {
      //puts ("eth: connection closing");
      eth_status = ETH_DISCONNECTING;
      return;
    }
    if (sock_state != SOCK_ESTABLISHED) {
      // at high SPI speeds, we might end up with transitory states in the socket state - 
      // - ignore these altogether
      return;
    }
    if(getSn_IR(SOCKET_DBGSRV) & Sn_IR_CON) {
      uint8_t cliip[4];
      uint16_t cliport;
      getSn_DIPR(SOCKET_DBGSRV, cliip);
      cliport = getSn_DPORT(SOCKET_DBGSRV);
      setSn_IR(SOCKET_DBGSRV,Sn_IR_CON);
      sprintf (eth_connected_port, "%u.%u.%u.%u:%u",
               cliip[0], cliip[1], cliip[2], cliip[3], cliport);
      notify_ip_config(1, eth_macaddr, eth_hostname, eth_listen_port, eth_connected_port);
      //printf ("eth: accepted connection from %s\n", eth_connected_port);
    }
    else {
      puts ("eth: accepted connection from unknown source!!!");
    }
    eth_status = ETH_CONNECTED;
    // reset the crc32 capability to unknown
    client_uses_crc32 = -1;
    // fall-through

  case ETH_CONNECTED:
    if(! check_link())
      return;
    // this should normally be in ESTABLISHED, or CLOSE_WAIT (on closed connection)
    // CLOSED (on timeout) is also a possibility
    sock_state = getSn_SR(SOCKET_DBGSRV);
    if (is_socket_state_transient(sock_state))
      return;
    if (sock_state == SOCK_CLOSED) {
      puts ("eth: socket timeout, disconnected");
      eth_status = ETH_READY;
      return;
    }
    if (sock_state == SOCK_CLOSE_WAIT) {
      puts ("eth: connection closing");
      eth_status = ETH_DISCONNECTING;
      return;
    }
    if (sock_state != SOCK_ESTABLISHED) {
      // at high SPI speeds, we might end up with transitory states in the socket state - 
      // - ignore these altogether
      return;
    }
    // see if there's any data to receive
    int len = getSn_RX_RSR(SOCKET_DBGSRV);
    if ((len < 0) || (len > ETH_BUFFER_SIZE)) {
      if (len < 0)
        puts ("eth: read error, disconnecting client...");
      else
        printf ("eth: packet too large (size=%u, max=%u), disconnecting client...\n", len, ETH_BUFFER_SIZE);
    do_close_socket:
      close(SOCKET_DBGSRV);
      eth_status = ETH_DISCONNECTING;
      return;
    }
    if (! len)
      return;
    eth_status = ETH_RECEIVING;
    // fall-through

  case ETH_RECEIVING:
    dbgsrv_in.size = recv(SOCKET_DBGSRV, dbgsrv_in.buf, len);

    if (((int)dbgsrv_in.size < 0) || (len != dbgsrv_in.size)) {
      if ((int)dbgsrv_in.size < 0)
        puts ("eth: receive error, disconnecting client...");
      else
        printf ("eth: read error (expected=%d, got=%d), disconnecting client...\n", len, dbgsrv_in.size);
      close(SOCKET_DBGSRV);
      eth_status = ETH_READY;
      return;
    }
    // good stuff; see what the client expects
    uint32_t payload_size = *(uint32_t*)&dbgsrv_in.buf[0];
    uint32_t result_size = *(uint32_t*)&dbgsrv_in.buf[4];
    // check if the client is using crc32, on the first 
    if (client_uses_crc32 == -1) {
      if (dbgsrv_in.size == payload_size + 12) {
        puts("eth: client uses crc32...");
        client_uses_crc32 = 1;
      }
      else if (dbgsrv_in.size == payload_size + 8) {
        puts("eth: client doesn't use crc32...");
        client_uses_crc32 = 0;
      }
      else { // can't get a re-send request on the first packet!
        puts("eth: protocol error, disconnecting client...");
        goto do_close_socket;
      }
    }
    // if we're using crc32, check the crc32
    if (client_uses_crc32) {
      // check for re-send requests
      if ((dbgsrv_in.size == 4) && (*(uint32_t*)dbgsrv_in.buf == RESEND_RESPONSE_MAGIC)) {
        puts("eth: client wants the last response re-sent...");
        eth_status = ETH_WAITING_REPLY;
        break;
      }
      // check for protocol violations
      if (dbgsrv_in.size != payload_size + 12) {
        printf("eth: protocol error (packet:%ub; payload=%db, expected%ub); disconnecting client...\n",
               dbgsrv_in.size, payload_size, 12+payload_size);
        goto do_close_socket;
      }
      // extract the crc32 from the input packet
      if (! (((uint32_t)&dbgsrv_in.buf[payload_size + 8]) & 3))
        dbgsrv_in.sender_crc32 = *(uint32_t*)&dbgsrv_in.buf[payload_size + 8];
      else
        memcpy(&dbgsrv_in.sender_crc32, &dbgsrv_in.buf[payload_size + 8], 4);
      eth_crc32_start(dbgsrv_in.buf, payload_size + 8, &dbgsrv_in.receiver_crc32);
      eth_crc32_wait();
      if (dbgsrv_in.sender_crc32 != dbgsrv_in.receiver_crc32) {
        printf("eth: packet crc error, got %08X, computed %08X; requesting re-send\n", dbgsrv_in.sender_crc32, dbgsrv_in.receiver_crc32);
        // send a re-request packet
        *(uint32_t*)&dbgsrv_out.buf = RESEND_COMMAND_MAGIC;
        unsigned sent = send(SOCKET_DBGSRV, dbgsrv_out.buf, 4);
        if (sent != 4) {
          puts("eth: failed to send send packet re-request!!!");
          strcpy(eth_err, "send() failed");
          goto fatal_error;
        }
        eth_status = ETH_RECEIVING;
        break;
      }
    } 
    // not using crc32
    else {
      // check for protocol violations
      if (dbgsrv_in.size != payload_size + 8) {
        printf("eth: protocol error (packet:%ub; payload=%db, expected%ub); disconnecting client...\n",
               dbgsrv_in.size, payload_size, 8+payload_size);
        goto do_close_socket;
      }
    }
    // run the commands anyway before sending a reply
    // the protocol is ping-pong - we always send a reply, even if that's zero-sized
    eth_status = ETH_JTAG_RUNNING;
    break;

  case ETH_JTAG_RUNNING:
    // stay here indefinitely - we get out on a submit_eth_data() callback
    return;

  case ETH_WAITING_REPLY:
    if(! check_link())
      return;
    // if something happened to the socket, disconnect
    sock_state = getSn_SR(SOCKET_DBGSRV);
    if (is_socket_state_transient(sock_state))
      return;
    if (sock_state == SOCK_CLOSED) {
      puts ("eth: socket timeout, disconnected");
      eth_status = ETH_READY;
      return;
    }
    if (sock_state == SOCK_CLOSE_WAIT) {
      puts ("eth: connection closing");
      eth_status = ETH_DISCONNECTING;
      return;
    }
    if (sock_state != SOCK_ESTABLISHED) {
      // at high SPI speeds, we might end up with transitory states in the socket state - 
      // - ignore these altogether
      return;
    }
    // if we produced the expected data OR no data was expected OR
    if (dbgsrv_out.size) {
      uint32_t expected_size = *(uint32_t*)&dbgsrv_in.buf[4];
      uint32_t got_size = dbgsrv_out.size - 4;
      uint32_t send_size = got_size;
      // adjust sending size, if open-ended reply
      if((int32_t)expected_size < 0) {
        expected_size &= ~0x80000000; // see the maximum buffer size
        if (expected_size > got_size)
          expected_size = got_size;
        else {
          printf ("eth: warning, have %u response bytes, client only requested up to %u; truncating response...\n", got_size, expected_size);
          *(uint32_t*)&dbgsrv_out.buf[0] = got_size = expected_size;
        }
      }
      // truncate sending size, if we produced too much data
      else {
        if (got_size > expected_size) {
          printf ("eth: warning, have %u response bytes, client only requested %u; truncating response...\n", got_size, expected_size);
          *(uint32_t*)&dbgsrv_out.buf[0] = got_size = expected_size;
        }
      }
      // if any reply length is fine
      if (got_size != expected_size) {
        printf ("eth: protocol error (client expects %d, have %d), disconnecting client...\n", expected_size, got_size);
        goto do_close_socket;
      }
      // send the reply and resume waiting for data
      unsigned sent = send(SOCKET_DBGSRV, dbgsrv_out.buf, 4+got_size);
      if (sent != 4+got_size) {
        puts ("eth: send error, disconnecting client...");
        goto do_close_socket;
      }
      // printf ("eth: sent response, size=%u...\n", sent);
      // protocol phase done - return to receiving
      eth_status = ETH_CONNECTED;
      return;
    }
    // if not, nothing to do yet - stay in waiting_reply
    break;

  case ETH_DISCONNECTING:
    if(! check_link())
      return;
    sock_state = getSn_SR(SOCKET_DBGSRV);
    if (is_socket_state_transient(sock_state))
      return;
    // socket successfully closed? go back to ready
    if (sock_state == SOCK_CLOSED) {
      eth_status = ETH_READY;
      notify_ip_config(1, eth_macaddr, eth_hostname, eth_listen_port, 0);
      return;
    }
    // stay in DISCONNECTING while the state if still CLOSE_WAIT
    if (sock_state == SOCK_CLOSE_WAIT) {
      close(SOCKET_DBGSRV);
      return;
    }
    // at high SPI speeds, we might end up with transitory states in the socket state - 
    // - ignore these altogether
    return;

  default:
    sprintf(eth_err, "Invalid ethernet state %u", eth_status);
    goto fatal_error;
  }
}

// crc32

#define CRC32_INIT ((uint32_t)-1l)
static int eth_crc32_dmach_idx;
static dma_channel_config eth_crc32_dmach;
static uint32_t *eth_crc32_destination = 0;

void eth_crc32_init()
{
  // configure the crc32 DMA (alternate, "reversed" shift direction)
  eth_crc32_dmach_idx = dma_claim_unused_channel(true);
  eth_crc32_dmach = dma_channel_get_default_config(eth_crc32_dmach_idx);
  channel_config_set_transfer_data_size(&eth_crc32_dmach, DMA_SIZE_8);
  channel_config_set_read_increment(&eth_crc32_dmach, true);
  channel_config_set_write_increment(&eth_crc32_dmach, false);
  dma_sniffer_set_output_reverse_enabled(true);
  dma_sniffer_enable(eth_crc32_dmach_idx, DMA_SNIFF_CTRL_CALC_VALUE_CRC32R, true);
  dma_sniffer_set_data_accumulator(CRC32_INIT);
}

void eth_crc32_start(const char *buf, unsigned len, uint32_t *destination)
{
  if (eth_crc32_destination) {
    puts("started a CRC32 while another was running!");
    return;
  }
  eth_crc32_destination = destination;
  // configure DMA and kick off
  static uint8_t dummy;
  dma_channel_configure(eth_crc32_dmach_idx, &eth_crc32_dmach,
                        &dummy, buf, len,
                        true);
}

void eth_crc32_wait()
{
  // don't do anything, if no transfer is in progress
  if (! eth_crc32_destination)
    return;
  // wait for the transfer, and record the crc32
  dma_channel_wait_for_finish_blocking(eth_crc32_dmach_idx);
  *eth_crc32_destination = dma_sniffer_get_data_accumulator();
  // reset accumulator and destination pointer
  dma_sniffer_set_data_accumulator(CRC32_INIT);
  eth_crc32_destination = 0;
}

// this only sees data that was crc32-checked already
int fetch_eth_data(char *dest)
{
  if (eth_status != ETH_JTAG_RUNNING)
    return 0;
  int n = *(uint32_t*)&dbgsrv_in.buf[0];
  if (n > 0) {
    memcpy(dest, &dbgsrv_in.buf[8], n);
    //printf ("eth: received %u cmd bytes\n", n);
    return n;
  }
  return 0;
}

int submit_eth_data(const char *src, unsigned n)
{
  // only accept replies in JTAG_RUNNING mode
  if (eth_status != ETH_JTAG_RUNNING)
    return 0;
  // limit the response size, if response size was open-ended
  int expected = *(uint32_t*)&dbgsrv_in.buf[4];
  if (expected < 0) {
    expected &= ~0x80000000;
    // if we have more data, trim the response
    if (n > expected)
      n = expected;
    // patch the input buffer to the number of bytes we actually produced
    // (note we already checked its, so patching's fine)
    *(uint32_t*)&dbgsrv_in.buf[4] = n;
  }
  // if the response size is wrong, eth_task() will pick this up
  dbgsrv_out.size = 4+n;
  *(uint32_t*)&dbgsrv_out.buf[0] = n;
  if (n > 0) {
    memcpy(&dbgsrv_out.buf[4], src, n);
    //printf ("eth: sending %u reply bytes\n", n);
  }
  // 
  // move on the ethernet state machine
  eth_status = ETH_WAITING_REPLY;
  return n;
}

int eth_sending_data() {
  return eth_status == ETH_WAITING_REPLY;
}

// timer that runs while DHCP is running
// (also toggles the LED every second)
bool every_1s(repeating_timer_t *rt)
{
  DHCP_time_handler();
  // toggle the LED every second while waiting for DHCP
  if (eth_status < ETH_READY)
    toggle_led();
  return true;
}

// error timer
// toggles the LED every .1s
bool every_100ms(repeating_timer_t *rt)
{
  // toggle the LED every second while waiting for DHCP
  if (eth_status == ETH_FATAL)
    toggle_led();
  return true;
}

void w5500_spi_init()
{
  // configure CS#
  gpio_init(PIN_ETH_CSn);
  gpio_put(PIN_ETH_CSn, 1); // initially de-selected
  gpio_set_dir(PIN_ETH_CSn, GPIO_OUT);
  bi_decl(bi_1pin_with_name(PIN_ETH_CSn, "ETH_CS#"));
  // configure RST#
  gpio_init(PIN_ETH_RSTn);
  gpio_put(PIN_ETH_RSTn, 0); // initially held in reset
  gpio_set_dir(PIN_ETH_RSTn, GPIO_OUT);
  bi_decl(bi_1pin_with_name(PIN_ETH_RSTn, "ETH_RST#"));
  // configure INT#
  gpio_init(PIN_ETH_INTn);
  gpio_set_dir(PIN_ETH_INTn, GPIO_IN);
  gpio_put(PIN_ETH_INTn, 0);
  bi_decl(bi_1pin_with_name(PIN_ETH_INTn, "ETH_INT#"));
  //gpio_set_irq_enabled_with_callback(PIN_W5500_INTn, GPIO_IRQ_EDGE_FALL, true, w5500_on_irq);
  // configure SPI itself - mode 0
  spi_init(SPI_ETH, FREQ_ETH_KHZ * 1000);
  eth_spi_speed = spi_get_baudrate(SPI_ETH);
  spi_set_format (SPI_ETH, 8, 0, 0, 0);
  // configure the SPI pins
  gpio_set_function(PIN_ETH_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_ETH_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_ETH_MISO, GPIO_FUNC_SPI);
  bi_decl(bi_3pins_with_func(PIN_ETH_MISO, PIN_ETH_MOSI, PIN_ETH_SCK, GPIO_FUNC_SPI));
}


int w5500_set_spi_speed(unsigned speed) {
# if 0
  printf ("W5500: attempting SPI at %u.%03uMHz...\n", 
          (eth_spi_speed+500)/1000000, ((eth_spi_speed+500)%1000000/1000) );
# endif
  // re-configure the SPI speed
  spi_set_baudrate(SPI_ETH, speed);
  eth_spi_speed = spi_get_baudrate(SPI_ETH);
  // pulse reset
  gpio_put(PIN_ETH_RSTn, 0); sleep_ms(1);
  gpio_put(PIN_ETH_RSTn, 1); sleep_ms(1);
  // initialize the chip
  ctlwizchip (CW_RESET_WIZCHIP, 0); sleep_ms(1);
	static const uint8_t socks_sizes[2][8] = {
    // maybe optimize later?
    { 2, 2, 2, 2, 2, 2, 2, 2 }, 
    { 2, 2, 2, 2, 2, 2, 2, 2 }
  };
  ctlwizchip (CW_INIT_WIZCHIP, (void*)socks_sizes); sleep_ms(1);
  ctlwizchip (CW_RESET_PHY, 0); sleep_ms(1);
  // check if communication works
  int ok = getVERSIONR() == 0x04;
  return ok;
}

int w5500_init()
{
  // set the operators
  reg_wizchip_cs_cbfunc(w5500_select, w5500_deselect); // chip select control
  reg_wizchip_spi_cbfunc(w5500_read, w5500_write); // receive/send
  reg_wizchip_spiburst_cbfunc(w5500_block_read, w5500_block_write); // receive/send block
  // grab the chip name (can't fail)
	ctlwizchip (CW_GET_ID, dhcp_buf);
  unsigned version = 0;
  // try to find the maximum frequency at which the chip would talk to us
  unsigned last_bad_spi_speed = 0;
  while (! w5500_set_spi_speed(eth_spi_speed)) {
    // if we're >= 10MHz, decrease by 1MHz
    if (eth_spi_speed >= 10000000)
      eth_spi_speed -= 1000000;
    // if we're >= 1MHz, decrease by 0.5MHz
    else if (eth_spi_speed >= 1000000)
      eth_spi_speed -= 500000;
    // if we can't connect even at 1MHz, give up
    else {
      eth_spi_speed = 0;
      return -4;
    }
  }
  // grab the initial PHY link
  ctlwizchip (CW_GET_PHYLINK, (void *)&eth_link_state);
  return 0;
}

void network_init()
{
  // stop the DHCP FSM, if startd
  cancel_repeating_timer (&dhcp_timer);
  DHCP_stop();
  // (re)set the network info
  *(uint32_t*)&network.ip = 0;
  *(uint32_t*)&network.gw = 0;
  *(uint32_t*)&network.sn = 0;
  *(uint32_t*)&network.dns = 0;
  network.dhcp = NETINFO_DHCP;
	ctlnetwork (CN_SET_NETINFO, (void*)&network);
  eth_link_state = 0xFF;
  eth_status = ETH_NONE;
  check_link();
}

// check if the link is still up
static int check_link()
{
  // check if we have a link
  ctlwizchip(CW_GET_PHYLINK, (void *)&eth_link_state);
  // if we lost the link, say so
  if (eth_link_state == PHY_LINK_OFF) {
    if (eth_status == ETH_NO_LINK)
      return 0;
    set_led(0);
    notify_ip_config(0, eth_macaddr, eth_hostname, 0, 0);
    //printf ("eth: %s PHY link\n", (eth_status == ETH_NONE) ? "no" : "lost");
    // attempt to close the server socket
    close(SOCKET_DBGSRV);
    // also set the link down, and stop/cleanup the DHCP state
    cancel_repeating_timer (&dhcp_timer);
    DHCP_stop();
    eth_status = ETH_NO_LINK;
    return 0;
  }
  if (eth_status < ETH_HAVE_LINK) {
    notify_ip_config(1, eth_macaddr, eth_hostname, 0, 0);
    //printf ("eth: %s PHY link\n", (eth_status == ETH_NONE) ? "have" : "got");
    eth_status = ETH_HAVE_LINK;
  }
  return 1;
}

/*
// debug server PCB
static struct tcp_pcb *dbgsrv_pcb = 0;
// debug server connection
struct dbgsrv_conn_s
{
  struct tcp_pcb *pcb; // current PCB
  struct pbuf *p;      // pointer to the current pbuf
  enum dbgsrv_states_e {
    ES_NONE = 0,
    ES_ACCEPTED,
    ES_RECEIVED,
    ES_CLOSING
  } state;              // current state
  u8_t retries;
} dbgsrv_conn;

// API
void dbgsrv_init();
void dbgsrv_send(struct tcp_pcb*);
void dbgsrv_connection_close(struct tcp_pcb*);

// callbacks
err_t dbgsrv_on_accept(void*, struct tcp_pcb*, err_t);
err_t dbgsrv_on_recv(void*, struct tcp_pcb*, struct pbuf*, err_t err);
void dbgsrv_on_err(void*, err_t);
err_t dbgsrv_on_poll(void*, struct tcp_pcb*);
err_t dbgsrv_on_sent(void*, struct tcp_pcb*, u16_t);

void dbgsrv_init()
{
  // cleanup, if need be
  if (dbgsrv_pcb) {
    memp_free(MEMP_TCP_PCB, dbgsrv_pcb);
    dbgsrv_pcb = 0;
  }
  // create the new tcp
  dbgsrv_pcb = tcp_new();
  if (! dbgsrv_pcb) {
    eth_status = ETH_FATAL;
    strcpy(w5500_err, "failed to allocate PCB for debug server");
    return;
  }
  // bind to the debug port
  if (tcp_bind (dbgsrv_pcb, &e0.ip_addr, PORT_DBGSRV) != ERR_OK) {
    eth_status = ETH_FATAL;
    sprintf(w5500_err, "failed to bind to %s:%u", ip4addr_ntoa((const ip4_addr_t*)&eth_own_ip), PORT_DBGSRV);
    memp_free(MEMP_TCP_PCB, dbgsrv_pcb);
    dbgsrv_pcb = 0;
    return;
  }
  // listen to the port
  dbgsrv_pcb = tcp_listen(dbgsrv_pcb);
  // accept connections
  tcp_accept(dbgsrv_pcb, dbgsrv_on_accept);
  // show the message
  printf ("bound to %s:%u\n", ip4addr_ntoa((const ip4_addr_t*)&eth_own_ip), PORT_DBGSRV);
}

err_t dbgsrv_on_accept(void *arg, struct tcp_pcb *newpcb, err_t err)
{
  // args aren't used
  (void)arg; (void)err;
  // set priority for the newly accepted tcp connection
  tcp_setprio(newpcb, TCP_PRIO_MIN);
  dbgsrv_conn.state = ES_ACCEPTED;
  dbgsrv_conn.pcb = newpcb;
  dbgsrv_conn.retries = 0;
  dbgsrv_conn.p = NULL;
  // // pass the connection state structure as argument to newpcb
  // tcp_arg(newpcb, &dbgsrv_conn);
  // set the callbacks
  tcp_recv(newpcb, dbgsrv_on_recv); // on received
  tcp_err(newpcb, dbgsrv_on_err);   // on error
  tcp_poll(newpcb, dbgsrv_on_poll, 0); // on poll
  // // for error handling:
  // dbgsrv_connection_close(newpcb);
  return ERR_OK;
}

err_t dbgsrv_on_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
  // close connection if we received an empty tcp frame
  if (! p) {
    // client closed connection
    dbgsrv_conn.state = ES_CLOSING;
    if (! dbgsrv_conn.p)
      // done sending - close connection
      dbgsrv_connection_close(tpcb);
    else {
      // not done:
      // - acknowledge received packet
      tcp_sent(tpcb, dbgsrv_on_sent);
      // - send remaining date
      dbgsrv_send(tpcb);
    }
    return ERR_OK;
  }
  // if a non-empty frame was received, but there's some error
  if (err != ERR_OK) {
    // free received pbuf
    if (p) {
      dbgsrv_conn.p = NULL;
      pbuf_free(p);
    }
    return err;
  }
  // initially: receive data
  if (dbgsrv_conn.state == ES_ACCEPTED) {
    // first data chunk is in in p->payload
    dbgsrv_conn.state = ES_RECEIVED;
    // store the reference to the incoming pbuf (chain)
    dbgsrv_conn.p = p;
    // <!!!>
    // setup the on-sent callback 
    tcp_sent(tpcb, dbgsrv_on_sent);
    // send back the result
    dbgsrv_send(tpcb);
    // </!!!>
    return ERR_OK;
  }
  // received more data?
  // (this shouldn't happen)
  if (dbgsrv_conn.state == ES_RECEIVED) {
    // if we already sent the reply to the previous request
    if (! dbgsrv_conn.p) {
      dbgsrv_conn.p = p;
      // <!!!>
      // send back the response
      // dbgsrv_send(tpcb, es);
      // </!!!>
    }
    // if not yet: enqueue the response
    else
      // chain pbufs to the end of what we recv'ed previously
      pbuf_chain(dbgsrv_conn.p, p);
    return ERR_OK;
  }
  // closing twice?!
  if (dbgsrv_conn.state == ES_CLOSING) {
    // just discard the data
    tcp_recved(tpcb, p->tot_len);
    dbgsrv_conn.p = NULL;
    pbuf_free(p);
    return ERR_OK;
  }
  // unknown state: discard the data
  tcp_recved(tpcb, p->tot_len);
  dbgsrv_conn.p = NULL;
  pbuf_free(p);
  return ERR_OK;
}

void dbgsrv_on_err(void *arg, err_t err)
{
  LWIP_UNUSED_ARG(err);
  printf ("TCP error!\n");
}

err_t dbgsrv_on_poll(void *arg, struct tcp_pcb *tpcb)
{
  err_t ret_err;
  // this shouldn't happen
  if (! arg) {
    // nothing to do
    tcp_abort(tpcb);
    return ERR_ABRT;
  }
  // send remaining queued data (if any)
  if (dbgsrv_conn.p) {
    // set onSent callback
    tcp_sent(tpcb, dbgsrv_on_sent);
    // there is a remaining pbuf (chain) , try to send data
    dbgsrv_send(tpcb);
  }
  // no remaining pbuf in the chain?
  else if (dbgsrv_conn.state == ES_CLOSING)
    //  close the tcp connection
    dbgsrv_connection_close(tpcb);
  return ERR_OK;
}

err_t dbgsrv_on_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  (void)len;
  dbgsrv_conn.retries = 0;
  // send, if we've more data queued to send
  if (dbgsrv_conn.p) {
    tcp_sent(tpcb, dbgsrv_on_sent);
    dbgsrv_send(tpcb);
  }
  // if no more data to send and client closed connection
  else if (dbgsrv_conn.state == ES_CLOSING)
    dbgsrv_connection_close(tpcb);
  return ERR_OK;
}

void dbgsrv_send(struct tcp_pcb *tpcb)
{
  struct pbuf *ptr;
  err_t wr_err = ERR_OK;

  while ((wr_err == ERR_OK) && dbgsrv_conn.p && (dbgsrv_conn.p->len <= tcp_sndbuf(tpcb)))
  {
    // get the pointer to pbuf
    ptr = dbgsrv_conn.p;
    // enqueue data for transmission
    wr_err = tcp_write(tpcb, ptr->payload, ptr->len, 1);
    if (wr_err == ERR_OK) {
      unsigned plen = ptr->len;
      // continue with next pbuf in chain (if any)
      dbgsrv_conn.p = ptr->next;
      if (dbgsrv_conn.p)
        // increment reference count for dbgsrv_conn.p
        pbuf_ref(dbgsrv_conn.p);
      // chop first pbuf from chain
      unsigned freed;
      do
        // try to free pbuf
        freed = pbuf_free(ptr);
      while (! freed);
      // we can read more data now
      tcp_recved(tpcb, plen);
    }
    else if (wr_err == ERR_MEM)
      // if we're low on memory, try later / harder, defer to poll
      dbgsrv_conn.p = ptr;
    else
      // other problem ??
      printf ("dbgsrv_send(): error %d\n", wr_err);
  }
}

void dbgsrv_connection_close(struct tcp_pcb *tpcb)
{
  // remove all callbacks
  tcp_arg(tpcb, NULL);
  tcp_sent(tpcb, NULL);
  tcp_recv(tpcb, NULL);
  tcp_err(tpcb, NULL);
  tcp_poll(tpcb, NULL, 0);
  // close the tcp connection
  tcp_close(tpcb);
}

*/

// DHCP events
// - address assigned
void on_dhcp_assign(void)
{
  void default_ip_assign(void);
  default_ip_assign();
  getIPfromDHCP(network.ip);
  getGWfromDHCP(network.gw);
  getSNfromDHCP(network.sn);
  getDNSfromDHCP(network.dns);
  network.dhcp = NETINFO_DHCP;
  // set the network info
	ctlnetwork (CN_SET_NETINFO, (void*)&network);
  sprintf (eth_own_ip, "%u.%u.%u.%u", network.ip[0], network.ip[1], network.ip[2], network.ip[3]);
	//printf ("DHCP: got IP %s from %d.%d.%d.%d, netmask %d.%d.%d.%d; DNS %u.%u.%u.%u\n",
  //        eth_ip,
	//        network.gw[0], network.gw[1], network.gw[2], network.gw[3],
	//        network.sn[0], network.sn[1], network.sn[2], network.sn[3],
  //        network.dns[0], network.dns[1], network.dns[2], network.dns[3]);
}
// - address changed
void on_dhcp_update(void)
{
  printf ("DHCP: address changed; restarting network\n");
  network_init();
  eth_link_state = ETH_NO_LINK;
}
// - IP conflict
void on_dhcp_conflict(void)
{
  printf ("DHCP: address conflict; aborting network\n");
  network_init();
  strcpy (eth_err, "DHCP: conflict");
  eth_status = ETH_FATAL;
}

// other W5500 functions

uint16_t socket_irq_enabled = 0;
uint8_t socket_intmask[8], socket_interrupt[8];
socket_handler_t socket_handlers[8];

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
  printf ("w5500_irq: %c%c%c%c%c%c%c%c %c%c\n",
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
        printf ("S%c: %c%c%c%c\n", '0'+i, 
                (irqs & SIK_CONNECTED) ? 'C': '-',
                (irqs & SIK_DISCONNECTED) ? 'D': '-',
                (irqs & SIK_RECEIVED) ? 'R': '-',
                (irqs & SIK_SENT) ? 'S': '-',
                (irqs & SIK_TIMEOUT) ? 'T': '-');
        //ctlsocket(i, CS_CLR_INTERRUPT, &socket_interrupt[i]);
      }
    }
  }
  //cwirq &= IK_SOCK_ALL;
  //ctlwizchip(CW_CLR_INTERRUPT, &cwirq);
}


// SPI functions

void w5500_select(void)
{
  // reload the SPI settings, if needed
  if (spi_get_baudrate(SPI_ETH) != eth_spi_speed)
    spi_set_baudrate(SPI_ETH, eth_spi_speed);
  gpio_put(PIN_ETH_CSn, 0);
}

void w5500_deselect(void)
{
  gpio_put(PIN_ETH_CSn, 1);
}

uint8_t w5500_read(void)
{
  uint8_t rx_data = 0;
  uint8_t tx_data = 0xFF;
  spi_read_blocking(SPI_ETH, tx_data, &rx_data, 1);
  return rx_data;
}

void w5500_write(uint8_t tx_data)
{
  spi_write_blocking(SPI_ETH, &tx_data, 1);
}

static uint eth_dma_tx, eth_dma_rx;
static dma_channel_config eth_dma_tx_cfg, eth_dma_rx_cfg;

void w5500_dma_init()
{
  eth_dma_tx = dma_claim_unused_channel(true);
  eth_dma_tx_cfg = dma_channel_get_default_config(eth_dma_tx);
  channel_config_set_transfer_data_size(&eth_dma_tx_cfg, DMA_SIZE_8);
  channel_config_set_dreq(&eth_dma_tx_cfg, spi_get_dreq(SPI_ETH, true));
  channel_config_set_write_increment(&eth_dma_tx_cfg, false);
  eth_dma_rx = dma_claim_unused_channel(true);
  eth_dma_rx_cfg = dma_channel_get_default_config(eth_dma_rx);
  channel_config_set_transfer_data_size(&eth_dma_rx_cfg, DMA_SIZE_8);
  channel_config_set_dreq(&eth_dma_rx_cfg, spi_get_dreq(SPI_ETH, false));
  channel_config_set_read_increment(&eth_dma_rx_cfg, false);
}

void w5500_block_read(uint8_t *pBuf, uint16_t len)
{
  // use spi_read_blocking() for small blocks
  if (len <= 16) {
    spi_read_blocking(SPI_ETH, 0xFF, pBuf, len);
    return;
  }
  // DMAs for larger transfers
  static const uint8_t dummy_data = 0xFF;
  channel_config_set_read_increment(&eth_dma_tx_cfg, false);
  dma_channel_configure(eth_dma_tx, &eth_dma_tx_cfg,
                        &spi_get_hw(SPI_ETH)->dr, // write address
                        &dummy_data,                // read address
                        len,                        // element count (each element is of size transfer_data_size)
                        false);                     // don't start yet
  channel_config_set_write_increment(&eth_dma_rx_cfg, true);
  dma_channel_configure(eth_dma_rx, &eth_dma_rx_cfg,
                        pBuf,                       // write address
                        &spi_get_hw(SPI_ETH)->dr, // read address
                        len,                        // element count (each element is of size transfer_data_size)
                        false);                     // don't start yet
  dma_start_channel_mask((1u << eth_dma_tx) | (1u << eth_dma_rx));
  dma_channel_wait_for_finish_blocking(eth_dma_rx);
}

void w5500_block_write(uint8_t *pBuf, uint16_t len)
{
  // use spi_read_blocking() for small blocks
  if (len <= 16) {
    spi_write_blocking(SPI_ETH, pBuf, len);
    return;
  }
  // DMAs for larger transfers
  static uint8_t dummy_data;
  channel_config_set_read_increment(&eth_dma_tx_cfg, true);
  dma_channel_configure(eth_dma_tx, &eth_dma_tx_cfg,
                        &spi_get_hw(SPI_ETH)->dr, // write address
                        pBuf,                       // read address
                        len,                        // element count (each element is of size transfer_data_size)
                        false);                     // don't start yet
  channel_config_set_write_increment(&eth_dma_rx_cfg, false);
  dma_channel_configure(eth_dma_rx, &eth_dma_rx_cfg,
                        &dummy_data,                // write address
                        &spi_get_hw(SPI_ETH)->dr, // read address
                        len,                        // element count (each element is of size transfer_data_size)
                        false);                     // don't start yet
  dma_start_channel_mask((1u << eth_dma_tx) | (1u << eth_dma_rx));
  dma_channel_wait_for_finish_blocking(eth_dma_rx);
}
