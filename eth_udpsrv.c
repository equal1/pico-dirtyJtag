#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>
#include <wizchip_conf.h>
#include <w5500.h>
#include <socket.h>

#include "config.h"
#include "ethernet.h"
#include "pico-w5500/Ethernet/W5500/w5500.h"
#include "utils.h"

//#define DEBUG

#ifdef DEBUG
#define dprintf(...) printf(__VA_ARGS__)
#else
#define dprintf(...) (void)(__VA_ARGS__)
#endif

static void udpsrv_on_link_up(int);
static void udpsrv_on_link_down(int);
static void udpsrv_process(int, int);

int udpsrv_init()
{
  int res = eth_register_service(SOCKET_UDPSRV,
                                 udpsrv_on_link_up, udpsrv_on_link_down,
                                 udpsrv_process);
  if (res)
    printf("udpsrv: FAILED to register UDP server on port %u, socket #%u\n",
           PORT_UDPSRV, SOCKET_UDPSRV);
  return res;
}

// data buffers
static struct {
  struct {
    int32_t payload_size, response_size;
    uint8_t payload[BUFFER_SIZE-2*sizeof(int32_t)];
  } buf;
  uint32_t size;
} dbgsrv_in;

static struct {
  struct {
    int32_t response_size;
    uint8_t payload[BUFFER_SIZE-sizeof(int32_t)];
  } buf;
  int32_t expected_size, actual_size;
} dbgsrv_out;

enum {
  UDPSRV_RECV_ETH = 0,
  UDPSRV_WAIT_GETCMD,
  UDPSRV_WAIT_RESPONSE,
  UDPSRV_SEND_ETH
};
static int udpsrv_state;

static uint32_t last_client_ip = 0;
static uint16_t last_client_port = 0;
static int last_sstate;

void udpsrv_on_link_up(int sock)
{
  // reset the buffers
  dbgsrv_in.size = 0;
  dbgsrv_out.expected_size = 0;
  dbgsrv_out.actual_size = 0;
  // get the socket to the point where we can receive incoming connections
  last_client_ip = 0; last_client_port = 0;
  dprintf ("udpsrv: enabling\n");
  if (sock != socket(sock, Sn_MR_UDP, PORT_UDPSRV, 0x00)) {
    puts("udpsrv: cannot allocate socket!");
    return;
  }
  int sstate = getSn_SR(sock);
  last_sstate = sstate;
  if (sstate != SOCK_UDP) {
    printf("udpsrv: socket state wrong after initialization: %s!\n", ssstr(sstate));
    return;
  }
  printf("udpsrv: listening on port %u, socket #%u\n", PORT_UDPSRV, sock);
  udpsrv_state = UDPSRV_RECV_ETH;
}

void udpsrv_on_link_down(int sock)
{
  // close the socket
  close(sock);
  last_sstate = SOCK_CLOSED;
  puts("udpsrv: disabling");
}

void udpsrv_process(int sock, int sstate)
{
  // nothing to do if we're waiting for JTAG to pick up the commands, or to 
  // produce a response
  if ((udpsrv_state == UDPSRV_WAIT_GETCMD) ||
      (udpsrv_state == UDPSRV_WAIT_RESPONSE))
    return;

  // if the socket was closed, or not established, there's also nothing to do
  if (sstate == SOCK_CLOSED) {
    //if (last_sstate != SOCK_CLOSED)
    //  puts("udpsrv: socket closed");
    udpsrv_on_link_up(sock);
  }
  // if nothing connected, return
  if (sstate != SOCK_UDP) {
    last_sstate = sstate;
    return;
  }
  // -------------------------------------------------------------------------
  // if we're in the send phase
  if (udpsrv_state == UDPSRV_SEND_ETH) {
    // bail out if we don't have a response
    if (! dbgsrv_out.actual_size)
      return;
    int expected = dbgsrv_out.expected_size;
    int actual = dbgsrv_out.actual_size;
    int maxsize = expected < 0;
    if (maxsize)
      expected &= ~0x80000000;
    if (!actual || !expected)
      return;
    dprintf("udpsrv: send response, expected=%u, actual=%u\n");
    // if we have a maximum size specified, patch the out structure
    // - if we produced less data than the client would accept, limit the
    //   expectation
    // - if we produced more data than the client would accept, trim the data
    if (maxsize) {
      if (actual < expected)
        expected = actual;
      else if (actual > expected) {
        printf("udpsrv: trimming response (have %u, requested <%u)\n",
               actual, expected);
        actual = expected;
      }
      // patch the response
      dbgsrv_out.buf.response_size = actual - 4;
    }
    // check if the protocol is obeyed
    if (expected != actual) {
      printf("udpsrv: protocol error: expected response: %ubytes, have: %ubytes!\n",
             expected, actual);
    close_and_exit:
      close(sock);
      return;
    }
    if (actual != dbgsrv_out.buf.response_size + 4) {
      printf("udpsrv: internal error: response size set to: %ubytes, payload: %ubytes!\n",
             actual, dbgsrv_out.buf.response_size);
      goto close_and_exit;
    }
    // send the reply and resume waiting for data
    int transferred = sendto(sock, (uint8_t*)&dbgsrv_out.buf, actual, (uint8_t*)&last_client_ip, last_client_port);
    if (transferred != actual) {
      printf("udpsrv: send error (expected %d, actual %d).\n", actual, transferred);
      //goto close_and_exit;
    }
    dprintf("udpsrv: sent packet, payload size=%u\n",
            dbgsrv_out.buf.response_size);
    // reset the send pointer
    dbgsrv_out.expected_size = 0;
    udpsrv_state = UDPSRV_RECV_ETH;
  }

  // -------------------------------------------------------------------------
  // if we're in the receive phase
  if (udpsrv_state == UDPSRV_RECV_ETH) {
    // check if there's data to receive
    int expected = getSn_RX_RSR(sock);
    if (! expected)
      return;
    if (expected < 0) {
      puts ("udpsrv: read error...");
      goto close_and_exit;
    }
    if (expected > BUFFER_SIZE) {
      printf ("udpsrv: packet too large (size=%d, max=%u)...\n", expected, BUFFER_SIZE);
      goto close_and_exit;
    }
    uint8_t cliip[4]; uint16_t cliport;
    int actual = recvfrom(sock, (uint8_t*)&dbgsrv_in.buf, expected, cliip, &cliport);
    if (actual < 0) {
      puts ("udpsrv: receive error...");
      goto close_and_exit;
    }
    // there's fewer bytes than we expect - the RX_RSR tells us the entire package size, including
    // the header
    if (actual < expected - 8) {
      printf ("udpsrv: recvfrom error (expected=%d, got=%d)...\n", expected-8, actual);
      goto close_and_exit;
    }
    // if it's a new client, say so
    if ((last_client_ip != *(uint32_t*)cliip )||(last_client_port != cliport)) {
      last_client_ip = *(uint32_t*)cliip; last_client_port = cliport;
      printf("udpsrv: talking to %u.%u.%u.%u:%u\n",
             cliip[0], cliip[1], cliip[2], cliip[3], cliport);
    }
    // good stuff; see what the client expects
    if (actual != (dbgsrv_in.buf.payload_size + 8)) {
      printf ("udpsrv: protocol error "
              "(payload: claimed %d, actual %d; expected %s%d), "
              "disconnecting client...\n",
              dbgsrv_in.buf.payload_size, dbgsrv_in.size - 8,
              (dbgsrv_in.buf.response_size < 0)? "<" : "",
               dbgsrv_in.buf.response_size & ~0x80000000);
      goto close_and_exit;
    }
    dprintf("udpsrv: received packet, payload size=%u; "
            "expected response of size %s%u\n",
            dbgsrv_in.buf.payload_size,
            (dbgsrv_in.buf.response_size < 0)? "<" : "",
             dbgsrv_in.buf.response_size & ~0x80000000);
    // mark the buffer being ready to use
    dbgsrv_in.size = actual;
    dbgsrv_out.expected_size = dbgsrv_in.buf.response_size + 4;
    udpsrv_state = UDPSRV_WAIT_GETCMD;
  }
}

int udpsrv_fetch(char *dest)
{
  // if the state isn't WAIT_GETCMD, we don't have data
  if (udpsrv_state != UDPSRV_WAIT_GETCMD)
    return 0;
  int n = dbgsrv_in.size;
  if (n <= 4)
    return 0;
  n = dbgsrv_in.buf.payload_size;
  if (n > 0) {
    memcpy(dest, dbgsrv_in.buf.payload, n);
    dprintf ("udpsrv: issuing cmds, %u bytes\n", n);
  }
  // mark the buffer as available
  dbgsrv_in.size = 0;
  udpsrv_state = UDPSRV_WAIT_RESPONSE;
  return n;
}

int udpsrv_submit(const char *src, unsigned n)
{
  // if the state isn't WAIT_RESPONSE, we can't accept the data
  if (udpsrv_state != UDPSRV_WAIT_RESPONSE) {
    puts ("udpsrv: internal error, submitted response before receiving commands!");
    return 0;
  }
  dbgsrv_out.actual_size = 4 + n;
  dbgsrv_out.buf.response_size = n;
  if (n > 0) {
    memcpy(dbgsrv_out.buf.payload, src, n);
    dprintf ("jtag/udp: queueing resp, %u bytes\n", n);
  }
  udpsrv_state = UDPSRV_SEND_ETH;
  return n;
}

int is_udpsrv_receiving() {
  return (udpsrv_state == UDPSRV_RECV_ETH);
}

int is_udpsrv_sending() {
  return (udpsrv_state == UDPSRV_SEND_ETH);
}
