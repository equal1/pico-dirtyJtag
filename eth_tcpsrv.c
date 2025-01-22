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

static void tcpsrv_on_link_up(int);
static void tcpsrv_on_link_down(int);
static void tcpsrv_process(int, int);

int tcpsrv_init()
{
  int res = eth_register_service(SOCKET_TCPSRV,
                                 tcpsrv_on_link_up, tcpsrv_on_link_down,
                                 tcpsrv_process);
  if (res)
    printf("tcpsrv: FAILED to register TCP server on port %u, socket #%u\n",
           PORT_TCPSRV, SOCKET_TCPSRV);
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
  TCPSRV_RECV_ETH = 0,
  TCPSRV_WAIT_GETCMD,
  TCPSRV_WAIT_RESPONSE,
  TCPSRV_SEND_ETH
};
static int tcpsrv_state;

int client_connected = 0;
static char client[] = "???.???.???.???:?????";
static int last_sstate;

void tcpsrv_on_link_up(int sock)
{
  // reset the buffers
  dbgsrv_in.size = 0;
  dbgsrv_out.expected_size = 0;
  dbgsrv_out.actual_size = 0;
  // get the socket to the point where we can receive incoming connections
  dprintf ("tcpsrv: enabling\n");
  if (sock != socket(sock, Sn_MR_TCP, PORT_TCPSRV, 0x00)) {
    puts("tcpsrv: cannot allocate socket!");
    return;
  }
  int sstate = getSn_SR(sock);
  last_sstate = sstate;
  if (sstate != SOCK_INIT) {
    puts("tcpsrv: socket state wrong after initialization!");
    return;
  }
  if(listen(sock) != SOCK_OK) {
    puts("tcpsrv: cannot listen on socket!");
    return;
  }
  client_connected = 0;
  strcpy(client, "?.?.?.?:?");
  printf("tcpsrv: listening on port %u, socket #%u\n", PORT_TCPSRV, sock);
  tcpsrv_state = TCPSRV_RECV_ETH;
}

void tcpsrv_on_link_down(int sock)
{
  // close the socket
  close(sock);
  client_connected = 0;
  last_sstate = SOCK_CLOSED;
  puts("tcpsrv: disabling");
}

static const char *sock_state(int s)
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
  default: sprintf (sstxt,"0x%02X", s); return sstxt;
  }
}

void tcpsrv_process(int sock, int sstate)
{
  // nothing to do if we're waiting for JTAG to pick up the commands, or to 
  // produce a response
  if ((tcpsrv_state == TCPSRV_WAIT_GETCMD) ||
      (tcpsrv_state == TCPSRV_WAIT_RESPONSE))
    return;

  // if the socket was closed, or not established, there's also nothing to do
  if (sstate == SOCK_CLOSED) {
    if (last_sstate != SOCK_CLOSED)
      puts("tcpsrv: disconnected client");
    tcpsrv_on_link_up(sock);
  }
  // if nothing connected, return
  if (sstate != SOCK_ESTABLISHED) {
    last_sstate = sstate;
    return;
  }
  // if we just connected, see to what
  if (last_sstate != SOCK_ESTABLISHED) {
    if (getSn_IR(sock) & Sn_IR_CON) {
      uint8_t cliip[4];
      uint16_t cliport;
      getSn_DIPR(sock, cliip);
      cliport = getSn_DPORT(sock);
      setSn_IR(sock, Sn_IR_CON);
      sprintf(client, "%u.%u.%u.%u:%u",
              cliip[0], cliip[1], cliip[2], cliip[3], cliport);
      printf ("tcpsrv: accepted connection from %s\n", client);
    }
  }
  last_sstate = sstate;

  // -------------------------------------------------------------------------
  // if we're in the send phase
  if (tcpsrv_state == TCPSRV_SEND_ETH) {
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
    dprintf("tcpsrv: send response, expected=%u, actual=%u\n");
    // if we have a maximum size specified, patch the out structure
    // - if we produced less data than the client would accept, limit the
    //   expectation
    // - if we produced more data than the client would accept, trim the data
    if (maxsize) {
      if (actual < expected)
        expected = actual;
      else if (actual > expected) {
        printf("tcpsrv: trimming response (have %u, requested <%u)\n",
               actual, expected);
        actual = expected;
      }
      // patch the response
      dbgsrv_out.buf.response_size = actual - 4;
    }
    // check if the protocol is obeyed
    if (expected != actual) {
      printf("tcpsrv: protocol error: "
             "expected response: %ubytes, have: %ubytes; "
             "disconnecting client...\n",
             expected, actual);
    close_and_exit:
      close(sock);
      return;
    }
    if (actual != dbgsrv_out.buf.response_size + 4) {
      printf("tcpsrv: internal error: "
             "response size set to: %ubytes, payload: %ubytes; "
             "disconnecting client...\n",
             actual, dbgsrv_out.buf.response_size);
      goto close_and_exit;
    }
    // send the reply and resume waiting for data
    int transferred = send(sock, (uint8_t*)&dbgsrv_out.buf, actual);
    if (transferred != actual) {
      puts("tcpsrv: send error, disconnecting client...");
      goto close_and_exit;
    }
    dprintf("tcpsrv: sent packet, payload size=%u\n",
            dbgsrv_out.buf.response_size);
    // reset the send pointer
    dbgsrv_out.expected_size = 0;
    tcpsrv_state = TCPSRV_RECV_ETH;
  }

  // -------------------------------------------------------------------------
  // if we're in the receive phase
  if (tcpsrv_state == TCPSRV_RECV_ETH) {
    // check if there's data to receive
    int expected = getSn_RX_RSR(sock);
    if (! expected)
      return;
    if (expected < 0) {
      puts ("tcpsrv: read error, disconnecting client...");
      goto close_and_exit;
    }
    if (expected > BUFFER_SIZE) {
      printf ("tcpsrv: packet too large (size=%d, max=%u), ",
              "disconnecting client...\n", expected, BUFFER_SIZE);
      goto close_and_exit;
    }
    int actual = recv(sock, (uint8_t*)&dbgsrv_in.buf, expected);
    if (actual < 0) {
      puts ("tcpsrv: receive error, disconnecting client...");
      goto close_and_exit;
    }
    if (expected != actual) {
      printf ("tcpsrv: read error (expected=%d, got=%d), "
              "disconnecting client...\n", expected, actual);
      goto close_and_exit;
    }
    // good stuff; see what the client expects
    if (actual != (dbgsrv_in.buf.payload_size + 8)) {
      printf ("tcpsrv: protocol error "
              "(payload: claimed %d, actual %d; expected %s%d), "
              "disconnecting client...\n",
              dbgsrv_in.buf.payload_size, dbgsrv_in.size - 8,
              (dbgsrv_in.buf.response_size < 0)? "<" : "",
               dbgsrv_in.buf.response_size & ~0x80000000);
      goto close_and_exit;
    }
    dprintf("tcpsrv: received packet, payload size=%u; "
            "expected response of size %s%u\n",
            dbgsrv_in.buf.payload_size,
            (dbgsrv_in.buf.response_size < 0)? "<" : "",
             dbgsrv_in.buf.response_size & ~0x80000000);
    // mark the buffer being ready to use
    dbgsrv_in.size = actual;
    dbgsrv_out.expected_size = dbgsrv_in.buf.response_size + 4;
    tcpsrv_state = TCPSRV_WAIT_GETCMD;
  }
}

int tcpsrv_fetch(char *dest)
{
  // if the state isn't WAIT_GETCMD, we don't have data
  if (tcpsrv_state != TCPSRV_WAIT_GETCMD)
    return 0;
  int n = dbgsrv_in.size;
  if (n <= 4)
    return 0;
  n = dbgsrv_in.buf.payload_size;
  if (n > 0) {
    memcpy(dest, dbgsrv_in.buf.payload, n);
    dprintf ("tcpsrv: issuing cmds, %u bytes\n", n);
  }
  // mark the buffer as available
  dbgsrv_in.size = 0;
  tcpsrv_state = TCPSRV_WAIT_RESPONSE;
  return n;
}

int tcpsrv_submit(const char *src, unsigned n)
{
  // if the state isn't WAIT_RESPONSE, we can't accept the data
  if (tcpsrv_state != TCPSRV_WAIT_RESPONSE) {
    puts ("tcpsrv: internal error, submitted response before receiving commands!");
    return 0;
  }
  dbgsrv_out.actual_size = 4 + n;
  dbgsrv_out.buf.response_size = n;
  if (n > 0) {
    memcpy(dbgsrv_out.buf.payload, src, n);
    dprintf ("jtag/tcp: queueing resp, %u bytes\n", n);
  }
  tcpsrv_state = TCPSRV_SEND_ETH;
  return n;
}

int is_tcpsrv_receiving() {
  return (tcpsrv_state == TCPSRV_RECV_ETH);
}

int is_tcpsrv_sending() {
  return (tcpsrv_state == TCPSRV_SEND_ETH);
}
