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

static void dbgsrv_on_link_up(int, struct dbgsvc_s*);
static void dbgsrv_on_link_down(int, struct dbgsvc_s*);
static void dbgsrv_process(int, int, struct dbgsvc_s*);

// data buffers
static struct dbgsvc_s tcp, udp;

//=[ services init ]=========================================================

int tcpsrv_init()
{
  tcp.is_tcp = 1;
  strcpy(tcp.svcname, "tcpsrv");
  tcp.state = SVC_NO_LINK;
  int res = eth_register_service(SOCKET_TCPSRV, &tcp,
                                 dbgsrv_on_link_up, dbgsrv_on_link_down,
                                 dbgsrv_process);
  if (res)
    printf("%s: FAILED to register service on port %u, socket #%u\n",
           tcp.svcname, PORT_DBGSRV, SOCKET_TCPSRV);
  return res;
}

int udpsrv_init()
{
  udp.is_tcp = 0;
  strcpy(udp.svcname, "udpsrv");
  udp.state = SVC_NO_LINK;
  int res = eth_register_service(SOCKET_UDPSRV, &udp,
                                 dbgsrv_on_link_up, dbgsrv_on_link_down,
                                 dbgsrv_process);
  if (res)
    printf("%s: FAILED to register service on port %u, socket #%u\n",
           tcp.svcname, PORT_DBGSRV, SOCKET_UDPSRV);
  return res;
}

//=[ link up/down handlers ]=================================================

void dbgsrv_on_link_up(int sock, struct dbgsvc_s *svc)
{
  dprintf("dbgsrv_on_link_up(%u,%s@%p)\n", sock, (svc==&tcp)?"tcp":(svc==&udp)?"udp":"???", svc);
  // reset the state
  svc->in.size = 0;
  svc->out.expected_size = 0;
  svc->out.actual_size = 0;
  // reset the (tcp: connected; udp: most recently seen) client
  svc->cli_ip.ip = 0; svc->cli_port = 0;
  // TCP only: update the currently connected client
  // (mostly for updating whoami data)
  if (svc->is_tcp)
    svc->tcp_set_cli(0, 0);
  // enter the wait_for_commands phase
  svc->state = SVC_WAIT_CMDS;
  // get the socket to the point where we can receive incoming connections
  dprintf("%s: enabling\n", svc->svcname);
  int s, exp_ss;
  s = socket(sock, svc->is_tcp ? Sn_MR_TCP : Sn_MR_UDP, PORT_DBGSRV, 0x00);
  exp_ss = svc->is_tcp ? SOCK_INIT : SOCK_UDP;
  if (s != sock) {
    printf("%s: cannot allocate socket #%u!\n", svc->svcname, sock);
    return;
  }
  int sstate = getSn_SR(sock);
  svc->last_sstate = sstate;
  if (sstate != exp_ss) {
    char ss[40];
    // just in case - ssstr only has 1 buffer, if the answer isn't canned for both
    strcpy (ss, ssstr(sstate));
    printf("%s: socket state wrong after initialization (%s), expected %s!\n",
           svc->svcname, ss, ssstr(exp_ss));
    return;
  }
  // for TCP: listen on the socket
  if (svc->is_tcp && (listen(sock) != SOCK_OK)) {
    printf("%s: cannot listen on socket!\n", svc->svcname);
    return;
  }
  printf("%s: listening on port %.3s/%u, socket #%u\n",
         svc->svcname, svc->svcname, PORT_DBGSRV, sock);
}

void dbgsrv_on_link_down(int sock, struct dbgsvc_s *svc)
{
  dprintf("dbgsrv_on_link_down(%u,%s@%p)\n", sock, (svc==&tcp)?"tcp":(svc==&udp)?"udp":"???", svc);
  // set the state to "no link"; the rest of the state will get
  // reset when the link goes up
  svc->state = SVC_NO_LINK;
  // close the socket
  close(sock);
  printf("%s: disabling\n", svc->svcname);
}

//=[ main service loop ]=====================================================

void dbgsrv_process(int sock, int sstate, struct dbgsvc_s *svc)
{
  // nothing to do if we're waiting for JTAG to pick up the commands, or to 
  // produce a response
  if ((svc->state == SVC_WAIT_GETCMD) ||
      (svc->state == SVC_WAIT_RESPONSE))
    return;

  // if the socket was closed, or not established, there's also nothing to do
  if (sstate == SOCK_CLOSED) {
    // if this was TCP, say that the client disconnected
    if (svc->is_tcp && (svc->last_sstate != SOCK_CLOSED))
      printf("%s: disconnected client\n", svc->svcname);
    dbgsrv_on_link_up(sock, svc);
  }
  // TCP only:
  if (svc->is_tcp) {
    // - if nothing connected, return;
    if (sstate != SOCK_ESTABLISHED) {
      svc->last_sstate = sstate;
      return;
    }
    // - if we just connected, say so
    if (svc->last_sstate != SOCK_ESTABLISHED) {
      if (getSn_IR(sock) & Sn_IR_CON) {
        uint8_t cliip[4];
        uint16_t cliport;
        getSn_DIPR(sock, cliip);
        cliport = getSn_DPORT(sock);
        setSn_IR(sock, Sn_IR_CON);
        svc->tcp_set_cli(cliip, cliport);
        printf("%s: accepted connection from %s\n", svc->svcname, ethstr(ETHSTR_TCPCLI));
      }
    }
  }
  svc->last_sstate = sstate;

  // -------------------------------------------------------------------------
  // if we're in the send-reply phase
  if (svc->state == SVC_DO_SEND) {
    // bail out if we don't have a response
    if (! svc->out.actual_size)
      return;
    int expected = svc->out.expected_size;
    int actual = svc->out.actual_size;
    int maxsize = expected < 0;
    if (maxsize)
      expected &= ~0x80000000;
    if ((! actual) || (! expected))
      return;
    dprintf("%s: send response, expected=%u, actual=%u\n", svc->svcname, expected, actual);
    // if we have a maximum size specified, patch the out structure
    // - if we produced less data than the client would accept, limit the
    //   expectation
    // - if we produced more data than the client would accept, trim the data
    if (maxsize) {
      if (actual < expected)
        expected = actual;
      else if (actual > expected) {
        printf("%s: trimming response (have %u, requested <%u)\n",
               svc->svcname, actual, expected);
        actual = expected;
      }
      // patch the response
      svc->out.buf.response_size = actual - 4;
    }
    // check if the protocol is obeyed
    if (expected != actual) {
      printf("%s: protocol error: "
             "expected response: %ubytes, have: %ubytes%s...\n",
             svc->svcname, expected, actual,
             svc->is_tcp ? "; disconnecting client" : "");
      // reboot if the protocol was badly violated
      if ((expected > BUFFER_SIZE) || (actual > BUFFER_SIZE))
        bad_error();
    close_and_exit:
      // close the socket, even if UDP (force a reconnect on next iteration)
      close(sock);
      return;
    }
    if (actual != svc->out.buf.response_size + 4) {
      printf("%s: internal error: "
             "response size set to: %ubytes, payload: %ubytes%s...\n",
             svc->svcname, actual, svc->out.buf.response_size,
             svc->is_tcp ? "; disconnecting client" : "");
      goto close_and_exit;
    }
    // send the reply and resume waiting for data
    int transferred;
    if (svc->is_tcp)
      transferred = send(sock, (uint8_t*)&svc->out.buf, actual);
    else
      // note: cli_ip and cli_port only change during the SVC_RECV_ETH state,
      // so in SVC_SEND_ETH state, they reflect the IP/port from which we got the most
      // recent command
      transferred = sendto(sock, (uint8_t*)&svc->out.buf, actual, svc->cli_ip.byte, svc->cli_port);
    if (transferred != actual) {
      printf("%s: send error%s...\n",
             svc->svcname, svc->is_tcp ? "; disconnecting client" : "");
      goto close_and_exit;
    }
    dprintf("%s: sent packet, payload size=%u\n",
            svc->svcname, svc->out.buf.response_size);
    // reset the send pointer
    svc->out.expected_size = 0;
    svc->state = SVC_WAIT_CMDS;
    // fall-through, just in case there's already some inbound received data
  }

  // -------------------------------------------------------------------------
  // if we're in the receive commands
  if (svc->state == SVC_WAIT_CMDS) {
    // check if there's data to receive
    int expected = getSn_RX_RSR(sock);
    if (! expected)
      return;
    // packets received over UDP have an 8-byte header, apparently
    // that's probably {
    //   uint16_t src_port, dst_port, len, cksum; // udp_hdr @0, @2, @4, @6
    // }
    if (! svc->is_tcp) {
      expected -= 8;
      if (! expected)
        return;
    }
    if (expected < 0) {
      printf("%s: read error%s...\n",
             svc->svcname, svc->is_tcp ? "; disconnecting client" : "");
      goto close_and_exit;
    }
    if (expected > BUFFER_SIZE) {
      printf("%s: packet too large (size=%d, max=%u)%s...\n",
              "disconnecting client...\n", expected, BUFFER_SIZE);
      goto close_and_exit;
    }
    int actual;
    // for UDP, we also get the IP and port of the sender
    uint32_t cli_ip; uint16_t cli_port;
    if (svc->is_tcp)
      actual = recv(sock, (uint8_t*)&svc->in.buf, expected);
    else
      actual = recvfrom(sock, (uint8_t*)&svc->in.buf, expected, (uint8_t*)&cli_ip, &cli_port);
    if (actual < 0) {
      printf("%s: receive error%s...\n",
             svc->svcname, svc->is_tcp ? "; disconnecting client" : "");
      goto close_and_exit;
    }
    if (expected != actual) {
      printf("%s: read error (expected=%d, got=%d)%s...\n",
             svc->svcname, expected, actual,
             svc->is_tcp ? "; disconnecting client" : "");
      goto close_and_exit;
    }
    // UDP only: if this is a different client from the last one we've seen, say so
    if (! svc->is_tcp) {
      if ((svc->cli_ip.ip != cli_ip )||(svc->cli_port != cli_port)) {
        svc->cli_ip.ip = cli_ip;
        svc->cli_port = cli_port;
        printf("%s: talking to %u.%u.%u.%u:%u\n",
               svc->svcname,
               svc->cli_ip.byte[0], svc->cli_ip.byte[1], svc->cli_ip.byte[2], svc->cli_ip.byte[3],
               cli_port);
      }
    }
    // good stuff; see what the client expects
    if (actual != (svc->in.buf.payload_size + 8)) {
      printf("%s: protocol error "
              "(payload: claimed %d, actual %d; expected %s%d)%s...\n",
              svc->svcname,
              svc->in.buf.payload_size, actual - 8,
              (svc->in.buf.response_size < 0)? "<=" : "",
               svc->in.buf.response_size & ~0x80000000,
              svc->is_tcp ? "; disconnecting client" : "");
      // reboot if the protocol was badly violated
      if ((svc->in.buf.payload_size > BUFFER_SIZE) || ((svc->in.buf.response_size & ~0x80000000) > BUFFER_SIZE))
        bad_error();
      goto close_and_exit;
    }
    dprintf("%s: received packet, payload size=%u; "
            "expected response of size %s%u\n",
            svc->svcname, 
            svc->in.buf.payload_size,
            (svc->in.buf.response_size < 0)? "<=" : "",
             svc->in.buf.response_size & ~0x80000000);
    // mark the buffer being ready to use
    svc->in.size = actual;
    svc->out.expected_size = svc->in.buf.response_size + 4;
    svc->state = SVC_WAIT_GETCMD;
  }
}

//=[ commands request ]======================================================

static int dbgsrv_fetch(struct dbgsvc_s *svc, char *dest)
{
  // if the state isn't WAIT_GETCMD, we don't have data
  if (svc->state != SVC_WAIT_GETCMD)
    return 0;
  int n = svc->in.size;
  if (n <= 4)
    return 0;
  n = svc->in.buf.payload_size;
  if (n > 0) {
    memcpy(dest, svc->in.buf.payload, n);
    dprintf("%s: issuing cmds, %u bytes\n", svc->svcname, n);
  }
  // mark the buffer as available
  svc->in.size = 0;
  svc->state = SVC_WAIT_RESPONSE;
  return n;
}

int tcpsrv_fetch(char *dest)
{
  return dbgsrv_fetch(&tcp, dest);
}
int udpsrv_fetch(char *dest)
{
  return dbgsrv_fetch(&udp, dest);
}

//=[ reply sending ]=========================================================

static int dbgsrv_submit(struct dbgsvc_s *svc, const char *src, unsigned n)
{
  // if the state isn't WAIT_RESPONSE, we can't accept the data
  if (svc->state != SVC_WAIT_RESPONSE) {
    printf("%s: internal error, submitted response before receiving commands!\n",
           svc->svcname);
    return 0;
  }
  svc->out.actual_size = 4 + n;
  svc->out.buf.response_size = n;
  if (n > 0) {
    memcpy(svc->out.buf.payload, src, n);
    dprintf("jtag/%.3s: queueing resp, %u bytes\n", 
            svc->svcname, n);
  }
  svc->state = SVC_DO_SEND;
  return n;
}

int tcpsrv_submit(const char *src, unsigned n)
{
  return dbgsrv_submit(&tcp, src, n);
}
int udpsrv_submit(const char *src, unsigned n)
{
  return dbgsrv_submit(&udp, src, n);
}

//=[ state checking ]========================================================

static int is_dbgsrv_receiving(struct dbgsvc_s *svc)
{
  return (svc->state == SVC_WAIT_CMDS);
}

static int is_dbgsrv_sending(struct dbgsvc_s *svc)
{
  return (svc->state == SVC_DO_SEND);
}

int is_tcpsrv_receiving()
{
  return is_dbgsrv_receiving(&tcp);
}
int is_udpsrv_receiving()
{
  return is_dbgsrv_receiving(&udp);
}

int is_tcpsrv_sending()
{
  return is_dbgsrv_sending(&tcp);
}
int is_udpsrv_sending()
{
  return is_dbgsrv_sending(&udp);
}
