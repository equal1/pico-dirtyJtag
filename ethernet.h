#pragma once

#include <stdint.h>
#include "config.h"

// socket allocation
#define SOCKET_TCPSRV 0
#define SOCKET_UDPSRV 1
#define SOCKET_FWUPD 2
#define SOCKET_DHCP 7

// ports
#define PORT_JTAG 8901
#define PORT_DBGSRV PORT_JTAG // use the same port number on both tcp and udp (they're different)
#define PORT_TFTP  69    // firmware update port: using tftp

// declare the pins
void eth_pins_init();

// initialize the ethernet
int eth_init(uint64_t board_id);

// main ethernet_task
void eth_task();

struct dbgsvc_s;

// register socket callbacks
typedef void (*link_callback_t)(int socket, struct dbgsvc_s*);
typedef void (*socket_callback_t)(int socket, int sock_state, struct dbgsvc_s*);

int eth_register_service(int socket, struct dbgsvc_s *svc,
                        link_callback_t on_link_up,
                        link_callback_t on_link_down,
                        socket_callback_t process);

// is ethernet physically present?
// -1: unknown; 0: nope; 1: 
int board_has_ethernet();

// int is_ethernet_connected(); // [unused]
// int eth_error(); // [unused]

// the TCP server service (e1mon)
int tcpsrv_init();
int tcpsrv_fetch(char*);
int tcpsrv_submit(const char*, unsigned);
int is_tcpsrv_receiving();
int is_tcpsrv_sending();

// the UDP server service (djcli)
int udpsrv_init();
int udpsrv_fetch(char*);
int udpsrv_submit(const char*, unsigned);
int is_udpsrv_receiving();
int is_udpsrv_sending();

// firmware updated service (tftp)
int fwupd_init();

// read string values
enum {
  ETHSTR_ERROR = -1,
  ETHSTR_CHIP = 0,
  ETHSTR_SPIFREQ,
  ETHSTR_MAC,
  ETHSTR_HOSTNAME,
  ETHSTR_IP,
  ETHSTR_NETWORK,
  ETHSTR_GATEWAY,
  ETHSTR_SUBNET,
  ETHSTR_DNS,
  ETHSTR_TCPCLI
};
const char *ethstr(int);
const char *ssstr(int); // explain socket states

// service-describing structure
struct dbgsvc_s {
  int is_tcp; // whether the service provides TCP or UDP
  char svcname[8]; // service name in prints
  int last_sstate; // most recent socket state
  // client
  // - TCP: set while connected
  // - UDP: most recently seen client
  union {
    uint8_t  byte[4];
    uint32_t ip;
  } cli_ip;
  uint16_t cli_port;
  // curent service state
  enum ethsvc_state_e {
    SVC_NO_LINK = 0,   // network not connected
    SVC_WAIT_CMDS,     // wait for a request
    SVC_WAIT_GETCMD,   // got a request, waiting for the main loop to pick it up
    SVC_WAIT_RESPONSE, // request passed on, waiting for response
    SVC_DO_SEND        // got the response - send the reply at earliest opportunity
  } state;
  // tcp-only: callback for setting the client
  void (*tcp_set_cli)(const uint8_t*, uint16_t);
  // the data buffer structures for Ethernet
  struct dbgsrv_buf_in_s {
    // transported data
    struct {
      int32_t payload_size, response_size;
      uint8_t payload[BUFFER_SIZE-2*sizeof(int32_t)];
    } buf;
    // state data
    uint32_t size;
  } in;
  struct dbgsrv_buf_out_s {
    // transported data
    struct {
      int32_t response_size;
      uint8_t payload[BUFFER_SIZE-sizeof(int32_t)];
    } buf;
    // state data
    int32_t expected_size, actual_size;
  } out;
};

#ifdef W5500_USE_BLOCK
#ifdef W5500_USE_BLOCK_DMA
# define W5500_MODE "DMA-enabled SPI bursts"
#else
# define W5500_MODE "simple SPI bursts"
# endif
# else
# define W5500_MODE "simple SPI accesses"
#endif

//=============================================================================
//=[ TFTP protocol packets ]===================================================
//=============================================================================

#define TFTP_RRQ   1
#define TFTP_WRQ   2
#define TFTP_DATA  3
#define TFTP_ACK   4 // acknowledged, minus options
#define TFTP_ERROR 5
#define TFTP_OACK  6 // option acknowledge

#define TFTP_ERR_GENERIC  0 // not specified (see error message)
#define TFTP_ERR_NOFILE   1 // file not found
#define TFTP_ERR_ACCESS   2 // access violation
#define TFTP_ERR_DISKFULL 3 // disk full / allocation exceeded
#define TFTP_ERR_PROTOCOL 4 // illegal TFTP operation
#define TFTP_ERR_BADID    5 // unknown tranfer ID
#define TFTP_ERR_EXISTS   6 // file already exists
#define TFTP_ERR_NOUSER   7 // no such user
#define TFTP_ERR_BADOPT   8 // bad option


// RRQ/WRQ structure:
//   <opcode:int16> <filename:asciiz> <mode:asciiz> [{<optname:asciiz> <optval:asciiz>}...]
//     case insensitive:
//     mode := { "netascii", "octet", "mail" }
//     optname := { "blksize", "timeout", "tsize" }
// DATA structure:
//   [opcode:int16] [block#:int16] [data]
// ACK structure:
//   [opcode:int16] [block#:int16]
// ERROR structure:
//   [opcode:int16] [errcode:int16] [msg:asciiz]

// blksize: default 512
// timeout: default ???
// tsize: (transfer size: opt)
