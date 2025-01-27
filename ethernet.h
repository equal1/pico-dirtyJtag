#pragma once

#include <stdint.h>
#include "config.h"

// socket allocation
#define SOCKET_TCPSRV 0
#define SOCKET_UDPSRV 1
#define SOCKET_FWUPD 2
#define SOCKET_DHCP 7

// ports
#define PORT_BASE 8901
#define PORT_DBGSRV PORT_BASE // use the same port number on both tcp and udp (they're different)
//#define PORT_TCPSRV PORT_DBGSRV
//#define PORT_UDPSRV PORT_DBGSRV
#define PORT_TFTP  69           //  firmware update port: tftp request port
#define PORT_FWUPD (PORT_BASE+1) // firmware update port: tftp data port


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
