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
#define PORT_TCPSRV PORT_BASE
#define PORT_UDPSRV PORT_BASE // same port as the TCP one, except this one is UDP
#define PORT_FWUPD_TFTP  69           // firmware update port: tftp request port
#define PORT_FWUPD_DATA (PORT_BASE+1) // firmware update port: tftp data port


// declare the pins
void eth_pins_init();

// initialize the ethernet
int eth_init(uint64_t board_id);

// main ethernet_task
void eth_task();

// register socket callbacks
typedef void (*link_callback_t)(int socket);
typedef void (*socket_callback_t)(int socket, int sock_state);

int eth_register_service(int socket, link_callback_t on_link_up, 
                                     link_callback_t on_link_down,
                                     socket_callback_t process);

// is ethernet physically present?
// -1: unknown; 0: nope; 1: 
int board_has_ethernet();

// is ethernet connected?
int is_ethernet_connected();

// are we having an error
int eth_error();

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
};
const char *ethstr(int);
const char *ssstr(int); // explain socket states

// the data buffer structures for Ethernet
struct dbgsrv_buf_in_s {
  struct {
    int32_t payload_size, response_size;
    uint8_t payload[BUFFER_SIZE-2*sizeof(int32_t)];
  } buf; // transported data
  uint32_t size; // state data
};
struct dbgsrv_buf_out_s {
  struct {
    int32_t response_size;
    uint8_t payload[BUFFER_SIZE-sizeof(int32_t)];
  } buf; // transported data
  int32_t expected_size, actual_size; // state data
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
