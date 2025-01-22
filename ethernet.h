#pragma once

#include "config.h"

// socket allocation
#define SOCKET_TCPSRV 0
#define SOCKET_UPDSRV 1
#define SOCKET_FWUPD 2
#define SOCKET_DHCP 7

// ports
#define PORT_BASE 8901
#define PORT_TCPSRV PORT_BASE
#define PORT_UDPSRV (PORT_BASE+1)
#define PORT_FWUPD  (PORT_BASE+2)


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

// initialize the TCP server
int tcpsrv_init();
int tcpsrv_fetch(char*);
int tcpsrv_submit(const char*, unsigned);
int is_tcpsrv_receiving();
int is_tcpsrv_sending();

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

#ifdef W5500_USE_BLOCK
#ifdef W5500_USE_BLOCK_DMA
# define W5500_MODE "DMA-enabled SPI bursts"
#else
# define W5500_MODE "simple SPI bursts"
# endif
# else
# define W5500_MODE "simple SPI accesses"
#endif
