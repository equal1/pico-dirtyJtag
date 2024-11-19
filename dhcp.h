// override hostname
#pragma once

#include "pico-w5500/Internet/DHCP/dhcp.h"

#undef DCHP_HOST_NAME
#define DCHP_HOST_NAME           "pico-??????????\0"

extern uint8_t HOST_NAME[];
