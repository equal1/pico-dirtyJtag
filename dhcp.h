#pragma once

#include <stdint.h>
#include "pico-w5500/Internet/DHCP/dhcp.h"

// override hostname: we want the whole Pico UID in the name
// for this to work, pico-w5500.gitdiff needs to be applied to the pico-w5500
// external
// the patch itself is trivial - replaces
//   #include "dhcp.h"
// with
//   #include <dhcp.h>
// thus overriding which dhcp.h gets included

#undef DCHP_HOST_NAME
// the stock WizChip code automatically adds the LS 24 bits of the MAC
//   to this, so only reserve space for the topmost 40 bits of the Pico UID
#define DCHP_HOST_NAME           "pico-??????????\0"

extern uint8_t HOST_NAME[];
