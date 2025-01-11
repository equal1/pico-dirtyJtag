/*
  Copyright (c) 2017 Jean THOMAS.
  
  Permission is hereby granted, free of charge, to any person obtaining
  a copy of this software and associated documentation files (the "Software"),
  to deal in the Software without restriction, including without limitation
  the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software
  is furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
  
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
  OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
  OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdint.h>

/**
 * @brief Handle a DirtyJTAG command
 *
 * @param usbd_dev USB device
 * @param buf Buffer index
 * @param cmdbuf Commands buffer
 * @param cmdsz Size of commands buffer
 * @param respbuf Response buffer
 * @return Number of produced response bytes
 */
unsigned cmd_execute(pio_jtag_inst_t* jtag, char buf, const uint8_t *cmdbuf, unsigned cmdsz, uint8_t *respbuf);

//-[ DirtyJTAG protocol, with extensions ]------------------------------------

enum CommandIdentifier {
  // standard DirtyJTAG2 commands
  CMD_STOP = 0x00,
  CMD_INFO = 0x01,
  CMD_FREQ = 0x02,
  CMD_XFER = 0x03, // opt: [7]=no_read, [6]=extend_length
  CMD_SETSIG = 0x04,
  CMD_GETSIG = 0x05,
  CMD_CLK = 0x06, // opt: [7]=do_read
# if 0
  CMD_SETVOLTAGE = 0x07, // not implemented
# endif
  CMD_REBOOT = 0x07, // use 7 for this instead
  CMD_GOTOBOOTLOADER = 0x08, // actually works!
# if 0
  CMD_REBOOT_OLD = 0x09, // DirtyJTAG extension - old ID of reboot command (rarely used)
# endif
  // equal1: get firmware capabilities
  CMD_GET_CAPS = 0x09,
  // equal1: get clocks configuration (jtagclk, a5clk)
  CMD_GETCLKS = 0x0a,
  // A5 clock control
  CMD_A5FREQ = 0x0b, // DirtyJTAG extension: set A5 clock frequency
  CMD_A5CLK = 0x0c,  // DirtyJTAG extension: enable/disable A5 clock; [7]=turn_on
  // GPIO configuration (both on Pico and IOX)
  CMD_PINCFG_SET_ALL = 0x0d, // DirtyJTAG extension: configure all pins
  CMD_PINCFG_SET = 0x0e, // DirtyJTAG extension: configure a specific pin (or all pins)
  CMD_PINCFG_GET = 0x0f, // DirtyJTAG extension: configure a specific pin
  // ADC commands
  CMD_ADC_GETREGS = 0x10, // DirtyJTAG extension: get all ADC registers
  CMD_ADC_CHREAD_MUX = 0x11, // DirtyJTAG extension: do a single read is MUX mode
  CMD_ADC_CHREAD_SCAN = 0x12, // DirtyJTAG extension: do a read on any channels combination using SCAN mode
  CMD_ADC_SETREG = 0x13, // DirtyJTAG extension: set a single ADC register
  CMD_ADC_SAMPLE = 0x14, // DirtyJTAG extension: kick off a single conversion and return result
};

enum CommandModifier
{
  // CMD_XFER
  NO_READ = 0x80,
  EXTEND_LENGTH = 0x40,
  // CMD_CLK
  READOUT = 0x80,
  // CMD_A5CLK
  TURN_ON = 0x80,
};

enum SignalIdentifier {
  SIG_TCK = 1 << 1,
  SIG_TDI = 1 << 2,
  SIG_TDO = 1 << 3,
  SIG_TMS = 1 << 4,
  SIG_TRST = 1 << 5,
  SIG_SRST = 1 << 6
};
