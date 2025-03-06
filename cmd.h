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
#include "pio_jtag.h"

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
  // equal1 accelerated commands
  // - general purpose
  XCMD_BYPASS_COUNT = 0x15,
  XCMD_GET_IDCODES = 0x16,
  // - target config (xRSCAN, ARM commands, CPU and SYS default APs)
  XCMD_CONFIG = 0x17,
  XCMD_IRSCAN = 0x18,
  XCMD_DRSCAN = 0x19,
  // - DPACC, APACC and ABORT
  XCMD_ABORT = 0x1A,
  XCMD_DPACC_RD = 0x1B,
  XCMD_DPACC_WR = 0x1C,
  XCMD_APACC_RD = 0x1D,
  XCMD_APACC_WR = 0x1E,
  // - bus (SYS) and CPU accesses
  XCMD_BUS_RD = 0x1F,
  XCMD_BUS_WR = 0x20,
  XCMD_CPU_RD = 0x21,
  XCMD_CPU_WR = 0x22,
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
  // SIG_TRST = 1 << 5,
  SIG_SRST = 1 << 6
};

// the structure that gets passed by GETCLK (defined in pio_jtag.h)
struct djtag_clk_s;

// configure the target device; takes in the structure
struct djtag_cfg_s { 
  // for xRSCAN
  struct {
     uint8_t lead1s, tail1s;
  } ir, dr; 
  // - bitmaps of the ARM commands
  uint32_t dpacc, apacc, abort;
  // for the ARM commands
  // - size in bits of the IR
  uint8_t ir_size;
  // - number of extra clock cycles to apply in RTI after the command
  ///  was sent
  uint8_t armcmd_xcycles;
  // for fast accesses - index of the sys and cpu APs
  uint8_t sys_ap, cpu_ap;
};

// capabilities: available commands

// count devices in the JTAG chain, using the BYPASS method
//   returns 0 for no devices, FF for chain disconnected, or the
//   number of devices (scans ups to 64 devices)
#define CAP_BYPASS_COUNT 0x00000001
// get the IDCODES; returns a single 00 if none found, FF for chain
//   disconnected, or the list of devices in the chain
#define CAP_GET_IDCODES  0x00000002

#define CAP_CONFIG  0x00000004

// perform scans; same format and arguments as CMD_XFER (they honor NO_READ
//   and EXTEND_LENGTH), /but/ data doesn't have to be bit-swapped on either input
//   or output
// these scans leave the JTAG FSM in RTI
#define CAP_SCAN  0x00000008

// ARM-specific commands

// these use the codes as configured with XCMD_CONFIG, take in 2 words
// (address, data), plus 1 byhe status bits
int jtag_apacc_rd(int ap, uint32_t addr, uint32_t *data);
int jtag_apacc_wr(int ap, uint32_t addr, uint32_t data);

// addr[0] is the R/W# bit
// addr[1] is is the extra-idle-clocks enable bit
// without NO_READ, they return the result word, followed by the status byte ([2:0]);
// with NO_READ, only the status byte is returned
// 
#define CAP_ABORT   0x00000010
#define CAP_DPACC   0x00000020
#define CAP_APACC   0x00000040

// perform bus accesses on a specified AP, as configured with XCMD_CONFIG
// these are automatically retried on wait, aborted on 2nd wait

// read takes in the 1-byte AP index, and the 1-word target address;
// returns either 1 byte on error, or 4 bytes (the result) on success
#define CAP_READ_AP  0x00000080
// write takes in the 1-byte AP index, and the 1-word target address;
// returns the 1 byte status byte
#define CAP_WRITE_AP 0x00000100

// perform bus accesses on the SYS AP
// identical to CAP_{READ_WRITE}_AP, except they use the SYSAP configured
// with XCMD_CONFIG

#define CAP_SYS_RD  0x00000200
#define CAP_SYS_WR  0x00000400

// perform bus accesses on the CPU AP
// identical to CAP_{READ_WRITE}_AP, except they use the CPUAP configured
// with XCMD_CONFIG

#define CAP_CPU_RD  0x00000800
#define CAP_CPU_WR  0x00001000


//=[ jtagx api ]===============================================================

// set default CPU/SYS APs based on what TILESEL points 
// this overrides the most recent XCMD_SET_CONFIG, but only when TILESEL is
// actually changed (via PINCFG_SET)
void jcfg_set_tilesel(int tile);

// count devices in the JTAG chain, using the BYPASS method
//   returns 0 for no devices, FF for chain disconnected, or the
//   number of devices
unsigned jtag_do_bypass();

// get the IDCODEs in the chain
//   returns 0xFF is TDO is stuck at zero, or the number of devices (scans ups
//   to 16 (=512/32) devices)
unsigned jtag_get_idcodes(uint32_t *idcode);

// load the jcfg structure from the client
//   returns the size of the jtag structure
unsigned jtag_set_config(const void *cfg);

// perform a JTAG scan
// this is very similar to CMD_XFER, except
// - it also handles the last bit
// - the JTAG FSM will be left in Run-Test-Idle
// - bitswapping is performed (the client needn't do it)
// for DR scans _only_, bits [7:1] of ir represents the number of extra cycles
//   to stay in RTI after the scan (the XCMD interface does not expose this
//   however)
// a null OUT causes that number of 1s to be scanned out
// a null IN discards the scan result (also, the result value is set to zero)
// returns the number of input _bytes_
unsigned jtag_do_scan(int ir, unsigned n_bits, const void *out, void *in);

// ABORT
int jtag_abort(int arg);
// DPACC reads and writes
// they return the status bits
int jtag_dpacc_rd(uint32_t addr, uint32_t *data);
int jtag_dpacc_wr(uint32_t addr, uint32_t data);
// APACC reads and writes
// they return the status bits
int jtag_apacc_rd(int ap, uint32_t addr, uint32_t *data);
int jtag_apacc_wr(int ap, uint32_t addr, uint32_t data);

// BYPASS_COUNT and GET_IDCODES ignore config
// IRSCAN, DRSCAN use only config.{ir|dr}.{lead1s|tail1s}
//   (in particular, IRSCAN ignores config.ir_size)
// ABORT, {DP|AP}ACC_{RD|WR} addtionally use config.ir_size and config.{abort|dpacc|apacc}
// APACC_{RD|WR} addtionally use config.armcmd_xcycles
// {BUS|CPU}_{RD|WR} target the 

#define IMPLEMENTED_CAPS ( \
  CAP_BYPASS_COUNT | CAP_GET_IDCODES | \
  CAP_CONFIG | \
  CAP_SCAN | \
  CAP_ABORT | CAP_DPACC | CAP_APACC | \
  0)

// CAP_SCAN: IRSCAN, DRSCAN implemented
// CAP_ABORT: ABORT implemented
// CAP_DPACC: DPACC_RD, DPACC_WR implemented
// CAP_APACC: APACC_RD, APACC_WR implemented
