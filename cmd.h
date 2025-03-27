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
  // equal1 JTAG accelerated commands
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
  XCMD_CPU_WRRD = 0x23, // write and immediately read the register
  // - bus (SYS) block accesses
  XCMD_BLOCK_RD   = 0x24,
  XCMD_BLOCK_WR   = 0x25,
  XCMD_BLOCK_FILL = 0x26,
  // equal1 ARM/m0core accelerated commands
  XCMD_ARM_INIT = 0x27,
  XCMD_ARM_RESUME = 0x28,
  XCMD_ARM_QUERY = 0x29,
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
// commands: XCMD_BYPASS_COUNT
#define CAP_BYPASS_COUNT 0x00000001
// get the IDCODES; returns a single 00 if none found, FF for chain
//   disconnected, or the list of devices in the chain
// commands: XCMD_GET_IDCODES
#define CAP_GET_IDCODES  0x00000002

// load the jtagx config structure (struct djtag_cfg_s)
// that structure is initialized specifically for an ADIv6 Cortex-M0+, and
//   setting TILESEL automatically changes the default SYS and CPU AP indices
// HOWEVER. if used, this enables some extra functionality, such as addressing
//   a specific device in the JTAG chain, or tweaking the number of idle RTI
//   cycles on APACC accesses
#define CAP_CONFIG  0x00000004

// perform scans; the SCAN commands have exactly the same format and arguments
//   as CMD_XFER (they honor NO_READ and EXTEND_LENGTH), *but*
// - they honor the jtagx settings for leading/trailing 1 bits, which enables
//   targeting a specific device in the JTAG chain
// - they understand the JTAG FSM, and get to RTI before doing anything
// - they leave the JTAG FSM in RTI
// - data is pure little endian (bit reversal and special handling of the last
//   bit is not required, unlike the CMD_CLK/CMD_XFER/CMD_CLK equivalent)
// - a DRSCAN issued while IR is set to APACC will have idle RTI cycles added
//   as configured via CONFIG
// effectively, these commands render all the "legacy" DirtyJTAG2 commands
//   obsolete, and significantly minimize the data traffic over the comms
//   interface - the only instance where the old-style command would be useful
//   is when a single scan of 512+ bits is required 
// commands: XCMD_IRSCAN, XCMD_DRSCAN
#define CAP_SCAN  0x00000008

// issue ARM ABORT, DPACC or APACC accesses
// - the IR codes for these, default to those used by an ADIv6 CortexM0+ CPU;
//   however, they can be configured differently using XCMD_CONFIG
// - ABORT is write-only
//   - has a single 8-bit argument, that gets scanned into bits [10:3] of the
//     ABORT DR (of these, only bits [7:3] are actually used; 0 is scanned in
//     the remaining, unused, bits)
//   - returns nothing
//   - commands: XCMD_ABORT
// - DPACC accesses can be reads or writes,
//   - have a 6-bit address (4-bit bank + 2-bit in-cmd address) encoded as a
//     single byte; for writes, the 32-bit LE-encoded data value follows;
//   - return 1 byte containing the status bits [2:0]; for reads, 4 more bytes
//     with the 32-bit LE-encoded result follow
//   - DPACC accesses might cause an extra write the DP.SELECT, if the last
//     used DP register bank is different from the one currently requested
//   - DPACC reads of SELECT (which is write-only) are not performed; instead,
//     they return the value most recently written to that register, with the
//     status set to OK
//   - commands: XCMD_DPACC_RD, XCMD_DPACC_WR
// - APACC accesses can be reads or writes,
//   - have a 5-bit AP index encoded as a byte; bit 7 of that byte directs the
//     code to perform a DP RDBUFF, instead of an AP same-reg, read, for
//     fetching the status and, for reads, the result value), and 
//     - an 11-bit in-AP address, encoded as a LE halfword
//     - for writes, the 32-bit LE-encoded word to be written follows;
//   - produce 1 byte containing the status bits, plus (for reads) 4 more bytes
//     with the LE-encoded result
//   - APACC accesses typically cause extra writes to DP.SELECT
//   - APACC accesses add idle cycles in RTI after the request is made (8, by
//     default; can be changed with XCMD_CONFIG)
//   - if an APACC access results in WAIT, more (hardcoded: 16) RTI idle cycles
//     are added, then fetching the result is retried *once*
//   - if the status (whether of the initial fetch, or of the retry) is anything
//     *except* OK or WAIT, the transaction is automatically aborted
//   - commands: XCMD_APACC_RD, XCMD_APACC_WR
#define CAP_ABORT   0x00000010
#define CAP_DPACC   0x00000020
#define CAP_APACC   0x00000040

// perform bus, and CPU bus, accesses on the currently selected APs
// - the APs used for these accesses are selected automatically whenever
//   TILESEL is set, but can be overriden using XCMD_CONFIG
// - bus reads and writes specify the 4-byte, LE-encoded target address, and
//   return either 1 byte with the status code, OR (only for succeeded reads)
//   the 4 bytes containing the LE-encoded read result
// - CPU bus reads and writes specify the *2*-byte, LE-encoded target address;
//   this is internally promoted to 0xE0000000|(addr), since the SCB is the
//   only part of the address space where it makes sense to perform accesses
//   on the bus, rather than the SYS, bus
//   return either, or (for reads only) the 4 bytes containing the LE-encoded
//   read result
// - on the CPU bus, combined WR/RD is implemented (has the args of a write,
//   and the result of a read)
// - commands: XCMD_BUS_RD, XCMD_CPU_RD, XCMD_BUS_WR, XCMD_CPU_WR
//     XCMD_CPU_WRRD
#define CAP_BUSACC  0x00000080

// perform bus block accesses on the currently selected APs
// - the AP used for these accesses is selected automatically whenever
//   TILESEL is set, but can be overriden using XCMD_CONFIG
// - block accesses are only supported on the SYS AP; such functionality
//   would make little sense on the CPU AP
// - reads specify a single byte, containing the number of words to read
//   *minus 1*, followed by the 4-byte, LE-encoded start address, and return
//   4*n bytes with the read results; if a failure is encountered, a single
//   byte with the status bits is added to the response
// - writes specify a single byte, containing the number of words to write
//   *minus 1*, followed by a 4-byte, LE-encoded start address, and 4*n bytes
//   containing the values to write; they return 2 bytes, the number of
//   locations NOT written, followed by the most recent status bits
// - fills specify a single byte, containing the number of words to write
//   *minus 1*, followed by a 4-byte, LE-encoded start address, and 4 bytes
//   containing the value to write; they return 2 bytes, the number of
//   locations NOT written, followed by the most recent status bits
// - commands: XCMD_BLOCK_RD, XCMD_BLOCK_WR, XCMD_BLOCK_FILL
#define CAP_BURST   0x00000100

// ARM state tracking
#define CAP_ARM   0x00000200

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

// SYS and CPU accesses for the currently selected tile
int jtag_bus_rd(uint32_t addr, uint32_t *data);
int jtag_bus_wr(uint32_t addr, uint32_t data);
int jtag_cpu_rd(uint32_t addr, uint32_t *data);
int jtag_cpu_wr(uint32_t addr, uint32_t data);

// BYPASS_COUNT and GET_IDCODES ignore config
// IRSCAN, DRSCAN use only config.{ir|dr}.{lead1s|tail1s}
//   (in particular, IRSCAN ignores config.ir_size)
// ABORT, {DP|AP}ACC_{RD|WR} addtionally use config.ir_size and config.{abort|dpacc|apacc}
// APACC_{RD|WR} addtionally use config.armcmd_xcycles
// {BUS|CPU}_{RD|WR} and CPU_WRRD additionally use config.{sys_ap|cpu_ap}

//=[ armx api ]================================================================

// arm subsystem init; provide the imc_write function, and the reset config
// bit 0 of reset_addr indicates whether resets are active-high (0) or
// active-low (1)
// this function, besides initializing the state tracking, resets the m0
void arm_init(
  uint32_t imc_write, unsigned sz_imc_write,
  uint32_t reset_addr, unsigned dbg_bit, unsigned m0_bit);

// resume arm execution; if needed, this also removes the m0 reset
int arm_resume(void);

int get_arm_state(uint8_t *resp);

#define IMPLEMENTED_CAPS ( \
  CAP_BYPASS_COUNT | CAP_GET_IDCODES | \
  CAP_CONFIG | \
  CAP_SCAN | \
  CAP_ABORT | CAP_DPACC | CAP_APACC | \
  CAP_BUSACC | CAP_BURST | \
  CAP_ARM | \
  0)

// CAP_SCAN: IRSCAN, DRSCAN implemented
// CAP_ABORT: ABORT implemented
// CAP_DPACC: DPACC_RD, DPACC_WR implemented
// CAP_APACC: APACC_RD, APACC_WR implemented
