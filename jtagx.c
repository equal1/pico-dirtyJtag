#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "config.h"
#include "cmd.h"
#include "pio_jtag.h"

//#define DEBUG // print what the commands are doing
//#define DEBUG_SCAN // print what's happening while scanning the chains

#ifdef DEBUG
#define dprintf(...) printf(__VA_ARGS__)
#ifdef DEBUG_SCAN
#define ddprintf(...) printf(__VA_ARGS__)
#else
#define ddprintf(...) ((void)((__VA_ARGS__)))
#endif
#else
#define dprintf(...) ((void)((__VA_ARGS__)))
#define ddprintf(...) ((void)((__VA_ARGS__)))
#endif

// default settings
static struct djtag_cfg_s jcfg = {
  .ir = { .lead1s = 0, .tail1s = 0 },
  .dr = { .lead1s = 0, .tail1s = 0 },
  .abort = 0b1000, // 0x8
  .dpacc = 0b1010, // 0xA
  .apacc = 0b1011, // 0xB
  .ir_size = 4,
  .armcmd_xcycles = 8, // cycles after issuing a read/write with APACC, before checking the result
  .cpu_ap = 0, // these get set via XCMD_CONFIG,
  .sys_ap = 0  // or whenever PINCFG changes the value of TILESEL
};

// ARM's DPACC/APACC commands
#define IR_ABORT  0x8 // 4'b1000
#define IR_DPACC  0xA // 4'b1010
#define IR_APACC  0xB // 4'b1011
#define IR_IDCODE 0xE // 4'b1110 // not used, we reply on TLR instead
#define IR_BYPASS 0xF // 4'b1111
#define N_AP 9 // ROM + 4x{CPU,SYS}

// most recent known state
static struct {
  uint8_t in_tlr, in_rti;
  uint32_t ir; // most recent IR
  uint32_t select; // current value of DP.SELECT
  uint32_t tar[N_AP]; // current value of AP[i].TAR
} last;
  
// void jtag_transfer(const pio_jtag_inst_t *jtag, uint32_t length, const uint8_t* in, uint8_t* out);
// uint8_t jtag_strobe(const pio_jtag_inst_t *jtag, uint32_t length, bool tms, bool tdi);
// void pio_jtag_write_blocking(const pio_jtag_inst_t *jtag, const uint8_t *src, size_t len);
// void pio_jtag_write_read_blocking(const pio_jtag_inst_t *jtag, const uint8_t *src, uint8_t *dst, size_t len);

extern pio_jtag_inst_t jtag;

//=[ jcfg_set_tilesel() ]------------------------------------------------------

// this gets called whenever tilesel is changed via PINCFG
void jcfg_set_tilesel(int tile)
{
  if ((tile < 0) || (tile > 3))
    return;
  if (tile != (((jcfg.cpu_ap >> 13) - 1) >> 1))
    printf ("set default tile to %d\n", tile);
  jcfg.cpu_ap = 0x2000 + 0x4000*tile;
  jcfg.sys_ap = 0x4000 + 0x4000*tile;
}

//=[ jtag_go_tlr() ]-----------------------------------------------------------

// go to Test-Level-Reset
void jtag_go_tlr()
{
  int n = 5; // cycles to get to TLR from any state, with TMS=1
  if (last.in_tlr)
    n = 0; // if already in TLR, no clocks required
  else if (last.in_rti)
    n = 3; // from RTI, 3 clocks required
  if (n) {
    ddprintf("jtag_strobe(%d, TMS, TDI) # go to TLR%s\n", n, (n==3)?" from RTI":"");
    jtag_strobe(&jtag, n, SIG_TMS, SIG_TDI);
  }
  // after a TAP reset, the IDCODE DR is selected
  last.in_tlr = 1; last.in_rti = 0;
}

//-[ jtag_go_rti() ]-----------------------------------------------------------

// go to Run-Test-Idle
void jtag_go_rti()
{
  // nothing to do if we're already in RTI
  if (last.in_rti)
    return;
  // if we're not in TLR, get there first (5x TMS=1)
  if (! last.in_tlr) {
    ddprintf("jtag_strobe(5, TMS, TDI) # go to TLR\n");
    jtag_strobe(&jtag, 5, SIG_TMS, SIG_TDI);
  }
  // from TLR, get to RTI (1x TMS=0)
  ddprintf("jtag_strobe(1, #TMS, TDI) # go to RTI\n");
  jtag_strobe(&jtag, 1, 0, SIG_TDI);
  last.in_tlr = 0; last.in_rti = 1;
}

//=============================================================================

// dummy buffers filled with 0x00, and 0xFF respectively
static const uint8_t zeroes[512/8] __attribute__((aligned(8)));
static uint8_t ones[512/8] __attribute__((aligned(8)));

// fast bit-reverse
static uint8_t revbits[256];

__attribute__((constructor))
static void mk_revbin(void)
{
  for (unsigned x = 0; x < 0x100; ++x) {
    unsigned y = ((x & 0x80) >> 7) | 
                 ((x & 0x40) >> 5) |
                 ((x & 0x20) >> 3) |
                 ((x & 0x10) >> 1) |
                 ((x & 0x08) << 1) |
                 ((x & 0x04) << 3) |
                 ((x & 0x02) << 5) |
                 ((x & 0x01) << 7);
    revbits[x] = y;
  }
  memset(ones, 0xFF, sizeof(ones));
}

//-[ jtag_do_scan() ]----------------------------------------------------------

// local buffer (512-bits)
static uint8_t jtag_out[512/8] __attribute__((aligned(8)));
static uint8_t jtag_in[512/8] __attribute__((aligned(8)));

#define DP_SELECT 0x08  // WO
#define MEMAP_TAR 0xD04 // Target Address Register

// execute a scan
// returns the number of bytes of output
unsigned jtag_do_scan(int ir, unsigned n_bits, const void *out_, void *in)
{
  if (n_bits < 1)
    return 0;
  const uint8_t *out = (const uint8_t*)out_;
  // prepare the output buffer (pad the data)
  // also, if we're scanning IR, record the value
  int lead_bits, tail_bits;
  unsigned extra_clocks = 0;
  if (ir & 1) {
    lead_bits = jcfg.ir.lead1s;
    tail_bits = jcfg.ir.tail1s;
  }
  else {
    lead_bits = jcfg.dr.lead1s;
    tail_bits = jcfg.dr.tail1s;
    extra_clocks = (ir >> 1)& 0x7f;
  }
  ir &= 1;

  if (! out) {
    // if we're scanning into the IR, record the value (BYPASS)
    if (ir)
      last.ir = 0xffffffff & ((1 << jcfg.ir_size) - 1);
    // don't bother with other checks - xPACC writes have at least the R/W# bit clear
  }
  else {
    // if we're scanning into the IR, record the value
    if (ir) {
      uint32_t mask = 0xffffffff;
      if (n_bits < 32)
        mask = (1 << n_bits) - 1;
      last.ir = *(uint32_t*)out & mask &  ((1 << jcfg.ir_size) - 1);
      dprintf ("(last.ir=0x%X)\n", last.ir);
    }
    // otherwise, if we're scanning into the DR, and it's a write
    else if (! (*out & 1)) {
      if (last.ir == jcfg.dpacc) {
        // record DP.SELECT writes
        if ((*out & 0x6) == (DP_SELECT >> 1)) {
          last.select = (uint32_t)(*(uint64_t*)out >> 3);
          dprintf ("(last.select=0x%X)\n", last.select);
        }
      } else if (last.ir == jcfg.apacc) {
        // record AP<n>.TAR writes
        if (((last.select & 0x1ffc & ~0xf) == (MEMAP_TAR & ~0xf)) &&
            ((*out & 0x6) == (MEMAP_TAR & 0xC)>> 1))
        {
          int ap = last.select >> 13;
          if (ap < sizeof(last.tar)/sizeof(last.tar[0])) {
            last.tar[ap] = (uint32_t)(*(uint64_t*)out >> 3);
            dprintf ("(last.tar[%u]=0x%X)\n", ap, last.tar[ap]);
          }
        }
      }
    }
  }
  // reverse the bits, if we have data to output
  if (out) {
    for (int i = 0; i < ((n_bits + 7) / 8); ++i)
      jtag_out[i] = revbits[out[i]];
    out = jtag_out;
  }
  // (otherwise, we'll just output 1's)
  else
    out = ones;
  // and extract the last bit
  uint8_t last_bit = 0x80 >> ((n_bits - 1) & 0x7);
  uint8_t last_tdi = last_bit;
  last_tdi &= out[(n_bits - 1) / 8];

  // go to RTI
  jtag_go_rti();
  // go to the right Select-xR state with TMS=1 (1 clk to Select-DR, 1 more to Select-IR)
  ddprintf (" jtag_strobe(%d, TMS, TDI) # go to Select-%cR\n", ir+1, ir?'I':'D');
  jtag_strobe(&jtag, ir ? 2 : 1, SIG_TMS, SIG_TDI);
  last.in_rti = 0;
  // go to the corresponding Shift state with TMS=0 (1 clk to Capture-xR, 1 more to Shift-xR)
  ddprintf (" jtag_strobe(2, #TMS, TDI) # go to Shift-%cR\n", ir?'I':'D');
  jtag_strobe(&jtag, 2, 0, SIG_TDI);

  // send the leading TDI=1 bits
  if (lead_bits) {
    ddprintf (" pio_jtag_write_blocking(ones, %u) # leading `1' bits\n", lead_bits);
    pio_jtag_write_blocking(&jtag, ones, lead_bits);
  }
  // if we have tail bits, capture all the data bits
  if (tail_bits) {
    // transfer all bits, including the last one
    // (send the data, or 1's if data not provided)
    ddprintf (" pio_jtag_write_read_blocking(out, jtag_in, %u) # all the data\n", n_bits);
    pio_jtag_write_read_blocking(&jtag, out, jtag_in, n_bits);
    // send the tail TDI=1 bits, save for the last one, staying in Shift-xR
    if (tail_bits > 1) {
      ddprintf (" jtag_strobe(%d, #TMS, TDI) # all the tail `1' bits, save for the last\n", tail_bits-1);
      jtag_strobe(&jtag, tail_bits - 1, 0, SIG_TDI);
    }
    // send the last tail bit while setting TMS (go to Exit1-xR)
    ddprintf (" jtag_strobe(2, TMS, TDI) # go to Exit1-%cR (sending the last tail `1' bit), then to Update-%cR\n",
              ir?'I':'D', ir?'I':'D');
    jtag_strobe(&jtag, 2, SIG_TMS, SIG_TDI);
  }
  // if we don't, we'll have to capture the last bit from the cycle in which we're leaving Shift-IR
  else {
    // transfer all bits, except the last one
    if (n_bits > 1) {
      ddprintf (" pio_jtag_write_read_blocking(out, jtag_in, %u) # all the data, except the last bit\n", n_bits-1);
      pio_jtag_write_read_blocking(&jtag, out, jtag_in, n_bits - 1);
    }
    // transfer the last bit, as we're exiting Shift-xR
    ddprintf (" jtag_strobe(2, TMS, %sTDI) # go to Exit1-%cR (send last bit, %u), then to Update-%cR\n", 
              last_tdi?"":"#", ir?'I':'D', last_tdi?1:0, ir?'I':'D');
    int last_tdo = jtag_strobe(&jtag, 2, SIG_TMS, last_tdi ? SIG_TDI : 0);
    ddprintf (" > last_tdo=%u\n", last_tdo?1:0);
    if (last_tdo)
      jtag_in[(n_bits - 1) / 8] |= last_bit;
  }
  // JTAG FSM is now in Update-xR
  // go to RTI with TMS=0, and stay there for extra_clocks cycles
  // (see ADIv6 page 95)
  // note, extra_clocks is supposed to be sent only for APACC DR
  // scans, and is automatically 0 for IR scans
  ddprintf (" jtag_strobe(%d, #TMS, TDI) # go to RTI, and stay there for %u cycles\n", 1+extra_clocks, extra_clocks);
  jtag_strobe(&jtag, 1+extra_clocks, 0, SIG_TDI);
  last.in_rti = 1;

  // done, if data is not requested
  if (! in)
    return 0;

  // reverse the result, if we have an input buffer
  int res = (n_bits + 7) / 8;
  if (in)
    for (int i = 0; i < res; ++i)
      ((uint8_t*)in)[i] = revbits[jtag_in[i]];
  return res;
}

//-[ jtag_do_bypass() ]--------------------------------------------------------

// count devices in the JTAG chain, using the BYPASS method
//   returns 0 for no devices, FF for chain disconnected, or the
//   number of devices (scans ups to 64 devices)

unsigned jtag_do_bypass()
{
  // go to RTI
  dprintf ("jtag_go_rti()\n");
  jtag_go_rti();
  // backup current state - we want to do this with no lead/tail bits
  unsigned il = jcfg.ir.lead1s, it = jcfg.ir.tail1s;
  unsigned dl = jcfg.dr.lead1s, dt = jcfg.dr.tail1s;
  jcfg.ir.lead1s = jcfg.ir.tail1s = 0;
  jcfg.dr.lead1s = jcfg.dr.tail1s = 0;
  // flush the IR chain with all ones, putting every device in bypass
  // (BYPASS is defined as the command with an all-1s value, regardless
  //  of the size of the IRs in the chain) 
  dprintf ("jtag_do_scan(1, 256, NULL, NULL) # fill IRs with BYPASS\n");
  jtag_do_scan(1, 256, NULL, NULL);
  // flush the DR chain with all zeroes
  dprintf ("jtag_do_scan(0, 256, zeroes, NULL) # fill DRs with zeroes\n");
  jtag_do_scan(0, 256, zeroes, 0);
  // flush the DR chain with all ones and capture (in-place bit reversal)
  dprintf ("jtag_do_scan(0, 256, NULL, jtag_in) # clock out the DR, injecting 1's\n");
  jtag_do_scan(0, 256, NULL, jtag_in);
  // count the zeros still available
  int n_devices;
  for (n_devices = 0; n_devices < 255; ++n_devices)
    if (jtag_in[n_devices/8] & (1 << (n_devices & 7)))
      break;
  // restore previous state
  jcfg.ir.lead1s = il; jcfg.ir.tail1s = it;
  jcfg.dr.lead1s = dl; jcfg.dr.tail1s = dt;
  //printf ("> %d\n", n_devices);
  return n_devices;
}

//-[ jtag_get_idcodes() ]------------------------------------------------------

// get the IDCODEs in the chain
//   returns 0 if TDO is stuck (at zero OR one), or the number of devices (scans ups
//   to 16 (=512/32) devices)

// return the number of devices, and write the idcode of the first device
// to the argument
unsigned jtag_get_idcodes(uint32_t *idcode)
{
  // reset the JTAG FSM
  dprintf ("jtag_go_tlr()\n");
  jtag_go_tlr();
  // after a JTAG reset, all devices select their IDCODE DR chains
  dprintf ("jtag_go_rti()\n");
  jtag_go_rti();
  // flush out the DR chain (clock 1's in); use jtag_in in-place
  dprintf ("jtag_do_scan(0, 512, NULL, jtag_in) # flush out the IDCODE chain\n");
  jtag_do_scan(0, 512, NULL, jtag_in);

  // if the first ID is 0x00000000, then the chain is disconnected
  if (! *(uint32_t*)jtag_in)
    return 0;

  // count the devices
  int ndev = 0;
  for (int i = 0; i < 64; i += 4 ) {
    unsigned crt_id = *(uint32_t*)(jtag_in + i);
    if (crt_id == 0xffffffff)
      break;
    // store the current IDCODE
    if (idcode)
      *idcode++ = crt_id;
    ++ndev;
  }
  return ndev;
}

//=[ jtag_set_config() ]=======================================================

// load the jcfg structure from the client

// returns the size of the jtag structure
unsigned jtag_set_config(const void *cfg)
{
  memcpy(&jcfg, cfg, sizeof(jcfg));
  // mask the commands to the IR size, just in case
  jcfg.abort &= (1 << jcfg.ir_size) - 1;
  jcfg.apacc &= (1 << jcfg.ir_size) - 1;
  jcfg.dpacc &= (1 << jcfg.ir_size) - 1;
  return sizeof(jcfg);
}

//=[ jtag_abort() ]=========================================================

// the ABORT and xPACC registers contain
// { 32'b{data}, 2'b{addr[3:2]},1'b{R/W#} }
#define MK_JTAG_READ(addr,val) ((((uint64_t)val)<<3)|((addr >> 1)&0x6)|1)
#define MK_JTAG_WRITE(addr,val) ((((uint64_t)val)<<3)|((addr >> 1)&0x6))

#define   ABORT_ORUNERRCLR (0x10) // clear CTRLSTAT.STICKYORUN
//#define ABORT_WDERRCLR   (0x08) // clear CTRLSTAT.WDATAERR // SWD-only
#define   ABORT_STKERRCLR  (0x04) // clear CTRLSTAT.STICKYERR
#define   ABORT_STKCMPCLR  (0x02) // clear CTRLSTAT.STICKYCMP
#define   ABORT_AP         (0x01) // abort the current AP transaction
#define   ABORT_ALL        (0x17)


// issue an ABORT command, with a command word having the specified argument
// returns the status bits 
int jtag_abort(int arg)
{
  uint64_t tmp;
  // select ABORT using IR
  dprintf ("jtag.ir=%u'b%0*b # abort\n", jcfg.ir_size, jcfg.ir_size, jcfg.abort);
  jtag_do_scan(1, jcfg.ir_size, &jcfg.abort, 0);
  // scan in the abort bits in the ABORT DR
  tmp = MK_JTAG_WRITE(0, arg);
  dprintf ("jtag.abort=32'b%032b,3'b0\n", arg);
  jtag_do_scan(0, 35, (uint8_t*)&tmp, (uint8_t*)&tmp);
  // reset the cached state - DP's SELECT value, and every AP's TAR
  last.select = 0xffffffff;
  ddprintf("(last.select=0xFFFFFFFF)\n");
  for (int i = 0; i < N_AP; ++i)
    last.tar[i] = 0xffffffff;;
  ddprintf("(last.tar[*]=0xFFFFFFFF)\n");
  // return the status bits (note, these have no defined meaning)
  return tmp & 0x7;
}

//=[ jtag_dpacc_rd() ]=========================================================

// DP registers ([7:4]bank [3:2]addr [1:0]=2'b00)
#define DP_IDR        0x00 // RO
#define DP_CTRLSTAT   0x04 // RW
#define DP_SELECT     0x08 // WO // [31:4]=ADDR, [3:0]=DPBANKSEL
#define DP_RDBUFF     0x0C // RO
#define DP_IDR1       0x10 // RO
#define DP_BASEPTR0   0x20 // RO
#define DP_TARGETID   0x24 // RO
#define DP_DLPIDR     0x34 // RO
#define DP_EVENTSTAT  0x44 // RO

#define JTAG_WAIT  0x1
#define JTAG_FAULT 0x2
#define JTAG_OK    0x4

// issue an DPACC read
// returns the status bits 
int jtag_dpacc_rd(uint32_t addr, uint32_t *data)
{
  // if the last IR scan selecting something other than DPACC, select DPACC now
  if (last.ir != jcfg.dpacc) {
    dprintf ("jtag.ir=%u'b%0*b # dpacc\n", jcfg.ir_size, jcfg.ir_size, jcfg.dpacc);
    jtag_do_scan(1, jcfg.ir_size, &jcfg.dpacc, NULL);
  }
  uint64_t tmp;
  // if the register isn't SELECT or RDBUFF, load SELECT.DPBANKSEL first
  if (((addr & 0xC) < DP_SELECT) &&
      ((last.select & 0xF) != ((addr >> 4) & 0xF)))
  {
    uint32_t new_select = (last.select & ~0xF) | ((addr >> 4) & 0xF);
    tmp = MK_JTAG_WRITE(DP_SELECT, new_select);
    dprintf ("jtag.dp.select=0x%06X:%X\n", new_select >> 4, new_select&0xf);
    jtag_do_scan(0, 35, &tmp, NULL);
  }
  // SELECT is write-only; if attempting to read it, just return the most
  // recent value
  if ((addr & 0xC) == DP_SELECT)
    tmp = ((uint64_t)last.select << 3) | JTAG_OK;
  // issue the read request, unless we're reading RDBUFF
  else {
    if ((addr & 0xC) != DP_RDBUFF) {
      tmp = MK_JTAG_READ(addr, 0);
      // request the register read: ? DPREG(addr)
      dprintf ("? jtag.dpacc addr[3:2]=2'b%02b # issue the read\n", (addr >> 2)&3);
      jtag_do_scan(0, 35, &tmp, NULL);
    }
    // get the result: ? DPREG(RDBUFF)
    *(uint8_t*)&tmp = MK_JTAG_READ(DP_RDBUFF, 0); // 3'b111
    dprintf ("? jtag.dp.rdbuf # fetch the result\n");
    jtag_do_scan(0, 35, &tmp, &tmp);
  }
  // perform the transaction
  if (data)
    *data = (uint32_t)(tmp >> 3);
  return (int)tmp & 0x7;
}

//-[ jtag_dpacc_wr() ]---------------------------------------------------------

int jtag_dpacc_wr(uint32_t addr, uint32_t data)
{
  // the only writable registers in out DP are CTRL/STAT and SELECT
  if (((addr & 0xC) != DP_SELECT) && (addr != DP_CTRLSTAT))
    return JTAG_OK;
  // select DPACC
  if (last.ir != jcfg.dpacc) {
    dprintf ("jtag.ir=%u'b%0*b # dpacc\n", jcfg.ir_size, jcfg.ir_size, jcfg.dpacc);
    jtag_do_scan(1, jcfg.ir_size, &jcfg.dpacc, NULL);
  }
  uint64_t tmp;
  // if the register isn't SELECT, load SELECT.DPBANKSEL first
  if (((addr & 0xC) != DP_SELECT) &&
      ((last.select & 0xF) != ((addr >> 4) & 0xF)))
  {
    uint32_t new_select = (last.select & ~0xF) | ((addr >> 4) & 0xF);
    tmp = MK_JTAG_WRITE(DP_SELECT, new_select);
    dprintf ("jtag.dp.select=0x%06X:%X\n", new_select >> 4, new_select&0xf);
    jtag_do_scan(0, 35, &tmp, NULL);
  }
  // perform the register write: DPREG(addr):=data
  tmp = MK_JTAG_WRITE(addr, data);
  dprintf ("jtag.dpacc[2'b%02b]=0x%08X # issue the write\n", (addr >> 2)&3, data);
  jtag_do_scan(0, 35, &tmp, NULL);
  // issue a RDBUFF read just to get the result bits: ? DPREG(RDBUFF)
  tmp = MK_JTAG_READ(DP_RDBUFF, 0); // 3'b111
  dprintf ("? jtag.dp.rdbuf # fetch the status\n");
  jtag_do_scan(0, 35, &tmp, &tmp);
  return (int)tmp & 0x7;
}

//=[ jtag_apacc_rd() ]=========================================================

#define MEMAP_DAR0       0x000 // 256 Data Access Registers; access is relative to {TAR[31:10], 10'h0}
#define MEMAP_DAR_TOP    0x400
#define MEMAP_CSW        0xD00
#define MEMAP_TAR        0xD04 // Target Address Register
#define MEMAP_DRW        0xD0C // Data Read/Write; access at TAR; supports byte/halfword accesses (config in CSW)
#define MEMAP_BD0        0xD10 // 4 Banked Data Registers; access is relative to {TAR[31:4], 3'h0}
#define MEMAP_BD1        0xD14
#define MEMAP_BD2        0xD18
#define MEMAP_BD3        0xD1C
#define MEMAP_MBT        0xD20 // Memory Barrier Transfer // probably not implemented
#define MEMAP_TRR        0xD24 // Transfer Response
#define MEMAP_T0TR       0xD30 // Tag 0 Transfer
#define MEMAP_CFG1       0xDE0
#define MEMAP_CFG        0xDF4 // Config; RO
#define MEMAP_BASE       0xDF8 // Debug Base Address; RO
#define MEMAP_ITSTATUS   0xEFC

// issue an SPACC read
// returns the status bits 
int jtag_apacc_rd(int ap, uint32_t addr, uint32_t *data)
{
  // bit 7 of ap set => read rdbuff to fetch the result
  int do_rdbuff_check = ap & 0x80;
  // build the target APACC address
  // an AP register's address is { 15'b0, 4'b{ap}, 11'b{addr[13:2]}, 2'b0 }
  ap &= 0xF; // APs are 0..8
  addr &= 0x1ffc; // AP space is 32KB, and all regs are word-aligned
  uint32_t taddr = addr | (ap << 13);
  uint64_t tmp;

  // reprogram DP.SELECT.ADDR if need be
  if ((last.select & ~0xf) != (taddr & ~0xf)) {
    // select DPACC
    if (last.ir != jcfg.dpacc)  {
      dprintf ("jtag.ir=%u'b%0*b # dpacc\n", jcfg.ir_size, jcfg.ir_size, jcfg.dpacc);
      jtag_do_scan(1, jcfg.ir_size, &jcfg.dpacc, NULL);
    }
    // select AP register bank: DPREG(SELECT)={28'{addr[31:4]}, 4'{last_dpbanksel}}
    uint32_t new_select = (taddr & ~0xf) | (last.select & 0xf);
    tmp = MK_JTAG_WRITE(DP_SELECT, new_select);
    dprintf ("jtag.dp.select=0x%06X:%X\n", new_select >> 4, new_select&0xf);
    jtag_do_scan(0, 35, &tmp, NULL);
  }

  // it seems that sometimes APACC de-selects itself?
  // (for example, apacc_wr(tar) right after an apacc_rd(idr) doesn't seem to change TAR,
  //  unless we either make this unconditional or we issue an abort inbetween)
  // only apacc_wr() was observed to do this, but for consistency, do it for apacc_rd() too
# if 0
  // if the last IR scan selected something other than APACC, select APACC now
  if (last.ir != jcfg.apacc) {
# endif
    dprintf ("jtag.ir=%u'b%0*b # apacc\n", jcfg.ir_size, jcfg.ir_size, jcfg.apacc);
    jtag_do_scan(1, jcfg.ir_size, &jcfg.apacc, NULL);
# if 0
  }
# endif
    // attempt the register read: ? APREG(addr)
  tmp = MK_JTAG_READ(taddr, 0);
  dprintf ("? jtag.apacc addr[3:2]=2'b%02b # issue the read (%u extra cycles)\n", (addr >> 2)&3, jcfg.armcmd_xcycles);
  jtag_do_scan((jcfg.armcmd_xcycles << 1), 35, &tmp, NULL);

  // read result can be fetched either via APACC read from same address,
  // or from DPACC read from RDBUFF
  if (! do_rdbuff_check) {
    // re-issue the read to get the result (APACC already selected, and tmp is fine)
    dprintf ("? jtag.apacc addr[3:2]=2'b%02b # fetch the result\n", (addr >> 2)&3);
    jtag_do_scan(0, 35, &tmp, &tmp);
  }
  else {
    // select DPACC and read RDBUFF
    dprintf ("jtag.ir=%u'b%0*b # dpacc\n", jcfg.ir_size, jcfg.ir_size, jcfg.dpacc);
    jtag_do_scan(1, jcfg.ir_size, &jcfg.dpacc, NULL);
    *(uint8_t*)&tmp = MK_JTAG_READ(DP_RDBUFF, 0); // 3'b111
    dprintf ("? jtag.dp.rdbuff # fetch the result\n");
    jtag_do_scan(0, 35, &tmp, &tmp);
  }
  // perform the transaction
  if (data)
    *data = (uint32_t)(tmp >> 3);
  int status = (int)tmp & 0x7;

  // if we got WAIT, do 16 TCK pulses and retry ONCE
  // the right IR is still selected, but tmp was changed
  if (status == JTAG_WAIT) {
    dprintf ("jtag_strobe(16) # got WAIT - retry after 16 TCK pulses\n", (addr >> 2)&3);
    jtag_strobe(&jtag, 16, 0, SIG_TDI);
    if (! do_rdbuff_check) {
      tmp = MK_JTAG_READ(taddr, 0);
      dprintf ("? jtag.apacc addr[3:2]=2'b%02b # try #2\n", (addr >> 2)&3);
      jtag_do_scan(0, 35, &tmp, &tmp);
    }
    else {
      tmp = MK_JTAG_READ(DP_RDBUFF, 0); // 3'b111
      dprintf ("? jtag.dp.rdbuff # try #2\n");
      jtag_do_scan(0, 35, &tmp, &tmp);
    }
    if (data)
      *data = (uint32_t)(tmp >> 3);
    status = (int)tmp & 0x7;
  }

  // if we got anything other than OK or WAIT (either after the first try, or after the re-try), issue the abort immediately
  // normally this means FAULT, but also applies to bad status bits
  if ((status != JTAG_OK) && (status != JTAG_WAIT)) {
    dprintf ("jtag_abort(5'b%05b) # got FAULT - ABORT the AP access\n", ABORT_ALL);
    jtag_abort(ABORT_ALL);
  }
  return status;
}

//-[ jtag_dpacc_wr() ]---------------------------------------------------------

int jtag_apacc_wr(int ap, uint32_t addr, uint32_t data)
{
  // bit 7 of ap set => read rdbuff to fetch the status
  int do_rdbuff_check = ap & 0x80;
  // build the target APACC address
  // an AP register's address is { 15'b0, 4'b{ap}, 11'b{addr[13:2]}, 2'b0 }
  ap &= 0xF; // APs are 0..8
  addr &= 0x1ffc; // AP space is 32KB, and all regs are word-aligned
  uint32_t taddr = addr | (ap << 13);
  uint64_t tmp;

  // reprogram DP.SELECT.ADDR if need be
  if ((last.select & ~0xf) != (taddr & ~0xf)) {
    // select DPACC
    if (last.ir != jcfg.dpacc) {
      dprintf ("jtag.ir=%u'b%0*b # dpacc\n", jcfg.ir_size, jcfg.ir_size, jcfg.dpacc);
      jtag_do_scan(1, jcfg.ir_size, &jcfg.dpacc, NULL);
    }
    // select AP register bank: DPREG(SELECT)={28'{addr[31:4]}, 4'{last_dpbanksel}}
    uint32_t new_select = (taddr & ~0xf) | (last.select & 0xf);
    tmp = MK_JTAG_WRITE(DP_SELECT, new_select);
    dprintf ("jtag.dp.select=0x%06X:%X\n", new_select >> 4, new_select&0xf);
    jtag_do_scan(0, 35, &tmp, NULL);
  }

  // it seems that sometimes APACC de-selects itself?
  // (for example, apacc_wr(tar) right after an apacc_rd(idr) doesn't seem to change TAR,
  //  unless we either make this unconditional or we issue an abort inbetween)
# if 0
  // if the last IR scan selected something other than APACC, select APACC now
  if (last.ir != jcfg.apacc) {
# endif
    dprintf ("jtag.ir=%u'b%0*b # apacc\n", jcfg.ir_size, jcfg.ir_size, jcfg.apacc);
    jtag_do_scan(1, jcfg.ir_size, &jcfg.apacc, NULL);
# if 0
  }
# endif
  // attempt the register write: APREG(addr):=data
  tmp = MK_JTAG_WRITE(taddr, data);
  dprintf ("jtag.apacc[2'b%02b]=0x%08X # issue the write (%u extra cycles)\n", (addr >> 2)&3, data, jcfg.armcmd_xcycles);
  jtag_do_scan((jcfg.armcmd_xcycles << 1), 35, &tmp, NULL);

  // to get the status bits, either issue an APACC read from the same address,
  // or do a DPACC read from RDBUFF
  if (! do_rdbuff_check) {
    // issue a read to the same address, just to get the last status bits (APACC still selected)
    tmp = MK_JTAG_READ(taddr, 0);
    dprintf ("? jtag.apacc addr[3:2]=2'b%02b # fetch the status\n", (addr >> 2)&3);
    jtag_do_scan((jcfg.armcmd_xcycles << 1), 35, &tmp, &tmp);
  }
  else {
    // select DPACC and read RDBUFF, just to get the status bits
    dprintf ("jtag.ir=%u'b%0*b # dpacc\n", jcfg.ir_size, jcfg.ir_size, jcfg.dpacc);
    jtag_do_scan(1, jcfg.ir_size, &jcfg.dpacc, NULL);
    tmp = MK_JTAG_READ(DP_RDBUFF, 0); // 3'b111
    dprintf ("? jtag.dp.rdbuff # fetch the status\n");
    jtag_do_scan(0, 35, &tmp, &tmp);
  }
  int status = (int)tmp & 0x7;

  // if we got WAIT, do 16 TCK pulses and retry ONCE
  // the right IR is still selected, but tmp was changed
  if (status == JTAG_WAIT) {
    dprintf ("jtag_strobe(16) # got WAIT - retry after 16 TCK pulses\n", (addr >> 2)&3);
    jtag_strobe(&jtag, 16, 0, SIG_TDI);
    if (! do_rdbuff_check) {
      dprintf ("? jtag.apacc addr[3:2]=2'b%02b # try #2\n", (addr >> 2)&3);
      tmp = MK_JTAG_READ(taddr, 0);
      jtag_do_scan(0, 35, &tmp, &tmp);
    }
    else {
      dprintf ("? jtag.dp.rdbuff # try #2\n");
      tmp = MK_JTAG_READ(DP_RDBUFF, 0); // 3'b111
      jtag_do_scan(0, 35, &tmp, &tmp);
    }
    status = (int)tmp & 0x7;
  }

  // if we got anything other than OK or WAIT (either after the first try, or after the re-try), issue the abort immediately
  // normally this means FAULT, but also applies to bad status bits
  if ((status != JTAG_OK) && (status != JTAG_WAIT)) {
    dprintf ("jtag_abort(5'b%05b) # got FAULT - ABORT the AP access\n", ABORT_ALL);
    jtag_abort(ABORT_ALL);
  }
  return status;
}

