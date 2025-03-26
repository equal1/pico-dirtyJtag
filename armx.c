#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "arm.h"
#include "cmd.h"

//#define DEBUG // print what the commands are doing
//#define DEBUG_VERBOSE // print what's happening while scanning the chains

//#define MIRROR_IMC // mirror the IMC output

#ifdef DEBUG
#define dprintf(...) (printf(__VA_ARGS__))
#ifdef DEBUG_VERBOSE
#define ddprintf(...) (printf(__VA_ARGS__))
#else
#define ddprintf(...) ((void)((__VA_ARGS__)))
#endif
#else
#define dprintf(...) ((void)((__VA_ARGS__)))
#define ddprintf(...) ((void)((__VA_ARGS__)))
#endif

#ifdef MIRROR_IMC
# define imc_printf(...) (printf(__VA_ARGS__))
# define imc_puts(s) (puts(s))
#else
# define imc_printf(...) ((void)((__VA_ARGS__)))
# define imc_puts(s) ((void)(s))
#endif

static struct {
  struct {
    uint32_t dhcsr, dfsr, icsr;
    uint32_t systick;
    uint32_t vtor;
    union {
      uint32_t r[0x14];
      struct {
        uint32_t _r[13]; // r0..r12
        uint32_t sp; // r13
        uint32_t lr; // r14
        uint32_t pc; // r15
        uint32_t psr;
        uint32_t msp;
        uint32_t _psp; // not using a process stack
        uint32_t _r19; // reg 0x13 is reserved
        uint32_t ctrl_primask;
      };
    } regs;
    struct {
      union {
        uint32_t on_stack[8];
      };
      struct {
        uint32_t r0[4]; // r[0..3] @sp+0,4,8,C
        uint32_t r12;   // r12 @sp+10
        uint32_t lr;    // r14 @sp+14
        uint32_t pc;    // r15 @sp+18
        uint32_t psr;   // psr @sp+1c
        uint32_t sp;    // (r13) is sp+(psr.a?4:0)
      };
    } except;
  } arm;
  struct {
    // address of in-chip data
    uint32_t ptr_buf; // address of the buffer
    uint32_t sz_buf; // size of the buffer
    uint32_t ptr_wrpos_count;
    uint32_t ptr_rdpos;
    // state tracking
    uint32_t last_wrpos_count; // most recent 
    uint32_t crt_wrpos, crt_count; // updated by ARM
    uint32_t last_rdpos;
    // buffers
    char *buf; // copy of the relevant bits of the on-chip buffer
    char *data; // extracted data
    unsigned sz_data;
    // 
  } imc;
  struct {
    int halted_by_us;
    struct {
      uint32_t start, end;
    } fn_imc_write;
    struct {
      int active_low;
      uint32_t addr;
      uint32_t m0_bit, dbg_bit;
    } reset;
  } cfg;
  int last;
} state;

//=============================================================================

static void add_imc_text(unsigned, const char *);

static int console_update();

#define DHCSR_NOT_RUNNING (DHCSR_S_HALT | DHCSR_S_SLEEP | DHCSR_S_LOCKUP)

static uint32_t get_cpu_reg(unsigned addr, const char *name) {
  uint32_t res;
  int e = jtag_cpu_rd(addr, &res);
  if (e != JTAG_OK) {
    // try once more
    e = jtag_cpu_rd(addr, &res);
    if (e != JTAG_OK) {
      printf("failed to read %s@0x%04X!\n", name, addr);
      return UNKNOWN;
    }
  }
  ddprintf ("%s: 0x%X\n", name, res);
  return res;
}

static uint32_t get_bus_reg(unsigned addr, const char *name) {
  uint32_t res;
  int e = jtag_bus_rd(addr, &res);
  if (e != JTAG_OK) {
    // try once more
    e = jtag_bus_rd(addr, &res);
    if (e != JTAG_OK) {
      printf("failed to read %s@0x%04X!\n", name, addr);
      return UNKNOWN_DATA; // '????'
    }
  }
  ddprintf ("%s: 0x%X\n", name, res);
  return res;
}

static uint32_t set_cpu_reg(unsigned addr, uint32_t val, const char *name) {
  int e = jtag_cpu_wr(addr, val);
  if (e != JTAG_OK) {
    // try once more
    e = jtag_cpu_wr(addr, val);
    if (e != JTAG_OK) {
      printf("failed to write %s@0x%04X to 0x%X!\n", name, addr, val);
      return -1;
    }
  }
  ddprintf ("%s:=0x%X\n", name, val);
  return 0;
}

static uint32_t set_bus_reg(unsigned addr, uint32_t val, const char *name) {
  int e = jtag_bus_wr(addr, val);
  if (e != JTAG_OK) {
    // try once more
    e = jtag_bus_wr(addr, val);
    if (e != JTAG_OK) {
      printf("failed to write %s@0x%04X to 0x%X!\n", name, addr, val);
      return -1;
    }
  }
  ddprintf ("%s:=0x%X\n", name, val);
  return 0;
}

static uint32_t get_dhcsr()
{ return get_cpu_reg(DBG_DHCSR, "DHCSR"); }
static uint32_t get_dfsr()
{ return get_cpu_reg(DBG_DFSR, "DFSR"); }
static uint32_t get_demcr()
{ return get_cpu_reg(DBG_DEMCR, "DEMCR"); }
static uint32_t get_dcrsr()
{ return get_cpu_reg(DBG_DCRSR, "DCCSR"); }
static uint32_t get_dcrdr()
{ return get_cpu_reg(DBG_DCRDR, "DCRDR"); }
static uint32_t get_vtor()
{ return get_cpu_reg(SCB_VTOR, "VTOR"); }
static uint32_t get_icsr()
{ return get_cpu_reg(SCB_ICSR, "ICSR"); }
static uint32_t get_systick()
{ return get_cpu_reg(SYST_CVR, "SYSTICK"); }
static uint32_t get_imc_wrpos_cnt()
{ return get_bus_reg(state.imc.ptr_wrpos_count, "IMC.WRPOS"); }
static uint32_t get_imc_buf(uint32_t ofs)
{ return get_bus_reg(state.imc.ptr_buf + ofs, "IMC.BUF"); }

#define DHCSR_KEEP_FIELDS (DHCSR_C_HALT | DHCSR_C_STEP | DHCSR_C_MASKINTS)

static uint32_t set_dhcsr(uint32_t x) {
  x = (x & DHCSR_KEEP_FIELDS) | DHCSR_DBGKEY | DHCSR_DEBUGEN;
  return set_cpu_reg(DBG_DHCSR, x, "DHCSR");
}
static uint32_t set_dfsr(uint32_t x)
{ return set_cpu_reg(DBG_DFSR, x, "DFSR"); }
static uint32_t set_demcr(uint32_t x)
{ return set_cpu_reg(DBG_DEMCR, x, "DEMCR"); }
static uint32_t set_dcrsr(uint32_t x)
{ return set_cpu_reg(DBG_DCRSR, x, "DCRSR"); }
static uint32_t set_dcrdr(uint32_t x)
{ return set_cpu_reg(DBG_DCRDR, x, "DCRDR"); }
static uint32_t set_vtor(uint32_t x)
{ return set_cpu_reg(SCB_VTOR, x, "VTOR"); }
static uint32_t set_icsr(uint32_t x)
{ return set_cpu_reg(SCB_ICSR, x, "ICSR"); }
static uint32_t set_imc_rdpos(uint32_t x)
{ return set_bus_reg(state.imc.ptr_rdpos, x | 0x10000, "IMC.RDPOS"); }
  
static int arm_update(int force_all)
{
  //---------------------------------------------------------------------------
  // read DHCSR first
  if ((state.arm.dhcsr = get_dhcsr()) == UNKNOWN)
    return -1;
  // if core debug isn't enabled, enable it first!
  uint32_t dhcsr = state.arm.dhcsr;
  if (! (dhcsr & DHCSR_DEBUGEN )) {
    dprintf("enabling DHCSR.DEBUGEN\n");
    set_dhcsr(dhcsr);
    // re-read DHCSR to confirm 
    if ((dhcsr = get_dhcsr()) == UNKNOWN)
      return -1;
  }
  state.cfg.halted_by_us = 0;
  //---------------------------------------------------------------------------
  // if we haven't initialized the IMC yet, do so
  if (! state.imc.ptr_buf) {
    // if we're really early, wait for the initial system configuration to
    // complete (the cue is that the ARM vector table address is no longer 0)
    if ((! state.arm.vtor) || (state.arm.vtor == UNKNOWN)) {
      // read VTOR
      state.arm.vtor = get_vtor();
      if ((! state.arm.vtor) || (state.arm.vtor == UNKNOWN)) {
        goto check_arm_state;
      }
    }
    // read the IMC buffer pointer and size
    uint32_t imc_addr = state.arm.vtor + offsetof(struct vectors_s, imc_tx);
    uint32_t buf_ptr, buf_sz;
    int e;
    e = jtag_bus_rd(imc_addr + offsetof(struct imc_s, buf_addr), &buf_ptr);
    e |= jtag_bus_rd(imc_addr + offsetof(struct imc_s, buf_size), &buf_sz);
    if (e != JTAG_OK)
      return -3;
    state.imc.ptr_buf = buf_ptr;
    state.imc.sz_buf = buf_sz;
    state.imc.ptr_wrpos_count = imc_addr + offsetof(struct imc_s, wrpos_count);
    state.imc.ptr_rdpos = imc_addr + offsetof(struct imc_s, rdpos);
    // allocate a buffer for the in-chip buffer
    unsigned sz = (state.imc.sz_buf + 3) & ~3;
    if (state.imc.buf)
      free (state.imc.buf);
    state.imc.buf = (char*)malloc(sz);
    // allocate a buffer for the response data
    if (state.imc.data)
      free (state.imc.data);
    state.imc.data = (char*)malloc(sz + 8);
    // setup the pointer for wrpos/count, so we won't have to re-compute it later
    dprintf("IMC buffer 0x%04X..%04X, ctrl@0x%04X(wrpos@+0x%X, rdpos@+0x%X)\n",
           buf_ptr, buf_ptr + buf_sz - 1, imc_addr,
           state.imc.ptr_wrpos_count-imc_addr, state.imc.ptr_rdpos-imc_addr);
  }
  //---------------------------------------------------------------------------
check_arm_state:
  // if the ARM is running, not much left to do
  if (! (dhcsr & DHCSR_NOT_RUNNING)) {
    // printf ("arm running\n");
    return 0;
  }
  //---------------------------------------------------------------------------
  // in any other state, we need at the very least the PC
  // - halt ARM first
  if (! (dhcsr & DHCSR_S_HALT)) {
    set_dhcsr(dhcsr | DHCSR_C_HALT);
    dhcsr = get_dhcsr();
    if (((dhcsr == UNKNOWN) || !(dhcsr & DHCSR_S_HALT)))
      return -4;
    state.cfg.halted_by_us = 1;
  }
  // grab the current PC
  uint32_t pc;
  set_dcrsr(DCRSR_READ(REG_PC));
  dhcsr = get_dhcsr();
  state.arm.regs.pc = get_dcrdr();
  if ((dhcsr == UNKNOWN) || ! (dhcsr & DHCSR_S_REGRDY) 
      || (state.arm.regs.pc == UNKNOWN))
    return -5;
  // clear PC's thumb bit
  state.arm.regs.pc &= ~1;
  // if the core is sleeping, return (leaving the core halted)
  // UNLESS force_all is set
  if (((state.arm.dhcsr & DHCSR_NOT_RUNNING) == DHCSR_S_SLEEP)) {
    dprintf ("arm sleeping, pc=0x%X\n", state.arm.regs.pc);
    if (! force_all)
      return 1;
  }
  // in any other situation, read systick and the other regs too
  state.arm.systick = get_systick();
  for (int i = 0; i < N_ARM_REGS; ++i) {
    if ((i != REG_PC) && (i != 0x13)) { // we already have PC, and r19 is undefined
      set_dcrsr(DCRSR_READ(i));
      dhcsr = get_dhcsr();
      state.arm.regs._r[i] = get_dcrdr();
      if ((dhcsr == UNKNOWN) || ! (dhcsr & DHCSR_S_REGRDY))
        state.arm.regs._r[i] = UNKNOWN;
    }
  }
  // if we had only been sleeping (and force_all was set)
  if ((state.arm.dhcsr & DHCSR_NOT_RUNNING) == DHCSR_S_SLEEP)
    return 1;
  // grab DFSR to see if we have an exception
  state.arm.dfsr = get_dfsr();
  // if we hit a lockup
  if ((state.arm.dhcsr & DHCSR_NOT_RUNNING) == DHCSR_S_LOCKUP) {
    printf("arm lockup, pc=0x%X, dhcsr=0x%X, dfsr=0x%X\n", 
      state.arm.regs.pc, state.arm.dhcsr, state.arm.dfsr);
    return 1;
  }
  // did the core get halted?
  if ((state.arm.dfsr == UNKNOWN) || ! (state.arm.dfsr & DFSR_VCATCH)) {
    dprintf("arm halted?, pc=0x%X, dhcsr=0x%X, dfsr=0x%X\n", 
           state.arm.regs.pc, state.arm.dhcsr, state.arm.dfsr);
    return 2;
  }
  // we do: grab ICSR too
  state.arm.icsr = get_icsr();
  // if SP is valid, fetch the 8 words on the stack too
  uint32_t sp = state.arm.regs.sp;
  if ((sp >= RAM_START) && ((sp+0x20) <= RAM_END)) {
    for (int i = 0; i < 8; ++i) {
      int e = jtag_bus_rd(sp + (i << 2), &state.arm.except.on_stack[i]);
      if (e != JTAG_OK)
        state.arm.except.on_stack[i] = UNKNOWN;
    }
    // rebuilt the sp at exception time
    state.arm.except.sp = UNKNOWN;
    if (state.arm.except.psr != UNKNOWN)
      state.arm.except.sp = sp + ((state.arm.except.psr & PSR_a) ? 0x24 : 0x20);
  }
  printf("arm crashed!, pc=0x%X(0x%X), dhcsr=0x%X, dfsr=0x%X, icsr=0x%X\n",
         state.arm.regs.pc, state.arm.except.on_stack[6],
         state.arm.dhcsr, state.arm.dfsr, state.arm.icsr);
  return 3;
}

//=============================================================================

int console_update()
{
  // bail out if console isn't initialized yet
  char *buf = state.imc.buf;
  if (! buf)
    return 0;
  state.imc.sz_data = 0;
  // bail out if there's no new data in the IMC
  uint32_t crt_wrpos_count = get_imc_wrpos_cnt(); 
  if (crt_wrpos_count == UNKNOWN_DATA)
    return 0;
  // see how much the ARM has output
  int n = (crt_wrpos_count >> 16) - (state.imc.last_wrpos_count >> 16);
  if (n < 0)
    n += 0x10000;
  if (! n)
    return 0;
  unsigned last_rdpos = state.imc.last_rdpos;
  unsigned new_rdpos = last_rdpos + n;
  // if we've overrun the buffer
  unsigned bsz = state.imc.sz_buf;
  if (n > bsz) {
    // if the core was running, halt it
    int new = n;
    if (! (state.arm.dhcsr & DHCSR_NOT_RUNNING)) {
      set_dhcsr(state.arm.dhcsr | DHCSR_C_HALT);
      state.cfg.halted_by_us = 1;
    }
    // re-read wrpos and count
    crt_wrpos_count = get_imc_wrpos_cnt();
    new = (crt_wrpos_count >> 16) - (state.imc.last_wrpos_count >> 16);
    if (new < 0)
      new += 0x10000;
    // fixup rdpos; skip the oldest char, it might've been overwritteen
    new_rdpos = crt_wrpos_count & 0xffff;
    last_rdpos = new_rdpos + 2;
    if (last_rdpos >= bsz)
      last_rdpos -= bsz;
    n = new_rdpos - last_rdpos;
    while (n <= 0)
      n += bsz;
    // read the whole buffer
    for (unsigned i = 0; i < state.imc.sz_buf; i += 4)
      *(uint32_t*)(buf + i) = get_imc_buf(i);
    dprintf("console_update: wrpos/cnt=%u:%u,last=%u:%u; rdpos=%u,next=%u+%u; n=%u (of %u)\n", 
      crt_wrpos_count & 0xffff, crt_wrpos_count >> 16, 
      state.imc.last_wrpos_count & 0xffff, state.imc.last_wrpos_count >> 16,
      last_rdpos, (new_rdpos>=bsz)?bsz:0, (new_rdpos>=bsz)?(new_rdpos-bsz):new_rdpos, n, new);
    imc_printf ("imc(%u)={", 5+n);
    // add an ellipsis
    add_imc_text(5, "[...]");
    // print the oldest data (in the 2nd part of buf)
    add_imc_text(state.imc.sz_buf - last_rdpos, buf + last_rdpos);
    // print the newer data (in the 1st part of buf)
    if (new_rdpos)
      add_imc_text(new_rdpos, buf + 0);
    imc_printf ("}\n");
    // set the count to the buffer size, plus the ellipsis length
    n = 5 + state.imc.sz_buf;
  }
  else {
    dprintf("console_update: wrpos/cnt=%u:%u,last=%u:%u; rdpos=%u,next=%u; n=%u\n", 
      crt_wrpos_count & 0xffff, crt_wrpos_count >> 16, 
      state.imc.last_wrpos_count & 0xffff, state.imc.last_wrpos_count >> 16,
      last_rdpos, new_rdpos, n);
    // note, new_rdpos *might* exceed sz_buf; we handle this by reading with 2
    // indices, one of which wraps around
    int wa = 0;
    for (unsigned i = last_rdpos & ~3, j = i; i < new_rdpos; i += 4, j += 4) {
      if (j >= state.imc.sz_buf) {
        wa = 1;
        j -= state.imc.sz_buf;
      }
      *(uint32_t*)(buf + j) = get_imc_buf(j);
    }
    // handle wrap-arounds
    imc_printf ("imc(%u)={", n);
    if (! wa)
      add_imc_text(n, buf + last_rdpos);
    else {
      add_imc_text(state.imc.sz_buf - last_rdpos, buf + last_rdpos);
      add_imc_text(new_rdpos - state.imc.sz_buf, buf + 0);
    }
    imc_printf ("}\n");
  }
  if (new_rdpos > state.imc.sz_buf)
    new_rdpos -= state.imc.sz_buf;
  // mark that we've updated rdpos
  set_imc_rdpos(new_rdpos);
  // update the last state
  state.imc.last_wrpos_count = crt_wrpos_count;
  state.imc.last_rdpos = new_rdpos;
  return n;
}

//-----------------------------------------------------------------------------

void add_imc_text(unsigned n, const char *s)
{
  if (! n)
    return;
  imc_printf ("%.*s", n, s);
  memcpy(state.imc.data + state.imc.sz_data, s, n);
  state.imc.sz_data += n;
}

//=============================================================================

void arm_init(
  uint32_t imc_write, unsigned sz_imc_write,
  uint32_t reset_addr, unsigned dbg_bit, unsigned m0_bit)
{
  // cleanup most recent state
  if (state.imc.buf)
    free(state.imc.buf);
  if (state.imc.data)
    free(state.imc.data);
  memset(&state, 0, sizeof(state));
  // setup the imc_write() address range
  imc_write &= ~1;
  state.cfg.fn_imc_write.start = imc_write;
  state.cfg.fn_imc_write.end = imc_write + sz_imc_write;
  dprintf("registered imc_write @0x%04X..%04X\n", 
    imc_write, imc_write + sz_imc_write - 1);
  // setup the reset config
  if (reset_addr & ~3) {
    state.cfg.reset.addr = reset_addr & ~3;
    state.cfg.reset.active_low = reset_addr & 1;
    if (dbg_bit < 32)
      state.cfg.reset.dbg_bit = 1 << dbg_bit;
    if (m0_bit < 32)
      state.cfg.reset.m0_bit = 1 << m0_bit;
    // read the current resets
    uint32_t crt_rsts;
    int e = jtag_bus_rd(reset_addr & ~3, &crt_rsts);
    if (e != JTAG_OK)
      return;
    dprintf("reset reg at 0x%08X, active %s; dbg=0x%X, m0=0x%X\n",
      reset_addr & ~3, (reset_addr&1)?"low":"high",
      state.cfg.reset.dbg_bit, state.cfg.reset.m0_bit);
    // assert resets for both the M0 and the M0 debug unit
    if (! (reset_addr & 1))
      crt_rsts |= state.cfg.reset.dbg_bit | state.cfg.reset.m0_bit;
    else
      crt_rsts &= ~(state.cfg.reset.dbg_bit | state.cfg.reset.m0_bit);
    e = jtag_bus_wr(reset_addr, crt_rsts);
    // de-assert the M0 debug unit reset, keeping the M0 main reset
    if (! (reset_addr & 1))
      crt_rsts &= ~state.cfg.reset.dbg_bit;
    else
      crt_rsts |= state.cfg.reset.dbg_bit;
    e = jtag_bus_wr(reset_addr, crt_rsts);
  }
  // enable DEBUGEN in DHCSR
  set_dhcsr(DHCSR_C_HALT | DHCSR_DEBUGEN);
  // set the core to halt on leaving RESET
  set_demcr(DEMCR_VC_CORERST | DEMCR_VC_HARDERR);
  // read back DHCSR/VTOR
  state.arm.dhcsr = get_dhcsr();
  state.arm.vtor = get_vtor();
  uint32_t demcr = get_demcr();
  dprintf("after init: DHCSR=0x%X VTOR=0x%X DEMCR=0x%X\n", state.arm.dhcsr, state.arm.vtor, demcr);
}

int arm_resume(void)
{
  uint32_t dhcsr;
  if ((dhcsr = get_dhcsr()) == UNKNOWN)
    return -1;
  ddprintf("DHCSR=0x%08X\n", dhcsr);
  // if the core has C_HALT set, but S_HALT clear: remove the reset
  if ((dhcsr & (DHCSR_C_HALT | DHCSR_S_HALT)) == DHCSR_C_HALT) {
    uint32_t crt_rsts;
    jtag_bus_rd(state.cfg.reset.addr, &crt_rsts);
    if (state.cfg.reset.active_low) {
      // make sure C_HALT is clear and DEBUGEN is set
      if (! (crt_rsts & state.cfg.reset.m0_bit)) {
        //puts("removing m0 reset#");
        jtag_bus_wr(state.cfg.reset.addr, crt_rsts | state.cfg.reset.m0_bit);
        goto no_reset;
      }
    } else {
      if (crt_rsts & state.cfg.reset.m0_bit) {
        //puts("removing m0 reset");
        jtag_bus_wr(state.cfg.reset.addr, crt_rsts & ~state.cfg.reset.m0_bit);
        goto no_reset;
      }
    }
    return -2;
  no_reset:
    // paint the register with junk values
    unsigned rval = 0x01010101;
    for (int i = 0; i < REG_PC; ++i) {
      if (i != REG_SP) {
        set_dcrsr(DCRSR_WRITE(i));
        set_dcrdr(rval);
      }
      rval += 0x01010101;
    }
    dprintf("after leaving reset: DHCSR=0x%X VTOR=0x%X DEMCR=0x%X\n", get_dhcsr(), get_vtor(), get_demcr());
    // clear the VCATCH in DFSR
    set_dfsr((uint32_t)get_dfsr());
    // un-halt the CPU
    set_dhcsr(0);
    // wait until VTOR changes
    while ((! state.arm.vtor ) || (state.arm.vtor == UNKNOWN))
      state.arm.vtor = get_vtor();
    dhcsr = get_dhcsr();
    // refresh the state
    goto done;
  }
  // if the core is running, there's nothing to do
  if (! (dhcsr & DHCSR_NOT_RUNNING))
    return 0;
  // if the core is locked up, there's also nothing to do
  if (dhcsr & DHCSR_S_LOCKUP)
    return 2;
  // if the core is sleeping, halt it first
  if (! (dhcsr & DHCSR_S_HALT)) {
    set_dhcsr(dhcsr | DHCSR_C_HALT);
  }
  // here, we're neither running, sleeping nor locked up - just unhalt
  set_dhcsr(dhcsr & ~DHCSR_C_HALT);
  dhcsr = get_dhcsr();
done:
  // confirm whether we succeeded
  ddprintf("DHCSR=0x%08X\n", dhcsr);
  // return 3 if we failed, 1 if we successfully resumed
  return (dhcsr & DHCSR_NOT_RUNNING) ? 3 : 1;
}

//=============================================================================

int get_arm_state(uint8_t *resp, int force_all)
{
  // update ARM state
  // output:
  //   <0: critical JTAG error
  //   0: running
  //   1: sleeping
  //   2: lockup/halted
  //   3: exception
  // if state >= 1, pc is available and the core is halted
  // if state >= 2, all regs are available
  // if state >= 3, the exception state is also available
  int i = arm_update(force_all);
  int j = 0;
  // update the IMC console
  if (state.imc.buf)
    j = console_update();
  // if we halted the arm ourselves, unhalt it
  if (state.cfg.halted_by_us) {
    dprintf ("unhalting (self-halted)\n");
    set_dhcsr(state.arm.dhcsr & ~DHCSR_C_HALT);
  }
  // if we've been sleeping, in __write_imc(), pretend we had been running
  uint32_t pc = state.arm.regs.pc & ~1;
  if ((i == 1) &&
      (pc >= state.cfg.fn_imc_write.start) && (pc <  state.cfg.fn_imc_write.end))
  {
    // furthermore, if we had been sleeping in __write_imc(),
    // don't admit to having slept
    // (so that only dhcsr would be reported)
    i = 0;
  }
  // the response is structured as
  // {
  //   reg_sz, imc_sz : uint16_t;
  //   regs : {
  //     uint32_t dhcsr;
  //     // if not running (sleeping/lockup/halted/exception)
  //     uint32_t pc;
  //     // if not running/sleeping (lockup/halted/exception)
  //     uint32_t systick, dfsr, icsr;
  //     uint32_t r[14], psr, ctrl_primask;
  //     // if an exception happened
  //     uint32_t except_r0[4], except_r12[4], psr;
  //   }
  //   imc : char[imc_sz];
  // }
  // align the output pointer (it's unlikely this would do anything TBF)
  unsigned n_padding = 0;
  while ((ptrdiff_t)resp & 3) {
    *resp++ = 0;
    ++n_padding;
  }
  uint32_t *p = (uint32_t*)(resp + 4);
  *p++ = state.arm.dhcsr;
  if ((i >= 1) || force_all) {
    *p++ = state.arm.regs.pc;
    if (i >= 2) {
      *p++ = state.arm.systick;
      *p++ = state.arm.dfsr;
      *p++ = state.arm.icsr;
      memcpy(p, &state.arm.regs.r[0], 0x3c); p += 15; // 15regs x 4bytes/reg
      *p++ = state.arm.regs.psr;
      *p++ = state.arm.regs.ctrl_primask;
      if (i > 2) {
        memcpy(p, &state.arm.except.r0, 0x10); p += 4; // 4regs x 4bytes/reg
        *p++ = state.arm.except.r12;
        *p++ = state.arm.except.sp;
        *p++ = state.arm.except.lr;
        *p++ = state.arm.except.pc;
        *p++ = state.arm.except.psr;
      }
    }
  }
  j = state.imc.sz_data;
  // set the header
  unsigned arm_state_size = (ptrdiff_t)p - (ptrdiff_t)(resp + 4);
  ((uint16_t*)resp)[0] = arm_state_size;
  ((uint16_t*)resp)[1] = j;
  // if we have anything in the console, append that
  if (j)
    memcpy(p, state.imc.data, j);
  return n_padding + 4 + arm_state_size + j;
}
