#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "arm.h"
#include "cmd.h"
#include "utils.h"

//#define DEBUG // print what the commands are doing
//#define DEBUG_VERBOSE // print what's happening while scanning the chains

//#define MIRROR_IMC // mirror the IMC output

#define LOCAL_IMC_BUF_SIZE 16384 // have a 16K text buffer

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
    // all cases
    uint32_t systick;     // [+00] SYSTICK.CVR
    uint32_t dhcsr;       // [+04] DBG.DHCSR
    // sleeping et al.
    uint32_t pc;          // [+08] ARM.PC
    // locked/halted and crashed
    union {
      uint32_t _r[21];
      struct {
        uint32_t r[13];   // [+0C..3C]: ARM.r[0..12]
        uint32_t sp;      // [+40] ARM.sp (r13)
        uint32_t lr;      // [+44] ARM.lr (r14)
        uint32_t dfsr;    // [+48] DBG.DFSR (pc/r15 already stored at +4)
        uint32_t psr;     // [+4C] ARM.psr
        uint32_t msp;     // [+50] ARM.msp
        uint32_t psp;     // [+54] ARM.psp (not used)
        uint32_t icsr;    // [+58] SCB.ICSR
        uint32_t ctrl_primask;  // [+5C] ARM.CTRL / ARM.PRIMASK
      };
    };
    // crashed (DFSR.VCATCH) only
    union {
      uint32_t _on_stack[8];
      struct {
        uint32_t r[4];    // [+60..6C] EX.r[0..3] @sp+0,4,8,C
        uint32_t r12;     // [+70] EX.r12 @sp+10
        uint32_t lr;      // [+74] EX.lr  @sp+14
        uint32_t pc;      // [+78] EX.pc  @sp+18
        uint32_t psr;     // [+7C] EX.psr @sp+1c
        uint32_t sp;      // [+80] EX.(r13); computed, ARM.sp + (EX.psr.a ? 4 : 0)
      };
    } except;
    // internal only
    uint32_t last_systick; // previous value of systick
    uint32_t vtor;
  } arm;
  struct {
    // address of in-chip data
    uint32_t ptr_buf, sz_buf; // address and size of the buffer
    uint32_t ptr_wrpos_count;
    uint32_t ptr_rdpos;
    // most recently seen values
    uint32_t last_wrpos_count;
    uint32_t last_rdpos;
    // work buffer
    char *buf; // copy of the relevant bits of the on-chip buffer
    // flag saying that we don't really have an IMC
    // (gets set if VTOR was set, but the pointer is null)
    int no_imc;
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
  // currently available IMC data
  char *imc_data; // buffer
  unsigned sz_imc_data; // total capacity
  unsigned n_imc_data; // used capacity
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

static uint32_t get_arm_reg(unsigned i, const char *name) {
  // issue a read request
  int e = set_cpu_reg(DBG_DCRSR, DCRSR_READ(i), "DCRSR");
  if (e) {
    // try once more
    e = set_cpu_reg(DBG_DCRSR, DCRSR_READ(i), "DCRSR");
    if (e) {
      printf("failed to set DCRSR while attemping to read ARM.%s!\n", name);
      return UNKNOWN;
    }
  }
  // wait for the result
  uint32_t val = get_cpu_reg(DBG_DHCSR, "DHCSR");
  if ((val == UNKNOWN) || ! (val & DHCSR_S_REGRDY)) {
    // try once more
    val = get_cpu_reg(DBG_DHCSR, "DHCSR");
    if ((val == UNKNOWN) || ! (val & DHCSR_S_REGRDY)) {
      printf("failed waiting for DHCSR.S_REGRDY while attemping to read ARM.%s!\n", name);
      return UNKNOWN;
    }
  }
  // fetch the result
  val = get_cpu_reg(DBG_DCRDR, "DCRDR");
  if (val == UNKNOWN) {
    // try once more
    val = get_cpu_reg(DBG_DCRDR, "DCRDR");
    if (val == UNKNOWN) {
      printf("failed to get DCRDR while attemping to read ARM.%s!\n", name);
      return UNKNOWN;
    }
  }
  ddprintf ("ARM.%s: 0x%X\n", name, val);
  return val;
}

static int set_arm_reg(unsigned i, uint32_t val, const char *name) {
  // issue a write request
  int e = set_cpu_reg(DBG_DCRSR, DCRSR_WRITE(i), "DCRSR");
  if (e) {
    // try once more
    e = set_cpu_reg(DBG_DCRSR, DCRSR_WRITE(i), "DCRSR");
    if (e) {
      printf("failed to set DCRSR while attemping to write ARM.%s!\n", name);
      return UNKNOWN;
    }
  }
  // issue the write
  e = set_cpu_reg(DBG_DCRDR, val, "DCRDR");
  if (e) {
    // try once more
    e = set_cpu_reg(DBG_DCRDR, val, "DCRDR");
    if (val == UNKNOWN) {
      printf("failed to set DCRDR while attemping to write ARM.%s!\n", name);
      return UNKNOWN;
    }
  }
  // wait for confirmation
  uint32_t res = get_cpu_reg(DBG_DHCSR, "DHCSR");
  if ((res == UNKNOWN) || ! (res & DHCSR_S_REGRDY)) {
    // try once more
    res = get_cpu_reg(DBG_DHCSR, "DHCSR");
    if ((res == UNKNOWN) || ! (res & DHCSR_S_REGRDY)) {
      printf("failed waiting for DHCSR.S_REGRDY while attemping to write ARM.%s!\n", name);
      return UNKNOWN;
    }
  }
  ddprintf ("ARM.%s:=0x%X\n", name, res);
  return 0;
}

static uint32_t get_ahb_reg(unsigned addr, const char *name) {
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

static uint32_t set_ahb_reg(unsigned addr, uint32_t val, const char *name) {
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
{ return get_ahb_reg(state.imc.ptr_wrpos_count, "IMC.WRPOS"); }
static uint32_t get_imc_buf(uint32_t ofs)
{ return get_ahb_reg(state.imc.ptr_buf + ofs, "IMC.BUF"); }

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
{ return set_ahb_reg(state.imc.ptr_rdpos, x | 0x10000, "IMC.RDPOS"); }
  
static void update_systick()
{
  // take this opportunity to read SYSTICK
  uint32_t systick = get_systick();
  if (systick != UNKNOWN) {
    state.arm.last_systick = state.arm.systick;
    state.arm.systick = systick;
  }
}

static const char *armregnames[] = {
  "r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
  "r8", "r9", "r10", "r11", "r12", "sp", "lr", "pc",
  "psr", "msp", "psp", "res19", "ctrl/primask"
};

static const char *armexnames[] = {
  "ex.r0", "ex.r1", "ex.r2", "ex.r3",
  "ex.r12", "ex.lr", "ex.pc", "ex.psr"
};

// update state.arm
// returns:
//   <0: error
//    0: running (systick/dhcsr available)
//    1: sleeping (pc also available)
//    2: lockup (all arm regs, +dfsr/icsr, also available)
//    3: halted (same as lockup)
//    4: crashed (stack dump and reconstructed sp also available)
static int arm_update()
{
  //---------------------------------------------------------------------------
  // read DHCSR first
  uint32_t dhcsr = get_dhcsr();
  if (dhcsr == UNKNOWN)
    return -1;
  state.arm.dhcsr = dhcsr;
  // if the core isn't sleeping, read SYSTICK
  if (! (dhcsr & DHCSR_S_SLEEP))
    update_systick();

  state.cfg.halted_by_us = 0;
  //---------------------------------------------------------------------------
  // if we haven't initialized the IMC yet, do so
  if (! state.imc.ptr_buf && ! state.imc.no_imc) {
    // if we're really early, wait for the initial system configuration to
    // complete (the cue is that the ARM vector table address is no longer 0)
    // however, it seems VTOR can't be read while the ARM is sleeping, so halt it first
    if ((dhcsr & DHCSR_S_SLEEP)) {
      if (set_dhcsr(dhcsr | DHCSR_C_HALT))
        return -3;
      update_systick();
      dhcsr |= DHCSR_C_HALT | DHCSR_S_HALT;
      state.cfg.halted_by_us = 1;
    }
    uint32_t vtor = get_vtor();
    if (vtor == UNKNOWN) {
      printf ("cannot read VTOR!\n");
      goto check_arm_state;
    }
    state.arm.vtor = vtor;
    // if VTOR wasn't set up yet, try again next time
    if (! vtor) {
      dprintf ("VTOR hasn't been set yet!\n");
      goto check_arm_state;
    }
    // read the IMC buffer pointer and size
    uint32_t imc_addr = vtor + offsetof(struct vectors_s, imc_tx);
    uint32_t buf_ptr, buf_sz;
    int e;
    buf_ptr = get_ahb_reg(imc_addr + offsetof(struct imc_s, buf_addr), "ICM.BUF_ADDR");
    buf_sz = get_ahb_reg(imc_addr + offsetof(struct imc_s, buf_size), "IMC.BUF_SIZE");
    if ((buf_ptr == UNKNOWN_DATA) || (buf_sz == UNKNOWN_DATA))
      return -2;
    // detect if we don't really have an IMC - very unlikely, but we don't want to try autodetection
    // every time, when there's none
    if (!buf_ptr || !buf_sz) {
      printf ("Seems there's no IMC!\n");
      state.imc.no_imc = 1;
      goto check_arm_state;
    }

    state.imc.ptr_buf = buf_ptr;
    state.imc.sz_buf = buf_sz;
    state.imc.ptr_wrpos_count = imc_addr + offsetof(struct imc_s, wrpos_count);
    state.imc.ptr_rdpos = imc_addr + offsetof(struct imc_s, rdpos);
    // allocate a buffer for the in-chip buffer
    unsigned sz = (buf_sz + 3) & ~3;
    if (state.imc.buf)
      free (state.imc.buf);
    state.imc.buf = (char*)malloc(sz);
    // setup the pointer for wrpos/count, so we won't have to re-compute it later
    dprintf("IMC buffer 0x%04X..%04X, ctrl@0x%04X(wrpos@+0x%X, rdpos@+0x%X)\n",
           buf_ptr, buf_ptr + buf_sz - 1, imc_addr,
           state.imc.ptr_wrpos_count-imc_addr, state.imc.ptr_rdpos-imc_addr);
  }
  //---------------------------------------------------------------------------
check_arm_state:
  // if the ARM is running, not much left to do
  if (! (dhcsr & DHCSR_NOT_RUNNING))
    return 0;
  //---------------------------------------------------------------------------
  // in any other state, we need at the very least the PC
  // - halt ARM first
  if (! (dhcsr & DHCSR_S_HALT)) {
    if (set_dhcsr(dhcsr | DHCSR_C_HALT))
      return -3;
    update_systick();
    dhcsr |= DHCSR_C_HALT | DHCSR_S_HALT;
    state.cfg.halted_by_us = 1;
  }
  // grab the current PC
  state.arm.pc = get_arm_reg(REG_PC, "pc");
  // if the core is sleeping, return (leaving the core halted)
  if (((state.arm.dhcsr & DHCSR_NOT_RUNNING) == DHCSR_S_SLEEP)) {
    dprintf ("arm sleeping, pc=0x%X\n", state.arm.pc);
    return 1;
  }
  // in any other situation, read the other regs too
  for (int i = 0; i < countof(state.arm._r); ++i)
    if ((i != REG_PC) && (i != 19)) // we already have PC, and r19 is undefined
      state.arm._r[i] = get_arm_reg(i, armregnames[i]);
  // fill in the state.arm._r gaps
  // - dfsr where pc (r15) would've been
  state.arm.dfsr = get_dfsr();
  // - icsr where r19 (reserved) would've been
  state.arm.icsr = get_icsr();
  // - vtor won't be sent
  // if we hit a lockup
  if ((state.arm.dhcsr & DHCSR_NOT_RUNNING) == DHCSR_S_LOCKUP) {
    printf("arm lockup, pc=0x%X, dhcsr=0x%X, dfsr=0x%X\n", 
      state.arm.pc, state.arm.dhcsr, state.arm.dfsr);
    return 2;
  }
  // did the core get halted?
  if ((state.arm.dfsr == UNKNOWN) || ! (state.arm.dfsr & DFSR_VCATCH)) {
    dprintf("arm halted?, pc=0x%X, dhcsr=0x%X, dfsr=0x%X\n", 
           state.arm.pc, state.arm.dhcsr, state.arm.dfsr);
    return 3;
  }
  // if we got here, we've got an exception
  // if SP is valid, fetch the 8 words on the stack too
  uint32_t sp = state.arm.sp;
  if ((sp >= RAM_START) && ((sp+0x20) <= RAM_END)) {
    for (int i = 0; i < countof(state.arm.except._on_stack); ++i)
      state.arm.except._on_stack[i] = get_ahb_reg(sp + i*4, armexnames[i]);
    // rebuilt the sp at exception time
    state.arm.except.sp = UNKNOWN;
    if (state.arm.except.psr != UNKNOWN)
      state.arm.except.sp = sp + ((state.arm.except.psr & PSR_a) ? 0x24 : 0x20);
  }
  printf("arm crashed!, pc=0x%X(0x%X), sp=0x%X(0x%X) dhcsr=0x%X, dfsr=0x%X, icsr=0x%X\n",
         state.arm.pc, state.arm.except.pc,
         state.arm.sp, state.arm.except.sp,
         state.arm.dhcsr, state.arm.dfsr, state.arm.icsr);
  return 4;
}

//=============================================================================

int console_update()
{
  // bail out if console isn't initialized yet
  char *buf = state.imc.buf;
  if (! buf)
    return 0;
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
      update_systick();
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
    for (unsigned i = 0; i < bsz; i += 4)
      *(uint32_t*)(buf + i) = get_imc_buf(i);
    dprintf("console_update: wrpos/cnt=%u:%u,last=%u:%u; rdpos=%u,next=%u+%u; n=%u (of %u)\n", 
      crt_wrpos_count & 0xffff, crt_wrpos_count >> 16, 
      state.imc.last_wrpos_count & 0xffff, state.imc.last_wrpos_count >> 16,
      last_rdpos, (new_rdpos>=bsz)?bsz:0, (new_rdpos>=bsz)?(new_rdpos-bsz):new_rdpos, n, new);
    imc_printf ("imc(%u)={", 5+n);
    // add an ellipsis
    add_imc_text(5, "[...]");
    // print the oldest data (in the 2nd part of buf)
    add_imc_text(bsz - last_rdpos, buf + last_rdpos);
    // print the newer data (in the 1st part of buf)
    if (new_rdpos)
      add_imc_text(new_rdpos, buf + 0);
    imc_printf ("}\n");
    // set the count to the buffer size, plus the ellipsis length
    n = 5 + bsz;
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
      if (j >= bsz) {
        wa = 1;
        j -= bsz;
      }
      *(uint32_t*)(buf + j) = get_imc_buf(j);
    }
    // handle wrap-arounds
    imc_printf ("imc(%u)={", n);
    if (! wa)
      add_imc_text(n, buf + last_rdpos);
    else {
      add_imc_text(bsz - last_rdpos, buf + last_rdpos);
      add_imc_text(new_rdpos - bsz, buf + 0);
    }
    imc_printf ("}\n");
  }
  if (new_rdpos > bsz)
    new_rdpos -= bsz;
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
  if (n)
    imc_printf ("%.*s", n, s);
  // don't try to stuff in more text than we have room for
  if (! state.imc_data)
    n = 0;
  if (n > state.sz_imc_data - state.n_imc_data)
    n = state.sz_imc_data - state.n_imc_data;
  if (! n)
    return;
  memcpy(state.imc_data + state.n_imc_data, s, n);
  state.n_imc_data += n;
}

//=============================================================================

void arm_init(
  uint32_t imc_write, unsigned sz_imc_write,
  uint32_t reset_addr, unsigned dbg_bit, unsigned m0_bit)
{
  // cleanup most recent state
  free_console_buffers();
  memset(&state, 0, sizeof(state));
  state.imc_data = (char*)malloc(LOCAL_IMC_BUF_SIZE);
  if (state.imc_data)
    state.sz_imc_data = LOCAL_IMC_BUF_SIZE;
  // setup the imc_write() address range
  imc_write &= ~1;
  state.cfg.fn_imc_write.start = imc_write;
  state.cfg.fn_imc_write.end = imc_write + sz_imc_write;
  dprintf("registered imc_write @0x%04X..%04X\n", 
    imc_write, imc_write + sz_imc_write - 1);
  // setup the reset config
  if (reset_addr & ~3) {
    int active_low = reset_addr & 1;
    state.cfg.reset.active_low = active_low;
    reset_addr &= ~3;
    state.cfg.reset.addr = reset_addr;
    if (dbg_bit < 32)
      state.cfg.reset.dbg_bit = 1 << dbg_bit;
    if (m0_bit < 32)
      state.cfg.reset.m0_bit = 1 << m0_bit;
    // read the current resets (no error checking)
    uint32_t crt_rsts;
    crt_rsts = get_ahb_reg(reset_addr, "QTILE.RST");
    if (crt_rsts == UNKNOWN_DATA)
      crt_rsts = 0;
    dprintf("reset reg at 0x%08X, active %s; dbg=0x%X, m0=0x%X\n",
      reset_addr, (reset_addr&1)?"low":"high",
      state.cfg.reset.dbg_bit, state.cfg.reset.m0_bit);
    // assert resets for both the M0 and the M0 debug unit
    if (! active_low)
      crt_rsts |= state.cfg.reset.dbg_bit | state.cfg.reset.m0_bit;
    else
      crt_rsts &= ~(state.cfg.reset.dbg_bit | state.cfg.reset.m0_bit);
    set_ahb_reg(reset_addr, crt_rsts, "QTILE.RST");
    // de-assert the M0 debug unit reset, keeping the M0 main reset
    if (! active_low)
      crt_rsts &= ~state.cfg.reset.dbg_bit;
    else
      crt_rsts |= state.cfg.reset.dbg_bit;
    set_ahb_reg(reset_addr, crt_rsts, "QTILE.RST");
  }
  // enable DEBUGEN/C_HALT in DHCSR, so we could program DEMCR
  set_dhcsr(DHCSR_C_HALT);
  // set the core to halt on leaving RESET
  set_demcr(DEMCR_VC_CORERST /*| DEMCR_VC_HARDERR*/);
}

int arm_resume(void)
{
  uint32_t dhcsr = get_dhcsr();
  if (dhcsr == UNKNOWN)
    return -1;
  state.arm.dhcsr = dhcsr;
  ddprintf("DHCSR=0x%08X\n", dhcsr);
  // if the core has C_HALT set, but S_HALT clear: we're in reset, so remove the reset
  if ((dhcsr & (DHCSR_C_HALT | DHCSR_S_HALT)) == DHCSR_C_HALT) {
    uint32_t crt_rsts = get_ahb_reg(state.cfg.reset.addr, "QTILE.RST");
    if (state.cfg.reset.active_low) {
      // set the RST#.M0 bit
      if (! (crt_rsts & state.cfg.reset.m0_bit)) {
        dprintf("removing m0 reset#\n");
        set_ahb_reg(state.cfg.reset.addr, crt_rsts | state.cfg.reset.m0_bit, "QTILE.RST");
      }
    } else {
      // set the RST.M0 bit
      if (crt_rsts & state.cfg.reset.m0_bit) {
        dprintf("removing m0 reset\n");
        set_ahb_reg(state.cfg.reset.addr, crt_rsts & ~state.cfg.reset.m0_bit, "QTILE.RST");
      }
    }
    // paint registers r0..r15 (excluding pc and sp) with junk values
    unsigned rval = 0x01010101;
    for (int i = 0; i < REG_PC; ++i) {
      if (i != REG_SP)
        set_arm_reg(i, rval, armregnames[i]);
      rval += 0x01010101;
    }
    dhcsr = get_dhcsr();
    state.arm.vtor = get_vtor();
    uint32_t dfsr = get_dfsr();
    dprintf("after leaving reset: DHCSR=0x%X DFSR=0x%X VTOR=0x%X DEMCR=0x%X\n",
            dhcsr, dfsr, state.arm.vtor, get_demcr());
    // clear the VCATCH in DFSR
    set_dfsr(dfsr);
    // un-halt the CPU
    if (set_dhcsr(dhcsr & ~DHCSR_C_HALT))
      return -2;
  }
  if (dhcsr == UNKNOWN)
    return -3;
  // if the core is running, there's nothing to do
  if (! (dhcsr & DHCSR_NOT_RUNNING))
    return 0;
  // if the core is locked up, there's also nothing to do
  if (dhcsr & DHCSR_S_LOCKUP)
    return 2;
  // if the core is sleeping (not locked up/halted), halt it first
  if (! (dhcsr & DHCSR_S_HALT))
    set_dhcsr(dhcsr | DHCSR_C_HALT);
  // here, we're halting (potentially because we were sleeping) - 
  // - unhalt to clear both
  set_dhcsr(dhcsr & ~DHCSR_C_HALT);
  dhcsr = get_dhcsr();
  if (dhcsr != UNKNOWN)
    state.arm.dhcsr = dhcsr;
  // confirm whether we succeeded
  ddprintf("DHCSR=0x%08X\n", dhcsr);
  // return 3 if we didn't succeed, 1 if we successfully resumed
  // note: 3 is very likely if we're attempting to resume while in a tight
  //  sleep loop, such as when the ARM is waiting for an IRQ to happen, or on
  //  the debugger to do something, so it doesn't necessarily indicate
  //  failure
  return (dhcsr & DHCSR_NOT_RUNNING) ? 3 : 1;
}

//=============================================================================

int get_arm_state(uint8_t *resp)
{
  // update ARM state
  // output:
  //  <0: critical JTAG error
  //   0: running
  //   1: sleeping
  // 2/3: lockup/halted
  //   4: exception
  // if state >= 0, systick and dhcsr are available
  // if state >= 1, pc is available and the core is halted
  // if state >= 2, all regs are available
  // if state >= 4, the exception state is also available
  int i = arm_update();
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
  uint32_t pc = state.arm.pc & ~1;
  if ((i == 1) &&
      (pc >= state.cfg.fn_imc_write.start) && (pc < state.cfg.fn_imc_write.end))
  {
    // furthermore, if we had been sleeping in __write_imc(),
    // don't admit to having slept (so that only systick/dhcsr would be reported)
    i = 0;
  }
  // if we have an output buffer, prepare the response
  // NOTE: we'll use this without a buffer if we enable continuous polling
  if (resp) {
    // align the output pointer (it's unlikely this would do anything TBF)
    unsigned n_padding = 0;
    while ((ptrdiff_t)resp & 3) {
      *resp++ = 0xCD;
      ++n_padding;
    }
    unsigned state_size = 4*2; // systick, dhcsr
    if (i >= 1)
      state_size += 4*1; // add pc
    if (i >= 2)
      state_size += 4*21; // add r0..12, sp, lr, dfsr, psr, msp, psp, icsr, ctrl/primask
    if (i >= 4)
      state_size += 4*9; // add the exception structure
    // 
    unsigned max_imc_len = MAX_RESP_SIZE - (4 + state_size);
    int j = state.n_imc_data;
    if (j > max_imc_len)
      j = max_imc_len;
    // setup the header
    ((uint16_t*)resp)[0] = state_size;
    ((uint16_t*)resp)[1] = j;
    // copy over the state data
    memcpy(resp + 4, &state.arm, state_size);
    // if we have imc data, copy it over
    if (j) {
      memcpy(resp + 4 + state_size, state.imc_data, j);
      // if we've more data which we couldn't copy
      if (state.n_imc_data > j)
        memmove(state.imc_data, state.imc_data + j, state.n_imc_data - j);
      state.n_imc_data -= j;
    }
    return n_padding + 4 + state_size + j;
  }
}

void free_console_buffers()
{
  if (state.imc.buf) {
    free(state.imc.buf);
    state.imc.buf = NULL;
  }
  if (state.imc_data) {
    free(state.imc_data);
    state.imc_data = NULL;
  }
}