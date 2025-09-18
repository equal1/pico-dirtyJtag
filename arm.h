#pragma once

#include <stdint.h>

// ARM's DPACC/APACC commands
#define IR_ABORT  0x8 // 4'b1000
#define IR_DPACC  0xA // 4'b1010
#define IR_APACC  0xB // 4'b1011
#define IR_IDCODE 0xE // 4'b1110 // not used, we reply on TLR instead
#define IR_BYPASS 0xF // 4'b1111
#define N_AP 9 // ROM + 4x{CPU,SYS}

// JTAG responses
#define JTAG_WAIT  0x1
#define JTAG_FAULT 0x2
#define JTAG_OK    0x4

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

// DP registers ([7:4]bank [3:2]addr [1:0]=2'b00)
#define DP_IDR        0x00 // RO
#define DP_CTRLSTAT   0x04 // RW
#define DP_SELECT     0x08 // WO // [31:4]=ADDR, [3:0]=DPBANKSEL
#define DP_RDBUFF     0x0C // RO

// AP registers
#define MEMAP_DAR0       0x000 // 256 Data Access Registers; access is relative to {TAR[31:10], 10'h0}
#define MEMAP_CSW        0xD00
#define MEMAP_TAR        0xD04 // Target Address Register
#define MEMAP_DRW        0xD0C // Data Read/Write; access at TAR; supports byte/halfword accesses (config in CSW)
#define MEMAP_BD0        0xD10 // 4 Banked Data Registers; access is relative to {TAR[31:4], 3'h0}
#define MEMAP_BD1        0xD14
#define MEMAP_BD2        0xD18
#define MEMAP_BD3        0xD1C

// values we'll use for regs we're unable to fetch
#define UNKNOWN 0xdeadbeef
#define UNKNOWN_DATA 0x3f3f3f3f // '????'

// System control and ID registers
#define SCB_ICSR   0xE000ED04 // RW: Interrupt Control State
#define SCB_VTOR   0xE000ED08 // RW: Vector Table Offset Register
#define SCB_AIRCR  0xE000ED0C // RW: Application Interrupt and Reset Control
#define SCB_SCR    0xE000ED10 // RW: System Control (optional)
#define SCB_CCR    0xE000ED14 // RO: Configuration and Control
#define SCB_SHPR2  0xE000ED1C // RW: System Handler Priority 2
#define SCB_SHPR3  0xE000ED20 // RW: System Handler Priority 3
#define SCB_SHCSR  0xE000ED24 // RW: System Handler Control and State

#define ICSR_NMIPENDSET (1<<31)
#define ICSR_NMIPEND    ICSR_NMIPENDSET
#define ICSR_PENDSVSET  (1<<28)
#define ICSR_PENDSV     ICSR_PENDSVSET
#define ICSR_PENDSVCLR  (1<<27)
#define ICSR_PENDSTSET  (1<<26)
#define ICSR_PENDST     ICSR_PENDSTSET
#define ICSR_PENDSTCLR  (1<<25)
#define ICSR_ISRPREEMPT (1<<23)
#define ICSR_ISRPENDING (1<<22)
#define ICSR_VECTPENDING_POS  12
#define ICSR_VECTPENDING_MASK 0x1ff
#define ICSR_VECTACTIVE_POS   0
#define ICSR_VECTACTIVE_MASK  0x1ff

// SysTick registers
#define SYST_CSR    0xE010 // RW: SysTick Control and Status
#define SYST_RVR    0xE014 // RW: SysTick Reload Value
#define SYST_CVR    0xE018 // RW: SysTick Current Value
#define SYST_CALIB  0xE01C // RW: SysTick Calibration Value

// Nested Vector Interrupt Controller registers
#define NVIC_ISER  0xE100 // RW: NVIC Interrupt Set-Enable
#define NVIC_ICER  0xE180 // RW: NVIC Interrupt Clear-Enable
#define NVIC_ISPR  0xE200 // RW: NVIC Interrupt Set-Pending
#define NVIC_ICPR  0xE280 // RW: NVIC Interrupt Clear-Pending
#define NVIC_IPR0  0xE400 // RW: NVIC Interrupt Priority #n at IPR0+((n/4)*4), bits 7+(n%4)*8:6+(n%4)*8

// Debug registers
#define DBG_DFSR   0xED30 // RW: Debug Fault Status
#define DBG_DHCSR  0xEDF0 // RW: Debug Halting Control/Status
#define DBG_DCRSR  0xEDF4 // WO: Debug Core Register Selector
#define DBG_DCRDR  0xEDF8 // RW: Debug Core Register Data
#define DBG_DEMCR  0xEDFC // RW: Debug Exception Monitor Control

// DWT registers
#define DWT_CTRL     0x1000 // RO: DWT Control
#define DWT_PCSR     0x101C // RO: DWT PC Sample
#define DWT_COMP(n)  (0x1020+(((n)&0xf)<<4)) // RW: DWT Comparator #n at DWT_COMP0+(n*0x10)
#define DWT_MASK(n)  (0x1024+(((n)&0xf)<<4)) // RW: DWT Comparator Mask #n at DWT_MASK0+(n*0x10)
#define DWT_FN(n)    (0x1028+(((n)&0xf)<<4)) // RW: DWT Comparator Function #n at DWT_FN0+(n*0x10)

// BPU registers
#define BP_CTRL     0x2000 // RO: BP Control
#define BP_COMP(n)  (0x2008+(((n)&0xf)<<2)) // RW: BP Comparator #n at BP_COMP0+(n*4)

// ARMv6-M ROM table
// entries are
// [31:12] : signed base address offset relative to ROM base (0xE00FF000)
//     [1] : 1 (32-bit format)
//     [0] : present (if [31:1]==0: EOL)
// note: we can't really read these on alpha7, since
//   HADDR[31:28] is 4'hF (selects DSU space)
//   HADDR[27:16] selectes the tile (12'h000..003 for tile 0..3, 12'hFFF for top CPU)
#define ROM_SCS   0xFF000 // RO: pointer to SCS @0xE000E000 (expected: 0xFFF0F003)
#define ROM_DWT   0xFF004 // RO: pointer to DWT @0xE0001000 (expected: 0xFFF0200[23])
#define ROM_BPU   0xFF008 // RO: pointer to BPU @0xE0002000 (expected: 0xFFF0300[23])
#define ROM_END   0xFF00C // RO: end of list (expected: 0)

//-[ Debug Halt Control and Status ]-------------------------------------------

// bits on write
#define DHCSR_DBGKEY (0xA05F<<16) // WO, mandatory write value to enable writing (to DHCSR_C_*)
// bits on read
#define DHCSR_S_RESET_ST  (1<<25) // RO, 1 if reset since last DHCSR read (self-clearing)
#define DHCSR_S_RETIRE_ST (1<<24) // RO, 1 if at least 1 instruction executed reset since last DHCSR read (self-clearing)
#define DHCSR_S_LOCKUP    (1<<19) // RO, 1 if locked up by unrecoverable exception
#define DHCSR_S_SLEEP     (1<<18) // RO, 1 if sleeping (must halt/wait for wakeup, to gain control)
#define DHCSR_S_HALT      (1<<17) // RO, 1 if in debug control
#define DHCSR_S_REGRDY    (1<<16) // RO, cleared on a DCRSR write, set on completion of the write
// bits on R/W
#define DHCSR_C_MASKINTS  (1<< 3) // RW, 1=mask PendSV, SysTick, ExtInt; must be changed while C_HALT is set, with C_HALT set
#define DHCSR_C_STEP      (1<< 2) // RW, 1=single_stepping enabled (see ddi0419e, page 282)
#define DHCSR_C_HALT      (1<< 1) // RW, 0=request_run, 1=request_halt (see ddi0419e, page 282)
#define DHCSR_DEBUGEN     (1<< 0) // RW, 0=halting_debug_enable; 0->1 change must write C_MASKINTS to 0

//-[ Debug Fault Status ]-----------------------------------------------------

#define DFSR_EXTERNAL (1<<4) // EDBGRQ asserted
#define DFSR_VCATCH   (1<<3) // vector catch (see DEMCR)
#define DFSR_DWTTRAP  (1<<2) // DWT (data watchpoint trap)
#define DFSR_BKPT     (1<<1) // breakpoint (either BKPT instruction or BPU hit)
#define DFSR_HALTED   (1<<0) // C_HALT or C_STEP

//-[ Debug Core Register Select ]---------------------------------------------

#define DCRSR_READ(i)  ((0<<16) | ((i & 0x1f) << 0))
#define DCRSR_WRITE(i) ((1<<16) | ((i & 0x1f) << 0))

//-[ Debug Exception Monitor Control ]----------------------------------------

#define DEMCR_DWTENA     (1<<24) // DWT enable
#define DEMCR_VC_HARDERR (1<<10) // enabled halting debug trap on HardFault (only while DHCSR.C_DEBUGEN)
#define DEMCR_VC_CORERST (1<< 0) // enabled reset vector catch (only while DHCSR.C_DEBUGEN)

//-[ Debug Core Register Selector ]-------------------------------------------

#define REG_R0      0x00
#define REG_R1      0x01
#define REG_R2      0x02
#define REG_R3      0x03
#define REG_R4      0x04
#define REG_R5      0x05
#define REG_R6      0x06
#define REG_R7      0x07
#define REG_R8      0x08
#define REG_R9      0x09
#define REG_R10     0x0A
#define REG_R11     0x0B
#define REG_R12     0x0C
#define REG_SP      0x0D  // current SP (either MSP or PSP)
#define REG_LR      0x0E
#define REG_PC      0x0F
#define REG_PSR     0x10  // xPSR
#define REG_MSP     0x11  // main SP
#define REG_PSP     0x12  // process SP
#define REG_CONTROL_PRIMASK 0x14 // CONTROL:=[31:23], PRIMASK:=[7:0]
#define N_ARM_REGS (1+REG_CONTROL_PRIMASK)

#define PSR_N (1<<31)
#define PSR_Z (1<<30)
#define PSR_C (1<<29)
#define PSR_V (1<<28)
#define PSR_T (1<<24)
#define PSR_a (1<<9)
#define PSR_EXCEPT_POS  0
#define PSR_EXCEPT_MASK 0x3f

#define RAM_START 0x00000000
#define RAM_END   0x00008000

struct vectors_s {
  uint32_t stack_top;       // [0]
  uint32_t fn_start;        // [1]
  uint32_t fn_nmi;          // [2]
  uint32_t fn_hardfault;    // [3]
  struct imc_s { // [4..7]
    uint32_t wrpos_count;   // [4]
    uint32_t rdpos;         // [5]
    uint32_t buf_addr;      // [6]
    uint32_t buf_size;      // [7]
  } imc_tx;
  struct monitor_s { // [8..10]
    uint32_t pass_fail;     // [8]
    uint32_t read_addr;     // [9]
    uint32_t expected_data; // [10]
  } monitor;
  uint32_t fn_svc;          // [11]
  uint32_t syscfg;          // [12]
  uint32_t sysclk_khz;      // [13]
  uint32_t fn_pendsv;       // [14]
  uint32_t fn_systick;      // [15]
  uint32_t fn_irq[32];      // [16..47]
};
