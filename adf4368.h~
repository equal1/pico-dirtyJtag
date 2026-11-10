#pragma once

#include <stdint.h>

//-[ frequency generator api ]------------------------------------------------

void fpll_init();

// reconfigure the SPI frequency
int do_fpll_config(unsigned freq_khz);

// do a single SPI transaction
int do_fpll_wrrd(const uint8_t *out, uint8_t *in, unsigned n);

// set a single register
int do_fpll_set(unsigned addr, uint8_t val);

// get a single register
int do_fpll_get(unsigned addr);

//-[ data structures ]--------------------------------------------------------

extern int fpll_spi_speed;
extern int fpll_chip_version;

//-[ chip stuff ]-------------------------------------------------------------

// register 0
#define FPLL_R00_SOFT_RESET   0x81 // reset SPI regs, except r00; self-clearing; see FG_R31_RST_SYS
#define FPLL_R00_LSB_FIRST    0x42
#define FPLL_R00_MSB_FIRST    0x00
#define FPLL_R00_ADDR_AUTOINC 0x24
#define FPLL_R00_ADDR_AUTODEC 0x00
#define FPLL_R00_MODE_SPI     0x18
#define FPLL_R00_MODE_3WIRE   0x00

// register 1
#define FPLL_R01_STREAMING_DISABLED    0x80
#define FPLL_R01_STREAMING_ENABLED     0x00
#define FPLL_R01_READBACK_MAIN         0x20
#define FPLL_R01_READBACK_SUBORDINATE  0x00

// chip identification (RO registers)
// chip type = 0x6; product id = 0x0007; spi revision = 0x01; vendor id = 0x0456
// register 3
#define FPLL_R03_CHIP_TYPE_MASK  0x0f
#define FPLL_R03_CHIP_TYPE_VAL   0x06
// registers 4, 5
#define FPLL_R05_PROD_ID_HI_MASK  0xff
#define FPLL_R05_PROD_ID_HI_VAL   0x00
#define FPLL_R04_PROD_ID_LO_MASK  0xff
#define FPLL_R04_PROD_ID_LO_VAL   0x07
// register 0x6
#define FPLL_R06_PROD_GRADE_MASK    0xf0
#define FPLL_R06_PROD_GRADE_VAL     0x00
#define FPLL_R06_DEV_REVISION_MASK  0x0f
#define FPLL_R06_DEV_REVISION_VAL   0x00
// register 0xB
#define FPLL_R0B_SPI_REVISION_MASK  0xff
#define FPLL_R0B_SPI_REVISION_VAL   0x01
// registers 0xC, 0xD
#define FPLL_R0D_VEN_ID_HI_MASK  0xff
#define FPLL_R0D_VEN_ID_HI_VAL   0x04
#define FPLL_R0C_VEN_ID_LO_MASK  0xff
#define FPLL_R0C_VEN_ID_LO_VAL   0x56

// register 0xA is a scratchpad
#define FPLL_R0A_SCRATCHPAD_MASK  0xff

// registers 0x10, 0x11
// writes to r10 trigger autocalibration if EN_AUTOCAL
#define FPLL_R11_CLKDIV_8       0xC0
#define FPLL_R11_CLKDIV_4       0x80
#define FPLL_R11_CLKDIV_2       0x40
#define FPLL_R11_CLKDIV_NONE    0x00
#define FPLL_R11_MODE_INT       0x20
#define FPLL_R11_MODE_FRAC      0x00
#define FPLL_R11_INV_CLKOUT     0x10
#define FPLL_R11_N_INT_HI_MASK  0x0f
#define FPLL_R10_N_INT_LO_MASK  0xff

// registers 0x12, 0x13, 0x14, 0x15
// frac1word is 25-bit, {r15[0], r14, r13, r12}
#define FPLL_R15_VCO_CORE(n)         (((n)&3)<<6) // applies when O_VCO_CORE=1
#define FPLL_R15_VCO_CORE_HIGHEST    0xC0
#define FPLL_R15_VCO_CORE_LOWEST     0x00
#define FPLL_R15_VCO_BIAS(n)         (((n)&0xf)<<2) // applies when O_VCO_BIAS=1
#define FPLL_R15_VCO_BIAS_MAX        0x3C
#define FPLL_R15_VCO_BIAS_MIN        0x00
#define FPLL_R15_CMOS_3V3            0x02
#define FPLL_R15_CMOS_1V8            0x00
#define FPLL_R15_FRAC1WORD_B24_MASK  0x01
#define FPLL_R14_FRAC1WORD_HI_MASK   0xff
#define FPLL_R13_FRAC1WORD_MID_MASK  0xff
#define FPLL_R12_FRAC1WORD_LO_MASK   0xff

// register 0x16: VCO band
#define FPLL_R16_M_VCO_BAND_LOWEST   0xFF
#define FPLL_R16_M_VCO_BAND_HIGHEST  0x00

// registers 0x17, 0x18, 0x19: frac2word
// 24-bit {r19, r18, r17}
#define FPLL_R19_FRAC2WORD_HI_MASK   0xff
#define FPLL_R18_FRAC2WORD_MID_MASK  0xff
#define FPLL_R17_FRAC2WORD_LO_MASK   0xff

// registers 0x1a, 0x1b, 0x1c: mod2word
// 24-bit {r1c, r1b, r1a}
#define FPLL_R1C_MOD2WORD_HI_MASK   0xff
#define FPLL_R1B_MOD2WORD_MID_MASK  0xff
#define FPLL_R1A_MOD2WORD_LO_MASK   0xff

// registers 0x1d, 0x1e
#define FPLL_R1E_PHASE_RESYNC_EN   0x80
#define FPLL_R1E_PHASE_RESYNC_DIS  0x00 
#define FPLL_R1E_REF_RST_EN        0x40 // reset R_DIV via SW_SYNC or Pin Sync
#define FPLL_R1E_REF_RST_DIS       0x00
#define FPLL_R1E_TIMED_SYNC_EN     0x20 // sync signal retimed with reference input clock
#define FPLL_R1E_TIMED_SYNC_DIS    0x00 // RDIV are reset asynchronously
#define FPLL_R1E_BLEED_HI_MASK     0x1f // bleed current {coarse[12:9], fine [8:0]}
#define FPLL_R1D_BLEED_LO_MASK     0xff

// register 0x1f
#define FPLL_R1F_SW_SYNC         0x80 // software sync request
#define FPLL_R1F_PHASE_ADJ       0x40 // apply phase adjustment
#define FPLL_R1F_BLEED_POL_SRC   0x20 // bleed polarity: current source
#define FPLL_R1F_BLEED_POL_SINK  0x00 // bleed polarity: current sink
#define FPLL_R1F_BLEED_EN        0x10 // bleed current enable
// charge pump current [mA]
//  [0]  [1]  [2]  [3]  [4]  [5]  [6]  [7]  [8]  [9] [10] [11] [12] [13] [14]  [15]
// 0.79 0.99 1.19 1.38 1.59 1.98 2.39 2.79 3.18 3.97 4.77 5.57 6.33 7.91 9.51 11.10
#define FPLL_R1F_CP_CURRENT(n)   ((n)&0xF)

// register 0x20
#define FPLL_R20_AUTOCAL_EN      0x80 // enable VCO calibration (triggers on writes to r10)
#define FPLL_R20_REF_DOUBLER_EN  0x40 // enable reference doubler
#define FPLL_R20_RDIV(n)         ((n)&0x3f)

// registers 0x21, 0x22, 0x23: phase_word
// 24-bit {r23, r22, r21}
#define FPLL_R23_PHASE_WORD_HI_MASK   0xff
#define FPLL_R22_PHASE_WORD_MID_MASK  0xff
#define FPLL_R21_PHASE_WORD_LO_MASK   0xff
// register 0x24: phase adjustment: Phase[°]*2^12/360°
#define FPLL_R24_PHASE_ADJ_MASK  0xff

// registers 0x25, 0x26, 0x27: resync_wait
// 24-bit {r27, r26, r25}
#define FPLL_R27_PHASE_WORD_HI_MASK   0xff // time to wait after writes to r10, to apply
#define FPLL_R26_PHASE_WORD_MID_MASK  0xff // resynchronization (resync_wait * PFD)
#define FPLL_R25_PHASE_WORD_LO_MASK   0xff

// register 0x28
#define FPLL_R28_LSB_PLUS1   0x40 // add +1 to SDM LSB enable/disable
#define FPLL_R28_VAR_MOD_EN  0x20 // enable auxiliary SDM

// register 0x29
#define FPLL_R29_CLK2_OPOWER(n)   (((n)&0xf)<<4)
#define FPLL_R29_CLK2_OPOWER_MAX  0xf0
#define FPLL_R29_CLK2_OPOWER_MIN  0x00
#define FPLL_R29_CLK1_OPOWER(n)   ((n)&0xf)
#define FPLL_R29_CLK1_OPOWER_MAX  0x0f
#define FPLL_R29_CLK1_OPOWER_MIN  0x00

// register 0x2A
#define FPLL_R2A_PHASE_ADJ_SUB  0x40 // polarity of phase adjust: subtract
#define FPLL_R2A_PHASE_ADJ_ADD  0x00 // polarity of phase adjust: add
#define FPLL_R2A_SYNC_PD        0x10 // synchronization power-down
#define FPLL_R2A_REFDETECT_PD   0x04 // reference detector power-down

// register 0x2B
#define FPLL_R2B_ALL_PD         0x80 // main power-down
#define FPLL_R2B_LOCKDETECT_PD  0x08 // lock detector power-down
#define FPLL_R2B_CLKOUT1_PD     0x02 // clock output 1 power-down
#define FPLL_R2B_CLKOUT2_PD     0x01 // clock output 2 power-down

// register 0x2C: lock detector pulse window
// LDWIN_PW:
//   0: integer pll, t_BLEED <= 85ps
//   1: integer pll, 85ps < t_BLEED <= 250ps
//   2: fractional pll, f_PFD > 200MHz, RF > 6.4GHz
//   3: fractional pll, f_PFD > 200MHz, RF > 5GHz
//   4: fractional pll, f_PFD < 200MHz
//   5: fractional pll, f_PFD < 100MHz
//   6: fractional pll, f_PFD < 50MHz
//   7: fractional pll, f_PFD < 40MHz
#define FPLL_R2C_LDWIN_PW(n)   (((n)&0x7)<<5)
// LD_COUNT formula is
//   n_PFD = 3+(3+LDWIN_PW[0])*8*(2^LDWIN_PW[4:1])
// 0:  27=3+(3*8*(2^0)) |  8:  387=3+(3*8*(2^4)) | 16:  6147=3+(3*8*(2^8))  | 24:   98307=3+(3*8*(2^12))
// 1:  35=3+(4*8*(2^0)) |  9:  515=3+(4*8*(2^4)) | 17:  8195=3+(4*8*(2^8))  | 25:  131075=3+(4*8*(2^12))
// 2:  51=3+(3*8*(2^1)) | 10:  771=3+(3*8*(2^5)) | 18: 12291=3+(3*8*(2^9))  | 26:  196611=3+(3*8*(2^13))
// 3:  67=3+(4*8*(2^1)) | 11: 1027=3+(4*8*(2^5)) | 19: 16387=3+(4*8*(2^9))  | 27:  262147=3+(4*8*(2^13))
// 4:  99=3+(3*8*(2^2)) | 12: 1539=3+(3*8*(2^6)) | 20: 24579=3+(3*8*(2^10)) | 28:  393219=3+(3*8*(2^14))
// 5: 131=3+(4*8*(2^2)) | 13: 2051=3+(4*8*(2^6)) | 21: 32771=3+(4*8*(2^10)) | 29:  524291=3+(4*8*(2^14))
// 6: 195=3+(3*8*(2^3)) | 14: 3075=3+(3*8*(2^7)) | 22: 49155=3+(3*8*(2^11)) | 30:  786435=3+(3*8*(2^15))
// 7: 259=3+(4*8*(2^3)) | 15: 4099=3+(4*8*(2^7)) | 23: 65539=3+(4*8*(2^11)) | 31: 1048579=3+(4*8*(2^15))
#define FPLL_R2C_LD_COUNT(n)  ((n)&0x1f)

// register 0x2D
#define FPLL_R2D_DIV_NCLK_EN    0x80
#define FPLL_R2D_DIV_RCLK_EN    0x40
#define FPLL_R2D_LOL_DETECT_EN  0x20 // loss of lock detector
#define FPLL_R2D_LDWIN_EN       0x10 // lock detector pulse window
#define FPLL_R2D_LD_RST         0x04 // reset lock detector to unlocked

// register 0x2E
#define FPLL_R2E_MUXOUT(n)          (((n)&0xF)<<4)
#define FPLL_R2E_MUXOUT_HIZ         0x00
#define FPLL_R2E_MUXOUT_LKDET       0x10
#define FPLL_R2E_MUXOUT_LOW         0x20 // also: 0x30, 0x70, (0xB..0xD)<<4
#define FPLL_R2E_MUXOUT_DIV_RCLK_2  0x40
#define FPLL_R2E_MUXOUT_DIV_NCLK_2  0x50
#define FPLL_R2E_MUXOUT_HIGH        0x80
#define FPLL_R2E_CPTEST_EN          0x04 // charge pump test (force up/down) mode
#define FPLL_R2E_CP_DOWN            0x02 // CP force down
#define FPLL_R2E_CP_UP              0x01 // CP force up

// register 0x2F
#define FPLL_R2F_BOOST_REF    0x80 // REF_SEL=1 (low amplitude sine wave reference input): enable boost (input <1.6V pk2pk)
#define FPLL_R2F_FILTER_REF   0x40 // sine wave reference input: enable noise filter
#define FPLL_R2F_REF_SEL_LNA  0x20 // REF_SEL=1: low-noise amplifier
#define FPLL_R2F_REF_SEL_DMA  0x00 // REF_SEL=0: delay-matched amplifier

// register 0x30
#define FPLL_R30_MUTE_NCLK  0x80 // mute N divider output (not set: normal operation)

// register 0x31
#define FPLL_R31_RST_SYS     0x10 // reset digital, exclusive SPI, to POR
#define FPLL_R31_ADC_CLK_EN  0x08 // enable ADC clock

// register 0x35
#define FPLL_R35_DCLK_MODE  0x04 // during VCO calibration: halven RCLK, NCLK frequency

// register 0x36
#define FPLL_R36_CLKOUT_DIV_DB  0x80 // enable double buffering for clkout_div
#define FPLL_R36_DCLK1_DIV_DB  0x80 // enable double buffering for dclk1_div

// register 0x37
// time for each VCO calibration decision: 16*VCO_band_div/(div_Rclk frequency)
#define FPLL_R37_VCO_BAND_DIV_MASK  0xff

// registers 0x38, 0x39
// timeout for calibration DAC settling time during a VCO calibration:
// synth_lock_timeout/(f_div_Rclk frequency)
#define FPLL_R39_O_VCO_DB            0x80 // double buffering for VCO core, bias and band
#define FPLL_R39_SYNTH_LOCK_HI_MASK  0x7f
#define FPLL_R38_SYNTH_LOCK_LO_MASK  0xff

// registers 0x3A, 0x3B
// timeout for auto level control (ALC) during a VCO calibration:
// vco_alc_timeout/(f_div_Rclk frequency)
#define FPLL_R3B_DELAY_CTRL_DB            0x80 // double buffering for delay controls (inv_clkout, bleed_i, bleed_pol)
#define FPLL_R3B_VCO_ALC_TIMEOUT_HI_MASK  0x7f
#define FPLL_R3A_VCO_ALC_TIMEOUT_LO_MASK  0xff

// register 0x3E
// ADC clock divider value: desired ADC clock freq <400KHz
// ??? if adc_clk_div=roundup((f_div_rclk/desired_adc_freq)-2)/4
#define FPLL_R3E_ADC_CLK_DIV_MASK  0xff

// register 0x3F
// if adc_auto_conv is disabled, conversion can be triggered by writting ADC_START_CONV
#define FPLL_R3F_ADC_CONV_EN    0x80 // enable ADC conversion (1: normal operation)
#define FPLL_R3F_ADC_EN         0x02 // enable ADC
#define FPLL_R3F_ADC_AUTO_CONV  0x01 // enable ADC conversion at start of VCO calibration

// register 0x40
#define FPLL_R40_MUTE_CLKOUT2(n)  (((n)&7)<<3)
#define FPLL_R40_MUTE_CLKOUT1(n)  ((n)&7)

// register 0x43
#define FPLL_R43_ADC_CLK_SEL_SCLK  0x40 // use SPI SCLK as ADC clock source (test mode)
#define FPLL_R43_ADC_CLK_SEL_RCLK  0x00 // use Rclk as ADC clock source (normal operation)

// register 0x4E
#define FPLL_R4E_DCLK_DIV1(n)  (((n)&3)<<4) // control div_Rclk div1, div_Nclk div1
#define FPLL_R4E_O_VCO_BAND    0x08 // override VCO band (1: from M_VCO_BAND; 0: from calibration SM)
#define FPLL_R4E_O_VCO_CORE    0x04 // override VCO core (1: from M_VCO_CORE; 0: from calibration SM)
#define FPLL_R4E_O_VCO_BIAS    0x02 // override VCO bias (1: from M_VCO_BIAS; 0: from calibration SM)

// register 0x53
#define FPLL_R53_SYNC_MON_PD    0x40 // sysref setup/hold monitor power-down
#define FPLL_R53_SYNC_SEL_LVDS  0x20 // sysref source (1: LVDS; 0: CML/PECL)
#define FPLL_R53_SYNC_SEL_LVDS  0x20 // sysref source (1: LVDS; 0: CML/PECL)
#define FPLL_R53_SYNC_MON_RST   0x10 // clear output latch of sysref setup/hold monitor

// register 0x54
#define FPLL_R54_ADC_START_CONV  0x01 // manually start ADC conversion

// register 0x58 [RO]
#define FPLL_R58_CLK2_EN   0x80 // logic state of EN_CLK2 pin
#define FPLL_R58_CLK1_EN   0x40 // logic state of EN_CLK1 pin
#define FPLL_R58_SYNC_OK   0x20 // sysref is correctly set up wrt reference
#define FPLL_R58_REF_OK    0x08 // reference input ok (amplitude above threshold)
#define FPLL_R58_ADC_BUSY  0x04 // ADC conversion in progress
#define FPLL_R58_SM_BUSY   0x02 // VCO calibration in progress
#define FPLL_R58_LOCK      0x01 // lock detector output

// register 0x5A [RO]
#define FPLL_R5A_VCO_CORE_MASK  0x03 // VCO core readback value

// registers 0x5A, 0x5B [RO]
// temperature measured by ADC
#define FPLL_R5B_CHIP_TEMP_HI_MASK  0x01
#define FPLL_R5A_CHIP_TEMP_LO_MASK  0xff

// register 0x5E [RO]
#define FPLL_R5E_VCO_BAND_MASK  0xff // VCO band readback value

// register 0x60 [RO]
#define FPLL_R60_VCO_BIAS_MASK  0x0f // VCO bias readback value

// register 0x63
#define FPLL_R63_VERSION_MASK  0xff
//#define FPLL_R63_VERSION_VAL   0x00 // datasheet value
#define FPLL_R63_VERSION_VAL   0x03 // actual value

#define FPLL_N_REGS 0x64
