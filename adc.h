#pragma once

#include <stdint.h>

//-[ adc api ]----------------------------------------------------------------

void adc_init();

// read all regs; writes out and adcregs_s structure, returns the number of bytes
int do_adc_get_regs(uint8_t *dest);

// set a single register
int do_adc_set_reg(int which, uint32_t value);

// do a single conversion
// (works in either mux or scan mode, and returns all the data that got
//  sampled)
// data is provided in sign-extended format, with the topmost byte replaced
//  with either the MUX value, or the MUX value equivalent to the sampled
//  channel
int do_adc_conv(uint8_t *dest);

// do a MUX read
int do_adc_chread_mux(uint8_t *dest, unsigned mux);

// do a SCAN read
int do_adc_chread_scan(uint8_t *dest, unsigned bmp);

//-[ data structures ]--------------------------------------------------------

// structure for reporting the reg contents to the client
struct adcregs_s {
  uint32_t adcdata, crccfg;
  uint32_t scan, timer, offsetcal, gaincal;
  uint8_t config[4];
  uint8_t irq, mux;
  uint8_t lock;
  uint8_t cmd, cmd_resp;
};

extern int adc_spi_speed;
extern int adc_addr;

//-[ chip stuff ]-------------------------------------------------------------

// adc initial command bits
#define ADC_DO_CONV_START     ((0xA)<<2)
#define ADC_DO_STANDBY        ((0xB)<<2)
#define ADC_DO_SHUTDOWN       ((0xC)<<2)
#define ADC_DO_SHUTDOWN_FULL  ((0xD)<<2)
#define ADC_DO_RESET          ((0xE)<<2)
#define ADC_DO_READ(a)        ((((a)&0xF)<<2)|1)
#define ADC_DO_WRITE_BURST(a) ((((a)&0xF)<<2)|2)
#define ADC_DO_READ_BURST(a)  ((((a)&0xF)<<2)|3)

// adc registers
#define ADCR_ADCDATA   0x0
#define ADCR_CONFIG0   0x1
#define ADCR_CONFIG1   0x2
#define ADCR_CONFIG2   0x3
#define ADCR_CONFIG3   0x4
#define ADCR_IRQ       0x5
#define ADCR_MUX       0x6
#define ADCR_SCAN      0x7
#define ADCR_TIMER     0x8
#define ADCR_OFFSETCAL 0x9
#define ADCR_GAINCAL   0xA
#define ADCR_LOCK      0xD
#define ADCR_CRCCFG    0xF

// ADC CONFIG[0] fields

#define ADC_CFG0_VREFSEL_INTERNAL (1<<7) // default; 0->external
#define ADC_CFG0_NOT_PARTIAL_SHUTDOWN (1<<6) // 0->partial shutdown

#define ADC_CFG0_CLKSEL_POS 4
#define ADC_CFG0_CLKSEL_MASK (3<<4)
#define ADC_CFG0_CLKSEL_EXT_DIGICLOCK  (0<<4) // default
#define ADC_CFG0_CLKSEL_EXT_DIGICLOCK1 (1<<4) // ??? doc bug?
#define ADC_CFG0_CLKSEL_INT_NOCLK      (2<<4) // internal clock selected; no clock output on CLK
#define ADC_CFG0_CLKSEL_INT_CLK        (3<<4) // internal clock selected, outputting AMCLK

// current source/sink for sensor bias
// (source on Vin+, sink on Vin-)
#define ADC_CFG0_CSRC_POS 2
#define ADC_CFG0_CSRC_MASK (3<<2)
#define ADC_CFG0_CSRC_NONE   (0<<2) // no current source applied; default
#define ADC_CFG0_CSRC_0_9uA  (1<<2) // 0.9uA applied
#define ADC_CFG0_CSRC_3_7uA  (2<<2) // 3.7uA applied
#define ADC_CFG0_CSRC_15_0uA (3<<2) // 15uA applied

// ADC operating mode
#define ADC_CFG0_MODE_POS 0
#define ADC_CFG0_MODE_MASK (3<<0)
#define ADC_CFG0_MODE_SHUTDOWN   (0<<0) // default
#define ADC_CFG0_MODE_SHUTDOWN1  (1<<0) // ??? doc bug
#define ADC_CFG0_MODE_STANDBY    (2<<0)
#define ADC_CFG0_MODE_CONVERSION (3<<0)

// ADC CONFIG[1] fields

// prescaler
#define ADC_CFG1_PRE_POS 6
#define ADC_CFG1_PRE_MASK (3<<6)
#define ADC_CFG1_PRE_NONE    (0<<6) // AMCLK = MCLK; default
#define ADC_CFG1_PRE_HALF    (1<<6) // AMCLK = MCLK/2
#define ADC_CFG1_PRE_QUARTER (2<<6) // AMCLK = MCLK/4
#define ADC_CFG1_PRE_EIGHTH  (3<<6) // AMCLK = MCLK/8

// oversampling ratio
#define ADC_CFG1_OSR_POS 2
#define ADC_CFG1_OSR_MASK    (0xF<<2)
#define ADC_CFG1_OSR_DEFAULT (3<<2) // oversampling ratio: 256
//static unsigned OSR_vals[] = {
// 32, 64, 128,
// 256, // default (4'b0011
// 512, 1024, 2048, 4096, 8192, 16384,
// 20480, 24576,
// 40960, 49152,
// 81920. 98304
//};

// ADC CONFIG[2] fields

// ADC bias current selection
#define ADC_CFG2_BOOST_POS 6
#define ADC_CFG2_BOOST_MASK (3<<6)
#define ADC_CFG2_BOOST_1_2  (0<<6) // ADC has current * 0.5
#define ADC_CFG2_BOOST_2_3  (1<<6) // ADC has current * 0.66
#define ADC_CFG2_BOOST_1    (2<<6) // ADC has current * 1; default
#define ADC_CFG2_BOOST_2    (3<<6) // ADC has current * 2
#define ADC_CFG2_BOOST_NONE ADC_CFG2_BOOST_1

// ADC gain selection
#define ADC_CFG2_GAIN_POS 3
#define ADC_CFG2_GAIN_MASK (7<<3)
#define ADC_CFG2_GAIN_1_3  (0<<3) // gain is 1/3
#define ADC_CFG2_GAIN_1    (1<<3) // gain is 1; default
#define ADC_CFG2_GAIN_2    (2<<3) // gain is 2
#define ADC_CFG2_GAIN_4    (3<<3) // gain is 4
#define ADC_CFG2_GAIN_8    (4<<3) // gain is 8
#define ADC_CFG2_GAIN_16   (5<<3) // gain is 16
#define ADC_CFG2_GAIN_32   (6<<3) // gain is 32 (analog 16x, digital 2x)
#define ADC_CFG2_GAIN_64   (7<<3) // gain is 64 (analog 16x, digital 4x)
#define ADC_CFG2_GAIN_NONE ADC_CFG2_GAIN_1

// auto-zeroing MUX setting
// when enabled, multiplies conversion time by 2, prevents continuous mode
#define ADC_CFG2_AZ_MUX_EN (1<<2) // default is disabled

// auto-zeroing reference buffer
// when enabled, internal vref chopping enabled; no effect when VREF_SEL=0
// (no effect when external voltage reference is selected)
#define ADC_CFG2_AZ_REF_EN (1<<2) // default

// ADC CONFIG[3] fields

// ADC conversion mode
#define ADC_CFG3_CMODE_POS 6
#define ADC_CFG3_CMODE_MASK (3<<6)
#define ADC_CFG3_CMODE_ONESHOT_SHUTDOWN  (0<<6) // one-shot, goes to shutdown after; default
#define ADC_CFG3_CMODE_ONESHOT_SHUTDOWN1 (1<<6) 
#define ADC_CFG3_CMODE_ONESHOT_STANDBY   (2<<6) // one-shot, goes to standby after
#define ADC_CFG3_CMODE_CONTINUOUS        (3<<6) // continuous

// ADC data format
#define ADC_CFG3_DFMT_POS 4
#define ADC_CFG3_DFMT_MASK (3<<4)
#define ADC_CFG3_DFMT_16BIT           (0<<4) // 16-bit; default
#define ADC_CFG3_DFMT_32BIT_LJUSTIFY  (1<<4) // 32-bit, {data[15:0], 16'b0}
#define ADC_CFG3_DFMT_32BIT_SGNX      (2<<4) // 32-bit, {8'b?, 8x{sgn}, data[15:0]}
#define ADC_CFG3_DFMT_32BIT_SGNX_CHID (3<<4) // 32-bit, {ch_id[3:0], 12x{sgn}, data[15:0]}

// CRC format
#define ADC_CFG3_CRCFMT_32BIT (1<<3) // default: disabled (16-bit)

// CRC enable
#define ADC_CFG3_CRC_EN (1<<2) // default: disabled

// enable digital offset calibration
#define ADC_CFG3_OFFCAL_EN (1<<1) // enable digital offset calibration; default: off

// enable digital gain calibration
#define ADC_CFG3_GAINCAL_EN (1<<0) // enable digital gain calibration; default: off

// ADC IRQ fields

#define ADC_IRQ_NO_NEW_DATA  (1<<6) // DR_STATUS# (since last reading/last reset)
#define ADC_IRQ_NO_CRC_ERROR (1<<5) // CRCCFG_STATUS#
#define ADC_IRQ_NO_POR       (1<<4) // POR_STATUS# (POR has not occured since last read)

// configure the IRQ#/MDAT pin
// with MDAT enabled, modulator output codes available on MDAT pin *and* ADCDATA reg
#define ADC_IRQ_PIN_MDAT_EN (1<<3) // output is MDAT, unless POR/IRQ; if disabled, IRQ# at all times
#define ADC_IRQ_INACTIVE_HIGH_EN (1<<2) // output is high if not IRQ; if disabled, HiZ

#define ADC_IRQ_FASTCMD_EN (1<<1) // enable fast commands; default
#define ADC_IRQ_STP_EN     (1<<0) // enable conversion start interrupt output; default

// ADC MUX fields
#define ADC_MUX_VINP_SEL_POS 4
#define ADC_MUX_VINP_SEL_MASK (0xF<<4)
#define ADC_MUX_VINN_SEL_POS 0
#define ADC_MUX_VINN_SEL_MASK (0xF<<0)

// these are the values for either mux
#define ADC_MUXVAL_CH(n)      ((n)&7) // select channel
#define ADC_MUXVAL_AGND       0x8
#define ADC_MUXVAL_AVDD       0x9
#define ADC_MUXVAL_REFINP_OUT 0xB // REFIN+/OUT
#define ADC_MUXVAL_REFINN     0xC // REFIN-
#define ADC_MUXVAL_DIODE_P    0xD
#define ADC_MUXVAL_DIODE_M    0xE
#define ADC_MUXVAL_VCM        0xF // internal VCM
// note: for reading temperature, the MUX reg value should be
// (ADC_MUXVAL_DIODE_P << ADC_MUX_VINP_SEL_POS) | (ADC_MUXVAL_DIODE_M << ADC_MUX_VINN_SEL_POS)

#define MK_ADC_MUX(a,b) ((((a)&0xf) << ADC_MUX_VINP_SEL_POS) | (((b)&0xf) << ADC_MUX_VINN_SEL_POS))

// ADC lock
#define ADC_LOCK_MAGIC 0xA5 // if anything else, write access restricted to the LOCK register

// channel IDs for SCAN mode
#define ADC_CHID_CH(n)   ((n)&7)
#define ADC_CHID_DIFF(n) (8+((n)&3))
#define ADC_CHID_TEMP    12
#define ADC_CHID_AVDD    13
#define ADC_CHID_AVCM    14
#define ADC_CHID_OFSFET  15

// equivalent mux configs
#define ADC_CHID_MUX_CH(n)   MK_ADC_MUX(ADC_MUXVAL_CH(n), ADC_MUXVAL_AGND)
#define ADC_CHID_MUX_DIFF(n) MK_ADC_MUX(ADC_MUXVAL_CH(2*(n)), ADC_MUXVAL_CH(2*(n)+1))
#define ADC_CHID_MUX_TEMP    MK_ADC_MUX(ADC_MUXVAL_DIODE_P, ADC_MUXVAL_DIODE_M)
#define ADC_CHID_MUX_AVDD    MK_ADC_MUX(ADC_MUXVAL_AVDD, ADC_MUXVAL_AGND)
#define ADC_CHID_MUX_AVCM    MK_ADC_MUX(ADC_MUXVAL_VCM, ADC_MUXVAL_AGND)
#define ADC_CHID_MUX_OFFSET  MK_ADC_MUX(ADC_MUXVAL_AGND, ADC_MUXVAL_AGND)
