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
  CMD_STOP = 0x00,
  CMD_INFO = 0x01,
  CMD_FREQ = 0x02,
  CMD_XFER = 0x03, // opt: [7]=no_read, [6]=extend_length
  CMD_SETSIG = 0x04,
  CMD_GETSIG = 0x05,
  CMD_CLK = 0x06, // opt: [7]=do_read
  CMD_SETVOLTAGE = 0x07, // not implemented
  CMD_REBOOT = 0x07, // use 7 for this instead
  CMD_GOTOBOOTLOADER = 0x08, // opt:
  CMD_REBOOT_OLD = 0x09, // DirtyJTAG extension; TBR with GETCAPS
  CMD_GETCLKS = 0x0a, // DirtyJTAG extension
  CMD_A5FREQ = 0x0b, // DirtyJTAG extension: set A5 clock frequency
  CMD_A5CLK = 0x0c,  // DirtyJTAG extension: enable/disable A5 clock; [7]=turn_on
  CMD_PINCFG_SET_ALL = 0x0d, // DirtyJTAG extension: configure all pins
  CMD_PINCFG_SET = 0x0e, // DirtyJTAG extension: configure a specific pin (or all pins)
  CMD_PINCFG_GET = 0x0f, // DirtyJTAG extension: configure a specific pin
  CMD_ADC_GETREGS = 0x10, // DirtyJTAG extension: get all ADC registers
  CMD_ADC_CHREAD = 0x11, // DirtyJTAG extension: do a read on a single channel
  CMD_ADC_CHREAD_ALL = 0x12, // DirtyJTAG extension: do a read on all 4 channels
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

//-[ pin configuration settings ]---------------------------------------------
// on IOX, only output value, direction and pullup are supported

// note: PINCFG_SET_ALL only changes pins on the pico, and for those, only
// the slew rate, the drive strength, the pull configuration and the input
// hysteresis (direction, output value and pin function won't be changed)

// note: PINCFG_SET can change pins on both pico and the iox
// this command can change the output value, but not the direction or the pin
// function
// additionally, on the iox, the only supported pull configurations are NONE
// and UP (DOWN or HOLD will be ignored)

// pincfg get/set
#define PINCFG_SLEW_RATE_POS (0)
#define PINCFG_SLEW_RATE_MASK (1<<0)
#define PINCFG_SLEW_RATE_SLOW (0<<0) // GPIO_SLEW_RATE_SLOW
#define PINCFG_SLEW_RATE_FAST (1<<0) // GPIO_SLEW_RATE_FAST

#define PINCFG_DRIVE_STRENGTH_POS (1)
#define PINCFG_DRIVE_STRENGTH_MASK (3<<1)
#define PINCFG_DRIVE_STRENGTH_2MA  (0<<1) // GPIO_DRIVE_STRENGTH_2MA
#define PINCFG_DRIVE_STRENGTH_4MA  (1<<1) // GPIO_DRIVE_STRENGTH_4MA
#define PINCFG_DRIVE_STRENGTH_8MA  (2<<1) // GPIO_DRIVE_STRENGTH_8MA
#define PINCFG_DRIVE_STRENGTH_12MA (3<<1) // GPIO_DRIVE_STRENGTH_12MA

#define PINCFG_PULL_POS  (3)
#define PINCFG_PULL_MASK (3<<3)
#define PINCFG_PULL_NONE (0<<3)
#define PINCFG_PULL_LOW  (1<<3)
#define PINCFG_PULL_HIGH (2<<3)
#define PINCFG_PULL_KEEP (3<<3)

#define PINCFG_HYSTERESIS_POS  (5)
#define PINCFG_HYSTERESIS_MASK (1<<5)
#define PINCFG_HYSTERESIS_OFF  (0<<5)
#define PINCFG_HYSTERESIS_ON   (1<<5)

// PINCFG_GET only
#define PINCFG_DIR_POS  (6)
#define PINCFG_DIR_MASK (1<<6)
#define PINCFG_DIR_IN   (0<<6)
#define PINCFG_DIR_OUT  (1<<6)

#define PINCFG_VALUE_POS  (7)
#define PINCFG_VALUE_MASK (1<<7)
#define PINCFG_VALUE_LOW  (0<<7)
#define PINCFG_VALUE_HIGH (1<<7)

#define PINCFG_FN_POS (8)
#define PINCFG_FN_MASK (0x1f<<8)
#define PINCFG_FN_XIP  (   0<<8)
#define PINCFG_FN_SPI  (   1<<8)
#define PINCFG_FN_UART (   2<<8)
#define PINCFG_FN_I2C  (   3<<8)
#define PINCFG_FN_PWM  (   4<<8)
#define PINCFG_FN_SIO  (   5<<8)
#define PINCFG_FN_PIO0 (   6<<8)
#define PINCFG_FN_PIO1 (   7<<8)
#define PINCFG_FN_GPCK (   8<<8)
#define PINCFG_FN_USB  (   9<<8)
#define PINCFG_FN_NULL (0x1f<<8)

//-[ iox api ]----------------------------------------------------------------

// iox spi link
#define IOX_DO_RD 0x80
#define IOX_DO_WR 0x00

// register indices (renamed to make what they do more clear)
// read gpios (GSR, GPIO State)
#define IOX_CMD_GET 0x00
// set outputs (OCR, Output Control)
#define IOX_CMD_SET 0x02
// invert read value (PIR, Input Polarity Inversion)
// note, we only use this for probing whether the IOX is accessible
#define IOX_CMD_RDINV 0x04 // invert input polarity; 0=off
// config gpios (GCR, GPIO Configuration); 1=input
#define IOX_CMD_CFG 0x06
// pull-up enable (PUR, Internal Pull-up Enable), 1=enabled
#define IOX_CMD_PULLUP 0x08 // pull-up enable; defaults to 0(off)

// registers we don't use
//#define IOX_CMD_INTEN 0x0a // interrupt enable; defaults to 0(off)
//#define IOX_CMD_HIZ 0x0c // output Hi-Z; defaults to 0(driven outputs)
//#define IOX_CMD_INTST 0x0e // interrupt status
//#define IOX_CMD_INTPOS 0x10 // enable interrupt on positive edge; defaults to 0(off)
//#define IOX_CMD_INTNEG 0x12 // enable interrupt on negative edge; defaults to 0(off)
//#define IOX_CMD_INTFLT 0x14 // input filtering (ignore <225ns pulses, acknowledge >1075ns; anything inbetween may or may not be filtered); defaults to 1(on)

extern int iox_spi_speed;

int a5_pico_pins_init();
int a5_iox_pins_init();

// check if the IOX is present
int iox_check();
// print IOX state/config over debug link
void iox_debug();

int iox_get_pin_pullup(int);
int iox_set_pin_pullup(int, int);
int iox_get_pin_direction(int);
int iox_set_pin_direction(int, int);
int iox_get_pin_output(int); // this only tells you what the pin is trying to output
int iox_set_pin_output(int, int);
int iox_get_pin_value(int); // this tells you what's really on the pin

// read value for all iox pins
int32_t iox_get_all();
// set output value for all iox pins
int iox_set_all(uint32_t all);
// configure all iox pins (0=out, 1=in)
int iox_config_all(uint32_t all);
// configure all iox pullups (1=enabled)
int iox_pullup_all(uint32_t all);

//-[ adc api ]----------------------------------------------------------------

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

// ADC lock
#define ADC_LOCK_MAGIC 0xA5 // if anything else, write access restricted to the LOCK register

// structure for reporting the reg contents to the client
struct adcregs_s {
  uint32_t adcdata;
  uint8_t config[4];
  uint8_t irq, mux;
  uint32_t scan, timer, offsetcal, gaincal;
  uint8_t lock;
  uint16_t crccfg;
  uint8_t cmd, cmd_resp;
};

extern int adc_spi_speed;
extern int adc_addr;

// check it the ADC is present
int adc_probe();

// read all regs; writes out and adcregs_s structure, returns the number of bytes
int do_adc_getregs(uint8_t *dest);

// read on a specific channel
int do_adc_chread(uint8_t *dest, unsigned ch);
