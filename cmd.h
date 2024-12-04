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
  CMD_ADC_GETREGS = 0x10, // DirtyJTAG extension
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

extern int adc_spi_speed;
extern int adc_addr;

// check it the ADC is present
int adc_probe();
