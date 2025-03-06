#pragma once

#include <stdint.h>
#include <config.h>

// API for dealing with the A5 signals, other than the JTAG/UART/SPI signals

extern int iox_spi_speed;

// initialize A5 JTAG signals (connected directly to the Pico)
int jtag_pins_init();
// initialize A5 JTAG clock signal (connected directly to the Pico)
int a5clk_pin_init();

// initialize A5 signals connected directly to the Pico (other than JTAG)
int a5_pico_pins_init();
// initialize A5 signals connecred to the IO expander
int a5_iox_pins_init();

// actually initialize the IO expander
void iox_init();
// check if the IOX is present
int iox_check();
// print IOX state/config over debug link
void iox_debug();

//-[ commands ]--------------------------------------------------------------

// handle the commands

// configure pins *on the pico* (not really meaningful, since we have the
//   LSs on everything connected to the A5)
// (only applies the slew rate, drive strength, pulls and input hysteresis)
int pincfg_set_all(int cfg);

// configure an individual pin; in addition to slew rate, drive strength,
// pulls and input hysteresis, the ouput value is also set (though it
// doesn't impact input pins)
int pincfg_set(int pin, int cfg, int on_iox);

// get the configuration of an individual pin
int pincfg_get(int pin, int on_iox);

//-[ IOX ops ]---------------------------------------------------------------

// IOX ops
int iox_get_pin_pullup(int);
int iox_set_pin_pullup(int, int);
int iox_get_pin_direction(int);
int iox_set_pin_direction(int, int);
int iox_get_pin_output(int); // this only tells you what the pin is trying to output
int iox_set_pin_output(int, int);
int iox_get_pin_value(int); // this tells you what's really on the pin

// read value for all iox pins
int32_t iox_get_all();
// read config for all iox pins (0=output, 1=input)
int32_t iox_getcfg_all();
// read pullup config for all iox pins (0=output, 1=input)
int32_t iox_getpull_all();
// set output value for all iox pins
int iox_set_all(uint32_t all);
// configure all iox pins (0=out, 1=in)
int iox_config_all(uint32_t all);
// configure all iox pullups (1=enabled)
int iox_pullup_all(uint32_t all);

//-[ pin/pincfg describe ]---------------------------------------------------

struct pindesc_s {
  int pinidx;
  int valid; // pin number is valid
  int on_iox; // pin is on IO expander
  const char *location, *signal; // description of location and signal
};

// populate the pindesc_s structure
int describe_pin(int, struct pindesc_s*);
// return a string describing the configuration
const char *describe_pincfg(int cfg, int with_out, int on_iox);

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

// these only make sense for Pico (RP2040)
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

#define IOX_REG_GSR  (0x00<<1) // GPIO State (read GPIOs) R/O
#define IOX_REG_OCR  (0x01<<1) // Output Control (set outputs)
#define IOX_REG_PIR  (0x02<<1) // input Polarity Inversion; default=0(off)
#define IOX_REG_GCR  (0x03<<1) // GPIO Config; default=1(input)
#define IOX_REG_PUR  (0x04<<1) // internal Pull-Up enable; default=0(off)
#define IOX_REG_IER  (0x05<<1) // Interrupt Enable; defaults=0
#define IOX_REG_TSCR (0x06<<1) // Tri-State Control (output Hi-Z); default=0(driven)
#define IOX_REG_ISR  (0x07<<1) // Interrupt Status R/O
#define IOX_REG_REIR (0x08<<1) // Rising Edge Interrupt enable; default=0(off)
#define IOX_REG_FEIR (0x09<<1) // Falling Edge Interrupt enable; default=0(off)
// input filtering: ignore <225ns pulses, acknowledge >1075ns
// (anything inbetween may or may not be filtered)
#define IOX_REG_IFR  (0x0A<<1) // Input Filtering; default=1(on)

// IOX registers, renamed to make what they do more clear
#define IOX_CMD_GET    IOX_REG_GSR
#define IOX_CMD_SET    IOX_REG_OCR
#define IOX_CMD_CFG    IOX_REG_GCR
#define IOX_CMD_PULLUP IOX_REG_PUR

#define TILESEL_POS  (PIN_A5_TILESEL0&~IOX_PIN_BASE)
#define TILESEL_MASK (0b11 << TILESEL_POS)
#define CLKSRC_POS   (PIN_A5_CLKSRC&~IOX_PIN_BASE)
#define CLKSRC_MASK  (0b1 << CLKSRC_POS)
