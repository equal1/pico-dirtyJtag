/*
  Copyright (c) 2017 Jean THOMAS.
  Copyright (c) 2020-2022 Patrick Dussud
  
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
#include <stdbool.h>
#include <string.h>

#include <pico/stdlib.h>
#include <pico/bootrom.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>
#include <hardware/spi.h>

#include "jtag.pio.h"
#include "tusb.h"
#include "pio_jtag.h"
#include "git.h"
#include "cmd.h"

//#define cmd_printf(...) printf(__VA_ARGS__)
#define cmd_printf(...) (void)(__VA_ARGS__)

enum CommandIdentifier {
  CMD_STOP = 0x00,
  CMD_INFO = 0x01,
  CMD_FREQ = 0x02,
  CMD_XFER = 0x03,
  CMD_SETSIG = 0x04,
  CMD_GETSIG = 0x05,
  CMD_CLK = 0x06,
  CMD_SETVOLTAGE = 0x07, // not implemented
  CMD_GOTOBOOTLOADER = 0x08,
  CMD_REBOOT = 0x09, // DirtyJTAG extension
  CMD_GETCLKS = 0x0a, // DirtyJTAG extension
  CMD_A5FREQ = 0x0b, // DirtyJTAG extension: set A5 clock frequency
  CMD_A5CLK = 0x0c,  // DirtyJTAG extension: enable/disable A5 clock
  CMD_PINCFG_SET_ALL = 0x0d, // DirtyJTAG extension: configure all pins
  CMD_PINCFG_SET = 0x0e, // DirtyJTAG extension: configure a specific pin (or all pins)
  CMD_PINCFG_GET = 0x0f, // DirtyJTAG extension: configure a specific pin
  CMD_PINS_SET = 0x10, // DirtyJTAG extension: set all pins' output value
  CMD_PINS_GET = 0x11, // DirtyJTAG extension: get all pins' input value
  CMD_PIN_SET = 0x12, // DirtyJTAG extension: set a single pin's output value
  CMD_PIN_GET = 0x13, // DirtyJTAG extension: read a single pin's input value
};

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

//#


// GPIOs on the PICO itself
static const uint32_t pico_pins = 
  (1 << PIN_RST) | (1 << PIN_A5_CLK) | // a5 reset#, sysclk
  (1 << PIN_TMS) | (1 << PIN_TCK) | (1 << PIN_TDO) | (1 << PIN_TDI) |  // a5 jtag signals
  (1 << PIN_A5_UART_TX) | (1 << PIN_A5_UART_RX) | // a5 gpio 0, 1
  (1 << PIN_A5_SSn) | (1 << PIN_A5_SCK) | (1 << PIN_A5_MOSI) | (1 << PIN_A5_MISO) | // a5 gpio 2..5
  (1 << PIN_A5_GPIO6) | (1 << PIN_A5_GPIO7) | (1 << PIN_A5_GPIO10) | // a5 gpio 6, 7, 10
  (1 << PIN_A5_SCANIN) | (1 << PIN_A5_SCANOUT); // a5 scanin/scanout
// GPIOs on the IOX
static const uint32_t iox_pins = 
  (1 << (PIN_A5_CLKSRC & ~0x40))   |
  (1 << (PIN_A5_TILESEL0 & ~0x40)) | (1 << (PIN_A5_TILESEL1 & ~0x40)) |
  (1 << (PIN_A5_TESTEN & ~0x40))   |
  (1 << (PIN_A5_GPIO8 & ~0x40))    | (1 << (PIN_A5_GPIO9 & ~0x40))    |
  (1 << (PIN_A5_GPIO11 & ~0x40))   | (1 << (PIN_A5_GPIO12 & ~0x40))   | 
  (1 << (PIN_A5_GPIO13 & ~0x40))   | (1 << (PIN_A5_GPIO14 & ~0x40))   |
  (1 << (PIN_A5_GPIO15 & ~0x40))   | (1 << (PIN_A5_GPIO16 & ~0x40));

static const char *pico_signames[32] = {
  // on Pico
  [PIN_RST] =        "RST#",
  [PIN_A5_CLK] =     "CLK",
  [PIN_TMS] =        "TMS",
  [PIN_TCK] =        "TCK",
  [PIN_TDO] =        "TDO",
  [PIN_TDI] =        "TDI",
  [PIN_A5_UART_TX] = "TX",
  [PIN_A5_UART_RX] = "RX",
  [PIN_A5_SSn] =     "SS#",
  [PIN_A5_SCK] =     "SCK",
  [PIN_A5_MOSI] =    "MOSI",
  [PIN_A5_MISO] =    "MISO",
  [PIN_A5_GPIO6] =   "GPIO6",
  [PIN_A5_GPIO7] =   "GPIO7",
  [PIN_A5_GPIO10] =  "GPIO10",
  [PIN_A5_SCANIN] =  "SCANIN",
  [PIN_A5_SCANOUT] = "SCANOUT",
}, *iox_signames[16] = {
  //on IOX
  [PIN_A5_CLKSRC & ~0x40] =   "CLKSRC",
  [PIN_A5_TILESEL0 & ~0x40] = "TILESEL0",
  [PIN_A5_TILESEL1 & ~0x40] = "TILESEL1",
  [PIN_A5_TESTEN & ~0x40] =   "TESTEN",
  [PIN_A5_GPIO8 & ~0x40]  =   "GPIO8",
  [PIN_A5_GPIO9 & ~0x40]  =   "GPIO9",
  [PIN_A5_GPIO11 & ~0x40] =   "GPIO11",
  [PIN_A5_GPIO12 & ~0x40] =   "GPIO12",
  [PIN_A5_GPIO13 & ~0x40] =   "GPIO13",
  [PIN_A5_GPIO14 & ~0x40] =   "GPIO14",
  [PIN_A5_GPIO15 & ~0x40] =   "GPIO15",
  [PIN_A5_GPIO16 & ~0x40] =   "GPIO16",
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
  // CMD_PINS_SET, CMD_PINS_GET
  IOX_PINS = 0x80,
};

enum SignalIdentifier {
  SIG_TCK = 1 << 1,
  SIG_TDI = 1 << 2,
  SIG_TDO = 1 << 3,
  SIG_TMS = 1 << 4,
  SIG_TRST = 1 << 5,
  SIG_SRST = 1 << 6
};

unsigned cmd_execute(pio_jtag_inst_t* jtag, char buf,const uint8_t *cmdbuf, unsigned cmdsz, uint8_t *respbuf)
{
  unsigned cmdpos = 0, resppos = 0;
  const char *djtag_whoami();
  extern struct djtag_clk_s djtag_clocks;
  int n, m;
  while (cmdpos < cmdsz) {
    uint8_t cmd = cmdbuf[cmdpos];
    if (cmd == CMD_STOP)
      break;
    switch (cmd) {

    case CMD_SETVOLTAGE:
      // this one is a dummy
      cmd_printf (" %c# @%u CMD_SETVOLTAGE (no-op)\n", buf, cmdpos);
      ++cmdpos;
      break;

    case CMD_GOTOBOOTLOADER:
      cmd_printf (" %c# @%u CMD_BOOTLOADER\n", buf, cmdpos);
      puts ("Rebooting to bootloader...");
      sleep_ms(200);
      reset_usb_boot(0, 0);
      while (1)
        asm volatile ("wfe");
      break;

    case CMD_REBOOT:
      cmd_printf (" %c# @%u CMD_REBOOT\n", buf, cmdpos);
      puts ("Rebooting...");
      sleep_ms(200);
      watchdog_reboot(0, 0, 1); // standard boot in 1ms
      while (1)
        asm volatile ("wfe");
      break;

    case CMD_INFO:
      n = strlen(djtag_whoami());
      cmd_printf (" %c# @%u CMD_INFO >%u\n", buf, cmdpos, n);
      memcpy(respbuf+resppos, djtag_whoami(), n);
      resppos += n;
      ++cmdpos;
      break;

    case CMD_GETCLKS:
      cmd_printf (" %c# @%u CMD_GETCLKS >%u\n", buf, cmdpos, sizeof(djtag_clocks));
      memcpy(respbuf+resppos, &djtag_clocks, sizeof(djtag_clocks));
      resppos += sizeof(djtag_clocks);
      ++cmdpos;
      break;

    case CMD_FREQ:
      cmd_printf (" %c# @%u CMD_FREQ\n", buf, cmdpos);
      jtag_set_clk_freq(jtag, ((unsigned)cmdbuf[cmdpos+1] << 8) | cmdbuf[cmdpos+2]);
      cmdpos += 3;
      break;

    case CMD_A5FREQ:
      cmd_printf (" %c# @%u CMD_A5FREQ\n", buf, cmdpos);
      extern pio_a5clk_inst_t a5clk;
      a5clk_set_freq(&a5clk, ((unsigned)cmdbuf[cmdpos+1] << 8) | cmdbuf[cmdpos+2]);
      cmdpos += 3;
      break;

    case CMD_PINCFG_SET_ALL:
      int cfg = cmdbuf[cmdpos+1];
      static const char *drvstrs[4] = { "2mA", "4mA", "8mA", "12mA" };
      static const char *pulls[4] = { "none", "low", "high", "keep" };
      cmd_printf (" %c# @%u CMD_PINCFG_SET_ALL SLEW=%s DRIVE=%s PULL=%s HYSTERESIS=%s\n", buf, cmdpos,
        (cfg & PINCFG_SLEW_RATE_MASK) ? "fast" : "slow",
        drvstrs[(cfg & PINCFG_DRIVE_STRENGTH_MASK) >> PINCFG_DRIVE_STRENGTH_POS],
        pulls[(cfg & PINCFG_PULL_MASK) >> PINCFG_PULL_POS],
        (cfg & PINCFG_HYSTERESIS_ON) ? "on" : "off" );
      // foreach pin on PICO
      for (int pin = 0; pin < 32; ++pin) {
        if (pico_pins & (1 << pin)) {
          // apply slew rate
          gpio_set_slew_rate(pin, (cfg & PINCFG_SLEW_RATE_MASK) >> PINCFG_SLEW_RATE_POS);
          // apply drive strength
          gpio_set_drive_strength(pin, (cfg & PINCFG_DRIVE_STRENGTH_MASK) >> PINCFG_DRIVE_STRENGTH_POS);
          // apply pulls
          gpio_set_pulls(pin, cfg & PINCFG_PULL_HIGH, cfg & PINCFG_PULL_LOW);
          // apply hysteresis
          gpio_set_input_hysteresis_enabled(pin, cfg & PINCFG_HYSTERESIS_ON);
        }
      }
      break;

    case CMD_PINCFG_SET:
      int valid_pin = 0, iox_pin = 0;
      const char *get_pin_location(unsigned pin);
      const char *get_pin_name(unsigned pin);
      int pin = cmdbuf[cmdpos+1];
      cfg = cmdbuf[cmdpos+2];
      const char *loc = get_pin_location(pin);
      const char *sig = get_pin_name(pin);
      if (pin < 32)
        valid_pin = pico_pins & (1u << pin);
      else if (pin < 0x40)
        valid_pin = 0;
      else if (pin < 0x40 + 16)
        valid_pin = iox_pins & (1u << (pin-0x40)),
        iox_pin = 1;
      cmd_printf (" %c# @%u CMD_PINCFG_SET %u(%s.%s) SLEW=%s DRIVE=%s PULL=%s HYSTERESIS=%s\n", buf, cmdpos,
        pin, loc ? loc : "???", sig ? sig : "???",
        (cfg & PINCFG_SLEW_RATE_MASK) ? "fast" : "slow",
        drvstrs[(cfg & PINCFG_DRIVE_STRENGTH_MASK) >> PINCFG_DRIVE_STRENGTH_POS],
        pulls[(cfg & PINCFG_PULL_MASK) >> PINCFG_PULL_POS],
        (cfg & PINCFG_HYSTERESIS_ON) ? "on" : "off" );
      if (valid_pin) {
        // PICO pins
        if(! iox_pin) {
          // apply slew rate
          gpio_set_slew_rate(pin, (cfg & PINCFG_SLEW_RATE_MASK) >> PINCFG_SLEW_RATE_POS);
          // apply drive strength
          gpio_set_drive_strength(pin, (cfg & PINCFG_DRIVE_STRENGTH_MASK) >> PINCFG_DRIVE_STRENGTH_POS);
          // apply pulls
          gpio_set_pulls(pin, cfg & PINCFG_PULL_HIGH, cfg & PINCFG_PULL_LOW);
          // apply hysteresis
          gpio_set_input_hysteresis_enabled(pin, cfg & PINCFG_HYSTERESIS_ON);
        }
        // IOX pins
        else {
          // todo
        }
      }
      cmdpos += 3;
      break;

    case CMD_PINCFG_GET:
      valid_pin = 0, iox_pin = 0;
      pin = cmdbuf[cmdpos+1];
      loc = get_pin_location(pin);
      sig = get_pin_name(pin);
      if (pin < 32)
        valid_pin = pico_pins & (1u << pin);
      else if (pin < 0x40)
        valid_pin = 0;
      else if (pin < 0x40 + 16)
        valid_pin = iox_pins & (1u << (pin-0x40)),
        iox_pin = 1;
      cmd_printf (" %c# @%u CMD_PINCFG_GET %u(%s.%s) >1\n", buf, cmdpos,
        pin, loc ? loc : "???", sig ? sig : "???");
      unsigned result = 0;
      if (valid_pin) {
        result = 0x8000;
        // PICO pins
        if(! iox_pin) {
          // get slew rate
          result |= gpio_get_slew_rate(pin) ? PINCFG_SLEW_RATE_FAST : PINCFG_SLEW_RATE_SLOW;
          // get drive strength
          result |= (gpio_get_drive_strength(pin) << PINCFG_DRIVE_STRENGTH_POS) & PINCFG_DRIVE_STRENGTH_MASK;
          // get pulls
          result |= gpio_is_pulled_up(pin) ? PINCFG_PULL_HIGH : 0;
          result |= gpio_is_pulled_down(pin) ? PINCFG_PULL_LOW : 0;
          // get hysteresis
          result |= gpio_is_input_hysteresis_enabled(pin) ? PINCFG_HYSTERESIS_ON : PINCFG_HYSTERESIS_OFF;
          // get direction
          result |= gpio_is_dir_out(pin) ? PINCFG_DIR_OUT : PINCFG_DIR_IN;
          // get value (either what we drive, or the external input)
          result |= gpio_get(pin) ? PINCFG_VALUE_HIGH : PINCFG_VALUE_LOW;
          // get function (should be SIO for most, PIO0 for TDI/TDO/TMS/TCK, PIO1 for A5CLK)
          result |= (gpio_get_function(pin) << PINCFG_FN_POS) & PINCFG_FN_MASK;
        }
        // IOX pins
        else {
          // todo
          result = 0;
        }
      }
      respbuf[resppos++] = result;
      cmdpos += 2;
      break;

    case CMD_GETSIG:
      cmd_printf (" %c# @%u CMD_PINCFG_GET >1\n", buf, cmdpos);
      n = 0;
      if (jtag_get_tdo(jtag))
        n |= SIG_TDO;
      respbuf[resppos++] = n;
      ++cmdpos;
      break;

    case CMD_SETSIG:
      cmd_printf (" %c# @%u CMD_SETSIG\n", buf, cmdpos);
      n = cmdbuf[cmdpos+1]; // mask
      m = cmdbuf[cmdpos+2]; // status
      if (n & SIG_TCK)
        jtag_set_clk(jtag, m & SIG_TCK);
      if (n & SIG_TDI)
        jtag_set_tdi(jtag, m & SIG_TDI);
      if (n & SIG_TMS)
        jtag_set_tms(jtag, m & SIG_TMS);
      if (n & SIG_TRST)
        jtag_set_trst(jtag, m & SIG_TRST);
      if (n & SIG_SRST)
        jtag_set_rst(jtag, m & SIG_SRST);
      cmdpos += 3;
      break;

    case CMD_A5CLK:
    case CMD_A5CLK|TURN_ON:
      cmd_printf (" %c# @%u CMD_A5CLK %s\n", buf, cmdpos, (cmd == CMD_A5CLK) ? "OFF" : "ON");
      extern pio_a5clk_inst_t a5clk;
      a5clk.enabled = cmd != CMD_A5CLK;
      //static int last_slew_rate = 0, last_drive_strength = 0;
      //// if turning on the 1st time, set slew rate to low, drive strength to 2mA
      //if (! djtag_clocks.a5clk_en) {
      //  gpio_set_slew_rate (PIN_A5_CLK, last_slew_rate = 0);
      //  gpio_set_drive_strength (PIN_A5_CLK, last_drive_strength = 0);
      //} else {
      //  if (! last_slew_rate)
      //    gpio_set_slew_rate (PIN_A5_CLK, last_slew_rate = 1);
      //  else {
      //    gpio_set_slew_rate (PIN_A5_CLK, last_slew_rate = 0);
      //    ++last_drive_strength;
      //    if (last_drive_strength >= 4)
      //      last_drive_strength = 0;
      //    gpio_set_drive_strength (PIN_A5_CLK, last_drive_strength);
      //  }
      //}
      djtag_clocks.a5clk_en = a5clk.enabled;
      pio_sm_set_enabled(a5clk.pio, a5clk.sm, a5clk.enabled);
      ++cmdpos;
      break;

    case CMD_CLK:
    case CMD_CLK|READOUT:
      n = cmdbuf[cmdpos+1]; // signals
      m = cmdbuf[cmdpos+2]; // number of clock pulses
      cmd_printf (" %c# @%u CMD_CLK%s %u%s\n", buf, cmdpos, (cmd & READOUT)?",READOUT":"", m, (cmd & READOUT)?" >1":"");
      n = jtag_strobe(jtag, m, n & SIG_TMS, n & SIG_TDI);
      cmdpos += 3;
      if (cmd & READOUT)
        respbuf[resppos++] = n;
      break;

    case CMD_XFER:
    case CMD_XFER|EXTEND_LENGTH:
    case CMD_XFER|NO_READ:
    case CMD_XFER|EXTEND_LENGTH|NO_READ:
      n = cmdbuf[cmdpos+1]; // number of bits to transfer
      if (cmd & EXTEND_LENGTH)
        n += 256;
      m = (n + 7) / 8;
      if (! (cmd & NO_READ))
        cmd_printf (" %c# @%u CMD_XFER%s,NOREAD %u\n", buf, cmdpos,
                    (n >= 256)?",LARGE":"", n);
      else
        cmd_printf (" %c# @%u CMD_XFER%s %u >%u\n", buf, cmdpos,
                    (n >= 256)?",LARGE":"", n, m);
      // we can probably get away with using the response buffer even if the
      // read isn't requested
      jtag_transfer(jtag, n, cmdbuf+cmdpos+2, respbuf+resppos);
      if (! (cmd & NO_READ))
        resppos += m;
      cmdpos += m + 2;
      break;

    default:
      // invalid command: reboot
      // also: print the entire command byte, not just cmd[6:0]
      cmd_printf (" %c# @%u ??? 0x%02X\n", buf, cmdpos, cmdbuf[cmdpos]);
      watchdog_reboot(0, 0, 100); // standard boot in 100ms (give time to UART to flush)
      while (1)
        asm volatile ("wfe");
      break;
    }
  }
  // protocol forbids responses that are a multiple of 64 bytes, sof if that was the case, add one extra byte
  // there is one obvious exception, the null response
  if (resppos && ! (resppos & 63)) {
    cmd_printf (" %c# resp_sz=0x%X; padding\n", buf, resppos);
    respbuf[resppos++] = 0xA5;
  }
  if (resppos) {
    cmd_printf (" %c# [", buf);
    for (int i = 0; i < resppos; ++i ) {
      cmd_printf (" %02X", respbuf[i]);
    }
    cmd_printf (" ]\n");
  }
  cmd_printf (" %c# done, cmd_sz=%u(%u) resp_sz=0x%u\n", buf, cmdpos, cmdsz, resppos);
  return resppos;
}

#define IOX_DO_RD 0x80
#define IOX_DO_WR 0x00

#define IOX_CMD_GET 0x00 // read gpios
#define IOX_CMD_SET 0x02 // set output value; defaults to 1(high)
//#define IOX_CMD_RDINV 0x04 // invert input polarity; defaults to 0(off)
#define IOX_CMD_CFG 0x06 // config gpios; defaults to 1(inputs)
//#define IOX_CMD_PULLUP 0x08 // pull-up enable; defaults to 0(off)
//#define IOX_CMD_INTEN 0x0a // interrupt enable; defaults to 0(off)
//#define IOX_CMD_HIZ 0x0c // output Hi-Z; defaults to 0(driven outputs)
//#define IOX_CMD_INTST 0x0e // interrupt status
//#define IOX_CMD_INTPOS 0x10 // enable interrupt on positive edge; defaults to 0(off)
//#define IOX_CMD_INTNEG 0x12 // enable interrupt on negative edge; defaults to 0(off)
//#define IOX_CMD_INTFLT 0x14 // input filtering (ignore <225ns pulses, acknowledge >1075ns; anything inbetween may or may not be filtered); defaults to 1(on)

int32_t iox_readcmd_all(unsigned cmd) {
  const uint8_t
    cmd_l[2] = { IOX_DO_RD | ((cmd | 0) << 1), 0xff},
    cmd_h[2] = { IOX_DO_RD | ((cmd | 1) << 1), 0xff};
  uint8_t resp_l[2], resp_h[2];
  // read in the low, then high, values
  if (2 != spi_write_read_blocking(SPI_IOX, cmd_l, resp_l, 2))
    return -1;
  if (2 != spi_write_read_blocking(SPI_IOX, cmd_h, resp_h, 2))
    return -1;
  uint32_t result = (((uint32_t)resp_h[1]) << 8) | resp_l[1];
  return result;
}

int32_t iox_writecmd_all(unsigned cmd, uint32_t all) {
  uint8_t
    cmd_l[2] = { IOX_DO_WR | ((cmd | 0) << 1), all & 0xFF },
    cmd_h[2] = { IOX_DO_WR | ((cmd | 1) << 1), (all >> 8) & 0xFF };
  // wrute the low, then high, values
  if (2 != spi_write_blocking(SPI_IOX, cmd_l, 2))
    return -1;
  if (2 != spi_write_blocking(SPI_IOX, cmd_h, 2))
    return -2;
  return 0;
}

// get all the pins
// (for output pins, we get the intended output value, not the actual value)
int32_t iox_get_all() {
  return iox_readcmd_all(IOX_CMD_GET);
}

// set all the pins
// (no effect for input pins)
static uint32_t last_output_values = 0xffffffff;
int32_t iox_set_all(uint32_t all) {
  last_output_values = all;
  return iox_writecmd_all(IOX_CMD_SET, all);
}

// configure all the pins (0=out, 1=in)
static uint32_t last_config_values = 0xffffffff;
int32_t iox_config_all(uint32_t all) {
  last_config_values = all;
  return iox_writecmd_all(IOX_CMD_CFG, all);
}

const char *get_pin_location(unsigned pin)
{
  if (pin < 32)
    return "pico";
  else if (pin < 0x40)
    return 0;
  else if (pin < 0x40 + 16)
    return "iox";
  else
    return 0;
}

const char *get_pin_name(unsigned pin)
{
  int valid_pin = 0, iox_pin = 0;
  if (pin < 32)
    return pico_signames[pin];
  else if (pin < 0x40)
    return 0;
  else if (pin < 0x40 + 16)
    return pico_signames[pin - 0x40];
  else
    return 0;
}
