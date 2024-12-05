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
#include <pico/binary_info.h>
#include <pico/bootrom.h>
#include <hardware/clocks.h>
#include <hardware/gpio.h>
#include <hardware/watchdog.h>
#include <hardware/spi.h>

#include "dirtyJtagConfig.h"
#include "jtag.pio.h"
#include "tusb.h"
#include "pio_jtag.h"
#include "git.h"
#include "cmd.h"

//#define cmd_printf(...) printf(__VA_ARGS__)
#define cmd_printf(...) (void)(__VA_ARGS__)

// GPIOs on the PICO itself
static const uint32_t pico_pins = 
  (1 << PIN_RST) | (1 << PIN_A5_CLK) | // a5 reset#, sysclk
  (1 << PIN_TMS) | (1 << PIN_TCK) | (1 << PIN_TDO) | (1 << PIN_TDI) |  // a5 jtag signals
  (1 << PIN_A5_UART_TX) | (1 << PIN_A5_UART_RX) | // a5 gpio 0, 1
  (1 << PIN_A5_SSn) | (1 << PIN_A5_SCK) | (1 << PIN_A5_MOSI) | (1 << PIN_A5_MISO) | // a5 gpio 2..5
  (1 << PIN_A5_GPIO6) | (1 << PIN_A5_GPIO10) | // a5 gpio 6,10
  (1 << PIN_A5_SCANIN) | (1 << PIN_A5_SCANOUT) | (1 << PIN_A5_SCANEN); // a5 scan*
// GPIOs on the IOX
static const uint32_t iox_pins = 
  (1 << (PIN_A5_CLKSRC & ~0x40))   |
  (1 << (PIN_A5_TILESEL0 & ~0x40)) | (1 << (PIN_A5_TILESEL1 & ~0x40)) |
  (1 << (PIN_A5_TESTEN & ~0x40))   |
  (1 << (PIN_A5_GPIO7 & ~0x40))    | (1 << (PIN_A5_GPIO8 & ~0x40))    |
  (1 << (PIN_A5_GPIO9 & ~0x40))    | (1 << (PIN_A5_GPIO11 & ~0x40))   |
  (1 << (PIN_A5_GPIO12 & ~0x40))   | (1 << (PIN_A5_GPIO13 & ~0x40))   |
  (1 << (PIN_A5_GPIO14 & ~0x40))   | (1 << (PIN_A5_GPIO15 & ~0x40))   |
  (1 << (PIN_A5_GPIO16 & ~0x40));

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
  [PIN_A5_GPIO10] =  "GPIO10",
  [PIN_A5_SCANIN] =  "SCANIN",
  [PIN_A5_SCANEN] =  "SCANEN",
  [PIN_A5_SCANOUT] = "SCANOUT",
}, *iox_signames[16] = {
  //on IOX
  [PIN_A5_CLKSRC & ~0x40] =   "CLKSRC",
  [PIN_A5_TILESEL0 & ~0x40] = "TILESEL0",
  [PIN_A5_TILESEL1 & ~0x40] = "TILESEL1",
  [PIN_A5_TESTEN & ~0x40] =   "TESTEN",
  [PIN_A5_GPIO7 & ~0x40]  =   "GPIO7",
  [PIN_A5_GPIO8 & ~0x40]  =   "GPIO8",
  [PIN_A5_GPIO9 & ~0x40]  =   "GPIO9",
  [PIN_A5_GPIO11 & ~0x40] =   "GPIO11",
  [PIN_A5_GPIO12 & ~0x40] =   "GPIO12",
  [PIN_A5_GPIO13 & ~0x40] =   "GPIO13",
  [PIN_A5_GPIO14 & ~0x40] =   "GPIO14",
  [PIN_A5_GPIO15 & ~0x40] =   "GPIO15",
  [PIN_A5_GPIO16 & ~0x40] =   "GPIO16",
};

extern volatile int adc_busy, eth_busy;

static void claim_spi_for_adc() {
retry:
  // wait while ethernet is using the interface
  while (eth_busy)
    ;
  adc_busy = 1;
  // make sure we didn't hit a race condition here
  // we only have this check in the ADC code, to avoid
  // deadlocks (effectively giving ETH higher priority)
  if (eth_busy) {
    adc_busy = 0;
    goto retry;
  }
  if (spi_get_baudrate(SPI_ADC) != adc_spi_speed)
    spi_set_baudrate(SPI_ADC, adc_spi_speed);
}

static void release_adc_spi() {
  adc_busy = 0;
}

unsigned cmd_execute(pio_jtag_inst_t* jtag, char buf,const uint8_t *cmdbuf, unsigned cmdsz, uint8_t *respbuf)
{
  unsigned cmdpos = 0, resppos = 0;
  const char *djtag_whoami();
  extern struct djtag_clk_s djtag_clocks;
  int n, m;
  int do_iox_debug = 0;
  while (cmdpos < cmdsz) {
    uint8_t cmd = cmdbuf[cmdpos];
    if (cmd == CMD_STOP)
      break;
    switch (cmd) {

    case CMD_GOTOBOOTLOADER:
      cmd_printf (" %c# @%u CMD_BOOTLOADER\n", buf, cmdpos);
      puts ("Rebooting to bootloader...");
      sleep_ms(200);
      reset_usb_boot(0, 0);
      while (1)
        asm volatile ("wfe");
      break;

    case CMD_REBOOT:
    case CMD_REBOOT_OLD: // need to ditch this, replace with GETCAPS
      cmd_printf (" %c# @%u CMD_REBOOT\n", buf, cmdpos);
      puts ("Rebooting...");
      sleep_ms(200);
      watchdog_reboot(0, 0, 1); // standard boot in 1ms
      while (1)
        asm volatile ("wfe");
      break;

#   if 0
    // keep this in, just a reminder that the code point was actually
    // reused as REBOOT
    case CMD_SETVOLTAGE:
      // this one is a dummy
      cmd_printf (" %c# @%u CMD_SETVOLTAGE (no-op)\n", buf, cmdpos);
      ++cmdpos;
      break;
#   endif

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
      // we ignore direction and value
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
      if (! iox_pin) {
        cmd_printf (" %c# @%u CMD_PINCFG_SET %u(%s.%s) cfg=0x%02X: SLEW=%s DRIVE=%s PULL=%s HYSTERESIS=%s OUTPUT=%u\n", buf, cmdpos,
          pin, loc ? loc : "???", sig ? sig : "???", cfg,
          (cfg & PINCFG_SLEW_RATE_MASK) ? "fast" : "slow",
          drvstrs[(cfg & PINCFG_DRIVE_STRENGTH_MASK) >> PINCFG_DRIVE_STRENGTH_POS],
          pulls[(cfg & PINCFG_PULL_MASK) >> PINCFG_PULL_POS],
          (cfg & PINCFG_HYSTERESIS_ON) ? "on" : "off",
          (cfg >> PINCFG_VALUE_POS) & 1);
      } else {
        cmd_printf (" %c# @%u CMD_PINCFG_SET %u(%s.%s) cfg=0x%02X: PULL=%s VALUE=%u\n", buf, cmdpos,
          pin, loc ? loc : "???", sig ? sig : "???", cfg,
          pulls[(cfg & PINCFG_PULL_HIGH) >> PINCFG_PULL_POS],
          (cfg >> PINCFG_VALUE_POS) & 1);
      }
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
          // apply output value
          // (this doesn't configure the pin as an output, so it only affects outputs)
          gpio_put(pin, cfg & PINCFG_VALUE_HIGH);
        }
        // no such settings on the IOX (with the exception of pullup support)
        // so only apply the value, and the pull-up (that, only if pulldown is not requested,
        // since pull-down and bus-keeper functions are not available)
        else {
          if (((cfg & PINCFG_PULL_MASK) == PINCFG_PULL_HIGH) || 
              ((cfg & PINCFG_PULL_MASK) == PINCFG_PULL_NONE))
          {
            iox_set_pin_pullup(pin & 0xf, cfg & PINCFG_PULL_HIGH);
          }
          iox_set_pin_output(pin & 0xf, cfg & PINCFG_VALUE_HIGH);
        }
        do_iox_debug = 1;
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
      cmd_printf (" %c# @%u CMD_PINCFG_GET %u(%s.%s) >2\n", buf, cmdpos,
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
        // IOX pins don't have pulldown, slew rate, drive strength an hysteresis
        else {
          result |= iox_get_pin_pullup(pin) ? PINCFG_PULL_HIGH : 0;
          result |= (iox_get_pin_direction(pin) == GPIO_OUT) ? PINCFG_DIR_OUT : PINCFG_DIR_IN;
          result |= iox_get_pin_value(pin) ? PINCFG_VALUE_HIGH : PINCFG_VALUE_LOW;
          result |= (PINCFG_FN_NULL << PINCFG_FN_POS) & PINCFG_FN_MASK;
        }
      }
      respbuf[resppos++] = result & 0xff;
      respbuf[resppos++] = result >> 8;
      cmd_printf ("       > %04X\n", result);
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

    case CMD_ADC_CHREAD:
      cmd_printf (" %c# @%u ADC_CHREAD %u\n", buf, cmdpos, cmdbuf[cmdpos+1] & 3);
      if (adc_spi_speed) {
        claim_spi_for_adc();
        resppos += do_adc_chreads(respbuf+resppos, 1 << (cmdbuf[cmdpos+1] & 0xf));
        release_adc_spi();
      } else {
        // send a dummy response, so we won't break the protocol
        memset(respbuf+resppos, 0xff, sizeof(uint32_t));
        resppos += sizeof(uint32_t);
      }
      cmdpos += 2;
      break;

    case CMD_ADC_CHREAD_ALL:
      cmd_printf (" %c# @%u ADC_CHREAD_ALL\n", buf, cmdpos);
      if (adc_spi_speed) {
        claim_spi_for_adc();
        resppos += do_adc_chreads(respbuf+resppos, 0xf); // read CH0..CH3
        release_adc_spi();
      } else {
        // send a dummy response, so we won't break the protocol
        memset(respbuf+resppos, 0xff, 4*sizeof(uint32_t));
        resppos += 4*sizeof(uint32_t);
      }
      ++cmdpos;
      break;

    case CMD_ADC_GETREGS:
      cmd_printf (" %c# @%u ADC_GETREGS\n", buf, cmdpos);
      if (adc_spi_speed) {
        claim_spi_for_adc();
        resppos += do_adc_get_regs(respbuf+resppos);
        release_adc_spi();
      } else {
        // send a dummy response, so we won't break the protocol
        memset(respbuf+resppos, 0xff, sizeof(struct adcregs_s));
        resppos += sizeof(struct adcregs_s);
      }
      ++cmdpos;
      break;

    case CMD_ADC_SETREG:
      uint32_t value =
        ((uint32_t)cmdbuf[cmdpos+2] >> 0 ) | ((uint32_t)cmdbuf[cmdpos+3] >> 8) |
        ((uint32_t)cmdbuf[cmdpos+4] >> 16) | ((uint32_t)cmdbuf[cmdpos+5] >> 24);
      cmd_printf (" %c# @%u ADC_SETREG %u \n", buf, cmdpos, cmdbuf[cmdpos+1] & 0xF, value);
      if (adc_spi_speed) {
        claim_spi_for_adc();
        do_adc_set_reg(cmdbuf[cmdpos+1] & 0xF, value);
        release_adc_spi();
      }
      cmdpos += 6; // cmd + reg + 32bit value
      break;

    default:
      // invalid command: reboot
      // also: print the entire command byte, not just cmd[6:0]
      printf (" %c# @%u ??? 0x%02X\n", buf, cmdpos, cmdbuf[cmdpos]);
      watchdog_reboot(0, 0, 100); // standard boot in 100ms (give time to UART to flush)
      while (1)
        asm volatile ("wfe");
      break;
    }
  }
  if (do_iox_debug)
    iox_debug();
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

static int32_t iox_readcmd_all(unsigned cmd) {
  if (iox_spi_speed < 1000)
    return -1;
  const uint8_t
    cmd_l[2] = { IOX_DO_RD | ((cmd | 0) << 1), 0xff},
    cmd_h[2] = { IOX_DO_RD | ((cmd | 1) << 1), 0xff};
  uint8_t resp_l[2], resp_h[2];
  // read in the low, then high, values
  int a, b;
  if (spi_get_baudrate(SPI_IOX) != iox_spi_speed)
    spi_set_baudrate(SPI_IOX, iox_spi_speed);
  gpio_put(PIN_IOX_SSn, 0);
  a = spi_write_read_blocking(SPI_IOX, cmd_l, resp_l, 2);
  gpio_put(PIN_IOX_SSn, 1);
  gpio_put(PIN_IOX_SSn, 0);
  b = spi_write_read_blocking(SPI_IOX, cmd_h, resp_h, 2);
  gpio_put(PIN_IOX_SSn, 1);
  if ((a != 2) || (b != 2))
    return -1;
  uint32_t result = (((uint32_t)resp_h[1]) << 8) | resp_l[1];
  return result;
}

static int32_t iox_writecmd_all(unsigned cmd, uint32_t all) {
  if (iox_spi_speed < 1000)
    return -1;
  uint8_t
    cmd_l[2] = { IOX_DO_WR | ((cmd | 0) << 1), all & 0xFF },
    cmd_h[2] = { IOX_DO_WR | ((cmd | 1) << 1), (all >> 8) & 0xFF };
  // write the low, then high, values
  int a, b;
  if (spi_get_baudrate(SPI_IOX) != iox_spi_speed)
    spi_set_baudrate(SPI_IOX, iox_spi_speed);
  gpio_put(PIN_IOX_SSn, 0);
  a = spi_write_blocking(SPI_IOX, cmd_l, 2);
  gpio_put(PIN_IOX_SSn, 1);
  gpio_put(PIN_IOX_SSn, 0);
  b = spi_write_blocking(SPI_IOX, cmd_h, 2);
  gpio_put(PIN_IOX_SSn, 1);
  if ((a != 2) || (b != 2))
    return -1;
  return 0;
}

// get all the pins
// (for output pins, we get the intended output value, not the actual value)
int32_t iox_get_all() {
  return iox_readcmd_all(IOX_CMD_GET);
}

// set all the pins
// (no effect for input pins)
static int last_output_values = -1;
int iox_set_all(uint32_t all) {
  last_output_values = all;
  return iox_writecmd_all(IOX_CMD_SET, all);
}

// configure all the pins (0=out, 1=in)
static int last_config_values = -1;
int iox_config_all(uint32_t all) {
  last_config_values = all;
  return iox_writecmd_all(IOX_CMD_CFG, all);
}

// configure pullups on all the pins (0=off, 1=pullup)
static int last_pullup_values = -1;
int iox_pullup_all(uint32_t all) {
  last_pullup_values = all;
  return iox_writecmd_all(IOX_CMD_PULLUP, all);
}

// we know last_{output,config,pullup}_values exist, since we configured them
// UNLESS there's no IOX
int iox_get_pin_pullup(int pin) {
  if (iox_spi_speed < 1000)
    return -1;
  return (last_pullup_values >> (pin & 0xf)) & 1;
}
int iox_set_pin_pullup(int pin, int en) {
  if (iox_spi_speed < 1000)
    return -1;
  int prev_val = last_pullup_values;
  const int mask = 1 << (pin & 0xf);
  if (en)
    iox_pullup_all(last_pullup_values | mask);
  else
    iox_pullup_all(last_pullup_values & ~mask);
  return prev_val & mask;
}

int iox_get_pin_direction(int pin) {
  if (iox_spi_speed < 1000)
    return -1;
  return (last_config_values & (1 << (pin & 0xf))) ? GPIO_IN : GPIO_OUT;
}
int iox_set_pin_direction(int pin, int dir) {
  if (iox_spi_speed < 1000)
    return -1;
  int prev_val = last_config_values;
  const int mask = 1 << (pin & 0xf);
  if (dir == GPIO_IN)
    iox_config_all(last_config_values | mask);
  else if (dir == GPIO_OUT)
    iox_config_all(last_config_values & ~mask);
  return (prev_val & mask) ? GPIO_IN : GPIO_OUT;
}

int iox_get_pin_output(int pin) {
  if (iox_spi_speed < 1000)
    return -1;
  return (last_output_values >> (pin & 0xf)) & 1;
}
int iox_set_pin_output(int pin, int high) {
  if (iox_spi_speed < 1000)
    return -1;
  int prev_val = last_output_values;
  const int mask = 1 << (pin & 0xf);
  if (high)
    iox_set_all(last_output_values | mask);
  else
    iox_set_all(last_output_values & ~mask);
  return (prev_val >> (pin & 0xf)) & 1;
}

int iox_get_pin_value(int pin) {
  if (iox_spi_speed < 1000)
    return -1;
  return (iox_get_all() >> (pin & 0xf)) & 1;
}

// check if IOX SPI works
// the way we do this is, we confirm we can write correctly
// to a register in the IOX (RDINV chosen)
int iox_check() {
  iox_writecmd_all(IOX_CMD_RDINV, 0x5A5A);
  unsigned rdinv = iox_readcmd_all(IOX_CMD_RDINV);
  if (rdinv != 0x5A5A) {
    iox_spi_speed = 0;
    return -1;
  }
  iox_writecmd_all(IOX_CMD_RDINV, 0xA5A5);
  rdinv = iox_readcmd_all(IOX_CMD_RDINV);
  if (rdinv != 0xA5A5) {
    iox_spi_speed = 0;
    return -1;
  }
  iox_writecmd_all(IOX_CMD_RDINV, 0);
  return 0;
}

// configure all A5 pins on the Pico itself
int a5_pico_pins_init() {
  // initialize UART pins
  gpio_set_function(PIN_A5_UART_RX, GPIO_FUNC_UART);
  gpio_set_function(PIN_A5_UART_TX, GPIO_FUNC_UART);
  bi_decl(bi_2pins_with_func(PIN_A5_UART_RX, PIN_A5_UART_TX, GPIO_FUNC_UART));
  // initialize SPI pins
  gpio_set_function(PIN_A5_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_A5_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_A5_MISO, GPIO_FUNC_SPI);
  bi_decl(bi_3pins_with_func(PIN_A5_SCK, PIN_A5_MOSI, PIN_A5_MISO, GPIO_FUNC_SPI));
  gpio_init(PIN_A5_SSn);
  gpio_put(PIN_A5_SSn, 1); // initially de-selected
  gpio_set_dir(PIN_A5_SSn, GPIO_OUT);
  bi_decl(bi_1pin_with_name(PIN_A5_SSn, "A5_SS#"));
  // scan pins (weak pulldown on SCANOUT)
  gpio_init(PIN_A5_SCANEN);
  gpio_put(PIN_A5_SCANEN, 0); // initially de-selected
  gpio_set_dir(PIN_A5_SCANEN, GPIO_OUT);
  bi_decl(bi_1pin_with_name(PIN_A5_SCANEN, "A5_SCANEN"));
  gpio_init(PIN_A5_SCANIN);
  gpio_put(PIN_A5_SCANIN, 0); // initially de-selected
  gpio_set_dir(PIN_A5_SCANIN, GPIO_OUT);
  bi_decl(bi_1pin_with_name(PIN_A5_SCANIN, "A5_SCANIN"));
  gpio_init(PIN_A5_SCANOUT);
  gpio_set_pulls(PIN_A5_SCANOUT, false, true);
  gpio_set_dir(PIN_A5_SCANOUT, GPIO_IN);
  bi_decl(bi_1pin_with_name(PIN_A5_SCANOUT, "A5_SCANOUT"));
  // GPIOs on the Pico - make these inputs until we confirm otherwise, and enable pull-downs
  gpio_init(PIN_A5_GPIO6);
  gpio_set_pulls(PIN_A5_SCANOUT, false, true);
  gpio_set_dir(PIN_A5_SCANIN, GPIO_IN);
  bi_decl(bi_1pin_with_name(PIN_A5_GPIO6, "A5_GPIO6"));
  gpio_init(PIN_A5_GPIO10);
  gpio_set_pulls(PIN_A5_SCANOUT, false, true);
  gpio_set_dir(PIN_A5_GPIO10, GPIO_IN);
  bi_decl(bi_1pin_with_name(PIN_A5_GPIO10, "A5_GPIO10"));
  return 0;
}

// configure all A5 pins on the IOX
int a5_iox_pins_init() {
  if (! iox_spi_speed)
    return -1;
  // set CLKSRC, TILESEL*, TESTEN to outputs
  // set the rest to inputs for now
  // have CLKSRC and TESTEN output 0, TILESEL* outputting 2'b01 by default
  last_output_values = (1 << (PIN_A5_TILESEL0&0xf));
  last_config_values = ~((1 << (PIN_A5_CLKSRC&0xf))   | (1 << (PIN_A5_TESTEN&0xf)) |
                         (1 << (PIN_A5_TILESEL1&0xf)) | (1 << (PIN_A5_TILESEL0&0xf)));
  last_pullup_values = last_config_values;
  // config the output values
  iox_writecmd_all(IOX_CMD_SET, last_output_values);
  // enable pull-ups on all the input pins
  iox_writecmd_all(IOX_CMD_PULLUP, last_pullup_values);
  // configure the pin directions
  iox_writecmd_all(IOX_CMD_CFG, last_config_values);
  return 0;
}

void iox_debug() {
  if (! iox_spi_speed) {
    puts("IOX not available!");
    return;
  }
  unsigned get = iox_readcmd_all(IOX_CMD_GET);
  unsigned set = iox_readcmd_all(IOX_CMD_SET);
  unsigned cfg = iox_readcmd_all(IOX_CMD_CFG);
  unsigned pullup = iox_readcmd_all(IOX_CMD_PULLUP);
  unsigned rdinv = iox_readcmd_all(IOX_CMD_RDINV);
  const unsigned tilesel_mask = (1<<(PIN_A5_TILESEL1&0xF)) | (1<<(PIN_A5_TILESEL0&0xF));
  printf("IOX: GSR=%04X OCR=%04X GCR=%04X PIR=%04X PUR=%04X\n"
         "     TILESEL=", get, set, cfg, rdinv, pullup);
  if (cfg & tilesel_mask)
    printf("floating");
  else
    printf("%u", (set & tilesel_mask) >> (PIN_A5_TILESEL0&0xF));
  printf (" CLKSRC=");
  if (cfg & (1<<(PIN_A5_CLKSRC&0x1f)))
    printf("floating\n");
  else
    printf("%u\n", (set & (1<<(PIN_A5_CLKSRC&0x1f))) ? 1 : 0);
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
    return iox_signames[pin - 0x40];
  else
    return 0;
}

//#define DEBUG

// size of every register, in bits (for ADCDATA, it can be 1/2/4; for CRC, it can be 2/4)
// masks will be applied before writing (and_mask of 0 means, read-only)
// we mark the reserved regs as r/o, even if they're technically r/w
// we also cache the last seen value (read or written)
static struct {
  unsigned size, and_mask, or_mask, last_val;
} adc_regs[16] = {
  [ADCR_ADCDATA]   = { 4, 0, 0 }, // R/O
  [ADCR_CONFIG0]   = { 1, 0xff, 0x00 },
  [ADCR_CONFIG1]   = { 1, 0xfc, 0x00 }, // cfg1[1:0] reserved, set to 2'b00
  [ADCR_CONFIG2]   = { 1, 0xfe, 0x01 }, // cfg2[0] reserved, set to 1'b1
  [ADCR_CONFIG3]   = { 1, 0xff, 0x00 },
  [ADCR_IRQ]       = { 1, 0x0f, 0x00 }, // irq: only bits [3:0] are r/w
  [ADCR_MUX]       = { 1, 0xff, 0x00 },
  [ADCR_SCAN]      = { 3, 0xe0ffff, 0 }, // scan[20] reserved, set to 1'b0; scan[19:16] unimplemented
  [ADCR_TIMER]     = { 3, 0xffffff, 0 },
  [ADCR_OFFSETCAL] = { 3, 0xffff00, 0 }, // offsetcal[7:0] unimplemented
  [ADCR_GAINCAL]   = { 3, 0xffff00, 0 }, // gaincal[7:0] unimplemented
  [0xB]            = { 3, 0, 0 }, // reserved
  [0xC]            = { 1, 0, 0 }, // reserved
  [ADCR_LOCK]      = { 1, 0xff, 0x00 },
  [0xE]            = { 2, 0, 0 }, // reserved
  [ADCR_CRCCFG]    = { 2, 0, 0 } // R/O
};
#define ADC_REGMAP_SIZE_NO_CRC_NO_DATA (0+1+1+1+1+1+1+3+3+3+3+3+1+1+2+0)
#define ADC_REGMAP_SIZE_MAX (4+1+1+1+1+1+1+3+3+3+3+3+1+1+2+4)
static unsigned adc_regmap_size = 0;

static uint8_t adc_cmds[ADC_REGMAP_SIZE_MAX], adc_data[ADC_REGMAP_SIZE_MAX];

int adc_probe()
{
  // the ADC can have one of 4 addresses, which'll get sent in reply to 
  // the 1st byte
  adc_addr = -1;
  if (adc_spi_speed < 1000)
    return -1;
  claim_spi_for_adc();
  // attempt all 4 possible addresses with a dummy command
  // to the command word, the response will be
  // {2'b0, cmd[7:6], ~cmd[6], data_ready#, crc_en#, por_int#}
  uint8_t expected_resp, cmp_mask = 0x38;
  for (unsigned addr = 0; addr < 4; ++addr) {
    adc_cmds[0] = (addr<<6)|ADC_DO_READ(0);
    expected_resp = addr << 4;
    if (! (addr & 1))
      expected_resp |= 1 << 3;
    adc_data[0] = 0xff;
    gpio_put(PIN_ADC_SSn, 0);
    spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 1);
    gpio_put(PIN_ADC_SSn, 1);
#   ifdef DEBUG
    printf ("ADC: addr %u: cmd=%02X resp=%02X exp_resp=%02X|mask=%02X\n", addr, cmds[0], data[0], expected_resp, cmp_mask);
#   endif
    if ((adc_data[0] & cmp_mask) == expected_resp) {
      adc_addr = addr;
      break;
    }
  }
  if (adc_addr == -1) {
    release_adc_spi();
    return -1;
  }
  // configure the ADC
  // - make sure register writes are unlocked
  adc_cmds[0] = (adc_addr << 6) | ADC_DO_WRITE_BURST(ADCR_LOCK);
  adc_cmds[1] = adc_regs[ADCR_LOCK].last_val = 
    ADC_LOCK_MAGIC;
  gpio_put(PIN_ADC_SSn, 0);
  spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 2);
  gpio_put(PIN_ADC_SSn, 1);
# ifdef DEBUG
  printf("ADC unlock: %02X.%02X -> %02X.%02X\n", cmds[0], cmds[1], data[0], data[1]);
# endif

  // setup the CONFIG0..GAINCAL regs, inclusive
  adc_cmds[0] = (adc_addr << 6) | ADC_DO_WRITE_BURST(ADCR_CONFIG0);
  // set mode to defaults + standby, don't output MCLK
  adc_cmds[1] = adc_regs[ADCR_CONFIG0].last_val =
    ADC_CFG0_VREFSEL_INTERNAL | ADC_CFG0_CLKSEL_INT_NOCLK | 
    ADC_CFG0_CSRC_NONE | ADC_CFG0_MODE_STANDBY; 
  adc_cmds[2] = adc_regs[ADCR_CONFIG1].last_val = 
    ADC_CFG1_PRE_NONE | ADC_CFG1_OSR_DEFAULT; // use defaults
  adc_cmds[3] = adc_regs[ADCR_CONFIG2].last_val = 0x01 | // CFG2[0] needs to be 1 at all times
    ADC_CFG2_BOOST_NONE | ADC_CFG2_GAIN_NONE; // use defaults
  // defaults + cause one-shots to return to standby, data format: max
  // (defaults include, crc disabled and crc set to 16bit)
  adc_cmds[4] = adc_regs[ADCR_CONFIG3].last_val = 
    ADC_CFG3_CMODE_ONESHOT_STANDBY | ADC_CFG3_DFMT_32BIT_SGNX_CHID;
  // defaults + disable conversion start interrupt output
  adc_cmds[5] = adc_regs[ADCR_IRQ].last_val =
    ADC_IRQ_FASTCMD_EN; // no ADC_IRQ_INACTIVE_HIGH_EN, since that's pulled up
  // default to measuring temperature - doesn't matter, since read_ch and read_ch_all set this
  adc_cmds[6] = adc_regs[ADCR_MUX].last_val =
    (ADC_MUXVAL_DIODE_P << ADC_MUX_VINP_SEL_POS) |
    (ADC_MUXVAL_DIODE_M << ADC_MUX_VINN_SEL_POS);
  // reset the SCAN, TIMER, OFFSETCAL and GAINCAL regs
  memset(adc_cmds+7, 0, 4*3);
  adc_regs[ADCR_SCAN].last_val = 0;
  adc_regs[ADCR_TIMER].last_val = 0;
  adc_regs[ADCR_OFFSETCAL].last_val = 0;
  adc_regs[ADCR_GAINCAL].last_val = 0;
  adc_regmap_size = ADC_REGMAP_SIZE_NO_CRC_NO_DATA + 4 + 2;
  // perform the initial programming
  gpio_put(PIN_ADC_SSn, 0);
  spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 7+12);
  gpio_put(PIN_ADC_SSn, 1);

# ifdef DEBUG
  printf("ADC config: %02X.%02X%02X%02X%02X:%02X:%02X:0000:0000:0000:0000 -> %02X....\n",
         cmds[0], cmds[1], cmds[2], cmds[3], cmds[4], cmds[5], cmds[6],
         data[0]);
# endif
  release_adc_spi();
  return adc_addr;
}

static struct adcregs_s adc_reg_vals;

int do_adc_get_regs(uint8_t *dest) {
  // do a readall of all registers, starting with register 1 (CONFIG0), finishing with ADCDATA
  // build a DO_READ_BURST starting with 1 (CONFIG0)
  memset (adc_cmds, 0xff, sizeof(adc_cmds));
  adc_cmds[0] = (adc_addr << 6) | ADC_DO_READ_BURST(ADCR_CONFIG0);
  // get the registers
  gpio_put(PIN_ADC_SSn, 0);
  spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, adc_regmap_size);
  gpio_put(PIN_ADC_SSn, 1);
  adc_busy = 0;
  uint8_t *pdata = &adc_data[0];
  adc_reg_vals.cmd       = adc_cmds[0];
  adc_reg_vals.cmd_resp  = *pdata++;
  adc_regs[ADCR_CONFIG0].last_val = *pdata++;
  adc_regs[ADCR_CONFIG1].last_val = *pdata++;
  adc_regs[ADCR_CONFIG2].last_val = *pdata++;
  adc_regs[ADCR_CONFIG3].last_val = *pdata++;
  adc_regs[ADCR_IRQ].last_val     = *pdata++;
  adc_regs[ADCR_MUX].last_val     = *pdata++;
  adc_regs[ADCR_SCAN].last_val      = ((uint32_t)pdata[0] << 16) | ((uint32_t)pdata[1] << 8) | pdata[2]; pdata += 3;
  adc_regs[ADCR_TIMER].last_val     = ((uint32_t)pdata[0] << 16) | ((uint32_t)pdata[1] << 8) | pdata[2]; pdata += 3;
  adc_regs[ADCR_OFFSETCAL].last_val = ((uint32_t)pdata[0] << 16) | ((uint32_t)pdata[1] << 8) | pdata[2]; pdata += 3;
  adc_regs[ADCR_GAINCAL].last_val   = ((uint32_t)pdata[0] << 16) | ((uint32_t)pdata[1] << 8) | pdata[2]; pdata += 3;
  adc_regs[0xB].last_val            = ((uint32_t)pdata[0] << 16) | ((uint32_t)pdata[1] << 8) | pdata[2]; pdata += 3;
  adc_regs[0xC].last_val            = *pdata++;
  adc_regs[ADCR_LOCK].last_val      = *pdata++;
  adc_regs[0xE].last_val            = ((uint32_t)pdata[0] << 8) | pdata[1]; pdata += 2;
  adc_regs[ADCR_CRCCFG].last_val    = ((uint32_t)pdata[0] << 8) | pdata[1]; pdata += 2;
  if (adc_regs[ADCR_CRCCFG].size == 4) {
    adc_regs[ADCR_CRCCFG].last_val  = (adc_regs[ADCR_CRCCFG].last_val << 16) |
                                      ((uint32_t)pdata[0] << 8) | pdata[1];
    pdata += 2;
  }
  adc_regs[ADCR_ADCDATA].last_val   = ((uint32_t)pdata[0] << 8) | pdata[1]; pdata += 2;
  if (adc_regs[ADCR_ADCDATA].size == 4) {
    adc_regs[ADCR_ADCDATA].last_val = (adc_regs[ADCR_ADCDATA].last_val << 16) |
                                      ((uint32_t)pdata[0] << 8) | pdata[1];
    pdata += 2;
  }
# ifdef DEBUG
  printf ("ADC regs: >%02X <%02X: {%02X %02X %02X %02X, %02X %02X, %06X %06X %06X %06X (%06X %02X) %02X (%02X) %04X %08X}\n",
          adc_reg_vals.cmd, adc_reg_vals.cmd_resp, 
          adc_reg_vals.config[0], adc_reg_vals.config[1], adc_reg_vals.config[2], adc_reg_vals.config[3], 
          adc_reg_vals.irq, adc_reg_vals.mux, 
          adc_reg_vals.scan, adc_reg_vals.timer, adc_reg_vals.offsetcal, adc_reg_vals.gaincal,
          adc_regs[0xB].last_val, adc_regs[0xC].last_val,
          adc_reg_vals.lock,
          adc_regs[0xE].last_val,
          adc_reg_vals.crccfg, adc_reg_vals.adcdata);
# endif
  // if a destination was provided
  if (dest) {
    // compile the response
    adc_reg_vals.adcdata   = adc_regs[ADCR_ADCDATA].last_val;
    adc_reg_vals.config[0] = adc_regs[ADCR_CONFIG0].last_val;
    adc_reg_vals.config[1] = adc_regs[ADCR_CONFIG1].last_val;
    adc_reg_vals.config[2] = adc_regs[ADCR_CONFIG2].last_val;
    adc_reg_vals.config[3] = adc_regs[ADCR_CONFIG3].last_val;
    adc_reg_vals.irq       = adc_regs[ADCR_IRQ].last_val;
    adc_reg_vals.mux       = adc_regs[ADCR_MUX].last_val;
    adc_reg_vals.scan      = adc_regs[ADCR_SCAN].last_val;
    adc_reg_vals.timer     = adc_regs[ADCR_TIMER].last_val;
    adc_reg_vals.offsetcal = adc_regs[ADCR_OFFSETCAL].last_val;
    adc_reg_vals.gaincal   = adc_regs[ADCR_GAINCAL].last_val;
    adc_reg_vals.lock      = adc_regs[ADCR_LOCK].last_val;
    adc_reg_vals.crccfg    = adc_regs[ADCR_CRCCFG].last_val;
    memcpy(dest, &adc_regs, sizeof(adc_regs));
  }
  return sizeof(adc_regs);
}

int do_adc_chreads(uint8_t *dest, unsigned bmp)
{
  int n_chan = __builtin_popcount(bmp);
  if (! n_chan)
    return 0;
  // setup SCANwith the requested bitmap
  if (adc_regs[ADCR_SCAN].last_val != bmp) {
    // reprogram SCAN
    adc_cmds[0] = (adc_addr << 6) | ADC_DO_WRITE_BURST(ADCR_SCAN);
    adc_cmds[1] = (bmp >> 16) & 0xFF;
    adc_cmds[2] = (bmp >> 8 ) & 0xFF;
    adc_cmds[3] = (bmp >> 0 ) & 0xFF;
    gpio_put(PIN_ADC_SSn, 0);
    spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 4);
    gpio_put(PIN_ADC_SSn, 1);
    printf ("set SCAN to %04X\n", bmp);
  }
  // issue a conversion
  adc_cmds[0] = (adc_addr << 6) | ADC_DO_CONV_START;
  gpio_put(PIN_ADC_SSn, 0);
  spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 1);
  gpio_put(PIN_ADC_SSn, 1);
  // fetch all the responses
  adc_cmds[0] = (adc_addr << 6) | ADC_DO_READ(ADCR_ADCDATA);
  for (int i = 0; i < n_chan; ++i) {
    unsigned n = 0;
    // poll until result is available
    do {
      gpio_put(PIN_ADC_SSn, 0);
      spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 1);
      gpio_put(PIN_ADC_SSn, 1);
      ++n;
    } while (adc_data[0] & (1<<2));
    // actually fetch the data
    gpio_put(PIN_ADC_SSn, 0);
    spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 1+adc_regs[ADCR_ADCDATA].size);
    gpio_put(PIN_ADC_SSn, 1);
    // record the result
    unsigned data;
    if (adc_regs[ADCR_ADCDATA].size == 2)
      data = ((unsigned)adc_data[1] << 8) | adc_data[2] | ((adc_data[2] & 0x80) ? 0xffff0000 : 0);
    else
      data = ((unsigned)adc_data[1] << 24) | ((unsigned)adc_data[2] << 16) |
             ((unsigned)adc_data[3] << 8 ) | ((unsigned)adc_data[4] << 0);
    //printf ("got data for ch%d after %u polls: %d\n", ((unsigned)data>>28), n, ((int)data << 8) >> 8);
    *dest++ = (data >> 0 ) & 0xff;
    *dest++ = (data >> 8 ) & 0xff;
    *dest++ = (data >> 16) & 0xff;
    *dest++ = (data >> 24) & 0xff;
  }
  return 4*n_chan;
}

// set a single register
int do_adc_set_reg(int which, uint32_t value)
{
  which &= 0xf;
  // don't support writing to reserved/ro registers
  if (! adc_regs[which].and_mask)
    return 0;
  // fixup the value
  value = (value & adc_regs[which].and_mask) | adc_regs[which].or_mask;
  adc_regs[which].last_val = value;
  // perform the write and update the cached value
  uint8_t *p = &adc_cmds[0];
  *p++ = (adc_addr << 6) | ADC_DO_WRITE_BURST(which);
  // write the bytes is MSB order
  //if (adc_regs[which].size >= 4) // no writable 32-bit registers
  //  *p++ = (value >> 24) & 0xFF;
  if (adc_regs[which].size >= 3)
    *p++ = (value >> 16) & 0xFF;
  if (adc_regs[which].size >= 2)
    *p++ = (value >> 8) & 0xFF;
  *p++ = value & 0xff;
  gpio_put(PIN_ADC_SSn, 0);
  spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 1)+adc_regs[which].size;
  gpio_put(PIN_ADC_SSn, 1);
  // writes to CONFIG3 or IRQ could potentially update the size of ADCDATA
  if ((which == ADCR_CONFIG3) || (which == ADCR_IRQ)) {
    if (adc_regs[ADCR_IRQ].last_val & ADC_IRQ_PIN_MDAT_EN)
      adc_regs[ADCR_ADCDATA].size = 1;
    else if ((adc_regs[ADCR_CONFIG3].last_val & ADC_CFG3_DFMT_MASK) == ADC_CFG3_DFMT_16BIT)
      adc_regs[ADCR_ADCDATA].size = 2;
    else
      adc_regs[ADCR_ADCDATA].size = 4;
  }
  // writes to CONFIG3 could potentially update the size of CRCCFG
  if (which == ADCR_CONFIG3) {
    if (value & ADC_CFG3_CRCFMT_32BIT)
      adc_regs[ADCR_CRCCFG].size = 4;
    else
      adc_regs[ADCR_CRCCFG].size = 2;
  }
}
