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

    case CMD_ADC_GETREGS:
      cmd_printf (" %c# @%u ADC_GETREGS\n", buf, cmdpos);
      m = do_adc_getregs(respbuf+resppos);
      if (! m)
        puts("No ADC preset for ADC_GETREGS");
      else
       resppos += m;
      ++cmdpos;
      break;
    case CMD_ADC_CHREAD:
      cmd_printf (" %c# @%u ADC_CHREAD %u\n", buf, cmdpos, cmdbuf[cmdpos+1] & 3);
      // need to do this here - CHREAD_ALL calls do_adc_chread in a loop
      while (eth_busy) // wait for the other core to finish whatever it's doing on ETH
        ;
      adc_busy = 1;
      // if the W5500 was selected, de-select it!
      if (spi_get_baudrate(SPI_ADC) != adc_spi_speed)
        spi_set_baudrate(SPI_ADC, adc_spi_speed);
      m = do_adc_chread(respbuf+resppos, cmdbuf[cmdpos+1] & 3);
      // release the SPI
      adc_busy = 0;
      if (! m)
        puts("No ADC preset for ADC_CHREAD");
      else
       resppos += m;
      cmdpos += 2;
      break;
    case CMD_ADC_CHREAD_ALL:
      cmd_printf (" %c# @%u ADC_CHREAD_ALL\n", buf, cmdpos);
      // need to do this here - CHREAD_ALL calls do_adc_chread in a loop
      while (eth_busy) // wait for the other core to finish whatever it's doing on ETH
        ;
      adc_busy = 1;
      // if the W5500 was selected, de-select it!
      if (spi_get_baudrate(SPI_ADC) != adc_spi_speed)
        spi_set_baudrate(SPI_ADC, adc_spi_speed);
      m = do_adc_chread(respbuf+resppos, 0);
      if (m == 4) {
        m += do_adc_chread(respbuf+resppos+m, 1);
        m += do_adc_chread(respbuf+resppos+m, 2);
        m += do_adc_chread(respbuf+resppos+m, 3);
      }
      // release the SPI
      adc_busy = 0;
      if (! m)
        puts("No ADC preset for ADC_CHREAD");
      else
       resppos += m;
      ++cmdpos;
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

// force the most recent channel to be set on the 1st read
int last_adc_ch;

int adc_probe()
{
  // the ADC can have one of 4 addresses, which'll get sent in reply to 
  // the 1st byte
  adc_addr = -1;
  last_adc_ch = -1;
  if (adc_spi_speed < 1000)
    return -1;
  while (eth_busy) // wait for the other core to finish whatever it's doing on ETH
    ;
  adc_busy = 1;
  if (spi_get_baudrate(SPI_ADC) != adc_spi_speed)
    spi_set_baudrate(SPI_ADC, adc_spi_speed);
  // attempt all 4 possible addresses with a dummy command
  uint8_t cmds[8], data[8];
  // to the command word, the response will be
  // {2'b0, cmd[7:6], ~cmd[6], data_ready#, crc_en#, por_int#}
  uint8_t expected_resp, cmp_mask = 0x38;
  for (unsigned addr = 0; addr < 4; ++addr) {
    cmds[0] = (addr<<6)|ADC_DO_READ(0);
    expected_resp = addr << 4;
    if (! (addr & 1))
      expected_resp |= 1 << 3;
    data[0] = 0xff;
    gpio_put(PIN_ADC_SSn, 0); spi_write_read_blocking(SPI_ADC, cmds, data, 1); gpio_put(PIN_ADC_SSn, 1);
#   ifdef DEBUG
    printf ("ADC: addr %u: cmd=%02X resp=%02X exp_resp=%02X|mask=%02X\n", addr, cmds[0], data[0], expected_resp, cmp_mask);
#   endif
    if ((data[0] & cmp_mask) == expected_resp) {
      adc_addr = addr;
      break;
    }
  }
  if (adc_addr == -1) {
    adc_busy = 0;
    return -1;
  }

  // configure the ADC
  // - make sure register writes are unlocked
  cmds[0] = (adc_addr << 6) | ADC_DO_WRITE_BURST(ADCR_LOCK);
  cmds[1] = ADC_LOCK_MAGIC;
  gpio_put(PIN_ADC_SSn, 0); spi_write_read_blocking(SPI_ADC, cmds, data, 2); gpio_put(PIN_ADC_SSn, 1);
# ifdef DEBUG
  printf("ADC unlock: %02X.%02X -> %02X.%02X\n", cmds[0], cmds[1], data[0], data[1]);
# endif
  // - setup the 4 CONFIG registers
  cmds[0] = (adc_addr << 6) | ADC_DO_WRITE_BURST(ADCR_CONFIG0);
  // set mode to defaults + standby, don't output MCLK
  cmds[1] = ADC_CFG0_VREFSEL_INTERNAL | ADC_CFG0_CLKSEL_INT_NOCLK | ADC_CFG0_CSRC_NONE |
            ADC_CFG0_MODE_SHUTDOWN; 
  cmds[2] = ADC_CFG1_PRE_NONE | ADC_CFG1_OSR_DEFAULT; // use defaults
  cmds[3] = ADC_CFG2_BOOST_NONE | ADC_CFG2_GAIN_NONE | ADC_CFG2_AZ_MUX_EN | ADC_CFG2_AZ_REF_EN; // use defaults + auto-zero algorithms
  // defaults + cause one-shots to return to standby, data format: max
  // (defaults include, crc disabled and crc set to 16bit)
  cmds[4] = ADC_CFG3_CMODE_ONESHOT_SHUTDOWN | ADC_CFG3_DFMT_32BIT_SGNX_CHID;
  // defaults + disable conversion start interrupt output
  cmds[5] = ADC_IRQ_INACTIVE_HIGH_EN | ADC_IRQ_FASTCMD_EN;
  // default to measuring temperature - doesn't matter, since read_ch and read_ch_all set this
  cmds[6] = (ADC_MUXVAL_DIODE_P << ADC_MUX_VINP_SEL_POS) |
            (ADC_MUXVAL_DIODE_M << ADC_MUX_VINN_SEL_POS);
  // we're not using scanning mode, so leave the following regs alone

  // perform the initial programming
  gpio_put(PIN_ADC_SSn, 0); spi_write_read_blocking(SPI_ADC, cmds, data, 7); gpio_put(PIN_ADC_SSn, 1);
# ifdef DEBUG
  printf("ADC config: %02X.%02X%02X%02X%02X:%02X:%02X -> %02X.%02X%02X%02X%02X:%02X:%02X\n",
         cmds[0], cmds[1], cmds[2], cmds[3], cmds[4], cmds[5], cmds[6],
         data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
  // read back the registers
  memset(cmds, 0xff, sizeof(cmds));
  cmds[0] = (adc_addr << 6) | ADC_DO_READ_BURST(ADCR_CONFIG0);
  gpio_put(PIN_ADC_SSn, 0); spi_write_read_blocking(SPI_ADC, cmds, data, 7); gpio_put(PIN_ADC_SSn, 1);
  printf(" read back: %02X.%02X%02X%02X%02X:%02X:%02X -> %02X.%02X%02X%02X%02X:%02X:%02X\n",
         cmds[0], cmds[1], cmds[2], cmds[3], cmds[4], cmds[5], cmds[6],
         data[0], data[1], data[2], data[3], data[4], data[5], data[6]);
# endif
  adc_busy = 0;
  return adc_addr;
}

// size of every register, in bits (for ADCDATA, it can be 1/2/4; for CRC, it can be 2/4)
static uint8_t adc_rsize[16] = {
  4, // ADCDATA; size is (CONFIG[3]&(3<<4))?4:2
  1, 1, 1, 1, // CONFIG[0:3]
  1, 1, // IRQ, MUX
  3, 3, 3, 3, // SCAN, TIMER, OFFSETCAL, GAINCAL
  3, // RESERVED
  1, // RESERVED
  1, // LOCK
  2, // RESERVED
  2  // CRCCFG; size is 2*(1+((CONFIG[3]>>2)&1)
};

static struct adcregs_s adc_regs;

int do_adc_getregs(uint8_t *dest) {
  if (adc_spi_speed < 1000)
    return 0;
  while (eth_busy) // wait for the other core to finish whatever it's doing on ETH
    ;
  adc_busy = 1;
  // if the W5500 was selected, de-select it!
  if (spi_get_baudrate(SPI_ADC) != adc_spi_speed)
    spi_set_baudrate(SPI_ADC, adc_spi_speed);
  // do a readall of all registers, starting with register 1 (CONFIG0), finishing with ADCDATA
  // we know we have 16-bit CRC and 32-bit DATA (initialization does that)
  static uint8_t cmds[1+(1+1+1+1+1+1+3+3+3+3+3+1+1+2+2+4)];
  static uint8_t data[1+(1+1+1+1+1+1+3+3+3+3+3+1+1+2+2+4)];
  // build a DO_READ_BURST starting with 1 (CONFIG0)
  memset (cmds, 0xff, sizeof(cmds));
  cmds[0] = (adc_addr << 6) | ADC_DO_READ_BURST(ADCR_CONFIG0);
  // get the registers
  gpio_put(PIN_ADC_SSn, 0); spi_write_read_blocking(SPI_ADC, cmds, data, sizeof(cmds)); gpio_put(PIN_ADC_SSn, 1);
  adc_busy = 0;
  adc_regs.cmd       = cmds[0];
  adc_regs.cmd_resp  = data[0];
  adc_regs.config[0] = data[1];
  adc_regs.config[1] = data[2];
  adc_regs.config[2] = data[3];
  adc_regs.config[3] = data[4];
  adc_regs.irq       = data[5];
  adc_regs.mux       = data[6];
  adc_regs.scan      = ((uint32_t)data[7] << 16) | ((uint32_t)data[7+1] << 8) | data[7+2];
  adc_regs.timer     = ((uint32_t)data[10] << 16) | ((uint32_t)data[10+1] << 8) | data[10+2];
  adc_regs.offsetcal = ((uint32_t)data[13] << 16) | ((uint32_t)data[13+1] << 8) | data[13+2];
  adc_regs.gaincal   = ((uint32_t)data[16] << 16) | ((uint32_t)data[16+1] << 8) | data[16+2];
  uint32_t rsvd_0xB  = ((uint32_t)data[19] << 16) | ((uint32_t)data[19+1] << 8) | data[19+2];
  uint8_t rsvd_0xC   =  data[22];
  adc_regs.lock      =  data[23];
  uint8_t rsvd_0xE   =  data[24];
  adc_regs.crccfg    = ((uint32_t)data[25] << 8) | data[25+1];
  adc_regs.adcdata   = ((uint32_t)data[27+0] << 24) | ((uint32_t)data[27+1] << 16) |
                       ((uint32_t)data[27+2] << 8)  |            data[27+3];
# ifdef DEBUG
  printf ("ADC regs: >%02X <%02X: {%02X %02X %02X %02X, %02X %02X, %06X %06X %06X %06X (%06X %02X) %02X (%02X) %04X %08X}\n",
          cmds[0], data[0], 
          data[1], data[2], data[3], data[4],
          data[5], data[6],
          adc_regs.scan, adc_regs.timer, adc_regs.offsetcal, adc_regs.gaincal,
          rsvd_0xB, data[22],
          data[23],
          data[24],
          adc_regs.crccfg, adc_regs.adcdata);
# endif
  // if a destination was provided
  if (dest)
    memcpy(dest, &adc_regs, sizeof(adc_regs));
  return sizeof(adc_regs);
}


int do_adc_chread(uint8_t *dest, unsigned ch)
{
  if (adc_spi_speed < 1000)
    return 0;
  // handle eth conflicts in the CALLER, since this gets called in a loop
  // in chread_all!!!

  // change the mux if needed
  ch &= 3;
  uint8_t cmd[8], data[8];
  if (last_adc_ch != ch) {
    cmd[0] = (adc_addr << 6) | ADC_DO_WRITE_BURST(ADCR_MUX);
    cmd[1] = (ADC_MUXVAL_CH(ch) << ADC_MUX_VINP_SEL_POS) |
             (ADC_MUXVAL_AGND << ADC_MUX_VINN_SEL_POS);
    gpio_put(PIN_ADC_SSn, 0); spi_write_read_blocking(SPI_ADC, cmd, data, 2); gpio_put(PIN_ADC_SSn, 1);
    printf ("set ADC channel to %u\n", ch);
    last_adc_ch = ch;
  }
  // issue a fast conversion
  cmd[0] = (adc_addr << 6) | ADC_DO_CONV_START;
  gpio_put(PIN_ADC_SSn, 0); spi_write_read_blocking(SPI_ADC, cmd, data, 1); gpio_put(PIN_ADC_SSn, 1);
  // poll until result is available
  cmd[0] = (adc_addr << 6) | ADC_DO_READ(ADCR_ADCDATA);
  cmd[1] = cmd[2] = cmd[3] = cmd[4] = 0xff;
  int n = 0;
  do {
    gpio_put(PIN_ADC_SSn, 0); spi_write_read_blocking(SPI_ADC, cmd, data, 1); gpio_put(PIN_ADC_SSn, 1);
    ++n;
    // data should be{2'b0, cmd[7:6], ~cmd[6], data_ready#, crc_en#, por_int#}
  } while (data[0] & (1<<2));
  printf ("got data after %u polls\n", n);
  // acually do the read
  gpio_put(PIN_ADC_SSn, 0); spi_write_read_blocking(SPI_ADC, cmd, data, 5); gpio_put(PIN_ADC_SSn, 1);
  dest[0] = data[4];
  dest[1] = data[3];
  dest[2] = data[2];
  dest[3] = data[1];
  return 4;
}
