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
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/bootrom.h>
#include <hardware/clocks.h>
#include <hardware/watchdog.h>
#include <tusb.h>

#include "a5pins.h"
#include "config.h"
#include "pio_jtag.h"
#include "jtag.pio.h"
#include "adc.h"
#include "utils.h"
#include "cmd.h"

//#define cmd_printf(...) printf(__VA_ARGS__)
#define cmd_printf(...) (void)(__VA_ARGS__)

unsigned cmd_execute(pio_jtag_inst_t* jtag, char buf, const uint8_t *cmdbuf, unsigned cmdsz, uint8_t *respbuf)
{
  unsigned cmdpos = 0, resppos = 0;
  int pin, cfg; // pin config
  struct pindesc_s pindesc; // pin description
  extern struct djtag_clk_s djtag_clocks;
  int n, m;
  int do_iox_debug = 0;
  //cmd_printf("buf=%c(%p) sz=%u\n", buf, cmdbuf, cmdsz);
  while (cmdpos < cmdsz) {
    uint8_t cmd = cmdbuf[cmdpos];
    if (cmd == CMD_STOP) {
      cmd_printf(" %c# @%u CMD_STOP\n", buf, cmdpos);
      break;
    }
    switch (cmd) {

    case CMD_GOTOBOOTLOADER:
      cmd_printf(" %c# @%u CMD_BOOTLOADER\n", buf, cmdpos);
      puts("Rebooting to bootloader...");
      sleep_ms(200);
      reset_usb_boot(0, 0);
      while (1)
        asm volatile ("wfe");
      break;

    case CMD_REBOOT:
      cmd_printf(" %c# @%u CMD_REBOOT\n", buf, cmdpos);
      puts("Rebooting...");
      watchdog_reboot(0, 0, 200); // standard boot in 0.2s
      while (1)
        asm volatile ("wfe");
      break;

#   if 0
    // keep this in, just a reminder that the code point was actually
    // reused as REBOOT
    case CMD_SETVOLTAGE:
      // this one is a dummy
      cmd_printf(" %c# @%u CMD_SETVOLTAGE (no-op)\n", buf, cmdpos);
      ++cmdpos;
      break;
#   endif

    case CMD_INFO:
      n = strlen(djtag_whoami());
      cmd_printf(" %c# @%u CMD_INFO >%u\n", buf, cmdpos, n);
      memcpy(respbuf+resppos, djtag_whoami(), n);
      resppos += n;
      ++cmdpos;
      break;

    case CMD_GETCLKS:
      cmd_printf(" %c# @%u CMD_GETCLKS >%u\n", buf, cmdpos, sizeof(djtag_clocks));
      memcpy(respbuf+resppos, &djtag_clocks, sizeof(djtag_clocks));
      resppos += sizeof(djtag_clocks);
      ++cmdpos;
      break;

    case CMD_FREQ:
      cmd_printf(" %c# @%u CMD_FREQ\n", buf, cmdpos);
      jtag_set_clk_freq(jtag, ((unsigned)cmdbuf[cmdpos+1] << 8) | cmdbuf[cmdpos+2]);
      cmdpos += 3;
      break;

    case CMD_A5FREQ:
      cmd_printf(" %c# @%u CMD_A5FREQ\n", buf, cmdpos);
      extern pio_a5clk_inst_t a5clk;
      a5clk_set_freq(&a5clk, ((unsigned)cmdbuf[cmdpos+1] << 8) | cmdbuf[cmdpos+2]);
      cmdpos += 3;
      break;

    case CMD_PINCFG_SET_ALL:
      cfg = cmdbuf[cmdpos+1];
      cmd_printf(" %c# @%u CMD_PINCFG_SET_ALL 0x%02X: %s\n", buf, cmdpos,
                  cfg, describe_pincfg(cfg, 0, 0));
      pincfg_set_all(cfg); // we ignore direction and value (sets slew rate/hysteresis/pulls)
      break;

    case CMD_PINCFG_SET:
      pin = cmdbuf[cmdpos+1];
      cfg = cmdbuf[cmdpos+2];
      describe_pin(pin, &pindesc);
      cmd_printf(" %c# @%u CMD_PINCFG_SET %u(%s.%s) 0x%02X: %s\n", buf, cmdpos,
                  pin, pindesc.location, pindesc.signal, cfg, 
                  describe_pincfg(cfg, 1, pindesc.on_iox));
      if (pindesc.valid) {
        pincfg_set(pin, cfg, pindesc.on_iox);
        do_iox_debug = 1;
      }
      cmdpos += 3;
      break;

    case CMD_PINCFG_GET:
      pin = cmdbuf[cmdpos+1];
      describe_pin(pin, &pindesc);
      cmd_printf(" %c# @%u CMD_PINCFG_GET %u(%s.%s) >2\n", buf, cmdpos,
                  pin, pindesc.location, pindesc.signal);
      unsigned result = 0;
      if (pindesc.valid)
        result = pincfg_get(pin, pindesc.on_iox);
      respbuf[resppos++] = result & 0xff;
      respbuf[resppos++] = result >> 8;
      cmd_printf("       > %04X\n", result);
      cmdpos += 2;
      break;

    case CMD_GETSIG:
      cmd_printf(" %c# @%u CMD_PINCFG_GET >1\n", buf, cmdpos);
      n = 0;
      if (jtag_get_tdo(jtag))
        n |= SIG_TDO;
      respbuf[resppos++] = n;
      ++cmdpos;
      break;

    case CMD_SETSIG:
      cmd_printf(" %c# @%u CMD_SETSIG\n", buf, cmdpos);
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
      cmd_printf(" %c# @%u CMD_A5CLK %s\n", buf, cmdpos, (cmd == CMD_A5CLK) ? "OFF" : "ON");
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
      cmd_printf(" %c# @%u CMD_CLK%s %u%s\n", buf, cmdpos, (cmd & READOUT)?",READOUT":"", m, (cmd & READOUT)?" >1":"");
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
        cmd_printf(" %c# @%u CMD_XFER%s,NOREAD %u\n", buf, cmdpos,
                    (n >= 256)?",LARGE":"", n);
      else
        cmd_printf(" %c# @%u CMD_XFER%s %u >%u\n", buf, cmdpos,
                    (n >= 256)?",LARGE":"", n, m);
      // we can probably get away with using the response buffer even if the
      // read isn't requested
      jtag_transfer(jtag, n, cmdbuf+cmdpos+2, respbuf+resppos);
      if (! (cmd & NO_READ))
        resppos += m;
      cmdpos += m + 2;
      break;

    case CMD_ADC_GETREGS:
      cmd_printf(" %c# @%u ADC_GETREGS\n", buf, cmdpos);
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
      cfg = cmdbuf[cmdpos+5];
      cfg = (cfg << 8) | cmdbuf[cmdpos+4];
      cfg = (cfg << 8) | cmdbuf[cmdpos+3];
      cfg = (cfg << 8) | cmdbuf[cmdpos+2];
      cmd_printf(" %c# @%u ADC_SETREG %u 0x%X (%02X %02X %02X %02X)\n", buf, cmdpos, cmdbuf[cmdpos+1] & 0xF, cfg,
        cmdbuf[cmdpos+2], cmdbuf[cmdpos+3], cmdbuf[cmdpos+4], cmdbuf[cmdpos+5]);
      if (adc_spi_speed) {
        claim_spi_for_adc();
        do_adc_set_reg(cmdbuf[cmdpos+1] & 0xF, cfg);
        release_adc_spi();
      }
      cmdpos += 6; // cmd + reg + 32bit value
      break;

    case CMD_ADC_CHREAD_MUX:
      cmd_printf(" %c# @%u ADC_CHREAD_MUX 0x%02X\n", buf, cmdpos, cmdbuf[cmdpos+1]);
      if (adc_spi_speed) {
        claim_spi_for_adc();
        resppos += do_adc_chread_mux(respbuf+resppos, cmdbuf[cmdpos+1]);
        release_adc_spi();
      } else {
        // send a dummy response, so we won't break the protocol
        memset(respbuf+resppos, 0xff, sizeof(uint32_t));
        resppos += sizeof(uint32_t);
      }
      cmdpos += 2;
      break;

    case CMD_ADC_CHREAD_SCAN:
      cfg = ((unsigned)cmdbuf[cmdpos+2] << 8) | cmdbuf[cmdpos+1];
      cmd_printf(" %c# @%u ADC_CHREAD_SCAN 0x%04X\n", buf, cmdpos, cfg);
      if (adc_spi_speed) {
        claim_spi_for_adc();
        resppos += do_adc_chread_scan(respbuf+resppos, cfg);
        release_adc_spi();
      } else {
        // send a dummy response, so we won't break the protocol
        memset(respbuf+resppos, 0xff, __builtin_popcount(cfg) * sizeof(uint32_t));
        resppos += __builtin_popcount(cfg) * sizeof(uint32_t);
      }
      cmdpos += 3;
      break;

    case CMD_ADC_SAMPLE:
      cmd_printf(" %c# @%u ADC_SAMPLE\n", buf, cmdpos);
      if (adc_spi_speed) {
        claim_spi_for_adc();
        resppos += do_adc_conv(respbuf+resppos);
        release_adc_spi();
      } else {
        // send a dummy response, so we won't break the protocol
        memset(respbuf+resppos, 0xff, sizeof(uint32_t));
        resppos += sizeof(uint32_t);
      }
      ++cmdpos;
      break;

    default:
      // invalid command: reboot
      // also: print the entire command byte, not just cmd[6:0]
      printf(" %c# @%u ??? 0x%02X\n", buf, cmdpos, cmdbuf[cmdpos]);
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
    cmd_printf(" %c# resp_sz=0x%X; padding\n", buf, resppos);
    respbuf[resppos++] = 0xA5;
  }
  if (resppos) {
    cmd_printf(" %c# [", buf);
    for (int i = 0; i < resppos; ++i ) {
      cmd_printf(" %02X", respbuf[i]);
    }
    cmd_printf(" ]\n");
  }
  cmd_printf(" %c# done, cmd_sz=%u(%u) resp_sz=0x%u\n", buf, cmdpos, cmdsz, resppos);
  return resppos;
}
