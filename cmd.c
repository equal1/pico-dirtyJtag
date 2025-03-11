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

#define GET_HWORD_AT(ptr) \
  (((uint32_t)((ptr)[0])) | \
   ((uint32_t)((ptr)[1]) << 8))
#define GET_WORD_AT(ptr) \
  (((uint32_t)((ptr)[0])) | \
   ((uint32_t)((ptr)[1]) << 8) | \
   ((uint32_t)((ptr)[2]) << 16) | \
   ((uint32_t)((ptr)[3]) << 24))

#define SET_HWORD_AT(ptr,data) \
   ((ptr)[0] = data & 0xFF, \
    (ptr)[1] = data >> 8)
#define SET_WORD_AT(ptr,data) \
   ((ptr)[0] = data & 0xFF, \
    (ptr)[1] = data >> 8, \
    (ptr)[2] = data >> 16, \
    (ptr)[3] = data >> 24)

unsigned cmd_execute(pio_jtag_inst_t* jtag, char buf, const uint8_t *cmdbuf, unsigned cmdsz, uint8_t *respbuf)
{
  unsigned cmdpos = 0, resppos = 0;
  int pin, cfg; // pin config
  struct pindesc_s pindesc; // pin description
  extern struct djtag_clk_s djtag_clocks;
  int n, m;
  //cmd_printf("buf=%c(%p) sz=%u\n", buf, cmdbuf, cmdsz);
  struct {
    uint32_t pin, cfg, pull;
  } last, crt;
  int have_pincfg_set = 0;
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
        if (pindesc.on_iox && ! have_pincfg_set) {
          have_pincfg_set = 1;
          last.pin = iox_get_all();
          last.cfg = iox_getcfg_all();
          last.pull = iox_getpull_all();
        }
        pincfg_set(pin, cfg, pindesc.on_iox);
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
      SET_HWORD_AT(respbuf+resppos, result);
      resppos += 2;
      cmd_printf("\t> %04X\n", result);
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
      //if (n & SIG_TRST)
      //  jtag_set_trst(jtag, m & SIG_TRST);
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
      if (cmd & NO_READ)
        m = 0;
      cmd_printf(" %c# @%u CMD_XFER %u >%u\n", buf, cmdpos, n, m);
      // we can probably get away with using the response buffer even if the
      // read isn't requested
      jtag_transfer(jtag, n, cmdbuf+cmdpos+2, respbuf+resppos);
      resppos += m;
      cmdpos += ((n + 7) / 8) + 2;
      break;

    // count devices in the chain using BYPASS mode
    // returns a single uint8_t
    case XCMD_BYPASS_COUNT:
      cmd_printf(" %c# @%u XCMD_BYPASS\n", buf, cmdpos);
      n = jtag_do_bypass();
      cmd_printf("\t> %u\n", n);
      respbuf[resppos++] = n;
      cmdpos++;
      break;

    // get the device IDCODEs from the chain
    // returns a single zero if no devices, or 4*n_idcodes
    case XCMD_GET_IDCODES:
      cmd_printf(" %c# @%u XCMD_GET_IDCODES\n", buf, cmdpos);
      static uint32_t idcodes[16];
      n = jtag_get_idcodes(idcodes);
      if (!n) {
        cmd_printf("\t> %d", 0);
        respbuf[resppos++] = 0;
      } else {
        cmd_printf("\t> %d: %08X%s\n", n, idcodes[0], (n > 1)?"...":"");
        memcpy(respbuf + resppos, idcodes, 4*n);
        resppos += 4*n;
      }
      cmdpos++;
      break;

    // configure jtagx mode
    case XCMD_CONFIG:
      cmd_printf(" %c# @%u XCMD_CONFIG\n", buf, cmdpos);
      cmdpos += 1 + jtag_set_config(cmdbuf + cmdpos + 1);
      break;

    // perform scans
    case XCMD_IRSCAN:
    case XCMD_IRSCAN|EXTEND_LENGTH:
    case XCMD_IRSCAN|NO_READ:
    case XCMD_IRSCAN|EXTEND_LENGTH|NO_READ:
    case XCMD_DRSCAN:
    case XCMD_DRSCAN|EXTEND_LENGTH:
    case XCMD_DRSCAN|NO_READ:
    case XCMD_DRSCAN|EXTEND_LENGTH|NO_READ:
      n = cmdbuf[cmdpos+1]; // number of bits to transfer
      if (cmd & EXTEND_LENGTH)
        n += 256;
      m = (n + 7) / 8;
      if (cmd & NO_READ)
        m = 0;
      int ir = (cmd & ~(EXTEND_LENGTH|NO_READ)) == XCMD_IRSCAN;
      cmd_printf(" %c# @%u CMD_%cRSCAN %u >%u\n", buf, cmdpos, ir?'I':'D', n, m);
      // we can probably get away with using the response buffer even if the
      // read isn't requested
      resppos += jtag_do_scan(ir?1:0, n, cmdbuf+cmdpos+2, m ? respbuf+resppos : NULL);
      cmdpos += ((n + 7) / 8) + 2;
      break;

    // perform an ABORT
    case XCMD_ABORT:
      // grab the command byte
      n = cmdbuf[cmdpos+1];
      cmd_printf(" %c# @%u ABORT 0x%02X\n", buf, cmdpos, n);
      // do the abort
      n = jtag_abort(n);
      // no response to ABORT
      cmdpos += 2;
      break;

    // perform a DPACC read
    case XCMD_DPACC_RD:
      // grab the address, 6-bits, rshifted by 2: (0..0xfc)>>2
      n = (cmdbuf[cmdpos+1] & 0x3f) << 2;
      cmd_printf(" %c# @%u DPACC_RD @0x%02X\n", buf, cmdpos, n);
      // do the read
      n = jtag_dpacc_rd(n, (uint32_t*)&m);
      // prepare the response
      respbuf[resppos] = n;
      SET_WORD_AT(respbuf+resppos+1, m);
      cmdpos += 2; resppos += 5;
      cmd_printf("\t> (%X) %08X\n", n, m);
      break;

    // perform a DPACC write
    case XCMD_DPACC_WR:
      // grab the address, 6-bits, rshifted by 2: (0..0xfc)>>2
      n = (cmdbuf[cmdpos+1] & 0x3f) << 2;
      // grab the data
      m = GET_WORD_AT(cmdbuf+cmdpos+2);
      cmd_printf(" %c# @%u DPACC_WR @0x%02X 0x%X\n", buf, cmdpos, n, m);
      // do the write
      n = jtag_dpacc_wr(n, m);
      // prepare the response
      respbuf[resppos] = n;
      cmdpos += 6; resppos += 1;
      cmd_printf("\t> (%X)\n", n);
      break;

    // perform an APACC read
    case XCMD_APACC_RD:
      // grab the AP
      n = cmdbuf[cmdpos+1];
      // grab the address, 11-bits, rshifted by 2: (0..0x1ffc)>>2
      m = (GET_HWORD_AT(cmdbuf+cmdpos+2) & 0x7ff) << 2;
      cmd_printf(" %c# @%u APACC_RD%s @%u:0x%04X\n", buf, cmdpos,
                 (n >> 7)?"\'":"", n & 0x7f, m );
      // do the read
      n = jtag_apacc_rd(n, m, (uint32_t*)&m);
      // prepare the response
      respbuf[resppos] = n;
      SET_WORD_AT(respbuf+resppos+1, m);
      cmdpos += 4; resppos += 5;
      cmd_printf("\t> (%X) %08X\n", n, m);
      break;

    // perform an APACC write
    case XCMD_APACC_WR:
      // grab the AP
      n = cmdbuf[cmdpos+1];
      // grab the address, 11-bits, rshifted by 2: (0..0x1ffc)>>2
      m = (GET_HWORD_AT(cmdbuf+cmdpos+2) & 0x7ff) << 2;
      // grab the data
      uint32_t p = GET_WORD_AT(cmdbuf+cmdpos+4);
      cmd_printf(" %c# @%u APACC_WR%s @%u:0x%04X 0x%08X\n", buf, cmdpos,
                 (n >> 7)?"\'":"", n & 0x7f, m, p );
      // do the write
      n = jtag_apacc_wr(n, m, p);
      // prepare the response
      respbuf[resppos] = n;
      cmd_printf("\t> (%X)\n", n);
      cmdpos += 8; resppos += 1;
      break;

    // perform bus reads
    case XCMD_BUS_RD:
      // grab the address
      n = GET_WORD_AT(cmdbuf+cmdpos+1) & ~3;
      cmd_printf(" %c# @%u BUS_RD 0x%08X\n", buf, cmdpos, n);
      // perform the read
      n = jtag_bus_rd(n, (uint32_t*)&m);
      cmd_printf("\t> (%X) %08X\n", n, m);
      // prepare the response
      if (n == 0b100) { // OK
        SET_WORD_AT(respbuf+resppos, m);
        resppos += 4;
      } else
        respbuf[resppos++] = n;
      cmdpos += 5;
      break;
    case XCMD_CPU_RD:
      // grab the 16-bit address
      n = GET_HWORD_AT(cmdbuf+cmdpos+1) & ~3;
      n |= 0xe0000000; // SCB_BASE
      cmd_printf(" %c# @%u CPU_RD 0x%08X\n", buf, cmdpos, n);
      // perform the read
      n = jtag_cpu_rd(n, (uint32_t*)&m);
      cmd_printf("\t> (%X) %08X\n", n, m);
      // prepare the response
      if (n == 0b100) { // OK
        SET_WORD_AT(respbuf+resppos, m);
        resppos += 4;
      } else
        respbuf[resppos++] = n;
      cmdpos += 3;
      break;

    // perform bus writes
    case XCMD_BUS_WR:
      // grab the address
      n = GET_WORD_AT(cmdbuf+cmdpos+1) & ~3;
      m = GET_WORD_AT(cmdbuf+cmdpos+5);
      cmd_printf(" %c# @%u BUS_WR 0x%08X 0x%08X\n", buf, cmdpos, n, m);
      // perform the write
      n = jtag_bus_wr(n, m);
      cmd_printf("\t> (%X)\n", n);
      // prepare the response
      respbuf[resppos++] = n;
      cmdpos += 9;
      break;
    case XCMD_CPU_WR:
      // grab the address
      n = GET_HWORD_AT(cmdbuf+cmdpos+1) & ~3;
      m = GET_WORD_AT(cmdbuf+cmdpos+3);
      n |= 0xe0000000; // SCB_BASE
      cmd_printf(" %c# @%u CPU_WR 0x%08X 0x%08X\n", buf, cmdpos, n, m);
      // perform the write
      n = jtag_cpu_wr(n, m);
      cmd_printf("\t> (%X)\n", n);
      // prepare the response
      respbuf[resppos++] = n;
      cmdpos += 7;
      break;

    // perform CPU bus write-and-readback
    case XCMD_CPU_WRRD:
      // grab the address
      n = GET_HWORD_AT(cmdbuf+cmdpos+1) & ~3;
      m = GET_WORD_AT(cmdbuf+cmdpos+3);
      n |= 0xe0000000; // SCB_BASE
      cmd_printf(" %c# @%u CPU_WRRD 0x%08X 0x%08X\n", buf, cmdpos, n, m);
      // perform the write
      m = jtag_cpu_wr(n, m);
      // if the write failed, don't attempt the read back
      if (m != 0b100) {
        cmd_printf("\twr> (%X)\n", m);
        // prepare the response
        respbuf[resppos++] = n|0x10;
      }
      // if it succeeded
      else {
        n = jtag_cpu_rd(n, (uint32_t*)&m);
        cmd_printf("\t> (%X) %08X\n", n, m);
        if (n == 0b100) { // OK
          SET_WORD_AT(respbuf+resppos, m);
          resppos += 4;
        } else
          respbuf[resppos++] = n;
      }
      cmdpos += 7;
      break;

#   if 0
      // - bus (SYS) block accesses
    case XCMD_BLOCK_RD:
    case XCMD_BLOCK_WR:
    case XCMD_BLOCK_FILL:
      break;
#   endif
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
  if (have_pincfg_set) {
    crt.pin = iox_get_all();
    crt.cfg = iox_getcfg_all();
    crt.pull = iox_getcfg_all();
    // if, and only if, both TILESEL pins are outputs
    if (! (crt.cfg & TILESEL_MASK)) {
      // if a TILESEL pins was previously an input, or if the the value changed
      if ((last.cfg & TILESEL_MASK) ||
          (last.pin & TILESEL_MASK) != (crt.pin & TILESEL_MASK))
        jcfg_set_tilesel((crt.pin & TILESEL_MASK) >> TILESEL_POS);
    }
    if ((last.pull != crt.pull) || (last.cfg != crt.cfg) ||
        ((last.pin & ~last.cfg) != (crt.pin & ~crt.cfg)))
      iox_debug();
    else
      printf("IOX: nothing changed (TILESEL=%u, CLKSRC=%u)\n", 
        (crt.pin & TILESEL_MASK) >> TILESEL_POS, (crt.pin & CLKSRC_MASK) >> CLKSRC_POS);
  }
  // protocol forbids responses that are a multiple of 64 bytes, so if that was the case, add one extra byte
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
