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
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <hardware/clocks.h>
#include <tusb.h>

#include "a5pins.h"
#include "arm.h"
#include "config.h"
#include "pio_jtag.h"
#include "jtag.pio.h"
#include "adc.h"
#include "adf4368.h"
#include "utils.h"
#include "cmd.h"

#define cmd_printf(...) ((void)(__VA_ARGS__))
//#define cmd_printf(...) (printf(__VA_ARGS__))

extern struct djtag_cfg_s jcfg;

static const char *jtag_codes[] = {
  "'b000", "WAIT",  "FAULT", "'b011",
  "OK",    "'b101", "'b110", "'b111"
};
static const char *jtag_decode_response(unsigned r)
{
  if (r < 8)
    return jtag_codes[r];
  static char jtag_code[12];
  sprintf(jtag_code, "%d", r);
  return jtag_code;
}

unsigned cmd_execute(pio_jtag_inst_t* jtag, char buf, const uint8_t *cmdbuf, unsigned cmdsz, uint8_t *respbuf)
{
  unsigned cmdpos = 0, resppos = 0;
  int pin, cfg; // pin config
  struct pindesc_s pindesc; // pin description
  extern struct djtag_clk_s djtag_clocks;
  int n, m;
  uint32_t a0, a1, a2, a3; // arguments (jtagx,armx)
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
      fatal_error();
      break;

    case CMD_REBOOT:
      cmd_printf(" %c# @%u CMD_REBOOT\n", buf, cmdpos);
      puts("Rebooting...");
      bad_error();
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
      // support values above 64K by passing 0 as the 16-bit value
      n = m = ((unsigned)cmdbuf[cmdpos+1] << 8) | cmdbuf[cmdpos+2];
      if (! n) {
        n = GET_WORD_AT(cmdbuf+cmdpos+3);
        cmdpos += 4;
      }
      cmd_printf(" %c# @%u CMD_FREQ%s %u\n", buf, cmdpos, m?"":"X", n);
      jtag_set_clk_freq(jtag, n);
      cmdpos += 3;
      break;

    case CMD_A5FREQ:
      // support values above 64K by passing 0 as the 16-bit value
      n = m = ((unsigned)cmdbuf[cmdpos+1] << 8) | cmdbuf[cmdpos+2];
      if (! n) {
        n = GET_WORD_AT(cmdbuf+cmdpos+3);
        cmdpos += 4;
      }
      extern pio_a5clk_inst_t a5clk;
      cmd_printf(" %c# @%u CMD_A5FREQ%s %u\n", buf, cmdpos, m?"":"X", n);
      a5clk_set_freq(&a5clk, n);
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
      n = cmdbuf[cmdpos+1]; // mask
      m = cmdbuf[cmdpos+2]; // status
      cmd_printf(" %c# @%u CMD_SETSIG %02X %02X\n", buf, cmdpos, n, m);
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
      if (n != 1)
        printf("!BYPASS:%u\n", n);
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
      if (n != 1) {
        printf("!IDCODE_SCAN:%u\n", n);
        for (int i = 0; i < n; ++i)
          printf(" %08X", idcodes[i]);
        printf("\n");
      }
      cmdpos++;
      break;

    // configure jtagx mode
    case XCMD_CONFIG:
      cmd_printf(" %c# @%u XCMD_CONFIG\n", buf, cmdpos);
      cmdpos += 1 + jtag_set_config(cmdbuf + cmdpos + 1, cmdsz - (cmdpos + 1));
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
      cmdpos += 2;
      // do the abort
      n = jtag_abort(n);
      // no response to ABORT
      break;

    // perform a DPACC read
    case XCMD_DPACC_RD:
      // grab the address, 6-bits, rshifted by 2: (0..0xfc)>>2
      a0 = (cmdbuf[cmdpos+1] & 0x3f) << 2;
      cmd_printf(" %c# @%u DPACC_RD @0x%02X\n", buf, cmdpos, a0);
      cmdpos += 2;
      // do the read
      n = jtag_dpacc_rd(a0, (uint32_t*)&m);
      // prepare the response
      respbuf[resppos++] = n;
      SET_WORD_AT(respbuf+resppos, m);
      resppos += 4;
      if (n == JTAG_OK)
        cmd_printf("\t> %08X\n", m);
      else {
        cmd_printf("\t> (%s)\n", jtag_decode_response(n));
        printf("!DPACC_RD(0x%02X) > 'b%03b\n", a0, n);
      }
      break;

    // perform a DPACC write
    case XCMD_DPACC_WR:
      // grab the address, 6-bits, rshifted by 2: (0..0xfc)>>2
      a0 = (cmdbuf[cmdpos+1] & 0x3f) << 2;
      // grab the data
      a1 = GET_WORD_AT(cmdbuf+cmdpos+2);
      cmd_printf(" %c# @%u DPACC_WR @0x%02X 0x%X\n", buf, cmdpos, a0, a1);
      cmdpos += 6;
      // do the write
      n = jtag_dpacc_wr(a0, a1);
      // prepare the response
      respbuf[resppos++] = n;
      cmd_printf("\t> (%s)\n", jtag_decode_response(n));
      if (n != JTAG_OK)
        printf("!DPACC_WR(0x%02X,0x%08X) > 'b%03b\n", a0, a1, n);
      break;

    // perform an APACC read
    case XCMD_APACC_RD:
      // grab the AP
      a0 = cmdbuf[cmdpos+1];
      // grab the address, 11-bits, rshifted by 2: (0..0x1ffc)>>2
      a1 = (GET_HWORD_AT(cmdbuf+cmdpos+2) & 0x7ff) << 2;
      cmdpos += 4;
      cmd_printf(" %c# @%u APACC_RD%s @%u:0x%04X\n", buf, cmdpos,
                 (a0 >> 7)?"\'":"", a0 & 0x7f, a1 );
      // do the read
      n = jtag_apacc_rd(a0, a1, (uint32_t*)&m);
      // prepare the response
      respbuf[resppos++] = n;
      SET_WORD_AT(respbuf+resppos, m);
      resppos += 4;
      if (n == JTAG_OK)
        cmd_printf("\t> %08X\n", m);
      else {
        cmd_printf("\t> (%s)\n", jtag_decode_response(n));
        printf("!APACC_RD(%u:0x%04X) > 'b%03b\n", a0&0x7f, a1, n);
      }
      break;

    // perform an APACC write
    case XCMD_APACC_WR:
      // grab the AP
      a0 = cmdbuf[cmdpos+1];
      // grab the address, 11-bits, rshifted by 2: (0..0x1ffc)>>2
      a1 = (GET_HWORD_AT(cmdbuf+cmdpos+2) & 0x7ff) << 2;
      // grab the data
      a2 = GET_WORD_AT(cmdbuf+cmdpos+4);
      cmd_printf(" %c# @%u APACC_WR%s @%u:0x%04X 0x%X\n", buf, cmdpos,
                 (a0 >> 7)?"\'":"", a0 & 0x7f, a1, a2 );
      cmdpos += 8;
      // do the write
      n = jtag_apacc_wr(a0, a1, a2);
      // prepare the response
      respbuf[resppos++] = n;
      cmd_printf("\t> (%s)\n", jtag_decode_response(n));
      if (n != JTAG_OK)
        printf("!APACC_WR(%u:0x%04X,0x%08X) > 'b%03b\n", a0&0x7f, a1, a2, n);
      break;

    // perform bus reads
    case XCMD_BUS_RD:
      // grab the address
      a0 = GET_WORD_AT(cmdbuf+cmdpos+1) & ~3;
      cmd_printf(" %c# @%u BUS_RD 0x%04X\n", buf, cmdpos, a0);
      cmdpos += 5;
      // perform the read
      n = jtag_bus_rd(a0, (uint32_t*)&m);
      cmd_printf("\t> (%X) %08X\n", n, m);
      // prepare the response
      respbuf[resppos++] = n;
      if (n == JTAG_OK) {
        SET_WORD_AT(respbuf+resppos-1, m);
        resppos += 3;
        cmd_printf("\t> %08X\n", m);
      }
      else {
        cmd_printf("\t> (%s)\n", jtag_decode_response(n));
        printf("!BUS_RD(0x%08X) > 'b%03b\n", a0, n);
      }
      break;
    case XCMD_CPU_RD:
      // grab the 16-bit address
      a0 = GET_HWORD_AT(cmdbuf+cmdpos+1) & ~3;
      a0 |= jcfg.dsubase; // SCB_BASE
      cmd_printf(" %c# @%u CPU_RD 0x%X\n", buf, cmdpos, a0);
      cmdpos += 3;
      // perform the read
      n = jtag_cpu_rd(a0, (uint32_t*)&m);
      cmd_printf("\t> (%X) %08X\n", n, m);
      // prepare the response
      respbuf[resppos++] = n;
      if (n == JTAG_OK) {
        SET_WORD_AT(respbuf+resppos-1, m);
        resppos += 3;
        cmd_printf("\t> %08X\n", m);
      }
      else {
        cmd_printf("\t> (%s)\n", jtag_decode_response(n));
        printf("!CPU_RD(0x%08X) > 0b%03b\n", a0, n);
      }
      break;

    // perform bus writes
    case XCMD_BUS_WR:
      // grab the address
      a0 = GET_WORD_AT(cmdbuf+cmdpos+1) & ~3;
      a1 = GET_WORD_AT(cmdbuf+cmdpos+5);
      cmd_printf(" %c# @%u BUS_WR 0x%04X 0x%X\n", buf, cmdpos, a0, a1);
      cmdpos += 9;
      // perform the write
      n = jtag_bus_wr(a0, a1);
      // prepare the response
      respbuf[resppos++] = n;
      cmd_printf("\t> (%s)\n", jtag_decode_response(n));
      if (n != JTAG_OK)
        printf("!BUS_WR(0x%08X,0x%08X) > 'b%03b\n", a0, a1, n);
      break;
    case XCMD_CPU_WR:
      // grab the address
      a0 = GET_HWORD_AT(cmdbuf+cmdpos+1) & ~3;
      a1 = GET_WORD_AT(cmdbuf+cmdpos+3);
      a0 |= jcfg.dsubase; // SCB_BASE
      cmd_printf(" %c# @%u CPU_WR 0x%X 0x%X\n", buf, cmdpos, a0, a1);
      cmdpos += 7;
      // perform the write
      n = jtag_cpu_wr(a0, a1);
      // prepare the response
      respbuf[resppos++] = n;
      cmd_printf("\t> (%s)\n", jtag_decode_response(n));
      if (n != JTAG_OK)
        printf("!CPU_WR(0x%08X,0x%08X) > 'b%03b\n", a0, a1, n);
      break;

    // perform CPU bus write-and-readback
    case XCMD_CPU_WRRD:
      // grab the address
      a0 = GET_HWORD_AT(cmdbuf+cmdpos+1) & ~3;
      a1 = GET_WORD_AT(cmdbuf+cmdpos+3);
      a0 |= jcfg.dsubase; // SCB_BASE
      cmd_printf(" %c# @%u CPU_WRRD 0x%X 0x%X\n", buf, cmdpos, a0, a1);
      cmdpos += 7;
      // perform the write
      m = jtag_cpu_wr(a0, a1);
      // if the write failed, don't attempt the read back
      if (m != JTAG_OK) {
        // prepare the response
        respbuf[resppos++] = m|0x10;
        cmd_printf("\twr> (%s)\n", jtag_decode_response(m));
        printf("!CPU_WRRD(0x%08X,0x%08X) wr > 'b%03b\n", a0, a1, m);
        break;
      }
      // if it succeeded
      n = jtag_cpu_rd(a0, (uint32_t*)&m);
      // prepare the response
      respbuf[resppos++] = n;
      if (n == JTAG_OK) {
        SET_WORD_AT(respbuf+resppos-1, m);
        resppos += 3;
        cmd_printf("\trd> %08X\n", m);
      }
      else {
        cmd_printf("\trd> (%s)\n", jtag_decode_response(n));
        printf("!CPU_WRRD(0x%08X,0x%08X) wr > 'b%03b\n", a0, a1, n);
      }
      break;

    // bus (SYS) block accesses
    case XCMD_BLOCK_RD:
      // grab the address and count
      a0 = 1 + cmdbuf[cmdpos+1];
      a1 = GET_WORD_AT(cmdbuf+cmdpos+2) & ~3;
      cmd_printf(" %c# @%u BLOCK_RD 0x%04X %u\n", buf, cmdpos, a1, a0);
      cmdpos += 6;
      // perform the reads; bail out on the first error
      m = a0; n = a1;
      int p;
      while (m--) {
        uint32_t q;
        p = jtag_bus_rd(n, &q);
        if (p == JTAG_OK)
          SET_WORD_AT(respbuf+resppos, q);
        else {
          respbuf[resppos++] = p;
          printf("!BUS_XRD(%u,0x%04X) @0x%04X > 'b%03b\n", a0, a1, n, p);
          break;
        }
        n += 4; resppos += 4;
      }
      // print the number of read left, the last address we reached, and the most recent response
      cmd_printf("\t> %d 0x%04X (%s)\n", a0-(m+1), n, jtag_decode_response(p));
      break;
    case XCMD_BLOCK_WR:
      // grab the address and count
      a0 = 1 + cmdbuf[cmdpos+1];
      a1 = GET_WORD_AT(cmdbuf+cmdpos+2) & ~3;
      cmd_printf(" %c# @%u BLOCK_WR 0x%04X %u\n", buf, cmdpos, a1, a0);
      cmdpos += 6 + (a0*4);
      // perform the writes; bail out on the first error
      m = a0; n = a1;
      p = cmdpos - (a0*4);
      while (m--) {
        a2 = jtag_bus_wr(n, GET_WORD_AT(cmdbuf+p));
        if (a2 != JTAG_OK) {
          printf("!BUS_XWR(%u,0x%04X) @0x%04X -> 'b%03b\n", a0, a1, n, a2);
          break;
        }
        n += 4; p += 4;
      }
      // send back the number of writes not completed, and the most recent response
      respbuf[resppos++] = m;
      respbuf[resppos++] = a2;
      // print the number of writes left, the last address we reached, and the most recent response
      cmd_printf("\t> %d 0x%04X (%s)\n", m+1, n, jtag_decode_response(p));
      // skip the reminder of the command regardless of the number of locations actually written
      break;
    case XCMD_BLOCK_FILL:
      // grab the address, count and value
      a0 = 1 + cmdbuf[cmdpos+1];
      a1 = GET_WORD_AT(cmdbuf+cmdpos+2) & ~3;
      a2 = GET_WORD_AT(cmdbuf+cmdpos+6);
      cmdpos += 10;
      cmd_printf(" %c# @%u BLOCK_FILL 0x%04X %u 0x%08X\n", buf, cmdpos, a1, a0, a2);
      // perform the writes; bail out on the first error
      m = a0; n = a1;
      while (m--) {
        p = jtag_bus_wr(n, a2);
        if (p != JTAG_OK) {
          printf("!BUS_FILL(%u,0x%04X,0x%08X) @0x%04X -> 'b%03b\n", a0, a1, a2, n, p);
          break;
        }
        n += 4;
      }
      // send back the number of writes not completed, and the most recent response
      respbuf[resppos++] = m;
      respbuf[resppos++] = p;
      // print the number of writes left, the last address we reached, and the most recent response
      cmd_printf("\t> %d 0x%04X (%s)\n", a0-(m+1), n, jtag_decode_response(p));
      break;
    // read an ASCIIZ string
    case XCMD_ASCIIZ_RD:
      // grab the address
      a0 = GET_WORD_AT(cmdbuf+cmdpos+1);
      cmd_printf(" %c# @%u BUS_RD_AZ 0x%X\n", buf, cmdpos, a0);
      cmdpos += 5;
      // perform the read (directly to respbuf)
      // send back a single 0 (empty string), single non-zero (JTAG state) or
      // a pascal string with the result
      n = jtag_bus_asciiz_rd(a0, respbuf+resppos+1);
      if (n >= 0) {
        cmd_printf("\t> (%d) \"%.*s%s\"\n", n, ((n-1)>20)?16:n, respbuf+resppos+1, ((n-1)>20)?"...":"");
        respbuf[resppos] = n;
        resppos += 1 + n;
      } else {
        cmd_printf("\t> (%s)\n", jtag_decode_response(-n));
        respbuf[resppos++] = -n;
      }
      break;

    case XCMD_ARM_INIT: {
      // void arm_init(
      //   uint32_t imc_write, unsigned sz_imc_write,
      //   uint32_t reset_addr, unsigned dbg_bit, unsigned m0_bit);
      uint32_t imcwrite_addr = GET_WORD_AT(cmdbuf+cmdpos+1) & ~1;
      uint16_t imcwrite_sz = GET_WORD_AT(cmdbuf+cmdpos+5) & ~1;
      uint32_t reset_addr = GET_WORD_AT(cmdbuf+cmdpos+7) & ~3;
      uint8_t rst_dbg_bit = cmdbuf[cmdpos+11];
      uint8_t rst_m0_bit = cmdbuf[cmdpos+12];
      cmdpos += 13;
      // this is as ugly as it gets; if the reset address is 0x5... (that means, alpha7 tile),
      // set ram_base to 0x1...
      uint32_t ram_base = 0;
      if ((reset_addr >> 28) == 0x5) {
        ram_base = 0x10000000 | (reset_addr & 0x0fff0000);
      }
      // if reset_addr is 0x4..., we could be on either a5 or a7
      else if ((reset_addr >> 28) == 0x4) {
        extern uint32_t last_seen_idcode;
        ram_base = 0x1fff0000;
        if (last_seen_idcode != IDCODE_A7)
          ram_base = 0;
      }
      cmd_printf(" %c# @%u ARM_INIT imc_write=0x%04X..%04X, rst_addr=0x%X m0=%u m0dbg=%u ram_base=0x%08X\n", 
        buf, cmdpos,
        imcwrite_addr, imcwrite_addr + imcwrite_sz - 1, 
        reset_addr, rst_dbg_bit, rst_m0_bit,
        ram_base);
      arm_init(imcwrite_addr, imcwrite_sz, 
               reset_addr, rst_dbg_bit, rst_m0_bit, ram_base);
      }
      break;
    case XCMD_ARM_RESUME:
      // int arm_resume(void);
      // returns: 0=already_running, 1=resumed, 2=locked_up, <0=error
      cmd_printf(" %c# @%u ARM_RESUME\n", buf, cmdpos);
      cmdpos++;
      n = arm_resume();
      cmd_printf("\t> %d\n", n);
      respbuf[resppos++] = n;
      break;
    case XCMD_ARM_QUERY:
      // int get_arm_state(uint8_t *resp)
      // function returns the number of output bytes
      // {
      //   reg_sz, imc_sz : uint16_t;
      //   regs : {
      //     systick, dhcsr : uint32_t; // systick valid only if NOT sleeping
      //     // if sleeping, locked/halted or crashed
      //     pc : uint32_t;
      //     // if locked/halted or crashed
      //     r[15] : uint32_t;
      //     dfsr : uint32_t;
      //     msp, psp : uint32_t;
      //     icsr : uint32_t;
      //     ctrl_primask : uint32_t;
      //     // if an exception happened (crashed)
      //     except_r[4], except_r12 : uint32_t;
      //     except_lr, except_pc, except_psr, except_sp : uint32_t;
      //   }
      //   imc : char[imc_sz];
      // }
      cmd_printf(" %c# @%u ARM_QUERY\n", buf, cmdpos);
      while (resppos & 3)
        respbuf[resppos++] = 0xcd;
      n = get_arm_state(respbuf + resppos);
      m = *(uint16_t*)(respbuf + resppos);
      cmd_printf("\t> %d (%d+%d)", n, m, *(uint16_t*)(respbuf + resppos + 2));
      if (m >= 8) {
        uint32_t dhcsr = *(uint32_t*)(respbuf + resppos + 8);
        const char *state = "running";
        if (dhcsr & DHCSR_S_SLEEP)
          state = "asleep";
        else if (dhcsr & DHCSR_S_LOCKUP)
          state = "lockup";
        else if (dhcsr & DHCSR_S_HALT)
          state = "halted";
        cmd_printf(" %s @t=0x%06X", state, *(uint32_t*)(respbuf + resppos + 4));
        if (m >= 12)
          cmd_printf(" pc=0x%04X", *(uint32_t*)(respbuf + resppos + 12));
      }
      cmd_printf("\n");
      cmdpos++;
      resppos += n;
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

    // fPLL_setBaud(int f_khz) -> int
    case CMD_FPLL_SETBAUD:
      cfg = GET_WORD_AT(cmdbuf+cmdpos+1);
      cmd_printf(" %c# @%u FPLL_SETBAUD %uKHz\n", buf, cmdpos, cfg);
      cfg = do_fpll_config(cfg);
      cmd_printf("\t> %d\n", cfg);
      SET_WORD_AT(respbuf+resppos, cfg);
      cmdpos += 5;
      resppos += sizeof(uint32_t);
      break;
    // fPLL_setBaud(uint8 n, uint8[n] data) -> uint8 n, uint8[n] 
    case CMD_FPLL_WRRD:
      n = cmdbuf[cmdpos+1];
      m = (((unsigned)cmdbuf[cmdpos+2]) << 8) | cmdbuf[cmdpos+3];
      cmd_printf(" %c# @%u FPLL_WRRD %u", buf, cmdpos, 1+n);
      if (n >= 2) {
        if(! (m & 0x8000))
          cmd_printf(" w%02X,%02X%s\n", m, cmdbuf[cmdpos+4], (n>2)?"...":"");
        else
          cmd_printf(" r%02X\n", m&0x7fff);
      }
      m = do_fpll_wrrd(cmdbuf + cmdpos + 2, respbuf + resppos + 1, 1+n) - 1;
      if (m < 0)
        m = 0;
      if (m != n)
        cmd_printf("\t> error\n");
      else {
        if (m >= 2) {
          if(! (cmdbuf[cmdpos+2] & 0x80))
            cmd_printf("\t> ok\n");
          else
            cmd_printf("\t> %02X%s\n", respbuf[resppos+1], (m>2)?"...":"");
        }
      }
      cmdpos += 3+n;
      respbuf[resppos++] = m;
      resppos += 1+n;
      break;

    default:
      // invalid command: reboot
      // also: print the entire command byte, not just cmd[6:0]
      printf(" %c# @%u ??? 0x%02X\n", buf, cmdpos, cmdbuf[cmdpos]);
      fatal_error();
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
  cmd_printf("    [%u(%u) > 0x%X]", cmdpos, cmdsz, resppos);
  if (resppos) {
    cmd_printf(" :" );
    int i;
    if (resppos <= 8) {
      for (i = 0; i < resppos; ++i)
        cmd_printf(" %02X", respbuf[i]);
    } else {
      for (i = 0; i < 4; ++i)
        cmd_printf(" %02X", respbuf[i]);
      cmd_printf (" ...");
      for (i = resppos - 4; i < resppos; ++i)
        cmd_printf(" %02X", respbuf[i]);
    }
  }
  cmd_printf("\n");
  return resppos;
}
