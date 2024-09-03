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
};

enum CommandModifier
{
  // CMD_XFER
  NO_READ = 0x80,
  EXTEND_LENGTH = 0x40,
  // CMD_CLK
  READOUT = 0x80,
  // CMD_A5CLK
  TURN_ON = 0x80
};

enum SignalIdentifier {
  SIG_TCK = 1 << 1,
  SIG_TDI = 1 << 2,
  SIG_TDO = 1 << 3,
  SIG_TMS = 1 << 4,
  SIG_TRST = 1 << 5,
  SIG_SRST = 1 << 6
};

unsigned cmd_execute(pio_jtag_inst_t* jtag, int buf,const uint8_t *cmdbuf, unsigned cmdsz, uint8_t *respbuf)
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
      cmd_printf (" %c# @%u CMD_SETVOLTAGE (no-op)\n", '0'+buf, cmdpos);
      ++cmdpos;
      break;

    case CMD_GOTOBOOTLOADER:
      printf (" %c# @%u CMD_BOOTLOADER\n", '0'+buf, cmdpos);
      reset_usb_boot(0, 0);
      while (1)
        asm volatile ("wfe");
      break;

    case CMD_REBOOT:
      printf (" %c# @%u CMD_REBOOT\n", '0'+buf, cmdpos);
      watchdog_reboot(0, 0, 1); // standard boot in 1ms
      while (1)
        asm volatile ("wfe");
      break;

    case CMD_INFO:
      n = strlen(djtag_whoami());
      cmd_printf (" %c# @%u CMD_INFO >%u\n", '0'+buf, cmdpos, n);
      memcpy(respbuf+resppos, djtag_whoami(), n);
      resppos += n;
      ++cmdpos;
      break;

    case CMD_GETCLKS:
      cmd_printf (" %c# @%u CMD_GETCLKS >%u\n", '0'+buf, cmdpos, sizeof(djtag_clocks));
      memcpy(respbuf+resppos, &djtag_clocks, sizeof(djtag_clocks));
      resppos += sizeof(djtag_clocks);
      ++cmdpos;
      break;

    case CMD_FREQ:
      cmd_printf (" %c# @%u CMD_FREQ\n", '0'+buf, cmdpos);
      jtag_set_clk_freq(jtag, ((unsigned)cmdbuf[cmdpos+1] << 8) | cmdbuf[cmdpos+2]);
      cmdpos += 3;
      break;

    case CMD_A5FREQ:
      cmd_printf (" %c# @%u CMD_A5FREQ\n", '0'+buf, cmdpos);
      extern pio_a5clk_inst_t a5clk;
      a5clk_set_freq(&a5clk, ((unsigned)cmdbuf[cmdpos+1] << 8) | cmdbuf[cmdpos+2]);
      cmdpos += 3;
      break;

    case CMD_GETSIG:
      cmd_printf (" %c# @%u CMD_GETSIG >1\n", '0'+buf, cmdpos);
      n = 0;
      if (jtag_get_tdo(jtag))
        n |= SIG_TDO;
      respbuf[resppos++] = n;
      ++cmdpos;
      break;

    case CMD_SETSIG:
      cmd_printf (" %c# @%u CMD_SETSIG\n", '0'+buf, cmdpos);
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
      cmd_printf (" %c# @%u CMD_A5CLK %s\n", '0'+buf, cmdpos, (cmd == CMD_A5CLK) ? "OFF" : "ON");
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
      cmd_printf (" %c# @%u CMD_CLK%s %u%s\n", '0'+buf, cmdpos, (cmd & READOUT)?",READOUT":"", m, (cmd & READOUT)?" >1":"");
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
        cmd_printf (" %c# @%u CMD_XFER%s,NOREAD %u\n", '0'+buf, cmdpos,
                    (n >= 256)?",LARGE":"", n);
      else
        cmd_printf (" %c# @%u CMD_XFER%s %u >%u\n", '0'+buf, cmdpos,
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
      cmd_printf (" %c# @%u ??? 0x%02X\n", '0'+buf, cmdpos, cmdbuf[cmdpos]);
      watchdog_reboot(0, 0, 100); // standard boot in 100ms (give time to UART to flush)
      while (1)
        asm volatile ("wfe");
      break;
    }
  }
  // protocol forbids responses that are a multiple of 64 bytes, sof if that was the case, add one extra byte
  // there is one obvious exception, the null response
  if (resppos && ! (resppos & 63)) {
    cmd_printf (" %c# resp_sz=0x%X; padding\n", '0'+buf, resppos);
    respbuf[resppos++] = 0xA5;
  }
  if (resppos) {
    cmd_printf (" %c# [", '0'+buf);
    for (int i = 0; i < resppos; ++i ) {
      cmd_printf (" %02X", respbuf[i]);
    }
    cmd_printf (" ]\n");
  }
  cmd_printf (" %c# done, cmd_sz=%u(%u) resp_sz=0x%u\n", '0'+buf, cmdpos, cmdsz, resppos);
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

