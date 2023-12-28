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

#include "jtag.pio.h"
#include "tusb.h"
#include "pio_jtag.h"
#include "git.h"
#include "cmd.h"


enum CommandIdentifier {
  CMD_STOP = 0x00,
  CMD_INFO = 0x01,
  CMD_FREQ = 0x02,
  CMD_XFER = 0x03,
  CMD_SETSIG = 0x04,
  CMD_GETSIG = 0x05,
  CMD_CLK = 0x06,
  CMD_SETVOLTAGE = 0x07,
  CMD_GOTOBOOTLOADER = 0x08,
  CMD_REBOOT = 0x09,
  CMD_GETCLKS = 0x0a
};

enum CommandModifier
{
  // CMD_XFER
  NO_READ = 0x80,
  EXTEND_LENGTH = 0x40,
  // CMD_CLK
  READOUT = 0x80,
};

enum SignalIdentifier {
  SIG_TCK = 1 << 1,
  SIG_TDI = 1 << 2,
  SIG_TDO = 1 << 3,
  SIG_TMS = 1 << 4,
  SIG_TRST = 1 << 5,
  SIG_SRST = 1 << 6
};

/**
 * @brief Handle CMD_INFO command
 *
 * CMD_INFO returns a string to the host software. This
 * could be used to check DirtyJTAG firmware version
 * or supported commands.
 *
 * @param usbd_dev USB device
 */
static uint32_t  cmd_info(uint8_t *buffer);

/**
 * @brief Handle CMD_FREQ command
 *
 * CMD_FREQ sets the clock frequency on the probe.
 * Currently this does not changes anything.
 *
 * @param commands Command data
 */
static void cmd_freq(pio_jtag_inst_t* jtag, const uint8_t *commands);

/**
 * @brief Handle CMD_XFER command
 *
 * CMD_XFER reads and writes data simultaneously.
 *
 * @param usbd_dev USB device
 * @param commands Command data
 */
static uint32_t cmd_xfer(pio_jtag_inst_t* jtag, const uint8_t *commands, bool extend_length, bool no_read, uint8_t* tx_buf);

/**
 * @brief Handle CMD_SETSIG command
 *
 * CMD_SETSIG set the logic state of the JTAG signals.
 *
 * @param commands Command data
 */
static void cmd_setsig(pio_jtag_inst_t* jtag, const uint8_t *commands);

/**
 * @brief Handle CMD_GETSIG command
 *
 * CMD_GETSIG gets the current signal state.
 * 
 * @param usbd_dev USB device
 */
static uint32_t cmd_getsig(pio_jtag_inst_t* jtag, uint8_t *buffer);

/**
 * @brief Handle CMD_CLK command
 *
 * CMD_CLK sends clock pulses with specific TMS and TDI state.
 *
 * @param usbd_dev USB device
 * @param commands Command data
 * @param readout Enable TDO readout
 */
static uint32_t cmd_clk(pio_jtag_inst_t *jtag, const uint8_t *commands, bool readout, uint8_t *buffer);
/**
 * @brief Handle CMD_SETVOLTAGE command
 *
 * CMD_SETVOLTAGE sets the I/O voltage for devices that support this feature.
 *
 * @param commands Command data
 */
static void cmd_setvoltage(const uint8_t *commands);

/**
 * @brief Handle CMD_GOTOBOOTLOADER command
 *
 * CMD_GOTOBOOTLOADER resets the MCU and enters its bootloader (if installed)
 */
static void cmd_gotobootloader(void);

/**
 * @brief Handle CMD_REBOOT command
 *
 * CMD_REBOOT resets the MCU, without entering the bootloader
 */
static void cmd_reboot(void);

/**
 * @brief Handle CMD_GETCLKS command
 *
 * CMD_GETCLKS gets the jtag clock configuration
 */
static uint32_t cmd_getclks(uint8_t *buffer);

void cmd_handle(pio_jtag_inst_t* jtag, uint8_t* rxbuf, uint32_t count, uint8_t* tx_buf) {
  uint8_t *commands= (uint8_t*)rxbuf;
  uint8_t *output_buffer = tx_buf;
  while ((commands < (rxbuf + count)) && (*commands != CMD_STOP))
  {
    switch ((*commands)&0x0F) {
    case CMD_INFO:
    {
      uint32_t trbytes = cmd_info(output_buffer);
      output_buffer += trbytes;
      break;
    }
    case CMD_GETCLKS:
    {
      uint32_t trbytes = cmd_getclks(output_buffer);
      output_buffer += trbytes;
      break;
    }
    case CMD_FREQ:
      cmd_freq(jtag, commands);
      commands += 2;
      break;

    case CMD_XFER:
    {
      bool no_read = *commands & NO_READ;
      uint32_t trbytes = cmd_xfer(jtag, commands, *commands & EXTEND_LENGTH, no_read, output_buffer);
      commands += 1 + trbytes;
      output_buffer += (no_read ? 0 : trbytes);
      break;
    }
    case CMD_SETSIG:
      cmd_setsig(jtag, commands);
      commands += 2;
      break;

    case CMD_GETSIG:
    {
      uint32_t trbytes = cmd_getsig(jtag, output_buffer);
      output_buffer += trbytes;
      break;
    }
    case CMD_CLK:
    {
      uint32_t trbytes = cmd_clk(jtag, commands, !!(*commands & READOUT), output_buffer);
      output_buffer += trbytes;
      commands += 2;
      break;
    }
    case CMD_SETVOLTAGE:
      cmd_setvoltage(commands);
      commands += 1;
      break;

    case CMD_GOTOBOOTLOADER:
      cmd_gotobootloader();
      break;

    case CMD_REBOOT:
      cmd_reboot();
      break;

    default:
      return; /* Unsupported command, halt */
      break;
    }

    commands++;
  }
  /* Send the transfer response back to host */
# if 1
  if (tx_buf != output_buffer)
  {
    unsigned n = output_buffer - tx_buf;
    if (n > 64)
      n = 64;
    tud_vendor_write(tx_buf, n);
    tud_vendor_flush();
  }
# else
  uint8_t *pout = tx_buf;
  while (pout < output_buffer) {
    /* wait until we can send something */
    unsigned avl = tud_vendor_write_available();
    while (! avl) {
      tud_vendor_flush();
      sleep_ms(1);
      avl = tud_vendor_write_available();
    }
    /* send as much as we can right now */
    unsigned n = output_buffer - pout;
    if (n > avl)
      n = avl;
    tud_vendor_write(pout, n);
    unsigned m = 0;
    do {
      m = tud_vendor_flush();
      if (! m)
        sleep_ms(1);
    } while (m < n);
    pout += n;
  }
# endif
  return;
}

static uint32_t cmd_info(uint8_t *buffer) {
  const char *djtag_whoami();
  const char *whoami = djtag_whoami();
  int n = strlen(whoami);
  if (n > 252)
    n = 252;
  memcpy(buffer, whoami, n);
  return n;
}

static void cmd_freq(pio_jtag_inst_t* jtag, const uint8_t *commands) {
  jtag_set_clk_freq(jtag, (commands[1] << 8) | commands[2]);
}

//static uint8_t output_buffer[64];

static uint32_t cmd_xfer(pio_jtag_inst_t* jtag, const uint8_t *commands, bool extend_length, bool no_read, uint8_t* tx_buf) {
  uint16_t transferred_bits;
  uint8_t* output_buffer = 0;
  transferred_bits = commands[1];
  if (extend_length)
  {
    transferred_bits += 256;
  }
  // Ensure we don't do over-read
  if (transferred_bits > 62 * 8)
  {
    transferred_bits = 62 * 8;
  }

  /* Fill the output buffer with zeroes */
  if (!no_read)
  {
    output_buffer = tx_buf;
    memset(output_buffer, 0, (transferred_bits + 7) / 8);
  }

  jtag_transfer(jtag, transferred_bits, commands+2, output_buffer);

  return (transferred_bits + 7) / 8;
}

static void cmd_setsig(pio_jtag_inst_t* jtag, const uint8_t *commands) {
  uint8_t signal_mask, signal_status;

  signal_mask = commands[1];
  signal_status = commands[2];

  if (signal_mask & SIG_TCK) {
    jtag_set_clk(jtag,signal_status & SIG_TCK);
  }

  if (signal_mask & SIG_TDI) {
    jtag_set_tdi(jtag, signal_status & SIG_TDI);
  }

  if (signal_mask & SIG_TMS) {
    jtag_set_tms(jtag, signal_status & SIG_TMS);
  }
  
  if (signal_mask & SIG_TRST) {
    jtag_set_trst(jtag, signal_status & SIG_TRST);
  }

  if (signal_mask & SIG_SRST) {
    jtag_set_rst(jtag, signal_status & SIG_SRST);
  }
}

static uint32_t cmd_getsig(pio_jtag_inst_t* jtag, uint8_t *buffer)
{
  uint8_t signal_status = 0;
  
  if (jtag_get_tdo(jtag)) {
    signal_status |= SIG_TDO;
  }
  buffer[0] = signal_status;
  return 1;
}

static uint32_t cmd_clk(pio_jtag_inst_t *jtag, const uint8_t *commands, bool readout, uint8_t *buffer)
{
  uint8_t signals, clk_pulses;
  signals = commands[1];
  clk_pulses = commands[2];
  uint8_t readout_val = jtag_strobe(jtag, clk_pulses, signals & SIG_TMS, signals & SIG_TDI);

  if (readout)
  {
    buffer[0] = readout_val;
  }
  return readout ? 1 : 0;
}

static void cmd_setvoltage(const uint8_t *commands) {
  (void)commands;
}

static void cmd_gotobootloader(void) {
  reset_usb_boot(0, 0);
}

static void cmd_reboot(void) {
  watchdog_reboot(0, 0, 100); // standard boot in 100ms
  while (1)
  {
    asm volatile ("wfi");
  }
}

static uint32_t cmd_getclks(uint8_t *buffer) {
  extern const struct djtag_clk_s djtag_clocks;
  memcpy(buffer, &djtag_clocks, sizeof(djtag_clocks));
  return sizeof(djtag_clocks);
}
