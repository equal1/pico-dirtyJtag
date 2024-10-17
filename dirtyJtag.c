#include <stdint.h>
#include <stdio.h>
#include <pico/bootrom.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/pio.h"
#include "pico/multicore.h"
#include "pio_jtag.h"
#include "cdc_uart.h"
#include "led.h"
#include "bsp/board.h"
#include "tusb.h"
#include "cmd.h"
#include "get_serial.h"
#include "git.h"

#include "dirtyJtagConfig.h"

//#define DEBUG

#define MTU_SIZE 1500

// round up the buffer size to the next multiple of 64 bytes
#define BUFFER_SIZE ((MTU_SIZE + 63) & ~63)

#ifdef DEBUG
#define dprintf(...) printf(__VA_ARGS__)
#else
#define dprintf(...) (void)(__VA_ARGS__)
#endif

void init_pins()
{
  bi_decl(bi_4pins_with_names(PIN_TCK, "TCK", PIN_TDI, "TDI", PIN_TDO, "TDO", PIN_TMS, "TMS"));
# if (PIN_RST != -1)
  bi_decl(bi_1pin_with_name(PIN_RST, "RST"));
# endif
# if (PIN_TRST != -1)
  bi_decl(bi_1pin_with_name(PIN_TRST, "TRST"));
# endif
}

pio_jtag_inst_t jtag = {
  .pio = pio0,
  .sm = 0
};
pio_a5clk_inst_t a5clk = {
  .pio = pio1,
  .sm = 0
};


static char whoami[256];

const char *djtag_whoami()
{
  return whoami;
}

static void fatal_error()
{
  printf ("Rebooting to bootloader...\n");
  // this is fatal - wait 100ms then reboot to bootloader
  sleep_ms(100);
  reset_usb_boot(0, 0);
  while (1) asm volatile ("wfe");
}

void djtag_init()
{
  init_pins();
# if BOARD_TYPE != BOARD_QMTECH_RP2040_DAUGHTERBOARD
  init_jtag(&jtag, 1000, PIN_TCK, PIN_TDI, PIN_TDO, PIN_TMS, PIN_RST, PIN_TRST);
# else
  init_jtag(&jtag, 1000, PIN_TCK, PIN_TDI, PIN_TDO, PIN_TMS, 255, 255);
# endif
  init_a5clk(&a5clk, 1000, PIN_A5_CLK);
  // // enable A5 clock
  // pio_sm_set_enabled(a5clk.pio, a5clk.sm, true);
}

// use double buffering - while one buffer is busy receiving/transmitting,
// gather/generate data in the other one
typedef struct buffer_s
{
  // state:
  //   0: available
  //   1: receiving commands
  //   2: executing commands
  //   3: sending response
  volatile enum { UNUSED = 0, RECEIVING, READY_TO_EXECUTE, EXECUTING, SENDING } state;
  // byte counts
  volatile unsigned cmd_n, resp_n;
  // data
  uint8_t cmd[BUFFER_SIZE], resp[BUFFER_SIZE];
} buffer_t;

buffer_t buf[2];
static int crt_buf; // the currently receiving buffer

static void cdc_tasks();

void jtag_main_task()
{
  int done_buf = -1;
  if (multicore_fifo_rvalid()) {
    // if we get here, a reponse was just computed
    done_buf = multicore_fifo_pop_blocking();
    // this can ONLY ever happen on the buffer that isn't current!
    dprintf ("buf%u executed (resp_sz=%u)\n", done_buf, buf[done_buf].resp_n);
    if (done_buf != (crt_buf^1) ) {
      printf ("INTERNAL ERROR: got command execution on the wrong buffer !!!\n"
              "  done_buf=%u (expected %u); crt_buf=%u\n"
              "  buf[0]={state=%u cmd_n=%u resp_n=%u} buf[1]={state=%u cmd_n=%u resp_n=%u}\n",
              done_buf, crt_buf ^ 1, crt_buf, 
              buf[0].state, buf[0].cmd_n, buf[0].resp_n,
              buf[1].state, buf[1].cmd_n, buf[1].resp_n);
      fatal_error();
    }
  }
  // if a buffer was just completed
  if (done_buf >= 0) {
    // if it produced data, mark it as sending (otherwise, unused)
    if (buf[done_buf].resp_n) {
      buf[done_buf].state = SENDING;
      dprintf ("buf%u sending\n", done_buf);
    }
    else {
      buf[done_buf].state = UNUSED;
      dprintf ("buf%u available\n", done_buf);
    }
  }

  // if the current buffer is unused, mark it as receiving
  if (buf[crt_buf].state == UNUSED) {
    buf[crt_buf].state = RECEIVING;
    buf[crt_buf].cmd_n = 0;
    buf[crt_buf].resp_n = 0;
    dprintf ("buf%u receiving\n", crt_buf);
  }
  unsigned rxpos = buf[crt_buf].cmd_n;

  // while we have data to send, send it
  // (also receive in the current buffer; technically, no data should be received)
  if ((done_buf >= 0) && (buf[done_buf].state == SENDING)) {
    led_tx(1);
    unsigned txpos = 0;
    unsigned txn = buf[done_buf].resp_n;
    while (txn) {
      int had_usb_activity = 0;
      // enqueue as much data as we can (if any)
      unsigned avl = tud_vendor_write_available();
      unsigned n = txn;
      if (n > avl)
        n = avl;
      // enqueue any data, if we could
      if (n > 0) {
        dprintf (" %c> %u(%u)\n", '0'+done_buf, n, txn - n); // {buf}>{transmitted}({left})
        tud_vendor_write(buf[done_buf].resp + txpos, n);
        tud_vendor_flush();
        had_usb_activity = 1;
        txpos += n; txn -= n;
      }
      // run the TinyUSB device task
      tud_task();
      // if we got any data, enqueue it in the current buffer
      if (tud_vendor_available()) {
        // sanitize receive buffer state
        if (buf[crt_buf].state != RECEIVING) {
          printf ("INTERNAL ERROR: got data for buffer %u, but that buffer isn't RECEIVING !!!\n"
                  "  crt_buf=%u buf[0]={state=%u cmd_n=%u resp_n=%u} buf[1]={state=%u cmd_n=%u resp_n=%u}\n",
                  crt_buf,
                  buf[0].state, buf[0].cmd_n, buf[0].resp_n,
                  buf[1].state, buf[1].cmd_n, buf[1].resp_n);
          fatal_error();
        }
        n = tud_vendor_read(buf[crt_buf].cmd + rxpos, 64);
        if (n > 0) {
          had_usb_activity = 1;
          // protocol demands that a command sequence is not a multiple of 64
          // bytes; therefore, if we have a number of bytes already that's NOT a
          // multiple of 64 (0 _is_ a multiple of 64), it means the client begun
          // writing a new command already, which isn't supported
          if (rxpos & 63) {
            printf ("PROTOCOL ERROR: the client started a new command before the previous one was executed!\n"
                    "  cmd_n=%u(complete command), new_n=%u(new command)\n",
                    rxpos, n);
            fatal_error();
          }
          rxpos += n;
          dprintf (" %c< %u(%u)\n", '0'+crt_buf, n, rxpos); // {buf}<{received}({total_received})
        }
      }
      // whenever we neither sent nor received data, run the CDC tasks
#     if NCDC > 0
      if (! had_usb_activity)
        cdc_tasks();
#     endif
    }
    // here, we're done sending the response; mark the buffer as available
    buf[done_buf].resp_n = 0; buf[done_buf].cmd_n = 0; 
    buf[done_buf].state = UNUSED;
    // turn off the TX led, but if we started receiving, turn on the RX one
    // (on stock pico, these are the same, so the sequence matters)
    led_tx(0);
    if (rxpos)
      led_rx(1);
#   ifdef DEBUG
    printf ("buf%u available\n", done_buf);
#   endif
    // update the current buffer, just in case we received something
    buf[crt_buf].cmd_n = rxpos;
    // finally, if the current buffer has a complete data set, kick it off and swap current buffers
    // NOTE: a "complete data set" means 64n+k bytes were enqueued, k!=0
    // this is the protocol we enforce, to be able to send >64bytes in one go
    if (rxpos && (rxpos & 63)) {
      led_rx(0);
      // mark the current buffer as READY_TO_EXECUTE
      buf[crt_buf].state = READY_TO_EXECUTE;
#     ifdef DEBUG
      printf ("buf%u ready to execute (cmd_sz=%u)\n", crt_buf, buf[crt_buf].cmd_n);
#     endif
    }
  }

  // if the current buffer is ready to execute, kick it off first
  // (without attempting to do any USB tasks)
try_execute:
  if (buf[crt_buf].state == READY_TO_EXECUTE) {
    // if the other buffer is executing, we have nothing to do for now
    if (buf[crt_buf^1].state >= EXECUTING) // EXECUTING or SENDING
      return;
    // it's not? great, kick it off
    buf[crt_buf].state = EXECUTING;
#   ifdef DEBUG
    printf ("buf%u executing\n", crt_buf);
#   endif
    multicore_fifo_push_blocking(crt_buf);
    // and switch the current buffer
    crt_buf ^= 1;
    return;
  }

  // here, we've no more data to send - try receiving, and failing that, run the CDC tasks
  // run the TinyUSB Device task
  tud_task();
  // if we haven't got any data, run the CDC tasks and we're done
  if (! tud_vendor_available()) {
#   if NCDC > 0
    cdc_tasks();
#   endif
    return;
  }
  // here: we might have some data
  // sanitize receive buffer state
  if (buf[crt_buf].state != RECEIVING) {
    printf ("INTERNAL ERROR: got data for buffer %u, but that buffer isn't RECEIVING !!!\n"
            "  crt_buf=%u buf[0]={state=%u cmd_n=%u resp_n=%u} buf[1]={state=%u cmd_n=%u resp_n=%u}\n",
            crt_buf,
            buf[0].state, buf[0].cmd_n, buf[0].resp_n,
            buf[1].state, buf[1].cmd_n, buf[1].resp_n);
    fatal_error();
  }
  // if there's actually no data, run the CDC tasks and we're done
  int n = tud_vendor_read(buf[crt_buf].cmd + rxpos, 64);
  if (n < 1) {
#   if NCDC > 0
    cdc_tasks();
#   endif
    return;
  }
  // here: we definitely have data; keep receiving until we have a complete commands batch
  // first, sanitize state
  if (rxpos & 63) {
    printf ("PROTOCOL ERROR: the client started a new command before the previous one was executed!\n"
            "  cmd_n=%u(complete command), new_n=%u(new command)\n",
            rxpos, n);
    fatal_error;
  }

  // if we got an entire command, mark it as ready to execute
  rxpos += n;
  dprintf (" %c< %u(%u)\n", '0' + crt_buf, n, rxpos); // <{received}({total_received})
  buf[crt_buf].cmd_n = rxpos;
  if (rxpos && (rxpos & 63)) {
    led_rx(0);
    // mark the current buffer as READY_TO_EXECUTE
    buf[crt_buf].state = READY_TO_EXECUTE;
#   ifdef DEBUG
    printf ("buf%u ready to execute (cmd_sz=%u)\n", crt_buf, buf[crt_buf].cmd_n);
#   endif
    goto try_execute;
  }

  // if we don't have a complete command set, keep receiving
  if (! rxpos)
    led_rx(1);
  while (1) {
    int had_usb_activity = 0;
    // run the TinyUSB Device task
    tud_task();
    if (tud_vendor_available()) {
      // if there's actually no data, run the CDC tasks and we're done
      int n = tud_vendor_read(buf[crt_buf].cmd + rxpos, 64);
      if (n > 0) {
        had_usb_activity = 1;
        rxpos += n;
        dprintf (" %c< %u(%u)\n", '0' + crt_buf, n, rxpos); // <{received}({total_received})
        // complete command?
        if (rxpos && (rxpos & 63)) {
          led_rx(0);
          // mark the current buffer as READY_TO_EXECUTE
          buf[crt_buf].state = READY_TO_EXECUTE;
          buf[crt_buf].cmd_n = rxpos;
#         ifdef DEBUG
          printf ("buf%u ready to execute (cmd_sz=%u)\n", crt_buf, buf[crt_buf].cmd_n);
#         endif
          goto try_execute;
        }
      }
    }
    // if we got nothing, run the CDC tasks
#   if NCDC > 0
    if (! had_usb_activity)
      cdc_tasks();
#   endif
  }

  // we should NEVER get here!
  printf ("INTERNAL ERROR: got to the end of the infinite loop :>\n");
  fatal_error();
}

// Core 1 runs the JTAG tasks:
// - receive ID of buffer to execute 
// - execute and update buffer
// - send ID of buffer that executed
void core1_entry() {
  djtag_init();
  while (1) {
    unsigned i = multicore_fifo_pop_blocking();
    assert (buf[i].state == EXECUTING);
    buf[i].resp_n = cmd_execute(&jtag, i, buf[i].cmd, buf[i].cmd_n, buf[i].resp);
    multicore_fifo_push_blocking(i);
  }
}

// this is to work around the fact that tinyUSB does not handle setup request automatically
// Hence this boiler plate code
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request)
{
  if (stage != CONTROL_STAGE_SETUP)
    return true;
  return false;
}

int main()
{
  board_init();
  usb_serial_init();
  tusb_init();
  // enable stdio over uart1 on pins tx=8, rx=9
  stdio_uart_init_full(uart1, 115200, 8, -1);

  snprintf (whoami, sizeof(whoami), 
            "DirtyJTAG2-pico %s %s%s %s [custom]\n"
            "    with: A5clk, driven_RST#, pincfg_setall", 
            git_Branch, git_Describe, git_AnyUncommittedChanges?"(dirty)":"", git_Remote);
  printf ("\n%s\n", whoami);

  led_init( LED_INVERTED, PIN_LED_TX, PIN_LED_RX, PIN_LED_ERROR );
# if USB_CDC_UART_BRIDGE
  cdc_uart_init( PIN_UART0, PIN_UART0_RX, PIN_UART0_TX );
# if PIN_UART_INTF_COUNT == 2
  cdc_uart_init( PIN_UART1, PIN_UART1_RX, PIN_UART1_TX );
# endif
# endif

  multicore_launch_core1(core1_entry);
  while (1)
    jtag_main_task();
}

// delme
void cdc_spi_task() {}

void cdc_tasks()
{
# ifdef USB_CDC_UART_BRIDGE
  cdc_uart_task();
# endif
# ifdef USB_CDC_SPI_BRIDGE
  cdc_spi_task();
# endif
}

