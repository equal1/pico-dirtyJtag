#include <pico/stdlib.h>
#include <hardware/dma.h>
#include <hardware/irq.h>
#include <stdio.h>
#include <string.h>
#include "config.h"
#include "utils.h"

#define UART_RX_BUFFER_SIZE 1024

static struct {
  char rx_buf[UART_RX_BUFFER_SIZE];
  unsigned read_pos;
  struct {
    unsigned rx, read;
  } total;
  int rx_dma_channel;
} uart = {
  .read_pos = UART_RX_BUFFER_SIZE,
  .total = { 0, 0 },
  .rx_dma_channel = -1
};
static void onUartDmaIrq();

int djtag_uart_init()
{
  // configure the GPIOs
  gpio_set_function(PIN_A5_UART_TX, GPIO_FUNC_UART);
  gpio_set_function(PIN_A5_UART_RX, GPIO_FUNC_UART);
  gpio_set_pulls(PIN_A5_UART_TX, 1, 0);
  gpio_set_pulls(PIN_A5_UART_RX, 1, 0);
  // init the UART
  djtag_uart_set_baud(BAUD_A5_UART);
  //-------------------------------------------------------------------------
  // setup the RX DMA
  uint dma_chan = dma_claim_unused_channel(true);
  // Tell the DMA to raise IRQ line 1 when the channel finishes a block
  dma_channel_set_irq1_enabled(dma_chan, true);
  // Configure the processor to run onUartDmaIrq() when DMA IRQ 1 is asserted
  irq_add_shared_handler(DMA_IRQ_1, onUartDmaIrq, PICO_SHARED_IRQ_HANDLER_DEFAULT_ORDER_PRIORITY);
  irq_set_enabled(DMA_IRQ_1, true);
  // enable DMA RX
  hw_write_masked(&uart_get_hw(UART_A5)->dmacr, 1 << UART_UARTDMACR_RXDMAE_LSB, UART_UARTDMACR_RXDMAE_BITS);
  dma_channel_config c = dma_channel_get_default_config(dma_chan);
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
  channel_config_set_read_increment(&c, false);
  channel_config_set_write_increment(&c, true);
  channel_config_set_dreq(&c, uart_get_dreq(UART_A5, false));
  hw_clear_bits(&uart_get_hw(UART_A5)->rsr, UART_UARTRSR_BITS); // clear
  dma_channel_configure(
    dma_chan,
    &c,
    uart.rx_buf,                // Write Address
    &uart_get_hw(UART_A5)->dr,  // Read Address
    UART_RX_BUFFER_SIZE,        // transfer count
    true                        // start
  );
  uart.read_pos = 0;
  uart.rx_dma_channel = dma_chan;
  return 0;
}

void djtag_uart_set_baud(unsigned baud)
{
  // de-initialize the UART before reconfiguring
  uart_deinit(UART_A5);
  // re-initialize with the new baud rate
  uart_init(UART_A5, baud);
  uart_set_hw_flow(UART_A5, false, false);
  uart_set_format(UART_A5, 8, 1, UART_PARITY_NONE);
  uart_set_fifo_enabled(UART_A5, true);
}

// shared between rx_dma_channel and tx_dma_channel
void onUartDmaIrq()
{
  volatile uint32_t ints = dma_hw->ints1;
  // rewind pointer if the DMA completed
  if (dma_channel_get_irq1_status(uart.rx_dma_channel))
    dma_channel_set_write_addr(uart.rx_dma_channel, uart.rx_buf, true);
  dma_hw->ints1 = ints;
  uart.total.rx += UART_RX_BUFFER_SIZE;
}

int uart_read(char *d)
{
  volatile char *wa = (volatile char*)(dma_channel_hw_addr(uart.rx_dma_channel)->write_addr);
  unsigned wpos = wa - uart.rx_buf;
  // this can happen if we get really bad timing on the IRQ
  if (wpos == UART_RX_BUFFER_SIZE)
    wpos = 0;
  // extract all the new data
  unsigned rpos = uart.read_pos;
  unsigned extracted = 0;
  int n = wpos - rpos;
  while (n) {
    // if we had a wrap-around, read until the end of the buffer first
    if (n < 0)
      n = UART_RX_BUFFER_SIZE - rpos;
    // copy the data
    memcpy(d, uart.rx_buf + rpos, n);
    d += n;
    rpos += n; extracted += n;
    if (rpos >= UART_RX_BUFFER_SIZE)
      rpos -= UART_RX_BUFFER_SIZE;
    // update the left characters count, just in case we had a wraparound
    n = wpos - rpos;
  }
  // update the read pointer
  if (extracted) {
    uart.read_pos = rpos;
    uart.total.read += extracted;
  }
  // notify of overflows
  unsigned total_got = uart.total.rx + wpos;
  static int n_overflows = 0;
  if (total_got - (uart.total.read + n_overflows * UART_RX_BUFFER_SIZE) > UART_RX_BUFFER_SIZE) {
    ++n_overflows;
    puts("!!! UART overflow !!!");
  }
  return extracted;
}
