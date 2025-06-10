#include <hardware/clocks.h>
#include <hardware/dma.h>
#include "pio_jtag.h"
#include "config.h"
#include "jtag.pio.h"
#include "a5clk.pio.h"

// we use multicore, so all the JTAG tasks run on core 1 _while_ core 0
// processes I/O

static int tx_dma_chan = -1;
static int rx_dma_chan;
static dma_channel_config tx_c;
static dma_channel_config rx_c;
static bool last_tdo = false;

void jtag_dma_init()
{
  if (tx_dma_chan == -1) {
    // Configure a channel to write a buffer to PIO0
    // SM0's TX FIFO, paced by the data request signal from that peripheral.
    tx_dma_chan = dma_claim_unused_channel(true);
    tx_c = dma_channel_get_default_config(tx_dma_chan);
    channel_config_set_transfer_data_size(&tx_c, DMA_SIZE_8);
    channel_config_set_read_increment(&tx_c, true);
    channel_config_set_dreq(&tx_c, DREQ_PIO0_TX0);
    dma_channel_configure(tx_dma_chan, &tx_c,
      &pio0_hw->txf[0], // Write address (only need to set this once)
      NULL,             // Don't provide a read address yet
      0,                // Don't provide the count yet
      false);           // Don't start yet
    // Configure a channel to read a buffer from PIO0
    // SM0's RX FIFO, paced by the data request signal from that peripheral.
    rx_dma_chan = dma_claim_unused_channel(true);
    rx_c = dma_channel_get_default_config(rx_dma_chan);
    channel_config_set_transfer_data_size(&rx_c, DMA_SIZE_8);
    channel_config_set_write_increment(&rx_c, false);
    channel_config_set_read_increment(&rx_c, false);
    channel_config_set_dreq(&rx_c, DREQ_PIO0_RX0);
    dma_channel_configure(rx_dma_chan, &rx_c,
      NULL,             // Dont provide a write address yet
      &pio0_hw->rxf[0], // Read address (only need to set this once)
      0,                // Don't provide the count yet
      false);           // Don't start yet
  }
}

void __time_critical_func(pio_jtag_write_blocking)
  (const pio_jtag_inst_t *jtag, const uint8_t *bsrc, size_t len) 
{
  size_t byte_length = (len+7 >> 3);
  size_t last_shift = ((byte_length << 3) - len);
  size_t tx_remain = byte_length, rx_remain = last_shift ? byte_length : byte_length+1;
  io_rw_8 *txfifo = (io_rw_8 *) &jtag->pio->txf[jtag->sm];
  io_rw_8 *rxfifo = (io_rw_8 *) &jtag->pio->rxf[jtag->sm];
  uint8_t x; // scratch local to receive data
  //kick off the process by sending the len to the tx pipeline
  *(io_rw_32*)txfifo = len-1;
  if (byte_length > 4) {
    jtag_dma_init();
    channel_config_set_read_increment(&tx_c, true);
    channel_config_set_write_increment(&rx_c, false);
    dma_channel_set_config(rx_dma_chan, &rx_c, false);
    dma_channel_set_config(tx_dma_chan, &tx_c, false);
    dma_channel_transfer_to_buffer_now(rx_dma_chan, (void*)&x, rx_remain);
    dma_channel_transfer_from_buffer_now(tx_dma_chan, (void*)bsrc, tx_remain);
    while (dma_channel_is_busy(rx_dma_chan))
      tight_loop_contents();
    // stop the compiler hoisting a non volatile buffer access above the DMA completion.
    __compiler_memory_barrier();
  } else {
    while (tx_remain || rx_remain) {
      if (tx_remain && !pio_sm_is_tx_fifo_full(jtag->pio, jtag->sm)) {
        *txfifo = *bsrc++;
        --tx_remain;
      }
      if (rx_remain && !pio_sm_is_rx_fifo_empty(jtag->pio, jtag->sm)) {
        x = *rxfifo;
        --rx_remain;
      }
    }
  }
  last_tdo = !!(x & 1);
}

void __time_critical_func(pio_jtag_write_read_blocking)
  (const pio_jtag_inst_t *jtag, const uint8_t *bsrc, uint8_t *bdst, size_t len) 
{
  size_t byte_length = (len+7 >> 3);
  size_t last_shift = ((byte_length << 3) - len);
  size_t tx_remain = byte_length, rx_remain = last_shift ? byte_length : byte_length+1;
  uint8_t* rx_last_byte_p = &bdst[byte_length-1];
  io_rw_8 *txfifo = (io_rw_8 *) &jtag->pio->txf[jtag->sm];
  io_rw_8 *rxfifo = (io_rw_8 *) &jtag->pio->rxf[jtag->sm];
  //kick off the process by sending the len to the tx pipeline
  *(io_rw_32*)txfifo = len-1;
  if (byte_length > 4) {
    jtag_dma_init();
    channel_config_set_read_increment(&tx_c, true);
    channel_config_set_write_increment(&rx_c, true);
    dma_channel_set_config(rx_dma_chan, &rx_c, false);
    dma_channel_set_config(tx_dma_chan, &tx_c, false);
    dma_channel_transfer_to_buffer_now(rx_dma_chan, (void*)bdst, rx_remain);
    dma_channel_transfer_from_buffer_now(tx_dma_chan, (void*)bsrc, tx_remain);
    while (dma_channel_is_busy(rx_dma_chan))
      tight_loop_contents();
    // stop the compiler hoisting a non volatile buffer access above the DMA completion.
    __compiler_memory_barrier();
  } else {
    while (tx_remain || rx_remain) {
      if (tx_remain && !pio_sm_is_tx_fifo_full(jtag->pio, jtag->sm)) {
        *txfifo = *bsrc++;
        --tx_remain;
      }
      if (rx_remain && !pio_sm_is_rx_fifo_empty(jtag->pio, jtag->sm)) {
        *bdst++ = *rxfifo;
        --rx_remain;
      }
    }
  }
  last_tdo = !!(*rx_last_byte_p & 1);
  //fix the last byte
  if (last_shift) 
    *rx_last_byte_p = *rx_last_byte_p << last_shift;
}

uint8_t __time_critical_func(pio_jtag_write_tms_blocking)
  (const pio_jtag_inst_t *jtag, bool tdi, bool tms, size_t len)
{
  size_t byte_length = (len+7 >> 3);
  size_t last_shift = ((byte_length << 3) - len);
  size_t tx_remain = byte_length, rx_remain = last_shift ? byte_length : byte_length+1;
  io_rw_8 *txfifo = (io_rw_8 *) &jtag->pio->txf[jtag->sm];
  io_rw_8 *rxfifo = (io_rw_8 *) &jtag->pio->rxf[jtag->sm];
  uint8_t x; // scratch local to receive data
  uint8_t tdi_word = tdi ? 0xFF : 0x0;
  gpio_put(jtag->pin_tms, tms);
  //kick off the process by sending the len to the tx pipeline
  *(io_rw_32*)txfifo = len-1;
  if (byte_length > 4) {
    jtag_dma_init();
    channel_config_set_read_increment(&tx_c, false);
    channel_config_set_write_increment(&rx_c, false);
    dma_channel_set_config(rx_dma_chan, &rx_c, false);
    dma_channel_set_config(tx_dma_chan, &tx_c, false);
    dma_channel_transfer_to_buffer_now(rx_dma_chan, (void*)&x, rx_remain);
    dma_channel_transfer_from_buffer_now(tx_dma_chan, (void*)&tdi_word, tx_remain);
    while (dma_channel_is_busy(rx_dma_chan))
      tight_loop_contents();
    // stop the compiler hoisting a non volatile buffer access above the DMA completion.
    __compiler_memory_barrier();
  } else {
    while (tx_remain || rx_remain) {
      if (tx_remain && !pio_sm_is_tx_fifo_full(jtag->pio, jtag->sm)) {
        *txfifo = tdi_word;
        --tx_remain;
      }
      if (rx_remain && !pio_sm_is_rx_fifo_empty(jtag->pio, jtag->sm)) {
        x = *rxfifo;
        --rx_remain;
      }
    }
  }
  last_tdo = !!(x & 1);
  return last_tdo ? 0xFF : 0x00;
}

static void init_jtag_pins(uint pin_tck, uint pin_tdi, uint pin_tdo, uint pin_tms, uint pin_rst)
{
  // initialize gpios - would make them inputs
  gpio_init_mask((1u << pin_tms) | (1u << pin_rst));
  // drive 0 on TMS, 1 on SRST# initially
  gpio_clr_mask((1u << pin_tms));
  gpio_set_mask((1u << pin_rst));
  // set TMS ans SRST# as outputs
  gpio_set_dir_masked( (1u << pin_tms) | (1u << pin_rst), 0xffffffffu);
  // set TDO as input
  gpio_init(pin_tdo);
  gpio_set_dir(pin_tdo, false);
}

static void init_a5clk_pin(uint pin)
{
  gpio_clr_mask(1u << pin);
  gpio_init_mask(1u << pin);
}

void init_jtag(pio_jtag_inst_t* jtag, uint freq, uint pin_tck, uint pin_tdi, uint pin_tdo, uint pin_tms, uint pin_rst)
{
  init_jtag_pins(pin_tck, pin_tdi, pin_tdo, pin_tms, pin_rst);
  jtag->pin_tdi = pin_tdi;
  jtag->pin_tdo = pin_tdo;
  jtag->pin_tck = pin_tck;
  jtag->pin_tms = pin_tms;
  jtag->pin_rst = pin_rst;
  // the JTAG PIO program is 4 cycles (out:1, in:1, jmp:2)
  // so the JTAG clock will be sysclk (125MHz) / 4 / clkdiv
  // for a 1MHz JTAG frequency, clkdiv is 125M/1M/4 = 31.25
  unsigned clkdiv = (unsigned)(31.25 * 256);  // 1 MHz @ 125MHz clk_sys (jtag_clk @ 250KHz)
  pio_jtag_init(jtag->pio, jtag->sm, clkdiv, pin_tck, pin_tdi, pin_tdo);
  jtag_set_clk_freq(jtag, freq);
}

void init_a5clk(pio_a5clk_inst_t* a5clk, uint freq, uint pin)
{
  init_a5clk_pin(pin);
  a5clk->pin = pin;
  // the A5CLK PIO program is 2 cycles
  // so the JTAG clock will be sysclk (125MHz) / 2 / clkdiv
  // for a 1MHz JTAG frequency, clkdiv is 125M/1M/2 = 62.5
  unsigned clkdiv = (unsigned)(62.5 * 256);  // 1 MHz @ 125MHz clk_sys
  pio_a5clk_init(a5clk->pio, a5clk->sm, clkdiv, pin);
  a5clk_set_freq(a5clk, freq);
  a5clk->enabled = 0;
}

struct djtag_clk_s djtag_clocks;

void jtag_set_clk_freq(const pio_jtag_inst_t *jtag, uint freq_khz) {
  uint clk_sys_freq_khz = clock_get_hz(clk_sys) / 1000;
  unsigned wanted_pio_freq = freq_khz * 4;
  // round to nearest
  // do the 256* thing because we want a Q16.8 result
  uint32_t clkdiv = (256*clk_sys_freq_khz + wanted_pio_freq/2 - 1) / wanted_pio_freq;
  if (clkdiv < 0x200)
    clkdiv = 0x200;
  else if (clkdiv > 0xffffff)
    clkdiv = 0xffffff;
  djtag_clocks.sys_khz = clk_sys_freq_khz;
  djtag_clocks.jtag_divider = clkdiv;
  djtag_clocks.jtag_khz = 256*clk_sys_freq_khz / clkdiv / 4;
  // mess with the clocks with the state machine DISABLED
  pio_sm_set_enabled(jtag->pio, jtag->sm, false);
  pio_sm_set_clkdiv_int_frac(jtag->pio, jtag->sm, clkdiv >> 8, clkdiv & 0xff);
  pio_sm_set_enabled(jtag->pio, jtag->sm, true);
}

void a5clk_set_freq(const pio_a5clk_inst_t *a5clk, uint freq_khz) {
  uint clk_sys_freq_khz = clock_get_hz(clk_sys) / 1000;
  unsigned wanted_pio_freq = freq_khz * 2;
  // round to nearest
  // do the 256* thing because we want a Q16.8 result
  uint32_t clkdiv = (256*clk_sys_freq_khz + wanted_pio_freq/2 - 1) / wanted_pio_freq;
  if (clkdiv < 0x200)
    clkdiv = 0x200;
  else if (clkdiv > 0xffffff)
    clkdiv = 0xffffff;
  djtag_clocks.a5clk_divider = clkdiv;
  djtag_clocks.a5clk_khz = 256*clk_sys_freq_khz / clkdiv / 2;
  if (djtag_clocks.a5clk_en)
    pio_sm_set_enabled(a5clk->pio, a5clk->sm, false);
  pio_sm_set_clkdiv_int_frac(a5clk->pio, a5clk->sm, clkdiv >> 8, clkdiv & 0xff);
  if (djtag_clocks.a5clk_en)
    pio_sm_set_enabled(a5clk->pio, a5clk->sm, true);
}

void jtag_transfer(const pio_jtag_inst_t *jtag, uint32_t length, const uint8_t* in, uint8_t* out)
{
  // set tms to low
  jtag_set_tms(jtag, 0);
  if (out)
    pio_jtag_write_read_blocking(jtag, in, out, length);
  else
    pio_jtag_write_blocking(jtag, in, length);
}

uint8_t jtag_strobe(const pio_jtag_inst_t *jtag, uint32_t length, bool tms, bool tdi)
{
  if (! length)
    return jtag_get_tdo(jtag) ? 0xFF : 0x00;
  return pio_jtag_write_tms_blocking(jtag, tdi, tms, length);
}

static uint8_t toggle_bits_out_buffer[4];
static uint8_t toggle_bits_in_buffer[4];

void jtag_set_tdi(const pio_jtag_inst_t *jtag, bool value)
{
  toggle_bits_out_buffer[0] = value ? 1u << 7 : 0;
}

void jtag_set_clk(const pio_jtag_inst_t *jtag, bool value)
{
  if (value) {
    toggle_bits_in_buffer[0] = 0; 
    pio_jtag_write_read_blocking(jtag, toggle_bits_out_buffer, toggle_bits_in_buffer, 1);
  }
}

bool jtag_get_tdo(const pio_jtag_inst_t *jtag)
{
  return last_tdo;
}
