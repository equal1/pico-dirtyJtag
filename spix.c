#include <stdio.h>
#include <string.h>

#include <pico/binary_info.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>

#include "config.h"
#include "utils.h"

//#define DEBUG

#ifdef DEBUG
#define dprintf(...) (printf(__VA_ARGS__))
#else
#define dprintf(...) ((void)((__VA_ARGS__)))
#endif

int spix_spi_speed = -1, spix_spi_mode = -1, spix_spi_ss = -1;

//=[ detect/initialize the fractional PLL ]===================================

static void _assert_spix_cs(const char *fn)
{
  (void)fn;
  gpio_put(spix_spi_ss, 0);
}
#define assert_spix_cs() _assert_spix_cs(__FUNCTION__)

static void _deassert_spix_cs(const char *fn)
{
  (void)fn;
  gpio_put(spix_spi_ss, 1);
}
#define deassert_spix_cs() _deassert_spix_cs(__FUNCTION__)

void _check_spix_state(const char *fn)
{
  (void)fn;
}
#define check_spix_state() _check_spix_state(__FUNCTION__)

void spix_init()
{
  dprintf("spix_init() spi%u %uKHz, {SS#:%u SCK:%u MOSI:%u MISO:%u}\n", 
          (SPI_SPIX==spi0)?0:1, DEFAULT_FREQ_SPIX_KHZ,
          DEFAULT_PIN_SPIX_SSn, PIN_SPIX_SCK, PIN_SPIX_MOSI, PIN_SPIX_MISO);
  // load the defaults
  gpio_set_function(PIN_SPIX_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SPIX_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_SPIX_MISO, GPIO_FUNC_SPI);
  bi_decl(bi_3pins_with_func(PIN_SPIX_SCK, PIN_SPIX_MOSI, PIN_SPIX_MISO, GPIO_FUNC_SPI));
  // SS# configured by spix_config()
  bi_decl(bi_1pin_with_name(DEFAULT_PIN_SPIX_SSn, "SPIX_SS#"));
  spix_config(DEFAULT_FREQ_SPIX_KHZ, 0, DEFAULT_PIN_SPIX_SSn);
  dprintf("  actual frequency: %u.%03uKHz\n", spix_spi_speed/1000, spix_spi_speed%1000);
}

//=[ re-configure the SPIx device ]==========================================

void spix_config(unsigned freq_khz, unsigned mode, unsigned ssn)
{
  // update the slave select pin
  if (ssn != spix_spi_ss) {
    // make the old pin an input
    if (spix_spi_ss >= 0) {
      deassert_spix_cs();
      gpio_set_dir(spix_spi_ss, GPIO_IN);
    }
    // update the SS# pin
    spix_spi_ss = ssn;
    gpio_init(spix_spi_ss);
    gpio_put(spix_spi_ss, 1); // initially de-selected
    gpio_set_dir(spix_spi_ss, GPIO_OUT);
    deassert_spix_cs();
  }
  // updated the SPI rate
  if ((freq_khz * 1000) != spix_spi_speed) {
    spi_set_baudrate(SPI_SPIX, freq_khz * 1000);
    spix_spi_speed = spi_get_baudrate(SPI_SPIX);
  }
  // updated the SPI mode
  // SPI mode is split into polarity (mode[1]) and phase (mode[0])
  if ((mode & 3) != spix_spi_mode) {
    spi_set_format(SPI_SPIX, 8, (mode >> 1) & 1, (mode >> 0) & 1, 0);
    spix_spi_mode = mode & 3;
  }
}

//=[ random data transfer ]===================================================

int spix_wrrd(const uint8_t *out, uint8_t *in, unsigned n)
{
  // re-configure SPI rate if we most recently used another device
  if (spi_get_baudrate(SPI_SPIX) != spix_spi_speed)
    spi_set_baudrate(SPI_SPIX, spix_spi_speed);
  assert_spix_cs();
  spi_write_read_blocking(SPI_SPIX, out, in, n);
  deassert_spix_cs();
  return n;
}
