#include <stdio.h>
#include <string.h>

#include <pico/binary_info.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>

#include "config.h"
#include "adf4368.h"
#include "utils.h"

//#define DEBUG

int fpll_spi_speed, fpll_chip_version;

//=[ detect/initialize the fractional PLL ]===================================

static int fpll_probe();

static void _assert_fpll_cs(const char *fn)
{
  (void)fn;
  gpio_put(PIN_FPLL_SSn, 0);
}
#define assert_fpll_cs() _assert_fpll_cs(__FUNCTION__)

static void _deassert_fpll_cs(const char *fn)
{
  (void)fn;
  gpio_put(PIN_FPLL_SSn, 1);
}
#define deassert_fpll_cs() _deassert_fpll_cs(__FUNCTION__)

void _check_fpll_state(const char *fn)
{
  (void)fn;
}
#define check_fpll_state() _check_fpll_state(__FUNCTION__)

// sets fpll_spi_speed to 0, if device not present
void fpll_init()
{
  // configure CS#
  gpio_init(PIN_FPLL_SSn);
  gpio_put(PIN_FPLL_SSn, 1); // initially de-selected
  gpio_set_dir(PIN_FPLL_SSn, GPIO_OUT);
  deassert_fpll_cs();
  //bi_decl(bi_1pin_with_name(PIN_FPLL_SSn, "ADF4368_SS#")); // technically this is handled by a5pins (as "a5 ss#")
  // configure SPI itself - mode 0
  spi_init(SPI_FPLL, FREQ_FPLL_KHZ * 1000);
  spi_set_format(SPI_FPLL, 8, 0, 0, 0);
  // configure the SPI pins
  gpio_set_function(PIN_FPLL_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_FPLL_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_FPLL_MISO, GPIO_FUNC_SPI);
  //bi_decl(bi_3pins_with_func(PIN_FPLL_MISO, PIN_FPLL_MOSI, PIN_FPLL_SCK, GPIO_FUNC_SPI)); // technically this is handled by a5pins (as "a5 spi.*")
  fpll_spi_speed = spi_get_baudrate(SPI_FPLL);
  fpll_chip_version = -1;

  // perform detection
  if ((fpll_chip_version = fpll_probe()) < 0) {
    //printf("FPLL does NOT work at %u.%uMHz!\n", (fpll_spi_speed+500)/1000000, ((fpll_spi_speed+500)%1000000)/1000);
    fpll_spi_speed = 0;
    return;
  }
}

//-[ fpll_probe() ]-----------------------------------------------------------

int fpll_probe()
{
  // there's a very good chance there's no ADF4368 connected, but probe for it anyway
  // (if it is, it's connected to the A5 spi/CS#)
  // try to initialize the chip
  do_fpll_set(0, FPLL_R00_SOFT_RESET); // soft reset
  do_fpll_set(0, FPLL_R00_MSB_FIRST|FPLL_R00_ADDR_AUTODEC|FPLL_R00_MODE_SPI); // enable SPI mode
  // check that the initial write succeeded
  if (do_fpll_get(0x00) != (FPLL_R00_MSB_FIRST|FPLL_R00_ADDR_AUTODEC|FPLL_R00_MODE_SPI))
    return -1;
  // check versions
  if ((do_fpll_get(0x03) != FPLL_R03_CHIP_TYPE_VAL) || // chip_type: 0x06
      (do_fpll_get(0x05) != FPLL_R05_PROD_ID_HI_VAL) || (do_fpll_get(0x04) != FPLL_R04_PROD_ID_LO_VAL) || // prod_id: 0x0007
      (do_fpll_get(0x06) != (FPLL_R06_PROD_GRADE_VAL|FPLL_R06_DEV_REVISION_VAL)) || // r06: 0x00
      (do_fpll_get(0x0B) != FPLL_R0B_SPI_REVISION_VAL) || // spi_version: 0x01
      (do_fpll_get(0x0D) != FPLL_R0D_VEN_ID_HI_VAL) || (do_fpll_get(0x0C) != FPLL_R0C_VEN_ID_LO_VAL)) // ven_id: 0x0456
  {
    return -2;
  }
  // let's assume the chip is there, and return the version
  return do_fpll_get(0x64);
}

//=[ re-configure the ADF4368 SPI rate ]======================================

// reconfigure the SPI frequency
int do_fpll_config(unsigned freq_khz)
{
  if (fpll_chip_version < 0)
    return -1;
  // set the new frequency
  int crt_spi_freq = fpll_spi_speed;
  spi_set_baudrate(SPI_FPLL, freq_khz * 1000);
  // make sure the chip's still there
  int t = fpll_probe();
  if (t == fpll_chip_version) {
    fpll_spi_speed = spi_get_baudrate(SPI_FPLL);
    // re-generate the WHOAMI info
    void whoami_init();
    whoami_init();
    return (fpll_spi_speed + 500) / 1000;
  }
  // no joy - return to previous settings
  spi_set_baudrate(SPI_FPLL, crt_spi_freq);
  return -1;
}

//=[ set a single ADF4368 register ]==========================================

int do_fpll_set(unsigned addr, uint8_t value)
{
  if (fpll_chip_version < 0)
    return -1;
  if (addr > FPLL_N_REGS)
    return -2;
  // re-configure SPI rate if we most recently used another device
  if (spi_get_baudrate(SPI_FPLL) != fpll_spi_speed)
    spi_set_baudrate(SPI_FPLL, fpll_spi_speed);
  // do the transfer
  assert_fpll_cs();
  uint8_t cmd[3];
  cmd[0] = (addr >> 8) & 0x7f;
  cmd[1] = addr & 0xff;
  cmd[2] = value;
  spi_write_blocking(SPI_FPLL, cmd, 3);
  deassert_fpll_cs();
  return value;
}

//=[ get a single ADF4368 register ]==========================================

int do_fpll_get(unsigned addr)
{
  if (fpll_chip_version < 0)
    return -1;
  if (addr > FPLL_N_REGS)
    return -2;
  // re-configure SPI rate if we most recently used another device
  if (spi_get_baudrate(SPI_FPLL) != fpll_spi_speed)
    spi_set_baudrate(SPI_FPLL, fpll_spi_speed);
  assert_fpll_cs();
  uint8_t data[3];
  data[0] = 0x80 | ((addr >> 8) & 0x7f);
  data[1] = addr & 0xff;
  data[2] = 0xFF;
  spi_write_read_blocking(SPI_FPLL, data, data, 3);
  deassert_fpll_cs();
  return data[2];
}

//=[ random data transfer ]===================================================

int do_fpll_wrrd(const uint8_t *out, uint8_t *in, unsigned n)
{
  if (fpll_chip_version < 0)
    return -1;
  // re-configure SPI rate if we most recently used another device
  if (spi_get_baudrate(SPI_FPLL) != fpll_spi_speed)
    spi_set_baudrate(SPI_FPLL, fpll_spi_speed);
  assert_fpll_cs();
  spi_write_read_blocking(SPI_FPLL, out, in, n);
  deassert_fpll_cs();
  return n;
}
