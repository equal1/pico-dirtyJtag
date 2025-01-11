#include <stdint.h>
#include <stdio.h>

#include <pico/binary_info.h>
#include <hardware/spi.h>
#include <hardware/gpio.h>

#include "config.h"
#include "a5pins.h"

static int32_t iox_readcmd_all(unsigned cmd) {
  if (iox_spi_speed < 1000)
    return -1;
  const uint8_t
    cmd_l[2] = { IOX_DO_RD | ((cmd | 0) << 1), 0xff},
    cmd_h[2] = { IOX_DO_RD | ((cmd | 1) << 1), 0xff};
  uint8_t resp_l[2], resp_h[2];
  // read in the low, then high, values
  int a, b;
  if (spi_get_baudrate(SPI_IOX) != iox_spi_speed)
    spi_set_baudrate(SPI_IOX, iox_spi_speed);
  gpio_put(PIN_IOX_SSn, 0);
  a = spi_write_read_blocking(SPI_IOX, cmd_l, resp_l, 2);
  gpio_put(PIN_IOX_SSn, 1);
  gpio_put(PIN_IOX_SSn, 0);
  b = spi_write_read_blocking(SPI_IOX, cmd_h, resp_h, 2);
  gpio_put(PIN_IOX_SSn, 1);
  if ((a != 2) || (b != 2))
    return -1;
  uint32_t result = (((uint32_t)resp_h[1]) << 8) | resp_l[1];
  return result;
}

static int32_t iox_writecmd_all(unsigned cmd, uint32_t all) {
  if (iox_spi_speed < 1000)
    return -1;
  uint8_t
    cmd_l[2] = { IOX_DO_WR | ((cmd | 0) << 1), all & 0xFF },
    cmd_h[2] = { IOX_DO_WR | ((cmd | 1) << 1), (all >> 8) & 0xFF };
  // write the low, then high, values
  int a, b;
  if (spi_get_baudrate(SPI_IOX) != iox_spi_speed)
    spi_set_baudrate(SPI_IOX, iox_spi_speed);
  gpio_put(PIN_IOX_SSn, 0);
  a = spi_write_blocking(SPI_IOX, cmd_l, 2);
  gpio_put(PIN_IOX_SSn, 1);
  gpio_put(PIN_IOX_SSn, 0);
  b = spi_write_blocking(SPI_IOX, cmd_h, 2);
  gpio_put(PIN_IOX_SSn, 1);
  if ((a != 2) || (b != 2))
    return -1;
  return 0;
}

// get all the pins
// (for output pins, we get the intended output value, not the actual value)
int32_t iox_get_all() {
  return iox_readcmd_all(IOX_CMD_GET);
}

// set all the pins
// (no effect for input pins)
static int last_output_values = -1;
int iox_set_all(uint32_t all) {
  last_output_values = all;
  return iox_writecmd_all(IOX_CMD_SET, all);
}

// configure all the pins (0=out, 1=in)
static int last_config_values = -1;
int iox_config_all(uint32_t all) {
  last_config_values = all;
  return iox_writecmd_all(IOX_CMD_CFG, all);
}

// configure pullups on all the pins (0=off, 1=pullup)
static int last_pullup_values = -1;
int iox_pullup_all(uint32_t all) {
  last_pullup_values = all;
  return iox_writecmd_all(IOX_CMD_PULLUP, all);
}

// we know last_{output,config,pullup}_values exist, since we configured them
// UNLESS there's no IOX
int iox_get_pin_pullup(int pin) {
  if (iox_spi_speed < 1000)
    return -1;
  return (last_pullup_values >> (pin & 0xf)) & 1;
}
int iox_set_pin_pullup(int pin, int en) {
  if (iox_spi_speed < 1000)
    return -1;
  int prev_val = last_pullup_values;
  const int mask = 1 << (pin & 0xf);
  if (en)
    iox_pullup_all(last_pullup_values | mask);
  else
    iox_pullup_all(last_pullup_values & ~mask);
  return prev_val & mask;
}

int iox_get_pin_direction(int pin) {
  if (iox_spi_speed < 1000)
    return -1;
  return (last_config_values & (1 << (pin & 0xf))) ? GPIO_IN : GPIO_OUT;
}
int iox_set_pin_direction(int pin, int dir) {
  if (iox_spi_speed < 1000)
    return -1;
  int prev_val = last_config_values;
  const int mask = 1 << (pin & 0xf);
  if (dir == GPIO_IN)
    iox_config_all(last_config_values | mask);
  else if (dir == GPIO_OUT)
    iox_config_all(last_config_values & ~mask);
  return (prev_val & mask) ? GPIO_IN : GPIO_OUT;
}

int iox_get_pin_output(int pin) {
  if (iox_spi_speed < 1000)
    return -1;
  return (last_output_values >> (pin & 0xf)) & 1;
}
int iox_set_pin_output(int pin, int high) {
  if (iox_spi_speed < 1000)
    return -1;
  int prev_val = last_output_values;
  const int mask = 1 << (pin & 0xf);
  if (high)
    iox_set_all(last_output_values | mask);
  else
    iox_set_all(last_output_values & ~mask);
  return (prev_val >> (pin & 0xf)) & 1;
}

int iox_get_pin_value(int pin) {
  if (iox_spi_speed < 1000)
    return -1;
  return (iox_get_all() >> (pin & 0xf)) & 1;
}

// check if IOX SPI works
// the way we do this is, we confirm we can write correctly
// to a register in the IOX (RDINV chosen)
int iox_check() {
  iox_writecmd_all(IOX_REG_PIR, 0x5A5A);
  unsigned rdinv = iox_readcmd_all(IOX_REG_PIR);
  if (rdinv != 0x5A5A) {
    iox_spi_speed = 0;
    return -1;
  }
  iox_writecmd_all(IOX_REG_PIR, 0xA5A5);
  rdinv = iox_readcmd_all(IOX_REG_PIR);
  if (rdinv != 0xA5A5) {
    iox_spi_speed = 0;
    return -1;
  }
  iox_writecmd_all(IOX_REG_PIR, 0);
  return 0;
}

// configure all A5 pins on the Pico itself
int a5_pico_pins_init() {
  // initialize UART pins
  gpio_set_function(PIN_A5_UART_RX, GPIO_FUNC_UART);
  gpio_set_function(PIN_A5_UART_TX, GPIO_FUNC_UART);
  bi_decl(bi_2pins_with_func(PIN_A5_UART_RX, PIN_A5_UART_TX, GPIO_FUNC_UART));
  // initialize SPI pins
  gpio_set_function(PIN_A5_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_A5_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_A5_MISO, GPIO_FUNC_SPI);
  bi_decl(bi_3pins_with_func(PIN_A5_SCK, PIN_A5_MOSI, PIN_A5_MISO, GPIO_FUNC_SPI));
  gpio_init(PIN_A5_SSn);
  gpio_put(PIN_A5_SSn, 1); // initially de-selected
  gpio_set_dir(PIN_A5_SSn, GPIO_OUT);
  bi_decl(bi_1pin_with_name(PIN_A5_SSn, "A5_SS#"));
  // scan pins (weak pulldown on SCANOUT)
  gpio_init(PIN_A5_SCANEN);
  gpio_put(PIN_A5_SCANEN, 0); // initially de-selected
  gpio_set_dir(PIN_A5_SCANEN, GPIO_OUT);
  bi_decl(bi_1pin_with_name(PIN_A5_SCANEN, "A5_SCANEN"));
  gpio_init(PIN_A5_SCANIN);
  gpio_put(PIN_A5_SCANIN, 0); // initially de-selected
  gpio_set_dir(PIN_A5_SCANIN, GPIO_OUT);
  bi_decl(bi_1pin_with_name(PIN_A5_SCANIN, "A5_SCANIN"));
  gpio_init(PIN_A5_SCANOUT);
  gpio_set_pulls(PIN_A5_SCANOUT, false, true);
  gpio_set_dir(PIN_A5_SCANOUT, GPIO_IN);
  bi_decl(bi_1pin_with_name(PIN_A5_SCANOUT, "A5_SCANOUT"));
  // GPIOs on the Pico - make these inputs until we confirm otherwise, and enable pull-downs
  gpio_init(PIN_A5_GPIO6);
  gpio_set_pulls(PIN_A5_SCANOUT, false, true);
  gpio_set_dir(PIN_A5_SCANIN, GPIO_IN);
  bi_decl(bi_1pin_with_name(PIN_A5_GPIO6, "A5_GPIO6"));
  gpio_init(PIN_A5_GPIO10);
  gpio_set_pulls(PIN_A5_SCANOUT, false, true);
  gpio_set_dir(PIN_A5_GPIO10, GPIO_IN);
  bi_decl(bi_1pin_with_name(PIN_A5_GPIO10, "A5_GPIO10"));
  return 0;
}

// configure all A5 pins on the IOX
int a5_iox_pins_init() {
  if (! iox_spi_speed)
    return -1;
  // set CLKSRC, TILESEL*, TESTEN to outputs
  // set the rest to inputs for now
  // have CLKSRC and TESTEN output 0, TILESEL* outputting 2'b01 by default
  last_output_values = (1 << (PIN_A5_TILESEL0&0xf));
  last_config_values = ~((1 << (PIN_A5_CLKSRC&0xf))   | (1 << (PIN_A5_TESTEN&0xf)) |
                         (1 << (PIN_A5_TILESEL1&0xf)) | (1 << (PIN_A5_TILESEL0&0xf)));
  last_pullup_values = last_config_values;
  // config the output values
  iox_writecmd_all(IOX_CMD_SET, last_output_values);
  // enable pull-ups on all the input pins
  iox_writecmd_all(IOX_CMD_PULLUP, last_pullup_values);
  // configure the pin directions
  iox_writecmd_all(IOX_CMD_CFG, last_config_values);
  return 0;
}

void iox_debug() {
  if (! iox_spi_speed) {
    puts("IOX not available!");
    return;
  }
  unsigned get = iox_readcmd_all(IOX_CMD_GET);
  unsigned set = iox_readcmd_all(IOX_CMD_SET);
  unsigned cfg = iox_readcmd_all(IOX_CMD_CFG);
  unsigned pullup = iox_readcmd_all(IOX_CMD_PULLUP);
  unsigned rdinv = iox_readcmd_all(IOX_REG_PIR);
  const unsigned tilesel_mask = (1<<(PIN_A5_TILESEL1&0xF)) | (1<<(PIN_A5_TILESEL0&0xF));
  printf("IOX: GSR=%04X OCR=%04X GCR=%04X PIR=%04X PUR=%04X\n"
         "     TILESEL=", get, set, cfg, rdinv, pullup);
  if (cfg & tilesel_mask)
    printf("floating");
  else
    printf("%u", (set & tilesel_mask) >> (PIN_A5_TILESEL0&0xF));
  printf (" CLKSRC=");
  if (cfg & (1<<(PIN_A5_CLKSRC&0x1f)))
    printf("floating\n");
  else
    printf("%u\n", (set & (1<<(PIN_A5_CLKSRC&0x1f))) ? 1 : 0);
}

//=[ pincfg ops ]============================================================

// GPIOs on the PICO itself
static const uint32_t pico_pins = 
  // A5 RESET# and SYSCLK inputs
  (1 << PIN_RST) | (1 << PIN_A5_CLK) |
  // A5 JTAG signals
  (1 << PIN_TMS) | (1 << PIN_TCK) | (1 << PIN_TDO) | (1 << PIN_TDI) |
  // A5 UART pins
  (1 << PIN_A5_UART_TX) | (1 << PIN_A5_UART_RX) |
  // A5 SPI pins
  (1 << PIN_A5_SSn) | (1 << PIN_A5_SCK) | (1 << PIN_A5_MOSI) | (1 << PIN_A5_MISO) |
  // A5 SCAN pins
  (1 << PIN_A5_SCANIN) | (1 << PIN_A5_SCANOUT) | (1 << PIN_A5_SCANEN) |
  // directly-connected A5 GPIOs
  (1 << PIN_A5_GPIO6) | (1 << PIN_A5_GPIO10);

// GPIOs on the IOX
static const uint32_t iox_pins = 
  // clock source select (when set, uses the SMA clock instead of the
  // Pico-provided one; needs a few blocks enabled first)
  (1 << (PIN_A5_CLKSRC & ~IOX_PIN_BASE))   |
  // main tile selection; chooses to which tile is the peripherals
  // bus connected; GPIO, SPI, UART, etc only have one instance, and
  // they're only available on this bus
  (1 << (PIN_A5_TILESEL0 & ~IOX_PIN_BASE)) | (1 << (PIN_A5_TILESEL1 & ~IOX_PIN_BASE)) |
  // test enable
  (1 << (PIN_A5_TESTEN & ~IOX_PIN_BASE))   |
  // other A5 GPIOs
  (1 << (PIN_A5_GPIO7  & ~IOX_PIN_BASE))   | (1 << (PIN_A5_GPIO8  & ~IOX_PIN_BASE))   |
  (1 << (PIN_A5_GPIO9  & ~IOX_PIN_BASE))   | (1 << (PIN_A5_GPIO11 & ~IOX_PIN_BASE))   |
  (1 << (PIN_A5_GPIO12 & ~IOX_PIN_BASE))   | (1 << (PIN_A5_GPIO13 & ~IOX_PIN_BASE))   |
  (1 << (PIN_A5_GPIO14 & ~IOX_PIN_BASE))   | (1 << (PIN_A5_GPIO15 & ~IOX_PIN_BASE))   |
  (1 << (PIN_A5_GPIO16 & ~IOX_PIN_BASE));

static const char *pico_signames[32] = {
  // on Pico
  [PIN_RST] =        "RST#",
  [PIN_A5_CLK] =     "CLK",
  [PIN_TMS] =        "TMS",
  [PIN_TCK] =        "TCK",
  [PIN_TDO] =        "TDO",
  [PIN_TDI] =        "TDI",
  [PIN_A5_UART_TX] = "TX",
  [PIN_A5_UART_RX] = "RX",
  [PIN_A5_SSn] =     "SS#",
  [PIN_A5_SCK] =     "SCK",
  [PIN_A5_MOSI] =    "MOSI",
  [PIN_A5_MISO] =    "MISO",
  [PIN_A5_GPIO6] =   "GPIO6",
  [PIN_A5_GPIO10] =  "GPIO10",
  [PIN_A5_SCANIN] =  "SCANIN",
  [PIN_A5_SCANEN] =  "SCANEN",
  [PIN_A5_SCANOUT] = "SCANOUT",
}, *iox_signames[16] = {
  //on IOX
  [PIN_A5_CLKSRC & ~IOX_PIN_BASE] =   "CLKSRC",
  [PIN_A5_TILESEL0 & ~IOX_PIN_BASE] = "TILESEL0",
  [PIN_A5_TILESEL1 & ~IOX_PIN_BASE] = "TILESEL1",
  [PIN_A5_TESTEN & ~IOX_PIN_BASE] =   "TESTEN",
  [PIN_A5_GPIO7  & ~IOX_PIN_BASE]  =  "GPIO7",
  [PIN_A5_GPIO8  & ~IOX_PIN_BASE]  =  "GPIO8",
  [PIN_A5_GPIO9  & ~IOX_PIN_BASE]  =  "GPIO9",
  [PIN_A5_GPIO11 & ~IOX_PIN_BASE] =   "GPIO11",
  [PIN_A5_GPIO12 & ~IOX_PIN_BASE] =   "GPIO12",
  [PIN_A5_GPIO13 & ~IOX_PIN_BASE] =   "GPIO13",
  [PIN_A5_GPIO14 & ~IOX_PIN_BASE] =   "GPIO14",
  [PIN_A5_GPIO15 & ~IOX_PIN_BASE] =   "GPIO15",
  [PIN_A5_GPIO16 & ~IOX_PIN_BASE] =   "GPIO16",
};

//=[ pincfg ops ]============================================================

int pincfg_set_all(int cfg)
{
  // foreach pin on PICO
  for (int pin = 0; pin < 32; ++pin) {
    if (pico_pins & (1 << pin)) {
      // apply slew rate
      gpio_set_slew_rate(pin, (cfg & PINCFG_SLEW_RATE_MASK) >> PINCFG_SLEW_RATE_POS);
      // apply drive strength
      gpio_set_drive_strength(pin, (cfg & PINCFG_DRIVE_STRENGTH_MASK) >> PINCFG_DRIVE_STRENGTH_POS);
      // apply pulls
      gpio_set_pulls(pin, cfg & PINCFG_PULL_HIGH, cfg & PINCFG_PULL_LOW);
      // apply hysteresis
      gpio_set_input_hysteresis_enabled(pin, cfg & PINCFG_HYSTERESIS_ON);
    }
  }
  return 0;
}

int pincfg_set(int pin, int cfg, int on_iox)
{
  // PICO pins
  if(! on_iox) {
    // apply slew rate
    gpio_set_slew_rate(pin, (cfg & PINCFG_SLEW_RATE_MASK) >> PINCFG_SLEW_RATE_POS);
    // apply drive strength
    gpio_set_drive_strength(pin, (cfg & PINCFG_DRIVE_STRENGTH_MASK) >> PINCFG_DRIVE_STRENGTH_POS);
    // apply pulls
    gpio_set_pulls(pin, cfg & PINCFG_PULL_HIGH, cfg & PINCFG_PULL_LOW);
    // apply hysteresis
    gpio_set_input_hysteresis_enabled(pin, cfg & PINCFG_HYSTERESIS_ON);
    // apply output value
    // (this doesn't configure the pin as an output, so it only affects outputs)
    gpio_put(pin, cfg & PINCFG_VALUE_HIGH);
  }
  // no such settings on the IOX (with the exception of pullup support)
  // so only apply the value, and the pull-up (that, only if pulldown is not requested,
  // since pull-down and bus-keeper functions are not available)
  else {
    if (((cfg & PINCFG_PULL_MASK) == PINCFG_PULL_HIGH) || 
        ((cfg & PINCFG_PULL_MASK) == PINCFG_PULL_NONE))
    {
      iox_set_pin_pullup(pin & ~IOX_PIN_BASE, cfg & PINCFG_PULL_HIGH);
    }
    iox_set_pin_output(pin & ~IOX_PIN_BASE, cfg & PINCFG_VALUE_HIGH);
  }
}

int pincfg_get(int pin, int on_iox)
{
  int result = 0x8000;
  // PICO pins
  if(! on_iox) {
    // get slew rate
    result |= gpio_get_slew_rate(pin) ? PINCFG_SLEW_RATE_FAST : PINCFG_SLEW_RATE_SLOW;
    // get drive strength
    result |= (gpio_get_drive_strength(pin) << PINCFG_DRIVE_STRENGTH_POS) & PINCFG_DRIVE_STRENGTH_MASK;
    // get pulls
    result |= gpio_is_pulled_up(pin) ? PINCFG_PULL_HIGH : 0;
    result |= gpio_is_pulled_down(pin) ? PINCFG_PULL_LOW : 0;
    // get hysteresis
    result |= gpio_is_input_hysteresis_enabled(pin) ? PINCFG_HYSTERESIS_ON : PINCFG_HYSTERESIS_OFF;
    // get direction
    result |= gpio_is_dir_out(pin) ? PINCFG_DIR_OUT : PINCFG_DIR_IN;
    // get value (either what we drive, or the external input)
    result |= gpio_get(pin) ? PINCFG_VALUE_HIGH : PINCFG_VALUE_LOW;
    // get function (should be SIO for most, PIO0 for TDI/TDO/TMS/TCK, PIO1 for A5CLK)
    result |= (gpio_get_function(pin) << PINCFG_FN_POS) & PINCFG_FN_MASK;
  }
  // IOX pins don't have pulldown, slew rate, drive strength an hysteresis
  else {
    result |= iox_get_pin_pullup(pin) ? PINCFG_PULL_HIGH : 0;
    result |= (iox_get_pin_direction(pin) == GPIO_OUT) ? PINCFG_DIR_OUT : PINCFG_DIR_IN;
    result |= iox_get_pin_value(pin) ? PINCFG_VALUE_HIGH : PINCFG_VALUE_LOW;
    result |= (PINCFG_FN_NULL << PINCFG_FN_POS) & PINCFG_FN_MASK;
  }
  return result;
}

//=[ describe_pin() ]=========================================================

int describe_pin(int pin, struct pindesc_s *d)
{
  d->pinidx = pin;
  d->on_iox = 0;
  if (pin < 0)
    d->valid = 0;
  else if (pin < 32)
    d->valid = pico_pins & (1u << pin);
  else if (pin < IOX_PIN_BASE)
    d->valid = 0;
  else if (pin < IOX_PIN_BASE + IOX_PIN_COUNT) {
    d->valid = iox_pins & (1u << (pin-IOX_PIN_BASE));
    d->on_iox = 1;
  }
  if (! d->valid) {
    d->location = "???";
    d->signal = "???";
    return -1;
  }
  if (pin < 32) {
    d->location = "pico";
    d->signal = pico_signames[pin];
  } else {
    d->location = "iox";
    d->signal = iox_signames[pin - IOX_PIN_BASE];
  }
  return 0;
}

//=[ describe_pincfg() ]======================================================

const char *describe_pincfg(int cfg, int with_out, int on_iox)
{
  static const char *drvstrs[4] = { "2mA", "4mA", "8mA", "12mA" };
  static const char *pulls[4] = { "none", "low", "high", "keep" };
  static char pincfg[80];
  pincfg[0] = 0;
  int n;
  if (on_iox)
    n = sprintf(pincfg, "PULL=%s",
                pulls[(cfg & PINCFG_PULL_HIGH) >> PINCFG_PULL_POS]);
  else
    n = sprintf(pincfg, "SLEW=%s DRIVE=%s PULL=%s HYSTERESIS=%s\n",
                (cfg & PINCFG_SLEW_RATE_MASK) ? "fast" : "slow",
                drvstrs[(cfg & PINCFG_DRIVE_STRENGTH_MASK) >> PINCFG_DRIVE_STRENGTH_POS],
                pulls[(cfg & PINCFG_PULL_MASK) >> PINCFG_PULL_POS],
                (cfg & PINCFG_HYSTERESIS_ON) ? "on" : "off" );
  if (with_out)
    sprintf(pincfg + n, " VALUE=%u", (cfg >> PINCFG_VALUE_POS) & 1);
  return pincfg;
}
