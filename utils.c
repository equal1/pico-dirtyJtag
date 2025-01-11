#include <hardware/gpio.h>
#include <hardware/spi.h>
#include "utils.h"
#include "config.h"

//=[ LED stuff ]==============================================================

static int led_pin = -1;
static int eth_led_state = 0, jtag_led_state = 0;

void led_init(int pin) {
  led_pin = pin;
  if ( pin != -1 ) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);
  }
  eth_led_state = 0;
  jtag_led_state = 0;
}

static void do_set_led() {
  if (led_pin != -1)
    gpio_put(led_pin, eth_led_state ^ jtag_led_state);
}

void set_led(int which, int state) {
  if (which == LED_ETHERNET)
    eth_led_state = state;
  else if (which == LED_JTAG)
    jtag_led_state = state;
  do_set_led();
}
void toggle_led(int which) {
  if (which == LED_ETHERNET)
    eth_led_state ^= 1;
  else if (which == LED_JTAG)
    jtag_led_state ^= 1;
  do_set_led();
}

//=[ SPI sharing ]============================================================

void claim_spi_for_eth() {
  while (adc_busy) // if the ADC is running on the other core, wait for it to finish
    ;
  eth_busy = 1;
  if (spi_get_baudrate(SPI_ETH) != eth_spi_speed)
    spi_set_baudrate(SPI_ETH, eth_spi_speed);
}

void claim_spi_for_adc() {
retry:
  // wait while ethernet is using the interface
  while (eth_busy)
    ;
  adc_busy = 1;
  // make sure we didn't hit a race condition here
  // we only have this check in the ADC code, to avoid
  // deadlocks (effectively giving ETH higher priority)
  if (eth_busy) {
    adc_busy = 0;
    goto retry;
  }
  if (spi_get_baudrate(SPI_ADC) != adc_spi_speed)
    spi_set_baudrate(SPI_ADC, adc_spi_speed);
}

