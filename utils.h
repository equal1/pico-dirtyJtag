#pragma once

#include <stdint.h>

//=[ LED stuff ]==============================================================

enum led_type_e {
  LED_NONE = 0,
  LED_ETHERNET,
  LED_JTAG
};

void led_init(int pin);

void set_led(int which, int val);
void toggle_led(int which);

//=[ SPI sharing ]============================================================

extern volatile uint8_t adc_busy, eth_busy;
extern int adc_spi_speed, eth_spi_speed;

void claim_spi_for_eth();
void claim_spi_for_adc();
static inline void release_eth_spi()
  { eth_busy = 0; }
static inline void release_adc_spi()
  { adc_busy = 0; }
