#include "led.h"
#include "hardware/gpio.h"

static int led_pin = -1, last_state = 0;

void led_init( int pin ) {
  led_pin = pin;
  if ( pin != -1 ) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, last_state = 0);
  }
}

void set_led(int state) {
  if (led_pin != -1)  
    gpio_put(led_pin, last_state = state);
}

void toggle_led() {
  if (led_pin != -1)  
    gpio_put(led_pin, last_state ^= 1);
}
