#include <stdio.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/bootrom.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>
#include <hardware/clocks.h>
#include <hardware/watchdog.h>
#include <bsp/board.h>
#include "utils.h"
#include "config.h"
#include "ethernet.h"

//=[ LED stuff ]==============================================================

static int led_pin = -1;
static int eth_led_state = 0, jtag_led_state = 0;

void led_init(int pin) {
  led_pin = pin;
  if ( pin != -1 ) {
    if (pin == PIN_LED) {
      bi_decl(bi_1pin_with_name(PIN_LED, "LED")); // this requires a static expression
    }
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
  eth_busy = 1;
  while (adc_busy) // if the ADC is running on the other core, wait for it to finish
    ;
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

//=[ update the ethernet state ]==============================================

char *ipconfig_ptr;

void notify_ip_config(int link, const char *srvip, const char *cliip)
{
  char *p = ipconfig_ptr;
  static int last_link = -1;
  static const char *last_srvip = 0;
  int reprint = 0;
  // re-print the last setting
  if (srvip == (const char*)-1) {
    reprint = 1;
    link = last_link;
    srvip = last_srvip;
  } 
  if (p) {
    if (link < 0)
      sprintf(p, "eth error: %s", srvip);
    else {
      p += sprintf(p, "eth: link ");
      if (! link)
        sprintf(p, "down");
      else {
        p += sprintf(p, "up; ");
        if (! srvip)
          sprintf(p, "waiting for IP...");
        else {
          p += sprintf(p, "address %s; ", srvip);
          if (! cliip)
            sprintf (p, "waiting for client...");
          else
            sprintf (p, "connected to tcp://%s.", cliip);
        }
      }
    }
    // avoid duplicate messages; also, don't actually print when only cliip changes
    // (the tcp service will print instead)
    if (reprint || (link != last_link) || (srvip != last_srvip))
      puts(ipconfig_ptr);
  }
  last_link = link; last_srvip = srvip;
}

//=[ miscellaneous ]==========================================================

#define DEBUG_SPI_SPEEDS

int detect_max_spi_speed(spi_inst_t *device, spi_speed_detect_fn_t detector,
                         unsigned max_speed, const char *devname)
{
  // the flow is: we attempt frenquencies starting at sysclk/2; for every
  // frequency we attempt, if the detector function should fail, we either
  // - try a 1MHz lower frequency than the /achieved/ frequency, if the failed
  //   frequency is >= 10MHz
  // - try a .5MHz lower frequency than the achieved frequency, if the failed
  //   frequency is >= 1MHz
  // - give up, if we got to frequencies below 1MHz
  // re-configure the SPI speed
  unsigned speed = max_speed;
  if (! speed) 
    speed = clock_get_hz(clk_sys) / 2;
  spi_set_baudrate(device, speed);
  speed = spi_get_baudrate(device);
  (void)devname;
  do {
    if (detector()) {
#   ifdef DEBUG_SPI_SPEEDS
      printf("%s SPI: %u.%03uMHz works.\n",
             devname,
             (speed+500)/1000000, ((speed+500)%1000000)/1000);
#     endif
      return speed;
    }
#   ifdef DEBUG_SPI_SPEEDS
    printf("%s SPI: %u.%03uMHz didn't work...\n",
           devname,
           (speed+500)/1000000, ((speed+500)%1000000)/1000);
#   endif
    if (speed >= 10000000)
      speed -= 1000000;
    else if (speed >= 1000000)
      speed -= 500000;
    spi_set_baudrate(device, speed);
    speed = spi_get_baudrate(device);
  } while (speed >= 1000000);
  return 0;
}

void bad_error()
{
  watchdog_reboot(0, 0, 200); // standard boot in 0.2s
  while (1)
    asm volatile ("wfe");
}

void fatal_error()
{
  // this is fatal - wait 200ms then reboot to bootloader
  sleep_ms(200);
  reset_usb_boot(0, 0);
  while (1)
    asm volatile ("wfe");
}

// same as the default panic, excepte we end with fatal_error()
void __attribute__((noreturn)) my_panic(const char *fmt, ...)
{
  // register a standard reboot in 1s
  watchdog_reboot(0, 0, 1000);
  // we start with scheduling the reboot so that even if puts/vprintf mess up,
  // the reboot will still happen
  puts("\n*** PANIC ***\n");
  if (fmt) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    puts("\n");
  }
  puts("Rebooting...");
  while (1)
    asm volatile ("wfe");
}

// just in case: attempt to override _exit too
void __attribute__((noreturn)) _exit(int e)
{
  bad_error();
}
