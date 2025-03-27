#pragma once

#include <stdint.h>
#include <hardware/spi.h>

//#define SPI_PARANOIA

// helpers for accessing data in a byte buffer

#define GET_HWORD_AT(ptr) \
  (((uint32_t)((ptr)[0])) | \
   ((uint32_t)((ptr)[1]) << 8))
#define GET_WORD_AT(ptr) \
  (((uint32_t)((ptr)[0])) | \
   ((uint32_t)((ptr)[1]) << 8) | \
   ((uint32_t)((ptr)[2]) << 16) | \
   ((uint32_t)((ptr)[3]) << 24))

#define SET_HWORD_AT(ptr,data) \
   ((ptr)[0] = data & 0xFF, \
    (ptr)[1] = data >> 8)
#define SET_WORD_AT(ptr,data) \
   ((ptr)[0] = data & 0xFF, \
    (ptr)[1] = data >> 8, \
    (ptr)[2] = data >> 16, \
    (ptr)[3] = data >> 24)

// more helpers
#ifndef offsetof
# define offsetof(s,f) ((unsigned)&((*(s*)0).(f)))
#endif
#ifndef countof
# define countof(a) (sizeof(a)/sizeof(a[0]))
#endif


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

extern volatile char adc_busy, eth_busy;
extern int adc_spi_speed, eth_spi_speed;
#ifdef SPI_PARANOIA
extern volatile char adc_ss, eth_ss;
#endif

void claim_spi_for_eth();
void claim_spi_for_adc();
static inline void release_eth_spi()
  { eth_busy = 0; }
static inline void release_adc_spi()
  { adc_busy = 0; }

//=[ miscellaneous ]==========================================================

// these functions shall return 0 if the device is not detected, !=0 otherwise
typedef int (*spi_speed_detect_fn_t)(void);

//#define DEBUG_SPI_SPEEDS

// this function returns 0 if the device is not detected at any attempted
// speed
// note, devname is only used in debug mode
int detect_max_spi_speed(spi_inst_t *device, spi_speed_detect_fn_t detector,
                         unsigned max_speed, const char *devname);

const char *djtag_whoami();

//=[ update the ethernet state ]==============================================

extern const char *macaddr, *hostname;
extern char *ipconfig_ptr;

// if link==-1, srvip is the error message
// if link==0|1, srvip is the IP address, or NULL if in the DHCP phase
void notify_ip_config(int link, const char *srvip, const char *cliip);

//=[ reboot/bootloader logic ]=================================================

// free up the memory reserved by the arm IMC console tracking
void free_console_buffers();

// used by other things to reboot the pico
__attribute__((noreturn))
void bad_error();
// used to reboot to bootloader
__attribute__((noreturn))
void fatal_error();
