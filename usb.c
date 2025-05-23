/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <stdint.h>
#include <string.h>
#include "config.h"
#include <tusb.h>

//#define USB_BCD   0x0110
#define USB_BCD   0x0200

#define USB_VENDOR 0x1209
#define USB_DEVICE 0xC0CB
#define STR_VENDOR "equal1"
#define STR_DEVICE "JTAG"
#define STR_CDC    "JTAG debug interface"


/* C string for iSerialNumber in USB Device Descriptor, two chars per byte + terminating NUL */
static char usb_serial[20];

int usb_init(uint64_t uid)
{
    sprintf(usb_serial, "%016llX", uid);
    tusb_init();
    return 0;
}

// this is to work around the fact that tinyUSB does not handle setup request automatically
// Hence this boiler plate code
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *request);

//--------------------------------------------------------------------+
// Device Descriptors
//--------------------------------------------------------------------+
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = 0x00, // Each interface specifies its own
    .bDeviceSubClass    = 0x00, // Each interface specifies its own
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

	  .idVendor = USB_VENDOR,
	  .idProduct = USB_DEVICE,
	  .bcdDevice = 0x0111,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const * tud_descriptor_device_cb(void)
{
  return (uint8_t const *) &desc_device;
}

//--------------------------------------------------------------------+
// Configuration Descriptor
//--------------------------------------------------------------------+

enum
{
  ITF_NUM_PROBE = 0,
# ifdef ENABLE_USB_TTY
  ITF_NUM_CDC_1 = 1, // stdout
  ITF_NUM_CDC_1_DATA,
# endif
  ITF_NUM_TOTAL
};

#define PROBE_OUT_EP_NUM  0x01
#define PROBE_IN_EP_NUM   0x82
#define CDC_NOTIF_EP1_NUM 0x83 // stdout
#define CDC_OUT_EP1_NUM   0x03
#define CDC_IN_EP1_NUM    0x84

#ifdef ENABLE_USB_TTY
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN + TUD_CDC_DESC_LEN)
#else
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)
#endif

uint8_t const desc_configuration[CONFIG_TOTAL_LEN] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

  // Interface 2 : Interface number, string index, EP Out & IN address, EP size
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_PROBE, 0, PROBE_OUT_EP_NUM, PROBE_IN_EP_NUM, 64),

# ifdef ENABLE_USB_TTY
  // Interface 3 : Interface number, string index, EP notification address and size, EP data address (out, in) and size.
  TUD_CDC_DESCRIPTOR(ITF_NUM_CDC_1, 4, CDC_NOTIF_EP1_NUM, 8, CDC_OUT_EP1_NUM, CDC_IN_EP1_NUM, 64)
# endif
};

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const * tud_descriptor_configuration_cb(uint8_t index)
{
  (void) index; // for multiple configurations
  return desc_configuration;
}

//--------------------------------------------------------------------+
// String Descriptors
//--------------------------------------------------------------------+

// array of pointer to string descriptors
static const char *const descriptor[] =
{
  (const char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
  STR_VENDOR,                 // 1: Manufacturer
  STR_DEVICE,                 // 2: Product
  usb_serial,                 // 3: Serial, uses flash unique ID
# ifdef ENABLE_USB_TTY
  STR_CDC,                    // 4: CDC Interface 0
# endif
};

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void)langid;
  unsigned n;
  static uint16_t _desc_str[32];
  uint16_t *d = _desc_str + 1;
  // first descriptor: language list ([0x0409: en-US])
  if (! index) {
    *d = *(uint16_t*)(descriptor[0]);
    n = 1;
  } 
  // bad index?
  else if ((index >= sizeof(descriptor)/sizeof(descriptor[0])) )
    return NULL;
  // all other descriptors: UTF16 strings
  else {
    const char *s = descriptor[index];
    n = 0;
    while (*s) {
      *d++ = *s++;
      if (++n == 31)
        break;
    }
  }
  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*n + 2);
  return _desc_str;
}
