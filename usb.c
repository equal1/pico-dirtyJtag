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
  ITF_NUM_TOTAL
};

#define PROBE_OUT_EP_NUM 0x01
#define PROBE_IN_EP_NUM  0x82

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

uint8_t const desc_configuration[CONFIG_TOTAL_LEN] =
{
  // Config number, interface count, string index, total length, attribute, power in mA
  TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

  // Interface 2 : Interface number, string index, EP Out & IN address, EP size
  TUD_VENDOR_DESCRIPTOR(ITF_NUM_PROBE, 0, PROBE_OUT_EP_NUM, PROBE_IN_EP_NUM, 64),
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
char const *string_desc_arr[] =
{
    (const char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    STR_VENDOR,                 // 1: Manufacturer
    STR_DEVICE,                 // 2: Product
    usb_serial,                 // 3: Serial, uses flash unique ID
};

static uint16_t _desc_str[32];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint16_t const* tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
  (void) langid;

  uint8_t chr_count;

  if ( index == 0)
  {
    memcpy(&_desc_str[1], string_desc_arr[0], 2);
    chr_count = 1;
  }else
  {
    // Convert ASCII string into UTF-16

    if ( !(index < sizeof(string_desc_arr)/sizeof(string_desc_arr[0])) ) return NULL;

    const char* str = string_desc_arr[index];

    // Cap at max char
    chr_count = strlen(str);
    if ( chr_count > 31 ) chr_count = 31;

    for(uint8_t i=0; i<chr_count; i++)
    {
      _desc_str[1+i] = str[i];
    }
  }

  // first byte is length (including header), second byte is string type
  _desc_str[0] = (TUSB_DESC_STRING << 8 ) | (2*chr_count + 2);

  return _desc_str;
}
