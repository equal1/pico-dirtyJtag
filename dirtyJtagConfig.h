#ifndef DirtyJtagConfig_h
#define DirtyJtagConfig_h

// Set to 0 to disable USB-CDC-UART bridge
#define USB_CDC_UART_BRIDGE  0

#define BOARD_PICO           0
#define BOARD_RP2040_ZERO    5
#define BOARD_E1           6

#define BOARD_TYPE BOARD_E1

// General mapping
// TDI  SPIO RX
// TDO  SPIO TX
// TCK  SPIO SCK
// TMS  SPIO CS
// RST  GPIO
// TRST GPIO

#if ( BOARD_TYPE == BOARD_E1 )

// equal1 jtag

#define PIN_TMS    2 // GP2, header.4
#define PIN_TCK    3 // GP3, header.5
#define PIN_TDO    4 // GP4, header.6
#define PIN_TDI    5 // GP5, header.7
#define PIN_RST    7 // GP7, header.10

#define PIN_LED   25 // GP25, 
#define PIN_DBGTX  8 // GP8, header.12

#define PIN_SCK   10 // GP10, header.14
#define PIN_MOSI  11 // GP11, header.15
#define PIN_MISO  12 // GP12, header.16

#define PIN_TX     1 // GP1, header.1
#define PIN_RX     2 // GP2, header.2

#define UART_A5        uart0
#define PIN_A5_UART_TX PIN_RX
#define PIN_A5_UART_RX PIN_TX

//#define FREQ_W5500_KHZ  62500 // won't work, but we'll try to find a speed that works
#define FREQ_W5500_KHZ  6000 // actually 5.682MHz - this is the highest that seems to work fine
#define SPI_W5500       spi0
#define PIN_W5500_CSn   17 // GP17, header.22
#define PIN_W5500_SCK   18 // GP18, header.24
#define PIN_W5500_MOSI  19 // GP19, header.25r
#define PIN_W5500_MISO  16 // GP16, header.21
#define PIN_W5500_RSTn  20 // GP20, header.26
#define PIN_W5500_INTn  21 // GP21, header.27

#define SPI_IOX      spi1
#define PIN_IOX_SSn  14 // GP14, header.19
#define PIN_IOX_SCK  PIN_SCK
#define PIN_IOX_MOSI PIN_MOSI
#define PIN_IOX_MISO PIN_MISO

#define SPI_A5      spi1
#define PIN_A5_SSn  13 // GP13, header.17
#define PIN_A5_SCK  PIN_SCK
#define PIN_A5_MOSI PIN_MOSI
#define PIN_A5_MISO PIN_MISO

// other pins on Pico
#define PIN_A5_SCANIN  9  // GP9,  header.12
#define PIN_A5_SCANOUT 15 // GP15, header.20
#define PIN_A5_CLK     28 // GP28, header.34
#define PIN_A5_GPIO10  27 // GP27, header.32
#define PIN_A5_GPIO7   26 // GP26, header.31
#define PIN_A5_GPIO6   22 // GP22, header.29

// pins on IOX
#define PIN_A5_CLKSRC   (0x40|4)  // P4,  iox.8
#define PIN_A5_TILESEL0 (0x40|5)  // P5,  iox.8
#define PIN_A5_TILESEL1 (0x40|6)  // P6,  iox.10
#define PIN_A5_TESTEN   (0x40|7)  // P7,  iox.11
#define PIN_A5_GPIO8    (0x40|8)  // P8,  iox.13
#define PIN_A5_GPIO9    (0x40|9)  // P9,  iox.14
#define PIN_A5_GPIO11   (0x40|10) // P10, iox.15
#define PIN_A5_GPIO12   (0x40|11) // P11, iox.16
#define PIN_A5_GPIO13   (0x40|12) // P12, iox.17
#define PIN_A5_GPIO14   (0x40|13) // P13, iox.18
#define PIN_A5_GPIO15   (0x40|14) // P14, iox.19
#define PIN_A5_GPIO16   (0x40|15) // P15, iox.20

#define PIN_A5_IOX_FIRST PIN_A5_CLKSRC
#define PIN_A5_IOX_LAST  PIN_A5_GPIO16

#undef USB_CDC_UART_BRIDGE
#define USB_CDC_UART_BRIDGE  1
#define USB_CDC_SPI_BRIDGE  1

#define PIN_UART_INTF_COUNT 1
#define PIN_UART0 uart0
#define PIN_UART0_TX    0 // header.1
#define PIN_UART0_RX    1 // header.2

#define PIN_SPI_INTF_COUNT 1
#define PIN_SPI0 spi1
#define PIN_SPI0_SCLK    10 // header.14
#define PIN_SPI0_MOSI    11 // header.15
#define PIN_SPI0_MISO    12 // header.16
#define PIN_SPI0_CSn     13 // header.17

#else
# error Only the e1 jtag supported on this tree
#endif // BOARD_TYPE

#endif // DirtyJtagConfig_h
