#pragma once

#if PICO_RP2350
# define BOARD_NAME "pico2"
# define CHIP_NAME "rp2350"
# define RASPBERRY_BOARD_NAME "Raspberry_Pico2"
# define WIZCHIP_BOARD_NAME "W5500-EVB-Pico2"
# define SYSCLK_KHZ_DEFAULT 150000
//# warning Pico2 (rp2350) build
#elif PICO_RP2040
# define BOARD_NAME "pico"
# define CHIP_NAME "rp2040"
# define RASPBERRY_BOARD_NAME "Raspberry_Pico"
# define WIZCHIP_BOARD_NAME "W5500-EVB-Pico"
# define SYSCLK_KHZ_DEFAULT 125000
//# warning Pico (rp2040) build
#else
# define BOARD_NAME "jtag"
# define CHIP_NAME "???"
# define RASPBERRY_BOARD_NAME BOARD_NAME
# define WIZCHIP_BOARD_NAME BOARD_NAME
# define SYSCLK_KHZ_DEFAULT 0
# error Unknown board - should be either PICO_RP2040 or PICO_RP2350 
#endif

// size of any incoming, or outgoing, buffer
#if 1
# define MTU_SIZE 1500
// round up the buffer size to the next multiple of 64 bytes
# define BUFFER_SIZE ((MTU_SIZE + 63) & ~63)
#else
# define BUFFER_SIZE 2048
#endif

// use optimal W5500 settings
#define W5500_USE_BLOCK
#define W5500_USE_BLOCK_DMA

// equal1 jtag

#define PIN_TMS    2 // GP2, header.4
#define PIN_TCK    3 // GP3, header.5
#define PIN_TDO    4 // GP4, header.6
#define PIN_TDI    5 // GP5, header.7
#define PIN_RST    7 // GP7, header.10

#define PIN_LED   25 // GP25, no header

#define PIN_SPI1_SCK   10 // GP10, header.14
#define PIN_SPI1_MOSI  11 // GP11, header.15
#define PIN_SPI1_MISO  12 // GP12, header.16

#define PIN_SPI0_SCK   18 // GP18, header.24
#define PIN_SPI0_MOSI  19 // GP19, header.25
#define PIN_SPI0_MISO  16 // GP16, header.21

#define PIN_UART0_TX    0 // GP1, header.1
#define PIN_UART0_RX    1 // GP2, header.2

#define PIN_UART1_TX    8 // GP1, header.1
#define PIN_UART1_RX   -1 // not mapped

#define UART_DBG       uart1
#define PIN_DBGTX      PIN_UART1_TX

#define UART_A5        uart0
#define PIN_A5_UART_TX PIN_UART0_RX
#define PIN_A5_UART_RX PIN_UART0_TX

#define ETH_NAME      "W5500"
// #define FREQ_ETH_KHZ  80000 // seems to high, comms are unreliable
#define FREQ_ETH_KHZ  10000
#define SPI_ETH       spi0
#define PIN_ETH_SCK   PIN_SPI0_SCK
#define PIN_ETH_MOSI  PIN_SPI0_MOSI
#define PIN_ETH_MISO  PIN_SPI0_MISO
#define PIN_ETH_CSn   17 // GP17, header.22
#define PIN_ETH_RSTn  20 // GP20, header.26
#define PIN_ETH_INTn  21 // GP21, header.27

#define ADC_NAME    "3464R"
#define FREQ_ADC_KHZ 10000
#define SPI_ADC      spi0
#define PIN_ADC_SCK  PIN_SPI0_SCK
#define PIN_ADC_MOSI PIN_SPI0_MOSI
#define PIN_ADC_MISO PIN_SPI0_MISO
#define PIN_ADC_SSn  26 // GP26

#define IOX_NAME     "XRA1403"
#define FREQ_IOX_KHZ 26000
#define SPI_IOX      spi1
#define PIN_IOX_SCK  PIN_SPI1_SCK
#define PIN_IOX_MOSI PIN_SPI1_MOSI
#define PIN_IOX_MISO PIN_SPI1_MISO
#define PIN_IOX_SSn  14 // GP14, header.19

// maximum A5 SPI frequency is sysclk/12
// with a sysclock of 62.5MHz, max A5 SPI frequency is 5.20MHz
#define FREQ_A5_KHZ 5208
#define SPI_A5      spi1
#define PIN_A5_SCK  PIN_SPI1_SCK
#define PIN_A5_MOSI PIN_SPI1_MOSI
#define PIN_A5_MISO PIN_SPI1_MISO
#define PIN_A5_SSn  13 // GP13, header.17

// other pins on Pico
#define PIN_A5_SCANEN  6  // GP6,  header.9
#define PIN_A5_SCANIN  9  // GP9,  header.12
#define PIN_A5_SCANOUT 15 // GP15, header.20
#define PIN_A5_CLK     28 // GP28, header.34
#define PIN_A5_GPIO10  27 // GP27, header.32
#define PIN_A5_GPIO6   22 // GP22, header.29

// pins on IOX
#define IOX_PIN_BASE    0x40
#define IOX_PIN_COUNT   16
#define PIN_A5_GPIO7    (IOX_PIN_BASE|3)  // P4,  iox.6
#define PIN_A5_CLKSRC   (IOX_PIN_BASE|4)  // P4,  iox.7
#define PIN_A5_TILESEL0 (IOX_PIN_BASE|5)  // P5,  iox.8
#define PIN_A5_TILESEL1 (IOX_PIN_BASE|6)  // P6,  iox.10
#define PIN_A5_TESTEN   (IOX_PIN_BASE|7)  // P7,  iox.11
#define PIN_A5_GPIO8    (IOX_PIN_BASE|8)  // P8,  iox.13
#define PIN_A5_GPIO9    (IOX_PIN_BASE|9)  // P9,  iox.14
#define PIN_A5_GPIO11   (IOX_PIN_BASE|10) // P10, iox.15
#define PIN_A5_GPIO12   (IOX_PIN_BASE|11) // P11, iox.16
#define PIN_A5_GPIO13   (IOX_PIN_BASE|12) // P12, iox.17
#define PIN_A5_GPIO14   (IOX_PIN_BASE|13) // P13, iox.18
#define PIN_A5_GPIO15   (IOX_PIN_BASE|14) // P14, iox.19
#define PIN_A5_GPIO16   (IOX_PIN_BASE|15) // P15, iox.20
