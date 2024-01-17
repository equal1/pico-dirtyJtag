#ifndef DirtyJtagConfig_h
#define DirtyJtagConfig_h

// Set to 0 to disable USB-CDC-UART bridge
#define USB_CDC_UART_BRIDGE  1


#define BOARD_PICO           0
#define BOARD_ADAFRUIT_ITSY  1
#define BOARD_SPOKE_RP2040   2
#define BOARD_QMTECH_RP2040_DAUGHTERBOARD 3
#define BOARD_WERKZEUG       4
#define BOARD_RP2040_ZERO    5
#define BOARD_E1           6

// Select the board type from the above
//#define BOARD_TYPE BOARD_PICO
//#define BOARD_TYPE BOARD_ADAFRUIT_ITSY
//#define BOARD_TYPE BOARD_SPOKE_RP2040
//#define BOARD_TYPE BOARD_WERKZEUG
//#define BOARD_TYPE BOARD_QMTECH_RP2040_DAUGHTERBOARD
//#define BOARD_TYPE BOARD_RP2040_ZERO
#define BOARD_TYPE BOARD_E1

// General mapping
// TDI  SPIO RX
// TDO  SPIO TX
// TCK  SPIO SCK
// TMS  SPIO CS
// RST  GPIO
// TRST GPIO

#if ( BOARD_TYPE == BOARD_PICO )

#define PIN_TDI 16 
#define PIN_TDO 17
#define PIN_TCK 18
#define PIN_TMS 19
#define PIN_RST 20
#define PIN_TRST 21

#define LED_INVERTED   0
#define PIN_LED_TX     25
#define PIN_LED_ERROR  25
#define PIN_LED_RX     25

#if ( USB_CDC_UART_BRIDGE )
#define PIN_UART_INTF_COUNT 2
#define PIN_UART0 uart0
#define PIN_UART0_TX    12
#define PIN_UART0_RX    13
#define PIN_UART1 uart1
#define PIN_UART1_TX    4
#define PIN_UART1_RX    5
#endif // USB_CDC_UART_BRIDGE

#elif ( BOARD_TYPE == BOARD_E1 )

// equal1 jtag
// - provide 2 CDC interfaces
//   - CDC0 is the UART bridge
//   - CDC1 is the SPI bridge
// - use GP0/GP1 (uart0; header.0,1) for the UART CDC
// - use GP2..GP7 (header.4..7,9,10) for the JTAG
// - use GP10..13 (spi1; header.14..17) for the SPI CDC
// this leaves GP16..21 (spi0+GP20,21; header.21,22,24..27) free
// (in turn, allowing the use of a W5500-EVB-Pico, enabling connections
//  over Ethernet instead of USB in the future)

#define PIN_TMS  2 // header.4
#define PIN_TCK  3 // header.5
#define PIN_TDO  4 // header.6
#define PIN_TDI  5 // header.7
#define PIN_TRST 6 // header.9
#define PIN_RST  7 // header.10

#define LED_INVERTED   0
#define PIN_LED_TX     25
#define PIN_LED_ERROR  25
#define PIN_LED_RX     25

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

#elif ( BOARD_TYPE == BOARD_ADAFRUIT_ITSY )

#define PIN_TDI 28 
#define PIN_TDO 27
#define PIN_TCK 26
#define PIN_TMS 29
#define PIN_RST 24
#define PIN_TRST 25

// no regular LEDs on the Itsy - it's a neopixel
#define LED_INVERTED   0
#define PIN_LED_TX     -1
#define PIN_LED_ERROR  -1
#define PIN_LED_RX     -1

#if ( USB_CDC_UART_BRIDGE )
#define PIN_UART_INTF_COUNT 1
#define PIN_UART0       uart0
#define PIN_UART0_TX    0
#define PIN_UART0_RX    1
#endif // USB_CDC_UART_BRIDGE

#elif ( BOARD_TYPE == BOARD_SPOKE_RP2040 )

#define PIN_TDI 23 
#define PIN_TDO 20
#define PIN_TCK 22
#define PIN_TMS 21
#define PIN_RST 26
#define PIN_TRST 27

#define LED_INVERTED   1
#define PIN_LED_TX     16
#define PIN_LED_ERROR  17
#define PIN_LED_RX     18

#if ( USB_CDC_UART_BRIDGE )
#define PIN_UART_INTF_COUNT 1
#define PIN_UART0       uart0
#define PIN_UART0_TX    28
#define PIN_UART0_RX    29
#endif // USB_CDC_UART_BRIDGE


#elif ( BOARD_TYPE == BOARD_WERKZEUG )

#define PIN_TDI 1
#define PIN_TDO 2
#define PIN_TCK 0
#define PIN_TMS 3
#define PIN_RST 4
#define PIN_TRST 5

#define LED_INVERTED   1
#define PIN_LED_TX     20
#define PIN_LED_ERROR  21
#define PIN_LED_RX     20

#if ( USB_CDC_UART_BRIDGE )
#define PIN_UART_INTF_COUNT 1
#define PIN_UART0       uart0
#define PIN_UART0_TX    28
#define PIN_UART0_RX    29
#endif // USB_CDC_UART_BRIDGE
#elif ( BOARD_TYPE == BOARD_QMTECH_RP2040_DAUGHTERBOARD )

// in rp2040 daughterboard UART pins are connected to FPGA pins
// depending on the FPGA pin configuration there is a possibility
// of damage so these pins are not going to be setup
#define USB_CDC_UART_BRIDGE  0

#define PIN_TDI  16 
#define PIN_TDO  17
#define PIN_TCK  18
#define PIN_TMS  19
// in rp2040 daughterboard these pins are connected to FPGA pins
// depending on the FPGA pin configuration there is a possibility
// of damage so these pins are not going to be setup
// #define PIN_RST  X
// #define PIN_TRST X

#define LED_INVERTED   0
#define PIN_LED_TX     25
#define PIN_LED_ERROR  25
#define PIN_LED_RX     25

#elif ( BOARD_TYPE == BOARD_RP2040_ZERO )

#define PIN_TDI 0
#define PIN_TDO 3
#define PIN_TCK 2
#define PIN_TMS 1
#define PIN_RST 4
#define PIN_TRST 5

// the LED is actually a ws2812 neopixel, using a
// spare pin where we could attach a led, updating the
// neopixel would probably be slow
#define LED_INVERTED   1
#define PIN_LED_TX     29
#define PIN_LED_ERROR  29
#define PIN_LED_RX     29

#if ( USB_CDC_UART_BRIDGE )
#define PIN_UART_INTF_COUNT 2
#define PIN_UART0       uart0
#define PIN_UART0_TX    12
#define PIN_UART0_RX    13
#define PIN_UART1       uart1
#define PIN_UART1_TX    8
#define PIN_UART1_RX    9
#endif // USB_CDC_UART_BRIDGE


#endif // BOARD_TYPE

#ifndef USB_CDC_SPI_BRIDGE
#define USB_CDC_SPI_BRIDGE 0
#endif

#define CDC_NONE  0
#define CDC_UART0 1
#define CDC_UART1 2
#define CDC_SPI   3

#if ( USB_CDC_UART_BRIDGE &&( PIN_UART_INTF_COUNT == 2 )&& USB_CDC_SPI_BRIDGE )
# define NCDC 3
# define CDC0_TYPE CDC_UART0
# define CDC1_TYPE CDC_UART1
# define CDC2_TYPE CDC_SPI
# define CDC0_NAME "UART 0"
# define CDC1_NAME "UART 1"
# define CDC2_NAME "SPI"
#elif ( USB_CDC_UART_BRIDGE &&( PIN_UART_INTF_COUNT == 2 ))
# define NCDC 2
# define CDC0_TYPE CDC_UART0
# define CDC1_TYPE CDC_UART1
# define CDC2_TYPE CDC_NONE
# define CDC0_NAME "UART 0"
# define CDC1_NAME "UART 1"
#elif ( USB_CDC_UART_BRIDGE && USB_CDC_SPI_BRIDGE )
# define NCDC 2
# define CDC0_TYPE CDC_UART0
# define CDC1_TYPE CDC_SPI
# define CDC2_TYPE CDC_NONE
# define CDC0_NAME "UART"
# define CDC1_NAME "SPI"
#elif ( USB_CDC_UART_BRIDGE )
# define NCDC 1
# define CDC0_TYPE CDC_UART0
# define CDC1_TYPE CDC_NONE
# define CDC2_TYPE CDC_NONE
# define CDC0_NAME "UART"
#elif ( USB_CDC_UART_BRIDGE )
# define NCDC 1
# define CDC0_TYPE CDC_SPI
# define CDC1_TYPE CDC_NONE
# define CDC2_TYPE CDC_NONE
# define CDC0_NAME "SPI"
#else
# define NCDC 0
# define CDC0_TYPE CDC_NONE
# define CDC1_TYPE CDC_NONE
# define CDC2_TYPE CDC_NONE
#endif

#endif // DirtyJtagConfig_h
