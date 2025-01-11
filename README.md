# EQUAL1-JTAG

This code is a Pico firmware for using it as a JTAG adapter; additionally,
it features a lot of functionality specifically targeting talking to equal1
alpha-class chips, attached to equal1 DD-class boards.

The code started as an evolution of
 [pico-DirtyJtag](https://github.com/phdussud/pico-dirtyJtag), itself a port
of 
 [DirtyJtag](https://github.com/jeanthom/DirtyJTAG) project; however, currently
only small parts of the original code remain.

## Differences from pico-DirtyJtag

Four major changes distinguish this from the pico-DirtyJtag project:
1. Single USB transfers longer than 64 bytes are supported
2. Ethernet support
3. ACM support for UARTs was removed
4. Additional commands have been added, either to optimize talking to the ARM
   debug infrastructure, or to perform equal1 chip-specific tasks.

### USB changes

The project should still be mostly compatible with OG pico-DirtyJtag clients.

One major exception is that *USB data packets of exactly 64 bytes are not
supported* (the USB code was modified such that any packet cannot be an
exact multiple of 64 bytes; thus, the client shall append a CMD_STOP should the
TX data be exactly 64\*N bytes long, in order to make it 64\*N+1; additionally,
if the response would be exactly 64\*N bytes long, a single dummy 0xA5 byte
will be added to it before sending it back)

Furthermore, if a commands sequence would produce response data, the code
*expects* that the result would be read before the next commands sequence gets
received. This might, or might not, be the same way OG pico-DirtyJTAG clients
work (it really depends on the internals of the
 [TinyUSB](https://docs.tinyusb.org/en/latest/index.html)
stack - the implementation is completely different though).

The above limitation is considered acceptable, since the reason the OG
pico-DirtyJtag presumably used multi-way buffering was specifically that any
USB transfer was limited to the endpoint size, which is 64 bytes for USB1.1
devices such as the Pico.

The way the USB serial is generated is slightly changed from OG pico-DirtyJTAG;
the unique board ID, which was used to generate the serial number in BE fashion
(board id `0x123456789abcdef0` -> USB serial `f0debc9a78563412`), is now
rotated to the right by 1 byte (resulting in USB serial `12f0debc9a785634`).
This was done when it was realized that attempting to use the last 2 bytes of
the USB serial as a board label resulted in multiple boards with the same
label.

**However**. Since we cannot guarantee 100% compatibility, the USB identifier
was changed from `1209:C0CA` to `1209:C0CB`.

### Ethernet support

The project expects that a W5500 chip is connected to the Pico - the main
target being the
 [W5500-EVB-Pico](https://docs.wiznet.io/Product/iEthernet/W5500/w5500-evb-pico)
board; external W5500 chips are expected to work, if connected to a regular
Pico in exact same fashion it is on the W5500-EVB-Pico board.

The code begins by attempting to detect a SPI frequency at which communication
with the W5500 works; if that chip is not detected at SPI frequencies above
1MHz, the board will switch to USB-only mode.

If Ethernet is present, the value of the LED is a XOR or what Ethernet, and
JTAG respectively, would want. The Etherned aspect of the LED is:
* solid OFF while there's no link;
* flash slowly (.5Hz) while waiting for DHCP to assign an address
* solid ON while there is a link, and an IP was acquired - whether
  a client is connected or not
* a fatal network error will be signalled by the LED flashing fast (5Hz); in
  all cases, the network state can be "reset" by unplugging the LAN cable,
  the re-plugging of which would cause the whole network initialization to
  happen again

---

If a W5500 is detected, the MAC will be set to one based on the Pico's unique
ID (which is 64-bit):
```
  mac = bswapped_rotated_pico_uid & 0x0000f0ffffffffffull | (1ull<<41)
```
per IEEE 802c, that marks the IP address as unicast, administratively assigned
(SLAC AAI) - "locally administered" rather than "universally administered".

> For example, the Pico with unique ID `0xe6625ca563326c38`, USB serial
 `38E6625CA563326C` would use the MAC address `62:5C:A5:63:32:6C`.

Once the MAC is set, a DHCP request will be sent, with the hostname set to
 `pico-<usb_sn>`
(in the above example, it would be 
 `pico-38E6625CA563326C`);
while the DHCP negotiation happens, the LED will flash slowly; once the Pico
gets an IP address, it will turn to solid green.

In the unlikely event of a DHCP error (be it a timeout, or an address conflict),
the ethernet stack will enter the persistent "fatal error" state, with the LED
flashing fast; this can be recovered from by re-plugging the LAN cable or by
resetting the Pico. **Note that the USB connection works even if the ethernet
stack is down** (and that, additionally, DHCP errors are extremely uncommon,
unless the board was connected in a network lacking a DHCP server).



## Signal mapping

### Firmware debug

| Pico signal | Pico GPIO      | Pico pin# |
|:------------|:---------------| -         |
| UART TX     | GP8 (UART1.TX) | 12        |

### Ethernet chip
(WizChip W5500; connected on-board to SPI0, SS# on GP17; not shared)

| W5500 signal | Pico GPIO        | Pico pin# |
|:-------------|:-----------------| -         |
| SCS#         | GP17 (out)       | 22        |
| SCLK         | GP18 (SPI0.SCK)  | 24        |
| MOSI         | GP19 (SPI0.MOSI) | 25        |
| MISO         | GP16 (SPI0.MISO) | 21        |
| RST#         | GP20 (out)       | 26        |
| INT#         | GP21 (in)        | 27        |

### IO Expander chip
(EXAR XRA1403; connected to Pico's SPI1, SS# on GP14; SPI lines shared with A5
 and, on DDv2+, with the ADC)

| IOX signal | Pico GPIO        | Pico pin# | IOX pin# |
|:-----------|:-----------------|:----------| -        |
| CS#        | GP14 (out)       | 19        | 21       |
| SCL        | GP10 (SPI1.SCK)  | 14        | 22       |
| SI         | GP11 (SPI1.MOSI) | 15        | 2        |
| SO         | GP12 (SPI1.MISO) | 16        | 23       |

### ADC chip
(MCP3464R; only available on DDv2+; SPI is connected to Pico's SPI0, SS# on
 GP26; lines shared with A5 and the ADC)

| ADC signal | Pico GPIO        | Pico pin# | ADC pin# |
|:-----------|:-----------------|:----------| -        |
| CS#        | GP26 (out)       | 31        | 11       |
| SCK        | GP18 (SPI0.SCK)  | 24        | 12       |
| SDI        | GP19 (SPI0.MOSI) | 25        | 13       |
| SDO        | GP16 (SPI0.MISO) | 21        | 14       |

### JTAG

| Signal   | Pico GPIO | Pico pin# |
|:---------|:----------| -         |
| TMS      | GP2 (out) | 4         |
| TCK      | GP3 (out) | 5         |
| TDO      | GP4 (in)  | 6         |
| TDI      | GP5 (out) | 7         |

### Alpha chip, serial interfaces
(A5's SPI is connected to Pico's SPI1, SS# on GP13; SPI lines shared with the
 IOX and, on DDv2+, with the ADC)

| A5 signal        | Pico signal      | Pico pin# |
|:-----------------|:-----------------| -         |
| GPIO0 (UART.RX)  | GP0 (UART0.TX)   | 1         |
| GPIO1 (UART.TX)  | GP1 (UART0.RX)   | 2         |
| GPIO2 (SPI.MISO) | GP12 (SPI1.MISO) | 16        |
| GPIO3 (SPI.MOSI) | GP11 (SPI1.MOSI) | 15        |
| GPIO4 (SPI.SCK)  | GP10 (SPI1.SCK)  | 14        |
| GPIO5 (SPI.SS#)  | GP13 (out)       | 17        |

### Alpha chip, directly-connected signals

| A5 signal | Pico signal | Pico pin# |
|:----------|:------------| -         |
| RST#      | GP7 (out)   | 10        |
| CLK       | GP28 (out)  | 34        |
| SCAN_EN   | GP6 (out)   | 9         |
| SCANIN    | GP9 (out)   | 12        |
| SCANOUT   | GP15 (in)   | 20        |
| GPIO6     | GP22 (out)  | 29        |
| GPIO7*    | GP26 (out)* | 31        |
| GPIO10    | GP27 (in)   | 32        |

\* A5's GPIO7 is connected to Pico's GP26 on DDv1 only

### Alpha chip, IOX-connected signals

| Signal   | IOX port  | IOX pin# |
|:---------|:----------| -        |
| GPIO7*   | P3 (out)  | 7        |
| CLK_SRC  | P4 (out)  | 8        |
| TILESEL0 | P5 (out)  | 9        |
| TILESEL1 | P6 (out)  | 10       |
| TEST_EN  | P7 (out)  | 11       |
| GPIO8**  | P8 (out)  | 13       |
| GPIO9    | P9 (out)  | 14       |
| GPIO11   | P10 (out) | 15       |
| GPIO12** | P11 (out) | 16       |
| GPIO13** | P12 (out) | 17       |
| GPIO14** | P13 (out) | 18       |
| GPIO15** | P14 (out) | 19       |
| GPIO16** | P15 (out) | 20       |

\* A5's GPIO7 is connected to IOX's P3 on DDv2 only
\*\* Direction could change at a later time
