#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#include <pico/stdlib.h>
#include <hardware/watchdog.h>
#include <hardware/flash.h>
#include <hardware/xip_cache.h>
#include <hardware/sync.h>
#include <hardware/structs/psm.h>
#include <pico/flash.h>
#include <pico/multicore.h>
#include <pico/bootrom.h>
#include <wizchip_conf.h>
#include <w5500.h>
#include <socket.h>

#include "config.h"
#include "ethernet.h"
#include "pico-w5500/Ethernet/W5500/w5500.h"
#include "utils.h"

//#define DEBUG

#ifdef DEBUG
#define dprintf(...) printf(__VA_ARGS__)
#else
#define dprintf(...) (void)(__VA_ARGS__)
#endif

static void fwupd_on_link_up(int, struct dbgsvc_s*);
static void fwupd_on_link_down(int, struct dbgsvc_s*);
static void fwupd_process(int, int, struct dbgsvc_s*);

// service structure
static struct fwupd_s {
  // current state
  enum {
    NO_LINK = 0,
    WAIT_WRQ,
    WAIT_DATA,
    DONE
  } state;
  unsigned next_seq;
# if 0
  // tftp options - currently unused
  unsigned blksize, tsize;
# endif
  // current tftp client
  union {
    uint8_t  byte[4];
    uint32_t ip;
  } cli_ip;
  uint16_t cli_port;
  // I/O buffers
  union {
    uint8_t in[516];
    struct {
      struct {
        uint16_t op;
        uint16_t seq;
      } in_tftp;
      struct {
        uint32_t magic0;       // +0x000
        uint32_t magic1;       // +0x004
        uint32_t flags;        // +0x008
        uint32_t target_addr;  // +0x00c
        uint32_t payload_size; // +0x010
        uint32_t block_no;     // +0x014
        uint32_t num_blocks;   // +0x018
        union {                // +0x01c
          uint32_t family_id;  
          uint32_t file_size;
        };
        uint8_t  data[476];    // +0x020
        uint32_t magic_end;    // +0x1fc
      } in_uf2;
    };
  };
  uint8_t out[60];
} fwupd;

// firmware structure
struct {
  // update buffer
  uint8_t *buf;
  unsigned size;
  uint32_t start, end, next_addr;
  unsigned n_blocks, got_blocks, next_block;
  unsigned n_flash_blocks, n_flash_sectors, n_flash_pages;
} fw;

//=[ services init ]=========================================================

volatile int flash_in_progress;

int fwupd_init()
{
  flash_in_progress = 0;
  fw.buf = 0;
  int res = eth_register_service(SOCKET_FWUPD, NULL,
                                 fwupd_on_link_up, fwupd_on_link_down,
                                 fwupd_process);
  if (res)
    printf("%s: FAILED to register service on port %u, socket #%u\n",
           "fwupd", PORT_TFTP, SOCKET_FWUPD);
  return res;
}

//=[ link up/down handlers ]=================================================

static void fwupd_state_reset()
{
  // reset the state
  fwupd.state = WAIT_WRQ;
  fwupd.next_seq = 0;
  if (fw.buf) {
    free(fw.buf);
    fw.buf = 0;
  }
  fw.start = 0; fw.end = 0;
  fw.size = 0;
  fw.n_blocks = 0; fw.got_blocks = 0;
}

void fwupd_on_link_up(int sock, struct dbgsvc_s *svc)
{
  dprintf("fwupd_on_link_up(%u)\n", sock);
  fwupd_state_reset();
  // get the socket to the point where we can receive incoming connections
  dprintf("fwupd: enabling\n");
  if (sock != socket(sock, Sn_MR_UDP, PORT_TFTP, 0x00)) {
    printf("fwupd: cannot allocate socket #%u!\n", sock);
    return;
  }
  int sstate = getSn_SR(sock);
  if (sstate != SOCK_UDP) {
    printf("fwupd: socket state wrong after initialization (%s), expected SOCK_UDP!\n",
           ssstr(sstate));
    return;
  }
  // use svc!=NULL as "quiet mode"
  if (! svc)
    printf("fwupd: listening on port udp/%u, socket #%u\n",
           PORT_TFTP, sock);
}

void fwupd_on_link_down(int sock, struct dbgsvc_s *svc)
{
  (void)svc;
  dprintf("fwupd_on_link_down(%u)\n", sock);
  // set the state to "no link"; the rest of the state will get
  // reset when the link goes up
  fwupd.state = NO_LINK;
  // close the socket
  close(sock);
  // use svc!=NULL as "quiet mode"
  if (! svc)
    puts("fwupd: disabling");
}

//=[ main service loop ]=====================================================

static int tftp_error(int code, const char *msg);

#define UF2_MAGIC_START0 0x0A324655
#define UF2_MAGIC_START1 0x9E5D5157
#define UF2_MAGIC_END    0x0AB16F30
#define UF2_FLAG_NOT_MAIN_FLASH         0x00000001
#define UF2_FLAG_FILE_CONTAINER         0x00001000
#define UF2_FLAG_FAMILY_ID_PRESENT      0x00002000
#define UF2_FLAG_MD5_CHKSUM_PRESENT     0x00004000
#define UF2_FLAG_EXTENSION_TAGS_PRESENT 0x00008000

#define UF2_FLAGS_CHECK_MASK (UF2_FLAG_NOT_MAIN_FLASH|UF2_FLAG_FILE_CONTAINER|UF2_FLAG_FAMILY_ID_PRESENT)
#define UF2_FLAGS_CHECK_EXPECTED (UF2_FLAG_FAMILY_ID_PRESENT) // ! not_main_flash, ! file_container, family_id_present

#define PLAT_RP2040          0xe48bff56
#define PLAT_RP2XXX_ABSOLUTE 0xe48bff57
#define PLAT_RP2XXX_DATA     0xe48bff58
#define PLAT_RP2350_ARM_S    0xe48bff59
#define PLAT_RP2350_RISCV    0xe48bff5a
#define PLAT_RP2350_ARM_NS   0xe48bff5b

// validate an uf2 sector; return 0 on success, pointer to the error on failure
static const char *uf2_err()
{
  // check that the format is actually UF2
  if ((fwupd.in_uf2.magic0 != UF2_MAGIC_START0) ||
      (fwupd.in_uf2.magic1 != UF2_MAGIC_START1) ||
      (fwupd.in_uf2.magic_end != UF2_MAGIC_END))
    return "image format not UF2";
  // check that we have the expected flags
  if ((fwupd.in_uf2.flags & UF2_FLAGS_CHECK_MASK) != UF2_FLAGS_CHECK_EXPECTED) {
    printf ("fwupd: image flags: 0x%08X, expected 0x%08X\n", fwupd.in_uf2.flags, 
            (fwupd.in_uf2.flags & ~UF2_FLAGS_CHECK_MASK) | UF2_FLAGS_CHECK_EXPECTED);
    return "unexpected UF2 flags";
  }
  // check that we have a known architecture
  if ((fwupd.in_uf2.family_id < PLAT_RP2040) || (fwupd.in_uf2.family_id > PLAT_RP2350_ARM_NS))
  {
    printf ("fwupd: bad family_id 0x%08X\n", fwupd.in_uf2.family_id);
    return "unexpected UF2 flags";
  }
  return 0;
}

// can't make this static, lest it gets inlined, and not sure how to use 
//void __no_inline_not_in_flash_func(do_flash_board)(void*);
__attribute__((noinline, section(".time_critical.fwupd")))
static void do_flash_board(void*);

void fwupd_process(int sock, int sstate, struct dbgsvc_s *svc)
{
  // if the socket was closed, or not established, there's nothing to do
  if (sstate == SOCK_CLOSED)
    fwupd_on_link_up(sock, NULL);

  // check if there's data to receive
  int expect = getSn_RX_RSR(sock);
  if (! expect)
    return;
  // packets received over UDP have an 8-byte header, apparently
  // that's probably {
  //   uint16_t src_port, dst_port, len, cksum; // udp_hdr @0, @2, @4, @6
  // }
  expect -= 8;
  if (! expect)
    return;
  if (expect < 0) {
    puts("fwupd: read error");
  close_and_exit:
    puts("fwupd: aborting session");
    fwupd_on_link_down(sock, (struct dbgsvc_s*)1);
    fwupd_on_link_up(sock, (struct dbgsvc_s*)1);
    return;
  }
  if (expect > sizeof(fwupd.in)) {
    printf("fwupd: packet too large (size=%d, max=%u)\n", expect, sizeof(fwupd.in));
    goto close_and_exit;
  }
  int got;
  // for UDP, we also get the IP and port of the sender
  uint32_t cli_ip; uint16_t cli_port;
  got = recvfrom(sock, fwupd.in, expect, (uint8_t*)&cli_ip, &cli_port);
  if (got < 0) {
    puts("fwupd: receive error");
    goto close_and_exit;
  }
  if (expect != got) {
    printf("fwupd: read error (expected=%d, got=%d)\n", expect, got);
    goto close_and_exit;
  }
  if (got < 4) {
    printf("fwupd: ignoring bogus packet (size=%d)\n", got);
    return;
  }
  // if this is a different client from the last one we've seen, say so
  if ((fwupd.cli_ip.ip != cli_ip )||(fwupd.cli_port != cli_port)) {
    fwupd.cli_ip.ip = cli_ip;
    fwupd.cli_port = cli_port;
    printf("%s: talking to %u.%u.%u.%u:%u\n",
           "fwupd",
           fwupd.cli_ip.byte[0], fwupd.cli_ip.byte[1], fwupd.cli_ip.byte[2], fwupd.cli_ip.byte[3],
           cli_port);
  }
  int err = 0; // error response size
  unsigned tftp_op = __builtin_bswap16(fwupd.in_tftp.op);
  unsigned tftp_seq = __builtin_bswap16(fwupd.in_tftp.seq);
  do { // } while(0) - just to abort sequence via break
    //-------------------------------------------------------------------------
    // is this the beginning of a session?
    if (fwupd.state == WAIT_WRQ) {
      // convert everything to lowercase
      for (int i = 0; i < got; ++i)
        if (isupper(fwupd.in[i]))
          fwupd.in[i] = tolower(fwupd.in[i]);
      if (tftp_op != TFTP_WRQ) {
        err = tftp_error(TFTP_ERR_ACCESS, "expected WRQ");
        break;
      }
      int i = 2;
      // skip the filename
      while(fwupd.in[i] && (i < got))
        ++i;
      if (! fwupd.in[i])
        ++i;
      if(i >= got) {
        err = tftp_error(TFTP_ERR_GENERIC, "malformed WRQ packet");
        break;
      }
      // make sure that the transfer type is "octet"
      if (memcmp (fwupd.in + i, "octet", 6)) {
        err = tftp_error(TFTP_ERR_GENERIC, "expected \"octet\" transfer");
        break;
      } else
        i += 6;
      // ignore options - should be fine, since ACK means "acknowledge without
      // the options"
      printf("fwupd: got WRQ (%s)\n", fwupd.in + 2);
      fwupd.state = WAIT_DATA;
    }
    //-------------------------------------------------------------------------
    // nope: expect data packet
    else { // if (fwupd.state == WAIT_DATA)
      if (tftp_op != TFTP_DATA) {
        err = tftp_error(TFTP_ERR_ACCESS, "expected DATA");
        break;
      }
      // check sequence
      if (fwupd.next_seq != tftp_seq) {
        err = tftp_error(TFTP_ERR_GENERIC, "packet sequence lost");
        break;
      }
      printf("fwupd: got DATA (%u)\n", tftp_seq, got);
      // all packets except the last one need to be precisely 512 byte long, with a
      // 4-byte header
      if (got == 516) {
        // sanity check the uf2
        const char *uf2err = uf2_err();
        if (uf2err) {
          printf ("fwupd: malformed image (not/bad UF2 format)\n");
          err = tftp_error(TFTP_ERR_GENERIC, uf2err);
          break;
        }
        // note: pico2 fw images are strangely constructed
        //   first block has target_addr=0x10fffff0, size=0x100, block 0/2 and tags
        // just ignore these
        if ((fwupd.in_uf2.family_id == PLAT_RP2XXX_ABSOLUTE) &&
            (fwupd.in_uf2.target_addr == 0x10ffff00) &&
            (fwupd.in_uf2.payload_size == 0x100) &&
            (! fwupd.in_uf2.block_no) && (fwupd.in_uf2.num_blocks == 2))
        {
          printf ("fwupd: pico2 magic block; ignoring\n");
          break;
        }
        // if we got here, family_id is in the acceptable range
        // check for the right type of firmware image
#       if PICO_RP2040
        if (fwupd.in_uf2.family_id != PLAT_RP2040) {
          if ((fwupd.in_uf2.family_id == PLAT_RP2350_ARM_NS) ||
              (fwupd.in_uf2.family_id == PLAT_RP2350_ARM_S) ||
              (fwupd.in_uf2.family_id == PLAT_RP2350_RISCV))
            err = tftp_error(TFTP_ERR_GENERIC, "expected Pico firmware (not Pico2)");
          else
            err = tftp_error(TFTP_ERR_GENERIC, "unsupported firmware type");
        }
#       elif PICO_RP2350
        if ((fwupd.in_uf2.family_id != PLAT_RP2350_ARM_NS) &&
            (fwupd.in_uf2.family_id != PLAT_RP2350_ARM_S))
        {

          if (fwupd.in_uf2.family_id == PLAT_RP2040)
            err = tftp_error(TFTP_ERR_GENERIC, "expected Pico2 firmware (not Pico)");
          else
            err = tftp_error(TFTP_ERR_GENERIC, "unsupported firmware type");
        }
#       else
        err = tftp_error(TFTP_ERR_GENERIC, "unknown firmware type");
#       endif
        // on format error here, print the details
        if (err) {
          printf ("fwupd: bad uf2 = {.family=0x%08X, .addr=0x%08X, .size=0x%X, .block=%u/%u }\n",
                  fwupd.in_uf2.family_id, fwupd.in_uf2.target_addr, fwupd.in_uf2.payload_size,
                  fwupd.in_uf2.block_no, fwupd.in_uf2.num_blocks);
          break;
        }
        // on the 1st block, init some extra state
        if (! fw.buf) {
          // check that the first block has sequence #0, and begins at the beginning of the flash
          if (fwupd.in_uf2.block_no) {
            printf ("fwupd: invalid block_no %u, expected 0\n", fwupd.in_uf2.block_no);
            err = tftp_error(TFTP_ERR_GENERIC, "firmware doesn't begin with block 0");
            break;
          }
          if (fwupd.in_uf2.target_addr != 0x10000000) {
            printf ("fwupd: invalid address 0x%08X, expected 0x10000000\n", fwupd.in_uf2.target_addr);
            err = tftp_error(TFTP_ERR_GENERIC, "firmware doesn't target the beginning of the flash");
            break;
          }
          // seems good - allocate the buffer
          unsigned alloc_size = 256 * fwupd.in_uf2.num_blocks;
          printf ("fwupd: allocating a %ubytes firmware buffer\n", alloc_size);
          if (! (fw.buf = (uint8_t*)malloc(alloc_size))) {
            // try again, after sacrificing the ARM IMC console buffers
            free_console_buffers();
            if (! (fw.buf = (uint8_t*)malloc(alloc_size))) {
              err = tftp_error(TFTP_ERR_GENERIC, "not enough memory");
              break;
            }
          }
          // reset the rest of the state
          fw.size = 0; fw.got_blocks = 0;
          fw.n_blocks = fwupd.in_uf2.num_blocks;
          fw.start = fwupd.in_uf2.target_addr; fw.end = fw.start + alloc_size;
          // the first block is always expected
          fw.next_block = fwupd.in_uf2.block_no;
          fw.next_addr = fw.start;
        }
        // make sure we got what we expected, and which makes sense
        if ((fw.next_addr != fwupd.in_uf2.target_addr) ||
            (fw.next_block != fwupd.in_uf2.block_no) ||
            (fw.n_blocks != fwupd.in_uf2.num_blocks) ||
            (fwupd.in_uf2.target_addr < fw.start) ||
            ((fwupd.in_uf2.target_addr + fwupd.in_uf2.payload_size) > fw.end))
        {
          err = tftp_error(TFTP_ERR_GENERIC, "uf2 sequence error");
          break;
        }
        // copy the first buffer
        dprintf ("> %06x..%06x\n", fw.size, fw.size + fwupd.in_uf2.payload_size - 1);
        memcpy(fw.buf + fw.size, fwupd.in_uf2.data, fwupd.in_uf2.payload_size);
        // update state
        fw.size += fwupd.in_uf2.payload_size; fw.got_blocks++;
        fw.next_block++;
        fw.next_addr = fw.start + fw.size;
        if (fw.next_addr > fw.end) {
          err = tftp_error(TFTP_ERR_GENERIC, "uf2 buffer overflow");
          break;
        }
      }
      // last packet? should be 4-byte header + 0-byte payload
      else { // if (got != 516) {
        if (got != 4) {
          err = tftp_error(TFTP_ERR_GENERIC, "invalid last packet");
          break;
        }
        fwupd.state = DONE;
      }
    } 
  } while (0);
  // if we got an error
  if (err) {
    printf("fwupd: err %02X \"%s\"\n", fwupd.out[3], fwupd.out + 4);
#   if 0
    printf("> (%u)", err);
    int i;
    for (i = 0; i < err && i < 40; ++i)
      printf(" %02X", fwupd.out[i]);
    if (i < err)
      printf("...");
    printf(" [");
    for (i = 0; i < err && i < 40; ++i)
      putchar ((fwupd.out[i] >= ' ')? fwupd.out[i] : '?');
    if (i < err)
      printf("...");
    puts("]");
    printf("  in response to\n");
    printf("< (%u)", got);
    for (i = 0; i < got && i < 40; ++i)
      printf(" %02X", fwupd.in[i]);
    if (i < got)
      printf("...");
    printf(" [");
    for (i = 0; i < got && i < 40; ++i)
      putchar ((fwupd.in[i] >= ' ')? fwupd.in[i] : '?');
    if (i < got)
      printf("...");
    puts("]");
#   endif
    // send the error packet
    sendto(sock, fwupd.out, err, fwupd.cli_ip.byte, fwupd.cli_port);
    // reset state to wait-for-transfer
    fwupd_state_reset();
  }
  // if no error, send an ACK
  else {
    *(uint32_t*)fwupd.out = 0;
    fwupd.out[1] = TFTP_ACK;
    fwupd.out[2] = (fwupd.next_seq >> 8) & 0xff;
    fwupd.out[3] = fwupd.next_seq & 0xff;
    int rsz = 4;
    dprintf("fwupd: ack %u\n", (((unsigned)fwupd.out[2]) << 8)| fwupd.out[3]);
#   if 0
    printf("> (%u)", rsz);
    int i;
    for (i = 0; i < rsz && i < 40; ++i)
      printf(" %02X", fwupd.out[i]);
    puts("");
    printf("  in response to\n");
    printf("< (%u)", got);
    for (i = 0; i < got && i < 40; ++i)
      printf(" %02X", fwupd.in[i]);
    if (i < got)
      printf("...");
    printf(" [");
    for (i = 0; i < got && i < 40; ++i)
      putchar ((fwupd.in[i] >= ' ')? fwupd.in[i] : '?');
    if (i < got)
      printf("...");
    puts("]");
#   endif
    // send the error packet
    sendto(sock, fwupd.out, rsz, fwupd.cli_ip.byte, fwupd.cli_port);
    // iterate the ID of the next block
    fwupd.next_seq++;
  }
  // if we got the whole image
  if (fwupd.state == DONE) {
    fw.n_flash_blocks = (fw.size + (FLASH_BLOCK_SIZE-1)) / FLASH_BLOCK_SIZE;
    fw.n_flash_sectors = (fw.size + (FLASH_SECTOR_SIZE-1)) / FLASH_SECTOR_SIZE;
    fw.n_flash_pages = (fw.size + (FLASH_PAGE_SIZE-1)) / FLASH_PAGE_SIZE;
    printf("fwupd: done receiving; size=%u (bytes=0x%X: blocks=%u, sectors=%u, n_pages=%u)\n",
           fw.size, fw.size,
           fw.n_flash_blocks, fw.n_flash_sectors, fw.n_flash_pages);
    puts("fwupd: stopping core 1...");
    // put core1 in reset
    multicore_reset_core1();
    puts("fwupd: start flashing...");
    // flash the new image
    do_flash_board(0);
    // unreachable
    puts("fwupd: flashing failed (somehow)");
    fwupd_state_reset();
  }
}

int tftp_error(int code, const char *msg)
{
  *(uint32_t*)fwupd.out = 0;
  fwupd.out[1] = TFTP_ERROR;
  fwupd.out[3] = code;
  strcpy(fwupd.out + 4, msg);
  return 4 + strlen(fwupd.out + 4) + 1;
}

#define FLASH_BLOCK_ERASE_CMD 0xd8

// shameless adaptation of the code in the Pico SDK, except it does the erasing,
// programming, and rebooting back-to-back; also it doesn't bother with keeping
// the system functional after the flashing
// had to do it this way, since there was no easy way of keeping all the fns we
// needed in RAM/ROM
void do_flash_board(void *unused)
{
  (void)unused;
  // schedule a reboot in 15s
  watchdog_reboot(0, 0, 15000);
  gpio_put(PIN_LED, 1);

  // grab the required ROM function pointers
  rom_connect_internal_flash_fn connect_internal_flash_func = (rom_connect_internal_flash_fn)rom_func_lookup_inline(ROM_FUNC_CONNECT_INTERNAL_FLASH);
  rom_flash_exit_xip_fn flash_exit_xip_func = (rom_flash_exit_xip_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_EXIT_XIP);
  rom_flash_range_erase_fn flash_range_erase_func = (rom_flash_range_erase_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_ERASE);
  rom_flash_range_program_fn flash_range_program_func = (rom_flash_range_program_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_RANGE_PROGRAM);
  rom_flash_flush_cache_fn flash_flush_cache_func = (rom_flash_flush_cache_fn)rom_func_lookup_inline(ROM_FUNC_FLASH_FLUSH_CACHE);
  if (! (connect_internal_flash_func && flash_exit_xip_func && flash_range_erase_func && flash_range_program_func && flash_flush_cache_func))
    return;
  unsigned erase_size = (fw.size + (FLASH_BLOCK_SIZE - 1)) & ~(FLASH_BLOCK_SIZE - 1);
  unsigned program_size = (fw.size + (FLASH_PAGE_SIZE - 1)) & ~(FLASH_PAGE_SIZE - 1);
  __compiler_memory_barrier();
  // disable all interrupts
  save_and_disable_interrupts();

  // Commit any pending writes to external RAM, to avoid losing them in the subsequent flush
  xip_cache_clean_all();
  // No flash accesses after this point
  __compiler_memory_barrier();

  // actually do the flahing
  connect_internal_flash_func();
  flash_exit_xip_func();
  flash_range_erase_func(0, erase_size, FLASH_BLOCK_SIZE, FLASH_BLOCK_ERASE_CMD);
  flash_range_program_func(0, fw.buf, program_size);
  flash_flush_cache_func();

  // trigger a reboot, now
  // watchdog_enable(0,0,0)
  hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
  watchdog_hw->scratch[4] = 0;
  // watchdog_enable:_watchdog_enable(0,0)
  hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);
  hw_set_bits(&psm_hw->wdsel, PSM_WDSEL_BITS & ~(PSM_WDSEL_ROSC_BITS | PSM_WDSEL_XOSC_BITS));
  uint32_t dbg_bits = WATCHDOG_CTRL_PAUSE_DBG0_BITS |
                      WATCHDOG_CTRL_PAUSE_DBG1_BITS |
                      WATCHDOG_CTRL_PAUSE_JTAG_BITS;
  hw_clear_bits(&watchdog_hw->ctrl, dbg_bits);
  hw_set_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_TRIGGER_BITS);
}
