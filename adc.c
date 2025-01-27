#include <stdio.h>
#include <string.h>

#include <pico/binary_info.h>
#include <hardware/gpio.h>
#include <hardware/spi.h>

#include "config.h"
#include "adc.h"
#include "utils.h"

//#define DEBUG

volatile char adc_busy;
#ifdef SPI_PARANOIA
volatile char adc_ss;
#endif
short adc_addr;
int adc_spi_speed;

// size of every register, in bits (for ADCDATA, it can be 1/2/4; for CRC, it can be 2/4)
// masks will be applied before writing (and_mask of 0 means, read-only)
// we mark the reserved regs as r/o, even if they're technically r/w
// we also cache the last seen value (read or written)
static struct {
  unsigned size;
  const unsigned and_mask, or_mask;
  unsigned last_val;
} adc_regs[16] = {
  [ADCR_ADCDATA]   = { 4, 0, 0 }, // R/O
  [ADCR_CONFIG0]   = { 1, 0xff, 0x00 },
  [ADCR_CONFIG1]   = { 1, 0xfc, 0x00 }, // cfg1[1:0] reserved, set to 2'b00
  [ADCR_CONFIG2]   = { 1, 0xfe, 0x01 }, // cfg2[0] reserved, set to 1'b1
  [ADCR_CONFIG3]   = { 1, 0xff, 0x00 },
  [ADCR_IRQ]       = { 1, 0x0f, 0x00 }, // irq: only bits [3:0] are r/w
  [ADCR_MUX]       = { 1, 0xff, 0x00 },
  [ADCR_SCAN]      = { 3, 0xe0ffff, 0 }, // scan[20] reserved, set to 1'b0; scan[19:16] unimplemented
  [ADCR_TIMER]     = { 3, 0xffffff, 0 },
  [ADCR_OFFSETCAL] = { 3, 0xffff00, 0 }, // offsetcal[7:0] unimplemented
  [ADCR_GAINCAL]   = { 3, 0xffff00, 0 }, // gaincal[7:0] unimplemented
  [0xB]            = { 3, 0, 0 }, // reserved
  [0xC]            = { 1, 0, 0 }, // reserved
  [ADCR_LOCK]      = { 1, 0xff, 0x00 },
  [0xE]            = { 2, 0, 0 }, // reserved
  [ADCR_CRCCFG]    = { 2, 0, 0 } // R/O
};
#define ADC_REGMAP_SIZE_NO_CRC_NO_DATA (0+1+1+1+1+1+1+3+3+3+3+3+1+1+2+0)
#define ADC_REGMAP_SIZE_MAX (4+1+1+1+1+1+1+3+3+3+3+3+1+1+2+4)
static unsigned adc_regmap_size = 0;

// n_scan_ch is the population count of the SCAN register
//   (set when SCAN is written)
// scan_ch_id is set when the scan register selects exactly 1 channel, -1
//   otherwise (set when SCAN is written)
// have_chid is set when, in scanning mode, 32-bit with channel id mode is enabled
//   (set upon writes to CONFIG2 and/or IRQ)
// adc_dsize is set whenever the data format changes
//   (set upon writes to CONFIG2 and/or IRQ)
static char n_scan_ch = 0, first_scan_ch = 16;
static char adc_dsize = 0;

static uint8_t adc_cmds[ADC_REGMAP_SIZE_MAX], adc_data[ADC_REGMAP_SIZE_MAX];

//=[ detect/initialize the ADC ]==============================================

static int adc_probe();

static void _assert_adc_cs(const char *fn)
{
  (void)fn;
# ifdef SPI_PARANOIA
  if (! adc_ss)
    printf("P: in %s(): ADC CS# already asserted!!!\n", fn);
# endif
  gpio_put(PIN_ADC_SSn, 0);
# ifdef SPI_PARANOIA
  adc_ss = 0;
# endif
}
#define assert_adc_cs() _assert_adc_cs(__FUNCTION__)

static void _deassert_adc_cs(const char *fn)
{
  (void)fn;
# ifdef SPI_PARANOIA
  if (adc_ss)
    printf("P: in %s(): ADC CS# not yet asserted!!!\n", fn);
# endif
  gpio_put(PIN_ADC_SSn, 1);
# ifdef SPI_PARANOIA
  adc_ss = 1;
# endif
}
#define deassert_adc_cs() _deassert_adc_cs(__FUNCTION__)

void _check_adc_state(const char *fn)
{
  (void)fn;
# ifdef PARANOID
  if (! adc_busy)
    printf("P: %s() called while !adc_busy\n", fn);
  // it's ok to have eth_busy - it an "hey, I want it" announcement, not a "hey, I have it" statement
  //if (eth_busy)
  //  printf("P: %s() called while eth_busy\n", fn);
  if (adc_ss)
    printf("P: %s() called while ADC CS# is not yet active\n", fn);
  if (! eth_ss)
    printf("P: %s() called while ETH CS# is active\n", fn);
# endif
}
#define check_adc_state() _check_adc_state(__FUNCTION__)

// sets adc_spi_speed to 0, if device not present
void adc_init()
{
  // configure CS#
  gpio_init(PIN_ADC_SSn);
  gpio_put(PIN_ADC_SSn, 1); // initially de-selected
  gpio_set_dir(PIN_ADC_SSn, GPIO_OUT);
  deassert_adc_cs();
  bi_decl(bi_1pin_with_name(PIN_ADC_SSn, "ADC_SS#"));
  // configure SPI itself - mode 0
  spi_init(SPI_ADC, FREQ_ADC_KHZ * 1000);
  spi_set_format (SPI_ADC, 8, 0, 0, 0);
  // configure the SPI pins
  gpio_set_function(PIN_ADC_SCK, GPIO_FUNC_SPI);
  gpio_set_function(PIN_ADC_MOSI, GPIO_FUNC_SPI);
  gpio_set_function(PIN_ADC_MISO, GPIO_FUNC_SPI);
  bi_decl(bi_3pins_with_func(PIN_ADC_MISO, PIN_ADC_MOSI, PIN_ADC_SCK, GPIO_FUNC_SPI));
  adc_spi_speed = spi_get_baudrate(SPI_ADC);

  // perform detection
  if (adc_probe() < 0) {
    //printf("ADC does NOT work at %u.%uMHz!\n", (adc_spi_speed+500)/1000000, ((adc_spi_speed+500)%1000000)/1000);
    adc_spi_speed = 0;
    return;
  }

  // perform initial configuration
  claim_spi_for_adc();

  // - make sure register writes are unlocked
  adc_cmds[0] = (adc_addr << 6) | ADC_DO_WRITE_BURST(ADCR_LOCK);
  adc_cmds[1] = adc_regs[ADCR_LOCK].last_val = 
    ADC_LOCK_MAGIC;
  assert_adc_cs();
  spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 2);
  deassert_adc_cs();
# ifdef DEBUG
  printf("ADC unlock: %02X.%02X -> %02X.%02X\n", adc_cmds[0], adc_cmds[1], adc_cmds[0], adc_cmds[1]);
# endif

  // setup the CONFIG0..GAINCAL regs, inclusive
  adc_cmds[0] = (adc_addr << 6) | ADC_DO_WRITE_BURST(ADCR_CONFIG0);
  // set mode to defaults + standby, don't output MCLK
  adc_cmds[1] = adc_regs[ADCR_CONFIG0].last_val =
    /*ADC_CFG0_VREFSEL_INTERNAL |*/ ADC_CFG0_CLKSEL_INT_NOCLK | 
    ADC_CFG0_CSRC_NONE | ADC_CFG0_MODE_STANDBY; 
  adc_cmds[2] = adc_regs[ADCR_CONFIG1].last_val = 
    ADC_CFG1_PRE_NONE | ADC_CFG1_OSR_DEFAULT; // use defaults
  adc_cmds[3] = adc_regs[ADCR_CONFIG2].last_val = 0x01 | // CFG2[0] needs to be 1 at all times
    ADC_CFG2_BOOST_NONE | ADC_CFG2_GAIN_NONE; // use defaults
  // defaults + cause one-shots to return to standby, data format: max
  // (defaults include, crc disabled and crc set to 16bit)
  adc_cmds[4] = adc_regs[ADCR_CONFIG3].last_val = 
    ADC_CFG3_CMODE_ONESHOT_STANDBY | ADC_CFG3_DFMT_32BIT_SGNX_CHID;
  // defaults + disable conversion start interrupt output
  adc_cmds[5] = adc_regs[ADCR_IRQ].last_val =
    ADC_IRQ_FASTCMD_EN; // no ADC_IRQ_INACTIVE_HIGH_EN, since that's pulled up
  // measure the offset once
  adc_cmds[6] = adc_regs[ADCR_MUX].last_val = ADC_CHID_MUX_OFFSET;
  // reset the SCAN, TIMER, OFFSETCAL and GAINCAL regs
  memset(adc_cmds+7, 0, 4*3);
  adc_regs[ADCR_SCAN].last_val = 0;
  adc_regs[ADCR_TIMER].last_val = 0;
  adc_regs[ADCR_OFFSETCAL].last_val = 0;
  adc_regs[ADCR_GAINCAL].last_val = 0;
  // perform the initial programming
  assert_adc_cs(); check_adc_state();
  spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 7+12);
  deassert_adc_cs();
# if 0
  // sample the offset
  int offset = 0;
  do_adc_conv((uint8_t*)&offset, ADC_CHID_OFSFET);
  printf("ADC offset: (%u) %d\n", (unsigned)offset >> 28, (offset << 17) >> 17 );
# endif

  // we're done with the SPI for now
  release_adc_spi();

  // update internal state
  adc_regmap_size = ADC_REGMAP_SIZE_NO_CRC_NO_DATA + 4 + 2;
  n_scan_ch  = 0; first_scan_ch = 16; // SCAN mode disabled initially
  adc_dsize  = 17; // data in initial format is 17bit signed
# ifdef DEBUG
  printf("ADC config: %02X.%02X%02X%02X%02X:%02X:%02X:0000:0000:0000:0000 -> %02X....\n",
         adc_cmds[0], adc_cmds[1], adc_cmds[2], adc_cmds[3], adc_cmds[4], adc_cmds[5], adc_cmds[6],
         adc_data[0]);
# endif
}

//-[ adc_probe() ]------------------------------------------------------------

int adc_probe()
{
  // the ADC can have one of 4 addresses, which'll get sent in reply to 
  // the 1st byte
  adc_addr = -1;
  if (adc_spi_speed < 1000)
    return -1;
  claim_spi_for_adc();
  // attempt all 4 possible addresses with a dummy command
  // to the command word, the response will be
  // {2'b0, cmd[7:6], ~cmd[6], data_ready#, crc_en#, por_int#}
  uint8_t expected_resp, cmp_mask = 0x38;
  for (unsigned addr = 0; addr < 4; ++addr) {
    adc_cmds[0] = (addr<<6)|ADC_DO_READ(0);
    expected_resp = addr << 4;
    if (! (addr & 1))
      expected_resp |= 1 << 3;
    adc_data[0] = 0xff;
    assert_adc_cs(); check_adc_state();
    spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 1);
    deassert_adc_cs();
#   ifdef DEBUG
    printf("ADC: addr %u: cmd=%02X resp=%02X exp_resp=%02X|mask=%02X\n", addr, 
      adc_cmds[0], adc_data[0], expected_resp, cmp_mask);
#   endif
    if ((adc_data[0] & cmp_mask) == expected_resp) {
      adc_addr = addr;
      break;
    }
  }
  release_adc_spi();
  return adc_addr;
}

//=[ read all the ADC registers ]=============================================

static struct adcregs_s adc_reg_vals;

int do_adc_get_regs(uint8_t *dest) {
  // do a readall of all registers, starting with register 1 (CONFIG0), finishing with ADCDATA
  // build a DO_READ_BURST starting with 1 (CONFIG0)
  memset (adc_cmds, 0xff, sizeof(adc_cmds));
  adc_cmds[0] = (adc_addr << 6) | ADC_DO_READ_BURST(ADCR_CONFIG0);
  // get the registers
  assert_adc_cs(); check_adc_state();
  spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, adc_regmap_size);
  deassert_adc_cs();
  adc_busy = 0;
  uint8_t *pdata = &adc_data[0];
  adc_reg_vals.cmd       = adc_cmds[0];
  adc_reg_vals.cmd_resp  = *pdata++;
  adc_regs[ADCR_CONFIG0].last_val   = adc_reg_vals.config[0] = *pdata++;
  adc_regs[ADCR_CONFIG1].last_val   = adc_reg_vals.config[1] = *pdata++;
  adc_regs[ADCR_CONFIG2].last_val   = adc_reg_vals.config[2] = *pdata++;
  adc_regs[ADCR_CONFIG3].last_val   = adc_reg_vals.config[3] = *pdata++;
  adc_regs[ADCR_IRQ].last_val       = adc_reg_vals.irq       = *pdata++;
  adc_regs[ADCR_MUX].last_val       = adc_reg_vals.mux       = *pdata++;
  adc_regs[ADCR_SCAN].last_val      = adc_reg_vals.scan      = ((uint32_t)pdata[0] << 16) | ((uint32_t)pdata[1] << 8) | pdata[2]; pdata += 3;
  adc_regs[ADCR_TIMER].last_val     = adc_reg_vals.timer     = ((uint32_t)pdata[0] << 16) | ((uint32_t)pdata[1] << 8) | pdata[2]; pdata += 3;
  adc_regs[ADCR_OFFSETCAL].last_val = adc_reg_vals.offsetcal = ((uint32_t)pdata[0] << 16) | ((uint32_t)pdata[1] << 8) | pdata[2]; pdata += 3;
  adc_regs[ADCR_GAINCAL].last_val   = adc_reg_vals.gaincal   = ((uint32_t)pdata[0] << 16) | ((uint32_t)pdata[1] << 8) | pdata[2]; pdata += 3;
  adc_regs[0xB].last_val            = ((uint32_t)pdata[0] << 16) | ((uint32_t)pdata[1] << 8) | pdata[2]; pdata += 3;
  adc_regs[0xC].last_val            = *pdata++;
  adc_regs[ADCR_LOCK].last_val      = *pdata++;
  adc_regs[0xE].last_val            = ((uint32_t)pdata[0] << 8) | pdata[1]; pdata += 2;
  adc_regs[ADCR_CRCCFG].last_val    = ((uint32_t)pdata[0] << 8) | pdata[1]; pdata += 2;
  if (adc_regs[ADCR_CRCCFG].size == 4) {
    adc_regs[ADCR_CRCCFG].last_val  = (adc_regs[ADCR_CRCCFG].last_val << 16) |
                                      ((uint32_t)pdata[0] << 8) | pdata[1];
    pdata += 2;
  }
  adc_regs[ADCR_ADCDATA].last_val   = ((uint32_t)pdata[0] << 8) | pdata[1]; pdata += 2;
  if (adc_regs[ADCR_ADCDATA].size == 4) {
    adc_regs[ADCR_ADCDATA].last_val = (adc_regs[ADCR_ADCDATA].last_val << 16) |
                                      ((uint32_t)pdata[0] << 8) | pdata[1];
    pdata += 2;
  }
# ifdef DEBUG
  printf("ADC regs: >%02X <%02X: {%02X %02X %02X %02X, %02X %02X, %06X %06X %06X %06X (%06X %02X) %02X (%02X) %04X %08X}\n",
          adc_reg_vals.cmd, adc_reg_vals.cmd_resp, 
          adc_reg_vals.config[0], adc_reg_vals.config[1], adc_reg_vals.config[2], adc_reg_vals.config[3], 
          adc_reg_vals.irq, adc_reg_vals.mux, 
          adc_reg_vals.scan, adc_reg_vals.timer, adc_reg_vals.offsetcal, adc_reg_vals.gaincal,
          adc_regs[0xB].last_val, adc_regs[0xC].last_val,
          adc_reg_vals.lock,
          adc_regs[0xE].last_val,
          adc_reg_vals.crccfg, adc_regs[ADCR_ADCDATA].last_val);
# endif
  // if a destination was provided
  if (dest) {
    // compile the response
    adc_reg_vals.adcdata   = adc_regs[ADCR_ADCDATA].last_val;
    adc_reg_vals.lock      = adc_regs[ADCR_LOCK].last_val;
    adc_reg_vals.crccfg    = adc_regs[ADCR_CRCCFG].last_val;
    memcpy(dest, &adc_reg_vals, sizeof(adc_reg_vals));
  }
  return sizeof(adc_reg_vals);
}

//=[ set a single ADC register ]==============================================

static inline void adc_scan_updated(unsigned value)
{
  n_scan_ch = 0;
  first_scan_ch = 16;
  for (int i = 15; i >= 0; --i) {
    if (value & (1 << i)) {
      ++n_scan_ch;
      if (first_scan_ch == 16)
          first_scan_ch = i;
    }
  }
  //printf("adc scan %04x n_ch=%d first_ch=%d\n", value, n_scan_ch, first_scan_ch);
}

static inline void adc_cfg3_irq_updated(unsigned cfg3, unsigned irq)
{
  // update size of CRCCFG depending on cfg3
  adc_regs[ADCR_CRCCFG].size = 2;
  if (cfg3 & ADC_CFG3_CRCFMT_32BIT)
    adc_regs[ADCR_CRCCFG].size = 4;
  // update format (including ADCDATA size) depending on cfg3 and irq
  adc_dsize = 17;
  adc_regs[ADCR_ADCDATA].size = 4;
  if (irq & ADC_IRQ_PIN_MDAT_EN) {
    adc_regs[ADCR_ADCDATA].size = 1;
    adc_dsize = 4;
  }
  else if ((cfg3 & ADC_CFG3_DFMT_MASK) == ADC_CFG3_DFMT_16BIT) {
    adc_regs[ADCR_ADCDATA].size = 2;
    adc_dsize = 16;
  }
}

int do_adc_set_reg(int which, uint32_t value)
{
  which &= 0xf;
  // don't support writing to reserved/ro registers
  if (! adc_regs[which].and_mask)
    return 0;
  // unlock register writes, if disabled
  if (adc_regs[ADCR_LOCK].last_val != ADC_LOCK_MAGIC) {
    adc_cmds[0] = (adc_addr << 6) | ADC_DO_WRITE_BURST(ADCR_LOCK);;
    adc_cmds[1] = ADC_LOCK_MAGIC;
    assert_adc_cs(); check_adc_state();
    spi_write_blocking(SPI_ADC, adc_cmds, 2);
    deassert_adc_cs();
    adc_regs[ADCR_LOCK].last_val = ADC_LOCK_MAGIC;
  }
  // fixup&cache the value
  value = (value & adc_regs[which].and_mask) | adc_regs[which].or_mask;
  adc_regs[which].last_val = value;
  // perform the write and update the cached value
  uint8_t *p = &adc_cmds[0];
  *p++ = (adc_addr << 6) | ADC_DO_WRITE_BURST(which);
  // write the bytes is MSB order
# if 0
  // no writable 32-bit registers
  if (adc_regs[which].size >= 4) 
    *p++ = (value >> 24) & 0xFF;
# endif
  if (adc_regs[which].size >= 3)
    *p++ = (value >> 16) & 0xFF;
  if (adc_regs[which].size >= 2)
    *p++ = (value >> 8) & 0xFF;
  *p++ = value & 0xff;
  assert_adc_cs(); check_adc_state();
  spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 1+adc_regs[which].size);
  deassert_adc_cs();
  // writes to CONFIG3/IRQ could potentially update the size of CRCCFG, as well
  // as the size of ADCDATA
  if ((which == ADCR_CONFIG3) || (which == ADCR_IRQ)) {
    if (which == ADCR_CONFIG3)
      adc_cfg3_irq_updated(value, adc_regs[ADCR_IRQ].last_val);
    else if (which == ADCR_IRQ)
      adc_cfg3_irq_updated(adc_regs[ADCR_CONFIG3].last_val, value);
    adc_regmap_size = ADC_REGMAP_SIZE_NO_CRC_NO_DATA +
                      adc_regs[ADCR_CRCCFG].size +
                      adc_regs[ADCR_ADCDATA].size;
  }
  // writes to SCAN could change its population count and first channel
  if (which == ADCR_SCAN) 
    adc_scan_updated(value);
}

//=[ ADC conversions ]========================================================

int do_adc_chread_scan(uint8_t *dest, unsigned bmp)
{
  // clear IRQ_MODE[1] if set
  if (adc_dsize < 16)
    do_adc_set_reg(ADCR_IRQ, adc_regs[ADCR_IRQ].last_val & ~ADC_IRQ_PIN_MDAT_EN);
  // reprogram SCAN, if it had changed
  if (adc_regs[ADCR_SCAN].last_val != bmp)
    do_adc_set_reg(ADCR_SCAN, bmp);
  // do the conversion(s)
  return do_adc_conv(dest);
}

//----------------------------------------------------------------------------

int do_adc_chread_mux(uint8_t *dest, unsigned mux)
{
  // clear IRQ_MODE[1] if set
  if (adc_dsize < 16)
    do_adc_set_reg(ADCR_IRQ, adc_regs[ADCR_IRQ].last_val & ~ADC_IRQ_PIN_MDAT_EN);
  // clear SCAN channel map, if not empty
  // (SCAN mode overrides MUX mode)
  if (adc_regs[ADCR_SCAN].last_val & 0xffff)
    do_adc_set_reg(ADCR_SCAN, adc_regs[ADCR_SCAN].last_val & ~0xffff);
  // update MUX, if different from previous
  if (adc_regs[ADCR_MUX].last_val != mux)
    do_adc_set_reg(ADCR_MUX, mux);
  // do the conversion
  return do_adc_conv(dest);
}

//----------------------------------------------------------------------------

static const uint8_t channel_mux_config[16], mux2ch[256];

// keep a record of the most recently read samples, per channel
// channel 16 is for data we can't otherwise trace
static int last_adc_sample[17];

int do_adc_conv(uint8_t *dest)
{
  // issue a conversion
  adc_cmds[0] = (adc_addr << 6) | ADC_DO_CONV_START;
  assert_adc_cs(); check_adc_state();
  spi_write_blocking(SPI_ADC, adc_cmds, 1);
  deassert_adc_cs();
  // fetch the responses
  adc_cmds[0] = (adc_addr << 6) | ADC_DO_READ(ADCR_ADCDATA);
  int n_chan = 1;
  unsigned scan = adc_regs[ADCR_SCAN].last_val;
  unsigned mux = adc_regs[ADCR_MUX].last_val;
  if (n_scan_ch)
    n_chan = n_scan_ch;
  int scan_ch = first_scan_ch;
  int next_scan_ch = scan_ch - 1;
  for (int i = 0; i < n_chan; ++i) {
    // in scan mode, scan_ch is the channel ID of the next sample
    // find out which if the next channel id will be
    if (n_scan_ch)
      while ((next_scan_ch >= 0) && ! (scan & (1 << next_scan_ch )))
        --next_scan_ch;
    // poll until the next result is available
    do {
      assert_adc_cs(); check_adc_state();
      spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 1);
      deassert_adc_cs();
    } while (adc_data[0] & (1<<2));
    // actually fetch the data
    assert_adc_cs(); check_adc_state();
    spi_write_read_blocking(SPI_ADC, adc_cmds, adc_data, 1+adc_regs[ADCR_ADCDATA].size);
    deassert_adc_cs();
    // record the result
    int data;
    if (adc_regs[ADCR_ADCDATA].size == 1)
      data = 0xdead0000u | adc_data[1]; // no padding in this case
    else {
      if (adc_regs[ADCR_ADCDATA].size == 2)
        // align result as if it had been 17-bit sign-extended data
        data = ((adc_data[1] & 0x80) ? 0xfffe0000u : 0u) |
               ((unsigned)adc_data[1] << 9) | ((unsigned)adc_data[2] << 1);
      else
        data = ((unsigned)adc_data[1] << 24) | ((unsigned)adc_data[2] << 16) |
               ((unsigned)adc_data[3] << 8 ) | adc_data[4];
      // in MUX mode, patch in the MUX config
      // in SCAN mode, patch in the MUX config corresponding to the channel
      if (n_scan_ch)
        mux = channel_mux_config[scan_ch];
      data = (data & ~0xff000000) | ((unsigned)mux << 24);
    }
    // in scan mode, load in the ID of the next channel
    if (n_scan_ch) {
        scan_ch = next_scan_ch;
        --next_scan_ch;
    }
    // log the read to the appropriate channel
    // "channel" 16 contains the most recently read sample
    if (mux2ch[mux])
      last_adc_sample[mux2ch[mux]] = data;
    last_adc_sample[16] = data;
    // don't even THINK about printing stuff in this loop - you WILL miss conversion
    // samples and end up with the for loop not finishing
    *dest++ = (data >> 0 ) & 0xff;
    *dest++ = (data >> 8 ) & 0xff;
    *dest++ = (data >> 16) & 0xff;
    *dest++ = (data >> 24) & 0xff;
  }
  return 4 * n_chan;
}
static const uint8_t channel_mux_config[16] = {
  ADC_CHID_MUX_CH(0),   ADC_CHID_MUX_CH(1),   ADC_CHID_MUX_CH(2),   ADC_CHID_MUX_CH(3),
  ADC_CHID_MUX_CH(4),   ADC_CHID_MUX_CH(5),   ADC_CHID_MUX_CH(6),   ADC_CHID_MUX_CH(7),
  ADC_CHID_MUX_DIFF(0), ADC_CHID_MUX_DIFF(1), ADC_CHID_MUX_DIFF(2), ADC_CHID_MUX_DIFF(3),
  ADC_CHID_MUX_TEMP,    ADC_CHID_MUX_AVDD,    ADC_CHID_MUX_AVCM,    ADC_CHID_MUX_OFFSET
};
static const uint8_t mux2ch[256] = {
  [ADC_CHID_MUX_CH(0)]   = 1+ADC_CHID_CH(0),
  [ADC_CHID_MUX_CH(1)]   = 1+ADC_CHID_CH(1),
  [ADC_CHID_MUX_CH(2)]   = 1+ADC_CHID_CH(2),
  [ADC_CHID_MUX_CH(3)]   = 1+ADC_CHID_CH(3),
  [ADC_CHID_MUX_CH(4)]   = 1+ADC_CHID_CH(4),
  [ADC_CHID_MUX_CH(5)]   = 1+ADC_CHID_CH(5),
  [ADC_CHID_MUX_CH(6)]   = 1+ADC_CHID_CH(6),
  [ADC_CHID_MUX_CH(7)]   = 1+ADC_CHID_CH(7),
  [ADC_CHID_MUX_DIFF(0)] = 1+ADC_CHID_DIFF(0),
  [ADC_CHID_MUX_DIFF(1)] = 1+ADC_CHID_DIFF(1),
  [ADC_CHID_MUX_DIFF(2)] = 1+ADC_CHID_DIFF(2),
  [ADC_CHID_MUX_DIFF(3)] = 1+ADC_CHID_DIFF(3),
  [ADC_CHID_MUX_TEMP]    = 1+ADC_CHID_TEMP,
  [ADC_CHID_MUX_AVDD]    = 1+ADC_CHID_AVDD,
  [ADC_CHID_MUX_AVCM]    = 1+ADC_CHID_AVCM,
  [ADC_CHID_MUX_OFFSET]  = 1+ADC_CHID_AVDD
};
