#pragma once

#include <hardware/pio.h>
#include "config.h"

typedef struct pio_jtag_inst {
    PIO pio;
    uint sm;
    uint pin_tdi;
    uint pin_tdo;
    uint pin_tck;
    uint pin_tms;
    uint pin_rst;
} pio_jtag_inst_t;

typedef struct pio_a5clk_inst {
    PIO pio;
    uint sm;
    uint pin;
    int enabled;
} pio_a5clk_inst_t;

struct djtag_clk_s {
    uint32_t sys_khz;
    uint32_t jtag_divider;
    uint32_t jtag_khz;
    uint32_t a5clk_divider;
    uint32_t a5clk_khz;
    uint32_t a5clk_en;
};

void init_jtag(pio_jtag_inst_t* jtag, uint freq, uint pin_tck, uint pin_tdi, uint pin_tdo, uint pin_tms, uint pin_rst);
void pio_jtag_write_blocking(const pio_jtag_inst_t *jtag, const uint8_t *src, size_t len);
void pio_jtag_write_read_blocking(const pio_jtag_inst_t *jtag, const uint8_t *src, uint8_t *dst, size_t len);
uint8_t pio_jtag_write_tms_blocking(const pio_jtag_inst_t *jtag, bool tdi, bool tms, size_t len);
void jtag_set_clk_freq(const pio_jtag_inst_t *jtag, uint freq_khz);
void jtag_transfer(const pio_jtag_inst_t *jtag, uint32_t length, const uint8_t* in, uint8_t* out);
uint8_t jtag_strobe(const pio_jtag_inst_t *jtag, uint32_t length, bool tms, bool tdi);

void init_a5clk(pio_a5clk_inst_t* a5clk, uint freq, uint pin_a5clk);
void a5clk_set_freq(const pio_a5clk_inst_t *a5clk, uint freq_khz);
void a5clk_enable(const pio_a5clk_inst_t *a5clk);
void a5clk_disable(const pio_a5clk_inst_t *a5clk);


static inline void jtag_set_tms(const pio_jtag_inst_t *jtag, bool value)
{
  gpio_put(jtag->pin_tms, value);
}
static inline void jtag_set_rst(const pio_jtag_inst_t *jtag, bool value)
{
  /* Change the direction to out to drive pin to 0 or to in to emulate open drain */
//  gpio_set_dir(jtag->pin_rst, !value);
  gpio_put(jtag->pin_rst, value);
}
static inline void jtag_set_trst(const pio_jtag_inst_t *jtag, bool value)
{
  // no TRST on E1 board
}

// The following APIs assume that they are called in the following order:
// jtag_set_XXX where XXX is any pin. if XXX is clk it needs to be false
// jtag_set_clk where clk is true This will initiate a one cycle pio_jtag_write_read_blocking
// possibly jtag_get_tdo which will get what was read during the previous step
void jtag_set_tdi(const pio_jtag_inst_t *jtag, bool value);

void jtag_set_clk(const pio_jtag_inst_t *jtag, bool value);

bool jtag_get_tdo(const pio_jtag_inst_t *jtag);
