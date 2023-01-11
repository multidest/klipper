// Commands for sending messages to a 4-bit hd44780 lcd driver
//
// Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h" // CONFIG_MACH_AVR
#include "basecmd.h" // oid_alloc
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_from_us
#include "command.h" // DECL_COMMAND
#include "sched.h" // DECL_SHUTDOWN

struct hd44780 {
    uint32_t last_cmd_time, cmd_wait_ticks;
    uint8_t last;
    struct gpio_out rs, e, d0, d1, d2, d3, d4, d5, d6, d7;
};


/****************************************************************
 * Transmit functions
 ****************************************************************/

static uint32_t
nsecs_to_ticks(uint32_t ns)
{
    return timer_from_us(ns * 1000) / 1000000;
}

static inline void
ndelay(uint32_t nsecs)
{
    if (CONFIG_MACH_AVR)
        // Slower MCUs don't require a delay
        return;
    uint32_t end = timer_read_time() + nsecs_to_ticks(nsecs);
    while (timer_is_before(timer_read_time(), end))
        irq_poll();
}

// Write 8 bits to the hd44780 using the 8bit parallel interface
static __always_inline void
hd44780_xmit_bits(uint8_t data, struct gpio_out e, struct gpio_out d0
                  , struct gpio_out d1, struct gpio_out d2, struct gpio_out d3
                  , struct gpio_out d4, struct gpio_out d5, struct gpio_out d6
                  , struct gpio_out d7)
{
    ndelay(320000);
    gpio_out_write(d0, data & 0x01);
    gpio_out_write(d1, data & 0x02);
    gpio_out_write(d2, data & 0x04);
    gpio_out_write(d3, data & 0x08);
    gpio_out_write(d4, data & 0x10);
    gpio_out_write(d5, data & 0x20);
    gpio_out_write(d6, data & 0x40);
    gpio_out_write(d7, data & 0x80);
    gpio_out_write(e, 1);
    ndelay(5000);
    gpio_out_write(e, 0);
    ndelay(5000); // delay per char
}

// Transmit 8 bits to the chip
static void
hd44780_xmit_byte(struct hd44780 *h, uint8_t data)
{
    struct gpio_out e = h->e, d0 = h->d0, d1 = h->d1, d2 = h->d2, d3 = h->d3, d4 = h->d4, d5 = h->d5, d6 = h->d6, d7 = h->d7;
    hd44780_xmit_bits(data, e, d0, d1, d2, d3, d4, d5, d6, d7);
    ndelay(500);
}

// Transmit a series of bytes to the chip
static void
hd44780_xmit(struct hd44780 *h, uint8_t len, uint8_t *data)
{
    uint32_t last_cmd_time=h->last_cmd_time, cmd_wait_ticks=h->cmd_wait_ticks;
    while (len--) {
        uint8_t b = *data++;
        while (timer_read_time() - last_cmd_time < cmd_wait_ticks)
            irq_poll();
        hd44780_xmit_byte(h, b);
        last_cmd_time = timer_read_time();
    }
    h->last_cmd_time = last_cmd_time;
}


/****************************************************************
 * Interface
 ****************************************************************/

void
command_config_hd44780(uint32_t *args)
{
    struct hd44780 *h = oid_alloc(args[0], command_config_hd44780, sizeof(*h));
    h->d0 = gpio_out_setup(args[3], 1);
    h->d1 = gpio_out_setup(args[4], 1);
    h->d2 = gpio_out_setup(args[5], 1);
    h->d3 = gpio_out_setup(args[6], 1);
    h->d4 = gpio_out_setup(args[7], 1);
    h->d5 = gpio_out_setup(args[8], 1);
    h->d6 = gpio_out_setup(args[9], 1);
    h->d7 = gpio_out_setup(args[10], 1);
    h->rs = gpio_out_setup(args[1], 0);
    
    ndelay(5000); // Wait 5us before sending data
    h->e = gpio_out_setup(args[2], 0);

    if (!CONFIG_HAVE_STRICT_TIMING) {
        h->cmd_wait_ticks = args[11];
        return;
    }

    // Calibrate cmd_wait_ticks
    irq_disable();
    uint32_t start = timer_read_time();
    hd44780_xmit_byte(h, 0);
    uint32_t end = timer_read_time();
    irq_enable();
    uint32_t diff = end - start, delay_ticks = args[11];
    if (delay_ticks > diff)
        h->cmd_wait_ticks = delay_ticks - diff;
}
DECL_COMMAND(command_config_hd44780,
             "config_hd44780 oid=%c rs_pin=%u e_pin=%u"
             " d0_pin=%u d1_pin=%u d2_pin=%u d3_pin=%u"
             " d4_pin=%u d5_pin=%u d6_pin=%u d7_pin=%u delay_ticks=%u");

void
command_hd44780_send_cmds(uint32_t *args)
{
    struct hd44780 *h = oid_lookup(args[0], command_config_hd44780);
    gpio_out_write(h->rs, 0);
    ndelay(5000); // Wait 5us before sending data
    uint8_t len = args[1], *cmds = command_decode_ptr(args[2]);
    hd44780_xmit(h, len, cmds);
}
DECL_COMMAND(command_hd44780_send_cmds, "hd44780_send_cmds oid=%c cmds=%*s");

void
command_hd44780_send_data(uint32_t *args)
{
    struct hd44780 *h = oid_lookup(args[0], command_config_hd44780);
    gpio_out_write(h->rs, 1);
    ndelay(5000); // Wait 5us before sending data
    uint8_t len = args[1], *data = command_decode_ptr(args[2]);
    hd44780_xmit(h, len, data);
}
DECL_COMMAND(command_hd44780_send_data, "hd44780_send_data oid=%c data=%*s");

void
hd44780_shutdown(void)
{
    uint8_t i;
    struct hd44780 *h;
    foreach_oid(i, h, command_config_hd44780) {
        gpio_out_write(h->rs, 0);
        gpio_out_write(h->e, 0);
        gpio_out_write(h->d0, 0);
        gpio_out_write(h->d1, 0);
        gpio_out_write(h->d2, 0);
        gpio_out_write(h->d3, 0);
        gpio_out_write(h->d4, 0);
        gpio_out_write(h->d5, 0);
        gpio_out_write(h->d6, 0);
        gpio_out_write(h->d7, 0);
    }
}
DECL_SHUTDOWN(hd44780_shutdown);
