// SPDX-License-Identifier: GPL-2.0-only
// platform.h - CH32V305 platform abstraction

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "ch32v30x.h"

void led_hw_init(void);
void led_hw_set(int val);
void timer_init(void);
uint32_t timer_us(void);

static inline void platform_init(void)
{
	led_hw_init();
	timer_init();
}

static inline void led_set(int val)        { led_hw_set(val); }
static inline uint32_t time_us_32(void)    { return timer_us(); }
static inline void tight_loop_contents(void) { __NOP(); }

#define USB_TX_MAX 512
