// SPDX-License-Identifier: GPL-2.0-only
// platform.h - RP2040 platform abstraction

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "pico/stdlib.h"

static inline void platform_init(void)
{
	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
}

static inline void led_set(int val)
{
	gpio_put(PICO_DEFAULT_LED_PIN, val);
}

#define USB_TX_MAX 64
