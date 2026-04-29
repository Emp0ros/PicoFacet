// SPDX-License-Identifier: GPL-2.0-only
// pins.h - CH32V305 pin definitions for PicoFacet
//
// SPI1 is used for the Arasan eMMC controller interface.
// Pin assignments (directly wired to the eMMC adapter):
//   PA5  = SPI_CLK  (SPI1_SCK)
//   PA6  = SPI_MISO (SPI1_MISO)
//   PA7  = SPI_MOSI (SPI1_MOSI)
//   PA4  = SPI_SS_N (GPIO, directly controlled)
//   PC4  = SMC_RST_N (GPIO)

#pragma once

#include "ch32v30x_gpio.h"

// SPI port and pins (directly on SPI1 default mapping)
#define SPI_PORT        GPIOA
#define SPI_CLK_PIN     GPIO_Pin_5
#define SPI_MISO_PIN    GPIO_Pin_6
#define SPI_MOSI_PIN    GPIO_Pin_7
#define SPI_SS_PORT     GPIOA
#define SPI_SS_PIN      GPIO_Pin_4

// SMC reset
#define SMC_RST_PORT    GPIOC
#define SMC_RST_PIN     GPIO_Pin_4

// Initialize GPIO pins
void pins_init(void);

// LED on PA3
void led_hw_init(void);
void led_hw_set(int val);

// Microsecond timer (TIM6-based)
void timer_init(void);
uint32_t timer_us(void);
