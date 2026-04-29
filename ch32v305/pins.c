// SPDX-License-Identifier: GPL-2.0-only
// pins.c - CH32V305 GPIO/SPI pin initialization, LED, and timer

#include "pins.h"
#include "ch32v30x.h"
#include "ch32v30x_rcc.h"
#include "ch32v30x_gpio.h"

void pins_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitTypeDef gpio = {0};

	// SMC_RST_N: push-pull output, default high
	gpio.GPIO_Pin = SMC_RST_PIN;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(SMC_RST_PORT, &gpio);
	GPIO_WriteBit(SMC_RST_PORT, SMC_RST_PIN, Bit_SET);

	// SPI_SS_N: push-pull output, default high
	gpio.GPIO_Pin = SPI_SS_PIN;
	GPIO_Init(SPI_SS_PORT, &gpio);
	GPIO_WriteBit(SPI_SS_PORT, SPI_SS_PIN, Bit_SET);
}

// ======================================================================
// LED on PA3
// ======================================================================

void led_hw_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef gpio = {0};
	gpio.GPIO_Pin = GPIO_Pin_3;
	gpio.GPIO_Mode = GPIO_Mode_Out_PP;
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &gpio);
}

void led_hw_set(int val)
{
	GPIO_WriteBit(GPIOA, GPIO_Pin_3, val ? Bit_SET : Bit_RESET);
}

// ======================================================================
// Microsecond timer using TIM6
// ======================================================================
// TIM6 is a basic 16-bit timer on APB1.
// APB1 prescaler = /2, so timer clock = SYSCLK (2x when APB divider > 1).
// PSC = (SYSCLK_MHz - 1) → 1 MHz tick.  Overflow every 65536 us.
// ISR extends to 32-bit software counter.

static volatile uint32_t timer_high;

void TIM6_IRQHandler(void) __attribute__((interrupt));
void TIM6_IRQHandler(void)
{
	TIM6->INTFR = ~(uint16_t)1; // clear UIF
	timer_high += 65536;
}

void timer_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	TIM6->PSC = SystemCoreClock / 1000000 - 1;
	TIM6->ATRLR = 0xFFFF;   // full 16-bit range
	TIM6->CNT = 0;
	TIM6->SWEVGR = 1;       // generate update event to load PSC
	TIM6->INTFR = 0;        // clear any pending flag
	TIM6->DMAINTENR = 1;    // UIE — update interrupt enable
	NVIC_EnableIRQ(TIM6_IRQn);
	TIM6->CTLR1 = 1;        // CEN — counter enable
}

uint32_t timer_us(void)
{
	uint32_t hi, lo;
	do {
		hi = timer_high;
		lo = TIM6->CNT;
	} while (hi != timer_high);
	return hi + lo;
}
