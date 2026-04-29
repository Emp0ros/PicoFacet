// SPDX-License-Identifier: GPL-2.0-only
// spi_reg.c - CH32V305 SPI register access
//
// 10-bit header is bit-banged (2-bit cmd + 8-bit addr, LSB-first).
// Data phase uses hardware SPI1 in 8-bit LSB-first mode with DMA.

#include "spi_reg.h"
#include "pins.h"
#include "platform.h"

#include "ch32v30x.h"
#include "ch32v30x_spi.h"
#include "ch32v30x_dma.h"
#include "ch32v30x_rcc.h"
#include "ch32v30x_gpio.h"

// SPI clock divider — SPI1 is on APB2 (144 MHz)
// Init must be slow — RP2040 uses 5 KHz. Max prescaler is 256 → 562 KHz.
// Bit-bang delay compensates to bring effective rate down further.
#define SPI_INIT_PRESCALER    SPI_BaudRatePrescaler_256  // ~562 KHz

static uint16_t spi_prescaler = SPI_INIT_PRESCALER;
static uint32_t spi_divider = 256;  // actual divider for bit-bang delay

// DMA1 Channel3 = SPI1_TX, Channel2 = SPI1_RX
#define SPI_DMA         DMA1
#define SPI_DMA_TX_CH   DMA1_Channel3
#define SPI_DMA_RX_CH   DMA1_Channel2

static volatile int dma_busy;
static const uint8_t dma_tx_ff = 0xFF;

// ======================================================================
// Bit-bang header
// ======================================================================

// Pin control helpers — direct register access for speed
static inline void sck_low(void)  { SPI_PORT->BCR = SPI_CLK_PIN; }
static inline void sck_high(void) { SPI_PORT->BSHR = SPI_CLK_PIN; }
static inline void mosi_low(void) { SPI_PORT->BCR = SPI_MOSI_PIN; }
static inline void mosi_high(void){ SPI_PORT->BSHR = SPI_MOSI_PIN; }
static inline void cs_low(void)   { SPI_SS_PORT->BCR = SPI_SS_PIN; }
static inline void cs_high(void)  { SPI_SS_PORT->BSHR = SPI_SS_PIN; }

// Fast pin mode switching via direct CFGLR register writes.
// Only SCK (pin 5) and MOSI (pin 7) change mode; MISO (pin 6) stays floating input.
// CFGLR nibble layout: pin5 = bits[23:20], pin7 = bits[31:28]
//   GPIO out PP 50 MHz: MODE=0x3 CNF=0x0 → nibble 0x3
//   AF out PP 50 MHz:   MODE=0x3 CNF=0x2 → nibble 0xB
#define PIN57_MASK      0xF0F00000u
#define PIN57_GPIO_PP   0x30300000u
#define PIN57_AF_PP     0xB0B00000u

static inline void spi_pins_gpio(void)
{
	SPI_PORT->CFGLR = (SPI_PORT->CFGLR & ~PIN57_MASK) | PIN57_GPIO_PP;
}

static inline void spi_pins_af(void)
{
	SPI_PORT->CFGLR = (SPI_PORT->CFGLR & ~PIN57_MASK) | PIN57_AF_PP;
}

static void spi_hw_init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	SPI_InitTypeDef spi = {0};
	spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	spi.SPI_Mode = SPI_Mode_Master;
	spi.SPI_DataSize = SPI_DataSize_8b;
	spi.SPI_CPOL = SPI_CPOL_Low;
	spi.SPI_CPHA = SPI_CPHA_1Edge;
	spi.SPI_NSS = SPI_NSS_Soft;
	spi.SPI_BaudRatePrescaler = spi_prescaler;
	spi.SPI_FirstBit = SPI_FirstBit_LSB;
	SPI_Init(SPI1, &spi);
	// SSI must be high for master mode with software NSS, otherwise
	// a mode fault resets MSTR and SPI falls back to slave mode.
	SPI1->CTLR1 |= SPI_CTLR1_SSI;
	SPI_Cmd(SPI1, ENABLE);
}

// Precomputed bit-bang delay count (updated by spi_reg_set_frequency and init)
static uint32_t bitbang_delay_n;

// Delay one SPI half-clock period
static inline void bitbang_delay(void)
{
	volatile uint32_t n = bitbang_delay_n;
	while (n--)
		;
}

// Bit-bang 10-bit header, LSB-first. CPHA=0: data sampled on rising edge.
// SCK must be low at entry. Pins must be in GPIO mode.
static inline void bitbang_header(uint16_t header)
{
	for (int i = 0; i < 10; i++) {
		if (header & 1)
			mosi_high();
		else
			mosi_low();
		bitbang_delay();
		sck_high();
		bitbang_delay();
		sck_low();
		header >>= 1;
	}
}

// Bit-bang one byte, LSB-first, full-duplex. Returns MISO byte.
// CPHA=0: slave sets MISO on falling edge, master samples before rising edge.
static uint8_t bitbang_xfer_byte(uint8_t tx)
{
	uint8_t rx = 0;
	for (int i = 0; i < 8; i++) {
		if (tx & 1)
			mosi_high();
		else
			mosi_low();
		tx >>= 1;
		bitbang_delay();
		// Sample MISO — data was set up on previous falling edge
		if (SPI_PORT->INDR & SPI_MISO_PIN)
			rx |= (1 << i);
		sck_high();
		bitbang_delay();
		sck_low();
	}
	return rx;
}

// When HW SPI prescaler can't go slow enough, bit-bang entire transaction
static bool use_bitbang_data;

// Begin transaction: CS low, bit-bang header.
// In slow mode, stays in GPIO mode for data. In fast mode, switches to HW SPI.
static void send_header(uint16_t header)
{
	SPI_Cmd(SPI1, DISABLE);
	spi_pins_gpio();
	sck_low();
	cs_low();
	bitbang_header(header);
	if (!use_bitbang_data) {
		spi_pins_af();
		SPI1->CTLR1 |= SPI_CTLR1_SSI;
		SPI_Cmd(SPI1, ENABLE);
	}
}

static void end_transaction(void)
{
	cs_high();
	if (use_bitbang_data) {
		// Return to AF mode so HW SPI is ready for next fast transaction
		spi_pins_af();
		SPI1->CTLR1 |= SPI_CTLR1_SSI;
		SPI_Cmd(SPI1, ENABLE);
	}
}

// ======================================================================
// Blocking SPI byte transfer (used for small transfers)
// ======================================================================

static uint8_t spi_xfer_byte(uint8_t tx)
{
	while (!(SPI1->STATR & SPI_I2S_FLAG_TXE))
		;
	SPI1->DATAR = tx;
	while (!(SPI1->STATR & SPI_I2S_FLAG_RXNE))
		;
	return (uint8_t)SPI1->DATAR;
}

// Drain any leftover RX data
static void spi_drain_rx(void)
{
	while (SPI1->STATR & SPI_I2S_FLAG_RXNE)
		(void)SPI1->DATAR;
}

// ======================================================================
// Public API
// ======================================================================

void spi_reg_init(void)
{
	use_bitbang_data = true;  // start slow — HW SPI can't go below 562 KHz
	bitbang_delay_n = SystemCoreClock / (2 * 5000 * 4);
	pins_init();
	spi_hw_init();
	spi_pins_af();
}

void spi_reg_read(uint8_t addr, uint8_t *dst, size_t size)
{
	uint16_t header = 0x01 | ((uint16_t)addr << 2);
	send_header(header);

	if (use_bitbang_data) {
		for (size_t i = 0; i < 2; i++)
			(void)bitbang_xfer_byte(0xFF);
		for (size_t i = 0; i < size; i++)
			dst[i] = bitbang_xfer_byte(0xFF);
	} else {
		spi_drain_rx();
		for (size_t i = 0; i < 2; i++)
			(void)spi_xfer_byte(0xFF);
		for (size_t i = 0; i < size; i++)
			dst[i] = spi_xfer_byte(0xFF);
		while (SPI1->STATR & SPI_I2S_FLAG_BSY)
			;
	}
	end_transaction();
}

void spi_reg_read_dma(uint8_t addr, uint8_t *dst, size_t size)
{
	uint16_t header = 0x01 | ((uint16_t)addr << 2);

	send_header(header);
	spi_drain_rx();

	// Disable SPI DMA requests while configuring
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);

	// RX DMA: first 2 bytes to discard, then size bytes to dst.
	// Use two-phase: phase 1 discards, phase 2 stores.
	// For simplicity, do the 2 wait bytes blocking, then DMA the rest.
	(void)spi_xfer_byte(0xFF);
	(void)spi_xfer_byte(0xFF);

	// Now DMA the actual data
	DMA_Cmd(SPI_DMA_TX_CH, DISABLE);
	DMA_Cmd(SPI_DMA_RX_CH, DISABLE);

	// Clear DMA flags from previous transfer (CH2=RX, CH3=TX)
	DMA1->INTFCR = (0xF << 4) | (0xF << 8);

	// TX DMA: feed constant 0xFF
	SPI_DMA_TX_CH->PADDR = (uint32_t)&SPI1->DATAR;
	SPI_DMA_TX_CH->MADDR = (uint32_t)&dma_tx_ff;
	SPI_DMA_TX_CH->CNTR = size;
	SPI_DMA_TX_CH->CFGR = DMA_DIR_PeripheralDST |
			       DMA_PeripheralDataSize_Byte |
			       DMA_MemoryDataSize_Byte |
			       DMA_Mode_Normal |
			       DMA_Priority_High |
			       DMA_CFGR1_EN;

	// RX DMA: collect to dst
	SPI_DMA_RX_CH->PADDR = (uint32_t)&SPI1->DATAR;
	SPI_DMA_RX_CH->MADDR = (uint32_t)dst;
	SPI_DMA_RX_CH->CNTR = size;
	SPI_DMA_RX_CH->CFGR = DMA_PeripheralDataSize_Byte |
			       DMA_MemoryDataSize_Byte |
			       DMA_MemoryInc_Enable |
			       DMA_Mode_Normal |
			       DMA_Priority_High |
			       DMA_CFGR1_EN;

	dma_busy = 1;

	// Enable SPI DMA requests — transfers start immediately
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, ENABLE);
}

bool spi_reg_read_dma_busy(void)
{
	if (!dma_busy)
		return false;

	if (SPI_DMA_RX_CH->CNTR != 0)
		return true;

	// DMA complete — wait for SPI to finish, then deassert CS
	while (SPI1->STATR & SPI_I2S_FLAG_BSY)
		;

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx | SPI_I2S_DMAReq_Rx, DISABLE);
	DMA_Cmd(SPI_DMA_TX_CH, DISABLE);
	DMA_Cmd(SPI_DMA_RX_CH, DISABLE);
	end_transaction();
	dma_busy = 0;
	return false;
}

void spi_reg_write(uint8_t addr, uint32_t val)
{
	uint16_t header = 0x02 | ((uint16_t)addr << 2);
	uint8_t data[4] = { val, val >> 8, val >> 16, val >> 24 };

	send_header(header);

	if (use_bitbang_data) {
		for (int i = 0; i < 4; i++)
			(void)bitbang_xfer_byte(data[i]);
	} else {
		spi_drain_rx();
		for (int i = 0; i < 4; i++)
			(void)spi_xfer_byte(data[i]);
		while (SPI1->STATR & SPI_I2S_FLAG_BSY)
			;
	}
	end_transaction();
}

void spi_reg_write_buf(uint8_t addr, const uint8_t *src, size_t size)
{
	for (size_t off = 0; off < size; off += 4) {
		uint32_t val = 0;
		size_t chunk = (size - off < 4) ? (size - off) : 4;
		for (size_t i = 0; i < chunk; i++)
			val |= (uint32_t)src[off + i] << (i * 8);
		spi_reg_write(addr, val);
	}
}

int spi_reg_set_frequency(uint32_t freq_hz)
{
	// APB2 clock = 72 MHz (or 144 MHz depending on config)
	// SPI prescaler options: 2,4,8,16,32,64,128,256
	uint32_t apb2 = SystemCoreClock; // SPI1 on APB2, assume same as sysclk
	uint16_t prescalers[] = {
		SPI_BaudRatePrescaler_2,
		SPI_BaudRatePrescaler_4,
		SPI_BaudRatePrescaler_8,
		SPI_BaudRatePrescaler_16,
		SPI_BaudRatePrescaler_32,
		SPI_BaudRatePrescaler_64,
		SPI_BaudRatePrescaler_128,
		SPI_BaudRatePrescaler_256,
	};
	uint32_t dividers[] = { 2, 4, 8, 16, 32, 64, 128, 256 };

	// Find smallest prescaler where apb2/div <= freq_hz
	spi_prescaler = SPI_BaudRatePrescaler_256; // fallback
	spi_divider = 256;
	for (int i = 0; i < 8; i++) {
		if (apb2 / dividers[i] <= freq_hz) {
			spi_prescaler = prescalers[i];
			spi_divider = dividers[i];
			break;
		}
	}

	// HW SPI minimum is PCLK2/256. Use bit-bang if requested freq is lower.
	uint32_t hw_min = apb2 / 256;
	use_bitbang_data = (freq_hz < hw_min);
	uint32_t bbfreq = freq_hz > 0 ? freq_hz : 5000;
	bitbang_delay_n = SystemCoreClock / (2 * bbfreq * 4);

	SPI_Cmd(SPI1, DISABLE);
	SPI1->CTLR1 = (SPI1->CTLR1 & ~SPI_BaudRatePrescaler_256) | spi_prescaler;
	SPI1->CTLR1 |= SPI_CTLR1_SSI;
	SPI_Cmd(SPI1, ENABLE);
	return 0;
}

// ======================================================================
// SMC reset (same logic as RP2040, different GPIO API)
// ======================================================================

static bool reset_smc_en;
static bool reset_smc_dbg;

void reset_smc_request(bool dbg)
{
	reset_smc_en = true;
	reset_smc_dbg = dbg;
}

bool reset_smc_done(void)
{
	return !reset_smc_en;
}

void reset_smc_task(void)
{
	static enum {
		ASSERT_RST,
		START_TIMER,
		WAIT_TIMER,
		DEASSERT_RST,
	} state;
	static uint32_t wait_start;

	if (!reset_smc_en)
		return;

	#define SMC_RST_HOLD_US (100 * 1000) // 100 ms

	switch (state) {
	case ASSERT_RST:
		if (reset_smc_dbg)
			GPIO_WriteBit(SPI_SS_PORT, SPI_SS_PIN, Bit_RESET);
		GPIO_WriteBit(SMC_RST_PORT, SMC_RST_PIN, Bit_RESET);
		state = START_TIMER;
		break;
	case START_TIMER:
		wait_start = time_us_32();
		state = WAIT_TIMER;
		break;
	case WAIT_TIMER:
		if ((time_us_32() - wait_start) >= SMC_RST_HOLD_US)
			state = DEASSERT_RST;
		break;
	case DEASSERT_RST:
		GPIO_WriteBit(SMC_RST_PORT, SMC_RST_PIN, Bit_SET);
		if (reset_smc_dbg)
			GPIO_WriteBit(SPI_SS_PORT, SPI_SS_PIN, Bit_SET);
		reset_smc_en = false;
		state = ASSERT_RST;
		break;
	}
}
