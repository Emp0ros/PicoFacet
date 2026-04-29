// SPDX-License-Identifier: GPL-2.0-only
// spi_reg.c - SPI register access

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"

#include "spi.pio.h"

#include "pins.h"
#include "spi_reg.h"

// Baudrate for initialization (5 KHz)
#define SPI_REG_INIT_BAUD 5000

// Baudrate for full-speed access (30 MHz)
#define SPI_REG_FULL_BAUD 30000000

// DMA channels for async reads
static int dma_tx_chan, dma_rx_skip_chan, dma_rx_data_chan;
static uint8_t dma_discard; // sink for the 2 wait bytes
static const uint8_t dma_tx_byte = 0xFF;

void spi_reg_init(void)
{
	// Setup SMC_RST_N as GPIO output
	gpio_init(SMC_RST_N);
	gpio_put(SMC_RST_N, 1);
	gpio_set_dir(SMC_RST_N, GPIO_OUT);

	// Drive SPI_SS_N high via GPIO before PIO takes over
	gpio_init(SPI_SS_N);
	gpio_put(SPI_SS_N, 1);
	gpio_set_dir(SPI_SS_N, GPIO_OUT);

	// Setup PIO SPI — CSn (pin 1) and SCK (pin 2) via 2-bit sideset
	uint offs = pio_add_program(pio0, &spi_proto_program);
	float clkdiv = (float)SYS_CLK_HZ / (4 * SPI_REG_INIT_BAUD);
	pio_spi_proto_init(pio0, 0, offs, clkdiv, SPI_SS_N, SPI_MOSI, SPI_MISO);

	// Claim DMA channels for async reads
	dma_tx_chan = dma_claim_unused_channel(true);
	dma_rx_skip_chan = dma_claim_unused_channel(true);
	dma_rx_data_chan = dma_claim_unused_channel(true);
}

void spi_reg_read_dma(uint8_t addr, uint8_t *dst, size_t size)
{
	uint32_t header = 0x01 | ((uint32_t)addr << 2);
	size_t total = size + 2;

	// Push header and count — PIO FIFO is empty so these won't block long
	pio_sm_put_blocking(pio0, 0, header);
	pio_sm_put_blocking(pio0, 0, total - 1);

	// TX DMA: feed constant 0xFF bytes to PIO TX FIFO
	dma_channel_config tx_c = dma_channel_get_default_config(dma_tx_chan);
	channel_config_set_transfer_data_size(&tx_c, DMA_SIZE_8);
	channel_config_set_read_increment(&tx_c, false);
	channel_config_set_write_increment(&tx_c, false);
	channel_config_set_dreq(&tx_c, pio_get_dreq(pio0, 0, true));
	dma_channel_configure(dma_tx_chan, &tx_c,
		&pio0->txf[0], &dma_tx_byte, total, false);

	// RX skip DMA: drain 2 wait bytes into discard, then chain to data DMA
	dma_channel_config skip_c = dma_channel_get_default_config(dma_rx_skip_chan);
	channel_config_set_transfer_data_size(&skip_c, DMA_SIZE_8);
	channel_config_set_read_increment(&skip_c, false);
	channel_config_set_write_increment(&skip_c, false);
	channel_config_set_dreq(&skip_c, pio_get_dreq(pio0, 0, false));
	channel_config_set_chain_to(&skip_c, dma_rx_data_chan);
	dma_channel_configure(dma_rx_skip_chan, &skip_c,
		&dma_discard, &pio0->rxf[0], 2, false);

	// RX data DMA: collect size bytes directly to destination
	dma_channel_config data_c = dma_channel_get_default_config(dma_rx_data_chan);
	channel_config_set_transfer_data_size(&data_c, DMA_SIZE_8);
	channel_config_set_read_increment(&data_c, false);
	channel_config_set_write_increment(&data_c, true);
	channel_config_set_dreq(&data_c, pio_get_dreq(pio0, 0, false));
	dma_channel_configure(dma_rx_data_chan, &data_c,
		dst, &pio0->rxf[0], size, false);

	// Start TX and RX skip simultaneously
	dma_start_channel_mask((1u << dma_tx_chan) | (1u << dma_rx_skip_chan));
}

bool spi_reg_read_dma_busy(void)
{
	return dma_channel_is_busy(dma_rx_data_chan);
}

void __time_critical_func(spi_reg_read)(uint8_t addr, uint8_t *dst, size_t size)
{
	// Header: 2-bit read cmd (01) + 8-bit addr = 10 bits, LSB-first
	uint32_t header = 0x01 | ((uint32_t)addr << 2);
	// 2 wait bytes + size data bytes
	size_t total = size + 2;

	pio_sm_put_blocking(pio0, 0, header);
	pio_sm_put_blocking(pio0, 0, total - 1);

	size_t tx_remain = total, rx_remain = total;
	size_t rx_skip = 2;

	while (tx_remain || rx_remain) {
		if (tx_remain && !pio_sm_is_tx_fifo_full(pio0, 0)) {
			pio_sm_put(pio0, 0, 0xFF);
			tx_remain--;
		}
		if (rx_remain && !pio_sm_is_rx_fifo_empty(pio0, 0)) {
			uint8_t val = (uint8_t)pio_sm_get(pio0, 0);
			if (rx_skip)
				rx_skip--;
			else
				*dst++ = val;
			rx_remain--;
		}
	}
}

void __time_critical_func(spi_reg_write)(uint8_t addr, uint32_t val)
{
	// Header: 2-bit write cmd (10) + 8-bit addr = 10 bits, LSB-first
	uint32_t header = 0x02 | ((uint32_t)addr << 2);
	uint8_t data[4] = { val, val >> 8, val >> 16, val >> 24 };

	pio_sm_put_blocking(pio0, 0, header);
	pio_sm_put_blocking(pio0, 0, 3); // count-1 = 3 (4 bytes)

	size_t tx_i = 0, rx_remain = 4;
	while (tx_i < 4 || rx_remain) {
		if (tx_i < 4 && !pio_sm_is_tx_fifo_full(pio0, 0)) {
			pio_sm_put(pio0, 0, data[tx_i++]);
		}
		if (rx_remain && !pio_sm_is_rx_fifo_empty(pio0, 0)) {
			pio_sm_get(pio0, 0);
			rx_remain--;
		}
	}
}

void __time_critical_func(spi_reg_write_buf)(uint8_t addr, const uint8_t *src, size_t size)
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
	float clkdiv = (float)clock_get_hz(clk_sys) / (4 * freq_hz);
	if (clkdiv < 1.0f)
		clkdiv = 1.0f;
	pio_sm_set_clkdiv(pio0, 0, clkdiv);
	pio_sm_clkdiv_restart(pio0, 0);
	return 0;
}

// Number of microseconds to assert SMC_RST_N for to reset
#define SMC_RST_HOLD_US (100 * 1000) // 100 ms

// Is the reste_smc state machine running?
static bool reset_smc_en;

// Is the requested reset into debug mode?
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
	static int wait_start;

	if (!reset_smc_en)
		return;

	switch (state) {
	case ASSERT_RST: // Assert SMC_RST_N (first assert SPI_SS_N if debug requested)
		if (reset_smc_dbg) {
			gpio_set_function(SPI_SS_N, GPIO_FUNC_SIO); // reclaim from PIO
			gpio_put(SPI_SS_N, 0);
			sleep_us(10);
		}
		gpio_put(SMC_RST_N, 0);
		state = START_TIMER;
		break;
	case START_TIMER: // Start timer
		wait_start = time_us_32();
		state = WAIT_TIMER;
		break;
	case WAIT_TIMER: // Wait timer
		if (time_us_32() > wait_start + SMC_RST_HOLD_US)
			state = DEASSERT_RST;
		break;
	case DEASSERT_RST: // Deassert SMC_RST_N (then deassert SPI_SS_N if debug requested)
		gpio_put(SMC_RST_N, 1);
		if (reset_smc_dbg) {
			sleep_us(10);
			gpio_put(SPI_SS_N, 1);
			pio_gpio_init(pio0, SPI_SS_N); // hand back to PIO
		}
		reset_smc_en = false;
		state = ASSERT_RST;
		break;
	}
}
