// SPDX-License-Identifier: GPL-2.0-only
// spi_reg.h - SPI register access

#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Initialize SPI register access
void spi_reg_init(void);

// Read SPI register (blocking, for small reads)
void spi_reg_read(uint8_t addr, uint8_t *dst, size_t size);

// Start async DMA read of SPI register. dst must remain valid until done.
void spi_reg_read_dma(uint8_t addr, uint8_t *dst, size_t size);

// Returns true while a DMA read is still in progress.
bool spi_reg_read_dma_busy(void);

// Write SPI register (single 32-bit value)
void spi_reg_write(uint8_t addr, uint32_t val);

// Write SPI register buffer (multiple bytes, 4 bytes at a time)
void spi_reg_write_buf(uint8_t addr, const uint8_t *src, size_t size);

// Set SPI clock frequency in Hz. Returns 0 on success.
int spi_reg_set_frequency(uint32_t freq_hz);

// Request an SMC reset.
// Bring the SMC into debug mode if requested.
void reset_smc_request(bool dbg);

// See if the reset requested above is done.
bool reset_smc_done(void);

// Execute SMC resets requested by reset_smc_request().
void reset_smc_task(void);
