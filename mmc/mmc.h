// SPDX-License-Identifier: GPL-2.0-only
// mmc.h - eMMC controller public API

#pragma once
#include <stdint.h>

// ======================================================================
// Status codes (shared by init, read, write)
// ======================================================================

enum {
	MMC_IDLE,
	MMC_BUSY,
	MMC_DONE,
	MMC_ERROR,
};

// ======================================================================
// Init — async state machine
// ======================================================================

void mmc_init_start(void);
void mmc_init_task(void);
int  mmc_init_status(void);
int  mmc_init_error_state(void);  // which state failed (for debug)

// Stored during init
const uint8_t *mmc_get_cid(void);   // 16 bytes
const uint8_t *mmc_get_csd(void);   // 16 bytes

// ======================================================================
// EXT_CSD read — async state machine
// ======================================================================

void mmc_read_ext_csd_start(uint8_t *buf);
void mmc_read_ext_csd_task(void);
int  mmc_read_ext_csd_status(void);

// ======================================================================
// Abort — async state machine (CMD12 + software reset)
// ======================================================================

void mmc_abort_start(void);
void mmc_abort_task(void);
int  mmc_abort_status(void);

// ======================================================================
// Block read — async state machine
// ======================================================================

void     mmc_read_start(uint32_t lba, uint16_t count);
void     mmc_read_task(void);
int      mmc_read_status(void);
uint32_t mmc_last_error(void);
int      mmc_read_state(void);   // current state machine state (debug)

// Pull completed 512-byte blocks
const uint8_t *mmc_read_get_buf(void);
void mmc_read_consume(void);

// ======================================================================
// Block write — async state machine
// ======================================================================

void mmc_write_start(uint32_t lba, uint16_t count);
void mmc_write_task(void);
int  mmc_write_status(void);

// Push 512-byte blocks
uint8_t *mmc_write_get_buf(void);
void mmc_write_submit(void);
