// SPDX-License-Identifier: GPL-2.0-only
// write.c - Non-blocking MMC multi-block write state machine

#include "mmc.h"
#include "internal.h"
#include "spi_reg.h"

// Write buffers (per-block for now)
uint8_t mmc_buf[2][MMC_BLOCK_SIZE];

static enum {
	WS_SEND_CMD,
	WS_POLL_CMD_COMPLETE,
	WS_FIND_BUF,
	WS_POLL_BUF_READY,
	WS_WRITE_DATA,
	WS_ADVANCE,
	WS_POLL_XFER_COMPLETE,
} write_state;

void mmc_write_start(uint32_t lba, uint16_t count)
{
	mmc_bio_status = MMC_BUSY;
	mmc_is_read = 0;
	mmc_cur_lba = lba;
	mmc_blocks_total = count;
	mmc_blocks_issued = 0;
	mmc_blocks_completed = 0;
	mmc_blocks_consumed = 0;
	mmc_active_buf = 0;
	mmc_buf_state[0] = BUF_FREE;
	mmc_buf_state[1] = BUF_FREE;
	write_state = WS_SEND_CMD;
}

int mmc_write_status(void)
{
	return mmc_bio_status;
}

void mmc_write_task(void)
{
	uint32_t status;

	if (mmc_bio_status != MMC_BUSY || mmc_is_read)
		return;

	switch (write_state) {
	case WS_SEND_CMD:
		reg_write(REG_BLOCK_SIZE_COUNT,
			  MMC_BLOCK_SIZE | ((uint32_t)mmc_blocks_total << 16));
		reg_write(REG_INT_STATUS, 0xFFFFFFFF);
		reg_write(REG_ARGUMENT, mmc_cur_lba);
		reg_write(REG_XFER_MODE_CMD, CMD25_WRITE_MULTI);
		write_state = WS_POLL_CMD_COMPLETE;
		break;

	case WS_POLL_CMD_COMPLETE:
		status = reg_read(REG_INT_STATUS);
		if (status & INT_ERROR_MASK) {
			mmc_bio_status = MMC_ERROR;
			break;
		}
		if (status & INT_CMD_COMPLETE) {
			reg_write(REG_INT_STATUS, INT_CMD_COMPLETE);
			write_state = WS_FIND_BUF;
		}
		break;

	// --- hot path: advance → find buf → poll ready → write data ---

	case WS_ADVANCE:
		mmc_buf_state[mmc_active_buf] = BUF_FREE;
		mmc_blocks_issued++;
		mmc_blocks_completed++;
		if (mmc_blocks_completed >= mmc_blocks_total) {
			write_state = WS_POLL_XFER_COMPLETE;
			break;
		}
		write_state = WS_FIND_BUF;
		/* fall through */

	case WS_FIND_BUF:
		if (mmc_buf_state[0] == BUF_READY_FOR_SPI)
			mmc_active_buf = 0;
		else if (mmc_buf_state[1] == BUF_READY_FOR_SPI)
			mmc_active_buf = 1;
		else
			break;
		write_state = WS_POLL_BUF_READY;
		/* fall through */

	case WS_POLL_BUF_READY:
		status = reg_read(REG_INT_STATUS);
		if (status & INT_ERROR_MASK) {
			mmc_buf_state[mmc_active_buf] = BUF_FREE;
			mmc_bio_status = MMC_ERROR;
			break;
		}
		if (!(status & INT_BUF_WRITE_READY))
			break;
		reg_write(REG_INT_STATUS, INT_BUF_WRITE_READY);
		write_state = WS_WRITE_DATA;
		/* fall through */

	case WS_WRITE_DATA:
		spi_reg_write_buf(REG_BUFFER_DATA, mmc_buf[mmc_active_buf], MMC_BLOCK_SIZE);
		write_state = WS_ADVANCE;
		break;

	// --- end of hot path ---

	case WS_POLL_XFER_COMPLETE:
		status = reg_read(REG_INT_STATUS);
		if (status & INT_ERROR_MASK) {
			mmc_bio_status = MMC_ERROR;
			break;
		}
		if (status & INT_XFER_COMPLETE) {
			reg_write(REG_INT_STATUS, INT_XFER_COMPLETE);
			mmc_bio_status = MMC_DONE;
		}
		break;
	}
}

uint8_t *mmc_write_get_buf(void)
{
	if (mmc_buf_state[0] == BUF_FREE) {
		mmc_buf_state[0] = BUF_USB_FILLING;
		return mmc_buf[0];
	}
	if (mmc_buf_state[1] == BUF_FREE) {
		mmc_buf_state[1] = BUF_USB_FILLING;
		return mmc_buf[1];
	}
	return 0;
}

void mmc_write_submit(void)
{
	if (mmc_buf_state[0] == BUF_USB_FILLING) {
		mmc_buf_state[0] = BUF_READY_FOR_SPI;
		mmc_blocks_consumed++;
	} else if (mmc_buf_state[1] == BUF_USB_FILLING) {
		mmc_buf_state[1] = BUF_READY_FOR_SPI;
		mmc_blocks_consumed++;
	}
}
