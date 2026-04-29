// SPDX-License-Identifier: GPL-2.0-only
// read.c - Non-blocking MMC multi-block read state machine
//
// Issues one CMD18 per chunk of CHUNK_BLOCKS, reading all blocks
// in a single SPI/DMA transaction per chunk.

#include "mmc.h"
#include "internal.h"
#include "spi_reg.h"

#define CHUNK_BLOCKS 1
#define CHUNK_SIZE   (CHUNK_BLOCKS * MMC_BLOCK_SIZE)

// Shared state (defined here, externed in internal.h)
int      mmc_buf_state[2];
int      mmc_bio_status;
uint32_t mmc_bio_error;
int      mmc_is_read;
uint32_t mmc_cur_lba;
uint16_t mmc_blocks_total;
uint16_t mmc_blocks_issued;
uint16_t mmc_blocks_completed;
uint16_t mmc_blocks_consumed;
int      mmc_active_buf;

static uint8_t read_buf[2][CHUNK_SIZE];
static uint16_t buf_blocks[2];
static uint16_t buf_sent[2];

static enum {
	RS_FIND_BUF,
	RS_SEND_CMD,
	RS_POLL_CMD_COMPLETE,
	RS_POLL_BUF_READY,
	RS_READ_BULK,
	RS_READ_BULK_WAIT,
	RS_POLL_XFER_COMPLETE,
	RS_ADVANCE,
} read_state;

void mmc_read_start(uint32_t lba, uint16_t count)
{
	mmc_bio_status = MMC_BUSY;
	mmc_is_read = 1;
	mmc_cur_lba = lba;
	mmc_blocks_total = count;
	mmc_blocks_issued = 0;
	mmc_blocks_completed = 0;
	mmc_blocks_consumed = 0;
	mmc_active_buf = 0;
	mmc_buf_state[0] = BUF_FREE;
	mmc_buf_state[1] = BUF_FREE;
	buf_blocks[0] = buf_blocks[1] = 0;
	buf_sent[0] = buf_sent[1] = 0;
	read_state = RS_FIND_BUF;
}

int mmc_read_status(void)
{
	return mmc_bio_status;
}

uint32_t mmc_last_error(void)
{
	return mmc_bio_error;
}

int mmc_read_state(void)
{
	return (int)read_state;
}

void mmc_read_task(void)
{
	uint32_t status;

	if (mmc_bio_status != MMC_BUSY || !mmc_is_read)
		return;

	if (mmc_blocks_issued >= mmc_blocks_total) {
		if (mmc_blocks_consumed >= mmc_blocks_total)
			mmc_bio_status = MMC_DONE;
		return;
	}

	switch (read_state) {

	// --- per-chunk: find buf → cmd → poll cmd → poll ready → bulk DMA → poll xfer → advance → loop

	case RS_FIND_BUF:
		if (mmc_buf_state[0] == BUF_FREE)
			mmc_active_buf = 0;
		else if (mmc_buf_state[1] == BUF_FREE)
			mmc_active_buf = 1;
		else
			break;
		mmc_buf_state[mmc_active_buf] = BUF_SPI_FILLING;
		read_state = RS_SEND_CMD;
		/* fall through */

	case RS_SEND_CMD: {
		uint16_t remaining = mmc_blocks_total - mmc_blocks_issued;
		uint16_t n = (remaining > CHUNK_BLOCKS) ? CHUNK_BLOCKS : remaining;
		buf_blocks[mmc_active_buf] = n;
		buf_sent[mmc_active_buf] = 0;

		reg_write(REG_BLOCK_SIZE_COUNT, MMC_BLOCK_SIZE | ((uint32_t)n << 16));
		reg_write(REG_INT_STATUS, 0xFFFFFFFF);
		reg_write(REG_ARGUMENT, mmc_cur_lba + mmc_blocks_issued);
		reg_write(REG_XFER_MODE_CMD, CMD18_READ_MULTI);
		read_state = RS_POLL_CMD_COMPLETE;
		break;
	}

	case RS_POLL_CMD_COMPLETE:
		status = reg_read(REG_INT_STATUS);
		if (status & INT_ERROR_MASK) {
			mmc_buf_state[mmc_active_buf] = BUF_FREE;
			mmc_bio_error = status;
			mmc_bio_status = MMC_ERROR;
			break;
		}
		if (status & INT_CMD_COMPLETE) {
			reg_write(REG_INT_STATUS, INT_CMD_COMPLETE);
			read_state = RS_POLL_BUF_READY;
		}
		break;

	case RS_POLL_BUF_READY:
		status = reg_read(REG_INT_STATUS);
		if (status & INT_ERROR_MASK) {
			mmc_buf_state[mmc_active_buf] = BUF_FREE;
			mmc_bio_error = status;
			mmc_bio_status = MMC_ERROR;
			break;
		}
		if (!(status & INT_BUF_READ_READY))
			break;
		// Don't clear BUF_READ_READY — matches reference code fast path
		// (clear_on_match=0). The bulk read consumes all buffered data.
		read_state = RS_READ_BULK;
		/* fall through */

	case RS_READ_BULK:
		spi_reg_read_dma(REG_BUFFER_DATA, read_buf[mmc_active_buf],
				 buf_blocks[mmc_active_buf] * MMC_BLOCK_SIZE);
		read_state = RS_READ_BULK_WAIT;
		break;

	case RS_READ_BULK_WAIT:
		if (spi_reg_read_dma_busy())
			break;
		read_state = RS_POLL_XFER_COMPLETE;
		/* fall through */

	case RS_POLL_XFER_COMPLETE:
		status = reg_read(REG_INT_STATUS);
		if (status & INT_ERROR_MASK) {
			mmc_buf_state[mmc_active_buf] = BUF_FREE;
			mmc_bio_error = status;
			mmc_bio_status = MMC_ERROR;
			break;
		}
		if (!(status & INT_XFER_COMPLETE))
			break;
		reg_write(REG_INT_STATUS, INT_XFER_COMPLETE);
		read_state = RS_ADVANCE;
		/* fall through */

	case RS_ADVANCE:
		mmc_buf_state[mmc_active_buf] = BUF_READY_FOR_USB;
		mmc_blocks_issued += buf_blocks[mmc_active_buf];
		mmc_blocks_completed += buf_blocks[mmc_active_buf];
		if (mmc_blocks_issued < mmc_blocks_total)
			read_state = RS_FIND_BUF;
		break;
	}
}

const uint8_t *mmc_read_get_buf(void)
{
	for (int i = 0; i < 2; i++) {
		if (mmc_buf_state[i] == BUF_READY_FOR_USB &&
		    buf_sent[i] < buf_blocks[i])
			return read_buf[i] + buf_sent[i] * MMC_BLOCK_SIZE;
	}
	return 0;
}

void mmc_read_consume(void)
{
	for (int i = 0; i < 2; i++) {
		if (mmc_buf_state[i] == BUF_READY_FOR_USB &&
		    buf_sent[i] < buf_blocks[i]) {
			buf_sent[i]++;
			mmc_blocks_consumed++;
			if (buf_sent[i] >= buf_blocks[i])
				mmc_buf_state[i] = BUF_FREE;
			return;
		}
	}
}
