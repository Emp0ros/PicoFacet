// SPDX-License-Identifier: GPL-2.0-only
// abort.c - Async SDHCI abort (CMD12 + software reset)

#include "mmc.h"
#include "internal.h"

static int abort_status;

static enum {
	AB_POLL_CMD12,
	AB_POLL_RESET,
} abort_state;

void mmc_abort_start(void)
{
	abort_status = MMC_BUSY;
	// Capture INT_STATUS before clearing — this is what the Arasan
	// was doing when the host timed out
	mmc_bio_error = reg_read(REG_INT_STATUS);
	reg_write(REG_INT_STATUS, 0xFFFFFFFF);
	reg_write(REG_ARGUMENT, 0);
	reg_write(REG_XFER_MODE_CMD, CMD12_STOP);
	abort_state = AB_POLL_CMD12;
}

int mmc_abort_status(void)
{
	return abort_status;
}

void mmc_abort_task(void)
{
	if (abort_status != MMC_BUSY)
		return;

	switch (abort_state) {
	case AB_POLL_CMD12: {
		uint32_t st = reg_read(REG_INT_STATUS);
		if (st & (INT_CMD_COMPLETE | INT_ERROR_MASK)) {
			reg_write(REG_INT_STATUS, 0xFFFFFFFF);
			uint32_t clk = reg_read(REG_CLOCK_CTRL);
			reg_write(REG_CLOCK_CTRL, clk | CLK_SW_RESET_CMD_DAT);
			abort_state = AB_POLL_RESET;
		}
		break;
	}

	case AB_POLL_RESET:
		if ((reg_read(REG_CLOCK_CTRL) & CLK_SW_RESET_CMD_DAT) == 0) {
			reg_write(REG_INT_STATUS, 0xFFFFFFFF);
			mmc_bio_status = MMC_IDLE;
			abort_status = MMC_DONE;
		}
		break;
	}
}
