// SPDX-License-Identifier: GPL-2.0-only
// init.c - eMMC controller and card initialization
//
// Fully non-blocking state machine split into three phases:
//   1. SMC reset
//   2. Controller open (SPI config, sanity check, clock, interrupts)
//   3. Card init (CMD0..CMD16, mode switching, final clock)

#include "mmc.h"
#include "internal.h"
#include "platform.h"
#include <string.h>

#define BASE_CLOCK_MHZ        196.875
#define SPI_MAX_FREQ          30000000u
#define SPI_CONFIG_VALUE      0x03
#define TARGET_CLOCK_MHZ      50

// ======================================================================
// Reusable command helpers
// ======================================================================

static void cmd_begin(uint32_t cmd_word, uint32_t arg)
{
	reg_write(REG_INT_STATUS, 0xFFFFFFFF);
	reg_write(REG_ARGUMENT, arg);
	reg_write(REG_XFER_MODE_CMD, cmd_word);
}

// Returns: 0=busy, 1=complete
static int cmd_poll(void)
{
	uint32_t st = reg_read(REG_INT_STATUS);
	if (st & INT_CMD_COMPLETE) {
		reg_write(REG_INT_STATUS, INT_CMD_COMPLETE);
		return 1;
	}
	return 0;
}

// Returns: 0=busy, 1=complete, -1=error
static int cmd_poll_check(void)
{
	uint32_t st = reg_read(REG_INT_STATUS);
	if (st & INT_ERROR_MASK)
		return -1;
	if (st & INT_CMD_COMPLETE) {
		reg_write(REG_INT_STATUS, INT_CMD_COMPLETE);
		return 1;
	}
	return 0;
}

// Returns: 0=busy, 1=complete, -1=error
static int xfer_poll_check(void)
{
	uint32_t st = reg_read(REG_INT_STATUS);
	if (st & INT_ERROR_MASK)
		return -1;
	if (st & INT_XFER_COMPLETE) {
		reg_write(REG_INT_STATUS, INT_XFER_COMPLETE);
		return 1;
	}
	return 0;
}

static void read_response_128(uint8_t *out)
{
	for (int i = 0; i < 4; i++) {
		uint32_t val = reg_read(REG_RESPONSE_0 + i);
		memcpy(out + i * 4, &val, 4);
	}
}

// ======================================================================
// Clock sub-machine
// ======================================================================

static enum { CLK_IDLE, CLK_WAIT_STABLE, CLK_SET_DIVIDER } clk_state;
static double clk_target_mhz;

static void clock_begin(double target_mhz)
{
	clk_target_mhz = target_mhz;
	uint32_t clk = reg_read(REG_CLOCK_CTRL);
	clk |= CLK_INTERNAL_EN;
	reg_write(REG_CLOCK_CTRL, clk);
	clk_state = CLK_WAIT_STABLE;
}

// Returns: 0=busy, 1=done
static int clock_poll(void)
{
	if (clk_state == CLK_WAIT_STABLE) {
		if (!(reg_read(REG_CLOCK_CTRL) & CLK_INTERNAL_STABLE))
			return 0;
		clk_state = CLK_SET_DIVIDER;
	}

	uint32_t clk = reg_read(REG_CLOCK_CTRL);
	clk &= ~CLK_SD_EN;
	reg_write(REG_CLOCK_CTRL, clk);

	uint32_t div = 0;
	if (clk_target_mhz > 0 && clk_target_mhz < BASE_CLOCK_MHZ)
		div = (uint32_t)(BASE_CLOCK_MHZ / (2.0 * clk_target_mhz) + 0.5);

	uint32_t div_bits = ((div & 0xFF) << 8) | ((div >> 8) << 6);
	clk = reg_read(REG_CLOCK_CTRL);
	clk &= 0xFFFF0000u;
	clk |= div_bits | CLK_INTERNAL_EN | CLK_SD_EN;
	reg_write(REG_CLOCK_CTRL, clk);
	return 1;
}

// ======================================================================
// Sanity check
// ======================================================================

static const uint32_t sanity_patterns[] = { 0x12345678, 0xEDCBA987 };

static int sanity_check(void)
{
	for (int p = 0; p < 2; p++) {
		reg_write(REG_ARGUMENT, sanity_patterns[p]);
		uint32_t rb = reg_read(REG_ARGUMENT);
		if (rb != sanity_patterns[p])
			return -1;
	}
	return 0;
}

// ======================================================================
// Phase 1: SMC reset
// ======================================================================

static enum {
	SMC_START,
	SMC_WAIT,
} smc_state;

// Returns: 0=busy, 1=done
static int smc_reset_step(void)
{
	switch (smc_state) {
	case SMC_START:
		reset_smc_request(1);
		smc_state = SMC_WAIT;
		return 0;
	case SMC_WAIT:
		return reset_smc_done() ? 1 : 0;
	}
	return 0;
}

// ======================================================================
// Phase 2: Controller open
// ======================================================================

static int sanity_retries;
static uint32_t delay_start;

static enum {
	CTRL_SPI_FREQ,
	CTRL_SPI_CONFIG,
	CTRL_SANITY_SLOW,
	CTRL_SANITY_DELAY,
	CTRL_SANITY_FAST,
	CTRL_CLOCK_START,
	CTRL_CLOCK_POLL,
	CTRL_INT_ENABLE,
} ctrl_state;

// Returns: 0=busy, 1=done, -1=error
static int controller_open_step(void)
{
	switch (ctrl_state) {
	case CTRL_SPI_FREQ:
		spi_reg_set_frequency(5000);
		ctrl_state = CTRL_SPI_CONFIG;
		return 0;

	case CTRL_SPI_CONFIG:
		reg_write(REG_SPI_CONFIG, SPI_CONFIG_VALUE);
		sanity_retries = 0;
		ctrl_state = CTRL_SANITY_SLOW;
		return 0;

	case CTRL_SANITY_SLOW:
		if (sanity_check() == 0) {
			spi_reg_set_frequency(SPI_MAX_FREQ);
			ctrl_state = CTRL_SANITY_FAST;
			return 0;
		}
		if (++sanity_retries > 10)
			return -1;
		delay_start = time_us_32();
		ctrl_state = CTRL_SANITY_DELAY;
		return 0;

	case CTRL_SANITY_DELAY:
		if ((time_us_32() - delay_start) >= 100000)
			ctrl_state = CTRL_SANITY_SLOW;
		return 0;

	case CTRL_SANITY_FAST:
		if (sanity_check() == 0) {
			ctrl_state = CTRL_CLOCK_START;
			return 0;
		}
		if (++sanity_retries > 10)
			return -1;
		spi_reg_set_frequency(5000);
		delay_start = time_us_32();
		ctrl_state = CTRL_SANITY_DELAY;
		return 0;

	case CTRL_CLOCK_START:
		clock_begin(0.384521484375);
		ctrl_state = CTRL_CLOCK_POLL;
		return 0;

	case CTRL_CLOCK_POLL:
		if (!clock_poll())
			return 0;
		ctrl_state = CTRL_INT_ENABLE;
		return 0;

	case CTRL_INT_ENABLE: {
		uint32_t val = reg_read(REG_INT_ENABLE);
		reg_write(REG_INT_ENABLE, val | INT_STATUS_EN_DEFAULT);
		val = reg_read(REG_SIGNAL_ENABLE);
		reg_write(REG_SIGNAL_ENABLE, val | INT_SIGNAL_EN_DEFAULT);
		return 1;
	}
	}
	return 0;
}

// ======================================================================
// Phase 3: Card init
// ======================================================================

static uint8_t stored_cid[16];
static uint8_t stored_csd[16];

static enum {
	CARD_CMD0,
	CARD_CMD0_POLL,
	CARD_EMMC_TIMEOUT,
	CARD_CMD1_START,
	CARD_CMD1_POLL,
	CARD_CMD1_CHECK,
	CARD_CMD2_START,
	CARD_CMD2_POLL,
	CARD_CMD3_START,
	CARD_CMD3_POLL,
	CARD_CMD9_START,
	CARD_CMD9_POLL,
	CARD_CMD7_START,
	CARD_CMD7_POLL,
	CARD_SWITCH_HS_START,
	CARD_SWITCH_HS_CMD_POLL,
	CARD_SWITCH_HS_XFER_POLL,
	CARD_SWITCH_BW_START,
	CARD_SWITCH_BW_CMD_POLL,
	CARD_SWITCH_BW_XFER_POLL,
	CARD_HOST_CTRL1,
	CARD_CMD16_START,
	CARD_CMD16_POLL,
	CARD_FINAL_CLOCK_START,
	CARD_FINAL_CLOCK_POLL,
} card_state;

// Returns: 0=busy, 1=done, -1=error
static int card_init_step(void)
{
	int rc;

	switch (card_state) {
	case CARD_CMD0:
		cmd_begin(CMD0_GO_IDLE, 0);
		card_state = CARD_CMD0_POLL;
		return 0;

	case CARD_CMD0_POLL:
		if (cmd_poll())
			card_state = CARD_EMMC_TIMEOUT;
		return 0;

	case CARD_EMMC_TIMEOUT: {
		uint32_t clk = reg_read(REG_CLOCK_CTRL);
		reg_write(REG_CLOCK_CTRL, clk | CLK_EMMC_TIMEOUT);
		card_state = CARD_CMD1_START;
		return 0;
	}

	case CARD_CMD1_START:
		cmd_begin(CMD1_SEND_OP_COND, CMD1_ARG);
		card_state = CARD_CMD1_POLL;
		return 0;

	case CARD_CMD1_POLL:
		rc = cmd_poll_check();
		if (rc < 0) { card_state = CARD_CMD1_START; return 0; }
		if (rc == 0) return 0;
		card_state = CARD_CMD1_CHECK;
		return 0;

	case CARD_CMD1_CHECK: {
		uint32_t resp = reg_read(REG_RESPONSE_0);
		if (resp & 0x80000000u)
			card_state = CARD_CMD2_START;
		else
			card_state = CARD_CMD1_START;
		return 0;
	}

	case CARD_CMD2_START:
		cmd_begin(CMD2_ALL_SEND_CID, 0);
		card_state = CARD_CMD2_POLL;
		return 0;

	case CARD_CMD2_POLL:
		rc = cmd_poll_check();
		if (rc < 0) return -1;
		if (rc == 0) return 0;
		read_response_128(stored_cid);
		card_state = CARD_CMD3_START;
		return 0;

	case CARD_CMD3_START:
		cmd_begin(CMD3_SET_RCA, MMC_RCA);
		card_state = CARD_CMD3_POLL;
		return 0;

	case CARD_CMD3_POLL:
		rc = cmd_poll_check();
		if (rc < 0) return -1;
		if (rc == 0) return 0;
		card_state = CARD_CMD9_START;
		return 0;

	case CARD_CMD9_START:
		cmd_begin(CMD9_SEND_CSD, MMC_RCA);
		card_state = CARD_CMD9_POLL;
		return 0;

	case CARD_CMD9_POLL:
		rc = cmd_poll_check();
		if (rc < 0) return -1;
		if (rc == 0) return 0;
		read_response_128(stored_csd);
		card_state = CARD_CMD7_START;
		return 0;

	case CARD_CMD7_START:
		cmd_begin(CMD7_SELECT, MMC_RCA);
		card_state = CARD_CMD7_POLL;
		return 0;

	case CARD_CMD7_POLL:
		rc = cmd_poll_check();
		if (rc < 0) return -1;
		if (rc == 0) return 0;
		card_state = CARD_SWITCH_HS_START;
		return 0;

	case CARD_SWITCH_HS_START:
		cmd_begin(CMD6_SWITCH, 0x03B70100u); // HS_TIMING = 1
		card_state = CARD_SWITCH_HS_CMD_POLL;
		return 0;

	case CARD_SWITCH_HS_CMD_POLL:
		rc = cmd_poll_check();
		if (rc < 0) return -1;
		if (rc == 0) return 0;
		card_state = CARD_SWITCH_HS_XFER_POLL;
		return 0;

	case CARD_SWITCH_HS_XFER_POLL:
		rc = xfer_poll_check();
		if (rc < 0) return -1;
		if (rc == 0) return 0;
		card_state = (TARGET_CLOCK_MHZ > 25)
			? CARD_SWITCH_BW_START : CARD_HOST_CTRL1;
		return 0;

	case CARD_SWITCH_BW_START:
		cmd_begin(CMD6_SWITCH, 0x03B90100u); // BUS_WIDTH = 4-bit
		card_state = CARD_SWITCH_BW_CMD_POLL;
		return 0;

	case CARD_SWITCH_BW_CMD_POLL:
		rc = cmd_poll_check();
		if (rc < 0) return -1;
		if (rc == 0) return 0;
		card_state = CARD_SWITCH_BW_XFER_POLL;
		return 0;

	case CARD_SWITCH_BW_XFER_POLL:
		rc = xfer_poll_check();
		if (rc < 0) return -1;
		if (rc == 0) return 0;
		card_state = CARD_HOST_CTRL1;
		return 0;

	case CARD_HOST_CTRL1: {
		uint32_t ctrl = reg_read(REG_HOST_CTRL1);
		if (TARGET_CLOCK_MHZ > 25)
			ctrl |= HOST_CTRL1_4BIT | HOST_CTRL1_HIGH_SPEED;
		reg_write(REG_HOST_CTRL1, ctrl);
		card_state = CARD_CMD16_START;
		return 0;
	}

	case CARD_CMD16_START:
		cmd_begin(CMD16_SET_BLOCKLEN, 512);
		card_state = CARD_CMD16_POLL;
		return 0;

	case CARD_CMD16_POLL:
		rc = cmd_poll_check();
		if (rc < 0) return -1;
		if (rc == 0) return 0;
		card_state = CARD_FINAL_CLOCK_START;
		return 0;

	case CARD_FINAL_CLOCK_START:
		clock_begin(TARGET_CLOCK_MHZ);
		card_state = CARD_FINAL_CLOCK_POLL;
		return 0;

	case CARD_FINAL_CLOCK_POLL:
		if (!clock_poll())
			return 0;
		return 1;
	}
	return 0;
}

// ======================================================================
// Top-level init — sequences the three phases
// ======================================================================

static int status;

static enum {
	PHASE_SMC_RESET,
	PHASE_CONTROLLER_OPEN,
	PHASE_CARD_INIT,
	PHASE_DONE,
} phase;

void mmc_init_start(void)
{
	status = MMC_BUSY;
	phase = PHASE_SMC_RESET;
	smc_state = SMC_START;
	ctrl_state = CTRL_SPI_FREQ;
	card_state = CARD_CMD0;
}

int mmc_init_status(void)
{
	return status;
}

int mmc_init_error_state(void)
{
	return (phase << 8) | (int)(phase == PHASE_SMC_RESET ? smc_state :
				    phase == PHASE_CONTROLLER_OPEN ? ctrl_state :
				    card_state);
}

const uint8_t *mmc_get_cid(void) { return stored_cid; }
const uint8_t *mmc_get_csd(void) { return stored_csd; }

void mmc_init_task(void)
{
	if (status != MMC_BUSY)
		return;

	int rc;

	switch (phase) {
	case PHASE_SMC_RESET:
		if (smc_reset_step())
			phase = PHASE_CONTROLLER_OPEN;
		break;

	case PHASE_CONTROLLER_OPEN:
		rc = controller_open_step();
		if (rc > 0) phase = PHASE_CARD_INIT;
		if (rc < 0) status = MMC_ERROR;
		break;

	case PHASE_CARD_INIT:
		rc = card_init_step();
		if (rc > 0) { phase = PHASE_DONE; status = MMC_DONE; }
		if (rc < 0) status = MMC_ERROR;
		break;

	case PHASE_DONE:
		break;
	}
}

// ======================================================================
// EXT_CSD read state machine
// ======================================================================

static int ext_csd_status;
static uint8_t *ext_csd_buf;

static enum {
	ECS_IDLE,
	ECS_CMD_POLL,
	ECS_BUF_READY_POLL,
	ECS_READ_DATA,
	ECS_XFER_POLL,
} ext_csd_state;

void mmc_read_ext_csd_start(uint8_t *buf)
{
	ext_csd_buf = buf;
	ext_csd_status = MMC_BUSY;

	reg_write(REG_BLOCK_SIZE_COUNT, 512 | (1u << 16));
	cmd_begin(CMD8_SEND_EXT_CSD, 0);
	ext_csd_state = ECS_CMD_POLL;
}

int mmc_read_ext_csd_status(void)
{
	return ext_csd_status;
}

void mmc_read_ext_csd_task(void)
{
	if (ext_csd_status != MMC_BUSY)
		return;

	switch (ext_csd_state) {
	case ECS_CMD_POLL: {
		int rc = cmd_poll_check();
		if (rc < 0) { ext_csd_status = MMC_ERROR; return; }
		if (rc == 0) return;
		ext_csd_state = ECS_BUF_READY_POLL;
		break;
	}

	case ECS_BUF_READY_POLL: {
		uint32_t st = reg_read(REG_INT_STATUS);
		if (st & INT_ERROR_MASK) { ext_csd_status = MMC_ERROR; return; }
		if (!(st & INT_BUF_READ_READY)) return;
		reg_write(REG_INT_STATUS, INT_BUF_READ_READY);
		ext_csd_state = ECS_READ_DATA;
		break;
	}

	case ECS_READ_DATA:
		spi_reg_read(REG_BUFFER_DATA, ext_csd_buf, 512);
		ext_csd_state = ECS_XFER_POLL;
		break;

	case ECS_XFER_POLL: {
		int rc = xfer_poll_check();
		if (rc < 0) { ext_csd_status = MMC_ERROR; return; }
		if (rc == 0) return;
		ext_csd_status = MMC_DONE;
		break;
	}

	default:
		break;
	}
}

