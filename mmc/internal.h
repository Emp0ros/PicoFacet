// SPDX-License-Identifier: GPL-2.0-only
// internal.h - Shared definitions for mmc init/read/write

#pragma once
#include <stdint.h>
#include "spi_reg.h"

// ======================================================================
// Register indices (SPI register index = SDHCI byte offset / 4)
// ======================================================================

#define REG_BLOCK_SIZE_COUNT  0x01
#define REG_ARGUMENT          0x02
#define REG_XFER_MODE_CMD     0x03
#define REG_RESPONSE_0        0x04
#define REG_RESPONSE_2        0x05
#define REG_RESPONSE_4        0x06
#define REG_RESPONSE_6        0x07
#define REG_BUFFER_DATA       0x08
#define REG_HOST_CTRL1        0x0A
#define REG_CLOCK_CTRL        0x0B
#define REG_INT_STATUS        0x0C
#define REG_INT_ENABLE        0x0D
#define REG_SIGNAL_ENABLE     0x0E
#define REG_SPI_CONFIG        0x44

// ======================================================================
// Interrupt status bits
// ======================================================================

#define INT_CMD_COMPLETE      (1u << 0)
#define INT_XFER_COMPLETE     (1u << 1)
#define INT_BUF_WRITE_READY   (1u << 4)
#define INT_BUF_READ_READY    (1u << 5)
#define INT_ERROR_MASK        0xFFFF0000u

// ======================================================================
// Clock control bits
// ======================================================================

#define CLK_INTERNAL_EN       (1u << 0)
#define CLK_INTERNAL_STABLE   (1u << 1)
#define CLK_SD_EN             (1u << 2)
#define CLK_EMMC_TIMEOUT      0x000E0000u
#define CLK_SW_RESET_CMD_DAT  0x06000000u

// ======================================================================
// Host control 1 bits
// ======================================================================

#define HOST_CTRL1_4BIT       (1u << 1)
#define HOST_CTRL1_HIGH_SPEED (1u << 2)

// ======================================================================
// Interrupt enable defaults
// ======================================================================

#define INT_STATUS_EN_DEFAULT 0x1FFF0033u
#define INT_SIGNAL_EN_DEFAULT 0x17FF0033u

// ======================================================================
// eMMC command words (cmd_index << 24 | response_flags << 16 | xfer_mode)
// ======================================================================

#define CMD0_GO_IDLE          0x00000000u
#define CMD1_SEND_OP_COND     0x01020000u
#define CMD2_ALL_SEND_CID     0x02090000u
#define CMD3_SET_RCA          0x031A0000u
#define CMD6_SWITCH           0x061B0000u  // R1b
#define CMD7_SELECT           0x071A0000u
#define CMD8_SEND_EXT_CSD     0x083A0012u  // R1+data, blk_cnt_en | read
#define CMD9_SEND_CSD         0x09090000u
#define CMD12_STOP            0x0C1B0000u  // R1b
#define CMD16_SET_BLOCKLEN    0x101A0000u
#define CMD17_READ_SINGLE     0x113A0012u  // R1+data, blk_cnt_en | read
#define CMD18_READ_MULTI      0x123A0037u  // R1+data, auto_cmd12 | read | multi
#define CMD24_WRITE_SINGLE    0x183A0002u  // R1+data, blk_cnt_en
#define CMD25_WRITE_MULTI     0x193A0027u  // R1+data, auto_cmd12 | multi

#define MMC_RCA               0x000A0000u
#define CMD1_ARG              0x40000080u  // sector mode + voltage

// ======================================================================
// Register access helpers
// ======================================================================

static inline void reg_write(uint8_t reg, uint32_t val)
{
	spi_reg_write(reg, val);
}

static inline uint32_t reg_read(uint8_t reg)
{
	uint32_t val = 0;
	spi_reg_read(reg, (uint8_t *)&val, 4);
	return val;
}

// ======================================================================
// Shared block I/O state
// ======================================================================

#define MMC_BLOCK_SIZE 512

// Write buffers (small, per-block — write doesn't have bulk SPI yet)
extern uint8_t mmc_buf[2][MMC_BLOCK_SIZE];

enum {
	BUF_FREE,
	BUF_SPI_FILLING,
	BUF_READY_FOR_USB,
	BUF_USB_FILLING,
	BUF_READY_FOR_SPI,
};
extern int mmc_buf_state[2];

extern int      mmc_bio_status;
extern uint32_t mmc_bio_error;   // raw INT_STATUS when error occurred
extern int      mmc_is_read;
extern uint32_t mmc_cur_lba;
extern uint16_t mmc_blocks_total;
extern uint16_t mmc_blocks_issued;
extern uint16_t mmc_blocks_completed;
extern uint16_t mmc_blocks_consumed;
extern int      mmc_active_buf;

