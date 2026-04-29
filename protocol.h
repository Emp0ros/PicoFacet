// SPDX-License-Identifier: GPL-2.0-only
// protocol.h - USB protocol

#pragma once

#define PICOFACET_VERSION 1

enum {
	// Normal operation
	CMD_GET_VERSION     = 0x00,
	CMD_INIT_MMC        = 0x01,  // SMC reset (debug) + Arasan + card init
	CMD_RELEASE_MMC     = 0x02,  // SMC reset (non-debug), release for boot
	CMD_GET_STATUS      = 0x03,  // poll: IDLE / INITIALIZING / READY / ERROR
	CMD_READ_CID        = 0x04,  // 16 bytes on data channel
	CMD_READ_CSD        = 0x05,  // 16 bytes on data channel
	CMD_READ_EXT_CSD    = 0x06,  // 512 bytes on data channel
	CMD_READ_FUSES      = 0x07,  // 48 bytes on data channel
	CMD_BLOCK_READ      = 0x08,  // stream on EP4
	CMD_BLOCK_WRITE     = 0x09,  // receive on EP3
	CMD_ABORT           = 0x0A,  // stop active stream

	// Debug
	CMD_RESET_SMC       = 0x10,  // raw SMC reset (1 byte arg: debug flag)
	CMD_RESET_SMC_DONE  = 0x11,  // poll SMC reset complete
	CMD_SPI_REG_READ    = 0x12,  // read SPI register
	CMD_SPI_REG_WRITE   = 0x13,  // write SPI register
	CMD_SPI_SET_FREQ    = 0x14,  // set SPI clock frequency

	// Status codes (in response packets)
	STATUS_ACK = 0x00,
	STATUS_NAK = 0xFF,

	// Device states (returned by CMD_GET_STATUS)
	DEV_STATE_IDLE         = 0x00,
	DEV_STATE_INITIALIZING = 0x01,
	DEV_STATE_READY        = 0x02,
	DEV_STATE_ERROR        = 0x03,
};
