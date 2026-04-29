// SPDX-License-Identifier: GPL-2.0-only
// device.c - PicoFacet device protocol

#include "device.h"
#include "usb.h"
#include "../protocol.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>

int pf_get_version(uint32_t *version)
{
	return pf_cmd(CMD_GET_VERSION, NULL, 0, version, sizeof(*version));
}

int pf_init_mmc(void)
{
	if (pf_cmd(CMD_INIT_MMC, NULL, 0, NULL, 0) < 0)
		return -1;

	for (;;) {
		uint8_t state;
		uint8_t init_state;
		uint32_t last_error;
		if (pf_get_status(&state, &init_state, &last_error) < 0)
			return -1;
		if (state == DEV_STATE_READY)
			return 0;
		if (state == DEV_STATE_ERROR) {
			fprintf(stderr, "init failed at state %d\n", init_state);
			pf_print_mmc_error(last_error);
			return -1;
		}
		usleep(10000);
	}
}

int pf_get_status(uint8_t *state, uint8_t *init_state, uint32_t *last_error)
{
	struct __attribute__((packed)) {
		uint8_t  state;
		uint8_t  init_state;
		uint8_t  bio_status;
		uint8_t  stream;
		uint32_t last_error;
	} resp = {0};
	if (pf_cmd(CMD_GET_STATUS, NULL, 0, &resp, sizeof resp) < 0)
		return -1;
	if (state)      *state = resp.state;
	if (init_state) *init_state = resp.init_state;
	if (last_error) *last_error = resp.last_error;
	return 0;
}

void pf_print_mmc_error(uint32_t status)
{
	if (!(status & 0xFFFF0000))
		return;
	fprintf(stderr, "MMC INT_STATUS=0x%08X:", status);
	if (status & (1u << 16)) fprintf(stderr, " CommandTimeout");
	if (status & (1u << 17)) fprintf(stderr, " CommandCrc");
	if (status & (1u << 18)) fprintf(stderr, " CommandEndBit");
	if (status & (1u << 19)) fprintf(stderr, " CommandIndex");
	if (status & (1u << 20)) fprintf(stderr, " DataTimeout");
	if (status & (1u << 21)) fprintf(stderr, " DataCrc");
	if (status & (1u << 22)) fprintf(stderr, " DataEndBit");
	if (status & (1u << 23)) fprintf(stderr, " CurrentLimit");
	if (status & (1u << 24)) fprintf(stderr, " AutoCmd");
	if (status & (1u << 25)) fprintf(stderr, " Adma");
	if (status & (1u << 26)) fprintf(stderr, " Tuning");
	if (status & (1u << 28)) fprintf(stderr, " TargetResponse");
	fprintf(stderr, "\n");
}

int pf_release_mmc(void)
{
	return pf_cmd(CMD_RELEASE_MMC, NULL, 0, NULL, 0) < 0 ? -1 : 0;
}

int pf_read_cid(uint8_t *cid)
{
	if (pf_cmd(CMD_READ_CID, NULL, 0, NULL, 0) < 0)
		return -1;
	return pf_data_read(cid, 16);
}

int pf_read_csd(uint8_t *csd)
{
	if (pf_cmd(CMD_READ_CSD, NULL, 0, NULL, 0) < 0)
		return -1;
	return pf_data_read(csd, 16);
}

int pf_read_ext_csd(uint8_t *ext_csd)
{
	if (pf_cmd(CMD_READ_EXT_CSD, NULL, 0, NULL, 0) < 0)
		return -1;
	return pf_data_read(ext_csd, 512);
}

int pf_read_fuses(uint32_t *fuses)
{
	if (pf_cmd(CMD_READ_FUSES, NULL, 0, NULL, 0) < 0)
		return -1;
	return pf_data_read(fuses, 48);
}

int pf_block_read(uint32_t lba, uint16_t count, void *buf)
{
	struct __attribute__((packed)) {
		uint32_t lba;
		uint16_t count;
	} arg = { lba, count };

	if (pf_cmd(CMD_BLOCK_READ, &arg, sizeof arg, NULL, 0) < 0)
		return -1;
	return pf_data_read(buf, (size_t)count * 512);
}

int pf_block_write(uint32_t lba, uint16_t count, const void *buf)
{
	struct __attribute__((packed)) {
		uint32_t lba;
		uint16_t count;
	} arg = { lba, count };

	if (pf_cmd(CMD_BLOCK_WRITE, &arg, sizeof arg, NULL, 0) < 0)
		return -1;
	return pf_data_write(buf, (size_t)count * 512);
}
