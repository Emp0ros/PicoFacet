// SPDX-License-Identifier: GPL-2.0-only
// device.h - PicoFacet device protocol

#pragma once
#include <stdint.h>

int pf_get_version(uint32_t *version);
int pf_init_mmc(void);
int pf_release_mmc(void);

int pf_read_cid(uint8_t *cid);         // 16 bytes
int pf_read_csd(uint8_t *csd);         // 16 bytes
int pf_read_ext_csd(uint8_t *ext_csd); // 512 bytes
int pf_read_fuses(uint32_t *fuses);    // 12 dwords

int pf_block_read(uint32_t lba, uint16_t count, void *buf);
int pf_block_write(uint32_t lba, uint16_t count, const void *buf);

// Query device status including last MMC error
int pf_get_status(uint8_t *state, uint8_t *init_state, uint32_t *last_error);

void pf_print_mmc_error(uint32_t status);
