// SPDX-License-Identifier: GPL-2.0-only
// fuses.c - Xbox One SMC fuse interpretation

#include "fuses.h"
#include <stdio.h>
#include <string.h>

static const struct { const char *desc; uint8_t digest[16]; } known_digests[] = {
	{ "Development Mode, SMCFWKey:dev",
	  { 0xC0, 0xDE, 0x15, 0xB9, 0x00, 0x00, 0xFF, 0xFF, 0xA5, 0xA5, 0x5A, 0x5A, 0x12, 0x34, 0xFE, 0xDC } },
	{ "\"FixedBits\" Corner Part",
	  { 0xB6, 0x90, 0x34, 0x6A, 0xEA, 0x9B, 0x7C, 0xAE, 0x4F, 0xB0, 0x60, 0x3B, 0xF3, 0xA2, 0x2A, 0xC8 } },
	{ "Production Mode, SMCFWKey:rtlA",
	  { 0x2C, 0x02, 0x78, 0xDB, 0xD3, 0x71, 0x6D, 0x19, 0x96, 0xC5, 0xE5, 0xA4, 0x56, 0x0B, 0x3F, 0x6A } },
	{ "Production Mode, SMCFWKey:rtlB",
	  { 0x40, 0x42, 0x7E, 0x91, 0x53, 0xE8, 0x8C, 0xA7, 0xB2, 0xBD, 0x38, 0x12, 0xFE, 0xB6, 0x9B, 0x65 } },
	{ "Production Mode, SMCFWKey:rtlC",
	  { 0xA3, 0x19, 0x29, 0x69, 0xB3, 0xB3, 0x06, 0x8F, 0x12, 0x46, 0xB9, 0xB4, 0xEF, 0x18, 0xE9, 0x9E } },
	{ "Production Mode, SMCFWKey:rtlD",
	  { 0xDF, 0x21, 0x9A, 0xBE, 0x76, 0x0F, 0x9B, 0x32, 0xBC, 0xBE, 0x86, 0xC2, 0x54, 0x01, 0x0F, 0x52 } },
	{ NULL, {0} }
};

static const struct { const char *name; uint32_t chip_id; } sb_gen_map[] = {
	{ "Orion A0", 0x00000000 },
	{ "Orion B0", 0x00000010 },
	{ "Orion B1", 0x00000011 },
	{ NULL, 0 }
};

static void print_hex(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++)
		printf("%02X", data[i]);
}

void print_fuse_info(const uint32_t *fuse_data)
{
	int all_zero = 1;
	for (int i = 0; i < 12; i++)
		if (fuse_data[i]) { all_zero = 0; break; }

	if (all_zero) {
		printf("All fuses read as zero - bypass mode?\n");
		return;
	}

	printf("ECID:            ");
	print_hex((const uint8_t *)&fuse_data[0], 8);
	printf("\n");

	const uint8_t *digest = (const uint8_t *)&fuse_data[2];
	const char *digest_desc = "<unknown>";
	for (int i = 0; known_digests[i].desc; i++) {
		if (memcmp(digest, known_digests[i].digest, 16) == 0) {
			digest_desc = known_digests[i].desc;
			break;
		}
	}
	printf("Exp1SMCBLDigest: %s", digest_desc);
	if (strcmp(digest_desc, "<unknown>") == 0) {
		printf(" (");
		print_hex(digest, 16);
		printf(")");
	}
	printf("\n");

	printf("RsvdPublic:      ");
	print_hex((const uint8_t *)&fuse_data[6], 8);
	printf("\n");

	printf("ChipID:          ");
	print_hex((const uint8_t *)&fuse_data[8], 12);
	printf("\n");

	printf("SB Rev:          ");
	print_hex((const uint8_t *)&fuse_data[11], 4);
	printf("\n");

	uint32_t chip_id = fuse_data[11];
	uint32_t nn = (chip_id >> 8) & 0xF;
	printf("SB Revision:     Orion");
	if (nn) printf("%02u", nn);
	printf(" %c%u\n", (char)(((chip_id >> 4) & 0xF) + 'A'), chip_id & 0xF);

	const char *gen_name = "<unknown>";
	for (int i = 0; sb_gen_map[i].name; i++) {
		if (sb_gen_map[i].chip_id == chip_id) {
			gen_name = sb_gen_map[i].name;
			break;
		}
	}
	printf("SB Generation:   %s\n", gen_name);
}
