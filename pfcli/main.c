// SPDX-License-Identifier: GPL-2.0-only
// main.c - PicoFacet CLI

#include "usb.h"
#include "device.h"
#include "fuses.h"
#include "../protocol.h"

#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// ======================================================================
// Signal handling — clean up firmware state on SIGINT
// ======================================================================

static volatile sig_atomic_t interrupted;

static void sigint_handler(int sig)
{
	(void)sig;
	interrupted = 1;
}

static void cleanup(void)
{
	fprintf(stderr, "\nInterrupted, aborting...\n");
	pf_abort();
	pf_data_drain();
	pf_usb_close();
}

// ======================================================================
// Helpers
// ======================================================================

#define EXT_CSD_SEC_COUNT        212
#define EXT_CSD_BOOT_SIZE_MULT  226
#define EXT_CSD_RPMB_SIZE_MULT  168

static void print_hex(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++)
		printf("%02X", data[i]);
}

static uint32_t get_sector_count(void)
{
	uint8_t ext_csd[512];
	if (pf_read_ext_csd(ext_csd) < 0)
		return 0;
	return (uint32_t)ext_csd[EXT_CSD_SEC_COUNT] |
	       ((uint32_t)ext_csd[EXT_CSD_SEC_COUNT + 1] << 8) |
	       ((uint32_t)ext_csd[EXT_CSD_SEC_COUNT + 2] << 16) |
	       ((uint32_t)ext_csd[EXT_CSD_SEC_COUNT + 3] << 24);
}

// ======================================================================
// Commands
// ======================================================================

static int cmd_info(void)
{
	uint8_t cid[16], csd[16], ext_csd[512];

	if (pf_read_cid(cid) < 0)
		return fprintf(stderr, "failed to read CID\n"), -1;
	if (pf_read_csd(csd) < 0)
		return fprintf(stderr, "failed to read CSD\n"), -1;
	if (pf_read_ext_csd(ext_csd) < 0)
		return fprintf(stderr, "failed to read EXT_CSD\n"), -1;

	printf("CID: ");
	print_hex(cid, 16);
	printf("\n");

	printf("CSD: ");
	print_hex(csd, 16);
	printf("\n");
	printf("  CSD_STRUCTURE:  %u\n", (csd[14] >> 6) & 0x03);
	printf("  SPEC_VERS:      %u\n", (csd[14] >> 2) & 0x0F);
	printf("  READ_BL_LEN:    %u (%u bytes)\n", csd[9] & 0x0F, 1u << (csd[9] & 0x0F));

	uint32_t sec_count = (uint32_t)ext_csd[EXT_CSD_SEC_COUNT] |
	                     ((uint32_t)ext_csd[EXT_CSD_SEC_COUNT + 1] << 8) |
	                     ((uint32_t)ext_csd[EXT_CSD_SEC_COUNT + 2] << 16) |
	                     ((uint32_t)ext_csd[EXT_CSD_SEC_COUNT + 3] << 24);
	uint64_t dev_bytes = (uint64_t)sec_count * 512;
	uint32_t boot_bytes = (uint32_t)ext_csd[EXT_CSD_BOOT_SIZE_MULT] * 128 * 1024;
	uint32_t rpmb_bytes = (uint32_t)ext_csd[EXT_CSD_RPMB_SIZE_MULT] * 128 * 1024;

	printf("\nEXT_CSD:\n");
	printf("  SEC_COUNT:      %u (%llu MB)\n", sec_count,
	       (unsigned long long)(dev_bytes / (1024 * 1024)));
	printf("  BUS_WIDTH:      %u\n", ext_csd[183]);
	printf("  HS_TIMING:      %u\n", ext_csd[185]);
	printf("  DEVICE_TYPE:    0x%02X\n", ext_csd[196]);
	printf("  Boot Area:      %u MB x2\n", boot_bytes / (1024 * 1024));
	printf("  RPMB:           %u MB\n", rpmb_bytes / (1024 * 1024));
	printf("  User Data:      %llu MB\n",
	       (unsigned long long)(dev_bytes / (1024 * 1024)));

	return 0;
}

static int cmd_fuses(void)
{
	uint32_t fuses[12];
	if (pf_read_fuses(fuses) < 0)
		return fprintf(stderr, "failed to read fuses\n"), -1;
	print_fuse_info(fuses);
	return 0;
}

static int cmd_verify(const char *file_path);

#define DUMP_CHUNK 64

static void print_progress(uint32_t lba, uint32_t sec_count,
			   struct timespec *t_start, struct timespec *t_last_print)
{
	struct timespec t_now;
	clock_gettime(CLOCK_MONOTONIC, &t_now);
	double since_print = (t_now.tv_sec - t_last_print->tv_sec) +
		(t_now.tv_nsec - t_last_print->tv_nsec) / 1e9;

	if (since_print < 0.5 && lba < sec_count)
		return;

	*t_last_print = t_now;
	double elapsed = (t_now.tv_sec - t_start->tv_sec) +
		(t_now.tv_nsec - t_start->tv_nsec) / 1e9;
	double mb_done = (double)lba * 512 / (1024 * 1024);
	double mb_total = (double)sec_count * 512 / (1024 * 1024);
	double speed = elapsed > 0 ? mb_done / elapsed : 0;
	uint64_t pct = (uint64_t)lba * 100 / sec_count;

	int bar_width = 30;
	int filled = (int)(pct * bar_width / 100);
	printf("\r  [");
	for (int i = 0; i < bar_width; i++)
		putchar(i < filled ? '#' : '.');
	printf("] %3llu%% %.0f/%.0f MB  %.3f MB/s",
	       (unsigned long long)pct, mb_done, mb_total, speed);

	if (speed > 0 && lba < sec_count) {
		double remain = (mb_total - mb_done) / speed;
		int eta_m = (int)(remain / 60);
		int eta_s = (int)remain % 60;
		printf("  ETA %d:%02d", eta_m, eta_s);
	}
	printf("   ");
	fflush(stdout);
}

static int cmd_dump(const char *out_path, int verify)
{
	uint32_t sec_count = get_sector_count();
	if (sec_count == 0)
		return fprintf(stderr, "failed to get sector count\n"), -1;

	FILE *fp = fopen(out_path, "wb");
	if (!fp)
		return perror("failed to open output file"), -1;

	uint64_t dev_bytes = (uint64_t)sec_count * 512;
	printf("Dumping %u sectors (%llu MB) to %s...\n",
	       sec_count,
	       (unsigned long long)(dev_bytes / (1024 * 1024)),
	       out_path);

	uint8_t chunk_buf[DUMP_CHUNK * 512];
	uint32_t lba = 0;
	struct timespec t_start, t_last_print;
	clock_gettime(CLOCK_MONOTONIC, &t_start);
	t_last_print = t_start;

	int retries = 0;
	int total_errors = 0;
	while (lba < sec_count && !interrupted) {
		uint16_t count = DUMP_CHUNK;
		if (sec_count - lba < count)
			count = sec_count - lba;

		if (pf_block_read(lba, count, chunk_buf) < 0) {
			total_errors++;
			fprintf(stderr, "\nblock_read failed at LBA %u: %s\n",
				lba, strerror(errno));
			pf_abort();
			pf_data_drain();

			{
				struct __attribute__((packed)) {
					uint8_t  state;
					uint8_t  init_state;
					uint8_t  bio_status;
					uint8_t  stream;
					uint32_t last_error;
					uint16_t blocks_done;
					uint16_t blocks_total;
				} st = {0};
				if (pf_cmd(CMD_GET_STATUS, NULL, 0, &st, sizeof st) >= 0) {
					fprintf(stderr, "bio=%d stream=%d blocks=%u/%u INT_STATUS=0x%08X\n",
						st.bio_status, st.stream,
						st.blocks_done, st.blocks_total,
						st.last_error);
					if (st.last_error)
						pf_print_mmc_error(st.last_error);
				}
			}

			if (++retries > 5) {
				fprintf(stderr, "Failed at LBA %u after %d consecutive retries\n",
					lba, retries);
				fclose(fp);
				return -1;
			}
			fprintf(stderr, "Retrying LBA %u (attempt %d)\n", lba, retries);
			continue;
		}
		retries = 0;

		if (fwrite(chunk_buf, 512, count, fp) != count) {
			perror("\nfwrite failed");
			fclose(fp);
			return -1;
		}

		lba += count;
		print_progress(lba, sec_count, &t_start, &t_last_print);
	}

	if (interrupted) {
		fclose(fp);
		return -1;
	}

	printf("\nDump complete: %s", out_path);
	if (total_errors)
		printf(" (%d recovered errors)", total_errors);
	printf("\n");
	fclose(fp);

	if (verify)
		return cmd_verify(out_path);
	return 0;
}

static int cmd_verify(const char *file_path)
{
	uint32_t sec_count = get_sector_count();
	if (sec_count == 0)
		return fprintf(stderr, "failed to get sector count\n"), -1;

	FILE *fp = fopen(file_path, "rb");
	if (!fp)
		return perror("failed to open file for verification"), -1;

	printf("Verifying %s against eMMC (%u sectors)...\n", file_path, sec_count);

	uint8_t chunk_buf[DUMP_CHUNK * 512];
	uint8_t file_buf[DUMP_CHUNK * 512];
	uint32_t lba = 0;
	struct timespec t_start, t_last_print;
	clock_gettime(CLOCK_MONOTONIC, &t_start);
	t_last_print = t_start;

	int retries = 0;
	int mismatches = 0;
	while (lba < sec_count && !interrupted) {
		uint16_t count = DUMP_CHUNK;
		if (sec_count - lba < count)
			count = sec_count - lba;

		if (pf_block_read(lba, count, chunk_buf) < 0) {
			fprintf(stderr, "\nverify read failed at LBA %u: %s\n",
				lba, strerror(errno));
			pf_abort();
			pf_data_drain();

			if (++retries > 5) {
				fprintf(stderr, "Verify failed at LBA %u after %d consecutive retries\n",
					lba, retries);
				fclose(fp);
				return -1;
			}
			fprintf(stderr, "Retrying LBA %u (attempt %d)\n", lba, retries);
			continue;
		}
		retries = 0;

		if (fread(file_buf, 512, count, fp) != count) {
			perror("\nfread failed during verify");
			fclose(fp);
			return -1;
		}

		for (uint16_t i = 0; i < count; i++) {
			if (memcmp(chunk_buf + i * 512, file_buf + i * 512, 512) != 0) {
				fprintf(stderr, "\nMISMATCH at LBA %u\n", lba + i);
				mismatches++;
			}
		}

		lba += count;
		print_progress(lba, sec_count, &t_start, &t_last_print);
	}

	if (interrupted) {
		fclose(fp);
		return -1;
	}

	printf("\nVerify complete: ");
	if (mismatches)
		printf("%d sector(s) differ\n", mismatches);
	else
		printf("OK — all sectors match\n");
	fclose(fp);
	return mismatches ? -1 : 0;
}

static void usage(const char *prog)
{
	fprintf(stderr,
		"Usage: %s <command> [args]\n"
		"\n"
		"Commands:\n"
		"  info              Show card information\n"
		"  fuses             Read and display SMC fuses\n"
		"  dump <file>       Dump eMMC contents to file\n"
		"  dump -v <file>    Dump and verify\n"
		"  verify <file>     Verify file against eMMC\n"
		"  release           Release SMC for system boot\n"
		, prog);
}

// ======================================================================
// Main
// ======================================================================

int main(int argc, char **argv)
{
	if (argc < 2) {
		usage(argv[0]);
		return 1;
	}

	// Validate command and args before touching hardware
	const char *cmd = argv[1];
	if (strcmp(cmd, "info") != 0 &&
	    strcmp(cmd, "fuses") != 0 &&
	    strcmp(cmd, "release") != 0 &&
	    strcmp(cmd, "dump") != 0 &&
	    strcmp(cmd, "verify") != 0) {
		usage(argv[0]);
		return 1;
	}
	if (strcmp(cmd, "dump") == 0) {
		int need = (argc >= 3 && strcmp(argv[2], "-v") == 0) ? 4 : 3;
		if (argc < need) {
			fprintf(stderr, "dump requires an output file\n");
			return 1;
		}
	}
	if (strcmp(cmd, "verify") == 0 && argc < 3) {
		fprintf(stderr, "verify requires a file\n");
		return 1;
	}

	// Open device and initialize
	if (pf_usb_open() < 0) {
		fprintf(stderr, "failed to open device\n");
		return 1;
	}

	signal(SIGINT, sigint_handler);

	int ret = 1;

	uint32_t version;
	if (pf_get_version(&version) < 0) {
		perror("failed to read version");
		goto out;
	}
	printf("PicoFacet version %d\n", version);

	printf("Initializing MMC... ");
	fflush(stdout);
	if (pf_init_mmc() < 0) {
		fprintf(stderr, "failed\n");
		goto out;
	}
	printf("done\n\n");

	// Dispatch
	if (strcmp(cmd, "info") == 0)
		ret = cmd_info() < 0 ? 1 : 0;
	else if (strcmp(cmd, "fuses") == 0)
		ret = cmd_fuses() < 0 ? 1 : 0;
	else if (strcmp(cmd, "dump") == 0) {
		int verify = (argc >= 3 && strcmp(argv[2], "-v") == 0);
		const char *path = argv[verify ? 3 : 2];
		ret = cmd_dump(path, verify) < 0 ? 1 : 0;
	}
	else if (strcmp(cmd, "verify") == 0)
		ret = cmd_verify(argv[2]) < 0 ? 1 : 0;
	else if (strcmp(cmd, "release") == 0)
		ret = pf_release_mmc() < 0 ? 1 : 0;

out:
	if (interrupted)
		cleanup();
	else
		pf_usb_close();
	return interrupted ? 130 : ret;
}
