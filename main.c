// SPDX-License-Identifier: GPL-2.0-only
// main.c - Main program

#include <string.h>
#include "platform.h"
#include "mmc/mmc.h"
#include "mmc/internal.h"
#include "protocol.h"
#include "spi_reg.h"
#include "usb_dev.h"

// LED blink pattern — lowest bit drives LED, rotates right every tick.
// Other tasks set this to indicate state.
// 32 bits at 100ms/bit = 3.2 second cycle
#define LED_PATTERN_USB_WAIT   0x03030303  // 4x single blip
#define LED_PATTERN_IDLE       0x00030003  // 2x single blip
#define LED_PATTERN_STREAMING  0x0F0F0F0F  // fast pulse
#define LED_PATTERN_INIT       0x55555555  // rapid blink
#define LED_PATTERN_ERROR      0x00770077  // 2x double blip

static uint32_t led_mode = LED_PATTERN_USB_WAIT;
static uint32_t led_pattern;
static uint8_t  led_bit;

static void led_set_mode(uint32_t mode)
{
	if (led_mode != mode) {
		led_mode = mode;
		led_pattern = mode;
		led_bit = 0;
	}
}

static void led_task(void)
{
	static uint32_t next_time = 0;

	uint32_t now = time_us_32();
	if ((int32_t)(now - next_time) < 0)
		return;
	next_time = now + 100000;

	led_set(!(led_pattern & 1));
	led_pattern >>= 1;
	if (++led_bit >= 32) {
		led_pattern = led_mode;
		led_bit = 0;
	}
}

static void send_response(uint8_t status, const void *data, uint32_t len)
{
	volatile uint8_t *txp;
	while (!(txp = usb_cmd_tx_buf()))
		;
	txp[0] = status;
	if (data && len)
		memcpy((void *)&txp[1], data, len);
	usb_cmd_tx_submit(1 + len);
	usb_cmd_tx_drain();
}

static void send_data(const uint8_t *data, uint32_t size)
{
	while (size > 0) {
		volatile uint8_t *txp;
		while (!(txp = usb_data_tx_buf()))
			;
		uint32_t chunk = (size > USB_TX_MAX) ? USB_TX_MAX : size;
		memcpy((void *)txp, data, chunk);
		usb_data_tx_submit(chunk);
		data += chunk;
		size -= chunk;
	}
	usb_data_tx_drain();
}

static uint8_t ext_csd_buf[512];

static enum {
	STREAM_IDLE,
	STREAM_BLOCK_READ,
	STREAM_BLOCK_WRITE,
	STREAM_EXT_CSD_READ,
	STREAM_ABORTING,
} stream_state;

static void cmd_task(void)
{
	uint32_t pkt_len;
	volatile uint8_t *pkt = usb_cmd_rx_buf(&pkt_len);
	if (!pkt || pkt_len < 1)
		return;

	uint8_t cmd = pkt[0];
	usb_cmd_rx_consume();

	switch (cmd) {

	// ---- Normal operation ----

	case CMD_GET_VERSION: {
		uint32_t version = PICOFACET_VERSION;
		send_response(STATUS_ACK, &version, sizeof version);
		break;
	}

	case CMD_INIT_MMC:
		// Abort any active stream from a previous session
		if (stream_state != STREAM_IDLE) {
			usb_data_tx_drain();
			uint32_t dlen;
			while (usb_data_rx_buf(&dlen))
				usb_data_rx_consume();
			stream_state = STREAM_IDLE;
		}
		mmc_init_start();
		led_set_mode(LED_PATTERN_INIT);
		send_response(STATUS_ACK, NULL, 0);
		break;

	case CMD_RELEASE_MMC:
		reset_smc_request(0);
		send_response(STATUS_ACK, NULL, 0);
		break;

	case CMD_GET_STATUS: {
		struct __attribute__((packed)) {
			uint8_t  state;
			uint8_t  init_state;
			uint8_t  bio_status;
			uint8_t  stream;
			uint32_t last_error;
			uint16_t blocks_done;
			uint16_t blocks_total;
		} resp;
		int st = mmc_init_status();
		switch (st) {
		case MMC_IDLE:  resp.state = DEV_STATE_IDLE; break;
		case MMC_BUSY:  resp.state = DEV_STATE_INITIALIZING; break;
		case MMC_DONE:  resp.state = DEV_STATE_READY; led_set_mode(LED_PATTERN_IDLE); break;
		default:        resp.state = DEV_STATE_ERROR; led_set_mode(LED_PATTERN_ERROR); break;
		}
		resp.init_state = (uint8_t)mmc_init_error_state();
		resp.bio_status = (uint8_t)mmc_read_status();
		resp.stream = (uint8_t)stream_state;
		resp.last_error = mmc_last_error();
		resp.blocks_done = mmc_blocks_completed;
		resp.blocks_total = mmc_blocks_total;
		send_response(STATUS_ACK, &resp, sizeof resp);
		break;
	}

	case CMD_READ_CID:
		send_response(STATUS_ACK, NULL, 0);
		send_data(mmc_get_cid(), 16);
		break;

	case CMD_READ_CSD:
		send_response(STATUS_ACK, NULL, 0);
		send_data(mmc_get_csd(), 16);
		break;

	case CMD_READ_EXT_CSD:
		mmc_read_ext_csd_start(ext_csd_buf);
		stream_state = STREAM_EXT_CSD_READ;
		send_response(STATUS_ACK, NULL, 0);
		break;

	case CMD_READ_FUSES: {
		static const uint8_t fuse_regs[12] = {
			0xC0, 0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7,
			0xCA, 0xCB, 0xCC, 0xCD,
		};

		uint32_t fuses[12];
		for (int i = 0; i < 12; i++) {
			uint32_t val = 0;
			spi_reg_read(fuse_regs[i], (uint8_t *)&val, 4);
			fuses[i] = val;
		}

		send_response(STATUS_ACK, NULL, 0);
		send_data((const uint8_t *)fuses, 48);
		break;
	}

	case CMD_BLOCK_READ:
		if (pkt_len >= 7) {
			uint32_t lba;
			uint16_t count;
			memcpy(&lba, (void *)&pkt[1], 4);
			memcpy(&count, (void *)&pkt[5], 2);
			mmc_read_start(lba, count);
			if (stream_state != STREAM_BLOCK_READ)
				led_set_mode(LED_PATTERN_STREAMING);
			stream_state = STREAM_BLOCK_READ;
			send_response(STATUS_ACK, NULL, 0);
		} else {
			send_response(STATUS_NAK, NULL, 0);
		}
		break;

	case CMD_BLOCK_WRITE:
		if (pkt_len >= 7) {
			uint32_t lba;
			uint16_t count;
			memcpy(&lba, (void *)&pkt[1], 4);
			memcpy(&count, (void *)&pkt[5], 2);
			mmc_write_start(lba, count);
			if (stream_state != STREAM_BLOCK_WRITE)
				led_set_mode(LED_PATTERN_STREAMING);
			stream_state = STREAM_BLOCK_WRITE;
			send_response(STATUS_ACK, NULL, 0);
		} else {
			send_response(STATUS_NAK, NULL, 0);
		}
		break;

	case CMD_ABORT:
		usb_data_tx_drain();
		// Discard pending write data
		{ uint32_t dlen; while (usb_data_rx_buf(&dlen)) usb_data_rx_consume(); }
		mmc_abort_start();
		stream_state = STREAM_ABORTING;
		led_set_mode(LED_PATTERN_IDLE);
		send_response(STATUS_ACK, NULL, 0);
		break;

	// ---- Debug ----

	case CMD_RESET_SMC:
		if (pkt_len >= 2) {
			reset_smc_request(pkt[1]);
			send_response(STATUS_ACK, NULL, 0);
		} else {
			send_response(STATUS_NAK, NULL, 0);
		}
		break;

	case CMD_RESET_SMC_DONE: {
		uint8_t done = reset_smc_done();
		send_response(STATUS_ACK, &done, sizeof done);
		break;
	}

	case CMD_SPI_REG_READ:
		if (pkt_len >= 6) {
			uint8_t addr = pkt[1];
			uint32_t size;
			memcpy(&size, (void *)&pkt[2], 4);

			static uint8_t buf[4096];
			spi_reg_read(addr, buf, size);
			send_response(STATUS_ACK, NULL, 0);
			send_data(buf, size);
		} else {
			send_response(STATUS_NAK, NULL, 0);
		}
		break;

	case CMD_SPI_REG_WRITE:
		if (pkt_len >= 6) {
			uint8_t addr = pkt[1];
			uint32_t val;
			memcpy(&val, (void *)&pkt[2], 4);
			spi_reg_write(addr, val);
			send_response(STATUS_ACK, NULL, 0);
		} else {
			send_response(STATUS_NAK, NULL, 0);
		}
		break;

	case CMD_SPI_SET_FREQ:
		if (pkt_len >= 5) {
			uint32_t freq_hz;
			memcpy(&freq_hz, (void *)&pkt[1], 4);
			spi_reg_set_frequency(freq_hz);
			send_response(STATUS_ACK, NULL, 0);
		} else {
			send_response(STATUS_NAK, NULL, 0);
		}
		break;

	default:
		send_response(STATUS_NAK, NULL, 0);
		break;
	}
}

static void stream_task(void)
{
	static const uint8_t *usb_block;
	static uint32_t usb_sent;

	switch (stream_state) {
	case STREAM_BLOCK_READ: {
		int bio = mmc_read_status();
		if (bio == MMC_ERROR) {
			usb_data_tx_drain();
			stream_state = STREAM_IDLE;
			led_set_mode(LED_PATTERN_ERROR);
			return;
		}

		if (!usb_block) {
			usb_block = mmc_read_get_buf();
			usb_sent = 0;
		}

		if (usb_block) {
			volatile uint8_t *txp = usb_data_tx_buf();
			if (txp) {
				uint32_t remain = 512 - usb_sent;
				uint32_t chunk = remain > USB_TX_MAX ? USB_TX_MAX : remain;
				memcpy((void *)txp, usb_block + usb_sent, chunk);
				usb_data_tx_submit(chunk);
				usb_sent += chunk;
				if (usb_sent >= 512) {
					mmc_read_consume();
					usb_block = 0;
					usb_sent = 0;
				}
			}
		}

		if (bio == MMC_DONE && !usb_block)
			stream_state = STREAM_IDLE;
		break;
	}

	case STREAM_BLOCK_WRITE: {
		int bio = mmc_write_status();
		if (bio == MMC_ERROR || bio == MMC_DONE) {
			stream_state = STREAM_IDLE;
			if (bio == MMC_ERROR)
				led_set_mode(LED_PATTERN_ERROR);
			return;
		}
		// TODO: implement block write using usb_data_rx_buf/consume
		break;
	}

	case STREAM_EXT_CSD_READ: {
		mmc_read_ext_csd_task();
		int st = mmc_read_ext_csd_status();
		if (st == MMC_DONE) {
			send_data(ext_csd_buf, 512);
			stream_state = STREAM_IDLE;
		} else if (st == MMC_ERROR) {
			stream_state = STREAM_IDLE;
		}
		break;
	}

	case STREAM_ABORTING:
		if (mmc_abort_status() != MMC_BUSY)
			stream_state = STREAM_IDLE;
		break;

	default:
		break;
	}
}

int main(void)
{
	usb_dev_init();
	platform_init();
	spi_reg_init();

	while (!usb_dev_configured())
		led_task();
	led_set_mode(LED_PATTERN_IDLE);

	for (;;) {
		cmd_task();
		mmc_read_task();
		mmc_write_task();
		stream_task();
		mmc_abort_task();
		mmc_init_task();
		led_task();
		reset_smc_task();
	}
}
