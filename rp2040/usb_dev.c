// SPDX-License-Identifier: GPL-2.0-only
// usb_dev.c - RP2040 USB FS bulk device driver for PicoFacet
//
// All 4 endpoints are double-buffered. No ring buffers — callers work
// directly with packet buffers via rx_buf/consume and tx_buf/submit.
//
// EP1 OUT: commands from host
// EP2 IN:  command responses to host
// EP3 OUT: block write data from host
// EP4 IN:  block read data to host

#include "usb_dev.h"
#include "usb_descriptors.h"

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/regs/usb.h"
#include "hardware/structs/usb.h"
#include "hardware/irq.h"
#include "hardware/resets.h"
#include "hardware/sync.h"

#define usb_hw_set   ((usb_hw_t *)hw_set_alias_untyped(usb_hw))
#define usb_hw_clear ((usb_hw_t *)hw_clear_alias_untyped(usb_hw))

#define USB_DIR_OUT 0x00
#define USB_DIR_IN  0x80

#define EP_PKT_SIZE 64  // FS bulk max

// ======================================================================
// DPRAM layout (64 bytes per buffer)
// ======================================================================
// EP1 OUT buf0: epx_data[0..63]
// EP1 OUT buf1: epx_data[64..127]
// EP2 IN  buf0: epx_data[128..191]
// EP2 IN  buf1: epx_data[192..255]
// EP3 OUT buf0: epx_data[256..319]
// EP3 OUT buf1: epx_data[320..383]
// EP4 IN  buf0: epx_data[384..447]
// EP4 IN  buf1: epx_data[448..511]
// Total: 512 bytes of 3840 available

#define EP1_OUT_BUF0  (&usb_dpram->epx_data[0])
#define EP1_OUT_BUF1  (&usb_dpram->epx_data[64])
#define EP2_IN_BUF0   (&usb_dpram->epx_data[128])
#define EP2_IN_BUF1   (&usb_dpram->epx_data[192])
#define EP3_OUT_BUF0  (&usb_dpram->epx_data[256])
#define EP3_OUT_BUF1  (&usb_dpram->epx_data[320])
#define EP4_IN_BUF0   (&usb_dpram->epx_data[384])
#define EP4_IN_BUF1   (&usb_dpram->epx_data[448])

// struct usb_setup_packet is defined in usb_descriptors.h

// ======================================================================
// Double-buffered state — same structs as CH32V305 driver
// ======================================================================

typedef struct {
	volatile uint8_t  ready[2];
	volatile uint16_t len[2];
	volatile uint8_t  hw_sel;
	uint8_t           app_sel;
} rx_state_t;

typedef struct {
	volatile uint8_t free[2];
	volatile uint8_t hw_sel;
	uint8_t          fill_sel;
} tx_state_t;

static rx_state_t cmd_rx;
static rx_state_t data_rx;
static tx_state_t cmd_tx;
static tx_state_t data_tx;

// ======================================================================
// Device state
// ======================================================================

static volatile bool configured;
static bool should_set_address;
static uint8_t dev_addr;
static uint8_t ep0_buf[64];

// ======================================================================
// Helpers
// ======================================================================

static inline uint32_t usb_buffer_offset(volatile uint8_t *buf)
{
	return (uint32_t)buf ^ (uint32_t)usb_dpram;
}

// ======================================================================
// Double-buffered OUT arming (16-bit writes to each half of buf_ctrl)
// ======================================================================
// RP2040 double-buffered endpoints: buf_ctrl is split into two 16-bit halves.
// Low half = buf0, high half = buf1. Each is armed independently.
// PID toggling is handled by hardware in double-buffered mode.

static void __not_in_flash_func(arm_out_slot)(volatile uint16_t *half)
{
	uint16_t val = EP_PKT_SIZE | (uint16_t)USB_BUF_CTRL_AVAIL;
	*half = val;
}

static void __not_in_flash_func(arm_in_slot)(volatile uint16_t *half, uint32_t len)
{
	uint16_t val = (uint16_t)len | (uint16_t)USB_BUF_CTRL_FULL;
	*half = val;
	busy_wait_at_least_cycles(12); // RP2040 errata E15
	*half = val | (uint16_t)USB_BUF_CTRL_AVAIL;
}

// ======================================================================
// EP0 control transfer handling
// ======================================================================

static uint8_t ep0_in_pid, ep0_out_pid;

static void ep0_start_transfer(uint8_t *buf, uint16_t len, bool is_in)
{
	uint32_t val = len | USB_BUF_CTRL_AVAIL;

	if (is_in) {
		if (buf && len)
			memcpy((void *)usb_dpram->ep0_buf_a, buf, len);
		val |= USB_BUF_CTRL_FULL;
		val |= ep0_in_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
		ep0_in_pid ^= 1u;
		usb_dpram->ep_buf_ctrl[0].in = val;
	} else {
		val |= ep0_out_pid ? USB_BUF_CTRL_DATA1_PID : USB_BUF_CTRL_DATA0_PID;
		ep0_out_pid ^= 1u;
		usb_dpram->ep_buf_ctrl[0].out = val;
	}
}

static void arm_all_rx(void);

static void handle_setup_packet(void)
{
	volatile struct usb_setup_packet *vpkt =
		(volatile struct usb_setup_packet *)&usb_dpram->setup_packet;

	struct usb_setup_packet pkt;
	memcpy(&pkt, (void *)vpkt, sizeof(pkt));

	ep0_in_pid = 1;

	if (pkt.bmRequestType == 0x00) {
		/* Host-to-device standard requests */
		if (pkt.bRequest == USB_REQ_SET_ADDRESS) {
			dev_addr = pkt.wValue & 0xFF;
			should_set_address = true;
		} else if (pkt.bRequest == USB_REQ_SET_CONFIGURATION) {
			configured = true;
			arm_all_rx();
		}
		ep0_start_transfer(NULL, 0, true);

	} else if (pkt.bmRequestType == 0x80) {
		/* Device-to-host standard requests */
		const uint8_t *data = NULL;
		uint16_t len = 0;

		if (pkt.bRequest == USB_REQ_GET_DESCRIPTOR) {
			uint8_t desc_type = pkt.wValue >> 8;
			uint8_t desc_idx  = pkt.wValue & 0xFF;

			switch (desc_type) {
			case USB_DT_DEVICE:
				data = (const uint8_t *)&usb_dev_desc;
				len = sizeof(usb_dev_desc);
				break;
			case USB_DT_CONFIG:
				len = usb_build_config_desc(ep0_buf, USB_FS_BULK_SIZE);
				data = ep0_buf;
				break;
			case USB_DT_STRING:
				if (desc_idx < usb_string_descs_count
				    && usb_string_descs[desc_idx]) {
					data = usb_string_descs[desc_idx];
					len = data[0];
				}
				break;
			}
		}

		if (data) {
			if (len > pkt.wLength)
				len = pkt.wLength;
			if (data != ep0_buf)
				memcpy(ep0_buf, data, len);
			ep0_start_transfer(ep0_buf, len, true);
		} else {
			ep0_start_transfer(NULL, 0, true);
		}
	} else {
		ep0_start_transfer(NULL, 0, true);
	}
}

// ======================================================================
// Double-buffered RX/TX arm helpers
// ======================================================================

// Arm both slots of a double-buffered OUT endpoint for initial receive
static void arm_rx_ep(volatile uint32_t *buf_ctrl)
{
	volatile uint16_t *lo = (volatile uint16_t *)buf_ctrl;
	volatile uint16_t *hi = lo + 1;
	// Arm buf0 with DATA0, buf1 with DATA1
	*lo = EP_PKT_SIZE | (uint16_t)USB_BUF_CTRL_AVAIL;
	*hi = EP_PKT_SIZE | (uint16_t)USB_BUF_CTRL_AVAIL;
}

static void arm_all_rx(void)
{
	arm_rx_ep(&usb_dpram->ep_buf_ctrl[1].out);  // EP1 OUT
	arm_rx_ep(&usb_dpram->ep_buf_ctrl[3].out);  // EP3 OUT
}

// ======================================================================
// ISR
// ======================================================================

static void __not_in_flash_func(rx_isr)(rx_state_t *rx, uint16_t len,
					volatile uint16_t *half)
{
	rx->len[rx->hw_sel] = len;
	rx->ready[rx->hw_sel] = 1;
	uint8_t done_slot = rx->hw_sel;
	rx->hw_sel ^= 1u;

	// If both full, don't re-arm (NAK by not setting AVAIL)
	if (!rx->ready[rx->hw_sel]) {
		volatile uint16_t *next_half = (done_slot == 0) ? (half + 1) : (half - 1);
		arm_out_slot(next_half);
	}
}

static void __not_in_flash_func(tx_isr)(tx_state_t *tx)
{
	tx->free[tx->hw_sel] = 1;
	tx->hw_sel ^= 1u;
	// Hardware auto-NAKs when no more data; nothing to do here.
}

static void __not_in_flash_func(usb_handle_buff_status)(void)
{
	uint32_t buffers = usb_hw->buf_status;

	/* EP0 IN (bit 0) */
	if (buffers & 0x1u) {
		usb_hw_clear->buf_status = 0x1u;
		if (should_set_address) {
			usb_hw->dev_addr_ctrl = dev_addr;
			should_set_address = false;
		} else {
			ep0_start_transfer(NULL, 0, false);
		}
	}

	/* EP0 OUT (bit 1) */
	if (buffers & 0x2u) {
		usb_hw_clear->buf_status = 0x2u;
	}

	/* EP1 OUT (bit 3) - commands */
	if (buffers & (1u << 3)) {
		usb_hw_clear->buf_status = (1u << 3);
		volatile uint16_t *lo = (volatile uint16_t *)&usb_dpram->ep_buf_ctrl[1].out;
		// Determine which buf completed from hw_sel
		uint16_t bc = cmd_rx.hw_sel ? *(lo + 1) : *lo;
		uint16_t len = bc & USB_BUF_CTRL_LEN_MASK;
		rx_isr(&cmd_rx, len, lo);
	}

	/* EP2 IN (bit 4) - command response complete */
	if (buffers & (1u << 4)) {
		usb_hw_clear->buf_status = (1u << 4);
		tx_isr(&cmd_tx);
	}

	/* EP3 OUT (bit 7) - write data */
	if (buffers & (1u << 7)) {
		usb_hw_clear->buf_status = (1u << 7);
		volatile uint16_t *lo = (volatile uint16_t *)&usb_dpram->ep_buf_ctrl[3].out;
		uint16_t bc = data_rx.hw_sel ? *(lo + 1) : *lo;
		uint16_t len = bc & USB_BUF_CTRL_LEN_MASK;
		rx_isr(&data_rx, len, lo);
	}

	/* EP4 IN (bit 8) - data stream complete */
	if (buffers & (1u << 8)) {
		usb_hw_clear->buf_status = (1u << 8);
		tx_isr(&data_tx);
	}
}

void __not_in_flash_func(isr_usbctrl)(void)
{
	uint32_t status = usb_hw->ints;

	if (status & USB_INTS_SETUP_REQ_BITS) {
		usb_hw_clear->sie_status = USB_SIE_STATUS_SETUP_REC_BITS;
		handle_setup_packet();
	}

	if (status & USB_INTS_BUFF_STATUS_BITS) {
		usb_handle_buff_status();
	}

	if (status & USB_INTS_BUS_RESET_BITS) {
		usb_hw_clear->sie_status = USB_SIE_STATUS_BUS_RESET_BITS;
		dev_addr = 0;
		should_set_address = false;
		usb_hw->dev_addr_ctrl = 0;
		configured = false;
		memset(&cmd_rx, 0, sizeof(cmd_rx));
		memset(&data_rx, 0, sizeof(data_rx));
		cmd_tx.free[0] = cmd_tx.free[1] = 1;
		cmd_tx.hw_sel = cmd_tx.fill_sel = 0;
		data_tx.free[0] = data_tx.free[1] = 1;
		data_tx.hw_sel = data_tx.fill_sel = 0;
	}
}

// ======================================================================
// Init
// ======================================================================

void usb_dev_init(void)
{
	reset_unreset_block_num_wait_blocking(RESET_USBCTRL);
	memset(usb_dpram, 0, sizeof(*usb_dpram));

	irq_set_enabled(USBCTRL_IRQ, true);

	usb_hw->muxing = USB_USB_MUXING_TO_PHY_BITS |
			 USB_USB_MUXING_SOFTCON_BITS;
	usb_hw->pwr = USB_USB_PWR_VBUS_DETECT_BITS |
		      USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
	usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS;
	usb_hw->sie_ctrl = USB_SIE_CTRL_EP0_INT_1BUF_BITS;
	usb_hw->inte = USB_INTS_BUFF_STATUS_BITS |
		       USB_INTS_BUS_RESET_BITS |
		       USB_INTS_SETUP_REQ_BITS;

	// EP1 OUT - command channel, double-buffered
	usb_dpram->ep_ctrl[0].out = EP_CTRL_ENABLE_BITS |
				    EP_CTRL_DOUBLE_BUFFERED_BITS |
				    EP_CTRL_INTERRUPT_PER_BUFFER |
				    (2u << EP_CTRL_BUFFER_TYPE_LSB) |
				    usb_buffer_offset(EP1_OUT_BUF0);

	// EP2 IN - command response, double-buffered
	usb_dpram->ep_ctrl[1].in = EP_CTRL_ENABLE_BITS |
				   EP_CTRL_DOUBLE_BUFFERED_BITS |
				   EP_CTRL_INTERRUPT_PER_BUFFER |
				   (2u << EP_CTRL_BUFFER_TYPE_LSB) |
				   usb_buffer_offset(EP2_IN_BUF0);

	// EP3 OUT - data write channel, double-buffered
	usb_dpram->ep_ctrl[2].out = EP_CTRL_ENABLE_BITS |
				    EP_CTRL_DOUBLE_BUFFERED_BITS |
				    EP_CTRL_INTERRUPT_PER_BUFFER |
				    (2u << EP_CTRL_BUFFER_TYPE_LSB) |
				    usb_buffer_offset(EP3_OUT_BUF0);

	// EP4 IN - data stream, double-buffered
	usb_dpram->ep_ctrl[3].in = EP_CTRL_ENABLE_BITS |
				   EP_CTRL_DOUBLE_BUFFERED_BITS |
				   EP_CTRL_INTERRUPT_PER_BUFFER |
				   (2u << EP_CTRL_BUFFER_TYPE_LSB) |
				   usb_buffer_offset(EP4_IN_BUF0);

	// Init state
	memset(&cmd_rx, 0, sizeof(cmd_rx));
	memset(&data_rx, 0, sizeof(data_rx));
	cmd_tx.free[0] = cmd_tx.free[1] = 1;
	cmd_tx.hw_sel = cmd_tx.fill_sel = 0;
	data_tx.free[0] = data_tx.free[1] = 1;
	data_tx.hw_sel = data_tx.fill_sel = 0;
	configured = false;

	usb_hw_set->sie_ctrl = USB_SIE_CTRL_PULLUP_EN_BITS;
}

bool usb_dev_configured(void)
{
	return configured;
}

// ======================================================================
// Command channel (EP1 OUT / EP2 IN)
// ======================================================================

volatile uint8_t *usb_cmd_rx_buf(uint32_t *len)
{
	uint8_t s = cmd_rx.app_sel;
	if (!cmd_rx.ready[s])
		return NULL;
	*len = cmd_rx.len[s];
	return s ? EP1_OUT_BUF1 : EP1_OUT_BUF0;
}

void usb_cmd_rx_consume(void)
{
	uint8_t s = cmd_rx.app_sel;
	cmd_rx.ready[s] = 0;
	cmd_rx.app_sel ^= 1u;

	// Re-arm the consumed slot
	volatile uint16_t *lo = (volatile uint16_t *)&usb_dpram->ep_buf_ctrl[1].out;
	arm_out_slot(s ? (lo + 1) : lo);
}

volatile uint8_t *usb_cmd_tx_buf(void)
{
	if (!cmd_tx.free[cmd_tx.fill_sel])
		return NULL;
	return cmd_tx.fill_sel ? EP2_IN_BUF1 : EP2_IN_BUF0;
}

void usb_cmd_tx_submit(uint32_t len)
{
	uint8_t s = cmd_tx.fill_sel;
	cmd_tx.free[s] = 0;
	cmd_tx.fill_sel ^= 1u;

	volatile uint16_t *lo = (volatile uint16_t *)&usb_dpram->ep_buf_ctrl[2].in;
	arm_in_slot(s ? (lo + 1) : lo, len);
}

void usb_cmd_tx_drain(void)
{
	while (!cmd_tx.free[0] || !cmd_tx.free[1])
		tight_loop_contents();
}

// ======================================================================
// Data channel (EP3 OUT / EP4 IN)
// ======================================================================

volatile uint8_t *usb_data_rx_buf(uint32_t *len)
{
	uint8_t s = data_rx.app_sel;
	if (!data_rx.ready[s])
		return NULL;
	*len = data_rx.len[s];
	return s ? EP3_OUT_BUF1 : EP3_OUT_BUF0;
}

void usb_data_rx_consume(void)
{
	uint8_t s = data_rx.app_sel;
	data_rx.ready[s] = 0;
	data_rx.app_sel ^= 1u;

	volatile uint16_t *lo = (volatile uint16_t *)&usb_dpram->ep_buf_ctrl[3].out;
	arm_out_slot(s ? (lo + 1) : lo);
}

volatile uint8_t *usb_data_tx_buf(void)
{
	if (!data_tx.free[data_tx.fill_sel])
		return NULL;
	return data_tx.fill_sel ? EP4_IN_BUF1 : EP4_IN_BUF0;
}

void usb_data_tx_submit(uint32_t len)
{
	uint8_t s = data_tx.fill_sel;
	data_tx.free[s] = 0;
	data_tx.fill_sel ^= 1u;

	volatile uint16_t *lo = (volatile uint16_t *)&usb_dpram->ep_buf_ctrl[4].in;
	arm_in_slot(s ? (lo + 1) : lo, len);
}

void usb_data_tx_drain(void)
{
	while (!data_tx.free[0] || !data_tx.free[1])
		tight_loop_contents();
}
