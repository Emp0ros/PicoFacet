// SPDX-License-Identifier: GPL-2.0-only
// usb_dev.c - CH32V305 USBHS bulk device driver for PicoFacet
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

#include "ch32v30x.h"
#include "ch32v30x_usb.h"
#include "ch32v30x_rcc.h"
#include <string.h>

#ifndef USBHS_SPEED_TYPE_MASK
#define USBHS_SPEED_TYPE_MASK  ((uint8_t)(0x03))
#define USBHS_SPEED_FULL       ((uint8_t)(0x00))
#define USBHS_SPEED_HIGH       ((uint8_t)(0x01))
#endif

static void rcc_init(void);

// ======================================================================
// EP0 enumeration state
// ======================================================================

static const uint8_t *desc_ptr;        // descriptor being streamed to host
static volatile uint16_t desc_remain;  // bytes remaining to send
static volatile uint8_t  last_req;     // for SET_ADDRESS deferred handling
static volatile uint8_t  dev_config;
static volatile uint8_t  dev_addr;
static volatile uint8_t  dev_speed;
static volatile uint8_t  dev_configured;

// ======================================================================
// Buffers — all endpoints double-buffered
// ======================================================================

static __attribute__((aligned(4))) uint8_t ep0_buf[USB_EP0_SIZE];

// EP1 OUT (cmd RX): double-buffered via TX_DMA/RX_DMA trick
static __attribute__((aligned(4))) uint8_t ep1_buf0[USB_HS_BULK_SIZE];
static __attribute__((aligned(4))) uint8_t ep1_buf1[USB_HS_BULK_SIZE];

// EP2 IN (cmd TX): double-buffered
static __attribute__((aligned(4))) uint8_t ep2_buf0[USB_HS_BULK_SIZE];
static __attribute__((aligned(4))) uint8_t ep2_buf1[USB_HS_BULK_SIZE];

// EP3 OUT (data RX): double-buffered
static __attribute__((aligned(4))) uint8_t ep3_buf0[USB_HS_BULK_SIZE];
static __attribute__((aligned(4))) uint8_t ep3_buf1[USB_HS_BULK_SIZE];

// EP4 IN (data TX): double-buffered
static __attribute__((aligned(4))) uint8_t ep4_buf0[USB_HS_BULK_SIZE];
static __attribute__((aligned(4))) uint8_t ep4_buf1[USB_HS_BULK_SIZE];

static uint8_t cfg_buf[USB_CONFIG_DESCRIPTOR_SIZE];

// ======================================================================
// Double-buffered RX state (EP1 OUT, EP3 OUT)
// ======================================================================
// Hardware alternates between buf0 and buf1 via auto-toggle.
// ISR marks the just-received buffer as ready with its length.
// Caller reads it, then calls consume() to release it back.

typedef struct {
	volatile uint8_t  ready[2];
	volatile uint16_t len[2];
	volatile uint8_t  hw_sel;   // which buffer hardware receives into next
	uint8_t           app_sel;  // which buffer the caller reads next
} rx_state_t;

static rx_state_t cmd_rx;
static rx_state_t data_rx;

// ======================================================================
// Double-buffered TX state (EP2 IN, EP4 IN)
// ======================================================================

typedef struct {
	volatile uint8_t free[2];
	volatile uint8_t hw_sel;    // which buffer the ISR retires next
	uint8_t          fill_sel;  // which buffer the caller fills next
} tx_state_t;

static tx_state_t cmd_tx;
static tx_state_t data_tx;

// ======================================================================
// Endpoint init
// ======================================================================

static void endp_init(void)
{
	USBHSD->ENDP_CONFIG = USBHS_UEP1_R_EN | USBHS_UEP2_T_EN
			     | USBHS_UEP3_R_EN | USBHS_UEP4_T_EN;

	// All endpoints double-buffered
	USBHSD->BUF_MODE = USBHS_UEP1_BUF_MOD | USBHS_UEP2_BUF_MOD
			  | USBHS_UEP3_BUF_MOD | USBHS_UEP4_BUF_MOD;

	USBHSD->UEP0_MAX_LEN = USB_EP0_SIZE;
	USBHSD->UEP1_MAX_LEN = USB_HS_BULK_SIZE;
	USBHSD->UEP2_MAX_LEN = USB_HS_BULK_SIZE;
	USBHSD->UEP3_MAX_LEN = USB_HS_BULK_SIZE;
	USBHSD->UEP4_MAX_LEN = USB_HS_BULK_SIZE;

	// EP0
	USBHSD->UEP0_DMA = (uint32_t)ep0_buf;
	USBHSD->UEP0_TX_LEN  = 0;
	USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_RES_NAK;
	USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_RES_ACK;

	// Double-buffered RX: TOG[0]=0 → RX_DMA (buf0), TOG[0]=1 → TX_DMA (buf1)
	// Double-buffered TX: TOG[0]=0 → TX_DMA (buf0), TOG[0]=1 → RX_DMA (buf1)

	// EP1 OUT (cmd RX)
	USBHSD->UEP1_RX_DMA = (uint32_t)ep1_buf0;
	USBHSD->UEP1_TX_DMA = (uint32_t)ep1_buf1;
	USBHSD->UEP1_RX_CTRL = USBHS_UEP_R_RES_ACK | USBHS_UEP_R_TOG_AUTO;

	// EP2 IN (cmd TX)
	USBHSD->UEP2_TX_DMA = (uint32_t)ep2_buf0;
	USBHSD->UEP2_RX_DMA = (uint32_t)ep2_buf1;
	USBHSD->UEP2_TX_LEN  = 0;
	USBHSD->UEP2_TX_CTRL = USBHS_UEP_T_RES_NAK | USBHS_UEP_T_TOG_AUTO;

	// EP3 OUT (data RX)
	USBHSD->UEP3_RX_DMA = (uint32_t)ep3_buf0;
	USBHSD->UEP3_TX_DMA = (uint32_t)ep3_buf1;
	USBHSD->UEP3_RX_CTRL = USBHS_UEP_R_RES_ACK | USBHS_UEP_R_TOG_AUTO;

	// EP4 IN (data TX)
	USBHSD->UEP4_TX_DMA = (uint32_t)ep4_buf0;
	USBHSD->UEP4_RX_DMA = (uint32_t)ep4_buf1;
	USBHSD->UEP4_TX_LEN  = 0;
	USBHSD->UEP4_TX_CTRL = USBHS_UEP_T_RES_NAK | USBHS_UEP_T_TOG_AUTO;

	// Init RX state
	memset(&cmd_rx, 0, sizeof(cmd_rx));
	memset(&data_rx, 0, sizeof(data_rx));

	// Init TX state
	cmd_tx.free[0] = cmd_tx.free[1] = 1;
	cmd_tx.fill_sel = 0;
	cmd_tx.hw_sel = 0;
	data_tx.free[0] = data_tx.free[1] = 1;
	data_tx.fill_sel = 0;
	data_tx.hw_sel = 0;
}

// ======================================================================
// USB init
// ======================================================================

void usb_dev_init(void)
{
	rcc_init();

	USBHSD->CONTROL = USBHS_UC_CLR_ALL | USBHS_UC_RESET_SIE;
	Delay_Us(10);
	USBHSD->CONTROL &= ~USBHS_UC_RESET_SIE;

	USBHSD->HOST_CTRL = USBHS_UH_PHY_SUSPENDM;
	USBHSD->CONTROL = USBHS_UC_DMA_EN | USBHS_UC_INT_BUSY |
			   USBHS_UC_SPEED_HIGH;
	USBHSD->INT_EN = USBHS_UIE_SETUP_ACT | USBHS_UIE_TRANSFER |
			  USBHS_UIE_DETECT | USBHS_UIE_SUSPEND;

	endp_init();

	USBHSD->CONTROL |= USBHS_UC_DEV_PU_EN;
	dev_configured = 0;

	NVIC_EnableIRQ(USBHS_IRQn);
}

static void rcc_init(void)
{
	RCC_USBCLK48MConfig(RCC_USBCLK48MCLKSource_USBPHY);
	RCC_USBHSPLLCLKConfig(RCC_HSBHSPLLCLKSource_HSE);
	RCC_USBHSConfig(RCC_USBPLL_Div2);
	RCC_USBHSPLLCKREFCLKConfig(RCC_USBHSPLLCKREFCLK_4M);
	RCC_USBHSPHYPLLALIVEcmd(ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHS, ENABLE);
}

// ======================================================================
// ISR helpers for double-buffered endpoints
// ======================================================================

static inline void rx_isr(rx_state_t *rx, uint16_t len, volatile uint8_t *rx_ctrl)
{
	rx->len[rx->hw_sel] = len;
	rx->ready[rx->hw_sel] = 1;
	rx->hw_sel ^= 1u;

	if (rx->ready[0] && rx->ready[1])
		*rx_ctrl = (*rx_ctrl & ~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_NAK;
}

static inline void tx_isr(tx_state_t *tx, volatile uint8_t *tx_ctrl)
{
	tx->free[tx->hw_sel] = 1;
	tx->hw_sel ^= 1u;

	if (tx->free[tx->hw_sel])
		*tx_ctrl = (*tx_ctrl & ~USBHS_UEP_T_RES_MASK) | USBHS_UEP_T_RES_NAK;
}

// ======================================================================
// EP0 setup packet handling
// ======================================================================

static void handle_setup_packet(void)
{
	USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_NAK;
	USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_NAK;

	struct usb_setup_packet *pkt = (struct usb_setup_packet *)ep0_buf;
	last_req = pkt->bRequest;
	desc_remain = 0;

	if (pkt->bmRequestType == 0x00) {
		/* Host-to-device standard requests */
		if (pkt->bRequest == USB_REQ_SET_ADDRESS) {
			dev_addr = pkt->wValue & 0xFF;
		} else if (pkt->bRequest == USB_REQ_SET_CONFIGURATION) {
			dev_config = pkt->wValue & 0xFF;
			dev_configured = 1;
		}
		USBHSD->UEP0_TX_LEN = 0;
		USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;

	} else if (pkt->bmRequestType == 0x80) {
		/* Device-to-host standard requests */
		const uint8_t *data = NULL;
		uint16_t len = 0;

		if (pkt->bRequest == USB_REQ_GET_DESCRIPTOR) {
			uint8_t desc_type = pkt->wValue >> 8;
			uint8_t desc_idx  = pkt->wValue & 0xFF;

			switch (desc_type) {
			case USB_DT_DEVICE:
				data = (const uint8_t *)&usb_dev_desc;
				len = sizeof(usb_dev_desc);
				break;
			case USB_DT_CONFIG: {
				uint16_t ep_size = USB_HS_BULK_SIZE;
				if ((USBHSD->SPEED_TYPE & USBHS_SPEED_TYPE_MASK) != USBHS_SPEED_HIGH)
					ep_size = USB_FS_BULK_SIZE;
				len = usb_build_config_desc(cfg_buf, ep_size);
				data = cfg_buf;
				break;
			}
			case USB_DT_STRING:
				if (desc_idx < usb_string_descs_count
				    && usb_string_descs[desc_idx]) {
					data = usb_string_descs[desc_idx];
					len = data[0];
				}
				break;
			case USB_DT_QUALIFIER:
				data = (const uint8_t *)&usb_qual_desc;
				len = sizeof(usb_qual_desc);
				break;
			}
		}

		if (data) {
			if (len > pkt->wLength)
				len = pkt->wLength;
			desc_ptr = data;
			desc_remain = len;
			uint16_t chunk = (len >= USB_EP0_SIZE) ? USB_EP0_SIZE : len;
			memcpy(ep0_buf, desc_ptr, chunk);
			desc_ptr += chunk;
			desc_remain -= chunk;
			USBHSD->UEP0_TX_LEN = chunk;
			USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
		} else {
			USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_STALL;
			USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 | USBHS_UEP_R_RES_STALL;
		}
	} else {
		USBHSD->UEP0_TX_LEN = 0;
		USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 | USBHS_UEP_T_RES_ACK;
	}
}

// ======================================================================
// ISR
// ======================================================================

void __attribute__((interrupt)) USBHS_IRQHandler(void)
{
	uint8_t intflag = USBHSD->INT_FG;
	uint8_t intst   = USBHSD->INT_ST;

	if (intflag & USBHS_UIF_TRANSFER) {
		switch (intst & USBHS_UIS_TOKEN_MASK) {

		case USBHS_UIS_TOKEN_IN:
			switch (intst & (USBHS_UIS_TOKEN_MASK | USBHS_UIS_ENDP_MASK)) {

			case USBHS_UIS_TOKEN_IN | 0x00: // EP0 IN
				if (desc_remain == 0) {
					// Status stage — arm EP0 OUT for ZLP
					USBHSD->UEP0_RX_CTRL = USBHS_UEP_R_TOG_DATA1 |
								USBHS_UEP_R_RES_ACK;
					if (last_req == USB_REQ_SET_ADDRESS)
						USBHSD->DEV_AD = dev_addr;
				} else {
					// Continuation — send next chunk of descriptor
					uint16_t len = (desc_remain >= USB_EP0_SIZE)
						     ? USB_EP0_SIZE : desc_remain;
					memcpy(ep0_buf, desc_ptr, len);
					desc_remain -= len;
					desc_ptr += len;
					USBHSD->UEP0_TX_LEN = len;
					USBHSD->UEP0_TX_CTRL ^= USBHS_UEP_T_TOG_DATA1;
				}
				break;

			case USBHS_UIS_TOKEN_IN | 0x02: // EP2 IN complete (cmd TX)
				tx_isr(&cmd_tx, &USBHSD->UEP2_TX_CTRL);
				break;

			case USBHS_UIS_TOKEN_IN | 0x04: // EP4 IN complete (data TX)
				tx_isr(&data_tx, &USBHSD->UEP4_TX_CTRL);
				break;
			}
			break;

		case USBHS_UIS_TOKEN_OUT:
			switch (intst & (USBHS_UIS_TOKEN_MASK | USBHS_UIS_ENDP_MASK)) {

			case USBHS_UIS_TOKEN_OUT | 0x00: // EP0 OUT
				if (intst & USBHS_UIS_TOG_OK) {
					desc_remain -= USBHSD->RX_LEN;
					if (desc_remain == 0) {
						USBHSD->UEP0_TX_LEN  = 0;
						USBHSD->UEP0_TX_CTRL = USBHS_UEP_T_TOG_DATA1 |
									USBHS_UEP_T_RES_ACK;
					}
				}
				break;

			case USBHS_UIS_TOKEN_OUT | 0x01: // EP1 OUT (cmd RX)
				if (intst & USBHS_UIS_TOG_OK)
					rx_isr(&cmd_rx, USBHSD->RX_LEN, &USBHSD->UEP1_RX_CTRL);
				break;

			case USBHS_UIS_TOKEN_OUT | 0x03: // EP3 OUT (data RX)
				if (intst & USBHS_UIS_TOG_OK)
					rx_isr(&data_rx, USBHSD->RX_LEN, &USBHSD->UEP3_RX_CTRL);
				break;
			}
			break;

		case USBHS_UIS_TOKEN_SOF:
			break;
		}
		USBHSD->INT_FG = USBHS_UIF_TRANSFER;
	}
	else if (intflag & USBHS_UIF_SETUP_ACT) {
		handle_setup_packet();
		USBHSD->INT_FG = USBHS_UIF_SETUP_ACT;
	}
	else if (intflag & USBHS_UIF_BUS_RST) {
		dev_config = 0;
		dev_addr = 0;
		dev_configured = 0;
		USBHSD->DEV_AD = 0;
		endp_init();
		USBHSD->INT_FG = USBHS_UIF_BUS_RST;
	}
	else if (intflag & USBHS_UIF_SUSPEND) {
		USBHSD->INT_FG = USBHS_UIF_SUSPEND;
	}
	else {
		USBHSD->INT_FG = intflag;
	}
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
	return s ? ep1_buf1 : ep1_buf0;
}

void usb_cmd_rx_consume(void)
{
	cmd_rx.ready[cmd_rx.app_sel] = 0;
	cmd_rx.app_sel ^= 1u;
	USBHSD->UEP1_RX_CTRL = (USBHSD->UEP1_RX_CTRL &
		~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
}

volatile uint8_t *usb_cmd_tx_buf(void)
{
	if (!cmd_tx.free[cmd_tx.fill_sel])
		return NULL;
	return cmd_tx.fill_sel ? ep2_buf1 : ep2_buf0;
}

void usb_cmd_tx_submit(uint32_t len)
{
	cmd_tx.free[cmd_tx.fill_sel] = 0;
	cmd_tx.fill_sel ^= 1u;
	USBHSD->UEP2_TX_LEN = len;
	USBHSD->UEP2_TX_CTRL = (USBHSD->UEP2_TX_CTRL & ~USBHS_UEP_T_RES_MASK)
			      | USBHS_UEP_T_RES_ACK;
}

void usb_cmd_tx_drain(void)
{
	while (!cmd_tx.free[0] || !cmd_tx.free[1])
		;
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
	return s ? ep3_buf1 : ep3_buf0;
}

void usb_data_rx_consume(void)
{
	data_rx.ready[data_rx.app_sel] = 0;
	data_rx.app_sel ^= 1u;
	USBHSD->UEP3_RX_CTRL = (USBHSD->UEP3_RX_CTRL &
		~USBHS_UEP_R_RES_MASK) | USBHS_UEP_R_RES_ACK;
}

volatile uint8_t *usb_data_tx_buf(void)
{
	if (!data_tx.free[data_tx.fill_sel])
		return NULL;
	return data_tx.fill_sel ? ep4_buf1 : ep4_buf0;
}

void usb_data_tx_submit(uint32_t len)
{
	data_tx.free[data_tx.fill_sel] = 0;
	data_tx.fill_sel ^= 1u;
	USBHSD->UEP4_TX_LEN = len;
	USBHSD->UEP4_TX_CTRL = (USBHSD->UEP4_TX_CTRL & ~USBHS_UEP_T_RES_MASK)
			      | USBHS_UEP_T_RES_ACK;
}

void usb_data_tx_drain(void)
{
	while (!data_tx.free[0] || !data_tx.free[1])
		;
}

bool usb_dev_configured(void)
{
	return dev_configured;
}
