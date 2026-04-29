// SPDX-License-Identifier: GPL-2.0-only
// usb_dev.h - Custom low-level USB bulk device driver
//
// All endpoints are double-buffered. Callers work directly with packet
// buffers — no ring buffers or byte-stream copying.

#pragma once
#include <stdbool.h>
#include <stdint.h>

void usb_dev_init(void);
bool usb_dev_configured(void);

// ======================================================================
// Command channel: EP1 OUT (host->device) + EP2 IN (device->host)
// ======================================================================

// RX: get pointer to received packet, or NULL if none ready.
// *len is set to the number of bytes received.
volatile uint8_t *usb_cmd_rx_buf(uint32_t *len);
// Release the RX buffer back to hardware for the next receive.
void usb_cmd_rx_consume(void);

// TX: get pointer to a free TX buffer, or NULL if both in flight.
volatile uint8_t *usb_cmd_tx_buf(void);
void usb_cmd_tx_submit(uint32_t len);
void usb_cmd_tx_drain(void);

// ======================================================================
// Data channel: EP3 OUT (host->device) + EP4 IN (device->host)
// ======================================================================

volatile uint8_t *usb_data_rx_buf(uint32_t *len);
void usb_data_rx_consume(void);

volatile uint8_t *usb_data_tx_buf(void);
void usb_data_tx_submit(uint32_t len);
void usb_data_tx_drain(void);
