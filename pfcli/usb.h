// SPDX-License-Identifier: GPL-2.0-only
// usb.h - USB transport layer for PicoFacet

#pragma once
#include <stdint.h>
#include <stddef.h>

// Open the PicoFacet USB device. Returns 0 on success.
int pf_usb_open(void);
void pf_usb_close(void);

// Send a command packet [cmd, args...] and read one response packet.
// Returns payload length on success, -1 on error.
int pf_cmd(uint8_t cmd, const void *arg, size_t arg_size,
           void *resp, size_t resp_size);

// Bulk data channel
int pf_data_read(void *buf, size_t size);
int pf_data_write(const void *buf, size_t size);
void pf_data_drain(void);

// Abort active stream
int pf_abort(void);
