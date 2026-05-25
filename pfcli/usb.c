// SPDX-License-Identifier: GPL-2.0-only
// usb.c - USB transport layer for PicoFacet

#include "usb.h"
#include "../protocol.h"

#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#ifdef __APPLE__
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif

#define VID 0x600D
#define PID 0x7002
#define USB_TIMEOUT_MS 5000

static libusb_device_handle *g_dev;
static uint8_t EP_CMD_OUT;
static uint8_t EP_CMD_IN;
static uint8_t EP_DATA_OUT;
static uint8_t EP_DATA_IN;

static int bulk_write(uint8_t ep, const void *buf, size_t size)
{
	const unsigned char *p = buf;
	while (size > 0) {
		int transferred = 0;
		int rc = libusb_bulk_transfer(g_dev, ep,
					      (unsigned char *)p, size,
					      &transferred, USB_TIMEOUT_MS);
		if (rc < 0) {
			fprintf(stderr, "USB bulk write (EP 0x%02x): %s\n",
				ep, libusb_strerror(rc));
			errno = EIO;
			return -1;
		}
		p += transferred;
		size -= transferred;
	}
	return 0;
}

static int bulk_read(uint8_t ep, void *buf, size_t size)
{
	unsigned char *p = buf;
	while (size > 0) {
		int transferred = 0;
		int rc = libusb_bulk_transfer(g_dev, ep, p, size,
					      &transferred, USB_TIMEOUT_MS);
		if (rc < 0) {
			fprintf(stderr, "USB bulk read (EP 0x%02x): %s\n",
				ep, libusb_strerror(rc));
			errno = EIO;
			return -1;
		}
		p += transferred;
		size -= transferred;
	}
	return 0;
}

int pf_usb_open(void)
{
	libusb_context *ctx = NULL;
	int rc = libusb_init(&ctx);
	if (rc < 0) {
		fprintf(stderr, "libusb_init: %s\n", libusb_strerror(rc));
		return -1;
	}

	g_dev = libusb_open_device_with_vid_pid(ctx, VID, PID);
	if (!g_dev) {
		fprintf(stderr, "Device %04x:%04x not found\n", VID, PID);
		libusb_exit(ctx);
		return -1;
	}

	if (libusb_kernel_driver_active(g_dev, 0) == 1)
		libusb_detach_kernel_driver(g_dev, 0);

	rc = libusb_claim_interface(g_dev, 0);
	if (rc < 0) {
		fprintf(stderr, "claim interface: %s\n", libusb_strerror(rc));
		libusb_close(g_dev);
		g_dev = NULL;
		libusb_exit(ctx);
		return -1;
	}

	struct libusb_config_descriptor *cfg;
	rc = libusb_get_active_config_descriptor(libusb_get_device(g_dev), &cfg);
	if (rc < 0) {
		fprintf(stderr, "get config descriptor: %s\n", libusb_strerror(rc));
		libusb_close(g_dev);
		g_dev = NULL;
		libusb_exit(ctx);
		return -1;
	}

	EP_CMD_OUT = EP_CMD_IN = EP_DATA_OUT = EP_DATA_IN = 0;
	const struct libusb_interface_descriptor *iface = &cfg->interface[0].altsetting[0];
	for (int i = 0; i < iface->bNumEndpoints; i++) {
		const struct libusb_endpoint_descriptor *ep = &iface->endpoint[i];
		if ((ep->bmAttributes & 0x03) != LIBUSB_TRANSFER_TYPE_BULK)
			continue;
		uint8_t addr = ep->bEndpointAddress;
		uint8_t num  = addr & 0x0F;
		bool is_in   = (addr & 0x80) != 0;

		if (num == 1 && !is_in)      EP_CMD_OUT  = addr;
		else if (num == 2 && is_in)  EP_CMD_IN   = addr;
		else if (num == 3 && !is_in) EP_DATA_OUT = addr;
		else if (num == 4 && is_in)  EP_DATA_IN  = addr;
	}
	libusb_free_config_descriptor(cfg);

	if (!EP_CMD_OUT || !EP_CMD_IN || !EP_DATA_OUT || !EP_DATA_IN) {
		fprintf(stderr, "could not find all 4 bulk endpoints\n");
		libusb_close(g_dev);
		g_dev = NULL;
		return -1;
	}

	return 0;
}

void pf_usb_close(void)
{
	if (g_dev) {
		libusb_release_interface(g_dev, 0);
		libusb_close(g_dev);
		g_dev = NULL;
	}
}

int pf_cmd(uint8_t cmd, const void *arg, size_t arg_size,
           void *resp, size_t resp_size)
{
	uint8_t pkt[512];
	pkt[0] = cmd;
	if (arg_size) memcpy(pkt + 1, arg, arg_size);
	if (bulk_write(EP_CMD_OUT, pkt, 1 + arg_size) < 0)
		return -1;

	uint8_t rbuf[512];
	int transferred = 0;
	int rc = libusb_bulk_transfer(g_dev, EP_CMD_IN, rbuf, sizeof(rbuf),
				      &transferred, USB_TIMEOUT_MS);
	if (rc < 0) {
		fprintf(stderr, "USB bulk read (EP 0x%02x): %s\n",
			EP_CMD_IN, libusb_strerror(rc));
		errno = EIO;
		return -1;
	}
	if (transferred < 1 || rbuf[0] != STATUS_ACK) {
		errno = EPROTO;
		return -1;
	}
	int payload_len = transferred - 1;
	if (resp && payload_len > 0) {
		size_t copy = (size_t)payload_len > resp_size ? resp_size : (size_t)payload_len;
		memcpy(resp, rbuf + 1, copy);
	}
	return payload_len;
}

int pf_data_read(void *buf, size_t size)
{
	return bulk_read(EP_DATA_IN, buf, size);
}

int pf_data_write(const void *buf, size_t size)
{
	return bulk_write(EP_DATA_OUT, buf, size);
}

void pf_data_drain(void)
{
	unsigned char tmp[512];
	int transferred;
	for (int i = 0; i < 64; i++) {
		int rc = libusb_bulk_transfer(g_dev, EP_DATA_IN, tmp, sizeof(tmp),
					      &transferred, 100);
		if (rc == LIBUSB_ERROR_TIMEOUT || transferred == 0)
			break;
	}
}

int pf_abort(void)
{
	return pf_cmd(CMD_ABORT, NULL, 0, NULL, 0) < 0 ? -1 : 0;
}
