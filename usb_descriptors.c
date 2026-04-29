// SPDX-License-Identifier: GPL-2.0-only
// usb_descriptors.c - Shared USB descriptor instances for PicoFacet

#include "usb_descriptors.h"
#include <string.h>

const struct usb_device_descriptor usb_dev_desc = {
	.bLength            = sizeof(struct usb_device_descriptor),
	.bDescriptorType    = USB_DT_DEVICE,
	.bcdUSB             = 0x0200,
	.bDeviceClass       = USB_DEV_CLASS,
	.bDeviceSubClass    = USB_DEV_SUBCLASS,
	.bDeviceProtocol    = USB_DEV_PROTOCOL,
	.bMaxPacketSize0    = USB_EP0_SIZE,
	.idVendor           = USB_VID,
	.idProduct          = USB_PID,
	.bcdDevice          = USB_BCD_DEVICE,
	.iManufacturer      = USB_STR_MANUFACTURER,
	.iProduct           = USB_STR_PRODUCT,
	.iSerialNumber      = USB_STR_SERIAL,
	.bNumConfigurations = 1,
};

const struct usb_interface_descriptor usb_iface_desc = {
	.bLength            = sizeof(struct usb_interface_descriptor),
	.bDescriptorType    = USB_DT_INTERFACE,
	.bInterfaceNumber   = 0,
	.bAlternateSetting  = 0,
	.bNumEndpoints      = USB_NUM_ENDPOINTS,
	.bInterfaceClass    = USB_DEV_CLASS,
	.bInterfaceSubClass = USB_DEV_SUBCLASS,
	.bInterfaceProtocol = USB_DEV_PROTOCOL,
	.iInterface         = 0,
};

const struct usb_qualifier_descriptor usb_qual_desc = {
	.bLength            = sizeof(struct usb_qualifier_descriptor),
	.bDescriptorType    = USB_DT_QUALIFIER,
	.bcdUSB             = 0x0200,
	.bDeviceClass       = USB_DEV_CLASS,
	.bDeviceSubClass    = USB_DEV_SUBCLASS,
	.bDeviceProtocol    = USB_DEV_PROTOCOL,
	.bMaxPacketSize0    = USB_EP0_SIZE,
	.bNumConfigurations = 1,
	.bReserved          = 0,
};

static const uint8_t lang_desc[] = USB_LANG_DESCRIPTOR_DATA;
static const uint8_t manu_desc[] = USB_MANUFACTURER_DESCRIPTOR_DATA;
static const uint8_t prod_desc[] = USB_PRODUCT_DESCRIPTOR_DATA;
static const uint8_t sn_desc[]   = USB_SERIAL_DESCRIPTOR_DATA;

const uint8_t *usb_string_descs[] = {
	[USB_STR_LANG]         = lang_desc,
	[USB_STR_MANUFACTURER] = manu_desc,
	[USB_STR_PRODUCT]      = prod_desc,
	[USB_STR_SERIAL]       = sn_desc,
};

const uint8_t usb_string_descs_count = sizeof(usb_string_descs) / sizeof(usb_string_descs[0]);

uint16_t usb_build_config_desc(uint8_t *buf, uint16_t ep_size)
{
	uint8_t *p = buf;

	struct usb_configuration_descriptor cfg = {
		.bLength             = sizeof(struct usb_configuration_descriptor),
		.bDescriptorType     = USB_DT_CONFIG,
		.wTotalLength        = USB_CONFIG_DESCRIPTOR_SIZE,
		.bNumInterfaces      = USB_NUM_INTERFACES,
		.bConfigurationValue = 1,
		.iConfiguration      = 0,
		.bmAttributes        = 0xC0,
		.bMaxPower           = USB_MAX_POWER_MA / 2,
	};
	memcpy(p, &cfg, sizeof(cfg));
	p += sizeof(cfg);

	memcpy(p, &usb_iface_desc, sizeof(usb_iface_desc));
	p += sizeof(usb_iface_desc);

	struct usb_endpoint_descriptor ep = {
		.bLength         = sizeof(struct usb_endpoint_descriptor),
		.bDescriptorType = USB_DT_ENDPOINT,
		.bmAttributes    = 0x02,
		.bInterval       = 0,
	};

	static const uint8_t ep_addrs[] = {
		USB_EP_CMD_OUT, USB_EP_CMD_IN,
		USB_EP_DATA_OUT, USB_EP_DATA_IN,
	};

	for (int i = 0; i < USB_NUM_ENDPOINTS; i++) {
		ep.bEndpointAddress = ep_addrs[i];
		ep.wMaxPacketSize   = ep_size;
		memcpy(p, &ep, sizeof(ep));
		p += sizeof(ep);
	}

	return (uint16_t)(p - buf);
}
