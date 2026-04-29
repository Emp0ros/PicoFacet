// SPDX-License-Identifier: GPL-2.0-only
// usb_descriptors.h - Shared USB descriptor definitions for PicoFacet

#ifndef USB_DESCRIPTORS_H_
#define USB_DESCRIPTORS_H_

#include <stdint.h>

// Device identity
#define USB_VID             0x600D
#define USB_PID             0x7002
#define USB_BCD_DEVICE      0x0200

// String descriptor indices
#define USB_STR_LANG         0
#define USB_STR_MANUFACTURER 1
#define USB_STR_PRODUCT      2
#define USB_STR_SERIAL       3

// Device class
#define USB_DEV_CLASS       0xFF
#define USB_DEV_SUBCLASS    0x00
#define USB_DEV_PROTOCOL    0x00

// Configuration
#define USB_MAX_POWER_MA    100
#define USB_NUM_INTERFACES  1
#define USB_NUM_ENDPOINTS   4

// EP0
#define USB_EP0_SIZE        64

// Endpoint addresses (all unidirectional)
#define USB_EP_CMD_OUT      0x01    // EP1 OUT - commands from host
#define USB_EP_CMD_IN       0x82    // EP2 IN  - responses to host
#define USB_EP_DATA_OUT     0x03    // EP3 OUT - block write data from host
#define USB_EP_DATA_IN      0x84    // EP4 IN  - block read data to host

// Packet sizes
#define USB_HS_BULK_SIZE    512
#define USB_FS_BULK_SIZE    64

// Standard descriptor types
#define USB_DT_DEVICE       0x01
#define USB_DT_CONFIG       0x02
#define USB_DT_STRING       0x03
#define USB_DT_INTERFACE    0x04
#define USB_DT_ENDPOINT     0x05
#define USB_DT_QUALIFIER    0x06

// ======================================================================
// Standard USB descriptor structs
// ======================================================================

struct usb_device_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} __attribute__((packed));

struct usb_configuration_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
} __attribute__((packed));

struct usb_interface_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} __attribute__((packed));

struct usb_endpoint_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
} __attribute__((packed));

struct usb_qualifier_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint8_t  bNumConfigurations;
    uint8_t  bReserved;
} __attribute__((packed));

struct usb_setup_packet {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __attribute__((packed));

// Standard request codes
#define USB_REQ_GET_DESCRIPTOR    0x06
#define USB_REQ_SET_ADDRESS       0x05
#define USB_REQ_SET_CONFIGURATION 0x09

// ======================================================================
// Shared descriptor instances (defined in usb_descriptors.c)
// ======================================================================

extern const struct usb_device_descriptor    usb_dev_desc;
extern const struct usb_interface_descriptor usb_iface_desc;
extern const struct usb_qualifier_descriptor usb_qual_desc;

extern const uint8_t *usb_string_descs[];
extern const uint8_t  usb_string_descs_count;

#define USB_CONFIG_DESCRIPTOR_SIZE \
    (sizeof(struct usb_configuration_descriptor) + \
     sizeof(struct usb_interface_descriptor) + \
     sizeof(struct usb_endpoint_descriptor) * USB_NUM_ENDPOINTS)

uint16_t usb_build_config_desc(uint8_t *buf, uint16_t ep_size);

// ======================================================================
// Pre-encoded UTF-16LE string descriptors
// ======================================================================

#define USB_LANG_DESCRIPTOR_DATA    { 4, USB_DT_STRING, 0x09, 0x04 }

#define USB_MANUFACTURER_DESCRIPTOR_DATA { \
    20, USB_DT_STRING, \
    'P', 0, 'i', 0, 'c', 0, 'o', 0, 'F', 0, 'a', 0, 'c', 0, 'e', 0, 't', 0, \
}

#define USB_PRODUCT_DESCRIPTOR_DATA { \
    34, USB_DT_STRING, \
    'P', 0, 'i', 0, 'c', 0, 'o', 0, 'F', 0, 'a', 0, 'c', 0, 'e', 0, 't', 0, \
    ' ', 0, 'D', 0, 'e', 0, 'v', 0, 'i', 0, 'c', 0, 'e', 0, \
}

#define USB_SERIAL_DESCRIPTOR_DATA { \
    22, USB_DT_STRING, \
    '0', 0, '1', 0, '2', 0, '3', 0, '4', 0, \
    '5', 0, '6', 0, '7', 0, '8', 0, '9', 0, \
}

#endif
