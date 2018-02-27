/*!<Includes */
#include "targetdev.h"

/*!<USB HID Report Descriptor */
const uint8_t g_u8HIDDeviceReportDescriptor[] = {
    0x05, 0x01, // USAGE_PAGE (Generic Desktop)
    0x09, 0x00, // USAGE (0)
    0xA1, 0x01, // COLLECTION (Application)
    0x15, 0x00, //     LOGICAL_MINIMUM (0)
    0x25, 0xFF, //     LOGICAL_MAXIMUM (255)
    0x19, 0x00, // USAGE_MINIMUM (0x00)
    0x29, 0xFF, // USAGE_MAXIMUM (0xFF)
    0x95, 0x40, //     REPORT_COUNT (8)
    0x75, 0x08, //     REPORT_SIZE (8)
    0x81, 0x02, //     INPUT (Data,Var,Abs)
    0x19, 0x00, // USAGE_MINIMUM (0x00)
    0x29, 0xFF, // USAGE_MAXIMUM (0xFF)
    0x91, 0x02, //   OUTPUT (Data,Var,Abs)
    0xC0        // END_COLLECTION
};


/*----------------------------------------------------------------------------*/
/*!<USB Device Descriptor */
const uint8_t g_u8HIDDeviceDescriptor[] = {
    LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
    0x10, 0x01,     /* bcdUSB */
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    EP0_MAX_PKT_SIZE,   /* bMaxPacketSize0 */
    /* idVendor */
    USBD_VID & 0x00FF,
    (USBD_VID & 0xFF00) >> 8,
                        /* idProduct */
                        USBD_PID & 0x00FF,
                        (USBD_PID & 0xFF00) >> 8,
                        0x00, 0x00,     /* bcdDevice */
                        0x01,           /* iManufacture */
                        0x02,           /* iProduct */
                        0x00,           /* iSerialNumber - no serial */
                        0x01            /* bNumConfigurations */
};

/*----------------------------------------------------------------------------*/
/*!<USB Configure Descriptor */
const uint8_t g_u8HIDConfigDescriptor[] = {
    LEN_CONFIG,     /* bLength */
    DESC_CONFIG,    /* bDescriptorType */
    /* wTotalLength */
    (LEN_CONFIG + LEN_INTERFACE + LEN_HID + LEN_ENDPOINT * 2) & 0x00FF,
    ((LEN_CONFIG + LEN_INTERFACE + LEN_HID + LEN_ENDPOINT * 2) & 0xFF00) >> 8,
            0x01,           /* bNumInterfaces */
            0x01,           /* bConfigurationValue */
            0x00,           /* iConfiguration */
            0x80 | (USBD_SELF_POWERED << 6) | (USBD_REMOTE_WAKEUP << 5),/* bmAttributes */
            USBD_MAX_POWER,         /* MaxPower */

            /* I/F descr: HID */
            LEN_INTERFACE,  /* bLength */
            DESC_INTERFACE, /* bDescriptorType */
            0x00,           /* bInterfaceNumber */
            0x00,           /* bAlternateSetting */
            0x02,           /* bNumEndpoints */
            0x03,           /* bInterfaceClass */
            0x00,           /* bInterfaceSubClass */
            0x00,           /* bInterfaceProtocol */
            0x00,           /* iInterface */

            /* HID Descriptor */
            LEN_HID,        /* Size of this descriptor in UINT8s. */
            DESC_HID,       /* HID descriptor type. */
            0x10, 0x01,     /* HID Class Spec. release number. */
            0x00,           /* H/W target country. */
            0x01,           /* Number of HID class descriptors to follow. */
            DESC_HID_RPT,   /* Descriptor type. */
            /* Total length of report descriptor. */
            sizeof(g_u8HIDDeviceReportDescriptor) & 0x00FF,
            (sizeof(g_u8HIDDeviceReportDescriptor) & 0xFF00) >> 8,

            /* EP Descriptor: interrupt in. */
            LEN_ENDPOINT,   /* bLength */
            DESC_ENDPOINT,  /* bDescriptorType */
            (INT_IN_EP_NUM | EP_INPUT), /* bEndpointAddress */
            EP_INT,         /* bmAttributes */
            /* wMaxPacketSize */
            EP2_MAX_PKT_SIZE & 0x00FF,
            (EP2_MAX_PKT_SIZE & 0xFF00) >> 8,
            HID_DEFAULT_INT_IN_INTERVAL,        /* bInterval */

            /* EP Descriptor: interrupt out. */
            LEN_ENDPOINT,   /* bLength */
            DESC_ENDPOINT,  /* bDescriptorType */
            (INT_OUT_EP_NUM | EP_OUTPUT),   /* bEndpointAddress */
            EP_INT,         /* bmAttributes */
            /* wMaxPacketSize */
            EP3_MAX_PKT_SIZE & 0x00FF,
            (EP3_MAX_PKT_SIZE & 0xFF00) >> 8,
            HID_DEFAULT_INT_IN_INTERVAL     /* bInterval */
};

/*----------------------------------------------------------------------------*/
/*!<USB Language String Descriptor */
const uint8_t g_u8HIDStringLang[4] = {
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*----------------------------------------------------------------------------*/
/*!<USB Vendor String Descriptor */
const uint8_t g_u8HIDVendorStringDesc[] = {
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*----------------------------------------------------------------------------*/
/*!<USB Product String Descriptor */
const uint8_t g_u8HIDProductStringDesc[] = {
    16,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'W', 0, 'P', 0, 'M', 0, ' ', 0, 'U', 0, 'S', 0, 'B', 0
};

/*----------------------------------------------------------------------------*/
#if 0
const uint8_t g_u8HIDStringSerial[26] = {
    22,             // bLength
    DESC_STRING,    // bDescriptorType
    'I', 0, '9', 0, '4', 0, '1', 0, '0', 0, '0', 0, ' ', 0, 'H', 0, 'I', 0, 'D', 0,
};
#endif

const uint8_t *g_pu8HIDUsbString[4] = {
    g_u8HIDStringLang,
    g_u8HIDVendorStringDesc,
    g_u8HIDProductStringDesc,
    NULL
};

const uint8_t *g_pu8HIDUsbHidReport[2] = {
    g_u8HIDDeviceReportDescriptor,
    NULL
};

const uint32_t g_u32HIDUsbHidReportLen[2] = {
    sizeof(g_u8HIDDeviceReportDescriptor),
    0
};

const uint32_t g_u32HIDConfigHidDescIdx[2] = {
    (LEN_CONFIG + LEN_INTERFACE),
    0
};

#if 0
/*!<USB BOS Descriptor */
const uint8_t g_u8HIDBOSDescriptor[] = {
    LEN_BOS,        /* bLength */
    DESC_BOS,       /* bDescriptorType */
    /* wTotalLength */
    0x0C & 0x00FF,
    (0x0C & 0xFF00) >> 8,
                    0x01,           /* bNumDeviceCaps */

                    /* Device Capability */
                    0x7,            /* bLength */
                    DESC_CAPABILITY,/* bDescriptorType */
                    CAP_USB20_EXT,  /* bDevCapabilityType */
                    0x02, 0x00, 0x00, 0x00  /* bmAttributes */
};
#endif

const S_USBD_INFO_T gsInfo = {
    g_u8HIDDeviceDescriptor,
    g_u8HIDConfigDescriptor,
    g_pu8HIDUsbString,
    g_pu8HIDUsbHidReport,
    NULL, //(uint8_t *)g_u8HIDBOSDescriptor,
    g_u32HIDUsbHidReportLen,
    g_u32HIDConfigHidDescIdx
};

