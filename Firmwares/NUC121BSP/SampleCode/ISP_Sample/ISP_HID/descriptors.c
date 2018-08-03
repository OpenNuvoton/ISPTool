/******************************************************************************//**
 * @file     descriptors.c
 * @version  V3.00
 * @brief    NUC121 series USBD descriptor
 *
 * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
/*!<Includes */
#include "NUC121.h"
#include "hid_transfer.h"

/*!<USB HID Report Descriptor */
const uint8_t HID_DeviceReportDescriptor[] = {
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
const uint8_t gu8DeviceDescriptor[] = {
    LEN_DEVICE,     /* bLength */
    DESC_DEVICE,    /* bDescriptorType */
#ifdef SUPPORT_LPM
    0x01, 0x02,     /* bcdUSB => 0x0201 to support LPM */
#else
    0x10, 0x01,     /* bcdUSB */
#endif
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
                        0x03,           /* iSerialNumber */
                        0x01            /* bNumConfigurations */
};

/*!<USB Configure Descriptor */
const uint8_t gu8ConfigDescriptor[] = {
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
            sizeof(HID_DeviceReportDescriptor) & 0x00FF,
            (sizeof(HID_DeviceReportDescriptor) & 0xFF00) >> 8,

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

#ifdef SUPPORT_LPM
const uint8_t gu8BosDescriptor[] = {
    LEN_BOS,                         /* bLength */
    DESC_BOS,                        /* bDescriptorType */
    ((LEN_BOS + LEN_DEVCAP) & 0xFF), /* wTotalLength */
    ((LEN_BOS + LEN_DEVCAP) >> 8),   /* wTotalLength */
    0x01,                            /* bNumDevcieCaps */
    LEN_DEVCAP,                      /* bLength */
    DESC_DEVCAP,                     /* bDescriptorType */
    0x02,                            /* bDevCapabilityType, 0x02 is USB 2.0 Extension */
    0x06, 0x04, 0x00, 0x00  /* bmAttributs, 32 bits                                              */
    /* bit 0 : Reserved. Must 0.                                         */
    /* bit 1 : 1 to support LPM.                                         */
    /* bit 2 : 1 to support BSL & Alternat HIRD                          */
    /* bit 3 : 1 to recommend Baseline BESL                              */
    /* bit 4 : 1 to recommand Deep BESL                                  */
    /* bit 11:8 : Recommend Baseline BESL value. Ignore by bit3 is zero. */
    /* bit 15:12 : Recommend Deep BESL value. Ignore by bit4 is zero.    */
    /* bit 31:16 : Reserved. Must 0.                                     */
};
#endif


/*!<USB Language String Descriptor */
const uint8_t gu8StringLang[4] = {
    4,              /* bLength */
    DESC_STRING,    /* bDescriptorType */
    0x09, 0x04
};

/*!<USB Vendor String Descriptor */
const uint8_t gu8VendorStringDesc[] = {
    16,
    DESC_STRING,
    'N', 0, 'u', 0, 'v', 0, 'o', 0, 't', 0, 'o', 0, 'n', 0
};

/*!<USB Product String Descriptor */
const uint8_t gu8ProductStringDesc[] = {
    16,             /* bLength          */
    DESC_STRING,    /* bDescriptorType  */
    'W', 0, 'P', 0, 'M', 0, ' ', 0, 'U', 0, 'S', 0, 'B', 0

};



const uint8_t gu8StringSerial[26] = {
    26,             // bLength
    DESC_STRING,    // bDescriptorType
    'A', 0, '0', 0, '2', 0, '0', 0, '1', 0, '4', 0, '0', 0, '9', 0, '0', 0, '3', 0, '0', 0, '4', 0
};

const uint8_t *gpu8UsbString[4] = {
    gu8StringLang,
    gu8VendorStringDesc,
    gu8ProductStringDesc,
    gu8StringSerial
};

const uint8_t *gpu8UsbHidReport[2] = {
    HID_DeviceReportDescriptor,
    NULL
};

const uint32_t gu32UsbHidReportLen[2] = {
    sizeof(HID_DeviceReportDescriptor),
    0
};

const uint32_t gu32ConfigHidDescIdx[2] = {
    (LEN_CONFIG + LEN_INTERFACE),
    0
};

const S_USBD_INFO_T gsInfo = {
    gu8DeviceDescriptor,
    gu8ConfigDescriptor,
    gpu8UsbString,
    gpu8UsbHidReport,
    gu32UsbHidReportLen,
    gu32ConfigHidDescIdx,
#ifdef SUPPORT_LPM
    gu8BosDescriptor
#endif

};

