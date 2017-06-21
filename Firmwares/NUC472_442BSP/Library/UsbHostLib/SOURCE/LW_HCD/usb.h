
#ifndef _USBH_H_
#define _USBH_H_

/// @cond HIDDEN_SYMBOLS

enum OCHI_CC_CODE {
/* mapping of the OHCI CC status to error codes */
/* No  Error  */               CC_NOERROR,
/* CRC Error  */               CC_CRC,
/* Bit Stuff  */               CC_BITSTUFF,
/* Data Toggle*/               CC_DATA_TOGGLE,
/* Stall      */               CC_STALL,
/* DevNotResp */               CC_NOTRESPONSE,
/* PIDCheck   */               CC_PID_CHECK,
/* UnExpPID   */               CC_UNEXPECTED_PID,
/* DataOver   */               CC_DATA_OVERRUN,
/* DataUnder  */               CC_DATA_UNDERRUN,
/* reserved   */               CC_RESERVED1,
/* reserved   */               CC_RESERVED2,
/* BufferOver */               CC_BUFFER_OVERRUN,
/* BuffUnder  */               CC_BUFFER_UNDERRUN,
/* Not Access */               CC_NOT_ACCESS
};


/* ED States */
#define ED_NEW          0x00
#define ED_UNLINK       0x01
#define ED_OPER         0x02
#define ED_DEL          0x04
#define ED_URB_DEL      0x08

/* usb_ohci_ed */
typedef struct ed {
    uint32_t    hwINFO;
    uint32_t    hwTailP;
    uint32_t    hwHeadP;
    uint32_t    hwNextED;
} ED_T;


/* TD info field */
#define TD_CC       0xF0000000
#define TD_CC_GET(td_p) ((td_p >>28) & 0x0F)
#define TD_CC_SET(td_p, cc) (td_p) = ((td_p) & 0x0FFFFFFF) | (((cc) & 0x0F) << 28)
#define TD_EC              0x0C000000
#define TD_T_DATA0         0x02000000
#define TD_T_DATA1         0x03000000
#define TD_T_TOGGLE        0x00000000
#define TD_R               0x00040000
#define TD_DI              0x00E00000
#define TD_DI_SET(X)       (((X) & 0x07)<< 21)
#define TD_DP              0x00180000
#define TD_DP_SETUP        0x00000000
#define TD_DP_IN           0x00100000
#define TD_DP_OUT          0x00080000

#define TD_ISO             0x00010000
#define TD_DEL             0x00020000

#define MAXPSW             8

typedef struct td {
    uint32_t    hwINFO;
    uint32_t    hwCBP;            /* Current Buffer Pointer */
    uint32_t    hwNextTD;         /* Next TD Pointer */
    uint32_t    hwBE;             /* Memory Buffer End Pointer */
} TD_T;


/*
 * The HCCA (Host Controller Communications Area) is a 256 byte
 * structure defined in the OHCI spec. that the host controller is
 * told the base address of.  It must be 256-byte aligned.
 */

#define NUM_INTS                32     /* part of the OHCI standard */

typedef struct ohci_hcca {
    uint32_t   int_table[NUM_INTS];    /* Interrupt ED table */
#ifdef __BIG_ENDIAN
    uint16_t   pad1;                   /* set to 0 on each frame_no change */
    uint16_t   frame_no;               /* current frame number */
#else
    uint16_t   frame_no;               /* current frame number */
    uint16_t   pad1;                   /* set to 0 on each frame_no change */
#endif
    uint32_t   done_head;              /* info returned for an interrupt */
    uint8_t    reserved_for_hc[116];
} OHCI_HCCA_T;


/* OHCI CONTROL AND STATUS REGISTER MASKS */


/* pre-shifted values for HCFS */
#define OHCI_USB_RESET   (0 << 6)
#define OHCI_USB_RESUME  (1 << 6)
#define OHCI_USB_OPER    (2 << 6)
#define OHCI_USB_SUSPEND (3 << 6)

/*
 * HcCommandStatus (cmdstatus) register masks
 */
#define OHCI_HCR        (1 << 0)        /* host controller reset */
#define OHCI_CLF        (1 << 1)        /* control list filled */
#define OHCI_BLF        (1 << 2)        /* bulk list filled */
#define OHCI_OCR        (1 << 3)        /* ownership change request */
#define OHCI_SOC        (3 << 16)       /* scheduling overrun count */

#define OHCI_INTR_MIE   0x80000000      /* master interrupt enable */

/*
 * USB directions
 */
#define USB_DIR_OUT                     0
#define USB_DIR_IN                      0x80

/*
 * Descriptor types
 */
#define USB_DT_DEVICE                   0x01
#define USB_DT_CONFIG                   0x02
#define USB_DT_STRING                   0x03
#define USB_DT_INTERFACE                0x04
#define USB_DT_ENDPOINT                 0x05

/*
 * Standard requests
 */
#define USB_REQ_GET_STATUS              0x00
#define USB_REQ_CLEAR_FEATURE           0x01
#define USB_REQ_SET_FEATURE             0x03
#define USB_REQ_SET_ADDRESS             0x05
#define USB_REQ_GET_DESCRIPTOR          0x06
#define USB_REQ_SET_CONFIGURATION       0x09
#define USB_REQ_SET_INTERFACE           0x0B

typedef struct {
    __packed uint8_t  requesttype;
    __packed uint8_t  request;
    __packed uint16_t value;
    __packed uint16_t index;
    __packed uint16_t length;
} DEV_REQ_T;


/*-----------------------------------------------------------------------------------
 *  USB device descriptor
 */
typedef struct usb_device_descriptor {  /*!< device descriptor structure            */
    __packed uint8_t  bLength;          /*!< Length of device descriptor            */
    __packed uint8_t  bDescriptorType;  /*!< Device descriptor type                 */
    __packed uint16_t bcdUSB;           /*!< USB version number                     */
    __packed uint8_t  bDeviceClass;     /*!< Device class code                      */
    __packed uint8_t  bDeviceSubClass;  /*!< Device subclass code                   */
    __packed uint8_t  bDeviceProtocol;  /*!< Device protocol code                   */
    __packed uint8_t  bMaxPacketSize0;  /*!< Maximum packet size of control endpoint*/
    __packed uint16_t idVendor;         /*!< Vendor ID                              */
    __packed uint16_t idProduct;        /*!< Product ID                             */
    __packed uint16_t bcdDevice;        /*!< Device ID                              */
    __packed uint8_t  iManufacturer;    /*!< Manufacture description string ID      */
    __packed uint8_t  iProduct;         /*!< Product description string ID          */
    __packed uint8_t  iSerialNumber;    /*!< Serial number description string ID    */
    __packed uint8_t  bNumConfigurations; /*!< Total number of configurations       */
} USB_DEV_DESC_T;                       /*!< device descriptor structure            */


/*-----------------------------------------------------------------------------------
 *  Configuration descriptor
 */
typedef struct usb_config_descriptor {  /*!< Configuration descriptor structure     */
    __packed uint8_t   bLength;         /*!< Length of configuration descriptor     */
    __packed uint8_t   bDescriptorType; /*!< Descriptor type                        */
    __packed uint16_t  wTotalLength;    /*!< Total length of this configuration     */
    __packed uint8_t   bNumInterfaces;  /*!< Total number of interfaces             */
    __packed uint8_t   bConfigurationValue; /*!< Configuration descriptor number    */
    __packed uint8_t   iConfiguration;  /*!< String descriptor ID                   */
    __packed uint8_t   bmAttributes;    /*!< Configuration characteristics          */
    __packed uint8_t   MaxPower;        /*!< Maximum power consumption              */
} USB_CONFIG_DESC_T;                    /*!< Configuration descriptor structure     */


/*-----------------------------------------------------------------------------------
 *  USB interface descriptor
 */
typedef struct usb_interface_descriptor { /*!< Interface descriptor structure         */
    __packed uint8_t  bLength;          /*!< Length of interface descriptor         */
    __packed uint8_t  bDescriptorType;  /*!< Descriptor type                        */
    __packed uint8_t  bInterfaceNumber; /*!< Interface number                       */
    __packed uint8_t  bAlternateSetting;/*!< Alternate setting number               */
    __packed uint8_t  bNumEndpoints;    /*!< Number of endpoints                    */
    __packed uint8_t  bInterfaceClass;  /*!< Interface class code                   */
    __packed uint8_t  bInterfaceSubClass; /*!< Interface subclass code              */
    __packed uint8_t  bInterfaceProtocol; /*!< Interface protocol code              */
    __packed uint8_t  iInterface;       /*!< Interface ID                           */
} USB_IF_DESC_T;                        /*!< Interface descriptor structure         */


/*-----------------------------------------------------------------------------------
 *  USB endpoint descriptor
 */
typedef struct usb_endpoint_descriptor { /*!< Endpoint descriptor structure          */
    __packed uint8_t  bLength;          /*!< Length of endpoint descriptor          */
    __packed uint8_t  bDescriptorType;  /*!< Descriptor type                        */
    __packed uint8_t  bEndpointAddress; /*!< Endpoint address                       */
    __packed uint8_t  bmAttributes;     /*!< Endpoint attribute                     */
    __packed uint16_t wMaxPacketSize;   /*!< Maximum packet size                    */
    __packed uint8_t  bInterval;        /*!< Synchronous transfer interval          */
    __packed uint8_t  bRefresh;         /*!< Refresh                                */
    __packed uint8_t  bSynchAddress;    /*!< Sync address                           */
} USB_EP_DESC_T;                        /*!< Endpoint descriptor structure          */


#define MINISEC_1           84000
#define MINISEC_10          840000
#define MINISEC_100         8400000

/// @endcond

#endif  /* _USBH_H_ */
