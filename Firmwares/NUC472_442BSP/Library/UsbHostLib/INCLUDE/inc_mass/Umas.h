
#ifndef _UMAS_H_
#define _UMAS_H_

#include "UmasScsi.h"


/// @cond HIDDEN_SYMBOLS

#define UMAS_MAX_DEV    1

#undef DEBUG
//#define DEBUG
//#define UMAS_VERBOSE_DEBUG

/* Debug Options */
#ifdef UMAS_VERBOSE_DEBUG
#define UMAS_DEBUG    printf
#define UMAS_VDEBUG   printf
#else
#ifdef DEBUG
#define UMAS_DEBUG    printf
#else
#define UMAS_DEBUG(...)
#endif
#define UMAS_VDEBUG(...)
#endif

struct umas_data;

/*
 * Unusual device list definitions
 */

typedef struct umas_unusual_dev {
    uint8_t   useProtocol;
    uint8_t   useTransport;
} UMAS_UUDEV_T;


/* Flag definitions */
#define UMAS_FL_SINGLE_LUN      0x00000001  /* allow access to only LUN 0      */
#define UMAS_FL_MODE_XLATE      0x00000002  /* translate _6 to _10 commands for
                                                Win/MacOS compatibility */
#define UMAS_FL_START_STOP      0x00000004  /* ignore START_STOP commands      */
#define UMAS_FL_IGNORE_SER      0x00000010  /* Ignore the serial number given  */
#define UMAS_FL_SCM_MULT_TARG   0x00000020  /* supports multiple targets */

typedef int  (*trans_cmnd)(SCSI_CMD_T *, struct umas_data *);
typedef int  (*trans_reset)(struct umas_data *);
typedef void (*proto_cmnd)(SCSI_CMD_T*, struct umas_data *);


struct umas_drive;

/* we allocate one of these for every device that we remember */
typedef struct umas_data {             /* LINUX: struct us_data */
    USB_DEV_T     *pusb_dev;           /* this usb_device */
    uint16_t      vendor_id;
    uint16_t      product_id;
    uint32_t      sector_size;
    uint32_t      sector_number;

    /* information about the device -- only good if device is attached */
    uint8_t       ifnum;               /* interface number   */
    uint8_t       ep_in;               /* bulk in endpoint   */
    uint8_t       ep_out;              /* bulk out endpoint  */
    int           ep_int;              /* interrupt endpoint */

    struct umas_data  *next;           /* next device */
    uint32_t      flags;               /* from filter initially */

    /* information about the device -- always good */
    char          *transport_name;
    char          *protocol_name;
    uint8_t       subclass;
    uint8_t       protocol;
    uint8_t       max_lun;

    /* function pointers for this device */
    trans_cmnd    transport;           /* transport function     */
    trans_reset   transport_reset;     /* transport device reset */
    proto_cmnd    proto_handler;       /* protocol handler       */

    /* SCSI interfaces */
    SCSI_CMD_T    srb;                /* current srb         */

    int           ip_wanted;           /* is an IRQ expected? (atomic_t) */
    URB_T         *current_urb;        /* non-int USB requests */
    URB_T         *irq_urb;            /* for USB int requests */
    uint8_t       irqbuf[4];           /* buffer for USB IRQ   */
    uint8_t       irqdata[2];          /* data from USB IRQ    */

    UMAS_UUDEV_T  *unusual_dev;        /* If unusual device       */
    struct umas_drive  *drive_list;
} UMAS_DATA_T;


typedef struct umas_drive {
    UMAS_DATA_T     *umas;
    uint8_t         lun_no;
    void            *client;           /* file system client data */
    struct umas_drive  *next;          /* next device */
} UMAS_DRIVE_T;



/*======================================================= Global Variables ==*/

/* The structure which defines our driver */
extern USB_DRIVER_T  _UsbMassStorageDriver;   /* LINUX: usb_storage_driver */


/*===================================================== Exported Functions ==*/
extern int  UMAS_InitUmasDriver(void);

extern int  UMAS_InitUmasDevice(UMAS_DATA_T *umas);

extern void UMAS_ScanAllDevice(void);
extern void UMAS_ScanDeviceLun(UMAS_DATA_T *umas);

/*
 * Debug helper routines, in UsbDebug.c
 */
extern void UMAS_DEBUG_ShowCommand(SCSI_CMD_T   *srb);
extern void UMAS_DEBUG_PrintScsiCommand(SCSI_CMD_T *cmd);

#include "UmasProtocol.h"
#include "UmasTransport.h"

/// @endcond HIDDEN_SYMBOLS

#endif  /* _UMAS_H_ */
