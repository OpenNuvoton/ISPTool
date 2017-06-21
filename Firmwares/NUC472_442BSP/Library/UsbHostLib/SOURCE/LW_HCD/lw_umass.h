#ifndef _LW_UMASS_H_
#define _LW_UMASS_H_

#include "diskio.h"     /* FatFs lower layer API */

/** @addtogroup NUC472_442_Device_Driver NUC472/NUC442 Device Driver
  @{
*/

/** @addtogroup NUC472_442_LWHCD_Driver LW_USBH Driver
  @{
*/

/// @cond HIDDEN_SYMBOLS

#define USB_CLASS_MASS_STORAGE   8

/* Sub Classes */
#define UMAS_SC_RBC              0x01    /* Typically, flash devices */
#define UMAS_SC_8020             0x02    /* CD-ROM */
#define UMAS_SC_QIC              0x03    /* QIC-157 Tapes */
#define UMAS_SC_UFI              0x04    /* Floppy */
#define UMAS_SC_8070             0x05    /* Removable media */
#define UMAS_SC_SCSI             0x06    /* Transparent */

/* Protocols */
#define UMAS_PR_CBI              0x00    /* Control/Bulk/Interrupt */
#define UMAS_PR_CB               0x01    /* Control/Bulk w/o interrupt */
#define UMAS_PR_BULK             0x50    /* bulk only */
#define UMAS_PR_DPCM_USB         0xf0    /* Combination CB/SDDR09 */


/* command block wrapper */
struct bulk_cb_wrap {
    uint32_t  Signature;                 /* contains 'USBC' */
    uint32_t  Tag;                       /* unique per command id */
    uint32_t  DataTransferLength;        /* size of data */
    uint8_t   Flags;                     /* direction in bit 0 */
    uint8_t   Lun;                       /* LUN normally 0 */
    uint8_t   Length;                    /* of of the CDB */
    uint8_t   CDB[16];                   /* max command */
};

#define UMAS_BULK_CB_WRAP_LEN    31
#define UMAS_BULK_CB_SIGN        0x43425355   /* spells out USBC */
#define UMAS_BULK_FLAG_IN        1
#define UMAS_BULK_FLAG_OUT       0

/* command status wrapper */
struct bulk_cs_wrap {
    uint32_t  Signature;                 /* should = 'USBS' */
    uint32_t  Tag;                       /* same as original command */
    uint32_t  Residue;                   /* amount not transferred */
    uint8_t   Status;                    /* see below */
};

#define UMAS_BULK_CS_WRAP_LEN     13
#define UMAS_BULK_CS_SIGN         0x53425355      /* spells out 'USBS' */
#define UMAS_BULK_STAT_OK         0      /* command passed */
#define UMAS_BULK_STAT_FAIL       1      /* command failed */
#define UMAS_BULK_STAT_PHASE      2      /* phase error */

/* bulk-only class specific requests */
#define UMAS_BULK_RESET_REQUEST   0xff
#define UMAS_BULK_GET_MAX_LUN     0xfe


/*
 *      SCSI opcodes
 */
#define TEST_UNIT_READY         0x00
#define REQUEST_SENSE           0x03
#define INQUIRY                 0x12
#define MODE_SENSE              0x1a
#define READ_CAPACITY           0x25
#define READ_10                 0x28
#define WRITE_10                0x2a
#define MODE_SENSE_10           0x5a

DRESULT  usbh_umas_read(uint8_t *buff, uint32_t sector_no, int number_of_sector);
DRESULT  usbh_umas_write(uint8_t *buff, uint32_t sector_no, int number_of_sector);
DRESULT  usbh_umas_ioctl(int cmd, void *buff);
int  usbh_umas_disk_status(void);

/// @endcond

/** @addtogroup NUC472_442_LWHCD_EXPORTED_FUNCTIONS LW_USBH Exported Functions
  @{
*/

int usbh_probe_umass(void);

/*@}*/ /* end of group NUC472_442_LWHCD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC472_442_LWHCD_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */

#endif  /* _LW_UMASS_H_ */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


