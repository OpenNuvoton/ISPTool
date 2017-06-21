/**************************************************************************//**
 * @file     Umas_FATFS.c
 * @version  V1.00
 * $Revision: 8 $
 * $Date: 15/12/03 10:22a $
 * @brief    NUC472/NUC442 MCU USB Host Mass Storage driver for FATFS
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NUC472_442.h"
#include "usbh_core.h"

#include "Umas.h"
#include "diskio.h"
#include "usbh_umas.h"

/// @cond HIDDEN_SYMBOLS

UMAS_DATA_T         *g_umas = NULL;
static int          g_disk_lun;
static uint32_t     g_sector_size, g_total_sector_num;



static uint8_t usb_stor_sense_notready[18] = {
    0x70/* current error */, 0, 0x02/* not ready */, 0 , 0,
    0x0a/* additional length */, 0, 0, 0, 0,
    0x04/* not ready */, 0x03 /* manual intervention */
};


/* run command */
static int  run_scsi_command(SCSI_CMD_T *srb, UMAS_DATA_T *umas)
{
    /* reject the command if the direction indicator is UNKNOWN */
    if (srb->sc_data_direction == SCSI_DATA_UNKNOWN) {
        UMAS_DEBUG("run_scsi_command - UNKNOWN data direction\n");
        umas->srb.result = DID_ERROR << 16;
        return -1;
    }

#if 0
    /* reject if target != 0 or if LUN is higher than the maximum known LUN */
    if (umas->srb.target && (!(umas->flags & UMAS_FL_SCM_MULT_TARG))) {
        UMAS_DEBUG("run_scsi_command - Bad target number (%d/%d)\n", umas->srb.target, umas->srb.lun);
        umas->srb.result = DID_BAD_TARGET << 16;
        return -1;
    }
#endif

    if (umas->srb.lun > umas->max_lun) {
        UMAS_DEBUG("run_scsi_command - Bad LUN (%d/%d)\n", srb->target, srb->lun);
        umas->srb.result = DID_BAD_TARGET << 16;
        return -1;
    }

    /* handle those devices which can't do a START_STOP */
    if ((srb->cmnd[0] == START_STOP) && (umas->flags & UMAS_FL_START_STOP)) {
        UMAS_DEBUG("run_scsi_command - Skipping START_STOP command\n");
        umas->srb.result = GOOD << 1;
        return -1;
    }

    /* our device has gone - pretend not ready */
    if (!umas->pusb_dev) {
        UMAS_DEBUG("run_scsi_command - Request is for removed device\n");
        /*
         * For REQUEST_SENSE, it's the data.  But for anything else,
         * it should look like we auto-sensed for it.
         */
        if (umas->srb.cmnd[0] == REQUEST_SENSE) {
            memcpy(srb->request_buff,
                   usb_stor_sense_notready, sizeof(usb_stor_sense_notready));
            srb->result = GOOD << 1;
        } else {
            memcpy(srb->sense_buffer,
                   usb_stor_sense_notready, sizeof(usb_stor_sense_notready));
            srb->result = CHECK_CONDITION << 1;
        }
    } else { /* !umas->pusb_dev */
#ifdef UMAS_VERBOSE_DEBUG
        UMAS_DEBUG_ShowCommand(srb);
#endif
        /* we've got a command, let's do it! */
        umas->proto_handler(srb, umas);
    }

    /* indicate that the command is done */
    if (umas->srb.result != DID_ABORT << 16) {
        UMAS_VDEBUG("run_scsi_command - scsi cmd done, result=0x%x\n", srb->result);
    } else {
        UMAS_DEBUG("run_scsi_command - scsi command aborted\n");
    }

    return srb->result;
}


int  test_unit_ready(UMAS_DATA_T *umas, int lun)
{
    SCSI_CMD_T      *srb = &umas->srb;

    memset(srb->cmnd, 0, MAX_COMMAND_SIZE);
    srb->cmnd[0] = TEST_UNIT_READY;
    srb->cmnd[1] = lun << 5;
    srb->cmd_len = 6;
    srb->request_bufflen = 0;
    srb->use_sg = 0;
    srb->sc_data_direction = SCSI_DATA_READ;
    srb->sense_buffer[0] = 0;
    srb->sense_buffer[2] = 0;
    return run_scsi_command(srb, umas);
}


int  request_sense(UMAS_DATA_T *umas, int lun)
{
    SCSI_CMD_T      *srb = &umas->srb;

    memset(srb->cmnd, 0, MAX_COMMAND_SIZE);
    srb->cmnd[0] = REQUEST_SENSE;
    srb->cmnd[1] = lun << 5;
    srb->cmnd[4] = 18;
    srb->cmd_len = 12;
    srb->request_bufflen = 18;
    srb->use_sg = 0;
    srb->sc_data_direction = SCSI_DATA_READ;
    return run_scsi_command(srb, umas);
}


int try_test_unit_ready(UMAS_DATA_T *umas)
{
    int  retries;

    for (retries = 0; retries < 10; retries++) {
        if (test_unit_ready(umas, 0) == 0)
            return 0;
    }
    return -1;
}


/*===========================================================================
 *
 * USB Disk Driver
 *
 *==========================================================================*/

int  usbh_umas_disk_status(void)
{
    if (g_umas == NULL)
        return STA_NODISK;
    return 0;
}


DRESULT  usbh_umas_ioctl(int cmd, void *buff)
{
    switch (cmd) {
    case CTRL_SYNC:
        return RES_OK;

    case GET_SECTOR_COUNT:
        *(uint32_t *)buff = g_total_sector_num;
        return RES_OK;

    case GET_SECTOR_SIZE:
        *(uint32_t *)buff = g_sector_size;
        return RES_OK;

    case GET_BLOCK_SIZE:
        *(uint32_t *)buff = g_sector_size;
        return RES_OK;

#if (_FATFS == 82786)
    case CTRL_ERASE_SECTOR:
        return RES_OK;
#else
    case CTRL_TRIM:
        return RES_OK;
#endif
    }
    return RES_PARERR;
}


DRESULT  usbh_umas_read(uint8_t *buff, uint32_t sector_no, int number_of_sector)
{
    SCSI_CMD_T      *srb;
    int             retry = 10;

    if (g_umas == NULL)
        return RES_ERROR;

    //UMAS_DEBUG("usbh_umas_read - buff=0x%x, sector=%d, count=%d\n", (int)buff, sector_no, number_of_sector);
    if (sector_no >= g_total_sector_num) {
        UMAS_DEBUG("usbh_umas_read - exceed disk size! (%d/%d)\n", sector_no, g_total_sector_num);
        return RES_ERROR;
    }

    srb = &g_umas->srb;

do_retry:

    srb->request_buff = (uint8_t *)buff;

    memset(srb->cmnd, 0, MAX_COMMAND_SIZE);
    srb->cmnd[0] = READ_10;
    srb->cmnd[1] = g_disk_lun << 5;
    srb->cmnd[2] = (sector_no >> 24) & 0xFF;
    srb->cmnd[3] = (sector_no >> 16) & 0xFF;
    srb->cmnd[4] = (sector_no >> 8) & 0xFF;
    srb->cmnd[5] = sector_no & 0xFF;
    srb->cmnd[7] = (number_of_sector >> 8) & 0xFF;
    srb->cmnd[8] = number_of_sector & 0xFF;
    srb->cmd_len = 10;

    srb->request_bufflen = g_sector_size * number_of_sector;

    srb->use_sg = 0;
    srb->sc_data_direction = SCSI_DATA_READ;

    if (run_scsi_command(srb, g_umas) != 0) {
        if (retry > 0) {
            retry--;
            goto do_retry;
        }

        UMAS_DEBUG("usbh_umas_read - failed at sector %d (%d)\n", sector_no, number_of_sector);
        return RES_ERROR;
    }
    return RES_OK;
}



DRESULT  usbh_umas_write(uint8_t *buff, uint32_t sector_no, int number_of_sector)
{
    SCSI_CMD_T      *srb;
    int             retry = 3;

    if (g_umas == NULL)
        return RES_ERROR;

    if (sector_no >= g_total_sector_num) {
        UMAS_DEBUG("usbh_umas_write - exceed disk size! (%d/%d)\n", sector_no, g_total_sector_num);
        return RES_ERROR;
    }

    srb = &g_umas->srb;

do_retry:
    //UMAS_DEBUG("usbh_umas_write - Write Sector: %d %d\n", sector_no, number_of_sector);
    srb->request_buff = (uint8_t *)buff;

    memset(srb->cmnd, 0, MAX_COMMAND_SIZE);
    srb->cmnd[0] = WRITE_10;
    srb->cmnd[1] = g_disk_lun << 5;
    srb->cmnd[2] = (sector_no >> 24) & 0xFF;
    srb->cmnd[3] = (sector_no >> 16) & 0xFF;
    srb->cmnd[4] = (sector_no >> 8) & 0xFF;
    srb->cmnd[5] = sector_no & 0xFF;
    srb->cmnd[7] = (number_of_sector >> 8) & 0xFF;
    srb->cmnd[8] = number_of_sector & 0xFF;
    srb->cmd_len = 10;

    srb->request_bufflen = g_sector_size * number_of_sector;

    srb->use_sg = 0;
    srb->sc_data_direction = SCSI_DATA_WRITE;

    if (run_scsi_command(srb, g_umas) != 0) {
        if (retry > 0) {
            retry--;
            goto do_retry;
        }

        UMAS_DEBUG("usbh_umas_write - failed at sector %d (%d)\n", sector_no, number_of_sector);
        return RES_ERROR;
    }
    return RES_OK;
}


int  UMAS_InitUmasDevice(UMAS_DATA_T *umas)
{
    int             retries;
    SCSI_CMD_T      *srb = &umas->srb;
    int8_t          bHasMedia;
    uint32_t        stack_buff[256/4];

    memset(srb, 0, sizeof(SCSI_CMD_T));
    srb->request_buff = (void *)&stack_buff[0];
    memset(srb->request_buff, 0, 256);

    for (g_disk_lun = umas->max_lun; g_disk_lun >= 0; g_disk_lun--) {
        UMAS_DEBUG("\n\n\n******* Read lun %d ******\n\n", g_disk_lun);

        UMAS_DEBUG("INQUIRY ==>\n");
        memset(srb->cmnd, 0, MAX_COMMAND_SIZE);
        srb->cmnd[0] = INQUIRY;
        srb->cmnd[1] = g_disk_lun << 5;
        srb->cmnd[4] = 36;
        srb->cmd_len = 6;
        srb->request_bufflen = 36;
        srb->use_sg = 0;
        srb->sc_data_direction = SCSI_DATA_READ;

        if (run_scsi_command(srb, umas) != 0)
            UMAS_DEBUG("INQUIRY - command failed!\n");

        UMAS_DEBUG("TEST UNIT READY ==>\n");
        bHasMedia = FALSE;
        for (retries = 0; retries < 3; retries++) {
            if (test_unit_ready(umas, g_disk_lun) != 0) {
                //UMAS_DEBUG("TEST_UNIT_READY - command failed\n");
                //break;
            } else if (srb->result == 0) {
                bHasMedia = TRUE;
                break;
            }

            if ((srb->result < 0) || (srb->sense_buffer[2] != UNIT_ATTENTION)) {
                UMAS_DEBUG("TEST_UNIT_READY not UNIT_ATTENTION!\n");
                //break;
            }

            /*
             * If the drive has indicated to us that it doesn't have
             * any media in it, don't bother with any of the rest of
             * this crap.
             */
            if ((srb->result < 0) &&
                    (srb->sense_buffer[2] == UNIT_ATTENTION) &&
                    (srb->sense_buffer[12] == 0x3A)) {
                UMAS_DEBUG("TEST_UNIT_READY - no media\n");
                //break;
            }

            /* Look for non-removable devices that return NOT_READY.
             * Issue command to spin up drive for these cases. */
            if ((srb->result < 0) && (srb->sense_buffer[2] == NOT_READY)) {
                UMAS_DEBUG("TEST_UNIT_READY - not ready, will retry.\n");
            }
            usbh_mdelay(100);
        }

        if (bHasMedia == FALSE)
            continue;

        UMAS_DEBUG("REQUEST SENSE ==>\n");

        if (request_sense(umas, g_disk_lun) == 0) {
            //HexDumpBuffer("REQUEST_SENSE result", srb->request_buff, 256);
            if ((srb->request_buff[16] == 0) && (srb->request_buff[17] == 0))
                UMAS_DEBUG("REQUEST_SENSE - no sense\n");
            else
                UMAS_DEBUG("REQUEST_SENSE - attention %02x %02x\n", srb->request_buff[16], srb->request_buff[17]);
        } else
            UMAS_DEBUG("REQUEST_SENSE failed!\n");;

        UMAS_DEBUG("READ CAPACITY ==>\n");
        memset(srb->cmnd, 0, MAX_COMMAND_SIZE);
        srb->cmnd[0] = READ_CAPACITY;
        srb->cmnd[1] = g_disk_lun << 5;
        srb->cmd_len = 9;
        srb->sense_buffer[0] = 0;
        srb->sense_buffer[2] = 0;
        srb->sc_data_direction = SCSI_DATA_READ;
        if (run_scsi_command(srb, umas) != 0) {
            UMAS_DEBUG("READ_CAPACITY failed!\n");
            continue;
        }

        g_total_sector_num = (srb->request_buff[0] << 24) | (srb->request_buff[1] << 16) |
                             (srb->request_buff[2] << 8) | srb->request_buff[3];
        g_sector_size = (srb->request_buff[4] << 24) | (srb->request_buff[5] << 16) |
                        (srb->request_buff[6] << 8) | srb->request_buff[7];

        UMAS_DEBUG("USB disk found: size=%d MB, uTotalSectorN=%d\n", g_total_sector_num / 2048, g_total_sector_num);

        ((mass_disk_t*)umas)->sector_size = g_sector_size;
        ((mass_disk_t *)umas)->sector_number = g_total_sector_num;

        break;
    }

    printf("g_disk_lun = %d\n", g_disk_lun);

    g_umas = umas;

    return 0;
}

#if 0
// dummy
void UMAS_ScanDeviceLun(UMAS_DATA_T *umas)
{
}
#endif

/// @endcond HIDDEN_SYMBOLS


/** @addtogroup NUC472_442_Device_Driver NUC472/NUC442 Device Driver
  @{
*/

/** @addtogroup NUC472_442_USBH_MASS_Driver USB Host Mass Storage Driver
  @{
*/


/** @addtogroup NUC472_442_USBH_MASS_EXPORTED_FUNCTIONS USB Host Mass Storage Driver Exported Functions
  @{
*/


/**
  * @brief    Read raw data from connected USB Mass Storage disk
  * @param[in] disk      The disk pointer.
  * @param[in] sectorN   The logical sector number to be read.
  * @param[in] scnt      Number of continuous sectors to be read.
  * @param[out] buff     Data buffer used to hold data read from device.
  * @return   Success or not.
  * @retval   0   Success
  * @retval   -1   Failed
  */
int32_t  USBH_MassRawRead(mass_disk_t * disk, uint32_t sectorN, int32_t scnt, uint8_t *buff)
{
    UMAS_DATA_T  *backup_umas;
    int          ret;

    if (!disk)
        return -1;

    backup_umas = g_umas;
    g_umas = (UMAS_DATA_T *)disk;
    ret = usbh_umas_read(buff, sectorN, scnt);
    g_umas = backup_umas;

    if (ret == RES_OK)
        return 0;
    else
        return -1;
}


/**
  * @brief    Write raw data to connected USB Mass Storage disk
  * @param[in] disk      The disk pointer.
  * @param[in] sectorN   The logical sector number to be written.
  * @param[in] scnt      Number of continuous sectors to be written.
  * @param[in] buff      Data to be written to device.
  * @return   Success or not.
  * @retval   0    Success
  * @retval   -1   Failed
  */
int32_t  USBH_MassRawWrite(mass_disk_t *disk, uint32_t sectorN, int32_t scnt, uint8_t *buff)
{
    UMAS_DATA_T  *backup_umas;
    int          ret;

    if (!disk)
        return -1;

    backup_umas = g_umas;
    g_umas = (UMAS_DATA_T *)disk;
    ret = usbh_umas_write(buff, sectorN, scnt);
    g_umas = backup_umas;

    if (ret == RES_OK)
        return 0;
    else
        return -1;
}

/*@}*/ /* end of group NUC472_442_USBH_MASS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC472_442_USBH_MASS_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

