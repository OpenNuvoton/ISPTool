/**************************************************************************//**
 * @file     lw_umass.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/12/07 9:16a $
 * @brief    Light-weight USB mass storage class driver
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NUC472_442.h"
#include "diskio.h"                // FATFS header
#include "usb.h"
#include "lw_usbh.h"
#include "lw_umass.h"


/** @addtogroup NUC472_442_Device_Driver NUC472/NUC442 Device Driver
  @{
*/

/** @addtogroup NUC472_442_LWHCD_Driver LW_USBH Driver
  @{
*/

/** @addtogroup NUC472_442_LWHCD_EXPORTED_FUNCTIONS LW_USBH Exported Functions
  @{
*/

/// @cond HIDDEN_SYMBOLS


#define umass_dbg_msg       printf
//#define umass_dbg_msg(...)


static int       iface_num;
static uint16_t  bulk_in_ep_addr, bulk_out_ep_addr;
static uint8_t   bulk_in_toggle, bulk_out_toggle;

int    g_disk_lun, g_max_lun;

#ifdef __ICCARM__
#pragma data_alignment=4
static struct bulk_cb_wrap   g_cb;
#endif

#ifdef __ICCARM__
#pragma data_alignment=4
static struct bulk_cs_wrap   g_cs;
#endif

#ifdef __ARMCC_VERSION
static __align(4) struct bulk_cb_wrap   g_cb;
static __align(4) struct bulk_cs_wrap   g_cs;
#endif


static uint32_t  g_total_sector_num, g_sector_size;


int umas_bulk_xfer(uint16_t ep_addr, uint8_t *toggle,
                   uint8_t *data_buff, int data_len, int timeout)
{
    int  ret, retry = 3;

    for ( ; retry; retry--) {
        ret = usbh_drv_bulk_xfer(ep_addr, toggle, data_buff, data_len, timeout);
        if (ret == 0)
            return 0;
    }
    return ret;
}


static void get_max_lun()
{
    int       ret;
    uint8_t   buff[2];

    g_max_lun = 0;

    /*------------------------------------------------------------------------------------*/
    /* Issue GET DESCRIPTOR command to get device descriptor                              */
    /*------------------------------------------------------------------------------------*/

    ret = usbh_drv_ctrl_req(0xA1, 0xFE, 0, 0, 1, 1, buff, 0);
    if (ret < 0) {
        umass_dbg_msg("Get Max Lun command failed!\n");
        if (ret == USBH_RET_STALL)
            usbh_clear_halt(0);
        return;
    }
    g_max_lun = buff[0];
    umass_dbg_msg("Max lun is %d\n", g_max_lun);
}


static void umass_reset()
{
#if 1
    int       ret;

    printf("Reset UMAS device...\n");

    ret = usbh_drv_ctrl_req(0x21, 0xFF, 0, 0, 1, 1, NULL, 1);
    if (ret < 0) {
        umass_dbg_msg("UAMSS reset request failed!\n");
        return;
    }
#endif

    usbh_clear_halt(bulk_in_ep_addr);
}

static int __tag = 0x10e24388;

static int  umass_run_command(uint8_t *buff, int data_len, int is_data_in, int timeout)
{
    int   ret;

    g_cb.Signature = UMAS_BULK_CB_SIGN;
    g_cb.Tag = __tag++;
    g_cb.DataTransferLength = data_len;
    g_cb.Lun = g_disk_lun;

    ret = umas_bulk_xfer(bulk_out_ep_addr, &bulk_out_toggle, (uint8_t *)&g_cb, 31, 10000);
    if (ret < 0)
        return ret;

    if (data_len > 0) {
        if (is_data_in)
            ret = umas_bulk_xfer(bulk_in_ep_addr, &bulk_in_toggle, buff, data_len, timeout);
        else
            ret = umas_bulk_xfer(bulk_out_ep_addr, &bulk_out_toggle, buff, data_len, timeout);
    }

    if (ret < 0)
        return ret;

    ret = umas_bulk_xfer(bulk_in_ep_addr, &bulk_in_toggle, (uint8_t *)&g_cs, 13, 1000);
    if (ret < 0)
        return ret;

    if (g_cs.Status != 0) {
        umass_dbg_msg("CSW status error.\n");
        return USBH_RET_ERR_CLASS_CMD;
    }

    //umass_dbg_msg("command 0x%0x done.\n", g_cb.CDB[0]);

    return ret;
}


static int  umass_inquiry()
{
    int  ret;

    umass_dbg_msg("INQUIRY...\n");

    memset((uint8_t *)&g_cb, 0, sizeof(g_cb));
    g_cb.Flags = 0x80;
    g_cb.Length = 6;
    g_cb.CDB[0] = INQUIRY;         /* INQUIRY */
    g_cb.CDB[1] = g_disk_lun << 5;
    g_cb.CDB[4] = 36;

    ret = umass_run_command(_transfer_buffer, 36, 1, 10000);
    if (ret < 0) {
        umass_dbg_msg("INQUIRY command failed. [%d]\n", ret);
        return ret;
    } else {
        umass_dbg_msg("INQUIRY command success.\n");
    }
    return ret;
}


static int  umass_request_sense()
{
    int  ret;

    umass_dbg_msg("REQUEST_SENSE...\n");

    memset((uint8_t *)&g_cb, 0, sizeof(g_cb));
    g_cb.Flags = 0x80;
    g_cb.Length = 12;
    g_cb.CDB[0] = REQUEST_SENSE;
    g_cb.CDB[1] = g_disk_lun << 5;
    g_cb.CDB[4] = 18;

    ret = umass_run_command(_transfer_buffer, 18, 1, 10000);
    if (ret < 0) {
        umass_dbg_msg("REQUEST_SENSE command failed.\n");
        if (ret == USBH_RET_STALL)
            umass_reset();
        return ret;
    } else {
        umass_dbg_msg("REQUEST_SENSE command success.\n");
        if (_transfer_buffer[2] != 0x6) {
            umass_dbg_msg("Device is still not attention. 0x%x\n", _transfer_buffer[2]);
            return -1;
        }
    }

    return ret;
}


static int  umass_test_unit_ready()
{
    int  ret;

    umass_dbg_msg("TEST_UNIT_READY...\n");

    memset((uint8_t *)&g_cb, 0, sizeof(g_cb));
    g_cb.Flags = 0x80;
    g_cb.Length = 6;
    g_cb.CDB[0] = TEST_UNIT_READY;
    g_cb.CDB[1] = g_disk_lun << 5;

    ret = umass_run_command(NULL, 0, 1, 10000);
    if (ret < 0) {
        if (ret == USBH_RET_STALL)
            umass_reset();
        return ret;
    } else {
        umass_dbg_msg("TEST_UNIT_READY command success.\n");
    }
    return ret;
}


DRESULT  usbh_umas_read(uint8_t *buff, uint32_t sector_no, int number_of_sector)
{
    int   ret;

    //umass_dbg_msg("read sector 0x%x\n", sector_no);

    memset((uint8_t *)&g_cb, 0, sizeof(g_cb));
    g_cb.Flags = 0x80;
    g_cb.Length = 10;
    g_cb.CDB[0] = READ_10;
    g_cb.CDB[1] = g_disk_lun << 5;
    g_cb.CDB[2] = (sector_no >> 24) & 0xFF;
    g_cb.CDB[3] = (sector_no >> 16) & 0xFF;
    g_cb.CDB[4] = (sector_no >> 8) & 0xFF;
    g_cb.CDB[5] = sector_no & 0xFF;
    g_cb.CDB[7] = (number_of_sector >> 8) & 0xFF;
    g_cb.CDB[8] = number_of_sector & 0xFF;

    ret = umass_run_command(buff, number_of_sector * 512, 1, 3000);
    if (ret < 0) {
        umass_dbg_msg("usbh_umas_read failed!\n");
        return RES_ERROR;
    }
    return RES_OK;
}


DRESULT  usbh_umas_write(uint8_t *buff, uint32_t sector_no, int number_of_sector)
{
    int   ret;

    memset((uint8_t *)&g_cb, 0, sizeof(g_cb));
    g_cb.Flags = 0;
    g_cb.Length = 10;
    g_cb.CDB[0] = WRITE_10;
    g_cb.CDB[1] = g_disk_lun << 5;
    g_cb.CDB[2] = (sector_no >> 24) & 0xFF;
    g_cb.CDB[3] = (sector_no >> 16) & 0xFF;
    g_cb.CDB[4] = (sector_no >> 8) & 0xFF;
    g_cb.CDB[5] = sector_no & 0xFF;
    g_cb.CDB[7] = (number_of_sector >> 8) & 0xFF;
    g_cb.CDB[8] = number_of_sector & 0xFF;

    ret = umass_run_command(buff, number_of_sector * 512, 0, 3000);
    if (ret < 0) {
        umass_dbg_msg("usbh_umas_write failed!\n");
        return RES_ERROR;
    }
    return RES_OK;
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


int  usbh_umas_disk_status(void)
{
    if ((usbh_probe_port(0) != USBH_RET_DEV_CONN_KEEP) &&
            (usbh_probe_port(1) != USBH_RET_DEV_CONN_KEEP))
        return STA_NODISK;
    return 0;
}


static int  umass_init_device(void)
{
    int             retries, ret;
    volatile int    i;
    int8_t          bHasMedia = 0;

    for (g_disk_lun = 0; g_disk_lun <= g_max_lun; g_disk_lun++) {
        umass_dbg_msg("\n\n\n******* Read lun %d ******\n\n", g_disk_lun);

        bHasMedia = 0;
        for (retries = 0; retries < 3; retries++) {
            if (umass_inquiry() == USBH_RET_STALL)
                umass_reset();

            if (umass_test_unit_ready() == USBH_RET_NO_ERR) {
                bHasMedia = 1;
                break;
            }

            if (umass_request_sense() == USBH_RET_NO_ERR) {
                bHasMedia = 1;
                break;
            }

            for (i = 0; i < MINISEC_100; i++);
        }

        if (!bHasMedia)
            continue;

        for (retries = 0; retries < 3; retries++) {
            umass_dbg_msg("READ CAPACITY ==>\n");

            memset((uint8_t *)&g_cb, 0, sizeof(g_cb));
            g_cb.Flags = 0x80;
            g_cb.Length = 10;
            g_cb.CDB[0] = READ_CAPACITY;
            g_cb.CDB[1] = g_disk_lun << 5;

            ret = umass_run_command(_transfer_buffer, 8, 1, 1000);
            if (ret < 0) {
                umass_dbg_msg("READ_CAPACITY failed!\n");
                if (ret == USBH_RET_STALL)
                    umass_reset();
                continue;
            } else
                break;
        }

        if (retries >= 3)
            return USBH_RET_ERR_DEV_INIT;

        g_total_sector_num = (_transfer_buffer[0] << 24) | (_transfer_buffer[1] << 16) |
                             (_transfer_buffer[2] << 8) | _transfer_buffer[3];
        g_sector_size = (_transfer_buffer[4] << 24) | (_transfer_buffer[5] << 16) |
                        (_transfer_buffer[6] << 8) | _transfer_buffer[7];
        umass_dbg_msg("USB disk found: size=%d MB, uTotalSectorN=%d\n", g_total_sector_num / 2048, g_total_sector_num);
        break;
    }

    if (bHasMedia) {
        umass_dbg_msg("g_disk_lun = %d\n", g_disk_lun);
        return 0;
    }
    return USBH_RET_ERR_DEV_INIT;
}

/// @endcond


/**
  * @brief    Try to probe and initialize an USB mass storage device.
  * @return   Success or not.
  * @retval   0   Success
  * @retval   Otherwise   Failed
  */
int usbh_probe_umass(void)
{
    USB_CONFIG_DESC_T *cfg_desc;
    USB_IF_DESC_T  *if_desc;
    USB_EP_DESC_T  *ep_desc;
    uint8_t        *bptr;
    int            tlen;
    int  ret;

    /*------------------------------------------------------------------------------------*/
    /* Issue GET DESCRIPTOR command to get configuration descriptor                       */
    /*------------------------------------------------------------------------------------*/
    ret = get_config_descriptor(_transfer_buffer);
    if (ret < 0) {
        umass_dbg_msg("GET Config Descriptor command failed!!\n");
        return ret;
    }

    /*------------------------------------------------------------------------------------*/
    /* Parsing configuration descriptor and find an USB Mass Storage class interface      */
    /*------------------------------------------------------------------------------------*/
    iface_num = -1;
    bulk_in_ep_addr = 0;
    bulk_out_ep_addr = 0;

    cfg_desc = (USB_CONFIG_DESC_T *)_transfer_buffer;
    tlen = cfg_desc->wTotalLength;
    bptr = (uint8_t *)cfg_desc;

    while (tlen > 0) {
        switch (bptr[1]) {
        case USB_DT_INTERFACE:
            if_desc = (USB_IF_DESC_T *)bptr;
            if ((if_desc->bInterfaceClass == USB_CLASS_MASS_STORAGE) &&
                    ((if_desc->bInterfaceSubClass == UMAS_SC_SCSI) || (if_desc->bInterfaceSubClass == UMAS_SC_8070)) &&
                    (if_desc->bInterfaceProtocol == UMAS_PR_BULK))
                iface_num = if_desc->bInterfaceNumber;
            else {
                iface_num = -1;
                bulk_in_ep_addr = 0;
                bulk_out_ep_addr = 0;
            }
            break;

        case USB_DT_ENDPOINT:
            ep_desc = (USB_EP_DESC_T *)bptr;
            if ((iface_num != -1) && ((ep_desc->bmAttributes & 0x3) == 0x2)) {
                if (ep_desc->bEndpointAddress & 0x80)
                    bulk_in_ep_addr = ep_desc->bEndpointAddress;
                else
                    bulk_out_ep_addr = ep_desc->bEndpointAddress;
            }
            break;
        }
        if (bptr[0] == 0)
            break;
        tlen -= bptr[0];
        bptr += bptr[0];
    }

    if ((iface_num == -1) || (bulk_in_ep_addr == 0) || (bulk_out_ep_addr == 0))
        return USBH_RET_UNSUPPORT;

    bulk_in_toggle = 0;
    bulk_out_toggle = 0;

    umass_dbg_msg("USB Mass Storage device found.\n");

    /* Set configuration */
    ret = usbh_set_configuration(cfg_desc->bConfigurationValue);
    if (ret < 0)
        return ret;

    get_max_lun();

    return umass_init_device();
}

/*@}*/ /* end of group NUC472_442_LWHCD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC472_442_LWHCD_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


