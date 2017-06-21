/**************************************************************************//**
 * @file     uac_driver.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/10/31 10:32a $
 * @brief    NUC472/NUC442 MCU USB Host Audio Class driver
 *
 * @note     Support mono and stero audio input and output.
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "NUC472_442.h"
#include "usbh_core.h"

#include "usbh_uac.h"
#include "uac.h"


/** @addtogroup NUC472_442_Device_Driver NUC472/NUC442 Device Driver
  @{
*/

/** @addtogroup NUC472_442_USBH_AS_Driver USB Host Audio Class Driver
  @{
*/

/** @addtogroup NUC472_442_USBH_AS_EXPORTED_FUNCTIONS USB Host Audio Class Driver Exported Functions
  @{
*/

/// @cond HIDDEN_SYMBOLS

static UAC_DEV_T    g_au_dev[CONFIG_AU_MAX_DEV];
static UAC_INFO_T   g_uac_info[CONFIG_AU_MAX_DEV];

static UAC_DEV_T *g_audev_list = NULL;

int  au_parsing_descriptors(UAC_DEV_T *audev);

static UAC_DEV_T *alloc_as_device(void)
{
    int     i;

    for (i = 0; i < CONFIG_AU_MAX_DEV; i++) {
        if (g_au_dev[i].udev == NULL) {
            memset((char *)&g_au_dev[i], 0, sizeof(UAC_DEV_T));
            g_au_dev[i].ctrl_ifnum = -1;
            g_au_dev[i].au_in_ifnum = -1;
            g_au_dev[i].au_out_ifnum = -1;
            memset((char *)&g_uac_info[i], 0, sizeof(UAC_INFO_T));
            g_au_dev[i].priv = (void *)&g_uac_info[i];
            return &g_au_dev[i];
        }
    }
    return NULL;
}

static void  free_au_device(UAC_DEV_T *audev)
{
    audev->udev = NULL;
}

UAC_DEV_T *find_as_deivce_by_udev(USB_DEV_T *udev)
{
    int     i;

    if (udev == NULL)
        return NULL;

    for (i = 0; i < CONFIG_AU_MAX_DEV; i++) {
        if (g_au_dev[i].udev == udev) {
            return &g_au_dev[i];
        }
    }
    return NULL;
}

int  find_as_device(UAC_DEV_T *hdev)
{
    int    i;

    for (i = 0; i < CONFIG_AU_MAX_DEV; i++) {
        if (&g_au_dev[i] == hdev) {
            return TRUE;
        }
    }
    return FALSE;
}


static int  au_probe(USB_DEV_T *dev, USB_IF_DESC_T *ifd, const USB_DEV_ID_T *id)
{
    EP_INFO_T   *ep_info;
    int         ifnum;
    UAC_DEV_T    *audev, *p;
    int         i;

    if ((ifd->bInterfaceSubClass != SUBCLS_AUDIOCONTROL) &&
            (ifd->bInterfaceSubClass != SUBCLS_AUDIOSTREAMING)) {
        printf("Does not support audio sub-class %x\n", ifd->bInterfaceSubClass);
        return UAC_RET_DRV_NOT_SUPPORTED;
    }


    ifnum = ifd->bInterfaceNumber;

    audev = find_as_deivce_by_udev(dev);
    if (audev == NULL) {
        audev = alloc_as_device();
        if (audev == NULL)
            return USB_ERR_NODEV;
    }

    audev->udev = dev;
    audev->next = NULL;

    USBAS_DBGMSG("au_probe called for ifnum %d\n", ifnum);

    if (ifd->bInterfaceSubClass == SUBCLS_AUDIOSTREAMING) {
        ep_info = NULL;
        for (i = 0; i < dev->ep_list_cnt; i++) {
            if ((dev->ep_list[i].ifnum == ifnum) &&
                    ((dev->ep_list[i].bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_ISOC)) {
                ep_info = &dev->ep_list[i];
                USBAS_DBGMSG("Isochronous Endpoint 0x%x found.\n", ep_info->bEndpointAddress);
            }
        }
        if (ep_info == NULL) {
            USBAS_DBGMSG("couldn't find isochronous endpoints\n");
            return USB_ERR_NODEV;
        }

        if ((ep_info->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) {
            audev->ep_au_in = ep_info;
            audev->au_in_ifnum = ifnum;
            USBAS_DBGMSG("Audio in stream interface is %d\n", ifnum);
        } else {
            audev->ep_au_out = ep_info;
            audev->au_out_ifnum = ifnum;
            USBAS_DBGMSG("Audio out stream interface is %d\n", ifnum);
        }
    } else {
        audev->ctrl_ifnum = ifnum;
        USBAS_DBGMSG("Audio control interface is %d\n", ifnum);
    }

    /*
     *  Chaining newly found Audio Class device to end of Audio Class device list.
     */
    if (g_audev_list == NULL)
        g_audev_list = audev;
    else {
        for (p = g_audev_list; p->next != NULL; p = p->next)
            ;
        p->next = audev;
    }

    return uac_config_parser(audev);
}


static void  au_disconnect(USB_DEV_T *dev)
{
    UAC_DEV_T   *audev, *p;
    int    i;

    USBAS_DBGMSG("Audio Class device disconnected!\n");

    audev = find_as_deivce_by_udev(dev);
    if (audev == NULL)
        return;

    for (i = 0; i < ISO_IN_URB_CNT; i++) {
        if (audev->urbin[i]) {
            USBH_UnlinkUrb(audev->urbin[i]);
            USBH_FreeUrb(audev->urbin[i]);
        }
    }

    for (i = 0; i < ISO_OUT_URB_CNT; i++) {
        if (audev->urbout[i]) {
            USBH_UnlinkUrb(audev->urbout[i]);
            USBH_FreeUrb(audev->urbout[i]);
        }
    }

    /*
     *  Remove this Audio Class device from device list.
     */
    if (audev == g_audev_list)
        g_audev_list = g_audev_list->next;
    else {
        for (p = g_audev_list; p != NULL; p = p->next) {
            if (p->next == audev) {
                p->next = audev->next;
                break;
            }
        }
    }
    free_au_device(audev);
}


static USB_DEV_ID_T  au_id_table[] = {
    USB_DEVICE_ID_MATCH_INT_CLASS,    /* match_flags */
    0, 0, 0, 0, 0, 0, 0,
    UAC_IFACE_CODE,                    /* Audio Interface Class Code */
    0,                                /* Audio Interface Subclass Codes, support streaming only */
    0, 0
};


static USB_DRIVER_T  au_driver = {
    "audio class driver",
    au_probe,
    au_disconnect,
    au_id_table,
    NULL,                       /* suspend */
    NULL,                       /* resume */
    {NULL,NULL}                 /* driver_list */
};


/// @endcond HIDDEN_SYMBOLS

/**
  * @brief    Initialize this USB Audio Class driver.
  * @return   None
  */
void UAC_Init(void)
{
    memset((char *)&g_au_dev[0], 0, sizeof(g_au_dev));
    g_audev_list = NULL;
    USBH_RegisterDriver(&au_driver);
}


/**
 *  @brief   Get a list of currently connected USB Audio Class devices.
 *  @return  List of current connected UAC devices.
 *  @retval  NULL       There's no UAC devices found.
 *  @retval  Otherwise  A list of connected UAC devices.
 *
 *  The Audio Class devices are chained by the "next" member of UAC_DEV_T.
 */
UAC_DEV_T * UAC_GetDeviceList(void)
{
    return g_audev_list;
}


/*@}*/ /* end of group NUC472_442_USBH_AS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC472_442_USBH_AS_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

