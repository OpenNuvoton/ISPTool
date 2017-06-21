/**************************************************************************//**
 * @file     hid_driver.c
 * @version  V1.00
 * $Revision: 12 $
 * $Date: 15/04/27 3:26p $
 * @brief    NUC472/NUC442 MCU USB Host HID driver
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "NUC472_442.h"
#include "usbh_core.h"

#include "usbh_hid.h"


/** @addtogroup NUC472_442_Device_Driver NUC472/NUC442 Device Driver
  @{
*/

/** @addtogroup NUC472_442_USBH_HID_Driver USB Host HID Driver
  @{
*/

/** @addtogroup NUC472_442_USBH_HID_EXPORTED_FUNCTIONS USB Host HID Driver Exported Functions
  @{
*/

/// @cond HIDDEN_SYMBOLS

static HID_DEV_T    g_hid_dev[CONFIG_HID_MAX_DEV];

static HID_DEV_T *g_hdev_list = NULL;

static HID_DEV_T *alloc_hid_device(void)
{
    int     i;

    for (i = 0; i < CONFIG_HID_MAX_DEV; i++) {
        if (g_hid_dev[i].udev == NULL) {
            memset((char *)&g_hid_dev[i], 0, sizeof(HID_DEV_T));
            return &g_hid_dev[i];
        }
    }
    return NULL;
}

void  free_hid_device(HID_DEV_T *hid_dev)
{
    hid_dev->udev = NULL;
    memset((char *)hid_dev, 0, sizeof(HID_DEV_T));
}

HID_DEV_T *find_hid_deivce_by_urb(URB_T *urb)
{
    int     i;

    if (urb->dev == NULL)
        return NULL;

    for (i = 0; i < CONFIG_HID_MAX_DEV; i++) {
        if (g_hid_dev[i].udev == urb->dev) {
            if ((g_hid_dev[i].urbin == urb) || (g_hid_dev[i].urbout == urb))
                return &g_hid_dev[i];
        }
    }
    return NULL;
}

int  find_hid_device(HID_DEV_T *hdev)
{
    int    i;

    for (i = 0; i < CONFIG_HID_MAX_DEV; i++) {
        if (&g_hid_dev[i] == hdev) {
            return TRUE;
        }
    }
    return FALSE;
}


static int  usbhid_probe(USB_DEV_T *dev, USB_IF_DESC_T *ifd, const USB_DEV_ID_T *id)
{
    EP_INFO_T   *ep_info;
    int         ifnum;
    HID_DEV_T   *hid, *p;
    int         i;

    HID_DBGMSG("usbhid_probe - dev=0x%x\n", (int)dev);

    ifnum = ifd->bInterfaceNumber;

    HID_DBGMSG("HID probe called for ifnum %d\n", ifnum);

    ep_info = NULL;
    for (i = 0; i < dev->ep_list_cnt; i++) {
        if ((dev->ep_list[i].ifnum == ifnum) &&
                ((dev->ep_list[i].bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT)) {
            ep_info = &dev->ep_list[i];
            HID_DBGMSG("Interrupt Endpoint 0x%x found.\n", ep_info->bEndpointAddress);
        }
    }
    if (ep_info == NULL) {
        HID_DBGMSG("couldn't find an input interrupt endpoint\n");
        return USB_ERR_NODEV;
    }

    hid = alloc_hid_device();
    if (hid == NULL)
        return USB_ERR_NODEV;

    HID_DBGMSG("New HID device 0x%x.\n", (int)hid);

    hid->ifnum = ifnum;
    hid->udev = dev;
    hid->next = NULL;
    hid->bSubClassCode = ifd->bInterfaceSubClass;
    hid->bProtocolCode = ifd->bInterfaceProtocol;

    /*
     *  Chaining newly found HID device to end of HID device list.
     */
    if (g_hdev_list == NULL)
        g_hdev_list = hid;
    else {
        for (p = g_hdev_list; p->next != NULL; p = p->next)
            ;
        p->next = hid;
    }

    return 0;
}


static void  hid_disconnect(USB_DEV_T *dev)
{
    HID_DEV_T   *hid_dev, *p;
    int    i;

    HID_DBGMSG("HID device disconnected!\n");

    for (i = 0; i < CONFIG_HID_MAX_DEV; i++) {
        if (g_hid_dev[i].udev == dev) {
            hid_dev = &g_hid_dev[i];

            if (hid_dev->urbin) {
                USBH_UnlinkUrb(hid_dev->urbin);
                USBH_FreeUrb(hid_dev->urbin);
            }
            if (hid_dev->urbout) {
                USBH_UnlinkUrb(hid_dev->urbout);
                USBH_FreeUrb(hid_dev->urbout);
            }

            /*
             *  Remove this HID device from HID device list.
             */
            if (hid_dev == g_hdev_list)
                g_hdev_list = g_hdev_list->next;
            else {
                for (p = g_hdev_list; p != NULL; p = p->next) {
                    if (p->next == hid_dev) {
                        p->next = hid_dev->next;
                        break;
                    }
                }
            }
            free_hid_device(hid_dev);
        }
    }
}


#define USB_INTERFACE_CLASS_HID         3

static USB_DEV_ID_T  hid_id_table[] = {
    USB_DEVICE_ID_MATCH_INT_CLASS,     /* match_flags */
    0, 0, 0, 0, 0, 0, 0,
    USB_INTERFACE_CLASS_HID,           /* bInterfaceClass */
    0, 0, 0
};


static USB_DRIVER_T  hid_driver = {
    "hid driver",
    usbhid_probe,
    hid_disconnect,
    hid_id_table,
    NULL,                       /* suspend */
    NULL,                       /* resume */
    {NULL,NULL}                 /* driver_list */
};


EP_INFO_T *hid_get_ep_info(USB_DEV_T *dev, int ifnum, uint16_t dir)
{
    int   i;

    for (i = 0; i < dev->ep_list_cnt; i++) {
        if ((dev->ep_list[i].ifnum == ifnum) &&
                ((dev->ep_list[i].bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT)) {
            if ((dev->ep_list[i].bEndpointAddress & USB_ENDPOINT_DIR_MASK) == dir) {
                //printf("Endpoint found: 0x%x\n", dev->ep_list[i].bEndpointAddress);
                return &dev->ep_list[i];
            }
        }
    }
    return NULL;
}

/// @endcond /* HIDDEN_SYMBOLS */


/**
  * @brief    Init USB Host HID driver.
  * @return   None
  */
void USBH_HidInit(void)
{
    memset((char *)&g_hid_dev[0], 0, sizeof(g_hid_dev));
    g_hdev_list = NULL;
    USBH_RegisterDriver(&hid_driver);
}


/**
 *  @brief   Get a list of currently connected USB Hid devices.
 *  @return  List of HID devices.
 *  @retval  NULL       There's no HID device found.
 *  @retval  Otherwise  A list of connected HID devices.
 *
 *  The HID devices are chained by the "next" member of HID_DEV_T.
 */
HID_DEV_T * USBH_HidGetDeviceList(void)
{
    return g_hdev_list;
}


/*@}*/ /* end of group NUC472_442_USBH_HID_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC472_442_USBH_HID_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

