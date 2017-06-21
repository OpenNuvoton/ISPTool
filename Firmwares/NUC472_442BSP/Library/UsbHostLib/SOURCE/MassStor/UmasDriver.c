/* Driver for USB Mass Storage compliant devices
 *
 * $Id: usb.c,v 1.61 2001/01/13 00:10:59 mdharm Exp $
 *
 * Current development and maintenance by:
 *   (c) 1999, 2000 Matthew Dharm (mdharm-usb@one-eyed-alien.net)
 *
 * Developed with the assistance of:
 *   (c) 2000 David L. Brown, Jr. (usb-storage@davidb.org)
 *
 * Initial work by:
 *   (c) 1999 Michael Gee (michael@linuxspecific.com)
 *
 * usb_device_id support by Adam J. Richter (adam@yggdrasil.com):
 *   (c) 2000 Yggdrasil Computing, Inc.
 *
 * This driver is based on the 'USB Mass Storage Class' document. This
 * describes in detail the protocol used to communicate with such
 * devices.  Clearly, the designers had SCSI and ATAPI commands in
 * mind when they created this document.  The commands are all very
 * similar to commands in the SCSI-II and ATAPI specifications.
 *
 * It is important to note that in a number of cases this class
 * exhibits class-specific exemptions from the USB specification.
 * Notably the usage of NAK, STALL and ACK differs from the norm, in
 * that they are used to communicate wait, failed and OK on commands.
 *
 * Also, for certain devices, the interrupt endpoint is used to convey
 * status of a command.
 *
 * Please see http://www.one-eyed-alien.net/~mdharm/linux-usb for more
 * information about this driver.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 *  MODULE_AUTHOR("Matthew Dharm <mdharm-usb@one-eyed-alien.net>");
 *  MODULE_DESCRIPTION("USB Mass Storage driver for Linux");
 */

/**************************************************************************//**
 * @file     UmasDriver.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/03/23 8:34a $
 * @brief    NUC472/NUC442 MCU USB Host Mass Storage Library
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

extern UMAS_DATA_T  *g_umas;

#define UNUSUAL_DEV(id_vendor, id_product, bcdDeviceMin, bcdDeviceMax, \
                    vendorName, productName,useProtocol, useTransport, \
                    initFunction, flags) \
USB_DEVICE_VER(id_vendor, id_product, bcdDeviceMin,bcdDeviceMax)

static USB_DEV_ID_T  _UmasDeviceIDs[] = {
//#include "unusual_devs.h"
#undef UNUSUAL_DEV
    /* Control/Bulk transport for all SubClass values */
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_RBC, UMAS_PR_CB),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_8020, UMAS_PR_CB),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_QIC, UMAS_PR_CB),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_UFI, UMAS_PR_CB),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_8070, UMAS_PR_CB),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_SCSI, UMAS_PR_CB),

    /* Control/Bulk/Interrupt transport for all SubClass values */
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_RBC, UMAS_PR_CBI),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_8020, UMAS_PR_CBI),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_QIC, UMAS_PR_CBI),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_UFI, UMAS_PR_CBI),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_8070, UMAS_PR_CBI),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_SCSI, UMAS_PR_CBI),

    /* Bulk-only transport for all SubClass values */
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_RBC, UMAS_PR_BULK),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_8020, UMAS_PR_BULK),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_QIC, UMAS_PR_BULK),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_UFI, UMAS_PR_BULK),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_8070, UMAS_PR_BULK),
    USB_INTERFACE_INFO(USB_CLASS_MASS_STORAGE, UMAS_SC_SCSI, UMAS_PR_BULK),

    /* Terminating entry */
    { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};


#undef UNUSUAL_DEV
#define UNUSUAL_DEV(idVendor, idProduct, bcdDeviceMin, bcdDeviceMax, \
                    vendor_name, product_name, use_protocol, use_transport, \
                    init_function, Flags) \
{ vendor_name, product_name, use_protocol, use_transport, init_function, Flags }

static UMAS_UUDEV_T  _UmasUUDevList[] = {
#undef UNUSUAL_DEV
    /* Control/Bulk transport for all SubClass values */
    { UMAS_SC_RBC,  UMAS_PR_CB },
    { UMAS_SC_8020, UMAS_PR_CB },
    { UMAS_SC_QIC,  UMAS_PR_CB },
    { UMAS_SC_UFI,  UMAS_PR_CB },
    { UMAS_SC_8070, UMAS_PR_CB },
    { UMAS_SC_SCSI, UMAS_PR_CB },

    /* Control/Bulk/Interrupt transport for all SubClass values */
    { UMAS_SC_RBC,  UMAS_PR_CBI },
    { UMAS_SC_8020, UMAS_PR_CBI },
    { UMAS_SC_QIC,  UMAS_PR_CBI },
    { UMAS_SC_UFI,  UMAS_PR_CBI },
    { UMAS_SC_8070, UMAS_PR_CBI },
    { UMAS_SC_SCSI, UMAS_PR_CBI },

    /* Bulk-only transport for all SubClass values */
    { UMAS_SC_RBC,  UMAS_PR_BULK },
    { UMAS_SC_8020, UMAS_PR_BULK },
    { UMAS_SC_QIC,  UMAS_PR_BULK },
    { UMAS_SC_UFI,  UMAS_PR_BULK },
    { UMAS_SC_8070, UMAS_PR_BULK },
    { UMAS_SC_SCSI, UMAS_PR_BULK },

    /* Terminating entry */
    { 0 }
};


static UMAS_DATA_T  _umas_pool[UMAS_MAX_DEV];
static uint8_t      _umac_alloc_mark[UMAS_MAX_DEV];

#ifdef USE_NVTFAT
extern void free_umas_drive(UMAS_DRIVE_T * umas_drive);
#endif

static UMAS_DATA_T * alloc_umas()
{
    int     i;
    for (i = 0; i < UMAS_MAX_DEV; i++) {
        if (_umac_alloc_mark[i] == 0) {
            _umac_alloc_mark[i] = 1;
            return &_umas_pool[i];
        }
    }
    return NULL;
}


void free_umas(UMAS_DATA_T * umas)
{
    int     i;

    g_umas = NULL;
    for (i = 0; i < UMAS_MAX_DEV; i++) {
        if (umas == &_umas_pool[i])
            _umac_alloc_mark[i] = 0;
    }
}


static int  storage_probe(USB_DEV_T *dev, USB_IF_DESC_T *ifd, const USB_DEV_ID_T *id);
static void  storage_disconnect(USB_DEV_T *dev);

USB_DRIVER_T  _UsbMassStorageDriver = {
    "UMAS",
    storage_probe,
    storage_disconnect,
    _UmasDeviceIDs,
    NULL,                       /* suspend */
    NULL,                       /* resume */
    {NULL,NULL}                 /* driver_list */
};


/*
 * Set up the IRQ pipe and handler
 * Note that this function assumes that all the data in the umas_data
 * strucuture is current.  This includes the ep_int field, which gives umas
 * the endpoint for the interrupt.
 * Returns non-zero on failure, zero on success
 */
static int  usb_stor_allocate_irq(UMAS_DATA_T *umas)
{
    uint32_t    pipe;
    int         maxp;
    int         result;
    uint8_t     bEndpointAddress, bInterval;

    UMAS_DEBUG("Allocating IRQ for CBI transport\n");

    bEndpointAddress = umas->pusb_dev->ep_list[umas->ep_int].bEndpointAddress;
    bInterval = umas->pusb_dev->ep_list[umas->ep_int].bInterval;

    /* allocate the URB */
    umas->irq_urb = USBH_AllocUrb();
    if (!umas->irq_urb) {
        UMAS_DEBUG("couldn't allocate interrupt URB");
        return USB_ERR_NOMEM;
    }

    /* calculate the pipe and max packet size */
    pipe = usb_rcvintpipe(umas->pusb_dev, (bEndpointAddress & USB_ENDPOINT_NUMBER_MASK));
    maxp = usb_maxpacket(umas->pusb_dev, pipe, usb_pipeout(pipe));
    if (maxp > sizeof(umas->irqbuf))
        maxp = sizeof(umas->irqbuf);

    /* fill in the URB with our data */
    FILL_INT_URB(umas->irq_urb, umas->pusb_dev, pipe, umas->irqbuf, maxp,
                 UMAS_CbiIrq, umas, bInterval);   /* UMAS_CbiIrq() in UmasTransport.c */

    umas->ip_wanted = 1;
    /* submit the URB for processing */
    result = USBH_SubmitUrb(umas->irq_urb);
    if (result)
        USBH_FreeUrb(umas->irq_urb);

    return result;
}



/* Probe to see if a new device is actually a SCSI device */
static int  storage_probe(USB_DEV_T *dev, USB_IF_DESC_T *ifd, const USB_DEV_ID_T *id)
{
    int             i, ifnum;
    const  int      id_index = id - _UmasDeviceIDs;
    UMAS_UUDEV_T    *unusual_dev;
    UMAS_DATA_T     *umas = NULL;
    int             ep_in = 0;
    int             ep_out = 0;
    int             ep_int = 0;
    uint8_t         subclass = 0;
    uint8_t         protocol = 0;

    ifnum = ifd->bInterfaceNumber;

    //UMAS_DEBUG("id_index calculated to be: %d\n", id_index);

    if (id_index < sizeof(_UmasUUDevList) / sizeof(_UmasUUDevList[0])) {
        unusual_dev = &_UmasUUDevList[id_index];
    } else
        /* no, we can't support it */
        return USB_ERR_NODEV;

    /* At this point, we know we've got a live one */
    UMAS_DEBUG("USB Mass Storage device detected\n");

    /* Determine subclass and protocol, or copy from the interface */
    subclass = unusual_dev->useProtocol;
    protocol = unusual_dev->useTransport;

    /*
     * Find the endpoints we need
     * We are expecting a minimum of 2 endpoints - in and out (bulk).
     * An optional interrupt is OK (necessary for CBI protocol).
     * We will ignore any others.
     */
    for (i = 1; i < dev->ep_list_cnt; i++) {
        if (dev->ep_list[i].ifnum != ifnum)
            continue;

        /* is it an BULK endpoint? */
        if ((dev->ep_list[i].bmAttributes &
                USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK) {
            /* BULK in or out? */
            if (dev->ep_list[i].bEndpointAddress & USB_DIR_IN)
                ep_in = i;
            else
                ep_out = i;
        }

        /* is it an interrupt endpoint? */
        if ((dev->ep_list[i].bmAttributes &
                USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT) {
            ep_int = i;
        }
    }

    //UMAS_DEBUG("Endpoints: In: %d Out: %d Int: %d\n", ep_in, ep_out, ep_int, ep_int);

    /* Do some basic sanity checks, and bail if we find a problem */
    if (!ep_in || !ep_out || ((protocol == UMAS_PR_CBI) && !ep_int)) {
        UMAS_DEBUG("Endpoint sanity check failed! Rejecting dev.\n");
        return USB_ERR_NODEV;
    }

    umas = alloc_umas();
    if (umas == NULL) {
        UMAS_DEBUG("umas - Out of memory\n");
        return USB_ERR_NOMEM;
    }
    memset((char *)umas, 0, sizeof(UMAS_DATA_T));

    /* allocate the URB we're going to use */
    umas->current_urb = USBH_AllocUrb();
    if (!umas->current_urb) {
        free_umas(umas);
        return USB_ERR_NOMEM;
    }

    /* copy over the subclass and protocol data */
    umas->subclass = subclass;
    umas->protocol = protocol;
    umas->flags = 0;
    umas->unusual_dev = unusual_dev;

    /* copy over the endpoint data */
    if (ep_in)
        umas->ep_in = dev->ep_list[ep_in].bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
    if (ep_out)
        umas->ep_out = dev->ep_list[ep_out].bEndpointAddress & USB_ENDPOINT_NUMBER_MASK;
    umas->ep_int = ep_int;

    /* establish the connection to the new device */
    umas->ifnum = ifnum;
    umas->pusb_dev = dev;
    umas->vendor_id = dev->descriptor.idVendor;
    umas->product_id = dev->descriptor.idProduct;
    umas->sector_size = 0;
    umas->sector_number = 0;

    /*
     * Set the handler pointers based on the protocol
     * Again, this data is persistant across reattachments
     */
    switch (umas->protocol) {
    case UMAS_PR_CB:
        umas->transport_name = "Control/Bulk";
        umas->transport = UMAS_CbTransport;
        umas->transport_reset = UMAS_CbReset;
        umas->max_lun = 7;
        break;

    case UMAS_PR_CBI:
        umas->transport_name = "Control/Bulk/Interrupt";
        umas->transport = UMAS_CbiTransport;
        umas->transport_reset = UMAS_CbReset;
        umas->max_lun = 7;
        break;

    case UMAS_PR_BULK:
        umas->transport_name = "Bulk-only";
        umas->transport = UMAS_BulkTransport;
        umas->transport_reset = UMAS_BulkReset;
        umas->max_lun = UMAS_BulkMaxLun(umas);
        UMAS_DEBUG("Bulk max logical unit number: %d\n", umas->max_lun);
        break;

    default:
        umas->transport_name = "Unknown";
        goto err_ret;
    } /* end of switch */

    UMAS_DEBUG("Mass storage transport: %s\n", umas->transport_name);

    /* fix for single-lun devices */
    if (umas->flags & UMAS_FL_SINGLE_LUN)
        umas->max_lun = 0;

    switch (umas->subclass) {
    case UMAS_SC_RBC:
        umas->protocol_name = "Reduced Block Commands (RBC)";
        umas->proto_handler = UMAS_TransparentScsiCommand;
        break;

    case UMAS_SC_8020:
        umas->protocol_name = "8020i";
        umas->proto_handler = UMAS_AtapiCommand;
        umas->max_lun = 0;
        break;

    case UMAS_SC_QIC:
        umas->protocol_name = "QIC-157";
        umas->proto_handler = UMAS_Qic157Command;
        umas->max_lun = 0;
        break;

    case UMAS_SC_8070:
        umas->protocol_name = "8070i";
        umas->proto_handler = UMAS_AtapiCommand;
        umas->max_lun = 0;
        break;

    case UMAS_SC_SCSI:
        umas->protocol_name = "Transparent SCSI";
        umas->proto_handler = UMAS_TransparentScsiCommand;
        break;

    case UMAS_SC_UFI:
        umas->protocol_name = "Uniform Floppy Interface (UFI)";
        umas->proto_handler = UMAS_UfiCommand;
        break;

    default:
        umas->protocol_name = "Unknown";
        goto err_ret;
    } /* end of switch */

    UMAS_DEBUG("Mass storage protocol: %s\n", umas->protocol_name);

    /* allocate an IRQ callback if one is needed */
    if ((umas->protocol == UMAS_PR_CBI) && usb_stor_allocate_irq(umas)) {
        return USB_ERR_IO;
    }

    UMAS_DEBUG("WARNING: USB Mass Storage data integrity not assured\n");
    UMAS_DEBUG("USB Mass Storage device found at %d\n", dev->devnum);

    if (UMAS_InitUmasDevice(umas) < 0)
        goto err_ret;

    return 0;

err_ret:
    if (umas->current_urb) {
        USBH_UnlinkUrb(umas->current_urb);
        USBH_FreeUrb(umas->current_urb);
    }
    free_umas(umas);
    return USB_ERR_IO;
}



/* Handle a disconnect event from the USB core */
static void storage_disconnect(USB_DEV_T *dev)
{
    UMAS_DATA_T     *umas = NULL;
#ifdef USE_NVTFAT
    UMAS_DRIVE_T    *umas_drive, *next_drv;
#endif
    int             i;

    UMAS_DEBUG("storage_disconnect called\n");

    for (i = 0; i < UMAS_MAX_DEV; i++) {
        if ((_umac_alloc_mark[i]) && (_umas_pool[i].pusb_dev == dev))
            umas = &_umas_pool[i];
    }

    /* this is the odd case -- we disconnected but weren't using it */
    if (!umas) {
        UMAS_DEBUG("storage_disconnect - device not in use\n");
        return;
    }

#ifdef USE_NVTFAT
    /* Unmount disk */
    umas_drive = umas->drive_list;
    while (umas_drive != NULL) {
        next_drv = umas_drive->next;
        fsPhysicalDiskDisconnected(umas_drive->client);
        free_umas_drive(umas_drive);
        umas_drive = next_drv;
    }
#endif

    if (umas->irq_urb) {
        UMAS_DEBUG("storage_disconnect -- releasing irq URB\n");
#ifdef DEBUG
        UMAS_DEBUG("storage_disconnect -- usb_unlink_urb() returned %d\n", USBH_UnlinkUrb(umas->irq_urb));
#else
        USBH_UnlinkUrb(umas->irq_urb);
#endif
        USBH_FreeUrb(umas->irq_urb);
        umas->irq_urb = NULL;
    }

    /* free up the main URB for this device */
    UMAS_DEBUG("storage_disconnect -- releasing main URB\n");
#ifdef DEBUG
    UMAS_DEBUG("storage_disconnect -- usb_unlink_urb() returned %d\n", USBH_UnlinkUrb(umas->current_urb));
#else
    USBH_UnlinkUrb(umas->current_urb);
#endif
    USBH_FreeUrb(umas->current_urb);
    umas->current_urb = NULL;

    free_umas(umas);
}


#if 0
void UMAS_ScanAllDevice()
{
    int     i;
    for (i = 0; i < UMAS_MAX_DEV; i++) {
        if (_umac_alloc_mark[i] == 0)
            continue;

        UMAS_ScanDeviceLun(&_umas_pool[i]);
    }
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
  * @brief    Initialize USB Host Mass Storage driver
  * @return   Success or not.
  * @retval   0   Success
  * @retval   Otherwise  Failed
  */
int  USBH_MassInit(void)
{
    int     i = 0;

    g_umas = NULL;

    for (i = 0; i < UMAS_MAX_DEV; i++)
        _umac_alloc_mark[i] = 0;
    /* register the driver, return -1 if error */
    if (USBH_RegisterDriver(&_UsbMassStorageDriver) < 0)
        return -1;
    return 0;
}


/**
  * @brief    Obtain the list of currently connected USB Mass Storage disk
  * @param[out] dlist  An array of disk pointer.
  * @param[in]  max    Maximum avalable entries of dlist.
  * @return   Number of disk found.
  * @retval   0   No disk found
  * @retval   Otherwise  Number of disk
  */
int32_t  USBH_MassGetDiskList(mass_disk_t * dlist[], int max)
{
    int  i, idx;

    for (i = 0, idx = 0; (i < UMAS_MAX_DEV) && (idx < max); i++) {
        if (_umac_alloc_mark[i]) {
            dlist[idx] = (mass_disk_t *)&_umas_pool[i];
            idx++;
        }
    }
    return idx;
}

/*@}*/ /* end of group NUC472_442_USBH_MASS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC472_442_USBH_MASS_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


