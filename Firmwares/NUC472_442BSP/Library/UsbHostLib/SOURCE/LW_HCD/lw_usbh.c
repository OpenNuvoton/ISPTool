/**************************************************************************//**
 * @file     lw_usbh.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 14/10/07 5:47p $
 * @brief    Light-Weight USB Host driver
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "NUC472_442.h"
#include "usb.h"
#include "lw_usbh.h"


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

#define usb_dbg_msg         printf
//#define usb_dbg_msg(...)

#define usb_desc_dump       printf
//#define usb_desc_dump(...)

static int  DEV_ADDR;

int  is_low_speed;

#ifdef __ICCARM__
#pragma data_alignment=256
struct ohci_hcca _OHCI_HCCA;
#endif

#ifdef __ICCARM__
#pragma data_alignment=16
ED_T  _ed;
#endif

#ifdef __ICCARM__
#pragma data_alignment=16
TD_T  _td[8];
#endif

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t  _transfer_buffer[USBH_INTR_BUFF_SIZE];
#endif

#ifdef __ARMCC_VERSION
__align(256) struct ohci_hcca _OHCI_HCCA;
__align(16) ED_T  _ed;
__align(16) TD_T  _td[8];
__align(4)  uint8_t  _transfer_buffer[USBH_INTR_BUFF_SIZE];
#endif


DEV_REQ_T   _request;
int         _last_td_idx;


static int get_ms_elapsed(int jiffy)
{
    uint32_t  fnum = USBH->HcFmNumber & 0xffff;

    if (fnum >= jiffy)
        return fnum - jiffy;
    else
        return (0xffff - jiffy) + fnum;
}


static void  dump_device_descriptor(USB_DEV_DESC_T *desc)
{
    usb_desc_dump("\n[Device Descriptor]\n");
    usb_desc_dump("----------------------------------------------\n");
    usb_desc_dump("  Length              = %2d\n",  desc->bLength);
    usb_desc_dump("  DescriptorType      = %02x\n", desc->bDescriptorType);
    usb_desc_dump("  USB version         = %x.%02x\n",
                  desc->bcdUSB >> 8, desc->bcdUSB & 0xff);
    usb_desc_dump("  Vendor:Product      = %04x:%04x\n",
                  desc->idVendor, desc->idProduct);
    usb_desc_dump("  MaxPacketSize0      = %d\n",   desc->bMaxPacketSize0);
    usb_desc_dump("  NumConfigurations   = %d\n",   desc->bNumConfigurations);
    usb_desc_dump("  Device version      = %x.%02x\n",
                  desc->bcdDevice >> 8, desc->bcdDevice & 0xff);
    usb_desc_dump("  Device Class:SubClass:Protocol = %02x:%02x:%02x\n",
                  desc->bDeviceClass, desc->bDeviceSubClass, desc->bDeviceProtocol);
}

static void  dump_config_descriptor(USB_CONFIG_DESC_T *desc)
{
    uint8_t   *bptr = (uint8_t *)desc;
    int       tlen = desc->wTotalLength;
    USB_IF_DESC_T  *if_desc;
    USB_EP_DESC_T  *ep_desc;

    while (tlen > 0) {
        switch (bptr[1]) {
        case USB_DT_CONFIG:
            usb_desc_dump("\n[Configuration Descriptor]\n");
            usb_desc_dump("----------------------------------------------\n");
            usb_desc_dump("  Length              = %2d\n",  desc->bLength);
            usb_desc_dump("  DescriptorType      = %02x\n", desc->bDescriptorType);
            usb_desc_dump("  wTotalLength        = %2d\n", desc->wTotalLength);
            usb_desc_dump("  bNumInterfaces      = %d\n", desc->bNumInterfaces);
            usb_desc_dump("  bConfigurationValue = %d\n", desc->bConfigurationValue);
            usb_desc_dump("  iConfiguration      = %d\n", desc->iConfiguration);
            usb_desc_dump("  bmAttributes        = 0x%02x\n", desc->bmAttributes);
            usb_desc_dump("  MaxPower            = %d\n", desc->MaxPower);
            break;

        case USB_DT_INTERFACE:
            if_desc = (USB_IF_DESC_T *)bptr;
            usb_desc_dump("\n[Interface Descriptor]\n");
            usb_desc_dump("----------------------------------------------\n");
            usb_desc_dump("  Length              = %2d\n",  if_desc->bLength);
            usb_desc_dump("  DescriptorType      = %02x\n", if_desc->bDescriptorType);
            usb_desc_dump("  bInterfaceNumber    = %d\n", if_desc->bInterfaceNumber);
            usb_desc_dump("  bAlternateSetting   = %d\n", if_desc->bAlternateSetting);
            usb_desc_dump("  bNumEndpoints       = %d\n", if_desc->bNumEndpoints);
            usb_desc_dump("  bInterfaceClass     = 0x%02x\n", if_desc->bInterfaceClass);
            usb_desc_dump("  bInterfaceSubClass  = 0x%02x\n", if_desc->bInterfaceSubClass);
            usb_desc_dump("  bInterfaceProtocol  = 0x%02x\n", if_desc->bInterfaceProtocol);
            usb_desc_dump("  iInterface          = %d\n", if_desc->iInterface);
            break;

        case USB_DT_ENDPOINT:
            ep_desc = (USB_EP_DESC_T *)bptr;
            usb_desc_dump("\n[Endoint Descriptor]\n");
            usb_desc_dump("----------------------------------------------\n");
            usb_desc_dump("  Length              = %2d\n",  ep_desc->bLength);
            usb_desc_dump("  DescriptorType      = %02x\n", ep_desc->bDescriptorType);
            usb_desc_dump("  bEndpointAddress    = 0x%02x\n", ep_desc->bEndpointAddress);
            usb_desc_dump("  bmAttributes        = 0x%02x\n", ep_desc->bmAttributes);
            usb_desc_dump("  wMaxPacketSize      = %d\n", ep_desc->wMaxPacketSize);
            usb_desc_dump("  bInterval           = %d\n", ep_desc->bInterval);
            usb_desc_dump("  bRefresh            = %d\n", ep_desc->bRefresh);
            usb_desc_dump("  bSynchAddress       = %d\n", ep_desc->bSynchAddress);
            break;
        }
        if (bptr[0] == 0)
            break;
        tlen -= bptr[0];
        bptr += bptr[0];
    }
}

static void td_fill(int td_idx, uint32_t info, uint8_t *buff, uint32_t data_len)
{
    TD_T  *td = &_td[td_idx];

    td->hwINFO = info;
    td->hwCBP = (uint32_t)((!buff || !data_len) ? 0 : buff);
    td->hwBE = (uint32_t)((!buff || !data_len ) ? 0 : (uint32_t)buff + data_len - 1);
    td->hwNextTD = (uint32_t)&_td[td_idx+1];

    //usb_dbg_msg("TD [0x%x]: 0x%x, 0x%x, 0x%x, 0x%x\n", (int)td, td->hwINFO, td->hwCBP, td->hwBE, td->hwNextTD);
}


static void fill_ctrl_xfer_descriptors(int dev_addr, uint8_t *data_buff, int data_len, int pipe_out)
{
    uint32_t    info;
    int     td_idx = 0;

    /*------------------------------------------------------------------------------------*/
    /* prepare SETUP stage TD                                                             */
    /*------------------------------------------------------------------------------------*/
    info = TD_CC | TD_DP_SETUP | TD_T_DATA0;
    td_fill(td_idx, info, (uint8_t *)&_request, 8);
    td_idx++;

    /*------------------------------------------------------------------------------------*/
    /* prepare DATA stage TD                                                              */
    /*------------------------------------------------------------------------------------*/
    if (data_len > 0) {
        if (pipe_out)
            info = (TD_CC | TD_R | TD_DP_OUT | TD_T_DATA1);
        else
            info = (TD_CC | TD_R | TD_DP_IN | TD_T_DATA1);

        td_fill(td_idx, info, data_buff, data_len);
        td_idx++;
    }

    /*------------------------------------------------------------------------------------*/
    /* prepare STATUS stage TD                                                            */
    /*------------------------------------------------------------------------------------*/
    if (pipe_out)
        info = (TD_CC | TD_DP_IN | TD_T_DATA1);
    else
        info = (TD_CC | TD_DP_OUT | TD_T_DATA1);

    td_fill(td_idx, info, NULL, 0);
    _last_td_idx = td_idx;

    td_idx++;

    /* dummy TD */
    td_fill(td_idx, 0x01, NULL, 0);
    _td[td_idx].hwNextTD = 0;;

    /*------------------------------------------------------------------------------------*/
    /* prepare ED                                                                         */
    /*------------------------------------------------------------------------------------*/
    _ed.hwTailP = (uint32_t)&_td[td_idx];
    _ed.hwHeadP = (uint32_t)&_td[0];
    _ed.hwINFO  = ((is_low_speed ? 8 : 64) << 16)   /* Maximum Packet Size                    */
                  | (0 << 15)                 /* Format                                 */
                  | (0 << 14)                 /* Skip                                   */
                  | (is_low_speed << 13)      /* Speed: full speed                      */
                  | (0 << 11)                 /* Direction                              */
                  | (0 << 7)                  /* Endpoint Number                        */
                  | (dev_addr);               /* Function Address                       */
    _ed.hwNextED = 0;
}


static void fill_bulk_xfer_descriptors(uint16_t ep_addr, uint8_t *toggle,
                                       uint8_t *data_buff, int data_len, int pipe_out)
{
    uint32_t   info;
    int        xfer_len;
    int        td_idx = 0;

    do {
        if (pipe_out)
            info = (TD_CC | TD_R | TD_DP_OUT);
        else
            info = (TD_CC | TD_R | TD_DP_IN);

        info &= ~ (1 << 25);       /* let OHCI hardware do the data toggle */

        if (data_len > 4096)
            xfer_len = 4096;
        else
            xfer_len = data_len;

        td_fill(td_idx, info, data_buff, xfer_len);
        _last_td_idx = td_idx;
        td_idx++;
        data_buff += xfer_len;
        data_len -= xfer_len;
    }   while (data_len > 0);

    /* dummy TD */
    td_fill(td_idx, 0x01, NULL, 0);
    _td[td_idx].hwNextTD = 0;

    /*------------------------------------------------------------------------------------*/
    /* prepare ED                                                                         */
    /*------------------------------------------------------------------------------------*/
    _ed.hwTailP = (uint32_t)&_td[td_idx];
    _ed.hwHeadP = (uint32_t)&_td[0] | (*toggle << 1);
    _ed.hwINFO  = ((is_low_speed ? 8 : 64) << 16)    /* Maximum Packet Size               */
                  | (0 << 15)                 /* Format                                 */
                  | (0 << 14)                 /* Skip                                   */
                  | (is_low_speed << 13)      /* Speed: full speed                      */
                  | (0 << 11)                 /* Direction                              */
                  | ((ep_addr & 0xf) << 7)    /* Enpoint Number                         */
                  | (DEV_ADDR);               /* Function Address                       */
    _ed.hwNextED = 0;
}

/// @endcond


/**
 *  @brief  Execute a control transfer request.
 *  @param[in] requesttype  USB message request type value
 *  @param[in] request      USB message request value
 *  @param[in] value        USB message value
 *  @param[in] index        USB message index value
 *  @param[in] length       USB message length value
 *  @param[in] data_len     length in bytes of the data to send/receive
 *  @param[in,out] buffer   Pointer to buffer hold the data to send and to receive.
 *  @param[in] dir          Data transfer direction. 0 - in; 1 - out.
 *  @return   Success or not.
 *  @retval   0  Success
 *  @retval   Otherwise  Failed
 *
 *  This function sends a simple control message to a specified endpoint
 *  and waits for the message to complete, or timeout.
 *
 *  If successful, it returns 0, otherwise a negative error number.
 */
int usbh_drv_ctrl_req(uint8_t  requesttype,
                      uint8_t  request,
                      uint16_t value,
                      uint16_t index,
                      uint16_t length,
                      int      data_len,
                      uint8_t  *buffer,
                      int      dir)
{
    int  t0, i, ret = USBH_RET_NO_ERR;

    /*------------------------------------------------------------------------------------*/
    /* Fill the SETUP packet for SET ADDRESS command                                      */
    /*------------------------------------------------------------------------------------*/
    _request.requesttype  = requesttype;
    _request.request      = request;
    _request.value        = value;
    _request.index        = index;
    _request.length       = length;

    USBH->HcInterruptStatus = USBH->HcInterruptStatus;                 /* clear status   */
    fill_ctrl_xfer_descriptors((request == USB_REQ_SET_ADDRESS) ? 0 : DEV_ADDR, buffer, data_len, dir);

    /*------------------------------------------------------------------------------------*/
    /* Do transfer                                                                        */
    /*------------------------------------------------------------------------------------*/
    USBH->HcControlHeadED = (uint32_t)&_ed;            /* Link ED to OHCI                        */
    USBH->HcControl |= USBH_HcControl_CLE_Msk;
    USBH->HcCommandStatus = OHCI_CLF;                   /* start Control list                     */

    t0 = USBH->HcFmNumber & 0xffff;
    while (get_ms_elapsed(t0) < 1000) {
        if ((USBH->HcInterruptStatus & USBH_HcInterruptStatus_WDH_Msk) &&
                ((_td[_last_td_idx].hwINFO & 0xf0000000) != 0xf0000000))
            break;
    }

    USBH->HcInterruptStatus = USBH_HcInterruptStatus_WDH_Msk;

    if (get_ms_elapsed(t0) >= 1000) {
        usb_dbg_msg("Control transfer failed!\n");
        ret = USBH_RET_XFER_TIMEOUT;
        goto ctrl_out;
    }

    for (i = 0; i <= _last_td_idx; i++) {
        if (_td[i].hwINFO & 0xf0000000) {
            usb_dbg_msg("USB xfer error, CC=0x%x [0x%x]\n", (_td[i].hwINFO >> 28), (int)&_td[i]);

            if (ret == USBH_RET_STALL)
                continue;

            if ((_td[i].hwINFO >> 28) == 0x4)
                ret = USBH_RET_STALL;
            else
                ret = USBH_RET_XFER_ERR;
        }
    }

ctrl_out:

    USBH->HcControl &= ~USBH_HcControl_CLE_Msk;
    USBH->HcControlHeadED = 0;
    USBH->HcDoneHead = 0;
    _OHCI_HCCA.done_head = 0;
    return ret;
}


/**
 *  @brief  Execute a control transfer request.
 *  @param[in] ep_addr Endpoint address.
 *  @param[in,out] toggle Set the last data packet toggle setting and get new data packet toggle setting
 *  @param[in,out] data_buff  Buffer hold the data to send and the data receive.
 *  @param[in] data_len  Length in bytes of the data to send/receive
 *  @param[in] timeout   Transfer time-out setting in millisecond.
 *  @return   Success or not.
 *  @retval   0  Success
 *  @retval   Otherwise  Failed
 */
int usbh_drv_bulk_xfer(uint16_t ep_addr, uint8_t *toggle,
                       uint8_t *data_buff, int data_len, int timeout)
{
    int  t0, i, ret = USBH_RET_NO_ERR;

    fill_bulk_xfer_descriptors(ep_addr, toggle, data_buff, data_len, (ep_addr & 0x80) ? 0: 1);

    /*------------------------------------------------------------------------------------*/
    /* Do transfer                                                                        */
    /*------------------------------------------------------------------------------------*/
    USBH->HcBulkHeadED = (uint32_t)&_ed;            /* Link ED to OHCI                        */
    USBH->HcControl |= USBH_HcControl_BLE_Msk;
    USBH->HcCommandStatus = OHCI_BLF;                   /* start Control list                     */

    t0 = USBH->HcFmNumber & 0xffff;
    while (get_ms_elapsed(t0) < timeout) {
        if ((USBH->HcInterruptStatus & USBH_HcInterruptStatus_WDH_Msk) &&
                ((_td[_last_td_idx].hwINFO & 0xf0000000) != 0xf0000000))
            break;
    }
    //printf("0x%x, 0x%x\n", _OHCI_HCCA.done_head, &_td[_last_td_idx]);
    USBH->HcInterruptStatus = USBH_HcInterruptStatus_WDH_Msk;

    for (i = 0; i <= _last_td_idx; i++) {
        if (_td[i].hwINFO & 0xf0000000) {
            usb_dbg_msg("USB xfer error, CC=0x%x [0x%x]\n", (_td[i].hwINFO >> 28), (int)&_td[i]);

            if (ret == USBH_RET_STALL)
                continue;

            if ((_td[i].hwINFO >> 28) == 0x4)
                ret = USBH_RET_STALL;
            else
                ret = USBH_RET_XFER_ERR;
        }
    }

    if (get_ms_elapsed(t0) >= timeout) {
        usb_dbg_msg("Bulk transfer failed!\n");
        ret = USBH_RET_XFER_TIMEOUT;
        goto bulk_exit;
    }

bulk_exit:

    *toggle = (_ed.hwHeadP >> 1) & 0x1;

    USBH->HcControl &= ~USBH_HcControl_BLE_Msk;
    USBH->HcBulkHeadED = 0;
    USBH->HcDoneHead = 0;
    _OHCI_HCCA.done_head = 0;

    return ret;
}


/**
 *  @brief  Get device descriptor from the USB device.
 *  @param[out] desc_buff  Data buffer to receive device descriptor data.
 *  @return   Success or not.
 *  @retval   0  Success
 *  @retval   Otherwise  Failed
 */
int usbh_get_device_descriptor(uint8_t *desc_buff)
{
    volatile int i;
    int  ret, len;

    /*------------------------------------------------------------------------------------*/
    /* Issue GET DESCRIPTOR command to get device descriptor                              */
    /*------------------------------------------------------------------------------------*/
    ret = usbh_drv_ctrl_req(USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8, 0, 64,
                            64 /* data len */, desc_buff /* buffer */, 0 /* dir in */);
    if (ret < 0)
        return ret;

    len = desc_buff[0];
    if (len < 0x12) len = 0x12;

    ret = usbh_drv_ctrl_req(USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_DEVICE << 8, 0, len,
                            len /* data len */, desc_buff /* buffer */, 0 /* dir in */);
    if (ret < 0)
        return ret;
    for (i = 0; i < MINISEC_10; i++);
    dump_device_descriptor((USB_DEV_DESC_T *)desc_buff);

    return 0;
}


/**
 *  @brief  Get configuration descriptor from the USB device.
 *  @param[out] desc_buff  Data buffer to receive configuration descriptor data.
 *  @return   Success or not.
 *  @retval   0  Success
 *  @retval   Otherwise  Failed
 */
int get_config_descriptor(uint8_t *desc_buff)
{
    int  ret, len;

    /*------------------------------------------------------------------------------------*/
    /* Issue GET DESCRIPTOR command to get device descriptor                              */
    /*------------------------------------------------------------------------------------*/

    ret = usbh_drv_ctrl_req(USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_CONFIG << 8, 0, 8,
                            8 /* data len */, desc_buff /* buffer */, 0 /* dir in */);
    if (ret < 0)
        return ret;

    len = ((USB_CONFIG_DESC_T*)desc_buff)->wTotalLength & 0x1ff;

    ret = usbh_drv_ctrl_req(USB_DIR_IN, USB_REQ_GET_DESCRIPTOR, USB_DT_CONFIG << 8, 0, len,
                            len /* data len */, desc_buff /* buffer */, 0 /* dir in */);
    if (ret < 0)
        return ret;

    dump_config_descriptor((USB_CONFIG_DESC_T *)desc_buff);

    return 0;
}


/**
 *  @brief  Issue a standard request SET_CONFIGURATION to USB device.
 *  @param[in] conf_val  The configuration number to be set.
 *  @return   Success or not.
 *  @retval   0  Success
 *  @retval   Otherwise  Failed
 */
int usbh_set_configuration(int conf_val)
{
    return  usbh_drv_ctrl_req(0, USB_REQ_SET_CONFIGURATION, conf_val, 0, 0,
                              0 /* data len */, NULL /* buffer */, 1 /* dir out */);
}


/**
 *  @brief  Issue a standard request SET_FEATURE to clear USB device endpoint halt state.
 *  @param[in] ep_addr  Endpoint to be clear halt.
 *  @return   Success or not.
 *  @retval   0  Success
 *  @retval   Otherwise  Failed
 */
int usbh_clear_halt(uint16_t ep_addr)
{
    usb_dbg_msg("Clear endpoint 0x%x halt.\n", ep_addr);
    return  usbh_drv_ctrl_req(0x2, 0x1, 0, ep_addr, 0, 0, NULL, 1);
}


/// @cond HIDDEN_SYMBOLS

static int do_port_reset(int port)
{
    int  i, retry;

    for (retry = 1; retry <= 3; retry++) {
        USBH->HcRhPortStatus[port] = USBH_HcRhPortStatus_PRS_Msk;
        for (i = 0; i < MINISEC_10*retry; i++) {
            if (USBH->HcRhPortStatus[port] & USBH_HcRhPortStatus_PES_Msk)
                break;
        }
        if (USBH->HcRhPortStatus[port] & USBH_HcRhPortStatus_PES_Msk)
            break;
    }
    if (retry > 3) {
        usb_dbg_msg("Port reset failed\n");
        return USBH_RET_ERR_PORT_RST;
    }

    usb_dbg_msg("Port reset OK.\n");
    USBH->HcRhPortStatus[port] = USBH_HcRhPortStatus_PRSC_Msk;  /* clear port reset status */
    return 0;
}

/// @endcond


/**
 *  @brief  Probe USB root-hub port connect/disconnect status. A newly connected device will be
 *          initialized in this function.
 *  @param[in] port  The root-hub port number, 0 for port 0 and 1 for port 1.
 *  @return   The current port connect/disconnect status.
 *  @retval   0     A new device connected and initialized successfully.
 *  @retval   USBH_RET_DEV_CONN_KEEP   USB device is still connected.
 *  @retval   USBH_RET_NO_DEVICE       There's no device connected.
 *  @retval   USBH_RET_DEV_REMOVED     A connected device had been removed.
 */
int usbh_probe_port(uint32_t port)
{
    uint32_t  port_staus;
    volatile int  i, retry;
    int       ret;

    if (port > 1)
        return USBH_RET_ERR_PARM;

    port_staus = USBH->HcRhPortStatus[port];
    USBH->HcRhPortStatus[port] = port_staus & 0xffff0000;       /* clear port chage status */

    if (!(port_staus & USBH_HcRhPortStatus_CSC_Msk)) {
        /*
         * No connect change status
         */
        if (port_staus & USBH_HcRhPortStatus_CCS_Msk)
            return USBH_RET_DEV_CONN_KEEP;
        else
            return USBH_RET_NO_DEVICE;
    }

    /*
     * connect status change
     */
    if (!(port_staus & USBH_HcRhPortStatus_CCS_Msk))
        return USBH_RET_DEV_REMOVED;

    /*
     * New device connected.
     */
    usb_dbg_msg("Device connected.\n");

    /*------------------------------------------------------------------------------------*/
    /* Doing port reset                                                                   */
    /*------------------------------------------------------------------------------------*/
    if (do_port_reset(port) < 0)
        return USBH_RET_ERR_PORT_RST;

    if ((USBH->HcRhPortStatus[port] & 0x03) == 0x03) {
        is_low_speed = (USBH->HcRhPortStatus[port] & USBH_HcRhPortStatus_LSDA_Msk) ? 1 : 0;
        usb_dbg_msg("USB device enabled (%s).\n", is_low_speed ? "Low-speed" : "Full-speed");
    } else {
        usb_dbg_msg("Failed to enable USB device!!\n");
        return USBH_RET_ERR_PORT_ENABLE;
    }

    for (i = 0; i < MINISEC_10; i++);

    DEV_ADDR = 1;

    /*------------------------------------------------------------------------------------*/
    /* Issue SET ADDRESS command                                                          */
    /*------------------------------------------------------------------------------------*/
    for (retry = 0; retry < 3; retry++) {
        ret =  usbh_drv_ctrl_req(0, USB_REQ_SET_ADDRESS, DEV_ADDR, 0, 0,
                                 0 /* data len */, NULL /* buffer */, 1 /* dir out */);
        if (ret == 0)
            break;

        do_port_reset(port);

        for (i = 0; i < MINISEC_100; i++);
    }
    if (ret < 0) {
        usb_dbg_msg("Failed to set device address!!\n");
        return ret;
    }

    ret = usbh_get_device_descriptor(_transfer_buffer);
    if (ret < 0) {
        usb_dbg_msg("GET Device Descriptor command failed!!\n");
        return ret;
    }
    return ret;
}

/// @cond HIDDEN_SYMBOLS

static int ohci_reset(void)
{
    volatile int  t0;

    /* Disable HC interrupts                                                              */
    USBH->HcInterruptDisable = OHCI_INTR_MIE;

    /* HC Reset requires max 10 ms delay                                                  */
    USBH->HcControl = 0;
    USBH->HcCommandStatus = OHCI_HCR;

    for (t0 = 0; t0 < MINISEC_10; t0++) {
        if ((USBH->HcCommandStatus & OHCI_HCR) == 0)
            break;
    }
    if (t0 >= MINISEC_10) {
        usb_dbg_msg("Error! - USB OHCI reset timed out!\n");
        return -1;
    }

    USBH->HcRhStatus = USBH_HcRhStatus_OCI_Msk | USBH_HcRhStatus_LPS_Msk;

    USBH->HcControl = OHCI_USB_RESET;

    for (t0 = 0; t0 < MINISEC_10; t0++) {
        if ((USBH->HcCommandStatus & OHCI_HCR) == 0)
            break;
    }
    if (t0 >= MINISEC_10) {
        usb_dbg_msg("Error! - USB HC reset timed out!\n");
        return -1;
    }
    return 0;
}


static int ohci_start()
{
    uint32_t    fminterval;
    volatile int    i;

    for (i = 0; i < NUM_INTS; i++)
        _OHCI_HCCA.int_table[i] = 0;

    /* Tell the controller where the control and bulk lists are
     * The lists are empty now. */
    USBH->HcControlHeadED = 0;                             /* control ED list head                   */
    USBH->HcBulkHeadED = 0;                             /* bulk ED list head                      */

    USBH->HcHCCA = (uint32_t)&_OHCI_HCCA;           /* HCCA area                              */

    /* periodic start 90% of frame interval                                                   */
    fminterval = 0x2edf;                            /* 11,999                                 */
    USBH->HcPeriodicStart = (fminterval * 9) / 10;

    /* set FSLargestDataPacket, 10,104 for 0x2edf frame interval                              */
    fminterval |= ((((fminterval - 210) * 6) / 7) << 16);
    USBH->HcFmInterval = fminterval;

    USBH->HcLSThreshold = 0x628;

    /* start controller operations                                                            */
    USBH->HcControl = 0x3 | OHCI_USB_OPER;

    /* Choose the interrupts we care about now, others later on demand                        */
    //mask = OHCI_INTR_MIE | OHCI_INTR_UE | OHCI_INTR_WDH;
    //USBH->HcIntEn = mask;
    //USBH->HcInterruptStatus = mask;

    /* NUC400 OCHI use no power switching mode                                            */
    USBH->HcRhDescriptorA = (USBH->HcRhDescriptorA | USBH_HcRhDescriptorA_NPS_Msk) & ~USBH_HcRhDescriptorA_PSM_Msk;
    USBH->HcRhStatus = USBH_HcRhStatus_LPSC_Msk;

    /* POTPGT delay is bits 24-31, in 20 ms units.                                         */
    for (i = 0; i < MINISEC_10 * 2; i++);

    return 0;
}

/// @endcond

/**
 *  @brief    Initialized USB host controller driver.
 *  @return   Success or not.
 *  @retval   0  Success
 *  @retval   Otherwise  Failed
 */
int usbh_init(void)
{
    volatile int loop;

    USBH->HcMiscControl &= ~0x8;        /* over current setting */

    if (ohci_reset() < 0) {
        usb_dbg_msg("OHCI reset failed!!\n");
        return USBH_RET_INIT;
    }

    if (ohci_start() < 0) {
        usb_dbg_msg("OHCI reset failed!!\n");
        return USBH_RET_INIT;
    }

    for (loop = 0; loop < 0x100000; loop++);

    return USBH_RET_NO_ERR;
}

/*@}*/ /* end of group NUC472_442_LWHCD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC472_442_LWHCD_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/



