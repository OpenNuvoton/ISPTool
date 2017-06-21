/**************************************************************************//**
 * @file     usbh_config.h
 * @version  V1.00
 * $Revision 2 $
 * $Date: 15/09/21 9:15a $
 * @brief    USB Host core configuration file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef  _USB_CONFIG_H_
#define  _USB_CONFIG_H_


/** @addtogroup NUC472_442_Device_Driver NUC472/NUC442 Device Driver
  @{
*/

/** @addtogroup NUC472_442_USBH_Driver USBH Driver
  @{
*/

/** @addtogroup NUC472_442_USBH_EXPORTED_CONSTANTS USBH Exported Constants
  @{
*/


/// @cond HIDDEN_SYMBOLS

#ifdef __ICCARM__
#define __inline    inline
#endif


/*
 *  Debug messages...
 */
//#define USB_DEBUG                       /*!< Enable debug message \hideinitializer      */
//#define USB_VERBOSE_DEBUG
//#define DUMP_DEV_DESCRIPTORS

/// @endcond HIDDEN_SYMBOLS

/*
 *  Static Memory Settings...
 */
#define DEV_MAX_NUM             8       /*!< Maximum number of connected devices \hideinitializer       */
#define URB_MAX_NUM             12      /*!< Maximum number of URBs in memory pool \hideinitializer     */
#define ED_MAX_NUM              12      /*!< Maximum number of OHCI EDs in memory pool \hideinitializer */
#define TD_MAX_NUM              64      /*!< Maximum number of OHCI TDs in memory pool \hideinitializer */

#define MAX_ENDPOINTS           16      /*!< Maximum number of endpoints per device \hideinitializer    */
#define MAX_DRIVER_PER_DEV      3       /*!< Maximum number of drivers for a device \hideinitializer    */
#define MAX_TD_PER_OHCI_URB     8       /*!< Maximum number of OHCI TDs per URB \hideinitializer        */
#define MAX_HUB_DEVICE          2       /*!< Maximum number of connected Hub devices \hideinitializer   */


#define ISO_FRAME_COUNT         1       /*!< Transfer frames per Isohronous TD. (Must be 1 for isochronous out.) \hideinitializer     */
#define OHCI_ISO_DELAY          8       /*!< Delay isochronous transfer frame time \hideinitializer     */

/*
 * Class driver support...
 */
#define SUPPORT_HUB_CLASS               /*!< Support Hub driver \hideinitializer      */


/// @cond HIDDEN_SYMBOLS

/*
 *  Debug/Warning/Information to be printed on console or not
 */
#define USB_error               printf
#ifdef USB_DEBUG
#define USB_debug               printf
#else
#define USB_debug(...)
#endif

#ifdef USB_VERBOSE_DEBUG
#define USB_warning             printf
#define USB_info                printf
#else
#define USB_warning(...)
#define USB_info(...)
#endif

#define DISABLE_USB_INT()       NVIC_DisableIRQ(USBH_IRQn);
#define ENABLE_USB_INT()        NVIC_EnableIRQ(USBH_IRQn);


/*
 * I/O
 */
#define OHCI_BASE_ADDR          0x40009000

#define USB_JIFFY               (OHCI->FMNUM & 0xffff)


/*---  CPU clock speed ---*/
#define HZ                      (84)

#define USB_SWAP16(x)           (((x>>8)&0xff)|((x&0xff)<<8))
#define USB_SWAP32(x)           (((x>>24)&0xff)|((x>>8)&0xff00)|((x&0xff00)<<8)|((x&0xff)<<24))

/// @endcond HIDDEN_SYMBOLS


/*@}*/ /* end of group NUC472_442_USBH_EXPORTED_CONSTANTS USBH Exported Constants */

/*@}*/ /* end of group NUC472_442_USBH_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

#endif  /* _USB_CONFIG_H_ */


