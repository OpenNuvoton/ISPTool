#ifndef __INCLUDED_HID_H__
#define __INCLUDED_HID_H__

#include "usbh_core.h"

/** @addtogroup NUC472_442_Device_Driver NUC472/NUC442 Device Driver
  @{
*/

/** @addtogroup NUC472_442_USBH_HID_Driver USB Host HID Driver
  @{
*/


/// @cond HIDDEN_SYMBOLS
// #define HID_DEBUG

/*
 * Debug message
 */
#ifdef HID_DEBUG
#define HID_DBGMSG      printf
#else
#define HID_DBGMSG(...)
#endif

/// @endcond


/** @addtogroup NUC472_442_USBH_HID_EXPORTED_CONSTANTS USB Host HID Driver Exported Constants
  @{
*/


#define CONFIG_HID_MAX_DEV            3     /*!< Maximum number of HID device.  \hideinitializer */
#define HID_MAX_BUFFER_SIZE           64    /*!< HID interrupt in transfer buffer size   \hideinitializer */
#define HID_CLIENT_SIZE               64    /*!< Maximum size of HID device client data area   \hideinitializer */

/*
 * Constants
 */
#define HID_RET_OK                      0   /*!< Return with no errors.  \hideinitializer */
#define HID_RET_DEV_NOT_FOUND          -9   /*!< HID device not found or removed.  \hideinitializer */
#define HID_RET_IO_ERR                -11   /*!< USB transfer failed.  \hideinitializer */
#define HID_RET_INVALID_PARAMETER     -13   /*!< Invalid parameter.  \hideinitializer */
#define HID_RET_OUT_OF_MEMORY         -15   /*!< Out of memory.  \hideinitializer */
#define HID_RET_NOT_SUPPORTED         -17   /*!< Function not supported.  \hideinitializer */


#define HID_REPORT_GET                0x01  /*!< Get_Report_Request code.  \hideinitializer */
#define HID_GET_IDLE                  0x02  /*!< Get_Idle code.  \hideinitializer */
#define HID_GET_PROTOCOL              0x03  /*!< Get_Protocol code.  \hideinitializer */
#define HID_REPORT_SET                0x09  /*!< Set_Report_Request code.  \hideinitializer */
#define HID_SET_IDLE                  0x0A  /*!< Set_Idle code.  \hideinitializer */
#define HID_SET_PROTOCOL              0x0B  /*!< Set_Protocol code.  \hideinitializer */


/*
 * Report type
 */
#define RT_INPUT          1      /*!< Report type: Input    \hideinitializer */
#define RT_OUTPUT         2      /*!< Report type: Output   \hideinitializer */
#define RT_FEATURE        3      /*!< Report type: Feature  \hideinitializer */


/*@}*/ /* end of group NUC472_442_USBH_HID_EXPORTED_CONSTANTS */


struct usbhid_dev;               /*!< HID device structure  \hideinitializer             */


/** @addtogroup NUC472_442_USBH_HID_EXPORTED_TYPEDEFS USB Host HID Driver Exported Type Define
  @{
*/

typedef void (HID_IR_FUNC)(struct usbhid_dev *hdev, uint8_t *rdata, int data_len);      /*!< interrupt in callback function \hideinitializer */
typedef void (HID_IW_FUNC)(struct usbhid_dev *hdev, uint8_t **wbuff, int *buff_size);   /*!< interrupt out callback function \hideinitializer */


/*@}*/ /* end of group NUC472_442_USBH_HID_EXPORTED_TYPEDEFS */


/** @addtogroup NUC472_442_USBH_HID_EXPORTED_STRUCTURES USB Host HID Driver Exported Structures
  @{
*/

/*-----------------------------------------------------------------------------------
 *  HID device
 */
/*! HID device structure \hideinitializer             */
typedef struct usbhid_dev {
    USB_DEV_T           *udev;          /*!< USB device pointer of HID_DEV_T \hideinitializer  */
    int                 ifnum;          /*!< Interface numder \hideinitializer                 */
    uint8_t             bSubClassCode;  /*!< Interface subclass code \hideinitializer          */
    uint8_t             bProtocolCode;  /*!< Interface protocol code \hideinitializer          */
    URB_T               *urbin;         /*!< Input URB \hideinitializer                        */
    URB_T               *urbout;        /*!< Output URB \hideinitializer                       */
    uint8_t             inbuf[HID_MAX_BUFFER_SIZE];  /*!< Input buffer \hideinitializer        */
    HID_IR_FUNC         *read_func;     /*!< Interrupt-in callback function \hideinitializer   */
    HID_IW_FUNC         *write_func;    /*!< Interrupt-out callback function \hideinitializer  */
    struct usbhid_dev   *next;          /*!< Point to the next HID device \hideinitializer     */
    uint8_t             client[HID_CLIENT_SIZE]; /*!< HID device client data area   \hideinitializer */
} HID_DEV_T;                            /*! HID device structure \hideinitializer             */


/*@}*/ /* end of group NUC472_442_USBH_HID_EXPORTED_STRUCTURES */


#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup NUC472_442_USBH_HID_EXPORTED_FUNCTIONS USB Host HID Driver Exported Functions
  @{
*/

void     USBH_HidInit(void);
HID_DEV_T * USBH_HidGetDeviceList(void);
int32_t  HID_HidGetReportDescriptor(HID_DEV_T *hdev, uint8_t *desc_buf, int buf_max_len);
int32_t  HID_HidGetReport(HID_DEV_T *hdev, int rtp_typ, int rtp_id, uint8_t *data, int len);
int32_t  HID_HidSetReport(HID_DEV_T *hdev, int rtp_typ, int rtp_id, uint8_t *data, int len);
int32_t  HID_HidGetIdle(HID_DEV_T *hdev, int rtp_id, uint8_t *idle_rate);
int32_t  HID_HidSetIdle(HID_DEV_T *hdev, int rtp_id, uint8_t idle_rate);
int32_t  HID_HidGetProtocol(HID_DEV_T *hdev, uint8_t *protocol);
int32_t  HID_HidSetProtocol(HID_DEV_T *hdev, uint8_t protocol);
int32_t  USBH_HidStartIntReadPipe(HID_DEV_T *hdev, HID_IR_FUNC *func);
int32_t  USBH_HidStartIntWritePipe(HID_DEV_T *hdev, HID_IW_FUNC *func);


/*@}*/ /* end of group NUC472_442_USBH_HID_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC472_442_USBH_HID_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */


#ifdef __cplusplus
}
#endif

#endif /* __INCLUDED_HID_H__ */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

