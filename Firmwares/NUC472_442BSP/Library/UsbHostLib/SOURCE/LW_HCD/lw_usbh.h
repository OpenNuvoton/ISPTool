
#ifndef _LW_USBH_H_
#define _LW_USBH_H_

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup NUC472_442_Device_Driver NUC472/NUC442 Device Driver
  @{
*/

/** @addtogroup NUC472_442_LWHCD_Driver LW_USBH Driver
  @{
*/


/** @addtogroup NUC472_442_LWHCD_EXPORTED_CONSTANTS LW_USBH Exported Constants
  @{
*/

#define USBH_INTR_BUFF_SIZE      256                /*!< USB driver internal buffer size. \hideinitializer       */

/// @cond HIDDEN_SYMBOLS

extern uint8_t  _transfer_buffer[USBH_INTR_BUFF_SIZE];

/// @endcond



/*---------------------------------------------------------------------------------------------------------*/
/* Return Code                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define USBH_RET_NO_ERR          0                  /*!< No errors. \hideinitializer                             */
#define USBH_RET_ERR_PARM        -1                 /*!< Wrong parameter \hideinitializer                        */
#define USBH_RET_INIT            -2                 /*!< USB Host init failed \hideinitializer                   */
#define USBH_RET_NO_DEVICE       -11                /*!< No device connected \hideinitializer                    */
#define USBH_RET_DEV_CONN_KEEP   -12                /*!< A connected device is still connected.\hideinitializer  */
#define USBH_RET_DEV_REMOVED     -13                /*!< A connected device was disconnected. \hideinitializer   */
#define USBH_RET_XFER_TIMEOUT    -21                /*!< USB transfer time-out \hideinitializer                  */
#define USBH_RET_STALL           -22                /*!< Devie STALL \hideinitializer                            */
#define USBH_RET_XFER_ERR        -23                /*!< USB transfer error occurred \hideinitializer            */
#define USBH_RET_UNSUPPORT       -31                /*!< Not supported USB device \hideinitializer               */
#define USBH_RET_DEV_NOT_READY   -35                /*!< Device is not ready \hideinitializer                    */
#define USBH_RET_ERR_CLASS_CMD   -41                /*!< Class specific command failed \hideinitializer          */
#define USBH_RET_ERR_DEV_INIT    -42                /*!< Device initialization failed \hideinitializer           */
#define USBH_RET_ERR_PORT_RST    -101               /*!< Port reset failed. \hideinitializer                     */
#define USBH_RET_ERR_PORT_ENABLE -102               /*!< Port enable failed. \hideinitializer                    */

/*@}*/ /* end of group NUC472_442_LWHCD_EXPORTED_CONSTANTS */


/** @addtogroup NUC472_442_LWHCD_EXPORTED_FUNCTIONS LW_USBH Exported Functions
  @{
*/
int usbh_init(void);
int usbh_probe_port(uint32_t port);
int usbh_get_device_descriptor(uint8_t *desc_buff);
int get_config_descriptor(uint8_t *desc_buff);
int usbh_set_configuration(int conf_val);
int usbh_clear_halt(uint16_t ep_addr);
int usbh_drv_ctrl_req(uint8_t  requesttype, uint8_t  request, uint16_t value, uint16_t index, uint16_t length, int data_len, uint8_t *buffer, int dir);
int usbh_drv_bulk_xfer(uint16_t ep_addr, uint8_t *toggle, uint8_t *data_buff, int data_len, int timeout);


/*@}*/ /* end of group NUC472_442_LWHCD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC472_442_LWHCD_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/



#endif  /* _LW_USBH_H_ */
