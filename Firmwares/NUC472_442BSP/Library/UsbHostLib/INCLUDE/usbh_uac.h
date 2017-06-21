#ifndef __INCLUDED_USBHUAC_H__
#define __INCLUDED_USBHUAC_H__

#include "usbh_core.h"

/** @addtogroup NUC472_442_Device_Driver NUC472/NUC442 Device Driver
  @{
*/

/** @addtogroup NUC472_442_USBH_AS_Driver USB Host Audio Class Driver
  @{
*/

/** @addtogroup NUC472_442_USBH_AS_EXPORTED_CONSTANTS USB Host Audio Class Driver Exported Constants
  @{
*/


#define CONFIG_AU_MAX_DEV            1      /*!< Maximum number of Audio Class device.  \hideinitializer */
#define MAX_CFG_DESC_SIZE            256    /*!< The acceptable maximum size of Audio Class device configuration descriptor.  \hideinitializer */
#define MAX_FEATURE_UNIT             8      /*!< The acceptable maximum number of feature units.  \hideinitializer */
#define ISO_IN_URB_CNT               2      /*!< Number of USB transfer blocks used by audio in stream  \hideinitializer */
#define ISO_OUT_URB_CNT              2      /*!< Number of USB transfer blocks used by audio out stream  \hideinitializer */
#define AU_IN_MAX_PKTSZ              128    /*!< Audio in maximum packet size supported   \hideinitializer */
#define AU_OUT_MAX_PKTSZ             256    /*!< Audio in maximum packet size supported   \hideinitializer */
#define UAC_REQ_TIMEOUT              10000  /*!< UAC control request timeout value in miniseconds.   \hideinitializer */

#define UAC_SPEAKER                  1      /*!< Control target is speaker of UAC device. \hideinitializer */
#define UAC_MICROPHONE               2      /*!< Control target is microphone of UAC device. \hideinitializer */


/*
 * Constants
 */
#define UAC_RET_OK                   0      /*!< Return with no errors.  \hideinitializer */
#define UAC_RET_DEV_NOT_FOUND        -9     /*!< Audio Class device not found or removed.  \hideinitializer */
#define UAC_RET_IO_ERR               -11    /*!< USB transfer failed.  \hideinitializer */
#define UAC_RET_INVALID              -13    /*!< Invalid parameter or usage.  \hideinitializer */
#define UAC_RET_OUT_OF_MEMORY        -15    /*!< Out of memory.  \hideinitializer */
#define UAC_RET_DRV_NOT_SUPPORTED    -17    /*!< Function not supported by this UAC driver.  \hideinitializer */
#define UAC_RET_DEV_NOT_SUPPORTED    -19    /*!< Function not supported by the UAC device.  \hideinitializer */


/*
 * Audio Class-Specific Request Codes
 */
#define UAC_SET_CUR                  0x01   /*!< UAC request to set current value  \hideinitializer */
#define UAC_GET_CUR                  0x81   /*!< UAC request to get current value  \hideinitializer */
#define UAC_SET_MIN                  0x02   /*!< UAC request to set lower-bound setting  \hideinitializer */
#define UAC_GET_MIN                  0x82   /*!< UAC request to get lower-bound setting  \hideinitializer */
#define UAC_SET_MAX                  0x03   /*!< UAC request to set upper-bound setting  \hideinitializer */
#define UAC_GET_MAX                  0x83   /*!< UAC request to get upper-bound setting  \hideinitializer */
#define UAC_SET_RES                  0x04   /*!< UAC request to set resolution  \hideinitializer */
#define UAC_GET_RES                  0x84   /*!< UAC request to get resolution  \hideinitializer */
#define UAC_GET_STAT                 0xFF   /*!< UAC request to get status  \hideinitializer */

/*
 * Audio Class-Specific Channel selection
 */
#define UAC_CH_MASTER                0      /*!< Select all channels  \hideinitializer */
#define UAC_CH_LEFT_FRONT            1      /*!< Select Left Front (L) channel \hideinitializer */
#define UAC_CH_RIGHT_FRONT           2      /*!< Select Right Front (R) channel  \hideinitializer */
#define UAC_CH_CENTER_FRONT          3      /*!< Select Center Front (C) channel \hideinitializer */
#define UAC_CH_LOW_FREQ_EN           4      /*!< Select Low Frequency Enhancement (LFE) channel \hideinitializer */
#define UAC_CH_LEFT_SRN              5      /*!< Select Left Surround (LS) channel \hideinitializer */
#define UAC_CH_RIGHT_SRN             6      /*!< Select Right Surround (RS) channel \hideinitializer */
#define UAC_CH_LEFT_OF_CENTER        7      /*!< Select Left of Center (LC) channel \hideinitializer */
#define UAC_CH_RIGHT_OF_CENTER       8      /*!< Select Right of Center (RC) channel \hideinitializer */
#define UAC_CH_SURROUND              9      /*!< Select Surround (S) channel \hideinitializer */
#define UAC_CH_SIDE_LEFT             10     /*!< Select Side Left (SL) channel \hideinitializer */
#define UAC_CH_SIDE_RIGHT            11     /*!< Select Side Right (SR) channel \hideinitializer */
#define UAC_CH_TOP                   12     /*!< Select Top (T) channel \hideinitializer */



/*@}*/ /* end of group NUC472_442_USBH_AS_EXPORTED_CONSTANTS */

struct uac_dev_t;


/** @addtogroup NUC472_442_USBH_AS_EXPORTED_TYPEDEFS USB Host Audio Class Driver Exported Type Define
  @{
*/

typedef int (UAC_CB_FUNC)(struct uac_dev_t *dev, uint8_t *data, int len);    /*!< audio in callback function \hideinitializer */


/*@}*/ /* end of group NUC472_442_USBH_AS_EXPORTED_TYPEDEFS */


/** @addtogroup NUC472_442_USBH_AS_EXPORTED_STRUCTURES USB Host Audio Class Driver Exported Structures
  @{
*/

/*-----------------------------------------------------------------------------------
 *  Audio Class device
 */
/*! Audio Class device structure \hideinitializer      */
typedef struct uac_dev_t {
    USB_DEV_T           *udev;          /*!< USB device pointer of UAC_DEV_T \hideinitializer  */
    int                 ctrl_ifnum;     /*!< Audio control interface numder \hideinitializer   */
    int                 au_in_ifnum;    /*!< Audio data-in interface numder \hideinitializer   */
    int                 au_out_ifnum;   /*!< Audio data-out interface numder \hideinitializer  */
    void                *priv;          /*!< Internal used by audio class driver \hideinitializer  */
    struct uac_dev_t    *next;          /*!< Point to the next Audio Class device \hideinitializer     */

    /*
     *  The followings are used for audio streaming.
     */
    EP_INFO_T           *ep_au_in;      /*!< Audio data input endoint \hideinitializer         */
    EP_INFO_T           *ep_au_out;     /*!< Audio data output endoint \hideinitializer        */
    URB_T *             urbin[ISO_IN_URB_CNT];       /*!< Audio data input URB \hideinitializer             */
    URB_T *             urbout[ISO_OUT_URB_CNT];     /*!< Audio data output URB \hideinitializer            */
    uint8_t             iso_inbuf[ISO_IN_URB_CNT][AU_IN_MAX_PKTSZ*ISO_FRAME_COUNT];    /*!< USB isochronous-in buffer \hideinitializer  */
    uint8_t             iso_outbuf[ISO_OUT_URB_CNT][AU_OUT_MAX_PKTSZ*ISO_FRAME_COUNT]; /*!< USB isochronous-out buffer \hideinitializer  */
    uint8_t             in_streaming;   /*!< Audio data in is streaming or not. \hideinitializer        */
    uint8_t             out_streaming;  /*!< Audio data out is streaming or not. \hideinitializer        */

    /*
     *  The followings are used for interworking with user application.
     */
    UAC_CB_FUNC         *au_in_func;    /*!< Audio data input callback function \hideinitializer   */
    UAC_CB_FUNC         *au_out_func;   /*!< Audio data output callback function \hideinitializer  */
    uint8_t             *au_in_buff;    /*!< Point to the user provided audio input buffer \hideinitializer  */
    int                 au_in_bufsz;    /*!< Size of au_in_buff \hideinitializer  */
    int                 au_in_bufidx;   /*!< Index for Audio Class driver writing au_in_buff \hideinitializer  */
} UAC_DEV_T;                             /*! Audio Class device structure \hideinitializer             */


/*@}*/ /* end of group NUC472_442_USBH_AS_EXPORTED_STRUCTURES */


#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup NUC472_442_USBH_AS_EXPORTED_FUNCTIONS USB Host Audio Class Driver Exported Functions
  @{
*/

void     UAC_Init(void);
UAC_DEV_T * UAC_GetDeviceList(void);

int32_t  UAC_GetChannelNumber(UAC_DEV_T *audev, uint8_t target);
int32_t  UAC_GetBitResolution(UAC_DEV_T *audev, uint8_t target, uint8_t *val8);
int32_t  UAC_GetSamplingRate(UAC_DEV_T *audev, uint8_t target, uint32_t *srate_list, int max_cnt, uint8_t *type);
int32_t  UAC_MuteControl(UAC_DEV_T *audev, uint8_t target, uint8_t req, uint16_t chn, uint8_t *data);
int32_t  UAC_VolumeControl(UAC_DEV_T *audev, uint8_t target, uint8_t req, uint16_t chn, uint16_t *volume);
int32_t  UAC_AutoGainControl(UAC_DEV_T *audev, uint8_t target, uint8_t req, uint16_t chn, uint8_t *gain);
int32_t  UAC_SamplingRateControl(UAC_DEV_T *audev, uint8_t target, uint8_t req, uint32_t *srate);

int32_t  UAC_InstallIsoInCbFun(UAC_DEV_T *audev, uint8_t *au_in_buff, int bufsiz, UAC_CB_FUNC *func);
int32_t  UAC_StartIsoInPipe(UAC_DEV_T *audev);
int32_t  UAC_StopIsoInPipe(UAC_DEV_T *audev);

int32_t  UAC_InstallIsoOutCbFun(UAC_DEV_T *audev, UAC_CB_FUNC *func);
int32_t  UAC_StartIsoOutPipe(UAC_DEV_T *audev);
int32_t  UAC_StopIsoOutPipe(UAC_DEV_T *audev);


/*@}*/ /* end of group NUC472_442_USBH_AS_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC472_442_USBH_AS_Driver */

/*@}*/ /* end of group NUC472_442_Device_Driver */


#ifdef __cplusplus
}
#endif

#endif /* __INCLUDED_USBHUAC_H__ */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

