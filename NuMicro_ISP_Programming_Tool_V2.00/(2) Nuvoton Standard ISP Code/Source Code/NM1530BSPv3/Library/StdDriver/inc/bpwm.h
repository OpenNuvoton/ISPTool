/**************************************************************************//**
 * @file     bpwm.h
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 15/04/01 3:00p $
 * @brief    BPWM driver header file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __BPWM_H__
#define __BPWM_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup BPWM_Driver BPWM Driver
  @{
*/

/** @addtogroup BPWM_EXPORTED_CONSTANTS BPWM Exported Constants
  @{
*/
#define BPWM_CHANNEL_NUM                     (2)   /*!< BPWM channel number */

/*---------------------------------------------------------------------------------------------------------*/
/*  BPWM Mode Clock Pre-Divider Selection Constant Definitions                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define BPWM_CLK_DIV_1                       (4UL) /*!< BPWM clock divide by 1 */
#define BPWM_CLK_DIV_2                       (0UL) /*!< BPWM clock divide by 2 */
#define BPWM_CLK_DIV_4                       (1UL) /*!< BPWM clock divide by 4 */
#define BPWM_CLK_DIV_8                       (2UL) /*!< BPWM clock divide by 8 */
#define BPWM_CLK_DIV_16                      (3UL) /*!< BPWM clock divide by 16 */

/*---------------------------------------------------------------------------------------------------------*/
/*  Aligned Type Constant Definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define BPWM_EDGE_ALIGNED                    (0UL)                    /*!< BPWM working in edge aligned type */
#define BPWM_CENTER_ALIGNED                  (BPWM_PCR_PWM01TYPE_Msk) /*!< BPWM working in center aligned type */
#define BPWM_PERIOD_INT_UNDERFLOW            (0)                         /*!< BPWM period interrupt triggered if counter underflow */
#define BPWM_PERIOD_INT_MATCH_CNR            (BPWM_PIER_INTTYPE_Msk)     /*!< BPWM period interrupt triggered if counter match CNR */

/*---------------------------------------------------------------------------------------------------------*/
/*  Capture Control Constant Definitions                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define BPWM_CAPTURE_INT_RISING_LATCH        (BPWM_CCR_CRL_IE0_Msk)      /*!< BPWM capture interrupt if channel has rising transition */
#define BPWM_CAPTURE_INT_FALLING_LATCH       (BPWM_CCR_CFL_IE0_Msk)      /*!< BPWM capture interrupt if channel has falling transition */

/*---------------------------------------------------------------------------------------------------------*/
/*  BPWM Group channel number constants definitions                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BPWM_CH0                             0x0                         /*!< BPWM channel 0 */
#define BPWM_CH1                             0x1                         /*!< BPWM channel 1 */

#define BPWM_CCR_MASK                        0x000F000F                  /*!< BPWM CCR bit0~3 and bit16~19 mask */

/*@}*/ /* end of group BPWM_EXPORTED_CONSTANTS */


/** @addtogroup BPWM_EXPORTED_FUNCTIONS BPWM Exported Functions
  @{
*/


/**
 * @brief Enable output inverter of specified channel(s).
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 represents channel 0, bit 1 represents channel 1.
 * @return None.
 * @details This macro is used to enable output inverter for specified channel(s).
 * \hideinitializer
 */
#define BPWM_ENABLE_OUTPUT_INVERTER(bpwm, u32ChannelMask) \
    do{ \
        int i;\
        (bpwm)->PCR &= ~(BPWM_PCR_CH0INV_Msk|BPWM_PCR_CH1INV_Msk);\
        for(i = 0; i < BPWM_CHANNEL_NUM; i++) { \
            if((u32ChannelMask) & (1 << i)) \
                (bpwm)->PCR |= (BPWM_PCR_CH0INV_Msk << (BPWM_PCR_CH0INV_Pos * (i * 4))); \
        } \
    }while(0)

/**
 * @brief Get captured rising data of specified channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1.
 * @return The timer counter, 0~0xFFFF.
 * @details This macro is used to get captured rising data for specified channel.
 */
#define BPWM_GET_CAPTURE_RISING_DATA(bpwm, u32ChannelNum) (*((__IO uint32_t *) ((((uint32_t)&((bpwm)->CRLR0)) + (u32ChannelNum) * 8))))

/**
 * @brief Get captured falling data of specified channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1.
 * @return The timer counter, 0~0xFFFF.
 * @details This macro is used to get captured falling data for specified channel.
 */
#define BPWM_GET_CAPTURE_FALLING_DATA(bpwm, u32ChannelNum) (*((__IO uint32_t *) ((((uint32_t)&((bpwm)->CFLR0)) + (u32ChannelNum) * 8))))

/**
 * @brief Set the prescaler of the selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. This parameter is not used.
 * @param[in] u32Prescaler Clock prescaler of specified channel. Valid values are between 1 ~ 0xFF.
 * @return None.
 * @details This macro is used to set timer pre-scale for specified channel.
 * @note If u32Prescaler = 0, corresponding BPWM-timer will be stopped.
 * @note If u32Prescaler = x (x not equal to 0), it means Clock input is divided by (x + 1) before it is fed to the corresponding BPWM counter.
 * @note Channel 0 and channel 1 share a prescaler.
 */
#define BPWM_SET_PRESCALER(bpwm, u32ChannelNum, u32Prescaler) \
    ((bpwm)->PPR = ((bpwm)->PPR & ~(BPWM_PPR_CP01_Msk)) | (u32Prescaler))

/**
 * @brief Set the divider of the selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1.
 * @param[in] u32Divider Clock divider of specified channel. Valid values are
 *              - \ref BPWM_CLK_DIV_1
 *              - \ref BPWM_CLK_DIV_2
 *              - \ref BPWM_CLK_DIV_4
 *              - \ref BPWM_CLK_DIV_8
 *              - \ref BPWM_CLK_DIV_16
 * @return None.
 * @details This macro is used to set Timer clock source divider selection for specified channel.
 */
#define BPWM_SET_DIVIDER(bpwm, u32ChannelNum, u32Divider) \
    ((bpwm)->CSR = ((bpwm)->CSR & ~(BPWM_CSR_CSR0_Msk << ((u32ChannelNum) * 4))) | ((u32Divider) << ((u32ChannelNum) * 4)))

/**
 * @brief Set the duty of the selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1.
 * @param[in] u32CMR Duty of specified channel. Valid values are between 0~0xFFFF.
 * @return None.
 * @details This macro is used to set BPWM Comparator value for specified channel.
 * @note This new setting will take effect on next BPWM period.
 */
#define BPWM_SET_CMR(bpwm, u32ChannelNum, u32CMR) (*((__IO uint32_t *) ((((uint32_t)&((bpwm)->CMR0)) + (u32ChannelNum) * 12))) = (u32CMR))

/**
 * @brief Set the period of the selected channel.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelNum BPWM channel number. Valid values are between 0~1.
 * @param[in] u32CNR Period of specified channel. Valid values are between 0~0xFFFF for edge aligned type and Valid values are between 0~0xFFFE for center aligned type.
 * @return None.
 * @details This macro is used to set timer loaded value(CNR) for specified channel.\n
 *          Loaded value determines the BPWM period.
 * @note This new setting will take effect on next BPWM period.
 * @note BPWM counter will stop if period length set to 0.
 */
#define BPWM_SET_CNR(bpwm, u32ChannelNum, u32CNR)  (*((__IO uint32_t *) ((((uint32_t)&((bpwm)->CNR0)) + (u32ChannelNum) * 12))) = (u32CNR))

/**
 * @brief Set the BPWM aligned type.
 * @param[in] bpwm The pointer of the specified BPWM module.
 * @param[in] u32ChannelMask Combination of enabled channels. This parameter is not used.
 * @param[in] u32AlignedType BPWM aligned type, valid values are:
 *                  - \ref BPWM_EDGE_ALIGNED
 *                  - \ref BPWM_CENTER_ALIGNED
 * @return None.
 * @details This macro is used to set the BPWM aligned type.
 * @note Channel 0 and channel 1 share the same configuration.
 */
#define BPWM_SET_ALIGNED_TYPE(bpwm, u32ChannelMask, u32AlignedType) ((bpwm)->PCR = ((bpwm)->PCR & ~(BPWM_PCR_PWM01TYPE_Msk)) | (u32AlignedType))


uint32_t BPWM_ConfigCaptureChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32UnitTimeNsec, uint32_t u32CaptureEdge);
uint32_t BPWM_ConfigOutputChannel(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Frequncy, uint32_t u32DutyCycle);
void BPWM_Start(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_Stop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_ForceStop(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableCapture(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_DisableOutput(BPWM_T *bpwm, uint32_t u32ChannelMask);
void BPWM_EnableDeadZone(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void BPWM_DisableDeadZone(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_DisableCaptureInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
void BPWM_ClearCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32Edge);
uint32_t BPWM_GetCaptureIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void BPWM_DisableDutyInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetDutyIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_EnablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void BPWM_DisablePeriodInt(BPWM_T *bpwm, uint32_t u32ChannelNum);
void BPWM_ClearPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);
uint32_t BPWM_GetPeriodIntFlag(BPWM_T *bpwm, uint32_t u32ChannelNum);



/*@}*/ /* end of group BPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group BPWM_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__BPWM_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
