/**************************************************************************//**
 * @file     pwm.h
 * @version  V1.00
 * $Revision: 11 $
 * $Date: 15/09/23 1:57p $ 
 * @brief    Mini51 series PWM driver header file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/ 
#ifndef __PWM_H__
#define __PWM_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_PWM_Driver PWM Driver
  @{
*/

/** @addtogroup MINI51_PWM_EXPORTED_CONSTANTS PWM Exported Constants
  @{
*/
#define PWM_CHANNEL_NUM                     (6)   /*!< PWM channel number */
#define PWM_CLK_DIV_1                       (4UL) /*!< PWM clock divide by 1 */
#define PWM_CLK_DIV_2                       (0UL) /*!< PWM clock divide by 2 */
#define PWM_CLK_DIV_4                       (1UL) /*!< PWM clock divide by 4 */
#define PWM_CLK_DIV_8                       (2UL) /*!< PWM clock divide by 8 */
#define PWM_CLK_DIV_16                      (3UL) /*!< PWM clock divide by 16 */
#define PWM_EDGE_ALIGNED                    (0UL)                   /*!< PWM working in edge aligned type */
#define PWM_CENTER_ALIGNED                  (PWM_PCR_PWMTYPE_Msk)   /*!< PWM working in center aligned type */
#define PWM_TRIGGER_ADC_CNTR_IS_0           PWM_TRGCON0_P0TRGEN_Msk     /*!< PWM trigger ADC while counter matches 0 */
#define PWM_TRIGGER_ADC_CNTR_IS_CMR_D       PWM_TRGCON0_CM0TRGFEN_Msk   /*!< PWM trigger ADC while counter matches CMR during down count */
#define PWM_TRIGGER_ADC_CNTR_IS_CNR         PWM_TRGCON0_CNT0TRGEN_Msk   /*!< PWM trigger ADC while counter matches CNR */
#define PWM_TRIGGER_ADC_CNTR_IS_CMR_U       PWM_TRGCON0_CM0TRGREN_Msk   /*!< PWM trigger ADC while counter matches CMR during up count  */
#define PWM_FB0_EINT0       (PWM_PFBCON_BKEN0_Msk)                              /*!< External interrupt 0 as fault brake 0 source */
#define PWM_FB0_ACMP1       (PWM_PFBCON_BKEN0_Msk | PWM_PFBCON_CPO1BKEN_Msk)    /*!< Comparator 1 as fault brake 0 source */
#define PWM_FB1_EINT1       (PWM_PFBCON_BKEN1_Msk)                              /*!< External interrupt 1 as fault brake 1 source */
#define PWM_FB1_ACMP0       (PWM_PFBCON_BKEN1_Msk | PWM_PFBCON_CPO0BKEN_Msk)    /*!< Comparator 0 as fault brake 1 source */
#define PWM_PERIOD_INT_UNDERFLOW            (0)                         /*!< PWM period interrupt trigger if counter underflow */
#define PWM_PERIOD_INT_MATCH_CNR            (PWM_PIER_INT_TYPE_Msk)     /*!< PWM period interrupt trigger if counter match CNR */

/*@}*/ /* end of group MINI51_PWM_EXPORTED_CONSTANTS */


/** @addtogroup MINI51_PWM_EXPORTED_FUNCTIONS PWM Exported Functions
  @{
*/

/**
 * @brief This macro enable complementary mode
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_ENABLE_COMPLEMENTARY_MODE(pwm) (PWM->PCR = (PWM->PCR & ~PWM_PCR_PWMMOD_Msk) |(1UL << PWM_PCR_PWMMOD_Pos))

/**
 * @brief This macro disable complementary mode, and enable independent mode.
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_DISABLE_COMPLEMENTARY_MODE(pwm) (PWM->PCR &= ~PWM_PCR_PWMMOD_Msk)

/**
 * @brief This macro enable group mode
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_ENABLE_GROUP_MODE(pwm) (PWM->PCR |= PWM_PCR_GRP_Msk)

/**
 * @brief This macro disable group mode
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_DISABLE_GROUP_MODE(pwm) (PWM->PCR &= ~PWM_PCR_GRP_Msk)

/**
 * @brief This macro enable synchronous mode
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_ENABLE_SYNC_MODE(pwm) (PWM->PCR = (PWM->PCR & ~PWM_PCR_PWMMOD_Msk) |(2UL << PWM_PCR_PWMMOD_Pos))
 
/**
 * @brief This macro disable synchronous mode, and enable independent mode.
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_DISABLE_SYNC_MODE(pwm) (PWM->PCR &= ~PWM_PCR_PWMMOD_Msk)

/**
 * @brief This macro enable output inverter of specified channel(s)
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Bit 0 represents channel 0, bit 1 represents channel 1...
 * @return None
 * \hideinitializer
 */
#define PWM_ENABLE_OUTPUT_INVERTER(pwm, u32ChannelMask) \
    do{ \
        int i;\
        for(i = 0; i < 6; i++) { \
            if((u32ChannelMask) & (1 << i)) {\
                PWM->PCR |= (1 << (PWM_PCR_CH0INV_Pos + (i * 4))); \
            }\
        } \
    }while(0)

/**
 * @brief This macro set the prescaler of the selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Prescaler Clock prescaler of specified channel. Valid values are between 1 ~ 0xFF
 * @return None
 * @note Every even channel N, and channel (N + 1) share a prescaler. So if channel 0 prescaler changed, 
 *       channel 1 will also be affected.
 * \hideinitializer
 */
#define PWM_SET_PRESCALER(pwm, u32ChannelNum, u32Prescaler) \
    (PWM->PPR = (PWM->PPR & ~(PWM_PPR_CP01_Msk << (((u32ChannelNum) >> 1) * 8))) | ((u32Prescaler) << (((u32ChannelNum) >> 1) * 8))) 

/**
 * @brief This macro set the divider of the selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Divider Clock divider of specified channel. Valid values are
 *              - \ref PWM_CLK_DIV_1
 *              - \ref PWM_CLK_DIV_2
 *              - \ref PWM_CLK_DIV_4
 *              - \ref PWM_CLK_DIV_8
 *              - \ref PWM_CLK_DIV_16 
 * @return None
 * \hideinitializer
 */
#define PWM_SET_DIVIDER(pwm, u32ChannelNum, u32Divider) \
    (PWM->CSR = (PWM->CSR & ~(PWM_CSR_CSR0_Msk << ((u32ChannelNum) * 4))) | ((u32Divider) << ((u32ChannelNum) * 4)))

/**
 * @brief This macro set the duty of the selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5 
 * @param[in] u32CMR Duty of specified channel. Valid values are between 0~0xFFFF
 * @return None
 * @note This new setting will take effect on next PWM period
 * \hideinitializer
 */
#define PWM_SET_CMR(pwm, u32ChannelNum, u32CMR) (PWM->CMR[u32ChannelNum] = (u32CMR))

/**
 * @brief This macro set the period of the selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5 
 * @param[in] u32CNR Period of specified channel. Valid values are between 0~0xFFFF
 * @return None
 * @note This new setting will take effect on next PWM period
 * @note PWM counter will stop if period length set to 0
 * \hideinitializer
 */
#define PWM_SET_CNR(pwm, u32ChannelNum, u32CNR)  (PWM->CNR[u32ChannelNum] = (u32CNR))

/**
 * @brief This macro set the PWM aligned type
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask This parameter is not used
 * @param[in] u32AlignedType PWM aligned type, valid values are:
 *                  - PWM_EDGE_ALIGNED
 *                  - PWM_CENTER_ALIGNED
 * @return None
 * \hideinitializer
 */
#define PWM_SET_ALIGNED_TYPE(pwm, u32ChannelMask, u32AlignedType) \
    (PWM->PCR = (PWM->PCR & ~PWM_PCR_PWMTYPE_Msk) | (u32AlignedType))


uint32_t PWM_ConfigOutputChannel(PWM_T *pwm,
                                  uint32_t u32ChannelNum, 
                                  uint32_t u32Frequency, 
                                  uint32_t u32DutyCycle);
void PWM_Start(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_Stop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_ForceStop(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
void PWM_DisableADCTrigger(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
uint32_t PWM_GetADCTriggerFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBrake(PWM_T *pwm, 
                           uint32_t u32ChannelMask, 
                           uint32_t u32LevelMask, 
                           uint32_t u32BrakeSource);
void PWM_ClearFaultBrakeFlag(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_EnableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_DisableOutput(PWM_T *pwm, uint32_t u32ChannelMask);
void PWM_EnableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration);
void PWM_DisableDeadZone(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType);
void PWM_DisableDutyInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetDutyIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_EnableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_DisableFaultBrakeInt(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_ClearFaultBrakeIntFlag(PWM_T *pwm, uint32_t u32BrakeSource);
uint32_t PWM_GetFaultBrakeIntFlag(PWM_T *pwm, uint32_t u32BrakeSource);
void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);



/*@}*/ /* end of group MINI51_PWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_PWM_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__PWM_H__

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
