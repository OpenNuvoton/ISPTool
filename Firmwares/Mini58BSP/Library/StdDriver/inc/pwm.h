/**************************************************************************//**
 * @file     pwm.h
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/06/05 1:41p $ 
 * @brief    Mini58 series PWM driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/ 
#ifndef __PWM_H__
#define __PWM_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_PWM_Driver PWM Driver
  @{
*/

/** @addtogroup Mini58_PWM_EXPORTED_CONSTANTS PWM Exported Constants
  @{
*/
#define PWM_CHANNEL_NUM                     (6)   /*!< PWM channel number */
#define PWM_CLK_DIV_1                       (4UL) /*!< PWM clock divide by 1 */
#define PWM_CLK_DIV_2                       (0UL) /*!< PWM clock divide by 2 */
#define PWM_CLK_DIV_4                       (1UL) /*!< PWM clock divide by 4 */
#define PWM_CLK_DIV_8                       (2UL) /*!< PWM clock divide by 8 */
#define PWM_CLK_DIV_16                      (3UL) /*!< PWM clock divide by 16 */
#define PWM_EDGE_ALIGNED                    (0UL)                   		/*!< PWM working in edge aligned type */
#define PWM_CENTER_ALIGNED                  (PWM_CTL_CNTTYPE_Msk)   		/*!< PWM working in center aligned type */
#define PWM_TRIGGER_ADC_CNTR_IS_0           PWM_ADCTCTL0_ZPTRGEN0_Msk   /*!< PWM trigger ADC while counter matches 0 */
#define PWM_TRIGGER_ADC_CNTR_IS_CMR_D       PWM_ADCTCTL0_CDTRGEN0_Msk   /*!< PWM trigger ADC while counter matches CMR during down count */
#define PWM_TRIGGER_ADC_CNTR_IS_CNR         PWM_ADCTCTL0_CPTRGEN0_Msk   /*!< PWM trigger ADC while counter matches CNR */
#define PWM_TRIGGER_ADC_CNTR_IS_CMR_U       PWM_ADCTCTL0_CUTRGEN0_Msk   /*!< PWM trigger ADC while counter matches CMR during up count  */
#define PWM_FB0_EINT0       								(PWM_BRKCTL_BRK0EN_Msk)                              /*!< External interrupt 0 as fault brake 0 source */
#define PWM_FB0_ACMP1       								(PWM_BRKCTL_BRK0EN_Msk | PWM_BRKCTL_BRK1SEL_Msk)     /*!< Comparator 1 as fault brake 0 source */
#define PWM_FB1_EINT1       								(PWM_BRKCTL_BRK1EN_Msk)                              /*!< External interrupt 1 as fault brake 1 source */
#define PWM_FB1_ACMP0       								(PWM_BRKCTL_BRK1EN_Msk | PWM_BRKCTL_BRK0SEL_Msk)     /*!< Comparator 0 as fault brake 1 source */
#define PWM_PERIOD_INT_UNDERFLOW            (0)                                  /*!< PWM period interrupt trigger if counter underflow */
#define PWM_PERIOD_INT_MATCH_CNR            (PWM_INTEN_PINTTYPE_Msk)             /*!< PWM period interrupt trigger if counter match CNR */
/*---------------------------------------------------------------------------------------------------------*/
/*  PWM Group channel number constants definitions                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define PWM_CH0                             0x0                         /*!< PWM channel 0 */
#define PWM_CH1                             0x1                         /*!< PWM channel 1 */
#define PWM_CH2                             0x2                         /*!< PWM channel 2 */
#define PWM_CH3                             0x3                         /*!< PWM channel 3 */
#define PWM_CH4                             0x4                         /*!< PWM channel 4 */
#define PWM_CH5                             0x5                         /*!< PWM channel 5 */

/*@}*/ /* end of group Mini58_PWM_EXPORTED_CONSTANTS */


/** @addtogroup Mini58_PWM_EXPORTED_FUNCTIONS PWM Exported Functions
  @{
*/

/**
 * @brief This macro enable complementary mode
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_ENABLE_COMPLEMENTARY_MODE(pwm) ((pwm)->CTL = (PWM->CTL & ~PWM_CTL_MODE_Msk) |(1UL << PWM_CTL_MODE_Pos))

/**
 * @brief This macro disable complementary mode, and enable independent mode.
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_DISABLE_COMPLEMENTARY_MODE(pwm) ((pwm)->CTL &= ~PWM_CTL_MODE_Msk)

/**
 * @brief This macro enable group mode
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_ENABLE_GROUP_MODE(pwm) ((pwm)->CTL |= PWM_CTL_GROUPEN_Msk)

/**
 * @brief This macro disable group mode
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_DISABLE_GROUP_MODE(pwm) ((pwm)->CTL &= ~PWM_CTL_GROUPEN_Msk)

/**
 * @brief This macro enable synchronous mode
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_ENABLE_SYNC_MODE(pwm) (PWM->CTL = ((pwm)->CTL & ~PWM_CTL_MODE_Msk) |(2UL << PWM_CTL_MODE_Pos))
 
/**
 * @brief This macro disable synchronous mode, and enable independent mode.
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_DISABLE_SYNC_MODE(pwm) ((pwm)->CTL &= ~PWM_CTL_MODE_Msk)

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
			  (pwm)->CTL &= ~(PWM_CTL_PINV0_Msk|PWM_CTL_PINV1_Msk|PWM_CTL_PINV2_Msk|PWM_CTL_PINV3_Msk|PWM_CTL_PINV4_Msk|PWM_CTL_PINV5_Msk);\
        for(i = 0; i < 6; i++) { \
            if((u32ChannelMask) & (1 << i)) \
                (pwm)->CTL |= (1 << (PWM_CTL_PINV0_Pos + (i * 4))); \
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
    ((pwm)->CLKPSC = ((pwm)->CLKPSC & ~(PWM_CLKPSC_CLKPSC01_Msk << (((u32ChannelNum) >> 1) * 8))) | ((u32Prescaler) << (((u32ChannelNum) >> 1) * 8))) 

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
    ((pwm)->CLKDIV = ((pwm)->CLKDIV & ~(PWM_CLKDIV_CLKDIV0_Msk << ((u32ChannelNum) * 4))) | ((u32Divider) << ((u32ChannelNum) * 4)))

/**
 * @brief This macro set the duty of the selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5 
 * @param[in] u32CMR Duty of specified channel. Valid values are between 0~0xFFFF
 * @return None
 * @note This new setting will take effect on next PWM period
 * \hideinitializer
 */
#define PWM_SET_CMR(pwm, u32ChannelNum, u32CMR) (*((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4)))= (u32CMR))

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
#define PWM_SET_CNR(pwm, u32ChannelNum, u32CNR)  (*((__IO uint32_t *) ((((uint32_t)&((pwm)->PERIOD0)) + (u32ChannelNum) * 4))) = (u32CNR))

/**
 * @brief This macro set the duty of the selected channel for PWM asymmetric Mode
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5 
 * @param[in] u32CMRD Down counter in Asymmetric Mode. Valid values are between 0~0xFFFF
 * @return None
 * @note This new setting will take effect on next PWM period
 * \hideinitializer
 */
#define PWM_SET_CMRD(pwm, u32ChannelNum, u32CMRD) (*((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + (u32ChannelNum) * 4)))= (u32CMRD << 16))

/**
 * @brief This macro set the PWM aligned type
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask This parameter is not used
 * @param[in] u32AlignedType PWM aligned type, valid values are:
 *                  - \ref PWM_EDGE_ALIGNED
 *                  - \ref PWM_CENTER_ALIGNED
 * @return None
 * \hideinitializer
 */
#define PWM_SET_ALIGNED_TYPE(pwm, u32ChannelMask, u32AlignedType) \
    ((pwm)->CTL = ((pwm)->CTL & ~PWM_CTL_CNTTYPE_Msk) | (u32AlignedType))

/**
 * @brief This macro enables PWM asymmetric mode
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_ENABLE_ASYMMETRIC_MODE(pwm) ((pwm)->CTL |= PWM_CTL_ASYMEN_Msk)

/**
 * @brief This macro enables PWM Precise Center-Aligned Type
 * @param[in] pwm The base address of PWM module
 * @return None
 * \hideinitializer
 */
#define PWM_ENABLE_PCAEN(pwm) (PWM->PCACTL |= PWM_PCACTL_PCAEN_Msk)

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
uint32_t PWM_GetADCTriggerFlag (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition);
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

void PWM_EnablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_DisablePeriodInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetPeriodIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);

void PWM_EnableCenterInt(PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntPeriodType);
void PWM_DisableCenterInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearCenterIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetCenterIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);

void PWM_EnableRiseInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_DisableRiseInt(PWM_T *pwm, uint32_t u32ChannelNum);
void PWM_ClearRiseIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);
uint32_t PWM_GetRiseIntFlag(PWM_T *pwm, uint32_t u32ChannelNum);

/*@}*/ /* end of group Mini58_PWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_PWM_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__PWM_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
