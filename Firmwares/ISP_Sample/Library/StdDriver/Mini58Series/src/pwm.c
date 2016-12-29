/**************************************************************************//**
 * @file     PWM.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/06/05 1:43p $
 * @brief    Mini58 series PWM driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini58Series.h"

/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_PWM_Driver PWM Driver
  @{
*/


/** @addtogroup Mini58_PWM_EXPORTED_FUNCTIONS PWM Exported Functions
  @{
*/

/**
 * @brief This function config PWM generator and get the nearest frequency in edge aligned auto-reload mode
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Frequency Target generator frequency
 * @param[in] u32DutyCycle Target generator duty cycle percentage. Valid range are between 0 ~ 100. 10 means 10%, 20 means 20%...
 * @return Nearest frequency clock in nano second
 * @note Since every two channels, (0 & 1), (2 & 3), (4 & 5), shares a prescaler. Call this API to configure PWM frequency may affect
 *       existing frequency of other channel.
 */
uint32_t PWM_ConfigOutputChannel (PWM_T *pwm,
                                  uint32_t u32ChannelNum,
                                  uint32_t u32Frequency,
                                  uint32_t u32DutyCycle)
{
    uint32_t i = SystemCoreClock / u32Frequency;
    uint8_t  u8Divider = 1, u8Prescale = 0xFF;
    uint16_t u16CNR = 0xFFFF;

    for(; u8Divider < 17; u8Divider <<= 1) {  // clk divider could only be 1, 2, 4, 8, 16
        i = (SystemCoreClock / u32Frequency) / u8Divider;
        // If target value is larger than CNR * prescale, need to use a larger divider
        if(i > (0x10000 * 0x100))
            continue;

        // CNR = 0xFFFF + 1, get a prescaler that CNR value is below 0xFFFF
        u8Prescale = (i + 0xFFFF)/ 0x10000;

        // u8Prescale must at least be 2, otherwise the output stop
        if(u8Prescale < 3)
            u8Prescale = 2;

        i /= u8Prescale;

        if(i <= 0x10000) {
            if(i == 1)
                u16CNR = 1;     // Too fast, and PWM cannot generate expected frequency...
            else
                u16CNR = i;
            break;
        }

    }
    // Store return value here 'cos we're gonna change u8Divider & u8Prescale & u16CNR to the real value to fill into register
    i = SystemCoreClock / (u8Prescale * u8Divider * u16CNR);

    u8Prescale -= 1;
    u16CNR -= 1;
    // convert to real register value
    if(u8Divider == 1)
        u8Divider = 4;
    else if (u8Divider == 2)
        u8Divider = 0;
    else if (u8Divider == 4)
        u8Divider = 1;
    else if (u8Divider == 8)
        u8Divider = 2;
    else // 16
        u8Divider = 3;

    // every two channels share a prescaler
    PWM->CLKPSC = (PWM->CLKPSC & ~(PWM_CLKPSC_CLKPSC01_Msk << ((u32ChannelNum >> 1) * 8))) | (u8Prescale << ((u32ChannelNum >> 1) * 8));
    PWM->CLKDIV = (PWM->CLKDIV & ~(PWM_CLKDIV_CLKDIV0_Msk << (4 * u32ChannelNum))) | (u8Divider << (4 * u32ChannelNum));
    PWM->CTL = (PWM->CTL & ~PWM_CTL_CNTTYPE_Msk) | (PWM_CTL_CNTMODE0_Msk << ((4 * u32ChannelNum)));

    if(u32DutyCycle == 0)
        *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = 0;
    else
        *((__IO uint32_t *)((((uint32_t) & ((pwm)->CMPDAT0)) + u32ChannelNum * 4))) = u32DutyCycle * (u16CNR + 1) / 100 - 1;

    *((__IO uint32_t *)((((uint32_t) & ((pwm)->PERIOD0)) + (u32ChannelNum) * 4))) = u16CNR;

    return(i);
}


/**
 * @brief This function start PWM module
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 */
void PWM_Start (PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t u32Mask = 0, i;
    for(i = 0; i < PWM_CHANNEL_NUM; i ++) {
        if(u32ChannelMask & (1 << i)) {
            u32Mask |= (PWM_CTL_CNTEN0_Msk << (i * 4));
        }
    }

    PWM->CTL |= u32Mask;
}

/**
 * @brief This function stop PWM module
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 */
void PWM_Stop (PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t i;
    for(i = 0; i < PWM_CHANNEL_NUM; i ++) {
        if(u32ChannelMask & (1 << i)) {
            *((__IO uint32_t *)((((uint32_t) & ((pwm)->PERIOD0)) + (i) * 4))) = 0;
        }
    }

}

/**
 * @brief This function stop PWM generation immediately by clear channel enable bit
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Bit 0 is channel 0, bit 1 is channel 1...
 * @return None
 */
void PWM_ForceStop (PWM_T *pwm, uint32_t u32ChannelMask)
{
    uint32_t u32Mask = 0, i;
    for(i = 0; i < PWM_CHANNEL_NUM; i ++) {
        if(u32ChannelMask & (1 << i)) {
            u32Mask |= (PWM_CTL_CNTEN0_Msk << (i * 4));
        }
    }

    PWM->CTL &= ~u32Mask;
}

/**
 * @brief This function enable selected channel to trigger ADC
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Condition The condition to trigger ADC. Combination of following conditions:
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_0
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CMR_D
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CNR
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CMR_U
 * @return None
 */
void PWM_EnableADCTrigger (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    if(u32ChannelNum < 4) {
        PWM->ADCTCTL0 = (PWM->ADCTCTL0 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * u32ChannelNum))) | (u32Condition << (8 * u32ChannelNum));
    } else {
        PWM->ADCTCTL1 = (PWM->ADCTCTL1 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * (u32ChannelNum - 4)))) | (u32Condition << (8 * (u32ChannelNum - 4)));
    }
}

/**
 * @brief This function disable selected channel to trigger ADC
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_DisableADCTrigger (PWM_T *pwm, uint32_t u32ChannelNum)
{
    if(u32ChannelNum < 4) {
        PWM->ADCTCTL0 = (PWM->ADCTCTL0 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * u32ChannelNum)));
    } else {
        PWM->ADCTCTL1 = (PWM->ADCTCTL1 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                            PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                            PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * (u32ChannelNum - 4))));
    }
}

/**
 * @brief This function clear selected channel trigger ADC flag
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Condition PWM triggered ADC flag to be cleared.
 * @return None
 */
void PWM_ClearADCTriggerFlag (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    if(u32ChannelNum < 4) {
        PWM->ADCTSTS0 |= (u32Condition << (8 * u32ChannelNum));
    } else {
        PWM->ADCTSTS1 |= (u32Condition << (8 * (u32ChannelNum - 4)));
    }
}

/**
 * @brief This function get selected channel trigger ADC flag
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Condition PWM triggered ADC flag to be selected.
 * @return Get status of the selected channel trigger ADC
 */
uint32_t PWM_GetADCTriggerFlag (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    if(u32ChannelNum < 4) {
        return(PWM->ADCTSTS0 & (u32Condition << (8 * u32ChannelNum)) ? 1 : 0);
    } else {
        return(PWM->ADCTSTS1 & (u32Condition << (8 * (u32ChannelNum - 4))) ? 1 : 0);
    }
}

/**
 * @brief This function enable fault brake of selected channels
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask This parameter is not used
 * @param[in] u32LevelMask Output high or low while fault brake occurs, each bit represent the level of a channel
 *                         while fault brake occur. Bit 0 represents channel 0, bit 1 represents channel 1...
 *                         , bit 6 represent D6, and bit 7 represents D7
 * @param[in] u32BrakeSource Fault brake source, could be one of following source
 *                  - \ref PWM_FB0_EINT0
 *                  - \ref PWM_FB0_ACMP1
 *                  - \ref PWM_FB1_EINT1
 *                  - \ref PWM_FB1_ACMP0
 * @return None
 */
void PWM_EnableFaultBrake (PWM_T *pwm,
                           uint32_t u32ChannelMask,
                           uint32_t u32LevelMask,
                           uint32_t u32BrakeSource)
{
    PWM->BRKCTL = (u32LevelMask << PWM_BRKCTL_BKOD0_Pos) | u32BrakeSource;
}

/**
 * @brief This function clear fault brake flag
 * @param[in] pwm The base address of PWM module
 * @param[in] u32BrakeSource This parameter is not used
 * @return None
 * @note After fault brake occurred, application must clear fault brake source before re-enable PWM output
 */
void PWM_ClearFaultBrakeFlag (PWM_T *pwm, uint32_t u32BrakeSource)
{
    PWM->BRKCTL = PWM_BRKCTL_BRKSTS_Msk;
}

/**
 * @brief This function enables PWM output generation of selected channels
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel.
 *                           Set bit 0 to 1 enables channel 0 output, set bit 1 to 1 enables channel 1 output...
 * @return None
 */
void PWM_EnableOutput (PWM_T *pwm, uint32_t u32ChannelMask)
{
    PWM->POEN |= u32ChannelMask;
}

/**
 * @brief This function disables PWM output generation of selected channels
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelMask Combination of enabled channels. Each bit corresponds to a channel
 *                           Set bit 0 to 1 disables channel 0 output, set bit 1 to 1 disables channel 1 output...
 * @return None
 */
void PWM_DisableOutput (PWM_T *pwm, uint32_t u32ChannelMask)
{
    PWM->POEN &= ~u32ChannelMask;
}

/**
 * @brief This function enable Dead zone of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Duration Dead Zone length in PWM clock count, valid values are between 0~0xFF, but 0 means there is no
 *                        dead zone.
 * @return None
 */
void PWM_EnableDeadZone (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Duration)
{
    // every two channels shares the same setting
    u32ChannelNum >>= 1;
    // set duration
    PWM->DTCTL = (PWM->DTCTL & ~(PWM_DTCTL_DTI01_Msk << (8 * u32ChannelNum))) | (u32Duration << (8 * u32ChannelNum));
    // enable dead zone
    PWM->CTL |= (PWM_CTL_DTCNT01_Msk << u32ChannelNum);
}

/**
 * @brief This function disable Dead zone of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_DisableDeadZone (PWM_T *pwm, uint32_t u32ChannelNum)
{
    // every two channels shares the same setting
    u32ChannelNum >>= 1;
    // enable dead zone
    PWM->CTL &= ~(PWM_CTL_DTCNT01_Msk << u32ChannelNum);
}

/**
 * @brief This function enable duty interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32IntDutyType This parameter is not used
 * @return None
 */
void PWM_EnableDutyInt (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32IntDutyType)
{
    (pwm)->INTEN |= ((1 << PWM_INTEN_CMPDIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function disable duty interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_DisableDutyInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN &= ~((1 << PWM_INTEN_CMPDIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function clears duty interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_ClearDutyIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->INTSTS = (PWM_INTSTS_CMPDIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get duty interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return Duty interrupt flag of specified channel
 * @retval 0 Duty interrupt did not occurred
 * @retval 1 Duty interrupt occurred
 */
uint32_t PWM_GetDutyIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(PWM->INTSTS & (PWM_INTSTS_CMPDIF0_Msk << u32ChannelNum) ? 1 : 0);
}

/**
 * @brief This function enable Period interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_EnablePeriodInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN |= ((1 << PWM_INTEN_ZIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function disable Period interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_DisablePeriodInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN &= ~((1 << PWM_INTEN_ZIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function clears Period interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_ClearPeriodIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->INTSTS = (PWM_INTSTS_ZIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Period interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return Period interrupt flag of specified channel
 * @retval 0 Period interrupt did not occurred
 * @retval 1 Period interrupt occurred
 */
uint32_t PWM_GetPeriodIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(PWM->INTSTS & (PWM_INTSTS_ZIF0_Msk << u32ChannelNum) ? 1 : 0);
}

/**
 * @brief This function enable Rise interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_EnableRiseInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN |= ((1 << PWM_INTEN_CMPUIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function disable Rise interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_DisableRiseInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    (pwm)->INTEN &= ~((1 << PWM_INTEN_CMPUIEN0_Pos) << u32ChannelNum);
}

/**
 * @brief This function clears Rise interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_ClearRiseIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->INTSTS = (PWM_INTSTS_CMPUIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Rise interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return Rise interrupt flag of specified channel
 * @retval 0 Rise interrupt did not occurred
 * @retval 1 Rise interrupt occurred
 */
uint32_t PWM_GetRiseIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(PWM->INTSTS & (PWM_INTSTS_CMPUIF0_Msk << u32ChannelNum) ? 1 : 0);
}

/**
 * @brief This function enable fault brake interrupt
 * @param[in] pwm The base address of PWM module
 * @param[in] u32BrakeSource This parameter is not used
 * @return None
 */
void PWM_EnableFaultBrakeInt (PWM_T *pwm, uint32_t u32BrakeSource)
{
    PWM->INTEN |= PWM_INTEN_BRKIEN_Msk;
}

/**
 * @brief This function disable fault brake interrupt
 * @param[in] pwm The base address of PWM module
 * @param[in] u32BrakeSource This parameter is not used
 * @return None
 */
void PWM_DisableFaultBrakeInt (PWM_T *pwm, uint32_t u32BrakeSource)
{
    PWM->INTEN &= ~PWM_INTEN_BRKIEN_Msk;
}

/**
 * @brief This function clear fault brake interrupt of selected source
 * @param[in] pwm The base address of PWM module
 * @param[in] u32BrakeSource Fault brake source, could be either
 *                  - \ref PWM_INTSTS_BRKIF0_Msk, or
 *                  - \ref PWM_INTSTS_BRKIF1_Msk
 * @return None
 */
void PWM_ClearFaultBrakeIntFlag (PWM_T *pwm, uint32_t u32BrakeSource)
{
    PWM->INTSTS = u32BrakeSource;
}

/**
 * @brief This function get fault brake interrupt of selected source
 * @param[in] pwm The base address of PWM module
 * @param[in] u32BrakeSource Fault brake source, could be either
 *                  - \ref PWM_INTSTS_BRKIF0_Msk, or
 *                  - \ref PWM_INTSTS_BRKIF1_Msk
 * @return Fault brake interrupt flag of specified source
 * @retval 0 Fault brake interrupt did not occurred
 * @retval 1 Fault brake interrupt occurred
 */
uint32_t PWM_GetFaultBrakeIntFlag (PWM_T *pwm, uint32_t u32BrakeSource)
{
    return (PWM->INTSTS & u32BrakeSource ? 1 : 0);
}

/**
 * @brief This function enable Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32IntPeriodType Period interrupt type, could be either
 *              - \ref PWM_PERIOD_INT_UNDERFLOW
 *              - \ref PWM_PERIOD_INT_MATCH_CNR
 * @return None
 * @note All channels share the same Central interrupt type setting.
 */
void PWM_EnableCenterInt (PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType)
{
    PWM->INTEN = (PWM->INTEN & ~PWM_INTEN_PINTTYPE_Msk & ~(PWM_INTEN_PIEN0_Msk << u32ChannelNum)) | (PWM_INTEN_PIEN0_Msk << u32ChannelNum) | u32IntPeriodType;
}

/**
 * @brief This function disable Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_DisableCenterInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->INTEN &= ~(PWM_INTEN_PIEN0_Msk << u32ChannelNum);
}

/**
 * @brief This function clear Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_ClearCenterIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->INTSTS = (PWM_INTSTS_PIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get Central interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return Central interrupt flag of specified channel
 * @retval 0 Central interrupt did not occurred
 * @retval 1 Central interrupt occurred
 */
uint32_t PWM_GetCenterIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(PWM->INTSTS & (PWM_INTSTS_PIF0_Msk << u32ChannelNum) ? 1 : 0);
}


/*@}*/ /* end of group Mini58_PWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_PWM_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
