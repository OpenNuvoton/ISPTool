/**************************************************************************//**
 * @file     PWM.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 14/01/20 10:06a $
 * @brief    MINI51 series PWM driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini51Series.h"

/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_PWM_Driver PWM Driver
  @{
*/


/** @addtogroup MINI51_PWM_EXPORTED_FUNCTIONS PWM Exported Functions
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
    PWM->PPR = (PWM->PPR & ~(PWM_PPR_CP01_Msk << ((u32ChannelNum >> 1) * 8))) | (u8Prescale << ((u32ChannelNum >> 1) * 8));
    PWM->CSR = (PWM->CSR & ~(PWM_CSR_CSR0_Msk << (4 * u32ChannelNum))) | (u8Divider << (4 * u32ChannelNum));
    PWM->PCR = (PWM->PCR & ~PWM_PCR_PWMTYPE_Msk) | (PWM_PCR_CH0MOD_Msk << (4 * u32ChannelNum));
    if(u32DutyCycle == 0)
        PWM->CMR[u32ChannelNum] = 0;
    else
        PWM->CMR[u32ChannelNum] = u32DutyCycle * (u16CNR + 1) / 100 - 1;
    PWM->CNR[u32ChannelNum] = u16CNR;

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
            u32Mask |= (PWM_PCR_CH0EN_Msk << (i * 4));
        }
    }

    PWM->PCR |= u32Mask;
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
            PWM->CNR[i] = 0;
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
            u32Mask |= (PWM_PCR_CH0EN_Msk << (i * 4));
        }
    }

    PWM->PCR &= ~u32Mask;
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
        PWM->TRGCON0 = (PWM->TRGCON0 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                          PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                          PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                          PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * u32ChannelNum))) | (u32Condition << (8 * u32ChannelNum));
    } else {
        PWM->TRGCON1 = (PWM->TRGCON1 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
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
        PWM->TRGCON0 = (PWM->TRGCON0 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                          PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                          PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                          PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * u32ChannelNum)));
    } else {
        PWM->TRGCON1 = (PWM->TRGCON1 & ~((PWM_TRIGGER_ADC_CNTR_IS_0 |
                                          PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                                          PWM_TRIGGER_ADC_CNTR_IS_CNR |
                                          PWM_TRIGGER_ADC_CNTR_IS_CMR_U ) << (8 * (u32ChannelNum - 4))));
    }
}

/**
 * @brief This function clear selected channel trigger ADC flag
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32Condition PWM triggered ADC flag to be cleared. A combination of following flags:
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_0
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CMR_D
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CNR
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CMR_U
 * @return None
 */
void PWM_ClearADCTriggerFlag (PWM_T *pwm, uint32_t u32ChannelNum, uint32_t u32Condition)
{
    if(u32ChannelNum < 4) {
        PWM->TRGSTS0 |= (u32Condition << (8 * u32ChannelNum));
    } else {
        PWM->TRGSTS1 |= (u32Condition << (8 * (u32ChannelNum - 4)));
    }
}

/**
 * @brief This function get selected channel trigger ADC flag
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return Combination of following trigger conditions which triggered ADC
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_0
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CMR_D
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CNR
 *                  - \ref PWM_TRIGGER_ADC_CNTR_IS_CMR_U
 */
uint32_t PWM_GetADCTriggerFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    uint32_t u32Ret;

    if(u32ChannelNum < 4) {
        u32Ret = PWM->TRGSTS0 >> (8 * u32ChannelNum);
    } else {
        u32Ret = PWM->TRGSTS1 >> (8 * (u32ChannelNum - 4 ));
    }

    return (u32Ret & (PWM_TRIGGER_ADC_CNTR_IS_0 |
                      PWM_TRIGGER_ADC_CNTR_IS_CMR_D |
                      PWM_TRIGGER_ADC_CNTR_IS_CNR |
                      PWM_TRIGGER_ADC_CNTR_IS_CMR_U));
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
    PWM->PFBCON = (u32LevelMask << PWM_PFBCON_PWMBKO0_Pos) | u32BrakeSource;
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
    PWM->PFBCON = PWM_PFBCON_BKF_Msk;
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
    PWM->POE |= u32ChannelMask;
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
    PWM->POE &= ~u32ChannelMask;
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
    PWM->PDZIR = (PWM->PDZIR & ~(PWM_DZIR_DZI01_Msk << (8 * u32ChannelNum))) | (u32Duration << (8 * u32ChannelNum));
    // enable dead zone
    PWM->PCR |= (PWM_PCR_DZEN01_Msk << u32ChannelNum);
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
    PWM->PCR &= ~(PWM_PCR_DZEN01_Msk << u32ChannelNum);
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
    PWM->PIER |= (PWM_PIER_PWMDIE0_Msk << u32ChannelNum);
}

/**
 * @brief This function disable duty interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_DisableDutyInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->PIER &= ~(PWM_PIER_PWMDIE0_Msk << u32ChannelNum);
}

/**
 * @brief This function clears duty interrupt flag of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_ClearDutyIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->PIIR = (PWM_PIIR_PWMDIF0_Msk << u32ChannelNum);
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
    return(PWM->PIIR & (PWM_PIIR_PWMDIF0_Msk << u32ChannelNum) ? 1 : 0);
}

/**
 * @brief This function enable fault brake interrupt
 * @param[in] pwm The base address of PWM module
 * @param[in] u32BrakeSource This parameter is not used
 * @return None
 */
void PWM_EnableFaultBrakeInt (PWM_T *pwm, uint32_t u32BrakeSource)
{
    PWM->PIER |= PWM_PIER_BRKIE_Msk;
}

/**
 * @brief This function disable fault brake interrupt
 * @param[in] pwm The base address of PWM module
 * @param[in] u32BrakeSource This parameter is not used
 * @return None
 */
void PWM_DisableFaultBrakeInt (PWM_T *pwm, uint32_t u32BrakeSource)
{
    PWM->PIER &= ~PWM_PIER_BRKIE_Msk;
}

/**
 * @brief This function clear fault brake interrupt of selected source
 * @param[in] pwm The base address of PWM module
 * @param[in] u32BrakeSource Fault brake source, could be either
 *                  - \ref PWM_PIIR_BKF0_Msk, or
 *                  - \ref PWM_PIIR_BKF1_Msk
 * @return None
 */
void PWM_ClearFaultBrakeIntFlag (PWM_T *pwm, uint32_t u32BrakeSource)
{
    PWM->PIIR = u32BrakeSource;
}

/**
 * @brief This function get fault brake interrupt of selected source
 * @param[in] pwm The base address of PWM module
 * @param[in] u32BrakeSource Fault brake source, could be either
 *                  - \ref PWM_PIIR_BKF0_Msk, or
 *                  - \ref PWM_PIIR_BKF1_Msk
 * @return Fault brake interrupt flag of specified source
 * @retval 0 Fault brake interrupt did not occurred
 * @retval 1 Fault brake interrupt occurred
 */
uint32_t PWM_GetFaultBrakeIntFlag (PWM_T *pwm, uint32_t u32BrakeSource)
{
    return (PWM->PIIR & u32BrakeSource ? 1 : 0);
}

/**
 * @brief This function enable period interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @param[in] u32IntPeriodType Period interrupt type, could be either
 *              - \ref PWM_PERIOD_INT_UNDERFLOW
 *              - \ref PWM_PERIOD_INT_MATCH_CNR
 * @return None
 * @note All channels share the same period interrupt type setting.
 */
void PWM_EnablePeriodInt (PWM_T *pwm, uint32_t u32ChannelNum,  uint32_t u32IntPeriodType)
{
    PWM->PIER = (PWM->PIER & ~PWM_PIER_INT_TYPE_Msk) | (PWM_PIER_PWMPIE0_Msk << u32ChannelNum) | u32IntPeriodType;
}

/**
 * @brief This function disable period interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_DisablePeriodInt (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->PIER &= ~(PWM_PIER_PWMPIE0_Msk << u32ChannelNum);
}

/**
 * @brief This function clear period interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return None
 */
void PWM_ClearPeriodIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    PWM->PIIR = (PWM_PIIR_PWMPIF0_Msk << u32ChannelNum);
}

/**
 * @brief This function get period interrupt of selected channel
 * @param[in] pwm The base address of PWM module
 * @param[in] u32ChannelNum PWM channel number. Valid values are between 0~5
 * @return Period interrupt flag of specified channel
 * @retval 0 Period interrupt did not occurred
 * @retval 1 Period interrupt occurred
 */
uint32_t PWM_GetPeriodIntFlag (PWM_T *pwm, uint32_t u32ChannelNum)
{
    return(PWM->PIIR & (PWM_PIIR_PWMPIF0_Msk << u32ChannelNum) ? 1 : 0);
}



/*@}*/ /* end of group MINI51_PWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_PWM_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
