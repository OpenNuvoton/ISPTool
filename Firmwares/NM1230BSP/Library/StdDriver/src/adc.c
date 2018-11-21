/**************************************************************************//**
 * @file     adc.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 2018/10/09 14:12 $
 * @brief    NM1230 ADC driver source file
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NM1230.h"

/** @addtogroup NM1230_Device_Driver NM1230 Device Driver
  @{
*/

/** @addtogroup NM1230_ADC_Driver ADC Driver
  @{
*/


/** @addtogroup NM1230_ADC_EXPORTED_FUNCTIONS ADC Exported Functions
  @{
*/

/**
  * @brief      This function make ADC_module be ready to convert.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32InputMode Decides the input mode. This parameter is not used in NM1230.
  * @return     None.
  * @details    This function is used to set analog input mode and enable A/D Converter.
  *             Before starting A/D conversion function, ADCEN bit (ADC_CTL[0]) should be set to 1.
  */
void ADC_Open(ADC_T *adc, uint32_t u32InputMode)
{
    adc->CTL |= ADC_CTL_ADCEN_Msk;
}

/**
  * @brief      Disable ADC_module.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @return     None.
  * @details    Clear ADCEN bit (ADC_CTL[0]) to disable A/D converter analog circuit power consumption.
  */
void ADC_Close(ADC_T *adc)
{
    adc->CTL = adc->CTL
             | (ADC0SEL_VSS << ADC_CTL_ADC0CHSEL_Pos)                 /* Switching to channel Vss to save power */
             | ((ADC1SEL_VSS-ADC_ADC1) << ADC_CTL_ADC1CHSEL_Pos)  /* Switching to channel Vss to save power */
             & (~ADC_CTL_ADCEN_Msk);
    SYS_ResetModule(ADC_RST);
}

/**
  * @brief      Configure the sample control logic module.
  * @param[in]  adc The pointer of the specified ADC module.
  * @param[in]  u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref ADC0SEL_ADC0_CH0
  *                          - \ref ADC0SEL_ADC0_CH1
  *                          - \ref ADC0SEL_ADC0_CH2
  *                          - \ref ADC0SEL_ADC0_CH3
  *                          - \ref ADC0SEL_ADC0_CH4
  *                          - \ref ADC0SEL_PGA 
  *                          - \ref ADC0SEL_BAND_GAP
  *                          - \ref ADC0SEL_VSS     
  *                          - \ref ADC0SEL_OP0     
  *                          - \ref ADC0SEL_OP1     
  *                          - \ref ADC0SEL_OP2     
  *                          - \ref ADC0SEL_DAC0    
  *                          - \ref ADC0SEL_DAC1    
  *                          - \ref ADC0SEL_ADC0_CH5
  *                          - \ref ADC0SEL_ADC0_CH6
  *                          - \ref ADC0SEL_ADC0_CH7
  *                          - \ref ADC1SEL_ADC1_CH0
  *                          - \ref ADC1SEL_ADC1_CH1
  *                          - \ref ADC1SEL_ADC1_CH2
  *                          - \ref ADC1SEL_ADC0_CH0
  *                          - \ref ADC1SEL_ADC0_CH4
  *                          - \ref ADC1SEL_PGA     
  *                          - \ref ADC1SEL_TEMP_SNR
  *                          - \ref ADC1SEL_VSS     
  *                          - \ref ADC1SEL_OP0     
  *                          - \ref ADC1SEL_OP1     
  *                          - \ref ADC1SEL_OP2     
  *                          - \ref ADC1SEL_ADC1_CH3
  *                          - \ref ADC1SEL_ADC1_CH4
  *                          - \ref ADC1SEL_ADC1_CH5
  *                          - \ref ADC1SEL_ADC1_CH6
  *                          - \ref ADC1SEL_ADC1_CH7
  * @param[in] u32TriggerSrc Decides the trigger source. Valid values are:
  *                          - \ref ADC_SOFTWARE_TRIGGER
  *                          - \ref ADC_RISING_EDGE_TRIGGER
  *                          - \ref ADC_FALLING_EDGE_TRIGGER
  *                          - \ref ADC_FALLING_RISING_EDGE_TRIGGER
  *                          - \ref ADC_EPWM0_FALLING_TRIGGER
  *                          - \ref ADC_EPWM0_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM0_RISING_TRIGGER
  *                          - \ref ADC_EPWM0_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM1_FALLING_TRIGGER
  *                          - \ref ADC_EPWM1_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM1_RISING_TRIGGER
  *                          - \ref ADC_EPWM1_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM2_FALLING_TRIGGER
  *                          - \ref ADC_EPWM2_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM2_RISING_TRIGGER
  *                          - \ref ADC_EPWM2_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM3_FALLING_TRIGGER
  *                          - \ref ADC_EPWM3_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM3_RISING_TRIGGER
  *                          - \ref ADC_EPWM3_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM4_FALLING_TRIGGER
  *                          - \ref ADC_EPWM4_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM4_RISING_TRIGGER
  *                          - \ref ADC_EPWM4_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM5_FALLING_TRIGGER
  *                          - \ref ADC_EPWM5_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM5_RISING_TRIGGER
  *                          - \ref ADC_EPWM5_PERIOD_TRIGGER
  *                          - \ref ADC_TIMER0_TRIGGER
  *                          - \ref ADC_TIMER1_TRIGGER
  *                          - \ref ADC_ECAPPHG_TRIGGER
  *                          - \ref ADC_ADC0F_TRIGGER
  *                          - \ref ADC_ADC1F_TRIGGER
  * @param[in]  u32Channel  Specifies the sample module channel, valid value are from 0 to 7. This parameter is not used in NM1230.
  * @return     None.
  * @details    Each of ADC control logic modules A0~7 which is configurable for ADC converter channel ADC0_CH0~7 and trigger source.
  *             And each of ADC control logic modules B0~7 which is configurable for ADC converter channel ADC1_CH0~7 and trigger source.
  * @note       Sample module 0 channels 0~7 share the same hardware trigger configuration in ADC_TRGSOR[7:0] and sample module 1 channels 0~7 share the same hardware trigger configuration in ADC_TRGSOR[23:16].
  */
void ADC_ConfigSampleModule(ADC_T *adc,
                           uint32_t u32ModuleNum,
                           uint32_t u32TriggerSrc,
                           uint32_t u32Channel)
{
    if (u32ModuleNum <= ADC0SEL_ADC0_CH7)
    {
        adc->CTL = (adc->CTL & ~ADC_CTL_ADC0CHSEL_Msk) | (u32ModuleNum << ADC_CTL_ADC0CHSEL_Pos);
        if (u32TriggerSrc == ADC_SOFTWARE_TRIGGER)
            adc->CTL &= ~ADC_CTL_ADC0HWTRGEN_Msk;
        else
        {
            adc->TRGSOR = (adc->TRGSOR & ~(ADC_TRGSOR_ADC0TRGSOR_Msk | ADC_TRGSOR_ADC0STADCSEL_Msk | ADC_TRGSOR_ADC0PWMTRGSEL_Msk))
                         | (u32TriggerSrc << ADC_TRGSOR_ADC0TRGSOR_Pos);
            adc->CTL |= ADC_CTL_ADC0HWTRGEN_Msk;
        }
    }
    else if (u32ModuleNum <= ADC1SEL_ADC1_CH7)
    {
        adc->CTL = (adc->CTL & ~ADC_CTL_ADC1CHSEL_Msk) | ((u32ModuleNum - ADC_ADC1) << ADC_CTL_ADC1CHSEL_Pos);
        if (u32TriggerSrc == ADC_SOFTWARE_TRIGGER)
            adc->CTL &= ~ADC_CTL_ADC1HWTRGEN_Msk;
        else
        {
            adc->TRGSOR = (adc->TRGSOR & ~(ADC_TRGSOR_ADC1TRGSOR_Msk | ADC_TRGSOR_ADC1STADCSEL_Msk | ADC_TRGSOR_ADC1PWMTRGSEL_Msk))
                         | (u32TriggerSrc << ADC_TRGSOR_ADC1TRGSOR_Pos);
            adc->CTL |= ADC_CTL_ADC1HWTRGEN_Msk;
        }
    }
}

/**
  * @brief     Enable the secondary trigger source in Simultaneous mode.
  * @param[in] adc The pointer of the specified ADC module.
  * @param[in] u32TriggerSrc Decides the trigger source. Valid values are:
  *                          - \ref ADC_SOFTWARE_TRIGGER
  *                          - \ref ADC_RISING_EDGE_TRIGGER
  *                          - \ref ADC_FALLING_EDGE_TRIGGER
  *                          - \ref ADC_FALLING_RISING_EDGE_TRIGGER
  *                          - \ref ADC_EPWM0_FALLING_TRIGGER
  *                          - \ref ADC_EPWM0_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM0_RISING_TRIGGER
  *                          - \ref ADC_EPWM0_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM1_FALLING_TRIGGER
  *                          - \ref ADC_EPWM1_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM1_RISING_TRIGGER
  *                          - \ref ADC_EPWM1_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM2_FALLING_TRIGGER
  *                          - \ref ADC_EPWM2_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM2_RISING_TRIGGER
  *                          - \ref ADC_EPWM2_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM3_FALLING_TRIGGER
  *                          - \ref ADC_EPWM3_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM3_RISING_TRIGGER
  *                          - \ref ADC_EPWM3_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM4_FALLING_TRIGGER
  *                          - \ref ADC_EPWM4_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM4_RISING_TRIGGER
  *                          - \ref ADC_EPWM4_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM5_FALLING_TRIGGER
  *                          - \ref ADC_EPWM5_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM5_RISING_TRIGGER
  *                          - \ref ADC_EPWM5_PERIOD_TRIGGER
  *                          - \ref ADC_TIMER0_TRIGGER
  *                          - \ref ADC_TIMER1_TRIGGER
  *                          - \ref ADC_ECAPPHG_TRIGGER
  *                          - \ref ADC_ADC0F_TRIGGER
  *                          - \ref ADC_ADC1F_TRIGGER
  * @return     None.
  * @details    This function is used to enable secondary trigger source in Simultaneous mode.
  * @note       
  */
void ADC_EnableSecondaryTrigger(ADC_T *adc, uint32_t u32TriggerSrc)
{
    adc->CTL |= ADC_CTL_SECTRIEN_Msk;
    adc->TRGSOR = (adc->TRGSOR & ~(ADC_TRGSOR_SADC0TRGSOR_Msk | ADC_TRGSOR_SADC0STADCSEL_Msk | ADC_TRGSOR_SADC0PWMTRGSEL_Msk))
                 | (u32TriggerSrc << ADC_TRGSOR_SADC0TRGSOR_Pos);
}

/**
  * @brief     Disable the secondary trigger source in Simultaneous mode.
  * @param[in] adc    The pointer of the specified ADC module.
  * @param[in] u32TriggerSrc Decides the trigger source. Valid values are:
  *                          - \ref ADC_SOFTWARE_TRIGGER
  *                          - \ref ADC_RISING_EDGE_TRIGGER
  *                          - \ref ADC_FALLING_EDGE_TRIGGER
  *                          - \ref ADC_FALLING_RISING_EDGE_TRIGGER
  *                          - \ref ADC_EPWM0_FALLING_TRIGGER
  *                          - \ref ADC_EPWM0_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM0_RISING_TRIGGER
  *                          - \ref ADC_EPWM0_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM1_FALLING_TRIGGER
  *                          - \ref ADC_EPWM1_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM1_RISING_TRIGGER
  *                          - \ref ADC_EPWM1_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM2_FALLING_TRIGGER
  *                          - \ref ADC_EPWM2_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM2_RISING_TRIGGER
  *                          - \ref ADC_EPWM2_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM3_FALLING_TRIGGER
  *                          - \ref ADC_EPWM3_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM3_RISING_TRIGGER
  *                          - \ref ADC_EPWM3_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM4_FALLING_TRIGGER
  *                          - \ref ADC_EPWM4_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM4_RISING_TRIGGER
  *                          - \ref ADC_EPWM4_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM5_FALLING_TRIGGER
  *                          - \ref ADC_EPWM5_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM5_RISING_TRIGGER
  *                          - \ref ADC_EPWM5_PERIOD_TRIGGER
  *                          - \ref ADC_TIMER0_TRIGGER
  *                          - \ref ADC_TIMER1_TRIGGER
  *                          - \ref ADC_ECAPPHG_TRIGGER
  *                          - \ref ADC_ADC0F_TRIGGER
  *                          - \ref ADC_ADC1F_TRIGGER
  * @return     None.
  * @details    This function is used to disable secondary trigger source in Simultaneous mode.
  * @note       
  */
void ADC_DisableSecondaryTrigger(ADC_T *adc)
{
    adc->CTL &= ~ADC_CTL_SECTRIEN_Msk;
}

/**
  * @brief      Set trigger delay time.
  * @param[in]  adc The pointer of the specified ADC module.
  * @param[in]  u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref ADC_ADC0
  *                          - \ref ADC_ADC1
  * @param[in]  u32TriggerDelayTime Decides the trigger delay time, valid range are between 0~0xFF.
  * @param[in]  u32DelayClockDivider Decides the trigger delay clock divider. This parameter is not used in NM1230.
  * @return     None.
  * @details    User can configure the trigger delay time by setting ADCnDELAY (ADC_TRGDLY[7:0] and ADC_TRGDLY[23:16], n=0~1).
  *             Trigger delay time = (4 * u32TriggerDelayTime) x system clock period.
  * @note       Sample module 0 channels 0~7 share the same configuration in ADC0DELAY(ADC_TRGDLY[7:0]) and sample module 1 channels 0~7 share the same configuration in ADC1DELAY(ADC_TRGDLY[23:16]).
  */
void ADC_SetTriggerDelayTime(ADC_T *adc,
                            uint32_t u32ModuleNum,
                            uint32_t u32TriggerDelayTime,
                            uint32_t u32DelayClockDivider)
{
    if (u32ModuleNum == ADC_ADC0)
        adc->TRGDLY = (adc->TRGDLY & (~ADC_TRGDLY_ADC0DELAY_Msk)) | ((u32TriggerDelayTime & 0xFF) << ADC_TRGDLY_ADC0DELAY_Pos);
    else if (u32ModuleNum == ADC_ADC1)
        adc->TRGDLY = (adc->TRGDLY & (~ADC_TRGDLY_ADC1DELAY_Msk)) | ((u32TriggerDelayTime & 0xFF) << ADC_TRGDLY_ADC1DELAY_Pos);
}

/**
  * @brief      Set ADC sampling time.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleNum Decides the sample module number. This parameter is not used in NM1230.
  * @param[in]  u32SampleCnt Decides the extend sampling time, the range is from 1~1024 ADC clock. Valid value are from 0 to 15.
  *                     0 = 1 * ADC Clock
  *                     1 = 2 * ADC Clock
  *                     2 = 3 * ADC Clock
  *                     3 = 4 * ADC Clock
  *                     4 = 5 * ADC Clock
  *                     5 = 6 * ADC Clock
  *                     6 = 7 * ADC Clock
  *                     7 = 8 * ADC Clock
  *                     8 = 16 * ADC Clock
  *                     9 = 32 * ADC Clock
  *                     10 = 64 * ADC Clock
  *                     11 = 128 * ADC Clock
  *                     12 = 256 * ADC Clock
  *                     13 = 512 * ADC Clock
  *                     14 = 1024 * ADC Clock
  *                     15 = 1024 * ADC Clock
  * @return     None.
  * @details    When A/D converting at high conversion rate, the sampling time of analog input voltage may not enough if input channel loading is heavy,
  *             user can extend A/D sampling time after trigger source is coming to get enough sampling time.
  * @note       All sample module 0 channels 0~7 and sample module 1 channels 0~7 share the same configuration in ADCSMPCNT(ADC_SMPCNT[3:0]).
  */
void ADC_SetSampleCnt(ADC_T *adc, uint32_t u32ModuleNum, uint32_t u32SampleCnt)
{
    adc->SMPCNT = (adc->SMPCNT & (~ADC_SMPCNT_ADCSMPCNT_Msk)) | ((u32SampleCnt & 0xF) << ADC_SMPCNT_ADCSMPCNT_Pos);
}

/**
  * @brief      Configure the PWM trigger condition.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleNum Decides the sample module number. Valid values are:
  *                          - \ref ADC_ADC0
  *                          - \ref ADC_ADC1
  * @param[in] u32Source     Decides the hardware trigger source. Valid values are:
  *                          - \ref ADC_EPWM0_FALLING_TRIGGER
  *                          - \ref ADC_EPWM0_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM0_RISING_TRIGGER
  *                          - \ref ADC_EPWM0_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM1_FALLING_TRIGGER
  *                          - \ref ADC_EPWM1_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM1_RISING_TRIGGER
  *                          - \ref ADC_EPWM1_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM2_FALLING_TRIGGER
  *                          - \ref ADC_EPWM2_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM2_RISING_TRIGGER
  *                          - \ref ADC_EPWM2_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM3_FALLING_TRIGGER
  *                          - \ref ADC_EPWM3_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM3_RISING_TRIGGER
  *                          - \ref ADC_EPWM3_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM4_FALLING_TRIGGER
  *                          - \ref ADC_EPWM4_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM4_RISING_TRIGGER
  *                          - \ref ADC_EPWM4_PERIOD_TRIGGER
  *                          - \ref ADC_EPWM5_FALLING_TRIGGER
  *                          - \ref ADC_EPWM5_CENTRAL_TRIGGER
  *                          - \ref ADC_EPWM5_RISING_TRIGGER
  *                          - \ref ADC_EPWM5_PERIOD_TRIGGER
  * @param[in]  u32Param    ADC trigger by external pin, this parameter is used to set trigger condition. This parameter is not used in NM1230.
  * @return     None
  * @details    User can configure the EPWM trigger condition of sample module 0 and 1 by PWM rising edge, falling edge, period and center point.
  * @note       Sample module is triggered by PWM center point that is only when EPWM in Center-aligned mode.
  * @note       Sample module 0 channels 0~7 share the same EPWM trigger configuration in ADC_TRGSOR[5:0] and sample module 1 channels 0~7 share the same EPWM trigger configuration in ADC_TRGSOR[21:16].
  */
void ADC_EnablePWMTrigger(ADC_T *adc,
                         uint32_t u32ModuleNum,
                         uint32_t u32Source,
                         uint32_t u32Param)
{
    if (u32ModuleNum == ADC_ADC0)
    {
        adc->TRGSOR = (adc->TRGSOR & ~(ADC_TRGSOR_ADC0TRGSOR_Msk | ADC_TRGSOR_ADC0STADCSEL_Msk | ADC_TRGSOR_ADC0PWMTRGSEL_Msk))
                     | (u32Source << ADC_TRGSOR_ADC0TRGSOR_Pos);
        adc->CTL |= ADC_CTL_ADC0HWTRGEN_Msk;
    }
    else if (u32ModuleNum == ADC_ADC1)
    {
        adc->TRGSOR = (adc->TRGSOR & ~(ADC_TRGSOR_ADC1TRGSOR_Msk | ADC_TRGSOR_ADC1STADCSEL_Msk | ADC_TRGSOR_ADC1PWMTRGSEL_Msk))
                     | (u32Source << ADC_TRGSOR_ADC1TRGSOR_Pos);
        adc->CTL |= ADC_CTL_ADC1HWTRGEN_Msk;
    }
}

/**
  * @brief      Disable the PWM trigger condition for specified module.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref ADC_ADC0
  *                          - \ref ADC_ADC1
  * @param[in]  u32Source   Decides the hardware trigger source. This parameter is not used in NM1230.
  * @param[in]  u32Param    ADC trigger by external pin, this parameter is used to set trigger condition. This parameter is not used in NM1230.
  * @return     None
  * @details    User can disable the EPWM trigger condition of sample module 0 and 1. In fact, the all hardware trigger source will be disabled.
  * @note       Sample module 0 channels 0~7 share the same EPWM trigger configuration in ADC_TRGSOR[5:0] and sample module 1 channels 0~7 share the same EPWM trigger configuration in ADC_TRGSOR[21:16].
  */
void ADC_DisablePWMTrigger(ADC_T *adc,
                          uint32_t u32ModuleNum,
                          uint32_t u32Source,
                          uint32_t u32Param)
{
    if (u32ModuleNum == ADC_ADC0)
    {
        adc->TRGSOR = adc->TRGSOR & ~(ADC_TRGSOR_ADC0TRGSOR_Msk | ADC_TRGSOR_ADC0PWMTRGSEL_Msk);
        adc->CTL &= ~ADC_CTL_ADC0HWTRGEN_Msk;
    }
    else if (u32ModuleNum == ADC_ADC1)
    {
        adc->TRGSOR = adc->TRGSOR & ~(ADC_TRGSOR_ADC1TRGSOR_Msk | ADC_TRGSOR_ADC1PWMTRGSEL_Msk);
        adc->CTL &= ~ADC_CTL_ADC1HWTRGEN_Msk;
    }
}

/**
  * @brief      Disable all PWM trigger ADC function.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref ADC_ADC0
  *                          - \ref ADC_ADC1
  * @return     None
  * @details    Disable triggering of A/D conversion by EPWM.
  */
void ADC_DisableAllPWMTrigger(ADC_T *adc, uint32_t u32ModuleNum)
{
    ADC_DisablePWMTrigger(adc, u32ModuleNum, NULL, NULL);
}

/**
  * @brief      Disable ADC Window Comparator feature
  * @param[in]  adc    The pointer of the specified ADC module.
  * @return     None
  * @details    Disable ADC Window Comparator feature. Both ADC0 and ADC1 use same one Window Comparator.
  */
void ADC_DisableWCMP(ADC_T *adc)
{
    adc->WCMPCTL &= ~ADC_WCMPCTL_WCMPEN_Msk;
}

/**
  * @brief      Configure the Window Comparator feature and enable it.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32HighBound    ADC Window Comparator high bound data. The range is from 0x0 ~ 0xFFF.
  * @param[in]  u32LowBound     ADC Window Comparator low  bound data. The range is from 0x0 ~ 0xFFF.
  * @param[in]  u32FlagEN       Set the Window Comparator match condition. Valid values to combine are:
  *                     - \ref ADC_WCMP_HIGH_ENABLE   : Match count if conversion result >= High Bound.
  *                     - \ref ADC_WCMP_MIDDLE_ENABLE : Match count if conversion result <  High Bound and >= Low Bound.
  *                     - \ref ADC_WCMP_LOW_ENABLE    : Match count if conversion result <  Low Bound.
  * @param[in]  u32MatchCount   Specifies the match count setting, valid values are between 1~16
  * @param[in]  u32FlagCTL      Auto update the Window Comparator flag in ADC_STATUS register or not. Valid values are:
  *                     - \ref ADC_WCMP_FLAG_AUTO_UPDATE
  *                     - \ref ADC_WCMP_FLAG_NONE
  * @return     None
  * @details    Enable ADC Window Comparator feature. Both ADC0 and ADC1 use same one Window Comparator.
  *             For example, ADC_EnableWCMP(ADC, 3000, 100, ADC_WCMP_MIDDLE_ENABLE, 5, ADC_WCMP_FLAG_AUTO_UPDATE);
  *             means ADC will assert Window Comparator flag if conversion result is less than 3000 and
  *             greater or equal to 100 for 5 times continuously, and a compare interrupt request is generated.
  */
void ADC_EnableWCMP(ADC_T *adc,
              uint32_t u32HighBound,
              uint32_t u32LowBound,
              uint32_t u32FlagEN,
              uint32_t u32MatchCount,
              uint32_t u32FlagCTL)
{
    /* MUST disable WCMP first to reset internal compare match counter. */
    ADC_DisableWCMP(adc);

    adc->WCMPDAT = ((u32HighBound & 0xFFF) << ADC_WCMPDAT_WCMPHIGHDAT_Pos)
                  | ((u32LowBound  & 0xFFF) << ADC_WCMPDAT_WCMPLOWDAT_Pos);

    if (u32MatchCount == 16)
        u32MatchCount = 0;  /* set WCMPMCNT to 0 means count 16. */
    adc->WCMPCTL = (ADC_WCMPCTL_WCMPEN_Msk)
                  | u32FlagEN
                  | u32FlagCTL
                  | ((u32MatchCount & 0xF)  << ADC_WCMPCTL_WCMPMCNT_Pos);
}

/**
  * @brief      Reset internal compare match counter of ADC Window Comparator.
  * @param[in]  adc     The pointer of the specified ADC module.
  * @return     None
  * @details    Reset internal compare match counter of ADC Window Comparator.
  *             You should call this after Window Comparator match to make sure next compare match work.
  *             Both ADC0 and ADC1 use same one Window Comparator.
  */
void ADC_ResetWCMPCounter(ADC_T *adc)
{
    adc->WCMPCTL &= ~ADC_WCMPCTL_WCMPEN_Msk;
    adc->WCMPCTL |= ADC_WCMPCTL_WCMPEN_Msk;
}

/**
  * @brief      Get the data valid flag of the user-specified sample module.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleMask The combination of data valid status bits. Each bit corresponds to a data valid status, valid range are between 1~0xF.
  *                     Bit 0 is ADC0 DAT0, bit 1 is ADC1 DAT0, bit 2 is ADC0 DAT1, bit 3 is ADC1 DAT1 or use macro as below
  *                     - \ref ADC_BIT_MASK_ADC0
  *                     - \ref ADC_BIT_MASK_ADC1
  *                     - \ref ADC_BIT_MASK_ADC0_DAT1
  *                     - \ref ADC_BIT_MASK_ADC1_DAT1
  * @return     Return the data valid flag of the user-specified sample module.
  * @details    This macro is used to read ADCnVALID bit (ADC_DATn[]) field to get data valid flag.
  * @note       Since the valid bit will be cleared by hardware after the ADC_DATn register is read,
  *             user MUST call this function BEFORE any other functions that could to read ADC_DATn register.
  *             That includes ADC_GET_CONV_DATA() and ADC_IS_DATA_VALID().
  */
uint32_t ADC_Get_Data_Valid_Flag(ADC_T *adc, uint32_t u32ModuleMask)
{
    uint32_t reg_adc0, reg_adc1, valid_flag;

    reg_adc0 = ADC->DAT0;
    reg_adc1 = ADC->DAT1;

    valid_flag = ( ((reg_adc0 & ADC_DAT0_ADC0VALID_Msk) >> (ADC_DAT0_ADC0VALID_Pos)) |
                   ((reg_adc0 & ADC_DAT0_ADC1VALID_Msk) >> (ADC_DAT0_ADC1VALID_Pos-1)) |
                   ((reg_adc1 & ADC_DAT1_ADC0VALID_Msk) >> (ADC_DAT1_ADC0VALID_Pos-2)) |
                   ((reg_adc1 & ADC_DAT1_ADC1VALID_Msk) >> (ADC_DAT1_ADC1VALID_Pos-3)) )
                 & (u32ModuleMask);
    return (valid_flag);
}

/**
  * @brief      Get the secondary data valid flag of the user-specified sample module.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleMask The combination of data valid status bits. Each bit corresponds to a data valid status, valid range are between 1~0xF.
  *                     Bit 0 is ADC0 secondary DAT0, bit 1 is ADC1 secondary DAT0, bit 2 is ADC0 secondary DAT1, 
                        bit 3 is ADC1 secondary DAT1 or use macro as below
  *                     - \ref ADC_BIT_MASK_ADC0
  *                     - \ref ADC_BIT_MASK_ADC1
  *                     - \ref ADC_BIT_MASK_ADC0_DAT1
  *                     - \ref ADC_BIT_MASK_ADC1_DAT1
  * @return     Return the data valid flag of the user-specified sample module.
  * @details    This macro is used to read ADCnVALID bit (ADC_SECDATn) field to get data valid flag.
  * @note       Since the valid bit will be cleared by hardware after the ADC_DATn register is read,
  *             user MUST call this function BEFORE any other functions that could to read ADC_SECDATn register.
  *             That includes ADC_GET_SEC_CONV_DATA() and ADC_IS_SEC_DATA_VALID().
  */
uint32_t ADC_Get_Sec_Data_Valid_Flag(ADC_T *adc, uint32_t u32ModuleMask)
{
    uint32_t reg_adc0, reg_adc1, valid_flag;

    reg_adc0 = ADC->SECDAT0;
    reg_adc1 = ADC->SECDAT1;

    valid_flag = ( ((reg_adc0 & ADC_SECDAT0_ADC0VALID_Msk) >> (ADC_SECDAT0_ADC0VALID_Pos)) |
                   ((reg_adc0 & ADC_SECDAT0_ADC1VALID_Msk) >> (ADC_SECDAT0_ADC1VALID_Pos-1)) |
                   ((reg_adc1 & ADC_SECDAT1_ADC0VALID_Msk) >> (ADC_SECDAT1_ADC0VALID_Pos-2)) |
                   ((reg_adc1 & ADC_SECDAT1_ADC1VALID_Msk) >> (ADC_SECDAT1_ADC1VALID_Pos-3)) )
                 & u32ModuleMask;
    return (valid_flag);
}

/*@}*/ /* end of group NM1230_ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1230_ADC_Driver */

/*@}*/ /* end of group NM1230_Device_Driver */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
