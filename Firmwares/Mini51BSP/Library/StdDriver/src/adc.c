/**************************************************************************//**
 * @file     adc.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 13/11/07 4:40p $
 * @brief    MINI51 series ADC driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini51Series.h"

/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_ADC_Driver ADC Driver
  @{
*/


/** @addtogroup MINI51_ADC_EXPORTED_FUNCTIONS ADC Exported Functions
  @{
*/

/**
  * @brief This API configures ADC module to be ready for convert the input from selected channel
  * @param[in] adc Base address of ADC module
  * @param[in] u32InputMode This parameter is unused
  * @param[in] u32OpMode This parameter is unused
  * @param[in] u32ChMask Channel enable bit. Each bit corresponds to a input channel. Bit 0 is channel 0, bit 1 is channel 1...
  * @return  None
  * @note Mini51 series MCU ADC can only convert 1 channel at a time. If more than 1 channels are enabled, only channel
  *       with smallest number will be convert.
  * @note This API does not turn on ADC power nor does trigger ADC conversion
  */
void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask)
{

    ADC->ADCR = 0;  // A clean start.
    ADC->ADCHER  = (ADC->ADCHER & ~ADC_ADCHER_CHEN_Msk) | u32ChMask;
    return;
}

/**
  * @brief Disable ADC module
  * @param[in] adc Base address of ADC module
  * @return None
  */
void ADC_Close(ADC_T *adc)
{
    SYS->IPRSTC2 |= SYS_IPRSTC2_ADC_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_ADC_RST_Msk;
    return;

}

/**
  * @brief Configure the hardware trigger condition and enable hardware trigger
  * @param[in] adc Base address of ADC module
  * @param[in] u32Source Decides the hardware trigger source. Valid values are:
  *                 - \ref ADC_TRIGGER_BY_EXT_PIN
  *                 - \ref ADC_TRIGGER_BY_PWM
  * @param[in] u32Param While ADC trigger by PWM, this parameter is used to set the delay between PWM
  *                     trigger and ADC conversion. Valid values are from 0 ~ 0xFF, and actual delay
  *                     time is (4 * u32Param * HCLK). While ADC trigger by external pin, this parameter
  *                     is used to set trigger condition. Valid values are:
  *                 - \ref ADC_FALLING_EDGE_TRIGGER
  *                 - \ref ADC_RISING_EDGE_TRIGGER
  * @return None
  */
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param)
{
    ADC->ADCR &= ~(ADC_TRIGGER_BY_PWM | ADC_RISING_EDGE_TRIGGER | ADC_ADCR_TRGEN_Msk);
    if(u32Source == ADC_TRIGGER_BY_EXT_PIN) {
        ADC->ADCR |= u32Source | u32Param | ADC_ADCR_TRGEN_Msk;
    } else {
        ADC->ADTDCR = (ADC->ADTDCR & ~ADC_ADTDCR_PTDT_Msk) | u32Param;
        ADC->ADCR |= u32Source | ADC_ADCR_TRGEN_Msk;
    }
    return;
}

/**
  * @brief Disable hardware trigger ADC function.
  * @param[in] adc Base address of ADC module
  * @return None
  */
void ADC_DisableHWTrigger(ADC_T *adc)
{
    ADC->ADCR &= ~(ADC_TRIGGER_BY_PWM | ADC_RISING_EDGE_TRIGGER | ADC_ADCR_TRGEN_Msk);
    return;
}

/**
  * @brief Set ADC sample time for designated channel.
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum This parameter is not used
  * @param[in] u32SampleTime ADC sample ADC time, valid values are
  *                 - \ref ADC_SAMPLE_CLOCK_0
  *                 - \ref ADC_SAMPLE_CLOCK_1
  *                 - \ref ADC_SAMPLE_CLOCK_2
  *                 - \ref ADC_SAMPLE_CLOCK_4
  *                 - \ref ADC_SAMPLE_CLOCK_8
  *                 - \ref ADC_SAMPLE_CLOCK_16
  *                 - \ref ADC_SAMPLE_CLOCK_32
  *                 - \ref ADC_SAMPLE_CLOCK_64
  *                 - \ref ADC_SAMPLE_CLOCK_128
  *                 - \ref ADC_SAMPLE_CLOCK_256
  *                 - \ref ADC_SAMPLE_CLOCK_512
  *                 - \ref ADC_SAMPLE_CLOCK_1024
  * @return None
  */
void ADC_SetExtraSampleTime(ADC_T *adc,
                            uint32_t u32ChNum,
                            uint32_t u32SampleTime)
{
    ADC->ADSAMP = (ADC->ADSAMP & ~ADC_ADSAMP_SAMPCNT_Msk) | u32SampleTime;
}

/**
  * @brief Enable the interrupt(s) selected by u32Mask parameter.
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be enabled.
  *                     - \ref ADC_ADF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return None
  */
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask)
{
    if(u32Mask & ADC_ADF_INT)
        ADC->ADCR |= ADC_ADCR_ADIE_Msk;
    if(u32Mask & ADC_CMP0_INT)
        ADC->ADCMPR[0] |= ADC_ADCMPR_CMPIE_Msk;
    if(u32Mask & ADC_CMP1_INT)
        ADC->ADCMPR[1] |= ADC_ADCMPR_CMPIE_Msk;

    return;
}

/**
  * @brief Disable the interrupt(s) selected by u32Mask parameter.
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask  The combination of interrupt status bits listed below. Each bit
  *                     corresponds to a interrupt status. This parameter decides which
  *                     interrupts will be disabled.
  *                     - \ref ADC_ADF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return None
  */
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask)
{
    if(u32Mask & ADC_ADF_INT)
        ADC->ADCR &= ~ADC_ADCR_ADIE_Msk;
    if(u32Mask & ADC_CMP0_INT)
        ADC->ADCMPR[0] &= ~ADC_ADCMPR_CMPIE_Msk;
    if(u32Mask & ADC_CMP1_INT)
        ADC->ADCMPR[1] &= ~ADC_ADCMPR_CMPIE_Msk;

    return;
}



/*@}*/ /* end of group MINI51_ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_ADC_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
