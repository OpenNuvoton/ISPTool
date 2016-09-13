/**************************************************************************//**
 * @file     eadc.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 15/04/01 3:00p $
 * @brief    EADC driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NM1530.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup EADC_Driver EADC Driver
  @{
*/

/** @addtogroup EADC_EXPORTED_FUNCTIONS EADC Exported Functions
  @{
*/

/**
  * @brief This function make EADC_module be ready to convert.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32InputMode Decides the input mode. This parameter is not used.
  * @return None.
  * @details This function is used to set analog input mode and enable A/D Converter.
  *          Before starting A/D conversion function, AD_EN bit (ADCR[0]) should be set to 1.
  */
void EADC_Open(EADC_T *eadc, uint32_t u32InputMode)
{
    eadc->ADCR |= EADC_ADCR_AD_EN_Msk;
}

/**
  * @brief Disable EADC_module.
  * @param[in] eadc The pointer of the specified EADC module..
  * @return None.
  * @details Clear AD_EN bit (ADCR[0]) to disable A/D converter analog circuit power consumption.
  */
void EADC_Close(EADC_T *eadc)
{
    eadc->ADCR &= ~EADC_ADCR_AD_EN_Msk;
}

/**
  * @brief Configure the sample control logic module.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_SMPA0
  *                          - \ref EADC_SMPA1
  *                          - \ref EADC_SMPA2
  *                          - \ref EADC_SMPA3
  *                          - \ref EADC_SMPA4
  *                          - \ref EADC_SMPA5
  *                          - \ref EADC_SMPA6
  *                          - \ref EADC_SMPA7
  *                          - \ref EADC_SMPB0
  *                          - \ref EADC_SMPB1
  *                          - \ref EADC_SMPB2
  *                          - \ref EADC_SMPB3
  *                          - \ref EADC_SMPB4
  *                          - \ref EADC_SMPB5
  *                          - \ref EADC_SMPB6
  *                          - \ref EADC_SMPB7
  * @param[in] u32TriggerSrc Decides the trigger source. Valid values are:
  *                           - \ref EADC_SOFTWARE_TRIGGER              : Disable hardware trigger
  *                           - \ref EADC_FALLING_EDGE_TRIGGER          : STADC pin falling edge trigger
  *                           - \ref EADC_RISING_EDGE_TRIGGER           : STADC pin rising edge trigger
  *                           - \ref EADC_FALLING_RISING_EDGE_TRIGGER   : STADC pin both falling and rising edge trigger
  *                           - \ref EADC_ADINT0_TRIGGER                : EADC ADINT0 interrupt EOC pulse trigger
  *                           - \ref EADC_ADINT1_TRIGGER                : EADC ADINT1 interrupt EOC pulse trigger
  *                           - \ref EADC_TIMER0_TRIGGER                : Timer0 overflow pulse trigger
  *                           - \ref EADC_TIMER1_TRIGGER                : Timer1 overflow pulse trigger
  *                           - \ref EADC_TIMER2_TRIGGER                : Timer2 overflow pulse trigger
  *                           - \ref EADC_TIMER3_TRIGGER                : Timer3 overflow pulse trigger
  *                           - \ref EADC_PWM00_TRIGGER                 : PWM00 trigger (EADC_SMPAn and EADC_SMPBn, n=4~7 aren't support this configuration)
  *                           - \ref EADC_PWM02_TRIGGER                 : PWM02 trigger (EADC_SMPAn and EADC_SMPBn, n=4~7 aren't support this configuration)
  *                           - \ref EADC_PWM04_TRIGGER                 : PWM04 trigger (EADC_SMPAn and EADC_SMPBn, n=4~7 aren't support this configuration)
  *                           - \ref EADC_PWM10_TRIGGER                 : PWM10 trigger (EADC_SMPAn and EADC_SMPBn, n=4~7 aren't support this configuration)
  *                           - \ref EADC_PWM12_TRIGGER                 : PWM12 trigger (EADC_SMPAn and EADC_SMPBn, n=4~7 aren't support this configuration)
  *                           - \ref EADC_PWM14_TRIGGER                 : PWM14 trigger (EADC_SMPAn and EADC_SMPBn, n=4~7 aren't support this configuration)
  *                           - \ref EADC_PWM20_TRIGGER                 : PWM20 trigger (EADC_SMPAn and EADC_SMPBn, n=4~7 aren't support this configuration)
  *                           - \ref EADC_PWM21_TRIGGER                 : PWM21 trigger (EADC_SMPAn and EADC_SMPBn, n=4~7 aren't support this configuration)
  * @param[in] u32Channel Specifies the sample module channel, valid value are from 0 to 7.
  * @return None.
  * @details Each of EADC control logic modules A0~7 which is configurable for EADC converter channel ADC0_CH0~7 and trigger source.
  *          And each of EADC control logic modules B0~7 which is configurable for EADC converter channel ADC1_CH0~7 and trigger source.
  */
void EADC_ConfigSampleModule(EADC_T *eadc, \
                             uint32_t u32ModuleNum, \
                             uint32_t u32TriggerSrc, \
                             uint32_t u32Channel)
{
    *(__IO uint32_t *)(&((eadc)->ADSPCRA[0]) + (u32ModuleNum)) &= ~(EADC_ADSPCR_EXTFEN_Msk | EADC_ADSPCR_EXTREN_Msk | EADC_ADSPCR_TRGSEL_Msk | EADC_ADSPCR_CHSEL_Msk);
    *(__IO uint32_t *)(&((eadc)->ADSPCRA[0]) + (u32ModuleNum)) |= (u32TriggerSrc | u32Channel);
}

/**
  * @brief Set trigger delay time.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_SMPA0
  *                          - \ref EADC_SMPA1
  *                          - \ref EADC_SMPA2
  *                          - \ref EADC_SMPA3
  *                          - \ref EADC_SMPB0
  *                          - \ref EADC_SMPB1
  *                          - \ref EADC_SMPB2
  *                          - \ref EADC_SMPB3
  * @param[in] u32TriggerDelayTime Decides the trigger delay time, valid range are between 0~0xFF.
  * @param[in] u32DelayClockDivider Decides the trigger delay clock divider. Valid values are:
  *                                  - \ref EADC_ADSPCR_TRGDLYDIV_1    : Trigger delay clock frequency is EADC_CLK/1
  *                                  - \ref EADC_ADSPCR_TRGDLYDIV_2    : Trigger delay clock frequency is EADC_CLK/2
  *                                  - \ref EADC_ADSPCR_TRGDLYDIV_4    : Trigger delay clock frequency is EADC_CLK/4
  *                                  - \ref EADC_ADSPCR_TRGDLYDIV_16   : Trigger delay clock frequency is EADC_CLK/16
  * @return None.
  * @details User can configure the trigger delay time by setting TRGDLYCNT (ADSPCRAn[15:8] and ADSPCRBn[15:8], n=0~3)\n
  *          and TRGDLYDIV (ADSPCRAn[7:6] and ADSPCRBn[7:6], n=0~3).
  *          Trigger delay time = (u32TriggerDelayTime) x Trigger delay clock period.
  */
void EADC_SetTriggerDelayTime(EADC_T *eadc, \
                              uint32_t u32ModuleNum, \
                              uint32_t u32TriggerDelayTime, \
                              uint32_t u32DelayClockDivider)
{
    *(__IO uint32_t *)(&((eadc)->ADSPCRA[0]) + (u32ModuleNum)) &= ~(EADC_ADSPCR_TRGDLYDIV_Msk | EADC_ADSPCR_TRGDLYCNT_Msk);
    *(__IO uint32_t *)(&((eadc)->ADSPCRA[0]) + (u32ModuleNum)) |= ((u32TriggerDelayTime << EADC_ADSPCR_TRGDLYCNT_Pos) | u32DelayClockDivider);
}

/**
  * @brief Set EADC extend sample time.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_ADTCR_SMPA
  *                          - \ref EADC_ADTCR_SMPB
  * @param[in] u32ExtendSampleTime Decides the extend sampling time, the range is from 0~255 EADC clock. Valid value are from 0 to 0xFF.
  * @return None.
  * @details When A/D converting at high conversion rate, the sampling time of analog input voltage may not enough if input channel loading is heavy,
  *          user can extend A/D sampling time after trigger source is coming to get enough sampling time.
  * @note Sample module A0~7 share the same configuration in ADAEST(ADTCR[7:0]) and Sample module B0~7 share the same configuration in ADBEST(ADTCR[23:16]).
  */
void EADC_SetExtendSampleTime(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32ExtendSampleTime)
{
    eadc->ADTCR &= ~(EADC_ADTCR_ADAEST_Msk << (u32ModuleNum));
    eadc->ADTCR |= u32ExtendSampleTime << (u32ModuleNum);
}

/**
  * @brief Configure the PWM trigger condition.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_SMPTRG_A0
  *                          - \ref EADC_SMPTRG_A1
  *                          - \ref EADC_SMPTRG_A2
  *                          - \ref EADC_SMPTRG_A3
  *                          - \ref EADC_SMPTRG_B0
  *                          - \ref EADC_SMPTRG_B1
  *                          - \ref EADC_SMPTRG_B2
  *                          - \ref EADC_SMPTRG_B3
  * @param[in] u32Source Decides the hardware trigger source. Valid values are:
  *                       - \ref EADC_SMPTRG_PWM00           :A/D conversion is started by PWM00.
  *                       - \ref EADC_SMPTRG_PWM02           :A/D conversion is started by PWM02.
  *                       - \ref EADC_SMPTRG_PWM04           :A/D conversion is started by PWM04.
  *                       - \ref EADC_SMPTRG_PWM10           :A/D conversion is started by PWM10.
  *                       - \ref EADC_SMPTRG_PWM12           :A/D conversion is started by PWM12.
  *                       - \ref EADC_SMPTRG_PWM14           :A/D conversion is started by PWM14.
  *                       - \ref EADC_SMPTRG_PWM20           :A/D conversion is started by PWM20.
  *                       - \ref EADC_SMPTRG_PWM21           :A/D conversion is started by PWM21.
  * @param[in] u32Param EADC trigger by external pin, this parameter is used to set trigger condition. Valid values are:
  *                      - \ref EADC_TRGCOND_RISING_EDGE     :PWM Rising edge active.
  *                      - \ref EADC_TRGCOND_FALLING_EDGE    :PWM Falling edge active.
  *                      - \ref EADC_TRGCOND_PERIOD          :PWM Period active.
  *                      - \ref EADC_TRGCOND_CENTER          :PWM Center active.
  * @return None
  * @details User can configure the PWM trigger condition of sample module An and Bn(n=0~3) by PWM rising edge, falling edge, period and center point.
  * @note Sample module is triggered by PWM center point that is only when PWM in Center-aligned mode.
  */
void EADC_EnablePWMTrigger(EADC_T *eadc,
                           uint32_t u32ModuleNum,
                           uint32_t u32Source,
                           uint32_t u32Param)
{
    *(__IO uint32_t *)(&((eadc)->SMPTRGA[0]) + (u32ModuleNum)) |= ((u32Source) << (u32Param));
}

/**
  * @brief Disable the PWM trigger condition for specified module.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_SMPTRG_A0
  *                          - \ref EADC_SMPTRG_A1
  *                          - \ref EADC_SMPTRG_A2
  *                          - \ref EADC_SMPTRG_A3
  *                          - \ref EADC_SMPTRG_B0
  *                          - \ref EADC_SMPTRG_B1
  *                          - \ref EADC_SMPTRG_B2
  *                          - \ref EADC_SMPTRG_B3
  * @param[in] u32Source Decides the hardware trigger source. Valid values are:
  *                       - \ref EADC_SMPTRG_PWM00           :A/D conversion is started by PWM00.
  *                       - \ref EADC_SMPTRG_PWM02           :A/D conversion is started by PWM02.
  *                       - \ref EADC_SMPTRG_PWM04           :A/D conversion is started by PWM04.
  *                       - \ref EADC_SMPTRG_PWM10           :A/D conversion is started by PWM10.
  *                       - \ref EADC_SMPTRG_PWM12           :A/D conversion is started by PWM12.
  *                       - \ref EADC_SMPTRG_PWM14           :A/D conversion is started by PWM14.
  *                       - \ref EADC_SMPTRG_PWM20           :A/D conversion is started by PWM20.
  *                       - \ref EADC_SMPTRG_PWM21           :A/D conversion is started by PWM21.
  * @param[in] u32Param EADC trigger by external pin, this parameter is used to set trigger condition. Valid values are:
  *                      - \ref EADC_TRGCOND_RISING_EDGE     :PWM Rising edge active.
  *                      - \ref EADC_TRGCOND_FALLING_EDGE    :PWM Falling edge active.
  *                      - \ref EADC_TRGCOND_PERIOD          :PWM Period active.
  *                      - \ref EADC_TRGCOND_CENTER          :PWM Center active.
  * @return None
  * @details User can disable the PWM trigger condition of sample module An and Bn(n=0~3).
  */
void EADC_DisablePWMTrigger(EADC_T *eadc,
                            uint32_t u32ModuleNum,
                            uint32_t u32Source,
                            uint32_t u32Param)
{
    *(__IO uint32_t *)(&((eadc)->SMPTRGA[0]) + (u32ModuleNum)) &= ~((u32Source) << (u32Param));
}

/**
  * @brief Disable all PWM trigger EADC function.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_SMPTRG_A0
  *                          - \ref EADC_SMPTRG_A1
  *                          - \ref EADC_SMPTRG_A2
  *                          - \ref EADC_SMPTRG_A3
  *                          - \ref EADC_SMPTRG_B0
  *                          - \ref EADC_SMPTRG_B1
  *                          - \ref EADC_SMPTRG_B2
  *                          - \ref EADC_SMPTRG_B3
  * @return None
  * @details  Disable triggering of A/D conversion by PWM.
  */
void EADC_DisableAllPWMTrigger(EADC_T *eadc, uint32_t u32ModuleNum)
{
    *(__IO uint32_t *)(&((eadc)->SMPTRGA[0]) + (u32ModuleNum)) = 0;
}

/*@}*/ /* end of group EADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group EADC_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
