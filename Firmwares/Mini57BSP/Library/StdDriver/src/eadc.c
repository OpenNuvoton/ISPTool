/**************************************************************************//**
 * @file     eadc.c
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series EADC driver source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini57Series.h"

/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_EADC_Driver EADC Driver
  @{
*/


/** @addtogroup Mini57_EADC_EXPORTED_FUNCTIONS EADC Exported Functions
  @{
*/

/**
  * @brief      This function make EADC_module be ready to convert.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32InputMode Decides the input mode. This parameter is not used in Mini57.
  * @return     None.
  * @details    This function is used to set analog input mode and enable A/D Converter.
  *             Before starting A/D conversion function, ADCEN bit (EADC_CTL[0]) should be set to 1.
  */
void EADC_Open(EADC_T *eadc, uint32_t u32InputMode)
{
    eadc->CTL |= EADC_CTL_ADCEN_Msk;
}

/**
  * @brief      Disable EADC_module.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @return     None.
  * @details    Clear ADCEN bit (EADC_CTL[0]) to disable A/D converter analog circuit power consumption.
  */
void EADC_Close(EADC_T *eadc)
{
    eadc->CTL = eadc->CTL
             | (EADC_EADC0_7 << EADC_CTL_ADC0CHSEL_Pos)                 /* Switching to channel Vss to save power */
             | ((EADC_EADC1_7-EADC_EADC1_0) << EADC_CTL_ADC1CHSEL_Pos)  /* Switching to channel Vss to save power */
             & (~EADC_CTL_ADCEN_Msk);
    SYS_ResetModule(EADC_RST);
}

/**
  * @brief      Configure the sample control logic module.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_EADC0_0
  *                          - \ref EADC_EADC0_1
  *                          - \ref EADC_EADC0_2
  *                          - \ref EADC_EADC0_3
  *                          - \ref EADC_EADC0_4
  *                          - \ref EADC_EADC0_5
  *                          - \ref EADC_EADC0_6
  *                          - \ref EADC_EADC0_7
  *                          - \ref EADC_EADC1_0
  *                          - \ref EADC_EADC1_1
  *                          - \ref EADC_EADC1_2
  *                          - \ref EADC_EADC1_3
  *                          - \ref EADC_EADC1_4
  *                          - \ref EADC_EADC1_5
  *                          - \ref EADC_EADC1_6
  *                          - \ref EADC_EADC1_7
  * @param[in] u32TriggerSrc Decides the trigger source. Valid values are:
  *                          - \ref EADC_SOFTWARE_TRIGGER
  *                          - \ref EADC_RISING_EDGE_TRIGGER
  *                          - \ref EADC_FALLING_EDGE_TRIGGER
  *                          - \ref EADC_FALLING_RISING_EDGE_TRIGGER
  *                          - \ref EADC_EPWM0_FALLING_TRIGGER
  *                          - \ref EADC_EPWM0_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM0_RISING_TRIGGER
  *                          - \ref EADC_EPWM0_PERIOD_TRIGGER
  *                          - \ref EADC_EPWM1_FALLING_TRIGGER
  *                          - \ref EADC_EPWM1_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM1_RISING_TRIGGER
  *                          - \ref EADC_EPWM1_PERIOD_TRIGGER
  *                          - \ref EADC_EPWM2_FALLING_TRIGGER
  *                          - \ref EADC_EPWM2_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM2_RISING_TRIGGER
  *                          - \ref EADC_EPWM2_PERIOD_TRIGGER
  *                          - \ref EADC_EPWM3_FALLING_TRIGGER
  *                          - \ref EADC_EPWM3_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM3_RISING_TRIGGER
  *                          - \ref EADC_EPWM3_PERIOD_TRIGGER
  *                          - \ref EADC_EPWM4_FALLING_TRIGGER
  *                          - \ref EADC_EPWM4_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM4_RISING_TRIGGER
  *                          - \ref EADC_EPWM4_PERIOD_TRIGGER
  *                          - \ref EADC_EPWM5_FALLING_TRIGGER
  *                          - \ref EADC_EPWM5_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM5_RISING_TRIGGER
  *                          - \ref EADC_EPWM5_PERIOD_TRIGGER
  *                          - \ref EADC_TIMER0_TRIGGER
  *                          - \ref EADC_TIMER1_TRIGGER
  *                          - \ref EADC_TIMER2_TRIGGER
  *                          - \ref EADC_ADC0F_TRIGGER
  *                          - \ref EADC_ADC1F_TRIGGER
  * @param[in]  u32Channel  Specifies the sample module channel, valid value are from 0 to 7. This parameter is not used in Mini57.
  * @return     None.
  * @details    Each of EADC control logic modules A0~7 which is configurable for EADC converter channel ADC0_CH0~7 and trigger source.
  *             And each of EADC control logic modules B0~7 which is configurable for EADC converter channel ADC1_CH0~7 and trigger source.
  * @note       Sample module 0 channels 0~7 share the same hardware trigger configuration in EADC_TRGSOR[7:0] and sample module 1 channels 0~7 share the same hardware trigger configuration in EADC_TRGSOR[23:16].
  */
void EADC_ConfigSampleModule(EADC_T *eadc,
                             uint32_t u32ModuleNum,
                             uint32_t u32TriggerSrc,
                             uint32_t u32Channel)
{
    if (u32ModuleNum <= EADC_EADC0_7)
    {
        eadc->CTL = (eadc->CTL & ~EADC_CTL_ADC0CHSEL_Msk) | (u32ModuleNum << EADC_CTL_ADC0CHSEL_Pos);
        if (u32TriggerSrc == EADC_SOFTWARE_TRIGGER)
            eadc->CTL &= ~EADC_CTL_ADC0HWTRGEN_Msk;
        else
        {
            eadc->TRGSOR = (eadc->TRGSOR & ~(EADC_TRGSOR_ADC0TRGSOR_Msk | EADC_TRGSOR_ADC0STADCSEL_Msk | EADC_TRGSOR_ADC0PWMTRGSEL_Msk))
                         | (u32TriggerSrc << EADC_TRGSOR_ADC0TRGSOR_Pos);
            eadc->CTL |= EADC_CTL_ADC0HWTRGEN_Msk;
        }
    }
    else if (u32ModuleNum <= EADC_EADC1_7)
    {
        eadc->CTL = (eadc->CTL & ~EADC_CTL_ADC1CHSEL_Msk) | ((u32ModuleNum - EADC_EADC1_0) << EADC_CTL_ADC1CHSEL_Pos);
        if (u32TriggerSrc == EADC_SOFTWARE_TRIGGER)
            eadc->CTL &= ~EADC_CTL_ADC1HWTRGEN_Msk;
        else
        {
            eadc->TRGSOR = (eadc->TRGSOR & ~(EADC_TRGSOR_ADC1TRGSOR_Msk | EADC_TRGSOR_ADC1STADCSEL_Msk | EADC_TRGSOR_ADC1PWMTRGSEL_Msk))
                         | (u32TriggerSrc << EADC_TRGSOR_ADC1TRGSOR_Pos);
            eadc->CTL |= EADC_CTL_ADC1HWTRGEN_Msk;
        }
    }
}

/**
  * @brief      Set trigger delay time.
  * @param[in]  eadc The pointer of the specified EADC module.
  * @param[in]  u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_EADC0
  *                          - \ref EADC_EADC1
  * @param[in]  u32TriggerDelayTime Decides the trigger delay time, valid range are between 0~0xFF.
  * @param[in]  u32DelayClockDivider Decides the trigger delay clock divider. This parameter is not used in Mini57.
  * @return     None.
  * @details    User can configure the trigger delay time by setting ADCnDELAY (EADC_TRGDLY[7:0] and EADC_TRGDLY[23:16], n=0~1).
  *             Trigger delay time = (4 * u32TriggerDelayTime) x system clock period.
  * @note       Sample module 0 channels 0~7 share the same configuration in ADC0DELAY(EADC_TRGDLY[7:0]) and sample module 1 channels 0~7 share the same configuration in ADC1DELAY(EADC_TRGDLY[23:16]).
  */
void EADC_SetTriggerDelayTime(EADC_T *eadc,
                              uint32_t u32ModuleNum,
                              uint32_t u32TriggerDelayTime,
                              uint32_t u32DelayClockDivider)
{
    if (u32ModuleNum == EADC_EADC0)
        eadc->TRGDLY = (eadc->TRGDLY & (~EADC_TRGDLY_ADC0DELAY_Msk)) | ((u32TriggerDelayTime & 0xFF) << EADC_TRGDLY_ADC0DELAY_Pos);
    else if (u32ModuleNum == EADC_EADC1)
        eadc->TRGDLY = (eadc->TRGDLY & (~EADC_TRGDLY_ADC1DELAY_Msk)) | ((u32TriggerDelayTime & 0xFF) << EADC_TRGDLY_ADC1DELAY_Pos);
}

/**
  * @brief      Set EADC extend sample time.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleNum Decides the sample module number. This parameter is not used in Mini57.
  * @param[in]  u32ExtendSampleTime Decides the extend sampling time, the range is from 1~1024 EADC clock. Valid value are from 0 to 15.
  *                     0 = 1 * EADC Clock
  *                     1 = 2 * EADC Clock
  *                     2 = 3 * EADC Clock
  *                     3 = 4 * EADC Clock
  *                     4 = 5 * EADC Clock
  *                     5 = 6 * EADC Clock
  *                     6 = 7 * EADC Clock
  *                     7 = 8 * EADC Clock
  *                     8 = 16 * EADC Clock
  *                     9 = 32 * EADC Clock
  *                     10 = 64 * EADC Clock
  *                     11 = 128 * EADC Clock
  *                     12 = 256 * EADC Clock
  *                     13 = 512 * EADC Clock
  *                     14 = 1024 * EADC Clock
  *                     15 = 1024 * EADC Clock
  * @return     None.
  * @details    When A/D converting at high conversion rate, the sampling time of analog input voltage may not enough if input channel loading is heavy,
  *             user can extend A/D sampling time after trigger source is coming to get enough sampling time.
  * @note       All sample module 0 channels 0~7 and sample module 1 channels 0~7 share the same configuration in ADCSMPCNT(EADC_SMPCNT[3:0]).
  */
void EADC_SetExtendSampleTime(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32ExtendSampleTime)
{
    eadc->SMPCNT = (eadc->SMPCNT & (~EADC_SMPCNT_ADCSMPCNT_Msk)) | ((u32ExtendSampleTime & 0xF) << EADC_SMPCNT_ADCSMPCNT_Pos);
}

/**
  * @brief      Configure the PWM trigger condition.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleNum Decides the sample module number. Valid values are:
  *                          - \ref EADC_EADC0
  *                          - \ref EADC_EADC1
  * @param[in] u32Source     Decides the hardware trigger source. Valid values are:
  *                          - \ref EADC_EPWM0_FALLING_TRIGGER
  *                          - \ref EADC_EPWM0_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM0_RISING_TRIGGER
  *                          - \ref EADC_EPWM0_PERIOD_TRIGGER
  *                          - \ref EADC_EPWM1_FALLING_TRIGGER
  *                          - \ref EADC_EPWM1_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM1_RISING_TRIGGER
  *                          - \ref EADC_EPWM1_PERIOD_TRIGGER
  *                          - \ref EADC_EPWM2_FALLING_TRIGGER
  *                          - \ref EADC_EPWM2_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM2_RISING_TRIGGER
  *                          - \ref EADC_EPWM2_PERIOD_TRIGGER
  *                          - \ref EADC_EPWM3_FALLING_TRIGGER
  *                          - \ref EADC_EPWM3_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM3_RISING_TRIGGER
  *                          - \ref EADC_EPWM3_PERIOD_TRIGGER
  *                          - \ref EADC_EPWM4_FALLING_TRIGGER
  *                          - \ref EADC_EPWM4_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM4_RISING_TRIGGER
  *                          - \ref EADC_EPWM4_PERIOD_TRIGGER
  *                          - \ref EADC_EPWM5_FALLING_TRIGGER
  *                          - \ref EADC_EPWM5_CENTRAL_TRIGGER
  *                          - \ref EADC_EPWM5_RISING_TRIGGER
  *                          - \ref EADC_EPWM5_PERIOD_TRIGGER
  * @param[in]  u32Param    EADC trigger by external pin, this parameter is used to set trigger condition. This parameter is not used in Mini57.
  * @return     None
  * @details    User can configure the EPWM trigger condition of sample module 0 and 1 by PWM rising edge, falling edge, period and center point.
  * @note       Sample module is triggered by PWM center point that is only when EPWM in Center-aligned mode.
  * @note       Sample module 0 channels 0~7 share the same EPWM trigger configuration in EADC_TRGSOR[5:0] and sample module 1 channels 0~7 share the same EPWM trigger configuration in EADC_TRGSOR[21:16].
  */
void EADC_EnablePWMTrigger(EADC_T *eadc,
                           uint32_t u32ModuleNum,
                           uint32_t u32Source,
                           uint32_t u32Param)
{
    if (u32ModuleNum == EADC_EADC0)
    {
        eadc->TRGSOR = (eadc->TRGSOR & ~(EADC_TRGSOR_ADC0TRGSOR_Msk | EADC_TRGSOR_ADC0STADCSEL_Msk | EADC_TRGSOR_ADC0PWMTRGSEL_Msk))
                     | (u32Source << EADC_TRGSOR_ADC0TRGSOR_Pos);
        eadc->CTL |= EADC_CTL_ADC0HWTRGEN_Msk;
    }
    else if (u32ModuleNum == EADC_EADC1)
    {
        eadc->TRGSOR = (eadc->TRGSOR & ~(EADC_TRGSOR_ADC1TRGSOR_Msk | EADC_TRGSOR_ADC1STADCSEL_Msk | EADC_TRGSOR_ADC1PWMTRGSEL_Msk))
                     | (u32Source << EADC_TRGSOR_ADC1TRGSOR_Pos);
        eadc->CTL |= EADC_CTL_ADC1HWTRGEN_Msk;
    }
}

/**
  * @brief      Disable the PWM trigger condition for specified module.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_EADC0
  *                          - \ref EADC_EADC1
  * @param[in]  u32Source   Decides the hardware trigger source. This parameter is not used in Mini57.
  * @param[in]  u32Param    EADC trigger by external pin, this parameter is used to set trigger condition. This parameter is not used in Mini57.
  * @return     None
  * @details    User can disable the EPWM trigger condition of sample module 0 and 1. In fact, the all hardware trigger source will be disabled.
  * @note       Sample module 0 channels 0~7 share the same EPWM trigger configuration in EADC_TRGSOR[5:0] and sample module 1 channels 0~7 share the same EPWM trigger configuration in EADC_TRGSOR[21:16].
  */
void EADC_DisablePWMTrigger(EADC_T *eadc,
                            uint32_t u32ModuleNum,
                            uint32_t u32Source,
                            uint32_t u32Param)
{
    if (u32ModuleNum == EADC_EADC0)
    {
        eadc->TRGSOR = eadc->TRGSOR & ~(EADC_TRGSOR_ADC0TRGSOR_Msk | EADC_TRGSOR_ADC0PWMTRGSEL_Msk);
        eadc->CTL &= ~EADC_CTL_ADC0HWTRGEN_Msk;
    }
    else if (u32ModuleNum == EADC_EADC1)
    {
        eadc->TRGSOR = eadc->TRGSOR & ~(EADC_TRGSOR_ADC1TRGSOR_Msk | EADC_TRGSOR_ADC1PWMTRGSEL_Msk);
        eadc->CTL &= ~EADC_CTL_ADC1HWTRGEN_Msk;
    }
}

/**
  * @brief      Disable all PWM trigger EADC function.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_EADC0
  *                          - \ref EADC_EADC1
  * @return     None
  * @details    Disable triggering of A/D conversion by EPWM.
  */
void EADC_DisableAllPWMTrigger(EADC_T *eadc, uint32_t u32ModuleNum)
{
    EADC_DisablePWMTrigger(eadc, u32ModuleNum, NULL, NULL);
}

/**
  * @brief      Disable EADC Window Comparator feature
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @return     None
  * @details    Disable EADC Window Comparator feature. Both EADC0 and EADC1 use same one Window Comparator.
  */
void EADC_DisableWCMP(EADC_T *eadc)
{
    eadc->WCMPCTL &= ~EADC_WCMPCTL_WCMPEN_Msk;
}

/**
  * @brief      Configure the Window Comparator feature and enable it.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32HighBound    EADC Window Comparator high bound data. The range is from 0x0 ~ 0xFFF.
  * @param[in]  u32LowBound     EADC Window Comparator low  bound data. The range is from 0x0 ~ 0xFFF.
  * @param[in]  u32FlagEN       Set the Window Comparator match condition. Valid values to combine are:
  *                     - \ref EADC_WCMP_HIGH_ENABLE   : Match count if conversion result >= High Bound.
  *                     - \ref EADC_WCMP_MIDDLE_ENABLE : Match count if conversion result <  High Bound and >= Low Bound.
  *                     - \ref EADC_WCMP_LOW_ENABLE    : Match count if conversion result <  Low Bound.
  * @param[in]  u32MatchCount   Specifies the match count setting, valid values are between 1~16
  * @param[in]  u32FlagCTL      Auto update the Window Comparator flag in EADC_STATUS register or not. Valid values are:
  *                     - \ref EADC_WCMP_FLAG_AUTO_UPDATE
  *                     - \ref EADC_WCMP_FLAG_NONE
  * @return     None
  * @details    Enable EADC Window Comparator feature. Both EADC0 and EADC1 use same one Window Comparator.
  *             For example, EADC_EnableWCMP(EADC, 3000, 100, EADC_WCMP_MIDDLE_ENABLE, 5, EADC_WCMP_FLAG_AUTO_UPDATE);
  *             means EADC will assert Window Comparator flag if conversion result is less than 3000 and
  *             greater or equal to 100 for 5 times continuously, and a compare interrupt request is generated.
  */
void EADC_EnableWCMP(EADC_T *eadc,
              uint32_t u32HighBound,
              uint32_t u32LowBound,
              uint32_t u32FlagEN,
              uint32_t u32MatchCount,
              uint32_t u32FlagCTL)
{
    /* MUST disable WCMP first to reset internal compare match counter. */
    EADC_DisableWCMP(eadc);

    eadc->WCMPDAT = ((u32HighBound & 0xFFF) << EADC_WCMPDAT_WCMPHIGHDAT_Pos)
                  | ((u32LowBound  & 0xFFF) << EADC_WCMPDAT_WCMPLOWDAT_Pos);

    if (u32MatchCount == 16)
        u32MatchCount = 0;  /* set WCMPMCNT to 0 means count 16. */
    eadc->WCMPCTL = (EADC_WCMPCTL_WCMPEN_Msk)
                  | u32FlagEN
                  | u32FlagCTL
                  | ((u32MatchCount & 0xF)  << EADC_WCMPCTL_WCMPMCNT_Pos);
}

/**
  * @brief      Reset internal compare match counter of EADC Window Comparator.
  * @param[in]  eadc     The pointer of the specified EADC module.
  * @return     None
  * @details    Reset internal compare match counter of EADC Window Comparator.
  *             You should call this after Window Comparator match to make sure next compare match work.
  *             Both EADC0 and EADC1 use same one Window Comparator.
  */
void EADC_ResetWCMPCounter(EADC_T *eadc)
{
    eadc->WCMPCTL &= ~EADC_WCMPCTL_WCMPEN_Msk;
    eadc->WCMPCTL |= EADC_WCMPCTL_WCMPEN_Msk;
}

/**
  * @brief      Get the data valid flag of the user-specified sample module.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleMask The combination of data valid status bits. Each bit corresponds to a data valid status, valid range are between 1~0xF.
  *                     Bit 0 is EADC0 DAT0, bit 1 is EADC1 DAT0, bit 2 is EADC0 DAT1, bit 3 is EADC1 DAT1 or use macro as below
  *                     - \ref EADC_BIT_MASK_EADC0
  *                     - \ref EADC_BIT_MASK_EADC1
  *                     - \ref EADC_BIT_MASK_EADC0_DAT1
  *                     - \ref EADC_BIT_MASK_EADC1_DAT1
  * @return     Return the data valid flag of the user-specified sample module.
  * @details    This macro is used to read ADCnVALID bit (EADC_DATn[]) field to get data valid flag.
  * @note       Since the valid bit will be cleared by hardware after the EADC_DATn register is read,
  *             user MUST call this function BEFORE any other functions that could to read EADC_DATn register.
  *             That includes EADC_GET_CONV_DATA() and EADC_IS_DATA_VALID().
  */
uint32_t EADC_Get_Data_Valid_Flag(EADC_T *eadc, uint32_t u32ModuleMask)
{
    uint32_t reg_eadc0, reg_eadc1, valid_flag;

    reg_eadc0 = EADC->DAT[0];
    reg_eadc1 = EADC->DAT[1];

    valid_flag = ( ((reg_eadc0 & EADC_DAT0_ADC0VALID_Msk) >> (EADC_DAT0_ADC0VALID_Pos)) |
                   ((reg_eadc0 & EADC_DAT0_ADC1VALID_Msk) >> (EADC_DAT0_ADC1VALID_Pos-1)) |
                   ((reg_eadc1 & EADC_DAT1_ADC0VALID_Msk) >> (EADC_DAT1_ADC0VALID_Pos-2)) |
                   ((reg_eadc1 & EADC_DAT1_ADC1VALID_Msk) >> (EADC_DAT1_ADC1VALID_Pos-3)) )
                 & u32ModuleMask;
    return (valid_flag);
}

/*@}*/ /* end of group Mini57_EADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_EADC_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
