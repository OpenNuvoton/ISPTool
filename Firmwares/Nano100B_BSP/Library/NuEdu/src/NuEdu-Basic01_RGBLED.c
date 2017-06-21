/**************************************************************************//**
 * @file     NuEdu-NuEdu-Basic01_RGBLED.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/12 11:15p $
 * @brief    NuEdu-Basic01_RGBLED driver source file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_RGBLED.h"


/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS Buzzer Exported Constants
  @{
*/

/**
 * @brief       Set multi-function pins for PWM0 channel 0,1,2
 * @return      None
 */
void Initial_PWM_LED(void)
{
    /* Set PA 12,13,7 multi-function pins for PWM0 channel 0,1,2*/
    SYS->PA_H_MFP = (SYS->PA_H_MFP & ~(SYS_PA_H_MFP_PA12_MFP_Msk | SYS_PA_H_MFP_PA13_MFP_Msk));
    SYS->PA_L_MFP = (SYS->PA_L_MFP & ~(SYS_PA_L_MFP_PA7_MFP_Msk));
    SYS->PA_H_MFP |= (SYS_PA_H_MFP_PA12_MFP_PWM0_CH0 | SYS_PA_H_MFP_PA13_MFP_PWM0_CH1);
    SYS->PA_L_MFP |= SYS_PA_L_MFP_PA7_MFP_PWM0_CH2;
}

/**
 * @brief       Set PWM clock enable and HCLK as PWM clock source,
 *              PWM frequency is 100Hz, duty 30% and enable output of all PWM channels
 * @return      None
 */
void PWM_LED(void)
{
    /* Enable PWM clock */
    CLK_EnableModuleClock(PWM0_CH01_MODULE);
    /* Set HCLK as PWM clock source */
    CLK_SetModuleClock(PWM0_CH01_MODULE, CLK_CLKSEL1_PWM0_CH01_S_HCLK, 0);
    CLK_SetModuleClock(PWM0_CH23_MODULE, CLK_CLKSEL1_PWM0_CH23_S_HCLK, 0);
    // PWM0 frequency is 100Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM0, 0, 100, 30);
    PWM_ConfigOutputChannel(PWM0, 1, 100, 30);
    PWM_ConfigOutputChannel(PWM0, 2, 100, 30);
    // Enable output of all PWM channels
    PWM_EnableOutput(PWM0, 0x07);
    PWM_Start(PWM0, 0x07);
}

/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS Buzzer Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */
/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. **
