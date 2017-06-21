/**************************************************************************//**
 * @file     NuEdu-Basic01_PWMWDAC.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/09/15 2:30p $
 * @brief    NuEdu-Basic01 PWM DAC driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_PWMDAC.h"

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS PWM DAC Exported Functions
  @{
*/


/**
  * @brief This function is used to set PWM for PWM DAC
  * @return None
  */
void Write_PWMDAC(void)
{
    /* Enable PWM clock */
    CLK_EnableModuleClock(PWM1_CH01_MODULE);
    /* Set HCLK as PWM clock source */
    CLK_SetModuleClock(PWM1_CH01_MODULE, (0x2UL<<4), 0);
    // PWM0 frequency is 100Hz, duty 30%,
    PWM_ConfigOutputChannel(PWM1, 0, 100, 30);
    // Enable PWM0 output
    PWM_EnableOutput(PWM1, 0x01);
    // Start PWM module
    PWM_Start(PWM1, 0x01);
}

/**
  * @brief This function is used to setup multi-function pin of PC12 for PWM1_CH0
  * @return None
  */
void Initial_PWM_DAC(void)
{
    SYS->PC_H_MFP = (SYS->PC_H_MFP & ~(SYS_PC_H_MFP_PC12_MFP_Msk ));
    SYS->PC_H_MFP |= SYS_PC_H_MFP_PC12_MFP_PWM1_CH0;
}

/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS PWM DAC Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
