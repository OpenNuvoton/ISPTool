/**************************************************************************//**
 * @file     NuEdu-Basic01_Buzzer.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/12 3:52p $
 * @brief    NuEdu-Basic01 Buzzer driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_Buzzer.h"


/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS Buzzer Exported Constants
  @{
*/
#define Buzzer_Power_ON PE6=1             ///< Control Buzzer module power on
#define Buzzer_Power_OFF PE6=0            ///< Control Buzzer module power down

/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS Buzzer Exported Constants */

/** @addtogroup Nano130_Basic01_FUNCTIONS Buzzer Exported Functions
  @{
*/

/**
 * @brief This function config PC.13 multi-function for PWM to drive Buzzer module,
                    and config PE.6 multi-function for GPIO to control Buzzer power
 * @return None
 */
void Initial_PWM_GPIO(void)
{
    GPIO_SetMode(PC, BIT13, GPIO_PMD_OUTPUT);  //Buzzer OUT
    GPIO_SetMode(PE, BIT6, GPIO_PMD_OUTPUT);   //Buzzer POWER

    /* Set PC 13 multi-function pins for PWM0 channel 0*/
    SYS->PC_H_MFP = (SYS->PC_H_MFP & ~(SYS_PC_H_MFP_PC13_MFP_Msk ));
    SYS->PC_H_MFP |= SYS_PC_H_MFP_PC13_MFP_PWM1_CH1;

}

/**
 * @brief This function enable PWM1 module clock and set clock source
                    to start Buzzer module
 * @return None
 */
void Open_Buzzer(void)
{

    Initial_PWM_GPIO();
    Buzzer_Power_ON;

    /* Enable PWM clock */
    CLK_EnableModuleClock(PWM1_CH01_MODULE);

    /* Set HCLK as PWM clock source */
    CLK_SetModuleClock(PWM1_CH01_MODULE, (0x2UL<<4), 0);

    /* Start PWM1 channel 1 */
    PWM_Start(PWM1, 0x02);
}

/**
 * @brief This function set PWM output frequence and duty to drive Buzzer module
 * @return None
 */
void Write_Buzzer(unsigned int frequence, unsigned int duty)
{

    // Config PWM1 channel 1
    PWM_ConfigOutputChannel(PWM1, 1, frequence, duty);
    // Enable PWM1 channel 1 to drive buzzer module
    PWM_EnableOutput(PWM1, 0x02);
}

/**
 * @brief This function power down the Buzzer module
 * @return None
 */
void Close_Buzzer(void)
{
    Buzzer_Power_OFF;
}


/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS Buzzer Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
