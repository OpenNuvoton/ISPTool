/****************************************************************************//**
 * @file     NuEdu-Basic01_Timer_Ouput_Capture.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/09/18 11:58a $
 * @brief    Nano100 series ACMP Threashold driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuEdu-Basic01_Timer_Ouput_Capture.h"

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS Timer_Ouput_Capture driver Functions
  @{
*/

/**
  * @brief This function is used to TM2 output and TMR3 capture function
  * @return none
  */
void Initial_Timer_port(void)
{


    /* Set PB multi-function pins for TM2  external counter pins;  TMR3 external capture pins */
    SYS->GPB_MFP |= SYS_GPB_MFP_PB10_TM2 |SYS_GPB_MFP_PB3_T3EX;

    /* Set ALT MPF settings for TMR0 ~ TMR3 external counter and capture functions */
    SYS->ALT_MFP |= SYS_ALT_MFP_PB10_TM2|SYS_ALT_MFP_PB3_T3EX;

    /* Set ALT MPF1 settings for TMR3 external capture */
    SYS->ALT_MFP1 |= SYS_ALT_MFP1_PB3_T3EX;
}

/**
  * @brief This function is enable TM2 toggle
  * @return none
  */
void initial_Timer_Toggle(void)
{
    /* Enable IP clock */

    SYSCLK->APBCLK |= SYSCLK_APBCLK_TMR2_EN_Msk;

    SYSCLK->CLKSEL1 |= SYSCLK_CLKSEL1_TMR2_HCLK;

    /* Reset and stop , TIMER2   counting first */

    _TIMER_RESET(TIMER2);

    /* Configure TCMP values of TIMER0 and TIMER3 */
    TIMER2->TCMPR = (__XTAL/1000);  // For 1000Hz (Equal to 500Hz toggle rate)

    /* Start TIMER0 counting and output T0 frequency - 500 Hz*/
    TIMER2->TCSR = TIMER_TCSR_CEN_Msk | TIMER_TCSR_MODE_TOGGLE | TIMER_TCSR_TDR_EN_Msk | TIMER_TCSR_PRESCALE(1);
}

/**
  * @brief This function is enable TM2 count
  * @return none
  */
void initial_Timer_count(void)
{
    /* Enable IP clock */
    SYSCLK->APBCLK |= SYSCLK_APBCLK_TMR2_EN_Msk;

    SYSCLK->CLKSEL1 |= SYSCLK_CLKSEL1_TMR2_HCLK;

    /* Reset and stop , TIMER2   counting first */

    _TIMER_RESET(TIMER2);

    /* Configure TCMP values of TIMER0 and TIMER3 */
    TIMER2->TCMPR = (__XTAL/1000);  // For 1000Hz (Equal to 500Hz toggle rate)

    /* Start TIMER0 counting and output T0 frequency - 500 Hz*/
    TIMER2->TCSR = TIMER_TCSR_CEN_Msk | TIMER_TCSR_MODE_TOGGLE | TIMER_TCSR_TDR_EN_Msk |TIMER_TCSR_CTB_ENABLE | TIMER_TCSR_PRESCALE(1);
}

/**
  * @brief This function is enable TM3 Capture
  * @return none
  */
void initial_Timer_capture(void)
{
    /* Enable IP clock */
    SYSCLK->APBCLK |= SYSCLK_APBCLK_TMR3_EN_Msk;

    SYSCLK->CLKSEL1 |= SYSCLK_CLKSEL1_TMR3_HCLK;

    /* Reset and stop , TIMER2   counting first */

    _TIMER_RESET(TIMER3);
    /* Enable TIMER1 counter and capture function */
    TIMER1->TCMPR = 0xFFFFFF;
    TIMER1->TCSR = TIMER_TCSR_CEN_Msk | TIMER_TCSR_IE_Msk | TIMER_TCSR_MODE_CONTINUOUS | TIMER_TCSR_CTB_ENABLE |
                   TIMER_TCSR_TDR_EN_Msk | TIMER_TCSR_PRESCALE(1);
    TIMER1->TEXCON = TIMER_TEXCON_MODE_CAP | TIMER_TEXCON_TEXIEN_ENABLE | TIMER_TEXCON_TEXEN_ENABLE;
}
/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */