/**************************************************************************//**
 * @file     NuEdu-Basic01_Interrupt.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/12/12 10:35a $
 * @brief    NuEdu-Basic01_Interrupt.c interrupt driver source file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NUC200Series.h"
#include "NuEdu-Basic01_Interrupt.h"


/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS Buzzer Exported Functions
  @{
*/

/**
 * @brief This function initializes PB.15 multi-function for EINT1 to detect external interrupt.
 * @return None
 */
void Initial_EINT1_GPIO(void)
{
    /* Set PB.15 as Input */
    _GPIO_SET_PIN_MODE(PB, 15, GPIO_PMD_INPUT);

    /* Set PB.15 multi-function pins for EINT1 */
    SYS->GPB_MFP |= SYS_GPB_MFP_PB15_INT1;
}

/**
 * @brief This function enables EINT1 interrupt and IRQ handler, and configs trigger condition for falling edge.
 * @return None
 */
void Open_EINT1(void)
{
    Initial_EINT1_GPIO();

    /* Enable interrupt by falling edge trigger */
    GPIO_EnableInt(PB, 15, GPIO_INT_FALLING);
    NVIC_EnableIRQ(EINT1_IRQn);

}


/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS Interrupt Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
