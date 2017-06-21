/**************************************************************************//**
 * @file     NuEdu-Basic01_Button.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 14/09/12 4:15p $
 * @brief    NuEdu-Basic01 Button driver source file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS Button Functions
    @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/*  GPIO for Button Definitions                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define KEY1_INPUT PD12                     /*!< GPIO PD12 for Button1 */
#define KEY2_INPUT PE12                     /*!< GPIO PE12 for Button2 */
#define KEY3_INPUT PD0                      /*!< GPIO PD0 for Button3 */
#define KEY4_INPUT PD1                      /*!< GPIO PD1 for Button4 */

/**
 * @brief       Set Button GPIO to input mode
 * @return      None
 */
void Initial_KEY_INPUT(void)
{
    GPIO_SetMode(PD, BIT12, GPIO_PMD_INPUT);
    GPIO_SetMode(PE, BIT12, GPIO_PMD_INPUT);
    GPIO_SetMode(PD, BIT0, GPIO_PMD_INPUT);
    GPIO_SetMode(PD, BIT1, GPIO_PMD_INPUT);
}
/**
  * @brief  This function get button status.
  * @return Temp Button status
  */
unsigned char Get_KEY_INPUT(void)
{
    unsigned char temp=0;
    if (KEY1_INPUT == 1)
        temp|=0x1;

    if (KEY2_INPUT == 1)
        temp|=0x2;

    if (KEY3_INPUT == 1)
        temp|=0x4;

    if (KEY4_INPUT == 1)
        temp|=0x8;
    return   temp;
}
/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
