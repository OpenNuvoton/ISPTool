/**************************************************************************//**
 * @file     NuEdu-Basic01_System.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/12/12 10:35a $
 * @brief    NuEdu-Basic01 System driver source file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_System.h"


/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS System Exported Functions
  @{
*/


/**
 * @brief This function enables HXT, LXT and LIRC clock and sets HCLK source from HXT to 42MHz.
 * @return None
 */
/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source from HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /* Enable external 12MHz HXT, 32KHz LXT and HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HXT_EN_Msk | CLK_PWRCTL_LXT_EN_Msk | CLK_PWRCTL_HIRC_EN_Msk);

    /*  Set HCLK frequency 42MHz */
    CLK_SetCoreClock(42000000);

    /* Lock protected registers */
    SYS_LockReg();

}


/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS System Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
