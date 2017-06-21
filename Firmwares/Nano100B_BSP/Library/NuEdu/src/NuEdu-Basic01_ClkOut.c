/**************************************************************************//**
 * @file     NuEdu-Basic01_ClkOut.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/09/12 9:49a $
 * @brief    NuEdu-Basic01 ClkOut driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_ClkOut.h"


/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS ClkOut Exported Functions
  @{
*/

/**
  * @brief  This function enable frequency divider module clock,
  *         enable frequency divider clock function and configure frequency divider.
  * @param[in]  Clock_Source is ClkOut function clock source
  *         - \ref CLK_CLKSEL2_FRQDIV_S_HXT
  *         - \ref CLK_CLKSEL2_FRQDIV_S_LXT
  *         - \ref CLK_CLKSEL2_FRQDIV_S_HCLK
  *         - \ref CLK_CLKSEL2_FRQDIV_S_HIRC
  * @param[in]  FRQDIV_FSEL is divider output frequency selection
  * @return None
  *
  * @details    Output selected clock to FCLKO. The output clock frequency is divided by FRQDIV_FSEL.
  *             The formula is:
  *                 FCLKO frequency = (Clock_Source frequency) / 2^(FRQDIV_FSEL + 1)
  */
void Open_CLK_OUT(uint32_t Clock_Source, uint32_t FRQDIV_FSEL)
{

    //Initial FCLKO Function Pin
    SYS->PB_H_MFP = (SYS->PB_H_MFP & ~SYS_PB_H_MFP_PB12_MFP_Msk) | SYS_PB_H_MFP_PB12_MFP_CKO;

    if(FRQDIV_FSEL>15) {
        printf("\nOpen CLK OUT FAIL\n");
        return;
    }

    //Initial FCLKO Clock Divider
    CLK->FRQDIV = CLK_FRQDIV_FDIV_EN_Msk | FRQDIV_FSEL;

    /* Select FCLKO clock source */
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_FRQDIV_S_Msk)) | Clock_Source;

    /* Enable FCLKO clock source */
    CLK->APBCLK |= CLK_APBCLK_FDIV_EN_Msk;

    SYS_LockReg();

}

/**
  * @brief  This function disable frequency output function.
  * @return None
  */
void Close_CLK_OUT(void)
{

    //Disable FCLKO Function
    CLK->FRQDIV &= ~CLK_FRQDIV_FDIV_EN_Msk;

    //Disable FCLKO Clock Source
    SYS_UnlockReg();
    CLK->APBCLK &= ~CLK_APBCLK_FDIV_EN_Msk;
    SYS_LockReg();
}


/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS ClkOut Exported Functions */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library */

/*@}*/ /* end of group NANO100_Library NANO100 Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
