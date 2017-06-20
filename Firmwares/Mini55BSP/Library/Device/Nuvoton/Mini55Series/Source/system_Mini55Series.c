/**************************************************************************//**
 * @file     system_Mini55Series.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/06/29 11:16a $
 * @brief    Mini55 series system clock init code and assert handler
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdint.h>
#include "Mini55Series.h"


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t gau32HIRCTbl[4] = {__IRC48M, __IRC48M_DIV2, __IRC44M, __IRC44M_DIV2};
uint32_t __HSI = __IRC44M_DIV2;                 /*!< Factory Default is internal high speed RC 44.2368M divided by 2 */
uint32_t SystemCoreClock;                       /*!< System Clock Frequency (Core Clock) */
uint32_t CyclesPerUs;                           /*!< Cycles per micro second */

/**
 *  @brief  Check HIRC clock rate feed to HCLK
 *
 *  @return none
 */

void SystemInit (void)
{
    uint32_t u32CoreFreq;

    /* Read the User Configuration words. */
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
    FMC->ISPCMD = FMC_ISPCMD_READ;
    FMC->ISPADDR = FMC_CONFIG_BASE;
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
    while (FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk) ;

    u32CoreFreq = (FMC->ISPDAT >> 27) & 0x1;  // 0 : Internal 48M  1: Internal 44M
    u32CoreFreq = (u32CoreFreq << 1);
    u32CoreFreq |= (FMC->ISPDAT >> 15) & 0x1; // 1: divided by 2
    __HSI = gau32HIRCTbl[u32CoreFreq];

}

/**
  * @brief  This function is used to update the variable SystemCoreClock
  *   and must be called whenever the core clock is changed.
  * @param  None.
  * @retval None.
  */

void SystemCoreClockUpdate (void)
{
    uint32_t u32CoreFreq, u32ClkSrc;

    u32ClkSrc = CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk;

    if (u32ClkSrc == 0)
        u32CoreFreq = __XTAL;       /* External crystal clock */
    else if (u32ClkSrc == 3)
        u32CoreFreq = __IRC10K;     /* Internal 10K crystal clock */
    else if (u32ClkSrc ==  7)
        u32CoreFreq = __HSI;        /* Factory Default is internal 44M divided by 2 */
    else
        u32CoreFreq = __HSI;        /* unknown value, use Factory Default is internal 44M divided by 2 */

    SystemCoreClock = (u32CoreFreq/((CLK->CLKDIV & CLK_CLKDIV_HCLKDIV_Msk) + 1));
    CyclesPerUs = (SystemCoreClock + 500000) / 1000000;
}

#if USE_ASSERT

/**
 * @brief      Assert Error Message
 *
 * @param[in]  file  the source file name
 * @param[in]  line  line number
 *
 * @return     None
 *
 * @details    The function prints the source file name and line number where
 *             the ASSERT_PARAM() error occurs, and then stops in an infinite loop.
 */
void AssertError(uint8_t * file, uint32_t line)
{

    printf("[%s] line %d : wrong parameters.\r\n", file, line);

    /* Infinite loop */
    while(1) ;
}
#endif

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
