/**************************************************************************//**
 * @file     system_Mini51Series.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 13/11/07 4:40p $
 * @brief    Mini51 series system clock init code and assert handler
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include <stdint.h>
#include "Mini51Series.h"


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock  = __HSI;              /*!< System Clock Frequency (Core Clock) */
uint32_t CyclesPerUs      = (__HSI / 1000000);  /*!< Cycles per micro second */


/**
  * @brief  This function is used to update the variable SystemCoreClock
  *   and must be called whenever the core clock is changed.
  * @param  None.
  * @retval None.
  */

void SystemCoreClockUpdate (void)
{
    uint32_t u32CoreFreq, u32ClkSrc;

    u32ClkSrc = CLK->CLKSEL0 & CLK_CLKSEL0_HCLK_S_Msk;

    if (u32ClkSrc == 0)
        u32CoreFreq = __XTAL;       /* External crystal clock */
    else if (u32ClkSrc == 3)
        u32CoreFreq = __IRC10K;     /* Internal 10K crystal clock */
    else if (u32ClkSrc ==  7)
        u32CoreFreq = __IRC22M;     /* Internal 22M */
    else
        u32CoreFreq = __IRC22M;     /* unknown value, use default Internal 22M */

    SystemCoreClock = (u32CoreFreq/((CLK->CLKDIV & CLK_CLKDIV_HCLK_N_Msk) + 1));
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

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
