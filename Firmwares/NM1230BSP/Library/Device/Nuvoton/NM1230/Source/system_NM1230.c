/**************************************************************************//**
 * @file     system_NM1230.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 2018/07/15 16:31 $
 * @brief    NM1230 Series CMSIS System File
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdint.h>
#include "NM1230.h"

#define DEBUG   0

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t __HIRC           = 48000000;             /* HIRC default is 48MHz */
uint32_t SystemCoreClock  = 48000000;             /*!< System Clock Frequency (Core Clock) */
uint32_t CyclesPerUs      = (48000000 / 1000000); /* Cycles per micro second */


/*----------------------------------------------------------------------------
  Clock functions
  This function is used to update the variable SystemCoreClock and __HIRC
  and must be called whenever the core clock is changed.
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate(void)
{
    uint32_t u32Freq, u32ClkSrc;
    uint32_t u32HclkDiv;

    __HIRC = (CLK->PWRCTL & CLK_PWRCTL_HIRC_SEL_Msk) ? FREQ_72MHZ : FREQ_48MHZ;
    u32ClkSrc = (CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk) >> CLK_CLKSEL0_HCLKSEL_Pos;
    if (u32ClkSrc == 0)
    {
        u32Freq = __HXT_LXT;
    }
    else if (u32ClkSrc == 1)
    {
        u32Freq = __LIRC;
    }
    else if (u32ClkSrc == 3)
    {
        u32Freq = __HIRC;
    }
    else
    {
        u32Freq = NULL;
    }
    u32HclkDiv = ((CLK->CLKDIV & CLK_CLKDIV_HCLKDIV_Msk) >> CLK_CLKDIV_HCLKDIV_Pos) + 1;

    /* Update System Core Clock */
    SystemCoreClock = u32Freq / u32HclkDiv;

    CyclesPerUs = (SystemCoreClock + 500000) / 1000000;
#if DEBUG
    printf("SystemCoreClockUpdate(): u32ClkSrc=%d, u32Freq=%d, u32HclkDiv=%d, SystemCoreClock=%d\n",
        u32ClkSrc, u32Freq, u32HclkDiv, SystemCoreClock);
#endif
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: SystemInit                                                                                    */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*      None                                                                                               */
/*                                                                                                         */
/* Returns:                                                                                                */
/*      None                                                                                               */
/*                                                                                                         */
/* Description:                                                                                            */
/*      The necessary initialization of system.                                                           */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void SystemInit(void)
{
}
/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
