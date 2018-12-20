/**************************************************************************//**
 * @file     system_Mini57Series.c
 * @version  V3.0
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series CMSIS System File
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdint.h>
#include "Mini57Series.h"

#define DEBUG   0

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock  = __HIRC;             /*!< System Clock Frequency (Core Clock) */
uint32_t CyclesPerUs      = (__HIRC / 1000000); /* Cycles per micro second */
uint32_t PllClock         = __HIRC;             /*!< PLL Output Clock Frequency */
uint32_t gau32ClkSrcTbl[] = {__HXT_LXT, __LIRC, NULL, __HIRC};


/*----------------------------------------------------------------------------
  Clock functions
  This function is used to update the variable SystemCoreClock
  and must be called whenever the core clock is changed.
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate(void)
{
    uint32_t u32Freq, u32ClkSrc;
    uint32_t u32HclkDiv;

    u32ClkSrc = (CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk) >> CLK_CLKSEL0_HCLKSEL_Pos;
    u32Freq = gau32ClkSrcTbl[u32ClkSrc];
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
