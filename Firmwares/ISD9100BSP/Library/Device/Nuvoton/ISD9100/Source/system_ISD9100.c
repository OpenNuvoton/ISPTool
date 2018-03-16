/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2014 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "ISD9100.h"

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __HSI;                          /*!< System Clock Frequency (Core Clock)*/
uint32_t CyclesPerUs = (__HSI / 1000000);                  /*!< Cycles per micro second            */
uint32_t gau32ClkSrcTbl[] = {__HIRC, __LXT, __LIRC};               /*!< System clock source table */
uint32_t gau32HiRCSrcTbl[] = {__HIRC, 32768000};           /*!< OSC48M Frequency Select table */

/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)                          /* Get Core Clock Frequency      */
{
    uint32_t u32ClkSrc;
    uint32_t u32HclkDiv;
  
    u32ClkSrc = gau32ClkSrcTbl[(CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk)];

    if(u32ClkSrc == __HIRC)
    {
        /* Use the clock sources OSC48M oscillator */
        u32ClkSrc = gau32HiRCSrcTbl[(CLK->CLKSEL0 & CLK_CLKSEL0_HIRCFSEL_Msk)>>CLK_CLKSEL0_HIRCFSEL_Pos];
    } 

    u32HclkDiv = (CLK->CLKDIV0 & CLK_CLKDIV0_HCLKDIV_Msk) + 1;

    /* Update System Core Clock */
    SystemCoreClock = u32ClkSrc/u32HclkDiv;
    
    CyclesPerUs = (SystemCoreClock) / 1000000;
}

/**
 * Initialize the system
 * @return none
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
  
}
