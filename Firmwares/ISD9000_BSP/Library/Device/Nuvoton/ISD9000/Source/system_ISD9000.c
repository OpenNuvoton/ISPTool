/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2016 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "ISD9000.h"

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock = __HSI;                          /*!< System Clock Frequency (Core Clock)*/
uint32_t CyclesPerUs = (__HSI / 1000000);                  /*!< Cycles per micro second            */

/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)                          /* Get Core Clock Frequency      */
{
    uint32_t u32ClkSrc;
  
	switch(CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk)
	{
		case 0x001:	u32ClkSrc = __LXT;  break;
		case 0x010:	u32ClkSrc = __LIRC; break;
		default:    u32ClkSrc = __HIRC; break;
	}
	
    /* Update System Core Clock */
    SystemCoreClock = u32ClkSrc/((CLK->CLKDIV & CLK_CLKDIV_HCLKDIV_Msk) + 1);
    
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
