/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2016 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdint.h>
#include "Platform.h"

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
	uint32_t u32Div = (CLK->CLKDIV0&CLK_CLKDIV0_HCLKDIV_Msk)+1;
	
	/* Update System Core Clock */
	switch((CLK->CLKSEL0&CLK_CLKSEL0_HCLKSEL_Msk))
	{
		case 1:  SystemCoreClock = __LXT/u32Div;  break;
		case 2:  SystemCoreClock = __LIRC/u32Div; break;
		case 3:  SystemCoreClock = __HXT/u32Div;  break;
		default:
			if( ((CLK->CLKSEL0&CLK_CLKSEL0_HIRCFSEL_Msk)>>CLK_CLKSEL0_HIRCFSEL_Pos)==1 )
				SystemCoreClock = __HIRC_32M/u32Div;
			else
				SystemCoreClock = __HIRC_48M/u32Div;
			break;	
	}
    CyclesPerUs = SystemCoreClock/1000000;
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
