/******************************************************************************
 * @file     system_I94100.c
 * @version  V0.10
 * $Revision: 1 $
 * $Date: 16/06/14 10:24a $ 
 * @brief    I94100 system clock init code and assert handler
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "I94100.h"

/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock  = __SYSTEM_CLOCK;    /*!< System Clock Frequency (Core Clock)*/
uint32_t CyclesPerUs      = 0; 				   /* Cycles per micro second */
uint32_t PllClock         = 0;                 /*!< PLL Output Clock Frequency         */
uint32_t gau32ClkSrcTbl[] = {__HXT, __LXT, 0, __LIRC, 0, 0, 0, 0};


/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
    uint32_t u32Freq, u32ClkSrc;
    uint32_t u32HclkDiv;

    /* Update PLL Clock */
    PllClock = CLK_GetPLLClockFreq();

    u32ClkSrc = CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk;

    if(u32ClkSrc != CLK_CLKSEL0_HCLKSEL_PLL)
   {
        /* Use the clock sources directly */
	    if (u32ClkSrc == CLK_CLKSEL0_HCLKSEL_HIRC)
			u32Freq = gau32ClkSrcTbl[u32ClkSrc] = __HIRC;
		else
			u32Freq = gau32ClkSrcTbl[u32ClkSrc];
   }
    else
    {
        /* Use PLL clock */
        u32Freq = PllClock;
    }
 
    u32HclkDiv = (CLK->CLKDIV0 & CLK_CLKDIV0_HCLKDIV_Msk) + 1; 
    
    /* Update System Core Clock */
    SystemCoreClock = u32Freq/u32HclkDiv;

    CyclesPerUs = (SystemCoreClock + 500000) / 1000000;
}

/**
 * Initialize the system
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system.
 *         Initialize the System.
 */
void SystemInit (void)
{
    SYS_UnlockReg();
	
    /* Disable internal POR circuit to avoid unpredictable noise to cause chip reset
	   by writing 0x5AA5 to this field */
	SYS->PORCTL = 0x00005AA5;
	
	/* Force to use GM type with HXT and gain control must be larger than L2 */
    CLK->PWRCTL = (CLK->PWRCTL & ~CLK_PWRCTL_HXTGAIN_Msk) | CLK_PWRCTL_HXTSELTYP_Msk | (0x2 << CLK_PWRCTL_HXTGAIN_Pos);
		
    SYS_LockReg();
  
  
  /* FPU settings (enable floating-point instructions) ------------------------*/
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |=  0xf<<20;               /* set CP10 and CP11 Full Access */
#endif

	
}
/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
