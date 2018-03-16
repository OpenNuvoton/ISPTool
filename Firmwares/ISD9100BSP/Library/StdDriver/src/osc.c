/**************************************************************************//**
 * @file     OSC.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/09 10:00a $
 * @brief    ISD9100 Series OSC driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "ISD9100.h"

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/
	
/** @addtogroup ISD9100_OSC_Driver OSC Driver
  @{
*/
	
/** @addtogroup ISD9100_OSC_EXPORTED_FUNCTIONS OSC Exported Functions
  @{
*/

const uint16_t OSC_TC_TABLE[2][8] = 
{
    (1+(15<<3)), (1+(15<<3)), (1+(15<<3)), (1+(15<<3)), (1+( 8<<3)), (1+( 8<<3)), (1+( 8<<3)), (1+( 8<<3)),
    (1+(15<<3)), (1+(15<<3)), (1+(15<<3)), (1+(15<<3)), (1+(15<<3)), (1+(15<<3)), (1+(15<<3)), (1+(15<<3))
};

#if(OSC_NUM_SFINE == 4)
    #define OSC_SFINE(x)  (((x)==3) ? 0xE : (x) )
    #define OSC_RSFINE(x) (((x)==0xE) ? 3 : (x) )
#else
    #define OSC_SFINE(x)  (((x)==7) ? 0xf :((x)==6) ? 0xE : ((x)==5) ? 0x7 :((x)==4) ? 0x6 :(x) )
    #define OSC_RSFINE(x) (((x)==0xf) ? 7 :((x)==0xe) ? 6 : ((x)==7) ? 5   :((x)==6) ? 4 :(x) )
#endif

#define OSC_ABS(x)        (((x)>=0) ? (x):(-x) )

/**
  * @brief  This function is to measure the internal oscillator frequency against
  *         32.768 kHz crystal referecne
  * @param  i32HCLKDiv is the current division of OSC48M to HCLK.
  * @param  u32Cycles is the number of 32k cycles to measure HCLK.
  * @return Measurement frequency(if == 0, measure timeout.)
  */
int32_t OSC_Measure( int32_t i32HCLKDiv, uint32_t u32Cycles )
{
	uint32_t u32TimeCounter = 0, u32Frequency;	
	/* Set frequescy cycle count */
    ANA->FQMMCYC = u32Cycles - 1;
	/* Start to measure frequency */
    ANA->FQMMCTL |= ANA_FQMMCTL_FQMMEN_Msk;	
	/* Frequency measurement(Timeout will return 0) */ 
	while( (ANA->FQMMCTL&ANA_FQMMCTL_MMSTS_Msk) == 0 )
	{
		if( (u32TimeCounter++) >= 0x10000 )
			return 0;
	}
	/* Stop to measure frequency */
    ANA->FQMMCTL &= (~ANA_FQMMCTL_FQMMEN_Msk);
	/* Get frequency counter from register */ 
	u32Frequency = ANA->FQMMCNT;
	/* return measure frequency value */
	return (int32_t)(u32Frequency*(uint32_t)(32768/8)*(uint32_t)i32HCLKDiv/(uint32_t)u32Cycles*(uint32_t)8);
}

/**
  * @brief  This function is to trim the oscillator to a desired frequency.
  * @param  i32Target is the target frequency.
  * @param  u8TrimIdx is the index(0/1) of trim.
  * @return Trim frequency(if == 0, trim fail.)
  */
int32_t OSC_Trim(int32_t i32Target, uint8_t u8TrimIdx) 
{
	uint8_t  u8Fine, u8Coarse, u8MaxFine, u8MinFine;
	int32_t  i32Freq, i32LastFreq;
	uint32_t u32CurFrqSel, u32CurHCLKDiv, u32CurHCLKSel;

	/* Store current configuration */
	u32CurFrqSel = ((CLK->CLKSEL0&CLK_CLKSEL0_HIRCFSEL_Msk)>>CLK_CLKSEL0_HIRCFSEL_Pos);
	u32CurHCLKDiv = ((CLK->CLKDIV0&CLK_CLKDIV0_HCLKDIV_Msk)>>CLK_CLKDIV0_HCLKDIV_Pos);
	u32CurHCLKSel = ((CLK->CLKSEL0&CLK_CLKSEL0_HCLKSEL_Msk)>>CLK_CLKSEL0_HCLKSEL_Pos);
	
	/* Turn on analog peripheral clock */
	CLK->APBCLK0 |= CLK_APBCLK0_ANACKEN_Msk;
	/* Osc trim and measurement test... Select reference source as 32kHz XTAL input */	 
	ANA->FQMMCTL = ( (ANA->FQMMCTL&(~ANA_FQMMCTL_CLKSEL_Msk)) | (1<<ANA_FQMMCTL_CLKSEL_Pos) );
    ANA->FQMMCTL &= (~ANA_FQMMCTL_FQMMEN_Msk);
	SYS_UnlockReg();
	
	/* Set HCLK divide to ensure oscillator does not violate access timing */
	CLK->CLKDIV0 = ( (CLK->CLKDIV0&(~CLK_CLKDIV0_HCLKDIV_Msk)) | ((OSC_MEASURE_HCL_DIV-1)<<CLK_CLKDIV0_HCLKDIV_Pos) );
	CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
	/* Select trim source */
	CLK->CLKSEL0 = ( (CLK->CLKSEL0&(~CLK_CLKSEL0_HCLKSEL_Msk)) | (u8TrimIdx<<CLK_CLKSEL0_HCLKSEL_Pos) );
	/* Select fine configuration */
	*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_FINE_Msk)) | (OSC_TRIM_FINE_100R<<SYS_IRCTTRIM_FINE_Pos) );
	
    /* Make sure slowest frequency is lower than target, best resolution at bottom of range. */
	#if( OSC_LOWEST_FRQ_THAN_TARGET_FRQ == 1 )
	*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) &= (~SYS_IRCTTRIM_RANGE_Msk);
	u8Fine = OSC_RESERVE_RANGE;
	u8Coarse = 0;
	*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_TRIM_Msk)) | ((u8Fine + u8Coarse*(OSC_MAX_FINE+1))<<SYS_IRCTTRIM_TRIM_Pos) );
	i32Freq = OSC_Measure( OSC_MEASURE_HCL_DIV, OSC_NUM_CYCLES);
	*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_TC_Msk)) | (OSC_TC_TABLE[0][u8Coarse]<<SYS_IRCTTRIM_TC_Pos) );
    if(i32Freq > i32Target)
        *(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_RANGE_Msk)) | (1<<SYS_IRCTTRIM_RANGE_Pos) );
	/* Make sure fastest frequency is higher than target, better TC at top of range. */
	#else
	*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_RANGE_Msk)) | (1<<SYS_IRCTTRIM_RANGE_Pos) );
	u8Fine = OSC_MAX_FINE-OSC_RESERVE_RANGE;
	u8Coarse = OSC_MAX_COARSE;	
	*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_TRIM_Msk)) | ((u8Fine + u8Coarse*(OSC_MAX_FINE+1))<<SYS_IRCTTRIM_TRIM_Pos) );
	i32Freq = OSC_Measure( OSC_MEASURE_HCL_DIV, OSC_NUM_CYCLES);
	*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_TC_Msk)) | (OSC_TC_TABLE[0][u8Coarse]<<SYS_IRCTTRIM_TC_Pos) );
	if(i32Freq < i32Target)
		*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) &= (~SYS_IRCTTRIM_RANGE_Msk);
	#endif

    /* There are 32 (OSC_MAX_FINE+1) fine settings and 8 coarse ranges. 
       Find suitable coarse range, the range where measured frequency first exceeds target. */
    u8Fine = OSC_MAX_FINE - OSC_RESERVE_RANGE;
    u8Coarse = 0;
    do
    {
    	*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_TRIM_Msk)) | ((u8Fine + u8Coarse*(OSC_MAX_FINE+1))<<SYS_IRCTTRIM_TRIM_Pos) );
        *(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_TC_Msk)) | (OSC_TC_TABLE[((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_RANGE_Msk))>>SYS_IRCTTRIM_RANGE_Pos][u8Coarse]<<SYS_IRCTTRIM_TC_Pos) );
        i32Freq = OSC_Measure( OSC_MEASURE_HCL_DIV, OSC_NUM_CYCLES);
        u8Coarse++;
    }while((i32Freq < i32Target) && (u8Coarse <= OSC_MAX_COARSE) );
    u8Coarse--;
	
    /* At this point measured frequency should be greater than target and a valid coarse range selected. 
	   Check for error condition: coarse range overflow.                                                  */
    if(i32Freq < i32Target) 
        return 0;

    /* Now trim to this range using binary search. */ 
    u8MaxFine = (OSC_MAX_FINE-OSC_RESERVE_RANGE)*OSC_NUM_SFINE;
    u8MinFine = OSC_RESERVE_RANGE*OSC_NUM_SFINE;
	
	/* Until maxFine and minFine are 1 apart */ 
    do
    {
        i32LastFreq = i32Freq; 
        u8Fine = ((u8MaxFine+u8MinFine)/2);
		*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_TRIM_Msk)) | ((u8Fine/OSC_NUM_SFINE + u8Coarse*32)<<SYS_IRCTTRIM_TRIM_Pos) );
        *(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_FINE_Msk)) | (OSC_SFINE((u8Fine & (OSC_NUM_SFINE-1)))<<SYS_IRCTTRIM_FINE_Pos) );
        i32Freq = OSC_Measure( OSC_MEASURE_HCL_DIV, OSC_NUM_CYCLES);
		 /* Check if Freq is greter or less than target, move max and mins accordingly */
        if (i32Freq < i32Target)   
            u8MinFine = u8Fine; 
        else
            u8MaxFine = u8Fine;
    }while ((u8MinFine-u8MinFine != 1) );	
		
	/* If the target freq is greater than the measured increase u8Fine */
    if (i32Freq <= i32Target)
    {    
        u8Fine++;
        i32LastFreq = i32Freq;
		*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_TRIM_Msk)) | ((u8Fine/OSC_NUM_SFINE + u8Coarse*(OSC_MAX_FINE+1))<<SYS_IRCTTRIM_TRIM_Pos) );
        *(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_FINE_Msk)) | (OSC_SFINE((u8Fine & (OSC_NUM_SFINE-1)))<<SYS_IRCTTRIM_FINE_Pos) );
        i32Freq = OSC_Measure( OSC_MEASURE_HCL_DIV, OSC_NUM_CYCLES);
    }
	
	/* If the last measured freq is greater than the target set minFine to be the last freq */
    if (i32LastFreq>=i32Target)
	{
		*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_TRIM_Msk)) | ((u8MinFine/OSC_NUM_SFINE + u8Coarse*(OSC_MAX_FINE+1))<<SYS_IRCTTRIM_TRIM_Pos) );
        *(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_FINE_Msk)) | (OSC_SFINE((u8MinFine & (OSC_NUM_SFINE-1)))<<SYS_IRCTTRIM_FINE_Pos) );
        i32LastFreq = OSC_Measure( OSC_MEASURE_HCL_DIV, OSC_NUM_CYCLES);
    } 
	
	/* LastFreq was closer, decrement fine trim */
    if(OSC_ABS(i32Target-i32LastFreq) < OSC_ABS(i32Freq-i32Target))
	{    
        u8Fine--;
        i32Freq = i32LastFreq;
    }	
	
    /* Set final trim and return clock divider to unity. */
    *(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_TRIM_Msk)) | ((u8Fine/OSC_NUM_SFINE + u8Coarse*(OSC_MAX_FINE+1))<<SYS_IRCTTRIM_TRIM_Pos) );
    *(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4) = (((*(volatile uint32_t *)((uint32_t)&(SYS->IRCTTRIM0)+u8TrimIdx*4))&(~SYS_IRCTTRIM_FINE_Msk)) | (OSC_SFINE((u8Fine & (OSC_NUM_SFINE-1)))<<SYS_IRCTTRIM_FINE_Pos) );
    i32Freq = OSC_Measure( OSC_MEASURE_HCL_DIV, OSC_NUM_CYCLES);
	
	/* Resotre configuration into register */
	CLK->CLKSEL0 = ( (CLK->CLKSEL0&(~CLK_CLKSEL0_HIRCFSEL_Msk)) | (u32CurFrqSel<<CLK_CLKSEL0_HIRCFSEL_Pos) );
	CLK->CLKDIV0 = ( (CLK->CLKDIV0&(~CLK_CLKDIV0_HCLKDIV_Msk)) | (u32CurHCLKDiv<<CLK_CLKDIV0_HCLKDIV_Pos) );
	CLK->CLKSEL0 = ( (CLK->CLKSEL0&(~CLK_CLKSEL0_HCLKSEL_Msk)) | (u32CurHCLKSel<<CLK_CLKSEL0_HCLKSEL_Pos) );
	
	return (i32Freq);
}

/*@}*/ /* end of group ISD9100_OSC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_OSC_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
