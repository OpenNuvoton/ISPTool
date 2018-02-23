/**************************************************************************//**
 * @file     DPWM.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 16/08/19 07:27p $
 * @brief    ISD9200 DPWM driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_DPWM_Driver DPWM Driver
  @{
*/

/** @addtogroup I91200_DPWM_EXPORTED_FUNCTIONS DPWM Exported Functions
  @{
*/

/**
  * @brief      DPWM open function
  * @return     None
  * @details    
  */
void DPWM_Open(void)
{
	
}

/**
  * @brief      DPWM close function
  * @return     None
  * @details   
  */
void DPWM_Close(void)
{   
      
}

/**
  * @brief      Write to DPWM FIFO for transmit
  * @param      pi16Stream is pointer of input data stream for transmit.
  * @param      u32Count is transmit sample count.  
  * @return     None
  * @details    If all channels of PDMA are occupied, programmer can call this 
  *             function to write FIFO by CPU. But programmer needs to 
  *             handle suitable time for writing.  
  */
void DPWM_WriteFIFO(int16_t *pi16Stream, uint32_t u32Count)
{
	uint32_t u32CountTmp = u32Count;
	
	while(u32CountTmp > 0)	
	{
		while(DPWM->STS == 1); //FIFO is full
		DPWM->DATA = *pi16Stream++;
		u32CountTmp--;	
	}
}

/**
  * @brief      Set the sample Rate of data 
  * @param      u32SampleRate is sample Rate of data.
  * @return     Real sample rate.
  */
uint32_t DPWM_SetSampleRate(uint32_t u32SampleRate)
{
	uint32_t u32Div, u32Ratio, u32DPWMClk;
	
	// Get DPWM clock frequency.
	u32DPWMClk = (CLK->CLKSEL1&CLK_CLKSEL1_DPWMSEL_Msk)?(__HXT):CLK_GetHCLKFreq()/(((CLK->CLKDIV0&CLK_CLKDIV0_DPWMDIV_Msk)>>CLK_CLKDIV0_DPWMDIV_Pos)+1);	
	// Get BIQ's ratio.
	u32Ratio = ((BIQ->CTL&BIQ_CTL_BIQEN_Msk)&&(BIQ->CTL&BIQ_CTL_PATHSEL_Msk))?(64*((BIQ->CTL&BIQ_CTL_DPWMPUSR_Msk)>>BIQ_CTL_DPWMPUSR_Pos)):64;
	// Cal divider value.(Fs = DPWM's clock / DIVIDER / BIQ's ratio)
	DPWM->ZOHDIV = ((u32Div=(u32DPWMClk/u32Ratio)/u32SampleRate)>255)?255:((u32Div<1)?1:u32Div);
	// Returb really frequency.
	return ((u32DPWMClk/(DPWM->ZOHDIV))/u32Ratio);
}

/**
  * @brief      Get the sample Rate of data 
  * @return     Real sample rate.
  */
uint32_t DPWM_GetSampleRate(void)
{
	uint32_t u32Ratio,u32DPWMClk;

	// Get DPWM clock frequency.
	u32DPWMClk = (CLK->CLKSEL1&CLK_CLKSEL1_DPWMSEL_Msk)?(__HXT):CLK_GetHCLKFreq()/(((CLK->CLKDIV0&CLK_CLKDIV0_DPWMDIV_Msk)>>CLK_CLKDIV0_DPWMDIV_Pos)+1);	
	// Get BIQ's ratio.
	u32Ratio = ((BIQ->CTL&BIQ_CTL_BIQEN_Msk)&&(BIQ->CTL&BIQ_CTL_PATHSEL_Msk))?(64*((BIQ->CTL&BIQ_CTL_DPWMPUSR_Msk)>>BIQ_CTL_DPWMPUSR_Pos)):64;
	// Returb really frequency.
	return ((u32DPWMClk/(DPWM->ZOHDIV))/u32Ratio);	
}

/*@}*/ /* end of group I91200_DPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_DPWM_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
