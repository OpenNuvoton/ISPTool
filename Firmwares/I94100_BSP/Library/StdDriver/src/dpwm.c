/**************************************************************************//**
 * @file     dpwm.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/07/26 05:37p $
 * @brief    I94100 Series dpwm driver source file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "I94100.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_DPWM_Driver DPWM Driver
  @{
*/

/** @addtogroup I94100_DPWM_EXPORTED_FUNCTIONS DPWM Exported Functions
  @{
*/

void DPWM_Open(void){
}
/**
  * @brief      Stop DPWM module 
  * @param      None.
  * @return     None.
  */
void DPWM_Close(void)
{
	DPWM_STOP_PLAY(DPWM);
	DPWM_DISABLE_DRIVER(DPWM);
}

/**
  * @brief      Set the sample Rate of data 
  * @param      u32SampleRate is sample Rate of data.
  * @return     Real sample rate.
  */
uint32_t DPWM_SetSampleRate(uint32_t u32SampleRate)
{
	uint32_t u32Temp, u32ZOHDiv, u32Ratio, u32DPWMClk, u32ClkDiv, u32Error1, u32Error2;
	u32ClkDiv = 0;
	
	SystemCoreClockUpdate();
	
	/* Get DPWM working clock ratio and clock divisor. */
	u32Ratio = (DPWM->CTL&DPWM_CLKSET_500FS)?125:128;
	/* Get DPWM clock source frequency. */
	u32Temp = CLK->CLKSEL2 & CLK_CLKSEL2_DPWMSEL_Msk;
	switch(u32Temp)
	{
		case CLK_CLKSEL2_DPWMSEL_PCLK0:
			u32DPWMClk = CLK_GetPCLK0Freq();
		break;
		case CLK_CLKSEL2_DPWMSEL_PLL:
			u32DPWMClk = CLK_GetPLLClockFreq();
		break;
		case CLK_CLKSEL2_DPWMSEL_HXT:
			u32DPWMClk = CLK_GetHXTFreq();
		break;
		case CLK_CLKSEL2_DPWMSEL_HIRC:
			u32DPWMClk = __HIRC;
		break;
		default:
			return 0;
	}
	
	if(u32SampleRate == 48000 || u32SampleRate == 96000)
	{
		u32ZOHDiv = 4;
		u32ClkDiv = u32DPWMClk / ((u32Ratio * u32ZOHDiv) * u32SampleRate) - 1;
	}
	else
	{
		/* Calculate divider value.(overall divisor = (DPWM's source clock/Fs)/Ratio). */
		u32Temp = (u32DPWMClk/u32SampleRate)/u32Ratio;
		/* Adjust Error */
		u32Error1 = ((u32DPWMClk/u32Ratio)/u32Temp)-u32SampleRate;
		u32Error2 = u32SampleRate- ((u32DPWMClk/u32Ratio)/(u32Temp+1));
		u32Temp = (u32Error1>u32Error2)?(u32Temp+1):u32Temp;
		
		if ((u32Temp >= DPWM_ZOHDIV_MIN))
		{
			if(u32Temp <= DPWM_ZOHDIV_MAX)
			{
				u32ZOHDiv = u32Temp;
				u32ClkDiv = 0;
			}else
			{
				do
				{
					u32ZOHDiv = u32Temp/(++u32ClkDiv+1);
				}
				while(u32ZOHDiv > DPWM_ZOHDIV_MAX);
			}
		}
		else
		{
			u32ZOHDiv = DPWM_ZOHDIV_MIN;
			u32ClkDiv = 0;
		}
	}
	
	

	DPWM_SET_CLOCKDIV(DPWM, u32ClkDiv);
	DPWM_SET_ZOHDIV(DPWM, u32ZOHDiv);
	
	/* Return exact frequency. */
	return ((u32DPWMClk/DPWM_GET_ZOHDIV(DPWM))/u32Ratio);
}

/**
  * @brief      Get the sample Rate of data 
  * @return     Real sample rate.
  */
uint32_t DPWM_GetSampleRate(void)
{
	uint32_t u32DPWMClk, u32Ratio;
	
	SystemCoreClockUpdate();
	
	/* Get DPWM clock source frequency. */
	if(CLK->CLKSEL2 & CLK_CLKSEL2_DPWMSEL_PCLK0)
		u32DPWMClk = CLK_GetPCLK0Freq()/(DPWM_GET_CLOCKDIV(DPWM) + 1);
	else if(CLK->CLKSEL2 & CLK_CLKSEL2_DPWMSEL_PLL)
		u32DPWMClk = CLK_GetPLLClockFreq()/(DPWM_GET_CLOCKDIV(DPWM) + 1);
	else
		return 0;
	
	/* Get DPWM_CLK/Fs ratio: K. */
	u32Ratio = (DPWM->CTL&DPWM_CLKSET_500FS)?125:128;
	
	/* Return exact frequency. */
	return ((u32DPWMClk/DPWM_GET_ZOHDIV(DPWM))/u32Ratio);
}

/**
  * @brief      Write to DPWM FIFO for transmit
  * @param      pi16Stream is pointer of input data stream for transmit.
  * @param      u32count is transmit sample count.  
  * @return     None
  * @details    If all channels of PDMA are occupied, programmer can call this 
  *				function to write FIFO by CPU. But programmer needs to 
  *				handle suitable time for writing.  
  */
void DPWM_WriteFIFO(int16_t *pi16Stream, uint32_t u32Count)
{
	uint32_t u32CountTmp = u32Count;
	
	while(u32CountTmp > 0)	
	{
		while(DPWM_IS_FIFOFULL(DPWM)); //FIFO is full
		DPWM->FIFO = *pi16Stream++;
		u32CountTmp--;
	}
}

/**
  * @brief      Write mono data to DPWM FIFO for stereo
  * @param      pi16Stream is pointer of input data stream for transmit.
  * @param      u32count is transmit sample count.
  * @return     None
  * @details    This API is similar to DPWM_WriteFIFO. If the data stream is in mono format,
  *				use this API write data to stereo FIFO.
  */
void DPWM_WriteMonotoStereo(int16_t *pi16Stream, uint32_t u32Count)
{
	uint32_t u32CountTmp = u32Count;
	
	while(u32CountTmp > 0)	
	{
		while(DPWM_IS_FIFOFULL(DPWM)); //FIFO is full
		DPWM->FIFO = *pi16Stream;
		while(DPWM_IS_FIFOFULL(DPWM)); //FIFO is full
		DPWM->FIFO = *pi16Stream++;
		u32CountTmp--;
	}
}

void DPWM_WriteCoeff(uint32_t u32Band, uint8_t u8Coeff, uint32_t u32Value)
{
	*(volatile uint32_t *)((uint32_t)&DPWM->COEFF0 + u32Band*0x14 + u8Coeff*0x04) =  u32Value;
}

/*@}*/ /* end of group I94100_DPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_DPWM_Driver */

/*@}*/ /* end of group I94100_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
