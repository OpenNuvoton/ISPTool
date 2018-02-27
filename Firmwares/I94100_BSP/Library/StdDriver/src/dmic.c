/**************************************************************************//**
 * @file     dmic.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/12/26 05:37p $
 * @brief    I94100 Series dmic driver source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
 
#include "stdio.h"
#include "I94100.H"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_DMIC_Driver DMIC Driver
  @{
*/

/** @addtogroup I94100_DMIC_EXPORTED_FUNCTIONS DMIC Exported Functions
  @{
*/

/**
  * @brief      Start DMIC module 
  * @param      None.
  * @return     None.
  */
void DMIC_Open(DMIC_T *dmic)
{
}

/**
  * @brief      Stop DMIC module 
  * @param      None.
  * @return     None.
  */
void DMIC_Close(DMIC_T *dmic)
{
	DMIC_DISABLE_CHANNEL(dmic,DMIC_CTL_CH0|DMIC_CTL_CH1|DMIC_CTL_CH2|DMIC_CTL_CH3);
}

/**
  * @brief      Set the sample Rate of data 
  * @param      u32SampleRate is sample Rate of data.
  * @return     Real sample rate.
  */
uint32_t DMIC_SetSampleRate(DMIC_T* dmic,uint32_t u32SampleRate)
{
	uint16_t const au16OSRTable[4]={32,64,128,256};
	uint32_t u32SourceClock,u32BusClock,u32MainClock,u32OSR, u32Temp;
	
	// Get DMIC clock source.
	switch(CLK->CLKSEL2&CLK_CLKSEL2_DMICSEL_Msk)
	{
		case CLK_CLKSEL2_DMICSEL_PCLK0:  u32SourceClock = CLK_GetPCLK0Freq();      break;
		case CLK_CLKSEL2_DMICSEL_PLL:    u32SourceClock = CLK_GetPLLClockFreq();   break;
		case CLK_CLKSEL2_DMICSEL_HXT:    u32SourceClock = CLK_GetHXTFreq();        break;
		case CLK_CLKSEL2_DMICSEL_HIRC:   u32SourceClock = __HIRC;                  break;
		default:                                                                   return 0;
	}	
	// Get OSR config and cal BusClock.
	switch( dmic->CTL&DMIC_CTL_OSR_Msk )
	{
		case DMIC_CTL_DOWNSAMPLE_32:
		case DMIC_CTL_DOWNSAMPLE_64:
		case DMIC_CTL_DOWNSAMPLE_128:
		case DMIC_CTL_DOWNSAMPLE_256:		u32OSR = au16OSRTable[(dmic->CTL&DMIC_CTL_OSR_Msk)>>DMIC_CTL_OSR_Pos];		break;
		case DMIC_CTL_DOWNSAMPLE_100_50:	u32OSR = (u32SampleRate>=32500)?50:100;										break;
		default:     						u32OSR = 1;																	break;
	}
	// Cal BusClock.
	u32BusClock = u32SampleRate*u32OSR;
	
	// Cal main working clock(Depends on whether the frequency is divided).
	u32MainClock = u32SampleRate*(((u32SourceClock%u32SampleRate)!=0)? 256:500);

	//
	dmic->DIV = (dmic->DIV & ~(DMIC_DIV_MCLKDIV_Msk|DMIC_DIV_PCLKDIV_Msk)) | ((u32MainClock/u32BusClock-1)<<DMIC_DIV_MCLKDIV_Pos) | ((u32SourceClock/u32MainClock-1)<<DMIC_DIV_PCLKDIV_Pos);

        u32Temp = (((dmic->DIV&DMIC_DIV_PCLKDIV_Msk)>>DMIC_DIV_PCLKDIV_Pos)+1);
        
	return ((u32SourceClock/u32Temp)/(((dmic->DIV&DMIC_DIV_MCLKDIV_Msk)>>DMIC_DIV_MCLKDIV_Pos)+1))/u32OSR;
}


/*@}*/ /* end of group I91400_DMIC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91400_DMIC_Driver */

/*@}*/ /* end of group I91400_Device_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
