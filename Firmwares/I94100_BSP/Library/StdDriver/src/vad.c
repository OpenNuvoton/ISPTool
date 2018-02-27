/**************************************************************************//**
 * @file     vad.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/12/28 05:37p $
 * @brief    I94100 Series vad driver source file
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

/** @addtogroup I94100_VAD_Driver VAD Driver
  @{
*/

/** @addtogroup I94100_VAD_EXPORTED_FUNCTIONS VAD Exported Functions
  @{
*/

/**
  * @brief      Start VAD module 
  * @param[in]  vad The base address of VAD module
  * @return     None.
  */
void VAD_Open(VAD_T *vad)
{
}

/**
  * @brief      Stop VAD module 
  * @param[in]  vad The base address of VAD module
  * @return     None.
  */
void VAD_Close(VAD_T *vad)
{
}

/**
  * @brief      Set the detect voice's sample Rate 
  * @param[in]  vad The base address of VAD module
  * @param[in]  u32SampleRate Sample Rate of input voice data.
  * @return     Real detect sample rate.
  */
uint32_t VAD_SetSampleRate(VAD_T* vad,uint32_t u32SampleRate)
{
	uint16_t const au16OSRTable[4]={48,64,96};
	uint32_t u32SourceClock,u32BusClock,u32MainClock,u32OSR;
	
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
	u32OSR = (((vad->SINC_CTL&VAD_SINC_CTL_SINCOSR_Msk)>>VAD_SINC_CTL_SINCOSR_Pos)>=3)?1:(au16OSRTable[(vad->SINC_CTL&VAD_SINC_CTL_SINCOSR_Msk)>>VAD_SINC_CTL_SINCOSR_Pos]);
	// Cal BusClock.
	u32BusClock = u32SampleRate*u32OSR;
	// Cal main working clock
	u32MainClock = u32BusClock*4;
	
	DMIC->DIV = (DMIC->DIV & ~(DMIC_DIV_MCLKDIV_Msk|DMIC_DIV_PCLKDIV_Msk)) | ((u32MainClock/u32BusClock-1)<<DMIC_DIV_MCLKDIV_Pos) | ((u32SourceClock/u32MainClock-1)<<DMIC_DIV_PCLKDIV_Pos);

	return ((u32SourceClock/(((DMIC->DIV&DMIC_DIV_PCLKDIV_Msk)>>DMIC_DIV_PCLKDIV_Pos)+1))/(((DMIC->DIV&DMIC_DIV_MCLKDIV_Msk)>>DMIC_DIV_MCLKDIV_Pos)+1))/u32OSR;
}

/**
  * @brief      Write BIQ's coefficient value.
  * @param[in]  vad The base address of VAD module
  * @return     Real sample rate.
  */
void VAD_WriteBIQCoeff(VAD_T *vad, uint8_t u8Coeff, uint32_t u32Value)
{
	switch(u8Coeff)
	{
		case VAD_COEFF_B0:	(vad)->BIQ_CTL1 = ((vad)->BIQ_CTL1 & ~VAD_BIQ_CTL1_BIQB0_Msk)|((u32Value<<VAD_BIQ_CTL1_BIQB0_Pos)&VAD_BIQ_CTL1_BIQB0_Msk);	break;
		case VAD_COEFF_B1:	(vad)->BIQ_CTL1 = ((vad)->BIQ_CTL1 & ~VAD_BIQ_CTL1_BIQB1_Msk)|((u32Value<<VAD_BIQ_CTL1_BIQB1_Pos)&VAD_BIQ_CTL1_BIQB1_Msk);	break;
		case VAD_COEFF_B2:	(vad)->BIQ_CTL2 = ((vad)->BIQ_CTL2 & ~VAD_BIQ_CTL2_BIQB2_Msk)|((u32Value<<VAD_BIQ_CTL2_BIQB2_Pos)&VAD_BIQ_CTL2_BIQB2_Msk);	break;
		case VAD_COEFF_A1:	(vad)->BIQ_CTL0 = ((vad)->BIQ_CTL0 & ~VAD_BIQ_CTL0_BIQA1_Msk)|((u32Value<<VAD_BIQ_CTL0_BIQA1_Pos)&VAD_BIQ_CTL0_BIQA1_Pos);	break;	
		case VAD_COEFF_A2:	(vad)->BIQ_CTL0 = ((vad)->BIQ_CTL0 & ~VAD_BIQ_CTL0_BIQA2_Msk)|((u32Value<<VAD_BIQ_CTL0_BIQA2_Pos)&VAD_BIQ_CTL0_BIQA2_Pos);	break;			
	}
}


/*@}*/ /* end of group I91400_VAD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91400_VAD_Driver */

/*@}*/ /* end of group I91400_Device_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
