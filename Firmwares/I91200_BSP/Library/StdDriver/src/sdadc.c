/**************************************************************************//**
 * @file     sdadc.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 16/08/22 13:27p $
 * @brief    I91200 SDADC driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_SDADC_Driver SDADC Driver
  @{
*/


/** @addtogroup I91200_SDADC_EXPORTED_FUNCTIONS SDADC Exported Functions
  @{
*/



/**
  * @brief      ADC open function
  * @return     None
  * @details    
  */
void ADC_Open(void)
{

}

/**
  * @brief      ADC close function
  * @return     None
  * @details   
  */
void ADC_Close(void)
{   
      
}

/**
  * @brief      Set SDADC sample rate for audio path
  * @return     Real sample rate, unit is Hz.
  * @details    The sample rate is calculated by: Fs = MCLK กา CLK_DIV กา DSR
  *             The vaild sample rate range: 48kHz~8kHz.
  *             If BIQ modile is enabled, this function must be called after BIQ initialization
  */
uint32_t SDADC_SetAudioSampleRate(uint32_t u32SampleRate)
{
	uint32_t u32MCLK;
	uint16_t u16DSR;
	uint8_t u8Div;
	
	//Check BIQ module whether enable or not
	if (((BIQ->CTL&BIQ_CTL_SDADCWNSR_Msk)>BIQ_CTL_SDADC_DOWNSAMPLE1X)&&((BIQ->CTL&BIQ_CTL_PATHSEL_Msk)==0))
	{
		// Set SDADC clock source: HIRC = 49.152M/1--->SDADC MCLK 49.152M
		CLK_SetModuleClock(SDADC_MODULE, CLK_CLKSEL1_SDADCSEL_HCLK, CLK_CLKDIV0_SDADC(1));
		u32MCLK = SystemCoreClock; 
		u16DSR = 32*((BIQ->CTL&BIQ_CTL_SDADCWNSR_Msk)>>BIQ_CTL_SDADCWNSR_Pos);
		SDADC_SET_DSRATIO(SDADC, SDADC_DS_RATION_32);
	}else
	{
		// Set SDADC clock source: HIRC = 49.152M/2--->SDADC MCLK 24.576M
		CLK_SetModuleClock(SDADC_MODULE, CLK_CLKSEL1_SDADCSEL_HCLK, CLK_CLKDIV0_SDADC(2));
		u32MCLK = SystemCoreClock>>1; 
		u16DSR = 64;
		SDADC_SET_DSRATIO(SDADC, SDADC_DS_RATION_64);
	}
	
	// The maximum clock rate of the Sigma-Delta Converter is 6.144MHz. 
	// Sample rate is calculated by: Fs = MCLK กา CLK_DIV กา DSR
	// where DSR only uses SDADC DSRATE; didn't enable BIQ down sample
	// Note :MCLK / ADC_CLK >= 4
	// The using SDADC_DS_RATION_64 can make sure sample rate range: 96k~8k and satisfy limitations
	u8Div = ((u32MCLK/u16DSR) + (u32SampleRate>>1))/u32SampleRate;
	
	if (u8Div == 0)
		return 0;
	
	SDADC_SET_SDCLKDIV(SDADC, u8Div);
	
	return (u32MCLK/u8Div/u16DSR);

}

/**
  * @brief      Get sample rate of SDADC audio path
  * @return     Sample rate, unit is Hz.
  * @details    This function calculate sample rate according to SDCLK 
  *             divisor and over sampling ration.
  *             Formula: Fs = HCLK/CLKDIV/OSR.
  */
uint32_t SDADC_GetAudioSampleRate(void)
{
    uint32_t au32DSR[4] = {1, 16, 32, 64};
	uint32_t u32OSR, u32ADCClk, u32Div;

    u32OSR = au32DSR[SDADC->CTL&SDADC_CTL_DSRATE_Msk];
	if (BIQ->CTL&BIQ_CTL_BIQEN_Msk)
		u32OSR*=((BIQ->CTL&BIQ_CTL_SDADCWNSR_Msk) >> BIQ_CTL_SDADCWNSR_Pos);
    u32Div = (CLK->CLKDIV0&CLK_CLKDIV0_SDADCDIV_Msk) >> CLK_CLKDIV0_SDADCDIV_Pos;
	u32ADCClk = CLK_GetHCLKFreq()/(u32Div+1);
    return (u32ADCClk/(SDADC->CLKDIV)/u32OSR);
}

/**
  * @brief      Enable SDADC interrupts
  * @param      u32Mask is interrupt flags.
  *             - \ref SDADC_FIFO_INT 
  *             - \ref SDADC_CMP0_INT 
  *             - \ref SDADC_CMP1_INT
  *             - \ref SDADC_ALC_PLMT_INT
  *             - \ref SDADC_ALC_NG_INT
  *             - \ref SDADC_ALC_GINC_INT
  *             - \ref SDADC_ALC_GDEC_INT
  *             - \ref SDADC_ALC_GMAX_INT
  *             - \ref SDADC_ALC_GMIN_INT
  * @return     None.
  * @details    SDADC_FIFO_INT is generated whenever FIFO level exceeds
  *             that set in u8Level of SDADC_SET_FIFOINTLEVEL macro.
  *             SDADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             SDADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void SDADC_EnableInt(uint32_t u32Mask)
{
	if (u32Mask&SDADC_FIFO_INT)
	{
		/* ADC FIFO interrupt and PDMA don't coexist */
		SDADC->PDMACTL = 0;
		SDADC->CTL |= SDADC_CTL_FIFOTHIE_Msk;
	}
	
	if (u32Mask&SDADC_CMP0_INT)
	{
		/* clear the CMP interrupt flags for safe */
		SDADC->CMPR[0] |= (SDADC_CMPR_CMPF_Msk);
		SDADC->CMPR[0] |= (SDADC_CMPR_CMPIE_Msk);
	}
	
	if (u32Mask&SDADC_CMP1_INT)
	{
		/* clear the CMP interrupt flags for safe */
		SDADC->CMPR[1] |= (SDADC_CMPR_CMPF_Msk);
		SDADC->CMPR[1] |= (SDADC_CMPR_CMPIE_Msk);
	}
	
	if (u32Mask&0x00003F00)
	{
		/* clear the ALC interrupt flags for safe */
		ALC->INTCTL |= ((0x3F << ALC_INTCTL_PLMTIF_Pos)|ALC_INTCTL_ALCIF_Msk);
		ALC->INTCTL |= ((u32Mask&0x00003F00) >> ALC_INTCTL_PLMTIF_Pos);
	}
}

/**
  * @brief      Disable ADC interrupts
  * @param      u32Mask is interrupt flags.
  *             - \ref SDADC_FIFO_INT 
  *             - \ref SDADC_CMP0_INT 
  *             - \ref SDADC_CMP1_INT
  *             - \ref SDADC_ALC_PLMT_INT
  *             - \ref SDADC_ALC_NG_INT
  *             - \ref SDADC_ALC_GINC_INT
  *             - \ref SDADC_ALC_GDEC_INT
  *             - \ref SDADC_ALC_GMAX_INT
  *             - \ref SDADC_ALC_GMIN_INT
  * @return     None.
  * @details    ADC_FIFO_INT is generated whenever FIFO level exceeds
  *             that set in u8Level of ADC_SET_FIFOINTLEVEL macro.
  *             ADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             ADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void SDADC_DisableInt(uint32_t u32Mask)
{
	if (u32Mask&SDADC_FIFO_INT)
		SDADC->CTL &= (~SDADC_CTL_FIFOTHIE_Msk);
		
	if (u32Mask&SDADC_CMP0_INT)
		SDADC->CMPR[0] &= (~SDADC_CMPR_CMPIE_Msk);
	
	if (u32Mask&SDADC_CMP1_INT)
		SDADC->CMPR[1] &= (~SDADC_CMPR_CMPIE_Msk);
		
	if (u32Mask&0x00003F00)
		ALC->INTCTL &= ~((u32Mask&0x00003F00) >> ALC_INTCTL_PLMTIF_Pos);
}

/**
  * @brief     Get the interrupt status bits
  * @param     u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *             - \ref SDADC_FIFO_INT 
  *             - \ref SDADC_CMP0_INT 
  *             - \ref SDADC_CMP1_INT
  *             - \ref SDADC_ALC_PLMT_INT
  *             - \ref SDADC_ALC_NG_INT
  *             - \ref SDADC_ALC_GINC_INT
  *             - \ref SDADC_ALC_GDEC_INT
  *             - \ref SDADC_ALC_GMAX_INT
  *             - \ref SDADC_ALC_GMIN_INT
  * @return    interrupt status bits
  */
uint32_t SDADC_GetIntFlag(uint32_t u32Mask)
{
	uint32_t u32Flag = 0;
	
	if (u32Mask&SDADC_FIFO_INT)
		u32Flag |= (SDADC->FIFOSTS&SDADC_FIFOSTS_THIF_Msk)?SDADC_FIFO_INT:0;
	
	if (u32Mask&SDADC_CMP0_INT)
		u32Flag |= (SDADC->CMPR[0]&SDADC_CMPR_CMPF_Msk)?SDADC_CMP0_INT:0;
	
	if (u32Mask&SDADC_CMP1_INT)
		u32Flag |= (SDADC->CMPR[1]&SDADC_CMPR_CMPF_Msk)?SDADC_CMP1_INT:0;
	
	if (u32Mask&0x00003F00)
		u32Flag |= ALC->INTCTL&0x00003F00;
	
	return u32Flag;
}

/**
  * @brief     Clear the selected interrupt status bits
  * @param     u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *             - \ref SDADC_CMP0_INT 
  *             - \ref SDADC_CMP1_INT
  *             - \ref SDADC_ALC_PLMT_INT
  *             - \ref SDADC_ALC_NG_INT
  *             - \ref SDADC_ALC_GINC_INT
  *             - \ref SDADC_ALC_GDEC_INT
  *             - \ref SDADC_ALC_GMAX_INT
  *             - \ref SDADC_ALC_GMIN_INT
  * @return    None
  * @details   ADC_FIFO_INT don't need to clear.
  */
void SDADC_ClearIntFlag(uint32_t u32Mask)
{
	if (u32Mask&SDADC_CMP0_INT)
		SDADC->CMPR[0] |= SDADC_CMPR_CMPF_Msk;
	
	if (u32Mask&SDADC_CMP1_INT)
		SDADC->CMPR[1] |= SDADC_CMPR_CMPF_Msk;
	
	if (u32Mask&0x00003F00)
		ALC->INTCTL |= ((u32Mask&0x00003F00)|ALC_INTCTL_ALCIF_Msk);
}

/**
  * @brief      Enable microphone bias generator
  * @param      u32BiasSel is microphone bias voltage
  *             - \ref SDADC_MICBSEL_1P5V
  *             - \ref SDADC_MICBSEL_1P8V
  *             - \ref SDADC_MICBSEL_1P95V
  *             - \ref SDADC_MICBSEL_2P1V
  *             - \ref SDADC_MICBSEL_2P25V
  *             - \ref SDADC_MICBSEL_2P4V
  *             - \ref SDADC_MICBSEL_2P55V   
  *             - \ref SDADC_MICBSEL_2P7V   
  * @return     void
  * @details    The user should consider the microphone manufacturers specification 
  *             in deciding on the optimum MICBIAS voltage to use. Generally, a microphone
  *             will require a current of 0.1mA to a maximum 0.5mA and a voltage of 1V to 3V across it.
  */
void SDADC_EnableMICBias(uint32_t u32BiasSel)  
{
	ANA->MICBSEL = (u32BiasSel&ANA_MICBSEL_LVL_Msk);
	ANA->MICBEN = 0;
} 

/**
  * @brief      Disable microphone bias generator
  * @return     None
  */
void SDADC_DisableMICBias(void)  
{
	ANA->MICBSEL = 0x0;
	ANA->MICBEN = ANA_MICBEN_PD_Msk;
}

/**
  * @brief      Set ALC Maximum gain
  * @param      i32MaxGaindB gain value
  *             i32MaxGaindB/100 = MAXGAIN * 6 - 6.75
  * \n           
  *             i32MaxGaindB    mapping dB:                                             
  *              3525           -> 35.25 dB;
  *              1725           -> 17.25 dB;
  *               525           ->  5.25 dB;
  *              -675           -> -6.75 dB;
  *
  * @return     Real gain value
  * @details    From -6.75 to +35.25dB in 6dB step size. 0x0 is lowest gain setting at
  *             -6.75dB and 0x7 is largest gain at 35.25dB.
  */
int32_t SDADC_SetALCMaxGaindB(int32_t i32MaxGaindB)
{
	if(i32MaxGaindB < -675)
		i32MaxGaindB = -675;
	else if(i32MaxGaindB > 3525)
		i32MaxGaindB = 3525;
	
	i32MaxGaindB = ((i32MaxGaindB + 675) / 600);
	ALC->CTL = (ALC->CTL&~ALC_CTL_MAXGAIN_Msk)|(i32MaxGaindB << ALC_CTL_MAXGAIN_Pos);
	
	return (i32MaxGaindB*600-675);
}

/**
  * @brief      Set ALC Minimum gain
  * @param      i32MinGaindB gain value
  *             i32MinGaindB/100 = MINGAIN * 6 - 12
  * \n            
  *             i32MinGaindB    mapping dB:                                             
  *              3000           ->  30 dB;
  *              1725           ->   0 dB;
  *               525           ->  -6 dB;
  *             -1200           -> -12 dB;
  *
  * @return     Real gain value
  * @details    From -12 to +30 dB in 6dB step size. 0x0 is lowest gain setting at
  *             -12dB and 0x7 is largest gain at 30dB.
  */
int32_t SDADC_SetALCMinGaindB(int32_t i32MinGaindB)
{
	if(i32MinGaindB < -1200)
		i32MinGaindB = -1200;
	else if(i32MinGaindB > 3000)
		i32MinGaindB = 3000;
	
	i32MinGaindB = ((i32MinGaindB + 1200) / 600);
	ALC->CTL = (ALC->CTL&~ALC_CTL_MINGAIN_Msk)|(i32MinGaindB << ALC_CTL_MINGAIN_Pos);
	
	return (i32MinGaindB*600-1200);
}

/**
  * @brief      Set ALC target lavel
  * @param      i32TargetLevel target level value (dB)
  *             i32TargetLevel/100 = TARGETLV * 1.5 - 28.5
  * \n            
  *             i32TargetLevel    mapping dB:                                             
  *              -600           ->    -6 dB;
  *             -2850           -> -28.5 dB;
  *
  * @return     Real target level
  * @details    From -28.5 to -6 dB in 1.5dB step size. 0x0 is lowest target setting at
  *             -28.5dB and 0xf is largest target at -6dB.
  */
int32_t SDADC_SetALCTargetLevel(int32_t i32TargetLevel)
{
	if(i32TargetLevel < -2850)
		i32TargetLevel = -2850;
	else if(i32TargetLevel > -600)
		i32TargetLevel = -600;
	
	i32TargetLevel = ((i32TargetLevel + 2850) / 150);
	ALC->CTL = (ALC->CTL&~ALC_CTL_TARGETLV_Msk)|(i32TargetLevel << ALC_CTL_TARGETLV_Pos);
	
	return (i32TargetLevel*150-2850);
}

/*@}*/ /* end of group I91200_SDADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_SDADC_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/

