/**************************************************************************//**
 * @file     adc.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 14/06/20 13:27p $
 * @brief    ISD9100 ADC driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include  "ISD9100.h"
/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_ADC_Driver ADC Driver
  @{
*/


/** @addtogroup ISD9100_ADC_EXPORTED_FUNCTIONS ADC Exported Functions
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
  * @brief      Get ADC sample rate
  * @return     Sample rate, unit is Hz.
  * @details    This function calculate sample rate according to SDCLK 
  *             divisor and over sampling ration.
  *             Formula: Fs = HCLK/CLKDIV/OSR.
  */
uint32_t ADC_GetSampleRate(void)
{
    uint32_t u32OSR, u32ADCClk, u32Div;

    switch(ADC->DCICTL)
    {
        case ADC_OSR_RATION_64:
            u32OSR=64;
            break;
        case ADC_OSR_RATION_128:
            u32OSR=128;
            break;
        case ADC_OSR_RATION_192:
            u32OSR=192;
            break;
        case ADC_OSR_RATION_384:
            u32OSR=384;
            break;
        default:
            u32OSR=64;
    }
    u32Div = (CLK->CLKDIV0&0xff00) >> CLK_CLKDIV0_ADCDIV_Pos;
	u32ADCClk = CLK_GetHCLKFreq()/(u32Div+1);
    return (u32ADCClk/(ADC->CLKDIV)/u32OSR);
}

/**
  * @brief      Enable ADC interrupts
  * @param      u32Mask is interrupt flags.
  *             - \ref ADC_FIFO_INT 
  *             - \ref ADC_CMP0_INT 
  *             - \ref ADC_CMP1_INT
  *             - \ref ADC_ALC_INT
  * @return     None.
  * @details    ADC_FIFO_INT is generated whenever FIFO level exceeds
  *             that set in u8Level of ADC_SET_FIFOINTLEVEL macro.
  *             ADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             ADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void ADC_EnableInt(uint32_t u32Mask)
{
	if (u32Mask&ADC_FIFO_INT)
	{
		/* ADC FIFO interrupt and PDMA don't coexist */
		ADC->PDMACTL = 0;
		ADC->INTCTL |= ADC_INTCTL_INTEN_Msk;
	}
	
	if (u32Mask&ADC_CMP0_INT)
	{
		/* clear the CMP interrupt flags for safe */
		ADC->CMP0 |= (ADC_CMP0_CMPFLAG_Msk);
		ADC->CMP0 |= (ADC_CMP0_ADCMPIE_Msk);
	}
	
	if (u32Mask&ADC_CMP1_INT)
	{
		/* clear the CMP interrupt flags for safe */
		ADC->CMP1 |= (ADC_CMP1_CMPFLAG_Msk);
		ADC->CMP1 |= (ADC_CMP1_ADCMPIE_Msk);
	}
	
	if (u32Mask&ADC_ALC_INT)
	{
		/* clear the ALC interrupt flags for safe */
		ALC->INTSTS |= (ALC_INTSTS_INTFLAG_Msk);
		ALC->INTCTL |= (ALC_INTCTL_INTEN_Msk);
	}
}

/**
  * @brief      Disable ADC interrupts
  * @param      u32Mask is interrupt flags.
  *             - \ref ADC_FIFO_INT 
  *             - \ref ADC_CMP0_INT 
  *             - \ref ADC_CMP1_INT
  *             - \ref ADC_ALC_INT
  * @return     None.
  * @details    ADC_FIFO_INT is generated whenever FIFO level exceeds
  *             that set in u8Level of ADC_SET_FIFOINTLEVEL macro.
  *             ADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             ADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void ADC_DisableInt(uint32_t u32Mask)
{
	if (u32Mask&ADC_FIFO_INT)
		ADC->INTCTL &= (~ADC_INTCTL_INTEN_Msk);
		
	if (u32Mask&ADC_CMP0_INT)
		ADC->CMP0 &= (~ADC_CMP0_ADCMPIE_Msk);
	
	if (u32Mask&ADC_CMP1_INT)
		ADC->CMP1 &= (~ADC_CMP1_ADCMPIE_Msk);
		
	if (u32Mask&ADC_ALC_INT)
		ALC->INTCTL &= (~ALC_INTCTL_INTEN_Msk);
}

/**
  * @brief     Get the interrupt status bits
  * @param     u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *            - \ref ADC_CMP0_INT
  *            - \ref ADC_CMP1_INT
  *            - \ref ADC_ALC_INT
  * @return    interrupt status bits
  */
uint32_t ADC_GetIntFlag(uint32_t u32Mask)
{
	if (u32Mask&ADC_CMP0_INT)
		return ADC->CMP0&ADC_CMP0_CMPFLAG_Msk;
	
	if (u32Mask&ADC_CMP1_INT)
		return ADC->CMP1&ADC_CMP1_CMPFLAG_Msk;
	
	if (u32Mask&ADC_ALC_INT)
		return ALC->INTSTS&0x1;
	
	return 0;
}

/**
  * @brief     Clear the selected interrupt status bits
  * @param     u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *            - \ref ADC_CMP0_INT
  *            - \ref ADC_CMP1_INT
  *            - \ref ADC_ALC_INT
  * @return    None
  * @details   ADC_FIFO_INT don't need to clear.
  */
void ADC_ClearIntFlag(uint32_t u32Mask)
{
	if (u32Mask&ADC_CMP0_INT)
		ADC->CMP0 |= ADC_CMP0_CMPFLAG_Msk;
	
	if (u32Mask&ADC_CMP1_INT)
		ADC->CMP1 |= ADC_CMP1_CMPFLAG_Msk;
	
	if (u32Mask&ADC_ALC_INT)
		ALC->INTSTS |= (ALC_INTSTS_INTFLAG_Msk);
}

/**
  * @brief      Select PGA input path
  * @param      u32AMUXSel is path selectin of analog multiplexer for PGA input.
  *             - \ref ADC_MUXCTL_MIC_PATH         
  *             - \ref ADC_MUXCTL_TEMP_PATH
  *             - \ref ADC_MUXCTL_GPIO_PATH
  * @param      u32MUXPSel is positive input selection Of PGA.
  *             - \ref ADC_MUXCTL_POSINSEL_NONE
  *             - \ref ADC_MUXCTL_POSINSEL_GPB1         
  *             - \ref ADC_MUXCTL_POSINSEL_GPB3
  *             - \ref ADC_MUXCTL_POSINSEL_GPB5
  *             - \ref ADC_MUXCTL_POSINSEL_GPB7
  * @param      u32MUXNSel is negative input selection Of PGA.
  *             - \ref ADC_MUXCTL_NEGINSEL_NONE
  *             - \ref ADC_MUXCTL_NEGINSEL_GPB0         
  *             - \ref ADC_MUXCTL_NEGINSEL_GPB1
  *             - \ref ADC_MUXCTL_NEGINSEL_GPB2
  *             - \ref ADC_MUXCTL_NEGINSEL_GPB3
  *             - \ref ADC_MUXCTL_NEGINSEL_GPB4
  *             - \ref ADC_MUXCTL_NEGINSEL_GPB5
  *             - \ref ADC_MUXCTL_NEGINSEL_GPB6
  *             - \ref ADC_MUXCTL_NEGINSEL_GPB7
  * @return     None
  * @details    ISD9100 provides an analog multiplexer which allows the PGA input 
  *             from the dedicated MICP/MICN analog inputs to any of the analog enabled GPIO (GPIOB[7:0]).
  */
void ADC_SetAMUX(uint32_t u32AMUXSel, uint32_t u32MUXPSel, uint32_t u32MUXNSel)
{
	ANA->MUXCTL &= 0x0000;
	
	if (u32AMUXSel == ADC_MUXCTL_MIC_PATH)
	{ 
		ANA->MUXCTL |= (ANA_MUXCTL_MUXEN_Msk|ADC_MUXCTL_MIC_PATH);
	}
	else if (u32AMUXSel == ADC_MUXCTL_TEMP_PATH)
	{
		ANA->MUXCTL |= (ANA_MUXCTL_MUXEN_Msk|ADC_MUXCTL_TEMP_PATH);
	}
	else if (u32AMUXSel == ADC_MUXCTL_GPIO_PATH)
	{
		ANA->MUXCTL |= (ANA_MUXCTL_MUXEN_Msk|u32MUXPSel|u32MUXNSel);
	}    
}

/**
  * @brief      Set GPIO input path to PGA_INP and GPA_INN
  * @param      u32Mode is path selectin of analog multiplexer for PGA input.
  *             - \ref ADC_GPIO_SINGLEEND_CH0_N         
  *             - \ref ADC_GPIO_SINGLEEND_CH1_P
  *             - \ref ADC_GPIO_SINGLEEND_CH1_N
  *             - \ref ADC_GPIO_SINGLEEND_CH2_N
  *             - \ref ADC_GPIO_SINGLEEND_CH3_P
  *             - \ref ADC_GPIO_SINGLEEND_CH3_N
  *             - \ref ADC_GPIO_SINGLEEND_CH4_N
  *             - \ref ADC_GPIO_SINGLEEND_CH5_P
  *             - \ref ADC_GPIO_SINGLEEND_CH5_N
  *             - \ref ADC_GPIO_SINGLEEND_CH6_N
  *             - \ref ADC_GPIO_SINGLEEND_CH7_P
  *             - \ref ADC_GPIO_SINGLEEND_CH7_N
  *             - \ref ADC_GPIO_DIFFERENTIAL_CH01
  *             - \ref ADC_GPIO_DIFFERENTIAL_CH23
  *             - \ref ADC_GPIO_DIFFERENTIAL_CH45
  *             - \ref ADC_GPIO_DIFFERENTIAL_CH67
  * @return     None
  * @details    The negative input of the PGA connects to GPIOB[7:0], 
  *             while the positive PGA input connects to the odd numbered GPIOB[7:1]
  */
void ADC_SetGPIOChannel(uint32_t u32Mode)
{
	ADC_SetAMUX(ADC_MUXCTL_GPIO_PATH, u32Mode&0x0f00, u32Mode&0x00ff);
	u32Mode = u32Mode >> 12;
	GPIO_SetMode(PB, u32Mode, GPIO_MODE_INPUT); // Set GIPO as input mode
}

/**
  * @brief      Enable microphone bias generator
  * @param      u32BiasSel is microphone bias voltage
  *             - \ref ADC_MICBSEL_90_VCCA
	*             - \ref ADC_MICBSEL_65_VCCA
	*             - \ref ADC_MICBSEL_75_VCCA
	*             - \ref ADC_MICBSEL_50_VCCA
	*             - \ref ADC_MICBSEL_24V
	*             - \ref ADC_MICBSEL_17V
	*             - \ref ADC_MICBSEL_20V   
	*             - \ref ADC_MICBSEL_13V   
  * @return     void
  * @details    The user should consider the microphone manufacturers specification 
  *             in deciding on the optimum MICBIAS voltage to use. Generally, a microphone
  *             will require a current of 0.1mA to a maximum 0.5mA and a voltage of 1V to 3V across it.
  */
void ADC_EnableMICBias(uint32_t u32BiasSel)  
{
	ANA->MICBSEL = (ANA->MICBSEL&0x0)|u32BiasSel;
	ANA->MICBEN = ANA_MICBEN_MICBEN_Msk;
} 

/**
  * @brief      Disable microphone bias generator
  * @return     None
  */
void ADC_DisableMICBias(void)  
{
	ANA->MICBSEL = 0x0;
	ANA->MICBEN = 0x0;
}

/**
  * @brief      Set PGA gain
  * @param      i32PGAGainIndB gain value
  *             i32PGAGainIndB/100 = PGAGAIN * 0.75 - 12 
  * \n           
  *             i32PGAGainIndB mapping dB:                                               
  *              3525           -> 35.25 dB;
  *               100           -> 1.00 dB;
  *                 0           -> 0 dB;
  *             -1200           -> -12.00 dB;
  *
  * @return     Real gain value
  * @details    From -12dB to 35.25dB in 0.75dB step size. 0x00 is lowest gain setting at
  *             -12dB and 0x3F is largest gain at 35.25dB.
  */
int32_t ADC_SetPGAGaindB(int32_t i32PGAGainIndB)
{
	if(i32PGAGainIndB < -1200)
		i32PGAGainIndB = -1200;
	else if(i32PGAGainIndB > 3525)
		i32PGAGainIndB = 3525;
	i32PGAGainIndB = ((i32PGAGainIndB + 1200) / 75)&ANA_PGAGAIN_GAINSET_Msk;
	ANA->PGAGAIN = i32PGAGainIndB;
	
	return (i32PGAGainIndB*75-1200);
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
int32_t ADC_SetALCMaxGaindB(int32_t i32MaxGaindB)
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
int32_t ADC_SetALCMinGaindB(int32_t i32MinGaindB)
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
int32_t ADC_SetALCTargetLevel(int32_t i32TargetLevel)
{
	if(i32TargetLevel < -2850)
		i32TargetLevel = -2850;
	else if(i32TargetLevel > -600)
		i32TargetLevel = -600;
	
	i32TargetLevel = ((i32TargetLevel + 2850) / 150);
	ALC->CTL = (ALC->CTL&~ALC_CTL_TARGETLV_Msk)|(i32TargetLevel << ALC_CTL_TARGETLV_Pos);
	
	return (i32TargetLevel*150-2850);
}

/**
  * @brief      Get PGA gain
  * @return     Real gain value
  *             i32PGAGainIndB/100 = PGAGAIN * 0.75 - 12
  * \n            
  *             i32PGAGainIndB    mapping dB:                                             
  *              3525           -> 35.25 dB;
  *               100           -> 1.00 dB;
  *                 0           -> 0 dB;
  *             -1200           -> -12.00 dB;
  *
  * @details    From -12dB to 35.25dB in 0.75dB step size. 0x00 is lowest gain setting at
  *             -12dB and 0x3F is largest gain at 35.25dB.
  */
int32_t ADC_GetPGAGaindB(void)
{
	int32_t i32Gain;
	
	i32Gain = ANA->PGAGAIN;
	i32Gain >>= (ANA_PGAGAIN_GAINREAD_Pos);
	return (i32Gain*75-1200);
}

/*@}*/ /* end of group ISD9100_ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_ADC_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

