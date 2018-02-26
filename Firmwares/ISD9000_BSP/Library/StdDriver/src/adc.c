/**************************************************************************//**
 * @file     adc.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/03/07 02:35p $
 * @brief    ISD9000 ADC driver source file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include  "ISD9000.h"

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_ADC_Driver ADC Driver
  @{
*/

/** @addtogroup ISD9000_ADC_EXPORTED_FUNCTIONS ADC Exported Functions
  @{
*/

/**
  * @brief      ADC open function
  * @param[in]  adc Base address of ADC module
  * @return     None
  * @details    
  */
void ADC_Open(ADC_T *adc)
{

}

/**
  * @brief      ADC close function
  * @param[in]  adc Base address of ADC module
  * @return     None
  * @details   
  */
void ADC_Close(ADC_T *adc)
{   
      
}

/**
  * @brief      Enable ADC interrupts
  * @param[in]  adc Base address of ADC module
  * @param[in]  u32Mask is interrupt flags.
  *             - \ref ADC_ADF_INT 
  *             - \ref ADC_CMP0_INT 
  *             - \ref ADC_CMP1_INT
  * @return     None.
  * @details    ADC_ADF_INT is generated whenever A/D conversion end.
  *             ADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             ADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask)
{
	if (u32Mask&ADC_ADF_INT)
		adc->CTL |= ADC_CTL_ADCIE_Msk;

	if (u32Mask&ADC_CMP0_INT)
		adc->CMP[0] |= ADC_CMP_ADCMPIE_Msk;
	
	if (u32Mask&ADC_CMP1_INT)
		adc->CMP[1] |= ADC_CMP_ADCMPIE_Msk;
}  

/**
  * @brief      Disable ADC interrupts
  * @param[in]  adc Base address of ADC module
  * @param[in]  u32Mask is interrupt flags.
  *             - \ref ADC_ADF_INT 
  *             - \ref ADC_CMP0_INT 
  *             - \ref ADC_CMP1_INT
	* @return     None.
  * @details    ADC_ADF_INT is generated whenever A/D conversion end.
  *             ADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             ADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask)
{
	if (u32Mask&ADC_ADF_INT)
		adc->CTL &= ~ADC_CTL_ADCIE_Msk;

	if (u32Mask&ADC_CMP0_INT)
		adc->CMP[0] &= ~ADC_CMP_ADCMPIE_Msk;
	
	if (u32Mask&ADC_CMP1_INT)
		adc->CMP[1] &= ~ADC_CMP_ADCMPIE_Msk;
}

/**
  * @brief      Set PGA gain
  * @param[in]  adc Base address of ADC module
  * @param[in]  i32PGAGainIndB is gain value(~18dB~45dB)             
  * @return     Real gain value
  * @details    In ISD9000F064 chip, it's range is from -18dB to 45dB.
  */
int32_t ADC_SetPGAGaindB(ADC_T *adc, int32_t i32PGAGainIndB)
{
	if(i32PGAGainIndB < -18)
		i32PGAGainIndB = -18;
	else if(i32PGAGainIndB > 45)
		i32PGAGainIndB = 45;
	
	adc->PGCTL = (adc->PGCTL&(~ADC_PGCTL_PGA_SEL_Msk))|((i32PGAGainIndB+18)<<ADC_PGCTL_PGA_SEL_Pos);

	return i32PGAGainIndB;
}

/**
  * @brief      Get PGA gain
  * @param[in]  adc Base address of ADC module                         
  * @return     Real gain value(-18dB ~ 45dB)
  * @details    In ISD9000F064 chip, it's range is from -18dB to 45dB.
  */
int32_t ADC_GetPGAGaindB(ADC_T *adc)
{
	return (((adc->PGCTL&ADC_PGCTL_PGA_SEL_Msk)>>ADC_PGCTL_PGA_SEL_Pos)-18);
}

/**
  * @brief      Enable microphone bias generator
  * @param      u32BiasSel is microphone bias voltage
    *             - \ref ADC_MICBSEL_0_VCCA
	*             - \ref ADC_MICBSEL_65_VCCA
	*             - \ref ADC_MICBSEL_70_VCCA
	*             - \ref ADC_MICBSEL_50_VCCA  
  * @return     void
  * @details    The user should consider the microphone manufacturers specification 
  *             in deciding on the optimum MICBIAS voltage to use. Generally, a microphone
  *             will require a current of 0.1mA to a maximum 0.5mA and a voltage of 1V to 3V across it.
  */
void ADC_EnableMICBias(ADC_T *adc, uint32_t u32BiasSel)  
{
	adc->PGCTL = (adc->PGCTL&(~ADC_PGCTL_MICB_VSEL_Msk))|(ADC_PGCTL_MICB_EN_Msk|u32BiasSel);
} 

/**
  * @brief      Disable microphone bias generator
  * @return     None
  */
void ADC_DisableMICBias(ADC_T *adc)  
{
	adc->PGCTL &= (~ADC_PGCTL_MICB_EN_Msk);
}

/**
  * @brief      ADC conversion clock number 
  * @param[in]  adc Base address of ADC module
  * @param[in]  u32Cycle ADC Conversion clock number = (u32Cycle + 1)
  * @return     None
  * @details    u32Cycle valid range is from 12~128.
  */
void ADC_SetConversionCycle(ADC_T *adc, uint32_t u32Cycle)
{
	u32Cycle--;
	if (u32Cycle<11)
		u32Cycle = 11;
	if (u32Cycle>127)
		u32Cycle = 127;
	adc->HWPARA = (adc->HWPARA&~ADC_HWPARA_CONV_N_Msk) | (u32Cycle << ADC_HWPARA_CONV_N_Pos);
}

/*@}*/ /* end of group ISD9000_ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_ADC_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
