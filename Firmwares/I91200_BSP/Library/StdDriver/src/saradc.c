/**************************************************************************//**
 * @file     saradc.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/08/23 2:35p $
 * @brief    I91200 SARADC driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

#define SARADC_MAX_CLK 						(10 * 1000 * 1000) 	// 10MHz 
#define SARADC_CONVERSION_CYCLE 			(28)

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/ 

/** @addtogroup I91200_SARADC_Driver SARADC Driver
  @{
*/


/** @addtogroup I91200_SARADC_EXPORTED_FUNCTIONS SARADC Exported Functions
  @{
*/

/**
  * @brief      SARADC open function
  * @return     None
  * @details    
  */
void SARADC_Open(void)
{

}

/**
  * @brief      SARADC close function
  * @return     None
  * @details   
  */
void SARADC_Close(void)
{   
      
}

/**
  * @brief      Enable SARADC interrupts
  * @param[in]  u32Mask is interrupt enabled bits according to defined int falg
  *             - \ref SARADC_ADF_INT 
  *             - \ref SARADC_CMP0_INT 
  *             - \ref SARADC_CMP1_INT
  * @return     None.
  * @details    SARADC_ADF_INT is generated whenever A/D conversion end.
  *             SARADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             SARADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void SARADC_EnableInt(uint32_t u32Mask)
{
	if (u32Mask&SARADC_ADF_INT)
		SARADC->CTL |= SARADC_CTL_ADCIE_Msk;

	if (u32Mask&SARADC_CMP0_INT)
		SARADC->CMP[0] |= SARADC_CMP_ADCMPIE_Msk;
	
	if (u32Mask&SARADC_CMP1_INT)
		SARADC->CMP[1] |= SARADC_CMP_ADCMPIE_Msk;
}  

/**
  * @brief      Disable SARADC interrupts
  * @param[in]  u32Mask is interrupt flags.
  *             - \ref SARADC_ADF_INT 
  *             - \ref SARADC_CMP0_INT 
  *             - \ref SARADC_CMP1_INT
	* @return     None.
  * @details    SARADC_ADF_INT is generated whenever A/D conversion end.
  *             SARADC_CMP0_INT is generated whenever compared 0 matches
  *             the setting of count and condition.
  *             SARADC_CMP1_INT is generated whenever compared 1 matches
  *             the setting of count and condition.             
  */
void SARADC_DisableInt(uint32_t u32Mask)
{
	if (u32Mask&SARADC_ADF_INT)
		SARADC->CTL &= ~SARADC_CTL_ADCIE_Msk;

	if (u32Mask&SARADC_CMP0_INT)
		SARADC->CMP[0] &= ~SARADC_CMP_ADCMPIE_Msk;
	
	if (u32Mask&SARADC_CMP1_INT)
		SARADC->CMP[1] &= ~SARADC_CMP_ADCMPIE_Msk;
}

/**
  * @brief      Get SARADC module clock source
  * @return     SARADC clock source.            
  */
uint32_t SARADC_GetClockSrc(void)
{
	uint32_t u32ClkSrc; 
	
	switch (CLK->CLKSEL1&CLK_CLKSEL1_SARADCSEL_Msk)
	{
		case CLK_CLKSEL1_SARADCSEL_HCLK:
			u32ClkSrc = SystemCoreClock;
			break;
		case CLK_CLKSEL1_SARADCSEL_LIRC:
			u32ClkSrc = 10000UL;
			break;
		case CLK_CLKSEL1_SARADCSEL_HIRC:
			u32ClkSrc = 49152000UL;
			break;
		case CLK_CLKSEL1_SARADCSEL_LXT:
			u32ClkSrc = 32678UL;
			break;
		default:
			u32ClkSrc = 0;
			break;
	}
	
	return u32ClkSrc;
}

/**
  * @brief      Set SARADC Sample rate
  * @param[in]  u32SampleRate is set sample rate.
  * @return     Real sample rate.            
  */
uint32_t SARADC_SetSampleRate(uint32_t u32SampleRate)
{
	uint32_t u32ClkSrc, u32ClkDiv, u32AdcClk;
	
	// Need 14 clocks to covert
	u32AdcClk = u32SampleRate*SARADC_CONVERSION_CYCLE;
	
	if (u32AdcClk > SARADC_MAX_CLK || u32AdcClk == 0 )
		return 0;
	
	u32ClkSrc = SARADC_GetClockSrc();
	
	u32ClkDiv = ((u32ClkSrc + (u32AdcClk>>1))/(u32AdcClk));
	
	if ((u32ClkDiv == 0) || (u32ClkDiv>0xff))
		return 0;
	
	CLK->CLKDIV0 = (CLK->CLKDIV0&~CLK_CLKDIV0_SARADCDIV_Msk)|((u32ClkDiv - 1)<<CLK_CLKDIV0_SARADCDIV_Pos);
	
	return(u32ClkSrc/u32ClkDiv/SARADC_CONVERSION_CYCLE);
}

/**
  * @brief      Get SARADC Sample rate
  * @return     Real sample rate.            
  */
uint32_t SARADC_GetSampleRate(void)
{
	uint32_t u32ClkSrc;
	u32ClkSrc = SARADC_GetClockSrc();
	return((u32ClkSrc/(((CLK->CLKDIV0&CLK_CLKDIV0_SARADCDIV_Msk)>>CLK_CLKDIV0_SARADCDIV_Pos) + 1))/SARADC_CONVERSION_CYCLE);
}

/*@}*/ /* end of group I91200_SARADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_SARADC_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
