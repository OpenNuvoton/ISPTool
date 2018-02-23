/**************************************************************************//**
 * @file     capsense.c
 * @version  V1.01
 * $Revision: 1 $
 * $Date: 17/07/21 2:35p $
 * @brief    I91200 capture sense driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/ 

/** @addtogroup I91200_CAPSENSE_Driver CapSense Driver
  @{
*/

/** @addtogroup I91200_CAPSENSE_EXPORTED_FUNCTIONS CapSense Exported Functions
  @{
*/

/**
  * @brief  This function is set scan 'pin map' mode(GPIOB0~GPIOB15).
  * @param  u32PinMap is pin map for scan.(ex.BIT0|BIT1|BIT15 etc.)
  * @return None
  */
void CapSense_SetScanPinMap(uint32_t u32PinMap) {
	CSCAN->CTRL |= CSCAN_CTRL_MODE0_Msk;
	CSCAN->CYCCNT = ((CSCAN->CYCCNT&(~CSCAN_CYCCNT_MASK_Msk))|(u32PinMap<<CSCAN_CYCCNT_MASK_Pos));
	CSCAN->AGPIO |= u32PinMap;
}

/**
  * @brief  This function is set scan 'one pin' mode(GPIOB0,GPIOB2,...GPIOB15 etc.).
  * @param  u8Pin is one pin for scan.(ex.BIT0,BIT1, ..BIT15 etc.)
  * @return None
  */
void CapSense_SetScanOnePin(uint8_t u8Pin) {
	CSCAN->CTRL &= ~CSCAN_CTRL_MODE0_Msk;
	CSCAN->CTRL = ((CSCAN->CTRL&(~CSCAN_CTRL_SEL_Msk))|u8Pin);
}

/**
  * @brief  This function is to clear scan 'pin map'.
  * @param  u32PinMap is pin map for scan.(ex.BIT0|BIT1|BIT15 etc.)
  * @return None
  */
void CapSense_ClearScanPin(uint32_t u32PinMap) {
	CSCAN->AGPIO &= ~(u32PinMap);
}

/**
  * @brief  This function is get current scan mode
  * @return 1(pin map),0(one pin)
  */
uint8_t CapSense_GetScanMode(void) {
	return ((CSCAN->CTRL&CSCAN_CTRL_MODE0_Msk)?1:0);
}

/**
  * @brief  This function is get capture values in pin map mode.
  * @param  pu16Value is capture values buffer.
  * @return None.
  */
void CapSense_GetPinMapValue(uint16_t* pu16Value)
{
	uint8_t u8i;

	for( u8i=0; u8i<8; u8i++ )
	{
		pu16Value[u8i*2] = SBRAM->D[u8i]&0xFFFF;
		pu16Value[u8i*2+1] = SBRAM->D[u8i]>>16;
	}
}

/**
  * @brief  This function is to get capture values in one pin mode.
  * @return Capture values.
  */
uint16_t CapSense_GetOnePinValue(void)
{
	return CSCAN->COUNT&0xFFFF;
}

/*@}*/ /* end of group I91200_CAPSENSE_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_CAPSENSE_Driver */


/** @addtogroup I91200_OPA_Driver OPA Driver
  @{
*/

/** @addtogroup I91200_OPA_EXPORTED_FUNCTIONS OPA Exported Functions
  @{
*/

/**
  * @brief  This function is to enable OPA and configure input, output and feedback.
  * @param  u8Num is the number of which OPA to enable.
			- \ref OPA0
			- \ref OPA1
  * @param  u32PosInput the input configuration.
			- \ref OPA_POSIN_PIN
			- \ref OPA_POSIN_VBIAS_HI
			- \ref OPA_POSIN_VBIAS_MID
			- \ref OPA_POSIN_VBIAS_LO
			- \ref OPA_POSIN_NOCONNECT
			- \ref OPA_NEGIN_OPA
  * @param  u32NegInput the input configuration.
			- \ref OPA_NEGIN_PIN
			- \ref OPA_NEGIN_NOCONNECT
			- \ref OPA_NEGIN_OPA
  * @param  u32OutputEnable 1 to enable output, 0 to disable output.
  * @param  u32FeedbackEnable 1 to enable feedback, 0 to disable feedback.
  * @return TRUE: OPA enabled, FALSE: OPA enable failed.
  */
uint8_t OPA_Enable(uint8_t u8Num, uint32_t u32PosInput, uint32_t u32NegInput, uint32_t u32OutputEnable, uint32_t u32FeedbackEnable)
{
	uint32_t u32Input = u32PosInput | u32NegInput;
	uint32_t u32Shift = OPA_SHIFT * u8Num;
	
	switch(u8Num)
	{
		case OPA0:
		{
			CSCAN->OPACTL = (CSCAN->OPACTL & ~CSCAN_OPACTL_A0OEN_Msk) | (u32OutputEnable << CSCAN_OPACTL_A0OEN_Pos);
			CSCAN->OPACTL = (CSCAN->OPACTL & ~CSCAN_OPACTL_A0O2N_Msk) | (u32FeedbackEnable << CSCAN_OPACTL_A0O2N_Pos);
		}
		break;
		case OPA1:
		{
			CSCAN->OPACTL = (CSCAN->OPACTL & ~OPA_A0O2A1_MASK) | (u32Input & OPA_A0O2A1_MASK);
			CSCAN->OPACTL = (CSCAN->OPACTL & ~CSCAN_OPACTL_A1OEN_Msk) | (u32OutputEnable << CSCAN_OPACTL_A1OEN_Pos);
			CSCAN->OPACTL = (CSCAN->OPACTL & ~CSCAN_OPACTL_A1O2N_Msk) | (u32FeedbackEnable << CSCAN_OPACTL_A1O2N_Pos);
		}
		break;
		default:
			return FALSE;
	}

	if(u32Input & CSCAN_OPACTL_A0PSEL_Msk)
		OPA_EN_VREF();
	
	u32Input = (u32Input & OPA_INPUT_MASK) << u32Shift;
	
	CSCAN->OPACTL = (CSCAN->OPACTL & ~(OPA_INPUT_MASK << u32Shift)) | u32Input;
	
	CSCAN->OPACTL |= CSCAN_OPACTL_A0EN_Msk << u32Shift;
	
	return TRUE;
}

/**
  * @brief  This function is to disable OPA.
  * @param  u8Num is the number of which OPA to disable.
			- \ref OPA0
			- \ref OPA1
  * @return None.
  */
void OPA_Disable(uint8_t u8Num)
{
	OPA_DISABLE(u8Num);
}

/**
  * @brief  This function is to enable and set OPA1 gain.
  * @param  u32Gain the gain configuration of OPA1.
			- \ref OPA_GAIN_1
			- \ref OPA_GAIN_8
			- \ref OPA_GAIN_16
			- \ref OPA_GAIN_24
			- \ref OPA_GAIN_32
			- \ref OPA_GAIN_40
			- \ref OPA_GAIN_48
			- \ref OPA_GAIN_56
  * @return None.
  */
void OPA_GainEnable(uint32_t u32Gain)
{
	OPA_DIS_NEGINTPIN(OPA1);
	OPA_OPA1_SET_GAIN(u32Gain);
	OPA_DI_FEEDBACK(OPA1);
	OPA_OPA1_EN_GAIN();
}

/**
  * @brief  This function is to disable OPA1 gain.
  * @return None.
  */
void OPA_GainDisable()
{
	OPA_OPA1_DIS_GAIN();
}

/*@}*/ /* end of group I91200_OPA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_OPA_Driver */


/** @addtogroup I91200_CMP_Driver CMP Driver
  @{
*/

/** @addtogroup I91200_CMP_EXPORTED_FUNCTIONS CMP Exported Functions
  @{
*/


/**
  * @brief  This function is to enable and configure comaprator.
  * @param  u8Num is the number of which OPA to enable.
  *			- \ref CMP1
  *			- \ref CMP2
  * @param  u32PInput the non-inverting input configuration.
			- \ref CMP1_POSIN_CNP
			- \ref CMP1_POSIN_OPA0
			- \ref CMP1_POSIN_OPA1
			- \ref CMP2_POSIN_C2P
			- \ref CMP2_POSIN_VL0
  * @param  u32NInput the inverting input configuration.
			- \ref CMP1_NEGIN_C1N
			- \ref CMP1_NEGIN_VH0
			- \ref CMP2_NEGIN_CNP
			- \ref CMP2_NEGIN_OPA0
			- \ref CMP2_NEGIN_OPA1
  * @param  u32OutputEnable 1 to enable output, 0 to disable output.
  * @return Capture values.
  */
uint8_t CMP_Enable(uint8_t u8Num, uint32_t u32PInput, uint32_t u32NInput, uint32_t u32OutputEnable)
{
	uint8_t u8Shift = (u8Num - CMP1) * CMP_SHIFT;
	
	if(u32PInput & CSCAN_CMPCTL_CNPSEL_Msk)
		CSCAN->OPACTL = (CSCAN->OPACTL & ~(CSCAN_OPACTL_A1O2CIN_Msk | CSCAN_OPACTL_A0O2CIN_Msk)) | (u32PInput & (CSCAN_OPACTL_A1O2CIN_Msk | CSCAN_OPACTL_A0O2CIN_Msk));
	
	if(u32NInput & CSCAN_CMPCTL_CNPSEL_Msk)
		CSCAN->OPACTL = (CSCAN->OPACTL & ~(CSCAN_OPACTL_A1O2CIN_Msk | CSCAN_OPACTL_A0O2CIN_Msk)) | (u32NInput & (CSCAN_OPACTL_A1O2CIN_Msk | CSCAN_OPACTL_A0O2CIN_Msk));
	
	u32PInput &= ~(CSCAN_OPACTL_A1O2CIN_Msk | CSCAN_OPACTL_A0O2CIN_Msk);
	u32NInput &= ~(CSCAN_OPACTL_A1O2CIN_Msk | CSCAN_OPACTL_A0O2CIN_Msk);
	
	switch(u8Num)
	{
		case CMP1:
			CSCAN->CMPCTL &= ~(CSCAN_CMPCTL_CNPSEL_Pos | CSCAN_CMPCTL_C1NSEL_Msk);
		break;
		case CMP2:
			CSCAN->CMPCTL &= ~(CSCAN_CMPCTL_CNPSEL_Pos | CSCAN_CMPCTL_C2PSEL_Msk);
		break;
		default:
			return FALSE;
	}
	
	CSCAN->CMPCTL |= u32PInput | u32NInput;
	CSCAN->CMPCTL |= ((u32OutputEnable << CSCAN_CMPCTL_C1OUTEN_Pos) | CSCAN_CMPCTL_CMP1EN_Msk) << u8Shift;
	
	return TRUE;
}

/**
  * @brief  This function is to disable comparator.
  * @param  u8Num is the number of which comparator to disable.
			- \ref CMP1
			- \ref CMP2
  * @return None.
  */
void CMP_Disable(uint8_t u8Num)
{
	if(u8Num == CMP1)
		CSCAN->CMPCTL &= ~(CSCAN_CMPCTL_CMP1EN_Msk);
	else if(u8Num == CMP2)
		CSCAN->CMPCTL &= ~(CSCAN_CMPCTL_CMP2EN_Msk);
}

/**
  * @brief  This function is to enable comaprator interrupt.
  * @param  u8Num is the number of which OPA to enable.
  *			- \ref CMP1
  *			- \ref CMP2
  * @param  u32Mode the interrupt mode configuration.
			- \ref CMP_RISING_EDGE
			- \ref CMP_FALLING_EDGE
			- \ref CMP_DUAL_EDGE
  * @return Capture values.
  */
void CMP_IntEnable(uint8_t u8Num, uint32_t u32Mode)
{
	switch(u8Num)
	{
		case CMP1:
			CSCAN->CMPCTL |= CSCAN_CMPCTL_C1INTEN_Msk | (u32Mode << CSCAN_CMPCTL_CMPES_Pos);
		break;
		case CMP2:
			CSCAN->CMPCTL |= CSCAN_CMPCTL_C2INTEN_Msk | (u32Mode << CSCAN_CMPCTL_CMPES_Pos);
		break;
		default:
			return;
	}
}

/**
  * @brief  This function is to disable comparator interrupt.
  * @param  u8Num is the number of which comparator to disable.
			- \ref CMP1
			- \ref CMP2
  * @return None.
  */
void CMP_IntDisable(uint8_t u8Num)
{
	switch(u8Num)
	{
		case CMP1:
			CSCAN->CMPCTL &= ~CSCAN_CMPCTL_C1INTEN_Msk;
		break;
		case CMP2:
			CSCAN->CMPCTL &= ~CSCAN_CMPCTL_C2INTEN_Msk;
		break;
		default:
			return;
	}
}

/*@}*/ /* end of group I91200_CMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_CMP_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
