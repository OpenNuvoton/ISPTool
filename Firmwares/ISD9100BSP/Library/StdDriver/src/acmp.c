/**************************************************************************//**
 * @file     ACMP.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/07/04 16:25p $
 * @brief    ISD9100 ACMP driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include  "ISD9100.h"
/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_ACMP_Driver ACMP Driver
  @{
*/


/** @addtogroup ISD9100_ACMP_EXPORTED_FUNCTIONS ACMP Exported Functions
  @{
*/

/**
  * @brief      Configure the specified ACMP module
  *
  * @param[in]  acmp The base address of ACMP module
  * @param[in]  u32ChNum comparator number, could be 0 or 1.
  * @param[in]  u32NegSrc is comparator negative input selection source
  *            - \ref ACMP_CMP0VNEG_VBG
  *            - \ref ACMP_CMP0VNEG_VMID
  *            - \ref ACMP_CMP1VNEG_GPB7
  *            - \ref ACMP_CMP1VNEG_VBG
  *
  * @param[in]  u32PosPin CMP0 positive input pin
  *            - \ref ACMP_CH0_POSPIN_GPB0
  *            - \ref ACMP_CH0_POSPIN_GPB1
  *            - \ref ACMP_CH0_POSPIN_GPB2
  *            - \ref ACMP_CH0_POSPIN_GPB3
  *            - \ref ACMP_CH0_POSPIN_GPB4
  *            - \ref ACMP_CH0_POSPIN_GPB5
  *            - \ref ACMP_CH0_POSPIN_GPB6
  *            - \ref ACMP_CH0_POSPIN_GPB7
  * @return None
  * @note      - Multi-function pin needs to be configured and set as input mode.
  *            - MID block must be powered up if using VMID, programmer can 
  *              call ADC_ENABLE_VMID macro to enable VMID.
  */
void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32PosPin)
{
	if (u32ChNum == 0)
	{
		ACMP_CH0SELECT_P(acmp,u32PosPin);
	}
	if (u32ChNum)
	{
		ACMP_SET_NEG_SRC(acmp,1,u32NegSrc);
		ACMP_ENABLE(acmp,1);
	}else
	{
		ACMP_SET_NEG_SRC(acmp,0,u32NegSrc);
		ACMP_ENABLE(acmp,0);
	}
}

/**
  * @brief      This function close comparator
  *
  * @param[in]  acmp The base address of ACMP module
  * @param[in]  u32ChNum comparator number.
  * @return     None
  */
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum)
{
	if (u32ChNum)
		ACMP_DISABLE(acmp,1);
	else
		ACMP_DISABLE(acmp,0);
}

/*@}*/ /* end of group ISD9100_ACMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_ACMP_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

