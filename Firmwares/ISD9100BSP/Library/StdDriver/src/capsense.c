/**************************************************************************//**
 * @file     CapSense.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/29 11:00a $
 * @brief    ISD9100 Series capture sense driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "ISD9100.h"

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/
	
/** @addtogroup ISD9100_CAPSENSE_Driver CapSense Driver
  @{
*/
	
/** @addtogroup ISD9100_CAPSENSE_EXPORTED_FUNCTIONS CapSense Exported Functions
  @{
*/

/**
  * @brief  This function select current source value.
  * @param  u16SrcSel is capture sense's cycle counts.
  * - \ref CAPSENSE_CURCTL0_VALSEL_500NA
  * - \ref CAPSENSE_CURCTL0_VALSEL_1000NA
  * - \ref CAPSENSE_CURCTL0_VALSEL_2500NA
  * - \ref CAPSENSE_CURCTL0_VALSEL_5000NA
  * @return None
  */
void CapSense_SelectCurrentSourceValue( uint16_t u16SrcSel )
{
	ANA->CURCTL0 &= (~ANA_CURCTL0_VALSEL_Msk);
	ANA->CURCTL0 |= u16SrcSel;
}

/**
  * @brief  This function set cycle counts before generating an interrupt 
  * @param  u32Count is capture sense's cycle counts.
  * @param  u16LowTime is cycle low time.
  * @return None
  */
void CapSense_SetCycleCounts( uint32_t u32Count, uint16_t u16LowTime )
{
	ANA->CAPSCTL &= (~ ( ANA_CAPSCTL_CYCLECNT_Msk | ANA_CAPSCTL_LOWTIME_Msk )) ;
    ANA->CAPSCTL |= ( u32Count << ANA_CAPSCTL_CYCLECNT_Pos );
	ANA->CAPSCTL |= u16LowTime;
}

/**
  * @brief  This function reset capsense count then release/activate capsense count.
  * @return None
  */
void CapSense_ResetCounter(void)
{
	ANA->CAPSCTL |= ANA_CAPSCTL_RSTCNT_Msk;
	ANA->CAPSCTL &= (~ANA_CAPSCTL_RSTCNT_Msk) ;
}

/**
  * @brief  This function return capsense counter.
  * @return Capture sense counter.
  */
uint32_t CapSense_GetCounter(void)
{
    return (ANA->CAPSCNT);
}

/*@}*/ /* end of group ISD9100_CAPSENSE_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_CAPSENSE_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
