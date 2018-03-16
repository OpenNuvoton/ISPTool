/**************************************************************************//**
 * @file     CapSense.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/07/07 11:00a $
 * @brief    ISD9100 Series Capture Sense Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
 
#ifndef __CAPSENSE_H__
#define __CAPSENSE_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_CAPSENSE_Driver CapSense Driver
  @{
*/

/** @addtogroup ISD9100_CAPSENSE_EXPORTED_CONSTANTS CapSense Exported Constants
  @{
*/
	
/*---------------------------------------------------------------------------------------------------------*/
/*  CURCTL0 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CAPSENSE_CURCTL0_VALSEL_500NA      ( 0x0UL<<ANA_CURCTL0_VALSEL_Pos )
#define CAPSENSE_CURCTL0_VALSEL_1000NA     ( 0x1UL<<ANA_CURCTL0_VALSEL_Pos )
#define CAPSENSE_CURCTL0_VALSEL_2500NA     ( 0x2UL<<ANA_CURCTL0_VALSEL_Pos )
#define CAPSENSE_CURCTL0_VALSEL_5000NA     ( 0x3UL<<ANA_CURCTL0_VALSEL_Pos )

/*---------------------------------------------------------------------------------------------------------*/
/*  CURCTL1 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CAPSENSE_CURCTL0_CURSRCEN_GPIOB0    ( 0x1UL<<0 )
#define CAPSENSE_CURCTL0_CURSRCEN_GPIOB1    ( 0x1UL<<1 ) 
#define CAPSENSE_CURCTL0_CURSRCEN_GPIOB2    ( 0x1UL<<2 ) 
#define CAPSENSE_CURCTL0_CURSRCEN_GPIOB3    ( 0x1UL<<3 ) 	
#define CAPSENSE_CURCTL0_CURSRCEN_GPIOB4    ( 0x1UL<<4 )
#define CAPSENSE_CURCTL0_CURSRCEN_GPIOB5    ( 0x1UL<<5 )
#define CAPSENSE_CURCTL0_CURSRCEN_GPIOB6    ( 0x1UL<<6 ) 
#define CAPSENSE_CURCTL0_CURSRCEN_GPIOB7    ( 0x1UL<<7 ) 

/*---------------------------------------------------------------------------------------------------------*/
/*  CTRL constant definitions.                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define CAPSENSE_CTRL_LOWTIME_1CYCLES       ( 0x0UL )
#define CAPSENSE_CTRL_LOWTIME_2CYCLES       ( 0x1UL )
#define CAPSENSE_CTRL_LOWTIME_8CYCLES       ( 0x2UL )
#define CAPSENSE_CTRL_LOWTIME_16CYCLES      ( 0x3UL )

/*@}*/ /* end of group ISD9100_CAPSENSE_EXPORTED_CONSTANTS */

/** @addtogroup ISD9100_CAPSENSE_EXPORTED_FUNCTIONS CapSense Exported Functions
  @{
*/

#define CAPSENSE_ENABLE_CURRENT_SOURCE_PIN( u16Pin )               ( ANA->CURCTL0 |= u16Pin )
#define CAPSENSE_ENABLE_INTERRUPT()                                ( ANA->CAPSCTL |= ANA_CAPSCTL_INTEN_Msk )
#define CAPSENSE_DISABLE_INTERRUPT()                               ( ANA->CAPSCTL &= (~ANA_CAPSCTL_INTEN_Msk) )
#define CAPSENSE_ENABLE()                                          ( ANA->CAPSCTL |= ANA_CAPSCTL_CAPSEN_Msk )
#define CAPSENSE_DISABLE()                                         ( ANA->CAPSCTL &= (~ANA_CAPSCTL_CAPSEN_Msk) )

void     CapSense_SelectCurrentSourceValue( uint16_t u16SrcSel );
void     CapSense_SetCycleCounts( uint32_t u32Count, uint16_t u16LowTime );

void     CapSense_ResetCounter(void);
uint32_t CapSense_GetCounter(void);

/*@}*/ /* end of group ISD9100_CAPSENSE_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_CAPSENSE_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
