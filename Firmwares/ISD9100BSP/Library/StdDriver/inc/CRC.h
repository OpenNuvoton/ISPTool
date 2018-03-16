/**************************************************************************//**
 * @file     crc.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/15 2:52p $
 * @brief    ISD9100 CRC Driver Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

#ifndef __CRC_H__
#define __CRC_H__

#ifdef __cplusplus
extern "C"
{
#endif

	/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_CRC_Driver CRC Driver
  @{
*/

/** @addtogroup ISD9100_CRC_EXPORTED_TYPEDEF CRC Exported Type Defines
  @{
*/

#define	CRC_MSB (0)
#define	CRC_LSB (CRC_CTL_MODE_Msk)

/*@}*/ /* end of group ISD9100_CRC_EXPORTED_CONSTANTS */

/** @addtogroup ISD9100_CRC_EXPORTED_FUNCTIONS CRC Exported Functions
  @{
*/
	
/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t CRC_Open(void);
int32_t CRC_Init(uint32_t eLSB, int32_t i32PacketLen);
int16_t CRC_Calc( uint32_t *Data, int32_t i32PacketLen);
void CRC_Close(void);

/*@}*/ /* end of group ISD9100_CRC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_CRC_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
