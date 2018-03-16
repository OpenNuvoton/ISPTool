/**************************************************************************//**
 * @file     crc.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/15 2:52p $
 * @brief    ISD9100 CRC driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

//* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "ISD9100.h"

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_CRC_Driver CRC Driver
  @{
*/

/** @addtogroup ISD9100_CRC_EXPORTED_FUNCTIONS CRC Exported Functions
  @{
*/

/**
  * @brief    Enable CRC Generator
  * @retval   E_SUCCESS    Success
  * @retval   -1   The specified parameters are invalid
  */
int32_t CRC_Open(void)
{
		CLK_EnableModuleClock(CRC_MODULE);
		SYS->IPRST1 = (SYS->IPRST1 | SYS_IPRST1_CRCRST_Msk);
		SYS->IPRST1 = (SYS->IPRST1 & (~SYS_IPRST1_CRCRST_Msk));

    return E_SUCCESS;
}

/**
  * @brief    Initialize CRC for new packet
  * @param    eLSB   					Specify LSB/MSB mode, CRC_MSB or CRC_LSB
  * @param    i32PacketLen    Specify length of packet to be processed.
  * @retval   E_SUCCESS    Success
  * @retval   -1   The specified parameters are invalid
  */
int32_t CRC_Init(uint32_t eLSB, int32_t i32PacketLen)
{
   
    CRC->CTL = (CRC->CTL | eLSB);
    if(i32PacketLen > 512)
        return -1;
    if(i32PacketLen%2==1)
        return -1;
    CRC->CTL = (CRC->CTL & (~CRC_CTL_PKTLEN_Msk) | (i32PacketLen-1)); 

    return E_SUCCESS;
}

/**
  * @brief    Process the data pointed to by Initialize CRC for new packet 
  * @param    Data   					Pointer to data. Word Aligned
  * @param    i32PacketLen    Specify length of packet to be processed.
  * @retval   E_SUCCESS    	Success
  * @retval   CRC   				Calculation of data. 
  */
int16_t CRC_Calc( uint32_t *Data, int32_t i32PacketLen)
{
   
    while( i32PacketLen > 0){
		CRC->DAT = *Data++;
		i32PacketLen-=4;
	}

    return CRC->CHECKSUM;
}

/**
  * @brief    Close the CRC Generator
  */
void CRC_Close(void)
{
		SYS->IPRST1 = (SYS->IPRST1 | SYS_IPRST1_CRCRST_Msk);
		SYS->IPRST1 = (SYS->IPRST1 & (~SYS_IPRST1_CRCRST_Msk));
		CLK_DisableModuleClock(CRC_MODULE);
}

/*@}*/ /* end of group ISD9100_CRC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_CRC_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
