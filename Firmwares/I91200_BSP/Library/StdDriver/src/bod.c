/*************************************************************************//**
 * @file     BOD.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/08/02 7:06p $
 * @brief    I91200 BOD driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_BOD_Driver BOD Driver
  @{
*/


/** @addtogroup I91200_BOD_EXPORTED_FUNCTIONS BOD Exported Functions
  @{
*/

/**
  * @brief      This function will enable BOD
  * @param[in]  u8Mode is BOD_BODEN_CONTINUOUS or BOD_BODEN_TIME_MULTIPLEXED.
  * @param[in]  u8BODLevel is BOD Voltage Level.
  * @return     None
  */
void BOD_Open(uint8_t u8Mode, uint8_t u8BODLevel)
{
	uint8_t u8Lock = SYS_Unlock();
	
    //TALARM and BOD operation require that the OSC16K low power oscillator is enabled
    CLK->PWRCTL &= (~CLK_PWRCTL_LIRCDPDEN_Msk);
    BOD->BODSEL |= u8BODLevel;
    BOD->BODCTL |= u8Mode;
	
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function will disable BOD
  * @return None
  */
void BOD_Close(void)
{
    BOD->BODCTL &= BOD_BODEN_DISABLE;
}

/**
  * @brief      This function will set up to take periodic samples of the supply voltage
  * @param[in]  u8OnDUR is time BOD detector is Active. (DURTON+1) * 100us. Minimum value is 1. (default is 400us)
  * @param[in]  u16OffDUR is time BOD detector is Off. (DURTOFF+1)*100us . Minimum value is 7. (default is 99.6ms)
  * @return     None
  */
void BOD_SetDetectionTime(uint8_t u8OnDUR, uint16_t u16OffDUR)
{
	BOD->BODDTMR |= (u8OnDUR<<BOD_BODDTMR_DURTON_Pos)&BOD_BODDTMR_DURTON_Msk;
	BOD->BODDTMR |= (u16OffDUR&BOD_BODDTMR_DURTOFF_Msk);
}




/*@}*/ /* end of group I91200_BOD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_BOD_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/

