/*************************************************************************//**
 * @file     BOD.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/06/26 7:06p $
 * @brief    ISD9000 BOD driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include  "ISD9000.h"
/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_BOD_Driver BOD Driver
  @{
*/


/** @addtogroup ISD9000_BOD_EXPORTED_FUNCTIONS BOD Exported Functions
  @{
*/

/**
  * @brief      This function will enable BOD
  * @param[in]  u8Mode is BOD_RESET_MODE or BOD_INTERRUPT_MODE.
  * @param[in]  u8BODLevel is BOD Voltage Level.
  * @return     None
  */
void BOD_Open(uint8_t u8Mode,uint8_t u8BODLevel)
{
	uint8_t u8Lock = SYS_Unlock();
	
  //BOD operation require that the OSC16K low power oscillator is enabled
  CLK->PWRCTL &= (~CLK_PWRCTL_LIRCDPDEN_Msk);

	SYS->BODCTL |= (SYS_BODCTL_BOD_EN_Msk);
	
	SYS->BODCTL &= (~SYS_BODCTL_BOD_LVL_Msk);
  SYS->BODCTL |= (u8BODLevel<< SYS_BODCTL_BOD_LVL_Pos);
	
	SYS->BODCTL &= (~SYS_BODCTL_BOD_RSTEN_Msk);
	SYS->BODCTL |= (u8Mode<<SYS_BODCTL_BOD_RSTEN_Pos);
	
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function will disable BOD
  * @return None
  */
void BOD_Close(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL &= (~SYS_BODCTL_BOD_EN_Msk);
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function will enable LVR
  * @return None
  */
void BOD_LVR_Enable(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL |= SYS_BODCTL_LVR_EN_Msk;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function will disable LVR
  * @return None
  */
void BOD_LVR_Disable(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL &= (~SYS_BODCTL_LVR_EN_Msk);
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function will enable BOD Hysteresis
  * @return None
  */
void BOD_HYS_Enable(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL |= SYS_BODCTL_BOD_HYS_Msk;
	SYS_Lock(u8Lock);
}

/**
  * @brief  This function will disable BOD Hysteresis
  * @return None
  */
void BOD_HYS_Disable(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL &= (~SYS_BODCTL_BOD_HYS_Msk);
	SYS_Lock(u8Lock);
}

/**
  * @brief     This function clears the BOD interrupt flag.
  * @return    None
  */
void BOD_ClearIntFlag(void)
{
	uint8_t u8Lock = SYS_Unlock();

	SYS->BODCTL |= SYS_BODCTL_BOD_INT_Msk;
	SYS_Lock(u8Lock);
}


/*@}*/ /* end of group ISD9000_BOD_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_BOD_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/

