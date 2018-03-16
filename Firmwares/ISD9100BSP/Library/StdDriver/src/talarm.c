/*************************************************************************//**
 * @file     TALARM.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/07/03 15:06p $
 * @brief    ISD9100 TALARM driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include  "ISD9100.h"
/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_TALARM_Driver TALARM Driver
  @{
*/


/** @addtogroup ISD9100_TALARM_EXPORTED_FUNCTIONS TALARM Exported Functions
  @{
*/

/**
  * @brief      This function will enable temperature alarm
  * @param[in]  u8TALMVL is temperature alarm sense level.
  *             - \ref TALARM_TALMVL_105C 
  *             - \ref TALARM_TALMVL_115C 
  *             - \ref TALARM_TALMVL_125C 
  *             - \ref TALARM_TALMVL_135C 
  *             - \ref TALARM_TALMVL_145C 
  * @return None
  */
void TALARM_Open(uint8_t u8TALMVL)
{
	uint8_t u8Lock = SYS_Unlock();
	
    //TALARM and BOD operation require that the OSC16K low power oscillator is enabled
    CLK->PWRCTL &= (~CLK_PWRCTL_LIRCDPDEN_Msk);
    BODTALM->TALMSEL = u8TALMVL&BODTALM_TALMSEL_TALMVL_Msk;
    BODTALM->TALMCTL |= BODTALM_TALMCTL_TALMEN_Msk;
	
	SYS_Lock(u8Lock);
}

/**
  * @brief     This function will disable temperature alarm
  * @return    None
  */
void TALARM_Close(void)
{
    BODTALM->TALMCTL &= (~BODTALM_TALMCTL_TALMEN_Msk);
}

/*@}*/ /* end of group ISD9100_TALARM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_TALARM_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

