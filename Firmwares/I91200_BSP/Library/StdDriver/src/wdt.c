/**************************************************************************//**
 * @file     wdt.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 16/08/01 1:00p $
 * @brief    I91200 WDT driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_WDT_Driver WDT Driver
  @{
*/


/** @addtogroup I91200_WDT_EXPORTED_FUNCTIONS WDT Exported Functions
  @{
*/

/**
 * @brief This function make WDT module start counting with different time-out interval
 * @param[in] u32TimeoutInterval  Time-out interval period of WDT module. Valid values are:
 *                - \ref WDT_TIMEOUT_2POW4
 *                - \ref WDT_TIMEOUT_2POW6
 *                - \ref WDT_TIMEOUT_2POW8
 *                - \ref WDT_TIMEOUT_2POW10
 *                - \ref WDT_TIMEOUT_2POW12
 *                - \ref WDT_TIMEOUT_2POW14
 *                - \ref WDT_TIMEOUT_2POW16
 *                - \ref WDT_TIMEOUT_2POW18
 * @param[in] u32EnableReset Enable WDT rest system function. Valid values are TRUE and FALSE
 * @return None
 */
void  WDT_Open( uint32_t u32TimeoutInterval, uint32_t u32EnableReset )
{
	uint8_t u8Lock = SYS_Unlock();
    WDT->CTL = u32TimeoutInterval | WDT_CTL_WDTEN_Msk |(u32EnableReset << WDT_CTL_RSTEN_Pos) ;
	SYS_Lock(u8Lock);
}

/**
 * @brief This function stops WDT counting and disable WDT module
 * @return None
 */
void WDT_Close(void)
{
	uint8_t u8Lock = SYS_Unlock();	
    WDT->CTL = 0;
	SYS_Lock(u8Lock);
}

/**
 * @brief This function enables the WDT time-out interrupt
 * @return None
 */
void WDT_EnableInt(void)
{
	uint8_t u8Lock = SYS_Unlock();	
    WDT->CTL |= WDT_CTL_INTEN_Msk;
    SYS_Lock(u8Lock);
}

/**
 * @brief This function disables the WDT time-out interrupt
 * @return None
 */
void WDT_DisableInt(void)
{
	uint8_t u8Lock = SYS_Unlock();	
    // Do not touch write 1 clear bits
    WDT->CTL &= ~(WDT_CTL_INTEN_Msk | WDT_CTL_RSTF_Msk | WDT_CTL_IF_Msk) ;
    SYS_Lock(u8Lock);
}

/**
  * @brief This macro clear WDT time-out reset system flag.
  * @return None
  */
void WDT_ClearResetFlag(void)
{
	uint8_t u8Lock = SYS_Unlock();	
	WDT->CTL = (WDT->CTL & ~WDT_CTL_IF_Msk) | WDT_CTL_RSTF_Msk;
    SYS_Lock(u8Lock);
}

/**
  * @brief This macro clear WDT time-out interrupt flag.
  * @return None
  */
void WDT_ClearTimeOutIntFlag(void)
{
	uint8_t u8Lock = SYS_Unlock();	
	WDT->CTL = (WDT->CTL & ~WDT_CTL_RSTF_Msk) | WDT_CTL_IF_Msk;	
	SYS_Lock(u8Lock);
}

/**
  * @brief This macro is used to reset 18-bit WDT counter.
  * @details If WDT is activated and enabled to reset system, software must reset WDT counter
  *  before WDT time-out plus reset delay reached. Or WDT generate a reset signal.
  */
void WDT_ResetCounter(void)
{
	uint8_t u8Lock = SYS_Unlock();
	WDT->CTL  = (WDT->CTL & ~(WDT_CTL_IF_Msk | WDT_CTL_RSTF_Msk)) | WDT_CTL_RSTCNT_Msk;
	SYS_Lock(u8Lock);
}

/*@}*/ /* end of group I91200_WDT_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_WDT_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
