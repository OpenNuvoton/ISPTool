/**************************************************************************//**
 * @file     wdt.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/15 1:00p $
 * @brief    ISD9000 WDT driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include  "ISD9000.h"

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_WDT_Driver WDT Driver
  @{
*/


/** @addtogroup ISD9000_WDT_EXPORTED_FUNCTIONS WDT Exported Functions
  @{
*/

/**
 * @brief This function is to make WDT module start counting with different time-out interval
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
	uint8_t u8Lock;
	
	u8Lock = SYS_Unlock();
	
    WDT->CTL = u32TimeoutInterval | WDT_CTL_WDTEN_Msk |(u32EnableReset << WDT_CTL_RSTEN_Pos) ;
	WDT_EnableInt();
	WDT_EnableWakeUp();
	NVIC_EnableIRQ(WDT_IRQn);
	
	SYS_Lock(u8Lock);
}

/**
 * @brief This function is to stop WDT counting and disable WDT module
 * @return None
 */
void WDT_Close(void)
{
	uint8_t u8Lock;
	
	u8Lock = SYS_Unlock();
    WDT->CTL = 0;
	SYS_Lock(u8Lock);
}

/**
 * @brief This function is to enable the WDT time-out interrupt
 * @return None
 */
void WDT_EnableInt(void)
{
	uint8_t u8Lock;
	
	u8Lock = SYS_Unlock();
    WDT->CTL |= WDT_CTL_INTEN_Msk;
	SYS_Lock(u8Lock);
}

/**
 * @brief This function is to disable the WDT time-out interrupt
 * @return None
 */
void WDT_DisableInt(void)
{
    // Do not touch write 1 clear bits
	uint8_t u8Lock;
	
	u8Lock = SYS_Unlock();
    WDT->CTL &= ~(WDT_CTL_INTEN_Msk | WDT_CTL_RSTF_Msk | WDT_CTL_IF_Msk | WDT_CTL_WKF_Msk);
	SYS_Lock(u8Lock);
}

/**
 * @brief This function is to enable the WDT time-out Wake-Up
 * @return None
 */
void WDT_EnableWakeUp(void)
{
	uint8_t u8Lock;
	
	u8Lock = SYS_Unlock();
    WDT->CTL |= WDT_CTL_WKEN_Msk;
	SYS_Lock(u8Lock);
}

/**
 * @brief This function is to disable the WDT time-out Wake-Up
 * @return None
 */
void WDT_DisableWakeUp(void)
{
    // Do not touch write 1 clear bits
	uint8_t u8Lock;
	
	u8Lock = SYS_Unlock();
    WDT->CTL &= ~(WDT_CTL_WKEN_Msk | WDT_CTL_RSTF_Msk | WDT_CTL_IF_Msk | WDT_CTL_WKF_Msk);
	SYS_Lock(u8Lock);
}


/**
  * @brief This function is to reset 19-bit WDT counter.
  * @details If WDT is activated and enabled to reset system, software must reset WDT counter
  *  before WDT time-out plus reset delay reached. Or WDT generate a reset signal.
  */
void WDT_ResetCounter(void)
{
	uint8_t u8Lock;
	
	u8Lock = SYS_Unlock();
	(WDT->CTL  = (WDT->CTL & ~(WDT_CTL_WKF_Msk | WDT_CTL_IF_Msk | WDT_CTL_RSTF_Msk)) | WDT_CTL_RSTCNT_Msk);
	SYS_Lock(u8Lock);
}

/**
  * @brief This function is to clear WDT time-out interrupt flag.
  * @return None
  * \hideinitializer
  */
void WDT_ClearTimeOutIntFlag(void)
{
	uint8_t u8Lock;
	
	u8Lock = SYS_Unlock();
	(WDT->CTL = (WDT->CTL & ~(WDT_CTL_WKF_Msk | WDT_CTL_RSTF_Msk)) | WDT_CTL_IF_Msk);
	SYS_Lock(u8Lock);
}
/**
  * @brief This functon is to clear WDT time-out reset system flag.
  * @return None
  */
void WDT_ClearResetFlag(void)
{
	uint8_t u8Lock;
	
	u8Lock = SYS_Unlock();
	(WDT->CTL = (WDT->CTL & ~(WDT_CTL_WKF_Msk | WDT_CTL_IF_Msk)) | WDT_CTL_RSTF_Msk);
	SYS_Lock(u8Lock);
}

/**
  * @brief This function is to clear WDT time-out Wake-Up flag.
  * @return None
  */
void WDT_ClearTimeOutWakeupFlag(void)
{
	uint8_t u8Lock;
	
	u8Lock = SYS_Unlock();
	(WDT->CTL = (WDT->CTL & ~(WDT_CTL_RSTF_Msk | WDT_CTL_IF_Msk)) | WDT_CTL_WKF_Msk);
	SYS_Lock(u8Lock);
}

/*@}*/ /* end of group ISD9000_WDT_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_WDT_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
