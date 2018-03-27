/**************************************************************************//**
 * @file     RTC.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/06/25 5:06p $
 * @brief    ISD9000 RTC driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/


#include <stdio.h>
#include "ISD9000.h"


/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_RTC_Driver RTC Driver
  @{
*/

/** @addtogroup ISD9000_RTC_EXPORTED_FUNCTIONS RTC Exported Functions
  @{
*/

/**
 *  @brief    Enable RTC function and select RTC timeout interval
 *  @param    u32TimerInterval: RTC timeout interval
  *         - \ref RTC_TICK_4_SEC
  *         - \ref RTC_TICK_2_SEC
  *         - \ref RTC_TICK_1_SEC
  *         - \ref RTC_TICK_1_2_SEC
  *         - \ref RTC_TICK_1_4_SEC
  *         - \ref RTC_TICK_1_8_SEC
  *         - \ref RTC_TICK_1_16_SEC
  *         - \ref RTC_TICK_1_32_SEC
 *  @return   None
 *
 */
void RTC_Open(uint32_t u32TimerInterval)
{
	CLK_EnableModuleClock(RTC_MODULE);
	RTC_SetTimerInterval(u32TimerInterval);
	RTC_ENABLE();
	RTC_EnableInt();
}

/**
 *  @brief    Disable RTC function.
 *  @return   None
 */
void RTC_Close(void)
{
	RTC_DisableInt();
	RTC_DISABLE();
	CLK_DisableModuleClock(RTC_MODULE);
}

/**
 *  @brief    The function is used to enable RTC interrupt.
 *  @return   None
 */
void RTC_EnableInt(void)
{
	RTC->CTL |= RTC_CTL_RTIE_Msk;
	RTC_CLEAR_INT_FLAG();
	NVIC_EnableIRQ(RTC_IRQn);
}

/**
 *  @brief    The function is used to disable RTC interrupt.
 *  @return  	None
 */
void RTC_DisableInt(void)
{
	RTC->CTL = RTC->CTL & (~RTC_CTL_RTIE_Msk);
	NVIC_DisableIRQ(RTC_IRQn);
}

/**
 *  @brief    The function is used to select RTC timeout interval
 *  @param    u32TimerInterval: RTC timeout interval
  *         - \ref RTC_TICK_4_SEC
  *         - \ref RTC_TICK_2_SEC
  *         - \ref RTC_TICK_1_SEC
  *         - \ref RTC_TICK_1_2_SEC
  *         - \ref RTC_TICK_1_4_SEC
  *         - \ref RTC_TICK_1_8_SEC
  *         - \ref RTC_TICK_1_16_SEC
  *         - \ref RTC_TICK_1_32_SEC
 *  @return   None
 *
 */
void RTC_SetTimerInterval(uint32_t u32TimerInterval)
{
	RTC->CTL = ( RTC->CTL & (~RTC_CTL_RTIS_Msk)) | u32TimerInterval;
}


/*@}*/ /* end of group ISD9000_RTC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_RTC_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/






