/**************************************************************************//**
 * @file     wdt.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/07/12 2:08p $
 * @brief    NM1120 WDT driver source file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NM1120.h"

/** @addtogroup NM1120_Device_Driver NM1120 Device Driver
  @{
*/

/** @addtogroup NM1120_WDT_Driver WDT Driver
  @{
*/


/** @addtogroup NM1120_WDT_EXPORTED_FUNCTIONS WDT Exported Functions
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
 * @param[in] u32ResetDelay This parameter is not used.
 * @param[in] u32EnableReset Enable WDT rest system function. Valid values are \ref TRUE and \ref FALSE
 * @param[in] u32EnableWakeup Enable WDT wake-up system function. Valid values are \ref TRUE and \ref FALSE
 * @return None
 */
void  WDT_Open(uint32_t u32TimeoutInterval,
               uint32_t u32ResetDelay,
               uint32_t u32EnableReset,
               uint32_t u32EnableWakeup)
{

    WDT->CTL = u32TimeoutInterval | WDT_CTL_WDTEN_Msk |
               (u32EnableReset << WDT_CTL_RSTEN_Pos) |
               (u32EnableWakeup << WDT_CTL_WKEN_Pos);
    return;
}


/*@}*/ /* end of group NM1120_WDT_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1120_WDT_Driver */

/*@}*/ /* end of group NM1120_Device_Driver */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
