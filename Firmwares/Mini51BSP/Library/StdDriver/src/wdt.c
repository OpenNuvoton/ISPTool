/**************************************************************************//**
 * @file     wdt.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/03/18 5:39p $
 * @brief    MINI51 series WDT driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini51Series.h"

/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_WDT_Driver WDT Driver
  @{
*/


/** @addtogroup MINI51_WDT_EXPORTED_FUNCTIONS WDT Exported Functions
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
 * @param[in] u32ResetDelay This parameter is current not used
 * @param[in] u32EnableReset Enable WDT reset system function. Valid values are TRUE and FALSE
 * @param[in] u32EnableWakeup Enable WDT wake-up system function. Valid values are TRUE and FALSE
 * @return None
 */
void  WDT_Open(uint32_t u32TimeoutInterval,
               uint32_t u32ResetDelay,
               uint32_t u32EnableReset,
               uint32_t u32EnableWakeup)
{

    WDT->WTCR = u32TimeoutInterval | WDT_WTCR_WTE_Msk |
                (u32EnableReset << WDT_WTCR_WTRE_Pos) |
                (u32EnableWakeup << WDT_WTCR_WTWKE_Pos);
    return;
}

/**
 * @brief This function stops WDT counting and disable WDT module
 * @param None
 * @return None
 */
void WDT_Close(void)
{
    WDT->WTCR = 0;
    return;
}

/**
 * @brief This function enables the WDT time-out interrupt
 * @param None
 * @return None
 */
void WDT_EnableInt(void)
{
    WDT->WTCR = (WDT->WTCR & ~(WDT_WTCR_WTIF_Msk | WDT_WTCR_WTWKF_Msk | WDT_WTCR_WTRF_Msk)) | WDT_WTCR_WTIE_Msk;
    return;
}

/**
 * @brief This function disables the WDT time-out interrupt
 * @param None
 * @return None
 */
void WDT_DisableInt(void)
{
    WDT->WTCR &= ~(WDT_WTCR_WTIF_Msk | WDT_WTCR_WTWKF_Msk | WDT_WTCR_WTRF_Msk | WDT_WTCR_WTIE_Msk);
    return;
}



/*@}*/ /* end of group MINI51_WDT_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_WDT_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
