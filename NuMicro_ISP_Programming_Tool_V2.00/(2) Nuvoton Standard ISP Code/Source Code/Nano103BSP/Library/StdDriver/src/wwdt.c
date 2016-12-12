/**************************************************************************//**
 * @file     wwdt.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 16/01/08 5:29p $
 * @brief    Nano 103 WWDT driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Nano103.h"

/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_WWDT_Driver WWDT Driver
  @{
*/


/** @addtogroup NANO103_WWDT_EXPORTED_FUNCTIONS WWDT Exported Functions
  @{
*/

/**
 * @brief This function make WWDT module start counting with different counter period and compared window value
 * @param[in] u32PreScale  Prescale period for the WWDT counter period. Valid values are:
 *              - \ref WWDT_PRESCALER_1
 *              - \ref WWDT_PRESCALER_2
 *              - \ref WWDT_PRESCALER_4
 *              - \ref WWDT_PRESCALER_8
 *              - \ref WWDT_PRESCALER_16
 *              - \ref WWDT_PRESCALER_32
 *              - \ref WWDT_PRESCALER_64
 *              - \ref WWDT_PRESCALER_128
 *              - \ref WWDT_PRESCALER_192
 *              - \ref WWDT_PRESCALER_256
 *              - \ref WWDT_PRESCALER_384
 *              - \ref WWDT_PRESCALER_512
 *              - \ref WWDT_PRESCALER_768
 *              - \ref WWDT_PRESCALER_1024
 *              - \ref WWDT_PRESCALER_1536
 *              - \ref WWDT_PRESCALER_2048
 * @param[in] u32CmpValue Window compared value. Valid values are between 0x2 to 0x3E
 * @param[in] u32EnableInt Enable WWDT interrupt or not. Valid values are TRUE and FALSE
 * @return None
 * @note Application can call this function can only once after boot up
 *       The parameter u32CmpValue Window compared value has limitation, valid values are between 2 to 0x3E.
 *       If the input value is more than 0x3E, the value will be changed to 0x3E.
 *       If the input value is less than 2, the value will be changed to 2.
 */
void WWDT_Open(uint32_t u32PreScale, uint32_t u32CmpValue, uint32_t u32EnableInt)
{
    /* Window compared value should be between 2 to 0x3E */
    if (u32CmpValue > 0x3E)
        u32CmpValue = 0x3E;
    if (u32CmpValue < 2)
        u32CmpValue = 2;

    WWDT->INTEN = u32EnableInt;
    WWDT->CTL = u32PreScale | (u32CmpValue << WWDT_CTL_WINCMP_Pos) | WWDT_CTL_WWDTEN_Msk;
    return;
}




/*@}*/ /* end of group NANO103_WDT_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_WDT_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
