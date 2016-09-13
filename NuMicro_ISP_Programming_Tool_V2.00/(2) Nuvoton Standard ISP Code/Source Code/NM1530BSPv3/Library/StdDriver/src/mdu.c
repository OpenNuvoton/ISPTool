/**************************************************************************//**
 * @file     mdu.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 15/04/01 3:00p $
 * @brief    MDU driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NM1530.h"

/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup MDU_Driver MDU Driver
  @{
*/


/** @addtogroup MDU_EXPORTED_FUNCTIONS MDU Exported Functions
  @{
*/

/**
 * @brief Open specified MDU module.
 * @param[in] mdu The base address of the specified MDU module.
 * @return None
 * @details This function is used to reset and configure the specified MDU module.
 */
void MDU_Open(MDU_T *mdu)
{
    SYS->IPRSTC2 |=  SYS_IPRSTC2_MDU_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_MDU_RST_Msk;

    mdu->MDUCON = 0x0;      // Clear all MDU control registers.
    mdu->MDUSTS = 0xEF;     // Clear all flags.
}

/**
 * @brief Close specified MDU module.
 * @param[in] mdu The base address of the specified MDU module.
 * @return None.
 * @details This function is used to close the specified MDU module.
 */
void MDU_Close(MDU_T *mdu)
{
    mdu->MDUCON = 0x0;
    mdu->MDUSTS = 0xEF;
}


/*@}*/ /* end of group MDU_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MDU_Driver */

/*@}*/ /* end of group Standard_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
