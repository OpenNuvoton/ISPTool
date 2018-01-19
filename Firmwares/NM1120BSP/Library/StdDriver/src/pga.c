/**************************************************************************//**
 * @file     pga.c
 * @version  V1.00
 * $Revision: 9 $
 * $Date: 17/05/04 5:25p $
 * @brief    NM1120 PGA driver source file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NM1120.h"

/** @addtogroup NM1120_Device_Driver NM1120 Device Driver
  @{
*/

/** @addtogroup NM1120_PGA_Driver PGA Driver
  @{
*/


/** @addtogroup NM1120_PGA_EXPORTED_FUNCTIONS PGA Exported Functions
  @{
*/

/**
  * @brief      Enable PGA module with gain value and output channel
  * @param[in]  pga     The pointer of the specified PGA module
  * @param[in]  u32Gain is gain index. Including :
  *             - \ref PGA_GAIN_1
  *             - \ref PGA_GAIN_2
  *             - \ref PGA_GAIN_3
  *             - \ref PGA_GAIN_5
  *             - \ref PGA_GAIN_7
  *             - \ref PGA_GAIN_9
  *             - \ref PGA_GAIN_11
  *             - \ref PGA_GAIN_13
  * @param[in]  u32OutputMask is PGA output channels mask. 0 to disable output channel; 1 to enable output channel.
  * @return  None
  */
void PGA_Open(PGA_T *pga, uint32_t u32Gain, uint32_t u32OutputMask)
{
    if (u32OutputMask == 0)
    {
        if ((SYS->GPC_MFP & SYS_GPC_MFP_PC3MFP_Msk) == SYS_GPC_MFP_PC3_PGA_O)
        {
            /* Chip version B: disable PGA output pin by set PC3 MFP to non-PGA mode */
            SYS->GPC_MFP = (SYS->GPC_MFP & ~(SYS_GPC_MFP_PC3MFP_Msk))
                            | (SYS_GPC_MFP_PC3_GPIO);
        }
    }
    else
    {
        /* Chip version B: enable PGA output pin by set PC3 MFP to PGA mode */
        SYS->GPC_MFP = (SYS->GPC_MFP & ~(SYS_GPC_MFP_PC3MFP_Msk))
                        | (SYS_GPC_MFP_PC3_PGA_O);
    }

    /* Chip version B: support gain level x1 by disable PGAEN bit */
    if (u32Gain == PGA_GAIN_1)
        pga->CTL = (u32Gain << PGA_CTL_GAIN_Pos);
    else
        pga->CTL = PGA_CTL_PGAEN_Msk | (u32Gain << PGA_CTL_GAIN_Pos);
}

/**
  * @brief      Disable PGA module
  * @param[in]  pga     The pointer of the specified PGA module
  * @return     None
  */
void PGA_Close(PGA_T *pga)
{
    pga->CTL = pga->CTL & (~PGA_CTL_PGAEN_Msk);
}

/*@}*/ /* end of group NM1120_PGA_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1120_PGA_Driver */

/*@}*/ /* end of group NM1120_Device_Driver */

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
