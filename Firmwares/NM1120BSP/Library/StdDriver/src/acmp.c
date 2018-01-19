/**************************************************************************//**
 * @file     acmp.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 16/03/22 9:38a $
 * @brief    NM1120 Analog Comparator(ACMP) driver source file
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "NM1120.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup NM1120_Device_Driver NM1120 Device Driver
  @{
*/

/** @addtogroup NM1120_ACMP_Driver ACMP Driver
  @{
*/


/** @addtogroup NM1120_ACMP_EXPORTED_FUNCTIONS ACMP Exported Functions
  @{
*/


/**
  * @brief  Configure the specified ACMP module
  *
  * @param[in]  Acmp The pointer of the specified ACMP module
  * @param[in]  u32ChNum Comparator number.
  * @param[in]  u32NegSrc Comparator negative input selection.  Including:
  *                  - \ref ACMP_CTL_NEGSEL_PIN
  *                  - \ref ACMP_CTL_NEGSEL_VBG
  *                  - \ref ACMP_CTL_NEGSEL_CRV
  * @param[in]  u32HysteresisEn The hysteresis function option. Including:
  *                  - \ref ACMP_CTL_HYSTERESIS_ENABLE
  *                  - \ref ACMP_CTL_HYSTERESIS_DISABLE
  *
  * @return     None
  *
  * @details    Configure hysteresis function, select the source of negative input and enable analog comparator.
  */
void ACMP_Open(ACMP_T *Acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn)
{
    Acmp->CTL[u32ChNum] = (Acmp->CTL[u32ChNum] & (~(ACMP_CTL_CPNSEL_Msk | ACMP_CTL_ACMPHYSEN_Msk))) | (u32NegSrc | u32HysteresisEn | ACMP_CTL_ACMPEN_Msk);
}

/**
  * @brief  Close analog comparator
  *
  * @param[in]  Acmp The pointer of the specified ACMP module
  * @param[in]  u32ChNum Comparator number.
  *
  * @return     None
  *
  * @details  This function will clear ACMPEN bit of ACMP_CTL register to disable analog comparator.
  */
void ACMP_Close(ACMP_T *Acmp, uint32_t u32ChNum)
{
    Acmp->CTL[u32ChNum] &= (~ACMP_CTL_ACMPEN_Msk);
}



/*@}*/ /* end of group NM1120_ACMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1120_ACMP_Driver */

/*@}*/ /* end of group NM1120_Device_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/

