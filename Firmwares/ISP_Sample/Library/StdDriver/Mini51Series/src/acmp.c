/**************************************************************************//**
 * @file     acmp.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/05/06 6:39p $
 * @brief    MINI51 series Analog Comparator(ACMP) driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "Mini51Series.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_ACMP_Driver ACMP Driver
  @{
*/


/** @addtogroup MINI51_ACMP_EXPORTED_FUNCTIONS ACMP Exported Functions
  @{
*/


/**
  * @brief  Configure the specified ACMP module
  *
  * @param[in]  acmp The base address of ACMP module
  * @param[in]  u32ChNum comparator number.
  * @param[in]  u32NegSrc is comparator negative input selection.  Including:
  *                  - \ref ACMP_VNEG_PIN
  *                  - \ref ACMP_VNEG_BANDGAP
  *                  - \ref ACMP_VNEG_4_OVER_24_VDD
  *                  - \ref ACMP_VNEG_5_OVER_24_VDD
  *                  - \ref ACMP_VNEG_6_OVER_24_VDD
  *                  - \ref ACMP_VNEG_7_OVER_24_VDD
  *                  - \ref ACMP_VNEG_8_OVER_24_VDD
  *                  - \ref ACMP_VNEG_9_OVER_24_VDD
  *                  - \ref ACMP_VNEG_10_OVER_24_VDD
  *                  - \ref ACMP_VNEG_11_OVER_24_VDD
  *                  - \ref ACMP_VNEG_12_OVER_24_VDD
  *                  - \ref ACMP_VNEG_13_OVER_24_VDD
  *                  - \ref ACMP_VNEG_14_OVER_24_VDD
  *                  - \ref ACMP_VNEG_15_OVER_24_VDD
  *                  - \ref ACMP_VNEG_16_OVER_24_VDD
  *                  - \ref ACMP_VNEG_17_OVER_24_VDD
  *                  - \ref ACMP_VNEG_18_OVER_24_VDD
  *                  - \ref ACMP_VNEG_19_OVER_24_VDD
  *
  * @param[in]  u32HysteresisEn is the hysteresis function option. Including:
  *                  - \ref ACMP_HYSTERESIS_ENABLE
  *                  - \ref ACMP_HYSTERESIS_DISABLE
  * @return None
  */
void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn)
{
    if(u32NegSrc != ACMP_VNEG_PIN)
        ACMP->CMPRVCR = u32NegSrc;
    ACMP->CMPCR[u32ChNum] = (ACMP->CMPCR[u32ChNum] & (~(ACMP_CMPCR_NEGSEL_Msk | ACMP_CMPCR_HYSEN_Msk))) |
                            ((u32NegSrc != ACMP_VNEG_PIN ? ACMP_CMPCR_NEGSEL_Msk : 0) | u32HysteresisEn | ACMP_CMPCR_ACMPEN_Msk);
}

/**
  * @brief  This function close comparator
  *
  * @param[in]  acmp The base address of ACMP module
  * @param[in]  u32ChNum comparator number.
  *
  * @return None
  */
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum)
{
    ACMP->CMPCR[u32ChNum] &= (~ACMP_CMPCR_ACMPEN_Msk);
}



/*@}*/ /* end of group MINI51_ACMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_ACMP_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

