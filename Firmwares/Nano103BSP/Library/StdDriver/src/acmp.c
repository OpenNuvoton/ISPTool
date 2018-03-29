/**************************************************************************//**
 * @file     acmp.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 15/12/10 5:28p $
 * @brief    Nano 103 Analog Comparator(ACMP) driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "Nano103.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_ACMP_Driver ACMP Driver
  @{
*/


/** @addtogroup NANO103_ACMP_EXPORTED_FUNCTIONS ACMP Exported Functions
  @{
*/


/**
  * @brief  This function open and configure comparator parameters
  *
  * @param[in]  acmp The base address of ACMP module
  * @param[in]  u32ChNum This parameter is not used in Nano103
  * @param[in]  u32NegSrc is comparator0 negative input selection.  Including :
  *                  - \ref ACMP_VNEG_PIN
  *                  - \ref ACMP_VNEG_IREF
  *                  - \ref ACMP_VNEG_AVSS
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
  *                  - \ref ACMP_VNEG_4_OVER_24_IREF
  *                  - \ref ACMP_VNEG_5_OVER_24_IREF
  *                  - \ref ACMP_VNEG_6_OVER_24_IREF
  *                  - \ref ACMP_VNEG_7_OVER_24_IREF
  *                  - \ref ACMP_VNEG_8_OVER_24_IREF
  *                  - \ref ACMP_VNEG_9_OVER_24_IREF
  *                  - \ref ACMP_VNEG_10_OVER_24_IREF
  *                  - \ref ACMP_VNEG_11_OVER_24_IREF
  *                  - \ref ACMP_VNEG_12_OVER_24_IREF
  *                  - \ref ACMP_VNEG_13_OVER_24_IREF
  *                  - \ref ACMP_VNEG_14_OVER_24_IREF
  *                  - \ref ACMP_VNEG_15_OVER_24_IREF
  *                  - \ref ACMP_VNEG_16_OVER_24_IREF
  *                  - \ref ACMP_VNEG_17_OVER_24_IREF
  *                  - \ref ACMP_VNEG_18_OVER_24_IREF
  *                  - \ref ACMP_VNEG_19_OVER_24_IREF
  * @param[in]  u32HysteresisEn is charge or discharge pin selection. Including :
  *                  - \ref ACMP_HYSTERESIS_ENABLE
  *                  - \ref ACMP_HYSTERESIS_DISABLE
  * @return None
  */
void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn)
{
    if(u32NegSrc & 0x80000000)   // one of ACMP_VNEG_PIN, ACMP_VNEG_IREF, ACMP_VNEG_AVSS
    {
        acmp->CTL0 = ~(u32NegSrc) | u32HysteresisEn | ACMP_CTL0_ACMPEN_Msk;
    }
    else
    {
        acmp->CTL0 = (1 << ACMP_CTL0_NEGSEL_Pos) | u32HysteresisEn | ACMP_CTL0_ACMPEN_Msk;
        acmp->VREF = u32NegSrc | ACMP_VREF_CRVEN_Msk;
    }
}

/**
  * @brief  This function close comparator
  * @param[in]  acmp The base address of ACMP module
  * @param[in]  u32ChNum This parameter is not used in Nano103
  */
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum)
{
    acmp->CTL0 &= ~ACMP_CTL0_ACMPEN_Msk;
}



/*@}*/ /* end of group NANO103_ACMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_ACMP_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/

