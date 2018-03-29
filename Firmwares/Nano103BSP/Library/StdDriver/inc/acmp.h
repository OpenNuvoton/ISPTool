/**************************************************************************//**
 * @file     acmp.h
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 16/02/16 11:23a $
 * @brief    NANO103 series Analog Comparator(ACMP) driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __ACMP_H__
#define __ACMP_H__

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

/** @addtogroup NANO103_ACMP_EXPORTED_CONSTANTS ACMP Exported Constants
  @{
*/
#define ACMP_HYSTERESIS_ENABLE    ACMP_CTL0_HYSEN_Msk         ///< ACMP hysteresis enable \hideinitializer
#define ACMP_HYSTERESIS_DISABLE   0                           ///< ACMP hysteresis disable \hideinitializer

#define ACMP_VNEG_PIN             ~(0 << ACMP_CTL0_NEGSEL_Pos)    ///< Selecting the voltage of ACMP0_N pin as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_IREF            ~(2 << ACMP_CTL0_NEGSEL_Pos)    ///< Selecting Internal reference voltage as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_AVSS            ~(3 << ACMP_CTL0_NEGSEL_Pos)    ///< Selecting AVSS pin as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_4_OVER_24_VDD   (0x0UL)                         ///< Selecting 4/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_5_OVER_24_VDD   (0x1UL)                         ///< Selecting 5/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_6_OVER_24_VDD   (0x2UL)                         ///< Selecting 6/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_7_OVER_24_VDD   (0x3UL)                         ///< Selecting 7/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_8_OVER_24_VDD   (0x4UL)                         ///< Selecting 8/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_9_OVER_24_VDD   (0x5UL)                         ///< Selecting 9/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_10_OVER_24_VDD  (0x6UL)                         ///< Selecting 10/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_11_OVER_24_VDD  (0x7UL)                         ///< Selecting 11/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_12_OVER_24_VDD  (0x8UL)                         ///< Selecting 12/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_13_OVER_24_VDD  (0x9UL)                         ///< Selecting 13/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_14_OVER_24_VDD  (0xAUL)                         ///< Selecting 14/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_15_OVER_24_VDD  (0xBUL)                         ///< Selecting 15/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_16_OVER_24_VDD  (0xCUL)                         ///< Selecting 16/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_17_OVER_24_VDD  (0xDUL)                         ///< Selecting 17/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_18_OVER_24_VDD  (0xEUL)                         ///< Selecting 18/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_19_OVER_24_VDD  (0xFUL)                         ///< Selecting 19/24 AVDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_4_OVER_24_IREF  (ACMP_VREF_CRVSSEL_Msk | 0x0UL) ///< Selecting 4/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_5_OVER_24_IREF  (ACMP_VREF_CRVSSEL_Msk | 0x1UL) ///< Selecting 5/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_6_OVER_24_IREF  (ACMP_VREF_CRVSSEL_Msk | 0x2UL) ///< Selecting 6/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_7_OVER_24_IREF  (ACMP_VREF_CRVSSEL_Msk | 0x3UL) ///< Selecting 7/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_8_OVER_24_IREF  (ACMP_VREF_CRVSSEL_Msk | 0x4UL) ///< Selecting 8/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_9_OVER_24_IREF  (ACMP_VREF_CRVSSEL_Msk | 0x5UL) ///< Selecting 9/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_10_OVER_24_IREF (ACMP_VREF_CRVSSEL_Msk | 0x6UL) ///< Selecting 10/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_11_OVER_24_IREF (ACMP_VREF_CRVSSEL_Msk | 0x7UL) ///< Selecting 11/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_12_OVER_24_IREF (ACMP_VREF_CRVSSEL_Msk | 0x8UL) ///< Selecting 12/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_13_OVER_24_IREF (ACMP_VREF_CRVSSEL_Msk | 0x9UL) ///< Selecting 13/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_14_OVER_24_IREF (ACMP_VREF_CRVSSEL_Msk | 0xAUL) ///< Selecting 14/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_15_OVER_24_IREF (ACMP_VREF_CRVSSEL_Msk | 0xBUL) ///< Selecting 15/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_16_OVER_24_IREF (ACMP_VREF_CRVSSEL_Msk | 0xCUL) ///< Selecting 16/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_17_OVER_24_IREF (ACMP_VREF_CRVSSEL_Msk | 0xDUL) ///< Selecting 17/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_18_OVER_24_IREF (ACMP_VREF_CRVSSEL_Msk | 0xEUL) ///< Selecting 18/24 IREF as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_19_OVER_24_IREF (ACMP_VREF_CRVSSEL_Msk | 0xFUL) ///< Selecting 19/24 IREF as the source of ACMP V- \hideinitializer

/*@}*/ /* end of group NANO103_ACMP_EXPORTED_CONSTANTS */


/** @addtogroup NANO103_ACMP_EXPORTED_FUNCTIONS ACMP Exported Functions
  @{
*/

/**
  * @brief This macro is used to enable output inverse
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @param[in] u32Src is comparator0 negative input selection.  Including :
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
  * @return None
  * \hideinitializer
  */
#define ACMP_SET_NEG_SRC(acmp, u32ChNum, u32Src) do{\
                                                     if(u32Src & 0x80000000) {\
                                                         ACMP->CTL0 = (~u32Src);\
                                                         ACMP->CTL0 |= ACMP_CTL0_ACMPEN_Msk;\
                                                     }\
                                                     else {\
                                                         ACMP->CTL0 = ((1 << ACMP_CTL0_NEGSEL_Pos));\
                                                         ACMP->CTL0 |= ACMP_CTL0_ACMPEN_Msk;\
                                                         ACMP->VREF = (u32Src | ACMP_VREF_CRVEN_Msk);\
                                                     }\
                                                 }while(0)


/**
  * @brief This macro is used to enable hysteresis
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @return None
  * \hideinitializer
  */
#define ACMP_ENABLE_HYSTERESIS(acmp,u32ChNum) (ACMP->CTL0 |= ACMP_CTL0_HYSEN_Msk)

/**
  * @brief This macro is used to disable hysteresis
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @return None
  * \hideinitializer
  */
#define ACMP_DISABLE_HYSTERESIS(acmp,u32ChNum) (ACMP->CTL0 &= ~ACMP_CTL0_HYSEN_Msk)

/**
  * @brief This macro is used to enable interrupt
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @return None
  * \hideinitializer
  */
#define ACMP_ENABLE_INT(acmp,u32ChNum) (ACMP->CTL0 |= ACMP_CTL0_ACMPIE_Msk)

/**
  * @brief This macro is used to disable interrupt
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @return None
  * \hideinitializer
  */
#define ACMP_DISABLE_INT(acmp,u32ChNum) (ACMP->CTL0 &= ~ACMP_CTL0_ACMPIE_Msk)


/**
  * @brief This macro is used to enable ACMP
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @return None
  * \hideinitializer
  */
#define ACMP_ENABLE(acmp,u32ChNum) (ACMP->CTL0 |= ACMP_CTL0_ACMPEN_Msk)

/**
  * @brief This macro is used to disable ACMP
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @return None
  * \hideinitializer
  */
#define ACMP_DISABLE(acmp,u32ChNum) (ACMP->CTL0 &= ~ACMP_CTL0_ACMPEN_Msk)

/**
  * @brief This macro is used to get ACMP output value
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @return  1 or 0
  * \hideinitializer
  */
#define ACMP_GET_OUTPUT(acmp,u32ChNum) ((ACMP->STATUS & ACMP_STATUS_ACMPO_Msk) ? 1 : 0)

/**
  * @brief This macro is used to get ACMP interrupt flag
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @return   ACMP interrupt occurred or not
  * \hideinitializer
  */
#define ACMP_GET_INT_FLAG(acmp,u32ChNum) (ACMP->STATUS & ACMP_STATUS_ACMPIF_Msk)

/**
  * @brief This macro is used to clear ACMP interrupt flag
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @return   None
  * \hideinitializer
  */
#define ACMP_CLR_INT_FLAG(acmp,u32ChNum) (ACMP->STATUS = ACMP_STATUS_ACMPIF_Msk)

/**
  * @brief This macro is used to enable ACMP wake-up
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @return   None
  * \hideinitializer
  */
#define ACMP_ENABLE_WAKEUP(acmp,u32ChNum) (ACMP->CTL0 |= ACMP_CTL0_WKEN_Msk)

/**
  * @brief This macro is used to disable ACMP wake-up
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum This parameter is not used in Nano103
  * @return   None
  * \hideinitializer
  */
#define ACMP_DISABLE_WAKEUP(acmp,u32ChNum) (ACMP->CTL0 &= ~ACMP_CTL0_WKEN_Msk)

/**
  * @brief This macro is used to enable ACMP wake-up
  * @param[in] u32Level  CRV level, possible values are
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
  * @return   None
  * @note This macro only set CVR level, and does not enable/disable CRV
  * \hideinitializer
  */
#define ACMP_CRV_SEL(u32Level) (ACMP->VREF = (ACMP->VREF & ~(ACMP_VREF_CRVSSEL_Msk | ACMP_VREF_CRVCTL_Msk))| u32Level)

/**
  * @brief This macro is used to enable CRV(comparator reference voltage)
  * @param[in] acmp The base address of ACMP module
  * @return   None
  * \hideinitializer
  */
#define ACMP_ENABLE_CRV(acmp) (ACMP->VREF |= ACMP_VREF_CRVEN_Msk)

/**
  * @brief This macro is used to disable CRV(comparator reference voltage)
  * @param[in] acmp The base address of ACMP module
  * @return   None
  * \hideinitializer
  */
#define ACMP_DISABLE_CRV(acmp) (ACMP->VREF &= ~ACMP_VREF_CRVEN_Msk)



void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn);
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum);


/*@}*/ /* end of group NANO103_ACMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_ACMP_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__ACMP_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
