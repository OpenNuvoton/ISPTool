/**************************************************************************//**
 * @file     acmp.h
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/05/26 5:56p $
 * @brief    Mini58 series Analog Comparator(ACMP) driver header file
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


/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_ACMP_Driver ACMP Driver
  @{
*/

/** @addtogroup Mini58_ACMP_EXPORTED_CONSTANTS ACMP Exported Constants
  @{
*/


#define ACMP_VNEG_PIN                  (0xFFUL)                         ///< Selecting the voltage of ACMP negative input pin as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_BANDGAP              (0x00UL)                         ///< Selecting band-gap voltage as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_4_OVER_24_VDD        (0x80UL)                         ///< Selecting 4/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_5_OVER_24_VDD        (0x81UL)                         ///< Selecting 5/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_6_OVER_24_VDD        (0x82UL)                         ///< Selecting 6/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_7_OVER_24_VDD        (0x83UL)                         ///< Selecting 7/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_8_OVER_24_VDD        (0x84UL)                         ///< Selecting 8/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_9_OVER_24_VDD        (0x85UL)                         ///< Selecting 9/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_10_OVER_24_VDD       (0x86UL)                         ///< Selecting 10/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_11_OVER_24_VDD       (0x87UL)                         ///< Selecting 11/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_12_OVER_24_VDD       (0x88UL)                         ///< Selecting 12/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_13_OVER_24_VDD       (0x89UL)                         ///< Selecting 13/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_14_OVER_24_VDD       (0x8AUL)                         ///< Selecting 14/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_15_OVER_24_VDD       (0x8BUL)                         ///< Selecting 15/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_16_OVER_24_VDD       (0x8CUL)                         ///< Selecting 16/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_17_OVER_24_VDD       (0x8DUL)                         ///< Selecting 17/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_18_OVER_24_VDD       (0x8EUL)                         ///< Selecting 18/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_VNEG_19_OVER_24_VDD       (0x8FUL)                         ///< Selecting 19/24 VDD as the source of ACMP V- \hideinitializer
#define ACMP_HYSTERESIS_ENABLE         (1UL << ACMP_CTL_HYSSEL_Pos)     ///< Enable hysteresis function \hideinitializer
#define ACMP_HYSTERESIS_DISABLE        (0UL)                            ///< Disable hysteresis function \hideinitializer
#define ACMP_CH0_POSPIN_P15            (0UL)                            ///< Selecting P1.5 as ACMP Channel 0 positive input pin \hideinitializer
#define ACMP_CH0_POSPIN_P10            (1UL << ACMP_CTL_POSSEL_Pos)     ///< Selecting P1.0 as ACMP Channel 0 positive input pin \hideinitializer
#define ACMP_CH0_POSPIN_P12            (2UL << ACMP_CTL_POSSEL_Pos)     ///< Selecting P1.2 as ACMP Channel 0 positive input pin \hideinitializer
#define ACMP_CH0_POSPIN_P13            (3UL << ACMP_CTL_POSSEL_Pos)     ///< Selecting P1.3 as ACMP Channel 0 positive input pin \hideinitializer
#define ACMP_CH1_POSPIN_P31            (0UL)                            ///< Selecting P3.1 as ACMP Channel 1 positive input pin \hideinitializer
#define ACMP_CH1_POSPIN_P32            (1UL << ACMP_CTL_POSSEL_Pos)     ///< Selecting P3.2 as ACMP Channel 1 positive input pin \hideinitializer
#define ACMP_CH1_POSPIN_P34            (2UL << ACMP_CTL_POSSEL_Pos)     ///< Selecting P3.4 as ACMP Channel 1 positive input pin \hideinitializer
#define ACMP_CH1_POSPIN_P35            (3UL << ACMP_CTL_POSSEL_Pos)     ///< Selecting P3.5 as ACMP Channel 1 positive input pin \hideinitializer
#define ACMP_FILTSEL_OFF               (0UL)                            ///<  Comparator output filter function is disabled \hideinitializer
#define ACMP_FILTSEL_1PCLK             (1UL << ACMP_CTL_FILTSEL_Pos)    ///<  Comparator output filter is sampled 1 consecutive PCLK \hideinitializer
#define ACMP_FILTSEL_2PCLK             (2UL << ACMP_CTL_FILTSEL_Pos)    ///<  Comparator output filter is sampled 2 consecutive PCLKs \hideinitializer
#define ACMP_FILTSEL_4PCLK             (3UL << ACMP_CTL_FILTSEL_Pos)    ///<  Comparator output filter is sampled 4 consecutive PCLKs \hideinitializer
#define ACMP_FILTSEL_8PCLK             (4UL << ACMP_CTL_FILTSEL_Pos)    ///<  Comparator output filter is sampled 8 consecutive PCLKs \hideinitializer
#define ACMP_FILTSEL_16PCLK            (5UL << ACMP_CTL_FILTSEL_Pos)    ///<  Comparator output filter is sampled 16 consecutive PCLKs \hideinitializer
#define ACMP_FILTSEL_32PCLK            (6UL << ACMP_CTL_FILTSEL_Pos)    ///<  Comparator output filter is sampled 32 consecutive PCLKs \hideinitializer
#define ACMP_FILTSEL_64PCLK            (7UL << ACMP_CTL_FILTSEL_Pos)    ///<  Comparator output filter is sampled 64 consecutive PCLKs \hideinitializer
#define ACMP_FILTSEL_128PCLK           (8UL << ACMP_CTL_FILTSEL_Pos)    ///<  Comparator output filter is sampled 128 consecutive PCLKs \hideinitializer
#define ACMP_FILTSEL_256PCLK           (9UL << ACMP_CTL_FILTSEL_Pos)    ///<  Comparator output filter is sampled 256 consecutive PCLKs \hideinitializer
#define ACMP_FILTSEL_512PCLK           (10UL << ACMP_CTL_FILTSEL_Pos)   ///<  Comparator output filter is sampled 512 consecutive PCLKs \hideinitializer

/*@}*/ /* end of group Mini58_ACMP_EXPORTED_CONSTANTS */


/** @addtogroup Mini58_ACMP_EXPORTED_FUNCTIONS ACMP Exported Functions
  @{
*/

/**
  * @brief This macro is used to select ACMP negative input source
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @param[in] u32Src is comparator negative input selection.  Including :
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
  * @return None
  * @note The V- setting is shared by both comparators if input source is not coming from PIN
  * \hideinitializer
  */
#define ACMP_SET_NEG_SRC(acmp, u32ChNum, u32Src) do{\
                                                     if(u32Src == ACMP_VNEG_PIN)\
                                                         ACMP->CTL[u32ChNum] &= ~ACMP_CTL_NEGSEL_Msk;\
                                                     else {\
                                                         ACMP->CTL[u32ChNum] |= ACMP_CTL_NEGSEL_Msk;\
                                                         ACMP->VREF = u32Src;\
                                                     }\
                                                 }while(0)

/**
  * @brief This macro is used to enable hysteresis function
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return None
  * \hideinitializer
  */
#define ACMP_ENABLE_HYSTERESIS(acmp, u32ChNum) (ACMP->CTL[u32ChNum] |= ACMP_CTL_HYSSEL_Msk)

/**
  * @brief This macro is used to disable hysteresis function
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return None
  * \hideinitializer
  */
#define ACMP_DISABLE_HYSTERESIS(acmp, u32ChNum) (ACMP->CTL[u32ChNum] &= ~ACMP_CTL_HYSSEL_Msk)

/**
  * @brief This macro is used to enable interrupt
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return None
  * \hideinitializer
  */
#define ACMP_ENABLE_INT(acmp, u32ChNum) (ACMP->CTL[u32ChNum] |= ACMP_CTL_ACMPIE_Msk)

/**
  * @brief This macro is used to disable interrupt
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return None
  * \hideinitializer
  */
#define ACMP_DISABLE_INT(acmp, u32ChNum) (ACMP->CTL[u32ChNum] &= ~ACMP_CTL_ACMPIE_Msk)


/**
  * @brief This macro is used to enable ACMP
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number
  * @return None
  * \hideinitializer
  */
#define ACMP_ENABLE(acmp, u32ChNum) (ACMP->CTL[u32ChNum] |= ACMP_CTL_ACMPEN_Msk)

/**
  * @brief This macro is used to disable ACMP
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return None
  * \hideinitializer
  */
#define ACMP_DISABLE(acmp, u32ChNum) (ACMP->CTL[u32ChNum] &= ~ACMP_CTL_ACMPEN_Msk)

/**
  * @brief This macro is used to get ACMP output value
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return  1 or 0
  * \hideinitializer
  */
#define ACMP_GET_OUTPUT(acmp, u32ChNum) (ACMP->STATUS & (ACMP_STATUS_ACMPO0_Msk<<(u32ChNum))?1:0)

/**
  * @brief This macro is used to get ACMP interrupt flag
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return   ACMP interrupt occurred or not
  * \hideinitializer
  */
#define ACMP_GET_INT_FLAG(acmp, u32ChNum) (ACMP->STATUS & (ACMP_STATUS_ACMPIF0_Msk<<(u32ChNum))?1:0)

/**
  * @brief This macro is used to clear ACMP interrupt flag
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return   None
  * \hideinitializer
  */
#define ACMP_CLR_INT_FLAG(acmp, u32ChNum) (ACMP->STATUS = (ACMP_STATUS_ACMPIF0_Msk<<(u32ChNum)))

/**
  * @brief This macro is used to select the V+ pin of ACMP
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @param[in] u32Pin The input pin. For channel 0, valid values are \ref ACMP_CH0_POSPIN_P15,
  *            \ref ACMP_CH0_POSPIN_P10, \ref ACMP_CH0_POSPIN_P12, and \ref ACMP_CH0_POSPIN_P13. For
  *            channel 1, valid values are , \ref ACMP_CH1_POSPIN_P31, \ref ACMP_CH1_POSPIN_P32,
  *            \ref ACMP_CH1_POSPIN_P34, and \ref ACMP_CH1_POSPIN_P35.
  * @return   None
  * @note   Except this setting, multi-function pin also needs to be configured
  * \hideinitializer
  */
#define ACMP_SELECT_P(acmp, u32ChNum, u32Pin)  (ACMP->CTL[u32ChNum] = (ACMP->CTL[u32ChNum] & ~ACMP_CTL_POSSEL_Msk) | (u32Pin))
/**
  * @brief This macro is used to set the level of CRV(Comparator Reference Voltage)
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32Level CRV level, possible values are
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
  * @return   None
  * \hideinitializer
  */
#define ACMP_CRV_SEL(acmp, u32Level) (ACMP->VREF = (ACMP->VREF & ~ACMP_VREF_CRVCTL_Msk) | ((u32Level) & ~ACMP_VREF_IREFSEL_Msk))
/**
  * @brief This macro is used to enable CRV(Comparator Reference Voltage)
  * @param[in] acmp The base address of ACMP module
  * @return   None
  * \hideinitializer
  */
#define ACMP_ENABLE_CRV(acmp) (ACMP->VREF |= ACMP_VREF_IREFSEL_Msk)
/**
  * @brief This macro is used to disable CRV(Comparator Reference Voltage)
  * @param[in] acmp The base address of ACMP module
  * @return   None
  * \hideinitializer
  */
#define ACMP_DISABLE_CRV(acmp) (ACMP->VREF &= ~ACMP_VREF_IREFSEL_Msk)

/**
  * @brief This macro is used to enable ACMP falling edge trigger Timer/PWM
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return   None
  * \hideinitializer
  */
#define ACMP_ENABLE_FALLING_EDGE_TRIGGER(acmp, u32ChNum) (ACMP->CTL[u32ChNum] |= ACMP_CTL_FTRGEN_Msk)

/**
  * @brief This macro is used to disable ACMP falling edge trigger Timer/PWM
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return   None
  * \hideinitializer
  */
#define ACMP_DISABLE_FALLING_EDGE_TRIGGER(acmp, u32ChNum) (ACMP->CTL[u32ChNum] &= ~ACMP_CTL_FTRGEN_Msk)

/**
  * @brief This macro is used to enable ACMP rising edge trigger Timer/PWM
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return   None
  * \hideinitializer
  */
#define ACMP_ENABLE_RISING_EDGE_TRIGGER(acmp, u32ChNum) (ACMP->CTL[u32ChNum] |= ACMP_CTL_RTRGEN_Msk)

/**
  * @brief This macro is used to disable ACMP rising edge trigger Timer/PWM
  * @param[in] acmp The base address of ACMP module
  * @param[in] u32ChNum The ACMP number, ether 0 or 1
  * @return   None
  * \hideinitializer
  */
#define ACMP_DISABLE_RISING_EDGE_TRIGGER(acmp, u32ChNum) (ACMP->CTL[u32ChNum] &= ~ACMP_CTL_RTRGEN_Msk)

/**
  * @brief This macro is used to set ACMP filter function
  * @param[in] acmp The pointer of the specified ACMP module
  * @param[in] u32ChNum The ACMP number
  * @param[in] u32Cnt is comparator filter count setting.
  *                  - \ref ACMP_FILTSEL_OFF
  *                  - \ref ACMP_FILTSEL_1PCLK
  *                  - \ref ACMP_FILTSEL_2PCLK
  *                  - \ref ACMP_FILTSEL_4PCLK
  *                  - \ref ACMP_FILTSEL_8PCLK
  *                  - \ref ACMP_FILTSEL_16PCLK
  *                  - \ref ACMP_FILTSEL_32PCLK
  *                  - \ref ACMP_FILTSEL_64PCLK
  *                  - \ref ACMP_FILTSEL_128PCLK
  *                  - \ref ACMP_FILTSEL_256PCLK
  *                  - \ref ACMP_FILTSEL_512PCLK
  * @return None
  * \hideinitializer
  */
#define ACMP_SET_FILTER(acmp, u32ChNum, u32Cnt) ((acmp)->CTL[(u32ChNum)] = ((acmp)->CTL[u32ChNum] & ~ACMP_CTL_FILTSEL_Msk) | (u32Cnt))

void ACMP_Open(ACMP_T *acmp, uint32_t u32ChNum, uint32_t u32NegSrc, uint32_t u32HysteresisEn);
void ACMP_Close(ACMP_T *acmp, uint32_t u32ChNum);

/*@}*/ /* end of group Mini58_ACMP_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_ACMP_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__ACMP_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
