/**************************************************************************//**
 * @file     eadc.h
 * @version  V3.00
 * $Revision: 1 $
 * $Date: 17/04/19 7:48p $
 * @brief    Mini57 Series EADC Driver Header File
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __EADC_H__
#define __EADC_H__


#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Mini57_Device_Driver Mini57 Device Driver
  @{
*/

/** @addtogroup Mini57_EADC_Driver EADC Driver
  @{
*/

/** @addtogroup Mini57_EADC_EXPORTED_CONSTANTS EADC Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* EADC Sample Module Constant Definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define EADC_EADC0_DAT0             (0)    /*!< Data buffer 0 of EADC0 (SAMPLE A) */
#define EADC_EADC1_DAT0             (1)    /*!< Data buffer 0 of EADC1 (SAMPLE B) */
#define EADC_EADC0_DAT1             (2)    /*!< Data buffer 1 of EADC0 (SAMPLE A) */
#define EADC_EADC1_DAT1             (3)    /*!< Data buffer 1 of EADC1 (SAMPLE B) */

#define EADC_EADC0                  (EADC_EADC0_0)  /*!< EADC0 (SAMPLE A) */
#define EADC_EADC1                  (EADC_EADC1_0)  /*!< EADC1 (SAMPLE B) */

#define EADC_EADC0_0                  (0)    /*!< EADC0 channel 0 */
#define EADC_EADC0_1                  (1)    /*!< EADC0 channel 1 */
#define EADC_EADC0_2                  (2)    /*!< EADC0 channel 2 */
#define EADC_EADC0_3                  (3)    /*!< EADC0 channel 3 */
#define EADC_EADC0_4                  (4)    /*!< EADC0 channel 4 */
#define EADC_EADC0_5                  (5)    /*!< EADC0 channel 5 */
#define EADC_EADC0_6                  (6)    /*!< EADC0 channel 6 */
#define EADC_EADC0_7                  (7)    /*!< EADC0 channel 7 */
#define EADC_EADC1_0                  (8)    /*!< EADC1 channel 0 */
#define EADC_EADC1_1                  (9)    /*!< EADC1 channel 1 */
#define EADC_EADC1_2                  (10)   /*!< EADC1 channel 2 */
#define EADC_EADC1_3                  (11)   /*!< EADC1 channel 3 */
#define EADC_EADC1_4                  (12)   /*!< EADC1 channel 4 */
#define EADC_EADC1_5                  (13)   /*!< EADC1 channel 5 */
#define EADC_EADC1_6                  (14)   /*!< EADC1 channel 6 */
#define EADC_EADC1_7                  (15)   /*!< EADC1 channel 7 */

/*----------------------------------------------------------------------------------------------------------*/
/* EADC Bit Mask Constant Definitions                                                                       */
/*----------------------------------------------------------------------------------------------------------*/
#define EADC_BIT_MASK_EADC0         (BIT0)  /*!< Bit mask for EADC0 DAT0                */
#define EADC_BIT_MASK_EADC1         (BIT1)  /*!< Bit mask for EADC1 DAT0                */
#define EADC_BIT_MASK_EADC0_DAT1    (BIT2)  /*!< Bit mask for EADC0 DAT1                */
#define EADC_BIT_MASK_EADC1_DAT1    (BIT3)  /*!< Bit mask for EADC1 DAT1                */

/*----------------------------------------------------------------------------------------------------------*/
/* EADC_TRGSOR Constant Definitions                                                                         */
/*----------------------------------------------------------------------------------------------------------*/
#define EADC_SOFTWARE_TRIGGER               (0xFFFFFFFFUL)     /*!< Software trigger (Disable hardware trigger) */

#define EADC_RISING_EDGE_TRIGGER            ((0x0UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << EADC_TRGSOR_ADC0STADCSEL_Pos))     /*!< STADC Rising Edge trigger */
#define EADC_FALLING_EDGE_TRIGGER           ((0x0UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << EADC_TRGSOR_ADC0STADCSEL_Pos))     /*!< STADC Falling Edge trigger */
#define EADC_FALLING_RISING_EDGE_TRIGGER    ((0x0UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << EADC_TRGSOR_ADC0STADCSEL_Pos))     /*!< STADC Rising or Falling Edge trigger */

#define EADC_EPWM0_FALLING_TRIGGER          ((0x1UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM0 Falling trigger */
#define EADC_EPWM0_CENTRAL_TRIGGER          ((0x1UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM0 Counter Central trigger */
#define EADC_EPWM0_RISING_TRIGGER           ((0x1UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM0 Rising trigger */
#define EADC_EPWM0_PERIOD_TRIGGER           ((0x1UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM0 Period trigger */

#define EADC_EPWM1_FALLING_TRIGGER          ((0x2UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM1 Falling trigger */
#define EADC_EPWM1_CENTRAL_TRIGGER          ((0x2UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM1 Counter Central trigger */
#define EADC_EPWM1_RISING_TRIGGER           ((0x2UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM1 Rising trigger */
#define EADC_EPWM1_PERIOD_TRIGGER           ((0x2UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM1 Period trigger */

#define EADC_EPWM2_FALLING_TRIGGER          ((0x3UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM2 Falling trigger */
#define EADC_EPWM2_CENTRAL_TRIGGER          ((0x3UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM2 Counter Central trigger */
#define EADC_EPWM2_RISING_TRIGGER           ((0x3UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM2 Rising trigger */
#define EADC_EPWM2_PERIOD_TRIGGER           ((0x3UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM2 Period trigger */

#define EADC_EPWM3_FALLING_TRIGGER          ((0x4UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM3 Falling trigger */
#define EADC_EPWM3_CENTRAL_TRIGGER          ((0x4UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM3 Counter Central trigger */
#define EADC_EPWM3_RISING_TRIGGER           ((0x4UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM3 Rising trigger */
#define EADC_EPWM3_PERIOD_TRIGGER           ((0x4UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM3 Period trigger */

#define EADC_EPWM4_FALLING_TRIGGER          ((0x5UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM4 Falling trigger */
#define EADC_EPWM4_CENTRAL_TRIGGER          ((0x5UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM4 Counter Central trigger */
#define EADC_EPWM4_RISING_TRIGGER           ((0x5UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM4 Rising trigger */
#define EADC_EPWM4_PERIOD_TRIGGER           ((0x5UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM4 Period trigger */

#define EADC_EPWM5_FALLING_TRIGGER          ((0x6UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM5 Falling trigger */
#define EADC_EPWM5_CENTRAL_TRIGGER          ((0x6UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM5 Counter Central trigger */
#define EADC_EPWM5_RISING_TRIGGER           ((0x6UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM5 Rising trigger */
#define EADC_EPWM5_PERIOD_TRIGGER           ((0x6UL << EADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << EADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM5 Period trigger */

#define EADC_TIMER0_TRIGGER                 (0x7UL << EADC_TRGSOR_ADC0TRGSOR_Pos)        /*!< Timer0 trigger */
#define EADC_TIMER1_TRIGGER                 (0x8UL << EADC_TRGSOR_ADC0TRGSOR_Pos)        /*!< Timer1 trigger */
#define EADC_TIMER2_TRIGGER                 (0x9UL << EADC_TRGSOR_ADC0TRGSOR_Pos)        /*!< Timer2 trigger */

#define EADC_ADC0F_TRIGGER                  (0xAUL << EADC_TRGSOR_ADC0TRGSOR_Pos)        /*!< ADC0F trigger */
#define EADC_ADC1F_TRIGGER                  (0xBUL << EADC_TRGSOR_ADC0TRGSOR_Pos)        /*!< ADC1F trigger */

/*----------------------------------------------------------------------------------------------------------*/
/* EADC_WCMPCTL Constant Definitions                                                                        */
/*----------------------------------------------------------------------------------------------------------*/
#define EADC_WCMP_HIGH_ENABLE       (EADC_WCMPCTL_WCMPHIGHEN_Msk)   /*!< EADC Window Comparator Enable Control: match count if conversion result >= High Bound.                     */
#define EADC_WCMP_MIDDLE_ENABLE     (EADC_WCMPCTL_WCMPMIDEN_Msk)    /*!< EADC Window Comparator Enable Control: match count if conversion result <  High Bound and >= Low Bound.    */
#define EADC_WCMP_LOW_ENABLE        (EADC_WCMPCTL_WCMPLOWEN_Msk)    /*!< EADC Window Comparator Enable Control: match count if conversion result <  Low Bound.                      */

#define EADC_WCMP_FLAG_AUTO_UPDATE  (0)                             /*!< EADC Window Comparator Flag Control: Auto update the Window Comparator flag in EADC_STATUS register        */
#define EADC_WCMP_FLAG_NONE         (EADC_WCMPCTL_WFLAGCTL_Msk)     /*!< EADC Window Comparator Flag Control: No update the Window Comparator flag in EADC_STATUS register          */

/*@}*/ /* end of group Mini57_EADC_EXPORTED_CONSTANTS */

/** @addtogroup Mini57_EADC_EXPORTED_FUNCTIONS EADC Exported Functions
  @{
*/

/**
  * @brief      Enable the interrupt.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32Mask Decides the combination of interrupt enable bits. Each bit corresponds to a interrupt.
  *                     This parameter decides which interrupts will be enabled.
  *                     Valid values are:
  *                     - \ref EADC_CTL_ADC0IEN_Msk
  *                     - \ref EADC_CTL_ADC1IEN_Msk
  * @return     None.
  * @details    The A/D converter generates a conversion end ADCnF (EADC_STATUS[]) upon the end of specific sample module A/D conversion.
  *             If ADCnIEN bit (EADC_CTL[]) is set, the conversion end interrupt request EADCn_INT is generated (n=0~1).
  */
#define EADC_ENABLE_INT(eadc, u32Mask)      ((eadc)->CTL |= (u32Mask))

/**
  * @brief      Disable the interrupt.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32Mask Decides the combination of interrupt enable bits. Each bit corresponds to a interrupt.
  *                     This parameter decides which interrupts will be disabled.
  *                     Valid values are:
  *                     - \ref EADC_CTL_ADC0IEN_Msk
  *                     - \ref EADC_CTL_ADC1IEN_Msk
  * @return     None.
  * @details    Specific sample module A/D EADCn_INT(n=0~1) interrupt function Disabled.
  */
#define EADC_DISABLE_INT(eadc, u32Mask)     ((eadc)->CTL &= ~(u32Mask))

/**
  * @brief      Start the A/D conversion.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleMask The combination of sample module. Each bit corresponds to a sample module.
  *                           This parameter decides which sample module will be conversion.
  *                           Valid values are:
  *                           - \ref EADC_CTL_ADC0SWTRG_Msk
  *                           - \ref EADC_CTL_ADC1SWTRG_Msk
  * @return     None.
  * @details    After write EADC_CTL register to start EADC conversion, the EADC_STATUS register will show which SAMPLE will conversion.
  */
#define EADC_START_CONV(eadc, u32ModuleMask)    ((eadc)->CTL |= (u32ModuleMask))

/**
  * @brief      Get the conversion data of the user-specified sample module.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_EADC0_DAT0
  *                          - \ref EADC_EADC0_DAT1
  *                          - \ref EADC_EADC1_DAT0
  *                          - \ref EADC_EADC1_DAT1
  * @return     Return the conversion data of the user-specified sample module.
  * @details    This macro is used to read ADCnDATm bit (EADC_DAT0[] and EADC_DAT1[]) field to get conversion data. (n=0~1, m=0~1)
  */
#define EADC_GET_CONV_DATA(eadc, u32ModuleNum)      ( \
            ((u32ModuleNum) == EADC_EADC0_DAT0) ? (((eadc)->DAT[0] & EADC_DAT0_ADC0DAT0_Msk) >> EADC_DAT0_ADC0DAT0_Pos) : \
          ( ((u32ModuleNum) == EADC_EADC0_DAT1) ? (((eadc)->DAT[1] & EADC_DAT1_ADC0DAT1_Msk) >> EADC_DAT1_ADC0DAT1_Pos) : \
          ( ((u32ModuleNum) == EADC_EADC1_DAT0) ? (((eadc)->DAT[0] & EADC_DAT0_ADC1DAT0_Msk) >> EADC_DAT0_ADC1DAT0_Pos) : \
          ( ((u32ModuleNum) == EADC_EADC1_DAT1) ? (((eadc)->DAT[1] & EADC_DAT1_ADC1DAT1_Msk) >> EADC_DAT1_ADC1DAT1_Pos) : \
            (0xFFFFFFFF) ) ) )  \
        )

/**
  * @brief      Get the data overrun flag of the user-specified sample module.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleMask The combination of data overrun status bits. Each bit corresponds to a data overrun status.
  *                     Valid values are:
  *                     - \ref EADC_STATUS_ADC0OV_Msk
  *                     - \ref EADC_STATUS_ADC1OV_Msk
  * @return     Return the data overrun flag of the user-specified sample module.
  * @details    This macro is used to read ADCnOV bit (EADC_STATUS[]) field to get data overrun status.
  */
#define EADC_GET_DATA_OVERRUN_FLAG(eadc, u32ModuleMask)     ((eadc)->STATUS & (u32ModuleMask))

/**
  * @brief      Get the data valid flag of the user-specified sample module.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleMask The combination of data valid status bits. Each bit corresponds to a data valid status, valid range are between 1~0xF.
  *                     Bit 0 is EADC0 DAT0, bit 1 is EADC1 DAT0, bit 3 is EADC0 DAT1, bit 4 is EADC1 DAT1 or use macro as below
  *                     - \ref EADC_BIT_MASK_EADC0
  *                     - \ref EADC_BIT_MASK_EADC1
  *                     - \ref EADC_BIT_MASK_EADC0_DAT1
  *                     - \ref EADC_BIT_MASK_EADC1_DAT1
  * @return     Return the data valid flag of the user-specified sample module.
  * @details    This macro is used to read ADCnVALID bit (EADC_DATn[]) field to get data valid flag.
  * @note       Since the valid bit will be cleared by hardware after the EADC_DATn register is read,
  *             user MUST call this function BEFORE any other functions that could to read EADC_DATn register.
  *             That includes EADC_GET_CONV_DATA() and EADC_IS_DATA_VALID().
  */
#define EADC_GET_DATA_VALID_FLAG(eadc, u32ModuleMask)   EADC_Get_Data_Valid_Flag((eadc), (u32ModuleMask))

/**
  * @brief      Get the user-specified interrupt flags.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32Mask The combination of interrupt status bits. Each bit corresponds to a interrupt status.
  *                     Valid values are:
  *                     - \ref EADC_STATUS_ADC0F_Msk
  *                     - \ref EADC_STATUS_ADC1F_Msk
  *                     - \ref EADC_STATUS_WCMPIF_Msk
  * @return     Return the user-specified interrupt flags.
  * @details    This macro is used to get the user-specified interrupt flags.
  */
#define EADC_GET_INT_FLAG(eadc, u32Mask)        ((eadc)->STATUS & (u32Mask))

/**
  * @brief      Clear the selected interrupt status bits.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32Mask The combination of compare interrupt status bits. Each bit corresponds to a compare interrupt status.
  *                     Valid values are:
  *                     - \ref EADC_STATUS_ADC0F_Msk
  *                     - \ref EADC_STATUS_ADC1F_Msk
  *                     - \ref EADC_STATUS_WCMPIF_Msk
  * @return     None.
  * @details    This macro is used to clear the selected interrupt status bits.
  */
#define EADC_CLR_INT_FLAG(eadc, u32Mask)    ((eadc)->STATUS = (u32Mask))

/**
  * @brief      Check all sample module A/D result data register overrun flags.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @retval     0       None of sample module data register overrun flag is set to 1.
  * @retval     1       Any one of sample module data register overrun flag is set to 1.
  * @details    The ADCnOV bit (EADC_STATUS[]) will keep 1 when sample module n data register
  *             overrun flag ADCnOV (EADC_DATm[]) is set to 1. (n=0~1, m=0~1)
  */
#define EADC_IS_DATA_OV(eadc)   (((eadc)->STATUS & (EADC_STATUS_ADC0OV_Msk | EADC_STATUS_ADC01V_Msk)) ? 1 : 0)

/**
  * @brief      Check all sample module A/D result data register valid flags.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @retval     0       None of sample module data register valid flag is set to 1.
  * @retval     1       Any one of sample module data register valid flag is set to 1.
  * @details    The ADCnVALID bit (EADC_DATm[]) will keep 1 when data in the sample module is valid. (n=0~1, m=0~1)
  * @note       Since the valid bit will be cleared by hardware after the EADC_DATn register is read,
  *             user MUST call this function BEFORE any other functions that could to read EADC_DATn register.
  *             That includes EADC_GET_CONV_DATA() and EADC_GET_DATA_VALID_FLAG().
  */
#define EADC_IS_DATA_VALID(eadc)    ( \
          ( ((eadc)->DAT[0] & (EADC_DAT0_ADC0VALID_Msk | EADC_DAT0_ADC1VALID_Msk)) || \
            ((eadc)->DAT[1] & (EADC_DAT1_ADC0VALID_Msk | EADC_DAT1_ADC1VALID_Msk)) )  \
          ? 1 : 0   \
        )

/**
  * @brief      Get the busy state of EADC.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @retval     0       Both A/D converter 0 and 1 are in idle state.
  * @retval     1       A/D converter 0 is in busy state.
  * @retval     2       A/D converter 1 is in busy state.
  * @retval     3       Both A/D converter 0 and 1 are in busy state.
  * @details    This macro is used to read ADC0BUSY/ADC1BUSY bit (EADC_STATUS[3]/EADC_STATUS[11]) to get busy state.
  */
#define EADC_IS_BUSY(eadc)  ( \
          (((eadc)->STATUS & EADC_STATUS_ADC0BUSY_Msk) >> EADC_STATUS_ADC0BUSY_Pos) |     \
          (((eadc)->STATUS & EADC_STATUS_ADC1BUSY_Msk) >> (EADC_STATUS_ADC1BUSY_Pos - 1)) \
        )

/**
  * @brief      Enable the Window Comparator interrupt.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32CMP  Specifies the compare register, valid value are from 0 to 1. This parameter is not used in Mini57.
  * @return     None
  * @details    If the Window Comparator function is enabled and the compare condition matches the setting of EADC_WCMPCTL,
  *             WCMPIF (EADC_STATUS[16]) will be asserted, in the meanwhile,
  *             if WCMPIEN is set to 1, a compare interrupt request is generated.
  */
#define EADC_ENABLE_CMP_INT(eadc, u32CMP)   ((eadc)->WCMPCTL |= EADC_WCMPCTL_WCMPIEN_Msk)

/**
  * @brief      Disable the Window Comparator interrupt.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32CMP  Specifies the compare register, valid value are from 0 to 1. This parameter is not used in Mini57.
  * @return     None.
  * @details    This macro is used to disable the Window Comparator interrupt.
  */
#define EADC_DISABLE_CMP_INT(eadc, u32CMP)  ((eadc)->WCMPCTL &= ~EADC_WCMPCTL_WCMPIEN_Msk)

/**
  * @brief      Set the EADC conversion mode to Independent Simple Mode.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @return     None.
  * @details    This macro is used to set the EADC conversion mode to Independent Simple Mode.
  */
#define EADC_SET_INDEPENDENT_SIMPLE_MODE(eadc)      ((eadc)->CTL = ((eadc)->CTL & ~EADC_CTL_ADCMODE_Msk) | (0x0 << EADC_CTL_ADCMODE_Pos))

/**
  * @brief      Set the EADC conversion mode to Independent 2SH Mode.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @return     None.
  * @details    This macro is used to set the EADC conversion mode to Independent 2SH Mode.
  */
#define EADC_SET_INDEPENDENT_2SH_MODE(eadc)         ((eadc)->CTL = ((eadc)->CTL & ~EADC_CTL_ADCMODE_Msk) | (0x1 << EADC_CTL_ADCMODE_Pos))

/**
  * @brief      Set the EADC conversion mode to Simultaneous Simple Mode.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @return     None.
  * @details    This macro is used to set the EADC conversion mode to Simultaneous Simple Mode.
  */
#define EADC_SET_SIMULTANEOUS_SIMPLE_MODE(eadc)     ((eadc)->CTL = ((eadc)->CTL & ~EADC_CTL_ADCMODE_Msk) | (0x2 << EADC_CTL_ADCMODE_Pos))

/**
  * @brief      Set the EADC conversion mode to Simultaneous Sequential 3R Mode.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleNumA Decides the sample module number for sequential input on sample module 0, valid value are:
  *                          - \ref EADC_EADC0_0
  *                          - \ref EADC_EADC0_1
  *                          - \ref EADC_EADC0_2
  *                          - \ref EADC_EADC0_3
  *                          - \ref EADC_EADC0_4
  *                          - \ref EADC_EADC0_5
  *                          - \ref EADC_EADC0_6
  *                          - \ref EADC_EADC0_7
  * @return     None.
  * @details    This macro is used to set the EADC conversion mode to Simultaneous Sequential 3R Mode.
  */
#define EADC_SET_SIMULTANEOUS_3R_MODE(eadc, u32ModuleNumA)   (   \
        (eadc)->CTL = ((eadc)->CTL & ~(EADC_CTL_ADCMODE_Msk | EADC_CTL_ADCSS3R_Msk | EADC_CTL_ADC0SEQSEL_Msk)) \
                      | (0x3 << EADC_CTL_ADCMODE_Pos) | EADC_CTL_ADCSS3R_Msk | (u32ModuleNumA << EADC_CTL_ADC0SEQSEL_Pos) )

/**
  * @brief      Set the EADC conversion mode to Simultaneous Sequential 4R Mode.
  * @param[in]  eadc    The pointer of the specified EADC module.
  * @param[in]  u32ModuleNumA Decides the sample module number for sequential input on sample module 0, valid value are:
  *                          - \ref EADC_EADC0_0
  *                          - \ref EADC_EADC0_1
  *                          - \ref EADC_EADC0_2
  *                          - \ref EADC_EADC0_3
  *                          - \ref EADC_EADC0_4
  *                          - \ref EADC_EADC0_5
  *                          - \ref EADC_EADC0_6
  *                          - \ref EADC_EADC0_7
  * @param[in]  u32ModuleNumB Decides the sample module number for sequential input on sample module 1, valid value are:
  *                          - \ref EADC_EADC1_0
  *                          - \ref EADC_EADC1_1
  *                          - \ref EADC_EADC1_2
  *                          - \ref EADC_EADC1_3
  *                          - \ref EADC_EADC1_4
  *                          - \ref EADC_EADC1_5
  *                          - \ref EADC_EADC1_6
  *                          - \ref EADC_EADC1_7
  * @return     None.
  * @details    This macro is used to set the EADC conversion mode to Simultaneous Sequential 4R Mode.
  */
#define EADC_SET_SIMULTANEOUS_4R_MODE(eadc, u32ModuleNumA, u32ModuleNumB)   (   \
        (eadc)->CTL = ((eadc)->CTL & ~(EADC_CTL_ADCMODE_Msk | EADC_CTL_ADCSS3R_Msk | EADC_CTL_ADC0SEQSEL_Msk | EADC_CTL_ADC1SEQSEL_Msk)) \
                      | (0x3 << EADC_CTL_ADCMODE_Pos) | (u32ModuleNumA << EADC_CTL_ADC0SEQSEL_Pos) | ((u32ModuleNumB - EADC_EADC1) << EADC_CTL_ADC1SEQSEL_Pos) )


/*---------------------------------------------------------------------------------------------------------*/
/* Define EADC functions prototype                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_Open(EADC_T *eadc, uint32_t u32InputMode);
void EADC_Close(EADC_T *eadc);
void EADC_ConfigSampleModule(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32TriggerSource, uint32_t u32Channel);
void EADC_SetTriggerDelayTime(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32TriggerDelayTime, uint32_t u32DelayClockDivider);
void EADC_SetExtendSampleTime(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32ExtendSampleTime);
void EADC_EnablePWMTrigger(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32Source, uint32_t u32Param);
void EADC_DisablePWMTrigger(EADC_T *eadc, uint32_t u32ModuleNum, uint32_t u32Source, uint32_t u32Param);
void EADC_DisableAllPWMTrigger(EADC_T *eadc, uint32_t u32ModuleNum);
void EADC_DisableWCMP(EADC_T *eadc);
void EADC_EnableWCMP(EADC_T *eadc, uint32_t u32HighBound, uint32_t u32LowBound, uint32_t u32FlagEN, uint32_t u32MatchCount, uint32_t u32FlagCTL);
void EADC_ResetWCMPCounter(EADC_T *eadc);
uint32_t EADC_Get_Data_Valid_Flag(EADC_T *eadc, uint32_t u32ModuleMask);

/*@}*/ /* end of group Mini57_EADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini57_EADC_Driver */

/*@}*/ /* end of group Mini57_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __EADC_H__ */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
