/**************************************************************************//**
 * @file     adc.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 2018/05/31 16:38 $
 * @brief    NM1230 ADC Driver Header File
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __ADC_H__
#define __ADC_H__


#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup NM1230_Device_Driver NM1230 Device Driver
  @{
*/

/** @addtogroup NM1230_ADC_Driver ADC Driver
  @{
*/

/** @addtogroup NM1230_ADC_EXPORTED_CONSTANTS ADC Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* ADC Sample Module Constant Definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_ADC0_DAT0             (0)    /*!< Data buffer 0 of ADC0 (SAMPLE A) */
#define ADC_ADC1_DAT0             (1)    /*!< Data buffer 0 of ADC1 (SAMPLE B) */
#define ADC_ADC0_DAT1             (2)    /*!< Data buffer 1 of ADC0 (SAMPLE A) */
#define ADC_ADC1_DAT1             (3)    /*!< Data buffer 1 of ADC1 (SAMPLE B) */

#define ADC_ADC0                  (ADC0SEL_ADC0_CH0)  /*!< ADC0 (SAMPLE A) */
#define ADC_ADC1                  (ADC1SEL_ADC1_CH0)  /*!< ADC1 (SAMPLE B) */

#define ADC0SEL_ADC0_CH0          (0)    /*!< ADC0 channel select ADC0_CH0 */
#define ADC0SEL_ADC0_CH1          (1)    /*!< ADC0 channel select ADC0_CH1 */
#define ADC0SEL_ADC0_CH2          (2)    /*!< ADC0 channel select ADC0_CH2 */
#define ADC0SEL_ADC0_CH3          (3)    /*!< ADC0 channel select ADC0_CH3 */
#define ADC0SEL_ADC0_CH4          (4)    /*!< ADC0 channel select ADC0_CH4 */
#define ADC0SEL_PGA               (5)    /*!< ADC0 channel select PGA */
#define ADC0SEL_BAND_GAP          (6)    /*!< ADC0 channel select band gap */
#define ADC0SEL_VSS               (7)    /*!< ADC0 channel select VSS */
#define ADC0SEL_OP0               (8)    /*!< ADC0 channel select OP0 */
#define ADC0SEL_OP1               (9)    /*!< ADC0 channel select OP1 */
#define ADC0SEL_OP2               (10)   /*!< ADC0 channel select OP2 */
#define ADC0SEL_DAC0              (11)   /*!< ADC0 channel select DAC0 */
#define ADC0SEL_DAC1              (12)   /*!< ADC0 channel select DAC1 */
#define ADC0SEL_ADC0_CH5          (13)   /*!< ADC0 channel select ADC0_CH5 */
#define ADC0SEL_ADC0_CH6          (14)   /*!< ADC0 channel select ADC0_CH6 */
#define ADC0SEL_ADC0_CH7          (15)   /*!< ADC0 channel select ADC0_CH7 */
#define ADC1SEL_ADC1_CH0          (16)   /*!< ADC1 channel select ADC1_CH0 */
#define ADC1SEL_ADC1_CH1          (17)   /*!< ADC1 channel select ADC1_CH1 */
#define ADC1SEL_ADC1_CH2          (18)   /*!< ADC1 channel select ADC1_CH2 */
#define ADC1SEL_ADC0_CH0          (19)   /*!< ADC1 channel select ADC0_CH0 */
#define ADC1SEL_ADC0_CH4          (20)   /*!< ADC1 channel select ADC0_CH4 */
#define ADC1SEL_PGA               (21)   /*!< ADC1 channel select PGA */
#define ADC1SEL_TEMP_SNR          (22)   /*!< ADC1 channel select temperature sensor */
#define ADC1SEL_VSS               (23)   /*!< ADC1 channel select VSS */
#define ADC1SEL_OP0               (24)   /*!< ADC1 channel select OP0 */
#define ADC1SEL_OP1               (25)   /*!< ADC1 channel select OP1 */
#define ADC1SEL_OP2               (26)   /*!< ADC1 channel select OP2 */
#define ADC1SEL_ADC1_CH3          (27)   /*!< ADC1 channel select ADC1_CH3 */
#define ADC1SEL_ADC1_CH4          (28)   /*!< ADC1 channel select ADC1_CH4 */
#define ADC1SEL_ADC1_CH5          (29)   /*!< ADC1 channel select ADC1_CH5 */
#define ADC1SEL_ADC1_CH6          (30)   /*!< ADC1 channel select ADC1_CH6 */
#define ADC1SEL_ADC1_CH7          (31)   /*!< ADC1 channel select ADC1_CH7 */

/*----------------------------------------------------------------------------------------------------------*/
/* ADC Bit Mask Constant Definitions                                                                       */
/*----------------------------------------------------------------------------------------------------------*/
#define ADC_BIT_MASK_ADC0         (BIT0)  /*!< Bit mask for ADC0 DAT0                */
#define ADC_BIT_MASK_ADC1         (BIT1)  /*!< Bit mask for ADC1 DAT0                */
#define ADC_BIT_MASK_ADC0_DAT1    (BIT2)  /*!< Bit mask for ADC0 DAT1                */
#define ADC_BIT_MASK_ADC1_DAT1    (BIT3)  /*!< Bit mask for ADC1 DAT1                */

/*----------------------------------------------------------------------------------------------------------*/
/* ADC_TRGSOR Constant Definitions                                                                         */
/*----------------------------------------------------------------------------------------------------------*/
#define ADC_SOFTWARE_TRIGGER               (0xFFFFFFFFUL)     /*!< Software trigger (Disable hardware trigger) */

#define ADC_RISING_EDGE_TRIGGER            ((0x0UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << ADC_TRGSOR_ADC0STADCSEL_Pos))     /*!< STADC Rising Edge trigger */
#define ADC_FALLING_EDGE_TRIGGER           ((0x0UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << ADC_TRGSOR_ADC0STADCSEL_Pos))     /*!< STADC Falling Edge trigger */
#define ADC_FALLING_RISING_EDGE_TRIGGER    ((0x0UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << ADC_TRGSOR_ADC0STADCSEL_Pos))     /*!< STADC Rising or Falling Edge trigger */

#define ADC_EPWM0_FALLING_TRIGGER          ((0x1UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM0 Falling trigger */
#define ADC_EPWM0_CENTRAL_TRIGGER          ((0x1UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM0 Counter Central trigger */
#define ADC_EPWM0_RISING_TRIGGER           ((0x1UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM0 Rising trigger */
#define ADC_EPWM0_PERIOD_TRIGGER           ((0x1UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM0 Period trigger */

#define ADC_EPWM1_FALLING_TRIGGER          ((0x2UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM1 Falling trigger */
#define ADC_EPWM1_CENTRAL_TRIGGER          ((0x2UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM1 Counter Central trigger */
#define ADC_EPWM1_RISING_TRIGGER           ((0x2UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM1 Rising trigger */
#define ADC_EPWM1_PERIOD_TRIGGER           ((0x2UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM1 Period trigger */

#define ADC_EPWM2_FALLING_TRIGGER          ((0x3UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM2 Falling trigger */
#define ADC_EPWM2_CENTRAL_TRIGGER          ((0x3UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM2 Counter Central trigger */
#define ADC_EPWM2_RISING_TRIGGER           ((0x3UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM2 Rising trigger */
#define ADC_EPWM2_PERIOD_TRIGGER           ((0x3UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM2 Period trigger */

#define ADC_EPWM3_FALLING_TRIGGER          ((0x4UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM3 Falling trigger */
#define ADC_EPWM3_CENTRAL_TRIGGER          ((0x4UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM3 Counter Central trigger */
#define ADC_EPWM3_RISING_TRIGGER           ((0x4UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM3 Rising trigger */
#define ADC_EPWM3_PERIOD_TRIGGER           ((0x4UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM3 Period trigger */

#define ADC_EPWM4_FALLING_TRIGGER          ((0x5UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM4 Falling trigger */
#define ADC_EPWM4_CENTRAL_TRIGGER          ((0x5UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM4 Counter Central trigger */
#define ADC_EPWM4_RISING_TRIGGER           ((0x5UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM4 Rising trigger */
#define ADC_EPWM4_PERIOD_TRIGGER           ((0x5UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM4 Period trigger */

#define ADC_EPWM5_FALLING_TRIGGER          ((0x6UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x0UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM5 Falling trigger */
#define ADC_EPWM5_CENTRAL_TRIGGER          ((0x6UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x1UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM5 Counter Central trigger */
#define ADC_EPWM5_RISING_TRIGGER           ((0x6UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x2UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM5 Rising trigger */
#define ADC_EPWM5_PERIOD_TRIGGER           ((0x6UL << ADC_TRGSOR_ADC0TRGSOR_Pos) | (0x3UL << ADC_TRGSOR_ADC0PWMTRGSEL_Pos))    /*!< EPWM5 Period trigger */

#define ADC_TIMER0_TRIGGER                 (0x7UL << ADC_TRGSOR_ADC0TRGSOR_Pos)        /*!< Timer0 match trigger */
#define ADC_TIMER1_TRIGGER                 (0x8UL << ADC_TRGSOR_ADC0TRGSOR_Pos)        /*!< Timer1 match trigger */
#define ADC_ECAPPHG_TRIGGER                (0x9UL << ADC_TRGSOR_ADC0TRGSOR_Pos)        /*!< ECAP trigger EPWM phase change event trigger */

#define ADC_ADC0F_TRIGGER                  (0xAUL << ADC_TRGSOR_ADC0TRGSOR_Pos)        /*!< ADC0F trigger */
#define ADC_ADC1F_TRIGGER                  (0xBUL << ADC_TRGSOR_ADC0TRGSOR_Pos)        /*!< ADC1F trigger */

/*----------------------------------------------------------------------------------------------------------*/
/* ADC_WCMPCTL Constant Definitions                                                                        */
/*----------------------------------------------------------------------------------------------------------*/
#define ADC_WCMP_HIGH_ENABLE                (ADC_WCMPCTL_WCMPHIGHEN_Msk)                /*!< ADC Window Comparator Enable Control: match count if conversion result >= High Bound.                     */
#define ADC_WCMP_MIDDLE_ENABLE              (ADC_WCMPCTL_WCMPMIDEN_Msk)                 /*!< ADC Window Comparator Enable Control: match count if conversion result <  High Bound and >= Low Bound.    */
#define ADC_WCMP_LOW_ENABLE                 (ADC_WCMPCTL_WCMPLOWEN_Msk)                 /*!< ADC Window Comparator Enable Control: match count if conversion result <  Low Bound.                      */

#define ADC_WCMP_FLAG_AUTO_UPDATE           (0)                                         /*!< ADC Window Comparator Flag Control: Auto update the Window Comparator flag in ADC_STATUS register        */
#define ADC_WCMP_FLAG_NONE                  (ADC_WCMPCTL_WFLAGCTL_Msk)                  /*!< ADC Window Comparator Flag Control: No update the Window Comparator flag in ADC_STATUS register          */

/*----------------------------------------------------------------------------------------------------------*/
/* ADC_SPEED Constant Definitions                                                                        */
/*----------------------------------------------------------------------------------------------------------*/
#define ADC_CONVERTION_RATE_FAST            (0 << ADC_CTL_ADCSPEED_POS)                 /*!< ADC fast convertion rate, but with slow accurate. */
#define ADC_CONVERTION_RATE_MIDDLE          (1 << ADC_CTL_ADCSPEED_POS)                 /*!< ADC middle convertion rate and accurate. */
#define ADC_CONVERTION_RATE_SLOW            (3 << ADC_CTL_ADCSPEED_POS)                 /*!< ADC slow convertion rate, but with high accurate. */


/*@}*/ /* end of group NM1230_ADC_EXPORTED_CONSTANTS */

/** @addtogroup NM1230_ADC_EXPORTED_FUNCTIONS ADC Exported Functions
  @{
*/

/**
  * @brief      Set the ADC conversion rate.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32rate is the selection of conversion rate, valid value are:
  *                          - \ref ADC_CONVERTION_RATE_FAST  
  *                          - \ref ADC_CONVERTION_RATE_MIDDLE
  *                          - \ref ADC_CONVERTION_RATE_SLOW  
  * @return     None.
  * @details    This macro is used to set the ADC conversion rate.
  */
#define ADC_SET_CONVERSION_RATE(adc, u32rate)    ((adc)->CTL = (adc)->CTL & ~(ADC_CTL_ADCSPEED_MSK) | (u32rate))

/**
  * @brief      Enable the interrupt.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32Mask Decides the combination of interrupt enable bits. Each bit corresponds to a interrupt.
  *                     This parameter decides which interrupts will be enabled.
  *                     Valid values are:
  *                     - \ref ADC_CTL_ADC0IEN_Msk
  *                     - \ref ADC_CTL_ADC1IEN_Msk
  * @return     None.
  * @details    The A/D converter generates a conversion end ADCnF (ADC_STATUS[]) upon the end of specific sample module A/D conversion.
  *             If ADCnIEN bit (ADC_CTL[]) is set, the conversion end interrupt request ADCn_INT is generated (n=0~1).
  */
#define ADC_ENABLE_INT(adc, u32Mask)    ((adc)->CTL |= (u32Mask))

/**
  * @brief      Disable the interrupt.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32Mask Decides the combination of interrupt enable bits. Each bit corresponds to a interrupt.
  *                     This parameter decides which interrupts will be disabled.
  *                     Valid values are:
  *                     - \ref ADC_CTL_ADC0IEN_Msk
  *                     - \ref ADC_CTL_ADC1IEN_Msk
  * @return     None.
  * @details    Specific sample module A/D ADCn_INT(n=0~1) interrupt function Disabled.
  */
#define ADC_DISABLE_INT(adc, u32Mask)   ((adc)->CTL &= ~(u32Mask))

/**
  * @brief      Start the A/D conversion.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleMask The combination of sample module. Each bit corresponds to a sample module.
  *                           This parameter decides which sample module will be conversion.
  *                           Valid values are:
  *                           - \ref ADC_CTL_ADC0SWTRG_Msk
  *                           - \ref ADC_CTL_ADC1SWTRG_Msk
  * @return     None.
  * @details    After write ADC_CTL register to start ADC conversion, the ADC_STATUS register will show which SAMPLE will conversion.
  */
#define ADC_START_CONV(adc, u32ModuleMask)    ((adc)->CTL |= (u32ModuleMask))

/**
  * @brief      Get the conversion data of the user-specified sample module.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref ADC_ADC0_DAT0
  *                          - \ref ADC_ADC0_DAT1
  *                          - \ref ADC_ADC1_DAT0
  *                          - \ref ADC_ADC1_DAT1
  * @return     Return the conversion data of the user-specified sample module.
  * @details    This macro is used to read ADCnDATm bit (ADC_DAT0 and ADC_DAT1) field to get conversion data. (n=0~1, m=0~1)
  */
#define ADC_GET_CONV_DATA(adc, u32ModuleNum)    ( \
            ((u32ModuleNum) == ADC_ADC0_DAT0) ? (((adc)->DAT0 & ADC_DAT0_ADC0DAT0_Msk) >> ADC_DAT0_ADC0DAT0_Pos) : \
          ( ((u32ModuleNum) == ADC_ADC0_DAT1) ? (((adc)->DAT1 & ADC_DAT1_ADC0DAT1_Msk) >> ADC_DAT1_ADC0DAT1_Pos) : \
          ( ((u32ModuleNum) == ADC_ADC1_DAT0) ? (((adc)->DAT0 & ADC_DAT0_ADC1DAT0_Msk) >> ADC_DAT0_ADC1DAT0_Pos) : \
          ( ((u32ModuleNum) == ADC_ADC1_DAT1) ? (((adc)->DAT1 & ADC_DAT1_ADC1DAT1_Msk) >> ADC_DAT1_ADC1DAT1_Pos) : \
            (0xFFFFFFFF) ) ) )  \
        )

/**
  * @brief      Get the secondary conversion data of the user-specified sample module.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref ADC_ADC0_DAT0
  *                          - \ref ADC_ADC0_DAT1
  *                          - \ref ADC_ADC1_DAT0
  *                          - \ref ADC_ADC1_DAT1
  * @return     Return the secondary conversion data of the user-specified sample module.
  * @details    This macro is used to read ADCnDATm bit (ADC_SECDAT0 and ADC_SECDAT1) field to get conversion data. (n=0~1, m=0~1)
  */
#define ADC_GET_SEC_CONV_DATA(adc, u32ModuleNum)    ( \
            ((u32ModuleNum) == ADC_ADC0_DAT0) ? (((adc)->SECDAT0 & ADC_SECDAT0_ADC0DAT0_Msk) >> ADC_SECDAT0_ADC0DAT0_Pos) : \
          ( ((u32ModuleNum) == ADC_ADC0_DAT1) ? (((adc)->SECDAT1 & ADC_SECDAT1_ADC0DAT1_Msk) >> ADC_SECDAT1_ADC0DAT1_Pos) : \
          ( ((u32ModuleNum) == ADC_ADC1_DAT0) ? (((adc)->SECDAT0 & ADC_SECDAT0_ADC1DAT0_Msk) >> ADC_SECDAT0_ADC1DAT0_Pos) : \
          ( ((u32ModuleNum) == ADC_ADC1_DAT1) ? (((adc)->SECDAT1 & ADC_SECDAT1_ADC1DAT1_Msk) >> ADC_SECDAT1_ADC1DAT1_Pos) : \
            (0xFFFFFFFF) ) ) )  \
        )
        
/**
  * @brief      Get the data overrun flag of the user-specified sample module.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleMask The combination of data overrun status bits. Each bit corresponds to a data overrun status.
  *                     Valid values are:
  *                     - \ref ADC_STATUS_ADC0OV_Msk
  *                     - \ref ADC_STATUS_ADC1OV_Msk
  * @return     Return the data overrun flag of the user-specified sample module.
  * @details    This macro is used to read ADCnOV bit (ADC_STATUS[]) field to get data overrun status.
  */
#define ADC_GET_DATA_OVERRUN_FLAG(adc, u32ModuleMask)    ((adc)->STATUS & (u32ModuleMask))

///**
//  * @brief      Get the data valid flag of the user-specified sample module.
//  * @param[in]  adc    The pointer of the specified ADC module.
//  * @param[in]  u32ModuleMask The combination of data valid status bits. Each bit corresponds to a data valid status, valid range are between 1~0xF.
//  *                     Bit 0 is ADC0 DAT0, bit 1 is ADC1 DAT0, bit 3 is ADC0 DAT1, bit 4 is ADC1 DAT1 or use macro as below
//  *                     - \ref ADC_BIT_MASK_ADC0
//  *                     - \ref ADC_BIT_MASK_ADC1
//  *                     - \ref ADC_BIT_MASK_ADC0_DAT1
//  *                     - \ref ADC_BIT_MASK_ADC1_DAT1
//  * @return     Return the data valid flag of the user-specified sample module.
//  * @details    This macro is used to read ADCnVALID bit (ADC_DATn[]) field to get data valid flag.
//  * @note       Since the valid bit will be cleared by hardware after the ADC_DATn register is read,
//  *             user MUST call this function BEFORE any other functions that could to read ADC_DATn register.
//  *             That includes ADC_GET_CONV_DATA() and ADC_IS_DATA_VALID().
//  */
//#define ADC_GET_DATA_VALID_FLAG(adc, u32ModuleMask)    ADC_Get_Data_Valid_Flag((adc), (u32ModuleMask))

/**
  * @brief      Get the user-specified interrupt flags.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32Mask The combination of interrupt status bits. Each bit corresponds to a interrupt status.
  *                     Valid values are:
  *                     - \ref ADC_STATUS_ADC0IF_Msk
  *                     - \ref ADC_STATUS_ADC1IF_Msk
  *                     - \ref ADC_STATUS_WCMPIF_Msk
  *                     - \ref ADC_STATUS_LOWFG_Msk
  *                     - \ref ADC_STATUS_MIDFG_Msk
  *                     - \ref ADC_STATUS_HIGHFG_Msk
  * @return     Return the user-specified interrupt flags.
  * @details    This macro is used to get the user-specified interrupt flags.
  */
#define ADC_GET_INT_FLAG(adc, u32Mask)    ((adc)->STATUS & (u32Mask))

/**
  * @brief      Clear the selected interrupt status bits.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32Mask The combination of compare interrupt status bits. Each bit corresponds to a compare interrupt status.
  *                     Valid values are:
  *                     - \ref ADC_STATUS_ADC0IF_Msk
  *                     - \ref ADC_STATUS_ADC1IF_Msk
  *                     - \ref ADC_STATUS_WCMPIF_Msk
  *                     - \ref ADC_STATUS_LOWFG_Msk
  *                     - \ref ADC_STATUS_MIDFG_Msk
  *                     - \ref ADC_STATUS_HIGHFG_Msk  
  * @return     None.
  * @details    This macro is used to clear the selected interrupt status bits.
  */
#define ADC_CLR_INT_FLAG(adc, u32Mask)    ((adc)->STATUS = (u32Mask))

/**
  * @brief      Check all sample module A/D result data register overrun flags.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @retval     0       None of sample module data register overrun flag is set to 1.
  * @retval     1       Any one of sample module data register overrun flag is set to 1.
  * @details    The ADCnOV bit (ADC_STATUS[]) will keep 1 when sample module n data register
  *             overrun flag ADCnOV (ADC_DATm[]) is set to 1. (n=0~1, m=0~1)
  */
#define ADC_IS_DATA_OV(adc)    (((adc)->STATUS & (ADC_STATUS_ADC0OV_Msk | ADC_STATUS_ADC01V_Msk)) ? 1 : 0)

/**
  * @brief      Get the current conversion channel
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ch  The sample module of ADC. Valid values are:
  *                     - \ref ADC_ADC0
  *                     - \ref ADC_ADC1
  * @return     Return the current convert channel of selected ADC sample module.
  * @details    This macro is used to get the user-specified interrupt flags.
  */
#define ADC_GET_CURRENT_CONV_CH(adc, u32ch)    (\
          ((u32ch == ADC_ADC0) ? \
          (((adc)->STATUS & (ADC_STATUS_ADC0CH_Msk)) >> ADC_STATUS_ADC0CH_Pos):\
          ((((adc)->STATUS & (ADC_STATUS_ADC1CH_Msk)) >> ADC_STATUS_ADC1CH_Pos) + ADC1SEL_ADC1_CH0))\
        )

/**
  * @brief      Check all sample module A/D result data register valid flags.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @retval     0       None of sample module data register valid flag is set to 1.
  * @retval     1       Any one of sample module data register valid flag is set to 1.
  * @details    The ADCnVALID bit (ADC_DATm[]) will keep 1 when data in the sample module is valid. (n=0~1, m=0~1)
  * @note       Since the valid bit will be cleared by hardware after the ADC_DATn register is read,
  *             user MUST call this function BEFORE any other functions that could to read ADC_DATn register.
  *             That includes ADC_GET_CONV_DATA() and ADC_GET_DATA_VALID_FLAG().
  */
#define ADC_IS_DATA_VALID(adc)    ( \
          ( ((adc)->DAT[0] & (ADC_DAT0_ADC0VALID_Msk | ADC_DAT0_ADC1VALID_Msk)) || \
            ((adc)->DAT[1] & (ADC_DAT1_ADC0VALID_Msk | ADC_DAT1_ADC1VALID_Msk)) )  \
          ? 1 : 0   \
        )

/**
  * @brief      Get the busy state of ADC.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @retval     0       Both A/D converter 0 and 1 are in idle state.
  * @retval     1       A/D converter 0 is in busy state.
  * @retval     2       A/D converter 1 is in busy state.
  * @retval     3       Both A/D converter 0 and 1 are in busy state.
  * @details    This macro is used to read ADC0BUSY/ADC1BUSY bit (ADC_STATUS[3]/ADC_STATUS[11]) to get busy state.
  */
#define ADC_IS_BUSY(adc)    ( \
          (((adc)->STATUS & ADC_STATUS_ADC0BUSY_Msk) >> ADC_STATUS_ADC0BUSY_Pos) |     \
          (((adc)->STATUS & ADC_STATUS_ADC1BUSY_Msk) >> (ADC_STATUS_ADC1BUSY_Pos - 1)) \
        )

/**
  * @brief      Enable the Window Comparator interrupt.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32CMP  Specifies the compare register, valid value are from 0 to 1. This parameter is not used in NM1230.
  * @return     None
  * @details    If the Window Comparator function is enabled and the compare condition matches the setting of ADC_WCMPCTL,
  *             WCMPIF (ADC_STATUS[16]) will be asserted, in the meanwhile,
  *             if WCMPIEN is set to 1, a compare interrupt request is generated.
  */
#define ADC_ENABLE_CMP_INT(adc, u32CMP)    ((adc)->WCMPCTL |= ADC_WCMPCTL_WCMPIEN_Msk)

/**
  * @brief      Disable the Window Comparator interrupt.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32CMP  Specifies the compare register, valid value are from 0 to 1. This parameter is not used in NM1230.
  * @return     None.
  * @details    This macro is used to disable the Window Comparator interrupt.
  */
#define ADC_DISABLE_CMP_INT(adc, u32CMP)    ((adc)->WCMPCTL &= ~ADC_WCMPCTL_WCMPIEN_Msk)

/**
  * @brief      Set the ADC conversion mode to Independent Simple Mode.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @return     None.
  * @details    This macro is used to set the ADC conversion mode to Independent Simple Mode.
  */
#define ADC_SET_INDEPENDENT_SIMPLE_MODE(adc)    ((adc)->CTL = ((adc)->CTL & ~ADC_CTL_ADCMODE_Msk) | (0x0 << ADC_CTL_ADCMODE_Pos))

/**
  * @brief      Set the ADC conversion mode to Independent 2SH Mode.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @return     None.
  * @details    This macro is used to set the ADC conversion mode to Independent 2SH Mode.
  */
#define ADC_SET_INDEPENDENT_2SH_MODE(adc)    ((adc)->CTL = ((adc)->CTL & ~ADC_CTL_ADCMODE_Msk) | (0x1 << ADC_CTL_ADCMODE_Pos))

/**
  * @brief      Set the ADC conversion mode to Simultaneous Simple Mode.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @return     None.
  * @details    This macro is used to set the ADC conversion mode to Simultaneous Simple Mode.
  */
#define ADC_SET_SIMULTANEOUS_SIMPLE_MODE(adc)    ((adc)->CTL = ((adc)->CTL & ~ADC_CTL_ADCMODE_Msk) | (0x2 << ADC_CTL_ADCMODE_Pos))

/**
  * @brief      Set the ADC conversion mode to Simultaneous Sequential 3R Mode.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleNumA Decides the sample module number for sequential input on sample module 0, valid value are:
  *                          - \ref ADC0SEL_ADC0_CH0
  *                          - \ref ADC0SEL_ADC0_CH1
  *                          - \ref ADC0SEL_ADC0_CH2
  *                          - \ref ADC0SEL_ADC0_CH3
  *                          - \ref ADC0SEL_ADC0_CH4
  *                          - \ref ADC0SEL_PGA 
  *                          - \ref ADC0SEL_BAND_GAP
  *                          - \ref ADC0SEL_VSS     
  *                          - \ref ADC0SEL_OP0     
  *                          - \ref ADC0SEL_OP1     
  *                          - \ref ADC0SEL_OP2     
  *                          - \ref ADC0SEL_DAC0    
  *                          - \ref ADC0SEL_DAC1    
  *                          - \ref ADC0SEL_ADC0_CH5
  *                          - \ref ADC0SEL_ADC0_CH6
  *                          - \ref ADC0SEL_ADC0_CH7
  * @return     None.
  * @details    This macro is used to set the ADC conversion mode to Simultaneous Sequential 3R Mode.
  */
#define ADC_SET_SIMULTANEOUS_3R_MODE(adc, u32ModuleNumA)    (   \
        (adc)->CTL = ((adc)->CTL & ~(ADC_CTL_ADCMODE_Msk | ADC_CTL_ADCSS3R_Msk | ADC_CTL_ADC0SEQSEL_Msk)) \
                      | (0x3 << ADC_CTL_ADCMODE_Pos) | ADC_CTL_ADCSS3R_Msk | (u32ModuleNumA << ADC_CTL_ADC0SEQSEL_Pos) )

/**
  * @brief      Set the ADC conversion mode to Simultaneous Sequential 4R Mode.
  * @param[in]  adc    The pointer of the specified ADC module.
  * @param[in]  u32ModuleNumA Decides the sample module number for sequential input on sample module 0, valid value are:
  *                          - \ref ADC0SEL_ADC0_CH0
  *                          - \ref ADC0SEL_ADC0_CH1
  *                          - \ref ADC0SEL_ADC0_CH2
  *                          - \ref ADC0SEL_ADC0_CH3
  *                          - \ref ADC0SEL_ADC0_CH4
  *                          - \ref ADC0SEL_PGA 
  *                          - \ref ADC0SEL_BAND_GAP
  *                          - \ref ADC0SEL_VSS     
  *                          - \ref ADC0SEL_OP0     
  *                          - \ref ADC0SEL_OP1     
  *                          - \ref ADC0SEL_OP2     
  *                          - \ref ADC0SEL_DAC0    
  *                          - \ref ADC0SEL_DAC1    
  *                          - \ref ADC0SEL_ADC0_CH5
  *                          - \ref ADC0SEL_ADC0_CH6
  *                          - \ref ADC0SEL_ADC0_CH7
  * @param[in]  u32ModuleNumB Decides the sample module number for sequential input on sample module 1, valid value are:
  *                          - \ref ADC1SEL_ADC1_CH0
  *                          - \ref ADC1SEL_ADC1_CH1
  *                          - \ref ADC1SEL_ADC1_CH2
  *                          - \ref ADC1SEL_ADC0_CH0
  *                          - \ref ADC1SEL_ADC0_CH4
  *                          - \ref ADC1SEL_PGA     
  *                          - \ref ADC1SEL_TEMP_SNR
  *                          - \ref ADC1SEL_VSS     
  *                          - \ref ADC1SEL_OP0     
  *                          - \ref ADC1SEL_OP1     
  *                          - \ref ADC1SEL_OP2     
  *                          - \ref ADC1SEL_ADC1_CH3
  *                          - \ref ADC1SEL_ADC1_CH4
  *                          - \ref ADC1SEL_ADC1_CH5
  *                          - \ref ADC1SEL_ADC1_CH6
  *                          - \ref ADC1SEL_ADC1_CH7
  * @return     None.
  * @details    This macro is used to set the ADC conversion mode to Simultaneous Sequential 4R Mode.
  */
#define ADC_SET_SIMULTANEOUS_4R_MODE(adc, u32ModuleNumA, u32ModuleNumB)    (   \
        (adc)->CTL = ((adc)->CTL & ~(ADC_CTL_ADCMODE_Msk | ADC_CTL_ADCSS3R_Msk | ADC_CTL_ADC0SEQSEL_Msk | ADC_CTL_ADC1SEQSEL_Msk)) \
                      | (0x3 << ADC_CTL_ADCMODE_Pos) | (u32ModuleNumA << ADC_CTL_ADC0SEQSEL_Pos) | ((unsigned long)(u32ModuleNumB - ADC_ADC1) << ADC_CTL_ADC1SEQSEL_Pos) )

/*---------------------------------------------------------------------------------------------------------*/
/* Define ADC functions prototype                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void ADC_Open(ADC_T *adc, uint32_t u32InputMode);
void ADC_Close(ADC_T *adc);
void ADC_ConfigSampleModule(ADC_T *adc, uint32_t u32ModuleNum, uint32_t u32TriggerSource, uint32_t u32Channel);
void ADC_EnableSecondaryTrigger(ADC_T *adc, uint32_t u32TriggerSrc);
void ADC_DisableSecondaryTrigger(ADC_T *adc);
void ADC_SetTriggerDelayTime(ADC_T *adc, uint32_t u32ModuleNum, uint32_t u32TriggerDelayTime, uint32_t u32DelayClockDivider);
void ADC_SetSampleCnt(ADC_T *adc, uint32_t u32ModuleNum, uint32_t u32SampleCnt);
void ADC_EnablePWMTrigger(ADC_T *adc, uint32_t u32ModuleNum, uint32_t u32Source, uint32_t u32Param);
void ADC_DisablePWMTrigger(ADC_T *adc, uint32_t u32ModuleNum, uint32_t u32Source, uint32_t u32Param);
void ADC_DisableAllPWMTrigger(ADC_T *adc, uint32_t u32ModuleNum);
void ADC_DisableWCMP(ADC_T *adc);
void ADC_EnableWCMP(ADC_T *adc, uint32_t u32HighBound, uint32_t u32LowBound, uint32_t u32FlagEN, uint32_t u32MatchCount, uint32_t u32FlagCTL);
void ADC_ResetWCMPCounter(ADC_T *adc);
uint32_t ADC_Get_Data_Valid_Flag(ADC_T *adc, uint32_t u32ModuleMask);
uint32_t ADC_Get_Sec_Data_Valid_Flag(ADC_T *adc, uint32_t u32ModuleMask);

/*@}*/ /* end of group NM1230_ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NM1230_ADC_Driver */

/*@}*/ /* end of group NM1230_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif  /* __ADC_H__ */

/*** (C) COPYRIGHT 2018 Nuvoton Technology Corp. ***/
