/**************************************************************************//**
 * @file     eadc.h
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/05/28 3:05p $
 * @brief    EADC Driver Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __EADC_H__
#define __EADC_H__


#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup EADC_Driver EADC Driver
  @{
*/

/** @addtogroup EADC_EXPORTED_CONSTANTS EADC Exported Constants
  @{
*/
/*---------------------------------------------------------------------------------------------------------*/
/* EADC Sample Module Constant Definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define EADC_SMPA0                  (0)    /*!< SAMPLE A0 */
#define EADC_SMPA1                  (1)    /*!< SAMPLE A1 */
#define EADC_SMPA2                  (2)    /*!< SAMPLE A2 */
#define EADC_SMPA3                  (3)    /*!< SAMPLE A3 */
#define EADC_SMPA4                  (4)    /*!< SAMPLE A4 */
#define EADC_SMPA5                  (5)    /*!< SAMPLE A5 */
#define EADC_SMPA6                  (6)    /*!< SAMPLE A6 */
#define EADC_SMPA7                  (7)    /*!< SAMPLE A7 */
#define EADC_SMPB0                  (8)    /*!< SAMPLE B0 */
#define EADC_SMPB1                  (9)    /*!< SAMPLE B1 */
#define EADC_SMPB2                  (10)   /*!< SAMPLE B2 */
#define EADC_SMPB3                  (11)   /*!< SAMPLE B3 */
#define EADC_SMPB4                  (12)   /*!< SAMPLE B4 */
#define EADC_SMPB5                  (13)   /*!< SAMPLE B5 */
#define EADC_SMPB6                  (14)   /*!< SAMPLE B6 */
#define EADC_SMPB7                  (15)   /*!< SAMPLE B7 */

/*---------------------------------------------------------------------------------------------------------*/
/* ADSPCRA/ADSPCRB Constant Definitions                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define EADC_ADSPCR_CHSEL(x)                ((x) << EADC_ADSPCR_CHSEL_Pos)       /*!< A/D sample module channel selection */
#define EADC_ADSPCR_TRGDLYDIV(x)            ((x) << EADC_ADSPCR_TRGDLYDIV_Pos)   /*!< A/D sample module start of conversion trigger delay clock divider selection */
#define EADC_ADSPCR_TRGDLYCNT(x)            ((x) << EADC_ADSPCR_TRGDLYCNT_Pos)   /*!< A/D sample module start of conversion trigger delay time */

#define EADC_SOFTWARE_TRIGGER               (0)                                                       /*!< Software trigger(Disable hardware trigger) */
#define EADC_FALLING_EDGE_TRIGGER           (EADC_ADSPCR_EXTFEN_Msk | (1UL<<EADC_ADSPCR_TRGSEL_Pos))  /*!< STADC pin falling edge trigger */
#define EADC_RISING_EDGE_TRIGGER            (EADC_ADSPCR_EXTREN_Msk | (1UL<<EADC_ADSPCR_TRGSEL_Pos))  /*!< STADC pin rising edge trigger */
#define EADC_FALLING_RISING_EDGE_TRIGGER    (EADC_ADSPCR_EXTFEN_Msk | EADC_ADSPCR_EXTREN_Msk | (1UL<<EADC_ADSPCR_TRGSEL_Pos)) /*!< STADC pin both falling and rising edge trigger */
#define EADC_ADINT0_TRIGGER                 (2UL<<EADC_ADSPCR_TRGSEL_Pos)       /*!< EADC ADINT0 interrupt EOC pulse trigger */
#define EADC_ADINT1_TRIGGER                 (3UL<<EADC_ADSPCR_TRGSEL_Pos)       /*!< EADC ADINT1 interrupt EOC pulse trigger */
#define EADC_TIMER0_TRIGGER                 (4UL<<EADC_ADSPCR_TRGSEL_Pos)       /*!< Timer0 overflow pulse trigger */
#define EADC_TIMER1_TRIGGER                 (5UL<<EADC_ADSPCR_TRGSEL_Pos)       /*!< Timer1 overflow pulse trigger */
#define EADC_TIMER2_TRIGGER                 (6UL<<EADC_ADSPCR_TRGSEL_Pos)       /*!< Timer2 overflow pulse trigger */
#define EADC_TIMER3_TRIGGER                 (7UL<<EADC_ADSPCR_TRGSEL_Pos)       /*!< Timer3 overflow pulse trigger */
#define EADC_PWM00_TRIGGER                  (8UL<<EADC_ADSPCR_TRGSEL_Pos)       /*!< PWM00 trigger */
#define EADC_PWM02_TRIGGER                  (9UL<<EADC_ADSPCR_TRGSEL_Pos)       /*!< PWM02 trigger */
#define EADC_PWM04_TRIGGER                  (0xAUL<<EADC_ADSPCR_TRGSEL_Pos)     /*!< PWM04 trigger */
#define EADC_PWM10_TRIGGER                  (0xBUL<<EADC_ADSPCR_TRGSEL_Pos)     /*!< PWM10 trigger */
#define EADC_PWM12_TRIGGER                  (0xCUL<<EADC_ADSPCR_TRGSEL_Pos)     /*!< PWM12 trigger */
#define EADC_PWM14_TRIGGER                  (0xDUL<<EADC_ADSPCR_TRGSEL_Pos)     /*!< PWM14 trigger */
#define EADC_PWM20_TRIGGER                  (0xEUL<<EADC_ADSPCR_TRGSEL_Pos)     /*!< PWM20 trigger */
#define EADC_PWM21_TRIGGER                  (0xFUL<<EADC_ADSPCR_TRGSEL_Pos)     /*!< PWM21 trigger */

#define EADC_ADSPCR_TRGDLYDIV_1             (0)                                 /*!< Trigger delay clock frequency is EADC_CLK/1 */
#define EADC_ADSPCR_TRGDLYDIV_2             (0x1UL<<EADC_ADSPCR_TRGDLYDIV_Pos)  /*!< Trigger delay clock frequency is EADC_CLK/2 */
#define EADC_ADSPCR_TRGDLYDIV_4             (0x2UL<<EADC_ADSPCR_TRGDLYDIV_Pos)  /*!< Trigger delay clock frequency is EADC_CLK/4 */
#define EADC_ADSPCR_TRGDLYDIV_16            (0x3UL<<EADC_ADSPCR_TRGDLYDIV_Pos)  /*!< Trigger delay clock frequency is EADC_CLK/16 */

/*---------------------------------------------------------------------------------------------------------*/
/* ADCMPR Constant Definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define EADC_ADCMPR_CMPCOND_LESS_THAN           (0UL<<EADC_ADCMPR_CMPCOND_Pos)   /*!< The compare condition is "less than" */
#define EADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL    (1UL<<EADC_ADCMPR_CMPCOND_Pos)   /*!< The compare condition is "greater than or equal to" */

#define EADC_CMPSMPL_A0                         (0)    /*!< SAMPLE A0 is selected to be compare */
#define EADC_CMPSMPL_A1                         (1)    /*!< SAMPLE A1 is selected to be compare */
#define EADC_CMPSMPL_A2                         (2)    /*!< SAMPLE A2 is selected to be compare */
#define EADC_CMPSMPL_A3                         (3)    /*!< SAMPLE A3 is selected to be compare */
#define EADC_CMPSMPL_B0                         (4)    /*!< SAMPLE B0 is selected to be compare */
#define EADC_CMPSMPL_B1                         (5)    /*!< SAMPLE B1 is selected to be compare */
#define EADC_CMPSMPL_B2                         (6)    /*!< SAMPLE B2 is selected to be compare */
#define EADC_CMPSMPL_B3                         (7)    /*!< SAMPLE B3 is selected to be compare */

/*---------------------------------------------------------------------------------------------------------*/
/* ADTCR Constant Definitions                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define EADC_ADTCR_SMPA                   (EADC_ADTCR_ADAEST_Pos)    /*!< Extend sampling time for all SAMPLE A */
#define EADC_ADTCR_SMPB                   (EADC_ADTCR_ADBEST_Pos)    /*!< Extend sampling time for all SAMPLE B */

/*---------------------------------------------------------------------------------------------------------*/
/* SMPTRGA/SMPTRGB Constant Definitions                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define EADC_SMPTRG_A0                    (0)    /*!< A/D trigger enable for SAMPLEA0 */
#define EADC_SMPTRG_A1                    (1)    /*!< A/D trigger enable for SAMPLEA1 */
#define EADC_SMPTRG_A2                    (2)    /*!< A/D trigger enable for SAMPLEA2 */
#define EADC_SMPTRG_A3                    (3)    /*!< A/D trigger enable for SAMPLEA3 */
#define EADC_SMPTRG_B0                    (4)    /*!< A/D trigger enable for SAMPLEB0 */
#define EADC_SMPTRG_B1                    (5)    /*!< A/D trigger enable for SAMPLEB1 */
#define EADC_SMPTRG_B2                    (6)    /*!< A/D trigger enable for SAMPLEB2 */
#define EADC_SMPTRG_B3                    (7)    /*!< A/D trigger enable for SAMPLEB3 */

#define EADC_SMPTRG_PWM00                 (EADC_SMPTRG_PWM00REN_Msk)     /*!< PWM00 trigger */
#define EADC_SMPTRG_PWM02                 (EADC_SMPTRG_PWM02REN_Msk)     /*!< PWM02 trigger */
#define EADC_SMPTRG_PWM04                 (EADC_SMPTRG_PWM04REN_Msk)     /*!< PWM04 trigger */
#define EADC_SMPTRG_PWM10                 (EADC_SMPTRG_PWM10REN_Msk)     /*!< PWM10 trigger */
#define EADC_SMPTRG_PWM12                 (EADC_SMPTRG_PWM12REN_Msk)     /*!< PWM12 trigger */
#define EADC_SMPTRG_PWM14                 (EADC_SMPTRG_PWM14REN_Msk)     /*!< PWM14 trigger */
#define EADC_SMPTRG_PWM20                 (EADC_SMPTRG_PWM20REN_Msk)     /*!< PWM20 trigger */
#define EADC_SMPTRG_PWM21                 (EADC_SMPTRG_PWM21REN_Msk)     /*!< PWM21 trigger */

#define EADC_TRGCOND_RISING_EDGE          (0)    /*!< PWM rising edge trigger enable  */
#define EADC_TRGCOND_FALLING_EDGE         (1)    /*!< PWM falling edge trigger enable */
#define EADC_TRGCOND_PERIOD               (2)    /*!< PWM period trigger enable */
#define EADC_TRGCOND_CENTER               (3)    /*!< PWM center trigger enable */

/*---------------------------------------------------------------------------------------------------------*/
/* Constant Definitions of EADC Channel 7 of Sample A Input Source                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define EADC_CH7_EXT_INPUT_SIGNAL         (0)  /*!< External input signal       */
#define EADC_CH7_INT_BANDGAP              (1)  /*!< Internal band-gap voltage   */
#define EADC_CH7_INT_TEMPERATURE_SENSOR   (2)  /*!< Internal temperature sensor */


/*@}*/ /* end of group EADC_EXPORTED_CONSTANTS */

/** @addtogroup EADC_EXPORTED_FUNCTIONS EADC Exported Functions
  @{
*/

/**
  * @brief A/D Converter Control Circuits Reset.
  * @param[in] eadc The pointer of the specified EADC module.
  * @return None
  * @details ADRESET bit (ADCR[1]) remains 1 during EADC reset, when EADC reset end, the ADRESET bit is automatically cleared to 0.
  *          The EADC control circuits will be reset to initial state, but the EADC register values will not be changed.
  */
#define EADC_CONV_RESET(eadc) ((eadc)->ADCR |= EADC_ADCR_ADRESET_Msk)

/**
  * @brief Enable double buffer mode.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_SMPA0
  *                          - \ref EADC_SMPA1
  *                          - \ref EADC_SMPA2
  *                          - \ref EADC_SMPA3
  *                          - \ref EADC_SMPB0
  *                          - \ref EADC_SMPB1
  *                          - \ref EADC_SMPB2
  *                          - \ref EADC_SMPB3
  * @return None.
  * @details The EADC controller supports a double buffer mode in sample module A/B 0~3.
  *         If user enable DBMAn(ADDBM[n], n=0~3) or DBMBn(ADDBM[m], m=8~11), the double buffer mode will enable.
  */
#define EADC_ENABLE_DOUBLE_BUFFER(eadc, u32ModuleNum) ((eadc)->ADDBM |= (1 << (u32ModuleNum)))

/**
  * @brief Disable double buffer mode.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_SMPA0
  *                          - \ref EADC_SMPA1
  *                          - \ref EADC_SMPA2
  *                          - \ref EADC_SMPA3
  *                          - \ref EADC_SMPB0
  *                          - \ref EADC_SMPB1
  *                          - \ref EADC_SMPB2
  *                          - \ref EADC_SMPB3
  * @return None.
  * @details Sample has one sample result register.
  */
#define EADC_DISABLE_DOUBLE_BUFFER(eadc, u32ModuleNum) ((eadc)->ADDBM &= ~(1 << (u32ModuleNum)))

/**
  * @brief Enable the interrupt.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32Mask Decides the combination of interrupt enable bits. Each bit corresponds to a interrupt.
  *                    This parameter decides which interrupts will be enabled. Bit 0 is ADIE0, bit 1 is ADIE1..., bit 3 is ADIE3.
  * @return None.
  * @details The A/D converter generates a conversion end ADFn (ADSR1[n]) upon the end of specific sample module A/D conversion.
  *         If ADIEn bit (ADCR[n+2]) is set, the conversion end interrupt request ADINTn is generated (n=0~3).
  */
#define EADC_ENABLE_INT(eadc, u32Mask) ((eadc)->ADCR |= ((u32Mask) << EADC_ADCR_ADIE0_Pos))

/**
  * @brief Disable the interrupt.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32Mask Decides the combination of interrupt enable bits. Each bit corresponds to a interrupt.
  *                    This parameter decides which interrupts will be disabled. Bit 0 is ADIE0, bit 1 is ADIE1..., bit 3 is ADIE3.
  * @return None.
  * @details Specific sample module A/D ADINTn(n=0~3) interrupt function Disabled.
  */
#define EADC_DISABLE_INT(eadc, u32Mask) ((eadc)->ADCR &= ~((u32Mask) << EADC_ADCR_ADIE0_Pos))

/**
  * @brief Enable the sample module interrupt.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32IntSel Decides which interrupt source will be used, valid value are from 0 to 3.
  * @param[in] u32ModuleMask the combination of sample module interrupt status bits. Each bit corresponds to a sample module interrupt status.
  *                          This parameter decides which sample module interrupts will be enabled, valid range are between 1~0xFFFF.
  * @return None.
  * @details There are 4 EADC interrupts ADINT0~3, and each of these interrupts has its own interrupt vector address.
  */
#define EADC_ENABLE_SAMPLE_MODULE_INT(eadc, u32IntSel, u32ModuleMask) ((eadc)->ADINTSRCTL[(u32IntSel)] |= (u32ModuleMask))

/**
  * @brief Disable the sample module interrupt.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32IntSel Decides which interrupt source will be used, valid value are from 0 to 3.
  * @param[in] u32ModuleMask the combination of sample module interrupt status bits. Each bit corresponds to a sample module interrupt status.
  *                          This parameter decides which sample module interrupts will be disabled, valid range are between 1~0xFFFF.
  *                          Bit 0 is sample module A0, bit 1 is sample module A1..., bit 15 is sample module B7.
  * @return None.
  * @details There are 4 EADC interrupts ADINT0~3, and each of these interrupts has its own interrupt vector address.
  */
#define EADC_DISABLE_SAMPLE_MODULE_INT(eadc, u32IntSel, u32ModuleMask) ((eadc)->ADINTSRCTL[(u32IntSel)] &= ~(u32ModuleMask))

/**
  * @brief Start the A/D conversion.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleMask The combination of sample module. Each bit corresponds to a sample module.
  *                          This parameter decides which sample module will be conversion, valid range are between 1~0xFFFF.
  *                          Bit 0 is sample module A0, bit 1 is sample module A1, ..., bit 15 is sample module B7.
  * @return None.
  * @details After write ADSSTR register to start EADC conversion, the ADSTPFR register will show which SAMPLE will conversion.
  */
#define EADC_START_CONV(eadc, u32ModuleMask) ((eadc)->ADSSTR = (u32ModuleMask))

/**
  * @brief Get the conversion pending flag.
  * @param[in] eadc The pointer of the specified EADC module.
  * @return Return the conversion pending sample module.
  * @details This STPFn(ADSTPFR[15:0]) bit remains 1 during pending state, when the respective EADC conversion is end,
  *          the STPFn (n=0~15) bit is automatically cleared to 0.
  */
#define EADC_GET_PENDING_CONV(eadc) ((eadc)->ADSTPFR)

/**
  * @brief Get the conversion data of the user-specified sample module.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_SMPA0
  *                          - \ref EADC_SMPA1
  *                          - \ref EADC_SMPA2
  *                          - \ref EADC_SMPA3
  *                          - \ref EADC_SMPA4
  *                          - \ref EADC_SMPA5
  *                          - \ref EADC_SMPA6
  *                          - \ref EADC_SMPA7
  *                          - \ref EADC_SMPB0
  *                          - \ref EADC_SMPB1
  *                          - \ref EADC_SMPB2
  *                          - \ref EADC_SMPB3
  *                          - \ref EADC_SMPB4
  *                          - \ref EADC_SMPB5
  *                          - \ref EADC_SMPB6
  *                          - \ref EADC_SMPB7
  * @return Return the conversion data of the user-specified sample module.
  * @details This macro is used to read RSLT bit (ADDRAn[11:0] and ADDRBn[11:0], n=0~7) field to get conversion data.
  */
#define EADC_GET_CONV_DATA(eadc, u32ModuleNum) (*(__IO uint32_t *) (&((eadc)->ADDRA[0]) + (u32ModuleNum)) & EADC_ADDR_RSLT_Msk)

/**
  * @brief Get the data overrun flag of the user-specified sample module.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleMask The combination of data overrun status bits. Each bit corresponds to a data overrun status, valid range are between 1~0xFFFF.
  * @return Return the data overrun flag of the user-specified sample module.
  * @details This macro is used to read OVERRUN bit (ADSR0[31:16]) field to get data overrun status.
  */
#define EADC_GET_DATA_OVERRUN_FLAG(eadc, u32ModuleMask) (((eadc)->ADSR0 >> EADC_ADSR0_OVERRUN_Pos) & (u32ModuleMask))

/**
  * @brief Get the data valid flag of the user-specified sample module.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleMask The combination of data valid status bits. Each bit corresponds to a data valid status, valid range are between 1~0xFFFF.
  * @return Return the data valid flag of the user-specified sample module.
  * @details This macro is used to read VALID bit (ADSR0[15:0]) field to get data valid flag.
  */
#define EADC_GET_DATA_VALID_FLAG(eadc, u32ModuleMask) (((eadc)->ADSR0 & EADC_ADSR0_VALID_Msk) & (u32ModuleMask))

/**
  * @brief Get the double data of the user-specified sample module.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Decides the sample module number, valid value are:
  *                          - \ref EADC_SMPA0
  *                          - \ref EADC_SMPA1
  *                          - \ref EADC_SMPA2
  *                          - \ref EADC_SMPA3
  *                          - \ref EADC_SMPB0
  *                          - \ref EADC_SMPB1
  *                          - \ref EADC_SMPB2
  *                          - \ref EADC_SMPB3
  * @return Return the double data of the user-specified sample module.
  * @details This macro is used to read RSLTDB bit (ADDRDBAn[11:0] and ADDRDBBn[11:0], n=0~3) field to get conversion data.
  */
#define EADC_GET_DOUBLE_DATA(eadc, u32ModuleNum) (*(__IO uint32_t *) (&((eadc)->ADDRDBA[0]) + ((u32ModuleNum) & 3) + ((u32ModuleNum) & 8)) & (EADC_ADDRDB_RSLTDB_Msk))

/**
  * @brief Get the user-specified interrupt flags.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32Mask The combination of interrupt status bits. Each bit corresponds to a interrupt status.
  *                    Bit 0 is ADF0, bit 1 is ADF1..., bit 3 is ADF3.
  *                    Bit 6 is ADCMPF0, bit 7 is ADCMPF1.
  * @return Return the user-specified interrupt flags.
  * @details This macro is used to get the user-specified interrupt flags.
  */
#define EADC_GET_INT_FLAG(eadc, u32Mask) ((eadc)->ADSR1 & (u32Mask))

/**
  * @brief Get the user-specified sample module overrun flags.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleMask The combination of sample module overrun status bits.
  *                          Each bit corresponds to a sample module overrun status, valid range are between 1~0xFFFF.
  * @return Return the user-specified sample module overrun flags.
  * @details This macro is used to get the user-specified sample module overrun flags.
  */
#define EADC_GET_SAMPLE_MODULE_OV_FLAG(eadc, u32ModuleMask) ((eadc)->ADSPOVFR & (u32ModuleMask))

/**
  * @brief Clear the selected interrupt status bits.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32Mask The combination of compare interrupt status bits. Each bit corresponds to a compare interrupt status.
  *                    Bit 0 is ADIF0, bit 1 is ADIF1..., bit 3 is ADIF3.
  *                    Bit 6 is ADCMPF0, bit 7 is ADCMPF1.
  * @return None.
  * @details This macro is used to clear clear the selected interrupt status bits.
  */
#define EADC_CLR_INT_FLAG(eadc, u32Mask) ((eadc)->ADSR1 = (u32Mask))

/**
  * @brief Clear the selected sample module overrun status bits.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleMask The combination of sample module overrun status bits.
  *                          Each bit corresponds to a sample module overrun status.
  *                          Bit 0 is SPOVF0, bit 1 is SPOVF1..., bit 15 is SPOVF15.
  * @return None.
  * @details This macro is used to clear the selected sample module overrun status bits.
  */
#define EADC_CLR_SAMPLE_MODULE_OV_FLAG(eadc, u32ModuleMask) ((eadc)->ADSPOVFR = (u32ModuleMask))

/**
  * @brief Check all sample module A/D result data register overrun flags.
  * @param[in] eadc The pointer of the specified EADC module.
  * @retval 0 None of sample module data register overrun flag is set to 1.
  * @retval 1 Any one of sample module data register overrun flag is set to 1.
  * @details The AOVERRUN bit (ADSR1[27]) will keep 1 when any one of sample module data register\n
  *          overrun flag OVERRUN (ADDRAn[16] or ADDRBn[16], n=0~7) is set to 1.
  */
#define EADC_IS_DATA_OV(eadc) (((eadc)->ADSR1 & EADC_ADSR1_AOVERRUN_Msk) >> EADC_ADSR1_AOVERRUN_Pos)

/**
  * @brief Check all sample module A/D result data register valid flags.
  * @param[in] eadc The pointer of the specified EADC module.
  * @retval 0 None of sample module data register valid flag is set to 1.
  * @retval 1 Any one of sample module data register valid flag is set to 1.
  * @details The AVALID bit (ADSR1[26]) will keep 1 when any one of sample module data register\n
  *          valid flag VALID (ADDRAn[17] or ADDRBn[17], n=0~7) is set to 1.
  */
#define EADC_IS_DATA_VALID(eadc) (((eadc)->ADSR1 & EADC_ADSR1_AVALID_Msk) >> EADC_ADSR1_AVALID_Pos)

/**
  * @brief Check all A/D sample module start of conversion overrun flags.
  * @param[in] eadc The pointer of the specified EADC module.
  * @retval 0 None of sample module event overrun flag is set to 1.
  * @retval 1 Any one of sample module event overrun flag is set to 1.
  * @details The ASPOVF bit (ADSR1[25]) will keep 1 when any one of sample module event overrun flag SPOVF (ADSPOVFR[n], n=0~15) is set to 1.
  */
#define EADC_IS_SAMPLE_MODULE_OV(eadc) (((eadc)->ADSR1 & EADC_ADSR1_ASPOVF_Msk) >> EADC_ADSR1_ASPOVF_Pos)

/**
  * @brief Check all A/D interrupt flag overrun bits.
  * @param[in] eadc The pointer of the specified EADC module.
  * @retval 0 None of ADINT interrupt flag is overwritten to 1.
  * @retval 1 Any one of ADINT interrupt flag is overwritten to 1.
  * @details The AADFOV bit (ADSR1[24]) will keep 1 when any one of ADINT interrupt flag ADFOV (ADIFOVR[3:0], n=0~3) is overwritten to 1.
  */
#define EADC_IS_INT_FLAG_OV(eadc) (((eadc)->ADSR1 & EADC_ADSR1_AADFOV_Msk) >> EADC_ADSR1_AADFOV_Pos)

/**
  * @brief Get the busy state of EADC.
  * @param[in] eadc The pointer of the specified EADC module.
  * @retval 0 Both A/D converter A and B are in idle state.
  * @retval 1 A/D converter A is in busy state.
  * @retval 2 A/D converter B is in busy state.
  * @retval 3 Both A/D converter A and B are in busy state.
  * @details This macro is used to read BUSYA/BUSYB bit (ADSR1[8]/ADSR1[16]) to get busy state.
  */
#define EADC_IS_BUSY(eadc) ((((eadc)->ADSR1 & EADC_ADSR1_BUSYA_Msk) >> EADC_ADSR1_BUSYA_Pos) | (((eadc)->ADSR1 & EADC_ADSR1_BUSYB_Msk) >> (EADC_ADSR1_BUSYB_Pos - 1)))

/**
  * @brief Configure the comparator 0 and enable it.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Specifies the compare sample module, valid value are:
  *                          - \ref EADC_CMPSMPL_A0
  *                          - \ref EADC_CMPSMPL_A1
  *                          - \ref EADC_CMPSMPL_A2
  *                          - \ref EADC_CMPSMPL_A3
  *                          - \ref EADC_CMPSMPL_B0
  *                          - \ref EADC_CMPSMPL_B1
  *                          - \ref EADC_CMPSMPL_B2
  *                          - \ref EADC_CMPSMPL_B3
  * @param[in] u32Condition Specifies the compare condition. Valid values are:
  *                          - \ref EADC_ADCMPR_CMPCOND_LESS_THAN            :The compare condition is "less than the compare value"
  *                          - \ref EADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value
  * @param[in] u16CMPData Specifies the compare value, valid range are between 0~0xFFF.
  * @param[in] u32MatchCount Specifies the match count setting, valid range are between 0~0xF.
  * @return None.
  * @details For example, EADC_ENABLE_CMP0(EADC, EADC_CMPSMPL_A1, EADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 10);
  *          Means EADC will assert comparator 0 flag if sample module A1 conversion result is greater or
  *          equal to 0x800 for 10 times continuously, and a compare interrupt request is generated.
  * \hideinitializer
  */
#define EADC_ENABLE_CMP0(eadc,\
                        u32ModuleNum,\
                        u32Condition,\
                        u16CMPData,\
                        u32MatchCount) ((eadc)->ADCMPR[0] |= (((u32ModuleNum) << EADC_ADCMPR_CMPSMPL_Pos)|\
                                                            (u32Condition) |\
                                                            ((u16CMPData) << EADC_ADCMPR_CMPD_Pos)| \
                                                            (((u32MatchCount) - 1) << EADC_ADCMPR_CMPMATCNT_Pos)|\
                                                            EADC_ADCMPR_ADCMP_EN_Msk))

/**
  * @brief Configure the comparator 1 and enable it.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleNum Specifies the compare sample module, valid value are:
  *                          - \ref EADC_CMPSMPL_A0
  *                          - \ref EADC_CMPSMPL_A1
  *                          - \ref EADC_CMPSMPL_A2
  *                          - \ref EADC_CMPSMPL_A3
  *                          - \ref EADC_CMPSMPL_B0
  *                          - \ref EADC_CMPSMPL_B1
  *                          - \ref EADC_CMPSMPL_B2
  *                          - \ref EADC_CMPSMPL_B3
  * @param[in] u32Condition Specifies the compare condition. Valid values are:
  *                          - \ref EADC_ADCMPR_CMPCOND_LESS_THAN            :The compare condition is "less than the compare value"
  *                          - \ref EADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value
  * @param[in] u16CMPData Specifies the compare value, valid range are between 0~0xFFF.
  * @param[in] u32MatchCount Specifies the match count setting, valid range are between 0~0xF.
  * @return None.
  * @details For example, EADC_ENABLE_CMP1(EADC, EADC_CMPSMPL_A1, EADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 10);
  *          Means EADC will assert comparator 0 flag if sample module A1 conversion result is greater or
  *          equal to 0x800 for 10 times continuously, and a compare interrupt request is generated.
  * \hideinitializer
  */
#define EADC_ENABLE_CMP1(eadc,\
                        u32ModuleNum,\
                        u32Condition,\
                        u16CMPData,\
                        u32MatchCount) ((eadc)->ADCMPR[1] |= (((u32ModuleNum) << EADC_ADCMPR_CMPSMPL_Pos)|\
                                                            (u32Condition) |\
                                                            ((u16CMPData) << EADC_ADCMPR_CMPD_Pos)| \
                                                            (((u32MatchCount) - 1) << EADC_ADCMPR_CMPMATCNT_Pos)|\
                                                            EADC_ADCMPR_ADCMP_EN_Msk))

/**
  * @brief Enable the compare interrupt.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32CMP Specifies the compare register, valid value are from 0 to 1.
  * @return None
  * @details If the compare function is enabled and the compare condition matches the setting of CMPCOND (ADCMPRn[2], n=0~1)
  *         and CMPMATCNT (ADCMPRn[11:8], n=0~1), ADCMPFn (ADSR1[7:6], n=0~1) will be asserted, in the meanwhile,
  *         if ADCMPIE is set to 1, a compare interrupt request is generated.
  */
#define EADC_ENABLE_CMP_INT(eadc, u32CMP) ((eadc)->ADCMPR[(u32CMP)] |= EADC_ADCMPR_ADCMPIE_Msk)

/**
  * @brief Disable the compare interrupt.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32CMP Specifies the compare register, valid value are from 0 to 1.
  * @return None.
  * @details This macro is used to disable the compare interrupt.
  */
#define EADC_DISABLE_CMP_INT(eadc, u32CMP) ((eadc)->ADCMPR[(u32CMP)] &= ~EADC_ADCMPR_ADCMPIE_Msk)

/**
  * @brief Disable comparator 0.
  * @param[in] eadc The pointer of the specified EADC module.
  * @return None.
  * @details This macro is used to disable comparator 0.
  */
#define EADC_DISABLE_CMP0(eadc) ((eadc)->ADCMPR[0] = 0)

/**
  * @brief Disable comparator 1.
  * @param[in] eadc The pointer of the specified EADC module.
  * @return None.
  * @details This macro is used to disable comparator 1.
  */
#define EADC_DISABLE_CMP1(eadc) ((eadc)->ADCMPR[1] = 0)

/**
  * @brief Configure the analog input source of channel 7 for sample A.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32Source Decides the analog input source of channel 7. Valid values are:
  *                       - \ref EADC_CH7_EXT_INPUT_SIGNAL        : External analog input.
  *                       - \ref EADC_CH7_INT_BANDGAP             : Internal bandgap voltage.
  *                       - \ref EADC_CH7_INT_TEMPERATURE_SENSOR  : Output of internal temperature sensor.
  * @return None.
  * @details Channel 7 of sample A supports 3 input sources: External analog voltage, internal Band-gap voltage, and internal temperature sensor output.
  * @note Channel 7 of sample B doesn't support this function.
  */
#define EADC_CONFIG_CH7(eadc, u32Source) ((eadc)->ADCHISELR = ((eadc)->ADCHISELR & ~EADC_ADCHISELR_PRESEL_Msk) | ((u32Source) << EADC_ADCHISELR_PRESEL_Pos))

/**
  * @brief Set the simultaneous sampling mode selection.
  * @param[in] eadc The pointer of the specified EADC module.
  * @param[in] u32ModuleMask The combination of sample module enable bits.
  *                          This parameter decides which sample module will be enabled, valid range are between 1~0xFF.
  * @return None.
  * @details This macro is used to set the user-specified sample module simultaneous sampling mode.
  */
#define EADC_SET_SIMULTANEOUS_MODE(eadc, u32ModuleMask) ((eadc)->ADSMSELR = (u32ModuleMask))

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

/*@}*/ /* end of group EADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group EADC_Driver */

/*@}*/ /* end of group Standard_Driver */

#ifdef __cplusplus
}
#endif

#endif //__EADC_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
