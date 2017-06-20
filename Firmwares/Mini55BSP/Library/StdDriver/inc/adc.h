/**************************************************************************//**
 * @file     adc.h
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/07/07 4:55p $
 * @brief    MINI55 series ADC driver header file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup MINI55_Device_Driver MINI55 Device Driver
  @{
*/

/** @addtogroup MINI55_ADC_Driver ADC Driver
  @{
*/

/** @addtogroup MINI55_ADC_EXPORTED_CONSTANTS ADC Exported Constants
  @{
*/

#define ADC_CH7_EXT                     (0UL)                       /*!< Use external input pin as ADC channel 7 source */
#define ADC_CH7_BGP                     (ADC_CHEN_CH7SEL_Msk)       /*!< Use internal band-gap voltage (VBG) as channel 7 source. */
#define ADC_CMP_LESS_THAN               (0UL)                       /*!< ADC compare condition less than */
#define ADC_CMP_GREATER_OR_EQUAL_TO     (ADC_CMP_CMPCOND_Msk)       /*!< ADC compare condition greater or equal to */
#define ADC_TRIGGER_BY_EXT_PIN          (0UL)                       /*!< ADC trigger by STADC (P3.2) pin */
#define ADC_TRIGGER_BY_PWM              (ADC_CTL_HWTRGSEL_Msk)      /*!< ADC trigger by PWM events */
#define ADC_FALLING_EDGE_TRIGGER        (0UL)                       /*!< External pin falling edge trigger ADC */
#define ADC_RISING_EDGE_TRIGGER         (ADC_CTL_HWTRGCOND_Msk)     /*!< External pin rising edge trigger ADC */
#define ADC_ADIF_INT                    (ADC_STATUS_ADIF_Msk)       /*!< ADC convert complete interrupt */
#define ADC_CMP0_INT                    (ADC_STATUS_ADCMPF0_Msk)    /*!< ADC comparator 0 interrupt */
#define ADC_CMP1_INT                    (ADC_STATUS_ADCMPF1_Msk)    /*!< ADC comparator 0 interrupt */
#define ADC_SAMPLE_CLOCK_0              (0UL)                       /*!< ADC sample time is 0 ADC clock */
#define ADC_SAMPLE_CLOCK_1              (1UL)                       /*!< ADC sample time is 1 ADC clock */
#define ADC_SAMPLE_CLOCK_2              (2UL)                       /*!< ADC sample time is 2 ADC clock */
#define ADC_SAMPLE_CLOCK_4              (3UL)                       /*!< ADC sample time is 4 ADC clock */
#define ADC_SAMPLE_CLOCK_8              (4UL)                       /*!< ADC sample time is 8 ADC clock */
#define ADC_SAMPLE_CLOCK_16             (5UL)                       /*!< ADC sample time is 16 ADC clock */
#define ADC_SAMPLE_CLOCK_32             (6UL)                       /*!< ADC sample time is 32 ADC clock */
#define ADC_SAMPLE_CLOCK_64             (7UL)                       /*!< ADC sample time is 64 ADC clock */
#define ADC_SAMPLE_CLOCK_128            (8UL)                       /*!< ADC sample time is 128 ADC clock */
#define ADC_SAMPLE_CLOCK_256            (9UL)                       /*!< ADC sample time is 256 ADC clock */
#define ADC_SAMPLE_CLOCK_512            (10UL)                      /*!< ADC sample time is 512 ADC clock */
#define ADC_SAMPLE_CLOCK_1024           (11UL)                      /*!< ADC sample time is 1024 ADC clock */
#define ADC_SEQMODE_TYPE_23SHUNT        (0UL)                       /*!< ADC sequential mode 23-shunt type */
#define ADC_SEQMODE_TYPE_1SHUNT         (1UL)                       /*!< ADC sequential mode 1-shunt type */
#define ADC_SEQMODE_MODESELECT_CH01     (0UL)                       /*!< ADC channel 0 then channel 1 conversion */
#define ADC_SEQMODE_MODESELECT_CH12     (1UL)                       /*!< ADC channel 1 then channel 2 conversion */
#define ADC_SEQMODE_MODESELECT_CH02     (2UL)                       /*!< ADC channel 0 then channel 2 conversion */
#define ADC_SEQMODE_TRISRC_PWM0           (0UL)                       /*!< ADC sequential mode PWM Trigger Source is PWM0*/
#define ADC_SEQMODE_TRISRC_PWM2           (1UL)                       /*!< ADC sequential mode PWM Trigger Source is PWM2*/
#define ADC_SEQMODE_TRISRC_PWM4         (2UL)                       /*!< ADC sequential mode PWM Trigger Source is PWM4*/
#define ADC_SEQMODE_PWM_RISING            (0UL)                       /*!< ADC sequential mode PWM rising trigger ADC*/
#define ADC_SEQMODE_PWM_CENTER            (1UL)                       /*!< ADC sequential mode PWM center trigger ADC*/
#define ADC_SEQMODE_PWM_FALLING         (2UL)                       /*!< ADC sequential mode PWM falling trigger ADC*/
#define ADC_SEQMODE_PWM_PERIOD          (3UL)                       /*!< ADC sequential mode PWM period trigger ADC*/

/*@}*/ /* end of group MINI55_ADC_EXPORTED_CONSTANTS */


/** @addtogroup MINI55_ADC_EXPORTED_FUNCTIONS ADC Exported Functions
  @{
*/


/**
  * @brief Configure the analog input source of channel 7
  * @param[in] adc Base address of ADC module
  * @param[in] u32Source Decides the analog input source of channel 7, valid values are
  *                     - \ref ADC_CH7_EXT
  *                     - \ref ADC_CH7_BGP
  * @return None
  * @note While using VBG as channel 7 source, ADC module clock must /b not exceed 300kHz
  * \hideinitializer
  */
#define ADC_CONFIG_CH7(adc, u32Source) (ADC->CHEN = (ADC->CHEN & ~ADC_CHEN_CH7SEL_Msk) | (u32Source))

/**
  * @brief Get the latest ADC conversion data
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum Currently not used
  * @return  Latest ADC conversion data
  * \hideinitializer
  */
#define ADC_GET_CONVERSION_DATA(adc, u32ChNum) (ADC->DAT & ADC_DAT_RESULT_Msk)

/**
  * @brief Return the user-specified interrupt flags
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *                     - \ref ADC_ADIF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return  User specified interrupt flags
  * \hideinitializer
  */
#define ADC_GET_INT_FLAG(adc, u32Mask) (ADC->STATUS & (u32Mask))

/**
  * @brief This macro clear the selected interrupt status bits
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *                     - \ref ADC_ADIF_INT
  *                     - \ref ADC_CMP0_INT
  *                     - \ref ADC_CMP1_INT
  * @return  None
  * \hideinitializer
  */
#define ADC_CLR_INT_FLAG(adc, u32Mask) (ADC->STATUS = (ADC->STATUS & ~(ADC_STATUS_ADIF_Msk | \
                                                                       ADC_STATUS_ADCMPF0_Msk | \
                                                                       ADC_STATUS_ADCMPF1_Msk)) | (u32Mask))

/**
  * @brief Get the busy state of ADC
  * @param[in] adc Base address of ADC module
  * @return busy state of ADC
  * @retval 0 ADC is not busy
  * @retval 1 ADC is busy
  * \hideinitializer
  */
#define ADC_IS_BUSY(adc) (ADC->STATUS & ADC_STATUS_BUSY_Msk ? 1 : 0)

/**
  * @brief Check if the ADC conversion data is over written or not
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum Currently not used
  * @return Over run state of ADC data
  * @retval 0 ADC data is not overrun
  * @retval 1 ADC data us overrun
  * \hideinitializer
  */
#define ADC_IS_DATA_OVERRUN(adc, u32ChNum) (ADC->STATUS & ADC_STATUS_OV_Msk ? 1 : 0)

/**
  * @brief Check if the ADC conversion data is valid or not
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum Currently not used
  * @return Valid state of ADC data
  * @retval 0 ADC data is not valid
  * @retval 1 ADC data us valid
  * \hideinitializer
  */
#define ADC_IS_DATA_VALID(adc, u32ChNum) (ADC->STATUS & ADC_STATUS_VALID_Msk ? 1 : 0)

/**
  * @brief Power down ADC module
  * @param[in] adc Base address of ADC module
  * @return None
  * \hideinitializer
  */
#define ADC_POWER_DOWN(adc) (ADC->CTL &= ~ADC_CTL_ADCEN_Msk)

/**
  * @brief Power on ADC module
  * @param[in] adc Base address of ADC module
  * @return None
  * \hideinitializer
  */
#define ADC_POWER_ON(adc) (ADC->CTL |= ADC_CTL_ADCEN_Msk)

/**
  * @brief Configure the comparator 0 and enable it
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum  Specifies the source channel, valid value are from 0 to 7
  * @param[in] u32Condition Specifies the compare condition
  *                     - \ref ADC_CMP_LESS_THAN
  *                     - \ref ADC_CMP_GREATER_OR_EQUAL_TO
  * @param[in] u32Data Specifies the compare value. Valid value are between 0 ~ 0x3FF
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16
  * @return None
  * @details For example, ADC_ENABLE_CMP0(ADC, 5, ADC_CMP_GREATER_OR_EQUAL_TO, 0x800, 10);
  *          Means ADC will assert comparator 0 flag if channel 5 conversion result is
  *          greater or equal to 0x800 for 10 times continuously.
  * \hideinitializer
  */
#define ADC_ENABLE_CMP0(adc, \
                        u32ChNum, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (ADC->CMP[0] = ((u32ChNum) << ADC_CMP_CMPCH_Pos) | \
                                                                   (u32Condition) | \
                                                                   ((u32Data) << ADC_CMP_CMPDAT_Pos) | \
                                                                   (((u32MatchCount) - 1) << ADC_CMP_CMPMCNT_Pos) |\
                                                                   ADC_CMP_ADCMPEN_Msk)

/**
  * @brief Disable comparator 0
  * @param[in] adc Base address of ADC module
  * \hideinitializer
  */
#define ADC_DISABLE_CMP0(adc) (ADC->CMP[0] = 0)

/**
  * @brief Configure the comparator 1 and enable it
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum  Specifies the source channel, valid value are from 0 to 7
  * @param[in] u32Condition Specifies the compare condition
  *                     - \ref ADC_CMP_LESS_THAN
  *                     - \ref ADC_CMP_GREATER_OR_EQUAL_TO
  * @param[in] u32Data Specifies the compare value. Valid value are between 0 ~ 0x3FF
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16
  * @return None
  * @details For example, ADC_ENABLE_CMP1(ADC, 5, ADC_CMP_GREATER_OR_EQUAL_TO, 0x800, 10);
  *          Means ADC will assert comparator 1 flag if channel 5 conversion result is
  *          greater or equal to 0x800 for 10 times continuously.
  * \hideinitializer
  */
#define ADC_ENABLE_CMP1(adc, \
                        u32ChNum, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (ADC->CMP[1] = ((u32ChNum) << ADC_CMP_CMPCH_Pos) | \
                                                                   (u32Condition) | \
                                                                   ((u32Data) << ADC_CMP_CMPDAT_Pos) | \
                                                                   ((u32MatchCount - 1) << ADC_CMP_CMPMCNT_Pos) |\
                                                                   ADC_CMP_ADCMPEN_Msk)

/**
  * @brief Disable comparator 1
  * @param[in] adc Base address of ADC module
  * \hideinitializer
  */
#define ADC_DISABLE_CMP1(adc) (ADC->CMP[1] = 0)

/**
  * @brief Set ADC input channel. Enabled channel will be converted while ADC starts.
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask  Channel enable bit. Each bit corresponds to a input channel. Bit 0 is channel 0, bit 1 is channel 1...
  * @return None
  * @note MINI55 series MCU ADC can only convert 1 channel at a time. If more than 1 channels are enabled, only channel
  *       with smallest number will be convert.
  * \hideinitializer
  */
#define ADC_SET_INPUT_CHANNEL(adc, u32Mask) (ADC->CHEN = (ADC->CHEN & ~ADC_CHEN_CHEN0_Msk) | (u32Mask))

/**
  * @brief Start the A/D conversion.
  * @param[in] adc Base address of ADC module
  * @return None
  * \hideinitializer
  */
#define ADC_START_CONV(adc) (ADC->CTL |= ADC_CTL_SWTRG_Msk)

/**
  * @brief Stop the A/D conversion.
  * @param[in] adc Base address of ADC module
  * @return None
  * \hideinitializer
  */
#define ADC_STOP_CONV(adc) (ADC->CTL &= ~ADC_CTL_SWTRG_Msk)

void ADC_Open(ADC_T *adc,
              uint32_t u32InputMode,
              uint32_t u32OpMode,
              uint32_t u32ChMask);
void ADC_Close(ADC_T *adc);
void ADC_EnableHWTrigger(ADC_T *adc,
                         uint32_t u32Source,
                         uint32_t u32Param);
void ADC_DisableHWTrigger(ADC_T *adc);
void ADC_SetExtraSampleTime(ADC_T *adc,
                            uint32_t u32ChNum,
                            uint32_t u32SampleTime);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);

void ADC_SeqModeEnable(ADC_T *adc, uint32_t u32SeqTYPE, uint32_t u32ModeSel);
void ADC_SeqModeTriggerSrc(ADC_T *adc, uint32_t u32SeqModeTrgSrc1, uint32_t u32Trg1Type, uint32_t u32SeqModeTrgSrc2, uint32_t u32Trg2Type);


/*@}*/ /* end of group MINI55_ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI55_ADC_Driver */

/*@}*/ /* end of group MINI55_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__ADC_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
