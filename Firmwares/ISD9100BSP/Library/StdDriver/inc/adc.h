/**************************************************************************//**
 * @file     adc.h
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 14/06/20 13:27p $
 * @brief    ISD9100 Series ADC Driver Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_ADC_Driver ADC Driver
  @{
*/

/** @addtogroup ISD9100_ADC_EXPORTED_CONSTANTS ADC Exported Constants
  @{
*/  

/*---------------------------------------------------------------------------------------------------------*/
/* ADC Over-sampling Ratio Of The Decimation Filter Constant Definitions                                   */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_OSR_RATION_64   0       /*!< ADC 64 ration over-sampling            */
#define ADC_OSR_RATION_128  1       /*!< ADC 128 ration over-sampling           */
#define ADC_OSR_RATION_192  2       /*!< ADC 192 ration over-sampling           */
#define ADC_OSR_RATION_384  3       /*!< ADC 384 ration over-sampling           */

/*---------------------------------------------------------------------------------------------------------*/
/* ADC And ALC Interrupt Flag Constant Definitions                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_FIFO_INT        (1)             /*!< ADC FIFO level interrupt enable flag   */
#define ADC_CMP0_INT        (2)             /*!< ADC compared 0 interrupt enable flag   */
#define ADC_CMP1_INT        (4)             /*!< ADC compared 1 interrupt enable flag   */
#define ADC_ALC_INT         (8)             /*!< ALC interrupt enable flag   */

/*---------------------------------------------------------------------------------------------------------*/
/* ADC ADCMPR Constant Definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_ADCMPR_CMPCOND_LESS_THAN          (0UL << 2)           /*!< The compare condition is "less than"          */
#define ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL   (1UL << 2)           /*!< The compare condition is "greater than or equal to" */
#define ADC_ADCMPR_CMPEN_DISABLE    (0UL << 0)            /*!< The compare function disable */
#define ADC_ADCMPR_CMPEN_ENABLE     (1UL << 0)            /*!< The compare function enable */

/*---------------------------------------------------------------------------------------------------------*/
/* VMID Constant Definitions                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_VMID_HIRES_CONNECT      (0x0ul << ANA_VMID_PDHIRES_Pos)     /*!< Connect the High Resistance reference to VMID */
#define ADC_VMID_HIRES_DISCONNECT   (0x1ul << ANA_VMID_PDHIRES_Pos)     /*!< The High Resistance reference is disconnected from VMID */
#define ADC_VMID_LORES_CONNECT      (0x0ul << ANA_VMID_PDLORES_Pos)     /*!< Connect the Low Resistance reference to VMID */
#define ADC_VMID_LORES_DISCONNECT   (0x1ul << ANA_VMID_PDLORES_Pos)     /*!< The Low Resistance reference is disconnected from VMID */

/*---------------------------------------------------------------------------------------------------------*/
/* MICBSEL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_MICBSEL_90_VCCA         ((0x0ul << ANA_MICBSEL_REFSEL_Pos)|0x0)   /*!< Microphone bias voltage is 90% VCCA  */
#define ADC_MICBSEL_65_VCCA         ((0x0ul << ANA_MICBSEL_REFSEL_Pos)|0x1)   /*!< Microphone bias voltage is 65% VCCA  */
#define ADC_MICBSEL_75_VCCA         ((0x0ul << ANA_MICBSEL_REFSEL_Pos)|0x2)   /*!< Microphone bias voltage is 75% VCCA  */
#define ADC_MICBSEL_50_VCCA         ((0x0ul << ANA_MICBSEL_REFSEL_Pos)|0x3)   /*!< Microphone bias voltage is 50% VCCA  */
#define ADC_MICBSEL_24V          ((0x1ul << ANA_MICBSEL_REFSEL_Pos)|0x0)   /*!< Microphone bias voltage is 2.4V VBG   */
#define ADC_MICBSEL_17V          ((0x1ul << ANA_MICBSEL_REFSEL_Pos)|0x1)   /*!< Microphone bias voltage is 1.7V VBG   */
#define ADC_MICBSEL_20V          ((0x1ul << ANA_MICBSEL_REFSEL_Pos)|0x2)   /*!< Microphone bias voltage is 2.0V VBG   */
#define ADC_MICBSEL_13V          ((0x1ul << ANA_MICBSEL_REFSEL_Pos)|0x3)   /*!< Microphone bias voltage is 1.3V VBG   */

/*---------------------------------------------------------------------------------------------------------*/
/* MUXCTL constant definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_MUXCTL_MIC_PATH          (ANA_MUXCTL_PGAINSEL_Msk)                           /*!< Select MICP/MICN To PGA Inputs   */
#define ADC_MUXCTL_TEMP_PATH         (ANA_MUXCTL_PTATCUR_Msk)                            /*!< Negative input to PGA, for temperature measurement   */
#define ADC_MUXCTL_GPIO_PATH         (ANA_MUXCTL_POSINSEL_Msk|ANA_MUXCTL_NEGINSEL_Msk)   /*!< GPIOB/A to PGA_INP/PGA_INN   */

#define ADC_MUXCTL_POSINSEL_NONE     (0x0ul << ANA_MUXCTL_POSINSEL_Pos)
#define ADC_MUXCTL_POSINSEL_GPB1     (0x1ul << ANA_MUXCTL_POSINSEL_Pos)           /*!< GPIOB[1] connected to PGA_INP   */
#define ADC_MUXCTL_POSINSEL_GPB3     (0x2ul << ANA_MUXCTL_POSINSEL_Pos)           /*!< GPIOB[3] connected to PGA_INP   */
#define ADC_MUXCTL_POSINSEL_GPB5     (0x4ul << ANA_MUXCTL_POSINSEL_Pos)           /*!< GPIOB[5] connected to PGA_INP   */
#define ADC_MUXCTL_POSINSEL_GPB7     (0x8ul << ANA_MUXCTL_POSINSEL_Pos)           /*!< GPIOB[7] connected to PGA_INP   */

#define ADC_MUXCTL_NEGINSEL_NONE     (0x00ul << ANA_MUXCTL_NEGINSEL_Pos)
#define ADC_MUXCTL_NEGINSEL_GPB0     (0x01ul << ANA_MUXCTL_NEGINSEL_Pos)           /*!< GPIOB[0] connected to PGA_INN   */
#define ADC_MUXCTL_NEGINSEL_GPB1     (0x02ul << ANA_MUXCTL_NEGINSEL_Pos)           /*!< GPIOB[1] connected to PGA_INN   */
#define ADC_MUXCTL_NEGINSEL_GPB2     (0x04ul << ANA_MUXCTL_NEGINSEL_Pos)           /*!< GPIOB[2] connected to PGA_INN   */
#define ADC_MUXCTL_NEGINSEL_GPB3     (0x08ul << ANA_MUXCTL_NEGINSEL_Pos)           /*!< GPIOB[3] connected to PGA_INN   */
#define ADC_MUXCTL_NEGINSEL_GPB4     (0x10ul << ANA_MUXCTL_NEGINSEL_Pos)           /*!< GPIOB[4] connected to PGA_INN   */
#define ADC_MUXCTL_NEGINSEL_GPB5     (0x20ul << ANA_MUXCTL_NEGINSEL_Pos)           /*!< GPIOB[5] connected to PGA_INN   */
#define ADC_MUXCTL_NEGINSEL_GPB6     (0x40ul << ANA_MUXCTL_NEGINSEL_Pos)           /*!< GPIOB[6] connected to PGA_INN   */
#define ADC_MUXCTL_NEGINSEL_GPB7     (0x80ul << ANA_MUXCTL_NEGINSEL_Pos)           /*!< GPIOB[7] connected to PGA_INN   */

#define ADC_GPIO_SINGLEEND_CH0_N     ((BIT0 << 12) | ADC_MUXCTL_NEGINSEL_GPB0 | ADC_MUXCTL_POSINSEL_NONE)        /*!< singel-end mode, GPIOB[0] connected to PGA_INN    */  
#define ADC_GPIO_SINGLEEND_CH1_P     ((BIT1 << 12) | ADC_MUXCTL_NEGINSEL_NONE | ADC_MUXCTL_POSINSEL_GPB1)        /*!< singel-end mode, GPIOB[1] connected to PGA_INP    */
#define ADC_GPIO_SINGLEEND_CH1_N     ((BIT1 << 12) | ADC_MUXCTL_NEGINSEL_GPB1 | ADC_MUXCTL_POSINSEL_NONE)        /*!< singel-end mode, GPIOB[1] connected to PGA_INN    */ 
#define ADC_GPIO_SINGLEEND_CH2_N     ((BIT2 << 12) | ADC_MUXCTL_NEGINSEL_GPB2 | ADC_MUXCTL_POSINSEL_NONE)        /*!< singel-end mode, GPIOB[2] connected to PGA_INN    */ 
#define ADC_GPIO_SINGLEEND_CH3_P     ((BIT3 << 12) | ADC_MUXCTL_NEGINSEL_NONE | ADC_MUXCTL_POSINSEL_GPB3)        /*!< singel-end mode, GPIOB[3] connected to PGA_INP    */
#define ADC_GPIO_SINGLEEND_CH3_N     ((BIT3 << 12) | ADC_MUXCTL_NEGINSEL_GPB3 | ADC_MUXCTL_POSINSEL_NONE)        /*!< singel-end mode, GPIOB[3] connected to PGA_INN    */ 
#define ADC_GPIO_SINGLEEND_CH4_N     ((BIT4 << 12) | ADC_MUXCTL_NEGINSEL_GPB4 | ADC_MUXCTL_POSINSEL_NONE)        /*!< singel-end mode, GPIOB[4] connected to PGA_INN    */ 
#define ADC_GPIO_SINGLEEND_CH5_P     ((BIT5 << 12) | ADC_MUXCTL_NEGINSEL_NONE | ADC_MUXCTL_POSINSEL_GPB5)        /*!< singel-end mode, GPIOB[5] connected to PGA_INP    */
#define ADC_GPIO_SINGLEEND_CH5_N     ((BIT5 << 12) | ADC_MUXCTL_NEGINSEL_GPB5 | ADC_MUXCTL_POSINSEL_NONE)        /*!< singel-end mode, GPIOB[5] connected to PGA_INN    */ 
#define ADC_GPIO_SINGLEEND_CH6_N     ((BIT6 << 12) | ADC_MUXCTL_NEGINSEL_GPB6 | ADC_MUXCTL_POSINSEL_NONE)        /*!< singel-end mode, GPIOB[6] connected to PGA_INN    */ 
#define ADC_GPIO_SINGLEEND_CH7_P     ((BIT7 << 12) | ADC_MUXCTL_NEGINSEL_NONE | ADC_MUXCTL_POSINSEL_GPB7)        /*!< singel-end mode, GPIOB[7] connected to PGA_INP    */
#define ADC_GPIO_SINGLEEND_CH7_N     ((BIT7 << 12) | ADC_MUXCTL_NEGINSEL_GPB7 | ADC_MUXCTL_POSINSEL_NONE)        /*!< singel-end mode, GPIOB[7] connected to PGA_INN    */ 
#define ADC_GPIO_DIFFERENTIAL_CH01   (((BIT0 | BIT1) << 12) | ADC_MUXCTL_NEGINSEL_GPB0 | ADC_MUXCTL_POSINSEL_GPB1)       /*!< differential mode, GPIOB[0] connected to PGA_INN and GPIOB[1] connected to PGA_INP  */ 
#define ADC_GPIO_DIFFERENTIAL_CH23   (((BIT2 | BIT3) << 12) | ADC_MUXCTL_NEGINSEL_GPB2 | ADC_MUXCTL_POSINSEL_GPB3)       /*!< differential mode, GPIOB[2] connected to PGA_INN and GPIOB[3] connected to PGA_INP  */  
#define ADC_GPIO_DIFFERENTIAL_CH45   (((BIT4 | BIT5) << 12) | ADC_MUXCTL_NEGINSEL_GPB4 | ADC_MUXCTL_POSINSEL_GPB5)       /*!< differential mode, GPIOB[4] connected to PGA_INN and GPIOB[5] connected to PGA_INP  */ 
#define ADC_GPIO_DIFFERENTIAL_CH67   (((BIT6 | BIT7) << 12) | ADC_MUXCTL_NEGINSEL_GPB6 | ADC_MUXCTL_POSINSEL_GPB7)       /*!< differential mode, GPIOB[6] connected to PGA_INN and GPIOB[7] connected to PGA_INP  */ 

/*---------------------------------------------------------------------------------------------------------*/
/* PGACTL constant definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/ 
#define ADC_PGACTL_BOSST_GAIN_0DB    (0x0ul << ANA_PGACTL_BSTGAIN_Pos)          /*!< Boost Stage Gain is 0dB   */
#define ADC_PGACTL_BOSST_GAIN_26DB   (0x1ul << ANA_PGACTL_BSTGAIN_Pos)          /*!< Boost Stage Gain is 26dB   */

#define ADC_PGACTL_REFSEL_VMID       (0x0ul << ANA_PGACTL_REFSEL_Pos)           /*!< Select VMID(VCCA/2) voltage as analog ground reference.   */
#define ADC_PGACTL_REFSEL_VBG        (0x1ul << ANA_PGACTL_REFSEL_Pos)           /*!<  Select Bandgap(1.2V) voltage as analog ground reference.   */

/*---------------------------------------------------------------------------------------------------------*/
/* SIGCTL constant definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_SIGCTL_MUTE_PGA      (ANA_SIGCTL_MUTEPGA_Msk)
#define ADC_SIGCTL_MUTE_IPBOOST  (ANA_SIGCTL_MUTEBST_Msk)
	
#define ADC_SIGCTL_ADCMOD_POWER      (ANA_SIGCTL_PUADCOP_Msk)      /*!<  Power for ADC £U£G modulator.   */
#define ADC_SIGCTL_IBGEN_POWER       (ANA_SIGCTL_PUCURB_Msk)       /*!<  Power for current bias generation.   */
#define ADC_SIGCTL_BUFADC_POWER      (ANA_SIGCTL_PUBUFADC_Msk)     /*!<  Power for ADC reference buffer.   */
#define ADC_SIGCTL_BUFPGA_POWER      (ANA_SIGCTL_PUBUFPGA_Msk)     /*!<  Power for PGA reference buffer.   */
#define ADC_SIGCTL_ZCD_POWER         (ANA_SIGCTL_PUZCDCMP_Msk)     /*!<  Power for zero cross detect comparator.   */

/*---------------------------------------------------------------------------------------------------------*/
/* ALC CTL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define ADC_ALCCTL_NORMAL_MODE     (0x0ul << ALC_CTL_MODESEL_Pos)  /*!<  ALC operates on normal mode.   */
#define ADC_ALCCTL_LIMITER_MODE    (0x1ul << ALC_CTL_MODESEL_Pos)  /*!<  ALC operates on limiter mode.   */

#define ADC_ALCCTL_ABS_PEAK       (0x0ul << ALC_CTL_PKSEL_Pos)   /*!<  use absolute peak value for ALC training.    */
#define ADC_ALCCTL_P2P_PEAK       (0x1ul << ALC_CTL_PKSEL_Pos)   /*!<  use peak-to-peak value for ALC training.   */

#define ADC_ALCCTL_FASTDEC_ON     (0x0ul << ALC_CTL_PKLIMEN_Pos)  /*!< enable fast decrement when signal exceeds 87.5% of full scale.    */
#define ADC_ALCCTL_FASTDEC_OFF    (0x1ul << ALC_CTL_PKLIMEN_Pos)  /*!< disable fast decrement when signal exceeds 87.5% of full scale.    */

#define ADC_ALCCTL_NGPEAK_ABS     (0x1ul << ALC_CTL_NGPKSEL_Pos)   /*!<  use absolute peak value for for noise gate threshold determination.    */
#define ADC_ALCCTL_NGPEAK_P2P     (0x0ul << ALC_CTL_NGPKSEL_Pos)   /*!<  use peak-to-peak value for for noise gate threshold determination.   */

#define ADC_ALCCTL_NGTH0     (0)   /*!<   Noise Gate Threshold 0 level.    */
#define ADC_ALCCTL_NGTH1     (1)   /*!<   Noise Gate Threshold 1 level.    */
#define ADC_ALCCTL_NGTH2     (2)   /*!<   Noise Gate Threshold 2 level.    */
#define ADC_ALCCTL_NGTH3     (3)   /*!<   Noise Gate Threshold 3 level.    */
#define ADC_ALCCTL_NGTH4     (4)   /*!<   Noise Gate Threshold 4 level.    */
#define ADC_ALCCTL_NGTH5     (5)   /*!<   Noise Gate Threshold 5 level.    */
#define ADC_ALCCTL_NGTH6     (6)   /*!<   Noise Gate Threshold 6 level.    */
#define ADC_ALCCTL_NGTH7     (7)   /*!<   Noise Gate Threshold 7 level.    */

/*@}*/ /* end of group ISD9100_ADC_EXPORTED_CONSTANTS */


/** @addtogroup ISD9100_ADC_EXPORTED_FUNCTIONS ADC Exported Functions
  @{
*/

/**
  * @brief     Set SDCLK divisor.
  * @param     adc Base address of ADC module.
  * @param     u8Divisor SDCLK divisore whcih must be greater than 2.
  * @return    None
  * @details   The clock division ration is between the incoming ADC_CLK (default HCLK)
  *            and the Delta-Sigma sampling clock of the ADC.
  */
#define ADC_SET_SDCLKDIV(adc, \
                         u8Divisor)   (adc->CLKDIV = (u8Divisor&0xff))
                         
/**
  * @brief     Set over sampling ratio.
  * @param     adc Base address of ADC module.
  * @param     u8Ration over sampling ratio.
  *            - \ref ADC_OSR_RATION_64 
  *            - \ref ADC_OSR_RATION_128
  *            - \ref ADC_OSR_RATION_192
  *            - \ref ADC_OSR_RATION_384
  * @return    None
  * @details   This macro determines the over-sampling ratio of the decimation filter.
  */
#define ADC_SET_OSRATION(adc, \
                         u8Ration)   (adc->DCICTL = (adc->DCICTL&(~ADC_DCICTL_OVSPLRAT_Msk))|(u8Ration&0xf))                         

/**
  * @brief     Set CIC filter additional gain.
  * @param     adc Base address of ADC module.
  * @param     u8Gain gain is value.
  * @return    None.
  * @details   This should normally remain default 0. It can be set to non-zero values to 
  *            provide additional digital gain from the decimation filter. An additional
  *            gain is applied to signal of GAIN/2.
  */
#define ADC_SET_CICGAIN(adc, \
                        u8Gain)   (adc->DCICTL = (adc->DCICTL&(~ADC_DCICTL_GAIN_Msk))|((u8Gain&0xf)<<ADC_DCICTL_GAIN_Pos))
                         
                        
/**
  * @brief     read data from the audio FIFO.
  * @param     adc Base address of ADC module.
  * @return    Signed 16 bits audio data.
  * @details   A read of this register will read data from the audio FIFO
  *            and increment the read pointer.
  */
#define ADC_GET_FIFODATA(adc)   (adc->DAT&0xffff)

/**
  * @brief     Set FIFO interrupt level.
  * @param     adc Base address of ADC module.
  * @param     u8Level is number of words present in ADC FIFO, total 8 word levels.
  * @return    None.
  * @details   Determines at what level the ADC FIFO will generate a servicing
  *            interrupt to the CPU. Interrupt will be generated when number of
  *            words present in ADC FIFO is greater than u8Level.
  */
#define ADC_SET_FIFOINTLEVEL(adc, \
                             u8Level)    (adc->INTCTL = (adc->INTCTL&(~ADC_INTCTL_FIFOINTLV_Msk))|(u8Level&0x7))

/**
  * @brief     Enable ADC PDMA receive channel.
  * @param     adc Base address of ADC module.
  * @return    None.
  * @details   ADC will request PDMA service when data is available. When PDMA transfer is enabled,
  *            the ADC interrupt must be disabled.
  */
#define ADC_ENABLE_PDMA(adc)     adc->PDMACTL |= ADC_PDMACTL_RXDMAEN_Msk; \
                                 adc->INTCTL &= (~ADC_INTCTL_INTEN_Msk)
                                  
/**
  * @brief     Disable ADC PDMA receive channel.
  * @param     adc Base address of ADC module.
  * @return    None.
  */
#define ADC_DISABLE_PDMA(adc)     (adc->PDMACTL &= (~ADC_PDMACTL_RXDMAEN_Msk))

/**
  * @brief     Configure the comparator 0 and enable it.
  * @param     adc Base address of ADC module.
  * @param     u32Condition Specifies the compare condition. Valid values are:
  *            - \ref ADC_ADCMPR_CMPCOND_LESS_THAN            :The compare condition is "less than the compare value".
  *            - \ref ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value.
  * @param     u32Data Specifies the compare value, valid value are between 0 ~ 0xFFFF.
  * @param     u32MatchCount Specifies the match count setting, valid values are between 1~16.
  * @return    None
  * @details   For example, ADC_ENABLE_CMP0(ADC, ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 10);
  *            Means ADC will assert comparator 0 flag if conversion result is greater or
  *            equal to 0x800 for 10 times continuously.
  */
#define ADC_ENABLE_CMP0(adc, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (adc->CMP0 = u32Condition | \
                                                    ((u32Data) << 16) | \
                                                    (((u32MatchCount) - 1) << 8) |\
                                                    ADC_ADCMPR_CMPEN_ENABLE)

/**
  * @brief     Disable comparator 0.
  * @param     adc Base address of ADC module.
  * @return    None
  * @details   Set CMPEN (CMP0[0]) to 0 to disable ADC controller to compare CMPDAT (CMP1[16:31]).
  */
#define ADC_DISABLE_CMP0(adc) (adc->CMP0 = ADC_ADCMPR_CMPEN_DISABLE)

/**
  * @brief     Configure the comparator 1 and enable it.
  * @param     adc Base address of ADC module.
  * @param     u32Condition Specifies the compare condition. Valid values are:
  *            - \ref ADC_ADCMPR_CMPCOND_LESS_THAN            :The compare condition is "less than the compare value".
  *            - \ref ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value.
  * @param     u32Data Specifies the compare value, valid value are between 0 ~ 0xFFFF.
  * @param     u32MatchCount Specifies the match count setting, valid values are between 1~16.
  * @return    None
  * @details   For example, ADC_ENABLE_CMP0(ADC, ADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 10);
  *            Means ADC will assert comparator 1 flag if conversion result is greater or
  *            equal to 0x800 for 10 times continuously.
  */
#define ADC_ENABLE_CMP1(adc, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (adc->CMP1 = u32Condition | \
                                                    ((u32Data) << 16) | \
                                                    (((u32MatchCount) - 1) << 8) |\
                                                    ADC_ADCMPR_CMPEN_ENABLE)

/**
  * @brief     Disable comparator 1.
  * @param     adc Base address of ADC module.
  * @return    None
  * @details   Set CMPEN (CMP1[0]) to 0 to disable ADC controller to compare CMPDAT (CMP1[16:31]).
  */
#define ADC_DISABLE_CMP1(adc) (adc->CMP1 = ADC_ADCMPR_CMPEN_DISABLE)

/**
  * @brief     Start the A/D conversion.
  * @param     adc Base address of ADC module.
  * @return    None
  * @details   CHEN (CHEN[0]) can be set to 1 for ADC conversion enabling. 
  */
#define ADC_START_CONV(adc) (adc->CHEN = ADC_CHEN_CHEN_Msk)

/**
  * @brief     Stop the A/D conversion.
  * @param     adc Base address of ADC module.
  * @return    None
  * @details   CHEN (CHEN[0]) can be set to 0 for ADC conversion disabling
  */
#define ADC_STOP_CONV(adc) (adc->CHEN = ~ADC_CHEN_CHEN_Msk)

/**
  * @brief      Enable VMID reference voltage
  * @param      adc Base address of ADC module
  * @param      Hires is High Resistance reference to VMID
  *             - \ref ADC_VMID_HIRES_CONNECT
  *             - \ref ADC_VMID_HIRES_DISCONNECT
  * @param      Lores is Low Resistance reference to VMID
  *             - \ref ADC_VMID_LORES_CONNECT
  *             - \ref ADC_VMID_LORES_DISCONNECT
  * @return     None
  * @details    The VMID needs to be enabled for operation before using the ADC, PGA or other analog blocks.
  */
#define ADC_ENABLE_VMID(adc, \
                        Hires, \
						Lores)  (ANA->VMID = (0x0|Hires|Lores))

/**
  * @brief      Disable VMID reference voltage
  * @param      adc Base address of ADC module
  * @return     None
  */
#define ADC_DISABLE_VMID(adc)	(ANA->VMID |= 0x7)

/**
  * @brief      Enable programmable gain amplifier (PGA) 
  * @param      adc The base address of ADC module
  * @param      u32REFSel is reference voltage for analog path
  *             - \ref ADC_PGACTL_REFSEL_VMID
	*             - \ref ADC_PGACTL_REFSEL_VBG
  * @param      u32BoostGain is boost stage gain setting
  *             - \ref ADC_PGACTL_BOSST_GAIN_0DB
	*             - \ref ADC_PGACTL_BOSST_GAIN_26DB
  * @return     None
  * @details    ISD9100 provides a Programmable Gain Amplifier (PGA) as the front-end
  *             to the ADC to allow the adjustment of signal path gain. It is used in conjunction
  *             with the ALC block to provide automatic level control of incoming audio signals.  
  */
#define ADC_ENABLE_PGA(adc, \
                       u32REFSel, \
                       u32BoostGain)  (ANA->PGACTL = (ANA->PGACTL&~(ANA_PGACTL_BSTGAIN_Msk|ANA_PGACTL_REFSEL_Msk))|(ANA_PGACTL_PUPGA_Msk|ANA_PGACTL_PUBOOST_Msk|u32REFSel|u32BoostGain))

/**
  * @brief      Disable programmable gain amplifier
  * @param      adc The base address of ADC module
  * @return     None
  */
#define ADC_DISABLE_PGA(adc)   (ANA->PGACTL = 0x0)

/**
  * @brief      Boost stage and PGA mute-on control
  * @param      adc The base address of ADC module
  * @param      u8PGAStage is mute stage
  *             - \ref ADC_SIGCTL_MUTE_PGA
  *             - \ref ADC_SIGCTL_MUTE_IPBOOST
  * @return     None
  */
#define ADC_MUTEON_PGA(adc, \
                       u8PGAStage)    (ANA->SIGCTL |= (u8PGAStage&(ANA_SIGCTL_MUTEPGA_Msk|ANA_SIGCTL_MUTEBST_Msk)))
/**
  * @brief      Boost stage and PGA mute-off control
  * @param      adc The base address of ADC module
  * @param      u8PGAStage is mute stage
  *             - \ref ADC_SIGCTL_MUTE_PGA
  *             - \ref ADC_SIGCTL_MUTE_IPBOOST
  * @return     None
  */
#define ADC_MUTEOFF_PGA(adc, \
                        u8PGAStage)    (ANA->SIGCTL &= ~(u8PGAStage&(ANA_SIGCTL_MUTEPGA_Msk|ANA_SIGCTL_MUTEBST_Msk))) 

/**
  * @brief      Signal path power up control
  * @param      adc The base address of ADC module
  * @param      u8SignalType is signal type
  *             - \ref ADC_SIGCTL_ADCMOD_POWER
  *             - \ref ADC_SIGCTL_IBGEN_POWER
  *             - \ref ADC_SIGCTL_BUFADC_POWER
  *             - \ref ADC_SIGCTL_BUFPGA_POWER
  *             - \ref ADC_SIGCTL_ZCD_POWER
  * @return     None
  */
#define ADC_ENABLE_SIGNALPOWER(adc, \
                               u8SignalType)    (ANA->SIGCTL = (ANA->SIGCTL&(~0x1F))|(u8SignalType&(0x1F)))

/**
  * @brief      Signal path power up control
  * @param      adc The base address of ADC module
  * @param      u8SignalType is signal type
  *             - \ref ADC_SIGCTL_ADCMOD_POWER
  *             - \ref ADC_SIGCTL_IBGEN_POWER
  *             - \ref ADC_SIGCTL_BUFADC_POWER
  *             - \ref ADC_SIGCTL_BUFPGA_POWER
  *             - \ref ADC_SIGCTL_ZCD_POWER
  * @return     None
  */
#define ADC_DISABLE_SIGNALPOWER(adc, \
                                u8SignalType)    (ANA->SIGCTL &= ~(u8SignalType&(0x1F)))                                                                                                            

/**
  * @brief      Enable ALC operation
  * @param      adc The base address of ADC module
  * @param      u32Mode is ALC operation mod
  *             - \ref ADC_ALCCTL_NORMAL_MODE
  *             - \ref ADC_ALCCTL_LIMITER_MODE
  * @param      u32PeakSel is ALC gain peak detector selection
  *             - \ref ADC_ALCCTL_ABS_PEAK   (default)
  *             - \ref ADC_ALCCTL_P2P_PEAK
  * @param      u32FastDec is fast decrement on/off selection if ALC operates on ADC_ALCCTL_LIMITER_MODE.
  *             - \ref ADC_ALCCTL_FASTDEC_ON (default)
  *             - \ref ADC_ALCCTL_FASTDEC_OFF  
  * @return     None
  */
#define ADC_ENABLE_ALC(adc, \
                       u32Mode, \
                       u32PeakSel, \
                       u32FastDec)  (ALC->CTL = (ALC->CTL &~(ALC_CTL_MODESEL_Msk|ALC_CTL_PKSEL_Msk|ALC_CTL_PKLIMEN_Msk))|(u32Mode|u32PeakSel|u32FastDec|ALC_CTL_ALCEN_Msk))

/**
  * @brief      Disable ALC operation
  * @param      adc The base address of ADC module
  * @return     None
  */
#define ADC_DISABLE_ALC(adc)       (ALC->CTL &= (~ALC_CTL_ALCEN_Msk))

/**
  * @brief      Enable noise gate operation
  * @param      adc The base address of ADC module
  * @param      u32PeakSel is ALC gain peak detector selection
  *             - \ref ADC_ALCCTL_NGPEAK_ABS   
  *             - \ref ADC_ALCCTL_NGPEAK_P2P   (default)
  * @return     None
  */
#define ADC_ENABLE_NOISEGATE(adc, \
                             u32PeakSel)  (ALC->CTL = ( ALC->CTL&~ALC_CTL_NGPKSEL_Msk)|(u32PeakSel|ALC_CTL_ALCEN_Msk|ADC_ALCCTL_NORMAL_MODE|ALC_CTL_NGEN_Msk))

/**
  * @brief      Disable noise gate operation
  * @param      adc The base address of ADC module
  * @return     None
  */
#define ADC_DISABLE_NOISEGATE(adc)       (ALC->CTL &= (~ALC_CTL_NGEN_Msk))                        

/**
  * @brief      Enable zero Crossing
  * @param      adc The base address of ADC module
  * @return     None
  */
#define ADC_ENABLE_ZEROCROSSING(adc)    (ALC->CTL |= (ALC_CTL_ZCEN_Msk))

/**
  * @brief      Enable zero Crossing
  * @param      adc The base address of ADC module
  * @return     None
  */
#define ADC_DISABLE_ZEROCROSSING(adc)    (ALC->CTL &= (~ALC_CTL_ZCEN_Msk))
  
/**
  * @brief      Set ALC attack time
  * @param      adc The base address of ADC module
  * @param      u8Step is range N = 0~10 Steps
  * @return     None
  * @details    Normal mode (500us~512ms):  Time = 500us * 2^N
  *             Limiter mode(125us~128ms):  Time = 125us * 2^N 
  */
#define ADC_SET_ALCATTACKTIME(adc, \
                              u8Step)    (ALC->CTL = (ALC->CTL&~ALC_CTL_ATKSEL_Msk)|((u8Step&0xf) << ALC_CTL_ATKSEL_Pos))

/**
  * @brief      Set ALC decay time
  * @param      adc The base address of ADC module
  * @param      u8Step is range N = 0~10 Steps
  * @return     None
  * @details    Normal mode (125us~128ms):  Time = 125us * 2^N 
  *             Limiter mode( 31us~ 32ms):  Time =  31us * 2^N
  */
#define ADC_SET_ALCDECAYTIME(adc, \
                             u8Step)    (ALC->CTL = (ALC->CTL&~ALC_CTL_DECAYSEL_Msk)|((u8Step&0xf) << ALC_CTL_DECAYSEL_Pos))

/**
  * @brief      Set ALC hold time
  * @param      adc The base address of ADC module
  * @param      u8Step is range N = 0~10 Steps
  * @return     None
  * @details    0ms~1sec:  Time = 0 + 2^N (ms) 
  */
#define ADC_SET_ALCHOLDTIME(adc, \
                            u8Step)    (ALC->CTL = (ALC->CTL&~ALC_CTL_HOLDTIME_Msk)|((u8Step&0xf) << ALC_CTL_HOLDTIME_Pos))

/**
  * @brief      Set ALC hold time
  * @param      adc The base address of ADC module
  * @param      u8Th is noise gate threshold levle.
  *             - \ref ADC_ALCCTL_NGTH0
  *             - \ref ADC_ALCCTL_NGTH1
  *             - \ref ADC_ALCCTL_NGTH2
  *             - \ref ADC_ALCCTL_NGTH3
  *             - \ref ADC_ALCCTL_NGTH4
  *             - \ref ADC_ALCCTL_NGTH5
  *             - \ref ADC_ALCCTL_NGTH6
  *             - \ref ADC_ALCCTL_NGTH7
  * @return     None
  * @details    Boost disabled:    Threshold aaa (-81+6*ADC_ALCCTL_NGTHX) dB 
  *             Boost enabled:     Threshold aaa (-87+6*ADC_ALCCTL_NGTHX) dB
  */
#define ADC_SET_NOISEGATE_TH(adc, \
                             u8Th)     (ALC->CTL = (ALC->CTL&~ALC_CTL_NGTHBST_Msk)|(u8Th))

/**
  * @brief      Get noise gate flag
  * @param      adc The base address of ADC module
  * @return     Asserted flag
  * @details    Asserted when signal level is detected to be below NGTH
  */
#define ADC_GET_NOISEFLAG(adc)    (ALC->STS&ALC_STS_NOISEF_Msk)

/**
  * @brief      Get clipping flag
  * @param      adc The base address of ADC module
  * @return     Asserted flag
  * @details    Asserted when signal level is detected to be above 87.5% of full scale
  */
#define ADC_GET_CLIPFLAG(adc)    (ALC->STS&ALC_STS_CLIPFLAG_Msk)

/**
  * @brief      Get peak-to-peak Value
  * @param      adc The base address of ADC module
  * @return     9 MSBs of measured peak-to-peak value
  */
#define ADC_GET_P2PVALUE(adc)    ((ALC->STS&ALC_STS_P2PVAL_Msk) >> ALC_STS_P2PVAL_Pos)

/**
  * @brief      Get peak Value
  * @param      adc The base address of ADC module
  * @return     9 MSBs of measured absolute peak value
  */
#define ADC_GET_PEAKVALUE(adc)    ((ALC->STS&ALC_STS_PEAKVAL_Msk) >> ALC_STS_PEAKVAL_Pos)


void ADC_Open(void);
void ADC_Close(void);
uint32_t ADC_GetSampleRate(void);
void ADC_EnableInt(uint32_t u32Mask);
void ADC_DisableInt(uint32_t u32Mask);
uint32_t ADC_GetIntFlag(uint32_t u32Mask);
void ADC_ClearIntFlag(uint32_t u32Mask);
void ADC_SetAMUX(uint32_t u32AMUXSel, uint32_t u32MUXPSel, uint32_t u32MUXNSel);
void ADC_SetGPIOChannel(uint32_t u32Mode);
void ADC_EnableMICBias(uint32_t u32BiasSel);
void ADC_DisableMICBias(void);
int32_t ADC_SetPGAGaindB(int32_t i32PGAGainIndB);
int32_t ADC_SetALCMaxGaindB(int32_t i32MaxGaindB);
int32_t ADC_SetALCMinGaindB(int32_t i32MinGaindB);
int32_t ADC_SetALCTargetLevel(int32_t i32TargetLevel);
int32_t ADC_GetPGAGaindB(void);

/*@}*/ /* end of group ISD9100_ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_ADC_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__ADC_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
