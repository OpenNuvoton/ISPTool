/**************************************************************************//**
 * @file     adc.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 16/03/07 02:41p $
 * @brief    ISD9000 Series ADC Driver Header File
 *
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_ADC_Driver ADC Driver
  @{
*/


/** @addtogroup ISD9000_ADC_EXPORTED_CONSTANTS ADC Exported Constants
  @{
*/  

#define ADC_HP_FSEL0		(0x0UL << ADC_CTL_HP_FSEL_Pos)			/*!< no DC remove  \hideinitializer */
#define ADC_HP_FSEL1		(0x1UL << ADC_CTL_HP_FSEL_Pos)			/*!< DC suppress approx. -40dB, -3dB at 0.005 x Sampling Rate  \hideinitializer */
#define ADC_HP_FSEL2		(0x2UL << ADC_CTL_HP_FSEL_Pos)			/*!< DC suppress approx. -40dB, -3dB at 0.010 x Sampling Rate  \hideinitializer */
#define ADC_HP_FSEL3		(0x3UL << ADC_CTL_HP_FSEL_Pos)			/*!< DC suppress approx. -40dB, -3dB at 0.014 x Sampling Rate  \hideinitializer */
#define ADC_HP_FSEL4		(0x4UL << ADC_CTL_HP_FSEL_Pos)			/*!< DC suppress approx. -40dB, -3dB at 0.019 x Sampling Rate  \hideinitializer */
#define ADC_HP_FSEL5		(0x5UL << ADC_CTL_HP_FSEL_Pos)			/*!< DC suppress approx. -40dB, -3dB at 0.023 x Sampling Rate  \hideinitializer */
#define ADC_HP_FSEL6		(0x6UL << ADC_CTL_HP_FSEL_Pos)			/*!< DC suppress approx. -40dB, -3dB at 0.027 x Sampling Rate  \hideinitializer */
#define ADC_HP_FSEL7		(0x7UL << ADC_CTL_HP_FSEL_Pos)			/*!< DC suppress approx. -40dB, -3dB at 0.032 x Sampling Rate  \hideinitializer */

#define ADC_DS_RATE2		(0x0UL << ADC_CTL_DS_RATE_Pos)			/*!< ADC down sample X2  \hideinitializer */
#define ADC_DS_RATE4		(0x1UL << ADC_CTL_DS_RATE_Pos)			/*!< ADC down sample X2  \hideinitializer */
#define ADC_DS_RATE8		(0x2UL << ADC_CTL_DS_RATE_Pos)			/*!< ADC down sample X2  \hideinitializer */
#define ADC_DS_RATE16		(0x3UL << ADC_CTL_DS_RATE_Pos)			/*!< ADC down sample X2  \hideinitializer */
#define ADC_DS_ONECH        (0x1UL << ADC_CTL_DS_1CH_Pos)			/*!< ADC down sample one channel  \hideinitializer */
#define ADC_DS_TWOCH        (0x0UL << ADC_CTL_DS_1CH_Pos)			/*!< ADC down sample two channel  \hideinitializer */
#define ADC_CH_2_MASK                    (0x2UL)                       /*!< ADC single channel 2 mask  \hideinitializer */
#define ADC_CH_3_MASK                    (0x3UL)                       /*!< ADC single channel 3 mask  \hideinitializer */
#define ADC_CH_MIC_MASK                  (0xcUL)                       /*!< ADC differential for MIC  \hideinitializer */
#define ADC_NONECH_MASK                  (0xeUL)                       /*!< ADC none channel mask  \hideinitializer */
#define ADC_SCANEND_MASK                 (0xfUL)                       /*!< ADC scan sequence end mask  \hideinitializer */

#define ADC_CMP_LESS_THAN                (0x0UL << ADC_CMP_CMPCOND_Pos)   /*!< ADC compare condition less than  \hideinitializer */
#define ADC_CMP_GREATER_OR_EQUAL_TO      (ADC_CMP_CMPCOND_Msk)            /*!< ADC compare condition greater or equal to  \hideinitializer */

#define ADC_ADF_INT                      (ADC_STATUS_ADIF_Msk)           /*!< ADC convert complete interrupt \hideinitializer */
#define ADC_CMP0_INT                     (ADC_STATUS_ADCMPF0_Msk)        /*!< ADC comparator 0 interrupt  \hideinitializer */
#define ADC_CMP1_INT                     (ADC_STATUS_ADCMPF1_Msk)        /*!< ADC comparator 1 interrupt  \hideinitializer */

#define ADC_OPERATION_MODE_SINGLE        (0x0UL << ADC_CTL_OPMODE_Pos)      /*!< ADC operation mode set to single conversion  \hideinitializer */
#define ADC_OPERATION_MODE_SINGLE_CYCLE  (0x2UL << ADC_CTL_OPMODE_Pos)      /*!< ADC operation mode set to single cycle scan  \hideinitializer */
#define ADC_OPERATION_MODE_CONTINUOUS    (0x3UL << ADC_CTL_OPMODE_Pos)      /*!< ADC operation mode set to continuous scan  \hideinitializer */

#define ADC_OUT_FORMAT_UNSIGNED    (0x0UL << ADC_CTL_ADCFM_Pos)      /*!< ADC differential mode output format with unsigned  \hideinitializer */
#define ADC_OUT_FORMAT_2COMPLEMENT (ADC_CTL_ADCFM_Msk)               /*!< ADC differential mode output format with 2's complement  \hideinitializer */

#define ADC_MICBSEL_90_VCCA              (0<<ADC_PGCTL_MICB_VSEL_Pos)
#define ADC_MICBSEL_65_VCCA              (1<<ADC_PGCTL_MICB_VSEL_Pos)
#define ADC_MICBSEL_75_VCCA              (2<<ADC_PGCTL_MICB_VSEL_Pos)
#define ADC_MICBSEL_50_VCCA              (3<<ADC_PGCTL_MICB_VSEL_Pos)

#define ADC_VMID_REFERENCE_HIGH          (ADC_VMID_PDHIRES_Msk)
#define ADC_VMID_REFERENCE_LOW           (ADC_VMID_PDLOWRES_Msk)       

/*@}*/ /* end of group ISD9000_ADC_EXPORTED_CONSTANTS */


/** @addtogroup ISD9000_ADC_EXPORTED_FUNCTIONS ADC Exported Functions
  @{
*/
/**
  * @brief     Get the latest ADC conversion data
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum Channel number
  * @return    Latest ADC conversion data
  * \hideinitializer
  */
#define ADC_GET_CONVERSION_DATA(adc, u32ChNum) (adc->DAT[u32ChNum])

/**
  * @brief     Return the user-specified interrupt flags
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *            - \ref ADC_ADF_INT
  *            - \ref ADC_CMP0_INT
  *            - \ref ADC_CMP1_INT
  * @return    User specified interrupt flags
  * \hideinitializer
  */
#define ADC_GET_INT_FLAG(adc, u32Mask) (adc->STATUS & (u32Mask))

/**
  * @brief     This macro clear the selected interrupt status bits
  * @param[in] adc Base address of ADC module
  * @param[in] u32Mask The combination of following interrupt status bits. Each bit corresponds to a interrupt status.
  *            - \ref ADC_ADF_INT
  *            - \ref ADC_CMP0_INT
  *            - \ref ADC_CMP1_INT
  * @return    None
  * \hideinitializer
  */
#define ADC_CLR_INT_FLAG(adc, u32Mask) (adc->STATUS = (ADC->STATUS & ~(ADC_ADF_INT | \
                                                                       ADC_CMP0_INT | \
                                                                       ADC_CMP1_INT)) | (u32Mask))

/**
  * @brief     Get the busy state of ADC
  * @param[in] adc Base address of ADC module
  * @return    busy state of ADC
  * @retval    0 ADC is not busy
  * @retval    1 ADC is busy
  * \hideinitializer
  */
#define ADC_IS_BUSY(adc) (adc->STATUS & ADC_STATUS_BUSY_Msk ? 1 : 0)

/**
  * @brief     Check if the ADC conversion data is over written or not
  * @param[in] adc Base address of ADC module
  * @param[in] u8ScanSeq desire scan sequence
  * @return    Over run state of ADC data
  * @retval    0 ADC data is not overwritten
  * @retval    1 ADC data is overwritten
  * \hideinitializer
  */
#define ADC_IS_DATA_OVERRUN(adc, u8ScanSeq) ((adc->DAT[u8ScanSeq] & ADC_DAT_OV_Msk) ? 1 : 0)

/**
  * @brief     Check if the ADC conversion data is valid or not
  * @param[in] adc Base address of ADC module
  * @param[in] u8ScanSeq desire scan sequence
  * @return    Valid state of ADC data
  * @retval    0 ADC data is not valid
  * @retval    1 ADC data is valid
  * \hideinitializer
  */
#define ADC_IS_DATA_VALID(adc, u8ScanSeq) ((adc->DAT[u8ScanSeq] & ADC_DAT_VALID_Msk) ? 1 : 0)

/**
  * @brief     Power down ADC module
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_POWER_DOWN(adc) (adc->CTL &= ~ADC_CTL_ADCEN_Msk)

/**
  * @brief     Power on ADC module
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_POWER_ON(adc) (adc->CTL |= ADC_CTL_ADCEN_Msk)

/**
  * @brief     Configure the comparator 0 and enable it
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum  Specifies the source channel, valid value are from 2 or 3
  * @param[in] u32Condition Specifies the compare condition
  *            - \ref ADC_CMP_LESS_THAN
  *            - \ref ADC_CMP_GREATER_OR_EQUAL_TO
  * @param[in] u32Data Specifies the compare value. Valid value are between 0 ~ 0x1F
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16
  * @return    None
  * @details   For example, ADC_ENABLE_CMP0(ADC, 2, ADC_CMP_GREATER_OR_EQUAL_TO, 0xc0, 10);
  *            Means ADC will assert comparator 0 flag if channel 2 conversion result is
  *            greater or equal to 0xc for 10 times continuously.
  * \hideinitializer
  */
#define ADC_ENABLE_CMP0(adc, \
                        u32ChNum, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (adc->CMP[0] = ((u32ChNum) << ADC_CMP_CMPCH_Pos) | \
                                                                   (u32Condition) | \
                                                                   ((u32Data) << ADC_CMP_CMPDAT_Pos) | \
                                                                   (((u32MatchCount) - 1) << ADC_CMP_CMPMCNT_Pos) |\
                                                                   ADC_CMP_ADCMPEN_Msk)

/**
  * @brief     Disable comparator 0
  * @param[in] adc Base address of ADC module
  * \hideinitializer
  */
#define ADC_DISABLE_CMP0(adc) (adc->CMP[0] = 0)			

/**
  * @brief     Configure the comparator 1 and enable it
  * @param[in] adc Base address of ADC module
  * @param[in] u32ChNum  Specifies the source channel, valid value are 2 or 3
  * @param[in] u32Condition Specifies the compare condition
  *            - \ref ADC_CMP_LESS_THAN
  *            - \ref ADC_CMP_GREATER_OR_EQUAL_TO
  * @param[in] u32Data Specifies the compare value. Valid value are between 0 ~ 0x1F
  * @param[in] u32MatchCount Specifies the match count setting, valid values are between 1~16
  * @return    None
  * @details   For example, ADC_ENABLE_CMP1(ADC, 2, ADC_CMP_GREATER_OR_EQUAL_TO, 0xc, 10);
  *            Means ADC will assert comparator 1 flag if channel 2 conversion result is
  *            greater or equal to 0xc for 10 times continuously.
  * \hideinitializer
  */
#define ADC_ENABLE_CMP1(adc, \
                        u32ChNum, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (adc->CMP[1] = ((u32ChNum) << ADC_CMP_CMPCH_Pos) | \
                                                                   (u32Condition) | \
                                                                   ((u32Data) << ADC_CMP_CMPDAT_Pos) | \
                                                                   ((u32MatchCount - 1) << ADC_CMP_CMPMCNT_Pos) |\
                                                                   ADC_CMP_ADCMPEN_Msk)

/**
  * @brief     Disable comparator 1
  * @param[in] adc Base address of ADC module
  * \hideinitializer
  */
#define ADC_DISABLE_CMP1(adc) (adc->CMP[1] = 0)

/**
  * @brief     Set ADC sacn sequence. Enabled sequence will be converted while ADC starts.
  * @param[in] adc Base address of ADC module
  * @param[in] u8Seq0,u8Seq1,u8Seq2,u8Seq3,u8Seq4,u8Seq5,u8Seq6,u8Seq7 Specifies sequence scan which channel.
  *            - \ref ADC_CH_2_MASK
  *            - \ref ADC_CH_3_MASK
  *            - \ref ADC_CH_MIC_MASK
  *            - \ref ADC_NONECH_MASK
  *            - \ref ADC_SCANEND_MASK
  * @return    None
  * \hideinitializer
  */
#define ADC_SET_SCAN_SEQUENCE(adc, \
							  u8Seq0, \
							  u8Seq1, \
							  u8Seq2, \
							  u8Seq3, \
							  u8Seq4, \
							  u8Seq5, \
							  u8Seq6, \
							  u8Seq7)	(adc->CHSEQ = (u8Seq0 << ADC_CHSEQ_CHSEQ0_Pos| \
													   u8Seq1 << ADC_CHSEQ_CHSEQ1_Pos| \
													   u8Seq2 << ADC_CHSEQ_CHSEQ2_Pos| \
													   u8Seq3 << ADC_CHSEQ_CHSEQ3_Pos| \
													   u8Seq4 << ADC_CHSEQ_CHSEQ4_Pos| \
													   u8Seq5 << ADC_CHSEQ_CHSEQ5_Pos| \
													   u8Seq6 << ADC_CHSEQ_CHSEQ6_Pos| \
													   u8Seq7 << ADC_CHSEQ_CHSEQ7_Pos))

/**
  * @brief     Start the A/D conversion.
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_START_CONV(adc) (adc->CTL |= ADC_CTL_SWTRG_Msk)

/**
  * @brief     Stop the A/D conversion.
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_STOP_CONV(adc) (adc->CTL &= ~ADC_CTL_SWTRG_Msk)

/**
  * @brief     Set the output format in differential input mode.
  * @param[in] adc Base address of ADC module
  * @param[in] u32Format Differential input mode output format. Valid values are:
  *            - \ref ADC_OUT_FORMAT_UNSIGNED
  *            - \ref ADC_OUT_FORMAT_2COMPLEMENT
  * @return    None
  * \hideinitializer
  */
#define ADC_SET_OUTFORMAT(adc, u32Format) (adc->CTL = (adc->CTL & ~ADC_CTL_ADCFM_Msk) | u32Format)

/**
  * @brief     Set A/D converter operation mode.
  * @param[in] adc Base address of ADC module
  * @param[in] u8Mode operation mode
  *            - \ref ADC_OPERATION_MODE_SINGLE
  *            - \ref ADC_OPERATION_MODE_SINGLE_CYCLE
  *            - \ref ADC_OPERATION_MODE_CONTINUOUS
  * @return    None
  * \hideinitializer
  */
#define ADC_SET_OPERATION(adc, u8Mode)	(adc->CTL = (adc->CTL & ~ADC_CTL_OPMODE_Msk) | u8Mode)

/**
  * @brief     Enable high pass filter.
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_ENABLE_HPFILTER(adc)	(adc->CTL |= ADC_CTL_HP_EN_Msk)

/**
  * @brief     Disable high pass filter.
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_DISABLE_HPFILTER(adc)	(adc->CTL &= ~ADC_CTL_HP_EN_Msk)

/**
  * @brief     High pass filter frequency selection.
  * @param[in] adc Base address of ADC module
  * @param[in] u32Fsel frequency selection
  *            - \ref ADC_HP_FSEL0
  *            - \ref ADC_HP_FSEL1
  *            - \ref ADC_HP_FSEL2
  *            - \ref ADC_HP_FSEL3
  *            - \ref ADC_HP_FSEL4
  *            - \ref ADC_HP_FSEL5
  *            - \ref ADC_HP_FSEL6
  *            - \ref ADC_HP_FSEL7
  * @return    None
  * \hideinitializer
  */
#define ADC_SET_HPFILTER_FREQ(adc, u32Fsel)	(adc->CTL = (adc->CTL&~ADC_CTL_HP_FSEL_Msk) | u32Fsel)

/**
  * @brief     Enable down sample.
  * @param[in] adc Base address of ADC module
  * @param[in] u32rate frequency selection
  *            - \ref ADC_DS_RATE2
  *            - \ref ADC_DS_RATE4
  *            - \ref ADC_DS_RATE8
  *            - \ref ADC_DS_RATE16
  * @param[in] u32ChNum sets scan channel numbers and save in 8 buffers
  *            - \ref ADC_DS_ONECH
  *            - \ref ADC_DS_TWOCH
  * @return    None
  * \hideinitializer
  */
#define ADC_ENABLE_DOWNSAMPLE(adc, u32rate, u32ChNum)			adc->CTL |= ADC_CTL_DS_EN_Msk; \
																adc->CTL = (adc->CTL & (~(ADC_CTL_DS_RATE_Msk|ADC_CTL_DS_1CH_Msk)))|u32rate|u32ChNum

/**
  * @brief     Disable down sample.
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_DISABLE_DOWNSAMPLE(adc) 	(adc->CTL &= ~ADC_CTL_DS_EN_Msk)

/**
  * @brief     Mute on control of first stage pre-amp for offset bias calibration.
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_MUTEON_PGA(adc)		(adc->PGCTL |= ADC_PGCTL_OPMUTE_Msk)

/**
  * @brief     Mute off control of first stage pre-amp.
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_MUTEOFF_PGA(adc)		(adc->PGCTL &= ~ADC_PGCTL_OPMUTE_Msk)

/**
  * @brief     Select reference of ADC to VCCA
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_VREF_VCCA(adc)		(adc->PGCTL &= ~ADC_PGCTL_SAR_VREF_Msk)

/**
  * @brief     Select reference of ADC to MICBIAS
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_VREF_MICBIAS(adc)		(adc->PGCTL |= ADC_PGCTL_SAR_VREF_Msk)

/**
  * @brief     Power on analog bias generation
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_POWERON_ANALOGBIAS(adc)		(adc->PGCTL &= ~ADC_PGCTL_PD_IBEN_Msk)

/**
  * @brief     Power off analog bias generation
  * @param[in] adc Base address of ADC module
  * @return    None
  * \hideinitializer
  */
#define ADC_POWEROFF_ANALOGBIAS(adc)	(adc->PGCTL |= ADC_PGCTL_PD_IBEN_Msk)

/**
  * @brief      Enable programmable gain amplifier (PGA) 
  * @param      adc The base address of ADC module
  */
#define ADC_ENABLE_PGA(adc)  (adc->PGCTL |= ADC_PGCTL_EN_PGA_Msk)

/**
  * @brief      Disable programmable gain amplifier
  * @param      adc The base address of ADC module
  */
#define ADC_DISABLE_PGA(adc) (adc->PGCTL &= ~ADC_PGCTL_EN_PGA_Msk)

/**
  * @brief     Enable ADC PDMA receive channel.
  * @param     adc Base address of ADC module.
  * @return    None.
  * @details   ADC will request PDMA service when data is available. When PDMA transfer is enabled,
  *            the ADC interrupt must be disabled.
  */
#define ADC_ENABLE_PDMA(adc)     (adc->CTL |= ADC_CTL_PDMAEN_Msk)
                                  
/**
  * @brief     Disable ADC PDMA receive channel.
  * @param     adc Base address of ADC module.
  * @return    None.
  */
#define ADC_DISABLE_PDMA(adc)     (adc->CTL &= (~ADC_CTL_PDMAEN_Msk))

/**
  * @brief     Disable ADC PDMA receive channel.
  * @param     adc Base address of ADC module.
  * @return    None.
  */
#define ADC_SET_REFERENCE(adc, u32Refer)  (adc->VMID = (adc->VMID|(ADC_VMID_PDLOWRES_Msk|ADC_VMID_PDHIRES_Msk))&(~(u32Refer)))

void ADC_Open(ADC_T *adc);
void ADC_Close(ADC_T *adc);
void ADC_EnableInt(ADC_T *adc, uint32_t u32Mask);
void ADC_DisableInt(ADC_T *adc, uint32_t u32Mask);
int32_t ADC_SetPGAGaindB(ADC_T *adc, int32_t i32PGAGainIndB);
int32_t ADC_GetPGAGaindB(ADC_T *adc);
void ADC_EnableMICBias(ADC_T *adc, uint32_t u32BiasSel);
void ADC_DisableMICBias(ADC_T *adc);
void ADC_SetConversionCycle(ADC_T *adc, uint32_t u32Cycle);

/*@}*/ /* end of group ISD9000_ADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_ADC_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__ADC_H__

/*** (C) COPYRIGHT 2016 Nuvoton Technology Corp. ***/
