/**************************************************************************//**
 * @file     sdadc.h
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 16/08/22 13:27p $
 * @brief    I91200 Series SDADC Driver Header File
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __SDADC_H__
#define __SDADC_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_SDADC_Driver SDADC Driver
  @{
*/

/** @addtogroup I91200_SDADC_EXPORTED_CONSTANTS SDADC Exported Constants
  @{
*/  

/*---------------------------------------------------------------------------------------------------------*/
/* SDADC Over-sampling Ratio Of The Decimation Filter Constant Definitions                                 */
/*---------------------------------------------------------------------------------------------------------*/
#define SDADC_DS_RATION_0   0       /*!< SDADC reserved down sampling Ratio            */
#define SDADC_DS_RATION_16  1       /*!< SDADC x16 down sampling Ratio                 */
#define SDADC_DS_RATION_32  2       /*!< SDADC x32 down sampling Ratio                 */
#define SDADC_DS_RATION_64  3       /*!< SDADC x64 down sampling Ratio                 */

/*---------------------------------------------------------------------------------------------------------*/
/* SDADC And ALC Interrupt Flag Constant Definitions                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define SDADC_FIFO_INT        (BIT0)             /*!< SDADC FIFO level interrupt enable flag   */
#define SDADC_CMP0_INT        (BIT1)             /*!< SDADC compared 0 interrupt enable flag   */
#define SDADC_CMP1_INT        (BIT2)             /*!< SDADC compared 1 interrupt enable flag   */
#define SDADC_ALC_PLMT_INT    (BIT8)             /*!< ALC Peak limiting Interrupt              */
#define SDADC_ALC_NG_INT  	  (BIT9)             /*!< ALC Noise Gating Interrupt               */
#define SDADC_ALC_GINC_INT    (BIT10)            /*!< GAIN Increase Interrupt                  */
#define SDADC_ALC_GDEC_INT    (BIT11)            /*!< GAIN Decrease Interrupt                  */
#define SDADC_ALC_GMAX_INT    (BIT12)            /*!< GAIN More Than Maximum GAIN Interrupt    */
#define SDADC_ALC_GMIN_INT    (BIT13)            /*!< GAIN Less Than Minimum GAIN Interrupt    */

/*---------------------------------------------------------------------------------------------------------*/
/* FIFO Data Bits Selections                                                							   */
/*---------------------------------------------------------------------------------------------------------*/
#define SDADC_FIFODATA_32BITS        (0x00ul << SDADC_CTL_FIFOBITS_Pos)             /*!< SDADC FIFO data 32 bits   */
#define SDADC_FIFODATA_16BITS        (0x01ul << SDADC_CTL_FIFOBITS_Pos)             /*!< SDADC FIFO data 16 bits   */
#define SDADC_FIFODATA_8BITS         (0x02ul << SDADC_CTL_FIFOBITS_Pos)             /*!< SDADC FIFO data 8 bits   */
#define SDADC_FIFODATA_24BITS        (0x03ul << SDADC_CTL_FIFOBITS_Pos)             /*!< SDADC FIFO data 24 bits   */

/*---------------------------------------------------------------------------------------------------------*/
/* SDADC ADCMPR Constant Definitions                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define SDADC_CMP_LESS_THAN          (0x00ul << SDADC_CMPR_CMPCOND_Pos)           /*!< The compare condition is "less than"          */
#define SDADC_CMP_GREATER_OR_EQUAL   (0x01ul << SDADC_CMPR_CMPCOND_Pos)           /*!< The compare condition is "greater than or equal to" */

/*---------------------------------------------------------------------------------------------------------*/
/* MICBSEL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define SDADC_MICBSEL_1P5V         (0x00ul << ANA_MICBSEL_LVL_Pos)   /*!< Microphone bias voltage is 1.5V  */
#define SDADC_MICBSEL_1P8V         (0x01ul << ANA_MICBSEL_LVL_Pos)   /*!< Microphone bias voltage is 1.8V  */
#define SDADC_MICBSEL_1P95V        (0x02ul << ANA_MICBSEL_LVL_Pos)   /*!< Microphone bias voltage is 1.95V */
#define SDADC_MICBSEL_2P1V         (0x03ul << ANA_MICBSEL_LVL_Pos)   /*!< Microphone bias voltage is 2.1V  */
#define SDADC_MICBSEL_2P25V        (0x04ul << ANA_MICBSEL_LVL_Pos)   /*!< Microphone bias voltage is 2.25V */
#define SDADC_MICBSEL_2P4V         (0x05ul << ANA_MICBSEL_LVL_Pos)   /*!< Microphone bias voltage is 2.4V  */
#define SDADC_MICBSEL_2P55V        (0x06ul << ANA_MICBSEL_LVL_Pos)   /*!< Microphone bias voltage is 2.55V */
#define SDADC_MICBSEL_2P7V         (0x07ul << ANA_MICBSEL_LVL_Pos)   /*!< Microphone bias voltage is 2.7V  */

/*---------------------------------------------------------------------------------------------------------*/
/* PGACTL constant definitions                                                                             */
/*---------------------------------------------------------------------------------------------------------*/ 
#define SDADC_PGACTL_GAIN_0DB    ((0x0ul << SDADC_SDCHOP_PGA_HZMODE_Pos)|(0x0ul << SDADC_SDCHOP_PGA_GAIN_Pos))         /*!< The Gain is 0dB   */
#define SDADC_PGACTL_GAIN_6DB    ((0x1ul << SDADC_SDCHOP_PGA_HZMODE_Pos)|(0x0ul << SDADC_SDCHOP_PGA_GAIN_Pos))         /*!< The Gain is 6dB  */
#define SDADC_PGACTL_GAIN_12DB   ((0x1ul << SDADC_SDCHOP_PGA_HZMODE_Pos)|(0x1ul << SDADC_SDCHOP_PGA_GAIN_Pos))         /*!< The Gain is 12dB  */
#define SDADC_PGACTL_GAIN_6DB_12K	((0x0ul << SDADC_SDCHOP_PGA_HZMODE_Pos)|(0x1ul << SDADC_SDCHOP_PGA_GAIN_Pos))         /*!< The Gain is 6dB  */

/*---------------------------------------------------------------------------------------------------------*/
/* ALC CTL constant definitions                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define SDADC_ALCCTL_NORMAL_MODE     (0x0ul << ALC_CTL_MODESEL_Pos)  /*!<  ALC operates on normal mode.   */
#define SDADC_ALCCTL_LIMITER_MODE    (0x1ul << ALC_CTL_MODESEL_Pos)  /*!<  ALC operates on limiter mode.   */

#define SDADC_ALCCTL_ABS_PEAK       (0x0ul << ALC_CTL_PKSEL_Pos)   /*!<  use absolute peak value for ALC training.    */
#define SDADC_ALCCTL_P2P_PEAK       (0x1ul << ALC_CTL_PKSEL_Pos)   /*!<  use peak-to-peak value for ALC training.   */

#define SDADC_ALCCTL_FASTDEC_ON     (0x0ul << ALC_CTL_PKLIMEN_Pos)  /*!< enable fast decrement when signal exceeds 87.5% of full scale.    */
#define SDADC_ALCCTL_FASTDEC_OFF    (0x1ul << ALC_CTL_PKLIMEN_Pos)  /*!< disable fast decrement when signal exceeds 87.5% of full scale.    */

#define SDADC_ALCCTL_NGPEAK_ABS     (0x1ul << ALC_CTL_NGPKSEL_Pos)   /*!<  use absolute peak value for for noise gate threshold determination.    */
#define SDADC_ALCCTL_NGPEAK_P2P     (0x0ul << ALC_CTL_NGPKSEL_Pos)   /*!<  use peak-to-peak value for for noise gate threshold determination.   */

#define SDADC_ALCCTL_NGTH0     (0)   /*!<   Noise Gate Threshold 0 level: -39dB.    */
#define SDADC_ALCCTL_NGTH1     (1)   /*!<   Noise Gate Threshold 1 level: -45dB.    */
#define SDADC_ALCCTL_NGTH2     (2)   /*!<   Noise Gate Threshold 2 level: -51dB.    */
#define SDADC_ALCCTL_NGTH3     (3)   /*!<   Noise Gate Threshold 3 level: -57dB.    */
#define SDADC_ALCCTL_NGTH4     (4)   /*!<   Noise Gate Threshold 4 level: -63dB.    */
#define SDADC_ALCCTL_NGTH5     (5)   /*!<   Noise Gate Threshold 5 level: -69dB.    */
#define SDADC_ALCCTL_NGTH6     (6)   /*!<   Noise Gate Threshold 6 level: -75dB.    */
#define SDADC_ALCCTL_NGTH7     (7)   /*!<   Noise Gate Threshold 7 level: -81dB.    */

#define SDADC_ALCCTL_RANGEHIGH     (ALC_CTL_ALCRANGESEL_Msk)   /*!<   ALC target range -22.5 ~-1.5dB.    */
#define SDADC_ALCCTL_RANGELOW      (0)   					   /*!<   ALC target range -28.5~ -6dB.    */

/*@}*/ /* end of group I91200_SDADC_EXPORTED_CONSTANTS */


/** @addtogroup I91200_SDADC_EXPORTED_FUNCTIONS SDADC Exported Functions
  @{
*/

/**
  * @brief     Set SDCLK divisor.
  * @param     sdadc Base address of SDADC module.
  * @param     u8Divisor SDCLK divisore whcih must be greater than 2.
  * @return    None
  * @details   The clock division ratio is between the incoming SDADC_CLK (default HCLK)
  *            and the Delta-Sigma sampling clock of the SDADC.
  */
#define SDADC_SET_SDCLKDIV(sdadc, \
                         u8Divisor)   (sdadc->CLKDIV = (u8Divisor&0xff))
                         
/**
  * @brief     Set DSRATE for SDADC down sampling ratio.
  * @param     sdadc Base address of SDADC module.
  * @param     u8Ratio down sampling ratio.
  *            - \ref SDADC_DS_RATION_0 
  *            - \ref SDADC_DS_RATION_16
  *            - \ref SDADC_DS_RATION_32
  *            - \ref SDADC_DS_RATION_64
  * @return    None
  * @details   This macro only determines the down sampling ratio of SDADC. If BIQ filter is enabled,
  *			   real DSR = SDADC_CTL.DSRATE * BIQ_CTL.ADCWNSR
  */
#define SDADC_SET_DSRATIO(sdadc, \
                         u8Ratio)   (sdadc->CTL = (sdadc->CTL&~(SDADC_CTL_DSRATE_Msk|SDADC_CTL_RATESEL_Msk))|u8Ratio)

/**
  * @brief     read data from the audio FIFO.
  * @param     sdadc Base address of SDADC module.
  * @return    Signed 16 bits audio data.
  * @details   A read of this register will read data from the audio FIFO
  *            and increment the read pointer.
  */
#define SDADC_GET_FIFODATA(sdadc)   (sdadc->DAT&SDADC_DAT_RESULT_Msk)

/**
  * @brief     FIFO data bits selection.
  * @param     sdadc Base address of SDADC module.
  * @param     u8Bits data bits.
  *            - \ref SDADC_FIFODATA_32BITS 
  *            - \ref SDADC_FIFODATA_16BITS
  *            - \ref SDADC_FIFODATA_8BITS
  *            - \ref SDADC_FIFODATA_24BITS
  * @return    None.
  */
#define SDADC_SET_FIFODATABITS(sdadc, \
							  u8Bits)   (sdadc->CTL = (sdadc->CTL&(~SDADC_CTL_FIFOBITS_Msk))|u8Bits)  

/**
  * @brief     Set FIFO interrupt level.
  * @param     sdadc Base address of SDADC module.
  * @param     u8Level is number of words present in SDADC FIFO, total 8 word levels.
  * @return    None.
  * @details   Determines at what level the SDADC FIFO will generate a servicing
  *            interrupt to the CPU. Interrupt will be generated when number of
  *            words present in SDADC FIFO is greater than u8Level.
  */
#define SDADC_SET_FIFOINTLEVEL(sdadc, \
                             u8Level)    (sdadc->CTL = (sdadc->CTL&(~SDADC_CTL_FIFOTH_Msk))|((u8Level&0x7)<<SDADC_CTL_FIFOTH_Pos))

/**
  * @brief      Check SDADC FIFO full or not
  * @param[in]  sdadc The base address of SDADC module
  * @return     0 = FIFO is not full
  *             1 = FIFO is full
  */
#define SDADC_IS_FIFOFULL(sdadc)    (sdadc->FIFOSTS&SDADC_FIFOSTS_FULL_Msk)

/**
  * @brief      Check SDADC FIFO empty or not
  * @param[in]  sdadc The base address of SDADC module
  * @return     0 = FIFO is not empty
  *             1 = FIFO is empty
  */
#define SDADC_IS_FIFOEMPTY(sdadc)    (sdadc->FIFOSTS&SDADC_FIFOSTS_EMPTY_Msk)

/**
  * @brief     Enable SDADC PDMA receive channel.
  * @param     sdadc Base address of SDADC module.
  * @return    None.
  * @details   SDADC will request PDMA service when data is available. When PDMA transfer is enabled,
  *            the SDADC interrupt must be disabled.
  */
#define SDADC_ENABLE_PDMA(sdadc)    sdadc->PDMACTL |= SDADC_PDMACTL_PDMAEN_Msk; \
									sdadc->CTL &= (~SDADC_CTL_FIFOTHIE_Msk)
                                  
/**
  * @brief     Disable SDADC PDMA receive channel.
  * @param     sdadc Base address of SDADC module.
  * @return    None.
  */
#define SDADC_DISABLE_PDMA(sdadc)     (sdadc->PDMACTL &= (~SDADC_PDMACTL_PDMAEN_Msk))

/**
  * @brief     Configure the comparator 0 and enable it.
  * @param     sdadc Base address of SDADC module.
  * @param     u32Condition Specifies the compare condition. Valid values are:
  *            - \ref SDADC_CMP_LESS_THAN            :The compare condition is "less than the compare value".
  *            - \ref SDADC_CMP_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value.
  * @param     u32Data Specifies 23 bits value to compare FIFO data, the valid value are between 0 ~ 0x007fffff.
  * @param     u32MatchCount Specifies the match count setting, valid values are between 1~16.
  * @return    None
  * @details   For example, SDADC_ENABLE_CMP0(SDADC, SDADC_CMP_GREATER_OR_EQUAL, 0x800, 10);
  *            Means SDADC will assert comparator 0 flag if conversion result is greater or
  *            equal to 0x800 for 10 times continuously.
  */
#define SDADC_ENABLE_CMP0(sdadc, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (sdadc->CMPR[0] = u32Condition | \
														 ((u32Data&0x007fffff) << SDADC_CMPR_CMPD_Pos) | \
														 (((u32MatchCount) - 1) << SDADC_CMPR_CMPMATCNT_Pos) | \
														 SDADC_CMPR_CMPOEN_Msk)

/**
  * @brief     Disable comparator 0.
  * @param     sdadc Base address of SDADC module.
  * @return    None
  * @details   Set CMPEN (CMP0[0]) to 0 to disable SDADC controller to compare CMPD[30:8].
  */
#define SDADC_DISABLE_CMP0(sdadc) (sdadc->CMPR[0] = 0)

/**
  * @brief     Configure the comparator 1 and enable it.
  * @param     sdadc Base address of SDADC module.
  * @param     u32Condition Specifies the compare condition. Valid values are:
  *            - \ref SDADC_CMP_LESS_THAN            :The compare condition is "less than the compare value".
  *            - \ref SDADC_CMP_GREATER_OR_EQUAL     :The compare condition is "greater than or equal to the compare value.
  * @param     u32Data Specifies 23 bits value to compare FIFO data, the valid value are between 0 ~ 0x007fffff.
  * @param     u32MatchCount Specifies the match count setting, valid values are between 1~16.
  * @return    None
  * @details   For example, SDADC_ENABLE_CMP1(SDADC, SDADC_CMP_GREATER_OR_EQUAL, 0x800, 10);
  *            Means SDADC will assert comparator 1 flag if conversion result is greater or
  *            equal to 0x800 for 10 times continuously.
  */
#define SDADC_ENABLE_CMP1(sdadc, \
                        u32Condition, \
                        u32Data, \
                        u32MatchCount) (sdadc->CMPR[1] = u32Condition | \
														 ((u32Data&0x007fffff) << SDADC_CMPR_CMPD_Pos) | \
														 (((u32MatchCount) - 1) << SDADC_CMPR_CMPMATCNT_Pos) | \
														 SDADC_CMPR_CMPOEN_Msk)

/**
  * @brief     Disable comparator 1.
  * @param     sdadc Base address of SDADC module.
  * @return    None
  * @details   Set CMPEN (CMP1[0]) to 0 to disable SDADC controller to compare CMPD[30:8].
  */
#define SDADC_DISABLE_CMP1(sdadc) (sdadc->CMPR[1] = 0)

/**
  * @brief     Start the A/D conversion.
  * @param     sdadc Base address of SDADC module.
  * @return    None
  * @details   SDADCEN (SDADCEN[0]) can be set to 1 for SDADC conversion enabling. 
  */
#define SDADC_START_CONV(sdadc) (sdadc->EN = SDADC_EN_SDADCEN_Msk)

/**
  * @brief     Stop the A/D conversion.
  * @param     sdadc Base address of SDADC module.
  * @return    None
  * @details   SDADCEN (SDADCEN[0]) can be set to 0 for SDADC conversion disabling
  */
#define SDADC_STOP_CONV(sdadc) (sdadc->EN = ~SDADC_EN_SDADCEN_Msk)

/**
  * @brief      Enable programmable gain amplifier (PGA) 
  * @param      sdadc The base address of SDADC module
  * @param      u32Gain is boost stage gain setting
  *             - \ref SDADC_PGACTL_GAIN_0DB
  *             - \ref SDADC_PGACTL_GAIN_6DB
  *             - \ref SDADC_PGACTL_GAIN_12DB
  * @return     None
  * @details    I91200 provides a Programmable Gain Amplifier (PGA) as the front-end
  *             to the SDADC to allow the adjustment of signal path gain. 
  */
#define SDADC_ENABLE_PGA(sdadc, \
                         u32Gain)  (sdadc->SDCHOP = (sdadc->SDCHOP&~(SDADC_SDCHOP_PGA_GAIN_Msk|SDADC_SDCHOP_PGA_HZMODE_Msk))|(SDADC_SDCHOP_PGA_PU_Msk|u32Gain))

/**
  * @brief      Disable programmable gain amplifier
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_DISABLE_PGA(sdadc)   (sdadc->SDCHOP &= ~SDADC_SDCHOP_PGA_PU_Msk)

/**
  * @brief      PGA signal mute-on 
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_MUTEON_PGA(sdadc)    (sdadc->SDCHOP |= SDADC_SDCHOP_PGA_MUTE_Msk)

/**
  * @brief      PGA signal mute-off 
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_MUTEOFF_PGA(sdadc)   (sdadc->SDCHOP &= ~SDADC_SDCHOP_PGA_MUTE_Msk) 

/**
  * @brief      Enable digital MIC function and adc data is from GPIO
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_ENABLE_DMIC(sdadc)    (sdadc->CTL |= SDADC_CTL_DMICEN_Msk)

/**
  * @brief      Disable digital MIC function
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_DISABLE_DMIC(sdadc)   (sdadc->CTL &= ~SDADC_CTL_DMICEN_Msk)

/**
  * @brief      Enable SDADC analog block power
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_ANALOG_POWERON(sdadc)    (sdadc->SDCHOP &= ~SDADC_SDCHOP_PD_Msk)

/**
  * @brief      Disable SDADC analog block power
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_ANALOG_POWEROFF(sdadc)    (sdadc->SDCHOP |= SDADC_SDCHOP_PD_Msk)

/**
  * @brief      Enable ALC operation
  * @param      sdadc The base address of SDADC module
  * @param      u32Mode is ALC operation mod
  *             - \ref SDADC_ALCCTL_NORMAL_MODE
  *             - \ref SDADC_ALCCTL_LIMITER_MODE
  * @param      u32PeakSel is ALC gain peak detector selection
  *             - \ref SDADC_ALCCTL_ABS_PEAK   (default)
  *             - \ref SDADC_ALCCTL_P2P_PEAK
  * @param      u32FastDec is fast decrement on/off selection if ALC operates on SDADC_ALCCTL_LIMITER_MODE.
  *             - \ref SDADC_ALCCTL_FASTDEC_ON (default)
  *             - \ref SDADC_ALCCTL_FASTDEC_OFF  
  * @return     None
  */
#define SDADC_ENABLE_ALC(sdadc, \
                       u32Mode, \
                       u32PeakSel, \
                       u32FastDec)  (ALC->CTL = (ALC->CTL &~(ALC_CTL_MODESEL_Msk|ALC_CTL_PKSEL_Msk|ALC_CTL_PKLIMEN_Msk))|(u32Mode|u32PeakSel|u32FastDec|ALC_CTL_ALCEN_Msk))

/**
  * @brief      Disable ALC operation
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_DISABLE_ALC(sdadc)       (ALC->CTL &= (~ALC_CTL_ALCEN_Msk))

/**
  * @brief      Enable noise gate operation
  * @param      sdadc The base address of SDADC module
  * @param      u32PeakSel is ALC gain peak detector selection
  *             - \ref SDADC_ALCCTL_NGPEAK_ABS   
  *             - \ref SDADC_ALCCTL_NGPEAK_P2P   (default)
  * @return     None
  */
#define SDADC_ENABLE_NOISEGATE(sdadc, \
                             u32PeakSel)  (ALC->CTL = ( ALC->CTL&~ALC_CTL_NGPKSEL_Msk)|(u32PeakSel|ALC_CTL_ALCEN_Msk|SDADC_ALCCTL_NORMAL_MODE|ALC_CTL_NGEN_Msk))

/**
  * @brief      Disable noise gate operation
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_DISABLE_NOISEGATE(sdadc)       (ALC->CTL &= (~ALC_CTL_NGEN_Msk))                        

/**
  * @brief      Set ALC attack time
  * @param      sdadc The base address of SDADC module
  * @param      u8Step is range N = 0~10 Steps
  * @return     None
  * @details    Normal mode (125us~128ms):  Time = 125us * 2^N
  *             Limiter mode(31us~32ms):  Time = 31us * 2^N 
  */
#define SDADC_SET_ALCATTACKTIME(sdadc, \
                              u8Step)   (ALC->CTL = (ALC->CTL&~ALC_CTL_ATKSEL_Msk)|((u8Step&0xf) << ALC_CTL_ATKSEL_Pos))

/**
  * @brief      Set ALC decay time
  * @param      sdadc The base address of SDADC module
  * @param      u8Step is range N = 0~10 Steps
  * @return     None
  * @details    Normal mode (500us~512ms):  Time = 500us * 2^N 
  *             Limiter mode(125us~128ms):  Time = 128us * 2^N
  */
#define SDADC_SET_ALCDECAYTIME(sdadc, \
                             u8Step)    (ALC->CTL = (ALC->CTL&~ALC_CTL_DECAYSEL_Msk)|((u8Step&0xf) << ALC_CTL_DECAYSEL_Pos))

/**
  * @brief      Set ALC hold time
  * @param      sdadc The base address of SDADC module
  * @param      u8Step is range N = 0~10 Steps
  * @return     None
  * @details    0ms~1sec:  Time = 0 + 2^N (ms) 
  */
#define SDADC_SET_ALCHOLDTIME(sdadc, \
                            u8Step)     (ALC->CTL = (ALC->CTL&~ALC_CTL_HOLDTIME_Msk)|((u8Step&0xf) << ALC_CTL_HOLDTIME_Pos))

/**
  * @brief      Set ALC hold time
  * @param      sdadc The base address of SDADC module
  * @param      u8Th is noise gate threshold level.
  *             - \ref SDADC_ALCCTL_NGTH0
  *             - \ref SDADC_ALCCTL_NGTH1
  *             - \ref SDADC_ALCCTL_NGTH2
  *             - \ref SDADC_ALCCTL_NGTH3
  *             - \ref SDADC_ALCCTL_NGTH4
  *             - \ref SDADC_ALCCTL_NGTH5
  *             - \ref SDADC_ALCCTL_NGTH6
  *             - \ref SDADC_ALCCTL_NGTH7
  * @return     None
  * @details    threshold disabled:    Threshold aaa (-39-6*level) dB 
  */
#define SDADC_SET_NOISEGATE_TH(sdadc, \
                             u8Th)     (ALC->CTL = (ALC->CTL&~ALC_CTL_NGTHBST_Msk)|(u8Th))

/**
  * @brief      ALC Target range
  * @param      sdadc The base address of SDADC module
  * @param      u8range the range of ALC target level
  *             - \ref SDADC_ALCCTL_RANGEHIGH
  *             - \ref SDADC_ALCCTL_RANGELOW
  * @return     None
  */
#define SDADC_SET_ALCRANGE(sdadc, u8range)    (ALC->CTL = (ALC->CTL&~ALC_CTL_ALCRANGESEL_Msk)|(u8range))

/**
  * @brief      Get noise gate flag
  * @param      sdadc The base address of SDADC module
  * @return     Asserted flag
  * @details    Asserted when signal level is detected to be below NGTH
  */
#define SDADC_GET_NOISEFLAG(sdadc)    (ALC->STS&ALC_STS_NOISEF_Msk)

/**
  * @brief      Get clipping flag
  * @param      sdadc The base address of SDADC module
  * @return     Asserted flag
  * @details    Asserted when signal level is detected to be above 87.5% of full scale
  */
#define SDADC_GET_CLIPFLAG(sdadc)    (ALC->STS&ALC_STS_CLIPF_Msk)

/**
  * @brief      Get peak-to-peak Value
  * @param      sdadc The base address of SDADC module
  * @return     9 MSBs of measured peak-to-peak value
  */
#define SDADC_GET_P2PVALUE(sdadc)    ((ALC->STS&ALC_STS_P2PVAL_Msk) >> ALC_STS_P2PVAL_Pos)

/**
  * @brief      Get peak Value
  * @param      sdadc The base address of SDADC module
  * @return     9 MSBs of measured absolute peak value
  */
#define SDADC_GET_PEAKVALUE(sdadc)    ((ALC->STS&ALC_STS_PEAKVAL_Msk) >> ALC_STS_PEAKVAL_Pos)
	 
/**
  * @brief      Get current ADC gain
  * @param      sdadc The base address of SDADC module
  * @return     Current ADC gain setting
  */
#define SDADC_GET_ALCGAIN(sdadc)    ((ALC->STS&ALC_STS_ALCGAIN_Msk) >> ALC_STS_ALCGAIN_Pos)

/**
  * @brief      Set ALC Initial Gain
  * @param      sdadc The base address of SDADC module
  * @param      u8gain ALC initial gain
  * @return     None
  * @details    Selects the PGA gain setting from -12dB to 35.25dB in 0.75dB step size. 
  *				0x00 is lowest gain setting at -12dB and 0x3F is largest gain at 35.25dB
  */
#define SDADC_SET_ALC_INITGAIN(sdadc, \
                             u8gain)     (ALC->GAIN = (ALC->GAIN&~ALC_GAIN_INITGAINEN_Msk)|(u8gain & 0x3F))
							 
/**
  * @brief      Enable ALC Update Initial Gain
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_ENABLE_ALC_INITGAIN(sdadc)    (ALC->GAIN |= (ALC_GAIN_INITGAINEN_Msk))

/**
  * @brief      Disable ALC Update Initial Gain
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_DISABLE_ALC_INITGAIN(sdadc)    (ALC->GAIN &= (~ALC_GAIN_INITGAINEN_Msk))
			
/**
  * @brief      Enable zero Crossing
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_ENABLE_ZEROCROSSING(sdadc)    (ALC->GAIN |= (ALC_GAIN_ZCEN_Msk))

/**
  * @brief      Enable zero Crossing
  * @param      sdadc The base address of SDADC module
  * @return     None
  */
#define SDADC_DISABLE_ZEROCROSSING(sdadc)    (ALC->GAIN &= (~ALC_GAIN_ZCEN_Msk))

/**
  * @brief      Set ALC Peak Limiter Threshold
  * @param      sdadc The base address of SDADC module
  * @param      u32Th Peak Limiter Threshold
  * @return     None
  * @details    Full scale - 0x7fff
  *				Default value is 0x6fdc - 87.5% of full scale
  */
#define SDADC_SET_PEAKLIMIT_TH(sdadc, \
                             u32Th)     (ALC->GAIN = (ALC->GAIN&~ALC_GAIN_PKLIMIT_Msk)|((u32Th & 0x7FFF) << ALC_GAIN_PKLIMIT_Pos))
							 
void SDADC_Open(void);
void SDADC_Close(void);
uint32_t SDADC_SetAudioSampleRate(uint32_t u32SampleRate);
uint32_t SDADC_GetAudioSampleRate(void);
void SDADC_EnableInt(uint32_t u32Mask);
void SDADC_DisableInt(uint32_t u32Mask);
uint32_t SDADC_GetIntFlag(uint32_t u32Mask);
void SDADC_ClearIntFlag(uint32_t u32Mask);
void SDADC_EnableMICBias(uint32_t u32BiasSel);
void SDADC_DisableMICBias(void);
int32_t SDADC_SetALCMaxGaindB(int32_t i32MaxGaindB);
int32_t SDADC_SetALCMinGaindB(int32_t i32MinGaindB);
int32_t SDADC_SetALCTargetLevel(int32_t i32TargetLevel);

/*@}*/ /* end of group I91200_SDADC_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_SDADC_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SDADC_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
