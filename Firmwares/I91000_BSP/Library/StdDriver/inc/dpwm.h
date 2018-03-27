/**************************************************************************//**
 * @file     DPWM.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/11/02 07:27p $
 * @brief    ISD9000 Series DPWM Driver Header File
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DPWM_H__
#define __DPWM_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9000_Device_Driver ISD9000 Device Driver
  @{
*/

/** @addtogroup ISD9000_DPWM_Driver DPWM Driver
  @{
*/

/** @addtogroup ISD9000_DPWM_EXPORTED_CONSTANTS DPWM Exported Constants
  @{
*/ 

/*---------------------------------------------------------------------------------------------------------*/
/* DPWM CTL Constant Definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define DPWM_CTL_MODUFRQ0       (0)       /*!< carrier modulation frequency0    */
#define DPWM_CTL_MODUFRQ1       (1)       /*!< carrier modulation frequency1    */
#define DPWM_CTL_MODUFRQ2       (2)       /*!< carrier modulation frequency2    */
#define DPWM_CTL_MODUFRQ3       (3)       /*!< carrier modulation frequency3    */
#define DPWM_CTL_MODUFRQ4       (4)       /*!< carrier modulation frequency4    */
#define DPWM_CTL_MODUFRQ5       (5)       /*!< carrier modulation frequency5    */
#define DPWM_CTL_MODUFRQ6       (6)       /*!< carrier modulation frequency6    */
#define DPWM_CTL_MODUFRQ7       (7)       /*!< carrier modulation frequency7    */

#define DPWM_CTL_DITHER_NONE    (0x0ul << DPWM_CTL_DITHEREN_Pos)       /*!< DPWM no dither bit    */
#define DPWM_CTL_DITHER_ONE     (0x1ul << DPWM_CTL_DITHEREN_Pos)       /*!< DPWM +/- 1 dither bit    */
#define DPWM_CTL_DITHER_TWO     (0x3ul << DPWM_CTL_DITHEREN_Pos)       /*!< DPWM +/- 2 dither bit    */

#define DPWM_CTL_DAC_13BIT      (0x0ul << DPWM_CTL_DAC_EN_10BIT_Pos)   /*!< DPWM DAC enable 13Bit mode */
#define DPWM_CTL_DAC_10BIT      (0x1ul << DPWM_CTL_DAC_EN_10BIT_Pos)   /*!< DPWM DAC enable 10Bit mode */


/*@}*/ /* end of group ISD9000_DPWM_EXPORTED_CONSTANTS */

/** @addtogroup ISD9000_DPWM_EXPORTED_FUNCTIONS DPWM Exported Functions
  @{
*/

/**
  * @brief      Enable DAC function
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    DAC function enable.
  */
#define DPWM_ENABLE_DAC(dpwm,u8BitMode)    (dpwm->CTL = ((dpwm->CTL&(~DPWM_CTL_DAC_EN_10BIT_Msk))|u8BitMode)|DPWM_CTL_DAC_EN_Msk)

/**
  * @brief      Disable DAC function
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    DAC function disable.
  */
#define DPWM_DISABLE_DAC(dpwm)    (dpwm->CTL &= (~DPWM_CTL_DAC_EN_Msk))

/**
  * @brief      Enable DPWM output data cross zero point.
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Output data cross zero point.
  */
#define DPWM_ENABLE_OUTPUTZEROCROSS(dpwm)    (dpwm->CTL |= DPWM_CTL_ZCIE_Msk)

/**
  * @brief      Disable DPWM output data cross zero point.
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Output data doesn? cross zero point.
  */
#define DPWM_DISABLE_OUTPUTZEROCROSS(dpwm)    (dpwm->CTL &= (~DPWM_CTL_ZCIE_Msk))

/**
  * @brief      Enable DPWM chipped data in DATA[31:0]
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Enabling CHIPIE will get clipped data in DATA[31:0] register.
  *             (But clipped with the range : 0x00 ~ 7fff  ~ 0x ffff ~8000. 
  *              The content of DATA[31:0] will not be change.)
  */
#define DPWM_ENABLE_CLIPIE(dpwm)    (dpwm->CTL |= DPWM_CTL_CLIPIE_Msk)

/**
  * @brief      Disable DPWM chipped data in DATA[31:0]
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Disabling CHIPIE will not get clipped data in DATA[31:0] register.
  */
#define DPWM_DISABLE_CLIPIE(dpwm)    (dpwm->CTL &= (~DPWM_CTL_CLIPIE_Msk))

/**
  * @brief      Enable DPWM FIFO threshold interrupt.
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    DPWM FIFO threshold interrupt Enabled.
  */
#define DPWM_ENABLE_FIFOTHRESHOLDINT(dpwm,u8Value)    (dpwm->CTL = (((dpwm->CTL&(~DPWM_CTL_RXTH_Msk))|((u8Value<<DPWM_CTL_RXTH_Pos)&DPWM_CTL_RXTH_Msk))|DPWM_CTL_RXTHIE_Msk)

/**
  * @brief      Disable DPWM FIFO threshold interrupt.
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    DPWM FIFO threshold interrupt Disabled.			
  */
#define DPWM_DISABLE_FIFOTHRESHOLDINT(dpwm)    (dpwm->CTL &= (~DPWM_CTL_RXTHIE_Msk))

/**
  * @brief      Set DPWM dither type
  * @param[in]  dpwm The base address of DPWM module
  * @param[in]  u8Type is dither type
  *             - \ref DPWM_CTL_DITHER_NONE
  *             - \ref DPWM_CTL_DITHER_ONE
  *             - \ref DPWM_CTL_DITHER_TWO
  * @return     None
  * @details    In order to prevent structured noise on PWM output due to DC offsets in the
  *             input signal, it is possible to add random dither to the PWM signal.
  */
#define DPWM_SET_DITHER(dpwm,u8Type)    (dpwm->CTL = (dpwm->CTL&(~DPWM_CTL_DITHEREN_Msk))|u8Type)

/**
  * @brief      Enable DPWM dead time
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Enabling dead time will insert an additional clock cycle 
  *             into the switching of PMOS and NMOS driver transistors.
  */
#define DPWM_ENABLE_DEADTIME(dpwm)    (dpwm->CTL |= DPWM_CTL_DEADTIME_Msk)

/**
  * @brief      Disable DPWM dead time
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  */
#define DPWM_DISABLE_DEADTIME(dpwm)    (dpwm->CTL &= (~DPWM_CTL_DEADTIME_Msk))

/**
  * @brief      Set DPWM modulation frequency
  * @param[in]  dpwm The base address of DPWM module
  * @param[in]  u8Frq is modulation frequency type
  *             - \ref DPWM_CTL_MODUFRQ0
  *             - \ref DPWM_CTL_MODUFRQ1
  *             - \ref DPWM_CTL_MODUFRQ2
  *             - \ref DPWM_CTL_MODUFRQ3
  *             - \ref DPWM_CTL_MODUFRQ4
  *             - \ref DPWM_CTL_MODUFRQ5
  *             - \ref DPWM_CTL_MODUFRQ6
  *             - \ref DPWM_CTL_MODUFRQ7
  * @return     None
  * @details    This parameter controls the carrier modulation frequency of 
  *             the PWM signal as a proportion of DPWM_CLK.
  */
#define DPWM_SET_MODFREQUENCY(dpwm,u8Frq)    (dpwm->CTL = (dpwm->CTL&(~DPWM_CTL_MODUFRQ_Msk))|u8Frq)

/**
  * @brief      Get DPWM modulation frequency
  * @param[in]  dpwm The base address of DPWM module
  * @return     modulation frequency type
  */
#define DPWM_GET_MODFREQUENCY(dpwm)    (dpwm->CTL & DPWM_CTL_MODUFRQ_Msk)

/**
  * @brief      Check DPWM FIFO full or not
  * @param[in]  dpwm The base address of DPWM module
  * @return     0 = FIFO is not full
  *             1 = FIFO is full
  */
#define DPWM_IS_FIFOFULL(dpwm)    (dpwm->STS&DPWM_STS_FULL_Msk)

/**
  * @brief      Check DPWM FIFO empty or not
  * @param[in]  dpwm The base address of DPWM module
  * @return     0 = FIFO is not empty
  *             1 = FIFO is empty
  */
#define DPWM_IS_FIFOEMPTY(dpwm)    (dpwm->STS&DPWM_STS_EMPTY_Msk)

/**
  * @brief     Enable DPWM PDMA interface.
  * @param[in] dpwm The base address of DPWM module
  * @return    None.
  * @details   DPWM will request data from PDMA controller whenever there is space in FIFO.
  */
#define DPWM_ENABLE_PDMA(dpwm)     (dpwm->DMACTL = 1)

/**
  * @brief     Disable DPWM PDMA interface.
  * @param[in] dpwm The base address of DPWM module
  * @return    None.
  */
#define DPWM_DISABLE_PDMA(dpwm)     (dpwm->DMACTL = 0)

/**
  * @brief     DPWM, SPK pins are enabled and driven, data is transferred from FIFO
  * @param[in] dpwm The base address of DPWM module
  * @return    None.
  */
#define DPWM_START_PLAY(dpwm)    (dpwm->CTL |= DPWM_CTL_DPWMEN_Msk)

/**
  * @brief     DPWM, SPK pins are tri-state, CIC filter is reset, FIFO pointers are reset
  * @param[in] dpwm The base address of DPWM module
  * @return    None.
  */
#define DPWM_STOP_PLAY(dpwm)     (dpwm->CTL &= (~DPWM_CTL_DPWMEN_Msk))

/**
  * @brief     Set input sample rate divisor of the DPWM
  * @param[in] dpwm The base address of DPWM module
  * @param[in] u8Divisor is zero order Hold, down-sampling divisor.
  * @return    None
  * @details   The input sample rate of the DPWM is set by HCLK frequency and the divisor 
  *            set in this register by the following formula: 
  *            
  *            Fs = HCLK/u8Divisor/64
  *            
  *            Valid range is 1 to 255. Default is 48, which gives a sample rate of 16kHz 
  *            for a 49.152MHz (default) HCLK.
  *            The audio stream is sampled by a zero-order hold and fed to an up-sampling Cascaded
  *            Integrator Comb (CIC) filter with an up-sampling ratio of 64.
  */
#define DPWM_SET_FSDIV(dpwm,u8Divisor)    (dpwm->ZOHDIV = (dpwm->ZOHDIV&~~DPWM_ZOHDIV_ZOHDIV_Msk))|(u8Divisor&0xff))

/**
  * @brief     Set gain value of the DPWM
  * @param[in] dpwm The base address of DPWM module6
  * @param[in] u8Value is gain value for dpwm configuration.
  * @return    None
  * @details   (GAIN[7:0]+1)/256
  */
#define DPWM_SET_GAIN(dpwm,u8Value)       (dpwm->ZOHDIV = (dpwm->ZOHDIV&(~DPWM_ZOHDIV_GAIN_Msk))|((uint32_t)u8Value<<DPWM_ZOHDIV_GAIN_Pos))

/**
  * @brief     Get gain value of the DPWM
  * @param[in] dpwm The base address of DPWM module
  * @return    gain value of DPWM
  */
#define DPWM_GET_GAIN(dpwm)               ((dpwm->ZOHDIV&DPWM_ZOHDIV_GAIN_Msk)>>DPWM_ZOHDIV_GAIN_Pos)

void DPWM_Open(void);

void DPWM_Close(void);

void DPWM_WriteFIFO(int16_t *pi16Stream, uint32_t u32count);

uint32_t DPWM_SetSampleRate(uint32_t u32SampleRate);

uint32_t DPWM_GetSampleRate(void);

/*@}*/ /* end of group ISD9000_DPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9000_DPWM_Driver */

/*@}*/ /* end of group ISD9000_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__DPWM_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
