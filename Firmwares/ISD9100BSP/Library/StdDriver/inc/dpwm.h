/**************************************************************************//**
 * @file     DPWM.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/07/04 07:27p $
 * @brief    ISD9100 Series DPWM Driver Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#ifndef __DPWM_H__
#define __DPWM_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_DPWM_Driver DPWM Driver
  @{
*/

/** @addtogroup ISD9100_DPWM_EXPORTED_CONSTANTS DPWM Exported Constants
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

/*@}*/ /* end of group ISD9100_DPWM_EXPORTED_CONSTANTS */

/** @addtogroup ISD9100_DPWM_EXPORTED_FUNCTIONS DPWM Exported Functions
  @{
*/

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
#define DPWM_SET_DITHER(dpwm, \
                        u8Type)     (dpwm->CTL = (dpwm->CTL&~DPWM_CTL_DITHEREN_Msk)|u8Type)

/**
  * @brief      Enable DPWM dead time
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Enabling dead time will insert an additional clock cycle 
  *             into the switching of PMOS and NMOS driver transistors.
  */
#define DPWM_ENABLE_DEADTIME(dpwm)   (dpwm->CTL |= DPWM_CTL_DEADTIME_Msk)

/**
  * @brief      Disable DPWM dead time
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  */
#define DPWM_DISABLE_DEADTIME(dpwm)   (dpwm->CTL &= (~DPWM_CTL_DEADTIME_Msk))

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
#define DPWM_SET_MODFREQUENCY(dpwm, \
                              u8Frq)  (dpwm->CTL = (dpwm->CTL&~DPWM_CTL_MODUFRQ_Msk)|u8Frq)

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
#define DPWM_SET_FSDIV(dpwm, \
                       u8Divisor)    (dpwm->ZOHDIV = (u8Divisor&0xff))

void DPWM_Open(void);
void DPWM_Close(void);
void DPWM_WriteFIFO(int16_t *pi16Stream, uint32_t u32count);
uint32_t DPWM_SetSampleRate(uint32_t u32SampleRate);
uint32_t DPWM_GetSampleRate(void);

/*@}*/ /* end of group ISD9100_DPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_DPWM_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__DPWM_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
