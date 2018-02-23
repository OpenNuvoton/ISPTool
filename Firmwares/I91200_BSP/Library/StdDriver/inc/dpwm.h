/**************************************************************************//**
 * @file     DPWM.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/4/17 07:27p $
 * @brief    I91200 Series DPWM Driver Header File
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __DPWM_H__
#define __DPWM_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_DPWM_Driver DPWM Driver
  @{
*/

/** @addtogroup I91200_DPWM_EXPORTED_CONSTANTS DPWM Exported Constants
  @{
*/ 

/*---------------------------------------------------------------------------------------------------------*/
/* DPWM CTL Constant Definitions                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define DPWM_FIFO_DATAWIDTH_MSB24BITS   (0x0ul << DPWM_CTL_FIFOWIDTH_Pos)   /*!< DPWM FIFO Data Width : 24 Bits and MSB[31:8]    */
#define DPWM_FIFO_DATAWIDTH_16BITS      (0x1ul << DPWM_CTL_FIFOWIDTH_Pos)   /*!< DPWM FIFO Data Width : 16 Bits [15:0]           */
#define DPWM_FIFO_DATAWIDTH_8BITS       (0x2ul << DPWM_CTL_FIFOWIDTH_Pos)   /*!< DPWM FIFO Data Width : 8 Bits [7:0]             */
#define DPWM_FIFO_DATAWIDTH_24BITS      (0x3ul << DPWM_CTL_FIFOWIDTH_Pos)   /*!< DPWM FIFO Data Width : 24 Bits [23:0]           */

/*@}*/ /* end of group I91200_DPWM_EXPORTED_CONSTANTS */

/** @addtogroup I91200_DPWM_EXPORTED_FUNCTIONS DPWM Exported Functions
  @{
*/

/**
  * @brief      Enable DPWM dead time
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    Enabling dead time will insert an additional clock cycle 
  *             into the switching of PMOS and NMOS driver transistors.
  */
#define DPWM_ENABLE_DEADTIME(dpwm)    ((dpwm)->CTL |= DPWM_CTL_DEADTIME_Msk)

/**
  * @brief      Disable DPWM dead time
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  */
#define DPWM_DISABLE_DEADTIME(dpwm)    ((dpwm)->CTL &= (~DPWM_CTL_DEADTIME_Msk))

/**
  * @brief      Enable DPWM FIFO threshold interrupt.
  * @param[in]  dpwm The base address of DPWM module
  * @param[in]	u8Value FIFO threshold value
  * @return     None
  * @details    DPWM FIFO threshold interrupt Enabled.
  */
#define DPWM_ENABLE_FIFOTHRESHOLDINT(dpwm,u8Value)    ((dpwm)->CTL = (((dpwm)->CTL&(~DPWM_CTL_RXTH_Msk))|((u8Value<<DPWM_CTL_RXTH_Pos)&DPWM_CTL_RXTH_Msk))|DPWM_CTL_RXTHIE_Msk)

/**
  * @brief      Disable DPWM FIFO threshold interrupt.
  * @param[in]  dpwm The base address of DPWM module
  * @return     None
  * @details    DPWM FIFO threshold interrupt Disabled.			
  */
#define DPWM_DISABLE_FIFOTHRESHOLDINT(dpwm)    ((dpwm)->CTL &= (~DPWM_CTL_RXTHIE_Msk))

/**
  * @brief      Check DPWM FIFO full or not
  * @param[in]  dpwm The base address of DPWM module
  * @return     0 = FIFO is not full
  *             1 = FIFO is full
  */
#define DPWM_IS_FIFOFULL(dpwm)    ((dpwm)->STS&DPWM_STS_FULL_Msk)

/**
  * @brief      Check DPWM FIFO empty or not
  * @param[in]  dpwm The base address of DPWM module
  * @return     0 = FIFO is not empty
  *             1 = FIFO is empty
  */
#define DPWM_IS_FIFOEMPTY(dpwm)    ((dpwm)->STS&DPWM_STS_EMPTY_Msk)

/**
  * @brief     Enable DPWM PDMA interface.
  * @param[in] dpwm The base address of DPWM module
  * @return    None.
  * @details   DPWM will request data from PDMA controller whenever there is space in FIFO.
  */
#define DPWM_ENABLE_PDMA(dpwm)    (dpwm->DMACTL = 1)

/**
  * @brief     Disable DPWM PDMA interface.
  * @param[in] dpwm The base address of DPWM module
  * @return    None.
  */
#define DPWM_DISABLE_PDMA(dpwm)    (dpwm->DMACTL = 0)

/**
  * @brief     DPWM, SPK pins are enabled and driven, data is transferred from FIFO
  * @param[in] dpwm The base address of DPWM module
  * @return    None.
  */
#define DPWM_START_PLAY(dpwm)    ((dpwm)->CTL |= DPWM_CTL_DPWMEN_Msk)

/**
  * @brief     DPWM, SPK pins are tri-state, CIC filter is reset, FIFO pointers are reset
  * @param[in] dpwm The base address of DPWM module
  * @return    None.
  */
#define DPWM_STOP_PLAY(dpwm)     ((dpwm)->CTL &= (~DPWM_CTL_DPWMEN_Msk))

/**
  * @brief     DPWM, Enable DPWM driver control.
  * @param[in] dpwm The base address of DPWM module
  * @return    None.
  */
#define DPWM_ENABLE_DRIVER(dpwm)    ((dpwm)->CTL |= DPWM_CTL_DWPMDRVEN_Msk)

/**
  * @brief     DPWM, Disable DPWM driver control.
  * @param[in] dpwm The base address of DPWM module
  * @return    None.
  */
#define DPWM_DISABLE_DRIVER(dpwm)    ((dpwm)->CTL &= (~DPWM_CTL_DWPMDRVEN_Msk))

/**
  * @brief     Set input sample rate divisor of the DPWM
  * @param[in] dpwm The base address of DPWM module
  * @param[in] u8Divisor is zero order Hold, down-sampling divisor.
  * @return    None
  * @details   The input sample rate of the DPWM is set by HCLK frequency and the divisor 
  *            set in this register by the following formula: 
  *            
  *            Fs = DPWM_CLK/u8FSDIV/BIQ_RATIO
  *            
  *            Valid range is 1 to 255. Default is 48, which gives a sample rate of 16kHz 
  *            for a 49.152MHz (default) HCLK.
  *            The audio stream is sampled by a zero-order hold and fed to an up-sampling Cascaded
  *            Integrator Comb (CIC) filter with an up-sampling ratio of 64.
  */
#define DPWM_SET_FSDIV(dpwm, u8Divisor)    (dpwm->ZOHDIV = (u8Divisor&0xff))
					   
/**
  * @brief     Get pointer that the valid data within the DPWM FIFO buffer
  * @param[in] dpwm The base address of DPWM module
  * @return    FIFO Pointer.
  * @details   The Maximum value is 15. When the using level of DPWM FIFO Buffer equals to 16,
  *            The FULL bit is set to 1.
  */
#define DPWM_GET_FIFOPOINTER(dpwm)		((dpwm->STS&DPWM_STS_FIFOPTR_Msk) >> DPWM_STS_FIFOPTR_Pos)

/**
  * @brief      Check DPWM FIFO count whether less than threshold
  * @param[in]  dpwm The base address of DPWM module
  * @return     0 = The valid data count within the DPWM FIFO buffer is larger than the setting value of RXTH
  *             1 = The valid data count within the transmit FIFO buffer is less than or equal to the setting value of RXTH.
  */
#define DPWM_IS_FIFOLESSTHANTH(dpwm)    (dpwm->STS&DPWM_STS_RXTHIF_Msk)

/**
  * @brief      Set DPWM FIFO data width selection
  * @param[in]  dpwm The base address of DPWM module
  * @param[in]  u32DataWidth DPWM data width
  *             - \ref DPWM_FIFO_DATAWIDTH_MSB24BITS
  *             - \ref DPWM_FIFO_DATAWIDTH_16BITS
  *             - \ref DPWM_FIFO_DATAWIDTH_8BITS
  *             - \ref DPWM_FIFO_DATAWIDTH_24BITS
  */
#define DPWM_SET_FIFODATAWIDTH(dpwm,u32DataWidth)    ((dpwm)->CTL = ((dpwm)->CTL&(~DPWM_CTL_FIFOWIDTH_Msk))|u32DataWidth)

void DPWM_Open(void);
void DPWM_Close(void);
void DPWM_WriteFIFO(int16_t *pi16Stream, uint32_t u32count);
uint32_t DPWM_SetSampleRate(uint32_t u32SampleRate);
uint32_t DPWM_GetSampleRate(void);

/*@}*/ /* end of group I91200_DPWM_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_DPWM_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__DPWM_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
