/**************************************************************************//**
 * @file     spi.h
 * @version  V1.00
 * $Revision: 15 $
 * $Date: 15/10/01 11:36a $ 
 * @brief    Mini51 series SPI driver header file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/ 
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_SPI_Driver SPI Driver
  @{
*/

/** @addtogroup MINI51_SPI_EXPORTED_CONSTANTS SPI Exported Constants
  @{
*/

#define SPI_MODE_0        (SPI_CNTRL_TX_NEG_Msk)                            /*!< CLKP=0; RX_NEG=0; TX_NEG=1 */
#define SPI_MODE_1        (SPI_CNTRL_RX_NEG_Msk)                            /*!< CLKP=0; RX_NEG=1; TX_NEG=0 */
#define SPI_MODE_2        (SPI_CNTRL_CLKP_Msk | SPI_CNTRL_RX_NEG_Msk)       /*!< CLKP=1; RX_NEG=1; TX_NEG=0 */
#define SPI_MODE_3        (SPI_CNTRL_CLKP_Msk | SPI_CNTRL_TX_NEG_Msk)       /*!< CLKP=1; RX_NEG=0; TX_NEG=1 */

#define SPI_SLAVE         (SPI_CNTRL_SLAVE_Msk)                             /*!< Set as slave */
#define SPI_MASTER        (0x0)                                             /*!< Set as master */

#define SPI_SS                (SPI_SSR_SSR_Msk)                             /*!< Set SS0 */
#define SPI_SS_ACTIVE_HIGH    (SPI_SSR_SS_LVL_Msk)                          /*!< SS active high */
#define SPI_SS_ACTIVE_LOW     (0x0)                                         /*!< SS active low */

#define SPI_IE_MASK                        (0x01)                           /*!< Interrupt enable mask */
#define SPI_SSTA_INTEN_MASK                (0x04)                           /*!< Slave 3-Wire mode start interrupt enable mask */
#define SPI_FIFO_TX_INTEN_MASK             (0x08)                           /*!< FIFO TX interrupt mask */
#define SPI_FIFO_RX_INTEN_MASK             (0x10)                           /*!< FIFO RX interrupt mask */
#define SPI_FIFO_RXOV_INTEN_MASK           (0x20)                           /*!< FIFO RX overrun interrupt mask */
#define SPI_FIFO_TIMEOUT_INTEN_MASK        (0x40)                           /*!< FIFO timeout interrupt mask */


/*@}*/ /* end of group MINI51_SPI_EXPORTED_CONSTANTS */


/** @addtogroup MINI51_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  Abort the current transfer in slave 3-wire mode.
  * @param  spi is the base address of SPI module.
  * @return none
  */
#define  SPI_ABORT_3WIRE_TRANSFER(spi) ( (spi)->CNTRL2 |= SPI_CNTRL2_SLV_ABORT_Msk )

/**
  * @brief  Clear the slave 3-wire mode start interrupt flag.
  * @param  spi is the base address of SPI module.
  * @return none
  */
#define SPI_CLR_3WIRE_START_INT_FLAG(spi) ( (spi)->STATUS = SPI_STATUS_SLV_START_INTSTS_Msk )

/**
  * @brief  Clear the unit transfer interrupt flag.
  * @param  spi is the base address of SPI module.
  * @return none
  */
#define SPI_CLR_UNIT_TRANS_INT_FLAG(spi) ( (spi)->STATUS = SPI_STATUS_IF_Msk )

/**
  * @brief  Disable slave 3-wire mode.
  * @param  spi is the base address of SPI module.
  * @return none
  */
#define SPI_DISABLE_3WIRE_MODE(spi) ( (spi)->CNTRL2 &= ~SPI_CNTRL2_NOSLVSEL_Msk )

/**
  * @brief  Enable slave 3-wire mode.
  * @param  spi is the base address of SPI module.
  * @return none
  */
#define SPI_ENABLE_3WIRE_MODE(spi) ( (spi)->CNTRL2 |= SPI_CNTRL2_NOSLVSEL_Msk )

/**
  * @brief  Get the count of available data in RX FIFO.
  * @param  spi is the base address of SPI module.
  * @return The count of available data in RX FIFO.
  */
#define SPI_GET_RX_FIFO_COUNT(spi) ( (((spi)->STATUS & SPI_STATUS_RX_FIFO_COUNT_Msk) >> SPI_STATUS_RX_FIFO_COUNT_Pos) & 0xf )

/**
  * @brief  Get the Rx FIFO empty flag.
  * @param  spi is the base address of SPI module.
  * @return Rx FIFO flag
  * @retval 0 Rx FIFO is not empty
  * @retval 1 Rx FIFO is empty
  */
#define SPI_GET_RX_FIFO_EMPTY_FLAG(spi) ( ((spi)->STATUS & SPI_STATUS_RX_EMPTY_Msk) == SPI_STATUS_RX_EMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Tx FIFO empty flag.
  * @param  spi is the base address of SPI module.
  * @return Tx FIFO flag
  * @retval 0 Tx FIFO is not empty
  * @retval 1 Tx FIFO is empty
  */
#define SPI_GET_TX_FIFO_EMPTY_FLAG(spi) ( ((spi)->STATUS & SPI_STATUS_TX_EMPTY_Msk) == SPI_STATUS_TX_EMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Tx FIFO full flag.
  * @param  spi is the base address of SPI module.
  * @return Tx FIFO flag
  * @retval 0 Tx FIFO is not full
  * @retval 1 Tx FIFO is full
  */
#define SPI_GET_TX_FIFO_FULL_FLAG(spi) ( ((spi)->STATUS & SPI_STATUS_TX_FULL_Msk) == SPI_STATUS_TX_FULL_Msk ? 1:0 )

/**
  * @brief  Get the datum read from Rx FIFO.
  * @param  spi is the base address of SPI module.
  * @return Data in Rx buffer
  */
#define SPI_READ_RX(spi) ( (spi)->RX )

/**
  * @brief  Write datum to TX register.
  * @param  spi is the base address of SPI module.
  * @param  u32TxData is the datum which user attempt to transfer through SPI bus.
  * @return none
  */
#define SPI_WRITE_TX(spi, u32TxData) ( (spi)->TX = u32TxData )

/**
  * @brief  Disable automatic slave select function and set SPI_SS pin to high state.
  * @param  spi is the base address of SPI module.
  * @return none
  */
static __INLINE void SPI_SET_SS_HIGH(SPI_T *spi)
{
  spi->SSR &= ~SPI_SSR_AUTOSS_Msk;  
  spi->SSR |= (SPI_SSR_LTRIG_FLAG_Msk | SPI_SSR_SS_LVL_Msk | SPI_SSR_SSR_Msk);  
}

/**
  * @brief  Disable automatic slave select function and set SPI_SS pin to low state.
  * @param  spi is the base address of SPI module.
  * @return none
  */
static __INLINE void SPI_SET_SS_LOW(SPI_T *spi)
{
  spi->SSR &= ~SPI_SSR_AUTOSS_Msk;
  spi->SSR |= SPI_SSR_LTRIG_FLAG_Msk;  
  spi->SSR &= ~SPI_SSR_SS_LVL_Msk;
  spi->SSR |= SPI_SSR_SSR_Msk;
}

/**
  * @brief Enable byte reorder function.
  * @param  spi is the base address of SPI module.
  * @return none
  */
#define SPI_ENABLE_BYTE_REORDER(spi) ( (spi)->CNTRL |= SPI_CNTRL_REORDER_Msk )

/**
  * @brief  Disable byte reorder function.
  * @param  spi is the base address of SPI module.
  * @return none
  */
#define SPI_DISABLE_BYTE_REORDER(spi) ( (spi)->CNTRL &= ~SPI_CNTRL_REORDER_Msk )

/**
  * @brief  Set the length of suspend interval.
  * @param  spi is the base address of SPI module.
  * @param  u32SuspCycle decides the length of suspend interval.
  * @return none
  */
#define SPI_SET_SUSPEND_CYCLE(spi, u32SuspCycle) ( (spi)->CNTRL = ((spi)->CNTRL & ~SPI_CNTRL_SP_CYCLE_Msk) | (u32SuspCycle << SPI_CNTRL_SP_CYCLE_Pos) )

/**
  * @brief  Set the SPI transfer sequence with LSB first.
  * @param  spi is the base address of SPI module.
  * @return none
  */
#define SPI_SET_LSB_FIRST(spi) ( (spi)->CNTRL |= SPI_CNTRL_LSB_Msk )

/**
  * @brief  Set the SPI transfer sequence with MSB first.
  * @param  spi is the base address of SPI module.
  * @return none
  */
#define SPI_SET_MSB_FIRST(spi) ( (spi)->CNTRL &= ~SPI_CNTRL_LSB_Msk )

/**
  * @brief  Set the data width of a SPI transaction.
  * @param  spi is the base address of SPI module.
  * @param  u32Width is the bit width of transfer data.  
  * @return none
  */
static __INLINE void SPI_SET_DATA_WIDTH(SPI_T *spi, uint32_t u32Width)
{
   if(u32Width == 32)
        u32Width = 0;
        
   spi->CNTRL = (spi->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) | (u32Width << SPI_CNTRL_TX_BIT_LEN_Pos);
}

/**
  * @brief  Get the SPI busy state.
  * @param  spi is the base address of SPI module.
  * @return SPI busy status
  * @retval 0 SPI module is not busy
  * @retval 1 SPI module is busy
  */
#define SPI_IS_BUSY(spi) ( ((spi)->CNTRL & SPI_CNTRL_GO_BUSY_Msk) == SPI_CNTRL_GO_BUSY_Msk ? 1:0 )

/**
  * @brief  Set the GO_BUSY bit to trigger SPI transfer.
  * @param  spi is the base address of SPI module.
  * @return none
  */
#define SPI_TRIGGER(spi) ( (spi)->CNTRL |= SPI_CNTRL_GO_BUSY_Msk )

uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_EnableFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
void SPI_DisableFIFO(SPI_T *spi);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);

/*@}*/ /* end of group MINI51_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_SPI_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SPI_H__

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
