/**************************************************************************//**
 * @file     spi.h
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 15/04/17 11:45a $ 
 * @brief    Mini58 series SPI driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/ 
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_SPI_Driver SPI Driver
  @{
*/

/** @addtogroup Mini58_SPI_EXPORTED_CONSTANTS SPI Exported Constants
  @{
*/

#define SPI_MODE_0        (SPI_CTL_TXNEG_Msk)                            /*!< CLKP=0; RX_NEG=0; TX_NEG=1 */
#define SPI_MODE_1        (SPI_CTL_RXNEG_Msk)                            /*!< CLKP=0; RX_NEG=1; TX_NEG=0 */
#define SPI_MODE_2        (SPI_CTL_CLKPOL_Msk | SPI_CTL_RXNEG_Msk)       /*!< CLKP=1; RX_NEG=1; TX_NEG=0 */
#define SPI_MODE_3        (SPI_CTL_CLKPOL_Msk | SPI_CTL_TXNEG_Msk)       /*!< CLKP=1; RX_NEG=0; TX_NEG=1 */

#define SPI_SLAVE         (SPI_CTL_SLAVE_Msk)                             /*!< Set as slave */
#define SPI_MASTER        (0x0)                                             /*!< Set as master */

#define SPI_SS                (SPI_SSCTL_SS_Msk)                             /*!< Set SS0 */
#define SPI_SS_ACTIVE_HIGH    (SPI_SSCTL_SSACTPOL_Msk)                          /*!< SS active high */
#define SPI_SS_ACTIVE_LOW     (0x0)                                         /*!< SS active low */

#define SPI_IE_MASK                        (0x01)                           /*!< Interrupt enable mask */
#define SPI_SSTA_INTEN_MASK                (0x04)                           /*!< Slave 3-Wire mode start interrupt enable mask */
#define SPI_FIFO_TX_INTEN_MASK             (0x08)                           /*!< FIFO TX interrupt mask */
#define SPI_FIFO_RX_INTEN_MASK             (0x10)                           /*!< FIFO RX interrupt mask */
#define SPI_FIFO_RXOV_INTEN_MASK           (0x20)                           /*!< FIFO RX overrun interrupt mask */
#define SPI_FIFO_TIMEOUT_INTEN_MASK        (0x40)                           /*!< FIFO timeout interrupt mask */


/*@}*/ /* end of group Mini58_SPI_EXPORTED_CONSTANTS */


/** @addtogroup Mini58_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  Abort the current transfer in slave 3-wire mode.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define  SPI_ABORT_3WIRE_TRANSFER(spi) ( (spi)->SLVCTL |= SPI_SLVCTL_SLVABT_Msk )

/**
  * @brief  Clear the slave 3-wire mode start interrupt flag.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_CLR_3WIRE_START_INT_FLAG(spi) ( (spi)->STATUS = SPI_STATUS_SLVSTIF_Msk )

/**
  * @brief  Clear the unit transfer interrupt flag.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_CLR_UNIT_TRANS_INT_FLAG(spi) ( (spi)->STATUS = SPI_STATUS_UNITIF_Msk )

/**
  * @brief  Disable slave 3-wire mode.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_DISABLE_3WIRE_MODE(spi) ( (spi)->SLVCTL &= ~SPI_SLVCTL_SLV3WIRE_Msk )

/**
  * @brief  Enable slave 3-wire mode.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_ENABLE_3WIRE_MODE(spi) ( (spi)->SLVCTL |= SPI_SLVCTL_SLV3WIRE_Msk )

/**
  * @brief  Get the count of available data in RX FIFO.
  * @param[in]  spi is the base address of SPI module.
  * @return The count of available data in RX FIFO.
  */
#define SPI_GET_RX_FIFO_COUNT(spi) ( (((spi)->STATUS & SPI_STATUS_RXCNT_Msk) >> SPI_STATUS_RXCNT_Pos) & 0xf )

/**
  * @brief  Get the Rx FIFO empty flag.
  * @param[in]  spi is the base address of SPI module.
  * @return Rx FIFO flag
  * @retval 0 Rx FIFO is not empty
  * @retval 1 Rx FIFO is empty
  */
#define SPI_GET_RX_FIFO_EMPTY_FLAG(spi) ( ((spi)->STATUS & SPI_STATUS_RXEMPTY_Msk) == SPI_STATUS_RXEMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Tx FIFO empty flag.
  * @param[in]  spi is the base address of SPI module.
  * @return Tx FIFO flag
  * @retval 0 Tx FIFO is not empty
  * @retval 1 Tx FIFO is empty
  */
#define SPI_GET_TX_FIFO_EMPTY_FLAG(spi) ( ((spi)->STATUS & SPI_STATUS_TXEMPTY_Msk) == SPI_STATUS_TXEMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Tx FIFO full flag.
  * @param  spi is the base address of SPI module.
  * @return Tx FIFO flag
  * @retval 0 Tx FIFO is not full
  * @retval 1 Tx FIFO is full
  */
#define SPI_GET_TX_FIFO_FULL_FLAG(spi) ( ((spi)->STATUS & SPI_STATUS_TXFULL_Msk) == SPI_STATUS_TXFULL_Msk ? 1:0 )

/**
  * @brief  Get the datum read from Rx FIFO.
  * @param[in]  spi is the base address of SPI module.
  * @return Data in Rx buffer
  */
#define SPI_READ_RX(spi) ( (spi)->RX )

/**
  * @brief  Write datum to TX register.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32TxData is the datum which user attempt to transfer through SPI bus.
  * @return none
  */
#define SPI_WRITE_TX(spi, u32TxData) ( (spi)->TX = u32TxData )

/**
  * @brief  Disable automatic slave select function and set SPI_SS pin to high state.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
static __INLINE void SPI_SET_SS_HIGH(SPI_T *spi)
{
  spi->SSCTL &= ~SPI_SSCTL_AUTOSS_Msk;  
  spi->SSCTL |= (SPI_SSCTL_LTF_Msk | SPI_SSCTL_SSACTPOL_Msk | SPI_SSCTL_SS_Msk);
}

/**
  * @brief  Disable automatic slave select function and set SPI_SS pin to low state.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
static __INLINE void SPI_SET_SS_LOW(SPI_T *spi)
{
  spi->SSCTL &= ~SPI_SSCTL_AUTOSS_Msk;
  spi->SSCTL |= SPI_SSCTL_LTF_Msk;  
  spi->SSCTL &= ~SPI_SSCTL_SSACTPOL_Msk;
  spi->SSCTL |= SPI_SSCTL_SS_Msk;
}

/**
  * @brief Enable byte reorder function.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_ENABLE_BYTE_REORDER(spi) ( (spi)->CTL |= SPI_CTL_REORDER_Msk )

/**
  * @brief  Disable byte reorder function.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_DISABLE_BYTE_REORDER(spi) ( (spi)->CTL &= ~SPI_CTL_REORDER_Msk )

/**
  * @brief  Set the length of suspend interval.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32SuspCycle decides the length of suspend interval.
  * @return none
  */
#define SPI_SET_SUSPEND_CYCLE(spi, u32SuspCycle) ( (spi)->CTL = ((spi)->CTL & ~SPI_CTL_SUSPITV_Msk) | (u32SuspCycle << SPI_CTL_SUSPITV_Pos) )

/**
  * @brief  Set the SPI transfer sequence with LSB first.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_SET_LSB_FIRST(spi) ( (spi)->CTL |= SPI_CTL_LSB_Msk )

/**
  * @brief  Set the SPI transfer sequence with MSB first.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_SET_MSB_FIRST(spi) ( (spi)->CTL &= ~SPI_CTL_LSB_Msk )

/**
  * @brief  Set the data width of a SPI transaction.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32Width is the bit width of transfer data.  
  * @return none
  */
static __INLINE void SPI_SET_DATA_WIDTH(SPI_T *spi, uint32_t u32Width)
{
   if(u32Width == 32)
        u32Width = 0;
        
   spi->CTL = (spi->CTL & ~SPI_CTL_DWIDTH_Msk) | (u32Width << SPI_CTL_DWIDTH_Pos);
}

/**
  * @brief  Get the SPI busy state.
  * @param[in]  spi is the base address of SPI module.
  * @return SPI busy status
  * @retval 0 SPI module is not busy
  * @retval 1 SPI module is busy
  */
#define SPI_IS_BUSY(spi) ( ((spi)->CTL & SPI_CTL_SPIEN_Msk) == SPI_CTL_SPIEN_Msk ? 1:0 )

/**
  * @brief  Set the GO_BUSY bit to trigger SPI transfer.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
#define SPI_TRIGGER(spi) ( (spi)->CTL |= SPI_CTL_SPIEN_Msk )

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

/*@}*/ /* end of group Mini58_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_SPI_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SPI_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
