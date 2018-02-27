/******************************************************************************
 * @file     spi.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/03/02 10:26a $
 * @brief    I94100 series SPI driver header file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __SPI_H__
#define __SPI_H__

#include "Platform.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_SPI_Driver SPI Driver
  @{
*/

/** @addtogroup I94100_SPI_EXPORTED_CONSTANTS SPI Exported Constants
  @{
*/

#define SPI_MODE_0        (SPI_CTL_TXNEG_Msk)                             /*!< CLKPOL=0; RXNEG=0; TXNEG=1 */
#define SPI_MODE_1        (SPI_CTL_RXNEG_Msk)                             /*!< CLKPOL=0; RXNEG=1; TXNEG=0 */
#define SPI_MODE_2        (SPI_CTL_CLKPOL_Msk | SPI_CTL_RXNEG_Msk)        /*!< CLKPOL=1; RXNEG=1; TXNEG=0 */
#define SPI_MODE_3        (SPI_CTL_CLKPOL_Msk | SPI_CTL_TXNEG_Msk)        /*!< CLKPOL=1; RXNEG=0; TXNEG=1 */

#define SPI_SLAVE         (SPI_CTL_SLAVE_Msk)                             /*!< Set as slave */
#define SPI_MASTER        (0x0)                                           /*!< Set as master */

#define SPI_SS0               (SPI_SSCTL_SS0_Msk)                          /*!< Set SS0 */
#define SPI_SS1               (SPI_SSCTL_SS1_Msk)                          /*!< Set SS1 */
#define SPI_SS_ACTIVE_HIGH    (SPI_SSCTL_SSACTPOL_Msk)                     /*!< SS active high */
#define SPI_SS_ACTIVE_LOW     (0x0)                                        /*!< SS active low */

/* SPI Interrupt Mask */
#define SPI_UNIT_INT_MASK                (0x001)                          /*!< Unit transfer interrupt mask */
#define SPI_SSACT_INT_MASK               (0x002)                          /*!< Slave selection signal active interrupt mask */
#define SPI_SSINACT_INT_MASK             (0x004)                          /*!< Slave selection signal inactive interrupt mask */
#define SPI_SLVUR_INT_MASK               (0x008)                          /*!< Slave under run interrupt mask */
#define SPI_SLVBE_INT_MASK               (0x010)                          /*!< Slave bit count error interrupt mask */
#define SPI_SLVTO_INT_MASK               (0x020)                          /*!< Slave time-out interrupt mask */
#define SPI_TXUF_INT_MASK                (0x040)                          /*!< Slave TX underflow interrupt mask */
#define SPI_FIFO_TXTH_INT_MASK           (0x080)                          /*!< FIFO TX threshold interrupt mask */
#define SPI_FIFO_RXTH_INT_MASK           (0x100)                          /*!< FIFO RX threshold interrupt mask */
#define SPI_FIFO_RXOV_INT_MASK           (0x200)                          /*!< FIFO RX overrun interrupt mask */
#define SPI_FIFO_RXTO_INT_MASK           (0x400)                          /*!< FIFO RX time-out interrupt mask */

/* SPI Status Mask */
#define SPI_BUSY_MASK                    (0x01)                           /*!< Busy status mask */
#define SPI_RX_EMPTY_MASK                (0x02)                           /*!< RX empty status mask */
#define SPI_RX_FULL_MASK                 (0x04)                           /*!< RX full status mask */
#define SPI_TX_EMPTY_MASK                (0x08)                           /*!< TX empty status mask */
#define SPI_TX_FULL_MASK                 (0x10)                           /*!< TX full status mask */
#define SPI_TXRX_RESET_MASK              (0x20)                           /*!< TX or RX reset status mask */
#define SPI_SPIEN_STS_MASK               (0x40)                           /*!< SPIEN status mask */
#define SPI_SSLINE_STS_MASK              (0x80)                           /*!< SPIn_SS line status mask */

/*@}*/ /* end of group I94100_SPI_EXPORTED_CONSTANTS */

/** @addtogroup I94100_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief      Clear the unit transfer interrupt flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Write 1 to UNITIF bit of SPI_STATUS register to clear the unit transfer interrupt flag.
  */
#define SPI_CLR_UNIT_TRANS_INT_FLAG(spi)   ((spi)->STATUS = SPI_STATUS_UNITIF_Msk)

/**
  * @brief      Disable 2-bit Transfer mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear TWOBIT bit of SPI_CTL register to disable 2-bit Transfer mode.
  */
#define SPI_DISABLE_2BIT_MODE(spi)   ((spi)->CTL &= ~SPI_CTL_TWOBIT_Msk)

/**
  * @brief      Disable Slave 3-wire mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear SLV3WIRE bit of SPI_SSCTL register to disable Slave 3-wire mode.
  */
#define SPI_DISABLE_3WIRE_MODE(spi)   ((spi)->SSCTL &= ~SPI_SSCTL_SLV3WIRE_Msk)

/**
  * @brief      Disable Dual I/O mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear DUALIOEN bit of SPI_CTL register to disable Dual I/O mode.
  */
#define SPI_DISABLE_DUAL_MODE(spi)   ((spi)->CTL &= ~SPI_CTL_DUALIOEN_Msk)

/**
  * @brief      Disable Quad I/O mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear QUADIOEN bit of SPI_CTL register to disable Quad I/O mode.
  */
#define SPI_DISABLE_QUAD_MODE(spi)   ((spi)->CTL &= ~SPI_CTL_QUADIOEN_Msk)

/**
  * @brief      Enable 2-bit Transfer mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set TWOBIT bit of SPI_CTL register to enable 2-bit Transfer mode.
  */
#define SPI_ENABLE_2BIT_MODE(spi)   ((spi)->CTL |= SPI_CTL_TWOBIT_Msk)

/**
  * @brief      Enable Slave 3-wire mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set SLV3WIRE bit of SPI_SSCTL register to enable Slave 3-wire mode.
  */
#define SPI_ENABLE_3WIRE_MODE(spi)   ((spi)->SSCTL |= SPI_SSCTL_SLV3WIRE_Msk)

/**
  * @brief      Enable Dual input mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear QDIODIR bit and set DUALIOEN bit of SPI_CTL register to enable Dual input mode.
  */
#define SPI_ENABLE_DUAL_INPUT_MODE(spi)   ((spi)->CTL = ((spi)->CTL & (~SPI_CTL_QDIODIR_Msk)) | SPI_CTL_DUALIOEN_Msk)

/**
  * @brief      Enable Dual output mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set QDIODIR bit and DUALIOEN bit of SPI_CTL register to enable Dual output mode.
  */
#define SPI_ENABLE_DUAL_OUTPUT_MODE(spi)   ((spi)->CTL |= (SPI_CTL_QDIODIR_Msk | SPI_CTL_DUALIOEN_Msk))

/**
  * @brief      Enable Quad input mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear QDIODIR bit and set QUADIOEN bit of SPI_CTL register to enable Quad input mode.
  */
#define SPI_ENABLE_QUAD_INPUT_MODE(spi)   ((spi)->CTL = ((spi)->CTL & (~SPI_CTL_QDIODIR_Msk)) | SPI_CTL_QUADIOEN_Msk)

/**
  * @brief      Enable Quad output mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set QDIODIR bit and QUADIOEN bit of SPI_CTL register to enable Quad output mode.
  */
#define SPI_ENABLE_QUAD_OUTPUT_MODE(spi)   ((spi)->CTL |= (SPI_CTL_QDIODIR_Msk | SPI_CTL_QUADIOEN_Msk))

/**
  * @brief      Trigger RX PDMA function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set RXPDMAEN bit of SPI_PDMACTL register to enable RX PDMA transfer function.
  */
#define SPI_TRIGGER_RX_PDMA(spi)   ((spi)->PDMACTL |= SPI_PDMACTL_RXPDMAEN_Msk)

/**
  * @brief      Trigger TX PDMA function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set TXPDMAEN bit of SPI_PDMACTL register to enable TX PDMA transfer function.
  */
#define SPI_TRIGGER_TX_PDMA(spi)   ((spi)->PDMACTL |= SPI_PDMACTL_TXPDMAEN_Msk)

/**
  * @brief      Disable RX PDMA transfer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear RXPDMAEN bit of SPI_PDMACTL register to disable RX PDMA transfer function.
  */
#define SPI_DISABLE_RX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI_PDMACTL_RXPDMAEN_Msk )

/**
  * @brief      Disable TX PDMA transfer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear TXPDMAEN bit of SPI_PDMACTL register to disable TX PDMA transfer function.
  */
#define SPI_DISABLE_TX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI_PDMACTL_TXPDMAEN_Msk )

/**
  * @brief      Get the count of available data in RX FIFO.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     The count of available data in RX FIFO.
  * @details    Read RXCNT (SPI_STATUS[27:24]) to get the count of available data in RX FIFO.
  */
#define SPI_GET_RX_FIFO_COUNT(spi)   (((spi)->STATUS & SPI_STATUS_RXCNT_Msk) >> SPI_STATUS_RXCNT_Pos)

/**
  * @brief      Get the RX FIFO empty flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 RX FIFO is not empty.
  * @retval     1 RX FIFO is empty.
  * @details    Read RXEMPTY bit of SPI_STATUS register to get the RX FIFO empty flag.
  */
#define SPI_GET_RX_FIFO_EMPTY_FLAG(spi)   (((spi)->STATUS & SPI_STATUS_RXEMPTY_Msk)>>SPI_STATUS_RXEMPTY_Pos)

/**
  * @brief      Get the TX FIFO empty flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 TX FIFO is not empty.
  * @retval     1 TX FIFO is empty.
  * @details    Read TXEMPTY bit of SPI_STATUS register to get the TX FIFO empty flag.
  */
#define SPI_GET_TX_FIFO_EMPTY_FLAG(spi)   (((spi)->STATUS & SPI_STATUS_TXEMPTY_Msk)>>SPI_STATUS_TXEMPTY_Pos)

/**
  * @brief      Get the TX FIFO full flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 TX FIFO is not full.
  * @retval     1 TX FIFO is full.
  * @details    Read TXFULL bit of SPI_STATUS register to get the TX FIFO full flag.
  */
#define SPI_GET_TX_FIFO_FULL_FLAG(spi)   (((spi)->STATUS & SPI_STATUS_TXFULL_Msk)>>SPI_STATUS_TXFULL_Pos)

/**
  * @brief      Get the datum read from RX register.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     Data in RX register.
  * @details    Read SPI_RX register to get the received datum.
  */
#define SPI_READ_RX(spi)   ((spi)->RX)

/**
  * @brief      Write datum to TX register.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32TxData The datum which user attempt to transfer through SPI bus.
  * @return     None.
  * @details    Write u32TxData to SPI_TX register.
  */
#define SPI_WRITE_TX(spi, u32TxData)   ((spi)->TX = (u32TxData))

/**
  * @brief      Set SPIn_SS pin to high state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Disable automatic slave selection function and set SPIn_SS pin to high state.
  */
#define SPI_SET_SS_HIGH(spi)   ((spi)->SSCTL = ((spi)->SSCTL & (~SPI_SSCTL_AUTOSS_Msk)) | (SPI_SSCTL_SSACTPOL_Msk | SPI_SSCTL_SS_Msk))

/**
  * @brief      Set SPIn_SS pin to low state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Disable automatic slave selection function and set SPIn_SS pin to low state.
  */
#define SPI_SET_SS_LOW(spi)   ((spi)->SSCTL = ((spi)->SSCTL & (~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SSACTPOL_Msk))) | SPI_SSCTL_SS_Msk)

/**
  * @brief      Enable Byte Reorder function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Enable Byte Reorder function. The suspend interval depends on the setting of SUSPITV (SPI_CTL[7:4]).
  */
#define SPI_ENABLE_BYTE_REORDER(spi)   ((spi)->CTL |=  SPI_CTL_REORDER_Msk)

/**
  * @brief      Disable Byte Reorder function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear REORDER bit field of SPI_CTL register to disable Byte Reorder function.
  */
#define SPI_DISABLE_BYTE_REORDER(spi)   ((spi)->CTL &= ~SPI_CTL_REORDER_Msk)

/**
  * @brief      Set the length of suspend interval.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32SuspCycle Decides the length of suspend interval. It could be 0 ~ 15.
  * @return     None.
  * @details    Set the length of suspend interval according to u32SuspCycle.
  *             The length of suspend interval is ((u32SuspCycle + 0.5) * the length of one SPI bus clock cycle).
  */
#define SPI_SET_SUSPEND_CYCLE(spi, u32SuspCycle)   ((spi)->CTL = ((spi)->CTL & ~SPI_CTL_SUSPITV_Msk) | ((u32SuspCycle) << SPI_CTL_SUSPITV_Pos))

/**
  * @brief      Set the SPI transfer sequence with LSB first.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set LSB bit of SPI_CTL register to set the SPI transfer sequence with LSB first.
  */
#define SPI_SET_LSB_FIRST(spi)   ((spi)->CTL |= SPI_CTL_LSB_Msk)

/**
  * @brief      Set the SPI transfer sequence with MSB first.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear LSB bit of SPI_CTL register to set the SPI transfer sequence with MSB first.
  */
#define SPI_SET_MSB_FIRST(spi)   ((spi)->CTL &= ~SPI_CTL_LSB_Msk)

/**
  * @brief      Set the data width of a SPI transaction.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Width The bit width of one transaction.
  * @return     None.
  * @details    The data width can be 8 ~ 32 bits.
  */
#define SPI_SET_DATA_WIDTH(spi, u32Width)   ((spi)->CTL = ((spi)->CTL & ~SPI_CTL_DWIDTH_Msk) | (((u32Width)&0x1F) << SPI_CTL_DWIDTH_Pos))

/**
  * @brief      Get the SPI busy state.
  * @param[in]  spi The pointer of the specified SPI module.
  * @retval     0 SPI controller is not busy.
  * @retval     1 SPI controller is busy.
  * @details    This macro will return the busy state of SPI controller.
  */
#define SPI_IS_BUSY(spi)   ( ((spi)->STATUS & SPI_STATUS_BUSY_Msk)>>SPI_STATUS_BUSY_Pos )

/**
  * @brief      Enable SPI controller.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Set SPIEN (SPI_CTL[0]) to enable SPI controller.
  */
#define SPI_ENABLE(spi)   ((spi)->CTL |= SPI_CTL_SPIEN_Msk)

/**
  * @brief      Disable SPI controller.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return     None.
  * @details    Clear SPIEN (SPI_CTL[0]) to disable SPI controller.
  */
#define SPI_DISABLE(spi)   ((spi)->CTL &= ~SPI_CTL_SPIEN_Msk)

/* Function prototype declaration */
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32DataWidth, uint32_t u32BusClock);
void SPI_Close(SPI_T *spi);
void SPI_ClearRxFIFO(SPI_T *spi);
void SPI_ClearTxFIFO(SPI_T *spi);
void SPI_DisableAutoSS(SPI_T *spi);
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void SPI_SetFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t SPI_GetBusClock(SPI_T *spi);
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask);
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask);

/*@}*/ /* end of group I94100_SPI_EXPORTED_FUNCTIONS */

/** @addtogroup I94100_SPI_I2S_EXPORTED_CONSTANTS SPI_I2S Exported Constants
  @{
*/

/* SPI_I2S Data Width */
#define SPI_I2SDATABIT_8           	(0 << SPI_I2SCTL_WDWIDTH_Pos)        	/*!< I2S data width is 8-bit */
#define SPI_I2SDATABIT_16          	(1 << SPI_I2SCTL_WDWIDTH_Pos)        	/*!< I2S data width is 16-bit */
#define SPI_I2SDATABIT_24          	(2 << SPI_I2SCTL_WDWIDTH_Pos)        	/*!< I2S data width is 24-bit */
#define SPI_I2SDATABIT_32          	(3 << SPI_I2SCTL_WDWIDTH_Pos)        	/*!< I2S data width is 32-bit */

/* SPI_I2S Audio Format */
#define SPI_I2SMONO                	SPI_I2SCTL_MONO_Msk						/*!< Monaural channel */
#define SPI_I2SSTEREO              	0                                  		/*!< Stereo channel */

/* SPI_I2S Operation mode */
#define SPI_I2SSLAVE               	SPI_I2SCTL_SLAVE_Msk                  	/*!< As slave mode */
#define SPI_I2SMASTER              	0                                  		/*!< As master mode */

/* SPI_I2S Data Format */
#define SPI_I2SFORMAT_I2S          	(0<<SPI_I2SCTL_FORMAT_Pos)           	/*!< I2S data format */
#define SPI_I2SFORMAT_MSB          	(1<<SPI_I2SCTL_FORMAT_Pos)            	/*!< MSB justified data format */
#define SPI_I2SFORMAT_PCMA         	(2<<SPI_I2SCTL_FORMAT_Pos)            	/*!< LSB justified data format */
#define SPI_I2SFORMAT_PCMB	   		(4<<SPI_I2SCTL_FORMAT_Pos)            	/*!< PCM standard mode data format */

/* SPI_I2S FIFO Threshold */
#define SPI_I2S

/**/
#define SPI_I2SORDER_HIGH			(0<<SPI_I2SCTL_ORDER_Pos)           	/*!< Left channel data at high byte. */
#define SPI_I2SORDER_LOW			(1<<SPI_I2SCTL_ORDER_Pos)           	/*!< Left channel data at low byte. */

/**/
#define SPI_I2SMONO_LEFT			(1<<SPI_I2SCTL_RXLCH_Pos)				/*!< Receive left channel data in Mono mode.*/
#define SPI_I2SMONO_RIGHT			(0<<SPI_I2SCTL_RXLCH_Pos)				/*!< Receive right channel data in Mono mode.*/

/* SPI_I2S*/
#define SPI_I2S_FIFO_RX_LEVEL_0		(0<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 0. */
#define SPI_I2S_FIFO_RX_LEVEL_1		(1<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 1. */
#define SPI_I2S_FIFO_RX_LEVEL_2		(2<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 2. */
#define SPI_I2S_FIFO_RX_LEVEL_3		(3<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 3. */
#define SPI_I2S_FIFO_RX_LEVEL_4		(4<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 4. */
#define SPI_I2S_FIFO_RX_LEVEL_5		(5<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 5. */
#define SPI_I2S_FIFO_RX_LEVEL_6		(6<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 6. */
#define SPI_I2S_FIFO_RX_LEVEL_7		(7<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 7. */
#define SPI_I2S_FIFO_RX_LEVEL_8		(8<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 8. */
#define SPI_I2S_FIFO_RX_LEVEL_9		(9<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 9. */
#define SPI_I2S_FIFO_RX_LEVEL_10	(10<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 10. */
#define SPI_I2S_FIFO_RX_LEVEL_11	(11<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 11. */
#define SPI_I2S_FIFO_RX_LEVEL_12	(12<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 12. */
#define SPI_I2S_FIFO_RX_LEVEL_13	(13<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 13. */
#define SPI_I2S_FIFO_RX_LEVEL_14	(14<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 14. */
#define SPI_I2S_FIFO_RX_LEVEL_15	(15<<SPI_FIFOCTL_RXTH_Pos)				/*!< Receive FIFO threshold at 15. */

/* SPI_I2S*/
#define SPI_I2S_FIFO_TX_LEVEL_0		(0<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 0. */
#define SPI_I2S_FIFO_TX_LEVEL_1		(1<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 1. */
#define SPI_I2S_FIFO_TX_LEVEL_2		(2<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 2. */
#define SPI_I2S_FIFO_TX_LEVEL_3		(3<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 3. */
#define SPI_I2S_FIFO_TX_LEVEL_4		(4<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 4. */
#define SPI_I2S_FIFO_TX_LEVEL_5		(5<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 5. */
#define SPI_I2S_FIFO_TX_LEVEL_6		(6<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 6. */
#define SPI_I2S_FIFO_TX_LEVEL_7		(7<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 7. */
#define SPI_I2S_FIFO_TX_LEVEL_8		(8<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 8. */
#define SPI_I2S_FIFO_TX_LEVEL_9		(9<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 9. */
#define SPI_I2S_FIFO_TX_LEVEL_10	(10<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 10. */
#define SPI_I2S_FIFO_TX_LEVEL_11	(11<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 11. */
#define SPI_I2S_FIFO_TX_LEVEL_12	(12<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 12. */
#define SPI_I2S_FIFO_TX_LEVEL_13	(13<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 13. */
#define SPI_I2S_FIFO_TX_LEVEL_14	(14<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 14. */
#define SPI_I2S_FIFO_TX_LEVEL_15	(15<<SPI_FIFOCTL_TXTH_Pos)				/*!< Transmit FIFO threshold at 15. */

/* SPI_I2S Interrupt Mask */
#define SPI_I2S_TXTH_INT_MASK  		(SPI_FIFOCTL_TXTHIEN_Msk)				/*!< TX FIFO threshold interrupt mask */
#define SPI_I2S_RXTH_INT_MASK       (SPI_FIFOCTL_RXTHIEN_Msk)       		/*!< RX FIFO threshold interrupt mask */
#define SPI_I2S_RXOV_INT_MASK       (SPI_FIFOCTL_RXOVIEN_Msk)				/*!< RX FIFO overrun interrupt mask */
#define SPI_I2S_TXUF_INT_MASK       (SPI_FIFOCTL_TXUFIEN_Msk)         		/*!< TX FIFO underflow interrupt mask */
#define SPI_I2S_RXTO_INT_MASK      	(SPI_FIFOCTL_RXTOIEN_Msk)          		/*!< RX FIFO time-out interrupt mask */

/* SPI_I2S Status/Interrupt Flag Mask */
#define SPI_I2S_FLAG_CHANNL_LR 		(SPI_I2SSTS_RIGHT_Msk)					/*!< Channel left or right flag */
#define SPI_I2S_FLAG_RX_EMPTY 		(SPI_I2SSTS_RXEMPTY_Msk)				/*!< RX FIFO empty flag */
#define SPI_I2S_FLAG_RX_FULL 		(SPI_I2SSTS_RXFULL_Msk)					/*!< RX FIFO full flag */
#define SPI_I2S_FLAG_RX_TH	 		(SPI_I2SSTS_RXTHIF_Msk)					/*!< RX FIFO threshold flag */
#define SPI_I2S_FLAG_RX_OV	 		(SPI_I2SSTS_RXOVIF_Msk)					/*!< RX FIFO overrun flag */
#define SPI_I2S_FLAG_RX_TO	 		(SPI_I2SSTS_RXTOIF_Msk)					/*!< RX receive time-out flag */
#define SPI_I2S_FLAG_I2SEN			(SPI_I2SSTS_I2SENSTS_Msk)				/*!< I2S control enable flag */
#define SPI_I2S_FLAG_TX_EMPTY 		(SPI_I2SSTS_TXEMPTY_Msk)				/*!< RX FIFO empty flag */
#define SPI_I2S_FLAG_TX_FULL 		(SPI_I2SSTS_TXFULL_Msk)					/*!< RX FIFO full flag */
#define SPI_I2S_FLAG_TX_TH	 		(SPI_I2SSTS_TXTHIF_Msk)					/*!< RX FIFO threshold flag */
#define SPI_I2S_FLAG_TX_OV	 		(SPI_I2SSTS_TXUFIF_Msk)					/*!< RX FIFO overrun flag */
#define SPI_I2S_FLAG_RZC_INT 		(SPI_I2SSTS_RZCIF_Msk)					/*!< Right channel zero cross iterrupt flag */
#define SPI_I2S_FLAG_LZC_INT 		(SPI_I2SSTS_LZCIF_Msk)					/*!< Left channel zero cross iterrupt flag */
#define SPI_I2S_FLAG_TXRXRST		(SPI_I2SSTS_TXRXRST_Msk)				/*!< TX and RX reset flag */

/*@}*/ /* end of group I94100_SPI_I2S_EXPORTED_CONSTANTS */

/** @addtogroup I94100_SPI_I2S_EXPORTED_CONSTANTS SPI_I2S Exported Functions
  @{
*/

/**
  * @brief  	Enable SPI_I2S TX function.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	This macro will set TXEN bit of SPI_I2SCTL register to enable SPI_I2S TX function.
  */
#define SPI_I2S_ENABLE_TX(spi) 				((spi)->I2SCTL |= SPI_I2SCTL_TXEN_Msk )

/**
  * @brief  	Disable SPI_I2S TX function.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	This macro will clear TXEN bit of SPI_I2SCTL register to disable SPI_I2S TX function.
  */
#define SPI_I2S_DISABLE_TX(spi) 			((spi)->I2SCTL &= ~SPI_I2SCTL_TXEN_Msk )

/**
  * @brief  	Enable SPI_I2S RX function.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	This macro will set RXEN bit of SPI_I2SCTL register to enable SPI_I2S RX function.
  */
#define SPI_I2S_ENABLE_RX(spi) 				((spi)->I2SCTL |= SPI_I2SCTL_RXEN_Msk )

/**
  * @brief  	Disable SPI_I2S RX function.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	This macro will clear RXEN bit of SPI_I2SCTL register to disable SPI_I2S RX function.
  */
#define SPI_I2S_DISABLE_RX(spi) 			((spi)->I2SCTL &= ~SPI_I2SCTL_RXEN_Msk )

/**
  * @brief  	Enable TX Mute function.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	This macro will set MUTE bit of SPI_I2SCTL register to enable SPI_I2S TX mute function.
  */
#define SPI_I2S_ENABLE_TX_MUTE(spi)  		((spi)->I2SCTL |= SPI_I2SCTL_MUTE_Msk )

/**
  * @brief  	Disable TX Mute function.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	This macro will clear MUTE bit of SPI_I2SCTL register to disable SPI_I2S TX mute function.
  */
#define SPI_I2S_DISABLE_TX_MUTE(spi) 		((spi)->I2SCTL &= ~SPI_I2SCTL_MUTE_Msk )

/**
  * @brief  	Set SPI_I2S Stereo Data Order in FIFO.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @param[in] 	u32Order Even channel data at high or low byte.
  *             - \ref SPI_I2SORDER_HIGH
  *             - \ref SPI_I2SORDER_LOW
  * @return 	None
  */
#define	SPI_I2S_SET_STEREOORDER(spi, u32Order)		((spi)->I2SCTL = ((spi)->I2SCTL & (~SPI_I2SCTL_ORDER_Msk)) | u32Order)

/**
  * @brief  	Enable SPI_I2S Force Right Channel Zero Cross Data.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @return 	None
  * @detail		If this bit is set to 1, when right channel data sign bit change or 
  *				next shift data bits are all 0 then RZCIF flag in SPIn_I2SSTS register 
  *				is set to 1 and right channel data will force zero. This function is only available in transmit operation.
  */
#define	SPI_I2S_ENABLE_FRZCDEN(spi)			((spi)->I2SCTL |= SPI_I2SCTL_RZCEN_Msk)

/**
  * @brief  	Disable SPI_I2S Force Right Channel Zero Cross Data.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @return 	None
  * @detail		If this bit is set to 1, when right channel data sign bit change or 
  *				next shift data bits are all 0 then RZCIF flag in SPIn_I2SSTS register 
  *				is set to 1 and right channel data will force zero. This function is only available in transmit operation.
  */
#define	SPI_I2S_DISABLE_FRZCDEN(spi)		((spi)->I2SCTL |= SPI_I2SCTL_RZCEN_Msk)

/**
  * @brief  	Enable SPI_I2S Force Left Channel Zero Cross Data.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @return 	None
  * @detail		If this bit is set to 1, when left channel data sign bit changes or 
  *				next shift data bits are all 0 then LZCIF flag in SPIn_I2SSTS register 
  *				is set to 1 and left channel data will force zero. This function is only available in transmit operation.
  */
#define	SPI_I2S_ENABLE_FLZCDEN(spi)			((spi)->I2SCTL |= SPI_I2SCTL_LZCEN_Msk)

/**
  * @brief  	Disable SPI_I2S Force Left Channel Zero Cross Data.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @return 	None
  * @detail		If this bit is set to 1, when right channel data sign bit change or 
  *				next shift data bits are all 0 then RZCIF flag in SPIn_I2SSTS register 
  *				is set to 1 and right channel data will force zero. This function is only available in transmit operation.
  */
#define	SPI_I2S_DISABLE_FLZCDEN(spi)		((spi)->I2SCTL &= ~SPI_I2SCTL_LZCEN_Msk)

/**
  * @brief  	This function sets the recording source channel when mono mode is used.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @param[in] 	u32Ch left or right channel. Valid values are:
  *             - \ref SPI_I2SMONO_LEFT
  *             - \ref SPI_I2SMONO_RIGHT
  * @return None
  * @details 	When monaural format is selected (MONO = 1), I2S controller will receive 
  *				right channel data if RXLCH is set to 0, and receive left channel data if RXLCH is set to 1.
  */
static __INLINE void SPI_I2S_SET_MONO_RX_CHANNEL(SPI_T *spi, uint32_t u32Ch)
{
    u32Ch == SPI_I2SMONO_LEFT ?
    (spi->I2SCTL |= SPI_I2SCTL_RXLCH_Msk) :
    (spi->I2SCTL &= ~SPI_I2SCTL_RXLCH_Msk);
}

/**
  * @brief  	Enable SPI_I2S Right Channel Zero Cross Interrupt.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @return 	None
  * @detail		Interrupt occurs if this bit is set to 1 and right channel zero cross event occurs.
  */
#define SPI_I2S_ENABLE_RZCINT(spi)			((spi)->I2SCTL |= SPI_I2SCTL_RZCIEN_Msk)

/**
  * @brief  	Disable SPI_I2S Right Channel Zero Cross Interrupt.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @return 	None
  * @detail		Interrupt occurs if this bit is set to 1 and right channel zero cross event occurs.
  */
#define SPI_I2S_DISABLE_RZCINT(spi)			((spi)->I2SCTL |= SPI_I2SCTL_RZCIEN_Msk)

/**
  * @brief  	Enable SPI_I2S Left Channel Zero Cross Interrupt.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @return 	None
  * @detail		Interrupt occurs if this bit is set to 1 and left channel zero cross event occurs.
  */
#define SPI_I2S_ENABLE_LZCINT(spi)			((spi)->I2SCTL |= SPI_I2SCTL_LZCIEN_Msk)

/**
  * @brief  	Disable SPI_I2S Left Channel Zero Cross Interrupt.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @return 	None
  * @detail		Interrupt occurs if this bit is set to 1 and left channel zero cross event occurs.
  */
#define SPI_I2S_DISABLE_LZCINT(spi)			((spi)->I2SCTL |= SPI_I2SCTL_LZCIEN_Msk)

/**
  * @brief  	Get the interrupt flag.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @param[in] 	u32Mask The mask value for all interrupt flags.
  *				- \ref SPI_I2S_FLAG_CHANNL_LR 		
  *				- \ref SPI_I2S_FLAG_RX_EMPTY 		
  *				- \ref SPI_I2S_FLAG_RX_FULL 		
  *				- \ref SPI_I2S_FLAG_RX_TH	 		
  *				- \ref SPI_I2S_FLAG_RX_OV	 		
  *				- \ref SPI_I2S_FLAG_RX_TO	 		
  *				- \ref SPI_I2S_FLAG_I2SEN			
  *				- \ref SPI_I2S_FLAG_TX_EMPTY 		
  *				- \ref SPI_I2S_FLAG_TX_FULL 		
  *				- \ref SPI_I2S_FLAG_TX_TH	 		
  *				- \ref SPI_I2S_FLAG_TX_OV	 		
  *				- \ref SPI_I2S_FLAG_RZC_INT 		
  *				- \ref SPI_I2S_FLAG_LZC_INT 		
  *				- \ref SPI_I2S_FLAG_TXRXRST
  * @return 	The interrupt flags specified by the u32mask parameter.
  * @details 	This macro will return the combination interrupt flags of SPI_I2SSTS register. The flags are specified by the u32mask parameter.
  */
#define SPI_I2S_GET_FLAG(spi, u32Mask) 		((spi)->I2SSTS & (u32Mask))

/**
  * @brief  	Clear the interrupt flag.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @param[in] 	u32Mask The mask value for all interrupt flags.
  *				- \ref  SPI_I2S_FLAG_CHANNL_LR 		
  *				- \ref SPI_I2S_FLAG_RX_EMPTY 		
  *				- \ref SPI_I2S_FLAG_RX_FULL 		
  *				- \ref SPI_I2S_FLAG_RX_TH	 		
  *				- \ref SPI_I2S_FLAG_RX_OV	 		
  *				- \ref SPI_I2S_FLAG_RX_TO	 		
  *				- \ref SPI_I2S_FLAG_I2SEN			
  *				- \ref SPI_I2S_FLAG_TX_EMPTY 		
  *				- \ref SPI_I2S_FLAG_TX_FULL 		
  *				- \ref SPI_I2S_FLAG_TX_TH	 		
  *				- \ref SPI_I2S_FLAG_TX_OV	 		
  *				- \ref SPI_I2S_FLAG_RZC_INT 		
  *				- \ref SPI_I2S_FLAG_LZC_INT 		
  *				- \ref SPI_I2S_FLAG_TXRXRST
  * @return 	None
  * @details 	This macro will clear the interrupt flags specified by the u32mask parameter.
  * @note 		Except TX and RX FIFO threshold interrupt flags, the other interrupt flags can be cleared by writing 1 to itself.
  */
#define SPI_I2S_CLR_INT_FLAG(spi, u32Mask) 	((spi)->I2SSTS = (u32Mask))

/**
  * @brief  	Get transmit FIFO level
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	TX FIFO level
  * @details 	This macro will return the number of available words in TX FIFO.
  */
#define SPI_I2S_GET_TX_FIFO_LEVEL(spi)		(((spi)->I2SSTS & SPI_I2SSTS_TXCNT_Msk) >> SPI_I2SSTS_TXCNT_Pos  )

/**
  * @brief  	Get receive FIFO level
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	RX FIFO level
  * @details 	This macro will return the number of available words in RX FIFO.
  */
#define SPI_I2S_GET_RX_FIFO_LEVEL(spi) 		(((spi)->I2SSTS & SPI_I2SSTS_RXCNT_Msk) >> SPI_I2SSTS_RXCNT_Pos )

/**
  * @brief  	Get TX is empty or not
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	TX is empty or not
  */
#define SPI_I2S_IS_TX_EMPTY(spi) 			(((spi)->I2SSTS & SPI_I2SSTS_TXEMPTY_Msk)?TRUE:FALSE )

/**
  * @brief  	Get TX is full or not
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	TX is full or not
  */
#define SPI_I2S_IS_TX_FULL(spi) 			(((spi)->I2SSTS & SPI_I2SSTS_TXFULL_Msk)?TRUE:FALSE )

/**
  * @brief  	Get RX is empty or not
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	Rx is empty or not
  */
#define SPI_I2S_IS_RX_EMPTY(spi) 			(((spi)->I2SSTS & SPI_I2SSTS_RXEMPTY_Msk)?TRUE:FALSE )

/**
  * @brief  	Get TX is full or not
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	TX is full or not
  */
#define SPI_I2S_IS_RX_FULL(spi) 			(((spi)->I2SSTS & SPI_I2SSTS_RXFULL_Msk)?TRUE:FALSE )

/**
  * @brief  	Enable SPI_I2S TX DMA function.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	This macro will set TXPDMAEN bit of SPI_PDMACTL register to transmit data with PDMA.
  */
#define SPI_I2S_ENABLE_TXDMA(spi)  			((spi)->PDMACTL |= SPI_PDMACTL_TXPDMAEN_Msk )

/**
  * @brief  	Disable SPI_I2S TX DMA function.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	This macro will clear TXPDMAEN bit of SPI_PDMACTL register to disable TX DMA function.
  */
#define SPI_I2S_DISABLE_TXDMA(spi) 			((spi)->PDMACTL &= ~SPI_PDMACTL_TXPDMAEN_Msk )

/**
  * @brief  	Enable SPI_I2S RX DMA function.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	This macro will set RXPDMAEN bit of SPI_PDMACTL register to receive data with PDMA.
  */
#define SPI_I2S_ENABLE_RXDMA(spi) 			((spi)->PDMACTL |= SPI_PDMACTL_RXPDMAEN_Msk )

/**
  * @brief  Disable SPI_I2S RX DMA function.
  * @param[in] spi The pointer of the specified SPI_I2S module.
  * @return None
  * @details This macro will clear RXPDMAEN bit of SPI_PDMACTL register to disable RX DMA function.
  */
#define SPI_I2S_DISABLE_RXDMA(spi) 			((spi)->PDMACTL &= ~SPI_PDMACTL_RXPDMAEN_Msk )

/**
  * @brief  	Reset RX FIFO.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	Reset receive FIFO pointer and receive circuit. 
  *				The RXFULL bit will be cleared to 0 and the RXEMPTY bit will be set to 1.
  *				This bit will be cleared to 0 by hardware about 3 system clock cycles + 
  *				2 peripheral clock cycles after it is set to 1.
  */
#define SPI_I2S_RST_RX_FIFO(spi) 			((spi)->FIFOCTL |= SPI_FIFOCTL_RXRST_Msk )

/**
  * @brief  	Reset TX FIFO.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	Reset transmit FIFO pointer and transmit circuit. 
  *				The TXEMPTY bit will be cleared to 0.
  *				This bit will be cleared to 0 by hardware about 3 system clock cycles + 
  *				2 peripheral clock cycles after it is set to 1.
  */
#define SPI_I2S_RST_TX_FIFO(spi) 			((spi)->FIFOCTL |= SPI_FIFOCTL_TXRST_Msk )

/**
  * @brief  	Clear RX FIFO.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	This macro will clear RX FIFO. The internal RX FIFO pointer will be reset to FIFO start point.
  */
#define SPI_I2S_CLR_RX_FIFO(spi) 		((spi)->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk )

/**
  * @brief  	Clear TX FIFO.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	None
  * @details 	This macro will clear TX FIFO. The internal TX FIFO pointer will be reset to FIFO start point.
  */
#define SPI_I2S_CLR_TX_FIFO(spi) 		((spi)->FIFOCTL |= SPI_FIFOCTL_TXFBCLR_Msk )

/**
  * @brief 		Set RX treshold.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @param[in] 	u32Treshold The Transmit FIFO Threshold Level.
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_0
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_1
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_2
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_3
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_4
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_5
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_6
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_7
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_8
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_9
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_10
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_11
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_12
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_13
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_14
  *            	- \ref SPI_I2S_FIFO_RX_LEVEL_15
  * @return 	None
  * @details 	Note: If remain data word number in transmit FIFO is the same or less than threshold level then TXTHIF (SPI_I2SSTS) flag is set.
  */
#define SPI_I2S_SET_RXTH(spi, u32Treshold)		((spi)->FIFOCTL = (spi)->FIFOCTL & (~SPI_FIFOCTL_RXTH_Msk) | u32Treshold)

/**
  * @brief 		Set TX treshold.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @param[in] 	u32Treshold The Receive FIFO Threshold Level.
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_0
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_1
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_2
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_3
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_4
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_5
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_6
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_7
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_8
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_9
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_10
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_11
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_12
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_13
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_14
  *            	- \ref SPI_I2S_FIFO_TX_LEVEL_15
  * @return 	None
  * @details 	When received data word number in receive buffer is greater than threshold level then RXTHIF (SPI_I2SSTS) flag is set.
  */
#define SPI_I2S_SET_TXTH(spi, u32Treshold)		((spi)->FIFOCTL = (spi)->FIFOCTL & (~SPI_FIFOCTL_TXTH_Msk) | u32Treshold)

/**
  * @brief  	Write data to SPI_I2S TX FIFO.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @param[in] 	u32Data The value written to TX FIFO.
  * @return 	None
  * @details 	This macro will write a value to TX FIFO.
  */
#define SPI_I2S_WRITE_TX_FIFO(spi, u32Data)		((spi)->TX = (u32Data))

/**
  * @brief  	Read RX FIFO.
  * @param[in] 	spi The pointer of the specified SPI_I2S module.
  * @return 	The value read from RX FIFO.
  * @details 	This function will return a value read from RX FIFO.
  */
#define SPI_I2S_READ_RX_FIFO(spi) 				((spi)->RX)

/* SPI_I2S Function prototype declaration */
uint32_t SPI_I2SOpen(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SampleRate, uint32_t u32WordWidth, uint32_t u32Mono, uint32_t u32DataFormat);
void SPI_I2SClose(SPI_T *spi);
void SPI_I2SEnableInt(SPI_T *spi, uint32_t u32Mask);
void SPI_I2SDisableInt(SPI_T *spi, uint32_t u32Mask);
uint32_t SPI_I2SEnableMCLK(SPI_T *spi, uint32_t u32MasterClock);
void SPI_I2SDisableMCLK(SPI_T *spi);
void SPI_I2SEnableControl(SPI_T *spi);
void SPI_I2SDisableControl(SPI_T *spi);
/*@}*/ /* end of group I94100_SPI_I2S_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_SPI_Driver */

/*@}*/ /* end of group I94100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SPI_H__

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
