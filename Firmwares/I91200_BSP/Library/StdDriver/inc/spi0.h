/****************************************************************************//**
 * @file     spi0.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/05/12 10:10a $
 * @brief    I91200 SPI0 driver header file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __SPI0_H__
#define __SPI0_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_SPI0_Driver SPI0 Driver
  @{
*/

/** @addtogroup I91200_SPI0_EXPORTED_CONSTANTS SPI0 Exported Constants
  @{
*/

#define SPI0_MODE_0            (SPI0_CTL_TXNEG_Msk)                        /*!< CLKP=0; RX_NEG=0; TX_NEG=1 \hideinitializer */
#define SPI0_MODE_1            (SPI0_CTL_RXNEG_Msk)                        /*!< CLKP=0; RX_NEG=1; TX_NEG=0 \hideinitializer */
#define SPI0_MODE_2            (SPI0_CTL_CLKPOL_Msk | SPI0_CTL_TXNEG_Msk)  /*!< CLKP=1; RX_NEG=1; TX_NEG=0 \hideinitializer */
#define SPI0_MODE_3            (SPI0_CTL_CLKPOL_Msk | SPI0_CTL_RXNEG_Msk)  /*!< CLKP=1; RX_NEG=0; TX_NEG=1 \hideinitializer */

#define SPI0_SLAVE             (SPI0_CTL_SLAVE_Msk)                         /*!< Set as slave \hideinitializer */
#define SPI0_MASTER            (0x0)                                       /*!< Set as master \hideinitializer */

#define SPI0_SS0               (0x1)                                       /*!< Set SS0 \hideinitializer */
#define SPI0_SS1               (0x2)                                       /*!< Set SS1 \hideinitializer */
#define SPI0_SS_ACTIVE_HIGH    (SPI_SSCTL_SSACTPOL_Msk)                    /*!< SS active high \hideinitializer */
#define SPI0_SS_ACTIVE_LOW     (0x0)                                       /*!< SS active low \hideinitializer */

#define SPI0_UNITIEN_MASK      (0x001)                                     /*!< Interrupt enable mask \hideinitializer */
#define SPI0_SSINAIEN_MASK     (0x002)                                     /*!< Slave Slave Inactive interrupt enable mask \hideinitializer */
#define SPI0_SSACTIEN_MASK     (0x004)                                     /*!< Slave Slave Active interrupt enable mask \hideinitializer */
#define SPI0_SLVURIEN_MASK     (0x008)                                     /*!< Slave Mode Error 1 interrupt enable mask \hideinitializer */
#define SPI0_SLVBEIEN_MASK     (0x010)                                     /*!< Slave Mode Error 0 interrupt enable mask \hideinitializer */
#define SPI0_SLVTOIEN_MASK     (0x020)                                     /*!< Slave Mode Time-out interrupt enable mask \hideinitializer */
#define SPI0_FIFO_TXTHIEN_MASK (0x040)                                     /*!< Transmit FIFO Threshold interrupt enable mask \hideinitializer */
#define SPI0_FIFO_RXTHIEN_MASK (0x080)                                     /*!< Receive FIFO Threshold interrupt enable mask \hideinitializer */
#define SPI0_FIFO_RXOVIEN_MASK (0x100)                                     /*!< Receive FIFO Overrun interrupt enable mask \hideinitializer */
#define SPI0_FIFO_TXUFIEN_MASK (0x200)                                     /*!< Slave Transmit Under Run interrupt enable mask \hideinitializer */
#define SPI0_FIFO_RXTOIEN_MASK (0x400)                                     /*!< Slave Receive Time-out interrupt enable mask \hideinitializer */

/*@}*/ /* end of group I91200_SPI0_EXPORTED_CONSTANTS */


/** @addtogroup I91200_SPI0_EXPORTED_FUNCTIONS SPI0 Exported Functions
  @{
*/

/**
  * @brief  Set time out period for slave.
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI0 module.
  * @param  u32TimeoutPeriod is the period of time out.
  * @return none
  * \hideinitializer
  */
#define SPI0_SET_SLAVE_TIMEOUT_PERIOD(spi, u32TimeoutPeriod) ( (spi)->SSCTL = ((spi)->SSCTL & ~SPI0_SSCTL_SLVTOCNT_Msk) | (u32TimeoutPeriod & 0xFFFF) )

/**
  * @brief  Enable time out clear function for FIFO mode.
  * @param  spi is the base address of SPI0 module.
  * @return none
  * \hideinitializer
  */
#define SPI0_ENABLE_TIMEOUT_FIFO_CLEAR(spi) ( (spi)->SSCTL |= SPI0_SSCTL_SLVTORST_Msk )

/**
  * @brief  Disable time out clear function for FIFO mode.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_DISABLE_TIMEOUT_FIFO_CLEAR(spi) ( (spi)->SSCTL &= ~SPI0_SSCTL_SLVTORST_Msk )

/**
  * @brief  Set data out signal to low (0) if transmit under-run occurs.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_SET_TX_UNDERRUN_DATA_LOW(spi) ( (spi)->FIFOCTL &= ~SPI0_FIFOCTL_TXUFPOL_Msk )

/**
  * @brief  Set data out signal to high (1) if transmit under-run occurs.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_SET_TX_UNDERRUN_DATA_HIGH(spi) ( (spi)->FIFOCTL |= SPI0_FIFOCTL_TXUFPOL_Msk )

/**
  * @brief  Get the status flags.
  * @param  spi is the base address of SPI module.
  * @return status flags
  * \hideinitializer
  */
#define SPI0_GET_STATUS(spi) ( (spi)->STATUS )

/**
  * @brief  Clear the unit transfer interrupt flag.
  * @param  spi is the base address of SPI0 module.
  * @return none
  * \hideinitializer
  */
#define SPI0_CLR_UNIT_TRANS_INT_FLAG(spi) ( (spi)->STATUS |= SPI0_STATUS_UNITIF_Msk )

/**
  * @brief  Disable slave 3-wire mode.
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI0 module.
  * @return none
  * \hideinitializer
  */
#define SPI_DISABLE_3WIRE_MODE(spi) ( (spi)->SSCTL &= ~SPI_SSCTL_SLV3WIRE_Msk )

/**
  * @brief  Enable slave 3-wire mode.
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI0 module.
  * @return none
  * \hideinitializer
  */
#define SPI0_ENABLE_3WIRE_MODE(spi) ( (spi)->SSCTL |= SPI0_SSCTL_SLV3WIRE_Msk )

/**
  * @brief  Get the count of available data in RX FIFO.
  * @param  spi is the base address of SPI module.
  * @return The count of available data in RX FIFO.
  * \hideinitializer
  */
#define SPI0_GET_RX_FIFO_COUNT(spi) ( (((spi)->STATUS & SPI0_STATUS_RXCNT_Msk) >> SPI0_STATUS_RXCNT_Pos) & 0xf )

/**
  * @brief  Get the Rx FIFO empty flag.
  * @param  spi is the base address of SPI module.
  * @return Rx FIFO flag
  * @retval 0: Rx FIFO is not empty
  * @retval 1: Rx FIFO is empty
  * \hideinitializer
  */
#define SPI0_GET_RX_FIFO_EMPTY_FLAG(spi) ( ((spi)->STATUS & SPI0_STATUS_RXEMPTY_Msk) == SPI0_STATUS_RXEMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Tx FIFO empty flag.
  * @param  spi is the base address of SPI module.
  * @return Tx FIFO flag
  * @retval 0: Tx FIFO is not empty
  * @retval 1: Tx FIFO is empty
  * \hideinitializer
  */
#define SPI0_GET_TX_FIFO_EMPTY_FLAG(spi) ( ((spi)->STATUS & SPI0_STATUS_TXEMPTY_Msk) == SPI0_STATUS_TXEMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Tx FIFO full flag.
  * @param  spi is the base address of SPI module.
  * @return Tx FIFO flag
  * @retval 0: Tx FIFO is not full
  * @retval 1: Tx FIFO is full
  * \hideinitializer
  */
#define SPI0_GET_TX_FIFO_FULL_FLAG(spi) ( ((spi)->STATUS & SPI_STATUS_TXFULL_Msk) == SPI_STATUS_TXFULL_Msk ? 1:0 )

/**
  * @brief  Get the datum read from R0 FIFO.
  * @param  spi is the base address of SPI module.
  * @return data in Rx register
  * \hideinitializer
  */
#define SPI0_READ_RX(spi) ((spi)->RX)

/**
  * @brief  Write datum to TX register.
  * @param  spi is the base address of SPI module.
  * @param  u32TxData is the datum which user attempt to transfer through SPI bus.
  * @return none
  * \hideinitializer
  */
#define SPI0_WRITE_TX(spi, u32TxData) ( (spi)->TX = u32TxData )

/**
  * @brief  Disable automatic slave select function and set SPI_SS pin to high state.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
static __INLINE void SPI0_SET_SS0_HIGH(SPI0_T *spi)
{
    spi->SSCTL &= ~SPI0_SSCTL_AUTOSS_Msk;
    spi->SSCTL |= SPI0_SSCTL_SSACTPOL_Msk;
    spi->SSCTL = (spi->SSCTL & ~SPI0_SSCTL_SS_Msk) | SPI0_SS0;
}

/**
  * @brief  Disable automatic slave select function and set SPI_SS pin to low state.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
static __INLINE void SPI0_SET_SS0_LOW(SPI0_T *spi)
{
    spi->SSCTL &= ~SPI0_SSCTL_AUTOSS_Msk;
    spi->SSCTL &= ~SPI0_SSCTL_SSACTPOL_Msk;
    spi->SSCTL = (spi->SSCTL & ~SPI0_SSCTL_SS_Msk) | SPI0_SS0;
}

/**
  * @brief  Disable automatic slave select function and set SPI_SS pin to high state.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
static __INLINE void SPI0_SET_SS1_HIGH(SPI0_T *spi)
{
    spi->SSCTL &= ~SPI0_SSCTL_AUTOSS_Msk;
    spi->SSCTL |= SPI0_SSCTL_SSACTPOL_Msk;
    spi->SSCTL = (spi->SSCTL & ~SPI0_SSCTL_SS_Msk) | SPI0_SS1;
}

/**
  * @brief  Disable automatic slave select function and set SPI_SS pin to low state.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
static __INLINE void SPI0_SET_SS1_LOW(SPI0_T *spi)
{
    spi->SSCTL &= ~SPI0_SSCTL_AUTOSS_Msk;
    spi->SSCTL |= SPI0_SSCTL_SSACTPOL_Msk;
    spi->SSCTL = (spi->SSCTL & ~SPI0_SSCTL_SS_Msk) | SPI0_SS1;
}

/**
  * @brief Enable byte reorder function.
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI0 module.
  * @return none
  * \hideinitializer
  */
#define SPI0_ENABLE_BYTE_REORDER(spi) ( (spi)->CTL |= SPI0_CTL_REORDER_Msk )

/**
  * @brief  Disable byte reorder function.
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_DISABLE_BYTE_REORDER(spi) ( (spi)->CTL &= ~SPI0_CTL_REORDER_Msk )

/**
  * @brief  Set the length of suspend interval.
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI0 module.
  * @param  u32SuspCycle decides the length of suspend interval.
  * @return none
  * \hideinitializer
  */
#define SPI0_SET_SUSPEND_CYCLE(spi, u32SuspCycle) ( (spi)->CTL = ((spi)->CTL & ~SPI0_CTL_SUSPITV_Msk) | (u32SuspCycle << SPI0_CTL_SUSPITV_Pos) )

/**
  * @brief  Set the SPI transfer sequence with LSB first.
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_SET_LSB_FIRST(spi) ( (spi)->CTL |= SPI0_CTL_LSB_Msk )

/**
  * @brief  Set the SPI transfer sequence with MSB first.
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_SET_MSB_FIRST(spi) ( (spi)->CTL &= ~SPI0_CTL_LSB_Msk )

/**
  * @brief  Set the data width of a SPI transaction.
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI0 module.
  * @param  u32Width data width
  * @return none
  * \hideinitializer
  */
static __INLINE void SPI0_SET_DATA_WIDTH(SPI0_T *spi, uint32_t u32Width)
{
    spi->CTL = (spi->CTL & ~SPI0_CTL_DWIDTH_Msk) | (((u32Width>=32)?0:u32Width) << SPI0_CTL_DWIDTH_Pos);
}

/**
  * @brief  Get the SPI busy state.
  * @param  spi is the base address of SPI module.
  * @return SPI busy status
  * @retval 0: SPI module is not busy
  * @retval 1: SPI module is busy
  * \hideinitializer
  */
#define SPI0_IS_BUSY(spi) ( ((spi)->STATUS & SPI0_STATUS_BUSY_Msk) == SPI0_STATUS_BUSY_Msk ? 1:0 )

/**
  * @brief  Set the SPIEN bit to trigger SPI transfer.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_TRIGGER(spi) ( (spi)->CTL |= SPI0_CTL_SPIEN_Msk )

/**
  * @brief  Disable SPI function.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_DISABLE(spi) (  (spi)->CTL &= ~SPI0_CTL_SPIEN_Msk )

/**
  * @brief  Enable SPI Dual IO function.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_ENABLE_DUAL_MODE(spi) ( (spi)->CTL |= SPI0_CTL_DUALIOEN_Msk )

/**
  * @brief  Disable SPI Dual IO function.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_DISABLE_DUAL_MODE(spi) ( (spi)->CTL &= ~SPI0_CTL_DUALIOEN_Msk )

/**
  * @brief  Set SPI Dual IO direction to input.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_ENABLE_DUAL_INPUT_MODE(spi) ( (spi)->CTL &= ~SPI0_CTL_QDIODIR_Msk )

/**
  * @brief  Set SPI Dual IO direction to output.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_ENABLE_DUAL_OUTPUT_MODE(spi) ( (spi)->CTL |= SPI0_CTL_QDIODIR_Msk )

/**
  * @brief  Enable SPI QUAD IO function.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_ENABLE_QUAD_MODE(spi) ( (spi)->CTL |= SPI0_CTL_QUADIOEN_Msk )

/**
  * @brief  Disable SPI Dual IO function.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_DISABLE_QUAD_MODE(spi) ( (spi)->CTL &= ~SPI0_CTL_QUADIOEN_Msk )

/**
  * @brief  Set SPI Quad IO direction to input.
  * @param  spi is the base address of SPI0 module.
  * @return none
  * \hideinitializer
  */
#define SPI0_ENABLE_QUAD_INPUT_MODE(spi) ( (spi)->CTL &= ~SPI0_CTL_QDIODIR_Msk )

/**
  * @brief  Set SPI Quad IO direction to output.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_ENABLE_QUAD_OUTPUT_MODE(spi) ( (spi)->CTL |= SPI0_CTL_QDIODIR_Msk )

/**
  * @brief  Trigger RX PDMA transfer.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_TRIGGER_RX_PDMA(spi) ( (spi)->PDMACTL |= SPI0_PDMACTL_RXPDMAEN_Msk )

/**
  * @brief  Trigger TX PDMA transfer.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_TRIGGER_TX_PDMA(spi) ( (spi)->PDMACTL |= SPI0_PDMACTL_TXPDMAEN_Msk )

/**
  * @brief  Disable RX PDMA transfer.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_DISABLE_RX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI0_PDMACTL_RXPDMAEN_Msk )

/**
  * @brief  Trigger TX PDMA transfer.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_DISABLE_TX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI0_PDMACTL_TXPDMAEN_Msk )

/**
  * @brief  Enable 2-bit transfer mode.
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI0 module.
  * @return none
  * \hideinitializer
  */
#define SPI0_ENABLE_2BIT_MODE(spi) ( (spi)->CTL |= SPI0_CTL_TWOBIT_Msk )

/**
  * @brief  Disable 2-bit transfer mode.
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI0 module.
  * @return none
  * \hideinitializer
  */
#define SPI0_DISABLE_2BIT_MODE(spi) ( (spi)->CTL &= ~SPI0_CTL_TWOBIT_Msk )

/**
  * @brief  Enable the FIFO receive mode    
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI0 module.
  * @return none
  * \hideinitializer
  */
#define SPI0_ENABLE_RX_MODE(spi) ( (spi)->CTL |= SPI0_CTL_RXMODEEN_Msk )

/**
  * @brief  Disable the FIFO receive mode    
  * @note Before calling this function, SPI must be stopped first. \ref SPI0_DISABLE must be called.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI0_DISABLE_RX_MODE(spi) ( (spi)->CTL &= (~SPI0_CTL_RXMODEEN_Msk) )

/**
  * @brief  Set the receive transaction count           
  * @param  spi is the base address of SPI module.
  * @param  u32Count receive data(if u32Count == 0, diable).
  * @return none
  * \hideinitializer
  */
static __INLINE void SPI0_ENABLE_RX_TRABS_COUNT(SPI0_T *spi, uint32_t u32Count)
{
    if(u32Count == 0)
    {
        spi->CTL &= (~SPI0_CTL_RXTCNTEN_Msk);
	}
	else
    {
        spi->CTL |= SPI0_CTL_RXTCNTEN_Msk;
        spi->RXTSNCNT = u32Count;
    }
}

uint32_t SPI0_Open(SPI0_T *spi,uint32_t u32MasterSlave, uint32_t u32SPIMode,  uint32_t u32DataWidth, uint32_t u32BusClock);
void     SPI0_Close(SPI0_T *spi);
void     SPI0_ClearRxFIFO(SPI0_T *spi);
void     SPI0_ClearTxFIFO(SPI0_T *spi);
void     SPI0_DisableAutoSS(SPI0_T *spi);
void     SPI0_EnableAutoSS(SPI0_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel);
uint32_t SPI0_SetBusClock(SPI0_T *spi, uint32_t u32BusClock);
void     SPI0_SetFIFOThreshold(SPI0_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold);
uint32_t SPI0_GetBusClock(SPI0_T *spi);
void     SPI0_EnableInt(SPI0_T *spi, uint32_t u32Mask);
void     SPI0_DisableInt(SPI0_T *spi, uint32_t u32Mask);


/*@}*/ /* end of group I91200_SPI0_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_SPI0_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SPI_H__

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
