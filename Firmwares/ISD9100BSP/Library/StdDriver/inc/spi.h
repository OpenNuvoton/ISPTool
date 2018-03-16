/****************************************************************************//**
 * @file     spi.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/25 10:10a $
 * @brief    ISD9100 SPI driver header file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_SPI_Driver SPI Driver
  @{
*/

/** @addtogroup ISD9100_SPI_EXPORTED_CONSTANTS SPI Exported Constants
  @{
*/

#define SPI_MODE_0            (SPI_CTL_TXNEG_Msk)                         /*!< CLKP=0; RX_NEG=0; TX_NEG=1 \hideinitializer */
#define SPI_MODE_1            (SPI_CTL_RXNET_Msk)                         /*!< CLKP=0; RX_NEG=1; TX_NEG=0 \hideinitializer */
#define SPI_MODE_2            (SPI_CTL_CLKPOL_Msk | SPI_CTL_TXNEG_Msk)    /*!< CLKP=1; RX_NEG=1; TX_NEG=0 \hideinitializer */
#define SPI_MODE_3            (SPI_CTL_CLKPOL_Msk | SPI_CTL_RXNET_Msk)    /*!< CLKP=1; RX_NEG=0; TX_NEG=1 \hideinitializer */

#define SPI_SLAVE             (0x1ul << SPI_CTL_SLAVE_Pos)                /*!< Set as slave \hideinitializer */
#define SPI_MASTER            (0x0ul << SPI_CTL_SLAVE_Pos)                /*!< Set as master \hideinitializer */

#define SPI_SS_NONE           (0x0ul<<SPI_SSCTL_SS_Pos)                   /*!< unset any spi port \hideinitializer */
#define SPI_SS0               (0x1ul<<SPI_SSCTL_SS_Pos)                   /*!< Set SS0 \hideinitializer */
#define SPI_SS1               (0x2ul<<SPI_SSCTL_SS_Pos)                   /*!< Set SS1 \hideinitializer */

#define SPI_SS_ACTIVE_HIGH    (SPI_SSCTL_SSACTPOL_Msk)                    /*!< SS active high \hideinitializer */
#define SPI_SS_ACTIVE_LOW     (0x0)                                       /*!< SS active low \hideinitializer */

#define SPI_TXNUM_ONE         (0x00ul << SPI_CTL_TX_NUM_Pos)              /*!< Only one transmit/receive word will be executed in one transfer \hideinitializer */      
#define SPI_TXNUM_TWO         (0x01ul << SPI_CTL_TX_NUM_Pos)              /*!< Two successive transmit/receive word will be executed in one transfer \hideinitializer */      

/*@}*/ /* end of group ISD9100_SPI_EXPORTED_CONSTANTS */

/** @addtogroup ISD9100_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  Get the status flags.
  * @param  spi is the base address of SPI module.
  * @return status flags
  * \hideinitializer
  */
#define SPI_GET_STATUS(spi) ( (spi)->CTL )

/**
  * @brief  Clear the unit transfer interrupt flag.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_CLR_UNIT_TRANS_INT_FLAG(spi) ( (spi)->CTL |= SPI_CTL_UNIT_INTSTS_Msk )

/**
  * @brief  Get the Rx FIFO empty flag.
  * @param  spi is the base address of SPI module.
  * @return Rx FIFO flag
  * @retval 0: Rx FIFO is not empty
  * @retval 1: Rx FIFO is empty
  * \hideinitializer
  */
#define SPI_GET_RX_FIFO_EMPTY_FLAG(spi) ( ((spi)->CTL & SPI_CTL_RXEMPTY_Msk) == SPI_CTL_RXEMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Rx FIFO full flag.
  * @param  spi is the base address of SPI module.
  * @return Rx FIFO flag
  * @retval 0: Rx FIFO is not full
  * @retval 1: Rx FIFO is full
  * \hideinitializer
  */
#define SPI_GET_RX_FIFO_FULL_FLAG(spi) ( ((spi)->CTL & SPI_CTL_RXEMPTY_Msk) == SPI_CTL_RXEMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Tx FIFO empty flag.
  * @param  spi is the base address of SPI module.
  * @return Tx FIFO flag
  * @retval 0: Tx FIFO is not empty
  * @retval 1: Tx FIFO is empty
  * \hideinitializer
  */
#define SPI_GET_TX_FIFO_EMPTY_FLAG(spi) ( ((spi)->CTL & SPI_CTL_TXEMPTY_Msk) == SPI_CTL_TXEMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Tx FIFO full flag.
  * @param  spi is the base address of SPI module.
  * @return Tx FIFO flag
  * @retval 0: Tx FIFO is not full
  * @retval 1: Tx FIFO is full
  * \hideinitializer
  */
#define SPI_GET_TX_FIFO_FULL_FLAG(spi) ( ((spi)->CTL & SPI_CTL_TXFULL_Msk) == SPI_CTL_TXFULL_Msk ? 1:0 )

/**
  * @brief  Get the datum read from RX0 FIFO.
  * @param  spi is the base address of SPI module.
  * @return data in Rx0 register
  * \hideinitializer
  */
#define SPI_READ_RX0(spi) ((spi)->RX0)


/**
  * @brief  Get the datum read from RX1 FIFO.
  * @param  spi is the base address of SPI module.
  * @return data in Rx1 register
  * \hideinitializer
  */
#define SPI_READ_RX1(spi) ((spi)->RX1)

/**
  * @brief  Write datum to TX0 register.
  * @param  spi is the base address of SPI module.
  * @param  u32TxData is the datum which user attempt to transfer through SPI bus.
  * @return none
  * \hideinitializer
  */
#define SPI_WRITE_TX0(spi, u32TxData) ( (spi)->TX0 = u32TxData )

/**
  * @brief  Write datum to TX1 register.
  * @param  spi is the base address of SPI module.
  * @param  u32TxData is the datum which user attempt to transfer through SPI bus.
  * @return none
  * \hideinitializer
  */
#define SPI_WRITE_TX1(spi, u32TxData) ( (spi)->TX1 = u32TxData )

/**
  * @brief  Configure the slave select pins. 
  * @param  spi is the base address of SPI module.
  * @param  u32SS is the specified slave select pins will be set to active state.
  * @return none
  * \hideinitializer
  */
#define SPI_SET_SS(spi,u32SS) ( (spi)->SSCTL = ( (spi)->SSCTL & ~SPI_SSCTL_SS_Msk ) | u32SS )

/**
  * @brief  Configure the set slave active level. 
  * @param  spi is the base address of SPI module.
  * @param  u32Level is the specified slave active level.
  * @return none
  * \hideinitializer
  */
#define SPI_SET_SLAVE_ACTIVE_LEVEL(spi,u32Level) ( (spi)->SSCTL = ( (spi)->SSCTL & ~SPI_SSCTL_SS_LVL_Msk ) | u32Level )

/**
  * @brief Enable byte reorder function.
  * @note Before calling this function, SPI must be stopped first.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_ENABLE_BYTE_REORDER(spi) ( (spi)->CTL |= SPI_CTL_REORDER_Msk )

/**
  * @brief  Disable byte reorder function.
  * @note Before calling this function, SPI must be stopped first.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_DISABLE_BYTE_REORDER(spi) ( (spi)->CTL &= ~SPI_CTL_REORDER_Msk )

/**
  * @brief  Set the length of suspend interval.
  * @note Before calling this function, SPI must be stopped first.
  * @param  spi is the base address of SPI module.
  * @param  u32SuspCycle decides the length of suspend interval.
  * @return none
  * \hideinitializer
  */
#define SPI_SET_SUSPEND_CYCLE(spi, u32SuspCycle) ( (spi)->CTL = ((spi)->CTL & ~SPI_CTL_SUSPITV_Msk) | (u32SuspCycle << SPI_CTL_SUSPITV_Pos) )

/**
  * @brief  Set the SPI transfer sequence with LSB first.
  * @note Before calling this function, SPI must be stopped first.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_SET_LSB_FIRST(spi) ( (spi)->CTL |= SPI_CTL_LSB_Msk )

/**
  * @brief  Set the SPI transfer sequence with MSB first.
  * @note Before calling this function, SPI must be stopped first.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_SET_MSB_FIRST(spi) ( (spi)->CTL &= ~SPI_CTL_LSB_Msk )

/**
  * @brief  Set the data width of a SPI transaction.
  * @note Before calling this function, SPI must be stopped first.
  * @param  spi is the base address of SPI module.
  * @param  u32Width data width
  * @return none
  * \hideinitializer
  */
#define SPI_SET_DATA_WIDTH(spi,u32Width) ( (spi)->CTL = ((spi)->CTL & ~SPI_CTL_DWIDTH_Msk)|((u32Width == 32 ? 0:u32Width)<<SPI_CTL_DWIDTH_Pos) ) 

/**
  * @brief  Set the data count of a SPI transaction.
  * @param  spi is the base address of SPI module.
  * @param  u32TxNum is the transmit/receive word numbers.
  * @return none
  * \hideinitializer
  */
#define SPI_SET_TX_NUM(spi,u32TxNum)  ((spi)->CTL = ((spi)->CTL & ~SPI_CTL_TX_NUM_Msk ) | u32TxNum)

/**
  * @brief  Get the SPI busy state.
  * @param  spi is the base address of SPI module.
  * @return SPI busy status
  * @retval 0: SPI module is not busy
  * @retval 1: SPI module is busy
  * \hideinitializer
  */
#define SPI_IS_BUSY(spi) ( ((spi)->CTL & SPI_CTL_BUSY_Msk) == SPI_CTL_BUSY_Msk ? 1:0 )

/**
  * @brief  Set SPI to busy state.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_GO(spi) ((spi)->CTL |= SPI_CTL_BUSY_Msk)

/**
  * @brief  Enable SPI interrupt.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_ENABLE_INTERRUPT(spi)  ((spi)->CTL |= SPI_CTL_UNIT_INTEN_Msk)

/**
  * @brief  Disable SPI interrupt.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_DISABLE_INTERRUPT(spi)  ((spi)->CTL &= ~SPI_CTL_UNIT_INTEN_Msk)

/**
  * @brief  Enable the automatic slave select function. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @param  u32SSPin specifies slave select pins. Valid values are:
  *                     - \ref SPI_SS0
  *                     - \ref SPI_SS1
  * @param  u32ActiveLvl specifies the active level of slave select signal. Valid values are:
  *                     - \ref SPI_SS_ACTIVE_HIGH
  *                     - \ref SPI_SS_ACTIVE_LOW
  * @return none
  * \hideinitializer
  */
#define SPI_ENABLE_AUTOSS(spi,u32SSPin,u32ActiveLvl)  ((spi)->SSCTL = ((spi)->SSCTL&~(SPI_SSCTL_SS_Msk|SPI_SSCTL_SS_LVL_Msk))|(u32SSPin|u32ActiveLvl|SPI_SSCTL_AUTOSS_Msk))

/**
  * @brief  Disable the automatic slave select function. 
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_DISABLE_AUTOSS(spi)  ((spi)->SSCTL &= ~SPI_SSCTL_AUTOSS_Msk)

/**
  * @brief  Trigger RX PDMA transfer.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_TRIGGER_RX_PDMA(spi) ( (spi)->PDMACTL |= SPI_PDMACTL_RXPDMAEN_Msk )

/**
  * @brief  Trigger TX PDMA transfer.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_TRIGGER_TX_PDMA(spi) ( (spi)->PDMACTL |= SPI_PDMACTL_TXPDMAEN_Msk )

/**
  * @brief  Disable RX PDMA transfer.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_DISABLE_RX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI_PDMACTL_RXPDMAEN_Msk )

/**
  * @brief  Trigger TX PDMA transfer.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_DISABLE_TX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI_PDMACTL_TXPDMAEN_Msk )

/**
  * @brief  Enable 2-bit transfer mode.
  * @note Before calling this function, SPI must be stopped first.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_ENABLE_2BIT_MODE(spi) ( (spi)->CTL |= SPI_CTL_TWOBIT_Msk )

/**
  * @brief  Disable 2-bit transfer mode.
  * @note Before calling this function, SPI must be stopped first.
  * @param  spi is the base address of SPI module.
  * @return none
  * \hideinitializer
  */
#define SPI_DISABLE_2BIT_MODE(spi) ( (spi)->CTL &= ~SPI_CTL_TWOBIT_Msk )

uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32BusClock, uint32_t u32VarClock);
void     SPI_Close(SPI_T *spi);
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock);
void     SPI_SetVarClock(SPI_T *spi, uint32_t u32VarClock);
uint32_t SPI_GetBusClock(SPI_T *spi);
uint32_t SPI_GetVarClock(SPI_T *spi);

/*@}*/ /* end of group ISD9100_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_SPI_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SPI_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
