/****************************************************************************//**
 * @file     spi1.h
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/25 10:10a $
 * @brief    I91200 SPI1 driver header file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __SPI1_H__
#define __SPI1_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_SPI1_Driver SPI1 Driver
  @{
*/

/** @addtogroup I91200_SPI1_EXPORTED_CONSTANTS SPI1 Exported Constants
  @{
*/

#define SPI1_MODE_0            (SPI1_CTL_TXNEG_Msk)                         /*!< CLKP=0; RX_NEG=0; TX_NEG=1 \hideinitializer */
#define SPI1_MODE_1            (SPI1_CTL_RXNEG_Msk)                         /*!< CLKP=0; RX_NEG=1; TX_NEG=0 \hideinitializer */
#define SPI1_MODE_2            (SPI1_CTL_CLKP_Msk | SPI1_CTL_TXNEG_Msk)     /*!< CLKP=1; RX_NEG=1; TX_NEG=0 \hideinitializer */
#define SPI1_MODE_3            (SPI1_CTL_CLKP_Msk | SPI1_CTL_RXNEG_Msk)     /*!< CLKP=1; RX_NEG=0; TX_NEG=1 \hideinitializer */

#define SPI1_SLAVE             (0x1ul << SPI1_CTL_SLAVE_Pos)                /*!< Set as slave \hideinitializer */
#define SPI1_MASTER            (0x0ul << SPI1_CTL_SLAVE_Pos)                /*!< Set as master \hideinitializer */

#define SPI1_SS_NONE           (0x0ul<<SPI1_SSCTL_SSR_Pos)                  /*!< unset any spi port \hideinitializer */
#define SPI1_SS0               (0x1ul<<SPI1_SSCTL_SSR_Pos)                  /*!< Set SS0 \hideinitializer */

#define SPI1_SS_ACTIVE_HIGH    (SPI1_SSCTL_SSLVL_Msk)                       /*!< SS active high \hideinitializer */
#define SPI1_SS_ACTIVE_LOW     (0x0)                                        /*!< SS active low \hideinitializer */

#define SPI1_TXNUM_ONE         (0x00ul << SPI1_CTL_TXNUM_Pos)               /*!< Only one transmit/receive word will be executed in one transfer \hideinitializer */      
#define SPI1_TXNUM_TWO         (0x01ul << SPI1_CTL_TXNUM_Pos)               /*!< Two successive transmit/receive word will be executed in one transfer \hideinitializer */      

/*@}*/ /* end of group I91200_SPI1_EXPORTED_CONSTANTS */

/** @addtogroup I91200_SPI1_EXPORTED_FUNCTIONS SPI1 Exported Functions
  @{
*/

/**
  * @brief  Get the status flags.
  * @param  spi is the base address of SPI1 module.
  * @return status flags
  * \hideinitializer
  */
#define SPI1_GET_STATUS(spi) ( (spi)->CTL )

/**
  * @brief  Clear the unit transfer interrupt flag.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_CLR_UNIT_TRANS_INT_FLAG(spi) ( (spi)->CTL |= SPI1_CTL_IF_Msk )

/**
  * @brief  Get the Rx FIFO empty flag.
  * @param  spi is the base address of SPI1 module.
  * @return Rx FIFO flag
  * @retval 0: Rx FIFO is not empty
  * @retval 1: Rx FIFO is empty
  * \hideinitializer
  */
#define SPI1_GET_RX_FIFO_EMPTY_FLAG(spi) ( ((spi)->CTL & SPI1_CTL_RXEMPTY_Msk) == SPI1_CTL_RXEMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Rx FIFO full flag.
  * @param  spi is the base address of SPI1 module.
  * @return Rx FIFO flag
  * @retval 0: Rx FIFO is not full
  * @retval 1: Rx FIFO is full
  * \hideinitializer
  */
#define SPI1_GET_RX_FIFO_FULL_FLAG(spi) ( ((spi)->CTL & SPI1_CTL_RXEMPTY_Msk) == SPI1_CTL_RXEMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Tx FIFO empty flag.
  * @param  spi is the base address of SPI1 module.
  * @return Tx FIFO flag
  * @retval 0: Tx FIFO is not empty
  * @retval 1: Tx FIFO is empty
  * \hideinitializer
  */
#define SPI1_GET_TX_FIFO_EMPTY_FLAG(spi) ( ((spi)->CTL & SPI1_CTL_TXEMPTY_Msk) == SPI1_CTL_TXEMPTY_Msk ? 1:0 )

/**
  * @brief  Get the Tx FIFO full flag.
  * @param  spi is the base address of SPI1 module.
  * @return Tx FIFO flag
  * @retval 0: Tx FIFO is not full
  * @retval 1: Tx FIFO is full
  * \hideinitializer
  */
#define SPI1_GET_TX_FIFO_FULL_FLAG(spi) ( ((spi)->CTL & SPI1_CTL_TXFULL_Msk) == SPI1_CTL_TXFULL_Msk ? 1:0 )

/**
  * @brief  Get the datum read from RX0 FIFO.
  * @param  spi is the base address of SPI1 module.
  * @return data in Rx0 register
  * \hideinitializer
  */
#define SPI1_READ_RX0(spi) ((spi)->RX0)


/**
  * @brief  Get the datum read from RX1 FIFO.
  * @param  spi is the base address of SPI1 module.
  * @return data in Rx1 register
  * \hideinitializer
  */
#define SPI1_READ_RX1(spi) ((spi)->RX1)

/**
  * @brief  Write datum to TX0 register.
  * @param  spi is the base address of SPI1 module.
  * @param  u32TxData is the datum which user attempt to transfer through SPI1 bus.
  * @return none
  * \hideinitializer
  */
#define SPI1_WRITE_TX0(spi, u32TxData) ( (spi)->TX0 = u32TxData )

/**
  * @brief  Write datum to TX1 register.
  * @param  spi is the base address of SPI1 module.
  * @param  u32TxData is the datum which user attempt to transfer through SPI1 bus.
  * @return none
  * \hideinitializer
  */
#define SPI1_WRITE_TX1(spi, u32TxData) ( (spi)->TX1 = u32TxData )

/**
  * @brief  Configure the slave select pins. 
  * @param  spi is the base address of SPI1 module.
  * @param  u32SS is the specified slave select pins will be set to active state.
  * @return none
  * \hideinitializer
  */
#define SPI1_SET_SS(spi,u32SS) ( (spi)->SSCTL = ( (spi)->SSCTL & ~SPI1_SSCTL_SSR_Msk ) | u32SS )

/**
  * @brief  Configure the set slave active level. 
  * @param  spi is the base address of SPI1 module.
  * @param  u32Level is the specified slave active level.
  * @return none
  * \hideinitializer
  */
#define SPI1_SET_SLAVE_ACTIVE_LEVEL(spi,u32Level) ( (spi)->SSCTL = ( (spi)->SSCTL & ~SPI1_SSCTL_SSLVL_Msk ) | u32Level )

/**
  * @brief Enable byte reorder function.
  * @note Before calling this function, SPI1 must be stopped first.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_ENABLE_BYTE_REORDER(spi) ( (spi)->CTL |= SPI1_CTL_BYTEENDIAN_Msk )

/**
  * @brief  Disable byte reorder function.
  * @note Before calling this function, SPI1 must be stopped first.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_DISABLE_BYTE_REORDER(spi) ( (spi)->CTL &= ~SPI1_CTL_BYTEENDIAN_Msk )

/**
  * @brief  Set the length of suspend interval.
  * @note Before calling this function, SPI1 must be stopped first.
  * @param  spi is the base address of SPI1 module.
  * @param  u32SuspCycle decides the length of suspend interval.
  * @return none
  * \hideinitializer
  */
#define SPI1_SET_SUSPEND_CYCLE(spi, u32SuspCycle) ( (spi)->CTL = ((spi)->CTL & ~SPI1_CTL_SLEEP_Msk) | (u32SuspCycle << SPI1_CTL_SLEEP_Pos) )

/**
  * @brief  Set the SPI1 transfer sequence with LSB first.
  * @note Before calling this function, SPI1 must be stopped first.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_SET_LSB_FIRST(spi) ( (spi)->CTL |= SPI1_CTL_LSB_Msk )

/**
  * @brief  Set the SPI1 transfer sequence with MSB first.
  * @note Before calling this function, SPI1 must be stopped first.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_SET_MSB_FIRST(spi) ( (spi)->CTL &= ~SPI1_CTL_LSB_Msk )

/**
  * @brief  Set the data width of a SPI1 transaction.
  * @note Before calling this function, SPI1 must be stopped first.
  * @param  spi is the base address of SPI1 module.
  * @param  u32Width data width
  * @return none
  * \hideinitializer
  */
#define SPI1_SET_DATA_WIDTH(spi,u32Width) ( (spi)->CTL = ((spi)->CTL & ~SPI1_CTL_TXBITLEN_Msk)|((u32Width == 32 ? 0:u32Width)<<SPI1_CTL_TXBITLEN_Pos) ) 

/**
  * @brief  Set the data count of a SPI1 transaction.
  * @param  spi is the base address of SPI1 module.
  * @param  u32TxNum is the transmit/receive word numbers.
  * @return none
  * \hideinitializer
  */
#define SPI1_SET_TX_NUM(spi,u32TxNum)  ((spi)->CTL = ((spi)->CTL & ~SPI1_CTL_TXNUM_Msk ) | u32TxNum)

/**
  * @brief  Get the SPI1 busy state.
  * @param  spi is the base address of SPI1 module.
  * @return SPI1 busy status
  * @retval 0: SPI1 module is not busy
  * @retval 1: SPI1 module is busy
  * \hideinitializer
  */
#define SPI1_IS_BUSY(spi) ( ((spi)->CTL & SPI1_CTL_EN_Msk) == SPI1_CTL_EN_Msk ? 1:0 )

/**
  * @brief  Set SPI1 to busy state.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_GO(spi) ((spi)->CTL |= SPI1_CTL_EN_Msk)

/**
  * @brief  Enable SPI1 interrupt.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_ENABLE_INTERRUPT(spi)  ((spi)->CTL |= SPI1_CTL_IE_Msk)

/**
  * @brief  Disable SPI1 interrupt.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_DISABLE_INTERRUPT(spi)  ((spi)->CTL &= ~SPI1_CTL_IE_Msk)

/**
  * @brief  Enable the automatic slave select function. Only available in Master mode.
  * @param  spi is the base address of SPI1 module.
  * @param  u32SSPin specifies slave select pins. Valid values are:
  *                     - \ref SPI1_SS0
  * @param  u32ActiveLvl specifies the active level of slave select signal. Valid values are:
  *                     - \ref SPI1_SS_ACTIVE_HIGH
  *                     - \ref SPI1_SS_ACTIVE_LOW
  * @return none
  * \hideinitializer
  */
#define SPI1_ENABLE_AUTOSS(spi,u32SSPin,u32ActiveLvl)  ((spi)->SSCTL = ((spi)->SSCTL&~(SPI1_SSCTL_SSR_Msk|SPI1_SSCTL_SSLVL_Msk))|(u32SSPin|u32ActiveLvl|SPI1_SSCTL_ASS_Msk))

/**
  * @brief  Disable the automatic slave select function. 
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_DISABLE_AUTOSS(spi)  ((spi)->SSCTL &= ~SPI1_SSCTL_ASS_Msk)

/**
  * @brief  Trigger RX PDMA transfer.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_TRIGGER_RX_PDMA(spi) ( (spi)->PDMACTL |= SPI1_PDMACTL_RXMDAEN_Msk )

/**
  * @brief  Trigger TX PDMA transfer.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_TRIGGER_TX_PDMA(spi) ( (spi)->PDMACTL |= SPI1_PDMACTL_TXMDAEN_Msk )

/**
  * @brief  Disable RX PDMA transfer.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_DISABLE_RX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI1_PDMACTL_RXMDAEN_Msk )

/**
  * @brief  Trigger TX PDMA transfer.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_DISABLE_TX_PDMA(spi) ( (spi)->PDMACTL &= ~SPI1_PDMACTL_TXMDAEN_Msk )

/**
  * @brief  Enable 2-bit transfer mode.
  * @note Before calling this function, SPI1 must be stopped first.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_ENABLE_2BIT_MODE(spi) ( (spi)->CTL |= SPI1_CTL_TWOB_Msk )

/**
  * @brief  Disable 2-bit transfer mode.
  * @note Before calling this function, SPI1 must be stopped first.
  * @param  spi is the base address of SPI1 module.
  * @return none
  * \hideinitializer
  */
#define SPI1_DISABLE_2BIT_MODE(spi) ( (spi)->CTL &= ~SPI1_CTL_TWOB_Msk )

uint32_t SPI1_Open(SPI1_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32BusClock, uint32_t u32VarClock);
void     SPI1_Close(SPI1_T *spi);
uint32_t SPI1_SetBusClock(SPI1_T *spi, uint32_t u32BusClock);
void     SPI1_SetVarClock(SPI1_T *spi, uint32_t u32VarClock);
uint32_t SPI1_GetBusClock(SPI1_T *spi);
uint32_t SPI1_GetVarClock(SPI1_T *spi);

/*@}*/ /* end of group I91200_SPI1_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_SPI1_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__SPI1_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
