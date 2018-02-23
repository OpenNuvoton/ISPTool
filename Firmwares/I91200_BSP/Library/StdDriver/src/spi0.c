/****************************************************************************//**
 * @file     spi0.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/05/12 10:00a $
 * @brief    I91200 SPI0 driver source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_SPI0_Driver SPI0 Driver
  @{
*/

/** @addtogroup I91200_SPI0_EXPORTED_FUNCTIONS SPI0 Exported Functions
  @{
*/

/**
  * @brief  This function make SPI0 module be ready to transfer.
  *         By default, the SPI transfer sequence is MSB first and
  *         the automatic slave select function is disabled. In
  *         Slave mode, the u32BusClock must be NULL and the SPI clock
  *         divider setting will be 0.
  * @param  spi is the base address of SPI0 module.
  * @param  u32MasterSlave decides the SPI0 module is operating in master mode or in slave mode. Valid values are:
  *                    - \ref SPI0_SLAVE
  *                    - \ref SPI0_MASTER
  * @param  u32SPIMode decides the transfer timing. Valid values are:
  *                    - \ref SPI0_MODE_0
  *                    - \ref SPI0_MODE_1
  *                    - \ref SPI0_MODE_2
  *                    - \ref SPI0_MODE_3
  * @param  u32DataWidth decides the data width of a SPI transaction.
  * @param  u32BusClock is the expected frequency of SPI bus clock in Hz.
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI0_Open(SPI0_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode,  uint32_t u32DataWidth, uint32_t u32BusClock)
{
    // Config SPI0's control register.
	spi->CTL = ( u32MasterSlave | (((u32DataWidth>=32)?0:u32DataWidth) << SPI0_CTL_DWIDTH_Pos) | (u32SPIMode) );
	// Set SPI0's bus clock and return real frequency.
	return SPI0_SetBusClock(spi,u32BusClock);
}

/**
  * @brief Disable SPI peripheral clock.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI0_Close(SPI0_T *spi)
{
	
}

/**
  * @brief Clear Rx FIFO buffer.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI0_ClearRxFIFO(SPI0_T *spi)
{
    spi->FIFOCTL |= SPI0_FIFOCTL_RXRST_Msk;
}

/**
  * @brief Clear Tx FIFO buffer.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI0_ClearTxFIFO(SPI0_T *spi)
{
    spi->FIFOCTL |= SPI0_FIFOCTL_TXRST_Msk;
}

/**
  * @brief Disable the automatic slave select function.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI0_DisableAutoSS(SPI0_T *spi)
{
    spi->SSCTL &= ~SPI0_SSCTL_AUTOSS_Msk;
}

/**
  * @brief Enable the automatic slave select function. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @param  u32SSPinMask specifies slave select pins. Valid values are:
  *                     - \ref SPI0_SS0
  *                     - \ref SPI0_SS1
  * @param  u32ActiveLevel specifies the active level of slave select signal. Valid values are:
  *                     - \ref SPI0_SS_ACTIVE_HIGH
  *                     - \ref SPI0_SS_ACTIVE_LOW
  * @return none
  */
void SPI0_EnableAutoSS(SPI0_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    spi->SSCTL |= (u32SSPinMask | u32ActiveLevel) | SPI0_SSCTL_AUTOSS_Msk;
}

/**
  * @brief Set the SPI bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @param  u32BusClock is the expected frequency of SPI bus clock.
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI0_SetBusClock(SPI0_T *spi, uint32_t u32BusClock)
{
    uint32_t u32Div = 0xffff,u32Pclk = CLK_GetHCLKFreq();

    if( u32BusClock !=0 ) 
	{
        u32Div = (u32Pclk/u32BusClock) - 1;
        spi->CLKDIV = (spi->CLKDIV & ~SPI0_CLKDIV_DIVIDER_Msk) | ((u32Div>0xff)?0xff:u32Div);
    } 
	else
        spi->CLKDIV = 0;

    return (u32Pclk/((spi->CLKDIV& SPI0_CLKDIV_DIVIDER_Msk)+1));
}

/**
  * @brief Set Tx FIFO threshold and Rx FIFO threshold configurations.
  * @param  spi is the base address of SPI module.
  * @param  u32TxThreshold decides the Tx FIFO threshold.
  * @param  u32RxThreshold decides the Rx FIFO threshold.
  * @return none
  */
void SPI0_SetFIFOThreshold(SPI0_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold)
{
    spi->FIFOCTL &= (~(SPI0_FIFOCTL_TXTH_Msk | SPI0_FIFOCTL_RXTH_Msk));
	spi->FIFOCTL |= (u32TxThreshold << SPI0_FIFOCTL_TXTH_Pos);
	spi->FIFOCTL |= (u32RxThreshold << SPI0_FIFOCTL_RXTH_Pos);
}

/**
  * @brief Get the actual frequency of SPI bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @return Actual SPI bus clock frequency.
  */
uint32_t SPI0_GetBusClock(SPI0_T *spi)
{
    return (CLK_GetHCLKFreq()/ ((spi->CLKDIV&0xff) + 1));
}

/**
  * @brief Enable FIFO related interrupts specified by u32Mask parameter.
  * @param  spi is the base address of SPI module.
  * @param  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be enabled. Valid values are:
  *           - \ref SPI0_UNITIEN_MASK
  *           - \ref SPI0_SSINAIEN_MASK
  *           - \ref SPI0_SSACTIEN_MASK
  *           - \ref SPI0_SLVURIEN_MASK
  *           - \ref SPI0_SLVBEIEN_MASK
  *           - \ref SPI0_SLVTOIEN_MASK
  *           - \ref SPI0_FIFO_TXTHIEN_MASK
  *           - \ref SPI0_FIFO_RXTHIEN_MASK
  *           - \ref SPI0_FIFO_RXOVIEN_MASK
  *           - \ref SPI0_FIFO_TXUFIEN_MASK
  *           - \ref SPI0_FIFO_RXTOIEN_MASK
  * @return none
  */
void SPI0_EnableInt(SPI0_T *spi, uint32_t u32Mask)
{
    if(u32Mask & SPI0_UNITIEN_MASK)
        spi->CTL |= SPI0_CTL_UNITIEN_Msk;

    if(u32Mask & SPI0_SSINAIEN_MASK)
        spi->SSCTL |= SPI0_SSCTL_SSINAIEN_Msk;

    if(u32Mask & SPI0_SSACTIEN_MASK)
        spi->SSCTL |= SPI0_SSCTL_SSACTIEN_Msk;

    if(u32Mask & SPI0_SLVURIEN_MASK)
        spi->SSCTL |= SPI0_SSCTL_SLVUDRIEN_Msk;

    if(u32Mask & SPI0_SLVBEIEN_MASK)
        spi->SSCTL |= SPI0_SSCTL_SLVBCEIEN_Msk;

    if(u32Mask & SPI0_SLVTOIEN_MASK)
        spi->SSCTL |= SPI0_SSCTL_SLVTOIEN_Msk;

    if(u32Mask & SPI0_FIFO_TXTHIEN_MASK)
        spi->FIFOCTL |= SPI0_FIFOCTL_TXTHIEN_Msk;

    if(u32Mask & SPI0_FIFO_RXTHIEN_MASK)
        spi->FIFOCTL |= SPI0_FIFOCTL_RXTHIEN_Msk;

    if(u32Mask & SPI0_FIFO_RXOVIEN_MASK)
        spi->FIFOCTL |= SPI0_FIFOCTL_RXOVIEN_Msk;

    if(u32Mask & SPI0_FIFO_TXUFIEN_MASK)
        spi->FIFOCTL |= SPI0_FIFOCTL_TXUDFIEN_Msk;

    if(u32Mask & SPI0_FIFO_RXTOIEN_MASK)
        spi->FIFOCTL |= SPI0_FIFOCTL_RXTOIEN_Msk;
}

/**
  * @brief Disable FIFO related interrupts specified by u32Mask parameter.
  * @param  spi is the base address of SPI module.
  * @param  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be enabled. Valid values are:
  *           - \ref SPI0_UNITIEN_MASK
  *           - \ref SPI0_SSINAIEN_MASK
  *           - \ref SPI0_SSACTIEN_MASK
  *           - \ref SPI0_SLVURIEN_MASK
  *           - \ref SPI0_SLVBEIEN_MASK
  *           - \ref SPI0_SLVTOIEN_MASK
  *           - \ref SPI0_FIFO_TXTHIEN_MASK
  *           - \ref SPI0_FIFO_RXTHIEN_MASK
  *           - \ref SPI0_FIFO_RXOVIEN_MASK
  *           - \ref SPI0_FIFO_TXUFIEN_MASK
  *           - \ref SPI0_FIFO_RXTOIEN_MASK
  * @return none
  */
void SPI0_DisableInt(SPI0_T *spi, uint32_t u32Mask)
{
    if(u32Mask & SPI0_UNITIEN_MASK)
        spi->CTL &= ~SPI0_CTL_UNITIEN_Msk;

    if(u32Mask & SPI0_SSINAIEN_MASK)
        spi->SSCTL &= ~SPI0_SSCTL_SSINAIEN_Msk;

    if(u32Mask & SPI0_SSACTIEN_MASK)
        spi->SSCTL &= ~SPI0_SSCTL_SSACTIEN_Msk;

    if(u32Mask & SPI0_SLVURIEN_MASK)
        spi->SSCTL &= ~SPI0_SSCTL_SLVUDRIEN_Msk;

    if(u32Mask & SPI0_SLVBEIEN_MASK)
        spi->SSCTL &= ~SPI0_SSCTL_SLVBCEIEN_Msk;

    if(u32Mask & SPI0_SLVTOIEN_MASK)
        spi->SSCTL &= ~SPI0_SSCTL_SLVTOIEN_Msk;

    if(u32Mask & SPI0_FIFO_TXTHIEN_MASK)
        spi->FIFOCTL &= ~SPI0_FIFOCTL_TXTHIEN_Msk;

    if(u32Mask & SPI0_FIFO_RXTHIEN_MASK)
        spi->FIFOCTL &= ~SPI0_FIFOCTL_RXTHIEN_Msk;

    if(u32Mask & SPI0_FIFO_RXOVIEN_MASK)
        spi->FIFOCTL &= ~SPI0_FIFOCTL_RXOVIEN_Msk;

    if(u32Mask & SPI0_FIFO_TXUFIEN_MASK)
        spi->FIFOCTL &= ~SPI0_FIFOCTL_TXUDFIEN_Msk;

    if(u32Mask & SPI0_FIFO_RXTOIEN_MASK)
        spi->FIFOCTL &= ~SPI0_FIFOCTL_RXTOIEN_Msk;
}

/*@}*/ /* end of group I91200_SPI0_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_SPI0_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
