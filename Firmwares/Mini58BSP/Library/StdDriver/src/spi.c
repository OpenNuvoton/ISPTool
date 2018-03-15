/**************************************************************************//**
 * @file     spi.c
 * @version  V1.00
 * $Revision: 7 $
 * $Date: 15/05/28 1:47p $
 * @brief    Mini58 series SPI driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Mini58Series.h"
/** @addtogroup Mini58_Device_Driver Mini58 Device Driver
  @{
*/

/** @addtogroup Mini58_SPI_Driver SPI Driver
  @{
*/


/** @addtogroup Mini58_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  This function make SPI module be ready to transfer.
  *         By default, the SPI transfer sequence is MSB first and
  *         the automatic slave select function is disabled. In
  *         Slave mode, the u32BusClock must be NULL and the SPI clock
  *         divider setting will be 0.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32MasterSlave decides the SPI module is operating in master mode or in slave mode. ( \ref SPI_SLAVE, \ref SPI_MASTER)
  * @param[in]  u32SPIMode decides the transfer timing. ( \ref SPI_MODE_0, \ref SPI_MODE_1, \ref SPI_MODE_2, \ref SPI_MODE_3)
  * @param[in]  u32DataWidth decides the data width of a SPI transaction.
  * @param[in]  u32BusClock is the expected frequency of SPI bus clock in Hz.
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI_Open(SPI_T *spi,
                  uint32_t u32MasterSlave,
                  uint32_t u32SPIMode,
                  uint32_t u32DataWidth,
                  uint32_t u32BusClock)
{
    if(u32DataWidth == 32)
        u32DataWidth = 0;

    spi->CTL = u32MasterSlave | (u32DataWidth << SPI_CTL_DWIDTH_Pos) | (u32SPIMode);

    if(u32MasterSlave == SPI_SLAVE)
        spi->SSCTL = SPI_SSCTL_SSLTEN_Msk;

    return ( SPI_SetBusClock(spi, u32BusClock) );
}

/**
  * @brief Reset SPI module and disable SPI peripheral clock.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_Close(SPI_T *spi)
{
    /* Reset SPI */
    SYS->IPRST1 |= SYS_IPRST1_SPI0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_SPI0RST_Msk;
}

/**
  * @brief Clear Rx FIFO buffer.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_ClearRxFIFO(SPI_T *spi)
{
    spi->FIFOCTL |= SPI_FIFOCTL_RXRST_Msk;
}

/**
  * @brief Clear Tx FIFO buffer.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_ClearTxFIFO(SPI_T *spi)
{
    spi->FIFOCTL |= SPI_FIFOCTL_TXRST_Msk;
}

/**
  * @brief Disable the automatic slave select function.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_DisableAutoSS(SPI_T *spi)
{
    spi->SSCTL &= ~SPI_SSCTL_AUTOSS_Msk;
}

/**
  * @brief Enable the automatic slave select function. Only available in Master mode.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32SSPinMask specifies slave select pins. ( \ref SPI_SS)
  * @param[in]  u32ActiveLevel specifies the active level of slave select signal. ( \ref SPI_SS_ACTIVE_HIGH, \ref SPI_SS_ACTIVE_LOW)
  * @return none
  */
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    spi->SSCTL = (spi->SSCTL & ~(SPI_SSCTL_SSACTPOL_Msk | SPI_SSCTL_SS_Msk)) | (u32SSPinMask | u32ActiveLevel) | SPI_SSCTL_AUTOSS_Msk;
}

/**
  * @brief Set the SPI bus clock. Only available in Master mode.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32BusClock is the expected frequency of SPI bus clock.
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock)
{
    uint32_t u32Div = 0;
    uint32_t u32ClkSrc;

    if((CLK->CLKSEL1 & CLK_CLKSEL1_SPISEL_Msk) == CLK_CLKSEL1_SPISEL_XTAL)
    {
        if((u32ClkSrc = CLK_GetHXTFreq()) == 0)
            u32ClkSrc = CLK_GetLXTFreq();
    }
    else if((CLK->CLKSEL1 & CLK_CLKSEL1_SPISEL_Msk) == CLK_CLKSEL1_SPISEL_PLL)
        u32ClkSrc = CLK_GetPLLClockFreq();
    else
        u32ClkSrc = CLK_GetHCLKFreq();

    if(u32BusClock > u32ClkSrc)
        u32BusClock = u32ClkSrc;

    if(u32BusClock != 0)
    {
        u32Div = (((u32ClkSrc / u32BusClock) + 1) >> 1) - 1;
        if(u32Div > SPI_CLKDIV_DIVIDER_Msk)
            u32Div = SPI_CLKDIV_DIVIDER_Msk;
    }
    else
        return 0;

    spi->CLKDIV = (spi->CLKDIV & ~SPI_CLKDIV_DIVIDER_Msk) | u32Div;

    return ( u32ClkSrc / ((u32Div+1)*2) );
}

/**
  * @brief Enable FIFO mode with user-specified Tx FIFO threshold and Rx FIFO threshold configurations.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32TxThreshold decides the Tx FIFO threshold.
  * @param[in]  u32RxThreshold decides the Rx FIFO threshold.
  * @return none
  */
void SPI_EnableFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold)
{
    spi->FIFOCTL = (spi->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk) |
                    (u32TxThreshold << SPI_FIFOCTL_TXTH_Pos) |
                    (u32RxThreshold << SPI_FIFOCTL_RXTH_Pos));

    spi->CTL |= SPI_CTL_FIFOEN_Msk;
}

/**
  * @brief Disable FIFO mode.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_DisableFIFO(SPI_T *spi)
{
    spi->CTL &= ~SPI_CTL_FIFOEN_Msk;
}

/**
  * @brief Get the actual frequency of SPI bus clock. Only available in Master mode.
  * @param[in]  spi is the base address of SPI module.
  * @return Actual SPI bus clock frequency.
  */
uint32_t SPI_GetBusClock(SPI_T *spi)
{
    uint32_t u32Div;
    uint32_t u32ClkSrc;

    if((CLK->CLKSEL1 & CLK_CLKSEL1_SPISEL_Msk) == CLK_CLKSEL1_SPISEL_XTAL)
    {
        if((u32ClkSrc = CLK_GetHXTFreq()) == 0)
            u32ClkSrc = CLK_GetLXTFreq();
    }
    else if((CLK->CLKSEL1 & CLK_CLKSEL1_SPISEL_Msk) == CLK_CLKSEL1_SPISEL_PLL)
        u32ClkSrc = CLK_GetPLLClockFreq();
    else
        u32ClkSrc = CLK_GetHCLKFreq();

    u32Div = spi->CLKDIV & SPI_CLKDIV_DIVIDER_Msk;

    return ( u32ClkSrc / ((u32Div + 1)*2) );
}

/**
  * @brief Enable FIFO related interrupts specified by u32Mask parameter.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be enabled.
  *            ( \ref SPI_IE_MASK, \ref SPI_SSTA_INTEN_MASK, \ref SPI_FIFO_TX_INTEN_MASK,
  *            \ref SPI_FIFO_RX_INTEN_MASK, \ref SPI_FIFO_RXOV_INTEN_MASK, \ref SPI_FIFO_TIMEOUT_INTEN_MASK)
  * @return none
  */
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask)
{
    if((u32Mask & SPI_IE_MASK) == SPI_IE_MASK)
        spi->CTL |= SPI_CTL_UNITIEN_Msk;

    if((u32Mask & SPI_SSTA_INTEN_MASK) == SPI_SSTA_INTEN_MASK)
        spi->SLVCTL |= SPI_SLVCTL_SLVSTIEN_Msk;

    if((u32Mask & SPI_FIFO_TX_INTEN_MASK) == SPI_FIFO_TX_INTEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_TXTHIEN_Msk;

    if((u32Mask & SPI_FIFO_RX_INTEN_MASK) == SPI_FIFO_RX_INTEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXTHIEN_Msk;

    if((u32Mask & SPI_FIFO_RXOV_INTEN_MASK) == SPI_FIFO_RXOV_INTEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXOVIEN_Msk;

    if((u32Mask & SPI_FIFO_TIMEOUT_INTEN_MASK) == SPI_FIFO_TIMEOUT_INTEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXTOIEN_Msk;
}

/**
  * @brief Disable FIFO related interrupts specified by u32Mask parameter.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be disabled.
  *            ( \ref SPI_IE_MASK, \ref SPI_SSTA_INTEN_MASK, \ref SPI_FIFO_TX_INTEN_MASK,
  *            \ref SPI_FIFO_RX_INTEN_MASK, \ref SPI_FIFO_RXOV_INTEN_MASK, \ref SPI_FIFO_TIMEOUT_INTEN_MASK)
  * @return none
  */
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask)
{
    if((u32Mask & SPI_IE_MASK) == SPI_IE_MASK)
        spi->CTL &= ~SPI_CTL_UNITIEN_Msk;

    if((u32Mask & SPI_SSTA_INTEN_MASK) == SPI_SSTA_INTEN_MASK)
        spi->SLVCTL &= ~SPI_SLVCTL_SLVSTIEN_Msk;

    if((u32Mask & SPI_FIFO_TX_INTEN_MASK) == SPI_FIFO_TX_INTEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_TXTHIEN_Msk;

    if((u32Mask & SPI_FIFO_RX_INTEN_MASK) == SPI_FIFO_RX_INTEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTHIEN_Msk;

    if((u32Mask & SPI_FIFO_RXOV_INTEN_MASK) == SPI_FIFO_RXOV_INTEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXOVIEN_Msk;

    if((u32Mask & SPI_FIFO_TIMEOUT_INTEN_MASK) == SPI_FIFO_TIMEOUT_INTEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTOIEN_Msk;
}

/*@}*/ /* end of group Mini58_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group Mini58_SPI_Driver */

/*@}*/ /* end of group Mini58_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
