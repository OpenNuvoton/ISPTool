/**************************************************************************//**
 * @file     spi.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 14/09/24 11:57a $
 * @brief    NUC029FAE SPI driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NUC029FAE.h"
/** @addtogroup NUC029FAE_Device_Driver NUC029FAE Device Driver
  @{
*/

/** @addtogroup NUC029FAE_SPI_Driver SPI Driver
  @{
*/


/** @addtogroup NUC029FAE_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  This function make SPI module be ready to transfer.
  *         By default, the SPI transfer sequence is MSB first and
  *         the automatic slave select function is disabled. In
  *         Slave mode, the u32BusClock must be NULL and the SPI clock
  *         divider setting will be 0.
  * @param  spi is the base address of SPI module.
  * @param  u32MasterSlave decides the SPI module is operating in master mode or in slave mode. (SPI_SLAVE, SPI_MASTER)
  * @param  u32SPIMode decides the transfer timing. (SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_03)
  * @param  u32DataWidth decides the data width of a SPI transaction.
  * @param  u32BusClock is the expected frequency of SPI bus clock in Hz.
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

    spi->CNTRL = u32MasterSlave | (u32DataWidth << SPI_CNTRL_TX_BIT_LEN_Pos) | (u32SPIMode);

    return ( SPI_SetBusClock(spi, u32BusClock) );
}

/**
  * @brief Reset SPI module and disable SPI peripheral clock.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI_Close(SPI_T *spi)
{
    /* Reset SPI */
    SYS->IPRSTC2 |= SYS_IPRSTC2_SPI_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_SPI_RST_Msk;

    /* Disable SPI clock */
    CLK->APBCLK &= ~CLK_APBCLK_SPI_EN_Msk;
}

/**
  * @brief Clear Rx FIFO buffer.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI_ClearRxFIFO(SPI_T *spi)
{
    spi->FIFO_CTL |= SPI_FIFO_CTL_RX_CLR_Msk;
}

/**
  * @brief Clear Tx FIFO buffer.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI_ClearTxFIFO(SPI_T *spi)
{
    spi->FIFO_CTL |= SPI_FIFO_CTL_TX_CLR_Msk;
}

/**
  * @brief Disable the automatic slave select function.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI_DisableAutoSS(SPI_T *spi)
{
    spi->SSR &= ~SPI_SSR_AUTOSS_Msk;
}

/**
  * @brief Enable the automatic slave select function. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @param  u32SSPinMask specifies slave select pins. (SPI_SS)
  * @param  u32ActiveLevel specifies the active level of slave select signal. (SPI_SS_ACTIVE_HIGH, SPI_SS_ACTIVE_LOW)
  * @return none
  */
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    spi->SSR |= (u32SSPinMask | u32ActiveLevel) | SPI_SSR_AUTOSS_Msk;
}

/**
  * @brief Set the SPI bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @param  u32BusClock is the expected frequency of SPI bus clock.
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock)
{
    uint32_t u32ClkSrc, u32Div = 0;

    if((CLK->CLKSEL1 & CLK_CLKSEL1_SPI_S_Msk) == CLK_CLKSEL1_SPI_S_HCLK)
        u32ClkSrc = CLK_GetHCLKFreq();
    else {
        if((CLK->PWRCON & CLK_PWRCON_XTLCLK_EN_Msk) == CLK_PWRCON_HXT)
            u32ClkSrc = CLK_GetHXTFreq();
        else
            u32ClkSrc = CLK_GetLXTFreq();
    }

    if(u32BusClock != 0 ) {
        u32Div = (u32ClkSrc / (2*u32BusClock)) - 1;
        if(u32Div > SPI_DIVIDER_DIVIDER_Msk)
            u32Div = SPI_DIVIDER_DIVIDER_Msk;
    }

    spi->DIVIDER = (spi->DIVIDER & ~SPI_DIVIDER_DIVIDER_Msk) | u32Div;

    return ( u32ClkSrc / ((u32Div + 1) *2) );
}

/**
  * @brief Enable FIFO mode with user-specified Tx FIFO threshold and Rx FIFO threshold configurations.
  * @param  spi is the base address of SPI module.
  * @param  u32TxThreshold decides the Tx FIFO threshold.
  * @param  u32RxThreshold decides the Rx FIFO threshold.
  * @return none
  */
void SPI_EnableFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold)
{
    spi->FIFO_CTL = (spi->FIFO_CTL & ~(SPI_FIFO_CTL_TX_THRESHOLD_Msk | SPI_FIFO_CTL_RX_THRESHOLD_Msk) |
                     (u32TxThreshold << SPI_FIFO_CTL_TX_THRESHOLD_Pos) |
                     (u32RxThreshold << SPI_FIFO_CTL_RX_THRESHOLD_Pos));

    spi->CNTRL |= SPI_CNTRL_FIFO_Msk;
}

/**
  * @brief Disable FIFO mode.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI_DisableFIFO(SPI_T *spi)
{
    spi->CNTRL &= ~SPI_CNTRL_FIFO_Msk;
}

/**
  * @brief Get the actual frequency of SPI bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @return Actual SPI bus clock frequency.
  */
uint32_t SPI_GetBusClock(SPI_T *spi)
{
    uint32_t u32Div = 0;
    uint32_t u32ClkSrc;

    if((CLK->CLKSEL1 & CLK_CLKSEL1_SPI_S_Msk) == CLK_CLKSEL1_SPI_S_HCLK)
        u32ClkSrc = CLK_GetHCLKFreq();
    else {
        if((CLK->PWRCON & CLK_PWRCON_XTLCLK_EN_Msk) == CLK_PWRCON_HXT)
            u32ClkSrc = CLK_GetHXTFreq();
        else
            u32ClkSrc = CLK_GetLXTFreq();
    }

    spi->DIVIDER = (spi->DIVIDER & ~SPI_DIVIDER_DIVIDER_Msk) | u32Div;

    return (u32ClkSrc / ((u32Div + 1) *2));
}

/**
  * @brief Enable FIFO related interrupts specified by u32Mask parameter.
  * @param  spi is the base address of SPI module.
  * @param  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be enabled.
  *            (SPI_IE_MASK, SPI_SSTA_INTEN_MASK, SPI_FIFO_TX_INTEN_MASK,
  *            SPI_FIFO_RX_INTEN_MASK, SPI_FIFO_RXOV_INTEN_MASK, SPI_FIFO_TIMEOUT_INTEN_MASK)
  * @return none
  */
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask)
{
    if((u32Mask & SPI_IE_MASK) == SPI_IE_MASK)
        spi->CNTRL |= SPI_CNTRL_IE_Msk;

    if((u32Mask & SPI_SSTA_INTEN_MASK) == SPI_SSTA_INTEN_MASK)
        spi->CNTRL2 |= SPI_CNTRL2_SSTA_INTEN_Msk;

    if((u32Mask & SPI_FIFO_TX_INTEN_MASK) == SPI_FIFO_TX_INTEN_MASK)
        spi->FIFO_CTL |= SPI_FIFO_CTL_TX_INTEN_Msk;

    if((u32Mask & SPI_FIFO_RX_INTEN_MASK) == SPI_FIFO_RX_INTEN_MASK)
        spi->FIFO_CTL |= SPI_FIFO_CTL_RX_INTEN_Msk;

    if((u32Mask & SPI_FIFO_RXOV_INTEN_MASK) == SPI_FIFO_RXOV_INTEN_MASK)
        spi->FIFO_CTL |= SPI_FIFO_CTL_RXOV_INTEN_Msk;

    if((u32Mask & SPI_FIFO_TIMEOUT_INTEN_MASK) == SPI_FIFO_TIMEOUT_INTEN_MASK)
        spi->FIFO_CTL |= SPI_FIFO_CTL_TIMEOUT_INTEN_Msk;
}

/**
  * @brief Disable FIFO related interrupts specified by u32Mask parameter.
  * @param  spi is the base address of SPI module.
  * @param  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be disabled.
  *            (SPI_IE_MASK, SPI_SSTA_INTEN_MASK, SPI_FIFO_TX_INTEN_MASK,
  *            SPI_FIFO_RX_INTEN_MASK, SPI_FIFO_RXOV_INTEN_MASK, SPI_FIFO_TIMEOUT_INTEN_MASK)
  * @return none
  */
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask)
{
    if((u32Mask & SPI_IE_MASK) == SPI_IE_MASK)
        spi->CNTRL &= ~SPI_CNTRL_IE_Msk;

    if((u32Mask & SPI_SSTA_INTEN_MASK) == SPI_SSTA_INTEN_MASK)
        spi->CNTRL2 &= ~SPI_CNTRL2_SSTA_INTEN_Msk;

    if((u32Mask & SPI_FIFO_TX_INTEN_MASK) == SPI_FIFO_TX_INTEN_MASK)
        spi->FIFO_CTL &= ~SPI_FIFO_CTL_TX_INTEN_Msk;

    if((u32Mask & SPI_FIFO_RX_INTEN_MASK) == SPI_FIFO_RX_INTEN_MASK)
        spi->FIFO_CTL &= ~SPI_FIFO_CTL_RX_INTEN_Msk;

    if((u32Mask & SPI_FIFO_RXOV_INTEN_MASK) == SPI_FIFO_RXOV_INTEN_MASK)
        spi->FIFO_CTL &= ~SPI_FIFO_CTL_RXOV_INTEN_Msk;

    if((u32Mask & SPI_FIFO_TIMEOUT_INTEN_MASK) == SPI_FIFO_TIMEOUT_INTEN_MASK)
        spi->FIFO_CTL &= ~SPI_FIFO_CTL_TIMEOUT_INTEN_Msk;
}

/*@}*/ /* end of group NUC029FAE_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NUC029FAE_SPI_Driver */

/*@}*/ /* end of group NUC029FAE_Device_Driver */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
