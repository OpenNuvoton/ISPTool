/****************************************************************************//**
 * @file     spi.c
 * @version  V0.10
 * $Revision: 7 $
 * $Date: 15/05/28 3:34p $
 * @brief    NANO100 series SPI driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "nano100series.h"

/** @addtogroup NANO100_Device_Driver NANO100 Device Driver
  @{
*/

/** @addtogroup NANO100_SPI_Driver SPI Driver
  @{
*/


/** @addtogroup NANO100_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
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

    spi->CTL = u32MasterSlave | (u32DataWidth << SPI_CTL_TX_BIT_LEN_Pos) | (u32SPIMode);

    return ( SPI_SetBusClock(spi, u32BusClock) );
}

/**
  * @brief Reset SPI module
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_Close(SPI_T *spi)
{
    /* Reset SPI */
    if(spi == SPI0) {
        SYS->IPRST_CTL2 |= SYS_IPRST_CTL2_SPI0_RST_Msk;
        SYS->IPRST_CTL2 &= ~SYS_IPRST_CTL2_SPI0_RST_Msk;
    } else if(spi == SPI1) {
        SYS->IPRST_CTL2 |= SYS_IPRST_CTL2_SPI1_RST_Msk;
        SYS->IPRST_CTL2 &= ~SYS_IPRST_CTL2_SPI1_RST_Msk;
    } else {
        SYS->IPRST_CTL2 |= SYS_IPRST_CTL2_SPI2_RST_Msk;
        SYS->IPRST_CTL2 &= ~SYS_IPRST_CTL2_SPI2_RST_Msk;
    }
}

/**
  * @brief Clear Rx FIFO buffer.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_ClearRxFIFO(SPI_T *spi)
{
    spi->FFCLR |= SPI_FFCLR_RX_CLR_Msk;
}

/**
  * @brief Clear Tx FIFO buffer.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_ClearTxFIFO(SPI_T *spi)
{
    spi->FFCLR |= SPI_FFCLR_TX_CLR_Msk;
}

/**
  * @brief Disable the automatic slave select function.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_DisableAutoSS(SPI_T *spi)
{
    spi->SSR &= ~SPI_SSR_AUTOSS_Msk;
}

/**
  * @brief Enable the automatic slave select function. Only available in Master mode.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32SSPinMask specifies slave select pins. ( \ref SPI_SS0 or \ref SPI_SS1 )
  * @param[in]  u32ActiveLevel specifies the active level of slave select signal. ( \ref SPI_SS0_ACTIVE_HIGH, \ref SPI_SS0_ACTIVE_LOW, \ref SPI_SS1_ACTIVE_HIGH, or \ref SPI_SS1_ACTIVE_LOW)
  * @return none
  */
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    spi->SSR = (spi->SSR & ~(SPI_SSR_SS_LVL_Msk | SPI_SSR_SSR_Msk)) | (u32SSPinMask | u32ActiveLevel) | SPI_SSR_AUTOSS_Msk;
}

/**
  * @brief Set the SPI bus clock. Only available in Master mode.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32BusClock is the expected frequency of SPI bus clock.
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock)
{
    uint32_t u32ClkSrc, u32Div = 0;

    u32ClkSrc = CLK_GetHCLKFreq();

    if(u32BusClock > u32ClkSrc)
        u32BusClock = u32ClkSrc;

    if(u32BusClock != 0 ) {
        u32Div = (u32ClkSrc / u32BusClock) - 1;
        if(u32Div > SPI_CLKDIV_DIVIDER1_Msk)
            u32Div = SPI_CLKDIV_DIVIDER1_Msk;
    } else
        return 0;

    spi->CLKDIV = (spi->CLKDIV & ~SPI_CLKDIV_DIVIDER1_Msk) | u32Div;

    return ( u32ClkSrc / (u32Div+1) );
}

/**
  * @brief Enable FIFO mode with user-specified Tx FIFO threshold and Rx FIFO threshold configurations.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_EnableFIFO(SPI_T *spi)
{
    spi->CTL |= SPI_CTL_FIFOM_Msk;
}

/**
  * @brief Disable FIFO mode.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_DisableFIFO(SPI_T *spi)
{
    spi->CTL &= ~SPI_CTL_FIFOM_Msk;
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

    u32ClkSrc = CLK_GetHCLKFreq();

    u32Div = spi->CLKDIV & SPI_CLKDIV_DIVIDER1_Msk;
    return (u32ClkSrc / (u32Div + 1));
}

/**
  * @brief Enable FIFO related interrupts specified by u32Mask parameter.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be enabled. Valid values are:
  *           - \ref SPI_IE_MASK
  *           - \ref SPI_SSTA_INTEN_MASK
  * @return none
  */
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask)
{
    if((u32Mask & SPI_IE_MASK) == SPI_IE_MASK)
        spi->CTL |= SPI_CTL_INTEN_Msk;

    if((u32Mask & SPI_SSTA_INTEN_MASK) == SPI_SSTA_INTEN_MASK)
        spi->SSR |= SPI_SSR_SSTA_INTEN_Msk;
}

/**
  * @brief Disable FIFO related interrupts specified by u32Mask parameter.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be enabled. Valid values are:
  *           - \ref SPI_IE_MASK
  *           - \ref SPI_SSTA_INTEN_MASK
  * @return none
  */
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask)
{
    if((u32Mask & SPI_IE_MASK) == SPI_IE_MASK)
        spi->CTL &= ~SPI_CTL_INTEN_Msk;

    if((u32Mask & SPI_SSTA_INTEN_MASK) == SPI_SSTA_INTEN_MASK)
        spi->SSR &= ~SPI_SSR_SSTA_INTEN_Msk;
}

/**
  * @brief Enable wake-up function.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_EnableWakeup(SPI_T *spi)
{
    spi->CTL |= SPI_CTL_WKEUP_EN_Msk;
}

/**
  * @brief Disable wake-up function.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_DisableWakeup(SPI_T *spi)
{
    spi->CTL &= ~SPI_CTL_WKEUP_EN_Msk;
}

/*@}*/ /* end of group NANO100_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO100_SPI_Driver */

/*@}*/ /* end of group NANO100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
