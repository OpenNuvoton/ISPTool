/****************************************************************************//**
 * @file     spi.c
 * @version  V0.10
 * $Revision: 9 $
 * $Date: 16/01/12 4:58p $
 * @brief    Nano 103 SPI driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include "Nano103.h"

/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_SPI_Driver SPI Driver
  @{
*/


/** @addtogroup NANO103_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  This function make SPI module be ready to transfer.
  *         By default, the SPI transfer sequence is MSB first and
  *         the automatic slave select function is disabled. In
  *         Slave mode, the u32BusClock must be NULL and the SPI clock
  *         divider setting will be 0.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32MasterSlave decides the SPI module is operating in master mode or in slave mode. Valid values are:
  *              - \ref SPI_MASTER
  *              - \ref SPI_SLAVE
  * @param[in]  u32SPIMode decides the transfer timing. Valid values are:
  *              - \ref SPI_MODE_0
  *              - \ref SPI_MODE_1
  *              - \ref SPI_MODE_2
  *              - \ref SPI_MODE_3
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
    if((uint32_t)spi == SPI0_BASE && (CLK->APBCLK & CLK_APBCLK_SPI0CKEN_Msk))
    {
        CLK->APBCLK &= ~CLK_APBCLK_SPI0CKEN_Msk;
        SYS->IPRST2 |= SYS_IPRST2_SPI0RST_Msk;
        SYS->IPRST2 &= ~SYS_IPRST2_SPI0RST_Msk;
        CLK->APBCLK |=  CLK_APBCLK_SPI0CKEN_Msk;
    }
    else if((uint32_t)spi == SPI1_BASE && (CLK->APBCLK & CLK_APBCLK_SPI1CKEN_Msk))
    {
        CLK->APBCLK &= ~CLK_APBCLK_SPI1CKEN_Msk;
        SYS->IPRST2 |= SYS_IPRST2_SPI1RST_Msk;
        SYS->IPRST2 &= ~SYS_IPRST2_SPI1RST_Msk;
        CLK->APBCLK |=  CLK_APBCLK_SPI1CKEN_Msk;
    }
    else if((uint32_t)spi == SPI2_BASE && (CLK->APBCLK & CLK_APBCLK_SPI2CKEN_Msk))
    {
        CLK->APBCLK &= ~CLK_APBCLK_SPI2CKEN_Msk;
        SYS->IPRST2 |= SYS_IPRST2_SPI2RST_Msk;
        SYS->IPRST2 &= ~SYS_IPRST2_SPI2RST_Msk;
        CLK->APBCLK |=  CLK_APBCLK_SPI2CKEN_Msk;
    }
    else if((uint32_t)spi == SPI3_BASE && (CLK->APBCLK & CLK_APBCLK_SPI3CKEN_Msk))
    {
        CLK->APBCLK &= ~CLK_APBCLK_SPI3CKEN_Msk;
        SYS->IPRST2 |= SYS_IPRST2_SPI3RST_Msk;
        SYS->IPRST2 &= ~SYS_IPRST2_SPI3RST_Msk;
        CLK->APBCLK |=  CLK_APBCLK_SPI3CKEN_Msk;
    }
}

/**
  * @brief Clear Rx FIFO buffer.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_ClearRxFIFO(SPI_T *spi)
{
    spi->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;
}

/**
  * @brief Clear Tx FIFO buffer.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_ClearTxFIFO(SPI_T *spi)
{
    spi->FIFOCTL |= SPI_FIFOCTL_TXFBCLR_Msk;
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
  * @param[in]  u32SSPinMask specifies slave select pins. (SPI_SS)
  * @param[in]  u32ActiveLevel specifies the active level of slave select signal. Valid values are:
  *              - \ref SPI_SS0_ACTIVE_HIGH
  *              - \ref SPI_SS0_ACTIVE_LOW
  *              - \ref SPI_SS1_ACTIVE_HIGH
  *              - \ref SPI_SS1_ACTIVE_LOW
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
    uint32_t u32ClkSrc, u32Div = 0;

    if(spi == SPI0)
    {
        if((CLK->CLKSEL1 & CLK_CLKSEL1_SPI0SEL_Msk) == CLK_CLKSEL1_SPI0SEL_HCLK)
            u32ClkSrc = CLK_GetHCLKFreq();
        else if((CLK->CLKSEL1 & CLK_CLKSEL1_SPI0SEL_Msk) == CLK_CLKSEL1_SPI0SEL_PLL)
            u32ClkSrc = CLK_GetPLLClockFreq();
        else if((CLK->CLKSEL1 & CLK_CLKSEL1_SPI0SEL_Msk) == CLK_CLKSEL1_SPI0SEL_HXT)
            u32ClkSrc = __HXT;
        else   //CLK_CLKSEL1_SPI0SEL_HIRC
        {
            if((CLK->CLKSEL0 & CLK_CLKSEL0_HIRCSEL_Msk)==CLK_CLKSEL0_HIRCSEL_Msk)
                u32ClkSrc = __HIRC36M;
            else
            {
                if((CLK->PWRCTL & CLK_PWRCTL_HIRC0EN_Msk)==CLK_PWRCTL_HIRC0EN_Msk)
                    u32ClkSrc = __HIRC16M;
                else
                    u32ClkSrc = __HIRC12M;
            }
        }
    }
    else if(spi == SPI1)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_HCLK)
            u32ClkSrc = CLK_GetHCLKFreq();
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_PLL)
            u32ClkSrc = CLK_GetPLLClockFreq();
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_HXT)
            u32ClkSrc = __HXT;
        else   //CLK_CLKSEL2_SPI1SEL_HIRC
        {
            if((CLK->CLKSEL0 & CLK_CLKSEL0_HIRCSEL_Msk)==CLK_CLKSEL0_HIRCSEL_Msk)
                u32ClkSrc = __HIRC36M;
            else
            {
                if((CLK->PWRCTL & CLK_PWRCTL_HIRC0EN_Msk)==CLK_PWRCTL_HIRC0EN_Msk)
                    u32ClkSrc = __HIRC16M;
                else
                    u32ClkSrc = __HIRC12M;
            }
        }
    }
    else if(spi == SPI2)
    {
        if((CLK->CLKSEL1 & CLK_CLKSEL1_SPI2SEL_Msk) == CLK_CLKSEL1_SPI2SEL_HCLK)
            u32ClkSrc = CLK_GetHCLKFreq();
        else if((CLK->CLKSEL1 & CLK_CLKSEL1_SPI2SEL_Msk) == CLK_CLKSEL1_SPI2SEL_PLL)
            u32ClkSrc = CLK_GetPLLClockFreq();
        else if((CLK->CLKSEL1 & CLK_CLKSEL1_SPI2SEL_Msk) == CLK_CLKSEL1_SPI2SEL_HXT)
            u32ClkSrc = __HXT;
        else   //CLK_CLKSEL1_SPI2SEL_HIRC
        {
            if((CLK->CLKSEL0 & CLK_CLKSEL0_HIRCSEL_Msk)== CLK_CLKSEL0_HIRCSEL_Msk)
                u32ClkSrc = __HIRC36M;
            else
            {
                if((CLK->PWRCTL & CLK_PWRCTL_HIRC0EN_Msk)==CLK_PWRCTL_HIRC0EN_Msk)
                    u32ClkSrc = __HIRC16M;
                else
                    u32ClkSrc = __HIRC12M;
            }
        }
    }
    else
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI3SEL_Msk) == CLK_CLKSEL2_SPI3SEL_HCLK)
            u32ClkSrc = CLK_GetHCLKFreq();
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI3SEL_Msk) == CLK_CLKSEL2_SPI3SEL_PLL)
            u32ClkSrc = CLK_GetPLLClockFreq();
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI3SEL_Msk) == CLK_CLKSEL2_SPI3SEL_HXT)
            u32ClkSrc = __HXT;
        else   //CLK_CLKSEL2_SPI3SEL_HIRC
        {
            if((CLK->CLKSEL0 & CLK_CLKSEL0_HIRCSEL_Msk)== CLK_CLKSEL0_HIRCSEL_Msk)
                u32ClkSrc = __HIRC36M;
            else
            {
                if((CLK->PWRCTL & CLK_PWRCTL_HIRC0EN_Msk)==CLK_PWRCTL_HIRC0EN_Msk)
                    u32ClkSrc = __HIRC16M;
                else
                    u32ClkSrc = __HIRC12M;
            }
        }
    }
    if(u32BusClock > u32ClkSrc)
        u32BusClock = u32ClkSrc;

    if(u32BusClock != 0 )
    {
        u32Div = (u32ClkSrc / u32BusClock) - 1;
        if(u32Div > SPI_CLKDIV_DIVIDER_Msk)
            u32Div = SPI_CLKDIV_DIVIDER_Msk;
    }
    else
        return 0;

    spi->CLKDIV = (spi->CLKDIV & ~SPI_CLKDIV_DIVIDER_Msk) | u32Div;

    return ( u32ClkSrc / (u32Div+1) );
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
    spi->FIFOCTL = ((spi->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk) | SPI_FIFOCTL_RXTH_Msk) |
                    (u32TxThreshold << SPI_FIFOCTL_TXTH_Pos) |
                    (u32RxThreshold << SPI_FIFOCTL_RXTH_Pos));

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

    if(spi == SPI0)
    {
        if((CLK->CLKSEL1 & CLK_CLKSEL1_SPI0SEL_Msk) == CLK_CLKSEL1_SPI0SEL_HCLK)
            u32ClkSrc = CLK_GetHCLKFreq();
        else
            u32ClkSrc = CLK_GetPLLClockFreq();
    }
    if(spi == SPI1)
    {
        if((CLK->CLKSEL1 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_HCLK)
            u32ClkSrc = CLK_GetHCLKFreq();
        else
            u32ClkSrc = CLK_GetPLLClockFreq();
    }
    if(spi == SPI2)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL1_SPI2SEL_Msk) == CLK_CLKSEL1_SPI2SEL_HCLK)
            u32ClkSrc = CLK_GetHCLKFreq();
        else
            u32ClkSrc = CLK_GetPLLClockFreq();
    }
    else
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI3SEL_Msk) == CLK_CLKSEL2_SPI3SEL_HCLK)
            u32ClkSrc = CLK_GetHCLKFreq();
        else
            u32ClkSrc = CLK_GetPLLClockFreq();
    }

    u32Div = spi->CLKDIV & SPI_CLKDIV_DIVIDER_Msk;
    return (u32ClkSrc / (u32Div + 1));
}

/**
  * @brief Enable FIFO related interrupts specified by u32Mask parameter.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be enabled. Valid values are:
  *           - \ref SPI_IE_MASK
  *           - \ref SPI_SSTAIEN_MASK
  *           - \ref SPI_FIFO_TXTHIEN_MASK
  *           - \ref SPI_FIFO_RXTHIEN_MASK
  *           - \ref SPI_FIFO_RXOVIEN_MASK
  *           - \ref SPI_FIFO_TIMEOUIEN_MASK
  * @return none
  */
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask)
{
    if((u32Mask & SPI_IE_MASK) == SPI_IE_MASK)
        spi->CTL |= SPI_CTL_UNITIEN_Msk;

    if((u32Mask & SPI_SSTAIEN_MASK) == SPI_SSTAIEN_MASK)
        spi->SSCTL |= SPI_SSCTL_SSTAIEN_Msk;

    if((u32Mask & SPI_FIFO_TXTHIEN_MASK) == SPI_FIFO_TXTHIEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_TXTHIEN_Msk;

    if((u32Mask & SPI_FIFO_RXTHIEN_MASK) == SPI_FIFO_RXTHIEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXTHIEN_Msk;

    if((u32Mask & SPI_FIFO_RXOVIEN_MASK) == SPI_FIFO_RXOVIEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXOVIEN_Msk;

    if((u32Mask & SPI_FIFO_TIMEOUIEN_MASK) == SPI_FIFO_TIMEOUIEN_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXTOIEN_Msk;
}

/**
  * @brief Disable FIFO related interrupts specified by u32Mask parameter.
  * @param[in]  spi is the base address of SPI module.
  * @param[in]  u32Mask is the combination of all related interrupt enable bits.
  *         Each bit corresponds to a interrupt bit.
  *         This parameter decides which interrupts will be enabled. Valid values are:
  *           - \ref SPI_IE_MASK
  *           - \ref SPI_SSTAIEN_MASK
  *           - \ref SPI_FIFO_TXTHIEN_MASK
  *           - \ref SPI_FIFO_RXTHIEN_MASK
  *           - \ref SPI_FIFO_RXOVIEN_MASK
  *           - \ref SPI_FIFO_TIMEOUIEN_MASK
  * @return none
  */
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask)
{
    if((u32Mask & SPI_IE_MASK) == SPI_IE_MASK)
        spi->CTL &= ~SPI_CTL_UNITIEN_Msk;

    if((u32Mask & SPI_SSTAIEN_MASK) == SPI_SSTAIEN_MASK)
        spi->SSCTL &= ~SPI_SSCTL_SSTAIEN_Msk;

    if((u32Mask & SPI_FIFO_TXTHIEN_MASK) == SPI_FIFO_TXTHIEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_TXTHIEN_Msk;

    if((u32Mask & SPI_FIFO_RXTHIEN_MASK) == SPI_FIFO_RXTHIEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTHIEN_Msk;

    if((u32Mask & SPI_FIFO_RXOVIEN_MASK) == SPI_FIFO_RXOVIEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXOVIEN_Msk;

    if((u32Mask & SPI_FIFO_TIMEOUIEN_MASK) == SPI_FIFO_TIMEOUIEN_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTOIEN_Msk;
}

/**
  * @brief Enable wake-up function.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_EnableWakeup(SPI_T *spi)
{
    spi->CTL |= SPI_CTL_WKCLKEN_Msk;
}

/**
  * @brief Disable wake-up function.
  * @param[in]  spi is the base address of SPI module.
  * @return none
  */
void SPI_DisableWakeup(SPI_T *spi)
{
    spi->CTL &= ~SPI_CTL_WKCLKEN_Msk;
}

/*@}*/ /* end of group NANO103_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_SPI_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
