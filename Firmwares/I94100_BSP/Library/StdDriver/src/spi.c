/**************************************************************************//**
 * @file     spi.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/03/14 10:26a $
 * @brief    I94100 series SPI driver source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_SPI_Driver SPI Driver
  @{
*/

/** @addtogroup I94100_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  This function make SPI module be ready to transfer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32MasterSlave Decides the SPI module is operating in master mode or in slave mode. (SPI_SLAVE, SPI_MASTER)
  * @param[in]  u32SPIMode Decides the transfer timing. (SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3)
  * @param[in]  u32DataWidth Decides the data width of a SPI transaction.
  * @param[in]  u32BusClock The expected frequency of SPI bus clock in Hz.
  * @return Actual frequency of SPI peripheral clock.
  * @details By default, the SPI transfer sequence is MSB first, the slave selection signal is active low and the automatic
  *          slave selection function is disabled.
  *          In Slave mode, the u32BusClock shall be NULL and the SPI clock divider setting will be 0.
  *          The actual clock rate may be different from the target SPI clock rate.
  *          For example, if the SPI source clock rate is 12MHz and the target SPI bus clock rate is 7MHz, the
  *          actual SPI clock rate will be 6MHz.
  * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note   If u32BusClock >= system clock frequency, SPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
  * @note   If u32BusClock >= SPI peripheral clock source, DIVIDER will be set to 0.
  * @note   In slave mode, the SPI peripheral clock rate will be equal to APB clock rate.
  */
uint32_t SPI_Open(SPI_T *spi,
                  uint32_t u32MasterSlave,
                  uint32_t u32SPIMode,
                  uint32_t u32DataWidth,
                  uint32_t u32BusClock)
{
    uint32_t u32HCLKFreq;

	/* Disable I2S mode */
    if((spi == SPI1) || (spi == SPI2))
        spi->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;

    if(u32DataWidth >= 32)
        u32DataWidth = 0;

    if(u32MasterSlave == SPI_MASTER)
    {
		return SPI_SetBusClock(spi, u32BusClock);
    }
    else     /* For slave mode, force the SPI peripheral clock rate to equal APB clock rate. */
    {
		/* Get system clock frequency */
		u32HCLKFreq = CLK_GetHCLKFreq();
	
        /* Default setting: slave selection signal is low level active. */
        spi->SSCTL = SPI_SS_ACTIVE_LOW;

        /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
        spi->CTL = u32MasterSlave | (u32DataWidth << SPI_CTL_DWIDTH_Pos) | (u32SPIMode) | SPI_CTL_SPIEN_Msk;

        /* Set DIVIDER = 0 */
        spi->CLKDIV = 0;

        /* Select PCLK as the clock source of SPI */
        if(spi == SPI0)
        {
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK0;
            /* Return slave peripheral clock rate */
            //if((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk) == CLK_CLKSEL0_PCLK0SEL_HCLK_DIV2)
                //return (u32HCLKFreq / 2);
            //else
                return u32HCLKFreq;
        }
        else if(spi == SPI1)
        {
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK1;
            /* Return slave peripheral clock rate */
            //if((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK1SEL_Msk) == CLK_CLKSEL0_PCLK1SEL_HCLK_DIV2)
               // return (u32HCLKFreq / 2);
            //else
                return u32HCLKFreq;
        }
        else
        {
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI2SEL_Msk)) | CLK_CLKSEL2_SPI2SEL_PCLK0;
            /* Return slave peripheral clock rate */
            //if((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk) == CLK_CLKSEL0_PCLK0SEL_HCLK_DIV2)
                //return (u32HCLKFreq / 2);
            //else
                return u32HCLKFreq;
        }
    }
}

/**
  * @brief  Disable SPI controller.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return None
  */
void SPI_Close(SPI_T *spi)
{
}

/**
  * @brief  Clear RX FIFO buffer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return None
  * @details This function will clear SPI RX FIFO buffer. The RXEMPTY (SPI_STATUS[8]) will be set to 1.
  */
void SPI_ClearRxFIFO(SPI_T *spi)
{
    spi->FIFOCTL |= SPI_FIFOCTL_RXFBCLR_Msk;
}

/**
  * @brief  Clear TX FIFO buffer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return None
  * @details This function will clear SPI TX FIFO buffer. The TXEMPTY (SPI_STATUS[16]) will be set to 1.
  * @note The TX shift register will not be cleared.
  */
void SPI_ClearTxFIFO(SPI_T *spi)
{
    spi->FIFOCTL |= SPI_FIFOCTL_TXFBCLR_Msk;
}

/**
  * @brief  Disable the automatic slave selection function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return None
  * @details This function will disable the automatic slave selection function and set slave selection signal to inactive state.
  */
void SPI_DisableAutoSS(SPI_T *spi)
{
    spi->SSCTL &= ~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SS0_Msk | SPI_SSCTL_SS1_Msk );
}

/**
  * @brief  Enable the automatic slave selection function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32SSPinMask Specifies slave selection pins. (SPI_SS0/SPI_SS1)
  * @param[in]  u32ActiveLevel Specifies the active level of slave selection signal. (SPI_SS_ACTIVE_HIGH, SPI_SS_ACTIVE_LOW)
  * @return None
  * @details This function will enable the automatic slave selection function. Only available in Master mode.
  *          The slave selection pin and the active level will be set in this function.
  */
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    spi->SSCTL = (spi->SSCTL & (~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SSACTPOL_Msk | SPI_SSCTL_SS0_Msk | SPI_SSCTL_SS1_Msk ))) | (u32SSPinMask | u32ActiveLevel | SPI_SSCTL_AUTOSS_Msk);
}

/**
  * @brief  Set the SPI bus clock.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32BusClock The expected frequency of SPI bus clock in Hz.
  * @return Actual frequency of SPI bus clock.
  * @details This function is only available in Master mode. The actual clock rate may be different from the target SPI bus clock rate.
  *          For example, if the SPI source clock rate is 12MHz and the target SPI bus clock rate is 7MHz, the actual SPI bus clock
  *          rate will be 6MHz.
  * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note   If u32BusClock >= system clock frequency, SPI peripheral clock source will be set to APB clock and DIVIDER will be set to 0.
  * @note   If u32BusClock >= SPI peripheral clock source, DIVIDER will be set to 0.
  */
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock)
{
    uint32_t u32ClkSrc, u32HCLKFreq;
    uint32_t u32Div;

    /* Get system clock frequency */
    u32HCLKFreq = CLK_GetHCLKFreq();

    if(u32BusClock >= u32HCLKFreq)
    {
        /* Select PCLK as the clock source of SPI */
        if(spi == SPI0)
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI0SEL_Msk)) | CLK_CLKSEL2_SPI0SEL_PCLK0;
        else if(spi == SPI1)
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK1;
        else
            CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI2SEL_Msk)) | CLK_CLKSEL2_SPI2SEL_PCLK0;
    }

    /* Check clock source of SPI */
    if(spi == SPI0)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_HXT)
            u32ClkSrc = __HXT; /* Clock source is HXT */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_PLL)
            u32ClkSrc = CLK_GetPLLClockFreq(); /* Clock source is PLL */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_PCLK0)
        {
            /* Clock source is PCLK0 */
            //if((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk) == CLK_CLKSEL0_PCLK0SEL_HCLK_DIV2)
                //u32ClkSrc = (u32HCLKFreq / 2);
            //else
                u32ClkSrc = u32HCLKFreq;
        }
        else
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
    }
    else if(spi == SPI1)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_HXT)
            u32ClkSrc = __HXT; /* Clock source is HXT */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_PLL)
            u32ClkSrc = CLK_GetPLLClockFreq(); /* Clock source is PLL */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_PCLK1)
        {
            /* Clock source is PCLK1 */
            //if((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK1SEL_Msk) == CLK_CLKSEL0_PCLK1SEL_HCLK_DIV2)
                //u32ClkSrc = (u32HCLKFreq / 2);
            //else
                u32ClkSrc = u32HCLKFreq;
        }
        else
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
    }
    else
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_HXT)
            u32ClkSrc = __HXT; /* Clock source is HXT */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_PLL)
            u32ClkSrc = CLK_GetPLLClockFreq(); /* Clock source is PLL */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_PCLK0)
        {
            /* Clock source is PCLK0 */
            //if((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk) == CLK_CLKSEL0_PCLK0SEL_HCLK_DIV2)
                //u32ClkSrc = (u32HCLKFreq / 2);
            //else
                u32ClkSrc = u32HCLKFreq;
        }
        else
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
    }

    if(u32BusClock >= u32HCLKFreq)
    {
        /* Set DIVIDER = 0 */
        spi->CLKDIV = 0;
        /* Return master peripheral clock rate */
        return u32ClkSrc;
    }
    else if(u32BusClock >= u32ClkSrc)
    {
        /* Set DIVIDER = 0 */
        spi->CLKDIV = 0;
        /* Return master peripheral clock rate */
        return u32ClkSrc;
    }
    else if(u32BusClock == 0)
    {
        /* Set DIVIDER to the maximum value 0xFF. f_spi = f_spi_clk_src / (DIVIDER + 1) */
        spi->CLKDIV |= SPI_CLKDIV_DIVIDER_Msk;
        /* Return master peripheral clock rate */
        return (u32ClkSrc / (0xFF + 1));
    }
    else
    {
        u32Div = (((u32ClkSrc * 10) / u32BusClock + 5) / 10) - 1; /* Round to the nearest integer */
        if(u32Div > 0xFF)
        {
            u32Div = 0xFF;
            spi->CLKDIV |= SPI_CLKDIV_DIVIDER_Msk;
            /* Return master peripheral clock rate */
            return (u32ClkSrc / (0xFF + 1));
        }
        else
        {
            spi->CLKDIV = (spi->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | (u32Div << SPI_CLKDIV_DIVIDER_Pos);
            /* Return master peripheral clock rate */
            return (u32ClkSrc / (u32Div + 1));
        }
    }
}

/**
  * @brief  Configure FIFO threshold setting.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32TxThreshold Decides the TX FIFO threshold. For SPI0, it could be 0 ~ 7. For SPI1 and SPI2, it could be 0 ~ 3.
  * @param[in]  u32RxThreshold Decides the RX FIFO threshold. For SPI0, it could be 0 ~ 7. For SPI1 and SPI2, it could be 0 ~ 3.
  * @return None
  * @details Set TX FIFO threshold and RX FIFO threshold configurations.
  */
void SPI_SetFIFO(SPI_T *spi, uint32_t u32TxThreshold, uint32_t u32RxThreshold)
{
    spi->FIFOCTL = (spi->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk) |
                    (u32TxThreshold << SPI_FIFOCTL_TXTH_Pos) |
                    (u32RxThreshold << SPI_FIFOCTL_RXTH_Pos));
}

/**
  * @brief  Get the actual frequency of SPI bus clock. Only available in Master mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return Actual SPI bus clock frequency in Hz.
  * @details This function will calculate the actual SPI bus clock rate according to the SPInSEL and DIVIDER settings. Only available in Master mode.
  */
uint32_t SPI_GetBusClock(SPI_T *spi)
{
    uint32_t u32Div;
    uint32_t u32ClkSrc, u32HCLKFreq;

    /* Get DIVIDER setting */
    u32Div = (spi->CLKDIV & SPI_CLKDIV_DIVIDER_Msk) >> SPI_CLKDIV_DIVIDER_Pos;

    /* Get system clock frequency */
    u32HCLKFreq = CLK_GetHCLKFreq();

    /* Check clock source of SPI */
    if(spi == SPI0)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_HXT)
            u32ClkSrc = __HXT; /* Clock source is HXT */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_PLL)
            u32ClkSrc = CLK_GetPLLClockFreq(); /* Clock source is PLL */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI0SEL_Msk) == CLK_CLKSEL2_SPI0SEL_PCLK0)
        {
            /* Clock source is PCLK0 */
            //if((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk) == CLK_CLKSEL0_PCLK0SEL_HCLK_DIV2)
                //u32ClkSrc = (u32HCLKFreq / 2);
            //else
                u32ClkSrc = u32HCLKFreq;
        }
        else
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
    }
    else if(spi == SPI1)
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_HXT)
            u32ClkSrc = __HXT; /* Clock source is HXT */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_PLL)
            u32ClkSrc = CLK_GetPLLClockFreq(); /* Clock source is PLL */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI1SEL_Msk) == CLK_CLKSEL2_SPI1SEL_PCLK1)
        {
            /* Clock source is PCLK1 */
            //if((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK1SEL_Msk) == CLK_CLKSEL0_PCLK1SEL_HCLK_DIV2)
                //u32ClkSrc = (u32HCLKFreq / 2);
            //else
                u32ClkSrc = u32HCLKFreq;
        }
        else
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
    }
    else
    {
        if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_HXT)
            u32ClkSrc = __HXT; /* Clock source is HXT */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_PLL)
            u32ClkSrc = CLK_GetPLLClockFreq(); /* Clock source is PLL */
        else if((CLK->CLKSEL2 & CLK_CLKSEL2_SPI2SEL_Msk) == CLK_CLKSEL2_SPI2SEL_PCLK0)
        {
            /* Clock source is PCLK0 */
            //if((CLK->CLKSEL0 & CLK_CLKSEL0_PCLK0SEL_Msk) == CLK_CLKSEL0_PCLK0SEL_HCLK_DIV2)
                //u32ClkSrc = (u32HCLKFreq / 2);
            //else
                u32ClkSrc = u32HCLKFreq;
        }
        else
            u32ClkSrc = __HIRC; /* Clock source is HIRC */
    }

    /* Return SPI bus clock rate */
    return (u32ClkSrc / (u32Div + 1));
}

/**
  * @brief  Enable interrupt function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt enable bit.
  *                     This parameter decides which interrupts will be enabled. It is combination of:
  *                       - \ref SPI_UNIT_INT_MASK
  *                       - \ref SPI_SSACT_INT_MASK
  *                       - \ref SPI_SSINACT_INT_MASK
  *                       - \ref SPI_SLVUR_INT_MASK
  *                       - \ref SPI_SLVBE_INT_MASK
  *                       - \ref SPI_SLVTO_INT_MASK
  *                       - \ref SPI_TXUF_INT_MASK
  *                       - \ref SPI_FIFO_TXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXOV_INT_MASK
  *                       - \ref SPI_FIFO_RXTO_INT_MASK
  *
  * @return None
  * @details Enable SPI related interrupts specified by u32Mask parameter.
  */
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask)
{
    /* Enable unit transfer interrupt flag */
    if((u32Mask & SPI_UNIT_INT_MASK) == SPI_UNIT_INT_MASK)
        spi->CTL |= SPI_CTL_UNITIEN_Msk;

    /* Enable slave selection signal active interrupt flag */
    if((u32Mask & SPI_SSACT_INT_MASK) == SPI_SSACT_INT_MASK)
        spi->SSCTL |= SPI_SSCTL_SSACTIEN_Msk;

    /* Enable slave selection signal inactive interrupt flag */
    if((u32Mask & SPI_SSINACT_INT_MASK) == SPI_SSINACT_INT_MASK)
        spi->SSCTL |= SPI_SSCTL_SSINAIEN_Msk;

    /* Enable slave TX under run interrupt flag */
    if((u32Mask & SPI_SLVUR_INT_MASK) == SPI_SLVUR_INT_MASK)
        spi->SSCTL |= SPI_SSCTL_SLVURIEN_Msk;

    /* Enable slave bit count error interrupt flag */
    if((u32Mask & SPI_SLVBE_INT_MASK) == SPI_SLVBE_INT_MASK)
        spi->SSCTL |= SPI_SSCTL_SLVBEIEN_Msk;

    /* Enable slave time-out interrupt flag */
    if((u32Mask & SPI_SLVTO_INT_MASK) == SPI_SLVTO_INT_MASK)
        spi->SSCTL |= SPI_SSCTL_SLVTOIEN_Msk;

    /* Enable slave TX underflow interrupt flag */
    if((u32Mask & SPI_TXUF_INT_MASK) == SPI_TXUF_INT_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_TXUFIEN_Msk;

    /* Enable TX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_TXTH_INT_MASK) == SPI_FIFO_TXTH_INT_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_TXTHIEN_Msk;

    /* Enable RX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_RXTH_INT_MASK) == SPI_FIFO_RXTH_INT_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXTHIEN_Msk;

    /* Enable RX overrun interrupt flag */
    if((u32Mask & SPI_FIFO_RXOV_INT_MASK) == SPI_FIFO_RXOV_INT_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXOVIEN_Msk;

    /* Enable RX time-out interrupt flag */
    if((u32Mask & SPI_FIFO_RXTO_INT_MASK) == SPI_FIFO_RXTO_INT_MASK)
        spi->FIFOCTL |= SPI_FIFOCTL_RXTOIEN_Msk;
}

/**
  * @brief  Disable interrupt function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt bit.
  *                     This parameter decides which interrupts will be disabled. It is combination of:
  *                       - \ref SPI_UNIT_INT_MASK
  *                       - \ref SPI_SSACT_INT_MASK
  *                       - \ref SPI_SSINACT_INT_MASK
  *                       - \ref SPI_SLVUR_INT_MASK
  *                       - \ref SPI_SLVBE_INT_MASK
  *                       - \ref SPI_SLVTO_INT_MASK
  *                       - \ref SPI_TXUF_INT_MASK
  *                       - \ref SPI_FIFO_TXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXOV_INT_MASK
  *                       - \ref SPI_FIFO_RXTO_INT_MASK
  *
  * @return None
  * @details Disable SPI related interrupts specified by u32Mask parameter.
  */
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask)
{
    /* Disable unit transfer interrupt flag */
    if((u32Mask & SPI_UNIT_INT_MASK) == SPI_UNIT_INT_MASK)
        spi->CTL &= ~SPI_CTL_UNITIEN_Msk;

    /* Disable slave selection signal active interrupt flag */
    if((u32Mask & SPI_SSACT_INT_MASK) == SPI_SSACT_INT_MASK)
        spi->SSCTL &= ~SPI_SSCTL_SSACTIEN_Msk;

    /* Disable slave selection signal inactive interrupt flag */
    if((u32Mask & SPI_SSINACT_INT_MASK) == SPI_SSINACT_INT_MASK)
        spi->SSCTL &= ~SPI_SSCTL_SSINAIEN_Msk;

    /* Disable slave TX under run interrupt flag */
    if((u32Mask & SPI_SLVUR_INT_MASK) == SPI_SLVUR_INT_MASK)
        spi->SSCTL &= ~SPI_SSCTL_SLVURIEN_Msk;

    /* Disable slave bit count error interrupt flag */
    if((u32Mask & SPI_SLVBE_INT_MASK) == SPI_SLVBE_INT_MASK)
        spi->SSCTL &= ~SPI_SSCTL_SLVBEIEN_Msk;

    /* Disable slave time-out interrupt flag */
    if((u32Mask & SPI_SLVTO_INT_MASK) == SPI_SLVTO_INT_MASK)
        spi->SSCTL &= ~SPI_SSCTL_SLVTOIEN_Msk;

    /* Disable slave TX underflow interrupt flag */
    if((u32Mask & SPI_TXUF_INT_MASK) == SPI_TXUF_INT_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_TXUFIEN_Msk;

    /* Disable TX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_TXTH_INT_MASK) == SPI_FIFO_TXTH_INT_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_TXTHIEN_Msk;

    /* Disable RX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_RXTH_INT_MASK) == SPI_FIFO_RXTH_INT_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTHIEN_Msk;

    /* Disable RX overrun interrupt flag */
    if((u32Mask & SPI_FIFO_RXOV_INT_MASK) == SPI_FIFO_RXOV_INT_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXOVIEN_Msk;

    /* Disable RX time-out interrupt flag */
    if((u32Mask & SPI_FIFO_RXTO_INT_MASK) == SPI_FIFO_RXTO_INT_MASK)
        spi->FIFOCTL &= ~SPI_FIFOCTL_RXTOIEN_Msk;
}

/**
  * @brief  Get interrupt flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be read. It is combination of:
  *                       - \ref SPI_UNIT_INT_MASK
  *                       - \ref SPI_SSACT_INT_MASK
  *                       - \ref SPI_SSINACT_INT_MASK
  *                       - \ref SPI_SLVUR_INT_MASK
  *                       - \ref SPI_SLVBE_INT_MASK
  *                       - \ref SPI_SLVTO_INT_MASK
  *                       - \ref SPI_TXUF_INT_MASK
  *                       - \ref SPI_FIFO_TXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXTH_INT_MASK
  *                       - \ref SPI_FIFO_RXOV_INT_MASK
  *                       - \ref SPI_FIFO_RXTO_INT_MASK
  *
  * @return Interrupt flags of selected sources.
  * @details Get SPI related interrupt flags specified by u32Mask parameter.
  */
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask)
{
    uint32_t u32IntFlag = 0;

    /* Check unit transfer interrupt flag */
    if((u32Mask & SPI_UNIT_INT_MASK) && (spi->STATUS & SPI_STATUS_UNITIF_Msk))
        u32IntFlag |= SPI_UNIT_INT_MASK;

    /* Check slave selection signal active interrupt flag */
    if((u32Mask & SPI_SSACT_INT_MASK) && (spi->STATUS & SPI_STATUS_SSACTIF_Msk))
        u32IntFlag |= SPI_SSACT_INT_MASK;

    /* Check slave selection signal inactive interrupt flag */
    if((u32Mask & SPI_SSINACT_INT_MASK) && (spi->STATUS & SPI_STATUS_SSINAIF_Msk))
        u32IntFlag |= SPI_SSINACT_INT_MASK;

    /* Check slave TX under run interrupt flag */
    if((u32Mask & SPI_SLVUR_INT_MASK) && (spi->STATUS & SPI_STATUS_SLVURIF_Msk))
        u32IntFlag |= SPI_SLVUR_INT_MASK;

    /* Check slave bit count error interrupt flag */
    if((u32Mask & SPI_SLVBE_INT_MASK) && (spi->STATUS & SPI_STATUS_SLVBEIF_Msk))
        u32IntFlag |= SPI_SLVBE_INT_MASK;

    /* Check slave time-out interrupt flag */
    if((u32Mask & SPI_SLVTO_INT_MASK) && (spi->STATUS & SPI_STATUS_SLVTOIF_Msk))
        u32IntFlag |= SPI_SLVTO_INT_MASK;

    /* Check slave TX underflow interrupt flag */
    if((u32Mask & SPI_TXUF_INT_MASK) && (spi->STATUS & SPI_STATUS_TXUFIF_Msk))
        u32IntFlag |= SPI_TXUF_INT_MASK;

    /* Check TX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_TXTH_INT_MASK) && (spi->STATUS & SPI_STATUS_TXTHIF_Msk))
        u32IntFlag |= SPI_FIFO_TXTH_INT_MASK;

    /* Check RX threshold interrupt flag */
    if((u32Mask & SPI_FIFO_RXTH_INT_MASK) && (spi->STATUS & SPI_STATUS_RXTHIF_Msk))
        u32IntFlag |= SPI_FIFO_RXTH_INT_MASK;

    /* Check RX overrun interrupt flag */
    if((u32Mask & SPI_FIFO_RXOV_INT_MASK) && (spi->STATUS & SPI_STATUS_RXOVIF_Msk))
        u32IntFlag |= SPI_FIFO_RXOV_INT_MASK;

    /* Check RX time-out interrupt flag */
    if((u32Mask & SPI_FIFO_RXTO_INT_MASK) && (spi->STATUS & SPI_STATUS_RXTOIF_Msk))
        u32IntFlag |= SPI_FIFO_RXTO_INT_MASK;

    return u32IntFlag;
}

/**
  * @brief  Clear interrupt flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flags will be cleared. It could be the combination of:
  *                       - \ref SPI_UNIT_INT_MASK
  *                       - \ref SPI_SSACT_INT_MASK
  *                       - \ref SPI_SSINACT_INT_MASK
  *                       - \ref SPI_SLVUR_INT_MASK
  *                       - \ref SPI_SLVBE_INT_MASK
  *                       - \ref SPI_SLVTO_INT_MASK 
  *                       - \ref SPI_TXUF_INT_MASK 
  *                       - \ref SPI_FIFO_RXOV_INT_MASK 
  *                       - \ref SPI_FIFO_RXTO_INT_MASK
  *
  * @return None
  * @details Clear SPI related interrupt flags specified by u32Mask parameter.
  */
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask)
{
    if(u32Mask & SPI_UNIT_INT_MASK)
        spi->STATUS = SPI_STATUS_UNITIF_Msk; /* Clear unit transfer interrupt flag */

    if(u32Mask & SPI_SSACT_INT_MASK)
        spi->STATUS = SPI_STATUS_SSACTIF_Msk; /* Clear slave selection signal active interrupt flag */

    if(u32Mask & SPI_SSINACT_INT_MASK)
        spi->STATUS = SPI_STATUS_SSINAIF_Msk; /* Clear slave selection signal inactive interrupt flag */

    if(u32Mask & SPI_SLVUR_INT_MASK)
        spi->STATUS = SPI_STATUS_SLVURIF_Msk; /* Clear slave TX under run interrupt flag */

    if(u32Mask & SPI_SLVBE_INT_MASK)
        spi->STATUS = SPI_STATUS_SLVBEIF_Msk; /* Clear slave bit count error interrupt flag */

    if(u32Mask & SPI_SLVTO_INT_MASK)
        spi->STATUS = SPI_STATUS_SLVTOIF_Msk; /* Clear slave time-out interrupt flag */

    if(u32Mask & SPI_TXUF_INT_MASK)
        spi->STATUS = SPI_STATUS_TXUFIF_Msk; /* Clear slave TX underflow interrupt flag */

    if(u32Mask & SPI_FIFO_RXOV_INT_MASK)
        spi->STATUS = SPI_STATUS_RXOVIF_Msk; /* Clear RX overrun interrupt flag */

    if(u32Mask & SPI_FIFO_RXTO_INT_MASK)
        spi->STATUS = SPI_STATUS_RXTOIF_Msk; /* Clear RX time-out interrupt flag */
}

/**
  * @brief  Get SPI status.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related sources.
  *                     Each bit corresponds to a source.
  *                     This parameter decides which flags will be read. It is combination of:
  *                       - \ref SPI_BUSY_MASK
  *                       - \ref SPI_RX_EMPTY_MASK
  *                       - \ref SPI_RX_FULL_MASK
  *                       - \ref SPI_TX_EMPTY_MASK
  *                       - \ref SPI_TX_FULL_MASK
  *                       - \ref SPI_TXRX_RESET_MASK
  *                       - \ref SPI_SPIEN_STS_MASK
  *                       - \ref SPI_SSLINE_STS_MASK
  *
  * @return Flags of selected sources.
  * @details Get SPI related status specified by u32Mask parameter.
  */
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask)
{
    uint32_t u32Flag = 0;

    /* Check busy status */
    if((u32Mask & SPI_BUSY_MASK) && (spi->STATUS & SPI_STATUS_BUSY_Msk))
        u32Flag |= SPI_BUSY_MASK;

    /* Check RX empty flag */
    if((u32Mask & SPI_RX_EMPTY_MASK) && (spi->STATUS & SPI_STATUS_RXEMPTY_Msk))
        u32Flag |= SPI_RX_EMPTY_MASK;

    /* Check RX full flag */
    if((u32Mask & SPI_RX_FULL_MASK) && (spi->STATUS & SPI_STATUS_RXFULL_Msk))
        u32Flag |= SPI_RX_FULL_MASK;

    /* Check TX empty flag */
    if((u32Mask & SPI_TX_EMPTY_MASK) && (spi->STATUS & SPI_STATUS_TXEMPTY_Msk))
        u32Flag |= SPI_TX_EMPTY_MASK;

    /* Check TX full flag */
    if((u32Mask & SPI_TX_FULL_MASK) && (spi->STATUS & SPI_STATUS_TXFULL_Msk))
        u32Flag |= SPI_TX_FULL_MASK;

    /* Check TX/RX reset flag */
    if((u32Mask & SPI_TXRX_RESET_MASK) && (spi->STATUS & SPI_STATUS_TXRXRST_Msk))
        u32Flag |= SPI_TXRX_RESET_MASK;

    /* Check SPIEN flag */
    if((u32Mask & SPI_SPIEN_STS_MASK) && (spi->STATUS & SPI_STATUS_SPIENSTS_Msk))
        u32Flag |= SPI_SPIEN_STS_MASK;

    /* Check SPIn_SS line status */
    if((u32Mask & SPI_SSLINE_STS_MASK) && (spi->STATUS & SPI_STATUS_SSLINE_Msk))
        u32Flag |= SPI_SSLINE_STS_MASK;

    return u32Flag;
}

/**
  * @brief  	This function configures some parameters of SPI_I2S interface for general purpose use.
  * @param[in] 	u32MasterSlave I2S operation mode. Valid values are listed below.
  *            	- \ref SPI_I2SMASTER
  *				- \ref SPI_I2SSLAVE
  * @param[in] 	u32SampleRate Sample rate
  * @param[in] 	u32WordWidth Data length. Valid values are listed below.
  * 			- \ref SPI_I2SDATABIT_8
  * 			- \ref SPI_I2SDATABIT_16
  *				- \ref SPI_I2SDATABIT_24
  *				- \ref SPI_I2SDATABIT_32
  * @param[in] 	u32Channels Audio format. Valid values are listed below.
  * 			- \ref SPI_I2SMONO
  *				- \ref SPI_I2SSTEREO
  * @param[in] 	u32DataFormat Data format. Valid values are listed below.
  *				- \ref SPI_I2SFORMAT_I2S
  *				- \ref SPI_I2SFORMAT_MSB
  *				- \ref SPI_I2SFORMAT_PCMA
  *				- \ref SPI_I2SFORMAT_PCMB
  * @return 	Real sample rate of master mode or peripheral clock rate of slave mode.
  * @details 	This function will reset SPI/I2S controller and configure I2S controller according to the input parameters.
  *          	Set TX and RX FIFO threshold to middle value. Both the TX and RX functions will be enabled.
  *          	The actual sample rate may be different from the target sample rate. The real sample rate will be returned for reference.
  * @note   	Only SPI1 and SPI2 support I2S mode.
  * @note   	In slave mode, the SPI peripheral clock rate will be equal to APB clock rate.
  */
uint32_t SPI_I2SOpen(SPI_T *spi, 
					 uint32_t u32MasterSlave, 
					 uint32_t u32SampleRate, 
					 uint32_t u32WordWidth, 
					 uint32_t u32Mono, 
					 uint32_t u32DataFormat)
{
	uint32_t u32Divider, u32BitRate, u32SrcClk;
	
	/* Reset SPI/I2S */
	if(spi == SPI1)
		SYS_ResetModule(SPI1_RST);
	else if(spi == SPI2)
		SYS_ResetModule(SPI2_RST);
	
	/* Configure I2S controller */
    spi->I2SCTL = u32MasterSlave | u32WordWidth | u32Mono | u32DataFormat;
	
	 if(u32MasterSlave == SPI_I2SMASTER)
    {
		uint32_t u32Error, u32Error2;
		
        /* Get the source clock rate */
		u32Divider = (spi->CLKDIV&SPI_CLKDIV_DIVIDER_Msk) + 1;
        u32SrcClk = SPI_GetBusClock(spi)*u32Divider;

		/* Calculate the parameter */
		u32WordWidth = (u32WordWidth>=SPI_I2SDATABIT_24)?32:(((u32WordWidth >> SPI_I2SCTL_WDWIDTH_Pos) + 1) * 8);
		u32Mono = (u32Mono == SPI_I2SMONO?1:2);
		
        /* Calculate the bit clock rate */
        u32BitRate = u32SampleRate * u32WordWidth * u32Mono;
        u32Divider = ((u32SrcClk / u32BitRate) >> 1) - 1;
		/* Adjust Error */
		u32Error = (u32SrcClk/(2*(u32Divider+1)) - u32BitRate);
		u32Error2 = (u32BitRate - u32SrcClk/(2*(u32Divider+2)));
		u32Divider = (u32Error < u32Error2)?u32Divider:(u32Divider+1);
		
        /* Set BCLKDIV setting */
        spi->I2SCLK = (spi->I2SCLK & ~SPI_I2SCLK_BCLKDIV_Msk) | (u32Divider << SPI_I2SCLK_BCLKDIV_Pos);

        /* Calculate real bit clock rate */
        u32BitRate = u32SrcClk / ((u32Divider + 1) * 2);
        /* Calculate real sample rate */
        u32SampleRate = u32BitRate / u32WordWidth / u32Mono;
    }
	else
    {
        /* Set BCLKDIV = 0 */
        spi->I2SCLK &= ~SPI_I2SCLK_BCLKDIV_Msk;
		/* Get the source clock rate */
        u32SampleRate = SPI_GetBusClock(spi);
    }
	
	return u32SampleRate;
}

/**
  * @brief  	Disable I2S function.
  * @param[in]  i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	Disable I2S function.
  */
void SPI_I2SClose(SPI_T *spi)
{
    spi->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;
}

/**
  * @brief  	Enable master clock (MCLK).
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32BusClock The target MCLK clock rate.
  * @return 	Actual MCLK clock rate
  * @details 	Set the master clock rate according to u32BusClock parameter and enable master clock output.
  *          	The actual master clock rate may be different from the target master clock rate. The real master clock rate will be returned for reference.
  */
uint32_t SPI_I2SEnableMCLK(SPI_T *spi, uint32_t u32MasterClock)
{
    uint32_t u32Divider;
    uint32_t u32SrcClk;

    /* Get the source clock rate */
	u32Divider = (spi->CLKDIV&SPI_CLKDIV_DIVIDER_Msk) + 1;
	u32SrcClk = SPI_GetBusClock(spi)*u32Divider;
	//u32SrcClk = SPI_GetBusClock(spi)*(spi->CLKDIV&SPI_CLKDIV_DIVIDER_Msk);
	
    if(u32MasterClock == u32SrcClk)
        u32Divider = 0;
    else
    {
        u32Divider = (u32SrcClk / u32MasterClock) >> 1;
        /* MCLKDIV is a 6-bit width configuration. The maximum value is 0x7F. */
        if(u32Divider > 0x7F)
            u32Divider = 0x7F;
    }

    /* Write u32Divider to MCLKDIV (SPI_I2SCLK[5:0]) */
    spi->I2SCLK = (spi->I2SCLK & ~SPI_I2SCLK_MCLKDIV_Msk) | (u32Divider << SPI_I2SCLK_MCLKDIV_Pos);

    /* Enable MCLK output */
    spi->I2SCTL |= I2S_CTL0_MCLKEN_Msk;

    if(u32Divider == 0)
        return u32SrcClk; /* If MCLKDIV=0, master clock rate is equal to the source clock rate. */
    else
        return ((u32SrcClk >> 1) / u32Divider); /* If MCLKDIV>0, master clock rate = source clock rate / (MCLKDIV * 2) */
}

/**
  * @brief  	Disable master clock (MCLK).
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32BusClock The target MCLK clock rate.
  * @return 	Actual MCLK clock rate
  * @details 	Set the master clock rate according to u32BusClock parameter and enable master clock output.
  *          	The actual master clock rate may be different from the target master clock rate. The real master clock rate will be returned for reference.
  */
void SPI_I2SDisableMCLK(SPI_T *spi)
{
	spi->I2SCTL &= ~SPI_I2SCTL_MCLKEN_Msk;
}

/**
  * @brief 		Enable interrupt function.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @param[in] 	u32Mask The combination of all related interrupt enable bits.
  *            	Each bit corresponds to a interrupt source. Valid values are listed below.
  *            	- \ref SPI_I2S_TXTH_INT_MASK
  *            	- \ref SPI_I2S_RXTH_INT_MASK
  *            	- \ref SPI_I2S_TXOV_INT_MASK
  *            	- \ref SPI_I2S_RXOV_INT_MASK
  *            	- \ref SPI_I2S_RXTO_INT_MASK
  * @return 	None
  * @details 	This function enables the interrupt according to the u32Mask parameter.
  */
void SPI_I2SEnableInt(SPI_T *spi, uint32_t u32Mask)
{
	spi->FIFOCTL |= u32Mask;
}

/**
  * @brief 		Disable interrupt function.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @param[in] 	u32Mask The combination of all related interrupt enable bits.
  *            	Each bit corresponds to a interrupt source. Valid values are listed below.
  *            	- \ref SPI_I2S_TXTH_INT_MASK
  *            	- \ref SPI_I2S_RXTH_INT_MASK
  *            	- \ref SPI_I2S_TXOV_INT_MASK
  *            	- \ref SPI_I2S_RXOV_INT_MASK
  *            	- \ref SPI_I2S_RXTO_INT_MASK
  * @return 	None
  * @details 	This function disables the interrupt according to the u32Mask parameter.
  * @note   	Only SPI1 and SPI2 support I2S mode.
  */
void SPI_I2SDisableInt(SPI_T *spi, uint32_t u32Mask)
{
	spi->FIFOCTL &= ~u32Mask;
}

/**
  * @brief 		Enable I2S control function.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @return 	None
  * @details 	This function enable the control function.
  * @note   	If enable this bit, I2Sx_BCLK will start to output in Master mode.
  * @note		Before changing the configurations of SPIn_I2SCTL, SPIn_I2SCLK, 
  *				and SPIn_FIFOCTL registers, user shall clear the I2SEN (SPIn_I2SCTL[0]) 
  *				and confirm the I2SENSTS (SPIn_I2SSTS[15]) is 0.
  */
void SPI_I2SEnableControl(SPI_T *spi)
{
	/* Enable I2S mode. */
	spi->I2SCTL |= SPI_I2SCTL_I2SEN_Msk;
}

/**
  * @brief 		Disable I2S control function.
  * @param[in] 	spi The pointer of the specified I2S module.
  * @return 	None
  * @details 	This function disable the control function.
  * @note   	If enable this bit, I2Sx_BCLK will start to output in Master mode.
  * @note		Before changing the configurations of SPIn_I2SCTL, SPIn_I2SCLK, 
  *				and SPIn_FIFOCTL registers, user shall clear the I2SEN (SPIn_I2SCTL[0]) 
  *				and confirm the I2SENSTS (SPIn_I2SSTS[15]) is 0.
  */
void SPI_I2SDisableControl(SPI_T *spi)
{
	/* Enable I2S mode. */
	spi->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;
}

/*@}*/ /* end of group I94100_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_SPI_Driver */

/*@}*/ /* end of group I94100_Device_Driver */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
