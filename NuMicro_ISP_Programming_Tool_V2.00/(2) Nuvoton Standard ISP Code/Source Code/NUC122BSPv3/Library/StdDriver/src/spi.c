/**************************************************************************//**
 * @file     spi.c
 * @version  V3.00
 * $Revision: 5 $
 * $Date: 15/07/24 5:39p $
 * @brief    SPI driver source file
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "NUC122.h"
/** @addtogroup Standard_Driver Standard Driver
  @{
*/

/** @addtogroup SPI_Driver SPI Driver
  @{
*/


/** @addtogroup SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  This function make SPI module be ready to transfer.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32MasterSlave Decides the SPI module is operating in master mode or in Slave mode. (SPI_SLAVE, SPI_MASTER)
  * @param[in]  u32SPIMode Decides the transfer timing. (SPI_MODE_0, SPI_MODE_1, SPI_MODE_2, SPI_MODE_3)
  * @param[in]  u32DataWidth Decides the data width of a SPI transaction.
  * @param[in]  u32BusClock The expected frequency of SPI bus clock in Hz. Only available for Master mode.
  * @return Actual frequency of SPI peripheral clock.
  * @details By default, the SPI transfer sequence is MSB first and the automatic slave selection function is disabled.
  *          In Slave mode, the u32BusClock parameter is useless and the SPI clock divider setting will be 0.
  *          The actual clock rate may be different from the target SPI clock rate.
  *          For example, if the SPI source clock rate is 12 MHz and the target SPI bus clock rate is 7 MHz, the
  *          actual SPI clock rate will be 6 MHz.
  * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note   If u32BusClock >= system clock frequency, the SPI peripheral clock rate will be set to a half of system clock rate.
  *         The DIVIDER (SPI_DIVIDER[15:0]) will be set to 0.
  * @note   In Slave mode, the slave selection signal will be set to low-level-active.
  */
uint32_t SPI_Open(SPI_T *spi,
                  uint32_t u32MasterSlave,
                  uint32_t u32SPIMode,
                  uint32_t u32DataWidth,
                  uint32_t u32BusClock)
{
    uint32_t u32Div, u32HCLKFreq;

    if(u32DataWidth == 32)
        u32DataWidth = 0;

    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
    spi->CNTRL = u32MasterSlave | (u32DataWidth << SPI_CNTRL_TX_BIT_LEN_Pos) | (u32SPIMode);

    /* Get system clock frequency */
    u32HCLKFreq = CLK_GetHCLKFreq();

    if(u32MasterSlave == SPI_MASTER)
    {
        /* Default setting: slave select signal is active low; disable automatic slave select function. */
        spi->SSR = SPI_SS_ACTIVE_LOW;

        if(u32BusClock >= u32HCLKFreq)
        {
            /* Set DIVIDER = 0 */
            spi->DIVIDER &= ~SPI_DIVIDER_DIVIDER_Msk;
            /* Return Master peripheral clock rate */
            return (u32HCLKFreq / 2);
        }
        else if(u32BusClock == 0)
        {
            /* Set DIVIDER to the maximum value 0xFFFF */
            spi->DIVIDER = (spi->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | (0xFFFF << SPI_DIVIDER_DIVIDER_Pos);
            /* Return master peripheral clock rate */
            return (u32HCLKFreq / ((0xFFFF + 1) * 2));
        }
        else
        {
            u32Div = (((u32HCLKFreq * 10) / (u32BusClock * 2) + 5) / 10) - 1; /* Round to the nearest integer */
            if(u32Div > 0xFFFF)
                u32Div = 0xFFFF;
            spi->DIVIDER = (spi->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | (u32Div << SPI_DIVIDER_DIVIDER_Pos);
            /* Return master peripheral clock rate */
            return (u32HCLKFreq / ((u32Div + 1) * 2));
        }
    }
    else     /* For Slave mode, the SPI peripheral clock rate is equal to the system clock rate. */
    {
        /* Default setting: slave selection signal is low level active. */
        spi->SSR = SPI_SSR_SS_LTRIG_Msk;

        /* Set DIVIDER = 0 */
        spi->DIVIDER &= ~SPI_DIVIDER_DIVIDER_Msk;
        /* Return slave peripheral clock rate */
        return u32HCLKFreq;
    }

}

/**
  * @brief  Disable SPI controller.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return None
  * @details This function will reset SPI controller.
  */
void SPI_Close(SPI_T *spi)
{
    if(spi == SPI0)
    {
        /* Reset SPI */
        SYS->IPRSTC2 |= SYS_IPRSTC2_SPI0_RST_Msk;
        SYS->IPRSTC2 &= ~SYS_IPRSTC2_SPI0_RST_Msk;
    }
    else
    {
        /* Reset SPI */
        SYS->IPRSTC2 |= SYS_IPRSTC2_SPI1_RST_Msk;
        SYS->IPRSTC2 &= ~SYS_IPRSTC2_SPI1_RST_Msk;
    }
}

/**
  * @brief  Disable the automatic slave selection function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return None
  * @details This function will disable the automatic slave selection function and set slave selection signal to inactive state.
  */
void SPI_DisableAutoSS(SPI_T *spi)
{
    spi->SSR &= ~(SPI_SSR_AUTOSS_Msk | SPI_SSR_SSR_Msk);
}

/**
  * @brief  Enable the automatic slave selection function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32SSPinMask Specifies slave selection pins. (SPI_SS0, SPI_SS1)
  * @param[in]  u32ActiveLevel Specifies the active level of slave selection signal. (SPI_SS_ACTIVE_HIGH, SPI_SS_ACTIVE_LOW)
  * @return None
  * @details This function will enable the automatic slave selection function. Only available in Master mode.
  *          The slave selection pin and the active level will be set in this function.
  */
void SPI_EnableAutoSS(SPI_T *spi, uint32_t u32SSPinMask, uint32_t u32ActiveLevel)
{
    spi->SSR = (spi->SSR & (~(SPI_SSR_AUTOSS_Msk | SPI_SSR_SS_LVL_Msk | SPI_SSR_SSR_Msk))) | (u32SSPinMask | u32ActiveLevel | SPI_SSR_AUTOSS_Msk);
}

/**
  * @brief  Set the SPI bus clock.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32BusClock The expected frequency of SPI bus clock in Hz.
  * @return Actual frequency of SPI bus clock.
  * @details This function is only available in Master mode. The actual clock rate may be different from the target SPI bus clock rate.
  *          For example, if the SPI source clock rate is 12 MHz and the target SPI bus clock rate is 7 MHz, the actual SPI bus clock
  *          rate will be 6 MHz.
  * @note   If u32BusClock = 0, DIVIDER setting will be set to the maximum value.
  * @note   If u32BusClock >= system clock frequency, the SPI peripheral clock rate will be set to a half of system clock rate.
  *         The DIVIDER (SPI_DIVIDER[15:0]) will be set to 0.
  */
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock)
{
    uint32_t u32HCLKFreq;
    uint32_t u32Div;

    /* Get system clock frequency */
    u32HCLKFreq = CLK_GetHCLKFreq();

    if(u32BusClock >= u32HCLKFreq)
    {
        /* Set DIVIDER = 0 */
        spi->DIVIDER &= ~SPI_DIVIDER_DIVIDER_Msk;
        /* Return Master peripheral clock rate */
        return (u32HCLKFreq / 2);
    }
    else if(u32BusClock == 0)
    {
        /* Set DIVIDER to the maximum value 0xFFFF */
        spi->DIVIDER = (spi->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | (0xFFFF << SPI_DIVIDER_DIVIDER_Pos);
        /* Return Master peripheral clock rate */
        return (u32HCLKFreq / ((0xFFFF + 1) * 2));
    }
    else
    {
        u32Div = (((u32HCLKFreq * 10) / (u32BusClock * 2) + 5) / 10) - 1; /* Round to the nearest integer */
        if(u32Div > 0xFFFF)
            u32Div = 0xFFFF;
        spi->DIVIDER = (spi->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | (u32Div << SPI_DIVIDER_DIVIDER_Pos);
        /* Return Master peripheral clock rate */
        return (u32HCLKFreq / ((u32Div + 1) * 2));
    }
}

/**
  * @brief  Get the actual frequency of SPI bus clock. Only available in Master mode.
  * @param[in]  spi The pointer of the specified SPI module.
  * @return Actual SPI bus clock frequency.
  * @details This function will calculate the actual SPI bus clock rate. Only available in Master mode.
  */
uint32_t SPI_GetBusClock(SPI_T *spi)
{
    uint32_t u32Div;
    uint32_t u32ClkSrc;

    /* Get DIVIDER setting */
    u32Div = (spi->DIVIDER & SPI_DIVIDER_DIVIDER_Msk) >> SPI_DIVIDER_DIVIDER_Pos;
    /* Get the clock source of SPI */
    u32ClkSrc = CLK_GetHCLKFreq();
    /* Return SPI bus clock rate */
    return (u32ClkSrc / ((u32Div + 1) * 2));
}

/**
  * @brief  Enable interrupt function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt bit.
  *                     This parameter decides which interrupt will be enabled.
  *                     Only SPI_UNIT_INT_MASK is available on NUC122.
  * @return None
  * @details Enable SPI related interrupts specified by u32Mask parameter.
  */
void SPI_EnableInt(SPI_T *spi, uint32_t u32Mask)
{
    /* Enable unit transfer interrupt flag */
    if((u32Mask & SPI_UNIT_INT_MASK) == SPI_UNIT_INT_MASK)
        spi->CNTRL |= SPI_CNTRL_IE_Msk;
}

/**
  * @brief  Disable interrupt function.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt enable bits.
  *                     Each bit corresponds to a interrupt bit.
  *                     This parameter decides which interrupt will be disabled.
  *                     Only SPI_UNIT_INT_MASK is available on NUC122.
  * @return None
  * @details Disable SPI related interrupts specified by u32Mask parameter.
  */
void SPI_DisableInt(SPI_T *spi, uint32_t u32Mask)
{
    /* Disable unit transfer interrupt flag */
    if((u32Mask & SPI_UNIT_INT_MASK) == SPI_UNIT_INT_MASK)
        spi->CNTRL &= ~SPI_CNTRL_IE_Msk;
}

/**
  * @brief  Get interrupt flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flag will be read.
  *                     Only SPI_UNIT_INT_MASK is available on NUC122.
  * @return Interrupt flags of selected sources.
  * @details Get SPI related interrupt flags specified by u32Mask parameter.
  */
uint32_t SPI_GetIntFlag(SPI_T *spi, uint32_t u32Mask)
{
    uint32_t u32IntFlag = 0;

    /* Check unit transfer interrupt flag */
    if((u32Mask & SPI_UNIT_INT_MASK) && (spi->CNTRL & SPI_CNTRL_IF_Msk))
        u32IntFlag |= SPI_UNIT_INT_MASK;

    return u32IntFlag;
}

/**
  * @brief  Clear interrupt flag.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related interrupt sources.
  *                     Each bit corresponds to a interrupt source.
  *                     This parameter decides which interrupt flag will be cleared.
  *                     Only SPI_UNIT_INT_MASK is available on NUC122.
  * @return None
  * @details Clear SPI related interrupt flags specified by u32Mask parameter.
  */
void SPI_ClearIntFlag(SPI_T *spi, uint32_t u32Mask)
{
    if(u32Mask & SPI_UNIT_INT_MASK)
        spi->CNTRL |= SPI_CNTRL_IF_Msk; /* Clear unit transfer interrupt flag */
}

/**
  * @brief  Get SPI status.
  * @param[in]  spi The pointer of the specified SPI module.
  * @param[in]  u32Mask The combination of all related sources.
  *                     Each bit corresponds to a source.
  *                     This parameter decides which flag will be read.
  *                     Only SPI_BUSY_MASK is available on NUC122.
  * @return Flags of selected sources.
  * @details Get SPI related status specified by u32Mask parameter.
  */
uint32_t SPI_GetStatus(SPI_T *spi, uint32_t u32Mask)
{
    uint32_t u32Flag = 0;

    /* Check busy status */
    if((u32Mask & SPI_BUSY_MASK) && (spi->CNTRL & SPI_CNTRL_GO_BUSY_Msk))
        u32Flag |= SPI_BUSY_MASK;

    return u32Flag;
}

/*@}*/ /* end of group SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group SPI_Driver */

/*@}*/ /* end of group Device_Driver */

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/
