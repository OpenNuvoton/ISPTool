/****************************************************************************//**
 * @file     spi1.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 17/05/12 10:00a $
 * @brief    I91200 SPI1 driver source file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_SPI1_Driver SPI1 Driver
  @{
*/

/** @addtogroup I91200_SPI1_EXPORTED_FUNCTIONS SPI1 Exported Functions
  @{
*/

/**
  * @brief  This function make SPI module be ready to transfer.
  *         By default, the SPI transfer sequence is MSB first and
  *         the automatic slave select function is disabled. In
  *         Slave mode, the u32BusClock must be NULL and the SPI clock
  *         divider setting will be 0.
  * @param  spi is the base address of SPI1 module.
  * @param  u32MasterSlave decides the SPI module is operating in master mode or in slave mode. Valid values are:
  *                    - \ref SPI1_SLAVE
  *                    - \ref SPI1_MASTER
  * @param  u32SPIMode decides the transfer timing. Valid values are:
  *                    - \ref SPI1_MODE_0
  *                    - \ref SPI1_MODE_1
  *                    - \ref SPI1_MODE_2
  *                    - \ref SPI1_MODE_3
  * @param  u32BusClock is the expected frequency of SPI bus clock in Hz.
  * @param  u32VarClock is the variable clock 2. 
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI1_Open(SPI1_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32BusClock, uint32_t u32VarClock)
{
	/* Set configuration of master/slave mode, trigger mode */
	spi->CTL = u32MasterSlave | u32SPIMode;
	
	/* Set SPI variable clock */
	SPI1_SetVarClock(spi,u32VarClock);	
	
	/* Set SPI bus clock */
	SPI1_SetBusClock(spi,u32BusClock);
	
	return SPI1_GetBusClock(spi);
}

/**
  * @brief Reset SPI module and disable SPI peripheral clock.
  * @param  spi is the base address of SPI1 module.
  * @return none
  */
void SPI1_Close(SPI1_T *spi)
{
  
}

/**
  * @brief Set the SPI bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @param  u32BusClock is the expected frequency of SPI bus clock.
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI1_SetBusClock(SPI1_T *spi, uint32_t u32BusClock)
{
    uint32_t u32Pclk = CLK_GetHCLKFreq();
    uint32_t u32Div = 0xffff;

    if(u32BusClock !=0 ) 
	{
        u32Div = ((((2*u32Pclk)/u32BusClock)+1)>>2) - 1;
        if(u32Div > 0xffff)
            u32Div = 0xffff;
		else if(u32Div < 1)
			u32Div = 1;
        spi->CLKDIV = (spi->CLKDIV & ~SPI1_CLKDIV_CLKDIV0_Msk) | u32Div;
    } 
	else
        spi->CLKDIV = 0;

    return ( u32Pclk / ((u32Div+1)*2) );
}

/**
  * @brief Set the SPI variable clock.
  * @param  spi is the base address of SPI module.
  * @param  u32VarClock is clock rate of variable clock 2.
  * @return None.
  */
void SPI1_SetVarClock(SPI1_T *spi, uint32_t u32VarClock)
{
    uint32_t u32Pclk = CLK_GetHCLKFreq();
    uint32_t u32Div = 0xffff;

    if(u32VarClock !=0 ) 
	{
        u32Div = ((( u32Pclk / u32VarClock ) + 1) >> 1 ) - 1;
        if(u32Div > 0xffff)
            u32Div = 0xffff;
		else if(u32Div < 1)
			u32Div = 1;
        spi->CLKDIV = (spi->CLKDIV & ~SPI1_CLKDIV_CLKDIV1_Msk) | (u32Div<<SPI1_CLKDIV_CLKDIV1_Pos);
    } else
        spi->CLKDIV = 0;
}

/**
  * @brief Get the actual frequency of SPI1 bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI1 module.
  * @return Actual SPI bus clock frequency.
  */
uint32_t SPI1_GetBusClock(SPI1_T *spi)
{
    return ((CLK_GetHCLKFreq()>>1) / ((spi->CLKDIV&SPI1_CLKDIV_CLKDIV0_Msk) + 1));
}

/**
  * @brief Get the actual frequency of SPI bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI1 module.
  * @return Actual SPI variable clock frequency.
  */
uint32_t SPI1_GetVarClock(SPI1_T *spi)
{
	return ((CLK_GetHCLKFreq()>>1) / (((spi->CLKDIV&SPI1_CLKDIV_CLKDIV1_Msk)>>SPI1_CLKDIV_CLKDIV1_Pos)+1));	
}

/*@}*/ /* end of group I91200_SPI1_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_SPI1_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
