/****************************************************************************//**
 * @file     spi.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/07/25 10:00a $
 * @brief    ISD9100 SPI driver source file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "ISD9100.h"

/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_SPI_Driver SPI Driver
  @{
*/


/** @addtogroup ISD9100_SPI_EXPORTED_FUNCTIONS SPI Exported Functions
  @{
*/

/**
  * @brief  This function make SPI module be ready to transfer.
  *         By default, the SPI transfer sequence is MSB first and
  *         the automatic slave select function is disabled. In
  *         Slave mode, the u32BusClock must be NULL and the SPI clock
  *         divider setting will be 0.
  * @param  spi is the base address of SPI module.
  * @param  u32MasterSlave decides the SPI module is operating in master mode or in slave mode. Valid values are:
  *                    - \ref SPI_SLAVE
  *                    - \ref SPI_MASTER
  * @param  u32SPIMode decides the transfer timing. Valid values are:
  *                    - \ref SPI_MODE_0
  *                    - \ref SPI_MODE_1
  *                    - \ref SPI_MODE_2
  *                    - \ref SPI_MODE_3
  * @param  u32BusClock is the expected frequency of SPI bus clock in Hz.
  * @param  u32VarClock is the variable clock 2. 
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI_Open(SPI_T *spi, uint32_t u32MasterSlave, uint32_t u32SPIMode, uint32_t u32BusClock, uint32_t u32VarClock)
{
	/* Set configuration of master/slave mode, trigger mode */
	spi->CTL = u32MasterSlave | u32SPIMode;
	
	/* Set SPI variable clock */
	SPI_SetVarClock(spi,u32VarClock);	
	
	/* Set SPI bus clock */
	SPI_SetBusClock(spi,u32BusClock);
	
	return SPI_GetBusClock(spi);
}

/**
  * @brief Reset SPI module and disable SPI peripheral clock.
  * @param  spi is the base address of SPI module.
  * @return none
  */
void SPI_Close(SPI_T *spi)
{
    /* Reset SPI */
    if((uint32_t)spi == SPI0_BASE) 
	{
        SYS->IPRST1 |= SYS_IPRST1_SPI0RST_Msk;
        SYS->IPRST1 &= ~SYS_IPRST1_SPI0RST_Msk;
    }
}

/**
  * @brief Set the SPI bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @param  u32BusClock is the expected frequency of SPI bus clock.
  * @return Actual frequency of SPI peripheral clock.
  */
uint32_t SPI_SetBusClock(SPI_T *spi, uint32_t u32BusClock)
{
    uint32_t u32Pclk = CLK_GetHCLKFreq();
    uint32_t u32Div = 0xffff;

    if(u32BusClock !=0 ) 
	{
        u32Div = ((((2*u32Pclk)/u32BusClock)+1)>>2) - 1;
        if(u32Div > 0xFFFF)
            u32Div = 0xFFFF;
		else if(u32Div < 1)
			u32Div = 1;
        spi->CLKDIV = (spi->CLKDIV & ~SPI_CLKDIV_DIVIDER0_Msk) | u32Div;
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
void SPI_SetVarClock(SPI_T *spi, uint32_t u32VarClock)
{
    uint32_t u32Pclk = CLK_GetHCLKFreq();
    uint32_t u32Div = 0xffff;

    if(u32VarClock !=0 ) 
	{
        u32Div = ((( u32Pclk / u32VarClock ) + 1) >> 1 ) - 1;
        if(u32Div > 0xFFFF)
            u32Div = 0xFFFF;
		else if(u32Div < 1)
			u32Div = 1;
        spi->CLKDIV = (spi->CLKDIV & ~SPI_CLKDIV_DIVIDER1_Msk) | (u32Div<<SPI_CLKDIV_DIVIDER1_Pos);
    } else
        spi->CLKDIV = 0;
}

/**
  * @brief Get the actual frequency of SPI bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @return Actual SPI bus clock frequency.
  */
uint32_t SPI_GetBusClock(SPI_T *spi)
{
    uint32_t u32Div;
    uint32_t u32ApbClock;

    u32ApbClock = CLK_GetHCLKFreq();
    u32Div = spi->CLKDIV & SPI_CLKDIV_DIVIDER0_Msk;
    return ((u32ApbClock>>1) / (u32Div + 1));
}

/**
  * @brief Get the actual frequency of SPI bus clock. Only available in Master mode.
  * @param  spi is the base address of SPI module.
  * @return Actual SPI variable clock frequency.
  */
uint32_t SPI_GetVarClock(SPI_T *spi)
{
	uint32_t u32Div;
	uint32_t u32ApbClock;

	u32ApbClock = CLK_GetHCLKFreq();
	u32Div = (spi->CLKDIV&SPI_CLKDIV_DIVIDER1_Msk)>>SPI_CLKDIV_DIVIDER1_Pos;
	return ((u32ApbClock>>1) / (u32Div+1));	
}

/*@}*/ /* end of group ISD9100_SPI_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_SPI_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
