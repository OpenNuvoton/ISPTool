/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/


#include "ML51.h"

/**
  * @brief      Enable specify I2C controller and set divider
  * @param[in]  u8I2CSel:  Specify I2C port
	* @param[in]  u32SYSCLK: Define Fsys clock value in Hz.
  * @param[in]  u32I2CCLK: The target I2C bus clock in Hz. Use HIRC the I2C clock is from 23473 ~ 2000000
  * @return     None
  * @details    The function enable the specify I2C controller and set proper clock divider
  *             in I2C CLOCK DIVIDED REGISTER (I2CLK) according to the target I2C Bus clock.
  *             I2C bus clock = PCLK / (4*(u32I2CCLK+1).
	* @exmaple :  I2C0_Open(I2C0,24000000,100000);
  */
void I2C_Open(unsigned char u8I2CSel, unsigned long u32SYSCLK, unsigned long u32I2CCLK)
{
		SFRS = 0x00;
		switch (u8I2CSel)
		{
			case I2C0: I2C0CLK = (u32SYSCLK/4/u32I2CCLK-1); set_I2C0CON_I2CEN; break;
			case I2C1: I2C1CLK = (u32SYSCLK/4/u32I2CCLK-1); set_I2C1CON_I2CEN; break;
    }                           
}

/**
  * @brief      Disable I2C function
  * @param[in]  u8I2CSel:  Specify I2C port
	* @exmaple :  I2C0_Close(I2C0);
*/
void I2C_Close(unsigned char u8I2CSel)
{
		SFRS = 0;
		switch (u8I2CSel)
		{
			case I2C0: clr_I2C0CON_I2CEN; break;
			case I2C1: clr_I2C1CON_I2CEN; break;
    }         
}

/**
  * @brief      Enable specify I2C controller interrupt, also need enable globle interrupt in main loop.
  * @param[in]  u8I2CSel:  Specify I2C port
	* @exmaple :  I2C_EnableInt(I2C0);
*/
void I2C_EnableInt(unsigned char u8I2CSel)
{
    SFRS = 0;
		switch (u8I2CSel)
		{
			case I2C0: set_EIE0_EI2C0; break;
			case I2C1: set_EIE1_EI2C1; break;
    }  
}

/**
  * @brief      Disable specify I2C controller interrupt
  * @param[in]  u8I2CSel:  Specify I2C port
	* @exmaple :  I2C_DisableInt(I2C0);
*/
void I2C_DisableInt(unsigned char u8I2CSel)
{
    SFRS = 0;
		switch (u8I2CSel)
		{
			case I2C0: clr_EIE0_EI2C0; break;
			case I2C1: clr_EIE1_EI2C1; break;
    }  
}

/**
  * @brief      Get I2C bus status value
  * @param[in]  u8I2CSel:  Specify I2C port
  * @return     I2C status data 
	* @exmaple :  I2C_GetStatus(I2C0);
*/
unsigned char I2C_GetStatus(unsigned char u8I2CSel)
{
		unsigned char u8i2cstat;
		SFRS = 0;
		switch (u8I2CSel)
		{
			case I2C0: u8i2cstat=I2C0STAT; break;
			case I2C1: u8i2cstat=I2C1STAT; break;
		}
    return (u8i2cstat);
}

/**
 * @brief      Configure the mask bits of 7-bit Slave Address
 * @param[in]  u8I2CSel:  Specify I2C port
 * @param[in]  u8SlaveNo        Set the number of I2C address mask register (0~3)
 * @param[in]  u8SlaveAddrMask  A byte for slave address mask
 * @return     None
 * @details    This function is used to set 7-bit slave addresses.
 * @example    I2C_SetSlaveAddrMask(I2C0,0,0x33);
 *
 */
void I2C_SetSlaveAddrMask(unsigned char u8I2CSel, unsigned char u8SlaveNo, unsigned char u8SlaveAddrMask)
{
		switch (u8I2CSel)
		{
			case I2C0: 
				switch (u8SlaveNo)
				{
					case 0: SFRS=0;I2C0ADDR0=u8SlaveAddrMask; break;
					case 1: SFRS=2;I2C0ADDR1=u8SlaveAddrMask; break;
					case 2: SFRS=2;I2C0ADDR2=u8SlaveAddrMask; break;
					case 3: SFRS=2;I2C0ADDR3=u8SlaveAddrMask; break;
				}
			case I2C1:
				switch (u8SlaveNo)
				{
					case 0: SFRS=0;I2C1ADDR0=u8SlaveAddrMask; break;
					case 1: SFRS=2;I2C1ADDR1=u8SlaveAddrMask; break;
					case 2: SFRS=2;I2C1ADDR2=u8SlaveAddrMask; break;
					case 3: SFRS=2;I2C1ADDR3=u8SlaveAddrMask; break;
				}			
			break;
			}
}

/**
 * @brief      Enable Time-out Function with support long time-out
 * @param[in]  u8I2CSel:  Specify I2C port
 * @return     None
 * @details    This function enable time-out function and configure DIV4 to support long
 *             time-out.
 * @example    I2C_EnableTimeout(I2C0);
 */
void I2C_EnableTimeout(unsigned char u8I2CSel)
{
		SFRS = 0;
		switch (u8I2CSel)
		{
			case I2C0: I2C0TOC|=0x06; break;
			case I2C1: I2C1TOC|=0x06; break;
		}
}

/**
 * @brief      Disable Time-out Function
 * @param[in]  u8I2CSel:  Specify I2C port
 * @return     None
 * @details    This function disable time-out function.
 *
 */
void I2C_DisableTimeout(unsigned char u8I2CSel)
{
		SFRS = 0;
		switch (u8I2CSel)
		{
			case I2C0: I2C0TOC&=0xFB; break;
			case I2C1: I2C1TOC&=0xFB; break;
		}
}

void I2C_ClearTimeoutFlag(unsigned char u8I2CSel)
{
		SFRS = 0;
		switch (u8I2CSel)
		{
			case I2C0: I2C0TOC&=0xFE; break;
			case I2C1: I2C1TOC&=0xFE; break;
		}
}
