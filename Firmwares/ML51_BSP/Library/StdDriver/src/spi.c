/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2018 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/


#include "ML51.h"

/**
  * @brief      Enable specify SPI controller and set divider
  * @param[in]  u8SPISel:  Specify SPI port (SPI0,SPI1)
  * @param[in]  u32MasterSlave decides the SPI module is operating in master mode or in slave mode. (SPI_SLAVE, SPI_MASTER)
  * @param[in]  u32SPICLKDIV: The target SPI bus clock with HIRC divider value is (0~15)
											- \ref Value  SPR[3:0] Div	SPI clock rate(HIRC)
											- \ref   0      0000    2	   12M bit/s
											- \ref   1      0001    4	   6M bit/s
											- \ref   2      0010    8    3M bit/s
											- \ref   3      0011    16   1.5M bit/s
											- \ref   4      0100    32   750k bit/s
											- \ref   5      0101    64   375k bit/s
											- \ref   6      0110    128	 187k bit/s
											- \ref   7      0111    256  93.7k bit/s
											- \ref   8      1000    3	   8M bit/s
											- \ref   9      1001    6	   4M bit/s
											- \ref   10     1010    12	 2M bit/s
											- \ref   11     1011    24	 1M bit/s
											- \ref   12     1100    48   500k bit/s
											- \ref   13     1101    96   250k bit/s
											- \ref   14     1110    192	 125k bit/s
											- \ref   15     1111    384  62.5k bit/s
  * @param[in]  u8SPIMode: (SPI_MODE_0/SPI_MODE_1/SPI_MODE_2/SPI_MODE_3)
	* @param[in]  u8MSBLSB:SPI data is transferred MSB first or LSB first (MSB_FIRST,LSB_FIRST)
  * @return     None
  * @details    The function enable the specify SPI controller and set proper clock divider
  *             in I2C CLOCK DIVIDED REGISTER (SPInCR0 and SPInCR1) according to the target SPI Bus clock.
  *             detail see u32SPICLKDIV table
	* @exmaple :  SPI_Open(SPI0,SPI_MASTER,10,SPI_MODE_3,LSB_FIRST);
  */
	
void SPI_Open(  unsigned char u8SPISel, 
								unsigned char u8MasterSlave,
								unsigned char u8SPICLKDIV,
								unsigned char u8SPIMode,
								unsigned char u8MSBLSB)
{
	switch (u8SPISel)
	{
			case SPI0:
				set_SPI0SR_DISMODF;				/*Mode fault error detection disaable*/
				clr_SPI0CR0_SSOE;         /*SS pin use as GPIO*/
				SFRS = 0;
				SPI0CR0 = 0;
				SPI0CR0|= (u8MasterSlave<<4) | (u8SPICLKDIV&0x03)| (u8SPIMode<<2) | (u8MSBLSB<<5);
				SFRS = 1 ;
				SPI0CR1 = 0 ; 
				SPI0CR1 |= (u8SPICLKDIV&0x0C)<<2;
				set_SPI0CR0_SPIEN;
			break;
			case SPI1:
				set_SPI1SR_DISMODF;				/*Mode fault error detection disaable*/
				clr_SPI1CR0_SSOE;         /*SS pin use as GPIO*/
				SFRS = 0;
				SPI1CR0 = 0;
				SPI1CR0|= (u8MasterSlave<<4) | (u8SPICLKDIV&0x03)| (u8SPIMode<<2) | (u8MSBLSB<<5);;
				SPI1CR1 = 0 ; 
				SPI1CR1|= (u8SPICLKDIV&0x0C)<<2;
				set_SPI1CR0_SPIEN;
			break;
	}
}

/**
  * @brief      Enable  spi controller interrupt
  * @param[in]  u8SPISel:  Specify SPI port (SPI0,SPI1)
  * @param[in]  u8SPIINTStatus: decides the SPI interrupt function. (Disable, Enable)
  * @return     None
  * @details    None
	* @exmaple :  SPI_Open(SPI0,Enable);
  */
void SPI_Interrupt(unsigned char u8SPISel, unsigned char u8SPIINTStatus)
{
		switch (u8SPISel)
		{
				case SPI0: EIE0 |= u8SPIINTStatus<<6; break;
				case SPI1: EIE1 |= u8SPIINTStatus<<6; break;
		}
}