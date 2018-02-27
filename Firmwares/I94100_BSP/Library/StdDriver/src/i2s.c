/****************************************************************************
 * @file     i2s.c
 * @version  V3.00
 * $Revision: 11 $
 * $Date: 17/08/24 04:26p $
 * @brief    I94100 series I2S driver source file
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I94100_Device_Driver I94100 Device Driver
  @{
*/

/** @addtogroup I94100_I2S_Driver I2S Driver
  @{
*/

/** @addtogroup I94100_I2S_EXPORTED_FUNCTIONS I2S Exported Functions
  @{
*/

/**
  * @brief  This function is used to get I2S source clock frequency.
  * @param[in]  i2s The pointer of the specified I2S module.
  * @return I2S source clock frequency (Hz).
  * @details Return the source clock frequency according to the setting of SPI1SEL (CLKSEL2[5:4]) or SPI2SEL (CLKSEL2[7:6]).
  */
static uint32_t I2S_GetSourceClockFreq(I2S_T *i2s)
{
    uint32_t u32Freq, u32HCLKFreq, u32temp;

	if((CLK->CLKSEL3 & CLK_CLKSEL3_I2S0SEL_Msk) == CLK_CLKSEL3_I2S0SEL_HXT)
		u32Freq = __HXT; /* Clock source is HXT */
	else if((CLK->CLKSEL3 & CLK_CLKSEL3_I2S0SEL_Msk) == CLK_CLKSEL3_I2S0SEL_PLL)
		u32Freq = CLK_GetPLLClockFreq(); /* Clock source is PLL */
	else if((CLK->CLKSEL3 & CLK_CLKSEL3_I2S0SEL_Msk) == CLK_CLKSEL3_I2S0SEL_PCLK0)
	{
		/* Get system clock frequency */
		u32HCLKFreq = CLK_GetHCLKFreq();
		/* Clock source is PCLK0 */
		if((u32temp = (CLK->PCLKDIV & CLK_PCLKDIV_APB0DIV_Msk)) == CLK_PCLKDIV_PCLK0DIV2)
			u32Freq = (u32HCLKFreq / 2);
		else if(u32temp == CLK_PCLKDIV_PCLK0DIV4)
			u32Freq = (u32HCLKFreq / 4);
		else if(u32temp == CLK_PCLKDIV_PCLK0DIV8)
			u32Freq = (u32HCLKFreq / 8);
		else if(u32temp == CLK_PCLKDIV_PCLK0DIV16)
			u32Freq = (u32HCLKFreq / 16);
		else
			u32Freq = u32HCLKFreq;
	}
	else
		u32Freq = __HIRC; /* Clock source is HIRC */

    return u32Freq;
}

/**
  * @brief  	This function configures some parameters of I2S interface for general purpose use.
  * @param[in] 	u32MasterSlave I2S operation mode. Valid values are listed below.
  *            	- \ref I2S_MASTER
  *				- \ref I2S_SLAVE
  * @param[in] 	u32SampleRate Sample rate
  * @param[in] 	u32WordWidth Data length. Valid values are listed below.
  * 			- \ref I2S_DATABIT_8
  * 			- \ref I2S_DATABIT_16
  *				- \ref I2S_DATABIT_24
  *				- \ref I2S_DATABIT_32
  * @param[in] 	u32Stereo Audio format. Valid values are listed below.
  * 			- \ref I2S_TDMCHNUM_2CH
  *				- \ref I2S_TDMCHNUM_4CH
  * 			- \ref I2S_TDMCHNUM_6CH
  *				- \ref I2S_TDMCHNUM_8CH
  * @param[in] 	u32Mono Audio format. Valid values are listed below.
  * 			- \ref I2S_MONO
  *				- \ref I2S_STEREO
  * @param[in] 	u32DataFormat Data format. Valid values are listed below.
  *				- \ref I2S_FORMAT_I2S
  *				- \ref I2S_FORMAT_MSB
  *				- \ref I2S_FORMAT_LSB
  *				- \ref I2S_FORMAT_PCM
  *				- \ref I2S_FORMAT_PCMMSB
  *				- \ref I2S_FORMAT_PCMLSB
  * @return 	Real sample rate of master mode or peripheral clock rate of slave mode.
  * @details 	This function will reset SPI/I2S controller and configure I2S controller according to the input parameters.
  *          	Set TX and RX FIFO threshold to middle value. Both the TX and RX functions will be enabled.
  *          	The actual sample rate may be different from the target sample rate. The real sample rate will be returned for reference.
  * @note   	Only SPI1 and SPI2 support I2S mode.
  * @note   	In slave mode, the SPI peripheral clock rate will be equal to APB clock rate.
  */
uint32_t I2S_Open(I2S_T *i2s, 
				  uint32_t u32MasterSlave, 
				  uint32_t u32SampleRate, 
				  uint32_t u32WordWidth, 
				  uint32_t u32Channels,
				  uint32_t u32Mono,
				  uint32_t u32DataFormat)
{
    uint32_t u32Divider;
    uint32_t u32BitRate, u32SrcClk;

    /* Reset SPI/I2S */
	if(i2s == I2S0)
		SYS_ResetModule(I2S0_RST);
	else
		return 0;

    /* Configure I2S controller */
    i2s->CTL0 = u32MasterSlave | u32WordWidth | u32Mono | u32DataFormat;
	I2S_SET_TDMCHNUM(I2S0, u32Channels);
	/* Check Mono, Format, Channel, Datawidth setting*/
	if(u32Mono == I2S_MONO)
		u32Channels = 1;
	else
	{
		if(u32DataFormat == I2S_FORMAT_PCM || u32DataFormat == I2S_FORMAT_PCMMSB || u32DataFormat == I2S_FORMAT_PCMLSB)
			u32Channels = ((u32Channels >> I2S_CTL0_TDMCHNUM_Pos) +1) * 2;
		else
			u32Channels = 2;

		if(u32WordWidth >= I2S_DATABIT_24)
			u32WordWidth = 4;
		else
			u32WordWidth = (u32WordWidth >> I2S_CTL0_DATWIDTH_Pos) + 1;
		
		if(u32WordWidth == 1 && u32Channels > 2)
			u32WordWidth = 2;
	}

    if(u32MasterSlave == I2S_MASTER)
    {
		uint32_t u32Error, u32Error2;
		
        /* Get the source clock rate */
        u32SrcClk = I2S_GetSourceClockFreq(i2s);
		
        u32BitRate = u32SampleRate * u32WordWidth * 8 * u32Channels;
        u32Divider = ((u32SrcClk / u32BitRate) >> 1) - 1;
		/* Adjust Error */
		u32Error = (u32SrcClk/(2*(u32Divider+1)) - u32BitRate);
		u32Error2 = (u32BitRate - u32SrcClk/(2*(u32Divider+2)));
		u32Divider = (u32Error < u32Error2)?u32Divider:(u32Divider+1);
		
        /* Set BCLKDIV setting */
        i2s->CLKDIV = (I2S0->CLKDIV & ~I2S_CLKDIV_BCLKDIV_Msk) | (u32Divider << I2S_CLKDIV_BCLKDIV_Pos);

        /* Calculate bit clock rate */
        u32BitRate = u32SrcClk / ((u32Divider + 1) * 2);
        /* Calculate real sample rate */
        u32SampleRate = u32BitRate /(u32WordWidth * 8 * u32Channels);
    }
    else
    {
        /* Set BCLKDIV = 0 */
        i2s->CLKDIV &= ~I2S_CLKDIV_BCLKDIV_Msk;
		/* Get the source clock rate */
        u32SampleRate = I2S_GetSourceClockFreq(i2s);
    }
	
	/* Return the real sample rate */
    return u32SampleRate;
}

/**
  * @brief  	Disable I2S function.
  * @param[in]  i2s The pointer of the specified I2S module.
  * @return 	None
  * @details 	Disable I2S function.
  */
void I2S_Close(I2S_T *i2s)
{
    i2s->CTL0 &= ~I2S_CTL0_I2SEN_Msk;
}

/**
  * @brief  	Enable master clock (MCLK).
  * @param[in] 	i2s The pointer of the specified I2S module.
  * @param[in] 	u32BusClock The target MCLK clock rate.
  * @return 	Actual MCLK clock rate
  * @details 	Set the master clock rate according to u32BusClock parameter and enable master clock output.
  *          	The actual master clock rate may be different from the target master clock rate. The real master clock rate will be returned for reference.
  */
uint32_t I2S_EnableMCLK(I2S_T *i2s, uint32_t u32BusClock)
{
    uint32_t u32Divider, u32Error, u32Error2;
    uint32_t u32SrcClk;

    u32SrcClk = I2S_GetSourceClockFreq(i2s);
    if(u32BusClock == u32SrcClk)
        u32Divider = 0;
    else
    {
        u32Divider = (u32SrcClk / u32BusClock) >> 1;
        /* MCLKDIV is a 6-bit width configuration. The maximum value is 0x3F. */
        if(u32Divider > 0x7F)
            u32Divider = 0x7F;
		
		/* Adjust Error */
		u32Error = u32SrcClk/(2*u32Divider) - u32BusClock;
		u32Error2 = u32BusClock - u32SrcClk/(2*(u32Divider+1));
		u32Divider = (u32Error < u32Error2)?u32Divider:(u32Divider+1);
    }

    /* Write u32Divider to MCLKDIV (SPI_I2SCLK[5:0]) */
    i2s->CLKDIV = (i2s->CLKDIV & ~I2S_CLKDIV_MCLKDIV_Msk) | (u32Divider << I2S_CLKDIV_MCLKDIV_Pos);

    /* Enable MCLK output */
    i2s->CTL0 |= I2S_CTL0_MCLKEN_Msk;

    if(u32Divider == 0)
        return u32SrcClk; /* If MCLKDIV=0, master clock rate is equal to the source clock rate. */
    else
        return ((u32SrcClk >> 1) / u32Divider); /* If MCLKDIV>0, master clock rate = source clock rate / (MCLKDIV * 2) */
}

void I2S_DisableMCLK(I2S_T *i2s)
{
	i2s->CTL0 &= ~I2S_CTL0_MCLKEN_Msk;
}
/*@}*/ /* end of group I94100_I2S_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I94100_I2S_Driver */

/*@}*/ /* end of group I94100_Device_Driver */

#ifdef __cplusplus
}
#endif

/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
