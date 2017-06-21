/**************************************************************************//**
 * @file     NuEdu-Basic01_SPI_Flash.c
 * @version  V1.00
 * $Revision: 3 $
 * $Date: 14/09/16 9:27a $
 * @brief    NuEdu-Basic01_SPI_Flash driver source file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_SPI_Flash.h"

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS SPI Flash Functions
  @{
*/

/// @cond HIDDEN_SYMBOLS
/*---------------------------------------------------------------------------------------------------------*/
/*  Definitons                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define SPI_FLASH_PORT SPI0
/// @endcond

/**
 * @brief       Open GPIO port for SPI interface and configure this SPI controller as Master,
 *              MSB first, clock idle low, TX at falling-edge, RX at rising-edge, 32-bit length
 *      transaction, disable the automatic hardware slave select function and SPI serial
 *      clock rate = 2 MHz.
 *
 * @return      None
 */
void Open_SPI_Flash(void)
{

    /* Init GPIO for SPI Flash Port, set PE1, PE2, PE3 and PE4 for SPI0 */
    SYS->PE_L_MFP |= (SYS_PE_L_MFP_PE1_MFP_SPI0_SS0 | SYS_PE_L_MFP_PE2_MFP_SPI0_SCLK |
                      SYS_PE_L_MFP_PE3_MFP_SPI0_MISO0 | SYS_PE_L_MFP_PE4_MFP_SPI0_MOSI0);

    /* Enable SPI0 IP clock */
    CLK->APBCLK |= CLK_APBCLK_SPI0_EN_Msk;

    /* Configure SPI_FLASH_PORT as a master, MSB first, clock idle low, TX at falling-edge, RX at rising-edge and 32-bit transaction */
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~(SPI_CTL_SLAVE_Msk | SPI_CTL_LSB_Msk | SPI_CTL_CLKP_Msk | SPI_CTL_TX_BIT_LEN_Msk | SPI_CTL_TX_NEG_Msk | SPI_CTL_RX_NEG_Msk)) | SPI_CTL_TX_NEG_Msk;

    /* Disable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk);

    /* Set SPI clock rate = HCLK / (20+1) = 42MHz / 21 = 2MHz */
    SPI_FLASH_PORT->CLKDIV = (SPI_FLASH_PORT->CLKDIV & ~SPI_CLKDIV_DIVIDER1_Msk) | (0x14 << SPI_CLKDIV_DIVIDER1_Pos);

}

/**
 * @brief       Read back the Manufacturer ID and Device ID from SPI Flash device.
 *
 * @return      Manufacturer ID and Device ID of SPI Flash device.
 *          For W25Q16BV, Manufacturer ID: 0xEF; Device ID: 0x14
 *      For W26X16, Manufacturer ID: 0xEF; Device ID: 0x14
*/
unsigned int SpiFlash_ReadMidDid(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // configure transaction length as 8 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x08 << SPI_CTL_TX_BIT_LEN_Pos);

    // /CS: active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk) | 0x1;

    // send Command: 0x90, Read Manufacturer/Device ID
    au32SourceData = 0x90;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x18 << SPI_CTL_TX_BIT_LEN_Pos);

    // send 24-bit '0', dummy
    au32SourceData = 0x0;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // configure transaction length as 16 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x10 << SPI_CTL_TX_BIT_LEN_Pos);

    // receive
    au32SourceData = 0x0;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // /CS: de-active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk);

    // dump Rx register
    au32DestinationData = SPI_FLASH_PORT->RX0;

    return (au32DestinationData & 0xffff);

}

/**
 * @brief       This function do the chip erasing to SPI Flash device.
 *
 * @return      None
 */
void SpiFlash_ChipErase(void)
{
    unsigned int au32SourceData;

    // configure transaction length as 8 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x08 << SPI_CTL_TX_BIT_LEN_Pos);

    // /CS: active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk) | 0x1;

    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // /CS: de-active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk);

    // /CS: active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk) | 0x1;

    // send Command: 0xC7, Chip Erase
    au32SourceData = 0xc7;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // /CS: de-active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk);

}

/**
 * @brief       Read back the Status Register 1 from SPI Flash device.
 *
 * @return      Status Register 1 value of SPI Flash device.
 */
unsigned int SpiFlash_ReadStatusReg1(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // configure transaction length as 16 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x10 << SPI_CTL_TX_BIT_LEN_Pos);

    // /CS: active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk) | 0x1;

    // send Command: 0x05, Read status register 1
    au32SourceData = 0x0500;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // /CS: de-active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk);

    // dump Rx register
    au32DestinationData = SPI_FLASH_PORT->RX0;

    return (au32DestinationData & 0xFF);

}

/**
 * @brief       Read back the Status Register 2 from SPI Flash device.
 *
 * @return      Status Register 2 value of SPI Flash device.
 */
unsigned int SpiFlash_ReadStatusReg2(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // configure transaction length as 16 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x10 << SPI_CTL_TX_BIT_LEN_Pos);

    // /CS: active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk) | 0x1;

    // send Command: 0x35, Read status register 2
    au32SourceData = 0x3500;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // /CS: de-active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk);

    // dump Rx register
    au32DestinationData = SPI_FLASH_PORT->RX0;

    return (au32DestinationData & 0xFF);

}

/**
 * @brief       Waiting for the BUSY bit of SPI Flash that be cleared to 0.
 *
 * @return      None
 */
void SpiFlash_WaitReady(void)
{
    unsigned int ReturnValue;

    do {
        ReturnValue = SpiFlash_ReadStatusReg1();
        ReturnValue = ReturnValue & 1;
    } while(ReturnValue!=0); // check the BUSY bit

}

/**
 * @brief       This function do the page programming to SPI Flash device.
 *
 * @param[in]   *DataBuffer A Point that point to source data buffer.
 *
 * @param[in]   StartAddress    A start address of SPI Flash that will be programmed.
 *
 * @param[in]   ByteCount   Byte count number that will be programmed.
 *
 * @return      None
 */
void SpiFlash_PageProgram(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;
    unsigned int Counter;

    // configure transaction length as 8 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x08 << SPI_CTL_TX_BIT_LEN_Pos);

    // /CS: active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk) | 0x1;

    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // /CS: de-active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk);

    // /CS: active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk) | 0x1;

    // send Command: 0x02, Page program
    au32SourceData = 0x02;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x18 << SPI_CTL_TX_BIT_LEN_Pos);

    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // configure transaction length as 8 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x08 << SPI_CTL_TX_BIT_LEN_Pos);

    for(Counter=0; Counter<ByteCount; Counter++) {
        // send   data to program
        au32SourceData = DataBuffer[Counter];
        SPI_FLASH_PORT->TX0 = au32SourceData;
        SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

        // wait
        while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}
    }

    // /CS: de-active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk);

}

/**
 * @brief       This function do the data reading from SPI Flash device.
 *
 * @param[in]   *DataBuffer A Point that point to destination data buffer.
 *
 * @param[in]   StartAddress    A start address of SPI Flash that will be read.
 *
 * @param[in]   ByteCount   Byte count number that will be read.
 *
 * @return      None
 */
void SpiFlash_ReadData(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;
    unsigned int Counter;

    // configure transaction length as 8 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x08 << SPI_CTL_TX_BIT_LEN_Pos);

    // /CS: active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk) | 0x1;

    // send Command: 0x03, Read data
    au32SourceData = 0x03;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x18 << SPI_CTL_TX_BIT_LEN_Pos);

    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI_FLASH_PORT->TX0 = au32SourceData;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait
    while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

    // configure transaction length as 8 bits
    SPI_FLASH_PORT->CTL = (SPI_FLASH_PORT->CTL & ~SPI_CTL_TX_BIT_LEN_Msk) | (0x08 << SPI_CTL_TX_BIT_LEN_Pos);

    for(Counter=0; Counter<ByteCount; Counter++) {
        // receive
        au32SourceData = 0x0;
        SPI_FLASH_PORT->TX0 = au32SourceData;
        SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

        // wait
        while (SPI_FLASH_PORT->CTL & SPI_CTL_GO_BUSY_Msk) {}

        // dump Rx register
        au32DestinationData = SPI_FLASH_PORT->RX0;
        DataBuffer[Counter] = (unsigned char) au32DestinationData;
    }

    // /CS: de-active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk);

}


/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
