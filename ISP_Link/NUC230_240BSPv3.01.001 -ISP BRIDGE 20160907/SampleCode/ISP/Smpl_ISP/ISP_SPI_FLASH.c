#include "NUC230_240.h"
#include "ISP_BSP.h"
void Open_SPI_Flash(void)
{

    /* Set PC0, PC1, PC2 and PC3 for SPI0 */
    SYS->GPC_MFP |= SYS_GPC_MFP_PC0_SPI0_SS0 | SYS_GPC_MFP_PC1_SPI0_CLK | SYS_GPC_MFP_PC2_SPI0_MISO0 | SYS_GPC_MFP_PC3_SPI0_MOSI0;

    /* Enable SPI2 IP clock */
    CLK->APBCLK |= CLK_APBCLK_SPI0_EN_Msk;

    /* Configure SPI2 as a master, MSB first, clock idle low, falling clock edge Tx, rising edge Rx and 32-bit transaction */
    //SPI2->CNTRL = SPI_CNTRL_MASTER_MODE | SPI_CNTRL_MSB_FIRST | SPI_CNTRL_CLK_IDLE_LOW | SPI_CNTRL_TX_FALLING |
    //            SPI_CNTRL_RX_RISING | SPI_CNTRL_TX_BIT_LEN(32);

    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 32, 2000000);
    /* Disable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    SPI_DisableAutoSS(SPI0);
    SPI_SET_SS0_HIGH(SPI0);

}
// **************************************
// For W25Q16BV, Manufacturer ID: 0xEF; Device ID: 0x14
// For W26X16, Manufacturer ID: 0xEF; Device ID: 0x14
unsigned int SpiFlash_ReadMidDid(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // configure transaction length as 8 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI0, 8);

    // /CS: active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI0);

    // send Command: 0x90, Read Manufacturer/Device ID
    au32SourceData = 0x90;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(24);
    SPI_SET_DATA_WIDTH(SPI0, 24);
    // send 24-bit '0', dummy
    au32SourceData = 0x0;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 16 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(16);
    SPI_SET_DATA_WIDTH(SPI0, 16);
    // receive
    au32SourceData = 0x0;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI0);
    // dump Rx register
    au32DestinationData = SPI0->RX[0];

    return (au32DestinationData & 0xffff);

}

// **************************************
void SpiFlash_ChipErase(void)
{
    unsigned int au32SourceData;

    // configure transaction length as 8 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI0, 8);
    // /CS: active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI0);

    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI0);
    // /CS: active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI0);
    // send Command: 0xC7, Chip Erase
    au32SourceData = 0xc7;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI0);
}

// **************************************
unsigned int SpiFlash_ReadStatusReg1(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // configure transaction length as 16 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(16);
    SPI_SET_DATA_WIDTH(SPI0, 16);
    // /CS: active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI0);
    // send Command: 0x05, Read status register 1
    au32SourceData = 0x0500;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI0);
    // dump Rx register
    au32DestinationData = SPI0->RX[0];

    return (au32DestinationData & 0xFF);

}


// **************************************
unsigned int SpiFlash_ReadStatusReg2(void)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;

    // configure transaction length as 16 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(16);
    SPI_SET_DATA_WIDTH(SPI0, 16);
    // /CS: active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI0);
    // send Command: 0x35, Read status register 2
    au32SourceData = 0x3500;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI0);
    // dump Rx register
    au32DestinationData = SPI0->RX[0];

    return (au32DestinationData & 0xFF);

}


// **************************************
void SpiFlash_WaitReady(void)
{
    unsigned int ReturnValue;

    do
    {
        ReturnValue = SpiFlash_ReadStatusReg1();
        ReturnValue = ReturnValue & 1;
    }
    while(ReturnValue != 0); // check the BUSY bit

}

// **************************************
void SpiFlash_PageProgram(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;
    unsigned int Counter;

    // configure transaction length as 8 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI0, 8);
    // /CS: active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI0);
    // send Command: 0x06, Write enable
    au32SourceData = 0x06;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // /CS: de-active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI0);
    // /CS: active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI0);
    // send Command: 0x02, Page program
    au32SourceData = 0x02;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(24);
    SPI_SET_DATA_WIDTH(SPI0, 24);
    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 8 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI0, 8);
    for(Counter = 0; Counter < ByteCount; Counter++)
    {
        // send   data to program
        au32SourceData = DataBuffer[Counter];
        SPI0->TX[0] = au32SourceData;
        SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

        // wait
        while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}
    }

    // /CS: de-active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI0);
}


// **************************************
void SpiFlash_ReadData(unsigned char *DataBuffer, unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;
    unsigned int au32DestinationData;
    unsigned int Counter;

    // configure transaction length as 8 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI0, 8);
    // /CS: active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_LOW;
    SPI_SET_SS0_LOW(SPI0);

    // send Command: 0x03, Read data
    au32SourceData = 0x03;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 24 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(24);
    SPI_SET_DATA_WIDTH(SPI0, 24);
    // send 24-bit start address
    au32SourceData = StartAddress;
    SPI0->TX[0] = au32SourceData;
    SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

    // wait
    while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

    // configure transaction length as 8 bits
    //SPI0->CNTRL = SPI0->CNTRL & (~SPI_CNTRL_TX_BIT_LEN_Msk) | SPI_CNTRL_TX_BIT_LEN(8);
    SPI_SET_DATA_WIDTH(SPI0, 8);

    for(Counter = 0; Counter < ByteCount; Counter++)
    {
        // receive
        au32SourceData = 0x0;
        SPI0->TX[0] = au32SourceData;
        SPI0->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;

        // wait
        while(SPI0->CNTRL & SPI_CNTRL_GO_BUSY_Msk) {}

        // dump Rx register
        au32DestinationData = SPI0->RX[0];
        DataBuffer[Counter] = (unsigned char) au32DestinationData;
    }

    // /CS: de-active
    //SPI0->SSR = SPI_SSR_SW_SS_PIN_HIGH;
    SPI_SET_SS0_HIGH(SPI0);
}

