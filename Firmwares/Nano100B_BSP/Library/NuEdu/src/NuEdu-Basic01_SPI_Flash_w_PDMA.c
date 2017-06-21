/**************************************************************************//**
 * @file     NuEdu-Basic01_SPI_Flash_w_PDMA.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 14/10/17 3:56p $
 * @brief    NuEdu-Basic01_SPI_Flash_w_PDMA driver source file for NuEdu-SDK-Nano130
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "Nano100Series.h"
#include "NuEdu-Basic01_SPI_Flash_w_PDMA.h"

/** @addtogroup NANO100_Library NANO100 Library
  @{
*/

/** @addtogroup NuEdu-SDK-Nano130_Basic01 Nano130_Basic01 Library
  @{
*/

/** @addtogroup Nano130_Basic01_FUNCTIONS SPI Flash with PDMA Functions
  @{
*/

/// @cond HIDDEN_SYMBOLS
/*---------------------------------------------------------------------------------------------------------*/
/*  Definitons                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
#define SPI_FLASH_PORT  SPI0

#define TEST_NUMBER         1   /* page numbers */
#define TEST_LENGTH         256 /* length */
#define CH1                     1
#define CH2                     2

#define MODE_PER2MEM        1
#define MODE_MEM2PER        2

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile    uint32_t    PDMA_CH1_INT_Flag;
volatile    uint32_t    PDMA_CH2_INT_Flag;
/// @endcond

/**
 * @brief       PDMA interrupt handler.
 *              Check the PDMA interrupt flag and clear the corresponding event flag.
 *
 * @return      None
 */
void PDMA_IRQHandler(void)
{
    uint32_t status = PDMAGCR->GCRISR;

    /* CH1 */
    if(status & 0x2) {
        if(PDMA_GET_CH_INT_STS(1) & 0x2) {
            PDMA_CH1_INT_Flag = 1;
            PDMA_CLR_CH_INT_FLAG(1, PDMA_ISR_TD_IS_Msk);
        }
        /* CH2 */
    } else if(status & 0x4) {
        if(PDMA_GET_CH_INT_STS(2) & 0x2) {
            PDMA_CH2_INT_Flag = 1;
            PDMA_CLR_CH_INT_FLAG(2, PDMA_ISR_TD_IS_Msk);
        }
    }

}

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
 * @brief       This function initializes the PDMA channel 1 for SPI0 transmitting TX and the data that will be transmiitted out
 *              are stored in the source buffer.
 *
 * @param[in]   u32SrcAddr  A source address for the transmitting data buffer.
 *
 * @return      None
 */
void Init_PDMA_CH1_for_SPI0_TX(uint32_t u32SrcAddr)
{
    uint32_t SPI0_TX;
    PDMA_T *PDMA_CH1;

    // PDMA Channel 1 control registers
    PDMA_CH1 = (PDMA_T *)((uint32_t) PDMA1_BASE + (0x100 * (CH1-1)));

    // SPI0 TX0 register
    SPI0_TX = SPI0_BASE + 0x20;

    // Enable DMA IP clock
    CLK->AHBCLK |= CLK_AHBCLK_DMA_EN_Msk;

    // Enable Channel 1 clock
    PDMAGCR->GCRCSR |= (CH1 << 9);

    // Set Channel 1 for SPI0_TX
    PDMAGCR->DSSR0 = (PDMAGCR->DSSR0 & ~DMA_GCR_DSSR0_CH1_SEL_Msk) | (PDMA_SPI0_TX << DMA_GCR_DSSR0_CH1_SEL_Pos);

    // Set Transfer Byte Count
    PDMA_CH1->BCR = TEST_LENGTH;

    // Set Source Address
    PDMA_CH1->SAR = u32SrcAddr;

    // Set Destination Address
    PDMA_CH1->DAR = SPI0_TX;

    // Set Transfer Width = 8 bits, Source Direction = INC, Destination Direction = FIX and Mode = Memory to Peripheral
    PDMA_CH1->CSR = (PDMA_CH1->CSR & ~(PDMA_CSR_APB_TWS_Msk | PDMA_CSR_SAD_SEL_Msk | PDMA_CSR_DAD_SEL_Msk | PDMA_CSR_MODE_SEL_Msk)) |
                    (PDMA_WIDTH_8 | PDMA_SAR_INC | PDMA_DAR_FIX | (MODE_MEM2PER << PDMA_CSR_MODE_SEL_Pos));

    // Enable Transfer Block Done Interrupt
    PDMA_CH1->IER = (PDMA_CH1->IER & ~(PDMA_IER_TABORT_IE_Msk | PDMA_IER_TD_IE_Msk)) | PDMA_IER_TD_IE_Msk;

}

/**
 * @brief       This function initializes the PDMA channel 2 for SPI0 receiving RX and the receiving data will
 *              be stored into the destination buffer.
 *
 * @param[in]   u32DstAddr  A destination address for the receiving data buffer.
 *
 * @return      None
 */
void Init_PDMA_CH2_for_SPI0_RX(uint32_t u32DstAddr)
{
    uint32_t SPI0_RX;
    PDMA_T *PDMA_CH2;

    // PDMA Channel 1 control registers
    PDMA_CH2 = (PDMA_T *)((uint32_t) PDMA1_BASE + (0x100 * (CH2-1)));

    // SPI0 TX0 register
    SPI0_RX = SPI0_BASE + 0x10;

    // Enable DMA IP clock
    CLK->AHBCLK |= CLK_AHBCLK_DMA_EN_Msk;

    // Enable Channel 2 clock
    PDMAGCR->GCRCSR |= (CH2 << 9);

    // Set Channel 2 for SPI0_RX
    PDMAGCR->DSSR0 = (PDMAGCR->DSSR0 & ~DMA_GCR_DSSR0_CH2_SEL_Msk) | (PDMA_SPI0_RX << DMA_GCR_DSSR0_CH2_SEL_Pos);

    // Set Transfer Byte Count
    PDMA_CH2->BCR = TEST_LENGTH;

    // Set Source Address
    PDMA_CH2->SAR = SPI0_RX;

    // Set Destination Address
    PDMA_CH2->DAR = u32DstAddr;

    // Set Transfer Width = 8 bits, Source Direction = FIX, Destination Direction = INC and Mode = Peripheral to Memory
    PDMA_CH2->CSR = (PDMA_CH2->CSR & ~(PDMA_CSR_APB_TWS_Msk | PDMA_CSR_SAD_SEL_Msk | PDMA_CSR_DAD_SEL_Msk | PDMA_CSR_MODE_SEL_Msk)) |
                    (PDMA_WIDTH_8 | PDMA_SAR_FIX | PDMA_DAR_INC | (MODE_PER2MEM << PDMA_CSR_MODE_SEL_Pos));

    // Enable Transfer Block Done Interrupt
    PDMA_CH2->IER = (PDMA_CH2->IER & ~(PDMA_IER_TABORT_IE_Msk | PDMA_IER_TD_IE_Msk)) | PDMA_IER_TD_IE_Msk;

}

/**
 * @brief       Read back the Manufacturer ID and Device ID from SPI Flash device.
 *
 * @return      Manufacturer ID and Device ID of SPI Flash device.
 *          For W25Q16BV, Manufacturer ID: 0xEF; Device ID: 0x14
 *      For W26X16, Manufacturer ID: 0xEF; Device ID: 0x14
*/
unsigned int SpiFlash_w_PDMA_ReadMidDid(void)
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
void SpiFlash_w_PDMA_ChipErase(void)
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
 * @brief       Read back the Status Register 1 from SPI Flash device
 *
 * @return      Status Register 1 value of SPI Flash device
 */
unsigned int SpiFlash_w_PDMA_ReadStatusReg1(void)
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
 * @brief       Read back the Status Register 2 from SPI Flash device
 *
 * @return      Status Register 2 value of SPI Flash device
 */
unsigned int SpiFlash_w_PDMA_ReadStatusReg2(void)
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
void SpiFlash_w_PDMA_WaitReady(void)
{
    unsigned int ReturnValue;

    do {
        ReturnValue = SpiFlash_w_PDMA_ReadStatusReg1();
        ReturnValue = ReturnValue & 1;
    } while(ReturnValue!=0); // check the BUSY bit

}

/**
 * @brief       This function do the page programming to SPI Flash device.
 *
 * @param[in]   StartAddress    A start address of SPI Flash that will be programmed.
 *
 * @param[in]   ByteCount   Byte count number that will be programmed.
 *
 * @return      None
 */
void SpiFlash_w_PDMA_PageProgram(unsigned int StartAddress, unsigned int ByteCount)
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

    // enable SPI PDMA
    SPI_FLASH_PORT->DMA = (SPI_FLASH_PORT->DMA & ~(SPI_DMA_RX_DMA_EN_Msk | SPI_DMA_TX_DMA_EN_Msk)) | SPI_DMA_TX_DMA_EN_Msk;

    // SPI go
    PDMA_CH1_INT_Flag = 0;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait PDMA done
    while (1) {
        if (PDMA_CH1_INT_Flag) {
            PDMA_CH1_INT_Flag = 0;
            break;
        }
    }

    // /CS: de-active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk);

}

/**
 * @brief       This function do the data reading from SPI Flash device.
 *
 * @param[in]   StartAddress    A start address of SPI Flash that will be read.
 *
 * @param[in]   ByteCount   Byte count number that will be read.
 *
 * @return      None
 */
void SpiFlash_w_PDMA_ReadData(unsigned int StartAddress, unsigned int ByteCount)
{
    unsigned int au32SourceData;

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

    // enable SPI PDMA
    SPI_FLASH_PORT->DMA = (SPI_FLASH_PORT->DMA & ~(SPI_DMA_RX_DMA_EN_Msk | SPI_DMA_TX_DMA_EN_Msk)) | SPI_DMA_RX_DMA_EN_Msk;

    // SPI go
    PDMA_CH2_INT_Flag = 0;
    SPI_FLASH_PORT->CTL |= SPI_CTL_GO_BUSY_Msk;

    // wait PDMA done
    while (1) {
        if (PDMA_CH2_INT_Flag) {
            PDMA_CH2_INT_Flag = 0;
            break;
        }
    }

    // /CS: de-active
    SPI_FLASH_PORT->SSR = (SPI_FLASH_PORT->SSR & ~SPI_SSR_SSR_Msk);

}


/*@}*/ /* end of group Nano130_Basic01_FUNCTIONS */

/*@}*/ /* end of group NuEdu-SDK-Nano130_Basic01 */

/*@}*/ /* end of group NANO100_Library */

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/

