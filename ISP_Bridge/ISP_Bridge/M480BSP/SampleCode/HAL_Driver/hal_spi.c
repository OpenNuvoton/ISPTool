#include "NuMicro.h"
#include <string.h>

#ifdef __cplusplus
extern "C"
{
#endif

void SPI1_Init(uint32_t Pclk0)
{
    uint32_t u32BusClock = 1000000;
    uint32_t u32Div = (((Pclk0 * 10U) / u32BusClock + 5U) / 10U) - 1U;
    u32Div &= 0xFF;
    /* Enable SPI1 peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_SPI1CKEN_Msk;
    /* Setup SPI1 multi-function pins */
    SYS->GPE_MFPL = (SYS->GPE_MFPL & ~(SYS_GPE_MFPL_PE0MFP_Msk | SYS_GPE_MFPL_PE1MFP_Msk))
                    | (SYS_GPE_MFPL_PE0MFP_SPI1_MOSI | SYS_GPE_MFPL_PE1MFP_SPI1_MISO);
    SYS->GPH_MFPH = (SYS->GPH_MFPH & ~(SYS_GPH_MFPH_PH8MFP_Msk | SYS_GPH_MFPH_PH9MFP_Msk))
                    | (SYS_GPH_MFPH_PH8MFP_SPI1_CLK | SYS_GPH_MFPH_PH9MFP_SPI1_SS);
    /* Enable SPI1 clock pin (PH8) schmitt trigger */
    PH->SMTEN |= GPIO_SMTEN_SMTEN8_Msk;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2MHz */
    // SPI_Open(SPI1, SPI_MASTER, SPI_MODE_0, 32, 1000000);
    /* Disable I2S mode */
    SPI1->I2SCTL &= ~SPI_I2SCTL_I2SEN_Msk;
    /* Default setting: slave selection signal is active low; disable automatic slave selection function. */
    SPI1->SSCTL = SPI_SS_ACTIVE_LOW;
    /* Default setting: MSB first, disable unit transfer interrupt, SP_CYCLE = 0. */
    SPI1->CTL = SPI_MASTER | ((32 & 0x1F) << SPI_CTL_DWIDTH_Pos) | (SPI_MODE_0) | SPI_CTL_SPIEN_Msk;
    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL2_SPI1SEL_Msk)) | CLK_CLKSEL2_SPI1SEL_PCLK0;
    SPI1->CLKDIV = (SPI1->CLKDIV & (~SPI_CLKDIV_DIVIDER_Msk)) | (u32Div << SPI_CLKDIV_DIVIDER_Pos);
    /* Enable the automatic hardware slave select function. Select the SS pin and configure as low-active. */
    // SPI_EnableAutoSS(SPI1, SPI_SS, SPI_SS_ACTIVE_LOW);
    // SPI1->SSCTL = (SPI1->SSCTL & (~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SSACTPOL_Msk | SPI_SSCTL_SS_Msk))) | (SPI_SS | SPI_SS_ACTIVE_LOW | SPI_SSCTL_AUTOSS_Msk);
    SPI1->SSCTL = (SPI1->SSCTL & (~(SPI_SSCTL_AUTOSS_Msk | SPI_SSCTL_SSACTPOL_Msk | SPI_SSCTL_SS_Msk)));
    /* Set TX FIFO threshold, enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    // SPI_SetFIFO(SPI1, 4, 4);
    SPI1->FIFOCTL = (SPI1->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk)) |
                    (4 << SPI_FIFOCTL_TXTH_Pos) |
                    (4 << SPI_FIFOCTL_RXTH_Pos);
    // SPI_EnableInt(SPI1, SPI_FIFO_TXTH_INT_MASK | SPI_FIFO_RXTO_INT_MASK);
    SPI1->FIFOCTL |= (SPI_FIFOCTL_TXTHIEN_Msk | SPI_FIFOCTL_RXTOIEN_Msk);
    // SPI2 Pins are connected with SPI1
    GPIO_SetMode(PG, BIT2, GPIO_MODE_INPUT);
    GPIO_SetMode(PG, BIT3, GPIO_MODE_INPUT);
    GPIO_SetMode(PG, BIT4, GPIO_MODE_INPUT);
    GPIO_SetMode(PF, BIT11, GPIO_MODE_INPUT);
}


uint32_t SPI1_Write(uint32_t *buf, uint32_t len)
{
    uint32_t u32Dummy, u32RxDataCount, u32TxDataCount;
    *buf = (*buf | 0x53504900);
    SPI1->SSCTL |= SPI_SSCTL_SS_Msk;
    u32TxDataCount = 0;
    u32RxDataCount = 0;

    /* Wait for transfer done */
    while ((u32RxDataCount < len) || (u32TxDataCount < len)) {
        /* Check TX FULL flag and TX data count */
        if ((SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0) && (u32TxDataCount < len)) {
            /* Write to TX FIFO */
            SPI_WRITE_TX(SPI1, buf[u32TxDataCount++]);
        }

        /* Check RX EMPTY flag */
        if ((SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0) && (u32RxDataCount < len)) {
            /* Read RX FIFO */
            u32Dummy += SPI_READ_RX(SPI1); // use "+=" instead of "+" to prevent warning about dummy read
            u32RxDataCount++;
        }
    }

    SPI1->SSCTL &= ~(SPI_SSCTL_SS_Msk);
    return 0;
}

uint32_t SPI1_Read(uint32_t *buf, uint32_t len)
{
    uint32_t u32RxDataCount, u32TxDataCount, u32Dummy;
    SPI1->SSCTL |= SPI_SSCTL_SS_Msk;
    u32TxDataCount = 0;
    u32RxDataCount = 0;
    u32Dummy = 0;

    /* Wait for transfer done */
    while ((u32RxDataCount < (len - 1)) || (u32TxDataCount < len)) {
        /* Check TX FULL flag and TX data count */
        if ((SPI_GET_TX_FIFO_FULL_FLAG(SPI1) == 0) && (u32TxDataCount < len)) {
            /* Write to TX FIFO */
            SPI_WRITE_TX(SPI1, u32TxDataCount++); // Don't care.
        }

        /* Check RX EMPTY flag */
        if ((SPI_GET_RX_FIFO_EMPTY_FLAG(SPI1) == 0) && (u32RxDataCount < (len - 1))) {
            if (!u32Dummy) {
                u32Dummy = SPI_READ_RX(SPI1);
                u32Dummy = 1;
            } else {
                /* Read RX FIFO */
                buf[u32RxDataCount++] = SPI_READ_RX(SPI1);
            }
        }
    }

    SPI1->SSCTL &= ~(SPI_SSCTL_SS_Msk);
    return 0;
}

#ifdef __cplusplus
}
#endif
