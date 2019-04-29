#include <stdio.h>
#include "targetdev.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define TEST_COUNT 16

uint32_t spi_rcvbuf[TEST_COUNT];
volatile uint32_t g_u32TxDataCount;
volatile uint32_t g_u32RxDataCount;

volatile uint8_t bSpiDataReady = 0;

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure SPI0 as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI0->CTL = SPI_SLAVE | SPI_CTL_TXNEG_Msk | SPI_CTL_SPIEN_Msk;
    /* Configure SPI0 as a low level active device. */
    SPI0->SSCTL = 0;
    /* Set IP clock divider. SPI peripheral clock rate = f_PCLK0 */
    SPI0->CLKDIV = 0;
    /* Set TX FIFO threshold and enable FIFO mode. */
    SPI0->FIFOCTL = (SPI0->FIFOCTL & ~(SPI_FIFOCTL_TXTH_Msk | SPI_FIFOCTL_RXTH_Msk)) |
                    (4 << SPI_FIFOCTL_TXTH_Pos) |
                    (4 << SPI_FIFOCTL_RXTH_Pos);
    /* Enable slave selection signal active interrupt flag */
    SPI0->SSCTL |= SPI_SSCTL_SSACTIEN_Msk;
    SPI_WRITE_TX(SPI0, 0xFFFFFFFF);    /* Dummy Write to prevent TX under run */
    NVIC_EnableIRQ(SPI0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  SPI0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SPI0_IRQHandler(void)
{
    uint32_t *_response_buff;
    _response_buff = (uint32_t *)response_buff; // in isp_user.c

    if (SPI0->STATUS & SPI_STATUS_SSACTIF_Msk) {
        SPI0->STATUS |= SPI_STATUS_SSACTIF_Msk;
        SPI0->FIFOCTL |= (SPI_FIFOCTL_RXFBCLR_Msk | SPI_FIFOCTL_TXFBCLR_Msk);
        g_u32TxDataCount = 0;
        g_u32RxDataCount = 0;

        // Active
        while (!(SPI0->STATUS & SPI_STATUS_SSINAIF_Msk)) {
            /* Check TX FULL flag and TX data count */
            if ((SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) && (g_u32TxDataCount < TEST_COUNT)) {
                SPI_WRITE_TX(SPI0, _response_buff[g_u32TxDataCount]);    /* Write to TX FIFO */
                g_u32TxDataCount++;
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                SysTick->LOAD = 1000 * CyclesPerUs;
                SysTick->VAL   = (0x00);
                SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
            }

            /* Check RX EMPTY flag */
            if (SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0) {
                g_u32RxDataCount &= 0x0F;
                spi_rcvbuf[g_u32RxDataCount++] = SPI_READ_RX(SPI0);    /* Read RX FIFO */
#ifdef ReadyPin
                // If hardware flow control pin is used, the slave side needs to pull this pin to high status before exiting irq.
                if ((g_u32RxDataCount == 1) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900)) {
                    ReadyPin = 1;
                }
#endif
            }

            if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
                /* Disable SysTick counter */
                SysTick->CTRL = 0UL;
                break;
            }
        }

        if (SPI0->STATUS & SPI_STATUS_SSINAIF_Msk) {
            SPI0->STATUS |= SPI_STATUS_SSINAIF_Msk;

            if ((g_u32RxDataCount == 16) && ((spi_rcvbuf[0] & 0xFFFFFF00) == 0x53504900)) {
                bSpiDataReady = 1;
            }

            spi_rcvbuf[0] &= 0x000000FF;
            g_u32TxDataCount = 0;
            g_u32RxDataCount = 0;

            if (SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) {
                SPI_WRITE_TX(SPI0, 0xFFFFFFFF);    /* Write to TX FIFO */
            }
        }
    } else {
    }
}
