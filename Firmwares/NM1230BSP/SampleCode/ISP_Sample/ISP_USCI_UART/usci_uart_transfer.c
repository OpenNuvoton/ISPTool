/*!<Includes */
#include <string.h>
#include "targetdev.h"
#include "usci_uart_transfer.h"

__align(4) uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};

uint8_t volatile bUartDataReady = 0;
uint8_t volatile bufhead = 0;
uint32_t volatile rcvsize = 0;


/* please check "targetdev.h" for chip specifc define option */
void USCI0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset USCI0 */
    SYS->IPRST1 |=  SYS_IPRST1_USCI0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_USCI0RST_Msk;
    /* Configure USCI0 as UART mode */
    UUART0->CTL = (2 << UUART_CTL_FUNMODE_Pos);                                 /* Select UART function mode */
    UUART0->LINECTL = UUART_WORD_LEN_8 | UUART_LINECTL_LSB_Msk;                 /* Set UART line configuration */
    UUART0->DATIN0 = (2 << UUART_DATIN0_EDGEDET_Pos);                           /* Set falling edge detection */
    UUART0->BRGEN = (38 << UUART_BRGEN_CLKDIV_Pos) | (7 << UUART_BRGEN_DSCNT_Pos) | (1 << UUART_BRGEN_PDSCNT_Pos);
    UUART0->PROTCTL |= UUART_PROTCTL_PROTEN_Msk;                                /* Enable UART protocol */
    /* Enable USCI UART receive end interrupt */
    UUART_ENABLE_TRANS_INT(UUART0, UUART_INTEN_RXENDIEN_Msk);
    NVIC_EnableIRQ(USCI0_IRQn);
}

void USCI0_IRQHandler(void)
{
    uint32_t u32IntSts = UUART0->PROTSTS;

    if (u32IntSts & UUART_PROTSTS_RXENDIF_Msk) {
        /* Cleare receive end interrupt flag */
        UUART_CLR_PROT_INT_FLAG(UUART0, UUART_PROTSTS_RXENDIF_Msk);

        /* Get all the input characters */
        while (!UUART_IS_RX_EMPTY(UUART0)) {
            /* Get the character from USCI UART Buffer */
            uart_rcvbuf[bufhead++] = UUART_READ(UUART0);
            bufhead &= 0x3F;
            rcvsize++;
        }
    }
}

extern __align(4) uint8_t response_buff[64];
uint32_t PutString(void)
{
    uint32_t  u32Count, u32delayno;

    for (u32Count = 0; u32Count != MAX_PKT_SIZE; u32Count++) {
        u32delayno = 0;

        while ((UUART0->BUFSTS & UUART_BUFSTS_TXEMPTY_Msk) == 0) { /* Wait Tx empty */
            u32delayno++;

            if (u32delayno >= 0x40000000) {
                return FALSE;
            }
        }

        UUART0->TXDAT = response_buff[u32Count];    /* Send USCI_UART Data to buffer */
    }

    return u32Count;
}

