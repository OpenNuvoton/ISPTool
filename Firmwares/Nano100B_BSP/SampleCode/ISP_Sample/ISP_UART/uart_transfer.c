/**************************************************************************//**
 * @file     uart_transfer.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    General UART ISP slave Sample file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "targetdev.h"

#if (USE_ISP_UART)

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};
#else
__align(4) uint8_t  uart_rcvbuf[MAX_PKT_SIZE] = {0};
#endif

uint8_t volatile bUartDataReady = 0;
uint8_t volatile bufhead = 0;


/* please check "targetdev.h" for chip specifc define option */

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART_N_IRQHandler(void)
{
    /*----- Determine interrupt source -----*/
    uint32_t u32IntSrc = UART_N->ISR;

    if (u32IntSrc & 0x11) { //RDA FIFO interrupt & RDA timeout interrupt
        while (((UART_N->FSR & UART_FSR_RX_EMPTY_F_Msk) == 0) && (bufhead < MAX_PKT_SIZE)) { //RX fifo not empty
            uart_rcvbuf[bufhead++] = UART_N->RBR;
        }
    }

    if (bufhead == MAX_PKT_SIZE) {
        bUartDataReady = TRUE;
        bufhead = 0;
    } else if (u32IntSrc & 0x10) {
        bufhead = 0;
    }
}

extern __align(4) uint8_t response_buff[64];
void PutString(void)
{
    uint32_t i;

    for (i = 0; i < MAX_PKT_SIZE; i++) {
        while ((UART_N->FSR & UART_FSR_TX_FULL_F_Msk));

        UART_N->THR = response_buff[i];
    }
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_N->TLCTL = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1 |
                    UART_TLCTL_RFITL_14BYTES | UART_TLCTL_RTS_TRI_LEV_14BYTES;
    UART_N->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HIRC, 115200));
    UART_N->TMCTL = 0x40;
    NVIC_SetPriority(UART_N_IRQn, 2);
    NVIC_EnableIRQ(UART_N_IRQn);
    UART_N->IER = (UART_IER_RTO_IE_Msk | UART_IER_RDA_IE_Msk);
}
#endif // #if (USE_ISP_UART)
