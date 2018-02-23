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
    uint32_t u32IntSrc = UART_N->INTSTS;

    if (u32IntSrc & 0x11) { //RDA FIFO interrupt & RDA timeout interrupt
        while (((UART_N->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (bufhead < MAX_PKT_SIZE)) {	//RX fifo not empty
            uart_rcvbuf[bufhead++] = UART_N->DAT;
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
        while ((UART_N->FIFOSTS & UART_FIFOSTS_TXFULL_Msk));

        UART_N->DAT = response_buff[i];
    }
}

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_N->FUNCSEL = UART_FUNCSEL_UART;
    UART_N->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    UART_N->FIFO = UART_FIFO_RFITL_1BYTE | UART_FIFO_RTSTRGLV_1BYTE;
    UART_N->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__HSI, 115200));
    UART_N->TOUT = (UART_N->TOUT & ~UART_TOUT_TOIC_Msk) | (0x40);;
    NVIC_SetPriority(UART_N_IRQn, 2);
    NVIC_EnableIRQ(UART_N_IRQn);
    UART_N->INTEN = (UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_RDAIEN_Msk);
}
