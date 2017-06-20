/**************************************************************************//**
 * @file     uart_transfer.c
 * @version  V1.00
 * $Date: 14/11/17 5:36p $
 * @brief    General UART ISP slave Sample file
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/*!<Includes */
#include <string.h>
#include "targetdev.h"
#include "uart_transfer.h"

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

    if(u32IntSrc & 0x11) { //RDA FIFO interrupt & RDA timeout interrupt
        while(((UART_N->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (bufhead < MAX_PKT_SIZE))	//RX fifo not empty
            uart_rcvbuf[bufhead++] = UART_N->DAT;
    }

    if(bufhead == MAX_PKT_SIZE) {
        bUartDataReady = TRUE;
        bufhead = 0;
    } else if(u32IntSrc & 0x10) {
        bufhead = 0;
    }
}

extern __align(4) uint8_t response_buff[64];
void PutString(void)
{
    uint32_t i;

    for(i = 0; i < MAX_PKT_SIZE; i++) {
        while ((UART_N->FIFOSTS & UART_FIFOSTS_TXFULL_Msk));
        UART_N->DAT = response_buff[i];
    }
}

/*
uint32_t UART_IS_CONNECT(void)
{
    if((bufhead >= 4) || (bUartDataReady == TRUE)) {
        uint32_t lcmd;
        lcmd = inpw(uart_rcvbuf);
        if(lcmd == 0x000000AE) {	// CMD_CONNECT
            return 1;
        } else {
            bUartDataReady = 0;
            bufhead = 0;
        }
    }
    return 0;
}
*/

void UART_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
#if 1
//  UART_N->FUN_SEL = UART_FUNC_SEL_UART;
    UART_N->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    UART_N->FIFO = UART_FIFO_RFITL_14BYTES | UART_FIFO_RTSTRGLV_14BYTES;


//    UART_N->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(__IRC44M, 115200));

//    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk |
//                  (((__HSI + (115200/2)) / 115200)-2);

    UART0->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HSI, 115200));


//  UART_N->TOR = (UART_N->TOR & ~UART_TOR_TOIC_Msk)| (0x40);
    UART_N->TOUT = 0x40;

    NVIC_SetPriority (UART_N_IRQn, 2);
    NVIC_EnableIRQ(UART_N_IRQn);

    UART_N->INTEN = (UART_INTEN_TOCNTEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_RDAIEN_Msk);
		
#else		
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    // 22.1184 MHz reference clock input, for 115200 bps
    // 22118400 / 115200 = 192. Using mode 2 to calculate baudrate, 192 - 2 = 190 = 0xBE
    UART0->BAUD = UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk | (0xBE);

// 	 SYS->IPRST1 = SYS_IPRST1_UART0RST_Msk;
// 	 SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    // Set UART to 8 bit character length, 1 stop bit, and no parity
    UART0->LINE = UART_LINE_WLS_Msk;
    outpw(&UART0->FIFO, 0x32); //14
    outpw(&UART0->TOUT, 0x40);

    outpw(&UART0->INTEN, 0x811);
    NVIC_EnableIRQ(UART0_IRQn);		
#endif		
}

