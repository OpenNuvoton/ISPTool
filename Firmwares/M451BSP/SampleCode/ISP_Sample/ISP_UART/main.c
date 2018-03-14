/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 14/10/28 9:01a $
 * @brief
 *           Show how to use auto baud rate detection function.
 *           This sample code needs to work with UART_AutoBaudRate_Slave.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M451Series.h"
#include "ISP_USER.h"

#define PLLCTL_SETTING  CLK_PLLCTL_72MHz_HIRC
#define PLL_CLOCK       71884880

uint8_t volatile bufhead = 0;
uint8_t volatile bUartDataReady = 0;
__align(4) uint8_t  uart_rcvbuf[64];

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
int32_t main(void);
void AutoBaudRate_TestItem(void);
void AutoBaudRate_TxTest(void);


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184MHz) */
    CLK->PWRCTL |= (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Select HCLK clock source as HIRC and and HCLK clock divider as 1 */
    //CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLKSEL_Msk;
    //CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_HCLKDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_HCLK(1);

    /* Set PLL to Power-down mode */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));
    //CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    //CLK->CLKSEL0 |= CLK_CLKSEL0_HCLKSEL_PLL;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UARTSEL_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UARTSEL_HIRC;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_UARTDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_UART(1);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PD multi-function pins for UART0 RXD(PD.0) and TXD(PD.1) */
    SYS->GPD_MFPL &= ~(SYS_GPD_MFPL_PD0MFP_Msk | SYS_GPD_MFPL_PD1MFP_Msk);
    SYS->GPD_MFPL |= (SYS_GPD_MFPL_PD0MFP_UART0_RXD | SYS_GPD_MFPL_PD1MFP_UART0_TXD);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS->IPRST1 |=  SYS_IPRST1_UART0RST_Msk;
    SYS->IPRST1 &= ~SYS_IPRST1_UART0RST_Msk;

    /* Configure UART0 and set UART0 baud rate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);
    UART0->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Set RTS Trigger Level as 14 bytes */
    UART0->FIFO = (UART0->FIFO & (~UART_FIFO_RTSTRGLV_Msk)) | UART_FIFO_RTSTRGLV_14BYTES;

    /* Set RX Trigger Level as 14 bytes */
    UART0->FIFO = (UART0->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_14BYTES;

    /* Enable RDA\RTO Interrupt */
    UART0->INTEN |= (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);

    /* Set Timeout time 0x3E bit-time and time-out counter enable */
    UART0->TOUT = (UART0->TOUT & ~UART_TOUT_TOIC_Msk) | (0x3E);
    UART0->INTEN |= UART_INTEN_TOCNTEN_Msk;

    /* Enable UART0 interrupt */
    NVIC_EnableIRQ(UART0_IRQn);
}

extern __align(4) uint8_t response_buff[64];
void PutString(void)
{
    uint32_t i;

    for(i = 0; i < 64; i++) {	
        while ((UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk));
        UART0->DAT = response_buff[i];
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    /*----- Determine interrupt source -----*/
    uint32_t u32IntSrc = UART0->INTSTS;

    if(u32IntSrc & 0x11) { //RDA FIFO interrupt & RDA timeout interrupt
        while(((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (bufhead < 64))	//RX fifo not empty
            uart_rcvbuf[bufhead++] = UART0->DAT;
    }

    if(bufhead == 64) {
        bUartDataReady = TRUE;
        bufhead = 0;
    } else if(u32IntSrc & 0x10) {
        bufhead = 0;
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    WDT->CTL &= ~WDT_CTL_WDTEN_Msk;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

//    /* Lock protected registers */
//    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;	// (1ul << 0)

    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr , &g_dataFlashSize);
		
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   =  (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock

    while(1) {
        if((bufhead >= 4) || (bUartDataReady == TRUE)) {
            uint32_t lcmd;
            lcmd = inpw(uart_rcvbuf);
            if(lcmd == CMD_CONNECT) {
                break;
            } else {
                bUartDataReady = FALSE;
                bufhead = 0;
            }
        }

        //if((SysTick->CTRL & (1 << 16)) != 0)//timeout, then goto APROM
        if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            goto _APROM;
    }

    while(1) {
        if (bUartDataReady == TRUE) {
            bUartDataReady = FALSE;
            ParseCmd(uart_rcvbuf, 64);
            PutString();
        }
    }

_APROM:

    outpw(&SYS->RSTSTS, 3);//clear bit
    outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while(1);

}

