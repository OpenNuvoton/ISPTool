/****************************************************************************
 * @file     main.c
 * @version  V3.01
 * $Revision: 2 $
 * $Date: 15/02/24 10:39a $
 * @brief    Transmit and receive data from PC terminal through RS232 interface.
 * @note
 * Copyright (C) 2016-2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "Mini57Series.h"
#include "usci_uart_transfer.h"
#include "isp_user.h"

#define DetectPin   				PA0

void SYS_Init(void)
{
    /* Enable Internal and External RC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    //CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLKSEL_Msk);
    //CLK->CLKSEL0 |= CLK_HCLK_SRC_HIRC;
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_HCLK_SRC_HIRC;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLKDIV_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(1);
    SystemCoreClock = __HIRC / 1;        		// HCLK
    CyclesPerUs     = __HIRC / 1000000;  		// For SYS_SysTickDelay()

    /* Enable USCI module clock */
    CLK->APBCLK |= CLK_APBCLK_USCI0CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPD5 as output mode and GPD6 as Input mode */
    PD->MODE = (PD->MODE & (~GPIO_MODE_MODE5_Msk)) | (GPIO_MODE_OUTPUT << GPIO_MODE_MODE5_Pos);
    PD->MODE = (PD->MODE & (~GPIO_MODE_MODE6_Msk)) | (GPIO_MODE_INPUT << GPIO_MODE_MODE6_Pos);

    /* USCI-Uart0-GPD5(TX) + GPD6(RX) */
    /* Set GPD multi-function pins for USCI UART0 GPD5(TX) and GPD6(RX) */
    SYS->GPD_MFP = SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);
}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
//    /* Unlock protected registers */
    SYS_UnlockReg();

//    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 */
    USCI0_Init();

    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;

    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;

    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr , &g_dataFlashSize);

_ISP:
    while(DetectPin == 0) {
        if(bUartDataReady == TRUE) {
            bUartDataReady = FALSE;
            ParseCmd(uart_rcvbuf, 64);
            PutString();
        }
    }

_APROM:
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
		SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while(1);
}
