/****************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/04/10 9:20a $
 * @brief    Transmit and receive data from PC terminal through RS232
 *           interface.
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */
			 
    /* Unlock protected registers */
    /* Unlock protected registers */
    while(SYS->REGLCTL != 1) {
        SYS->REGLCTL = 0x59;
        SYS->REGLCTL = 0x16;
        SYS->REGLCTL = 0x88;
    }

    /* Read User Config to select internal high speed RC */
    SystemInit();		
		
    /* Enable HIRC */
    CLK->PWRCTL |= (CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_XTLEN_Msk);

    /* Waiting for clock ready */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Enable IP clock */
    CLK->APBCLK |= CLK_APBCLK_UART0CKEN_Msk; // UART Clock Enable

    /* Select IP clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART0SEL_Msk;
    CLK->CLKSEL1 |= (0x2 << CLK_CLKSEL1_UART0SEL_Pos);// Clock source from HIRC clock

//    SystemCoreClock = __HSI;
//    CyclesPerUs = (SystemCoreClock + 500000) / 1000000;		
    CyclesPerUs = (__HSI) / 1000000;		
		
    /* Set P0 multi-function pins for UART1 RTS and CTS */
//    SYS->P0_MFP &= ~(SYS_MFP_P00_Msk | SYS_MFP_P01_Msk);
    SYS->P0_MFP |= (SYS_MFP_P00_TXD | SYS_MFP_P01_RXD);

    /* Init UART for printf */
    UART_Init();

//    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
//    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;

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
                goto _ISP;
            } else {
                bUartDataReady = FALSE;
                bufhead = 0;
            }
        }

        //if((SysTick->CTRL & (1 << 16)) != 0)//timeout, then goto APROM
        if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
            goto _APROM;
    }


_ISP:
    while(1) {
        if(bUartDataReady == TRUE) {
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
