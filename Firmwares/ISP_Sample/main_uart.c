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
#include "targetdev.h"
#include "clk_hirc.h"
#include "uart_transfer.h"

#if defined(TARGET_NANO100A) || defined(TARGET_NANO100B) || defined(TARGET_NANO1X2) || defined(TARGET_NUC472_442)
/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
 // This variable is missing in system_Nano100Series.c
uint32_t PllClock = __HIRC;              /*!< PLL Output Clock Frequency         */
#endif


void SYS_Init(void)
{
    /* Enable Internal and External RC clock */
    CLK->PWRCON |= PWRCON_SETTING;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_HIRC_STB_Msk));

#if defined(TARGET_NANO103)
    CLK_EnablePLL(CLK_PLLCTL_PLL_SRC_MIRC, PLL_CLOCK);
    CLK_WaitClockReady( CLK_STATUS_PLLSTB_Msk);
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL, CLK_HCLK_CLK_DIVIDER(1));
#elif defined(PLLCON_SETTING)
    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(HCLK_DIV);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            			// PLL
    SystemCoreClock = PLL_CLOCK / HCLK_DIV;				// HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()
#else
    SystemCoreClock = __HIRC / 1;        		// HCLK
    CyclesPerUs     = __HIRC / 1000000;  		// For SYS_SysTickDelay()
#endif

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HIRC;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    UART_MFP_Setting();
}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 */
    UART_Init();

#ifdef CLK_AHBCLK_ISP_EN_Msk
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
#endif
    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;

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
    SYS->RSTSRC = (SYS_RSTSRC_RSTS_POR_Msk | SYS_RSTSRC_RSTS_RESET_Msk);
    FMC->ISPCON &= ~(FMC_ISPCON_ISPEN_Msk | FMC_ISPCON_BS_Msk);
		SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while(1);
}
