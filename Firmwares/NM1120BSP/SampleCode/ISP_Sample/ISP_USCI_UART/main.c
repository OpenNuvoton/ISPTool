#include <stdio.h>
#include "targetdev.h"
#include "usci_uart_transfer.h"
#include "isp_user.h"

void SYS_Init(void)
{
    /* Enable Internal and External RC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for Internal RC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

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
    SYS->GPD_MFP = (SYS->GPD_MFP & ~(SYS_GPD_MFP_PD5MFP_Msk | SYS_GPD_MFP_PD6MFP_Msk)) | (SYS_GPD_MFP_PD5_UART0_TXD | SYS_GPD_MFP_PD6_UART0_RXD);
}


/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    uint32_t rcvsize_bak = 0;
    uint32_t checkcnt = 0;
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 */
    USCI0_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    g_apromSize = 0x7600;  // 29.5 K
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock

    while (1) {
        if (rcvsize_bak != rcvsize) {
            rcvsize_bak = rcvsize;
            checkcnt = 0;
        } else if (rcvsize_bak != 0) {
            checkcnt++;

            if (checkcnt > 1000) {
                // Check the first isp command must be CMD_CONNECT
                if ((rcvsize_bak == 64) && (inpw(uart_rcvbuf) == CMD_CONNECT)) {
                    __set_PRIMASK(1);
                    rcvsize = 0;
                    bUartDataReady = TRUE;
                    __set_PRIMASK(0);
                    goto _ISP;
                } else {
                    // Drop Invalid Data & Reset index pointer for UART Buffer
                    __set_PRIMASK(1);
                    rcvsize = 0;
                    bufhead = 0;
                    bUartDataReady = FALSE;
                    __set_PRIMASK(0);
                    rcvsize_bak = 0;
                }
            }
        }

        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            goto _APROM;
        }
    }

_ISP:

    while (1) {
        if (bUartDataReady == TRUE) {
            bUartDataReady = FALSE;
            ParseCmd(uart_rcvbuf, 64);
            PutString();
            rcvsize_bak = 0;
            rcvsize = 0;
        }

        if (rcvsize_bak != rcvsize) {
            rcvsize_bak = rcvsize;
            checkcnt = 0;
        } else if (rcvsize_bak != 0) {
            checkcnt++;

            if (checkcnt > 1000) {
                if (rcvsize_bak == 64) {
                    __set_PRIMASK(1);
                    rcvsize = 0;
                    bUartDataReady = TRUE;
                    __set_PRIMASK(0);
                    // continue;
                } else {
                    // Drop Invalid Data & Reset index pointer for UART Buffer
                    __set_PRIMASK(1);
                    rcvsize = 0;
                    bufhead = 0;
                    __set_PRIMASK(0);
                }
            }
        }
    }

_APROM:
    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);
    FMC->ISPCTL &= ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}
