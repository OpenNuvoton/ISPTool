#include <stdio.h>
#include <string.h>
#include "targetdev.h"

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable External OSC49M */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);
    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV_HCLK(1));
    /* LDO On */
    CLK_EnableLDO(CLK_LDOSEL_3_3V);
    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set GPG multi-function pins for UART0 RXD and TXD */
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA10MFP_Msk)) | SYS_GPA_MFP_PA10MFP_UARTTX;
    SYS->GPA_MFP  = (SYS->GPA_MFP & (~SYS_GPA_MFP_PA11MFP_Msk)) | SYS_GPA_MFP_PA11MFP_UARTRX;
    /* Lock protected registers */
    // SYS_LockReg();
}

int main(void)
{
    /* Init System, IP clock and multi-function I/O */
    SYS_Init();
    /* Init UART to 115200-8n1 for print message */
    UART_Init();
    FMC->ISPCTL |=  FMC_ISPCTL_ISPEN_Msk;
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock

    while (1) {
        if ((bufhead >= 4) || (bUartDataReady == TRUE)) {
            uint32_t lcmd;
            lcmd = inpw(uart_rcvbuf);

            if (lcmd == CMD_CONNECT) {
                break;
            } else {
                bUartDataReady = FALSE;
                bufhead = 0;
            }
        }

        //if((SysTick->CTRL & (1 << 16)) != 0)//timeout, then goto APROM
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
            goto _APROM;
        }
    }

    while (1) {
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
    while (1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
