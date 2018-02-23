#include <stdio.h>
#include <string.h>
#include "targetdev.h"

// UART ===============================================================================
#define UART0_PINS_MSK    (SYS_GPA_MFP_PA4MFP_Msk|SYS_GPA_MFP_PA5MFP_Msk)
#define UART0_PINS        (SYS_GPA_MFP_PA4MFP_UART0_TX|SYS_GPA_MFP_PA5MFP_UART0_RX)

void SYS_Init(void)
{
    // Unlock protected registers
    SYS_UnlockReg();
    // Enable clock source
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC_EN);
    // Switch HCLK clock source to HIRC
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKSEL0_HIRCFSEL_48M, CLK_CLKDIV0_HCLK(1));
    // Enable LDO 3.3V
    CLK_EnableLDO(CLK_LDOSEL_3_3V);
    // Update System Core Clock
    SystemCoreClockUpdate();
    // Lock protected registers
    // SYS_LockReg();
    CLK_EnableModuleClock(UART0_MODULE);
    // Init I/O multi-function.
    // UART0: GPA4=TX, GPA5= RX
    SYS->GPA_MFP = (SYS->GPA_MFP & (~UART0_PINS_MSK)) | UART0_PINS;
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
