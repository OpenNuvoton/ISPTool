#include <stdio.h>
#include "targetdev.h"

#define PLLCTL_SETTING  CLK_PLLCTL_144MHz_HXT
#define PLL_CLOCK       144000000

/* If crystal-less is enabled, system won't use any crystal as clock source
   If using crystal-less, system will be 48MHz, otherwise, system is 72MHz
*/
#define CRYSTAL_LESS        1
#define HIRC48_AUTO_TRIM    0x412   /* Use USB signal to fine tune HIRC 48MHz */
#define TRIM_INIT           (SYS_BASE+0x118)


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 22.1184 MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for Internal RC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);
#ifndef CRYSTAL_LESS
    /* Enable external XTAL 12 MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for external XTAL clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HXTSTB_Msk));

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCTL = PLLCTL_SETTING;

    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_PLL;
    /* Select HCLK clock divider as 2 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(2);
    CLK->CLKSEL3 |= CLK_CLKSEL3_USBDSEL_PLL;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_USBDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_USB(3);
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 2;        // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()
#else
    /* Enable Internal RC 48MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRC48EN_Msk;

    /* Waiting for Internal RC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRC48STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_HCLKSEL_Msk) | CLK_CLKSEL0_HCLKSEL_HIRC48;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & ~CLK_CLKDIV0_HCLKDIV_Msk) | CLK_CLKDIV0_HCLK(1);
    CLK->CLKSEL3 &= ~CLK_CLKSEL3_USBDSEL_Msk;
    CLK->CLKDIV0 &= ~CLK_CLKDIV0_USBDIV_Msk;
    CLK->CLKDIV0 |= CLK_CLKDIV0_USB(1);
    SystemCoreClock = 48000000;        // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()
#endif
    /* Enable module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TrimInit;
    /* Unlock write-protected registers */
    SYS_UnlockReg();
    WDT->CTL &= ~(WDT_CTL_WDTEN_Msk | WDT_CTL_ICEDEBUG_Msk);
    WDT->CTL |= (WDT_TIMEOUT_2POW18 | WDT_CTL_RSTEN_Msk);
    /* Init system and multi-funcition I/O */
    SYS_Init();
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;	// (1ul << 0)
    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    if (DetectPin != 0) {
        goto _APROM;
    }

    /* Open USB controller */
    USBD_Open(&gsInfo, HID_ClassRequest, NULL);
    /*Init Endpoint configuration for HID */
    HID_Init();
    /* Start USB device */
    USBD_Start();
    /* Enable USB device interrupt */
    NVIC_EnableIRQ(USBD_IRQn);
#ifdef CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif
    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while (DetectPin == 0) {
#ifdef CRYSTAL_LESS

        /* Start USB trim if it is not enabled. */
        if ((SYS->IRCTCTL1 & SYS_IRCTCTL1_FREQSEL_Msk) != 2) {
            /* Start USB trim only when SOF */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
                /* Re-enable crystal-less */
                SYS->IRCTCTL1 = HIRC48_AUTO_TRIM;
            }
        }

        /* Disable USB Trim when error */
        if (SYS->IRCTISTS & (SYS_IRCTISTS_CLKERRIF1_Msk | SYS_IRCTISTS_TFAILIF1_Msk)) {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;
            /* Disable crystal-less */
            SYS->IRCTCTL1 = 0;
            /* Clear error flags */
            SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF1_Msk | SYS_IRCTISTS_TFAILIF1_Msk;
            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }

#endif

        if (bUsbDataReady == TRUE) {
            WDT->CTL &= ~(WDT_CTL_WDTEN_Msk | WDT_CTL_ICEDEBUG_Msk);
            WDT->CTL |= (WDT_TIMEOUT_2POW18 | WDT_CTL_RSTEN_Msk);
            ParseCmd((uint8_t *)usb_rcvbuf, EP3_MAX_PKT_SIZE);
            EP2_Handler();
            bUsbDataReady = FALSE;
        }
    }

_APROM:
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

    outpw(&SYS->RSTSTS, 3);//clear bit
    outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while (1);
}

