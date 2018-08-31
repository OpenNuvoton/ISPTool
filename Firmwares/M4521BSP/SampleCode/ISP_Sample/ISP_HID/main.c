#include <stdio.h>
#include "targetdev.h"

#define PLLCTL_SETTING      CLK_PLLCTL_72MHz_HIRC
#define PLL_CLOCK           72000000

#define TRIM_INIT           (GCR_BASE+0x12C)
/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Waiting for Internal RC clock ready */
    while ((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) == 0);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    /* Set PLL to Power-down mode */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;
    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCTL = PLLCTL_SETTING;

    while (!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk));

    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;
    /* Update System Core Clock */
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()
    // CLK_EnableXtalRC(CLK_PWRCTL_HIRC48MEN_Msk);
    CLK->PWRCTL |= CLK_PWRCTL_HIRC48MEN_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_USBCKSEL_Msk;
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
}

void USBD_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TrimInit;
    /* Unlock write-protected registers */
    SYS_UnlockReg();
    // Switch DetectPin to GPIO_MODE_QUASI mode
    PD->MODE = (PD->MODE & ~(0x3 << (0 << 1))) | (GPIO_MODE_QUASI << (0 << 1));
    /* Init system and multi-function I/O */
    SYS_Init();
    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;
    GetDataFlashInfo(&g_dataFlashAddr, &g_dataFlashSize);

    if (DetectPin == 0) {
        /* Open USB controller */
        USBD_Open(&gsInfo, HID_ClassRequest, NULL);
        /*Init Endpoint configuration for HID */
        HID_Init();
        /* Start USB device */
        USBD_Start();
        /* Backup init trim */
        u32TrimInit = M32(TRIM_INIT);
        /* Waiting for USB bus stable */
        USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

        while ((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);

        /* Enable USB crystal-less */
        SYS->IRC48MTCTL |= (SYS_IRC48MTCTL_REFCKSEL_Msk | 0x1);
        /* DO NOT Enable USB device interrupt */
        // NVIC_EnableIRQ(USBD_IRQn);
    }

    while (DetectPin == 0) {
        /* Re-start crystal-less when any error found */
        if (SYS->IRC48MTISTS & (SYS_IRC48MTISTS_TFAILIF_Msk | SYS_IRC48MTISTS_CLKERRIF_Msk)) {
            SYS->IRC48MTISTS = SYS_IRC48MTISTS_TFAILIF_Msk | SYS_IRC48MTISTS_CLKERRIF_Msk;
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;
            /* Waiting for USB bus stable */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

            while ((USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) == 0);

            /* Re-enable crystal-less */
            SYS->IRC48MTCTL |= (SYS_IRC48MTCTL_REFCKSEL_Msk | 0x1);
            //printf("USB trim fail. Just retry. SYS->HIRCTSTS = 0x%x, SYS->HIRCTCTL = 0x%x\n", SYS->HIRCTSTS, SYS->HIRCTCTL);
        } else {
            // polling USBD interrupt flag
            USBD_IRQHandler();

            if (bUsbDataReady == TRUE) {
                ParseCmd((uint8_t *)usb_rcvbuf, EP3_MAX_PKT_SIZE);
                EP2_Handler();
                bUsbDataReady = FALSE;
            }
        }
    }

    SYS->RSTSTS = (SYS_RSTSTS_PORF_Msk | SYS_RSTSTS_PINRF_Msk);//clear bit
    FMC->ISPCTL &=  ~(FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_BS_Msk);
    SCB->AIRCR = (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ);

    /* Trap the CPU */
    while (1);
}
