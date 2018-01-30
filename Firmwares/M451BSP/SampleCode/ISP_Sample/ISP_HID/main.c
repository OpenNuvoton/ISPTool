/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief
 *           ISP HID sample.
 * @note
 * Copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock and external XTAL clock*/
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk;

    /* Waiting for Internal RC clock ready */
    while((CLK->STATUS & CLK_STATUS_HIRCSTB_Msk) == 0);
    /* Waiting for external XTAL clock ready */
    while((CLK->STATUS & CLK_STATUS_HXTSTB_Msk) == 0);

    /* Set Flash Access Delay */
    FMC->FTCTL |= FMC_FTCTL_FOM_Msk;

    /* Enable and apply new PLL setting. */
    CLK->PLLCTL = CLK_PLLCTL_144MHz_HXT;//144Mhz

    /* Wait for PLL clock stable */
    while((CLK->STATUS & CLK_STATUS_PLLSTB_Msk) == 0);

    /* Apply new Divider */
    CLK->CLKDIV0 = (1 << CLK_CLKDIV0_HCLKDIV_Pos) | (2 << CLK_CLKDIV0_USBDIV_Pos);

    /* Switch HCLK to new HCLK source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    SystemCoreClock = 72000000;
    PllClock = 144000000;
    CyclesPerUs = 72;

    /* Enable module clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
    SYS->USBPHY = SYS_USBPHY_LDO33EN_Msk;
}

void USBD_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Init system and multi-function I/O */
    SYS_Init();

    CLK->AHBCLK |= CLK_AHBCLK_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk;

    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr , &g_dataFlashSize);

    if(DetectPin == 0) {
        /* Open USB controller */
        USBD_Open(&gsInfo, HID_ClassRequest, NULL);

        /*Init Endpoint configuration for HID */
        HID_Init();

        /* Start USB device */
        USBD_Start();

        /* DO NOT Enable USB device interrupt */
        // NVIC_EnableIRQ(USBD_IRQn);
    }
		
    while(DetectPin == 0) {
        // polling USBD interrupt flag
        USBD_IRQHandler();

        if(bUsbDataReady == TRUE) {
            WDT->CTL &= ~(WDT_CTL_WDTEN_Msk | WDT_CTL_ICEDEBUG_Msk);
            WDT->CTL |= (WDT_CTL_RSTCNT_Msk | WDT_TIMEOUT_2POW18);
            ParseCmd((uint8_t *)usb_rcvbuf, EP3_MAX_PKT_SIZE);
            EP2_Handler();

            bUsbDataReady = FALSE;
        }
    }

    outpw(&SYS->RSTSTS, 3);//clear bit
    outpw(&FMC->ISPCTL, inpw(&FMC->ISPCTL) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while(1);
}

/*** (C) COPYRIGHT 2014~2015 Nuvoton Technology Corp. ***/

