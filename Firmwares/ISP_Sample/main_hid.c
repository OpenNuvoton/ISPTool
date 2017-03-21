/******************************************************************************
 * @file     main.c
 * @brief
 *           Demonstrate how to transfer data between USB device and PC through USB HID interface.
 *           A windows tool is also included in this sample code to connect with USB device.
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "targetdev.h"
#include "clk_hxt.h"
#include "hid_transfer.h"
#include "hid_DetectPin.h"

#if defined(TARGET_NANO100A) || defined(TARGET_NANO100B) || defined(TARGET_NANO1X2) || defined(TARGET_NUC472_442)
/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
 // This variable is missing in system_Nano100Series.c
uint32_t PllClock = __HIRC;              /*!< PLL Output Clock Frequency         */
#endif

/*--------------------------------------------------------------------------*/
#if defined(TARGET_NUC472_442)
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    WDT->CTL &= ~(WDT_CTL_WDTEN_Msk);
    WDT->CTL |= (WDT_TIMEOUT_2POW18 | WDT_ALTCTL_RSTDSEL_Pos);

    /* Enable External XTAL (4~24 MHz) */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Waiting for 12MHz clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Switch HCLK clock source to HXT */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_CLKDIV0_HCLK(1));

    /* Set PLL to power down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL frequency */
    CLK->PLLCTL = CLK_PLLCTL_84MHz_HXT;

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_PLLSTB_Msk);

    /* Switch HCLK clock source to PLL */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_PLL,CLK_CLKDIV0_HCLK(1));

    /* Enable IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UARTSEL_HIRC, CLK_CLKDIV0_UART(1));

    /* Enable USB PHY */
    SYS->USBPHY = 0x100;  // USB device

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock. */
    SystemCoreClockUpdate();
}
#else
void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC clock and external XTAL clock */
    CLK->PWRCON |= PWRCON_SETTING;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_HXT_STB_Msk));

#if defined(PLLCON_SETTING)
    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(HCLK_DIV);
    CLK->CLKDIV &= ~CLK_CLKDIV_USB_N_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_USB(USBD_DIV);

    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            			// PLL
    SystemCoreClock = PLL_CLOCK / HCLK_DIV; 			// HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()
#else
    SystemCoreClock = __HIRC / 1;        		// HCLK
    CyclesPerUs     = __HIRC / 1000000;  		// For SYS_SysTickDelay()
#endif

#ifdef CLK_CLKSEL3_USBDSEL_PLL
    CLK->CLKSEL3 |= CLK_CLKSEL3_USBDSEL_PLL;
#endif

    /* Enable module clock */
    CLK->APBCLK |= CLK_APBCLK_USBD_EN_Msk;

#ifdef CLK_AHBCLK_ISP_EN_Msk
    CLK->AHBCLK |= CLK_AHBCLK_ISP_EN_Msk;
#endif
}
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Init system and multi-funcition I/O */
    SYS_Init();

    FMC->ISPCON |= FMC_ISPCON_ISPEN_Msk;	// (1ul << 0)

    g_apromSize = GetApromSize();
    GetDataFlashInfo(&g_dataFlashAddr , &g_dataFlashSize);

    while (DetectPin == 0) {

        /* Open USB controller */
        USBD_Open(&gsInfo, HID_ClassRequest, NULL);

        /*Init Endpoint configuration for HID */
        HID_Init();

        /* Start USB device */
        USBD_Start();

        /* Enable USB device interrupt */
        NVIC_EnableIRQ(USBD_IRQn);


        while (DetectPin == 0) {
            if(bUsbDataReady == TRUE) {
                ParseCmd((uint8_t *)usb_rcvbuf, 64);
                EP2_Handler();

                bUsbDataReady = FALSE;
            }
        }

        goto _APROM;
    }

    //CLK_SysTickDelay(300000);
    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL  = (0x00);
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);

_APROM:

    outpw(&SYS->RSTSRC, 3);//clear bit
    outpw(&FMC->ISPCON, inpw(&FMC->ISPCON) & 0xFFFFFFFC);
    outpw(&SCB->AIRCR, (V6M_AIRCR_VECTKEY_DATA | V6M_AIRCR_SYSRESETREQ));

    /* Trap the CPU */
    while(1);
}
/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/
