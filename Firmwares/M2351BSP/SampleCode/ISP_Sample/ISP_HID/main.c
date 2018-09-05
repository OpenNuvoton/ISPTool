#include <stdio.h>
#include "NuMicro.h"
#include "hid_transfer.h"
#include "isp_user.h"

// For M2351 Ver. C API
volatile ISP_INFO_T     g_ISPInfo = {0};
volatile BL_USBD_INFO_T g_USBDInfo = {0};


#define DetectPin PB12

#define TRIM_INIT           (SYS_BASE+0x10C)

#define CLK_PLLCTL_144MHz_HXT   (CLK_PLLCTL_PLLSRC_HXT  | CLK_PLLCTL_NR(2) | CLK_PLLCTL_NF( 12) | CLK_PLLCTL_NO_1)

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 48MHz clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRC48EN_Msk;

    /* Waiting for Internal RC clock ready */
    while (!(CLK->STATUS & CLK_STATUS_HIRC48STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC48;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    /* Use HIRC48 as USB clock source */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_USBSEL_Msk)) | CLK_CLKSEL0_USBSEL_HIRC48;
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_USBDIV_Msk)) | CLK_CLKDIV0_USB(1);
    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_OTGPHYEN_Msk | SYS_USBPHY_SBO_Msk;
    /* Enable IP clock */
    CLK->APBCLK0 |= CLK_APBCLK0_USBDCKEN_Msk;
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SYS->GPA_MFPH &= ~(SYS_GPA_MFPH_PA12MFP_Msk | SYS_GPA_MFPH_PA13MFP_Msk | SYS_GPA_MFPH_PA14MFP_Msk | SYS_GPA_MFPH_PA15MFP_Msk);
    SYS->GPA_MFPH |= (SYS_GPA_MFPH_PA12MFP_USB_VBUS | SYS_GPA_MFPH_PA13MFP_USB_D_N | SYS_GPA_MFPH_PA14MFP_USB_D_P | SYS_GPA_MFPH_PA15MFP_USB_OTG_ID);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TrimInit;
    ISP_INFO_T      *pISPInfo;
    BL_USBD_INFO_T  *pUSBDInfo;
    pISPInfo = &g_ISPInfo;
    pUSBDInfo = &g_USBDInfo;
    memset((void *)&g_ISPInfo, 0x0, sizeof(ISP_INFO_T));
    memset((void *)&g_USBDInfo, 0x0, sizeof(BL_USBD_INFO_T));
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    if (DetectPin) {
        goto _APROM;
    }

    BL_USBDOpen(&gsInfo, NULL, NULL, (uint32_t *)pUSBDInfo);
    /* Endpoint configuration */
    HID_Init();
    BL_USBDInstallEPHandler(EP3, EP3_Handler, (uint32_t *)pISPInfo->pfnUSBDEP);
    NVIC_EnableIRQ(USBD_IRQn);
    BL_USBDStart();
    g_apromSize = BL_EnableFMC();
    g_dataFlashAddr = SCU->FNSADDR;

    if (g_dataFlashAddr < g_apromSize) {
        g_dataFlashSize = (g_apromSize - g_dataFlashAddr);
    } else {
        g_dataFlashSize = 0;
    }

    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while (DetectPin == 0) {
        /* Start USB trim if it is not enabled. */
        if ((SYS->TCTL48M & SYS_TCTL48M_FREQSEL_Msk) != 1) {
            /* Start USB trim only when SOF */
            if (USBD->INTSTS & USBD_INTSTS_SOFIF_Msk) {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
                /* Re-enable crystal-less */
                SYS->TCTL48M = 0x01;
                SYS->TCTL48M |= SYS_TCTL48M_REFCKSEL_Msk;
            }
        }

        /* Disable USB Trim when error */
        if (SYS->TISTS48M & (SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk)) {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;
            /* Disable crystal-less */
            SYS->TCTL48M = 0;
            /* Clear error flags */
            SYS->TISTS48M = SYS_TISTS48M_CLKERRIF_Msk | SYS_TISTS48M_TFAILIF_Msk;
            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }

        if (bUsbDataReady == 1) {
            ParseCmd((uint8_t *)usb_rcvbuf, 64);
            EP2_Handler();
            bUsbDataReady = 0;
        }
    }

_APROM:
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Trap the CPU */
    while (1);
}

void USBD_IRQHandler(void)
{
    BL_ProcessUSBDInterrupt((uint32_t *)g_ISPInfo.pfnUSBDEP, (uint32_t *)&g_ISPInfo, (uint32_t *)&g_USBDInfo);
}
