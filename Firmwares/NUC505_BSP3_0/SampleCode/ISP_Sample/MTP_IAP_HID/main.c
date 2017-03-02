#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"
#include "hid_transfer.h"
#include "ISP_USER.h"

#define DetectPin PB14_PIN

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Unlock protected registers */
    // SYS_UnlockReg();

    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    CLK_SetCoreClock(96000000);

    /* Set PCLK divider */
    CLK_SetModuleClock(PCLK_MODULE, NULL, 1);

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);

    /* Enable USB IP clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /* Select USB IP clock source */
    CLK_SetModuleClock(USBD_MODULE, CLK_USBD_SRC_EXT, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Relocate vector table in SRAM for fast interrupt handling. */
    /* YT modify for MTP */
    {
        extern uint32_t __Vectors[];
        extern uint32_t __Vectors_Size[];
        extern uint32_t Image$$ER_VECTOR2$$ZI$$Base[];
    
        //printf("Relocate vector table in SRAM (0x%08X) for fast interrupt handling.\n", Image$$ER_VECTOR2$$ZI$$Base);
        memcpy((void *) Image$$ER_VECTOR2$$ZI$$Base, (void *) __Vectors, (unsigned int) __Vectors_Size);
        SCB->VTOR = (uint32_t) Image$$ER_VECTOR2$$ZI$$Base;
    }
	
    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    if(DetectPin == 0) {
        USBD_Open(&gsInfo, HID_ClassRequest, NULL);

        /* Endpoint configuration */
        HID_Init();

        /* Enable USBD interrupt */
        NVIC_EnableIRQ(USBD_IRQn);

        /* Start transaction */
        USBD_Start();
    }

    while(DetectPin == 0) {
        if(bUsbDataReady == TRUE) {
            ParseCmd((uint8_t *)usb_rcvbuf, EPA_MAX_PKT_SIZE, TRUE);
            EPA_Handler();
            bUsbDataReady = FALSE;
        }
    }

    /* Jump to Address: 0x0000_4000 */
    // Reset_AfterREVMP(); // YT modify for MTP
    SYS->LVMPADDR = 0x00004000;
    SYS->LVMPLEN = 0x10;
    SYS->RVMPLEN = 0x01;
    SYS_ResetCPU();
    __NOP();
    __NOP();
    
    /* Trap the CPU */
    while(1);
}
