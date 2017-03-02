#include <stdio.h>
#include <string.h>
#include "NUC505Series.h"
#include "spiflash_drv.h"
#include "ISP_USER.h"

BOOL bUartDataReady;

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

static uint8_t volatile	bufhead;

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable  XTAL */
    CLK->PWRCTL |= CLK_PWRCTL_HXTEN_Msk;

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select IP clock source */
    /* PCLK divider = 1 (/2) */
    CLK_SetModuleClock(PCLK_MODULE, 0, 1);
    /* UART0 clock source = XIN */
    CLK_SetModuleClock(UART0_MODULE, CLK_UART0_SRC_EXT, 0);

    /* Update System Core Clock */
    /* Note too high system clock will cause over-spec of SPI Flash read command on running SPIROM code. */
    CLK_SetCoreClock(100000000);
    SystemCoreClockUpdate();

    /* Init I/O multi-function pins */
    /* Configure multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB0MFP_Msk) ) | SYS_GPB_MFPL_PB0MFP_UART0_TXD;
    SYS->GPB_MFPL  = (SYS->GPB_MFPL & (~SYS_GPB_MFPL_PB1MFP_Msk) ) | SYS_GPB_MFPL_PB1MFP_UART0_RXD;

}

volatile uint8_t aprom_buf[4096];

void SysTimerDelay(uint32_t us)
{
    SysTick->LOAD = us * CyclesPerUs;
    SysTick->VAL   =  (0x00);
    SysTick->CTRL = SysTick->CTRL| SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Waiting for down-count to zero */
    while ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) == 0);   // Wait for down-count to zero.
}

#define UART_GET_TX_POINTER(UART)    (((UART)->FIFOSTS & UART_FIFOSTS_TXPTR_Msk) >> UART_FIFOSTS_TXPTR_Pos)

/**
send data for UART
**/
static /*__inline*/ void PutString()
{
    int i;

    for(i = 0; i < 64; i++) {
        while(UART_GET_TX_POINTER(UART0) >= 14);

        //while(UART0->FSR.TX_EMPTY == 0);
        UART0->DAT = g_u8SendBuf[i];
    }

}

/**
UART0/UART1 interrupt handler. handle receive only
**/
void UART0_IRQHandler(void)	//UART1_IRQHandler
{
    /*----- Determine interrupt source -----*/
    uint32_t u32IntSrc = inpw(&UART0->INTSTS);

    if(u32IntSrc & 0x11) { //RDA FIFO interrupt & RDA timeout interrupt
        while(((inpw(&UART0->FIFOSTS) & UART_FIFOSTS_RXEMPTY_Msk) == 0) && (bufhead < CMDBUFSIZE))	//RX fifo not empty
            g_u8RcvBuf[bufhead++] = inpw(&UART0->DAT);
    }

    if(bufhead == CMDBUFSIZE) {
        bUartDataReady = TRUE;
        bufhead = 0;
    } else if(u32IntSrc & 0x10) {
        bufhead = 0;
    }
}

int main(void)
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

    /* Init UART to 115200-8n1 for UART ISP command handler */
    UART_Open(UART0, 115200);
    /* Enable RDA\RLS\RTO Interrupt  */
    NVIC_SetPriority(UART0_IRQn , 2);
    UART_EnableInt(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    SysTick->LOAD = 300000 * CyclesPerUs; /* using 100MHz cpu clock*/
    SysTick->VAL   =  (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;//using cpu clock

    // Check the very first command which must be CMD_CONNECT
    while(1) {
        uint32_t lcmd;

        if((bufhead >= 4) || (bUartDataReady == TRUE)) {
            lcmd = inpw(g_u8RcvBuf);
            if(lcmd == CMD_CONNECT) {
                goto _ISP;
            } else {
                bUartDataReady = FALSE;
                bufhead = 0;
            }
        }

        if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)//timeout, then goto APROM
            goto _APROM;

    }

_ISP:

    while(1) {
        if(bUartDataReady == TRUE) {
            bUartDataReady = FALSE;
            ParseCmd(g_u8RcvBuf, CMDBUFSIZE, FALSE); //YT
            PutString();
        }
    }

_APROM:

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
