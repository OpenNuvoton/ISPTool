#include <stdio.h>
#include "NuMicro.h"

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t _CyclesPerUs      = 192; /* Cycles per micro second */

__weak uint32_t CLK_GetPLLClockFreq(void)
{
    return FREQ_192MHZ;
}

__weak uint32_t CLK_GetPCLK0Freq(void)
{
    return (FREQ_192MHZ / 2);
}

__weak uint32_t CLK_GetPCLK1Freq(void)
{
    return (FREQ_192MHZ / 2);
}

__weak void GPIO_SetMode(GPIO_T *port, uint32_t u32PinMask, uint32_t u32Mode)
{
    uint32_t i;

    for (i = 0ul; i < GPIO_PIN_MAX; i++) {
        if ((u32PinMask & (1ul << i)) == (1ul << i)) {
            port->MODE = (port->MODE & ~(0x3ul << (i << 1))) | (u32Mode << (i << 1));
        }
    }
}

#ifndef DEBUG_PORT
#define DEBUG_PORT   UART0
#endif

void UART_Init(void)
{
    if (DEBUG_PORT == (UART_T *)UART0) {
        /* Enable UART module clock */
        CLK->APBCLK0 |= CLK_APBCLK0_UART0CKEN_Msk;
        /* Select UART module clock source */
        CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART0SEL_Msk)) | CLK_CLKSEL1_UART0SEL_HXT;
        /* Set GPB multi-function pins for UART0 RXD and TXD */
        SYS->GPB_MFPH &= ~(SYS_GPB_MFPH_PB12MFP_Msk | SYS_GPB_MFPH_PB13MFP_Msk);
        SYS->GPB_MFPH |= (SYS_GPB_MFPH_PB12MFP_UART0_RXD | SYS_GPB_MFPH_PB13MFP_UART0_TXD);
    } else if (DEBUG_PORT == (UART_T *)UART2) {
        /* Enable UART module clock */
        CLK->APBCLK0 |= CLK_APBCLK0_UART2CKEN_Msk;
        /* Select UART module clock source */
        CLK->CLKSEL3 = (CLK->CLKSEL3 & (~CLK_CLKSEL3_UART2SEL_Msk)) | CLK_CLKSEL3_UART2SEL_HXT;
        /* Set GPE multi-function pins for UART2 RXD and TXD */
        SYS->GPE_MFPH = (SYS->GPE_MFPH & ~(SYS_GPE_MFPH_PE8MFP_Msk | SYS_GPE_MFPH_PE9MFP_Msk))
                        | (SYS_GPE_MFPH_PE8MFP_UART2_TXD | SYS_GPE_MFPH_PE9MFP_UART2_RXD);
    } else {
        return;
    }

    /* Init UART to 115200-8n1 for print message */
    DEBUG_PORT->FUNCSEL = UART_FUNCSEL_UART;
    DEBUG_PORT->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    DEBUG_PORT->FIFO = UART_FIFO_RFITL_1BYTE | UART_FIFO_RTSTRGLV_1BYTE;
    DEBUG_PORT->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200));

    if (DEBUG_PORT == (UART_T *)UART0) {
        printf("\n\n");
        printf("+-------------------------------------------------+\n");
        printf("|  M487 Pin Configuration                         |\n");
        printf("|  UART0: TXD (134), RXD (135)   -- ICE VCOM      |\n");
        printf("|  I2C2: SCL (94), SDA (95)      -- I2C Master    |\n");
        printf("|  I2C0: SCL (63), SDA (64)      -- I2C Slave     |\n");
        printf("|  SPI0: SS (65), CLK (66), MISO (67), MOSI (68)  |\n");
        printf("+-------------------------------------------------+\n");
        printf("\n");
    } else {
        printf("\n\n");
        printf("+-------------------------------------------------+\n");
        printf("|  Nu-Link2 Pin Configuration (CON6)              |\n");
        printf("|  UART2: TXD (1), RXD (2)                        |\n");
        printf("|  UI2C0: SCL (3), SDA (4)                        |\n");
        printf("|  SPI1: SS (5), CLK (6), MOSI (7), MISO (8)      |\n");
        printf("+-------------------------------------------------+\n");
        printf("\n");
    }
}

void PrintfBufByte(uint8_t *Buf, uint32_t len)
{
    uint32_t i = 0;

    for (i = 0; i < len; i++) {
        if ((i & 0x0F) == 0) {
            printf("\n0x%04X:", (i >> 4));
        }

        printf(" 0x%02x", Buf[i]);
    }
}

void PrintfBufWord(uint32_t *Buf, uint32_t len)
{
    uint32_t i = 0;
    printf("\n");

    for (i = 0; i < len; i++) {
        if ((i & 0x03) == 0) {
            printf("\n0x%04X:", (i >> 2));
        }

        printf(" 0x%08x", Buf[i]);
    }
}

#ifdef __cplusplus
}
#endif
