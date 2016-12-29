/**************************************************************************//**
 * @file     uart.c
 * @version  V1.00
 * $Revision: 12 $
 * $Date: 15/09/25 10:10a $
 * @brief    MINI51 series UART driver source file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Mini51Series.h"

/** @addtogroup MINI51_Device_Driver MINI51 Device Driver
  @{
*/

/** @addtogroup MINI51_UART_Driver UART Driver
  @{
*/


/** @addtogroup MINI51_UART_EXPORTED_FUNCTIONS UART Exported Functions
  @{
*/

/**
 *    @brief The function is used to clear UART specified interrupt flag.
 *
 *    @param uart                The base address of UART module.
 *    @param u32InterruptFlag    The specified interrupt of UART module..
 *
 *    @return None
 */
void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag)
{

    if(u32InterruptFlag & UART_ISR_RLS_INT_Msk) { /* clear Receive Line Status Interrupt */
        uart->FSR |= UART_FSR_BIF_Msk | UART_FSR_FEF_Msk | UART_FSR_FEF_Msk;
        uart->FSR |= UART_FSR_RS485_ADD_DETF_Msk;
    }

    if(u32InterruptFlag & UART_ISR_MODEM_INT_Msk)  /* clear Modem Interrupt */
        uart->MSR |= UART_MSR_DCTSF_Msk;

    if(u32InterruptFlag & UART_ISR_BUF_ERR_INT_Msk) { /* clear Buffer Error Interrupt */
        uart->FSR |= UART_FSR_RX_OVER_IF_Msk | UART_FSR_TX_OVER_IF_Msk;
    }

    if(u32InterruptFlag & UART_ISR_TOUT_INT_Msk)  /* clear Modem Interrupt */
        uart->ISR |= UART_ISR_TOUT_IF_Msk;

}


/**
 *  @brief The function is used to disable UART.
 *
 *  @param uart        The base address of UART module.
 *
 *  @return None
 */
void UART_Close(UART_T* uart)
{
    uart->IER = 0;
}


/**
 *  @brief The function is used to disable UART auto flow control.
 *
 *  @param uart        The base address of UART module.
 *
 *  @return None
 */
void UART_DisableFlowCtrl(UART_T* uart)
{
    uart->IER &= ~(UART_IER_AUTO_RTS_EN_Msk | UART_IER_AUTO_CTS_EN_Msk);
}


/**
 *    @brief    The function is used to disable UART specified interrupt and disable NVIC UART IRQ.
 *
 *    @param    uart                The base address of UART module.
 *    @param    u32InterruptFlag    The specified interrupt of UART module.
 *                                - UART_IER_TIME_OUT_EN_Msk    : Rx Time Out interrupt
 *                                - UART_IER_WAKE_EN_Msk           : Wakeup interrupt
 *                                - UART_IER_BUF_ERR_IEN_Msk    : Buffer Error interrupt
 *                                - UART_IER_RTO_IEN_Msk         : Rx time-out interrupt
 *                                - UART_IER_MODEM_IEN_Msk       : Modem interrupt
 *                                - UART_IER_RLS_IEN_Msk         : Rx Line status interrupt
 *                                - UART_IER_THRE_IEN_Msk        : Tx empty interrupt
 *                                - UART_IER_RDA_IEN_Msk         : Rx ready interrupt
 *
 *    @return    None
 */
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag )
{
    uart->IER &= ~ u32InterruptFlag;
}



/**
 *    @brief    The function is used to Enable UART auto flow control.
 *
 *    @param    uart    The base address of UART module.
 *
 *    @return    None
 */
void UART_EnableFlowCtrl(UART_T* uart )
{
    uart->MCR |= UART_MCR_LEV_RTS_Msk;
    uart->MCR &= ~UART_MCR_RTS_Msk;
    uart->MSR |= UART_MSR_LEV_CTS_Msk;
    uart->IER |= UART_IER_AUTO_RTS_EN_Msk | UART_IER_AUTO_CTS_EN_Msk;
}


/**
 *    @brief    The function is used to enable UART specified interrupt and disable NVIC UART IRQ.
 *
 *    @param    uart                The base address of UART module.
 *    @param    u32InterruptFlag    The specified interrupt of UART module:
 *                                - UART_IER_TIME_OUT_EN_Msk    : Rx Time Out interrupt
 *                                - UART_IER_WAKE_EN_Msk           : Wakeup interrupt
 *                                - UART_IER_BUF_ERR_IEN_Msk    : Buffer Error interrupt
 *                                - UART_IER_RTO_IEN_Msk         : Rx time-out interrupt
 *                                - UART_IER_MODEM_IEN_Msk       : Modem interrupt
 *                                - UART_IER_RLS_IEN_Msk         : Rx Line status interrupt
 *                                - UART_IER_THRE_IEN_Msk        : Tx empty interrupt
 *                                - UART_IER_RDA_IEN_Msk         : Rx ready interrupt
 *
 *    @return None
 */
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag )
{
    uart->IER |= u32InterruptFlag;
}


/**
 *    @brief    This function use to enable UART function and set baud-rate.
 *
 *    @param    uart    The base address of UART module.
 *    @param    u32baudrate    The baudrate of UART module.
 *
 *    @return    None
 */
void UART_Open(UART_T* uart, uint32_t u32baudrate)
{
    uint8_t u8UartClkSrcSel;
    uint32_t u32ClkDiv = 0;
    uint32_t u32Clk = 0;
    uint32_t u32ClkTbl[4] = {__XTAL, 0, __IRC22M, __IRC22M};
    uint32_t u32Baud_Div;
    u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART_S_Msk) >> CLK_CLKSEL1_UART_S_Pos;

    u32ClkDiv = ( (CLK->CLKDIV & CLK_CLKDIV_UART_N_Msk) >> CLK_CLKDIV_UART_N_Pos );

    uart->FUN_SEL = UART_FUNC_SEL_UART;
    uart->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    uart->FCR = UART_FCR_RFITL_1BYTE | UART_FCR_RTS_TRI_LEV_1BYTE;

    u32Clk = u32ClkTbl[u8UartClkSrcSel]/(u32ClkDiv + 1);

    if(u32baudrate != 0) {
        u32Baud_Div = UART_BAUD_MODE2_DIVIDER(u32Clk, u32baudrate);

        if(u32Baud_Div > 0xFFFF)
            uart->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(u32Clk, u32baudrate));
        else
            uart->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);
    }

}


/**
 *    @brief    The function is used to read Rx data from RX FIFO and the data will be stored in pu8RxBuf.
 *
 *    @param    uart            The base address of UART module.
 *    @param    pu8RxBuf        The buffer to receive the data of receive FIFO.
 *    @param    u32ReadBytes    The the read bytes number of data.
 *
 *  @return     u32Count: Receive byte count
 *
 */
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes)
{
    uint32_t  u32Count, u32delayno;

    for(u32Count=0; u32Count < u32ReadBytes; u32Count++) {
        u32delayno = 0;

        while(uart->FSR & UART_FSR_RX_EMPTY_Msk) { /* Check RX empty => failed */
            u32delayno++;
            if( u32delayno >= 0x40000000 )
                return FALSE;
        }
        pu8RxBuf[u32Count] = uart->RBR;    /* Get Data from UART RX  */
    }

    return u32Count;

}


/**
 *    @brief    This function use to config UART line setting.
 *
 *    @param    uart            The base address of UART module.
 *    @param    u32baudrate        The register value of baudrate of UART module.
 *                            if u32baudrate = 0, UART baudrate will not change.
 *    @param    u32data_width    The data length of UART module.
 *    @param    u32parity        The parity setting (odd/even/none) of UART module.
 *    @param    u32stop_bits    The stop bit length (1/1.5 bit) of UART module.
 *
 *    @return    None
 */
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits)
{
    uint8_t u8UartClkSrcSel;
    uint32_t u32ClkDiv = 0;
    uint32_t u32Clk = 0;
    uint32_t u32ClkTbl[4] = {__XTAL, 0, __IRC22M, __IRC22M};
    uint32_t u32Baud_Div = 0;
    u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART_S_Msk) >> CLK_CLKSEL1_UART_S_Pos;

    u32ClkDiv = ( (CLK->CLKDIV & CLK_CLKDIV_UART_N_Msk) >> CLK_CLKDIV_UART_N_Pos );

    u32Clk = u32ClkTbl[u8UartClkSrcSel]/(u32ClkDiv + 1);

    if(u32baudrate != 0) {
        u32Baud_Div = UART_BAUD_MODE2_DIVIDER(u32Clk, u32baudrate);

        if(u32Baud_Div > 0xFFFF)
            uart->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(u32Clk, u32baudrate));
        else
            uart->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);
    }

    uart->LCR = u32data_width | u32parity | u32stop_bits;
}


/**
 *    @brief    This function use to set Rx timeout count.
 *
 *    @param    uart    The base address of UART module.
 *    @param    u32TOC    Rx timeout counter.
 *
 *    @return    None
 */
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC)
{
    uart->TOR = (uart->TOR & ~UART_TOR_TOIC_Msk)| (u32TOC);
    uart->IER |= UART_IER_TIME_OUT_EN_Msk;
}


/**
 *    @brief    The function is used to configure IrDA relative settings. It consists of TX or RX mode and baudrate.
 *
 *    @param    uart            The base address of UART module.
 *    @param    u32Buadrate        The baudrate of UART module.
 *    @param    u32Direction    The direction(transmit:1/receive:0) of UART module in IrDA mode.
 *
 *    @return    None
 */
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction)
{
    uint8_t u8UartClkSrcSel = 0;
    uint32_t u32ClkDiv = 0;
    uint32_t u32Clk = 0;
    uint32_t u32Baud_Div = 0;

    u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART_S_Msk) >> CLK_CLKSEL1_UART_S_Pos;

    u32ClkDiv = ( (CLK->CLKDIV & CLK_CLKDIV_UART_N_Msk) >> CLK_CLKDIV_UART_N_Pos );

    if(u8UartClkSrcSel == 0)
        u32Clk = __XTAL;
    else if(u8UartClkSrcSel >= 2)
        u32Clk = __IRC22M;

    u32Clk = u32Clk/(u32ClkDiv + 1);

    if(u32Buadrate != 0) {
        u32Baud_Div = UART_BAUD_MODE2_DIVIDER(u32Clk, u32Buadrate);

        if(u32Baud_Div > 0xFFFF)
            uart->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(u32Clk, u32Buadrate));
        else
            uart->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);
    }

    uart->IRCR    &=  ~UART_IRCR_INV_TX_Msk;
    uart->IRCR |=     UART_IRCR_INV_RX_Msk;
    uart->IRCR    = u32Direction ? uart->IRCR | UART_IRCR_TX_SELECT_Msk : uart->IRCR &~ UART_IRCR_TX_SELECT_Msk;
    uart->FUN_SEL = (0x2 << UART_FUN_SEL_FUN_SEL_Pos);
}


/**
 *    @brief    The function is used to set RS485 relative setting.
 *
 *    @param    uart        The base address of UART module.
 *    @param    u32Mode        The operation mode(NMM/AUD/AAD).
 *    @param    u32Addr        The RS485 address.
 *
 *    @return    None
 */
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr)
{
    uart->FUN_SEL = UART_FUNC_SEL_RS485;
    uart->ALT_CSR = 0;
    uart->ALT_CSR |= u32Mode | (u32Addr << UART_ALT_CSR_ADDR_MATCH_Pos);
}


/**
 *    @brief    The function is to write data into TX buffer to transmit data by UART.
 *
 *    @param    uart            The base address of UART module.
 *    @param    pu8TxBuf        The buffer to send the data to UART transmission FIFO.
 *    @param    u32WriteBytes    The byte number of data.
 *
 *  @return u32Count: transfer byte count
 */
uint32_t UART_Write(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t  u32Count, u32delayno;

    for(u32Count=0; u32Count != u32WriteBytes; u32Count++) {
        u32delayno = 0;
        while((uart->FSR & UART_FSR_TE_FLAG_Msk) == 0) { /* Wait Tx empty and Time-out manner */
            u32delayno++;
            if( u32delayno >= 0x40000000 )
                return FALSE;
        }
        uart->THR = pu8TxBuf[u32Count];    /* Send UART Data from buffer */
    }

    return u32Count;

}


/*@}*/ /* end of group MINI51_UART_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group MINI51_UART_Driver */

/*@}*/ /* end of group MINI51_Device_Driver */

/*** (C) COPYRIGHT 2012 Nuvoton Technology Corp. ***/



