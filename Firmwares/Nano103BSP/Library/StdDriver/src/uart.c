/**************************************************************************//**
 * @file     uart.c
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 15/12/29 2:46p $
 * @brief    Nano 103 Smartcard UART mode (UART) driver source file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include "Nano103.h"

/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_UART_Driver UART Driver
  @{
*/

/// @cond HIDDEN_SYMBOLS

/**
  * @brief  Get UART1 clock.
  * @param  None.
  * @return clk  uart clock.
  */
uint32_t _UART_GetUartClk(UART_T* uart)
{
    uint8_t u8UartClkSrcSel = 0;
    uint32_t clk =0, div = 1;

    if(uart == UART0)
    {
        u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART0SEL_Msk) >> CLK_CLKSEL1_UART0SEL_Pos;
        div = ( (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos) + 1; /* Get uart clock divider */
    }
    else if(uart == UART1)
    {
        u8UartClkSrcSel = (CLK->CLKSEL2 & CLK_CLKSEL2_UART1SEL_Msk) >> CLK_CLKSEL2_UART1SEL_Pos;
        div = ( (CLK->CLKDIV0 & CLK_CLKDIV0_UART1DIV_Msk) >> CLK_CLKDIV0_UART1DIV_Pos) + 1; /* Get uart clock divider */
    }

    switch (u8UartClkSrcSel)   /* Get uart selected clock source */
    {
    case 0:
        clk = __HXT; /* HXT */
        break;
    case 1:
        clk = __LXT;  /* LXT */
        break;
    case 2:
        clk = SysGet_PLLClockFreq(); /* PLL */
        break;
    case 3:
        if(CLK->CLKSEL0 & CLK_CLKSEL0_HIRCSEL_Msk)
            clk = __HIRC36M; /* HIRC 36M Hz*/
        else
        {
            if(CLK->PWRCTL & CLK_PWRCTL_HIRC0FSEL_Msk)
                clk = __HIRC16M; /* HIRC 16M Hz*/
            else
                clk = __HIRC12M; /* HIRC 12M Hz*/
        }
        break;

    default:
        clk = __MIRC;
        break;

    }

    clk /= div; /* calculate uart clock */

    return clk;
}

/// @endcond HIDDEN_SYMBOLS

/** @addtogroup NANO103_UART_EXPORTED_FUNCTIONS UART Exported Functions
  @{
*/

extern uint32_t SysGet_PLLClockFreq(void);



/**
 *    @brief The function is used to clear UART specified interrupt flag.
 *
 *    @param[in] uart                The base address of UART module.
 *    @param[in] u32InterruptFlag    The specified interrupt of UART module..
 *
 *    @return None
 */
void UART_ClearIntFlag(UART_T* uart, uint32_t u32InterruptFlag)
{

    if(u32InterruptFlag & UART_INTSTS_RLSIF_Msk)   /* clear Receive Line Status Interrupt */
    {
        uart->FIFOSTS = UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk;
        uart->TRSR = UART_TRSR_ADDRDETF_Msk;
    }

    if(u32InterruptFlag & UART_INTSTS_MODEMIF_Msk)  /* clear Modem Interrupt */
        uart->MODEM |= UART_MODEM_CTSDETF_Msk;

    if(u32InterruptFlag & UART_INTSTS_BUFERRIF_Msk)   /* clear Buffer Error Interrupt */
    {
        uart->FIFOSTS = UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_TXOVIF_Msk;
    }

    if(u32InterruptFlag & UART_INTSTS_WKUPIF_Msk)   /* clear wake up Interrupt */
    {
        uart->INTSTS = UART_INTSTS_WKUPIF_Msk;
    }

    if(u32InterruptFlag & UART_INTSTS_ABRIF_Msk)   /* clear auto-baud rate Interrupt */
    {
        uart->TRSR = UART_TRSR_ABRDTOIF_Msk | UART_TRSR_ABRDIF_Msk;
    }

    if(u32InterruptFlag & UART_INTSTS_LINIF_Msk)   /* clear LIN break Interrupt */
    {
        uart->TRSR = UART_TRSR_LINTXIF_Msk | UART_TRSR_LINRXIF_Msk | UART_TRSR_BITEF_Msk;
    }

}


/**
 *  @brief The function is used to disable UART.
 *
 *  @param[in] uart        The base address of UART module.
 *
 *  @return None
 */
void UART_Close(UART_T* uart)
{
    uart->INTEN = 0;
}


/**
 *  @brief The function is used to disable UART auto flow control.
 *
 *  @param[in] uart        The base address of UART module.
 *
 *  @return None
 */
void UART_DisableFlowCtrl(UART_T* uart)
{
    uart->CTRL &= ~(UART_CTRL_ATORTSEN_Msk | UART_CTRL_ATOCTSEN_Msk);
}


/**
 *    @brief    The function is used to disable UART specified interrupt.
 *
 *    @param[in]    uart                The base address of UART module.
 *    @param[in]    u32InterruptFlag    The specified interrupt of UART module.
 *                                - \ref UART_INTEN_TXENDIEN_Msk    : Tx transfer end interrupt
 *                                - \ref UART_INTEN_LINIEN_Msk      : LIN interrupt
 *                                - \ref UART_INTEN_WKUPIEN_Msk     : Wakeup interrupt
 *                                - \ref UART_INTEN_ABRIEN_Msk      : Auto Baud-rate interrupt
 *                                - \ref UART_INTEN_BUFERRIEN_Msk   : Buffer Error interrupt
 *                                - \ref UART_INTEN_RXTOIEN_Msk     : Rx time-out interrupt
 *                                - \ref UART_INTEN_MODEMIEN_Msk    : Modem interrupt
 *                                - \ref UART_INTEN_RLSIEN_Msk      : Rx Line status interrupt
 *                                - \ref UART_INTEN_THREIEN_Msk     : Tx empty interrupt
 *                                - \ref UART_INTEN_RDAIEN_Msk      : Rx ready interrupt
 *
 *    @return    None
 */
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag )
{
    uart->INTEN &= ~ u32InterruptFlag;
}



/**
 *    @brief    The function is used to Enable UART auto flow control.
 *
 *    @param[in]    uart    The base address of UART module.
 *
 *    @return    None
 */
void UART_EnableFlowCtrl(UART_T* uart )
{
    uart->MODEM |= UART_MODEM_RTSACTLV_Msk | UART_MODEM_CTSACTLV_Msk;
    uart->CTRL |= UART_CTRL_ATORTSEN_Msk | UART_CTRL_ATOCTSEN_Msk;
}


/**
 *    @brief    The function is used to enable UART specified interrupt.
 *
 *    @param[in]    uart                The base address of UART module.
 *    @param[in]    u32InterruptFlag    The specified interrupt of UART module:
 *                                - \ref UART_INTEN_TXENDIEN_Msk    : Tx transfer end interrupt
 *                                - \ref UART_INTEN_LINIEN_Msk      : LIN interrupt
 *                                - \ref UART_INTEN_WKUPIEN_Msk     : Wakeup interrupt
 *                                - \ref UART_INTEN_ABRIEN_Msk      : Auto Baud-rate interrupt
 *                                - \ref UART_INTEN_BUFERRIEN_Msk   : Buffer Error interrupt
 *                                - \ref UART_INTEN_RXTOIEN_Msk     : Rx time-out interrupt
 *                                - \ref UART_INTEN_MODEMIEN_Msk    : Modem interrupt
 *                                - \ref UART_INTEN_RLSIEN_Msk      : Rx Line status interrupt
 *                                - \ref UART_INTEN_THREIEN_Msk     : Tx empty interrupt
 *                                - \ref UART_INTEN_RDAIEN_Msk      : Rx ready interrupt
 *
 *    @return None
 */
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag )
{
    uart->INTEN |= u32InterruptFlag;
}


/**
 *    @brief    This function use to enable UART function and set baud-rate.
 *
 *    @param[in]    uart    The base address of UART module.
 *    @param[in]    u32baudrate    The baudrate of UART module.
 *
 *    @return    None
 */
void UART_Open(UART_T* uart, uint32_t u32baudrate)
{
    uint32_t u32Baud_Div;
    uint32_t u32SrcFreq;

    u32SrcFreq = _UART_GetUartClk(uart);

    uart->CTRL &= ~(UART_CTRL_RXOFF_Msk | UART_CTRL_TXOFF_Msk);

    uart->FUNCSEL = UART_FUNCSEL_UART;
    uart->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1 |
                 UART_LINE_RFITL_1BYTE | UART_LINE_RTS_TRI_LEV_1BYTE;

    if(u32baudrate != 0)
    {
        u32Baud_Div = UART_BAUD_MODE0_DIVIDER(u32SrcFreq, u32baudrate);

        if(u32Baud_Div > 0xFFFF)
            uart->BAUD = (UART_BAUD_MODE1 | UART_BAUD_MODE1_DIVIDER(u32SrcFreq, u32baudrate));
        else
            uart->BAUD = (UART_BAUD_MODE0 | u32Baud_Div);
    }
}


/**
 *    @brief    The function is used to read Rx data from RX FIFO and the data will be stored in pu8RxBuf.
 *
 *    @param[in]     uart            The base address of UART module.
 *    @param[out]    pu8RxBuf        The buffer to receive the data of receive FIFO.
 *    @param[in]     u32ReadBytes    The the read bytes number of data.
 *
 *  @return     u32Count: Receive byte count
 *
 */
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes)
{
    uint32_t  u32Count, u32delayno;

    for(u32Count=0; u32Count < u32ReadBytes; u32Count++)
    {
        u32delayno = 0;

        while(uart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk)   /* Check RX empty => failed */
        {
            u32delayno++;
            if( u32delayno >= 0x40000000 )
                return FALSE;
        }
        pu8RxBuf[u32Count] = uart->DAT;    /* Get Data from UART RX  */
    }

    return u32Count;

}


/**
 *    @brief    This function use to config UART line setting.
 *
 *    @param[in]    uart            The base address of UART module.
 *    @param[in]    u32baudrate        The register value of baudrate of UART module.
 *                            if u32baudrate = 0, UART baudrate will not change.
 *    @param[in]    u32data_width    The data length of UART module.
 *    @param[in]    u32parity        The parity setting (odd/even/none) of UART module.
 *    @param[in]    u32stop_bits    The stop bit length (1/1.5 bit) of UART module.
 *
 *    @return    None
 */
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits)
{
    uint32_t u32Baud_Div = 0;
    uint32_t u32SrcFreq;

    u32SrcFreq = _UART_GetUartClk(uart);

    if(u32baudrate != 0)
    {
        u32Baud_Div = UART_BAUD_MODE0_DIVIDER(u32SrcFreq, u32baudrate);

        if(u32Baud_Div > 0xFFFF)
            uart->BAUD = (UART_BAUD_MODE1 | UART_BAUD_MODE1_DIVIDER(u32SrcFreq, u32baudrate));
        else
            uart->BAUD = (UART_BAUD_MODE0 | u32Baud_Div);
    }

    uart->LINE = u32data_width | u32parity | u32stop_bits;
}


/**
 *    @brief    This function use to set Rx timeout count.
 *
 *    @param[in]    uart    The base address of UART module.
 *    @param[in]    u32TOC    Rx timeout counter.
 *
 *    @return    None
 */
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC)
{
    uart->TOUT = (uart->TOUT & ~UART_TOUT_TOIC_Msk)| (u32TOC);
    uart->INTEN |= UART_INTEN_RXTOIEN_Msk;
}


/**
 *    @brief    The function is used to configure IrDA relative settings. It consists of TX or RX mode and baudrate.
 *
 *    @param[in]    uart            The base address of UART module.
 *    @param[in]    u32Buadrate        The baudrate of UART module.
 *    @param[in]    u32Direction    The direction(transmit:1/receive:0) of UART module in IrDA mode.
 *
 *    @return    None
 */
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction)
{
    uint32_t u32SrcFreq;

    u32SrcFreq = _UART_GetUartClk(uart);

    uart->BAUD = UART_BAUD_MODE1 | UART_BAUD_MODE1_DIVIDER(u32SrcFreq, u32Buadrate);

    uart->IRDA    &=  ~UART_IRDA_TXINV_Msk;
    uart->IRDA |=     UART_IRDA_RXINV_Msk;
    uart->IRDA    = u32Direction ? uart->IRDA | UART_IRDA_TXEN_Msk : uart->IRDA &~ UART_IRDA_TXEN_Msk;
    uart->FUNCSEL = (0x2 << UART_FUNCSEL_FUNCSEL_Pos);
}


/**
 *    @brief    The function is used to set RS485 relative setting.
 *
 *    @param[in]    uart        The base address of UART module.
 *    @param[in]    u32Mode        The operation mode( \ref UART_ALTCTL_RS485NMM_Msk / \ref UART_ALTCTL_RS485AUD_Msk / \ref UART_ALTCTL_RS485AAD_Msk).
 *    @param[in]    u32Addr        The RS485 address.
 *
 *    @return    None
 */
void UART_SelectRS485Mode(UART_T* uart, uint32_t u32Mode, uint32_t u32Addr)
{
    uart->FUNCSEL = UART_FUNCSEL_RS485;
    uart->ALTCTL = 0;
    uart->ALTCTL |= u32Mode | (u32Addr << UART_ALTCTL_ADRMPID_Pos);
}

/**
 *    @brief        Select and configure LIN function
 *
 *    @param[in]    uart            The pointer of the specified UART module.
 *    @param[in]    u32Mode         The LIN direction :
 *                                  - UART_ALTCTL_LINTXEN_Msk
 *                                  - UART_ALTCTL_LINRXEN_Msk
 *                                  - (UART_ALTCTL_LINTXEN_Msk|UART_ALTCTL_LINRXEN_Msk)
 *    @param[in]    u32BreakLength  The breakfield length.
 *
 *    @return       None
 *
 *    @details      The function is used to set LIN relative setting.
 */
void UART_SelectLINMode(UART_T* uart, uint32_t u32Mode, uint32_t u32BreakLength)
{
    /* Select LIN function mode */
    uart->FUNCSEL = UART_FUNCSEL_LIN;

    /* Select LIN function setting : Tx enable, Rx enable and break field length */
    uart->FUNCSEL = UART_FUNCSEL_LIN;
    uart->ALTCTL &= ~(UART_ALTCTL_BRKFL_Msk | UART_ALTCTL_LINRXEN_Msk | UART_ALTCTL_LINTXEN_Msk);
    uart->ALTCTL |= u32BreakLength & UART_ALTCTL_BRKFL_Msk;
    uart->ALTCTL |= u32Mode;
}

/**
 *    @brief    The function is to write data into TX buffer to transmit data by UART.
 *
 *    @param[in]    uart            The base address of UART module.
 *    @param[in]    pu8TxBuf        The buffer to send the data to UART transmission FIFO.
 *    @param[in]    u32WriteBytes    The byte number of data.
 *
 *  @return u32Count: transfer byte count
 */
uint32_t UART_Write(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t  u32Count, u32delayno;

    for(u32Count=0; u32Count != u32WriteBytes; u32Count++)
    {
        u32delayno = 0;
        while((uart->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk) == 0)   /* Wait Tx empty and Time-out manner */
        {
            u32delayno++;
            if( u32delayno >= 0x40000000 )
                return FALSE;
        }
        uart->DAT = pu8TxBuf[u32Count];    /* Send UART Data from buffer */
    }

    return u32Count;

}


/*@}*/ /* end of group NANO103_UART_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_UART_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/



