/**************************************************************************//**
 * @file     uart.c
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/08/01 2:45p $
 * @brief    I91200 UART driver source file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "Platform.h"

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_UART_Driver UART Driver
  @{
*/


/** @addtogroup I91200_UART_EXPORTED_FUNCTIONS UART Exported Functions
  @{
*/


/**
 *    @brief  The function is used to clear UART specified interrupt flag.
 *
 *    @param  uart                The base address of UART module.
 *    @param  u32InterruptFlag    The specified interrupt of UART module..
 *
 *    @return None
 */
void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag)
{
    if(u32InterruptFlag & UART_INTSTS_RLSINT_Msk)      /* clear Receive Line Status Interrupt */
        uart->FIFOSTS |= UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk;

    if(u32InterruptFlag & UART_INTSTS_MODEMINT_Msk)    /* clear Modem Interrupt */
        uart->MODEMSTS |= UART_MODEMSTS_CTSDETF_Msk;

    if(u32InterruptFlag & UART_INTSTS_BUFERRINT_Msk)   /* clear Buffer Error Interrupt */
        uart->FIFOSTS |= ( UART_FIFOSTS_RXOVIF_Msk | UART_FIFOSTS_TXOVIF_Msk );

    if(u32InterruptFlag & UART_INTSTS_RXTOINT_Msk)     /* clear Modem Interrupt */
        uart->INTSTS |= UART_INTSTS_RXTOIF_Msk;
}

/**
 *  @brief  The function is used to disable UART.
 *
 *  @param  uart        The base address of UART module.
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
 *  @param uart        The base address of UART module.
 *
 *  @return None
 */
void UART_DisableFlowCtrl(UART_T* uart)
{
    uart->INTEN &= ~(UART_INTEN_ATORTSEN_Msk | UART_INTEN_ATOCTSEN_Msk);
}

/**
 *    @brief    The function is used to Enable UART auto flow control.
 *
 *    @param    uart    The base address of UART module.
 *
 *    @return   None
 */
void UART_EnableFlowCtrl(UART_T* uart )
{
    uart->MODEM    |= UART_MODEM_RTSACTLV_Msk;
    uart->MODEM    &= UART_MODEM_RTS_Msk;
    uart->MODEMSTS |= UART_MODEMSTS_CTSACTLV_Msk;
    uart->INTEN    |= UART_INTEN_ATORTSEN_Msk | UART_INTEN_ATOCTSEN_Msk;
}

/**
 *    @brief    This function use to enable UART function and set baud-rate.
 *
 *    @param    uart           The base address of UART module.
 *    @param    u32baudrate    The baudrate of UART module.
 *
 *    @return   None
 */
void UART_Open(UART_T* uart, uint32_t u32baudrate)
{
    uint32_t u32Clk;
    uint32_t u32Baud_Div;

    uart->FUNCSEL = UART_FUNCSEL_UART;
    uart->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
    uart->FIFO = UART_FIFO_RFITL_1BYTE | UART_FIFO_RTSTRGLV_1BYTE;

    u32Clk = CLK_GetHCLKFreq() / (((CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos) + 1);

    if(u32baudrate != 0) 
	{
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
 *    @return   u32Count: Receive byte count
 *
 */
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes)
{
    uint32_t  u32Count;

    for(u32Count=0; u32Count < u32ReadBytes; u32Count++) 
	{
        if(uart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk)  /* Check RX empty => failed */
            return u32Count;
        
        pu8RxBuf[u32Count] = uart->DAT;               /* Get Data from UART RX  */
    }

    return u32Count;
}

/**
 *    @brief    This function use to config UART line setting.
 *
 *    @param    uart            The base address of UART module.
 *    @param    u32baudrate     The register value of baudrate of UART module.
 *                              if u32baudrate = 0, UART baudrate will not change.
 *    @param    u32data_width   The data length of UART module. [UART_WORD_LEN_5 / UART_WORD_LEN_6 / UART_WORD_LEN_7 / UART_WORD_LEN_8]
 *    @param    u32parity       The parity setting (odd/even/none) of UART module. [UART_PARITY_NONE / UART_PARITY_ODD / UART_PARITY_EVEN / UART_PARITY_MARK / UART_PARITY_SPACE]
 *    @param    u32stop_bits    The stop bit length (1/1.5/2 bit) of UART module. [UART_STOP_BIT_1 / UART_STOP_BIT_1_5 / UART_STOP_BIT_2]
 *
 *    @return   None
 */
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits)
{
    uint32_t u32Clk;
    uint32_t u32Baud_Div = 0;
	
    u32Clk = CLK_GetHCLKFreq() / (((CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos) + 1);

    if(u32baudrate != 0) 
	{
        u32Baud_Div = UART_BAUD_MODE2_DIVIDER(u32Clk, u32baudrate);

        if(u32Baud_Div > 0xFFFF)
            uart->BAUD = (UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(u32Clk, u32baudrate));
        else
            uart->BAUD = (UART_BAUD_MODE2 | u32Baud_Div);
    }

    uart->LINE = u32data_width | u32parity | u32stop_bits;
}


/**
 *    @brief    This function use to set Rx timeout count.
 *
 *    @param    uart    The base address of UART module.
 *    @param    u32TOC  Rx timeout counter.
 *
 *    @return   None
 */
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC)
{
    uart->TOUT = (uart->TOUT & ~UART_TOUT_TOIC_Msk)| (u32TOC);
    uart->INTEN |= UART_INTEN_TOCNTEN_Msk;
}


/**
 *    @brief    The function is used to configure IrDA relative settings. It consists of TX or RX mode and baudrate.
 *
 *    @param    uart            The base address of UART module.
 *    @param    u32Buadrate     The baudrate of UART module.
 *    @param    u32Direction    The direction(transmit:1/receive:0) of UART module in IrDA mode.
 *
 *    @return   None
 */
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction)
{
    uart->BAUD = UART_BAUD_MODE0 | UART_BAUD_MODE0_DIVIDER(12000000, 57600);

    uart->IRDA    &= ~UART_IRDA_TXINV_Msk;
    uart->IRDA    |=  UART_IRDA_RXINV_Msk;
    uart->IRDA     =  u32Direction ? uart->IRDA | UART_IRDA_TXEN_Msk : uart->IRDA &~ UART_IRDA_TXEN_Msk;
    uart->FUNCSEL  =  UART_FUNCSEL_IrDA;
}

/**
 *    @brief    The function is to write data into TX buffer to transmit data by UART.
 *
 *    @param    uart            The base address of UART module.
 *    @param    pu8TxBuf        The buffer to send the data to UART transmission FIFO.
 *    @param    u32WriteBytes   The byte number of data.
 *
 *    @return   u32Count: transfer byte count
 */
uint32_t UART_Write(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    uint32_t  u32Count;

    for(u32Count=0; u32Count != u32WriteBytes; u32Count++) 
	{
        if(uart->FIFOSTS & UART_FIFOSTS_TXFULL_Msk) /* Wait Tx empty and Time-out manner */
            return u32Count;

        uart->DAT = pu8TxBuf[u32Count];             /* Send UART Data from buffer */
    }

    return u32Count;
}


/*@}*/ /* end of group I91200_UART_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_UART_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
