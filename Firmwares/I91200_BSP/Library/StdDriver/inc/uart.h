/**************************************************************************//**
 * @file     uart.h
 * @version  V1.00
 * $Revision: 2 $
 * $Date: 16/08/01 2:50p $
 * @brief    I91200 Series UART driver header file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __UART_H__
#define __UART_H__

#ifdef __cplusplus
extern "C"
{
#endif

/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_UART_Driver UART Driver
  @{
*/

/** @addtogroup I91200_UART_EXPORTED_CONSTANTS UART Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/* UART_FIFO constants definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#define UART_FIFO_RFITL_1BYTE        (0x0 << UART_FIFO_RFITL_Pos)   /*!< UART_FIFO setting to set RX FIFO Trigger Level to 1 bit */
#define UART_FIFO_RFITL_4BYTES       (0x1 << UART_FIFO_RFITL_Pos)   /*!< UART_FIFO setting to set RX FIFO Trigger Level to 4 bits */
#define UART_FIFO_RFITL_8BYTES       (0x2 << UART_FIFO_RFITL_Pos)   /*!< UART_FIFO setting to set RX FIFO Trigger Level to 8 bits */
#define UART_FIFO_RFITL_14BYTES      (0x3 << UART_FIFO_RFITL_Pos)   /*!< UART_FIFO setting to set RX FIFO Trigger Level to 14 bits */

#define UART_FIFO_RTSTRGLV_1BYTE     (0x0 << UART_FIFO_RTSTRGLV_Pos)  /*!< UART_FIFO setting to set RTS Trigger Level to 1 bit */
#define UART_FIFO_RTSTRGLV_4BYTES    (0x1 << UART_FIFO_RTSTRGLV_Pos)  /*!< UART_FIFO setting to set RTS Trigger Level to 4 bits */
#define UART_FIFO_RTSTRGLV_8BYTES    (0x2 << UART_FIFO_RTSTRGLV_Pos)  /*!< UART_FIFO setting to set RTS Trigger Level to 8 bits */
#define UART_FIFO_RTSTRGLV_14BYTES   (0x3 << UART_FIFO_RTSTRGLV_Pos)  /*!< UART_FIFO setting to set RTS Trigger Level to 14 bits */

/*---------------------------------------------------------------------------------------------------------*/
/* UART_LINE constants definitions                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define UART_WORD_LEN_5     (0) /*!< UART_LINE setting to set UART word length to 5 bits */
#define UART_WORD_LEN_6     (1) /*!< UART_LINE setting to set UART word length to 6 bits */
#define UART_WORD_LEN_7     (2) /*!< UART_LINE setting to set UART word length to 7 bits */
#define UART_WORD_LEN_8     (3) /*!< UART_LINE setting to set UART word length to 8 bits */

#define UART_PARITY_NONE    (0x0 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to set UART as no parity   */
#define UART_PARITY_ODD     (0x1 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to set UART as odd parity  */
#define UART_PARITY_EVEN    (0x3 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to set UART as even parity */
#define UART_PARITY_MARK    (0x5 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to keep parity bit as '1'  */
#define UART_PARITY_SPACE   (0x7 << UART_LINE_PBE_Pos) /*!< UART_LINE setting to keep parity bit as '0'  */

#define UART_STOP_BIT_1     (0x0 << UART_LINE_NSB_Pos) /*!< UART_LINE setting for one stop bit  */
#define UART_STOP_BIT_1_5   (0x1 << UART_LINE_NSB_Pos) /*!< UART_LINE setting for 1.5 stop bit when 5-bit word length  */
#define UART_STOP_BIT_2     (0x1 << UART_LINE_NSB_Pos) /*!< UART_LINE setting for two stop bit when 6, 7, 8-bit word length */

/*---------------------------------------------------------------------------------------------------------*/
/* UART RTS LEVEL TRIGGER constants definitions                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#define UART_RTS_IS_HIGH_LEV_TRG    (0x1 << UART_MODEM_RTSACTLV_Pos) /*!< Set RTS is High Level Trigger   */
#define UART_RTS_IS_LOW_LEV_TRG     (0x0 << UART_MODEM_RTSACTLV_Pos) /*!< Set RTS is Low Level Trigger    */

/*---------------------------------------------------------------------------------------------------------*/
/* UART_FUNCSEL constants definitions                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define UART_FUNCSEL_UART  ((0x0 << UART_FUNCSEL_LINEN_Pos)|(0x0 << UART_FUNCSEL_IRDAEN_Pos))  /*!< UART_FUNCSEL setting to set UART Function  (Default) */
#define UART_FUNCSEL_LINE  (0x1 << UART_FUNCSEL_LINEN_Pos)                                     /*!< UART_FUNCSEL setting to set Line Function           */
#define UART_FUNCSEL_IrDA  (0x1 << UART_FUNCSEL_IRDAEN_Pos)                                    /*!< UART_FUNCSEL setting to set IrDA Function            */

/*---------------------------------------------------------------------------------------------------------*/
/* UART BAUDRATE MODE constants definitions                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
#define UART_BAUD_MODE0     (0) /*!< Set UART Baudrate Mode is Mode0 */
#define UART_BAUD_MODE2     (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk) /*!< Set UART Baudrate Mode is Mode2 */



/*@}*/ /* end of group I91200_UART_EXPORTED_CONSTANTS */


/** @addtogroup I91200_UART_EXPORTED_FUNCTIONS UART Exported Functions
  @{
*/


/**
 *    @brief   Calculate UART baudrate mode0 divider
 *
 *    @param   u32SrcFreq      UART clock frequency
 *    @param   u32BaudRate     Baudrate of UART module
 *
 *    @return  UART baudrate mode0 divider
 *  \hideinitializer 
 *
 */
#define UART_BAUD_MODE0_DIVIDER(u32SrcFreq, u32BaudRate)    (((u32SrcFreq + (u32BaudRate*8)) / u32BaudRate >> 4)-2)

/**
 *    @brief   Calculate UART baudrate mode2 divider
 *
 *    @param   u32SrcFreq     UART clock frequency
 *    @param   u32BaudRate    Baudrate of UART module
 *
 *    @return  UART baudrate mode2 divider
 * \hideinitializer 
 */
#define UART_BAUD_MODE2_DIVIDER(u32SrcFreq, u32BaudRate)    (((u32SrcFreq + (u32BaudRate/2)) / u32BaudRate)-2)


/**
 *    @brief   Write Data to Tx data register
 *
 *    @param   uart    The base address of UART module.
 *    @param   u8Data  Data byte to transmit
 *
 *    @return  None
 * \hideinitializer 
 */
#define UART_WRITE(uart, u8Data)    (uart->DAT = (u8Data))

/**
 *    @brief   Read Rx data register
 *
 *    @param   uart   The base address of UART module.
 *
 *    @return  The oldest data byte in RX FIFO
 * \hideinitializer 
 */
#define UART_READ(uart)    (uart->DAT)


/**
 *    @brief    Get Tx empty register value.
 *
 *    @param    uart    The base address of UART module
 *
 *    @return   Tx empty register value.
 * \hideinitializer 
 */
#define UART_GET_TX_EMPTY(uart)    (uart->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk)


/**
 *    @brief    Get Rx empty register value.
 *
 *    @param    uart    The base address of UART module
 *
 *    @return   Rx empty register value.
 * \hideinitializer 
 */
#define UART_GET_RX_EMPTY(uart)    (uart->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk)

/**
 *    @brief    Check specified uart port transmission is over.
 *
 *    @param    uart    The base address of UART module
 *
 *    @return   TE_Flag.
 * \hideinitializer 
 */
#define UART_IS_TX_EMPTY(uart)    ((uart->FIFOSTS & UART_FIFOSTS_TXEMPTYF_Msk) >> UART_FIFOSTS_TXEMPTYF_Pos)


/**
 *    @brief    Wait specified uart port transmission is over
 *
 *    @param    uart    The base address of UART module
 *
 *    @return   None
 *  \hideinitializer 
 */
#define UART_WAIT_TX_EMPTY(uart)    while(!(((uart->FIFOSTS) & UART_FIFOSTS_TXEMPTYF_Msk) >> UART_FIFOSTS_TXEMPTYF_Pos))

/**
 *    @brief    Check RDA_IF is set or not
 *
 *    @param    uart    The base address of UART module
 *
 *    @return
 *            0 : The number of bytes in the RX FIFO is less than the RFITL
 *            1 : The number of bytes in the RX FIFO equals or larger than RFITL
 * \hideinitializer 
 */
#define UART_IS_RX_READY(uart)    ((uart->INTSTS & UART_INTSTS_RDAIF_Msk)>>UART_INTSTS_RDAIF_Pos)


/**
 *    @brief    Check TX FIFO is full or not
 *
 *    @param    uart    The base address of UART module
 *
 *    @return
 *            1 = TX FIFO is full
 *            0 = TX FIFO is not full
 * \hideinitializer 
 */
#define UART_IS_TX_FULL(uart)    ((uart->FIFOSTS & UART_FIFOSTS_TXFULL_Msk)>>UART_FIFOSTS_TXFULL_Pos)

/**
 *    @brief    Check RX FIFO is full or not
 *
 *    @param    uart    The base address of UART module
 *
 *    @return
 *            1 = RX FIFO is full
 *            0 = RX FIFO is not full
 * \hideinitializer 
 *
 */
#define UART_IS_RX_FULL(uart)    ((uart->FIFOSTS & UART_FIFOSTS_RXFULL_Msk)>>UART_FIFOSTS_RXFULL_Pos)


/**
 *    @brief    Get Tx full register value
 *
 *    @param    uart    The base address of UART module
 *
 *    @return   Tx full register value
 * \hideinitializer 
 */
#define UART_GET_TX_FULL(uart)    (uart->FIFOSTS & UART_FIFOSTS_TXFULL_Msk)


/**
 *    @brief    Get Rx full register value
 *
 *    @param    uart    The base address of UART module
 *
 *    @return   Rx full register value
 * \hideinitializer 
 */
#define UART_GET_RX_FULL(uart)    (uart->FIFOSTS & UART_FIFOSTS_RXFULL_Msk)


/**
 *    @brief    Enable specified interrupt
 *
 *    @param    uart          The base address of UART module
 *    @param    u32eIntSel    Interrupt type select
 *                               - UART_INTEN_TOCNTEN_Msk    : Rx Time Out interrupt
 *                               - UART_INTEN_WKCTSIEN_Msk   : Wakeup interrupt
 *                               - UART_INTEN_BUFERRIEN_Msk  : Buffer Error interrupt
 *                               - UART_INTEN_RXTOIEN_Msk    : Rx time-out interrupt
 *                               - UART_INTEN_MODEMIEN_Msk   : Modem interrupt
 *                               - UART_INTEN_RLSIEN_Msk     : Rx Line status interrupt
 *                               - UART_INTEN_THREIEN_Msk    : Tx empty interrupt
 *                               - UART_INTEN_RDAIEN_Msk     : Rx ready interrupt
 *
 *    @return    None
 * \hideinitializer 
 */
#define UART_ENABLE_INT(uart, u32eIntSel)    (uart->INTEN |= (u32eIntSel))


/**
 *    @brief    Disable specified interrupt
 *
 *    @param    uart         The base address of UART module
 *    @param    u32eIntSel   Interrupt type select
 *                               - UART_INTEN_TOCNTEN_Msk    : Rx Time Out interrupt
 *                               - UART_INTEN_WKCTSIEN_Msk   : Wakeup interrupt
 *                               - UART_INTEN_BUFERRIEN_Msk  : Buffer Error interrupt
 *                               - UART_INTEN_RXTOIEN_Msk    : Rx time-out interrupt
 *                               - UART_INTEN_MODEMIEN_Msk   : Modem interrupt
 *                               - UART_INTEN_RLSIEN_Msk     : Rx Line status interrupt
 *                               - UART_INTEN_THREIEN_Msk    : Tx empty interrupt
 *                               - UART_INTEN_RDAIEN_Msk     : Rx ready interrupt
 *    @return    None
 * \hideinitializer 
 */
#define UART_DISABLE_INT(uart, u32eIntSel)    (uart->INTEN &= ~ (u32eIntSel))


/**
 *    @brief    Get specified interrupt flag/status
 *
 *    @param    uart              The base address of UART module
 *    @param    u32eIntTypeFlag   Interrupt type select
 *                               - UART_INTEN_TOCNTEN_Msk    : Rx Time Out interrupt
 *                               - UART_INTEN_BUFERRIEN_Msk  : Buffer Error interrupt
 *                               - UART_INTEN_RXTOIEN_Msk    : Rx time-out interrupt
 *                               - UART_INTEN_MODEMIEN_Msk   : Modem interrupt
 *                               - UART_INTEN_RLSIEN_Msk     : Rx Line status interrupt
 *                               - UART_INTEN_THREIEN_Msk    : Tx empty interrupt
 *                               - UART_INTEN_RDAIEN_Msk     : Rx ready interrupt
 *
 *    @return
 *            0 = The specified interrupt is not happened.
 *            1 = The specified interrupt is happened.
 * \hideinitializer 
 */
#define UART_GET_INT_FLAG(uart,u32eIntTypeFlag)    ((uart->INTSTS & (u32eIntTypeFlag))?1:0)


/**
 *    @brief    Set RTS pin is low
 *
 *    @param    uart    The base address of UART module
 *    @return   None
 */
__STATIC_INLINE void UART_CLEAR_RTS(UART_T* uart)
{
    uart->MODEM |= UART_MODEM_RTSACTLV_Msk;
    uart->MODEM &= UART_MODEM_RTS_Msk;
}

/**
 *    @brief    Set RTS pin is high
 *
 *    @param    uart    The base address of UART module
 *    @return   None
 */
__STATIC_INLINE void UART_SET_RTS(UART_T* uart)
{
    uart->MODEM |= UART_MODEM_RTSACTLV_Msk | UART_MODEM_RTS_Msk;
}

/**
 *    @brief    Clear RS-485 Address Byte Detection Flag
 *
 *    @param    uart    The base address of UART module
 *    @return   None
 * \hideinitializer 
 */
#define UART_RS485_CLEAR_ADDR_FLAG(uart)    (uart->FIFOSTS  |= UART_FIFOSTS_ADDRDETF_Msk)


/**
 *    @brief    Get RS-485 Address Byte Detection Flag
 *
 *    @param    uart    The base address of UART module
 *    @return   RS-485  Address Byte Detection Flag
 * \hideinitializer 
 */
#define UART_RS485_GET_ADDR_FLAG(uart)    ((uart->FIFOSTS  & UART_FIFOSTS_ADDRDETF_Msk) >> UART_FIFOSTS_ADDRDETF_Pos)


void UART_ClearIntFlag(UART_T* uart , uint32_t u32InterruptFlag);
void UART_Close(UART_T* uart );
void UART_DisableFlowCtrl(UART_T* uart );
void UART_DisableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_EnableFlowCtrl(UART_T* uart );
void UART_EnableInt(UART_T*  uart, uint32_t u32InterruptFlag );
void UART_Open(UART_T* uart, uint32_t u32baudrate);
uint32_t UART_Read(UART_T* uart, uint8_t *pu8RxBuf, uint32_t u32ReadBytes);
void UART_SetLine_Config(UART_T* uart, uint32_t u32baudrate, uint32_t u32data_width, uint32_t u32parity, uint32_t  u32stop_bits);
void UART_SetTimeoutCnt(UART_T* uart, uint32_t u32TOC);
void UART_SelectIrDAMode(UART_T* uart, uint32_t u32Buadrate, uint32_t u32Direction);
uint32_t UART_Write(UART_T* uart,uint8_t *pu8TxBuf, uint32_t u32WriteBytes);


/*@}*/ /* end of group I91200_UART_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_UART_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__UART_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/




