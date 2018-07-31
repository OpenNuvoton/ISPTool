/**************************************************************************//**
 * @file     uart_reg.h
 * @version  V1.00
 * @brief    UART register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __UART_REG_H__
#define __UART_REG_H__


/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */


/*---------------------- Universal Asynchronous Receiver/Transmitter Controller -------------------------*/
/**
    @addtogroup UART Universal Asynchronous Receiver/Transmitter Controller (UART)
    Memory Mapped Structure for UART Controller
@{ */



typedef struct
{

/**
 * @var UART_T::DATA
 * Offset: 0x00  UART Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DATA      |Data Register
 * |        |          |By writing to this register, the UART will send out an 8-bit data through the UART_TXD pin (LSB first).
 * |        |          |By reading this register, the UART will return an 8-bit data received from UART_RXD pin (LSB first).
 * @var UART_T::THR
 * Offset: 0x00  UART Transmit Holding Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |THR       |Transmit Holding Register
 * |        |          |By writing to this register, the UART will send out an 8-bit data through the UART_TXD pin (LSB first).
 * @var UART_T::RBR
 * Offset: 0x00  UART Receive Buffer Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RBR       |Receive Buffer Register (Read Only)
 * |        |          |By reading this register, the UART will return an 8-bit data received from UART_RXD pin (LSB first).
 * @var UART_T::IER
 * Offset: 0x04  UART Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RDA_IEN   |Receive Data Available Interrupt Enable Control
 * |        |          |0 = RDA_INT Masked off.
 * |        |          |1 = RDA_INT Enabled.
 * |[1]     |THRE_IEN  |Transmit Holding Register Empty Interrupt Enable Control
 * |        |          |0 = THRE_INT Masked off.
 * |        |          |1 = THRE_INT Enabled.
 * |[2]     |RLS_IEN   |Receive Line Status Interrupt Enable Control
 * |        |          |0 = RLS_INT Masked off.
 * |        |          |1 = RLS_INT Enabled
 * |[3]     |MODEM_IEN |Modem Status Interrupt Enable Control (Not Available In UART2 Channel)
 * |        |          |0 = MODEM_INT Masked off.
 * |        |          |1 = MODEM_INT Enabled.
 * |[4]     |TOUT_IEN  |RX Time-Out Interrupt Enable Control
 * |        |          |0 = TOUT_INT Masked off.
 * |        |          |1 = TOUT_INT Enabled.
 * |[5]     |BUF_ERR_IEN|Buffer Error Interrupt Enable Control
 * |        |          |0 = BUF_ERR_INT Masked off.
 * |        |          |1 = BUF_ERR_INT Enabled.
 * |[6]     |WAKE_EN   |UART Wake-Up Function Enable (Not Available In UART2 Channel)
 * |        |          |0 = UART wake-up function Disabled.
 * |        |          |1 = UART wake-up function Enabled, when the chip is in Power-down mode, an external CTS change will wake-up chip from Power-down mode.
 * |[8]     |LIN_IEN   |LIN Bus Interrupt Enable
 * |        |          |0 = Lin bus interrupt Disabled.
 * |        |          |1 = Lin bus interrupt Enabled.
 * |        |          |Note: This field is used for LIN function mode.
 * |[11]    |TIME_OUT_EN|Time-Out Counter Enable
 * |        |          |0 = Time-out counter Disabled.
 * |        |          |1 = Time-out counter Enabled.
 * |[12]    |AUTO_RTS_EN|RTS Auto Flow Control Enable (Not Available in UART2 Channel)
 * |        |          |0 = RTS auto flow control Disabled.
 * |        |          |1 = RTS auto flow control Enabled.
 * |        |          |When RTS auto-flow is enabled, if the number of bytes in the RX FIFO equals the RTS_TRI_LEV
 * |        |          |(UA_FCR [19:16]), the UART will de-assert RTS signal.
 * |[13]    |AUTO_CTS_EN|CTS Auto Flow Control Enable (Not Available in UART2 Channel)
 * |        |          |0 = CTS auto flow control Disabled.
 * |        |          |1 = CTS auto flow control Enabled.
 * |        |          |When CTS auto-flow is enabled, the UART will send data to external device when CTS input assert (UART will not send data to device until CTS is asserted).
 * |[14]    |DMA_TX_EN |TX DMA Enable (Not Available In UART2 Channel)
 * |        |          |This bit can enable or disable TX DMA service.
 * |        |          |0 = TX DMA Disabled.
 * |        |          |1 = TX DMA Enabled.
 * |[15]    |DMA_RX_EN |RX DMA Enable (Not Available In UART2 Channel)
 * |        |          |This bit can enable or disable RX DMA service.
 * |        |          |0 = RX DMA Disabled.
 * |        |          |1 = RX DMA Enabled.
 * @var UART_T::FCR
 * Offset: 0x08  UART FIFO Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |RFR       |RX Field Software Reset
 * |        |          |When RFR is set, all the byte in the receiver FIFO and RX internal state machine are cleared.
 * |        |          |0 = No effect.
 * |        |          |1 = Reset the RX internal state machine and pointers.
 * |        |          |Note: This bit will automatically clear at least 3 UART peripheral clock cycles.
 * |[2]     |TFR       |TX Field Software Reset
 * |        |          |When TFR is set, all the byte in the transmit FIFO and TX internal state machine are cleared.
 * |        |          |0 = No effect.
 * |        |          |1 = Reset the TX internal state machine and pointers.
 * |        |          |Note: This bit will automatically clear at least 3 UART peripheral clock cycles.
 * |[7:4]   |RFITL     |RX FIFO Interrupt Trigger Level
 * |        |          |When the number of bytes in the receive FIFO equals the RFITL, the RDA_IF will be set (if RDA_IEN (UA_IER[0]) enabled, and an interrupt will be generated).
 * |        |          |0000 = RX FIFO Interrupt Trigger Level is 1 byte.
 * |        |          |0001 = RX FIFO Interrupt Trigger Level is 4 bytes.
 * |        |          |0010 = RX FIFO Interrupt Trigger Level is 8 bytes.
 * |        |          |0011 = RX FIFO Interrupt Trigger Level is 14 bytes.
 * |        |          |0100 = RX FIFO Interrupt Trigger Level is 30/14 bytes (High Speed/Normal Speed).
 * |        |          |0101 = RX FIFO Interrupt Trigger Level is 46/14 bytes (High Speed/Normal Speed).
 * |        |          |0110 = RX FIFO Interrupt Trigger Level is 62/14 bytes (High Speed/Normal Speed).
 * |        |          |Other = Reserved.
 * |[8]     |RX_DIS    |Receiver Disable Register
 * |        |          |The receiver is disabled or not (set 1 to disable receiver).
 * |        |          |0 = Receiver Enabled.
 * |        |          |1 = Receiver Disabled.
 * |        |          |Note: This field is used for RS-485 Normal Multi-drop mode. It should be programmed before RS485_NMM (UA_ALT_CSR[8]) is programmed.
 * |[19:16] |RTS_TRI_LEV|RTS Trigger Level For Auto-Flow Control Use (Not Available In UART2 Channel)
 * |        |          |0000 = RTS Trigger Level is 1 byte.
 * |        |          |0001 = RTS Trigger Level is 4 bytes.
 * |        |          |0010 = RTS Trigger Level is 8 bytes.
 * |        |          |0011 = RTS Trigger Level is 14 bytes.
 * |        |          |0100 = RTS Trigger Level is 30/14 bytes (High Speed/Normal Speed).
 * |        |          |0101 = RTS Trigger Level is 46/14 bytes (High Speed/Normal Speed).
 * |        |          |0110 = RTS Trigger Level is 62/14 bytes (High Speed/Normal Speed).
 * |        |          |Other = Reserved.
 * |        |          |Note: This field is used for RTS auto-flow control.
 * @var UART_T::LCR
 * Offset: 0x0C  UART Line Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |WLS       |Word Length Selection
 * |        |          |00 = Word length is 5-bit.
 * |        |          |01 = Word length is 6-bit.
 * |        |          |10 = Word length is 7-bit
 * |        |          |11 = Word length is 8-bit
 * |[2]     |NSB       |Number Of "STOP Bit"
 * |        |          |0 = One " STOP bit" is generated in the transmitted data.
 * |        |          |1 = When select 5-bit word length, 1.5 "STOP bit" is generated in the transmitted data.
 * |        |          |When select 6-,7- and 8-bit word length, 2 "STOP bit" is generated in the transmitted data.
 * |[3]     |PBE       |Parity Bit Enable
 * |        |          |0 = No parity bit.
 * |        |          |1 = Parity bit is generated on each outgoing character and is checked on each incoming data.
 * |[4]     |EPE       |Even Parity Enable
 * |        |          |0 = Odd number of logic 1's is transmitted and checked in each word.
 * |        |          |1 = Even number of logic 1's is transmitted and checked in each word.
 * |        |          |This bit has effect only when PBE (UA_LCR[3]) is set.
 * |[5]     |SPE       |Stick Parity Enable
 * |        |          |0 = Stick parity Disabled.
 * |        |          |1 = If PBE (UA_LCR[3]) and EBE (UA_LCR[4]) are logic 1, the parity bit is transmitted and
 * |        |          |checked as logic 0.
 * |        |          |If PBE (UA_LCR[3]) is 1 and EBE (UA_LCR[4]) is 0 then the parity bit is transmitted and checked as 1.
 * |[6]     |BCB       |Break Control Bit
 * |        |          |When this bit is set to logic 1, the serial data output (TX) is forced to the Spacing State (logic 0).
 * |        |          |This bit acts only on TX and has no effect on the transmitter logic.
 * @var UART_T::MCR
 * Offset: 0x10  UART Modem Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |RTS       |RTS (Request-To-Send) Signal Control (Not Available In UART2 Channel)
 * |        |          |This bit is direct control internal RTS signal active or not, and then drive the RTS pin output with LEV_RTS bit configuration.
 * |        |          |0 = RTS signal is active.
 * |        |          |1 = RTS signal is inactive.
 * |        |          |Note1: This RTS signal control bit is not effective when RTS auto-flow control is enabled in UART function mode.
 * |        |          |Note2: This RTS signal control bit is not effective when RS-485 auto direction mode (AUD) is enabled in RS-485 function mode.
 * |[9]     |LEV_RTS   |RTS Pin Active Level (Not Available In UART2 Channel)
 * |        |          |This bit defines the active level state of RTS pin output.
 * |        |          |0 = RTS pin output is high level active.
 * |        |          |1 = RTS pin output is low level active.
 * |[13]    |RTS_ST    |RTS Pin State (Read Only) (Not Available In UART2 Channel)
 * |        |          |This bit mirror from RTS pin output of voltage logic status.
 * |        |          |0 = RTS pin output is low level voltage logic state.
 * |        |          |1 = RTS pin output is high level voltage logic state.
 * @var UART_T::MSR
 * Offset: 0x14  UART Modem Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |DCTSF     |Detect CTS State Change Flag (Not Available In UART2 Channel)
 * |        |          |This bit is set whenever CTS input has change state, and it will generate Modem interrupt to CPU when MODEM_IEN (UA_IER [3]) is set to 1.
 * |        |          |0 = CTS input has not change state.
 * |        |          |1 = CTS input has change state.
 * |        |          |Note: This bit can be cleared by writing "1" to it.
 * |[4]     |CTS_ST    |CTS Pin Status (Read Only) (Not Available In UART2 Channel)
 * |        |          |This bit mirror from CTS pin input of voltage logic status.
 * |        |          |0 = CTS pin input is low level voltage logic state.
 * |        |          |1 = CTS pin input is high level voltage logic state.
 * |        |          |Note: This bit echoes when UART Controller peripheral clock is enabled, and CTS multi-function port is selected.
 * |[8]     |LEV_CTS   |CTS Pin Active Level
 * |        |          |This bit defines the active level state of CTS pin input.
 * |        |          |0 = CTS pin input is high level active.
 * |        |          |1 = CTS pin input is low level active.
 * @var UART_T::FSR
 * Offset: 0x18  UART FIFO Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RX_OVER_IF|RX Overflow Error Interrupt Flag
 * |        |          |This bit is set when RX FIFO overflow.
 * |        |          |If the number of bytes of received data is greater than RX_FIFO (UA_RBR) size, 64/16/16 bytes of UART0/UART1/UART2, this bit will be set.
 * |        |          |0 = RX FIFO is not overflow.
 * |        |          |1 = RX FIFO is overflow.
 * |        |          |Note: This bit can be cleared by writing "1" to it.
 * |[3]     |RS485_ADD_DETF|RS-485 Address Byte Detection Flag 
 * |        |          |0 = Receiver detects a data that is not an address bit (bit 9 ='1').
 * |        |          |1 = Receiver detects a data that is an address bit (bit 9 ='1').
 * |        |          |Note1: This field is used for RS-485 function mode and RS485_ADD_EN (UA_ALT_CSR[15]) is set to 1 to enable Address detection mode.
 * |        |          |Note2: This bit can be cleared by writing '1' to it.
 * |[4]     |PEF       |Parity Error Flag 
 * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "parity bit", and is reset whenever the CPU writes 1 to this bit.
 * |        |          |0 = No parity error is generated.
 * |        |          |1 = Parity error is generated.
 * |        |          |Note: This bit can be cleared by writing "1" to it.
 * |[5]     |FEF       |Framing Error Flag
 * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "stop bit"
 * |        |          |(that is, the stop bit following the last data bit or parity bit is detected as logic 0), and is
 * |        |          |reset whenever the CPU writes 1 to this bit.
 * |        |          |0 = No framing error is generated.
 * |        |          |1 = Framing error is generated.
 * |        |          |Note: This bit can be cleared by writing "1" to it.
 * |[6]     |BIF       |Break Interrupt Flag
 * |        |          |This bit is set to logic 1 whenever the received data input(RX) is held in the "spacing state"
 * |        |          |(logic 0) for longer than a full word transmission time (that is, the total time of "start bit"
 * |        |          |+ data bits + parity + stop bits) and is reset whenever the CPU writes 1 to this bit.
 * |        |          |0 = No Break interrupt is generated.
 * |        |          |1 = Break interrupt is generated.
 * |        |          |Note: This bit can be cleared by writing "1" to it.
 * |[13:8]  |RX_POINTER|RX FIFO Pointer (Read Only)
 * |        |          |This field indicates the RX FIFO Buffer Pointer.
 * |        |          |When UART receives one byte from external device, then RX_POINTER increases one.
 * |        |          |When one byte of RX FIFO is read by CPU, then RX_POINTER decreases one.
 * |        |          |The Maximum value shown in RX_POINTER is 63/15/15 (UART0/UART1/UART2).
 * |        |          |When the using level of RX FIFO Buffer equal to 64/16/16, the RX_FULL bit is set to 1 and RX_POINTER will show 0.
 * |        |          |As one byte of RX FIFO is read by CPU, the RX_FULL bit is cleared to 0 and RX_POINTER will show
 * |        |          |63/15/15 (UART0/UART1/UART2).
 * |[14]    |RX_EMPTY  |Receiver FIFO Empty (Read Only)
 * |        |          |This bit initiate RX FIFO empty or not.
 * |        |          |0 = RX FIFO is not empty.
 * |        |          |1 = RX FIFO is empty.
 * |        |          |Note: When the last byte of RX FIFO has been read by CPU, hardware sets this bit high. It will be cleared when UART receives any new data.
 * |[15]    |RX_FULL   |Receiver FIFO Full (Read Only)
 * |        |          |This bit initiates RX FIFO is full or not.
 * |        |          |0 = RX FIFO is not full.
 * |        |          |1 = RX FIFO is full.
 * |        |          |Note: This bit is set when the number of usage in RX FIFO Buffer is equal to 64/16/16(UART0/UART1/UART2), otherwise is cleared by hardware.
 * |[21:16] |TX_POINTER|TX FIFO Pointer (Read Only)
 * |        |          |This field indicates the TX FIFO Buffer Pointer.
 * |        |          |When CPU writes one byte into UA_THR, then TX_POINTER increases one.
 * |        |          |When one byte of TX FIFO is transferred to Transmitter Shift Register, then TX_POINTER decreases one.
 * |        |          |The Maximum value shown in TX_POINTER is 63/15/15 (UART0/UART1/UART2).
 * |        |          |When the using level of TX FIFO Buffer equal to 64/16/16, the TX_FULL bit is set to 1 and TX_POINTER will show 0.
 * |        |          |As one byte of TX FIFO is transferred to Transmitter Shift Register, the TX_FULL bit is cleared to 0 and TX_POINTER will show 63/15/15 (UART0/UART1/UART2).
 * |[22]    |TX_EMPTY  |Transmitter FIFO Empty (Read Only)
 * |        |          |This bit indicates TX FIFO empty or not.
 * |        |          |0 = TX FIFO is not empty.
 * |        |          |1 = TX FIFO is empty.
 * |        |          |Note: When the last byte of TX FIFO has been transferred to Transmitter Shift Register, hardware sets this bit high. It will be cleared when writing data into THR (TX FIFO not empty).
 * |[23]    |TX_FULL   |Transmitter FIFO Full (Read Only)
 * |        |          |This bit indicates TX FIFO full or not.
 * |        |          |0 = TX FIFO is not full.
 * |        |          |1 = TX FIFO is full.
 * |        |          |This bit is set when the number of usage in TX FIFO Buffer is equal to
 * |        |          |64/16/16(UART0/UART1/UART2), otherwise is cleared by hardware.
 * |[24]    |TX_OVER_IF|TX Overflow Error Interrupt Flag 
 * |        |          |If TX FIFO (UA_THR) is full, an additional write to UA_THR will cause this bit to logic 1.
 * |        |          |0 = TX FIFO is not overflow.
 * |        |          |1 = TX FIFO is overflow.
 * |        |          |Note: This bit can be cleared by writing "1" to it.
 * |[28]    |TE_FLAG   |Transmitter Empty Flag (Read Only)
 * |        |          |This bit is set by hardware when TX FIFO (UA_THR) is empty and the STOP bit of the last byte has been transmitted.
 * |        |          |0 = TX FIFO is not empty.
 * |        |          |1 = TX FIFO is empty.
 * |        |          |Note: This bit is cleared automatically when TX FIFO is not empty or the last byte transmission has not completed.
 * @var UART_T::ISR
 * Offset: 0x1C  UART Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RDA_IF    |Receive Data Available Interrupt Flag (Read Only)
 * |        |          |When the number of bytes in the RX FIFO equals the RFITL then the RDA_IF(UA_ISR[0]) will be set.
 * |        |          |If RDA_IEN (UA_IER [0]) is enabled, the RDA interrupt will be generated.
 * |        |          |0 = No RDA interrupt flag is generated.
 * |        |          |1 = RDA interrupt flag is generated.
 * |        |          |Note: This bit is read only and it will be cleared when the number of unread bytes of RX FIFO drops below the threshold level RFITL(UA_FCR[7:4]).
 * |[1]     |THRE_IF   |Transmit Holding Register Empty Interrupt Flag (Read Only)
 * |        |          |This bit is set when the last data of TX FIFO is transferred to Transmitter Shift Register.
 * |        |          |If THRE_IEN (UA_IER[1]) is enabled, the THRE interrupt will be generated.
 * |        |          |0 = No THRE interrupt flag is generated.
 * |        |          |1 = THRE interrupt flag is generated.
 * |        |          |Note: This bit is read only and it will be cleared when writing data into THR (TX FIFO not empty).
 * |[2]     |RLS_IF    |Receive Line Interrupt Flag
 * |        |          |This bit is set when the RX receive data have parity error, frame error or break error (at least one of 3 bits, BIF(UA_FSR[6]), FEF(UA_FSR[5]) and PEF(UA_FSR[4]), is set).
 * |        |          |If RLS_IEN (UA_IER [2]) is enabled, the RLS interrupt will be generated.
 * |        |          |0 = No RLS interrupt flag is generated.
 * |        |          |1 = RLS interrupt flag is generated.
 * |        |          |Note1: In RS-485 function mode, this field is set include receiver detect and received address byte character (bit9 = '1') bit. 
 * |        |          |At the same time, the bit of RS485_ADD_DETF(UA_FSR[3]) is also set.
 * |        |          |Note2: This bit is read only and reset to 0 when all bits of BIF(UA_FSR[6]), FEF(UA_FSR[5]) and PEF(UA_FSR[4]) are cleared.
 * |        |          |Note3: In RS-485 function mode, this bit is read only and reset to 0 when all bits of BIF(UA_FSR[6]) , FEF(UA_FSR[5]) and PEF(UA_FSR[4]) and RS485_ADD_DETF (UA_FSR[3]) are cleared.
 * |[3]     |MODEM_IF  |MODEM Interrupt Flag (Read Only) (Not Available In UART2 Channel)
 * |        |          |This bit is set when the CTS pin has state change (DCTSF (UA_MSR[0]) = 1).
 * |        |          |If MODEM_IEN (UA_IER [3]) is enabled, the Modem interrupt will be generated.
 * |        |          |0 = No Modem interrupt flag is generated.
 * |        |          |1 = Modem interrupt flag is generated.
 * |        |          |Note: This bit is read only and reset to 0 when bit DCTSF is cleared by a write 1 on DCTSF(UA_MSR[0]).
 * |[4]     |TOUT_IF   |Time-out Interrupt Flag (Read Only)
 * |        |          |This bit is set when the RX FIFO is not empty and no activities occurred in the RX FIFO and the time-out counter equal to TOIC(UA_TOR[7:0]).
 * |        |          |If TOUT_IEN (UA_IER [4]) is enabled, the Time-out interrupt will be generated.
 * |        |          |0 = No Time-out interrupt flag is generated.
 * |        |          |1 = Time-out interrupt flag is generated.
 * |        |          |Note: This bit is read only and user can read UA_RBR (RX is in active) to clear it.
 * |[5]     |BUF_ERR_IF|Buffer Error Interrupt Flag (Read Only)
 * |        |          |This bit is set when the TX FIFO or RX FIFO overflows (TX_OVER_IF (UA_FSR[24]) or RX_OVER_IF
 * |        |          |(UA_FSR[0]) is set).
 * |        |          |When BUF_ERR_IF (UA_ISR[5])is set, the transfer is not correct.
 * |        |          |If BUF_ERR_IEN (UA_IER [8]) is enabled, the buffer error interrupt will be generated.
 * |        |          |0 = No buffer error interrupt flag is generated.
 * |        |          |1 = Buffer error interrupt flag is generated.
 * |        |          |Note: This bit is read only and reset to 0 when all bits of TX_OVER_IF(UA_FSR[24]) and RX_OVER_IF(UA_FSR[0]) are cleared.
 * |[7]     |LIN_IF    |LIN Bus Flag (Read Only)
 * |        |          |This bit is set when LIN slave header detect (LINS_HDET_F (UA_LIN_SR[0] = 1)), LIN break detect (LIN_BKDET_F(UA_LIN_SR[9]=1)), 
 * |        |          |bit error detect (BIT_ERR_F(UA_LIN_SR[9])=1), LIN slave ID parity error (LINS_IDPERR_F(UA_LIN_SR[2]) = 1) or LIN slave header error detect (LINS_HERR_F (UA_LIN_SR[1])).
 * |        |          |If LIN_ IEN (UA_IER [8]) is enabled the LIN interrupt will be generated.
 * |        |          |0 = None of LINS_HDET_F, LIN_BKDET_F, BIT_ERR_F, LINS_IDPERR_F and LINS_HERR_F is generated.
 * |        |          |1 = At least one of LINS_HDET_F, LIN_BKDET_F, BIT_ERR_F, LINS_IDPERR_F and LINS_HERR_F is generated.
 * |        |          |Note: This bit is read only. This bit is cleared when LINS_HDET_F(UA_LIN_SR[0]),
 * |        |          |LIN_BKDET_F(UA_LIN_SR[9]), BIT_ERR_F(UA_LIN_SR[9]), LINS_IDPENR_F (UA_LIN_SR[2]) and LINS_HERR_F(UA_LIN_SR[1]) all are cleared.
 * |[8]     |RDA_INT   |Receive Data Available Interrupt Indicator (Read Only)
 * |        |          |This bit is set if RDA_IEN (UA_IER[0]) and RDA_IF (UA_ISR[0]) are both set to 1.
 * |        |          |0 = No RDA interrupt is generated.
 * |        |          |1 = RDA interrupt is generated.
 * |[9]     |THRE_INT  |Transmit Holding Register Empty Interrupt Indicator (Read Only)
 * |        |          |This bit is set if THRE_IEN (UA_IER[1])and THRE_IF(UA_SR[1]) are both set to 1.
 * |        |          |0 = No THRE interrupt is generated.
 * |        |          |1 = THRE interrupt is generated.
 * |[10]    |RLS_INT   |Receive Line Status Interrupt Indicator (Read Only)
 * |        |          |This bit is set if RLS_IEN (UA_IER[2]) and RLS_IF(UA_ISR[2]) are both set to 1.
 * |        |          |0 = No RLS interrupt is generated.
 * |        |          |1 = RLS interrupt is generated
 * |[11]    |MODEM_INT |MODEM Status Interrupt Indicator (Read Only) (Not Available In UART2 Channel)
 * |        |          |This bit is set if MODEM_IEN(UA_IER[3]) and MODEM_IF(UA_ISR[4]) are both set to 1
 * |        |          |0 = No Modem interrupt is generated.
 * |        |          |1 = Modem interrupt is generated.
 * |[12]    |TOUT_INT  |Time-Out Interrupt Indicator (Read Only)
 * |        |          |This bit is set if TOUT_IEN(UA_IER[4]) and TOUT_IF(UA_ISR[4]) are both set to 1.
 * |        |          |0 = No Time-Out interrupt is generated.
 * |        |          |1 = Time-Out interrupt is generated.
 * |[13]    |BUF_ERR_INT|Buffer Error Interrupt Indicator (Read Only)
 * |        |          |This bit is set if BUF_ERR_IEN(UA_IER[5]) and BUF_ERR_IF(UA_ISR[5]) are both set to 1.
 * |        |          |0 = No buffer error interrupt is generated.
 * |        |          |1 = Buffer error interrupt is generated.
 * |[15]    |LIN_INT   |LIN Bus Interrupt Indicator (Read Only)
 * |        |          |This bit is set if LIN_IEN (UA_IER[8]) and LIN _IF(UA_ISR[7]) are both set to 1.
 * |        |          |0 = No LIN Bus interrupt is generated.
 * |        |          |1 = The LIN Bus interrupt is generated.
 * |[18]    |HW_RLS_IF |In DMA Mode, Receive Line Status Flag (Read Only)
 * |        |          |This bit is set when the RX receive data have parity error, frame error or break error (at least
 * |        |          |one of 3 bits, BIF (UA_FSR[6]), FEF (UA_FSR[5]) and PEF (UA_FSR[4]) is set).
 * |        |          |If RLS_IEN (UA_IER [2]) is enabled, the RLS interrupt will be generated.
 * |        |          |0 = No RLS interrupt flag is generated in DMA mode.
 * |        |          |1 = RLS interrupt flag is generated in DMA mode.
 * |        |          |Note1: In RS-485 function mode, this field include receiver detect any address byte received address byte character (bit9 = '1') bit.
 * |        |          |Note2: In UART function mode, this bit is read only and reset to 0 when all bits of BIF(UA_FSR[6]) , FEF(UA_FSR[5]) and PEF(UA_FSR[4]) are cleared.
 * |        |          |Note3: In RS-485 function mode, this bit is read only and reset to 0 when all bits of
 * |        |          |BIF(UA_FSR[6]) , FEF(UA_FSR[5]) and PEF(UA_FSR[4]) and RS485_ADD_DETF (UA_FSR[3]) are cleared.
 * |[19]    |HW_MODEM_IF|In DMA Mode, MODEM Interrupt Flag (Read Only) (Not Available In UART2 Channel)
 * |        |          |This bit is set when the CTS pin has state change (DCTSF (US_MSR[0] =1)).
 * |        |          |If MODEM_IEN (UA_IER [3]) is enabled, the Modem interrupt will be generated.
 * |        |          |0 = No Modem interrupt flag is generated in DMA mode.
 * |        |          |1 = Modem interrupt flag is generated in DMA mode.
 * |        |          |Note: This bit is read only and reset to 0 when the bit DCTSF(US_MSR[0]) is cleared by writing 1 on DCTSF (US_MSR[0]).
 * |[20]    |HW_TOUT_IF|In DMA Mode, Time-Out Interrupt Flag (Read Only)
 * |        |          |This bit is set when the RX FIFO is not empty and no activities occurred in the RX FIFO and the time-out counter equal to TOIC (UA_TOR[7:0]).
 * |        |          |If TOUT_IEN (UA_IER [4]) is enabled, the Tout interrupt will be generated.
 * |        |          |0 = No Time-out interrupt flag is generated in DMA mode.
 * |        |          |1 = Time-out interrupt flag is generated in DMA mode.
 * |        |          |Note: This bit is read only and user can read UA_RBR (RX is in active) to clear it.
 * |[21]    |HW_BUF_ERR_IF|In DMA Mode, Buffer Error Interrupt Flag (Read Only)
 * |        |          |This bit is set when the TX or RX FIFO overflows (TX_OVER_IF (UA__FSR[24]) or RX_OVER_IF (UA_FSR[0]) is set).
 * |        |          |When BUF_ERR_IF (UA_ISR[5]) is set, the transfer maybe is not correct.
 * |        |          |If BUF_ERR_IEN (UA_IER [5]) is enabled, the buffer error interrupt will be generated.
 * |        |          |0 = No buffer error interrupt flag is generated in DMA mode.
 * |        |          |1 = Buffer error interrupt flag is generated in DMA mode.
 * |        |          |Note: This bit is cleared when both TX_OVER_IF (UA_FSR[24]]) and RX_OVER_IF (UA_FSR[0]) are
 * |        |          |cleared.
 * |[26]    |HW_RLS_INT|In DMA Mode, Receive Line Status Interrupt Indicator (Read Only)
 * |        |          |This bit is set if RLS_IEN (UA_IER[2])and HW_RLS_IF(UA_ISR[18]) are both set to 1.
 * |        |          |0 = No RLS interrupt is generated in DMA mode.
 * |        |          |1 = RLS interrupt is generated in DMA mode.
 * |[27]    |HW_MODEM_INT|In DMA Mode, MODEM Status Interrupt Indicator (Read Only) (Not Available In UART2 Channel)
 * |        |          |This bit is set if MODEM_IEN(UA_IER[3]) and HW_MODEM_IF(UA_ ISR[3]) are both set to 1.
 * |        |          |0 = No Modem interrupt is generated in DMA mode.
 * |        |          |1 = Modem interrupt is generated in DMA mode.
 * |[28]    |HW_TOUT_INT|In DMA Mode, Time-Out Interrupt Indicator (Read Only)
 * |        |          |This bit is set if TOUT_IEN (UA_IER[4])and HW_TOUT_IF(UA_ISR[20]) are both set to 1.
 * |        |          |0 = No Tout interrupt is generated in DMA mode.
 * |        |          |1 = Tout interrupt is generated in DMA mode.
 * |[29]    |HW_BUF_ERR_INT|In DMA Mode, Buffer Error Interrupt Indicator (Read Only)
 * |        |          |This bit is set if BUF_ERR_IEN (UA_IER[5]) and HW_BUF_ERR_IF (UA_ISR[5])are both set to 1.
 * |        |          |0 = No buffer error interrupt is generated in DMA mode.
 * |        |          |1 = Buffer error interrupt is generated in DMA mode.
 * @var UART_T::TOR
 * Offset: 0x20  UART Time-out Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TOIC      |Time-out Interrupt Comparator
 * |        |          |The time-out counter resets and starts counting (the counting clock = baud rate) whenever the RX
 * |        |          |FIFO receives a new data word.
 * |        |          |Once the content of time-out counter is equal to that of time-out interrupt comparator (TOIC
 * |        |          |(UA_TOR[7:0])), a receiver time-out interrupt is generated if TOUT_IEN (UA_IER [4]) enabled.
 * |        |          |A new incoming data word or RX FIFO empty will clear TOUT_IF(UA_IER[4]).
 * |        |          |In order to avoid receiver time-out interrupt generation immediately during one character is
 * |        |          |being received, TOIC (UA_TOR[7:0]) value should be set between 40 and 255.
 * |        |          |So, for example, if TOIC (UA_TOR[7:0]) is set with 40, the time-out interrupt is generated after
 * |        |          |four characters are not received when 1 stop bit and no parity check is set for UART transfer.
 * |[15:8]  |DLY       |TX Delay Time Value
 * |        |          |This field is used to programming the transfer delay time between the last stop bit and next
 * |        |          |start bit.
 * @var UART_T::BAUD
 * Offset: 0x24  UART Baud Rate Divisor Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |BRD       |Baud Rate Divider
 * |        |          |The field indicates the baud rate divider.
 * |[27:24] |DIVIDER_X |Divider X
 * |        |          |The baud rate divider M = X+1.
 * |[28]    |DIV_X_ONE |Divider X Equal To 1
 * |        |          |0 = Divider M is X+1 (the equation of M = X+1, but DIVIDER_X[27:24] must >= 8).
 * |        |          |1 = Divider M is 1.
 * |[29]    |DIV_X_EN  |Divider X Enable
 * |        |          |The BRD = Baud Rate Divider, and the baud rate equation is
 * |        |          |Baud Rate = Clock / [M * (BRD + 2)]; The default value of M is 16.
 * |        |          |0 = Divider X Disabled (the equation of M = 16).
 * |        |          |1 = Divider X Enabled (the equation of M = X+1, but DIVIDER_X [27:24] must >= 8).
 * |        |          |Note: In IrDA mode, this bit must disable.
 * @var UART_T::IRCR
 * Offset: 0x28  UART IrDA Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |TX_SELECT |IrDA Receiver/Transmitter Selection Enable Control
 * |        |          |0 = IrDA Transmitter Disabled and Receiver Enabled.
 * |        |          |1 = IrDA Transmitter Enabled and Receiver Disabled.
 * |[5]     |INV_TX    |IrDA inverse Transmitting Output Signal Control
 * |        |          |0 = None inverse transmitting signal.
 * |        |          |1 = Inverse transmitting output signal.
 * |[6]     |INV_RX    |IrDA inverse Receive Input Signal Control
 * |        |          |0 = None inverse receiving input signal.
 * |        |          |1 = Inverse receiving input signal.
 * @var UART_T::ALT_CSR
 * Offset: 0x2C  UART Alternate Control/Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |LIN_BKFL  |UART LIN Break Field Length
 * |        |          |This field indicates a 4-bit LIN TX break field count.
 * |        |          |Note1: This break field length is UA_LIN_BKFL + 1.
 * |        |          |Note2: According to LIN spec, the reset value is 0xC (break field length = 13).
 * |[6]     |LIN_RX_EN |LIN RX Enable
 * |        |          |0 = LIN RX mode Disabled.
 * |        |          |1 = LIN RX mode Enabled.
 * |[7]     |LIN_TX_EN |LIN TX Break Mode Enable
 * |        |          |0 = LIN TX Break mode Disabled.
 * |        |          |1 = LIN TX Break mode Enabled.
 * |        |          |Note: When TX break field transfer operation finished, this bit will be cleared automatically.
 * |[8]     |RS485_NMM |RS-485 Normal Multi-Drop Operation Mode (NMM)
 * |        |          |0 = RS-485 Normal Multi-drop Operation mode (NMM) Disabled.
 * |        |          |1 = RS-485 Normal Multi-drop Operation mode (NMM) Enabled.
 * |        |          |Note: It cannot be active with RS-485_AAD operation mode.
 * |[9]     |RS485_AAD |RS-485 Auto Address Detection Operation Mode (AAD)
 * |        |          |0 = RS-485 Auto Address Detection Operation mode (AAD) Disabled.
 * |        |          |1 = RS-485 Auto Address Detection Operation mode (AAD) Enabled.
 * |        |          |Note: It cannot be active with RS-485_NMM operation mode.
 * |[10]    |RS485_AUD |RS-485 Auto Direction Mode (AUD)
 * |        |          |0 = RS-485 Auto Direction Operation mode (AUO) Disabled.
 * |        |          |1 = RS-485 Auto Direction Operation mode (AUO) Enabled.
 * |        |          |Note: It can be active with RS-485_AAD or RS-485_NMM operation mode.
 * |[15]    |RS485_ADD_EN|RS-485 Address Detection Enable
 * |        |          |This bit is used to enable RS-485 Address Detection mode.
 * |        |          |0 = Address detection mode Disabled.
 * |        |          |1 = Address detection mode Enabled.
 * |        |          |Note: This bit is used for RS-485 any operation mode.
 * |[31:24] |ADDR_MATCH|Address Match Value Register
 * |        |          |This field contains the RS-485 address match values.
 * |        |          |Note: This field is used for RS-485 auto address detection mode.
 * @var UART_T::FUN_SEL
 * Offset: 0x30  UART Function Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |FUN_SEL   |Function Select Enable
 * |        |          |00 = UART function Enabled.
 * |        |          |01 = LIN function Enabled.
 * |        |          |10 = IrDA function Enabled.
 * |        |          |11 = RS-485 function Enabled.
 * @var UART_T::LIN_CTL
 * Offset: 0x34  UART LIN Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |LINS_EN   |LIN Slave Mode Enable Control
 * |        |          |0 = LIN slave mode Disabled.
 * |        |          |1 = LIN slave mode Enabled.
 * |[1]     |LINS_HDET_EN|LIN Slave Header Detection Enable Control
 * |        |          |0 = LIN slave header detection Disabled.
 * |        |          |1 = LIN slave header detection Enabled.
 * |        |          |Note1: This bit only valid when in LIN slave mode (LINS_EN (UA_LIN_CTL[0]) = 1).
 * |        |          |Note2: In LIN function mode, when detect header field (break + sync + frame ID), LINS_HDET_F
 * |        |          |(UA_LIN_SR [0]) flag will be asserted. If the LIN_IEN (UA_IER[8]) = 1, an interrupt will begenerated.
 * |[2]     |LINS_ARS_EN|LIN Slave Automatic Resynchronization Mode Enable Control
 * |        |          |0 = LIN automatic resynchronization Disabled.
 * |        |          |1 = LIN automatic resynchronization Enabled.
 * |        |          |Note1: This bit only valid when in LIN slave mode (LINS_EN (UA_LIN_CTL[0]) = 1).
 * |        |          |Note2: When operation in Automatic Resynchronization mode, the baud rate setting must be mode2
 * |        |          |(BAUD_M1 (UA_BAUD [29]) and BAUD_M0 (UA_BAUD [28]) must be 1). 
 * |        |          |(Slave mode with automatic resynchronization).
 * |[3]     |LINS_DUM_EN|LIN Slave Divider Update Method Enable Control
 * |        |          |0 = UA_BAUD updated is written by software (if no automatic resynchronization update occurs at the same time).
 * |        |          |1 = UA_BAUD is updated at the next received character. User must set the bit before checksum reception.
 * |        |          |Note1: This bit only valid when in LIN slave mode (LINS_EN (UA_LIN_CTL[0]) = 1).
 * |        |          |Note2: This bit used for LIN Slave Automatic Resynchronization mode.
 * |        |          |(for Non-Automatic Resynchronization mode, this bit should be kept cleared).
 * |        |          |(Slave mode with automatic resynchronization).
 * |[4]     |LIN_MUTE_EN|LIN Mute Mode Enable Control
 * |        |          |0 = LIN mute mode Disabled.
 * |        |          |1 = LIN mute mode Enabled.
 * |        |          |Note: The exit from mute mode condition and each control and interactions of this field are explained in (LIN slave mode).
 * |[8]     |LIN_SHD   |LIN TX Send Header Enable Control
 * |        |          |The LIN TX header can be "break field" or "break and sync field" or "break, sync and frame ID
 * |        |          |field", it is depend on setting LIN_HEAD_SEL (UA_LIN_CTL[23:22]).
 * |        |          |0 = Send LIN TX header Disabled.
 * |        |          |1 = Send LIN TX header Enabled.
 * |        |          |Note1: These registers are shadow registers of LIN_SHD (UA_ALT_CSR [7]); user can read/write it
 * |        |          |by setting LIN_SHD (UA_ALT_CSR [7]) or LIN_SHD (UA_LIN_CTL [8]).
 * |        |          |Note2: When transmitter header field (it may be "break" or "break + sync" or "break + sync +
 * |        |          |frame ID" selected by LIN_HEAD_SEL (UA_LIN_CTL[23:22]) field) transfer operation finished, this
 * |        |          |bit will be cleared automatically.
 * |[9]     |LIN_IDPEN |LIN ID Parity Enable Control
 * |        |          |0 = LIN frame ID parity Disabled.
 * |        |          |1 = LIN frame ID parity Enabled.
 * |        |          |Note1: This bit can be used for LIN master to sending header field LIN_SHD (UA_LIN_CTL[8]) = 1
 * |        |          |and LIN_HEAD_SEL (UA_LIN_CTL[23:22]) = 10 or be used for enable LIN slave received frame ID parity checked.
 * |        |          |Note2: This bit is only use when the operation header transmitter is in LIN_HEAD_SEL (UA_LIN_CTL[23:22]) = 10.
 * |[10]    |LIN_BKDET_EN|LIN Break Detection Enable Control
 * |        |          |When detect consecutive dominant greater than 11 bits, and are followed by a delimiter
 * |        |          |character, the LIN_BKDET_F (UA_LIN_SR[8]) flag is set in UA_LIN_SR register at the end of break field.
 * |        |          |If the LIN_IEN (UA_IER [8])=1, an interrupt will be generated.
 * |        |          |0 = LIN break detection Disabled.
 * |        |          |1 = LIN break detection Enabled.
 * |[11]    |LIN_RX_DIS|LIN Receiver Disable Control
 * |        |          |If the receiver is enabled (LIN_RX_DIS (UA_LIN_CTL[11]) = 0), all received byte data will be
 * |        |          |accepted and stored in the RX-FIFO, and if the receiver is disabled (LIN_RX_DIS (UA_LIN_CTL[11])
 * |        |          |= 1), all received byte data will be ignore.
 * |        |          |0 = LIN receiver Enabled.
 * |        |          |1 = LIN receiver Disabled.
 * |        |          |Note: This bit is only valid when operating in LIN function mode (FUN_SEL (UA_FUN_SEL[1:0]) = 01).
 * |[12]    |BIT_ERR_EN|Bit Error Detect Enable Control
 * |        |          |0 = Bit error detection function Disabled.
 * |        |          |1 = Bit error detection Enabled.
 * |        |          |Note: In LIN function mode, when occur bit error, the BIT_ERR_F (UA_LIN_SR[9]) flag will be
 * |        |          |asserted. If the LIN_IEN (UA_IER[8]) = 1, an interrupt will be generated.
 * |[19:16] |LIN_BKFL  |LIN Break Field Length
 * |        |          |This field indicates a 4-bit LIN TX break field count.
 * |        |          |Note1: These registers are shadow registers of LIN_BKFL, User can read/write it by setting
 * |        |          |LIN_BKFL (UA_ALT_CSR[3:0]) or LIN_BKFL (UA_LIN_CTL[19:16]).
 * |        |          |Note2: This break field length is LIN_BKFL + 1.
 * |        |          |Note3: According to LIN spec, the reset value is 12 (break field length = 13).
 * |[21:20] |LIN_BS_LEN|LIN Break/Sync Delimiter Length
 * |        |          |00 = The LIN break/sync delimiter length is 1 bit time.
 * |        |          |10 = The LIN break/sync delimiter length is 2 bit time.
 * |        |          |10 = The LIN break/sync delimiter length is 3 bit time.
 * |        |          |11 = The LIN break/sync delimiter length is 4 bit time.
 * |        |          |Note: This bit used for LIN master to sending header field.
 * |[23:22] |LIN_HEAD_SEL|LIN Header Select
 * |        |          |00 = The LIN header includes "break field".
 * |        |          |01 = The LIN header includes "break field" and "sync field".
 * |        |          |10 = The LIN header includes "break field", "sync field" and "frame ID field".
 * |        |          |11 = Reserved.
 * |        |          |Note: This bit is used to master mode for LIN to send header field (LIN_SHD (UA_LIN_CTL [8]) = 1)
 * |        |          |or used to slave to indicates exit from mute mode condition (LIN_MUTE_EN (UA_LIN_CTL[4]) = 1).
 * |[31:24] |LIN_PID   |LIN PID Register
 * |        |          |This field contains the LIN frame ID value when in LIN function mode, the frame ID parity can be
 * |        |          |generated by software or hardware depends on LIN_IDPEN (UA_LIN_CTL[9]) = 1.
 * |        |          |If the parity generated by hardware, user fill ID0~ID5, (LIN_PID [29:24] )hardware will
 * |        |          |calculate P0 (LIN_PID[30]) and P1 (LIN_PID[31]), otherwise user must filled frame ID and parity in this field.
 * |        |          |Note1: User can fill any 8-bit value to this field and the bit 24 indicates ID0 (LSB first).
 * |        |          |Note2: This field can be used for LIN master mode or slave mode.
 * @var UART_T::LIN_SR
 * Offset: 0x38  UART LIN Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |LINS_HDET_F|LIN Slave Header Detection Flag
 * |        |          |This bit is set by hardware when a LIN header is detected in LIN slave mode and be cleared by writing 1 to it.
 * |        |          |0 = LIN header not detected.
 * |        |          |1 = LIN header detected (break + sync + frame ID).
 * |        |          |Note1: This bit is can be cleared by writing 1 to it.
 * |        |          |Note2: This bit is only valid when in LIN slave mode (LINS_EN (UA_LIN_CTL [0]) = 1) and enable
 * |        |          |LIN slave header detection function (LINS_HDET_EN (UA_LIN_CTL [1])).
 * |        |          |Note3: When enable ID parity check LIN_IDPEN (UA_LIN_CTL [9]), if hardware detect complete
 * |        |          |header ("break + sync + frame ID"), the LINS_HEDT_F will be set whether the frame ID correct or not.
 * |[1]     |LINS_HERR_F|LIN Slave Header Error Flag
 * |        |          |This bit is set by hardware when a LIN header error is detected in LIN slave mode and be cleared
 * |        |          |by writing 1 to it.
 * |        |          |The header errors include "break delimiter is too short (less than 0.5 bit time)", "frame error
 * |        |          |in sync field or Identifier field", "sync field data is not 0x55 in Non-Automatic
 * |        |          |Resynchronization mode", "sync field deviation error with Automatic Resynchronization mode",
 * |        |          |"sync field measure time-out with Automatic Resynchronization mode" and "LIN header reception time-out".
 * |        |          |0 = LIN header error not detected.
 * |        |          |1 = LIN header error detected.
 * |        |          |Note1: This bit can be cleared by writing 1 to it.
 * |        |          |Note2: This bit is only valid when UART is operated in LIN slave mode (LINS_EN (UA_LIN_CTL [0])
 * |        |          |= 1) and enables LIN slave header detection function (LINS_HDET_EN (UA_LIN_CTL [1])).
 * |[2]     |LINS_IDPERR_F|LIN Slave ID Parity Error Flag
 * |        |          |This bit is set by hardware when receipted frame ID parity is not correct.
 * |        |          |0 = No active.
 * |        |          |1 = Receipted frame ID parity is not correct.
 * |        |          |Note1: This bit iscan be cleared by writing "1" to it.
 * |        |          |Note2: This bit is only valid when in LIN slave mode (LINS_EN (UA_LIN_CTL [0])= 1) and enable
 * |        |          |LIN frame ID parity check function LIN_IDPEN (UA_LIN_CTL [9]).
 * |[3]     |LINS_SYNC_F|LIN Slave Sync Field
 * |        |          |This bit indicates that the LIN sync field is being analyzed in Automatic Resynchronization mode.
 * |        |          |When the receiver header have some error been detect, user must reset the internal circuit to
 * |        |          |re-search new frame header by writing 1 to this bit.
 * |        |          |0 = The current character is not at LIN sync state.
 * |        |          |1 = The current character is at LIN sync state.
 * |        |          |Note1: This bit is only valid when in LIN Slave mode (LINS_EN(UA_LIN_CTL[0]) = 1).
 * |        |          |Note2: This bitcan be cleared by writing 1 to it.
 * |        |          |Note3: When writing 1 to it, hardware will reload the initial baud rate and re-search a new frame header.
 * |[8]     |LIN_BKDET_F|LIN Break Detection Flag
 * |        |          |This bit is set by hardware when a break is detected and be cleared by writing 1 to it through software.
 * |        |          |0 = LIN break not detected.
 * |        |          |1 = LIN break detected.
 * |        |          |Note1: This bitcan be cleared by writing 1 to it.
 * |        |          |Note2: This bit is only valid when LIN break detection function is enabled (LIN_BKDET_EN (UA_LIN_CTL[10]) =1).
 * |[9]     |BIT_ERR_F |Bit Error Detect Status Flag
 * |        |          |At TX transfer state, hardware will monitoring the bus state, if the input pin (SIN) state not
 * |        |          |equals to the output pin (SOUT) state, BIT_ERR_F (UA_LIN_SR[9]) will be set.
 * |        |          |When occur bit error, if the LIN_IEN (UA_IER[8]) = 1, an interrupt will be generated.
 * |        |          |Note1: This bit iscan be cleared by writing 1 to it.
 * |        |          |Note2: This bit is only valid when enable bit error detection function (BIT_ERR_EN (UA_LIN_CTL [12]) = 1).
 */

    union {
        __IO uint32_t DATA;          /* Offset: 0x00  UART Data Register                                                 */
        __IO uint32_t THR;           /* Offset: 0x00  UART Transmit Holding Register                                     */
        __IO uint32_t RBR;           /* Offset: 0x00  UART Receive Buffer Register                                       */
    };
    __IO uint32_t IER;           /* Offset: 0x04  UART Interrupt Enable Register                                     */
    __IO uint32_t FCR;           /* Offset: 0x08  UART FIFO Control Register                                         */
    __IO uint32_t LCR;           /* Offset: 0x0C  UART Line Control Register                                         */
    __IO uint32_t MCR;           /* Offset: 0x10  UART Modem Control Register                                        */
    __IO uint32_t MSR;           /* Offset: 0x14  UART Modem Status Register                                         */
    __IO uint32_t FSR;           /* Offset: 0x18  UART FIFO Status Register                                          */
    __IO uint32_t ISR;           /* Offset: 0x1C  UART Interrupt Status Register                                     */
    __IO uint32_t TOR;           /* Offset: 0x20  UART Time-out Register                                             */
    __IO uint32_t BAUD;          /* Offset: 0x24  UART Baud Rate Divisor Register                                    */
    __IO uint32_t IRCR;          /* Offset: 0x28  UART IrDA Control Register                                         */
    __IO uint32_t ALT_CSR;       /* Offset: 0x2C  UART Alternate Control/Status Register                             */
    __IO uint32_t FUN_SEL;       /* Offset: 0x30  UART Function Select Register                                      */
    __IO uint32_t LIN_CTL;       /* Offset: 0x34  UART LIN Control Register                                          */
    __IO uint32_t LIN_SR;        /* Offset: 0x38  UART LIN Status Register                                           */
    

} UART_T;



/**
    @addtogroup UART_CONST UART Bit Field Definition
    Constant Definitions for UART Controller
@{ */


/* UART THR Bit Field Definitions */
#define UART_THR_THR_Pos         0                                          /*!< UART_T::THR: THR Position  */
#define UART_THR_THR_Msk        (0xFul << UART_THR_THR_Pos)                 /*!< UART_T::THR: THR Mask      */

/* UART RBR Bit Field Definitions */
#define UART_RBR_RBR_Pos         0                                          /*!< UART_T::RBR: RBR Posistion */
#define UART_RBR_RBR_Msk        (0xFul << UART_RBR_RBR_Pos)                 /*!< UART_T::RBR: RBR Mask      */

/* UART IER Bit Field Definitions */
#define UART_IER_DMA_RX_EN_Pos      15                                      /*!< UART_T::IER: RX DMA Enable Posistion */
#define UART_IER_DMA_RX_EN_Msk      (1ul << UART_IER_DMA_RX_EN_Pos)         /*!< UART_T::IER: RX DMA Enable Mask      */

#define UART_IER_DMA_TX_EN_Pos      14                                      /*!< UART_T::IER: TX DMA Enable Posistion */
#define UART_IER_DMA_TX_EN_Msk      (1ul << UART_IER_DMA_TX_EN_Pos)         /*!< UART_T::IER: TX DMA Enable Mask      */

#define UART_IER_AUTO_CTS_EN_Pos    13                                      /*!< UART_T::IER: AUTO_CTS_EN Posistion      */
#define UART_IER_AUTO_CTS_EN_Msk    (1ul << UART_IER_AUTO_CTS_EN_Pos)       /*!< UART_T::IER: AUTO_CTS_EN Mask           */

#define UART_IER_AUTO_RTS_EN_Pos    12                                      /*!< UART_T::IER: AUTO_RTS_EN Posistion      */
#define UART_IER_AUTO_RTS_EN_Msk    (1ul << UART_IER_AUTO_RTS_EN_Pos)       /*!< UART_T::IER: AUTO_RTS_EN Mask           */

#define UART_IER_TIME_OUT_EN_Pos    11                                      /*!< UART_T::IER: TIME_OUT_EN Posistion      */
#define UART_IER_TIME_OUT_EN_Msk    (1ul << UART_IER_TIME_OUT_EN_Pos)       /*!< UART_T::IER: TIME_OUT_EN Mask           */

#define UART_IER_LIN_IEN_Pos        8                                       /*!< UART_T::IER: LIN_IEN Posistion          */
#define UART_IER_LIN_IEN_Msk        (1ul << UART_IER_LIN_IEN_Pos)           /*!< UART_T::IER: LIN_IEN Mask               */

#define UART_IER_WAKE_EN_Pos        6                                       /*!< UART_T::IER: WAKE_EN Posistion          */
#define UART_IER_WAKE_EN_Msk        (1ul << UART_IER_WAKE_EN_Pos)           /*!< UART_T::IER: WAKE_EN Mask               */

#define UART_IER_BUF_ERR_IEN_Pos    5                                       /*!< UART_T::IER: BUF_ERR_IEN Posistion      */
#define UART_IER_BUF_ERR_IEN_Msk    (1ul << UART_IER_BUF_ERR_IEN_Pos)       /*!< UART_T::IER: BUF_ERR_IEN Mask           */

#define UART_IER_TOUT_IEN_Pos        4                                      /*!< UART_T::IER: TOUT_IEN Posistion          */
#define UART_IER_TOUT_IEN_Msk        (1ul << UART_IER_TOUT_IEN_Pos)         /*!< UART_T::IER: TOUT_IEN Mask               */

#define UART_IER_MODEM_IEN_Pos      3                                       /*!< UART_T::IER: MODEM_IEN Posistion        */
#define UART_IER_MODEM_IEN_Msk      (1ul << UART_IER_MODEM_IEN_Pos)         /*!< UART_T::IER: MODEM_IEN Mask             */

#define UART_IER_RLS_IEN_Pos        2                                       /*!< UART_T::IER: RLS_IEN Posistion          */
#define UART_IER_RLS_IEN_Msk        (1ul << UART_IER_RLS_IEN_Pos)           /*!< UART_T::IER: RLS_IEN Mask               */

#define UART_IER_THRE_IEN_Pos       1                                       /*!< UART_T::IER: THRE_IEN Posistion         */
#define UART_IER_THRE_IEN_Msk       (1ul << UART_IER_THRE_IEN_Pos)          /*!< UART_T::IER: THRE_IEN Mask              */

#define UART_IER_RDA_IEN_Pos        0                                       /*!< UART_T::IER: RDA_IEN Position           */
#define UART_IER_RDA_IEN_Msk        (1ul << UART_IER_RDA_IEN_Pos)           /*!< UART_T::IER: RDA_IEN Mask               */

/* UART FCR Bit Field Definitions */
#define UART_FCR_RTS_TRI_LEV_Pos    16                                      /*!< UART_T::FCR: RTS_TRI_LEV Position       */
#define UART_FCR_RTS_TRI_LEV_Msk    (0xFul << UART_FCR_RTS_TRI_LEV_Pos)     /*!< UART_T::FCR: RTS_TRI_LEV Mask           */

#define UART_FCR_RX_DIS_Pos         8                                       /*!< UART_T::FCR: RX_DIS Position            */
#define UART_FCR_RX_DIS_Msk         (1ul << UART_FCR_RX_DIS_Pos)            /*!< UART_T::FCR: RX_DIS Mask                */

#define UART_FCR_RFITL_Pos          4                                       /*!< UART_T::FCR: RFITL Position             */
#define UART_FCR_RFITL_Msk          (0xFul << UART_FCR_RFITL_Pos)           /*!< UART_T::FCR: RFITL Mask                 */

#define UART_FCR_TFR_Pos            2                                       /*!< UART_T::FCR: TFR Position               */
#define UART_FCR_TFR_Msk            (1ul << UART_FCR_TFR_Pos)               /*!< UART_T::FCR: TFR Mask                   */

#define UART_FCR_RFR_Pos            1                                       /*!< UART_T::FCR: RFR Position               */
#define UART_FCR_RFR_Msk            (1ul << UART_FCR_RFR_Pos)               /*!< UART_T::FCR: RFR Mask                   */

/* UART LCR Bit Field Definitions */
#define UART_LCR_BCB_Pos            6                                       /*!< UART_T::LCR: BCB Position               */
#define UART_LCR_BCB_Msk            (1ul << UART_LCR_BCB_Pos)               /*!< UART_T::LCR: BCB Mask                   */

#define UART_LCR_SPE_Pos            5                                       /*!< UART_T::LCR: SPE Position               */
#define UART_LCR_SPE_Msk            (1ul << UART_LCR_SPE_Pos)               /*!< UART_T::LCR: SPE Mask                   */

#define UART_LCR_EPE_Pos            4                                       /*!< UART_T::LCR: EPE Position               */
#define UART_LCR_EPE_Msk            (1ul << UART_LCR_EPE_Pos)               /*!< UART_T::LCR: EPE Mask                   */

#define UART_LCR_PBE_Pos            3                                       /*!< UART_T::LCR: PBE Position               */
#define UART_LCR_PBE_Msk            (1ul << UART_LCR_PBE_Pos)               /*!< UART_T::LCR: PBE Mask                   */

#define UART_LCR_NSB_Pos            2                                       /*!< UART_T::LCR: NSB Position               */
#define UART_LCR_NSB_Msk            (1ul << UART_LCR_NSB_Pos)               /*!< UART_T::LCR: NSB Mask                   */

#define UART_LCR_WLS_Pos            0                                       /*!< UART_T::LCR: WLS Position               */
#define UART_LCR_WLS_Msk            (0x3ul << UART_LCR_WLS_Pos)             /*!< UART_T::LCR: WLS Mask                   */

/* UART MCR Bit Field Definitions */
#define UART_MCR_RTS_ST_Pos         13                                      /*!< UART_T::MCR: RTS_ST Position            */
#define UART_MCR_RTS_ST_Msk         (1ul << UART_MCR_RTS_ST_Pos)            /*!< UART_T::MCR: RTS_ST Mask                */

#define UART_MCR_LEV_RTS_Pos        9                                       /*!< UART_T::MCR: LEV_RTS Position           */
#define UART_MCR_LEV_RTS_Msk        (1ul << UART_MCR_LEV_RTS_Pos)           /*!< UART_T::MCR: LEV_RTS Mask               */

#define UART_MCR_RTS_Pos            1                                       /*!< UART_T::MCR: RTS Position               */
#define UART_MCR_RTS_Msk            (1ul << UART_MCR_RTS_Pos)               /*!< UART_T::MCR: RTS Mask                   */

/* UART MSR Bit Field Definitions */
#define UART_MSR_LEV_CTS_Pos        8                                       /*!< UART_T::MSR: LEV_CTS Position           */
#define UART_MSR_LEV_CTS_Msk        (1ul << UART_MSR_LEV_CTS_Pos)           /*!< UART_T::MSR: LEV_CTS Mask               */

#define UART_MSR_CTS_ST_Pos         4                                       /*!< UART_T::MSR: CTS_ST Position            */
#define UART_MSR_CTS_ST_Msk         (1ul << UART_MSR_CTS_ST_Pos)            /*!< UART_T::MSR: CTS_ST Mask                */

#define UART_MSR_DCTSF_Pos          0                                       /*!< UART_T::MSR: DCTST Position             */
#define UART_MSR_DCTSF_Msk          (1ul << UART_MSR_DCTSF_Pos)             /*!< UART_T::MSR: DCTST Mask                 */

/* UART FSR Bit Field Definitions */
#define UART_FSR_TE_FLAG_Pos        28                                      /*!< UART_T::FSR: TE_FLAG Position           */
#define UART_FSR_TE_FLAG_Msk        (1ul << UART_FSR_TE_FLAG_Pos)           /*!< UART_T::FSR: TE_FLAG Mask               */

#define UART_FSR_TX_OVER_IF_Pos     24                                      /*!< UART_T::FSR: TX_OVER_IF Position        */
#define UART_FSR_TX_OVER_IF_Msk     (1ul << UART_FSR_TX_OVER_IF_Pos)        /*!< UART_T::FSR: TX_OVER_IF Mask            */

#define UART_FSR_TX_FULL_Pos        23                                      /*!< UART_T::FSR: TX_FULL Position           */
#define UART_FSR_TX_FULL_Msk        (1ul << UART_FSR_TX_FULL_Pos)           /*!< UART_T::FSR: TX_FULL Mask               */

#define UART_FSR_TX_EMPTY_Pos       22                                      /*!< UART_T::FSR: TX_EMPTY Position          */
#define UART_FSR_TX_EMPTY_Msk       (1ul << UART_FSR_TX_EMPTY_Pos)          /*!< UART_T::FSR: TX_EMPTY Mask              */

#define UART_FSR_TX_POINTER_Pos     16                                      /*!< UART_T::FSR: TX_POINTER Position        */
#define UART_FSR_TX_POINTER_Msk     (0x3Ful << UART_FSR_TX_POINTER_Pos)     /*!< UART_T::FSR: TX_POINTER Mask            */

#define UART_FSR_RX_FULL_Pos        15                                      /*!< UART_T::FSR: RX_FULL Position           */
#define UART_FSR_RX_FULL_Msk        (1ul << UART_FSR_RX_FULL_Pos)           /*!< UART_T::FSR: RX_FULL Mask               */

#define UART_FSR_RX_EMPTY_Pos       14                                      /*!< UART_T::FSR: RX_EMPTY Position          */
#define UART_FSR_RX_EMPTY_Msk       (1ul << UART_FSR_RX_EMPTY_Pos)          /*!< UART_T::FSR: RX_EMPTY Mask              */

#define UART_FSR_RX_POINTER_Pos     8                                       /*!< UART_T::FSR: RX_POINTERS Position       */
#define UART_FSR_RX_POINTER_Msk     (0x3Ful << UART_FSR_RX_POINTER_Pos)     /*!< UART_T::FSR: RX_POINTER Mask            */

#define UART_FSR_BIF_Pos            6                                       /*!< UART_T::FSR: BIF Position               */
#define UART_FSR_BIF_Msk            (1ul << UART_FSR_BIF_Pos)               /*!< UART_T::FSR: BIF Mask                   */

#define UART_FSR_FEF_Pos            5                                       /*!< UART_T::FSR: FEF Position               */
#define UART_FSR_FEF_Msk            (1ul << UART_FSR_FEF_Pos)               /*!< UART_T::FSR: FEF Mask                   */

#define UART_FSR_PEF_Pos            4                                       /*!< UART_T::FSR: PEF Position               */
#define UART_FSR_PEF_Msk            (1ul << UART_FSR_PEF_Pos)               /*!< UART_T::FSR: PEF Mask                   */

#define UART_FSR_RS485_ADD_DETF_Pos 3                                       /*!< UART_T::FSR: RS485_ADD_DETF Position    */
#define UART_FSR_RS485_ADD_DETF_Msk (1ul << UART_FSR_RS485_ADD_DETF_Pos)    /*!< UART_T::FSR: RS485_ADD_DETF Mask        */

#define UART_FSR_RX_OVER_IF_Pos     0                                       /*!< UART_T::FSR: RX_OVER_IF Position        */
#define UART_FSR_RX_OVER_IF_Msk     (1ul << UART_FSR_RX_OVER_IF_Pos)        /*!< UART_T::FSR: RX_OVER_IF Mask            */

/* UART ISR Bit Field Definitions */
#define UART_ISR_HW_BUF_ERR_INT_Pos 29                                      /*!< UART_T::ISR: HW BUF_ERR_INT Position    */
#define UART_ISR_HW_BUF_ERR_INT_Msk (1ul << UART_ISR_HW_BUF_ERR_INT_Pos)    /*!< UART_T::ISR: HW BUF_ERR_INT Mask        */

#define UART_ISR_HW_TOUT_INT_Pos    28                                      /*!< UART_T::ISR: HW TOUT_INT Position       */
#define UART_ISR_HW_TOUT_INT_Msk    (1ul << UART_ISR_HW_TOUT_INT_Pos)       /*!< UART_T::ISR: HW TOUT_INT Mask           */

#define UART_ISR_HW_MODEM_INT_Pos   27                                      /*!< UART_T::ISR: HW MODEM_INT Position      */
#define UART_ISR_HW_MODEM_INT_Msk   (1ul << UART_ISR_HW_MODEM_INT_Pos)      /*!< UART_T::ISR: HW MODEM_INT Mask          */

#define UART_ISR_HW_RLS_INT_Pos     26                                      /*!< UART_T::ISR: HW RLS_INT Position        */
#define UART_ISR_HW_RLS_INT_Msk     (1ul << UART_ISR_HW_RLS_INT_Pos)        /*!< UART_T::ISR: HW RLS_INT Position        */

#define UART_ISR_HW_BUF_ERR_IF_Pos  21                                      /*!< UART_T::ISR: HW BUF_ERR_IF Position     */
#define UART_ISR_HW_BUF_ERR_IF_Msk  (1ul << UART_ISR_HW_BUF_ERR_IF_Pos)     /*!< UART_T::ISR: HW BUF_ERR_IF Mask         */

#define UART_ISR_HW_TOUT_IF_Pos     20                                      /*!< UART_T::ISR: HW TOUT_IF Position        */
#define UART_ISR_HW_TOUT_IF_Msk     (1ul << UART_ISR_HW_TOUT_IF_Pos)        /*!< UART_T::ISR: HW TOUT_IF Mask            */

#define UART_ISR_HW_MODEM_IF_Pos    19                                      /*!< UART_T::ISR: HW MODEM_IF Position       */
#define UART_ISR_HW_MODEM_IF_Msk    (1ul << UART_ISR_HW_MODEM_IF_Pos)       /*!< UART_T::ISR: HW MODEM_IF Mask           */

#define UART_ISR_HW_RLS_IF_Pos      18                                      /*!< UART_T::ISR: HW RLS_IF Position         */
#define UART_ISR_HW_RLS_IF_Msk      (1ul << UART_ISR_HW_RLS_IF_Pos)         /*!< UART_T::ISR: HW RLS_IF Mark             */

#define UART_ISR_LIN_INT_Pos        15                                      /*!< UART_T::ISR: LIN_INT Position           */
#define UART_ISR_LIN_INT_Msk        (1ul << UART_ISR_LIN_INT_Pos)           /*!< UART_T::ISR: LIN_INT Mask               */

#define UART_ISR_BUF_ERR_INT_Pos    13                                      /*!< UART_T::ISR: BUF_ERR_INT Position       */
#define UART_ISR_BUF_ERR_INT_Msk    (1ul << UART_ISR_BUF_ERR_INT_Pos)       /*!< UART_T::ISR: BUF_ERR_INT Mask           */

#define UART_ISR_TOUT_INT_Pos       12                                      /*!< UART_T::ISR: TOUT_INT Position          */
#define UART_ISR_TOUT_INT_Msk       (1ul << UART_ISR_TOUT_INT_Pos)          /*!< UART_T::ISR: TOUT_INT Mask              */

#define UART_ISR_MODEM_INT_Pos      11                                      /*!< UART_T::ISR: MODEM_INT Position         */
#define UART_ISR_MODEM_INT_Msk      (1ul << UART_ISR_MODEM_INT_Pos)         /*!< UART_T::ISR: MODEM_INT Mask             */

#define UART_ISR_RLS_INT_Pos        10                                      /*!< UART_T::ISR: RLS_INT Position           */
#define UART_ISR_RLS_INT_Msk        (1ul << UART_ISR_RLS_INT_Pos)           /*!< UART_T::ISR: RLS_INT Mask               */

#define UART_ISR_THRE_INT_Pos       9                                       /*!< UART_T::ISR: THRE_INT Position          */
#define UART_ISR_THRE_INT_Msk       (1ul << UART_ISR_THRE_INT_Pos)          /*!< UART_T::ISR: THRE_INT Mask              */

#define UART_ISR_RDA_INT_Pos        8                                       /*!< UART_T::ISR: RDA_INT Position           */
#define UART_ISR_RDA_INT_Msk        (1ul << UART_ISR_RDA_INT_Pos)           /*!< UART_T::ISR: RDA_INT Mask               */

#define UART_ISR_LIN_IF_Pos         7                                       /*!< UART_T::ISR: LIN RX_IF Position         */
#define UART_ISR_LIN_IF_Msk         (1ul << UART_ISR_LIN_IF_Pos)            /*!< UART_T::ISR: LIN RX_IF Mask             */

#define UART_ISR_BUF_ERR_IF_Pos     5                                       /*!< UART_T::ISR: BUF_ERR_IF Position        */
#define UART_ISR_BUF_ERR_IF_Msk     (1ul << UART_ISR_BUF_ERR_IF_Pos)        /*!< UART_T::ISR: BUF_ERR_IF Mask            */

#define UART_ISR_TOUT_IF_Pos        4                                       /*!< UART_T::ISR: TOUT_IF Position           */
#define UART_ISR_TOUT_IF_Msk        (1ul << UART_ISR_TOUT_IF_Pos)           /*!< UART_T::ISR: TOUT_IF Mask               */

#define UART_ISR_MODEM_IF_Pos       3                                       /*!< UART_T::ISR: MODEM_IF Position          */
#define UART_ISR_MODEM_IF_Msk       (1ul << UART_ISR_MODEM_IF_Pos)          /*!< UART_T::ISR: MODEM_IF Mask              */

#define UART_ISR_RLS_IF_Pos         2                                       /*!< UART_T::ISR: RLS_IF Position            */
#define UART_ISR_RLS_IF_Msk         (1ul << UART_ISR_RLS_IF_Pos)            /*!< UART_T::ISR: RLS_IF Mask                */

#define UART_ISR_THRE_IF_Pos        1                                       /*!< UART_T::ISR: THRE_IF Position           */
#define UART_ISR_THRE_IF_Msk        (1ul << UART_ISR_THRE_IF_Pos)           /*!< UART_T::ISR: THRE_IF Mask               */

#define UART_ISR_RDA_IF_Pos         0                                       /*!< UART_T::ISR: RDA_IF Position            */
#define UART_ISR_RDA_IF_Msk         (1ul << UART_ISR_RDA_IF_Pos)            /*!< UART_T::ISR: RDA_IF Mask                */

/* UART TOR Bit Field Definitions */
#define UART_TOR_DLY_Pos           8                                        /*!< UART_T::TOR: DLY Position               */
#define UART_TOR_DLY_Msk           (0xFFul << UART_TOR_DLY_Pos)             /*!< UART_T::TOR: DLY Mask                   */

#define UART_TOR_TOIC_Pos          0                                        /*!< UART_T::TOR: TOIC Position              */
#define UART_TOR_TOIC_Msk          (0xFFul << UART_TOR_TOIC_Pos)

/* UART BAUD Bit Field Definitions */
#define UART_BAUD_DIV_X_EN_Pos    29                                        /*!< UART_T::BAUD: DIV_X_EN Position         */
#define UART_BAUD_DIV_X_EN_Msk    (1ul << UART_BAUD_DIV_X_EN_Pos)           /*!< UART_T::BAUD: DIV_X_EN Mask             */

#define UART_BAUD_DIV_X_ONE_Pos   28                                        /*!< UART_T::BAUD: DIV_X_ONE Position        */
#define UART_BAUD_DIV_X_ONE_Msk   (1ul << UART_BAUD_DIV_X_ONE_Pos)          /*!< UART_T::BAUD: DIV_X_ONE Mask            */

#define UART_BAUD_DIVIDER_X_Pos   24                                        /*!< UART_T::BAUD: DIVIDER_X Position        */
#define UART_BAUD_DIVIDER_X_Msk   (0xFul << UART_BAUD_DIVIDER_X_Pos)        /*!< UART_T::BAUD: DIVIDER_X Mask            */

#define UART_BAUD_BRD_Pos         0                                         /*!< UART_T::BAUD: BRD Position              */
#define UART_BAUD_BRD_Msk         (0xFFFFul << UART_BAUD_BRD_Pos)           /*!< UART_T::BAUD: BRD Mask                  */

/* UART IRCR Bit Field Definitions */
#define UART_IRCR_INV_RX_Pos      6                                         /*!< UART_T::IRCR: INV_RX Position           */
#define UART_IRCR_INV_RX_Msk     (1ul << UART_IRCR_INV_RX_Pos)              /*!< UART_T::IRCR: INV_RX Mask               */

#define UART_IRCR_INV_TX_Pos      5                                         /*!< UART_T::IRCR: INV_TX Position           */
#define UART_IRCR_INV_TX_Msk     (1ul << UART_IRCR_INV_TX_Pos)              /*!< UART_T::IRCR: INV_TX Mask               */

#define UART_IRCR_TX_SELECT_Pos   1                                         /*!< UART_T::IRCR: TX_SELECT Position        */
#define UART_IRCR_TX_SELECT_Msk   (1ul << UART_IRCR_TX_SELECT_Pos)          /*!< UART_T::IRCR: TX_SELECT Mask            */

/* UART ALT_CSR Bit Field Definitions */
#define UART_ALT_CSR_ADDR_MATCH_Pos      24                                      /*!< UART_T::ALT_CSR: ADDR_MATCH Position    */
#define UART_ALT_CSR_ADDR_MATCH_Msk     (0xFFul << UART_ALT_CSR_ADDR_MATCH_Pos)  /*!< UART_T::ALT_CSR: ADDR_MATCH Mask        */

#define UART_ALT_CSR_RS485_ADD_EN_Pos   15                                       /*!< UART_T::ALT_CSR: RS485_ADD_EN Position  */
#define UART_ALT_CSR_RS485_ADD_EN_Msk   (1ul << UART_ALT_CSR_RS485_ADD_EN_Pos)   /*!< UART_T::ALT_CSR: RS485_ADD_EN Mask      */

#define UART_ALT_CSR_RS485_AUD_Pos      10                                       /*!< UART_T::ALT_CSR: RS485_AUD Position     */
#define UART_ALT_CSR_RS485_AUD_Msk      (1ul << UART_ALT_CSR_RS485_AUD_Pos)      /*!< UART_T::ALT_CSR: RS485_AUD Mask         */

#define UART_ALT_CSR_RS485_AAD_Pos      9                                        /*!< UART_T::ALT_CSR: RS485_AAD Position     */
#define UART_ALT_CSR_RS485_AAD_Msk      (1ul << UART_ALT_CSR_RS485_AAD_Pos)      /*!< UART_T::ALT_CSR: RS485_AAD Mask         */

#define UART_ALT_CSR_RS485_NMM_Pos      8                                        /*!< UART_T::ALT_CSR: RS485_NMM Position     */
#define UART_ALT_CSR_RS485_NMM_Msk      (1ul << UART_ALT_CSR_RS485_NMM_Pos)      /*!< UART_T::ALT_CSR: RS485_NMM Mask         */

#define UART_ALT_CSR_LIN_TX_EN_Pos      7                                        /*!< UART_T::ALT_CSR: LIN TX Break Mode Enable Position     */
#define UART_ALT_CSR_LIN_TX_EN_Msk      (1ul << UART_ALT_CSR_LIN_TX_EN_Pos)      /*!< UART_T::ALT_CSR: LIN TX Break Mode Enable Mask         */

#define UART_ALT_CSR_LIN_RX_EN_Pos      6                                        /*!< UART_T::ALT_CSR: LIN RX Enable Position     */
#define UART_ALT_CSR_LIN_RX_EN_Msk      (1ul << UART_ALT_CSR_LIN_RX_EN_Pos)      /*!< UART_T::ALT_CSR: LIN RX Enable Mask         */

#define UART_ALT_CSR_UA_LIN_BKFL_Pos    0                                        /*!< UART_T::ALT_CSR: UART LIN Break Field Length Position     */
#define UART_ALT_CSR_UA_LIN_BKFL_Msk    (0xFul << UART_ALT_CSR_UA_LIN_BKFL_Pos)  /*!< UART_T::ALT_CSR: UART LIN Break Field Length Mask         */

/* UART FUN_SEL Bit Field Definitions */
#define UART_FUN_SEL_FUN_SEL_Pos        0                                        /*!< UART_T::FUN_SEL: FUN_SEL Position       */
#define UART_FUN_SEL_FUN_SEL_Msk       (0x3ul << UART_FUN_SEL_FUN_SEL_Pos)       /*!< UART_T::FUN_SEL: FUN_SEL Mask           */

/* UART LIN_CTL Bit Field Definitions */
#define UART_LIN_CTL_LIN_PID_Pos        24                                        /*!< UART_T::LIN_CTL: LIN_PID Position       */
#define UART_LIN_CTL_LIN_PID_Msk        (0xFFul << UART_LIN_CTL_LIN_PID_Pos)      /*!< UART_T::LIN_CTL: LIN_PID Mask           */

#define UART_LIN_CTL_LIN_HEAD_SEL_Pos   22                                        /*!< UART_T::LIN_CTL: LIN_HEAD_SEL Position       */
#define UART_LIN_CTL_LIN_HEAD_SEL_Msk   (0x3ul << UART_LIN_CTL_LIN_HEAD_SEL_Pos)  /*!< UART_T::LIN_CTL: LIN_HEAD_SEL Mask           */

#define UART_LIN_CTL_LIN_BS_LEN_Pos     20                                        /*!< UART_T::LIN_CTL: LIN_BS_LEN Position       */
#define UART_LIN_CTL_LIN_BS_LEN_Msk     (0x3ul << UART_LIN_CTL_LIN_BS_LEN_Pos)    /*!< UART_T::LIN_CTL: LIN_BS_LEN Mask           */

#define UART_LIN_CTL_LIN_BKFL_Pos       16                                        /*!< UART_T::LIN_CTL: LIN_BKFL Position       */
#define UART_LIN_CTL_LIN_BKFL_Msk       (0xFul << UART_LIN_CTL_LIN_BKFL_Pos)      /*!< UART_T::LIN_CTL: LIN_BKFL Mask           */

#define UART_LIN_CTL_BIT_ERR_EN_Pos     12                                        /*!< UART_T::LIN_CTL: BIT_ERR_EN Position       */
#define UART_LIN_CTL_BIT_ERR_EN_Msk     (1ul << UART_LIN_CTL_BIT_ERR_EN_Pos)      /*!< UART_T::LIN_CTL: BIT_ERR_EN Mask           */

#define UART_LIN_CTL_LIN_RX_DIS_Pos     11                                        /*!< UART_T::LIN_CTL: LIN_RX_DIS Position       */
#define UART_LIN_CTL_LIN_RX_DIS_Msk     (1ul << UART_LIN_CTL_LIN_RX_DIS_Pos)      /*!< UART_T::LIN_CTL: LIN_RX_DIS Mask           */

#define UART_LIN_CTL_LIN_BKDET_EN_Pos   10                                        /*!< UART_T::LIN_CTL: LIN_BKDET_EN Position       */
#define UART_LIN_CTL_LIN_BKDET_EN_Msk   (1ul << UART_LIN_CTL_LIN_BKDET_EN_Pos)    /*!< UART_T::LIN_CTL: LIN_BKDET_EN Mask           */

#define UART_LIN_CTL_LIN_IDPEN_Pos      9                                         /*!< UART_T::LIN_CTL: LIN_IDPEN Position       */
#define UART_LIN_CTL_LIN_IDPEN_Msk      (1ul << UART_LIN_CTL_LIN_IDPEN_Pos)       /*!< UART_T::LIN_CTL: LIN_IDPEN Mask           */

#define UART_LIN_CTL_LIN_SHD_Pos        8                                         /*!< UART_T::LIN_CTL: LIN_SHD Position       */
#define UART_LIN_CTL_LIN_SHD_Msk        (1ul << UART_LIN_CTL_LIN_SHD_Pos)         /*!< UART_T::LIN_CTL: LIN_SHD Mask           */

#define UART_LIN_CTL_LIN_MUTE_EN_Pos    4                                          /*!< UART_T::LIN_CTL: LIN_MUTE_EN Position       */
#define UART_LIN_CTL_LIN_MUTE_EN_Msk    (1ul << UART_LIN_CTL_LIN_MUTE_EN_Pos)      /*!< UART_T::LIN_CTL: LIN_MUTE_EN Mask           */

#define UART_LIN_CTL_LINS_DUM_EN_Pos    3                                          /*!< UART_T::LIN_CTL: LINS_DUM_EN Position       */
#define UART_LIN_CTL_LINS_DUM_EN_Msk    (1ul << UART_LIN_CTL_LINS_DUM_EN_Pos)      /*!< UART_T::LIN_CTL: LINS_DUM_EN Mask           */

#define UART_LIN_CTL_LINS_ARS_EN_Pos    2                                          /*!< UART_T::LIN_CTL: LINS_ARS_EN Position       */
#define UART_LIN_CTL_LINS_ARS_EN_Msk    (1ul << UART_LIN_CTL_LINS_ARS_EN_Pos)      /*!< UART_T::LIN_CTL: LINS_ARS_EN Mask           */

#define UART_LIN_CTL_LINS_HDET_EN_Pos   1                                          /*!< UART_T::LIN_CTL: LINS_HDET_EN Position       */
#define UART_LIN_CTL_LINS_HDET_EN_Msk   (1ul << UART_LIN_CTL_LINS_HDET_EN_Pos)     /*!< UART_T::LIN_CTL: LINS_HDET_EN Mask           */

#define UART_LIN_CTL_LINS_EN_Pos        0                                          /*!< UART_T::LIN_CTL: LINS_EN Position       */
#define UART_LIN_CTL_LINS_EN_Msk        (1ul << UART_LIN_CTL_LINS_EN_Pos)          /*!< UART_T::LIN_CTL: LINS_EN Mask           */

/* UART LIN_SR Bit Field Definitions */
#define UART_LIN_SR_BIT_ERR_F_Pos       9                                           /*!< UART_T::LIN_SR: BIT_ERR_F Position         */
#define UART_LIN_SR_BIT_ERR_F_Msk       (1ul << UART_LIN_SR_BIT_ERR_F_Pos)          /*!< UART_T::LIN_SR: BIT_ERR_F Mask             */

#define UART_LIN_SR_LINS_BKDET_F_Pos    8                                           /*!< UART_T::LIN_SR: LINS_BKDET_F Position      */
#define UART_LIN_SR_LINS_BKDET_F_Msk    (1ul << UART_LIN_SR_LINS_BKDET_F_Pos)       /*!< UART_T::LIN_SR: LINS_BKDET_F Mask          */

#define UART_LIN_SR_LINS_SYNC_F_Pos     3                                           /*!< UART_T::LIN_SR: LINS_SYNC_F Position       */
#define UART_LIN_SR_LINS_SYNC_F_Msk     (1ul << UART_LIN_SR_LINS_SYNC_F_Pos)        /*!< UART_T::LIN_SR: LINS_SYNC_F Mask           */

#define UART_LIN_SR_LINS_IDPERR_F_Pos   2                                           /*!< UART_T::LIN_SR: LINS_IDPERR_F Position     */
#define UART_LIN_SR_LINS_IDPERR_F_Msk   (1ul << UART_LIN_SR_LINS_IDPERR_F_Pos)      /*!< UART_T::LIN_SR: LINS_IDPERR_F Mask         */

#define UART_LIN_SR_LINS_HERR_F_Pos     1                                           /*!< UART_T::LIN_SR: LINS_HERR_F Position       */
#define UART_LIN_SR_LINS_HERR_F_Msk     (1ul << UART_LIN_SR_LINS_HERR_F_Pos)        /*!< UART_T::LIN_SR: LINS_HERR_F Mask           */

#define UART_LIN_SR_LINS_HDET_F_Pos     0                                           /*!< UART_T::LIN_SR: LINS_HDET_F Position       */
#define UART_LIN_SR_LINS_HDET_F_Msk     (1ul << UART_LIN_SR_LINS_HDET_F_Pos)        /*!< UART_T::LIN_SR: LINS_HDET_F Mask           */

/*@}*/ /* end of group UART_CONST */
/*@}*/ /* end of group UART */
/**@}*/ /* end of REGISTER group */


#endif /* __UART_REG_H__ */
