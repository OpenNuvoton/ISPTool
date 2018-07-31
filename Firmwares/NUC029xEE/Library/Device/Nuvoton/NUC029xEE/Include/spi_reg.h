/**************************************************************************//**
 * @file     spi_reg.h
 * @version  V1.00
 * @brief    SPI register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __SPI_REG_H__
#define __SPI_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */



/*---------------------- Serial Peripheral Interface Controller -------------------------*/
/**
    @addtogroup SPI Serial Peripheral Interface Controller (SPI)
    Memory Mapped Structure for SPI Controller
@{ */



typedef struct
{


/**
 * @var SPI_T::CNTRL
 * Offset: 0x00  Control and Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GO_BUSY   |SPI Transfer Control Bit And Busy Status
 * |        |          |0 = Data transfer stopped.
 * |        |          |1 = In Master mode, writing 1 to this bit to start the SPI data transfer; in Slave mode,
 * |        |          |    writing 1 to this bit indicates that the slave is ready to communicate with a master.
 * |        |          |If FIFO mode is disabled, during the data transfer, this bit keeps the value of 1.
 * |        |          |As the transfer is finished, this bit will be cleared automatically.
 * |        |          |Software can read this bit to check if the SPI is in busy status.
 * |        |          |In FIFO mode, this bit will be controlled by hardware.
 * |        |          |Software should not modify this bit.
 * |        |          |In Slave mode, this bit always returns 1 when this register is read by software.
 * |        |          |In Master mode, this bit reflects the busy or idle status of SPI.
 * |        |          |Note:
 * |        |          |1. When FIFO mode is disabled, all configurations should be set before writing 1 to this GO_BUSY
 * |        |          |bit.
 * |        |          |2. When FIFO mode is disabled and the software uses TX or RX PDMA function to transfer data,
 * |        |          |this bit
 * |        |          |   will be cleared after the PDMA finishes the data transfer.
 * |[1]     |RX_NEG    |Receive On Negative Edge
 * |        |          |0 = Received data input signal is latched on the rising edge of SPI bus clock.
 * |        |          |1 = Received data input signal is latched on the falling edge of SPI bus clock.
 * |[2]     |TX_NEG    |Transmit On Negative Edge
 * |        |          |0 = Transmitted data output signal is changed on the rising edge of SPI bus clock.
 * |        |          |1 = Transmitted data output signal is changed on the falling edge of SPI bus clock.
 * |[7:3]   |TX_BIT_LEN|Transmit Bit Length
 * |        |          |This field specifies how many bits can be transmitted / received in one transaction.
 * |        |          |The minimum bit length is 8 bits and can up to 32 bits.
 * |        |          |TX_BIT_LEN = 0x08 ... 8 bits.
 * |        |          |TX_BIT_LEN = 0x09 ... 9 bits.
 * |        |          |......
 * |        |          |TX_BIT_LEN = 0x1F ... 31 bits.
 * |        |          |TX_BIT_LEN = 0x00 ... 32 bits.
 * |[10]    |LSB       |Send LSB First
 * |        |          |0 = The MSB, which bit of transmit/receive register depends on the setting of TX_BIT_LEN, is
 * |        |          |transmitted/received first.
 * |        |          |1 = The LSB, bit 0 of the SPI TX0/1 register, is sent first to the SPI data output pin, and the
 * |        |          |first bit received from
 * |        |          | the SPI data input pin will be put in the LSB position of the RX register (bit 0 of SPI_RX0/1).
 * |[11]    |CLKP      |Clock Polarity
 * |        |          |0 = SPI bus clock is idle low.
 * |        |          |1 = SPI bus clock is idle high.
 * |[15:12] |SP_CYCLE  |Suspend Interval (Master Only)
 * |        |          |The four bits provide configurable suspend interval between two successive transmit/receive
 * |        |          |transaction in a transfer.
 * |        |          |The definition of the suspend interval is the interval between the last clock edge of the
 * |        |          |preceding transaction word
 * |        |          |and the first clock edge of the following transaction word.
 * |        |          |The default value is 0x3.
 * |        |          |The period of the suspend interval is obtained according to the following equation.
 * |        |          |(SP_CYCLE[3:0] + 0.5) * period of SPI bus clock cycle
 * |        |          |Example:
 * |        |          |SP_CYCLE = 0x0 ... 0.5 SPI bus clock cycle.
 * |        |          |SP_CYCLE = 0x1 ... 1.5 SPI bus clock cycle.
 * |        |          |......
 * |        |          |SP_CYCLE = 0xE ... 14.5 SPI bus clock cycle.
 * |        |          |SP_CYCLE = 0xF ... 15.5 SPI bus clock cycle.
 * |        |          |If the variable clock function is enabled and the transmit FIFO buffer is not empty, the minimum
 * |        |          |period of suspend
 * |        |          |interval between the successive transactions is (6.5 + SP_CYCLE) * SPI bus clock cycle.
 * |[16]    |IF        |Unit Transfer Interrupt Flag
 * |        |          |0 = No transaction has been finished since this bit was cleared to 0.
 * |        |          |1 = SPI controller has finished one unit transfer.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[17]    |IE        |Unit Transfer Interrupt Enable
 * |        |          |0 = SPI unit transfer interrupt Disabled.
 * |        |          |1 = SPI unit transfer interrupt Enabled.
 * |[18]    |SLAVE     |Slave Mode Enable
 * |        |          |0 = Master mode.
 * |        |          |1 = Slave mode.
 * |[19]    |REORDER   |Byte Reorder Function Enable
 * |        |          |0 = Byte Reorder function Disabled.
 * |        |          |1 = Byte Reorder function Enabled.
 * |        |          |A byte suspend interval will be inserted among each byte.
 * |        |          |The period of the byte suspend interval depends on the setting of SP_CYCLE.
 * |        |          |Note:
 * |        |          |1. Byte Reorder function is only available if TX_BIT_LEN is defined as 16, 24, and 32 bits.
 * |        |          |2. In Slave mode with level-trigger configuration, the slave select pin must be kept at active
 * |        |          |state during the
 * |        |          |   byte suspend interval.
 * |        |          |3. The Byte Reorder function is not supported when the variable bus clock function or Dual I/O
 * |        |          |mode is enabled.
 * |[21]    |FIFO      |FIFO Mode Enable
 * |        |          |0 = FIFO mode Disabled.
 * |        |          |1 = FIFO mode Enabled.
 * |        |          |Note:
 * |        |          |1. Before enabling FIFO mode, the other related settings should be set in advance.
 * |        |          |2. In Master mode, if the FIFO mode is enabled, the GO_BUSY bit will be set to 1 automatically
 * |        |          |after writing data
 * |        |          | to the transmit FIFO buffer; the GO_BUSY bit will be cleared to 0 automatically when the SPI
 * |        |          |controller is in idle.
 * |        |          | If all data stored at transmit FIFO buffer are sent out, the TX_EMPTY bit will be set to 1 and
 * |        |          |the GO_BUSY bit will be cleared to 0.
 * |        |          |3. After clearing this bit to 0, user must wait for at least 2 peripheral clock periods before
 * |        |          |setting this bit to 1 again.
 * |[23]    |VARCLK_EN |Variable Clock Enable (Master Only)
 * |        |          |0 = SPI clock output frequency is fixed and decided only by the value of DIVIDER.
 * |        |          |1 = SPI clock output frequency is variable.
 * |        |          |The output frequency is decided by the value of VARCLK, DIVIDER, and DIVIDER2.
 * |        |          |Note: When this VARCLK_EN bit is set to 1, the setting of TX_BIT_LEN must be programmed as 0x10
 * |        |          |(16-bit mode).
 * |[24]    |RX_EMPTY  |Receive FIFO Buffer Empty Indicator (Read Only)
 * |        |          |It is a mutual mirror bit of SPI_STATUS[24].
 * |        |          |0 = Receive FIFO buffer is not empty.
 * |        |          |1 = Receive FIFO buffer is empty.
 * |[25]    |RX_FULL   |Receive FIFO Buffer Full Indicator (Read Only)
 * |        |          |It is a mutual mirror bit of SPI_STATUS[25].
 * |        |          |0 = Receive FIFO buffer is not full.
 * |        |          |1 = Receive FIFO buffer is full.
 * |[26]    |TX_EMPTY  |Transmit FIFO Buffer Empty Indicator (Read Only)
 * |        |          |It is a mutual mirror bit of SPI_STATUS[26].
 * |        |          |0 = Transmit FIFO buffer is not empty.
 * |        |          |1 = Transmit FIFO buffer is empty.
 * |[27]    |TX_FULL   |Transmit FIFO Buffer Full Indicator (Read Only)
 * |        |          |It is a mutual mirror bit of SPI_STATUS[27].
 * |        |          |0 = Transmit FIFO buffer is not full.
 * |        |          |1 = Transmit FIFO buffer is full.
 * @var SPI_T::DIVIDER
 * Offset: 0x04  Clock Divider Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DIVIDER   |Clock Divider 1 Register
 * |        |          |The value in this field is the frequency divider for generating the SPI peripheral clock and the
 * |        |          |SPI bus clock of SPI master.
 * |        |          |The frequency is obtained according to the following equation.
 * |        |          |If the bit of BCn, SPI_CNTRL2[31], is set to 0,
 * |        |          |   SPI peripheral clock frequency = system clock frequency / (DIVIDER + 1) / 2
 * |        |          |else if BCn is set to 1,
 * |        |          |   SPI peripheral clock frequency = SPI peripheral clock source frequency / (DIVIDER + 1)
 * |        |          |The SPI peripheral clock source is defined in the CLKSEL1 register.
 * |[23:16] |DIVIDER2  |Clock Divider 2 Register (Master Only)
 * |        |          |The value in this field is the 2nd frequency divider for generating the second clock of the
 * |        |          |variable clock function.
 * |        |          |The frequency is obtained according to the following equation:
 * |        |          |   f_clk2 = SPI peripheral clock frequency / (DIVIDER2 + 1) / 2
 * |        |          |If the VARCLK_EN bit is cleared to 0, this setting is unmeaning.
 * @var SPI_T::SSR
 * Offset: 0x08  Slave Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SSR       |Slave Select Control Bits (Master Only)
 * |        |          |If AUTOSS bit is cleared, writing 1 to bit of this field sets the proper SPIn_SS
 * |        |          |line to an active state and writing 0 sets the line back to inactive state.
 * |        |          |If the AUTOSS bit is set, writing 0 to bit location of this field will keep the
 * |        |          |corresponding
 * |        |          |SPIn_SS line at inactive state; writing 1 to bit location of this field will select
 * |        |          |appropriate SPIn_SS line to be automatically driven to active state for the duration of
 * |        |          |the
 * |        |          |transmit/receive, and will be driven to inactive state for the rest of the time.
 * |        |          |The active state of SPIn_SS is specified in SS_LVL.
 * |        |          |Note: SPIn_SS is defined as the slave select input in Slave mode.
 * |[2]     |SS_LVL    |Slave Select Active Level
 * |        |          |This bit defines the active status of slave select signal (SPIn_SS).
 * |        |          |0 = The slave select signal SPIn_SS is active on low-level/falling-edge.
 * |        |          |1 = The slave select signal SPIn_SS is active on high-level/rising-edge.
 * |[3]     |AUTOSS    |Automatic Slave Select Function Enable (Master Only)
 * |        |          |0 = If this bit is cleared, slave select signals will be asserted/de-asserted by setting
 * |        |          |/clearing
 * |        |          |    the corresponding bits of SPI_SSR[0].
 * |        |          |1 = If this bit is set, SPIn_SS signal will be generated automatically.
 * |        |          | It means that device/slave select signal, which is set in SPI_SSR[0], will be asserted by the
 * |        |          | SPI controller when transmit/receive is started, and will be de-asserted after each
 * |        |          |transmit/receive is finished.
 * |[4]     |SS_LTRIG  |Slave Select Level Trigger Enable (Slave Only)
 * |        |          |0 = Slave select signal is edge-trigger.
 * |        |          |    This is the default value.
 * |        |          |    The SS_LVL bit decides the signal is active after a falling-edge or rising-edge.
 * |        |          |1 = Slave select signal is level-trigger.
 * |        |          |    The SS_LVL bit decides the signal is active low or active high.
 * |[5]     |LTRIG_FLAG|Level Trigger Accomplish Flag
 * |        |          |In Slave mode, this bit indicates whether the received bit number meets the requirement or not
 * |        |          |after the current transaction done.
 * |        |          |0 = Transferred bit length of one transaction does not meet the specified requirement.
 * |        |          |1 = Transferred bit length meets the specified requirement which defined in TX_BIT_LEN.
 * |        |          |Note: This bit is READ only.
 * |        |          |As the GO_BUSY bit is set to 1 by software, the LTRIG_FLAG will be cleared to 0 after 4 SPI
 * |        |          |peripheral clock periods plus 1 system clock period.
 * |        |          |In FIFO mode, this bit has no meaning.
 * @var SPI_T::RX
 * Offset: 0x10  Data Receive Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |RX        |Data Receive Register
 * |        |          |The data receive register holds the datum received from SPI data input pin.
 * |        |          |If FIFO mode is disabled, the last received data can be accessed through software by reading
 * |        |          |this register.
 * |        |          |If the FIFO bit is set as 1 and the RX_EMPTY bit, SPI_CNTRL[24] or SPI_STATUS[24], is not set to
 * |        |          |1, the receive
 * |        |          |FIFO buffer can be accessed through software by reading this register. This is a read-only
 * |        |          |register.
 * @var SPI_T::TX
 * Offset: 0x20  Data Transmit Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |TX        |Data Transmit Register
 * |        |          |The data transmit registers hold the data to be transmitted in the next transfer.
 * |        |          |The number of valid bits depends on the setting of transmit bit length field of the SPI_CNTRL
 * |        |          |register.
 * |        |          |For example, if TX_BIT_LEN is set to 0x08, the bits TX[7:0] will be transmitted in next
 * |        |          |transfer.
 * |        |          |If TX_BIT_LEN is set to 0x00, the SPI controller will perform a 32-bit transfer.
 * |        |          |Note 1: When the SPI controller is configured as a slave device and FIFO mode is disabled, if
 * |        |          |the SPI
 * |        |          | controller attempts to transmit data to a master, the transmit data register should be updated
 * |        |          |        by software before setting the GO_BUSY bit to 1.
 * |        |          |Note 2: In Master mode, SPI controller will start to transfer after 5 peripheral clock cycles
 * |        |          |after user writes to this register.
 * @var SPI_T::VARCLK
 * Offset: 0x34  Variable Clock Pattern Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |VARCLK    |Variable Clock Pattern
 * |        |          |This register defines the clock pattern of the SPI transfer.
 * |        |          |If the variable clock function is disabled, this setting is unmeaning.
 * @var SPI_T::DMA
 * Offset: 0x38  SPI DMA Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TX_DMA_GO |Transmit DMA Start
 * |        |          |Setting this bit to 1 will start the transmit PDMA process.
 * |        |          |SPI controller will issue request to PDMA controller automatically.
 * |        |          |Hardware will clear this bit to 0 automatically after PDMA transfer done.
 * |        |          |If the SPI transmit PDMA function is used to transfer data, the GO_BUSY bit should not be set to
 * |        |          |1 by software.
 * |        |          |The PDMA control logic of SPI controller will set it automatically whenever necessary.
 * |        |          |In Slave mode and when FIFO mode is disabled, the minimal suspend interval between two
 * |        |          |successive transactions
 * |        |          |must be larger than (8 SPI clock periods + 14 APB clock periods) for edge-trigger mode or
 * |        |          |(9.5 SPI clock periods + 14 APB clock periods) for level-trigger mode.
 * |        |          |If the 2-bit Transfer mode is enabled, additional 18 APB clock periods for the above conditions
 * |        |          |is required.
 * |[1]     |RX_DMA_GO |Receive DMA Start
 * |        |          |Setting this bit to 1 will start the receive PDMA process.
 * |        |          |The SPI controller will issue request to PDMA controller automatically when the SPI receive
 * |        |          |buffer is not empty.
 * |        |          |This bit will be cleared to 0 by hardware automatically after PDMA transfer is done.
 * |        |          |If the software uses the receive PDMA function to access the received data of SPI and does not
 * |        |          |use the transmit
 * |        |          |PDMA function, the GO_BUSY bit should be set by software.
 * |        |          |Enabling FIFO mode is recommended if the software uses more than one PDMA channel to transfer
 * |        |          |data.
 * |        |          |In Slave mode and when FIFO mode is disabled, if the software only uses one PDMA channel for SPI
 * |        |          |receive PDMA
 * |        |          |function and the other PDMA channels are not in use, the minimal suspend interval between two
 * |        |          |successive
 * |        |          |transactions must be larger than (9 SPI slave peripheral clock periods + 4 APB clock periods)
 * |        |          |for Edge-trigger
 * |        |          |mode or (9.5 SPI slave peripheral clock periods + 4 APB clock periods) for Level-trigger mode.
 * |[2]     |PDMA_RST  |PDMA Reset
 * |        |          |0 = No effect.
 * |        |          |1 = Reset the PDMA control logic of the SPI controller. This bit will be cleared to 0
 * |        |          |automatically.
 * @var SPI_T::CNTRL2
 * Offset: 0x3C  Control and Status Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8]     |NOSLVSEL  |Slave 3-Wire Mode Enable
 * |        |          |This is used to ignore the slave select signal in Slave mode.
 * |        |          |The SPI controller can work with 3-wire interface including SPIn_CLK, SPIn_MISO, and SPIn_MOSI.
 * |        |          |0 = 4-wire bi-direction interface.
 * |        |          |1 = 3-wire bi-direction interface.
 * |        |          |Note: In Slave 3-wire mode, the SS_LTRIG, SPI_SSR[4] will be set as 1 automatically.
 * |[9]     |SLV_ABORT |Slave 3-Wire Mode Abort Control
 * |        |          |In normal operation, there is an interrupt event when the received data meet the required bits
 * |        |          |which defined in TX_BIT_LEN.
 * |        |          |If the received bits are less than the requirement and there is no more SPI clock input over the
 * |        |          |one transfer time in
 * |        |          |Slave 3-wire mode, the user can set this bit to force the current transfer done and then the
 * |        |          |user can get a transfer done interrupt event.
 * |        |          |Note: This bit will be cleared to 0 automatically by hardware after it is set to 1 by software.
 * |[10]    |SSTA_INTEN|Slave 3-Wire Mode Start Interrupt Enable
 * |        |          |Used to enable interrupt when the transfer has started in Slave 3-wire mode.
 * |        |          |If there is no transfer done interrupt over the time period which is defined by user after the
 * |        |          |transfer start,
 * |        |          |the user can set the SLV_ABORT bit to force the transfer done.
 * |        |          |0 = Transaction start interrupt Disabled.
 * |        |          |1 = Transaction start interrupt Enabled.
 * |        |          |It will be cleared to 0 as the current transfer is done or the SLV_START_INTSTS bit is cleared.
 * |[11]    |SLV_START_INTSTS|Slave 3-Wire Mode Start Interrupt Status
 * |        |          |This bit indicates if a transaction has started in Slave 3-wire mode.
 * |        |          |It is a mutual mirror bit of SPI_STATUS[11].
 * |        |          |0 = Slave has not detected any SPI clock transition since the SSTA_INTEN bit was set to 1.
 * |        |          |1 = A transaction has started in Slave 3-wire mode.
 * |        |          |It will be cleared automatically when a transaction is done or by writing 1 to this bit.
 * |[12]    |DUAL_IO_DIR|Dual I/O Mode Direction Control
 * |        |          |0 = Dual Input mode.
 * |        |          |1 = Dual Output mode.
 * |[13]    |DUAL_IO_EN|Dual I/O Mode Enable
 * |        |          |0 = Dual I/O mode Disabled.
 * |        |          |1 = Dual I/O mode Enabled.
 * |[16]    |SS_INT_OPT|Slave Select Inactive Interrupt Option
 * |        |          |This setting is only available if the SPI controller is configured as level trigger slave
 * |        |          |device.
 * |        |          |0 = As the slave select signal goes to inactive level, the IF bit will NOT be set to 1.
 * |        |          |1 = As the slave select signal goes to inactive level, the IF bit will be set to 1.
 * |[31]    |BCn       |SPI Peripheral Clock Backward Compatible Option
 * |        |          |0 = Backward compatible clock configuration.
 * |        |          |1 = Clock configuration is not backward compatible.
 * |        |          |Refer to the description of SPI_DIVIDER register for details.
 * @var SPI_T::FIFO_CTL
 * Offset: 0x40  SPI FIFO Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RX_CLR    |Clear Receive FIFO Buffer
 * |        |          |0 = No effect.
 * |        |          |1 = Clear receive FIFO buffer.
 * |        |          |The RX_FULL flag will be cleared to 0 and the RX_EMPTY flag will be set to 1.
 * |        |          |This bit will be cleared to 0 by hardware after it is set to 1 by software.
 * |[1]     |TX_CLR    |Clear Transmit FIFO Buffer
 * |        |          |0 = No effect.
 * |        |          |1 = Clear transmit FIFO buffer.
 * |        |          |The TX_FULL flag will be cleared to 0 and the TX_EMPTY flag will be set to 1.
 * |        |          |This bit will be cleared to 0 by hardware after it is set to 1 by software.
 * |[2]     |RX_INTEN  |Receive Threshold Interrupt Enable
 * |        |          |0 = RX threshold interrupt Disabled.
 * |        |          |1 = RX threshold interrupt Enabled.
 * |[3]     |TX_INTEN  |Transmit Threshold Interrupt Enable
 * |        |          |0 = TX threshold interrupt Disabled.
 * |        |          |1 = TX threshold interrupt Enabled.
 * |[6]     |RXOV_INTEN|Receive FIFO Overrun Interrupt Enable
 * |        |          |0 = Receive FIFO overrun interrupt Disabled.
 * |        |          |1 = Receive FIFO overrun interrupt Enabled.
 * |[21]    |TIMEOUT_INTEN|Receive FIFO Time-Out Interrupt Enable
 * |        |          |0 = Time-out interrupt Disabled.
 * |        |          |1 = Time-out interrupt Enabled.
 * |[26:24] |RX_THRESHOLD|Receive FIFO Threshold
 * |        |          |If the valid data count of the receive FIFO buffer is larger than the RX_THRESHOLD setting,
 * |        |          |the RX_INTSTS bit will be set to 1, else the RX_INTSTS bit will be cleared to 0.
 * |[30:28] |TX_THRESHOLD|Transmit FIFO Threshold
 * |        |          |If the valid data count of the transmit FIFO buffer is less than or equal to the TX_THRESHOLD
 * |        |          |setting, the TX_INTSTS bit will be set to 1, else the TX_INTSTS bit will be cleared to 0.
 * @var SPI_T::STATUS
 * Offset: 0x44  SPI Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RX_INTSTS |Receive FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count within the Rx FIFO buffer is smaller than or equal to the setting value
 * |        |          |of RX_THRESHOLD.
 * |        |          |1 = The valid data count within the receive FIFO buffer is larger than the setting value of
 * |        |          |RX_THRESHOLD.
 * |        |          |Note: If RX_INTEN = 1 and RX_INTSTS = 1, the SPI controller will generate a SPI interrupt
 * |        |          |request.
 * |[2]     |RX_OVERRUN|Receive FIFO Overrun Status
 * |        |          |When the receive FIFO buffer is full, the follow-up data will be dropped and this bit will be
 * |        |          |set to 1.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[4]     |TX_INTSTS |Transmit FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count within the transmit FIFO buffer is larger than the setting value of
 * |        |          |TX_THRESHOLD.
 * |        |          |1 = The valid data count within the transmit FIFO buffer is less than or equal to the setting
 * |        |          |value of TX_THRESHOLD.
 * |        |          |Note: If TX_INTEN = 1 and TX_INTSTS = 1, the SPI controller will generate a SPI interrupt
 * |        |          |request.
 * |[11]    |SLV_START_INTSTS|Slave Start Interrupt Status
 * |        |          |It is used to dedicate if a transaction has started in Slave 3-wire mode.
 * |        |          |It is a mutual mirror bit of SPI_CNTRL2[11].
 * |        |          |0 = Slave has not detected any SPI clock transition since the SSTA_INTEN bit was set to 1.
 * |        |          |1 = A transaction has started in Slave 3-wire mode.
 * |        |          |It will be cleared as a transaction is done or by writing 1 to this bit.
 * |[15:12] |RX_FIFO_COUNT|Receive FIFO Data Count (Read Only)
 * |        |          |This bit field indicates the valid data count of receive FIFO buffer.
 * |[16]    |IF        |SPI Unit Transfer Interrupt Flag
 * |        |          |It is a mutual mirror bit of SPI_CNTRL[16].
 * |        |          |0 = No transaction has been finished since this bit was cleared to 0.
 * |        |          |1 = SPI controller has finished one unit transfer.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[20]    |TIMEOUT   |Time-Out Interrupt Flag
 * |        |          |0 = No receive FIFO time-out event.
 * |        |          |1 = Receive FIFO buffer is not empty and no read operation on receive FIFO buffer over 64 SPI
 * |        |          |clock
 * |        |          |period in Master mode or over 576 SPI peripheral clock period in Slave mode.
 * |        |          |When the received FIFO buffer is read by software, the time-out status will be cleared
 * |        |          |automatically.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[24]    |RX_EMPTY  |Receive FIFO Buffer Empty Indicator (Read Only)
 * |        |          |It is a mutual mirror bit of SPI_CNTRL[24].
 * |        |          |0 = Receive FIFO buffer is not empty.
 * |        |          |1 = Receive FIFO buffer is empty.
 * |[25]    |RX_FULL   |Receive FIFO Buffer Empty Indicator (Read Only)
 * |        |          |It is a mutual mirror bit of SPI_CNTRL[25].
 * |        |          |0 = Receive FIFO buffer is not empty.
 * |        |          |1 = Receive FIFO buffer is empty.
 * |[26]    |TX_EMPTY  |Transmit FIFO Buffer Empty Indicator (Read Only)
 * |        |          |It is a mutual mirror bit of SPI_CNTRL[26].
 * |        |          |0 = Transmit FIFO buffer is not empty.
 * |        |          |1 = Transmit FIFO buffer is empty.
 * |[27]    |TX_FULL   |Transmit FIFO Buffer Full Indicator (Read Only)
 * |        |          |It is a mutual mirror bit of SPI_CNTRL[27].
 * |        |          |0 = Transmit FIFO buffer is not full.
 * |        |          |1 = Transmit FIFO buffer is full.
 * |[31:28] |TX_FIFO_COUNT|Transmit FIFO Data Count (Read Only)
 * |        |          |This bit field indicates the valid data count of transmit FIFO buffer.
 */

    __IO uint32_t CNTRL;         /* Offset: 0x00  Control and Status Register                                        */
    __IO uint32_t DIVIDER;       /* Offset: 0x04  Clock Divider Register                                             */
    __IO uint32_t SSR;           /* Offset: 0x08  Slave Select Register                                              */
    __I  uint32_t RESERVE0;     
    __I  uint32_t RX;            /* Offset: 0x10  Data Receive Register                                              */
    __I  uint32_t RESERVE1;     
    __I  uint32_t RESERVE2;     
    __I  uint32_t RESERVE3;     
    __O  uint32_t TX;            /* Offset: 0x20  Data Transmit Register                                             */
    __I  uint32_t RESERVE4;     
    __I  uint32_t RESERVE5;     
    __I  uint32_t RESERVE6;     
    __I  uint32_t RESERVE7;     
    __IO uint32_t VARCLK;        /* Offset: 0x34  Variable Clock Pattern Register                                    */
    __IO uint32_t DMA;           /* Offset: 0x38  SPI DMA Control Register                                           */
    __IO uint32_t CNTRL2;        /* Offset: 0x3C  Control and Status Register 2                                      */
    __IO uint32_t FIFO_CTL;      /* Offset: 0x40  SPI FIFO Control Register                                          */
    __IO uint32_t STATUS;        /* Offset: 0x44  SPI Status Register                                                */

} SPI_T;



/**
    @addtogroup SPI_CONST SPI Bit Field Definition
    Constant Definitions for SPI Controller
@{ */


/* SPI_CNTRL Bit Field Definitions */
#define SPI_CNTRL_TX_FULL_Pos      27                                     /*!< SPI_T::CNTRL: TX_FULL Position */
#define SPI_CNTRL_TX_FULL_Msk      (1ul << SPI_CNTRL_TX_FULL_Pos)         /*!< SPI_T::CNTRL: TX_FULL Mask     */

#define SPI_CNTRL_TX_EMPTY_Pos     26                                     /*!< SPI_T::CNTRL: TX_EMPTY Position */
#define SPI_CNTRL_TX_EMPTY_Msk     (1ul << SPI_CNTRL_TX_EMPTY_Pos)        /*!< SPI_T::CNTRL: TX_EMPTY Mask     */

#define SPI_CNTRL_RX_FULL_Pos      25                                     /*!< SPI_T::CNTRL: RX_FULL Position */
#define SPI_CNTRL_RX_FULL_Msk      (1ul << SPI_CNTRL_RX_FULL_Pos)         /*!< SPI_T::CNTRL: RX_FULL Mask     */

#define SPI_CNTRL_RX_EMPTY_Pos     24                                     /*!< SPI_T::CNTRL: RX_EMPTY Position */
#define SPI_CNTRL_RX_EMPTY_Msk     (1ul << SPI_CNTRL_RX_EMPTY_Pos)        /*!< SPI_T::CNTRL: RX_EMPTY Mask     */

#define SPI_CNTRL_VARCLK_EN_Pos    23                                     /*!< SPI_T::CNTRL: VARCLK_EN Position */
#define SPI_CNTRL_VARCLK_EN_Msk    (1ul << SPI_CNTRL_VARCLK_EN_Pos)       /*!< SPI_T::CNTRL: VARCLK_EN Mask     */

#define SPI_CNTRL_FIFO_Pos         21                                     /*!< SPI_T::CNTRL: FIFO Position */
#define SPI_CNTRL_FIFO_Msk         (1ul << SPI_CNTRL_FIFO_Pos)            /*!< SPI_T::CNTRL: FIFO Mask     */

#define SPI_CNTRL_REORDER_Pos      19                                     /*!< SPI_T::CNTRL: REORDER Position */
#define SPI_CNTRL_REORDER_Msk      (1ul << SPI_CNTRL_REORDER_Pos)         /*!< SPI_T::CNTRL: REORDER Mask     */

#define SPI_CNTRL_SLAVE_Pos        18                                     /*!< SPI_T::CNTRL: SLAVE Position */
#define SPI_CNTRL_SLAVE_Msk        (1ul << SPI_CNTRL_SLAVE_Pos)           /*!< SPI_T::CNTRL: SLAVE Mask     */

#define SPI_CNTRL_IE_Pos           17                                     /*!< SPI_T::CNTRL: IE Position */
#define SPI_CNTRL_IE_Msk           (1ul << SPI_CNTRL_IE_Pos)              /*!< SPI_T::CNTRL: IE Mask     */

#define SPI_CNTRL_IF_Pos           16                                     /*!< SPI_T::CNTRL: IF Position */
#define SPI_CNTRL_IF_Msk           (1ul << SPI_CNTRL_IF_Pos)              /*!< SPI_T::CNTRL: IF Mask     */

#define SPI_CNTRL_SP_CYCLE_Pos     12                                     /*!< SPI_T::CNTRL: SP_CYCLE Position */
#define SPI_CNTRL_SP_CYCLE_Msk     (0xFul << SPI_CNTRL_SP_CYCLE_Pos)      /*!< SPI_T::CNTRL: SP_CYCLE Mask     */

#define SPI_CNTRL_CLKP_Pos         11                                     /*!< SPI_T::CNTRL: CLKP Position */
#define SPI_CNTRL_CLKP_Msk         (1ul << SPI_CNTRL_CLKP_Pos)            /*!< SPI_T::CNTRL: CLKP Mask     */

#define SPI_CNTRL_LSB_Pos          10                                     /*!< SPI_T::CNTRL: LSB Position */
#define SPI_CNTRL_LSB_Msk          (1ul << SPI_CNTRL_LSB_Pos)             /*!< SPI_T::CNTRL: LSB Mask     */

#define SPI_CNTRL_TX_BIT_LEN_Pos   3                                      /*!< SPI_T::CNTRL: TX_BIT_LEN Position */
#define SPI_CNTRL_TX_BIT_LEN_Msk   (0x1Ful << SPI_CNTRL_TX_BIT_LEN_Pos)   /*!< SPI_T::CNTRL: TX_BIT_LEN Mask     */

#define SPI_CNTRL_TX_NEG_Pos       2                                      /*!< SPI_T::CNTRL: TX_NEG Position */
#define SPI_CNTRL_TX_NEG_Msk       (1ul << SPI_CNTRL_TX_NEG_Pos)          /*!< SPI_T::CNTRL: TX_NEG Mask     */

#define SPI_CNTRL_RX_NEG_Pos       1                                      /*!< SPI_T::CNTRL: RX_NEG Position */
#define SPI_CNTRL_RX_NEG_Msk       (1ul << SPI_CNTRL_RX_NEG_Pos)          /*!< SPI_T::CNTRL: RX_NEG Mask     */

#define SPI_CNTRL_GO_BUSY_Pos      0                                      /*!< SPI_T::CNTRL: GO_BUSY Position */
#define SPI_CNTRL_GO_BUSY_Msk      (1ul << SPI_CNTRL_GO_BUSY_Pos)         /*!< SPI_T::CNTRL: GO_BUSY Mask     */

/* SPI_DIVIDER Bit Field Definitions */
#define SPI_DIVIDER_DIVIDER2_Pos   16                                     /*!< SPI_T::DIVIDER: DIVIDER2 Position */
#define SPI_DIVIDER_DIVIDER2_Msk   (0xFFul << SPI_DIVIDER_DIVIDER2_Pos)   /*!< SPI_T::DIVIDER: DIVIDER2 Mask */

#define SPI_DIVIDER_DIVIDER_Pos    0                                      /*!< SPI_T::DIVIDER: DIVIDER Position */
#define SPI_DIVIDER_DIVIDER_Msk    (0xFFul << SPI_DIVIDER_DIVIDER_Pos)    /*!< SPI_T::DIVIDER: DIVIDER Mask */

/* SPI_SSR Bit Field Definitions */
#define SPI_SSR_LTRIG_FLAG_Pos     5                                 /*!< SPI_T::SSR: LTRIG_FLAG Position */
#define SPI_SSR_LTRIG_FLAG_Msk     (1ul << SPI_SSR_LTRIG_FLAG_Pos)   /*!< SPI_T::SSR: LTRIG_FLAG Mask */

#define SPI_SSR_SS_LTRIG_Pos       4                                 /*!< SPI_T::SSR: SS_LTRIG Position */
#define SPI_SSR_SS_LTRIG_Msk       (1ul << SPI_SSR_SS_LTRIG_Pos)     /*!< SPI_T::SSR: SS_LTRIG Mask */

#define SPI_SSR_AUTOSS_Pos         3                                 /*!< SPI_T::SSR: AUTOSS Position */
#define SPI_SSR_AUTOSS_Msk         (1ul << SPI_SSR_AUTOSS_Pos)       /*!< SPI_T::SSR: AUTOSS Mask */

#define SPI_SSR_SS_LVL_Pos         2                                 /*!< SPI_T::SSR: SS_LVL Position */
#define SPI_SSR_SS_LVL_Msk         (1ul << SPI_SSR_SS_LVL_Pos)       /*!< SPI_T::SSR: SS_LVL Mask */

#define SPI_SSR_SSR_Pos            0                                 /*!< SPI_T::SSR: SSR Position */
#define SPI_SSR_SSR_Msk            (1ul << SPI_SSR_SSR_Pos)          /*!< SPI_T::SSR: SSR Mask */

/* SPI_DMA Bit Field Definitions */
#define SPI_DMA_PDMA_RST_Pos   2                                     /*!< SPI_T::DMA: PDMA_RST Position */
#define SPI_DMA_PDMA_RST_Msk   (1ul << SPI_DMA_PDMA_RST_Pos)         /*!< SPI_T::DMA: PDMA_RST Mask */

#define SPI_DMA_RX_DMA_GO_Pos   1                                    /*!< SPI_T::DMA: RX_DMA_GO Position */
#define SPI_DMA_RX_DMA_GO_Msk   (1ul << SPI_DMA_RX_DMA_GO_Pos)       /*!< SPI_T::DMA: RX_DMA_GO Mask */

#define SPI_DMA_TX_DMA_GO_Pos   0                                    /*!< SPI_T::DMA: TX_DMA_GO Position */
#define SPI_DMA_TX_DMA_GO_Msk   (1ul << SPI_DMA_TX_DMA_GO_Pos)       /*!< SPI_T::DMA: TX_DMA_GO Mask */

/* SPI_CNTRL2 Bit Field Definitions */
#define SPI_CNTRL2_BCn_Pos   31                                                      /*!< SPI_T::CNTRL2: BCn Position */
#define SPI_CNTRL2_BCn_Msk   (1ul << SPI_CNTRL2_BCn_Pos)                             /*!< SPI_T::CNTRL2: BCn Mask */

#define SPI_CNTRL2_SS_INT_OPT_Pos   16                                               /*!< SPI_T::CNTRL2: SS_INT_OPT Position */
#define SPI_CNTRL2_SS_INT_OPT_Msk   (1ul << SPI_CNTRL2_SS_INT_OPT_Pos)               /*!< SPI_T::CNTRL2: SS_INT_OPT Mask */

#define SPI_CNTRL2_DUAL_IO_EN_Pos   13                                               /*!< SPI_T::CNTRL2: DUAL_IO_EN Position */
#define SPI_CNTRL2_DUAL_IO_EN_Msk   (1ul << SPI_CNTRL2_DUAL_IO_EN_Pos)               /*!< SPI_T::CNTRL2: DUAL_IO_EN Mask */

#define SPI_CNTRL2_DUAL_IO_DIR_Pos   12                                              /*!< SPI_T::CNTRL2: DUAL_IO_DIR Position */
#define SPI_CNTRL2_DUAL_IO_DIR_Msk   (1ul << SPI_CNTRL2_DUAL_IO_DIR_Pos)             /*!< SPI_T::CNTRL2: DUAL_IO_DIR Mask */

#define SPI_CNTRL2_SLV_START_INTSTS_Pos   11                                         /*!< SPI_T::CNTRL2: SLV_START_INTSTS Position */
#define SPI_CNTRL2_SLV_START_INTSTS_Msk   (1ul << SPI_CNTRL2_SLV_START_INTSTS_Pos)   /*!< SPI_T::CNTRL2: SLV_START_INTSTS Mask */

#define SPI_CNTRL2_SSTA_INTEN_Pos   10                                               /*!< SPI_T::CNTRL2: SSTA_INTEN Position */
#define SPI_CNTRL2_SSTA_INTEN_Msk   (1ul << SPI_CNTRL2_SSTA_INTEN_Pos)               /*!< SPI_T::CNTRL2: SSTA_INTEN Mask */

#define SPI_CNTRL2_SLV_ABORT_Pos    9                                                /*!< SPI_T::CNTRL2: SLV_ABORT Position */
#define SPI_CNTRL2_SLV_ABORT_Msk    (1ul << SPI_CNTRL2_SLV_ABORT_Pos)                /*!< SPI_T::CNTRL2: SLV_ABORT Mask */

#define SPI_CNTRL2_NOSLVSEL_Pos     8                                                /*!< SPI_T::CNTRL2: NOSLVSEL Position */
#define SPI_CNTRL2_NOSLVSEL_Msk     (1ul << SPI_CNTRL2_NOSLVSEL_Pos)                 /*!< SPI_T::CNTRL2: NOSLVSEL Mask */

/* SPI_FIFO_CTL Bit Field Definitions */
#define SPI_FIFO_CTL_TX_THRESHOLD_Pos   28                                         /*!< SPI_T::FIFO_CTL: TX_THRESHOLD Position */
#define SPI_FIFO_CTL_TX_THRESHOLD_Msk   (7ul << SPI_FIFO_CTL_TX_THRESHOLD_Pos)     /*!< SPI_T::FIFO_CTL: TX_THRESHOLD Mask */

#define SPI_FIFO_CTL_RX_THRESHOLD_Pos   24                                         /*!< SPI_T::FIFO_CTL: RX_THRESHOLD Position */
#define SPI_FIFO_CTL_RX_THRESHOLD_Msk   (7ul << SPI_FIFO_CTL_RX_THRESHOLD_Pos)     /*!< SPI_T::FIFO_CTL: RX_THRESHOLD Mask */

#define SPI_FIFO_CTL_TIMEOUT_INTEN_Pos   21                                        /*!< SPI_T::FIFO_CTL: TIMEOUT_INTEN Position */
#define SPI_FIFO_CTL_TIMEOUT_INTEN_Msk   (1ul << SPI_FIFO_CTL_TIMEOUT_INTEN_Pos)   /*!< SPI_T::FIFO_CTL: TIMEOUT_INTEN Mask */

#define SPI_FIFO_CTL_RXOV_INTEN_Pos    6                                           /*!< SPI_T::FIFO_CTL: RXOV_INTEN Position */
#define SPI_FIFO_CTL_RXOV_INTEN_Msk    (1ul << SPI_FIFO_CTL_RXOV_INTEN_Pos)        /*!< SPI_T::FIFO_CTL: RXOV_INTEN Mask */

#define SPI_FIFO_CTL_TX_INTEN_Pos    3                                             /*!< SPI_T::FIFO_CTL: TX_INTEN Position */
#define SPI_FIFO_CTL_TX_INTEN_Msk    (1ul << SPI_FIFO_CTL_TX_INTEN_Pos)            /*!< SPI_T::FIFO_CTL: TX_INTEN Mask */

#define SPI_FIFO_CTL_RX_INTEN_Pos    2                                             /*!< SPI_T::FIFO_CTL: RX_INTEN Position */
#define SPI_FIFO_CTL_RX_INTEN_Msk    (1ul << SPI_FIFO_CTL_RX_INTEN_Pos)            /*!< SPI_T::FIFO_CTL: RX_INTEN Mask */

#define SPI_FIFO_CTL_TX_CLR_Pos     1                                              /*!< SPI_T::FIFO_CTL: TX_CLR Position */
#define SPI_FIFO_CTL_TX_CLR_Msk     (1ul << SPI_FIFO_CTL_TX_CLR_Pos)               /*!< SPI_T::FIFO_CTL: TX_CLR Mask */

#define SPI_FIFO_CTL_RX_CLR_Pos      0                                             /*!< SPI_T::FIFO_CTL: RX_CLR Position */
#define SPI_FIFO_CTL_RX_CLR_Msk      (1ul << SPI_FIFO_CTL_RX_CLR_Pos)              /*!< SPI_T::FIFO_CTL: RX_CLR Mask */

/* SPI_STATUS Bit Field Definitions */
#define SPI_STATUS_TX_FIFO_COUNT_Pos   28                                            /*!< SPI_T::STATUS: TX_FIFO_COUNT Position */
#define SPI_STATUS_TX_FIFO_COUNT_Msk   (0xFul << SPI_STATUS_TX_FIFO_COUNT_Pos)       /*!< SPI_T::STATUS: TX_FIFO_COUNT Mask */

#define SPI_STATUS_TX_FULL_Pos   27                                                  /*!< SPI_T::STATUS: TX_FULL Position */
#define SPI_STATUS_TX_FULL_Msk   (1ul << SPI_STATUS_TX_FULL_Pos)                     /*!< SPI_T::STATUS: TX_FULL Mask */

#define SPI_STATUS_TX_EMPTY_Pos   26                                                 /*!< SPI_T::STATUS: TX_EMPTY Position */
#define SPI_STATUS_TX_EMPTY_Msk   (1ul << SPI_STATUS_TX_EMPTY_Pos)                   /*!< SPI_T::STATUS: TX_EMPTY Mask */

#define SPI_STATUS_RX_FULL_Pos   25                                                  /*!< SPI_T::STATUS: RX_FULL Position */
#define SPI_STATUS_RX_FULL_Msk   (1ul << SPI_STATUS_RX_FULL_Pos)                     /*!< SPI_T::STATUS: RX_FULL Mask */

#define SPI_STATUS_RX_EMPTY_Pos   24                                                 /*!< SPI_T::STATUS: RX_EMPTY Position */
#define SPI_STATUS_RX_EMPTY_Msk   (1ul << SPI_STATUS_RX_EMPTY_Pos)                   /*!< SPI_T::STATUS: RX_EMPTY Mask */

#define SPI_STATUS_TIMEOUT_Pos   20                                                  /*!< SPI_T::STATUS: TIMEOUT Position */
#define SPI_STATUS_TIMEOUT_Msk   (1ul << SPI_STATUS_TIMEOUT_Pos)                     /*!< SPI_T::STATUS: TIMEOUT Mask */

#define SPI_STATUS_IF_Pos   16                                                       /*!< SPI_T::STATUS: IF Position */
#define SPI_STATUS_IF_Msk   (1ul << SPI_STATUS_IF_Pos)                               /*!< SPI_T::STATUS: IF Mask     */

#define SPI_STATUS_RX_FIFO_COUNT_Pos   12                                            /*!< SPI_T::STATUS: RX_FIFO_COUNT Position */
#define SPI_STATUS_RX_FIFO_COUNT_Msk   (0xFul << SPI_STATUS_RX_FIFO_COUNT_Pos)       /*!< SPI_T::STATUS: RX_FIFO_COUNT Mask */

#define SPI_STATUS_SLV_START_INTSTS_Pos   11                                         /*!< SPI_T::STATUS: SLV_START_INTSTS Position */
#define SPI_STATUS_SLV_START_INTSTS_Msk   (1ul << SPI_STATUS_SLV_START_INTSTS_Pos)   /*!< SPI_T::STATUS: SLV_START_INTSTS Mask */

#define SPI_STATUS_TX_INTSTS_Pos   4                                                 /*!< SPI_T::STATUS: TX_INTSTS Position */
#define SPI_STATUS_TX_INTSTS_Msk   (1ul << SPI_STATUS_TX_INTSTS_Pos)                 /*!< SPI_T::STATUS: TX_INTSTS Mask */

#define SPI_STATUS_RX_OVERRUN_Pos   2                                                /*!< SPI_T::STATUS: RX_OVERRUN Position */
#define SPI_STATUS_RX_OVERRUN_Msk   (1ul << SPI_STATUS_RX_OVERRUN_Pos)               /*!< SPI_T::STATUS: RX_OVERRUN Mask */

#define SPI_STATUS_RX_INTSTS_Pos   0                                                 /*!< SPI_T::STATUS: RX_INTSTS Position */
#define SPI_STATUS_RX_INTSTS_Msk   (1ul << SPI_STATUS_RX_INTSTS_Pos)                 /*!< SPI_T::STATUS: RX_INTSTS Mask */
/*@}*/ /* end of group SPI_CONST */
/*@}*/ /* end of group SPI */
/**@}*/ /* end of REGISTER group */



#endif /* __SPI_REG_H__ */
