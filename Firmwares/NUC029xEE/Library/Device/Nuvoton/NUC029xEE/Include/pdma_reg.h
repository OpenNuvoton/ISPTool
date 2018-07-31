/**************************************************************************//**
 * @file     pdma_reg.h
 * @version  V1.00
 * @brief    PDMA register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __PDMA_REG_H__
#define __PDMA_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */




/*---------------------- Peripheral Direct Memory Access Controller -------------------------*/
/**
    @addtogroup PDMA Peripheral Direct Memory Access Controller (PDMA)
    Memory Mapped Structure for PDMA Controller
@{ */


typedef struct
{


/**
 * @var PDMA_T::CSR
 * Offset: 0x00  PDMA Channel x Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDMACEN   |PDMA Channel Enable
 * |        |          |Setting this bit to 1 enables PDMA operation.
 * |        |          |If this bit is cleared, PDMA will ignore all PDMA request and force Bus Master into IDLE state.
 * |        |          |Note: SW_RST(PDMA_CSRx[1], x= 0~8) will clear this bit.
 * |[1]     |SW_RST    |Software Engine Reset
 * |        |          |0 = No effect.
 * |        |          |1 = Reset the internal state machine, pointers and internal buffer.
 * |        |          |The contents of control register will not be cleared.
 * |        |          |This bit will be automatically cleared after few clock cycles.
 * |[3:2]   |MODE_SEL  |PDMA Mode Selection
 * |        |          |00 = Memory to Memory mode (Memory-to-Memory).
 * |        |          |01 = Peripheral to Memory mode (Peripheral-to-Memory).
 * |        |          |10 = Memory to Peripheral mode (Memory-to-Peripheral).
 * |[5:4]   |SAD_SEL   |Transfer Source Address Direction Selection
 * |        |          |00 = Transfer source address is increasing successively.
 * |        |          |01 = Reserved.
 * |        |          |10 = Transfer source address is fixed (This feature can be used when data where transferred from
 * |        |          |a single source to multiple destinations).
 * |        |          |11 = Reserved.
 * |[7:6]   |DAD_SEL   |Transfer Destination Address Direction Selection
 * |        |          |00 = Transfer destination address is increasing successively.
 * |        |          |01 = Reserved.
 * |        |          |10 = Transfer destination address is fixed.
 * |        |          |(This feature can be used when data where transferred from multiple sources to a single
 * |        |          |destination).
 * |        |          |11 = Reserved.
 * |[20:19] |APB_TWS   |Peripheral Transfer Width Selection
 * |        |          |00 = One word (32-bit) is transferred for every PDMA operation.
 * |        |          |01 = One byte (8-bit) is transferred for every PDMA operation.
 * |        |          |10 = One half-word (16-bit) is transferred for every PDMA operation.
 * |        |          |11 = Reserved.
 * |        |          |Note: This field is meaningful only when MODE_SEL (PDMA_CSRx[3:2]) is Peripheral to Memory mode
 * |        |          |(Peripheral-to-Memory) or Memory to Peripheral mode (Memory-to-Peripheral).
 * |[23]    |TRIG_EN   |Trigger Enable
 * |        |          |0 = No effect.
 * |        |          |1 = PDMA data read or write transfer Enabled.
 * |        |          |Note: When PDMA transfer completed, this bit will be cleared automatically.
 * |        |          |If the bus error occurs, all PDMA transfer will be stopped.
 * |        |          |Software must reset all PDMA channel, and then trigger again.
 * @var PDMA_T::SAR
 * Offset: 0x04  PDMA Channel x Source Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |PDMA_SAR  |PDMA Transfer Source Address Register
 * |        |          |This field indicates a 32-bit source address of PDMA.
 * |        |          |Note: The source address must be word alignment.
 * @var PDMA_T::DAR
 * Offset: 0x08  PDMA Channel x Destination Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |PDMA_DAR  |PDMA Transfer Destination Address Register
 * |        |          |This field indicates a 32-bit destination address of PDMA.
 * |        |          |Note: The destination address must be word alignment
 * @var PDMA_T::BCR
 * Offset: 0x0C  PDMA Channel x Transfer Byte Count Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PDMA_BCR  |PDMA Transfer Byte Count Register
 * |        |          |This field indicates a 16-bit transfer byte count number of PDMA; it must be word alignment.
 * @var PDMA_T::POINT
 * Offset: 0x10  PDMA Channel x Internal buffer pointer Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |PDMA_POINT|PDMA Internal Buffer Pointer Register (Read Only)
 * |        |          |This field indicates the internal buffer pointer.
 * @var PDMA_T::CSAR
 * Offset: 0x14  PDMA Channel x Current Source Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |PDMA_CSAR |PDMA Current Source Address Register (Read Only)
 * |        |          |This field indicates the source address where the PDMA transfer just occurred.
 * @var PDMA_T::CDAR
 * Offset: 0x18  PDMA Channel x Current Destination Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |PDMA_CDAR |PDMA Current Destination Address Register (Read Only)
 * |        |          |This field indicates the destination address where the PDMA transfer just occurred.
 * @var PDMA_T::CBCR
 * Offset: 0x1C  PDMA Channel x Current Transfer Byte Count Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PDMA_CBCR |PDMA Current Byte Count Register (Read Only)
 * |        |          |This field indicates the current remained byte count of PDMA.
 * |        |          |Note: This field value will be cleared to 0, when software set SW_RST (PDMA_CSRx[1]) to "1".
 * @var PDMA_T::IER
 * Offset: 0x20  PDMA Channel x Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TABORT_IE |PDMA Read/Write Target Abort Interrupt Enable
 * |        |          |0 = Target abort interrupt generation Disabled during PDMA transfer.
 * |        |          |1 = Target abort interrupt generation Enabled during PDMA transfer.
 * |[1]     |BLKD_IE   |PDMA Block Transfer Done Interrupt Enable
 * |        |          |0 = Interrupt generator Disabled when PDMA transfer is done.
 * |        |          |1 = Interrupt generator Enabled when PDMA transfer is done.
 * @var PDMA_T::ISR
 * Offset: 0x24  PDMA Channel x Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TABORT_IF |PDMA Read/Write Target Abort Interrupt Flag
 * |        |          |Write 1 to clear this bit to 0.
 * |        |          |0 = No bus ERROR response received.
 * |        |          |1 = Bus ERROR response received.
 * |        |          |Note: This bit filed indicates bus master received ERROR response or not.
 * |        |          |If bus master received ERROR response, it means that target abort is happened.
 * |        |          |PDMA controller will stop transfer and respond this event to software then goes to IDLE state.
 * |        |          |When target abort occurred, software must reset PDMA, and then transfer those data again.
 * |[1]     |BLKD_IF   |PDMA Block Transfer Done Interrupt Flag
 * |        |          |This bit indicates that PDMA has finished all transfers.
 * |        |          |0 = Not finished.
 * |        |          |1 = Done.
 * |        |          |Write 1 to clear this bit to 0.
 * @var PDMA_T::SBUF
 * Offset: 0x80  PDMA Channel x Shared Buffer FIFO x Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |PDMA_SBUF0|PDMA Shared Buffer FIFO 0 (Read Only)
 * |        |          |Each channel has its own 1 word internal buffer.
 */

    __IO uint32_t CSR;           /* Offset: 0x00  PDMA Channel x Control Register                                    */
    __IO uint32_t SAR;           /* Offset: 0x04  PDMA Channel x Source Address Register                             */
    __IO uint32_t DAR;           /* Offset: 0x08  PDMA Channel x Destination Address Register                        */
    __IO uint32_t BCR;           /* Offset: 0x0C  PDMA Channel x Transfer Byte Count Register                        */
    __I  uint32_t POINT;         /* Offset: 0x10  PDMA Channel x Internal buffer pointer Register                    */
    __I  uint32_t CSAR;          /* Offset: 0x14  PDMA Channel x Current Source Address Register                     */
    __I  uint32_t CDAR;          /* Offset: 0x18  PDMA Channel x Current Destination Address Register                */
    __I  uint32_t CBCR;          /* Offset: 0x1C  PDMA Channel x Current Transfer Byte Count Register                */
    __IO uint32_t IER;           /* Offset: 0x20  PDMA Channel x Interrupt Enable Register                           */
    __IO uint32_t ISR;           /* Offset: 0x24  PDMA Channel x Interrupt Status Register                           */
    __I  uint32_t RESERVE[22];  
    __I  uint32_t SBUF;          /* Offset: 0x80  PDMA Channel x Shared Buffer FIFO x Register                       */

} PDMA_T;





typedef struct
{


/**
 * @var PDMA_GCR_T::GCRCSR
 * Offset: 0x00  PDMA Global Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8]     |CLK0_EN   |PDMA Controller Channel 0 Clock Enable Control
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[9]     |CLK1_EN   |PDMA Controller Channel 1 Clock Enable Control
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[10]    |CLK2_EN   |PDMA Controller Channel 2 Clock Enable Control
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[11]    |CLK3_EN   |PDMA Controller Channel 3 Clock Enable Control
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[12]    |CLK4_EN   |PDMA Controller Channel 4 Clock Enable Control
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[13]    |CLK5_EN   |PDMA Controller Channel 5 Clock Enable Control
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[14]    |CLK6_EN   |PDMA Controller Channel 6 Clock Enable Control
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[15]    |CLK7_EN   |PDMA Controller Channel 7 Clock Enable Control
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[16]    |CLK8_EN   |PDMA Controller Channel 8 Clock Enable Control
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[24]    |CRC_CLK_EN|CRC Controller Clock Enable Control
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * @var PDMA_GCR_T::PDSSR0
 * Offset: 0x04  PDMA Service Selection Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |SPI0_RXSEL|PDMA SPI0 RX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI0 RX.
 * |        |          |Software can change the channel RX setting by this field.
 * |        |          |For example, SPI0_RXSEL (PDMA_PDSSR0[3:0]) = 0110, that means SPI0_RX is connected to PDMA_CH6.
 * |        |          |0000: CH0
 * |        |          |0001: CH1
 * |        |          |0010: CH2
 * |        |          |0011: CH3
 * |        |          |0100: CH4
 * |        |          |0101: CH5
 * |        |          |0110: CH6
 * |        |          |0111: CH7
 * |        |          |1000: CH8
 * |        |          |Others : Reserved
 * |[7:4]   |SPI0_TXSEL|PDMA SPI0 TX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI0 TX.
 * |        |          |Software can configure the TX channel setting by this field.
 * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
 * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
 * |[11:8]  |SPI1_RXSEL|PDMA SPI1 RX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI1 RX.
 * |        |          |Software can configure the RX channel setting by this field.
 * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
 * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
 * |[15:12] |SPI1_TXSEL|PDMA SPI1 TX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI1 TX.
 * |        |          |Software can configure the TX channel setting by this field.
 * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
 * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
 * |[19:16] |SPI2_RXSEL|PDMA SPI2 RX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI2 RX.
 * |        |          |Software can configure the RX channel setting by this field.
 * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
 * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
 * |[23:20] |SPI2_TXSEL|PDMA SPI2 TX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI2 TX.
 * |        |          |Software can configure the TX channel setting by this field.
 * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
 * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
 * |[27:24] |SPI3_RXSEL|PDMA SPI3 RX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI3 RX.
 * |        |          |Software can configure the RX channel setting by this field.
 * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
 * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
 * |[31:28] |SPI3_TXSEL|PDMA SPI3 TX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral SPI3 TX.
 * |        |          |Software can configure the TX channel setting by this field.
 * |        |          |The channel configuration is the same as SPI0_RXSEL (PDMA_PDSSR0[3:0]) field.
 * |        |          |Please refer to the explanation of SPI0_RXSEL (PDMA_PDSSR0[3:0]).
 * @var PDMA_GCR_T::PDSSR1
 * Offset: 0x08  PDMA Service Selection Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |UART0_RXSEL|PDMA UART0 RX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral UART0 RX.
 * |        |          |Software can change the channel RX setting by this field.
 * |        |          |For example, UART0_RXSEL (PDMA_PDSSR1[3:0]) = 0110, which means UART0_RX is connected to
 * |        |          |PDMA_CH6.
 * |        |          |0000: CH0
 * |        |          |0001: CH1
 * |        |          |0010: CH2
 * |        |          |0011: CH3
 * |        |          |0100: CH4
 * |        |          |0101: CH5
 * |        |          |0110: CH6
 * |        |          |0111: CH7
 * |        |          |1000: CH8
 * |        |          |Others : Reserved
 * |[7:4]   |UART0_TXSEL|PDMA UART0 TX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral UART0 TX.
 * |        |          |Software can configure the TX channel setting by this field.
 * |        |          |The channel configuration is the same as UART0_RXSEL (PDMA_PDSSR1[3:0]) field.
 * |        |          |Please refer to the explanation of UART0_RXSEL (PDMA_PDSSR1[3:0]).
 * |[11:8]  |UART1_RXSEL|PDMA UART1 RX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral UART1 RX.
 * |        |          |Software can configure the RX channel setting by this field.
 * |        |          |The channel configuration is the same as UART0_RXSEL (PDMA_PDSSR1[3:0]) field.
 * |        |          |Please refer to the explanation of UART0_RXSEL (PDMA_PDSSR1[3:0]).
 * |[15:12] |UART1_TXSEL|PDMA UART1 TX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral UART1 TX.
 * |        |          |Software can configure the TX channel setting by this field.
 * |        |          |The channel configuration is the same as UART0_RXSEL (PDMA_PDSSR1[3:0]) field.
 * |        |          |Please refer to the explanation of UART0_RXSEL (PDMA_PDSSR1[3:0]).
 * |[27:24] |ADC_RXSEL |PDMA ADC RX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral ADC RX.
 * |        |          |Software can configure the RX channel setting by this field.
 * |        |          |The channel configuration is the same as UART0_RXSEL (PDMA_PDSSR1[3:0]) field.
 * |        |          |Please refer to the explanation of UART0_RXSEL (PDMA_PDSSR1[3:0]).
 * @var PDMA_GCR_T::GCRISR
 * Offset: 0x0C  PDMA Global Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |INTR0     |Interrupt Status Of Channel 0
 * |        |          |This bit is the interrupt status of PDMA channel0.
 * |        |          |Note: This bit is read only
 * |[1]     |INTR1     |Interrupt Status Of Channel 1
 * |        |          |This bit is the interrupt status of PDMA channel1.
 * |        |          |Note: This bit is read only
 * |[2]     |INTR2     |Interrupt Status Of Channel 2
 * |        |          |This bit is the interrupt status of PDMA channel2.
 * |        |          |Note: This bit is read only
 * |[3]     |INTR3     |Interrupt Status Of Channel 3
 * |        |          |This bit is the interrupt status of PDMA channel3.
 * |        |          |Note: This bit is read only
 * |[4]     |INTR4     |Interrupt Status Of Channel 4
 * |        |          |This bit is the interrupt status of PDMA channel4.
 * |        |          |Note: This bit is read only
 * |[5]     |INTR5     |Interrupt Status Of Channel 5
 * |        |          |This bit is the interrupt status of PDMA channel5.
 * |        |          |Note: This bit is read only
 * |[6]     |INTR6     |Interrupt Status Of Channel 6
 * |        |          |This bit is the interrupt status of PDMA channel6.
 * |        |          |Note: This bit is read only
 * |[7]     |INTR7     |Interrupt Status Of Channel 7
 * |        |          |This bit is the interrupt status of PDMA channel7.
 * |        |          |Note: This bit is read only
 * |[8]     |INTR8     |Interrupt Status Of Channel 8
 * |        |          |This bit is the interrupt status of PDMA channel8.
 * |        |          |Note: This bit is read only
 * |[16]    |INTRCRC   |Interrupt Status Of CRC Controller
 * |        |          |This bit is the interrupt status of CRC controller
 * |        |          |Note: This bit is read only
 * |[31]    |INTR      |Interrupt Status
 * |        |          |This bit is the interrupt status of PDMA controller.
 * |        |          |Note: This bit is read only
 * @var PDMA_GCR_T::PDSSR2
 * Offset: 0x10  PDMA Service Selection Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |I2S_RXSEL |PDMA I2S RX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral I2S RX.
 * |        |          |Software can change the channel RX setting by this field.
 * |        |          |For example: I2S_RXSEL (PDMA_PDSSR2[3:0]) = 0110, that means I2S_RX is connected to PDMA_CH6.
 * |        |          |0000: CH0
 * |        |          |0001: CH1
 * |        |          |0010: CH2
 * |        |          |0011: CH3
 * |        |          |0100: CH4
 * |        |          |0101: CH5
 * |        |          |0110: CH6
 * |        |          |0111: CH7
 * |        |          |1000: CH8
 * |        |          |Others : Reserved
 * |[7:4]   |I2S_TXSEL |PDMA I2S TX Selection
 * |        |          |This field defines which PDMA channel is connected to the on-chip peripheral I2S TX.
 * |        |          |Software can configure the TX channel setting by this field.
 * |        |          |The channel configuration is the same as I2S_RXSEL (PDMA_PDSSR2[3:0]) field.
 * |        |          |Please refer to the explanation of I2S_RXSEL (PDMA_PDSSR2[3:0]).
 */

    __IO uint32_t GCRCSR;        /* Offset: 0x00  PDMA Global Control Register                                       */
    __IO uint32_t PDSSR0;        /* Offset: 0x04  PDMA Service Selection Control Register 0                          */
    __IO uint32_t PDSSR1;        /* Offset: 0x08  PDMA Service Selection Control Register 1                          */
    __IO uint32_t GCRISR;        /* Offset: 0x0C  PDMA Global Interrupt Status Register                              */
    __IO uint32_t PDSSR2;        /* Offset: 0x10  PDMA Service Selection Control Register 2                          */

} PDMA_GCR_T;



/**
    @addtogroup PDMA_CONST PDMA Bit Field Definition
    Constant Definitions for GPIO Controller
@{ */


/* PDMA CSR Bit Field Definitions */
#define PDMA_CSR_TRIG_EN_Pos                        23                              /*!< PDMA_T::CSR: TRIG_EN Position */
#define PDMA_CSR_TRIG_EN_Msk                        (1ul << PDMA_CSR_TRIG_EN_Pos)   /*!< PDMA_T::CSR: TRIG_EN Mask */

#define PDMA_CSR_APB_TWS_Pos                        19                              /*!< PDMA_T::CSR: APB_TWS Position */
#define PDMA_CSR_APB_TWS_Msk                        (3ul << PDMA_CSR_APB_TWS_Pos)   /*!< PDMA_T::CSR: APB_TWS Mask */

#define PDMA_CSR_DAD_SEL_Pos                        6                               /*!< PDMA_T::CSR: DAD_SEL Position */
#define PDMA_CSR_DAD_SEL_Msk                        (3ul << PDMA_CSR_DAD_SEL_Pos)   /*!< PDMA_T::CSR: DAD_SEL Mask */

#define PDMA_CSR_SAD_SEL_Pos                        4                               /*!< PDMA_T::CSR: SAD_SEL Position */
#define PDMA_CSR_SAD_SEL_Msk                        (3ul << PDMA_CSR_SAD_SEL_Pos)   /*!< PDMA_T::CSR: SAD_SEL Mask */

#define PDMA_CSR_MODE_SEL_Pos                       2                               /*!< PDMA_T::CSR: MODE_SEL Position */
#define PDMA_CSR_MODE_SEL_Msk                       (3ul << PDMA_CSR_MODE_SEL_Pos)  /*!< PDMA_T::CSR: MODE_SEL Mask */

#define PDMA_CSR_SW_RST_Pos                         1                               /*!< PDMA_T::CSR: SW_RST Position */
#define PDMA_CSR_SW_RST_Msk                         (1ul << PDMA_CSR_SW_RST_Pos)    /*!< PDMA_T::CSR: SW_RST Mask */

#define PDMA_CSR_PDMACEN_Pos                        0                               /*!< PDMA_T::CSR: PDMACEN Position */
#define PDMA_CSR_PDMACEN_Msk                        (1ul << PDMA_CSR_PDMACEN_Pos)   /*!< PDMA_T::CSR: PDMACEN Mask */

/* PDMA BCR Bit Field Definitions */
#define PDMA_BCR_BCR_Pos                            0                               /*!< PDMA_T::BCR: BCR Position */
#define PDMA_BCR_BCR_Msk                            (0xFFFFul << PDMA_BCR_BCR_Pos)  /*!< PDMA_T::BCR: BCR Mask */

/* PDMA POINT Bit Field Definitions */
#define PDMA_POINT_POINT_Pos                        0                               /*!< PDMA_T::POINT: POINT Position */
#define PDMA_POINT_POINT_Msk                        (0xFul << PDMA_POINT_POINT_Pos) /*!< PDMA_T::POINT: POINT Mask */

/* PDMA CBCR Bit Field Definitions */
#define PDMA_CBCR_CBCR_Pos                          0                                   /*!< PDMA_T::CBCR: CBCR Position */
#define PDMA_CBCR_CBCR_Msk                          (0xFFFFul << PDMA_CBCR_CBCR_Pos)    /*!< PDMA_T::CBCR: CBCR Mask */


/* PDMA IER Bit Field Definitions */
#define PDMA_IER_BLKD_IE_Pos                        1                               /*!< PDMA_T::IER: BLKD_IE Position */
#define PDMA_IER_BLKD_IE_Msk                        (1ul << PDMA_IER_BLKD_IE_Pos)   /*!< PDMA_T::IER: BLKD_IE Mask */

#define PDMA_IER_TABORT_IE_Pos                      0                               /*!< PDMA_T::IER: TABORT_IE Position */
#define PDMA_IER_TABORT_IE_Msk                      (1ul << PDMA_IER_TABORT_IE_Pos) /*!< PDMA_T::IER: TABORT_IE Mask */

/* PDMA ISR Bit Field Definitions */
#define PDMA_ISR_BLKD_IF_Pos                        1                               /*!< PDMA_T::ISR: BLKD_IF Position */
#define PDMA_ISR_BLKD_IF_Msk                        (1ul << PDMA_ISR_BLKD_IF_Pos)   /*!< PDMA_T::ISR: BLKD_IF Mask */

#define PDMA_ISR_TABORT_IF_Pos                      0                               /*!< PDMA_T::ISR: TABORT_IF Position */
#define PDMA_ISR_TABORT_IF_Msk                      (1ul << PDMA_ISR_TABORT_IF_Pos) /*!< PDMA_T::ISR: TABORT_IF Mask */

/* PDMA GCRCSR Bit Field Definitions */
#define PDMA_GCRCSR_CRC_CLK_EN_Pos                  24                                  /*!< PDMA_GCR_T::GCRCSR: CRC_CLK_EN Position */
#define PDMA_GCRCSR_CRC_CLK_EN_Msk                  (1ul << PDMA_GCRCSR_CRC_CLK_EN_Pos) /*!< PDMA_GCR_T::GCRCSR: CRC_CLK_EN Mask */

#define PDMA_GCRCSR_CLK8_EN_Pos                     16                                  /*!< PDMA_GCR_T::GCRCSR: CLK8_EN Position */
#define PDMA_GCRCSR_CLK8_EN_Msk                     (1ul << PDMA_GCRCSR_CLK8_EN_Pos)    /*!< PDMA_GCR_T::GCRCSR: CLK8_EN Mask */

#define PDMA_GCRCSR_CLK7_EN_Pos                     15                                  /*!< PDMA_GCR_T::GCRCSR: CLK7_EN Position */
#define PDMA_GCRCSR_CLK7_EN_Msk                     (1ul << PDMA_GCRCSR_CLK7_EN_Pos)    /*!< PDMA_GCR_T::GCRCSR: CLK7_EN Mask */

#define PDMA_GCRCSR_CLK6_EN_Pos                     14                                  /*!< PDMA_GCR_T::GCRCSR: CLK6_EN Position */
#define PDMA_GCRCSR_CLK6_EN_Msk                     (1ul << PDMA_GCRCSR_CLK6_EN_Pos)    /*!< PDMA_GCR_T::GCRCSR: CLK6_EN Mask */

#define PDMA_GCRCSR_CLK5_EN_Pos                     13                                  /*!< PDMA_GCR_T::GCRCSR: CLK5_EN Position */
#define PDMA_GCRCSR_CLK5_EN_Msk                     (1ul << PDMA_GCRCSR_CLK5_EN_Pos)    /*!< PDMA_GCR_T::GCRCSR: CLK5_EN Mask */

#define PDMA_GCRCSR_CLK4_EN_Pos                     12                                  /*!< PDMA_GCR_T::GCRCSR: CLK4_EN Position */
#define PDMA_GCRCSR_CLK4_EN_Msk                     (1ul << PDMA_GCRCSR_CLK4_EN_Pos)    /*!< PDMA_GCR_T::GCRCSR: CLK4_EN Mask */

#define PDMA_GCRCSR_CLK3_EN_Pos                     11                                  /*!< PDMA_GCR_T::GCRCSR: CLK3_EN Position */
#define PDMA_GCRCSR_CLK3_EN_Msk                     (1ul << PDMA_GCRCSR_CLK3_EN_Pos)    /*!< PDMA_GCR_T::GCRCSR: CLK3_EN Mask */

#define PDMA_GCRCSR_CLK2_EN_Pos                     10                                  /*!< PDMA_GCR_T::GCRCSR: CLK2_EN Position */
#define PDMA_GCRCSR_CLK2_EN_Msk                     (1ul << PDMA_GCRCSR_CLK2_EN_Pos)    /*!< PDMA_GCR_T::GCRCSR: CLK2_EN Mask */

#define PDMA_GCRCSR_CLK1_EN_Pos                     9                                   /*!< PDMA_GCR_T::GCRCSR: CLK1_EN Position */
#define PDMA_GCRCSR_CLK1_EN_Msk                     (1ul << PDMA_GCRCSR_CLK1_EN_Pos)    /*!< PDMA_GCR_T::GCRCSR: CLK1_EN Mask */

#define PDMA_GCRCSR_CLK0_EN_Pos                     8                                   /*!< PDMA_GCR_T::GCRCSR: CLK0_EN Position */
#define PDMA_GCRCSR_CLK0_EN_Msk                     (1ul << PDMA_GCRCSR_CLK0_EN_Pos)    /*!< PDMA_GCR_T::GCRCSR: CLK0_EN Mask */

/* PDMA PDSSR0 Bit Field Definitions */
#define PDMA_PDSSR0_SPI3_TXSEL_Pos                  28                                      /*!< PDMA_GCR_T::PDSSR0: SPI3_TXSEL Position */
#define PDMA_PDSSR0_SPI3_TXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI3_TXSEL_Pos)   /*!< PDMA_GCR_T::PDSSR0: SPI3_TXSEL Mask */

#define PDMA_PDSSR0_SPI3_RXSEL_Pos                  24                                      /*!< PDMA_GCR_T::PDSSR0: SPI3_RXSEL Position */
#define PDMA_PDSSR0_SPI3_RXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI3_RXSEL_Pos)   /*!< PDMA_GCR_T::PDSSR0: SPI3_RXSEL Mask */

#define PDMA_PDSSR0_SPI2_TXSEL_Pos                  20                                      /*!< PDMA_GCR_T::PDSSR0: SPI2_TXSEL Position */
#define PDMA_PDSSR0_SPI2_TXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI2_TXSEL_Pos)   /*!< PDMA_GCR_T::PDSSR0: SPI2_TXSEL Mask */

#define PDMA_PDSSR0_SPI2_RXSEL_Pos                  16                                      /*!< PDMA_GCR_T::PDSSR0: SPI2_RXSEL Position */
#define PDMA_PDSSR0_SPI2_RXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI2_RXSEL_Pos)   /*!< PDMA_GCR_T::PDSSR0: SPI2_RXSEL Mask */

#define PDMA_PDSSR0_SPI1_TXSEL_Pos                  12                                      /*!< PDMA_GCR_T::PDSSR0: SPI1_TXSEL Position */
#define PDMA_PDSSR0_SPI1_TXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI1_TXSEL_Pos)   /*!< PDMA_GCR_T::PDSSR0: SPI1_TXSEL Mask */

#define PDMA_PDSSR0_SPI1_RXSEL_Pos                  8                                       /*!< PDMA_GCR_T::PDSSR0: SPI1_RXSEL Position */
#define PDMA_PDSSR0_SPI1_RXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI1_RXSEL_Pos)   /*!< PDMA_GCR_T::PDSSR0: SPI1_RXSEL Mask */

#define PDMA_PDSSR0_SPI0_TXSEL_Pos                  4                                       /*!< PDMA_GCR_T::PDSSR0: SPI0_TXSEL Position */
#define PDMA_PDSSR0_SPI0_TXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI0_TXSEL_Pos)   /*!< PDMA_GCR_T::PDSSR0: SPI0_TXSEL Mask */

#define PDMA_PDSSR0_SPI0_RXSEL_Pos                  0                                       /*!< PDMA_GCR_T::PDSSR0: SPI0_RXSEL Position */
#define PDMA_PDSSR0_SPI0_RXSEL_Msk                  (0xFul << PDMA_PDSSR0_SPI0_RXSEL_Pos)   /*!< PDMA_GCR_T::PDSSR0: SPI0_RXSEL Mask */

/* PDMA PDSSR1 Bit Field Definitions */
#define PDMA_PDSSR1_ADC_RXSEL_Pos                   24                                      /*!< PDMA_GCR_T::PDSSR1: ADC_RXSEL Position */
#define PDMA_PDSSR1_ADC_RXSEL_Msk                   (0xFul << PDMA_PDSSR1_ADC_RXSEL_Pos)    /*!< PDMA_GCR_T::PDSSR1: ADC_RXSEL Mask */

#define PDMA_PDSSR1_UART1_TXSEL_Pos                 12                                      /*!< PDMA_GCR_T::PDSSR1: UART1_TXSEL Position */
#define PDMA_PDSSR1_UART1_TXSEL_Msk                 (0xFul << PDMA_PDSSR1_UART1_TXSEL_Pos)  /*!< PDMA_GCR_T::PDSSR1: UART1_TXSEL Mask */

#define PDMA_PDSSR1_UART1_RXSEL_Pos                 8                                       /*!< PDMA_GCR_T::PDSSR1: UART1_RXSEL Position */
#define PDMA_PDSSR1_UART1_RXSEL_Msk                 (0xFul << PDMA_PDSSR1_UART1_RXSEL_Pos)  /*!< PDMA_GCR_T::PDSSR1: UART1_RXSEL Mask */

#define PDMA_PDSSR1_UART0_TXSEL_Pos                 4                                       /*!< PDMA_GCR_T::PDSSR1: UART0_TXSEL Position */
#define PDMA_PDSSR1_UART0_TXSEL_Msk                 (0xFul << PDMA_PDSSR1_UART0_TXSEL_Pos)  /*!< PDMA_GCR_T::PDSSR1: UART0_TXSEL Mask */

#define PDMA_PDSSR1_UART0_RXSEL_Pos                 0                                       /*!< PDMA_GCR_T::PDSSR1: UART0_RXSEL Position */
#define PDMA_PDSSR1_UART0_RXSEL_Msk                 (0xFul << PDMA_PDSSR1_UART0_RXSEL_Pos)  /*!< PDMA_GCR_T::PDSSR1: UART0_RXSEL Mask */

/* PDMA GCRISR Bit Field Definitions */
#define PDMA_GCRISR_INTR_Pos                        31                              /*!< PDMA_GCR_T::GCRISR: INTR Position */
#define PDMA_GCRISR_INTR_Msk                        (1ul << PDMA_GCRISR_INTR_Pos)   /*!< PDMA_GCR_T::GCRISR: INTR Mask */

#define PDMA_GCRISR_INTRCRC_Pos                     16                               /*!< PDMA_GCR_T::GCRISR: INTRCRC Position */
#define PDMA_GCRISR_INTRCRC_Msk                     (1ul << PDMA_GCRISR_INTRCRC_Pos) /*!< PDMA_GCR_T::GCRISR: INTRCRC Mask */

#define PDMA_GCRISR_INTR8_Pos                       8                               /*!< PDMA_GCR_T::GCRISR: INTR8 Position */
#define PDMA_GCRISR_INTR8_Msk                       (1ul << PDMA_GCRISR_INTR8_Pos)  /*!< PDMA_GCR_T::GCRISR: INTR8 Mask */

#define PDMA_GCRISR_INTR7_Pos                       7                               /*!< PDMA_GCR_T::GCRISR: INTR7 Position */
#define PDMA_GCRISR_INTR7_Msk                       (1ul << PDMA_GCRISR_INTR7_Pos)  /*!< PDMA_GCR_T::GCRISR: INTR7 Mask */

#define PDMA_GCRISR_INTR6_Pos                       6                               /*!< PDMA_GCR_T::GCRISR: INTR6 Position */
#define PDMA_GCRISR_INTR6_Msk                       (1ul << PDMA_GCRISR_INTR6_Pos)  /*!< PDMA_GCR_T::GCRISR: INTR6 Mask */

#define PDMA_GCRISR_INTR5_Pos                       5                               /*!< PDMA_GCR_T::GCRISR: INTR5 Position */
#define PDMA_GCRISR_INTR5_Msk                       (1ul << PDMA_GCRISR_INTR5_Pos)  /*!< PDMA_GCR_T::GCRISR: INTR5 Mask */

#define PDMA_GCRISR_INTR4_Pos                       4                               /*!< PDMA_GCR_T::GCRISR: INTR4 Position */
#define PDMA_GCRISR_INTR4_Msk                       (1ul << PDMA_GCRISR_INTR4_Pos)  /*!< PDMA_GCR_T::GCRISR: INTR4 Mask */

#define PDMA_GCRISR_INTR3_Pos                       3                               /*!< PDMA_GCR_T::GCRISR: INTR3 Position */
#define PDMA_GCRISR_INTR3_Msk                       (1ul << PDMA_GCRISR_INTR3_Pos)  /*!< PDMA_GCR_T::GCRISR: INTR3 Mask */

#define PDMA_GCRISR_INTR2_Pos                       2                               /*!< PDMA_GCR_T::GCRISR: INTR2 Position */
#define PDMA_GCRISR_INTR2_Msk                       (1ul << PDMA_GCRISR_INTR2_Pos)  /*!< PDMA_GCR_T::GCRISR: INTR2 Mask */

#define PDMA_GCRISR_INTR1_Pos                       1                               /*!< PDMA_GCR_T::GCRISR: INTR1 Position */
#define PDMA_GCRISR_INTR1_Msk                       (1ul << PDMA_GCRISR_INTR1_Pos)  /*!< PDMA_GCR_T::GCRISR: INTR1 Mask */

#define PDMA_GCRISR_INTR0_Pos                       0                               /*!< PDMA_GCR_T::GCRISR: INTR0 Position */
#define PDMA_GCRISR_INTR0_Msk                       (1ul << PDMA_GCRISR_INTR0_Pos)  /*!< PDMA_GCR_T::GCRISR: INTR0 Mask */

/* PDMA PDSSR2 Bit Field Definitions */
#define PDMA_PDSSR2_I2S_TXSEL_Pos                   4                                       /*!< PDMA_GCR_T::PDSSR2: I2S_TXSEL Position */
#define PDMA_PDSSR2_I2S_TXSEL_Msk                   (0xFul << PDMA_PDSSR2_I2S_TXSEL_Pos)    /*!< PDMA_GCR_T::PDSSR2: I2S_TXSEL Mask */

#define PDMA_PDSSR2_I2S_RXSEL_Pos                   0                                       /*!< PDMA_GCR_T::PDSSR2: I2S_RXSEL Position */
#define PDMA_PDSSR2_I2S_RXSEL_Msk                   (0xFul << PDMA_PDSSR2_I2S_RXSEL_Pos)    /*!< PDMA_GCR_T::PDSSR2: I2S_RXSEL Mask */
/*@}*/ /* end of group PDMA_CONST */
/*@}*/ /* end of group DMA */
/**@}*/ /* end of REGISTER group */

#endif /* __PDMA_REG_H__ */
