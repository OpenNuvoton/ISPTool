/**************************************************************************//**
 * @file     crc_reg.h
 * @version  V1.00
 * @brief    CRC register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __CRC_REG_H__
#define __CRC_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */


/*----------------------------- Cyclic Redundancy Check (CRC) Controller -----------------------------*/
/** @addtogroup CRC Cyclic Redundancy Check Controller (CRC)
  Memory Mapped Structure for Cyclic Redundancy Check
  @{
 */


typedef struct
{


/**
 * @var CRC_T::CTL
 * Offset: 0x00  CRC Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CRCCEN    |CRC Channel Enable
 * |        |          |0 = No effect.
 * |        |          |1 = CRC operation Enabled.
 * |        |          |Note1: When operating in CRC DMA mode (TRIG_EN (CRC_CTL[23]) = 1), if user clears this bit, the
 * |        |          |DMA operation will be continuous until all CRC DMA operation is done, and the TRIG_EN
 * |        |          |(CRC_CTL[23]) bit will keep 1until all CRC DMA operation done.
 * |        |          |But in this case, the CRC_BLKD_IF (CRC_DMAISR[1])flag will inactive, user can read CRC checksum
 * |        |          |result only if TRIG_EN (CRC_CTL[23]) clears to 0.
 * |        |          |Note2: When operating in CRC DMA mode (TRIG_EN (CRC_CTL[23]) = 1), if user wants to stop the
 * |        |          |transfer immediately, user can write 1 to CRC_RST (CRC_CTL [1]) bit to stop the transmission.
 * |[1]     |CRC_RST   |CRC Engine Reset
 * |        |          |0 = No effect.
 * |        |          |1 = Reset the internal CRC state machine and internal buffer.
 * |        |          |The others contents of CRC_CTL register will not be cleared.
 * |        |          |This bit will be cleared automatically.
 * |        |          |Note: When operated in CPU PIO mode, setting this bit will reload the initial seed value
 * |        |          |(CRC_SEED register).
 * |[23]    |TRIG_EN   |Trigger Enable
 * |        |          |This bit is used to trigger the CRC DMA transfer.
 * |        |          |0 = No effect.
 * |        |          |1 = CRC DMA data read or write transfer Enabled.
 * |        |          |Note1: If this bit asserts which indicates the CRC engine operation in CRC DMA mode, do not fill
 * |        |          |in any data in CRC_WDATA register.
 * |        |          |Note2: When CRC DMA transfer completed, this bit will be cleared automatically.
 * |        |          |Note3: If the bus error occurs when CRC DMA transfer data, all CRC DMA transfer will be stopped.
 * |        |          |Software must reset all DMA channel before trigger DMA again.
 * |[24]    |WDATA_RVS |Write Data Order Reverse
 * |        |          |This bit is used to enable the bit order reverse function for write data value in CRC_WDATA
 * |        |          |register.
 * |        |          |0 = Bit order reverse for CRC write data in Disabled.
 * |        |          |1 = Bit order reverse for CRC write data in Enabled (per byre).
 * |        |          |Note: If the write data is 0xAABBCCDD, the bit order reverse for CRC write data in is 0x55DD33BB
 * |[25]    |CHECKSUM_RVS|Checksum Reverse
 * |        |          |This bit is used to enable the bit order reverse function for write data value in CRC_CHECKSUM
 * |        |          |register.
 * |        |          |0 = Bit order reverse for CRC checksum Disabled.
 * |        |          |1 = Bit order reverse for CRC checksum Enabled.
 * |        |          |Note: If the checksum result is 0XDD7B0F2E, the bit order reverse for CRC checksum is 0x74F0DEBB
 * |[26]    |WDATA_COM |Write Data 1's Complement
 * |        |          |This bit is used to enable the 1's complement function for write data value in CRC_WDATA
 * |        |          |register.
 * |        |          |0 = 1's complement for CRC write data in Disabled.
 * |        |          |1 = 1's complement for CRC write data in Enabled.
 * |[27]    |CHECKSUM_COM|Checksum 1's Complement
 * |        |          |This bit is used to enable the 1's complement function for checksum result in CRC_CHECKSUM
 * |        |          |register.
 * |        |          |0 = 1's complement for CRC checksum Disabled.
 * |        |          |1 = 1's complement for CRC checksum Enabled.
 * |[29:28] |CPU_WDLEN |CPU Write Data Length
 * |        |          |This field indicates the CPU write data length only when operating in CPU PIO mode.
 * |        |          |00 = The write data length is 8-bit mode.
 * |        |          |01 = The write data length is 16-bit mode.
 * |        |          |10 = The write data length is 32-bit mode.
 * |        |          |11 = Reserved.
 * |        |          |Note1: This field is only valid when operating in CPU PIO mode.
 * |        |          |Note2: When the write data length is 8-bit mode, the valid data in CRC_WDATA register is only
 * |        |          |CRC_WDATA [7:0] bits; if the write data length is 16-bit mode, the valid data in CRC_WDATA
 * |        |          |register is only CRC_WDATA [15:0].
 * |[31:30] |CRC_MODE  |CRC Polynomial Mode
 * |        |          |This field indicates the CRC operation polynomial mode.
 * |        |          |00 = CRC-CCITT Polynomial Mode.
 * |        |          |01 = CRC-8 Polynomial Mode.
 * |        |          |10 = CRC-16 Polynomial Mode.
 * |        |          |11 = CRC-32 Polynomial Mode.
 * @var CRC_T::DMASAR
 * Offset: 0x04  CRC DMA Source Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |CRC_DMASAR|CRC DMA Transfer Source Address Register
 * |        |          |This field indicates a 32-bit source address of CRC DMA.
 * |        |          |(CRC_DMASAR + CRC_DMABCR) = (CRC_DMACSAR + CRC_DMACBCR).
 * |        |          |Note: The source address must be word alignment
 * @var CRC_T::DMABCR
 * Offset: 0x0C  CRC DMA Transfer Byte Count Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CRC_DMABCR|CRC DMA Transfer Byte Count Register
 * |        |          |This field indicates a 16-bit total transfer byte count number of CRC DMA
 * |        |          |(CRC_DMASAR + CRC_DMABCR) = (CRC_DMACSAR + CRC_DMACBCR).
 * @var CRC_T::DMACSAR
 * Offset: 0x14  CRC DMA Current Source Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |CRC_DMACSAR|CRC DMA Current Source Address Register (Read Only)
 * |        |          |This field indicates the current source address where the CRC DMA transfer just occurs.
 * |        |          |(CRC_DMASAR + CRC_DMABCR) = (CRC_DMACSAR + CRC_DMACBCR).
 * @var CRC_T::DMACBCR
 * Offset: 0x1C  CRC DMA Current Transfer Byte Count Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CRC_DMACBCR|CRC DMA Current Remained Byte Count Register (Read Only)
 * |        |          |This field indicates the current remained byte count of CRC DMA.
 * |        |          |(CRC_DMASAR + CRC_DMABCR) = (CRC_DMACSAR + CRC_DMACBCR).
 * |        |          |Note: Setting CRC_RST (CRC_CTL[1]) bit to 1 will clear this register value.
 * @var CRC_T::DMAIER
 * Offset: 0x20  CRC DMA Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CRC_TABORT_IE|CRC DMA Read/Write Target Abort Interrupt Enable
 * |        |          |Enable this bit will generate the CRC DMA Target Abort interrupt signal while CRC_TARBOT_IF
 * |        |          |(CRC_DMAISR[0]) bit is set to 1.
 * |        |          |0 = Target abort interrupt generation Disabled during CRC DMA transfer.
 * |        |          |1 = Target abort interrupt generation Enabled during CRC DMA transfer.
 * |[1]     |CRC_BLKD_IE|CRC DMA Block Transfer Done Interrupt Enable
 * |        |          |Enable this bit will generate the CRC DMA Transfer Done interrupt signal while CRC_BLKD_IF
 * |        |          |(CRC_DMAISR[1]) bit is set to 1.
 * |        |          |0 = Interrupt generator Disabled when CRC DMA transfer done.
 * |        |          |1 = Interrupt generator Enabled when CRC DMA transfer done.
 * @var CRC_T::DMAISR
 * Offset: 0x24  CRC DMA Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CRC_TABORT_IF|CRC DMA Read/Write Target Abort Interrupt Flag
 * |        |          |This bit indicates that CRC bus has error or not during CRC DMA transfer.
 * |        |          |0 = No bus error response received during CRC DMA transfer.
 * |        |          |1 = Bus error response received during CRC DMA transfer.
 * |        |          |It is cleared by writing 1 to it through software.
 * |        |          |Note: The bit filed indicate bus master received error response or not.
 * |        |          |If bus master received error response, it means that CRC transfer target abort is happened.
 * |        |          |DMA will stop transfer and respond this event to software then CRC state machine goes to IDLE
 * |        |          |state.
 * |        |          |When target abort occurred, software must reset DMA before transfer those data again.
 * |[1]     |CRC_BLKD_IF|CRC DMA Block Transfer Done Interrupt Flag
 * |        |          |This bit indicates that CRC DMA transfer has finished or not.
 * |        |          |0 = Not finished if TRIG_EN (CRC_CTL[23]) bit has enabled.
 * |        |          |1 = CRC transfer done if TRIG_EN (CRC_CTL[23]) bit has enabled.
 * |        |          |It is cleared by writing 1 to it through software.
 * |        |          |(When CRC DMA transfer done, TRIG_EN (CRC_CTL[23]) bit will be cleared automatically)
 * @var CRC_T::WDATA
 * Offset: 0x80  CRC Write Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |CRC_WDATA |CRC Write Data Register
 * |        |          |When operating in CPU PIO mode, software can write data to this field to perform CRC operation.
 * |        |          |When operating in DMA mode, this field indicates the DMA read data from memory and cannot be
 * |        |          |written.
 * |        |          |Note: When the write data length is 8-bit mode, the valid data in CRC_WDATA register is only
 * |        |          |CRC_WDATA [7:0] bits; if the write data length is 16-bit mode, the valid data in CRC_WDATA
 * |        |          |register is only CRC_WDATA [15:0].
 * @var CRC_T::SEED
 * Offset: 0x84  CRC Seed Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |CRC_SEED  |CRC Seed Register
 * |        |          |This field indicates the CRC seed value.
 * @var CRC_T::CHECKSUM
 * Offset: 0x88  CRC Checksum Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |CRC_CHECKSUM|CRC Checksum Register
 * |        |          |This fields indicates the CRC checksum result
 */

    __IO uint32_t CTL;           /* Offset: 0x00  CRC Control Register                                               */
    __IO uint32_t DMASAR;        /* Offset: 0x04  CRC DMA Source Address Register                                    */
    __I  uint32_t RESERVED0;    
    __IO uint32_t DMABCR ;       /* Offset: 0x0C  CRC DMA Transfer Byte Count Register                               */
    __I  uint32_t RESERVED1;    
    __I  uint32_t DMACSAR;       /* Offset: 0x14  CRC DMA Current Source Address Register                            */
    __I  uint32_t RESERVED2;    
    __I  uint32_t DMACBCR;       /* Offset: 0x1C  CRC DMA Current Transfer Byte Count Register                       */
    __IO uint32_t DMAIER ;       /* Offset: 0x20  CRC DMA Interrupt Enable Register                                  */
    __IO uint32_t DMAISR;        /* Offset: 0x24  CRC DMA Interrupt Status Register                                  */
    __I  uint32_t RESERVED3[22];
    __IO uint32_t WDATA;         /* Offset: 0x80  CRC Write Data Register                                            */
    __IO uint32_t SEED;          /* Offset: 0x84  CRC Seed Register                                                  */
    __I  uint32_t CHECKSUM;      /* Offset: 0x88  CRC Checksum Register                                              */

} CRC_T;


/**
    @addtogroup CRC_CONST CRC Bit Field Definition
    Constant Definitions for CRC Controller
@{ */


/* CRC CTL Bit Field Definitions */
#define CRC_CTL_CRC_MODE_Pos            30                                      /*!< CRC_T::CTL: CRC_MODE Position */
#define CRC_CTL_CRC_MODE_Msk            (0x3ul << CRC_CTL_CRC_MODE_Pos)         /*!< CRC_T::CTL: CRC_MODE Mask */

#define CRC_CTL_CPU_WDLEN_Pos           28                                      /*!< CRC_T::CTL: CPU_WDLEN Position */
#define CRC_CTL_CPU_WDLEN_Msk           (0x3ul << CRC_CTL_CPU_WDLEN_Pos)        /*!< CRC_T::CTL: CPU_WDLEN Mask */

#define CRC_CTL_CHECKSUM_COM_Pos        27                                      /*!< CRC_T::CTL: CHECKSUM_COM Position */
#define CRC_CTL_CHECKSUM_COM_Msk        (1ul << CRC_CTL_CHECKSUM_COM_Pos)       /*!< CRC_T::CTL: CHECKSUM_COM Mask */

#define CRC_CTL_WDATA_COM_Pos           26                                      /*!< CRC_T::CTL: WDATA_COM Position */
#define CRC_CTL_WDATA_COM_Msk           (1ul << CRC_CTL_WDATA_COM_Pos)          /*!< CRC_T::CTL: WDATA_COM Mask */

#define CRC_CTL_CHECKSUM_RVS_Pos        25                                      /*!< CRC_T::CTL: CHECKSUM_RVS Position */
#define CRC_CTL_CHECKSUM_RVS_Msk        (1ul << CRC_CTL_CHECKSUM_RVS_Pos)       /*!< CRC_T::CTL: CHECKSUM_RVS Mask */

#define CRC_CTL_WDATA_RVS_Pos           24                                      /*!< CRC_T::CTL: WDATA_RVS Position */
#define CRC_CTL_WDATA_RVS_Msk           (1ul << CRC_CTL_WDATA_RVS_Pos)          /*!< CRC_T::CTL: WDATA_RVS Mask */

#define CRC_CTL_TRIG_EN_Pos             23                                      /*!< CRC_T::CTL: TRIG_EN Position */
#define CRC_CTL_TRIG_EN_Msk             (1ul << CRC_CTL_TRIG_EN_Pos)            /*!< CRC_T::CTL: TRIG_EN Mask */

#define CRC_CTL_CRC_RST_Pos             1                                       /*!< CRC_T::CTL: CRC_RST Position */
#define CRC_CTL_CRC_RST_Msk             (1ul << CRC_CTL_CRC_RST_Pos)            /*!< CRC_T::CTL: CRC_RST Mask */

#define CRC_CTL_CRCCEN_Pos              0                                       /*!< CRC_T::CTL: CRCCEN Position */
#define CRC_CTL_CRCCEN_Msk              (1ul << CRC_CTL_CRCCEN_Pos)             /*!< CRC_T::CTL: CRCCEN Mask */

/* CRC DMASAR Bit Field Definitions */
#define CRC_DMASAR_CRC_DMASAR_Pos       0                                               /*!< CRC_T::DMASAR: CRC_DMASAR Position */
#define CRC_DMASAR_CRC_DMASAR_Msk       (0xFFFFFFFFul << CRC_DMASAR_CRC_DMASAR_Pos)     /*!< CRC_T::DMASAR: CRC_DMASAR Mask */

/* CRC DMABCR Bit Field Definitions */
#define CRC_DMABCR_CRC_DMABCR_Pos       0                                               /*!< CRC_T::DMABCR: CRC_DMABCR Position */
#define CRC_DMABCR_CRC_DMABCR_Msk       (0xFFFFul << CRC_DMABCR_CRC_DMABCR_Pos)         /*!< CRC_T::DMABCR: CRC_DMABCR Mask */

/* CRC DMACSAR Bit Field Definitions */
#define CRC_DMACSAR_CRC_DMACSAR_Pos     0                                               /*!< CRC_T::DMACSAR: CRC_DMACSAR Position */
#define CRC_DMACSAR_CRC_DMACSAR_Msk     (0xFFFFFFFFul << CRC_DMACSAR_CRC_DMACSAR_Pos)   /*!< CRC_T::DMACSAR: CRC_DMACSAR Mask */

/* CRC DMACBCR Bit Field Definitions */
#define CRC_DMACBCR_CRC_DMACBCR_Pos     0                                               /*!< CRC_T::DMACBCR: DMACBCR Position */
#define CRC_DMACBCR_CRC_DMACBCR_Msk     (0xFFFFul << CRC_DMACBCR_CRC_DMACBCR_Pos)       /*!< CRC_T::DMACBCR: DMACBCR Mask */

/* CRC DMAIER Bit Field Definitions */
#define CRC_DMAIER_CRC_BLKD_IE_Pos      1                                               /*!< CRC_T::DMAIER: CRC_BLKD_IE Position */
#define CRC_DMAIER_CRC_BLKD_IE_Msk      (1ul << CRC_DMAIER_CRC_BLKD_IE_Pos)             /*!< CRC_T::DMAIER: CRC_BLKD_IE Mask */

#define CRC_DMAIER_CRC_TABORT_IE_Pos    0                                               /*!< CRC_T::DMAIER: CRC_TABORT_IE Position */
#define CRC_DMAIER_CRC_TABORT_IE_Msk    (1ul << CRC_DMAIER_CRC_TABORT_IE_Pos)           /*!< CRC_T::DMAIER: CRC_TABORT_IE Mask */

/* CRC DMAISR Bit Field Definitions */
#define CRC_DMAISR_CRC_BLKD_IF_Pos      1                                               /*!< CRC_T::DMAISR: CRC_BLKD_IF Position */
#define CRC_DMAISR_CRC_BLKD_IF_Msk      (1ul << CRC_DMAISR_CRC_BLKD_IF_Pos)             /*!< CRC_T::DMAISR: CRC_BLKD_IF Mask */

#define CRC_DMAISR_CRC_TABORT_IF_Pos    0                                               /*!< CRC_T::DMAISR: CRC_TABORT_IF Position */
#define CRC_DMAISR_CRC_TABORT_IF_Msk    (1ul << CRC_DMAISR_CRC_TABORT_IF_Pos)           /*!< CRC_T::DMAISR: CRC_TABORT_IF Mask */

/* CRC WDATA Bit Field Definitions */
#define CRC_WDATA_CRC_WDATA_Pos         0                                               /*!< CRC_T::WDATA: CRC_WDATA Position */
#define CRC_WDATA_CRC_WDATA_Msk         (0xFFFFFFFFul << CRC_WDATA_CRC_WDATA_Pos)       /*!< CRC_T::WDATA: CRC_WDATA Mask */

/* CRC SEED Bit Field Definitions */
#define CRC_SEED_CRC_SEED_Pos           0                                               /*!< CRC_T::SEED: CRC_SEED Position */
#define CRC_SEED_CRC_SEED_Msk           (0xFFFFFFFFul << CRC_SEED_CRC_SEED_Pos)         /*!< CRC_T::SEED: CRC_SEED Mask */

/* CRC CHECKSUM Bit Field Definitions */
#define CRC_CHECKSUM_CRC_CHECKSUM_Pos   0                                               /*!< CRC_T::CHECKSUM: CRC_CHECKSUM Position */
#define CRC_CHECKSUM_CRC_CHECKSUM_Msk   (0xFFFFFFFFul << CRC_CHECKSUM_CRC_CHECKSUM_Pos) /*!< CRC_T::CHECKSUM: CRC_CHECKSUM Mask */
/*@}*/ /* end of group CRC_CONST */
/*@}*/ /* end of group CRC */

/**@}*/ /* end of REGISTER group */


#endif /* __CRC_REG_H__ */


