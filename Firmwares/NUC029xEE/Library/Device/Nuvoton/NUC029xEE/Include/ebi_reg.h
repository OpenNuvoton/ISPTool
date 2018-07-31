/**************************************************************************//**
 * @file     ebi_reg.h
 * @version  V1.00
 * @brief    EBI register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __EBI_REG_H__
#define __EBI_REG_H__


/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */


/*---------------------- External Bus Interface Controller -------------------------*/
/**
    @addtogroup EBI External Bus Interface Controller (EBI)
    Memory Mapped Structure for EBI Controller
@{ */


typedef struct
{


/**
 * @var EBI_T::EBICON
 * Offset: 0x00  EBI Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ExtEN     |EBI Enable
 * |        |          |This bit is the functional enable bit for EBI.
 * |        |          |0 = EBI function is disabled
 * |        |          |1 = EBI function is enabled
 * |[1]     |ExtBW16   |EBI data width 16 bit
 * |        |          |This bit defines if the data bus is 8-bit or 16-bit.
 * |        |          |0 = EBI data width is 8 bit
 * |        |          |1 = EBI data width is 16 bit
 * |[10:8]  |MCLKDIV   |External Output Clock Divider
 * |        |          |The frequency of EBI output clock is controlled by MCLKDIV.
 * |        |          |000 = HCLK/1
 * |        |          |001 = HCLK/2
 * |        |          |010 = HCLK/4
 * |        |          |011 = HCLK/8
 * |        |          |100 = HCLK/16
 * |        |          |101 = HCKL/32
 * |        |          |11X = default
 * |        |          |Notice: Default value of output clock is HCLK/1
 * |[18:16] |ExttALE   |Expand Time of ALE
 * |        |          |The ALE width (tALE) to latch the address can be controlled by ExttALE.
 * |        |          |tALE = (ExttALE + 1) * MCLK
 * @var EBI_T::EXTIME
 * Offset: 0x04  EBI Timing Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:3]   |ExttACC   |EBI Data Accesss Time
 * |        |          |ExttACC define data access time (tACC).
 * |        |          |tACC = (ExttACC + 1) * MCLK
 * |[10:8]  |ExttAHD   |EBI Data Access Hold Time
 * |        |          |ExttAHD define data access hold time (tAHD).
 * |        |          |tAHD = (ExttAHD + 1) * MCLK
 * |[15:12] |ExtIW2X   |Idle State Cycle After Write
 * |        |          |When write action is finish, idle state is inserted and nCS return to high if ExtIW2X is not
 * |        |          |zero.
 * |        |          |Idle state cycle = (ExtIW2X * MCLK)
 * |[27:24] |ExtIR2R   |Idle State Cycle Between Read-Read
 * |        |          |When read action is finish and next action is going to read, idle state is inserted and nCS
 * |        |          |return
 * |        |          |to high if ExtIR2R is not zero.
 * |        |          |Idle state cycle = (ExtIR2R * MCLK)
 * @var EBI_T::EBICON2
 * Offset: 0x08   External Bus Interface General Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WBUFF_EN  |EBI Write Buffer Enable Control
 * |        |          |0 = EBI write buffer Disabled.
 * |        |          |1 = EBI write buffer Enabled.c
 * |[1]     |RAHD_OFF  |Access Hold Time Disable Control When Read
 * |        |          |0 = tAHD is controlled by ExttAHD[2:0] when read through EBI.
 * |        |          |1 = Zero tAHD when read through EBI.
 * |[2]     |WAHD_OFF  |Access Hold Time Disable Control When Write 
 * |        |          |0 = tAHD is controlled by ExttAHD[2:0] when write through EBI.
 * |        |          |1 = Zero tAHD when write through EBI.
 */

    __IO uint32_t EBICON;        /* Offset: 0x00  EBI Control Register                                               */
    __IO uint32_t EXTIME;        /* Offset: 0x04  EBI Timing Control Register                                        */
    __IO uint32_t EBICON2;       /* Offset: 0x08   External Bus Interface General Control Register 2                 */

} EBI_T;



/**
    @addtogroup EBI_CONST EBI Bit Field Definition
    Constant Definitions for EBI Controller
@{ */

/* EBI EBICON Bit Field Definitions */
#define EBI_EBICON_ExttALE_Pos      16                                          /*!< EBI_T::EBICON: ExttALE Position */
#define EBI_EBICON_ExttALE_Msk      (0x7ul << EBI_EBICON_ExttALE_Pos)           /*!< EBI_T::EBICON: ExttALE Mask */

#define EBI_EBICON_MCLKDIV_Pos      8                                           /*!< EBI_T::EBICON: MCLKDIV Position */
#define EBI_EBICON_MCLKDIV_Msk      (0x7ul << EBI_EBICON_MCLKDIV_Pos)           /*!< EBI_T::EBICON: MCLKDIV Mask */

#define EBI_EBICON_ExtBW16_Pos      1                                           /*!< EBI_T::EBICON: ExtBW16 Position */
#define EBI_EBICON_ExtBW16_Msk      (1ul << EBI_EBICON_ExtBW16_Pos)             /*!< EBI_T::EBICON: ExtBW16 Mask */

#define EBI_EBICON_ExtEN_Pos        0                                           /*!< EBI_T::EBICON: ExtEN Position */
#define EBI_EBICON_ExtEN_Msk        (1ul << EBI_EBICON_ExtEN_Pos)               /*!< EBI_T::EBICON: ExtEN Mask */

/* EBI EXTIME Bit Field Definitions */
#define EBI_EXTIME_ExtIR2R_Pos      24                                          /*!< EBI_T::EXTIME: ExtIR2R Position */
#define EBI_EXTIME_ExtIR2R_Msk      (0xFul << EBI_EXTIME_ExtIR2R_Pos)           /*!< EBI_T::EXTIME: ExtIR2R Mask */

#define EBI_EXTIME_ExtIW2X_Pos      12                                          /*!< EBI_T::EXTIME: ExtIW2X Position */
#define EBI_EXTIME_ExtIW2X_Msk      (0xFul << EBI_EXTIME_ExtIW2X_Pos)           /*!< EBI_T::EXTIME: ExtIW2X Mask */

#define EBI_EXTIME_ExttAHD_Pos      8                                           /*!< EBI_T::EXTIME: ExttAHD Position */
#define EBI_EXTIME_ExttAHD_Msk      (0x7ul << EBI_EXTIME_ExttAHD_Pos)           /*!< EBI_T::EXTIME: ExttAHD Mask */

#define EBI_EXTIME_ExttACC_Pos      3                                           /*!< EBI_T::EXTIME: ExttACC Position */
#define EBI_EXTIME_ExttACC_Msk      (0x1Ful << EBI_EXTIME_ExttACC_Pos)          /*!< EBI_T::EXTIME: ExttACC Mask */

/* EBI EBICON2 Bit Field Definitions */
#define EBI_EBICON2_WAHD_OFF_Pos    2                                           /*!< EBI_T::EBICON2: WAHD_OFF Position */
#define EBI_EBICON2_WAHD_OFF_Msk    (1ul << EBI_EBICON2_WAHD_OFF_Pos)           /*!< EBI_T::EBICON2: WAHD_OFF Mask */

#define EBI_EBICON2_RAHD_OFF_Pos    1                                           /*!< EBI_T::EBICON2: RAHD_OFF Position */
#define EBI_EBICON2_RAHD_OFF_Msk    (1ul << EBI_EBICON2_RAHD_OFF_Pos)           /*!< EBI_T::EBICON2: RAHD_OFF Mask */

#define EBI_EBICON2_WBUFF_EN_Pos    0                                           /*!< EBI_T::EBICON2: WBUFF_EN Position */
#define EBI_EBICON2_WBUFF_EN_Msk    (1ul << EBI_EBICON2_WBUFF_EN_Pos)           /*!< EBI_T::EBICON2: WBUFF_EN Mask */
/*@}*/ /* end of group EBI_CONST */
/*@}*/ /* end of group EBI */

/**@}*/ /* end of REGISTER group */



#endif /* __EBI_REG_H__ */


