/**************************************************************************//**
 * @file     fmc_reg.h
 * @version  V1.00
 * @brief    FMC register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __FMC_REG_H__
#define __FMC_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */


/*---------------------- Flash Memory Controller -------------------------*/
/**
    @addtogroup FMC Flash Memory Controller (FMC)
    Memory Mapped Structure for FMC Controller
@{ */



typedef struct
{


/**
 * @var FMC_T::ISPCON
 * Offset: 0x00  ISP Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPEN     |ISP Enable
 * |        |          |This bit is protected bit. ISP function enable bit. Set this bit to enable ISP function.
 * |        |          |1 = Enable ISP function
 * |        |          |0 = Disable ISP function
 * |[1]     |BS        |Boot Select
 * |        |          |This bit is protected bit. Set/clear this bit to select next booting from LDROM/APROM,
 * |        |          |respectively. This bit also functions as MCU booting status flag, which can be used to check
 * |        |          |where
 * |        |          |MCU booted from. This bit is initiated with the inversed value of CBS in Config0 after power-
 * |        |          |on reset; It keeps the same value at other reset.
 * |        |          |1 = boot from LDROM
 * |        |          |0 = boot from APROM
 * |[4]     |CFGUEN    |Config Update Enable
 * |        |          |Writing this bit to 1 enables s/w to update Config value by ISP procedure regardless of program
 * |        |          |code is running in APROM or LDROM.
 * |        |          |1 = Config update enable
 * |        |          |0 = Config update disable
 * |[5]     |LDUEN     |LDROM Update Enable
 * |        |          |LDROM update enable bit.
 * |        |          |1 = LDROM can be updated when the MCU runs in APROM.
 * |        |          |0 = LDROM cannot be updated
 * |[6]     |ISPFF     |ISP Fail Flag
 * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
 * |        |          |(1) APROM writes to itself.
 * |        |          |(2) LDROM writes to itself.
 * |        |          |(3) Destination address is illegal, such as over an available range.
 * |        |          |Write 1 to clear.
 * |[7]     |SWRST     |Software Reset
 * |        |          |Writing 1 to this bit to start software reset.
 * |        |          |It is cleared by hardware after reset is finished.
 * |[10:8]  |PT        |Flash Program Time
 * |        |          |000 = 40 us
 * |        |          |001 = 45 us
 * |        |          |010 = 50 us
 * |        |          |011 = 55 us
 * |        |          |100 = 20 us
 * |        |          |101 = 25 us
 * |        |          |110 = 30 us
 * |        |          |111 = 35 us
 * |[14:12] |ET        |Flash Erase Time
 * |        |          |000 = 20 ms (default)
 * |        |          |001 = 25 ms
 * |        |          |010 = 30 ms
 * |        |          |011 = 35 ms
 * |        |          |100 = 3  ms
 * |        |          |101 = 5  ms
 * |        |          |110 = 10 ms
 * |        |          |111 = 15 ms
 * @var FMC_T::ISPADR
 * Offset: 0x04  ISP Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ISPADR    |ISP Address
 * |        |          |NuMicro NUC029xEE series has a maximum of 32Kx32 (128 KB) embedded Flash,
 * |        |          |which supports word program only. ISPADR[1:0] must be kept 00b for ISP operation.
 * @var FMC_T::ISPDAT
 * Offset: 0x08  ISP Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ISPDAT    |ISP Data
 * |        |          |Write data to this register before ISP program operation
 * |        |          |Read data from this register after ISP read operation
 * @var FMC_T::ISPCMD
 * Offset: 0x0C  ISP Command Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |FOEN_FCEN_FCTRL|ISP Command
 * |        |          |ISP command table is shown below:
 * |        |          |Operation Mode, FOEN, FCEN, FCTRL[3:0]
 * |        |          |Read          ,    0,    0, 0000
 * |        |          |Program       ,    1,    0, 0001
 * |        |          |Page Erase    ,    1,    0, 0010
 * @var FMC_T::ISPTRG
 * Offset: 0x10  IISP Trigger Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPGO     |ISP start trigger
 * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when
 * |        |          |ISP
 * |        |          |operation is finish.
 * |        |          |1 = ISP is on going
 * |        |          |0 = ISP done
 * @var FMC_T::DFBADR
 * Offset: 0x14  Data Flash Base Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |DFBA      |Data Flash Base Address
 * |        |          |This register indicates data flash start address.
 * |        |          |It is a read only register.
 * |        |          |For 8/16/32/64kB flash memory device, the data flash size is 4kB and it start address is fixed
 * |        |          |at
 * |        |          |0x01F000 by hardware internally.
 * @var FMC_T::FATCON
 * Offset: 0x18  Flash Access Time Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FPSEN     |Flash Power Save Enable
 * |        |          |If CPU clock is slower than 24 MHz, then s/w can enable flash power saving function.
 * |        |          |1 = Enable flash power saving
 * |        |          |0 = Disable flash power saving
 * |[3:1]   |FATS      |Flash Access Time Window Select
 * |        |          |These bits are used to decide flash sense amplifier active duration.
 * |        |          |000 = 40 ns
 * |        |          |001 = 50 ns
 * |        |          |010 = 60 ns
 * |        |          |011 = 70 ns
 * |        |          |100 = 80 ns
 * |        |          |101 = 90 ns
 * |        |          |110 = 100 ns
 * |        |          |111 = Reserved
 * |[4]     |L_SPEED   |Flash Low Speed Mode Enable
 * |        |          |1 = Flash access always no wait state (zero wait state)
 * |        |          |0 = Insert wait state while Flash access discontinued address.
 * |        |          |Note: Set this bit only when HCLK <= 25MHz. If HCLK > 25MHz, CPU will fetch wrong
 * |        |          |code and cause fail result.
 * @var FMC_T::ISPSTA
 * Offset: 0x40  ISP Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPGO     |ISP Start Trigger (Read Only)
 * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware
 * |        |          |automatically when ISP operation is finished.
 * |        |          |0 = ISP operation finished.
 * |        |          |1 = ISP operation progressed.
 * |        |          |Note: This bit is the same as ISPTRG bit0
 * |[2:1]   |CBS       |Chip Boot Selection (Read Only)
 * |        |          |This is a mirror of CBS in Config0.
 * |[6]     |ISPFF     |ISP Fail Flag (Write-protection Bit)
 * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
 * |        |          |(1) APROM writes to itself.
 * |        |          |(2) LDROM writes to itself.
 * |        |          |(3) CONFIG is erased/programmed when CFGUEN is set to 0
 * |        |          |(4) Destination address is illegal, such as over an available range.
 * |[20:9]  |VECMAP    |Vector Page Mapping Address (Read Only)
 * |        |          |The current flash address space 0x0000_0000~0x0000_01FF is mapping to the address
 * |        |          |{VECMAP[11:0], 9'h000} ~ {VECMAP[11:0], 9'h1FF}
 */

    __IO uint32_t ISPCON;        /* Offset: 0x00  ISP Control Register                                               */
    __IO uint32_t ISPADR;        /* Offset: 0x04  ISP Address Register                                               */
    __IO uint32_t ISPDAT;        /* Offset: 0x08  ISP Data Register                                                  */
    __IO uint32_t ISPCMD;        /* Offset: 0x0C  ISP Command Register                                               */
    __IO uint32_t ISPTRG;        /* Offset: 0x10  IISP Trigger Control Register                                      */
    __I  uint32_t DFBADR;        /* Offset: 0x14  Data Flash Base Address Register                                   */
    __IO uint32_t FATCON;        /* Offset: 0x18  Flash Access Time Control Register                                 */
    __I  uint32_t RESERVED[9];  
    __IO uint32_t ISPSTA;        /* Offset: 0x40  ISP Status Register                                                */

} FMC_T;


/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for FMC Controller
@{ */


/* FMC ISPCON Bit Field Definitions */
#define FMC_ISPCON_ET_Pos                       12                                      /*!< FMC_T::ISPCON: ET Position */
#define FMC_ISPCON_ET_Msk                       (7ul << FMC_ISPCON_ET_Pos)              /*!< FMC_T::ISPCON: ET Mask     */

#define FMC_ISPCON_PT_Pos                       8                                       /*!< FMC_T::ISPCON: PT Position */
#define FMC_ISPCON_PT_Msk                       (7ul << FMC_ISPCON_PT_Pos)              /*!< FMC_T::ISPCON: PT Mask     */

#define FMC_ISPCON_ISPFF_Pos                    6                                       /*!< FMC_T::ISPCON: ISPFF Position */
#define FMC_ISPCON_ISPFF_Msk                    (1ul << FMC_ISPCON_ISPFF_Pos)           /*!< FMC_T::ISPCON: ISPFF Mask */

#define FMC_ISPCON_LDUEN_Pos                    5                                       /*!< FMC_T::ISPCON: LDUEN Position */
#define FMC_ISPCON_LDUEN_Msk                    (1ul << FMC_ISPCON_LDUEN_Pos)           /*!< FMC_T::ISPCON: LDUEN Mask */

#define FMC_ISPCON_CFGUEN_Pos                   4                                       /*!< FMC_T::ISPCON: CFGUEN Position */
#define FMC_ISPCON_CFGUEN_Msk                   (1ul << FMC_ISPCON_CFGUEN_Pos)          /*!< FMC_T::ISPCON: CFGUEN Mask */

#define FMC_ISPCON_APUEN_Pos                    3                                       /*!< FMC_T::ISPCON: APUEN Position */
#define FMC_ISPCON_APUEN_Msk                    (1ul << FMC_ISPCON_APUEN_Pos)           /*!< FMC_T::ISPCON: APUEN Mask */

#define FMC_ISPCON_BS_Pos                       1                                       /*!< FMC_T::ISPCON: BS Position */
#define FMC_ISPCON_BS_Msk                       (0x1ul << FMC_ISPCON_BS_Pos)            /*!< FMC_T::ISPCON: BS Mask */

#define FMC_ISPCON_ISPEN_Pos                    0                                       /*!< FMC_T::ISPCON: ISPEN Position */
#define FMC_ISPCON_ISPEN_Msk                    (1ul << FMC_ISPCON_ISPEN_Pos)           /*!< FMC_T::ISPCON: ISPEN Mask */

/* FMC ISPADR Bit Field Definitions */
#define FMC_ISPADR_ISPADR_Pos                   0                                       /*!< FMC_T::ISPADR: ISPADR Position */
#define FMC_ISPADR_ISPADR_Msk                   (0xFFFFFFFFul << FMC_ISPADR_ISPADR_Pos) /*!< FMC_T::ISPADR: ISPADR Mask     */

/* FMC ISPADR Bit Field Definitions */
#define FMC_ISPDAT_ISPDAT_Pos                   0                                       /*!< FMC_T::ISPDAT: ISPDAT Position */
#define FMC_ISPDAT_ISPDAT_Msk                   (0xFFFFFFFFul << FMC_ISPDAT_ISPDAT_Pos) /*!< FMC_T::ISPDAT: ISPDAT Mask     */

/* FMC ISPCMD Bit Field Definitions */
#define FMC_ISPCMD_FOEN_Pos                     5                                       /*!< FMC_T::ISPCMD: FOEN Position */
#define FMC_ISPCMD_FOEN_Msk                     (1ul << FMC_ISPCMD_FOEN_Pos)            /*!< FMC_T::ISPCMD: FOEN Mask */

#define FMC_ISPCMD_FCEN_Pos                     4                                       /*!< FMC_T::ISPCMD: FCEN Position */
#define FMC_ISPCMD_FCEN_Msk                     (1ul << FMC_ISPCMD_FCEN_Pos)            /*!< FMC_T::ISPCMD: FCEN Mask */

#define FMC_ISPCMD_FCTRL_Pos                    0                                       /*!< FMC_T::ISPCMD: FCTRL Position */
#define FMC_ISPCMD_FCTRL_Msk                    (0xFul << FMC_ISPCMD_FCTRL_Pos)         /*!< FMC_T::ISPCMD: FCTRL Mask */

/* FMC ISPTRG Bit Field Definitions */
#define FMC_ISPTRG_ISPGO_Pos                    0                                       /*!< FMC_T::ISPTRG: ISPGO Position */
#define FMC_ISPTRG_ISPGO_Msk                    (1ul << FMC_ISPTRG_ISPGO_Pos)           /*!< FMC_T::ISPTRG: ISPGO Mask */

/* FMC DFBADR Bit Field Definitions */
#define FMC_DFBADR_DFBA_Pos                     0                                       /*!< FMC_T::DFBADR: DFBA Position */
#define FMC_DFBADR_DFBA_Msk                     (0xFFFFFFFFul << FMC_DFBADR_DFBA_Pos)   /*!< FMC_T::DFBADR: DFBA Mask     */

/* FMC FATCON Bit Field Definitions */
#define FMC_FATCON_FOMSEL1_Pos                  6                                       /*!< FMC_T::FATCON: FOMSEL1 Position */
#define FMC_FATCON_FOMSEL1_Msk                  (1ul << FMC_FATCON_FOMSEL1_Pos)         /*!< FMC_T::FATCON: FOMSEL1 Mask */

#define FMC_FATCON_FOMSEL0_Pos                  4                                       /*!< FMC_T::FATCON: FOMSEL0 Position */
#define FMC_FATCON_FOMSEL0_Msk                  (1ul << FMC_FATCON_FOMSEL0_Pos)         /*!< FMC_T::FATCON: FOMSEL0 Mask */

#define FMC_FATCON_FATS_Pos                     1                                       /*!< FMC_T::FATCON: FATS Position */
#define FMC_FATCON_FATS_Msk                     (7ul << FMC_FATCON_FATS_Pos)            /*!< FMC_T::FATCON: FATS Mask */

#define FMC_FATCON_FPSEN_Pos                    0                                       /*!< FMC_T::FATCON: FPSEN Position */
#define FMC_FATCON_FPSEN_Msk                    (1ul << FMC_FATCON_FPSEN_Pos)           /*!< FMC_T::FATCON: FPSEN Mask */


#define FMC_ISPSTA_ISPGO_Pos                    0                                       /*!< FMC_T::ISPSTA: ISPGO Position */
#define FMC_ISPSTA_ISPGO_Msk                    (1ul << FMC_ISPSTA_ISPGO_Pos)           /*!< FMC_T::ISPSTA: ISPGO Mask */

#define FMC_ISPSTA_CBS_Pos                      1                                       /*!< FMC_T::ISPSTA: CBS Position */
#define FMC_ISPSTA_CBS_Msk                      (0x3ul << FMC_ISPSTA_CBS_Pos)           /*!< FMC_T::ISPSTA: CBS Mask */

#define FMC_ISPSTA_ISPFF_Pos                    6                                       /*!< FMC_T::ISPSTA: ISPFF Position */
#define FMC_ISPSTA_ISPFF_Msk                    (0x3ul << FMC_ISPSTA_ISPFF_Pos)         /*!< FMC_T::ISPSTA: ISPFF Mask */

#define FMC_ISPSTA_VECMAP_Pos                   9                                       /*!< FMC_T::ISPSTA: VECMAP Position */
#define FMC_ISPSTA_VECMAP_Msk                   (0xFFFul << FMC_ISPSTA_VECMAP_Pos)      /*!< FMC_T::ISPSTA: VECMAP Mask */
/*@}*/ /* end of group FMC_CONST */
/*@}*/ /* end of group FMC */
/**@}*/ /* end of REGISTER group */




#endif /* __FMC_REG_H__ */
