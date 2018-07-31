/**************************************************************************//**
 * @file     rtc_reg.h
 * @version  V1.00
 * @brief    RTC register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __RTC_REG_H__
#define __RTC_REG_H__


/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */




/*---------------------- Real Time Clock Controller -------------------------*/
/**
    @addtogroup RTC Real Time Clock Controller (RTC)
    Memory Mapped Structure for RTC Controller
@{ */


typedef struct
{


/**
 * @var RTC_T::INIR
 * Offset: 0x00  RTC Initiation Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |INIR0_Active|RTC Active Status (Read Only)
 * |        |          |0 = RTC is at reset state.
 * |        |          |1 = RTC is at normal active state.
 * |[31:1]  |INIR[31:1]|RTC Initiation
 * |        |          |When RTC block is powered on, RTC is at reset state.
 * |        |          |User has to write a number (0xa5eb1357) to INIR to make RTC leaving reset state.
 * |        |          |Once the INIR is written as 0xa5eb1357, the RTC will be in un-reset state permanently.
 * |        |          |The INIR is a write-only field and read value will be always 0.
 * @var RTC_T::AER
 * Offset: 0x04  RTC Access Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |AER       |RTC Register Access Enable Password (Write Only)
 * |        |          |Writing 0xA965 to this register will enable RTC access and keep 1024 RTC clocks.
 * |[16]    |ENF       |RTC Register Access Enable Flag (Read Only)
 * |        |          |0 = RTC register read/write access Disabled.
 * |        |          |1 = RTC register read/write access Enabled.
 * |        |          |This bit will be set after AER[15:0] is load a 0xA965, and will be cleared automatically after
 * |        |          |1024 RTC clocks.
 * @var RTC_T::FCR
 * Offset: 0x08  RTC Frequency Compensation Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |FRACTION  |Fraction Part
 * |        |          |Formula = (fraction part of detected value) x 60.
 * |        |          |Note: Digit in FCR must be expressed as hexadecimal number.
 * |[11:8]  |INTEGER   |Integer Part
 * @var RTC_T::TLR
 * Offset: 0x0C  Time Loading Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |1SEC      |1-Sec Time Digit (0~9)
 * |[6:4]   |10SEC     |10-Sec Time Digit (0~5)
 * |[11:8]  |1MIN      |1-Min Time Digit (0~9)
 * |[14:12] |10MIN     |10-Min Time Digit (0~5)
 * |[19:16] |1HR       |1-Hour Time Digit (0~9)
 * |[21:20] |10HR      |10-Hour Time Digit (0~2)
 * @var RTC_T::CLR
 * Offset: 0x10  Calendar Loading Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |1DAY      |1-Day Calendar Digit (0~9)
 * |[5:4]   |10DAY     |10-Day Calendar Digit (0~3)
 * |[11:8]  |1MON      |1-Month Calendar Digit (0~9)
 * |[12]    |10MON     |10-Month Calendar Digit (0~1)
 * |[19:16] |1YEAR     |1-Year Calendar Digit (0~9)
 * |[23:20] |10YEAR    |10-Year Calendar Digit (0~9)
 * @var RTC_T::TSSR
 * Offset: 0x14  Time Scale Selection Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |24H_12H   |24-Hour / 12-Hour Time Scale Selection
 * |        |          |It indicates that RTC TLR and TAR counter are in 24-hour time scale or 12-hour time scale.
 * |        |          |0 = 24-hour time scale selected.
 * |        |          |1 = 24-hour time scale selected.
 * @var RTC_T::DWR
 * Offset: 0x18  Day of the Week Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |DWR       |Day Of The Week Register
 * |        |          |000 = Sunday.
 * |        |          |001 = Monday.
 * |        |          |010 = Tuesday.
 * |        |          |011 = Wednesday.
 * |        |          |100 = Thursday.
 * |        |          |101 = Friday.
 * |        |          |110 = Saturday.
 * |        |          |111 = Reserved.
 * @var RTC_T::TAR
 * Offset: 0x1C  Time Alarm Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |1SEC      |1-Sec Time Digit of Alarm Setting (0~9)
 * |[6:4]   |10SEC     |10-Sec Time Digit of Alarm Setting (0~5)
 * |[11:8]  |1MIN      |1-Min Time Digit of Alarm Setting (0~9)
 * |[14:12] |10MIN     |10-Min Time Digit of Alarm Setting (0~5)
 * |[19:16] |1HR       |1-Hour Time Digit of Alarm Setting (0~9)
 * |[21:20] |10HR      |10-Hour Time Digit of Alarm Setting (0~2)
 * @var RTC_T::CAR
 * Offset: 0x20  Calendar Alarm Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |1DAY      |1-Day Calendar Digit of Alarm Setting (0~9)
 * |[5:4]   |10DAY     |10-Day Calendar Digit of Alarm Setting (0~3)
 * |[11:8]  |1MON      |1-Month Calendar Digit of Alarm Setting (0~9)
 * |[12]    |10MON     |10-Month Calendar Digit of Alarm Setting (0~1)
 * |[19:16] |1YEAR     |1-Year Calendar Digit of Alarm Setting (0~9)
 * |[23:20] |10YEAR    |10-Year Calendar Digit of Alarm Setting (0~9)
 * @var RTC_T::LIR
 * Offset: 0x24  Leap Year Indicator Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |LIR       |Leap Year Indication Register (Read Only)
 * |        |          |0 = This year is not a leap year.
 * |        |          |1 = This year is a leap year.
 * @var RTC_T::RIER
 * Offset: 0x28  RTC Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |AIER      |Alarm Interrupt Enable
 * |        |          |This bit is used to enable/disable RTC Alarm Interrupt, and generate an interrupt signal if AIF
 * |        |          |(RIIR[0] RTC Alarm Interrupt Flag) is set to 1.
 * |        |          |0 = RTC Alarm Interrupt Disabled.
 * |        |          |1 = RTC Alarm Interrupt Enabled.
 * |        |          |Note: This bit will also trigger a wake-up event while system runs in Idle/Power-down mode and
 * |        |          |RTC Alarm Interrupt signal generated.
 * |[1]     |TIER      |Time Tick Interrupt Enable
 * |        |          |This bit is used to enable/disable RTC Time Tick Interrupt, and generate an interrupt signal if
 * |        |          |TIF (RIIR[1] RTC Time Tick Interrupt Flag) is set to 1.
 * |        |          |0 = RTC Time Tick Interrupt Disabled.
 * |        |          |1 = RTC Time Tick Interrupt Enabled.
 * |        |          |Note: This bit will also trigger a wake-up event while system runs in Idle/Power-down mode and
 * |        |          |RTC Time Tick Interrupt signal generated.
 * |[2]     |SNOOPIER  |Snoop Detection Interrupt Enable
 * |        |          |This bit is used to enable/disable RTC Snoop Detection Interrupt, and generate an interrupt
 * |        |          |signal if SNOOPIF (RIIR[2] RTC Snoop Detection Interrupt Flag) is set to 1.
 * |        |          |0 = Snoop detected interrupt Disabled.
 * |        |          |1 = Snoop detected interrupt Enabled.
 * |        |          |Note: This bit will also trigger a wake-up event while system runs in Idle/Power-down mode and
 * |        |          |RTC Snoop Interrupt signal generated.
 * @var RTC_T::RIIR
 * Offset: 0x2C  RTC Interrupt Indicator Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |AIF       |RTC Alarm Interrupt Flag
 * |        |          |When RTC time counters TLR and CLR match the alarm setting time registers TAR and CAR, this bit
 * |        |          |will be set to 1 and an interrupt will be generated if RTC Alarm Interrupt enabled AIER
 * |        |          |(RIER[0]) is set to 1.
 * |        |          |Chip will be wake-up if RTC Alarm Interrupt is enabled when chip is at Power-down mode.
 * |        |          |0 = Alarm condition is not matched.
 * |        |          |1 = Alarm condition is matched.
 * |        |          |Note: Write 1 to clear this bit.
 * |[1]     |TIF       |RTC Time Tick Interrupt Flag
 * |        |          |When RTC time tick happened, this bit will be set to 1 and an interrupt will be generated if RTC
 * |        |          |Tick Interrupt enabled TIER (RIER[1]) is set to 1.
 * |        |          |Chip will also be wake-up if RTC Tick Interrupt is enabled and this bit is set to 1 when chip is
 * |        |          |running at Power-down mode.
 * |        |          |0 = Tick condition does not occur.
 * |        |          |1 = Tick condition occur.
 * |        |          |Note: Write 1 to clear to clear this bit.
 * |[2]     |SNOOPIF   |Snoop Detection Interrupt Flag
 * |        |          |When snooper pin transition event is detected, this bit is set to 1 and an interrupt is
 * |        |          |generated if Snoop Detection Interrupt enabled SNOOPIER (RIER[2]) is set to1.
 * |        |          |Chip will be wake-up from Power-down mode if Snoop Detection Interrupt is enabled.
 * |        |          |0 = No snoop event is detected.
 * |        |          |1 = Snoop event is detected.
 * |        |          |Note: Write 1 to clear this bit.
 * @var RTC_T::TTR
 * Offset: 0x30  RTC Time Tick Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |TTR       |Time Tick Register
 * |        |          |These bits are used to select RTC time tick period for Periodic Time Tick Interrupt request.
 * |        |          |000 = Time tick is 1 second.
 * |        |          |001 = Time tick is 1/2 second.
 * |        |          |010 = Time tick is 1/4 second.
 * |        |          |011 = Time tick is 1/8 second.
 * |        |          |100 = Time tick is 1/16 second.
 * |        |          |101 = Time tick is 1/32 second.
 * |        |          |110 = Time tick is 1/64 second.
 * |        |          |111 = Time tick is 1/28 second.
 * |        |          |Note: This register can be read back after the RTC register access enable bit ENF (AER[16]) is
 * |        |          |active.
 * @var RTC_T::SPRCTL
 * Offset: 0x3C  RTC Spare Functional Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SNOOPEN   |Snoop Detection Enable
 * |        |          |0 = Snoop detection function Disabled.
 * |        |          |1 = Snoop detection function Enabled.
 * |[1]     |SNOOPLEVEL|Snoop Detection Level Selection
 * |        |          |This bit controls snoop detect event is high level/rising edge or low level/falling edge.
 * |        |          |0 = Low level/Falling edge detection.
 * |        |          |1 = High level/Rising edge detection.
 * |[2]     |SPREN     |SPR Register Enable
 * |        |          |0 = Spare register is Disabled.
 * |        |          |1 = Spare register is Enabled.
 * |        |          |Note: When spare register is disabled, RTC SPR0 ~ SPR19 cannot be accessed.
 * |[3]     |SNOOPMODE |Snoop Detection Mode Selection
 * |        |          |This bit controls snoop detect event is edge or level detection.
 * |        |          |0 = Level detection.
 * |        |          |1 = Edge detection.
 * |[5]     |SPRCFLG   |Spare Register Clear Flag
 * |        |          |This bit indicates if the SPR0 ~ SPR19 content is cleared when snoop specify event is detected.
 * |        |          |0 = Spare register content is not cleared.
 * |        |          |1 = Spare register content is cleared.
 * |        |          |Note: Writes 1 to clear this bit.
 * |[7]     |SPRRDY    |SPR Register Ready
 * |        |          |This bit indicates if the registers SPRCTL, SPR0 ~ SPR19 are ready to be accessed.
 * |        |          |After user writing registers SPRCTL, SPR0 ~ SPR19, read this bit to check if these registers are
 * |        |          |updated done is necessary.
 * |        |          |0 = SPRCTL, SPR0 ~ SPR19 updating is in progress.
 * |        |          |1 = SPRCTL, SPR0 ~ SPR19 are updated done and ready to be accessed.
 * |        |          |Note: This bit is read only and any write to it won't take any effect.
 * @var RTC_T::SPR
 * Offset: 0x40  RTC Spare Register 0 ~ 19
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |SPARE     |SPARE Register
 * |        |          |This field is used to store back-up information defined by software.
 * |        |          |This field will be cleared by hardware automatically once a snooper pin event is detected.
 * |        |          |Before store back-up information in to SPARE register, software should write 0xA965 to AER to
 * |        |          |make sure register read/write enable.
 */

    __IO uint32_t INIR;          /* Offset: 0x00  RTC Initiation Register                                            */
    __IO uint32_t AER;           /* Offset: 0x04  RTC Access Enable Register                                         */
    __IO uint32_t FCR;           /* Offset: 0x08  RTC Frequency Compensation Register                                */
    __IO uint32_t TLR;           /* Offset: 0x0C  Time Loading Register                                              */
    __IO uint32_t CLR;           /* Offset: 0x10  Calendar Loading Register                                          */
    __IO uint32_t TSSR;          /* Offset: 0x14  Time Scale Selection Register                                      */
    __IO uint32_t DWR;           /* Offset: 0x18  Day of the Week Register                                           */
    __IO uint32_t TAR;           /* Offset: 0x1C  Time Alarm Register                                                */
    __IO uint32_t CAR;           /* Offset: 0x20  Calendar Alarm Register                                            */
    __I  uint32_t LIR;           /* Offset: 0x24  Leap Year Indicator Register                                       */
    __IO uint32_t RIER;          /* Offset: 0x28  RTC Interrupt Enable Register                                      */
    __IO uint32_t RIIR;          /* Offset: 0x2C  RTC Interrupt Indicator Register                                   */
    __IO uint32_t TTR;           /* Offset: 0x30  RTC Time Tick Register                                             */
    __I  uint32_t RESERVED[2];  
    __IO uint32_t SPRCTL;        /* Offset: 0x3C  RTC Spare Functional Control Register                              */
    __IO uint32_t SPR[20];       /* Offset: 0x40  RTC Spare Register 0 ~ 19                                          */

} RTC_T;


/**
    @addtogroup RTC_CONST RTC Bit Field Definition
    Constant Definitions for RTC Controller
@{ */



/* RTC INIR Bit Field Definitions */
#define RTC_INIR_INIR_Pos       0                                               /*!< RTC_T::INIR: INIR Position */
#define RTC_INIR_INIR_Msk       (0xFFFFFFFFul << RTC_INIR_INIR_Pos)             /*!< RTC_T::INIR: INIR Mask */

#define RTC_INIR_ACTIVE_Pos     0                                               /*!< RTC_T::INIR: ACTIVE Position */
#define RTC_INIR_ACTIVE_Msk     (1ul << RTC_INIR_ACTIVE_Pos)                    /*!< RTC_T::INIR: ACTIVE Mask */

/* RTC AER Bit Field Definitions */
#define RTC_AER_ENF_Pos         16                                              /*!< RTC_T::AER: ENF Position */
#define RTC_AER_ENF_Msk         (1ul << RTC_AER_ENF_Pos)                        /*!< RTC_T::AER: ENF Mask */

#define RTC_AER_AER_Pos         0                                               /*!< RTC_T::AER: AER Position */
#define RTC_AER_AER_Msk         (0xFFFFul << RTC_AER_AER_Pos)                   /*!< RTC_T::AER: AER Mask */

/* RTC FCR Bit Field Definitions */
#define RTC_FCR_INTEGER_Pos     8                                               /*!< RTC_T::FCR: INTEGER Position */
#define RTC_FCR_INTEGER_Msk     (0xFul << RTC_FCR_INTEGER_Pos)                  /*!< RTC_T::FCR: INTEGER Mask */

#define RTC_FCR_FRACTION_Pos    0                                               /*!< RTC_T::FCR: FRACTION Position */
#define RTC_FCR_FRACTION_Msk    (0x3Ful << RTC_FCR_FRACTION_Pos)                /*!< RTC_T::FCR: FRACTION Mask */

/* RTC TLR Bit Field Definitions */
#define RTC_TLR_10HR_Pos        20                                              /*!< RTC_T::TLR: 10HR Position */
#define RTC_TLR_10HR_Msk        (0x3ul << RTC_TLR_10HR_Pos)                     /*!< RTC_T::TLR: 10HR Mask */

#define RTC_TLR_1HR_Pos         16                                              /*!< RTC_T::TLR: 1HR Position */
#define RTC_TLR_1HR_Msk         (0xFul << RTC_TLR_1HR_Pos)                      /*!< RTC_T::TLR: 1HR Mask */

#define RTC_TLR_10MIN_Pos       12                                              /*!< RTC_T::TLR: 10MIN Position */
#define RTC_TLR_10MIN_Msk       (0x7ul << RTC_TLR_10MIN_Pos)                    /*!< RTC_T::TLR: 10MIN Mask */

#define RTC_TLR_1MIN_Pos        8                                               /*!< RTC_T::TLR: 1MIN Position */
#define RTC_TLR_1MIN_Msk        (0xFul << RTC_TLR_1MIN_Pos)                     /*!< RTC_T::TLR: 1MIN Mask */

#define RTC_TLR_10SEC_Pos       4                                               /*!< RTC_T::TLR: 10SEC Position */
#define RTC_TLR_10SEC_Msk       (0x7ul << RTC_TLR_10SEC_Pos)                    /*!< RTC_T::TLR: 10SEC Mask */

#define RTC_TLR_1SEC_Pos        0                                               /*!< RTC_T::TLR: 1SEC Position */
#define RTC_TLR_1SEC_Msk        (0xFul << RTC_TLR_1SEC_Pos)                     /*!< RTC_T::TLR: 1SEC Mask */

/* RTC CLR Bit Field Definitions */
#define RTC_CLR_10YEAR_Pos      20                                              /*!< RTC_T::CLR: 10YEAR Position */
#define RTC_CLR_10YEAR_Msk      (0xFul << RTC_CLR_10YEAR_Pos)                   /*!< RTC_T::CLR: 10YEAR Mask */

#define RTC_CLR_1YEAR_Pos       16                                              /*!< RTC_T::CLR: 1YEAR Position */
#define RTC_CLR_1YEAR_Msk       (0xFul << RTC_CLR_1YEAR_Pos)                    /*!< RTC_T::CLR: 1YEAR Mask */

#define RTC_CLR_10MON_Pos       12                                              /*!< RTC_T::CLR: 10MON Position */
#define RTC_CLR_10MON_Msk       (1ul << RTC_CLR_10MON_Pos)                      /*!< RTC_T::CLR: 10MON Mask */

#define RTC_CLR_1MON_Pos        8                                               /*!< RTC_T::CLR: 1MON Position */
#define RTC_CLR_1MON_Msk        (0xFul << RTC_CLR_1MON_Pos)                     /*!< RTC_T::CLR: 1MON Mask */

#define RTC_CLR_10DAY_Pos       4                                               /*!< RTC_T::CLR: 10DAY Position */
#define RTC_CLR_10DAY_Msk       (0x3ul << RTC_CLR_10DAY_Pos)                    /*!< RTC_T::CLR: 10DAY Mask */

#define RTC_CLR_1DAY_Pos        0                                               /*!< RTC_T::CLR: 1DAY Position */
#define RTC_CLR_1DAY_Msk        (0xFul << RTC_CLR_1DAY_Pos)                     /*!< RTC_T::CLR: 1DAY Mask */

/* RTC TSSR Bit Field Definitions */
#define RTC_TSSR_24H_12H_Pos    0                                               /*!< RTC_T::TSSR: 24H_12H Position */
#define RTC_TSSR_24H_12H_Msk    (1ul << RTC_TSSR_24H_12H_Pos)                   /*!< RTC_T::TSSR: 24H_12H Mask */

/* RTC DWR Bit Field Definitions */
#define RTC_DWR_DWR_Pos         0                                               /*!< RTC_T::DWR: DWR Position */
#define RTC_DWR_DWR_Msk         (0x7ul << RTC_DWR_DWR_Pos)                      /*!< RTC_T::DWR: DWR Mask */

/* RTC TAR Bit Field Definitions */
#define RTC_TAR_10HR_Pos        20                                              /*!< RTC_T::TAR: 10HR Position */
#define RTC_TAR_10HR_Msk        (0x3ul << RTC_TAR_10HR_Pos)                     /*!< RTC_T::TAR: 10HR Mask */

#define RTC_TAR_1HR_Pos         16                                              /*!< RTC_T::TAR: 1HR Position */
#define RTC_TAR_1HR_Msk         (0xFul << RTC_TAR_1HR_Pos)                      /*!< RTC_T::TAR: 1HR Mask */

#define RTC_TAR_10MIN_Pos       12                                              /*!< RTC_T::TAR: 10MIN Position */
#define RTC_TAR_10MIN_Msk       (0x7ul << RTC_TAR_10MIN_Pos)                    /*!< RTC_T::TAR: 10MIN Mask */

#define RTC_TAR_1MIN_Pos        8                                               /*!< RTC_T::TAR: 1MIN Position */
#define RTC_TAR_1MIN_Msk        (0xFul << RTC_TAR_1MIN_Pos)                     /*!< RTC_T::TAR: 1MIN Mask */

#define RTC_TAR_10SEC_Pos       4                                               /*!< RTC_T::TAR: 10SEC Position */
#define RTC_TAR_10SEC_Msk       (0x7ul << RTC_TAR_10SEC_Pos)                    /*!< RTC_T::TAR: 10SEC Mask */

#define RTC_TAR_1SEC_Pos        0                                               /*!< RTC_T::TAR: 1SEC Position */
#define RTC_TAR_1SEC_Msk        (0xFul << RTC_TAR_1SEC_Pos)                     /*!< RTC_T::TAR: 1SEC Mask */

/* RTC CAR Bit Field Definitions */
#define RTC_CAR_10YEAR_Pos      20                                              /*!< RTC_T::CAR: 10YEAR Position */
#define RTC_CAR_10YEAR_Msk      (0xFul << RTC_CAR_10YEAR_Pos)                   /*!< RTC_T::CAR: 10YEAR Mask */

#define RTC_CAR_1YEAR_Pos       16                                              /*!< RTC_T::CAR: 1YEAR Position */
#define RTC_CAR_1YEAR_Msk       (0xFul << RTC_CAR_1YEAR_Pos)                    /*!< RTC_T::CAR: 1YEAR Mask */

#define RTC_CAR_10MON_Pos       12                                              /*!< RTC_T::CAR: 10MON Position */
#define RTC_CAR_10MON_Msk       (1ul << RTC_CAR_10MON_Pos)                      /*!< RTC_T::CAR: 10MON Mask */

#define RTC_CAR_1MON_Pos        8                                               /*!< RTC_T::CAR: 1MON Position */
#define RTC_CAR_1MON_Msk        (0xFul << RTC_CAR_1MON_Pos)                     /*!< RTC_T::CAR: 1MON Mask */

#define RTC_CAR_10DAY_Pos       4                                               /*!< RTC_T::CAR: 10DAY Position */
#define RTC_CAR_10DAY_Msk       (0x3ul << RTC_CAR_10DAY_Pos)                    /*!< RTC_T::CAR: 10DAY Mask */

#define RTC_CAR_1DAY_Pos        0                                               /*!< RTC_T::CAR: 1DAY Position */
#define RTC_CAR_1DAY_Msk        (0xFul << RTC_CAR_1DAY_Pos)                     /*!< RTC_T::CAR: 1DAY Mask */

/* RTC LIR Bit Field Definitions */
#define RTC_LIR_LIR_Pos         0                                               /*!< RTC_T::LIR: LIR Position */
#define RTC_LIR_LIR_Msk         (1ul << RTC_LIR_LIR_Pos)                        /*!< RTC_T::LIR: LIR Mask */

/* RTC RIER Bit Field Definitions */
#define RTC_RIER_SNOOPIER_Pos   2                                               /*!< RTC_T::RIER: SNOOPIER Position */
#define RTC_RIER_SNOOPIER_Msk   (1ul << RTC_RIER_SNOOPIER_Pos)                  /*!< RTC_T::RIER: SNOOPIER Mask */

#define RTC_RIER_TIER_Pos       1                                               /*!< RTC_T::RIER: TIER Position */
#define RTC_RIER_TIER_Msk       (1ul << RTC_RIER_TIER_Pos)                      /*!< RTC_T::RIER: TIER Mask */

#define RTC_RIER_AIER_Pos       0                                               /*!< RTC_T::RIER: AIER Position */
#define RTC_RIER_AIER_Msk       (1ul << RTC_RIER_AIER_Pos)                      /*!< RTC_T::RIER: AIER Mask */

/* RTC RIIR Bit Field Definitions */
#define RTC_RIIR_SNOOPIF_Pos    2                                               /*!< RTC_T::RIIR: SNOOPIF Position */
#define RTC_RIIR_SNOOPIF_Msk    (1ul << RTC_RIIR_SNOOPIF_Pos)                   /*!< RTC_T::RIIR: SNOOPIF Mask */

#define RTC_RIIR_TIF_Pos        1                                               /*!< RTC_T::RIIR: TIF Position */
#define RTC_RIIR_TIF_Msk        (1ul << RTC_RIIR_TIF_Pos)                       /*!< RTC_T::RIIR: TIF Mask */

#define RTC_RIIR_AIF_Pos        0                                               /*!< RTC_T::RIIR: AIF Position */
#define RTC_RIIR_AIF_Msk        (1ul << RTC_RIIR_AIF_Pos)                       /*!< RTC_T::RIIR: AIF Mask */

/* RTC TTR Bit Field Definitions */
#define RTC_TTR_TTR_Pos         0                                               /*!< RTC_T::TTR: TTR Position */
#define RTC_TTR_TTR_Msk         (0x7ul << RTC_TTR_TTR_Pos)                      /*!< RTC_T::TTR: TTR Mask */

/* RTC SPRCTL Bit Field Definitions */
#define RTC_SPRCTL_SPRRDY_Pos       7                                           /*!< RTC_T::SPRCTL: SPRRDY Position */
#define RTC_SPRCTL_SPRRDY_Msk       (1ul << RTC_SPRCTL_SPRRDY_Pos)              /*!< RTC_T::SPRCTL: SPRRDY Mask */

#define RTC_SPRCTL_SPRCFLG_Pos      5                                           /*!< RTC_T::SPRCTL: SPRCFLG Position */
#define RTC_SPRCTL_SPRCFLG_Msk      (1ul << RTC_SPRCTL_SPRCFLG_Pos)             /*!< RTC_T::SPRCTL: SPRCFLG Mask */

#define RTC_SPRCTL_SNOOPMODE_Pos    3                                           /*!< RTC_T::SPRCTL: SNOOPMODE Position */
#define RTC_SPRCTL_SNOOPMODE_Msk    (1ul << RTC_SPRCTL_SNOOPMODE_Pos)           /*!< RTC_T::SPRCTL: SNOOPMODE Mask */

#define RTC_SPRCTL_SPREN_Pos        2                                           /*!< RTC_T::SPRCTL: SPREN Position */
#define RTC_SPRCTL_SPREN_Msk        (1ul << RTC_SPRCTL_SPREN_Pos)               /*!< RTC_T::SPRCTL: SPREN Mask */

#define RTC_SPRCTL_SNOOPLEVEL_Pos   1                                           /*!< RTC_T::SPRCTL: SNOOPLEVEL Position */
#define RTC_SPRCTL_SNOOPLEVEL_Msk   (1ul << RTC_SPRCTL_SNOOPLEVEL_Pos)          /*!< RTC_T::SPRCTL: SNOOPLEVEL Mask */

#define RTC_SPRCTL_SNOOPEN_Pos      0                                           /*!< RTC_T::SPRCTL: SNOOPEN Position */
#define RTC_SPRCTL_SNOOPEN_Msk      (1ul << RTC_SPRCTL_SNOOPEN_Pos)             /*!< RTC_T::SPRCTL: SNOOPEN Mask */
/*@}*/ /* end of group RTC_CONST */
/*@}*/ /* end of group RTC */
/**@}*/ /* end of REGISTER group */


#endif /* __RTC_REG_H__ */
