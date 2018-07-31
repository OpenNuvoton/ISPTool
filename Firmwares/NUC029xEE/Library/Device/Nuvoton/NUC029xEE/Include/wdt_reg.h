/**************************************************************************//**
 * @file     wdt_reg.h
 * @version  V1.00
 * @brief    CRC register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __WDT_REG_H__
#define __WDT_REG_H__


/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */



/*---------------------- Watch Dog Timer Controller -------------------------*/
/**
    @addtogroup WDT Watch Dog Timer Controller (WDT)
    Memory Mapped Structure for WDT Controller
@{ */


typedef struct
{


/**
 * @var WDT_T::WTCR
 * Offset: 0x00  Watchdog Timer Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WTR       |Reset Watchdog Timer Up Counter (Write Protect)
 * |        |          |0 = No effect.
 * |        |          |1 = Reset the internal 18-bit WDT up counter value.
 * |        |          |Note: This bit will be automatically cleared by hardware.
 * |[1]     |WTRE      |Watchdog Timer Reset Enable (Write Protect)
 * |        |          |Setting this bit will enable the WDT time-out reset function if the WDT up counter value has not
 * |        |          |been cleared after the specific WDT reset delay period expires.
 * |        |          |0 = WDT time-out reset function Disabled.
 * |        |          |1 = WDT time-out reset function Enabled.
 * |[2]     |WTRF      |Watchdog Timer Time-out Reset Flag
 * |        |          |This bit indicates the system has been reset by WDT time-out reset or not.
 * |        |          |0 = WDT time-out reset did not occur.
 * |        |          |1 = WDT time-out reset occurred.
 * |        |          |Note: This bit is cleared by writing 1 to it.
 * |[3]     |WTIF      |Watchdog Timer Time-out Interrupt Flag
 * |        |          |This bit will set to 1 while WDT up counter value reaches the selected WDT time-out interval.
 * |        |          |0 = WDT time-out interrupt did not occur.
 * |        |          |1 = WDT time-out interrupt occurred.
 * |        |          |Note: This bit is cleared by writing 1 to it.
 * |[4]     |WTWKE     |Watchdog Timer Time-out Wake-Up Function Control
 * |        |          |(Write Protect)
 * |        |          |If this bit is set to 1, while WTIF is generated to 1 and WTIE enabled, the WDT time-out
 * |        |          |interrupt signal will generate a wake-up trigger event to chip.
 * |        |          |0 = Wake-up trigger event Disabled if WDT time-out interrupt signal generated.
 * |        |          |1 = Wake-up trigger event Enabled if WDT time-out interrupt signal generated.
 * |        |          |Note: Chip can be woken-up by WDT time-out interrupt signal generated only if WDT clock source
 * |        |          |is selected to 10 kHz oscillator.
 * |[5]     |WTWKF     |Watchdog Timer Time-out Wake-Up Flag
 * |        |          |This bit indicates the interrupt wake-up flag status of WDT.
 * |        |          |0 = WDT does not cause chip wake-up.
 * |        |          |1 = Chip wake-up from Idle or Power-down mode if WDT time-out interrupt signal generated.
 * |        |          |Note: This bit is cleared by writing 1 to it.
 * |[6]     |WTIE      |Watchdog Timer Time-out Interrupt Enable Control (Write Protect)
 * |        |          |If this bit is enabled, the WDT time-out interrupt signal is generated and inform to CPU.
 * |        |          |0 = WDT time-out interrupt Disabled.
 * |        |          |1 = WDT time-out interrupt Enabled.
 * |[7]     |WTE       |Watchdog Timer Enable Control (Write Protect)
 * |        |          |0 = WDT Disabled. (This action will reset the internal up counter value.)
 * |        |          |1 = WDT Enabled.
 * |        |          |Note: If CWDTEN (CONFIG0[31] Watchdog Enable) bit is set to 0, this bit is forced as 1 and
 * |        |          | user cannot change this bit to 0.
 * |[10:8]  |WTIS      |Watchdog Timer Time-out Interval Selection (Write Protect)
 * |        |          |These three bits select the time-out interval period for the WDT.
 * |        |          |000 = 24 *TWDT.
 * |        |          |001 = 26 * TWDT.
 * |        |          |010 = 28 * TWDT.
 * |        |          |011 = 210 * TWDT.
 * |        |          |100 = 212 * TWDT.
 * |        |          |101 = 214 * TWDT.
 * |        |          |110 = 216 * TWDT.
 * |        |          |111 = 218 * TWDT.
 * |[31]    |DBGACK_WDT|ICE Debug Mode Acknowledge Disable Control (Write Protect)
 * |        |          |0 = ICE debug mode acknowledgement effects WDT counting.
 * |        |          |WDT up counter will be held while CPU is held by ICE.
 * |        |          |1 = ICE debug mode acknowledgement Disabled.
 * |        |          |WDT up counter will keep going no matter CPU is held by ICE or not.
 * @var WDT_T::WTCRALT
 * Offset: 0x04  Watchdog Timer Alternative Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |WTRDSEL   |Watchdog Timer Reset Delay Selection (Write Protect)
 * |        |          |When WDT time-out happened, user has a time named WDT Reset Delay Period to clear WDT counter to
 * |        |          |prevent WDT time-out reset happened.
 * |        |          |User can select a suitable value of WDT Reset Delay Period for different WDT time-out period.
 * |        |          |These bits are protected bit.
 * |        |          |It means programming this bit needs to write "59h", "16h", "88h" to address 0x5000_0100 to
 * |        |          |disable register protection.
 * |        |          |Reference the register REGWRPROT at address GCR_BA+0x100.
 * |        |          |00 = Watchdog Timer Reset Delay Period is 1026 * WDT_CLK.
 * |        |          |01 = Watchdog Timer Reset Delay Period is 130 * WDT_CLK.
 * |        |          |10 = Watchdog Timer Reset Delay Period is 18 * WDT_CLK.
 * |        |          |11 = Watchdog Timer Reset Delay Period is 3 * WDT_CLK.
 * |        |          |Note: This register will be reset to 0 if WDT time-out reset happened.
 */

    __IO uint32_t WTCR;          /* Offset: 0x00  Watchdog Timer Control Register                                    */
    __IO uint32_t WTCRALT;       /* Offset: 0x04  Watchdog Timer Alternative Control Register                        */

} WDT_T;



/**
    @addtogroup WDT_CONST WDT Bit Field Definition
    Constant Definitions for WDT Controller
@{ */



/* WDT WTCR Bit Field Definitions */
#define WDT_WTCR_DBGACK_WDT_Pos 31                                              /*!< WDT_T::WTCR: DBGACK_WDT Position */
#define WDT_WTCR_DBGACK_WDT_Msk (1ul << WDT_WTCR_DBGACK_WDT_Pos)                /*!< WDT_T::WTCR: DBGACK_WDT Mask */

#define WDT_WTCR_WTIS_Pos       8                                               /*!< WDT_T::WTCR: WTIS Position */
#define WDT_WTCR_WTIS_Msk       (0x7ul << WDT_WTCR_WTIS_Pos)                    /*!< WDT_T::WTCR: WTIS Mask */

#define WDT_WTCR_WTE_Pos        7                                               /*!< WDT_T::WTCR: WTE Position */
#define WDT_WTCR_WTE_Msk        (1ul << WDT_WTCR_WTE_Pos)                       /*!< WDT_T::WTCR: WTE Mask */

#define WDT_WTCR_WTIE_Pos       6                                               /*!< WDT_T::WTCR: WTIE Position */
#define WDT_WTCR_WTIE_Msk       (1ul << WDT_WTCR_WTIE_Pos)                      /*!< WDT_T::WTCR: WTIE Mask */

#define WDT_WTCR_WTWKF_Pos      5                                               /*!< WDT_T::WTCR: WTWKF Position */
#define WDT_WTCR_WTWKF_Msk      (1ul << WDT_WTCR_WTWKF_Pos)                     /*!< WDT_T::WTCR: WTWKF Mask */

#define WDT_WTCR_WTWKE_Pos      4                                               /*!< WDT_T::WTCR: WTWKE Position */
#define WDT_WTCR_WTWKE_Msk      (1ul << WDT_WTCR_WTWKE_Pos)                     /*!< WDT_T::WTCR: WTWKE Mask */

#define WDT_WTCR_WTIF_Pos       3                                               /*!< WDT_T::WTCR: WTIF Position */
#define WDT_WTCR_WTIF_Msk       (1ul << WDT_WTCR_WTIF_Pos)                      /*!< WDT_T::WTCR: WTIF Mask */

#define WDT_WTCR_WTRF_Pos       2                                               /*!< WDT_T::WTCR: WTRF Position */
#define WDT_WTCR_WTRF_Msk       (1ul << WDT_WTCR_WTRF_Pos)                      /*!< WDT_T::WTCR: WTRF Mask */

#define WDT_WTCR_WTRE_Pos       1                                               /*!< WDT_T::WTCR: WTRE Position */
#define WDT_WTCR_WTRE_Msk       (1ul << WDT_WTCR_WTRE_Pos)                      /*!< WDT_T::WTCR: WTRE Mask */

#define WDT_WTCR_WTR_Pos        0                                               /*!< WDT_T::WTCR: WTR Position */
#define WDT_WTCR_WTR_Msk        (1ul << WDT_WTCR_WTR_Pos)                       /*!< WDT_T::WTCR: WTR Mask */

/* WDT WTCRALT Bit Field Definitions */
#define WDT_WTCRALT_WTRDSEL_Pos 0                                               /*!< WDT_T::WTCRALT: WTRDSEL Position */
#define WDT_WTCRALT_WTRDSEL_Msk (0x3ul << WDT_WTCRALT_WTRDSEL_Pos)              /*!< WDT_T::WTCRALT: WTRDSEL Mask */
/*@}*/ /* end of group WDT_CONST */
/*@}*/ /* end of group WDT */
/**@}*/ /* end of REGISTER group */



#endif /* __WDT_REG_H__ */
