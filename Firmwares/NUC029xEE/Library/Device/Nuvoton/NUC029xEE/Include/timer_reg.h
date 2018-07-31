/**************************************************************************//**
 * @file     timer_reg.h
 * @version  V1.00
 * @brief    TIMER register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __TIMER_REG_H__
#define __TIMER_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */



/*---------------------- Timer Controller -------------------------*/
/**
    @addtogroup TIMER Timer Controller (TIMER)
    Memory Mapped Structure for TMR Controller
@{ */


typedef struct
{


/**
 * @var TIMER_T::TCSR
 * Offset: 0x00  Timer Control and Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |PRESCALE  |Prescale Counter
 * |        |          |Timer input clock source is divided by (PRESCALE+1) before it is fed to the Timer up counter.
 * |        |          |If this field is 0 (PRESCALE = 0), then there is no scaling.
 * |[16]    |TDR_EN    |Data Load Enable Control
 * |        |          |When TDR_EN is set, TDR (Timer Data Register) will be updated continuously with the 24-bit
 * |        |          |up-timer value as the timer is counting.
 * |        |          |0 = Timer Data Register update Disabled.
 * |        |          |1 = Timer Data Register update Enabled while Timer counter is active.
 * |[23]    |WAKE_EN   |Wake Up Function Enable Control
 * |        |          |0 = Wake-up trigger event Disabled.
 * |        |          |1 = Wake-up trigger event Enabled.
 * |[24]    |CTB       |Counter Mode Enable Control
 * |        |          |This bit is for external counting pin function enabled.
 * |        |          |When timer is used as an event counter, this bit should be set to 1 and select HCLK as timer
 * |        |          |clock source.
 * |        |          |0 = External counter mode Disabled.
 * |        |          |1 = External counter mode Enabled.
 * |[25]    |CACT      |Timer Active Status (Read Only)
 * |        |          |This bit indicates the 24-bit up counter status.
 * |        |          |0 = 24-bit up counter is not active.
 * |        |          |1 = 24-bit up counter is active.
 * |[26]    |CRST      |Timer Reset
 * |        |          |0 = No effect.
 * |        |          |1 = Reset 8-bit prescale counter, 24-bit up counter value and CEN bit if CACT is 1.
 * |[28:27] |MODE      |Timer Operating Mode
 * |        |          |00 = The Timer controller is operated in One-shot mode.
 * |        |          |01 = The Timer controller is operated in Periodic mode.
 * |        |          |10 = The Timer controller is operated in Toggle-output mode.
 * |        |          |11 = The Timer controller is operated in Continuous Counting mode.
 * |[29]    |IE        |Interrupt Enable Control
 * |        |          |0 = Timer Interrupt function Disabled.
 * |        |          |1 = Timer Interrupt function Enabled.
 * |        |          |If this bit is enabled, when the timer interrupt flag (TISR[0] TIF) is set to 1, the timer
 * |        |          |interrupt signal is generated and inform to CPU.
 * |[30]    |CEN       |Timer Enable Control
 * |        |          |0 = Stops/Suspends counting.
 * |        |          |1 = Starts counting.
 * |        |          |Note1: In stop status, and then set CEN to 1 will enable the 24-bit up counter to keep counting
 * |        |          |from the last stop counting value.
 * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (TCSR [28:27] = 00) when the timer
 * |        |          |interrupt flag (TISR[0] TIF) is generated.
 * |[31]    |DBGACK_TMR|ICE Debug Mode Acknowledge Disable (Write Protect)
 * |        |          |0 = ICE debug mode acknowledgement effects TIMER counting.
 * |        |          |TIMER counter will be held while CPU is held by ICE.
 * |        |          |1 = ICE debug mode acknowledgement Disabled.
 * |        |          |TIMER counter will keep going no matter CPU is held by ICE or not.
 * @var TIMER_T::TCMPR
 * Offset: 0x04  Timer Compare Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |TCMP      |Timer Compared Value
 * |        |          |TCMP is a 24-bit compared value register.
 * |        |          |When the internal 24-bit up counter value is equal to TCMP value, the TIF flag will set to 1.
 * |        |          |Time-out period = (Period of Timer clock input) * (8-bit PRESCALE + 1) * (24-bit TCMP).
 * |        |          |Note1: Never write 0x0 or 0x1 in TCMP field, or the core will run into unknown state.
 * |        |          |Note2: When timer is operating at continuous counting mode, the 24-bit up counter will keep
 * |        |          |counting continuously even if user writes a new value into TCMP field.
 * |        |          |But if timer is operating at other modes, the 24-bit up counter will restart counting and using
 * |        |          |newest TCMP value to be the timer compared value if user writes a new value into TCMP field.
 * @var TIMER_T::TISR
 * Offset: 0x08  Timer Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TIF       |Timer Interrupt Flag
 * |        |          |This bit indicates the interrupt flag status of Timer while TDR value reaches to TCMP value.
 * |        |          |0 = No effect.
 * |        |          |1 = TDR value matches the TCMP value.
 * |        |          |Note: This bit is cleared by writing 1 to it.
 * |[1]     |TWF       |Timer Wake-Up Flag
 * |        |          |This bit indicates the interrupt wake-up flag status of Timer.
 * |        |          |0 = Timer does not cause CPU wake-up.
 * |        |          |1 = CPU wake-up from Idle or Power-down mode if Timer time-out interrupt signal generated.
 * |        |          |Note: This bit is cleared by writing 1 to it.
 * @var TIMER_T::TDR
 * Offset: 0x0C  Timer Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |TDR       |Timer Data Register
 * |        |          |If TDR_EN (TCSR[16]) is set to 1, TDR register will be updated continuously to monitor 24-bit up
 * |        |          |counter value.
 * @var TIMER_T::TCAP
 * Offset: 0x10  Timer Capture Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |TCAP      |Timer Capture Data Register
 * |        |          |When TEXIF flag is set to 1, the current TDR value will be auto-loaded into this TCAP filed
 * |        |          |immediately.
 * @var TIMER_T::TEXCON
 * Offset: 0x14  Timer External Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TX_PHASE  |Timer External Count Pin Phase Detect Selection
 * |        |          |This bit indicates the detection phase of TMx pin.
 * |        |          |0 = A falling edge of TMx pin will be counted.
 * |        |          |1 = A rising edge of TMx pin will be counted.
 * |[2:1]   |TEX_EDGE  |Timer External Capture Pin Edge Detect Selection
 * |        |          |00 = A 1 to 0 transition on TMx_EXT pin will be detected.
 * |        |          |01 = A 0 to 1 transition on TMx_EXT pin will be detected.
 * |        |          |10 = Either 1 to 0 or 0 to 1 transition on TMx_EXT pin will be detected.
 * |        |          |11 = Reserved.
 * |[3]     |TEXEN     |Timer External Pin Function Enable
 * |        |          |This bit enables the RSTCAPSEL function on the TxEX pin.
 * |        |          |0 = RSTCAPSEL function of TxEX pin will be ignored.
 * |        |          |1 = RSTCAPSEL function of TxEX pin is active.
 * |[4]     |RSTCAPSEL |Timer External Reset Counter / Timer External Capture Mode Selection
 * |        |          |0 = Transition on TMx_EXT
 * |        |          |pin is using to save the TDR value into TCAP value if TEXIF flag is set to 1.
 * |        |          |1 = Transition on TMx_EXT pin is using to reset the 24-bit up counter.
 * |[5]     |TEXIEN    |Timer External Capture Interrupt Enable Control
 * |        |          |0 = TMx_EXT pin detection Interrupt Disabled.
 * |        |          |1 = TMx_EXT pin detection Interrupt Enabled.
 * |        |          |If TEXIEN enabled, Timer will raise an external capture interrupt signal and inform to CPU while
 * |        |          |TEXIF flag is set to 1.
 * |[6]     |TEXDB     |Timer External Capture Input Pin De-Bounce Enable Control
 * |        |          |0 = TMx_EXT pin de-bounce Disabled.
 * |        |          |1 = TMx_EXT pin de-bounce Enabled.
 * |        |          |If this bit is enabled, the edge detection of TMx_EXT pin is detected with de-bounce circuit.
 * |[7]     |TCDB      |Timer External Counter Input Pin De-Bounce Enable Control
 * |        |          |0 = TMx pin de-bounce Disabled.
 * |        |          |1 = TMx pin de-bounce Enabled.
 * |        |          |If this bit is enabled, the edge detection of TMx pin is detected with de-bounce circuit.
 * @var TIMER_T::TEXISR
 * Offset: 0x18  Timer External Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TEXIF     |Timer External Capture Interrupt Flag
 * |        |          |This bit indicates the external capture interrupt flag status.
 * |        |          |When TEXEN enabled, TMx_EXT pin selected as external capture function, and a transition on
 * |        |          |TMx_EXT pin matched the TEX_EDGE setting, this flag will set to 1 by hardware.
 * |        |          |0 = TMx_EXT pin interrupt did not occur.
 * |        |          |1 = TMx_EXT pin interrupt occurred.
 * |        |          |Note: This bit is cleared by writing 1 to it.
 */

    __IO uint32_t TCSR;          /* Offset: 0x00  Timer Control and Status Register                                  */
    __IO uint32_t TCMPR;         /* Offset: 0x04  Timer Compare Register                                             */
    __IO uint32_t TISR;          /* Offset: 0x08  Timer Interrupt Status Register                                    */
    __I  uint32_t TDR;           /* Offset: 0x0C  Timer Data Register                                                */
    __I  uint32_t TCAP;          /* Offset: 0x10  Timer Capture Data Register                                        */
    __IO uint32_t TEXCON;        /* Offset: 0x14  Timer External Control Register                                    */
    __IO uint32_t TEXISR;        /* Offset: 0x18  Timer External Interrupt Status Register                           */

} TIMER_T;


/**
    @addtogroup TIMER_CONST TIMER Bit Field Definition
    Constant Definitions for TIMER Controller
@{ */


/* TIMER TCSR Bit Field Definitions */
#define TIMER_TCSR_DBGACK_TMR_Pos   31                                          /*!< TIMER_T::TCSR: DBGACK_TMR Position */
#define TIMER_TCSR_DBGACK_TMR_Msk   (1ul << TIMER_TCSR_DBGACK_TMR_Pos)          /*!< TIMER_T::TCSR: DBGACK_TMR Mask */

#define TIMER_TCSR_CEN_Pos          30                                          /*!< TIMER_T::TCSR: CEN Position */
#define TIMER_TCSR_CEN_Msk          (1ul << TIMER_TCSR_CEN_Pos)                 /*!< TIMER_T::TCSR: CEN Mask */

#define TIMER_TCSR_IE_Pos           29                                          /*!< TIMER_T::TCSR: IE Position */
#define TIMER_TCSR_IE_Msk           (1ul << TIMER_TCSR_IE_Pos)                  /*!< TIMER_T::TCSR: IE Mask */

#define TIMER_TCSR_MODE_Pos         27                                          /*!< TIMER_T::TCSR: MODE Position */
#define TIMER_TCSR_MODE_Msk         (0x3ul << TIMER_TCSR_MODE_Pos)              /*!< TIMER_T::TCSR: MODE Mask */

#define TIMER_TCSR_CRST_Pos         26                                          /*!< TIMER_T::TCSR: CRST Position */
#define TIMER_TCSR_CRST_Msk         (1ul << TIMER_TCSR_CRST_Pos)                /*!< TIMER_T::TCSR: CRST Mask */

#define TIMER_TCSR_CACT_Pos         25                                          /*!< TIMER_T::TCSR: CACT Position */
#define TIMER_TCSR_CACT_Msk         (1ul << TIMER_TCSR_CACT_Pos)                /*!< TIMER_T::TCSR: CACT Mask */

#define TIMER_TCSR_CTB_Pos          24                                          /*!< TIMER_T::TCSR: CTB Position */
#define TIMER_TCSR_CTB_Msk          (1ul << TIMER_TCSR_CTB_Pos)                 /*!< TIMER_T::TCSR: CTB Mask */

#define TIMER_TCSR_WAKE_EN_Pos      23                                          /*!< TIMER_T::TCSR: WAKE_EN Position */
#define TIMER_TCSR_WAKE_EN_Msk      (1ul << TIMER_TCSR_WAKE_EN_Pos)             /*!< TIMER_T::TCSR: WAKE_EN Mask */

#define TIMER_TCSR_TDR_EN_Pos       16                                          /*!< TIMER_T::TCSR: TDR_EN Position */
#define TIMER_TCSR_TDR_EN_Msk       (1ul << TIMER_TCSR_TDR_EN_Pos)              /*!< TIMER_T::TCSR: TDR_EN Mask */

#define TIMER_TCSR_PRESCALE_Pos     0                                           /*!< TIMER_T::TCSR: PRESCALE Position */
#define TIMER_TCSR_PRESCALE_Msk     (0xFFul << TIMER_TCSR_PRESCALE_Pos)         /*!< TIMER_T::TCSR: PRESCALE Mask */

/* TIMER TCMPR Bit Field Definitions */
#define TIMER_TCMP_TCMP_Pos         0                                           /*!< TIMER_T::TCMPR: TCMP Position */
#define TIMER_TCMP_TCMP_Msk         (0xFFFFFFul << TIMER_TCMP_TCMP_Pos)         /*!< TIMER_T::TCMPR: TCMP Mask */

/* TIMER TISR Bit Field Definitions */
#define TIMER_TISR_TWF_Pos          1                                           /*!< TIMER_T::TISR: TWF Position */
#define TIMER_TISR_TWF_Msk          (1ul << TIMER_TISR_TWF_Pos)                 /*!< TIMER_T::TISR: TWF Mask */

#define TIMER_TISR_TIF_Pos          0                                           /*!< TIMER_T::TISR: TIF Position */
#define TIMER_TISR_TIF_Msk          (1ul << TIMER_TISR_TIF_Pos)                 /*!< TIMER_T::TISR: TIF Mask */

/* TIMER TDR Bit Field Definitions */
#define TIMER_TDR_TDR_Pos           0                                           /*!< TIMER_T::TDR: TDR Position */
#define TIMER_TDR_TDR_Msk           (0xFFFFFFul << TIMER_TDR_TDR_Pos)           /*!< TIMER_T::TDR: TDR Mask */

/* TIMER TCAP Bit Field Definitions */
#define TIMER_TCAP_TCAP_Pos         0                                           /*!< TIMER_T::TCAP: TCAP Position */
#define TIMER_TCAP_TCAP_Msk         (0xFFFFFFul << TIMER_TCAP_TCAP_Pos)         /*!< TIMER_T::TCAP: TCAP Mask */

/* TIMER TEXCON Bit Field Definitions */
#define TIMER_TEXCON_TCDB_Pos       7                                           /*!< TIMER_T::TEXCON: TCDB Position */
#define TIMER_TEXCON_TCDB_Msk       (1ul << TIMER_TEXCON_TCDB_Pos)              /*!< TIMER_T::TEXCON: TCDB Mask */

#define TIMER_TEXCON_TEXDB_Pos      6                                           /*!< TIMER_T::TEXCON: TEXDB Position */
#define TIMER_TEXCON_TEXDB_Msk      (1ul << TIMER_TEXCON_TEXDB_Pos)             /*!< TIMER_T::TEXCON: TEXDB Mask */

#define TIMER_TEXCON_TEXIEN_Pos     5                                           /*!< TIMER_T::TEXCON: TEXIEN Position */
#define TIMER_TEXCON_TEXIEN_Msk     (1ul << TIMER_TEXCON_TEXIEN_Pos)            /*!< TIMER_T::TEXCON: TEXIEN Mask */

#define TIMER_TEXCON_RSTCAPSEL_Pos  4                                           /*!< TIMER_T::TEXCON: RSTCAPSEL Position */
#define TIMER_TEXCON_RSTCAPSEL_Msk  (1ul << TIMER_TEXCON_RSTCAPSEL_Pos)         /*!< TIMER_T::TEXCON: RSTCAPSEL Mask */

#define TIMER_TEXCON_TEXEN_Pos      3                                           /*!< TIMER_T::TEXCON: TEXEN Position */
#define TIMER_TEXCON_TEXEN_Msk      (1ul << TIMER_TEXCON_TEXEN_Pos)             /*!< TIMER_T::TEXCON: TEXEN Mask */

#define TIMER_TEXCON_TEX_EDGE_Pos   1                                           /*!< TIMER_T::TEXCON: TEX_EDGE Position */
#define TIMER_TEXCON_TEX_EDGE_Msk   (0x3ul << TIMER_TEXCON_TEX_EDGE_Pos)        /*!< TIMER_T::TEXCON: TEX_EDGE Mask */

#define TIMER_TEXCON_TX_PHASE_Pos   0                                           /*!< TIMER_T::TEXCON: TX_PHASE Position */
#define TIMER_TEXCON_TX_PHASE_Msk   (1ul << TIMER_TEXCON_TX_PHASE_Pos)          /*!< TIMER_T::TEXCON: TX_PHASE Mask */

/* TIMER TEXISR Bit Field Definitions */
#define TIMER_TEXISR_TEXIF_Pos      0                                           /*!< TIMER_T::TEXISR: TEXIF Position */
#define TIMER_TEXISR_TEXIF_Msk      (1ul << TIMER_TEXISR_TEXIF_Pos)             /*!< TIMER_T::TEXISR: TEXIF Mask */
/*@}*/ /* end of group TIMER_CONST */
/*@}*/ /* end of group TIMER */
/**@}*/ /* end of REGISTER group */


#endif /* __TIMER_REG_H__ */
