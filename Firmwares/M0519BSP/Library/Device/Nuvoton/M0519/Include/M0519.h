/**************************************************************************//**
 * @file     M0519.h
 * @version  V3.0
 * $Revision: 105 $
 * $Date: 15/07/07 5:00p $
 * @brief    M0519 Series Peripheral Access Layer Header File
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/



/**
  \mainpage Introduction
  *
  *
  * This user manual describes the usage of M0519 Series MCU device driver
  *
  * <b>Disclaimer</b>
  *
  * The Software is furnished "AS IS", without warranty as to performance or results, and
  * the entire risk as to performance or results is assumed by YOU. Nuvoton disclaims all
  * warranties, express, implied or otherwise, with regard to the Software, its use, or
  * operation, including without limitation any and all warranties of merchantability, fitness
  * for a particular purpose, and non-infringement of intellectual property rights.
  *
  * <b>Copyright Notice</b>
  *
  * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
  */

#ifndef __M0519_H__
#define __M0519_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
*/

/** @addtogroup CMSIS Device Definitions for CMSIS
  Interrupt Number Definition and Configurations for CMSIS
  @{
*/

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */

typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
    NonMaskableInt_IRQn         = -14,    /*!< Non Maskable Interrupt                                 */
    HardFault_IRQn              = -13,    /*!< Cortex-M0 Hard Fault Interrupt                         */
    SVCall_IRQn                 = -5,     /*!< Cortex-M0 SV Call Interrupt                            */
    PendSV_IRQn                 = -2,     /*!< Cortex-M0 Pend SV Interrupt                            */
    SysTick_IRQn                = -1,     /*!< Cortex-M0 System Tick Interrupt                        */

    /******  ARMIKMCU Swift specific Interrupt Numbers ************************************************/
    BOD_IRQn                  = 0,        /*!< Brown-Out Low Voltage Detected Interrupt               */
    WDT_IRQn                  = 1,        /*!< Watch Dog Timer Interrupt                              */
    EINT0_IRQn                = 2,        /*!< EINT0 Interrupt                                        */
    EINT1_IRQn                = 3,        /*!< EINT1 Interrupt                                        */
    GPG0_IRQn                 = 4,        /*!< GPIO_P0P1P2P3P4 Interrupt                              */
    GPG1_IRQn                 = 5,        /*!< GPIO_P5P6P7P8P9PA Interrupt                            */
    BPWM0_IRQn                 = 6,       /*!< BPWM0 Interrupt                                        */
    EADC0_IRQn                = 7,        /*!< EADC0 Interrupt                                        */
    TMR0_IRQn                 = 8,        /*!< TIMER0 Interrupt                                       */
    TMR1_IRQn                 = 9,        /*!< TIMER1 Interrupt                                       */
    TMR2_IRQn                 = 10,       /*!< TIMER2 Interrupt                                       */
    TMR3_IRQn                 = 11,       /*!< TIMER3 Interrupt                                       */
    UART0_IRQn                = 12,       /*!< UART0 Interrupt                                        */
    UART1_IRQn                = 13,       /*!< UART1 Interrupt                                        */
    SPI0_IRQn                 = 14,       /*!< SPI0 Interrupt                                         */
    SPI1_IRQn                 = 15,       /*!< SPI1 Interrupt                                         */
    SPI2_IRQn                 = 16,       /*!< SPI2 Interrupt                                         */
    REV_IRQn                  = 17,       
    I2C0_IRQn                 = 18,       /*!< I2C0 Interrupt                                         */
    CKD_IRQn                  = 19,       /*!< CKD Interrupt                                          */
    EPWM0_IRQn                = 21,       /*!< Enhanced PWM0 Interrupt                                */
    EPWM1_IRQn                = 22,       /*!< Enhanced PWM1 Interrupt                                */
    ECAP0_IRQn                = 23,       /*!< Enhanced Input Capture 0 Interrupt                     */
    ECAP1_IRQn                = 24,       /*!< Enhanced Input Capture 1 Interrupt                     */
    ACMP_IRQn                 = 25,       /*!< ACMP  Interrupt                                        */
    REV0_IRQn                 = 26,
    REV1_IRQn                 = 27,
    PWRWU_IRQn                = 28,       /*!< Power Down Wake Up Interrupt                           */
    EADC1_IRQn                = 29,       /*!< EADC1 Interrupt                                        */
    EADC2_IRQn                = 30,       /*!< EADC2 Interrupt                                        */
    EADC3_IRQn                = 31,       /*!< EADC3 Interrupt                                        */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __MPU_PRESENT           0       /*!< armikcmu does not provide a MPU present or not       */
#define __NVIC_PRIO_BITS        2       /*!< armikcmu Supports 2 Bits for the Priority Levels     */
#define __Vendor_SysTickConfig  0       /*!< Set to 1 if different SysTick Config is used         */


/*@}*/ /* end of group CMSIS */


#include "core_cm0.h"                   /* Cortex-M0 processor and core peripherals               */
#include "system_M0519.h"        /* M0519 System                                          */

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */

/*---------------------- Analog Comparator Controller -------------------------*/
/**
    @addtogroup ACMP Analog Comparator Controller (ACMP)
    Memory Mapped Structure for ACMP Controller
@{ */


typedef struct
{


    /**
     * @var ACMP_T::CR
     * Offset: 0x00 ~ 0x08 Analog Comparator 0 ~ 2 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMP_EN   |Comparator Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[1]     |ACMPIE    |Comparator Interrupt Enable
     * |        |          |0 = Interrupt function Disabled.
     * |        |          |1 = Interrupt function Enabled.
     * |[2]     |ACMP_HYS_EN |Comparator Hysteresis Enable
     * |        |          |0 = Hysteresis function Disabled (Default).
     * |        |          |1 = Hysteresis function Enabled.
     * |[3]     |CP        |Comparator Positive Input Selection
     * |        |          |0 = The comparator reference pin ACMPx_P (x=0,1) is selected as the positive comparator input.
     * |        |          |1 = The internal OP amplifier output is selected as the positive comparator input.
     * |        |          |Note: This bit field is reserved in ACMP2CR register.
     * |[4]     |CN        |Comparator Negative Input Selection
     * |        |          |0 = The comparator reference pin ACMPx_N (x=0,1,2) is selected as the negative comparator input.
     * |        |          |1 = Internal band-gap reference voltage is selected as the source of negative
     * |        |          |    comparator input.
     * |[6]     |ACMPINV  |Comparator Output Inverse Enable
     * |        |          |0 = Comparator analog output inverse function is Disabled.
     * |        |          |1 = Comparator analog output inverse function is Enabled.
     * @var ACMP_T::SR
     * Offset: 0x0C  Analog Comparator Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPF0    |Comparator 0 Interrupt Flag
     * |        |          |This bit is set by hardware whenever the comparator 0 output changes state.
     * |        |          |This will cause an interrupt if ACMPIE (ACMP0CR[1]) is set to 1.
     * |        |          |Write 1 to clear this bit to 0.
     * |[1]     |ACMPF1    |Comparator 1 Interrupt Flag
     * |        |          |This bit is set by hardware whenever the comparator 1 output changes state.
     * |        |          |This will cause an interrupt if ACMPIE (ACMP1CR[1]) is set to 1.
     * |        |          |Write 1 to clear this bit to 0.
     * |[2]     |ACMPF2    |Comparator 2 Interrupt Flag
     * |        |          |This bit is set by hardware whenever the comparator 2 output changes state.
     * |        |          |This will cause an interrupt if ACMPIE (ACMP2CR[1]) is set to 1.
     * |        |          |Write 1 to clear this bit to 0.
     * |[4]     |OPDF0     |OP Amplifier 0 Schmitt Trigger Digital Output Interrupt Flag
     * |        |          |OPDF0 interrupt flag is set by hardware whenever the OP amplifier 0 Schmitt
     * |        |          |trigger non-inverting buffer digital output changes state.
     * |        |          |Note: This bit is remapping from OPASR[4] and writing 1 to ACMPSR[4] or OPASR[4] can clear this bit.
     * |[5]     |OPDF1     |OP Amplifier 1 Schmitt Trigger Digital Output Interrupt Flag
     * |        |          |OPDF1 interrupt flag is set by hardware whenever the OP amplifier 1 Schmitt
     * |        |          |trigger non-inverting buffer digital output changes state.
     * |        |          |Note: This bit is remapping from OPASR[5] and writing 1 to ACMPSR[5] or OPASR[5] can clear this bit.
     * |[8]     |CO0       |Comparator 0 Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the comparator 0 is disabled (ACMP0_EN = 0).
     * |[9]     |CO1       |Comparator 1 Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the comparator 1 is disabled (ACMP1_EN = 0).
     * |[10]    |CO2       |Comparator 2 Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the comparator 2 is disabled (ACMP2_EN = 0).
     */

    __IO uint32_t CR[3];         /* Offset: 0x00 ~ 0x08 Analog Comparator 0 ~ 2 Control Register                     */
    __IO uint32_t SR;            /* Offset: 0x0C  Analog Comparator Status Register                                  */

} ACMP_T;



/**
    @addtogroup ACMP_CONST ACMP Bit Field Definition
    Constant Definitions for ACMP Controller
@{ */


/* CR Bit Field Definitions */
#define ACMP_CR_ACMPINV_Pos       6                                   /*!< ACMP_T::CR: ACMPINV Position */
#define ACMP_CR_ACMPINV_Msk       (1ul << ACMP_CR_ACMPINV_Pos)        /*!< ACMP_T::CR: ACMPINV Mask */

#define ACMP_CR_CN_Pos            4                                   /*!< ACMP_T::CR: CN Position */
#define ACMP_CR_CN_Msk            (1ul << ACMP_CR_CN_Pos)             /*!< ACMP_T::CR: CN Mask */

#define ACMP_CR_CP_Pos            3                                   /*!< ACMP_T::CR: CP Position */
#define ACMP_CR_CP_Msk            (1ul << ACMP_CR_CP_Pos)             /*!< ACMP_T::CR: CP Mask */

#define ACMP_CR_ACMP_HYS_EN_Pos   2                                   /*!< ACMP_T::CR: ACMP_HYS_EN Position */
#define ACMP_CR_ACMP_HYS_EN_Msk   (1ul << ACMP_CR_ACMP_HYS_EN_Pos)    /*!< ACMP_T::CR: ACMP_HYS_EN Mask */

#define ACMP_CR_ACMPIE_Pos        1                                   /*!< ACMP_T::CR: ACMPIE Position */
#define ACMP_CR_ACMPIE_Msk        (1ul << ACMP_CR_ACMPIE_Pos)         /*!< ACMP_T::CR: ACMPIE Mask */

#define ACMP_CR_ACMP_EN_Pos       0                                   /*!< ACMP_T::CR: ACMP_EN Position */
#define ACMP_CR_ACMP_EN_Msk       (1ul << ACMP_CR_ACMP_EN_Pos)        /*!< ACMP_T::CR: ACMP_EN Mask */

/* SR Bit Field Definitions */
#define ACMP_SR_CO2_Pos     10                               /*!< ACMP_T::SR: CO2 Position */
#define ACMP_SR_CO2_Msk     (1ul << ACMP_SR_CO2_Pos)         /*!< ACMP_T::SR: CO2 Mask */

#define ACMP_SR_CO1_Pos     9                                /*!< ACMP_T::SR: CO1 Position */
#define ACMP_SR_CO1_Msk     (1ul << ACMP_SR_CO1_Pos)         /*!< ACMP_T::SR: CO1 Mask */

#define ACMP_SR_CO0_Pos     8                                /*!< ACMP_T::SR: CO0 Position */
#define ACMP_SR_CO0_Msk     (1ul << ACMP_SR_CO0_Pos)         /*!< ACMP_T::SR: CO0 Mask */

#define ACMP_SR_OPDF1_Pos   5                                /*!< ACMP_T::SR: OPDF1 Position */
#define ACMP_SR_OPDF1_Msk   (1ul << ACMP_SR_OPDF1_Pos)       /*!< ACMP_T::SR: OPDF1 Mask */

#define ACMP_SR_OPDF0_Pos   4                                /*!< ACMP_T::SR: OPDF0 Position */
#define ACMP_SR_OPDF0_Msk   (1ul << ACMP_SR_OPDF0_Pos)       /*!< ACMP_T::SR: OPDF0 Mask */

#define ACMP_SR_ACMPF2_Pos   2                               /*!< ACMP_T::SR: ACMPF2 Position */
#define ACMP_SR_ACMPF2_Msk   (1ul << ACMP_SR_ACMPF2_Pos)     /*!< ACMP_T::SR: ACMPF2 Mask */

#define ACMP_SR_ACMPF1_Pos   1                               /*!< ACMP_T::SR: ACMPF1 Position */
#define ACMP_SR_ACMPF1_Msk   (1ul << ACMP_SR_ACMPF1_Pos)     /*!< ACMP_T::SR: ACMPF1 Mask */

#define ACMP_SR_ACMPF0_Pos   0                               /*!< ACMP_T::SR: ACMPF0 Position */
#define ACMP_SR_ACMPF0_Msk   (1ul << ACMP_SR_ACMPF0_Pos)     /*!< ACMP_T::SR: ACMPF0 Mask */

/*@}*/ /* end of group ACMP_CONST */
/*@}*/ /* end of group ACMP */



/*---------------------- Basic Pulse Width Modulation Controller -------------------------*/
/**
    @addtogroup Basic PWM Pulse Width Modulation Controller (BPWM)
    Memory Mapped Structure for BPWM Controller
@{ */

typedef struct
{


    /**
     * @var BPWM_T::PPR
     * Offset: 0x00  Basic PWM Prescaler Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits     |Field     |Descriptions
     * | :----:  | :----:   | :---- |
     * |[7:0]    |CP01      |Clock Prescaler
     * |         |          |Clock input is divided by (CP01 + 1) before it is fed to the corresponding PWM-timer
     * |         |          |If CP01=0, then the clock prescaler output clock will be stopped.
     * |         |          |So corresponding PWM-timer will also be stopped.
     * |[23:16]  |DZI01     |Dead-Zone Interval For Pair Of Channel 0 And Channel 1
     * |         |          |These 8-bit determine the Dead-zone length.
     * |         |          |The unit time of Dead-zone length = [(prescale+1)*(clock source divider)]/ BPWM_CLK.
     * @var BPWM_T::CSR
     * Offset: 0x04  Basic PWM Clock Source Divider Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits      |Field     |Descriptions
     * | :----:   | :----:   | :---- |
     * |[2:0]     |CSR0      |PWM Timer 0 Clock Source Divider Selection
     * |          |          |Select clock source divider for timer 0.
     * |          |          |(Table is the same as CSR1)
     * |[6:4]     |CSR1      |PWM Timer 1 Clock Source Divider Selection
     * |          |          |Select clock source divider for timer 1.
     * |          |          |000 = 2
     * |          |          |001 = 4
     * |          |          |010 = 8
     * |          |          |011 = 16
     * |          |          |100 = 1
     * @var BPWM_T::PCR
     * Offset: 0x08  Basic PWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits     |Field         |Descriptions
     * | :----:  | :----:       | :---- |
     * |[0]      |CH0EN         |PWM-Timer 0 Enable
     * |         |              |0 = The corresponding PWM-Timer stops running.
     * |         |              |1 = The corresponding PWM-Timer starts running.
     * |[1]      |CH0PINV       |PWM-Timer 0 Output Polar Inverse Enable
     * |         |              |0 = PWM0 output polar inverse disabled.
     * |         |              |1 = PWM0 output polar inverse enabled.
     * |[2]      |CH0INV        |PWM-Timer 0 Output Inverter Enable
     * |         |              |0 = Inverter disabled.
     * |         |              |1 = Inverter enabled.
     * |[3]      |CH0MOD        |PWM-Timer 0 Auto-Reload/One-Shot Mode
     * |         |              |0 = One-shot mode.
     * |         |              |1 = Auto-reload mode.
     * |         |              |Note: If there is a transition at this bit, it will cause CNR0 and CMR0 be cleared.
     * |[4]      |DZEN01        |Dead-Zone 0 Generator Enable
     * |         |              |0 = Disabled.
     * |         |              |1 = Enabled.
     * |         |              |Note: When Dead-zone generator is enabled, the pair of PWM0 and PWM1 becomes a complementary pair.
     * |[8]      |CH1EN         |PWM-Timer 1 Enable
     * |         |              |0 = Corresponding PWM-Timer stopped.
     * |         |              |1 = Corresponding PWM-Timer start running.
     * |[9]      |CH1PINV       |PWM-Timer 1 Output Polar Inverse Enable
     * |         |              |0 = PWM1 output polar inverse disabled.
     * |         |              |1 = PWM1 output polar inverse enabled.
     * |[10]     |CH1INV        |PWM-Timer 1 Output Inverter Enable
     * |         |              |0 = Inverter disable.
     * |         |              |1 = Inverter enable.
     * |[11]     |CH1MOD        |PWM-Timer 1 Auto-Reload/One-Shot Mode
     * |         |              |0 = One-shot mode.
     * |         |              |1 = Auto-reload mode.
     * |         |              |Note: If there is a transition at this bit, it will cause CNR1 and CMR1 be cleared.
     * |[30]     |PWM01TYPE     |PWM01 Aligned Type Selection Bit
     * |         |              |0 = Edge-aligned type.
     * |         |              |1 = Center-aligned type.
     * @var BPWM_T::CNR0
     * Offset: 0x0C  Basic PWM Counter Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRx      |PWM Timer Loaded Value
     * |        |          |CNR determines the PWM period.
     * |        |          |PWM frequency = BPWM_CLK/[(prescale+1)*(clock divider)*(CNR+1)].
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to 0xFFFE.
     * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When CNR value is set to 0, PWM output is always high.
     * @var BPWM_T::CMR0
     * Offset: 0x10  Basic PWM Comparator Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMRx      |PWM Comparator Register
     * |        |          |CMR determines the PWM duty.
     * |        |          |PWM frequency = BPWM_CLK/[(prescale+1)*(clock divider)*(CNR+1)].
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CMR will take effect in next PWM cycle.
     * @var BPWM_T::PDR0
     * Offset: 0x14  Basic PWM Data Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDRx      |PWM Data Register
     * |        |          |User can monitor PDR to know the current value in 16-bit counter.
     * @var BPWM_T::CNR1
     * Offset: 0x18  Basic PWM Counter Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRx      |PWM Timer Loaded Value
     * |        |          |CNR determines the PWM period.
     * |        |          |PWM frequency = BPWM_CLK/[(prescale+1)*(clock divider)*(CNR+1)].
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     * |        |          |Note: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to 0xFFFE.
     * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When CNR value is set to 0, PWM output is always high.
     * @var BPWM_T::CMR1
     * Offset: 0x1C  Basic PWM Comparator Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMRx      |PWM Comparator Register
     * |        |          |CMR determines the PWM duty.
     * |        |          |PWM frequency = BPWM_CLK/[(prescale+1)*(clock divider)*(CNR+1)].
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMR+1)/(CNR+1).
     * |        |          | CMR >= CNR: PWM output is always high.
     * |        |          | CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          | CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMR) + 1]/[2 x (CNR+1)].
     * |        |          | CMR > CNR: PWM output is always high.
     * |        |          | CMR <= CNR: PWM low width = 2 x (CNR-CMR) + 1 unit; PWM high width = (2 x CMR) + 1 unit.
     * |        |          | CMR = 0: PWM low width = 2 x CNR + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CMR will take effect in next PWM cycle.
     * @var BPWM_T::PDR1
     * Offset: 0x20  Basic PWM Data Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDRx      |PWM Data Register
     * |        |          |User can monitor PDR to know the current value in 16-bit counter.
     * @var BPWM_T::PIER
     * Offset: 0x40  Basic PWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWMPIE0   |PWM Channel 0 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[1]     |PWMPIE1   |PWM Channel 1 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[8]     |PWMDIE0   |PWM Channel 0 Duty Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[9]     |PWMDIE1   |PWM Channel 1 Duty Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[16]    |INTTYPE   |PWM Interrupt Period Type Selection Bit
     * |        |          |0 = PWMIFn will be set if PWM counter underflow.
     * |        |          |1 = PWMIFn will be set if PWM counter matches CNRn register.
     * |        |          |Note: This bit is effective when BPWM in Center-aligned type only.
     * @var BPWM_T::PIIR
     * Offset: 0x44  Basic PWM Interrupt Indication Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWMIF0    |PWM Channel 0 Period Interrupt Status
     * |        |          |This bit is set by hardware when channel 0 counter reaches the requirement of interrupt (depend on INTTYPE bit of PIER register), software can write 1 to clear this bit to 0.
     * |[1]     |PWMIF1    |PWM Channel 1 Period Interrupt Status
     * |        |          |This bit is set by hardware when channel 1 counter reaches the requirement of interrupt (depend on INTTYPE bit of PIER register), software can write 1 to clear this bit to 0.
     * |[8]     |PWMDIF0   |PWM Channel 0 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when channel 0 counter down count and reaches CMR0, software can clear this bit by writing a one to it.
     * |        |          |Note: If CMR equal to CNR, this flag is not working in Edge-aligned type selection.
     * |[9]     |PWMDIF1   |PWM Channel 1 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when channel 1 counter down count and reaches CMR1, software can clear this bit by writing a one to it.
     * |        |          |Note: If CMR equal to CNR, this flag is not working in Edge-aligned type selection.
     * @var BPWM_T::CCR
     * Offset: 0x50  Basic PWM Capture Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INV0      |Channel 0 Inverter Enable
     * |        |          |0 = Inverter disabled.
     * |        |          |1 = Inverter enabled. Reverse the input signal from GPIO before fed to capture timer.
     * |[1]     |CRL_IE0   |Channel 0 Rising Latch Interrupt Enable
     * |        |          |0 = Rising latch interrupt disabled.
     * |        |          |1 = Rising latch interrupt enabled.
     * |        |          |When enabled, if capture detects BPWM group channel 0 has rising transition, capture will issue an interrupt.
     * |[2]     |CFL_IE0   |Channel 0 Falling Latch Interrupt Enable
     * |        |          |0 = Falling latch interrupt disabled.
     * |        |          |1 = Falling latch interrupt enabled.
     * |        |          |When enabled, if capture detects BPWM group channel 0 has falling transition, capture will issue an interrupt.
     * |[3]     |CAPCH0EN  |Channel 0 Capture Function Enable
     * |        |          |0 = Capture function on BPWM group channel 0 disabled.
     * |        |          |1 = Capture function on BPWM group channel 0 enabled.
     * |        |          |When enabled, capture latched the PWM-counter value and saved to CRLR (Rising latch) and CFLR (Falling latch).
     * |        |          |When disabled, capture does not update CRLR and CFLR, and disable BPWM group channel 0 interrupt.
     * |[4]     |CAPIF0    |Channel 0 Capture Interrupt Indication Flag
     * |        |          |If BPWM channel 0 rising latch interrupt is enabled (CRL_IE0 = 1), a rising transition occurs at BPWM channel 0 will result in CAPIF0 to high; Similarly, a falling transition will cause CAPIF0 to be set high if BPWM channel 0 falling latch interrupt is enabled (CFL_IE0 = 1).
     * |        |          |Write 1 to clear this bit to 0.
     * |[6]     |CRLRI0    |CRLR0 Latched Indicator Bit
     * |        |          |When BPWM input channel 0 has a rising transition, CRLR0 was latched with the value of BPWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[7]     |CFLRI0    |CFLR0 Latched Indicator Bit
     * |        |          |When BPWM input channel 0 has a falling transition, CFLR0 was latched with the value of BPWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[16]    |INV1      |Channel 1 Inverter Enable
     * |        |          |0 = Inverter disabled.
     * |        |          |1 = Inverter enabled. Reverse the input signal from GPIO before fed to capture timer
     * |[17]    |CRL_IE1   |Channel 1 Rising Latch Interrupt Enable
     * |        |          |0 = Rising latch interrupt disabled.
     * |        |          |1 = Rising latch interrupt enabled.
     * |        |          |When enabled, if capture detects BPWM channel 1 has rising transition, capture will issue an interrupt.
     * |[18]    |CFL_IE1   |Channel 1 Falling Latch Interrupt Enable
     * |        |          |0 = Falling latch interrupt disabled.
     * |        |          |1 = Falling latch interrupt enabled.
     * |        |          |When enabled, if capture detects BPWM channel 1 has falling transition, capture will issue an interrupt.
     * |[19]    |CAPCH1EN  |Channel 1 Capture Function Enable
     * |        |          |0 = Capture function on BPWM channel 1 disabled.
     * |        |          |1 = Capture function on BPWM channel 1 enabled.
     * |        |          |When enabled, capture latched the BPWM-counter and saved to CRLR (Rising latch) and CFLR (Falling latch).
     * |        |          |When disabled, capture does not update CRLR and CFLR, and disable BPWM channel 1 interrupt.
     * |[20]    |CAPIF1    |Channel 1 Capture Interrupt Indication Flag
     * |        |          |If BPWM channel 1 rising latch interrupt is enabled (CRL_IE1 = 1), a rising transition occurs at BPWM channel 1 will result in CAPIF1 to high; Similarly, a falling transition will cause CAPIF1 to be set high if BPWM channel 1 falling latch interrupt is enabled (CFL_IE1 = 1).
     * |        |          |Write 1 to clear this bit to 0.
     * |[22]    |CRLRI1    |CRLR1 Latched Indicator Bit
     * |        |          |When BPWM input channel 1 has a rising transition, CRLR1 was latched with the value of BPWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[23]    |CFLRI1    |CFLR1 Latched Indicator Bit
     * |        |          |When BPWM input channel 1 has a falling transition, CFLR1 was latched with the value of BPWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * @var BPWM_T::CRLR0
     * Offset: 0x58  Basic PWM Capture Rising Latch Register (Channel 0)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRLRx     |Capture Rising Latch Register
     * |        |          |Latch the BPWM counter when channel 0/1 has rising transition.
     * @var BPWM_T::CFLR0
     * Offset: 0x5C  Basic PWM Capture Falling Latch Register (Channel 0)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CFLRx     |Capture Falling Latch Register
     * |        |          |Latch the BPWM counter when channel 0/1 has falling transition.
     * @var BPWM_T::CRLR1
     * Offset: 0x60  Basic PWM Capture Rising Latch Register (Channel 1)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRLRx     |Capture Rising Latch Register
     * |        |          |Latch the BPWM counter when channel 0/1 has rising transition.
     * @var BPWM_T::CFLR1
     * Offset: 0x64  Basic PWM Capture Falling Latch Register (Channel 1)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CFLRx     |Capture Falling Latch Register
     * |        |          |Latch the BPWM counter when channel 0/1 has falling transition.
     * @var BPWM_T::CAPENR
     * Offset: 0x78  Basic PWM Capture Input Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CINEN0    |Channel 0 Capture Input Enable
     * |        |          |0 = BPWM Channel 0 capture input path disabled.
     * |        |          |The input of BPWM channel 0 capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel 0 capture input path enabled.
     * |        |          |The input of BPWM channel 0 capture function comes from correlative multifunction pin if GPIO multi-function is set as BPWM0_CH0.
     * |[1]     |CINEN1    |Channel 1 Capture Input Enable
     * |        |          |0 = BPWM Channel 1 capture input path disabled.
     * |        |          |The input of BPWM channel 1 capture function is always regarded as 0.
     * |        |          |1 = BPWM Channel 1 capture input path enabled.
     * |        |          |The input of BPWM channel 1 capture function comes from correlative multifunction pin if GPIO multi-function is set as BPWM0_CH1.
     * @var BPWM_T::POE
     * Offset: 0x7C  Basic PWM Output Enable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |POE0      |Channel 0 Output Enable Register
     * |        |          |0 = BPWM channel 0 output to pin disabled.
     * |        |          |1 = BPWM channel 0 output to pin enabled.
     * |        |          |Note: The corresponding GPIO pin must also be switched to BPWM function
     * |[1]     |POE1      |Channel 1 Output Enable Register
     * |        |          |0 = BPWM channel 1 output to pin disabled.
     * |        |          |1 = BPWM channel 1 output to pin enabled.
     * |        |          |Note: The corresponding GPIO pin must also be switched to BPWM function
     */

    __IO uint32_t PPR;           /* Offset: 0x00  Basic PWM Prescaler Register                                       */
    __IO uint32_t CSR;           /* Offset: 0x04  Basic PWM Clock Source Divider Select Register                     */
    __IO uint32_t PCR;           /* Offset: 0x08  Basic PWM Control Register                                         */
    __IO uint32_t CNR0;          /* Offset: 0x0C  Basic PWM Counter Register 0                                       */
    __IO uint32_t CMR0;          /* Offset: 0x10  Basic PWM Comparator Register 0                                    */
    __I  uint32_t PDR0;          /* Offset: 0x14  Basic PWM Data Register 0                                          */
    __IO uint32_t CNR1;          /* Offset: 0x18  Basic PWM Counter Register 1                                       */
    __IO uint32_t CMR1;          /* Offset: 0x1C  Basic PWM Comparator Register 1                                    */
    __I  uint32_t PDR1;          /* Offset: 0x20  Basic PWM Data Register 1                                          */
    __I  uint32_t RESERVE0[7];
    __IO uint32_t PIER;          /* Offset: 0x40  Basic PWM Interrupt Enable Register                                */
    __IO uint32_t PIIR;          /* Offset: 0x44  Basic PWM Interrupt Indication Register                            */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t CCR;           /* Offset: 0x50  Basic PWM Capture Control Register                                 */
    __I  uint32_t RESERVE2[1];
    __IO uint32_t CRLR0;         /* Offset: 0x58  Basic PWM Capture Rising Latch Register (Channel 0)                */
    __IO uint32_t CFLR0;         /* Offset: 0x5C  Basic PWM Capture Falling Latch Register (Channel 0)               */
    __IO uint32_t CRLR1;         /* Offset: 0x60  Basic PWM Capture Rising Latch Register (Channel 1)                */
    __IO uint32_t CFLR1;         /* Offset: 0x64  Basic PWM Capture Falling Latch Register (Channel 1)               */
    __I  uint32_t RESERVE3[4];
    __IO uint32_t CAPENR;        /* Offset: 0x78  Basic PWM Capture Input Enable Register                            */
    __IO uint32_t POE;           /* Offset: 0x7C  Basic PWM Output Enable                                            */

} BPWM_T;



/**
    @addtogroup BPWM_CONST BPWM Bit Field Definition
    Constant Definitions for BPWM Controller
@{ */

/* BPWM PPR Bit Field Definitions */
#define BPWM_PPR_DZI01_Pos                       16                                  /*!< BPWM_T::PPR: DZI01 Position */
#define BPWM_PPR_DZI01_Msk                       (0xFFul << BPWM_PPR_DZI01_Pos)      /*!< BPWM_T::PPR: DZI01 Mask */

#define BPWM_PPR_CP01_Pos                        0                                   /*!< BPWM_T::PPR: CP01 Position */
#define BPWM_PPR_CP01_Msk                        (0xFFul << BPWM_PPR_CP01_Pos)       /*!< BPWM_T::PPR: CP01 Mask */

/* BPWM CSR Bit Field Definitions */
#define BPWM_CSR_CSR1_Pos                        4                                   /*!< BPWM_T::CSR: CSR1 Position */
#define BPWM_CSR_CSR1_Msk                        (7ul << BPWM_CSR_CSR1_Pos)          /*!< BPWM_T::CSR: CSR1 Mask */

#define BPWM_CSR_CSR0_Pos                        0                                   /*!< BPWM_T::CSR: CSR0 Position */
#define BPWM_CSR_CSR0_Msk                        (7ul << BPWM_CSR_CSR0_Pos)          /*!< BPWM_T::CSR: CSR0 Mask */

/* BPWM PCR Bit Field Definitions */
#define BPWM_PCR_PWM01TYPE_Pos                   30                                  /*!< BPWM_T::PCR: PWM01TYPE Position */
#define BPWM_PCR_PWM01TYPE_Msk                   (1ul << BPWM_PCR_PWM01TYPE_Pos)     /*!< BPWM_T::PCR: PWM01TYPE Mask */

#define BPWM_PCR_CH1MOD_Pos                      11                                  /*!< BPWM_T::PCR: CH1MOD Position */
#define BPWM_PCR_CH1MOD_Msk                      (1ul << BPWM_PCR_CH1MOD_Pos)        /*!< BPWM_T::PCR: CH1MOD Mask */

#define BPWM_PCR_CH1INV_Pos                      10                                  /*!< BPWM_T::PCR: CH1INV Position */
#define BPWM_PCR_CH1INV_Msk                      (1ul << BPWM_PCR_CH1INV_Pos)        /*!< BPWM_T::PCR: CH1INV Mask */

#define BPWM_PCR_CH1PINV_Pos                     9                                   /*!< BPWM_T::PCR: CH1PINV Position */
#define BPWM_PCR_CH1PINV_Msk                     (1ul << BPWM_PCR_CH1PINV_Pos)       /*!< BPWM_T::PCR: CH1PINV Mask */

#define BPWM_PCR_CH1EN_Pos                       8                                   /*!< BPWM_T::PCR: CH1EN Position */
#define BPWM_PCR_CH1EN_Msk                       (1ul << BPWM_PCR_CH1EN_Pos)         /*!< BPWM_T::PCR: CH1EN Mask */

#define BPWM_PCR_DZEN01_Pos                      4                                   /*!< BPWM_T::PCR: DZEN01 Position */
#define BPWM_PCR_DZEN01_Msk                      (1ul << BPWM_PCR_DZEN01_Pos)        /*!< BPWM_T::PCR: DZEN01 Mask */

#define BPWM_PCR_CH0MOD_Pos                      3                                   /*!< BPWM_T::PCR: CH0MOD Position */
#define BPWM_PCR_CH0MOD_Msk                      (1ul << BPWM_PCR_CH0MOD_Pos)        /*!< BPWM_T::PCR: CH0MOD Mask */

#define BPWM_PCR_CH0INV_Pos                      2                                   /*!< BPWM_T::PCR: CH0INV Position */
#define BPWM_PCR_CH0INV_Msk                      (1ul << BPWM_PCR_CH0INV_Pos)        /*!< BPWM_T::PCR: CH0INV Mask */

#define BPWM_PCR_CH0PINV_Pos                     1                                   /*!< BPWM_T::PCR: CH0PINV Position */
#define BPWM_PCR_CH0PINV_Msk                     (1ul << BPWM_PCR_CH0PINV_Pos)       /*!< BPWM_T::PCR: CH0PINV Mask */

#define BPWM_PCR_CH0EN_Pos                       0                                   /*!< BPWM_T::PCR: CH0EN Position */
#define BPWM_PCR_CH0EN_Msk                       (1ul << BPWM_PCR_CH0EN_Pos)         /*!< BPWM_T::PCR: CH0EN Mask */

/* PWM CNR Bit Field Definitions */
#define BPWM_CNR_CNR_Pos                         0                                   /*!< BPWM_T::CNR: CNR Position */
#define BPWM_CNR_CNR_Msk                         (0xFFFFul << BPWM_CNR_CNR_Pos)      /*!< BPWM_T::CNR: CNR Mask */

/* PWM CMR Bit Field Definitions */
#define BPWM_CMR_CMR_Pos                         0                                   /*!< BPWM_T::CMR: CMR Position */
#define BPWM_CMR_CMR_Msk                         (0xFFFFul << BPWM_CMR_CMR_Pos)      /*!< BPWM_T::CMR: CMR Mask */

/* PWM PDR Bit Field Definitions */
#define BPWM_PDR_PDR_Pos                         0                                   /*!< BPWM_T::PDR: PDR Position */
#define BPWM_PDR_PDR_Msk                         (0xFFFFul << BPWM_PDR_PDR_Pos)      /*!< BPWM_T::PDR: PDR Mask */

/* PWM PIER Bit Field Definitions */
#define BPWM_PIER_INTTYPE_Pos                    16                                  /*!< BPWM_T::PIER: INTTYPE Position */
#define BPWM_PIER_INTTYPE_Msk                    (1ul << BPWM_PIER_INTTYPE_Pos)      /*!< BPWM_T::PIER: INTTYPE Mask */

#define BPWM_PIER_PWMDIE1_Pos                    9                                   /*!< BPWM_T::PIER: PWMDIE1 Position */
#define BPWM_PIER_PWMDIE1_Msk                    (1ul << BPWM_PIER_PWMDIE1_Pos)      /*!< BPWM_T::PIER: PWMDIE1 Mask */

#define BPWM_PIER_PWMDIE0_Pos                    8                                   /*!< BPWM_T::PIER: PWMDIE0 Position */
#define BPWM_PIER_PWMDIE0_Msk                    (1ul << BPWM_PIER_PWMDIE0_Pos)      /*!< BPWM_T::PIER: PWMDIE0 Mask */

#define BPWM_PIER_PWMPIE1_Pos                    1                                   /*!< BPWM_T::PIER: PWMPIE1 Position */
#define BPWM_PIER_PWMPIE1_Msk                    (1ul << BPWM_PIER_PWMPIE1_Pos)      /*!< BPWM_T::PIER: PWMPIE1 Mask */

#define BPWM_PIER_PWMPIE0_Pos                    0                                   /*!< BPWM_T::PIER: PWMPIE0 Position */
#define BPWM_PIER_PWMPIE0_Msk                    (1ul << BPWM_PIER_PWMPIE0_Pos)      /*!< BPWM_T::PIER: PWMPIE0 Mask */

/* PWM PIIR Bit Field Definitions */
#define BPWM_PIIR_PWMDIF1_Pos                    9                                   /*!< BPWM_T::PIIR: PWMDIF1 Position */
#define BPWM_PIIR_PWMDIF1_Msk                    (1ul << BPWM_PIIR_PWMDIF1_Pos)      /*!< BPWM_T::PIIR: PWMDIF1 Mask */

#define BPWM_PIIR_PWMDIF0_Pos                    8                                   /*!< BPWM_T::PIIR: PWMDIF0 Position */
#define BPWM_PIIR_PWMDIF0_Msk                    (1ul << BPWM_PIIR_PWMDIF0_Pos)      /*!< BPWM_T::PIIR: PWMDIF0 Mask */

#define BPWM_PIIR_PWMIF1_Pos                     1                                   /*!< BPWM_T::PIIR: PWMIF1 Position */
#define BPWM_PIIR_PWMIF1_Msk                     (1ul << BPWM_PIIR_PWMIF1_Pos)       /*!< BPWM_T::PIIR: PWMIF1 Mask */

#define BPWM_PIIR_PWMIF0_Pos                     0                                   /*!< BPWM_T::PIIR: PWMIF0 Position */
#define BPWM_PIIR_PWMIF0_Msk                     (1ul << BPWM_PIIR_PWMIF0_Pos)       /*!< BPWM_T::PIIR: PWMIF0 Mask */

/* PWM CCR Bit Field Definitions */
#define BPWM_CCR_CFLRI1_Pos                      23                                  /*!< BPWM_T::CCR: CFLRI1 Position */
#define BPWM_CCR_CFLRI1_Msk                      (1ul << BPWM_CCR_CFLRI1_Pos)        /*!< BPWM_T::CCR: CFLRI1 Mask */

#define BPWM_CCR_CRLRI1_Pos                      22                                  /*!< BPWM_T::CCR: CRLRI1 Position */
#define BPWM_CCR_CRLRI1_Msk                      (1ul << BPWM_CCR_CRLRI1_Pos)        /*!< BPWM_T::CCR: CRLRI1 Mask */

#define BPWM_CCR_CAPIF1_Pos                      20                                  /*!< BPWM_T::CCR: CAPIF1 Position */
#define BPWM_CCR_CAPIF1_Msk                      (1ul << BPWM_CCR_CAPIF1_Pos)        /*!< BPWM_T::CCR: CAPIF1 Mask */

#define BPWM_CCR_CAPCH1EN_Pos                    19                                  /*!< BPWM_T::CCR: CAPCH1EN Position */
#define BPWM_CCR_CAPCH1EN_Msk                    (1ul << BPWM_CCR_CAPCH1EN_Pos)      /*!< BPWM_T::CCR: CAPCH1EN Mask */

#define BPWM_CCR_CFL_IE1_Pos                     18                                  /*!< BPWM_T::CCR: CFL_IE1 Position */
#define BPWM_CCR_CFL_IE1_Msk                     (1ul << BPWM_CCR_CFL_IE1_Pos)       /*!< BPWM_T::CCR: CFL_IE1 Mask */

#define BPWM_CCR_CRL_IE1_Pos                     17                                  /*!< BPWM_T::CCR: CRL_IE1 Position */
#define BPWM_CCR_CRL_IE1_Msk                     (1ul << BPWM_CCR_CRL_IE1_Pos)       /*!< BPWM_T::CCR: CRL_IE1 Mask */

#define BPWM_CCR_INV1_Pos                        16                                  /*!< BPWM_T::CCR: INV1 Position */
#define BPWM_CCR_INV1_Msk                        (1ul << BPWM_CCR_INV1_Pos)          /*!< BPWM_T::CCR: INV1 Mask */

#define BPWM_CCR_CFLRI0_Pos                      7                                   /*!< BPWM_T::CCR: CFLRI0 Position */
#define BPWM_CCR_CFLRI0_Msk                      (1ul << BPWM_CCR_CFLRI0_Pos)        /*!< BPWM_T::CCR: CFLRI0 Mask */

#define BPWM_CCR_CRLRI0_Pos                      6                                   /*!< BPWM_T::CCR: CRLRI0 Position */
#define BPWM_CCR_CRLRI0_Msk                      (1ul << BPWM_CCR_CRLRI0_Pos)        /*!< BPWM_T::CCR: CRLRI0 Mask */

#define BPWM_CCR_CAPIF0_Pos                      4                                   /*!< BPWM_T::CCR: CAPIF0 Position */
#define BPWM_CCR_CAPIF0_Msk                      (1ul << BPWM_CCR_CAPIF0_Pos)        /*!< BPWM_T::CCR: CAPIF0 Mask */

#define BPWM_CCR_CAPCH0EN_Pos                    3                                   /*!< BPWM_T::CCR: CAPCH0EN Position */
#define BPWM_CCR_CAPCH0EN_Msk                    (1ul << BPWM_CCR_CAPCH0EN_Pos)      /*!< BPWM_T::CCR: CAPCH0EN Mask */

#define BPWM_CCR_CFL_IE0_Pos                     2                                   /*!< BPWM_T::CCR: CFL_IE0 Position */
#define BPWM_CCR_CFL_IE0_Msk                     (1ul << BPWM_CCR_CFL_IE0_Pos)       /*!< BPWM_T::CCR: CFL_IE0 Mask */

#define BPWM_CCR_CRL_IE0_Pos                     1                                   /*!< BPWM_T::CCR: CRL_IE0 Position */
#define BPWM_CCR_CRL_IE0_Msk                     (1ul << BPWM_CCR_CRL_IE0_Pos)       /*!< BPWM_T::CCR: CRL_IE0 Mask */

#define BPWM_CCR_INV0_Pos                        0                                   /*!< BPWM_T::CCR: INV0 Position */
#define BPWM_CCR_INV0_Msk                        (1ul << BPWM_CCR_INV0_Pos)          /*!< BPWM_T::CCR: INV0 Mask */

/* PWM CRLR Bit Field Definitions */
#define BPWM_CRLR_CRLR_Pos                       0                                   /*!< BPWM_T::CRLR: CRLR Position */
#define BPWM_CRLR_CRLR_Msk                       (0xFFFFul << BPWM_CRLR_CRLR_Pos)    /*!< BPWM_T::CRLR: CRLR Mask */

/* PWM CFLR Bit Field Definitions */
#define BPWM_CFLR_CFLR_Pos                       0                                   /*!< BPWM_T::CFLR: CFLR Position */
#define BPWM_CFLR_CFLR_Msk                       (0xFFFFul << BPWM_CFLR_CFLR_Pos)    /*!< BPWM_T::CFLR: CFLR Mask */

/* PWM CAPENR Bit Field Definitions */
#define BPWM_CAPENR_CINEN1_Pos                   1                                   /*!< BPWM_T::CAPENR: CINEN1 Position */
#define BPWM_CAPENR_CINEN1_Msk                   (1ul << BPWM_CAPENR_CINEN1_Pos)     /*!< BPWM_T::CAPENR: CINEN1 Mask */

#define BPWM_CAPENR_CINEN0_Pos                   0                                   /*!< BPWM_T::CAPENR: CINEN0 Position */
#define BPWM_CAPENR_CINEN0_Msk                   (1ul << BPWM_CAPENR_CINEN0_Pos)     /*!< BPWM_T::CAPENR: CINEN0 Mask */

/* PWM POE Bit Field Definitions */
#define BPWM_POE_POE1_Pos                        1                                   /*!< BPWM_T::POE: POE1 Position */
#define BPWM_POE_POE1_Msk                        (1ul << BPWM_POE_POE1_Pos)          /*!< BPWM_T::POE: POE1 Mask */

#define BPWM_POE_POE0_Pos                        0                                   /*!< BPWM_T::POE: POE0 Position */
#define BPWM_POE_POE0_Msk                        (1ul << BPWM_POE_POE0_Pos)          /*!< BPWM_T::POE: POE0 Mask */

/*@}*/ /* end of group BPWM_CONST */
/*@}*/ /* end of group BPWM */


/*---------------------------- Clock Controller ------------------------------*/
/** @addtogroup CLK System Clock Controller (CLK)
  Memory Mapped Structure for System Clock Controller
  @{
 */

typedef struct
{



    /**
     * @var CLK_T::PWRCON
     * Offset: 0x00  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |XTL12M_EN |External 4~24 MHz High Speed Crystal Enable (HXT) Control (Write Protect)
     * |        |          |The bit default value is set by flash controller user configuration register CFOSC (Config0[26:24]).
     * |        |          |When the default clock source is from external 4~24 MHz high speed crystal, this bit is set to 1 automatically.
     * |        |          |0 = External 4~24 MHz high speed crystal oscillator (HXT) Disabled.
     * |        |          |1 = External 4~24 MHz high speed crystal oscillator (HXT) Enabled.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[2]     |OSC22M_EN |Internal 22.1184 MHz High Speed Oscillator (HIRC) Enable Control (Write Protect)
     * |        |          |0 = Internal 22.1184 MHz high speed oscillator (HIRC) Disabled.
     * |        |          |1 = Internal 22.1184 MHz high speed oscillator (HIRC) Enabled.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[3]     |OSC10K_EN |Internal 10 KHz Low Speed Oscillator (LIRC) Enable Control (Write Protect)
     * |        |          |0 = Internal 10 kHz low speed oscillator (LIRC) Disabled.
     * |        |          |1 = Internal 10 kHz low speed oscillator (LIRC) Enabled.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[5]     |PD_WU_INT_EN|Power-Down Mode Wake-Up Interrupt Enable Control (Write Protect)
     * |        |          |0 = Power-down mode wake-up interrupt Disabled.
     * |        |          |1 = Power-down mode wake-up interrupt Enabled.
     * |        |          |Note1: The interrupt will occur when both PD_WU_STS and PD_WU_INT_EN are high.
     * |        |          |Note2: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[6]     |PD_WU_STS |Power-down Mode Wake-up Interrupt Status
     * |        |          |Set by "Power-down wake-up event", it indicates that resume from Power-down mode. 
     * |        |          |This bit can be cleared to 0 by software writing "1".
     * |        |          |Note: This bit is working only if PD_WU_INT_EN (PWRCON[5]) set to 1.
     * @var CLK_T::AHBCLK
     * Offset: 0x04  AHB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |HDIV_EN   |Hardware Divider Controller Clock Enable Control
     * |        |          |0 = Hardware Divider controller clock Disabled.
     * |        |          |1 = Hardware Divider controller clock Enabled.
     * @var CLK_T::APBCLK
     * Offset: 0x08  APB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDT_EN    |Watchdog Timer Clock Enable Control (Write Protect)
     * |        |          |0 = Watchdog Timer clock Disabled.
     * |        |          |1 = Watchdog Timer clock Enabled.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[2]     |TMR0_EN   |Timer0 Clock Enable Control
     * |        |          |0 = Timer0 clock Disabled.
     * |        |          |1 = Timer0 clock Enabled.
     * |[3]     |TMR1_EN   |Timer1 Clock Enable Control
     * |        |          |0 = Timer1 clock Disabled.
     * |        |          |1 = Timer1 clock Enabled.
     * |[4]     |TMR2_EN   |Timer2 Clock Enable Control
     * |        |          |0 = Timer2 clock Disabled.
     * |        |          |1 = Timer2 clock Enabled.
     * |[5]     |TMR3_EN   |Timer3 Clock Enable Control
     * |        |          |0 = Timer3 clock Disabled.
     * |        |          |1 = Timer3 clock Enabled.
     * |[6]     |FDIV_EN   |Frequency Divider Output Clock Enable Control
     * |        |          |0 = Frequency divider output clock Disabled.
     * |        |          |1 = Frequency divider output clock Enabled.
     * |[8]     |I2C0_EN   |I2C0 Clock Enable Control
     * |        |          |0 = I2C0 clock Disabled.
     * |        |          |1 = I2C0 clock Enabled.
     * |[12]    |SPI0_EN   |SPI0 Clock Enable Control
     * |        |          |0 = SPI0 clock Disabled.
     * |        |          |1 = SPI0 clock Enabled.
     * |[13]    |SPI1_EN   |SPI1 Clock Enable Control
     * |        |          |0 = SPI1 clock Disabled.
     * |        |          |1 = SPI1 clock Enabled.
     * |[14]    |SPI2_EN   |SPI2 Clock Enable Control
     * |        |          |0 = SPI2 clock Disabled.
     * |        |          |1 = SPI2 clock Enabled.
     * |[16]    |UART0_EN  |UART0 Clock Enable Control
     * |        |          |0 = UART0 clock Disabled.
     * |        |          |1 = UART0 clock Enabled.
     * |[17]    |UART1_EN  |1 Clock Enable Control
     * |        |          |0 = UART1 clock Disabled.
     * |        |          |1 = UART1 clock Enabled.
     * |[19]    |BPWM0_EN  |Basic PWM0 Clock Enable Control
     * |        |          |0 = Basic PWM0 clock Disabled.
     * |        |          |1 = Basic PWM0 clock Enabled.
     * |[20]    |EPWM0_EN  |Enhanced PWM0 Clock Enable Control
     * |        |          |0 = Enhanced PWM0 clock Disabled.
     * |        |          |1 = Enhanced PWM0 clock Enabled.
     * |[21]    |EPWM1_EN  |Enhanced PWM1 Clock Enable Control
     * |        |          |0 = Enhanced PWM1 clock Disabled.
     * |        |          |1 = Enhanced PWM1 clock Enabled.
     * |[22]    |ACMP_EN   |Analog Comparator Clock Enable Control
     * |        |          |0 = Analog comparator clock Disabled.
     * |        |          |1 = Analog comparator clock Enabled.
     * |[26]    |ECAP0_EN  |Enhanced Input Capture 0 Clock Enable Control
     * |        |          |0 = Enhanced input capture 0 clock Disabled.
     * |        |          |1 = Enhanced input capture 0 clock Enabled.
     * |[27]    |ECAP1_EN  |Enhanced Input Capture 1 Clock Enable Control
     * |        |          |0 = Enhanced input capture 1 clock Disabled.
     * |        |          |1 = Enhanced input capture 1 clock Enabled.
     * |[28]    |EADC_EN   |EADC Clock Enable Control
     * |        |          |0 = EADC clock Disabled.
     * |        |          |1 = EADC clock Enabled.
     * |[29]    |OPA_EN    |OPA0 and OPA1 Clock Enable Control
     * |        |          |0 = OPA0 and OPA1 clock Disabled.
     * |        |          |1 = OPA0 and OPA1 clock Enabled.
     * @var CLK_T::CLKSTATUS
     * Offset: 0x0C  Clock Status Monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |XTL12M_STB|External 4~24 MHz High Speed Crystal (HXT) Clock Source Stable Flag (Read Only)
     * |        |          |0 = External 4~24 MHz high speed crystal clock (HXT) is not stable or disabled.
     * |        |          |1 = External 4~24 MHz high speed crystal clock (HXT) is stable and enabled.
     * |[2]     |PLL_STB   |Internal PLL Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal PLL clock is not stable or disabled.
     * |        |          |1 = Internal PLL clock is stable in normal mode.
     * |[3]     |OSC10K_STB|Internal 10 KHz Low Speed Oscillator (LIRC) Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal 10 kHz low speed oscillator clock (LIRC) is not stable or disabled.
     * |        |          |1 = Internal 10 kHz low speed oscillator clock (LIRC) is stable and enabled.
     * |[4]     |OSC22M_STB|Internal 22.1184 MHz High Speed Oscillator (HIRC) Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal 22.1184 MHz high speed oscillator (HIRC) clock is not stable or disabled.
     * |        |          |1 = Internal 22.1184 MHz high speed oscillator (HIRC) clock is stable and enabled.
     * |[7]     |CLK_SW_FAIL|Clock Switching Fail Flag (Read Only)
     * |        |          |0 = Clock switching success.
     * |        |          |1 = Clock switching failure.
     * |        |          |This bit is an index that if current system clock source is match as user defined at HCLK_S (CLKSEL0[2:0]).
     * |        |          |When user switch system clock, the system clock source will keep old clock until the new clock is stable.
     * |        |          |During the period that waiting new clock stable, this bit will be an index shows system clock source is not match as user wanted.
     * @var CLK_T::CLKSEL0
     * Offset: 0x10  Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HCLK_S    |HCLK Clock Source Selection (Write Protect)
     * |        |          |1. Before clock switching, the related clock sources (both pre-select and new-select) must be turn on.
     * |        |          |2. The 3-bit default value is reloaded from the value of CFOSC (Config0[26:24]) in user configuration register of Flash controller by any reset. Therefore the default value is either 000b or 111b.
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |010 = Clock source from PLL clock.
     * |        |          |011 = Clock source from internal 10 kHz low speed oscillator clock.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |Note: These bits are write protected bit. Refer to the REGWRPROT register.
     * |[5:3]   |STCLK_S   |Cortex-M0 SysTick Clock Source Selection (Write Protect)
     * |        |          |If SYST_CSR[2] = 1, SysTick clock source is from HCLK.
     * |        |          |If SYST_CSR[2] = 0, SysTick clock source is defined by STCLK_S(CLKSEL0[5:3]).
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
     * |        |          |010 = Clock source from external 4~24 MHz high speed crystal clock/2.
     * |        |          |011 = Clock source from HCLK/2.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock/2.
     * |        |          |Note1: These bits are write protected bit. Refer to the REGWRPROT register.
     * |        |          |Note2: if SysTick clock source is not from HCLK (i.e. SYST_CSR[2] = 0), SysTick clock source must less than or equal to HCLK/2.
     * @var CLK_T::CLKSEL1
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDT_S     |Watchdog Timer Clock Source Select (Write Protect)
     * |        |          |00 = Clock source from HCLK/128 clock.
     * |        |          |01 = Clock source from HCLK/512 clock.
     * |        |          |10 = Clock source from HCLK/2048 clock.
     * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
     * |        |          |Note: These bits are write protected bit. Refer to the REGWRPROT register.
     * |[4]     |SPI0_S    |SPI0 Clock Source Selection
     * |        |          |0 = Clock source from PLL clock.
     * |        |          |1 = Clock source from HCLK.
     * |[5]     |SPI1_S    |SPI1 Clock Source Selection
     * |        |          |0 = Clock source from PLL clock.
     * |        |          |1 = Clock source from HCLK.
     * |[6]     |SPI2_S    |SPI2 Clock Source Selection
     * |        |          |0 = Clock source from PLL clock.
     * |        |          |1 = Clock source from HCLK.
     * |[25:24] |UART_S    |UART Clock Source Selection
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |01 = Clock source from PLL clock.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * @var CLK_T::CLKDIV
     * Offset: 0x18  Clock Divider Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLK_N    |HCLK Clock Divide Number From HCLK Clock Source
     * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLK_N + 1).
     * |[11:8]  |UART_N    |UART Clock Divide Number From UART Clock Source
     * |        |          |UART clock frequency = (UART clock source frequency) / (UART_N + 1).
     * |[23:16] |EADC_N    |EADC Clock Divide Number From EADC Clock Source
     * |        |          |EADC clock frequency = (EADC clock source frequency) / (EADC_N + 1).
     * @var CLK_T::CLKSEL2
     * Offset: 0x1C  Clock Source Select Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:2]   |FRQDIV_S  |Clock Divider Clock Source Selection
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |[17:16] |WWDT_S    |Window Watchdog Timer Clock Source Selection
     * |        |          |10 = Clock source from HCLK/2048 clock.
     * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
     * @var CLK_T::PLLCON
     * Offset: 0x20  PLL Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |FB_DV     |PLL Feedback Divider Control Bits
     * |        |          |Refer to the PLL formulas.
     * |[13:9]  |IN_DV     |PLL Input Divider Control Bits
     * |        |          |Refer to the PLL formulas.
     * |[15:14] |OUT_DV    |PLL Output Divider Control Bits
     * |        |          |Refer to the PLL formulas.
     * |[16]    |PD        |Power-Down Mode
     * |        |          |0 = PLL is in Normal mode.
     * |        |          |1 = PLL is in Power-down mode (default).
     * |[17]    |BP        |PLL Bypass Control
     * |        |          |0 = PLL is in Normal mode (default).
     * |        |          |1 = PLL clock output is same as PLL source clock input.
     * |[18]    |OE        |PLL OE (FOUT Enable) Control
     * |        |          |0 = PLL FOUT Enabled.
     * |        |          |1 = PLL FOUT is fixed low.
     * |[19]    |PLL_SRC   |PLL Source Clock Selection
     * |        |          |0 = PLL source clock from external 4~24 MHz high speed crystal.
     * |        |          |1 = PLL source clock from internal 22.1184 MHz high speed oscillator.
     * |[20]    |FCO_SEL   |PLL FCO Selection
     * |        |          |0 = When the FCO frequency range between 100MHz and 200MHz, this bit should be set as 0.
     * |        |          |1 = When the FCO frequency range between 200MHz to 500MHz, this bit should be set as 1.
     * @var CLK_T::FRQDIV
     * Offset: 0x24  Frequency Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FSEL      |Divider Output Frequency Selection Bits
     * |        |          |The formula of output frequency is Fout = Fin/(2^(N+1)).
     * |        |          |Fin is the input clock frequency.
     * |        |          |Fout is the frequency of divider output clock.
     * |        |          |N is the 4-bit value of FSEL[3:0].
     * |[4]     |DIVIDER_EN|Frequency Divider Enable Bit
     * |        |          |0 = Frequency Divider function Disabled.
     * |        |          |1 = Frequency Divider function Enabled.
     * |[5]     |DIV1      |Frequency Divider Divide 1 Enable Bit
     * |        |          |0 = Frequency divider will output clock with source frequency divide by FSEL.
     * |        |          |1 = Frequency divider will output clock with source frequency.
     * @var CLK_T::CLKDCR
     * Offset: 0x28  Clock Detect Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CKD_EN    |Clock Detect Enable Bit
     * |        |          |0 = Clock detect Disabled.
     * |        |          |1 = Clock detect Enabled.
     * |[1]     |CKD_IEN   |Clock Detect Interrupt Enable Bit
     * |        |          |0 = Clock detect interrupt Disabled.
     * |        |          |1 = Clock detect interrupt Enabled.
     * @var CLK_T::CLKDSR
     * Offset: 0x2C  Clock Detect Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CKSTOP_IF |Clock Stop Interrupt Flag
     * |        |          |When system clock stop is detected, this bit will be set. Software can write 1 to clear this bit.
     * |[1]     |CKSTOP    |Clock Stop Status
     * |        |          |This bit reflect clock detect circuit result.
     */

    __IO uint32_t PWRCON;        /* Offset: 0x00  System Power-down Control Register                                 */
    __IO uint32_t AHBCLK;        /* Offset: 0x04  AHB Devices Clock Enable Control Register                          */
    __IO uint32_t APBCLK;        /* Offset: 0x08  APB Devices Clock Enable Control Register                          */
    __IO uint32_t CLKSTATUS;     /* Offset: 0x0C  Clock Status Monitor Register                                      */
    __IO uint32_t CLKSEL0;       /* Offset: 0x10  Clock Source Select Control Register 0                             */
    __IO uint32_t CLKSEL1;       /* Offset: 0x14  Clock Source Select Control Register 1                             */
    __IO uint32_t CLKDIV;        /* Offset: 0x18  Clock Divider Number Register                                      */
    __IO uint32_t CLKSEL2;       /* Offset: 0x1C  Clock Source Select Control Register 2                             */
    __IO uint32_t PLLCON;        /* Offset: 0x20  PLL Control Register                                               */
    __IO uint32_t FRQDIV;        /* Offset: 0x24  Frequency Divider Control Register                                 */
    __IO uint32_t CLKDCR;        /* Offset: 0x28  Clock Detect Control Register                                      */
    __IO uint32_t CLKDSR;        /* Offset: 0x2C  Clock Detect Status Register                                       */

} CLK_T;



/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

/* CLK PWRCON Bit Field Definitions */
#define CLK_PWRCON_PD_WU_STS_Pos             6                                    /*!< CLK_T::PWRCON: PD_WU_STS Position */
#define CLK_PWRCON_PD_WU_STS_Msk             (1ul << CLK_PWRCON_PD_WU_STS_Pos)    /*!< CLK_T::PWRCON: PD_WU_STS Mask */

#define CLK_PWRCON_PD_WU_INT_EN_Pos          5                                    /*!< CLK_T::PWRCON: PD_WU_INT_EN Position */
#define CLK_PWRCON_PD_WU_INT_EN_Msk          (1ul << CLK_PWRCON_PD_WU_INT_EN_Pos) /*!< CLK_T::PWRCON: PD_WU_INT_EN Mask */

#define CLK_PWRCON_OSC10K_EN_Pos             3                                    /*!< CLK_T::PWRCON: OSC10K_EN Position */
#define CLK_PWRCON_OSC10K_EN_Msk             (1ul << CLK_PWRCON_OSC10K_EN_Pos)    /*!< CLK_T::PWRCON: OSC10K_EN Mask */
#define CLK_PWRCON_IRC10K_EN_Pos             3                                    /*!< CLK_T::PWRCON: IRC10K_EN Position */
#define CLK_PWRCON_IRC10K_EN_Msk             (1ul << CLK_PWRCON_IRC10K_EN_Pos)    /*!< CLK_T::PWRCON: IRC10K_EN Mask */

#define CLK_PWRCON_OSC22M_EN_Pos             2                                    /*!< CLK_T::PWRCON: OSC22M_EN Position */
#define CLK_PWRCON_OSC22M_EN_Msk             (1ul << CLK_PWRCON_OSC22M_EN_Pos)    /*!< CLK_T::PWRCON: OSC22M_EN Mask */
#define CLK_PWRCON_IRC22M_EN_Pos             2                                    /*!< CLK_T::PWRCON: IRC22M_EN Position */
#define CLK_PWRCON_IRC22M_EN_Msk             (1ul << CLK_PWRCON_IRC22M_EN_Pos)    /*!< CLK_T::PWRCON: IRC22M_EN Mask */

#define CLK_PWRCON_XTL12M_EN_Pos             0                                    /*!< CLK_T::PWRCON: XTL12M_EN Position */
#define CLK_PWRCON_XTL12M_EN_Msk             (1ul << CLK_PWRCON_XTL12M_EN_Pos)    /*!< CLK_T::PWRCON: XTL12M_EN Mask */

/* CLK AHBCLK Bit Field Definitions */
#define CLK_AHBCLK_HDIV_EN_Pos               4                                    /*!< CLK_T::AHBCLK: HDIV_EN Position */
#define CLK_AHBCLK_HDIV_EN_Msk               (1ul << CLK_AHBCLK_HDIV_EN_Pos)      /*!< CLK_T::AHBCLK: HDIV_EN Mask */

/* CLK APBCLK Bit Field Definitions */

#define CLK_APBCLK_OPA_EN_Pos                29                                   /*!< CLK_T::APBCLK: OPA_EN Position */
#define CLK_APBCLK_OPA_EN_Msk                (1ul << CLK_APBCLK_OPA_EN_Pos)       /*!< CLK_T::APBCLK: OPA_EN Mask */

#define CLK_APBCLK_EADC_EN_Pos               28                                   /*!< CLK_T::APBCLK: EADC_EN Position */
#define CLK_APBCLK_EADC_EN_Msk               (1ul << CLK_APBCLK_EADC_EN_Pos)      /*!< CLK_T::APBCLK: EADC_EN Mask */

#define CLK_APBCLK_ECAP1_EN_Pos              27                                   /*!< CLK_T::APBCLK: ECAP1_EN Position */
#define CLK_APBCLK_ECAP1_EN_Msk              (1ul << CLK_APBCLK_ECAP1_EN_Pos)     /*!< CLK_T::APBCLK: ECAP1_EN Mask */

#define CLK_APBCLK_ECAP0_EN_Pos              26                                   /*!< CLK_T::APBCLK: ECAP0_EN Position */
#define CLK_APBCLK_ECAP0_EN_Msk              (1ul << CLK_APBCLK_ECAP0_EN_Pos)     /*!< CLK_T::APBCLK: ECAP0_EN Mask */

#define CLK_APBCLK_ACMP_EN_Pos               22                                   /*!< CLK_T::APBCLK: ACMP_EN Position */
#define CLK_APBCLK_ACMP_EN_Msk               (1ul << CLK_APBCLK_ACMP_EN_Pos)      /*!< CLK_T::APBCLK: ACMP_EN Mask */

#define CLK_APBCLK_EPWM1_EN_Pos              21                                   /*!< CLK_T::APBCLK: EPWM1_EN Position */
#define CLK_APBCLK_EPWM1_EN_Msk              (1ul << CLK_APBCLK_EPWM1_EN_Pos)     /*!< CLK_T::APBCLK: EPWM1_EN Mask */

#define CLK_APBCLK_EPWM0_EN_Pos              20                                   /*!< CLK_T::APBCLK: EPWM0_EN Position */
#define CLK_APBCLK_EPWM0_EN_Msk              (1ul << CLK_APBCLK_EPWM0_EN_Pos)     /*!< CLK_T::APBCLK: EPWM0_EN Mask */

#define CLK_APBCLK_BPWM0_EN_Pos              19                                   /*!< CLK_T::APBCLK: BPWM0_EN Position */
#define CLK_APBCLK_BPWM0_EN_Msk              (1ul << CLK_APBCLK_BPWM0_EN_Pos)     /*!< CLK_T::APBCLK: BPWM0_EN Mask */

#define CLK_APBCLK_UART1_EN_Pos              17                                   /*!< CLK_T::APBCLK: UART1_EN Position */
#define CLK_APBCLK_UART1_EN_Msk              (1ul << CLK_APBCLK_UART1_EN_Pos)     /*!< CLK_T::APBCLK: UART1_EN Mask */

#define CLK_APBCLK_UART0_EN_Pos              16                                   /*!< CLK_T::APBCLK: UART0_EN Position */
#define CLK_APBCLK_UART0_EN_Msk              (1ul << CLK_APBCLK_UART0_EN_Pos)     /*!< CLK_T::APBCLK: UART0_EN Mask */

#define CLK_APBCLK_SPI2_EN_Pos               14                                   /*!< CLK_T::APBCLK: SPI2_EN Position */
#define CLK_APBCLK_SPI2_EN_Msk               (1ul << CLK_APBCLK_SPI2_EN_Pos)      /*!< CLK_T::APBCLK: SPI2_EN Mask */

#define CLK_APBCLK_SPI1_EN_Pos               13                                   /*!< CLK_T::APBCLK: SPI1_EN Position */
#define CLK_APBCLK_SPI1_EN_Msk               (1ul << CLK_APBCLK_SPI1_EN_Pos)      /*!< CLK_T::APBCLK: SPI1_EN Mask */

#define CLK_APBCLK_SPI0_EN_Pos               12                                   /*!< CLK_T::APBCLK: SPI0_EN Position */
#define CLK_APBCLK_SPI0_EN_Msk               (1ul << CLK_APBCLK_SPI0_EN_Pos)      /*!< CLK_T::APBCLK: SPI0_EN Mask */

#define CLK_APBCLK_I2C0_EN_Pos               8                                    /*!< CLK_T::APBCLK: I2C0_EN_ Position */
#define CLK_APBCLK_I2C0_EN_Msk               (1ul << CLK_APBCLK_I2C0_EN_Pos)      /*!< CLK_T::APBCLK: I2C0_EN_ Mask */

#define CLK_APBCLK_FDIV_EN_Pos               6                                    /*!< CLK_T::APBCLK: FDIV_EN Position */
#define CLK_APBCLK_FDIV_EN_Msk               (1ul << CLK_APBCLK_FDIV_EN_Pos)      /*!< CLK_T::APBCLK: FDIV_EN Mask */

#define CLK_APBCLK_TMR3_EN_Pos               5                                    /*!< CLK_T::APBCLK: TMR3_EN Position */
#define CLK_APBCLK_TMR3_EN_Msk               (1ul << CLK_APBCLK_TMR3_EN_Pos)      /*!< CLK_T::APBCLK: TMR3_EN Mask */

#define CLK_APBCLK_TMR2_EN_Pos               4                                    /*!< CLK_T::APBCLK: TMR2_EN Position */
#define CLK_APBCLK_TMR2_EN_Msk               (1ul << CLK_APBCLK_TMR2_EN_Pos)      /*!< CLK_T::APBCLK: TMR2_EN Mask */

#define CLK_APBCLK_TMR1_EN_Pos               3                                    /*!< CLK_T::APBCLK: TMR1_EN Position */
#define CLK_APBCLK_TMR1_EN_Msk               (1ul << CLK_APBCLK_TMR1_EN_Pos)      /*!< CLK_T::APBCLK: TMR1_EN Mask */

#define CLK_APBCLK_TMR0_EN_Pos               2                                    /*!< CLK_T::APBCLK: TMR0_EN Position */
#define CLK_APBCLK_TMR0_EN_Msk               (1ul << CLK_APBCLK_TMR0_EN_Pos)      /*!< CLK_T::APBCLK: TMR0_EN Mask */

#define CLK_APBCLK_WDT_EN_Pos                0                                    /*!< CLK_T::APBCLK: WDT_EN Position */
#define CLK_APBCLK_WDT_EN_Msk                (1ul << CLK_APBCLK_WDT_EN_Pos)       /*!< CLK_T::APBCLK: WDT_EN Mask */

/* CLK CLKSTATUS Bit Field Definitions */
#define CLK_CLKSTATUS_CLK_SW_FAIL_Pos        7                                        /*!< CLK_T::CLKSTATUS: CLK_SW_FAIL Position */
#define CLK_CLKSTATUS_CLK_SW_FAIL_Msk        (1ul << CLK_CLKSTATUS_CLK_SW_FAIL_Pos)   /*!< CLK_T::CLKSTATUS: CLK_SW_FAIL Mask */

#define CLK_CLKSTATUS_OSC22M_STB_Pos         4                                        /*!< CLK_T::CLKSTATUS: OSC22M_STB Position */
#define CLK_CLKSTATUS_OSC22M_STB_Msk         (1ul << CLK_CLKSTATUS_OSC22M_STB_Pos)    /*!< CLK_T::CLKSTATUS: OSC22M_STB Mask */
#define CLK_CLKSTATUS_IRC22M_STB_Pos         4                                        /*!< CLK_T::CLKSTATUS: IRC22M_STB Position */
#define CLK_CLKSTATUS_IRC22M_STB_Msk         (1ul << CLK_CLKSTATUS_IRC22M_STB_Pos)    /*!< CLK_T::CLKSTATUS: IRC22M_STB Mask */

#define CLK_CLKSTATUS_OSC10K_STB_Pos         3                                        /*!< CLK_T::CLKSTATUS: OSC10K_STB Position */
#define CLK_CLKSTATUS_OSC10K_STB_Msk         (1ul << CLK_CLKSTATUS_OSC10K_STB_Pos)    /*!< CLK_T::CLKSTATUS: OSC10K_STB Mask */
#define CLK_CLKSTATUS_IRC10K_STB_Pos         3                                        /*!< CLK_T::CLKSTATUS: IRC10K_STB Position */
#define CLK_CLKSTATUS_IRC10K_STB_Msk         (1ul << CLK_CLKSTATUS_IRC10K_STB_Pos)    /*!< CLK_T::CLKSTATUS: IRC10K_STB Mask */

#define CLK_CLKSTATUS_PLL_STB_Pos            2                                        /*!< CLK_T::CLKSTATUS: PLL_STB Position */
#define CLK_CLKSTATUS_PLL_STB_Msk            (1ul << CLK_CLKSTATUS_PLL_STB_Pos)       /*!< CLK_T::CLKSTATUS: PLL_STB Mask */

#define CLK_CLKSTATUS_XTL12M_STB_Pos         0                                        /*!< CLK_T::CLKSTATUS: XTL12M_STB Position */
#define CLK_CLKSTATUS_XTL12M_STB_Msk         (1ul << CLK_CLKSTATUS_XTL12M_STB_Pos)    /*!< CLK_T::CLKSTATUS: XTL12M_STB Mask */

/* CLK CLKSEL0 Bit Field Definitions */
#define CLK_CLKSEL0_STCLK_S_Pos              3                                        /*!< CLK_T::CLKSEL0: STCLK_S Position */
#define CLK_CLKSEL0_STCLK_S_Msk              (7ul << CLK_CLKSEL0_STCLK_S_Pos)         /*!< CLK_T::CLKSEL0: STCLK_S Mask */

#define CLK_CLKSEL0_HCLK_S_Pos               0                                        /*!< CLK_T::CLKSEL0: HCLK_S Position */
#define CLK_CLKSEL0_HCLK_S_Msk               (7ul << CLK_CLKSEL0_HCLK_S_Pos)          /*!< CLK_T::CLKSEL0: HCLK_S Mask */

/* CLK CLKSEL1 Bit Field Definitions */
#define CLK_CLKSEL1_UART_S_Pos               24                                       /*!< CLK_T::CLKSEL1: UART_S Position */
#define CLK_CLKSEL1_UART_S_Msk               (3ul << CLK_CLKSEL1_UART_S_Pos)          /*!< CLK_T::CLKSEL1: UART_S Mask */

#define CLK_CLKSEL1_SPI2_S_Pos               6                                        /*!< CLK_T::CLKSEL1: SPI2_S Position */
#define CLK_CLKSEL1_SPI2_S_Msk               (1ul << CLK_CLKSEL1_SPI2_S_Pos)          /*!< CLK_T::CLKSEL1: SPI2_S Mask */

#define CLK_CLKSEL1_SPI1_S_Pos               5                                        /*!< CLK_T::CLKSEL1: SPI1_S Position */
#define CLK_CLKSEL1_SPI1_S_Msk               (1ul << CLK_CLKSEL1_SPI1_S_Pos)          /*!< CLK_T::CLKSEL1: SPI1_S Mask */

#define CLK_CLKSEL1_SPI0_S_Pos               4                                        /*!< CLK_T::CLKSEL1: SPI0_S Position */
#define CLK_CLKSEL1_SPI0_S_Msk               (1ul << CLK_CLKSEL1_SPI0_S_Pos)          /*!< CLK_T::CLKSEL1: SPI0_S Mask */

#define CLK_CLKSEL1_WDT_S_Pos                0                                        /*!< CLK_T::CLKSEL1: WDT_S Position */
#define CLK_CLKSEL1_WDT_S_Msk                (3ul << CLK_CLKSEL1_WDT_S_Pos)           /*!< CLK_T::CLKSEL1: WDT_S Mask */

/* CLK CLKSEL2 Bit Field Definitions */
#define CLK_CLKSEL2_WWDT_S_Pos               16                                       /*!< CLK_T::CLKSEL2: WWDT_S Position */
#define CLK_CLKSEL2_WWDT_S_Msk               (3ul << CLK_CLKSEL2_WWDT_S_Pos)          /*!< CLK_T::CLKSEL2: WWDT_S Mask */

#define CLK_CLKSEL2_FRQDIV_S_Pos             2                                        /*!< CLK_T::CLKSEL2: FRQDIV_S Position */
#define CLK_CLKSEL2_FRQDIV_S_Msk             (3ul << CLK_CLKSEL2_FRQDIV_S_Pos)        /*!< CLK_T::CLKSEL2: FRQDIV_S Mask */

/* CLK CLKDIV Bit Field Definitions */
#define CLK_CLKDIV_EADC_N_Pos                16                                       /*!< CLK_T::CLKDIV: EADC_N Position */
#define CLK_CLKDIV_EADC_N_Msk                (0xFFul << CLK_CLKDIV_EADC_N_Pos)        /*!< CLK_T::CLKDIV: EADC_N Mask */

#define CLK_CLKDIV_UART_N_Pos                8                                        /*!< CLK_T::CLKDIV: UART_N Position */
#define CLK_CLKDIV_UART_N_Msk                (0xFul << CLK_CLKDIV_UART_N_Pos)         /*!< CLK_T::CLKDIV: UART_N Mask */

#define CLK_CLKDIV_HCLK_N_Pos                0                                        /*!< CLK_T::CLKDIV: HCLK_N Position */
#define CLK_CLKDIV_HCLK_N_Msk                (0xFul << CLK_CLKDIV_HCLK_N_Pos)         /*!< CLK_T::CLKDIV: HCLK_N Mask */

/* CLK PLLCON Bit Field Definitions */
#define CLK_PLLCON_FCO_SEL_Pos               20                                       /*!< CLK_T::PLLCON: FCO_SEL Position */
#define CLK_PLLCON_FCO_SEL_Msk               (1ul << CLK_PLLCON_FCO_SEL_Pos)          /*!< CLK_T::PLLCON: FCO_SEL Mask */

#define CLK_PLLCON_PLL_SRC_Pos               19                                       /*!< CLK_T::PLLCON: PLL_SRC Position */
#define CLK_PLLCON_PLL_SRC_Msk               (1ul << CLK_PLLCON_PLL_SRC_Pos)          /*!< CLK_T::PLLCON: PLL_SRC Mask */

#define CLK_PLLCON_OE_Pos                    18                                       /*!< CLK_T::PLLCON: OE Position */
#define CLK_PLLCON_OE_Msk                    (1ul << CLK_PLLCON_OE_Pos)               /*!< CLK_T::PLLCON: OE Mask */

#define CLK_PLLCON_BP_Pos                    17                                       /*!< CLK_T::PLLCON: BP Position */
#define CLK_PLLCON_BP_Msk                    (1ul << CLK_PLLCON_BP_Pos)               /*!< CLK_T::PLLCON: BP Mask */

#define CLK_PLLCON_PD_Pos                    16                                       /*!< CLK_T::PLLCON: PD Position */
#define CLK_PLLCON_PD_Msk                    (1ul << CLK_PLLCON_PD_Pos)               /*!< CLK_T::PLLCON: PD Mask */

#define CLK_PLLCON_OUT_DV_Pos                14                                       /*!< CLK_T::PLLCON: OUT_DV Position */
#define CLK_PLLCON_OUT_DV_Msk                (3ul << CLK_PLLCON_OUT_DV_Pos)           /*!< CLK_T::PLLCON: OUT_DV Mask */

#define CLK_PLLCON_IN_DV_Pos                 9                                        /*!< CLK_T::PLLCON: IN_DV Position */
#define CLK_PLLCON_IN_DV_Msk                 (0x1Ful << CLK_PLLCON_IN_DV_Pos)         /*!< CLK_T::PLLCON: IN_DV Mask */

#define CLK_PLLCON_FB_DV_Pos                 0                                        /*!< CLK_T::PLLCON: FB_DV Position */
#define CLK_PLLCON_FB_DV_Msk                 (0x1FFul << CLK_PLLCON_FB_DV_Pos)        /*!< CLK_T::PLLCON: FB_DV Mask */

/* CLK FRQDIV Bit Field Definitions */
#define CLK_FRQDIV_DIV1_Pos                  5                                        /*!< CLK_T::FRQDIV: DIV1 Position */
#define CLK_FRQDIV_DIV1_Msk                  (1ul << CLK_FRQDIV_DIV1_Pos)             /*!< CLK_T::FRQDIV: DIV1 Mask */

#define CLK_FRQDIV_DIVIDER_EN_Pos            4                                        /*!< CLK_T::FRQDIV: DIVIDER_EN Position */
#define CLK_FRQDIV_DIVIDER_EN_Msk            (1ul << CLK_FRQDIV_DIVIDER_EN_Pos)       /*!< CLK_T::FRQDIV: DIVIDER_EN Mask */

#define CLK_FRQDIV_FSEL_Pos                  0                                        /*!< CLK_T::FRQDIV: FSEL Position */
#define CLK_FRQDIV_FSEL_Msk                  (0xFul << CLK_FRQDIV_FSEL_Pos)           /*!< CLK_T::FRQDIV: FSEL Mask */

/* CLK CLKDCR Bit Field Definitions */
#define CLK_CLKDCR_CKD_IEN_Pos               1                                        /*!< CLK_T::CLKDCR: CKD_IEN Position */
#define CLK_CLKDCR_CKD_IEN_Msk               (1ul << CLK_CLKDCR_CKD_IEN_Pos)          /*!< CLK_T::CLKDCR: CKD_IEN Mask */

#define CLK_CLKDCR_CKD_EN_Pos                0                                        /*!< CLK_T::CLKDCR: CKD_EN Position */
#define CLK_CLKDCR_CKD_EN_Msk                (1ul << CLK_CLKDCR_CKD_EN_Pos)           /*!< CLK_T::CLKDCR: CKD_EN Mask */

/* CLK CLKDSR Bit Field Definitions */
#define CLK_CLKDSR_CKSTOP_Pos                1                                        /*!< CLK_T::CLKDSR: CKSTOP Position */
#define CLK_CLKDSR_CKSTOP_Msk                (1ul << CLK_CLKDSR_CKSTOP_Pos)           /*!< CLK_T::CLKDSR: CKSTOP Mask */

#define CLK_CLKDSR_CKSTOP_IF_Pos             0                                        /*!< CLK_T::CLKDSR: CKSTOP_IF Position */
#define CLK_CLKDSR_CKSTOP_IF_Msk             (1ul << CLK_CLKDSR_CKSTOP_IF_Pos)        /*!< CLK_T::CLKDSR: CKSTOP_IF Mask */

/*@}*/ /* end of group CLK_CONST */
/*@}*/ /* end of group CLK */


/*----------------------------- EADC Controller -------------------------------*/
/** @addtogroup EADC Enhanced Analog to Digital Converter (EADC)
  Memory Mapped Structure for EADC Controller
  @{
 */

typedef struct
{


    /**
     * @var EADC_T::ADDRA[8]
     * Offset: 0x00~0x1C  A/D Data Register n for SAMPLEAn, n=0~7
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |RSLT      |A/D Conversion Result
     * |        |          |This field contains 12-bit conversion result.
     * |[16]    |OVERRUN   |Overrun Flag
     * |        |          |0 = Data in RSLT[11:0] is the recent conversion result.
     * |        |          |1 = Data in RSLT[11:0] is overwritten.
     * |        |          |If converted data in RSLT[11:0] has not been read before new conversion result is loaded to
     * |        |          |this register, OVERRUN is set to 1. It is cleared by hardware after ADDR register is read.
     * |[17]    |VALID     |Valid Flag
     * |        |          |0 = Data in RSLT[11:0] bits is not valid.
     * |        |          |1 = Data in RSLT[11:0] bits is valid.
     * |        |          |This bit is set to 1 when corresponding SAMPLE channel analog input conversion is completed
     * |        |          |and cleared by hardware after ADDR register is read.
     * @var EADC_T::ADDRB[8]
     * Offset: 0x20~0x3C  A/D Data Register m for SAMPLEAn, m=8~15, n=0~7
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |RSLT      |A/D Conversion Result
     * |        |          |This field contains 12-bit conversion result.
     * |[16]    |OVERRUN   |Overrun Flag
     * |        |          |0 = Data in RSLT[11:0] is the recent conversion result.
     * |        |          |1 = Data in RSLT[11:0] is overwritten.
     * |        |          |If converted data in RSLT[11:0] has not been read before new conversion result is loaded to
     * |        |          |this register, OVERRUN is set to 1. It is cleared by hardware after ADDR register is read.
     * |[17]    |VALID     |Valid Flag
     * |        |          |0 = Data in RSLT[11:0] bits is not valid.
     * |        |          |1 = Data in RSLT[11:0] bits is valid.
     * |        |          |This bit is set to 1 when corresponding SAMPLE channel analog input conversion is completed
     * |        |          |and cleared by hardware after ADDR register is read.
     * @var EADC_T::ADCR
     * Offset: 0x40  EADC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |AD_EN     |A/D Converter Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Before starting A/D conversion function, this bit should be set to 1.
     * |        |          |Clear it to 0 to disable A/D converter analog circuit for saving power consumption.
     * |[1]     |ADRESET   |EADCA, EADCB A/D Converter Control Circuits Reset
     * |        |          |0 = Writing 0 has no effect.
     * |        |          |1 = Writing 1 will cause EADC control circuits reset to initial state, but not change the EADC registers value.
     * |        |          |ADRESET bit remains 1 during EADC reset, when EADC reset end, the ADRESET bit is automatically cleared to 0.
     * |[2]     |ADIE0     |Specific SAMPLE A/D ADINT0 Interrupt Enable
     * |        |          |0 = Specific SAMPLE A/D ADINT0 interrupt function Disabled.
     * |        |          |1 = Specific SAMPLE A/D ADINT0 interrupt function Enabled.
     * |        |          |The A/D converter generates a conversion end ADF0 flag in ADSR1 register upon the end of
     * |        |          |specific SAMPLE A/D conversion. If ADIE0 bit is set then conversion end interrupt request
     * |        |          |ADINT0 is generated.
     * |[3]     |ADIE1     |Specific SAMPLE A/D ADINT1 Interrupt Enable
     * |        |          |0 = Specific SAMPLE A/D ADINT1 interrupt function Disabled.
     * |        |          |1 = Specific SAMPLE A/D ADINT1 interrupt function Enabled.
     * |        |          |The A/D converter generates a conversion end ADF1 flag in ADSR1 register upon the end of
     * |        |          |specific SAMPLE A/D conversion. If ADIE1 bit is set then conversion end interrupt request
     * |        |          |ADINT1 is generated.
     * |[4]     |ADIE2     |Specific SAMPLE A/D ADINT2 Interrupt Enable
     * |        |          |0 = Specific SAMPLE A/D ADINT2 interrupt function Disabled.
     * |        |          |1 = Specific SAMPLE A/D ADINT2 interrupt function Enabled.
     * |        |          |The A/D converter generates a conversion end ADF2 flag in ADSR1 register upon the end of
     * |        |          |specific SAMPLE A/D conversion. If ADIE2 bit is set then conversion end interrupt request
     * |        |          |ADINT2 is generated.
     * |[5]     |ADIE3     |Specific SAMPLE A/D ADINT3 Interrupt Enable
     * |        |          |0 = Specific SAMPLE A/D ADINT3 interrupt function Disabled.
     * |        |          |1 = Specific SAMPLE A/D ADINT3 interrupt function Enabled.
     * |        |          |The A/D converter generates a conversion end ADF3 flag in ADSR1 register upon the end of
     * |        |          |specific SAMPLE A/D conversion. If ADIE3 bit is set then conversion end interrupt request
     * |        |          |ADINT3 is generated.
     * @var EADC_T::ADCHISELR
     * Offset: 0x44  A/D Channel Input Sources Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |AINA0SEL  |A/D Channel AINA[0] Analog Input Selection
     * |        |          |0 = AINA[0] pin P6.0/AINA0 is selected as the EADC AINA[0] input signal.
     * |        |          |1 = OP Amplifier 0 output  is selected as the EADC AINA[0] input signal.
     * |[1]     |AINB0SEL  |A/D Channel AINB[0] Analog Input Selection
     * |        |          |0 = AINB[0] pin P7.0/AINB0 is selected as the A/D AINB[0] input signal.
     * |        |          |1 = OP Amplifier 1 output  is selected as the A/D AINB[0] input signal.
     * |[3:2]   |PRESEL    |A/D Channel AINA[7] Analog Input Selection
     * |        |          |00 = Analog Input Channel AINA7.
     * |        |          |01 = Band-gap (VBG) Analog Input.
     * |        |          |10 = VTEMP Internal Temperature Sensor Analog Input.
     * |        |          |11 = Analog ground.
     * @var EADC_T::ADSSTR
     * Offset: 0x48  A/D SAMPLE Software Start Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ADST[7:0] |A/D SAMPLEA7~0 Software Force To Start EADC Conversion Register
     * |        |          |0 = No effect.
     * |        |          |1 = Cause an EADC conversion when the priority is given to SAMPLEA.
     * |[15:8]  |ADST[15:8]|A/D SAMPLEB7~0 Software Force To Start EADC Conversion Register
     * |        |          |0 = No effect.
     * |        |          |1 = Cause an EADC conversion when the priority is given to SAMPLEB.
     * @var EADC_T::ADSTPFR
     * Offset: 0x4C  A/D SAMPLE Start of Conversion Pending Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |STPF[7:0] |A/D SAMPLEA7~0 Start Conversion Pending Flag
     * |        |          |0 = 0 = There is no pending conversion for SAMPLEA.
     * |        |          |1 = SAMPLEA EADC start of conversion is pending.
     * |        |          |This bit remains 1 during pending state, when the respective EADC conversion is started, the
     * |        |          |STPFx bit is automatically cleared to 0.
     * |[15:8]  |STPF[15:8]|A/D SAMPLEB7~0 Software Force To Start EADC Conversion Register
     * |        |          |0 = No pending conversion for SAMPLEB.
     * |        |          |1 = SAMPLEB EADC start of conversion is pending.
     * |        |          |This bit remains 1 during pending state, when the respective EADC conversion is started, the
     * |        |          |STPFx bit is automatically cleared to 0.
     * @var EADC_T::ADIFOVR
     * Offset: 0x50  A/D ADINT3~0 Interrupt Flag Over Run Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADFOV0    |A/D ADINT0 Interrupt Flag Over Run Bit
     * |        |          |0 = ADINT0 interrupt flag is not over run.
     * |        |          |1 = ADINT0 interrupt flag is overwrite to 1.
     * |        |          |It is cleared by write 1.
     * |[1]     |ADFOV1    |A/D ADINT1 Interrupt Flag Over Run Bit
     * |        |          |0 = ADINT1 interrupt flag is not over run.
     * |        |          |1 = ADINT1 interrupt flag is overwrite to 1.
     * |        |          |It is cleared by write 1.
     * |[2]     |ADFOV2    |A/D ADINT2 Interrupt Flag Over Run Bit
     * |        |          |0 = ADINT2 interrupt flag is not over run.
     * |        |          |1 = ADINT2 interrupt flag is overwrite to 1.
     * |        |          |It is cleared by write 1.
     * |[3]     |ADFOV3    |A/D ADINT3 Interrupt Flag Over Run Bit
     * |        |          |0 = ADINT3 interrupt flag is not over run.
     * |        |          |1 = ADINT3 interrupt flag is overwrite to 1.
     * |        |          |It is cleared by write 1.
     * @var EADC_T::ADSPOVFR
     * Offset: 0x54  A/D SAMPLE Start of Conversion Over Run Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field      |Descriptions
     * | :----: | :----:    | :---- |
     * |[7:0]   |SPOVF[7:0] |A/D SAMPLEA7~0 Start Conversion Over Run Flag
     * |        |           |0 = No SAMPLE event over run.
     * |        |           |1 = A new SAMPLEA event is generated while an old one event is pending.
     * |        |           |It is cleared by writing 1.
     * |[15:8]  |SPOVF[15:8]|A/D SAMPLEB7~0 Start Conversion Over Run Flag
     * |        |           |0 = No SAMPLE event over run.
     * |        |           |1 = A new SAMPLEB event is generated while an old one event is pending.
     * |        |           |It is cleared by writing 1.
     * @var EADC_T::ADSPCRA[8]
     * Offset: 0x58~0x64 ADSPCRA0~3 A/D SAMPLEAn Control Register, n=0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CHSEL     |A/D SAMPLEA Channel Selection
     * |        |          |00H = EADC0_CH0.
     * |        |          |01H = EADC0_CH1.
     * |        |          |02H = EADC0_CH2.
     * |        |          |03H = EADC0_CH3.
     * |        |          |04H = EADC0_CH4.
     * |        |          |05H = EADC0_CH5.
     * |        |          |06H = EADC0_CH6.
     * |        |          |07H = EADC0_CH7.
     * |[7:4]   |TRGSEL    |A/D SAMPLE Start Conversion Trigger Source Selection
     * |        |          |0H = Disable hardware trigger.
     * |        |          |1H = External trigger from STADC pin input.
     * |        |          |2H = EADC ADINT0 interrupt EOC pulse  trigger.
     * |        |          |3H = EADC ADINT1 interrupt EOC pulse  trigger.
     * |        |          |4H = Timer0 overflow pulse trigger.
     * |        |          |5H = Timer1 overflow pulse trigger.
     * |        |          |6H = Timer2 overflow pulse trigger.
     * |        |          |7H = Timer3 overflow pulse trigger.
     * |        |          |8H = EPWM00 trigger.
     * |        |          |9H = EPWM02 trigger.
     * |        |          |AH = EPWM04 trigger.
     * |        |          |BH = EPWM10 trigger.
     * |        |          |CH = EPWM12 trigger.
     * |        |          |DH = EPWM14 trigger.
     * |        |          |EH = BPWM00 trigger.
     * |        |          |FH = BPWM01 trigger.
     * |[15:8]  |TRGDLYCNT |A/D SAMPLE Start Conversion Trigger Delay Time
     * |        |          |Trigger delay time = (TRGDLYCNT + 4) x Trigger delay clock period.
     * |[17:16] |TRGDLYDIV |A/D SAMPLE Start Conversion Trigger Delay Clock Divider Selection
     * |        |          |Trigger delay clock frequency:
     * |        |          |00 = EADC_CLK/1.
     * |        |          |01 = EADC_CLK/2.
     * |        |          |10 = EADC_CLK/4.
     * |        |          |11 = EADC_CLK/16.
     * |[20]    |EXTREN    |A/D External Trigger Rising Edge Enabled
     * |        |          |0 = Rising edge Disabled when A/D selects STADC as trigger source.
     * |        |          |1 = Rising edge Enabled when A/D selects STADC as trigger source.
     * |[21]    |EXTFEN    |A/D External Trigger Falling Edge Enabled
     * |        |          |0 = Falling edge Disabled when A/D selects STADC as trigger source.
     * |        |          |1 = Falling edge Enabled when A/D selects STADC as trigger source.
     * Offset: 0x68~0x74 ADSPCRA4~7 A/D SAMPLEAn Control Register, n=4~7
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CHSEL     |A/D SAMPLEA Channel Selection
     * |        |          |00H = EADC0_CH0.
     * |        |          |01H = EADC0_CH1.
     * |        |          |02H = EADC0_CH2.
     * |        |          |03H = EADC0_CH3.
     * |        |          |04H = EADC0_CH4.
     * |        |          |05H = EADC0_CH5.
     * |        |          |06H = EADC0_CH6.
     * |        |          |07H = EADC0_CH7.
     * |[6:4]   |TRGSEL    |A/D SAMPLE Start Conversion Trigger Source Selection
     * |        |          |0H = Disable hardware trigger.
     * |        |          |1H = External trigger from STADC pin input.
     * |        |          |2H = EADC ADINT0 interrupt EOC pulse  trigger.
     * |        |          |3H = EADC ADINT1 interrupt EOC pulse  trigger.
     * |        |          |4H = Timer0 overflow pulse trigger.
     * |        |          |5H = Timer1 overflow pulse trigger.
     * |        |          |6H = Timer2 overflow pulse trigger.
     * |        |          |7H = Timer3 overflow pulse trigger.
     * |[20]    |EXTREN    |A/D External Trigger Rising Edge Enabled
     * |        |          |0 = Rising edge Disabled when A/D selects STADC as trigger source.
     * |        |          |1 = Rising edge Enabled when A/D selects STADC as trigger source.
     * |[21]    |EXTFEN    |A/D External Trigger Falling Edge Enabled
     * |        |          |0 = Falling edge Disabled when A/D selects STADC as trigger source.
     * |        |          |1 = Falling edge Enabled when A/D selects STADC as trigger source.
     * @var EADC_T::ADSPCRB[8]
     * Offset: 0x78~0x84 ADSPCRB0~3 A/D SAMPLEBn Control Register, n=0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CHSEL     |A/D SAMPLEB Channel Selection
     * |        |          |00H = EADC1_CH0.
     * |        |          |01H = EADC1_CH1.
     * |        |          |02H = EADC1_CH2.
     * |        |          |03H = EADC1_CH3.
     * |        |          |04H = EADC1_CH4.
     * |        |          |05H = EADC1_CH5.
     * |        |          |06H = EADC1_CH6.
     * |        |          |07H = EADC1_CH7.
     * |[7:4]   |TRGSEL    |A/D SAMPLE Start Conversion Trigger Source Selection
     * |        |          |0H = Disable hardware trigger.
     * |        |          |1H = External trigger from STADC pin input.
     * |        |          |2H = EADC ADINT0 interrupt EOC pulse  trigger.
     * |        |          |3H = EADC ADINT1 interrupt EOC pulse  trigger.
     * |        |          |4H = Timer0 overflow pulse trigger.
     * |        |          |5H = Timer1 overflow pulse trigger.
     * |        |          |6H = Timer2 overflow pulse trigger.
     * |        |          |7H = Timer3 overflow pulse trigger.
     * |        |          |8H = EPWM00 trigger.
     * |        |          |9H = EPWM02 trigger.
     * |        |          |AH = EPWM04 trigger.
     * |        |          |BH = EPWM10 trigger.
     * |        |          |CH = EPWM12 trigger.
     * |        |          |DH = EPWM14 trigger.
     * |        |          |EH = BPWM00 trigger.
     * |        |          |FH = BPWM01 trigger.
     * |[15:8]  |TRGDLYCNT |A/D SAMPLE Start Conversion Trigger Delay Time
     * |        |          |Trigger delay time = (TRGDLYCNT + 4) x Trigger delay clock period.
     * |[17:16] |TRGDLYDIV |A/D SAMPLE Start Conversion Trigger Delay Clock Divider Selection
     * |        |          |Trigger delay clock frequency:
     * |        |          |00 = EADC_CLK/1.
     * |        |          |01 = EADC_CLK/2.
     * |        |          |10 = EADC_CLK/4.
     * |        |          |11 = EADC_CLK/16.
     * |[20]    |EXTREN    |A/D External Trigger Rising Edge Enabled
     * |        |          |0 = Rising edge Disabled when A/D selects STADC as trigger source.
     * |        |          |1 = Rising edge Enabled when A/D selects STADC as trigger source.
     * |[21]    |EXTFEN    |A/D External Trigger Falling Edge Enabled
     * |        |          |0 = Falling edge Disabled when A/D selects STADC as trigger source.
     * |        |          |1 = Falling edge Enabled when A/D selects STADC as trigger source.
     * Offset: 0x88~0x94 ADSPCRB4~7 A/D SAMPLEBn Control Register, n=4~7
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CHSEL     |A/D SAMPLEB Channel Selection
     * |        |          |00H = EADC1_CH0.
     * |        |          |01H = EADC1_CH1.
     * |        |          |02H = EADC1_CH2.
     * |        |          |03H = EADC1_CH3.
     * |        |          |04H = EADC1_CH4.
     * |        |          |05H = EADC1_CH5.
     * |        |          |06H = EADC1_CH6.
     * |        |          |07H = EADC1_CH7.
     * |[6:4]   |TRGSEL    |A/D SAMPLE Start Conversion Trigger Source Selection
     * |        |          |0H = Disable hardware trigger.
     * |        |          |1H = External trigger from STADC pin input.
     * |        |          |2H = EADC ADINT0 interrupt EOC pulse  trigger.
     * |        |          |3H = EADC ADINT1 interrupt EOC pulse  trigger.
     * |        |          |4H = Timer0 overflow pulse trigger.
     * |        |          |5H = Timer1 overflow pulse trigger.
     * |        |          |6H = Timer2 overflow pulse trigger.
     * |        |          |7H = Timer3 overflow pulse trigger.
     * |[20]    |EXTREN    |A/D External Trigger Rising Edge Enabled
     * |        |          |0 = Rising edge Disabled when A/D selects STADC as trigger source.
     * |        |          |1 = Rising edge Enabled when A/D selects STADC as trigger source.
     * |[21]    |EXTFEN    |A/D External Trigger Falling Edge Enabled
     * |        |          |0 = Falling edge Disabled when A/D selects STADC as trigger source.
     * |        |          |1 = Falling edge Enabled when A/D selects STADC as trigger source.
     * @var EADC_T::ADSMSELR
     * Offset: 0xA4  A/D SAMPLE Simultaneous Mode Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SIMUSEL0  |A/D SAMPLEA0, SAMPLEB0 Simultaneous Sampling Mode Selection
     * |        |          |0 = SAMPLEA0, SAMPLEB0 are in single sampling mode, both SAMPLEA0 and
     * |        |          |SAMPLEB0's 3 bits of CHSEL define the ADC channels to be converted.
     * |        |          |1 = SAMPLEA0, SAMPLEB0 are in simultaneous sampling mode, Only SAMPLEA0 can
     * |        |          |trigger both the EADC conversions of SAMPLEA0 and SAMPLEB0, SAMPLEB0.trigger
     * |        |          |select TRGSEL is ignored. if SAMPLEA0's CHSEL = 1, and SAMPLEB0's CHSEL = 3, the
     * |        |          |pair of channels are AINA[1], AINB[3], they will do the EADC conversion at the same time.
     * |[1]     |SIMUSEL1  |A/D SAMPLEA1, SAMPLEB1 Simultaneous Sampling Mode Selection
     * |        |          |0 = SAMPLEA1, SAMPLEB1 are in single sampling mode, both SAMPLEA1 and
     * |        |          |SAMPLEB1's 3 bits of CHSEL define the ADC channels to be converted.
     * |        |          |1 = SAMPLEA1, SAMPLEB1 are in simultaneous sampling mode, Only SAMPLEA1 can
     * |        |          |trigger both the EADC conversions of SAMPLEA1 and SAMPLEB1, SAMPLEB1.trigger
     * |        |          |select TRGSEL is ignored. if SAMPLEA1's CHSEL = 1, and SAMPLEB1's CHSEL = 3, the
     * |        |          |pair of channels are AINA[1], AINB[3], they will do the EADC conversion at the same time.
     * |[2]     |SIMUSEL2  |A/D SAMPLEA2, SAMPLEB2 Simultaneous Sampling Mode Selection
     * |        |          |0 = SAMPLEA2, SAMPLEB2 are in single sampling mode, both SAMPLEA2 and
     * |        |          |SAMPLEB2's 3 bits of CHSEL define the ADC channels to be converted.
     * |        |          |1 = SAMPLEA2, SAMPLEB2 are in simultaneous sampling mode, Only SAMPLEA2 can
     * |        |          |trigger both the EADC conversions of SAMPLEA2 and SAMPLEB2, SAMPLEB2.trigger
     * |        |          |select TRGSEL is ignored. if SAMPLEA2's CHSEL = 1, and SAMPLEB2's CHSEL = 3, the
     * |        |          |pair of channels are AINA[1], AINB[3], they will do the EADC conversion at the same time.
     * |[3]     |SIMUSEL3  |A/D SAMPLEA3, SAMPLEB3 Simultaneous Sampling Mode Selection
     * |        |          |0 = SAMPLEA3, SAMPLEB3 are in single sampling mode, both SAMPLEA3 and
     * |        |          |SAMPLEB3's 3 bits of CHSEL define the ADC channels to be converted.
     * |        |          |1 = SAMPLEA3, SAMPLEB3 are in simultaneous sampling mode, Only SAMPLEA3 can
     * |        |          |trigger both the EADC conversions of SAMPLEA3 and SAMPLEB3, SAMPLEB3.trigger
     * |        |          |select TRGSEL is ignored. if SAMPLEA3's CHSEL = 1, and SAMPLEB3's CHSEL = 3, the
     * |        |          |pair of channels are AINA[1], AINB[3], they will do the EADC conversion at the same time.
     * |[4]     |SIMUSEL4  |A/D SAMPLEA4, SAMPLEB4 Simultaneous Sampling Mode Selection
     * |        |          |0 = SAMPLEA4, SAMPLEB4 are in single sampling mode, both SAMPLEA4 and
     * |        |          |SAMPLEB4's 3 bits of CHSEL define the ADC channels to be converted.
     * |        |          |1 = SAMPLEA4, SAMPLEB4 are in simultaneous sampling mode, Only SAMPLEA4 can
     * |        |          |trigger both the EADC conversions of SAMPLEA4 and SAMPLEB4, SAMPLEB4.trigger
     * |        |          |select TRGSEL is ignored. if SAMPLEA4's CHSEL = 1, and SAMPLEB4's CHSEL = 3, the
     * |        |          |pair of channels are AINA[1], AINB[3], they will do the EADC conversion at the same time.
     * |[5]     |SIMUSEL5  |A/D SAMPLEA5, SAMPLEB5 Simultaneous Sampling Mode Selection
     * |        |          |0 = SAMPLEA5, SAMPLEB5 are in single sampling mode, both SAMPLEA5 and
     * |        |          |SAMPLEB5's 3 bits of CHSEL define the ADC channels to be converted.
     * |        |          |1 = SAMPLEA5, SAMPLEB5 are in simultaneous sampling mode, Only SAMPLEA5 can
     * |        |          |trigger both the EADC conversions of SAMPLEA5 and SAMPLEB5, SAMPLEB5.trigger
     * |        |          |select TRGSEL is ignored. if SAMPLEA5's CHSEL = 1, and SAMPLEB5's CHSEL = 3, the
     * |        |          |pair of channels are AINA[1], AINB[3], they will do the EADC conversion at the same time.
     * |[6]     |SIMUSEL6  |A/D SAMPLEA6, SAMPLEB6 Simultaneous Sampling Mode Selection
     * |        |          |0 = SAMPLEA6, SAMPLEB6 are in single sampling mode, both SAMPLEA6 and
     * |        |          |SAMPLEB6's 3 bits of CHSEL define the ADC channels to be converted.
     * |        |          |1 = SAMPLEA6, SAMPLEB6 are in simultaneous sampling mode, Only SAMPLEA6 can
     * |        |          |trigger both the EADC conversions of SAMPLEA6 and SAMPLEB6, SAMPLEB6.trigger
     * |        |          |select TRGSEL is ignored. if SAMPLEA6's CHSEL = 1, and SAMPLEB6's CHSEL = 3, the
     * |        |          |pair of channels are AINA[1], AINB[3], they will do the EADC conversion at the same time.
     * |[7]     |SIMUSEL7  |A/D SAMPLEA7, SAMPLEB7 Simultaneous Sampling Mode Selection
     * |        |          |0 = SAMPLEA7, SAMPLEB7 are in single sampling mode, both SAMPLEA7 and
     * |        |          |SAMPLEB7's 3 bits of CHSEL define the ADC channels to be converted.
     * |        |          |1 = SAMPLEA7, SAMPLEB7 are in simultaneous sampling mode, Only SAMPLEA7 can
     * |        |          |trigger both the EADC conversions of SAMPLEA7 and SAMPLEB7, SAMPLEB7.trigger
     * |        |          |select TRGSEL is ignored. if SAMPLEA7's CHSEL = 1, SAMPLEB7's CHSEL = 3, the pair
     * |        |          |of channels are AINA[1], AINB[3], they will do the EADC conversion at the same time.
     * @var EADC_T::ADCMPR[2]
     * Offset: 0xA8~0xAC  A/D Result Compare Register 0/1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCMP_EN  |A/D Result Compare Enable
     * |        |          |0 = Compare Disabled.
     * |        |          |1 = Compare Enabled.
     * |        |          |Set this bit to 1 to enable compare CMPD[11:0] with specified SAMPLE conversion result
     * |        |          |when converted data is loaded into ADDR register.
     * |[1]     |ADCMPIE   |A/D Result Compare Interrupt Enable
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |        |          |If the compare function is enabled and the compare condition matches the setting of
     * |        |          |CMPCOND and CMPMATCNT, ADCMPF bit will be asserted, in the meanwhile, if
     * |        |          |ADCMPIE is set to 1, a compare interrupt request is generated.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0= Set the compare condition as that when a 12-bit A/D conversion result is less than the
     * |        |          |12-bit CMPD (ADCMPRx[27:16]), the internal match counter will increase one.
     * |        |          |1= Set the compare condition as that when a 12-bit A/D conversion result is greater or
     * |        |          |equal to the 12-bit CMPD (ADCMPRx[27:16]), the internal match counter will increase
     * |        |          |one.
     * |        |          |Note: When the internal counter reaches the value to (CMPMATCNT +1), the CMPF bit
     * |        |          |will be set.
     * |[5:3]   |CMPSMPL   |Compare SAMPLE Selection
     * |        |          |000 = SAMPLEA0 conversion result ADDRA0 is selected to be compared.
     * |        |          |001 = SAMPLEA1 conversion result ADDRA1 is selected to be compared.
     * |        |          |010 = SAMPLEA2 conversion result ADDRA2 is selected to be compared.
     * |        |          |011 = SAMPLEA3 conversion result ADDRA3 is selected to be compared.
     * |        |          |100 = SAMPLEB0 conversion result ADDRB0 is selected to be compared.
     * |        |          |101 = SAMPLEB1 conversion result ADDRB1 is selected to be compared.
     * |        |          |110 = SAMPLEB2 conversion result ADDRB2 is selected to be compared.
     * |        |          |111 = SAMPLEB3 conversion result ADDRB3 is selected to be compared.
     * |[11:8]  |CMPMATCNT |Compare Match Count
     * |        |          |When the specified A/D SAMPLE analog conversion result matches the compare
     * |        |          |condition defined by CMPCOND, the internal match counter will increase 1. When the
     * |        |          |internal counter reaches the value to (CMPMATCNT +1), the CMPF bit will be set.
     * |[27:16] |CMPD      |Comparison Data
     * |        |          |The 12 bits data is used to compare with the conversion result of specified SAMPLE.
     * |        |          |Software can use it to monitor the external analog input pin voltage transition without
     * |        |          |imposing a load on software.
     * @var EADC_T::ADSR0
     * Offset: 0xB0  A/D Status Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |VALID     |ADDRA7~0 Data Valid Flag
     * |        |          |It is a mirror of VALID bit in SAMPLEA A/D result data register ADDRAn, n=0~7.
     * |[15:8]  |VALID     |ADDRB7~0 Data Valid Flag
     * |        |          |It is a mirror of VALID bit in SAMPLEB A/D result data register ADDRBn, n=0~7.
     * |[23:16] |OV        |ADDRA7~0 Over Run Flag
     * |        |          |It is a mirror to OVERRUN bit in SAMPLEA A/D result data register ADDRAn, n=0~7.
     * |[31:24] |OV        |ADDRB7~0 Over Run Flag
     * |        |          |It is a mirror to OVERRUN bit in SAMPLEB A/D result data register ADDRBn, n=0~7.
     * @var EADC_T::ADSR1
     * Offset: 0xB4  A/D Status Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADF0      |A/D ADINT0 Interrupt Flag
     * |        |          |0 = No ADINT0 interrupt pulse received.
     * |        |          |1 = ADINT0 interrupt pulse received.
     * |        |          |It is cleared by writing 1.
     * |        |          |This bit indicates whether an A/D conversion of specific SAMPLE has been completed.
     * |[1]     |ADF1      |A/D ADINT1 Interrupt Flag
     * |        |          |0 = No ADINT1 interrupt pulse received.
     * |        |          |1 = ADINT1 interrupt pulse received.
     * |        |          |It is cleared by writing 1.
     * |        |          |This bit indicates whether an A/D conversion of specific SAMPLE has been completed.
     * |[2]     |ADF2      |A/D ADINT2 Interrupt Flag
     * |        |          |0 = No ADINT2 interrupt pulse received.
     * |        |          |1 = ADINT2 interrupt pulse received.
     * |        |          |It is cleared by writing 1.
     * |        |          |This bit indicates whether an A/D conversion of specific SAMPLE has been completed.
     * |[3]     |ADF3      |A/D ADINT3 Interrupt Flag
     * |        |          |0 = No ADINT3 interrupt pulse received.
     * |        |          |1 = ADINT3 interrupt pulse received.
     * |        |          |It is cleared by writing 1.
     * |        |          |This bit indicates whether an A/D conversion of specific SAMPLE has been completed.
     * |[4]     |ADCMPO0   |EADC Compare 0 Output Status Bit
     * |        |          |The 12 bits compare0 data (ADCMPR0[27:16])  is used to compare with conversion
     * |        |          |result of specified SAMPLE. Software can use it to monitor the external analog input pin
     * |        |          |voltage status.
     * |        |          |0 = Conversion result in ADDR is less than ADCMPR0[27:16] setting.
     * |        |          |1 = Conversion result in ADDR is great than or equal ADCMPR0[27:16] setting.
     * |[5]     |ADCMPO1   |EADC Compare 1 Output Status Bit
     * |        |          |The 12 bits compare1 data (ADCMPR1[27:16])  is used to compare with conversion
     * |        |          |result of specified SAMPLE. Software can use it to monitor the external analog input pin
     * |        |          |voltage status.
     * |        |          |0 = Conversion result in ADDR is less than ADCMPR1[27:16] setting.
     * |        |          |1 = Conversion result in ADDR is great than or equal ADCMPR1[27:16] setting.
     * |[6]     |ADCMPF0   |EADC Compare 0 Flag
     * |        |          |When the specific SAMPLE A/D conversion result meets setting condition in ADCMPR0
     * |        |          |then this bit is set to 1. And it is cleared by write 1.
     * |        |          |0 = Conversion result in ADDR does not meet ADCMPR0 setting.
     * |        |          |1 = Conversion result in ADDR meets ADCMPR0 setting.
     * |[7]     |ADCMPF1   |EADC Compare 1 Flag
     * |        |          |When the specific SAMPLE A/D conversion result meets setting condition in ADCMPR1
     * |        |          |then this bit is set to 1. And it is cleared by write 1.
     * |        |          |0 = Conversion result in ADDR does not meet ADCMPR1 setting.
     * |        |          |1 = Conversion result in ADDR meets ADCMPR1 setting.
     * |[8]     |BUSYA     |BUSY/IDLE
     * |        |          |0 = A/D converter A (EADCA) is in idle state.
     * |        |          |1 = A/D converter A (EADCA) is busy at conversion.
     * |        |          |It is read only.
     * |[14:12] |CHANNELA  |Current Conversion Channel
     * |        |          |This filed reflects EADCA current conversion channel when BUSYA=1. When BUSYA=0, it
     * |        |          |shows the last converted channel.
     * |        |          |It is read only.
     * |        |          |00H = EADC0_CH0.
     * |        |          |01H = EADC0_CH1.
     * |        |          |02H = EADC0_CH2.
     * |        |          |03H = EADC0_CH3.
     * |        |          |04H = EADC0_CH4.
     * |        |          |05H = EADC0_CH5.
     * |        |          |06H = EADC0_CH6.
     * |        |          |07H = EADC0_CH7.
     * |[16]    |BUSYB     |BUSY/IDLE
     * |        |          |0 = A/D converter B (EADCB) is in idle state.
     * |        |          |1 = A/D converter B (EADCB) is busy at conversion.
     * |        |          |It is read only.
     * |[22:20] |CHANNELB  |Current Conversion Channel
     * |        |          |This filed reflects EADCB current conversion channel when BUSYB=1. When BUSYB=0, it
     * |        |          |shows the last converted channel.
     * |        |          |It is read only.
     * |        |          |00H = EADC1_CH0.
     * |        |          |01H = EADC1_CH1.
     * |        |          |02H = EADC1_CH2.
     * |        |          |03H = EADC1_CH3.
     * |        |          |04H = EADC1_CH4.
     * |        |          |05H = EADC1_CH5.
     * |        |          |06H = EADC1_CH6.
     * |        |          |07H = EADC1_CH7.
     * |[24]    |AADFOV    |All A/D Interrupt Flag Over Run Bits Check
     * |        |          |0 = None of ADINT interrupt flag ADFOVx is overwritten to 1.
     * |        |          |1 = Any one of ADINT interrupt flag ADFOVx is overwritten to 1.
     * |        |          |This bit will keep 1 when any ADFOVx Flag is equal to 1.
     * |[25]    |ASPOVF    |All A/D SAMPLE Start Conversion Over Run Flags Check
     * |        |          |0 = None of SAMPLE event over run flag SPOVFx is set to 1.
     * |        |          |1 = Any one of SAMPLE event over run flag SPOVFx is set to 1.
     * |        |          |This bit will keep 1 when any SPOVFx Flag is equal to 1.
     * |[26]    |AVALID    |All SAMPLE A/D Result Data Register ADDR Data Valid Flag Check
     * |        |          |0 = None of SAMPLE data register valid flag VALIDx is set to 1.
     * |        |          |1 = Any one of SAMPLE data register valid flag VALIDx is set to 1.
     * |        |          |This bit will keep 1 when any VALIDx Flag is equal to 1.
     * |[27]    |AOVERRUN  |All SAMPLE A/D Result Data Register Over Run Flags Check
     * |        |          |0 = None of SAMPLE data register over run flag OVERRUNx is set to 1.
     * |        |          |1 = Any one of SAMPLE data register over run flag OVERRUNx is set to 1.
     * |        |          |This bit will keep 1 when any OVERRUNx Flag is equal to 1.
     * @var EADC_T::ADTCR
     * Offset: 0xB8  A/D Timing Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ADAEST    |EADCA Extend Sampling Time
     * |        |          |When A/D converting at high conversion rate, the sampling time of analog input voltage
     * |        |          |may not enough if input channel loading is heavy, SW can extend  A/D sampling time
     * |        |          |after trigger source is coming to get enough sampling time.
     * |        |          |The range of start delay time is from 0~255 EADC clock.
     * |[23:16] |ADBEST    |EADCB Extend Sampling Time
     * |        |          |When A/D converting at high conversion rate, the sampling time of analog input voltage
     * |        |          |may not enough if input channel loading is heavy, SW can extend  A/D sampling time
     * |        |          |after trigger source is coming to get enough sampling time.
     * |        |          |The range of start delay time is from 0~255 EADC clock.
     * @var EADC_T::ADDRDBA[4]
     * Offset: 0x100~0x10C  A/D Data Register Double Buffer for SAMPLE An, n=0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |RSLTDB    |A/D Conversion Result
     * |        |          |This field contains 12-bit conversion result.
     * |[16]    |VALID     |Valid Flag
     * |        |          |0 = Data in RSLT[11:0] bits is not valid.
     * |        |          |1 = Data in RSLT[11:0] bits is valid.
     * |        |          |This bit is set to 1 when corresponding SAMPLE channel analog input conversion is
     * |        |          |completed and cleared by hardware after ADDR register is read.
     * @var EADC_T::ADDRDBB[4]
     * Offset: 0x120~0x12C  A/D Data Register Double Buffer for SAMPLE Bn, n=0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |RSLTDB    |A/D Conversion Result
     * |        |          |This field contains 12-bit conversion result.
     * |[16]    |VALID     |Valid Flag
     * |        |          |0 = Data in RSLT[11:0] bits is not valid.
     * |        |          |1 = Data in RSLT[11:0] bits is valid.
     * |        |          |This bit is set to 1 when corresponding SAMPLE channel analog input conversion is
     * |        |          |completed and cleared by hardware after ADDR register is read.
     * @var EADC_T::ADDBM
     * Offset: 0x130  A/D Double Buffer Mode select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DBMA0     |Double Buffer Mode For SAMPLE A0
     * |        |          |0 = SampleA0 has one sample result register. (default).
     * |        |          |1 = SampleA0 has two sample result registers.
     * |[1]     |DBMA1     |Double Buffer Mode For SAMPLE A1
     * |        |          |0 = SampleA1 has one sample result register. (default).
     * |        |          |1 = SampleA1 has two sample result registers.
     * |[2]     |DBMA2     |Double Buffer Mode For SAMPLE A2
     * |        |          |0 = SampleA2 has one sample result register. (default).
     * |        |          |1 = SampleA2 has two sample result registers.
     * |[3]     |DBMA3     |Double Buffer Mode For SAMPLE A3
     * |        |          |0 = SampleA3 has one sample result register. (default).
     * |        |          |1 = SampleA3 has two sample result registers.
     * |[8]     |DBMB0     |Double Buffer Mode For SAMPLE B0
     * |        |          |0 = SampleB0 has one sample result register. (default).
     * |        |          |1 = SampleB0 has two sample result registers.
     * |[9]     |DBMB1     |Double Buffer Mode For SAMPLE B1
     * |        |          |0 = SampleB1 has one sample result register. (default).
     * |        |          |1 = SampleB1 has two sample result registers.
     * |[10]    |DBMB2     |Double Buffer Mode For SAMPLE B2
     * |        |          |0 = SampleB2 has one sample result register. (default).
     * |        |          |1 = SampleB2 has two sample result registers.
     * |[11]    |DBMB3     |Double Buffer Mode For SAMPLE B3
     * |        |          |0 = SampleB3 has one sample result register. (default).
     * |        |          |1 = SampleB3 has two sample result registers.
     * @var EADC_T::ADINTSRCTL[4]
     * Offset: 0x134~0x140  A/D interrupt n Source Enable Control Register, n=0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IESPLA0   |SAMPLE A0 Interrupt Mask Enable
     * |        |          |0 = SAMPLE A0 interrupt mask Disabled.
     * |        |          |1 = SAMPLE A0 interrupt mask Enabled.
     * |[1]     |IESPLA1   |SAMPLE A1 Interrupt Mask Enable
     * |        |          |0 = SAMPLE A1 interrupt mask Disabled.
     * |        |          |1 = SAMPLE A1 interrupt mask Enabled.
     * |[2]     |IESPLA2   |SAMPLE A2 Interrupt Mask Enable
     * |        |          |0 = SAMPLE A2 interrupt mask Disabled.
     * |        |          |1 = SAMPLE A2 interrupt mask Enabled.
     * |[3]     |IESPLA3   |SAMPLE A3 Interrupt Mask Enable
     * |        |          |0 = SAMPLE A3 interrupt mask Disabled.
     * |        |          |1 = SAMPLE A3 interrupt mask Enabled.
     * |[4]     |IESPLA4   |SAMPLE A4 Interrupt Mask Enable
     * |        |          |0 = SAMPLE A4 interrupt mask Disabled.
     * |        |          |1 = SAMPLE A4 interrupt mask Enabled.
     * |[5]     |IESPLA5   |SAMPLE A5 Interrupt Mask Enable
     * |        |          |0 = SAMPLE A5 interrupt mask Disabled.
     * |        |          |1 = SAMPLE A5 interrupt mask Enabled.
     * |[6]     |IESPLA6   |SAMPLE A6 Interrupt Mask Enable
     * |        |          |0 = SAMPLE A6 interrupt mask Disabled.
     * |        |          |1 = SAMPLE A6 interrupt mask Enabled.
     * |[7]     |IESPLA7   |SAMPLE A7 Interrupt Mask Enable
     * |        |          |0 = SAMPLE A7 interrupt mask Disabled.
     * |        |          |1 = SAMPLE A7 interrupt mask Enabled.
     * |[8]     |IESPLB0   |SAMPLE B0 Interrupt Mask Enable
     * |        |          |0 = SAMPLE B0 interrupt mask Disabled.
     * |        |          |1 = SAMPLE B0 interrupt mask Enabled.
     * |[9]     |IESPLB1   |SAMPLE B1 Interrupt Mask Enable
     * |        |          |0 = SAMPLE B1 interrupt mask Disabled.
     * |        |          |1 = SAMPLE B1 interrupt mask Enabled.
     * |[10]    |IESPLB2   |SAMPLE B2 Interrupt Mask Enable
     * |        |          |0 = SAMPLE B2 interrupt mask Disabled.
     * |        |          |1 = SAMPLE B2 interrupt mask Enabled.
     * |[11]    |IESPLB3   |SAMPLE B3 Interrupt Mask Enable
     * |        |          |0 = SAMPLE B3 interrupt mask Disabled.
     * |        |          |1 = SAMPLE B3 interrupt mask Enabled.
     * |[12]    |IESPLB4   |SAMPLE B4 Interrupt Mask Enable
     * |        |          |0 = SAMPLE B4 interrupt mask Disabled.
     * |        |          |1 = SAMPLE B4 interrupt mask Enabled.
     * |[13]    |IESPLB5   |SAMPLE B5 Interrupt Mask Enable
     * |        |          |0 = SAMPLE B5 interrupt mask Disabled.
     * |        |          |1 = SAMPLE B5 interrupt mask Enabled.
     * |[14]    |IESPLB6   |SAMPLE B6 Interrupt Mask Enable
     * |        |          |0 = SAMPLE B6 interrupt mask Disabled.
     * |        |          |1 = SAMPLE B6 interrupt mask Enabled.
     * |[15]    |IESPLB7   |SAMPLE B7 Interrupt Mask Enable
     * |        |          |0 = SAMPLE B7 interrupt mask Disabled.
     * |        |          |1 = SAMPLE B7 interrupt mask Enabled.
     * @var EADC_T::SMPTRGA[4]
     * Offset: 0x144~0x150  A/D trigger condition for SAMPLEAn, n=0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWM00REN  |PWM00 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[1]     |PWM00FEN  |PWM00 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[2]     |PWM00PEN  |PWM00 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[3]     |PWM00CEN  |PWM00 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[4]     |PWM02REN  |PWM02 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[5]     |PWM02FEN  |PWM02 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[6]     |PWM02PEN  |PWM02 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[7]     |PWM02CEN  |PWM00 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[8]     |PWM04REN  |PWM04 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[9]     |PWM04FEN  |PWM04 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[10]    |PWM04PEN  |PWM04 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[11]    |PWM04CEN  |PWM04 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[12]    |PWM10REN  |PWM10 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[13]    |PWM10FEN  |PWM10 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[14]    |PWM10PEN  |PWM10 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[15]    |PWM10CEN  |PWM10 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[16]    |PWM12REN  |PWM12 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[17]    |PWM12FEN  |PWM12 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[18]    |PWM12PEN  |PWM12 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[19]    |PWM12CEN  |PWM12 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[20]    |PWM14REN  |PWM14 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[21]    |PWM14FEN  |PWM14 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[22]    |PWM14PEN  |PWM14 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[23]    |PWM14CEN  |PWM14 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[24]    |PWM20REN  |PWM20 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[25]    |PWM20FEN  |PWM20 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[26]    |PWM20PEN  |PWM20 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[27]    |PWM20CEN  |PWM20 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[28]    |PWM21REN  |PWM21 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[29]    |PWM21FEN  |PWM21 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[30]    |PWM21PEN  |PWM21 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[31]    |PWM21CEN  |PWM21 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * @var EADC_T::SMPTRGB[4]
     * Offset: 0x154~0x160  A/D trigger condition for SAMPLEBn, n=0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWM00REN  |PWM00 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[1]     |PWM00FEN  |PWM00 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[2]     |PWM00PEN  |PWM00 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[3]     |PWM00CEN  |PWM00 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[4]     |PWM02REN  |PWM02 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[5]     |PWM02FEN  |PWM02 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[6]     |PWM02PEN  |PWM02 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[7]     |PWM02CEN  |PWM00 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[8]     |PWM04REN  |PWM04 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[9]     |PWM04FEN  |PWM04 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[10]    |PWM04PEN  |PWM04 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[11]    |PWM04CEN  |PWM04 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[12]    |PWM10REN  |PWM10 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[13]    |PWM10FEN  |PWM10 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[14]    |PWM10PEN  |PWM10 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[15]    |PWM10CEN  |PWM10 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[16]    |PWM12REN  |PWM12 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[17]    |PWM12FEN  |PWM12 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[18]    |PWM12PEN  |PWM12 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[19]    |PWM12CEN  |PWM12 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[20]    |PWM14REN  |PWM14 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[21]    |PWM14FEN  |PWM14 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[22]    |PWM14PEN  |PWM14 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[23]    |PWM14CEN  |PWM14 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[24]    |PWM20REN  |PWM20 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[25]    |PWM20FEN  |PWM20 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[26]    |PWM20PEN  |PWM20 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[27]    |PWM20CEN  |PWM20 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[28]    |PWM21REN  |PWM21 Rising Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[29]    |PWM21FEN  |PWM21 Falling Edge Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[30]    |PWM21PEN  |PWM21 Period Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[31]    |PWM21CEN  |PWM21 Center Trigger Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     */

    __I  uint32_t ADDRA[8];      /* Offset: 0x00~0x1C  A/D Data Register n for SAMPLEAn, n=0~7                       */
    __I  uint32_t ADDRB[8];      /* Offset: 0x20~0x3C  A/D Data Register m for SAMPLEAn, m=8~15, n=0~7               */
    __IO uint32_t ADCR;          /* Offset: 0x40  EADC Control Register                                              */
    __IO uint32_t ADCHISELR;     /* Offset: 0x44  A/D Channel Input Sources Select Register                          */
    __O  uint32_t ADSSTR;        /* Offset: 0x48  A/D SAMPLE Software Start Register                                 */
    __I  uint32_t ADSTPFR;       /* Offset: 0x4C  A/D SAMPLE Start of Conversion Pending Flag Register               */
    __IO uint32_t ADIFOVR;       /* Offset: 0x50  A/D ADINT3~0 Interrupt Flag Over Run Register                      */
    __IO uint32_t ADSPOVFR;      /* Offset: 0x54  A/D SAMPLE Start of Conversion Over Run Flag Register              */
    __IO uint32_t ADSPCRA[8];    /* Offset: 0x58~0x74 ADSPCRA0~7 A/D SAMPLEAn Control Register, n=0~7                */
    __IO uint32_t ADSPCRB[8];    /* Offset: 0x78~0x94 ADSPCRB0~7 A/D SAMPLEBn Control Register, n=0~7                */
    __I  uint32_t RESERVE0[3];                                                                                         
    __IO uint32_t ADSMSELR;      /* Offset: 0xA4  A/D SAMPLE Simultaneous Mode Select Register                       */
    __IO uint32_t ADCMPR[2];     /* Offset: 0xA8~0xAC  A/D Result Compare Register 0/1                               */
    __I  uint32_t ADSR0;         /* Offset: 0xB0  A/D Status Register 0                                              */
    __IO uint32_t ADSR1;         /* Offset: 0xB4  A/D Status Register 1                                              */
    __IO uint32_t ADTCR;         /* Offset: 0xB8  A/D Timing Control Register                                        */
    __I  uint32_t RESERVE1[17];                                                                                        
    __I  uint32_t ADDRDBA[4];    /* Offset: 0x100~0x10C  A/D Data Register Double Buffer for SAMPLE An, n=0~3        */
    __I  uint32_t RESERVE2[4];                                                                                         
    __I  uint32_t ADDRDBB[4];    /* Offset: 0x120~0x12C  A/D Data Register Double Buffer for SAMPLE Bn, n=0~3        */
    __IO uint32_t ADDBM;         /* Offset: 0x130  A/D Double Buffer Mode select Register                            */
    __IO uint32_t ADINTSRCTL[4]; /* Offset: 0x134~0x140  A/D interrupt n Source Enable Control Register, n=0~3       */
    __IO uint32_t SMPTRGA[4];    /* Offset: 0x144~0x150  A/D trigger condition for SAMPLEAn, n=0~3                   */
    __IO uint32_t SMPTRGB[4];    /* Offset: 0x154~0x160  A/D trigger condition for SAMPLEBn, n=0~3                   */

} EADC_T;



/**
    @addtogroup EADC_CONST EADC Bit Field Definition
    Constant Definitions for EADC Controller
@{ */

/* ADDR Bit Field Definitions */
#define EADC_ADDR_VALID_Pos           17                                      /*!< EADC_T::ADDR: VALID Position */
#define EADC_ADDR_VALID_Msk           (1ul << EADC_ADDR_VALID_Pos)            /*!< EADC_T::ADDR: VALID Mask */

#define EADC_ADDR_OVERRUN_Pos         16                                      /*!< EADC_T::ADDR: OVERRUN Position */
#define EADC_ADDR_OVERRUN_Msk         (1ul << EADC_ADDR_OVERRUN_Pos)          /*!< EADC_T::ADDR: OVERRUN Mask */

#define EADC_ADDR_RSLT_Pos            0                                       /*!< EADC_T::ADDR: RSLT Position */
#define EADC_ADDR_RSLT_Msk            (0xFFFul << EADC_ADDR_RSLT_Pos)         /*!< EADC_T::ADDR: RSLT Mask */

/* ADDRA Bit Field Definitions */
#define EADC_ADDRA_VALID_Pos          17                                      /*!< EADC_T::ADDRA: VALID Position */
#define EADC_ADDRA_VALID_Msk          (1ul << EADC_ADDRA_VALID_Pos)           /*!< EADC_T::ADDRA: VALID Mask */

#define EADC_ADDRA_OVERRUN_Pos        16                                      /*!< EADC_T::ADDRA: OVERRUN Position */
#define EADC_ADDRA_OVERRUN_Msk        (1ul << EADC_ADDRA_OVERRUN_Pos)         /*!< EADC_T::ADDRA: OVERRUN Mask */

#define EADC_ADDRA_RSLT_Pos           0                                       /*!< EADC_T::ADDRA: RSLT Position */
#define EADC_ADDRA_RSLT_Msk           (0xFFFul << EADC_ADDRA_RSLT_Pos)        /*!< EADC_T::ADDRA: RSLT Mask */

/* ADDRB Bit Field Definitions */
#define EADC_ADDRB_VALID_Pos          17                                      /*!< EADC_T::ADDRB: VALID Position */
#define EADC_ADDRB_VALID_Msk          (1ul << EADC_ADDRB_VALID_Pos)           /*!< EADC_T::ADDRB: VALID Mask */

#define EADC_ADDRB_OVERRUN_Pos        16                                      /*!< EADC_T::ADDRB: OVERRUN Position */
#define EADC_ADDRB_OVERRUN_Msk        (1ul << EADC_ADDRB_OVERRUN_Pos)         /*!< EADC_T::ADDRB: OVERRUN Mask */

#define EADC_ADDRB_RSLT_Pos           0                                       /*!< EADC_T::ADDRB: RSLT Position */
#define EADC_ADDRB_RSLT_Msk           (0xFFFul << EADC_ADDRB_RSLT_Pos)        /*!< EADC_T::ADDRB: RSLT Mask */

/* ADCR Bit Field Definitions */
#define EADC_ADCR_ADIE3_Pos           5                                       /*!< EADC_T::ADCR: ADIE3 Position */
#define EADC_ADCR_ADIE3_Msk           (1ul << EADC_ADCR_ADIE3_Pos)            /*!< EADC_T::ADCR: ADIE3 Mask */

#define EADC_ADCR_ADIE2_Pos           4                                       /*!< EADC_T::ADCR: ADIE2 Position */
#define EADC_ADCR_ADIE2_Msk           (1ul << EADC_ADCR_ADIE2_Pos)            /*!< EADC_T::ADCR: ADIE2 Mask */

#define EADC_ADCR_ADIE1_Pos           3                                       /*!< EADC_T::ADCR: ADIE1 Position */
#define EADC_ADCR_ADIE1_Msk           (1ul << EADC_ADCR_ADIE1_Pos)            /*!< EADC_T::ADCR: ADIE1 Mask */

#define EADC_ADCR_ADIE0_Pos           2                                       /*!< EADC_T::ADCR: ADIE0 Position */
#define EADC_ADCR_ADIE0_Msk           (1ul << EADC_ADCR_ADIE0_Pos)            /*!< EADC_T::ADCR: ADIE0 Mask */

#define EADC_ADCR_ADRESET_Pos         1                                       /*!< EADC_T::ADCR: ADRESET Position */
#define EADC_ADCR_ADRESET_Msk         (1ul << EADC_ADCR_ADRESET_Pos)          /*!< EADC_T::ADCR: ADRESET Mask */

#define EADC_ADCR_AD_EN_Pos           0                                       /*!< EADC_T::ADCR: AD_EN Position */
#define EADC_ADCR_AD_EN_Msk           (1ul << EADC_ADCR_AD_EN_Pos)            /*!< EADC_T::ADCR: AD_EN Mask */

/* ADCHISELR Bit Field Definitions */
#define EADC_ADCHISELR_PRESEL_Pos     2                                       /*!< EADC_T::ADCHISELR: PRESEL Position */
#define EADC_ADCHISELR_PRESEL_Msk     (3ul << EADC_ADCHISELR_PRESEL_Pos)      /*!< EADC_T::ADCHISELR: PRESEL Mask */

#define EADC_ADCHISELR_AINB0SEL_Pos   1                                       /*!< EADC_T::ADCHISELR: AINB0SEL Position */
#define EADC_ADCHISELR_AINB0SEL_Msk   (1ul << EADC_ADCHISELR_AINB0SEL_Pos)    /*!< EADC_T::ADCHISELR: AINB0SEL Mask */

#define EADC_ADCHISELR_AINA0SEL_Pos   0                                       /*!< EADC_T::ADCHISELR: AINA0SEL Position */
#define EADC_ADCHISELR_AINA0SEL_Msk   (1ul << EADC_ADCHISELR_AINA0SEL_Pos)    /*!< EADC_T::ADCHISELR: AINA0SEL Mask */

/* ADSSTR Bit Field Definitions */
#define EADC_ADSSTR_ADST_Pos          0                                       /*!< EADC_T::ADSSTR: ADST Position */
#define EADC_ADSSTR_ADST_Msk          (0xFFFFul << EADC_ADSSTR_ADST_Pos)      /*!< EADC_T::ADSSTR: ADST Mask */

/* ADSTPFR Bit Field Definitions */
#define EADC_ADSTPFR_STPF_Pos         16                                      /*!< EADC_T::ADSTPFR: STPF Position */
#define EADC_ADSTPFR_STPF_Msk         (0xFFFFul << EADC_ADSTPFR_STPF_Pos)     /*!< EADC_T::ADSTPFR: STPF Mask */

/* ADIFOVR Bit Field Definitions */
#define EADC_ADIFOVR_ADFOV0_Pos       0                                       /*!< EADC_T::ADIFOVR: ADFOV0 Position */
#define EADC_ADIFOVR_ADFOV0_Msk       (1ul << EADC_ADIFOVR_ADFOV0_Pos)        /*!< EADC_T::ADIFOVR: ADFOV0 Mask */

#define EADC_ADIFOVR_ADFOV1_Pos       1                                       /*!< EADC_T::ADIFOVR: ADFOV1 Position */
#define EADC_ADIFOVR_ADFOV1_Msk       (1ul << EADC_ADIFOVR_ADFOV1_Pos)        /*!< EADC_T::ADIFOVR: ADFOV1 Mask */

#define EADC_ADIFOVR_ADFOV2_Pos       2                                       /*!< EADC_T::ADIFOVR: ADFOV2 Position */
#define EADC_ADIFOVR_ADFOV2_Msk       (1ul << EADC_ADIFOVR_ADFOV2_Pos)        /*!< EADC_T::ADIFOVR: ADFOV2 Mask */

#define EADC_ADIFOVR_ADFOV3_Pos       3                                       /*!< EADC_T::ADIFOVR: ADFOV3 Position */
#define EADC_ADIFOVR_ADFOV3_Msk       (1ul << EADC_ADIFOVR_ADFOV3_Pos)        /*!< EADC_T::ADIFOVR: ADFOV3 Mask */

/* ADSPOVFR Bit Field Definitions */
#define EADC_ADSPOVFR_SPOVF_Pos       0                                       /*!< EADC_T::ADSPOVFR: SPOVF Position */
#define EADC_ADSPOVFR_SPOVF_Msk       (0xFFFFul << EADC_ADSPOVFR_SPOVF_Pos)   /*!< EADC_T::ADSPOVFR: SPOVF Mask */

/* ADSPCR Bit Field Definitions */
#define EADC_ADSPCR_EXTFEN_Pos        21                                      /*!< EADC_T::ADSPCR: EXTFEN Position */
#define EADC_ADSPCR_EXTFEN_Msk        (1ul << EADC_ADSPCR_EXTFEN_Pos)         /*!< EADC_T::ADSPCR: EXTFEN Mask */

#define EADC_ADSPCR_EXTREN_Pos        20                                      /*!< EADC_T::ADSPCR: EXTREN Position */
#define EADC_ADSPCR_EXTREN_Msk        (1ul << EADC_ADSPCR_EXTREN_Pos)         /*!< EADC_T::ADSPCR: EXTREN Mask */

#define EADC_ADSPCR_TRGDLYDIV_Pos     16                                      /*!< EADC_T::ADSPCR: TRGDLYDIV Position */
#define EADC_ADSPCR_TRGDLYDIV_Msk     (3ul << EADC_ADSPCR_TRGDLYDIV_Pos)      /*!< EADC_T::ADSPCR: TRGDLYDIV Mask */

#define EADC_ADSPCR_TRGDLYCNT_Pos     8                                       /*!< EADC_T::ADSPCR: TRGDLYCNT Position */
#define EADC_ADSPCR_TRGDLYCNT_Msk     (0xFFul << EADC_ADSPCR_TRGDLYCNT_Pos)   /*!< EADC_T::ADSPCR: TRGDLYCNT Mask */

#define EADC_ADSPCR_TRGSEL_Pos        4                                       /*!< EADC_T::ADSPCR: TRGSEL Position */
#define EADC_ADSPCR_TRGSEL_Msk        (0xFul << EADC_ADSPCR_TRGSEL_Pos)       /*!< EADC_T::ADSPCR: TRGSEL Mask */

#define EADC_ADSPCR_CHSEL_Pos         0                                       /*!< EADC_T::ADSPCR: CHSEL Position */
#define EADC_ADSPCR_CHSEL_Msk         (7ul << EADC_ADSPCR_CHSEL_Pos)          /*!< EADC_T::ADSPCR: CHSEL Mask */

/* ADSPCRA Bit Field Definitions */
#define EADC_ADSPCRA_EXTFEN_Pos       21                                      /*!< EADC_T::ADSPCRA: EXTFEN Position */
#define EADC_ADSPCRA_EXTFEN_Msk       (1ul << EADC_ADSPCRA_EXTFEN_Pos)        /*!< EADC_T::ADSPCRA: EXTFEN Mask */

#define EADC_ADSPCRA_EXTREN_Pos       20                                      /*!< EADC_T::ADSPCRA: EXTREN Position */
#define EADC_ADSPCRA_EXTREN_Msk       (1ul << EADC_ADSPCRA_EXTREN_Pos)        /*!< EADC_T::ADSPCRA: EXTREN Mask */

#define EADC_ADSPCRA_TRGDLYDIV_Pos    16                                      /*!< EADC_T::ADSPCRA: TRGDLYDIV Position */
#define EADC_ADSPCRA_TRGDLYDIV_Msk    (3ul << EADC_ADSPCRA_TRGDLYDIV_Pos)     /*!< EADC_T::ADSPCRA: TRGDLYDIV Mask */

#define EADC_ADSPCRA_TRGDLYCNT_Pos    8                                       /*!< EADC_T::ADSPCRA: TRGDLYCNT Position */
#define EADC_ADSPCRA_TRGDLYCNT_Msk    (0xFFul << EADC_ADSPCRA_TRGDLYCNT_Pos)  /*!< EADC_T::ADSPCRA: TRGDLYCNT Mask */

#define EADC_ADSPCRA_TRGSEL_Pos       4                                       /*!< EADC_T::ADSPCRA: TRGSEL Position */
#define EADC_ADSPCRA_TRGSEL_Msk       (0xFul << EADC_ADSPCRA_TRGSEL_Pos)      /*!< EADC_T::ADSPCRA: TRGSEL Mask */

#define EADC_ADSPCRA_CHSEL_Pos        0                                       /*!< EADC_T::ADSPCRA: CHSEL Position */
#define EADC_ADSPCRA_CHSEL_Msk        (7ul << EADC_ADSPCRA_CHSEL_Pos)         /*!< EADC_T::ADSPCRA: CHSEL Mask */

/* ADSPCRB Bit Field Definitions */
#define EADC_ADSPCRB_EXTFEN_Pos       21                                      /*!< EADC_T::ADSPCRB: EXTFEN Position */
#define EADC_ADSPCRB_EXTFEN_Msk       (1ul << EADC_ADSPCRB_EXTFEN_Pos)        /*!< EADC_T::ADSPCRB: EXTFEN Mask */

#define EADC_ADSPCRB_EXTREN_Pos       20                                      /*!< EADC_T::ADSPCRB: EXTREN Position */
#define EADC_ADSPCRB_EXTREN_Msk       (1ul << EADC_ADSPCRB_EXTREN_Pos)        /*!< EADC_T::ADSPCRB: EXTREN Mask */

#define EADC_ADSPCRB_TRGDLYDIV_Pos    16                                      /*!< EADC_T::ADSPCRB: TRGDLYDIV Position */
#define EADC_ADSPCRB_TRGDLYDIV_Msk    (3ul << EADC_ADSPCRB_TRGDLYDIV_Pos)     /*!< EADC_T::ADSPCRB: TRGDLYDIV Mask */

#define EADC_ADSPCRB_TRGDLYCNT_Pos    8                                       /*!< EADC_T::ADSPCRB: TRGDLYCNT Position */
#define EADC_ADSPCRB_TRGDLYCNT_Msk    (0xFFul << EADC_ADSPCRB_TRGDLYCNT_Pos)  /*!< EADC_T::ADSPCRB: TRGDLYCNT Mask */

#define EADC_ADSPCRB_TRGSEL_Pos       4                                       /*!< EADC_T::ADSPCRB: TRGSEL Position */
#define EADC_ADSPCRB_TRGSEL_Msk       (0xFul << EADC_ADSPCRB_TRGSEL_Pos)      /*!< EADC_T::ADSPCRB: TRGSEL Mask */

#define EADC_ADSPCRB_CHSEL_Pos        0                                       /*!< EADC_T::ADSPCRB: CHSEL Position */
#define EADC_ADSPCRB_CHSEL_Msk        (7ul << EADC_ADSPCRB_CHSEL_Pos)         /*!< EADC_T::ADSPCRB: CHSEL Mask */

/* ADSMSELR Bit Field Definitions */
#define EADC_ADSMSELR_SIMUSEL_Pos     0                                       /*!< EADC_T::ADSMSELR: SIMUSEL Position */
#define EADC_ADSMSELR_SIMUSEL_Msk     (0xFFul << EADC_ADSMSELR_SIMUSEL_Pos)   /*!< EADC_T::ADSMSELR: SIMUSEL Mask */

/* ADCMPR Bit Field Definitions */
#define EADC_ADCMPR_CMPD_Pos          16                                      /*!< EADC_T::ADCMPR: CMPD Position */
#define EADC_ADCMPR_CMPD_Msk          (0xFFFul << EADC_ADCMPR_CMPD_Pos)       /*!< EADC_T::ADCMPR: CMPD Mask */

#define EADC_ADCMPR_CMPMATCNT_Pos     8                                       /*!< EADC_T::ADCMPR: CMPMATCNT Position */
#define EADC_ADCMPR_CMPMATCNT_Msk     (0xFul << EADC_ADCMPR_CMPMATCNT_Pos)    /*!< EADC_T::ADCMPR: CMPMATCNT Mask */

#define EADC_ADCMPR_CMPSMPL_Pos       3                                       /*!< EADC_T::ADCMPR: CMPSMPL Position */
#define EADC_ADCMPR_CMPSMPL_Msk       (7ul << EADC_ADCMPR_CMPSMPL_Pos)        /*!< EADC_T::ADCMPR: CMPSMPL Mask */

#define EADC_ADCMPR_CMPCOND_Pos       2                                       /*!< EADC_T::ADCMPR: CMPCOND Position */
#define EADC_ADCMPR_CMPCOND_Msk       (1ul << EADC_ADCMPR_CMPCOND_Pos)        /*!< EADC_T::ADCMPR: CMPCOND Mask */

#define EADC_ADCMPR_ADCMPIE_Pos       1                                       /*!< EADC_T::ADCMPR: ADCMPIE Position */
#define EADC_ADCMPR_ADCMPIE_Msk       (1ul << EADC_ADCMPR_ADCMPIE_Pos)        /*!< EADC_T::ADCMPR: ADCMPIE Mask */

#define EADC_ADCMPR_ADCMP_EN_Pos      0                                       /*!< EADC_T::ADCMPR: ADCMP_EN Position */
#define EADC_ADCMPR_ADCMP_EN_Msk      (1ul << EADC_ADCMPR_ADCMP_EN_Pos)       /*!< EADC_T::ADCMPR: ADCMP_EN Mask */

/* ADSR0 Bit Field Definitions */
#define EADC_ADSR0_OVERRUN_Pos        16                                      /*!< EADC_T::ADSR0: OVERRUN Position */
#define EADC_ADSR0_OVERRUN_Msk        (0xFFFFul << EADC_ADSR0_OVERRUN_Pos)    /*!< EADC_T::ADSR0: OVERRUN Mask */

#define EADC_ADSR0_VALID_Pos          0                                       /*!< EADC_T::ADSR0: VALID Position */
#define EADC_ADSR0_VALID_Msk          (0xFFFFul << EADC_ADSR0_VALID_Pos)      /*!< EADC_T::ADSR0: VALID Mask */

/* ADSR1 Bit Field Definitions */
#define EADC_ADSR1_AOVERRUN_Pos       27                                      /*!< EADC_T::ADSR1: AOVERRUN Position */
#define EADC_ADSR1_AOVERRUN_Msk       (1ul << EADC_ADSR1_AOVERRUN_Pos)        /*!< EADC_T::ADSR1: AOVERRUN Mask */

#define EADC_ADSR1_AVALID_Pos         26                                      /*!< EADC_T::ADSR1: AVALID Position */
#define EADC_ADSR1_AVALID_Msk         (1ul << EADC_ADSR1_AVALID_Pos)          /*!< EADC_T::ADSR1: AVALID Mask */

#define EADC_ADSR1_ASPOVF_Pos         25                                      /*!< EADC_T::ADSR1: ASPOVF Position */
#define EADC_ADSR1_ASPOVF_Msk         (1ul << EADC_ADSR1_ASPOVF_Pos)          /*!< EADC_T::ADSR1: ASPOVF Mask */

#define EADC_ADSR1_AADFOV_Pos         24                                      /*!< EADC_T::ADSR1: AADFOV Position */
#define EADC_ADSR1_AADFOV_Msk         (1ul << EADC_ADSR1_AADFOV_Pos)          /*!< EADC_T::ADSR1: AADFOV Mask */

#define EADC_ADSR1_CHANNELB_Pos       20                                      /*!< EADC_T::ADSR1: CHANNELB Position */
#define EADC_ADSR1_CHANNELB_Msk       (7ul << EADC_ADSR1_CHANNELB_Pos)        /*!< EADC_T::ADSR1: CHANNELB Mask */

#define EADC_ADSR1_BUSYB_Pos          16                                      /*!< EADC_T::ADSR1: BUSYB Position */
#define EADC_ADSR1_BUSYB_Msk          (1ul << EADC_ADSR1_BUSYB_Pos)           /*!< EADC_T::ADSR1: BUSYB Mask */

#define EADC_ADSR1_CHANNELA_Pos       12                                      /*!< EADC_T::ADSR1: CHANNELA Position */
#define EADC_ADSR1_CHANNELA_Msk       (7ul << EADC_ADSR1_CHANNELA_Pos)        /*!< EADC_T::ADSR1: CHANNELA Mask */

#define EADC_ADSR1_BUSYA_Pos          8                                       /*!< EADC_T::ADSR1: BUSYA Position */
#define EADC_ADSR1_BUSYA_Msk          (1ul << EADC_ADSR1_BUSYA_Pos)           /*!< EADC_T::ADSR1: BUSYA Mask */

#define EADC_ADSR1_ADCMPF1_Pos        7                                       /*!< EADC_T::ADSR1: ADCMPF1 Position */
#define EADC_ADSR1_ADCMPF1_Msk        (1ul << EADC_ADSR1_ADCMPF1_Pos)         /*!< EADC_T::ADSR1: ADCMPF1 Mask */

#define EADC_ADSR1_ADCMPF0_Pos        6                                       /*!< EADC_T::ADSR1: ADCMPF0 Position */
#define EADC_ADSR1_ADCMPF0_Msk        (1ul << EADC_ADSR1_ADCMPF0_Pos)         /*!< EADC_T::ADSR1: ADCMPF0 Mask */

#define EADC_ADSR1_ADCMPO1_Pos        5                                       /*!< EADC_T::ADSR1: ADCMPO1 Position */
#define EADC_ADSR1_ADCMPO1_Msk        (1ul << EADC_ADSR1_ADCMPO1_Pos)         /*!< EADC_T::ADSR1: ADCMPO1 Mask */

#define EADC_ADSR1_ADCMPO0_Pos        4                                       /*!< EADC_T::ADSR1: ADCMPO0 Position */
#define EADC_ADSR1_ADCMPO0_Msk        (1ul << EADC_ADSR1_ADCMPO0_Pos)         /*!< EADC_T::ADSR1: ADCMPO0 Mask */

#define EADC_ADSR1_ADF3_Pos           3                                       /*!< EADC_T::ADSR1: ADF3 Position */
#define EADC_ADSR1_ADF3_Msk           (1ul << EADC_ADSR1_ADF3_Pos)            /*!< EADC_T::ADSR1: ADF3 Mask */

#define EADC_ADSR1_ADF2_Pos           2                                       /*!< EADC_T::ADSR1: ADF2 Position */
#define EADC_ADSR1_ADF2_Msk           (1ul << EADC_ADSR1_ADF2_Pos)            /*!< EADC_T::ADSR1: ADF2 Mask */

#define EADC_ADSR1_ADF1_Pos           1                                       /*!< EADC_T::ADSR1: ADF1 Position */
#define EADC_ADSR1_ADF1_Msk           (1ul << EADC_ADSR1_ADF1_Pos)            /*!< EADC_T::ADSR1: ADF1 Mask */

#define EADC_ADSR1_ADF0_Pos           0                                       /*!< EADC_T::ADSR1: ADF0 Position */
#define EADC_ADSR1_ADF0_Msk           (1ul << EADC_ADSR1_ADF0_Pos)            /*!< EADC_T::ADSR1: ADF0 Mask */

/* ADTCR Bit Field Definitions */
#define EADC_ADTCR_ADBEST_Pos         16                                      /*!< EADC_T::ADTCR: ADBEST Position */
#define EADC_ADTCR_ADBEST_Msk         (0xFFul << EADC_ADTCR_ADBEST_Pos)       /*!< EADC_T::ADTCR: ADBEST Mask */

#define EADC_ADTCR_ADAEST_Pos         0                                       /*!< EADC_T::ADTCR: ADAEST Position */
#define EADC_ADTCR_ADAEST_Msk         (0xFFul << EADC_ADTCR_ADAEST_Pos)       /*!< EADC_T::ADTCR: ADAEST Mask */

/* ADDRDB Bit Field Definitions */
#define EADC_ADDRDB_VALID_Pos         16                                      /*!< EADC_T::ADDRDB: VALID Position */
#define EADC_ADDRDB_VALID_Msk         (1ul << EADC_ADDRDB_VALID_Pos)          /*!< EADC_T::ADDRDB: VALID Mask */

#define EADC_ADDRDB_RSLTDB_Pos        0                                       /*!< EADC_T::ADDRDB: RSLTDB Position */
#define EADC_ADDRDB_RSLTDB_Msk        (0xFFFul << EADC_ADDRDB_RSLTDB_Pos)     /*!< EADC_T::ADDRDB: RSLTDB Mask */

/* ADDRDBA Bit Field Definitions */
#define EADC_ADDRDBA_VALID_Pos        16                                      /*!< EADC_T::ADDRDBA: VALID Position */
#define EADC_ADDRDBA_VALID_Msk        (1ul << EADC_ADDRDBA_VALID_Pos)         /*!< EADC_T::ADDRDBA: VALID Mask */

#define EADC_ADDRDBA_RSLTDB_Pos       0                                       /*!< EADC_T::ADDRDBA: RSLTDB Position */
#define EADC_ADDRDBA_RSLTDB_Msk       (0xFFFul << EADC_ADDRDBA_RSLTDB_Pos)    /*!< EADC_T::ADDRDBA: RSLTDB Mask */

/* ADDRDBB Bit Field Definitions */
#define EADC_ADDRDBB_VALID_Pos        16                                      /*!< EADC_T::ADDRDBB: VALID Position */
#define EADC_ADDRDBB_VALID_Msk        (1ul << EADC_ADDRDBB_VALID_Pos)         /*!< EADC_T::ADDRDBB: VALID Mask */

#define EADC_ADDRDBB_RSLTDB_Pos       0                                       /*!< EADC_T::ADDRDBB: RSLTDB Position */
#define EADC_ADDRDBB_RSLTDB_Msk       (0xFFFul << EADC_ADDRDBB_RSLTDB_Pos)    /*!< EADC_T::ADDRDBB: RSLTDB Mask */

/* ADDBM Bit Field Definitions */
#define EADC_ADDBM_DBMB3_Pos          11                                      /*!< EADC_T::ADDBM: DBMB3 Position */
#define EADC_ADDBM_DBMB3_Msk          (1ul << EADC_ADDBM_DBMB3_Pos)           /*!< EADC_T::ADDBM: DBMB3 Mask */

#define EADC_ADDBM_DBMB2_Pos          10                                      /*!< EADC_T::ADDBM: DBMB2 Position */
#define EADC_ADDBM_DBMB2_Msk          (1ul << EADC_ADDBM_DBMB2_Pos)           /*!< EADC_T::ADDBM: DBMB2 Mask */

#define EADC_ADDBM_DBMB1_Pos          9                                       /*!< EADC_T::ADDBM: DBMB1 Position */
#define EADC_ADDBM_DBMB1_Msk          (1ul << EADC_ADDBM_DBMB1_Pos)           /*!< EADC_T::ADDBM: DBMB1 Mask */

#define EADC_ADDBM_DBMB0_Pos          8                                       /*!< EADC_T::ADDBM: DBMB0 Position */
#define EADC_ADDBM_DBMB0_Msk          (1ul << EADC_ADDBM_DBMB0_Pos)           /*!< EADC_T::ADDBM: DBMB0 Mask */

#define EADC_ADDBM_DBMA3_Pos          3                                       /*!< EADC_T::ADDBM: DBMA3 Position */
#define EADC_ADDBM_DBMA3_Msk          (1ul << EADC_ADDBM_DBMA3_Pos)           /*!< EADC_T::ADDBM: DBMA3 Mask */

#define EADC_ADDBM_DBMA2_Pos          2                                       /*!< EADC_T::ADDBM: DBMA2 Position */
#define EADC_ADDBM_DBMA2_Msk          (1ul << EADC_ADDBM_DBMA2_Pos)           /*!< EADC_T::ADDBM: DBMA2 Mask */

#define EADC_ADDBM_DBMA1_Pos          1                                       /*!< EADC_T::ADDBM: DBMA1 Position */
#define EADC_ADDBM_DBMA1_Msk          (1ul << EADC_ADDBM_DBMA1_Pos)           /*!< EADC_T::ADDBM: DBMA1 Mask */

#define EADC_ADDBM_DBMA0_Pos          0                                       /*!< EADC_T::ADDBM: DBMA0 Position */
#define EADC_ADDBM_DBMA0_Msk          (1ul << EADC_ADDBM_DBMA0_Pos)           /*!< EADC_T::ADDBM: DBMA0 Mask */

/* ADINTSRCTL Bit Field Definitions */
#define EADC_ADINTSRCTL_IESPLB7_Pos   15                                      /*!< EADC_T::ADINTSRCTL: IESPLB7 Position */
#define EADC_ADINTSRCTL_IESPLB7_Msk   (1ul << EADC_ADINTSRCTL_IESPLB7_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLB7 Mask */

#define EADC_ADINTSRCTL_IESPLB6_Pos   14                                      /*!< EADC_T::ADINTSRCTL: IESPLB6 Position */
#define EADC_ADINTSRCTL_IESPLB6_Msk   (1ul << EADC_ADINTSRCTL_IESPLB6_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLB6 Mask */

#define EADC_ADINTSRCTL_IESPLB5_Pos   13                                      /*!< EADC_T::ADINTSRCTL: IESPLB5 Position */
#define EADC_ADINTSRCTL_IESPLB5_Msk   (1ul << EADC_ADINTSRCTL_IESPLB5_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLB5 Mask */

#define EADC_ADINTSRCTL_IESPLB4_Pos   12                                      /*!< EADC_T::ADINTSRCTL: IESPLB4 Position */
#define EADC_ADINTSRCTL_IESPLB4_Msk   (1ul << EADC_ADINTSRCTL_IESPLB4_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLB4 Mask */

#define EADC_ADINTSRCTL_IESPLB3_Pos   11                                      /*!< EADC_T::ADINTSRCTL: IESPLB3 Position */
#define EADC_ADINTSRCTL_IESPLB3_Msk   (1ul << EADC_ADINTSRCTL_IESPLB3_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLB3 Mask */

#define EADC_ADINTSRCTL_IESPLB2_Pos   10                                      /*!< EADC_T::ADINTSRCTL: IESPLB2 Position */
#define EADC_ADINTSRCTL_IESPLB2_Msk   (1ul << EADC_ADINTSRCTL_IESPLB2_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLB2 Mask */

#define EADC_ADINTSRCTL_IESPLB1_Pos   9                                       /*!< EADC_T::ADINTSRCTL: IESPLB1 Position */
#define EADC_ADINTSRCTL_IESPLB1_Msk   (1ul << EADC_ADINTSRCTL_IESPLB1_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLB1 Mask */

#define EADC_ADINTSRCTL_IESPLB0_Pos   8                                       /*!< EADC_T::ADINTSRCTL: IESPLB0 Position */
#define EADC_ADINTSRCTL_IESPLB0_Msk   (1ul << EADC_ADINTSRCTL_IESPLB0_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLB0 Mask */

#define EADC_ADINTSRCTL_IESPLA7_Pos   7                                       /*!< EADC_T::ADINTSRCTL: IESPLA7 Position */
#define EADC_ADINTSRCTL_IESPLA7_Msk   (1ul << EADC_ADINTSRCTL_IESPLA7_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLA7 Mask */

#define EADC_ADINTSRCTL_IESPLA6_Pos   6                                       /*!< EADC_T::ADINTSRCTL: IESPLA6 Position */
#define EADC_ADINTSRCTL_IESPLA6_Msk   (1ul << EADC_ADINTSRCTL_IESPLA6_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLA6 Mask */

#define EADC_ADINTSRCTL_IESPLA5_Pos   5                                       /*!< EADC_T::ADINTSRCTL: IESPLA5 Position */
#define EADC_ADINTSRCTL_IESPLA5_Msk   (1ul << EADC_ADINTSRCTL_IESPLA5_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLA5 Mask */

#define EADC_ADINTSRCTL_IESPLA4_Pos   4                                       /*!< EADC_T::ADINTSRCTL: IESPLA4 Position */
#define EADC_ADINTSRCTL_IESPLA4_Msk   (1ul << EADC_ADINTSRCTL_IESPLA4_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLA4 Mask */

#define EADC_ADINTSRCTL_IESPLA3_Pos   3                                       /*!< EADC_T::ADINTSRCTL: IESPLA3 Position */
#define EADC_ADINTSRCTL_IESPLA3_Msk   (1ul << EADC_ADINTSRCTL_IESPLA3_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLA3 Mask */

#define EADC_ADINTSRCTL_IESPLA2_Pos   2                                       /*!< EADC_T::ADINTSRCTL: IESPLA2 Position */
#define EADC_ADINTSRCTL_IESPLA2_Msk   (1ul << EADC_ADINTSRCTL_IESPLA2_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLA2 Mask */

#define EADC_ADINTSRCTL_IESPLA1_Pos   1                                       /*!< EADC_T::ADINTSRCTL: IESPLA1 Position */
#define EADC_ADINTSRCTL_IESPLA1_Msk   (1ul << EADC_ADINTSRCTL_IESPLA1_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLA1 Mask */

#define EADC_ADINTSRCTL_IESPLA0_Pos   0                                       /*!< EADC_T::ADINTSRCTL: IESPLA0 Position */
#define EADC_ADINTSRCTL_IESPLA0_Msk   (1ul << EADC_ADINTSRCTL_IESPLA0_Pos)    /*!< EADC_T::ADINTSRCTL: IESPLA0 Mask */

/* SMPTRG Bit Field Definitions */
#define EADC_SMPTRG_PWM21CEN_Pos      31                                    /*!< EADC_T::SMPTRG: PWM21CEN Position */
#define EADC_SMPTRG_PWM21CEN_Msk      (1ul << EADC_SMPTRG_PWM21CEN_Pos)     /*!< EADC_T::SMPTRG: PWM21CEN Mask */

#define EADC_SMPTRG_PWM21PEN_Pos      30                                    /*!< EADC_T::SMPTRG: PWM21PEN Position */
#define EADC_SMPTRG_PWM21PEN_Msk      (1ul << EADC_SMPTRG_PWM21PEN_Pos)     /*!< EADC_T::SMPTRG: PWM21PEN Mask */

#define EADC_SMPTRG_PWM21FEN_Pos      29                                    /*!< EADC_T::SMPTRG: PWM21FEN Position */
#define EADC_SMPTRG_PWM21FEN_Msk      (1ul << EADC_SMPTRG_PWM21FEN_Pos)     /*!< EADC_T::SMPTRG: PWM21FEN Mask */

#define EADC_SMPTRG_PWM21REN_Pos      28                                    /*!< EADC_T::SMPTRG: PWM21REN Position */
#define EADC_SMPTRG_PWM21REN_Msk      (1ul << EADC_SMPTRG_PWM21REN_Pos)     /*!< EADC_T::SMPTRG: PWM21REN Mask */

#define EADC_SMPTRG_PWM20CEN_Pos      27                                    /*!< EADC_T::SMPTRG: PWM20CEN Position */
#define EADC_SMPTRG_PWM20CEN_Msk      (1ul << EADC_SMPTRG_PWM20CEN_Pos)     /*!< EADC_T::SMPTRG: PWM20CEN Mask */

#define EADC_SMPTRG_PWM20PEN_Pos      26                                    /*!< EADC_T::SMPTRG: PWM20PEN Position */
#define EADC_SMPTRG_PWM20PEN_Msk      (1ul << EADC_SMPTRG_PWM20PEN_Pos)     /*!< EADC_T::SMPTRG: PWM20PEN Mask */

#define EADC_SMPTRG_PWM20FEN_Pos      25                                    /*!< EADC_T::SMPTRG: PWM20FEN Position */
#define EADC_SMPTRG_PWM20FEN_Msk      (1ul << EADC_SMPTRG_PWM20FEN_Pos)     /*!< EADC_T::SMPTRG: PWM20FEN Mask */

#define EADC_SMPTRG_PWM20REN_Pos      24                                    /*!< EADC_T::SMPTRG: PWM20REN Position */
#define EADC_SMPTRG_PWM20REN_Msk      (1ul << EADC_SMPTRG_PWM20REN_Pos)     /*!< EADC_T::SMPTRG: PWM20REN Mask */

#define EADC_SMPTRG_PWM14CEN_Pos      23                                    /*!< EADC_T::SMPTRG: PWM14CEN Position */
#define EADC_SMPTRG_PWM14CEN_Msk      (1ul << EADC_SMPTRG_PWM14CEN_Pos)     /*!< EADC_T::SMPTRG: PWM14CEN Mask */

#define EADC_SMPTRG_PWM14PEN_Pos      22                                    /*!< EADC_T::SMPTRG: PWM14PEN Position */
#define EADC_SMPTRG_PWM14PEN_Msk      (1ul << EADC_SMPTRG_PWM14PEN_Pos)     /*!< EADC_T::SMPTRG: PWM14PEN Mask */

#define EADC_SMPTRG_PWM14FEN_Pos      21                                    /*!< EADC_T::SMPTRG: PWM14FEN Position */
#define EADC_SMPTRG_PWM14FEN_Msk      (1ul << EADC_SMPTRG_PWM14FEN_Pos)     /*!< EADC_T::SMPTRG: PWM14FEN Mask */

#define EADC_SMPTRG_PWM14REN_Pos      20                                    /*!< EADC_T::SMPTRG: PWM14REN Position */
#define EADC_SMPTRG_PWM14REN_Msk      (1ul << EADC_SMPTRG_PWM14REN_Pos)     /*!< EADC_T::SMPTRG: PWM14REN Mask */

#define EADC_SMPTRG_PWM12CEN_Pos      19                                    /*!< EADC_T::SMPTRG: PWM12CEN Position */
#define EADC_SMPTRG_PWM12CEN_Msk      (1ul << EADC_SMPTRG_PWM12CEN_Pos)     /*!< EADC_T::SMPTRG: PWM12CEN Mask */

#define EADC_SMPTRG_PWM12PEN_Pos      18                                    /*!< EADC_T::SMPTRG: PWM12PEN Position */
#define EADC_SMPTRG_PWM12PEN_Msk      (1ul << EADC_SMPTRG_PWM12PEN_Pos)     /*!< EADC_T::SMPTRG: PWM12PEN Mask */

#define EADC_SMPTRG_PWM12FEN_Pos      17                                    /*!< EADC_T::SMPTRG: PWM12FEN Position */
#define EADC_SMPTRG_PWM12FEN_Msk      (1ul << EADC_SMPTRG_PWM12FEN_Pos)     /*!< EADC_T::SMPTRG: PWM12FEN Mask */

#define EADC_SMPTRG_PWM12REN_Pos      16                                    /*!< EADC_T::SMPTRG: PWM12REN Position */
#define EADC_SMPTRG_PWM12REN_Msk      (1ul << EADC_SMPTRG_PWM12REN_Pos)     /*!< EADC_T::SMPTRG: PWM12REN Mask */

#define EADC_SMPTRG_PWM10CEN_Pos      15                                    /*!< EADC_T::SMPTRG: PWM10CEN Position */
#define EADC_SMPTRG_PWM10CEN_Msk      (1ul << EADC_SMPTRG_PWM10CEN_Pos)     /*!< EADC_T::SMPTRG: PWM10CEN Mask */

#define EADC_SMPTRG_PWM10PEN_Pos      14                                    /*!< EADC_T::SMPTRG: PWM10PEN Position */
#define EADC_SMPTRG_PWM10PEN_Msk      (1ul << EADC_SMPTRG_PWM10PEN_Pos)     /*!< EADC_T::SMPTRG: PWM10PEN Mask */

#define EADC_SMPTRG_PWM10FEN_Pos      13                                    /*!< EADC_T::SMPTRG: PWM10FEN Position */
#define EADC_SMPTRG_PWM10FEN_Msk      (1ul << EADC_SMPTRG_PWM10FEN_Pos)     /*!< EADC_T::SMPTRG: PWM10FEN Mask */

#define EADC_SMPTRG_PWM10REN_Pos      12                                    /*!< EADC_T::SMPTRG: PWM10REN Position */
#define EADC_SMPTRG_PWM10REN_Msk      (1ul << EADC_SMPTRG_PWM10REN_Pos)     /*!< EADC_T::SMPTRG: PWM10REN Mask */

#define EADC_SMPTRG_PWM04CEN_Pos      11                                    /*!< EADC_T::SMPTRG: PWM04CEN Position */
#define EADC_SMPTRG_PWM04CEN_Msk      (1ul << EADC_SMPTRG_PWM04CEN_Pos)     /*!< EADC_T::SMPTRG: PWM04CEN Mask */

#define EADC_SMPTRG_PWM04PEN_Pos      10                                    /*!< EADC_T::SMPTRG: PWM04PEN Position */
#define EADC_SMPTRG_PWM04PEN_Msk      (1ul << EADC_SMPTRG_PWM04PEN_Pos)     /*!< EADC_T::SMPTRG: PWM04PEN Mask */

#define EADC_SMPTRG_PWM04FEN_Pos      9                                     /*!< EADC_T::SMPTRG: PWM04FEN Position */
#define EADC_SMPTRG_PWM04FEN_Msk      (1ul << EADC_SMPTRG_PWM04FEN_Pos)     /*!< EADC_T::SMPTRG: PWM04FEN Mask */

#define EADC_SMPTRG_PWM04REN_Pos      8                                     /*!< EADC_T::SMPTRG: PWM04REN Position */
#define EADC_SMPTRG_PWM04REN_Msk      (1ul << EADC_SMPTRG_PWM04REN_Pos)     /*!< EADC_T::SMPTRG: PWM04REN Mask */

#define EADC_SMPTRG_PWM02CEN_Pos      7                                     /*!< EADC_T::SMPTRG: PWM02CEN Position */
#define EADC_SMPTRG_PWM02CEN_Msk      (1ul << EADC_SMPTRG_PWM02CEN_Pos)     /*!< EADC_T::SMPTRG: PWM02CEN Mask */

#define EADC_SMPTRG_PWM02PEN_Pos      6                                     /*!< EADC_T::SMPTRG: PWM02PEN Position */
#define EADC_SMPTRG_PWM02PEN_Msk      (1ul << EADC_SMPTRG_PWM02PEN_Pos)     /*!< EADC_T::SMPTRG: PWM02PEN Mask */

#define EADC_SMPTRG_PWM02FEN_Pos      5                                     /*!< EADC_T::SMPTRG: PWM02FEN Position */
#define EADC_SMPTRG_PWM02FEN_Msk      (1ul << EADC_SMPTRG_PWM02FEN_Pos)     /*!< EADC_T::SMPTRG: PWM02FEN Mask */

#define EADC_SMPTRG_PWM02REN_Pos      4                                     /*!< EADC_T::SMPTRG: PWM02REN Position */
#define EADC_SMPTRG_PWM02REN_Msk      (1ul << EADC_SMPTRG_PWM02REN_Pos)     /*!< EADC_T::SMPTRG: PWM02REN Mask */

#define EADC_SMPTRG_PWM00CEN_Pos      3                                     /*!< EADC_T::SMPTRG: PWM00CEN Position */
#define EADC_SMPTRG_PWM00CEN_Msk      (1ul << EADC_SMPTRG_PWM00CEN_Pos)     /*!< EADC_T::SMPTRG: PWM00CEN Mask */

#define EADC_SMPTRG_PWM00PEN_Pos      2                                     /*!< EADC_T::SMPTRG: PWM00PEN Position */
#define EADC_SMPTRG_PWM00PEN_Msk      (1ul << EADC_SMPTRG_PWM00PEN_Pos)     /*!< EADC_T::SMPTRG: PWM00PEN Mask */

#define EADC_SMPTRG_PWM00FEN_Pos      1                                     /*!< EADC_T::SMPTRG: PWM00FEN Position */
#define EADC_SMPTRG_PWM00FEN_Msk      (1ul << EADC_SMPTRG_PWM00FEN_Pos)     /*!< EADC_T::SMPTRG: PWM00FEN Mask */

#define EADC_SMPTRG_PWM00REN_Pos      0                                     /*!< EADC_T::SMPTRG: PWM00REN Position */
#define EADC_SMPTRG_PWM00REN_Msk      (1ul << EADC_SMPTRG_PWM00REN_Pos)     /*!< EADC_T::SMPTRG: PWM00REN Mask */

/* SMPTRGA Bit Field Definitions */
#define EADC_SMPTRGA_PWM21CEN_Pos     31                                    /*!< EADC_T::SMPTRGA: PWM21CEN Position */
#define EADC_SMPTRGA_PWM21CEN_Msk     (1ul << EADC_SMPTRGA_PWM21CEN_Pos)    /*!< EADC_T::SMPTRGA: PWM21CEN Mask */

#define EADC_SMPTRGA_PWM21PEN_Pos     30                                    /*!< EADC_T::SMPTRGA: PWM21PEN Position */
#define EADC_SMPTRGA_PWM21PEN_Msk     (1ul << EADC_SMPTRGA_PWM21PEN_Pos)    /*!< EADC_T::SMPTRGA: PWM21PEN Mask */

#define EADC_SMPTRGA_PWM21FEN_Pos     29                                    /*!< EADC_T::SMPTRGA: PWM21FEN Position */
#define EADC_SMPTRGA_PWM21FEN_Msk     (1ul << EADC_SMPTRGA_PWM21FEN_Pos)    /*!< EADC_T::SMPTRGA: PWM21FEN Mask */

#define EADC_SMPTRGA_PWM21REN_Pos     28                                    /*!< EADC_T::SMPTRGA: PWM21REN Position */
#define EADC_SMPTRGA_PWM21REN_Msk     (1ul << EADC_SMPTRGA_PWM21REN_Pos)    /*!< EADC_T::SMPTRGA: PWM21REN Mask */

#define EADC_SMPTRGA_PWM20CEN_Pos     27                                    /*!< EADC_T::SMPTRGA: PWM20CEN Position */
#define EADC_SMPTRGA_PWM20CEN_Msk     (1ul << EADC_SMPTRGA_PWM20CEN_Pos)    /*!< EADC_T::SMPTRGA: PWM20CEN Mask */

#define EADC_SMPTRGA_PWM20PEN_Pos     26                                    /*!< EADC_T::SMPTRGA: PWM20PEN Position */
#define EADC_SMPTRGA_PWM20PEN_Msk     (1ul << EADC_SMPTRGA_PWM20PEN_Pos)    /*!< EADC_T::SMPTRGA: PWM20PEN Mask */

#define EADC_SMPTRGA_PWM20FEN_Pos     25                                    /*!< EADC_T::SMPTRGA: PWM20FEN Position */
#define EADC_SMPTRGA_PWM20FEN_Msk     (1ul << EADC_SMPTRGA_PWM20FEN_Pos)    /*!< EADC_T::SMPTRGA: PWM20FEN Mask */

#define EADC_SMPTRGA_PWM20REN_Pos     24                                    /*!< EADC_T::SMPTRGA: PWM20REN Position */
#define EADC_SMPTRGA_PWM20REN_Msk     (1ul << EADC_SMPTRGA_PWM20REN_Pos)    /*!< EADC_T::SMPTRGA: PWM20REN Mask */

#define EADC_SMPTRGA_PWM14CEN_Pos     23                                    /*!< EADC_T::SMPTRGA: PWM14CEN Position */
#define EADC_SMPTRGA_PWM14CEN_Msk     (1ul << EADC_SMPTRGA_PWM14CEN_Pos)    /*!< EADC_T::SMPTRGA: PWM14CEN Mask */

#define EADC_SMPTRGA_PWM14PEN_Pos     22                                    /*!< EADC_T::SMPTRGA: PWM14PEN Position */
#define EADC_SMPTRGA_PWM14PEN_Msk     (1ul << EADC_SMPTRGA_PWM14PEN_Pos)    /*!< EADC_T::SMPTRGA: PWM14PEN Mask */

#define EADC_SMPTRGA_PWM14FEN_Pos     21                                    /*!< EADC_T::SMPTRGA: PWM14FEN Position */
#define EADC_SMPTRGA_PWM14FEN_Msk     (1ul << EADC_SMPTRGA_PWM14FEN_Pos)    /*!< EADC_T::SMPTRGA: PWM14FEN Mask */

#define EADC_SMPTRGA_PWM14REN_Pos     20                                    /*!< EADC_T::SMPTRGA: PWM14REN Position */
#define EADC_SMPTRGA_PWM14REN_Msk     (1ul << EADC_SMPTRGA_PWM14REN_Pos)    /*!< EADC_T::SMPTRGA: PWM14REN Mask */

#define EADC_SMPTRGA_PWM12CEN_Pos     19                                    /*!< EADC_T::SMPTRGA: PWM12CEN Position */
#define EADC_SMPTRGA_PWM12CEN_Msk     (1ul << EADC_SMPTRGA_PWM12CEN_Pos)    /*!< EADC_T::SMPTRGA: PWM12CEN Mask */

#define EADC_SMPTRGA_PWM12PEN_Pos     18                                    /*!< EADC_T::SMPTRGA: PWM12PEN Position */
#define EADC_SMPTRGA_PWM12PEN_Msk     (1ul << EADC_SMPTRGA_PWM12PEN_Pos)    /*!< EADC_T::SMPTRGA: PWM12PEN Mask */

#define EADC_SMPTRGA_PWM12FEN_Pos     17                                    /*!< EADC_T::SMPTRGA: PWM12FEN Position */
#define EADC_SMPTRGA_PWM12FEN_Msk     (1ul << EADC_SMPTRGA_PWM12FEN_Pos)    /*!< EADC_T::SMPTRGA: PWM12FEN Mask */

#define EADC_SMPTRGA_PWM12REN_Pos     16                                    /*!< EADC_T::SMPTRGA: PWM12REN Position */
#define EADC_SMPTRGA_PWM12REN_Msk     (1ul << EADC_SMPTRGA_PWM12REN_Pos)    /*!< EADC_T::SMPTRGA: PWM12REN Mask */

#define EADC_SMPTRGA_PWM10CEN_Pos     15                                    /*!< EADC_T::SMPTRGA: PWM10CEN Position */
#define EADC_SMPTRGA_PWM10CEN_Msk     (1ul << EADC_SMPTRGA_PWM10CEN_Pos)    /*!< EADC_T::SMPTRGA: PWM10CEN Mask */

#define EADC_SMPTRGA_PWM10PEN_Pos     14                                    /*!< EADC_T::SMPTRGA: PWM10PEN Position */
#define EADC_SMPTRGA_PWM10PEN_Msk     (1ul << EADC_SMPTRGA_PWM10PEN_Pos)    /*!< EADC_T::SMPTRGA: PWM10PEN Mask */

#define EADC_SMPTRGA_PWM10FEN_Pos     13                                    /*!< EADC_T::SMPTRGA: PWM10FEN Position */
#define EADC_SMPTRGA_PWM10FEN_Msk     (1ul << EADC_SMPTRGA_PWM10FEN_Pos)    /*!< EADC_T::SMPTRGA: PWM10FEN Mask */

#define EADC_SMPTRGA_PWM10REN_Pos     12                                    /*!< EADC_T::SMPTRGA: PWM10REN Position */
#define EADC_SMPTRGA_PWM10REN_Msk     (1ul << EADC_SMPTRGA_PWM10REN_Pos)    /*!< EADC_T::SMPTRGA: PWM10REN Mask */

#define EADC_SMPTRGA_PWM04CEN_Pos     11                                    /*!< EADC_T::SMPTRGA: PWM04CEN Position */
#define EADC_SMPTRGA_PWM04CEN_Msk     (1ul << EADC_SMPTRGA_PWM04CEN_Pos)    /*!< EADC_T::SMPTRGA: PWM04CEN Mask */

#define EADC_SMPTRGA_PWM04PEN_Pos     10                                    /*!< EADC_T::SMPTRGA: PWM04PEN Position */
#define EADC_SMPTRGA_PWM04PEN_Msk     (1ul << EADC_SMPTRGA_PWM04PEN_Pos)    /*!< EADC_T::SMPTRGA: PWM04PEN Mask */

#define EADC_SMPTRGA_PWM04FEN_Pos     9                                     /*!< EADC_T::SMPTRGA: PWM04FEN Position */
#define EADC_SMPTRGA_PWM04FEN_Msk     (1ul << EADC_SMPTRGA_PWM04FEN_Pos)    /*!< EADC_T::SMPTRGA: PWM04FEN Mask */

#define EADC_SMPTRGA_PWM04REN_Pos     8                                     /*!< EADC_T::SMPTRGA: PWM04REN Position */
#define EADC_SMPTRGA_PWM04REN_Msk     (1ul << EADC_SMPTRGA_PWM04REN_Pos)    /*!< EADC_T::SMPTRGA: PWM04REN Mask */

#define EADC_SMPTRGA_PWM02CEN_Pos     7                                     /*!< EADC_T::SMPTRGA: PWM02CEN Position */
#define EADC_SMPTRGA_PWM02CEN_Msk     (1ul << EADC_SMPTRGA_PWM02CEN_Pos)    /*!< EADC_T::SMPTRGA: PWM02CEN Mask */

#define EADC_SMPTRGA_PWM02PEN_Pos     6                                     /*!< EADC_T::SMPTRGA: PWM02PEN Position */
#define EADC_SMPTRGA_PWM02PEN_Msk     (1ul << EADC_SMPTRGA_PWM02PEN_Pos)    /*!< EADC_T::SMPTRGA: PWM02PEN Mask */

#define EADC_SMPTRGA_PWM02FEN_Pos     5                                     /*!< EADC_T::SMPTRGA: PWM02FEN Position */
#define EADC_SMPTRGA_PWM02FEN_Msk     (1ul << EADC_SMPTRGA_PWM02FEN_Pos)    /*!< EADC_T::SMPTRGA: PWM02FEN Mask */

#define EADC_SMPTRGA_PWM02REN_Pos     4                                     /*!< EADC_T::SMPTRGA: PWM02REN Position */
#define EADC_SMPTRGA_PWM02REN_Msk     (1ul << EADC_SMPTRGA_PWM02REN_Pos)    /*!< EADC_T::SMPTRGA: PWM02REN Mask */

#define EADC_SMPTRGA_PWM00CEN_Pos     3                                     /*!< EADC_T::SMPTRGA: PWM00CEN Position */
#define EADC_SMPTRGA_PWM00CEN_Msk     (1ul << EADC_SMPTRGA_PWM00CEN_Pos)    /*!< EADC_T::SMPTRGA: PWM00CEN Mask */

#define EADC_SMPTRGA_PWM00PEN_Pos     2                                     /*!< EADC_T::SMPTRGA: PWM00PEN Position */
#define EADC_SMPTRGA_PWM00PEN_Msk     (1ul << EADC_SMPTRGA_PWM00PEN_Pos)    /*!< EADC_T::SMPTRGA: PWM00PEN Mask */

#define EADC_SMPTRGA_PWM00FEN_Pos     1                                     /*!< EADC_T::SMPTRGA: PWM00FEN Position */
#define EADC_SMPTRGA_PWM00FEN_Msk     (1ul << EADC_SMPTRGA_PWM00FEN_Pos)    /*!< EADC_T::SMPTRGA: PWM00FEN Mask */

#define EADC_SMPTRGA_PWM00REN_Pos     0                                     /*!< EADC_T::SMPTRGA: PWM00REN Position */
#define EADC_SMPTRGA_PWM00REN_Msk     (1ul << EADC_SMPTRGA_PWM00REN_Pos)    /*!< EADC_T::SMPTRGA: PWM00REN Mask */

/* SMPTRGB Bit Field Definitions */
#define EADC_SMPTRGB_PWM21CEN_Pos     31                                    /*!< EADC_T::SMPTRGB: PWM21CEN Position */
#define EADC_SMPTRGB_PWM21CEN_Msk     (1ul << EADC_SMPTRGB_PWM21CEN_Pos)    /*!< EADC_T::SMPTRGB: PWM21CEN Mask */

#define EADC_SMPTRGB_PWM21PEN_Pos     30                                    /*!< EADC_T::SMPTRGB: PWM21PEN Position */
#define EADC_SMPTRGB_PWM21PEN_Msk     (1ul << EADC_SMPTRGB_PWM21PEN_Pos)    /*!< EADC_T::SMPTRGB: PWM21PEN Mask */

#define EADC_SMPTRGB_PWM21FEN_Pos     29                                    /*!< EADC_T::SMPTRGB: PWM21FEN Position */
#define EADC_SMPTRGB_PWM21FEN_Msk     (1ul << EADC_SMPTRGB_PWM21FEN_Pos)    /*!< EADC_T::SMPTRGB: PWM21FEN Mask */

#define EADC_SMPTRGB_PWM21REN_Pos     28                                    /*!< EADC_T::SMPTRGB: PWM21REN Position */
#define EADC_SMPTRGB_PWM21REN_Msk     (1ul << EADC_SMPTRGB_PWM21REN_Pos)    /*!< EADC_T::SMPTRGB: PWM21REN Mask */

#define EADC_SMPTRGB_PWM20CEN_Pos     27                                    /*!< EADC_T::SMPTRGB: PWM20CEN Position */
#define EADC_SMPTRGB_PWM20CEN_Msk     (1ul << EADC_SMPTRGB_PWM20CEN_Pos)    /*!< EADC_T::SMPTRGB: PWM20CEN Mask */

#define EADC_SMPTRGB_PWM20PEN_Pos     26                                    /*!< EADC_T::SMPTRGB: PWM20PEN Position */
#define EADC_SMPTRGB_PWM20PEN_Msk     (1ul << EADC_SMPTRGB_PWM20PEN_Pos)    /*!< EADC_T::SMPTRGB: PWM20PEN Mask */

#define EADC_SMPTRGB_PWM20FEN_Pos     25                                    /*!< EADC_T::SMPTRGB: PWM20FEN Position */
#define EADC_SMPTRGB_PWM20FEN_Msk     (1ul << EADC_SMPTRGB_PWM20FEN_Pos)    /*!< EADC_T::SMPTRGB: PWM20FEN Mask */

#define EADC_SMPTRGB_PWM20REN_Pos     24                                    /*!< EADC_T::SMPTRGB: PWM20REN Position */
#define EADC_SMPTRGB_PWM20REN_Msk     (1ul << EADC_SMPTRGB_PWM20REN_Pos)    /*!< EADC_T::SMPTRGB: PWM20REN Mask */

#define EADC_SMPTRGB_PWM14CEN_Pos     23                                    /*!< EADC_T::SMPTRGB: PWM14CEN Position */
#define EADC_SMPTRGB_PWM14CEN_Msk     (1ul << EADC_SMPTRGB_PWM14CEN_Pos)    /*!< EADC_T::SMPTRGB: PWM14CEN Mask */

#define EADC_SMPTRGB_PWM14PEN_Pos     22                                    /*!< EADC_T::SMPTRGB: PWM14PEN Position */
#define EADC_SMPTRGB_PWM14PEN_Msk     (1ul << EADC_SMPTRGB_PWM14PEN_Pos)    /*!< EADC_T::SMPTRGB: PWM14PEN Mask */

#define EADC_SMPTRGB_PWM14FEN_Pos     21                                    /*!< EADC_T::SMPTRGB: PWM14FEN Position */
#define EADC_SMPTRGB_PWM14FEN_Msk     (1ul << EADC_SMPTRGB_PWM14FEN_Pos)    /*!< EADC_T::SMPTRGB: PWM14FEN Mask */

#define EADC_SMPTRGB_PWM14REN_Pos     20                                    /*!< EADC_T::SMPTRGB: PWM14REN Position */
#define EADC_SMPTRGB_PWM14REN_Msk     (1ul << EADC_SMPTRGB_PWM14REN_Pos)    /*!< EADC_T::SMPTRGB: PWM14REN Mask */

#define EADC_SMPTRGB_PWM12CEN_Pos     19                                    /*!< EADC_T::SMPTRGB: PWM12CEN Position */
#define EADC_SMPTRGB_PWM12CEN_Msk     (1ul << EADC_SMPTRGB_PWM12CEN_Pos)    /*!< EADC_T::SMPTRGB: PWM12CEN Mask */

#define EADC_SMPTRGB_PWM12PEN_Pos     18                                    /*!< EADC_T::SMPTRGB: PWM12PEN Position */
#define EADC_SMPTRGB_PWM12PEN_Msk     (1ul << EADC_SMPTRGB_PWM12PEN_Pos)    /*!< EADC_T::SMPTRGB: PWM12PEN Mask */

#define EADC_SMPTRGB_PWM12FEN_Pos     17                                    /*!< EADC_T::SMPTRGB: PWM12FEN Position */
#define EADC_SMPTRGB_PWM12FEN_Msk     (1ul << EADC_SMPTRGB_PWM12FEN_Pos)    /*!< EADC_T::SMPTRGB: PWM12FEN Mask */

#define EADC_SMPTRGB_PWM12REN_Pos     16                                    /*!< EADC_T::SMPTRGB: PWM12REN Position */
#define EADC_SMPTRGB_PWM12REN_Msk     (1ul << EADC_SMPTRGB_PWM12REN_Pos)    /*!< EADC_T::SMPTRGB: PWM12REN Mask */

#define EADC_SMPTRGB_PWM10CEN_Pos     15                                    /*!< EADC_T::SMPTRGB: PWM10CEN Position */
#define EADC_SMPTRGB_PWM10CEN_Msk     (1ul << EADC_SMPTRGB_PWM10CEN_Pos)    /*!< EADC_T::SMPTRGB: PWM10CEN Mask */

#define EADC_SMPTRGB_PWM10PEN_Pos     14                                    /*!< EADC_T::SMPTRGB: PWM10PEN Position */
#define EADC_SMPTRGB_PWM10PEN_Msk     (1ul << EADC_SMPTRGB_PWM10PEN_Pos)    /*!< EADC_T::SMPTRGB: PWM10PEN Mask */

#define EADC_SMPTRGB_PWM10FEN_Pos     13                                    /*!< EADC_T::SMPTRGB: PWM10FEN Position */
#define EADC_SMPTRGB_PWM10FEN_Msk     (1ul << EADC_SMPTRGB_PWM10FEN_Pos)    /*!< EADC_T::SMPTRGB: PWM10FEN Mask */

#define EADC_SMPTRGB_PWM10REN_Pos     12                                    /*!< EADC_T::SMPTRGB: PWM10REN Position */
#define EADC_SMPTRGB_PWM10REN_Msk     (1ul << EADC_SMPTRGB_PWM10REN_Pos)    /*!< EADC_T::SMPTRGB: PWM10REN Mask */

#define EADC_SMPTRGB_PWM04CEN_Pos     11                                    /*!< EADC_T::SMPTRGB: PWM04CEN Position */
#define EADC_SMPTRGB_PWM04CEN_Msk     (1ul << EADC_SMPTRGB_PWM04CEN_Pos)    /*!< EADC_T::SMPTRGB: PWM04CEN Mask */

#define EADC_SMPTRGB_PWM04PEN_Pos     10                                    /*!< EADC_T::SMPTRGB: PWM04PEN Position */
#define EADC_SMPTRGB_PWM04PEN_Msk     (1ul << EADC_SMPTRGB_PWM04PEN_Pos)    /*!< EADC_T::SMPTRGB: PWM04PEN Mask */

#define EADC_SMPTRGB_PWM04FEN_Pos     9                                     /*!< EADC_T::SMPTRGB: PWM04FEN Position */
#define EADC_SMPTRGB_PWM04FEN_Msk     (1ul << EADC_SMPTRGB_PWM04FEN_Pos)    /*!< EADC_T::SMPTRGB: PWM04FEN Mask */

#define EADC_SMPTRGB_PWM04REN_Pos     8                                     /*!< EADC_T::SMPTRGB: PWM04REN Position */
#define EADC_SMPTRGB_PWM04REN_Msk     (1ul << EADC_SMPTRGB_PWM04REN_Pos)    /*!< EADC_T::SMPTRGB: PWM04REN Mask */

#define EADC_SMPTRGB_PWM02CEN_Pos     7                                     /*!< EADC_T::SMPTRGB: PWM02CEN Position */
#define EADC_SMPTRGB_PWM02CEN_Msk     (1ul << EADC_SMPTRGB_PWM02CEN_Pos)    /*!< EADC_T::SMPTRGB: PWM02CEN Mask */

#define EADC_SMPTRGB_PWM02PEN_Pos     6                                     /*!< EADC_T::SMPTRGB: PWM02PEN Position */
#define EADC_SMPTRGB_PWM02PEN_Msk     (1ul << EADC_SMPTRGB_PWM02PEN_Pos)    /*!< EADC_T::SMPTRGB: PWM02PEN Mask */

#define EADC_SMPTRGB_PWM02FEN_Pos     5                                     /*!< EADC_T::SMPTRGB: PWM02FEN Position */
#define EADC_SMPTRGB_PWM02FEN_Msk     (1ul << EADC_SMPTRGB_PWM02FEN_Pos)    /*!< EADC_T::SMPTRGB: PWM02FEN Mask */

#define EADC_SMPTRGB_PWM02REN_Pos     4                                     /*!< EADC_T::SMPTRGB: PWM02REN Position */
#define EADC_SMPTRGB_PWM02REN_Msk     (1ul << EADC_SMPTRGB_PWM02REN_Pos)    /*!< EADC_T::SMPTRGB: PWM02REN Mask */

#define EADC_SMPTRGB_PWM00CEN_Pos     3                                     /*!< EADC_T::SMPTRGB: PWM00CEN Position */
#define EADC_SMPTRGB_PWM00CEN_Msk     (1ul << EADC_SMPTRGB_PWM00CEN_Pos)    /*!< EADC_T::SMPTRGB: PWM00CEN Mask */

#define EADC_SMPTRGB_PWM00PEN_Pos     2                                     /*!< EADC_T::SMPTRGB: PWM00PEN Position */
#define EADC_SMPTRGB_PWM00PEN_Msk     (1ul << EADC_SMPTRGB_PWM00PEN_Pos)    /*!< EADC_T::SMPTRGB: PWM00PEN Mask */

#define EADC_SMPTRGB_PWM00FEN_Pos     1                                     /*!< EADC_T::SMPTRGB: PWM00FEN Position */
#define EADC_SMPTRGB_PWM00FEN_Msk     (1ul << EADC_SMPTRGB_PWM00FEN_Pos)    /*!< EADC_T::SMPTRGB: PWM00FEN Mask */

#define EADC_SMPTRGB_PWM00REN_Pos     0                                     /*!< EADC_T::SMPTRGB: PWM00REN Position */
#define EADC_SMPTRGB_PWM00REN_Msk     (1ul << EADC_SMPTRGB_PWM00REN_Pos)    /*!< EADC_T::SMPTRGB: PWM00REN Mask */

/*@}*/ /* end of group EADC_CONST */
/*@}*/ /* end of group EADC */



/*---------------------- Enhanced Pulse Width Modulation Controller -------------------------*/
/**
    @addtogroup Enhanced PWM Pulse Width Modulation Controller (EPWM)
    Memory Mapped Structure for EPWM Controller
@{ */

typedef struct
{


    /**
     * @var EPWM_T::PWMCON
     * Offset: 0x00  EPWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits     |Field     |Descriptions
     * | :----:  | :----:   | :---- |
     * |[1:0]    |PWMMOD    |PWM Mode Selection
     * |         |          |00 = Independent mode.
     * |         |          |01 = Pair/Complementary mode.
     * |         |          |10 = Synchronized mode.
     * |         |          |11 = Reserved.
     * |[3:2]    |PWMDIV    |PWM Clock Pre-Divider Selection
     * |         |          |00 = PWM clock = EPWMx_CLK.
     * |         |          |01 = PWM clock = EPWMx_CLK/2.
     * |         |          |10 = PWM clock = EPWMx_CLK/4.
     * |         |          |11 = PWM clock = EPWMx_CLK/16.
     * |[4]      |PWMI_EN   |Enable PWM Interrupt
     * |         |          |0 = Flag PWMF Disabled to trigger PWM interrupt.
     * |         |          |1 = Flag PWMF Enabled to trigger PWM interrupt.
     * |[5]      |BRKI_EN   |Enable Brake0 And Brak1 Interrupt
     * |         |          |0 = Flags BFK0 and BFK1 Disabled to trigger PWM interrupt.
     * |         |          |1 = Flags BKF0 and BKF1 Enabled to trigger PWM interrupt.
     * |[6]      |LOAD      |Reload PWM Period Registers (PWMP) And PWM Duty Registers (PWM0~3) Control Bit
     * |         |          |0 = No action if written with 0. The value of PWM period register (PWMP) and PWM duty
     * |         |          |registers (PWMn0~PWMn3) are not loaded to PWM counter and Comparator registers.
     * |         |          |1 = Hardware will update the value of PWM period register (PWMP) and PWM duty
     * |         |          |registers (PWMn0~PWMn3) to PWM Counter and Comparator register at the time of PWM
     * |         |          |Counter matches PWMP in Edge- and Center-aligned modes or at the time of PWM
     * |         |          |Counter down counts with underflow in Center-aligned mode.
     * |         |          |Note1: n=0-1 for PWM unit0-1.
     * |         |          |Note2: This bit is written by software, cleared by hardware, and always read as 0.
     * |[7]      |PWMRUN    |Start PWMRUN Control Bit
     * |         |          |0 = The PWM stops running.
     * |         |          |1 = The PWM counter starts running.
     * |[8]      |INT_TYPE  |PWM Interrupt Type Selection Bit
     * |         |          |0 = PWMF will be set if PWM counter underflow.
     * |         |          |1 = PWMF will be set if PWM counter matches PWMP register.
     * |         |          |Note: This bit is effective when PWM is in Center-aligned mode only.
     * |[9]      |PWMINV    |Inverse PWM Comparator Output
     * |         |          |When PWMINV is set to high the PWM comparator output signals will be inverted,
     * |         |          |therefore the PWM Duty (in percentage) is changed to (1-Duty) before PWMINV is set to
     * |         |          |high.
     * |         |          |0 = Not inverse PWM comparator output.
     * |         |          |1 = Inverse PWM comparator output.
     * |[11]     |CLRPWM    |Clear PWM Counter Control Bit
     * |         |          |1 = Clear 16-bit PWM counter to 000H.
     * |         |          |Note: It is automatically cleared by hardware.
     * |[12]     |PWMTYPE   |PWM Aligned Type Selection Bit
     * |         |          |0 = Edge-aligned type.
     * |         |          |1 = Center-aligned type.
     * |[13]     |GRP       |Group Bit
     * |         |          |0 = The signals timing of PWM0, PWM2 and PWM4 are independent.
     * |         |          |1 = Unify the signals timing of PWM0, PWM2 and PWM4 in the same phase which is
     * |         |          |controlled by PWM0.
     * |[14]     |INVBKP0   |Inverse BKP0 State
     * |         |          |0 = The state of pin BKPx0 is passed to the negative edge detector.
     * |         |          |1 = The inverted state of pin BKPx0 is passed to the negative edge detector.
     * |[15]     |INVBKP1   |Inverse BKP1 State
     * |         |          |0 = The state of pin BKPx1 is passed to the negative edge detector.
     * |         |          |1 = The inverted state of pin BKPx1 is passed to the negative edge detector.
     * |[16]     |BK0_EN    |BKPx0 Pin Trigger Brake Function 0 Enable
     * |         |          |0 = PWMx Brake Function 0 Disabled.
     * |         |          |1 = PWMx Brake Function 0 Enabled.
     * |         |          |Note: x = 0~1 for PWM unit0~1.
     * |[17]     |BK1_EN    |BKPx1 Pin Trigger Brake Function 1 Enable
     * |         |          |0 = PWMx Brake Function 1 Disabled.
     * |         |          |1 = PWMx Brake Function 1 Enabled.
     * |         |          |Note: x=0~1 for PWM unit0~1.
     * |[19:18]  |BK1SEL    |Brake Function 1 Source Selection
     * |         |          |00 = From external pin BKPx1 (x=0~1 for unit0~1).
     * |         |          |01 = From analog comparator 0 output (CPO0).
     * |         |          |10 = From analog comparator 1 output (CPO1).
     * |         |          |11 = From analog comparator 2 output (CPO2).
     * |[21:20]  |BK0FILT   |Brake 0 (BKPx0 Pin) Edge Detector Filter Clock Selection
     * |         |          |00 = Filter clock = HCLK.
     * |         |          |01 = Filter clock = HCLK/2.
     * |         |          |10 = Filter clock = HCLK/4.
     * |         |          |11 = Filter clock = HCLK/16.
     * |[23:22]  |BK1FILT   |Brake 1 (BKPx1 Pin) Edge Detector Filter Clock Selection
     * |         |          |00 = Filter clock = HCLK.
     * |         |          |01 = Filter clock = HCLK/2.
     * |         |          |10 = Filter clock = HCLK/4.
     * |         |          |11 = Filter clock = HCLK/16.
     * |[24]     |CPO0BK_EN |Enable CPO0 Digital Output As Brake0 Source
     * |         |          |0 = CPO0 as one brake source in Brake 0 Disabled.
     * |         |          |1 = CPO0 as one brake source in Brake 0 Enabled.
     * |[25]     |CPO1BK_EN |Enable CPO1 Digital Output As Brake 0 Source
     * |         |          |0 = CPO1 as one brake source in Brake 0 Disabled.
     * |         |          |1 = CPO1 as one brake source in Brake 0 Enabled.
     * |[26]     |CPO2BK_EN |CPO2 Digital Output As Brake 0 Source Enable
     * |         |          |0 = CPO2 as one brake source in Brake 0 Disabled.
     * |         |          |1 = CPO2 as one brake source in Brake 0 Enabled.
     * |[27]     |LVDBK_EN  |Low-Level Detection Trigger PWM Brake Function 1 Enable
     * |         |          |0 = Brake Function 1 triggered by Low-level detection Disabled.
     * |         |          |1 = Brake Function 1 triggered by Low-level detection Enabled.
     * |[28]     |BK0NF_DIS |PWM Brake 0 Noise Filter Disable
     * |         |          |0 = Noise filter of PWM Brake 0 Enabled.
     * |         |          |1 = Noise filter of PWM Brake 0 Disabled.
     * |[29]     |BK1NF_DIS |PWM Brake 1 Noise Filter Disable
     * |         |          |0 = Noise filter of PWM Brake 1 Enabled.
     * |         |          |1 = Noise filter of PWM Brake 1 Disabled.
     * |[31]     |CLDMD     |Center Reload Mode Enable
     * |         |          |0 = EPWM reload duty register at the period point of PWM counter.
     * |         |          |1 = EPWM reload duty register at the center point of PWM counter.
     * |         |          |Note: This bit only works when EPWM operating in Center-aligned mode.
     * @var EPWM_T::PWMSTS
     * Offset: 0x04  EPWM Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits      |Field     |Descriptions
     * | :----:   | :----:   | :---- |
     * |[0]       |BKF0      |PWM Brake0 Flag
     * |          |          |0 = PWM Brake 0 is able to poll falling signal at BKP0 and has not recognized any one.
     * |          |          |1 = When PWM Brake 0 detects a falling signal at BKP0, this flag will be set to high.
     * |          |          |Note: This bit must be cleared by writing 1 to itself through software.
     * |[1]       |BKF1      |PWM Brake1 Flag
     * |          |          |0 = PWM Brake 1 is able to poll falling signal at BKP1 and has not recognized any one.
     * |          |          |1 = When PWM Brake 1 detects a falling signal at pin BKP1, this flag will be set to high.
     * |          |          |Note: This bit must be cleared by writing 1 to itself through software.
     * |[2]       |PWMF      |PWM Period Flag
     * |          |          |0 = The PWM Counter has not up counted to the value of PWMP or down counted with underflow.
     * |          |          |1 = Hardware will set this flag to high at the time of PWM Counter matches PWMP in Edge-
     * |          |          |and Center-aligned modes or at the time of PWM Counter down counts with underflow in
     * |          |          |Center-aligned mode.
     * |          |          |Note: This bit must be cleared by writing 1 to itself through software.
     * |[4]       |PWM0EF    |PWMx0 Edge Flag
     * |          |          |0 = PWMx0 not toggled.
     * |          |          |1 = Hardware will set this flag to high at the time of PWMx0 rising or falling. If EINT0_TYPE =
     * |          |          |0, this bit is set when PWMx0 falling is detected. If EINT0_TYPE = 1, this bit is set when
     * |          |          |PWMx0 rising is detected.
     * |          |          |Note: This bit must be cleared by writing 1 to itself through software.
     * |[5]       |PWM2EF    |PWMx2 Edge Flag
     * |          |          |0 = PWMx2 not toggled.
     * |          |          |1 = Hardware will set this flag to high at the time of PWMx2 rising or falling. If EINT2_TYPE =
     * |          |          |0, this bit is set when PWMx2 falling is detected. If EINT2_TYPE = 1, this bit is set when
     * |          |          |PWMx2 rising is detected.
     * |          |          |Note: This bit must be cleared by writing 1 to itself through software.
     * |[6]       |PWM4EF    |PWMx4 Edge Flag
     * |          |          |0 = PWMx4 not toggled.
     * |          |          |1 = Hardware will set this flag to high at the time of PWMx4 rising or falling. If EINT4_TYPE =
     * |          |          |0, this bit is set when PWMx4 falling is detected. If EINT4_TYPE = 1, this bit is set when
     * |          |          |PWMx4 rising is detected.
     * |          |          |Note: This bit must be cleared by writing 1 to itself through software.
     * |[8]       |BKLK0     |PWM Brake0 Locked
     * |          |          |0 = Brake 0 state is released.
     * |          |          |1 = When PWM Brake detects a falling signal at BKP0, this flag will be set to high to indicate
     * |          |          |the Brake0 state is locked.
     * |          |          |Note: This bit must be cleared by writing 1 to itself through software.
     * |[24]      |BK0STS    |Brake 0 Status (Read Only)
     * |          |          |0 = PWM had been out of Brake 0 state.
     * |          |          |1 = PWM is in Brake 0 state.
     * |[25]      |BK1STS    |Brake 1 Status (Read Only)
     * |          |          |0 = PWM had been out of Brake 1 state.
     * |          |          |1 = PWM is in Brake 1 state.
     * @var EPWM_T::PWMP
     * Offset: 0x08  EPWM Period Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits     |Field         |Descriptions
     * | :----:  | :----:       | :---- |
     * |[15:0]   |PWMP          |PWM Period Register
     * |         |              |Edge-aligned:
     * |         |              |Period = (PWMP + 1) * EPWMx_CLK period/pre-scalar.
     * |         |              |Duty = (Duty + 1) * EPWMx_CLK period /pre-scalar.
     * |         |              |Center-aligned:
     * |         |              |Period = (PWMP * 2) * EPWMx_CLK period/pre-scalar.
     * |         |              |Duty = (Duty * 2 + 1) * EPWMx_CLK period/pre-scalar.
     * @var EPWM_T::PWM0
     * Offset: 0x0C  EPWM PWM0 Duty Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PWM_Duty  |PWM Duty Register
     * |        |          |Edge-aligned:
     * |        |          |Period = (PWMP + 1) * EPWMx_CLK period/pre-scalar.
     * |        |          |Duty = (Duty + 1) * EPWMx_CLK period /pre-scalar.
     * |        |          |Center-aligned:
     * |        |          |Period = (PWMP * 2) * EPWMx_CLK period/pre-scalar.
     * |        |          |Duty = (Duty * 2 + 1) * EPWMx_CLK period/pre-scalar.
     * @var EPWM_T::PWM2
     * Offset: 0x10  EPWM PWM2 Duty Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PWM_Duty  |PWM Duty Register
     * |        |          |Edge-aligned:
     * |        |          |Period = (PWMP + 1) * EPWMx_CLK period/pre-scalar.
     * |        |          |Duty = (Duty + 1) * EPWMx_CLK period /pre-scalar.
     * |        |          |Center-aligned:
     * |        |          |Period = (PWMP * 2) * EPWMx_CLK period/pre-scalar.
     * |        |          |Duty = (Duty * 2 + 1) * EPWMx_CLK period/pre-scalar.
     * @var EPWM_T::PWM4
     * Offset: 0x14  EPWM PWM4 Duty Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PWM_Duty  |PWM Duty Register
     * |        |          |Edge-aligned:
     * |        |          |Period = (PWMP + 1) * EPWMx_CLK period/pre-scalar.
     * |        |          |Duty = (Duty + 1) * EPWMx_CLK period /pre-scalar.
     * |        |          |Center-aligned:
     * |        |          |Period = (PWMP * 2) * EPWMx_CLK period/pre-scalar.
     * |        |          |Duty = (Duty * 2 + 1) * EPWMx_CLK period/pre-scalar.
     * @var EPWM_T::PMSKE
     * Offset: 0x18  EPWM Mask Mode Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |PMSKE     |PWM Mask Enable Bit
     * |        |          |The PWM generator signal will be masked when this bit is enabled. The corresponding
     * |        |          |PWMn channel will be output with PMD.n data.
     * |        |          |0 = PWM generator signal is output to next stage.
     * |        |          |1 = PWM generator signal is masked and PMD.n is output to next stage.
     * |        |          |Note: n = 0~5.
     * @var EPWM_T::PMSKD
     * Offset: 0x1C  EPWM Mask Mode Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |PMSKD     |PWM Mask Data Bit
     * |        |          |This data bit control the state of PWMn output pin, if corresponding PME.n = 1.
     * |        |          |0 = Output logic low to PWMn.
     * |        |          |1 = Output logic high to PWMn.
     * |        |          |Note: n = 0~5.
     * @var EPWM_T::PDTC
     * Offset: 0x2C  EPWM Dead-time Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[10:0]  |DTCNT     |Dead-Time Counter
     * |        |          |The dead-time can be calculated according to the following formula:
     * |        |          |Dead-time = EPWMx_CLK * (DTCNT.[10:0]+1).
     * |[16]    |DTEN0     |Enable Dead-Time Insertion For PWMx Pair (PWM0, PWM1)
     * |        |          |Dead-time insertion is only active when this pair of complementary PWM is enabled. If dead-
     * |        |          |time insertion is inactive, the outputs of pin pair are complementary without any delay.
     * |        |          |0 = Dead-time insertion Disabled on the pin pair (PWM0, PWM1).
     * |        |          |1 = Dead-time insertion Enabled on the pin pair (PWM0, PWM1).
     * |        |          |Note: x=0~1 for PWM unit0~1.
     * |[17]    |DTEN2     |Enable Dead-Time Insertion For PWMx Pair (PWM2, PWM3)
     * |        |          |Dead-time insertion is only active when this pair of complementary PWM is enabled. If dead-
     * |        |          |time insertion is inactive, the outputs of pin pair are complementary without any delay.
     * |        |          |0 = Dead-time insertion Disabled on the pin pair (PWM2, PWM3).
     * |        |          |1 = Dead-time insertion Enabled on the pin pair (PWM2, PWM3).
     * |        |          |Note: x=2~3 for PWM unit0~1.
     * |[18]    |DTEN4     |Enable Dead-Time Insertion For PWMx Pair (PWM4, PWM5)
     * |        |          |Dead-time insertion is only active when this pair of complementary PWM is enabled. If dead-
     * |        |          |time insertion is inactive, the outputs of pin pair are complementary without any delay.
     * |        |          |0 = Dead-time insertion Disabled on the pin pair (PWM4, PWM5).
     * |        |          |1 = Dead-time insertion Enabled on the pin pair (PWM4, PWM5).
     * |        |          |Note: x=4~5 for PWM unit0~1.
     * @var EPWM_T::PWMB
     * Offset: 0x30  EPWM Brake Output
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |PWMB      |PWM Brake Output
     * |        |          |When PWM Brake is asserted, the PWM0~5 output state before polarity control will follow
     * |        |          |PWMB bit0~5 setting, respectively.
     * |        |          |0 = PWMn output before polarity control is low when Brake is asserted.
     * |        |          |1 = PWMn output before polarity control is high when Brake is asserted.
     * |        |          |Note: n = 0~5.
     * @var EPWM_T::PNPC
     * Offset: 0x34  EPWM Negative Polarity Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |PNPn      |PWM Negative Polarity Control
     * |        |          |The register bit controls polarity/active state of real PWM output.
     * |        |          |0 = PWMn output is active high.
     * |        |          |1 = PWMn output is active low.
     * |        |          |Note: n = 0~5.
     * @var EPWM_T::PWMFCNT
     * Offset: 0x3C  EPWMF Compared Counter
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PWMFCNT   |PWMF Compared Counter
     * |        |          |The register sets the count number which defines how many times of PWM period occurs to
     * |        |          |set bit PWMF to request the PWM period interrupt.
     * |        |          |PWMF will be set in every (1 + PWMFCNT[3:0]) time of PWM period or center point defined
     * |        |          |by INT_TYPE at PWMCON[8] occurs.
     * @var EPWM_T::PWMEIC
     * Offset: 0x40  EPWM Edge Interrupt Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWM0EI_EN |Enable PWMx0 Edge Interrupt
     * |        |          |0 = Flag PWM0EF Disabled to trigger PWM interrupt.
     * |        |          |1 = Flag PWM0EF Enabled to trigger PWM interrupt.
     * |[1]     |PWM2EI_EN |Enable PWMx2 Edge Interrupt
     * |        |          |0 = Flag PWM2EF Disabled to trigger PWM interrupt.
     * |        |          |1 = Flag PWM2EF Enabled to trigger PWM interrupt.
     * |[2]     |PWM4EI_EN |Enable PWMx4 Edge Interrupt
     * |        |          |0 = Flag PWM4EF Disabled to trigger PWM interrupt.
     * |        |          |1 = Flag PWM4EF Enabled to trigger PWM interrupt.
     * |[8]     |EINT0_TYPE|PWMx0 Edge Interrupt Type
     * |        |          |0 = PWM0EF will be set if falling edge is detected at PWMx0.
     * |        |          |1 = PWM0EF will be set if rising edge is detected at PWMx0.
     * |[9]     |EINT2_TYPE|PWMx2 Edge Interrupt Type
     * |        |          |0 = PWM2EF will be set if falling edge is detected at PWMx2.
     * |        |          |1 = PWM2EF will be set if rising edge is detected at PWMx2.
     * |[10]    |EINT4_TYPE|PWMx4 Edge Interrupt Type
     * |        |          |0 = PWM4EF will be set if falling edge is detected at PWMx4.
     * |        |          |1 = PWM4EF will be set if rising edge is detected at PWMx4.
     */

    __IO uint32_t PWMCON;        /* Offset: 0x00  EPWM Control Register                                              */
    __IO uint32_t PWMSTS;        /* Offset: 0x04  EPWM Status Register                                               */
    __IO uint32_t PWMP;          /* Offset: 0x08  EPWM Period Register                                               */
    __IO uint32_t PWM0;          /* Offset: 0x0C  EPWM PWM0 Duty Register                                            */
    __IO uint32_t PWM2;          /* Offset: 0x10  EPWM PWM2 Duty Register                                            */
    __IO uint32_t PWM4;          /* Offset: 0x14  EPWM PWM4 Duty Register                                            */
    __IO uint32_t PMSKE;         /* Offset: 0x18  EPWM Mask Mode Enable Register                                     */
    __IO uint32_t PMSKD;         /* Offset: 0x1C  EPWM Mask Mode Data Register                                       */
    __I  uint32_t RESERVE0[3];
    __IO uint32_t PDTC;          /* Offset: 0x2C  EPWM Dead-time Control Register                                    */
    __IO uint32_t PWMB;          /* Offset: 0x30  EPWM Brake Output                                                  */
    __IO uint32_t PNPC;          /* Offset: 0x34  EPWM Negative Polarity Control                                     */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t PWMFCNT;       /* Offset: 0x3C  EPWMF Compared Counter                                             */
    __IO uint32_t PWMEIC;        /* Offset: 0x40  EPWM Edge Interrupt Control Register                               */

} EPWM_T;



/**
    @addtogroup EPWM_CONST EPWM Bit Field Definition
    Constant Definitions for EPWM Controller
@{ */

/* EPWM PWMCON Bit Field Definitions */
#define EPWM_PWMCON_CLDMD_Pos                    31                                     /*!< EPWM_T::PWMCON: CLDMD Position */
#define EPWM_PWMCON_CLDMD_Msk                    (0x1ul << EPWM_PWMCON_CLDMD_Pos)       /*!< EPWM_T::PWMCON: CLDMD Mask */

#define EPWM_PWMCON_AUTOLD_Pos                   30                                     /*!< EPWM_T::PWMCON: AUTOLD Position */
#define EPWM_PWMCON_AUTOLD_Msk                   (0x1ul << EPWM_PWMCON_AUTOLD_Pos)      /*!< EPWM_T::PWMCON: AUTOLD Mask */

#define EPWM_PWMCON_BK1NF_DIS_Pos                29                                     /*!< EPWM_T::PWMCON: BK1NF_DIS Position */
#define EPWM_PWMCON_BK1NF_DIS_Msk                (0x1ul << EPWM_PWMCON_BK1NF_DIS_Pos)   /*!< EPWM_T::PWMCON: BK1NF_DIS Mask */

#define EPWM_PWMCON_BK0NF_DIS_Pos                28                                     /*!< EPWM_T::PWMCON: BK0NF_DIS Position */
#define EPWM_PWMCON_BK0NF_DIS_Msk                (0x1ul << EPWM_PWMCON_BK0NF_DIS_Pos)   /*!< EPWM_T::PWMCON: BK0NF_DIS Mask */

#define EPWM_PWMCON_LVDBK_EN_Pos                 27                                     /*!< EPWM_T::PWMCON: LVDBK_EN Position */
#define EPWM_PWMCON_LVDBK_EN_Msk                 (0x1ul << EPWM_PWMCON_LVDBK_EN_Pos)    /*!< EPWM_T::PWMCON: LVDBK_EN Mask */

#define EPWM_PWMCON_CPO2BK_EN_Pos                26                                     /*!< EPWM_T::PWMCON: CPO2BK_EN Position */
#define EPWM_PWMCON_CPO2BK_EN_Msk                (0x1ul << EPWM_PWMCON_CPO2BK_EN_Pos)   /*!< EPWM_T::PWMCON: CPO2BK_EN Mask */

#define EPWM_PWMCON_CPO1BK_EN_Pos                25                                     /*!< EPWM_T::PWMCON: CPO1BK_EN Position */
#define EPWM_PWMCON_CPO1BK_EN_Msk                (0x1ul << EPWM_PWMCON_CPO1BK_EN_Pos)   /*!< EPWM_T::PWMCON: CPO1BK_EN Mask */

#define EPWM_PWMCON_CPO0BK_EN_Pos                24                                     /*!< EPWM_T::PWMCON: CPO0BK_EN Position */
#define EPWM_PWMCON_CPO0BK_EN_Msk                (0x1ul << EPWM_PWMCON_CPO0BK_EN_Pos)   /*!< EPWM_T::PWMCON: CPO0BK_EN Mask */

#define EPWM_PWMCON_BK1FILT_Pos                  22                                     /*!< EPWM_T::PWMCON: BK1FILT Position */
#define EPWM_PWMCON_BK1FILT_Msk                  (0x3ul << EPWM_PWMCON_BK1FILT_Pos)     /*!< EPWM_T::PWMCON: BK1FILT Mask */

#define EPWM_PWMCON_BK0FILT_Pos                  20                                     /*!< EPWM_T::PWMCON: BK0FILT Position */
#define EPWM_PWMCON_BK0FILT_Msk                  (0x3ul << EPWM_PWMCON_BK0FILT_Pos)     /*!< EPWM_T::PWMCON: BK0FILT Mask */

#define EPWM_PWMCON_BK1SEL_Pos                   18                                     /*!< EPWM_T::PWMCON: BK1SEL Position */
#define EPWM_PWMCON_BK1SEL_Msk                   (0x3ul << EPWM_PWMCON_BK1SEL_Pos)      /*!< EPWM_T::PWMCON: BK1SEL Mask */

#define EPWM_PWMCON_BK1_EN_Pos                   17                                     /*!< EPWM_T::PWMCON: BK1_EN Position */
#define EPWM_PWMCON_BK1_EN_Msk                   (0x1ul << EPWM_PWMCON_BK1_EN_Pos)      /*!< EPWM_T::PWMCON: BK1_EN Mask */

#define EPWM_PWMCON_BK0_EN_Pos                   16                                     /*!< EPWM_T::PWMCON: BK0_EN Position */
#define EPWM_PWMCON_BK0_EN_Msk                   (0x1ul << EPWM_PWMCON_BK0_EN_Pos)      /*!< EPWM_T::PWMCON: BK0_EN Mask */

#define EPWM_PWMCON_INVBKP1_Pos                  15                                     /*!< EPWM_T::PWMCON: INVBKP1 Position */
#define EPWM_PWMCON_INVBKP1_Msk                  (0x1ul << EPWM_PWMCON_INVBKP1_Pos)     /*!< EPWM_T::PWMCON: INVBKP1 Mask */

#define EPWM_PWMCON_INVBKP0_Pos                  14                                     /*!< EPWM_T::PWMCON: INVBKP0 Position */
#define EPWM_PWMCON_INVBKP0_Msk                  (0x1ul << EPWM_PWMCON_INVBKP0_Pos)     /*!< EPWM_T::PWMCON: INVBKP0 Mask */

#define EPWM_PWMCON_GRP_Pos                      13                                     /*!< EPWM_T::PWMCON: GRP Position */
#define EPWM_PWMCON_GRP_Msk                      (0x1ul << EPWM_PWMCON_GRP_Pos)         /*!< EPWM_T::PWMCON: GRP Mask */

#define EPWM_PWMCON_PWMTYPE_Pos                  12                                     /*!< EPWM_T::PWMCON: PWMTYPE Position */
#define EPWM_PWMCON_PWMTYPE_Msk                  (0x1ul << EPWM_PWMCON_PWMTYPE_Pos)     /*!< EPWM_T::PWMCON: PWMTYPE Mask */

#define EPWM_PWMCON_CLRPWM_Pos                   11                                     /*!< EPWM_T::PWMCON: CLRPWM Position */
#define EPWM_PWMCON_CLRPWM_Msk                   (0x1ul << EPWM_PWMCON_CLRPWM_Pos)      /*!< EPWM_T::PWMCON: CLRPWM Mask */

#define EPWM_PWMCON_PWMINV_Pos                   9                                      /*!< EPWM_T::PWMCON: PWMINV Position */
#define EPWM_PWMCON_PWMINV_Msk                   (0x1ul << EPWM_PWMCON_PWMINV_Pos)      /*!< EPWM_T::PWMCON: PWMINV Mask */

#define EPWM_PWMCON_INT_TYPE_Pos                 8                                      /*!< EPWM_T::PWMCON: INT_TYPE Position */
#define EPWM_PWMCON_INT_TYPE_Msk                 (0x1ul << EPWM_PWMCON_INT_TYPE_Pos)    /*!< EPWM_T::PWMCON: INT_TYPE Mask */

#define EPWM_PWMCON_PWMRUN_Pos                   7                                      /*!< EPWM_T::PWMCON: PWMRUN Position */
#define EPWM_PWMCON_PWMRUN_Msk                   (0x1ul << EPWM_PWMCON_PWMRUN_Pos)      /*!< EPWM_T::PWMCON: PWMRUN Mask */

#define EPWM_PWMCON_LOAD_Pos                     6                                      /*!< EPWM_T::PWMCON: LOAD Position */
#define EPWM_PWMCON_LOAD_Msk                     (0x1ul << EPWM_PWMCON_LOAD_Pos)        /*!< EPWM_T::PWMCON: LOAD Mask */

#define EPWM_PWMCON_BRKI_EN_Pos                  5                                      /*!< EPWM_T::PWMCON: BRKI_EN Position */
#define EPWM_PWMCON_BRKI_EN_Msk                  (0x1ul << EPWM_PWMCON_BRKI_EN_Pos)     /*!< EPWM_T::PWMCON: BRKI_EN Mask */

#define EPWM_PWMCON_PWMI_EN_Pos                  4                                      /*!< EPWM_T::PWMCON: PWMI_EN Position */
#define EPWM_PWMCON_PWMI_EN_Msk                  (0x1ul << EPWM_PWMCON_PWMI_EN_Pos)     /*!< EPWM_T::PWMCON: PWMI_EN Mask */

#define EPWM_PWMCON_PWMDIV_Pos                   2                                      /*!< EPWM_T::PWMCON: PWMDIV Position */
#define EPWM_PWMCON_PWMDIV_Msk                   (0x3ul << EPWM_PWMCON_PWMDIV_Pos)      /*!< EPWM_T::PWMCON: PWMDIV Mask */

#define EPWM_PWMCON_PWMMOD_Pos                   0                                      /*!< EPWM_T::PWMCON: PWMMOD Position */
#define EPWM_PWMCON_PWMMOD_Msk                   (0x3ul << EPWM_PWMCON_PWMMOD_Pos)      /*!< EPWM_T::PWMCON: PWMMOD Mask */

/* EPWM PWMSTS Bit Field Definitions */
#define EPWM_PWMSTS_BK1STS_Pos                   25                                     /*!< EPWM_T::PWMSTS: BK1STS Position */
#define EPWM_PWMSTS_BK1STS_Msk                   (0x1ul << EPWM_PWMSTS_BK1STS_Pos)      /*!< EPWM_T::PWMSTS: BK1STS Mask */

#define EPWM_PWMSTS_BK0STS_Pos                   24                                     /*!< EPWM_T::PWMSTS: BK0STS Position */
#define EPWM_PWMSTS_BK0STS_Msk                   (0x1ul << EPWM_PWMSTS_BK0STS_Pos)      /*!< EPWM_T::PWMSTS: BK0STS Mask */

#define EPWM_PWMSTS_BKLK0_Pos                    8                                      /*!< EPWM_T::PWMSTS: BKLK0 Position */
#define EPWM_PWMSTS_BKLK0_Msk                    (0x1ul << EPWM_PWMSTS_BKLK0_Pos)       /*!< EPWM_T::PWMSTS: BKLK0 Mask */

#define EPWM_PWMSTS_PWM4EF_Pos                   6                                      /*!< EPWM_T::PWMSTS: PWM4EF Position */
#define EPWM_PWMSTS_PWM4EF_Msk                   (0x1ul << EPWM_PWMSTS_PWM4EF_Pos)      /*!< EPWM_T::PWMSTS: PWM4EF Mask */

#define EPWM_PWMSTS_PWM2EF_Pos                   5                                      /*!< EPWM_T::PWMSTS: PWM2EF Position */
#define EPWM_PWMSTS_PWM2EF_Msk                   (0x1ul << EPWM_PWMSTS_PWM2EF_Pos)      /*!< EPWM_T::PWMSTS: PWM2EF Mask */

#define EPWM_PWMSTS_PWM0EF_Pos                   4                                      /*!< EPWM_T::PWMSTS: PWM0EF Position */
#define EPWM_PWMSTS_PWM0EF_Msk                   (0x1ul << EPWM_PWMSTS_PWM0EF_Pos)      /*!< EPWM_T::PWMSTS: PWM0EF Mask */

#define EPWM_PWMSTS_PWMF_Pos                     2                                      /*!< EPWM_T::PWMSTS: PWMF Position */
#define EPWM_PWMSTS_PWMF_Msk                     (0x1ul << EPWM_PWMSTS_PWMF_Pos)        /*!< EPWM_T::PWMSTS: PWMF Mask */

#define EPWM_PWMSTS_BKF1_Pos                     1                                      /*!< EPWM_T::PWMSTS: BKF1 Position */
#define EPWM_PWMSTS_BKF1_Msk                     (0x1ul << EPWM_PWMSTS_BKF1_Pos)        /*!< EPWM_T::PWMSTS: BKF1 Mask */

#define EPWM_PWMSTS_BKF0_Pos                     0                                      /*!< EPWM_T::PWMSTS: BKF0 Position */
#define EPWM_PWMSTS_BKF0_Msk                     (0x1ul << EPWM_PWMSTS_BKF0_Pos)        /*!< EPWM_T::PWMSTS: BKF0 Mask */

/* EPWM PWMP Bit Field Definitions */
#define EPWM_PWMP_PWMP_Pos                       0                                      /*!< EPWM_T::PWMP: PWMP Position */
#define EPWM_PWMP_PWMP_Msk                       (0xFFFFul << EPWM_PWMP_PWMP_Pos)       /*!< EPWM_T::PWMP: PWMP Mask */

/* EPWM PWM0 Bit Field Definitions */
#define EPWM_PWM0_PWM_Duty_Pos                   0                                      /*!< EPWM_T::PWM0: PWM_Duty Position */
#define EPWM_PWM0_PWM_Duty_Msk                   (0xFFFFul << EPWM_PWM0_PWM_Duty_Pos)   /*!< EPWM_T::PWM0: PWM_Duty Mask */

/* EPWM PWM2 Bit Field Definitions */
#define EPWM_PWM2_PWM_Duty_Pos                   0                                      /*!< EPWM_T::PWM2: PWM_Duty Position */
#define EPWM_PWM2_PWM_Duty_Msk                   (0xFFFFul << EPWM_PWM2_PWM_Duty_Pos)   /*!< EPWM_T::PWM2: PWM_Duty Mask */

/* EPWM PWM4 Bit Field Definitions */
#define EPWM_PWM4_PWM_Duty_Pos                   0                                      /*!< EPWM_T::PWM4: PWM_Duty Position */
#define EPWM_PWM4_PWM_Duty_Msk                   (0xFFFFul << EPWM_PWM4_PWM_Duty_Pos)   /*!< EPWM_T::PWM4: PWM_Duty Mask */

/* EPWM PMSKE Bit Field Definitions */
#define EPWM_PMSKE_PMSKE_Pos                     0                                      /*!< EPWM_T::PMSKE: PMSKE Position */
#define EPWM_PMSKE_PMSKE_Msk                     (0x3Ful << EPWM_PMSKE_PMSKE_Pos)       /*!< EPWM_T::PMSKE: PMSKE Mask */

/* EPWM PMSKD Bit Field Definitions */
#define EPWM_PMSKD_PMSKD_Pos                     0                                      /*!< EPWM_T::PMSKD: PMSKD Position */
#define EPWM_PMSKD_PMSKD_Msk                     (0x3Ful << EPWM_PMSKD_PMSKD_Pos)       /*!< EPWM_T::PMSKD: PMSKD Mask */

/* EPWM PDTC Bit Field Definitions */
#define EPWM_PDTC_DTEN4_Pos                      18                                     /*!< EPWM_T::PDTC: DTEN4 Position */
#define EPWM_PDTC_DTEN4_Msk                      (0x1ul << EPWM_PDTC_DTEN4_Pos)         /*!< EPWM_T::PDTC: DTEN4 Mask */

#define EPWM_PDTC_DTEN2_Pos                      17                                     /*!< EPWM_T::PDTC: DTEN2 Position */
#define EPWM_PDTC_DTEN2_Msk                      (0x1ul << EPWM_PDTC_DTEN2_Pos)         /*!< EPWM_T::PDTC: DTEN2 Mask */

#define EPWM_PDTC_DTEN0_Pos                      16                                     /*!< EPWM_T::PDTC: DTEN0 Position */
#define EPWM_PDTC_DTEN0_Msk                      (0x1ul << EPWM_PDTC_DTEN0_Pos)         /*!< EPWM_T::PDTC: DTEN0 Mask */

#define EPWM_PDTC_DTCNT_Pos                      0                                      /*!< EPWM_T::PDTC: DTCNT Position */
#define EPWM_PDTC_DTCNT_Msk                      (0x7FFul << EPWM_PDTC_DTCNT_Pos)       /*!< EPWM_T::PDTC: DTCNT Mask */

/* EPWM PWMB Bit Field Definitions */
#define EPWM_PWMB_PWMB_Pos                       0                                      /*!< EPWM_T::PWMB: PWMB Position */
#define EPWM_PWMB_PWMB_Msk                       (0x3Ful << EPWM_PWMB_PWMB_Pos)         /*!< EPWM_T::PWMB: PWMB Mask */

/* EPWM PNPC Bit Field Definitions */
#define EPWM_PNPC_PNP5_Pos                       5                                      /*!< EPWM_T::PNPC: PNP5 Position */
#define EPWM_PNPC_PNP5_Msk                       (0x1ul << EPWM_PNPC_PNP5_Pos)          /*!< EPWM_T::PNPC: PNP5 Mask */

#define EPWM_PNPC_PNP4_Pos                       4                                      /*!< EPWM_T::PNPC: PNP4 Position */
#define EPWM_PNPC_PNP4_Msk                       (0x1ul << EPWM_PNPC_PNP4_Pos)          /*!< EPWM_T::PNPC: PNP4 Mask */

#define EPWM_PNPC_PNP3_Pos                       3                                      /*!< EPWM_T::PNPC: PNP3 Position */
#define EPWM_PNPC_PNP3_Msk                       (0x1ul << EPWM_PNPC_PNP3_Pos)          /*!< EPWM_T::PNPC: PNP3 Mask */

#define EPWM_PNPC_PNP2_Pos                       2                                      /*!< EPWM_T::PNPC: PNP2 Position */
#define EPWM_PNPC_PNP2_Msk                       (0x1ul << EPWM_PNPC_PNP2_Pos)          /*!< EPWM_T::PNPC: PNP2 Mask */

#define EPWM_PNPC_PNP1_Pos                       1                                      /*!< EPWM_T::PNPC: PNP1 Position */
#define EPWM_PNPC_PNP1_Msk                       (0x1ul << EPWM_PNPC_PNP1_Pos)          /*!< EPWM_T::PNPC: PNP1 Mask */

#define EPWM_PNPC_PNP0_Pos                       1                                      /*!< EPWM_T::PNPC: PNP0 Position */
#define EPWM_PNPC_PNP0_Msk                       (0x1ul << EPWM_PNPC_PNP0_Pos)          /*!< EPWM_T::PNPC: PNP0 Mask */

/* EPWM PWMFCNT Bit Field Definitions */
#define EPWM_PWMFCNT_PWMFCNT_Pos                 0                                      /*!< EPWM_T::PWMFCNT: PWMFCNT Position */
#define EPWM_PWMFCNT_PWMFCNT_Msk                 (0xFul << EPWM_PWMFCNT_PWMFCNT_Pos)    /*!< EPWM_T::PWMFCNT: PWMFCNT Mask */

/* EPWM PWMEIC Bit Field Definitions */
#define EPWM_PWMEIC_EINT4_TYPE_Pos               10                                     /*!< EPWM_T::PWMEIC: EINT4_TYPE Position */
#define EPWM_PWMEIC_EINT4_TYPE_Msk               (0x1ul << EPWM_PWMEIC_EINT4_TYPE_Pos)  /*!< EPWM_T::PWMEIC: EINT4_TYPE Mask */

#define EPWM_PWMEIC_EINT2_TYPE_Pos               9                                      /*!< EPWM_T::PWMEIC: EINT2_TYPE Position */
#define EPWM_PWMEIC_EINT2_TYPE_Msk               (0x1ul << EPWM_PWMEIC_EINT2_TYPE_Pos)  /*!< EPWM_T::PWMEIC: EINT2_TYPE Mask */

#define EPWM_PWMEIC_EINT0_TYPE_Pos               8                                      /*!< EPWM_T::PWMEIC: EINT0_TYPE Position */
#define EPWM_PWMEIC_EINT0_TYPE_Msk               (0x1ul << EPWM_PWMEIC_EINT0_TYPE_Pos)  /*!< EPWM_T::PWMEIC: EINT0_TYPE Mask */

#define EPWM_PWMEIC_PWM4EI_EN_Pos                2                                      /*!< EPWM_T::PWMEIC: PWM4EI_EN Position */
#define EPWM_PWMEIC_PWM4EI_EN_Msk                (0x1ul << EPWM_PWMEIC_PWM4EI_EN_Pos)   /*!< EPWM_T::PWMEIC: PWM4EI_EN Mask */

#define EPWM_PWMEIC_PWM2EI_EN_Pos                1                                      /*!< EPWM_T::PWMEIC: PWM2EI_EN Position */
#define EPWM_PWMEIC_PWM2EI_EN_Msk                (0x1ul << EPWM_PWMEIC_PWM2EI_EN_Pos)   /*!< EPWM_T::PWMEIC: PWM2EI_EN Mask */

#define EPWM_PWMEIC_PWM0EI_EN_Pos                0                                      /*!< EPWM_T::PWMEIC: PWM0EI_EN Position */
#define EPWM_PWMEIC_PWM0EI_EN_Msk                (0x1ul << EPWM_PWMEIC_PWM0EI_EN_Pos)   /*!< EPWM_T::PWMEIC: PWM0EI_EN Mask */

/*@}*/ /* end of group EPWM_CONST */
/*@}*/ /* end of group EPWM */



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
     * |[0]     |ISPEN     |ISP Enable(Write-Protected)
     * |        |          |This bit is protected bit. ISP function enable bit. Set this bit to enable ISP function.
     * |        |          |0 = Disable ISP function
     * |        |          |1 = Enable ISP function
     * |[1]     |BS        |Boot Select(Write-Protected)
     * |        |          |This bit is protected bit. Set/clear this bit to select next booting from LDROM/APROM,
     * |        |          |respectively. This bit also functions as MCU booting status flag, which can be used to check where
     * |        |          |MCU booted from. This bit is initiated with the inverted value of CBS in Config0 after power-
     * |        |          |on reset; It keeps the same value at other reset.
     * |        |          |0 = boot from APROM
     * |        |          |1 = boot from LDROM
     * |[3]     |APUEN     |APROM Update Enable(Write-Protected)
     * |        |          |0 = APROM cannot be updated when the chip runs in APROM
     * |        |          |1 = APROM can be updated when the chip runs in APROM
     * |[4]     |CFGUEN    |Config Update Enable(Write-Protected)
     * |        |          |Writing this bit to 1 enables S/W to update Config value by ISP procedure regardless of program
     * |        |          |code is running in APROM or LDROM.
     * |        |          |0 = Config update disable
     * |        |          |1 = Config update enable
     * |[5]     |LDUEN     |LDROM Update Enable(Write-Protected)
     * |        |          |LDROM update enable bit.
     * |        |          |0 = LDROM cannot be updated
     * |        |          |1 = LDROM can be updated when the MCU runs in APROM.
     * |[6]     |ISPFF     |ISP Fail Flag(Write-Protected)
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself.
     * |        |          |(2) LDROM writes to itself.
     * |        |          |(3) CONFIG is erased/programmed when CFGUEN is set to 0
     * |        |          |(4) Destination address is illegal, such as over an available range.
     * |        |          |Write 1 to clear.
     * @var FMC_T::ISPADR
     * Offset: 0x04  ISP Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPADR    |ISP Address
     * |        |          |The NuMicro M0519 Series has a maximum 32Kx32 (128 KB) of embedded Flash,
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
     * |[5:0]   |ISPCMD    |ISP Command
     * |        |          |ISP command table is shown below:
     * |        |          |0x00   =  Read.
     * |        |          |0x21   =  Program.
     * |        |          |0x22   =  Page Erase.
     * |        |          |0x2E   =  Vector page Re-Map.
     * @var FMC_T::ISPTRG
     * Offset: 0x10  ISP Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP Start Trigger(Write-Protection)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP
     * |        |          |operation is finish.
     * |        |          |0 = ISP done
     * |        |          |1 = ISP is on going
     * @var FMC_T::DFBADR
     * Offset: 0x14  Data Flash Base Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DFBADR    |Data Flash Base Address
     * |        |          |This register indicates data flash start address.
     * |        |          |It is read only.
     * |        |          |For 128 KB flash memory device, the data flash size is defined by user configuration,
     * |        |          |register content is loaded from Config1 when chip is powered on but for 64/32 KB device,
     * |        |          |it is fixed at 0x0001_F000.
     * @var FMC_T::FATCON
     * Offset: 0x18  Flash Access Time Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |FOM_SEL[0]|Chip Frequency Optimization Mode Select(Write-Protection)
     * |        |          |When chip operation frequency is lower than 25 MHz, chip can work more efficiently by setting FOM_SEL[1:0] = 01.
     * |        |          |When chip operation frequency is lower than 50 MHz, chip can work more efficiently by setting FOM_SEL[1:0] = 00.
     * |        |          |When chip operation frequency is over than 50 MHz, chip only can work by setting FOM_SEL[1:0] = 11.
     * |        |          |00 = Middle frequency optimization mode Enabled.
     * |        |          |01 = Low frequency optimization mode Enabled.
     * |        |          |10 = Reserved.
     * |        |          |11 = High frequency optimization mode Enabled.
     * |[6]     |FOM_SEL[1]|Chip Frequency Optimization Mode Select(Write-Protection)
     * |        |          |When chip operation frequency is lower than 25 MHz, chip can work more efficiently by setting FOM_SEL[1:0] = 01.
     * |        |          |When chip operation frequency is lower than 50 MHz, chip can work more efficiently by setting FOM_SEL[1:0] = 00.
     * |        |          |When chip operation frequency is over than 50 MHz, chip only can work by setting FOM_SEL[1:0] = 11.
     * |        |          |00 = Middle frequency optimization mode Enabled.
     * |        |          |01 = Low frequency optimization mode Enabled.
     * |        |          |10 = Reserved.
     * |        |          |11 = High frequency optimization mode Enabled.
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
    __IO uint32_t ISPTRG;        /* Offset: 0x10  ISP Trigger Control Register                                       */
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
#define FMC_ISPCMD_ISPCMD_Pos                   0                                       /*!< FMC_T::ISPCMD: ISPCMD Position */
#define FMC_ISPCMD_ISPCMD_Msk                   (0x3Ful << FMC_ISPCMD_ISPCMD_Pos)       /*!< FMC_T::ISPCMD: ISPCMD Mask */

/* FMC ISPTRG Bit Field Definitions */
#define FMC_ISPTRG_ISPGO_Pos                    0                                       /*!< FMC_T::ISPTRG: ISPGO Position */
#define FMC_ISPTRG_ISPGO_Msk                    (1ul << FMC_ISPTRG_ISPGO_Pos)           /*!< FMC_T::ISPTRG: ISPGO Mask */

/* FMC DFBADR Bit Field Definitions */
#define FMC_DFBADR_DFBADR_Pos                   0                                       /*!< FMC_T::DFBADR: DFBADR Position */
#define FMC_DFBADR_DFBADR_Msk                   (0xFFFFFFFFul << FMC_DFBADR_DFBADR_Pos) /*!< FMC_T::DFBADR: DFBADR Mask     */

/* FMC FATCON Bit Field Definitions */
#define FMC_FATCON_FOMSEL1_Pos                  6                                       /*!< FMC_T::FATCON: FOMSEL1 Position */
#define FMC_FATCON_FOMSEL1_Msk                  (1ul << FMC_FATCON_FOMSEL1_Pos)         /*!< FMC_T::FATCON: FOMSEL1 Mask */

#define FMC_FATCON_FOMSEL0_Pos                  4                                       /*!< FMC_T::FATCON: FOMSEL0 Position */
#define FMC_FATCON_FOMSEL0_Msk                  (1ul << FMC_FATCON_FOMSEL0_Pos)         /*!< FMC_T::FATCON: FOMSEL0 Mask */

/* FMC ISPSTA Bit Field Definitions */
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




/*---------------------- General Purpose Input/Output Controller -------------------------*/
/**
    @addtogroup GPIO General Purpose Input/Output Controller (GPIO)
    Memory Mapped Structure for GPIO Controller
@{ */

typedef struct
{


    /**
     * @var GPIO_T::PMD
     * Offset: 0x00  GPIO I/O Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2n+1:2n]|PMDn     |Port 0-A I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of Px.n pins.
     * |        |          |00 = Px.n is in input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode. 
     * |        |          |Note: Max. n = 1 for PA; Max. n = 7 for P0/P1/P2/P3/P4/P5/P6/P7/P8/P9.
     * @var GPIO_T::OFFD
     * Offset: 0x04  GPIO Port Digital Input Path Disable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:16] |OFFD      |Port 0-A Pin [n] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled. If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note: Max. n = 1 for PA; Max. n = 7 for P0/P1/P2/P3/P4/P5/P6/P7/P8/P9.
     * @var GPIO_T::DOUT
     * Offset: 0x08  GPIO Port Data Output Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DOUT[n]   |Port 0-A Pin [n] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note: Max. n = 1 for PA; Max. n = 7 for P0/P1/P2/P3/P4/P5/P6/P7/P8/P9.
     * @var GPIO_T::DMASK
     * Offset: 0x0C  GPIO Port Data Output Write Mask
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DMASK[n]  |Port 0-A Pin [n] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding Px_DOUT[n] bit. When the DMASK[n] bit is set to 1, the corresponding Px_DOUT[n] bit is protected. If the write signal is masked, writing data to the protect bit is ignored.
     * |        |          |0 = Corresponding Px_DOUT[n] bit can be updated.
     * |        |          |1 = Corresponding Px_DOUT[n] bit protected.
     * |        |          |Note1: Max. n = 1 for PA; Max. n = 7 for P0/P1/P2/P3/P4/P5/P6/P7/P8/P9.
     * |        |          |Note2: This function only protects the corresponding Px_DOUT[n] bit, and will not protect the corresponding Px_n bit control register.
     * @var GPIO_T::PIN
     * Offset: 0x10 GPIO Port Pin Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PIN[n]    |Port 0-A Pin [n] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note: Max. n = 1 for PA; Max. n = 7 for P0/P1/P2/P3/P4/P5/P6/P7/P8/P9.
     * @var GPIO_T::DBEN
     * Offset: 0x14  GPIO Port De-bounce Enable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DBEN[n]   |Port 0-A Pin [n] Input Signal De-bounce Enable Control
     * |        |          |DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBNCECON[4], one de-bounce sample cycle period is controlled by DBNCECON[3:0].
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt. If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note1: Max. n = 1 for PA; Max. n = 7 for P0/P1/P2/P3/P4/P5/P6/P7/P8/P9.
     * |        |          |Note2: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
     * @var GPIO_T::IMD
     * Offset: 0x18 GPIO Port Interrupt Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |IMD[n]    |Port 0-A Pin [n] Edge or Level Detection Interrupt Mode Control
     * |        |          |IMD[n] bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers Px_IEN. If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt. If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note: Max. n = 1 for PA; Max. n = 7 for P0/P1/P2/P3/P4/P5/P6/P7/P8/P9.
     * @var GPIO_T::IEN
     * Offset: 0x1C  GPIO Port Interrupt Enable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:16] |IR_EN[n]  |Port 0-A Pin [n] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |IR_EN[n] bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the IR_EN[n] bit to 1 :
     * |        |          |If the interrupt is level trigger (IMD[n] is 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (IMD[n] is 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note: Max. n = 1 for PA; Max. n = 7 for P0/P1/P2/P3/P4/P5/P6/P7/P8/P9.
     * |[7:0]   |IF_EN[n]  |Port 0-A Pin [n] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |IF_EN[n] bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the IF_EN[n] bit to 1 :
     * |        |          |If the interrupt is level trigger (IMD[n] is 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger (IMD[n] is 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note: Max. n = 1 for PA; Max. n = 7 for P0/P1/P2/P3/P4/P5/P6/P7/P8/P9.
     * @var GPIO_T::ISF
     * Offset: 0x20  GPIO Port Interrupt Source Flag
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |IF_ISF[n] |Port 0-A Pin [n] Interrupt Source Flag
     * |        |          |Write :
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read :
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note: Max. n = 1 for PA; Max. n = 7 for P0/P1/P2/P3/P4/P5/P6/P7/P8/P9.
     */

    __IO uint32_t PMD;           /* Offset: 0x00  GPIO Port I/O Mode Control                                         */
    __IO uint32_t OFFD;          /* Offset: 0x04  GPIO Port Digital Input Path Disable Control                       */
    __IO uint32_t DOUT;          /* Offset: 0x08  GPIO Port Data Output Value                                        */
    __IO uint32_t DMASK;         /* Offset: 0x0C  GPIO Port Data Output Write Mask                                   */
    __I  uint32_t PIN;           /* Offset: 0x10  GPIO Port Pin Value                                                 */
    __IO uint32_t DBEN;          /* Offset: 0x14  GPIO Port De-bounce Enable                                         */
    __IO uint32_t IMD;           /* Offset: 0x18  GPIO Port Interrupt Mode Control                                    */
    __IO uint32_t IEN;           /* Offset: 0x1C  GPIO Port Interrupt Enable Control                                 */
    __IO uint32_t ISF;           /* Offset: 0x20  GPIO Port Interrupt Source Flag                                    */

} GPIO_T;




typedef struct
{


    /**
     * @var GPIO_DBNCECON_T::DBNCECON
     * Offset: 0x2E0  External Interrupt De-bounce Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |DBCLKSEL  |De-Bounce Sampling Cycle Selection
     * |        |          |0000 = Sample interrupt input once per 1 clocks
     * |        |          |0001 = Sample interrupt input once per 2 clocks
     * |        |          |0010 = Sample interrupt input once per 4 clocks
     * |        |          |0011 = Sample interrupt input once per 8 clocks
     * |        |          |0100 = Sample interrupt input once per 16 clocks
     * |        |          |0101 = Sample interrupt input once per 32 clocks
     * |        |          |0110 = Sample interrupt input once per 64 clocks
     * |        |          |0111 = Sample interrupt input once per 128 clocks
     * |        |          |1000 = Sample interrupt input once per 256 clocks
     * |        |          |1001 = Sample interrupt input once per 2*256 clocks
     * |        |          |1010 = Sample interrupt input once per 4*256clocks
     * |        |          |1011 = Sample interrupt input once per 8*256 clocks
     * |        |          |1100 = Sample interrupt input once per 16*256 clocks
     * |        |          |1101 = Sample interrupt input once per 32*256 clocks
     * |        |          |1110 = Sample interrupt input once per 64*256 clocks
     * |        |          |1111 = Sample interrupt input once per 128*256 clocks
     * |[4]     |DBCLKSRC  |De-Bounce Counter Clock Source Selection
     * |        |          |0 = De-bounce counter clock source is the HCLK.
     * |        |          |1 = De-bounce counter clock source is the internal 10 kHz low speed oscillator.
     * |[5]     |ICLK_ON   |Interrupt Clock On Mode
     * |        |          |0 = Edge detection circuit is active only if I/O pin corresponding Px_IEN bit is set to 1.
     * |        |          |1 = All I/O pins edge detection circuit is always active after reset.
     * |        |          |It is recommended to turn off this bit to save system power if no special application concern.
     * @var GPIO_DBNCECON_T::PWMPOEN
     * Offset: 0x2E4  PWM Port Output Enable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HZ_Even0  |Enhanced PWM Unit0 Even Ports Output Control
     * |        |          |0 = The driving mode of EPWM unit0 even ports are controlled by GPIO mode register (Px_PMD) or multi-function register (Px_MFP).
     * |        |          |1 = The driving mode of EPWM unit0 even ports are forced in tri-state (Hi-Z) all the time.
     * |        |          |The initial value is loaded from CHZ_Even0 (Config0[8]) after any reset.
     * |[1]     |HZ_Odd0   |Enhanced PWM Unit0 Odd Ports Output Control
     * |        |          |0 = The driving mode of EPWM unit0 odd ports are controlled by GPIO mode register (Px_PMD) or multi-function register (Px_MFP).
     * |        |          |1 = The driving mode of EPWM unit0 odd ports are forced in tri-state (Hi-Z) all the time.
     * |        |          |The initial value is loaded from CHZ_Odd0 (Config0[9]) after any reset.
     * |[2]     |HZ_Even1  |Enhanced PWM Unit1 Even Ports Output Control
     * |        |          |0 = The driving mode of EPWM unit1 even ports are controlled by GPIO mode register (Px_PMD) or multi-function register (Px_MFP).
     * |        |          |1 = The driving mode of EPWM unit1 even ports are forced in tri-state (Hi-Z) all the time.
     * |        |          |The initial value is loaded from CHZ_Even1 (Config0[10]) after any reset.
     * |[3]     |HZ_Odd1   |Enhanced PWM Unit1 Odd Ports Output Control
     * |        |          |0 = The driving mode of EPWM unit1 odd ports are controlled by GPIO mode register (Px_PMD) or multi-function register (Px_MFP).
     * |        |          |1 = The driving mode of EPWM unit1 odd ports are forced in tri-state (Hi-Z) all the time.
     * |        |          |The initial value is loaded from CHZ_Odd1 (Config0[11]) after any reset.
     * |[4]     |HZ_BPWM   |Basic PWM Ports Output Control
     * |        |          |0 = The driving mode of Basic PWM ports are controlled by GPIO mode register (Px_PMD) or multi-function register (Px_MFP).
     * |        |          |1 = The driving mode of Basic PWM ports are forced in tri-state (Hi-Z) all the time.
     * |        |          |The initial value is loaded from CHZ_BPWM (Config0[12]) after any reset.
     */

    __IO uint32_t DBNCECON;      /* Offset: 0x2E0  External Interrupt De-bounce Control                              */
    __IO uint32_t PWMPOEN;       /* Offset: 0x2E4  PWM Port Output Enable                                            */

} GPIO_DBNCECON_T;



/**
    @addtogroup GPIO_CONST GPIO Bit Field Definition
    Constant Definitions for GPIO Controller
@{ */


/* GPIO PMD Bit Field Definitions */
#define GPIO_PMD_PMD7_Pos           14                                          /*!< GPIO_T::PMD: PMD7 Position */
#define GPIO_PMD_PMD7_Msk           (0x3ul << GPIO_PMD_PMD7_Pos)                /*!< GPIO_T::PMD: PMD7 Mask */

#define GPIO_PMD_PMD6_Pos           12                                          /*!< GPIO_T::PMD: PMD6 Position */
#define GPIO_PMD_PMD6_Msk           (0x3ul << GPIO_PMD_PMD6_Pos)                /*!< GPIO_T::PMD: PMD6 Mask */

#define GPIO_PMD_PMD5_Pos           10                                          /*!< GPIO_T::PMD: PMD5 Position */
#define GPIO_PMD_PMD5_Msk           (0x3ul << GPIO_PMD_PMD5_Pos)                /*!< GPIO_T::PMD: PMD5 Mask */

#define GPIO_PMD_PMD4_Pos           8                                           /*!< GPIO_T::PMD: PMD4 Position */
#define GPIO_PMD_PMD4_Msk           (0x3ul << GPIO_PMD_PMD4_Pos)                /*!< GPIO_T::PMD: PMD4 Mask */

#define GPIO_PMD_PMD3_Pos           6                                           /*!< GPIO_T::PMD: PMD3 Position */
#define GPIO_PMD_PMD3_Msk           (0x3ul << GPIO_PMD_PMD3_Pos)                /*!< GPIO_T::PMD: PMD3 Mask */

#define GPIO_PMD_PMD2_Pos           4                                           /*!< GPIO_T::PMD: PMD2 Position */
#define GPIO_PMD_PMD2_Msk           (0x3ul << GPIO_PMD_PMD2_Pos)                /*!< GPIO_T::PMD: PMD2 Mask */

#define GPIO_PMD_PMD1_Pos           2                                           /*!< GPIO_T::PMD: PMD1 Position */
#define GPIO_PMD_PMD1_Msk           (0x3ul << GPIO_PMD_PMD1_Pos)                /*!< GPIO_T::PMD: PMD1 Mask */

#define GPIO_PMD_PMD0_Pos           0                                           /*!< GPIO_T::PMD: PMD0 Position */
#define GPIO_PMD_PMD0_Msk           (0x3ul << GPIO_PMD_PMD0_Pos)                /*!< GPIO_T::PMD: PMD0 Mask */

/* GPIO OFFD Bit Field Definitions */
#define GPIO_OFFD_OFFD_Pos          16                                          /*!< GPIO_T::OFFD: OFFD Position */
#define GPIO_OFFD_OFFD_Msk          (0xFFul << GPIO_OFFD_OFFD_Pos)              /*!< GPIO_T::OFFD: OFFD Mask */

/* GPIO DOUT Bit Field Definitions */
#define GPIO_DOUT_DOUT_Pos          0                                           /*!< GPIO_T::DOUT: DOUT Position */
#define GPIO_DOUT_DOUT_Msk          (0xFFul << GPIO_DOUT_DOUT_Pos)              /*!< GPIO_T::DOUT: DOUT Mask */

/* GPIO DMASK Bit Field Definitions */
#define GPIO_DMASK_DMASK_Pos        0                                           /*!< GPIO_T::DMASK: DMASK Position */
#define GPIO_DMASK_DMASK_Msk        (0xFFul << GPIO_DMASK_DMASK_Pos)            /*!< GPIO_T::DMASK: DMASK Mask */

/* GPIO PIN Bit Field Definitions */
#define GPIO_PIN_PIN_Pos            0                                           /*!< GPIO_T::PIN: PIN Position */
#define GPIO_PIN_PIN_Msk            (0xFFul << GPIO_PIN_PIN_Pos)                /*!< GPIO_T::PIN: PIN Mask */

/* GPIO DBEN Bit Field Definitions */
#define GPIO_DBEN_DBEN_Pos          0                                           /*!< GPIO_T::DBEN: DBEN Position */
#define GPIO_DBEN_DBEN_Msk          (0xFFul << GPIO_DBEN_DBEN_Pos)              /*!< GPIO_T::DBEN: DBEN Mask */

/* GPIO IMD Bit Field Definitions */
#define GPIO_IMD_IMD_Pos            0                                           /*!< GPIO_T::IMD: IMD Position */
#define GPIO_IMD_IMD_Msk            (0xFFul << GPIO_IMD_IMD_Pos)                /*!< GPIO_T::IMD: IMD Mask */

/* GPIO IEN Bit Field Definitions */
#define GPIO_IEN_IR_EN_Pos          16                                          /*!< GPIO_T::IEN: IR_EN Position */
#define GPIO_IEN_IR_EN_Msk          (0xFFul << GPIO_IEN_IR_EN_Pos)              /*!< GPIO_T::IEN: IR_EN Mask */

#define GPIO_IEN_IF_EN_Pos          0                                           /*!< GPIO_T::IEN: IF_EN Position */
#define GPIO_IEN_IF_EN_Msk          (0xFFul << GPIO_IEN_IF_EN_Pos)              /*!< GPIO_T::IEN: IF_EN Mask */

/* GPIO ISF Bit Field Definitions */
#define GPIO_ISF_IF_ISF_Pos         0                                           /*!< GPIO_T::ISF: IF_ISF Position */
#define GPIO_ISF_IF_ISF_Msk         (0xFFul << GPIO_ISF_IF_ISF_Pos)             /*!< GPIO_T::ISF: IF_ISF Mask */

/* GPIO DBNCECON Bit Field Definitions */
#define GPIO_DBNCECON_ICLK_ON_Pos   5                                           /*!< GPIO_DBNCECON_T::DBNCECON: ICLK_ON  Position */
#define GPIO_DBNCECON_ICLK_ON_Msk   (1ul << GPIO_DBNCECON_ICLK_ON_Pos)          /*!< GPIO_DBNCECON_T::DBNCECON: ICLK_ON  Mask */

#define GPIO_DBNCECON_DBCLKSRC_Pos  4                                           /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSRC Position */
#define GPIO_DBNCECON_DBCLKSRC_Msk  (1ul << GPIO_DBNCECON_DBCLKSRC_Pos)         /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSRC Mask */

#define GPIO_DBNCECON_DBCLKSEL_Pos  0                                           /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSEL Position */
#define GPIO_DBNCECON_DBCLKSEL_Msk  (0xFul << GPIO_DBNCECON_DBCLKSEL_Pos)       /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSEL Mask */

/* GPIO PWMPOEN Bit Field Definitions */
#define GPIO_PWMPOEN_HZ_BPWM_Pos    4                                           /*!< GPIO_T::PWMPOEN: HZ_BPWM Position */
#define GPIO_PWMPOEN_HZ_BPWM_Msk    (1ul << GPIO_PWMPOEN_HZ_BPWM_Pos)           /*!< GPIO_T::PWMPOEN: HZ_BPWM Mask */

#define GPIO_PWMPOEN_HZ_Odd1_Pos    3                                           /*!< GPIO_T::PWMPOEN: HZ_Odd1 Position */
#define GPIO_PWMPOEN_HZ_Odd1_Msk    (1ul << GPIO_PWMPOEN_HZ_Odd1_Pos)           /*!< GPIO_T::PWMPOEN: HZ_Odd1 Mask */

#define GPIO_PWMPOEN_HZ_Even1_Pos   2                                           /*!< GPIO_T::PWMPOEN: HZ_Even1 Position */
#define GPIO_PWMPOEN_HZ_Even1_Msk   (1ul << GPIO_PWMPOEN_HZ_Even1_Pos)          /*!< GPIO_T::PWMPOEN: HZ_Even1 Mask */

#define GPIO_PWMPOEN_HZ_Odd0_Pos    1                                           /*!< GPIO_T::PWMPOEN: HZ_Odd0 Position */
#define GPIO_PWMPOEN_HZ_Odd0_Msk    (1ul << GPIO_PWMPOEN_HZ_Odd0_Pos)           /*!< GPIO_T::PWMPOEN: HZ_Odd0 Mask */

#define GPIO_PWMPOEN_HZ_Even0_Pos   0                                           /*!< GPIO_T::PWMPOEN: HZ_Even0 Position */
#define GPIO_PWMPOEN_HZ_Even0_Msk   (1ul << GPIO_PWMPOEN_HZ_Even0_Pos)          /*!< GPIO_T::PWMPOEN: HZ_Even0 Mask */

/*@}*/ /* end of group GPIO_CONST */
/*@}*/ /* end of group GPIO */


/*----------------------- Hardware Divider Interface Controller ------------------*/
/** @addtogroup HDIV Hardware Divider(HDIV)
  Memory Mapped Structure for Divider Interface Controller
  @{
 */

typedef struct
{



    /**
     * @var HDIV_T::DIVIDEND
     * Offset: 0x04  Dividend Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DIVIDEND  |Dividend Source.
     * |        |          |This register is given the dividend of divider before calculation starts.
     * @var HDIV_T::DIVISOR
     * Offset: 0x08  Divisor Source Resister
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |DIVISOR   |Divisor Source.
     * |        |          |This register is given the divisor of divider before calculation starts.
     * |        |          |Note: when this register is written, hardware divider will start calculate
     * @var HDIV_T::DIVQUO
     * Offset: 0x0C  Quotient Result Resister
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |QUOTIENT  |Quotient Result
     * |        |          |This register holds the quotient result of divider after calculation complete.
     * @var HDIV_T::DIVREM
     * Offset: 0x10  Reminder Result Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |REMINDER  |Reminder Result
     * |        |          |This register holds the reminder result of divider after calculation complete.
     * @var HDIV_T::DIVSTS
     * Offset: 0x14  Divider Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DIV_FINISH|Divider operation finished.
     * |        |          |0 = The divider calculation is not yet.
     * |        |          |1 = The divider calculation is finished.
     * |        |          |This register is read only.
     * |[1]     |DIV0      |Divisor zero warning.
     * |        |          |0 = The divisor is not 0.
     * |        |          |1 = The divisor is 0.
     * |        |          |This register is read only.
     * |[2]     |DIVFF     |Divider Operation Finish Flag.
     * |        |          |When divider calculation has finished, this bit is set to 1.
     * |        |          |This bit is cleared to 0 by writing 1 to it through software.
     */

    __IO int32_t  RESERVED;
    __IO int32_t  DIVIDEND;      /* Offset: 0x04  Dividend Source Register                                           */
    __IO int32_t  DIVISOR;       /* Offset: 0x08  Divisor Source Resister                                            */
    __IO int32_t  DIVQUO;        /* Offset: 0x0C  Quotient Result Resister                                           */
    __IO int32_t  DIVREM;        /* Offset: 0x10  Reminder Result Register                                           */
    __IO uint32_t DIVSTS;        /* Offset: 0x14  Divider Status Register                                            */

} HDIV_T;




/** @addtogroup HDIV_CONST HDIV Bit Field Definition
  Constant Definitions for HDIV Controller
  @{
 */

#define HDIV_DIVCON_DIVST_Pos        0
#define HDIV_DIVCON_DIVST_Mas        (1ul << HDIV_DIVCON_DIVST_Pos)

#define HDIV_DIVSTS_DIV_FINISH_Pos   0
#define HDIV_DIVSTS_DIV_FINISH_Msk   (1ul << HDIV_DIVSTS_DIV_FINISH_Pos)

#define HDIV_DIVSTS_DIV0_Pos         1
#define HDIV_DIVSTS_DIV0_Msk         (1ul << HDIV_DIVSTS_DIV0_Pos)

#define HDIV_DIVSTS_DIVFF_Pos        2
#define HDIV_DIVSTS_DIVFF_Msk        (1ul << HDIV_DIVSTS_DIVFF_Pos)

/**@}*/ /* HDIV_CONST */
/**@}*/ /* DIV */


/*---------------------- Inter-IC Bus Controller -------------------------*/
/**
    @addtogroup I2C Inter-IC Bus Controller (I2C)
    Memory Mapped Structure for I2C Controller
@{ */

typedef struct
{


    /**
     * @var I2C_T::I2CON
     * Offset: 0x00  I2C Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |AA        |Assert Acknowledge Control
     * |        |          |When AA =1 prior to address or data received, an acknowledged (low level to I2Cn_SDA) will be returned during the acknowledge clock pulse on the I2Cn_SCL line when 1.) A slave is acknowledging the address sent from master, 2.) The receiver devices are acknowledging the data sent by transmitter.
     * |        |          |When AA=0 prior to address or data received, a Not acknowledged (high level to I2Cn_SDA) will be returned during the acknowledge clock pulse on the I2Cn_SCL line.
     * |[3]     |SI        |I2C Interrupt Flag
     * |        |          |When a new I2C state is present in the I2CSTATUS register, the SI flag is set by hardware, and if bit EI (I2CON [7]) is set, the I2C interrupt is requested.
     * |        |          |SI must be cleared by software.
     * |        |          |Clear SI by writing 1 to this bit.
     * |[4]     |STO       |I2C STOP Control
     * |        |          |In Master mode, setting STO to transmit a STOP condition to bus then I2C hardware will check the bus condition if a STOP condition is detected this bit will be cleared by hardware automatically.
     * |        |          |In a slave mode, setting STO resets I2C hardware to the defined "not addressed" slave mode.
     * |        |          |This means it is NO LONGER in the slave receiver mode to receive data from the master transmit device.
     * |[5]     |STA       |I2C START Control
     * |        |          |Setting STA to logic 1 to enter Master mode, the I2C hardware sends a START or repeat START condition to bus when the bus is free.
     * |[6]     |ENS1      |I2C Controller Enable
     * |        |          |Set to enable I2C serial function controller.
     * |        |          |When ENS1=1 the I2C serial function enables, the multi-function pin function of I2Cn_SDA and I2Cn_SCL must set to I2C function first.
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[7]     |EI        |Enable Interrupt
     * |        |          |0 = I2C interrupt Disabled.
     * |        |          |1 = I2C interrupt Enabled.
     * @var I2C_T::I2CADDR0
     * Offset: 0x04  I2C Slave Address Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     * @var I2C_T::I2CDAT
     * Offset: 0x08  I2C Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2CDAT    |I2C Data Register
     * |        |          |Bit [7:0] is located with the 8-bit transferred data of I2C serial port.
     * @var I2C_T::I2CSTATUS
     * Offset: 0x0C  I2C Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2CSTATUS |I2C Status Register
     * |        |          |The status register of I2C:
     * |        |          |The three least significant bits are always 0.
     * |        |          |The five most significant bits contain the status code.
     * |        |          |There are 26 possible status codes.
     * |        |          |When I2CSTATUS contains F8H, no serial interrupt is requested.
     * |        |          |All other I2CSTATUS values correspond to defined I2C states.
     * |        |          |When each of these states is entered, a status interrupt is requested (SI = 1).
     * |        |          |A valid status code is present in I2CSTATUS one cycle after SI is set by hardware and is still present one cycle after SI has been reset by software.
     * |        |          |In addition, states 00H stands for a Bus Error.
     * |        |          |A Bus Error occurs when a START or STOP condition is present at an illegal position in the formation frame.
     * |        |          |Example of illegal position are during the serial transfer of an address byte, a data byte or an acknowledge bit.
     * @var I2C_T::I2CLK
     * Offset: 0x10  I2C Clock Divided Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2CLK     |I2C Clock Divided Register
     * |        |          |The I2C clock rate bits: Data Baud Rate of I2C = (system clock) / (4x (I2CLK+1)).
     * |        |          |Note: The minimum value of I2CLK is 4.
     * @var I2C_T::I2CTOC
     * Offset: 0x14  I2C Time-out Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Time-out Flag
     * |        |          |This bit is set by H/W when I2C time-out happened and it can interrupt CPU if I2C interrupt enable bit (EI) is set to 1.
     * |        |          |Note: Write 1 to clear this bit.
     * |[1]     |DIV4      |Time-out Counter Input Clock Divided by 4
     * |        |          |When Enabled, The time-out period is extend 4 times.
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[2]     |ENTI      |Time-out Counter Enable/Disable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When Enabled, the 14-bit time-out counter will start counting when SI is clear.
     * |        |          |Setting flag SI to high will reset counter and re-start up counting after SI is cleared.
     * @var I2C_T::I2CADDR1
     * Offset: 0x18  I2C Slave Address Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     * @var I2C_T::I2CADDR2
     * Offset: 0x1C  I2C Slave Address Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     * @var I2C_T::I2CADDR3
     * Offset: 0x20  I2C Slave Address Register3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the chip's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     * @var I2C_T::I2CADM0
     * Offset: 0x24  I2C Slave Address Mask Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * @var I2C_T::I2CADM1
     * Offset: 0x28  I2C Slave Address Mask Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * @var I2C_T::I2CADM2
     * Offset: 0x2C  I2C Slave Address Mask Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * @var I2C_T::I2CADM3
     * Offset: 0x30  I2C Slave Address Mask Register3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * @var I2C_T::I2CWKUPCON
     * Offset: 0x3C  I2C Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKUPEN    |I2C Wake-up Enable
     * |        |          |0 = I2C wake-up function Disabled.
     * |        |          |1 = I2C wake-up function Enabled.
     * @var I2C_T::I2CWKUPSTS
     * Offset: 0x40  I2C Wake-up Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKUPIF    |I2C Wake-up Flag
     * |        |          |When chip is woken up from Power-down mode by I2C, this bit is set to 1.
     * |        |          |Software can write 1 to clear this bit.
     */

    __IO uint32_t I2CON;         /* Offset: 0x00  I2C Control Register                                               */
    __IO uint32_t I2CADDR0;      /* Offset: 0x04  I2C Slave Address Register0                                        */
    __IO uint32_t I2CDAT;        /* Offset: 0x08  I2C Data Register                                                  */
    __I  uint32_t I2CSTATUS;     /* Offset: 0x0C  I2C Status Register                                                */
    __IO uint32_t I2CLK;         /* Offset: 0x10  I2C Clock Divided Register                                         */
    __IO uint32_t I2CTOC;        /* Offset: 0x14  I2C Time-out Counter Register                                      */
    __IO uint32_t I2CADDR1;      /* Offset: 0x18  I2C Slave Address Register1                                        */
    __IO uint32_t I2CADDR2;      /* Offset: 0x1C  I2C Slave Address Register2                                        */
    __IO uint32_t I2CADDR3;      /* Offset: 0x20  I2C Slave Address Register3                                        */
    __IO uint32_t I2CADM0;       /* Offset: 0x24  I2C Slave Address Mask Register0                                   */
    __IO uint32_t I2CADM1;       /* Offset: 0x28  I2C Slave Address Mask Register1                                   */
    __IO uint32_t I2CADM2;       /* Offset: 0x2C  I2C Slave Address Mask Register2                                   */
    __IO uint32_t I2CADM3;       /* Offset: 0x30  I2C Slave Address Mask Register3                                   */
    __I  uint32_t RESERVED0[2];
    __IO uint32_t I2CWKUPCON;    /* Offset: 0x3C  I2C Wake-up Control Register                                       */
    __IO uint32_t I2CWKUPSTS;    /* Offset: 0x40  I2C Wake-up Status Register                                        */

} I2C_T;




/**
    @addtogroup I2C_CONST I2C Bit Field Definition
    Constant Definitions for I2C Controller
@{ */

/* I2C I2CON Bit Field Definitions */
#define I2C_I2CON_EI_Pos                        7                                       /*!< I2C_T::I2CON: EI Position */
#define I2C_I2CON_EI_Msk                        (1ul << I2C_I2CON_EI_Pos)               /*!< I2C_T::I2CON: EI Mask */

#define I2C_I2CON_ENS1_Pos                      6                                       /*!< I2C_T::I2CON: ENS1 Position */
#define I2C_I2CON_ENS1_Msk                      (1ul << I2C_I2CON_ENS1_Pos)             /*!< I2C_T::I2CON: ENS1 Mask */

#define I2C_I2CON_STA_Pos                       5                                       /*!< I2C_T::I2CON: STA Position */
#define I2C_I2CON_STA_Msk                       (1ul << I2C_I2CON_STA_Pos)              /*!< I2C_T::I2CON: STA Mask */

#define I2C_I2CON_STO_Pos                       4                                       /*!< I2C_T::I2CON: STO Position */
#define I2C_I2CON_STO_Msk                       (1ul << I2C_I2CON_STO_Pos)              /*!< I2C_T::I2CON: STO Mask */

#define I2C_I2CON_SI_Pos                        3                                       /*!< I2C_T::I2CON: SI Position */
#define I2C_I2CON_SI_Msk                        (1ul << I2C_I2CON_SI_Pos)               /*!< I2C_T::I2CON: SI Mask */

#define I2C_I2CON_AA_Pos                        2                                       /*!< I2C_T::I2CON: AA Position */
#define I2C_I2CON_AA_Msk                        (1ul << I2C_I2CON_AA_Pos)               /*!< I2C_T::I2CON: AA Mask */

/* I2C I2CADDR Bit Field Definitions */
#define I2C_I2CADDR_I2CADDR_Pos                 1                                       /*!< I2C_T::I2CADDR1: I2CADDR Position */
#define I2C_I2CADDR_I2CADDR_Msk                 (0x7Ful << I2C_I2CADDR_I2CADDR_Pos)     /*!< I2C_T::I2CADDR1: I2CADDR Mask */

#define I2C_I2CADDR_GC_Pos                      0                                       /*!< I2C_T::I2CADDR1: GC Position */
#define I2C_I2CADDR_GC_Msk                      (1ul << I2C_I2CADDR_GC_Pos)             /*!< I2C_T::I2CADDR1: GC Mask */

/* I2C I2CDAT Bit Field Definitions */
#define I2C_I2CDAT_I2CDAT_Pos                   0                                       /*!< I2C_T::I2CDAT: I2CDAT Position */
#define I2C_I2CDAT_I2CDAT_Msk                   (0xFFul << I2C_I2CDAT_I2CDAT_Pos)       /*!< I2C_T::I2CDAT: I2CDAT Mask */

/* I2C I2CSTATUS Bit Field Definitions */
#define I2C_I2CSTATUS_I2CSTATUS_Pos             0                                       /*!< I2C_T::I2CSTATUS: I2CSTATUS Position */
#define I2C_I2CSTATUS_I2CSTATUS_Msk             (0xFFul << I2C_I2CSTATUS_I2CSTATUS_Pos) /*!< I2C_T::I2CSTATUS: I2CSTATUS Mask */

/* I2C I2CLK Bit Field Definitions */
#define I2C_I2CLK_I2CLK_Pos                     0                                       /*!< I2C_T::I2CLK: I2CLK Position */
#define I2C_I2CLK_I2CLK_Msk                     (0xFFul << I2C_I2CLK_I2CLK_Pos)         /*!< I2C_T::I2CLK: I2CLK Mask */

/* I2C I2CTOC Bit Field Definitions */
#define I2C_I2CTOC_ENTI_Pos                     2                                       /*!< I2C_T::I2CTOC: ENTI Position */
#define I2C_I2CTOC_ENTI_Msk                     (1ul << I2C_I2CTOC_ENTI_Pos)            /*!< I2C_T::I2CTOC: ENTI Mask */

#define I2C_I2CTOC_DIV4_Pos                     1                                       /*!< I2C_T::I2CTOC: DIV4 Position */
#define I2C_I2CTOC_DIV4_Msk                     (1ul << I2C_I2CTOC_DIV4_Pos)            /*!< I2C_T::I2CTOC: DIV4 Mask */

#define I2C_I2CTOC_TIF_Pos                      0                                       /*!< I2C_T::I2CTOC: TIF Position */
#define I2C_I2CTOC_TIF_Msk                      (1ul << I2C_I2CTOC_TIF_Pos)             /*!< I2C_T::I2CTOC: TIF Mask */

/* I2C I2CADM Bit Field Definitions */
#define I2C_I2CADM_I2CADM_Pos                   1                                       /*!< I2C_T::I2CADM0: I2CADM Position */
#define I2C_I2CADM_I2CADM_Msk                   (0x7Ful << I2C_I2CADM_I2CADM_Pos)       /*!< I2C_T::I2CADM0: I2CADM Mask */

/* I2C I2CWKUPCON Bit Field Definitions */
#define I2C_I2CWKUPCON_WKUPEN_Pos               0                                       /*!< I2C_T::I2CWKUPCON: WKUPEN Position */
#define I2C_I2CWKUPCON_WKUPEN_Msk               (1ul << I2C_I2CWKUPCON_WKUPEN_Pos)      /*!< I2C_T::I2CWKUPCON: WKUPEN Mask */

/* I2C I2CWKUPSTS Bit Field Definitions */
#define I2C_I2CWKUPSTS_WKUPIF_Pos               0                                       /*!< I2C_T::I2CWKUPSTS: WKUPIF Position */
#define I2C_I2CWKUPSTS_WKUPIF_Msk               (1ul << I2C_I2CWKUPSTS_WKUPIF_Pos)      /*!< I2C_T::I2CWKUPSTS: WKUPIF Mask */

/*@}*/ /* end of group I2C_CONST */
/*@}*/ /* end of I2C group  */



/*---------------------- Enhanced Input Capture Timer -------------------------*/
/**
    @addtogroup ECAP Enhanced Input Capture Timer(ECAP)
    Memory Mapped Structure for ECAP Controller
@{ */


typedef struct
{




    /**
     * @var ECAP_T::CNT
     * Offset: 0x00  Input Capture Counter (24-bit up counter)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |VAL       |Input Capture Timer/Counter
     * |        |          |The input Capture Timer/Counter is a 24-bit up-counting counter.
     * |        |          |The clock source for the counter is from the clock divider output which the CAP_CLK is software optionally divided by 1,4,16 or 32.
     * @var ECAP_T::HOLD0
     * Offset: 0x04  Input Capture Counter Hold Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |VAL       |Input Capture Counter Hold Register
     * |        |          |When an active input capture channel detects a valid edge signal change, the ECAP_CNT value is latched into the corresponding holding register.
     * |        |          |Each input channel has itself holding register named by ECAP_HOLDx where x is from 0 to 2 to indicate inputs from IC0 to IC2, respectively.
     * @var ECAP_T::HOLD1
     * Offset: 0x08  Input Capture Counter Hold Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |VAL       |Input Capture Counter Hold Register
     * |        |          |When an active input capture channel detects a valid edge signal change, the ECAP_CNT value is latched into the corresponding holding register.
     * |        |          |Each input channel has itself holding register named by ECAP_HOLDx where x is from 0 to 2 to indicate inputs from IC0 to IC2, respectively.
     * @var ECAP_T::HOLD2
     * Offset: 0x0C  Input Capture Counter Hold Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |VAL       |Input Capture Counter Hold Register
     * |        |          |When an active input capture channel detects a valid edge signal change, the ECAP_CNT value is latched into the corresponding holding register.
     * |        |          |Each input channel has itself holding register named by ECAP_HOLDx where x is from 0 to 2 to indicate inputs from IC0 to IC2, respectively.
     * @var ECAP_T::CNTCMP
     * Offset: 0x10  Input Capture Counter Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |VAL       |Input Capture Counter Compare Register
     * |        |          |If the compare function is enabled (CMPEN = 1), the compare register is loaded with the value that the compare function compares the capture counter (ECAP_CNT) with.
     * |        |          |If the reload control is enabled (RLDEN = 1), an overflow event or capture events will trigger the hardware to reload ECAP_CNTCMP into ECAP_CNT.
     * @var ECAP_T::CTL0
     * Offset: 0x14  Input Capture Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |NFDIS     |Noise Filter Clock Pre-Divide Selection
     * |        |          |To determine the sampling frequency of the Noise Filter clock
     * |        |          |00 = CAP_CLK.
     * |        |          |01 = CAP_CLK/2.
     * |        |          |10 = CAP_CLK/4.
     * |        |          |11 = CAP_CLK/16.
     * |[3]     |CAPNF_DIS |Input Capture Noise Filter Disable Control
     * |        |          |0 = Noise filter of Input Capture Enabled.
     * |        |          |1 = Noise filter of Input Capture Disabled.
     * |[4]     |CAPEN0    |Port Pin IC0 Input To Input Capture Unit Enable Control
     * |        |          |0 = IC0 input to Input Capture Unit Disabled.
     * |        |          |1 = IC0 input to Input Capture Unit Enabled.
     * |[5]     |CAPEN1    |Port Pin IC1 Input To Input Capture Unit Enable Control
     * |        |          |0 = IC1 input to Input Capture Unit Disabled.
     * |        |          |1 = IC1 input to Input Capture Unit Enabled.
     * |[6]     |CAPEN2    |Port Pin IC2 Input To Input Capture Unit Enable Control
     * |        |          |0 = IC2 input to Input Capture Unit Disabled.
     * |        |          |1 = IC2 input to Input Capture Unit Enabled.
     * |[9:8]   |CAPSEL0   |CAP0 Input Source Selection
     * |        |          |00 = CAP0 input is from port pin IC0.
     * |        |          |01 = CAP0 input is from signal CPO0 (Analog comparator 0 output).
     * |        |          |10 = Reserved.
     * |        |          |11 = CAP0 input is from signal OPDO0 (OP0 digital output).
     * |[11:10] |CAPSEL1   |CAP1 Input Source Selection
     * |        |          |00 = CAP1 input is from port pin IC1.
     * |        |          |01 = CAP1 input is from signal CPO1 (Analog comparator 1 output).
     * |        |          |10 = Reserved.
     * |        |          |11 = CAP1 input is from signal OPDO1 (OP1 digital output).
     * |[13:12] |CAPSEL2   |CAP2 Input Source Selection
     * |        |          |00 = CAP2 input is from port pin IC2.
     * |        |          |01 = CAP2 input is from signal CPO2 (Analog comparator 2 output).
     * |        |          |10 = Reserved.
     * |        |          |11 = CAP2 input is from signal ADCMPOx (ADC compare output x).
     * |[16]    |CAPIEN0   |Input Capture Channel 0 Interrupt Enable Control
     * |        |          |0 = The flag CAPF0 can trigger Input Capture interrupt Disabled.
     * |        |          |1 = The flag CAPF0 can trigger Input Capture interrupt Enabled.
     * |[17]    |CAPIEN1   |Input Capture Channel 1 Interrupt Enable Control
     * |        |          |0 = The flag CAPF1 can trigger Input Capture interrupt Disabled.
     * |        |          |1 = The flag CAPF1 can trigger Input Capture interrupt Enabled.
     * |[18]    |CAPIEN2   |Input Capture Channel 2 Interrupt Enable Control
     * |        |          |0 = The flag CAPF2 can trigger Input Capture interrupt Disabled.
     * |        |          |1 = The flag CAPF2 can trigger Input Capture interrupt Enabled.
     * |[20]    |OVIEN     |OVF Trigger Input Capture Interrupt Enable Control
     * |        |          |0 = The flag OVUNF can trigger Input Capture interrupt Disabled.
     * |        |          |1 = The flag OVUNF can trigger Input Capture interrupt Enabled.
     * |[21]    |CMPIEN    |CMPF Trigger Input Capture Interrupt Enable Control
     * |        |          |0 = The flag CMPF can trigger Input Capture interrupt Disabled.
     * |        |          |1 = The flag CMPF can trigger Input Capture interrupt Enabled.
     * |[24]    |CNTEN     |Input Capture Counter Start
     * |        |          |Setting this bit to 1, the capture counter (ECAP_CNT) starts up-counting synchronously with capture clock input (CAP_CLK).
     * |        |          |0 = ECAP_CNT stop counting.
     * |        |          |1 = ECAP_CNT starts up-counting.
     * |[25]    |CMPCLR    |Input Capture Counter Cleared By Compare-Match Control
     * |        |          |If this bit is set to 1, the capture counter (ECAP_CNT) will be cleared to 0 when the compare-match event (CAMCMPF = 1) occurs.
     * |        |          |0 = Compare-match event (CAMCMPF) can clear capture counter (ECAP_CNT) Disabled.
     * |        |          |1 = Compare-match event (CAMCMPF) can clear capture counter (ECAP_CNT) Enabled.
     * |[26]    |CPTCLR    |Input Capture Counter Cleared By Capture Events Control
     * |        |          |If this bit is set to 1, the capture counter (ECAP_CNT) will be cleared to zero when any one of capture events (CAPF0~3) occurs.
     * |        |          |0 = Capture events (CAPF0~3) can clear capture counter (ECAP_CNT) Disabled.
     * |        |          |1 = Capture events (CAPF0~3) can clear capture counter (ECAP_CNT) Enabled.
     * |[27]    |RLDEN     |Reload Function Enable Control
     * |        |          |Setting this bit to enable the reload function.
     * |        |          |If the reload control is enabled, an overflow event (OVF) or capture events (CAPFx) will trigger the hardware to reload ECAP_CNTCMP into ECAP_CNT.
     * |        |          |0 = The reload function Disabled.
     * |        |          |1 = The reload function Enabled.
     * |[28]    |CMPEN     |Compare Function Enable Control
     * |        |          |The compare function in input capture timer/counter is to compare the dynamic counting ECAP_CNT with the compare register ECAP_CNTCMP, if ECAP_CNT value reaches ECAP_CNTCMP, the flag CMPF will be set.
     * |        |          |0 = The compare function Disabled.
     * |        |          |1 = The compare function Enabled.
     * |[29]    |CAPEN     |Input Capture Timer/Counter Enable Control
     * |        |          |0 = Input Capture function Disabled.
     * |        |          |1 = Input Capture function Enabled.
     * @var ECAP_T::CTL1
     * Offset: 0x18  Input Capture Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |EDGESEL0  |Channel 0 Captured Edge Selection
     * |        |          |Input capture can detect falling edge change only, rising edge change only or one of both edge change
     * |        |          |00 = Detect rising edge.
     * |        |          |01 = Detect falling edge.
     * |        |          |1x = Detect either rising or falling edge.
     * |[3:2]   |EDGESEL1  |Channel 1 Captured Edge Selection
     * |        |          |Input capture can detect falling edge change only, rising edge change only or one of both edge change
     * |        |          |00 = Detect rising edge.
     * |        |          |01 = Detect falling edge.
     * |        |          |1x = Detect either rising or falling edge.
     * |[5:4]   |EDGESEL2  |Channel 2 Captured Edge Selection
     * |        |          |Input capture can detect falling edge change or rising edge change only, or one of both edge changes.
     * |        |          |00 = Detect rising edge.
     * |        |          |01 = Detect falling edge.
     * |        |          |1x = Detect either rising or falling edge.
     * |[10:8]  |RLDSEL    |ECAP_CNT Reload Trigger Source Selection
     * |        |          |If the reload function is enabled RLDEN (ECAP_CTL0[27]) = 1, when a reload trigger event comes, the ECAP_CNT is reloaded with ECAP_CNTCMP.
     * |        |          |RLDSEL[2:0] determines the ECAP_CNT reload trigger source
     * |        |          |000 = CAPF0.
     * |        |          |001 = CAPF1.
     * |        |          |010 = CAPF2.
     * |        |          |100 = OVF.
     * |        |          |Others = Reserved.
     * |[14:12] |CLKSEL    |Capture Timer Clock Divide Selection
     * |        |          |The capture timer clock has a pre-divider with four divided options controlled by CLKSEL[1:0].
     * |        |          |000 = CAP_CLK/1.
     * |        |          |001 = CAP_CLK/4.
     * |        |          |010 = CAP_CLK/16.
     * |        |          |011 = CAP_CLK/32.
     * |        |          |100 = CAP_CLK/64.
     * |        |          |101 = CAP_CLK/96.
     * |        |          |110 = CAP_CLK/112.
     * |        |          |111 = CAP_CLK/128.
     * |[17:16] |SRCSEL    |Capture Timer/Counter Clock Source Selection
     * |        |          |Select the capture timer/counter clock source.
     * |        |          |00 = CAP_CLK (default).
     * |        |          |01 = CAP0.
     * |        |          |10 = CAP1.
     * |        |          |11 = CAP2.
     * @var ECAP_T::STATUS
     * Offset: 0x1C  Input Capture Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPF0     |Input Capture Channel 0 Captured Flag
     * |        |          |When the input capture channel 0 detects a valid edge change at CAP0 input, it will set flag CAPF0 to high.
     * |        |          |0 = No valid edge change is detected at CAP0 input.
     * |        |          |1 = A valid edge change is detected at CAP0 input.
     * |        |          |Note: This bit is only cleared by writing 1 to it.
     * |[1]     |CAPF1     |Input Capture Channel 1 Captured Flag
     * |        |          |When the input capture channel 1 detects a valid edge change at CAP1 input, it will set flag CAPF1 to high.
     * |        |          |0 = No valid edge change is detected at CAP1 input.
     * |        |          |1 = A valid edge change is detected at CAP1 input.
     * |        |          |Note: This bit is only cleared by writing 1 to it.
     * |[2]     |CAPF2     |Input Capture Channel 2 Captured Flag
     * |        |          |When the input capture channel 2 detects a valid edge change at CAP2 input, it will set flag CAPF2 to high.
     * |        |          |0 = No valid edge change is detected at CAP2 input.
     * |        |          |1 = A valid edge change is detected at CAP2 input.
     * |        |          |Note: This bit is only cleared by writing 1 to it.
     * |[4]     |CMPF      |Input Capture Compare-Match Flag
     * |        |          |If the input capture compare function is enabled, the flag is set by hardware while capture counter (ECAP_CNT) up counts and reach to the ECAP_CNTCMP value.
     * |        |          |0 = ECAP_CNT does not match with ECAP_CNTCMP value.
     * |        |          |1 = ECAP_CNT counts to the same as ECAP_CNTCMP value.
     * |        |          |Note: This bit is only cleared by writing 1 to it.
     * |[5]     |OVF       |Input Capture Counter Overflow Flag
     * |        |          |Flag is set by hardware when input capture up counter (ECAP_CNT) overflows from 0x00FF_FFFF to zero.
     * |        |          |0 = No overflow occurs in ECAP_CNT.
     * |        |          |1 = ECAP_CNT overflows.
     * |        |          |Note: This bit is only cleared by writing 1 to it.
        */

    __IO uint32_t CNT;           /* Offset: 0x00  Input Capture Counter (24-bit up counter)                          */
    __IO uint32_t HOLD0;         /* Offset: 0x04  Input Capture Counter Hold Register 0                              */
    __IO uint32_t HOLD1;         /* Offset: 0x08  Input Capture Counter Hold Register 1                              */
    __IO uint32_t HOLD2;         /* Offset: 0x0C  Input Capture Counter Hold Register 2                              */
    __IO uint32_t CNTCMP;        /* Offset: 0x10  Input Capture Counter Compare Register                             */
    __IO uint32_t CTL0;          /* Offset: 0x14  Input Capture Control Register 0                                   */
    __IO uint32_t CTL1;          /* Offset: 0x18  Input Capture Control Register 1                                   */
    __IO uint32_t STATUS;        /* Offset: 0x1C  Input Capture Status Register                                      */

} ECAP_T;



/**
    @addtogroup ECAP_CONST ECAP Bit Field Definition
    Constant Definitions for ECAP Controller
@{ */

#define ECAP_CNT_VAL_Pos                 (0)                                               /*!< ECAP CNT: VAL Position                 */
#define ECAP_CNT_VAL_Msk                 (0xfffffful << ECAP_CNT_VAL_Pos)                  /*!< ECAP CNT: VAL Mask                     */

#define ECAP_HOLD0_VAL_Pos               (0)                                               /*!< ECAP HOLD0: VAL Position               */
#define ECAP_HOLD0_VAL_Msk               (0xfffffful << ECAP_HOLD0_VAL_Pos)                /*!< ECAP HOLD0: VAL Mask                   */

#define ECAP_HOLD1_VAL_Pos               (0)                                               /*!< ECAP HOLD1: VAL Position               */
#define ECAP_HOLD1_VAL_Msk               (0xfffffful << ECAP_HOLD1_VAL_Pos)                /*!< ECAP HOLD1: VAL Mask                   */

#define ECAP_HOLD2_VAL_Pos               (0)                                               /*!< ECAP HOLD2: VAL Position               */
#define ECAP_HOLD2_VAL_Msk               (0xfffffful << ECAP_HOLD2_VAL_Pos)                /*!< ECAP HOLD2: VAL Mask                   */

#define ECAP_CNTCMP_VAL_Pos              (0)                                               /*!< ECAP CNTCMP: VAL Position              */
#define ECAP_CNTCMP_VAL_Msk              (0xfffffful << ECAP_CNTCMP_VAL_Pos)               /*!< ECAP CNTCMP: VAL Mask                  */

#define ECAP_CTL0_NFDIS_Pos              (0)                                               /*!< ECAP CTL0: NFDIS Position              */
#define ECAP_CTL0_NFDIS_Msk              (0x3ul << ECAP_CTL0_NFDIS_Pos)                    /*!< ECAP CTL0: NFDIS Mask                  */

#define ECAP_CTL0_CAPNF_DIS_Pos          (3)                                               /*!< ECAP CTL0: CAPNF_DIS Position          */
#define ECAP_CTL0_CAPNF_DIS_Msk          (0x1ul << ECAP_CTL0_CAPNF_DIS_Pos)                /*!< ECAP CTL0: CAPNF_DIS Mask              */

#define ECAP_CTL0_CAPEN0_Pos             (4)                                               /*!< ECAP CTL0: CAPEN0 Position             */
#define ECAP_CTL0_CAPEN0_Msk             (0x1ul << ECAP_CTL0_CAPEN0_Pos)                   /*!< ECAP CTL0: CAPEN0 Mask                 */

#define ECAP_CTL0_CAPEN1_Pos             (5)                                               /*!< ECAP CTL0: CAPEN1 Position             */
#define ECAP_CTL0_CAPEN1_Msk             (0x1ul << ECAP_CTL0_CAPEN1_Pos)                   /*!< ECAP CTL0: CAPEN1 Mask                 */

#define ECAP_CTL0_CAPEN2_Pos             (6)                                               /*!< ECAP CTL0: CAPEN2 Position             */
#define ECAP_CTL0_CAPEN2_Msk             (0x1ul << ECAP_CTL0_CAPEN2_Pos)                   /*!< ECAP CTL0: CAPEN2 Mask                 */

#define ECAP_CTL0_CAPSEL0_Pos            (8)                                               /*!< ECAP CTL0: CAPSEL0 Position            */
#define ECAP_CTL0_CAPSEL0_Msk            (0x3ul << ECAP_CTL0_CAPSEL0_Pos)                  /*!< ECAP CTL0: CAPSEL0 Mask                */

#define ECAP_CTL0_CAPSEL1_Pos            (10)                                              /*!< ECAP CTL0: CAPSEL1 Position            */
#define ECAP_CTL0_CAPSEL1_Msk            (0x3ul << ECAP_CTL0_CAPSEL1_Pos)                  /*!< ECAP CTL0: CAPSEL1 Mask                */

#define ECAP_CTL0_CAPSEL2_Pos            (12)                                              /*!< ECAP CTL0: CAPSEL2 Position            */
#define ECAP_CTL0_CAPSEL2_Msk            (0x3ul << ECAP_CTL0_CAPSEL2_Pos)                  /*!< ECAP CTL0: CAPSEL2 Mask                */

#define ECAP_CTL0_CAPIEN0_Pos            (16)                                              /*!< ECAP CTL0: CAPIEN0 Position            */
#define ECAP_CTL0_CAPIEN0_Msk            (0x1ul << ECAP_CTL0_CAPIEN0_Pos)                  /*!< ECAP CTL0: CAPIEN0 Mask                */

#define ECAP_CTL0_CAPIEN1_Pos            (17)                                              /*!< ECAP CTL0: CAPIEN1 Position            */
#define ECAP_CTL0_CAPIEN1_Msk            (0x1ul << ECAP_CTL0_CAPIEN1_Pos)                  /*!< ECAP CTL0: CAPIEN1 Mask                */

#define ECAP_CTL0_CAPIEN2_Pos            (18)                                              /*!< ECAP CTL0: CAPIEN2 Position            */
#define ECAP_CTL0_CAPIEN2_Msk            (0x1ul << ECAP_CTL0_CAPIEN2_Pos)                  /*!< ECAP CTL0: CAPIEN2 Mask                */

#define ECAP_CTL0_OVIEN_Pos              (20)                                              /*!< ECAP CTL0: OVIEN Position              */
#define ECAP_CTL0_OVIEN_Msk              (0x1ul << ECAP_CTL0_OVIEN_Pos)                    /*!< ECAP CTL0: OVIEN Mask                  */

#define ECAP_CTL0_CMPIEN_Pos             (21)                                              /*!< ECAP CTL0: CMPIEN Position             */
#define ECAP_CTL0_CMPIEN_Msk             (0x1ul << ECAP_CTL0_CMPIEN_Pos)                   /*!< ECAP CTL0: CMPIEN Mask                 */

#define ECAP_CTL0_CNTEN_Pos              (24)                                              /*!< ECAP CTL0: CNTEN Position              */
#define ECAP_CTL0_CNTEN_Msk              (0x1ul << ECAP_CTL0_CNTEN_Pos)                    /*!< ECAP CTL0: CNTEN Mask                  */

#define ECAP_CTL0_CMPCLR_Pos             (25)                                              /*!< ECAP CTL0: CMPCLR Position             */
#define ECAP_CTL0_CMPCLR_Msk             (0x1ul << ECAP_CTL0_CMPCLR_Pos)                   /*!< ECAP CTL0: CMPCLR Mask                 */

#define ECAP_CTL0_CPTCLR_Pos             (26)                                              /*!< ECAP CTL0: CPTCLR Position             */
#define ECAP_CTL0_CPTCLR_Msk             (0x1ul << ECAP_CTL0_CPTCLR_Pos)                   /*!< ECAP CTL0: CPTCLR Mask                 */

#define ECAP_CTL0_RLDEN_Pos              (27)                                              /*!< ECAP CTL0: RLDEN Position              */
#define ECAP_CTL0_RLDEN_Msk              (0x1ul << ECAP_CTL0_RLDEN_Pos)                    /*!< ECAP CTL0: RLDEN Mask                  */

#define ECAP_CTL0_CMPEN_Pos              (28)                                              /*!< ECAP CTL0: CMPEN Position              */
#define ECAP_CTL0_CMPEN_Msk              (0x1ul << ECAP_CTL0_CMPEN_Pos)                    /*!< ECAP CTL0: CMPEN Mask                  */

#define ECAP_CTL0_CAPEN_Pos              (29)                                              /*!< ECAP CTL0: CAPEN Position              */
#define ECAP_CTL0_CAPEN_Msk              (0x1ul << ECAP_CTL0_CAPEN_Pos)                    /*!< ECAP CTL0: CAPEN Mask                  */

#define ECAP_CTL1_EDGESEL0_Pos           (0)                                               /*!< ECAP CTL1: EDGESEL0 Position           */
#define ECAP_CTL1_EDGESEL0_Msk           (0x3ul << ECAP_CTL1_EDGESEL0_Pos)                 /*!< ECAP CTL1: EDGESEL0 Mask               */

#define ECAP_CTL1_EDGESEL1_Pos           (2)                                               /*!< ECAP CTL1: EDGESEL1 Position           */
#define ECAP_CTL1_EDGESEL1_Msk           (0x3ul << ECAP_CTL1_EDGESEL1_Pos)                 /*!< ECAP CTL1: EDGESEL1 Mask               */

#define ECAP_CTL1_EDGESEL2_Pos           (4)                                               /*!< ECAP CTL1: EDGESEL2 Position           */
#define ECAP_CTL1_EDGESEL2_Msk           (0x3ul << ECAP_CTL1_EDGESEL2_Pos)                 /*!< ECAP CTL1: EDGESEL2 Mask               */

#define ECAP_CTL1_RLDSEL_Pos             (8)                                               /*!< ECAP CTL1: RLDSEL Position             */
#define ECAP_CTL1_RLDSEL_Msk             (0x7ul << ECAP_CTL1_RLDSEL_Pos)                   /*!< ECAP CTL1: RLDSEL Mask                 */

#define ECAP_CTL1_CLKSEL_Pos             (12)                                              /*!< ECAP CTL1: CLKSEL Position             */
#define ECAP_CTL1_CLKSEL_Msk             (0x7ul << ECAP_CTL1_CLKSEL_Pos)                   /*!< ECAP CTL1: CLKSEL Mask                 */

#define ECAP_CTL1_SRCSEL_Pos             (16)                                              /*!< ECAP CTL1: SRCSEL Position             */
#define ECAP_CTL1_SRCSEL_Msk             (0x3ul << ECAP_CTL1_SRCSEL_Pos)                   /*!< ECAP CTL1: SRCSEL Mask                 */

#define ECAP_STATUS_CAPF0_Pos            (0)                                               /*!< ECAP STATUS: CAPF0 Position            */
#define ECAP_STATUS_CAPF0_Msk            (0x1ul << ECAP_STATUS_CAPF0_Pos)                  /*!< ECAP STATUS: CAPF0 Mask                */

#define ECAP_STATUS_CAPF1_Pos            (1)                                               /*!< ECAP STATUS: CAPF1 Position            */
#define ECAP_STATUS_CAPF1_Msk            (0x1ul << ECAP_STATUS_CAPF1_Pos)                  /*!< ECAP STATUS: CAPF1 Mask                */

#define ECAP_STATUS_CAPF2_Pos            (2)                                               /*!< ECAP STATUS: CAPF2 Position            */
#define ECAP_STATUS_CAPF2_Msk            (0x1ul << ECAP_STATUS_CAPF2_Pos)                  /*!< ECAP STATUS: CAPF2 Mask                */

#define ECAP_STATUS_CMPF_Pos             (4)                                               /*!< ECAP STATUS: CMPF Position             */
#define ECAP_STATUS_CMPF_Msk             (0x1ul << ECAP_STATUS_CMPF_Pos)                   /*!< ECAP STATUS: CMPF Mask                 */

#define ECAP_STATUS_OVF_Pos              (5)                                               /*!< ECAP STATUS: OVF Position              */
#define ECAP_STATUS_OVF_Msk              (0x1ul << ECAP_STATUS_OVF_Pos)                    /*!< ECAP STATUS: OVF Mask                  */

/**@}*/ /* ECAP_CONST */
/**@}*/ /* end of ECAP register group */


/*---------------------- OP Amplifier -------------------------*/
/**
    @addtogroup OPA OP Amplifier(OPA)
    Memory Mapped Structure for OPA Controller
@{ */



typedef struct
{




    /**
     * @var OPA_T::CR
     * Offset: 0x00  OP Amplifier Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |OP0_EN    |OP Amplifier 0 Enable
     * |        |          |0 = OP Amplifier 0 Disabled.
     * |        |          |1 = OP Amplifier 0 Enabled.
     * |        |          |Note: The OP Amplifier 0 output needs to wait 20us stable time after OP0_EN is set.
     * |[1]     |OP1_EN    |OP Amplifier 1 Enable
     * |        |          |0 = OP Amplifier 1 Disabled.
     * |        |          |1 = OP Amplifier 1 Enabled.
     * |        |          |Note: The OP Amplifier 1 output needs to wait 20us stable time after OP1_EN is set.
     * |[4]     |OPSCH0_EN |OP Amplifier 0 Schmitt Trigger Non-Inverting Buffer Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[5]     |OPSCH1_EN |OP Amplifier 1 Schmitt Trigger Non-Inverting Buffer Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[8]     |OPDIE0    |OP Amplifier 0 Schmitt Trigger Digital Output Interrupt Enable
     * |        |          |The OPDF0 interrupt flag is set by hardware whenever the OP amplifier 0 Schmitt trigger
     * |        |          |non-inverting buffer digital output changes state, in the meanwhile, if OPDIE0 is set to 1,
     * |        |          |a comparator interrupt request is generated.
     * |        |          |0 = OP Amplifier 0 digital output interrupt function Disabled.
     * |        |          |1 = OP Amplifier 0 digital output interrupt function Enabled.
     * |[9]     |OPDIE1    |OP Amplifier 1 Schmitt Trigger Digital Output Interrupt Enable
     * |        |          |The OPDF1 interrupt flag is set by hardware whenever the OP amplifier 1 Schmitt trigger
     * |        |          |non-inverting buffer digital output changes state, in the meanwhile, if OPDIE1 is set to 1,
     * |        |          |a comparator interrupt request is generated.
     * |        |          |0 = OP Amplifier 1 digital output interrupt function Disabled.
     * |        |          |1 = OP Amplifier 1 digital output interrupt function Enabled.
     * @var OPA_T::SR
     * Offset: 0x04  OP Amplifier Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |OPDO0     |OP Amplifier 0 Digital Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the Schmitt trigger buffer is disabled (OPSCH0_EN = 0).
     * |[1]     |OPDO1     |OP Amplifier 1 Digital Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the Schmitt trigger buffer is disabled (OPSCH1_EN = 0).
     * |[4]     |OPDF0     |OP Amplifier 0 Schmitt Trigger Digital Output Interrupt Flag
     * |        |          |OPDF0 interrupt flag is set by hardware whenever the OP amplifier 0 Schmitt trigger
     * |        |          |non-inverting buffer digital output changes state.
     * |        |          |This bit is cleared by writing 1 to itself.
     * |[5]     |OPDF1     |OP Amplifier 1 Schmitt Trigger Digital Output Interrupt Flag
     * |        |          |OPDF1 interrupt flag is set by hardware whenever the OP amplifier 1 Schmitt trigger
     * |        |          |non-inverting buffer digital output changes state.
     * |        |          |This bit is cleared by writing 1 to itself.
     */

    __IO uint32_t CR;            /* Offset: 0x00  OP Amplifier Control Register                                      */
    __IO uint32_t SR;            /* Offset: 0x04  OP Amplifier Status Register                                       */

} OPA_T;



/**
    @addtogroup OPA_CONST OPA Bit Field Definition
    Constant Definitions for OPA Controller
@{ */

/* CR Bit Field Definitions */
#define OPA_CR_OP0_EN_Pos             (0)                                            /*!< OPA_T::CR: OP0_EN Position             */
#define OPA_CR_OP0_EN_Msk             (0x1ul << OPA_CR_OP0_EN_Pos)                   /*!< OPA_T::CR: OP0_EN Mask                 */

#define OPA_CR_OP1_EN_Pos             (1)                                            /*!< OPA_T::CR: OP1_EN Position             */
#define OPA_CR_OP1_EN_Msk             (0x1ul << OPA_CR_OP1_EN_Pos)                   /*!< OPA_T::CR: OP1_EN Mask                 */

#define OPA_CR_OPSCH0_EN_Pos          (4)                                            /*!< OPA_T::CR: OPSCH0_EN Position          */
#define OPA_CR_OPSCH0_EN_Msk          (0x1ul << OPA_CR_OPSCH0_EN_Pos)                /*!< OPA_T::CR: OPSCH0_EN Mask              */

#define OPA_CR_OPSCH1_EN_Pos          (5)                                            /*!< OPA_T::CR: OPSCH1_EN Position          */
#define OPA_CR_OPSCH1_EN_Msk          (0x1ul << OPA_CR_OPSCH1_EN_Pos)                /*!< OPA_T::CR: OPSCH1_EN Mask              */

#define OPA_CR_OPDIE0_Pos             (8)                                            /*!< OPA_T::CR: OPDIE0 Position             */
#define OPA_CR_OPDIE0_Msk             (0x1ul << OPA_CR_OPDIE0_Pos)                   /*!< OPA_T::CR: OPDIE0 Mask                 */

#define OPA_CR_OPDIE1_Pos             (9)                                            /*!< OPA_T::CR: OPDIE1 Position             */
#define OPA_CR_OPDIE1_Msk             (0x1ul << OPA_CR_OPDIE1_Pos)                   /*!< OPA_T::CR: OPDIE1 Mask                 */

/* SR Bit Field Definitions */
#define OPA_SR_OPDO0_Pos              (0)                                            /*!< OPA_T::SR: OPDO0 Position              */
#define OPA_SR_OPDO0_Msk              (0x1ul << OPA_SR_OPDO0_Pos)                    /*!< OPA_T::SR: OPDO0 Mask                  */

#define OPA_SR_OPDO1_Pos              (1)                                            /*!< OPA_T::SR: OPDO1 Position              */
#define OPA_SR_OPDO1_Msk              (0x1ul << OPA_SR_OPDO1_Pos)                    /*!< OPA_T::SR: OPDO1 Mask                  */

#define OPA_SR_OPDF0_Pos              (4)                                            /*!< OPA_T::SR: OPDF0 Position              */
#define OPA_SR_OPDF0_Msk              (0x1ul << OPA_SR_OPDF0_Pos)                    /*!< OPA_T::SR: OPDF0 Mask                  */

#define OPA_SR_OPDF1_Pos              (5)                                            /*!< OPA_T::SR: OPDF1 Position              */
#define OPA_SR_OPDF1_Msk              (0x1ul << OPA_SR_OPDF1_Pos)                    /*!< OPA_T::SR: OPDF1 Mask                  */

/**@}*/ /* end of group OPA_CONST */
/**@}*/ /* end of OPA register group */




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
     * |[0]     |GO_BUSY   |SPI Transfer Control Bit and Busy Status
     * |        |          |If FIFO mode is disabled, during the data transfer, this bit keeps the value of 1.
     * |        |          |As the transfer is finished, this bit will be cleared automatically.
     * |        |          |Software can read this bit to check if the SPI is in busy status.
     * |        |          |In FIFO mode, this bit will be controlled by hardware.
     * |        |          |Software should not modify this bit.
     * |        |          |In Slave mode, this bit always returns 1 when this register is read by software.
     * |        |          |In Master mode, this bit reflects the busy or idle status of SPI.
     * |        |          |0 = Data transfer stopped.
     * |        |          |1 = In Master mode, writing 1 to this bit to start the SPI data transfer; in Slave mode,
     * |        |          |    writing 1 to this bit indicates that the slave is ready to communicate with a master.
     * |        |          |Note:
     * |        |          |When FIFO mode is disabled, all configurations should be set before writing 1 to this GO_BUSY bit.
     * |[1]     |RX_NEG    |Receive on Negative Edge
     * |        |          |0 = Received data input signal is latched on the rising edge of SPI bus clock.
     * |        |          |1 = Received data input signal is latched on the falling edge of SPI bus clock.
     * |[2]     |TX_NEG    |Transmit on Negative Edge
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
     * |        |          |0 = The MSB, which bit of transmit/receive register depends on the setting of TX_BIT_LEN, is transmitted/received first.
     * |        |          |1 = The LSB, bit 0 of the SPI TX register, is sent first to the SPI data output pin, and the first bit received from
     * |        |          |    the SPI data input pin will be put in the LSB position of the RX register (bit 0 of SPI_RX).
     * |[11]    |CLKP      |Clock Polarity
     * |        |          |0 = SPI bus clock is idle low.
     * |        |          |1 = SPI bus clock is idle high.
     * |[15:12] |SP_CYCLE  |Suspend Interval (Master Only)
     * |        |          |The four bits provide configurable suspend interval between two successive transmit/receive transaction in a transfer.
     * |        |          |The definition of the suspend interval is the interval between the last clock edge of the preceding transaction word
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
     * |        |          |2. In Slave mode with level-trigger configuration, the slave select pin must be kept at active state during the
     * |        |          |   byte suspend interval.
     * |[21]    |FIFO      |FIFO Mode Enable
     * |        |          |0 = FIFO mode Disabled.
     * |        |          |1 = FIFO mode Enabled.
     * |        |          |Note:
     * |        |          |1. Before enabling FIFO mode, the other related settings should be set in advance.
     * |        |          |2. In Master mode, if the FIFO mode is enabled, the GO_BUSY bit will be set to 1 automatically after writing data
     * |        |          |   to the transmit FIFO buffer; the GO_BUSY bit will be cleared to 0 automatically when the SPI controller is in idle.
     * |        |          |   If all data stored at transmit FIFO buffer are sent out, the TX_EMPTY bit will be set to 1 and the GO_BUSY bit will be cleared to 0.
     * |        |          |3. After clearing this bit to 0, user must wait for at least 2 peripheral clock periods before setting this bit to 1 again.
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
     * |        |          |The value in this field is the frequency divider for generating the SPI peripheral clock and the SPI bus clock of SPI master.
     * |        |          |The frequency is obtained according to the following equation.
     * |        |          |   SPI peripheral clock frequency = SPI peripheral clock source frequency / (DIVIDER + 1) / 2
     * |        |          |The SPI peripheral clock source is defined in the CLKSEL1 register.
     * @var SPI_T::SSR
     * Offset: 0x08  Slave Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SSR       |Slave Select Control Bit (Master Only)
     * |        |          |If AUTOSS bit is cleared, writing 1 to this bit sets the proper SPIn_SPISS
     * |        |          |line to an active state and writing 0 sets the line back to inactive state.
     * |        |          |If the AUTOSS bit is set, writing 0 to this bit will keep the SPIn_SPISS
     * |        |          |line at inactive state; writing 1 to this bit will select SPIn_SPISS
     * |        |          |line to be automatically driven to active state for the duration of the
     * |        |          |transmit/receive, and will be driven to inactive state for the rest of the time.
     * |        |          |The active state of SPIn_SPISS is specified in SS_LVL.
     * |[2]     |SS_LVL    |Slave Select Active Level
     * |        |          |This bit defines the active status of slave select signal (SPIn_SPISS).
     * |        |          |0 = The slave select signal SPIn_SPISS is active on low-level/falling-edge.
     * |        |          |1 = The slave select signal SPIn_SPISS is active on high-level/rising-edge.
     * |[3]     |AUTOSS    |Automatic Slave Select Function Enable (Master Only)
     * |        |          |0 = If this bit is cleared, slave select signals will be asserted/de-asserted by
     * |        |          |    setting /clearing SPI_SSR[0].
     * |        |          |1 = If this bit is set, SPIn_SPISS signal will be generated automatically.
     * |        |          |    It means that device/slave select signal will be asserted by the SPI controller
     * |        |          |    when transmit/receive is started, and will be de-asserted after each transmit/receive is finished.
     * |[4]     |SS_LTRIG  |Slave Select Level Trigger Enable (Slave Only)
     * |        |          |0 = Slave select signal is edge-trigger.
     * |        |          |    This is the default value.
     * |        |          |    The SS_LVL bit decides the signal is active after a falling-edge or rising-edge.
     * |        |          |1 = Slave select signal is level-trigger.
     * |        |          |    The SS_LVL bit decides the signal is active low or active high.
     * |[5]     |LTRIG_FLAG|Level Trigger Accomplish Flag
     * |        |          |In Slave mode, this bit indicates whether the received bit number meets the requirement or not after the current transaction done.
     * |        |          |0 = Transferred bit length of one transaction does not meet the specified requirement.
     * |        |          |1 = Transferred bit length meets the specified requirement which defined in TX_BIT_LEN.
     * |        |          |Note: This bit is READ only.
     * |        |          |As the GO_BUSY bit is set to 1 by software, the LTRIG_FLAG will be cleared to 0 after 4 SPI peripheral clock periods plus 1 system clock period.
     * |        |          |In FIFO mode, this bit has no meaning.
     * @var SPI_T::RX
     * Offset: 0x10  Data Receive Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register
     * |        |          |The data receive register holds the datum received from SPI data input pin.
     * |        |          |If FIFO mode is disabled, the last received data can be accessed through software by reading this register.
     * |        |          |If the FIFO bit is set as 1 and the RX_EMPTY bit, SPI_CNTRL[24] or SPI_STATUS[24], is not set to 1, the receive
     * |        |          |FIFO buffer can be accessed through software by reading this register. This is a read-only register.
     * @var SPI_T::TX
     * Offset: 0x20  Data Transmit Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The data transmit registers hold the data to be transmitted in the next transfer.
     * |        |          |The number of valid bits depends on the setting of transmit bit length field of the SPI_CNTRL register.
     * |        |          |For example, if TX_BIT_LEN is set to 0x08, the bits TX[7:0] will be transmitted in next transfer.
     * |        |          |If TX_BIT_LEN is set to 0x00, the SPI controller will perform a 32-bit transfer.
     * |        |          |Note 1: When the SPI controller is configured as a slave device and FIFO mode is disabled, if the SPI
     * |        |          |        controller attempts to transmit data to a master, the transmit data register should be updated
     * |        |          |        by software before setting the GO_BUSY bit to 1.
     * |        |          |Note 2: In Master mode, SPI controller will start to transfer after 5 peripheral clock cycles after user writes to this register.
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
     * |        |          |In normal operation, there is an interrupt event when the received data meet the required bits which defined in TX_BIT_LEN.
     * |        |          |If the received bits are less than the requirement and there is no more SPI clock input over the one transfer time in
     * |        |          |Slave 3-wire mode, the user can set this bit to force the current transfer done and then the user can get a transfer done interrupt event.
     * |        |          |Note: This bit will be cleared to 0 automatically by hardware after it is set to 1 by software.
     * |[10]    |SSTA_INTEN|Slave 3-Wire Mode Start Interrupt Enable
     * |        |          |Used to enable interrupt when the transfer has started in Slave 3-wire mode.
     * |        |          |If there is no transfer done interrupt over the time period which is defined by user after the transfer start,
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
     * |[16]    |SS_INT_OPT|Slave Select Inactive Interrupt Option
     * |        |          |This setting is only available if the SPI controller is configured as level trigger slave device.
     * |        |          |0 = As the slave select signal goes to inactive level, the IF bit will NOT be set to 1.
     * |        |          |1 = As the slave select signal goes to inactive level, the IF bit will be set to 1.
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
     * |        |          |0 = The valid data count within the Rx FIFO buffer is smaller than or equal to the setting value of RX_THRESHOLD.
     * |        |          |1 = The valid data count within the receive FIFO buffer is larger than the setting value of RX_THRESHOLD.
     * |        |          |Note: If RX_INTEN = 1 and RX_INTSTS = 1, the SPI controller will generate a SPI interrupt request.
     * |[2]     |RX_OVERRUN|Receive FIFO Overrun Status
     * |        |          |When the receive FIFO buffer is full, the follow-up data will be dropped and this bit will be set to 1.
     * |        |          |Note: This bit will be cleared by writing 1 to itself.
     * |[4]     |TX_INTSTS |Transmit FIFO Threshold Interrupt Status (Read Only)
     * |        |          |0 = The valid data count within the transmit FIFO buffer is larger than the setting value of TX_THRESHOLD.
     * |        |          |1 = The valid data count within the transmit FIFO buffer is less than or equal to the setting value of TX_THRESHOLD.
     * |        |          |Note: If TX_INTEN = 1 and TX_INTSTS = 1, the SPI controller will generate a SPI interrupt request.
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
     * |        |          |1 = Receive FIFO buffer is not empty and no read operation on receive FIFO buffer over 64 SPI clock
     * |        |          |period in Master mode or over 576 SPI peripheral clock period in Slave mode.
     * |        |          |When the received FIFO buffer is read by software, the time-out status will be cleared automatically.
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
    __I  uint32_t RESERVE1[3];
    __O  uint32_t TX;            /* Offset: 0x20  Data Transmit Register                                             */
    __I  uint32_t RESERVE2[6];
    __IO uint32_t CNTRL2;        /* Offset: 0x3C  Control and Status Register 2                                      */
    __IO uint32_t FIFO_CTL;      /* Offset: 0x40  SPI FIFO Control Register                                          */
    __IO uint32_t STATUS;        /* Offset: 0x44  SPI Status Register                                                */

} SPI_T;



/**
    @addtogroup SPI_CONST SPI Bit Field Definition
    Constant Definitions for SPI Controller
@{ */

/* CNTRL Bit Field Definitions */
#define SPI_CNTRL_TX_FULL_Pos      27                                     /*!< SPI_T::CNTRL: TX_FULL Position */
#define SPI_CNTRL_TX_FULL_Msk      (1ul << SPI_CNTRL_TX_FULL_Pos)         /*!< SPI_T::CNTRL: TX_FULL Mask     */

#define SPI_CNTRL_TX_EMPTY_Pos     26                                     /*!< SPI_T::CNTRL: TX_EMPTY Position */
#define SPI_CNTRL_TX_EMPTY_Msk     (1ul << SPI_CNTRL_TX_EMPTY_Pos)        /*!< SPI_T::CNTRL: TX_EMPTY Mask     */

#define SPI_CNTRL_RX_FULL_Pos      25                                     /*!< SPI_T::CNTRL: RX_FULL Position */
#define SPI_CNTRL_RX_FULL_Msk      (1ul << SPI_CNTRL_RX_FULL_Pos)         /*!< SPI_T::CNTRL: RX_FULL Mask     */

#define SPI_CNTRL_RX_EMPTY_Pos     24                                     /*!< SPI_T::CNTRL: RX_EMPTY Position */
#define SPI_CNTRL_RX_EMPTY_Msk     (1ul << SPI_CNTRL_RX_EMPTY_Pos)        /*!< SPI_T::CNTRL: RX_EMPTY Mask     */

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

/* DIVIDER Bit Field Definitions */
#define SPI_DIVIDER_DIVIDER_Pos    0                                      /*!< SPI_T::DIVIDER: DIVIDER Position */
#define SPI_DIVIDER_DIVIDER_Msk    (0xFFul << SPI_DIVIDER_DIVIDER_Pos)    /*!< SPI_T::DIVIDER: DIVIDER Mask */

/* SSR Bit Field Definitions */
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

/* CNTRL2 Bit Field Definitions */
#define SPI_CNTRL2_SS_INT_OPT_Pos         16                                         /*!< SPI_T::CNTRL2: SS_INT_OPT Position */
#define SPI_CNTRL2_SS_INT_OPT_Msk         (1ul << SPI_CNTRL2_SS_INT_OPT_Pos)         /*!< SPI_T::CNTRL2: SS_INT_OPT Mask */

#define SPI_CNTRL2_SLV_START_INTSTS_Pos   11                                         /*!< SPI_T::CNTRL2: SLV_START_INTSTS Position */
#define SPI_CNTRL2_SLV_START_INTSTS_Msk   (1ul << SPI_CNTRL2_SLV_START_INTSTS_Pos)   /*!< SPI_T::CNTRL2: SLV_START_INTSTS Mask */

#define SPI_CNTRL2_SSTA_INTEN_Pos         10                                         /*!< SPI_T::CNTRL2: SSTA_INTEN Position */
#define SPI_CNTRL2_SSTA_INTEN_Msk         (1ul << SPI_CNTRL2_SSTA_INTEN_Pos)         /*!< SPI_T::CNTRL2: SSTA_INTEN Mask */

#define SPI_CNTRL2_SLV_ABORT_Pos          9                                          /*!< SPI_T::CNTRL2: SLV_ABORT Position */
#define SPI_CNTRL2_SLV_ABORT_Msk          (1ul << SPI_CNTRL2_SLV_ABORT_Pos)          /*!< SPI_T::CNTRL2: SLV_ABORT Mask */

#define SPI_CNTRL2_NOSLVSEL_Pos           8                                          /*!< SPI_T::CNTRL2: NOSLVSEL Position */
#define SPI_CNTRL2_NOSLVSEL_Msk           (1ul << SPI_CNTRL2_NOSLVSEL_Pos)           /*!< SPI_T::CNTRL2: NOSLVSEL Mask */

/* FIFO_CTL Bit Field Definitions */
#define SPI_FIFO_CTL_TX_THRESHOLD_Pos     28                                         /*!< SPI_T::FIFO_CTL: TX_THRESHOLD Position */
#define SPI_FIFO_CTL_TX_THRESHOLD_Msk     (7ul << SPI_FIFO_CTL_TX_THRESHOLD_Pos)     /*!< SPI_T::FIFO_CTL: TX_THRESHOLD Mask */

#define SPI_FIFO_CTL_RX_THRESHOLD_Pos     24                                         /*!< SPI_T::FIFO_CTL: RX_THRESHOLD Position */
#define SPI_FIFO_CTL_RX_THRESHOLD_Msk     (7ul << SPI_FIFO_CTL_RX_THRESHOLD_Pos)     /*!< SPI_T::FIFO_CTL: RX_THRESHOLD Mask */

#define SPI_FIFO_CTL_TIMEOUT_INTEN_Pos    21                                         /*!< SPI_T::FIFO_CTL: TIMEOUT_INTEN Position */
#define SPI_FIFO_CTL_TIMEOUT_INTEN_Msk    (1ul << SPI_FIFO_CTL_TIMEOUT_INTEN_Pos)    /*!< SPI_T::FIFO_CTL: TIMEOUT_INTEN Mask */

#define SPI_FIFO_CTL_RXOV_INTEN_Pos       6                                          /*!< SPI_T::FIFO_CTL: RXOV_INTEN Position */
#define SPI_FIFO_CTL_RXOV_INTEN_Msk       (1ul << SPI_FIFO_CTL_RXOV_INTEN_Pos)       /*!< SPI_T::FIFO_CTL: RXOV_INTEN Mask */

#define SPI_FIFO_CTL_TX_INTEN_Pos         3                                          /*!< SPI_T::FIFO_CTL: TX_INTEN Position */
#define SPI_FIFO_CTL_TX_INTEN_Msk         (1ul << SPI_FIFO_CTL_TX_INTEN_Pos)         /*!< SPI_T::FIFO_CTL: TX_INTEN Mask */

#define SPI_FIFO_CTL_RX_INTEN_Pos         2                                          /*!< SPI_T::FIFO_CTL: RX_INTEN Position */
#define SPI_FIFO_CTL_RX_INTEN_Msk         (1ul << SPI_FIFO_CTL_RX_INTEN_Pos)         /*!< SPI_T::FIFO_CTL: RX_INTEN Mask */

#define SPI_FIFO_CTL_TX_CLR_Pos           1                                          /*!< SPI_T::FIFO_CTL: TX_CLR Position */
#define SPI_FIFO_CTL_TX_CLR_Msk           (1ul << SPI_FIFO_CTL_TX_CLR_Pos)           /*!< SPI_T::FIFO_CTL: TX_CLR Mask */

#define SPI_FIFO_CTL_RX_CLR_Pos           0                                          /*!< SPI_T::FIFO_CTL: RX_CLR Position */
#define SPI_FIFO_CTL_RX_CLR_Msk           (1ul << SPI_FIFO_CTL_RX_CLR_Pos)           /*!< SPI_T::FIFO_CTL: RX_CLR Mask */

/* STATUS Bit Field Definitions */
#define SPI_STATUS_TX_FIFO_COUNT_Pos      28                                         /*!< SPI_T::STATUS: TX_FIFO_COUNT Position */
#define SPI_STATUS_TX_FIFO_COUNT_Msk      (0xFul << SPI_STATUS_TX_FIFO_COUNT_Pos)    /*!< SPI_T::STATUS: TX_FIFO_COUNT Mask */

#define SPI_STATUS_TX_FULL_Pos            27                                         /*!< SPI_T::STATUS: TX_FULL Position */
#define SPI_STATUS_TX_FULL_Msk            (1ul << SPI_STATUS_TX_FULL_Pos)            /*!< SPI_T::STATUS: TX_FULL Mask */

#define SPI_STATUS_TX_EMPTY_Pos           26                                         /*!< SPI_T::STATUS: TX_EMPTY Position */
#define SPI_STATUS_TX_EMPTY_Msk           (1ul << SPI_STATUS_TX_EMPTY_Pos)           /*!< SPI_T::STATUS: TX_EMPTY Mask */

#define SPI_STATUS_RX_FULL_Pos            25                                         /*!< SPI_T::STATUS: RX_FULL Position */
#define SPI_STATUS_RX_FULL_Msk            (1ul << SPI_STATUS_RX_FULL_Pos)            /*!< SPI_T::STATUS: RX_FULL Mask */

#define SPI_STATUS_RX_EMPTY_Pos           24                                         /*!< SPI_T::STATUS: RX_EMPTY Position */
#define SPI_STATUS_RX_EMPTY_Msk           (1ul << SPI_STATUS_RX_EMPTY_Pos)           /*!< SPI_T::STATUS: RX_EMPTY Mask */

#define SPI_STATUS_TIMEOUT_Pos            20                                         /*!< SPI_T::STATUS: TIMEOUT Position */
#define SPI_STATUS_TIMEOUT_Msk            (1ul << SPI_STATUS_TIMEOUT_Pos)            /*!< SPI_T::STATUS: TIMEOUT Mask */

#define SPI_STATUS_IF_Pos                 16                                         /*!< SPI_T::STATUS: IF Position */
#define SPI_STATUS_IF_Msk                 (1ul << SPI_STATUS_IF_Pos)                 /*!< SPI_T::STATUS: IF Mask     */

#define SPI_STATUS_RX_FIFO_COUNT_Pos      12                                         /*!< SPI_T::STATUS: RX_FIFO_COUNT Position */
#define SPI_STATUS_RX_FIFO_COUNT_Msk      (0xFul << SPI_STATUS_RX_FIFO_COUNT_Pos)    /*!< SPI_T::STATUS: RX_FIFO_COUNT Mask */

#define SPI_STATUS_SLV_START_INTSTS_Pos   11                                         /*!< SPI_T::STATUS: SLV_START_INTSTS Position */
#define SPI_STATUS_SLV_START_INTSTS_Msk   (1ul << SPI_STATUS_SLV_START_INTSTS_Pos)   /*!< SPI_T::STATUS: SLV_START_INTSTS Mask */

#define SPI_STATUS_TX_INTSTS_Pos          4                                          /*!< SPI_T::STATUS: TX_INTSTS Position */
#define SPI_STATUS_TX_INTSTS_Msk          (1ul << SPI_STATUS_TX_INTSTS_Pos)          /*!< SPI_T::STATUS: TX_INTSTS Mask */

#define SPI_STATUS_RX_OVERRUN_Pos         2                                          /*!< SPI_T::STATUS: RX_OVERRUN Position */
#define SPI_STATUS_RX_OVERRUN_Msk         (1ul << SPI_STATUS_RX_OVERRUN_Pos)         /*!< SPI_T::STATUS: RX_OVERRUN Mask */

#define SPI_STATUS_RX_INTSTS_Pos          0                                          /*!< SPI_T::STATUS: RX_INTSTS Position */
#define SPI_STATUS_RX_INTSTS_Msk          (1ul << SPI_STATUS_RX_INTSTS_Pos)          /*!< SPI_T::STATUS: RX_INTSTS Mask */
/*@}*/ /* end of group SPI_CONST */
/*@}*/ /* end of group SPI */




/*---------------------- System Manger Controller -------------------------*/
/**
    @addtogroup SYS System Manger Controller (SYS)
    Memory Mapped Structure for SYS Controller
@{ */


typedef struct
{



    /**
     * @var GCR_T::PDID
     * Offset: 0x00  Part Device Identification Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDID      |Part Device Identification Number
     * |        |          |This register reflects device part number code. Software can read this register to identify which device is used.
     * @var GCR_T::RSTSRC
     * Offset: 0x04  System Reset Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RSTS_POR  |Power-on Reset Flag
     * |        |          |The RSTS_POR flag is set by the "reset signal" from the Power-on Reset (POR) controller or bit CHIP_RST (IPRSTC1[0]) to indicate the previous reset source.
     * |        |          |0 = No reset from POR or CHIP_RST (IPRSTC1[0]).
     * |        |          |1 = Power-on Reset (POR) or CHIP_RST (IPRSTC1[0]) had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared to 0 by software writing "1".
     * |[1]     |RSTS_RESET|Reset Pin Reset Flag
     * |        |          |The RSTS_RESET flag is set by the "Reset Signal" from the nRESET pin to indicate the previous reset source.
     * |        |          |0 = No reset from the nRESET pin.
     * |        |          |1 = The nRESET pin had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared to 0 by software writing "1".     
     * |[2]     |RSTS_WDT  |Watchdog Reset Flag
     * |        |          |The RSTS_WDT flag is set by the "Reset Signal" from the Watchdog Timer to indicate the previous reset source
     * |        |          |0 = No reset from watchdog timer.
     * |        |          |1 = The watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note1: This bit can be cleared to 0 by software writing "1".
     * |        |          |Note2: Watchdog Timer register WTRF (WTCR[2]) bit is set if the system has been reset by WDT time-out reset. 
     * |        |          |Window Watchdog Timer register WWDTRF (WWDTSR[1]) bit is set if the system has been reset by WWDT time-out reset.     
     * |[3]     |RSTS_LVR  |Low Voltage Reset Flag
     * |        |          |The RSTS_LVR flag is set by the "Reset Signal" from the Low-Voltage-Reset controller to indicate the previous reset source.
     * |        |          |0 = No reset from LVR.
     * |        |          |1 = The LVR controller had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared to 0 by software writing "1".
     * |[4]     |RSTS_BOD  |Brown-out Detector Reset Flag
     * |        |          |The RSTS_BOD flag is set by the "Reset Signal" from the Brown-out Detector to indicate the previous reset source.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared to 0 by software writing "1".
     * |[5]     |RSTS_SYS  |System Reset Flag
     * |        |          |The RSTS_SYS flag is set by the "Reset Signal" from the Cortex-M0 kernel to indicate the previous reset source.
     * |        |          |0 = No reset from Cortex-M0.
     * |        |          |1 = The Cortex-M0 had issued the reset signal to reset the system by writing 1 to bit SYSRESETREQ (AIRCR[2], Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M0 kernel.
     * |        |          |Note: This bit can be cleared to 0 by software writing "1".
     * |[7]     |RSTS_CPU  |CPU Reset Flag
     * |        |          |The RSTS_CPU flag is set by hardware if software writes CPU_RST (IPRSTC1[1]) 1 to reset Cortex-M0 CPU kernel and flash Memory Controller (FMC).
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = Cortex-M0 CPU kernel and FMC are reset by software setting CPU_RST(IPRSTC1[1]) to 1.
     * |        |          |Note: This bit can be cleared to 0 by software writing "1".
     * @var GCR_T::IPRSTC1
     * Offset: 0x08  Peripheral Reset Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIP_RST  |CHIP One-shot Reset (Write Protect)
     * |        |          |Setting this bit will reset the whole chip, including CPU kernel and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |The CHIP_RST is the same as the POR reset, all the chip controllers are reset and the chip setting from flash are also reload.
     * |        |          |0 = CHIP normal operation.
     * |        |          |1 = CHIP one-shot reset.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[1]     |CPU_RST   |CPU Kernel One-shot Reset (Write Protect)
     * |        |          |Setting this bit will only reset the CPU kernel and Flash Memory Controller(FMC), and this bit will automatically return 0 after the two clock cycles
     * |        |          |0 = CPU normal operation.
     * |        |          |1 = CPU one-shot reset.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[4]     |HDIV_RST  |HDIV Controller Reset (Write Protect)
     * |        |          |Set this bit to 1 will generate a reset signal to the hardware divider. User need to set this bit to 0 to release from the reset state.
     * |        |          |0 = Hardware divider controller normal operation.
     * |        |          |1 = Hardware divider controller reset.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * @var GCR_T::IPRSTC2
     * Offset: 0x0C  Peripheral Reset Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |GPIO_RST  |GPIO Controller Reset
     * |        |          |0 = GPIO controller normal operation.
     * |        |          |1 = GPIO controller reset.
     * |[2]     |TMR0_RST  |Timer0 Controller Reset
     * |        |          |0 = Timer0 controller normal operation.
     * |        |          |1 = Timer0 controller reset.
     * |[3]     |TMR1_RST  |Timer1 Controller Reset
     * |        |          |0 = Timer1 controller normal operation.
     * |        |          |1 = Timer1 controller reset.
     * |[4]     |TMR2_RST  |Timer2 Controller Reset
     * |        |          |0 = Timer2 controller normal operation.
     * |        |          |1 = Timer2 controller reset.
     * |[5]     |TMR3_RST  |Timer3 Controller Reset
     * |        |          |0 = Timer3 controller normal operation.
     * |        |          |1 = Timer3 controller reset.
     * |[8]     |I2C0_RS   |I2C0 Controller Reset
     * |        |          |0 = I2C0 controller normal operation.
     * |        |          |1 = I2C0 controller reset.
     * |[12]    |SPI0_RST  |SPI0 Controller Reset
     * |        |          |0 = SPI0 controller normal operation.
     * |        |          |1 = SPI0 controller reset.
     * |[13]    |SPI1_RST  |SPI1 Controller Reset
     * |        |          |0 = SPI1 controller normal operation.
     * |        |          |1 = SPI1 controller reset.
     * |[14]    |SPI2_RST  |SPI2 Controller Reset
     * |        |          |0 = SPI2 controller normal operation.
     * |        |          |1 = SPI2 controller reset.
     * |[16]    |UART0_RST |UART0 Controller Reset
     * |        |          |0 = UART0 controller normal operation.
     * |        |          |1 = UART0 controller reset.
     * |[17]    |UART1_RST |UART1 Controller Reset
     * |        |          |0 = UART1 controller normal operation.
     * |        |          |1 = UART1 controller reset.
     * |[19]    |BPWM0_RST |Basic PWM0 Controller Reset
     * |        |          |0 = Basic PWM0 controller normal operation.
     * |        |          |1 = Basic PWM0 controller reset.
     * |[20]    |EPWM0_RST |Enhanced PWM0 Controller Reset
     * |        |          |0 = EPWM0 controller normal operation.
     * |        |          |1 = EPWM0 controller reset.
     * |[21]    |EPWM1_RST |Enhanced PWM1 Controller Reset
     * |        |          |0 = EPWM1 controller normal operation.
     * |        |          |1 = EPWM1 controller reset.
     * |[22]    |ACMP_RST  |Analog Comparator Controller Reset
     * |        |          |0 = Analog Comparator controller normal operation.
     * |        |          |1 = Analog Comparator controller reset.
     * |[26]    |ECAP0_RST |Enhanced Input Capture 0 Controller Reset
     * |        |          |0 = Enhanced input capture 0 controller normal operation.
     * |        |          |1 = Enhanced input capture 0 controller reset.
     * |[27]    |ECAP1_RST |Enhanced Input Capture 1 Controller Reset
     * |        |          |0 = Enhanced input capture 1 controller normal operation.
     * |        |          |1 = Enhanced input capture 1 controller reset.
     * |[28]    |EADC_RST  |EADC Controller Reset
     * |        |          |0 = EADC controller normal operation.
     * |        |          |1 = EADC controller reset.
     * |[29]    |OPA_RST   |OPA0 and OPA1 Controller Reset
     * |        |          |0 = OPA0 and OPA1 controller normal operation.
     * |        |          |1 = OPA0 and OPA1 controller reset.
     * @var GCR_T::BODCR
     * Offset: 0x18  Brown-out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BOD_EN    |Brown-out Detector Enable (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CBODEN (Config0[23]) bit.
     * |        |          |0 = Brown-out Detector function Disabled.
     * |        |          |1 = Brown-out Detector function Enabled.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[2:1]   |BOD_VL    |Brown-out Detector Threshold Voltage Selection (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CBOV (Config0[22:21]) bit.
     * |        |          |00 = Brown-out voltage is 2.2V.
     * |        |          |01 = Brown-out voltage is 2.7V.
     * |        |          |10 = Brown-out voltage is 3.7V.
     * |        |          |11 = Brown-out voltage is 4.4V.
     * |        |          |Note: These bits are write protected bit. Refer to the REGWRPROT register.
     * |[3]     |BOD_RSTEN |Brown-out Reset Enable Control (Write Protect)
     * |        |          |0 = Brown-out "INTERRUPT" function Enabled.
     * |        |          |While the BOD function is enabled (BOD_EN high) and BOD interrupt function is enabled (BOD_RSTEN low), BOD will assert an interrupt if BOD_OUT is high.
     * |        |          |BOD interrupt will keep till to the BOD_EN set to 0. BOD interrupt can be blocked by disabling the NVIC BOD interrupt or disabling BOD function (set BOD_EN low).
     * |        |          |1 = Brown-out "RESET" function Enabled.
     * |        |          |Note1: While the Brown-out Detector function is enabled (BOD_EN high) and BOD reset function is enabled (BOD_RSTEN high), BOD will assert a signal to reset chip when the detected voltage is lower than the threshold (BOD_OUT high).
     * |        |          |Note2: The default value is set by flash controller user configuration register CBORST (Config0[20]) bit.
     * |        |          |Note3: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[4]     |BOD_INTF  |Brown-out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage draft at VDD down through or up through the voltage of BOD_VL setting.
     * |        |          |1 = When Brown-out Detector detects the VDD is dropped down through the voltage of BOD_VL setting or the VDD is raised up through the voltage of BOD_VL setting, this bit is set to 1 and the Brown-out interrupt is requested if Brown-out interrupt is enabled.
     * |        |          |Note: This bit can be cleared to 0 by software writing "1".
     * |[5]     |BOD_LPM   |Brown-out Detector Low Power Mode (Write Protection)
     * |        |          |0 = BOD operated in Normal mode (default).
     * |        |          |1 = BOD Low Power mode Enabled.
     * |        |          |Note1: The BOD consumes about 100 uA in Normal mode, and the low power mode can reduce the current to about 1/10 but slow the BOD response.
     * |        |          |Note2: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[6]     |BOD_OUT   |Brown-out Detector Output Status
     * |        |          |0 = Brown-out Detector output status is 0. It means the detected voltage is higher than BOD_VL setting or BOD_EN is 0.
     * |        |          |1 = Brown-out Detector output status is 1. It means the detected voltage is lower than BOD_VL setting. If the BOD_EN is 0, BOD function disabled, this bit always responds to 0.
     * |[7]     |LVR_EN    |Low Voltage Reset Enable Bit (Write Protect)
     * |        |          |The LVR function reset the chip when the input power voltage is lower than LVR circuit setting. LVR function is enabled by default.
     * |        |          |0 = Low Voltage Reset function Disabled.
     * |        |          |1 = Low Voltage Reset function Enabled - After enabling the bit, the LVR function will be active with 100us delay for LVR output stable (default).
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * @var GCR_T::TEMPCR
     * Offset: 0x1C  Temperature Sensor Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VTEMP_EN  |Temperature Sensor Enable Control
     * |        |          |This bit is used to enable/disable temperature sensor function.
     * |        |          |0 = Temperature sensor function Disabled (default).
     * |        |          |1 = Temperature sensor function Enabled.
     * |        |          |Note: After this bit is set to 1, the value of temperature can be obtained from ADC conversion result.
     * |        |          |Please refer to the EADC function chapter for detail ADC conversion functional description.
     * @var GCR_T::PORCR
     * Offset: 0x24  Power-on Reset Controller Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |POR_DIS_CODE|Power-on Reset Enable Control (Write Protect)
     * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again.
     * |        |          |User can disable internal POR circuit to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
     * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note: These bits are write protected bit. Refer to the REGWRPROT register.
     * @var GCR_T::P0_MFP
     * Offset: 0x30  P0 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |P0_MFP[0] |P0.0 Multi-Function Selection
     * |        |          |This bit combined with P0_ALT[0] selects P0.0 multi-function.
     * |        |          |(P0_ALT[0], P0_MFP[0]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = EPWM0_CH0 function is selected.
     * |        |          |(1, 1) = ECAP1_IC0 function is selected.
     * |[1]     |P0_MFP[1] |P0.1 Multi-Function Selection
     * |        |          |This bit combined with P0_ALT[1] selects P0.1 multi-function.
     * |        |          |(P0_ALT[1], P0_MFP[1]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = EPWM0_CH1 function is selected.
     * |        |          |(1, 1) = ECAP1_IC1 function is selected.
     * |[2]     |P0_MFP[2] |P0.2 Multi-Function Selection
     * |        |          |This bit combined with P0_ALT[2] selects P0.2 multi-function.
     * |        |          |(P0_ALT[2], P0_MFP[2]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = EPWM0_CH2 function is selected.
     * |        |          |(1, 1) = ECAP1_IC2 function is selected.
     * |[3]     |P0_MFP[3] |P0.3 Multi-Function Selection
     * |        |          |This bit combined with P0_ALT[3] selects P0.3 multi-function.
     * |        |          |(P0_ALT[3], P0_MFP[3]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = EPWM0_CH3 function is selected.
     * |        |          |(1, 1) = STADC function is selected.
     * |[4]     |P0_MFP[4] |P0.4 Multi-Function Selection
     * |        |          |This bit selects P0.4 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM0_CH4 function is selected.
     * |[5]     |P0_MFP[5] |P0.5 Multi-Function Selection
     * |        |          |This bit selects P0.5 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM0_CH5 function is selected.
     * |[6]     |P0_MFP[6] |P0.6 Multi-Function Selection
     * |        |          |This bit selects P0.6 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM0_BRAKE1 function is selected.
     * |[7]     |P0_MFP[7] |P0.7 Multi-Function Selection
     * |        |          |This bit selects P0.7 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = STADC function is selected.
     * |[8]     |P0_ALT[0] |P0.0 Alternative Function
     * |        |          |See P0_MFP[0].
     * |[9]     |P0_ALT[1] |P0.1 Alternative Function
     * |        |          |See P0_MFP[1].
     * |[10]    |P0_ALT[2] |P0.2 Alternative Function
     * |        |          |See P0_MFP[2].
     * |[11]    |P0_ALT[3] |P0.3 Alternative Function
     * |        |          |See P0_MFP[3].
     * |[23:16] |P0_TYPEn  |P0[7:0] Input Schmitt Trigger Function Enable Bits
     * |        |          |0 = P0[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P0[7:0] I/O input Schmitt Trigger function Enabled.
     * @var GCR_T::P1_MFP
     * Offset: 0x34  P1 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |P1_MFP[0] |P1.0 Multi-Function Selection
     * |        |          |This bit selects P1.0 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM1_CH0 function is selected.
     * |[1]     |P1_MFP[1] |P1.1 Multi-Function Selection
     * |        |          |This bit selects P1.1 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM1_CH1 function is selected.
     * |[2]     |P1_MFP[2] |P1.2 Multi-Function Selection
     * |        |          |This bit selects P1.2 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM1_CH2 function is selected.
     * |[3]     |P1_MFP[3] |P1.3 Multi-Function Selection
     * |        |          |This bit selects P1.3 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM1_CH3 function is selected.
     * |[4]     |P1_MFP[4] |P1.4 Multi-Function Selection
     * |        |          |This bit selects P1.4 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM1_CH4 function is selected.
     * |[5]     |P1_MFP[5] |P1.5 Multi-Function Selection
     * |        |          |This bit selects P1.5 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM1_CH5 function is selected.
     * |[6]     |P1_MFP[6] |P1.6 Multi-Function Selection
     * |        |          |This bit selects P1.6 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM0_BRAKE0 function is selected.
     * |[7]     |P1_MFP[7] |P1.7 Multi-Function Selection
     * |        |          |This bit selects P1.7 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM1_BRAKE0 function is selected.
     * |[23:16] |P1_TYPEn  |P1[7:0] Input Schmitt Trigger Function Enable Bits
     * |        |          |0 = P1[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P1[7:0] I/O input Schmitt Trigger function Enabled.
     * @var GCR_T::P2_MFP
     * Offset: 0x38  P2 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |P2_MFP[0] |P2.0 Multi-Function Selection
     * |        |          |This bit combined with P2_ALT[0] selects P2.0 multi-function.
     * |        |          |(P2_ALT[0], P2_MFP[0]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI2_MOSI function is selected.
     * |        |          |(1, 0) = ACMP2_O function is selected.
     * |[1]     |P2_MFP[1] |P2.1 Multi-Function Selection
     * |        |          |This bit selects P2.1 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = ECAP0_IC2 function is selected.
     * |[2]     |P2_MFP[2] |P2.2 Multi-Function Selection
     * |        |          |This bit selects P2.2 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = ECAP0_IC1 function is selected.
     * |[3]     |P2_MFP[3] |P2.3 Multi-Function Selection
     * |        |          |This bit selects P2.3 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = ECAP0_IC0 function is selected.
     * |[4]     |P2_MFP[4] |P2.4 Multi-Function Selection
     * |        |          |This bit selects P2.4 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = Reserved.
     * |[5]     |P2_MFP[5] |P2.5 Multi-Function Selection
     * |        |          |This bit selects P2.5 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = Reserved.
     * |[6]     |P2_MFP[6] |P2.6 Multi-Function Selection
     * |        |          |This bit combined with P2_ALT[6] selects P2.6 multi-function.
     * |        |          |(P2_ALT[6], P2_MFP[6]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = Reserved.
     * |        |          |(1, 0) = SPI0_SS function is selected.
     * |        |          |(1, 1) = UART1_nCTS function is selected.
     * |[7]     |P2_MFP[7] |P2.7 Multi-Function Selection
     * |        |          |This bit combined with P2_ALT[7] selects P2.7 multi-function.
     * |        |          |(P2_ALT[7], P2_MFP[7]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(1, 0) = SPI0_CLK function is selected.
     * |        |          |(1, 1) = UART1_nRTS function is selected.
     * |[8]     |P2_ALT[0] |P2.0 Alternative Function
     * |        |          |See P2_MFP[0].
     * |[14]    |P2_ALT[6] |P2.6 Alternative Function
     * |        |          |See P2_MFP[6].
     * |[15]    |P2_ALT[7] |P2.7 Alternative Function
     * |        |          |See P2_MFP[7].
     * |[23:16] |P2_TYPEn  |P2[7:0] Input Schmitt Trigger Function Enable
     * |        |          |0 = P2[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P2[7:0] I/O input Schmitt Trigger function Enabled.
     * @var GCR_T::P3_MFP
     * Offset: 0x3C  P3 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |P3_MFP[0] |P3.0 Multi-Function Selection
     * |        |          |This bit combined with P3_ALT[0] selects P3.0 multi-function.
     * |        |          |(P3_ALT[0], P3_MFP[0]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART0_RXD function is selected.
     * |        |          |(1, 0) = CLKO function is selected.
     * |[1]     |P3_MFP[1] |P3.1 Multi-Function Selection
     * |        |          |This bit combined with P3_ALT[1] selects P3.1 multi-function.
     * |        |          |(P3_ALT[1], P3_MFP[1]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART0_TXD function is selected.
     * |        |          |(1, 0) = ACMP0_O function is selected.
     * |[2]     |P3_MFP[2] |P3.2 Multi-Function Selection
     * |        |          |This bit selects P3.2 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = INT0 function is selected.
     * |[3]     |P3_MFP[3] |P3.3 Multi-Function Selection
     * |        |          |This bit selects P3.4 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = INT1 function is selected.
     * |[4]     |P3_MFP[4] |P3.4 Multi-Function Selection
     * |        |          |This bit combined with P3_ALT[4] selects P3.4 multi-function.
     * |        |          |(P3_ALT[4], P3_MFP[4]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = TM0 function is selected.
     * |        |          |(1, 0) = I2C0_SDA function is selected.
     * |[5]     |P3_MFP[5] |P3.5 Multi-Function Selection
     * |        |          |This bit combined with P3_ALT[5] selects P3.5 multi-function.
     * |        |          |(P3_ALT[5], P3_MFP[5]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = TM1 function is selected.
     * |        |          |(1, 0) = I2C0_SCL function is selected.
     * |[8]     |P3_ALT[0] |P3.0 Alternative Function
     * |        |          |See P3_MFP[0].
     * |[9]     |P3_ALT[1] |P3.1 Alternative Function
     * |        |          |See P3_MFP[1].
     * |[12]    |P3_ALT[4] |P3.4 Alternative Function
     * |        |          |See P3_MFP4].
     * |[13]    |P3_ALT[5] |P3.5 Alternative Function
     * |        |          |See P3_MFP[5].
     * |[23:16] |P3_TYPEn  |P3[7:0] Input Schmitt Trigger Function Enable Bits
     * |        |          |0 = P3[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P3[7:0] I/O input Schmitt Trigger function Enabled.
     * @var GCR_T::P4_MFP
     * Offset: 0x40  P4 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |P4_MFP[0] |P4.0 Multi-Function Selection
     * |        |          |This bit selects P4.0 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = ECAP1_IC0 function is selected.
     * |[1]     |P4_MFP[1] |P4.1 Multi-Function Selection
     * |        |          |This bit selects P4.1 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = ECAP1_IC1 function is selected.
     * |[2]     |P4_MFP[2] |P4.2 Multi-Function Selection
     * |        |          |This bit selects P4.2 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = ECAP1_IC2 function is selected.
     * |[4]     |P4_MFP[4] |P4.4 Multi-Function Selection
     * |        |          |This bit selects P4.4 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = Reserved.
     * |[5]     |P4_MFP[5] |P4.5 Multi-Function Selection
     * |        |          |This bit selects P4.5 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = Reserved.
     * |[6]     |P4_MFP[6] |P4.6 Multi-Function Selection
     * |        |          |This bit combined with P4_ALT[6] selects P4.6 multi-function.
     * |        |          |(P4_ALT[6], P4_MFP[6]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = TM2 function is selected.
     * |        |          |(1, 0) = Reserved.
     * |[7]     |P4_MFP[7] |P4.7 Multi-Function Selection
     * |        |          |This bit selects P4.7 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = TM3 function is selected.
     * |[14]    |P4_ALT[6] |P4.6 Alternative Function
     * |        |          |See P4_MFP[6].
     * |[23:16] |P4_TYPEn  |P4[7:0] Input Schmitt Trigger Function Enable Bits
     * |        |          |0 = P4[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P4[7:0] I/O input Schmitt Trigger function Enabled.
     * @var GCR_T::P5_MFP
     * Offset: 0x44  P5 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |P5_MFP[0] |P5.0 Multi-Function Selection
     * |        |          |This bit combined with P5_ALT[0] selects P5.0 multi-function.
     * |        |          |(P5_ALT[0], P5_MFP[0]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIIO function is selected.
     * |        |          |(0, 1) = SPI0_MOSI function is selected.
     * |        |          |(1, 0) = UART0_nRTS function is selected.
     * |        |          |(1, 1) = I2C0_SCL function is selected.
     * |[1]     |P5_MFP[1] |P5.1 Multi-Function Selection
     * |        |          |This bit combined with P5_ALT[1] selects P5.1 multi-function.
     * |        |          |(P5_ALT[1], P5_MFP[1]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIIO function is selected.
     * |        |          |(0, 1) = SPI0_MISO function is selected.
     * |        |          |(1, 0) = UART0_nCTS function is selected.
     * |        |          |(1, 1) = I2C0_SDA function is selected.
     * |[2]     |P5_MFP[2] |P5.2 Multi-Function Selection
     * |        |          |This bit combined with P5_ALT[2] selects P5.2 multi-function.
     * |        |          |(P5_ALT[2], P5_MFP[2]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIIO function is selected.
     * |        |          |(0, 1) = SPI2_MISO function is selected.
     * |        |          |(1, 0) = ACMP1_O function is selected.
     * |[3]     |P5_MFP[3] |P5.3 Multi-Function Selection
     * |        |          |This bit selects P5.3 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = SPI2_CLK function is selected.
     * |[4]     |P5_MFP[4] |P5.4 Multi-Function Selection
     * |        |          |This bit selects P5.4 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = SPI2_SS function is selected.
     * |[5]     |P5_MFP[5] |P5.5 Multi-Function Selection
     * |        |          |This bit selects P5.5 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = CLKO function is selected.
     * |[6]     |P5_MFP[6] |P5.6 Multi-Function Selection
     * |        |          |This bit selects P5.6 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = BPWM0_CH0 function is selected.
     * |[7]     |P5_MFP[7] |P5.7 Multi-Function Selection
     * |        |          |This bit selects P5.7 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = BPWM0_CH1 function is selected.
     * |[8]     |P5_ALT[0] |P5.0 Alternative Function
     * |        |          |See P5_MFP[0].
     * |[9]     |P5_ALT[1] |P5.1 Alternative Function
     * |        |          |See P5_MFP[1].
     * |[10]    |P5_ALT[2] |P5.2 Alternative Function
     * |        |          |See P5_MFP[2].
     * |[23:16] |P5_TYPEn  |P5[7:0] Input Schmitt Trigger Function Enable Bits
     * |        |          |0 = P5[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P5[7:0] I/O input Schmitt Trigger function Enabled.
     * @var GCR_T::P6_MFP
     * Offset: 0x48  P6 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |P6_MFP[0] |P6.0 Multi-Function Selection
     * |        |          |This bit selects P6.0 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC0_CH0 function is selected.
     * |[1]     |P6_MFP[1] |P6.1 Multi-Function Selection
     * |        |          |This bit selects P6.1 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC0_CH1 function is selected.
     * |[2]     |P6_MFP[2] |P6.2 Multi-Function Selection
     * |        |          |This bit selects P6.2 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC0_CH2 function is selected.
     * |[3]     |P6_MFP[3] |P6.3 Multi-Function Selection
     * |        |          |This bit selects P6.3 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC0_CH3 function is selected.
     * |[4]     |P6_MFP[4] |P6.4 Multi-Function Selection
     * |        |          |This bit selects P6.4 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC0_CH4 or ACMP1_N function is selected.
     * |[5]     |P6_MFP[5] |P6.5 Multi-Function Selection
     * |        |          |This bit selects P6.5 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC0_CH5 or ACMP1_P function is selected.
     * |[6]     |P6_MFP[6] |P6.6 Multi-Function Selection
     * |        |          |This bit selects P6.6 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC0_CH6 function is selected.
     * |[7]     |P6_MFP[7] |P6.7 Multi-Function Selection
     * |        |          |This bit selects P6.7 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC0_CH7 function is selected.
     * |[23:16] |P6_TYPEn  |P6[7:0] Input Schmitt Trigger Function Enable Bits
     * |        |          |0 = P6[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P6[7:0] I/O input Schmitt Trigger function Enabled.
     * @var GCR_T::P7_MFP
     * Offset: 0x4C  P7 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |P7_MFP[0] |P7.0 Multi-Function Selection
     * |        |          |This bit selects P7.0 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC1_CH0 function is selected.
     * |[1]     |P7_MFP[1] |P7.1 Multi-Function Selection
     * |        |          |This bit selects P7.1 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC1_CH1 function is selected.
     * |[2]     |P7_MFP[2] |P7.2 Multi-Function Selection
     * |        |          |This bit selects P7.2 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC1_CH2 function is selected.
     * |[3]     |P7_MFP[3] |P7.3 Multi-Function Selection
     * |        |          |This bit selects P7.3 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC1_CH3 function is selected.
     * |[4]     |P7_MFP[4] |P7.4 Multi-Function Selection
     * |        |          |This bit selects P7.4 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC1_CH4 or ACMP2_N function is selected.
     * |[5]     |P7_MFP[5] |P7.5 Multi-Function Selection
     * |        |          |This bit selects P7.5 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC1_CH5 or ACMP2_N function is selected.
     * |[6]     |P7_MFP[6] |P7.6 Multi-Function Selection
     * |        |          |This bit selects P7.6 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC1_CH6 function is selected.
     * |[7]     |P7_MFP[7] |P7.7 Multi-Function Selection
     * |        |          |This bit selects P7.7 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EADC1_CH7 function is selected.
     * |[23:16] |P7_TYPEn  |P7[7:0] Input Schmitt Trigger Function Enable Bits
     * |        |          |0 = P7[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P7[7:0] I/O input Schmitt Trigger function Enabled.
     * @var GCR_T::P8_MFP
     * Offset: 0x50  P8 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |P8_MFP[0] |P8.0 Multi-Function Selection
     * |        |          |This bit selects P8.0 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = OP0_P function is selected.
     * |[1]     |P8_MFP[1] |P8.1 Multi-Function Selection
     * |        |          |This bit selects P8.1 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = OP0_N function is selected.
     * |[2]     |P8_MFP[2] |P8.2 Multi-Function Selection
     * |        |          |This bit selects P8.2 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = OP0_O function is selected.
     * |[3]     |P8_MFP[3] |P8.3 Multi-Function Selection
     * |        |          |This bit selects P8.3 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = ACMP0_N function is selected.
     * |[4]     |P8_MFP[4] |P8.4 Multi-Function Selection
     * |        |          |This bit selects P8.4 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = ACMP0_P function is selected.
     * |[7]     |P8_MFP[7] |P8.7 Multi-Function Selection
     * |        |          |This bit selects P8.7 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = ACMP0_O function is selected.
     * |[23:16] |P8_TYPEn  |P8[7:0] Input Schmitt Trigger Function Enable Bits
     * |        |          |0 = P8[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P8[7:0] I/O input Schmitt Trigger function Enabled.
     * @var GCR_T::P9_MFP
     * Offset: 0x54  P9 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |P9_MFP[0] |P9.0 Multi-Function Selection
     * |        |          |This bit selects P9.0 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = OP1_O function is selected.
     * |[1]     |P9_MFP[1] |P9.1 Multi-Function Selection
     * |        |          |This bit selects P9.1 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = OP1_N function is selected.
     * |[2]     |P9_MFP[2] |P9.2 Multi-Function Selection
     * |        |          |This bit selects P9.2 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = OP1_P function is selected.
     * |[3]     |P9_MFP[3] |P9.3 Multi-Function Selection
     * |        |          |This bit selects P9.3 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = EPWM1_BRAKE1 function is selected.
     * |[4]     |P9_MFP[4] |P9.4 Multi-Function Selection
     * |        |          |This bit selects P9.4 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = SPI1_CLK function is selected.
     * |[5]     |P9_MFP[5] |P9.5 Multi-Function Selection
     * |        |          |This bit selects P9.5 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = SPI1_MISO function is selected.
     * |[6]     |P9_MFP[6] |P9.6 Multi-Function Selection
     * |        |          |This bit selects P9.6 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = SPI1_MOSI function is selected.
     * |[7]     |P9_MFP[7] |P9.7 Multi-Function Selection
     * |        |          |This bit selects P9.7 multi-function.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = SPI1_SS function is selected.
     * |[23:16] |P9_TYPEn  |P9[7:0] Input Schmitt Trigger Function Enable Bits
     * |        |          |0 = P9[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P9[7:0] I/O input Schmitt Trigger function Enabled.
     * @var GCR_T::PA_MFP
     * Offset: 0x58  PA Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PA_MFP[0] |PA.0 Multi-Function Selection
     * |        |          |This bit combined with PA_ALT[0] selects PA.0 multi-function.
     * |        |          |(PA_ALT[0], PA_MFP[0]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIIO function is selected.
     * |        |          |(0, 1) = UART1_TXD function is selected.
     * |        |          |(1, 1) = I2C0_SDA function is selected.
     * |[1]     |PA_MFP[1] |PA.1 Multi-Function Selection
     * |        |          |This bit combined with PA_ALT[1] selects PA.1 multi-function.
     * |        |          |(PA_ALT[1], PA_MFP[1]) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIIO function is selected.
     * |        |          |(0, 1) = UART1_RXD function is selected.
     * |        |          |(1, 1) = I2C0_SCL function is selected.
     * |[8]     |PA_ALT[0] |PA.0 Alternative Function
     * |        |          |See PA_MFP[0].
     * |[9]     |PA_ALT[1] |PA.1 Alternative Function
     * |        |          |See PA_MFP[1].
     * |[23:16] |PA_TYPEn  |PA[7:0] Input Schmitt Trigger Function Enable Bits
     * |        |          |0 = PA[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = PA[7:0] I/O input Schmitt Trigger function Enabled.
     * @var GCR_T::REGWRPROT
     * Offset: 0x100  Register Write Protection Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |REGPROTDIS|Register Write-Protection Disable Index (Read Only)
     * |        |          |0 = Write-protection is enabled for writing protected registers.
     * |        |          |Any write to the protected register is ignored.
     * |        |          |1 = Write-protection is disabled for writing protected registers.
     * |        |          |Note: The bits which are write-protected will be noted as" (Write Protect)" beside the description.
     * |[7:0]   |REGWRPROT |Register Write-Protection Code (Write Only)
     * |        |          |Some registers have write-protection function.
     * |        |          |Writing these registers have to disable the protected function by writing the sequence value "59h", "16h", "88h" to this field.
     * |        |          |After this sequence is completed, the REGPROTDIS bit will be set to 1 and write-protection registers can be normal write.
     */

    __I  uint32_t PDID;          /* Offset: 0x00  Part Device Identification Number Register                         */
    __IO uint32_t RSTSRC;        /* Offset: 0x04  System Reset Source Register                                       */
    __IO uint32_t IPRSTC1;       /* Offset: 0x08  Peripheral Reset Control Register 1                                */
    __IO uint32_t IPRSTC2;       /* Offset: 0x0C  Peripheral Reset Control Register 2                                */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t BODCR;         /* Offset: 0x18  Brown-out Detector Control Register                                */
    __IO uint32_t TEMPCR;        /* Offset: 0x1C  Temperature Sensor Control Register                                */
    __I  uint32_t RESERVE1;
    __IO uint32_t PORCR;         /* Offset: 0x24  Power-on-Reset Controller Register                                 */
    __I  uint32_t RESERVE2[2];
    __IO uint32_t P0_MFP;        /* Offset: 0x30  P0 Multiple Function and Input Type Control Register               */
    __IO uint32_t P1_MFP;        /* Offset: 0x34  P1 Multiple Function and Input Type Control Register               */
    __IO uint32_t P2_MFP;        /* Offset: 0x38  P2 Multiple Function and Input Type Control Register               */
    __IO uint32_t P3_MFP;        /* Offset: 0x3C  P3 Multiple Function and Input Type Control Register               */
    __IO uint32_t P4_MFP;        /* Offset: 0x40  P4 Multiple Function and Input Type Control Register               */
    __IO uint32_t P5_MFP;        /* Offset: 0x44  P5 Multiple Function and Input Type Control Register               */
    __IO uint32_t P6_MFP;        /* Offset: 0x48  P6 Multiple Function and Input Type Control Register               */
    __IO uint32_t P7_MFP;        /* Offset: 0x4C  P7 Multiple Function and Input Type Control Register               */
    __IO uint32_t P8_MFP;        /* Offset: 0x50  P8 Multiple Function and Input Type Control Register               */
    __IO uint32_t P9_MFP;        /* Offset: 0x54  P9 Multiple Function and Input Type Control Register               */
    __IO uint32_t PA_MFP;        /* Offset: 0x58  PA Multiple Function and Input Type Control Register               */
    __I  uint32_t RESERVE3[41];
    __IO uint32_t REGWRPROT;     /* Offset: 0x100  Register Write Protection Register                                */

} GCR_T;




/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */

/* GCR RSTSRC Bit Field Definitions */
#define SYS_RSTSRC_RSTS_CPU_Pos                 7                                   /*!< GCR_T::RSTSRC: RSTS_CPU Position */
#define SYS_RSTSRC_RSTS_CPU_Msk                 (1ul << SYS_RSTSRC_RSTS_CPU_Pos)    /*!< GCR_T::RSTSRC: RSTS_CPU Mask */

#define SYS_RSTSRC_RSTS_SYS_Pos                 5                                   /*!< GCR_T::RSTSRC: RSTS_SYS Position */
#define SYS_RSTSRC_RSTS_SYS_Msk                 (1ul << SYS_RSTSRC_RSTS_SYS_Pos)    /*!< GCR_T::RSTSRC: RSTS_SYS Mask */

#define SYS_RSTSRC_RSTS_BOD_Pos                 4                                   /*!< GCR_T::RSTSRC: RSTS_BOD Position */
#define SYS_RSTSRC_RSTS_BOD_Msk                 (1ul << SYS_RSTSRC_RSTS_BOD_Pos)    /*!< GCR_T::RSTSRC: RSTS_BOD Mask */

#define SYS_RSTSRC_RSTS_LVR_Pos                 3                                   /*!< GCR_T::RSTSRC: RSTS_LVR Position */
#define SYS_RSTSRC_RSTS_LVR_Msk                 (1ul << SYS_RSTSRC_RSTS_LVR_Pos)    /*!< GCR_T::RSTSRC: RSTS_LVR Mask */

#define SYS_RSTSRC_RSTS_WDT_Pos                 2                                   /*!< GCR_T::RSTSRC: RSTS_WDT Position */
#define SYS_RSTSRC_RSTS_WDT_Msk                 (1ul << SYS_RSTSRC_RSTS_WDT_Pos)    /*!< GCR_T::RSTSRC: RSTS_WDT Mask */

#define SYS_RSTSRC_RSTS_RESET_Pos               1                                   /*!< GCR_T::RSTSRC: RSTS_RESET Position */
#define SYS_RSTSRC_RSTS_RESET_Msk               (1ul << SYS_RSTSRC_RSTS_RESET_Pos)  /*!< GCR_T::RSTSRC: RSTS_RESET Mask */

#define SYS_RSTSRC_RSTS_POR_Pos                 0                                   /*!< GCR_T::RSTSRC: RSTS_POR Position */
#define SYS_RSTSRC_RSTS_POR_Msk                 (1ul << SYS_RSTSRC_RSTS_POR_Pos)    /*!< GCR_T::RSTSRC: RSTS_POR Mask */

/* GCR IPRSTC1 Bit Field Definitions */
#define SYS_IPRSTC1_HDIV_RST_Pos                4                                   /*!< GCR_T::IPRSTC1: HDIV_RST Position */
#define SYS_IPRSTC1_HDIV_RST_Msk                (1ul << SYS_IPRSTC1_HDIV_RST_Pos)   /*!< GCR_T::IPRSTC1: HDIV_RST Mask */

#define SYS_IPRSTC1_CPU_RST_Pos                 1                                   /*!< GCR_T::IPRSTC1: CPU_RST Position */
#define SYS_IPRSTC1_CPU_RST_Msk                 (1ul << SYS_IPRSTC1_CPU_RST_Pos)    /*!< GCR_T::IPRSTC1: CPU_RST Mask */

#define SYS_IPRSTC1_CHIP_RST_Pos                0                                   /*!< GCR_T::IPRSTC1: CHIP_RST Position */
#define SYS_IPRSTC1_CHIP_RST_Msk                (1ul << SYS_IPRSTC1_CHIP_RST_Pos)   /*!< GCR_T::IPRSTC1: CHIP_RST Mask */

/* GCR IPRSTC2 Bit Field Definitions */
#define SYS_IPRSTC2_OPA_RST_Pos                 29                                  /*!< GCR_T::IPRSTC2: OPA_RST Position */
#define SYS_IPRSTC2_OPA_RST_Msk                 (1ul << SYS_IPRSTC2_OPA_RST_Pos)    /*!< GCR_T::IPRSTC2: OPA_RST Mask */

#define SYS_IPRSTC2_EADC_RST_Pos                28                                  /*!< GCR_T::IPRSTC2: EADC_RST Position */
#define SYS_IPRSTC2_EADC_RST_Msk                (1ul << SYS_IPRSTC2_EADC_RST_Pos)   /*!< GCR_T::IPRSTC2: EADC_RST Mask */

#define SYS_IPRSTC2_ECAP1_RST_Pos               27                                  /*!< GCR_T::IPRSTC2: ECAP1_RST Position */
#define SYS_IPRSTC2_ECAP1_RST_Msk               (1ul << SYS_IPRSTC2_ECAP1_RST_Pos)  /*!< GCR_T::IPRSTC2: ECAP1_RST Mask */

#define SYS_IPRSTC2_ECAP0_RST_Pos               26                                  /*!< GCR_T::IPRSTC2: ECAP0_RST Position */
#define SYS_IPRSTC2_ECAP0_RST_Msk               (1ul << SYS_IPRSTC2_ECAP0_RST_Pos)  /*!< GCR_T::IPRSTC2: ECAP0_RST Mask */

#define SYS_IPRSTC2_ACMP_RST_Pos                22                                  /*!< GCR_T::IPRSTC2: ACMP_RST Position */
#define SYS_IPRSTC2_ACMP_RST_Msk                (1ul << SYS_IPRSTC2_ACMP_RST_Pos)   /*!< GCR_T::IPRSTC2: ACMP_RST Mask */

#define SYS_IPRSTC2_EPWM1_RST_Pos               21                                  /*!< GCR_T::IPRSTC2: EPWM1_RST Position */
#define SYS_IPRSTC2_EPWM1_RST_Msk               (1ul << SYS_IPRSTC2_EPWM1_RST_Pos)  /*!< GCR_T::IPRSTC2: EPWM1_RST Mask */

#define SYS_IPRSTC2_EPWM0_RST_Pos               20                                  /*!< GCR_T::IPRSTC2: EPWM0_RST Position */
#define SYS_IPRSTC2_EPWM0_RST_Msk               (1ul << SYS_IPRSTC2_EPWM0_RST_Pos)  /*!< GCR_T::IPRSTC2: EPWM0_RST Mask */

#define SYS_IPRSTC2_BPWM0_RST_Pos               19                                  /*!< GCR_T::IPRSTC2: BPWM0_RST Position */
#define SYS_IPRSTC2_BPWM0_RST_Msk               (1ul << SYS_IPRSTC2_BPWM0_RST_Pos)  /*!< GCR_T::IPRSTC2: BPWM0_RST Mask */

#define SYS_IPRSTC2_UART1_RST_Pos               17                                  /*!< GCR_T::IPRSTC2: UART1_RST Position */
#define SYS_IPRSTC2_UART1_RST_Msk               (1ul << SYS_IPRSTC2_UART1_RST_Pos)  /*!< GCR_T::IPRSTC2: UART1_RST Mask */

#define SYS_IPRSTC2_UART0_RST_Pos               16                                  /*!< GCR_T::IPRSTC2: UART0_RST Position */
#define SYS_IPRSTC2_UART0_RST_Msk               (1ul << SYS_IPRSTC2_UART0_RST_Pos)  /*!< GCR_T::IPRSTC2: UART0_RST Mask */

#define SYS_IPRSTC2_SPI2_RST_Pos                14                                  /*!< GCR_T::IPRSTC2: SPI2_RST Position */
#define SYS_IPRSTC2_SPI2_RST_Msk                (1ul << SYS_IPRSTC2_SPI2_RST_Pos)   /*!< GCR_T::IPRSTC2: SPI2_RST Mask */

#define SYS_IPRSTC2_SPI1_RST_Pos                13                                  /*!< GCR_T::IPRSTC2: SPI1_RST Position */
#define SYS_IPRSTC2_SPI1_RST_Msk                (1ul << SYS_IPRSTC2_SPI1_RST_Pos)   /*!< GCR_T::IPRSTC2: SPI1_RST Mask */

#define SYS_IPRSTC2_SPI0_RST_Pos                12                                  /*!< GCR_T::IPRSTC2: SPI0_RST Position */
#define SYS_IPRSTC2_SPI0_RST_Msk                (1ul << SYS_IPRSTC2_SPI0_RST_Pos)   /*!< GCR_T::IPRSTC2: SPI0_RST Mask */

#define SYS_IPRSTC2_I2C0_RST_Pos                8                                   /*!< GCR_T::IPRSTC2: I2C0_RST Position */
#define SYS_IPRSTC2_I2C0_RST_Msk                (1ul << SYS_IPRSTC2_I2C0_RST_Pos)   /*!< GCR_T::IPRSTC2: I2C0_RST Mask */

#define SYS_IPRSTC2_TMR3_RST_Pos                5                                   /*!< GCR_T::IPRSTC2: TMR3_RST Position */
#define SYS_IPRSTC2_TMR3_RST_Msk                (1ul << SYS_IPRSTC2_TMR3_RST_Pos)   /*!< GCR_T::IPRSTC2: TMR3_RST Mask */

#define SYS_IPRSTC2_TMR2_RST_Pos                4                                   /*!< GCR_T::IPRSTC2: TMR2_RST Position */
#define SYS_IPRSTC2_TMR2_RST_Msk                (1ul << SYS_IPRSTC2_TMR2_RST_Pos)   /*!< GCR_T::IPRSTC2: TMR2_RST Mask */

#define SYS_IPRSTC2_TMR1_RST_Pos                3                                   /*!< GCR_T::IPRSTC2: TMR1_RST Position */
#define SYS_IPRSTC2_TMR1_RST_Msk                (1ul << SYS_IPRSTC2_TMR1_RST_Pos)   /*!< GCR_T::IPRSTC2: TMR1_RST Mask */

#define SYS_IPRSTC2_TMR0_RST_Pos                2                                   /*!< GCR_T::IPRSTC2: TMR0_RST Position */
#define SYS_IPRSTC2_TMR0_RST_Msk                (1ul << SYS_IPRSTC2_TMR0_RST_Pos)   /*!< GCR_T::IPRSTC2: TMR0_RST Mask */

#define SYS_IPRSTC2_GPIO_RST_Pos                1                                   /*!< GCR_T::IPRSTC2: GPIO_RST Position */
#define SYS_IPRSTC2_GPIO_RST_Msk                (1ul << SYS_IPRSTC2_GPIO_RST_Pos)   /*!< GCR_T::IPRSTC2: GPIO_RST Mask */

/* GCR BODCR Bit Field Definitions */
#define SYS_BODCR_LVR_EN_Pos                    7                                   /*!< GCR_T::BODCR: LVR_EN Position */
#define SYS_BODCR_LVR_EN_Msk                    (1ul << SYS_BODCR_LVR_EN_Pos)       /*!< GCR_T::BODCR: LVR_EN Mask */

#define SYS_BODCR_BOD_OUT_Pos                   6                                   /*!< GCR_T::BODCR: BOD_OUT Position */
#define SYS_BODCR_BOD_OUT_Msk                   (1ul << SYS_BODCR_BOD_OUT_Pos)      /*!< GCR_T::BODCR: BOD_OUT Mask */

#define SYS_BODCR_BOD_LPM_Pos                   5                                   /*!< GCR_T::BODCR: BOD_LPM Position */
#define SYS_BODCR_BOD_LPM_Msk                   (1ul << SYS_BODCR_BOD_LPM_Pos)      /*!< GCR_T::BODCR: BOD_LPM Mask */

#define SYS_BODCR_BOD_INTF_Pos                  4                                   /*!< GCR_T::BODCR: BOD_INTF Position */
#define SYS_BODCR_BOD_INTF_Msk                  (1ul << SYS_BODCR_BOD_INTF_Pos)     /*!< GCR_T::BODCR: BOD_INTF Mask */

#define SYS_BODCR_BOD_RSTEN_Pos                 3                                   /*!< GCR_T::BODCR: BOD_RSTEN Position */
#define SYS_BODCR_BOD_RSTEN_Msk                 (1ul << SYS_BODCR_BOD_RSTEN_Pos)    /*!< GCR_T::BODCR: BOD_RSTEN Mask */

#define SYS_BODCR_BOD_VL_Pos                    1                                   /*!< GCR_T::BODCR: BOD_VL Position */
#define SYS_BODCR_BOD_VL_Msk                    (3ul << SYS_BODCR_BOD_VL_Pos)       /*!< GCR_T::BODCR: BOD_VL Mask */

#define SYS_BODCR_BOD_EN_Pos                    0                                   /*!< GCR_T::BODCR: BOD_EN Position */
#define SYS_BODCR_BOD_EN_Msk                    (1ul << SYS_BODCR_BOD_EN_Pos)       /*!< GCR_T::BODCR: BOD_EN Mask */

/* GCR TEMPCR Bit Field Definitions */
#define SYS_TEMPCR_VTEMP_EN_Pos                 0                                   /*!< GCR_T::TEMPCR: VTEMP_EN Position */
#define SYS_TEMPCR_VTEMP_EN_Msk                 (1ul << SYS_TEMPCR_VTEMP_EN_Pos)    /*!< GCR_T::TEMPCR: VTEMP_EN Mask */

/* GCR PORCR Bit Field Definitions */
#define SYS_PORCR_POR_DIS_CODE_Pos              0                                           /*!< GCR_T::PORCR: POR_DIS_CODE Position */
#define SYS_PORCR_POR_DIS_CODE_Msk              (0xFFFFul << SYS_PORCR_POR_DIS_CODE_Pos)    /*!< GCR_T::PORCR: POR_DIS_CODE Mask */

/* GCR P0_MFP Bit Field Definitions */
#define SYS_P0_MFP_P0_TYPE_Pos                  16                                      /*!< GCR_T::P0_MFP: P0_TYPE Position */
#define SYS_P0_MFP_P0_TYPE_Msk                  (0xFFul << SYS_P0_MFP_P0_TYPE_Pos)      /*!< GCR_T::P0_MFP: P0_TYPE Mask */

#define SYS_P0_MFP_P0_ALT_Pos                   8                                       /*!< GCR_T::P0_MFP: P0_ALT Position */
#define SYS_P0_MFP_P0_ALT_Msk                   (0xFFul << SYS_P0_MFP_P0_ALT_Pos)       /*!< GCR_T::P0_MFP: P0_ALT Mask */

#define SYS_P0_MFP_P0_MFP_Pos                   0                                       /*!< GCR_T::P0_MFP: P0_MFP Position */
#define SYS_P0_MFP_P0_MFP_Msk                   (0xFFul << SYS_P0_MFP_P0_MFP_Pos)       /*!< GCR_T::P0_MFP: P0_MFP Mask */

/* GCR P1_MFP Bit Field Definitions */
#define SYS_P1_MFP_P1_TYPE_Pos                  16                                      /*!< GCR_T::P1_MFP: P1_TYPE Position */
#define SYS_P1_MFP_P1_TYPE_Msk                  (0xFFul << SYS_P1_MFP_P1_TYPE_Pos)      /*!< GCR_T::P1_MFP: P1_TYPE Mask */

#define SYS_P1_MFP_P1_MFP_Pos                   0                                       /*!< GCR_T::P1_MFP: P1_MFP Position */
#define SYS_P1_MFP_P1_MFP_Msk                   (0xFFul << SYS_P1_MFP_P1_MFP_Pos)       /*!< GCR_T::P1_MFP: P1_MFP Mask */

/* GCR P2_MFP Bit Field Definitions */
#define SYS_P2_MFP_P2_TYPE_Pos                  16                                      /*!< GCR_T::P2_MFP: P2_TYPE Position */
#define SYS_P2_MFP_P2_TYPE_Msk                  (0xFFul << SYS_P2_MFP_P2_TYPE_Pos)      /*!< GCR_T::P2_MFP: P2_TYPE Mask */

#define SYS_P2_MFP_P2_ALT_Pos                   8                                       /*!< GCR_T::P2_MFP: P2_ALT Position */
#define SYS_P2_MFP_P2_ALT_Msk                   (0xFFul << SYS_P2_MFP_P2_ALT_Pos)       /*!< GCR_T::P2_MFP: P2_ALT Mask */

#define SYS_P2_MFP_P2_MFP_Pos                   0                                       /*!< GCR_T::P2_MFP: P2_MFP Position */
#define SYS_P2_MFP_P2_MFP_Msk                   (0xFFul << SYS_P2_MFP_P2_MFP_Pos)       /*!< GCR_T::P2_MFP: P2_MFP Mask */

/* GCR P3_MFP Bit Field Definitions */
#define SYS_P3_MFP_P3_TYPE_Pos                  16                                      /*!< GCR_T::P3_MFP: P3_TYPE Position */
#define SYS_P3_MFP_P3_TYPE_Msk                  (0xFFul << SYS_P3_MFP_P3_TYPE_Pos)      /*!< GCR_T::P3_MFP: P3_TYPE Mask */

#define SYS_P3_MFP_P3_ALT_Pos                   8                                       /*!< GCR_T::P3_MFP: P3_ALT Position */
#define SYS_P3_MFP_P3_ALT_Msk                   (0xFFul << SYS_P3_MFP_P3_ALT_Pos)       /*!< GCR_T::P3_MFP: P3_ALT Mask */

#define SYS_P3_MFP_P3_MFP_Pos                   0                                       /*!< GCR_T::P3_MFP: P3_MFP Position */
#define SYS_P3_MFP_P3_MFP_Msk                   (0xFFul << SYS_P3_MFP_P3_MFP_Pos)       /*!< GCR_T::P3_MFP: P3_MFP Mask */

/* GCR P4_MFP Bit Field Definitions */
#define SYS_P4_MFP_P4_TYPE_Pos                  16                                      /*!< GCR_T::P4_MFP: P4_TYPE Position */
#define SYS_P4_MFP_P4_TYPE_Msk                  (0xFFul << SYS_P4_MFP_P4_TYPE_Pos)      /*!< GCR_T::P4_MFP: P4_TYPE Mask */

#define SYS_P4_MFP_P4_ALT_Pos                   8                                       /*!< GCR_T::P4_MFP: P4_ALT Position */
#define SYS_P4_MFP_P4_ALT_Msk                   (0xFFul << SYS_P4_MFP_P4_ALT_Pos)       /*!< GCR_T::P4_MFP: P4_ALT Mask */

#define SYS_P4_MFP_P4_MFP_Pos                   0                                       /*!< GCR_T::P4_MFP: P4_MFP Position */
#define SYS_P4_MFP_P4_MFP_Msk                   (0xFFul << SYS_P4_MFP_P4_MFP_Pos)       /*!< GCR_T::P4_MFP: P4_MFP Mask */

/* GCR P5_MFP Bit Field Definitions */
#define SYS_P5_MFP_P5_TYPE_Pos                  16                                      /*!< GCR_T::P5_MFP: P5_TYPE Position */
#define SYS_P5_MFP_P5_TYPE_Msk                  (0xFFul << SYS_P5_MFP_P5_TYPE_Pos)      /*!< GCR_T::P5_MFP: P5_TYPE Mask */

#define SYS_P5_MFP_P5_ALT_Pos                   8                                       /*!< GCR_T::P5_MFP: P5_ALT Position */
#define SYS_P5_MFP_P5_ALT_Msk                   (0xFFul << SYS_P5_MFP_P5_ALT_Pos)       /*!< GCR_T::P5_MFP: P5_ALT Mask */

#define SYS_P5_MFP_P5_MFP_Pos                   0                                       /*!< GCR_T::P5_MFP: P5_MFP Position */
#define SYS_P5_MFP_P5_MFP_Msk                   (0xFFul << SYS_P5_MFP_P5_MFP_Pos)       /*!< GCR_T::P5_MFP: P5_MFP Mask */

/* GCR P6_MFP Bit Field Definitions */
#define SYS_P6_MFP_P6_TYPE_Pos                  16                                      /*!< GCR_T::P6_MFP: P6_TYPE Position */
#define SYS_P6_MFP_P6_TYPE_Msk                  (0xFFul << SYS_P6_MFP_P6_TYPE_Pos)      /*!< GCR_T::P6_MFP: P6_TYPE Mask */

#define SYS_P6_MFP_P6_MFP_Pos                   0                                       /*!< GCR_T::P6_MFP: P6_MFP Position */
#define SYS_P6_MFP_P6_MFP_Msk                   (0xFFul << SYS_P6_MFP_P6_MFP_Pos)       /*!< GCR_T::P6_MFP: P6_MFP Mask */

/* GCR P7_MFP Bit Field Definitions */
#define SYS_P7_MFP_P7_TYPE_Pos                  16                                      /*!< GCR_T::P7_MFP: P7_TYPE Position */
#define SYS_P7_MFP_P7_TYPE_Msk                  (0xFFul << SYS_P7_MFP_P7_TYPE_Pos)      /*!< GCR_T::P7_MFP: P7_TYPE Mask */

#define SYS_P7_MFP_P7_MFP_Pos                   0                                       /*!< GCR_T::P7_MFP: P7_MFP Position */
#define SYS_P7_MFP_P7_MFP_Msk                   (0xFFul << SYS_P7_MFP_P7_MFP_Pos)       /*!< GCR_T::P7_MFP: P7_MFP Mask */

/* GCR P8_MFP Bit Field Definitions */
#define SYS_P8_MFP_P8_TYPE_Pos                  16                                      /*!< GCR_T::P8_MFP: P8_TYPE Position */
#define SYS_P8_MFP_P8_TYPE_Msk                  (0xFFul << SYS_P8_MFP_P8_TYPE_Pos)      /*!< GCR_T::P8_MFP: P8_TYPE Mask */

#define SYS_P8_MFP_P8_MFP_Pos                   0                                       /*!< GCR_T::P8_MFP: P8_MFP Position */
#define SYS_P8_MFP_P8_MFP_Msk                   (0xFFul << SYS_P8_MFP_P8_MFP_Pos)       /*!< GCR_T::P8_MFP: P8_MFP Mask */

/* GCR P9_MFP Bit Field Definitions */
#define SYS_P9_MFP_P9_TYPE_Pos                  16                                      /*!< GCR_T::P9_MFP: P9_TYPE Position */
#define SYS_P9_MFP_P9_TYPE_Msk                  (0xFFul << SYS_P9_MFP_P9_TYPE_Pos)      /*!< GCR_T::P9_MFP: P9_TYPE Mask */

#define SYS_P9_MFP_P9_MFP_Pos                   0                                       /*!< GCR_T::P9_MFP: P9_MFP Position */
#define SYS_P9_MFP_P9_MFP_Msk                   (0xFFul << SYS_P9_MFP_P9_MFP_Pos)       /*!< GCR_T::P9_MFP: P9_MFP Mask */

/* GCR PA_MFP Bit Field Definitions */
#define SYS_PA_MFP_PA_TYPE_Pos                  16                                      /*!< GCR_T::PA_MFP: PA_TYPE Position */
#define SYS_PA_MFP_PA_TYPE_Msk                  (0xFFul << SYS_PA_MFP_PA_TYPE_Pos)      /*!< GCR_T::PA_MFP: PA_TYPE Mask */

#define SYS_PA_MFP_PA_ALT_Pos                   8                                       /*!< GCR_T::PA_MFP: PA_ALT Position */
#define SYS_PA_MFP_PA_ALT_Msk                   (0xFFul << SYS_PA_MFP_PA_ALT_Pos)       /*!< GCR_T::PA_MFP: PA_ALT Mask */

#define SYS_PA_MFP_PA_MFP_Pos                   0                                       /*!< GCR_T::PA_MFP: PA_MFP Position */
#define SYS_PA_MFP_PA_MFP_Msk                   (0xFFul << SYS_PA_MFP_PA_MFP_Pos)       /*!< GCR_T::PA_MFP: PA_MFP Mask */

/* GCR REGWRPROT Bit Field Definitions */
#define SYS_REGWRPROT_REGWRPROT_Pos             0                                       /*!< GCR_T::REGWRPROT: REGWRPROT Position */
#define SYS_REGWRPROT_REGWRPROT_Msk             (0xFFul << SYS_REGWRPROT_REGWRPROT_Pos) /*!< GCR_T::REGWRPROT: REGWRPROT Mask */

#define SYS_REGWRPROT_REGPROTDIS_Pos            0                                       /*!< GCR_T::REGWRPROT: REGPROTDIS Position */
#define SYS_REGWRPROT_REGPROTDIS_Msk            (1ul << SYS_REGWRPROT_REGPROTDIS_Pos)   /*!< GCR_T::REGWRPROT: REGPROTDIS Mask */


typedef struct
{


    /**
     * @var GCR_INT_T::IRQ_SRC[32]
     * Offset: 0x00-0x7C  IRQn(n=0~31) Interrupt Source Identity Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |INT_SRC   |Interrupt Source Identity
     * |        |          |IRQ_SRC[0].0 - BOD INT
     * |        |          |IRQ_SRC[1].0 - WDT INT
     * |        |          |IRQ_SRC[1].1 - WWDT INT
     * |        |          |IRQ_SRC[2].0 - EINT0, external interrupt 0 from P3.2
     * |        |          |IRQ_SRC[3].0 - EINT1, external interrupt 1 from P3.3
     * |        |          |IRQ_SRC[4].0 - P0 INT
     * |        |          |IRQ_SRC[4].1 - P1 INT
     * |        |          |IRQ_SRC[4].2 - P2 INT
     * |        |          |IRQ_SRC[4].3 - P3_INT
     * |        |          |IRQ_SRC[4].4 - P4_INT
     * |        |          |IRQ_SRC[5].0 - P5 INT
     * |        |          |IRQ_SRC[5].1 - P6 INT
     * |        |          |IRQ_SRC[5].2 - P7 INT
     * |        |          |IRQ_SRC[5].3 - P8 INT
     * |        |          |IRQ_SRC[5].4 - P9 INT
     * |        |          |IRQ_SRC[5].5 - PA INT
     * |        |          |IRQ_SRC[6].0 - BPWM0 CH0 INT
     * |        |          |IRQ_SRC[6].1 - BPWM0 CH1 INT
     * |        |          |IRQ_SRC[7].0 - EADC0 INT
     * |        |          |IRQ_SRC[8].0 - TIMER0 INT
     * |        |          |IRQ_SRC[9].0 - TIMER1 INT
     * |        |          |IRQ_SRC[10].0 - TIMER2 INT
     * |        |          |IRQ_SRC[11].0 - TIMER3 INT
     * |        |          |IRQ_SRC[12].0 - UART0 INT
     * |        |          |IRQ_SRC[13].0 - UART1 INT
     * |        |          |IRQ_SRC[14].0 - SPI0 INT
     * |        |          |IRQ_SRC[15].0 - SPI1 INT
     * |        |          |IRQ_SRC[16].0 - SPI2 INT
     * |        |          |IRQ_SRC[18].0 - I2C0 INT
     * |        |          |IRQ_SRC[21].0 - EPWM0 INT
     * |        |          |IRQ_SRC[22].0 - EPWM1 INT
     * |        |          |IRQ_SRC[23].0 - ECAP0 INT
     * |        |          |IRQ_SRC[24].0 - ECAP1 INT
     * |        |          |IRQ_SRC[25].0 - ACMP INT
     * |        |          |IRQ_SRC[28].0 - Power Down Wake up INT
     * |        |          |IRQ_SRC[29].0 - EADC1 INT
     * |        |          |IRQ_SRC[30].0 - EADC2 INT
     * |        |          |IRQ_SRC[31].0 - EADC3 INT
     * @var GCR_INT_T::NMI_SEL
     * Offset: 0x80  NMI Interrupt Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |NMI_SEL   |NMI interrupt source selection
     * |        |          |The NMI interrupt to Cortex-M0 can be selected from one of IRQ0~IRQ31 by setting NMI_SEL with IRQ number.
     * |        |          |The default NMI interrupt is assigned as IRQ0 interrupt if NMI is enabled by setting NMI_SEL[8].
     * |[8]     |NMI_EN    |NMI interrupt enable (Write Protect)
     * |        |          |0 = IRQ0~31 assigned to NMI interrupt Disabled. (NMI still can be software triggered by setting its pending flag.)
     * |        |          |1 = IRQ0~31 assigned to NMI interrupt Enabled.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * @var GCR_INT_T::MCU_IRQ
     * Offset: 0x84  MCU Interrupt Request Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |MCU_IRQ   |MCU IRQ Source Register
     * |        |          |The MCU_IRQ collects all the interrupts from the peripherals and generates the synchronous interrupt to Cortex-M0.
     * |        |          |When the MCU_IRQ[n] is 0, setting:
     * |        |          |0 = No effect.
     * |        |          |1 = Generate an interrupt to Cortex_M0 NVIC[n].
     * |        |          |When the MCU_IRQ[n] is 1 (means an interrupt is assert), setting:
     * |        |          |0 = No effect.
     * |        |          |1 = Clear the interrupt and MCU_IRQ[n].
     * @var GCR_INT_T::MCU_IRQCR
     * Offset: 0x88  MCU Interrupt Request Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |FAST_IRQ  |Fast IRQ Latency Enable Control
     * |        |          |0 = MCU IRQ latency is fixed at 13 clock cycles of HCLK, MCU will enter IRQ handler after this fixed latency when interrupt happened.
     * |        |          |1 = MCU IRQ latency will not fixed, MCU will enter IRQ handler as soon as possible when interrupt happened.
     */

    __I  uint32_t IRQSRC[32];   /* Offset: 0x00-0x7C  IRQn(n=0~31) Interrupt Source Identity Register               */
    __IO uint32_t NMISEL;       /* Offset: 0x80  NMI Interrupt Source Select Control Register                       */
    __IO uint32_t MCUIRQ;       /* Offset: 0x84  MCU Interrupt Request Source Register                              */
    __IO uint32_t MCUIRQCR;     /* Offset: 0x88  MCU Interrupt Request Control Register                             */

} GCR_INT_T;



/* INT IRQ_SRC Bit Field Definitions */
#define INT_IRQ_SRC_INT_SRC_Pos                 0                                   /*!< GCR_INT_T::IRQ_SRC: INT_SRC Position */
#define INT_IRQ_SRC_INT_SRC_Msk                 (0x1Ful << INT_IRQ_SRC_INT_SRC_Pos) /*!< GCR_INT_T::IRQ_SRC: INT_SRC Msk */

/* INT NMI_SEL Bit Field Definitions */
#define INT_NMI_SEL_NMI_EN_Pos                  8                                   /*!< GCR_INT_T::NMISEL: NMI_EN Position */
#define INT_NMI_SEL_NMI_EN_Msk                  (1ul << INT_NMI_SEL_NMI_EN_Pos)     /*!< GCR_INT_T::NMISEL: NMI_EN Mask */

#define INT_NMI_SEL_NMI_SEL_Pos                 0                                   /*!< GCR_INT_T::NMISEL: NMI_SEL Position */
#define INT_NMI_SEL_NMI_SEL_Msk                 (0x1Ful << INT_NMI_SEL_NMI_SEL_Pos) /*!< GCR_INT_T::NMISEL: NMI_SEL Mask */

/* INT MCU_IRQ Bit Field Definitions */
#define INT_MCU_IRQ_MCU_IRQ_Pos                 0                                           /*!< GCR_INT_T::MCU_IRQ: MCU_IRQ Position */
#define INT_MCU_IRQ_MCU_IRQ_Msk                 (0xFFFFFFFFul << INT_MCU_IRQ_MCU_IRQ_Pos)   /*!< GCR_INT_T::MCU_IRQ: MCU_IRQ Mask */

/* INT MCU_IRQCR Bit Field Definitions */
#define INT_MCU_IRQCR_FAST_IRQ_Pos              0                                   /*!< GCR_INT_T::MCU_IRQCR: FAST_IRQ Position */
#define INT_MCU_IRQCR_FAST_IRQ_Msk              (1ul << INT_MCU_IRQCR_FAST_IRQ_Pos) /*!< GCR_INT_T::MCU_IRQCR: FAST_IRQ Mask */
/*@}*/ /* end of group SYS_CONST */
/*@}*/ /* end of group SYS */




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
     * |        |          |Timer input clock source is divided by (PRESCALE+1) before it is fed to the timer up counter.
     * |        |          |If this field is 0 (PRESCALE = 0), then there is no scaling.
     * |[16]    |TDR_EN    |Data Load Enable
     * |        |          |When this bit is set, timer counter value (TDR) will be updated continuously to monitor internal 24-bit up counter value as the counter is counting.
     * |        |          |0 = Timer Data Register update Disabled.
     * |        |          |1 = Timer Data Register update Enabled while timer counter is active.
     * |[24]    |CTB       |Counter Mode Enable Bit
     * |        |          |This bit is for external counting pin function enabled.
     * |        |          |0 = Event counter mode Disabled.
     * |        |          |1 = Event counter mode Enabled.
     * |[25]    |CACT      |Timer Active Status Bit (Read Only)
     * |        |          |This bit indicates the up-timer status.
     * |        |          |0 = Timer is not active.
     * |        |          |1 = Timer is active.
     * |[26]    |CRST      |Timer Counter Reset Bit
     * |        |          |Setting this bit will reset the 24-bit up counter value TDR and also force CEN (TCSR[30]) to 0 if CACT (TCSR[25]) is 1.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset internal 8-bit prescale counter, 24-bit up counter value and CEN bit.
     * |[28:27] |MODE      |Timer Counting Mode Select
     * |        |          |00 = The Timer controller is operated in One-shot mode.
     * |        |          |01 = The Timer controller is operated in Periodic mode.
     * |        |          |10 = The Timer controller is operated in Toggle-output mode.
     * |        |          |11 = The Timer controller is operated in Continuous Counting mode.
     * |[29]    |IE        |Interrupt Enable Bit
     * |        |          |0 = Timer Interrupt Disabled.
     * |        |          |1 = Timer Interrupt Enabled.
     * |        |          |Note: If this bit is enabled, when the timer interrupt flag TIF is set to 1, the timer interrupt signal is generated and inform to CPU.
     * |[30]    |CEN       |Timer Enable Bit
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note1: In stop status, and then set CEN to 1 will enable the 24-bit up counter to keep counting from the last stop counting value.
     * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (TCSR[28:27] = 00) when the timer interrupt flag TIF (TISR[0]) is generated.
     * |[31]    |DBGACK_TMR|ICE Debug Mode Acknowledge Disable (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgment effects TIMER counting.
     * |        |          |TIMER counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgment Disabled.
     * |        |          |TIMER counter will keep going no matter CPU is held by ICE or not.
     * |        |          |Note: This bit is write protected. Refer to the REGWRPROT register.
     * @var TIMER_T::TCMPR
     * Offset: 0x04  Timer Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |TCMP      |Timer Compared Value
     * |        |          |TCMP is a 24-bit compared value register.
     * |        |          |When the internal 24-bit up counter value is equal to TCMP value, the TIF (TISR[0] Timer Interrupt Flag) will set to 1.
     * |        |          |Time-out period = (Period of timer clock input) * (8-bit PRESCALE + 1) * (24-bit TCMP).
     * |        |          |Note1: Never write 0x0 or 0x1 in TCMP field, or the core will run into unknown state.
     * |        |          |Note2: When timer is operating at continuous counting mode, the 24-bit up counter will keep counting continuously even if user writes a new value into TCMP field.
     * |        |          |But if timer is operating at other modes, the 24-bit up counter will restart counting from 0 and using newest TCMP value to be the timer compared value while user writes a new value into TCMP field.
     * @var TIMER_T::TISR
     * Offset: 0x08  Timer Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Timer Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of Timer while 24-bit timer up counter TDR value reaches to TCMP (TCMPR[23:0]) value.
     * |        |          |0 = No effect.
     * |        |          |1 = TDR value matches the TCMP value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var TIMER_T::TDR
     * Offset: 0x0C  Timer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |TDR       |Timer Data Register
     * |        |          |This field can be reflected the internal 24-bit timer counter value or external event input counter value from TMx (x=0~3) pin.
     * |        |          |If CTB (TCSR[24]) is 0, user can read TDR value for getting current 24- bit counter value.
     * |        |          |If CTB (TCSR[24]) is 1, user can read TDR value for getting current 24- bit event input counter value.
     * @var TIMER_T::TCAP
     * Offset: 0x10  Timer Capture Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |TCAP      |Timer Capture Data Register
     * |        |          |When TEXEN (TEXCON[3]) bit is set, RSTCAPn (TEXCON[4]) bit is 0, and a transition on TMx (x=0~3) pin matched the TEX_EDGE (TEXCON[2:1]) setting.
     * |        |          |TEXIF (TEXISR[0]) will set to 1 and the current timer counter value TDR will be auto-loaded into this TCAP field.
     * @var TIMER_T::TEXCON
     * Offset: 0x14  Timer External Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TX_PHASE  |Timer External Count Phase
     * |        |          |This bit indicates the detection phase of external counting pin TMx (x= 0~3).
     * |        |          |0 = A Falling edge of external counting pin will be counted.
     * |        |          |1 = A Rising edge of external counting pin will be counted.
     * |[2:1]   |TEX_EDGE  |Timer External Capture Pin Edge Detect
     * |        |          |00 = A Falling edge on TMx (x= 0~3) pin will be detected.
     * |        |          |01 = A Rising edge on TMx (x= 0~3) pin will be detected.
     * |        |          |10 = Either Rising or Falling edge on TMx (x= 0~3) pin will be detected.
     * |        |          |11 = Reserved.
     * |[3]     |TEXEN     |Timer External Capture Pin Enable
     * |        |          |This bit enables the TMx pin.
     * |        |          |0 = TMx (x= 0~3) pin Disabled.
     * |        |          |1 = TMx (x= 0~3) pin Enabled.
     * |[4]     |RSTCAPSEL |Capture Function Selection
     * |        |          |0 = External Capture Mode Enabled.
     * |        |          |1 = External Reset Mode Enabled.
     * |        |          |Note1: When RSTCAPn is 0, transition on TMx (x= 0~3) pin is using to save the 24-bit timer counter value.
     * |        |          |Note2: When RSTCAPn is 1, transition on TMx (x= 0~3) pin is using to reset the 24-bit timer counter value.
     * |[5]     |TEXIEN    |Timer External Capture Interrupt Enable
     * |        |          |0 = TMx (x= 0~3) pin detection Interrupt Disabled.
     * |        |          |1 = TMx (x= 0~3) pin detection Interrupt Enabled.
     * |        |          |Note: TEXIEN is used to enable timer external interrupt. If TEXIEN enabled, timer will rise an interrupt when TEXIF (TEXISR[0]) is 1.
     * |        |          |For example, while TEXIEN = 1, TEXEN = 1, and TEX_EDGE = 00, a 1 to 0 transition on the TMx pin will cause the TEXIF to be set then the interrupt signal is generated and sent to NVIC to inform CPU.
     * |[6]     |TEXDB     |Timer External Capture Pin De-Bounce Enable
     * |        |          |0 = TMx (x= 0~3) pin de-bounce Disabled.
     * |        |          |1 = TMx (x= 0~3) pin de-bounce Enabled.
     * |        |          |Note: If this bit is enabled, the edge detection of TMx pin is detected with de-bounce circuit.
     * |[7]     |TCDB      |Timer Counter Pin De-Bounce Enable
     * |        |          |0 = TMx (x= 0~3) pin de-bounce Disabled.
     * |        |          |1 = TMx (x= 0~3) pin de-bounce Enabled.
     * |        |          |Note: If this bit is enabled, the edge detection of TMx pin is detected with de-bounce circuit.
     * @var TIMER_T::TEXISR
     * Offset: 0x18  Timer External Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TEXIF     |Timer External Capture Interrupt Flag
     * |        |          |This bit indicates the timer external capture interrupt flag status.
     * |        |          |0 = TMx (x= 0~3) pin interrupt did not occur.
     * |        |          |1 = TMx (x= 0~3) pin interrupt occurred.
     * |        |          |Note1: This bit is cleared by writing 1 to it.
     * |        |          |Note2: When TEXEN (TEXCON[3]) bit is set, RSTCAPn (TEXCON[4]) bit is 0, and a transition on TMx (x= 0~3) pin matched the TEX_EDGE (TEXCON[2:1]) setting, this bit will set to 1 by hardware.
     * |        |          |Note3: There is a new incoming capture event detected before CPU clearing the TEXIF status. If the above condition occurred, the Timer will keep register TCAP unchanged and drop the new capture value.
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
    @addtogroup TIMER_CONST TMR Bit Field Definition
    Constant Definitions for TMR Controller
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

#define TIMER_TCSR_TDR_EN_Pos       16                                          /*!< TIMER_T::TCSR: TDR_EN Position */
#define TIMER_TCSR_TDR_EN_Msk       (1ul << TIMER_TCSR_TDR_EN_Pos)              /*!< TIMER_T::TCSR: TDR_EN Mask */

#define TIMER_TCSR_PRESCALE_Pos     0                                           /*!< TIMER_T::TCSR: PRESCALE Position */
#define TIMER_TCSR_PRESCALE_Msk     (0xFFul << TIMER_TCSR_PRESCALE_Pos)         /*!< TIMER_T::TCSR: PRESCALE Mask */

/* TIMER TCMPR Bit Field Definitions */
#define TIMER_TCMP_TCMP_Pos         0                                           /*!< TIMER_T::TCMPR: TCMP Position */
#define TIMER_TCMP_TCMP_Msk         (0xFFFFFFul << TIMER_TCMP_TCMP_Pos)         /*!< TIMER_T::TCMPR: TCMP Mask */

/* TIMER TISR Bit Field Definitions */
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
    * |[7:0]   |DATA      |UART Data Register
    * |        |          |By writing one byte to this register, the data byte will be stored in transmitter FIFO. 
    * |        |          |The UART Controller will send out the data stored in transmitter FIFO top location through the UART_TXD pin (LSB first). 
    * |        |          |By reading this register, the UART will return an 8-bit data received from UART_RXD pin (LSB first).
    * @var UART_T::THR
    * Offset: 0x00  UART Transmit Holding Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[7:0]   |THR       |Transmit Holding Register
    * |        |          |By writing one byte to this register, the data byte will be stored in transmitter FIFO. 
    * |        |          |The UART Controller will send out the data stored in transmitter FIFO top location through the UART_TXD pin (LSB first). 
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
    * |[0]     |RDA_IEN   |Receive Data Available Interrupt Enable Bit
    * |        |          |0 = Receive data available interrupt Disabled.
    * |        |          |1 = Receive data available interrupt Enabled.
    * |[1]     |THRE_IEN  |Transmit Holding Register Empty Interrupt Enable Bit
    * |        |          |0 = Transmit holding register empty interrupt Disabled.
    * |        |          |1 = Transmit holding register empty interrupt Enabled.
    * |[2]     |RLS_IEN   |Receive Line Status Interrupt Enable Bit
    * |        |          |0 = Receive Line Status interrupt Disabled.
    * |        |          |1 = Receive Line Status interrupt Enabled.
    * |[3]     |MODEM_IEN |Modem Status Interrupt Enable Bit
    * |        |          |0 = Modem status interrupt Disabled.
    * |        |          |1 = Modem status interrupt Enabled.
    * |[4]     |TOUT_IEN  |RX Time-out Interrupt Enable Bit
    * |        |          |0 = RX time-out interrupt Disabled.
    * |        |          |1 = RX time-out interrupt Enabled.
    * |[5]     |BUF_ERR_IEN|Buffer Error Interrupt Enable Bit
    * |        |          |0 = Buffer error interrupt Disabled.
    * |        |          |1 = Buffer error interrupt Enabled.
    * |[6]     |WAKE_EN   |UART Wake-up Function Enable Bit
    * |        |          |0 = UART wake-up function Disabled.
    * |        |          |1 = UART wake-up function Enabled, when chip is in Power-down mode, an external nCTS change will wake up chip from Power-down mode.
    * |[8]     |LIN_IEN   |LIN Bus Interrupt Enable Bit
    * |        |          |0 = Lin bus interrupt Disabled.
    * |        |          |1 = Lin bus interrupt Enabled.
    * |        |          |Note: This field is used for LIN function mode.
    * |[11]    |TIME_OUT_EN|Receive Buffer Time-out Counter Enable
    * |        |          |0 = Receive Buffer Time-out counter Disabled.
    * |        |          |1 = Receive Buffer Time-out counter Enabled.
    * |[12]    |AUTO_RTS_EN|nRTS Auto Flow Control Enable
    * |        |          |0 = nRTS auto flow control Disabled.
    * |        |          |1 = nRTS auto flow control Enabled.
    * |        |          |When nRTS auto-flow is enabled, if the number of bytes in the RX FIFO equals the RTS_TRI_LEV (UA_FCR[19:16]), the UART will de-assert nRTS signal.
    * |[13]    |AUTO_CTS_EN|nCTS Auto Flow Control Enable
    * |        |          |0 = nCTS auto flow control Disabled.
    * |        |          |1 = nCTS auto flow control Enabled.
    * |        |          |When nCTS auto-flow is enabled, the UART will send data to external device when nCTS input assert (UART will not send data to device until nCTS is asserted).
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
    * |        |          |When the number of bytes in the receive FIFO equals the RFITL, the RDA_IF (UA_ISR[0]) will be set (if RDA_IEN (UA_IER[0] enabled, and an interrupt will be generated).
    * |        |          |0000 = RX FIFO Interrupt Trigger Level is 1 byte.
    * |        |          |0001 = RX FIFO Interrupt Trigger Level is 4 bytes.
    * |        |          |0010 = RX FIFO Interrupt Trigger Level is 8 bytes.
    * |        |          |0011 = RX FIFO Interrupt Trigger Level is 14 bytes.
    * |        |          |Others = Reserved.
    * |[8]     |RX_DIS    |Receiver Disable Bit
    * |        |          |The receiver is disabled or not (set 1 to disable receiver).
    * |        |          |0 = Receiver Enabled.
    * |        |          |1 = Receiver Disabled.
    * |        |          |Note: This field is used for RS-485 Normal Multi-drop mode. It should be programmed before RS485_NMM (UA_ALT_CSR[8]) is programmed.
    * |[19:16] |RTS_TRI_LEV|nRTS Trigger Level For Auto-flow Control Use
    * |        |          |0000 = RTS Trigger Level is 1 byte.
    * |        |          |0001 = RTS Trigger Level is 4 bytes.
    * |        |          |0010 = RTS Trigger Level is 8 bytes.
    * |        |          |0011 = RTS Trigger Level is 14 bytes.
    * |        |          |Others = Reserved.
    * |        |          |Note: This field is used for nRTS auto-flow control.
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
    * |[3]     |PBE       |Parity Bit Enable Bit
    * |        |          |0 = No parity bit.
    * |        |          |1 = Parity bit is generated on each outgoing character and is checked on each incoming data.
    * |[4]     |EPE       |Even Parity Enable Bit
    * |        |          |0 = Odd number of logic 1's is transmitted and checked in each word.
    * |        |          |1 = Even number of logic 1's is transmitted and checked in each word.
    * |        |          |This bit has effect only when PBE (UA_LCR[3]) is set.
    * |[5]     |SPE       |Stick Parity Enable Bit
    * |        |          |0 = Stick parity Disabled.
    * |        |          |1 = If PBE (UA_LCR[3]) and EBE (UA_LCR[4]) are logic 1, the parity bit is transmitted and checked as logic 0.
    * |        |          |If PBE (UA_LCR[3]) is 1 and EBE (UA_LCR[4]) is 0 then the parity bit is transmitted and checked as 1.
    * |[6]     |BCB       |Break Control Bit
    * |        |          |When this bit is set to logic 1, the serial data output (TX) is forced to the Spacing State (logic 0).
    * |        |          |This bit acts only on TX and has no effect on the transmitter logic.
    * @var UART_T::MCR
    * Offset: 0x10  UART Modem Control Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[1]     |RTS       |nRTS (Request-to-send) Signal Control
    * |        |          |This bit is direct control internal nRTS signal active or not, and then drive the nRTS pin output with LEV_RTS bit configuration.
    * |        |          |0 = nRTS signal is active.
    * |        |          |1 = nRTS signal is inactive.
    * |        |          |Note1: This nRTS signal control bit is not effective when RTS auto-flow control is enabled in UART function mode.
    * |        |          |Note2: This nRTS signal control bit is not effective when RS-485 auto direction mode (AUD) is enabled in RS-485 function mode.
    * |[9]     |LEV_RTS   |nRTS Pin Active Level
    * |        |          |This bit defines the active level state of nRTS pin output.
    * |        |          |0 = nRTS pin output is high level active.
    * |        |          |1 = nRTS pin output is low level active.
    * |[13]    |RTS_ST    |nRTS Pin State (Read Only)
    * |        |          |This bit mirror from nRTS pin output of voltage logic status.
    * |        |          |0 = nRTS pin output is low level voltage logic state.
    * |        |          |1 = nRTS pin output is high level voltage logic state.
    * @var UART_T::MSR
    * Offset: 0x14  UART Modem Status Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |DCTSF     |Detect nCTS State Change Flag
    * |        |          |This bit is set whenever nCTS input has change state, and it will generate Modem interrupt to CPU when MODEM_IEN (UA_IER[3]) is set to 1.
    * |        |          |0 = nCTS input has not change state.
    * |        |          |1 = nCTS input has change state.
    * |        |          |Note: This bit can be cleared by writing "1" to it.
    * |[4]     |CTS_ST    |nCTS Pin Status (Read Only)
    * |        |          |This bit mirror from nCTS pin input of voltage logic status.
    * |        |          |0 = nCTS pin input is low level voltage logic state.
    * |        |          |1 = nCTS pin input is high level voltage logic state.
    * |        |          |Note: This bit echoes when UART Controller peripheral clock is enabled, and nCTS multi-function port is selected.
    * |[8]     |LEV_CTS   |nCTS Pin Active Level
    * |        |          |This bit defines the active level state of nCTS pin input.
    * |        |          |0 = nCTS pin input is high level active.
    * |        |          |1 = nCTS pin input is low level active.
    * @var UART_T::FSR
    * Offset: 0x18  UART FIFO Status Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |RX_OVER_IF|RX Overflow Error Interrupt Flag
    * |        |          |This bit is set when RX FIFO overflow.
    * |        |          |If the number of bytes of received data is greater than RX_FIFO (UA_RBR) size, this bit will be set.
    * |        |          |0 = RX FIFO is not overflow.
    * |        |          |1 = RX FIFO is overflow.
    * |        |          |Note: This bit can be cleared by writing "1" to it.
    * |[3]     |RS485_ADD_DETF|RS-485 Address Byte Detection Flag
    * |        |          |0 = Receiver detects a data that is not an address byte (bit 9 = "0").
    * |        |          |1 = Receiver detects a data that is an address byte (bit 9 = "1").
    * |        |          |Note1: This field is used for RS-485 function mode and RS485_ADD_EN (UA_ALT_CSR[15]) is set to 1 to enable Address detection mode.
    * |        |          |Note2: This bit can be cleared by writing "1" to it.
    * |[4]     |PEF       |Parity Error Flag
    * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "parity bit", and is reset whenever the CPU writes 1 to this bit.
    * |        |          |0 = No parity error is generated.
    * |        |          |1 = Parity error is generated.
    * |        |          |Note: This bit can be cleared by writing "1" to it.
    * |[5]     |FEF       |Framing Error Flag
    * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "stop bit" (that is, the stop bit following the last data bit or parity bit is detected as logic 0), and is reset whenever the CPU writes 1 to this bit.
    * |        |          |0 = No framing error is generated.
    * |        |          |1 = Framing error is generated.
    * |        |          |Note: This bit can be cleared by writing "1" to it.
    * |[6]     |BIF       |Break Interrupt Flag
    * |        |          |This bit is set to logic 1 whenever the received data input(RX) is held in the "spacing state" (logic 0) for longer than a full word transmission time (that is, the total time of "start bit" + data bits + parity + stop bits) and is reset whenever the CPU writes 1 to this bit.
    * |        |          |0 = No Break interrupt is generated.
    * |        |          |1 = Break interrupt is generated.
    * |        |          |Note: This bit can be cleared by writing "1" to it.
    * |[13:8]  |RX_POINTER|RX FIFO Pointer (Read Only)
    * |        |          |This field indicates the RX FIFO Buffer Pointer. When UART receives one byte from external device, RX_POINTER increases one. When one byte of RX FIFO is read by CPU, RX_POINTER decreases one.
    * |        |          |The Maximum value shown in RX_POINTER is 15. When the using level of RX FIFO Buffer equal to 16, the RX_FULL bit is set to 1 and RXPTR will show 0. As one byte of RX FIFO is read by CPU, the RX_FULL bit is cleared to 0 and RX_POINTER will show 15.
    * |[14]    |RX_EMPTY  |Receiver FIFO Empty (Read Only)
    * |        |          |This bit initiate RX FIFO empty or not.
    * |        |          |0 = RX FIFO is not empty.
    * |        |          |1 = RX FIFO is empty.
    * |        |          |Note: When the last byte of RX FIFO has been read by CPU, hardware sets this bit high. It will be cleared when UART receives any new data.
    * |[15]    |RX_FULL   |Receiver FIFO Full (Read Only)
    * |        |          |This bit indicates RX FIFO full or not.
    * |        |          |0 = RX FIFO is not full.
    * |        |          |1 = RX FIFO is full.
    * |        |          |Note: This bit is set when the using level of RX FIFO Buffer equal to 16; otherwise, it is cleared by hardware.
    * |[21:16] |TX_POINTER|TX FIFO Pointer (Read Only)
    * |        |          |This field indicates the TX FIFO Buffer Pointer. When CPU writes one byte into UA_THR, TX_POINTER increases one. When one byte of TX FIFO is transferred to Transmitter Shift Register, TX_POINTER decreases one.
    * |        |          |The Maximum value shown in TX_POINTER is 15. When the using level of TX FIFO Buffer equal to 16, the TX_FULL bit is set to 1 and TX_POINTER will show 0. As one byte of TX FIFO is transferred to Transmitter Shift Register, the TX_FULL bit is cleared to 0 and TX_POINTER will show 15.
    * |[22]    |TX_EMPTY  |Transmitter FIFO Empty (Read Only)
    * |        |          |This bit indicates TX FIFO is empty or not.
    * |        |          |0 = TX FIFO is not empty.
    * |        |          |1 = TX FIFO is empty.
    * |        |          |Note: When the last byte of TX FIFO has been transferred to Transmitter Shift Register, hardware sets this bit high. It will be cleared when writing data into UA_THR (TX FIFO not empty).
    * |[23]    |TX_FULL   |Transmitter FIFO Full (Read Only)
    * |        |          |This bit indicates TX FIFO full or not.
    * |        |          |0 = TX FIFO is not full.
    * |        |          |1 = TX FIFO is full.
    * |        |          |Note: This bit is set when the using level of TX FIFO Buffer equal to 16; otherwise, it is cleared by hardware.
    * |[24]    |TX_OVER_IF|Tx Overflow Error Interrupt Flag
    * |        |          |If TX FIFO (UA_THR) is full, an additional write to UA_THR will cause this bit to logic 1.
    * |        |          |0 = TX FIFO is not overflow.
    * |        |          |1 = TX FIFO is overflow.
    * |        |          |Note: This bit can be cleared by writing "1" to it.
    * |[28]    |TE_FLAG   |Transmitter Empty Flag (Read Only)
    * |        |          |This bit is set by hardware when TX FIFO (UA_THR) is empty and the STOP bit of the last byte has been transmitted.
    * |        |          |0 = TX FIFO is not empty or the STOP bit of the last byte has been not transmitted.
    * |        |          |1 = TX FIFO is empty and the STOP bit of the last byte has been transmitted.
    * |        |          |Note: This bit is cleared automatically when TX FIFO is not empty or the last byte transmission has not completed.
    * @var UART_T::ISR
    * Offset: 0x1C  UART Interrupt Status Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |RDA_IF    |Receive Data Available Interrupt Flag (Read Only)
    * |        |          |When the number of bytes in the RX FIFO equals the RFITL (UA_FCR[7:4]) then the RDA_IF (UA_ISR[0]) will be set.
    * |        |          |If RDA_IEN (UA_IER[0]) is enabled, the RDA interrupt will be generated.
    * |        |          |0 = No RDA interrupt flag is generated.
    * |        |          |1 = RDA interrupt flag is generated.
    * |        |          |Note: This bit is read only and it will be cleared when the number of unread bytes of RX FIFO drops below the threshold level RFITL (UA_FCR[7:4]).
    * |[1]     |THRE_IF   |Transmit Holding Register Empty Interrupt Flag (Read Only)
    * |        |          |This bit is set when the last data of TX FIFO is transferred to Transmitter Shift Register.
    * |        |          |If THRE_IEN (UA_IER[1]) is enabled, the THRE interrupt will be generated.
    * |        |          |0 = No THRE interrupt flag is generated.
    * |        |          |1 = THRE interrupt flag is generated.
    * |        |          |Note: This bit is read only and it will be cleared when writing data into THR (TX FIFO not empty).
    * |[2]     |RLS_IF    |Receive Line Interrupt Flag (Read Only)
    * |        |          |This bit is set when the RX receive data have parity error, frame error or break error (at least one of 3 bits, BIF(UA_FSR[6]), FEF(UA_FSR[5]) and PEF(UA_FSR[4]), is set).
    * |        |          |If RLS_IEN (UA_IER [2]) is enabled, the RLS interrupt will be generated.
    * |        |          |0 = No RLS interrupt flag is generated.
    * |        |          |1 = RLS interrupt flag is generated.
    * |        |          |Note1: In RS-485 function mode, this field is set include receiver detect and received address byte character (bit9 = '1') bit. At the same time, the bit of RS485_ADD_DETF (UA_FSR[3]) is also set.
    * |        |          |Note2: This bit is read only and reset to 0 when all bits of BIF (UA_FSR[6]), FEF (UA_FSR[5]) and PEF (UA_FSR[4]) are cleared.
    * |        |          |Note3: In RS-485 function mode, this bit is read only and reset to 0 when all bits of BIF (UA_FSR[6]) , FEF (UA_FSR[5]) and PEF (UA_FSR[4]) and RS485_ADD_DETF (UA_FSR[3]) are cleared.
    * |[3]     |MODEM_IF  |MODEM Interrupt Flag (Read Only)
    * |        |          |This bit is set when the nCTS pin has state change (DCTSF (UA_MSR[0]) = 1).
    * |        |          |If MODEM_IEN (UA_IER[3]) is enabled, the Modem interrupt will be generated.
    * |        |          |0 = No Modem interrupt flag is generated.
    * |        |          |1 = Modem interrupt flag is generated.
    * |        |          |Note: This bit is reset to 0 when bit DCTSF (UA_MSR[0]) is cleared by a write 1 on DCTSF (UA_MSR[0]).
    * |[4]     |TOUT_IF   |Rx Time-out Interrupt Flag (Read Only)
    * |        |          |This bit is set when the RX FIFO is not empty and no activities occurred in the RX FIFO and the time-out counter equal to TOIC (UA_TOR[7:0]).
    * |        |          |If TOUT_IEN (UA_IER[4]) is enabled, the Rx Time-out interrupt will be generated.
    * |        |          |0 = No Rx Time-out interrupt flag is generated.
    * |        |          |1 = Rx Time-out interrupt flag is generated.
    * |        |          |Note: This bit is read only and user can read UA_RBR (RX is in active) to clear it
    * |[5]     |BUF_ERR_IF|Buffer Error Interrupt Flag (Read Only)
    * |        |          |This bit is set when the TX FIFO or RX FIFO overflows (TX_OVER_IF (UA_FSR[24]) or RX_OVER_IF (UA_FSR[0]) is set).
    * |        |          |When BUF_ERR_IF (UA_ISR[5]) is set, the transfer is not correct.
    * |        |          |If BUF_ERR_IEN (UA_IER[8]) is enabled, the buffer error interrupt will be generated.
    * |        |          |0 = No buffer error interrupt flag is generated.
    * |        |          |1 = Buffer error interrupt flag is generated.
    * |        |          |Note: This bit is read only and reset to 0 when all bits of TX_OVER_IF (UA_FSR[24]) and RX_OVER_IF (UA_FSR[0]) are cleared.
    * |[7]     |LIN_IF    |LIN Bus Interrupt Flag (Read Only)
    * |        |          |This bit is set when LIN slave header detect (LINS_HDET_F (UA_LIN_SR[0]) = 1), LIN break detect (LIN_BKDET_F (UA_LIN_SR[8]) = 1), bit error detect (BIT_ERR_F (UA_LIN_SR[9]) = 1), LIN slave ID parity error (LINS_IDPERR_F (UA_LIN_SR[2]) = 1) or LIN slave header error detect (LINS_HERR_F (UA_LIN_SR[1]) = 1). 
    * |        |          |If LIN_IEN (UA_IER[8]) is enabled the LIN interrupt will be generated.    
    * |        |          |0 = None of LINS_HDET_F, LIN_BKDET_F, BIT_ERR_F, LINS_IDPERR_F and LINS_HERR_F is generated.
    * |        |          |1 = At least one of LINS_HDET_F, LIN_BKDET_F, BIT_ERR_F, LINS_IDPERR_F and LINS_HERR_F is generated.
    * |        |          |Note: This bit is cleared when LINS_HDET_F, LIN_BKDET_F, BIT_ERR_F, LINS_IDPENR_F and LINS_HERR_F all are cleared.
    * |[8]     |RDA_INT   |Receive Data Available Interrupt Indicator (Read Only)
    * |        |          |This bit is set if RDA_IEN (UA_IER[0]) and RDA_IF (UA_ISR[0]) are both set to 1.
    * |        |          |0 = No RDA interrupt is generated.
    * |        |          |1 = RDA interrupt is generated.
    * |[9]     |THRE_INT  |Transmit Holding Register Empty Interrupt Indicator (Read Only)
    * |        |          |This bit is set if THRE_IEN (UA_IER[1]) and THRE_IF (UA_ISR[1]) are both set to 1.
    * |        |          |0 = No THRE interrupt is generated.
    * |        |          |1 = THRE interrupt is generated.
    * |[10]    |RLS_INT   |Receive Line Status Interrupt Indicator (Read Only)
    * |        |          |This bit is set if RLS_IEN (UA_IER[2]) and RLS_IF (UA_ISR[2]) are both set to 1.
    * |        |          |0 = No RLS interrupt is generated.
    * |        |          |1 = RLS interrupt is generated
    * |[11]    |MODEM_INT |MODEM Status Interrupt Indicator (Read Only)
    * |        |          |This bit is set if MODEM_IEN (UA_IER[3]) and MODEM_IF (UA_ISR[3]) are both set to 1.
    * |        |          |0 = No Modem interrupt is generated.
    * |        |          |1 = Modem interrupt is generated.
    * |[12]    |TOUT_INT  |Rx Time-out Interrupt Indicator (Read Only)
    * |        |          |This bit is set if TOUT_IEN (UA_IER[4]) and TOUT_IF (UA_ISR[4]) are both set to 1.
    * |        |          |0 = No Rx Time-out interrupt is generated.
    * |        |          |1 = Rx Time-out interrupt is generated.
    * |[13]    |BUF_ERR_INT|Buffer Error Interrupt Indicator (Read Only)
    * |        |          |This bit is set if BUF_ERR_IEN (UA_IER[5]) and BUF_ERR_IF (UA_ISR[5]) are both set to 1.
    * |        |          |0 = No buffer error interrupt is generated.
    * |        |          |1 = Buffer error interrupt is generated.
    * |[15]    |LIN_INT   |LIN Bus Interrupt Indicator (Read Only)
    * |        |          |This bit is set if LIN_IEN (UA_IER[8]) and LIN_IF (UA_ISR[7]) are both set to 1.
    * |        |          |0 = No LIN Bus interrupt is generated.
    * |        |          |1 = The LIN Bus interrupt is generated.
    * @var UART_T::TOR
    * Offset: 0x20  UART Time-out Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[7:0]   |TOIC      |Time-out Interrupt Comparator
    * |        |          |The time-out counter resets and starts counting (the counting clock = baud rate) whenever the RX FIFO receives a new data word  if time-out counter is enabled by setting TIME_OUT_EN (UA_IER[11]).
    * |        |          |Once the content of time-out counter is equal to that of time-out interrupt comparator (TOIC (UA_TOR[7:0])), a receiver time-out interrupt is generated if TOUT_IEN (UA_IER[4]) enabled.
    * |        |          |A new incoming data word or RX FIFO empty will clear TOUT_IF (UA_IER[4]).
    * |        |          |In order to avoid receiver time-out interrupt generation immediately during one character is being received, TOIC (UA_TOR[7:0]) value should be set between 40 and 255.
    * |        |          |So, for example, if TOIC (UA_TOR[7:0]) is set with 40, the time-out interrupt is generated after four characters are not received when 1 stop bit and no parity check is set for UART transfer.
    * |[15:8]  |DLY       |TX Delay Time Value
    * |        |          |This field is used to programming the transfer delay time between the last stop bit and next start bit.
    * @var UART_T::BAUD
    * Offset: 0x24  UART Baud Rate Divisor Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[15:0]  |BRD       |Baud Rate Divider
    * |        |          |The field indicates the baud rate divider.
    * |[27:24] |DIVIDER_X |Divider X
    * |        |          |The baud rate divider M = X+1.
    * |[28]    |DIV_X_ONE |Divider X Equal to 1
    * |        |          |0 = Divider M is X+1.
    * |        |          |1 = Divider M is 1.
    * |[29]    |DIV_X_EN  |Divider X Enable Bit
    * |        |          |The BRD = Baud Rate Divider, and the baud rate equation is Baud Rate = Clock / [M * (BRD + 2)]; The default value of M is 16.
    * |        |          |0 = Divider X Disabled (the equation of M = 16).
    * |        |          |1 = Divider X Enabled (the equation of M = X+1, but DIVIDER_X (UA_BAUD[27:24]) must >= 8).
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
    * |        |          |Note1: This break field length is LIN_BKFL + 1.
    * |        |          |Note2: According to LIN spec, the reset value is 0xC (break field length = 13).
    * |[6]     |LIN_RX_EN |LIN RX Enable Bit
    * |        |          |0 = LIN RX mode Disabled.
    * |        |          |1 = LIN RX mode Enabled.
    * |[7]     |LIN_TX_EN |LIN TX Break Mode Enable Bit
    * |        |          |0 = LIN TX Break mode Disabled.
    * |        |          |1 = LIN TX Break mode Enabled.
    * |        |          |Note: When TX break field transfer operation finished, this bit will be cleared automatically.
    * |[8]     |RS485_NMM |RS-485 Normal Multi-drop Operation Mode (NMM)
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
    * |[15]    |RS485_ADD_EN|RS-485 Address Detection Enable Bit
    * |        |          |This bit is used to enable RS-485 Address Detection mode.
    * |        |          |0 = Address detection mode Disabled.
    * |        |          |1 = Address detection mode Enabled.
    * |        |          |Note: This bit is used for RS-485 any operation mode.
    * |[31:24] |ADDR_MATCH|Address Match Value
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
    * |        |          |Note2: In LIN function mode, when detect header field (break + sync + frame ID), LINS_HDET_F (UA_LIN_SR[0]) flag will be asserted.
    * |        |          |If the LIN_IEN (UA_IER[8]) = 1, an interrupt will be generated.
    * |[2]     |LINS_ARS_EN|LIN Slave Automatic Resynchronization Mode Enable Control
    * |        |          |0 = LIN automatic resynchronization Disabled.
    * |        |          |1 = LIN automatic resynchronization Enabled.    
    * |        |          |Note1: This bit only valid when in LIN slave mode (LINS_EN (UA_LIN_CTL[0]) = 1).
    * |        |          |Note2: When operation in Automatic Resynchronization mode, the baud rate setting must be mode2 (DIV_X_EN (UA_BAUD[29]) and DIV_X_ONE (UA_BAUD[28]) must be 1).
    * |        |          |Note3: The control and interactions of this field are explained in Slave mode with automatic resynchronization section.
    * |[3]     |LINS_DUM_EN|LIN Slave Divider Update Method Enable Control
    * |        |          |0 = UA_BAUD updated is written by software (if no automatic resynchronization update occurs at the same time).
    * |        |          |1 = UA_BAUD is updated at the next received character. User must set the bit before checksum reception.   
    * |        |          |Note1: This bit only valid when in LIN slave mode (LINS_EN (UA_LIN_CTL[0]) = 1).
    * |        |          |Note2: This bit is used for LIN Slave Automatic Resynchronization mode. (for Non-Automatic Resynchronization mode, this bit should be kept cleared) 
    * |        |          |Note3: The control and interactions of this field are explained in Slave mode with automatic resynchronization section.
    * |[4]     |LIN_MUTE_EN|LIN Mute Mode Enable Control
    * |        |          |0 = LIN mute mode Disabled.
    * |        |          |1 = LIN mute mode Enabled.
    * |        |          |Note: The exit from mute mode condition and each control and interactions of this field are explained in (LIN slave mode).
    * |[8]     |LIN_SHD   |LIN TX Send Header Enable Control
    * |        |          |The LIN TX header can be "break field" or "break and sync field" or "break, sync and frame ID field", it is depend on setting LIN_HEAD_SEL (UA_LIN_CTL[23:22]).
    * |        |          |0 = Send LIN TX header Disabled.
    * |        |          |1 = Send LIN TX header Enabled.
    * |        |          |Note1: Tis bit is shadow bit of LIN_TX_EN (UA_ALT_CSR[7]); user can read/write it by setting LIN_TX_EN (UA_ALT_CSR[7]) or LIN_SHD (UA_LIN_CTL[8]).    
    * |        |          |Note2: When transmitter header field (it may be "break" or "break + sync" or "break + sync + frame ID" selected by LIN_HEAD_SEL (UA_LIN_CTL[23:22]) field) transfer operation finished, this bit will be cleared automatically.
    * |[9]     |LIN_IDPEN |LIN ID Parity Enable Control
    * |        |          |0 = LIN frame ID parity Disabled.
    * |        |          |1 = LIN frame ID parity Enabled.
    * |        |          |Note1: This bit can be used for LIN master to sending header field LIN_SHD (UA_LIN_CTL[8]) = 1 and LIN_HEAD_SEL (UA_LIN_CTL[23:22]) = 10) or be used for enable LIN slave received frame ID parity checked.
    * |        |          |Note2: This bit is only use when operation header transmitter is in LIN_HEAD_SEL (UA_LIN_CTL[23:22]) = 10.
    * |[10]    |LIN_BKDET_EN|LIN Break Detection Enable Control
    * |        |          |When detect consecutive dominant greater than 11 bits, and are followed by a delimiter character, the LIN_BKDET_F (UA_LIN_SR[8]) flag is set in UA_LIN_SR register at the end of break field.
    * |        |          |If the LIN_IEN (UA_IER[8]) = 1, an interrupt will be generated.
    * |        |          |0 = LIN break detection Disabled.
    * |        |          |1 = LIN break detection Enabled.
    * |[11]    |LIN_RX_DIS|LIN Receiver Disable Control
    * |        |          |If the receiver is enabled (LIN_RX_DIS (UA_LIN_CTL[11]) = 0), all received byte data will be accepted and stored in the RX-FIFO, and if the receiver is disabled (LIN_RX_DIS (UA_LIN_CTL[11]) = 1), all received byte data will be ignore.
    * |        |          |0 = LIN receiver Enabled.
    * |        |          |1 = LIN receiver Disabled.
    * |        |          |Note: This bit is only valid when operating in LIN function mode (FUN_SEL (UA_FUN_SEL[1:0]) = 01).
    * |[12]    |BIT_ERR_EN|Bit Error Detect Enable Control
    * |        |          |0 = Bit error detection function Disabled.
    * |        |          |1 = Bit error detection Enabled.
    * |        |          |Note: In LIN function mode, when occur bit error, the BIT_ERR_F (UA_LIN_SR[9]) flag will be asserted.
    * |        |          |If the LIN_IEN (UA_IER[8]) = 1, an interrupt will be generated.
    * |[19:16] |LIN_BKFL  |LIN Break Field Length
    * |        |          |This field indicates a 4-bit LIN TX break field count.
    * |        |          |Note1: These registers are shadow registers of LIN_BKFL (UA_ALT_CSR[19:16]), User can read/write it by setting LIN_BKFL (UA_ALT_CSR[3:0]) or LIN_BKFL (UA_LIN_CTL[19:16]).
    * |        |          |Note2: This break field length is LIN_BKFL + 1.
    * |        |          |Note3: According to LIN spec, the reset value is 0xC (break field length = 13).
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
    * |        |          |Note: This bit is used to master mode for LIN to send header field (LIN_SHD (UA_LIN_CTL[8]) = 1) or used to slave to indicates exit from mute mode condition (LIN_MUTE_EN (UA_LIN_CTL[4]) = 1).
    * |[31:24] |LIN_PID   |LIN PID Register
    * |        |          |This field contains the LIN frame ID value when in LIN function mode, the frame ID parity can be generated by software or hardware depends on LIN_IDPEN (UA_LIN_CTL[9]) = 1.
    * |        |          |If the parity generated by hardware, user fill ID0~ID5, (LIN_PID [29:24] )hardware will calculate P0 (LIN_PID[30]) and P1 (LIN_PID[31]), otherwise user must filled frame ID and parity in this field.
    * |        |          |Note1: User can fill any 8-bit value to this field and the bit 24 indicates ID0 (LSB first).
    * |        |          |Note2: This field can be used for LIN master mode or slave mode.
    * @var UART_T::LIN_SR
    * Offset: 0x38  UART LIN Status Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |LINS_HDET_F|LIN Slave Header Detection Flag
    * |        |          |This bit is set by hardware when a LIN header is detected in LIN slave mode.
    * |        |          |0 = LIN header not detected.
    * |        |          |1 = LIN header detected (break + sync + frame ID).
    * |        |          |Note1: This bit can be cleared by writing "1" to it.
    * |        |          |Note2: This bit is only valid when in LIN slave mode (LINS_EN (UA_LIN_CTL [0]) = 1) and enable LIN slave header detection function (LINS_HDET_EN (UA_LIN_CTL [1])).
    * |        |          |Note3: When enable ID parity check LIN_IDPEN (UA_LIN_CTL[9]), if hardware detect complete header ("break + sync + frame ID"), the LINS_HEDT_F will be set whether the frame ID correct or not.
    * |[1]     |LINS_HERR_F|LIN Slave Header Error Flag
    * |        |          |This bit is set by hardware when a LIN header error is detected in LIN slave mode.
    * |        |          |The header errors include "break delimiter is too short (less than 0.5 bit time)", "frame error in sync field or Identifier field", "sync field data is not 0x55 in Non-Automatic Resynchronization mode", "sync field deviation error with Automatic Resynchronization mode", "sync field measure time-out with Automatic Resynchronization mode" and "LIN header reception time-out".
    * |        |          |0 = LIN header error not detected.
    * |        |          |1 = LIN header error detected.
    * |        |          |Note1: This bit can be cleared by writing "1" to it.
    * |        |          |Note2: This bit is only valid when UART is operated in LIN slave mode (LINS_EN (UA_LIN_CTL[0]) = 1) and enables LIN slave header detection function (LINS_HDET_EN (UA_LIN_CTL[1])).
    * |[2]     |LINS_IDPERR_F|LIN Slave ID Parity Error Flag
    * |        |          |This bit is set by hardware when receipted frame ID parity is not correct.
    * |        |          |0 = No active.
    * |        |          |1 = Receipted frame ID parity is not correct.
    * |        |          |Note1: This bit is can be cleared by writing "1" to it.
    * |        |          |Note2: This bit is only valid when in LIN slave mode (LINS_EN (UA_LIN_CTL [0])= 1) and enable LIN frame ID parity check function LIN_IDPEN (UA_LIN_CTL[9]).
    * |[3]     |LINS_SYNC_F|LIN Slave Sync Field
    * |        |          |This bit indicates that the LIN sync field is being analyzed in Automatic Resynchronization mode.
    * |        |          |When the receiver header have some error been detect, user must reset the internal circuit to re-search new frame header by writing 1 to this bit.
    * |        |          |0 = The current character is not at LIN sync state.
    * |        |          |1 = The current character is at LIN sync state.
    * |        |          |Note1: This bit is only valid when in LIN Slave mode (LINS_EN(UA_LIN_CTL[0]) = 1).
    * |        |          |Note2: This bit is read only, but it can be cleared by writing 1 to it.
    * |        |          |Note3: When writing 1 to it, hardware will reload the initial baud rate and re-search a new frame header.
    * |[8]     |LIN_BKDET_F|LIN Break Detection Flag
    * |        |          |This bit is set by hardware when a break is detected.
    * |        |          |0 = LIN break not detected.
    * |        |          |1 = LIN break detected.
    * |        |          |Note1: This bit can be cleared by writing "1" to it.
    * |        |          |Note2: This bit is only valid when LIN break detection function is enabled (LIN_BKDET_EN (UA_LIN_CTL[10]) = 1).
    * |[9]     |BIT_ERR_F |Bit Error Detect Status Flag
    * |        |          |At TX transfer state, hardware will monitoring the bus state, if the input pin (SIN) state not equals to the output pin (SOUT) state, BIT_ERR_F (UA_LIN_SR[9]) will be set.
    * |        |          |When occur bit error, if the LIN_IEN (UA_IER[8]) = 1, an interrupt will be generated.
    * |        |          |Note1: This bit can be cleared by writing "1" to it.
    * |        |          |Note2: This bit is only valid when enable bit error detection function (BIT_ERR_EN (UA_LIN_CTL [12]) = 1).
    */

    union
    {
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
#define UART_THR_THR_Pos            0                                       /*!< UART_T::THR: THR Position  */
#define UART_THR_THR_Msk            (0xFul << UART_THR_THR_Pos)             /*!< UART_T::THR: THR Mask      */

/* UART RBR Bit Field Definitions */
#define UART_RBR_RBR_Pos            0                                       /*!< UART_T::RBR: RBR Position */
#define UART_RBR_RBR_Msk            (0xFul << UART_RBR_RBR_Pos)             /*!< UART_T::RBR: RBR Mask     */

/* UART IER Bit Field Definitions */
#define UART_IER_AUTO_CTS_EN_Pos    13                                      /*!< UART_T::IER: AUTO_CTS_EN Position   */
#define UART_IER_AUTO_CTS_EN_Msk    (1ul << UART_IER_AUTO_CTS_EN_Pos)       /*!< UART_T::IER: AUTO_CTS_EN Mask       */

#define UART_IER_AUTO_RTS_EN_Pos    12                                      /*!< UART_T::IER: AUTO_RTS_EN Position   */
#define UART_IER_AUTO_RTS_EN_Msk    (1ul << UART_IER_AUTO_RTS_EN_Pos)       /*!< UART_T::IER: AUTO_RTS_EN Mask       */

#define UART_IER_TIME_OUT_EN_Pos    11                                      /*!< UART_T::IER: TIME_OUT_EN Position   */
#define UART_IER_TIME_OUT_EN_Msk    (1ul << UART_IER_TIME_OUT_EN_Pos)       /*!< UART_T::IER: TIME_OUT_EN Mask       */

#define UART_IER_LIN_IEN_Pos        8                                       /*!< UART_T::IER: LIN_IEN Position       */
#define UART_IER_LIN_IEN_Msk        (1ul << UART_IER_LIN_IEN_Pos)           /*!< UART_T::IER: LIN_IEN Mask           */

#define UART_IER_WAKE_EN_Pos        6                                       /*!< UART_T::IER: WAKE_EN Position       */
#define UART_IER_WAKE_EN_Msk        (1ul << UART_IER_WAKE_EN_Pos)           /*!< UART_T::IER: WAKE_EN Mask           */

#define UART_IER_BUF_ERR_IEN_Pos    5                                       /*!< UART_T::IER: BUF_ERR_IEN Position   */
#define UART_IER_BUF_ERR_IEN_Msk    (1ul << UART_IER_BUF_ERR_IEN_Pos)       /*!< UART_T::IER: BUF_ERR_IEN Mask       */

#define UART_IER_TOUT_IEN_Pos       4                                       /*!< UART_T::IER: TOUT_IEN Position      */
#define UART_IER_TOUT_IEN_Msk       (1ul << UART_IER_TOUT_IEN_Pos)          /*!< UART_T::IER: TOUT_IEN Mask          */

#define UART_IER_MODEM_IEN_Pos      3                                       /*!< UART_T::IER: MODEM_IEN Position     */
#define UART_IER_MODEM_IEN_Msk      (1ul << UART_IER_MODEM_IEN_Pos)         /*!< UART_T::IER: MODEM_IEN Mask         */

#define UART_IER_RLS_IEN_Pos        2                                       /*!< UART_T::IER: RLS_IEN Position       */
#define UART_IER_RLS_IEN_Msk        (1ul << UART_IER_RLS_IEN_Pos)           /*!< UART_T::IER: RLS_IEN Mask           */

#define UART_IER_THRE_IEN_Pos       1                                       /*!< UART_T::IER: THRE_IEN Position      */
#define UART_IER_THRE_IEN_Msk       (1ul << UART_IER_THRE_IEN_Pos)          /*!< UART_T::IER: THRE_IEN Mask          */

#define UART_IER_RDA_IEN_Pos        0                                       /*!< UART_T::IER: RDA_IEN Position       */
#define UART_IER_RDA_IEN_Msk        (1ul << UART_IER_RDA_IEN_Pos)           /*!< UART_T::IER: RDA_IEN Mask           */

/* UART FCR Bit Field Definitions */
#define UART_FCR_RTS_TRI_LEV_Pos    16                                      /*!< UART_T::FCR: RTS_TRI_LEV Position   */
#define UART_FCR_RTS_TRI_LEV_Msk    (0xFul << UART_FCR_RTS_TRI_LEV_Pos)     /*!< UART_T::FCR: RTS_TRI_LEV Mask       */

#define UART_FCR_RX_DIS_Pos         8                                       /*!< UART_T::FCR: RX_DIS Position        */
#define UART_FCR_RX_DIS_Msk         (1ul << UART_FCR_RX_DIS_Pos)            /*!< UART_T::FCR: RX_DIS Mask            */

#define UART_FCR_RFITL_Pos          4                                       /*!< UART_T::FCR: RFITL Position         */
#define UART_FCR_RFITL_Msk          (0xFul << UART_FCR_RFITL_Pos)           /*!< UART_T::FCR: RFITL Mask             */

#define UART_FCR_TFR_Pos            2                                       /*!< UART_T::FCR: TFR Position           */
#define UART_FCR_TFR_Msk            (1ul << UART_FCR_TFR_Pos)               /*!< UART_T::FCR: TFR Mask               */

#define UART_FCR_RFR_Pos            1                                       /*!< UART_T::FCR: RFR Position           */
#define UART_FCR_RFR_Msk            (1ul << UART_FCR_RFR_Pos)               /*!< UART_T::FCR: RFR Mask               */

/* UART LCR Bit Field Definitions */
#define UART_LCR_BCB_Pos            6                                       /*!< UART_T::LCR: BCB Position           */
#define UART_LCR_BCB_Msk            (1ul << UART_LCR_BCB_Pos)               /*!< UART_T::LCR: BCB Mask               */

#define UART_LCR_SPE_Pos            5                                       /*!< UART_T::LCR: SPE Position           */
#define UART_LCR_SPE_Msk            (1ul << UART_LCR_SPE_Pos)               /*!< UART_T::LCR: SPE Mask               */

#define UART_LCR_EPE_Pos            4                                       /*!< UART_T::LCR: EPE Position           */
#define UART_LCR_EPE_Msk            (1ul << UART_LCR_EPE_Pos)               /*!< UART_T::LCR: EPE Mask               */

#define UART_LCR_PBE_Pos            3                                       /*!< UART_T::LCR: PBE Position           */
#define UART_LCR_PBE_Msk            (1ul << UART_LCR_PBE_Pos)               /*!< UART_T::LCR: PBE Mask               */

#define UART_LCR_NSB_Pos            2                                       /*!< UART_T::LCR: NSB Position           */
#define UART_LCR_NSB_Msk            (1ul << UART_LCR_NSB_Pos)               /*!< UART_T::LCR: NSB Mask               */

#define UART_LCR_WLS_Pos            0                                       /*!< UART_T::LCR: WLS Position           */
#define UART_LCR_WLS_Msk            (0x3ul << UART_LCR_WLS_Pos)             /*!< UART_T::LCR: WLS Mask               */

/* UART MCR Bit Field Definitions */
#define UART_MCR_RTS_ST_Pos         13                                      /*!< UART_T::MCR: RTS_ST Position        */
#define UART_MCR_RTS_ST_Msk         (1ul << UART_MCR_RTS_ST_Pos)            /*!< UART_T::MCR: RTS_ST Mask            */

#define UART_MCR_LEV_RTS_Pos        9                                       /*!< UART_T::MCR: LEV_RTS Position       */
#define UART_MCR_LEV_RTS_Msk        (1ul << UART_MCR_LEV_RTS_Pos)           /*!< UART_T::MCR: LEV_RTS Mask           */

#define UART_MCR_RTS_Pos            1                                       /*!< UART_T::MCR: RTS Position           */
#define UART_MCR_RTS_Msk            (1ul << UART_MCR_RTS_Pos)               /*!< UART_T::MCR: RTS Mask               */

/* UART MSR Bit Field Definitions */
#define UART_MSR_LEV_CTS_Pos        8                                       /*!< UART_T::MSR: LEV_CTS Position       */
#define UART_MSR_LEV_CTS_Msk        (1ul << UART_MSR_LEV_CTS_Pos)           /*!< UART_T::MSR: LEV_CTS Mask           */

#define UART_MSR_CTS_ST_Pos         4                                       /*!< UART_T::MSR: CTS_ST Position        */
#define UART_MSR_CTS_ST_Msk         (1ul << UART_MSR_CTS_ST_Pos)            /*!< UART_T::MSR: CTS_ST Mask            */

#define UART_MSR_DCTSF_Pos          0                                       /*!< UART_T::MSR: DCTST Position         */
#define UART_MSR_DCTSF_Msk          (1ul << UART_MSR_DCTSF_Pos)             /*!< UART_T::MSR: DCTST Mask             */

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
#define UART_FUN_SEL_FUN_SEL_Pos        0                                        /*!< UART_T::FUN_SEL: FUN_SEL Position          */
#define UART_FUN_SEL_FUN_SEL_Msk       (0x3ul << UART_FUN_SEL_FUN_SEL_Pos)       /*!< UART_T::FUN_SEL: FUN_SEL Mask              */

/* UART LIN_CTL Bit Field Definitions */
#define UART_LIN_CTL_LIN_PID_Pos        24                                       /*!< UART_T::LIN_CTL: LIN_PID Position          */
#define UART_LIN_CTL_LIN_PID_Msk        (0xFFul << UART_LIN_CTL_LIN_PID_Pos)     /*!< UART_T::LIN_CTL: LIN_PID Mask              */

#define UART_LIN_CTL_LIN_HEAD_SEL_Pos   22                                       /*!< UART_T::LIN_CTL: LIN_HEAD_SEL Position     */
#define UART_LIN_CTL_LIN_HEAD_SEL_Msk   (0x3ul << UART_LIN_CTL_LIN_HEAD_SEL_Pos) /*!< UART_T::LIN_CTL: LIN_HEAD_SEL Mask         */

#define UART_LIN_CTL_LIN_BS_LEN_Pos     20                                       /*!< UART_T::LIN_CTL: LIN_BS_LEN Position       */
#define UART_LIN_CTL_LIN_BS_LEN_Msk     (0x3ul << UART_LIN_CTL_LIN_BS_LEN_Pos)   /*!< UART_T::LIN_CTL: LIN_BS_LEN Mask           */

#define UART_LIN_CTL_LIN_BKFL_Pos       16                                       /*!< UART_T::LIN_CTL: LIN_BKFL Position         */
#define UART_LIN_CTL_LIN_BKFL_Msk       (0xFul << UART_LIN_CTL_LIN_BKFL_Pos)     /*!< UART_T::LIN_CTL: LIN_BKFL Mask             */

#define UART_LIN_CTL_BIT_ERR_EN_Pos     12                                       /*!< UART_T::LIN_CTL: BIT_ERR_EN Position       */
#define UART_LIN_CTL_BIT_ERR_EN_Msk     (1ul << UART_LIN_CTL_BIT_ERR_EN_Pos)     /*!< UART_T::LIN_CTL: BIT_ERR_EN Mask           */

#define UART_LIN_CTL_LIN_RX_DIS_Pos     11                                       /*!< UART_T::LIN_CTL: LIN_RX_DIS Position       */
#define UART_LIN_CTL_LIN_RX_DIS_Msk     (1ul << UART_LIN_CTL_LIN_RX_DIS_Pos)     /*!< UART_T::LIN_CTL: LIN_RX_DIS Mask           */

#define UART_LIN_CTL_LIN_BKDET_EN_Pos   10                                       /*!< UART_T::LIN_CTL: LIN_BKDET_EN Position     */
#define UART_LIN_CTL_LIN_BKDET_EN_Msk   (1ul << UART_LIN_CTL_LIN_BKDET_EN_Pos)   /*!< UART_T::LIN_CTL: LIN_BKDET_EN Mask         */

#define UART_LIN_CTL_LIN_IDPEN_Pos      9                                        /*!< UART_T::LIN_CTL: LIN_IDPEN Position        */
#define UART_LIN_CTL_LIN_IDPEN_Msk      (1ul << UART_LIN_CTL_LIN_IDPEN_Pos)      /*!< UART_T::LIN_CTL: LIN_IDPEN Mask            */

#define UART_LIN_CTL_LIN_SHD_Pos        8                                        /*!< UART_T::LIN_CTL: LIN_SHD Position          */
#define UART_LIN_CTL_LIN_SHD_Msk        (1ul << UART_LIN_CTL_LIN_SHD_Pos)        /*!< UART_T::LIN_CTL: LIN_SHD Mask              */

#define UART_LIN_CTL_LIN_MUTE_EN_Pos    4                                        /*!< UART_T::LIN_CTL: LIN_MUTE_EN Position      */
#define UART_LIN_CTL_LIN_MUTE_EN_Msk    (1ul << UART_LIN_CTL_LIN_MUTE_EN_Pos)    /*!< UART_T::LIN_CTL: LIN_MUTE_EN Mask          */

#define UART_LIN_CTL_LINS_DUM_EN_Pos    3                                        /*!< UART_T::LIN_CTL: LINS_DUM_EN Position      */
#define UART_LIN_CTL_LINS_DUM_EN_Msk    (1ul << UART_LIN_CTL_LINS_DUM_EN_Pos)    /*!< UART_T::LIN_CTL: LINS_DUM_EN Mask          */

#define UART_LIN_CTL_LINS_ARS_EN_Pos    2                                        /*!< UART_T::LIN_CTL: LINS_ARS_EN Position      */
#define UART_LIN_CTL_LINS_ARS_EN_Msk    (1ul << UART_LIN_CTL_LINS_ARS_EN_Pos)    /*!< UART_T::LIN_CTL: LINS_ARS_EN Mask          */

#define UART_LIN_CTL_LINS_HDET_EN_Pos   1                                        /*!< UART_T::LIN_CTL: LINS_HDET_EN Position     */
#define UART_LIN_CTL_LINS_HDET_EN_Msk   (1ul << UART_LIN_CTL_LINS_HDET_EN_Pos)   /*!< UART_T::LIN_CTL: LINS_HDET_EN Mask         */

#define UART_LIN_CTL_LINS_EN_Pos        0                                        /*!< UART_T::LIN_CTL: LINS_EN Position          */
#define UART_LIN_CTL_LINS_EN_Msk        (1ul << UART_LIN_CTL_LINS_EN_Pos)        /*!< UART_T::LIN_CTL: LINS_EN Mask              */

/* UART LIN_SR Bit Field Definitions */
#define UART_LIN_SR_BIT_ERR_F_Pos       9                                        /*!< UART_T::LIN_SR: BIT_ERR_F Position         */
#define UART_LIN_SR_BIT_ERR_F_Msk       (1ul << UART_LIN_SR_BIT_ERR_F_Pos)       /*!< UART_T::LIN_SR: BIT_ERR_F Mask             */

#define UART_LIN_SR_LINS_BKDET_F_Pos    8                                        /*!< UART_T::LIN_SR: LINS_BKDET_F Position      */
#define UART_LIN_SR_LINS_BKDET_F_Msk    (1ul << UART_LIN_SR_LINS_BKDET_F_Pos)    /*!< UART_T::LIN_SR: LINS_BKDET_F Mask          */

#define UART_LIN_SR_LINS_SYNC_F_Pos     3                                        /*!< UART_T::LIN_SR: LINS_SYNC_F Position       */
#define UART_LIN_SR_LINS_SYNC_F_Msk     (1ul << UART_LIN_SR_LINS_SYNC_F_Pos)     /*!< UART_T::LIN_SR: LINS_SYNC_F Mask           */

#define UART_LIN_SR_LINS_IDPERR_F_Pos   2                                        /*!< UART_T::LIN_SR: LINS_IDPERR_F Position     */
#define UART_LIN_SR_LINS_IDPERR_F_Msk   (1ul << UART_LIN_SR_LINS_IDPERR_F_Pos)   /*!< UART_T::LIN_SR: LINS_IDPERR_F Mask         */

#define UART_LIN_SR_LINS_HERR_F_Pos     1                                        /*!< UART_T::LIN_SR: LINS_HERR_F Position       */
#define UART_LIN_SR_LINS_HERR_F_Msk     (1ul << UART_LIN_SR_LINS_HERR_F_Pos)     /*!< UART_T::LIN_SR: LINS_HERR_F Mask           */

#define UART_LIN_SR_LINS_HDET_F_Pos     0                                        /*!< UART_T::LIN_SR: LINS_HDET_F Position       */
#define UART_LIN_SR_LINS_HDET_F_Msk     (1ul << UART_LIN_SR_LINS_HDET_F_Pos)     /*!< UART_T::LIN_SR: LINS_HDET_F Mask           */
/*@}*/ /* end of group UART_CONST */
/*@}*/ /* end of group UART */



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
     * |        |          |Setting this bit will enable the WDT time-out reset function if the WDT up counter value has not been cleared after the specific WDT reset delay period expires.
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
     * |[4]     |WTWKE     |Watchdog Timer Time-out Wake-Up Function Control (Write Protect)
     * |        |          |If this bit is set to 1, while WTIF is generated to 1 and WTIE enabled, the WDT time-out interrupt signal will generate a wake-up trigger event to chip.
     * |        |          |0 = Wake-up trigger event Disabled if WDT time-out interrupt signal generated.
     * |        |          |1 = Wake-up trigger event Enabled if WDT time-out interrupt signal generated.
     * |        |          |Note: Chip can be woken-up by WDT time-out interrupt signal generated only if WDT clock source is selected to 10 kHz oscillator.
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
     * |        |          |Note: If CWDTEN (CONFIG0[31] Watchdog Enable) bit is set to 0, this bit is forced as 1 and user cannot change this bit to 0.
     * |[10:8]  |WTIS      |Watchdog Timer Time-out Interval Selection (Write Protect)
     * |        |          |These three bits select the time-out interval period for the WDT.
     * |        |          |000 = 2^4 *WDT_CLK.
     * |        |          |001 = 2^6 * WDT_CLK.
     * |        |          |010 = 2^8 * WDT_CLK.
     * |        |          |011 = 2^10 * WDT_CLK.
     * |        |          |100 = 2^12 * WDT_CLK.
     * |        |          |101 = 2^14 * WDT_CLK.
     * |        |          |110 = 2^16 * WDT_CLK.
     * |        |          |111 = 2^18 * WDT_CLK.
     * |[31]    |DBGACK_WDT|ICE Debug Mode Acknowledge Disable Control (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgment effects WDT counting.
     * |        |          |WDT up counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgment Disabled.
     * |        |          |WDT up counter will keep going no matter CPU is held by ICE or not.
     * @var WDT_T::WTCRALT
     * Offset: 0x04  Watchdog Timer Alternative Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WTRDSEL   |Watchdog Timer Reset Delay Selection (Write Protect)
     * |        |          |When WDT time-out happened, user has a time named WDT Reset Delay Period to clear WDT counter to prevent WDT time-out reset happened.
     * |        |          |User can select a suitable value of WDT Reset Delay Period for different WDT time-out period.
     * |        |          |These bits are protected bit.
     * |        |          |It means programming this bit needs to write "59h", "16h", "88h" to address 0x5000_0100 to disable register protection.
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


/*---------------------- Window Watchdog Timer -------------------------*/
/**
    @addtogroup WWDT Window Watchdog Timer (WWDT)
    Memory Mapped Structure for WWDT Controller
@{ */

typedef struct
{


    /**
     * @var WWDT_T::WWDTRLD
     * Offset: 0x00  Window Watchdog Timer Reload Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |WWDTRLD   |WWDT Reload Counter Register
     * |        |          |Writing 0x00005AA5 to this register will reload the WWDT counter value to 0x3F.
     * |        |          |Note: User can only write WWDTRLD to reload WWDT counter value when current WWDT
     * |        |          |counter value between 0 and WINCMP. If user writes WWDTRLD when current WWDT
     * |        |          |counter value is larger than WINCMP, WWDT reset signal will generate immediately.
     * @var WWDT_T::WWDTCR
     * Offset: 0x04  Window Watchdog Timer Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDTEN    |WWDT Enable Control
     * |        |          |0 = WWDT counter is stopped.
     * |        |          |1 = WWDT counter is starting counting.
     * |[1]     |WWDTIE    |WWDT Interrupt Enable Control
     * |        |          |If this bit is enabled, the WWDT counter compare match interrupt signal is generated and inform to CPU.
     * |        |          |0 = WWDT counter compare match interrupt Disabled.
     * |        |          |1 = WWDT counter compare match interrupt Enabled.
     * |[11:8]  |PERIODSEL |WWDT Counter Prescale Period Selection
     * |        |          |0000 = Pre-scale is 1; Max time-out period is 1 * 64 * TWWDT.
     * |        |          |0001 = Pre-scale is 2; Max time-out period is 2 * 64 * TWWDT.
     * |        |          |0010 = Pre-scale is 4; Max time-out period is 4 * 64 * TWWDT.
     * |        |          |0011 = Pre-scale is 8; Max time-out period is 8 * 64 * TWWDT.
     * |        |          |0100 = Pre-scale is 16; Max time-out period is 16 * 64 * TWWDT.
     * |        |          |0101 = Pre-scale is 32; Max time-out period is 32 * 64 * TWWDT.
     * |        |          |0110 = Pre-scale is 64; Max time-out period is 64 * 64 * TWWDT.
     * |        |          |0111 = Pre-scale is 128; Max time-out period is 128 * 64 * TWWDT.
     * |        |          |1000 = Pre-scale is 192; Max time-out period is 192 * 64 * TWWDT.
     * |        |          |1001 = Pre-scale is 256; Max time-out period is 256 * 64 * TWWDT.
     * |        |          |1010 = Pre-scale is 384; Max time-out period is 384 * 64 * TWWDT.
     * |        |          |1011 = Pre-scale is 512; Max time-out period is 512 * 64 * TWWDT.
     * |        |          |1100 = Pre-scale is 768; Max time-out period is 768 * 64 * TWWDT.
     * |        |          |1101 = Pre-scale is 1024; Max time-out period is 1024 * 64 * TWWDT.
     * |        |          |1110 = Pre-scale is 1536; Max time-out period is 1536 * 64 * TWWDT.
     * |        |          |1111 = Pre-scale is 2048; Max time-out period is 2048 * 64 * TWWDT.
     * |[21:16] |WINCMP    |WWDT Window Compare Register
     * |        |          |Set this register to adjust the valid reload window.
     * |        |          |Note: User can only write WWDTRLD to reload WWDT counter value when current WWDT counter value between 0 and WINCMP.
     * |        |          |If user writes WWDTRLD when current WWDT counter value larger than WINCMP, WWDT reset signal will generate immediately.
     * |[31]    |DBGACK_WWDT|ICE Debug Mode Acknowledge Disable Control
     * |        |          |0 = ICE debug mode acknowledgment effects WWDT counting.
     * |        |          |WWDT down counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgment Disabled.
     * |        |          |WWDT down counter will keep going no matter CPU is held by ICE or not.
     * @var WWDT_T::WWDTSR
     * Offset: 0x08  Window Watchdog Timer Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WWDTIF    |WWDT Compare Match Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of WWDT while WWDT counter value matches WINCMP value.
     * |        |          |0 = No effect.
     * |        |          |1 = WWDT counter value matches WINCMP value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |WWDTRF    |WWDT Time-out Reset Flag
     * |        |          |This bit indicates the system has been reset by WWDT time-out reset or not.
     * |        |          |0 = WWDT time-out reset did not occur.
     * |        |          |1 = WWDT time-out reset occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var WWDT_T::WWDTCVR
     * Offset: 0x0C  Window Watchdog Timer Counter Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |WWDTCVAL  |WWDT Counter Value
     * |        |          |WWDTCVAL will be updated continuously to monitor 6-bit down counter value.
     */

    __IO uint32_t WWDTRLD;       /* Offset: 0x00  Window Watchdog Timer Reload Counter Register                      */
    __IO uint32_t WWDTCR;        /* Offset: 0x04  Window Watchdog Timer Control Register                             */
    __IO uint32_t WWDTSR;        /* Offset: 0x08  Window Watchdog Timer Status Register                              */
    __I  uint32_t WWDTCVR;       /* Offset: 0x0C  Window Watchdog Timer Counter Value Register                       */

} WWDT_T;



/**
    @addtogroup WWDT_CONST WWDT Bit Field Definition
    Constant Definitions for WWDT Controller
@{ */

/* WWDT WWDTRLD Bit Field Definitions */
#define WWDT_WWDTRLD_WWDTRLD_Pos    0                                           /*!< WWDT_T::WWDTRLD: WWDTRLD Position */
#define WWDT_WWDTRLD_WWDTRLD_Msk    (0xFFFFFFFFul << WWDT_WWDTRLD_WWDTRLD_Pos)  /*!< WWDT_T::WWDTRLD: WWDTRLD Mask */

/* WWDT WWDTCR Bit Field Definitions */
#define WWDT_WWDTCR_DBGACK_WWDT_Pos 31                                          /*!< WWDT_T::WWDTCR: DBGACK_WWDT Position */
#define WWDT_WWDTCR_DBGACK_WWDT_Msk (1ul << WWDT_WWDTCR_DBGACK_WWDT_Pos)        /*!< WWDT_T::WWDTCR: DBGACK_WWDT Mask */

#define WWDT_WWDTCR_WINCMP_Pos      16                                          /*!< WWDT_T::WWDTCR: WINCMP Position */
#define WWDT_WWDTCR_WINCMP_Msk      (0x3Ful << WWDT_WWDTCR_WINCMP_Pos)          /*!< WWDT_T::WWDTCR: WINCMP Mask */

#define WWDT_WWDTCR_PERIODSEL_Pos   8                                           /*!< WWDT_T::WWDTCR: PERIODSEL Position */
#define WWDT_WWDTCR_PERIODSEL_Msk   (0xFul << WWDT_WWDTCR_PERIODSEL_Pos)        /*!< WWDT_T::WWDTCR: PERIODSEL Mask */

#define WWDT_WWDTCR_WWDTIE_Pos      1                                           /*!< WWDT_T::WWDTCR: WWDTIE Position */
#define WWDT_WWDTCR_WWDTIE_Msk      (1ul << WWDT_WWDTCR_WWDTIE_Pos)             /*!< WWDT_T::WWDTCR: WWDTIE Mask */

#define WWDT_WWDTCR_WWDTEN_Pos      0                                           /*!< WWDT_T::WWDTCR: WWDTEN Position */
#define WWDT_WWDTCR_WWDTEN_Msk      (1ul << WWDT_WWDTCR_WWDTEN_Pos)             /*!< WWDT_T::WWDTCR: WWDTEN Mask */

/* WWDT WWDTSR Bit Field Definitions */
#define WWDT_WWDTSR_WWDTRF_Pos      1                                           /*!< WWDT_T::WWDTSR: WWDTRF Position */
#define WWDT_WWDTSR_WWDTRF_Msk      (1ul << WWDT_WWDTSR_WWDTRF_Pos)             /*!< WWDT_T::WWDTSR: WWDTRF Mask */

#define WWDT_WWDTSR_WWDTIF_Pos      0                                           /*!< WWDT_T::WWDTSR: WWDTIF Position */
#define WWDT_WWDTSR_WWDTIF_Msk      (1ul << WWDT_WWDTSR_WWDTIF_Pos)             /*!< WWDT_T::WWDTSR: WWDTIF Mask */

/* WWDT WWDTCVR Bit Field Definitions */
#define WWDT_WWDTCVR_WWDTCVAL_Pos   0                                           /*!< WWDT_T::WWDTCVR: WWDTRF Position */
#define WWDT_WWDTCVR_WWDTCVAL_Msk   (0x3Ful << WWDT_WWDTCVR_WWDTCVAL_Pos)       /*!< WWDT_T::WWDTCVR: WWDTRF Mask */
/*@}*/ /* end of group WWDT_CONST */
/*@}*/ /* end of group WWDT */
/*@}*/ /* end of group REGISTER */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup PERIPHERAL_MEM_MAP Peripheral Memory Map
  Memory Mapped Structure for Series Peripheral
  @{
 */
/* Peripheral and SRAM base address */
#define FLASH_BASE          ((     uint32_t)0x00000000)
#define SRAM_BASE           ((     uint32_t)0x20000000)
#define AHB_BASE            ((     uint32_t)0x50000000)
#define APB1_BASE           ((     uint32_t)0x40000000)
#define APB2_BASE           ((     uint32_t)0x40100000)

/* Peripheral memory map */
#define GPIO_BASE           (AHB_BASE       + 0x4000)                   /*!< GPIO Base Address                                   */
#define P0_BASE             (GPIO_BASE              )                   /*!< GPIO P0 Base Address                                */
#define P1_BASE             (GPIO_BASE      + 0x0040)                   /*!< GPIO P1 Base Address                                */
#define P2_BASE             (GPIO_BASE      + 0x0080)                   /*!< GPIO P2 Base Address                                */
#define P3_BASE             (GPIO_BASE      + 0x00C0)                   /*!< GPIO P3 Base Address                                */
#define P4_BASE             (GPIO_BASE      + 0x0100)                   /*!< GPIO P4 Base Address                                */
#define P5_BASE             (GPIO_BASE      + 0x0140)                   /*!< GPIO P5 Base Address                                */
#define P6_BASE             (GPIO_BASE      + 0x0180)                   /*!< GPIO P6 Base Address                                */
#define P7_BASE             (GPIO_BASE      + 0x01C0)                   /*!< GPIO P7 Base Address                                */
#define P8_BASE             (GPIO_BASE      + 0x0200)                   /*!< GPIO P8 Base Address                                */
#define P9_BASE             (GPIO_BASE      + 0x0240)                   /*!< GPIO P9 Base Address                                */
#define PA_BASE             (GPIO_BASE      + 0x0280)                   /*!< GPIO PA Base Address                                */
#define GPIO_DBNCECON_BASE  (GPIO_BASE      + 0x02E0)                   /*!< GPIO De-bounce Cycle Control Base Address           */
#define GPIO_PIN_DATA_BASE  (GPIO_BASE      + 0x0300)                   /*!< GPIO Pin Data Input/Output Control Base Address     */






#define UART0_BASE           (APB1_BASE      + 0x50000)                 /*!< UART0 Base Address                               */
#define UART1_BASE           (APB2_BASE      + 0x50000)                 /*!< UART1 Base Address                               */


#define TIMER0_BASE          (APB1_BASE      + 0x10000)                 /*!< Timer0 Base Address                              */
#define TIMER1_BASE          (APB1_BASE      + 0x10020)                 /*!< Timer1 Base Address                              */
#define TIMER2_BASE          (APB2_BASE      + 0x10000)                 /*!< Timer2 Base Address                              */
#define TIMER3_BASE          (APB2_BASE      + 0x10020)                 /*!< Timer3 Base Address                              */

#define WDT_BASE             (APB1_BASE      + 0x4000)                  /*!< Watchdog Timer Base Address                      */

#define WWDT_BASE            (APB1_BASE      + 0x4100)                  /*!< Window Watchdog Timer Base Address               */

#define SPI0_BASE            (APB1_BASE      + 0x30000)                 /*!< SPI0 Base Address                                */
#define SPI1_BASE            (APB1_BASE      + 0x34000)                 /*!< SPI1 Base Address                                */
#define SPI2_BASE            (APB2_BASE      + 0x30000)                 /*!< SPI2 Base Address                                */

#define I2C0_BASE            (APB1_BASE      + 0x20000)                 /*!< I2C0 Base Address                                */

#define EADC_BASE            (APB1_BASE      + 0xE0000)                 /*!< EADC Base Address                                */

#define ACMP_BASE            (APB1_BASE      + 0xD0000)                 /*!< ACMP Base Address                                */

#define OPA_BASE             (APB1_BASE      + 0xF0000)                 /*!< OPA Base Address                                 */

#define BPWM0_BASE           (APB1_BASE      + 0x40000)                 /*!< BPWM0 Base Address                               */

#define EPWM0_BASE           (APB2_BASE      + 0x90000)                 /*!< EPWM0 Base Address                               */
#define EPWM1_BASE           (APB2_BASE      + 0x94000)                 /*!< EPWM1 Base Address                               */

#define CLK_BASE             (AHB_BASE       + 0x00200)                 /*!< System Clock Controller Base Address             */

#define GCR_BASE             (AHB_BASE       + 0x00000)                 /*!< System Global Controller Base Address            */

#define INT_BASE             (AHB_BASE       + 0x00300)                 /*!< Interrupt Source Controller Base Address         */

#define FMC_BASE             (AHB_BASE       + 0x0C000)                 /*!< FMC Base Address                                */

#define HDIV_BASE            (AHB_BASE       + 0x14000)                 /*!< HDIV Base Address                                */

#define ECAP0_BASE           (APB2_BASE      + 0xB0000)                 /*!< ECAP0 Base Address                               */
#define ECAP1_BASE           (APB2_BASE      + 0xB4000)                 /*!< ECAP1 Base Address                               */


/*@}*/ /* end of group PERIPHERAL_MEM_MAP */

/******************************************************************************/
/*                         Peripheral Definitions                             */
/******************************************************************************/

/** @addtogroup PERIPHERAL Peripheral Definitions
  The Definitions of Series Peripheral
  @{
 */
#define P0                  ((GPIO_T *) P0_BASE)                        /*!< GPIO PORT0 Configuration Struct                        */
#define P1                  ((GPIO_T *) P1_BASE)                        /*!< GPIO PORT1 Configuration Struct                        */
#define P2                  ((GPIO_T *) P2_BASE)                        /*!< GPIO PORT2 Configuration Struct                        */
#define P3                  ((GPIO_T *) P3_BASE)                        /*!< GPIO PORT3 Configuration Struct                        */
#define P4                  ((GPIO_T *) P4_BASE)                        /*!< GPIO PORT4 Configuration Struct                        */
#define P5                  ((GPIO_T *) P5_BASE)                        /*!< GPIO PORT5 Configuration Struct                        */
#define P6                  ((GPIO_T *) P6_BASE)                        /*!< GPIO PORT6 Configuration Struct                        */
#define P7                  ((GPIO_T *) P7_BASE)                        /*!< GPIO PORT7 Configuration Struct                        */
#define P8                  ((GPIO_T *) P8_BASE)                        /*!< GPIO PORT8 Configuration Struct                        */
#define P9                  ((GPIO_T *) P9_BASE)                        /*!< GPIO PORT9 Configuration Struct                        */
#define PA                  ((GPIO_T *) PA_BASE)                        /*!< GPIO PORTA Configuration Struct                        */
#define GPIO                ((GPIO_DBNCECON_T *) GPIO_DBNCECON_BASE)    /*!< Interrupt De-bounce Cycle Control Configuration Struct */

#define UART0               ((UART_T *) UART0_BASE)                     /*!< UART0 Configuration Struct                       */
#define UART1               ((UART_T *) UART1_BASE)                     /*!< UART1 Configuration Struct                       */

#define TIMER0              ((TIMER_T *) TIMER0_BASE)                   /*!< Timer0 Configuration Struct                      */
#define TIMER1              ((TIMER_T *) TIMER1_BASE)                   /*!< Timer1 Configuration Struct                      */
#define TIMER2              ((TIMER_T *) TIMER2_BASE)                   /*!< Timer2 Configuration Struct                      */
#define TIMER3              ((TIMER_T *) TIMER3_BASE)                   /*!< Timer3 Configuration Struct                      */

#define WDT                 ((WDT_T *) WDT_BASE)                        /*!< Watchdog Timer Configuration Struct              */

#define WWDT                ((WWDT_T *) WWDT_BASE)                      /*!< Window Watchdog Timer Configuration Struct       */

#define SPI0                ((SPI_T *) SPI0_BASE)                       /*!< SPI0 Configuration Struct                        */
#define SPI1                ((SPI_T *) SPI1_BASE)                       /*!< SPI1 Configuration Struct                        */
#define SPI2                ((SPI_T *) SPI2_BASE)                       /*!< SPI2 Configuration Struct                        */

#define I2C0                ((I2C_T *) I2C0_BASE)                       /*!< I2C0 Configuration Struct                        */

#define EADC                ((EADC_T *) EADC_BASE)                      /*!< EADC Configuration Struct                        */

#define ACMP                ((ACMP_T *) ACMP_BASE)                      /*!< ACMP Configuration Struct                        */

#define OPA                 ((OPA_T *) OPA_BASE)                        /*!< OPA Configuration Struct                         */

#define CLK                 ((CLK_T *) CLK_BASE)                        /*!< System Clock Controller Configuration Struct     */

#define SYS                 ((GCR_T *) GCR_BASE)                        /*!< System Global Controller Configuration Struct    */

#define SYSINT              ((GCR_INT_T *) INT_BASE)                    /*!< Interrupt Source Controller Configuration Struct */

#define FMC                 ((FMC_T *) FMC_BASE)

#define BPWM0               ((BPWM_T *) BPWM0_BASE)                     /*!< BPWM0 Configuration Struct                       */

#define EPWM0               ((EPWM_T *) EPWM0_BASE)                     /*!< EPWM0 Configuration Struct                       */
#define EPWM1               ((EPWM_T *) EPWM1_BASE)                     /*!< EPWM1 Configuration Struct                       */

#define HDIV                ((HDIV_T *) HDIV_BASE)                      /*!< HDIV Configuration Struct                        */

#define ECAP0               ((ECAP_T *) ECAP0_BASE)                     /*!< ECAP0 Configuration Struct                       */
#define ECAP1               ((ECAP_T *) ECAP1_BASE)                     /*!< ECAP1 Configuration Struct                       */

/*@}*/ /* end of group PERIPHERAL */

//=============================================================================
/** @addtogroup IO_ROUTINE I/O routines
  The Declaration of I/O routines
  @{
 */

typedef volatile unsigned char  vu8;        ///< Define 8-bit unsigned volatile data type
typedef volatile unsigned short vu16;       ///< Define 16-bit unsigned volatile data type
typedef volatile unsigned long  vu32;       ///< Define 32-bit unsigned volatile data type

/**
  * @brief Get a 8-bit unsigned value from specified address
  * @param[in] addr Address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified address
  */
#define M8(addr)  (*((vu8  *) (addr)))

/**
  * @brief Get a 16-bit unsigned value from specified address
  * @param[in] addr Address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified address
  * @note The input address must be 16-bit aligned
  */
#define M16(addr) (*((vu16 *) (addr)))

/**
  * @brief Get a 32-bit unsigned value from specified address
  * @param[in] addr Address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified address
  * @note The input address must be 32-bit aligned
  */
#define M32(addr) (*((vu32 *) (addr)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outpw(port,value)     *((volatile unsigned int *)(port)) = (value)

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inpw(port)            (*((volatile unsigned int *)(port)))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outps(port,value)     *((volatile unsigned short *)(port)) = (value)

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inps(port)            (*((volatile unsigned short *)(port)))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outpb(port,value)     *((volatile unsigned char *)(port)) = (value)

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inpb(port)            (*((volatile unsigned char *)(port)))

/**
  * @brief Set a 32-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 32-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 32-bit aligned
  */
#define outp32(port,value)    *((volatile unsigned int *)(port)) = (value)

/**
  * @brief Get a 32-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 32-bit data from
  * @return  32-bit unsigned value stored in specified I/O port
  * @note The input port must be 32-bit aligned
  */
#define inp32(port)           (*((volatile unsigned int *)(port)))

/**
  * @brief Set a 16-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 16-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  * @note The output port must be 16-bit aligned
  */
#define outp16(port,value)    *((volatile unsigned short *)(port)) = (value)

/**
  * @brief Get a 16-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 16-bit data from
  * @return  16-bit unsigned value stored in specified I/O port
  * @note The input port must be 16-bit aligned
  */
#define inp16(port)           (*((volatile unsigned short *)(port)))

/**
  * @brief Set a 8-bit unsigned value to specified I/O port
  * @param[in] port Port address to set 8-bit data
  * @param[in] value Value to write to I/O port
  * @return  None
  */
#define outp8(port,value)     *((volatile unsigned char *)(port)) = (value)

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inp8(port)            (*((volatile unsigned char *)(port)))


#define UNLOCKREG(x)        do{M32(GCR_BASE + 0x100) = 0x59;M32(GCR_BASE + 0x100) = 0x16;M32(GCR_BASE + 0x100) = 0x88;}while(M32(GCR_BASE + 0x100)==0)
#define LOCKREG(x)          M32(GCR_BASE + 0x100) = 0x00

#define REGCOPY(dest, src)  *((uint32_t *)&(dest)) = *((uint32_t *)&(src))
#define CLEAR(dest)         *((uint32_t *)&(dest)) = 0

/*@}*/ /* end of group IO_ROUTINE */

/** @addtogroup legacy_Constants Legacy Constants
  Legacy Constants
  @{
*/


#define E_SUCCESS   0
#ifndef NULL
#define NULL        0
#endif

#define TRUE        1
#define FALSE       0

#define ENABLE     1
#define DISABLE    0

/* Define one bit mask */
#define BIT0    0x00000001
#define BIT1    0x00000002
#define BIT2    0x00000004
#define BIT3    0x00000008
#define BIT4    0x00000010
#define BIT5    0x00000020
#define BIT6    0x00000040
#define BIT7    0x00000080
#define BIT8    0x00000100
#define BIT9    0x00000200
#define BIT10   0x00000400
#define BIT11   0x00000800
#define BIT12   0x00001000
#define BIT13   0x00002000
#define BIT14   0x00004000
#define BIT15   0x00008000
#define BIT16   0x00010000
#define BIT17   0x00020000
#define BIT18   0x00040000
#define BIT19   0x00080000
#define BIT20   0x00100000
#define BIT21   0x00200000
#define BIT22   0x00400000
#define BIT23   0x00800000
#define BIT24   0x01000000
#define BIT25   0x02000000
#define BIT26   0x04000000
#define BIT27   0x08000000
#define BIT28   0x10000000
#define BIT29   0x20000000
#define BIT30   0x40000000
#define BIT31   0x80000000

/* Byte Mask Definitions */
#define BYTE0_Msk               (0x000000FFul)
#define BYTE1_Msk               (0x0000FF00ul)
#define BYTE2_Msk               (0x00FF0000ul)
#define BYTE3_Msk               (0xFF000000ul)

#define _GET_BYTE0(u32Param)    (((u32Param) & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define _GET_BYTE1(u32Param)    (((u32Param) & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define _GET_BYTE2(u32Param)    (((u32Param) & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define _GET_BYTE3(u32Param)    (((u32Param) & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/*@}*/ /* end of group legacy_Constants */

/*@}*/ /* end of group Definitions */


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "SYS.h"
#include "eadc.h"
#include "FMC.h"
#include "GPIO.h"
#include "I2C.h"
#include "bpwm.h"
#include "epwm.h"
#include "SPI.h"
#include "TIMER.h"
#include "WDT.h"
#include "WWDT.h"
#include "UART.h"
#include "CLK.h"
#include "ACMP.h"
#include "hdiv.h"
#include "opa.h"
#include "ecap.h"
#endif



