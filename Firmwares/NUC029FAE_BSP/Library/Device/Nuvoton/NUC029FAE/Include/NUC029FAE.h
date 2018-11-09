/**************************************************************************//**
 * @file     NUC029FAE.h
 * @version  V1.00
 * $Revision: 6 $
 * $Date: 14/11/28 6:34p $
 * @brief    NUC029FAE peripheral access layer header file.
 *           This file contains all the peripheral register's definitions,
 *           bits definitions and memory mapping for NuMicro NUC029FAE MCU.
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/**
   \mainpage NuMicro NUC029FAE Driver Reference Guide
   *
   * <b>Introduction</b>
   *
   * This user manual describes the usage of NUC029FAE MCU device driver
   *
   * <b>Disclaimer</b>
   *
   * The Software is furnished "AS IS", without warranty as to performance or results, and
   * the entire risk as to performance or results is assumed by YOU. Nuvoton disclaims all
   * warranties, express, implied or otherwise, with regard to the Software, its use, or
   * operation, including without limitation any and all warranties of merchantability, fitness
   * for a particular purpose, and non-infringement of intellectual property rights.
   *
   * <b>Important Notice</b>
   *
   * Nuvoton Products are neither intended nor warranted for usage in systems or equipment,
   * any malfunction or failure of which may cause loss of human life, bodily injury or severe
   * property damage. Such applications are deemed, "Insecure Usage".
   *
   * Insecure usage includes, but is not limited to: equipment for surgical implementation,
   * atomic energy control instruments, airplane or spaceship instruments, the control or
   * operation of dynamic, brake or safety systems designed for vehicular use, traffic signal
   * instruments, all types of safety devices, and other applications intended to support or
   * sustain life.
   *
   * All Insecure Usage shall be made at customer's risk, and in the event that third parties
   * lay claims to Nuvoton as a result of customer's Insecure Usage, customer shall indemnify
   * the damages and liabilities thus incurred by Nuvoton.
   *
   * Please note that all data and specifications are subject to change without notice. All the
   * trademarks of products and companies mentioned in this datasheet belong to their respective
   * owners.
   *
   * <b>Copyright Notice</b>
   *
   * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
   */


#ifndef __NUC029FAE_H__
#define __NUC029FAE_H__

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup NUC029FAE_CMSIS NUC029FAE Device CMSIS Definitions
  Configuration of the Cortex-M0 Processor and Core Peripherals
  @{
*/

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */
typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers *****************************************/

    NonMaskableInt_IRQn   = -14,    /*!< 2 Non Maskable Interrupt                           */
    HardFault_IRQn        = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                   */
    SVCall_IRQn           = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                     */
    PendSV_IRQn           = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                     */
    SysTick_IRQn          = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                 */

    /******  NUC029FAE specific Interrupt Numbers ***********************************************/

    BOD_IRQn              = 0,      /*!< Brownout low voltage detected interrupt            */
    WDT_IRQn              = 1,      /*!< Watch Dog Timer interrupt                          */
    EINT0_IRQn            = 2,      /*!< External signal interrupt from P3.2 pin            */
    EINT1_IRQn            = 3,      /*!< External signal interrupt from P3.3 pin            */
    GPIO01_IRQn           = 4,      /*!< External signal interrupt from P0/P1               */
    GPIO234_IRQn          = 5,      /*!< External interrupt from P2/P3/P4                   */
    PWM_IRQn              = 6,      /*!< PWM interrupt                                      */
    FB_IRQn               = 7,      /*!< Fault brake interrupt                              */
    TMR0_IRQn             = 8,      /*!< Timer 0 interrupt                                  */
    TMR1_IRQn             = 9,      /*!< Timer 1 interrupt                                  */
    UART_IRQn             = 12,     /*!< UART interrupt                                     */
    SPI_IRQn              = 14,     /*!< SPI interrupt                                      */
    GPIO5_IRQn            = 16,     /*!< External interrupt from P5                         */
    HIRC_IRQn             = 17,     /*!< HIRC trim interrupt                                */
    I2C_IRQn              = 18,     /*!< I2C interrupt                                      */
    ACMP_IRQn             = 25,     /*!< ACMP interrupt                                     */
    PDWU_IRQn             = 28,     /*!< Power Down Wake up interrupt                       */
    ADC_IRQn              = 29      /*!< ADC interrupt                                      */

} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */


/* Configuration of the Cortex-M0 Processor and Core Peripherals */
#define __CM0_REV                0x0201    /*!< Core Revision r2p1                               */
#define __NVIC_PRIO_BITS         2         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig   0         /*!< Set to 1 if different SysTick Config is used     */
#define __MPU_PRESENT            0         /*!< MPU present or not                               */
#define __FPU_PRESENT            0         /*!< FPU present or not                               */

/*@}*/ /* end of group NUC029FAE_CMSIS */


#include "core_cm0.h"                       /* Cortex-M0 processor and core peripherals           */
#include "system_NUC029FAE.h"            /* NUC029FAE System include file                  */
#include <stdint.h>

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup NUC029FAE_Peripherals NUC029FAE Control Register
  NUC029FAE Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM  )
#pragma anon_unions
#endif


/*---------------------- Analog Comparator Controller -------------------------*/
/** @addtogroup ACMP Analog Comparator Controller(ACMP)
  Memory Mapped Structure for ACMP Controller
  @{
 */
typedef struct
{
    __IO uint32_t  CMPCR[2];      /*!< Offset: 0x0000, 0x0004   Comparator Control 0 & 1                 */
    __IO uint32_t  CMPSR;         /*!< Offset: 0x0008   Comparator Status Register                      */
    __IO uint32_t  CMPRVCR;       /*!< Offset: 0x000C   Comparator Reference Voltage Control Register   */
} ACMP_T;
/// @cond HIDDEN_SYMBOLS
/* CMPCR Bit Field Definitions */
#define ACMP_CMPCR_CPPSEL_Pos        29                                        /*!< ACMP CMPCR: CPPSEL Position     */
#define ACMP_CMPCR_CPPSEL_Msk        (3ul << ACMP_CMPCR_CPPSEL_Pos)            /*!< ACMP CMPCR: CPPSEL Mask         */

#define ACMP_CMPCR_FALLING_Pos       9                                         /*!< ACMP CMPCR: FALLING Position    */
#define ACMP_CMPCR_FALLING_Msk       (1ul << ACMP_CMPCR_FALLING_Pos)           /*!< ACMP CMPCR: FALLING Mask        */

#define ACMP_CMPCR_RISING_Pos       8                                          /*!< ACMP CMPCR: RISING Position     */
#define ACMP_CMPCR_RISING_Msk       (1ul << ACMP_CMPCR_RISING_Pos)             /*!< ACMP CMPCR: RISING Mask         */

#define ACMP_CMPCR_NEGSEL_Pos       4                                          /*!< ACMP CMPCR: NEGSEL Position     */
#define ACMP_CMPCR_NEGSEL_Msk       (1ul << ACMP_CMPCR_NEGSEL_Pos)             /*!< ACMP CMPCR: NEGSEL Mask         */

#define ACMP_CMPCR_HYSEN_Pos        2                                          /*!< ACMP CMPCR: HYSEN Position      */
#define ACMP_CMPCR_HYSEN_Msk        (1ul << ACMP_CMPCR_HYSEN_Pos)              /*!< ACMP CMPCR: HYSEN Mask          */

#define ACMP_CMPCR_ACMPIE_Pos       1                                          /*!< ACMP CMPCR: ACMPIE Position     */
#define ACMP_CMPCR_ACMPIE_Msk       (1ul << ACMP_CMPCR_ACMPIE_Pos)             /*!< ACMP CMPCR: ACMPIE Mask         */

#define ACMP_CMPCR_ACMPEN_Pos       0                                          /*!< ACMP CMPCR: ACMPEN Position     */
#define ACMP_CMPCR_ACMPEN_Msk       (1ul << ACMP_CMPCR_ACMPEN_Pos)             /*!< ACMP CMPCR: ACMPEN Mask         */

/* CMPSR Bit Field Definitions */
#define ACMP_CMPSR_ACMPCO1_Pos      3                                          /*!< ACMP CMPSR: ACMPCO1 Position    */
#define ACMP_CMPSR_ACMPCO1_Msk      (1ul << ACMP_CMPSR_ACMPCO1_Pos)            /*!< ACMP CMPSR: ACMPCO1 Mask        */

#define ACMP_CMPSR_ACMPCO0_Pos      2                                          /*!< ACMP CMPSR: ACMPCO0 Position    */
#define ACMP_CMPSR_ACMPCO0_Msk      (1ul << ACMP_CMPSR_ACMPCO0_Pos)            /*!< ACMP CMPSR: ACMPCO0 Mask        */

#define ACMP_CMPSR_ACMPF1_Pos       1                                          /*!< ACMP CMPSR: ACMPF1 Position     */
#define ACMP_CMPSR_ACMPF1_Msk       (1ul << ACMP_CMPSR_ACMPF1_Pos)             /*!< ACMP CMPSR: ACMPF1 Mask         */

#define ACMP_CMPSR_ACMPF0_Pos       0                                          /*!< ACMP CMPSR: ACMPF0 Position     */
#define ACMP_CMPSR_ACMPF0_Msk       (1ul << ACMP_CMPSR_ACMPF0_Pos)             /*!< ACMP CMPSR: ACMPF0 Mask         */

#define ACMP_CMPRVCR_OUT_SEL_Pos    7                                          /*!< ACMP CMPRVCR: OUT_SEL Position  */
#define ACMP_CMPRVCR_OUT_SEL_Msk    (1ul << ACMP_CMPRVCR_OUT_SEL_Pos)          /*!< ACMP CMPRVCR: OUT_SEL Mask      */

#define ACMP_CMPRVCR_CRVS_Pos       0                                          /*!< ACMP CMPRVCR: CRVS Position     */
#define ACMP_CMPRVCR_CRVS_Msk       (0xFul << ACMP_CMPRVCR_CRVS_Pos)           /*!< ACMP CMPRVCR: CRVS Mask         */
/// @endcond /* HIDDEN_SYMBOLS */
/*@}*/ /* end of group NUC029FAE_ACMP */


/*---------------------------- Clock Controller ------------------------------*/

/** @addtogroup CLK System Clock Controller(CLK)
  Memory Mapped Structure for CLK Controller
  @{
 */
typedef struct
{
    /**
     * PWRCON
     * ===================================================================================================
     * Offset: 0x00  System Power Down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field       |Descriptions
     * | :----: | :----:     | :---- |
     * |[1:0]   |XTLCLK_EN   |External Crystal Oscillator Control
     * |        |            |The default clock source is from internal 22.1184 MHz. These two bits are default set to "00"
     * |             |                         |and the XTAL1 and XTAL2 pins are GPIO.
     * |        |            |00 = XTAL1 and XTAL2 are GPIO, disable both XTL32K & XTAL12M
     * |        |            |01 = XTAL12M (HXT) Enabled
     * |        |            |10 = XTAL32K (LXT) Enabled
     * |        |            |11 = XTAL1 is external clock input pin, XTAL2 is GPIO
     * |        |            |Note: To enable external XTAL function, P5_ALT[1:0] and P5_MFP[1:0] bits must also be set in P5_MFP.
     * |[2]     |OSC22M_EN   |Internal 22.1184 MHz Oscillator Control
     * |        |            |1 = 22.1184 MHz Oscillation enable
     * |        |            |0 = 22.1184 MHz Oscillation disable
     * |[3]     |OSC10K_EN   |Internal 10KHz Oscillator Control
     * |        |            |1 = 10KHz Oscillation enable
     * |        |            |0 = 10KHz Oscillation disable
     * |[4]     |PU_DLY      |Enable the wake up delay counter.
     * |        |            |When the chip wakes up from power down mode, the clock control will delay certain clock
     * |        |            |cycles to wait system clock stable.
     * |        |            |The delayed clock cycle is 4096 clock cycles when chip work at external crystal (4 ~
     * |        |            |24MHz), and 256 clock cycles when chip work at 22.1184 MHz oscillator.
     * |        |            |1 = Enable the clock cycle delay
     * |        |            |0 = Disable the clock cycle delay
     * |[5]     |WINT_EN     |Power down mode wake Up Interrupt Enable
     * |        |            |0 = Disable
     * |        |            |1 = Enable. The interrupt will occur when Power down mode wakeup.
     * |[6]     |PD_WU_STS   |Chip power down wake up status flag
     * |        |            |Set by "power down wake up", it indicates that resume from power down mode
     * |        |            |The flag is set if the GPIO, UART, WDT, ACMP, Timer or BOD wakeup
     * |        |            |Write 1 to clear the bit
     * |        |            |Note: This bit is working only if PD_WU_INT_EN (PWRCON[5]) set to 1.
     * |[7]     |PWR_DOWN    |System Power-down Active or Enable Bit
     * |        |            |When chip waked-up from power-down, this bit is automatically cleared, and user needs to set
     * |        |            |this bit again for the next power-down.
     * |        |            |In Power-down mode, the LDO, external crystal and the 22.1184 MHz OSC will be disabled, and
     * |        |            |the 10K enable is not controlled by this bit.
     * |        |            |Note: If XTLCLK_EN[1:0] = 10 (enable 32 KHz External Crystal Oscillator) and when PWR_DOWN_EN ="1" (system entering
     * |        |            | Power-down mode), the external crystal oscillator cannot be disabled to ensure system wake-up enabled.
     * |        |            |When power down, all of the AMBA clocks (HCLKx, CPU clock and the PCLKx) are also disabled, and the clock
     * |        |            |source selection is ignored. The IP engine clock is not controlled by this bit if the IP clock source is from
     * |        |            |the 10K clock and the WDT from 10K).
     * |        |            |1 = Chip entering the Power-down mode instantly or wait CPU Idle command
     * |        |            |0 = Chip operated in Normal mode or CPU enters into Idle mode.
     * |[9]     |PD_32K      |This bit controls the crystal oscillator active or not in Power-down mode.
     * |        |            |1 = If XTLCLK_EN[1:0] = 10, 32.768 KHz crystal oscillator (LXT) is still active in Power-down mode.
     * |        |            |0 = No effect to Power-down mode
     */
    __IO uint32_t  PWRCON;

    /**
     * AHBCLK
     * ===================================================================================================
     * Offset: 0x04  AHB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |ISP_EN    |Flash ISP Controller Clock Enable Control.
     * |        |          |1 = To enable the Flash ISP controller clock.
     * |        |          |0 = To disable the Flash ISP controller clock.
     */
    __IO uint32_t  AHBCLK;

    /**
     * APBCLK
     * ===================================================================================================
     * Offset: 0x08  APB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDT_EN    |Watch Dog Timer Clock Enable.
     * |        |          |This bit is the protected bit, program this need a open lock sequence, write "59h","16h","88h" to
     * |        |          |address 0x5000_0100 to un-lock this bit. Reference the register REGWRPROT at address
     * |        |          |SYS_BA + 0x100
     * |        |          |0 = Disable Watchdog Timer Clock
     * |        |          |1 = Enable Watchdog Timer Clock
     * |[2]     |TMR0_EN   |Timer0 Clock Enable Control
     * |        |          |0 = Disable Timer0 Clock
     * |        |          |1 = Enable Timer0 Clock
     * |[3]     |TMR1_EN   |Timer1 Clock Enable Control
     * |        |          |0 = Disable Timer1 Clock
     * |        |          |1 = Enable Timer1 Clock
     * |[6]     |FDIV_EN   |Clock Divider Clock Enable Control
     * |        |          |0 = Disable FDIV Clock
     * |        |          |1 = Enable FDIV Clock
     * |[8]     |I2C_EN    |I2C Clock Enable Control.
     * |        |          |0 = Disable I2C Clock
     * |        |          |1 = Enable I2C Clock
     * |[12]    |SPI_EN    |SPI Clock Enable Control.
     * |        |          |0 = Disable SPI Clock
     * |        |          |1 = Enable SPI Clock
     * |[16]    |UART_EN   |UART Clock Enable Control.
     * |        |          |1 = Enable UART clock
     * |        |          |0 = Disable UART clock
     * |[20]    |PWM01_EN  |PWM_01 Clock Enable Control.
     * |        |          |1 = Enable PWM01 clock
     * |        |          |0 = Disable PWM01 clock
     * |[21]    |PWM23_EN  |PWM_23 Clock Enable Control.
     * |        |          |1 = Enable PWM23 clock
     * |        |          |0 = Disable PWM23 clock
     * |[22]    |PWM45_EN  |PWM_45 Clock Enable Control.
     * |        |          |1 = Enable PWM45 clock
     * |        |          |0 = Disable PWM45 clock
     * |[28]    |ADC_EN    |Analog-Digital-Converter (ADC) Clock Enable Control.
     * |        |          |1 = Enable ADC clock
     * |        |          |0 = Disable ADC clock
     * |[30]    |CMP_EN    |Comparator Clock Enable Control.
     * |        |          |1 = Enable Analog Comparator clock
     * |        |          |0 = Disable Analog Comparator clock
     */
    __IO uint32_t  APBCLK;

    /**
     * CLKSTATUS
     * ===================================================================================================
     * Offset: 0x0C  Clock Status Monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field      |Descriptions
     * | :----: | :----:    | :---- |
     * |[0]     |XTL_STB    |XTL12M or XTL32K clock source stable flag
     * |        |           |1 = External Crystal clock is stable
     * |        |           |0 = External Crystal clock is not stable or not enable
     * |[3]     |OSC10K_STB |OSC10K clock source stable flag
     * |        |           |1 = OSC10K clock is stable
     * |        |           |0 = OSC10K clock is not stable or not enable
     * |[4]     |OSC22M_STB |OSC22M clock source stable flag
     * |        |           |1 = OSC22M clock is stable
     * |        |           |0 = OSC22M clock is not stable or not enable
     * |[7]     |CLK_SW_FAIL|Clock switch fail flag
     * |        |           |1 = Clock switch fail
     * |        |           |0 = Clock switch success
     * |        |           |This bit will be set when target switch clock source is not stable. Write 1 to clear this bit to zero.
     */
    __IO uint32_t  CLKSTATUS;

    /**
     * CLKSEL0
     * ===================================================================================================
     * Offset: 0x10  Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HCLK_S    |HCLK clock source select.
     * |        |          |Note:
     * |        |          |1.    Before clock switch the related clock sources (pre-select and new-select) must be turn on
     * |        |          |2.    These bits are protected bit, program this need an open lock sequence, write
     * |        |          |"59h","16h","88h" to address 0x5000_0100 to un-lock this bit. Reference the register
     * |        |          |REGWRPROT at address SYS_BA + 0x100
     * |        |          |3.  To set PWRCON[1:0] to select 12 MHz or 32 KHz crystal clock.
     * |        |          |000 = Clock source from external 12 MHz or 32 KHz crystal clock.
     * |        |          |011 = clock source from internal 10KHz oscillator clock
     * |        |          |111 = clock source from internal 22.1184 MHz oscillator clock
     * |        |          |others = Reserved
     * |[5:3]   |STCLK_S   |MCU Cortex_M0 SysTick clock source select.
     * |        |          |These bits are protected bit, program this need an open lock sequence, write "59h","16h","88h" to
     * |        |          |address 0x5000_0100 to un-lock this bit. Reference the register REGWRPROT at address SYS_BA
     * |        |          |+ 0x100
     * |        |          |000 = Clock source from 12 MHz or 32 KHz crystal clock
     * |        |          |010 = Clock source from 12 MHz or 32 KHz crystal clock/2
     * |        |          |011 = clock source from HCLK/2
     * |        |          |111 = clock source from internal 22.1184 MHz oscillator clock/2
     * |        |          |others = Reserved
     * |        |          |Note: To set PWRCON[1:0] to select 12 MHz or 32 KHz crystal clock.
     */
    __IO uint32_t  CLKSEL0;

    /**
     * CLKSEL1
     * ===================================================================================================
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDT_S     |Watchdog Timer clock source select.
     * |        |          |These bits are protected bit, program this need a open lock sequence, write "59h","16h","88h" to
     * |        |          |address 0x5000_0100 to un-lock this bit. Reference the register REGWRPROT at address
     * |        |          |SYS_BA + 0x100
     * |        |          |00 = Clock source from external 12 MHz or 32 KHz crystal clock.
     * |        |          |10 = clock source from HCLK/2048 clock
     * |        |          |11 = clock source from internal 10KHz oscillator clock
     * |[3:2]   |ADC_S     |ADC clock source select.
     * |        |          |00 = Clock source from external 12 MHz or 32 KHz crystal clock.
     * |        |          |10 = clock source from HCLK
     * |        |          |11 = clock source from internal 22.1184 MHz oscillator clock
     * |[10:8]  |TMR0_S    |TIMER0 clock source select.
     * |        |          |000 = Clock source from external 12 MHz or 32 KHz crystal clock
     * |        |          |001 = Clock source from internal 10 KHz oscillator clock.
     * |        |          |010 = clock source from HCLK
     * |        |          |011 = clock source from external trigger
     * |        |          |111 = clock source from internal 22.1184 MHz oscillator clock
     * |[14:12] |TMR1_S    |TIMER1 clock source select.
     * |        |          |000 = Clock source from external 12 MHz or 32 KHz crystal clock
     * |        |          |001 = Clock source from internal 10 KHz oscillator clock.
     * |        |          |010 = clock source from HCLK
     * |        |          |011 = clock source from external trigger
     * |        |          |111 = clock source from internal 22.1184 MHz oscillator clock
     * |[25:24] |UART_S    |UART clock source select.
     * |        |          |00 = Clock source from external 12 MHz or 32 KHz crystal clock
     * |        |          |10 = clock source from internal 22.1184 MHz oscillator clock
     * |[29:28] |PWM01_S   |PWM0 and PWM1 clock source select.
     * |        |          |PWM0 and PWM1 uses the same Engine clock source, both of them with the same pre-scalar
     * |        |          |10 = clock source from HCLK
     * |        |          |others = Reserved
     * |[31:30] |PWM23_S   |PWM2 and PWM3 clock source select.
     * |        |          |PWM2 and PWM3 uses the same Engine clock source, both of them with the same pre-scalar
     * |        |          |10 = clock source from HCLK
     * |        |          |others = Reserved
     */
    __IO uint32_t  CLKSEL1;

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x18  Clock Divider Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLK_N    |HCLK clock divide number from HCLK clock source
     * |        |          |The HCLK clock frequency = (HCLK clock source frequency) / (HCLK_N + 1)
     * |[11:8]  |UART_N    |UART clock divide number from UART clock source
     * |        |          |The UART clock frequency = (UART clock source frequency ) / (UART_N + 1)
     * |[23:16] |ADC_N     |ADC clock divide number from ADC clock source
     * |        |          |The ADC clock frequency = (ADC clock source frequency ) / (ADC_N + 1)
     */
    __IO uint32_t  CLKDIV;

    /**
     * CLKSEL2
     * ===================================================================================================
     * Offset: 0x1C  Clock Source Select Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:2]   |FRQDIV_S  |Clock Divider Clock Source Select
     * |        |          |00 = Clock source from external 12 MHz or 32 KHz crystal clock
     * |        |          |10 = clock source from HCLK
     * |        |          |11 = clock source from internal 22.1184 MHz oscillator clock
     * |[5:4]   |PWM45_S   |PWM4 and PWM5 clock source select. - PWM4 and PWM5 used the same Engine clock source,
     * |        |          |both of them with the same pre-scalar
     * |        |          |10 = clock source from HCLK
     * |        |          |others = Reserved
     */
    __IO uint32_t  CLKSEL2;
    /**
     * Reserved
     */
    uint32_t  RESERVED0;

    /**
     * FRQDIV
     * ===================================================================================================
     * Offset: 0x24  Frequency Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FSEL      |Divider Output Frequency Selection Bits
     * |        |          |The formula of output frequency is
     * |        |          |Fout = Fin/2^(N+1),
     * |        |          |where Fin is the input clock frequency, Fout is the frequency of divider output clock, N is the 4-bit
     * |        |          |value of FSEL[3:0].
     * |[4]     |DIVIDER_EN|Frequency Divider Enable Bit
     * |        |          |0 = Disable Frequency Divider
     * |        |          |1 = Enable Frequency Divider
     */
    __IO uint32_t  FRQDIV;
} CLK_T;
/// @cond HIDDEN_SYMBOLS
/* CLK PWRCON Bit Field Definitions */
#define CLK_PWRCON_PD_32K_Pos                9                                       /*!< CLK PWRCON: PD_32K Position   */
#define CLK_PWRCON_PD_32K_Msk                (1ul << CLK_PWRCON_PD_32K_Pos)          /*!< CLK PWRCON: PD_32K Mask       */

#define CLK_PWRCON_PWR_DOWN_EN_Pos           7                                       /*!< CLK PWRCON: PWR_DOWN_EN Position  */
#define CLK_PWRCON_PWR_DOWN_EN_Msk           (1ul << CLK_PWRCON_PWR_DOWN_EN_Pos)     /*!< CLK PWRCON: PWR_DOWN_EN Mask      */

#define CLK_PWRCON_PD_WU_STS_Pos             6                                       /*!< CLK PWRCON: PD_WU_STS Position    */
#define CLK_PWRCON_PD_WU_STS_Msk             (1ul << CLK_PWRCON_PD_WU_STS_Pos)       /*!< CLK PWRCON: PD_WU_STS Mask        */

#define CLK_PWRCON_WINT_EN_Pos               5                                       /*!< CLK PWRCON: WINT_EN Position  */
#define CLK_PWRCON_WINT_EN_Msk               (1ul << CLK_PWRCON_WINT_EN_Pos)         /*!< CLK PWRCON: WINT_EN Mask      */

#define CLK_PWRCON_WU_DLY_Pos                4                                       /*!< CLK PWRCON: WU_DLY Position   */
#define CLK_PWRCON_WU_DLY_Msk                (1ul << CLK_PWRCON_WU_DLY_Pos)          /*!< CLK PWRCON: WU_DLY Mask       */

#define CLK_PWRCON_OSC10K_EN_Pos             3                                       /*!< CLK PWRCON: OSC10K_EN Position    */
#define CLK_PWRCON_OSC10K_EN_Msk             (1ul << CLK_PWRCON_OSC10K_EN_Pos)       /*!< CLK PWRCON: OSC10K_EN Mask        */
#define CLK_PWRCON_IRC10K_EN_Pos             3                                       /*!< CLK PWRCON: OSC10K_EN Position    */
#define CLK_PWRCON_IRC10K_EN_Msk             (1ul << CLK_PWRCON_OSC10K_EN_Pos)       /*!< CLK PWRCON: OSC10K_EN Mask        */
#define CLK_PWRCON_LIRC_EN_Pos               3                                       /*!< CLK PWRCON: OSC10K_EN Position    */
#define CLK_PWRCON_LIRC_EN_Msk               (1ul << CLK_PWRCON_LIRC_EN_Pos)         /*!< CLK PWRCON: OSC10K_EN Mask        */

#define CLK_PWRCON_OSC22M_EN_Pos             2                                       /*!< CLK PWRCON: OSC22M_EN Position    */
#define CLK_PWRCON_OSC22M_EN_Msk             (1ul << CLK_PWRCON_OSC22M_EN_Pos)       /*!< CLK PWRCON: OSC22M_EN Mask        */
#define CLK_PWRCON_IRC22M_EN_Pos             2                                       /*!< CLK PWRCON: OSC22M_EN Position    */
#define CLK_PWRCON_IRC22M_EN_Msk             (1ul << CLK_PWRCON_OSC22M_EN_Pos)       /*!< CLK PWRCON: OSC22M_EN Mask        */
#define CLK_PWRCON_HIRC_EN_Pos               2                                       /*!< CLK PWRCON: OSC22M_EN Position    */
#define CLK_PWRCON_HIRC_EN_Msk               (1ul << CLK_PWRCON_HIRC_EN_Pos)         /*!< CLK PWRCON: OSC22M_EN Mask        */

#define CLK_PWRCON_XTLCLK_EN_Pos             0                                       /*!< CLK PWRCON: XTLCLK_EN Position    */
#define CLK_PWRCON_XTLCLK_EN_Msk             (3ul << CLK_PWRCON_XTLCLK_EN_Pos)       /*!< CLK PWRCON: XTLCLK_EN Mask        */

/* CLK AHBCLK Bit Field Definitions */
#define CLK_AHBCLK_ISP_EN_Pos                2                                       /*!< CLK AHBCLK: ISP_EN Position   */
#define CLK_AHBCLK_ISP_EN_Msk                (1ul << CLK_AHBCLK_ISP_EN_Pos)          /*!< CLK AHBCLK: ISP_EN Mask       */

/* CLK APBCLK Bit Field Definitions */
#define CLK_APBCLK_CMP_EN_Pos                30                                      /*!< CLK APBCLK: CMP_EN Position   */
#define CLK_APBCLK_CMP_EN_Msk                (1ul << CLK_APBCLK_CMP_EN_Pos)          /*!< CLK APBCLK: CMP_EN Mask       */

#define CLK_APBCLK_ADC_EN_Pos                28                                      /*!< CLK APBCLK: ADC_EN Position   */
#define CLK_APBCLK_ADC_EN_Msk                (1ul << CLK_APBCLK_ADC_EN_Pos)          /*!< CLK APBCLK: ADC_EN Mask       */

#define CLK_APBCLK_PWM45_EN_Pos              22                                      /*!< CLK APBCLK: PWM45_EN Position */
#define CLK_APBCLK_PWM45_EN_Msk              (1ul << CLK_APBCLK_PWM45_EN_Pos)        /*!< CLK APBCLK: PWM45_EN Mask     */

#define CLK_APBCLK_PWM23_EN_Pos              21                                      /*!< CLK APBCLK: PWM23_EN Position */
#define CLK_APBCLK_PWM23_EN_Msk              (1ul << CLK_APBCLK_PWM23_EN_Pos)        /*!< CLK APBCLK: PWM23_EN Mask     */

#define CLK_APBCLK_PWM01_EN_Pos              20                                      /*!< CLK APBCLK: PWM01_EN Position */
#define CLK_APBCLK_PWM01_EN_Msk              (1ul << CLK_APBCLK_PWM01_EN_Pos)        /*!< CLK APBCLK: PWM01_EN Mask     */

#define CLK_APBCLK_UART_EN_Pos               16                                      /*!< CLK APBCLK: UART_EN Position  */
#define CLK_APBCLK_UART_EN_Msk               (1ul << CLK_APBCLK_UART_EN_Pos)         /*!< CLK APBCLK: UART_EN Mask      */

#define CLK_APBCLK_SPI_EN_Pos                12                                      /*!< CLK APBCLK: SPI_EN Position   */
#define CLK_APBCLK_SPI_EN_Msk                (1ul << CLK_APBCLK_SPI_EN_Pos)          /*!< CLK APBCLK: SPI_EN Mask       */

#define CLK_APBCLK_I2C_EN_Pos                8                                       /*!< CLK APBCLK: I2C_EN Position   */
#define CLK_APBCLK_I2C_EN_Msk                (1ul << CLK_APBCLK_I2C_EN_Pos)          /*!< CLK APBCLK: I2C_EN Mask       */

#define CLK_APBCLK_FDIV_EN_Pos               6                                       /*!< CLK APBCLK: FDIV_EN Position  */
#define CLK_APBCLK_FDIV_EN_Msk               (1ul << CLK_APBCLK_FDIV_EN_Pos)         /*!< CLK APBCLK: FDIV_EN Mask      */

#define CLK_APBCLK_TMR1_EN_Pos               3                                       /*!< CLK APBCLK: TMR1_EN Position  */
#define CLK_APBCLK_TMR1_EN_Msk               (1ul << CLK_APBCLK_TMR1_EN_Pos)         /*!< CLK APBCLK: TMR1_EN Mask      */

#define CLK_APBCLK_TMR0_EN_Pos               2                                       /*!< CLK APBCLK: TMR0_EN Position  */
#define CLK_APBCLK_TMR0_EN_Msk               (1ul << CLK_APBCLK_TMR0_EN_Pos)         /*!< CLK APBCLK: TMR0_EN Mask      */

#define CLK_APBCLK_WDT_EN_Pos                0                                       /*!< CLK APBCLK: WDT_EN Position   */
#define CLK_APBCLK_WDT_EN_Msk                (1ul << CLK_APBCLK_WDT_EN_Pos)          /*!< CLK APBCLK: WDT_EN Mask       */

/* CLK CLKSTATUS Bit Field Definitions */
#define CLK_CLKSTATUS_CLK_SW_FAIL_Pos        7                                       /*!< CLK CLKSTATUS: CLK_SW_FAIL Position   */
#define CLK_CLKSTATUS_CLK_SW_FAIL_Msk        (1ul << CLK_CLKSTATUS_CLK_SW_FAIL_Pos)  /*!< CLK CLKSTATUS: CLK_SW_FAIL Mask       */

#define CLK_CLKSTATUS_OSC22M_STB_Pos         4                                       /*!< CLK CLKSTATUS: OSC22M_STB Position    */
#define CLK_CLKSTATUS_OSC22M_STB_Msk         (1ul << CLK_CLKSTATUS_OSC22M_STB_Pos)   /*!< CLK CLKSTATUS: OSC22M_STB Mask        */
#define CLK_CLKSTATUS_IRC22M_STB_Pos         4                                       /*!< CLK CLKSTATUS: OSC22M_STB Position    */
#define CLK_CLKSTATUS_IRC22M_STB_Msk         (1ul << CLK_CLKSTATUS_OSC22M_STB_Pos)   /*!< CLK CLKSTATUS: OSC22M_STB Mask        */
#define CLK_CLKSTATUS_HIRC_STB_Pos           4                                       /*!< CLK CLKSTATUS: OSC22M_STB Position    */
#define CLK_CLKSTATUS_HIRC_STB_Msk           (1ul << CLK_CLKSTATUS_HIRC_STB_Pos)     /*!< CLK CLKSTATUS: OSC22M_STB Mask        */

#define CLK_CLKSTATUS_OSC10K_STB_Pos         3                                       /*!< CLK CLKSTATUS: OSC10K_STB Position    */
#define CLK_CLKSTATUS_OSC10K_STB_Msk         (1ul << CLK_CLKSTATUS_OSC10K_STB_Pos)   /*!< CLK CLKSTATUS: OSC10K_STB Mask        */
#define CLK_CLKSTATUS_IRC10K_STB_Pos         3                                       /*!< CLK CLKSTATUS: OSC10K_STB Position    */
#define CLK_CLKSTATUS_IRC10K_STB_Msk         (1ul << CLK_CLKSTATUS_OSC10K_STB_Pos)   /*!< CLK CLKSTATUS: OSC10K_STB Mask        */
#define CLK_CLKSTATUS_LIRC_STB_Pos           3                                       /*!< CLK CLKSTATUS: OSC10K_STB Position    */
#define CLK_CLKSTATUS_LIRC_STB_Msk           (1ul << CLK_CLKSTATUS_LIRC_STB_Pos)     /*!< CLK CLKSTATUS: OSC10K_STB Mask        */

#define CLK_CLKSTATUS_XTL_STB_Pos            0                                       /*!< CLK CLKSTATUS: XTL_STB Position   */
#define CLK_CLKSTATUS_XTL_STB_Msk            (1ul << CLK_CLKSTATUS_XTL_STB_Pos)      /*!< CLK CLKSTATUS: XTL_STB Mask       */
#define CLK_CLKSTATUS_HXT_STB_Pos            0                                       /*!< CLK CLKSTATUS: XTL_STB Position   */
#define CLK_CLKSTATUS_HXT_STB_Msk            (1ul << CLK_CLKSTATUS_HXT_STB_Pos)      /*!< CLK CLKSTATUS: XTL_STB Mask       */
#define CLK_CLKSTATUS_LXT_STB_Pos            0                                       /*!< CLK CLKSTATUS: XTL_STB Position   */
#define CLK_CLKSTATUS_LXT_STB_Msk            (1ul << CLK_CLKSTATUS_LXT_STB_Pos)      /*!< CLK CLKSTATUS: XTL_STB Mask       */

/* CLK CLKSEL0 Bit Field Definitions */
#define CLK_CLKSEL0_STCLK_S_Pos              3                                       /*!< CLK CLKSEL0: STCLK_S Position */
#define CLK_CLKSEL0_STCLK_S_Msk              (7ul << CLK_CLKSEL0_STCLK_S_Pos)        /*!< CLK CLKSEL0: STCLK_S Mask     */

#define CLK_CLKSEL0_HCLK_S_Pos               0                                       /*!< CLK CLKSEL0: HCLK_S Position  */
#define CLK_CLKSEL0_HCLK_S_Msk               (7ul << CLK_CLKSEL0_HCLK_S_Pos)         /*!< CLK CLKSEL0: HCLK_S Mask      */

/* CLK CLKSEL1 Bit Field Definitions */
#define CLK_CLKSEL1_PWM23_S_Pos              30                                      /*!< CLK CLKSEL1: PWM23_S Position */
#define CLK_CLKSEL1_PWM23_S_Msk              (3ul << CLK_CLKSEL1_PWM23_S_Pos)        /*!< CLK CLKSEL1: PWM23_S Mask     */

#define CLK_CLKSEL1_PWM01_S_Pos              28                                      /*!< CLK CLKSEL1: PWM01_S Position */
#define CLK_CLKSEL1_PWM01_S_Msk              (3ul << CLK_CLKSEL1_PWM01_S_Pos)        /*!< CLK CLKSEL1: PWM01_S Mask     */

#define CLK_CLKSEL1_UART_S_Pos               24                                      /*!< CLK CLKSEL1: UART_S Position  */
#define CLK_CLKSEL1_UART_S_Msk               (3ul << CLK_CLKSEL1_UART_S_Pos)         /*!< CLK CLKSEL1: UART_S Mask      */

#define CLK_CLKSEL1_TMR1_S_Pos               12                                      /*!< CLK CLKSEL1: TMR1_S Position  */
#define CLK_CLKSEL1_TMR1_S_Msk               (7ul << CLK_CLKSEL1_TMR1_S_Pos)         /*!< CLK CLKSEL1: TMR1_S Mask      */

#define CLK_CLKSEL1_TMR0_S_Pos               8                                       /*!< CLK CLKSEL1: TMR0_S Position  */
#define CLK_CLKSEL1_TMR0_S_Msk               (7ul << CLK_CLKSEL1_TMR0_S_Pos)         /*!< CLK CLKSEL1: TMR0_S Mask      */

#define CLK_CLKSEL1_SPI_S_Pos                4                                       /*!< CLK CLKSEL1: SPI_S Position   */
#define CLK_CLKSEL1_SPI_S_Msk                (1ul << CLK_CLKSEL1_SPI_S_Pos)          /*!< CLK CLKSEL1: SPI_S Mask       */
#define CLK_CLKSEL1_ADC_S_Pos                2                                       /*!< CLK CLKSEL1: ADC_S Position   */
#define CLK_CLKSEL1_ADC_S_Msk                (3ul << CLK_CLKSEL1_ADC_S_Pos)          /*!< CLK CLKSEL1: ADC_S Mask       */

#define CLK_CLKSEL1_WDT_S_Pos                0                                       /*!< CLK CLKSEL1: WDT_S Position   */
#define CLK_CLKSEL1_WDT_S_Msk                (3ul << CLK_CLKSEL1_WDT_S_Pos)          /*!< CLK CLKSEL1: WDT_S Mask       */

/* CLK CLKSEL2 Bit Field Definitions */
#define CLK_CLKSEL2_PWM45_S_Pos              4                                       /*!< CLK CLKSEL2: PWM45_S Position */
#define CLK_CLKSEL2_PWM45_S_Msk              (3ul << CLK_CLKSEL2_PWM45_S_Pos)        /*!< CLK CLKSEL2: PWM45_S Mask     */

#define CLK_CLKSEL2_FRQDIV_S_Pos             2                                       /*!< CLK CLKSEL2: FRQDIV_S Position    */
#define CLK_CLKSEL2_FRQDIV_S_Msk             (3ul << CLK_CLKSEL2_FRQDIV_S_Pos)       /*!< CLK CLKSEL2: FRQDIV_S Mask        */

/* CLK CLKDIV Bit Field Definitions */
#define CLK_CLKDIV_ADC_N_Pos                 16                                      /*!< CLK CLKDIV: ADC_N Position        */
#define CLK_CLKDIV_ADC_N_Msk                 (0xFFul << CLK_CLKDIV_ADC_N_Pos)        /*!< CLK CLKDIV: ADC_N Mask            */

#define CLK_CLKDIV_UART_N_Pos                8                                       /*!< CLK CLKDIV: UART_N Position       */
#define CLK_CLKDIV_UART_N_Msk                (0xFul << CLK_CLKDIV_UART_N_Pos)        /*!< CLK CLKDIV: UART_N Mask           */

#define CLK_CLKDIV_HCLK_N_Pos                0                                       /*!< CLK CLKDIV: HCLK_N Position       */
#define CLK_CLKDIV_HCLK_N_Msk                (0xFul << CLK_CLKDIV_HCLK_N_Pos)        /*!< CLK CLKDIV: HCLK_N Mask           */

/* CLK FRQDIV Bit Field Definitions */
#define CLK_FRQDIV_DIVIDER1_Pos              5                                       /*!< CLK FRQDIV: DIVIDER1 Position     */
#define CLK_FRQDIV_DIVIDER1_Msk              (1ul << CLK_FRQDIV_DIVIDER_EN_Pos)      /*!< CLK FRQDIV: DIVIDER1 Mask         */

#define CLK_FRQDIV_DIVIDER_EN_Pos            4                                       /*!< CLK FRQDIV: DIVIDER_EN Position   */
#define CLK_FRQDIV_DIVIDER_EN_Msk            (1ul << CLK_FRQDIV_DIVIDER_EN_Pos)      /*!< CLK FRQDIV: DIVIDER_EN Mask       */

#define CLK_FRQDIV_FSEL_Pos                  0                                       /*!< CLK FRQDIV: FSEL Position         */
#define CLK_FRQDIV_FSEL_Msk                  (0xFul << CLK_FRQDIV_FSEL_Pos)          /*!< CLK FRQDIV: FSEL Mask             */
/// @endcond /* HIDDEN_SYMBOLS */
/*@}*/ /* end of group CLK */


/*---------------------- Analog to Digital Converter -------------------------*/

/** @addtogroup ADC Analog to Digital Converter(ADC)
  Memory Mapped Structure for ADC Controller
  @{
 */
typedef struct
{
    __IO uint32_t  ADDR;           /*!< Offset: 0x0000   A/D Data Register                    */
    uint32_t  RESERVED0[7];   /*!< Offset: 0x0004 ~ 0x001C   Reserved                    */
    __IO uint32_t  ADCR;           /*!< Offset: 0x0020   A/D Control Register                 */
    __IO uint32_t  ADCHER;         /*!< Offset: 0x0024   A/D Channel Enable Register          */
    __IO uint32_t  ADCMPR[2];      /*!< Offset: 0x0028, 0x002C   A/D Compare Register 0 & 1   */
    __IO uint32_t  ADSR;           /*!< Offset: 0x0030   A/D Status Register                  */
    __IO uint32_t  ADTDCR;         /*!< Offset: 0x0044   A/D Trigger Delay Control Register   */
    __IO uint32_t  ADSAMP;         /*!< Offset: 0x0048   ADC Sampling Time Counter Register   */
} ADC_T;
/// @cond HIDDEN_SYMBOLS
/* ADDR Bit Field Definitions */
#define ADC_ADDR_VALID_Pos      17                                      /*!< ADC ADDR: VALID Position       */
#define ADC_ADDR_VALID_Msk      (1ul << ADC_ADDR_VALID_Pos)             /*!< ADC ADDR: VALID Mask           */

#define ADC_ADDR_OVERRUN_Pos    16                                      /*!< ADC ADDR: OVERRUN Position     */
#define ADC_ADDR_OVERRUN_Msk    (1ul << ADC_ADDR_OVERRUN_Pos)           /*!< ADC ADDR: OVERRUN Mask         */

#define ADC_ADDR_RSLT_Pos       0                                       /*!< ADC ADDR: RSLT Position        */
#define ADC_ADDR_RSLT_Msk       (0x3FFul << ADC_ADDR_RSLT_Pos)          /*!< ADC ADDR: RSLT Mask            */

/* ADCR Bit Field Definitions */
#define ADC_ADCR_ADST_Pos       11                                      /*!< ADC ADCR: ADST Position        */
#define ADC_ADCR_ADST_Msk       (1ul << ADC_ADCR_ADST_Pos)              /*!< ADC ADCR: ADST Mask            */

#define ADC_ADCR_TRGEN_Pos      8                                       /*!< ADC ADCR: TRGEN Position       */
#define ADC_ADCR_TRGEN_Msk      (1ul << ADC_ADCR_TRGEN_Pos)             /*!< ADC ADCR: TRGEN Mask           */

#define ADC_ADCR_TRGCOND_Pos    6                                       /*!< ADC ADCR: TRGCOND Position     */
#define ADC_ADCR_TRGCOND_Msk    (1ul << ADC_ADCR_TRGCOND_Pos)           /*!< ADC ADCR: TRGCOND Mask         */

#define ADC_ADCR_TRGS_Pos       4                                       /*!< ADC ADCR: TRGS Position        */
#define ADC_ADCR_TRGS_Msk       (3ul << ADC_ADCR_TRGS_Pos)              /*!< ADC ADCR: TRGS Mask            */

#define ADC_ADCR_ADIE_Pos       1                                       /*!< ADC ADCR: ADIE Position        */
#define ADC_ADCR_ADIE_Msk       (1ul << ADC_ADCR_ADIE_Pos)              /*!< ADC ADCR: ADIE Mask            */

#define ADC_ADCR_ADEN_Pos       0                                       /*!< ADC ADCR: ADEN Position        */
#define ADC_ADCR_ADEN_Msk       (1ul << ADC_ADCR_ADEN_Pos)              /*!< ADC ADCR: ADEN Mask            */

/* ADCHER Bit Field Definitions */
#define ADC_ADCHER_PRESEL_Pos   8                                       /*!< ADC ADCHER: PRESEL Position    */
#define ADC_ADCHER_PRESEL_Msk   (1ul << ADC_ADCHER_PRESEL_Pos)          /*!< ADC ADCHER: PRESEL Mask        */

#define ADC_ADCHER_CHEN_Pos     0                                       /*!< ADC ADCHER: CHEN Position      */
#define ADC_ADCHER_CHEN_Msk     (0xFFul << ADC_ADCHER_CHEN_Pos)         /*!< ADC ADCHER: CHEN Mask          */

/* ADCMPR Bit Field Definitions */
#define ADC_ADCMPR_CMPD_Pos        16                                    /*!< ADC ADCMPR: CMPD Position     */
#define ADC_ADCMPR_CMPD_Msk        (0x3FFul << ADC_ADCMPR_CMPD_Pos)      /*!< ADC ADCMPR: CMPD Mask         */

#define ADC_ADCMPR_CMPMATCNT_Pos   8                                     /*!< ADC ADCMPR: CMPMATCNT Position    */
#define ADC_ADCMPR_CMPMATCNT_Msk   (0xFul << ADC_ADCMPR_CMPMATCNT_Pos)   /*!< ADC ADCMPR: CMPMATCNT Mask        */

#define ADC_ADCMPR_CMPCH_Pos       3                                     /*!< ADC ADCMPR: CMPCH Position    */
#define ADC_ADCMPR_CMPCH_Msk       (7ul << ADC_ADCMPR_CMPCH_Pos)         /*!< ADC ADCMPR: CMPCH Mask        */

#define ADC_ADCMPR_CMPCOND_Pos     2                                     /*!< ADC ADCMPR: CMPCOND Position  */
#define ADC_ADCMPR_CMPCOND_Msk     (1ul << ADC_ADCMPR_CMPCOND_Pos)       /*!< ADC ADCMPR: CMPCOND Mask      */

#define ADC_ADCMPR_CMPIE_Pos       1                                     /*!< ADC ADCMPR: CMPIE Position    */
#define ADC_ADCMPR_CMPIE_Msk       (1ul << ADC_ADCMPR_CMPIE_Pos)         /*!< ADC ADCMPR: CMPIE Mask        */

#define ADC_ADCMPR_CMPEN_Pos       0                                     /*!< ADC ADCMPR: CMPEN Position    */
#define ADC_ADCMPR_CMPEN_Msk       (1ul << ADC_ADCMPR_CMPEN_Pos)         /*!< ADC ADCMPR: CMPEN Mask        */

/* ADSR Bit Field Definitions */
#define ADC_ADSR_OVERRUN_Pos       16                                    /*!< ADC ADSR: OVERRUN Position    */
#define ADC_ADSR_OVERRUN_Msk       (0xFFul << ADC_ADSR_OVERRUN_Pos)      /*!< ADC ADSR: OVERRUN Mask        */

#define ADC_ADSR_VALID_Pos         8                                     /*!< ADC ADSR: VALID Position      */
#define ADC_ADSR_VALID_Msk         (0xFFul << ADC_ADSR_VALID_Pos)        /*!< ADC ADSR: VALID Mask          */

#define ADC_ADSR_CHANNEL_Pos       4                                     /*!< ADC ADSR: CHANNEL Position    */
#define ADC_ADSR_CHANNEL_Msk       (7ul << ADC_ADSR_CHANNEL_Pos)         /*!< ADC ADSR: CHANNEL Mask        */

#define ADC_ADSR_BUSY_Pos          3                                     /*!< ADC ADSR: BUSY Position       */
#define ADC_ADSR_BUSY_Msk          (1ul << ADC_ADSR_BUSY_Pos)            /*!< ADC ADSR: BUSY Mask           */

#define ADC_ADSR_CMPF1_Pos         2                                     /*!< ADC ADSR: CMPF1 Position      */
#define ADC_ADSR_CMPF1_Msk         (1ul << ADC_ADSR_CMPF1_Pos)           /*!< ADC ADSR: CMPF1 Mask          */

#define ADC_ADSR_CMPF0_Pos         1                                     /*!< ADC ADSR: CMPF0 Position      */
#define ADC_ADSR_CMPF0_Msk         (1ul << ADC_ADSR_CMPF0_Pos)           /*!< ADC ADSR: CMPF0 Mask          */

#define ADC_ADSR_ADF_Pos           0                                     /*!< ADC ADSR: ADF Position        */
#define ADC_ADSR_ADF_Msk           (1ul << ADC_ADSR_ADF_Pos)             /*!< ADC ADSR: ADF Mask            */

/* ADTDCR Bit Field Definitions */
#define ADC_ADTDCR_PTDT_Pos        0                                     /*!< ADC ADTDCR: PTDT Position     */
#define ADC_ADTDCR_PTDT_Msk        (0xFFul << ADC_ADTDCR_PTDT_Pos)       /*!< ADC ADTDCR: PTDT Mask         */

/* ADSAMP Bit Field Definitions */
#define ADC_ADSAMP_SAMPCNT_Pos     0                                     /*!< ADC ADSAMP: SAMPCNT Position */
#define ADC_ADSAMP_SAMPCNT_Msk     (0xFul << ADC_ADTDCR_PTDT_Pos)        /*!< ADC ADSAMP: SAMPCNT Mask     */
/// @endcond /* HIDDEN_SYMBOLS */
/*@}*/ /* end of group ADC */


/*-------------------------- Flash Memory Controller -------------------------*/

/** @addtogroup FMC Flash Memory Controller(FMC)
  Memory Mapped Structure for FMC Controller
  @{
 */
typedef struct
{
    /**
     * ISPCON
     * ===================================================================================================
     * Offset: 0x00  ISP Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPEN     |ISP Enable
     * |        |          |ISP function enable bit. Set this bit to enable ISP function.
     * |        |          |1 = Enable ISP function
     * |        |          |0 = Disable ISP function
     * |[1]     |BS        |Boot Select
     * |        |          |Set/clear this bit to select next booting from LDROM/APROM, respectively.
     * |        |          |This bit also functions as MCU booting status flag, which can be used to
     * |        |          |check where MCU booted from. This bit is initiated with the inverse value
     * |        |          |of CBS in Config0 after power-on reset; It keeps the same value at other reset.
     * |        |          |1 = Boot from LDROM
     * |        |          |0 = Boot from APROM
     * |[4]     |CFGUEN    |Enable Config-bits Update by ISP
     * |        |          |1 = Enable ISP can update config-bits
     * |        |          |0 = Disable ISP can update config-bits.
     * |[5]     |LDUEN     |LDROM Update Enable
     * |        |          |LDROM update enable bit.
     * |        |          |1 = LDROM can be updated when the MCU runs in APROM.
     * |        |          |0 = LDROM cannot be updated
     * |[6]     |ISPFF     |ISP Fail Flag
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself.
     * |        |          |(2) LDROM writes to itself.
     * |        |          |(3) CONFIG is erased/programmed when the MCU is running in APROM.
     * |        |          |(4) Destination address is illegal, such as over an available range.
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
     */
    __IO uint32_t ISPCON;

    /**
     * ISPADR
     * ===================================================================================================
     * Offset: 0x04  ISP Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPADR    |ISP Address
     * |        |          |NuMicro NUC029FAETM series supports word program only. ISPADR[1:0] must be kept
     * |        |          |00b for ISP operation.
     */
    __IO uint32_t ISPADR;

    /**
     * ISPDAT
     * ===================================================================================================
     * Offset: 0x08  ISP Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT    |ISP Data
     * |        |          |Write data to this register before ISP program operation
     * |        |          |Read data from this register after ISP read operation
     */
    __IO uint32_t ISPDAT;

    /**
     * ISPCMD
     * ===================================================================================================
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
     * |        |          |Read CID      ,    0,    0, 1011
     * |        |          |Read DID      ,    0,    0, 1100
     * |        |          |Read UID      ,    0,    0, 0100
     */
    __IO uint32_t ISPCMD;

    /**
     * ISPTRG
     * ===================================================================================================
     * Offset: 0x10  ISP Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP start trigger
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP
     * |        |          |operation is finish.
     * |        |          |1 = ISP is on going
     * |        |          |0 = ISP operation is finished.
     */
    __IO uint32_t ISPTRG;

    /**
     * DFBADR
     * ===================================================================================================
     * Offset: 0x14  Data Flash Base Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DFBA      |Data Flash Base Address
     * |        |          |This register indicates data flash start address. It is a read only register.
     * |        |          |
     * |        |          |It is a read only register.
     * |        |          |
     * |        |          |The data flash start address is defined by user. Since on chip flash erase
     * |        |          |unit is 512 bytes, it is mandatory to keep bit 8-0 as "0".
     */
    __I  uint32_t DFBADR;

} FMC_T;
/// @cond HIDDEN_SYMBOLS

/* FMC ISPCON Bit Field Definitions */
#define FMC_ISPCON_ISPFF_Pos                    6                                       /*!< FMC ISPCON: ISPFF Position */
#define FMC_ISPCON_ISPFF_Msk                    (1ul << FMC_ISPCON_ISPFF_Pos)           /*!< FMC ISPCON: ISPFF Mask     */

#define FMC_ISPCON_LDUEN_Pos                    5                                       /*!< FMC ISPCON: LDUEN Position */
#define FMC_ISPCON_LDUEN_Msk                    (1ul << FMC_ISPCON_LDUEN_Pos)           /*!< FMC ISPCON: LDUEN Mask     */

#define FMC_ISPCON_CFGUEN_Pos                   4                                       /*!< FMC ISPCON: CFGUEN Position    */
#define FMC_ISPCON_CFGUEN_Msk                   (1ul << FMC_ISPCON_CFGUEN_Pos)          /*!< FMC ISPCON: CFGUEN Mask        */

#define FMC_ISPCON_APUEN_Pos                    3                                       /*!< FMC ISPCON: APUEN Position */
#define FMC_ISPCON_APUEN_Msk                    (1ul << FMC_ISPCON_APUEN_Pos)           /*!< FMC ISPCON: APUEN Mask     */

#define FMC_ISPCON_BS_Pos                       1                                       /*!< FMC ISPCON: BS Position    */
#define FMC_ISPCON_BS_Msk                       (1ul << FMC_ISPCON_BS_Pos)              /*!< FMC ISPCON: BS Mask        */

#define FMC_ISPCON_ISPEN_Pos                    0                                       /*!< FMC ISPCON: ISPEN Position */
#define FMC_ISPCON_ISPEN_Msk                    (1ul << FMC_ISPCON_ISPEN_Pos)           /*!< FMC ISPCON: ISPEN Mask     */

/* FMC ISPCMD Bit Field Definitions */
#define FMC_ISPCMD_FOEN_Pos                     5                                       /*!< FMC ISPCMD: FOEN Position  */
#define FMC_ISPCMD_FOEN_Msk                     (1ul << FMC_ISPCMD_FOEN_Pos)            /*!< FMC ISPCMD: FOEN Mask      */

#define FMC_ISPCMD_FCEN_Pos                     4                                       /*!< FMC ISPCMD: FCEN Position  */
#define FMC_ISPCMD_FCEN_Msk                     (1ul << FMC_ISPCMD_FCEN_Pos)            /*!< FMC ISPCMD: FCEN Mask      */

#define FMC_ISPCMD_FCTRL_Pos                    0                                       /*!< FMC ISPCMD: FCTRL Position */
#define FMC_ISPCMD_FCTRL_Msk                    (0xFul << FMC_ISPCMD_FCTRL_Pos)         /*!< FMC ISPCMD: FCTRL Mask     */

/* FMC ISPTRG Bit Field Definitions */
#define FMC_ISPTRG_ISPGO_Pos                    0                                       /*!< FMC ISPTRG: ISPGO Position */
#define FMC_ISPTRG_ISPGO_Msk                    (1ul << FMC_ISPTRG_ISPGO_Pos)           /*!< FMC ISPTRG: ISPGO Mask     */
/// @endcond /* HIDDEN_SYMBOLS */

/*@}*/ /* end of group FMC */

/*---------------------- General Purpose Input/Output Controller -------------------------*/
/** @addtogroup GPIO General Purpose Input/Output Controller(GPIO)
  Memory Mapped Structure for GPIO Controller
  @{
 */

typedef struct
{
    __IO uint32_t  PMD;                        /*!< Offset: 0x0000   GPIO Port Bit Mode Control                         */
    __IO uint32_t  OFFD;                       /*!< Offset: 0x0004   GPIO Port Bit Off Digital Enable                   */
    __IO uint32_t  DOUT;                       /*!< Offset: 0x0008   GPIO Port Data Output                              */
    __IO uint32_t  DMASK;                      /*!< Offset: 0x000C   GPIO Port Data Output Write Mask                   */
    __I  uint32_t  PIN;                        /*!< Offset: 0x0010   GPIO Port Pin Value                                */
    __IO uint32_t  DBEN;                       /*!< Offset: 0x0014   GPIO Port De-bounce Enable                         */
    __IO uint32_t  IMD;                        /*!< Offset: 0x0018   GPIO Port Interrupt Mode Select                    */
    __IO uint32_t  IEN;                        /*!< Offset: 0x001C   GPIO Port Interrupt Enable                         */
    __IO uint32_t  ISRC;                       /*!< Offset: 0x0020   GPIO Port Interrupt Source Flag                    */
} GPIO_T;


typedef struct
{
    __IO uint32_t  DBNCECON;             /*!< Offset: 0x0000   GPIO De-bounce Cycle Control Register              */
} GPIO_DBNCECON_T;
/// @cond HIDDEN_SYMBOLS
/* GPIO PMD Bit Field Definitions */
#define GPIO_PMD_PMD7_Pos           14                                          /*!< GPIO PMD: PMD7 Position    */
#define GPIO_PMD_PMD7_Msk           (0x3ul << GPIO_PMD_PMD7_Pos)                /*!< GPIO PMD: PMD7 Mask        */

#define GPIO_PMD_PMD6_Pos           12                                          /*!< GPIO PMD: PMD6 Position    */
#define GPIO_PMD_PMD6_Msk           (0x3ul << GPIO_PMD_PMD6_Pos)                /*!< GPIO PMD: PMD6 Mask        */

#define GPIO_PMD_PMD5_Pos           10                                          /*!< GPIO PMD: PMD5 Position    */
#define GPIO_PMD_PMD5_Msk           (0x3ul << GPIO_PMD_PMD5_Pos)                /*!< GPIO PMD: PMD5 Mask        */

#define GPIO_PMD_PMD4_Pos           8                                           /*!< GPIO PMD: PMD4 Position    */
#define GPIO_PMD_PMD4_Msk           (0x3ul << GPIO_PMD_PMD4_Pos)                /*!< GPIO PMD: PMD4 Mask        */

#define GPIO_PMD_PMD3_Pos           6                                           /*!< GPIO PMD: PMD3 Position    */
#define GPIO_PMD_PMD3_Msk           (0x3ul << GPIO_PMD_PMD3_Pos)                /*!< GPIO PMD: PMD3 Mask        */

#define GPIO_PMD_PMD2_Pos           4                                           /*!< GPIO PMD: PMD2 Position    */
#define GPIO_PMD_PMD2_Msk           (0x3ul << GPIO_PMD_PMD2_Pos)                /*!< GPIO PMD: PMD2 Mask        */

#define GPIO_PMD_PMD1_Pos           2                                           /*!< GPIO PMD: PMD1 Position    */
#define GPIO_PMD_PMD1_Msk           (0x3ul << GPIO_PMD_PMD1_Pos)                /*!< GPIO PMD: PMD1 Mask        */

#define GPIO_PMD_PMD0_Pos           0                                           /*!< GPIO PMD: PMD0 Position    */
#define GPIO_PMD_PMD0_Msk           (0x3ul << GPIO_PMD_PMD0_Pos)                /*!< GPIO PMD: PMD0 Mask        */

/* GPIO OFFD Bit Field Definitions */
#define GPIO_OFFD_OFFD_Pos          16                                          /*!< GPIO OFFD: OFFD Position   */
#define GPIO_OFFD_OFFD_Msk          (0xFFul << GPIO_OFFD_OFFD_Pos)              /*!< GPIO OFFD: OFFD Mask       */

/* GPIO DOUT Bit Field Definitions */
#define GPIO_DOUT_DOUT_Pos          0                                           /*!< GPIO DOUT: DOUT Position   */
#define GPIO_DOUT_DOUT_Msk          (0xFFul << GPIO_DOUT_DOUT_Pos)              /*!< GPIO DOUT: DOUT Mask       */

/* GPIO DMASK Bit Field Definitions */
#define GPIO_DMASK_DMASK_Pos        0                                           /*!< GPIO DMASK: DMASK Position */
#define GPIO_DMASK_DMASK_Msk        (0xFFul << GPIO_DMASK_DMASK_Pos)            /*!< GPIO DMASK: DMASK Mask     */

/* GPIO PIN Bit Field Definitions */
#define GPIO_PIN_PIN_Pos            0                                           /*!< GPIO PIN: PIN Position     */
#define GPIO_PIN_PIN_Msk            (0xFFul << GPIO_PIN_PIN_Pos)                /*!< GPIO PIN: PIN Mask         */

/* GPIO DBEN Bit Field Definitions */
#define GPIO_DBEN_DBEN_Pos          0                                           /*!< GPIO DBEN: DBEN Position   */
#define GPIO_DBEN_DBEN_Msk          (0xFFul << GPIO_DBEN_DBEN_Pos)              /*!< GPIO DBEN: DBEN Mask       */

/* GPIO IMD Bit Field Definitions */
#define GPIO_IMD_IMD_Pos            0                                           /*!< GPIO IMD: IMD Position     */
#define GPIO_IMD_IMD_Msk            (0xFFul << GPIO_IMD_IMD_Pos)                /*!< GPIO IMD: IMD Mask         */

/* GPIO IEN Bit Field Definitions */
#define GPIO_IEN_IR_EN_Pos          16                                          /*!< GPIO IEN: IR_EN Position   */
#define GPIO_IEN_IR_EN_Msk          (0xFFul << GPIO_IEN_IR_EN_Pos)              /*!< GPIO IEN: IR_EN Mask       */

#define GPIO_IEN_IF_EN_Pos          0                                           /*!< GPIO IEN: IF_EN Position   */
#define GPIO_IEN_IF_EN_Msk          (0xFFul << GPIO_IEN_IF_EN_Pos)              /*!< GPIO IEN: IF_EN Mask       */

/* GPIO ISRC Bit Field Definitions */
#define GPIO_ISRC_ISRC_Pos          0                                           /*!< GPIO ISRC: ISRC Position   */
#define GPIO_ISRC_ISRC_Msk          (0xFFul << GPIO_ISRC_ISRC_Pos)              /*!< GPIO ISRC: ISRC Mask       */

/* GPIO DBNCECON Bit Field Definitions */
#define GPIO_DBNCECON_ICLK_ON_Pos   5                                           /*!< GPIO DBNCECON: ICLK_ON  Position   */
#define GPIO_DBNCECON_ICLK_ON_Msk   (1ul << GPIO_DBNCECON_ICLK_ON_Pos)          /*!< GPIO DBNCECON: ICLK_ON  Mask       */

#define GPIO_DBNCECON_DBCLKSRC_Pos  4                                           /*!< GPIO DBNCECON: DBCLKSRC Position   */
#define GPIO_DBNCECON_DBCLKSRC_Msk  (1ul << GPIO_DBNCECON_DBCLKSRC_Pos)         /*!< GPIO DBNCECON: DBCLKSRC Mask       */

#define GPIO_DBNCECON_DBCLKSEL_Pos  0                                           /*!< GPIO DBNCECON: DBCLKSEL Position   */
#define GPIO_DBNCECON_DBCLKSEL_Msk  (0xFul << GPIO_DBNCECON_DBCLKSEL_Pos)       /*!< GPIO DBNCECON: DBCLKSEL Mask       */
/// @endcond /* HIDDEN_SYMBOLS */

typedef struct
{
    __IO uint32_t  GP_BIT0;      /*!< Offset: 0x0000   Px.0 Data Output Value                         */
    __IO uint32_t  GP_BIT1;      /*!< Offset: 0x0004   Px.1 Data Output Value                         */
    __IO uint32_t  GP_BIT2;      /*!< Offset: 0x0008   Px.2 Data Output Value                         */
    __IO uint32_t  GP_BIT3;      /*!< Offset: 0x000C   Px.3 Data Output Value                         */
    __IO uint32_t  GP_BIT4;      /*!< Offset: 0x0010   Px.4 Data Output Value                         */
    __IO uint32_t  GP_BIT5;      /*!< Offset: 0x0014   Px.5 Data Output Value                         */
    __IO uint32_t  GP_BIT6;      /*!< Offset: 0x0018   Px.6 Data Output Value                         */
    __IO uint32_t  GP_BIT7;      /*!< Offset: 0x001C   Px.7 Data Output Value                         */
} GPIOBIT_T;

/*@}*/ /* end of group GPIO */


/*---------------------- Inter-IC Bus Controller -------------------------*/

/** @addtogroup I2C Inter-IC Bus Controller(I2C)
  Memory Mapped Structure for I2C Controller
  @{
 */

typedef struct
{
    __IO uint32_t  I2CON;            /*!< Offset: 0x0000  I2C Control Register                     */
    __IO uint32_t  I2CADDR0;         /*!< Offset: 0x0004  I2C Slave Address Register 0             */
    __IO uint32_t  I2CDAT;           /*!< Offset: 0x0008  I2C Data Register                        */
    __IO uint32_t  I2CSTATUS;        /*!< Offset: 0x000C  I2C Status Register                      */
    __IO uint32_t  I2CLK;            /*!< Offset: 0x0010  I2C Clock Divided Register               */
    __IO uint32_t  I2CTOC;           /*!< Offset: 0x0014  I2C Time-Out Counter Register            */
    __IO uint32_t  I2CADDR1;         /*!< Offset: 0x0018  I2C Slave Address Register 1             */
    __IO uint32_t  I2CADDR2;         /*!< Offset: 0x001C  I2C Slave Address Register 2             */
    __IO uint32_t  I2CADDR3;         /*!< Offset: 0x0020  I2C Slave Address Register 3             */
    __IO uint32_t  I2CADM0;          /*!< Offset: 0x0024  I2C Slave Address Mask Register 0        */
    __IO uint32_t  I2CADM1;          /*!< Offset: 0x0028  I2C Slave Address Mask Register 1        */
    __IO uint32_t  I2CADM2;          /*!< Offset: 0x002C  I2C Slave Address Mask Register 2        */
    __IO uint32_t  I2CADM3;          /*!< Offset: 0x0030  I2C Slave Address Mask Register 3        */
    uint32_t  RESERVED0;        /*!< Offset: 0x0034  Reserved                                 */
    uint32_t  RESERVED1;        /*!< Offset: 0x0038  Reserved                                 */
    __IO uint32_t  I2CON2;           /*!< Offset: 0x003C  I2C Control Register 2                   */
    __IO uint32_t  I2CSTATUS2;       /*!< Offset: 0x0040  I2C Status Register 2                    */
} I2C_T;
/// @cond HIDDEN_SYMBOLS

/* I2C I2CON Bit Field Definitions */
#define I2C_I2CON_EI_Pos                        7                                       /*!< I2C I2CON: EI Position             */
#define I2C_I2CON_EI_Msk                        (1ul << I2C_I2CON_EI_Pos)               /*!< I2C I2CON: EI Mask                 */

#define I2C_I2CON_ENSI_Pos                      6                                       /*!< I2C I2CON: ENSI Position           */
#define I2C_I2CON_ENSI_Msk                      (1ul << I2C_I2CON_ENSI_Pos)             /*!< I2C I2CON: ENSI Mask               */

#define I2C_I2CON_STA_Pos                       5                                       /*!< I2C I2CON: STA Position            */
#define I2C_I2CON_STA_Msk                       (1ul << I2C_I2CON_STA_Pos)              /*!< I2C I2CON: STA Mask                */

#define I2C_I2CON_STO_Pos                       4                                       /*!< I2C I2CON: STO Position            */
#define I2C_I2CON_STO_Msk                       (1ul << I2C_I2CON_STO_Pos)              /*!< I2C I2CON: STO Mask                */

#define I2C_I2CON_SI_Pos                        3                                       /*!< I2C I2CON: SI Position             */
#define I2C_I2CON_SI_Msk                        (1ul << I2C_I2CON_SI_Pos)               /*!< I2C I2CON: SI Mask                 */

#define I2C_I2CON_AA_Pos                        2                                       /*!< I2C I2CON: AA Position             */
#define I2C_I2CON_AA_Msk                        (1ul << I2C_I2CON_AA_Pos)               /*!< I2C I2CON: AA Mask                 */

/* I2C I2CADDR Bit Field Definitions */
#define I2C_I2CADDR_I2CADDR_Pos                 1                                       /*!< I2C I2CADDR: I2CADDR Position      */
#define I2C_I2CADDR_I2CADDR_Msk                 (0x7Ful << I2C_I2CADDR_I2CADDR_Pos)     /*!< I2C I2CADDR: I2CADDR Mask          */

#define I2C_I2CADDR_GC_Pos                      0                                       /*!< I2C I2CADDR: GC Position           */
#define I2C_I2CADDR_GC_Msk                      (1ul << I2C_I2CADDR_GC_Pos)             /*!< I2C I2CADDR: GC Mask               */

/* I2C I2CDAT Bit Field Definitions */
#define I2C_I2CDAT_I2CDAT_Pos                   0                                       /*!< I2C I2CDAT: I2CDAT Position        */
#define I2C_I2CDAT_I2CDAT_Msk                   (0xFFul << I2C_I2CDAT_I2CDAT_Pos)       /*!< I2C I2CDAT: I2CDAT Mask            */

/* I2C I2CSTATUS Bit Field Definitions */
#define I2C_I2CSTATUS_I2CSTATUS_Pos             0                                       /*!< I2C I2CSTATUS: I2CSTATUS Position  */
#define I2C_I2CSTATUS_I2CSTATUS_Msk             (0xFFul << I2C_I2CSTATUS_I2CSTATUS_Pos) /*!< I2C I2CSTATUS: I2CSTATUS Mask      */

/* I2C I2CLK Bit Field Definitions */
#define I2C_I2CLK_I2CLK_Pos                     0                                       /*!< I2C I2CLK: I2CLK Position          */
#define I2C_I2CLK_I2CLK_Msk                     (0xFFul << I2C_I2CLK_I2CLK_Pos)         /*!< I2C I2CLK: I2CLK Mask              */

/* I2C I2CTOC Bit Field Definitions */
#define I2C_I2CTOC_ENTI_Pos                     2                                       /*!< I2C I2CTOC: ENTI Position          */
#define I2C_I2CTOC_ENTI_Msk                     (1ul << I2C_I2CTOC_ENTI_Pos)            /*!< I2C I2CTOC: ENTI Mask              */

#define I2C_I2CTOC_DIV4_Pos                     1                                       /*!< I2C I2CTOC: DIV4 Position          */
#define I2C_I2CTOC_DIV4_Msk                     (1ul << I2C_I2CTOC_DIV4_Pos)            /*!< I2C I2CTOC: DIV4 Mask              */

#define I2C_I2CTOC_TIF_Pos                      0                                       /*!< I2C I2CTOC: TIF Position           */
#define I2C_I2CTOC_TIF_Msk                      (1ul << I2C_I2CTOC_TIF_Pos)             /*!< I2C I2CTOC: TIF Mask               */

/* I2C I2CADM Bit Field Definitions */
#define I2C_I2CADM_I2CADM_Pos                   1                                       /*!< I2C I2CADM: I2CADM Position        */
#define I2C_I2CADM_I2CADM_Msk                   (0x7Ful << I2C_I2CADM_I2CADM_Pos)       /*!< I2C I2CADM: I2CADM Mask            */

/* I2C I2CON2 Bit Field Definitions */
#define I2C_I2CON2_WKUPEN_Pos                   0                                       /*!< I2C I2CON2: WKUPEN Position        */
#define I2C_I2CON2_WKUPEN_Msk                   (1ul << I2C_I2CON2_WKUPEN_Pos)          /*!< I2C I2CON2: WKUPEN Mask            */

#define I2C_I2CON2_TWOFF_EN_Pos                 1                                       /*!< I2C I2CON2: TWOFF_EN Position      */
#define I2C_I2CON2_TWOFF_EN_Msk                 (1ul << I2C_I2CON2_TWOFF_EN_Pos)        /*!< I2C I2CON2: TWOFF_EN Mask          */

#define I2C_I2CON2_NOSTRETCH_Pos                2                                       /*!< I2C I2CON2: NOSTRETCH Position     */
#define I2C_I2CON2_NOSTRETCH_Msk                (1ul << I2C_I2CON2_NOSTRETCH_Pos)       /*!< I2C I2CON2: NOSTRETCH Mask         */

#define I2C_I2CON2_OVER_INTEN_Pos               3                                       /*!< I2C I2CON2: OVER_INTEN Position    */
#define I2C_I2CON2_OVER_INTEN_Msk               (1ul << I2C_I2CON2_OVER_INTEN_Pos)      /*!< I2C I2CON2: OVER_INTEN Mask        */

#define I2C_I2CON2_UNDER_INTEN_Pos              4                                       /*!< I2C I2CON2: UNDER_INTEN Position   */
#define I2C_I2CON2_UNDER_INTEN_Msk              (1ul << I2C_I2CON2_UNDER_INTEN_Pos)     /*!< I2C I2CON2: UNDER_INTEN Mask       */

/* I2C I2CSTATUS2 Bit Field Definitions */
#define I2C_I2CSTATUS2_WAKEUP_Pos               0                                       /*!< I2C I2CSTATUS2: WAKEUP Position    */
#define I2C_I2CSTATUS2_WAKEUP_Msk               (1ul << I2C_I2CSTATUS2_WAKEUP_Pos)      /*!< I2C I2CSTATUS2: WAKEUP Mask        */

#define I2C_I2CSTATUS2_FULL_Pos                 1                                       /*!< I2C I2CSTATUS2: FULL Position      */
#define I2C_I2CSTATUS2_FULL_Msk                 (1ul << I2C_I2CSTATUS2_FULL_Pos)        /*!< I2C I2CSTATUS2: FULL Mask          */

#define I2C_I2CSTATUS2_EMPTY_Pos                2                                       /*!< I2C I2CSTATUS2: EMPTY Position     */
#define I2C_I2CSTATUS2_EMPTY_Msk                (1ul << I2C_I2CSTATUS2_EMPTY_Pos)       /*!< I2C I2CSTATUS2: EMPTY Mask         */

#define I2C_I2CSTATUS2_OVERUN_Pos               3                                       /*!< I2C I2CSTATUS2: OVERRUN Position    */
#define I2C_I2CSTATUS2_OVERUN_Msk               (1ul << I2C_I2CSTATUS2_OVERUN_Pos)      /*!< I2C I2CSTATUS2: OVERRUN Mask        */

#define I2C_I2CSTATUS2_UNDERUN_Pos              4                                       /*!< I2C I2CSTATUS2: UNDERUN Position   */
#define I2C_I2CSTATUS2_UNDERUN_Msk              (1ul << I2C_I2CSTATUS2_UNDERUN_P        /*!< I2C I2CSTATUS2: UNDERUN Mask       */
/// @endcond /* HIDDEN_SYMBOLS */
/*@}*/ /* end of group I2C */

/*---------------------- Interrupt Source Controller -------------------------*/

/** @addtogroup INT Interrupt Source Controller (INT)
  Memory Mapped Structure for Interrupt Source Controller
  @{
 */

typedef struct
{
    __I  uint32_t IRQSRC[32]; /*!< Offset: 0x0000 ~ 0x007C   Interrupt Source Identity Registers            */
    __IO uint32_t NMICNO;     /*!< Offset: 0x0080            NMI Source Interrupt Select Control Register   */
    __IO uint32_t MCUIRQ;     /*!< Offset: 0x0084            MCU IRQ Number Identity Register               */

} INT_T;
/*@}*/ /* end of group INT */

/*---------------------- Pulse Width Modulation Controller -------------------------*/

/** @addtogroup PWM Pulse Width Modulation Controller(PWM)
  Memory Mapped Structure for PWM Controller
  @{
 */

typedef struct
{
    __IO uint32_t  PPR;                  /*!< Offset: 0x0000   PWM Pre-scale Register                */
    __IO uint32_t  CSR;                  /*!< Offset: 0x0004   PWM Clock Select Register             */
    __IO uint32_t  PCR;                  /*!< Offset: 0x0008   PWM Control Register                  */
    __IO uint32_t  CNR[6];               /*!< Offset: 0x000C ~ 0x0020 PWM Counter Register 0 ~ 5     */
    __IO uint32_t  CMR[6];               /*!< Offset: 0x0024 ~ 0x0038 PWM Comparator Register 0 ~ 5  */
    uint32_t  RESERVED0[6];         /*!< Offset: 0x003C ~ 0x0050 Reserved                       */
    __IO uint32_t  PIER;                 /*!< Offset: 0x0054   PWM Interrupt Enable Register         */
    __IO uint32_t  PIIR;                 /*!< Offset: 0x0058   PWM Interrupt Indication Register     */
    __IO uint32_t  POE;                  /*!< Offset: 0x005C   PWM Output Enable Register            */
    __IO uint32_t  PFBCON;               /*!< Offset: 0x0060   PWM Fault Brake Control Register      */
    __IO uint32_t  PDZIR;                /*!< Offset: 0x0064   PWM Dead-zone Interval Register       */
    __IO uint32_t  TRGCON0;              /*!< Offset: 0x0068   PWM Trigger Control Register 0        */
    __IO uint32_t  TRGCON1;              /*!< Offset: 0x006C   PWM Trigger Control Register 1        */
    __IO uint32_t  TRGSTS0;              /*!< Offset: 0x0070   PWM Trigger Status Register 0         */
    __IO uint32_t  TRGSTS1;              /*!< Offset: 0x0074   PWM Trigger Status Register 1         */
    __IO uint32_t  PHCHG;                /*!< Offset: 0x0078   PWM Phase Changed Register            */
    __IO uint32_t  PHCHGNXT;             /*!< Offset: 0x007C   PWM Next Phase Change Register        */
    __IO uint32_t  PHCHGMASK;            /*!< Offset: 0x0080   PWM Phase Change MASK Register        */
    __IO uint32_t  INTACCUCTL;           /*!< Offset: 0x0084   PWM Period Interrupt Accumulation Control Register   */
} PWM_T;
/// @cond HIDDEN_SYMBOLS
/* PWM PPR Bit Field Definitions */
#define PWM_PPR_CP45_Pos                        16                                  /*!< PWM PPR: CP45 Position     */
#define PWM_PPR_CP45_Msk                        (0xFFul << PWM_PPR_CP45_Pos)        /*!< PWM PPR: CP45 Mask         */

#define PWM_PPR_CP23_Pos                        8                                   /*!< PWM PPR: CP23 Position     */
#define PWM_PPR_CP23_Msk                        (0xFFul << PWM_PPR_CP23_Pos)        /*!< PWM PPR: CP23 Mask         */

#define PWM_PPR_CP01_Pos                        0                                   /*!< PWM PPR: CP01 Position     */
#define PWM_PPR_CP01_Msk                        (0xFFul << PWM_PPR_CP01_Pos)        /*!< PWM PPR: CP01 Mask         */

/* PWM CSR Bit Field Definitions */
#define PWM_CSR_CSR5_Pos                        20                                  /*!< PWM CSR: CSR5 Position     */
#define PWM_CSR_CSR5_Msk                        (7ul << PWM_CSR_CSR5_Pos)           /*!< PWM CSR: CSR5 Mask         */

#define PWM_CSR_CSR4_Pos                        16                                  /*!< PWM CSR: CSR4 Position     */
#define PWM_CSR_CSR4_Msk                        (7ul << PWM_CSR_CSR4_Pos)           /*!< PWM CSR: CSR4 Mask         */

#define PWM_CSR_CSR3_Pos                        12                                  /*!< PWM CSR: CSR3 Position     */
#define PWM_CSR_CSR3_Msk                        (7ul << PWM_CSR_CSR3_Pos)           /*!< PWM CSR: CSR3 Mask         */

#define PWM_CSR_CSR2_Pos                        8                                   /*!< PWM CSR: CSR2 Position     */
#define PWM_CSR_CSR2_Msk                        (7ul << PWM_CSR_CSR2_Pos)           /*!< PWM CSR: CSR2 Mask         */

#define PWM_CSR_CSR1_Pos                        4                                   /*!< PWM CSR: CSR1 Position     */
#define PWM_CSR_CSR1_Msk                        (7ul << PWM_CSR_CSR1_Pos)           /*!< PWM CSR: CSR1 Mask         */

#define PWM_CSR_CSR0_Pos                        0                                   /*!< PWM CSR: CSR0 Position     */
#define PWM_CSR_CSR0_Msk                        (7ul << PWM_CSR_CSR0_Pos)           /*!< PWM CSR: CSR0 Mask         */

/* PWM PCR Bit Field Definitions */
#define PWM_PCR_PWMTYPE_Pos                     31                                  /*!< PWM PCR: PWMTYPE Position  */
#define PWM_PCR_PWMTYPE_Msk                     (1ul << PWM_PCR_PWMTYPE_Pos)        /*!< PWM PCR: PWMTYPE Mask      */

#define PWM_PCR_GRP_Pos                         30                                  /*!< PWM PCR: GRP Position      */
#define PWM_PCR_GRP_Msk                         (1ul << PWM_PCR_GRP_Pos)            /*!< PWM PCR: GRP Mask          */

#define PWM_PCR_PWMMOD_Pos                      28                                  /*!< PWM PCR: PWMMOD Position   */
#define PWM_PCR_PWMMOD_Msk                      (3ul << PWM_PCR_PWMMOD_Pos)         /*!< PWM PCR: PWMMOD Mask       */

#define PWM_PCR_CLRPWM_Pos                      27                                  /*!< PWM PCR: CLRPWM Position   */
#define PWM_PCR_CLRPWM_Msk                      (1ul << PWM_PCR_CLRPWM_Pos)         /*!< PWM PCR: CLRPWM Mask       */

#define PWM_PCR_DZEN45_Pos                      26                                  /*!< PWM PCR: DZEN45 Position   */
#define PWM_PCR_DZEN45_Msk                      (1ul << PWM_PCR_DZEN45_Pos)         /*!< PWM PCR: DZEN45 Mask       */

#define PWM_PCR_DZEN23_Pos                      25                                  /*!< PWM PCR: DZEN23 Position   */
#define PWM_PCR_DZEN23_Msk                      (1ul << PWM_PCR_DZEN23_Pos)         /*!< PWM PCR: DZEN23 Mask       */

#define PWM_PCR_DZEN01_Pos                      24                                  /*!< PWM PCR: DZEN01 Position   */
#define PWM_PCR_DZEN01_Msk                      (1ul << PWM_PCR_DZEN01_Pos)         /*!< PWM PCR: DZEN01 Mask       */

#define PWM_PCR_CH5MOD_Pos                      23                                  /*!< PWM PCR: CH5MOD Position   */
#define PWM_PCR_CH5MOD_Msk                      (1ul << PWM_PCR_CH5MOD_Pos)         /*!< PWM PCR: CH5MOD Mask       */

#define PWM_PCR_CH5INV_Pos                      22                                  /*!< PWM PCR: CH5INV Position   */
#define PWM_PCR_CH5INV_Msk                      (1ul << PWM_PCR_CH5INV_Pos)         /*!< PWM PCR: CH5INV Mask       */

#define PWM_PCR_CH5EN_Pos                       20                                  /*!< PWM PCR: CH5EN Position    */
#define PWM_PCR_CH5EN_Msk                       (1ul << PWM_PCR_CH5EN_Pos)          /*!< PWM PCR: CH5EN Mask        */

#define PWM_PCR_CH4MOD_Pos                      19                                  /*!< PWM PCR: CH4MOD Position   */
#define PWM_PCR_CH4MOD_Msk                      (1ul << PWM_PCR_CH4MOD_Pos)         /*!< PWM PCR: CH4MOD Mask       */

#define PWM_PCR_CH4INV_Pos                      18                                  /*!< PWM PCR: CH4INV Position   */
#define PWM_PCR_CH4INV_Msk                      (1ul << PWM_PCR_CH4INV_Pos)         /*!< PWM PCR: CH4INV Mask       */

#define PWM_PCR_CH4EN_Pos                       16                                  /*!< PWM PCR: CH4EN Position    */
#define PWM_PCR_CH4EN_Msk                       (1ul << PWM_PCR_CH4EN_Pos)          /*!< PWM PCR: CH4EN Mask        */

#define PWM_PCR_CH3MOD_Pos                      15                                  /*!< PWM PCR: CH3MOD Position   */
#define PWM_PCR_CH3MOD_Msk                      (1ul << PWM_PCR_CH3MOD_Pos)         /*!< PWM PCR: CH3MOD Mask       */

#define PWM_PCR_CH3INV_Pos                      14                                  /*!< PWM PCR: CH3INV Position   */
#define PWM_PCR_CH3INV_Msk                      (1ul << PWM_PCR_CH3INV_Pos)         /*!< PWM PCR: CH3INV Mask       */

#define PWM_PCR_CH3EN_Pos                       12                                  /*!< PWM PCR: CH3EN Position    */
#define PWM_PCR_CH3EN_Msk                       (1ul << PWM_PCR_CH3EN_Pos)          /*!< PWM PCR: CH3EN Mask        */

#define PWM_PCR_CH2MOD_Pos                      11                                  /*!< PWM PCR: CH2MOD Position   */
#define PWM_PCR_CH2MOD_Msk                      (1ul << PWM_PCR_CH2MOD_Pos)         /*!< PWM PCR: CH2MOD Mask       */

#define PWM_PCR_CH2INV_Pos                      10                                  /*!< PWM PCR: CH2INV Position   */
#define PWM_PCR_CH2INV_Msk                      (1ul << PWM_PCR_CH2INV_Pos)         /*!< PWM PCR: CH2INV Mask       */

#define PWM_PCR_CH2EN_Pos                       8                                   /*!< PWM PCR: CH2EN Position    */
#define PWM_PCR_CH2EN_Msk                       (1ul << PWM_PCR_CH2EN_Pos)          /*!< PWM PCR: CH2EN Mask        */

#define PWM_PCR_CH1MOD_Pos                      7                                   /*!< PWM PCR: CH1MOD Position   */
#define PWM_PCR_CH1MOD_Msk                      (1ul << PWM_PCR_CH1MOD_Pos)         /*!< PWM PCR: CH1MOD Mask       */

#define PWM_PCR_CH1INV_Pos                      6                                   /*!< PWM PCR: CH1INV Position   */
#define PWM_PCR_CH1INV_Msk                      (1ul << PWM_PCR_CH1INV_Pos)         /*!< PWM PCR: CH1INV Mask       */

#define PWM_PCR_CH1EN_Pos                       4                                   /*!< PWM PCR: CH1EN Position    */
#define PWM_PCR_CH1EN_Msk                       (1ul << PWM_PCR_CH1EN_Pos)          /*!< PWM PCR: CH1EN Mask        */

#define PWM_PCR_CH0MOD_Pos                      3                                   /*!< PWM PCR: CH0MOD Position   */
#define PWM_PCR_CH0MOD_Msk                      (1ul << PWM_PCR_CH0MOD_Pos)         /*!< PWM PCR: CH0MOD Mask       */

#define PWM_PCR_CH0INV_Pos                      2                                   /*!< PWM PCR: CH0INV Position   */
#define PWM_PCR_CH0INV_Msk                      (1ul << PWM_PCR_CH0INV_Pos)         /*!< PWM PCR: CH0INV Mask       */

#define PWM_PCR_DB_MOD_Pos                      1                                   /*!< PWM PCR: DB_MOD Position   */
#define PWM_PCR_DB_MOD_Msk                     (1ul << PWM_PCR_DB_MOD_Pos)          /*!< PWM PCR: DB_MOD Mask       */

#define PWM_PCR_CH0EN_Pos                       0                                   /*!< PWM PCR: CH0EN Position    */
#define PWM_PCR_CH0EN_Msk                       (1ul << PWM_PCR_CH0EN_Pos)          /*!< PWM PCR: CH0EN Mask        */

/* PWM CNR Bit Field Definitions */
#define PWM_CNR_CNR_Pos                         0                                   /*!< PWM CNR: CNR Position  */
#define PWM_CNR_CNR_Msk                         (0xFFFFul << PWM_CNR_CNR_Pos)       /*!< PWM CNR: CNR Mask      */

/* PWM CMR Bit Field Definitions */
#define PWM_CMR_CMR_Pos                         0                                   /*!< PWM CMR: CMR Position  */
#define PWM_CMR_CMR_Msk                         (0xFFFFul << PWM_CMR_CMR_Pos)       /*!< PWM CMR: CMR Mask      */


/* PWM PIER Bit Field Definitions */
#define PWM_PIER_INT_TYPE_Pos                   17                                  /*!< PWM PIER: INT_TYPE Position    */
#define PWM_PIER_INT_TYPE_Msk                   (1ul << PWM_PIER_INT_TYPE_Pos)      /*!< PWM PIER: INT_TYPE Mask        */

#define PWM_PIER_BRKIE_Pos                      16                                  /*!< PWM PIER: BRKIE Position   */
#define PWM_PIER_BRKIE_Msk                      (1ul << PWM_PIER_BRKIE_Pos)         /*!< PWM PIER: BRKIE Mask       */

#define PWM_PIER_PWMDIE5_Pos                    13                                  /*!< PWM PIER: PWMDIE5 Position */
#define PWM_PIER_PWMDIE5_Msk                    (1ul << PWM_PIER_PWMDIE5_Pos)       /*!< PWM PIER: PWMDIE5 Mask     */

#define PWM_PIER_PWMDIE4_Pos                    12                                  /*!< PWM PIER: PWMDIE4 Position */
#define PWM_PIER_PWMDIE4_Msk                    (1ul << PWM_PIER_PWMDIE4_Pos)       /*!< PWM PIER: PWMDIE4 Mask     */

#define PWM_PIER_PWMDIE3_Pos                    11                                  /*!< PWM PIER: PWMDIE3 Position */
#define PWM_PIER_PWMDIE3_Msk                    (1ul << PWM_PIER_PWMDIE3_Pos)       /*!< PWM PIER: PWMDIE3 Mask     */

#define PWM_PIER_PWMDIE2_Pos                    10                                  /*!< PWM PIER: PWMDIE2 Position */
#define PWM_PIER_PWMDIE2_Msk                    (1ul << PWM_PIER_PWMDIE2_Pos)       /*!< PWM PIER: PWMDIE2 Mask     */

#define PWM_PIER_PWMDIE1_Pos                    9                                   /*!< PWM PIER: PWMDIE1 Position */
#define PWM_PIER_PWMDIE1_Msk                    (1ul << PWM_PIER_PWMDIE1_Pos)       /*!< PWM PIER: PWMDIE1 Mask     */

#define PWM_PIER_PWMDIE0_Pos                    8                                   /*!< PWM PIER: PWMDIE0 Position */
#define PWM_PIER_PWMDIE0_Msk                    (1ul << PWM_PIER_PWMDIE0_Pos)       /*!< PWM PIER: PWMDIE0 Mask     */

#define PWM_PIER_PWMPIE5_Pos                    5                                   /*!< PWM PIER: PWMPIE5 Position */
#define PWM_PIER_PWMPIE5_Msk                    (1ul << PWM_PIER_PWMPIE5_Pos)       /*!< PWM PIER: PWMPIE5 Mask     */

#define PWM_PIER_PWMPIE4_Pos                    4                                   /*!< PWM PIER: PWMPIE4 Position */
#define PWM_PIER_PWMPIE4_Msk                    (1ul << PWM_PIER_PWMPIE4_Pos)       /*!< PWM PIER: PWMPIE4 Mask     */

#define PWM_PIER_PWMPIE3_Pos                    3                                   /*!< PWM PIER: PWMPIE3 Position */
#define PWM_PIER_PWMPIE3_Msk                    (1ul << PWM_PIER_PWMPIE3_Pos)       /*!< PWM PIER: PWMPIE3 Mask     */

#define PWM_PIER_PWMPIE2_Pos                    2                                   /*!< PWM PIER: PWMPIE2 Position */
#define PWM_PIER_PWMPIE2_Msk                    (1ul << PWM_PIER_PWMPIE2_Pos)       /*!< PWM PIER: PWMPIE2 Mask     */

#define PWM_PIER_PWMPIE1_Pos                    1                                   /*!< PWM PIER: PWMPIE1 Position */
#define PWM_PIER_PWMPIE1_Msk                    (1ul << PWM_PIER_PWMPIE1_Pos)       /*!< PWM PIER: PWMPIE1 Mask     */

#define PWM_PIER_PWMPIE0_Pos                    0                                   /*!< PWM PIER: PWMPIE0 Position */
#define PWM_PIER_PWMPIE0_Msk                    (1ul << PWM_PIER_PWMPIE0_Pos)       /*!< PWM PIER: PWMPIE0 Mask     */

/* PWM PIIR Bit Field Definitions */
#define PWM_PIIR_BKF1_Pos                       17                                  /*!< PWM PIIR: BKF1 Position    */
#define PWM_PIIR_BKF1_Msk                       (1ul << PWM_PIIR_BKF1_Pos)          /*!< PWM PIIR: BKF1 Mask        */

#define PWM_PIIR_BKF0_Pos                       16                                  /*!< PWM PIIR: BKF0 Position    */
#define PWM_PIIR_BKF0_Msk                       (1ul << PWM_PIIR_BKF0_Pos)          /*!< PWM PIIR: BKF0 Mask        */

#define PWM_PIIR_PWMDIF5_Pos                    13                                  /*!< PWM PIIR: PWMDIF5 Position */
#define PWM_PIIR_PWMDIF5_Msk                    (1ul << PWM_PIIR_PWMDIF5_Pos)       /*!< PWM PIIR: PWMDIF5 Mask     */

#define PWM_PIIR_PWMDIF4_Pos                    12                                  /*!< PWM PIIR: PWMDIF4 Position */
#define PWM_PIIR_PWMDIF4_Msk                    (1ul << PWM_PIIR_PWMDIF4_Pos)       /*!< PWM PIIR: PWMDIF4 Mask     */

#define PWM_PIIR_PWMDIF3_Pos                    11                                  /*!< PWM PIIR: PWMDIF3 Position */
#define PWM_PIIR_PWMDIF3_Msk                    (1ul << PWM_PIIR_PWMDIF3_Pos)       /*!< PWM PIIR: PWMDIF3 Mask     */

#define PWM_PIIR_PWMDIF2_Pos                    10                                  /*!< PWM PIIR: PWMDIF2 Position */
#define PWM_PIIR_PWMDIF2_Msk                    (1ul << PWM_PIIR_PWMDIF2_Pos)       /*!< PWM PIIR: PWMDIF2 Mask     */

#define PWM_PIIR_PWMDIF1_Pos                    9                                   /*!< PWM PIIR: PWMDIF1 Position */
#define PWM_PIIR_PWMDIF1_Msk                    (1ul << PWM_PIIR_PWMDIF1_Pos)       /*!< PWM PIIR: PWMDIF1 Mask     */

#define PWM_PIIR_PWMDIF0_Pos                    8                                   /*!< PWM PIIR: PWMDIF0 Position */
#define PWM_PIIR_PWMDIF0_Msk                    (1ul << PWM_PIIR_PWMDIF0_Pos)       /*!< PWM PIIR: PWMDIF0 Mask     */

#define PWM_PIIR_PWMPIF5_Pos                    5                                   /*!< PWM PIIR: PWMPIF5 Position */
#define PWM_PIIR_PWMPIF5_Msk                    (1ul << PWM_PIIR_PWMPIF5_Pos)       /*!< PWM PIIR: PWMPIF5 Mask     */

#define PWM_PIIR_PWMPIF4_Pos                    4                                   /*!< PWM PIIR: PWMPIF4 Position */
#define PWM_PIIR_PWMPIF4_Msk                    (1ul << PWM_PIIR_PWMPIF4_Pos)       /*!< PWM PIIR: PWMPIF4 Mask     */

#define PWM_PIIR_PWMPIF3_Pos                    3                                   /*!< PWM PIIR: PWMPIF3 Position */
#define PWM_PIIR_PWMPIF3_Msk                    (1ul << PWM_PIIR_PWMPIF3_Pos)       /*!< PWM PIIR: PWMPIF3 Mask     */

#define PWM_PIIR_PWMPIF2_Pos                    2                                   /*!< PWM PIIR: PWMPIF2 Position */
#define PWM_PIIR_PWMPIF2_Msk                    (1ul << PWM_PIIR_PWMPIF2_Pos)       /*!< PWM PIIR: PWMPIF2 Mask     */

#define PWM_PIIR_PWMPIF1_Pos                    1                                   /*!< PWM PIIR: PWMPIF1 Position */
#define PWM_PIIR_PWMPIF1_Msk                    (1ul << PWM_PIIR_PWMPIF1_Pos)       /*!< PWM PIIR: PWMPIF1 Mask     */

#define PWM_PIIR_PWMPIF0_Pos                    0                                   /*!< PWM PIIR: PWMPIF0 Position */
#define PWM_PIIR_PWMPIF0_Msk                    (1ul << PWM_PIIR_PWMPIF0_Pos)       /*!< PWM PIIR: PWMPIF0 Mask     */

/* PWM POE Bit Field Definitions */
#define PWM_POE_PWM5_Pos                        5                                   /*!< PWM POE: POE5 Position     */
#define PWM_POE_PWM5_Msk                        (1ul << PWM_POE_PWM5_Pos)           /*!< PWM POE: POE5 Mask         */

#define PWM_POE_PWM4_Pos                        4                                   /*!< PWM POE: POE4 Position     */
#define PWM_POE_PWM4_Msk                        (1ul << PWM_POE_PWM4_Pos)           /*!< PWM POE: POE4 Mask         */

#define PWM_POE_PWM3_Pos                        3                                   /*!< PWM POE: POE3 Position     */
#define PWM_POE_PWM3_Msk                        (1ul << PWM_POE_PWM3_Pos)           /*!< PWM POE: POE3 Mask         */

#define PWM_POE_PWM2_Pos                        2                                   /*!< PWM POE: POE2 Position     */
#define PWM_POE_PWM2_Msk                        (1ul << PWM_POE_PWM2_Pos)           /*!< PWM POE: POE2 Mask         */

#define PWM_POE_PWM1_Pos                        1                                   /*!< PWM POE: POE1 Position     */
#define PWM_POE_PWM1_Msk                        (1ul << PWM_POE_PWM1_Pos)           /*!< PWM POE: POE1 Mask         */

#define PWM_POE_PWM0_Pos                        0                                   /*!< PWM POE: POE0 Position     */
#define PWM_POE_PWM0_Msk                        (1ul << PWM_POE_PWM0_Pos)           /*!< PWM POE: POE0 Mask         */

/* PWM PFBCON Bit Field Definitions */
#define PWM_PFBCON_D7BKO7_Pos                   31                                  /*!< PWM PFBCON: D7BKO7 Position    */
#define PWM_PFBCON_D7BKO7_Msk                   (1ul << PWM_PFBCON_D7BKO7_Pos)      /*!< PWM PFBCON: D7BKO7 Mask        */

#define PWM_PFBCON_D6BKO6_Pos                   30                                  /*!< PWM PFBCON: D6BKO6 Position    */
#define PWM_PFBCON_D6BKO6_Msk                   (1ul << PWM_PFBCON_D6BKO6_Pos)      /*!< PWM PFBCON: D6BKO6 Mask        */

#define PWM_PFBCON_PWMBKO5_Pos                  29                                  /*!< PWM PFBCON: PWMBKO5 Position   */
#define PWM_PFBCON_PWMBKO5_Msk                  (1ul << PWM_PFBCON_PWMBKO5_Pos)     /*!< PWM PFBCON: PWMBKO5 Mask       */

#define PWM_PFBCON_PWMBKO4_Pos                  28                                  /*!< PWM PFBCON: PWMBKO4 Position   */
#define PWM_PFBCON_PWMBKO4_Msk                  (1ul << PWM_PFBCON_PWMBKO4_Pos)     /*!< PWM PFBCON: PWMBKO4 Mask       */

#define PWM_PFBCON_PWMBKO3_Pos                  27                                  /*!< PWM PFBCON: PWMBKO3 Position   */
#define PWM_PFBCON_PWMBKO3_Msk                  (1ul << PWM_PFBCON_PWMBKO3_Pos)     /*!< PWM PFBCON: PWMBKO3 Mask       */

#define PWM_PFBCON_PWMBKO2_Pos                  26                                  /*!< PWM PFBCON: PWMBKO2 Position   */
#define PWM_PFBCON_PWMBKO2_Msk                  (1ul << PWM_PFBCON_PWMBKO2_Pos)     /*!< PWM PFBCON: PWMBKO2 Mask       */

#define PWM_PFBCON_PWMBKO1_Pos                  25                                  /*!< PWM PFBCON: PWMBKO1 Position   */
#define PWM_PFBCON_PWMBKO1_Msk                  (1ul << PWM_PFBCON_PWMBKO1_Pos)     /*!< PWM PFBCON: PWMBKO1 Mask       */

#define PWM_PFBCON_PWMBKO0_Pos                  24                                  /*!< PWM PFBCON: PWMBKO0 Position   */
#define PWM_PFBCON_PWMBKO0_Msk                  (1ul << PWM_PFBCON_PWMBKO0_Pos)     /*!< PWM PFBCON: PWMBKO0 Mask       */

#define PWM_PFBCON_BKF_Pos                      7                                   /*!< PWM PFBCON: BKF Position   */
#define PWM_PFBCON_BKF_Msk                      (1ul << PWM_PFBCON_BKF_Pos)         /*!< PWM PFBCON: BKF Mask       */

#define PWM_PFBCON_CPO0BKEN_Pos                 2                                   /*!< PWM PFBCON: CPO0BKEN Position  */
#define PWM_PFBCON_CPO0BKEN_Msk                 (1ul << PWM_PFBCON_CPO0BKEN_Pos)    /*!< PWM PFBCON: CPO0BKEN Mask      */

#define PWM_PFBCON_BKEN1_Pos                    1                                   /*!< PWM PFBCON: BKEN1 Position */
#define PWM_PFBCON_BKEN1_Msk                    (1ul << PWM_PFBCON_BKEN1_Pos)       /*!< PWM PFBCON: BKEN1 Mask     */

#define PWM_PFBCON_BKEN0_Pos                    0                                   /*!< PWM PFBCON: BKEN0 Position */
#define PWM_PFBCON_BKEN0_Msk                    (1ul << PWM_PFBCON_BKEN0_Pos)       /*!< PWM PFBCON: BKEN0 Mask     */

/* PWM DZIR Bit Field Definitions */
#define PWM_DZIR_DZI45_Pos                      16                                  /*!< PWM DZIR: DZI45 Position   */
#define PWM_DZIR_DZI45_Msk                      (0xFFul << PWM_DZIR_DZI45_Pos)      /*!< PWM DZIR: DZI45 Mask       */

#define PWM_DZIR_DZI23_Pos                      8                                   /*!< PWM DZIR: DZI23 Position   */
#define PWM_DZIR_DZI23_Msk                      (0xFFul << PWM_DZIR_DZI23_Pos)      /*!< PWM DZIR: DZI23 Mask       */

#define PWM_DZIR_DZI01_Pos                      0                                   /*!< PWM DZIR: DZI01 Position   */
#define PWM_DZIR_DZI01_Msk                      (0xFFul << PWM_DZIR_DZI01_Pos)      /*!< PWM DZIR: DZI01 Mask       */

/* PWM TRGCON0 Bit Field Definitions */
#define PWM_TRGCON0_P3TRGEN_Pos                 27                                  /*!< PWM TRGCON0: P3TRGEN Position      */
#define PWM_TRGCON0_P3TRGEN_Msk                 (1ul << PWM_TRGCON0_P3TRGEN_Pos)    /*!< PWM TRGCON0: P3TRGEN Mask          */

#define PWM_TRGCON0_CM3TRGFEN_Pos               26                                  /*!< PWM TRGCON0: CM3TRGFEN Position    */
#define PWM_TRGCON0_CM3TRGFEN_Msk               (1ul << PWM_TRGCON0_CM3TRGFEN_Pos)  /*!< PWM TRGCON0: CM3TRGFEN Mask        */

#define PWM_TRGCON0_CNT3TRGEN_Pos               25                                  /*!< PWM TRGCON0: CNT3TRGEN Position    */
#define PWM_TRGCON0_CNT3TRGEN_Msk               (1ul << PWM_TRGCON0_CNT3TRGEN_Pos)  /*!< PWM TRGCON0: CNT3TRGEN Mask        */

#define PWM_TRGCON0_CM3TRGREN_Pos               24                                  /*!< PWM TRGCON0: CM3TRGREN Position    */
#define PWM_TRGCON0_CM3TRGREN_Msk               (1ul << PWM_TRGCON0_CM3TRGREN_Pos)  /*!< PWM TRGCON0: CM3TRGREN Mask        */

#define PWM_TRGCON0_P2TRGEN_Pos                 19                                  /*!< PWM TRGCON0: P2TRGEN Position      */
#define PWM_TRGCON0_P2TRGEN_Msk                 (1ul << PWM_TRGCON0_P2TRGEN_Pos)    /*!< PWM TRGCON0: P2TRGEN Mask          */

#define PWM_TRGCON0_CM2TRGFEN_Pos               18                                  /*!< PWM TRGCON0: CM2TRGFEN Position    */
#define PWM_TRGCON0_CM2TRGFEN_Msk               (1ul << PWM_TRGCON0_CM2TRGFEN_Pos)  /*!< PWM TRGCON0: CM2TRGFEN Mask        */

#define PWM_TRGCON0_CNT2TRGEN_Pos               17                                  /*!< PWM TRGCON0: CNT2TRGEN Position    */
#define PWM_TRGCON0_CNT2TRGEN_Msk               (1ul << PWM_TRGCON0_CNT2TRGEN_Pos)  /*!< PWM TRGCON0: CNT2TRGEN Mask        */

#define PWM_TRGCON0_CM2TRGREN_Pos               16                                  /*!< PWM TRGCON0: CM2TRGREN Position    */
#define PWM_TRGCON0_CM2TRGREN_Msk               (1ul << PWM_TRGCON0_CM2TRGREN_Pos)  /*!< PWM TRGCON0: CM2TRGREN Mask        */

#define PWM_TRGCON0_P1TRGEN_Pos                 11                                  /*!< PWM TRGCON0: P1TRGEN Position      */
#define PWM_TRGCON0_P1TRGEN_Msk                 (1ul << PWM_TRGCON0_P1TRGEN_Pos)    /*!< PWM TRGCON0: P1TRGEN Mask          */

#define PWM_TRGCON0_CM1TRGFEN_Pos               10                                  /*!< PWM TRGCON0: CM1TRGFEN Position    */
#define PWM_TRGCON0_CM1TRGFEN_Msk               (1ul << PWM_TRGCON0_CM1TRGFEN_Pos)  /*!< PWM TRGCON0: CM1TRGFEN Mask        */

#define PWM_TRGCON0_CNT1TRGEN_Pos               9                                   /*!< PWM TRGCON0: CNT1TRGEN Position    */
#define PWM_TRGCON0_CNT1TRGEN_Msk               (1ul << PWM_TRGCON0_CNT1TRGEN_Pos)  /*!< PWM TRGCON0: CNT1TRGEN Mask        */

#define PWM_TRGCON0_CM1TRGREN_Pos               8                                   /*!< PWM TRGCON0: CM1TRGREN Position    */
#define PWM_TRGCON0_CM1TRGREN_Msk               (1ul << PWM_TRGCON0_CM1TRGREN_Pos)  /*!< PWM TRGCON0: CM1TRGREN Mask        */

#define PWM_TRGCON0_P0TRGEN_Pos                 3                                   /*!< PWM TRGCON0: P0TRGEN Position      */
#define PWM_TRGCON0_P0TRGEN_Msk                 (1ul << PWM_TRGCON0_P0TRGEN_Pos)    /*!< PWM TRGCON0: P0TRGEN Mask          */

#define PWM_TRGCON0_CM0TRGFEN_Pos               2                                   /*!< PWM TRGCON0: CM0TRGFEN Position    */
#define PWM_TRGCON0_CM0TRGFEN_Msk               (1ul << PWM_TRGCON0_CM0TRGFEN_Pos)  /*!< PWM TRGCON0: CM0TRGFEN Mask        */

#define PWM_TRGCON0_CNT0TRGEN_Pos               1                                   /*!< PWM TRGCON0: CNT0TRGEN Position    */
#define PWM_TRGCON0_CNT0TRGEN_Msk               (1ul << PWM_TRGCON0_CNT0TRGEN_Pos)  /*!< PWM TRGCON0: CNT0TRGEN Mask        */

#define PWM_TRGCON0_CM0TRGREN_Pos               0                                   /*!< PWM TRGCON0: CM0TRGREN Position    */
#define PWM_TRGCON0_CM0TRGREN_Msk               (1ul << PWM_TRGCON0_CM0TRGREN_Pos)  /*!< PWM TRGCON0: CM0TRGREN Mask        */

/* PWM TRGCON1 Bit Field Definitions */
#define PWM_TRGCON1_P5TRGEN_Pos                 11                                  /*!< PWM TRGCON1: P5TRGEN Position      */
#define PWM_TRGCON1_P5TRGEN_Msk                 (1ul << PWM_TRGCON1_P5TRGEN_Pos)    /*!< PWM TRGCON1: P5TRGEN Mask          */

#define PWM_TRGCON1_CM5TRGFEN_Pos               10                                  /*!< PWM TRGCON1: CM5TRGFEN Position    */
#define PWM_TRGCON1_CM5TRGFEN_Msk               (1ul << PWM_TRGCON1_CM5TRGFEN_Pos)  /*!< PWM TRGCON1: CM5TRGFEN Mask        */

#define PWM_TRGCON1_CNT5TRGEN_Pos               9                                   /*!< PWM TRGCON1: CNT5TRGEN Position    */
#define PWM_TRGCON1_CNT5TRGEN_Msk               (1ul << PWM_TRGCON1_CNT5TRGEN_Pos)  /*!< PWM TRGCON1: CNT5TRGEN Mask        */

#define PWM_TRGCON1_CM5TRGREN_Pos               8                                   /*!< PWM TRGCON1: CM5TRGREN Position    */
#define PWM_TRGCON1_CM5TRGREN_Msk               (1ul << PWM_TRGCON1_CM5TRGREN_Pos)  /*!< PWM TRGCON1: CM5TRGREN Mask        */

#define PWM_TRGCON1_P4TRGEN_Pos                 3                                   /*!< PWM TRGCON1: P4TRGEN Position      */
#define PWM_TRGCON1_P4TRGEN_Msk                 (1ul << PWM_TRGCON1_P4TRGEN_Pos)    /*!< PWM TRGCON1: P4TRGEN Mask          */

#define PWM_TRGCON1_CM4TRGFEN_Pos               2                                   /*!< PWM TRGCON1: CM4TRGFEN Position    */
#define PWM_TRGCON1_CM4TRGFEN_Msk               (1ul << PWM_TRGCON1_CM4TRGFEN_Pos)  /*!< PWM TRGCON1: CM4TRGFEN Mask        */

#define PWM_TRGCON1_CNT4TRGEN_Pos               1                                   /*!< PWM TRGCON1: CNT4TRGEN Position    */
#define PWM_TRGCON1_CNT4TRGEN_Msk               (1ul << PWM_TRGCON1_CNT4TRGEN_Pos)  /*!< PWM TRGCON1: CNT4TRGEN Mask        */

#define PWM_TRGCON1_CM4TRGREN_Pos               0                                   /*!< PWM TRGCON1: CM4TRGREN Position    */
#define PWM_TRGCON1_CM4TRGREN_Msk               (1ul << PWM_TRGCON1_CM4TRGREN_Pos)  /*!< PWM TRGCON1: CM4TRGREN Mask        */

/* PWM TRGSTS0 Bit Field Definitions */
#define PWM_TRGSTS0_PERID3FLAG_Pos              27                                  /*!< PWM TRGSTS0: PERID3FLAG Position   */
#define PWM_TRGSTS0_PERID3FLAG_Msk              (1ul << PWM_TRGSTS0_PERID3FLAG_Pos) /*!< PWM TRGSTS0: PERID3FLAG Mask       */

#define PWM_TRGSTS0_CMR3FLAG_F_Pos              26                                  /*!< PWM TRGSTS0: CMR3FLAG_F Position   */
#define PWM_TRGSTS0_CMR3FLAG_F_Msk              (1ul << PWM_TRGSTS0_CMR3FLAG_F_Pos) /*!< PWM TRGSTS0: CMR3FLAG_F Mask       */

#define PWM_TRGSTS0_CNT3FLAG_Pos                25                                  /*!< PWM TRGSTS0: CNT3FLAG Position     */
#define PWM_TRGSTS0_CNT3FLAG_Msk                (1ul << PWM_TRGSTS0_CNT3FLAG_Pos)   /*!< PWM TRGSTS0: CNT3FLAG Mask         */

#define PWM_TRGSTS0_CMR3FLAG_R_Pos              24                                  /*!< PWM TRGSTS0: CMR3FLAG_R Position   */
#define PWM_TRGSTS0_CMR3FLAG_R_Msk              (1ul << PWM_TRGSTS0_CMR3FLAG_R_Pos) /*!< PWM TRGSTS0: CMR3FLAG_R Mask       */

#define PWM_TRGSTS0_PERID2FLAG_Pos              19                                  /*!< PWM TRGSTS0: PERID2FLAG Position   */
#define PWM_TRGSTS0_PERID2FLAG_Msk              (1ul << PWM_TRGSTS0_PERID2FLAG_Pos) /*!< PWM TRGSTS0: PERID2FLAG Mask       */

#define PWM_TRGSTS0_CMR2FLAG_F_Pos              18                                  /*!< PWM TRGSTS0: CMR2FLAG_F Position   */
#define PWM_TRGSTS0_CMR2FLAG_F_Msk              (1ul << PWM_TRGSTS0_CMR2FLAG_F_Pos) /*!< PWM TRGSTS0: CMR2FLAG_F Mask       */

#define PWM_TRGSTS0_CNT2FLAG_Pos                17                                  /*!< PWM TRGSTS0: CNT2FLAG Position     */
#define PWM_TRGSTS0_CNT2FLAG_Msk                (1ul << PWM_TRGSTS0_CNT2FLAG_Pos)   /*!< PWM TRGSTS0: CNT2FLAG Mask         */

#define PWM_TRGSTS0_CMR2FLAG_R_Pos              16                                  /*!< PWM TRGSTS0: CMR2FLAG_R Position   */
#define PWM_TRGSTS0_CMR2FLAG_R_Msk              (1ul << PWM_TRGSTS0_CMR2FLAG_R_Pos) /*!< PWM TRGSTS0: CMR2FLAG_R Mask       */

#define PWM_TRGSTS0_PERID1FLAG_Pos              11                                  /*!< PWM TRGSTS0: PERID1FLAG Position   */
#define PWM_TRGSTS0_PERID1FLAG_Msk              (1ul << PWM_TRGSTS0_PERID1FLAG_Pos) /*!< PWM TRGSTS0: PERID1FLAG Mask       */

#define PWM_TRGSTS0_CMR1FLAG_F_Pos              10                                  /*!< PWM TRGSTS0: CMR1FLAG_F Position   */
#define PWM_TRGSTS0_CMR1FLAG_F_Msk              (1ul << PWM_TRGSTS0_CMR1FLAG_F_Pos) /*!< PWM TRGSTS0: CMR1FLAG_F Mask       */

#define PWM_TRGSTS0_CNT1FLAG_Pos                9                                   /*!< PWM TRGSTS0: CNT1FLAG Position     */
#define PWM_TRGSTS0_CNT1FLAG_Msk                (1ul << PWM_TRGSTS0_CNT1FLAG_Pos)   /*!< PWM TRGSTS0: CNT1FLAG Mask         */

#define PWM_TRGSTS0_CMR1FLAG_R_Pos              8                                   /*!< PWM TRGSTS0: CMR1FLAG_R Position   */
#define PWM_TRGSTS0_CMR1FLAG_R_Msk              (1ul << PWM_TRGSTS0_CMR1FLAG_R_Pos) /*!< PWM TRGSTS0: CMR1FLAG_R Mask       */

#define PWM_TRGSTS0_PERID0FLAG_Pos              3                                   /*!< PWM TRGSTS0: PERID0FLAG Position   */
#define PWM_TRGSTS0_PERID0FLAG_Msk              (1ul << PWM_TRGSTS0_PERID0FLAG_Pos) /*!< PWM TRGSTS0: PERID0FLAG Mask       */

#define PWM_TRGSTS0_CMR0FLAG_F_Pos              2                                   /*!< PWM TRGSTS0: CMR0FLAG_F Position   */
#define PWM_TRGSTS0_CMR0FLAG_F_Msk              (1ul << PWM_TRGSTS0_CMR0FLAG_F_Pos) /*!< PWM TRGSTS0: CMR0FLAG_F Mask       */

#define PWM_TRGSTS0_CNT0FLAG_Pos                1                                   /*!< PWM TRGSTS0: CNT0FLAG Position     */
#define PWM_TRGSTS0_CNT0FLAG_Msk                (1ul << PWM_TRGSTS0_CNT0FLAG_Pos)   /*!< PWM TRGSTS0: CNT0FLAG Mask         */

#define PWM_TRGSTS0_CMR0FLAG_R_Pos              0                                   /*!< PWM TRGSTS0: CMR0FLAG_R Position   */
#define PWM_TRGSTS0_CMR0FLAG_R_Msk              (1ul << PWM_TRGSTS0_CMR0FLAG_R_Pos) /*!< PWM TRGSTS0: CMR0FLAG_R Mask       */

/* PWM TRGSTS1 Bit Field Definitions */
#define PWM_TRGSTS1_PERID5FLAG_Pos              11                                  /*!< PWM TRGSTS1: PERID5FLAG Position   */
#define PWM_TRGSTS1_PERID5FLAG_Msk              (1ul << PWM_TRGSTS1_PERID5FLAG_Pos) /*!< PWM TRGSTS1: PERID5FLAG Mask       */

#define PWM_TRGSTS1_CMR5FLAG_F_Pos              10                                  /*!< PWM TRGSTS1: CMR5FLAG_F Position   */
#define PWM_TRGSTS1_CMR5FLAG_F_Msk              (1ul << PWM_TRGSTS1_CMR5FLAG_F_Pos) /*!< PWM TRGSTS1: CMR5FLAG_F Mask       */

#define PWM_TRGSTS1_CNT5FLAG_Pos                9                                   /*!< PWM TRGSTS1: CNT5FLAG Position     */
#define PWM_TRGSTS1_CNT5FLAG_Msk                (1ul << PWM_TRGSTS1_CNT5FLAG_Pos)   /*!< PWM TRGSTS1: CNT5FLAG Mask         */

#define PWM_TRGSTS1_CMR5FLAG_R_Pos              8                                   /*!< PWM TRGSTS1: CMR5FLAG_R Position   */
#define PWM_TRGSTS1_CMR5FLAG_R_Msk              (1ul << PWM_TRGSTS1_CMR5FLAG_R_Pos) /*!< PWM TRGSTS1: CMR5FLAG_R Mask       */

#define PWM_TRGSTS1_PERID4FLAG_Pos              3                                   /*!< PWM TRGSTS1: PERID4FLAG Position   */
#define PWM_TRGSTS1_PERID4FLAG_Msk              (1ul << PWM_TRGSTS1_PERID4FLAG_Pos) /*!< PWM TRGSTS1: PERID4FLAG Mask       */

#define PWM_TRGSTS1_CMR4FLAG_F_Pos              2                                   /*!< PWM TRGSTS1: CMR4FLAG_F Position   */
#define PWM_TRGSTS1_CMR4FLAG_F_Msk              (1ul << PWM_TRGSTS1_CMR4FLAG_F_Pos) /*!< PWM TRGSTS1: CMR4FLAG_F Mask       */

#define PWM_TRGSTS1_CNT4FLAG_Pos                1                                   /*!< PWM TRGSTS1: CNT4FLAG Position     */
#define PWM_TRGSTS1_CNT4FLAG_Msk                (1ul << PWM_TRGSTS1_CNT4FLAG_Pos)   /*!< PWM TRGSTS1: CNT4FLAG Mask         */

#define PWM_TRGSTS1_CMR4FLAG_R_Pos              0                                   /*!< PWM TRGSTS1: CMR4FLAG_R Position   */
#define PWM_TRGSTS1_CMR4FLAG_R_Msk              (1ul << PWM_TRGSTS1_CMR4FLAG_R_Pos) /*!< PWM TRGSTS1: CMR4FLAG_R Mask       */

/* PWM PHCHG Bit Field Definitions */
#define PWM_PHCHG_CE0_Pos                       31                                  /*!< PWM PHCHG: CE0 Position        */
#define PWM_PHCHG_CE0_Msk                       (1ul << PWM_PHCHG_CE0_Pos)          /*!< PWM PHCHG: CE0 Mask            */

#define PWM_PHCHG_T0_Pos                        30                                  /*!< PWM PHCHG: T0 Position         */
#define PWM_PHCHG_T0_Msk                        (1ul << PWM_PHCHG_T0_Pos)           /*!< PWM PHCHG: T0 Mask             */

#define PWM_PHCHG_CMP0SEL_Pos                   28                                  /*!< PWM PHCHG: CMP0SEL Position    */
#define PWM_PHCHG_CMP0SEL_Msk                   (3ul << PWM_PHCHG_CMP0SEL_Pos)      /*!< PWM PHCHG: CMP0SEL Mask        */

#define PWM_PHCHG_CH31TOFF0_Pos                 27                                  /*!< PWM PHCHG: CH31TOFF0 Position  */
#define PWM_PHCHG_CH31TOFF0_Msk                 (1ul << PWM_PHCHG_CH31TOFF0_Pos)    /*!< PWM PHCHG: CH31TOFF0 Mask      */

#define PWM_PHCHG_CH21TOFF0_Pos                 26                                  /*!< PWM PHCHG: CH21TOFF0 Position  */
#define PWM_PHCHG_CH21TOFF0_Msk                 (1ul << PWM_PHCHG_CH21TOFF0_Pos)    /*!< PWM PHCHG: CH21TOFF0 Mask      */

#define PWM_PHCHG_CH11TOFF0_Pos                 25                                  /*!< PWM PHCHG: CH11TOFF0 Position  */
#define PWM_PHCHG_CH11TOFF0_Msk                 (1ul << PWM_PHCHG_CH11TOFF0_Pos)    /*!< PWM PHCHG: CH11TOFF0 Mask      */

#define PWM_PHCHG_CH01TOFF0_Pos                 24                                  /*!< PWM PHCHG: CH01TOFF0 Position  */
#define PWM_PHCHG_CH01TOFF0_Msk                 (1ul << PWM_PHCHG_CH01TOFF0_Pos)    /*!< PWM PHCHG: CH01TOFF0 Mask      */

#define PWM_PHCHG_CE1_Pos                       23                                  /*!< PWM PHCHG: CE1 Position    */
#define PWM_PHCHG_CE1_Msk                       (1ul << PWM_PHCHG_CE1_Pos)          /*!< PWM PHCHG: CE1 Mask        */

#define PWM_PHCHG_T1_Pos                        22                                  /*!< PWM PHCHG: T1 Position     */
#define PWM_PHCHG_T1_Msk                        (1ul << PWM_PHCHG_T1_Pos)           /*!< PWM PHCHG: T1 Mask         */

#define PWM_PHCHG_CMP1SEL_Pos                   20                                  /*!< PWM PHCHG: CMP1SEL Position    */
#define PWM_PHCHG_CMP1SEL_Msk                   (3ul << PWM_PHCHG_CMP1SEL_Pos)      /*!< PWM PHCHG: CMP1SEL Mask        */

#define PWM_PHCHG_CH31TOFF1_Pos                 19                                  /*!< PWM PHCHG: CH31TOFF1 Position  */
#define PWM_PHCHG_CH31TOFF1_Msk                 (1ul << PWM_PHCHG_CH31TOFF1_Pos)    /*!< PWM PHCHG: CH31TOFF1 Mask      */

#define PWM_PHCHG_CH21TOFF1_Pos                 18                                  /*!< PWM PHCHG: CH21TOFF1 Position  */
#define PWM_PHCHG_CH21TOFF1_Msk                 (1ul << PWM_PHCHG_CH21TOFF1_Pos)    /*!< PWM PHCHG: CH21TOFF1 Mask      */

#define PWM_PHCHG_CH11TOFF1_Pos                 17                                  /*!< PWM PHCHG: CH11TOFF1 Position  */
#define PWM_PHCHG_CH11TOFF1_Msk                 (1ul << PWM_PHCHG_CH11TOFF1_Pos)    /*!< PWM PHCHG: CH11TOFF1 Mask      */

#define PWM_PHCHG_CH01TOFF1_Pos                 16                                  /*!< PWM PHCHG: CH01TOFF1 Position  */
#define PWM_PHCHG_CH01TOFF1_Msk                 (1ul << PWM_PHCHG_CH01TOFF1_Pos)    /*!< PWM PHCHG: CH01TOFF1 Mask      */

#define PWM_PHCHG_ACCNT1_Pos                    15                                  /*!< PWM PHCHG: ACCNT1 Position */
#define PWM_PHCHG_ACCNT1_Msk                    (1ul << PWM_PHCHG_ACCNT1_Pos)       /*!< PWM PHCHG: ACCNT1 Mask     */

#define PWM_PHCHG_ACCNT0_Pos                    14                                  /*!< PWM PHCHG: ACCNT0 Position */
#define PWM_PHCHG_ACCNT0_Msk                    (1ul << PWM_PHCHG_ACCNT0_Pos)       /*!< PWM PHCHG: ACCNT0 Mask     */

#define PWM_PHCHG_PWM5_Pos                      13                                  /*!< PWM PHCHG: PWM5 Position   */
#define PWM_PHCHG_PWM5_Msk                      (1ul << PWM_PHCHGPWM5_Pos)          /*!< PWM PHCHG: PWM5 Mask       */

#define PWM_PHCHG_PWM4_Pos                      12                                  /*!< PWM PHCHG: PWM4 Position   */
#define PWM_PHCHG_PWM4_Msk                      (1ul << PWM_PHCHG_PWM4_Pos)         /*!< PWM PHCHG: PWM4 Mask       */

#define PWM_PHCHG_PWM3_Pos                      11                                  /*!< PWM PHCHG: PWM3 Position   */
#define PWM_PHCHG_PWM3_Msk                      (1ul << PWM_PHCHG_PWM3_Pos)         /*!< PWM PHCHG: PWM3 Mask       */

#define PWM_PHCHG_PWM2_Pos                      10                                  /*!< PWM PHCHG: PWM2 Position   */
#define PWM_PHCHG_PWM2_Msk                      (1ul << PWM_PHCHG_PWM2_Pos)         /*!< PWM PHCHG: PWM2 Mask       */

#define PWM_PHCHG_PWM1_Pos                      9                                   /*!< PWM PHCHG: PWM1 Position   */
#define PWM_PHCHG_PWM1_Msk                      (1ul << PWM_PHCHG_PWM1_Pos)         /*!< PWM PHCHG: PWM1 Mask       */

#define PWM_PHCHG_PWM0_Pos                      8                                   /*!< PWM PHCHG: PWM0 Position   */
#define PWM_PHCHG_PWM0_Msk                      (1ul << PWM_PHCHG_PWM0_Pos)         /*!< PWM PHCHG: PWM0 Mask       */

#define PWM_PHCHG_D7_Pos                        7                                   /*!< PWM PHCHG: D7 Position */
#define PWM_PHCHG_D7_Msk                        (1ul << PWM_PHCHG_D7_Pos)           /*!< PWM PHCHG: D7 Mask     */

#define PWM_PHCHG_D6_Pos                        6                                   /*!< PWM PHCHG: D6 Position */
#define PWM_PHCHG_D6_Msk                        (1ul << PWM_PHCHG_D6_Pos)           /*!< PWM PHCHG: D6 Mask     */

#define PWM_PHCHG_D5_Pos                        5                                   /*!< PWM PHCHG: D5 Position */
#define PWM_PHCHG_D5_Msk                        (1ul << PWM_PHCHG_D5_Pos)           /*!< PWM PHCHG: D5 Mask     */

#define PWM_PHCHG_D4_Pos                        4                                   /*!< PWM PHCHG: D4 Position */
#define PWM_PHCHG_D4_Msk                        (1ul << PWM_PHCHG_D4_Pos)           /*!< PWM PHCHG: D4 Mask     */

#define PWM_PHCHG_D3_Pos                        3                                   /*!< PWM PHCHG: D3 Position */
#define PWM_PHCHG_D3_Msk                        (1ul << PWM_PHCHG_D3_Pos)           /*!< PWM PHCHG: D3 Mask     */

#define PWM_PHCHG_D2_Pos                        2                                   /*!< PWM PHCHG: D2 Position */
#define PWM_PHCHG_D2_Msk                        (1ul << PWM_PHCHG_D2_Pos)           /*!< PWM PHCHG: D2 Mask     */

#define PWM_PHCHG_D1_Pos                        1                                   /*!< PWM PHCHG: D1 Position */
#define PWM_PHCHG_D1_Msk                        (1ul << PWM_PHCHG_D1_Pos)           /*!< PWM PHCHG: D1 Mask     */

#define PWM_PHCHG_D0_Pos                        0                                   /*!< PWM PHCHG: D0 Position */
#define PWM_PHCHG_D0_Msk                        (1ul << PWM_PHCHG_D0_Pos)           /*!< PWM PHCHG: D0 Mask     */

/* PWM PHCHGNXT Bit Field Definitions */
#define PWM_PHCHGNXT_CE0_Pos                       31                                  /*!< PWM PHCHGNXT: CE0 Position  */
#define PWM_PHCHGNXT_CE0_Msk                       (1ul << PWM_PHCHGNXT_CE0_Pos)       /*!< PWM PHCHGNXT: CE0 Mask      */

#define PWM_PHCHGNXT_T0_Pos                        30                                  /*!< PWM PHCHGNXT: T0 Position   */
#define PWM_PHCHGNXT_T0_Msk                        (1ul << PWM_PHCHGNXT_T0_Pos)        /*!< PWM PHCHGNXT: T0 Mask       */

#define PWM_PHCHGNXT_CMP0SEL_Pos                   28                                  /*!< PWM PHCHGNXT: CMP0SEL Position  */
#define PWM_PHCHGNXT_CMP0SEL_Msk                   (3ul << PWM_PHCHGNXT_CMP0SEL_Pos)   /*!< PWM PHCHGNXT: CMP0SEL Mask      */

#define PWM_PHCHGNXT_CH31TOFF0_Pos                 27                                  /*!< PWM PHCHGNXT: CH31TOFF0 Position    */
#define PWM_PHCHGNXT_CH31TOFF0_Msk                 (1ul << PWM_PHCHGNXT_CH31TOFF0_Pos) /*!< PWM PHCHGNXT: CH31TOFF0 Mask        */

#define PWM_PHCHGNXT_CH21TOFF0_Pos                 26                                  /*!< PWM PHCHGNXT: CH21TOFF0 Position    */
#define PWM_PHCHGNXT_CH21TOFF0_Msk                 (1ul << PWM_PHCHGNXT_CH21TOFF0_Pos) /*!< PWM PHCHGNXT: CH21TOFF0 Mask        */

#define PWM_PHCHGNXT_CH11TOFF0_Pos                 25                                  /*!< PWM PHCHGNXT: CH11TOFF0 Position    */
#define PWM_PHCHGNXT_CH11TOFF0_Msk                 (1ul << PWM_PHCHGNXT_CH11TOFF0_Pos) /*!< PWM PHCHGNXT: CH11TOFF0 Mask        */

#define PWM_PHCHGNXT_CH01TOFF0_Pos                 24                                  /*!< PWM PHCHGNXT: CH01TOFF0 Position    */
#define PWM_PHCHGNXT_CH01TOFF0_Msk                 (1ul << PWM_PHCHGNXT_CH01TOFF0_Pos) /*!< PWM PHCHGNXT: CH01TOFF0 Mask        */

#define PWM_PHCHGNXT_CE1_Pos                       23                                  /*!< PWM PHCHGNXT: CE1 Position  */
#define PWM_PHCHGNXT_CE1_Msk                       (1ul << PWM_PHCHGNXT_CE1_Pos)       /*!< PWM PHCHGNXT: CE1 Mask      */

#define PWM_PHCHGNXT_T1_Pos                        22                                  /*!< PWM PHCHGNXT: T1 Position   */
#define PWM_PHCHGNXT_T1_Msk                        (1ul << PWM_PHCHGNXT_T1_Pos)        /*!< PWM PHCHGNXT: T1 Mask       */

#define PWM_PHCHGNXT_CMP1SEL_Pos                   20                                  /*!< PWM PHCHGNXT: CMP1SEL Position  */
#define PWM_PHCHGNXT_CMP1SEL_Msk                   (3ul << PWM_PHCHGNXT_CMP1SEL_Pos)   /*!< PWM PHCHGNXT: CMP1SEL Mask      */

#define PWM_PHCHGNXT_CH31TOFF1_Pos                 19                                  /*!< PWM PHCHGNXT: CH31TOFF1 Position    */
#define PWM_PHCHGNXT_CH31TOFF1_Msk                 (1ul << PWM_PHCHGNXT_CH31TOFF1_Pos) /*!< PWM PHCHGNXT: CH31TOFF1 Mask        */

#define PWM_PHCHGNXT_CH21TOFF1_Pos                 18                                  /*!< PWM PHCHGNXT: CH21TOFF1 Position    */
#define PWM_PHCHGNXT_CH21TOFF1_Msk                 (1ul << PWM_PHCHGNXT_CH21TOFF1_Pos) /*!< PWM PHCHGNXT: CH21TOFF1 Mask        */

#define PWM_PHCHGNXT_CH11TOFF1_Pos                 17                                  /*!< PWM PHCHGNXT: CH11TOFF1 Position    */
#define PWM_PHCHGNXT_CH11TOFF1_Msk                 (1ul << PWM_PHCHGNXT_CH11TOFF1_Pos) /*!< PWM PHCHGNXT: CH11TOFF1 Mask        */

#define PWM_PHCHGNXT_CH01TOFF1_Pos                 16                                  /*!< PWM PHCHGNXT: CH01TOFF1 Position    */
#define PWM_PHCHGNXT_CH01TOFF1_Msk                 (1ul << PWM_PHCHGNXT_CH01TOFF1_Pos) /*!< PWM PHCHGNXT: CH01TOFF1 Mask        */

#define PWM_PHCHGNXT_ACCNT1_Pos                    15                                  /*!< PWM PHCHGNXT: ACCNT1 Position   */
#define PWM_PHCHGNXT_ACCNT1_Msk                    (1ul << PWM_PHCHGNXT_ACCNT1_Pos)    /*!< PWM PHCHGNXT: ACCNT1 Mask       */

#define PWM_PHCHGNXT_ACCNT0_Pos                    14                                  /*!< PWM PHCHGNXT: ACCNT0 Position   */
#define PWM_PHCHGNXT_ACCNT0_Msk                    (1ul << PWM_PHCHGNXT_ACCNT0_Pos)    /*!< PWM PHCHGNXT: ACCNT0 Mask       */

#define PWM_PHCHGNXT_PWM5_Pos                      13                                  /*!< PWM PHCHGNXT: PWM5 Position */
#define PWM_PHCHGNXT_PWM5_Msk                      (1ul << PWM_PHCHGNXTPWM5_Pos)       /*!< PWM PHCHGNXT: PWM5 Mask     */

#define PWM_PHCHGNXT_PWM4_Pos                      12                                  /*!< PWM PHCHGNXT: PWM4 Position */
#define PWM_PHCHGNXT_PWM4_Msk                      (1ul << PWM_PHCHGNXT_PWM4_Pos)      /*!< PWM PHCHGNXT: PWM4 Mask     */

#define PWM_PHCHGNXT_PWM3_Pos                      11                                  /*!< PWM PHCHGNXT: PWM3 Position */
#define PWM_PHCHGNXT_PWM3_Msk                      (1ul << PWM_PHCHGNXT_PWM3_Pos)      /*!< PWM PHCHGNXT: PWM3 Mask     */

#define PWM_PHCHGNXT_PWM2_Pos                      10                                  /*!< PWM PHCHGNXT: PWM2 Position */
#define PWM_PHCHGNXT_PWM2_Msk                      (1ul << PWM_PHCHGNXT_PWM2_Pos)      /*!< PWM PHCHGNXT: PWM2 Mask     */

#define PWM_PHCHGNXT_PWM1_Pos                      9                                   /*!< PWM PHCHGNXT: PWM1 Position */
#define PWM_PHCHGNXT_PWM1_Msk                      (1ul << PWM_PHCHGNXT_PWM1_Pos)      /*!< PWM PHCHGNXT: PWM1 Mask     */

#define PWM_PHCHGNXT_PWM0_Pos                      8                                   /*!< PWM PHCHGNXT: PWM0 Position */
#define PWM_PHCHGNXT_PWM0_Msk                      (1ul << PWM_PHCHGNXT_PWM0_Pos)      /*!< PWM PHCHGNXT: PWM0 Mask     */

#define PWM_PHCHGNXT_D7_Pos                        7                                   /*!< PWM PHCHGNXT: D7 Position   */
#define PWM_PHCHGNXT_D7_Msk                        (1ul << PWM_PHCHGNXT_D7_Pos)        /*!< PWM PHCHGNXT: D7 Mask       */

#define PWM_PHCHGNXT_D6_Pos                        6                                   /*!< PWM PHCHGNXT: D6 Position   */
#define PWM_PHCHGNXT_D6_Msk                        (1ul << PWM_PHCHGNXT_D6_Pos)        /*!< PWM PHCHGNXT: D6 Mask       */

#define PWM_PHCHGNXT_D5_Pos                        5                                   /*!< PWM PHCHGNXT: D5 Position   */
#define PWM_PHCHGNXT_D5_Msk                        (1ul << PWM_PHCHGNXT_D5_Pos)        /*!< PWM PHCHGNXT: D5 Mask       */

#define PWM_PHCHGNXT_D4_Pos                        4                                   /*!< PWM PHCHGNXT: D4 Position   */
#define PWM_PHCHGNXT_D4_Msk                        (1ul << PWM_PHCHGNXT_D4_Pos)        /*!< PWM PHCHGNXT: D4 Mask       */

#define PWM_PHCHGNXT_D3_Pos                        3                                   /*!< PWM PHCHGNXT: D3 Position   */
#define PWM_PHCHGNXT_D3_Msk                        (1ul << PWM_PHCHGNXT_D3_Pos)        /*!< PWM PHCHGNXT: D3 Mask       */

#define PWM_PHCHGNXT_D2_Pos                        2                                   /*!< PWM PHCHGNXT: D2 Position   */
#define PWM_PHCHGNXT_D2_Msk                        (1ul << PWM_PHCHGNXT_D2_Pos)        /*!< PWM PHCHGNXT: D2 Mask       */

#define PWM_PHCHGNXT_D1_Pos                        1                                   /*!< PWM PHCHGNXT: D1 Position   */
#define PWM_PHCHGNXT_D1_Msk                        (1ul << PWM_PHCHGNXT_D1_Pos)        /*!< PWM PHCHGNXT: D1 Mask       */

#define PWM_PHCHGNXT_D0_Pos                        0                                   /*!< PWM PHCHGNXT: D0 Position   */
#define PWM_PHCHGNXT_D0_Msk                        (1ul << PWM_PHCHGNXT_D0_Pos)        /*!< PWM PHCHGNXT: D0 Mask       */

/* PWM PHCHGMASK Bit Field Definitions */
#define PWM_PHCHGMASK_CMPMASK_Pos                  8                                        /*!< PWM PHCHGMASK: CMPMASK Position        */
#define PWM_PHCHGMASK_CMPMASK_Msk                  (3ul << PWM_PHCHGMASK_CMPMASK_Pos)       /*!< PWM PHCHGMASK: CMPMASK Mask            */

#define PWM_PHCHGMASK_MASK7_Pos                    7                                        /*!< PWM PHCHGMASK: MASK7 Position          */
#define PWM_PHCHGMASK_MASK7_Msk                    (1ul << PWM_PHCHGMASK_MASK7_Pos)         /*!< PWM PHCHGMASK: MASK7 Mask              */

#define PWM_PHCHGMASK_MASK6_Pos                    6                                        /*!< PWM PHCHGMASK: MASK6 Position          */
#define PWM_PHCHGMASK_MASK6_Msk                    (1ul << PWM_PHCHGMASK_MASK6_Pos)         /*!< PWM PHCHGMASK: MASK6 Mask              */

/* PWM INTACCUCTL Bit Field Definitions */
#define PWM_INTACCUCTL_PERIODCNT_Pos               4                                        /*!< PWM INTACCUCTL: PERIODCNT Position     */
#define PWM_INTACCUCTL_PERIODCNT_Msk               (0xFul << PWM_INTACCUCTL_PERIODCNT_Pos)  /*!< PWM INTACCUCTL: PERIODCNT Mask         */

#define PWM_INTACCUCTL_INTACCUEN0_Pos              0                                        /*!< PWM INTACCUCTL: INTACCUEN0 Position    */
#define PWM_INTACCUCTL_INTACCUEN0_Msk              (1ul << PWM_INTACCUCTL_INTACCUEN0_Pos)   /*!< PWM INTACCUCTL: INTACCUEN0 Mask        */
/// @endcond /* HIDDEN_SYMBOLS */

/*@}*/ /* end of group PWM */


/*---------------------- Serial Peripheral Interface Controller -------------------------*/

/** @addtogroup SPI Serial Peripheral Interface Controller(SPI)
  Memory Mapped Structure for SPI Controller
  @{
 */

typedef struct
{
    __IO uint32_t CNTRL;          /*!< Offset: 0x0000  SPI Control and Status Register           */
    __IO uint32_t DIVIDER;        /*!< Offset: 0x0004  SPI Clock Divider Register                */
    __IO uint32_t SSR;            /*!< Offset: 0x0008  SPI Slave Select Register                 */
    uint32_t RESERVED0;      /*!< Offset: 0x000C  Reserved                                  */
    __I  uint32_t RX;             /*!< Offset: 0x0010  SPI Data Receive Register                 */
    uint32_t RESERVED1[3];   /*!< Offset: 0x0014~0x001C  Reserved                           */
    __O  uint32_t TX;             /*!< Offset: 0x0020  SPI Data Transmit Register                */
    uint32_t RESERVED2[6];   /*!< Offset: 0x0024 ~ 0x0038  Reserved                         */
    __IO uint32_t CNTRL2;         /*!< Offset: 0x003C  SPI Control and Status Register 2         */
    __IO uint32_t FIFO_CTL;       /*!< Offset: 0x0040  SPI FIFO Control Register                 */
    __IO uint32_t STATUS;         /*!< Offset: 0x0044  SPI Status Register                       */
} SPI_T;
/// @cond HIDDEN_SYMBOLS
/* SPI_CNTRL Bit Field Definitions */
#define SPI_CNTRL_TX_FULL_Pos      27                                     /*!< SPI CNTRL: TX_FULL Position      */
#define SPI_CNTRL_TX_FULL_Msk      (1ul << SPI_CNTRL_TX_FULL_Pos)         /*!< SPI CNTRL: TX_FULL Mask          */

#define SPI_CNTRL_TX_EMPTY_Pos     26                                     /*!< SPI CNTRL: TX_EMPTY Position     */
#define SPI_CNTRL_TX_EMPTY_Msk     (1ul << SPI_CNTRL_TX_EMPTY_Pos)        /*!< SPI CNTRL: TX_EMPTY Mask         */

#define SPI_CNTRL_RX_FULL_Pos      25                                     /*!< SPI CNTRL: RX_FULL Position      */
#define SPI_CNTRL_RX_FULL_Msk      (1ul << SPI_CNTRL_RX_FULL_Pos)         /*!< SPI CNTRL: RX_FULL Mask          */

#define SPI_CNTRL_RX_EMPTY_Pos     24                                     /*!< SPI CNTRL: RX_EMPTY Position     */
#define SPI_CNTRL_RX_EMPTY_Msk     (1ul << SPI_CNTRL_RX_EMPTY_Pos)        /*!< SPI CNTRL: RX_EMPTY Mask         */

#define SPI_CNTRL_FIFO_Pos         21                                     /*!< SPI CNTRL: FIFO Position         */
#define SPI_CNTRL_FIFO_Msk         (1ul << SPI_CNTRL_FIFO_Pos)            /*!< SPI CNTRL: FIFO Mask             */

#define SPI_CNTRL_REORDER_Pos      19                                     /*!< SPI CNTRL: REORDER Position      */
#define SPI_CNTRL_REORDER_Msk      (3ul << SPI_CNTRL_REORDER_Pos)         /*!< SPI CNTRL: REORDER Mask          */

#define SPI_CNTRL_SLAVE_Pos        18                                     /*!< SPI CNTRL: SLAVE Position        */
#define SPI_CNTRL_SLAVE_Msk        (1ul << SPI_CNTRL_SLAVE_Pos)           /*!< SPI CNTRL: SLAVE Mask            */

#define SPI_CNTRL_IE_Pos           17                                     /*!< SPI CNTRL: IE Position           */
#define SPI_CNTRL_IE_Msk           (1ul << SPI_CNTRL_IE_Pos)              /*!< SPI CNTRL: IE Mask               */

#define SPI_CNTRL_IF_Pos           16                                     /*!< SPI CNTRL: IF Position           */
#define SPI_CNTRL_IF_Msk           (1ul << SPI_CNTRL_IF_Pos)              /*!< SPI CNTRL: IF Mask               */

#define SPI_CNTRL_SP_CYCLE_Pos     12                                     /*!< SPI CNTRL: SP_CYCLE Position     */
#define SPI_CNTRL_SP_CYCLE_Msk     (0xFul << SPI_CNTRL_SP_CYCLE_Pos)      /*!< SPI CNTRL: SP_CYCLE Mask         */

#define SPI_CNTRL_CLKP_Pos         11                                     /*!< SPI CNTRL: CLKP Position         */
#define SPI_CNTRL_CLKP_Msk         (1ul << SPI_CNTRL_CLKP_Pos)            /*!< SPI CNTRL: CLKP Mask             */

#define SPI_CNTRL_LSB_Pos          10                                     /*!< SPI CNTRL: LSB Position          */
#define SPI_CNTRL_LSB_Msk          (1ul << SPI_CNTRL_LSB_Pos)             /*!< SPI CNTRL: LSB Mask              */

#define SPI_CNTRL_TX_BIT_LEN_Pos   3                                      /*!< SPI CNTRL: TX_BIT_LEN Position   */
#define SPI_CNTRL_TX_BIT_LEN_Msk   (0x1Ful << SPI_CNTRL_TX_BIT_LEN_Pos)   /*!< SPI CNTRL: TX_BIT_LEN Mask       */

#define SPI_CNTRL_TX_NEG_Pos       2                                      /*!< SPI CNTRL: TX_NEG Position       */
#define SPI_CNTRL_TX_NEG_Msk       (1ul << SPI_CNTRL_TX_NEG_Pos)          /*!< SPI CNTRL: TX_NEG Mask           */

#define SPI_CNTRL_RX_NEG_Pos       1                                      /*!< SPI CNTRL: RX_NEG Position       */
#define SPI_CNTRL_RX_NEG_Msk       (1ul << SPI_CNTRL_RX_NEG_Pos)          /*!< SPI CNTRL: RX_NEG Mask           */

#define SPI_CNTRL_GO_BUSY_Pos      0                                      /*!< SPI CNTRL: GO_BUSY Position      */
#define SPI_CNTRL_GO_BUSY_Msk      (1ul << SPI_CNTRL_GO_BUSY_Pos)         /*!< SPI CNTRL: GO_BUSY Mask          */

/* SPI_DIVIDER Bit Field Definitions */
#define SPI_DIVIDER_DIVIDER_Pos    0                                      /*!< SPI DIVIDER: DIVIDER Position    */
#define SPI_DIVIDER_DIVIDER_Msk    (0xFul << SPI_DIVIDER_DIVIDER_Pos)     /*!< SPI DIVIDER: DIVIDER Mask        */

/* SPI_SSR Bit Field Definitions */
#define SPI_SSR_LTRIG_FLAG_Pos     5                                 /*!< SPI SSR: LTRIG_FLAG Position  */
#define SPI_SSR_LTRIG_FLAG_Msk     (1ul << SPI_SSR_LTRIG_FLAG_Pos)   /*!< SPI SSR: LTRIG_FLAG Mask      */

#define SPI_SSR_SS_LTRIG_Pos       4                                 /*!< SPI SSR: SS_LTRIG Position    */
#define SPI_SSR_SS_LTRIG_Msk       (1ul << SPI_SSR_SS_LTRIG_Pos)     /*!< SPI SSR: SS_LTRIG Mask        */

#define SPI_SSR_AUTOSS_Pos         3                                 /*!< SPI SSR: AUTOSS Position      */
#define SPI_SSR_AUTOSS_Msk         (1ul << SPI_SSR_AUTOSS_Pos)       /*!< SPI SSR: AUTOSS Mask          */

#define SPI_SSR_SS_LVL_Pos         2                                 /*!< SPI SSR: SS_LVL Position      */
#define SPI_SSR_SS_LVL_Msk         (1ul << SPI_SSR_SS_LVL_Pos)       /*!< SPI SSR: SS_LVL Mask          */

#define SPI_SSR_SSR_Pos            0                                 /*!< SPI SSR: SSR Position         */
#define SPI_SSR_SSR_Msk            (1ul << SPI_SSR_SSR_Pos)          /*!< SPI SSR: SSR Mask             */

/* SPI_CNTRL2 Bit Field Definitions */
#define SPI_CNTRL2_BCn_Pos   31                                                      /*!< SPI CNTRL2: BCn Position          */
#define SPI_CNTRL2_BCn_Msk   (1ul << SPI_CNTRL2_BCn_Pos)                             /*!< SPI CNTRL2: BCn Mask              */

#define SPI_CNTRL2_SS_INT_OPT_Pos   16                                               /*!< SPI CNTRL2: SS_INT_OPT Position   */
#define SPI_CNTRL2_SS_INT_OPT_Msk   (1ul << SPI_CNTRL2_SS_INT_OPT_Pos)               /*!< SPI CNTRL2: SS_INT_OPT Mask       */

#define SPI_CNTRL2_SLV_START_INTSTS_Pos   11                                         /*!< SPI CNTRL2: SLV_START_INTSTS Position */
#define SPI_CNTRL2_SLV_START_INTSTS_Msk   (1ul << SPI_CNTRL2_SLV_START_INTSTS_Pos)   /*!< SPI CNTRL2: SLV_START_INTSTS Mask     */

#define SPI_CNTRL2_SSTA_INTEN_Pos   10                                               /*!< SPI CNTRL2: SSTA_INTEN Position   */
#define SPI_CNTRL2_SSTA_INTEN_Msk   (1ul << SPI_CNTRL2_SSTA_INTEN_Pos)               /*!< SPI CNTRL2: SSTA_INTEN Mask       */

#define SPI_CNTRL2_SLV_ABORT_Pos    9                                                /*!< SPI CNTRL2: SLV_ABORT Position    */
#define SPI_CNTRL2_SLV_ABORT_Msk    (1ul << SPI_CNTRL2_SLV_ABORT_Pos)                /*!< SPI CNTRL2: SLV_ABORT Mask        */

#define SPI_CNTRL2_NOSLVSEL_Pos     8                                                /*!< SPI CNTRL2: NOSLVSEL Position     */
#define SPI_CNTRL2_NOSLVSEL_Msk     (1ul << SPI_CNTRL2_NOSLVSEL_Pos)                 /*!< SPI CNTRL2: NOSLVSEL Mask         */

/* SPI_FIFO_CTL Bit Field Definitions */
#define SPI_FIFO_CTL_TX_THRESHOLD_Pos   28                                         /*!< SPI FIFO_CTL: TX_THRESHOLD Position */
#define SPI_FIFO_CTL_TX_THRESHOLD_Msk   (3ul << SPI_FIFO_CTL_TX_THRESHOLD_Pos)     /*!< SPI FIFO_CTL: TX_THRESHOLD Mask     */

#define SPI_FIFO_CTL_RX_THRESHOLD_Pos   24                                         /*!< SPI FIFO_CTL: RX_THRESHOLD Position */
#define SPI_FIFO_CTL_RX_THRESHOLD_Msk   (3ul << SPI_FIFO_CTL_RX_THRESHOLD_Pos)     /*!< SPI FIFO_CTL: RX_THRESHOLD Mask     */

#define SPI_FIFO_CTL_TIMEOUT_INTEN_Pos   21                                        /*!< SPI FIFO_CTL: TIMEOUT_INTEN Position    */
#define SPI_FIFO_CTL_TIMEOUT_INTEN_Msk   (1ul << SPI_FIFO_CTL_TIMEOUT_INTEN_Pos)   /*!< SPI FIFO_CTL: TIMEOUT_INTEN Mask        */

#define SPI_FIFO_CTL_RXOV_INTEN_Pos    6                                           /*!< SPI FIFO_CTL: RXOV_INTEN Position   */
#define SPI_FIFO_CTL_RXOV_INTEN_Msk    (1ul << SPI_FIFO_CTL_RXOV_INTEN_Pos)        /*!< SPI FIFO_CTL: RXOV_INTEN Mask       */

#define SPI_FIFO_CTL_TX_INTEN_Pos    3                                             /*!< SPI FIFO_CTL: TX_INTEN Position     */
#define SPI_FIFO_CTL_TX_INTEN_Msk    (1ul << SPI_FIFO_CTL_TX_INTEN_Pos)            /*!< SPI FIFO_CTL: TX_INTEN Mask         */

#define SPI_FIFO_CTL_RX_INTEN_Pos    2                                             /*!< SPI FIFO_CTL: RX_INTEN Position     */
#define SPI_FIFO_CTL_RX_INTEN_Msk    (1ul << SPI_FIFO_CTL_RX_INTEN_Pos)            /*!< SPI FIFO_CTL: RX_INTEN Mask         */

#define SPI_FIFO_CTL_TX_CLR_Pos     1                                              /*!< SPI FIFO_CTL: TX_CLR Position       */
#define SPI_FIFO_CTL_TX_CLR_Msk     (1ul << SPI_FIFO_CTL_TX_CLR_Pos)               /*!< SPI FIFO_CTL: TX_CLR Mask           */

#define SPI_FIFO_CTL_RX_CLR_Pos      0                                             /*!< SPI FIFO_CTL: RX_CLR Position       */
#define SPI_FIFO_CTL_RX_CLR_Msk      (1ul << SPI_FIFO_CTL_RX_CLR_Pos)              /*!< SPI FIFO_CTL: RX_CLR Mask           */

/* SPI_STATUS Bit Field Definitions */
#define SPI_STATUS_TX_FIFO_COUNT_Pos   28                                            /*!< SPI STATUS: TX_FIFO_COUNT Position    */
#define SPI_STATUS_TX_FIFO_COUNT_Msk   (0xFul << SPI_STATUS_TX_FIFO_COUNT_Pos)       /*!< SPI STATUS: TX_FIFO_COUNT Mask        */

#define SPI_STATUS_TX_FULL_Pos   27                                                  /*!< SPI STATUS: TX_FULL Position      */
#define SPI_STATUS_TX_FULL_Msk   (1ul << SPI_STATUS_TX_FULL_Pos)                     /*!< SPI STATUS: TX_FULL Mask          */

#define SPI_STATUS_TX_EMPTY_Pos   26                                                 /*!< SPI STATUS: TX_EMPTY Position     */
#define SPI_STATUS_TX_EMPTY_Msk   (1ul << SPI_STATUS_TX_EMPTY_Pos)                   /*!< SPI STATUS: TX_EMPTY Mask         */

#define SPI_STATUS_RX_FULL_Pos   25                                                  /*!< SPI STATUS: RX_FULL Position      */
#define SPI_STATUS_RX_FULL_Msk   (1ul << SPI_STATUS_RX_FULL_Pos)                     /*!< SPI STATUS: RX_FULL Mask          */

#define SPI_STATUS_RX_EMPTY_Pos   24                                                 /*!< SPI STATUS: RX_EMPTY Position     */
#define SPI_STATUS_RX_EMPTY_Msk   (1ul << SPI_STATUS_RX_EMPTY_Pos)                   /*!< SPI STATUS: RX_EMPTY Mask         */

#define SPI_STATUS_TIMEOUT_Pos   20                                                  /*!< SPI STATUS: TIMEOUT Position      */
#define SPI_STATUS_TIMEOUT_Msk   (1ul << SPI_STATUS_TIMEOUT_Pos)                     /*!< SPI STATUS: TIMEOUT Mask          */

#define SPI_STATUS_IF_Pos           16                                               /*!< SPI STATUS: IF Position           */
#define SPI_STATUS_IF_Msk           (1ul << SPI_STATUS_IF_Pos)                       /*!< SPI STATUS: IF Mask               */

#define SPI_STATUS_RX_FIFO_COUNT_Pos   12                                            /*!< SPI STATUS: RX_FIFO_COUNT Position    */
#define SPI_STATUS_RX_FIFO_COUNT_Msk   (0xFul << SPI_STATUS_RX_FIFO_COUNT_Pos)       /*!< SPI STATUS: RX_FIFO_COUNT Mask        */

#define SPI_STATUS_SLV_START_INTSTS_Pos   11                                         /*!< SPI STATUS: SLV_START_INTSTS Position */
#define SPI_STATUS_SLV_START_INTSTS_Msk   (1ul << SPI_STATUS_SLV_START_INTSTS_Pos)   /*!< SPI STATUS: SLV_START_INTSTS Mask     */

#define SPI_STATUS_TX_INTSTS_Pos    4                                                /*!< SPI STATUS: TX_INTSTS Position    */
#define SPI_STATUS_TX_INTSTS_Msk    (1ul << SPI_STATUS_TX_INTSTS_Pos)                /*!< SPI STATUS: TX_INTSTS Mask        */

#define SPI_STATUS_RX_OVERRUN_Pos    2                                               /*!< SPI STATUS: RX_OVERRUN Position   */
#define SPI_STATUS_RX_OVERRUN_Msk    (1ul << SPI_STATUS_RX_OVERRUN_Pos)              /*!< SPI STATUS: RX_OVERRUN Mask       */

#define SPI_STATUS_RX_INTSTS_Pos    0                                                /*!< SPI STATUS: RX_INTSTS Position    */
#define SPI_STATUS_RX_INTSTS_Msk    (1ul << SPI_STATUS_RX_INTSTS_Pos)                /*!< SPI STATUS: RX_INTSTS Mask        */
/// @endcond /* HIDDEN_SYMBOLS */
/*@}*/ /* end of group SPI */



/*---------------------------- System Controller -----------------------------*/

/** @addtogroup SYS System Manger Controller(SYS)
  Memory Mapped Structure for SYS Controller
  @{
 */
typedef struct
{
    /**
     * PDID
     * ===================================================================================================
     * Offset: 0x00  Part Device Identification Number Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDID      |This register reflects device part number code. S/W can read this register to identify which device is
     * |        |          |used.
     */
    __I  uint32_t  PDID;

    /**
     * RSTSRC
     * ===================================================================================================
     * Offset: 0x04  System Reset Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:8]  |Reserved  |Reserved
     * |[7]     |RSTS_CPU  |The RSTS_CPU flag is set by hardware if software writes CPU_RST (IPRSTC1[1]) 1 to reset Cortex-M0 CPU kernel and Flash memory controller (FMC).
     * |        |          |1 = The Cortex-M0 CPU kernel and FMC are reset by software setting CPU_RST to 1.
     * |        |          |0 = No reset from CPU
     * |        |          |Software can write 1 to clear this bit to zero.
     * |[6]     |Reserved  |Reserved
     * |[5]     |RSTS_MCU  |The RSTS_MCU flag is set by the "reset signal" from the MCU Cortex_M0 kernel to indicate the previous reset source.
     * |        |          |1= The MCU Cortex_M0 had issued the reset signal to reset the system by software writing 1 to bit SYSRESTREQ(AIRCR[2], Application Interrupt and Reset Control Register) in system control registers of Cortex_M0 kernel.
     * |        |          |0= No reset from MCU
     * |        |          |This bit is cleared by writing 1 to itself.
     * |[4]     |RSTS_BOD  |The RSTS_BOD flag is set by the "reset signal" from the Brown-Out Detector to indicate the previous reset source.
     * |        |          |1= The Brown-Out Detector module had issued the reset signal to reset the system.
     * |        |          |0= No reset from BOD
     * |        |          |Software can write 1 to clear this bit to zero.
     * |[3]     |Reserved  |Reserved
     * |[2]     |RSTS_WDT  |The RSTS_WDT flag is set by the "reset signal" from the Watchdog timer to indicate the previous reset source.
     * |        |          |1= The Watchdog timer had issued the reset signal to reset the system.
     * |        |          |0= No reset from Watchdog timer
     * |        |          |Software can write 1 to clear this bit to zero.
     * |[1]     |RSTS_RESET|The RSTS_RESET flag is set by the "reset signal" from the /RESET pin to indicate the previous reset source.
     * |        |          |1= The Pin /RESET had issued the reset signal to reset the system.
     * |        |          |0= No reset from Pin /RESET
     * |        |          |Software can write 1 to clear this bit to zero.
     * |[0]     |RSTS_POR  |The RSTS_POR flag is set by the "reset signal", which is from the Power-On Reset (POR) module or bit CHIP_RST (IPRSTC1[0]) is set, to indicate the previous reset source.
     * |        |          |1= The Power-On-Reset (POR) or CHIP_RST had issued the reset signal to reset the system.
     * |        |          |0= No reset from POR or CHIP_RST
     * |        |          |Software can write 1 to clear this bit to zero.
     */
    __IO uint32_t  RSTSRC;

    /**
     * IPRSTC1
     * ===================================================================================================
     * Offset: 0x08  Peripheral Reset Control Resister 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIP_RST  |CHIP one shot reset.
     * |        |          |Set this bit will reset the CHIP, including CPU kernel and all peripherals, and this bit will
     * |        |          |automatically return to "0" after the 2 clock cycles.
     * |        |          |The CHIP_RST is same as the POR reset , all the chip module is reset and the chip setting from
     * |        |          |flash are also reload
     * |        |          |This bit is the protected bit, program this need an open lock sequence, write "59h","16h","88h" to
     * |        |          |address 0x5000_0100 to un-lock this bit. Reference the register REGWRPROT at address SYS_BA
     * |        |          |+ 0x100
     * |        |          |0= Normal
     * |        |          |1= Reset CHIP
     * |[1]     |CPU_RST   |CPU kernel one shot reset.
     * |        |          |Set this bit will reset the Cortex-M0 CPU kernel and Flash memory controller (FMC). This bit will
     * |        |          |automatically return to "0" after the 2 clock cycles
     * |        |          |This bit is the protected bit, program this need an open lock sequence, write "59h","16h","88h" to
     * |        |          |address 0x5000_0100 to un-lock this bit. Reference the register REGWRPROT at address SYS_BA
     * |        |          |+ 0x100
     * |        |          |0= Normal
     * |        |          |1= Reset CPU
     */
    __IO uint32_t  IPRSTC1;

    /**
     * IPRSTC2
     * ===================================================================================================
     * Offset: 0x0C  Peripheral Reset Control Resister 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |GPIO_RST  |GPIO (P0~P4) controller Reset
     * |        |          |0= GPIO controller normal operation
     * |        |          |1= GPIO controller reset
     * |[2]     |TMR0_RST  |Timer0 controller Reset
     * |        |          |0= Timer0 controller normal operation
     * |        |          |1= Timer0 controller reset
     * |[3]     |TMR1_RST  |Timer1 controller Reset
     * |        |          |0= Timer1 controller normal operation
     * |        |          |1= Timer1 controller reset
     * |[8]     |I2C_RST   |I2C controller Reset
     * |        |          |0= I2C controller normal operation
     * |        |          |1= I2C controller reset
     * |[12]    |SPI_RST   |SPI controller Reset
     * |        |          |0= SPI controller normal operation
     * |        |          |1= SPI controller reset
     * |[16]    |UART_RST  |UART controller Reset
     * |        |          |0= UART controller Normal operation
     * |        |          |1= UART controller reset
     * |[20]    |PWM_RST   |PWM controller Reset
     * |        |          |0= PWM controller normal operation
     * |        |          |1= PWM controller reset
     * |[22]    |ACMP_RST  |ACMP controller Reset
     * |        |          |0= ACMP controller normal operation
     * |        |          |1= ACMP controller reset
     * |[28]    |ADC_RST   |ADC Controller Reset
     * |        |          |0= ADC controller normal operation
     * |        |          |1= ADC controller reset
     */
    __IO uint32_t  IPRSTC2;

    /**
     * RESERVED0
     * ===================================================================================================
     *
     * ---------------------------------------------------------------------------------------------------
     */
    uint32_t  RESERVED0[2];

    /**
     * BODCR
     * ===================================================================================================
     * Offset: 0x18  Brown-Out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:1]   |BOD_VL    |Brown Out Detector Threshold Voltage Selection (initiated & write-protected bit)
     * |        |          |The default value is set by flash controller user configuration register config0 bit[22:21]
     * |        |          |Brown out voltage
     * |        |          |11 = Disable 2.7V and 3.8V
     * |        |          |10 = 3.8V
     * |        |          |01 = 2.7V
     * |        |          |00 = Reserved
     * |[3]     |BOD_RSTEN |Brown Out Reset Enable (initiated & write-protected bit)
     * |        |          |1= Enable the Brown Out "RESET" function, when the Brown Out Detector function is enable
     * |        |          |and the detected voltage is lower than the threshold then assert a signal to reset the chip
     * |        |          |The default value is set by flash controller user configuration register config0 bit[20]
     * |        |          |0= Enable the Brown Out "INTERRUPT" function, when the Brown Out Detector function is
     * |        |          |enable and the detected voltage is lower than the threshold then assert a signal to interrupt
     * |        |          |the MCU Cortex-M0
     * |        |          |When the BOD_EN is enabled and the interrupt is assert, the interrupt will keep till to the
     * |        |          |BOD_EN set to "0". The interrupt for CPU can be blocked by disable the NVIC in CPU for BOD
     * |        |          |interrupt or disable the interrupt source by disable the BOD_EN and then re-enable the BOD_EN
     * |        |          |function if the BOD function is required
     * |[4]     |BOD_INTF  |Brown Out Detector Interrupt Flag
     * |        |          |1= When Brown Out Detector detects the VDD is dropped through the voltage of BOD_VL setting
     * |        |          |or the VDD is raised up through the voltage of BOD_VL setting, this bit is set to "1" and the
     * |        |          |brown out interrupt is requested if brown out interrupt is enabled.
     * |        |          |0= Brown Out Detector does not detect any voltage draft at VDD down through or up through the
     * |        |          |voltage of BOD_VL setting.
     * |[6]     |BOD_OUT   |The status for Brown Out Detector output state
     * |        |          |1= Brown Out Detector status output is 1, the detected voltage is lower than BOD_VL setting. If
     * |        |          |the BOD_EN is "0"(disabled), this bit always response "0"
     * |        |          |0= Brown Out Detector status output is 0, the detected voltage is higher than BOD_VL setting
     */
    __IO uint32_t  BODCTL;

    /**
     * RESERVED1
     * ===================================================================================================
     *
     * ---------------------------------------------------------------------------------------------------
     */
    uint32_t  RESERVED1[5];

    /**
     * P0_MFP
     * ===================================================================================================
     * Offset: 0x30  P0 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P0_MFP    |P0 multiple function Selection
     * |        |          |The pin function of P0 is depending on P0_MFP and P0_ALT.
     * |        |          |Refer to P0_ALT descriptions in detail.
     * |[8]     |P0_ALT0   |P0.0 alternate function Selection
     * |        |          |The pin function of P0.0 is depend on P0_MFP[0] and P0_ALT[0].
     * |        |          |P0_ALT[0]P0_MFP[0] = P0.0 Function
     * |        |          |00 = P0.0
     * |        |          |10 = CTS(UART)
     * |        |          |11 = TX(UART)
     * |[9]     |P0_ALT1   |P0.1 alternate function Selection
     * |        |          |The pin function of P0.1 is depend on P0_MFP[1] and P0_ALT[1].
     * |        |          |P0_ALT[1] P0_MFP[1] = P0.1 Function
     * |        |          |00 = P0.1
     * |        |          |01 = SPISS (SPI)
     * |        |          |10 = RTS(UART)
     * |        |          |11 = RX(UART)
     * |[12]    |P0_ALT4   |P0.4 alternate function Selection
     * |        |          |The pin function of P0.4 is depend on P0_MFP[4] and P0_ALT[4].
     * |        |          |P0_ALT[4] P0_MFP[4] = P0.4 Function
     * |        |          |00 = P0.4
     * |        |          |01 = Reserved
     * |        |          |10 = SPISS(SPI)
     * |        |          |11 = PWM5(PWM)
     * |[13]    |P0_ALT5   |P0.5 alternate function Selection
     * |        |          |The pin function of P0.5 is depend on P0_MFP[5] and P0_ALT[5].
     * |        |          |P0_ALT[5] P0_MFP[5] = P0.5 Function
     * |        |          |00 = P0.5
     * |        |          |01 = Reserved
     * |        |          |10 = MOSI(SPI)
     * |        |          |11 = Reserved
     * |[14]    |P0_ALT6   |P0.6 alternate function Selection
     * |        |          |The pin function of P0.6 is depend on P0_MFP[6] and P0_ALT[6].
     * |        |          |P0_ALT[6] P0_MFP[6] = P0.6 Function
     * |        |          |00 = P0.6
     * |        |          |01 = Reserved
     * |        |          |10 = MISO(SPI)
     * |        |          |11 = Reserved
     * |[15]    |P0_ALT7   |P0.7 alternate function Selection
     * |        |          |The pin function of P0.7 is depend on P0_MFP[7] and P0_ALT[7].
     * |        |          |P0_ALT[7] P0_MFP[7] = P0.7 Function
     * |        |          |00 = P0.7
     * |        |          |01 = Reserved
     * |        |          |10 = SPICLK(SPI)
     * |        |          |11 = Reserved
     * |[23:16] |P0_TYPEn  |P0[7:0] input Schmitt Trigger function Enable
     * |        |          |1= P0[7:0] I/O input Schmitt Trigger function enable
     * |        |          |0= P0[7:0] I/O input Schmitt Trigger function disable
     */
    __IO uint32_t  P0_MFP;

    /**
     * P1_MFP
     * ===================================================================================================
     * Offset: 0x34  P1 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P1_MFP    |P1 multiple function Selection
     * |        |          |The pin function of P1 is depending on P1_MFP and P1_ALT.
     * |        |          |Refer to P1_ALT descriptions in detail.
     * |[8]     |P1_ALT0   |P1.0 alternate function Selection
     * |        |          |The pin function of P1.0 is depend on P1_MFP[0] and P1_ALT[0].
     * |        |          |P1_ALT[0] P1_MFP[0] = P1.0 Function
     * |        |          |00 = P1.0
     * |        |          |01 = AIN1(ADC)
     * |        |          |10 = Reserved
     * |        |          |11 = CPP0 (ACMP)
     * |[10]    |P1_ALT2   |P1.2 alternate function Selection
     * |        |          |The pin function of P1.2 is depend on P1_MFP[2] and P1_ALT[2].
     * |        |          |P1_ALT[2] P1_MFP[2] = P1.2 Function
     * |        |          |00 = P1.2
     * |        |          |01 = AIN2(ADC)
     * |        |          |10 = RX(UART)
     * |        |          |11 = CPP0 (ACMP)
     * |[11]    |P1_ALT3   |P1.3 alternate function Selection
     * |        |          |The pin function of P1.3 is depend on P1_MFP[3] and P1_ALT[3].
     * |        |          |P1_ALT[3] P1_MFP[3] = P1.3 Function
     * |        |          |00 = P1.3
     * |        |          |01 = AIN3(ADC)
     * |        |          |10 = TX(UART)
     * |        |          |11 = CPP0 (ACMP)
     * |[12]    |P1_ALT4   |P1.4 alternate function Selection
     * |        |          |The pin function of P1.4 is depend on P1_MFP[4] and P1_ALT[4].
     * |        |          |P1_ALT[4] P1_MFP[4] = P1.4 Function
     * |        |          |00 = P1.4
     * |        |          |01 = AIN4(ADC)
     * |        |          |10 = Reserved
     * |        |          |11 = CPN0 (CMP)
     * |[13]    |P1_ALT5   |P1.5 alternate function Selection
     * |        |          |The pin function of P1.5 is depend on P1_MFP[5] and P1_ALT[5].
     * |        |          |P1_ALT[5] P1_MFP[5] = P1.5 Function
     * |        |          |00 = P1.5
     * |        |          |01 = AIN5(ADC)
     * |        |          |10 = Reserved
     * |        |          |11 = CPP0 (CMP)
     * |[23:16] |P1_TYPEn  |P1[7:0] input Schmitt Trigger function Enable
     * |        |          |1= P1[7:0] I/O input Schmitt Trigger function enable
     * |        |          |0= P1[7:0] I/O input Schmitt Trigger function disable
     */
    __IO uint32_t  P1_MFP;

    /**
     * P2_MFP
     * ===================================================================================================
     * Offset: 0x38  P2 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P2_MFP    |P2 multiple function Selection
     * |        |          |The pin function of P2 is depending on P2_MFP and P2_ALT.
     * |        |          |Refer to P2_ALT descriptions in detail.
     * |[10]    |P2_ALT2   |P2.2 alternate function Selection
     * |        |          |The pin function of P2.2 is depend on P2_MFP[2] and P2_ALT[2].
     * |        |          |P2_ALT[2] P2_MFP[2] = P2.2 Function
     * |        |          |00 = P2.2
     * |        |          |01 = Reserved
     * |        |          |10 = PWM0(PWM)
     * |        |          |11 = Reserved
     * |[11]    |P2_ALT3   |P2.3 alternate function Selection
     * |        |          |The pin function of P2.3 is depend on P2_MFP[3] and P2_ALT[3].
     * |        |          |P2_ALT[3] P2_MFP[3] = P2.3 Function
     * |        |          |00 = P2.3
     * |        |          |01 = Reserved
     * |        |          |10 = PWM1(PWM)
     * |        |          |11 = Reserved
     * |[12]    |P2_ALT4   |P2.4 alternate function Selection
     * |        |          |The pin function of P2.4 is depend on P2_MFP[4] and P2_ALT[4].
     * |        |          |P2_ALT[4] P2_MFP[4] = P0.4 Function
     * |        |          |00 = P2.4
     * |        |          |01 = Reserved
     * |        |          |10 = PWM2(PWM)
     * |        |          |11 = Reserved
     * |[13]    |P2_ALT5   |P2.5 alternate function Selection
     * |        |          |The pin function of P2.5 is depend on P2_MFP[5] and P2_ALT[5].
     * |        |          |P2_ALT[5] P2_MFP[5] = P2.5 Function
     * |        |          |00 = P2.5
     * |        |          |01 = Reserved
     * |        |          |10 = PWM3(PWM)
     * |        |          |11 = Reserved
     * |[14]    |P2_ALT6   |P2.6 alternate function Selection
     * |        |          |The pin function of P2.6 is depend on P2_MFP[6] and P2_ALT[6].
     * |        |          |P2_ALT[6] P2_MFP[6] = P2.6 Function
     * |        |          |00 = P2.6
     * |        |          |01 = Reserved
     * |        |          |10 = PWM4(PWM)
     * |        |          |11 = CPO1
     * |[23:16] |P2_TYPEn  |P2[7:0] input Schmitt Trigger function Enable
     * |        |          |1= P2[7:0] I/O input Schmitt Trigger function enable
     * |        |          |0= P2[7:0] I/O input Schmitt Trigger function disable
     */
    __IO uint32_t  P2_MFP;

    /**
     * P3_MFP
     * ===================================================================================================
     * Offset: 0x3C  P3 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P3_MFP    |P3 multiple function Selection
     * |        |          |The pin function of P3 is depending on P3_MFP and P3_ALT.
     * |        |          |Refer to P3_ALT descriptions in detail.
     * |[8]     |P3_ALT0   |P3.0 alternate function Selection
     * |        |          |The pin function of P3.0 is depend on P3_MFP[0] and P3_ALT[0].
     * |        |          |P3_ALT[0] P3_MFP[0] = P3.0 Function
     * |        |          |00 = P3.0
     * |        |          |01 = Reserved
     * |        |          |10 = CPN1
     * |        |          |11 = AIN6(ADC)
     * |[9]     |P3_ALT1   |P3.1 alternate function Selection
     * |        |          |The pin function of P3.1 is depend on P3_MFP[1] and P3_ALT[1].
     * |        |          |P3_ALT[1] P3_MFP[1] = P3.1 Function
     * |        |          |00 = P3.1
     * |        |          |01 = Reserved
     * |        |          |10 = CPP1
     * |        |          |11 = AIN7(ADC)
     * |[10]    |P3_ALT2   |P3.2 alternate function Selection
     * |        |          |The pin function of P3.2 is depend on P3_MFP[2] and P3_ALT[2].
     * |        |          |P3_ALT[2] P3_MFP[2] = P3.2 Function
     * |        |          |00 = P3.2
     * |        |          |01 = /INT0
     * |        |          |10 = T0EX
     * |        |          |11 = STADC(ADC)
     * |[12]    |P3_ALT4   |P3.4 alternate function Selection
     * |        |          |The pin function of P3.4 is depend on P3_MFP[4] and P3_ALT[4].
     * |        |          |P3_ALT[4] P3_MFP[4] = P3.4 Function
     * |        |          |00 = P3.4
     * |        |          |01 = T0(Timer0)
     * |        |          |10 = SDA(I2C)
     * |        |          |11 = CPP1(ACMP)
     * |[13]    |P3_ALT5   |P3.5 alternate function Selection
     * |        |          |The pin function of P3.5 is depend on P3_MFP[5] and P3_ALT[5].
     * |        |          |P3_ALT[5] P3_MFP[5] = P3.5 Function
     * |        |          |00 = P3.5
     * |        |          |01 = T1(Timer1)
     * |        |          |10 = SCL(I2C)
     * |        |          |11 = CPP1(ACMP)
     * |[14]    |P3_ALT6   |P3.6 alternate function Selection
     * |        |          |The pin function of P3.6 is depend on P3_MFP[6] and P3_ALT[6].
     * |        |          |P3_ALT[6] P3_MFP[6] = P3.6 Function
     * |        |          |00 = P3.6
     * |        |          |01 = T1EX
     * |        |          |10 = CKO(Clock Driver output)
     * |        |          |11 = CPO0(CMP)
     * |[23:16] |P3_TYPEn  |P3[7:0] input Schmitt Trigger function Enable
     * |        |          |1= P3[7:0] I/O input Schmitt Trigger function enable
     * |        |          |0= P3[7:0] I/O input Schmitt Trigger function disable
     * |[24]    |P32CPP1   |P3.2 Alternate Function Selection Extension
     * |        |          |0 = P3.2 is set by P3_ALT[2] and P3_MFP[2]
     * |        |          |1 = P3.2 is set to CPP1 of ACMP1
     */
    __IO uint32_t  P3_MFP;

    /**
     * P4_MFP
     * ===================================================================================================
     * Offset: 0x40  P4 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P4_MFP    |P4 multiple function Selection
     * |        |          |The pin function of P4 is depending on P4_MFP and P4_ALT.
     * |        |          |Refer to P4_ALT descriptions in detail.
     * |[14]    |P4_ALT6   |P4.6 alternate function Selection
     * |        |          |The pin function of P4.6 is depend on P4_MFP[6] and P4_ALT[6].
     * |        |          |P4_ALT[6] P4_MFP[6] = P4.6 Function
     * |        |          |00 = P4.6
     * |        |          |01 = ICE_CLK(ICE)
     * |        |          |1x = Reserved
     * |[15]    |P4_ALT7   |P4.7 alternate function Selection
     * |        |          |The pin function of P4.7 is depend on P4_MFP[7] and P4_ALT[7].
     * |        |          |P4_ALT[7] P4_MFP[7] = P4.7 Function
     * |        |          |00 = P4.7
     * |        |          |01 = ICE_DAT(ICE)
     * |        |          |1x = Reserved
     * |[23:16] |P4_TYPEn  |P4[7:0] input Schmitt Trigger function Enable
     * |        |          |1= P4[7:0] I/O input Schmitt Trigger function enable
     * |        |          |0= P4[7:0] I/O input Schmitt Trigger function disable
     */
    __IO uint32_t  P4_MFP;

    /**
     * P5_MFP
     * ===================================================================================================
     * Offset: 0x44  P5 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P5_MFP    |P5 multiple function Selection
     * |        |          |The pin function of P5 is depending on P5_MFP and P5_ALT.
     * |        |          |Refer to P5_ALT descriptions in detail.
     * |[8]     |P5_ALT0   |P5.0 alternate function Selection
     * |        |          |The pin function of P5.0 is depend on P5_MFP[0] and P5_ALT[0].
     * |        |          |P5_ALT[0] P5_MFP[0] = P5.0 Function
     * |        |          |00 = P5.0
     * |        |          |01 = XTAL1
     * |        |          |1x = Reserved
     * |[9]     |P5_ALT1   |P5.1 alternate function Selection
     * |        |          |The pin function of P5.1 is depend on P5_MFP[1] and P5_ALT[1].
     * |        |          |P5_ALT[1] P5_MFP[1] = P5.1 Function
     * |        |          |00 = P5.1
     * |        |          |01 = XTAL2
     * |        |          |1x = Reserved
     * |[10]    |P5_ALT2   |P5.2 alternate function Selection
     * |        |          |The pin function of P5.2 is depend on P5_MFP[2] and P5_ALT[2].
     * |        |          |P5_ALT[2] P5_MFP[2] = P5.2 Function
     * |        |          |00 = P5.2
     * |        |          |01 = /INT1
     * |        |          |1x = Reserved
     * |[11]    |P5_ALT3   |P5.3 alternate function Selection
     * |        |          |The pin function of P5.3 is depend on P5_MFP[3] and P5_ALT[3].
     * |        |          |P5_ALT[3] P5_MFP[3] = P5.3 Function
     * |        |          |00 = P5.3
     * |        |          |01 = AIN0(ADC)
     * |        |          |1x = Reserved
     * |[12]    |P5_ALT4   |P5.4 alternate function Selection
     * |        |          |The pin function of P5.4 is depend on P5_MFP[4] and P5_ALT[4].
     * |        |          |P5_ALT[4] P5_MFP[4] = P5.4 Function
     * |        |          |00 = P5.4
     * |        |          |01 = Reserved
     * |        |          |1x = Reserved
     * |[13]    |P5_ALT5   |P5.5 alternate function Selection
     * |        |          |The pin function of P5.5 is depend on P5_MFP[5] and P5_ALT[5].
     * |        |          |P5_ALT[5] P5_MFP[5] = P5.5 Function
     * |        |          |00 = P5.5
     * |        |          |01 = Reserved
     * |        |          |1x = Reserved
     * |[23:16] |P5_TYPEn  |P5[7:0] input Schmitt Trigger function Enable
     * |        |          |1= P5[7:0] I/O input Schmitt Trigger function enable
     * |        |          |0= P5[7:0] I/O input Schmitt Trigger function disable
     */
    __IO uint32_t  P5_MFP;

    /**
     * RESERVED3
     * ===================================================================================================
     *
     * ---------------------------------------------------------------------------------------------------
     */
    uint32_t  RESERVED3[14];

    /**
     * IRCTRIMCTL
     * ===================================================================================================
     * Offset: 0x80  HIRC Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TRIM_SEL  |Trim Frequency Selection
     * |        |          |This bit is to enable the HIRC auto trim.
     * |        |          |When setting this bit to "1", the HFIRC auto trim function will trim HIRC to 22.1184 MHz
     * |        |          |automatically based on the 32.768 KHz reference clock.
     * |        |          |During auto trim operation, if 32.768 KHz clock error is detected or trim retry limitation
     * |        |          |count reached, this field will be cleared to "0" automatically.
     * |        |          |0 = HIRC auto trim function Disabled.
     * |        |          |1 = HIRC auto trim function Enabled and HIRC trimmed to 22.1184 MHz.
     * |[5:4]   |TRIM_LOOP |Trim Calculation Loop
     * |        |          |This field defines trim value calculation based on the number of 32.768 KHz clock.
     * |        |          |This field also defines how many times the auto trim circuit will try to update the
     * |        |          |HIRC trim value before the frequency of HIRC is locked.
     * |        |          |Once the HIRC is locked, the internal trim value update counter will be reset
     * |        |          |If the trim value update counter reaches this limitation value and frequency of HIRC
     * |        |          |is still not locked, the auto trim operation will be disabled and TRIM_SEL will be cleared to "0".
     * |        |          |00 = Trim value calculation is based on average difference in 4 32.768 KHz clock and trim retry count limitation is 64.
     * |        |          |01 = Trim value calculation is based on average difference in 8 32.768 KHz clock and trim retry count limitation is 128.
     * |        |          |10 = Trim value calculation is based on average difference in 16 32.768 KHz clock and trim retry count limitation is 256.
     * |        |          |11 = Trim value calculation is based on average difference in 32 32.768 KHz clock and trim retry count limitation is 512.
     */
    __IO uint32_t  IRCTRIMCTL;

    /**
     * IRCTRIMIER
     * ===================================================================================================
     * Offset: 0x84  HIRC Trim Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field         |Descriptions
     * | :----: | :----:       | :---- |
     * |[1]     |TRIM_FAIL_IEN |Trim Failure Interrupt Enable
     * |        |              |This bit controls if an interrupt will be triggered while HIRC trim value update limitation
     * |        |              |count is reached and HIRC frequency is still not locked on target frequency set by TRIM_SEL.
     * |        |              |If this bit is high and TRIM_FAIL_INT is set during auto trim operation, an interrupt will be
     * |        |              |triggered to notify that HFIRC trim value update limitation count is reached.
     * |        |              |0 = TRIM_FAIL_INT status Disabled to trigger an interrupt to CPU.
     * |        |              |1 = TRIM_FAIL_INT status Enabled to trigger an interrupt to CPU.
     * |[2]     |32K_ERR_IEN   |32.768 KHz Clock Error Interrupt Enable
     * |        |              |This bit controls if CPU could get an interrupt while 32.768 KHz clock is inaccurate during
     * |        |              |auto trim operation.
     * |        |              |If this bit is high, and 32K_ERR_INT is set during auto trim operation, an interrupt will be triggered
     * |        |              |to notify the 32.768 KHz clock frequency is inaccurate.
     * |        |              |0 = 32K_ERR_INT status Disabled to trigger an interrupt to CPU.
     * |        |              |1 = 32K_ERR_INT status Enabled to trigger an interrupt to CPU.
     */
    __IO uint32_t  IRCTRIMIER;

    /**
     * IRCTRIMISR
     * ===================================================================================================
     * Offset: 0x88  HIRC Trim Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field         |Descriptions
     * | :----: | :----:       | :---- |
     * |[0]     |FREQ_LOCK     |HIRC Frequency Lock Status
     * |        |              |1 = This bit indicates the HIRC frequency lock.
     * |[1]     |TRIM_FAIL_INT |Trim Failure Interrupt Status
     * |        |              |This bit indicates that HIRC trim value update limitation count reached and HIRC
     * |        |              |clock frequency still doesn't lock. Once this bit is set, the auto trim operation
     * |        |              |stopped and TRIM_SEL will be cleared to "0" by hardware automatically.
     * |        |              |If this bit is set and TRIM_FAIL_IEN is high, an interrupt will be triggered to notify
     * |        |              |that HIRC trim value update limitation count was reached. Write "1" to clear this to zero.
     * |        |              |0 = Trim value update limitation count is not reached.
     * |        |              |1 = Trim value update limitation count is reached and HFIRC frequency is still not locked.
     * |[2]     |32K_ERR_INT   |32.768 KHz Clock Error Interrupt Status
     * |        |              |This bit indicates that 32.768 KHz clock frequency is inaccuracy. Once this bit is set, the
     * |        |              |auto trim operation stopped and TRIM_SEL will be cleared to "0" by hardware automatically.
     * |        |              |If this bit is set and 32K_ERR_IEN is high, an interrupt will be triggered to notify the
     * |        |              |32.768 KHz clock frequency is inaccuracy. Write "1" to clear this to zero.
     * |        |              |0 = 32.768 KHz clock frequency is accuracy.
     * |        |              |1 = 32.768 KHz clock frequency is inaccuracy.
     */
    __IO uint32_t  IRCTRIMISR;

    /**
     * RESERVED4
     * ===================================================================================================
     *
     * ---------------------------------------------------------------------------------------------------
     */
    uint32_t  RESERVED4[29];

    /**
     * RegLockAddr
     * ===================================================================================================
     * Offset: 0x100 Register Lock Key Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RegUnLock |Register Write-Protected Disable index (Read only)
     * |        |          |1 = Protected registers are Unlock.
     * |        |          |0 = Protected registers are locked. Any write to the target register is ignored.
     * |        |          |The Protected registers are:
     * |        |          |IPRSTC1 0x5000_0008 None
     * |        |          |BODCR 0x5000_0018 None
     * |        |          |LDOCR 0x5000_001C None
     * |        |          |PORCR 0x5000_0024 None
     * |        |          |PWRCON 0x5000_0200 bit[6] is not protected for power, wake-up interrupt clear
     * |        |          |APBCLK bit[0] 0x5000_0208 bit[0] is watch dog clock enable
     * |        |          |CLKSEL0 0x5000_0210 HCLK and CPU STCLK clock source select
     * |        |          |CLK_SEL1 bit[1:0] 0x5000_0214 Watch dog clock source select
     * |        |          |ISPCON 0x5000_C000 Flash ISP Control register
     * |        |          |WTCR 0x4000_4000 None
     * |        |          |NMI_SEL[8] - address 0x5000_380 (NMI interrupt source enable)
     */
    __IO uint32_t  RegLockAddr;
} SYS_T;
/// @cond HIDDEN_SYMBOLS
/* SYS RSTSRC Bit Field Definitions */
#define SYS_RSTSRC_RSTS_CPU_Pos                 7                                       /*!< SYS RSTSRC: RSTS_CPU Position  */
#define SYS_RSTSRC_RSTS_CPU_Msk                 (1ul << SYS_RSTSRC_RSTS_CPU_Pos)        /*!< SYS RSTSRC: RSTS_CPU Mask      */

#define SYS_RSTSRC_RSTS_MCU_Pos                 5                                       /*!< SYS RSTSRC: RSTS_MCU Position  */
#define SYS_RSTSRC_RSTS_MCU_Msk                 (1ul << SYS_RSTSRC_RSTS_MCU_Pos)        /*!< SYS RSTSRC: RSTS_MCU Mask      */

#define SYS_RSTSRC_RSTS_BOD_Pos                 4                                       /*!< SYS RSTSRC: RSTS_BOD Position  */
#define SYS_RSTSRC_RSTS_BOD_Msk                 (1ul << SYS_RSTSRC_RSTS_BOD_Pos)        /*!< SYS RSTSRC: RSTS_BOD Mask      */

#define SYS_RSTSRC_RSTS_WDT_Pos                 2                                       /*!< SYS RSTSRC: RSTS_WDT Position  */
#define SYS_RSTSRC_RSTS_WDT_Msk                 (1ul << SYS_RSTSRC_RSTS_WDT_Pos)        /*!< SYS RSTSRC: RSTS_WDT Mask      */

#define SYS_RSTSRC_RSTS_RESET_Pos               1                                       /*!< SYS RSTSRC: RSTS_RESET Position    */
#define SYS_RSTSRC_RSTS_RESET_Msk               (1ul << SYS_RSTSRC_RSTS_RESET_Pos)      /*!< SYS RSTSRC: RSTS_RESET Mask        */

#define SYS_RSTSRC_RSTS_POR_Pos                 0                                       /*!< SYS RSTSRC: RSTS_POR Position  */
#define SYS_RSTSRC_RSTS_POR_Msk                 (1ul << SYS_RSTSRC_RSTS_POR_Pos)        /*!< SYS RSTSRC: RSTS_POR Mask      */

/* SYS IPRSTC1 Bit Field Definitions */
#define SYS_IPRSTC1_CPU_RST_Pos                 1                                       /*!< SYS IPRSTC1: CPU_RST Position  */
#define SYS_IPRSTC1_CPU_RST_Msk                 (1ul << SYS_IPRSTC1_CPU_RST_Pos)        /*!< SYS IPRSTC1: CPU_RST Mask      */

#define SYS_IPRSTC1_CHIP_RST_Pos                0                                       /*!< SYS IPRSTC1: CHIP_RST Position */
#define SYS_IPRSTC1_CHIP_RST_Msk                (1ul << SYS_IPRSTC1_CHIP_RST_Pos)       /*!< SYS IPRSTC1: CHIP_RST Mask     */

/* SYS IPRSTC2 Bit Field Definitions */
#define SYS_IPRSTC2_ADC_RST_Pos                 28                                      /*!< SYS IPRSTC2: ADC_RST Position  */
#define SYS_IPRSTC2_ADC_RST_Msk                 (1ul << SYS_IPRSTC2_ADC_RST_Pos)        /*!< SYS IPRSTC2: ADC_RST Mask      */

#define SYS_IPRSTC2_ACMP_RST_Pos                22                                      /*!< SYS IPRSTC2: ACMP_RST Position */
#define SYS_IPRSTC2_ACMP_RST_Msk                (1ul << SYS_IPRSTC2_ACMP_RST_Pos)       /*!< SYS IPRSTC2: ACMP_RST Mask     */

#define SYS_IPRSTC2_PWM_RST_Pos                 20                                      /*!< SYS IPRSTC2: PWM_RST Position  */
#define SYS_IPRSTC2_PWM_RST_Msk                 (1ul << SYS_IPRSTC2_PWM_RST_Pos)        /*!< SYS IPRSTC2: PWM_RST Mask      */

#define SYS_IPRSTC2_UART_RST_Pos                16                                      /*!< SYS IPRSTC2: UART_RST Position */
#define SYS_IPRSTC2_UART_RST_Msk                (1ul << SYS_IPRSTC2_UART_RST_Pos)       /*!< SYS IPRSTC2: UART_RST Mask     */

#define SYS_IPRSTC2_SPI_RST_Pos                 12                                      /*!< SYS IPRSTC2: SPI_RST Position  */
#define SYS_IPRSTC2_SPI_RST_Msk                 (1ul << SYS_IPRSTC2_SPI_RST_Pos)        /*!< SYS IPRSTC2: SPI_RST Mask      */

#define SYS_IPRSTC2_I2C_RST_Pos                 8                                       /*!< SYS IPRSTC2: I2C_RST Position  */
#define SYS_IPRSTC2_I2C_RST_Msk                 (1ul << SYS_IPRSTC2_I2C_RST_Pos)        /*!< SYS IPRSTC2: I2C_RST Mask      */

#define SYS_IPRSTC2_TMR1_RST_Pos                3                                       /*!< SYS IPRSTC2: TMR1_RST Position */
#define SYS_IPRSTC2_TMR1_RST_Msk                (1ul << SYS_IPRSTC2_TMR1_RST_Pos)       /*!< SYS IPRSTC2: TMR1_RST Mask     */

#define SYS_IPRSTC2_TMR0_RST_Pos                2                                       /*!< SYS IPRSTC2: TMR0_RST Position */
#define SYS_IPRSTC2_TMR0_RST_Msk                (1ul << SYS_IPRSTC2_TMR0_RST_Pos)       /*!< SYS IPRSTC2: TMR0_RST Mask     */

#define SYS_IPRSTC2_GPIO_RST_Pos                1                                       /*!< SYS IPRSTC2: GPIO_RST Position */
#define SYS_IPRSTC2_GPIO_RST_Msk                (1ul << SYS_IPRSTC2_GPIO_RST_Pos)       /*!< SYS IPRSTC2: GPIO_RST Mask     */

/* SYS BODCR Bit Field Definitions */
#define SYS_BODCR_BOD_OUT_Pos                   6                                       /*!< SYS BODCR: BOD_OUT Position    */
#define SYS_BODCR_BOD_OUT_Msk                   (1ul << SYS_BODCR_BOD_OUT_Pos)          /*!< SYS BODCR: BOD_OUT Mask        */

#define SYS_BODCR_BOD_LPM_Pos                  5                                        /*!< SYS BODCR: BOD_LPM Position    */
#define SYS_BODCR_BOD_LPM_Msk                  (1ul << SYS_BODCR_BOD_LPM_Pos)           /*!< SYS BODCR: BOD_LPM Mask        */

#define SYS_BODCR_BOD_INTF_Pos                  4                                       /*!< SYS BODCR: BOD_INTF Position   */
#define SYS_BODCR_BOD_INTF_Msk                  (1ul << SYS_BODCR_BOD_INTF_Pos)         /*!< SYS BODCR: BOD_INTF Mask       */

#define SYS_BODCR_BOD_RSTEN_Pos                 3                                       /*!< SYS BODCR: BOD_RSTEN Position  */
#define SYS_BODCR_BOD_RSTEN_Msk                 (1ul << SYS_BODCR_BOD_RSTEN_Pos)        /*!< SYS BODCR: BOD_RSTEN Mask      */

#define SYS_BODCR_BOD_VL_Pos                    1                                       /*!< SYS BODCR: BOD_VL Position     */
#define SYS_BODCR_BOD_VL_Msk                    (3ul << SYS_BODCR_BOD_VL_Pos)           /*!< SYS BODCR: BOD_VL Mask         */

#define SYS_BODCR_BOD_VL_EXT_Pos                0                                       /*!< SYS BODCR: BOD_VL_EXT Position */
#define SYS_BODCR_BOD_VL_EXT_Msk                (1ul << SYS_BODCR_BOD_VL_EXT_Pos)       /*!< SYS BODCR: BOD_VL_EXT Mask     */

/* SYS P0_MFP Bit Field Definitions */
#define SYS_P0_MFP_P0_TYPE_Pos                  16                                      /*!< SYS P0_MFP: P0_TYPE Position   */
#define SYS_P0_MFP_P0_TYPE_Msk                  (0xFFul << SYS_P0_MFP_P0_TYPE_Pos)      /*!< SYS P0_MFP: P0_TYPE Mask       */

#define SYS_P0_MFP_P0_ALT_Pos                   8                                       /*!< SYS P0_MFP: P0_ALT Position    */
#define SYS_P0_MFP_P0_ALT_Msk                   (0xFFul << SYS_P0_MFP_P0_ALT_Pos)       /*!< SYS P0_MFP: P0_ALT Mask        */

#define SYS_P0_MFP_P0_MFP_Pos                   0                                       /*!< SYS P0_MFP: P0_MFP Position    */
#define SYS_P0_MFP_P0_MFP_Msk                   (0xFFul << SYS_P0_MFP_P0_MFP_Pos)       /*!< SYS P0_MFP: P0_MFP Mask        */

/* SYS P1_MFP Bit Field Definitions */
#define SYS_P1_MFP_P1_TYPE_Pos                  16                                      /*!< SYS P1_MFP: P1_TYPE Position   */
#define SYS_P1_MFP_P1_TYPE_Msk                  (0xFFul << SYS_P1_MFP_P1_TYPE_Pos)      /*!< SYS P1_MFP: P1_TYPE Mask       */

#define SYS_P1_MFP_P1_ALT_Pos                   8                                       /*!< SYS P1_MFP: P1_ALT Position    */
#define SYS_P1_MFP_P1_ALT_Msk                   (0xFFul << SYS_P1_MFP_P1_ALT_Pos)       /*!< SYS P1_MFP: P1_ALT Mask        */

#define SYS_P1_MFP_P1_MFP_Pos                   0                                       /*!< SYS P1_MFP: P1_MFP Position    */
#define SYS_P1_MFP_P1_MFP_Msk                   (0xFFul << SYS_P1_MFP_P1_MFP_Pos)       /*!< SYS P1_MFP: P1_MFP Mask        */

/* SYS P2_MFP Bit Field Definitions */
#define SYS_P2_MFP_P2_TYPE_Pos                  16                                      /*!< SYS P2_MFP: P2_TYPE Position   */
#define SYS_P2_MFP_P2_TYPE_Msk                  (0xFFul << SYS_P2_MFP_P2_TYPE_Pos)      /*!< SYS P2_MFP: P2_TYPE Mask       */

#define SYS_P2_MFP_P2_ALT_Pos                   8                                       /*!< SYS P2_MFP: P2_ALT Position    */
#define SYS_P2_MFP_P2_ALT_Msk                   (0xFFul << SYS_P2_MFP_P2_ALT_Pos)       /*!< SYS P2_MFP: P2_ALT Mask        */

#define SYS_P2_MFP_P2_MFP_Pos                   0                                       /*!< SYS P2_MFP: P2_MFP Position    */
#define SYS_P2_MFP_P2_MFP_Msk                   (0xFFul << SYS_P2_MFP_P2_MFP_Pos)       /*!< SYS P2_MFP: P2_MFP Mask        */

/* SYS P3_MFP Bit Field Definitions */
#define SYS_P3_MFP_P3_TYPE_Pos                  16                                      /*!< SYS P3_MFP: P3_TYPE Position   */
#define SYS_P3_MFP_P3_TYPE_Msk                  (0xFFul << SYS_P3_MFP_P3_TYPE_Pos)      /*!< SYS P3_MFP: P3_TYPE Mask       */

#define SYS_P3_MFP_P3_ALT_Pos                   8                                       /*!< SYS P3_MFP: P3_ALT Position    */
#define SYS_P3_MFP_P3_ALT_Msk                   (0xFFul << SYS_P3_MFP_P3_ALT_Pos)       /*!< SYS P3_MFP: P3_ALT Mask        */

#define SYS_P3_MFP_P3_MFP_Pos                   0                                       /*!< SYS P3_MFP: P3_MFP Position    */
#define SYS_P3_MFP_P3_MFP_Msk                   (0xFFul << SYS_P3_MFP_P3_MFP_Pos)       /*!< SYS P3_MFP: P3_MFP Mask        */

/* SYS P4_MFP Bit Field Definitions */
#define SYS_P4_MFP_P4_TYPE_Pos                  16                                      /*!< SYS P4_MFP: P4_TYPE Position   */
#define SYS_P4_MFP_P4_TYPE_Msk                  (0xFFul << SYS_P4_MFP_P4_TYPE_Pos)      /*!< SYS P4_MFP: P4_TYPE Mask       */

#define SYS_P4_MFP_P4_ALT_Pos                   8                                       /*!< SYS P4_MFP: P4_ALT Position    */
#define SYS_P4_MFP_P4_ALT_Msk                   (0xFFul << SYS_P4_MFP_P4_ALT_Pos)       /*!< SYS P4_MFP: P4_ALT Mask        */

#define SYS_P4_MFP_P4_MFP_Pos                   0                                       /*!< SYS P4_MFP: P4_MFP Position    */
#define SYS_P4_MFP_P4_MFP_Msk                   (0xFFul << SYS_P4_MFP_P4_MFP_Pos)       /*!< SYS P4_MFP: P4_MFP Mask        */

/* SYS P5_MFP Bit Field Definitions */
#define SYS_P5_MFP_P5_TYPE_Pos                  16                                      /*!< SYS P5_MFP: P5_TYPE Position   */
#define SYS_P5_MFP_P5_TYPE_Msk                  (0xFFul << SYS_P5_MFP_P5_TYPE_Pos)      /*!< SYS P5_MFP: P5_TYPE Mask       */

#define SYS_P5_MFP_P5_ALT_Pos                   8                                       /*!< SYS P5_MFP: P5_ALT Position    */
#define SYS_P5_MFP_P5_ALT_Msk                   (0xFFul << SYS_P5_MFP_P5_ALT_Pos)       /*!< SYS P5_MFP: P5_ALT Mask        */

#define SYS_P5_MFP_P5_MFP_Pos                   0                                       /*!< SYS P5_MFP: P5_MFP Position    */
#define SYS_P5_MFP_P5_MFP_Msk                   (0xFFul << SYS_P5_MFP_P5_MFP_Pos)       /*!< SYS P5_MFP: P5_MFP Mask        */

/* SYS IRCTRIMCTL Bit Field Definitions */
#define SYS_IRCTRIMCTL_TRIM_LOOP_Pos            3                                       /*!< SYS IRCTRIMCTL: TRIM_LOOP Position */
#define SYS_IRCTRIMCTL_TRIM_LOOP_Msk            (0x3ul << SYS_IRCTRIMCTL_TRIM_LOOP_Pos) /*!< SYS IRCTRIMCTL: TRIM_LOOP Mask     */

#define SYS_IRCTRIMCTL_TRIM_SEL_Pos             0                                       /*!< SYS IRCTRIMCTL: TRIM_SEL Position  */
#define SYS_IRCTRIMCTL_TRIM_SEL_Msk             (0x1ul << SYS_IRCTRIMCTL_TRIM_SEL_Pos)  /*!< SYS IRCTRIMCTL: TRIM_SEL Mask      */

/* SYS IRCTRIMIEN Bit Field Definitions */
#define SYS_IRCTRIMIEN_32K_ERR_IEN_Pos          2                                         /*!< SYS IRCTRIMIEN: 32K_ERR_IEN Position     */
#define SYS_IRCTRIMIEN_32K_ERR_IEN_Msk          (0x1ul << SYS_IRCTRIMIEN_32K_ERR_IEN_Pos) /*!< SYS IRCTRIMIEN: 32K_ERR_IEN Mask         */

#define SYS_IRCTRIMIEN_TRIM_FAIL_IEN_Pos        1                                         /*!< SYS IRCTRIMIEN: TRIM_FAIL_IEN Position   */
#define SYS_IRCTRIMIEN_TRIM_FAIL_IEN_Msk        (0x1ul << SYS_IRCTRIMIEN_TRIM_FAIL_IEN_Pos)/*!< SYS IRCTRIMIEN: TRIM_FAIL_IEN Mask      */

/* SYS IRCTRIMINT Bit Field Definitions */
#define SYS_IRCTRIMINT_32K_ERR_INT_Pos          2                                         /*!< SYS IRCTRIMINT: 32K_ERR_INT Position */
#define SYS_IRCTRIMINT_32K_ERR_INT_Msk          (0x1ul << SYS_IRCTRIMINT_32K_ERR_IEN_Pos) /*!< SYS IRCTRIMINT: 32K_ERR_INT Mask     */

#define SYS_IRCTRIMINT_TRIM_FAIL_INT_Pos        1                                         /*!< SYS IRCTRIMINT: TRIM_FAIL_INT Position   */
#define SYS_IRCTRIMINT_TRIM_FAIL_INT_Msk        (0x1ul << SYS_IRCTRIMINT_TRIM_FAIL_IEN_Pos)/*!< SYS IRCTRIMINT: TRIM_FAIL_INT Mask      */

#define SYS_IRCTRIMINT_FREQ_LOCK_Pos            0                                         /*!< SYS IRCTRIMINT: FREQ_LOCK Position   */
#define SYS_IRCTRIMINT_FREQ_LOCK_Msk            (0x1ul << SYS_IRCTRIMINT_FREQ_LOCK_Pos)   /*!< SYS IRCTRIMINT: FREQ_LOCK Mask       */

/* SYS RegLockAddr Bit Field Definitions */
#define SYS_RegLockAddr_RegUnLock_Pos           0                                         /*!< SYS RegLockAddr: RegUnLock Position  */
#define SYS_RegLockAddr_RegUnLock_Msk           (0x1ul << SYS_RegLockAddr_RegUnLock_Pos)  /*!< SYS RegLockAddr: RegUnLock Mask      */
/// @endcond /* HIDDEN_SYMBOLS */
/*@}*/ /* end of group SYS */



/*----------------------------- Timer Controller -----------------------------*/
/** @addtogroup TIMER Timer Controller(TIMER)
  Memory Mapped Structure for TIMER Controller
  @{
 */

typedef struct
{
    __IO uint32_t  TCSR;       /*!< Offset: 0x0000   Timer Control and Status Register          */
    __IO uint32_t  TCMPR;      /*!< Offset: 0x0004   Timer Compare Register                     */
    __IO uint32_t  TISR;       /*!< Offset: 0x0008   Timer Interrupt Status Register            */
    __I  uint32_t  TDR;        /*!< Offset: 0x000C   Timer Data Register                        */
    __I  uint32_t  TCAP;       /*!< Offset: 0x0010   Timer Capture Data Register                */
    __IO uint32_t  TEXCON;     /*!< Offset: 0x0014   Timer External Control Register            */
    __IO uint32_t  TEXISR;     /*!< Offset: 0x0018   Timer External Interrupt Status Register   */
} TIMER_T;
/// @cond HIDDEN_SYMBOLS
/* TIMER TCSR Bit Field Definitions */
#define TIMER_TCSR_DBGACK_TMR_Pos   31                                          /*!< TIMER TCSR: DBGACK_TMR Position    */
#define TIMER_TCSR_DBGACK_TMR_Msk   (1ul << TIMER_TCSR_DBGACK_TMR_Pos)          /*!< TIMER TCSR: DBGACK_TMR Mask        */

#define TIMER_TCSR_CEN_Pos          30                                          /*!< TIMER TCSR: CEN Position           */
#define TIMER_TCSR_CEN_Msk          (1ul << TIMER_TCSR_CEN_Pos)                 /*!< TIMER TCSR: CEN Mask               */

#define TIMER_TCSR_IE_Pos           29                                          /*!< TIMER TCSR: IE Position            */
#define TIMER_TCSR_IE_Msk           (1ul << TIMER_TCSR_IE_Pos)                  /*!< TIMER TCSR: IE Mask                */

#define TIMER_TCSR_MODE_Pos         27                                          /*!< TIMER TCSR: MODE Position          */
#define TIMER_TCSR_MODE_Msk         (0x3ul << TIMER_TCSR_MODE_Pos)              /*!< TIMER TCSR: MODE Mask              */

#define TIMER_TCSR_CRST_Pos         26                                          /*!< TIMER TCSR: CRST Position          */
#define TIMER_TCSR_CRST_Msk         (1ul << TIMER_TCSR_CRST_Pos)                /*!< TIMER TCSR: CRST Mask              */

#define TIMER_TCSR_CACT_Pos         25                                          /*!< TIMER TCSR: CACT Position          */
#define TIMER_TCSR_CACT_Msk         (1ul << TIMER_TCSR_CACT_Pos)                /*!< TIMER TCSR: CACT Mask              */

#define TIMER_TCSR_CTB_Pos          24                                          /*!< TIMER TCSR: CTB Position           */
#define TIMER_TCSR_CTB_Msk          (1ul << TIMER_TCSR_CTB_Pos)                 /*!< TIMER TCSR: CTB Mask               */

#define TIMER_TCSR_WAKE_EN_Pos      23                                          /*!< TIMER TCSR: WAKE_EN Position       */
#define TIMER_TCSR_WAKE_EN_Msk      (1ul << TIMER_TCSR_WAKE_EN_Pos)             /*!< TIMER TCSR: WAKE_EN Mask           */

#define TIMER_TCSR_CAP_SRC_Pos      19                                          /*!< TIMER TCSR: CAP_SRC Position       */
#define TIMER_TCSR_CAP_SRC_Msk      (1ul << TIMER_TCSR_CAP_SRC_Pos)             /*!< TIMER TCSR: CAP_SRC Mask           */

#define TIMER_TCSR_TOGGLE_PIN_Pos   18                                          /*!< TIMER TCSR: TOGGLE_PIN Position    */
#define TIMER_TCSR_TOGGLE_PIN_Msk   (1ul << TIMER_TCSR_TOGGLE_PIN_Pos)          /*!< TIMER TCSR: TOGGLE_PIN Mask        */

#define TIMER_TCSR_PERIODIC_SEL_Pos 17                                          /*!< TIMER TCSR: PERIODIC_SEL Position  */
#define TIMER_TCSR_PERIODIC_SEL_Msk (1ul << TIMER_TCSR_PERIODIC_SEL_Pos)        /*!< TIMER TCSR: PERIODIC_SEL Mask      */

#define TIMER_TCSR_TDR_EN_Pos       16                                          /*!< TIMER TCSR: TDR_EN Position        */
#define TIMER_TCSR_TDR_EN_Msk       (1ul << TIMER_TCSR_TDR_EN_Pos)              /*!< TIMER TCSR: TDR_EN Mask            */

#define TIMER_TCSR_PRESCALE_Pos     0                                           /*!< TIMER TCSR: PRESCALE Position      */
#define TIMER_TCSR_PRESCALE_Msk     (0xFFul << TIMER_TCSR_PRESCALE_Pos)         /*!< TIMER TCSR: PRESCALE Mask          */

/* TIMER TCMPR Bit Field Definitions */
#define TIMER_TCMP_TCMP_Pos         0                                           /*!< TIMER TCMPR: TCMP Position         */
#define TIMER_TCMP_TCMP_Msk         (0xFFFFFFul << TIMER_TCMP_TCMP_Pos)         /*!< TIMER TCMPR: TCMP Mask             */

/* TIMER TISR Bit Field Definitions */
#define TIMER_TISR_TWF_Pos          1                                           /*!< TIMER TISR: TWF Position           */
#define TIMER_TISR_TWF_Msk          (1ul << TIMER_TISR_TWF_Pos)                 /*!< TIMER TISR: TWF Mask               */

#define TIMER_TISR_TIF_Pos          0                                           /*!< TIMER TISR: TIF Position           */
#define TIMER_TISR_TIF_Msk          (1ul << TIMER_TISR_TIF_Pos)                 /*!< TIMER TISR: TIF Mask               */

/* TIMER TDR Bit Field Definitions */
#define TIMER_TDR_TDR_Pos           0                                           /*!< TIMER TDR: TDR Position            */
#define TIMER_TDR_TDR_Msk           (0xFFFFFFul << TIMER_TDR_TDR_Pos)           /*!< TIMER TDR: TDR Mask                */

/* TIMER TCAP Bit Field Definitions */
#define TIMER_TCAP_TCAP_Pos         0                                           /*!< TIMER TCAP: TCAP Position          */
#define TIMER_TCAP_TCAP_Msk         (0xFFFFFFul << TIMER_TCAP_TCAP_Pos)         /*!< TIMER TCAP: TCAP Mask              */

/* TIMER TEXCON Bit Field Definitions */
#define TIMER_TEXCON_CAP_MODE_Pos   8                                           /*!< TIMER TEXCON: CAP_MODE Position    */
#define TIMER_TEXCON_CAP_MODE_Msk   (1ul << TIMER_TEXCON_CAP_MODE_Pos)          /*!< TIMER TEXCON: CAP_MODE Mask        */

#define TIMER_TEXCON_TCDB_Pos       7                                           /*!< TIMER TEXCON: TCDB Position        */
#define TIMER_TEXCON_TCDB_Msk       (1ul << TIMER_TEXCON_TCDB_Pos)              /*!< TIMER TEXCON: TCDB Mask            */

#define TIMER_TEXCON_TEXDB_Pos      6                                           /*!< TIMER TEXCON: TEXDB Position       */
#define TIMER_TEXCON_TEXDB_Msk      (1ul << TIMER_TEXCON_TEXDB_Pos)             /*!< TIMER TEXCON: TEXDB Mask           */

#define TIMER_TEXCON_TEXIEN_Pos     5                                           /*!< TIMER TEXCON: TEXIEN Position      */
#define TIMER_TEXCON_TEXIEN_Msk     (1ul << TIMER_TEXCON_TEXIEN_Pos)            /*!< TIMER TEXCON: TEXIEN Mask          */

#define TIMER_TEXCON_RSTCAPSEL_Pos  4                                           /*!< TIMER TEXCON: RSTCAPSEL Position   */
#define TIMER_TEXCON_RSTCAPSEL_Msk  (1ul << TIMER_TEXCON_RSTCAPSEL_Pos)         /*!< TIMER TEXCON: RSTCAPSEL Mask       */

#define TIMER_TEXCON_TEXEN_Pos      3                                           /*!< TIMER TEXCON: TEXEN Position       */
#define TIMER_TEXCON_TEXEN_Msk      (1ul << TIMER_TEXCON_TEXEN_Pos)             /*!< TIMER TEXCON: TEXEN Mask           */

#define TIMER_TEXCON_TEX_EDGE_Pos   1                                           /*!< TIMER TEXCON: TEX_EDGE Position    */
#define TIMER_TEXCON_TEX_EDGE_Msk   (0x3ul << TIMER_TEXCON_TEX_EDGE_Pos)        /*!< TIMER TEXCON: TEX_EDGE Mask        */

#define TIMER_TEXCON_TX_PHASE_Pos   0                                           /*!< TIMER TEXCON: TX_PHASE Position    */
#define TIMER_TEXCON_TX_PHASE_Msk   (1ul << TIMER_TEXCON_TX_PHASE_Pos)          /*!< TIMER TEXCON: TX_PHASE Mask        */

/* TIMER TEXISR Bit Field Definitions */
#define TIMER_TEXISR_TEXIF_Pos      0                                           /*!< TIMER TEXISR: TEXIF Position       */
#define TIMER_TEXISR_TEXIF_Msk      (1ul << TIMER_TEXISR_TEXIF_Pos)             /*!< TIMER TEXISR: TEXIF Mask           */
/// @endcond /* HIDDEN_SYMBOLS */
/*@}*/ /* end of group TIMER */


/*---------------------- Universal Asynchronous Receiver/Transmitter Controller -------------------------*/

/** @addtogroup UART Universal Asynchronous Receiver/Transmitter Controller(UART)
  Memory Mapped Structure for UART Controller
  @{
 */
typedef struct
{
    union
    {
        __I   uint32_t  RBR;         /*!< Offset: 0x0000   UART Receive Buffer Register               */
        __O   uint32_t  THR;         /*!< Offset: 0x0000   UART Transmit Holding Register             */
    };
    __IO uint32_t  IER;          /*!< Offset: 0x0004   UART Interrupt Enable Register             */
    __IO uint32_t  FCR;          /*!< Offset: 0x0008   UART FIFO Control Register                 */
    __IO uint32_t  LCR;          /*!< Offset: 0x000C   UART Line Control Register                 */
    __IO uint32_t  MCR;          /*!< Offset: 0x0010   UART Modem Control Register                */
    __IO uint32_t  MSR;          /*!< Offset: 0x0014   UART Modem Status Register                 */
    __IO uint32_t  FSR;          /*!< Offset: 0x0018   UART FIFO Status Register                  */
    __IO uint32_t  ISR;          /*!< Offset: 0x001C   UART Interrupt Status Register             */
    __IO uint32_t  TOR;          /*!< Offset: 0x0020   UART Time-out Register                     */
    __IO uint32_t  BAUD;         /*!< Offset: 0x0024   UART Baud Rate Divisor Register            */
    __IO uint32_t  IRCR;         /*!< Offset: 0x0028   UART IrDA Control Register                 */
    __IO uint32_t  ALT_CSR;      /*!< Offset: 0x002C   UART Alternate Control/Status Register     */
    __IO uint32_t  FUN_SEL;      /*!< Offset: 0x0030   UART Function Select Register              */
} UART_T;
/// @cond HIDDEN_SYMBOLS
/* UART THR Bit Field Definitions */
#define UART_THR_THR_Pos         0                                          /*!< UART THR: THR Position  */
#define UART_THR_THR_Msk        (0xFul << UART_THR_THR_Pos)                 /*!< UART THR: THR Mask      */

/* UART RBR Bit Field Definitions */
#define UART_RBR_RBR_Pos         0                                          /*!< UART RBR: RBR Position */
#define UART_RBR_RBR_Msk        (0xFul << UART_RBR_RBR_Pos)                 /*!< UART RBR: RBR Mask      */

/* UART IER Bit Field Definitions */
#define UART_IER_AUTO_CTS_EN_Pos    13                                      /*!< UART IER: AUTO_CTS_EN Position      */
#define UART_IER_AUTO_CTS_EN_Msk    (1ul << UART_IER_AUTO_CTS_EN_Pos)       /*!< UART IER: AUTO_CTS_EN Mask           */

#define UART_IER_AUTO_RTS_EN_Pos    12                                      /*!< UART IER: AUTO_RTS_EN Position      */
#define UART_IER_AUTO_RTS_EN_Msk    (1ul << UART_IER_AUTO_RTS_EN_Pos)       /*!< UART IER: AUTO_RTS_EN Mask           */

#define UART_IER_TIME_OUT_EN_Pos    11                                      /*!< UART IER: TIME_OUT_EN Position      */
#define UART_IER_TIME_OUT_EN_Msk    (1ul << UART_IER_TIME_OUT_EN_Pos)       /*!< UART IER: TIME_OUT_EN Mask           */

#define UART_IER_WAKE_EN_Pos        6                                       /*!< UART IER: WAKE_EN Position          */
#define UART_IER_WAKE_EN_Msk        (1ul << UART_IER_WAKE_EN_Pos)           /*!< UART IER: WAKE_EN Mask               */

#define UART_IER_BUF_ERR_IEN_Pos    5                                       /*!< UART IER: BUF_ERR_IEN Position      */
#define UART_IER_BUF_ERR_IEN_Msk    (1ul << UART_IER_BUF_ERR_IEN_Pos)       /*!< UART IER: BUF_ERR_IEN Mask           */

#define UART_IER_RTO_IEN_Pos        4                                       /*!< UART IER: RTO_IEN Position          */
#define UART_IER_RTO_IEN_Msk        (1ul << UART_IER_RTO_IEN_Pos)           /*!< UART IER: RTO_IEN Mask               */

#define UART_IER_MODEM_IEN_Pos      3                                       /*!< UART IER: MODEM_IEN Position        */
#define UART_IER_MODEM_IEN_Msk      (1ul << UART_IER_MODEM_IEN_Pos)         /*!< UART IER: MODEM_IEN Mask             */

#define UART_IER_RLS_IEN_Pos        2                                       /*!< UART IER: RLS_IEN Position          */
#define UART_IER_RLS_IEN_Msk        (1ul << UART_IER_RLS_IEN_Pos)           /*!< UART IER: RLS_IEN Mask               */

#define UART_IER_THRE_IEN_Pos       1                                       /*!< UART IER: THRE_IEN Position         */
#define UART_IER_THRE_IEN_Msk       (1ul << UART_IER_THRE_IEN_Pos)          /*!< UART IER: THRE_IEN Mask              */

#define UART_IER_RDA_IEN_Pos        0                                       /*!< UART IER: RDA_IEN Position           */
#define UART_IER_RDA_IEN_Msk        (1ul << UART_IER_RDA_IEN_Pos)           /*!< UART IER: RDA_IEN Mask               */

/* UART FCR Bit Field Definitions */
#define UART_FCR_RTS_TRI_LEV_Pos    16                                      /*!< UART FCR: RTS_TRI_LEV Position       */
#define UART_FCR_RTS_TRI_LEV_Msk    (0xFul << UART_FCR_RTS_TRI_LEV_Pos)     /*!< UART FCR: RTS_TRI_LEV Mask           */

#define UART_FCR_RX_DIS_Pos         8                                       /*!< UART FCR: RX_DIS Position            */
#define UART_FCR_RX_DIS_Msk         (1ul << UART_FCR_RX_DIS_Pos)            /*!< UART FCR: RX_DIS Mask                */

#define UART_FCR_RFITL_Pos          4                                       /*!< UART FCR: RFITL Position             */
#define UART_FCR_RFITL_Msk          (0xFul << UART_FCR_RFITL_Pos)           /*!< UART FCR: RFITL Mask                 */

#define UART_FCR_TFR_Pos            2                                       /*!< UART FCR: TFR Position               */
#define UART_FCR_TFR_Msk            (1ul << UART_FCR_TFR_Pos)               /*!< UART FCR: TFR Mask                   */

#define UART_FCR_RFR_Pos            1                                       /*!< UART FCR: RFR Position               */
#define UART_FCR_RFR_Msk            (1ul << UART_FCR_RFR_Pos)               /*!< UART FCR: RFR Mask                   */

/* UART LCR Bit Field Definitions */
#define UART_LCR_BCB_Pos            6                                       /*!< UART LCR: BCB Position               */
#define UART_LCR_BCB_Msk            (1ul << UART_LCR_BCB_Pos)               /*!< UART LCR: BCB Mask                   */

#define UART_LCR_SPE_Pos            5                                       /*!< UART LCR: SPE Position               */
#define UART_LCR_SPE_Msk            (1ul << UART_LCR_SPE_Pos)               /*!< UART LCR: SPE Mask                   */

#define UART_LCR_EPE_Pos            4                                       /*!< UART LCR: EPE Position               */
#define UART_LCR_EPE_Msk            (1ul << UART_LCR_EPE_Pos)               /*!< UART LCR: EPE Mask                   */

#define UART_LCR_PBE_Pos            3                                       /*!< UART LCR: PBE Position               */
#define UART_LCR_PBE_Msk            (1ul << UART_LCR_PBE_Pos)               /*!< UART LCR: PBE Mask                   */

#define UART_LCR_NSB_Pos            2                                       /*!< UART LCR: NSB Position               */
#define UART_LCR_NSB_Msk            (1ul << UART_LCR_NSB_Pos)               /*!< UART LCR: NSB Mask                   */

#define UART_LCR_WLS_Pos            0                                       /*!< UART LCR: WLS Position               */
#define UART_LCR_WLS_Msk            (0x3ul << UART_LCR_WLS_Pos)             /*!< UART LCR: WLS Mask                   */

/* UART MCR Bit Field Definitions */
#define UART_MCR_RTS_ST_Pos         13                                      /*!< UART MCR: RTS_ST Position            */
#define UART_MCR_RTS_ST_Msk         (1ul << UART_MCR_RTS_ST_Pos)            /*!< UART MCR: RTS_ST Mask                */

#define UART_MCR_LEV_RTS_Pos        9                                       /*!< UART MCR: LEV_RTS Position           */
#define UART_MCR_LEV_RTS_Msk        (1ul << UART_MCR_LEV_RTS_Pos)           /*!< UART MCR: LEV_RTS Mask               */

#define UART_MCR_RTS_Pos            1                                       /*!< UART MCR: RTS Position               */
#define UART_MCR_RTS_Msk            (1ul << UART_MCR_RTS_Pos)               /*!< UART MCR: RTS Mask                   */


/* UART MSR Bit Field Definitions */
#define UART_MSR_LEV_CTS_Pos        8                                       /*!< UART MSR: LEV_CTS Position           */
#define UART_MSR_LEV_CTS_Msk        (1ul << UART_MSR_LEV_CTS_Pos)           /*!< UART MSR: LEV_CTS Mask               */

#define UART_MSR_CTS_ST_Pos         4                                       /*!< UART MSR: CTS_ST Position            */
#define UART_MSR_CTS_ST_Msk         (1ul << UART_MSR_CTS_ST_Pos)            /*!< UART MSR: CTS_ST Mask                */

#define UART_MSR_DCTSF_Pos          0                                       /*!< UART MSR: DCTST Position             */
#define UART_MSR_DCTSF_Msk          (1ul << UART_MSR_DCTSF_Pos)             /*!< UART MSR: DCTST Mask                 */


/* UART FSR Bit Field Definitions */
#define UART_FSR_TE_FLAG_Pos        28                                      /*!< UART FSR: TE_FLAG Position           */
#define UART_FSR_TE_FLAG_Msk        (1ul << UART_FSR_TE_FLAG_Pos)           /*!< UART FSR: TE_FLAG Mask               */

#define UART_FSR_TX_OVER_IF_Pos     24                                      /*!< UART FSR: TX_OVER_IF Position        */
#define UART_FSR_TX_OVER_IF_Msk     (1ul << UART_FSR_TX_OVER_IF_Pos)        /*!< UART FSR: TX_OVER_IF Mask            */

#define UART_FSR_TX_FULL_Pos        23                                      /*!< UART FSR: TX_FULL Position           */
#define UART_FSR_TX_FULL_Msk        (1ul << UART_FSR_TX_FULL_Pos)           /*!< UART FSR: TX_FULL Mask               */

#define UART_FSR_TX_EMPTY_Pos       22                                      /*!< UART FSR: TX_EMPTY Position          */
#define UART_FSR_TX_EMPTY_Msk       (1ul << UART_FSR_TX_EMPTY_Pos)          /*!< UART FSR: TX_EMPTY Mask              */

#define UART_FSR_TX_POINTER_Pos     16                                      /*!< UART FSR: TX_POINTER Position        */
#define UART_FSR_TX_POINTER_Msk     (0x3Ful << UART_FSR_TX_POINTER_Pos)     /*!< UART FSR: TX_POINTER Mask            */

#define UART_FSR_RX_FULL_Pos        15                                      /*!< UART FSR: RX_FULL Position           */
#define UART_FSR_RX_FULL_Msk        (1ul << UART_FSR_RX_FULL_Pos)           /*!< UART FSR: RX_FULL Mask               */

#define UART_FSR_RX_EMPTY_Pos       14                                      /*!< UART FSR: RX_EMPTY Position          */
#define UART_FSR_RX_EMPTY_Msk       (1ul << UART_FSR_RX_EMPTY_Pos)          /*!< UART FSR: RX_EMPTY Mask              */

#define UART_FSR_RX_POINTER_Pos     8                                       /*!< UART FSR: RX_POINTERS Position       */
#define UART_FSR_RX_POINTER_Msk     (0x3Ful << UART_FSR_RX_POINTER_Pos)     /*!< UART FSR: RX_POINTER Mask            */

#define UART_FSR_BIF_Pos            6                                       /*!< UART FSR: BIF Position               */
#define UART_FSR_BIF_Msk            (1ul << UART_FSR_BIF_Pos)               /*!< UART FSR: BIF Mask                   */

#define UART_FSR_FEF_Pos            5                                       /*!< UART FSR: FEF Position               */
#define UART_FSR_FEF_Msk            (1ul << UART_FSR_FEF_Pos)               /*!< UART FSR: FEF Mask                   */

#define UART_FSR_PEF_Pos            4                                       /*!< UART FSR: PEF Position               */
#define UART_FSR_PEF_Msk            (1ul << UART_FSR_PEF_Pos)               /*!< UART FSR: PEF Mask                   */

#define UART_FSR_RS485_ADD_DETF_Pos 3                                       /*!< UART FSR: RS485_ADD_DETF Position    */
#define UART_FSR_RS485_ADD_DETF_Msk (1ul << UART_FSR_RS485_ADD_DETF_Pos)    /*!< UART FSR: RS485_ADD_DETF Mask        */

#define UART_FSR_RX_OVER_IF_Pos     0                                       /*!< UART FSR: RX_OVER_IF Position        */
#define UART_FSR_RX_OVER_IF_Msk     (1ul << UART_FSR_RX_OVER_IF_Pos)        /*!< UART FSR: RX_OVER_IF Mask            */

/* UART ISR Bit Field Definitions */
#define UART_ISR_BUF_ERR_INT_Pos    13                                      /*!< UART ISR: BUF_ERR_INT Position       */
#define UART_ISR_BUF_ERR_INT_Msk    (1ul << UART_ISR_BUF_ERR_INT_Pos)       /*!< UART ISR: BUF_ERR_INT Mask           */

#define UART_ISR_TOUT_INT_Pos       12                                      /*!< UART ISR: TOUT_INT Position          */
#define UART_ISR_TOUT_INT_Msk       (1ul << UART_ISR_TOUT_INT_Pos)          /*!< UART ISR: TOUT_INT Mask              */

#define UART_ISR_MODEM_INT_Pos      11                                      /*!< UART ISR: MODEM_INT Position         */
#define UART_ISR_MODEM_INT_Msk      (1ul << UART_ISR_MODEM_INT_Pos)         /*!< UART ISR: MODEM_INT Mask             */

#define UART_ISR_RLS_INT_Pos        10                                      /*!< UART ISR: RLS_INT Position           */
#define UART_ISR_RLS_INT_Msk        (1ul << UART_ISR_RLS_INT_Pos)           /*!< UART ISR: RLS_INT Mask               */

#define UART_ISR_THRE_INT_Pos       9                                       /*!< UART ISR: THRE_INT Position          */
#define UART_ISR_THRE_INT_Msk       (1ul << UART_ISR_THRE_INT_Pos)          /*!< UART ISR: THRE_INT Mask              */

#define UART_ISR_RDA_INT_Pos        8                                       /*!< UART ISR: RDA_INT Position           */
#define UART_ISR_RDA_INT_Msk        (1ul << UART_ISR_RDA_INT_Pos)           /*!< UART ISR: RDA_INT Mask               */

#define UART_ISR_BUF_ERR_IF_Pos     5                                       /*!< UART ISR: BUF_ERR_IF Position        */
#define UART_ISR_BUF_ERR_IF_Msk     (1ul << UART_ISR_BUF_ERR_IF_Pos)        /*!< UART ISR: BUF_ERR_IF Mask            */

#define UART_ISR_TOUT_IF_Pos        4                                       /*!< UART ISR: TOUT_IF Position           */
#define UART_ISR_TOUT_IF_Msk        (1ul << UART_ISR_TOUT_IF_Pos)           /*!< UART ISR: TOUT_IF Mask               */

#define UART_ISR_MODEM_IF_Pos       3                                       /*!< UART ISR: MODEM_IF Position          */
#define UART_ISR_MODEM_IF_Msk       (1ul << UART_ISR_MODEM_IF_Pos)          /*!< UART ISR: MODEM_IF Mask              */

#define UART_ISR_RLS_IF_Pos         2                                       /*!< UART ISR: RLS_IF Position            */
#define UART_ISR_RLS_IF_Msk         (1ul << UART_ISR_RLS_IF_Pos)            /*!< UART ISR: RLS_IF Mask                */

#define UART_ISR_THRE_IF_Pos        1                                       /*!< UART ISR: THRE_IF Position           */
#define UART_ISR_THRE_IF_Msk        (1ul << UART_ISR_THRE_IF_Pos)           /*!< UART ISR: THRE_IF Mask               */

#define UART_ISR_RDA_IF_Pos         0                                       /*!< UART ISR: RDA_IF Position            */
#define UART_ISR_RDA_IF_Msk         (1ul << UART_ISR_RDA_IF_Pos)            /*!< UART ISR: RDA_IF Mask                */


/* UART TOR Bit Field Definitions */
#define UART_TOR_DLY_Pos           8                                        /*!< UART TOR: DLY Position               */
#define UART_TOR_DLY_Msk           (0xFFul << UART_TOR_DLY_Pos)             /*!< UART TOR: DLY Mask                   */

#define UART_TOR_TOIC_Pos          0                                        /*!< UART TOR: TOIC Position              */
#define UART_TOR_TOIC_Msk          (0xFFul << UART_TOR_TOIC_Pos)            /*!< UART TOR: TOIC Mask                  */

/* UART BAUD Bit Field Definitions */
#define UART_BAUD_DIV_X_EN_Pos    29                                        /*!< UART BARD: DIV_X_EN Position         */
#define UART_BAUD_DIV_X_EN_Msk    (1ul << UART_BAUD_DIV_X_EN_Pos)           /*!< UART BARD: DIV_X_EN Mask             */

#define UART_BAUD_DIV_X_ONE_Pos   28                                        /*!< UART BARD: DIV_X_ONE Position        */
#define UART_BAUD_DIV_X_ONE_Msk   (1ul << UART_BAUD_DIV_X_ONE_Pos)          /*!< UART BARD: DIV_X_ONE Mask            */

#define UART_BAUD_DIVIDER_X_Pos   24                                        /*!< UART BARD: DIVIDER_X Position        */
#define UART_BAUD_DIVIDER_X_Msk   (0xFul << UART_BAUD_DIVIDER_X_Pos)        /*!< UART BARD: DIVIDER_X Mask            */

#define UART_BAUD_BRD_Pos         0                                         /*!< UART BARD: BRD Position              */
#define UART_BAUD_BRD_Msk         (0xFFFFul << UART_BAUD_BRD_Pos)           /*!< UART BARD: BRD Mask                  */

/* UART IRCR Bit Field Definitions */
#define UART_IRCR_INV_RX_Pos      6                                         /*!< UART IRCR: INV_RX Position           */
#define UART_IRCR_INV_RX_Msk     (1ul << UART_IRCR_INV_RX_Pos)              /*!< UART IRCR: INV_RX Mask               */

#define UART_IRCR_INV_TX_Pos      5                                         /*!< UART IRCR: INV_TX Position           */
#define UART_IRCR_INV_TX_Msk     (1ul << UART_IRCR_INV_TX_Pos)              /*!< UART IRCR: INV_TX Mask               */

#define UART_IRCR_TX_SELECT_Pos   1                                         /*!< UART IRCR: TX_SELECT Position        */
#define UART_IRCR_TX_SELECT_Msk   (1ul << UART_IRCR_TX_SELECT_Pos)          /*!< UART IRCR: TX_SELECT Mask            */

/* UART ALT_CSR Bit Field Definitions */
#define UART_ALT_CSR_ADDR_MATCH_Pos      24                                      /*!< UART ALT_CSR: ADDR_MATCH Position    */
#define UART_ALT_CSR_ADDR_MATCH_Msk     (0xFFul << UART_ALT_CSR_ADDR_MATCH_Pos)  /*!< UART ALT_CSR: ADDR_MATCH Mask        */

#define UART_ALT_CSR_RS485_ADD_EN_Pos   15                                       /*!< UART ALT_CSR: RS485_ADD_EN Position  */
#define UART_ALT_CSR_RS485_ADD_EN_Msk   (1ul << UART_ALT_CSR_RS485_ADD_EN_Pos)   /*!< UART ALT_CSR: RS485_ADD_EN Mask      */

#define UART_ALT_CSR_RS485_AUD_Pos      10                                       /*!< UART ALT_CSR: RS485_AUD Position     */
#define UART_ALT_CSR_RS485_AUD_Msk      (1ul << UART_ALT_CSR_RS485_AUD_Pos)      /*!< UART ALT_CSR: RS485_AUD Mask         */

#define UART_ALT_CSR_RS485_AAD_Pos      9                                        /*!< UART ALT_CSR: RS485_AAD Position     */
#define UART_ALT_CSR_RS485_AAD_Msk      (1ul << UART_ALT_CSR_RS485_AAD_Pos)      /*!< UART ALT_CSR: RS485_AAD Mask         */

#define UART_ALT_CSR_RS485_NMM_Pos      8                                        /*!< UART ALT_CSR: RS485_NMM Position     */
#define UART_ALT_CSR_RS485_NMM_Msk      (1ul << UART_ALT_CSR_RS485_NMM_Pos)      /*!< UART ALT_CSR: RS485_NMM Mask         */

/* UART FUN_SEL Bit Field Definitions */
#define UART_FUN_SEL_FUN_SEL_Pos        0                                        /*!< UART FUN_SEL: FUN_SEL Position       */
#define UART_FUN_SEL_FUN_SEL_Msk       (0x3ul << UART_FUN_SEL_FUN_SEL_Pos)       /*!< UART FUN_SEL: FUN_SEL Mask           */

/// @endcond /* HIDDEN_SYMBOLS */
/*@}*/ /* end of group UART */

/*---------------------- Watchdog Timer Controller -------------------------*/

/** @addtogroup WDT Watchdog Timer Controller(WDT)
  Memory Mapped Structure for Watchdog Timer Controller
  @{
 */
/**
  * @brief WDT register map
  */
typedef struct
{
    __IO uint32_t  WTCR;          /*!< Offset: 0x0000   Watchdog Timer Control Register   */
} WDT_T;
/// @cond HIDDEN_SYMBOLS
/* WDT WTCR Bit Field Definitions */
#define WDT_WTCR_DBGACK_WDT_Pos 31                                              /*!< WDT WTCR : DBGACK_WDT Position */
#define WDT_WTCR_DBGACK_WDT_Msk (1ul << WDT_WTCR_DBGACK_WDT_Pos)                /*!< WDT WTCR : DBGACK_WDT Mask     */

#define WDT_WTCR_WTIS_Pos       8                                               /*!< WDT WTCR : WTIS Position       */
#define WDT_WTCR_WTIS_Msk       (0x7ul << WDT_WTCR_WTIS_Pos)                    /*!< WDT WTCR : WTIS Mask           */

#define WDT_WTCR_WTE_Pos        7                                               /*!< WDT WTCR : WTE Position        */
#define WDT_WTCR_WTE_Msk        (1ul << WDT_WTCR_WTE_Pos)                       /*!< WDT WTCR : WTE Mask            */

#define WDT_WTCR_WTIE_Pos       6                                               /*!< WDT WTCR : WTIE Position       */
#define WDT_WTCR_WTIE_Msk       (1ul << WDT_WTCR_WTIE_Pos)                      /*!< WDT WTCR : WTIE Mask           */

#define WDT_WTCR_WTWKF_Pos      5                                               /*!< WDT WTCR : WTWKF Position      */
#define WDT_WTCR_WTWKF_Msk      (1ul << WDT_WTCR_WTWKF_Pos)                     /*!< WDT WTCR : WTWKF Mask          */

#define WDT_WTCR_WTWKE_Pos      4                                               /*!< WDT WTCR : WTWKE Position      */
#define WDT_WTCR_WTWKE_Msk      (1ul << WDT_WTCR_WTWKE_Pos)                     /*!< WDT WTCR : WTWKE Mask          */

#define WDT_WTCR_WTIF_Pos       3                                               /*!< WDT WTCR : WTIF Position       */
#define WDT_WTCR_WTIF_Msk       (1ul << WDT_WTCR_WTIF_Pos)                      /*!< WDT WTCR : WTIF Mask           */

#define WDT_WTCR_WTRF_Pos       2                                               /*!< WDT WTCR : WTRF Position       */
#define WDT_WTCR_WTRF_Msk       (1ul << WDT_WTCR_WTRF_Pos)                      /*!< WDT WTCR : WTRF Mask           */

#define WDT_WTCR_WTRE_Pos       1                                               /*!< WDT WTCR : WTRE Position       */
#define WDT_WTCR_WTRE_Msk       (1ul << WDT_WTCR_WTRE_Pos)                      /*!< WDT WTCR : WTRE Mask           */

#define WDT_WTCR_WTR_Pos        0                                               /*!< WDT WTCR : WTR Position        */
#define WDT_WTCR_WTR_Msk        (1ul << WDT_WTCR_WTR_Pos)                       /*!< WDT WTCR : WTR Mask            */
/// @endcond /* HIDDEN_SYMBOLS */
/*@}*/ /* end of group WDT */


#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/*@}*/ /* end of group NUC029FAE_Peripherals */

/** @addtogroup NUC029FAE_PERIPHERAL_MEM_MAP NUC029FAE Peripheral Memory Base
  Memory Mapped Structure for NUC029FAE Peripheral
  @{
 */
/* Peripheral and SRAM base address */
#define FLASH_BASE            ((uint32_t)0x00000000)    ///< Flash base address
#define SRAM_BASE             ((uint32_t)0x20000000)    ///< SRAM base address
#define APB1PERIPH_BASE       ((uint32_t)0x40000000)    ///< APB1 base address
#define APB2PERIPH_BASE       ((uint32_t)0x40100000)    ///< APB2 base address
#define AHBPERIPH_BASE        ((uint32_t)0x50000000)    ///< AHB base address

/* Peripheral memory map */
#define WDT_BASE              (APB1PERIPH_BASE + 0x04000)    ///< WDT register base address
#define TIMER0_BASE           (APB1PERIPH_BASE + 0x10000)    ///< TIMER0 register base address
#define TIMER1_BASE           (APB1PERIPH_BASE + 0x10020)    ///< TIMER1 register base address
#define I2C_BASE              (APB1PERIPH_BASE + 0x20000)    ///< I2C register base address
#define SPI_BASE              (APB1PERIPH_BASE + 0x30000)    ///< SPI register base address
#define PWM_BASE              (APB1PERIPH_BASE + 0x40000)    ///< PWM register base address
#define UART_BASE             (APB1PERIPH_BASE + 0x50000)    ///< UART register base address
#define ACMP_BASE             (APB1PERIPH_BASE + 0xD0000)    ///< ACMP register base address
#define ADC_BASE              (APB1PERIPH_BASE + 0xE0000)    ///< ADC register base address

#define SYS_BASE              (AHBPERIPH_BASE + 0x00000)    ///< GCR register base address
#define CLK_BASE              (AHBPERIPH_BASE + 0x00200)    ///< CLK register base address
#define INT_BASE              (AHBPERIPH_BASE + 0x00300)    ///< INT register base address
#define P0_BASE               (AHBPERIPH_BASE + 0x04000)    ///< GPIO Port 0 register base address
#define P1_BASE               (AHBPERIPH_BASE + 0x04040)    ///< GPIO Port 1 register base address
#define P2_BASE               (AHBPERIPH_BASE + 0x04080)    ///< GPIO Port 2 register base address
#define P3_BASE               (AHBPERIPH_BASE + 0x040C0)    ///< GPIO Port 3 register base address
#define P4_BASE               (AHBPERIPH_BASE + 0x04100)    ///< GPIO Port 4 register base address
#define P5_BASE               (AHBPERIPH_BASE + 0x04140)    ///< GPIO Port 5 register base address
#define GPIO_DBNCECON_BASE    (AHBPERIPH_BASE + 0x04180)    ///< GPIO De-bounce register vase
#define GPIO_PIN_DATA_BASE    (AHBPERIPH_BASE + 0x04200)    ///< GPIO pin data register base address
#define GPIOBIT0_BASE         (AHBPERIPH_BASE + 0x04200)    ///< GPIO Port 0 bit access register base address
#define GPIOBIT1_BASE         (AHBPERIPH_BASE + 0x04220)    ///< GPIO Port 1 bit access register base address
#define GPIOBIT2_BASE         (AHBPERIPH_BASE + 0x04240)    ///< GPIO Port 2 bit access register base address
#define GPIOBIT3_BASE         (AHBPERIPH_BASE + 0x04260)    ///< GPIO Port 3 bit access register base address
#define GPIOBIT4_BASE         (AHBPERIPH_BASE + 0x04280)    ///< GPIO Port 4 bit access register base address
#define GPIOBIT5_BASE         (AHBPERIPH_BASE + 0x042A0)    ///< GPIO Port 5 bit access register base address
#define FMC_BASE              (AHBPERIPH_BASE + 0x0C000)    ///< FMC register base address

/*@}*/ /* end of group NUC029FAE_PERIPHERAL_MEM_MAP */


/** @addtogroup NUC029FAE_PERIPHERAL_DECLARATION NUC029FAE Peripheral Pointer
  The Declaration of NUC029FAE Peripheral
  @{
 */
#define WDT                   ((WDT_T *) WDT_BASE)              ///< Pointer to WDT register structure
#define TIMER0                ((TIMER_T *) TIMER0_BASE)         ///< Pointer to Timer 0 register structure
#define TIMER1                ((TIMER_T *) TIMER1_BASE)         ///< Pointer to Timer 1 register structure
#define I2C                   ((I2C_T *) I2C_BASE)              ///< Pointer to I2C register structure
#define SPI                   ((SPI_T *) SPI_BASE)              ///< Pointer to SPI register structure
#define PWM                   ((PWM_T *) PWM_BASE)              ///< Pointer to PWM register structure
#define UART                  ((UART_T *) UART_BASE)            ///< Pointer to UART register structure
#define ADC                   ((ADC_T *) ADC_BASE)              ///< Pointer to ADC register structure
#define ACMP                  ((ACMP_T *) ACMP_BASE)            ///< Pointer to ACMP register structure

#define SYS                   ((SYS_T *) SYS_BASE)              ///< Pointer to SYS register structure
#define CLK                   ((CLK_T *) CLK_BASE)              ///< Pointer to CLK register structure
#define INT                   ((INT_T *) INT_BASE)              ///< Pointer to INT register structure
#define P0                    ((GPIO_T *) P0_BASE)              ///< Pointer to GPIO port 0 register structure
#define P1                    ((GPIO_T *) P1_BASE)              ///< Pointer to GPIO port 1 register structure 
#define P2                    ((GPIO_T *) P2_BASE)              ///< Pointer to GPIO port 2 register structure
#define P3                    ((GPIO_T *) P3_BASE)              ///< Pointer to GPIO port 3 register structure
#define P4                    ((GPIO_T *) P4_BASE)              ///< Pointer to GPIO port 4 register structure
#define P5                    ((GPIO_T *) P5_BASE)              ///< Pointer to GPIO port 5 register structure
#define GPIO                  ((GPIO_DBNCECON_T *) GPIO_DBNCECON_BASE)      ///< Pointer to GPIO de-bounce register structure
#define GPIOBIT0              ((GPIOBIT_T *) GPIOBIT0_BASE)     ///< Pointer to GPIO port 0 bit access register structure
#define GPIOBIT1              ((GPIOBIT_T *) GPIOBIT1_BASE)     ///< Pointer to GPIO port 1 bit access register structure
#define GPIOBIT2              ((GPIOBIT_T *) GPIOBIT2_BASE)     ///< Pointer to GPIO port 2 bit access register structure
#define GPIOBIT3              ((GPIOBIT_T *) GPIOBIT3_BASE)     ///< Pointer to GPIO port 3 bit access register structure
#define GPIOBIT4              ((GPIOBIT_T *) GPIOBIT4_BASE)     ///< Pointer to GPIO port 4 bit access register structure
#define GPIOBIT5              ((GPIOBIT_T *) GPIOBIT5_BASE)     ///< Pointer to GPIO port 5 bit access register structure
#define FMC                   ((FMC_T *) FMC_BASE)              ///< Pointer to FMC register structure

/*@}*/ /* end of group NUC029FAE_PERIPHERAL_DECLARATION */

/** @addtogroup NUC029FAE_IO_ROUTINE NUC029FAE I/O routines
  The Declaration of NUC029FAE I/O routines
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
#define outpw(port,value)     *((volatile unsigned int *)(port)) = value

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
#define outps(port,value)     *((volatile unsigned short *)(port)) = value

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
#define outpb(port,value)     *((volatile unsigned char *)(port)) = value

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
#define outp32(port,value)    *((volatile unsigned int *)(port)) = value

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
#define outp16(port,value)    *((volatile unsigned short *)(port)) = value

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
#define outp8(port,value)     *((volatile unsigned char *)(port)) = value

/**
  * @brief Get a 8-bit unsigned value from specified I/O port
  * @param[in] port Port address to get 8-bit data from
  * @return  8-bit unsigned value stored in specified I/O port
  */
#define inp8(port)            (*((volatile unsigned char *)(port)))


/*@}*/ /* end of group NUC029FAE_IO_ROUTINE */

/******************************************************************************/
/*                Legacy Constants                                            */
/******************************************************************************/
/** @addtogroup NUC029FAE_legacy_Constants NUC029FAE Legacy Constants
  NUC029FAE Legacy Constants
  @{
*/

#ifndef NULL
#define NULL           (0)      ///< NULL pointer
#endif

#define TRUE           (1)      ///< Boolean true, define to use in API parameters or return value
#define FALSE          (0)      ///< Boolean false, define to use in API parameters or return value

#define ENABLE         (1)      ///< Enable, define to use in API parameters
#define DISABLE        (0)      ///< Disable, define to use in API parameters

/* Define one bit mask */
#define BIT0     (0x00000001)       ///< Bit 0 mask of an 32 bit integer
#define BIT1     (0x00000002)       ///< Bit 1 mask of an 32 bit integer
#define BIT2     (0x00000004)       ///< Bit 2 mask of an 32 bit integer
#define BIT3     (0x00000008)       ///< Bit 3 mask of an 32 bit integer
#define BIT4     (0x00000010)       ///< Bit 4 mask of an 32 bit integer
#define BIT5     (0x00000020)       ///< Bit 5 mask of an 32 bit integer
#define BIT6     (0x00000040)       ///< Bit 6 mask of an 32 bit integer
#define BIT7     (0x00000080)       ///< Bit 7 mask of an 32 bit integer
#define BIT8     (0x00000100)       ///< Bit 8 mask of an 32 bit integer
#define BIT9     (0x00000200)       ///< Bit 9 mask of an 32 bit integer
#define BIT10    (0x00000400)       ///< Bit 10 mask of an 32 bit integer
#define BIT11    (0x00000800)       ///< Bit 11 mask of an 32 bit integer
#define BIT12    (0x00001000)       ///< Bit 12 mask of an 32 bit integer
#define BIT13    (0x00002000)       ///< Bit 13 mask of an 32 bit integer
#define BIT14    (0x00004000)       ///< Bit 14 mask of an 32 bit integer
#define BIT15    (0x00008000)       ///< Bit 15 mask of an 32 bit integer
#define BIT16    (0x00010000)       ///< Bit 16 mask of an 32 bit integer
#define BIT17    (0x00020000)       ///< Bit 17 mask of an 32 bit integer
#define BIT18    (0x00040000)       ///< Bit 18 mask of an 32 bit integer
#define BIT19    (0x00080000)       ///< Bit 19 mask of an 32 bit integer
#define BIT20    (0x00100000)       ///< Bit 20 mask of an 32 bit integer
#define BIT21    (0x00200000)       ///< Bit 21 mask of an 32 bit integer
#define BIT22    (0x00400000)       ///< Bit 22 mask of an 32 bit integer
#define BIT23    (0x00800000)       ///< Bit 23 mask of an 32 bit integer
#define BIT24    (0x01000000)       ///< Bit 24 mask of an 32 bit integer
#define BIT25    (0x02000000)       ///< Bit 25 mask of an 32 bit integer
#define BIT26    (0x04000000)       ///< Bit 26 mask of an 32 bit integer
#define BIT27    (0x08000000)       ///< Bit 27 mask of an 32 bit integer
#define BIT28    (0x10000000)       ///< Bit 28 mask of an 32 bit integer
#define BIT29    (0x20000000)       ///< Bit 29 mask of an 32 bit integer
#define BIT30    (0x40000000)       ///< Bit 30 mask of an 32 bit integer
#define BIT31    (0x80000000)       ///< Bit 31 mask of an 32 bit integer

/* Byte Mask Definitions */
#define BYTE0_Msk              (0x000000FF)         ///< Mask to get bit0~bit7 from a 32 bit integer
#define BYTE1_Msk              (0x0000FF00)         ///< Mask to get bit8~bit15 from a 32 bit integer
#define BYTE2_Msk              (0x00FF0000)         ///< Mask to get bit16~bit23 from a 32 bit integer
#define BYTE3_Msk              (0xFF000000)         ///< Mask to get bit24~bit31 from a 32 bit integer

#define GET_BYTE0(u32Param)    ((u32Param & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define GET_BYTE1(u32Param)    ((u32Param & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define GET_BYTE2(u32Param)    ((u32Param & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define GET_BYTE3(u32Param)    ((u32Param & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/*@}*/ /* end of group NUC029FAE_legacy_Constants */

#ifdef __cplusplus
}
#endif


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "sys.h"
#include "clk.h"
#include "acmp.h"
#include "adc.h"
#include "fmc.h"
#include "gpio.h"
#include "i2c.h"
#include "pwm.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include "wdt.h"

#endif  // __NUC029FAE_H__

