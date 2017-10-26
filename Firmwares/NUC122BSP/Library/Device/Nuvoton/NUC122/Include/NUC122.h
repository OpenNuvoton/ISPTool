/**************************************************************************//**
 * @file     NUC122.h
 * @version  V3.0
 * $Revision: 52 $
 * $Date: 15/07/24 7:15p $
 * @brief    NUC122 Series Peripheral Access Layer Header File
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

/**
  \mainpage Introduction
  *
  *
  * This user manual describes the usage of NUC122 Series MCU device driver
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
  * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
  */


#ifndef __NUC122_H__
#define __NUC122_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
*/

/** @addtogroup MCU_CMSIS Device Definitions for CMSIS
  Interrupt Number Definition and Configurations for CMSIS
  @{
*/

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */

typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
    NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                             */
    HardFault_IRQn              = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                     */
    SVCall_IRQn                 = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                       */
    PendSV_IRQn                 = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                       */
    SysTick_IRQn                = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                   */

    /******  ARMIKMCU Swift specific Interrupt Numbers ************************************************/
    BOD_IRQn                  = 0,        /*!< Brown-Out Low Voltage Detected Interrupt             */
    WDT_IRQn                  = 1,        /*!< Watch Dog Timer Interrupt                            */
    EINT0_IRQn                = 2,        /*!< EINT0 Interrupt                                      */
    EINT1_IRQn                = 3,        /*!< EINT1 Interrupt                                      */
    GPAB_IRQn                 = 4,        /*!< GPIO_PA/PB Interrupt                                 */
    GPCD_IRQn                 = 5,        /*!< GPIO_PC/PD Interrupt                                 */
    PWMA_IRQn                 = 6,        /*!< PWMA Interrupt                                       */
    TMR0_IRQn                 = 8,        /*!< TIMER0 Interrupt                                     */
    TMR1_IRQn                 = 9,        /*!< TIMER1 Interrupt                                     */
    TMR2_IRQn                 = 10,       /*!< TIMER2 Interrupt                                     */
    TMR3_IRQn                 = 11,       /*!< TIMER3 Interrupt                                     */
    UART0_IRQn                = 12,       /*!< UART0 Interrupt                                      */
    UART1_IRQn                = 13,       /*!< UART1 Interrupt                                      */
    SPI0_IRQn                 = 14,       /*!< SPI0 Interrupt                                       */
    SPI1_IRQn                 = 15,       /*!< SPI1 Interrupt                                       */
    I2C1_IRQn                 = 19,       /*!< I2C1 Interrupt                                       */
    USBD_IRQn                 = 23,       /*!< USB device Interrupt                                 */
    PS2_IRQn                  = 24,       /*!< PS/2 device Interrupt                                */
    PWRWU_IRQn                = 28,       /*!< Power Down Wake Up Interrupt                         */
    RTC_IRQn                  = 31        /*!< RTC Interrupt                                        */
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


/*@}*/ /* end of group MCU_CMSIS */


#include "core_cm0.h"                   /* Cortex-M0 processor and core peripherals               */
#include "system_NUC122.h"              /* NUC122 System                                          */

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Peripheral Register
  Peripheral Control Registers
  @{
 */

/*---------------------------- Clock Controller ------------------------------*/
/** @addtogroup REG_CLK System Clock Controller (CLK)
  Memory Mapped Structure for System Clock Controller
  @{
 */
typedef struct
{

    /**
     * PWRCON
     * ===================================================================================================
     * Offset: 0x00  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |XTL12M_EN |External 4~24 MHz High Speed Crystal Enable (HXT) Control (Write Protect)
     * |        |          |The bit default value is set by flash controller user configuration register CONFIG0 [26:24].
     * |        |          |When the default clock source is from external 4~24 MHz high speed crystal, this bit is set to 1 automatically.
     * |        |          |0 = External 4~24 MHz high speed crystal oscillator (HXT) Disabled.
     * |        |          |1 = External 4~24 MHz high speed crystal oscillator (HXT) Enabled.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[1]     |XTL32K_EN |External 32.768 KHz Low Speed Crystal Enable (LXT) Control (Write Protect)
     * |        |          |0 = External 32.768 kHz low speed crystal oscillator (LXT) Disabled.
     * |        |          |1 = External 32.768 kHz low speed crystal oscillator (LXT) Enabled (Normal operation).
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[2]     |OSC22M_EN |Internal 22.1184 MHz High Speed Oscillator (HIRC) Enable Control (Write Protect)
     * |        |          |0 = Internal 22.1184 MHz high speed oscillator (HIRC) Disabled.
     * |        |          |1 = Internal 22.1184 MHz high speed oscillator (HIRC) Enabled.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[3]     |OSC10K_EN |Internal 10 KHz Low Speed Oscillator (LIRC) Enable Control (Write Protect)
     * |        |          |0 = Internal 10 kHz low speed oscillator (LIRC) Disabled.
     * |        |          |1 = Internal 10 kHz low speed oscillator (LIRC) Enabled.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[4]     |PD_WU_DLY |Wake-up Delay Counter Enable Control (Write Protect)
     * |        |          |When the chip wakes up from Power-down mode, the clock control will delay certain clock cycles to wait system clock stable.
     * |        |          |The delayed clock cycle is 4096 clock cycles when chip work at external 4~24 MHz high speed crystal, and 256 clock cycles when chip work at internal 22.1184 MHz high speed oscillator.
     * |        |          |0 = Clock cycles delay Disabled.
     * |        |          |1 = Clock cycles delay Enabled.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[5]     |PD_WU_INT_EN|Power-Down Mode Wake-Up Interrupt Enable Control (Write Protect)
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note1: The interrupt will occur when both PD_WU_STS and PD_WU_INT_EN are high.
     * |        |          |Note2: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[6]     |PD_WU_STS |Power-Down Mode Wake-Up Interrupt Status
     * |        |          |Set by "Power-down wake-up event", it indicates that resume from Power-down mode. The flag is set if the GPIO, USB, UART, WDT, BOD or RTC wake-up occurred. Write 1 to clear the bit to 0.
     * |        |          |Note: This bit is working only if PD_WU_INT_EN (PWRCON[5]) set to 1.
     * |[7]     |PWR_DOWN_EN|System Power-down Enable Bit (Write Protect)
     * |        |          |When this bit is set to 1, Power-down mode is enabled and chip Power-down behavior will depends on the PD_WAIT_CPU bit.
     * |        |          |(a) If the PD_WAIT_CPU is 0, then the chip enters Power-down mode immediately after the PWR_DOWN_EN bit set.
     * |        |          |(b) if the PD_WAIT_CPU is 1, then the chip keeps active till the CPU sleep mode is also active and then the chip enters Power-down mode (recommend)
     * |        |          |When chip wakes up from Power-down mode, this bit is cleared by hardware. User needs to set this bit again for next Power-down.
     * |        |          |In Power-down mode, external 4~24 MHz high speed crystal oscillator and the internal 22.1184 MHz high speed oscillator will be disabled in this mode, but the external 32.768 kHz low speed crystal and internal 10 kHz low speed oscillator are not controlled by Power-down mode.
     * |        |          |In Power- down mode, the PLL and system clock are disabled, and ignored the clock source selection.
     * |        |          |The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from external 32.768 kHz low speed crystal oscillator or the internal 10 kHz low speed oscillator.
     * |        |          |0 = Chip operating normally or chip in Idle mode because of WFI command.
     * |        |          |1 = Chip enters Power-down mode instantly or waits CPU sleep command WFI.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[8]     |PD_WAIT_CPU|This Bit Control The Power-Down Entry Condition (Write Protect)
     * |        |          |0 = Chip enters Power-down mode when the PWR_DOWN_EN bit is set to 1.
     * |        |          |1 = Chip enters Power- down mode when the both PD_WAIT_CPU and PWR_DOWN_EN bits are set to 1 and CPU run WFI instruction.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     */
    __IO uint32_t PWRCON;

    /**
    * AHBCLK
    * ===================================================================================================
    * Offset: 0x04  AHB Devices Clock Enable Control Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[2]     |ISP_EN    |Flash ISP Controller Clock Enable Control
    * |        |          |0 = Flash ISP peripheral clock Disabled.
    * |        |          |1 = Flash ISP peripheral clock Enabled.
    */
    __IO uint32_t AHBCLK;

    /**
     * APBCLK
     * ===================================================================================================
     * Offset: 0x08  APB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDT_EN    |Watchdog Timer Clock Enable Control (Write Protect)
     * |        |          |0 = Watchdog Timer clock Disabled.
     * |        |          |1 = Watchdog Timer clock Enabled.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[1]     |RTC_EN    |Real-Time-Clock APB Interface Clock Enable Control
     * |        |          |This bit is used to control the RTC APB clock only, The RTC peripheral clock source is from the external 32.768 kHz low speed crystal.
     * |        |          |0 = RTC clock Disabled.
     * |        |          |1 = RTC clock Enabled.     
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
     * |[9]     |I2C1_EN   |I2C1 Clock Enable Control
     * |        |          |0 = I2C1 clock Disabled.
     * |        |          |1 = I2C1 clock Enabled.
     * |[12]    |SPI0_EN   |SPI0 Clock Enable Control
     * |        |          |0 = SPI0 clock Disabled.
     * |        |          |1 = SPI0 clock Enabled.
     * |[13]    |SPI1_EN   |SPI1 Clock Enable Control
     * |        |          |0 = SPI1 clock Disabled.
     * |        |          |1 = SPI1 clock Enabled.
     * |[16]    |UART0_EN  |UART0 Clock Enable Control
     * |        |          |0 = UART0 clock Disabled.
     * |        |          |1 = UART0 clock Enabled.
     * |[17]    |UART1_EN  |UART1 Clock Enable Control
     * |        |          |0 = UART1 clock Disabled.
     * |        |          |1 = UART1 clock Enabled.
     * |[20]    |PWM01_EN  |PWM_01 Clock Enable Control
     * |        |          |0 = PWM01 clock Disabled.
     * |        |          |1 = PWM01 clock Enabled.
     * |[21]    |PWM23_EN  |PWM_23 Clock Enable Control
     * |        |          |0 = PWM23 clock Disabled.
     * |        |          |1 = PWM23 clock Enabled.
     * |[27]    |USBD_EN   |USB 2.0 FS Device Controller Clock Enable Control
     * |        |          |0 = USB clock Disabled.
     * |        |          |1 = USB clock Enabled.
     * |[31]    |PS2_EN    |PS/2 Clock Enable Control
     * |        |          |0 = PS/2 clock Disabled.
     * |        |          |1 = PS/2 clock Enabled.
     */
    __IO uint32_t APBCLK;

    /**
     * CLKSTATUS
     * ===================================================================================================
     * Offset: 0x0C  Clock status monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |XTL12M_STB|External 4~24 MHz High Speed Crystal (HXT) Clock Source Stable Flag (Read Only)
     * |        |          |0 = External 4~24 MHz high speed crystal clock (HXT) is not stable or disabled.
     * |        |          |1 = External 4~24 MHz high speed crystal clock (HXT) is stable.
     * |[1]     |XTL32K_STB|External 32.768 KHz Low Speed Crystal (LXT) Clock Source Stable Flag(Read Only)
     * |        |          |0 = External 32.768 kHz low speed crystal (LXT) clock is not stable or disabled.
     * |        |          |1 = External 32.768 kHz low speed crystal (LXT) clock is stable. 
     * |[2]     |PLL_STB   |Internal PLL Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal PLL clock is not stable or disabled.
     * |        |          |1 = Internal PLL clock is stable.
     * |[3]     |OSC10K_STB|Internal 10 KHz Low Speed Oscillator (LIRC) Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal 10 kHz low speed oscillator clock (LIRC) is not stable or disabled.
     * |        |          |1 = Internal 10 kHz low speed oscillator clock (LIRC) is stable.
     * |[4]     |OSC22M_STB|Internal 22.1184 MHz High Speed Oscillator (HIRC) Clock Source Stable Flag (Read Only)
     * |        |          |0 = Internal 22.1184 MHz high speed oscillator (HIRC) clock is not stable or disabled.
     * |        |          |1 = Internal 22.1184 MHz high speed oscillator (HIRC) clock is stable.
     * |[7]     |CLK_SW_FAIL|Clock Switching Fail Flag (Read Only)
     * |        |          |0 = Clock switching success.
     * |        |          |1 = Clock switching failure.
     * |        |          |This bit is an index that if current system clock source is match as user defined at HCLK_S (CLKSEL0[2:0]).
     * |        |          |When user switch system clock, the system clock source will keep old clock until the new clock is stable.
     * |        |          |During the period that waiting new clock stable, this bit will be an index shows system clock source is not match as user wanted.
     */
    __IO uint32_t CLKSTATUS;

    /**
     * CLKSEL0
     * ===================================================================================================
     * Offset: 0x10  Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HCLK_S    |HCLK Clock Source Select (Write Protect)
     * |        |          |1. Before clock switching, the related clock sources (both pre-select and new-select) must be turn on.
     * |        |          |2. The 3-bit default value is reloaded from the value of CFOSC (CONFIG0[26:24]) in user configuration register of Flash controller by any reset.
     * |        |          |Therefore the default value is either 000b or 111b.
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |001 = Clock source from external 32.768 kHz low speed crystal clock.      
     * |        |          |010 = Clock source from PLL clock.
     * |        |          |011 = Clock source from internal 10 kHz low speed oscillator clock.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |Note: These bits are write protected bit. Refer to the REGWRPROT register.
     * |[5:3]   |STCLK_S   |Cortex-M0 SysTick Clock Source Select (Write Protect)
     * |        |          |If SYST_CSR[2] = 1, SysTick clock source is from HCLK.
     * |        |          |If SYST_CSR[2] = 0, SysTick clock source is defined by STCLK_S (CLKSEL0[5:3]).
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
     * |        |          |001 = Clock source from external 32.768 kHz low speed crystal clock.      
     * |        |          |010 = Clock source from external 4~24 MHz high speed crystal clock/2.
     * |        |          |011 = Clock source from HCLK/2.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock/2.
     * |        |          |Note: These bits are write protected bit. Refer to the REGWRPROT register.
     * |        |          |Note2: if SysTick clock source is not from HCLK (i.e. SYST_CSR[2] = 0), SysTick clock source must less than or equal to HCLK/2.
     */
    __IO uint32_t CLKSEL0;

    /**
     * CLKSEL1
     * ===================================================================================================
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDT_S     |Watchdog Timer Clock Source Select (Write Protect)
     * |        |          |10 = Clock source from HCLK/2048 clock.
     * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * |[4]     |SPI0_S    |SPI0 Clock Source Selection
     * |        |          |0 = Clock source from PLL clock.
     * |        |          |1 = Clock source from HCLK.
     * |[5]     |SPI1_S    |SPI1 Clock Source Selection
     * |        |          |0 = Clock source from PLL clock.
     * |        |          |1 = Clock source from HCLK.
     * |[10:8]  |TMR0_S    |TIMER0 Clock Source Selection
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
     * |        |          |001 = Clock source from external 32.768 kHz low speed crystal clock.     
     * |        |          |010 = Clock source from HCLK.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |Others = Reserved.
     * |[14:12] |TMR1_S    |TIMER1 Clock Source Selection
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
     * |        |          |001 = Clock source from external 32.768 kHz low speed crystal clock.     
     * |        |          |010 = Clock source from HCLK.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |Others = Reserved.
     * |[18:16] |TMR2_S    |TIMER2 Clock Source Selection
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
     * |        |          |001 = Clock source from external 32.768 kHz low speed crystal clock.     
     * |        |          |010 = Clock source from HCLK.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |Others = Reserved.
     * |[22:20] |TMR3_S    |TIMER3 Clock Source Selection
     * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
     * |        |          |001 = Clock source from external 32.768 kHz low speed crystal clock.     
     * |        |          |010 = Clock source from HCLK.
     * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |        |          |Others = Reserved.
     * |[25:24] |UART_S    |UART Clock Source Selection
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |01 = Clock source from PLL clock.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |[29:28] |PWM01_S   |PWM0 and PWM1 Clock Source Selection
     * |        |          |PWM0 and PWM1 used the same peripheral clock source; both of them used the same prescaler.
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |01 = Clock source from external 32.768 kHz low speed crystal clock.     
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     * |[31:30] |PWM23_S   |PWM2 and PWM3 Clock Source Selection
     * |        |          |PWM2 and PWM3 used the same peripheral clock source; both of them used the same prescaler.
     * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
     * |        |          |01 = Clock source from external 32.768 kHz low speed crystal clock.       
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
     */
    __IO uint32_t CLKSEL1;

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x18  Clock Divider Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLK_N    |HCLK Clock Divide Number From HCLK Clock Source
     * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLK_N + 1).
     * |[7:4]   |USB_N     |USB Clock Divide Number From PLL Clock
     * |        |          |USB clock frequency = (PLL frequency) / (USB_N + 1).
     * |[11:8]  |UART_N    |UART Clock Divide Number From UART Clock Source
     * |        |          |UART clock frequency = (UART clock source frequency) / (UART_N + 1).
     */
    __IO uint32_t CLKDIV;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE0;
    /**
     * @endcond
     */

    /**
     * PLLCON
     * ===================================================================================================
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
     * |[16]    |PD        |Power-down Mode
     * |        |          |If the PWR_DOWN_EN bit is set to 1 in PWRCON register, the PLL will enter Power-down mode too.
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
     */
    __IO uint32_t PLLCON;


} CLK_T;

/** @addtogroup REG_CLK_BITMASK CLK Bit Mask
  @{
 */

/* CLK PWRCON Bit Field Definitions */
#define CLK_PWRCON_PD_WAIT_CPU_Pos           8                                    /*!< CLK_T::PWRCON: PD_WAIT_CPU Position */
#define CLK_PWRCON_PD_WAIT_CPU_Msk           (1ul << CLK_PWRCON_PD_WAIT_CPU_Pos)  /*!< CLK_T::PWRCON: PD_WAIT_CPU Mask */

#define CLK_PWRCON_PWR_DOWN_EN_Pos           7                                    /*!< CLK_T::PWRCON: PWR_DOWN_EN Position */
#define CLK_PWRCON_PWR_DOWN_EN_Msk           (1ul << CLK_PWRCON_PWR_DOWN_EN_Pos)  /*!< CLK_T::PWRCON: PWR_DOWN_EN Mask */

#define CLK_PWRCON_PD_WU_STS_Pos             6                                    /*!< CLK_T::PWRCON: PD_WU_STS Position */
#define CLK_PWRCON_PD_WU_STS_Msk             (1ul << CLK_PWRCON_PD_WU_STS_Pos)    /*!< CLK_T::PWRCON: PD_WU_STS Mask */

#define CLK_PWRCON_PD_WU_INT_EN_Pos          5                                    /*!< CLK_T::PWRCON: PD_WU_INT_EN Position */
#define CLK_PWRCON_PD_WU_INT_EN_Msk          (1ul << CLK_PWRCON_PD_WU_INT_EN_Pos) /*!< CLK_T::PWRCON: PD_WU_INT_EN Mask */

#define CLK_PWRCON_PD_WU_DLY_Pos             4                                    /*!< CLK_T::PWRCON: PD_WU_DLY Position */
#define CLK_PWRCON_PD_WU_DLY_Msk             (1ul << CLK_PWRCON_PD_WU_DLY_Pos)    /*!< CLK_T::PWRCON: PD_WU_DLY Mask */

#define CLK_PWRCON_OSC10K_EN_Pos             3                                    /*!< CLK_T::PWRCON: OSC10K_EN Position */
#define CLK_PWRCON_OSC10K_EN_Msk             (1ul << CLK_PWRCON_OSC10K_EN_Pos)    /*!< CLK_T::PWRCON: OSC10K_EN Mask */
#define CLK_PWRCON_IRC10K_EN_Pos             3                                    /*!< CLK_T::PWRCON: IRC10K_EN Position */
#define CLK_PWRCON_IRC10K_EN_Msk             (1ul << CLK_PWRCON_IRC10K_EN_Pos)    /*!< CLK_T::PWRCON: IRC10K_EN Mask */

#define CLK_PWRCON_OSC22M_EN_Pos             2                                    /*!< CLK_T::PWRCON: OSC22M_EN Position */
#define CLK_PWRCON_OSC22M_EN_Msk             (1ul << CLK_PWRCON_OSC22M_EN_Pos)    /*!< CLK_T::PWRCON: OSC22M_EN Mask */
#define CLK_PWRCON_IRC22M_EN_Pos             2                                    /*!< CLK_T::PWRCON: IRC22M_EN Position */
#define CLK_PWRCON_IRC22M_EN_Msk             (1ul << CLK_PWRCON_IRC22M_EN_Pos)    /*!< CLK_T::PWRCON: IRC22M_EN Mask */

#define CLK_PWRCON_XTL32K_EN_Pos             1                                    /*!< CLK_T::PWRCON: XTL32K_EN Position */
#define CLK_PWRCON_XTL32K_EN_Msk             (1ul << CLK_PWRCON_XTL32K_EN_Pos)    /*!< CLK_T::PWRCON: XTL32K_EN Mask */

#define CLK_PWRCON_XTL12M_EN_Pos             0                                    /*!< CLK_T::PWRCON: XTL12M_EN Position */
#define CLK_PWRCON_XTL12M_EN_Msk             (1ul << CLK_PWRCON_XTL12M_EN_Pos)    /*!< CLK_T::PWRCON: XTL12M_EN Mask */

/* CLK AHBCLK Bit Field Definitions */
#define CLK_AHBCLK_ISP_EN_Pos                2                                    /*!< CLK_T::AHBCLK: ISP_EN Position */
#define CLK_AHBCLK_ISP_EN_Msk                (1ul << CLK_AHBCLK_ISP_EN_Pos)       /*!< CLK_T::AHBCLK: ISP_EN Mask */

/* CLK APBCLK Bit Field Definitions */
#define CLK_APBCLK_PS2_EN_Pos                31                                   /*!< CLK_T::APBCLK: PS2_EN Position */
#define CLK_APBCLK_PS2_EN_Msk                (1ul << CLK_APBCLK_PS2_EN_Pos)       /*!< CLK_T::APBCLK: PS2_EN Mask */

#define CLK_APBCLK_USBD_EN_Pos               27                                   /*!< CLK_T::APBCLK: USBD_EN Position */
#define CLK_APBCLK_USBD_EN_Msk               (1ul << CLK_APBCLK_USBD_EN_Pos)      /*!< CLK_T::APBCLK: USBD_EN Mask */

#define CLK_APBCLK_PWM23_EN_Pos              21                                   /*!< CLK_T::APBCLK: PWM23_EN Position */
#define CLK_APBCLK_PWM23_EN_Msk              (1ul << CLK_APBCLK_PWM23_EN_Pos)     /*!< CLK_T::APBCLK: PWM23_EN Mask */

#define CLK_APBCLK_PWM01_EN_Pos              20                                   /*!< CLK_T::APBCLK: PWM01_EN Position */
#define CLK_APBCLK_PWM01_EN_Msk              (1ul << CLK_APBCLK_PWM01_EN_Pos)     /*!< CLK_T::APBCLK: PWM01_EN Mask */

#define CLK_APBCLK_UART1_EN_Pos              17                                   /*!< CLK_T::APBCLK: UART1_EN Position */
#define CLK_APBCLK_UART1_EN_Msk              (1ul << CLK_APBCLK_UART1_EN_Pos)     /*!< CLK_T::APBCLK: UART1_EN Mask */

#define CLK_APBCLK_UART0_EN_Pos              16                                   /*!< CLK_T::APBCLK: UART0_EN Position */
#define CLK_APBCLK_UART0_EN_Msk              (1ul << CLK_APBCLK_UART0_EN_Pos)     /*!< CLK_T::APBCLK: UART0_EN Mask */

#define CLK_APBCLK_SPI1_EN_Pos               13                                   /*!< CLK_T::APBCLK: SPI1_EN Position */
#define CLK_APBCLK_SPI1_EN_Msk               (1ul << CLK_APBCLK_SPI1_EN_Pos)      /*!< CLK_T::APBCLK: SPI1_EN Mask */

#define CLK_APBCLK_SPI0_EN_Pos               12                                   /*!< CLK_T::APBCLK: SPI0_EN Position */
#define CLK_APBCLK_SPI0_EN_Msk               (1ul << CLK_APBCLK_SPI0_EN_Pos)      /*!< CLK_T::APBCLK: SPI0_EN Mask */

#define CLK_APBCLK_I2C1_EN_Pos               9                                    /*!< CLK_T::APBCLK: I2C1_EN Position */
#define CLK_APBCLK_I2C1_EN_Msk               (1ul << CLK_APBCLK_I2C1_EN_Pos)      /*!< CLK_T::APBCLK: I2C1_EN Mask */

#define CLK_APBCLK_TMR3_EN_Pos               5                                    /*!< CLK_T::APBCLK: TMR3_EN Position */
#define CLK_APBCLK_TMR3_EN_Msk               (1ul << CLK_APBCLK_TMR3_EN_Pos)      /*!< CLK_T::APBCLK: TMR3_EN Mask */

#define CLK_APBCLK_TMR2_EN_Pos               4                                    /*!< CLK_T::APBCLK: TMR2_EN Position */
#define CLK_APBCLK_TMR2_EN_Msk               (1ul << CLK_APBCLK_TMR2_EN_Pos)      /*!< CLK_T::APBCLK: TMR2_EN Mask */

#define CLK_APBCLK_TMR1_EN_Pos               3                                    /*!< CLK_T::APBCLK: TMR1_EN Position */
#define CLK_APBCLK_TMR1_EN_Msk               (1ul << CLK_APBCLK_TMR1_EN_Pos)      /*!< CLK_T::APBCLK: TMR1_EN Mask */

#define CLK_APBCLK_TMR0_EN_Pos               2                                    /*!< CLK_T::APBCLK: TMR0_EN Position */
#define CLK_APBCLK_TMR0_EN_Msk               (1ul << CLK_APBCLK_TMR0_EN_Pos)      /*!< CLK_T::APBCLK: TMR0_EN Mask */

#define CLK_APBCLK_RTC_EN_Pos                1                                    /*!< CLK_T::APBCLK: RTC_EN Position */
#define CLK_APBCLK_RTC_EN_Msk                (1ul << CLK_APBCLK_RTC_EN_Pos)       /*!< CLK_T::APBCLK: RTC_EN Mask */

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

#define CLK_CLKSTATUS_XTL32K_STB_Pos         1                                        /*!< CLK_T::CLKSTATUS: XTL32K_STB Position */
#define CLK_CLKSTATUS_XTL32K_STB_Msk         (1ul << CLK_CLKSTATUS_XTL32K_STB_Pos)    /*!< CLK_T::CLKSTATUS: XTL32K_STB Mask */

#define CLK_CLKSTATUS_XTL12M_STB_Pos         0                                        /*!< CLK_T::CLKSTATUS: XTL12M_STB Position */
#define CLK_CLKSTATUS_XTL12M_STB_Msk         (1ul << CLK_CLKSTATUS_XTL12M_STB_Pos)    /*!< CLK_T::CLKSTATUS: XTL12M_STB Mask */

/* CLK CLKSEL0 Bit Field Definitions */
#define CLK_CLKSEL0_STCLK_S_Pos              3                                        /*!< CLK_T::CLKSEL0: STCLK_S Position */
#define CLK_CLKSEL0_STCLK_S_Msk              (7ul << CLK_CLKSEL0_STCLK_S_Pos)         /*!< CLK_T::CLKSEL0: STCLK_S Mask */

#define CLK_CLKSEL0_HCLK_S_Pos               0                                        /*!< CLK_T::CLKSEL0: HCLK_S Position */
#define CLK_CLKSEL0_HCLK_S_Msk               (7ul << CLK_CLKSEL0_HCLK_S_Pos)          /*!< CLK_T::CLKSEL0: HCLK_S Mask */

/* CLK CLKSEL1 Bit Field Definitions */
#define CLK_CLKSEL1_PWM23_S_Pos              30                                       /*!< CLK_T::CLKSEL1: PWM23_S Position */
#define CLK_CLKSEL1_PWM23_S_Msk              (3ul << CLK_CLKSEL1_PWM23_S_Pos)         /*!< CLK_T::CLKSEL1: PWM23_S Mask */

#define CLK_CLKSEL1_PWM01_S_Pos              28                                       /*!< CLK_T::CLKSEL1: PWM01_S Position */
#define CLK_CLKSEL1_PWM01_S_Msk              (3ul << CLK_CLKSEL1_PWM01_S_Pos)         /*!< CLK_T::CLKSEL1: PWM01_S Mask */

#define CLK_CLKSEL1_UART_S_Pos               24                                       /*!< CLK_T::CLKSEL1: UART_S Position */
#define CLK_CLKSEL1_UART_S_Msk               (3ul << CLK_CLKSEL1_UART_S_Pos)          /*!< CLK_T::CLKSEL1: UART_S Mask */

#define CLK_CLKSEL1_TMR3_S_Pos               20                                       /*!< CLK_T::CLKSEL1: TMR3_S Position */
#define CLK_CLKSEL1_TMR3_S_Msk               (7ul << CLK_CLKSEL1_TMR3_S_Pos)          /*!< CLK_T::CLKSEL1: TMR3_S Mask */

#define CLK_CLKSEL1_TMR2_S_Pos               16                                       /*!< CLK_T::CLKSEL1: TMR2_S Position */
#define CLK_CLKSEL1_TMR2_S_Msk               (7ul << CLK_CLKSEL1_TMR2_S_Pos)          /*!< CLK_T::CLKSEL1: TMR2_S Mask */

#define CLK_CLKSEL1_TMR1_S_Pos               12                                       /*!< CLK_T::CLKSEL1: TMR1_S Position */
#define CLK_CLKSEL1_TMR1_S_Msk               (7ul << CLK_CLKSEL1_TMR1_S_Pos)          /*!< CLK_T::CLKSEL1: TMR1_S Mask */

#define CLK_CLKSEL1_TMR0_S_Pos               8                                        /*!< CLK_T::CLKSEL1: TMR0_S Position */
#define CLK_CLKSEL1_TMR0_S_Msk               (7ul << CLK_CLKSEL1_TMR0_S_Pos)          /*!< CLK_T::CLKSEL1: TMR0_S Mask */

#define CLK_CLKSEL1_SPI1_S_Pos               5                                        /*!< CLK_T::CLKSEL1: SPI1_S Position */
#define CLK_CLKSEL1_SPI1_S_Msk               (1ul << CLK_CLKSEL1_SPI1_S_Pos)          /*!< CLK_T::CLKSEL1: SPI1_S Mask */

#define CLK_CLKSEL1_SPI0_S_Pos               4                                        /*!< CLK_T::CLKSEL1: SPI0_S Position */
#define CLK_CLKSEL1_SPI0_S_Msk               (1ul << CLK_CLKSEL1_SPI0_S_Pos)          /*!< CLK_T::CLKSEL1: SPI0_S Mask */

#define CLK_CLKSEL1_WDT_S_Pos                0                                        /*!< CLK_T::CLKSEL1: WDT_S Position */
#define CLK_CLKSEL1_WDT_S_Msk                (3ul << CLK_CLKSEL1_WDT_S_Pos)           /*!< CLK_T::CLKSEL1: WDT_S Mask */

/* CLK CLKDIV Bit Field Definitions */
#define CLK_CLKDIV_UART_N_Pos                8                                        /*!< CLK_T::CLKDIV: UART_N Position */
#define CLK_CLKDIV_UART_N_Msk                (0xFul << CLK_CLKDIV_UART_N_Pos)         /*!< CLK_T::CLKDIV: UART_N Mask */

#define CLK_CLKDIV_USB_N_Pos                 4                                        /*!< CLK_T::CLKDIV: USB_N Position */
#define CLK_CLKDIV_USB_N_Msk                 (0xFul << CLK_CLKDIV_USB_N_Pos)          /*!< CLK_T::CLKDIV: USB_N Mask */

#define CLK_CLKDIV_HCLK_N_Pos                0                                        /*!< CLK_T::CLKDIV: HCLK_N Position */
#define CLK_CLKDIV_HCLK_N_Msk                (0xFul << CLK_CLKDIV_HCLK_N_Pos)         /*!< CLK_T::CLKDIV: HCLK_N Mask */

/* CLK PLLCON Bit Field Definitions */
#define CLK_PLLCON_PLL_SRC_Pos               19                                       /*!< CLK_T::PLLCON: PLL_SRC Position */
#define CLK_PLLCON_PLL_SRC_Msk               (1ul << CLK_PLLCON_PLL_SRC_Pos)          /*!< CLK_T::PLLCON: PLL_SRC Mask */

#define CLK_PLLCON_OE_Pos                    18                                       /*!< CLK_T::PLLCON: PLL_SRC Position */
#define CLK_PLLCON_OE_Msk                    (1ul << CLK_PLLCON_OE_Pos)               /*!< CLK_T::PLLCON: PLL_SRC Mask */

#define CLK_PLLCON_BP_Pos                    17                                       /*!< CLK_T::PLLCON: OE Position */
#define CLK_PLLCON_BP_Msk                    (1ul << CLK_PLLCON_BP_Pos)               /*!< CLK_T::PLLCON: OE Mask */

#define CLK_PLLCON_PD_Pos                    16                                       /*!< CLK_T::PLLCON: PD Position */
#define CLK_PLLCON_PD_Msk                    (1ul << CLK_PLLCON_PD_Pos)               /*!< CLK_T::PLLCON: PD Mask */

#define CLK_PLLCON_OUT_DV_Pos                14                                       /*!< CLK_T::PLLCON: OUT_DV Position */
#define CLK_PLLCON_OUT_DV_Msk                (3ul << CLK_PLLCON_OUT_DV_Pos)           /*!< CLK_T::PLLCON: OUT_DV Mask */

#define CLK_PLLCON_IN_DV_Pos                 9                                        /*!< CLK_T::PLLCON: IN_DV Position */
#define CLK_PLLCON_IN_DV_Msk                 (0x1Ful << CLK_PLLCON_IN_DV_Pos)         /*!< CLK_T::PLLCON: IN_DV Mask */

#define CLK_PLLCON_FB_DV_Pos                 0                                        /*!< CLK_T::PLLCON: FB_DV Position */
#define CLK_PLLCON_FB_DV_Msk                 (0x1FFul << CLK_PLLCON_FB_DV_Pos)        /*!< CLK_T::PLLCON: FB_DV Mask */


/*@}*/ /* end of group REG_CLK_BITMASK */
/*@}*/ /* end of group REG_CLK */


/*-------------------------- FLASH Memory Controller -------------------------*/
/** @addtogroup REG_FMC FLASH Memory Controller (FMC)
  Memory Mapped Structure for Flash Memory Controller
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
     * |        |          |Note: This bit is write protected.
     * |[1]     |BS        |Boot Select
     * |        |          |Set/clear this bit to select next booting from LDROM/APROM,
     * |        |          |respectively. This bit also functions as MCU booting status flag, which can be used to check where
     * |        |          |MCU booted from. This bit is initiated with the inverted value of CBS in Config0 after power-
     * |        |          |on reset; It keeps the same value at other reset.
     * |        |          |1 = boot from LDROM
     * |        |          |0 = boot from APROM
     * |        |          |Note: This bit is write protected.    
     * |[4]     |CFGUEN    |Config Update Enable
     * |        |          |Writing this bit to 1 enables s/w to update Config value by ISP procedure regardless of program
     * |        |          |code is running in APROM or LDROM.
     * |        |          |1 = Config update enable
     * |        |          |0 = Config update disable
     * |        |          |Note: This bit is write protected.       
     * |[5]     |LDUEN     |LDROM Update Enable
     * |        |          |LDROM update enable bit.
     * |        |          |1 = LDROM can be updated when the MCU runs in APROM.
     * |        |          |0 = LDROM cannot be updated
     * |        |          |Note2: This bit is the protected bit.        
     * |[6]     |ISPFF     |ISP Fail Flag
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself.
     * |        |          |(2) LDROM writes to itself.
     * |        |          |(3) Destination address is illegal, such as over an available range.
     * |        |          |Note1: Write 1 to clear this bit.
     * |        |          |Note2: This bit is the protected bit.      
     * |[10:8]  |PT        |Flash Program Time
     * |        |          |000 = The program time is 40us.
     * |        |          |001 = The program time is 45us.   
     * |        |          |010 = The program time is 50us.
     * |        |          |011 = The program time is 55us.
     * |        |          |100 = The program time is 20us.
     * |        |          |101 = The program time is 25us.   
     * |        |          |110 = The program time is 30us.
     * |        |          |111 = The program time is 35us.  
     * |        |          |Note: This bit is write protected.    
     * |[14:12] |PT        |Flash Erase Time
     * |        |          |000 = The erase time is 20us(default).
     * |        |          |001 = The erase time is 25us.   
     * |        |          |010 = The erase time is 30us.
     * |        |          |011 = The erase time is 35us.
     * |        |          |100 = The erase time is 3us.
     * |        |          |101 = The erase time is 5us.   
     * |        |          |110 = The erase time is 10us.
     * |        |          |111 = The erase time is 15us.  
     * |        |          |Note: This bit is write protected.      
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
     * |        |          |it supports word program only. ISPARD[1:0] must be kept 2'b00 for ISP operation.
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
     * |[6:0]   |CMD       |ISP Command
     * |        |          |ISP command table is shown below:
     * |        |          |0x00 = Read.
     * |        |          |0x21 = Program.
     * |        |          |0x22 = Page Erase.
     */
    __IO uint32_t ISPCMD;

    /**
     * ISPTRG
     * ===================================================================================================
     * Offset: 0x10  IISP Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP Start Trigger
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP
     * |        |          |operation is finish.
     * |        |          |1 = ISP is on going
     * |        |          |0 = ISP operation is finished
     */
    __IO uint32_t ISPTRG;

    /**
     * @cond HIDDEN_SYMBOLS
     * RESERVED0
     * ===================================================================================================
     *
     * ---------------------------------------------------------------------------------------------------
     */
    __I  uint32_t RESERVED0[1];
    /**
     * @endcond
     */

    /**
     * FATCON
     * ===================================================================================================
     * Offset: 0x18  Flash Access Time Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4]     |LFOM      |Low Frequency Optimization Mode
     * |        |          |If chip operation frequency lower than 20MHz, chip can work more efficiently when this bit is set to 1.
     * |        |          |If chip operation frequency is >40MHz, both of LFOM and MFOM have to set to zero.
     * |        |          |Note: This bit is write protected.       
     * |[6]     |MFOM      |Middle Frequency Optimization Mode
     * |        |          |If chip operation frequency is between 20MHz~40MHz, chip can work more efficiently when this bit is set to 1.
     * |        |          |If chip operation frequency is >40MHz, both of LFOM and MFOM have to set to zero.   
     * |        |          |Note: This bit is write protected.       
     */
    __IO uint32_t FATCON;

} FMC_T;

/** @addtogroup REG_FMC_BITMASK FMC Bit Mask
  @{
 */

/* FMC ISPCON Bit Field Definitions */
#define FMC_ISPCON_ET_Pos                       12                                      /*!< FMC_T::ISPCON: ET Position */
#define FMC_ISPCON_ET_Msk                       (7ul << FMC_ISPCON_ET_Pos)              /*!< FMC_T::ISPCON: ET Mask     */

#define FMC_ISPCON_PT_Pos                       8                                       /*!< FMC_T::ISPCON: PT Position     */
#define FMC_ISPCON_PT_Msk                       (7ul << FMC_ISPCON_PT_Pos)              /*!< FMC_T::ISPCON: PT Mask         */

#define FMC_ISPCON_ISPFF_Pos                    6                                       /*!< FMC_T::ISPCON: ISPFF Position  */
#define FMC_ISPCON_ISPFF_Msk                    (1ul << FMC_ISPCON_ISPFF_Pos)           /*!< FMC_T::ISPCON: ISPFF Mask      */

#define FMC_ISPCON_LDUEN_Pos                    5                                       /*!< FMC_T::ISPCON: LDUEN Position  */
#define FMC_ISPCON_LDUEN_Msk                    (1ul << FMC_ISPCON_LDUEN_Pos)           /*!< FMC_T::ISPCON: LDUEN Mask      */

#define FMC_ISPCON_CFGUEN_Pos                   4                                       /*!< FMC_T::ISPCON: CFGUEN Position */
#define FMC_ISPCON_CFGUEN_Msk                   (1ul << FMC_ISPCON_CFGUEN_Pos)          /*!< FMC_T::ISPCON: CFGUEN Mask     */

#define FMC_ISPCON_BS_Pos                       1                                       /*!< FMC_T::ISPCON: BS Position     */
#define FMC_ISPCON_BS_Msk                       (1ul << FMC_ISPCON_BS_Pos)              /*!< FMC_T::ISPCON: BS Mask         */

#define FMC_ISPCON_ISPEN_Pos                    0                                       /*!< FMC_T::ISPCON: ISPEN Position  */
#define FMC_ISPCON_ISPEN_Msk                    (1ul << FMC_ISPCON_ISPEN_Pos)           /*!< FMC_T::ISPCON: ISPEN Mask      */

/* FMC ISPADR Bit Field Definitions */
#define FMC_ISPADR_ISPADR_Pos                   0                                       /*!< FMC_T::ISPADR: ISPADR Position */
#define FMC_ISPADR_ISPADR_Msk                   (0xFFFFFFFFul << FMC_ISPADR_ISPADR_Pos) /*!< FMC_T::ISPADR: ISPADR Mask     */

/* FMC ISPADR Bit Field Definitions */
#define FMC_ISPDAT_ISPDAT_Pos                   0                                       /*!< FMC_T::ISPDAT: ISPDAT Position */
#define FMC_ISPDAT_ISPDAT_Msk                   (0xFFFFFFFFul << FMC_ISPDAT_ISPDAT_Pos) /*!< FMC_T::ISPDAT: ISPDAT Mask     */

/* FMC ISPCMD Bit Field Definitions */
#define FMC_ISPCMD_CMD_Pos                      0                                       /*!< FMC_T::ISPCMD: CMD Position    */
#define FMC_ISPCMD_CMD_Msk                      (0x7Ful << FMC_ISPCMD_CMD_Pos)          /*!< FMC_T::ISPCMD: CMD Mask        */

/* FMC ISPTRG Bit Field Definitions */
#define FMC_ISPTRG_ISPGO_Pos                    0                                       /*!< FMC_T::ISPTRG: ISPGO Position  */
#define FMC_ISPTRG_ISPGO_Msk                    (1ul << FMC_ISPTRG_ISPGO_Pos)           /*!< FMC_T::ISPTRG: ISPGO Mask      */

/* FMC FATCON Bit Field Definitions */
#define FMC_FATCON_MFOM_Pos                     6                                       /*!< FMC_T::FATCON: MFOM Position   */
#define FMC_FATCON_MFOM_Msk                     (1ul << FMC_FATCON_MFOM_Pos)            /*!< FMC_T::FATCON: MFOM Mask       */

#define FMC_FATCON_LFOM_Pos                     4                                       /*!< FMC_T::FATCON: LFOM Position   */
#define FMC_FATCON_LFOM_Msk                     (1ul << FMC_FATCON_LFOM_Pos)            /*!< FMC_T::FATCON: LFOM Mask       */

/*@}*/ /* end of group REG_FMC_BITMASK */
/*@}*/ /* end of group REG_FMC */



/*--------------------- General Purpose I/O (GPIO) ---------------------*/
/** @addtogroup REG_GPIO General Purpose I/O (GPIO)
  Memory Mapped Structure for General Purpose I/O
  @{
 */
typedef struct
{
    /**
     * GPIOx_PMD
     * ===================================================================================================
     * Offset: 0x00/0x40/0x80/0xC0  GPIO Port [A/B/C/D] Pin I/O Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2n+1:2n]|PMDn     |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O mode of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in Input mode.
     * |        |          |01 = GPIO port [n] pin is in Push-pull Output mode.
     * |        |          |10 = GPIO port [n] pin is in Open-drain Output mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~14, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     */
    __IO uint32_t  PMD;

    /**
     * GPIOx_OFFD
     * ===================================================================================================
     * Offset: 0x04/0x44/0x84/0xC4  GPIO Port [A/B/C/D] Pin Digital Input Path Disable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:16] |OFFD      |GPIOx Pin[n] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding GPIO pin is disabled.
     * |        |          |If input is analog signal, users can disable GPIO digital input path to avoid current leakage.
     * |        |          |0 = I/O digital input path Enabled.
     * |        |          |1 = I/O digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~14, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     */
    __IO uint32_t  OFFD;

    /**
     * GPIOx_DOUT
     * ===================================================================================================
     * Offset: 0x08/0x48/0x88/0xC8  GPIO Port [A/B/C/D] Data Output Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |DOUTn     |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as Push-pull output, open-drain output or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B/C/D] Pin[n] will drive Low if the GPIO pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = GPIO port [A/B/C/D] Pin[n] will drive High if the GPIO pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~14, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     */
    __IO uint32_t  DOUT;

    /**
     * GPIOx_DMASK
     * ===================================================================================================
     * Offset: 0x0C/0x4C/0x8C/0xCC  GPIO Port [A/B/C/D] Data Output Write Mask
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |DMASKn    |Port [A/B/C/D] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of GPIOx_DOUT bit[n].
     * |        |          |When the DMASK bit[n] is set to 1, the corresponding GPIOx_DOUT[n] bit is protected.
     * |        |          |If the write signal is masked, write data to the protect bit is ignored.
     * |        |          |0 = Corresponding GPIOx_DOUT[n] bit can be updated.
     * |        |          |1 = Corresponding GPIOx_DOUT[n] bit protected.
     * |        |          |Note1: This function only protects the corresponding GPIOx_DOUT[n] bit, and will not protect the corresponding bit control register (GPIOAx_DOUT, GPIOBx_DOUT, GPIOCx_DOUT, GPIODx_DOUT, GPIOEx_DOUT and GPIOFx_DOUT).
     * |        |          |Note2:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~14, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     */
    __IO uint32_t  DMASK;

    /**
     * GPIOx_PIN
     * ===================================================================================================
     * Offset: 0x10/0x50/0x90/0xD0  GPIO Port [A/B/C/D] Pin Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |PINn      |Port [A/B/C/D] Pin Values
     * |        |          |Each bit of the register reflects the actual status of the respective GPIO pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high, else the pin status is low.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~14, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     */
    __I  uint32_t  PIN;

    /**
     * GPIOx_DBEN
     * ===================================================================================================
     * Offset: 0x14/0x54/0x94/0xD4  GPIO Port [A/B/C/D] De-bounce Enable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |DBENn     |Port [A/B/C/D] Input Signal De-Bounce Enable
     * |        |          |DBEN[n] is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBNCECON[4], one de-bounce sample cycle period is controlled by DBNCECON[3:0].
     * |        |          |0 = Bit[n] de-bounce function Disabled.
     * |        |          |1 = Bit[n] de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~14, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     */
    __IO uint32_t  DBEN;

    /**
     * GPIOA_IMD
     * ===================================================================================================
     * Offset: 0x18/0x58/0x98/0xD8  GPIO Port [A/B/C/D] Interrupt Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |IMDn      |Port [A/B/C/D] Edge Or Level Detection Interrupt Control
     * |        |          |IMD[n] is used to control the interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK.
     * |        |          |clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers GPIOx_IEN.
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~14, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     */
    __IO uint32_t  IMD;

    /**
     * GPIOx_IEN
     * ===================================================================================================
     * Offset: 0x1C/0x5C/0x9C/0xDC  GPIO Port [A/B/C/D] Interrupt Enable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |IF_ENn    |Port [A/B/C/D] Interrupt Enable By Input Falling Edge or Input Level Low
     * |        |          |IF_EN[n] is used to enable the interrupt for each of the corresponding input GPIO_PIN[n].
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the IF_EN[n] bit to 1:
     * |        |          |If the interrupt is level trigger, the input PIN[n] state at level "low" will generate the interrupt.
     * |        |          |If the interrupt is edge trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.
     * |        |          |0 = PIN[n] state low-level or high-to-low change interrupt Disabled.
     * |        |          |1 = PIN[n] state low-level or high-to-low change interrupt Enabled.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~14, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     * |[n+16]  |IR_ENn    |Port [A/B/C/D] Interrupt Enable By Input Rising Edge or Input Level High
     * |        |          |IR_EN[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the IR_EN[n] bit to 1:
     * |        |          |If the interrupt is level trigger, the input PIN[n] state at level "high" will generate the interrupt.
     * |        |          |If the interrupt is edge trigger, the input PIN[n] state change from "low-to-high" will generate the interrupt.
     * |        |          |0 = PIN[n] level-high or low-to-high interrupt Disabled.
     * |        |          |1 = PIN[n] level-high or low-to-high interrupt Enabled.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~14, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     */
    __IO uint32_t  IEN;

    /**
     * GPIOx_ISRC
     * ===================================================================================================
     * Offset: 0x20/0x60/0xA0/0xE0  GPIO Port [A/B/C/D] Interrupt Source Flag
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[n]     |ISRCn     |Port [A/B/C/D] Interrupt Source Flag
     * |        |          |Read :
     * |        |          |0 = No interrupt at GPIOx[n].
     * |        |          |1 = GPIOx[n] generates an interrupt.
     * |        |          |Write :
     * |        |          |0= No action.
     * |        |          |1= Clear the corresponding pending interrupt.
     * |        |          |Note:
     * |        |          |GPIOA: valid n are 15~10. Others are reserved.
     * |        |          |GPIOB: valid n are 15~14, 10~0. Others are reserved.
     * |        |          |GPIOC: valid n are 13~8, 5~0. Others are reserved.
     * |        |          |GPIOD: valid n are 11~8, 5~0. Others are reserved.
     */
    __IO uint32_t  ISRC;

} GPIO_T;

typedef struct
{
    /**
     * DBNCECON
     * ===================================================================================================
     * Offset: 0x180  External Interrupt De-bounce Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |DBCLKSEL  |De-Bounce Sampling Cycle Selection
     * |        |          |0000 = Sample interrupt input once per 1 clocks.
     * |        |          |0001 = Sample interrupt input once per 2 clocks.
     * |        |          |0010 = Sample interrupt input once per 4 clocks.
     * |        |          |0011 = Sample interrupt input once per 8 clocks.
     * |        |          |0100 = Sample interrupt input once per 16 clocks.
     * |        |          |0101 = Sample interrupt input once per 32 clocks.
     * |        |          |0110 = Sample interrupt input once per 64 clocks.
     * |        |          |0111 = Sample interrupt input once per 128 clocks.
     * |        |          |1000 = Sample interrupt input once per 256 clocks.
     * |        |          |1001 = Sample interrupt input once per 2*256 clocks.
     * |        |          |1010 = Sample interrupt input once per 4*256clocks.
     * |        |          |1011 = Sample interrupt input once per 8*256 clocks.
     * |        |          |1100 = Sample interrupt input once per 16*256 clocks.
     * |        |          |1101 = Sample interrupt input once per 32*256 clocks.
     * |        |          |1110 = Sample interrupt input once per 64*256 clocks.
     * |        |          |1111 = Sample interrupt input once per 128*256 clocks.
     * |[4]     |DBCLKSRC  |De-Bounce Counter Clock Source Selection
     * |        |          |0 = De-bounce counter clock source is the HCLK.
     * |        |          |1 = De-bounce counter clock source is the internal 10 kHz low speed oscillator.
     * |[5]     |ICLK_ON   |Interrupt Clock On Mode
     * |        |          |0 = Edge detection circuit is active only if I/O pin corresponding GPIOx_IEN bit is set to 1.
     * |        |          |1 = All I/O pins edge detection circuit is always active after reset.
     * |        |          |It is recommended to turn off this bit to save system power if no special application concern.
     */
    __IO uint32_t  DBNCECON;
} GPIO_DBNCECON_T;

/** @addtogroup REG_GPIO_BITMASK GPIO Bit Mask
  @{
 */

/* GPIO PMD Bit Field Definitions */
#define GPIO_PMD_PMD15_Pos          30                                          /*!< GPIO_T::PMD: PMD15 Position */
#define GPIO_PMD_PMD15_Msk          (0x3ul << GPIO_PMD_PMD15_Pos)               /*!< GPIO_T::PMD: PMD15 Mask */

#define GPIO_PMD_PMD14_Pos          28                                          /*!< GPIO_T::PMD: PMD14 Position */
#define GPIO_PMD_PMD14_Msk          (0x3ul << GPIO_PMD_PMD14_Pos)               /*!< GPIO_T::PMD: PMD14 Mask */

#define GPIO_PMD_PMD13_Pos          26                                          /*!< GPIO_T::PMD: PMD13 Position */
#define GPIO_PMD_PMD13_Msk          (0x3ul << GPIO_PMD_PMD13_Pos)               /*!< GPIO_T::PMD: PMD13 Mask */

#define GPIO_PMD_PMD12_Pos          24                                          /*!< GPIO_T::PMD: PMD12 Position */
#define GPIO_PMD_PMD12_Msk          (0x3ul << GPIO_PMD_PMD12_Pos)               /*!< GPIO_T::PMD: PMD12 Mask */

#define GPIO_PMD_PMD11_Pos          22                                          /*!< GPIO_T::PMD: PMD11 Position */
#define GPIO_PMD_PMD11_Msk          (0x3ul << GPIO_PMD_PMD11_Pos)               /*!< GPIO_T::PMD: PMD11 Mask */

#define GPIO_PMD_PMD10_Pos          20                                          /*!< GPIO_T::PMD: PMD10 Position */
#define GPIO_PMD_PMD10_Msk          (0x3ul << GPIO_PMD_PMD10_Pos)               /*!< GPIO_T::PMD: PMD10 Mask */

#define GPIO_PMD_PMD9_Pos           18                                          /*!< GPIO_T::PMD: PMD9 Position */
#define GPIO_PMD_PMD9_Msk           (0x3ul << GPIO_PMD_PMD9_Pos)                /*!< GPIO_T::PMD: PMD9 Mask */

#define GPIO_PMD_PMD8_Pos           16                                          /*!< GPIO_T::PMD: PMD8 Position */
#define GPIO_PMD_PMD8_Msk           (0x3ul << GPIO_PMD_PMD8_Pos)                /*!< GPIO_T::PMD: PMD8 Mask */

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
#define GPIO_OFFD_OFFD_Msk          (0xFFFFul << GPIO_OFFD_OFFD_Pos)            /*!< GPIO_T::OFFD: OFFD Mask */

/* GPIO DOUT Bit Field Definitions */
#define GPIO_DOUT_DOUT_Pos          0                                           /*!< GPIO_T::DOUT: DOUT Position */
#define GPIO_DOUT_DOUT_Msk          (0xFFFFul << GPIO_DOUT_DOUT_Pos)            /*!< GPIO_T::DOUT: DOUT Mask */

/* GPIO DMASK Bit Field Definitions */
#define GPIO_DMASK_DMASK_Pos        0                                           /*!< GPIO_T::DMASK: DMASK Position */
#define GPIO_DMASK_DMASK_Msk        (0xFFFFul << GPIO_DMASK_DMASK_Pos)          /*!< GPIO_T::DMASK: DMASK Mask */

/* GPIO PIN Bit Field Definitions */
#define GPIO_PIN_PIN_Pos            0                                           /*!< GPIO_T::PIN: PIN Position */
#define GPIO_PIN_PIN_Msk            (0xFFFFul << GPIO_PIN_PIN_Pos)              /*!< GPIO_T::PIN: PIN Mask */

/* GPIO DBEN Bit Field Definitions */
#define GPIO_DBEN_DBEN_Pos          0                                           /*!< GPIO_T::DBEN: DBEN Position */
#define GPIO_DBEN_DBEN_Msk          (0xFFFFul << GPIO_DBEN_DBEN_Pos)            /*!< GPIO_T::DBEN: DBEN Mask */

/* GPIO IMD Bit Field Definitions */
#define GPIO_IMD_IMD_Pos            0                                           /*!< GPIO_T::IMD: IMD Position */
#define GPIO_IMD_IMD_Msk            (0xFFFFul << GPIO_IMD_IMD_Pos)              /*!< GPIO_T::IMD: IMD Mask */

/* GPIO IEN Bit Field Definitions */
#define GPIO_IEN_IR_EN_Pos          16                                          /*!< GPIO_T::IEN: IR_EN Position */
#define GPIO_IEN_IR_EN_Msk          (0xFFFFul << GPIO_IEN_IR_EN_Pos)            /*!< GPIO_T::IEN: IR_EN Mask */

#define GPIO_IEN_IF_EN_Pos          0                                           /*!< GPIO_T::IEN: IF_EN Position */
#define GPIO_IEN_IF_EN_Msk          (0xFFFFul << GPIO_IEN_IF_EN_Pos)            /*!< GPIO_T::IEN: IF_EN Mask */

/* GPIO ISRC Bit Field Definitions */
#define GPIO_ISRC_ISRC_Pos          0                                           /*!< GPIO_T::ISRC: ISRC Position */
#define GPIO_ISRC_ISRC_Msk          (0xFFFFul << GPIO_ISRC_ISRC_Pos)            /*!< GPIO_T::ISRC: ISRC Mask */

/* GPIO DBNCECON Bit Field Definitions */
#define GPIO_DBNCECON_ICLK_ON_Pos   5                                           /*!< GPIO_T::DBNCECON: ICLK_ON  Position */
#define GPIO_DBNCECON_ICLK_ON_Msk   (1ul << GPIO_DBNCECON_ICLK_ON_Pos)          /*!< GPIO_T::DBNCECON: ICLK_ON  Mask */

#define GPIO_DBNCECON_DBCLKSRC_Pos  4                                           /*!< GPIO_T::DBNCECON: DBCLKSRC Position */
#define GPIO_DBNCECON_DBCLKSRC_Msk  (1ul << GPIO_DBNCECON_DBCLKSRC_Pos)         /*!< GPIO_T::DBNCECON: DBCLKSRC Mask */

#define GPIO_DBNCECON_DBCLKSEL_Pos  0                                           /*!< GPIO_T::DBNCECON: DBCLKSEL Position */
#define GPIO_DBNCECON_DBCLKSEL_Msk  (0xFul << GPIO_DBNCECON_DBCLKSEL_Pos)       /*!< GPIO_T::DBNCECON: DBCLKSEL Mask */
/*@}*/ /* end of group REG_GPIO_BITMASK */
/*@}*/ /* end of group REG_GPIO */

/*------------------------------ I2C Controller ------------------------------*/
/** @addtogroup REG_I2C Inter-IC Bus Controller (I2C)
  Memory Mapped Structure for I2C Serial Interface Controller
  @{
 */
typedef struct
{
    /**
     * I2CON
     * ===================================================================================================
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
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Set to enable I2C serial function controller.
     * |        |          |When ENS1=1 the I2C serial function enables.
     * |        |          |The multi-function pin function of I2Cn_SDA and I2Cn_SCL must set to I2C function first.
     * |[7]     |EI        |Enable Interrupt
     * |        |          |0 = I2C interrupt Disabled.
     * |        |          |1 = I2C interrupt Enabled.
     */
    __IO uint32_t I2CON;

    /**
     * I2CADDR0
     * ===================================================================================================
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
     */
    __IO uint32_t I2CADDR0;

    /**
     * I2CDAT
     * ===================================================================================================
     * Offset: 0x08  I2C Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2CDAT    |I2C Data Register
     * |        |          |Bit [7:0] is located with the 8-bit transferred data of I2C serial port.
     */
    __IO uint32_t I2CDAT;

    /**
     * I2CSTATUS
     * ===================================================================================================
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
     */
    __I  uint32_t I2CSTATUS;

    /**
     * I2CLK
     * ===================================================================================================
     * Offset: 0x10  I2C Clock Divided Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2CLK     |I2C Clock Divided Register
     * |        |          |The I2C clock rate bits: Data Baud Rate of I2C = (system clock) / (4x (I2CLK+1)).
     * |        |          |Note: The minimum value of I2CLK is 4.
     */
    __IO uint32_t I2CLK;

    /**
     * I2CTOC
     * ===================================================================================================
     * Offset: 0x14  I2C Time-out Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Time-out Flag
     * |        |          |This bit is set by H/W when I2C time-out happened and it can interrupt CPU if I2C interrupt enable bit (EI) is set to 1.
     * |        |          |Note: Write 1 to clear this bit.
     * |[1]     |DIV4      |Time-out Counter Input Clock Divided by 4
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When Enabled, The time-out period is extend 4 times.
     * |[2]     |ENTI      |Time-out Counter Enable/Disable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When Enabled, the 14-bit time-out counter will start counting when SI is clear.
     * |        |          |Setting flag SI to high will reset counter and re-start up counting after SI is cleared.
     */
    __IO uint32_t I2CTOC;

    /**
     * I2CADDR1
     * ===================================================================================================
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
     */
    __IO uint32_t I2CADDR1;

    /**
     * I2CADDR2
     * ===================================================================================================
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
     */
    __IO uint32_t I2CADDR2;

    /**
     * I2CADDR3
     * ===================================================================================================
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
     */
    __IO uint32_t I2CADDR3;

    /**
     * I2CADM0
     * ===================================================================================================
     * Offset: 0x24  I2C Slave Address Mask Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     */
    __IO uint32_t I2CADM0;

    /**
     * I2CADM1
     * ===================================================================================================
     * Offset: 0x28  I2C Slave Address Mask Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     */
    __IO uint32_t I2CADM1;

    /**
     * I2CADM2
     * ===================================================================================================
     * Offset: 0x2C  I2C Slave Address Mask Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     */
    __IO uint32_t I2CADM2;

    /**
     * I2CADM3
     * ===================================================================================================
     * Offset: 0x30  I2C Slave Address Mask Register3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Register
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care.
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     */
    __IO uint32_t I2CADM3;

} I2C_T;

/** @addtogroup REG_I2C_BITMASK I2C Bit Mask
  @{
 */

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
#define I2C_I2CADDR_I2CADDR_Pos                 1                                       /*!< I2C I2CADDR: I2CADDR Position */
#define I2C_I2CADDR_I2CADDR_Msk                 (0x7Ful << I2C_I2CADDR_I2CADDR_Pos)     /*!< I2C I2CADDR: I2CADDR Mask */

#define I2C_I2CADDR_GC_Pos                      0                                       /*!< I2C I2CADDR: GC Position */
#define I2C_I2CADDR_GC_Msk                      (1ul << I2C_I2CADDR_GC_Pos)             /*!< I2C I2CADDR: GC Mask */

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
#define I2C_I2CADM_I2CADM_Pos                   1                                       /*!< I2C I2CADM: I2CADM Position */
#define I2C_I2CADM_I2CADM_Msk                   (0x7Ful << I2C_I2CADM_I2CADM_Pos)       /*!< I2C I2CADM: I2CADM Mask */

/*@}*/ /* end of group REG_I2C_BITMASK */
/*@}*/ /* end of group REG_I2C */


/*------------------------------ PS2 Controller ------------------------------*/
/** @addtogroup REG_PS2 PS/2 Device Controller (PS2)
  Memory Mapped Structure for PS2 Serial Interface Controller
  @{
 */
typedef struct
{
    /**
     * PS2CON
     * ===================================================================================================
     * Offset: 0x00  PS/2 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PS2EN     |Enable PS/2 Device
     * |        |          |Enable PS/2 device controller
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[1]     |TXINTEN   |Enable Transmit Interrupt
     * |        |          |0 = Data transmit complete interrupt Disabled.
     * |        |          |1 = Data transmit complete interrupt Enabled.
     * |[2]     |RXINTEN   |Enable Receive Interrupt
     * |        |          |0 = Data receive complete interrupt Disabled.
     * |        |          |1 = Data receive complete interrupt Enabled.
     * |[6:3]   |TXFIFO_DEPTH|Transmit Data FIFO Depth
     * |        |          |There are 16 bytes buffer for data transmit.
     * |        |          |Software can define the FIFO depth from 1 to 16 bytes depends on application needs.
     * |        |          |0 = 1 byte.
     * |        |          |1 = 2 bytes.
     * |        |          |...
     * |        |          |14 = 15 bytes.
     * |        |          |15 = 16 bytes.
     * |[7]     |ACK       |Acknowledge Enable
     * |        |          |0 = Always send acknowledge to host at 12th clock for host to device communication.
     * |        |          |1 = If parity bit error or stop bit is not received correctly, acknowledge bit will not be sent to host at 12th clock.
     * |[8]     |CLRFIFO   |Clear TX FIFO
     * |        |          |Write 1 to this bit to terminate device to host transmission.
     * |        |          |The TXEMPTY(PS2STATUS[7]) bit will be set to 1 and pointer BYTEIDEX(PS2STATUS[11:8]) is reset to 0 regardless there is residue data in buffer or not.
     * |        |          |The buffer content is not been cleared.
     * |        |          |0 = Not active.
     * |        |          |1 = Clear FIFO.
     * |[9]     |OVERRIDE  |Software Override PS/2 CLK/DATA Pin State
     * |        |          |0 = PS2_CLK and PS2_DATA pins are controlled by internal state machine.
     * |        |          |1 = PS2_CLK and PS2_DATA pins are controlled by software.
     * |[10]    |FPS2CLK   |Force PS2CLK Line
     * |        |          |It forces PS2_CLK line high or low regardless of the internal state of the device controller if OVERRIDE(PS2CON[9]) is set to 1.
     * |        |          |0 = Force PS2_CLK line low.
     * |        |          |1 = Force PS2_CLK line high.
     * |[11]    |FPS2DAT   |Force PS2DATA Line
     * |        |          |It forces PS2_DATA high or low regardless of the internal state of the device controller if OVERRIDE (PS2CON[9]) is set to 1.
     * |        |          |0 = Force PS2_DATA low.
     * |        |          |1 = Force PS2_DATA high.
     */
    __IO uint32_t PS2CON;

    /**
     * PS2TXDATA0
     * ===================================================================================================
     * Offset: 0x04  PS/2 Transmit Data Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PS2TXDATAx|Transmit Data
     * |        |          |Writing data to this register starts in device to host communication if bus is in IDLE state.
     * |        |          |Software must enable PS2EN(PS2CON[0]) before writing data to TX buffer.
     */
    __IO uint32_t PS2TXDATA0;

    /**
     * PS2TXDATA1
     * ===================================================================================================
     * Offset: 0x08  PS/2 Transmit Data Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PS2TXDATAx|Transmit Data
     * |        |          |Writing data to this register starts in device to host communication if bus is in IDLE state.
     * |        |          |Software must enable PS2EN(PS2CON[0]) before writing data to TX buffer.
     */
    __IO uint32_t PS2TXDATA1;

    /**
     * PS2TXDATA2
     * ===================================================================================================
     * Offset: 0x0C  PS/2 Transmit Data Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PS2TXDATAx|Transmit Data
     * |        |          |Writing data to this register starts in device to host communication if bus is in IDLE state.
     * |        |          |Software must enable PS2EN(PS2CON[0]) before writing data to TX buffer.
     */
    __IO uint32_t PS2TXDATA2;

    /**
     * PS2TXDATA3
     * ===================================================================================================
     * Offset: 0x10  PS/2 Transmit Data Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PS2TXDATAx|Transmit Data
     * |        |          |Writing data to this register starts in device to host communication if bus is in IDLE state.
     * |        |          |Software must enable PS2EN(PS2CON[0]) before writing data to TX buffer.
     */
    __IO uint32_t PS2TXDATA3;

    /**
     * PS2RXDATA
     * ===================================================================================================
     * Offset: 0x14  PS/2 Receive Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RXDATA    |Received Data
     * |        |          |For host to device communication, after acknowledge bit is sent, the received data is copied from receive shift register to PS2RXDATA register.
     * |        |          |CPU must read this register before next byte reception complete, otherwise the data will be overwritten and RXOVF(PS2STATUS[6]) bit will be set to 1.
     */
    __IO uint32_t PS2RXDATA;

    /**
     * PS2STATUS
     * ===================================================================================================
     * Offset: 0x18  PS/2 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PS2CLK    |CLK Pin State
     * |        |          |This bit reflects the status of the PS2_CLK line after synchronizing.
     * |[1]     |PS2DATA   |DATA Pin State
     * |        |          |This bit reflects the status of the PS2_DATA line after synchronizing and sampling.
     * |[2]     |FRAMERR   |Frame Error
     * |        |          |For host to device communication, this bit sets to 1 if STOP bit (logic 1) is not received.
     * |        |          |If frame error occurs, the PS/2_DATA line may keep at low state after 12th clock.
     * |        |          |At this moment, software overrides PS2_CLK to send clock till PS2_DATA release to high state.
     * |        |          |After that, device sends a "Resend" command to host.
     * |        |          |0 = No frame error.
     * |        |          |1 = Frame error occur.
     * |        |          |Write 1 to clear this bit.
     * |[3]     |RXPARITY  |Received Parity
     * |        |          |This bit reflects the parity bit for the last received data byte (odd parity).
     * |        |          |This bit is read only.
     * |[4]     |RXBUSY    |Receive Busy
     * |        |          |This bit indicates that the PS/2 device is currently receiving data.
     * |        |          |0 = Idle.
     * |        |          |1 = Currently receiving data.
     * |        |          |This bit is read only.
     * |[5]     |TXBUSY    |Transmit Busy
     * |        |          |This bit indicates that the PS/2 device is currently sending data.
     * |        |          |0 = Idle.
     * |        |          |1 = Currently sending data.
     * |        |          |This bit is read only.
     * |[6]     |RXOVF     |RX Buffer Overwrite
     * |        |          |0 = No overwrite.
     * |        |          |1 = Data in PS2RXDATA register is overwritten by new received data.
     * |        |          |Write 1 to clear this bit.
     * |[7]     |TXEMPTY   |TX FIFO Empty
     * |        |          |When software writes data to PS2TXDATA0-3, the TXEMPTY bit is cleared to 0 immediately if PS2EN(PS2CON[0]) is enabled.
     * |        |          |When transmitted data byte number is equal to FIFODEPTH(PS2CON[6:3]) then TXEMPTY bit is set to 1.
     * |        |          |0 = There is data to be transmitted.
     * |        |          |1 = FIFO is empty.
     * |        |          |This bit is read only.
     * |[11:8]  |BYTEIDX   |Byte Index
     * |        |          |It indicates which data byte in transmit data shift register.
     * |        |          |When all data in FIFO is transmitted and it will be cleared to 0.
     * |        |          |This bit is read only.
     * |        |          |BYTEIDX,    DATA Transmit , BYTEIDX,    DATA Transmit
     * |        |          |0000   , PS2TXDATA0[ 7: 0], 1000   , PS2TXDATA2[ 7: 0],
     * |        |          |0001   , PS2TXDATA0[15: 8], 1001   , PS2TXDATA2[15: 8],
     * |        |          |0010   , PS2TXDATA0[23:16], 1010   , PS2TXDATA2[23:16],
     * |        |          |0011   , PS2TXDATA0[31:24], 1011   , PS2TXDATA2[31:24],
     * |        |          |0100   , PS2TXDATA1[ 7: 0], 1100   , PS2TXDATA3[ 7: 0],
     * |        |          |0101   , PS2TXDATA1[15: 8], 1101   , PS2TXDATA3[15: 8],
     * |        |          |0110   , PS2TXDATA1[23:16], 1110   , PS2TXDATA3[23:16],
     * |        |          |0111   , PS2TXDATA1[31:24], 1111   , PS2TXDATA3[31:24],
     */
    __IO uint32_t PS2STATUS;

    /**
     * PS2INTID
     * ===================================================================================================
     * Offset: 0x1C  PS/2 Interrupt Identification Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXINT     |Receive Interrupt
     * |        |          |This bit is set to 1 when acknowledge bit is sent for Host to device communication.
     * |        |          |Interrupt occurs if RXINTEN(PS2CON[2]) bit is set to 1.
     * |        |          |0 = No interrupt.
     * |        |          |1 = Receive interrupt occurs.
     * |        |          |Write 1 to clear this bit to 0.
     * |[1]     |TXINT     |Transmit Interrupt
     * |        |          |This bit is set to 1 after STOP bit is transmitted.
     * |        |          |Interrupt occur if TXINTEN(PS2CON[1]) bit is set to 1.
     * |        |          |0 = No interrupt.
     * |        |          |1 = Transmit interrupt occurs.
     * |        |          |Write 1 to clear this bit to 0.
     */
    __IO uint32_t PS2INTID;
} PS2_T;

/** @addtogroup REG_PS2_BITMASK PS2 Bit Mask
  @{
 */

/* PS2 PS2CON Bit Field Definitions */
#define PS2_PS2CON_PS2EN_Pos                       0                                        /*!< PS2_T::PS2CON: PS2EN Position */
#define PS2_PS2CON_PS2EN_Msk                       (1ul << PS2_PS2CON_PS2EN_Pos)            /*!< PS2_T::PS2CON: PS2EN Mask */

#define PS2_PS2CON_TXINTEN_Pos                     1                                        /*!< PS2_T::PS2CON: TXINTEN Position */
#define PS2_PS2CON_TXINTEN_Msk                     (1ul << PS2_PS2CON_TXINTEN_Pos)          /*!< PS2_T::PS2CON: TXINTEN Mask */

#define PS2_PS2CON_RXINTEN_Pos                     2                                        /*!< PS2_T::PS2CON: RXINTEN Position */
#define PS2_PS2CON_RXINTEN_Msk                     (1ul << PS2_PS2CON_RXINTEN_Pos)          /*!< PS2_T::PS2CON: RXINTEN Mask */

#define PS2_PS2CON_TXFIFO_DEPTH_Pos                3                                        /*!< PS2_T::PS2CON: TXFIFO_DEPTH Position */
#define PS2_PS2CON_TXFIFO_DEPTH_Msk                (0xFul << PS2_PS2CON_TXFIFO_DEPTH_Pos)   /*!< PS2_T::PS2CON: TXFIFO_DEPTH Mask */

#define PS2_PS2CON_ACK_Pos                         7                                        /*!< PS2_T::PS2CON: ACK Position */
#define PS2_PS2CON_ACK_Msk                         (1ul << PS2_PS2CON_ACK_Pos)              /*!< PS2_T::PS2CON: ACK Mask */

#define PS2_PS2CON_CLRFIFO_Pos                     8                                        /*!< PS2_T::PS2CON: CLRFIFO Position */
#define PS2_PS2CON_CLRFIFO_Msk                     (1ul << PS2_PS2CON_CLRFIFO_Pos)          /*!< PS2_T::PS2CON: CLRFIFO Mask */

#define PS2_PS2CON_OVERRIDE_Pos                    9                                        /*!< PS2_T::PS2CON: OVERRIDE Position */
#define PS2_PS2CON_OVERRIDE_Msk                    (1ul << PS2_PS2CON_OVERRIDE_Pos)         /*!< PS2_T::PS2CON: OVERRIDE Mask */

#define PS2_PS2CON_FPS2CLK_Pos                     10                                       /*!< PS2_T::PS2CON: FPS2CLK Position */
#define PS2_PS2CON_FPS2CLK_Msk                     (1ul << PS2_PS2CON_FPS2CLK_Pos)          /*!< PS2_T::PS2CON: FPS2CLK Mask */

#define PS2_PS2CON_FPS2DAT_Pos                     11                                       /*!< PS2_T::PS2CON: FPS2DAT Position */
#define PS2_PS2CON_FPS2DAT_Msk                     (1ul << PS2_PS2CON_FPS2DAT_Pos)          /*!< PS2_T::PS2CON: FPS2DAT Mask */

/* PS/2 PS2RXDATA Bit Field Definitions */
#define PS2_PS2RXDATA_RXDATA_Pos                   0                                        /*!< PS2_T::PS2RXDATA: RXDATA Position */
#define PS2_PS2RXDATA_RXDATA_Msk                   (0xFFul << PS2_PS2RXDATA_RXDATA_Pos)     /*!< PS2_T::PS2RXDATA: RXDATA Mask */

/* PS/2 PS2STATUS Bit Field Definitions */
#define PS2_PS2STATUS_PS2CLK_Pos                   0                                        /*!< PS2_T::PS2STATUS: PS2CLK Position */
#define PS2_PS2STATUS_PS2CLK_Msk                   (1ul << PS2_PS2STATUS_PS2CLK_Pos)        /*!< PS2_T::PS2STATUS: PS2CLK Mask */

#define PS2_PS2STATUS_PS2DATA_Pos                  1                                        /*!< PS2_T::PS2STATUS: PS2DATA Position */
#define PS2_PS2STATUS_PS2DATA_Msk                  (1ul << PS2_PS2STATUS_PS2DATA_Pos)       /*!< PS2_T::PS2STATUS: PS2DATA Mask */

#define PS2_PS2STATUS_FRAMERR_Pos                  2                                        /*!< PS2_T::PS2STATUS: FRAMERR Position */
#define PS2_PS2STATUS_FRAMERR_Msk                  (1ul << PS2_PS2STATUS_FRAMERR_Pos)       /*!< PS2_T::PS2STATUS: FRAMERR Mask */

#define PS2_PS2STATUS_RXPARITY_Pos                 3                                        /*!< PS2_T::PS2STATUS: RXPARITY Position */
#define PS2_PS2STATUS_RXPARITY_Msk                 (1ul << PS2_PS2STATUS_RXPARITY_Pos)      /*!< PS2_T::PS2STATUS: RXPARITY Mask */

#define PS2_PS2STATUS_RXBUSY_Pos                   4                                        /*!< PS2_T::PS2STATUS: RXBUSY Position */
#define PS2_PS2STATUS_RXBUSY_Msk                   (1ul << PS2_PS2STATUS_RXBUSY_Pos)        /*!< PS2_T::PS2STATUS: RXBUSY Mask */

#define PS2_PS2STATUS_TXBUSY_Pos                   5                                        /*!< PS2_T::PS2STATUS: TXBUSY Position */
#define PS2_PS2STATUS_TXBUSY_Msk                   (1ul << PS2_PS2STATUS_TXBUSY_Pos)        /*!< PS2_T::PS2STATUS: TXBUSY Mask */

#define PS2_PS2STATUS_RXOVF_Pos                    6                                        /*!< PS2_T::PS2STATUS: RXOVF Position */
#define PS2_PS2STATUS_RXOVF_Msk                    (1ul << PS2_PS2STATUS_RXOVF_Pos)         /*!< PS2_T::PS2STATUS: RXOVF Mask */

#define PS2_PS2STATUS_TXEMPTY_Pos                  7                                        /*!< PS2_T::PS2STATUS: TXEMPTY Position */
#define PS2_PS2STATUS_TXEMPTY_Msk                  (1ul << PS2_PS2STATUS_TXEMPTY_Pos)       /*!< PS2_T::PS2STATUS: TXEMPTY Mask */

#define PS2_PS2STATUS_BYTEIDX_Pos                  8                                        /*!< PS2_T::PS2STATUS: BYTEIDX Position */
#define PS2_PS2STATUS_BYTEIDX_Msk                  (0xFul << PS2_PS2STATUS_BYTEIDX_Pos)     /*!< PS2_T::PS2STATUS: BYTEIDX Mask */

/* PS/2 PS2INTID Bit Field Definitions */
#define PS2_PS2INTID_RXINT_Pos                     0                                        /*!< PS2_T::PS2INTID : RXINT Position */
#define PS2_PS2INTID_RXINT_Msk                     (1ul << PS2_PS2INTID_RXINT_Pos)          /*!< PS2_T::PS2INTID : RXINT Mask */

#define PS2_PS2INTID_TXINT_Pos                     1                                        /*!< PS2_T::PS2INTID : TXINT Position */
#define PS2_PS2INTID_TXINT_Msk                     (1ul << PS2_PS2INTID_TXINT_Pos)          /*!< PS2_T::PS2INTID : TXINT Mask */
/*@}*/ /* end of group REG_PS2_BITMASK */
/*@}*/ /* end of group REG_PS2 */

/*----------------------------- PWM Controller -------------------------------*/
/** @addtogroup REG_PWM PWM Generator and Capture Timer (PWM)
  Memory Mapped Structure for PWM Generator and Capture Timer
  @{
 */
typedef struct
{
    /**
     * PPR
     * ===================================================================================================
     * Offset: 0x00  PWM Prescaler Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits     |Field     |Descriptions
     * | :----:  | :----:   | :---- |
     * |[7:0]    |CP01      |Clock Prescaler 0 (PWM-Timer 0 / 1 For Group A)
     * |         |          |Clock input is divided by (CP01 + 1) before it is fed to the corresponding PWM-timer
     * |         |          |If CP01=0, then the clock prescaler 0 output clock will be stopped.
     * |         |          |So corresponding PWM-timer will also be stopped.
     * |[15:8]   |CP23      |Clock Prescaler 2 (PWM-Timer2 / 3 For Group A)
     * |         |          |Clock input is divided by (CP23 + 1) before it is fed to the corresponding PWM-timer
     * |         |          |If CP23=0, then the clock prescaler 2 output clock will be stopped.
     * |         |          |So corresponding PWM-timer will also be stopped.
     * |[23:16]  |DZI01     |Dead-Zone Interval For Pair Of Channel 0 And Channel 1 (PWM0 And PWM1 Pair For PWM Group A)
     * |         |          |These 8-bit determine the Dead-zone length.
     * |         |          |The unit time of Dead-zone length = [(prescale+1)*(clock source divider)]/ PWM01_CLK.
     * |[31:24]  |DZI23     |Dead-Zone Interval For Pair Of Channel2 And Channel3 (PWM2 And PWM3 Pair For PWM Group A)
     * |         |          |These 8-bit determine the Dead-zone length.
     * |         |          |The unit time of Dead-zone length = [(prescale+1)*(clock source divider)]/ PWM23_CLK.
     */
    __IO uint32_t PPR;

    /**
     * CSR
     * ===================================================================================================
     * Offset: 0x04  PWM Clock Source Divider Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits      |Field     |Descriptions
     * | :----:   | :----:   | :---- |
     * |[2:0]     |CSR0      |Timer 0 Clock Source Selection(PWM timer 0 for group A)
     * |          |          |Select clock input for timer.
     * |          |          |(Table is the same as CSR3)
     * |[6:4]     |CSR1      |Timer 1 Clock Source Selection(PWM timer 1 for group A)
     * |          |          |Select clock input for timer.
     * |          |          |(Table is the same as CSR3)
     * |[10:8]    |CSR2      |Timer 2 Clock Source Selection(PWM timer 2 for group A)
     * |          |          |Select clock input for timer.
     * |          |          |(Table is the same as CSR3)
     * |[14:12]   |CSR3      |Timer 3 Clock Source Selection (PWM timer 3 for group A)
     * |          |          |Select clock input for timer.
     * |          |          |CSRx[2:0] = Input clock divider
     * |          |          |000 = 2
     * |          |          |001 = 4
     * |          |          |010 = 8
     * |          |          |011 = 16
     * |          |          |100 = 1
     */
    __IO uint32_t CSR;

    /**
     * PCR
     * ===================================================================================================
     * Offset: 0x08  PWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits     |Field         |Descriptions
     * | :----:  | :----:       | :---- |
     * |[0]      |CH0EN         |PWM-Timer 0 Enable (PWM Timer 0 For Group A)
     * |         |              |0 = The corresponding PWM-Timer stops running.
     * |         |              |1 = The corresponding PWM-Timer starts running.
     * |[2]      |CH0INV        |PWM-Timer 0 Output Inverter Enable (PWM Timer 0 For Group A)
     * |         |              |0 = Inverter Disabled.
     * |         |              |1 = Inverter Enabled.
     * |[3]      |CH0MOD        |PWM-Timer 0 Auto-Reload/One-Shot Mode (PWM Timer 0 For Group A)
     * |         |              |0 = One-shot mode.
     * |         |              |1 = Auto-reload mode.
     * |         |              |Note: If there is a transition at this bit, it will cause CNR0 and CMR0 be cleared.
     * |[4]      |DZEN01        |Dead-Zone 0 Generator Enable (PWM0 And PWM1 Pair For PWM Group A)
     * |         |              |0 = Disabled.
     * |         |              |1 = Enabled.
     * |         |              |Note: When Dead-zone generator is enabled, the pair of PWM0 and PWM1 becomes a complementary pair for PWM group A.
     * |[5]      |DZEN23        |Dead-Zone 2 Generator Enable (PWM2 And PWM3 Pair For PWM Group A)
     * |         |              |0 = Disabled.
     * |         |              |1 = Enabled.
     * |         |              |Note: When Dead-zone generator is enabled, the pair of PWM2 and PWM3 becomes a complementary pair for PWM group A.
     * |[8]      |CH1EN         |PWM-Timer 1 Enable (PWM Timer 1 For Group A)
     * |         |              |0 = Corresponding PWM-Timer Stopped.
     * |         |              |1 = Corresponding PWM-Timer Start Running.
     * |[10]     |CH1INV        |PWM-Timer 1 Output Inverter Enable (PWM Timer 1 For Group A)
     * |         |              |0 = Inverter Disable.
     * |         |              |1 = Inverter Enable.
     * |[11]     |CH1MOD        |PWM-Timer 1 Auto-Reload/One-Shot Mode (PWM Timer 1 For Group A)
     * |         |              |0 = One-shot mode.
     * |         |              |1 = Auto-reload mode.
     * |         |              |Note: If there is a transition at this bit, it will cause CNR1 and CMR1 be cleared.
     * |[16]     |CH2EN         |PWM-Timer 2 Enable (PWM Timer 2 For Group A)
     * |         |              |0 = Corresponding PWM-Timer Stopped.
     * |         |              |1 = Corresponding PWM-Timer Start Running.
     * |[18]     |CH2INV        |PWM-Timer 2 Output Inverter Enable (PWM Timer 2 For Group A)
     * |         |              |0 = Inverter Disabled.
     * |         |              |1 = Inverter Enabled.
     * |[19]     |CH2MOD        |PWM-Timer 2 Auto-Reload/One-Shot Mode (PWM Timer 2 For Group A)
     * |         |              |0 = One-shot mode.
     * |         |              |1 = Auto-reload mode.
     * |         |              |Note: If there is a transition at this bit, it will cause CNR2 and CMR2 be cleared.
     * |[24]     |CH3EN         |PWM-Timer 3 Enable (PWM Timer 3 For Group A)
     * |         |              |0 = Corresponding PWM-Timer Stopped.
     * |         |              |1 = Corresponding PWM-Timer Start Running.
     * |[26]     |CH3INV        |PWM-Timer 3 Output Inverter Enable (PWM Timer 3 For Group A)
     * |         |              |0 = Inverter Disabled.
     * |         |              |1 = Inverter Enabled.
     * |[27]     |CH3MOD        |PWM-Timer 3 Auto-Reload/One-Shot Mode (PWM Timer 3 For Group A)
     * |         |              |0 = One-shot mode.
     * |         |              |1 = Auto-reload mode.
     * |         |              |Note: If there is a transition at this bit, it will cause CNR3 and CMR3 be cleared.
     */
    __IO uint32_t PCR;

    /**
     * CNR0
     * ===================================================================================================
     * Offset: 0x0C  PWM Counter Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRx      |PWM Timer Loaded Value
     * |        |          |CNR determines the PWM period.
     * |        |          |PWM frequency = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1)); where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |Duty ratio = (CMR+1)/(CNR+1).
     * |        |          |CMR >= CNR: PWM output is always high.
     * |        |          |CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          |CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When CNR value is set to 0, PWM output is always high.
     */
    __IO uint32_t CNR0;

    /**
     * CMR0
     * ===================================================================================================
     * Offset: 0x10  PWM Comparator Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMRx      |PWM Comparator Register
     * |        |          |CMR determines the PWM duty.
     * |        |          |PWM frequency = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1)); where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |Duty ratio = (CMR+1)/(CNR+1).
     * |        |          |CMR >= CNR: PWM output is always high.
     * |        |          |CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          |CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CMR will take effect in next PWM cycle.
     */
    __IO uint32_t CMR0;

    /**
     * PDR0
     * ===================================================================================================
     * Offset: 0x14  PWM Data Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDRx      |PWM Data Register
     * |        |          |User can monitor PDR to know the current value in 16-bit counter.
     */
    __I  uint32_t PDR0;

    /**
     * CNR1
     * ===================================================================================================
     * Offset: 0x18  PWM Counter Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRx      |PWM Timer Loaded Value
     * |        |          |CNR determines the PWM period.
     * |        |          |PWM frequency = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1)); where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |Duty ratio = (CMR+1)/(CNR+1).
     * |        |          |CMR >= CNR: PWM output is always high.
     * |        |          |CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          |CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When CNR value is set to 0, PWM output is always high.
     */
    __IO uint32_t CNR1;

    /**
     * CMR1
     * ===================================================================================================
     * Offset: 0x1C  PWM Comparator Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMRx      |PWM Comparator Register
     * |        |          |CMR determines the PWM duty.
     * |        |          |PWM frequency = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1)); where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |Duty ratio = (CMR+1)/(CNR+1).
     * |        |          |CMR >= CNR: PWM output is always high.
     * |        |          |CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          |CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CMR will take effect in next PWM cycle.
     */
    __IO uint32_t CMR1;

    /**
     * PDR1
     * ===================================================================================================
     * Offset: 0x20  PWM Data Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDRx      |PWM Data Register
     * |        |          |User can monitor PDR to know the current value in 16-bit counter.
     */
    __I  uint32_t PDR1;

    /**
     * CNR2
     * ===================================================================================================
     * Offset: 0x24  PWM Counter Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRx      |PWM Timer Loaded Value
     * |        |          |CNR determines the PWM period.
     * |        |          |PWM frequency = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1)); where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |Duty ratio = (CMR+1)/(CNR+1).
     * |        |          |CMR >= CNR: PWM output is always high.
     * |        |          |CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          |CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When CNR value is set to 0, PWM output is always high.
     */
    __IO uint32_t CNR2;

    /**
     * CMR2
     * ===================================================================================================
     * Offset: 0x28  PWM Comparator Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMRx      |PWM Comparator Register
     * |        |          |CMR determines the PWM duty.
     * |        |          |PWM frequency = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1)); where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |Duty ratio = (CMR+1)/(CNR+1).
     * |        |          |CMR >= CNR: PWM output is always high.
     * |        |          |CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          |CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CMR will take effect in next PWM cycle.
     */
    __IO uint32_t CMR2;

    /**
     * PDR2
     * ===================================================================================================
     * Offset: 0x2C  PWM Data Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDRx      |PWM Data Register
     * |        |          |User can monitor PDR to know the current value in 16-bit counter.
     */
    __I  uint32_t PDR2;

    /**
     * CNR3
     * ===================================================================================================
     * Offset: 0x30  PWM Counter Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRx      |PWM Timer Loaded Value
     * |        |          |CNR determines the PWM period.
     * |        |          |PWM frequency = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1)); where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |Duty ratio = (CMR+1)/(CNR+1).
     * |        |          |CMR >= CNR: PWM output is always high.
     * |        |          |CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          |CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CNR will take effect in next PWM cycle.
     * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When CNR value is set to 0, PWM output is always high.
     */
    __IO uint32_t CNR3;

    /**
     * CMR3
     * ===================================================================================================
     * Offset: 0x34  PWM Comparator Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMRx      |PWM Comparator Register
     * |        |          |CMR determines the PWM duty.
     * |        |          |PWM frequency = PWMxy_CLK/((prescale+1)*(clock divider)*(CNR+1)); where xy, could be 01 or 23, depends on selected PWM channel.
     * |        |          |Duty ratio = (CMR+1)/(CNR+1).
     * |        |          |CMR >= CNR: PWM output is always high.
     * |        |          |CMR < CNR: PWM low width = (CNR-CMR) unit; PWM high width = (CMR+1) unit.
     * |        |          |CMR = 0: PWM low width = (CNR) unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to CMR will take effect in next PWM cycle.
     */
    __IO uint32_t CMR3;

    /**
     * PDR3
     * ===================================================================================================
     * Offset: 0x38  PWM Data Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PDRx      |PWM Data Register
     * |        |          |User can monitor PDR to know the current value in 16-bit counter.
     */
    __I  uint32_t PDR3;

    /**
     * @cond HIDDEN_SYMBOLS
     * RESERVED0
     * ===================================================================================================
     *
     * ---------------------------------------------------------------------------------------------------
     */
    __I  uint32_t RESERVED0[1];
    /**
     * @endcond
     */

    /**
     * PIER
     * ===================================================================================================
     * Offset: 0x40  PWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWMIE0    |PWM Channel 0 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[1]     |PWMIE1    |PWM Channel 1 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[2]     |PWMIE2    |PWM Channel 2 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[3]     |PWMIE3    |PWM Channel 3 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     */
    __IO uint32_t PIER;

    /**
     * PIIR
     * ===================================================================================================
     * Offset: 0x44  PWM Interrupt Indication Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWMIF0    |PWM Channel 0 Period Interrupt Status
     * |        |          |This bit is set by hardware when PWM0 down counter reaches zero and PWM0 interrupt enable bit (PWMIE0) is 1, software can write 1 to clear this bit to zero.
     * |[1]     |PWMIF1    |PWM Channel 1 Period Interrupt Status
     * |        |          |This bit is set by hardware when PWM1 down counter reaches zero and PWM1 interrupt enable bit (PWMIE1) is 1, software can write 1 to clear this bit to zero.
     * |[2]     |PWMIF2    |PWM Channel 2 Period Interrupt Status
     * |        |          |This bit is set by hardware when PWM2 down counter reaches zero and PWM2 interrupt enable bit (PWMIE2) is 1, software can write 1 to clear this bit to zero.
     * |[3]     |PWMIF3    |PWM Channel 3 Period Interrupt Status
     * |        |          |This bit is set by hardware when PWM3 down counter reaches zero and PWM3 interrupt enable bit (PWMIE3) is 1, software can write 1 to clear this bit to zero.
     */
    __IO uint32_t PIIR;

    /**
     * @cond HIDDEN_SYMBOLS
     * RESERVE1
     * ===================================================================================================
     *
     * ---------------------------------------------------------------------------------------------------
     */
    __I  uint32_t RESERVE1[2];
    /**
     * @endcond
     */

    /**
     * CCR0
     * ===================================================================================================
     * Offset: 0x50  PWM Capture Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INV0      |Channel 0 Inverter Enable
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
     * |[1]     |CRL_IE0   |Channel 0 Rising Latch Interrupt Enable
     * |        |          |0 = Rising latch interrupt Disabled.
     * |        |          |1 = Rising latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 0 has rising transition, Capture will issue an Interrupt.
     * |[2]     |CFL_IE0   |Channel 0 Falling Latch Interrupt Enable
     * |        |          |0 = Falling latch interrupt Disabled.
     * |        |          |1 = Falling latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 0 has falling transition, Capture will issue an Interrupt.
     * |[3]     |CAPCH0EN  |Channel 0 Capture Function Enable
     * |        |          |0 = Capture function on PWM group channel 0 Disabled.
     * |        |          |1 = Capture function on PWM group channel 0 Enabled.
     * |        |          |When Enabled, Capture latched the PWM-counter value and saved to CRLR (Rising latch) and CFLR (Falling latch).
     * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 0 Interrupt.
     * |[4]     |CAPIF0    |Channel 0 Capture Interrupt Indication Flag
     * |        |          |If PWM group channel 0 rising latch interrupt is enabled (CRL_IE0 = 1), a rising transition occurs at PWM group channel 0 will result in CAPIF0 to high; Similarly, a falling transition will cause CAPIF0 to be set high if PWM group channel 0 falling latch interrupt is enabled (CFL_IE0 = 1).
     * |        |          |Write 1 to clear this bit to 0.
     * |[6]     |CRLRI0    |CRLR0 Latched Indicator Bit
     * |        |          |When PWM group input channel 0 has a rising transition, CRLR0 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[7]     |CFLRI0    |CFLR0 Latched Indicator Bit
     * |        |          |When PWM group input channel 0 has a falling transition, CFLR0 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[16]    |INV1      |Channel 1 Inverter Enable
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
     * |[17]    |CRL_IE1   |Channel 1 Rising Latch Interrupt Enable
     * |        |          |0 = Rising latch interrupt Disabled.
     * |        |          |1 = Rising latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 1 has rising transition, Capture will issue an Interrupt.
     * |[18]    |CFL_IE1   |Channel 1 Falling Latch Interrupt Enable
     * |        |          |0 = Falling latch interrupt Disabled.
     * |        |          |1 = Falling latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 1 has falling transition, Capture will issue an Interrupt.
     * |[19]    |CAPCH1EN  |Channel 1 Capture Function Enable
     * |        |          |0 = Capture function on PWM group channel 1 Disabled.
     * |        |          |1 = Capture function on PWM group channel 1 Enabled.
     * |        |          |When Enabled, Capture latched the PWM-counter and saved to CRLR (Rising latch) and CFLR (Falling latch).
     * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 1 Interrupt.
     * |[20]    |CAPIF1    |Channel 1 Capture Interrupt Indication Flag
     * |        |          |If PWM group channel 1 rising latch interrupt is enabled (CRL_IE1 = 1), a rising transition occurs at PWM group channel 1 will result in CAPIF1 to high; Similarly, a falling transition will cause CAPIF1 to be set high if PWM group channel 1 falling latch interrupt is enabled (CFL_IE1 = 1).
     * |        |          |Write 1 to clear this bit to 0.
     * |[22]    |CRLRI1    |CRLR1 Latched Indicator Bit
     * |        |          |When PWM group input channel 1 has a rising transition, CRLR1 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[23]    |CFLRI1    |CFLR1 Latched Indicator Bit
     * |        |          |When PWM group input channel 1 has a falling transition, CFLR1 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     */
    __IO uint32_t CCR0;

    /**
     * CCR2
     * ===================================================================================================
     * Offset: 0x54  PWM Capture Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INV2      |Channel 2 Inverter Enable
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
     * |[1]     |CRL_IE2   |Channel 2 Rising Latch Interrupt Enable
     * |        |          |0 = Rising latch interrupt Disabled.
     * |        |          |1 = Rising latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 2 has rising transition, Capture will issue an Interrupt.
     * |[2]     |CFL_IE2   |Channel 2 Falling Latch Interrupt Enable
     * |        |          |0 = Falling latch interrupt Disabled.
     * |        |          |1 = Falling latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 2 has falling transition, Capture will issue an Interrupt.
     * |[3]     |CAPCH2EN  |Channel 2 Capture Function Enable
     * |        |          |0 = Capture function on PWM group channel 2 Disabled.
     * |        |          |1 = Capture function on PWM group channel 2 Enabled.
     * |        |          |When Enabled, Capture latched the PWM-counter value and saved to CRLR (Rising latch) and CFLR (Falling latch).
     * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 2 Interrupt.
     * |[4]     |CAPIF2    |Channel 2 Capture Interrupt Indication Flag
     * |        |          |If PWM group channel 2 rising latch interrupt is enabled (CRL_IE2=1), a rising transition occurs at PWM group channel 2 will result in CAPIF2 to high; Similarly, a falling transition will cause CAPIF2 to be set high if PWM group channel 2 falling latch interrupt is enabled (CFL_IE2=1).
     * |        |          |Write 1 to clear this bit to 0
     * |[6]     |CRLRI2    |CRLR2 Latched Indicator Bit
     * |        |          |When PWM group input channel 2 has a rising transition, CRLR2 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[7]     |CFLRI2    |CFLR2 Latched Indicator Bit
     * |        |          |When PWM group input channel 2 has a falling transition, CFLR2 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[16]    |INV3      |Channel 3 Inverter Enable
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer
     * |[17]    |CRL_IE3   |Channel 3 Rising Latch Interrupt Enable
     * |        |          |0 = Rising latch interrupt Disabled.
     * |        |          |1 = Rising latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 3 has rising transition, Capture will issue an Interrupt.
     * |[18]    |CFL_IE3   |Channel 3 Falling Latch Interrupt Enable
     * |        |          |0 = Falling latch interrupt Disabled.
     * |        |          |1 = Falling latch interrupt Enabled.
     * |        |          |When Enabled, if Capture detects PWM group channel 3 has falling transition, Capture will issue an Interrupt.
     * |[19]    |CAPCH3EN  |Channel 3 Capture Function Enable
     * |        |          |0 = Capture function on PWM group channel 3 Disabled.
     * |        |          |1 = Capture function on PWM group channel 3 Enabled.
     * |        |          |When Enabled, Capture latched the PWM-counter and saved to CRLR (Rising latch) and CFLR (Falling latch).
     * |        |          |When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 3 Interrupt.
     * |[20]    |CAPIF3    |Channel 3 Capture Interrupt Indication Flag
     * |        |          |If PWM group channel 3 rising latch interrupt is enabled (CRL_IE3=1), a rising transition occurs at PWM group channel 3 will result in CAPIF3 to high; Similarly, a falling transition will cause CAPIF3 to be set high if PWM group channel 3 falling latch interrupt is enabled (CFL_IE3=1).
     * |        |          |Write 1 to clear this bit to 0
     * |[22]    |CRLRI3    |CRLR3 Latched Indicator Bit
     * |        |          |When PWM group input channel 3 has a rising transition, CRLR3 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[23]    |CFLRI3    |CFLR3 Latched Indicator Bit
     * |        |          |When PWM group input channel 3 has a falling transition, CFLR3 was latched with the value of PWM down-counter and this bit is set by hardware.
     * |        |          |Software can write 1 to clear this bit to 0.
      */
    __IO uint32_t CCR2;

    /**
     * CRLR0
     * ===================================================================================================
     * Offset: 0x58  PWM Capture Rising Latch Register (Channel 0)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRLRx     |Capture Rising Latch Register
     * |        |          |Latch the PWM counter when Channel 0 has rising transition.
     */
    __IO uint32_t CRLR0;

    /**
     * CFLR0
     * ===================================================================================================
     * Offset: 0x5C  PWM Capture Falling Latch Register (Channel 0)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CFLRx     |Capture Falling Latch Register
     * |        |          |Latch the PWM counter when Channel 0 has falling transition.
     */
    __IO uint32_t CFLR0;

    /**
     * CRLR1
     * ===================================================================================================
     * Offset: 0x60  PWM Capture Rising Latch Register (Channel 1)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRLRx     |Capture Rising Latch Register
     * |        |          |Latch the PWM counter when Channel 1 has rising transition.
     */
    __IO uint32_t CRLR1;

    /**
     * CFLR1
     * ===================================================================================================
     * Offset: 0x64  PWM Capture Falling Latch Register (Channel 1)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CFLRx     |Capture Falling Latch Register
     * |        |          |Latch the PWM counter when Channel 1 has falling transition.
     */
    __IO uint32_t CFLR1;

    /**
     * CRLR2
     * ===================================================================================================
     * Offset: 0x68  PWM Capture Rising Latch Register (Channel 2)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRLRx     |Capture Rising Latch Register
     * |        |          |Latch the PWM counter when Channel 2 has rising transition.
     */
    __IO uint32_t CRLR2;

    /**
     * CFLR2
     * ===================================================================================================
     * Offset: 0x6C  PWM Capture Falling Latch Register (Channel 2)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CFLRx     |Capture Falling Latch Register
     * |        |          |Latch the PWM counter when Channel 2 has falling transition.
     */
    __IO uint32_t CFLR2;

    /**
     * CRLR3
     * ===================================================================================================
     * Offset: 0x70  PWM Capture Rising Latch Register (Channel 3)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CRLRx     |Capture Rising Latch Register
     * |        |          |Latch the PWM counter when Channel 3 has rising transition.
     */
    __IO uint32_t CRLR3;

    /**
     * CFLR3
     * ===================================================================================================
     * Offset: 0x74  PWM Capture Falling Latch Register (Channel 3)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CFLRx     |Capture Falling Latch Register
     * |        |          |Latch the PWM counter when Channel 3 has falling transition.
     */
    __IO uint32_t CFLR3;

    /**
     * CAPENR
     * ===================================================================================================
     * Offset: 0x78  PWM Capture Input 0~3 Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CAPENR    |Capture Input Enable Register
     * |        |          |There are four capture inputs from pad. Bit0~Bit3 are used to control each input enable or disable.
     * |        |          |0 = Disable (PWMx multi-function pin input does not affect input capture function.)
     * |        |          |1 = Enable (PWMx multi-function pin input will affect its input capture function.)
     */
    __IO uint32_t CAPENR;

    /**
     * POE
     * ===================================================================================================
     * Offset: 0x7C  PWM Output Enable for Channel 0~3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWM0      |Channel 0 Output Enable
     * |        |          |0 = Disable PWM channel 0 output to pin.
     * |        |          |1 = Enable PWM channel 0 output to pin.
     * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
     * |[1]     |PWM1      |Channel 1 Output Enable
     * |        |          |0 = Disable PWM channel 1 output to pin.
     * |        |          |1 = Enable PWM channel 1 output to pin.
     * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
     * |[2]     |PWM2      |Channel 2 Output Enable
     * |        |          |0 = Disable PWM channel 2 output to pin.
     * |        |          |1 = Enable PWM channel 2 output to pin.
     * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
     * |[3]     |PWM3      |Channel 3 Output Enable
     * |        |          |0 = Disable PWM channel 3 output to pin.
     * |        |          |1 = Enable PWM channel 3 output to pin.
     * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function
     */
    __IO uint32_t POE;

} PWM_T;


/** @addtogroup REG_PWM_BITMASK PWM Bit Mask
  @{
 */

/* PWM PPR Bit Field Definitions */
#define PWM_PPR_DZI23_Pos                       24                                  /*!< PWM_T::PPR: DZI23 Position */
#define PWM_PPR_DZI23_Msk                       (0xFFul << PWM_PPR_DZI23_Pos)       /*!< PWM_T::PPR: DZI23 Mask */

#define PWM_PPR_DZI01_Pos                       16                                  /*!< PWM_T::PPR: DZI01 Position */
#define PWM_PPR_DZI01_Msk                       (0xFFul << PWM_PPR_DZI01_Pos)       /*!< PWM_T::PPR: DZI01 Mask */

#define PWM_PPR_CP23_Pos                        8                                   /*!< PWM_T::PPR: CP23 Position */
#define PWM_PPR_CP23_Msk                        (0xFFul << PWM_PPR_CP23_Pos)        /*!< PWM_T::PPR: CP23 Mask */

#define PWM_PPR_CP01_Pos                        0                                   /*!< PWM_T::PPR: CP01 Position */
#define PWM_PPR_CP01_Msk                        (0xFFul << PWM_PPR_CP01_Pos)        /*!< PWM_T::PPR: CP01 Mask */

/* PWM CSR Bit Field Definitions */
#define PWM_CSR_CSR3_Pos                        12                                  /*!< PWM_T::CSR: CSR3 Position */
#define PWM_CSR_CSR3_Msk                        (7ul << PWM_CSR_CSR3_Pos)           /*!< PWM_T::CSR: CSR3 Mask */

#define PWM_CSR_CSR2_Pos                        8                                   /*!< PWM_T::CSR: CSR2 Position */
#define PWM_CSR_CSR2_Msk                        (7ul << PWM_CSR_CSR2_Pos)           /*!< PWM_T::CSR: CSR2 Mask */

#define PWM_CSR_CSR1_Pos                        4                                   /*!< PWM_T::CSR: CSR1 Position */
#define PWM_CSR_CSR1_Msk                        (7ul << PWM_CSR_CSR1_Pos)           /*!< PWM_T::CSR: CSR1 Mask */

#define PWM_CSR_CSR0_Pos                        0                                   /*!< PWM_T::CSR: CSR0 Position */
#define PWM_CSR_CSR0_Msk                        (7ul << PWM_CSR_CSR0_Pos)           /*!< PWM_T::CSR: CSR0 Mask */

/* PWM PCR Bit Field Definitions */
#define PWM_PCR_CH3MOD_Pos                      27                                  /*!< PWM_T::PCR: CH3MOD Position */
#define PWM_PCR_CH3MOD_Msk                      (1ul << PWM_PCR_CH3MOD_Pos)         /*!< PWM_T::PCR: CH3MOD Mask */

#define PWM_PCR_CH3INV_Pos                      26                                  /*!< PWM_T::PCR: CH3INV Position */
#define PWM_PCR_CH3INV_Msk                      (1ul << PWM_PCR_CH3INV_Pos)         /*!< PWM_T::PCR: CH3INV Mask */

#define PWM_PCR_CH3EN_Pos                       24                                  /*!< PWM_T::PCR: CH3EN Position */
#define PWM_PCR_CH3EN_Msk                       (1ul << PWM_PCR_CH3EN_Pos)          /*!< PWM_T::PCR: CH3EN Mask */

#define PWM_PCR_CH2MOD_Pos                      19                                  /*!< PWM_T::PCR: CH2MOD Position */
#define PWM_PCR_CH2MOD_Msk                      (1ul << PWM_PCR_CH2MOD_Pos)         /*!< PWM_T::PCR: CH2MOD Mask */

#define PWM_PCR_CH2INV_Pos                      18                                  /*!< PWM_T::PCR: CH2INV Position */
#define PWM_PCR_CH2INV_Msk                      (1ul << PWM_PCR_CH2INV_Pos)         /*!< PWM_T::PCR: CH2INV Mask */

#define PWM_PCR_CH2EN_Pos                       16                                  /*!< PWM_T::PCR: CH2EN Position */
#define PWM_PCR_CH2EN_Msk                       (1ul << PWM_PCR_CH2EN_Pos)          /*!< PWM_T::PCR: CH2EN Mask */

#define PWM_PCR_CH1MOD_Pos                      11                                  /*!< PWM_T::PCR: CH1MOD Position */
#define PWM_PCR_CH1MOD_Msk                      (1ul << PWM_PCR_CH1MOD_Pos)         /*!< PWM_T::PCR: CH1MOD Mask */

#define PWM_PCR_CH1INV_Pos                      10                                  /*!< PWM_T::PCR: CH1INV Position */
#define PWM_PCR_CH1INV_Msk                      (1ul << PWM_PCR_CH1INV_Pos)         /*!< PWM_T::PCR: CH1INV Mask */

#define PWM_PCR_CH1EN_Pos                       8                                   /*!< PWM_T::PCR: CH1EN Position */
#define PWM_PCR_CH1EN_Msk                       (1ul << PWM_PCR_CH1EN_Pos)          /*!< PWM_T::PCR: CH1EN Mask */

#define PWM_PCR_DZEN23_Pos                      5                                   /*!< PWM_T::PCR: DZEN23 Position */
#define PWM_PCR_DZEN23_Msk                      (1ul << PWM_PCR_DZEN23_Pos)         /*!< PWM_T::PCR: DZEN23 Mask */

#define PWM_PCR_DZEN01_Pos                      4                                   /*!< PWM_T::PCR: DZEN01 Position */
#define PWM_PCR_DZEN01_Msk                      (1ul << PWM_PCR_DZEN01_Pos)         /*!< PWM_T::PCR: DZEN01 Mask */

#define PWM_PCR_CH0MOD_Pos                      3                                   /*!< PWM_T::PCR: CH0MOD Position */
#define PWM_PCR_CH0MOD_Msk                      (1ul << PWM_PCR_CH0MOD_Pos)         /*!< PWM_T::PCR: CH0MOD Mask */

#define PWM_PCR_CH0INV_Pos                      2                                   /*!< PWM_T::PCR: CH0INV Position */
#define PWM_PCR_CH0INV_Msk                      (1ul << PWM_PCR_CH0INV_Pos)         /*!< PWM_T::PCR: CH0INV Mask */

#define PWM_PCR_CH0EN_Pos                       0                                   /*!< PWM_T::PCR: CH0EN Position */
#define PWM_PCR_CH0EN_Msk                       (1ul << PWM_PCR_CH0EN_Pos)          /*!< PWM_T::PCR: CH0EN Mask */

/* PWM CNR Bit Field Definitions */
#define PWM_CNR_CNR_Pos                         0                                   /*!< PWM_T::CNR0: CNR Position */
#define PWM_CNR_CNR_Msk                         (0xFFFFul << PWM_CNR_CNR_Pos)       /*!< PWM_T::CNR0: CNR Mask */

/* PWM CMR Bit Field Definitions */
#define PWM_CMR_CMR_Pos                         0                                   /*!< PWM_T::CMR0: CMR Position */
#define PWM_CMR_CMR_Msk                         (0xFFFFul << PWM_CMR_CMR_Pos)       /*!< PWM_T::CMR0: CMR Mask */

/* PWM PDR Bit Field Definitions */
#define PWM_PDR_PDR_Pos                         0                                   /*!< PWM_T::PDR0: PDR Position */
#define PWM_PDR_PDR_Msk                         (0xFFFFul << PWM_PDR_PDR_Pos)       /*!< PWM_T::PDR0: PDR Mask */

/* PWM PIER Bit Field Definitions */
#define PWM_PIER_PWMIE3_Pos                     3                                   /*!< PWM_T::PIER: PWMIE3 Position */
#define PWM_PIER_PWMIE3_Msk                     (1ul << PWM_PIER_PWMIE3_Pos)        /*!< PWM_T::PIER: PWMIE3 Mask */

#define PWM_PIER_PWMIE2_Pos                     2                                   /*!< PWM_T::PIER: PWMIE2 Position */
#define PWM_PIER_PWMIE2_Msk                     (1ul << PWM_PIER_PWMIE2_Pos)        /*!< PWM_T::PIER: PWMIE2 Mask */

#define PWM_PIER_PWMIE1_Pos                     1                                   /*!< PWM_T::PIER: PWMIE1 Position */
#define PWM_PIER_PWMIE1_Msk                     (1ul << PWM_PIER_PWMIE1_Pos)        /*!< PWM_T::PIER: PWMIE1 Mask */

#define PWM_PIER_PWMIE0_Pos                     0                                   /*!< PWM_T::PIER: PWMIE0 Position */
#define PWM_PIER_PWMIE0_Msk                     (1ul << PWM_PIER_PWMIE0_Pos)        /*!< PWM_T::PIER: PWMIE0 Mask */

/* PWM PIIR Bit Field Definitions */
#define PWM_PIIR_PWMIF3_Pos                     3                                   /*!< PWM_T::PIIR: PWMIF3 Position */
#define PWM_PIIR_PWMIF3_Msk                     (1ul << PWM_PIIR_PWMIF3_Pos)        /*!< PWM_T::PIIR: PWMIF3 Mask */

#define PWM_PIIR_PWMIF2_Pos                     2                                   /*!< PWM_T::PIIR: PWMIF2 Position */
#define PWM_PIIR_PWMIF2_Msk                     (1ul << PWM_PIIR_PWMIF2_Pos)        /*!< PWM_T::PIIR: PWMIF2 Mask */

#define PWM_PIIR_PWMIF1_Pos                     1                                   /*!< PWM_T::PIIR: PWMIF1 Position */
#define PWM_PIIR_PWMIF1_Msk                     (1ul << PWM_PIIR_PWMIF1_Pos)        /*!< PWM_T::PIIR: PWMIF1 Mask */

#define PWM_PIIR_PWMIF0_Pos                     0                                   /*!< PWM_T::PIIR: PWMIF0 Position */
#define PWM_PIIR_PWMIF0_Msk                     (1ul << PWM_PIIR_PWMIF0_Pos)        /*!< PWM_T::PIIR: PWMIF0 Mask */

/* PWM CCR0 Bit Field Definitions */
#define PWM_CCR0_CFLRI1_Pos                     23                                  /*!< PWM_T::CCR0: CFLRI1 Position */
#define PWM_CCR0_CFLRI1_Msk                     (1ul << PWM_CCR0_CFLRI1_Pos)        /*!< PWM_T::CCR0: CFLRI1 Mask */

#define PWM_CCR0_CRLRI1_Pos                     22                                  /*!< PWM_T::CCR0: CRLRI1 Position */
#define PWM_CCR0_CRLRI1_Msk                     (1ul << PWM_CCR0_CRLRI1_Pos)        /*!< PWM_T::CCR0: CRLRI1 Mask */

#define PWM_CCR0_CAPIF1_Pos                     20                                  /*!< PWM_T::CCR0: CAPIF1 Position */
#define PWM_CCR0_CAPIF1_Msk                     (1ul << PWM_CCR0_CAPIF1_Pos)        /*!< PWM_T::CCR0: CAPIF1 Mask */

#define PWM_CCR0_CAPCH1EN_Pos                   19                                  /*!< PWM_T::CCR0: CAPCH1EN Position */
#define PWM_CCR0_CAPCH1EN_Msk                   (1ul << PWM_CCR0_CAPCH1EN_Pos)      /*!< PWM_T::CCR0: CAPCH1EN Mask */

#define PWM_CCR0_CFL_IE1_Pos                    18                                  /*!< PWM_T::CCR0: CFL_IE1 Position */
#define PWM_CCR0_CFL_IE1_Msk                    (1ul << PWM_CCR0_CFL_IE1_Pos)       /*!< PWM_T::CCR0: CFL_IE1 Mask */

#define PWM_CCR0_CRL_IE1_Pos                    17                                  /*!< PWM_T::CCR0: CRL_IE1 Position */
#define PWM_CCR0_CRL_IE1_Msk                    (1ul << PWM_CCR0_CRL_IE1_Pos)       /*!< PWM_T::CCR0: CRL_IE1 Mask */

#define PWM_CCR0_INV1_Pos                       16                                  /*!< PWM_T::CCR0: INV1 Position */
#define PWM_CCR0_INV1_Msk                       (1ul << PWM_CCR0_INV1_Pos)          /*!< PWM_T::CCR0: INV1 Mask */

#define PWM_CCR0_CFLRI0_Pos                     7                                   /*!< PWM_T::CCR0: CFLRI0 Position */
#define PWM_CCR0_CFLRI0_Msk                     (1ul << PWM_CCR0_CFLRI0_Pos)        /*!< PWM_T::CCR0: CFLRI0 Mask */

#define PWM_CCR0_CRLRI0_Pos                     6                                   /*!< PWM_T::CCR0: CRLRI0 Position */
#define PWM_CCR0_CRLRI0_Msk                     (1ul << PWM_CCR0_CRLRI0_Pos)        /*!< PWM_T::CCR0: CRLRI0 Mask */

#define PWM_CCR0_CAPIF0_Pos                     4                                   /*!< PWM_T::CCR0: CAPIF0 Position */
#define PWM_CCR0_CAPIF0_Msk                     (1ul << PWM_CCR0_CAPIF0_Pos)        /*!< PWM_T::CCR0: CAPIF0 Mask */

#define PWM_CCR0_CAPCH0EN_Pos                   3                                   /*!< PWM_T::CCR0: CAPCH0EN Position */
#define PWM_CCR0_CAPCH0EN_Msk                   (1ul << PWM_CCR0_CAPCH0EN_Pos)      /*!< PWM_T::CCR0: CAPCH0EN Mask */

#define PWM_CCR0_CFL_IE0_Pos                    2                                   /*!< PWM_T::CCR0: CFL_IE0 Position */
#define PWM_CCR0_CFL_IE0_Msk                    (1ul << PWM_CCR0_CFL_IE0_Pos)       /*!< PWM_T::CCR0: CFL_IE0 Mask */

#define PWM_CCR0_CRL_IE0_Pos                    1                                   /*!< PWM_T::CCR0: CRL_IE0 Position */
#define PWM_CCR0_CRL_IE0_Msk                    (1ul << PWM_CCR0_CRL_IE0_Pos)       /*!< PWM_T::CCR0: CRL_IE0 Mask */

#define PWM_CCR0_INV0_Pos                       0                                   /*!< PWM_T::CCR0: INV0 Position */
#define PWM_CCR0_INV0_Msk                       (1ul << PWM_CCR0_INV0_Pos)          /*!< PWM_T::CCR0: INV0 Mask */

/* PWM CCR2 Bit Field Definitions */
#define PWM_CCR2_CFLRI3_Pos                     23                                  /*!< PWM_T::CCR2: CFLRI3 Position */
#define PWM_CCR2_CFLRI3_Msk                     (1ul << PWM_CCR2_CFLRI3_Pos)        /*!< PWM_T::CCR2: CFLRI3 Mask */

#define PWM_CCR2_CRLRI3_Pos                     22                                  /*!< PWM_T::CCR2: CRLRI3 Position */
#define PWM_CCR2_CRLRI3_Msk                     (1ul << PWM_CCR2_CRLRI3_Pos)        /*!< PWM_T::CCR2: CRLRI3 Mask */

#define PWM_CCR2_CAPIF3_Pos                     20                                  /*!< PWM_T::CCR2: CAPIF3 Position */
#define PWM_CCR2_CAPIF3_Msk                     (1ul << PWM_CCR2_CAPIF3_Pos)        /*!< PWM_T::CCR2: CAPIF3 Mask */

#define PWM_CCR2_CAPCH3EN_Pos                   19                                  /*!< PWM_T::CCR2: CAPCH3EN Position */
#define PWM_CCR2_CAPCH3EN_Msk                   (1ul << PWM_CCR2_CAPCH3EN_Pos)      /*!< PWM_T::CCR2: CAPCH3EN Mask */

#define PWM_CCR2_CFL_IE3_Pos                    18                                  /*!< PWM_T::CCR2: CFL_IE3 Position */
#define PWM_CCR2_CFL_IE3_Msk                    (1ul << PWM_CCR2_CFL_IE3_Pos)       /*!< PWM_T::CCR2: CFL_IE3 Mask */

#define PWM_CCR2_CRL_IE3_Pos                    17                                  /*!< PWM_T::CCR2: CRL_IE3 Position */
#define PWM_CCR2_CRL_IE3_Msk                    (1ul << PWM_CCR2_CRL_IE3_Pos)       /*!< PWM_T::CCR2: CRL_IE3 Mask */

#define PWM_CCR2_INV3_Pos                       16                                  /*!< PWM_T::CCR2: INV3 Position */
#define PWM_CCR2_INV3_Msk                       (1ul << PWM_CCR2_INV3_Pos)          /*!< PWM_T::CCR2: INV3 Mask */

#define PWM_CCR2_CFLRI2_Pos                     7                                   /*!< PWM_T::CCR2: CFLRI2 Position */
#define PWM_CCR2_CFLRI2_Msk                     (1ul << PWM_CCR2_CFLRI2_Pos)        /*!< PWM_T::CCR2: CFLRI2 Mask */

#define PWM_CCR2_CRLRI2_Pos                     6                                   /*!< PWM_T::CCR2: CRLRI2 Position */
#define PWM_CCR2_CRLRI2_Msk                     (1ul << PWM_CCR2_CRLRI2_Pos)        /*!< PWM_T::CCR2: CRLRI2 Mask */

#define PWM_CCR2_CAPIF2_Pos                     4                                   /*!< PWM_T::CCR2: CAPIF2 Position */
#define PWM_CCR2_CAPIF2_Msk                     (1ul << PWM_CCR2_CAPIF2_Pos)        /*!< PWM_T::CCR2: CAPIF2 Mask */

#define PWM_CCR2_CAPCH2EN_Pos                   3                                   /*!< PWM_T::CCR2: CAPCH2EN Position */
#define PWM_CCR2_CAPCH2EN_Msk                   (1ul << PWM_CCR2_CAPCH2EN_Pos)      /*!< PWM_T::CCR2: CAPCH2EN Mask */

#define PWM_CCR2_CFL_IE2_Pos                    2                                   /*!< PWM_T::CCR2: CFL_IE2 Position */
#define PWM_CCR2_CFL_IE2_Msk                    (1ul << PWM_CCR2_CFL_IE2_Pos)       /*!< PWM_T::CCR2: CFL_IE2 Mask */

#define PWM_CCR2_CRL_IE2_Pos                    1                                   /*!< PWM_T::CCR2: CRL_IE2 Position */
#define PWM_CCR2_CRL_IE2_Msk                    (1ul << PWM_CCR2_CRL_IE2_Pos)       /*!< PWM_T::CCR2: CRL_IE2 Mask */

#define PWM_CCR2_INV2_Pos                       0                                   /*!< PWM_T::CCR2: INV2 Position */
#define PWM_CCR2_INV2_Msk                       (1ul << PWM_CCR2_INV2_Pos)          /*!< PWM_T::CCR2: INV2 Mask */

/* PWM CRLR Bit Field Definitions */
#define PWM_CRLR_CRLR_Pos                       0                                   /*!< PWM_T::CRLR0: CRLR Position */
#define PWM_CRLR_CRLR_Msk                       (0xFFFFul << PWM_CRLR_CRLR_Pos)     /*!< PWM_T::CRLR0: CRLR Mask */

/* PWM CFLR Bit Field Definitions */
#define PWM_CFLR_CFLR_Pos                       0                                   /*!< PWM_T::CFLR0: CFLR Position */
#define PWM_CFLR_CFLR_Msk                       (0xFFFFul << PWM_CFLR_CFLR_Pos)     /*!< PWM_T::CFLR0: CFLR Mask */

/* PWM CAPENR Bit Field Definitions */
#define PWM_CAPENR_CAPENR_Pos                   0                                   /*!< PWM_T::CAPENR: CAPENR Position */
#define PWM_CAPENR_CAPENR_Msk                   (0xFul << PWM_CAPENR_CAPENR_Pos)    /*!< PWM_T::CAPENR: CAPENR Mask */

/* PWM POE Bit Field Definitions */
#define PWM_POE_PWM3_Pos                        3                                   /*!< PWM_T::POE: PWM3 Position */
#define PWM_POE_PWM3_Msk                        (1ul << PWM_POE_PWM3_Pos)           /*!< PWM_T::POE: PWM3 Mask */

#define PWM_POE_PWM2_Pos                        2                                   /*!< PWM_T::POE: PWM2 Position */
#define PWM_POE_PWM2_Msk                        (1ul << PWM_POE_PWM2_Pos)           /*!< PWM_T::POE: PWM2 Mask */

#define PWM_POE_PWM1_Pos                        1                                   /*!< PWM_T::POE: PWM1 Position */
#define PWM_POE_PWM1_Msk                        (1ul << PWM_POE_PWM1_Pos)           /*!< PWM_T::POE: PWM1 Mask */

#define PWM_POE_PWM0_Pos                        0                                   /*!< PWM_T::POE: PWM0 Position */
#define PWM_POE_PWM0_Msk                        (1ul << PWM_POE_PWM0_Pos)           /*!< PWM_T::POE: PWM0 Mask */

/*@}*/ /* end of group REG_PWM_BITMASK */
/*@}*/ /* end of group REG_PWM */


/*---------------------- Real Time Clock Controller -------------------------*/
/**
    @addtogroup REG_RTC Real Time Clock Controller (RTC)
    Memory Mapped Structure for RTC Controller
@{ */
typedef struct
{
    /**
     * INIR
     * ===================================================================================================
     * Offset: 0x00  RTC Initiation Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |Active    |RTC Active Status
     * |        |          |0 = RTC is at reset state.
     * |        |          |1 = RTC is at normal active state.
     * |[31:0]  |INIR      |RTC Initiation
     * |        |          |When RTC block is powered on, RTC is at reset state.
     * |        |          |User has to write a number (0xa5eb1357) to INIR to make RTC leaving reset state.
     * |        |          |Once the INIR is written as 0xa5eb1357, the RTC will be in un-reset state permanently.
     * |        |          |The INIR[31:1] is a write-only field and read value will be always 0.
     */
    __IO uint32_t INIR;

    /**
     * AER
     * ===================================================================================================
     * Offset: 0x04  RTC Access Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |AER       |RTC Register Access Enable Password (Write Only)
     * |        |          |Writing 0xA965 to this register will enable RTC access and keep 512 RTC clocks.
     * |[16]    |ENF       |RTC Register Access Enable Flag (Read Only)
     * |        |          |0 = RTC register read/write access Disabled.
     * |        |          |1 = RTC register read/write access Enabled.
     * |        |          |This bit will be set after AER[15:0] is load a 0xA965, and will be cleared automatically after 512 RTC clocks.
     */
    __IO uint32_t AER;

    /**
     * FCR
     * ===================================================================================================
     * Offset: 0x08  RTC Frequency Compensation Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |FRACTION  |Fraction Part
     * |        |          |Formula = (fraction part of detected value) x 60.
     * |        |          |Note: Digit in FCR must be expressed as hexadecimal number.
     * |[11:8]  |INTEGER   |Integer Part
     */
    __IO uint32_t FCR;

    /**
     * TLR
     * ===================================================================================================
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
     */
    __IO uint32_t TLR;

    /**
     * CLR
     * ===================================================================================================
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
     */
    __IO uint32_t CLR;

    /**
     * TSSR
     * ===================================================================================================
     * Offset: 0x14  Time Scale Selection Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |24H_12H   |24-Hour / 12-Hour Time Scale Selection
     * |        |          |It indicates that RTC TLR and TAR counter are in 24-hour time scale or 12-hour time scale.
     * |        |          |0 = 24-hour time scale selected.
     * |        |          |1 = 24-hour time scale selected.
     */
    __IO uint32_t TSSR;

    /**
     * DWR
     * ===================================================================================================
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
     */
    __IO uint32_t DWR;

    /**
     * TAR
     * ===================================================================================================
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
     */
    __IO uint32_t TAR;

    /**
     * CAR
     * ===================================================================================================
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
     */
    __IO uint32_t CAR;

    /**
     * LIR
     * ===================================================================================================
     * Offset: 0x24  Leap Year Indicator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LIR       |Leap Year Indication Register (Read Only)
     * |        |          |0 = This year is not a leap year.
     * |        |          |1 = This year is a leap year.
     */
    __I  uint32_t LIR;

    /**
     * RIER
     * ===================================================================================================
     * Offset: 0x28  RTC Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |AIER      |Alarm Interrupt Enable
     * |        |          |This bit is used to enable/disable RTC Alarm Interrupt, and generate an interrupt signal if AIF (RIIR[0] RTC Alarm Interrupt Flag) is set to 1.
     * |        |          |0 = RTC Alarm Interrupt Disabled.
     * |        |          |1 = RTC Alarm Interrupt Enabled.
     * |[1]     |TIER      |Time Tick Interrupt Enable
     * |        |          |This bit is used to enable/disable RTC Time Tick Interrupt, and generate an interrupt signal if TIF (RIIR[1] RTC Time Tick Interrupt Flag) is set to 1.
     * |        |          |0 = RTC Time Tick Interrupt Disabled.
     * |        |          |1 = RTC Time Tick Interrupt Enabled.
     */
    __IO uint32_t RIER;

    /**
     * RIIR
     * ===================================================================================================
     * Offset: 0x2C  RTC Interrupt Indicator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |AIF       |RTC Alarm Interrupt Flag
     * |        |          |When RTC time counters TLR and CLR match the alarm setting time registers TAR and CAR, this bit will be set to 1 and an interrupt will be generated if RTC Alarm Interrupt enabled AIER (RIER[0]) is set to 1.
     * |        |          |Chip will be wake-up if RTC Alarm Interrupt is enabled when chip is at Power-down mode.
     * |        |          |0 = Alarm condition is not matched.
     * |        |          |1 = Alarm condition is matched.
     * |        |          |Note: Write 1 to clear this bit.
     * |[1]     |TIF       |RTC Time Tick Interrupt Flag
     * |        |          |When RTC time tick happened, this bit will be set to 1 and an interrupt will be generated if RTC Tick Interrupt enabled TIER (RIER[1]) is set to 1.
     * |        |          |Chip will also be wake-up if RTC Tick Interrupt is enabled and this bit is set to 1 when chip is running at Power-down mode.
     * |        |          |0 = Tick condition does not occur.
     * |        |          |1 = Tick condition occur.
     * |        |          |Note: Write 1 to clear to clear this bit.
     */
    __IO uint32_t RIIR;

    /**
     * TTR
     * ===================================================================================================
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
     * |        |          |Note: This register can be read back after the RTC register access enable bit ENF (AER[16]) is active.
     * |[3]     |TWKE      |RTC Timer Wake-Up Function Enable Bit 
     * |        |          |If TWKE is set before chip is in power down mode, chip will be woken-up by RTC controller when a RTC Time Tick occurs. The chip can also be woken-up by alarm match occur.
     * |        |          |0 = RTC Timer wake-up function Disabled.
     * |        |          |1 = Enable RTC Timer wake-up function that chip can be woken-up from power down mode by Time Tick or Alarm Match.
     * |        |          |Note: Tick timer setting follows TTR[2:0] description.
     */
    __IO uint32_t TTR;
} RTC_T;

/**
    @addtogroup REG_RTC_BITMASK RTC Bit Mask
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
#define RTC_RIER_TIER_Pos       1                                               /*!< RTC_T::RIER: TIER Position */
#define RTC_RIER_TIER_Msk       (1ul << RTC_RIER_TIER_Pos)                      /*!< RTC_T::RIER: TIER Mask */

#define RTC_RIER_AIER_Pos       0                                               /*!< RTC_T::RIER: AIER Position */
#define RTC_RIER_AIER_Msk       (1ul << RTC_RIER_AIER_Pos)                      /*!< RTC_T::RIER: AIER Mask */

/* RTC RIIR Bit Field Definitions */
#define RTC_RIIR_TIF_Pos        1                                               /*!< RTC_T::RIIR: TIF Position */
#define RTC_RIIR_TIF_Msk        (1ul << RTC_RIIR_TIF_Pos)                       /*!< RTC_T::RIIR: TIF Mask */

#define RTC_RIIR_AIF_Pos        0                                               /*!< RTC_T::RIIR: AIF Position */
#define RTC_RIIR_AIF_Msk        (1ul << RTC_RIIR_AIF_Pos)                       /*!< RTC_T::RIIR: AIF Mask */

/* RTC TTR Bit Field Definitions */
#define RTC_TTR_TWKE_Pos        3                                               /*!< RTC_T::TTR: TWKE Position */
#define RTC_TTR_TWKE_Msk        (1ul << RTC_TTR_TWKE_Pos)                       /*!< RTC_T::TTR: TWKE Mask */

#define RTC_TTR_TTR_Pos         0                                               /*!< RTC_T::TTR: TTR Position */
#define RTC_TTR_TTR_Msk         (0x7ul << RTC_TTR_TTR_Pos)                      /*!< RTC_T::TTR: TTR Mask */
/*@}*/ /* end of group REG_RTC_BITMASK */
/*@}*/ /* end of group REG_RTC */


/*------------------------- SPI Interface Controller -------------------------*/
/** @addtogroup REG_SPI Serial Peripheral Interface Controller (SPI)
  Memory Mapped Structure for SPI Controller
  @{
 */
typedef struct
{
    /**
     * SPI_CNTRL
     * ===================================================================================================
     * Offset: 0x00  Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GO_BUSY   |SPI Transfer Control Bit and Busy Status
     * |        |          |During data transfer, this bit keeps the value of 1. As the transfer is finished, this bit will be cleared automatically.
     * |        |          |Software can read this bit to check if the SPI is in busy status.
     * |        |          |0 = Writing 0 to this bit to stop data transfer if SPI is transferring.
     * |        |          |1 = In Master mode, writing 1 to this bit to start the SPI data transfer; in Slave mode,
     * |        |          |    writing 1 to this bit indicates that the Slave is ready to communicate with a Master.
     * |        |          |Note:
     * |        |          |All configurations should be set before writing 1 to this GO_BUSY bit.
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
     * |[9:8]   |TX_NUM    |Numbers of Transmit/Receive Word
     * |        |          |This field specifies how many transmit/receive word numbers should be executed in one transfer.
     * |        |          |00 = Only one transmit/receive word will be executed in one transfer.
     * |        |          |01 = Two successive transmit/receive words will be executed in one transfer. (burst mode)
     * |        |          |10 = Reserved.
     * |        |          |11 = Reserved.
     * |        |          |Note: In slave mode with level-trigger configuration, if TX_NUM is set to 01, the slave select pin must be kept at active
     * |        |          |state during the successive data transfer.
     * |[10]    |LSB       |Send LSB First
     * |        |          |0 = The MSB, which bit of transmit/receive register depends on the setting of TX_BIT_LEN, is transmitted/received first.
     * |        |          |1 = The LSB, bit 0 of the SPI TX0/1 register, is sent first to the SPI data output pin, and the first bit received from
     * |        |          |    the SPI data input pin will be put in the LSB position of the RX register (bit 0 of SPI_RX0/1).
     * |[11]    |CLKP      |Clock Polarity
     * |        |          |0 = SPI bus clock is idle low.
     * |        |          |1 = SPI bus clock is idle high.
     * |[15:12] |SP_CYCLE  |Suspend Interval (Master Only)
     * |        |          |The four bits provide configurable suspend interval between two successive transmit/receive transaction in a transfer.
     * |        |          |The definition of the suspend interval is the interval between the last clock edge of the preceding transaction word
     * |        |          |and the first clock edge of the following transaction word.
     * |        |          |The default value is 0x0.
     * |        |          |The period of the suspend interval is obtained according to the following equation.
     * |        |          |(SP_CYCLE[3:0] + 2) * period of SPI bus clock cycle
     * |        |          |Example:
     * |        |          |SP_CYCLE = 0x0 ... 2 SPI bus clock cycles.
     * |        |          |SP_CYCLE = 0x1 ... 3 SPI bus clock cycles.
     * |        |          |......
     * |        |          |SP_CYCLE = 0xE ... 16 SPI bus clock cycles.
     * |        |          |SP_CYCLE = 0xF ... 17 SPI bus clock cycles.
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
     * |[20:19] |REORDER   |Byte Reorder Function Selection
     * |        |          |A byte suspend interval will be inserted among each byte when byte suspend function is enabled.
     * |        |          |The period of the byte suspend interval depends on the setting of SP_CYCLE.
     * |        |          |00 = Disable both byte reorder and byte suspend functions.
     * |        |          |01 = Enable byte reorder function and insert a byte suspend interval (2~17 SPI bus clock cycles) among each byte.
     * |        |          |The setting of TX_BIT_LEN must be configured as 0x00. (32 bits/word)
     * |        |          |10 = Enable byte reorder function, but disable byte suspend function.
     * |        |          |11 = Disable byte reorder function, but insert a suspend interval (2~17 SPICLK cycles) among each byte.
     * |        |          |The setting of TX_BIT_LEN must be configured as 0x00. (32 bits/word)
     * |        |          |Note:
     * |        |          |1. Byte Reorder function is only available if TX_BIT_LEN is defined as 16, 24, and 32 bits.
     * |        |          |2. In Slave mode with level-trigger configuration, the slave select pin must be kept at active state during the
     * |        |          |   byte suspend interval.
     * |        |          |3. The Byte Reorder function is not supported when the variable bus clock function is enabled.
     * |[23]    |VARCLK_EN |Variable Clock Enable (Master Only)
     * |        |          |0 = SPI clock output frequency is fixed and decided only by the value of DIVIDER.
     * |        |          |1 = SPI clock output frequency is variable.
     * |        |          |The output frequency is decided by the value of VARCLK, DIVIDER, and DIVIDER2.
     * |        |          |Note: When this VARCLK_EN bit is set to 1, the setting of TX_BIT_LEN must be programmed as 0x10 (16 bits/word).
     */
    __IO uint32_t CNTRL;

    /**
     * SPI_DIVIDER
     * ===================================================================================================
     * Offset: 0x04  Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |DIVIDER   |Clock Divider 1 (Master Only)
     * |        |          |The value in this field is the frequency divider for generating the SPI peripheral clock and the SPI bus clock of SPI master.
     * |        |          |The frequency is obtained according to the following equation.
     * |        |          |   SPI peripheral clock frequency = system clock frequency / (DIVIDER + 1) / 2
     * |        |          |In Slave mode, the period of SPI clock driven by a master shall equal or over 5 times the period of PCLK.
     * |        |          |In other words, the maximum frequency of SPI clock is the fifth of the frequency of slave's PCLK.
     * |[31:16] |DIVIDER2  |Clock Divider 2 (Master Only)
     * |        |          |The value in this field is the 2nd frequency divider for generating the second clock of the variable clock function.
     * |        |          |The frequency is obtained according to the following equation:
     * |        |          |   f_clk2 = SPI peripheral clock frequency / (DIVIDER2 + 1) / 2
     * |        |          |If the VARCLK_EN bit is cleared to 0, this setting is unmeaning.
     */
    __IO uint32_t DIVIDER;

    /**
     * SPI_SSR
     * ===================================================================================================
     * Offset: 0x08  Slave Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |SSR       |Slave Select Control Bits (Master Only)
     * |        |          |If AUTOSS bit is cleared, writing 1 to any bit of this field sets the proper SPISSn0/1
     * |        |          |line to an active state and writing 0 sets the line back to inactive state.
     * |        |          |If the AUTOSS bit is set, writing 0 to any bit location of this field will keep the corresponding
     * |        |          |SPISSn0/1 line at inactive state; writing 1 to any bit location of this field will select
     * |        |          |appropriate SPISSn0/1 line to be automatically driven to active state for the duration of the
     * |        |          |transmit/receive, and will be driven to inactive state for the rest of the time.
     * |        |          |The active state of SPISSn0/1 is specified in SS_LVL.
     * |        |          |Note: SPISSn0 is defined as the slave select input in Slave mode.
     * |[2]     |SS_LVL    |Slave Select Active Level
     * |        |          |This bit defines the active status of slave select signal (SPISSn0/1).
     * |        |          |0 = The slave select signal SPISSn0/1 is active on low-level/falling-edge.
     * |        |          |1 = The slave select signal SPISSn0/1 is active on high-level/rising-edge.
     * |[3]     |AUTOSS    |Automatic Slave Select Function Enable (Master Only)
     * |        |          |0 = If this bit is cleared, slave select signals will be asserted/de-asserted by setting /clearing
     * |        |          |    the corresponding bits of SPI_SSR[1:0].
     * |        |          |1 = If this bit is set, SPISSn0/1 signals will be generated automatically.
     * |        |          |    It means that device/slave select signal, which is set in SPI_SSR[1:0], will be asserted by the
     * |        |          |    SPI controller when transmit/receive is started, and will be de-asserted after each transmit/receive is finished.
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
     */
    __IO uint32_t SSR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE0;
    /**
     * @endcond
     */

    /**
     * SPI_RX0
     * ===================================================================================================
     * Offset: 0x10  Data Receive Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register
     * |        |          |The Data Receive Registers hold the value of received data of the last executed transfer.
     * |        |          |Valid bits depend on the transmit bit length field in the SPI_CNTRL register.
     * |        |          |For example, if TX_BIT_LEN is set to 0x08 and TX_NUM is set to 0x0, bit RX0[7:0]
     * |        |          |holds the received data. The values of the other bits are unknown. The Data Receive Registers are read-only registers.
     */
    /**
     * SPI_RX1
     * ===================================================================================================
     * Offset: 0x14  Data Receive Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register
     * |        |          |The Data Receive Registers hold the value of received data of the last executed transfer.
     * |        |          |SPI_RX1 register is only available when TX_NUM is set to 1.
     * |        |          |Valid bits depend on the transmit bit length field in the SPI_CNTRL register.
     */
    __I  uint32_t RX[2];

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE1;
    __I  uint32_t RESERVE2;
    /**
     * @endcond
     */

    /**
     * SPI_TX0
     * ===================================================================================================
     * Offset: 0x20  Data Transmit Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The data transmit registers hold the data to be transmitted in the next transfer.
     * |        |          |The number of valid bits depends on the setting of transmit bit length field of the SPI_CNTRL register.
     * |        |          |For example, if TX_BIT_LEN is set to 0x08 and the TX_NUM is set to 0x0, the bits TX[7:0] will be transmitted in next transfer.
     * |        |          |If TX_BIT_LEN is set to 0x00 and TX_NUM is set to 0x1, the SPI controller will perform two 32-bit
     * |        |          |transmit/receive successive using the same setting. The transmission sequence is TX0[31:0] first and then TX1[31:0].
     */
    /**
     * SPI_TX1
     * ===================================================================================================
     * Offset: 0x24  Data Transmit Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The data transmit registers hold the data to be transmitted in the next transfer.
     * |        |          |The number of valid bits depends on the setting of transmit bit length field of the SPI_CNTRL register.
     * |        |          |For example, if TX_BIT_LEN is set to 0x08 and the TX_NUM is set to 0x0, the bits TX[7:0] will be transmitted in next transfer.
     * |        |          |If TX_BIT_LEN is set to 0x00 and TX_NUM is set to 0x1, the SPI controller will perform two 32-bit
     * |        |          |transmit/receive successive using the same setting. The transmission sequence is TX0[31:0] first and then TX1[31:0].
     */
    __O  uint32_t TX[2];

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE3;
    __I  uint32_t RESERVE4;
    __I  uint32_t RESERVE5;
    /**
     * @endcond
     */

    /**
     * SPI_VARCLK
     * ===================================================================================================
     * Offset: 0x34  Variable Clock Pattern Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |VARCLK    |Variable Clock Pattern
     * |        |          |This register defines the clock pattern of the SPI transfer.
     * |        |          |If the bit pattern of VARCLK is 0, the output frequency of SPICLK is according the value of DIVIDER.
     * |        |          |If the bit pattern of VARCLK is 1, the output frequency of SPICLK is according the value of DIVIDER2.
     * |        |          |If the variable clock function is disabled, this setting is unmeaning.
     */
    __IO uint32_t VARCLK;

} SPI_T;

/** @addtogroup REG_SPI_BITMASK SPI Bit Mask
  @{
 */

/* SPI_CNTRL Bit Field Definitions */
#define SPI_CNTRL_VARCLK_EN_Pos    23                                     /*!< SPI_T::CNTRL: VARCLK_EN Position */
#define SPI_CNTRL_VARCLK_EN_Msk    (1ul << SPI_CNTRL_VARCLK_EN_Pos)       /*!< SPI_T::CNTRL: VARCLK_EN Mask     */

#define SPI_CNTRL_REORDER_Pos      19                                     /*!< SPI_T::CNTRL: REORDER Position */
#define SPI_CNTRL_REORDER_Msk      (3ul << SPI_CNTRL_REORDER_Pos)         /*!< SPI_T::CNTRL: REORDER Mask     */

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

#define SPI_CNTRL_TX_NUM_Pos       8                                      /*!< SPI_T::CNTRL: TX_NUM Position */
#define SPI_CNTRL_TX_NUM_Msk       (3ul << SPI_CNTRL_TX_NUM_Pos)          /*!< SPI_T::CNTRL: TX_NUM Mask     */

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
#define SPI_DIVIDER_DIVIDER2_Msk   (0xFFFFul << SPI_DIVIDER_DIVIDER2_Pos) /*!< SPI_T::DIVIDER: DIVIDER2 Mask */

#define SPI_DIVIDER_DIVIDER_Pos    0                                      /*!< SPI_T::DIVIDER: DIVIDER Position */
#define SPI_DIVIDER_DIVIDER_Msk    (0xFFFFul << SPI_DIVIDER_DIVIDER_Pos)  /*!< SPI_T::DIVIDER: DIVIDER Mask */

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
#define SPI_SSR_SSR_Msk            (3ul << SPI_SSR_SSR_Pos)          /*!< SPI_T::SSR: SSR Mask */

/*@}*/ /* end of group REG_SPI_BITMASK */
/*@}*/ /* end of group REG_SPI */


/*---------------------------- Global Controller -----------------------------*/
/** @addtogroup REG_SYS System Controller (SYS)
  Memory Mapped Structure for System Controller
  @{
 */

typedef struct
{

    /**
     * PDID
     * ===================================================================================================
     * Offset: 0x00  Part Device Identification Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDID      |Part Device Identification Number
     * |        |          |This register reflects device part number code.
     * |        |          |Software can read this register to identify which device is used.
     */
    __I uint32_t PDID;

    /**
     * RSTSRC
     * ===================================================================================================
     * Offset: 0x04  System Reset Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RSTS_POR  |Power-on Reset Flag
     * |        |          |The RSTS_POR flag is set by the "reset signal" from the Power-On Reset (POR) controller or bit CHIP_RST (IPRSTC1[0]) to indicate the previous reset source.
     * |        |          |0 = No reset from POR or CHIP_RST (IPRSTC1[0]).
     * |        |          |1 = Power-on Reset (POR) or CHIP_RST (IPRSTC1[0]) had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared by writing "1" to it.
     * |[1]     |RSTS_RESET|Reset Pin Reset Flag
     * |        |          |The RSTS_RESET flag is set by the "Reset Signal" from the /RESET pin to indicate the previous reset source.
     * |        |          |0 = No reset from /RESET pin.
     * |        |          |1 = The Pin /RESET had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared by writing "1" to it.
     * |[2]     |RSTS_WDT  |Watchdog Reset Flag
     * |        |          |The RSTS_WDT flag is set by The "Reset Signal" from the Watchdog Timer to indicate the previous reset source
     * |        |          |0 = No reset from watchdog timer.
     * |        |          |1 = The watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared by writing "1" to it.
     * |[3]     |RSTS_LVR  |Low Voltage Reset Flag
     * |        |          |The RSTS_LVR flag Is Set By The "Reset Signal" From The Low-Voltage-Reset Controller To Indicate The Previous Reset Source
     * |        |          |0 = No reset from LVR.
     * |        |          |1 = The LVR controller had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[4]     |RSTS_BOD  |Brown-out Detector Reset Flag
     * |        |          |The RSTS_BOD flag is set by the "Reset Signal" from the Brown-Out Detector to indicate the previous reset source.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD had issued the reset signal to reset the system.
     * |        |          |Note: This bit can be cleared by writing "1" to it.
     * |[5]     |RSTS_SYS  |SYS Reset Flag
     * |        |          |The RSTS_SYS flag is set by the "Reset Signal" from the Cortex-M0 kernel to indicate the previous reset source.
     * |        |          |0 = No reset from Cortex-M0.
     * |        |          |1 = The Cortex-M0 had issued the reset signal to reset the system by writing 1 to bit SYSRESETREQ (AIRCR[2], Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M0 kernel.
     * |        |          |Note: This bit can be cleared by writing "1" to it.
     * |[7]     |RSTS_CPU  |CPU Reset Flag
     * |        |          |The RSTS_CPU flag is set by hardware if software writes CPU_RST (IPRSTC1[1]) 1 to reset Cortex-M0 CPU kernel and flash. Memory Controller (FMC)
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = Cortex-M0 CPU kernel and FMC are reset by software setting CPU_RST (IPRSTC1[1]) to 1.
     * |        |          |Note: This bit can be cleared by writing "1" to it.
     */
    __IO uint32_t RSTSRC;

    /**
     * IPRSTC1
     * ===================================================================================================
     * Offset: 0x08  Peripheral Reset Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIP_RST  |CHIP One-shot Reset (Write Protect)
     * |        |          |Setting this bit will reset the whole chip, including CPU kernel and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |The CHIP_RST is the same as the POR reset, all the chip controllers are reset and the chip setting from flash are also reload.
     * |        |          |0 = CHIP normal operation.
     * |        |          |1 = CHIP one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the REGWRPROT register.
     * |[1]     |CPU_RST   |CPU Kernel One-shot Reset (Write Protect)
     * |        |          |Setting this bit will only reset the CPU kernel and Flash Memory Controller(FMC), and this bit will automatically return 0 after the two clock cycles.
     * |        |          |0 = CPU normal operation.
     * |        |          |1 = CPU one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the REGWRPROT register.
     */
    __IO uint32_t IPRSTC1;

    /**
     * IPRSTC2
     * ===================================================================================================
     * Offset: 0x0C  IP Reset Control Register 2
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
     * |[9]     |I2C1_RST  |I2C1 Controller Reset
     * |        |          |0 = I2C1 controller normal operation.
     * |        |          |1 = I2C1 controller reset.
     * |[12]    |SPI0_RST  |SPI0 Controller Reset
     * |        |          |0 = SPI0 controller normal operation.
     * |        |          |1 = SPI0 controller reset.
     * |[13]    |SPI1_RST  |SPI1 Controller Reset
     * |        |          |0 = SPI1 controller normal operation.
     * |        |          |1 = SPI1 controller reset.
     * |[16]    |UART0_RST |UART0 Controller Reset
     * |        |          |0 = UART0 controller normal operation.
     * |        |          |1 = UART0 controller reset.
     * |[17]    |UART1_RST |UART1 Controller Reset
     * |        |          |0 = UART1 controller normal operation.
     * |        |          |1 = UART1 controller reset.
     * |[20]    |PWM03_RST |PWM03 Controller Reset
     * |        |          |0 = PWM03 controller normal operation.
     * |        |          |1 = PWM03 controller reset.
     * |[23]    |PS2_RST   |PS/2 Controller Reset
     * |        |          |0 = PS/2 controller normal operation.
     * |        |          |1 = PS/2 controller reset.
     * |[27]    |USBD_RST  |USB Device Controller Reset
     * |        |          |0 = USB device controller normal operation.
     * |        |          |1 = USB device controller reset.
     */
    __IO uint32_t IPRSTC2;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE0[2];
    /**
     * @endcond
     */

    /**
     * BODCR
     * ===================================================================================================
     * Offset: 0x18  Brown-out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BOD_EN    |Brown-Out Detector Enable (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CONFIG0 bit[23]
     * |        |          |0 = Brown-out Detector function Disabled.
     * |        |          |1 = Brown-out Detector function Enabled.
     * |        |          |Note: This bit is write protected. Refer to the REGWRPROT register.
     * |[2:1]   |BOD_VL    |Brown-Out Detector Threshold Voltage Selection (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register config0 bit[22:21].
     * |        |          |00 = Brown-out voltage is 2.2V.
     * |        |          |01 = Brown-out voltage is 2.7V.
     * |        |          |10 = Brown-out voltage is 3.7V.
     * |        |          |11 = Brown-out voltage is 4.4V.
     * |        |          |Note: This bit is write protected. Refer to the REGWRPROT register.
     * |[3]     |BOD_RSTEN |Brown-Out Reset Enable (Write Protect)
     * |        |          |0 = Brown-out "INTERRUPT" function Enabled.
     * |        |          |1 = Brown-out "RESET" function Enabled.
     * |        |          |While the Brown-out Detector function is enabled (BOD_EN high) and BOD reset function is enabled (BOD_RSTEN high), BOD will assert a signal to reset chip when the detected voltage is lower than the threshold (BOD_OUT high).
     * |        |          |Note1: While the BOD function is enabled (BOD_EN high) and BOD interrupt function is enabled (BOD_RSTEN low), BOD will assert an interrupt if BOD_OUT is high.
     * |        |          |BOD interrupt will keep till to the BOD_EN set to 0.
     * |        |          |BOD interrupt can be blocked by disabling the NVIC BOD interrupt or disabling BOD function (set BOD_EN low).
     * |        |          |Note2: The default value is set by flash controller user configuration register config0 bit[20].
     * |        |          |Note3: This bit is write protected. Refer to the REGWRPROT register.
     * |[4]     |BOD_INTF  |Brown-Out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage draft at VDD down through or up through the voltage of BOD_VL setting.
     * |        |          |1 = When Brown-out Detector detects the VDD is dropped down through the voltage of BOD_VL setting or the VDD is raised up through the voltage of BOD_VL setting, this bit is set to 1 and the Brown-out interrupt is requested if Brown-out interrupt is enabled.
     * |        |          |Note: This bit can be cleared to 0 by software writing "1".
     * |[5]     |BOD_LPM   |Brown-Out Detector Low Power Mode (Write Protection)
     * |        |          |0 = BOD operated in Normal mode (default).
     * |        |          |1 = BOD Low Power mode Enabled.
     * |        |          |Note1: The BOD consumes about 100 uA in Normal mode, and the low power mode can reduce the current to about 1/10 but slow the BOD response.
     * |        |          |Note2: This bit is the protected bit, and programming it needs to write "59h", "16h", and "88h" to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * |[6]     |BOD_OUT   |Brown-Out Detector Output Status
     * |        |          |0 = Brown-out Detector output status is 0. It means the detected voltage is higher than BOD_VL setting or BOD_EN is 0.
     * |        |          |1 = Brown-out Detector output status is 1. It means the detected voltage is lower than BOD_VL setting. If the BOD_EN is 0, BOD function disabled , this bit always responds to 0.
     * |[7]     |LVR_EN    |Low Voltage Reset Enable (Write Protection)
     * |        |          |The LVR function reset the chip when the input power voltage is lower than LVR circuit setting.
     * |        |          |LVR function is enabled by default.
     * |        |          |0 = Low Voltage Reset function Disabled.
     * |        |          |1 = Low Voltage Reset function Enabled - After enabling the bit, the LVR function will be active with 100us delay for LVR output stable (default).
     * |        |          |Note: This bit is write protected. Refer to the REGWRPROT register.
     */
    __IO uint32_t BODCR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE1[2];
    /**
     * @endcond
     */

    /**
     * PORCR
     * ===================================================================================================
     * Offset: 0x24  Power-on-Reset Controller Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |POR_DIS_CODE|Power-On-Reset Enable Control (Write Protect)
     * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again.
     * |        |          |User can disable internal POR circuit to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
     * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including: /RESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note: This bit is write protected. Refer to the REGWRPROT register.
     */
    __IO uint32_t PORCR;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE2[2];
    /**
     * @endcond
     */

    /**
     * GPA_MFP
     * ===================================================================================================
     * Offset: 0x30  GPIOA Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[10]    |GPA_MFP10 |PA.10 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP10 and ALT_MFP[11].
     * |        |          |(ALT_MFP[11], GPA_MFP10) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = I2C1_SDA function is selected.
     * |[11]    |GPA_MFP11 |PA.11 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP11 and ALT_MFP[11].
     * |        |          |(ALT_MFP[11], GPA_MFP11) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = I2C1_SCL function is selected.
     * |[12]    |GPA_MFP12 |PA.12 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP12 and ALT_MFP[11].
     * |        |          |(ALT_MFP[11], GPA_MFP12) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = PWM0 function is selected.
     * |[13]    |GPA_MFP13 |PA.13 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP13 and ALT_MFP[11].
     * |        |          |(ALT_MFP[11], GPA_MFP13) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = PWM1 function is selected.
     * |[14]    |GPA_MFP14 |PA.14 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP14 and ALT_MFP[11].
     * |        |          |(ALT_MFP[11], GPA_MFP14) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = PWM2 function is selected.
     * |[15]    |GPA_MFP15 |PA.15 Pin Function Selection
     * |        |          |The pin function depends on GPA_MFP15 and ALT_MFP[9].
     * |        |          |(ALT_MFP[9], GPA_MFP15) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = PWM3 function is selected.
     * |[31:16] |GPA_TYPEn |Trigger Function Selection
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger function Disabled.   
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger function Enabled.
     * |        |          |GPA[9:0] are reserved in this chip.
     */
    __IO uint32_t GPA_MFP;

    /**
     * GPB_MFP
     * ===================================================================================================
     * Offset: 0x34  GPIOB Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GPB_MFP0  |PB.0 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP0.     
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = UART0_RXD function is selected.
     * |[1]     |GPB_MFP1  |PB.1 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP1.     
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = UART0_TXD0 function is selected.
     * |[2]     |GPB_MFP2  |PB.2 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP2 and ALT_MFP[11].
     * |        |          |(ALT_MFP[11], GPB_MFP2) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART0_nRTS function is selected.
     * |[3]     |GPB_MFP3  |PB.3 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP3 and ALT_MFP[11].
     * |        |          |(ALT_MFP[11], GPB_MFP3) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART0_nCTS function is selected.
     * |[4]     |GPB_MFP4  |PB.4 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP4 and ALT_MFP[15].
     * |        |          |(ALT_MFP[15], GPB_MFP4) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_RXD function is selected.
     * |        |          |(1, 1) = SPI1_SS1 function is selected.
     * |[5]     |GPB_MFP5  |PB.5 Pin Function Selection
     * |        |          |0 = The GPIOB[5] is selected to the pin PB.5        
     * |        |          |1 = The UART1 TXD function is selected to the pin PB.5     
     * |[6]     |GPB_MFP6  |PB.6 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP6 and ALT_MFP[17].
     * |        |          |(ALT_MFP[17], GPB_MFP6) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_nRTS function is selected.
     * |[7]     |GPB_MFP7  |PB.7 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP7 and ALT_MFP[16].
     * |        |          |(ALT_MFP[16], GPB_MFP7) value and function mapping is as following list.
     * |        |          |(0, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_nCTS function is selected.
     * |[8]     |GPB_MFP8  |PB.8 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP8.     
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = TM0 function is selected.
     * |[9]     |GPB_MFP9  |PB.9 Pin Function Selection
     * |        |          |Bits PB9_S11(ALT_MFP[1]) and GPB_MFP[9] determine the PB.9 function.
     * |        |          |(PB9_S11 (ALT_MFP[1]), GPB_MFP9) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = TM1 function is selected.
     * |        |          |(1, 1) = SPI1_SS1 function is selected.
     * |[10]    |GPB_MFP10 |PB.10 Pin Function Selection
     * |        |          |Bits PB10_S01(ALT_MFP[0]) and GPB_MFP[10] determine the PB.10 function.
     * |        |          |(PB10_S01 (ALT_MFP[0]), GPB_MFP10) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = TM2 function is selected.
     * |        |          |(1, 1) = SPI0_SS1 function is selected.
     * |[14]    |GPB_MFP14 |PB.14 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP14 and ALT_MFP[3].
     * |        |          |(ALT_MFP[3], GPB_MFP14) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = INT0 function is selected.  
     * |[15]    |GPB_MFP15 |PB.15 Pin Function Selection
     * |        |          |The pin function depends on GPB_MFP15.
     * |        |          |0 = GPIO function is selected.
     * |        |          |1 = INT1 function is selected.       
     * |[31:16] |GPB_TYPEn |Trigger Function Selection
     * |        |          |0 = GPIOB[15:0] I/O input Schmitt Trigger function Disabled. 
     * |        |          |1 = GPIOB[15:0] I/O input Schmitt Trigger function Enabled.
     */
    __IO uint32_t GPB_MFP;

    /**
     * GPC_MFP
     * ===================================================================================================
     * Offset: 0x38  GPIOC Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GPC_MFP0  |PC.0 Pin Function Selection
     * |        |          |The pin function depends on GPC_MFP[0] and ALT_MFP[5].
     * |        |          |(ALT_MFP[5], GPC_MFP[0]) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_SS0 function is selected.
     * |[1]     |GPC_MFP1  |PC.1 Pin Function Selection
     * |        |          |The pin function depends on GPC_MFP[1] and ALT_MFP[6].     
     * |        |          |(ALT_MFP[6]), GPC_MFP[1]) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_CLK function is selected.
     * |[2]     |GPC_MFP2  |PC.2 Pin Function Selection
     * |        |          |The pin function depends on GPC_MFP[2] and ALT_MFP[7].     
     * |        |          |ALT_MFP[7] and GPC_MFP[2] determine the PC.2 function.
     * |        |          |(ALT_MFP[7], GPC_MFP[2]) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MISO0 function is selected.
     * |[3]     |GPC_MFP3  |PC.3 Pin Function Selection
     * |        |          |The pin function depends on GPC_MFP[3] and ALT_MFP[8].     
     * |        |          |(ALT_MFP[8], GPC_MFP[3]) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI0_MOSI0 function is selected.
     * |[4]     |GPC_MFP4  |PC.4 Pin Function Selection
     * |        |          |GPC_MFP[4] is needed to set 0 for GPIOC[4] function on PC.4.      
     * |[5]     |GPC_MFP5  |PC.5 Pin Function Selection
     * |        |          |GPC_MFP[5] is needed to set 0 for GPIOC[5] function on PC.5.
     * |[8]     |GPC_MFP8  |PC.8 Pin Function Selection     
     * |        |          |The pin function depends on GPC_MFP[8] and ALT_MFP[16].
     * |        |          |(ALT_MFP[16], GPC_MFP[8]) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI1_SS0 function is selected.     
     * |[9]     |GPC_MFP9  |PC.9 Pin Function Selection
     * |        |          |The pin function depends on GPC_MFP[9] and ALT_MFP[17].
     * |        |          |(ALT_MFP[17], GPC_MFP[9]) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI1_CLK function is selected.          
     * |[10]    |GPC_MFP10 |PC.10 Pin Function Selection
     * |        |          |The pin function depends on GPC_MFP[10] and ALT_MFP[18].
     * |        |          |(ALT_MFP[18], GPC_MFP[10]) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = SPI1_MISO0 function is selected.        
     * |[11]    |GPC_MFP11 |PC.11 Pin Function Selection
     * |        |          |The pin function depends on GPC_MFP[11] and ALT_MFP[19].
     * |        |          |(ALT_MFP[19], GPC_MFP[11]) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = PI1_MOSI0 function is selected.        
     * |[12]    |GPC_MFP12 |PC.12 Pin Function Selection
     * |        |          |Both GPC_MFP[12] and ALT_MFP[13] are needed to set 0 for GPIOC[12] function on PC.12.
     * |[13]    |GPC_MFP13 |PC.13 Pin Function Selection
     * |        |          |Both GPC_MFP[13] and ALT_MFP[21] are needed to set 0 for GPIOC[13] function on PC.13.
     * |[31:16] |GPC_TYPEn |Trigger Function Selection
     * |        |          |0 = GPIOC[15:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = GPIOC[15:0] I/O input Schmitt Trigger function Enabled.     
     */
    __IO uint32_t GPC_MFP;

    /**
     * GPD_MFP
     * ===================================================================================================
     * Offset: 0x3C  GPIOD Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GPD_MFP0  |PD.0 Pin Function Selection
     * |        |          |GPD_MFP[0] is needed to set 0 for GPIOD[0] function on PD.0.
     * |[1]     |GPD_MFP1  |PD.1 Pin Function Selection
     * |        |          |0 = The GPIOD[1] is selected to the pin PD.1.
     * |        |          |1 = The SPI0_SS1 function is selected to the pin PD.1.
     * |[2]     |GPD_MFP2  |PD.2 Pin Function Selection
     * |        |          |GPD_MFP[2] is needed to set 0 for GPIOD[2] function on PD.2.
     * |[3]     |GPD_MFP3  |PD.3 Pin Function Selection
     * |        |          |GPD_MFP[3] is needed to set 0 for GPIOD[3] function on PD.3.
     * |[4]     |GPD_MFP4  |PD.4 Pin Function Selection
     * |        |          |GPD_MFP[4] is needed to set 0 for GPIOD[4] function on PD.4.
     * |[5]     |GPD_MFP5  |PD.5 Pin Function Selection
     * |        |          |GPD_MFP[5] is needed to set 0 for GPIOD[5] function on PD.5.
     * |[8]     |GPD_MFP8  |PD.8 Pin Function Selection
     * |        |          |Both GPD_MFP[8] and ALT_MFP[18] are needed to set 0 for GPIOD[8] function on PD.8.
     * |[9]     |GPD_MFP9  |PD.9 Pin Function Selection
     * |        |          |Both GPD_MFP[9] and ALT_MFP[19] are needed to set 0 for GPIOD[9] function on PD.9.
     * |[10]    |GPD_MFP10 |PD.10 Pin Function Selection
     * |        |          |Both GPD_MFP[10] and ALT_MFP[20] are needed to set 0 for GPIOD[10] function on PD.10.
     * |[11]    |GPD_MFP11 |PD.11 Pin Function Selection
     * |        |          |Both GPD_MFP[11] and ALT_MFP[21] are needed to set 0 for GPIOD[11] function on PD.11.
     * |[31:16] |GPD_TYPEn |Trigger Function Selection
     * |        |          |0 = GPIOD[15:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = GPIOD[15:0] I/O input Schmitt Trigger function Enabled.
     * |        |          |GPIOD[15:12], GPIOD[7:6] are reserved.
     */
    __IO uint32_t GPD_MFP;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE3[4];
    /**
     * @endcond
     */

    /**
     * ALT_MFP
     * ===================================================================================================
     * Offset: 0x50  Alternative Multiple Function Pin Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PB10_S01  |PB.10 Pin Alternate Function Selection
     * |        |          |Bits PB10_S01 and GPB_MFP[10] determine the PB.10 function.
     * |        |          |(PB10_S01 (ALT_MFP[0]), GPB_MFP[10]) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = TM2 function is selected.
     * |        |          |(1, 1) = SPI0_SS1 function is selected.
     * |[1]     |PB9_S11   |PB.9 Pin Alternate Function Selection
     * |        |          |Bits PB9_S11(ALT_MFP[1]) and GPB_MFP[9] determine the PB.9 function.
     * |        |          |(PB9_S11 (ALT_MFP[1]), GPB_MFP9) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = TM1 function is selected.
     * |        |          |(1, 1) = SPI1_SS1 function is selected.
     * |[14:2]  |ALT_MFP[14:2]|Pin Alternate Function Selection
     * |        |          |They are necessary to set 0.     
     * |[15]    |ALT_MFP[15]|PB.4 Pin Alternate Function Selection
     * |        |          |The pin function depends on GPB_MFP4 and ALT_MFP[15].
     * |        |          |(ALT_MFP[15], GPB_MFP4) value and function mapping is as following list.
     * |        |          |(x, 0) = GPIO function is selected.
     * |        |          |(0, 1) = UART1_RXD function is selected.
     * |        |          |(1, 1) = SPI1_SS1 function is selected.
     * |[21:16] |ALT_MFP[14:2]|Pin Alternate Function Selection
     * |        |          |They are necessary to set 0.         
     */
    __IO uint32_t ALT_MFP;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE4[43];
    /**
     * @endcond
     */

    /**
     * REGWRPROT
     * ===================================================================================================
     * Offset: 0x100  Register Write Protect register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |REGPROTDIS|Register Write-Protection Disable index (Read Only)
     * |        |          |0 = Write-protection Enabled for writing protected registers.
     * |        |          |1 = Write-protection Disabled for writing protected registers.
     * |        |          |Any write to the protected register is ignored.
     * |        |          |The Protected registers are:
     * |        |          |IPRSTC1: address 0x5000_0008
     * |        |          |BODCR: address 0x5000_0018
     * |        |          |PORCR: address 0x5000_0024
     * |        |          |PWRCON: address 0x5000_0200 (bit[6] is not protected for power wake-up interrupt clear)
     * |        |          |APBCLK bit[0]: address 0x5000_0208 (bit[0] is Watchdog Timer clock enable)
     * |        |          |CLKSEL0: address 0x5000_0210 (for HCLK and CPU STCLK clock source select)
     * |        |          |CLKSEL1 bit[1:0]: address 0x5000_0214 (for Watchdog Timer clock source select)
     * |        |          |NMI_SEL bit[7]: address 0x5000_0380 (for interrupt test mode)
     * |        |          |ISPCON: address 0x5000_C000 (Flash ISP Control register)
     * |        |          |WTCR: address 0x4000_4000
     * |        |          |FATCON: address 0x5000_C018    
     * |[7:0]   |REGWRPROT |Register Write-Protection Code (Write Only)
     * |        |          |Some registers have write-protection function.
     * |        |          |Writing these registers has to disable the protected function by writing the sequence value "59h", "16h", "88h" to this field.
     * |        |          |After this sequence is completed, the REGPROTDIS bit will be set to 1 and write-protection registers can be normal write.
     */
    __IO uint32_t REGWRPROT;

} GCR_T;


/** @addtogroup REG_SYS_BITMASK SYS Bit Mask
  @{
 */

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
#define SYS_IPRSTC1_CPU_RST_Pos                 1                                   /*!< GCR_T::IPRSTC1: CPU_RST Position */
#define SYS_IPRSTC1_CPU_RST_Msk                 (1ul << SYS_IPRSTC1_CPU_RST_Pos)    /*!< GCR_T::IPRSTC1: CPU_RST Mask */

#define SYS_IPRSTC1_CHIP_RST_Pos                0                                   /*!< GCR_T::IPRSTC1: CHIP_RST Position */
#define SYS_IPRSTC1_CHIP_RST_Msk                (1ul << SYS_IPRSTC1_CHIP_RST_Pos)   /*!< GCR_T::IPRSTC1: CHIP_RST Mask */

/* GCR IPRSTC2 Bit Field Definitions */
#define SYS_IPRSTC2_USBD_RST_Pos                27                                  /*!< GCR_T::IPRSTC2: USBD_RST Position */
#define SYS_IPRSTC2_USBD_RST_Msk                (1ul << SYS_IPRSTC2_USBD_RST_Pos)   /*!< GCR_T::IPRSTC2: USBD_RST Mask */

#define SYS_IPRSTC2_PS2_RST_Pos                 23                                  /*!< GCR_T::IPRSTC2: PS2_RST Position */
#define SYS_IPRSTC2_PS2_RST_Msk                 (1ul << SYS_IPRSTC2_PS2_RST_Pos)    /*!< GCR_T::IPRSTC2: PS2_RST Mask */

#define SYS_IPRSTC2_PWM03_RST_Pos               20                                  /*!< GCR_T::IPRSTC2: PWM03_RST Position */
#define SYS_IPRSTC2_PWM03_RST_Msk               (1ul << SYS_IPRSTC2_PWM03_RST_Pos)  /*!< GCR_T::IPRSTC2: PWM03_RST Mask */

#define SYS_IPRSTC2_UART1_RST_Pos               17                                  /*!< GCR_T::IPRSTC2: UART1_RST Position */
#define SYS_IPRSTC2_UART1_RST_Msk               (1ul << SYS_IPRSTC2_UART1_RST_Pos)  /*!< GCR_T::IPRSTC2: UART1_RST Mask */

#define SYS_IPRSTC2_UART0_RST_Pos               16                                  /*!< GCR_T::IPRSTC2: UART0_RST Position */
#define SYS_IPRSTC2_UART0_RST_Msk               (1ul << SYS_IPRSTC2_UART0_RST_Pos)  /*!< GCR_T::IPRSTC2: UART0_RST Mask */

#define SYS_IPRSTC2_SPI1_RST_Pos                13                                  /*!< GCR_T::IPRSTC2: SPI1_RST Position */
#define SYS_IPRSTC2_SPI1_RST_Msk                (1ul << SYS_IPRSTC2_SPI1_RST_Pos)   /*!< GCR_T::IPRSTC2: SPI1_RST Mask */

#define SYS_IPRSTC2_SPI0_RST_Pos                12                                  /*!< GCR_T::IPRSTC2: SPI0_RST Position */
#define SYS_IPRSTC2_SPI0_RST_Msk                (1ul << SYS_IPRSTC2_SPI0_RST_Pos)   /*!< GCR_T::IPRSTC2: SPI0_RST Mask */

#define SYS_IPRSTC2_I2C1_RST_Pos                9                                   /*!< GCR_T::IPRSTC2: I2C1_RST Position */
#define SYS_IPRSTC2_I2C1_RST_Msk                (1ul << SYS_IPRSTC2_I2C1_RST_Pos)   /*!< GCR_T::IPRSTC2: I2C1_RST Mask */

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

/* GCR PORCR Bit Field Definitions */
#define SYS_PORCR_POR_DIS_CODE_Pos              0                                           /*!< GCR_T::PORCR: POR_DIS_CODE Position */
#define SYS_PORCR_POR_DIS_CODE_Msk              (0xFFFFul << SYS_PORCR_POR_DIS_CODE_Pos)    /*!< GCR_T::PORCR: POR_DIS_CODE Mask */

/* GCR GPAMFP Bit Field Definitions */
#define SYS_GPA_MFP_GPA_TYPE_Pos                 16                                         /*!< GCR_T::GPA_MFP: GPA_TYPE Position */
#define SYS_GPA_MFP_GPA_TYPE_Msk                 (0xFFFFul << SYS_GPA_MFP_GPA_TYPE_Pos)     /*!< GCR_T::GPA_MFP: GPA_TYPE Mask */

#define SYS_GPA_MFP_GPA_MFP_Pos                  0                                          /*!< GCR_T::GPA_MFP: GPA_MFP Position */
#define SYS_GPA_MFP_GPA_MFP_Msk                  (0xFFFFul << SYS_GPA_MFP_GPA_MFP_Pos)      /*!< GCR_T::GPA_MFP: GPA_MFP Mask */


/* GCR GPBMFP Bit Field Definitions */
#define SYS_GPB_MFP_GPB_TYPE_Pos                 16                                         /*!< GCR_T::GPB_MFP: GPB_TYPE Position */
#define SYS_GPB_MFP_GPB_TYPE_Msk                 (0xFFFFul << SYS_GPB_MFP_GPB_TYPE_Pos)     /*!< GCR_T::GPB_MFP: GPB_TYPE Mask */

#define SYS_GPB_MFP_GPB_MFP_Pos                  0                                          /*!< GCR_T::GPB_MFP: GPB_MFP Position */
#define SYS_GPB_MFP_GPB_MFP_Msk                  (0xFFFFul << SYS_GPB_MFP_GPB_MFP_Pos)      /*!< GCR_T::GPB_MFP: GPB_MFP Mask */

/* GCR GPCMFP Bit Field Definitions */
#define SYS_GPC_MFP_GPC_TYPE_Pos                 16                                         /*!< GCR_T::GPC_MFP: GPC_TYPE Position */
#define SYS_GPC_MFP_GPC_TYPE_Msk                 (0xFFFFul << SYS_GPC_MFP_GPC_TYPE_Pos)     /*!< GCR_T::GPC_MFP: GPC_TYPE Mask */

#define SYS_GPC_MFP_GPC_MFP_Pos                  0                                          /*!< GCR_T::GPC_MFP: GPC_MFP Position */
#define SYS_GPC_MFP_GPC_MFP_Msk                  (0xFFFFul << SYS_GPC_MFP_GPC_MFP_Pos)      /*!< GCR_T::GPC_MFP: GPC_MFP Mask */

/* GCR GPDMFP Bit Field Definitions */
#define SYS_GPD_MFP_GPD_TYPE_Pos                 16                                         /*!< GCR_T::GPD_MFP: GPD_TYPE Position */
#define SYS_GPD_MFP_GPD_TYPE_Msk                 (0xFFFFul << SYS_GPD_MFP_GPD_TYPE_Pos)     /*!< GCR_T::GPD_MFP: GPD_TYPE Mask */

#define SYS_GPD_MFP_GPD_MFP_Pos                  0                                          /*!< GCR_T::GPD_MFP: GPD_MFP Position */
#define SYS_GPD_MFP_GPD_MFP_Msk                  (0xFFFFul << SYS_GPD_MFP_GPD_MFP_Pos)      /*!< GCR_T::GPD_MFP: GPD_MFP Mask */

/* GCR ALTMFP Bit Field Definitions */
#define SYS_ALT_MFP_ALT_MFP_Pos                  0                                          /*!< GCR_T::ALT_MFP: ALT_MFP Position */
#define SYS_ALT_MFP_ALT_MFP_Msk                  (0x3FFFFFul << SYS_ALT_MFP_ALT_MFP_Pos)    /*!< GCR_T::ALT_MFP: ALT_MFP Mask */

#define SYS_ALT_MFP_PB9_S11_Pos                  1                                          /*!< GCR_T::ALT_MFP: PB9_S11 Position */
#define SYS_ALT_MFP_PB9_S11_Msk                  (1ul << SYS_ALT_MFP_PB9_S11_Pos)           /*!< GCR_T::ALT_MFP: PB9_S11 Mask */

#define SYS_ALT_MFP_PB10_S01_Pos                 0                                          /*!< GCR_T::ALT_MFP: PB10_S01 Position */
#define SYS_ALT_MFP_PB10_S01_Msk                 (1ul << SYS_ALT_MFP_PB10_S01_Pos)          /*!< GCR_T::ALT_MFP: PB10_S01 Mask */

/* GCR REGWRPROT Bit Field Definitions */
#define SYS_REGWRPROT_REGWRPROT_Pos              0                                          /*!< GCR_T::REGWRPROT: REGWRPROT Position */
#define SYS_REGWRPROT_REGWRPROT_Msk              (0xFFul << SYS_REGWRPROT_REGWRPROT_Pos)    /*!< GCR_T::REGWRPROT: REGWRPROT Mask */

#define SYS_REGWRPROT_REGPROTDIS_Pos             0                                          /*!< GCR_T::REGWRPROT: REGPROTDIS Position */
#define SYS_REGWRPROT_REGPROTDIS_Msk             (1ul << SYS_REGWRPROT_REGPROTDIS_Pos)      /*!< GCR_T::REGWRPROT: REGPROTDIS Mask */


/*@}*/ /* end of group REG_SYS_BITMASK */


typedef struct
{
    
    /**
     * IRQSRC
     * ===================================================================================================
     * Offset: 0x00~0x7C  IRQ0~IRQ31 Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     */       
    __I uint32_t IRQSRC[32];
    
    /**
     * NMISEL
     * ===================================================================================================
     * Offset: 0x80  NMI Source Interrupt Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |NMI_SEL   |NMI Interrupt Source Selection
     * |        |          |The NMI interrupt to Cortex-M0 can be selected from one of the peripheral interrupt by setting NMI_SEL.
     * |[7]     |NMI_TEST  |Interrupt Test Mode (Write Protect)
     * |        |          |Interrupt test mode. 
     * |        |          |Note: This bit is write protected. Refer to the REGWRPROT register.    
     */     
    __IO uint32_t NMISEL;
    

    /**
     * MCUIRQ
     * ===================================================================================================
     * Offset: 0x84  MCU Interrupt Request Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |MCU_IRQ   |MCU IRQ Source Register
     * |        |          |The MCU_IRQ collects all the interrupts from the peripherals and generates the synchronous interrupt to Cortex-M0.
     * |        |          |There are two modes to generate interrupt to Cortex-M0, the normal mode and test mode.
     * |        |          |The MCU_IRQ collects all interrupts from each peripheral and synchronizes them and interrupts the Cortex-M0.
     * |        |          |When the MCU_IRQ[n] is 0: Set MCU_IRQ[n] 1 will generate an interrupt to Cortex-M0 NVIC[n].
     * |        |          |When the MCU_IRQ[n] is 1 (mean an interrupt is assert), setting 1 to the MCU_IRQ[n] 1 will clear the interrupt and setting MCU_IRQ[n] 0: has no effect.
     */     
    __IO uint32_t MCUIRQ;
    
} GCR_INT_T;

/** @addtogroup REG_INT_BITMASK INT Bit Mask
  @{
 */

/* INT IRQSRC Bit Field Definitions */
#define INT_IRQ_SRC_INT_SRC_Pos                 0                                       /*!< GCR_INT_T::IRQSRC: INT_SRC Position */
#define INT_IRQ_SRC_INT_SRC_Msk                 (0xFul << INT_IRQ_SRC_INT_SRC_Pos)      /*!< GCR_INT_T::IRQSRC: INT_SRC Mask */

/* INT NMI_SEL Bit Field Definitions */
#define INT_NMI_SEL_INT_TEST_Pos                7                                       /*!< GCR_INT_T::NMISEL: INT_TEST Position */
#define INT_NMI_SEL_INT_TEST_Msk                (0x1Ful << INT_NMI_SEL_INT_TEST_Pos)    /*!< GCR_INT_T::NMISEL: INT_TEST Mask */

#define INT_NMI_SEL_NMI_SEL_Pos                 0                                       /*!< GCR_INT_T::NMISEL: NMI_SEL Position */
#define INT_NMI_SEL_NMI_SEL_Msk                 (0x1Ful << INT_NMI_SEL_NMI_SEL_Pos)     /*!< GCR_INT_T::NMISEL: NMI_SEL Mask */

/* INT MCUIRQ Bit Field Definitions */
#define INT_MCU_IRQ_MCU_IRQ_Pos                 0                                           /*!< GCR_INT_T::MCUIRQ: MCU_IRQ Position */
#define INT_MCU_IRQ_MCU_IRQ_Msk                 (0xFFFFFFFFul << INT_MCU_IRQ_MCU_IRQ_Pos)   /*!< GCR_INT_T::MCUIRQ: MCU_IRQ Mask */ 

/*@}*/ /* end of group REG_SYS_BITMASK */
/*@}*/ /* end of group REG_SYS*/

/*----------------------------- Timer Controller (TMR) -----------------------------*/
/** @addtogroup REG_TIMER Timer Controller (TIMER)
  Memory Mapped Structure for Timer Controller
  @{
 */
typedef struct
{
    /**
     * TCSR
     * ===================================================================================================
     * Offset: 0x00  Timer Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PRESCALE  |Prescale Counter
     * |        |          |Timer input clock source is divided by (PRESCALE+1) before it is fed to the Timer up counter.
     * |        |          |If this field is 0 (PRESCALE = 0), then there is no scaling.
     * |[16]    |TDR_EN    |Data Load Enable Control
     * |        |          |When TDR_EN is set, TDR (Timer Data Register) will be updated continuously with the 24-bit up-timer value as the timer is counting.
     * |        |          |0 = Timer Data Register update Disabled.
     * |        |          |1 = Timer Data Register update Enabled while Timer counter is active.
     * |[24]    |CTB       |Counter Mode Enable Control
     * |        |          |This bit is for external counting pin function enabled.
     * |        |          |When timer is used as an event counter, this bit should be set to 1 and select HCLK as timer clock source.
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
     * |        |          |If this bit is enabled, when the timer interrupt flag (TISR[0] TIF) is set to 1, the timer interrupt signal is generated and inform to CPU.
     * |[30]    |CEN       |Timer Enable Control
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note1: In stop status, and then set CEN to 1 will enable the 24-bit up counter to keep counting from the last stop counting value.
     * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (TCSR [28:27] = 00) when the timer interrupt flag (TISR[0] TIF) is generated.
     * |[31]    |DBGACK_TMR|ICE Debug Mode Acknowledge Disable (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgment effects TIMER counting.
     * |        |          |TIMER counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgment Disabled.
     * |        |          |TIMER counter will keep going no matter CPU is held by ICE or not.
     */
    __IO uint32_t  TCSR;

    /**
     * TCMPR
     * ===================================================================================================
     * Offset: 0x04  Timer Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |TCMP      |Timer Compared Value
     * |        |          |TCMP is a 24-bit compared value register.
     * |        |          |When the internal 24-bit up counter value is equal to TCMP value, the TIF flag will set to 1.
     * |        |          |Time-out period = (Period of Timer clock input) * (8-bit PRESCALE + 1) * (24-bit TCMP).
     * |        |          |Note1: Never write 0x0 or 0x1 in TCMP field, or the core will run into unknown state.
     * |        |          |Note2: When timer is operating at continuous counting mode, the 24-bit up counter will keep counting continuously even if user writes a new value into TCMP field.
     * |        |          |But if timer is operating at other modes, the 24-bit up counter will restart counting and using newest TCMP value to be the timer compared value if user writes a new value into TCMP field.
     */
    __IO uint32_t  TCMPR;

    /**
     * TISR
     * ===================================================================================================
     * Offset: 0x08  Timer Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Timer Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of Timer while TDR value reaches to TCMP value.
     * |        |          |0 = No effect.
     * |        |          |1 = TDR value matches the TCMP value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     */
    __IO uint32_t  TISR;

    /**
     * TDR
     * ===================================================================================================
     * Offset: 0x0C  Timer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |TDR       |Timer Data Register
     * |        |          |If TDR_EN (TCSR[16]) is set to 1, TDR register will be updated continuously to monitor 24-bit up counter value.
     */
    __I  uint32_t  TDR;
} TIMER_T;


/** @addtogroup REG_TIMER_BITMASK TIMER Bit Mask
  @{
 */


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
/*@}*/ /* end of group REG_TIMER_BITMASK */
/*@}*/ /* end of group REG_TIMER */


/*------------------------- UART Interface Controller ------------------------*/

/** @addtogroup REG_UART Universal Asynchronous Receiver/Transmitter Controller (UART)
  Memory Mapped Structure for UART Serial Interface Controller
  @{
 */
typedef struct
{

    union
    {
         /**
         * UA_DATA
         * ===================================================================================================
         * Offset: 0x00 UART Data Register
         * ---------------------------------------------------------------------------------------------------
         * |Bits    |Field     |Descriptions
         * | :----: | :----:   | :---- |
         * |[7:0]   |DATA      |Data Register
         * |        |          |By writing to this register, the UART will send out an 8-bit data through the UART_TXD pin (LSB first).
         * |        |          |By reading this register, the UART will return an 8-bit data received from UART_RXD pin (LSB first).
         */       
        __IO uint32_t DATA;

        /**
         * UA_THR
         * ===================================================================================================
         * Offset: 0x00 Transmit Holding DATA
         * ---------------------------------------------------------------------------------------------------
         * |Bits    |Field     |Descriptions
         * | :----: | :----:   | :---- |
         * |[7:0]   |THR       |Transmit Holding Register
         * |        |          |By writing to this register, the UART will send out an 8-bit data through the Tx pin (LSB first).
         */
        __IO uint32_t THR;

        /**
         * UA_RBR
         * ===================================================================================================
         * Offset: 0x00  UART Receive Buffer Register
         * ---------------------------------------------------------------------------------------------------
         * |Bits    |Field     |Descriptions
         * | :----: | :----:   | :---- |
         * |[7:0]   |RBR       |Receive Buffer Register (Read Only)
         * |        |          |By reading this register, the UART will return the 8-bit data received from RX pin (LSB first).
         */
        __IO uint32_t RBR;
    };

    /**
     * UA_IER
     * ===================================================================================================
     * Offset: 0x04  UART Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RDA_IEN   |Receive Data Available Interrupt Enable Control
     * |        |          |0 = RDA_INT Masked off.
     * |        |          |1 = RDA_INT Enabled.
     * |[1]     |THRE_IEN  |Transmit Holding Register Empty Interrupt Enable Control
     * |        |          |0 = THRE_INT Masked off.
     * |        |          |1 = THRE_INT Enabled.
     * |[2]     |RLS_IEN   |Receive Line Status Interrupt Enable Control
     * |        |          |0 = RLS_INT Masked off.
     * |        |          |1 = RLS_INT Enabled
     * |[3]     |MODEM_IEN |Modem Status Interrupt Enable Control
     * |        |          |0 = MODEM_INT Masked off.
     * |        |          |1 = MODEM_INT Enabled.
     * |[4]     |RTO_IEN   |RX Time-Out Interrupt Enable Control
     * |        |          |0 = TOUT_INT Masked off.
     * |        |          |1 = TOUT_INT Enabled.
     * |[5]     |BUF_ERR_IEN|Buffer Error Interrupt Enable Control
     * |        |          |0 = BUF_ERR_INT Masked off.
     * |        |          |1 = BUF_ERR_INT Enabled.
     * |[6]     |WAKE_EN   |UART Wake-Up Function Enable
     * |        |          |0 = UART wake-up function Disabled.
     * |        |          |1 = UART wake-up function Enabled, when the chip is in Power-down mode, an external CTS change will wake-up chip from Power-down mode.
     * |[11]    |TIME_OUT_EN|Rx Time-out Counter Enable
     * |        |          |0 = Time-out counter Disabled.
     * |        |          |1 = Time-out counter Enabled.
     * |[12]    |AUTO_RTS_EN|RTS Auto Flow Control Enable 
     * |        |          |0 = RTS auto flow control Disabled.
     * |        |          |1 = RTS auto flow control Enabled.
     * |        |          |When RTS auto-flow is enabled, if the number of bytes in the RX FIFO equals the RTS_TRI_LEV (UA_FCR [19:16]), the UART will de-assert RTS signal.
     * |[13]    |AUTO_CTS_EN|CTS Auto Flow Control Enable 
     * |        |          |0 = CTS auto flow control Disabled.
     * |        |          |1 = CTS auto flow control Enabled.
     * |        |          |When CTS auto-flow is enabled, the UART will send data to external device when CTS input assert (UART will not send data to device until CTS is asserted).
     */
    __IO uint32_t IER;

    /**
     * UA_FCR
     * ===================================================================================================
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
     * |        |          |When the number of bytes in the receive FIFO equals the RFITL, the RDA_IF will be set (if UA_IER [RDA_IEN] enabled, and an interrupt will be generated).
     * |        |          |0000 = RX FIFO Interrupt Trigger Level is 1 byte.
     * |        |          |0001 = RX FIFO Interrupt Trigger Level is 4 bytes.
     * |        |          |0010 = RX FIFO Interrupt Trigger Level is 8 bytes.
     * |        |          |0011 = RX FIFO Interrupt Trigger Level is 14 bytes.
     * |[8]     |RX_DIS    |Receiver Disable Control
     * |        |          |The receiver is disabled or not (set 1 to disable receiver).
     * |        |          |0 = Receiver Enabled.
     * |        |          |1 = Receiver Disabled.
     * |        |          |Note: This field is used for RS-485 Normal Multi-drop mode.
     * |        |          |It should be programmed before RS485_NMM (UA_ALT_CSR[8]) is programmed.
     * |[19:16] |RTS_TRI_LEV|RTS Trigger Level For Auto-Flow Control Use 
     * |        |          |0000 = RTS Trigger Level is 1 byte.
     * |        |          |0001 = RTS Trigger Level is 4 bytes.
     * |        |          |0010 = RTS Trigger Level is 8 bytes.
     * |        |          |0011 = RTS Trigger Level is 14 bytes.
     * |        |          |Note: This field is used for RTS auto-flow control.
     */
    __IO uint32_t FCR;

    /**
     * UA_LCR
     * ===================================================================================================
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
     * |[3]     |PBE       |Parity Bit Enable
     * |        |          |0 = No parity bit.
     * |        |          |1 = Parity bit is generated on each outgoing character and is checked on each incoming data.
     * |[4]     |EPE       |Even Parity Enable
     * |        |          |0 = Odd number of logic 1's is transmitted and checked in each word.
     * |        |          |1 = Even number of logic 1's is transmitted and checked in each word.
     * |        |          |This bit has effect only when PBE (UA_LCR[3]) is set.
     * |[5]     |SPE       |Stick Parity Enable
     * |        |          |0 = Stick parity Disabled.
     * |        |          |1 = If PBE (UA_LCR[3]) and EBE (UA_LCR[4]) are logic 1, the parity bit is transmitted and checked as logic 0.
     * |        |          |If PBE (UA_LCR[3]) is 1 and EBE (UA_LCR[4]) is 0 then the parity bit is transmitted and checked as 1.
     * |[6]     |BCB       |Break Control Bit
     * |        |          |When this bit is set to logic 1, the serial data output (TX) is forced to the Spacing State (logic 0).
     * |        |          |This bit acts only on TX and has no effect on the transmitter logic.
     */
    __IO uint32_t LCR;

    /**
     * UA_MCR
     * ===================================================================================================
     * Offset: 0x10  UART Modem Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RTS       |RTS (Request-To-Send) Signal Control 
     * |        |          |This bit is direct control internal RTS signal active or not, and then drive the RTS pin output with LEV_RTS bit configuration.
     * |        |          |0 = RTS signal is active.
     * |        |          |1 = RTS signal is inactive.
     * |        |          |Note1: This RTS signal control bit is not effective when RTS auto-flow control is enabled in UART function mode.
     * |        |          |Note2: This RTS signal control bit is not effective when RS-485 auto direction mode (AUD) is enabled in RS-485 function mode.
     * |[9]     |LEV_RTS   |RTS Pin Active Level 
     * |        |          |This bit defines the active level state of RTS pin output.
     * |        |          |0 = RTS pin output is high level active.
     * |        |          |1 = RTS pin output is low level active.
     * |[13]    |RTS_ST    |RTS Pin State (Read Only) 
     * |        |          |This bit mirror from RTS pin output of voltage logic status.
     * |        |          |0 = RTS pin output is low level voltage logic state.
     * |        |          |1 = RTS pin output is high level voltage logic state.
     */
    __IO uint32_t MCR;

    /**
     * UA_MSR
     * ===================================================================================================
     * Offset: 0x14  UART Modem Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DCTSF     |Detect CTS State Change Flag 
     * |        |          |This bit is set whenever CTS input has change state, and it will generate Modem interrupt to CPU when MODEM_IEN (UA_IER[3]) is set to 1.
     * |        |          |0 = CTS input has not change state.
     * |        |          |1 = CTS input has change state.
     * |        |          |Note: This bit can be cleared by writing "1" to it.
     * |[4]     |CTS_ST    |CTS Pin Status (Read Only)
     * |        |          |This bit mirror from CTS pin input of voltage logic status.
     * |        |          |0 = CTS pin input is low level voltage logic state.
     * |        |          |1 = CTS pin input is high level voltage logic state.
     * |        |          |Note: This bit echoes when UART Controller peripheral clock is enabled, and CTS multi-function port is selected.
     * |[8]     |LEV_CTS   |CTS Pin Active Level
     * |        |          |This bit defines the active level state of CTS pin input.
     * |        |          |0 = CTS pin input is high level active.
     * |        |          |1 = CTS pin input is low level active.
     */
    __IO uint32_t MSR;

    /**
     * UA_FSR
     * ===================================================================================================
     * Offset: 0x18  UART FIFO Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RX_OVER_IF|This bit is set when RX FIFO overflow.
     * |        |          |If the number of bytes of received data is greater than RX_FIFO (UA_RBR) size, this bit will be set.
     * |        |          |0 = RX FIFO is not overflow.
     * |        |          |1 = RX FIFO is overflow.
     * |        |          |Note: This bit can be cleared by writing "1" to it.   
     * |[3]     |RS485_ADD_DETF|RS-485 Address Byte Detection Flag     
     * |        |          |0 = Receiver detects a data that is not an address bit (bit 9 ='1').
     * |        |          |1 = Receiver detects a data that is an address bit (bit 9 ='1').
     * |        |          |Note1: This field is used for RS-485 function mode and RS485_ADD_EN (UA_ALT_CSR[15]) is set to 1 to enable Address detection mode .
     * |        |          |Note2: This bit can be cleared by writing "1" to it.  
     * |[4]     |PEF       |Parity Error Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "parity bit", and is reset whenever the CPU writes 1 to this bit.
     * |        |          |0 = No parity error is generated.
     * |        |          |1 = Parity error is generated.
     * |        |          |Note: This bit is read only, but can be cleared by writing "1" to RFR (UA_FCR [1]).
     * |[5]     |FEF       |Framing Error Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "stop bit" (that is, the stop bit following the last data bit or parity bit is detected as logic 0), and is reset whenever the CPU writes 1 to this bit.
     * |        |          |0 = No framing error is generated.
     * |        |          |1 = Framing error is generated.
     * |        |          |Note: This bit is read only, but can be cleared by writing "1" to RFR (UA_FCR [1]).
     * |[6]     |BIF       |Break Interrupt Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received data input(RX) is held in the "spacing state" (logic 0) for longer than a full word transmission time (that is, the total time of "start bit" + data bits + parity + stop bits) and is reset whenever the CPU writes 1 to this bit.
     * |        |          |0 = No Break interrupt is generated.
     * |        |          |1 = Break interrupt is generated.
     * |        |          |Note: This bit is read only, but can be cleared by writing "1" to RFR (UA_FCR [1]).
     * |[13:8]  |RX_POINTER|RX FIFO Pointer (Read Only)
     * |        |          |This field indicates the RX FIFO Buffer Pointer.
     * |        |          |When UART receives one byte from external device, then RX_POINTER increases one.
     * |        |          |When one byte of RX FIFO is read by CPU, then RX_POINTER decreases one.
     * |[14]    |RX_EMPTY  |Receiver FIFO Empty (Read Only)
     * |        |          |This bit initiate RX FIFO empty or not.
     * |        |          |0 = RX FIFO is not empty.
     * |        |          |1 = RX FIFO is empty.
     * |        |          |Note: When the last byte of RX FIFO has been read by CPU, hardware sets this bit high.
     * |        |          |It will be cleared when UART receives any new data.
     * |[15]    |RX_FULL   |Receiver FIFO Full (Read Only)
     * |        |          |This bit initiates RX FIFO is full or not.
     * |        |          |0 = RX FIFO is not full.
     * |        |          |1 = RX FIFO is full.
     * |        |          |Note: This bit is set when the number of usage in RX FIFO Buffer is more than 14, otherwise is cleared by hardware.
     * |[21:16] |TX_POINTER|TX FIFO Pointer (Read Only)
     * |        |          |This field indicates the TX FIFO Buffer Pointer. When CPU writes one byte into UA_THR, then TX_POINTER increases one.
     * |        |          |When one byte of TX FIFO is transferred to Transmitter Shift Register, then TX_POINTER decreases one.
     * |[22]    |TX_EMPTY  |Transmitter FIFO Empty (Read Only)
     * |        |          |This bit indicates TX FIFO empty or not.
     * |        |          |0 = TX FIFO is not empty.
     * |        |          |1 = TX FIFO is empty.
     * |        |          |Note: When the last byte of TX FIFO has been transferred to Transmitter Shift Register, hardware sets this bit high. It will be cleared when writing data into THR (TX FIFO not empty).
     * |[23]    |TX_FULL   |Transmitter FIFO Full (Read Only)
     * |        |          |This bit indicates TX FIFO full or not.
     * |        |          |0 = TX FIFO is not full.
     * |        |          |1 = TX FIFO is full.
     * |        |          |This bit is set when the number of usage in TX FIFO Buffer is more than 14, otherwise is cleared by hardware.
     * |[24]    |TX_OVER_IF|TX Overflow Error Interrupt Flag    
     * |        |          |If TX FIFO (UA_THR) is full, an additional write to UA_THR will cause this bit to logic 1.
     * |        |          |0 = TX FIFO is not overflow.
     * |        |          |1 = TX FIFO is overflow.
     * |        |          |Note: This bit can be cleared by writing "1" to it.   
     * |[28]    |TE_FLAG   |Transmitter Empty Flag (Read Only)
     * |        |          |This bit is set by hardware when TX FIFO (UA_THR) is empty and the STOP bit of the last byte has been transmitted.
     * |        |          |0 = TX FIFO is not empty or the STOP bit of the last byte has not been transmitted.
     * |        |          |1 = TX FIFO is empty and the STOP bit of the last byte has been transmitted.
     * |        |          |Note: This bit is cleared automatically when TX FIFO is not empty or the last byte transmission has not completed.
     */
    __IO uint32_t FSR;

    /**
     * UA_ISR
     * ===================================================================================================
     * Offset: 0x1C  UART Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RDA_IF    |Receive Data Available Interrupt Flag (Read Only)
     * |        |          |When the number of bytes in the RX FIFO equals the RFITL (UA_FCR[7:4]) then the RDA_IF(UA_ISR[0]) will be set.
     * |        |          |If RDA_IEN (UA_IER[0]) is enabled, the RDA interrupt will be generated.
     * |        |          |0 = No RDA interrupt flag is generated.
     * |        |          |1 = RDA interrupt flag is generated.
     * |        |          |Note: This bit is read only and it will be cleared when the number of unread bytes of RX FIFO drops below the threshold level (RFITL(UA_FCR[7:4]).
     * |[1]     |THRE_IF   |Transmit Holding Register Empty Interrupt Flag (Read Only)
     * |        |          |This bit is set when the last data of TX FIFO is transferred to Transmitter Shift Register.
     * |        |          |If THRE_IEN (UA_IER[1]) is enabled, the THRE interrupt will be generated.
     * |        |          |0 = No THRE interrupt flag is generated.
     * |        |          |1 = THRE interrupt flag is generated.
     * |        |          |Note: This bit is read only and it will be cleared when writing data into THR (TX FIFO not empty).
     * |[2]     |RLS_IF    |Receive Line Interrupt Flag (Read Only)
     * |        |          |This bit is set when the RX receive data have parity error, frame error or break error (at least one of 3 bits, BIF(UA_FSR[6]), FEF(UA_FSR[5]) and PEF(UA_FSR[4]), is set).
     * |        |          |If RLS_IEN (UA_IER[2]) is enabled, the RLS interrupt will be generated.
     * |        |          |0 = No RLS interrupt flag is generated.
     * |        |          |1 = RLS interrupt flag is generated.
     * |        |          |Note1: In RS-485 function mode, this field is set include receiver detect and received address byte character (bit9 = '1') bit.
     * |        |          |At the same time, the bit of UA_FSR[RS485_ADD_DETF] is also set.
     * |        |          |Note2: This bit is read only and reset to 0 when all bits of BIF(UA_FSR[6]), FEF(UA_FSR[5]) and PEF(UA_FSR[4]) are cleared.
     * |        |          |Note3: In RS-485 function mode, this bit is read only and reset to 0 when all bits of BIF(UA_FSR[6]) , FEF(UA_FSR[5]) and PEF(UA_FSR[4]) and RS485_ADD_DETF (UA_FSR[3]) are cleared.
     * |[3]     |MODEM_IF  |MODEM Interrupt Flag (Read Only) (Not Available In UART2 Channel)
     * |        |          |This bit is set when the CTS pin has state change (DCTSF (UA_MSR[0]) = 1).
     * |        |          |If MODEM_IEN (UA_IER[3]) is enabled, the Modem interrupt will be generated.
     * |        |          |0 = No Modem interrupt flag is generated.
     * |        |          |1 = Modem interrupt flag is generated.
     * |        |          |Note: This bit is read only and reset to 0 when bit DCTSF is cleared by a write 1 on DCTSF(UA_MSR[0]).
     * |[4]     |TOUT_IF   |Time-Out Interrupt Flag (Read Only)
     * |        |          |This bit is set when the RX FIFO is not empty and no activities occurred in the RX FIFO and the time-out counter equal to TOIC.
     * |        |          |If TOUT_IEN (UA_IER[4]) is enabled, the Tout interrupt will be generated.
     * |        |          |0 = No Time-out interrupt flag is generated.
     * |        |          |1 = Time-out interrupt flag is generated.
     * |        |          |Note: This bit is read only and user can read UA_RBR (RX is in active) to clear it
     * |[5]     |BUF_ERR_IF|Buffer Error Interrupt Flag (Read Only)
     * |        |          |This bit is set when the TX FIFO or RX FIFO overflows (TX_OVER_IF (UA_FSR[24]) or RX_OVER_IF (UA_FSR[0]) is set).
     * |        |          |When BUF_ERR_IF (UA_ISR[5])is set, the transfer is not correct.
     * |        |          |If BUF_ERR_IEN (UA_IER[8]) is enabled, the buffer error interrupt will be generated.
     * |        |          |0 = No buffer error interrupt flag is generated.
     * |        |          |1 = Buffer error interrupt flag is generated.
     * |        |          |Note: This bit is read only and reset to 0 when all bits of TX_OVER_IF(UA_FSR[24]) and RX_OVER_IF(UA_FSR[0]) are cleared
     * |[8]     |RDA_INT   |Receive Data Available Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RDA_IEN (UA_IER[0]) and RDA_IF (UA_ISR[0]) are both set to 1.
     * |        |          |0 = No RDA interrupt is generated.
     * |        |          |1 = RDA interrupt is generated.
     * |[9]     |THRE_INT  |Transmit Holding Register Empty Interrupt Indicator (Read Only)
     * |        |          |This bit is set if THRE_IEN (UA_IER[1])and THRE_IF(UA_SR[1]) are both set to 1.
     * |        |          |0 = No THRE interrupt is generated.
     * |        |          |1 = THRE interrupt is generated.
     * |[10]    |RLS_INT   |Receive Line Status Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RLS_IEN (UA_IER[2]) and RLS_IF(UA_ISR[2]) are both set to 1.
     * |        |          |0 = No RLS interrupt is generated.
     * |        |          |1 = RLS interrupt is generated
     * |[11]    |MODEM_INT |MODEM Status Interrupt Indicator (Read Only) (Not Available In UART2 Channel)
     * |        |          |This bit is set if MODEM_IEN(UA_IER[3] and MODEM_IF(UA_ISR[4]) are both set to 1
     * |        |          |0 = No Modem interrupt is generated.
     * |        |          |1 = Modem interrupt is generated.
     * |[12]    |TOUT_INT  |Receive Buffer Time-out Interrupt Indicator (Read Only)
     * |        |          |This bit is set if TOUT_IEN (UA_IER[4]) and TOUT_IF (UA_ISR[4]) are both set to 1.
     * |        |          |0 = No Receive Buffer Time-out interrupt is generated.
     * |        |          |1 = Receive Buffer Time-out interrupt is generated.
     * |[13]    |BUF_ERR_INT|Buffer Error Interrupt Indicator (Read Only)
     * |        |          |This bit is set if BUF_ERR_IEN(UA_IER[5] and BUF_ERR_IF(UA_ISR[5]) are both set to 1.
     * |        |          |0 = No buffer error interrupt is generated.
     * |        |          |1 = Buffer error interrupt is generated.
     */
    __IO uint32_t ISR;

    /**
     * UA_TOR
     * ===================================================================================================
     * Offset: 0x20  UART Time-out Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TOIC      |Time-out Interrupt Comparator     
     * |        |          |The time-out counter resets and starts counting (counting clock = baud rate) whenever the RX FIFO receives a new data word if time-out counter is enabled by setting TIME_OUT_EN(UA_IER [11]). 
     * |        |          |Once the content of time-out counter is equal to that of time-out interrupt comparator (TOIC (UA_TOR[7:0])), a receiver time-out interrupt (TOUT_INT (UA_ISR[12])) is generated if RTO_IEN (UA_IER[4]). 
     * |        |          |A new incoming data word or RX FIFO empty clears TOUT_IF (UA_ISR[4]). 
     * |        |          |In order to avoid receiver time-out interrupt generation immediately during one character is being received, TOIC value should be set between 40 and 255. 
     * |        |          |Thus, for example, if TOIC is set as 40, the time-out interrupt is generated after four characters are not received when 1 stop bit and no parity check is set for UART transfer.     
     * |[15:8]  |DLY       |TX Delay Time Value
     * |        |          |This field is used to programming the transfer delay time between the last stop bit and next start bit.
     */
    __IO uint32_t TOR;

    /**
     * UA_BAUD
     * ===================================================================================================
     * Offset: 0x24  UART Baud Rate Divisor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |BRD       |Baud Rate Divider
     * |        |          |The field indicates the baud rate divider.
     * |[27:24] |DIVIDER_X |Divider X
     * |        |          |The baud rate divider M = X+1.
     * |[28]    |DIV_X_ONE |Divider X Equal To 1
     * |        |          |0 = Divider M = X.
     * |        |          |1 = Divider M = 1.
     * |[29]    |DIV_X_EN  |Divider X Enable
     * |        |          |The BRD = Baud Rate Divider, and the baud rate equation is Baud Rate = Clock / [M * (BRD + 2)]; The default value of M is 16.
     * |        |          |0 = Divider X Disabled (the equation of M = 16).
     * |        |          |1 = Divider X Enabled (the equation of M = X+1, but DIVIDER_X [27:24] must >= 8).
     * |        |          |Note: In IrDA mode, this bit must disable.
     */
    __IO uint32_t BAUD;

    /**
     * UA_IRCR
     * ===================================================================================================
     * Offset: 0x28  UART IrDA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TX_SELECT |TX_SELECT
     * |        |          |0 = IrDA Transmitter Disabled and Receiver Enabled.
     * |        |          |1 = IrDA Transmitter Enabled and Receiver Disabled.
     * |[5]     |INV_TX    |IrDA inverse Transmitting Output Signal Control
     * |        |          |0 = None inverse transmitting signal.
     * |        |          |1 = Inverse transmitting output signal.
     * |[6]     |INV_RX    |IrDA inverse Receive Input Signal Control
     * |        |          |0 = None inverse receiving input signal.
     * |        |          |1 = Inverse receiving input signal.
     */
    __IO uint32_t IRCR;

    /**
     * UA_ALT_CSR
     * ===================================================================================================
     * Offset: 0x2C  UART Alternate Control/Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |RS485_NMM |RS-485 Normal Multi-Drop Operation Mode (NMM)
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
     * |[15]    |RS485_ADD_EN|RS-485 Address Detection Enable
     * |        |          |This bit is used to enable RS-485 Address Detection mode.
     * |        |          |0 = Address detection mode Disabled.
     * |        |          |1 = Address detection mode Enabled.
     * |        |          |Note: This bit is used for RS-485 any operation mode.
     * |[31:24] |ADDR_MATCH|Address Match Value Register
     * |        |          |This field contains the RS-485 address match values.
     * |        |          |Note: This field is used for RS-485 auto address detection mode.
     */
    __IO uint32_t ALT_CSR;

    /**
     * UA_FUN_SEL
     * ===================================================================================================
     * Offset: 0x30  UART Function Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |FUN_SEL   |Function Select Enable
     * |        |          |00 = UART function Enabled.
     * |        |          |10 = IrDA function Enabled.
     * |        |          |11 = RS-485 function Enabled.
     */
    __IO uint32_t FUN_SEL;

} UART_T;


/** @addtogroup REG_UART_BITMASK UART Bit Mask
  @{
 */

/* UART THR Bit Field Definitions */
#define UART_THR_THR_Pos         0                                          /*!< UART_T::THR: THR Position               */
#define UART_THR_THR_Msk        (0xFul << UART_THR_THR_Pos)                 /*!< UART_T::THR: THR Mask                   */

/* UART RBR Bit Field Definitions */
#define UART_RBR_RBR_Pos         0                                          /*!< UART_T::RBR: RBR Position               */
#define UART_RBR_RBR_Msk        (0xFul << UART_RBR_RBR_Pos)                 /*!< UART_T::RBR: RBR Mask                   */

/* UART IER Bit Field Definitions */
#define UART_IER_AUTO_CTS_EN_Pos    13                                      /*!< UART_T::IER: AUTO_CTS_EN Position       */
#define UART_IER_AUTO_CTS_EN_Msk    (1ul << UART_IER_AUTO_CTS_EN_Pos)       /*!< UART_T::IER: AUTO_CTS_EN Mask           */

#define UART_IER_AUTO_RTS_EN_Pos    12                                      /*!< UART_T::IER: AUTO_RTS_EN Position       */
#define UART_IER_AUTO_RTS_EN_Msk    (1ul << UART_IER_AUTO_RTS_EN_Pos)       /*!< UART_T::IER: AUTO_RTS_EN Mask           */

#define UART_IER_TIME_OUT_EN_Pos    11                                      /*!< UART_T::IER: TIME_OUT_EN Position       */
#define UART_IER_TIME_OUT_EN_Msk    (1ul << UART_IER_TIME_OUT_EN_Pos)       /*!< UART_T::IER: TIME_OUT_EN Mask           */

#define UART_IER_WAKE_EN_Pos        6                                       /*!< UART_T::IER: WAKE_EN Position           */
#define UART_IER_WAKE_EN_Msk        (1ul << UART_IER_WAKE_EN_Pos)           /*!< UART_T::IER: WAKE_EN Mask               */

#define UART_IER_BUF_ERR_IEN_Pos    5                                       /*!< UART_T::IER: BUF_ERR_IEN Position       */
#define UART_IER_BUF_ERR_IEN_Msk    (1ul << UART_IER_BUF_ERR_IEN_Pos)       /*!< UART_T::IER: BUF_ERR_IEN Mask           */

#define UART_IER_RTO_IEN_Pos        4                                       /*!< UART_T::IER: RTO_IEN Position           */
#define UART_IER_RTO_IEN_Msk        (1ul << UART_IER_RTO_IEN_Pos)           /*!< UART_T::IER: RTO_IEN Mask               */

#define UART_IER_MODEM_IEN_Pos      3                                       /*!< UART_T::IER: MODEM_IEN Position         */
#define UART_IER_MODEM_IEN_Msk      (1ul << UART_IER_MODEM_IEN_Pos)         /*!< UART_T::IER: MODEM_IEN Mask             */

#define UART_IER_RLS_IEN_Pos        2                                       /*!< UART_T::IER: RLS_IEN Position           */
#define UART_IER_RLS_IEN_Msk        (1ul << UART_IER_RLS_IEN_Pos)           /*!< UART_T::IER: RLS_IEN Mask               */

#define UART_IER_THRE_IEN_Pos       1                                       /*!< UART_T::IER: THRE_IEN Position          */
#define UART_IER_THRE_IEN_Msk       (1ul << UART_IER_THRE_IEN_Pos)          /*!< UART_T::IER: THRE_IEN Mask              */

#define UART_IER_RDA_IEN_Pos        0                                       /*!< UART_T::IER: RDA_IEN Position           */
#define UART_IER_RDA_IEN_Msk        (1ul << UART_IER_RDA_IEN_Pos)           /*!< UART_T::IER: RDA_IEN Mask               */

/* UART FCR Bit Field Definitions */
#define UART_FCR_RTS_TRI_LEV_Pos    16                                      /*!< UART_T::FCR: RTS_TRI_LEV Position       */
#define UART_FCR_RTS_TRI_LEV_Msk    (0xFul << UART_FCR_RTS_TRI_LEV_Pos)     /*!< UART_T::FCR: RTS_TRI_LEV Mask           */

#define UART_FCR_RX_DIS_Pos         8                                       /*!< UART_T::FCR: RX_DIS Position            */
#define UART_FCR_RX_DIS_Msk         (1ul << UART_FCR_RX_DIS_Pos)            /*!< UART_T::FCR: RX_DIS Mask                */

#define UART_FCR_RFITL_Pos          4                                       /*!< UART_T::FCR: RFITL Position             */
#define UART_FCR_RFITL_Msk          (0xFul << UART_FCR_RFITL_Pos)           /*!< UART_T::FCR: RFITL Mask                 */

#define UART_FCR_TFR_Pos            2                                       /*!< UART_T::FCR: TFR Position               */
#define UART_FCR_TFR_Msk            (1ul << UART_FCR_TFR_Pos)               /*!< UART_T::FCR: TFR Mask                   */

#define UART_FCR_RFR_Pos            1                                       /*!< UART_T::FCR: RFR Position               */
#define UART_FCR_RFR_Msk            (1ul << UART_FCR_RFR_Pos)               /*!< UART_T::FCR: RFR Mask                   */

/* UART LCR Bit Field Definitions */
#define UART_LCR_BCB_Pos            6                                       /*!< UART_T::LCR: BCB Position               */
#define UART_LCR_BCB_Msk            (1ul << UART_LCR_BCB_Pos)               /*!< UART_T::LCR: BCB Mask                   */

#define UART_LCR_SPE_Pos            5                                       /*!< UART_T::LCR: SPE Position               */
#define UART_LCR_SPE_Msk            (1ul << UART_LCR_SPE_Pos)               /*!< UART_T::LCR: SPE Mask                   */

#define UART_LCR_EPE_Pos            4                                       /*!< UART_T::LCR: EPE Position               */
#define UART_LCR_EPE_Msk            (1ul << UART_LCR_EPE_Pos)               /*!< UART_T::LCR: EPE Mask                   */

#define UART_LCR_PBE_Pos            3                                       /*!< UART_T::LCR: PBE Position               */
#define UART_LCR_PBE_Msk            (1ul << UART_LCR_PBE_Pos)               /*!< UART_T::LCR: PBE Mask                   */

#define UART_LCR_NSB_Pos            2                                       /*!< UART_T::LCR: NSB Position               */
#define UART_LCR_NSB_Msk            (1ul << UART_LCR_NSB_Pos)               /*!< UART_T::LCR: NSB Mask                   */

#define UART_LCR_WLS_Pos            0                                       /*!< UART_T::LCR: WLS Position               */
#define UART_LCR_WLS_Msk            (0x3ul << UART_LCR_WLS_Pos)             /*!< UART_T::LCR: WLS Mask                   */

/* UART MCR Bit Field Definitions */
#define UART_MCR_RTS_ST_Pos         13                                      /*!< UART_T::MCR: RTS_ST Position            */
#define UART_MCR_RTS_ST_Msk         (1ul << UART_MCR_RTS_ST_Pos)            /*!< UART_T::MCR: RTS_ST Mask                */

#define UART_MCR_LEV_RTS_Pos        9                                       /*!< UART_T::MCR: LEV_RTS Position           */
#define UART_MCR_LEV_RTS_Msk        (1ul << UART_MCR_LEV_RTS_Pos)           /*!< UART_T::MCR: LEV_RTS Mask               */

#define UART_MCR_RTS_Pos            1                                       /*!< UART_T::MCR: RTS Position               */
#define UART_MCR_RTS_Msk            (1ul << UART_MCR_RTS_Pos)               /*!< UART_T::MCR: RTS Mask                   */

/* UART MSR Bit Field Definitions */
#define UART_MSR_LEV_CTS_Pos        8                                       /*!< UART_T::MSR: LEV_CTS Position           */
#define UART_MSR_LEV_CTS_Msk        (1ul << UART_MSR_LEV_CTS_Pos)           /*!< UART_T::MSR: LEV_CTS Mask               */

#define UART_MSR_CTS_ST_Pos         4                                       /*!< UART_T::MSR: CTS_ST Position            */
#define UART_MSR_CTS_ST_Msk         (1ul << UART_MSR_CTS_ST_Pos)            /*!< UART_T::MSR: CTS_ST Mask                */

#define UART_MSR_DCTSF_Pos          0                                       /*!< UART_T::MSR: DCTST Position             */
#define UART_MSR_DCTSF_Msk          (1ul << UART_MSR_DCTSF_Pos)             /*!< UART_T::MSR: DCTST Mask                 */

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
#define UART_TOR_DLY_Pos            8                                       /*!< UART_T::TOR: DLY Position               */
#define UART_TOR_DLY_Msk            (0xFFul << UART_TOR_DLY_Pos)            /*!< UART_T::TOR: DLY Mask                   */

#define UART_TOR_TOIC_Pos           0                                       /*!< UART_T::TOR: TOIC Position              */
#define UART_TOR_TOIC_Msk           (0xFFul << UART_TOR_TOIC_Pos)           /*!< UART_T::TOR: TOIC Mask                  */

/* UART BAUD Bit Field Definitions */
#define UART_BAUD_DIV_X_EN_Pos      29                                      /*!< UART_T::BAUD: DIV_X_EN Position         */
#define UART_BAUD_DIV_X_EN_Msk      (1ul << UART_BAUD_DIV_X_EN_Pos)         /*!< UART_T::BAUD: DIV_X_EN Mask             */
  
#define UART_BAUD_DIV_X_ONE_Pos     28                                      /*!< UART_T::BAUD: DIV_X_ONE Position        */
#define UART_BAUD_DIV_X_ONE_Msk     (1ul << UART_BAUD_DIV_X_ONE_Pos)        /*!< UART_T::BAUD: DIV_X_ONE Mask            */
  
#define UART_BAUD_DIVIDER_X_Pos     24                                      /*!< UART_T::BAUD: DIVIDER_X Position        */
#define UART_BAUD_DIVIDER_X_Msk     (0xFul << UART_BAUD_DIVIDER_X_Pos)      /*!< UART_T::BAUD: DIVIDER_X Mask            */
  
#define UART_BAUD_BRD_Pos           0                                       /*!< UART_T::BAUD: BRD Position              */
#define UART_BAUD_BRD_Msk           (0xFFFFul << UART_BAUD_BRD_Pos)         /*!< UART_T::BAUD: BRD Mask                  */

/* UART IRCR Bit Field Definitions */
#define UART_IRCR_INV_RX_Pos        6                                       /*!< UART_T::IRCR: INV_RX Position           */
#define UART_IRCR_INV_RX_Msk        (1ul << UART_IRCR_INV_RX_Pos)           /*!< UART_T::IRCR: INV_RX Mask               */

#define UART_IRCR_INV_TX_Pos        5                                       /*!< UART_T::IRCR: INV_TX Position           */
#define UART_IRCR_INV_TX_Msk        (1ul << UART_IRCR_INV_TX_Pos)           /*!< UART_T::IRCR: INV_TX Mask               */

#define UART_IRCR_TX_SELECT_Pos     1                                       /*!< UART_T::IRCR: TX_SELECT Position        */
#define UART_IRCR_TX_SELECT_Msk     (1ul << UART_IRCR_TX_SELECT_Pos)        /*!< UART_T::IRCR: TX_SELECT Mask            */

/* UART ALT_CSR Bit Field Definitions */
#define UART_ALT_CSR_ADDR_MATCH_Pos     24                                       /*!< UART_T::ALT_CSR: ADDR_MATCH Position    */
#define UART_ALT_CSR_ADDR_MATCH_Msk     (0xFFul << UART_ALT_CSR_ADDR_MATCH_Pos)  /*!< UART_T::ALT_CSR: ADDR_MATCH Mask        */

#define UART_ALT_CSR_RS485_ADD_EN_Pos   15                                       /*!< UART_T::ALT_CSR: RS485_ADD_EN Position  */
#define UART_ALT_CSR_RS485_ADD_EN_Msk   (1ul << UART_ALT_CSR_RS485_ADD_EN_Pos)   /*!< UART_T::ALT_CSR: RS485_ADD_EN Mask      */

#define UART_ALT_CSR_RS485_AUD_Pos      10                                       /*!< UART_T::ALT_CSR: RS485_AUD Position     */
#define UART_ALT_CSR_RS485_AUD_Msk      (1ul << UART_ALT_CSR_RS485_AUD_Pos)      /*!< UART_T::ALT_CSR: RS485_AUD Mask         */

#define UART_ALT_CSR_RS485_AAD_Pos      9                                        /*!< UART_T::ALT_CSR: RS485_AAD Position     */
#define UART_ALT_CSR_RS485_AAD_Msk      (1ul << UART_ALT_CSR_RS485_AAD_Pos)      /*!< UART_T::ALT_CSR: RS485_AAD Mask         */

#define UART_ALT_CSR_RS485_NMM_Pos      8                                        /*!< UART_T::ALT_CSR: RS485_NMM Position     */
#define UART_ALT_CSR_RS485_NMM_Msk      (1ul << UART_ALT_CSR_RS485_NMM_Pos)      /*!< UART_T::ALT_CSR: RS485_NMM Mask         */

/* UART FUN_SEL Bit Field Definitions */
#define UART_FUN_SEL_FUN_SEL_Pos        0                                        /*!< UART_T::FUN_SEL: FUN_SEL Position       */
#define UART_FUN_SEL_FUN_SEL_Msk        (0x3ul << UART_FUN_SEL_FUN_SEL_Pos)      /*!< UART_T::FUN_SEL: FUN_SEL Mask           */

/*@}*/ /* end of group REG_UART_BITMASK */
/*@}*/ /* end of group REG_UART */

/*--------------------------- USB Device Controller --------------------------*/
/** @addtogroup REG_USBD USB Device Controller (USBD)
  Memory Mapped Structure for USB Device Controller
  @{
 */

typedef struct
{
    /**
     * USB_BUFSEG0~5
     * ===================================================================================================
     *     Offset: 0x20/0x30/0x40/0x50/0x60/0x70  Endpoint 0~5 Buffer Segmentation Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:3]   |BUFSEG    |Endpoint Buffer Segmentation
     * |        |          |It is used to indicate the offset address for each endpoint with the USB SRAM starting address The effective starting address of the endpoint is
     * |        |          |USB_SRAM address + { BUFSEG[8:3], 3'b000}
     * |        |          |Where the USB_SRAM address = USBD_BA+0x100h.
     * |        |          |Refer to the section 5.4.4.7 for the endpoint SRAM structure and its description.
     */
    __IO uint32_t BUFSEG;

    /**
     * USB_MXPLD0~5
     * ===================================================================================================
     *     Offset: 0x24/0x34/0x44/0x54/0x64/0x74  Endpoint 0~5 Maximal Payload Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |MXPLD     |Maximal Payload
     * |        |          |Define the data length which is transmitted to host (IN token) or the actual data length which is received from the host (OUT token).
     * |        |          |It also used to indicate that the endpoint is ready to be transmitted in IN token or received in OUT token.
     * |        |          |(1) When the register is written by CPU,
     * |        |          |For IN token, the value of MXPLD is used to define the data length to be transmitted and indicate the data buffer is ready.
     * |        |          |For OUT token, it means that the controller is ready to receive data from the host and the value of MXPLD is the maximal data length comes from host.
     * |        |          |(2) When the register is read by CPU,
     * |        |          |For IN token, the value of MXPLD is indicated by the data length be transmitted to host
     * |        |          |For OUT token, the value of MXPLD is indicated the actual data length receiving from host.
     * |        |          |Note: Once MXPLD is written, the data packets will be transmitted/received immediately after IN/OUT token arrived.
     */
    __IO uint32_t MXPLD;

    /**
     * USB_CFG0~5
     * ===================================================================================================
     *     Offset: 0x28/0x38/0x48/0x58/0x68/0x78  Endpoint 0~5 Configuration Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |EP_NUM    |Endpoint Number
     * |        |          |These bits are used to define the endpoint number of the current endpoint.
     * |[4]     |ISOCH     |Isochronous Endpoint
     * |        |          |This bit is used to set the endpoint as Isochronous endpoint, no handshake.
     * |        |          |0 = No Isochronous endpoint.
     * |        |          |1 = Isochronous endpoint.
     * |[6:5]   |STATE     |Endpoint STATE
     * |        |          |00 = Endpoint is Disabled.
     * |        |          |01 = Out endpoint.
     * |        |          |10 = IN endpoint.
     * |        |          |11 = Undefined.
     * |[7]     |DSQ_SYNC  |Data Sequence Synchronization
     * |        |          |0 = DATA0 PID.
     * |        |          |1 = DATA1 PID.
     * |        |          |Note: It is used to specify the DATA0 or DATA1 PID in the following IN token transaction.
     * |        |          |Hardware will toggle automatically in IN token base on the bit.
     * |[9]     |CSTALL    |Clear STALL Response
     * |        |          |0 = Disable the device to clear the STALL handshake in setup stage.
     * |        |          |1 = Clear the device to response STALL handshake in setup stage.
     */
    __IO uint32_t CFG;      /*!< Endpoint Configuration Register   */

    /**
    * USB_CFGP0~5
    * ===================================================================================================
    * Offset: 0x2C/0x3C/0x4C/0x5C/0x6C/0x7C  Endpoint 0~5 Set Stall and Clear In/Out Ready Control Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |CLRRDY    |Clear Ready
    * |        |          |When the USB_MXPLD register is set by user, it means that the endpoint is ready to transmit or receive data.
    * |        |          |If the user wants to turn off this transaction before the transaction start, users can set this bit to 1 to turn it off and it will be cleared to 0 automatically.
    * |        |          |For IN token, write '1' to clear the IN token had ready to transmit the data to USB.
    * |        |          |For OUT token, write '1' to clear the OUT token had ready to receive the data from USB.
    * |        |          |This bit is write 1 only and is always 0 when it is read back.
    * |[1]     |SSTALL    |Set STALL
    * |        |          |0 = Disable the device to response STALL.
    * |        |          |1 = Set the device to respond STALL automatically.
    */
    __IO uint32_t CFGP;     /*!< Endpoint Set Stall and Clear In/Out Ready Control Register */

} USBD_EP_T;


typedef struct
{
    /**
     * USB_INTEN
     * ===================================================================================================
     * Offset: 0x00  USB Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUS_IE    |Bus Event Interrupt Enable
     * |        |          |0 = BUS event interrupt Disabled.
     * |        |          |1 = BUS event interrupt Enabled.
     * |[1]     |USB_IE    |USB Event Interrupt Enable
     * |        |          |0 = USB event interrupt Disabled.
     * |        |          |1 = USB event interrupt Enabled.
     * |[2]     |FLDET_IE  |Floating Detection Interrupt Enable
     * |        |          |0 = Floating detection Interrupt Disabled.
     * |        |          |1 = Floating detection Interrupt Enabled.
     * |[3]     |WAKEUP_IE |USB Wake-Up Interrupt Enable
     * |        |          |0 = Wake-up Interrupt Disabled.
     * |        |          |1 = Wake-up Interrupt Enabled.
     * |[8]     |WAKEUP_EN |Wake-Up Function Enable
     * |        |          |0 = USB wake-up function Disabled.
     * |        |          |1 = USB wake-up function Enabled.
     * |[15]    |INNAK_EN  |Active NAK Function And Its Status In IN Token
     * |        |          |0 = When device responds NAK after receiving IN token, IN NAK status will not be
     * |        |          |    updated to USBD_EPSTS register, so that the USB interrupt event will not be asserted.
     * |        |          |1 = IN NAK status will be updated to USBD_EPSTS register and the USB interrupt event
     * |        |          |    will be asserted, when the device responds NAK after receiving IN token.
     */
    __IO uint32_t INTEN;

    /**
     * USB_INTSTS
     * ===================================================================================================
     * Offset: 0x04  USB Interrupt Event Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUS_STS   |BUS Interrupt Status
     * |        |          |The BUS event means that there is one of the suspense or the resume function in the bus.
     * |        |          |0 = No BUS event occurred.
     * |        |          |1 = Bus event occurred; check USB_ATTR[3:0] to know which kind of bus event was occurred, cleared by write 1 to USB_INTSTS[0].
     * |[1]     |USB_STS   |USB Event Interrupt Status
     * |        |          |The USB event includes the SETUP Token, IN Token, OUT ACK, ISO IN, or ISO OUT events in the bus.
     * |        |          |0 = No USB event occurred.
     * |        |          |1 = USB event occurred, check EPSTS0~7 to know which kind of USB event occurred.
     * |        |          |Cleared by write 1 to USB_INTSTS[1] or EPEVT0~7 and SETUP (USB_INTSTS[31]).
     * |[2]     |FLDET_STS |Floating Detection Interrupt Status
     * |        |          |0 = There is not attached/detached event in the USB.
     * |        |          |1 = There is attached/detached event in the USB bus and it is cleared by write 1 to USB_INTSTS[2].
     * |[3]     |WAKEUP_STS|Wake-Up Interrupt Status
     * |        |          |0 = No Wake-up event occurred.
     * |        |          |1 = Wake-up event occurred, cleared by write 1 to USB_INTSTS[3].
     * |[16]    |EPEVT0    |Endpoint 0's USB Event Status
     * |        |          |0 = No event occurred on endpoint 0.
     * |        |          |1 = USB event occurred on Endpoint 0, check USB_EPSTS[10:8] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[16] or USB_INTSTS[1].
     * |[17]    |EPEVT1    |Endpoint 1's USB Event Status
     * |        |          |0 = No event occurred on endpoint 1.
     * |        |          |1 = USB event occurred on Endpoint 1, check USB_EPSTS[13:11] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[17] or USB_INTSTS[1].
     * |[18]    |EPEVT2    |Endpoint 2's USB Event Status
     * |        |          |0 = No event occurred on endpoint 2.
     * |        |          |1 = USB event occurred on Endpoint 2, check USB_EPSTS[16:14] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[18] or USB_INTSTS[1].
     * |[19]    |EPEVT3    |Endpoint 3's USB Event Status
     * |        |          |0 = No event occurred on endpoint 3.
     * |        |          |1 = USB event occurred on Endpoint 3, check USB_EPSTS[19:17] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[19] or USB_INTSTS[1].
     * |[20]    |EPEVT4    |Endpoint 4's USB Event Status
     * |        |          |0 = No event occurred on endpoint 4.
     * |        |          |1 = USB event occurred on Endpoint 4, check USB_EPSTS[22:20] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[20] or USB_INTSTS[1].
     * |[21]    |EPEVT5    |Endpoint 5's USB Event Status
     * |        |          |0 = No event occurred on endpoint 5.
     * |        |          |1 = USB event occurred on Endpoint 5, check USB_EPSTS[25:23] to know which kind of USB event was occurred, cleared by write 1 to USB_INTSTS[21] or USB_INTSTS[1].
     * |[31]    |SETUP     |Setup Event Status
     * |        |          |0 = No Setup event.
     * |        |          |1 = SETUP event occurred, cleared by write 1 to USB_INTSTS[31].
     */
    __IO uint32_t INTSTS;

    /**
     * USB_FADDR
     * ===================================================================================================
     * Offset: 0x08  USB Device Function Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6:0]   |FADDR     |USB Device Function Address
     */
    __IO uint32_t FADDR;

    /**
     * USB_EPSTS
     * ===================================================================================================
     * Offset: 0x0C  USB Endpoint Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7]     |OVERRUN   |Overrun
     * |        |          |It indicates that the received data is over the maximum payload number or not.
     * |        |          |0 = No overrun.
     * |        |          |1 = Out Data is more than the Max Payload in MXPLD register or the Setup Data is more than 8 Bytes.
     * |[10:8]  |EPSTS0    |Endpoint 0 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[13:11] |EPSTS1    |Endpoint 1 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[16:14] |EPSTS2    |Endpoint 2 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[19:17] |EPSTS3    |Endpoint 3 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[22:20] |EPSTS4    |Endpoint 4 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     * |[25:23] |EPSTS5    |Endpoint 5 Bus Status
     * |        |          |These bits are used to indicate the current status of this endpoint
     * |        |          |000 = In ACK.
     * |        |          |001 = In NAK.
     * |        |          |010 = Out Packet Data0 ACK.
     * |        |          |110 = Out Packet Data1 ACK.
     * |        |          |011 = Setup ACK.
     * |        |          |111 = Isochronous transfer end.
     */
    __I  uint32_t EPSTS;

    /**
     * USB_ATTR
     * ===================================================================================================
     * Offset: 0x10  USB Bus Status and Attribution Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |USBRST    |USB Reset Status
     * |        |          |0 = Bus no reset.
     * |        |          |1 = Bus reset when SE0 (single-ended 0) is presented more than 2.5us.
     * |        |          |Note: This bit is read only.
     * |[1]     |SUSPEND   |Suspend Status
     * |        |          |0 = Bus no suspend.
     * |        |          |1 = Bus idle more than 3ms, either cable is plugged off or host is sleeping.
     * |        |          |Note: This bit is read only.
     * |[2]     |RESUME    |Resume Status
     * |        |          |0 = No bus resume.
     * |        |          |1 = Resume from suspend.
     * |        |          |Note: This bit is read only.
     * |[3]     |TIMEOUT   |Time-Out Status
     * |        |          |0 = No time-out.
     * |        |          |1 = No Bus response more than 18 bits time.
     * |        |          |Note: This bit is read only.
     * |[4]     |PHY_EN    |PHY Transceiver Function Enable
     * |        |          |0 = PHY transceiver function Disabled.
     * |        |          |1 = PHY transceiver function Enabled.
     * |[5]     |RWAKEUP   |Remote Wake-Up
     * |        |          |0 = Release the USB bus from K state.
     * |        |          |1 = Force USB bus to K (USB_D+ low, USB_D- high) state, used for remote wake-up.
     * |[7]     |USB_EN    |USB Controller Enable
     * |        |          |0 = USB Controller Disabled.
     * |        |          |1 = USB Controller Enabled.
     * |[8]     |DPPU_EN   |Pull-Up Resistor On USB_D+ Enable
     * |        |          |0 = Pull-up resistor in USB_D+ pin Disabled.
     * |        |          |1 = Pull-up resistor in USB_D+ pin Enabled.
     * |[9]     |PWRDN     |Power-Down PHY Transceiver, Low Active
     * |        |          |0 = Power-down related circuit of PHY transceiver.
     * |        |          |1 = Turn-on related circuit of PHY transceiver.
     * |[10]    |BYTEM     |CPU Access USB SRAM Size Mode Selection
     * |        |          |0 = Word mode: The size of the transfer from CPU to USB SRAM can be Word only.
     * |        |          |1 = Byte mode: The size of the transfer from CPU to USB SRAM can be Byte only.
     */
    __IO uint32_t ATTR;

    /**
     * USB_FLDET
     * ===================================================================================================
     * Offset: 0x14  USB Floating Detection Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FLDET     |Device Floating Detected
     * |        |          |0 = Controller is not attached into the USB host.
     * |        |          |1 =Controller is attached into the BUS.
     */
    __I  uint32_t FLDET;

    /**
     * USB_STBUFSEG
     * ===================================================================================================
     * Offset: 0x18  Setup Token Buffer Segmentation Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:3]   |STBUFSEG  |Setup Token Buffer Segmentation
     * |        |          |It is used to indicate the offset address for the SETUP token with the USB Device SRAM starting address The effective starting address is
     * |        |          |USB_SRAM address + {STBUFSEG[8:3], 3'b000}
     * |        |          |Where the USB_SRAM address = USBD_BA+0x100h.
     * |        |          |Note: It is used for SETUP token only.
     */
    __IO uint32_t STBUFSEG;

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE0;
    /**
     * @endcond
     */

    USBD_EP_T EP[6];

    /**
     * @cond HIDDEN_SYMBOLS
     */
    __I  uint32_t RESERVE1[4];
    /**
     * @endcond
     */

    /**
     * USB_DRVSE0
     * ===================================================================================================
     * Offset: 0x90  USB Drive SE0 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DRVSE0    |Drive Single Ended Zero In USB Bus
     * |        |          |The Single Ended Zero (SE0) is when both lines (USB_D+ and USB_D-) are being pulled low.
     * |        |          |0 = None.
     * |        |          |1 = Force USB PHY transceiver to drive SE0.
     */
    __IO uint32_t DRVSE0;

} USBD_T;


/** @addtogroup REG_USBD_BITMASK USBD Bit Mask
  @{
 */

/* USBD INTEN Bit Field Definitions */
#define USBD_INTEN_INNAK_EN_Pos    15                                    /*!< USBD_T::INTEN: INNAK_EN Position */
#define USBD_INTEN_INNAK_EN_Msk    (1ul << USBD_INTEN_INNAK_EN_Pos)      /*!< USBD_T::INTEN: INNAK_EN Mask */

#define USBD_INTEN_WAKEUP_EN_Pos   8                                     /*!< USBD_T::INTEN: RWAKEUP Position */
#define USBD_INTEN_WAKEUP_EN_Msk   (1ul << USBD_INTEN_WAKEUP_EN_Pos)     /*!< USBD_T::INTEN: RWAKEUP Mask */

#define USBD_INTEN_WAKEUP_IE_Pos   3                                     /*!< USBD_T::INTEN: WAKEUP_IE Position */
#define USBD_INTEN_WAKEUP_IE_Msk   (1ul << USBD_INTEN_WAKEUP_IE_Pos)     /*!< USBD_T::INTEN: WAKEUP_IE Mask */

#define USBD_INTEN_FLDET_IE_Pos    2                                     /*!< USBD_T::INTEN: FLDET_IE Position */
#define USBD_INTEN_FLDET_IE_Msk    (1ul << USBD_INTEN_FLDET_IE_Pos)      /*!< USBD_T::INTEN: FLDET_IE Mask */

#define USBD_INTEN_USB_IE_Pos      1                                     /*!< USBD_T::INTEN: USB_IE Position */
#define USBD_INTEN_USB_IE_Msk      (1ul << USBD_INTEN_USB_IE_Pos)        /*!< USBD_T::INTEN: USB_IE Mask */

#define USBD_INTEN_BUS_IE_Pos      0                                     /*!< USBD_T::INTEN: BUS_IE Position */
#define USBD_INTEN_BUS_IE_Msk      (1ul << USBD_INTEN_BUS_IE_Pos)        /*!< USBD_T::INTEN: BUS_IE Mask */

/* USBD INTSTS Bit Field Definitions */                                          
#define USBD_INTSTS_SETUP_Pos        31                                  /*!< USBD_T::INTSTS: SETUP Position */
#define USBD_INTSTS_SETUP_Msk        (1ul << USBD_INTSTS_SETUP_Pos)      /*!< USBD_T::INTSTS: SETUP Mask */

#define USBD_INTSTS_EPEVT_Pos        16                                  /*!< USBD_T::INTSTS: EPEVT Position */
#define USBD_INTSTS_EPEVT_Msk        (0xFFul << USBD_INTSTS_EPEVT_Pos)   /*!< USBD_T::INTSTS: EPEVT Mask */

#define USBD_INTSTS_WAKEUP_STS_Pos   3                                   /*!< USBD_T::INTSTS: WAKEUP_STS Position */
#define USBD_INTSTS_WAKEUP_STS_Msk   (1ul << USBD_INTSTS_WAKEUP_STS_Pos) /*!< USBD_T::INTSTS: WAKEUP_STS Mask */

#define USBD_INTSTS_FLDET_STS_Pos    2                                   /*!< USBD_T::INTSTS: FLDET_STS Position */
#define USBD_INTSTS_FLDET_STS_Msk    (1ul << USBD_INTSTS_FLDET_STS_Pos)  /*!< USBD_T::INTSTS: FLDET_STS Mask */

#define USBD_INTSTS_USB_STS_Pos      1                                   /*!< USBD_T::INTSTS: USB_STS Position */
#define USBD_INTSTS_USB_STS_Msk      (1ul << USBD_INTSTS_USB_STS_Pos)    /*!< USBD_T::INTSTS: USB_STS Mask */

#define USBD_INTSTS_BUS_STS_Pos      0                                   /*!< USBD_T::INTSTS: BUS_STS Position */
#define USBD_INTSTS_BUS_STS_Msk      (1ul << USBD_INTSTS_BUS_STS_Pos)    /*!< USBD_T::INTSTS: BUS_STS Mask */

/* USBD FADDR Bit Field Definitions */                                           
#define USBD_FADDR_FADDR_Pos     0                                       /*!< USBD_T::FADDR: FADDR Position */
#define USBD_FADDR_FADDR_Msk     (0x7Ful << USBD_FADDR_FADDR_Pos)        /*!< USBD_T::FADDR: FADDR Mask */

/* USBD EPSTS Bit Field Definitions */                                           
#define USBD_EPSTS_EPSTS5_Pos    23                                      /*!< USBD_T::EPSTS: EPSTS5 Position */
#define USBD_EPSTS_EPSTS5_Msk    (7ul << USBD_EPSTS_EPSTS5_Pos)          /*!< USBD_T::EPSTS: EPSTS5 Mask */

#define USBD_EPSTS_EPSTS4_Pos    20                                      /*!< USBD_T::EPSTS: EPSTS4 Position */
#define USBD_EPSTS_EPSTS4_Msk    (7ul << USBD_EPSTS_EPSTS4_Pos)          /*!< USBD_T::EPSTS: EPSTS4 Mask */

#define USBD_EPSTS_EPSTS3_Pos    17                                      /*!< USBD_T::EPSTS: EPSTS3 Position */
#define USBD_EPSTS_EPSTS3_Msk    (7ul << USBD_EPSTS_EPSTS3_Pos)          /*!< USBD_T::EPSTS: EPSTS3 Mask */

#define USBD_EPSTS_EPSTS2_Pos    14                                      /*!< USBD_T::EPSTS: EPSTS2 Position */
#define USBD_EPSTS_EPSTS2_Msk    (7ul << USBD_EPSTS_EPSTS2_Pos)          /*!< USBD_T::EPSTS: EPSTS2 Mask */

#define USBD_EPSTS_EPSTS1_Pos    11                                      /*!< USBD_T::EPSTS: EPSTS1 Position */
#define USBD_EPSTS_EPSTS1_Msk    (7ul << USBD_EPSTS_EPSTS1_Pos)          /*!< USBD_T::EPSTS: EPSTS1 Mask */

#define USBD_EPSTS_EPSTS0_Pos    8                                       /*!< USBD_T::EPSTS: EPSTS0 Position */
#define USBD_EPSTS_EPSTS0_Msk    (7ul << USBD_EPSTS_EPSTS0_Pos)          /*!< USBD_T::EPSTS: EPSTS0 Mask */

#define USBD_EPSTS_OVERRUN_Pos   7                                       /*!< USBD_T::EPSTS: OVERRUN Position */
#define USBD_EPSTS_OVERRUN_Msk   (1ul << USBD_EPSTS_OVERRUN_Pos)         /*!< USBD_T::EPSTS: OVERRUN Mask */

/* USBD ATTR Bit Field Definitions */
#define USBD_ATTR_BYTEM_Pos      10                                      /*!< USBD_T::ATTR: BYTEM Position */
#define USBD_ATTR_BYTEM_Msk      (1ul << USBD_ATTR_BYTEM_Pos)            /*!< USBD_T::ATTR: BYTEM Mask */

#define USBD_ATTR_PWRDN_Pos      9                                       /*!< USBD_T::ATTR: PWRDN Position */
#define USBD_ATTR_PWRDN_Msk      (1ul << USBD_ATTR_PWRDN_Pos)            /*!< USBD_T::ATTR: PWRDN Mask */

#define USBD_ATTR_DPPU_EN_Pos    8                                       /*!< USBD_T::ATTR: DPPU_EN Position */
#define USBD_ATTR_DPPU_EN_Msk    (1ul << USBD_ATTR_DPPU_EN_Pos)          /*!< USBD_T::ATTR: DPPU_EN Mask */

#define USBD_ATTR_USB_EN_Pos     7                                       /*!< USBD_T::ATTR: USB_EN Position */
#define USBD_ATTR_USB_EN_Msk     (1ul << USBD_ATTR_USB_EN_Pos)           /*!< USBD_T::ATTR: USB_EN Mask */

#define USBD_ATTR_RWAKEUP_Pos    5                                       /*!< USBD_T::ATTR: RWAKEUP Position */
#define USBD_ATTR_RWAKEUP_Msk    (1ul << USBD_ATTR_RWAKEUP_Pos)          /*!< USBD_T::ATTR: RWAKEUP Mask */

#define USBD_ATTR_PHY_EN_Pos     4                                       /*!< USBD_T::ATTR: PHY_EN Position */
#define USBD_ATTR_PHY_EN_Msk     (1ul << USBD_ATTR_PHY_EN_Pos)           /*!< USBD_T::ATTR: PHY_EN Mask */

#define USBD_ATTR_TIMEOUT_Pos    3                                       /*!< USBD_T::ATTR: TIMEOUT Position */
#define USBD_ATTR_TIMEOUT_Msk    (1ul << USBD_ATTR_TIMEOUT_Pos)          /*!< USBD_T::ATTR: TIMEOUT Mask */

#define USBD_ATTR_RESUME_Pos     2                                       /*!< USBD_T::ATTR: RESUME Position */
#define USBD_ATTR_RESUME_Msk     (1ul << USBD_ATTR_RESUME_Pos)           /*!< USBD_T::ATTR: RESUME Mask */

#define USBD_ATTR_SUSPEND_Pos    1                                       /*!< USBD_T::ATTR: SUSPEND Position */
#define USBD_ATTR_SUSPEND_Msk    (1ul << USBD_ATTR_SUSPEND_Pos)          /*!< USBD_T::ATTR: SUSPEND Mask */

#define USBD_ATTR_USBRST_Pos     0                                       /*!< USBD_T::ATTR: USBRST Position */
#define USBD_ATTR_USBRST_Msk     (1ul << USBD_ATTR_USBRST_Pos)           /*!< USBD_T::ATTR: USBRST Mask */

/* USBD FLDET Bit Field Definitions */                                           
#define USBD_FLDET_FLDET_Pos     0                                       /*!< USBD_T::FLDET: FLDET Position */
#define USBD_FLDET_FLDET_Msk     (1ul << USBD_FLDET_FLDET_Pos)           /*!< USBD_T::FLDET: FLDET Mask */

/* USBD STBUFSEG Bit Field Definitions */
#define USBD_STBUFSEG_STBUFSEG_Pos   3                                        /*!< USBD_T::STBUFSEG: STBUFSEG Position */
#define USBD_STBUFSEG_STBUFSEG_Msk   (0x3Ful << USBD_STBUFSEG_STBUFSEG_Pos)   /*!< USBD_T::STBUFSEG: STBUFSEG Mask */

/* USBD BUFSEG Bit Field Definitions */
#define USBD_BUFSEG_BUFSEG_Pos   3                                       /*!< USBD_EP_T::BUFSEG: BUFSEG Position */
#define USBD_BUFSEG_BUFSEG_Msk   (0x3Ful << USBD_BUFSEG_BUFSEG_Pos)      /*!< USBD_EP_T::BUFSEG: BUFSEG Mask */

/* USBD MXPLD Bit Field Definitions */                                           
#define USBD_MXPLD_MXPLD_Pos    0                                        /*!< USBD_EP_T::MXPLD: MXPLD Position */
#define USBD_MXPLD_MXPLD_Msk    (0x1FFul << USBD_MXPLD_MXPLD_Pos)        /*!< USBD_EP_T::MXPLD: MXPLD Mask */

/* USBD CFG Bit Field Definitions */                                             
#define USBD_CFG_CSTALL_Pos     9                                        /*!< USBD_EP_T::CFG: CSTALL Position */
#define USBD_CFG_CSTALL_Msk     (1ul << USBD_CFG_CSTALL_Pos)             /*!< USBD_EP_T::CFG: CSTALL Mask */

#define USBD_CFG_DSQ_SYNC_Pos   7                                        /*!< USBD_EP_T::CFG: DSQ_SYNC Position */
#define USBD_CFG_DSQ_SYNC_Msk   (1ul << USBD_CFG_DSQ_SYNC_Pos)           /*!< USBD_EP_T::CFG: DSQ_SYNC Mask */

#define USBD_CFG_STATE_Pos      5                                        /*!< USBD_EP_T::CFG: STATE Position */
#define USBD_CFG_STATE_Msk      (3ul << USBD_CFG_STATE_Pos)              /*!< USBD_EP_T::CFG: STATE Mask */

#define USBD_CFG_ISOCH_Pos      4                                        /*!< USBD_EP_T::CFG: ISOCH Position */
#define USBD_CFG_ISOCH_Msk      (1ul << USBD_CFG_ISOCH_Pos)              /*!< USBD_EP_T::CFG: ISOCH Mask */

#define USBD_CFG_EP_NUM_Pos     0                                        /*!< USBD_EP_T::CFG: EP_NUM Position */
#define USBD_CFG_EP_NUM_Msk     (0xFul << USBD_CFG_EP_NUM_Pos)           /*!< USBD_EP_T::CFG: EP_NUM Mask */

/* USBD CFGP Bit Field Definitions */                                            
#define USBD_CFGP_SSTALL_Pos    1                                        /*!< USBD_EP_T::CFGP: SSTALL Position */
#define USBD_CFGP_SSTALL_Msk    (1ul << USBD_CFGP_SSTALL_Pos)            /*!< USBD_EP_T::CFGP: SSTALL Mask */

#define USBD_CFGP_CLRRDY_Pos    0                                        /*!< USBD_EP_T::CFGP: CLRRDY Position */
#define USBD_CFGP_CLRRDY_Msk    (1ul << USBD_CFGP_CLRRDY_Pos)            /*!< USBD_EP_T::CFGP: CLRRDY Mask */

/* USBD DRVSE0 Bit Field Definitions */                                          
#define USBD_DRVSE0_DRVSE0_Pos   0                                       /*!< USBD_EP_T::DRVSE0: DRVSE0 Position */
#define USBD_DRVSE0_DRVSE0_Msk   (1ul << USBD_DRVSE0_DRVSE0_Pos)         /*!< USBD_EP_T::DRVSE0: DRVSE0 Mask */
/*@}*/ /* end of group REG_USBD_BITMASK */
/*@}*/ /* end of group REG_USBD */


/*----------------------------- Watchdog Timer (WDT) -----------------------------*/
/** @addtogroup REG_WDT Watchdog Timer (WDT)
  Memory Mapped Structure for Watchdog Timer
  @{
 */
typedef struct
{
    /**
     * WTCR
     * ===================================================================================================
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
     * |[4]     |WTWKE     |Watchdog Timer Time-out Wake-Up Function Control
     * |        |          |(Write Protect)
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
     * |        |          |Note: If CWDTEN (CONFIG0[31] Watchdog Enable) bit is set to 0, this bit is forced as 1 and
     * |        |          | user cannot change this bit to 0.
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
     */
    __IO uint32_t  WTCR;
} WDT_T;


/** @addtogroup REG_WDT_BITMASK WDT Bit Mask
  @{
 */

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
/*@}*/ /* end of group REG_WDT_BITMASK */
/*@}*/ /* end of group REG_WDT */
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
#define GPIO_BASE           (AHB_BASE        + 0x4000)                   /*!< GPIO Base Address                                   */
#define PA_BASE             (GPIO_BASE               )                   /*!< GPIO PORTA Base Address                             */
#define PB_BASE             (GPIO_BASE       + 0x0040)                   /*!< GPIO PORTB Base Address                             */
#define PC_BASE             (GPIO_BASE       + 0x0080)                   /*!< GPIO PORTC Base Address                             */
#define PD_BASE             (GPIO_BASE       + 0x00C0)                   /*!< GPIO PORTD Base Address                             */
#define GPIO_DBNCECON_BASE  (GPIO_BASE       + 0x0180)                   /*!< GPIO De-bounce Cycle Control Base Address           */
#define GPIO_PIN_DATA_BASE  (GPIO_BASE       + 0x0200)                   /*!< GPIO Pin Data Input/Output Control Base Address     */


#define UART0_BASE           (APB1_BASE      + 0x50000)                 /*!< UART0 Base Address                               */
#define UART1_BASE           (APB2_BASE      + 0x50000)                 /*!< UART1 Base Address                               */


#define TIMER0_BASE          (APB1_BASE      + 0x10000)                 /*!< Timer0 Base Address                              */
#define TIMER1_BASE          (APB1_BASE      + 0x10020)                 /*!< Timer1 Base Address                              */
#define TIMER2_BASE          (APB2_BASE      + 0x10000)                 /*!< Timer2 Base Address                              */
#define TIMER3_BASE          (APB2_BASE      + 0x10020)                 /*!< Timer3 Base Address                              */

#define WDT_BASE             (APB1_BASE      + 0x4000)                  /*!< Watchdog Timer Base Address                      */

#define SPI0_BASE            (APB1_BASE      + 0x30000)                 /*!< SPI0 Base Address                                */
#define SPI1_BASE            (APB1_BASE      + 0x34000)                 /*!< SPI1 Base Address                                */

#define I2C1_BASE            (APB2_BASE      + 0x20000)                 /*!< I2C1 Base Address                                */

#define RTC_BASE             (APB1_BASE      + 0x08000)                 /*!< RTC Base Address                                 */

#define CLK_BASE             (AHB_BASE       + 0x00200)                 /*!< System Clock Controller Base Address             */

#define GCR_BASE             (AHB_BASE       + 0x00000)                 /*!< System Global Controller Base Address            */

#define INT_BASE             (AHB_BASE       + 0x00300)                 /*!< Interrupt Source Controller Base Address         */

#define FMC_BASE             (AHB_BASE       + 0x0C000)

#define PS2_BASE             (APB2_BASE      + 0x00000)                 /*!< PS/2 Base Address                                */

#define USBD_BASE            (APB1_BASE      + 0x60000)                 /*!< USBD Base Address                                */

#define PWMA_BASE            (APB1_BASE      + 0x40000)                 /*!< PWMA Base Address                                */

/*@}*/ /* end of group PERIPHERAL_MEM_MAP */

/******************************************************************************/
/*                         Peripheral Definitions                             */
/******************************************************************************/

/** @addtogroup PERIPHERAL Peripheral Definitions
  The Definitions of Peripheral
  @{
 */
#define PA                  ((GPIO_T *) PA_BASE)                        /*!< GPIO PORTA Configuration Struct                        */
#define PB                  ((GPIO_T *) PB_BASE)                        /*!< GPIO PORTB Configuration Struct                        */
#define PC                  ((GPIO_T *) PC_BASE)                        /*!< GPIO PORTC Configuration Struct                        */
#define PD                  ((GPIO_T *) PD_BASE)                        /*!< GPIO PORTD Configuration Struct                        */
#define GPIO                ((GPIO_DBNCECON_T *) GPIO_DBNCECON_BASE)    /*!< Interrupt De-bounce Cycle Control Configuration Struct */

#define UART0               ((UART_T *) UART0_BASE)                     /*!< UART0 Configuration Struct                       */
#define UART1               ((UART_T *) UART1_BASE)                     /*!< UART1 Configuration Struct                       */

#define TIMER0              ((TIMER_T *) TIMER0_BASE)                   /*!< Timer0 Configuration Struct                      */
#define TIMER1              ((TIMER_T *) TIMER1_BASE)                   /*!< Timer1 Configuration Struct                      */
#define TIMER2              ((TIMER_T *) TIMER2_BASE)                   /*!< Timer2 Configuration Struct                      */
#define TIMER3              ((TIMER_T *) TIMER3_BASE)                   /*!< Timer3 Configuration Struct                      */

#define WDT                 ((WDT_T *) WDT_BASE)                        /*!< Watchdog Timer Configuration Struct              */

#define SPI0                ((SPI_T *) SPI0_BASE)                       /*!< SPI0 Configuration Struct                        */
#define SPI1                ((SPI_T *) SPI1_BASE)                       /*!< SPI1 Configuration Struct                        */

#define I2C1                ((I2C_T *) I2C1_BASE)                       /*!< I2C1 Configuration Struct                        */

#define RTC                 ((RTC_T *) RTC_BASE)                        /*!< RTC Configuration Struct                         */

#define CLK                 ((CLK_T *) CLK_BASE)                        /*!< System Clock Controller Configuration Struct     */

#define SYS                 ((GCR_T *) GCR_BASE)                        /*!< System Global Controller Configuration Struct    */

#define SYSINT              ((GCR_INT_T *) INT_BASE)                    /*!< Interrupt Source Controller Configuration Struct */

#define FMC                 ((FMC_T *) FMC_BASE)                        /*!< FMC Configuration Struct                         */

#define PS2                 ((PS2_T *) PS2_BASE)                        /*!< PS/2 Configuration Struct                        */

#define USBD                ((USBD_T *) USBD_BASE)                      /*!< USBD Configuration Struct                        */

#define PWMA                ((PWM_T *) PWMA_BASE)                       /*!< PWMA Configuration Struct                        */

/*@}*/ /* end of group PERIPHERAL */

#define UNLOCKREG()        do{*((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x59;*((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x16;*((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x88;}while(*((__IO uint32_t *)(GCR_BASE + 0x100))==0)
#define LOCKREG()          *((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x00

#define REGCOPY(dest, src)  *((uint32_t *)&(dest)) = *((uint32_t *)&(src))
#define CLEAR(dest)         *((uint32_t *)&(dest)) = 0

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
#define BYTE0_Msk               (0x000000FF)
#define BYTE1_Msk               (0x0000FF00)
#define BYTE2_Msk               (0x00FF0000)
#define BYTE3_Msk               (0xFF000000)

#define _GET_BYTE0(u32Param)    (((u32Param) & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define _GET_BYTE1(u32Param)    (((u32Param) & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define _GET_BYTE2(u32Param)    (((u32Param) & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define _GET_BYTE3(u32Param)    (((u32Param) & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/*@}*/ /* end of group legacy_Constants */


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "SYS.h"
#include "FMC.h"
#include "GPIO.h"
#include "I2C.h"
#include "PWM.h"
#include "SPI.h"
#include "TIMER.h"
#include "WDT.h"
#include "UART.h"
#include "USBD.h"
#include "PS2.h"
#include "CLK.h"
#include "RTC.h"

#endif


