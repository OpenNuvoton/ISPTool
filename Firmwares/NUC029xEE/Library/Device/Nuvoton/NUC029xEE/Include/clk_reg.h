/**************************************************************************//**
 * @file     clk_reg.h
 * @version  V1.00
 * @brief    CLK register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __CLK_REG_H__
#define __CLK_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */


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
 * |        |          |The bit default value is set by flash controller user configuration register CFOSC(Config0[26:24]).
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
 * |        |          |0 = Power-down mode wake-up interrupt Disabled.
 * |        |          |1 = Power-down mode wake-up interrupt Enabled.
 * |        |          |Note1: The interrupt will occur when both PD_WU_STS and PD_WU_INT_EN are high.
 * |        |          |Note2: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[6]     |PD_WU_STS |Power-Down Mode Wake-Up Interrupt Status
 * |        |          |Set by "Power-down wake-up event", it indicates that resume from Power-down mode.
 * |        |          |The flag is set if the GPIO, USB, UART, WDT, I2C, TIMER, ACMP, BOD or RTC wake-up occurred.
 * |        |          |Write 1 to clear the bit to 0.
 * |        |          |Note: This bit is working only if PD_WU_INT_EN (PWRCON[5]) set to 1.
 * |[7]     |PWR_DOWN_EN|System Power-Down Enable Bit (Write Protect)
 * |        |          |When this bit is set to 1, Power-down mode is enabled and chip Power-down behavior willdepends on the PD_WAIT_CPU bit.
 * |        |          |(a) If the PD_WAIT_CPU is 0, then the chip enters Power-down mode immediately after the PWR_DOWN_EN bit set.
 * |        |          |(b) if the PD_WAIT_CPU is 1, then the chip keeps active till the CPU sleep mode is also active and then the chip enters Power-down mode. (recommend)
 * |        |          |When chip wakes up from Power-down mode, this bit is cleared by hardware.
 * |        |          |User needs to set this bit again for next Power-down.
 * |        |          |In Power-down mode, external 4~24 MHz high speed crystal oscillator and the internal 22.1184 MHz high speed oscillator will be disabled in this mode, 
 * |        |          |but the external 32.768 kHz low speed crystal and internal 10 kHz low speed oscillator are not controlled by Power-down mode.
 * |        |          |In Power- down mode, the PLL and system clock are disabled, and ignored the clock source selection.
 * |        |          |The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from external 32.768 kHz low speed crystal oscillator or the internal 10 kHz low speed oscillator.
 * |        |          |0 = Chip operating normally or chip in Idle mode because of WFI command.
 * |        |          |1 = Chip enters Power-down mode instantly or waits CPU sleep command WFI.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[8]     |PD_WAIT_CPU|This Bit Control The Power-Down Entry Condition (Write Protect)
 * |        |          |0 = Chip enters Power-down mode when the PWR_DOWN_EN bit is set to 1.
 * |        |          |1 = Chip enters Power- down mode when the both PD_WAIT_CPU and PWR_DOWN_EN bits are set to 1 and CPU run WFI instruction.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[12]    |OSC48M_EN |48 MHz Internal High Speed RC Oscillator (HIRC48) Enable Bit (Write Protect)
 * |        |          |0 = 48 MHz internal high speed RC oscillator (HIRC48) Disabled.
 * |        |          |1 = 48 MHz internal high speed RC oscillator (HIRC48) Enabled.
 * |        |          |Note: This bit is write protected. Refer to the REGWRPROT register.
 * @var CLK_T::AHBCLK
 * Offset: 0x04  AHB Devices Clock Enable Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |PDMA_EN   |PDMA Controller Clock Enable Control
 * |        |          |0 = PDMA peripherial clock Disabled.
 * |        |          |1 = PDMA peripherial clock Enabled.
 * |[2]     |ISP_EN    |Flash ISP Controller Clock Enable Control
 * |        |          |0 = Flash ISP peripheral clock Disabled.
 * |        |          |1 = Flash ISP peripheral clock Enabled.    
 * |[3]     |EBI_EN    |EBI Controller Clock Enable Control
 * |        |          |0 = EBI peripherial clock Disabled.
 * |        |          |1 = EBI peripherial clock Enabled.
 * @var CLK_T::APBCLK
 * Offset: 0x08  APB Devices Clock Enable Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WDT_EN    |Watchdog Timer Clock Enable Control (Write Protect)
 * |        |          |0 = Watchdog Timer clock Disabled.
 * |        |          |1 = Watchdog Timer clock Enabled.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[1]     |RTC_EN    |Real-Time-Clock APB Interface Clock Enable Control
 * |        |          |This bit is used to control the RTC APB clock only, The RTC peripheral clock source is selected from RTC_SEL_10K(CLKSEL2[18]).
 * |        |          |It can be selected to the external 32.768 kHz low speed crystal or Internal 10 kHz low speed oscillator.
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
 * |[6]     |FDIV_EN   |Frequency Divider Output Clock Enable Control
 * |        |          |0 = FDIV clock Disabled.
 * |        |          |1 = FDIV clock Enabled.
 * |[8]     |I2C0_EN   |I2C0 Clock Enable Control
 * |        |          |0 = I2C0 clock Disabled.
 * |        |          |1 = I2C0 clock Enabled.
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
 * |[18]    |UART2_EN  |UART2 Clock Enable Control
 * |        |          |0 = UART2 clock Disabled.
 * |        |          |1 = UART2 clock Enabled.
 * |[20]    |PWM01_EN  |PWM_01 Clock Enable Control
 * |        |          |0 = PWM01 clock Disabled.
 * |        |          |1 = PWM01 clock Enabled.
 * |[21]    |PWM23_EN  |PWM_23 Clock Enable Control
 * |        |          |0 = PWM23 clock Disabled.
 * |        |          |1 = PWM23 clock Enabled.
 * |[22]    |PWM45_EN  |PWM_45 Clock Enable Control
 * |        |          |0 = PWM45 clock Disabled.
 * |        |          |1 = PWM45 clock Enabled.
 * |[27]    |USBD_EN   |USB 2.0 FS Device Controller Clock Enable Control
 * |        |          |0 = USB clock Disabled.
 * |        |          |1 = USB clock Enabled.
 * |[28]    |ADC_EN    |Analog-Digital-Converter (ADC) Clock Enable Control
 * |        |          |0 = ADC clock Disabled.
 * |        |          |1 = ADC clock Enabled.
 * @var CLK_T::CLKSTATUS
 * Offset: 0x0C  Clock status monitor Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |XTL12M_STB|External 4~24 MHz High Speed Crystal (HXT) Clock Source Stable Flag (Read Only)
 * |        |          |0 = External 4~24 MHz high speed crystal clock (HXT) is not stable or disabled.
 * |        |          |1 = External 4~24 MHz high speed crystal clock (HXT) is stable and enabled.
 * |[1]     |XTL32K_STB|External 32.768 KHz Low Speed Crystal (LXT) Clock Source Stable Flag(Read Only)
 * |        |          |0 = External 32.768 kHz low speed crystal (LXT) clock is not stable or disabled.
 * |        |          |1 = External 32.768 kHz low speed crystal (LXT) clock is stable and enabled.
 * |[2]     |PLL_STB   |Internal PLL Clock Source Stable Flag (Read Only)
 * |        |          |0 = Internal PLL clock is not stable or disabled.
 * |        |          |1 = Internal PLL clock is stable in normal mode.
 * |[3]     |OSC10K_STB|Internal 10 KHz Low Speed Oscillator (LIRC) Clock Source Stable Flag (Read Only)
 * |        |          |0 = Internal 10 kHz low speed oscillator clock (LIRC) is not stable or disabled.
 * |        |          |1 = Internal 10 kHz low speed oscillator clock (LIRC) is stable and enabled.
 * |[4]     |OSC22M_STB|Internal 22.1184 MHz High Speed Oscillator (HIRC) Clock Source Stable Flag (Read Only)
 * |        |          |0 = Internal 22.1184 MHz high speed oscillator (HIRC) clock is not stable or disabled.
 * |        |          |1 = Internal 22.1184 MHz high speed oscillator (HIRC) clock is stable and enabled.
 * |[5]     |OSC48M_STB|48 MHz Internal High Speed RC Oscillator (HIRC48) Clock Source Stable Flag (Read Only)
 * |        |          |0 = 48MHz internal high speed RC oscillator (HIRC48) clock is not stable or disabled.
 * |        |          |1 = 48MHz internal high speed RC oscillator (HIRC48) clock is stable and enabled.
 * |[7]     |CLK_SW_FAIL|Clock Switching Fail Flag (Read Only)
 * |        |          |0 = Clock switching success.
 * |        |          |1 = Clock switching failure.
 * |        |          |This bit is an index that if current system clock source is match as user defined at HCLK_S (CLKSEL[2:0]).
 * |        |          |When user switch system clock, the system clock source will keep old clock until the new clock is stable.
 * |        |          |During the period that waiting new clock stable, this bit will be an index shows system clock source is not match as user wanted.
 * @var CLK_T::CLKSEL0
 * Offset: 0x10  Clock Source Select Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |HCLK_S    |HCLK Clock Source Selection (Write Protect)
 * |        |          |1. Before clock switching, the related clock sources (both pre-select and new-select) must be turn on.
 * |        |          |2. The 3-bit default value is reloaded from the value of CFOSC (Config0[26:24]) in user configuration register of Flash controller by any reset.  
 * |        |          |Therefore the default value is either 000b or 111b.
 * |        |          |000 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
 * |        |          |001 = Clock source from external 32.768 kHz low speed crystal oscillator clock.
 * |        |          |010 = Clock source from PLL clock.
 * |        |          |011 = Clock source from internal 10 kHz low speed oscillator clock.
 * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |        |          |Note: These bits are write protected bit. Refer to the REGWRPROT register.
 * |[5:3]   |STCLK_S   |Cortex-M0 SysTick Clock Source Selection (Write Protect)
 * |        |          |If SYST_CSR[2] = 1, SysTick clock source is from HCLK.
 * |        |          |If SYST_CSR[2] = 0, SysTick clock source is defined by STCLK_S(CLKSEL0[5:3]).
 * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
 * |        |          |001 = Clock source from external 32.768 kHz low speed crystal clock.
 * |        |          |010 = Clock source from external 4~24 MHz high speed crystal clock/2.
 * |        |          |011 = Clock source from HCLK/2.
 * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock/2.
 * |        |          |Note1: These bits are write protected bit. Refer to the REGWRPROT register.
 * |        |          |Note2: if SysTick clock source is not from HCLK (i.e. SYST_CSR[2] = 0), SysTick clock source must less than or equal to HCLK/2.
 * |[8]     |USB_S     |USB Clock Source Selection (Write Protect)
 * |        |          |0 = Clock source from PLL clock.
 * |        |          |1 = Clock source from 48 MHz high speed RC oscillator clock.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register. 
 * @var CLK_T::CLKSEL1
 * Offset: 0x14  Clock Source Select Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |WDT_S     |Watchdog Timer Clock Source Select (Write Protect)
 * |        |          |00 = Reserved.
 * |        |          |01 = Clock source from external 32.768 kHz low speed crystal oscillator clock.
 * |        |          |10 = Clock source from HCLK/2048 clock.
 * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
 * |        |          |Note: These bits are write protected bit. Refer to the REGWRPROT register.
 * |[3:2]   |ADC_S     |ADC Clock Source Select
 * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
 * |        |          |01 = Clock source from PLL clock.
 * |        |          |10 = Clock source from HCLK.
 * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
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
 * |        |          |011 = Clock source from external trigger.
 * |        |          |101 = Clock source from internal 10 kHz low speed oscillator clock.
 * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |        |          |Others = Reserved.
 * |[14:12] |TMR1_S    |TIMER1 Clock Source Selection
 * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
 * |        |          |001 = Clock source from external 32.768 kHz low speed crystal clock.
 * |        |          |010 = Clock source from HCLK.
 * |        |          |011 = Clock source from external trigger.
 * |        |          |101 = Clock source from internal 10 kHz low speed oscillator clock.
 * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |        |          |Others = Reserved.
 * |[18:16] |TMR2_S    |TIMER2 Clock Source Selection
 * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
 * |        |          |001 = Clock source from external 32.768 kHz low speed crystal clock.
 * |        |          |010 = Clock source from HCLK.
 * |        |          |011 = Clock source from external trigger.
 * |        |          |101 = Clock source from internal 10 kHz low speed oscillator clock.
 * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |        |          |Others = Reserved.
 * |[22:20] |TMR3_S    |TIMER3 Clock Source Selection
 * |        |          |000 = Clock source from external 4~24 MHz high speed crystal clock.
 * |        |          |001 = Clock source from external 32.768 kHz low speed crystal clock.
 * |        |          |010 = Clock source from HCLK.
 * |        |          |011 = Clock source from external trigger.
 * |        |          |101 = Clock source from internal 10 kHz low speed oscillator clock.
 * |        |          |111 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |        |          |Others = Reserved.
 * |[25:24] |UART_S    |UART Clock Source Selection
 * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
 * |        |          |01 = Clock source from PLL clock.
 * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |[29:28] |PWM01_S   |PWM0 and PWM1 Clock Source Selection
 * |        |          |PWM0 and PWM1 used the same clock source; both of them used the same prescaler.
 * |        |          |The clock source of PWM0 and PWM1 is defined by PWM01_S (CLKSEL1[29:28]) and PWM01_S_E (CLKSEL2[8]).
 * |        |          |If PWM01_S_E = 0, the clock source of PWM0 and PWM1 defined by PWM01_S list below:
 * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
 * |        |          |01 = Clock source from external 32.768 kHz low speed crystal oscillator clock.
 * |        |          |10 = Clock source from HCLK.
 * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |        |          |If PWM01_S_E = 1, the clock source of PWM0 and PWM1 defined by PWM01_S list below:
 * |        |          |00 = Reserved.
 * |        |          |01 = Reserved.
 * |        |          |10 = Reserved.
 * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
 * |[31:30] |PWM23_S   |PWM2 And PWM3 Clock Source Selection
 * |        |          |PWM2 and PWM3 used the same clock source; both of them used the same prescaler.
 * |        |          |The clock source of PWM2 and PWM3 is defined by PWM23_S (CLKSEL1[31:30]) and PWM23_S_E (CLKSEL2[9]).
 * |        |          |If PWM23_S_E = 0, the clock source of PWM2 and PWM3 defined by PWM23_S list below:
 * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
 * |        |          |01 = Clock source from external 32.768 kHz low speed crystal oscillator clock.
 * |        |          |10 = Clock source from HCLK.
 * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |        |          |If PWM23_S_E = 1, the clock source of PWM2 and PWM3 defined by PWM23_S list below:
 * |        |          |00 = Reserved.
 * |        |          |01 = Reserved.
 * |        |          |10 = Reserved.
 * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
 * @var CLK_T::CLKDIV
 * Offset: 0x18  Clock Divider Number Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |HCLK_N    |HCLK Clock Divide Number from HCLK Clock Source
 * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLK_N + 1).
 * |[7:4]   |USB_N     |USB Clock Divide Number from USB Clock Source
 * |        |          |USB clock frequency = (USB clock source frequency) / (USB_N + 1).
 * |[11:8]  |UART_N    |UART Clock Divide Number from UART Clock Source
 * |        |          |UART clock frequency = (UART clock source frequency) / (UART_N + 1).
 * |[23:16] |ADC_N     |ADC Clock Divide Number from ADC Clock Source
 * |        |          |ADC clock frequency = (ADC clock source frequency) / (ADC_N + 1).
 * @var CLK_T::CLKSEL2
 * Offset: 0x1C  Clock Source Select Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:2]   |FRQDIV_S  |Clock Divider Clock Source Selection
 * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
 * |        |          |01 = Clock source from external 32.768 kHz low speed crystal oscillator clock.
 * |        |          |10 = Clock source from HCLK.
 * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |[5:4]   |PWM45_S   |PWM4 and PWM5 Clock Source Selection
 * |        |          |PWM4 and PWM5 used the same clock source; both of them used the same prescaler.
 * |        |          |The clock source of PWM4 and PWM5 is defined by PWM45_S (CLKSEL2[5:4]) and PWM45_S_E (CLKSEL2[10]).
 * |        |          |If PWM45_S_E = 0, the clock source of PWM4 and PWM5 defined by PWM45_S list below:
 * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
 * |        |          |01 = Clock source from external 32.768 kHz low speed crystal oscillator clock.
 * |        |          |10 = Clock source from HCLK.
 * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |        |          |If PWM45_S_E = 1, the clock source of PWM4 and PWM5 defined by PWM45_S list below:
 * |        |          |00 = Reserved.
 * |        |          |01 = Reserved.
 * |        |          |10 = Reserved.
 * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
 * |[8]     |PWM01_S_E |PWM0 And PWM1 Clock Source Selection Extend
 * |        |          |PWM0 and PWM1 used the same clock source; both of them used the same prescaler.
 * |        |          |The clock source of PWM0 and PWM1 is defined by PWM01_S (CLKSEL1[29:28]) and PWM01_S_E (CLKSEL2[8]).
 * |        |          |If PWM01_S_E = 0, the clock source of PWM0 and PWM1 defined by PWM01_S list below:
 * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
 * |        |          |01 = Clock source from external 32.768 kHz low speed crystal oscillator clock.
 * |        |          |10 = Clock source from HCLK.
 * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |        |          |If PWM01_S_E = 1, the clock source of PWM0 and PWM1 defined by PWM01_S list below:
 * |        |          |00 = Reserved.
 * |        |          |01 = Reserved.
 * |        |          |10 = Reserved.
 * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
 * |[9]     |PWM23_S_E |PWM2 And PWM3 Clock Source Selection Extend
 * |        |          |PWM2 and PWM3 used the same clock source; both of them used the same prescaler.
 * |        |          |The clock source of PWM2 and PWM3 is defined by PWM23_S (CLKSEL1[31:30]) and PWM23_S_E (CLKSEL2[9]).
 * |        |          |If PWM23_S_E = 0, the clock source of PWM2 and PWM3 defined by PWM23_S list below:
 * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
 * |        |          |01 = Clock source from external 32.768 kHz low speed crystal oscillator clock.
 * |        |          |10 = Clock source from HCLK.
 * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |        |          |If PWM23_S_E = 1, the clock source of PWM2 and PWM3 defined by PWM23_S listbelow:
 * |        |          |00 = Reserved.
 * |        |          |01 = Reserved.
 * |        |          |10 = Reserved.
 * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
 * |[10]    |PWM45_S_E |PWM4 And PWM5 Clock Source Selection Extend
 * |        |          |PWM4 and PWM5 used the same clock source; both of them used the same prescaler.
 * |        |          |The clock source of PWM4 and PWM5 is defined by PWM45_S (CLKSEL2[5:4]) and PWM45_S_E (CLKSEL2[10]).
 * |        |          |If PWM45_S_E = 0, the clock source of PWM4 and PWM5 defined by PWM45_S list below:
 * |        |          |00 = Clock source from external 4~24 MHz high speed crystal oscillator clock.
 * |        |          |01 = Clock source from external 32.768 kHz low speed crystal oscillator clock.
 * |        |          |10 = Clock source from HCLK.
 * |        |          |11 = Clock source from internal 22.1184 MHz high speed oscillator clock.
 * |        |          |If PWM45_S_E = 1, the clock source of PWM4 and PWM5 defined by PWM45_S list below:
 * |        |          |00 = Reserved.
 * |        |          |01 = Reserved.
 * |        |          |10 = Reserved.
 * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock. 
 * |[17:16] |WWDT_S    |Window Watchdog Timer Clock Source Selection
 * |        |          |10 = Clock source from HCLK/2048 clock.
 * |        |          |11 = Clock source from internal 10 kHz low speed oscillator clock.
 * |[18]    |RTC_SEL_10K|RTC Clock Source Selection
 * |        |          |0 = Clock source from external 32.768 kHz low speed crystal oscillator clock.
 * |        |          |1 = Clock source from internal 10 kHz low speed oscillator clock.
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
 * |[5]     |DIVIDER1  |Frequency Divider One Enable Bit
 * |        |          |0 = Frequency divider will output clock with source frequency divided by FSEL.
 * |        |          |1 = Frequency divider will output clock with source frequency.
 * |[6]     |CLKO_1HZ_EN|Clock Output 1Hz Enable Bit
 * |        |          |0 = 1 Hz clock output for 32K frequency compensation Disabled.
 * |        |          |1 = 1 Hz clock output for 32K frequency compensation Enabled.
 */

    __IO uint32_t PWRCON;        /* Offset: 0x00  System Power-down Control Register                                 */
    __IO uint32_t AHBCLK;        /* Offset: 0x04  AHB Devices Clock Enable Control Register                          */
    __IO uint32_t APBCLK;        /* Offset: 0x08  APB Devices Clock Enable Control Register                          */
    __IO uint32_t CLKSTATUS;     /* Offset: 0x0C  Clock status monitor Register                                      */
    __IO uint32_t CLKSEL0;       /* Offset: 0x10  Clock Source Select Control Register 0                             */
    __IO uint32_t CLKSEL1;       /* Offset: 0x14  Clock Source Select Control Register 1                             */
    __IO uint32_t CLKDIV;        /* Offset: 0x18  Clock Divider Number Register                                      */
    __IO uint32_t CLKSEL2;       /* Offset: 0x1C  Clock Source Select Control Register 2                             */
    __IO uint32_t PLLCON;        /* Offset: 0x20  PLL Control Register                                               */
    __IO uint32_t FRQDIV;        /* Offset: 0x24  Frequency Divider Control Register                                 */   

} CLK_T;


/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */


/* CLK PWRCON Bit Field Definitions */
#define CLK_PWRCON_OSC48M_EN_Pos             12                                   /*!< CLK_T::PWRCON: OSC48M_EN Position */
#define CLK_PWRCON_OSC48M_EN_Msk             (1ul << CLK_PWRCON_OSC48M_EN_Pos)    /*!< CLK_T::PWRCON: OSC48M_EN Mask */

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
#define CLK_AHBCLK_EBI_EN_Pos                3                                    /*!< CLK_T::AHBCLK: EBI_EN Position */
#define CLK_AHBCLK_EBI_EN_Msk                (1ul << CLK_AHBCLK_EBI_EN_Pos)       /*!< CLK_T::AHBCLK: EBI_EN Mask */

#define CLK_AHBCLK_ISP_EN_Pos                2                                    /*!< CLK_T::AHBCLK: ISP_EN Position */
#define CLK_AHBCLK_ISP_EN_Msk                (1ul << CLK_AHBCLK_ISP_EN_Pos)       /*!< CLK_T::AHBCLK: ISP_EN Mask */

#define CLK_AHBCLK_PDMA_EN_Pos               1                                    /*!< CLK_T::AHBCLK: PDMA_EN Position */
#define CLK_AHBCLK_PDMA_EN_Msk               (1ul << CLK_AHBCLK_PDMA_EN_Pos)      /*!< CLK_T::AHBCLK: PDMA_EN Mask */

/* CLK APBCLK Bit Field Definitions */
#define CLK_APBCLK_ADC_EN_Pos                28                                   /*!< CLK_T::APBCLK: ADC_EN Position */
#define CLK_APBCLK_ADC_EN_Msk                (1ul << CLK_APBCLK_ADC_EN_Pos)       /*!< CLK_T::APBCLK: ADC_EN Mask */

#define CLK_APBCLK_USBD_EN_Pos               27                                   /*!< CLK_T::APBCLK: USBD_EN Position */
#define CLK_APBCLK_USBD_EN_Msk               (1ul << CLK_APBCLK_USBD_EN_Pos)      /*!< CLK_T::APBCLK: USBD_EN Mask */

#define CLK_APBCLK_PWM45_EN_Pos              22                                   /*!< CLK_T::APBCLK: PWM45_EN Position */
#define CLK_APBCLK_PWM45_EN_Msk              (1ul << CLK_APBCLK_PWM45_EN_Pos)     /*!< CLK_T::APBCLK: PWM45_EN Mask */

#define CLK_APBCLK_PWM23_EN_Pos              21                                   /*!< CLK_T::APBCLK: PWM23_EN Position */
#define CLK_APBCLK_PWM23_EN_Msk              (1ul << CLK_APBCLK_PWM23_EN_Pos)     /*!< CLK_T::APBCLK: PWM23_EN Mask */

#define CLK_APBCLK_PWM01_EN_Pos              20                                   /*!< CLK_T::APBCLK: PWM01_EN Position */
#define CLK_APBCLK_PWM01_EN_Msk              (1ul << CLK_APBCLK_PWM01_EN_Pos)     /*!< CLK_T::APBCLK: PWM01_EN Mask */

#define CLK_APBCLK_UART2_EN_Pos              18                                   /*!< CLK_T::APBCLK: UART2_EN Position */
#define CLK_APBCLK_UART2_EN_Msk              (1ul << CLK_APBCLK_UART2_EN_Pos)     /*!< CLK_T::APBCLK: UART2_EN Mask */

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

#define CLK_APBCLK_RTC_EN_Pos                1                                    /*!< CLK_T::APBCLK: RTC_EN Position */
#define CLK_APBCLK_RTC_EN_Msk                (1ul << CLK_APBCLK_RTC_EN_Pos)       /*!< CLK_T::APBCLK: RTC_EN Mask */

#define CLK_APBCLK_WDT_EN_Pos                0                                    /*!< CLK_T::APBCLK: WDT_EN Position */
#define CLK_APBCLK_WDT_EN_Msk                (1ul << CLK_APBCLK_WDT_EN_Pos)       /*!< CLK_T::APBCLK: WDT_EN Mask */

/* CLK CLKSTATUS Bit Field Definitions */
#define CLK_CLKSTATUS_CLK_SW_FAIL_Pos        7                                        /*!< CLK_T::CLKSTATUS: CLK_SW_FAIL Position */
#define CLK_CLKSTATUS_CLK_SW_FAIL_Msk        (1ul << CLK_CLKSTATUS_CLK_SW_FAIL_Pos)   /*!< CLK_T::CLKSTATUS: CLK_SW_FAIL Mask */

#define CLK_CLKSTATUS_OSC48M_STB_Pos         5                                        /*!< CLK_T::CLKSTATUS: OSC48M_STB Position */
#define CLK_CLKSTATUS_OSC48M_STB_Msk         (1ul << CLK_CLKSTATUS_OSC48M_STB_Pos)    /*!< CLK_T::CLKSTATUS: OSC48M_STB Mask */

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
#define CLK_CLKSEL0_USB_S_Pos                8                                        /*!< CLK_T::CLKSEL0: USB_S Position */
#define CLK_CLKSEL0_USB_S_Msk                (1ul << CLK_CLKSEL0_USB_S_Pos)           /*!< CLK_T::CLKSEL0: USB_S Mask */

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

#define CLK_CLKSEL1_SPI3_S_Pos               7                                        /*!< CLK_T::CLKSEL1: SPI3_S Position */
#define CLK_CLKSEL1_SPI3_S_Msk               (1ul << CLK_CLKSEL1_SPI3_S_Pos)          /*!< CLK_T::CLKSEL1: SPI3_S Mask */

#define CLK_CLKSEL1_SPI2_S_Pos               6                                        /*!< CLK_T::CLKSEL1: SPI2_S Position */
#define CLK_CLKSEL1_SPI2_S_Msk               (1ul << CLK_CLKSEL1_SPI2_S_Pos)          /*!< CLK_T::CLKSEL1: SPI2_S Mask */

#define CLK_CLKSEL1_SPI1_S_Pos               5                                        /*!< CLK_T::CLKSEL1: SPI1_S Position */
#define CLK_CLKSEL1_SPI1_S_Msk               (1ul << CLK_CLKSEL1_SPI1_S_Pos)          /*!< CLK_T::CLKSEL1: SPI1_S Mask */

#define CLK_CLKSEL1_SPI0_S_Pos               4                                        /*!< CLK_T::CLKSEL1: SPI0_S Position */
#define CLK_CLKSEL1_SPI0_S_Msk               (1ul << CLK_CLKSEL1_SPI0_S_Pos)          /*!< CLK_T::CLKSEL1: SPI0_S Mask */

#define CLK_CLKSEL1_ADC_S_Pos                2                                        /*!< CLK_T::CLKSEL1: ADC_S Position */
#define CLK_CLKSEL1_ADC_S_Msk                (3ul << CLK_CLKSEL1_ADC_S_Pos)           /*!< CLK_T::CLKSEL1: ADC_S Mask */

#define CLK_CLKSEL1_WDT_S_Pos                0                                        /*!< CLK_T::CLKSEL1: WDT_S Position */
#define CLK_CLKSEL1_WDT_S_Msk                (3ul << CLK_CLKSEL1_WDT_S_Pos)           /*!< CLK_T::CLKSEL1: WDT_S Mask */

/* CLK CLKSEL2 Bit Field Definitions */
#define CLK_CLKSEL2_RTC_SEL_10K_Pos          18                                       /*!< CLK_T::CLKSEL2: RTC_SEL_10K Position */
#define CLK_CLKSEL2_RTC_SEL_10K_Msk          (1ul << CLK_CLKSEL2_RTC_SEL_10K_Pos)     /*!< CLK_T::CLKSEL2: RTC_SEL_10K Mask */

#define CLK_CLKSEL2_WWDT_S_Pos               16                                       /*!< CLK_T::CLKSEL2: WWDT_S Position */
#define CLK_CLKSEL2_WWDT_S_Msk               (3ul << CLK_CLKSEL2_WWDT_S_Pos)          /*!< CLK_T::CLKSEL2: WWDT_S Mask */

#define CLK_CLKSEL2_PWM45_S_E_Pos            10                                       /*!< CLK_T::CLKSEL2: PWM45_S_E Position */
#define CLK_CLKSEL2_PWM45_S_E_Msk            (1ul << CLK_CLKSEL2_PWM45_S_E_Pos)       /*!< CLK_T::CLKSEL2: PWM45_S_E Mask */
#define CLK_CLKSEL2_PWM45_S_EXT_Pos          10                                       /*!< CLK_T::CLKSEL2: PWM45_S_EXT Position */
#define CLK_CLKSEL2_PWM45_S_EXT_Msk          (1ul << CLK_CLKSEL2_PWM45_S_EXT_Pos)     /*!< CLK_T::CLKSEL2: PWM45_S_EXT Mask */

#define CLK_CLKSEL2_PWM23_S_E_Pos            9                                        /*!< CLK_T::CLKSEL2: PWM23_S_E Position */
#define CLK_CLKSEL2_PWM23_S_E_Msk            (1ul << CLK_CLKSEL2_PWM23_S_E_Pos)       /*!< CLK_T::CLKSEL2: PWM23_S_E Mask */
#define CLK_CLKSEL2_PWM23_S_EXT_Pos          9                                        /*!< CLK_T::CLKSEL2: PWM23_S_EXT Position */
#define CLK_CLKSEL2_PWM23_S_EXT_Msk          (1ul << CLK_CLKSEL2_PWM23_S_EXT_Pos)     /*!< CLK_T::CLKSEL2: PWM23_S_EXT Mask */

#define CLK_CLKSEL2_PWM01_S_E_Pos            8                                        /*!< CLK_T::CLKSEL2: PWM01_S_E Position */
#define CLK_CLKSEL2_PWM01_S_E_Msk            (1ul << CLK_CLKSEL2_PWM01_S_E_Pos)       /*!< CLK_T::CLKSEL2: PWM01_S_E Mask */
#define CLK_CLKSEL2_PWM01_S_EXT_Pos          8                                        /*!< CLK_T::CLKSEL2: PWM01_S_EXT Position */
#define CLK_CLKSEL2_PWM01_S_EXT_Msk          (1ul << CLK_CLKSEL2_PWM01_S_EXT_Pos)     /*!< CLK_T::CLKSEL2: PWM01_S_EXT Mask */

#define CLK_CLKSEL2_PWM45_S_Pos              4                                        /*!< CLK_T::CLKSEL2: PWM45_S Position */
#define CLK_CLKSEL2_PWM45_S_Msk              (3ul << CLK_CLKSEL2_PWM45_S_Pos)         /*!< CLK_T::CLKSEL2: PWM45_S Mask */

#define CLK_CLKSEL2_FRQDIV_S_Pos             2                                        /*!< CLK_T::CLKSEL2: FRQDIV_S Position */
#define CLK_CLKSEL2_FRQDIV_S_Msk             (3ul << CLK_CLKSEL2_FRQDIV_S_Pos)        /*!< CLK_T::CLKSEL2: FRQDIV_S Mask */

/* CLK CLKDIV Bit Field Definitions */
#define CLK_CLKDIV_ADC_N_Pos                 16                                       /*!< CLK_T::CLKDIV: ADC_N Position */
#define CLK_CLKDIV_ADC_N_Msk                 (0xFFul << CLK_CLKDIV_ADC_N_Pos)         /*!< CLK_T::CLKDIV: ADC_N Mask */

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

/* CLK FRQDIV Bit Field Definitions */
#define CLK_FRQDIV_CLKO_1HZ_EN_Pos           6                                        /*!< CLK_T::FRQDIV: CLKO_1HZ_EN Position */
#define CLK_FRQDIV_CLKO_1HZ_EN_Msk           (1ul << CLK_FRQDIV_CLKO_1HZ_EN_Pos)      /*!< CLK_T::FRQDIV: CLKO_1HZ_EN Mask */

#define CLK_FRQDIV_DIVIDER1_Pos              5                                        /*!< CLK_T::FRQDIV: DIVIDER1 Position */
#define CLK_FRQDIV_DIVIDER1_Msk              (1ul << CLK_FRQDIV_DIVIDER1_Pos)         /*!< CLK_T::FRQDIV: DIVIDER1 Mask */

#define CLK_FRQDIV_DIVIDER_EN_Pos            4                                        /*!< CLK_T::FRQDIV: DIVIDER_EN Position */
#define CLK_FRQDIV_DIVIDER_EN_Msk            (1ul << CLK_FRQDIV_DIVIDER_EN_Pos)       /*!< CLK_T::FRQDIV: DIVIDER_EN Mask */

#define CLK_FRQDIV_FSEL_Pos                  0                                        /*!< CLK_T::FRQDIV: FRQDIV_FSEL Position */
#define CLK_FRQDIV_FSEL_Msk                  (0xFul << CLK_FRQDIV_FSEL_Pos)           /*!< CLK_T::FRQDIV: FRQDIV_FSEL Mask */
/*@}*/ /* end of group CLK_CONST */
/*@}*/ /* end of group CLK */
/**@}*/ /* end of REGISTER group */

#endif /* __CLK_REG_H__ */


