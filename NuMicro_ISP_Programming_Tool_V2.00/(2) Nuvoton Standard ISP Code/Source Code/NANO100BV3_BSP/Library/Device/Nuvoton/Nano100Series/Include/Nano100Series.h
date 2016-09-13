
/**************************************************************************//**
 * @file     Nano100Series.h
 * @version  V1.00
 * $Revision: 81 $
 * $Date: 15/08/07 10:20a $
 * @brief    Nano100 series peripheral access layer header file.
 *           This file contains all the peripheral register's definitions,
 *           bits definitions and memory mapping for NuMicro Nano100 series MCU.
 *
 * @note
 * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/**
   \mainpage NuMicro NANO100BN Driver Reference Guide
   *
   * <b>Introduction</b>
   *
   * This user manual describes the usage of Nano100BN Series MCU device driver
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
   * Copyright (C) 2014~2015 Nuvoton Technology Corp. All rights reserved.
   */
/**
  * \page pg1 NuMicro NANO100 BSP Directory Structure
  * Please refer to Readme.pdf under BSP root directory for the BSP directory structure
  *
  * \page pg2 Revision History
  *
  * <b>Revision 3.02.000</b>
  * \li Removed FMC driver¡¦s FMC_SetBootSource(),FMC_DisableAPUpdate(),FMC_DisableConfigUpdate(),FMC_DisableLDUpdate(),FMC_EnableAPUpdate(),FMC_EnableConfigUpdate(),FMC_EnableLDUpdate() in fmc.h, because there exist functionally identical macros.
  * \li Removed SYS_IRCTRIMCTL_SEL_MASK, SYS_IRCTRIMCTL_LOOP_MASK and SYS_IRCTRIMCTL_RETRY_COUNT in sys.h.
  * \li Removed SPI_ENABLE_DUAL_MODE() in spi.h. 
  * \li Modified SPI_ENABLE_DUAL_INPUT_MODE() and SPI_ENABLE_DUAL_OUTPUT_MODE() to enable dual I/O with direction.
  * \li Modified USBD driver to pass USB Command Verify test in usbd.c and usbd.h.
  * \li Modified UART_SelectIrDAMode() to reload UART clock before calculating baudrate, in uart.c.
  * \li Modified SD card clock speed from 24 MHz to 5 MHz to make SPI operations stable in SDCard.c.
  * \li Modified MMC_FLASH_Init() to retry SD CMD0 command until success in SDCard.c.
  * \li Modified TIMER_Open() to not start timer in timer.c.
  * \li Updated TIMER_Open() and TIMER_Delay() to support extreme high clock input, in timer.c.
  * \li Renamed sample CRC to CRC_CCITT in StdDriver.
  * \li Renamed sample GPIO to GPIO_IOTest in StdDriver.
  * \li Renamed sample PDMA to PDMA_Memory in StdDriver.
  * \li Renamed sample SYS to SYS_Control in StdDriver.
  * \li Renamed SYS_Int_xxx_Msk to SYS_xxx_Msk in sys.h.
  * \li Renamed GP_DBNCECON_PUEN_* to GP_DBNCECON_DBCLKSEL_*  in Nano100Series.h.
  * \li Renamed SYS_IRCTRIMINT_32KERR_ENNT to SYS_IRCTRIMINT_32KERR_INT in sys.h.
  * \li Fixed the bug that RTC_AER enable flow may be interrupted by interrupt service routine in rtc.c.
  * \li Fixed the bug that PWM_ENABLE_OUTPUT_INVERTER() does not clear register field before writing input parameter to it in pwm.h.
  * \li Fixed the bug that TIMER_Delay() sets prescale to wrong register in timer.c.
  * \li Fixed SCUART_Open() and SCUART_SetLineConfig() baudrate calculation prescale setting error in scuart.c.
  * \li Fixed bugs of SPI_EnableAutoSS () and SPI_SetBusClock () in spi.c, and cleared bit mask of register field before writing input parameter to it.
  * \li Fixed the CLK_SysTickDelay() bug that continuously calling CLK_SysTickDelay() may imply an incorrect delay time by clearing control register on each call in clk.c.
  * \li Fixed implementation errors of CLK_PLLCTL_FB_DV_Msk and CLK_APBCLK_I2C0_EN in Nano100Series.h.
  * \li Fixed SYS_CLEAR_RST_SOURCE  implementation error in sys.h.
  * \li Fixed CLK_WK_INTSTS_IS implementation error in clk.h.
  * \li Fixed "GPIO_DISABLE_DOUT_MASK" and "GPIO_ENABLE_DOUT_MASK" implementation errors in gpio.h.
  * \li Fixed SC_SET_STOP_BIT_LEN implementation error in sc.h.
  * \li Fixed PDMA_IS_CH_BUSY implementation error in pdma.h.
  * \li Fixed LCD_CPUMP_DIV128 implementation error in lcd.h.
  * \li Fixed RTC_CLEAR_TAMPER_FLAG() implementation bug in rtc.h.
  * \li Fixed ADC_SET_DMOF() implementation error in adc.h.
  * \li Fixed CRC_SET_SEED ()implementation error in crc.h.
  * \li Disabled Rx before raising RST high during cold reset in SmartCardLib library.
  * \li Checked SC_RST and SC_DAT_O pin status during deactivation in SmartCardLib library.
  * \li Added SYS_PA_H_MFP_PA9_MFP_LCD_S7 macro in sys.h.
  * \li Added I2C_ClearIntFlag() and I2S_SetFIFO() functions in i2c.c.
  * \li Added bit definitions of MCLKO (Module Clock CKO) register in clk.h.
  * \li Added LCD_MODULE and DAC_MODULE macro sets in clk.c.
  * \li Added CLK_EnableSysTick() and CLK_DisableSysTick() in clk.c.
  * \li Added macros CLK_PLLCTL_*MHz_HXT and CLK_PLL_*MHz_HIRC for setting PLLCTL value in clk.c.
  * \li Added SYS_EnableIRCTrim() and SYS_DisableIRCTrim() functions in sys.c, and added macros SYS_GET_IRCTRIM_INT_FLAG() and SYS_CLEAR_IRCTRIM_INT_FLAG() in sys.h.
  * \li Added UART_SelectLINMode() in uart.c, and added UART_FUNC_SEL_LIN in uart.h.
  * \li Added a sample USBD_Audio_Speaker_And_HID_Transfer to Nu-LB-NANO130.
  * \li Added samples SYS_MCLKO, SYS_PLLClockOutput, and SYS_TrimIRC to StdDriver.
  * \li Added a sample Timer_Wakeup to StdDriver.
  * \li Added samples USBD_HID_Keyboard, USBD_HID_MouseKeyboard, USBD_HID_Touch, USBD_HID_Transfer_And_Keyboard, USBD_HID_Transfer_And_MSC, USBD_Mass_Storage_CDROM, USBD_Micro_Printer, USBD_Printer_And_HID_Transfer, USBD_VCOM_And_HID_Keyboard, USBD_VCOM_And_HID_Transfer, USBD_VCOM_And_Mass_Storage, USBD_VCOM_DualPort, and USBD_VCOM_SerialEmulator to StdDriver.
  *
  * <b>Revision 3.01.000</b>
  * \li Renamed register TESTCLK to MCLKO.
  * \li Renamed registers PDSSR0 and PDSSR1 to DSSR0 and DSSR1.
  * \li Renamed UBSD_ENABLE_INT() to USBD_ENABLE_INT().
  * \li Renamed I2S_Enable_MCLK() and I2S_Disable_MCLK() to I2S_EnableMCLK() and I2S_DisableMCLK().
  * \li Renamed CLK_CLKSEL1_PWM1_CH01_S_Msk and CLK_CLKSEL1_PWM1_CH23_S_Msk to CLK_CLKSEL2_PWM1_CH01_S_Msk and CLK_CLKSEL2_PWM1_CH23_S_Msk.
  * \li Renamed RTC_RIIR_SNOOPIS_Msk to RTC_RIIR_SNOOPIF_Msk.
  * \li Renamed PDMA_IER_BLKD_IE_Msk to PDMA_IER_TD_IE_Msk.
  * \li Modified PWM_EnablePDMA() function prototype, and added one more parameter to select captured edge.
  * \li Modified PWM capture interrupt flag relative macro definitions to improve performance.
  * \li Added ADC clock source bit position and mask definition.
  * \li Added ADC_SET_REF_VOLTAGE() macro and RES/REF definitions.
  * \li Added DAC driver.
  * \li Added ADC_Compare, ADC_TimerTrigger, ADC_PDMA, DAC_PDMATrigger, DAC_SoftwareTrigger, DAC_TimerTrigger, GPIO_PowerDown, Hard_Fault_Sample,
  *     PWM_CapturePDMA, SPI_LoopbackPDMA, UART_FlowCtrl, UART_RxWakeup, UART_TxRxPDMA, and USBD_HID_Transfer samples.
  *
  * <b>Revision 3.00.000</b>
  * \li Update major version number from 2 to 3.
  * \li Renamed RTC_GetDatAndTime() to RTC_GetDateAndTime().
  *
  * <b>Revision 2.00.000</b>
  * \li Initial release.
*/
#ifndef __NANO100SERIES_H__
#define __NANO100SERIES_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup NANO100_Definitions NANO100 Definitions
  This file defines all structures and symbols for Nano100:
    - interrupt numbers
    - registers and bit fields
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/

/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup NANO100_CMSIS Device CMSIS Definitions
  Configuration of the Cortex-M0 Processor and Core Peripherals
  @{
*/

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */
typedef enum IRQn {
    /******  Cortex-M0 Processor Exceptions Numbers *****************************************/

    NonMaskableInt_IRQn   = -14,    /*!< 2 Non Maskable Interrupt                           */
    HardFault_IRQn        = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                   */
    SVCall_IRQn           = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                     */
    PendSV_IRQn           = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                     */
    SysTick_IRQn          = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                 */

    /******  Nano100 specific Interrupt Numbers ***********************************************/
    BOD_IRQn              = 0,      /*!< Brownout low voltage detected interrupt                   */
    WDT_IRQn              = 1,      /*!< Watch Dog Timer interrupt                                 */
    EINT0_IRQn            = 2,      /*!< External signal interrupt from PB.14 pin                  */
    EINT1_IRQn            = 3,      /*!< External signal interrupt from PB.15 pin                  */
    GPABC_IRQn            = 4,      /*!< External signal interrupt from PA[15:0]/PB[13:0]/PC[15:0] */
    GPDEF_IRQn            = 5,      /*!< External interrupt from PD[15:0]/PE[15:0]/PF[15:0]        */
    PWM0_IRQn             = 6,      /*!< PWM 0 interrupt                                           */
    PWM1_IRQn             = 7,      /*!< PWM 1 interrupt                                           */
    TMR0_IRQn             = 8,      /*!< Timer 0 interrupt                                         */
    TMR1_IRQn             = 9,      /*!< Timer 1 interrupt                                         */
    TMR2_IRQn             = 10,     /*!< Timer 2 interrupt                                         */
    TMR3_IRQn             = 11,     /*!< Timer 3 interrupt                                         */
    UART0_IRQn            = 12,     /*!< UART0 interrupt                                           */
    UART1_IRQn            = 13,     /*!< UART1 interrupt                                           */
    SPI0_IRQn             = 14,     /*!< SPI0 interrupt                                            */
    SPI1_IRQn             = 15,     /*!< SPI1 interrupt                                            */
    SPI2_IRQn             = 16,     /*!< SPI2 interrupt                                            */
    HIRC_IRQn             = 17,     /*!< HIRC interrupt                                            */
    I2C0_IRQn             = 18,     /*!< I2C0 interrupt                                            */
    I2C1_IRQn             = 19,     /*!< I2C1 interrupt                                            */
    SC2_IRQn              = 20,     /*!< Smart Card 2 interrupt                                    */
    SC0_IRQn              = 21,     /*!< Smart Card 0 interrupt                                    */
    SC1_IRQn              = 22,     /*!< Smart Card 1 interrupt                                    */
    USBD_IRQn             = 23,     /*!< USB FS Device interrupt                                   */
    LCD_IRQn              = 25,     /*!< LCD interrupt                                             */
    PDMA_IRQn             = 26,     /*!< PDMA interrupt                                            */
    I2S_IRQn              = 27,     /*!< I2S interrupt                                             */
    PDWU_IRQn             = 28,     /*!< Power Down Wake up interrupt                              */
    ADC_IRQn              = 29,     /*!< ADC interrupt                                             */
    DAC_IRQn              = 30,     /*!< DAC interrupt                                             */
    RTC_IRQn              = 31      /*!< Real time clock interrupt                                 */
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

/*@}*/ /* end of group NANO100_CMSIS */


#include "core_cm0.h"                       /* Cortex-M0 processor and core peripherals           */
#include "system_Nano100Series.h"           /* Nano100 Series System include file                  */
#include <stdint.h>

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup NANO100_Peripherals NANO100 Peripherals
  NANO100 Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM  )
#pragma anon_unions
#endif



/*---------------------- Analog to Digital Converter -------------------------*/
/**
    @addtogroup ADC Analog to Digital Converter(ADC)
    Memory Mapped Structure for ADC Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RESULT0, RESULT1.. RESULT17</font><br><p> <font size="2">
Offset: 0x00 ~0x44 A/D Data Register 0~17</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[11:0]</td><td>RSLT</td><td><div style="word-wrap: break-word;"><b>A/D Conversion Result</b><br>
This field contains 12 bits conversion results.<br>
</div></td></tr><tr><td>
[16]</td><td>VALID</td><td><div style="word-wrap: break-word;"><b>Data Valid Flag</b><br>
It is a mirror of VALID bit in ADC_RESULTx<br>
</div></td></tr><tr><td>
[17]</td><td>OVERRUN</td><td><div style="word-wrap: break-word;"><b>Over Run Flag</b><br>
It is a mirror to OVERRUN bit in ADC_RESULTx<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t RESULT[18];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CR</font><br><p> <font size="2">
Offset: 0x48  A/D Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>ADEN</td><td><div style="word-wrap: break-word;"><b>A/D Converter Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
Before starting A/D conversion, this bit should be set to 1.<br>
Clear it to 0 to disable A/D converter analog circuit power consumption.<br>
</div></td></tr><tr><td>
[1]</td><td>ADIE</td><td><div style="word-wrap: break-word;"><b>A/D Interrupt Enable</b><br>
0 = A/D interrupt function Disabled.<br>
1 = A/D interrupt function Enabled.<br>
A/D conversion end interrupt request is generated if ADIE bit is set to 1.<br>
</div></td></tr><tr><td>
[3:2]</td><td>ADMD</td><td><div style="word-wrap: break-word;"><b>A/D Converter Operation Mode</b><br>
00 = Single conversion<br>
01 = Reserved<br>
10 = Single-cycle scan<br>
11 = Continuous scan<br>
</div></td></tr><tr><td>
[5:4]</td><td>TRGS</td><td><div style="word-wrap: break-word;"><b>Hardware Trigger Source</b><br>
This field must keep 00<br>
Software should disable TRGE and ADST before change TRGS.<br>
In hardware trigger mode, the ADST bit is set by the external trigger from STADC, However software has the highest priority to set or cleared ADST bit at any time.<br>
</div></td></tr><tr><td>
[7:6]</td><td>TRGCOND</td><td><div style="word-wrap: break-word;"><b>External Trigger Condition</b><br>
These two bits decide external pin STADC trigger event is level or edge.<br>
The signal must be kept at stable state at least 8 PCLKs for level trigger and 4 PCLKs at high and low state.<br>
00 = Low level<br>
01 = High level<br>
10 = Falling edge<br>
11 = Rising edge<br>
</div></td></tr><tr><td>
[8]</td><td>TRGE</td><td><div style="word-wrap: break-word;"><b>External Trigger Enable</b><br>
Enable or disable triggering of A/D conversion by external STADC pin.<br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[9]</td><td>PTEN</td><td><div style="word-wrap: break-word;"><b>PDMA Transfer Enable</b><br>
0 = PDMA data transfer Disabled.<br>
1 = PDMA data transfer in ADC_RESULT 0~17 Enabled.<br>
When A/D conversion is completed, the converted data is loaded into ADC_RESULT 0~10, software can enable this bit to generate a PDMA data transfer request.<br>
When PTEN=1, software must set ADIE=0 to disable interrupt.<br>
PDMA can access ADC_RESULT 0-17 registers by block or single transfer mode.<br>
</div></td></tr><tr><td>
[10]</td><td>DIFF</td><td><div style="word-wrap: break-word;"><b>Differential Mode Selection</b><br>
0 = ADC is operated in single-ended mode.<br>
1 = ADC is operated in differential mode.<br>
The A/D analog input ADC_CH0/ADC_CH1 consists of a differential pair.<br>
So as ADC_CH2/ADC_CH3, ADC_CH4/ADC_CH5, ADC_CH6/ADC_CH7, ADC_CH8/ADC_CH9 and ADC_CH10/ADC_CH11.<br>
The even channel defines as plus analog input voltage (Vplus) and the odd channel defines as minus analog input voltage (Vminus).<br>
Differential input voltage (Vdiff) = Vplus - Vminus, where Vplus is the analog input; Vminus is the inverted analog input.<br>
In differential input mode, only the even number of the two corresponding channels needs to be enabled in CHEN (ADCHER[11:0]).<br>
The conversion result will be placed to the corresponding data register of the enabled channel.<br>
Note: Calibration should calibrated each time when switching between single-ended and differential mode<br>
</div></td></tr><tr><td>
[11]</td><td>ADST</td><td><div style="word-wrap: break-word;"><b>A/D Conversion Start</b><br>
0 = Conversion stopped and A/D converter enter idle state.<br>
1 = Conversion starts.<br>
ADST bit can be set to 1 from two sources: software write and external pin STADC.<br>
ADST is cleared to 0 by hardware automatically at the end of single mode and single-cycle scan mode on specified channels.<br>
In continuous scan mode, A/D conversion is continuously performed sequentially unless software writes 0 to this bit or chip reset.<br>
Note: After ADC conversion done, SW needs to wait at least one ADC clock before to set this bit high again.<br>
</div></td></tr><tr><td>
[13:12]</td><td>TMSEL</td><td><div style="word-wrap: break-word;"><b>Select A/D Enable Time-Out Source</b><br>
00 = TMR0<br>
01 = TMR1<br>
10 = TMR2<br>
11 = TMR3<br>
</div></td></tr><tr><td>
[15]</td><td>TMTRGMOD</td><td><div style="word-wrap: break-word;"><b>Timer Event Trigger ADC Conversion</b><br>
0 = This function Disabled.<br>
1 = ADC Enabled by TIMER OUT event. Setting TMSEL to select timer event from timer0~3<br>
</div></td></tr><tr><td>
[17:16]</td><td>REFSEL</td><td><div style="word-wrap: break-word;"><b>Reference Voltage Source Selection</b><br>
00 = Reserved<br>
01 = Select Int_VREF as reference voltage<br>
10 = Select VREF as reference voltage<br>
11 = Reserved<br>
</div></td></tr><tr><td>
[19:18]</td><td>RESSEL</td><td><div style="word-wrap: break-word;"><b>Resolution Selection</b><br>
00 = 6 bits<br>
01 = 8 bits<br>
10 = 10 bits<br>
11 = 12 bits<br>
</div></td></tr><tr><td>
[31:24]</td><td>TMPDMACNT</td><td><div style="word-wrap: break-word;"><b>PDMA Count</b><br>
When each timer event occur PDMA will transfer TMPDMACNT +1 ADC result in the amount of this register setting<br>
Note: The total amount of PDMA transferring data should be set in PDMA byte count register.<br>
When PDMA finish is set, ADC will not be enabled and start transfer even though the timer event occurred.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CHEN</font><br><p> <font size="2">
Offset: 0x4C  A/D Channel Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>CHEN0</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 0 Enable (Convert Input Voltage From PA.0 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
If more than one channel in single mode is enabled by software, the least channel is converted and other enabled channels will be ignored.<br>
</div></td></tr><tr><td>
[1]</td><td>CHEN1</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 1 Enable(Convert Input Voltage From PA.1 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>CHEN2</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 2 Enable (Convert Input Voltage From PA.2 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[3]</td><td>CHEN3</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 3 Enable(Convert Input Voltage From PA.3 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[4]</td><td>CHEN4</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 4 Enable (Convert Input Voltage From PA.4 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[5]</td><td>CHEN5</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 5 Enable (Convert Input Voltage From PA.5 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[6]</td><td>CHEN6</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 6 Enable (Convert Input Voltage From PA.6 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[7]</td><td>CHEN7</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 7 Enable (Convert Input Voltage From PA.7 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[8]</td><td>CHEN8</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 8 Enable For DAC0 (Convert Input Voltage From PD.0 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[9]</td><td>CHEN9</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 9 Enable For DAC1 (Convert Input Voltage From PD.1 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[10]</td><td>CHEN10</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 10 Enable (Convert Input Voltage From PD.2 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[11]</td><td>CHEN11</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 11 Enable(Convert Input Voltage From PD.3 )</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[12]</td><td>CHEN12</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 12 Enable (Convert DAC0 Output Voltage)</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[13]</td><td>CHEN13</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 13 Enable (Convert DAC1 Output Voltage)</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[14]</td><td>CHEN14</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 14 Enable (Convert VTEMP)</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[15]</td><td>CHEN15</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 15 Enable (Convert Int_VREF)</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[16]</td><td>CHEN16</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 16 Enable (Convert AVDD)</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[17]</td><td>CHEN17</td><td><div style="word-wrap: break-word;"><b>Analog Input Channel 17 Enable (Convert AVSS)</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CHEN;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CMPR0</font><br><p> <font size="2">
Offset: 0x50  A/D Compare Register 0</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>CMPEN</td><td><div style="word-wrap: break-word;"><b>Compare Enable</b><br>
0 = Compare Disabled.<br>
1 = Compare Enabled.<br>
Set this bit to 1 to enable compare CMPD[11:0] with specified channel conversion result when converted data is loaded into ADC_RESULTx register.<br>
When this bit is set to 1, and CMPMATCNT is 0, the CMPF will be set once the match is hit<br>
</div></td></tr><tr><td>
[1]</td><td>CMPIE</td><td><div style="word-wrap: break-word;"><b>Compare Interrupt Enable</b><br>
0 = Compare function interrupt Disabled.<br>
1 = Compare function interrupt Enabled.<br>
If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMATCNT, CMPF bit will be asserted, in the meanwhile, if CMPIE is set to 1, a compare interrupt request is generated.<br>
</div></td></tr><tr><td>
[2]</td><td>CMPCOND</td><td><div style="word-wrap: break-word;"><b>Compare Condition</b><br>
0 = Set the compare condition as that when a 12-bit A/D conversion result is less than the 12-bit CMPD (ADCMPRx[27:16]), the internal match counter will increase one.<br>
1 = Set the compare condition as that when a 12-bit A/D conversion result is greater or equal to the 12-bit CMPD (ADCMPRx[27:16]), the internal match counter will increase by one.<br>
Note: When the internal counter reaches the value to (CMPMATCNT +1), the CMPF bit will be set.<br>
</div></td></tr><tr><td>
[7:3]</td><td>CMPCH</td><td><div style="word-wrap: break-word;"><b>Compare Channel Selection</b><br>
This field selects the channel whose conversion result is selected to be compared.<br>
</div></td></tr><tr><td>
[11:8]</td><td>CMPMATCNT</td><td><div style="word-wrap: break-word;"><b>Compare Match Count</b><br>
When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND[2], the internal match counter will increase 1.<br>
When the internal counter reaches the value to (CMPMATCNT +1), the CMPF bit will be set.<br>
</div></td></tr><tr><td>
[27:16]</td><td>CMPD</td><td><div style="word-wrap: break-word;"><b>Comparison Data</b><br>
The 12 bits data is used to compare with conversion result of specified channel.<br>
Software can use it to monitor the external analog input pin voltage variation in scan mode without imposing a load on software.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CMPR0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CMPR1</font><br><p> <font size="2">
Offset: 0x54  A/D Compare Register 1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>CMPEN</td><td><div style="word-wrap: break-word;"><b>Compare Enable</b><br>
0 = Compare Disabled.<br>
1 = Compare Enabled.<br>
Set this bit to 1 to enable compare CMPD[11:0] with specified channel conversion result when converted data is loaded into ADC_RESULTx register.<br>
When this bit is set to 1, and CMPMATCNT is 0, the CMPF will be set once the match is hit<br>
</div></td></tr><tr><td>
[1]</td><td>CMPIE</td><td><div style="word-wrap: break-word;"><b>Compare Interrupt Enable</b><br>
0 = Compare function interrupt Disabled.<br>
1 = Compare function interrupt Enabled.<br>
If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMATCNT, CMPF bit will be asserted, in the meanwhile, if CMPIE is set to 1, a compare interrupt request is generated.<br>
</div></td></tr><tr><td>
[2]</td><td>CMPCOND</td><td><div style="word-wrap: break-word;"><b>Compare Condition</b><br>
0 = Set the compare condition as that when a 12-bit A/D conversion result is less than the 12-bit CMPD (ADCMPRx[27:16]), the internal match counter will increase one.<br>
1 = Set the compare condition as that when a 12-bit A/D conversion result is greater or equal to the 12-bit CMPD (ADCMPRx[27:16]), the internal match counter will increase by one.<br>
Note: When the internal counter reaches the value to (CMPMATCNT +1), the CMPF bit will be set.<br>
</div></td></tr><tr><td>
[7:3]</td><td>CMPCH</td><td><div style="word-wrap: break-word;"><b>Compare Channel Selection</b><br>
This field selects the channel whose conversion result is selected to be compared.<br>
</div></td></tr><tr><td>
[11:8]</td><td>CMPMATCNT</td><td><div style="word-wrap: break-word;"><b>Compare Match Count</b><br>
When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND[2], the internal match counter will increase 1.<br>
When the internal counter reaches the value to (CMPMATCNT +1), the CMPF bit will be set.<br>
</div></td></tr><tr><td>
[27:16]</td><td>CMPD</td><td><div style="word-wrap: break-word;"><b>Comparison Data</b><br>
The 12 bits data is used to compare with conversion result of specified channel.<br>
Software can use it to monitor the external analog input pin voltage variation in scan mode without imposing a load on software.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CMPR1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SR</font><br><p> <font size="2">
Offset: 0x58  A/D Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>ADF</td><td><div style="word-wrap: break-word;"><b>A/D Conversion End Flag</b><br>
A status flag that indicates the end of A/D conversion.<br>
ADF is set to 1 at these two conditions:<br>
When A/D conversion ends in single mode<br>
When A/D conversion ends on all specified channels in scan mode.<br>
This flag can be cleared by writing 1 to it.<br>
</div></td></tr><tr><td>
[1]</td><td>CMPF0</td><td><div style="word-wrap: break-word;"><b>Compare Flag</b><br>
When the selected channel A/D conversion result meets setting condition in ADCMPR0 then this bit is set to 1.<br>
And it is cleared by writing 1 to self.<br>
0 = Conversion result in ADC_RESULTx does not meet ADCMPR0setting.<br>
1 = Conversion result in ADC_RESULTx meets ADCMPR0setting.<br>
This flag can be cleared by writing 1 to it.<br>
Note: When this flag is set, the matching counter will be reset to 0,and continue to count when user write 1 to clear CMPF0<br>
</div></td></tr><tr><td>
[2]</td><td>CMPF1</td><td><div style="word-wrap: break-word;"><b>Compare Flag</b><br>
When the selected channel A/D conversion result meets setting condition in ADCMPR1 then this bit is set to 1.<br>
And it is cleared by writing 1 to self.<br>
0 = Conversion result in ADC_RESULTx does not meet ADCMPR1 setting.<br>
1 = Conversion result in ADC_RESULTx meets ADCMPR1 setting.<br>
This flag can be cleared by writing 1 to it.<br>
Note: when this flag is set, the matching counter will be reset to 0,and continue to count when user write 1 to clear CMPF1<br>
</div></td></tr><tr><td>
[3]</td><td>BUSY</td><td><div style="word-wrap: break-word;"><b>BUSY/IDLE</b><br>
0 = A/D converter is in idle state.<br>
1 = A/D converter is busy at conversion.<br>
This bit is a mirror of ADST bit in ADCR. That is to say if ADST = 1,then BUSY is 1 and vice versa.<br>
It is read only.<br>
</div></td></tr><tr><td>
[8:4]</td><td>CHANNEL</td><td><div style="word-wrap: break-word;"><b>Current Conversion Channel</b><br>
This filed reflects current conversion channel when BUSY=1.<br>
When BUSY=0, it shows the next channel to be converted.<br>
It is read only.<br>
</div></td></tr><tr><td>
[16]</td><td>INITRDY</td><td><div style="word-wrap: break-word;"><b>ADC Power-Up Sequence Completed</b><br>
0 = ADC not powered up after system reset.<br>
1 = ADC has been powered up since the last system reset.<br>
Note: This bit will be set after system reset occurred and automatically cleared by power-up event.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SR;
    uint32_t RESERVE0[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PDMA</font><br><p> <font size="2">
Offset: 0x60  A/D PDMA current transfer data Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[11:0]</td><td>AD_PDMA</td><td><div style="word-wrap: break-word;"><b>ADC PDMA Current Transfer Data Register</b><br>
When PDMA transferring, read this register can monitor current PDMA transfer data.<br>
This is a read only register.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t PDMA;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PWRCTL</font><br><p> <font size="2">
Offset: 0x64  ADC Power Management Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>PWUPRDY</td><td><div style="word-wrap: break-word;"><b>ADC Power-Up Sequence Completed And Ready For Conversion</b><br>
0 = ADC is not ready for conversion; may be in power down state or in the progress of power up.<br>
1 = ADC is ready for conversion.<br>
</div></td></tr><tr><td>
[1]</td><td>PWDCALEN</td><td><div style="word-wrap: break-word;"><b>Power Up Calibration Function Enable</b><br>
1 = Power up with calibration.<br>
0 = Power up without calibration.<br>
Note: This bit work together with CALFBKSEL set 1<br>
</div></td></tr><tr><td>
[3:2]</td><td>PWDMOD</td><td><div style="word-wrap: break-word;"><b>Power-Down Mode</b><br>
00 = Power down<br>
01 = Reserved<br>
10 = Standby mode<br>
11 = Reserved<br>
Note: Different PWDMOD has different power down/up sequence, in order to avoid ADC powering up with wrong sequence; user must keep PWMOD consistent each time in powe down and power up<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PWRCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CALCTL</font><br><p> <font size="2">
Offset: 0x68  ADC Calibration  Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>CALEN</td><td><div style="word-wrap: break-word;"><b>Calibration Function Enable</b><br>
Enable this bit to turn on the calibration function block.<br>
0 = Disable<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>CALSTART</td><td><div style="word-wrap: break-word;"><b>Calibration Functional Block Start</b><br>
0 = Stops calibration functional block.<br>
1 = Starts calibration functional block.<br>
Note: This bit is set by SW and clear by HW; don't write 1 to this bit while CALEN = 0.<br>
</div></td></tr><tr><td>
[2]</td><td>CALDONE</td><td><div style="word-wrap: break-word;"><b>Calibrate Functional Block Complete</b><br>
0 = Not yet.<br>
1 = Selected functional block complete.<br>
</div></td></tr><tr><td>
[3]</td><td>CALSEL</td><td><div style="word-wrap: break-word;"><b>Select Calibration Functional Block</b><br>
0 = Load calibration functional block.<br>
1 = Calibration functional block.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CALCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CALWORD</font><br><p> <font size="2">
Offset: 0x6C  A/D calibration  load word register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[6:0]</td><td>CALWORD</td><td><div style="word-wrap: break-word;"><b>Calibration Word Register</b><br>
Write to this register with the previous calibration word before load calibration action<br>
Read this register after calibration done<br>
Note: The calibration block contains two parts "CALIBRATION" and "LOAD CALIBRATION"; if the calibration block is config as "CALIBRATION"; then this register represent the result of calibration when calibration is completed; if config as "LOAD CALIBRATION" ; config this register before loading calibration action, after loading calibration complete, the loaded calibration word will apply to the ADC;while in loading calibration function the loaded value will not be equal to the original CALWORD until calibration is done.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CALWORD;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SMPLCNT0</font><br><p> <font size="2">
Offset: 0x70  ADC Channel Sampling Time  Counter Register Group 0</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>CH0SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 0 Sampling Counter</b><br>
0000 = 0 ADC clock<br>
0001 = 1 ADC clock<br>
0010 = 2 ADC clocks<br>
0011 = 4 ADC clocks<br>
0100 = 8 ADC clocks<br>
0101 = 16 ADC clocks<br>
0110 = 32 ADC clocks<br>
0111 = 64 ADC clocks<br>
1000 = 128 ADC clocks<br>
1001 = 256 ADC clocks<br>
1010 = 512 ADC clocks<br>
Others = 1024 ADC clocks<br>
</div></td></tr><tr><td>
[7:4]</td><td>CH1SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 1 Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr><tr><td>
[11:8]</td><td>CH2SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 2 Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr><tr><td>
[15:12]</td><td>CH3SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 3 Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr><tr><td>
[19:16]</td><td>CH4SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 4 Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr><tr><td>
[23:20]</td><td>CH5SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 5 Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr><tr><td>
[27:24]</td><td>CH6SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 6 Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr><tr><td>
[31:28]</td><td>CH7SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 7 Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SMPLCNT0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SMPLCNT1</font><br><p> <font size="2">
Offset: 0x74  ADC Channel Sampling Time  Counter Register Group 1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>CH8SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 8 Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr><tr><td>
[7:4]</td><td>CH9SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 9 Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr><tr><td>
[11:8]</td><td>CH10SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 10 Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr><tr><td>
[15:12]</td><td>CH11SAMPCNT</td><td><div style="word-wrap: break-word;"><b>Channel 11 Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr><tr><td>
[19:16]</td><td>INTCHSAMPCNT</td><td><div style="word-wrap: break-word;"><b>Internal Channel (VTEMP, AVDD, AVSS, Int_VREF, DAC0, DAC1) Sampling Counter</b><br>
The same as Channel 0 sampling counter table.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SMPLCNT1;

} ADC_T;

/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */
#define ADC_RESULT_RSLT_Pos              (0)                                               /*!< ADC_T::RESULT: RSLT Position              */
#define ADC_RESULT_RSLT_Msk              (0xffful << ADC_RESULT_RSLT_Pos)                  /*!< ADC_T::RESULT: RSLT Mask                  */

#define ADC_RESULT_VALID_Pos             (16)                                              /*!< ADC_T::RESULT: VALID Position             */
#define ADC_RESULT_VALID_Msk             (0x1ul << ADC_RESULT_VALID_Pos)                   /*!< ADC_T::RESULT: VALID Mask                 */

#define ADC_RESULT_OVERRUN_Pos           (17)                                              /*!< ADC_T::RESULT: OVERRUN Position           */
#define ADC_RESULT_OVERRUN_Msk           (0x1ul << ADC_RESULT_OVERRUN_Pos)                 /*!< ADC_T::RESULT: OVERRUN Mask               */

#define ADC_CR_ADEN_Pos                (0)                                               /*!< ADC_T::CR: ADEN Position                */
#define ADC_CR_ADEN_Msk                (0x1ul << ADC_CR_ADEN_Pos)                      /*!< ADC_T::CR: ADEN Mask                    */

#define ADC_CR_ADIE_Pos                (1)                                               /*!< ADC_T::CR: ADIE Position                */
#define ADC_CR_ADIE_Msk                (0x1ul << ADC_CR_ADIE_Pos)                      /*!< ADC_T::CR: ADIE Mask                    */

#define ADC_CR_ADMD_Pos                (2)                                               /*!< ADC_T::CR: ADMD Position                */
#define ADC_CR_ADMD_Msk                (0x3ul << ADC_CR_ADMD_Pos)                      /*!< ADC_T::CR: ADMD Mask                    */

#define ADC_CR_TRGS_Pos                (4)                                               /*!< ADC_T::CR: TRGS Position                */
#define ADC_CR_TRGS_Msk                (0x3ul << ADC_CR_TRGS_Pos)                      /*!< ADC_T::CR: TRGS Mask                    */

#define ADC_CR_TRGCOND_Pos             (6)                                               /*!< ADC_T::CR: TRGCOND Position             */
#define ADC_CR_TRGCOND_Msk             (0x3ul << ADC_CR_TRGCOND_Pos)                   /*!< ADC_T::CR: TRGCOND Mask                 */

#define ADC_CR_TRGE_Pos                (8)                                               /*!< ADC_T::CR: TRGE Position                */
#define ADC_CR_TRGE_Msk                (0x1ul << ADC_CR_TRGE_Pos)                      /*!< ADC_T::CR: TRGE Mask                    */

#define ADC_CR_PTEN_Pos                (9)                                               /*!< ADC_T::CR: PTEN Position                */
#define ADC_CR_PTEN_Msk                (0x1ul << ADC_CR_PTEN_Pos)                      /*!< ADC_T::CR: PTEN Mask                    */

#define ADC_CR_DIFF_Pos                (10)                                              /*!< ADC_T::CR: DIFF Position                */
#define ADC_CR_DIFF_Msk                (0x1ul << ADC_CR_DIFF_Pos)                      /*!< ADC_T::CR: DIFF Mask                    */

#define ADC_CR_ADST_Pos                (11)                                              /*!< ADC_T::CR: ADST Position                */
#define ADC_CR_ADST_Msk                (0x1ul << ADC_CR_ADST_Pos)                      /*!< ADC_T::CR: ADST Mask                    */

#define ADC_CR_TMSEL_Pos               (12)                                              /*!< ADC_T::CR: TMSEL Position               */
#define ADC_CR_TMSEL_Msk               (0x3ul << ADC_CR_TMSEL_Pos)                     /*!< ADC_T::CR: TMSEL Mask                   */

#define ADC_CR_TMTRGMOD_Pos            (15)                                              /*!< ADC_T::CR: TMTRGMOD Position            */
#define ADC_CR_TMTRGMOD_Msk            (0x1ul << ADC_CR_TMTRGMOD_Pos)                  /*!< ADC_T::CR: TMTRGMOD Mask                */

#define ADC_CR_REFSEL_Pos              (16)                                              /*!< ADC_T::CR: REFSEL Position              */
#define ADC_CR_REFSEL_Msk              (0x3ul << ADC_CR_REFSEL_Pos)                    /*!< ADC_T::CR: REFSEL Mask                  */

#define ADC_CR_RESSEL_Pos              (18)                                              /*!< ADC_T::CR: RESSEL Position              */
#define ADC_CR_RESSEL_Msk              (0x3ul << ADC_CR_RESSEL_Pos)                    /*!< ADC_T::CR: RESSEL Mask                  */

#define ADC_CR_TMPDMACNT_Pos           (24)                                              /*!< ADC_T::CR: TMPDMACNT Position           */
#define ADC_CR_TMPDMACNT_Msk           (0xfful << ADC_CR_TMPDMACNT_Pos)                /*!< ADC_T::CR: TMPDMACNT Mask               */

#define ADC_CHEN_CHEN0_Pos             (0)                                               /*!< ADC_T::CHEN: CHEN0 Position             */
#define ADC_CHEN_CHEN0_Msk             (0x1ul << ADC_CHEN_CHEN0_Pos)                   /*!< ADC_T::CHEN: CHEN0 Mask                 */

#define ADC_CMPR_CMPEN_Pos            (0)                                               /*!< ADC_T::CMPR: CMPEN Position            */
#define ADC_CMPR_CMPEN_Msk            (0x1ul << ADC_CMPR_CMPEN_Pos)                  /*!< ADC_T::CMPR: CMPEN Mask                */

#define ADC_CMPR_CMPIE_Pos            (1)                                               /*!< ADC_T::CMPR: CMPIE Position            */
#define ADC_CMPR_CMPIE_Msk            (0x1ul << ADC_CMPR_CMPIE_Pos)                  /*!< ADC_T::CMPR: CMPIE Mask                */

#define ADC_CMPR_CMPCOND_Pos          (2)                                               /*!< ADC_T::CMPR: CMPCOND Position          */
#define ADC_CMPR_CMPCOND_Msk          (0x1ul << ADC_CMPR_CMPCOND_Pos)                /*!< ADC_T::CMPR: CMPCOND Mask              */

#define ADC_CMPR_CMPCH_Pos            (3)                                               /*!< ADC_T::CMPR: CMPCH Position            */
#define ADC_CMPR_CMPCH_Msk            (0x1ful << ADC_CMPR_CMPCH_Pos)                 /*!< ADC_T::CMPR: CMPCH Mask                */

#define ADC_CMPR_CMPMATCNT_Pos        (8)                                               /*!< ADC_T::CMPR: CMPMATCNT Position        */
#define ADC_CMPR_CMPMATCNT_Msk        (0xful << ADC_CMPR_CMPMATCNT_Pos)              /*!< ADC_T::CMPR: CMPMATCNT Mask            */

#define ADC_CMPR_CMPD_Pos             (16)                                              /*!< ADC_T::CMPR: CMPD Position             */
#define ADC_CMPR_CMPD_Msk             (0xffful << ADC_CMPR_CMPD_Pos)                 /*!< ADC_T::CMPR: CMPD Mask                 */

#define ADC_SR_ADF_Pos                 (0)                                               /*!< ADC_T::SR: ADF Position                 */
#define ADC_SR_ADF_Msk                 (0x1ul << ADC_SR_ADF_Pos)                       /*!< ADC_T::SR: ADF Mask                     */

#define ADC_SR_CMPF0_Pos               (1)                                               /*!< ADC_T::SR: CMPF0 Position               */
#define ADC_SR_CMPF0_Msk               (0x1ul << ADC_SR_CMPF0_Pos)                     /*!< ADC_T::SR: CMPF0 Mask                   */

#define ADC_SR_CMPF1_Pos               (2)                                               /*!< ADC_T::SR: CMPF1 Position               */
#define ADC_SR_CMPF1_Msk               (0x1ul << ADC_SR_CMPF1_Pos)                     /*!< ADC_T::SR: CMPF1 Mask                   */

#define ADC_SR_BUSY_Pos                (3)                                               /*!< ADC_T::SR: BUSY Position                */
#define ADC_SR_BUSY_Msk                (0x1ul << ADC_SR_BUSY_Pos)                      /*!< ADC_T::SR: BUSY Mask                    */

#define ADC_SR_CHANNEL_Pos             (4)                                               /*!< ADC_T::SR: CHANNEL Position             */
#define ADC_SR_CHANNEL_Msk             (0x1ful << ADC_SR_CHANNEL_Pos)                  /*!< ADC_T::SR: CHANNEL Mask                 */

#define ADC_SR_INITRDY_Pos             (16)                                              /*!< ADC_T::SR: INITRDY Position             */
#define ADC_SR_INITRDY_Msk             (0x1ul << ADC_SR_INITRDY_Pos)                   /*!< ADC_T::SR: INITRDY Mask                 */

#define ADC_PDMA_AD_PDMA_Pos           (0)                                               /*!< ADC_T::PDMA: AD_PDMA Position           */
#define ADC_PDMA_AD_PDMA_Msk           (0xffful << ADC_PDMA_AD_PDMA_Pos)               /*!< ADC_T::PDMA: AD_PDMA Mask               */

#define ADC_PWRCTL_PWUPRDY_Pos           (0)                                               /*!< ADC_T::PWRCTL: PWUPRDY Position           */
#define ADC_PWRCTL_PWUPRDY_Msk           (0x1ul << ADC_PWRCTL_PWUPRDY_Pos)                 /*!< ADC_T::PWRCTL: PWUPRDY Mask               */

#define ADC_PWRCTL_PWDCALEN_Pos          (1)                                               /*!< ADC_T::PWRCTL: PWDCALEN Position          */
#define ADC_PWRCTL_PWDCALEN_Msk          (0x1ul << ADC_PWRCTL_PWDCALEN_Pos)                /*!< ADC_T::PWRCTL: PWDCALEN Mask              */

#define ADC_PWRCTL_PWDMOD_Pos            (2)                                               /*!< ADC_T::PWRCTL: PWDMOD Position            */
#define ADC_PWRCTL_PWDMOD_Msk            (0x3ul << ADC_PWRCTL_PWDMOD_Pos)                  /*!< ADC_T::PWRCTL: PWDMOD Mask                */

#define ADC_CALCTL_CALEN_Pos          (0)                                               /*!< ADC_T::CALCTL: CALEN Position          */
#define ADC_CALCTL_CALEN_Msk          (0x1ul << ADC_CALCTL_CALEN_Pos)                /*!< ADC_T::CALCTL: CALEN Mask              */

#define ADC_CALCTL_CALSTART_Pos       (1)                                               /*!< ADC_T::CALCTL: CALSTART Position       */
#define ADC_CALCTL_CALSTART_Msk       (0x1ul << ADC_CALCTL_CALSTART_Pos)             /*!< ADC_T::CALCTL: CALSTART Mask           */

#define ADC_CALCTL_CALDONE_Pos        (2)                                               /*!< ADC_T::CALCTL: CALDONE Position        */
#define ADC_CALCTL_CALDONE_Msk        (0x1ul << ADC_CALCTL_CALDONE_Pos)              /*!< ADC_T::CALCTL: CALDONE Mask            */

#define ADC_CALCTL_CALSEL_Pos         (3)                                               /*!< ADC_T::CALCTL: CALSEL Position         */
#define ADC_CALCTL_CALSEL_Msk         (0x1ul << ADC_CALCTL_CALSEL_Pos)               /*!< ADC_T::CALCTL: CALSEL Mask             */

#define ADC_CALWORD_CALWORD_Pos       (0)                                               /*!< ADC_T::CALWORD: CALWORD Position       */
#define ADC_CALWORD_CALWORD_Msk       (0x7ful << ADC_CALWORD_CALWORD_Pos)            /*!< ADC_T::CALWORD: CALWORD Mask           */

#define ADC_SMPLCNT0_CH0SAMPCNT_Pos    (0)                                               /*!< ADC_T::SMPLCNT0: CH0SAMPCNT Position    */
#define ADC_SMPLCNT0_CH0SAMPCNT_Msk    (0xful << ADC_SMPLCNT0_CH0SAMPCNT_Pos)          /*!< ADC_T::SMPLCNT0: CH0SAMPCNT Mask        */

#define ADC_SMPLCNT1_CH8SAMPCNT_Pos    (0)                                               /*!< ADC_T::SMPLCNT1: CH8SAMPCNT Position    */
#define ADC_SMPLCNT1_CH8SAMPCNT_Msk    (0xful << ADC_SMPLCNT1_CH8SAMPCNT_Pos)          /*!< ADC_T::SMPLCNT1: CH8SAMPCNT Mask        */

#define ADC_SMPLCNT1_INTCHSAMPCNT_Pos  (16)                                              /*!< ADC_T::SMPLCNT1: INTCHSAMPCNT Position  */
#define ADC_SMPLCNT1_INTCHSAMPCNT_Msk  (0xful << ADC_SMPLCNT1_INTCHSAMPCNT_Pos)        /*!< ADC_T::SMPLCNT1: INTCHSAMPCNT Mask      */

/**@}*/ /* ADC_CONST */
/**@}*/ /* end of ADC register group */


/*---------------------- System Clock Controller -------------------------*/
/**
    @addtogroup CLK System Clock Controller(CLK)
    Memory Mapped Structure for CLK Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PWRCTL</font><br><p> <font size="2">
Offset: 0x00  System Power Down Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>HXT_EN</td><td><div style="word-wrap: break-word;"><b>HXT Control</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
The bit default value is set by flash controller user configuration register config0 [26].<br>
0 = Disabled.<br>
1 = Enabled.<br>
HXT is disabled by default.<br>
</div></td></tr><tr><td>
[1]</td><td>LXT_EN</td><td><div style="word-wrap: break-word;"><b>LXT Control</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Disabled.<br>
1 = Enabled.<br>
LXT is disabled by default.<br>
</div></td></tr><tr><td>
[2]</td><td>HIRC_EN</td><td><div style="word-wrap: break-word;"><b>HIRC Control</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Disabled.<br>
1 = Enabled.<br>
HIRC is enabled by default.<br>
</div></td></tr><tr><td>
[3]</td><td>LIRC_EN</td><td><div style="word-wrap: break-word;"><b>LIRC Control</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Disabled.<br>
1 = Enabled.<br>
LIRC is enabled by default.<br>
</div></td></tr><tr><td>
[4]</td><td>WK_DLY</td><td><div style="word-wrap: break-word;"><b>Wake-Up Delay Counter Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
When chip wakes up from Power-down mode, the clock control will delay 4096 clock cycles to wait HXT stable or 16 clock cycles to wait HIRC stable.<br>
0 = Delay clock cycle Disabled.<br>
1 = Delay clock cycle Enabled.<br>
</div></td></tr><tr><td>
[5]</td><td>PD_WK_IE</td><td><div style="word-wrap: break-word;"><b>Power-Down Mode Wake-Up Interrupt Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Disabled.<br>
1 = Enabled.<br>
PD_WK_INT will be set if both PD_WK_IS and PD_WK_IE are high.<br>
</div></td></tr><tr><td>
[6]</td><td>PD_EN</td><td><div style="word-wrap: break-word;"><b>Chip Power-Down Mode Enable Bit</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
When CPU sets this bit, the chip power down is enabled and chip will not enter Power-down mode until CPU sleep mode is also active<br>
When chip wakes up from Power-down mode, this bit will be auto cleared.<br>
When chip is in Power-down mode, the LDO, HXT and HIRC will be disabled, but LXT and LIRC are not controlled by Power-down mode.<br>
When power down, the PLL and system clock (CPU, HCLKx and PCLKx) are also disabled no matter the Clock Source selection.<br>
Peripheral clocks are not controlled by this bit, if peripheral Clock Source is from LXT or LIRC.<br>
In Power-down mode, flash macro power is ON.<br>
0 = Chip operated in Normal mode.<br>
1 = Chip power down Enabled.<br>
</div></td></tr><tr><td>
[8]</td><td>HXT_SELXT</td><td><div style="word-wrap: break-word;"><b>HXT SELXT</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = High frequency crystal loop back path Disabled. It is used for external oscillator.<br>
1 = High frequency crystal loop back path Enabled. It is used for external crystal.<br>
</div></td></tr><tr><td>
[9]</td><td>HXT_GAIN</td><td><div style="word-wrap: break-word;"><b>HXT Gain Control Bit</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
Gain control is used to enlarge the gain of crystal to make sure crystal wok normally.<br>
If gain control is enabled, crystal will consume more power than gain control off.<br>
0 = Gain control Disabled. It means HXT gain is always high.<br>
For 16MHz to 24MHz crystal.<br>
1 = Gain control Enabled. HXT gain will be high lasting 2ms then low. This is for power saving.<br>
For 4MHz to 16MHz crystal.<br>
</div></td></tr><tr><td>
[10]</td><td>LXT_SCNT</td><td><div style="word-wrap: break-word;"><b>LXT Stable Time Control</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Delay 4096 LXT before LXT output.<br>
1 = Delay 8192 LXT before LXT output.<br>
</div></td></tr><tr><td>
[12:11]</td><td>HXT_HF_ST</td><td><div style="word-wrap: break-word;"><b>HXT Frequency Selection</b><br>
Set this bit to meet HXT frequency selection (Recommended)<br>
00 = HXT frequency is from 4 MHz to 12 MHz.<br>
01 = HXT frequency is from 12 MHz to 16 MHz.<br>
10 = HXT frequency is from 16 MHz to 24 MHz.<br>
11 = Reserved.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PWRCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">AHBCLK</font><br><p> <font size="2">
Offset: 0x04  AHB Devices Clock Enable Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>GPIO_EN</td><td><div style="word-wrap: break-word;"><b>GPIO Controller Clock Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>DMA_EN</td><td><div style="word-wrap: break-word;"><b>DMA Controller Clock Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>ISP_EN</td><td><div style="word-wrap: break-word;"><b>Flash ISP Controller Clock Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[3]</td><td>EBI_EN</td><td><div style="word-wrap: break-word;"><b>EBI Controller Clock Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[4]</td><td>SRAM_EN</td><td><div style="word-wrap: break-word;"><b>SRAM Controller Clock Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[5]</td><td>TICK_EN</td><td><div style="word-wrap: break-word;"><b>System Tick Clock Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t AHBCLK;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">APBCLK</font><br><p> <font size="2">
Offset: 0x08  APB Devices Clock Enable Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>WDT_EN</td><td><div style="word-wrap: break-word;"><b>Watchdog Timer Clock Enable Control</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
This bit is used to control the WDT APB clock only, The WDT engine Clock Source is from LIRC.<br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>RTC_EN</td><td><div style="word-wrap: break-word;"><b>Real-Time-Clock Clock Enable Control</b><br>
This bit is used to control the RTC APB clock only, The RTC engine Clock Source is from LXT.<br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>TMR0_EN</td><td><div style="word-wrap: break-word;"><b>Timer0 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[3]</td><td>TMR1_EN</td><td><div style="word-wrap: break-word;"><b>Timer1 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[4]</td><td>TMR2_EN</td><td><div style="word-wrap: break-word;"><b>Timer2 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[5]</td><td>TMR3_EN</td><td><div style="word-wrap: break-word;"><b>Timer3 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[6]</td><td>FDIV_EN</td><td><div style="word-wrap: break-word;"><b>Frequency Divider Output Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[7]</td><td>SC2_EN</td><td><div style="word-wrap: break-word;"><b>SmartCard 2 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[8]</td><td>I2C0_EN</td><td><div style="word-wrap: break-word;"><b>I2C0 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[9]</td><td>I2C1_EN</td><td><div style="word-wrap: break-word;"><b>I2C1 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[12]</td><td>SPI0_EN</td><td><div style="word-wrap: break-word;"><b>SPI0 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[13]</td><td>SPI1_EN</td><td><div style="word-wrap: break-word;"><b>SPI1 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[14]</td><td>SPI2_EN</td><td><div style="word-wrap: break-word;"><b>SPI2 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[16]</td><td>UART0_EN</td><td><div style="word-wrap: break-word;"><b>UART0 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[17]</td><td>UART1_EN</td><td><div style="word-wrap: break-word;"><b>UART1 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[20]</td><td>PWM0_CH01_EN</td><td><div style="word-wrap: break-word;"><b>PWM0 Channel 0 And Channel 1Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[21]</td><td>PWM0_CH23_EN</td><td><div style="word-wrap: break-word;"><b>PWM0 Channel 2 And Channel 3 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[22]</td><td>PWM1_CH01_EN</td><td><div style="word-wrap: break-word;"><b>PWM1 Channel 0 And Channel 1 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[23]</td><td>PWM1_CH23_EN</td><td><div style="word-wrap: break-word;"><b>PWM1 Channel 2 And Channel 3 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[25]</td><td>DAC_EN</td><td><div style="word-wrap: break-word;"><b>12-Bit DAC Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[26]</td><td>LCD_EN</td><td><div style="word-wrap: break-word;"><b>LCD Controller Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[27]</td><td>USBD_EN</td><td><div style="word-wrap: break-word;"><b>USB FS Device Controller Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[28]</td><td>ADC_EN</td><td><div style="word-wrap: break-word;"><b>Analog-Digital-Converter (ADC) Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[29]</td><td>I2S_EN</td><td><div style="word-wrap: break-word;"><b>I2S Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[30]</td><td>SC0_EN</td><td><div style="word-wrap: break-word;"><b>SmartCard 0 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[31]</td><td>SC1_EN</td><td><div style="word-wrap: break-word;"><b>SmartCard 1 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t APBCLK;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CLKSTATUS</font><br><p> <font size="2">
Offset: 0x0C  Clock status monitor Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>HXT_STB</td><td><div style="word-wrap: break-word;"><b>HXT Clock Source Stable Flag</b><br>
0 = HXT clock is not stable or not enable.<br>
1 = HXT clock is stable.<br>
</div></td></tr><tr><td>
[1]</td><td>LXT_STB</td><td><div style="word-wrap: break-word;"><b>LXT Clock Source Stable Flag</b><br>
0 = LXT clock is not stable or not enable.<br>
1 = LXT clock is stable.<br>
</div></td></tr><tr><td>
[2]</td><td>PLL_STB</td><td><div style="word-wrap: break-word;"><b>PLL Clock Source Stable Flag</b><br>
0 = PLL clock is not stable or not enable.<br>
1 = PLL clock is stable.<br>
</div></td></tr><tr><td>
[3]</td><td>LIRC_STB</td><td><div style="word-wrap: break-word;"><b>LIRC Clock Source Stable Flag</b><br>
0 = LIRC clock is not stable or not enable.<br>
1 = LIRC clock is stable.<br>
</div></td></tr><tr><td>
[4]</td><td>HIRC_STB</td><td><div style="word-wrap: break-word;"><b>HIRC Clock Source Stable Flag</b><br>
0 = HIRC clock is not stable or not enable.<br>
1 = HIRC clock is stable.<br>
</div></td></tr><tr><td>
[7]</td><td>CLK_SW_FAIL</td><td><div style="word-wrap: break-word;"><b>Clock Switch Fail Flag</b><br>
0 = Clock switch success.<br>
1 = Clock switch fail.<br>
This bit will be set when target switch Clock Source is not stable. This bit is write 1 clear<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CLKSTATUS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CLKSEL0</font><br><p> <font size="2">
Offset: 0x10  Clock Source Select Control Register 0</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>HCLK_S</td><td><div style="word-wrap: break-word;"><b>HCLK Clock Source Selection</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
Note:<br>
Before Clock Source switches, the related clock sources (pre-select and new-select) must be turn on<br>
The 3-bit default value is reloaded with the value of CFOSC (Config0[26:24]) in user configuration register in Flash controller by any reset.<br>
Therefore the default value is either 000b or 111b.<br>
000 = HXT<br>
001 = LXT<br>
010 = PLL Clock<br>
011 = LIRC<br>
111 = HIRC<br>
Others = Reserved<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CLKSEL0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CLKSEL1</font><br><p> <font size="2">
Offset: 0x14  Clock Source Select Control Register 1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[1:0]</td><td>UART_S</td><td><div style="word-wrap: break-word;"><b>UART 0/1 Clock Source Selection (UART0 And UART1 Use The Same Clock Source Selection)</b><br>
00 = HXT<br>
01 = LXT<br>
10 = PLL Clock<br>
11 = HIRC<br>
</div></td></tr><tr><td>
[3:2]</td><td>ADC_S</td><td><div style="word-wrap: break-word;"><b>ADC Clock Source Selection</b><br>
00 = HXT<br>
01 = LXT<br>
10 = PLL Clock<br>
11 = HIRC<br>
</div></td></tr><tr><td>
[5:4]</td><td>PWM0_CH01_S</td><td><div style="word-wrap: break-word;"><b>PWM0 Channel 0 And Channel 1 Clock Source Selection</b><br>
PWM0 channel 0 and channel 1 use the same Engine clock source, both of them with the same prescaler<br>
00 = HXT<br>
01 = LXT<br>
10 = HCLK<br>
11 = HIRC<br>
</div></td></tr><tr><td>
[7:6]</td><td>PWM0_CH23_S</td><td><div style="word-wrap: break-word;"><b>PWM0 Channel 2 And Channel 3 Clock Source Selection</b><br>
PWM0 channel 2 and channel 3 use the same Engine clock source, both of them with the same prescaler<br>
00 = HXT<br>
01 = LXT<br>
10 = HCLK<br>
11 = HIRC<br>
</div></td></tr><tr><td>
[10:8]</td><td>TMR0_S</td><td><div style="word-wrap: break-word;"><b>Timer0 Clock Source Selection</b><br>
000 = HXT<br>
001 = LXT<br>
010 = LIRC<br>
011 = External Pin<br>
111 = HIRC<br>
Others = Reserved<br>
</div></td></tr><tr><td>
[14:12]</td><td>TMR1_S</td><td><div style="word-wrap: break-word;"><b>Timer1 Clock Source Selection</b><br>
000 = HXT<br>
001 = LXT<br>
010 = LIRC<br>
011 = External Pin<br>
111 = HIRC<br>
Others = Reserved<br>
</div></td></tr><tr><td>
[18]</td><td>LCD_S</td><td><div style="word-wrap: break-word;"><b>LCD Clock Source Selection</b><br>
0 = Clock Source from LXT.<br>
1 = Reserved.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CLKSEL1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CLKSEL2</font><br><p> <font size="2">
Offset: 0x18  Clock Source Select Control Register 2</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:2]</td><td>FRQDIV_S</td><td><div style="word-wrap: break-word;"><b>Clock Divider Clock Source Selection</b><br>
00 = HXT<br>
01 = LXT<br>
10 = HCLK<br>
11 = HIRC<br>
</div></td></tr><tr><td>
[5:4]</td><td>PWM1_CH01_S</td><td><div style="word-wrap: break-word;"><b>PWM1 Channel 0 And Channel 1 Clock Source Selection</b><br>
PWM1 channel 0 and channel 1 use the same Engine clock source, both of them with the same pre-scale<br>
00 = HXT<br>
01 = LXT<br>
10 = HCLK<br>
11 = HIRC<br>
</div></td></tr><tr><td>
[7:6]</td><td>PWM1_CH23_S</td><td><div style="word-wrap: break-word;"><b>PWM1 Channel 2 And Channel 2 Clock Source Selection</b><br>
PWM1 channel 2 and channel 3 use the same Engine clock source, both of them with the same pre-scale<br>
00 = HXT<br>
01 = LXT<br>
10 = HCLK<br>
11 = HIRC<br>
</div></td></tr><tr><td>
[10:8]</td><td>TMR2_S</td><td><div style="word-wrap: break-word;"><b>Timer2 Clock Source Selection</b><br>
000 = HXT<br>
001 = LXT<br>
010 = LIRC<br>
011 = External Pin<br>
111 = HIRC<br>
Others = Reserved<br>
</div></td></tr><tr><td>
[14:12]</td><td>TMR3_S</td><td><div style="word-wrap: break-word;"><b>Timer3 Clock Source Selection</b><br>
000 = HXT<br>
001 = LXT<br>
010 = LIRC<br>
011 = External Pin<br>
111 = HIRC<br>
Others = Reserved<br>
</div></td></tr><tr><td>
[17:16]</td><td>I2S_S</td><td><div style="word-wrap: break-word;"><b>I2S Clock Source Selection</b><br>
00 = HXT<br>
01 = PLL Clock<br>
10 = HIRC<br>
11 = HIRC<br>
</div></td></tr><tr><td>
[19:18]</td><td>SC_S</td><td><div style="word-wrap: break-word;"><b>SC Clock Source Selection</b><br>
00 = HXT<br>
01 = PLL Clock<br>
10 = HIRC<br>
11 = HIRC<br>
Note: SC0,SC1 and SC2 use the same Clock Source selection but they have different clock divider number.<br>
</div></td></tr><tr><td>
[20]</td><td>SPI0_S</td><td><div style="word-wrap: break-word;"><b>SPI0 Clock Source Selection</b><br>
0 = PLL.<br>
1 = HCLK.<br>
</div></td></tr><tr><td>
[21]</td><td>SPI1_S</td><td><div style="word-wrap: break-word;"><b>SPI1 Clock Source Selection</b><br>
0 = PLL.<br>
1 = HCLK.<br>
</div></td></tr><tr><td>
[22]</td><td>SPI2_S</td><td><div style="word-wrap: break-word;"><b>SPI2 Clock Source Selection</b><br>
0 = PLL.<br>
1 = HCLK.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CLKSEL2;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CLKDIV0</font><br><p> <font size="2">
Offset: 0x1C  Clock Divider Number Register 0</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>HCLK_N</td><td><div style="word-wrap: break-word;"><b>HCLK Clock Divide Number From HCLK Clock Source</b><br>
The HCLK clock frequency = (HCLK Clock Source frequency) / (HCLK_N + 1).<br>
</div></td></tr><tr><td>
[7:4]</td><td>USB_N</td><td><div style="word-wrap: break-word;"><b>USB Clock Divide Number From PLL Clock</b><br>
The USB clock frequency = (PLL frequency ) / (USB_N + 1).<br>
</div></td></tr><tr><td>
[11:8]</td><td>UART_N</td><td><div style="word-wrap: break-word;"><b>UART Clock Divide Number From UART Clock Source</b><br>
The UART clock frequency = (UART Clock Source frequency ) / (UART_N + 1).<br>
</div></td></tr><tr><td>
[15:12]</td><td>I2S_N</td><td><div style="word-wrap: break-word;"><b>I2S Clock Divide Number From I2S Clock Source</b><br>
The I2S clock frequency = (I2S Clock Source frequency ) / (I2S_N + 1).<br>
</div></td></tr><tr><td>
[23:16]</td><td>ADC_N</td><td><div style="word-wrap: break-word;"><b>ADC Clock Divide Number From ADC Clock Source</b><br>
The ADC clock frequency = (ADC Clock Source frequency ) / (ADC_N + 1).<br>
</div></td></tr><tr><td>
[31:28]</td><td>SC0_N</td><td><div style="word-wrap: break-word;"><b>SC 0 Clock Divide Number From SC 0 Clock Source</b><br>
The SC 0 clock frequency = (SC0 Clock Source frequency ) / (SC0_N + 1).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CLKDIV0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CLKDIV1</font><br><p> <font size="2">
Offset: 0x20  Clock Divider Number Register 1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>SC1_N</td><td><div style="word-wrap: break-word;"><b>SC 1 Clock Divide Number From SC 1 Clock Source</b><br>
The SC 1 clock frequency = (SC 1 Clock Source frequency ) / (SC1_N + 1).<br>
</div></td></tr><tr><td>
[7:4]</td><td>SC2_N</td><td><div style="word-wrap: break-word;"><b>SC 2 Clock Divide Number From SC2 Clock Source</b><br>
The SC 2 clock frequency = (SC 2 Clock Source frequency ) / (SC2_N + 1).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CLKDIV1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PLLCTL</font><br><p> <font size="2">
Offset: 0x24  PLL Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[4:0]</td><td>FB_DV</td><td><div style="word-wrap: break-word;"><b>PLL Feedback Divider Control Pins</b><br>
Refer to the formulas below the table.<br>
The range of FB_DV is from 0 to 63.<br>
</div></td></tr><tr><td>
[9:8]</td><td>IN_DV</td><td><div style="word-wrap: break-word;"><b>PLL Input Divider Control Pins</b><br>
Refer to the formulas below the table.<br>
</div></td></tr><tr><td>
[12]</td><td>OUT_DV</td><td><div style="word-wrap: break-word;"><b>PLL Output Divider Control Pins</b><br>
Refer to the formulas below the table. This bit MUST be 0 for PLL output low deviation.<br>
</div></td></tr><tr><td>
[16]</td><td>PD</td><td><div style="word-wrap: break-word;"><b>Power-Down Mode</b><br>
If set the PD_EN bit "1" in PWR_CTL register, the PLL will enter Power-down mode too<br>
0 = PLL is in normal mode.<br>
1 = PLL is in power-down mode (default).<br>
</div></td></tr><tr><td>
[17]</td><td>PLL_SRC</td><td><div style="word-wrap: break-word;"><b>PLL Source Clock Select</b><br>
0 = PLL source clock from HXT.<br>
1 = PLL source clock from HIRC.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PLLCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">FRQDIV</font><br><p> <font size="2">
Offset: 0x28  Frequency Divider Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>FSEL</td><td><div style="word-wrap: break-word;"><b>Divider Output Frequency Selection Bits</b><br>
The formula of output frequency is<br>
Fout = Fin/2^(N+1),.<br>
Where Fin is the input clock frequency, Fout is the frequency of divider output clock and N is the 4-bit value of FSEL[3:0].<br>
</div></td></tr><tr><td>
[4]</td><td>FDIV_EN</td><td><div style="word-wrap: break-word;"><b>Frequency Divider Enable Bit</b><br>
0 = Frequency Divider Disabled.<br>
1 = Frequency Divider Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t FRQDIV;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MCLKO</font><br><p> <font size="2">
Offset: 0x2C  Module Clock Output Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>MCLK_SEL</td><td><div style="word-wrap: break-word;"><b>Module Clock Output Source Selection (PC.0)</b><br>
000000 = ISP_CLK<br>
000001 = HIRC<br>
000010 = HXT<br>
000011 = LXT<br>
000100 = LIRC<br>
000101 = PLL output<br>
000110 = PLL input<br>
000111 = System Tick<br>
001000 = HCLK clock<br>
001010 = PCLK clock<br>
100000 = TMR0_CLK<br>
100001 = TMR1_CLK<br>
100010 = UART0_CLK<br>
100011 = USB_CLK<br>
100100 = ADC_CLK<br>
100101 = WDT_CLK<br>
100110 = PWM0_CH01_CLK<br>
100111 = PWM0_CH32_CLK<br>
101001 = LCD_CLK<br>
111000 = TMR2_CLK<br>
111001 = TMR3_CLK<br>
111010 = UART1_CLK<br>
111011 = PWM1_CH01_CLK<br>
111100 = PWM1_CH23_CLK<br>
111101 = I&sup2;S_CLK<br>
111110 = SC0_CLK<br>
111111 = SC1_CLK<br>
</div></td></tr><tr><td>
[7]</td><td>MCLK_EN</td><td><div style="word-wrap: break-word;"><b>Module Clock Output Enable</b><br>
User can get the module clock output from PC.0 pin via choosing the clock source in the MCLK_SEL bit field and then setting MCLK_EN bit to 1.<br>
0 = Module clock output Disabled.<br>
1 = Module clock output Enabled.<br>
Note: If this bit is enabled, PC.0 will be configured to module clock output and the setting of PC0_MFP will be ineffective<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MCLKO;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">WK_INTSTS</font><br><p> <font size="2">
Offset: 0x30  Wake-up interrupt status</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>PD_WK_IS</td><td><div style="word-wrap: break-word;"><b>Wake-Up Interrupt Status In Chip Power-Down Mode</b><br>
This bit indicates that some event resumes chip from Power-down mode<br>
The status is set if external interrupts, UART, GPIO, RTC, USB, SPI, Timer, WDT, and BOD wake-up occurred.<br>
Write 1 to clear this bit.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO  uint32_t WK_INTSTS;

} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_PWRCTL_HXT_EN_Pos            (0)                                               /*!< CLK_T::PWRCTL: HXT_EN Position            */
#define CLK_PWRCTL_HXT_EN_Msk            (0x1ul << CLK_PWRCTL_HXT_EN_Pos)                  /*!< CLK_T::PWRCTL: HXT_EN Mask                */

#define CLK_PWRCTL_LXT_EN_Pos            (1)                                               /*!< CLK_T::PWRCTL: LXT_EN Position            */
#define CLK_PWRCTL_LXT_EN_Msk            (0x1ul << CLK_PWRCTL_LXT_EN_Pos)                  /*!< CLK_T::PWRCTL: LXT_EN Mask                */

#define CLK_PWRCTL_HIRC_EN_Pos           (2)                                               /*!< CLK_T::PWRCTL: HIRC_EN Position           */
#define CLK_PWRCTL_HIRC_EN_Msk           (0x1ul << CLK_PWRCTL_HIRC_EN_Pos)                 /*!< CLK_T::PWRCTL: HIRC_EN Mask               */

#define CLK_PWRCTL_LIRC_EN_Pos           (3)                                               /*!< CLK_T::PWRCTL: LIRC_EN Position           */
#define CLK_PWRCTL_LIRC_EN_Msk           (0x1ul << CLK_PWRCTL_LIRC_EN_Pos)                 /*!< CLK_T::PWRCTL: LIRC_EN Mask               */

#define CLK_PWRCTL_WK_DLY_Pos            (4)                                               /*!< CLK_T::PWRCTL: WK_DLY Position            */
#define CLK_PWRCTL_WK_DLY_Msk            (0x1ul << CLK_PWRCTL_WK_DLY_Pos)                  /*!< CLK_T::PWRCTL: WK_DLY Mask                */

#define CLK_PWRCTL_PD_WK_IE_Pos          (5)                                               /*!< CLK_T::PWRCTL: PD_WK_IE Position          */
#define CLK_PWRCTL_PD_WK_IE_Msk          (0x1ul << CLK_PWRCTL_PD_WK_IE_Pos)                /*!< CLK_T::PWRCTL: PD_WK_IE Mask              */

#define CLK_PWRCTL_PD_EN_Pos             (6)                                               /*!< CLK_T::PWRCTL: PD_EN Position             */
#define CLK_PWRCTL_PD_EN_Msk             (0x1ul << CLK_PWRCTL_PD_EN_Pos)                   /*!< CLK_T::PWRCTL: PD_EN Mask                 */

#define CLK_PWRCTL_HXT_SELXT_Pos         (8)                                               /*!< CLK_T::PWRCTL: HXT_SELXT Position         */
#define CLK_PWRCTL_HXT_SELXT_Msk         (0x1ul << CLK_PWRCTL_HXT_SELXT_Pos)               /*!< CLK_T::PWRCTL: HXT_SELXT Mask             */

#define CLK_PWRCTL_HXT_GAIN_Pos          (9)                                               /*!< CLK_T::PWRCTL: HXT_GAIN Position          */
#define CLK_PWRCTL_HXT_GAIN_Msk          (0x1ul << CLK_PWRCTL_HXT_GAIN_Pos)                /*!< CLK_T::PWRCTL: HXT_GAIN Mask              */

#define CLK_PWRCTL_LXT_SCNT_Pos          (10)                                              /*!< CLK_T::PWRCTL: LXT_SCNT Position          */
#define CLK_PWRCTL_LXT_SCNT_Msk          (0x1ul << CLK_PWRCTL_LXT_SCNT_Pos)                /*!< CLK_T::PWRCTL: LXT_SCNT Mask              */

#define CLK_PWRCTL_HXT_HF_ST_Pos         (11)                                              /*!< CLK_T::PWRCTL: HXT_HF_ST Position         */
#define CLK_PWRCTL_HXT_HF_ST_Msk         (0x3ul << CLK_PWRCTL_HXT_HF_ST_Pos)               /*!< CLK_T::PWRCTL: HXT_HF_ST Mask             */

#define CLK_AHBCLK_GPIO_EN_Pos           (0)                                               /*!< CLK_T::AHBCLK: GPIO_EN Position           */
#define CLK_AHBCLK_GPIO_EN_Msk           (0x1ul << CLK_AHBCLK_GPIO_EN_Pos)                 /*!< CLK_T::AHBCLK: GPIO_EN Mask               */

#define CLK_AHBCLK_DMA_EN_Pos            (1)                                               /*!< CLK_T::AHBCLK: DMA_EN Position            */
#define CLK_AHBCLK_DMA_EN_Msk            (0x1ul << CLK_AHBCLK_DMA_EN_Pos)                  /*!< CLK_T::AHBCLK: DMA_EN Mask                */

#define CLK_AHBCLK_ISP_EN_Pos            (2)                                               /*!< CLK_T::AHBCLK: ISP_EN Position            */
#define CLK_AHBCLK_ISP_EN_Msk            (0x1ul << CLK_AHBCLK_ISP_EN_Pos)                  /*!< CLK_T::AHBCLK: ISP_EN Mask                */

#define CLK_AHBCLK_EBI_EN_Pos            (3)                                               /*!< CLK_T::AHBCLK: EBI_EN Position            */
#define CLK_AHBCLK_EBI_EN_Msk            (0x1ul << CLK_AHBCLK_EBI_EN_Pos)                  /*!< CLK_T::AHBCLK: EBI_EN Mask                */

#define CLK_AHBCLK_SRAM_EN_Pos           (4)                                               /*!< CLK_T::AHBCLK: SRAM_EN Position           */
#define CLK_AHBCLK_SRAM_EN_Msk           (0x1ul << CLK_AHBCLK_SRAM_EN_Pos)                 /*!< CLK_T::AHBCLK: SRAM_EN Mask               */

#define CLK_AHBCLK_TICK_EN_Pos           (5)                                               /*!< CLK_T::AHBCLK: TICK_EN Position           */
#define CLK_AHBCLK_TICK_EN_Msk           (0x1ul << CLK_AHBCLK_TICK_EN_Pos)                 /*!< CLK_T::AHBCLK: TICK_EN Mask               */

#define CLK_APBCLK_WDT_EN_Pos            (0)                                               /*!< CLK_T::APBCLK: WDT_EN Position            */
#define CLK_APBCLK_WDT_EN_Msk            (0x1ul << CLK_APBCLK_WDT_EN_Pos)                  /*!< CLK_T::APBCLK: WDT_EN Mask                */

#define CLK_APBCLK_RTC_EN_Pos            (1)                                               /*!< CLK_T::APBCLK: RTC_EN Position            */
#define CLK_APBCLK_RTC_EN_Msk            (0x1ul << CLK_APBCLK_RTC_EN_Pos)                  /*!< CLK_T::APBCLK: RTC_EN Mask                */

#define CLK_APBCLK_TMR0_EN_Pos           (2)                                               /*!< CLK_T::APBCLK: TMR0_EN Position           */
#define CLK_APBCLK_TMR0_EN_Msk           (0x1ul << CLK_APBCLK_TMR0_EN_Pos)                 /*!< CLK_T::APBCLK: TMR0_EN Mask               */

#define CLK_APBCLK_TMR1_EN_Pos           (3)                                               /*!< CLK_T::APBCLK: TMR1_EN Position           */
#define CLK_APBCLK_TMR1_EN_Msk           (0x1ul << CLK_APBCLK_TMR1_EN_Pos)                 /*!< CLK_T::APBCLK: TMR1_EN Mask               */

#define CLK_APBCLK_TMR2_EN_Pos           (4)                                               /*!< CLK_T::APBCLK: TMR2_EN Position           */
#define CLK_APBCLK_TMR2_EN_Msk           (0x1ul << CLK_APBCLK_TMR2_EN_Pos)                 /*!< CLK_T::APBCLK: TMR2_EN Mask               */

#define CLK_APBCLK_TMR3_EN_Pos           (5)                                               /*!< CLK_T::APBCLK: TMR3_EN Position           */
#define CLK_APBCLK_TMR3_EN_Msk           (0x1ul << CLK_APBCLK_TMR3_EN_Pos)                 /*!< CLK_T::APBCLK: TMR3_EN Mask               */

#define CLK_APBCLK_FDIV_EN_Pos           (6)                                               /*!< CLK_T::APBCLK: FDIV_EN Position           */
#define CLK_APBCLK_FDIV_EN_Msk           (0x1ul << CLK_APBCLK_FDIV_EN_Pos)                 /*!< CLK_T::APBCLK: FDIV_EN Mask               */

#define CLK_APBCLK_SC2_EN_Pos            (7)                                               /*!< CLK_T::APBCLK: SC2_EN Position            */
#define CLK_APBCLK_SC2_EN_Msk            (0x1ul << CLK_APBCLK_SC2_EN_Pos)                  /*!< CLK_T::APBCLK: SC2_EN Mask                */

#define CLK_APBCLK_I2C0_EN_Pos           (8)                                               /*!< CLK_T::APBCLK: I2C0_EN Position           */
#define CLK_APBCLK_I2C0_EN_Msk           (0x1ul << CLK_APBCLK_I2C0_EN_Pos)                 /*!< CLK_T::APBCLK: I2C0_EN Mask               */

#define CLK_APBCLK_I2C1_EN_Pos           (9)                                               /*!< CLK_T::APBCLK: I2C1_EN Position           */
#define CLK_APBCLK_I2C1_EN_Msk           (0x1ul << CLK_APBCLK_I2C1_EN_Pos)                 /*!< CLK_T::APBCLK: I2C1_EN Mask               */

#define CLK_APBCLK_SPI0_EN_Pos           (12)                                              /*!< CLK_T::APBCLK: SPI0_EN Position           */
#define CLK_APBCLK_SPI0_EN_Msk           (0x1ul << CLK_APBCLK_SPI0_EN_Pos)                 /*!< CLK_T::APBCLK: SPI0_EN Mask               */

#define CLK_APBCLK_SPI1_EN_Pos           (13)                                              /*!< CLK_T::APBCLK: SPI1_EN Position           */
#define CLK_APBCLK_SPI1_EN_Msk           (0x1ul << CLK_APBCLK_SPI1_EN_Pos)                 /*!< CLK_T::APBCLK: SPI1_EN Mask               */

#define CLK_APBCLK_SPI2_EN_Pos           (14)                                              /*!< CLK_T::APBCLK: SPI2_EN Position           */
#define CLK_APBCLK_SPI2_EN_Msk           (0x1ul << CLK_APBCLK_SPI2_EN_Pos)                 /*!< CLK_T::APBCLK: SPI2_EN Mask               */

#define CLK_APBCLK_UART0_EN_Pos          (16)                                              /*!< CLK_T::APBCLK: UART0_EN Position          */
#define CLK_APBCLK_UART0_EN_Msk          (0x1ul << CLK_APBCLK_UART0_EN_Pos)                /*!< CLK_T::APBCLK: UART0_EN Mask              */

#define CLK_APBCLK_UART1_EN_Pos          (17)                                              /*!< CLK_T::APBCLK: UART1_EN Position          */
#define CLK_APBCLK_UART1_EN_Msk          (0x1ul << CLK_APBCLK_UART1_EN_Pos)                /*!< CLK_T::APBCLK: UART1_EN Mask              */

#define CLK_APBCLK_PWM0_CH01_EN_Pos      (20)                                              /*!< CLK_T::APBCLK: PWM0_CH01_EN Position      */
#define CLK_APBCLK_PWM0_CH01_EN_Msk      (0x1ul << CLK_APBCLK_PWM0_CH01_EN_Pos)            /*!< CLK_T::APBCLK: PWM0_CH01_EN Mask          */

#define CLK_APBCLK_PWM0_CH23_EN_Pos      (21)                                              /*!< CLK_T::APBCLK: PWM0_CH23_EN Position      */
#define CLK_APBCLK_PWM0_CH23_EN_Msk      (0x1ul << CLK_APBCLK_PWM0_CH23_EN_Pos)            /*!< CLK_T::APBCLK: PWM0_CH23_EN Mask          */

#define CLK_APBCLK_PWM1_CH01_EN_Pos      (22)                                              /*!< CLK_T::APBCLK: PWM1_CH01_EN Position      */
#define CLK_APBCLK_PWM1_CH01_EN_Msk      (0x1ul << CLK_APBCLK_PWM1_CH01_EN_Pos)            /*!< CLK_T::APBCLK: PWM1_CH01_EN Mask          */

#define CLK_APBCLK_PWM1_CH23_EN_Pos      (23)                                              /*!< CLK_T::APBCLK: PWM1_CH23_EN Position      */
#define CLK_APBCLK_PWM1_CH23_EN_Msk      (0x1ul << CLK_APBCLK_PWM1_CH23_EN_Pos)            /*!< CLK_T::APBCLK: PWM1_CH23_EN Mask          */

#define CLK_APBCLK_DAC_EN_Pos            (25)                                              /*!< CLK_T::APBCLK: DAC_EN Position            */
#define CLK_APBCLK_DAC_EN_Msk            (0x1ul << CLK_APBCLK_DAC_EN_Pos)                  /*!< CLK_T::APBCLK: DAC_EN Mask                */

#define CLK_APBCLK_LCD_EN_Pos            (26)                                              /*!< CLK_T::APBCLK: LCD_EN Position            */
#define CLK_APBCLK_LCD_EN_Msk            (0x1ul << CLK_APBCLK_LCD_EN_Pos)                  /*!< CLK_T::APBCLK: LCD_EN Mask                */

#define CLK_APBCLK_USBD_EN_Pos           (27)                                              /*!< CLK_T::APBCLK: USBD_EN Position           */
#define CLK_APBCLK_USBD_EN_Msk           (0x1ul << CLK_APBCLK_USBD_EN_Pos)                 /*!< CLK_T::APBCLK: USBD_EN Mask               */

#define CLK_APBCLK_ADC_EN_Pos            (28)                                              /*!< CLK_T::APBCLK: ADC_EN Position            */
#define CLK_APBCLK_ADC_EN_Msk            (0x1ul << CLK_APBCLK_ADC_EN_Pos)                  /*!< CLK_T::APBCLK: ADC_EN Mask                */

#define CLK_APBCLK_I2S_EN_Pos            (29)                                              /*!< CLK_T::APBCLK: I2S_EN Position            */
#define CLK_APBCLK_I2S_EN_Msk            (0x1ul << CLK_APBCLK_I2S_EN_Pos)                  /*!< CLK_T::APBCLK: I2S_EN Mask                */

#define CLK_APBCLK_SC0_EN_Pos            (30)                                              /*!< CLK_T::APBCLK: SC0_EN Position            */
#define CLK_APBCLK_SC0_EN_Msk            (0x1ul << CLK_APBCLK_SC0_EN_Pos)                  /*!< CLK_T::APBCLK: SC0_EN Mask                */

#define CLK_APBCLK_SC1_EN_Pos            (31)                                              /*!< CLK_T::APBCLK: SC1_EN Position            */
#define CLK_APBCLK_SC1_EN_Msk            (0x1ul << CLK_APBCLK_SC1_EN_Pos)                  /*!< CLK_T::APBCLK: SC1_EN Mask                */

#define CLK_CLKSTATUS_HXT_STB_Pos        (0)                                               /*!< CLK_T::CLKSTATUS: HXT_STB Position        */
#define CLK_CLKSTATUS_HXT_STB_Msk        (0x1ul << CLK_CLKSTATUS_HXT_STB_Pos)              /*!< CLK_T::CLKSTATUS: HXT_STB Mask            */

#define CLK_CLKSTATUS_LXT_STB_Pos        (1)                                               /*!< CLK_T::CLKSTATUS: LXT_STB Position        */
#define CLK_CLKSTATUS_LXT_STB_Msk        (0x1ul << CLK_CLKSTATUS_LXT_STB_Pos)              /*!< CLK_T::CLKSTATUS: LXT_STB Mask            */

#define CLK_CLKSTATUS_PLL_STB_Pos        (2)                                               /*!< CLK_T::CLKSTATUS: PLL_STB Position        */
#define CLK_CLKSTATUS_PLL_STB_Msk        (0x1ul << CLK_CLKSTATUS_PLL_STB_Pos)              /*!< CLK_T::CLKSTATUS: PLL_STB Mask            */

#define CLK_CLKSTATUS_LIRC_STB_Pos       (3)                                               /*!< CLK_T::CLKSTATUS: LIRC_STB Position       */
#define CLK_CLKSTATUS_LIRC_STB_Msk       (0x1ul << CLK_CLKSTATUS_LIRC_STB_Pos)             /*!< CLK_T::CLKSTATUS: LIRC_STB Mask           */

#define CLK_CLKSTATUS_HIRC_STB_Pos       (4)                                               /*!< CLK_T::CLKSTATUS: HIRC_STB Position       */
#define CLK_CLKSTATUS_HIRC_STB_Msk       (0x1ul << CLK_CLKSTATUS_HIRC_STB_Pos)             /*!< CLK_T::CLKSTATUS: HIRC_STB Mask           */

#define CLK_CLKSTATUS_CLK_SW_FAIL_Pos    (7)                                               /*!< CLK_T::CLKSTATUS: CLK_SW_FAIL Position    */
#define CLK_CLKSTATUS_CLK_SW_FAIL_Msk    (0x1ul << CLK_CLKSTATUS_CLK_SW_FAIL_Pos)          /*!< CLK_T::CLKSTATUS: CLK_SW_FAIL Mask        */

#define CLK_CLKSEL0_HCLK_S_Pos           (0)                                               /*!< CLK_T::CLKSEL0: HCLK_S Position           */
#define CLK_CLKSEL0_HCLK_S_Msk           (0x7ul << CLK_CLKSEL0_HCLK_S_Pos)                 /*!< CLK_T::CLKSEL0: HCLK_S Mask               */

#define CLK_CLKSEL1_UART_S_Pos           (0)                                               /*!< CLK_T::CLKSEL1: UART_S Position           */
#define CLK_CLKSEL1_UART_S_Msk           (0x3ul << CLK_CLKSEL1_UART_S_Pos)                 /*!< CLK_T::CLKSEL1: UART_S Mask               */

#define CLK_CLKSEL1_ADC_S_Pos            (2)                                               /*!< CLK_T::CLKSEL1: ADC_S Position            */
#define CLK_CLKSEL1_ADC_S_Msk            (0x3ul << CLK_CLKSEL1_ADC_S_Pos)                  /*!< CLK_T::CLKSEL1: ADC_S Mask                */

#define CLK_CLKSEL1_PWM0_CH01_S_Pos      (4)                                               /*!< CLK_T::CLKSEL1: PWM0_CH01_S Position      */
#define CLK_CLKSEL1_PWM0_CH01_S_Msk      (0x3ul << CLK_CLKSEL1_PWM0_CH01_S_Pos)            /*!< CLK_T::CLKSEL1: PWM0_CH01_S Mask          */

#define CLK_CLKSEL1_PWM0_CH23_S_Pos      (6)                                               /*!< CLK_T::CLKSEL1: PWM0_CH23_S Position      */
#define CLK_CLKSEL1_PWM0_CH23_S_Msk      (0x3ul << CLK_CLKSEL1_PWM0_CH23_S_Pos)            /*!< CLK_T::CLKSEL1: PWM0_CH23_S Mask          */

#define CLK_CLKSEL1_TMR0_S_Pos           (8)                                               /*!< CLK_T::CLKSEL1: TMR0_S Position           */
#define CLK_CLKSEL1_TMR0_S_Msk           (0x7ul << CLK_CLKSEL1_TMR0_S_Pos)                 /*!< CLK_T::CLKSEL1: TMR0_S Mask               */

#define CLK_CLKSEL1_TMR1_S_Pos           (12)                                              /*!< CLK_T::CLKSEL1: TMR1_S Position           */
#define CLK_CLKSEL1_TMR1_S_Msk           (0x7ul << CLK_CLKSEL1_TMR1_S_Pos)                 /*!< CLK_T::CLKSEL1: TMR1_S Mask               */

#define CLK_CLKSEL1_LCD_S_Pos            (18)                                              /*!< CLK_T::CLKSEL1: LCD_S Position            */
#define CLK_CLKSEL1_LCD_S_Msk            (0x1ul << CLK_CLKSEL1_LCD_S_Pos)                  /*!< CLK_T::CLKSEL1: LCD_S Mask                */

#define CLK_CLKSEL2_FRQDIV_S_Pos         (2)                                               /*!< CLK_T::CLKSEL2: FRQDIV_S Position         */
#define CLK_CLKSEL2_FRQDIV_S_Msk         (0x3ul << CLK_CLKSEL2_FRQDIV_S_Pos)               /*!< CLK_T::CLKSEL2: FRQDIV_S Mask             */

#define CLK_CLKSEL2_PWM1_CH01_S_Pos      (4)                                               /*!< CLK_T::CLKSEL2: PWM1_CH01_S Position      */
#define CLK_CLKSEL2_PWM1_CH01_S_Msk      (0x3ul << CLK_CLKSEL2_PWM1_CH01_S_Pos)            /*!< CLK_T::CLKSEL2: PWM1_CH01_S Mask          */

#define CLK_CLKSEL2_PWM1_CH23_S_Pos      (6)                                               /*!< CLK_T::CLKSEL2: PWM1_CH23_S Position      */
#define CLK_CLKSEL2_PWM1_CH23_S_Msk      (0x3ul << CLK_CLKSEL2_PWM1_CH23_S_Pos)            /*!< CLK_T::CLKSEL2: PWM1_CH23_S Mask          */

#define CLK_CLKSEL2_TMR2_S_Pos           (8)                                               /*!< CLK_T::CLKSEL2: TMR2_S Position           */
#define CLK_CLKSEL2_TMR2_S_Msk           (0x7ul << CLK_CLKSEL2_TMR2_S_Pos)                 /*!< CLK_T::CLKSEL2: TMR2_S Mask               */

#define CLK_CLKSEL2_TMR3_S_Pos           (12)                                              /*!< CLK_T::CLKSEL2: TMR3_S Position           */
#define CLK_CLKSEL2_TMR3_S_Msk           (0x7ul << CLK_CLKSEL2_TMR3_S_Pos)                 /*!< CLK_T::CLKSEL2: TMR3_S Mask               */

#define CLK_CLKSEL2_I2S_S_Pos            (16)                                              /*!< CLK_T::CLKSEL2: I2S_S Position            */
#define CLK_CLKSEL2_I2S_S_Msk            (0x3ul << CLK_CLKSEL2_I2S_S_Pos)                  /*!< CLK_T::CLKSEL2: I2S_S Mask                */

#define CLK_CLKSEL2_SC_S_Pos             (18)                                              /*!< CLK_T::CLKSEL2: SC_S Position             */
#define CLK_CLKSEL2_SC_S_Msk             (0x3ul << CLK_CLKSEL2_SC_S_Pos)                   /*!< CLK_T::CLKSEL2: SC_S Mask                 */

#define CLK_CLKSEL2_SPI0_S_Pos           (20)                                              /*!< CLK_T::CLKSEL2: SPI0_S Position           */
#define CLK_CLKSEL2_SPI0_S_Msk           (0x1ul << CLK_CLKSEL2_SPI0_S_Pos)                 /*!< CLK_T::CLKSEL2: SPI0_S Mask               */

#define CLK_CLKSEL2_SPI1_S_Pos           (21)                                              /*!< CLK_T::CLKSEL2: SPI1_S Position           */
#define CLK_CLKSEL2_SPI1_S_Msk           (0x1ul << CLK_CLKSEL2_SPI1_S_Pos)                 /*!< CLK_T::CLKSEL2: SPI1_S Mask               */

#define CLK_CLKSEL2_SPI2_S_Pos           (22)                                              /*!< CLK_T::CLKSEL2: SPI2_S Position           */
#define CLK_CLKSEL2_SPI2_S_Msk           (0x1ul << CLK_CLKSEL2_SPI2_S_Pos)                 /*!< CLK_T::CLKSEL2: SPI2_S Mask               */

#define CLK_CLKDIV0_HCLK_N_Pos           (0)                                               /*!< CLK_T::CLKDIV0: HCLK_N Position           */
#define CLK_CLKDIV0_HCLK_N_Msk           (0xful << CLK_CLKDIV0_HCLK_N_Pos)                 /*!< CLK_T::CLKDIV0: HCLK_N Mask               */

#define CLK_CLKDIV0_USB_N_Pos            (4)                                               /*!< CLK_T::CLKDIV0: USB_N Position            */
#define CLK_CLKDIV0_USB_N_Msk            (0xful << CLK_CLKDIV0_USB_N_Pos)                  /*!< CLK_T::CLKDIV0: USB_N Mask                */

#define CLK_CLKDIV0_UART_N_Pos           (8)                                               /*!< CLK_T::CLKDIV0: UART_N Position           */
#define CLK_CLKDIV0_UART_N_Msk           (0xful << CLK_CLKDIV0_UART_N_Pos)                 /*!< CLK_T::CLKDIV0: UART_N Mask               */

#define CLK_CLKDIV0_I2S_N_Pos            (12)                                              /*!< CLK_T::CLKDIV0: I2S_N Position            */
#define CLK_CLKDIV0_I2S_N_Msk            (0xful << CLK_CLKDIV0_I2S_N_Pos)                  /*!< CLK_T::CLKDIV0: I2S_N Mask                */

#define CLK_CLKDIV0_ADC_N_Pos            (16)                                              /*!< CLK_T::CLKDIV0: ADC_N Position            */
#define CLK_CLKDIV0_ADC_N_Msk            (0xfful << CLK_CLKDIV0_ADC_N_Pos)                 /*!< CLK_T::CLKDIV0: ADC_N Mask                */

#define CLK_CLKDIV0_SC0_N_Pos            (28)                                              /*!< CLK_T::CLKDIV0: SC0_N Position            */
#define CLK_CLKDIV0_SC0_N_Msk            (0xful << CLK_CLKDIV0_SC0_N_Pos)                  /*!< CLK_T::CLKDIV0: SC0_N Mask                */

#define CLK_CLKDIV1_SC1_N_Pos            (0)                                               /*!< CLK_T::CLKDIV1: SC1_N Position            */
#define CLK_CLKDIV1_SC1_N_Msk            (0xful << CLK_CLKDIV1_SC1_N_Pos)                  /*!< CLK_T::CLKDIV1: SC1_N Mask                */

#define CLK_CLKDIV1_SC2_N_Pos            (4)                                               /*!< CLK_T::CLKDIV1: SC2_N Position            */
#define CLK_CLKDIV1_SC2_N_Msk            (0xful << CLK_CLKDIV1_SC2_N_Pos)                  /*!< CLK_T::CLKDIV1: SC2_N Mask                */

#define CLK_PLLCTL_FB_DV_Pos             (0)                                               /*!< CLK_T::PLLCTL: FB_DV Position             */
#define CLK_PLLCTL_FB_DV_Msk             (0x3ful << CLK_PLLCTL_FB_DV_Pos)                  /*!< CLK_T::PLLCTL: FB_DV Mask                 */

#define CLK_PLLCTL_IN_DV_Pos             (8)                                               /*!< CLK_T::PLLCTL: IN_DV Position             */
#define CLK_PLLCTL_IN_DV_Msk             (0x3ul << CLK_PLLCTL_IN_DV_Pos)                   /*!< CLK_T::PLLCTL: IN_DV Mask                 */

#define CLK_PLLCTL_OUT_DV_Pos            (12)                                              /*!< CLK_T::PLLCTL: OUT_DV Position            */
#define CLK_PLLCTL_OUT_DV_Msk            (0x1ul << CLK_PLLCTL_OUT_DV_Pos)                  /*!< CLK_T::PLLCTL: OUT_DV Mask                */

#define CLK_PLLCTL_PD_Pos                (16)                                              /*!< CLK_T::PLLCTL: PD Position                */
#define CLK_PLLCTL_PD_Msk                (0x1ul << CLK_PLLCTL_PD_Pos)                      /*!< CLK_T::PLLCTL: PD Mask                    */

#define CLK_PLLCTL_PLL_SRC_Pos           (17)                                              /*!< CLK_T::PLLCTL: PLL_SRC Position           */
#define CLK_PLLCTL_PLL_SRC_Msk           (0x1ul << CLK_PLLCTL_PLL_SRC_Pos)                 /*!< CLK_T::PLLCTL: PLL_SRC Mask               */

#define CLK_FRQDIV_FSEL_Pos              (0)                                               /*!< CLK_T::FRQDIV: FSEL Position              */
#define CLK_FRQDIV_FSEL_Msk              (0xful << CLK_FRQDIV_FSEL_Pos)                    /*!< CLK_T::FRQDIV: FSEL Mask                  */

#define CLK_FRQDIV_FDIV_EN_Pos           (4)                                               /*!< CLK_T::FRQDIV: FDIV_EN Position           */
#define CLK_FRQDIV_FDIV_EN_Msk           (0x1ul << CLK_FRQDIV_FDIV_EN_Pos)                 /*!< CLK_T::FRQDIV: FDIV_EN Mask               */

#define CLK_MCLKO_MCLK_SEL_Pos           (0)                                               /*!< CLK_T::MCLKO: MCLK_SEL Position           */
#define CLK_MCLKO_MCLK_SEL_Msk           (0x3ful << CLK_MCLKO_MCLK_SEL_Pos)                /*!< CLK_T::MCLKO: MCLK_SEL Mask               */

#define CLK_MCLKO_MCLK_EN_Pos            (7)                                               /*!< CLK_T::MCLKO: MCLK_EN Position            */
#define CLK_MCLKO_MCLK_EN_Msk            (0x1ul << CLK_MCLKO_MCLK_EN_Pos)                  /*!< CLK_T::MCLKO: MCLK_EN Mask                */

#define CLK_WK_INTSTS_PD_WK_IS_Pos       (0)                                               /*!< CLK_T::WK_INTSTS: PD_WK_IS Position          */
#define CLK_WK_INTSTS_PD_WK_IS_Msk       (0x1ul << CLK_WK_INTSTS_PD_WK_IS_Pos)             /*!< CLK_T::WK_INTSTS: PD_WK_IS Mask              */

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */


/*---------------------- Digital to Analog Converter -------------------------*/
/**
    @addtogroup DAC Digital to Analog Converter(DAC)
    Memory Mapped Structure for DAC Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTL0</font><br><p> <font size="2">
Offset: 0x00  DAC0 Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>DACEN</td><td><div style="word-wrap: break-word;"><b>DAC Enable</b><br>
0 = Power down DAC.<br>
1 = Power on DAC.<br>
Note: When DAC is powered on, DAC will automatically start conversion after waiting for DACPWONSTBCNT+1 PCLK cycle<br>
</div></td></tr><tr><td>
[1]</td><td>DACIE</td><td><div style="word-wrap: break-word;"><b>DAC Interrupt Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[6:4]</td><td>DACLSEL</td><td><div style="word-wrap: break-word;"><b>DAC Load Selection</b><br>
Select the load trigger for the DAC latch.<br>
000 = DAC latch loads when DACx_DAT written<br>
001 = PDMA ACK<br>
010 = Rising edge of TMR0<br>
011 = Rising edge of TMR1<br>
100 = Rising edge of TMR2<br>
101 = Rising edge of TMR3<br>
Others = Reserved<br>
</div></td></tr><tr><td>
[21:8]</td><td>DACPWONSTBCNT</td><td><div style="word-wrap: break-word;"><b>DACPWONSTBCNT</b><br>
DAC need 6 us to be stable after DAC is power on from power down state.<br>
This field controls a internal counter (in PCLK unit) to guarantee DAC stable time requirement.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTL0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DATA0</font><br><p> <font size="2">
Offset: 0x04  DAC0 Data Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[11:0]</td><td>DACData</td><td><div style="word-wrap: break-word;"><b>DAC data</b><br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DATA0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">STS0</font><br><p> <font size="2">
Offset: 0x08  DAC0 Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>DACIFG</td><td><div style="word-wrap: break-word;"><b>DAC Interrupt Flag</b><br>
0 = No interrupt pending.<br>
1 = Interrupt pending.<br>
Note: This bit is read only.<br>
</div></td></tr><tr><td>
[1]</td><td>DACSTFG</td><td><div style="word-wrap: break-word;"><b>DAC Start Flag</b><br>
0 = DAC is not start yet.<br>
1 = DAC has been started.<br>
Note: this bit is read only.<br>
</div></td></tr><tr><td>
[2]</td><td>BUSY</td><td><div style="word-wrap: break-word;"><b>BUSY Bit</b><br>
0 = DAC is not busy.<br>
1 = DAC is busy.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t STS0;
    uint32_t RESERVE0[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTL1</font><br><p> <font size="2">
Offset: 0x10  DAC1 Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>DACEN</td><td><div style="word-wrap: break-word;"><b>DAC Enable</b><br>
0 = Power down DAC.<br>
1 = Power on DAC.<br>
Note: When DAC is powered on, DAC will automatically start conversion after waiting for DACPWONSTBCNT+1 PCLK cycle<br>
</div></td></tr><tr><td>
[1]</td><td>DACIE</td><td><div style="word-wrap: break-word;"><b>DAC Interrupt Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[6:4]</td><td>DACLSEL</td><td><div style="word-wrap: break-word;"><b>DAC Load Selection</b><br>
Select the load trigger for the DAC latch.<br>
000 = DAC latch loads when DACx_DAT written<br>
001 = PDMA ACK<br>
010 = Rising edge of TMR0<br>
011 = Rising edge of TMR1<br>
100 = Rising edge of TMR2<br>
101 = Rising edge of TMR3<br>
Others = Reserved<br>
</div></td></tr><tr><td>
[21:8]</td><td>DACPWONSTBCNT</td><td><div style="word-wrap: break-word;"><b>DACPWONSTBCNT</b><br>
DAC need 6 us to be stable after DAC is power on from power down state.<br>
This field controls a internal counter (in PCLK unit) to guarantee DAC stable time requirement.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTL1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DATA1</font><br><p> <font size="2">
Offset: 0x14  DAC1 Data Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[11:0]</td><td>DACData</td><td><div style="word-wrap: break-word;"><b>DAC data</b><br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DATA1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">STS1</font><br><p> <font size="2">
Offset: 0x18  DAC1 Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>DACIFG</td><td><div style="word-wrap: break-word;"><b>DAC Interrupt Flag</b><br>
0 = No interrupt pending.<br>
1 = Interrupt pending.<br>
Note: This bit is read only.<br>
</div></td></tr><tr><td>
[1]</td><td>DACSTFG</td><td><div style="word-wrap: break-word;"><b>DAC Start Flag</b><br>
0 = DAC is not start yet.<br>
1 = DAC has been started.<br>
Note: this bit is read only.<br>
</div></td></tr><tr><td>
[2]</td><td>BUSY</td><td><div style="word-wrap: break-word;"><b>BUSY Bit</b><br>
0 = DAC is not busy.<br>
1 = DAC is busy.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t STS1;
    uint32_t RESERVE1[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">COMCTL</font><br><p> <font size="2">
Offset: 0x20  DAC01 Common Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>WAITDACCONV</td><td><div style="word-wrap: break-word;"><b>Wait DAC Conversion Complete</b><br>
The DAC needs at least 2 us to settle down every time when each data deliver to DAC, which means user cannot update each DACx_data register faster than 2 us; otherwise data will lost.<br>
Setting this register can adjust the time interval in PCLK unit between each DACx_data into DAC in order to meet the 2 us requirement.<br>
</div></td></tr><tr><td>
[8]</td><td>DAC01GRP</td><td><div style="word-wrap: break-word;"><b>Group DAC0 And DAC1</b><br>
0 = Not grouped.<br>
1 = Grouped.<br>
</div></td></tr><tr><td>
[10:9]</td><td>REFSEL</td><td><div style="word-wrap: break-word;"><b>Reference Voltage Selection</b><br>
00 = AVDD<br>
01 = Internal reference voltage<br>
10 = External reference voltage<br>
11= Reserved<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t COMCTL;

} DAC_T;

/**
    @addtogroup DAC_CONST DAC Bit Field Definition
    Constant Definitions for DAC Controller
@{ */

#define DAC_CTL_DACEN_Pos                (0)                                               /*!< DAC_T::CTL: DACEN Position                */
#define DAC_CTL_DACEN_Msk                (0x1ul << DAC_CTL_DACEN_Pos)                      /*!< DAC_T::CTL: DACEN Mask                    */

#define DAC_CTL_DACIE_Pos                (1)                                               /*!< DAC_T::CTL: DACIE Position                */
#define DAC_CTL_DACIE_Msk                (0x1ul << DAC_CTL_DACIE_Pos)                      /*!< DAC_T::CTL: DACIE Mask                    */

#define DAC_CTL_DACLSEL_Pos              (4)                                               /*!< DAC_T::CTL: DACLSEL Position              */
#define DAC_CTL_DACLSEL_Msk              (0x7ul << DAC_CTL_DACLSEL_Pos)                    /*!< DAC_T::CTL: DACLSEL Mask                  */

#define DAC_CTL_DACPWONSTBCNT_Pos        (8)                                               /*!< DAC_T::CTL: DACPWONSTBCNT Position        */
#define DAC_CTL_DACPWONSTBCNT_Msk        (0x3ffful << DAC_CTL_DACPWONSTBCNT_Pos)           /*!< DAC_T::CTL: DACPWONSTBCNT Mask            */

#define DAC_DATA_DACData_Pos             (0)                                               /*!< DAC_T::DATA: DACData Position             */
#define DAC_DATA_DACData_Msk             (0xffful << DAC_DATA_DACData_Pos)                 /*!< DAC_T::DATA: DACData Mask                 */

#define DAC_STS_DACIFG_Pos               (0)                                               /*!< DAC_T::STS: DACIFG Position               */
#define DAC_STS_DACIFG_Msk               (0x1ul << DAC_STS_DACIFG_Pos)                     /*!< DAC_T::STS: DACIFG Mask                   */

#define DAC_STS_DACSTFG_Pos              (1)                                               /*!< DAC_T::STS: DACSTFG Position              */
#define DAC_STS_DACSTFG_Msk              (0x1ul << DAC_STS_DACSTFG_Pos)                    /*!< DAC_T::STS: DACSTFG Mask                  */

#define DAC_STS_BUSY_Pos                 (2)                                               /*!< DAC_T::STS: BUSY Position                 */
#define DAC_STS_BUSY_Msk                 (0x1ul << DAC_STS_BUSY_Pos)                       /*!< DAC_T::STS: BUSY Mask                     */

#define DAC_COMCTL_WAITDACCONV_Pos       (0)                                               /*!< DAC_T::COMCTL: WAITDACCONV Position       */
#define DAC_COMCTL_WAITDACCONV_Msk       (0xfful << DAC_COMCTL_WAITDACCONV_Pos)            /*!< DAC_T::COMCTL: WAITDACCONV Mask           */

#define DAC_COMCTL_DAC01GRP_Pos          (8)                                               /*!< DAC_T::COMCTL: DAC01GRP Position          */
#define DAC_COMCTL_DAC01GRP_Msk          (0x1ul << DAC_COMCTL_DAC01GRP_Pos)                /*!< DAC_T::COMCTL: DAC01GRP Mask              */

#define DAC_COMCTL_REFSEL_Pos            (9)                                               /*!< DAC_T::COMCTL: REFSEL Position            */
#define DAC_COMCTL_REFSEL_Msk            (0x3ul << DAC_COMCTL_REFSEL_Pos)                  /*!< DAC_T::COMCTL: REFSEL Mask                */

/**@}*/ /* DAC_CONST */
/**@}*/ /* end of DAC register group */


/*---------------------- External Bus Interface Controller -------------------------*/
/**
    @addtogroup EBI External Bus Interface Controller(EBI)
    Memory Mapped Structure for EBI Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">EBICON</font><br><p> <font size="2">
Offset: 0x00  External Bus Interface General Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>ExtEN</td><td><div style="word-wrap: break-word;"><b>EBI Enable</b><br>
This bit is the functional enable bit for EBI.<br>
0 = EBI function is disabled.<br>
1 = EBI function is enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>ExtBW16</td><td><div style="word-wrap: break-word;"><b>EBI Data Width 16-Bit</b><br>
This bit defines if the data bus is 8-bit or 16-bit.<br>
0 = EBI data width is 8-bit.<br>
1 = EBI data width is 16-bit.<br>
</div></td></tr><tr><td>
[10:8]</td><td>MCLKDIV</td><td><div style="word-wrap: break-word;"><b>External Output Clock Divider</b><br>
The frequency of EBI output clock is controlled by MCLKDIV as shown in the following table.<br>
000 = HCLK/1.<br>
001 = HCLK/2.<br>
010 = HCLK/4.<br>
011 = HCLK/8.<br>
100 = HCLK/16.<br>
101 = HCLK/32.<br>
110 = Default.<br>
111 = Default.<br>
Notice: Default value of output clock is HCLK/1<br>
</div></td></tr><tr><td>
[11]</td><td>MCLKEN</td><td><div style="word-wrap: break-word;"><b>External Clock Enable</b><br>
This bit control if EBI generates the clock to external device.<br>
If external device is a synchronous device, it's necessary to set this bit high to enable EBI generating clock to external device.<br>
If the external device is an asynchronous device, keep this bit low is recommended to save power consumption.<br>
0 = EBI Disabled to generate clock to external device.<br>
1 = EBI Enabled to generate clock to external device.<br>
</div></td></tr><tr><td>
[18:16]</td><td>ExttALE</td><td><div style="word-wrap: break-word;"><b>Expand Time Of ALE</b><br>
The ALE width (tALE) to latch the address can be controlled by ExttALE.<br>
tALE = (ExttALE+1)*MCLK.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t EBICON;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">EXTIME</font><br><p> <font size="2">
Offset: 0x04  External Bus Interface Timing Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[4:0]</td><td>ExttACC</td><td><div style="word-wrap: break-word;"><b>EBI Data Access Time</b><br>
ExttACC define data access time (tACC).<br>
tACC = (ExttACC +1) * MCLK.<br>
</div></td></tr><tr><td>
[10:8]</td><td>ExttAHD</td><td><div style="word-wrap: break-word;"><b>EBI Data Access Hold Time</b><br>
ExttAHD define data access hold time (tAHD).<br>
tAHD = (ExttAHD +1) * MCLK.<br>
</div></td></tr><tr><td>
[15:12]</td><td>ExtIW2X</td><td><div style="word-wrap: break-word;"><b>Idle State Cycle After Write</b><br>
When write action is finish, idle state is inserted and nCS return to high if ExtIW2X is not zero.<br>
Idle state cycle = (ExtIW2X*MCLK).<br>
</div></td></tr><tr><td>
[19:16]</td><td>ExtIR2W</td><td><div style="word-wrap: break-word;"><b>Idle State Cycle Between Read-Write</b><br>
When read action is finish and next action is going to write, idle state is inserted and nCS return to high if ExtIR2W is not zero.<br>
Idle state cycle = (ExtIR2W*MCLK).<br>
</div></td></tr><tr><td>
[27:24]</td><td>ExtIR2R</td><td><div style="word-wrap: break-word;"><b>Idle State Cycle Between Read-Read</b><br>
When read action is finish and next action is going to read, idle state is inserted and nCS return to high if ExtIR2R is not zero.<br>
Idle state cycle = (ExtIR2R*MCLK).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t EXTIME;

} EBI_T;

/**
    @addtogroup EBI_CONST EBI Bit Field Definition
    Constant Definitions for EBI Controller
@{ */

#define EBI_EBICON_ExtEN_Pos             (0)                                               /*!< EBI_T::EBICON: ExtEN Position             */
#define EBI_EBICON_ExtEN_Msk             (0x1ul << EBI_EBICON_ExtEN_Pos)                   /*!< EBI_T::EBICON: ExtEN Mask                 */

#define EBI_EBICON_ExtBW16_Pos           (1)                                               /*!< EBI_T::EBICON: ExtBW16 Position           */
#define EBI_EBICON_ExtBW16_Msk           (0x1ul << EBI_EBICON_ExtBW16_Pos)                 /*!< EBI_T::EBICON: ExtBW16 Mask               */

#define EBI_EBICON_MCLKDIV_Pos           (8)                                               /*!< EBI_T::EBICON: MCLKDIV Position           */
#define EBI_EBICON_MCLKDIV_Msk           (0x7ul << EBI_EBICON_MCLKDIV_Pos)                 /*!< EBI_T::EBICON: MCLKDIV Mask               */

#define EBI_EBICON_MCLKEN_Pos            (11)                                              /*!< EBI_T::EBICON: MCLKEN Position            */
#define EBI_EBICON_MCLKEN_Msk            (0x1ul << EBI_EBICON_MCLKEN_Pos)                  /*!< EBI_T::EBICON: MCLKEN Mask                */

#define EBI_EBICON_ExttALE_Pos           (16)                                              /*!< EBI_T::EBICON: ExttALE Position           */
#define EBI_EBICON_ExttALE_Msk           (0x7ul << EBI_EBICON_ExttALE_Pos)                 /*!< EBI_T::EBICON: ExttALE Mask               */

#define EBI_EXTIME_ExttACC_Pos           (0)                                               /*!< EBI_T::EXTIME: ExttACC Position           */
#define EBI_EXTIME_ExttACC_Msk           (0x1ful << EBI_EXTIME_ExttACC_Pos)                /*!< EBI_T::EXTIME: ExttACC Mask               */

#define EBI_EXTIME_ExttAHD_Pos           (8)                                               /*!< EBI_T::EXTIME: ExttAHD Position           */
#define EBI_EXTIME_ExttAHD_Msk           (0x7ul << EBI_EXTIME_ExttAHD_Pos)                 /*!< EBI_T::EXTIME: ExttAHD Mask               */

#define EBI_EXTIME_ExtIW2X_Pos           (12)                                              /*!< EBI_T::EXTIME: ExtIW2X Position           */
#define EBI_EXTIME_ExtIW2X_Msk           (0xful << EBI_EXTIME_ExtIW2X_Pos)                 /*!< EBI_T::EXTIME: ExtIW2X Mask               */

#define EBI_EXTIME_ExtIR2W_Pos           (16)                                              /*!< EBI_T::EXTIME: ExtIR2W Position           */
#define EBI_EXTIME_ExtIR2W_Msk           (0xful << EBI_EXTIME_ExtIR2W_Pos)                 /*!< EBI_T::EXTIME: ExtIR2W Mask               */

#define EBI_EXTIME_ExtIR2R_Pos           (24)                                              /*!< EBI_T::EXTIME: ExtIR2R Position           */
#define EBI_EXTIME_ExtIR2R_Msk           (0xful << EBI_EXTIME_ExtIR2R_Pos)                 /*!< EBI_T::EXTIME: ExtIR2R Mask               */

/**@}*/ /* EBI_CONST */
/**@}*/ /* end of EBI register group */


/*---------------------- Flash Memory Controller -------------------------*/
/**
    @addtogroup FMC Flash Memory Controller(FMC)
    Memory Mapped Structure for FMC Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISPCON</font><br><p> <font size="2">
Offset: 0x00  ISP Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>ISPEN</td><td><div style="word-wrap: break-word;"><b>ISP Enable (Write-Protection Bit)</b><br>
ISP function enable bit. Set this bit to enable ISP function.<br>
0 = ISP function Disabled.<br>
1 = ISP function Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>BS</td><td><div style="word-wrap: break-word;"><b>Boot Select (Write-Protection Bit)</b><br>
Set/clear this bit to select next booting from LDROM/APROM, respectively.<br>
This bit also functions as chip booting status flag, which can be used to check where chip booted from.<br>
This bit is initiated with the inversed value of CBS in Config0 after power-on reset; It keeps the same value at other reset.<br>
0 = boot from APROM.<br>
1 = boot from LDROM.<br>
</div></td></tr><tr><td>
[3]</td><td>APUEN</td><td><div style="word-wrap: break-word;"><b>APROM Update Enable (Write-Protection Bit)</b><br>
APROM update enable bit.<br>
0 = APROM can not be updated.<br>
1 = APROM can be updated when the MCU runs in APROM.<br>
</div></td></tr><tr><td>
[4]</td><td>CFGUEN</td><td><div style="word-wrap: break-word;"><b>Enable Config-Bits Update By ISP (Write-Protection Bit)</b><br>
0 = Disabling ISP can update config-bits.<br>
1 = Enabling ISP can update config-bits.<br>
</div></td></tr><tr><td>
[5]</td><td>LDUEN</td><td><div style="word-wrap: break-word;"><b>LDROM Update Enable (Write-Protection Bit)</b><br>
LDROM update enable bit.<br>
0 = LDROM cannot be updated.<br>
1 = LDROM can be updated when the chip runs in APROM.<br>
</div></td></tr><tr><td>
[6]</td><td>ISPFF</td><td><div style="word-wrap: break-word;"><b>ISP Fail Flag (Write-Protection Bit)</b><br>
This bit is set by hardware when a triggered ISP meets any of the following conditions:<br>
(1) APROM writes to itself<br>
(2) LDROM writes to itself<br>
(3) CONFIG is erased/programmed if CFGUEN is set to 0<br>
(4) Destination address is illegal, such as over an available range<br>
Write 1 to clear.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISPCON;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISPADR</font><br><p> <font size="2">
Offset: 0x04  ISP Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>ISPADR</td><td><div style="word-wrap: break-word;"><b>ISP Address</b><br>
This chip supports word program only.<br>
ISPADR[1:0] must be kept 00b for ISP operation, and ISPADR[8:0] must be kept 0_0000_0000b for Vector Page Re-map Command.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISPADR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISPDAT</font><br><p> <font size="2">
Offset: 0x08  ISP Data Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>ISPDAT</td><td><div style="word-wrap: break-word;"><b>ISP Data</b><br>
Write data to this register before ISP program operation<br>
Read data from this register after ISP read operation<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISPDAT;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISPCMD</font><br><p> <font size="2">
Offset: 0x0C  ISP Command Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>FCTRL</td><td><div style="word-wrap: break-word;"><b>ISP Command</b><br>
The ISP command table is shown as follows.<br>
</div></td></tr><tr><td>
[4]</td><td>FCEN</td><td><div style="word-wrap: break-word;"><b>ISP Command</b><br>
The ISP command table is shown as follows.<br>
</div></td></tr><tr><td>
[5]</td><td>FOEN</td><td><div style="word-wrap: break-word;"><b>ISP Command</b><br>
The ISP command table is shown as follows.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISPCMD;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISPTRG</font><br><p> <font size="2">
Offset: 0x10  ISP Trigger Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>ISPGO</td><td><div style="word-wrap: break-word;"><b>ISP Start Trigger</b><br>
Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.<br>
0 = ISP operation is finished.<br>
1 = ISP is progressing.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISPTRG;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DFBADR</font><br><p> <font size="2">
Offset: 0x14  Data Flash Base Address</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>DFBA</td><td><div style="word-wrap: break-word;"><b>Data Flash Base Address</b><br>
This register indicates data flash start address. It is a read only register.<br>
The data flash start address is defined by user.<br>
Since on chip flash erase unit is 512 bytes, it is mandatory to keep bit 8-0 as 0.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t DFBADR;
    uint32_t RESERVE0[10];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISPSTA</font><br><p> <font size="2">
Offset: 0x40  ISP Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>ISPBUSY</td><td><div style="word-wrap: break-word;"><b>ISP BUSY</b><br>
0 = ISP operation is finished.<br>
1 = ISP operation is busy.<br>
Read Only<br>
</div></td></tr><tr><td>
[2:1]</td><td>CBS</td><td><div style="word-wrap: break-word;"><b>Config Boot Selection Status</b><br>
</div></td></tr><tr><td>
[6]</td><td>ISPFF</td><td><div style="word-wrap: break-word;"><b>ISP Fail Flag</b><br>
This bit is set by hardware when a triggered ISP meets any of the following conditions:<br>
(1) APROM writes to itself.<br>
(2) LDROM writes to itself.<br>
(3) CONFIG is erased/programmed when the MCU is running in APROM.<br>
(4) Destination address is illegal, such as over an available range.<br>
Write 1 to clear.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISPSTA;

} FMC_T;

/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for FMC Controller
@{ */

#define FMC_ISPCON_ISPEN_Pos             (0)                                               /*!< FMC_T::ISPCON: ISPEN Position             */
#define FMC_ISPCON_ISPEN_Msk             (0x1ul << FMC_ISPCON_ISPEN_Pos)                   /*!< FMC_T::ISPCON: ISPEN Mask                 */

#define FMC_ISPCON_BS_Pos                (1)                                               /*!< FMC_T::ISPCON: BS Position                */
#define FMC_ISPCON_BS_Msk                (0x1ul << FMC_ISPCON_BS_Pos)                      /*!< FMC_T::ISPCON: BS Mask                    */

#define FMC_ISPCON_APUEN_Pos             (3)                                               /*!< FMC_T::ISPCON: APUEN Position             */
#define FMC_ISPCON_APUEN_Msk             (0x1ul << FMC_ISPCON_APUEN_Pos)                   /*!< FMC_T::ISPCON: APUEN Mask                 */

#define FMC_ISPCON_CFGUEN_Pos            (4)                                               /*!< FMC_T::ISPCON: CFGUEN Position            */
#define FMC_ISPCON_CFGUEN_Msk            (0x1ul << FMC_ISPCON_CFGUEN_Pos)                  /*!< FMC_T::ISPCON: CFGUEN Mask                */

#define FMC_ISPCON_LDUEN_Pos             (5)                                               /*!< FMC_T::ISPCON: LDUEN Position             */
#define FMC_ISPCON_LDUEN_Msk             (0x1ul << FMC_ISPCON_LDUEN_Pos)                   /*!< FMC_T::ISPCON: LDUEN Mask                 */

#define FMC_ISPCON_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPCON: ISPFF Position             */
#define FMC_ISPCON_ISPFF_Msk             (0x1ul << FMC_ISPCON_ISPFF_Pos)                   /*!< FMC_T::ISPCON: ISPFF Mask                 */

#define FMC_ISPADR_ISPADR_Pos            (0)                                               /*!< FMC_T::ISPADR: ISPADR Position            */
#define FMC_ISPADR_ISPADR_Msk            (0xfffffffful << FMC_ISPADR_ISPADR_Pos)           /*!< FMC_T::ISPADR: ISPADR Mask                */

#define FMC_ISPDAT_ISPDAT_Pos            (0)                                               /*!< FMC_T::ISPDAT: ISPDAT Position            */
#define FMC_ISPDAT_ISPDAT_Msk            (0xfffffffful << FMC_ISPDAT_ISPDAT_Pos)           /*!< FMC_T::ISPDAT: ISPDAT Mask                */

#define FMC_ISPCMD_FCTRL_Pos             (0)                                               /*!< FMC_T::ISPCMD: FCTRL Position             */
#define FMC_ISPCMD_FCTRL_Msk             (0xful << FMC_ISPCMD_FCTRL_Pos)                   /*!< FMC_T::ISPCMD: FCTRL Mask                 */

#define FMC_ISPCMD_FCEN_Pos              (4)                                               /*!< FMC_T::ISPCMD: FCEN Position              */
#define FMC_ISPCMD_FCEN_Msk              (0x1ul << FMC_ISPCMD_FCEN_Pos)                    /*!< FMC_T::ISPCMD: FCEN Mask                  */

#define FMC_ISPCMD_FOEN_Pos              (5)                                               /*!< FMC_T::ISPCMD: FOEN Position              */
#define FMC_ISPCMD_FOEN_Msk              (0x1ul << FMC_ISPCMD_FOEN_Pos)                    /*!< FMC_T::ISPCMD: FOEN Mask                  */

#define FMC_ISPTRG_ISPGO_Pos             (0)                                               /*!< FMC_T::ISPTRG: ISPGO Position             */
#define FMC_ISPTRG_ISPGO_Msk             (0x1ul << FMC_ISPTRG_ISPGO_Pos)                   /*!< FMC_T::ISPTRG: ISPGO Mask                 */

#define FMC_DFBADR_DFBA_Pos              (0)                                               /*!< FMC_T::DFBADR: DFBA Position              */
#define FMC_DFBADR_DFBA_Msk              (0xfffffffful << FMC_DFBADR_DFBA_Pos)             /*!< FMC_T::DFBADR: DFBA Mask                  */

#define FMC_ISPSTA_ISPBUSY_Pos           (0)                                               /*!< FMC_T::ISPSTA: ISPBUSY Position           */
#define FMC_ISPSTA_ISPBUSY_Msk           (0x1ul << FMC_ISPSTA_ISPBUSY_Pos)                 /*!< FMC_T::ISPSTA: ISPBUSY Mask               */

#define FMC_ISPSTA_CBS_Pos               (1)                                               /*!< FMC_T::ISPSTA: CBS Position               */
#define FMC_ISPSTA_CBS_Msk               (0x3ul << FMC_ISPSTA_CBS_Pos)                     /*!< FMC_T::ISPSTA: CBS Mask                   */

#define FMC_ISPSTA_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPSTA: ISPFF Position             */
#define FMC_ISPSTA_ISPFF_Msk             (0x1ul << FMC_ISPSTA_ISPFF_Pos)                   /*!< FMC_T::ISPSTA: ISPFF Mask                 */

/**@}*/ /* FMC_CONST */
/**@}*/ /* end of FMC register group */


/*---------------------- System Global Control Registers -------------------------*/
/**
    @addtogroup System Global Control Registers(SYS)
    Memory Mapped Structure for SYS Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PDID</font><br><p> <font size="2">
Offset: 0x00  Part Device Identification number Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>PDID</td><td><div style="word-wrap: break-word;"><b>Part Device ID</b><br>
This register reflects device part number code.<br>
Software can read this register to identify which device is used.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t PDID;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RST_SRC</font><br><p> <font size="2">
Offset: 0x04  System Reset Source Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RSTS_POR</td><td><div style="word-wrap: break-word;"><b>The RSTS_POR Flag Is Set By The "Reset Signal" From The Power-On Reset (POR) Module Or Bit CHIP_RST (IPRSTC1[0]) To Indicate The Previous Reset Source</b><br>
0 = No reset from POR or CHIP_RST.<br>
1 = Power-on Reset (POR) or CHIP_RST had issued the reset signal to reset the system.<br>
This bit is cleared by writing 1 to itself.<br>
</div></td></tr><tr><td>
[1]</td><td>RSTS_PAD</td><td><div style="word-wrap: break-word;"><b>The RSTS_PAD Flag Is Set By The "Reset Signal" From The /RESET Pin To Indicate The Previous Reset Source</b><br>
0 = No reset from /RESET pin.<br>
1 = The /RESET pin had issued the reset signal to reset the system.<br>
This bit is cleared by writing 1 to itself.<br>
</div></td></tr><tr><td>
[2]</td><td>RSTS_WDT</td><td><div style="word-wrap: break-word;"><b>The RSTS_WDT Flag Is Set By The "Reset Signal" From The Watchdog Timer Module To Indicate The Previous Reset Source</b><br>
0 = No reset from Watchdog Timer.<br>
1 = The Watchdog Timer module had issued the reset signal to reset the system.<br>
This bit is cleared by writing 1 to itself.<br>
</div></td></tr><tr><td>
[4]</td><td>RSTS_BOD</td><td><div style="word-wrap: break-word;"><b>The RSTS_BOD Flag Is Set By The "Reset Signal" From The Brown-Out-Detected Module To Indicate The Previous Reset Source</b><br>
0 = No reset from BOD.<br>
1 = Brown-out-Detected module had issued the reset signal to reset the system.<br>
This bit is cleared by writing 1 to itself.<br>
</div></td></tr><tr><td>
[5]</td><td>RSTS_SYS</td><td><div style="word-wrap: break-word;"><b>The RSTS_SYS Flag Is Set By The "Reset Signal" From The Cortex_M0 Kernel To Indicate The Previous Reset Source</b><br>
0 = No reset from Cortex_M0.<br>
1 = Cortex_M0 had issued the reset signal to reset the system by writing 1 to the bit SYSRESTREQ(AIRCR[2], Application Interrupt and Reset Control Register) in system control registers of Cortex_M0 kernel.<br>
This bit is cleared by writing 1 to itself.<br>
</div></td></tr><tr><td>
[7]</td><td>RSTS_CPU</td><td><div style="word-wrap: break-word;"><b>The RSTS_CPU Flag Is Set By Hardware If Software Writes CPU_RST (IPRST_CTL1[1]) "1" To Rest Cortex-M0 CPU Kernel And Flash Memory Controller (FMC)</b><br>
0 = No reset from CPU.<br>
1 = Cortex-M0 CPU kernel and FMC are reset by software setting CPU_RST to 1.<br>
This bit is cleared by writing 1 to itself.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t RST_SRC;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IPRST_CTL1</font><br><p> <font size="2">
Offset: 0x08  IP Reset Control Resister1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>CHIP_RST</td><td><div style="word-wrap: break-word;"><b>CHIP One Shot Reset</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
Setting this bit will reset the whole chip, including CPU kernel and all peripherals like power-on reset and this bit will automatically return to "0" after the 2 clock cycles.<br>
The chip setting from flash will be also reloaded when chip one shot reset.<br>
0 = Normal.<br>
1 = Reset CHIP.<br>
Note: In the following conditions, chip setting from flash will be reloaded.<br>
Power-on Reset<br>
Brown-out-Detected Reset<br>
Low level on the /RESET pin<br>
Set IPRST_CTL1[CHIP_RST]<br>
</div></td></tr><tr><td>
[1]</td><td>CPU_RST</td><td><div style="word-wrap: break-word;"><b>CPU Kernel One Shot Reset</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
Setting this bit will only reset the CPU kernel and Flash Memory Controller(FMC), and this bit will automatically return to "0" after the 2 clock cycles<br>
0 = Normal.<br>
1 = Reset CPU.<br>
</div></td></tr><tr><td>
[2]</td><td>DMA_RST</td><td><div style="word-wrap: break-word;"><b>DMA Controller Reset</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
Set this bit "1" will generate a reset signal to the DMA.<br>
SW needs to set this bit to low to release reset signal.<br>
0 = Normal operation.<br>
1 = DMA IP reset.<br>
</div></td></tr><tr><td>
[3]</td><td>EBI_RST</td><td><div style="word-wrap: break-word;"><b>EBI Controller Reset</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
Set this bit "1" will generate a reset signal to the EBI.<br>
SW needs to set this bit to low to release reset signal.<br>
0 = Normal operation.<br>
1 = EBI IP reset.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IPRST_CTL1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IPRST_CTL2</font><br><p> <font size="2">
Offset: 0x0C  IP Reset Control Resister2</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[1]</td><td>GPIO_RST</td><td><div style="word-wrap: break-word;"><b>GPIO Controller Reset</b><br>
0 = GPIO normal operation.<br>
1 = GPIO reset.<br>
</div></td></tr><tr><td>
[2]</td><td>TMR0_RST</td><td><div style="word-wrap: break-word;"><b>Timer0 Controller Reset</b><br>
0 = Timer0 normal operation.<br>
1 = Timer0 reset.<br>
</div></td></tr><tr><td>
[3]</td><td>TMR1_RST</td><td><div style="word-wrap: break-word;"><b>Timer1 Controller Reset</b><br>
0 = Timer1 normal operation.<br>
1 = Timer1 block reset.<br>
</div></td></tr><tr><td>
[4]</td><td>TMR2_RST</td><td><div style="word-wrap: break-word;"><b>Timer2 Controller Reset</b><br>
0 = Timer2 normal operation.<br>
1 = Timer2 block reset.<br>
</div></td></tr><tr><td>
[5]</td><td>TMR3_RST</td><td><div style="word-wrap: break-word;"><b>Timer3 Controller Reset</b><br>
0 = Timer3 normal operation.<br>
1 = Timer3 block reset.<br>
</div></td></tr><tr><td>
[7]</td><td>SC2_RST</td><td><div style="word-wrap: break-word;"><b>SmartCard 2 Controller Reset</b><br>
0 = SmartCard 2 block normal operation.<br>
1 = SmartCard 2 block reset.<br>
</div></td></tr><tr><td>
[8]</td><td>I2C0_RST</td><td><div style="word-wrap: break-word;"><b>I2C0 Controller Reset</b><br>
0 = I2C0 normal operation.<br>
1 = I2C0 block reset.<br>
</div></td></tr><tr><td>
[9]</td><td>I2C1_RST</td><td><div style="word-wrap: break-word;"><b>I2C1 Controller Reset</b><br>
0 = I2C1 block normal operation.<br>
1 = I2C1 block reset.<br>
</div></td></tr><tr><td>
[12]</td><td>SPI0_RST</td><td><div style="word-wrap: break-word;"><b>SPI0 Controller Reset</b><br>
0 = SPI0 block normal operation.<br>
1 = SPI0 block reset.<br>
</div></td></tr><tr><td>
[13]</td><td>SPI1_RST</td><td><div style="word-wrap: break-word;"><b>SPI1 Controller Reset</b><br>
0 = SPI1 normal operation.<br>
1 = SPI1 block reset.<br>
</div></td></tr><tr><td>
[14]</td><td>SPI2_RST</td><td><div style="word-wrap: break-word;"><b>SPI2 Controller Reset</b><br>
0 = SPI2 normal operation.<br>
1 = SPI2 block reset.<br>
</div></td></tr><tr><td>
[16]</td><td>UART0_RST</td><td><div style="word-wrap: break-word;"><b>UART0 Controller Reset</b><br>
0 = UART0 normal operation.<br>
1 = UART0 block reset.<br>
</div></td></tr><tr><td>
[17]</td><td>UART1_RST</td><td><div style="word-wrap: break-word;"><b>UART1 Controller Reset</b><br>
0 = UART1 normal operation.<br>
1 = UART1 block reset.<br>
</div></td></tr><tr><td>
[20]</td><td>PWM0_RST</td><td><div style="word-wrap: break-word;"><b>PWM0 Controller Reset</b><br>
0 = PWM0 block normal operation.<br>
1 = PWM0 block reset.<br>
</div></td></tr><tr><td>
[21]</td><td>PWM1_RST</td><td><div style="word-wrap: break-word;"><b>PWM1 Controller Reset</b><br>
0 = PWM1 block normal operation.<br>
1 = PWM1 block reset.<br>
</div></td></tr><tr><td>
[25]</td><td>DAC_RST</td><td><div style="word-wrap: break-word;"><b>DAC Controller Reset</b><br>
0 = DAC block normal operation.<br>
1 = DAC block reset.<br>
</div></td></tr><tr><td>
[26]</td><td>LCD_RST</td><td><div style="word-wrap: break-word;"><b>LCD Controller Reset</b><br>
0 = LCD block normal operation.<br>
1 = LCD block reset.<br>
</div></td></tr><tr><td>
[27]</td><td>USBD_RST</td><td><div style="word-wrap: break-word;"><b>USB Device Controller Reset</b><br>
0 = USB block normal operation.<br>
1 = USB block reset.<br>
</div></td></tr><tr><td>
[28]</td><td>ADC_RST</td><td><div style="word-wrap: break-word;"><b>ADC Controller Reset</b><br>
0 = ADC block normal operation.<br>
1 = ADC block reset.<br>
</div></td></tr><tr><td>
[29]</td><td>I2S_RST</td><td><div style="word-wrap: break-word;"><b>I2S Controller Reset</b><br>
0 = I2S block normal operation.<br>
1 = I2S block reset.<br>
</div></td></tr><tr><td>
[30]</td><td>SC0_RST</td><td><div style="word-wrap: break-word;"><b>SmartCard 0 Controller Reset</b><br>
0 = SmartCard block normal operation.<br>
1 = SmartCard block reset.<br>
</div></td></tr><tr><td>
[31]</td><td>SC1_RST</td><td><div style="word-wrap: break-word;"><b>SmartCard1 Controller Reset</b><br>
0 = SmartCard block normal operation.<br>
1 = SmartCard block reset.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IPRST_CTL2;
    uint32_t RESERVE0[4];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TEMPCTL</font><br><p> <font size="2">
Offset: 0x20  Temperature Sensor Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>VTEMP_EN</td><td><div style="word-wrap: break-word;"><b>Temperature Sensor Enable</b><br>
0 = Temperature sensor function Disabled (default).<br>
1 = Temperature sensor function Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TEMPCTL;
    uint32_t RESERVE1[3];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PA_L_MFP</font><br><p> <font size="2">
Offset: 0x30  Port A low byte multiple function control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>PA0_MFP</td><td><div style="word-wrap: break-word;"><b>PA.0 Pin Function Selection</b><br>
001 = ADC input channel 0<br>
100 = SmartCard 2 card detect<br>
Others = GPIOA[0]<br>
</div></td></tr><tr><td>
[6:4]</td><td>PA1_MFP</td><td><div style="word-wrap: break-word;"><b>PA.1 Pin Function Selection</b><br>
001 = ADC input channel 1<br>
010 = EBI AD[12]<br>
Others = GPIOA[1]<br>
</div></td></tr><tr><td>
[10:8]</td><td>PA2_MFP</td><td><div style="word-wrap: break-word;"><b>PA.2 Pin Function Selection</b><br>
001 = ADC input channel 2<br>
010 = EBI AD[11]<br>
101 = UART1_RXD<br>
Others = GPIOA[2]<br>
</div></td></tr><tr><td>
[14:12]</td><td>PA3_MFP</td><td><div style="word-wrap: break-word;"><b>PA.3 Pin Function Selection</b><br>
001 = ADC input channel 3<br>
010 = EBI AD[10]<br>
101 = UART1_TXD<br>
Others = GPIOA[3]<br>
</div></td></tr><tr><td>
[18:16]</td><td>PA4_MFP</td><td><div style="word-wrap: break-word;"><b>PA.4 Pin Function Selection</b><br>
001 = ADC input channel 4<br>
010 = EBI AD[9]<br>
100 = SmartCard 2 power<br>
101 = I2C0 SDA<br>
111 = LCD SEG 39<br>
Others = GPIOA[4]<br>
</div></td></tr><tr><td>
[22:20]</td><td>PA5_MFP</td><td><div style="word-wrap: break-word;"><b>PA.5 Pin Function Selection</b><br>
001 = ADC input channel 5<br>
010 = EBI AD[8]<br>
100 = SmartCard2 RST<br>
101 = I2C0 SCL<br>
111 = LCD SEG 38<br>
Others = GPIOA[5]<br>
</div></td></tr><tr><td>
[26:24]</td><td>PA6_MFP</td><td><div style="word-wrap: break-word;"><b>PA.6 Pin Function Selection</b><br>
001 = ADC input channel 6<br>
010 = EBI AD[7]<br>
011 = Timer 3 Capture event<br>
100 = SmartCard 2 clock<br>
101 = PWM0 Channel 3<br>
111 = LCD SEG 37<br>
Others = GPIOA[6]<br>
</div></td></tr><tr><td>
[30:28]</td><td>PA7_MFP</td><td><div style="word-wrap: break-word;"><b>PA.7 Pin Function Selection</b><br>
001 = ADC input channel 7<br>
010 = EBI AD[6]<br>
011 = Timer 2 capture event<br>
100 = SmartCard 2 data pin<br>
101 = PWM0 Channel 2<br>
111 = LCD SEG 36<br>
Others = GPIOA[7]<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PA_L_MFP;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PA_H_MFP</font><br><p> <font size="2">
Offset: 0x34  Port A high byte  multiple function control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>PA8_MFP</td><td><div style="word-wrap: break-word;"><b>PA.8 Pin Function Selection</b><br>
001 = I2C0 SDA<br>
011 = SmartCard0 clock<br>
100 = SPI2 1st slave select pin<br>
111 = LCD SEG 20<br>
Others = GPIOA[8]<br>
</div></td></tr><tr><td>
[6:4]</td><td>PA9_MFP</td><td><div style="word-wrap: break-word;"><b>PA.9 Pin Function Selection</b><br>
001 = I2C0 SCL<br>
011 = SmartCard0 DATA<br>
100 = SPI2 SCLK<br>
111 = LCD SEG 21<br>
Others = GPIOA[9]<br>
</div></td></tr><tr><td>
[10:8]</td><td>PA10_MFP</td><td><div style="word-wrap: break-word;"><b>PA.10 Pin Function Selection</b><br>
001 = I2C1 SDA<br>
010 = EBI nWR<br>
011 = SmartCard0 Power<br>
100 = SPI2 MISO0<br>
111 = LCD SEG 22<br>
Others = GPIOA[10]<br>
</div></td></tr><tr><td>
[14:12]</td><td>PA11_MFP</td><td><div style="word-wrap: break-word;"><b>PA.11 Pin Function Selection</b><br>
001 = I2C1 SCL<br>
010 = EBI nRE<br>
011 = SmartCard0 RST<br>
100 = SPI2 MOSI0<br>
111 = LCD SEG 23<br>
Others = GPIOA[11]<br>
</div></td></tr><tr><td>
[18:16]</td><td>PA12_MFP</td><td><div style="word-wrap: break-word;"><b>PA.12 Pin Function Selection</b><br>
001 = PWM0 Channel 0<br>
010 = EBI AD[13]<br>
011 = Timer0 capture event<br>
101 = I2C0 SDA<br>
Others = GPIOA[12]<br>
</div></td></tr><tr><td>
[22:20]</td><td>PA13_MFP</td><td><div style="word-wrap: break-word;"><b>PA.13 Pin Function Selection</b><br>
001 = PWM0 Channel 1<br>
010 = EBI AD[14]<br>
011 = Timer1 capture event<br>
101 = I2C0 SCL<br>
Others = GPIOA[13]<br>
</div></td></tr><tr><td>
[26:24]</td><td>PA14_MFP</td><td><div style="word-wrap: break-word;"><b>PA.14 Pin Function Selection</b><br>
001 = PWM0 Channel 2<br>
010 = EBI AD[15]<br>
011 = Timer2 capture event<br>
110 = UART0 RX<br>
Others = GPIOA[14]<br>
</div></td></tr><tr><td>
[30:28]</td><td>PA15_MFP</td><td><div style="word-wrap: break-word;"><b>PA.15 Pin Function Selection</b><br>
001 = PWM0 Channel 3<br>
010 = I2S MCLK<br>
011 = Timer3 capture event<br>
100 = SmartCard 0 power<br>
110 = UART0 TX<br>
Others = GPIOA[15]<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PA_H_MFP;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PB_L_MFP</font><br><p> <font size="2">
Offset: 0x38  Port B low byte multiple function control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>PB0_MFP</td><td><div style="word-wrap: break-word;"><b>PB.0 Pin Function Selection</b><br>
001 = UART0 RX<br>
011 = SPI1 MOSI0<br>
111 = LCD SEG 7<br>
Others = GPIOB[0]<br>
</div></td></tr><tr><td>
[6:4]</td><td>PB1_MFP</td><td><div style="word-wrap: break-word;"><b>PB.1 Pin Function Selection</b><br>
001 = UART0 TX<br>
011 = SPI1 MISO0<br>
111 = LCD SEG 6<br>
Others = GPIOB[1]<br>
</div></td></tr><tr><td>
[10:8]</td><td>PB2_MFP</td><td><div style="word-wrap: break-word;"><b>PB.2 Pin Function Selection</b><br>
001 = UART0 RTSn<br>
010 = EBI nWRL<br>
011 = SPI1 SCLK<br>
111 = LCD SEG 5<br>
Others = GPIOB[2]<br>
</div></td></tr><tr><td>
[14:12]</td><td>PB3_MFP</td><td><div style="word-wrap: break-word;"><b>PB.3 Pin Function Selection</b><br>
001 = UART0 CTSn<br>
010 = EBI nWRH<br>
011 = SPI1 1st slave select pin<br>
111 = LCD SEG 4<br>
Others = GPIOB[3]<br>
</div></td></tr><tr><td>
[18:16]</td><td>PB4_MFP</td><td><div style="word-wrap: break-word;"><b>PB.4 Pin Function Selection</b><br>
001 = UART1 RX<br>
011 = SmartCard0 card detection<br>
100 = SPI2 1st slave select pin<br>
111 = LCD SEG 13<br>
Others = GPIOB[4]<br>
</div></td></tr><tr><td>
[22:20]</td><td>PB5_MFP</td><td><div style="word-wrap: break-word;"><b>PB.5 Pin Function Selection</b><br>
001 = UART1 TX<br>
011 = SmartCard0 RST<br>
100 = SPI2 SCLK<br>
111 = LCD SEG 12<br>
Others = GPIOB[5]<br>
</div></td></tr><tr><td>
[26:24]</td><td>PB6_MFP</td><td><div style="word-wrap: break-word;"><b>PB.6 Pin Function Selection</b><br>
001 = UART1 RTSn<br>
010 = EBI ALE<br>
100 = SPI2 MISO0<br>
111 = LCD SEG 11<br>
Others = GPIOB[6]<br>
</div></td></tr><tr><td>
[30:28]</td><td>PB7_MFP</td><td><div style="word-wrap: break-word;"><b>PB.7 Pin Function Selection</b><br>
001 = UART1 CTSn<br>
010 = EBI nCS<br>
100 = SPI2 MOSI0<br>
111 = LCD SEG 10<br>
Others = GPIOB[7]<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PB_L_MFP;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PB_H_MFP</font><br><p> <font size="2">
Offset: 0x3C  Port B high byte multiple function control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>PB8_MFP</td><td><div style="word-wrap: break-word;"><b>PB.8 Pin Function Selection</b><br>
001 = ADC external trigger<br>
010 = Timer0 external event input or Timer0 toggle output<br>
011 = External interrupt 0<br>
100 = SmartCard 2 power<br>
111 = LCD SEG 30<br>
Others = GPIOB[8]<br>
</div></td></tr><tr><td>
[6:4]</td><td>PB9_MFP</td><td><div style="word-wrap: break-word;"><b>PB.9 Pin Function Selection</b><br>
001 = SPI1 2nd slave select pin<br>
010 = Timer1 external event input or Timer1 toggle output<br>
100 = SmartCard 2 RST<br>
101 = External interrupt 0<br>
111 = LCD V1<br>
Others = GPIOB[9]<br>
</div></td></tr><tr><td>
[10:8]</td><td>PB10_MFP</td><td><div style="word-wrap: break-word;"><b>PB.10 Pin Function Selection</b><br>
001 = SPI0 2nd slave select pin<br>
010 = Timer2 external event input or Timer2 toggle output<br>
100 = SmartCard 2 clock<br>
101 = SPI0 MOSI0<br>
111 = LCD V2<br>
Others = GPIOB[10]<br>
</div></td></tr><tr><td>
[14:12]</td><td>PB11_MFP</td><td><div style="word-wrap: break-word;"><b>PB.11 Pin Function Selection</b><br>
001 = PWM1 Channel 0<br>
010 = Timer3 external event input or Timer3 toggle output<br>
100 = SmartCard 2 DATA<br>
101 = SPI0 MISO0<br>
111 = LCD V3<br>
Others = GPIOB[11]<br>
</div></td></tr><tr><td>
[18:16]</td><td>PB12_MFP</td><td><div style="word-wrap: break-word;"><b>PB.12 Pin Function Selection</b><br>
010 = EBI AD[0]<br>
100 = FRQDIV_CLK<br>
111 = LCD SEG 24<br>
Others = GPIOB[12]<br>
</div></td></tr><tr><td>
[22:20]</td><td>PB13_MFP</td><td><div style="word-wrap: break-word;"><b>PB.13 Pin Function Selection</b><br>
010 = EBI AD[1]<br>
111 = LCD SEG 25<br>
Others = GPIOB[13]<br>
</div></td></tr><tr><td>
[26:24]</td><td>PB14_MFP</td><td><div style="word-wrap: break-word;"><b>PB.14 Pin Function Selection</b><br>
001 = External interrupt 0<br>
011 = SmartCard 2 card detect<br>
100 = SPI2 2nd slave select pin<br>
111 = LCD SEG 26<br>
Others = GPIOB[14]<br>
</div></td></tr><tr><td>
[30:28]</td><td>PB15_MFP</td><td><div style="word-wrap: break-word;"><b>PB.15 Pin Function Selection</b><br>
001 = External interrupt 1<br>
011 = Snooper pin<br>
100 = SmartCard1 card detect<br>
111 = LCD SEG 31<br>
Others = GPIOB[15]<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PB_H_MFP;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PC_L_MFP</font><br><p> <font size="2">
Offset: 0x40  Port C low byte multiple function control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>PC0_MFP</td><td><div style="word-wrap: break-word;"><b>PC.0 Pin Function Selection</b><br>
001 = SPI0 1st slave select pin<br>
010 = I2S WS<br>
100 = SmartCard1 clock<br>
111 = LCD DH1<br>
Others = GPIOC[0]<br>
</div></td></tr><tr><td>
[6:4]</td><td>PC1_MFP</td><td><div style="word-wrap: break-word;"><b>PC.1 Pin Function Selection</b><br>
001 = SPI0 SCLK<br>
010 = I2S BCLK<br>
100 = SmartCard1 DATA<br>
111 = LCD DH2<br>
Others = GPIOC[1]<br>
</div></td></tr><tr><td>
[10:8]</td><td>PC2_MFP</td><td><div style="word-wrap: break-word;"><b>PC.2 Pin Function Selection</b><br>
001 = SPI0 MISO0<br>
010 = I2S Din<br>
100 = SmartCard1 Power<br>
111 = LCD COM 0<br>
Others = GPIOC[2]<br>
</div></td></tr><tr><td>
[14:12]</td><td>PC3_MFP</td><td><div style="word-wrap: break-word;"><b>PC.3 Pin Function Selection</b><br>
001 = SPI0 MOSI1<br>
010 = I2S Dout<br>
100 = SmartCard1 RST<br>
111 = LCD COM 1<br>
Others = GPIOC[3]<br>
</div></td></tr><tr><td>
[18:16]</td><td>PC4_MFP</td><td><div style="word-wrap: break-word;"><b>PC.4 Pin Function Selection</b><br>
001 = SPI0 MISO1<br>
111 = LCD COM 2<br>
Others = GPIOC[4]<br>
</div></td></tr><tr><td>
[22:20]</td><td>PC5_MFP</td><td><div style="word-wrap: break-word;"><b>PC.5 Pin Function Selection</b><br>
001 = SPI0 MOSI1<br>
111 = LCD COM 3<br>
Others = GPIOC[5]<br>
</div></td></tr><tr><td>
[26:24]</td><td>PC6_MFP</td><td><div style="word-wrap: break-word;"><b>PC.6 Pin Function Selection</b><br>
001 = DA out0<br>
010 = EBI AD[4]<br>
011 = Timer0 capture event<br>
100 = SmartCard1 card detection<br>
101 = PWM0 Channel 0<br>
Others = GPIOC[6]<br>
</div></td></tr><tr><td>
[30:28]</td><td>PC7_MFP</td><td><div style="word-wrap: break-word;"><b>PC.7 Pin Function Selection</b><br>
001 = DA out1<br>
010 = EBI AD[5]<br>
011 = Timer1 capture event<br>
101 = PWM0 Channel 1<br>
Others = GPIOC[7]<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PC_L_MFP;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PC_H_MFP</font><br><p> <font size="2">
Offset: 0x44  Port C high byte multiple function control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>PC8_MFP</td><td><div style="word-wrap: break-word;"><b>PC.8 Pin Function Selection</b><br>
001 = SPI1 1st slave select pin<br>
010 = EBI MCLK<br>
101 = I2C1 SDA<br>
Others = GPIOC[8]<br>
</div></td></tr><tr><td>
[6:4]</td><td>PC9_MFP</td><td><div style="word-wrap: break-word;"><b>PC.9 Pin Function Selection</b><br>
001 = SPI1 SCLK<br>
101 = I2C1 SCL<br>
Others = GPIOC[9]<br>
</div></td></tr><tr><td>
[10:8]</td><td>PC10_MFP</td><td><div style="word-wrap: break-word;"><b>PC.10 Pin Function Selection</b><br>
001 = SPI1 MISO0<br>
101 = UART1 RX<br>
Others = GPIOC[10]<br>
</div></td></tr><tr><td>
[14:12]</td><td>PC11_MFP</td><td><div style="word-wrap: break-word;"><b>PC.11 Pin Function Selection</b><br>
001 = SPI1 MOSI0<br>
101 = UART1 TX<br>
Others = GPIOC[11]<br>
</div></td></tr><tr><td>
[18:16]</td><td>PC12_MFP</td><td><div style="word-wrap: break-word;"><b>PC.12 Pin Function Selection</b><br>
001 = SPI1 MISO1<br>
010 = PWM1 Channel 0<br>
101 = External interrupt 0<br>
110 = I2C0 SDA<br>
Others = GPIOC[12]<br>
</div></td></tr><tr><td>
[22:20]</td><td>PC13_MFP</td><td><div style="word-wrap: break-word;"><b>PC.13 Pin Function Selection</b><br>
001 = SPI1 MOSI1<br>
010 = PWM1 Channel 1<br>
100 = Snooper pin<br>
101 = External interrupt 1<br>
110 = I2C0 SCL<br>
Others = GPIOC[13]<br>
</div></td></tr><tr><td>
[26:24]</td><td>PC14_MFP</td><td><div style="word-wrap: break-word;"><b>PC.14 Pin Function Selection</b><br>
010 = EBI AD[2]<br>
100 = PWM1 Channel 3<br>
111 = LCD SEG 32<br>
Others = GPIOC[14]<br>
</div></td></tr><tr><td>
[30:28]</td><td>PC15_MFP</td><td><div style="word-wrap: break-word;"><b>PC.15 Pin Function Selection</b><br>
010 = EBI AD[3]<br>
011 = Timer0 capture event<br>
100 = PWM1 Channel 2<br>
111 = LCD SEG 33<br>
Others = GPIOC[15]<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PC_H_MFP;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PD_L_MFP</font><br><p> <font size="2">
Offset: 0x48  Port D low byte multiple function control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>PD0_MFP</td><td><div style="word-wrap: break-word;"><b>PD.0 Pin Function Selection</b><br>
001 = UART1 RX<br>
011 = SPI2 1st slave select pin<br>
100 = SmartCard1 clock<br>
101 = ADC input channel8<br>
Others = GPIOD[0]<br>
</div></td></tr><tr><td>
[6:4]</td><td>PD1_MFP</td><td><div style="word-wrap: break-word;"><b>PD.1 Pin Function Selection</b><br>
001 = UART1 TX<br>
011 = SPI2 SCLK<br>
100 = SmartCard1 DATA<br>
101 = ADC input channel9<br>
Others = GPIOD[1]<br>
</div></td></tr><tr><td>
[10:8]</td><td>PD2_MFP</td><td><div style="word-wrap: break-word;"><b>PD.2 Pin Function Selection</b><br>
001 = UART1 RTSn<br>
010 = I2S WS<br>
011 = SPI2 MISO0<br>
100 = SmartCard1 power<br>
101 = ADC input channel10<br>
Others = GPIOD[2]<br>
</div></td></tr><tr><td>
[14:12]</td><td>PD3_MFP</td><td><div style="word-wrap: break-word;"><b>PD.3 Pin Function Selection</b><br>
001 = UART1 CTSn<br>
010 = I2S BCLK<br>
011 = SPI2 MOSI0<br>
100 = SmartCard1 reset<br>
101 = ADC input channel11<br>
Others = GPIOD[3]<br>
</div></td></tr><tr><td>
[18:16]</td><td>PD4_MFP</td><td><div style="word-wrap: break-word;"><b>PD.4 Pin Function Selection</b><br>
010 = I2S Din<br>
011 = SPI2 MISO1<br>
100 = SmartCard1 card detection<br>
111 = LCD SEG 35<br>
Others = GPIOD[4]<br>
</div></td></tr><tr><td>
[22:20]</td><td>PD5_MFP</td><td><div style="word-wrap: break-word;"><b>PD.5 Pin Function Selection</b><br>
010 = I2S Dout<br>
011 = SPI2 MOSI1<br>
111 = LCD SEG 34<br>
Others = GPIOD[5]<br>
</div></td></tr><tr><td>
[26:24]</td><td>PD6_MFP</td><td><div style="word-wrap: break-word;"><b>PD.6 Pin Function Selection</b><br>
111 = LCD SEG 3<br>
Others = GPIOD[6]<br>
</div></td></tr><tr><td>
[30:28]</td><td>PD7_MFP</td><td><div style="word-wrap: break-word;"><b>PD.7 Pin Function Selection</b><br>
111 = LCD SEG 2<br>
Others = GPIOD[7]<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PD_L_MFP;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PD_H_MFP</font><br><p> <font size="2">
Offset: 0x4C  Port D high byte multiple function control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>PD8_MFP</td><td><div style="word-wrap: break-word;"><b>PD.8 Pin Function Selection</b><br>
111 = LCD SEG 19<br>
Others = GPIOD[8]<br>
</div></td></tr><tr><td>
[6:4]</td><td>PD9_MFP</td><td><div style="word-wrap: break-word;"><b>PD.9 Pin Function Selection</b><br>
111 = LCD SEG 18<br>
Others = GPIOD[9]<br>
</div></td></tr><tr><td>
[10:8]</td><td>PD10_MFP</td><td><div style="word-wrap: break-word;"><b>PD.10 Pin Function Selection</b><br>
111 = LCD SEG 17<br>
Others = GPIOD[10]<br>
</div></td></tr><tr><td>
[14:12]</td><td>PD11_MFP</td><td><div style="word-wrap: break-word;"><b>PD.11 Pin Function Selection</b><br>
111 = LCD SEG 16<br>
Others = GPIOD[11]<br>
</div></td></tr><tr><td>
[18:16]</td><td>PD12_MFP</td><td><div style="word-wrap: break-word;"><b>PD.12 Pin Function Selection</b><br>
111 = LCD SEG 15<br>
Others = GPIOD[12]<br>
</div></td></tr><tr><td>
[22:20]</td><td>PD13_MFP</td><td><div style="word-wrap: break-word;"><b>PD.13 Pin Function Selection</b><br>
111 = LCD SEG 14<br>
Others = GPIOD[13]<br>
</div></td></tr><tr><td>
[26:24]</td><td>PD14_MFP</td><td><div style="word-wrap: break-word;"><b>PD.14 Pin Function Selection</b><br>
111 = LCD SEG 1<br>
Others = GPIOD[14]<br>
</div></td></tr><tr><td>
[30:28]</td><td>PD15_MFP</td><td><div style="word-wrap: break-word;"><b>PD.15 Pin Function Selection</b><br>
111 = LCD SEG 0<br>
Others = GPIOD[15]<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PD_H_MFP;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PE_L_MFP</font><br><p> <font size="2">
Offset: 0x50  Port E low byte multiple function control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>PE0_MFP</td><td><div style="word-wrap: break-word;"><b>PE.0 Pin Function Selection</b><br>
001 = PWM1 Channel 2<br>
010 = I2S MCLK<br>
Others = GPIOE[0]<br>
</div></td></tr><tr><td>
[6:4]</td><td>PE1_MFP</td><td><div style="word-wrap: break-word;"><b>PE.1 Pin Function Selection</b><br>
001 = PWM1 Channel 3<br>
110 = SPI0 1st slave select pin<br>
Others = GPIOE[1]<br>
</div></td></tr><tr><td>
[10:8]</td><td>PE2_MFP</td><td><div style="word-wrap: break-word;"><b>PE.2 Pin Function Selection</b><br>
110 = SPI0 SCLK<br>
Others = GPIOE[2]<br>
</div></td></tr><tr><td>
[14:12]</td><td>PE3_MFP</td><td><div style="word-wrap: break-word;"><b>PE.3 Pin Function Selection</b><br>
110 = SPI0 MISO0<br>
Others = GPIOE[3]<br>
</div></td></tr><tr><td>
[18:16]</td><td>PE4_MFP</td><td><div style="word-wrap: break-word;"><b>PE.4 Pin Function Selection</b><br>
110 = SPI0 MOSI0<br>
Others = GPIOE[4]<br>
</div></td></tr><tr><td>
[22:20]</td><td>PE5_MFP</td><td><div style="word-wrap: break-word;"><b>PE.5 Pin Function Selection</b><br>
001 = PWM1 Channel 1<br>
Others = GPIOE[5]<br>
</div></td></tr><tr><td>
[26:24]</td><td>PE6_MFP</td><td><div style="word-wrap: break-word;"><b>PE.6 Pin Function Selection</b><br>
GPIOE[6]<br>
</div></td></tr><tr><td>
[30:28]</td><td>PE7_MFP</td><td><div style="word-wrap: break-word;"><b>PE.7 Pin Function Selection</b><br>
111 = LCD SEG 8<br>
Others = GPIOE[7]<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PE_L_MFP;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PE_H_MFP</font><br><p> <font size="2">
Offset: 0x54  Port E high byte multiple function control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>PE8_MFP</td><td><div style="word-wrap: break-word;"><b>PE.8 Pin Function Selection</b><br>
111 = LCD SEG 9<br>
Others = GPIOE[8]<br>
</div></td></tr><tr><td>
[6:4]</td><td>PE9_MFP</td><td><div style="word-wrap: break-word;"><b>PE.9 Pin Function Selection</b><br>
111 = UART1 RX<br>
Others = GPIOE[9]<br>
</div></td></tr><tr><td>
[10:8]</td><td>PE10_MFP</td><td><div style="word-wrap: break-word;"><b>PE.10 Pin Function Selection</b><br>
111 = UART1 TX<br>
Others = GPIOE[10]<br>
</div></td></tr><tr><td>
[14:12]</td><td>PE11_MFP</td><td><div style="word-wrap: break-word;"><b>PE.11 Pin Function Selection</b><br>
111 = UART1 RTSn<br>
Others = GPIOE[11]<br>
</div></td></tr><tr><td>
[18:16]</td><td>PE12_MFP</td><td><div style="word-wrap: break-word;"><b>PE.12 Pin Function Selection</b><br>
111 = UART1 CTSn<br>
Others = GPIOE[12]<br>
</div></td></tr><tr><td>
[22:20]</td><td>PE13_MFP</td><td><div style="word-wrap: break-word;"><b>PE.13 Pin Function Selection</b><br>
111 = LCD SEG 27<br>
Others = GPIOE[13]<br>
</div></td></tr><tr><td>
[26:24]</td><td>PE14_MFP</td><td><div style="word-wrap: break-word;"><b>PE.14 Pin Function Selection</b><br>
111 = LCD SEG 28<br>
Others = GPIOE[14]<br>
</div></td></tr><tr><td>
[30:28]</td><td>PE15_MFP</td><td><div style="word-wrap: break-word;"><b>PE.15 Pin Function Selection</b><br>
111 = LCD SEG 2<br>
Others = GPIOE[15]<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PE_H_MFP;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PF_L_MFP</font><br><p> <font size="2">
Offset: 0x58  Port F low byte multiple function control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>PF0_MFP</td><td><div style="word-wrap: break-word;"><b>PF.0 Pin Function Selection</b><br>
101 = External interrupt 0<br>
111 = ICE DATA<br>
Others = GPIOF[1]<br>
</div></td></tr><tr><td>
[6:4]</td><td>PF1_MFP</td><td><div style="word-wrap: break-word;"><b>PF.1 Pin Function Selection</b><br>
100 = FRQDIV_CLK<br>
101 = External interrupt 1<br>
111 = ICE CLOCK<br>
Others = GPIOF[1]<br>
</div></td></tr><tr><td>
[10:8]</td><td>PF2_MFP</td><td><div style="word-wrap: break-word;"><b>PF.2 Pin Function Selection</b><br>
111 = HXT OUT<br>
Others = GPIOF[2]<br>
</div></td></tr><tr><td>
[14:12]</td><td>PF3_MFP</td><td><div style="word-wrap: break-word;"><b>PF.3 Pin Function Selection</b><br>
111 = HXT IN<br>
Others = GPIOF[3]<br>
</div></td></tr><tr><td>
[18:16]</td><td>PF4_MFP</td><td><div style="word-wrap: break-word;"><b>PF.4 Pin Function Selection</b><br>
001 = I2C0 SDA<br>
Others = GPIOF[4]<br>
</div></td></tr><tr><td>
[22:20]</td><td>PF5_MFP</td><td><div style="word-wrap: break-word;"><b>PF.5 Pin Function Selection</b><br>
001 = I2C0 SCL<br>
Others = GPIOF[5]<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PF_L_MFP;
    uint32_t RESERVE2[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PORCTL</font><br><p> <font size="2">
Offset: 0x60  Power-On-Reset Controller Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>POR_DIS_CODE</td><td><div style="word-wrap: break-word;"><b>Power-On Reset Enable Control</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again.<br>
If setting the POR_DIS_CODE to 0x5AA5, the POR reset function will be disabled and the POR function will be active again when POR_DIS_CODE is set to another value or POR_DIS_CODE is reset by chip other reset functions, including: /RESET, Watchdog Timer reset, BOD reset, ICE reset command and the software-chip reset function.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PORCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">BODCTL</font><br><p> <font size="2">
Offset: 0x64  Brown-out Detector Controller Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>BOD17_EN</td><td><div style="word-wrap: break-word;"><b>Brown-Out Detector 1.7V Function Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
The default value is set by flash controller user configuration register config0 bit[20:19]<br>
Users can disable BOD17_EN but it takes effective (disabled) only in Power-down mode.<br>
Once existing Power-down mode, BOD17 will be enabled by HW automatically.<br>
When CPU reads this bit, CPU will read whether BOD17 function enabled or not.<br>
In other words,CPU will always read high.<br>
0 = Brown-out Detector 1.7V function Disabled.<br>
1 = Brown-out Detector 1.7V function Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>BOD20_EN</td><td><div style="word-wrap: break-word;"><b>Brown-Out Detector 2.0 V Function Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Brown-out Detector 2.0 V function Disabled.<br>
1 = Brown-out Detector 2.0 V function Enabled.<br>
BOD20_EN is default on.<br>
If SW disables it, Brown-out Detector 2.0 V function is not disabled until chip enters power-down mode.<br>
If system is not in power-down mode, BOD20_EN will be enabled by hardware automatically.<br>
</div></td></tr><tr><td>
[2]</td><td>BOD25_EN</td><td><div style="word-wrap: break-word;"><b>Brown-Out Detector 2.5 V Function Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Brown-out Detector 2.5 V function Disabled.<br>
1 = Brown-out Detector 2.5 V function Enabled.<br>
</div></td></tr><tr><td>
[4]</td><td>BOD17_RST_EN</td><td><div style="word-wrap: break-word;"><b>BOD 1.7 V Reset Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Reset does not issue when BOD17 occurs.<br>
1 = Reset issues when BOD17 occurs.<br>
The default value is set by flash controller user configuration register config0 bit[20:19]<br>
BOD17_RST_EN can be controlled (enable or disable) only when BOD17_EN is high.<br>
</div></td></tr><tr><td>
[5]</td><td>BOD20_RST_EN</td><td><div style="word-wrap: break-word;"><b>BOD 2.0 V Reset Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Reset does not issue when BOD20 occurs.<br>
1 = Reset issues when BOD20 occurs.<br>
The default value is set by flash controller user configuration register config0 bit[20:19]<br>
</div></td></tr><tr><td>
[6]</td><td>BOD25_RST_EN</td><td><div style="word-wrap: break-word;"><b>BOD 2.5 V Reset Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Reset does not issue when BOD25 occurs.<br>
1 = Reset issues when BOD25 occurs.<br>
The default value is set by flash controller user configuration register config0 bit[20:19]<br>
</div></td></tr><tr><td>
[8]</td><td>BOD17_INT_EN</td><td><div style="word-wrap: break-word;"><b>BOD 1.7 V Interrupt Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Interrupt does not issue when BOD17 occurs.<br>
1 = Interrupt issues when BOD17 occurs.<br>
</div></td></tr><tr><td>
[9]</td><td>BOD20_INT_EN</td><td><div style="word-wrap: break-word;"><b>BOD 2.0 V Interrupt Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Interrupt does not issue when BOD20 occurs.<br>
1 = Interrupt issues when BOD20 occurs.<br>
</div></td></tr><tr><td>
[10]</td><td>BOD25_INT_EN</td><td><div style="word-wrap: break-word;"><b>BOD 2.5 V Interrupt Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Interrupt does not issue when BOD25 occurs.<br>
1 = Interrupt issues when BOD25 occurs.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t BODCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">BODSTS</font><br><p> <font size="2">
Offset: 0x68  Brown-out Detector Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>BOD_INT</td><td><div style="word-wrap: break-word;"><b>Brown-Out Detector Interrupt Status</b><br>
1 = When Brown-out Detector detects the VDD is dropped down through the target detected voltage or the VDD is raised up through the target detected voltage and Brown-out interrupt is enabled, this bit will be set to 1.<br>
0 = Brown-out Detector does not detect any voltage drift at VDD down through or up through the target detected voltage after interrupt is enabled.<br>
This bit is cleared by writing 1 to itself.<br>
</div></td></tr><tr><td>
[1]</td><td>BOD17_drop</td><td><div style="word-wrap: break-word;"><b>Brown-Out Detector Lower Than 1.7V Status</b><br>
Setting BOD17_drop high means once the detected voltage is lower than target detected voltage setting (1.7V).<br>
Software can write 1 to clear BOD17_drop.<br>
</div></td></tr><tr><td>
[2]</td><td>BOD20_drop</td><td><div style="word-wrap: break-word;"><b>Brown-Out Detector Lower Than 2.0V Status</b><br>
Setting BOD20_drop high means once the detected voltage is lower than target detected voltage setting (2.0V).<br>
Software can write 1 to clear BOD20_drop.<br>
</div></td></tr><tr><td>
[3]</td><td>BOD25_drop</td><td><div style="word-wrap: break-word;"><b>Brown-Out Detector Lower Than 2.5V Status</b><br>
Setting BOD25_drop high means once the detected voltage is lower than target detected voltage setting (2.5V).<br>
Software can write 1 to clear BOD25_drop.<br>
</div></td></tr><tr><td>
[4]</td><td>BOD17_rise</td><td><div style="word-wrap: break-word;"><b>Brown-Out Detector Higher Than 1.7V Status</b><br>
Setting BOD17_rise high means once the detected voltage is higher than target detected voltage setting (1.7V).<br>
Software can write 1 to clear BOD17_rise.<br>
</div></td></tr><tr><td>
[5]</td><td>BOD20_rise</td><td><div style="word-wrap: break-word;"><b>Brown-Out Detector Higher Than 2.0V Status</b><br>
Setting BOD20_rise high means once the detected voltage is higher than target detected voltage setting (2.0V).<br>
Software can write 1 to clear BOD20_rise.<br>
</div></td></tr><tr><td>
[6]</td><td>BOD25_rise</td><td><div style="word-wrap: break-word;"><b>Brown-Out Detector Higher Than 2.5V Status</b><br>
Setting BOD25_rise high means once the detected voltage is higher than target detected voltage setting (2.5V).<br>
Software can write 1 to clear BOD25_rise.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO  uint32_t BODSTS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">Int_VREFCTL</font><br><p> <font size="2">
Offset: 0x6C  Voltage reference Control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>BGP_EN</td><td><div style="word-wrap: break-word;"><b>Band-Gap Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
Band-gap is the reference voltage of internal reference voltage.<br>
User must enable band-gap if want to enable internal 1.8V or 2.5V reference voltage.<br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>REG_EN</td><td><div style="word-wrap: break-word;"><b>Regulator Enable</b><br>
Enable internal 1.8V or 2.5V reference voltage.<br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>SEL25</td><td><div style="word-wrap: break-word;"><b>Regulator Output Voltage Selection</b><br>
Select internal reference voltage level.<br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = 1.8V.<br>
1 = 2.5V.<br>
</div></td></tr><tr><td>
[3]</td><td>EXT_MODE</td><td><div style="word-wrap: break-word;"><b>Regulator External Mode</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
Users can output regulator output voltage in VREF pin if EXT_MODE is high.<br>
0 = No connection with external VREF pin.<br>
1 = Connect to external VREF pin.<br>
Connect a 1uF to 10uF capacitor to AVSS will let internal voltage reference be more stable.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t Int_VREFCTL;
    uint32_t RESERVE3[4];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IRCTRIMCTL</font><br><p> <font size="2">
Offset: 0x80  HIRC Trim Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[1:0]</td><td>TRIM_SEL</td><td><div style="word-wrap: break-word;"><b>Trim Frequency Selection</b><br>
This field indicates the target frequency of HIRC auto trim.<br>
If no any target frequency is selected (TRIM_SEL is 00), the HIRC auto trim function is disabled.<br>
During auto trim operation, if 32.768 kHz clock error detected or trim retry limitation count reached, this field will be cleared to 00 automatically.<br>
00 = Disable HIRC auto trim function<br>
01 = Enable HIRC auto trim function and trim HIRC to 11.0592 MHz<br>
10 = Enable HIRC auto trim function and trim HIRC to 12 MHz<br>
11 = Enable HIRC auto trim function and trim HIRC to 12.288 MHz<br>
</div></td></tr><tr><td>
[5:4]</td><td>TRIM_LOOP</td><td><div style="word-wrap: break-word;"><b>Trim Calculation Loop</b><br>
This field defines that trim value calculation is based on how many 32.768 kHz clock.<br>
For example, if TRIM_LOOP is set as 00, auto trim circuit will calculate trim value based on the average frequency difference in 4 32.768 kHz clock.<br>
00 = 4 32.768 kHz clock<br>
01 = 8 32.768 kHz clock<br>
10 = 16 32.768 kHz clock<br>
11 = 32 32.768 kHz clock<br>
</div></td></tr><tr><td>
[7:6]</td><td>TRIM_RETRY_CNT</td><td><div style="word-wrap: break-word;"><b>Trim Value Update Limitation Count</b><br>
This field defines that how many times the auto trim circuit will try to update the HIRC trim value before the frequency of HIRC locked.<br>
Once the HIRC locked, the internal trim value update counter will be reset.<br>
If the trim value update counter reached this limitation value and frequency of HIRC still doesn't lock, the auto trim operation will be disabled and TRIM_SEL will be cleared to 00.<br>
00 = Trim retry count limitation is 64<br>
01 = Trim retry count limitation is 128<br>
10 = Trim retry count limitation is 256<br>
11 = Trim retry count limitation is 512<br>
</div></td></tr><tr><td>
[8]</td><td>ERR_STOP</td><td><div style="word-wrap: break-word;"><b>Trim Stop When 32.768 KHz Error Detected</b><br>
This bit is used to control if stop the HIRC trim operation when 32.768 kHz clock error is detected.<br>
If set this bit high and 32.768 kHz clock error detected, the status 32K_ERR_INT would be set high and HIRC trim operation was stopped.<br>
If this bit is low and 32.768 kHz clock error detected, the status 23K_ERR_INT would be set high and HIRC trim operation is continuously.<br>
0 = Continue the HIRC trim operation even if 32.768 kHz clock error detected.<br>
1 = Stop the HIRC trim operation if 32.768 kHz clock error detected.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IRCTRIMCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IRCTRIMIEN</font><br><p> <font size="2">
Offset: 0x84  HIRC Trim Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[1]</td><td>TRIM_FAIL_IEN</td><td><div style="word-wrap: break-word;"><b>Trim Failure Interrupt Enable</b><br>
This bit controls if an interrupt will be triggered while HIRC trim value update limitation count reached and HIRC frequency still not locked on target frequency set by TRIM_SEL.<br>
If this bit is high and TRIM_FAIL_INT is set during auto trim operation, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached.<br>
0 = TRIM_FAIL_INT status Disabled to trigger an interrupt to CPU.<br>
1 = TRIM_FAIL_INT status Enabled to trigger an interrupt to CPU.<br>
</div></td></tr><tr><td>
[2]</td><td>32K_ERR_IEN</td><td><div style="word-wrap: break-word;"><b>32.768 KHz Clock Error Interrupt Enable</b><br>
This bit controls if CPU would get an interrupt while 32.768 kHz clock is inaccuracy during auto trim operation.<br>
If this bit is high, and 32K_ERR_INT is set during auto trim operation, an interrupt will be triggered to notify the 32.768 kHz clock frequency is inaccuracy.<br>
0 = 32K_ERR_INT status Disabled to trigger an interrupt to CPU.<br>
1 = 32K_ERR_INT status Enabled to trigger an interrupt to CPU.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IRCTRIMIEN;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IRCTRIMINT</font><br><p> <font size="2">
Offset: 0x88  HIRC Trim Interrupt Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>FREQ_LOCK</td><td><div style="word-wrap: break-word;"><b>HIRC Frequency Lock Status</b><br>
This bit indicates the HIRC frequency lock.<br>
This is a status bit and doesn't trigger any interrupt.<br>
</div></td></tr><tr><td>
[1]</td><td>TRIM_FAIL_INT</td><td><div style="word-wrap: break-word;"><b>Trim Failure Interrupt Status</b><br>
This bit indicates that HIRC trim value update limitation count reached and HIRC clock frequency still doesn't lock.<br>
Once this bit is set, the auto trim operation stopped and TRIM_SEL will be cleared to 00 by hardware automatically.<br>
If this bit is set and TRIM_FAIL_IEN is high, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached.<br>
Write 1 to clear this to zero.<br>
0 = Trim value update limitation count doesn't reach.<br>
1 = Trim value update limitation count reached and HIRC frequency still doesn't lock.<br>
</div></td></tr><tr><td>
[2]</td><td>32K_ERR_INT</td><td><div style="word-wrap: break-word;"><b>32.768 KHz Clock Error Interrupt Status</b><br>
This bit indicates that 32.768 kHz clock frequency is inaccuracy.<br>
Once this bit is set, the auto trim operation stopped and TRIM_SEL will be cleared to 00 by hardware automatically.<br>
If this bit is set and 32K_ERR_IEN is high, an interrupt will be triggered to notify the 32.768 kHz clock frequency is inaccuracy.<br>
Write 1 to clear this to zero.<br>
0 = 32.768 kHz clock frequency is accuracy.<br>
1 = 32.768 kHz clock frequency is inaccuracy.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IRCTRIMINT;
    uint32_t RESERVE4[29];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RegLockAddr</font><br><p> <font size="2">
Offset: 0x100  Register Lock Key address</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RegUnLock</td><td><div style="word-wrap: break-word;"><b>Register unlock bit</b><br>
0 = Protected register are Locked. Any write to the target register is ignored.<br>
1 = Protected registers are Unlocked.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t RegLockAddr;

} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */

#define SYS_PDID_PDID_Pos                (0)                                               /*!< SYS_T::PDID: PDID Position                */
#define SYS_PDID_PDID_Msk                (0xfffffffful << SYS_PDID_PDID_Pos)               /*!< SYS_T::PDID: PDID Mask                    */

#define SYS_RST_SRC_RSTS_POR_Pos         (0)                                               /*!< SYS_T::RST_SRC: RSTS_POR Position             */
#define SYS_RST_SRC_RSTS_POR_Msk         (0x1ul << SYS_RST_SRC_RSTS_POR_Pos)               /*!< SYS_T::RST_SRC: RSTS_POR Mask                 */

#define SYS_RST_SRC_RSTS_PAD_Pos         (1)                                               /*!< SYS_T::RST_SRC: RSTS_PAD Position             */
#define SYS_RST_SRC_RSTS_PAD_Msk         (0x1ul << SYS_RST_SRC_RSTS_PAD_Pos)               /*!< SYS_T::RST_SRC: RSTS_PAD Mask                 */

#define SYS_RST_SRC_RSTS_WDT_Pos         (2)                                               /*!< SYS_T::RST_SRC: RSTS_WDT Position             */
#define SYS_RST_SRC_RSTS_WDT_Msk         (0x1ul << SYS_RST_SRC_RSTS_WDT_Pos)               /*!< SYS_T::RST_SRC: RSTS_WDT Mask                 */

#define SYS_RST_SRC_RSTS_BOD_Pos         (4)                                               /*!< SYS_T::RST_SRC: RSTS_BOD Position             */
#define SYS_RST_SRC_RSTS_BOD_Msk         (0x1ul << SYS_RST_SRC_RSTS_BOD_Pos)               /*!< SYS_T::RST_SRC: RSTS_BOD Mask                 */

#define SYS_RST_SRC_RSTS_SYS_Pos         (5)                                               /*!< SYS_T::RST_SRC: RSTS_SYS_T::Position             */
#define SYS_RST_SRC_RSTS_SYS_Msk         (0x1ul << SYS_RST_SRC_RSTS_SYS_Pos)               /*!< SYS_T::RST_SRC: RSTS_SYS_T::Mask                 */

#define SYS_RST_SRC_RSTS_CPU_Pos         (7)                                               /*!< SYS_T::RST_SRC: RSTS_CPU Position             */
#define SYS_RST_SRC_RSTS_CPU_Msk         (0x1ul << SYS_RST_SRC_RSTS_CPU_Pos)               /*!< SYS_T::RST_SRC: RSTS_CPU Mask                 */

#define SYS_IPRST_CTL1_CHIP_RST_Pos      (0)                                               /*!< SYS_T::IPRST_CTL1: CHIP_RST Position            */
#define SYS_IPRST_CTL1_CHIP_RST_Msk      (0x1ul << SYS_IPRST_CTL1_CHIP_RST_Pos)            /*!< SYS_T::IPRST_CTL1: CHIP_RST Mask                */

#define SYS_IPRST_CTL1_CPU_RST_Pos       (1)                                               /*!< SYS_T::IPRST_CTL1: CPU_RST Position             */
#define SYS_IPRST_CTL1_CPU_RST_Msk       (0x1ul << SYS_IPRST_CTL1_CPU_RST_Pos)              /*!< SYS_T::IPRST_CTL1: CPU_RST Mask                 */

#define SYS_IPRST_CTL1_DMA_RST_Pos       (2)                                               /*!< SYS_T::IPRST_CTL1: DMA_RST Position             */
#define SYS_IPRST_CTL1_DMA_RST_Msk       (0x1ul << SYS_IPRST_CTL1_DMA_RST_Pos)             /*!< SYS_T::IPRST_CTL1: DMA_RST Mask                 */

#define SYS_IPRST_CTL1_EBI_RST_Pos       (3)                                               /*!< SYS_T::IPRST_CTL1: EBI_RST Position             */
#define SYS_IPRST_CTL1_EBI_RST_Msk       (0x1ul << SYS_IPRST_CTL1_EBI_RST_Pos)             /*!< SYS_T::IPRST_CTL1: EBI_RST Mask                 */

#define SYS_IPRST_CTL2_GPIO_RST_Pos      (1)                                               /*!< SYS_T::IPRST_CTL2: GPIO_RST Position            */
#define SYS_IPRST_CTL2_GPIO_RST_Msk      (0x1ul << SYS_IPRST_CTL2_GPIO_RST_Pos)            /*!< SYS_T::IPRST_CTL2: GPIO_RST Mask                */

#define SYS_IPRST_CTL2_TMR0_RST_Pos      (2)                                               /*!< SYS_T::IPRST_CTL2: TMR0_RST Position            */
#define SYS_IPRST_CTL2_TMR0_RST_Msk      (0x1ul << SYS_IPRST_CTL2_TMR0_RST_Pos)            /*!< SYS_T::IPRST_CTL2: TMR0_RST Mask                */

#define SYS_IPRST_CTL2_TMR1_RST_Pos      (3)                                               /*!< SYS_T::IPRST_CTL2: TMR1_RST Position            */
#define SYS_IPRST_CTL2_TMR1_RST_Msk      (0x1ul << SYS_IPRST_CTL2_TMR1_RST_Pos)            /*!< SYS_T::IPRST_CTL2: TMR1_RST Mask                */

#define SYS_IPRST_CTL2_TMR2_RST_Pos      (4)                                               /*!< SYS_T::IPRST_CTL2: TMR2_RST Position            */
#define SYS_IPRST_CTL2_TMR2_RST_Msk      (0x1ul << SYS_IPRST_CTL2_TMR2_RST_Pos)            /*!< SYS_T::IPRST_CTL2: TMR2_RST Mask                */

#define SYS_IPRST_CTL2_TMR3_RST_Pos      (5)                                               /*!< SYS_T::IPRST_CTL2: TMR3_RST Position            */
#define SYS_IPRST_CTL2_TMR3_RST_Msk      (0x1ul << SYS_IPRST_CTL2_TMR3_RST_Pos)            /*!< SYS_T::IPRST_CTL2: TMR3_RST Mask                */

#define SYS_IPRST_CTL2_SC2_RST_Pos       (7)                                               /*!< SYS_T::IPRST_CTL2: SC2_RST Position             */
#define SYS_IPRST_CTL2_SC2_RST_Msk       (0x1ul << SYS_IPRST_CTL2_SC2_RST_Pos)             /*!< SYS_T::IPRST_CTL2: SC2_RST Mask                 */

#define SYS_IPRST_CTL2_I2C0_RST_Pos      (8)                                               /*!< SYS_T::IPRST_CTL2: I2C0_RST Position            */
#define SYS_IPRST_CTL2_I2C0_RST_Msk      (0x1ul << SYS_IPRST_CTL2_I2C0_RST_Pos)            /*!< SYS_T::IPRST_CTL2: I2C0_RST Mask                */

#define SYS_IPRST_CTL2_I2C1_RST_Pos      (9)                                               /*!< SYS_T::IPRST_CTL2: I2C1_RST Position            */
#define SYS_IPRST_CTL2_I2C1_RST_Msk      (0x1ul << SYS_IPRST_CTL2_I2C1_RST_Pos)            /*!< SYS_T::IPRST_CTL2: I2C1_RST Mask                */

#define SYS_IPRST_CTL2_SPI0_RST_Pos      (12)                                              /*!< SYS_T::IPRST_CTL2: SPI0_RST Position            */
#define SYS_IPRST_CTL2_SPI0_RST_Msk      (0x1ul << SYS_IPRST_CTL2_SPI0_RST_Pos)            /*!< SYS_T::IPRST_CTL2: SPI0_RST Mask                */

#define SYS_IPRST_CTL2_SPI1_RST_Pos      (13)                                              /*!< SYS_T::IPRST_CTL2: SPI1_RST Position            */
#define SYS_IPRST_CTL2_SPI1_RST_Msk      (0x1ul << SYS_IPRST_CTL2_SPI1_RST_Pos)            /*!< SYS_T::IPRST_CTL2: SPI1_RST Mask                */

#define SYS_IPRST_CTL2_SPI2_RST_Pos      (14)                                              /*!< SYS_T::IPRST_CTL2: SPI2_RST Position            */
#define SYS_IPRST_CTL2_SPI2_RST_Msk      (0x1ul << SYS_IPRST_CTL2_SPI2_RST_Pos)            /*!< SYS_T::IPRST_CTL2: SPI2_RST Mask                */

#define SYS_IPRST_CTL2_UART0_RST_Pos     (16)                                              /*!< SYS_T::IPRST_CTL2: UART0_RST Position           */
#define SYS_IPRST_CTL2_UART0_RST_Msk     (0x1ul << SYS_IPRST_CTL2_UART0_RST_Pos)          /*!< SYS_T::IPRST_CTL2: UART0_RST Mask               */

#define SYS_IPRST_CTL2_UART1_RST_Pos     (17)                                              /*!< SYS_T::IPRST_CTL2: UART1_RST Position           */
#define SYS_IPRST_CTL2_UART1_RST_Msk     (0x1ul << SYS_IPRST_CTL2_UART1_RST_Pos)           /*!< SYS_T::IPRST_CTL2: UART1_RST Mask               */

#define SYS_IPRST_CTL2_PWM0_RST_Pos      (20)                                              /*!< SYS_T::IPRST_CTL2: PWM0_RST Position            */
#define SYS_IPRST_CTL2_PWM0_RST_Msk      (0x1ul << SYS_IPRST_CTL2_PWM0_RST_Pos)            /*!< SYS_T::IPRST_CTL2: PWM0_RST Mask                */

#define SYS_IPRST_CTL2_PWM1_RST_Pos      (21)                                              /*!< SYS_T::IPRST_CTL2: PWM1_RST Position            */
#define SYS_IPRST_CTL2_PWM1_RST_Msk      (0x1ul << SYS_IPRST_CTL2_PWM1_RST_Pos)            /*!< SYS_T::IPRST_CTL2: PWM1_RST Mask                */

#define SYS_IPRST_CTL2_DAC_RST_Pos       (25)                                              /*!< SYS_T::IPRST_CTL2: DAC_RST Position             */
#define SYS_IPRST_CTL2_DAC_RST_Msk       (0x1ul << SYS_IPRST_CTL2_DAC_RST_Pos)             /*!< SYS_T::IPRST_CTL2: DAC_RST Mask                 */

#define SYS_IPRST_CTL2_LCD_RST_Pos       (26)                                              /*!< SYS_T::IPRST_CTL2: LCD_RST Position             */
#define SYS_IPRST_CTL2_LCD_RST_Msk       (0x1ul << SYS_IPRST_CTL2_LCD_RST_Pos)             /*!< SYS_T::IPRST_CTL2: LCD_RST Mask                 */

#define SYS_IPRST_CTL2_USBD_RST_Pos      (27)                                              /*!< SYS_T::IPRST_CTL2: USBD_RST Position            */
#define SYS_IPRST_CTL2_USBD_RST_Msk      (0x1ul << SYS_IPRST_CTL2_USBD_RST_Pos)            /*!< SYS_T::IPRST_CTL2: USBD_RST Mask                */

#define SYS_IPRST_CTL2_ADC_RST_Pos       (28)                                              /*!< SYS_T::IPRST_CTL2: ADC_RST Position             */
#define SYS_IPRST_CTL2_ADC_RST_Msk       (0x1ul << SYS_IPRST_CTL2_ADC_RST_Pos)             /*!< SYS_T::IPRST_CTL2: ADC_RST Mask                 */

#define SYS_IPRST_CTL2_I2S_RST_Pos       (29)                                              /*!< SYS_T::IPRST_CTL2: I2S_RST Position             */
#define SYS_IPRST_CTL2_I2S_RST_Msk       (0x1ul << SYS_IPRST_CTL2_I2S_RST_Pos)             /*!< SYS_T::IPRST_CTL2: I2S_RST Mask                 */

#define SYS_IPRST_CTL2_SC0_RST_Pos       (30)                                              /*!< SYS_T::IPRST_CTL2: SC0_RST Position             */
#define SYS_IPRST_CTL2_SC0_RST_Msk       (0x1ul << SYS_IPRST_CTL2_SC0_RST_Pos)             /*!< SYS_T::IPRST_CTL2: SC0_RST Mask                 */

#define SYS_IPRST_CTL2_SC1_RST_Pos       (31)                                              /*!< SYS_T::IPRST_CTL2: SC1_RST Position             */
#define SYS_IPRST_CTL2_SC1_RST_Msk       (0x1ul << SYS_IPRST_CTL2_SC1_RST_Pos)             /*!< SYS_T::IPRST_CTL2: SC1_RST Mask                 */

#define SYS_TEMPCTL_VTEMP_EN_Pos         (0)                                               /*!< SYS_T::TEMPCTL: VTEMP_EN Position         */
#define SYS_TEMPCTL_VTEMP_EN_Msk         (0x1ul << SYS_TEMPCTL_VTEMP_EN_Pos)               /*!< SYS_T::TEMPCTL: VTEMP_EN Mask             */

#define SYS_PA_L_MFP_PA0_MFP_Pos         (0)                                               /*!< SYS_T::PA_L_MFP: PA0_MFP Position            */
#define SYS_PA_L_MFP_PA0_MFP_Msk         (0x7ul << SYS_PA_L_MFP_PA0_MFP_Pos)               /*!< SYS_T::PA_L_MFP: PA0_MFP Mask                */

#define SYS_PA_L_MFP_PA1_MFP_Pos         (4)                                               /*!< SYS_T::PA_L_MFP: PA1_MFP Position            */
#define SYS_PA_L_MFP_PA1_MFP_Msk         (0x7ul << SYS_PA_L_MFP_PA1_MFP_Pos)               /*!< SYS_T::PA_L_MFP: PA1_MFP Mask                */

#define SYS_PA_L_MFP_PA2_MFP_Pos         (8)                                               /*!< SYS_T::PA_L_MFP: PA2_MFP Position            */
#define SYS_PA_L_MFP_PA2_MFP_Msk         (0x7ul << SYS_PA_L_MFP_PA2_MFP_Pos)               /*!< SYS_T::PA_L_MFP: PA2_MFP Mask                */

#define SYS_PA_L_MFP_PA3_MFP_Pos         (12)                                              /*!< SYS_T::PA_L_MFP: PA3_MFP Position            */
#define SYS_PA_L_MFP_PA3_MFP_Msk         (0x7ul << SYS_PA_L_MFP_PA3_MFP_Pos)               /*!< SYS_T::PA_L_MFP: PA3_MFP Mask                */

#define SYS_PA_L_MFP_PA4_MFP_Pos         (16)                                              /*!< SYS_T::PA_L_MFP: PA4_MFP Position            */
#define SYS_PA_L_MFP_PA4_MFP_Msk         (0x7ul << SYS_PA_L_MFP_PA4_MFP_Pos)               /*!< SYS_T::PA_L_MFP: PA4_MFP Mask                */

#define SYS_PA_L_MFP_PA5_MFP_Pos         (20)                                              /*!< SYS_T::PA_L_MFP: PA5_MFP Position            */
#define SYS_PA_L_MFP_PA5_MFP_Msk         (0x7ul << SYS_PA_L_MFP_PA5_MFP_Pos)               /*!< SYS_T::PA_L_MFP: PA5_MFP Mask                */

#define SYS_PA_L_MFP_PA6_MFP_Pos         (24)                                              /*!< SYS_T::PA_L_MFP: PA6_MFP Position            */
#define SYS_PA_L_MFP_PA6_MFP_Msk         (0x7ul << SYS_PA_L_MFP_PA6_MFP_Pos)               /*!< SYS_T::PA_L_MFP: PA6_MFP Mask                */

#define SYS_PA_L_MFP_PA7_MFP_Pos         (28)                                              /*!< SYS_T::PA_L_MFP: PA7_MFP Position            */
#define SYS_PA_L_MFP_PA7_MFP_Msk         (0x7ul << SYS_PA_L_MFP_PA7_MFP_Pos)               /*!< SYS_T::PA_L_MFP: PA7_MFP Mask                */

#define SYS_PA_H_MFP_PA8_MFP_Pos         (0)                                               /*!< SYS_T::PA_H_MFP: PA8_MFP Position            */
#define SYS_PA_H_MFP_PA8_MFP_Msk         (0x7ul << SYS_PA_H_MFP_PA8_MFP_Pos)               /*!< SYS_T::PA_H_MFP: PA8_MFP Mask                */

#define SYS_PA_H_MFP_PA9_MFP_Pos         (4)                                               /*!< SYS_T::PA_H_MFP: PA9_MFP Position            */
#define SYS_PA_H_MFP_PA9_MFP_Msk         (0x7ul << SYS_PA_H_MFP_PA9_MFP_Pos)               /*!< SYS_T::PA_H_MFP: PA9_MFP Mask                */

#define SYS_PA_H_MFP_PA10_MFP_Pos        (8)                                               /*!< SYS_T::PA_H_MFP: PA10_MFP Position           */
#define SYS_PA_H_MFP_PA10_MFP_Msk        (0x7ul << SYS_PA_H_MFP_PA10_MFP_Pos)              /*!< SYS_T::PA_H_MFP: PA10_MFP Mask               */

#define SYS_PA_H_MFP_PA11_MFP_Pos        (12)                                              /*!< SYS_T::PA_H_MFP: PA11_MFP Position           */
#define SYS_PA_H_MFP_PA11_MFP_Msk        (0x7ul << SYS_PA_H_MFP_PA11_MFP_Pos)              /*!< SYS_T::PA_H_MFP: PA11_MFP Mask               */

#define SYS_PA_H_MFP_PA12_MFP_Pos        (16)                                              /*!< SYS_T::PA_H_MFP: PA12_MFP Position           */
#define SYS_PA_H_MFP_PA12_MFP_Msk        (0x7ul << SYS_PA_H_MFP_PA12_MFP_Pos)              /*!< SYS_T::PA_H_MFP: PA12_MFP Mask               */

#define SYS_PA_H_MFP_PA13_MFP_Pos        (20)                                              /*!< SYS_T::PA_H_MFP: PA13_MFP Position           */
#define SYS_PA_H_MFP_PA13_MFP_Msk        (0x7ul << SYS_PA_H_MFP_PA13_MFP_Pos)              /*!< SYS_T::PA_H_MFP: PA13_MFP Mask               */

#define SYS_PA_H_MFP_PA14_MFP_Pos        (24)                                              /*!< SYS_T::PA_H_MFP: PA14_MFP Position           */
#define SYS_PA_H_MFP_PA14_MFP_Msk        (0x7ul << SYS_PA_H_MFP_PA14_MFP_Pos)              /*!< SYS_T::PA_H_MFP: PA14_MFP Mask               */

#define SYS_PA_H_MFP_PA15_MFP_Pos        (28)                                              /*!< SYS_T::PA_H_MFP: PA15_MFP Position           */
#define SYS_PA_H_MFP_PA15_MFP_Msk        (0x7ul << SYS_PA_H_MFP_PA15_MFP_Pos)              /*!< SYS_T::PA_H_MFP: PA15_MFP Mask               */

#define SYS_PB_L_MFP_PB0_MFP_Pos         (0)                                               /*!< SYS_T::PB_L_MFP: PB0_MFP Position            */
#define SYS_PB_L_MFP_PB0_MFP_Msk         (0x7ul << SYS_PB_L_MFP_PB0_MFP_Pos)               /*!< SYS_T::PB_L_MFP: PB0_MFP Mask                */

#define SYS_PB_L_MFP_PB1_MFP_Pos         (4)                                               /*!< SYS_T::PB_L_MFP: PB1_MFP Position            */
#define SYS_PB_L_MFP_PB1_MFP_Msk         (0x7ul << SYS_PB_L_MFP_PB1_MFP_Pos)               /*!< SYS_T::PB_L_MFP: PB1_MFP Mask                */

#define SYS_PB_L_MFP_PB2_MFP_Pos         (8)                                               /*!< SYS_T::PB_L_MFP: PB2_MFP Position            */
#define SYS_PB_L_MFP_PB2_MFP_Msk         (0x7ul << SYS_PB_L_MFP_PB2_MFP_Pos)               /*!< SYS_T::PB_L_MFP: PB2_MFP Mask                */

#define SYS_PB_L_MFP_PB3_MFP_Pos         (12)                                              /*!< SYS_T::PB_L_MFP: PB3_MFP Position            */
#define SYS_PB_L_MFP_PB3_MFP_Msk         (0x7ul << SYS_PB_L_MFP_PB3_MFP_Pos)               /*!< SYS_T::PB_L_MFP: PB3_MFP Mask                */

#define SYS_PB_L_MFP_PB4_MFP_Pos         (16)                                              /*!< SYS_T::PB_L_MFP: PB4_MFP Position            */
#define SYS_PB_L_MFP_PB4_MFP_Msk         (0x7ul << SYS_PB_L_MFP_PB4_MFP_Pos)               /*!< SYS_T::PB_L_MFP: PB4_MFP Mask                */

#define SYS_PB_L_MFP_PB5_MFP_Pos         (20)                                              /*!< SYS_T::PB_L_MFP: PB5_MFP Position            */
#define SYS_PB_L_MFP_PB5_MFP_Msk         (0x7ul << SYS_PB_L_MFP_PB5_MFP_Pos)               /*!< SYS_T::PB_L_MFP: PB5_MFP Mask                */

#define SYS_PB_L_MFP_PB6_MFP_Pos         (24)                                              /*!< SYS_T::PB_L_MFP: PB6_MFP Position            */
#define SYS_PB_L_MFP_PB6_MFP_Msk         (0x7ul << SYS_PB_L_MFP_PB6_MFP_Pos)               /*!< SYS_T::PB_L_MFP: PB6_MFP Mask                */

#define SYS_PB_L_MFP_PB7_MFP_Pos         (28)                                              /*!< SYS_T::PB_L_MFP: PB7_MFP Position            */
#define SYS_PB_L_MFP_PB7_MFP_Msk         (0x7ul << SYS_PB_L_MFP_PB7_MFP_Pos)               /*!< SYS_T::PB_L_MFP: PB7_MFP Mask                */

#define SYS_PB_H_MFP_PB8_MFP_Pos         (0)                                               /*!< SYS_T::PB_H_MFP: PB8_MFP Position            */
#define SYS_PB_H_MFP_PB8_MFP_Msk         (0x7ul << SYS_PB_H_MFP_PB8_MFP_Pos)               /*!< SYS_T::PB_H_MFP: PB8_MFP Mask                */

#define SYS_PB_H_MFP_PB9_MFP_Pos         (4)                                               /*!< SYS_T::PB_H_MFP: PB9_MFP Position            */
#define SYS_PB_H_MFP_PB9_MFP_Msk         (0x7ul << SYS_PB_H_MFP_PB9_MFP_Pos)               /*!< SYS_T::PB_H_MFP: PB9_MFP Mask                */

#define SYS_PB_H_MFP_PB10_MFP_Pos        (8)                                               /*!< SYS_T::PB_H_MFP: PB10_MFP Position           */
#define SYS_PB_H_MFP_PB10_MFP_Msk        (0x7ul << SYS_PB_H_MFP_PB10_MFP_Pos)              /*!< SYS_T::PB_H_MFP: PB10_MFP Mask               */

#define SYS_PB_H_MFP_PB11_MFP_Pos        (12)                                              /*!< SYS_T::PB_H_MFP: PB11_MFP Position           */
#define SYS_PB_H_MFP_PB11_MFP_Msk        (0x7ul << SYS_PB_H_MFP_PB11_MFP_Pos)              /*!< SYS_T::PB_H_MFP: PB11_MFP Mask               */

#define SYS_PB_H_MFP_PB12_MFP_Pos        (16)                                              /*!< SYS_T::PB_H_MFP: PB12_MFP Position           */
#define SYS_PB_H_MFP_PB12_MFP_Msk        (0x7ul << SYS_PB_H_MFP_PB12_MFP_Pos)              /*!< SYS_T::PB_H_MFP: PB12_MFP Mask               */

#define SYS_PB_H_MFP_PB13_MFP_Pos        (20)                                              /*!< SYS_T::PB_H_MFP: PB13_MFP Position           */
#define SYS_PB_H_MFP_PB13_MFP_Msk        (0x7ul << SYS_PB_H_MFP_PB13_MFP_Pos)              /*!< SYS_T::PB_H_MFP: PB13_MFP Mask               */

#define SYS_PB_H_MFP_PB14_MFP_Pos        (24)                                              /*!< SYS_T::PB_H_MFP: PB14_MFP Position           */
#define SYS_PB_H_MFP_PB14_MFP_Msk        (0x7ul << SYS_PB_H_MFP_PB14_MFP_Pos)              /*!< SYS_T::PB_H_MFP: PB14_MFP Mask               */

#define SYS_PB_H_MFP_PB15_MFP_Pos        (28)                                              /*!< SYS_T::PB_H_MFP: PB15_MFP Position           */
#define SYS_PB_H_MFP_PB15_MFP_Msk        (0x7ul << SYS_PB_H_MFP_PB15_MFP_Pos)              /*!< SYS_T::PB_H_MFP: PB15_MFP Mask               */

#define SYS_PC_L_MFP_PC0_MFP_Pos         (0)                                               /*!< SYS_T::PC_L_MFP: PC0_MFP Position            */
#define SYS_PC_L_MFP_PC0_MFP_Msk         (0x7ul << SYS_PC_L_MFP_PC0_MFP_Pos)               /*!< SYS_T::PC_L_MFP: PC0_MFP Mask                */

#define SYS_PC_L_MFP_PC1_MFP_Pos         (4)                                               /*!< SYS_T::PC_L_MFP: PC1_MFP Position            */
#define SYS_PC_L_MFP_PC1_MFP_Msk         (0x7ul << SYS_PC_L_MFP_PC1_MFP_Pos)               /*!< SYS_T::PC_L_MFP: PC1_MFP Mask                */

#define SYS_PC_L_MFP_PC2_MFP_Pos         (8)                                               /*!< SYS_T::PC_L_MFP: PC2_MFP Position            */
#define SYS_PC_L_MFP_PC2_MFP_Msk         (0x7ul << SYS_PC_L_MFP_PC2_MFP_Pos)               /*!< SYS_T::PC_L_MFP: PC2_MFP Mask                */

#define SYS_PC_L_MFP_PC3_MFP_Pos         (12)                                              /*!< SYS_T::PC_L_MFP: PC3_MFP Position            */
#define SYS_PC_L_MFP_PC3_MFP_Msk         (0x7ul << SYS_PC_L_MFP_PC3_MFP_Pos)               /*!< SYS_T::PC_L_MFP: PC3_MFP Mask                */

#define SYS_PC_L_MFP_PC4_MFP_Pos         (16)                                              /*!< SYS_T::PC_L_MFP: PC4_MFP Position            */
#define SYS_PC_L_MFP_PC4_MFP_Msk         (0x7ul << SYS_PC_L_MFP_PC4_MFP_Pos)               /*!< SYS_T::PC_L_MFP: PC4_MFP Mask                */

#define SYS_PC_L_MFP_PC5_MFP_Pos         (20)                                              /*!< SYS_T::PC_L_MFP: PC5_MFP Position            */
#define SYS_PC_L_MFP_PC5_MFP_Msk         (0x7ul << SYS_PC_L_MFP_PC5_MFP_Pos)               /*!< SYS_T::PC_L_MFP: PC5_MFP Mask                */

#define SYS_PC_L_MFP_PC6_MFP_Pos         (24)                                              /*!< SYS_T::PC_L_MFP: PC6_MFP Position            */
#define SYS_PC_L_MFP_PC6_MFP_Msk         (0x7ul << SYS_PC_L_MFP_PC6_MFP_Pos)               /*!< SYS_T::PC_L_MFP: PC6_MFP Mask                */

#define SYS_PC_L_MFP_PC7_MFP_Pos         (28)                                              /*!< SYS_T::PC_L_MFP: PC7_MFP Position            */
#define SYS_PC_L_MFP_PC7_MFP_Msk         (0x7ul << SYS_PC_L_MFP_PC7_MFP_Pos)               /*!< SYS_T::PC_L_MFP: PC7_MFP Mask                */

#define SYS_PC_H_MFP_PC8_MFP_Pos         (0)                                               /*!< SYS_T::PC_H_MFP: PC8_MFP Position            */
#define SYS_PC_H_MFP_PC8_MFP_Msk         (0x7ul << SYS_PC_H_MFP_PC8_MFP_Pos)               /*!< SYS_T::PC_H_MFP: PC8_MFP Mask                */

#define SYS_PC_H_MFP_PC9_MFP_Pos         (4)                                               /*!< SYS_T::PC_H_MFP: PC9_MFP Position            */
#define SYS_PC_H_MFP_PC9_MFP_Msk         (0x7ul << SYS_PC_H_MFP_PC9_MFP_Pos)               /*!< SYS_T::PC_H_MFP: PC9_MFP Mask                */

#define SYS_PC_H_MFP_PC10_MFP_Pos        (8)                                               /*!< SYS_T::PC_H_MFP: PC10_MFP Position           */
#define SYS_PC_H_MFP_PC10_MFP_Msk        (0x7ul << SYS_PC_H_MFP_PC10_MFP_Pos)              /*!< SYS_T::PC_H_MFP: PC10_MFP Mask               */

#define SYS_PC_H_MFP_PC11_MFP_Pos        (12)                                              /*!< SYS_T::PC_H_MFP: PC11_MFP Position           */
#define SYS_PC_H_MFP_PC11_MFP_Msk        (0x7ul << SYS_PC_H_MFP_PC11_MFP_Pos)              /*!< SYS_T::PC_H_MFP: PC11_MFP Mask               */

#define SYS_PC_H_MFP_PC12_MFP_Pos        (16)                                              /*!< SYS_T::PC_H_MFP: PC12_MFP Position           */
#define SYS_PC_H_MFP_PC12_MFP_Msk        (0x7ul << SYS_PC_H_MFP_PC12_MFP_Pos)              /*!< SYS_T::PC_H_MFP: PC12_MFP Mask               */

#define SYS_PC_H_MFP_PC13_MFP_Pos        (20)                                              /*!< SYS_T::PC_H_MFP: PC13_MFP Position           */
#define SYS_PC_H_MFP_PC13_MFP_Msk        (0x7ul << SYS_PC_H_MFP_PC13_MFP_Pos)              /*!< SYS_T::PC_H_MFP: PC13_MFP Mask               */

#define SYS_PC_H_MFP_PC14_MFP_Pos        (24)                                              /*!< SYS_T::PC_H_MFP: PC14_MFP Position           */
#define SYS_PC_H_MFP_PC14_MFP_Msk        (0x7ul << SYS_PC_H_MFP_PC14_MFP_Pos)              /*!< SYS_T::PC_H_MFP: PC14_MFP Mask               */

#define SYS_PC_H_MFP_PC15_MFP_Pos        (28)                                              /*!< SYS_T::PC_H_MFP: PC15_MFP Position           */
#define SYS_PC_H_MFP_PC15_MFP_Msk        (0x7ul << SYS_PC_H_MFP_PC15_MFP_Pos)              /*!< SYS_T::PC_H_MFP: PC15_MFP Mask               */

#define SYS_PD_L_MFP_PD0_MFP_Pos         (0)                                               /*!< SYS_T::PD_L_MFP: PD0_MFP Position            */
#define SYS_PD_L_MFP_PD0_MFP_Msk         (0x7ul << SYS_PD_L_MFP_PD0_MFP_Pos)               /*!< SYS_T::PD_L_MFP: PD0_MFP Mask                */

#define SYS_PD_L_MFP_PD1_MFP_Pos         (4)                                               /*!< SYS_T::PD_L_MFP: PD1_MFP Position            */
#define SYS_PD_L_MFP_PD1_MFP_Msk         (0x7ul << SYS_PD_L_MFP_PD1_MFP_Pos)               /*!< SYS_T::PD_L_MFP: PD1_MFP Mask                */

#define SYS_PD_L_MFP_PD2_MFP_Pos         (8)                                               /*!< SYS_T::PD_L_MFP: PD2_MFP Position            */
#define SYS_PD_L_MFP_PD2_MFP_Msk         (0x7ul << SYS_PD_L_MFP_PD2_MFP_Pos)               /*!< SYS_T::PD_L_MFP: PD2_MFP Mask                */

#define SYS_PD_L_MFP_PD3_MFP_Pos         (12)                                              /*!< SYS_T::PD_L_MFP: PD3_MFP Position            */
#define SYS_PD_L_MFP_PD3_MFP_Msk         (0x7ul << SYS_PD_L_MFP_PD3_MFP_Pos)               /*!< SYS_T::PD_L_MFP: PD3_MFP Mask                */

#define SYS_PD_L_MFP_PD4_MFP_Pos         (16)                                              /*!< SYS_T::PD_L_MFP: PD4_MFP Position            */
#define SYS_PD_L_MFP_PD4_MFP_Msk         (0x7ul << SYS_PD_L_MFP_PD4_MFP_Pos)               /*!< SYS_T::PD_L_MFP: PD4_MFP Mask                */

#define SYS_PD_L_MFP_PD5_MFP_Pos         (20)                                              /*!< SYS_T::PD_L_MFP: PD5_MFP Position            */
#define SYS_PD_L_MFP_PD5_MFP_Msk         (0x7ul << SYS_PD_L_MFP_PD5_MFP_Pos)               /*!< SYS_T::PD_L_MFP: PD5_MFP Mask                */

#define SYS_PD_L_MFP_PD6_MFP_Pos         (24)                                              /*!< SYS_T::PD_L_MFP: PD6_MFP Position            */
#define SYS_PD_L_MFP_PD6_MFP_Msk         (0x7ul << SYS_PD_L_MFP_PD6_MFP_Pos)               /*!< SYS_T::PD_L_MFP: PD6_MFP Mask                */

#define SYS_PD_L_MFP_PD7_MFP_Pos         (28)                                              /*!< SYS_T::PD_L_MFP: PD7_MFP Position            */
#define SYS_PD_L_MFP_PD7_MFP_Msk         (0x7ul << SYS_PD_L_MFP_PD7_MFP_Pos)               /*!< SYS_T::PD_L_MFP: PD7_MFP Mask                */

#define SYS_PD_H_MFP_PD8_MFP_Pos         (0)                                               /*!< SYS_T::PD_H_MFP: PD8_MFP Position            */
#define SYS_PD_H_MFP_PD8_MFP_Msk         (0x7ul << SYS_PD_H_MFP_PD8_MFP_Pos)               /*!< SYS_T::PD_H_MFP: PD8_MFP Mask                */

#define SYS_PD_H_MFP_PD9_MFP_Pos         (4)                                               /*!< SYS_T::PD_H_MFP: PD9_MFP Position            */
#define SYS_PD_H_MFP_PD9_MFP_Msk         (0x7ul << SYS_PD_H_MFP_PD9_MFP_Pos)               /*!< SYS_T::PD_H_MFP: PD9_MFP Mask                */

#define SYS_PD_H_MFP_PD10_MFP_Pos        (8)                                               /*!< SYS_T::PD_H_MFP: PD10_MFP Position           */
#define SYS_PD_H_MFP_PD10_MFP_Msk        (0x7ul << SYS_PD_H_MFP_PD10_MFP_Pos)              /*!< SYS_T::PD_H_MFP: PD10_MFP Mask               */

#define SYS_PD_H_MFP_PD11_MFP_Pos        (12)                                              /*!< SYS_T::PD_H_MFP: PD11_MFP Position           */
#define SYS_PD_H_MFP_PD11_MFP_Msk        (0x7ul << SYS_PD_H_MFP_PD11_MFP_Pos)              /*!< SYS_T::PD_H_MFP: PD11_MFP Mask               */

#define SYS_PD_H_MFP_PD12_MFP_Pos        (16)                                              /*!< SYS_T::PD_H_MFP: PD12_MFP Position           */
#define SYS_PD_H_MFP_PD12_MFP_Msk        (0x7ul << SYS_PD_H_MFP_PD12_MFP_Pos)              /*!< SYS_T::PD_H_MFP: PD12_MFP Mask               */

#define SYS_PD_H_MFP_PD13_MFP_Pos        (20)                                              /*!< SYS_T::PD_H_MFP: PD13_MFP Position           */
#define SYS_PD_H_MFP_PD13_MFP_Msk        (0x7ul << SYS_PD_H_MFP_PD13_MFP_Pos)              /*!< SYS_T::PD_H_MFP: PD13_MFP Mask               */

#define SYS_PD_H_MFP_PD14_MFP_Pos        (24)                                              /*!< SYS_T::PD_H_MFP: PD14_MFP Position           */
#define SYS_PD_H_MFP_PD14_MFP_Msk        (0x7ul << SYS_PD_H_MFP_PD14_MFP_Pos)              /*!< SYS_T::PD_H_MFP: PD14_MFP Mask               */

#define SYS_PD_H_MFP_PD15_MFP_Pos        (28)                                              /*!< SYS_T::PD_H_MFP: PD15_MFP Position           */
#define SYS_PD_H_MFP_PD15_MFP_Msk        (0x7ul << SYS_PD_H_MFP_PD15_MFP_Pos)              /*!< SYS_T::PD_H_MFP: PD15_MFP Mask               */

#define SYS_PE_L_MFP_PE0_MFP_Pos         (0)                                               /*!< SYS_T::PE_L_MFP: PE0_MFP Position            */
#define SYS_PE_L_MFP_PE0_MFP_Msk         (0x7ul << SYS_PE_L_MFP_PE0_MFP_Pos)               /*!< SYS_T::PE_L_MFP: PE0_MFP Mask                */

#define SYS_PE_L_MFP_PE1_MFP_Pos         (4)                                               /*!< SYS_T::PE_L_MFP: PE1_MFP Position            */
#define SYS_PE_L_MFP_PE1_MFP_Msk         (0x7ul << SYS_PE_L_MFP_PE1_MFP_Pos)               /*!< SYS_T::PE_L_MFP: PE1_MFP Mask                */

#define SYS_PE_L_MFP_PE2_MFP_Pos         (8)                                               /*!< SYS_T::PE_L_MFP: PE2_MFP Position            */
#define SYS_PE_L_MFP_PE2_MFP_Msk         (0x7ul << SYS_PE_L_MFP_PE2_MFP_Pos)               /*!< SYS_T::PE_L_MFP: PE2_MFP Mask                */

#define SYS_PE_L_MFP_PE3_MFP_Pos         (12)                                              /*!< SYS_T::PE_L_MFP: PE3_MFP Position            */
#define SYS_PE_L_MFP_PE3_MFP_Msk         (0x7ul << SYS_PE_L_MFP_PE3_MFP_Pos)               /*!< SYS_T::PE_L_MFP: PE3_MFP Mask                */

#define SYS_PE_L_MFP_PE4_MFP_Pos         (16)                                              /*!< SYS_T::PE_L_MFP: PE4_MFP Position            */
#define SYS_PE_L_MFP_PE4_MFP_Msk         (0x7ul << SYS_PE_L_MFP_PE4_MFP_Pos)               /*!< SYS_T::PE_L_MFP: PE4_MFP Mask                */

#define SYS_PE_L_MFP_PE5_MFP_Pos         (20)                                              /*!< SYS_T::PE_L_MFP: PE5_MFP Position            */
#define SYS_PE_L_MFP_PE5_MFP_Msk         (0x7ul << SYS_PE_L_MFP_PE5_MFP_Pos)               /*!< SYS_T::PE_L_MFP: PE5_MFP Mask                */

#define SYS_PE_L_MFP_PE6_MFP_Pos         (24)                                              /*!< SYS_T::PE_L_MFP: PE6_MFP Position            */
#define SYS_PE_L_MFP_PE6_MFP_Msk         (0x7ul << SYS_PE_L_MFP_PE6_MFP_Pos)               /*!< SYS_T::PE_L_MFP: PE6_MFP Mask                */

#define SYS_PE_L_MFP_PE7_MFP_Pos         (28)                                              /*!< SYS_T::PE_L_MFP: PE7_MFP Position            */
#define SYS_PE_L_MFP_PE7_MFP_Msk         (0x7ul << SYS_PE_L_MFP_PE7_MFP_Pos)               /*!< SYS_T::PE_L_MFP: PE7_MFP Mask                */

#define SYS_PE_H_MFP_PE8_MFP_Pos         (0)                                               /*!< SYS_T::PE_H_MFP: PE8_MFP Position            */
#define SYS_PE_H_MFP_PE8_MFP_Msk         (0x7ul << SYS_PE_H_MFP_PE8_MFP_Pos)               /*!< SYS_T::PE_H_MFP: PE8_MFP Mask                */

#define SYS_PE_H_MFP_PE9_MFP_Pos         (4)                                               /*!< SYS_T::PE_H_MFP: PE9_MFP Position            */
#define SYS_PE_H_MFP_PE9_MFP_Msk         (0x7ul << SYS_PE_H_MFP_PE9_MFP_Pos)               /*!< SYS_T::PE_H_MFP: PE9_MFP Mask                */

#define SYS_PE_H_MFP_PE10_MFP_Pos        (8)                                               /*!< SYS_T::PE_H_MFP: PE10_MFP Position           */
#define SYS_PE_H_MFP_PE10_MFP_Msk        (0x7ul << SYS_PE_H_MFP_PE10_MFP_Pos)              /*!< SYS_T::PE_H_MFP: PE10_MFP Mask               */

#define SYS_PE_H_MFP_PE11_MFP_Pos        (12)                                              /*!< SYS_T::PE_H_MFP: PE11_MFP Position           */
#define SYS_PE_H_MFP_PE11_MFP_Msk        (0x7ul << SYS_PE_H_MFP_PE11_MFP_Pos)              /*!< SYS_T::PE_H_MFP: PE11_MFP Mask               */

#define SYS_PE_H_MFP_PE12_MFP_Pos        (16)                                              /*!< SYS_T::PE_H_MFP: PE12_MFP Position           */
#define SYS_PE_H_MFP_PE12_MFP_Msk        (0x7ul << SYS_PE_H_MFP_PE12_MFP_Pos)              /*!< SYS_T::PE_H_MFP: PE12_MFP Mask               */

#define SYS_PE_H_MFP_PE13_MFP_Pos        (20)                                              /*!< SYS_T::PE_H_MFP: PE13_MFP Position           */
#define SYS_PE_H_MFP_PE13_MFP_Msk        (0x7ul << SYS_PE_H_MFP_PE13_MFP_Pos)              /*!< SYS_T::PE_H_MFP: PE13_MFP Mask               */

#define SYS_PE_H_MFP_PE14_MFP_Pos        (24)                                              /*!< SYS_T::PE_H_MFP: PE14_MFP Position           */
#define SYS_PE_H_MFP_PE14_MFP_Msk        (0x7ul << SYS_PE_H_MFP_PE14_MFP_Pos)              /*!< SYS_T::PE_H_MFP: PE14_MFP Mask               */

#define SYS_PE_H_MFP_PE15_MFP_Pos        (28)                                              /*!< SYS_T::PE_H_MFP: PE15_MFP Position           */
#define SYS_PE_H_MFP_PE15_MFP_Msk        (0x7ul << SYS_PE_H_MFP_PE15_MFP_Pos)              /*!< SYS_T::PE_H_MFP: PE15_MFP Mask               */

#define SYS_PF_L_MFP_PF0_MFP_Pos         (0)                                               /*!< SYS_T::PF_L_MFP: PF0_MFP Position            */
#define SYS_PF_L_MFP_PF0_MFP_Msk         (0x7ul << SYS_PF_L_MFP_PF0_MFP_Pos)               /*!< SYS_T::PF_L_MFP: PF0_MFP Mask                */

#define SYS_PF_L_MFP_PF1_MFP_Pos         (4)                                               /*!< SYS_T::PF_L_MFP: PF1_MFP Position            */
#define SYS_PF_L_MFP_PF1_MFP_Msk         (0x7ul << SYS_PF_L_MFP_PF1_MFP_Pos)               /*!< SYS_T::PF_L_MFP: PF1_MFP Mask                */

#define SYS_PF_L_MFP_PF2_MFP_Pos         (8)                                               /*!< SYS_T::PF_L_MFP: PF2_MFP Position            */
#define SYS_PF_L_MFP_PF2_MFP_Msk         (0x7ul << SYS_PF_L_MFP_PF2_MFP_Pos)               /*!< SYS_T::PF_L_MFP: PF2_MFP Mask                */

#define SYS_PF_L_MFP_PF3_MFP_Pos         (12)                                              /*!< SYS_T::PF_L_MFP: PF3_MFP Position            */
#define SYS_PF_L_MFP_PF3_MFP_Msk         (0x7ul << SYS_PF_L_MFP_PF3_MFP_Pos)               /*!< SYS_T::PF_L_MFP: PF3_MFP Mask                */

#define SYS_PF_L_MFP_PF4_MFP_Pos         (16)                                              /*!< SYS_T::PF_L_MFP: PF4_MFP Position            */
#define SYS_PF_L_MFP_PF4_MFP_Msk         (0x7ul << SYS_PF_L_MFP_PF4_MFP_Pos)               /*!< SYS_T::PF_L_MFP: PF4_MFP Mask                */

#define SYS_PF_L_MFP_PF5_MFP_Pos         (20)                                              /*!< SYS_T::PF_L_MFP: PF5_MFP Position            */
#define SYS_PF_L_MFP_PF5_MFP_Msk         (0x7ul << SYS_PF_L_MFP_PF5_MFP_Pos)               /*!< SYS_T::PF_L_MFP: PF5_MFP Mask                */

#define SYS_PORCTL_POR_DIS_CODE_Pos      (0)                                               /*!< SYS_T::PORCTL: POR_DIS_CODE Position      */
#define SYS_PORCTL_POR_DIS_CODE_Msk      (0xfffful << SYS_PORCTL_POR_DIS_CODE_Pos)         /*!< SYS_T::PORCTL: POR_DIS_CODE Mask          */

#define SYS_BODCTL_BOD17_EN_Pos          (0)                                               /*!< SYS_T::BODCTL: BOD17_EN Position          */
#define SYS_BODCTL_BOD17_EN_Msk          (0x1ul << SYS_BODCTL_BOD17_EN_Pos)                /*!< SYS_T::BODCTL: BOD17_EN Mask              */

#define SYS_BODCTL_BOD20_EN_Pos          (1)                                               /*!< SYS_T::BODCTL: BOD20_EN Position          */
#define SYS_BODCTL_BOD20_EN_Msk          (0x1ul << SYS_BODCTL_BOD20_EN_Pos)                /*!< SYS_T::BODCTL: BOD20_EN Mask              */

#define SYS_BODCTL_BOD25_EN_Pos          (2)                                               /*!< SYS_T::BODCTL: BOD25_EN Position          */
#define SYS_BODCTL_BOD25_EN_Msk          (0x1ul << SYS_BODCTL_BOD25_EN_Pos)                /*!< SYS_T::BODCTL: BOD25_EN Mask              */

#define SYS_BODCTL_BOD17_RST_EN_Pos      (4)                                               /*!< SYS_T::BODCTL: BOD17_RST_EN Position      */
#define SYS_BODCTL_BOD17_RST_EN_Msk      (0x1ul << SYS_BODCTL_BOD17_RST_EN_Pos)            /*!< SYS_T::BODCTL: BOD17_RST_EN Mask          */

#define SYS_BODCTL_BOD20_RST_EN_Pos      (5)                                               /*!< SYS_T::BODCTL: BOD20_RST_EN Position      */
#define SYS_BODCTL_BOD20_RST_EN_Msk      (0x1ul << SYS_BODCTL_BOD20_RST_EN_Pos)            /*!< SYS_T::BODCTL: BOD20_RST_EN Mask          */

#define SYS_BODCTL_BOD25_RST_EN_Pos      (6)                                               /*!< SYS_T::BODCTL: BOD25_RST_EN Position      */
#define SYS_BODCTL_BOD25_RST_EN_Msk      (0x1ul << SYS_BODCTL_BOD25_RST_EN_Pos)            /*!< SYS_T::BODCTL: BOD25_RST_EN Mask          */

#define SYS_BODCTL_BOD17_INT_EN_Pos      (8)                                               /*!< SYS_T::BODCTL: BOD17_INT_EN Position      */
#define SYS_BODCTL_BOD17_INT_EN_Msk      (0x1ul << SYS_BODCTL_BOD17_INT_EN_Pos)            /*!< SYS_T::BODCTL: BOD17_INT_EN Mask          */

#define SYS_BODCTL_BOD20_INT_EN_Pos      (9)                                               /*!< SYS_T::BODCTL: BOD20_INT_EN Position      */
#define SYS_BODCTL_BOD20_INT_EN_Msk      (0x1ul << SYS_BODCTL_BOD20_INT_EN_Pos)            /*!< SYS_T::BODCTL: BOD20_INT_EN Mask          */

#define SYS_BODCTL_BOD25_INT_EN_Pos      (10)                                              /*!< SYS_T::BODCTL: BOD25_INT_EN Position      */
#define SYS_BODCTL_BOD25_INT_EN_Msk      (0x1ul << SYS_BODCTL_BOD25_INT_EN_Pos)            /*!< SYS_T::BODCTL: BOD25_INT_EN Mask          */

#define SYS_BODSTS_BOD_INT_Pos           (0)                                               /*!< SYS_T::BODSTS: BOD_INT Position           */
#define SYS_BODSTS_BOD_INT_Msk           (0x1ul << SYS_BODSTS_BOD_INT_Pos)                 /*!< SYS_T::BODSTS: BOD_INT Mask               */

#define SYS_BODSTS_BOD17_drop_Pos        (1)                                               /*!< SYS_T::BODSTS: BOD17_drop Position        */
#define SYS_BODSTS_BOD17_drop_Msk        (0x1ul << SYS_BODSTS_BOD17_drop_Pos)              /*!< SYS_T::BODSTS: BOD17_drop Mask            */

#define SYS_BODSTS_BOD20_drop_Pos        (2)                                               /*!< SYS_T::BODSTS: BOD20_drop Position        */
#define SYS_BODSTS_BOD20_drop_Msk        (0x1ul << SYS_BODSTS_BOD20_drop_Pos)              /*!< SYS_T::BODSTS: BOD20_drop Mask            */

#define SYS_BODSTS_BOD25_drop_Pos        (3)                                               /*!< SYS_T::BODSTS: BOD25_drop Position        */
#define SYS_BODSTS_BOD25_drop_Msk        (0x1ul << SYS_BODSTS_BOD25_drop_Pos)              /*!< SYS_T::BODSTS: BOD25_drop Mask            */

#define SYS_BODSTS_BOD17_rise_Pos        (4)                                               /*!< SYS_T::BODSTS: BOD17_rise Position        */
#define SYS_BODSTS_BOD17_rise_Msk        (0x1ul << SYS_BODSTS_BOD17_rise_Pos)              /*!< SYS_T::BODSTS: BOD17_rise Mask            */

#define SYS_BODSTS_BOD20_rise_Pos        (5)                                               /*!< SYS_T::BODSTS: BOD20_rise Position        */
#define SYS_BODSTS_BOD20_rise_Msk        (0x1ul << SYS_BODSTS_BOD20_rise_Pos)              /*!< SYS_T::BODSTS: BOD20_rise Mask            */

#define SYS_BODSTS_BOD25_rise_Pos        (6)                                               /*!< SYS_T::BODSTS: BOD25_rise Position        */
#define SYS_BODSTS_BOD25_rise_Msk        (0x1ul << SYS_BODSTS_BOD25_rise_Pos)              /*!< SYS_T::BODSTS: BOD25_rise Mask            */

#define SYS_VREFCTL_BGP_EN_Pos           (0)                                               /*!< SYS_T::VREFCTL: BGP_EN Position           */
#define SYS_VREFCTL_BGP_EN_Msk           (0x1ul << SYS_VREFCTL_BGP_EN_Pos)                 /*!< SYS_T::VREFCTL: BGP_EN Mask               */

#define SYS_VREFCTL_REG_EN_Pos           (1)                                               /*!< SYS_T::VREFCTL: REG_EN Position           */
#define SYS_VREFCTL_REG_EN_Msk           (0x1ul << SYS_VREFCTL_REG_EN_Pos)                 /*!< SYS_T::VREFCTL: REG_EN Mask               */

#define SYS_VREFCTL_SEL25_Pos            (2)                                               /*!< SYS_T::VREFCTL: SEL25 Position            */
#define SYS_VREFCTL_SEL25_Msk            (0x1ul << SYS_VREFCTL_SEL25_Pos)                  /*!< SYS_T::VREFCTL: SEL25 Mask                */

#define SYS_VREFCTL_EXT_MODE_Pos         (3)                                               /*!< SYS_T::VREFCTL: EXT_MODE Position         */
#define SYS_VREFCTL_EXT_MODE_Msk         (0x1ul << SYS_VREFCTL_EXT_MODE_Pos)               /*!< SYS_T::VREFCTL: EXT_MODE Mask             */

#define SYS_IRCTRIMCTL_TRIM_SEL_Pos      (0)                                               /*!< SYS_T::IRCTRIMCTL: TRIM_SEL Position      */
#define SYS_IRCTRIMCTL_TRIM_SEL_Msk      (0x3ul << SYS_IRCTRIMCTL_TRIM_SEL_Pos)            /*!< SYS_T::IRCTRIMCTL: TRIM_SEL Mask          */

#define SYS_IRCTRIMCTL_TRIM_LOOP_Pos     (4)                                               /*!< SYS_T::IRCTRIMCTL: TRIM_LOOP Position     */
#define SYS_IRCTRIMCTL_TRIM_LOOP_Msk     (0x3ul << SYS_IRCTRIMCTL_TRIM_LOOP_Pos)           /*!< SYS_T::IRCTRIMCTL: TRIM_LOOP Mask         */

#define SYS_IRCTRIMCTL_TRIM_RETRY_CNT_Pos (6)                                              /*!< SYS_T::IRCTRIMCTL: TRIM_RETRY_CNT Position*/
#define SYS_IRCTRIMCTL_TRIM_RETRY_CNT_Msk (0x3ul << SYS_IRCTRIMCTL_TRIM_RETRY_CNT_Pos)     /*!< SYS_T::IRCTRIMCTL: TRIM_RETRY_CNT Mask    */

#define SYS_IRCTRIMCTL_ERR_STOP_Pos      (8)                                               /*!< SYS_T::IRCTRIMCTL: ERR_STOP Position      */
#define SYS_IRCTRIMCTL_ERR_STOP_Msk      (0x1ul << SYS_IRCTRIMCTL_ERR_STOP_Pos)            /*!< SYS_T::IRCTRIMCTL: ERR_STOP Mask          */

#define SYS_IRCTRIMIEN_TRIM_FAIL_IEN_Pos (1)                                               /*!< SYS_T::IRCTRIMIEN: TRIM_FAIL_IEN Position */
#define SYS_IRCTRIMIEN_TRIM_FAIL_IEN_Msk (0x1ul << SYS_IRCTRIMIEN_TRIM_FAIL_IEN_Pos)       /*!< SYS_T::IRCTRIMIEN: TRIM_FAIL_IEN Mask     */

#define SYS_IRCTRIMIEN_32K_ERR_IEN_Pos   (2)                                               /*!< SYS_T::IRCTRIMIEN: 32K_ERR_IEN Position   */
#define SYS_IRCTRIMIEN_32K_ERR_IEN_Msk   (0x1ul << SYS_IRCTRIMIEN_32K_ERR_IEN_Pos)         /*!< SYS_T::IRCTRIMIEN: 32K_ERR_IEN Mask       */

#define SYS_IRCTRIMINT_FREQ_LOCK_Pos     (0)                                               /*!< SYS_T::IRCTRIMINT: FREQ_LOCK Position     */
#define SYS_IRCTRIMINT_FREQ_LOCK_Msk     (0x1ul << SYS_IRCTRIMINT_FREQ_LOCK_Pos)           /*!< SYS_T::IRCTRIMINT: FREQ_LOCK Mask         */

#define SYS_IRCTRIMINT_TRIM_FAIL_INT_Pos (1)                                               /*!< SYS_T::IRCTRIMINT: TRIM_FAIL_INT Position */
#define SYS_IRCTRIMINT_TRIM_FAIL_INT_Msk (0x1ul << SYS_IRCTRIMINT_TRIM_FAIL_INT_Pos)       /*!< SYS_T::IRCTRIMINT: TRIM_FAIL_INT Mask     */

#define SYS_IRCTRIMINT_32K_ERR_INT_Pos   (2)                                               /*!< SYS_T::IRCTRIMINT: 32K_ERR_INT Position   */
#define SYS_IRCTRIMINT_32K_ERR_INT_Msk   (0x1ul << SYS_IRCTRIMINT_32K_ERR_INT_Pos)         /*!< SYS_T::IRCTRIMINT: 32K_ERR_INT Mask       */

#define SYS_RegLockAddr_RegUnLock_Pos    (0)                                               /*!< SYS_T::RegLockAddr: RegUnLock Position    */
#define SYS_RegLockAddr_RegUnLock_Msk    (0x1ul << SYS_RegLockAddr_RegUnLock_Pos)          /*!< SYS_T::RegLockAddr: RegUnLock Mask        */

/**@}*/ /* SYS_CONST */
/**@}*/ /* end of SYS register group */


/*---------------------- General Purpose Input/Output Controller -------------------------*/
/**
    @addtogroup GPIO General Purpose Input/Output Controller(GPIO)
    Memory Mapped Structure for GPIO Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PMD</font><br><p> <font size="2">
Offset: 0x00  GPIO Port Pin I/O Mode Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[1:0]</td><td>PMD0</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[3:2]</td><td>PMD1</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[5:4]</td><td>PMD2</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[7:6]</td><td>PMD3</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[9:8]</td><td>PMD4</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[11:10]</td><td>PMD5</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[13:12]</td><td>PMD6</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[15:14]</td><td>PMD7</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[17:16]</td><td>PMD8</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[19:18]</td><td>PMD9</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[21:20]</td><td>PMD10</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[23:22]</td><td>PMD11</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[25:24]</td><td>PMD12</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[27:26]</td><td>PMD13</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[29:28]</td><td>PMD14</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr><tr><td>
[31:30]</td><td>PMD15</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Mode Control</b><br>
Determine the I/O type of GPIO port [x] pin [n]<br>
00 = GPIO port [x] pin [n] is in INPUT mode.<br>
01 = GPIO port [x] pin [n] is in OUTPUT mode.<br>
10 = GPIO port [x] pin [n] is in Open-Drain mode.<br>
11 = Reserved.<br>
Note: For GPIOF_PMD, PMD6 ~ PMD15 are reserved.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PMD;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">OFFD</font><br><p> <font size="2">
Offset: 0x04  GPIO Port Pin OFF Digital Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:16]</td><td>OFFD</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Digital Input Path Disable</b><br>
Determine if the digital input path of GPIO port [x] pin [n] is disabled.<br>
0 = Digital input path of GPIO port [x] pin [n] Enabled.<br>
1 = Digital input path of GPIO port [x] pin [n] Disabled (tied digital input to low).<br>
Note: For GPIOF_OFFD, bits [31:22] are reserved.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t OFFD;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DOUT</font><br><p> <font size="2">
Offset: 0x08  GPIO Port Data Output Value Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>DOUT</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Output Value</b><br>
Each of these bits controls the status of a GPIO port [x] pin [n] when the GPI/O pin is configures as output or open-drain mode<br>
0 = GPIO port [x] Pin [n] will drive Low if the corresponding output mode enabling bit is set.<br>
1 = GPIO port [x] Pin [n] will drive High if the corresponding output mode enabling bit is set.<br>
Note: For GPIOF_DOUT, bits [15:6] are reserved.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DOUT;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DMASK</font><br><p> <font size="2">
Offset: 0x0C  GPIO Port Data Output Write Mask Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>DMASK</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Data Output Write Mask</b><br>
These bits are used to protect the corresponding register of GPIOx_DOUT bit [n].<br>
When set the DMASK[n] to "1", the corresponding DOUT[n] bit is protected.<br>
The write signal is masked, write data to the protect bit is ignored.<br>
0 = The corresponding GPIO_DOUT bit [n] can be updated.<br>
1 = The corresponding GPIO_DOUT bit [n] is protected.<br>
Note: For GPIOF_DMASK, bits [15:6] are reserved.<br>
Note: These mask bits only take effect while CPU is doing write operation to register GPIOx_DOUT.<br>
If CPU is doing write operation to register GPIO[x][n], these mask bits will not take effect.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DMASK;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PIN</font><br><p> <font size="2">
Offset: 0x10  GPIO Port Pin Value Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>PIN</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Value</b><br>
The value read from each of these bit reflects the actual status of the respective GPI/O pin<br>
Note: For GPIOF_PIN, bits [15:6] are reserved.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t PIN;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DBEN</font><br><p> <font size="2">
Offset: 0x14  GPIO Port De-bounce Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>DBEN</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Input Signal De-Bounce Enable</b><br>
DBEN[n] used to enable the de-bounce function for each corresponding bit.<br>
If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle the input signal transition is seen as the signal bounce and will not trigger the interrupt.<br>
DBEN[n] is used for "edge-trigger" interrupt only, and ignored for "level trigger" interrupt<br>
0 = The GPIO port [x] Pin [n] input signal de-bounce function is disabled.<br>
1 = The GPIO port [x] Pin [n] input signal de-bounce function is enabled.<br>
The de-bounce function is valid for edge triggered interrupt.<br>
If the interrupt mode is level triggered, the de-bounce enable bit is ignored.<br>
Note: For GPIOF_DBEN, bits [15:6] are reserved.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DBEN;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IMD</font><br><p> <font size="2">
Offset: 0x18  GPIO Port Interrupt Mode Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>IMD</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Edge Or Level Detection Interrupt Control</b><br>
IMD[n] used to control the interrupt is by level trigger or by edge trigger.<br>
If the interrupt is by edge trigger, the trigger source is control de-bounce.<br>
If the interrupt is by level trigger, the input source is sampled by one clock and the generate the interrupt.<br>
0 = Edge trigger interrupt.<br>
1 = Level trigger interrupt.<br>
If set pin as the level trigger interrupt, then only one level can be set on the registers GPIOX_IER.<br>
If set both the level to trigger interrupt, the setting is ignored and no interrupt will occur.<br>
The de-bounce function is valid for edge triggered interrupt.<br>
If the interrupt mode is level triggered, the de-bounce enable bit is ignored.<br>
Note: For GPIOF_IMD, bits [15:6] are reserved.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IMD;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IER</font><br><p> <font size="2">
Offset: 0x1C  GPIO Port Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>FIER0</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[1]</td><td>FIER1</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[2]</td><td>FIER2</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[3]</td><td>FIER3</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[4]</td><td>FIER4</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[5]</td><td>FIER5</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[6]</td><td>FIER6</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[7]</td><td>FIER7</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[8]</td><td>FIER8</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[9]</td><td>FIER9</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[10]</td><td>FIER10</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[11]</td><td>FIER11</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[12]</td><td>FIER12</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[13]</td><td>FIER13</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[14]</td><td>FIER14</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[15]</td><td>FIER15</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low</b><br>
FIER[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the FIER[n] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[n] state at level "low" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[n] state change from "high-to-low" will generate the interrupt.<br>
1 = PIN[n] state low-level or high-to-low change interrupt Enabled.<br>
0 = PIN[n] state low-level or high-to-low change interrupt Disabled.<br>
Note: For GPIOF_IER, bits [15:6] are reserved.<br>
</div></td></tr><tr><td>
[16]</td><td>RIER0</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[17]</td><td>RIER1</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[18]</td><td>RIER2</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[19]</td><td>RIER3</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[20]</td><td>RIER4</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[21]</td><td>RIER5</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[22]</td><td>RIER6</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[23]</td><td>RIER7</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[24]</td><td>RIER8</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[25]</td><td>RIER9</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[26]</td><td>RIER10</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[27]</td><td>RIER11</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[28]</td><td>RIER12</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[29]</td><td>RIER13</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[30]</td><td>RIER14</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr><tr><td>
[31]</td><td>RIER15</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High</b><br>
RIER[x] used to enable the interrupt for each of the corresponding input GPIO_PIN[x].<br>
Set bit "1" also enable the pin wake-up function.<br>
When set the RIER[x] bit "1":<br>
If the interrupt is level mode trigger, the input PIN[x] state at level "high" will generate the interrupt.<br>
If the interrupt is edge mode trigger, the input PIN[x] state change from "low-to-high" will generate the interrupt.<br>
1 = PIN[x] level-high or low-to-high interrupt Enabled.<br>
0 = PIN[x] level-high or low-to-high interrupt Disabled.<br>
Note: For GPIOF_IE, bits [31:22] are reserved.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IER;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISRC</font><br><p> <font size="2">
Offset: 0x20  GPIO Port Interrupt Trigger Source Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>ISRC</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Interrupt Trigger Source Indicator</b><br>
Read :<br>
1 = Port x[n] generate an interrupt.<br>
0 = No interrupt at Port x[n].<br>
Write:<br>
1 = Clear the correspond pending interrupt.<br>
0 = No action.<br>
Note: For GPIOF_ISRC, bits [15:6] are reserved.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISRC;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PUEN</font><br><p> <font size="2">
Offset: 0x24  GPIO Port</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
 :---- |<br>
<tr><td>
[15:0]</td><td>PUEN</td><td><div style="word-wrap: break-word;"><b>GPIO Port [X] Pin [N] Pull-Up Enable Register</b><br>
Read :<br>
1 = GPIO port [A/B/C/D/E/F] bit [n] pull-up resistor Enabled.<br>
0 = GPIO port [A/B/C/D/E/F] bit [n] pull-up resistor Disabled.<br>
Note: For GPIOF_PUEN, bits [15:6] are reserved.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PUEN;

} GPIO_T;


typedef struct {
/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DBNCECON</font><br><p> <font size="2">
Offset: 0x180  De-bounce Cycle Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>PUEN</td><td><div style="word-wrap: break-word;"><b>De-Bounce Sampling Cycle Selection</b><br>
0000 = Sample interrupt input once per 1 clock.<br>
0001 = Sample interrupt input once per 2 clock.<br>
0010 = Sample interrupt input once per 4 clock.<br>
0011 = Sample interrupt input once per 8 clock.<br>
0100 = Sample interrupt input once per 16 clock.<br>
0101 = Sample interrupt input once per 32 clock.<br>
0110 = Sample interrupt input once per 64 clock.<br>
0111 = Sample interrupt input once per 128 clock.<br>
1000 = Sample interrupt input once per 256 clock.<br>
1001 = Sample interrupt input once per 512 clock.<br>
1010 = Sample interrupt input once per 1024 clock.<br>
1011 = Sample interrupt input once per 2048 clock.<br>
1100 = Sample interrupt input once per 4096 clock.<br>
1101 = Sample interrupt input once per 8192 clock.<br>
1110 = Sample interrupt input once per 16384 clock.<br>
1111 = Sample interrupt input once per 32768 clock.<br>
</div></td></tr><tr><td>
[4]</td><td>DBCLKSRC</td><td><div style="word-wrap: break-word;"><b>De-Bounce Counter Clock Source Selection</b><br>
0 = De-bounce counter Clock Source is the HCLK.<br>
1 = De-bounce counter Clock Source is the internal 10 kHz clock.<br>
</div></td></tr><tr><td>
[5]</td><td>DBCLK_ON</td><td><div style="word-wrap: break-word;"><b>De-Bounce Clock Enable</b><br>
This bit controls if the de-bounce clock is enabled.<br>
However, if GPI/O pin's interrupt is enabled, the de-bounce clock will be enabled automatically no matter what the DBCLK_ON value is.<br>
If CPU is in sleep mode, this bit didn't take effect.<br>
And only the GPI/O pin with interrupt enable could get de-bounce clock.<br>
0 = De-bounce clock Disabled.<br>
1 = De-bounce clock Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DBNCECON;
} GP_DB_T;

/**
    @addtogroup GPIO_CONST GPIO Bit Field Definition
    Constant Definitions for GPIO Controller
@{ */

#define GP_PMD_PMD0_Pos                  (0)                                               /*!< GPIO_T::PMD: PMD0 Position                  */
#define GP_PMD_PMD0_Msk                  (0x3ul << GP_PMD_PMD0_Pos)                        /*!< GPIO_T::PMD: PMD0 Mask                      */

#define GP_PMD_PMD1_Pos                  (2)                                               /*!< GPIO_T::PMD: PMD1 Position                  */
#define GP_PMD_PMD1_Msk                  (0x3ul << GP_PMD_PMD1_Pos)                        /*!< GPIO_T::PMD: PMD1 Mask                      */

#define GP_PMD_PMD2_Pos                  (4)                                               /*!< GPIO_T::PMD: PMD2 Position                  */
#define GP_PMD_PMD2_Msk                  (0x3ul << GP_PMD_PMD2_Pos)                        /*!< GPIO_T::PMD: PMD2 Mask                      */

#define GP_PMD_PMD3_Pos                  (6)                                               /*!< GPIO_T::PMD: PMD3 Position                  */
#define GP_PMD_PMD3_Msk                  (0x3ul << GP_PMD_PMD3_Pos)                        /*!< GPIO_T::PMD: PMD3 Mask                      */

#define GP_PMD_PMD4_Pos                  (8)                                               /*!< GPIO_T::PMD: PMD4 Position                  */
#define GP_PMD_PMD4_Msk                  (0x3ul << GP_PMD_PMD4_Pos)                        /*!< GPIO_T::PMD: PMD4 Mask                      */

#define GP_PMD_PMD5_Pos                  (10)                                              /*!< GPIO_T::PMD: PMD5 Position                  */
#define GP_PMD_PMD5_Msk                  (0x3ul << GP_PMD_PMD5_Pos)                        /*!< GPIO_T::PMD: PMD5 Mask                      */

#define GP_PMD_PMD6_Pos                  (12)                                              /*!< GPIO_T::PMD: PMD6 Position                  */
#define GP_PMD_PMD6_Msk                  (0x3ul << GP_PMD_PMD6_Pos)                        /*!< GPIO_T::PMD: PMD6 Mask                      */

#define GP_PMD_PMD7_Pos                  (14)                                              /*!< GPIO_T::PMD: PMD7 Position                  */
#define GP_PMD_PMD7_Msk                  (0x3ul << GP_PMD_PMD7_Pos)                        /*!< GPIO_T::PMD: PMD7 Mask                      */

#define GP_PMD_PMD8_Pos                  (16)                                              /*!< GPIO_T::PMD: PMD8 Position                  */
#define GP_PMD_PMD8_Msk                  (0x3ul << GP_PMD_PMD8_Pos)                        /*!< GPIO_T::PMD: PMD8 Mask                      */

#define GP_PMD_PMD9_Pos                  (18)                                              /*!< GPIO_T::PMD: PMD9 Position                  */
#define GP_PMD_PMD9_Msk                  (0x3ul << GP_PMD_PMD9_Pos)                        /*!< GPIO_T::PMD: PMD9 Mask                      */

#define GP_PMD_PMD10_Pos                 (20)                                              /*!< GPIO_T::PMD: PMD10 Position                 */
#define GP_PMD_PMD10_Msk                 (0x3ul << GP_PMD_PMD10_Pos)                       /*!< GPIO_T::PMD: PMD10 Mask                     */

#define GP_PMD_PMD11_Pos                 (22)                                              /*!< GPIO_T::PMD: PMD11 Position                 */
#define GP_PMD_PMD11_Msk                 (0x3ul << GP_PMD_PMD11_Pos)                       /*!< GPIO_T::PMD: PMD11 Mask                     */

#define GP_PMD_PMD12_Pos                 (24)                                              /*!< GPIO_T::PMD: PMD12 Position                 */
#define GP_PMD_PMD12_Msk                 (0x3ul << GP_PMD_PMD12_Pos)                       /*!< GPIO_T::PMD: PMD12 Mask                     */

#define GP_PMD_PMD13_Pos                 (26)                                              /*!< GPIO_T::PMD: PMD13 Position                 */
#define GP_PMD_PMD13_Msk                 (0x3ul << GP_PMD_PMD13_Pos)                       /*!< GPIO_T::PMD: PMD13 Mask                     */

#define GP_PMD_PMD14_Pos                 (28)                                              /*!< GPIO_T::PMD: PMD14 Position                 */
#define GP_PMD_PMD14_Msk                 (0x3ul << GP_PMD_PMD14_Pos)                       /*!< GPIO_T::PMD: PMD14 Mask                     */

#define GP_PMD_PMD15_Pos                 (30)                                              /*!< GPIO_T::PMD: PMD15 Position                 */
#define GP_PMD_PMD15_Msk                 (0x3ul << GP_PMD_PMD15_Pos)                       /*!< GPIO_T::PMD: PMD15 Mask                     */

#define GP_OFFD_OFFD_Pos                 (16)                                              /*!< GPIO_T::OFFD: OFFD Position                 */
#define GP_OFFD_OFFD_Msk                 (0xfffful << GP_OFFD_OFFD_Pos)                    /*!< GPIO_T::OFFD: OFFD Mask                     */

#define GP_DOUT_DOUT_Pos                 (0)                                               /*!< GPIO_T::DOUT: DOUT Position                 */
#define GP_DOUT_DOUT_Msk                 (0xfffful << GP_DOUT_DOUT_Pos)                    /*!< GPIO_T::DOUT: DOUT Mask                     */

#define GP_DMASK_DMASK_Pos               (0)                                               /*!< GPIO_T::DMASK: DMASK Position               */
#define GP_DMASK_DMASK_Msk               (0xfffful << GP_DMASK_DMASK_Pos)                  /*!< GPIO_T::DMASK: DMASK Mask                   */

#define GP_PIN_PIN_Pos                   (0)                                               /*!< GPIO_T::PIN: PIN Position                   */
#define GP_PIN_PIN_Msk                   (0xfffful << GP_PIN_PIN_Pos)                      /*!< GPIO_T::PIN: PIN Mask                       */

#define GP_DBEN_DBEN_Pos                 (0)                                               /*!< GPIO_T::DBEN: DBEN Position                 */
#define GP_DBEN_DBEN_Msk                 (0xfffful << GP_DBEN_DBEN_Pos)                    /*!< GPIO_T::DBEN: DBEN Mask                     */

#define GP_IMD_IMD_Pos                   (0)                                               /*!< GPIO_T::IMD: IMD Position                   */
#define GP_IMD_IMD_Msk                   (0xfffful << GP_IMD_IMD_Pos)                      /*!< GPIO_T::IMD: IMD Mask                       */

#define GP_IER_FIER0_Pos                 (0)                                               /*!< GPIO_T::IER: FIER0 Position                 */
#define GP_IER_FIER0_Msk                 (0x1ul << GP_IER_FIER0_Pos)                       /*!< GPIO_T::IER: FIER0 Mask                     */

#define GP_IER_FIER1_Pos                 (1)                                               /*!< GPIO_T::IER: FIER1 Position                 */
#define GP_IER_FIER1_Msk                 (0x1ul << GP_IER_FIER1_Pos)                       /*!< GPIO_T::IER: FIER1 Mask                     */

#define GP_IER_FIER2_Pos                 (2)                                               /*!< GPIO_T::IER: FIER2 Position                 */
#define GP_IER_FIER2_Msk                 (0x1ul << GP_IER_FIER2_Pos)                       /*!< GPIO_T::IER: FIER2 Mask                     */

#define GP_IER_FIER3_Pos                 (3)                                               /*!< GPIO_T::IER: FIER3 Position                 */
#define GP_IER_FIER3_Msk                 (0x1ul << GP_IER_FIER3_Pos)                       /*!< GPIO_T::IER: FIER3 Mask                     */

#define GP_IER_FIER4_Pos                 (4)                                               /*!< GPIO_T::IER: FIER4 Position                 */
#define GP_IER_FIER4_Msk                 (0x1ul << GP_IER_FIER4_Pos)                       /*!< GPIO_T::IER: FIER4 Mask                     */

#define GP_IER_FIER5_Pos                 (5)                                               /*!< GPIO_T::IER: FIER5 Position                 */
#define GP_IER_FIER5_Msk                 (0x1ul << GP_IER_FIER5_Pos)                       /*!< GPIO_T::IER: FIER5 Mask                     */

#define GP_IER_FIER6_Pos                 (6)                                               /*!< GPIO_T::IER: FIER6 Position                 */
#define GP_IER_FIER6_Msk                 (0x1ul << GP_IER_FIER6_Pos)                       /*!< GPIO_T::IER: FIER6 Mask                     */

#define GP_IER_FIER7_Pos                 (7)                                               /*!< GPIO_T::IER: FIER7 Position                 */
#define GP_IER_FIER7_Msk                 (0x1ul << GP_IER_FIER7_Pos)                       /*!< GPIO_T::IER: FIER7 Mask                     */

#define GP_IER_FIER8_Pos                 (8)                                               /*!< GPIO_T::IER: FIER8 Position                 */
#define GP_IER_FIER8_Msk                 (0x1ul << GP_IER_FIER8_Pos)                       /*!< GPIO_T::IER: FIER8 Mask                     */

#define GP_IER_FIER9_Pos                 (9)                                               /*!< GPIO_T::IER: FIER9 Position                 */
#define GP_IER_FIER9_Msk                 (0x1ul << GP_IER_FIER9_Pos)                       /*!< GPIO_T::IER: FIER9 Mask                     */

#define GP_IER_FIER10_Pos                (10)                                              /*!< GPIO_T::IER: FIER10 Position                */
#define GP_IER_FIER10_Msk                (0x1ul << GP_IER_FIER10_Pos)                      /*!< GPIO_T::IER: FIER10 Mask                    */

#define GP_IER_FIER11_Pos                (11)                                              /*!< GPIO_T::IER: FIER11 Position                */
#define GP_IER_FIER11_Msk                (0x1ul << GP_IER_FIER11_Pos)                      /*!< GPIO_T::IER: FIER11 Mask                    */

#define GP_IER_FIER12_Pos                (12)                                              /*!< GPIO_T::IER: FIER12 Position                */
#define GP_IER_FIER12_Msk                (0x1ul << GP_IER_FIER12_Pos)                      /*!< GPIO_T::IER: FIER12 Mask                    */

#define GP_IER_FIER13_Pos                (13)                                              /*!< GPIO_T::IER: FIER13 Position                */
#define GP_IER_FIER13_Msk                (0x1ul << GP_IER_FIER13_Pos)                      /*!< GPIO_T::IER: FIER13 Mask                    */

#define GP_IER_FIER14_Pos                (14)                                              /*!< GPIO_T::IER: FIER14 Position                */
#define GP_IER_FIER14_Msk                (0x1ul << GP_IER_FIER14_Pos)                      /*!< GPIO_T::IER: FIER14 Mask                    */

#define GP_IER_FIER15_Pos                (15)                                              /*!< GPIO_T::IER: FIER15 Position                */
#define GP_IER_FIER15_Msk                (0x1ul << GP_IER_FIER15_Pos)                      /*!< GPIO_T::IER: FIER15 Mask                    */

#define GP_IER_RIER0_Pos                 (16)                                              /*!< GPIO_T::IER: RIER0 Position                 */
#define GP_IER_RIER0_Msk                 (0x1ul << GP_IER_RIER0_Pos)                       /*!< GPIO_T::IER: RIER0 Mask                     */

#define GP_IER_RIER1_Pos                 (17)                                              /*!< GPIO_T::IER: RIER1 Position                 */
#define GP_IER_RIER1_Msk                 (0x1ul << GP_IER_RIER1_Pos)                       /*!< GPIO_T::IER: RIER1 Mask                     */

#define GP_IER_RIER2_Pos                 (18)                                              /*!< GPIO_T::IER: RIER2 Position                 */
#define GP_IER_RIER2_Msk                 (0x1ul << GP_IER_RIER2_Pos)                       /*!< GPIO_T::IER: RIER2 Mask                     */

#define GP_IER_RIER3_Pos                 (19)                                              /*!< GPIO_T::IER: RIER3 Position                 */
#define GP_IER_RIER3_Msk                 (0x1ul << GP_IER_RIER3_Pos)                       /*!< GPIO_T::IER: RIER3 Mask                     */

#define GP_IER_RIER4_Pos                 (20)                                              /*!< GPIO_T::IER: RIER4 Position                 */
#define GP_IER_RIER4_Msk                 (0x1ul << GP_IER_RIER4_Pos)                       /*!< GPIO_T::IER: RIER4 Mask                     */

#define GP_IER_RIER5_Pos                 (21)                                              /*!< GPIO_T::IER: RIER5 Position                 */
#define GP_IER_RIER5_Msk                 (0x1ul << GP_IER_RIER5_Pos)                       /*!< GPIO_T::IER: RIER5 Mask                     */

#define GP_IER_RIER6_Pos                 (22)                                              /*!< GPIO_T::IER: RIER6 Position                 */
#define GP_IER_RIER6_Msk                 (0x1ul << GP_IER_RIER6_Pos)                       /*!< GPIO_T::IER: RIER6 Mask                     */

#define GP_IER_RIER7_Pos                 (23)                                              /*!< GPIO_T::IER: RIER7 Position                 */
#define GP_IER_RIER7_Msk                 (0x1ul << GP_IER_RIER7_Pos)                       /*!< GPIO_T::IER: RIER7 Mask                     */

#define GP_IER_RIER8_Pos                 (24)                                              /*!< GPIO_T::IER: RIER8 Position                 */
#define GP_IER_RIER8_Msk                 (0x1ul << GP_IER_RIER8_Pos)                       /*!< GPIO_T::IER: RIER8 Mask                     */

#define GP_IER_RIER9_Pos                 (25)                                              /*!< GPIO_T::IER: RIER9 Position                 */
#define GP_IER_RIER9_Msk                 (0x1ul << GP_IER_RIER9_Pos)                       /*!< GPIO_T::IER: RIER9 Mask                     */

#define GP_IER_RIER10_Pos                (26)                                              /*!< GPIO_T::IER: RIER10 Position                */
#define GP_IER_RIER10_Msk                (0x1ul << GP_IER_RIER10_Pos)                      /*!< GPIO_T::IER: RIER10 Mask                    */

#define GP_IER_RIER11_Pos                (27)                                              /*!< GPIO_T::IER: RIER11 Position                */
#define GP_IER_RIER11_Msk                (0x1ul << GP_IER_RIER11_Pos)                      /*!< GPIO_T::IER: RIER11 Mask                    */

#define GP_IER_RIER12_Pos                (28)                                              /*!< GPIO_T::IER: RIER12 Position                */
#define GP_IER_RIER12_Msk                (0x1ul << GP_IER_RIER12_Pos)                      /*!< GPIO_T::IER: RIER12 Mask                    */

#define GP_IER_RIER13_Pos                (29)                                              /*!< GPIO_T::IER: RIER13 Position                */
#define GP_IER_RIER13_Msk                (0x1ul << GP_IER_RIER13_Pos)                      /*!< GPIO_T::IER: RIER13 Mask                    */

#define GP_IER_RIER14_Pos                (30)                                              /*!< GPIO_T::IER: RIER14 Position                */
#define GP_IER_RIER14_Msk                (0x1ul << GP_IER_RIER14_Pos)                      /*!< GPIO_T::IER: RIER14 Mask                    */

#define GP_IER_RIER15_Pos                (31)                                              /*!< GPIO_T::IER: RIER15 Position                */
#define GP_IER_RIER15_Msk                (0x1ul << GP_IER_RIER15_Pos)                      /*!< GPIO_T::IER: RIER15 Mask                    */

#define GP_ISRC_ISRC_Pos                 (0)                                               /*!< GPIO_T::ISRC: ISRC Position                 */
#define GP_ISRC_ISRC_Msk                 (0xfffful << GP_ISRC_ISRC_Pos)                    /*!< GPIO_T::ISRC: ISRC Mask                     */

#define GP_PUEN_PUEN_Pos                 (0)                                               /*!< GPIO_T::PUEN: PUEN Position                 */
#define GP_PUEN_PUEN_Msk                 (0xfffful << GP_PUEN_PUEN_Pos)                    /*!< GPIO_T::PUEN: PUEN Mask                     */
/**@}*/ /* GPIO_CONST */

/**
    @addtogroup GP_DB_CONST GP_DB Bit Field Definition
    Constant Definitions for GP_DB Controller
@{ */
#define GP_DBNCECON_DBCLKSEL_Pos         (0)                                               /*!< GP_DB_T::DBNCECON: DBCLKSEL Position             */
#define GP_DBNCECON_DBCLKSEL_Msk         (0xful << GP_DBNCECON_DBCLKSEL_Pos)               /*!< GP_DB_T::DBNCECON: DBCLKSEL Mask                 */

#define GP_DBNCECON_DBCLKSRC_Pos         (4)                                               /*!< GP_DB_T::DBNCECON: DBCLKSRC Position         */
#define GP_DBNCECON_DBCLKSRC_Msk         (0x1ul << GP_DBNCECON_DBCLKSRC_Pos)               /*!< GP_DB_T::DBNCECON: DBCLKSRC Mask             */

#define GP_DBNCECON_DBCLK_ON_Pos         (5)                                               /*!< GP_DB_T::DBNCECON: DBCLK_ON Position         */
#define GP_DBNCECON_DBCLK_ON_Msk         (0x1ul << GP_DBNCECON_DBCLK_ON_Pos)               /*!< GP_DB_T::DBNCECON: DBCLK_ON Mask             */


/**@}*/ /* GP_DB_CONST */
/**@}*/ /* end of GP register group */


/*---------------------- Inter-IC Bus Controller -------------------------*/
/**
    @addtogroup I2C Inter-IC Bus Controller(I2C)
    Memory Mapped Structure for I2C Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CON</font><br><p> <font size="2">
Offset: 0x00  I2C Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>IPEN</td><td><div style="word-wrap: break-word;"><b>I2C Function Enable</b><br>
When this bit is set to 1, the I2C serial function is enabled.<br>
0 = I2C function Disabled.<br>
1 = I2C function Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>ACK</td><td><div style="word-wrap: break-word;"><b>Assert Acknowledge Control Bit</b><br>
0 =: When this bit is set to 0 prior to address or data received, a Not acknowledged (high level to SDA) will be returned during the acknowledge clock pulse.<br>
1 = When this bit is set to 1 prior to address or data received, an acknowledged will be returned during the acknowledge clock pulse on the SCL line when.<br>
a. A slave is acknowledging the address sent from master<br>
b. The receiver devices are acknowledging the data sent by transmitter.<br>
</div></td></tr><tr><td>
[2]</td><td>STOP</td><td><div style="word-wrap: break-word;"><b>I2C STOP Control Bit</b><br>
In Master mode, set this bit to 1 to transmit a STOP condition to bus then the controller will check the bus condition if a STOP condition is detected and this bit will be cleared by hardware automatically.<br>
In Slave mode, set this bit to 1 to reset the controller to the defined "not addressed" Slave mode.<br>
This means it is NO LONGER in the slave receiver mode to receive data from the master transmit device.<br>
0 = Will be cleared by hardware automatically if a STOP condition is detected.<br>
1 = Sends a STOP condition to bus in Master mode or reset the controller to "not addressed" in Slave mode.<br>
</div></td></tr><tr><td>
[3]</td><td>START</td><td><div style="word-wrap: break-word;"><b>I2C START Command</b><br>
Setting this bit to 1 to enter Master mode, the device sends a START or repeat START condition to bus when the bus is free and it will be cleared to 0 after the START command is active and the STATUS has been updated.<br>
0 = After START or repeat START is active.<br>
1 = Sends a START or repeat START condition to bus.<br>
</div></td></tr><tr><td>
[4]</td><td>I2C_STS</td><td><div style="word-wrap: break-word;"><b>I2C Status</b><br>
When a new state is present in the I2CSTATUS register, this bit will be set automatically, and if the INTEN bit is set, the I2C interrupt is requested.<br>
It must be cleared by software by writing one to this bit and the I2C protocol function will go ahead until the STOP is active or the IPEN is disabled.<br>
0 = I2C's Status disabled and the I2C protocol function will go ahead.<br>
1 = I2C's Status active.<br>
</div></td></tr><tr><td>
[7]</td><td>INTEN</td><td><div style="word-wrap: break-word;"><b>Interrupt Enable</b><br>
0 = I2C interrupt Disabled.<br>
1 = I2C interrupt Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CON;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">INTSTS</font><br><p> <font size="2">
Offset: 0x04  I2C Interrupt Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>INTSTS</td><td><div style="word-wrap: break-word;"><b>I2C STATUS's Interrupt Status</b><br>
When a new state is present in the I2CSTATUS register, this bit will be set automatically, and if INTEN bit is set, the I2C interrupt is requested.<br>
Software can write 1 to cleat this bit.<br>
</div></td></tr><tr><td>
[1]</td><td>TIF</td><td><div style="word-wrap: break-word;"><b>Time-Out Status</b><br>
0 = No Time-out flag. Software can cleat this flag.<br>
1 = Time-Out flag active and it is set by hardware. It can interrupt CPU when INTEN bit is set.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t INTSTS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">STATUS</font><br><p> <font size="2">
Offset: 0x08  I2C Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>STATUS</td><td><div style="word-wrap: break-word;"><b>I2C Status Register</b><br>
This is a read only register.<br>
The three least significant bits are always 0.<br>
The five most significant bits contain the status code.<br>
When each of these states is entered, a status interrupt and I2C_STS are requested (I2C_STS = 1 and STAINTSTS = 1).<br>
A valid status code is present in STATUS one machine cycle after I2C_STS is set by hardware and is still present one machine cycle after I2C_STS has been reset by software.<br>
In addition, states 00H stands for a 'Bus Error'.<br>
A 'Bus Error' occurs when a START or STOP condition is present at an illegal position in the formation frame.<br>
Example of illegal position: a data byte or an acknowledge bit is present during the serial transfer of an address byte.<br>
To recover I2C from bus error, STOP should be set and I2C_STS should be cleared to enter not addressed Slave mode.<br>
Then clear STOP to release the bus and to wait new communication.<br>
I2C bus can not recognize stop condition during this action when bus error occurs.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t STATUS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DIV</font><br><p> <font size="2">
Offset: 0x0C  I2C clock divided Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>CLK_DIV</td><td><div style="word-wrap: break-word;"><b>I2C Clock Divider Control Register</b><br>
The I2C clock rate bits: Data Baud Rate of I2C = PCLK /( 4 x ( CLK_DIV + 1)).<br>
Note: the minimum value of CLK_DIV is 4.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DIV;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TOUT</font><br><p> <font size="2">
Offset: 0x10  I2C Time-out control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TOUTEN</td><td><div style="word-wrap: break-word;"><b>Time-Out Counter Enable/Disable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
When set this bit to enable, the 14 bits time-out counter will start counting when STAINTSTS is cleared.<br>
Setting flag STAINTSTS to high or the falling edge of I2C clock or stop signal will reset counter and re-start up counting after STAINTSTS is cleared.<br>
</div></td></tr><tr><td>
[1]</td><td>DIV4</td><td><div style="word-wrap: break-word;"><b>Time-Out Counter Input Clock Divider By 4</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
When this bit is set enabled, the Time-Out period is prolonging 4 times.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TOUT;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DATA</font><br><p> <font size="2">
Offset: 0x14  I2C DATA Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>DATA</td><td><div style="word-wrap: break-word;"><b>I2C Data Register</b><br>
The DATA contains a byte of serial data to be transmitted or a byte which has just been received.<br>
The user can read from or write to this 8-bit I2CDATA register directly while it is not in the process of shifting a byte.<br>
This occurs when the serial interrupt flag is set.<br>
Data in DATA remains stable as long as I2C_STS bit is set.<br>
While data is being shifted out, data on the bus is simultaneously being shifted in; The DATA always contains the last data byte present on the bus.<br>
Thus, in the event of arbitration lost, the transition from master transmitter to slave receiver is made with the correct data in DATA.<br>
DATA and the acknowledge bit form a 9-bit shift register, the acknowledge bit is controlled by the device hardware and cannot be accessed by the user.<br>
Serial data is shifted through the acknowledge bit into DATA on the rising edges of serial clock pulses on the SCL line.<br>
When a byte has been shifted into DATA, the serial data is available in DATA, and the acknowledge bit (ACK or NACK) is returned by the control logic during the ninth clock pulse.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DATA;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SADDR0</font><br><p> <font size="2">
Offset: 0x18  I2C Slave address Register0</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>GCALL</td><td><div style="word-wrap: break-word;"><b>General Call Function</b><br>
The I2C controller supports the "General Call" function.<br>
If the GCALL bit is set, the controller will respond to General Call address (00H).<br>
When GCALL bit is set, the controller is in Slave mode, it can receive the general call address by 00H after Master send general call address to the I2C bus, then it will follow status of GCALL mode.<br>
If it is in Master mode, the ACK bit must be cleared when it will send general call address of 00H to I2C bus.<br>
0 = General Call Function Disabled.<br>
1 = General Call Function Enabled.<br>
</div></td></tr><tr><td>
[7:1]</td><td>SADDR</td><td><div style="word-wrap: break-word;"><b>I2C Salve Address Register</b><br>
The content of this register is irrelevant when the device is in Master mode.<br>
In the Slave mode, the seven most significant bits must be loaded with the device's own address.<br>
The device will react if either of the address is matched.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SADDR0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SADDR1</font><br><p> <font size="2">
Offset: 0x1C  I2C Slave address Register1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>GCALL</td><td><div style="word-wrap: break-word;"><b>General Call Function</b><br>
The I2C controller supports the "General Call" function.<br>
If the GCALL bit is set, the controller will respond to General Call address (00H).<br>
When GCALL bit is set, the controller is in Slave mode, it can receive the general call address by 00H after Master send general call address to the I2C bus, then it will follow status of GCALL mode.<br>
If it is in Master mode, the ACK bit must be cleared when it will send general call address of 00H to I2C bus.<br>
0 = General Call Function Disabled.<br>
1 = General Call Function Enabled.<br>
</div></td></tr><tr><td>
[7:1]</td><td>SADDR</td><td><div style="word-wrap: break-word;"><b>I2C Salve Address Register</b><br>
The content of this register is irrelevant when the device is in Master mode.<br>
In the Slave mode, the seven most significant bits must be loaded with the device's own address.<br>
The device will react if either of the address is matched.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SADDR1;
    uint32_t RESERVE0[2];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SAMASK0</font><br><p> <font size="2">
Offset: 0x28  I2C Slave address Mask Register0</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:1]</td><td>SAMASK</td><td><div style="word-wrap: break-word;"><b>I2C Slave Address Mask Register</b><br>
0 = Mask disable (the received corresponding register bit should be exact the same as address register.).<br>
1 = Mask enable (the received corresponding address bit is don't care.).<br>
I2C bus controllers support multiple address recognition with two address mask registers.<br>
When the bit in the address mask register is set to b'1, it means the received corresponding address bit is don't-care.<br>
If the bit is set to b'0, that means the received corresponding register bit should be exact the same as address register.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SAMASK0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SAMASK1</font><br><p> <font size="2">
Offset: 0x2C  I2C Slave address Mask Register1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:1]</td><td>SAMASK</td><td><div style="word-wrap: break-word;"><b>I2C Slave Address Mask Register</b><br>
0 = Mask disable (the received corresponding register bit should be exact the same as address register.).<br>
1 = Mask enable (the received corresponding address bit is don't care.).<br>
I2C bus controllers support multiple address recognition with two address mask registers.<br>
When the bit in the address mask register is set to b'1, it means the received corresponding address bit is don't-care.<br>
If the bit is set to b'0, that means the received corresponding register bit should be exact the same as address register.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SAMASK1;
    uint32_t RESERVE1[4];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">WKUPCON</font><br><p> <font size="2">
Offset: 0x40  I2C Wake-up Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>WKUPEN</td><td><div style="word-wrap: break-word;"><b>I2C Wake-Up Function Enable</b><br>
0 = I2C wake-up function Disabled.<br>
1 = I2C wake-up function Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t WKUPCON;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">WKUPSTS</font><br><p> <font size="2">
Offset: 0x44  I2C Wake-up Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>WKUPIF</td><td><div style="word-wrap: break-word;"><b>Wake-Up Interrupt Flag</b><br>
0 = Wake-up flag inactive.<br>
1 = Wake-up flag active.<br>
Software can write 1 to clear this flag<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO  uint32_t WKUPSTS;

} I2C_T;

/**
    @addtogroup I2C_CONST I2C Bit Field Definition
    Constant Definitions for I2C Controller
@{ */

#define I2C_CON_IPEN_Pos              (0)                                               /*!< I2C_T::CON: IPEN Position              */
#define I2C_CON_IPEN_Msk              (0x1ul << I2C_CON_IPEN_Pos)                       /*!< I2C_T::CON: IPEN Mask                  */

#define I2C_CON_ACK_Pos               (1)                                               /*!< I2C_T::CON: ACK Position               */
#define I2C_CON_ACK_Msk               (0x1ul << I2C_CON_ACK_Pos)                        /*!< I2C_T::CON: ACK Mask                   */

#define I2C_CON_STOP_Pos              (2)                                               /*!< I2C_T::CON: STOP Position              */
#define I2C_CON_STOP_Msk              (0x1ul << I2C_CON_STOP_Pos)                       /*!< I2C_T::CON: STOP Mask                  */

#define I2C_CON_START_Pos             (3)                                               /*!< I2C_T::CON: START Position             */
#define I2C_CON_START_Msk             (0x1ul << I2C_CON_START_Pos)                      /*!< I2C_T::CON: START Mask                 */

#define I2C_CON_I2C_STS_Pos           (4)                                               /*!< I2C_T::CON: I2C_STS Position           */
#define I2C_CON_I2C_STS_Msk           (0x1ul << I2C_CON_I2C_STS_Pos)                    /*!< I2C_T::CON: I2C_STS Mask               */

#define I2C_CON_INTEN_Pos             (7)                                               /*!< I2C_T::CON: INTEN Position             */
#define I2C_CON_INTEN_Msk             (0x1ul << I2C_CON_INTEN_Pos)                      /*!< I2C_T::CON: INTEN Mask                 */

#define I2C_INTSTS_INTSTS_Pos         (0)                                               /*!< I2C_T::INTSTS: INTSTS Position         */
#define I2C_INTSTS_INTSTS_Msk         (0x1ul << I2C_INTSTS_INTSTS_Pos)                  /*!< I2C_T::INTSTS: INTSTS Mask             */

#define I2C_INTSTS_TIF_Pos            (1)                                               /*!< I2C_T::INTSTS: TIF Position            */
#define I2C_INTSTS_TIF_Msk            (0x1ul << I2C_INTSTS_TIF_Pos)                     /*!< I2C_T::INTSTS: TIF Mask                */

#define I2C_STATUS_STATUS_Pos         (0)                                               /*!< I2C_T::STATUS: STATUS Position         */
#define I2C_STATUS_STATUS_Msk         (0xfful << I2C_STATUS_STATUS_Pos)                 /*!< I2C_T::STATUS: STATUS Mask             */

#define I2C_DIV_CLK_DIV_Pos           (0)                                               /*!< I2C_T::DIV: CLK_DIV Position           */
#define I2C_DIV_CLK_DIV_Msk           (0xfful << I2C_DIV_CLK_DIV_Pos)                   /*!< I2C_T::DIV: CLK_DIV Mask               */

#define I2C_TOUT_TOUTEN_Pos           (0)                                               /*!< I2C_T::TOUT: TOUTEN Position           */
#define I2C_TOUT_TOUTEN_Msk           (0x1ul << I2C_TOUT_TOUTEN_Pos)                    /*!< I2C_T::TOUT: TOUTEN Mask               */

#define I2C_TOUT_DIV4_Pos             (1)                                               /*!< I2C_T::TOUT: DIV4 Position             */
#define I2C_TOUT_DIV4_Msk             (0x1ul << I2C_TOUT_DIV4_Pos)                      /*!< I2C_T::TOUT: DIV4 Mask                 */

#define I2C_DATA_DATA_Pos             (0)                                               /*!< I2C_T::DATA: DATA Position             */
#define I2C_DATA_DATA_Msk             (0xfful << I2C_DATA_DATA_Pos)                     /*!< I2C_T::DATA: DATA Mask                 */

#define I2C_SADDR0_GCALL_Pos          (0)                                               /*!< I2C_T::SADDR0: GCALL Position          */
#define I2C_SADDR0_GCALL_Msk          (0x1ul << I2C_SADDR0_GCALL_Pos)                   /*!< I2C_T::SADDR0: GCALL Mask              */

#define I2C_SADDR0_SADDR_Pos          (1)                                               /*!< I2C_T::SADDR0: SADDR Position          */
#define I2C_SADDR0_SADDR_Msk          (0x7ful << I2C_SADDR0_SADDR_Pos)                  /*!< I2C_T::SADDR0: SADDR Mask              */

#define I2C_SADDR1_GCALL_Pos          (0)                                               /*!< I2C_T::SADDR1: GCALL Position          */
#define I2C_SADDR1_GCALL_Msk          (0x1ul << I2C_SADDR1_GCALL_Pos)                   /*!< I2C_T::SADDR1: GCALL Mask              */

#define I2C_SADDR1_SADDR_Pos          (1)                                               /*!< I2C_T::SADDR1: SADDR Position          */
#define I2C_SADDR1_SADDR_Msk          (0x7ful << I2C_SADDR1_SADDR_Pos)                  /*!< I2C_T::SADDR1: SADDR Mask              */

#define I2C_SAMASK0_SAMASK_Pos        (1)                                               /*!< I2C_T::SAMASK0: SAMASK Position        */
#define I2C_SAMASK0_SAMASK_Msk        (0x7ful << I2C_SAMASK0_SAMASK_Pos)                /*!< I2C_T::SAMASK0: SAMASK Mask            */

#define I2C_SAMASK1_SAMASK_Pos        (1)                                               /*!< I2C_T::SAMASK1: SAMASK Position        */
#define I2C_SAMASK1_SAMASK_Msk        (0x7ful << I2C_SAMASK1_SAMASK_Pos)                /*!< I2C_T::SAMASK1: SAMASK Mask            */

#define I2C_WKUPCON_WKUPEN_Pos        (0)                                               /*!< I2C_T::WKUPCON: WKUPEN Position        */
#define I2C_WKUPCON_WKUPEN_Msk        (0x1ul << I2C_WKUPCON_WKUPEN_Pos)                 /*!< I2C_T::WKUPCON: WKUPEN Mask            */

#define I2C_WKUPSTS_WKUPIF_Pos        (0)                                               /*!< I2C_T::WKUPSTS: WKUPIF Position        */
#define I2C_WKUPSTS_WKUPIF_Msk        (0x1ul << I2C_WKUPSTS_WKUPIF_Pos)                 /*!< I2C_T::WKUPSTS: WKUPIF Mask            */

/**@}*/ /* I2C_CONST */
/**@}*/ /* end of I2C register group */


/*---------------------- I2S Interface Controller -------------------------*/
/**
    @addtogroup I2S I2S Interface Controller(I2S)
    Memory Mapped Structure for I2S Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTRL</font><br><p> <font size="2">
Offset: 0x00  I2S Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>I2SEN</td><td><div style="word-wrap: break-word;"><b>I2S Controller Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>TXEN</td><td><div style="word-wrap: break-word;"><b>Transmit Enable</b><br>
0 = Data transmitting Disabled.<br>
1 = Data transmitting Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>RXEN</td><td><div style="word-wrap: break-word;"><b>Receive Enable</b><br>
0 = Data receiving Disabled.<br>
1 = Data receiving Enabled.<br>
</div></td></tr><tr><td>
[3]</td><td>MUTE</td><td><div style="word-wrap: break-word;"><b>Transmitting Mute Enable</b><br>
0 = Transmit data in buffer to channel.<br>
1 = Transmit '0' to channel.<br>
</div></td></tr><tr><td>
[5:4]</td><td>WORDWIDTH</td><td><div style="word-wrap: break-word;"><b>Word Width</b><br>
00 = Data is 8 bit.<br>
01 = Data is 16 bit.<br>
10 = Data is 24 bit.<br>
11 = Data is 32 bit.<br>
</div></td></tr><tr><td>
[6]</td><td>MONO</td><td><div style="word-wrap: break-word;"><b>Monaural Data</b><br>
0 = Data is stereo format.<br>
1 = Data is monaural format and gets the right channel data from I2S bus when this mode is enabled.<br>
</div></td></tr><tr><td>
[7]</td><td>FORMAT</td><td><div style="word-wrap: break-word;"><b>Data Format</b><br>
0 = I2S data format.<br>
1 = MSB justified data format.<br>
</div></td></tr><tr><td>
[8]</td><td>SLAVE</td><td><div style="word-wrap: break-word;"><b>Slave Mode</b><br>
I2S can operate as master or Slave mode.<br>
For Master mode, I2S_BCLK and I2S_LRCLK pins are output mode and also outputs I2S_BCLK and I2S_LRCLK signals to the audio CODEC.<br>
When act as Slave mode, I2S_BCLK and I2S_LRCLK pins are input mode and I2S_BCLK and I2S_LRCLK signals are received from the outer audio CODEC chip.<br>
0 = Master mode.<br>
1 = Slave mode.<br>
</div></td></tr><tr><td>
[11:9]</td><td>TXTH</td><td><div style="word-wrap: break-word;"><b>Transmit FIFO Threshold Level</b><br>
If remain data word (32 bits) in transmitting FIFO is the same or less than threshold level then TXTHF flag is set.<br>
000 = 1 word data in transmitting FIFO.<br>
001 = 2 word data in transmitting FIFO.<br>
010 = 3 word data in transmitting FIFO.<br>
011 = 4 word data in transmitting FIFO.<br>
100 = 5 word data in transmitting FIFO.<br>
101 = 6 word data in transmitting FIFO.<br>
110 = 7 word data in transmitting FIFO.<br>
111 = 8 word data in transmitting FIFO.<br>
</div></td></tr><tr><td>
[14:12]</td><td>RXTH</td><td><div style="word-wrap: break-word;"><b>Receiving FIFO Threshold Level</b><br>
When received data word(s) in buffer is equal to or higher than threshold level, the RXTHF flag is set.<br>
000 = 1 word data in receiving FIFO.<br>
001 = 2 word data in receiving FIFO.<br>
010 = 3 word data in receiving FIFO.<br>
011 = 4 word data in receiving FIFO.<br>
100 = 5 word data in receiving FIFO.<br>
101 = 6 word data in receiving FIFO.<br>
110 = 7 word data in receiving FIFO.<br>
111 = 8 word data in receiving FIFO.<br>
</div></td></tr><tr><td>
[15]</td><td>MCLKEN</td><td><div style="word-wrap: break-word;"><b>Master Clock Enable</b><br>
Enable master MCLK timing output to the external audio codec device.<br>
The output frequency is according to MCLK_DIV[2:0] in the I2S_CLKDIV register.<br>
0 = Master Clock Disabled.<br>
1 = Master Clock Enabled.<br>
</div></td></tr><tr><td>
[16]</td><td>RCHZCEN</td><td><div style="word-wrap: break-word;"><b>Right Channel Zero Cross Detect Enable</b><br>
If this bit is set to "1", when right channel data sign bit is changed or next shift data bits are all zero then RZCF flag in I2S_STATUS register is set to "1".<br>
It works on transmitting mode only.<br>
0 = Right channel zero cross detection Disabled.<br>
1 = Right channel zero cross detection Enabled.<br>
</div></td></tr><tr><td>
[17]</td><td>LCHZCEN</td><td><div style="word-wrap: break-word;"><b>Left Channel Zero Cross Detect Enable</b><br>
If this bit is set to "1", when left channel data sign bit is changed or next shift data bits are all zero then LZCF flag in I2S_STATUS register is set to "1".<br>
It works on transmitting mode only.<br>
0 = Left channel zero cross detection Disabled.<br>
1 = Left channel zero cross detection Enabled.<br>
</div></td></tr><tr><td>
[18]</td><td>CLR_TXFIFO</td><td><div style="word-wrap: break-word;"><b>Clear Transmit FIFO</b><br>
Write "1" to clear transmitting FIFO, internal pointer is reset to FIFO start point, TX_LEVEL[3:0] returns to zero and transmitting FIFO becomes empty but data in transmit FIFO is not changed.<br>
This bit is cleared by hardware automatically, read it to return zero.<br>
</div></td></tr><tr><td>
[19]</td><td>CLR_RXFIFO</td><td><div style="word-wrap: break-word;"><b>Clear Receiving FIFO</b><br>
Write "1" to clear receiving FIFO, internal pointer is reset to FIFO start point, and RX_LEVEL[3:0] returns to zero and receiving FIFO becomes empty.<br>
This bit is cleared by hardware automatically, and read it return zero.<br>
</div></td></tr><tr><td>
[20]</td><td>TXDMA</td><td><div style="word-wrap: break-word;"><b>Enable Transmit DMA</b><br>
When TX DMA is enabled, I2S requests PDMA to transfer data from memory to transmitting FIFO if FIFO is not full<br>
0 = TX DMA Disabled.<br>
1 = TX DMA Enabled.<br>
</div></td></tr><tr><td>
[21]</td><td>RXDMA</td><td><div style="word-wrap: break-word;"><b>Enable Receive DMA</b><br>
When RX DMA is enabled, I2S requests PDMA to transfer data from receiving FIFO to memory if FIFO is not empty.<br>
0 = RX DMA Disabled.<br>
1 = RX DMA Enabled.<br>
</div></td></tr><tr><td>
[23]</td><td>RXLCH</td><td><div style="word-wrap: break-word;"><b>Receive Left Channel Enable</b><br>
When monaural format is selected (MONO = 1), I2S will receive right channel data if RXLCH is set to 0, and receive left channel data if RXLCH is set to 1.<br>
0 = Receives right channel data when monaural format is selected.<br>
1 = Receives left channel data when monaural format is selected.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTRL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CLKDIV</font><br><p> <font size="2">
Offset: 0x04  I2S Clock Divider Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>MCLK_DIV</td><td><div style="word-wrap: break-word;"><b>Master Clock Divider</b><br>
If the external crystal frequency is (2xMCLK_DIV)*256fs then software can program these bits to generate 256fs clock frequency to audio CODEC chip.<br>
If MCLK_DIV is set to "0", MCLK is the same as external clock input.<br>
For example, sampling rate is 48 kHz and the external crystal clock is 12.288 MHz, set MCLK_DIV=0.<br>
MCLK = I2SCLK/(2x(MCLK_DIV)).<br>
</div></td></tr><tr><td>
[15:8]</td><td>BCLK_DIV</td><td><div style="word-wrap: break-word;"><b>Bit Clock Divider</b><br>
If I2S is operated in Master mode, bit clock is provided by this chip.<br>
Software can program these bits to generate sampling rate clock frequency.<br>
BCLK = I2SCLK /(2x(BCLK_DIV + 1)).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CLKDIV;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">INTEN</font><br><p> <font size="2">
Offset: 0x08  I2S Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RXUDFIE</td><td><div style="word-wrap: break-word;"><b>Receiving FIFO Underflow Interrupt Enable</b><br>
Interrupt occurs if this bit is set to "1" and receiving FIFO underflow flag is set to "1".<br>
0 = Interrupt Disabled.<br>
1 = Interrupt Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>RXOVFIE</td><td><div style="word-wrap: break-word;"><b>Receiving FIFO Overflow Interrupt Enable</b><br>
Interrupt occurs if this bit is set to "1" and receiving FIFO overflow flag is set to "1"<br>
0 = Interrupt Disabled.<br>
1 = Interrupt Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>RXTHIE</td><td><div style="word-wrap: break-word;"><b>Receiving FIFO Threshold Level Interrupt Enable</b><br>
Interrupt occurs if this bit is set to "1" and data words in receiving FIFO is less than RXTH[2:0].<br>
0 = Interrupt Disabled.<br>
1 = Interrupt Enabled.<br>
</div></td></tr><tr><td>
[8]</td><td>TXUDFIE</td><td><div style="word-wrap: break-word;"><b>Transmitting FIFO Underflow Interrupt Enable</b><br>
Interrupt occurs if this bit is set to "1" and transmitting FIFO underflow flag is set to "1".<br>
0 = Interrupt Disabled.<br>
1 = Interrupt Enabled.<br>
</div></td></tr><tr><td>
[9]</td><td>TXOVFIE</td><td><div style="word-wrap: break-word;"><b>Transmitting FIFO Overflow Interrupt Enable</b><br>
Interrupt occurs if this bit is set to "1" and transmitting FIFO overflow flag is set to "1"<br>
0 = Interrupt Disabled.<br>
1 = Interrupt Enabled.<br>
</div></td></tr><tr><td>
[10]</td><td>TXTHIE</td><td><div style="word-wrap: break-word;"><b>Transmitting FIFO Threshold Level Interrupt Enable</b><br>
Interrupt occurs if this bit is set to "1" and data words in transmitting FIFO is less than TXTH[2:0].<br>
0 = Interrupt Disabled.<br>
1 = Interrupt Enabled.<br>
</div></td></tr><tr><td>
[11]</td><td>RZCIE</td><td><div style="word-wrap: break-word;"><b>Right Channel Zero Cross Interrupt Enable</b><br>
Interrupt occurs if this bit is set to "1" and right channel is zero crossing.<br>
0 = Interrupt Disabled.<br>
1 = Interrupt Enabled.<br>
</div></td></tr><tr><td>
[12]</td><td>LZCIE</td><td><div style="word-wrap: break-word;"><b>Left Channel Zero Cross Interrupt Enable</b><br>
Interrupt occurs if this bit is set to "1" and left channel is zero crossing.<br>
0 = Interrupt Disabled.<br>
1 = Interrupt Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t INTEN;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">STATUS</font><br><p> <font size="2">
Offset: 0x0C  I2S Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>I2SINT</td><td><div style="word-wrap: break-word;"><b>I2S Interrupt Flag</b><br>
0 = No I2S interrupt.<br>
1 = I2S interrupt occurred.<br>
It is wire-OR of I2STXINT and I2SRXINT bits.<br>
This bit is read only.<br>
</div></td></tr><tr><td>
[1]</td><td>I2SRXINT</td><td><div style="word-wrap: break-word;"><b>I2S Receiving Interrupt</b><br>
0 = No receiving interrupt occurred.<br>
1 = Receiving interrupt occurred.<br>
This bit is read only<br>
</div></td></tr><tr><td>
[2]</td><td>I2STXINT</td><td><div style="word-wrap: break-word;"><b>I2S Transmit Interrupt</b><br>
0 = No transmit interrupt occurred.<br>
1 = Transmit interrupt occurred.<br>
This bit is read only<br>
</div></td></tr><tr><td>
[3]</td><td>RIGHT</td><td><div style="word-wrap: break-word;"><b>Right Channel</b><br>
This bit indicates the current transmitting data is belong to right channel<br>
0 = Left channel.<br>
1 = Right channel.<br>
This bit is read only<br>
</div></td></tr><tr><td>
[8]</td><td>RXUDF</td><td><div style="word-wrap: break-word;"><b>Receiving FIFO Underflow Flag</b><br>
Read the receiving FIFO when it is empty, this bit set to "1" indicate underflow occur.<br>
0 = No underflow occurred.<br>
1 = Underflow occurred.<br>
This bit is cleared by writing 1.<br>
</div></td></tr><tr><td>
[9]</td><td>RXOVF</td><td><div style="word-wrap: break-word;"><b>Receiving FIFO Overflow Flag</b><br>
When the receiving FIFO is full and receiving hardware attempts to write data into receiving FIFO then this bit is set to "1".<br>
Data in 1st buffer is overwritten.<br>
0 = No overflow occurred.<br>
1 = Overflow occurred.<br>
This bit is cleared by writing 1.<br>
</div></td></tr><tr><td>
[10]</td><td>RXTHF</td><td><div style="word-wrap: break-word;"><b>Receiving FIFO Threshold Flag</b><br>
When data word(s) in the receiving FIFO is equal to or higher than threshold value set in RXTH[2:0], the RXTHF bit becomes to "1".<br>
It keeps at "1" till RX_LEVEL[3:0] less than RXTH[1:0] after software reads data from the RXFIFO register.<br>
0 = Data word(s) in receiving FIFO is lower than threshold level.<br>
1 = Data word(s) in receiving FIFO is equal to or higher than threshold level.<br>
This bit is read only<br>
</div></td></tr><tr><td>
[11]</td><td>RXFULL</td><td><div style="word-wrap: break-word;"><b>Receiving FIFO Full</b><br>
This bit reflect data word number in the receiving FIFO is 8<br>
0 = Not full.<br>
1 = Full.<br>
This bit is read only<br>
</div></td></tr><tr><td>
[12]</td><td>RXEMPTY</td><td><div style="word-wrap: break-word;"><b>Receiving FIFO Empty</b><br>
This bit reflect data word number in the receiving FIFO is zero<br>
0 = Empty.<br>
1 = Not empty.<br>
This bit is read only.<br>
</div></td></tr><tr><td>
[16]</td><td>TXUDF</td><td><div style="word-wrap: break-word;"><b>Transmitting FIFO Underflow Flag</b><br>
When the transmitting FIFO is empty and shift logic hardware read data from the data FIFO causes this set to "1".<br>
0 = No underflow.<br>
1 = Underflow.<br>
This bit is cleared by writing 1.<br>
</div></td></tr><tr><td>
[17]</td><td>TXOVF</td><td><div style="word-wrap: break-word;"><b>Transmit FIFO Overflow Flag</b><br>
Write data to the transmitting FIFO when it is full and this bit will set to "1"<br>
0 = No overflow.<br>
1 = Overflow.<br>
This bit is cleared by writing 1.<br>
</div></td></tr><tr><td>
[18]</td><td>TXTHF</td><td><div style="word-wrap: break-word;"><b>Transmitting FIFO Threshold Flag</b><br>
When data word(s) in the transmitting FIFO is equal to or lower than threshold value set in TXTH[2:0],the TXTHF bit becomes to "1".<br>
It keeps at 1 till TX_LEVEL[3:0] is higher than TXTH[1:0] after software writes data into the TXFIFO register.<br>
0 = Data word(s) in transmitting FIFO is higher than threshold level.<br>
1 = Data word(s) in transmitting FIFO is equal to or lower than threshold level.<br>
This bit is read only<br>
</div></td></tr><tr><td>
[19]</td><td>TXFULL</td><td><div style="word-wrap: break-word;"><b>Transmitting FIFO Full</b><br>
This bit reflect data word number in the transmitting FIFO is 8<br>
0 = Full.<br>
1 = Not full.<br>
This bit is read only<br>
</div></td></tr><tr><td>
[20]</td><td>TXEMPTY</td><td><div style="word-wrap: break-word;"><b>Transmitting FIFO Empty</b><br>
This bit reflect data word number in the transmitting FIFO is zero<br>
0 = Empty.<br>
1 = Not empty.<br>
This bit is read only.<br>
</div></td></tr><tr><td>
[21]</td><td>TXBUSY</td><td><div style="word-wrap: break-word;"><b>Transmitting Busy</b><br>
This bit is cleared to 0 when all data in the transmitting FIFO and shift buffer is shifted out.<br>
Set this bit to 1 when 1st data is loading to shift buffer.<br>
0 = Transmit shift buffer is empty.<br>
1 = Transmit shift buffer is busy.<br>
This bit is read only.<br>
</div></td></tr><tr><td>
[22]</td><td>RZCF</td><td><div style="word-wrap: break-word;"><b>Right Channel Zero Cross Flag</b><br>
It indicates the data sign of right channel next sample data is changed or all data bits are zero.<br>
0 = No zero cross.<br>
1 = Right channel zero cross is detected.<br>
This bit is cleared by writing 1.<br>
</div></td></tr><tr><td>
[23]</td><td>LZCF</td><td><div style="word-wrap: break-word;"><b>Left Channel Zero Cross Flag</b><br>
It indicates the next sample data sign bit of left channel is changed or all data bits are zero.<br>
0 = No zero cross.<br>
1 = Left channel zero cross is detected.<br>
This bit is cleared by writing 1.<br>
</div></td></tr><tr><td>
[27:24]</td><td>RX_LEVEL</td><td><div style="word-wrap: break-word;"><b>Receive FIFO Level</b><br>
These bits indicate the number of word(s) in the receiving FIFO<br>
</div></td></tr><tr><td>
[31:28]</td><td>TX_LEVEL</td><td><div style="word-wrap: break-word;"><b>Transmitting FIFO Level</b><br>
These bits indicate the number of word(s) in the transmitting FIFO<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t STATUS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TXFIFO</font><br><p> <font size="2">
Offset: 0x10  I2S Transmit FIFO Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>TXFIFO</td><td><div style="word-wrap: break-word;"><b>Transmitting FIFO Register</b><br>
I2S contains 8 words (8x32-bit) data buffer for data transmitting.<br>
Write data to this register in order to prepare data for transmitting.<br>
The remaining word number is indicated by TX_LEVEL[3:0] in the I2S_STATUS register.<br>
This register is write only.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __O  uint32_t TXFIFO;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RXFIFO</font><br><p> <font size="2">
Offset: 0x14  I2S Receive FIFO Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>RXFIFO</td><td><div style="word-wrap: break-word;"><b>Receiving FIFO Register</b><br>
I2S contains 8 words (8x32-bit) data buffer for data receiving.<br>
Read this register to get data in FIFO.<br>
The remaining data word number is indicated by RX_LEVEL[3:0] in the I2S_STATUS register.<br>
This register is read only.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t RXFIFO;

} I2S_T;

/**
    @addtogroup I2S_CONST I2S Bit Field Definition
    Constant Definitions for I2S Controller
@{ */

#define I2S_CTRL_I2SEN_Pos               (0)                                               /*!< I2S_T::CTRL: I2SEN Position               */
#define I2S_CTRL_I2SEN_Msk               (0x1ul << I2S_CTRL_I2SEN_Pos)                     /*!< I2S_T::CTRL: I2SEN Mask                   */

#define I2S_CTRL_TXEN_Pos                (1)                                               /*!< I2S_T::CTRL: TXEN Position                */
#define I2S_CTRL_TXEN_Msk                (0x1ul << I2S_CTRL_TXEN_Pos)                      /*!< I2S_T::CTRL: TXEN Mask                    */

#define I2S_CTRL_RXEN_Pos                (2)                                               /*!< I2S_T::CTRL: RXEN Position                */
#define I2S_CTRL_RXEN_Msk                (0x1ul << I2S_CTRL_RXEN_Pos)                      /*!< I2S_T::CTRL: RXEN Mask                    */

#define I2S_CTRL_MUTE_Pos                (3)                                               /*!< I2S_T::CTRL: MUTE Position                */
#define I2S_CTRL_MUTE_Msk                (0x1ul << I2S_CTRL_MUTE_Pos)                      /*!< I2S_T::CTRL: MUTE Mask                    */

#define I2S_CTRL_WORDWIDTH_Pos           (4)                                               /*!< I2S_T::CTRL: WORDWIDTH Position           */
#define I2S_CTRL_WORDWIDTH_Msk           (0x3ul << I2S_CTRL_WORDWIDTH_Pos)                 /*!< I2S_T::CTRL: WORDWIDTH Mask               */

#define I2S_CTRL_MONO_Pos                (6)                                               /*!< I2S_T::CTRL: MONO Position                */
#define I2S_CTRL_MONO_Msk                (0x1ul << I2S_CTRL_MONO_Pos)                      /*!< I2S_T::CTRL: MONO Mask                    */

#define I2S_CTRL_FORMAT_Pos              (7)                                               /*!< I2S_T::CTRL: FORMAT Position              */
#define I2S_CTRL_FORMAT_Msk              (0x1ul << I2S_CTRL_FORMAT_Pos)                    /*!< I2S_T::CTRL: FORMAT Mask                  */

#define I2S_CTRL_SLAVE_Pos               (8)                                               /*!< I2S_T::CTRL: SLAVE Position               */
#define I2S_CTRL_SLAVE_Msk               (0x1ul << I2S_CTRL_SLAVE_Pos)                     /*!< I2S_T::CTRL: SLAVE Mask                   */

#define I2S_CTRL_TXTH_Pos                (9)                                               /*!< I2S_T::CTRL: TXTH Position                */
#define I2S_CTRL_TXTH_Msk                (0x7ul << I2S_CTRL_TXTH_Pos)                      /*!< I2S_T::CTRL: TXTH Mask                    */

#define I2S_CTRL_RXTH_Pos                (12)                                              /*!< I2S_T::CTRL: RXTH Position                */
#define I2S_CTRL_RXTH_Msk                (0x7ul << I2S_CTRL_RXTH_Pos)                      /*!< I2S_T::CTRL: RXTH Mask                    */

#define I2S_CTRL_MCLKEN_Pos              (15)                                              /*!< I2S_T::CTRL: MCLKEN Position              */
#define I2S_CTRL_MCLKEN_Msk              (0x1ul << I2S_CTRL_MCLKEN_Pos)                    /*!< I2S_T::CTRL: MCLKEN Mask                  */

#define I2S_CTRL_RCHZCEN_Pos             (16)                                              /*!< I2S_T::CTRL: RCHZCEN Position             */
#define I2S_CTRL_RCHZCEN_Msk             (0x1ul << I2S_CTRL_RCHZCEN_Pos)                   /*!< I2S_T::CTRL: RCHZCEN Mask                 */

#define I2S_CTRL_LCHZCEN_Pos             (17)                                              /*!< I2S_T::CTRL: LCHZCEN Position             */
#define I2S_CTRL_LCHZCEN_Msk             (0x1ul << I2S_CTRL_LCHZCEN_Pos)                   /*!< I2S_T::CTRL: LCHZCEN Mask                 */

#define I2S_CTRL_CLR_TXFIFO_Pos          (18)                                              /*!< I2S_T::CTRL: CLR_TXFIFO Position          */
#define I2S_CTRL_CLR_TXFIFO_Msk          (0x1ul << I2S_CTRL_CLR_TXFIFO_Pos)                /*!< I2S_T::CTRL: CLR_TXFIFO Mask              */

#define I2S_CTRL_CLR_RXFIFO_Pos          (19)                                              /*!< I2S_T::CTRL: CLR_RXFIFO Position          */
#define I2S_CTRL_CLR_RXFIFO_Msk          (0x1ul << I2S_CTRL_CLR_RXFIFO_Pos)                /*!< I2S_T::CTRL: CLR_RXFIFO Mask              */

#define I2S_CTRL_TXDMA_Pos               (20)                                              /*!< I2S_T::CTRL: TXDMA Position               */
#define I2S_CTRL_TXDMA_Msk               (0x1ul << I2S_CTRL_TXDMA_Pos)                     /*!< I2S_T::CTRL: TXDMA Mask                   */

#define I2S_CTRL_RXDMA_Pos               (21)                                              /*!< I2S_T::CTRL: RXDMA Position               */
#define I2S_CTRL_RXDMA_Msk               (0x1ul << I2S_CTRL_RXDMA_Pos)                     /*!< I2S_T::CTRL: RXDMA Mask                   */

#define I2S_CTRL_RXLCH_Pos               (23)                                              /*!< I2S_T::CTRL: RXLCH Position               */
#define I2S_CTRL_RXLCH_Msk               (0x1ul << I2S_CTRL_RXLCH_Pos)                     /*!< I2S_T::CTRL: RXLCH Mask                   */

#define I2S_CLKDIV_MCLK_DIV_Pos          (0)                                               /*!< I2S_T::CLKDIV: MCLK_DIV Position          */
#define I2S_CLKDIV_MCLK_DIV_Msk          (0x7ul << I2S_CLKDIV_MCLK_DIV_Pos)                /*!< I2S_T::CLKDIV: MCLK_DIV Mask              */

#define I2S_CLKDIV_BCLK_DIV_Pos          (8)                                               /*!< I2S_T::CLKDIV: BCLK_DIV Position          */
#define I2S_CLKDIV_BCLK_DIV_Msk          (0xfful << I2S_CLKDIV_BCLK_DIV_Pos)               /*!< I2S_T::CLKDIV: BCLK_DIV Mask              */

#define I2S_INTEN_RXUDFIE_Pos            (0)                                               /*!< I2S_T::INTEN: RXUDFIE Position            */
#define I2S_INTEN_RXUDFIE_Msk            (0x1ul << I2S_INTEN_RXUDFIE_Pos)                  /*!< I2S_T::INTEN: RXUDFIE Mask                */

#define I2S_INTEN_RXOVFIE_Pos            (1)                                               /*!< I2S_T::INTEN: RXOVFIE Position            */
#define I2S_INTEN_RXOVFIE_Msk            (0x1ul << I2S_INTEN_RXOVFIE_Pos)                  /*!< I2S_T::INTEN: RXOVFIE Mask                */

#define I2S_INTEN_RXTHIE_Pos             (2)                                               /*!< I2S_T::INTEN: RXTHIE Position             */
#define I2S_INTEN_RXTHIE_Msk             (0x1ul << I2S_INTEN_RXTHIE_Pos)                   /*!< I2S_T::INTEN: RXTHIE Mask                 */

#define I2S_INTEN_TXUDFIE_Pos            (8)                                               /*!< I2S_T::INTEN: TXUDFIE Position            */
#define I2S_INTEN_TXUDFIE_Msk            (0x1ul << I2S_INTEN_TXUDFIE_Pos)                  /*!< I2S_T::INTEN: TXUDFIE Mask                */

#define I2S_INTEN_TXOVFIE_Pos            (9)                                               /*!< I2S_T::INTEN: TXOVFIE Position            */
#define I2S_INTEN_TXOVFIE_Msk            (0x1ul << I2S_INTEN_TXOVFIE_Pos)                  /*!< I2S_T::INTEN: TXOVFIE Mask                */

#define I2S_INTEN_TXTHIE_Pos             (10)                                              /*!< I2S_T::INTEN: TXTHIE Position             */
#define I2S_INTEN_TXTHIE_Msk             (0x1ul << I2S_INTEN_TXTHIE_Pos)                   /*!< I2S_T::INTEN: TXTHIE Mask                 */

#define I2S_INTEN_RZCIE_Pos              (11)                                              /*!< I2S_T::INTEN: RZCIE Position              */
#define I2S_INTEN_RZCIE_Msk              (0x1ul << I2S_INTEN_RZCIE_Pos)                    /*!< I2S_T::INTEN: RZCIE Mask                  */

#define I2S_INTEN_LZCIE_Pos              (12)                                              /*!< I2S_T::INTEN: LZCIE Position              */
#define I2S_INTEN_LZCIE_Msk              (0x1ul << I2S_INTEN_LZCIE_Pos)                    /*!< I2S_T::INTEN: LZCIE Mask                  */

#define I2S_STATUS_I2SINT_Pos            (0)                                               /*!< I2S_T::STATUS: I2SINT Position            */
#define I2S_STATUS_I2SINT_Msk            (0x1ul << I2S_STATUS_I2SINT_Pos)                  /*!< I2S_T::STATUS: I2SINT Mask                */

#define I2S_STATUS_I2SRXINT_Pos          (1)                                               /*!< I2S_T::STATUS: I2SRXINT Position          */
#define I2S_STATUS_I2SRXINT_Msk          (0x1ul << I2S_STATUS_I2SRXINT_Pos)                /*!< I2S_T::STATUS: I2SRXINT Mask              */

#define I2S_STATUS_I2STXINT_Pos          (2)                                               /*!< I2S_T::STATUS: I2STXINT Position          */
#define I2S_STATUS_I2STXINT_Msk          (0x1ul << I2S_STATUS_I2STXINT_Pos)                /*!< I2S_T::STATUS: I2STXINT Mask              */

#define I2S_STATUS_RIGHT_Pos             (3)                                               /*!< I2S_T::STATUS: RIGHT Position             */
#define I2S_STATUS_RIGHT_Msk             (0x1ul << I2S_STATUS_RIGHT_Pos)                   /*!< I2S_T::STATUS: RIGHT Mask                 */

#define I2S_STATUS_RXUDF_Pos             (8)                                               /*!< I2S_T::STATUS: RXUDF Position             */
#define I2S_STATUS_RXUDF_Msk             (0x1ul << I2S_STATUS_RXUDF_Pos)                   /*!< I2S_T::STATUS: RXUDF Mask                 */

#define I2S_STATUS_RXOVF_Pos             (9)                                               /*!< I2S_T::STATUS: RXOVF Position             */
#define I2S_STATUS_RXOVF_Msk             (0x1ul << I2S_STATUS_RXOVF_Pos)                   /*!< I2S_T::STATUS: RXOVF Mask                 */

#define I2S_STATUS_RXTHF_Pos             (10)                                              /*!< I2S_T::STATUS: RXTHF Position             */
#define I2S_STATUS_RXTHF_Msk             (0x1ul << I2S_STATUS_RXTHF_Pos)                   /*!< I2S_T::STATUS: RXTHF Mask                 */

#define I2S_STATUS_RXFULL_Pos            (11)                                              /*!< I2S_T::STATUS: RXFULL Position            */
#define I2S_STATUS_RXFULL_Msk            (0x1ul << I2S_STATUS_RXFULL_Pos)                  /*!< I2S_T::STATUS: RXFULL Mask                */

#define I2S_STATUS_RXEMPTY_Pos           (12)                                              /*!< I2S_T::STATUS: RXEMPTY Position           */
#define I2S_STATUS_RXEMPTY_Msk           (0x1ul << I2S_STATUS_RXEMPTY_Pos)                 /*!< I2S_T::STATUS: RXEMPTY Mask               */

#define I2S_STATUS_TXUDF_Pos             (16)                                              /*!< I2S_T::STATUS: TXUDF Position             */
#define I2S_STATUS_TXUDF_Msk             (0x1ul << I2S_STATUS_TXUDF_Pos)                   /*!< I2S_T::STATUS: TXUDF Mask                 */

#define I2S_STATUS_TXOVF_Pos             (17)                                              /*!< I2S_T::STATUS: TXOVF Position             */
#define I2S_STATUS_TXOVF_Msk             (0x1ul << I2S_STATUS_TXOVF_Pos)                   /*!< I2S_T::STATUS: TXOVF Mask                 */

#define I2S_STATUS_TXTHF_Pos             (18)                                              /*!< I2S_T::STATUS: TXTHF Position             */
#define I2S_STATUS_TXTHF_Msk             (0x1ul << I2S_STATUS_TXTHF_Pos)                   /*!< I2S_T::STATUS: TXTHF Mask                 */

#define I2S_STATUS_TXFULL_Pos            (19)                                              /*!< I2S_T::STATUS: TXFULL Position            */
#define I2S_STATUS_TXFULL_Msk            (0x1ul << I2S_STATUS_TXFULL_Pos)                  /*!< I2S_T::STATUS: TXFULL Mask                */

#define I2S_STATUS_TXEMPTY_Pos           (20)                                              /*!< I2S_T::STATUS: TXEMPTY Position           */
#define I2S_STATUS_TXEMPTY_Msk           (0x1ul << I2S_STATUS_TXEMPTY_Pos)                 /*!< I2S_T::STATUS: TXEMPTY Mask               */

#define I2S_STATUS_TXBUSY_Pos            (21)                                              /*!< I2S_T::STATUS: TXBUSY Position            */
#define I2S_STATUS_TXBUSY_Msk            (0x1ul << I2S_STATUS_TXBUSY_Pos)                  /*!< I2S_T::STATUS: TXBUSY Mask                */

#define I2S_STATUS_RZCF_Pos              (22)                                              /*!< I2S_T::STATUS: RZCF Position              */
#define I2S_STATUS_RZCF_Msk              (0x1ul << I2S_STATUS_RZCF_Pos)                    /*!< I2S_T::STATUS: RZCF Mask                  */

#define I2S_STATUS_LZCF_Pos              (23)                                              /*!< I2S_T::STATUS: LZCF Position              */
#define I2S_STATUS_LZCF_Msk              (0x1ul << I2S_STATUS_LZCF_Pos)                    /*!< I2S_T::STATUS: LZCF Mask                  */

#define I2S_STATUS_RX_LEVEL_Pos          (24)                                              /*!< I2S_T::STATUS: RX_LEVEL Position          */
#define I2S_STATUS_RX_LEVEL_Msk          (0xful << I2S_STATUS_RX_LEVEL_Pos)                /*!< I2S_T::STATUS: RX_LEVEL Mask              */

#define I2S_STATUS_TX_LEVEL_Pos          (28)                                              /*!< I2S_T::STATUS: TX_LEVEL Position          */
#define I2S_STATUS_TX_LEVEL_Msk          (0xful << I2S_STATUS_TX_LEVEL_Pos)                /*!< I2S_T::STATUS: TX_LEVEL Mask              */

#define I2S_TXFIFO_TXFIFO_Pos            (0)                                               /*!< I2S_T::TXFIFO: TXFIFO Position            */
#define I2S_TXFIFO_TXFIFO_Msk            (0xfffffffful << I2S_TXFIFO_TXFIFO_Pos)           /*!< I2S_T::TXFIFO: TXFIFO Mask                */

#define I2S_RXFIFO_RXFIFO_Pos            (0)                                               /*!< I2S_T::RXFIFO: RXFIFO Position            */
#define I2S_RXFIFO_RXFIFO_Msk            (0xfffffffful << I2S_RXFIFO_RXFIFO_Pos)           /*!< I2S_T::RXFIFO: RXFIFO Mask                */

/**@}*/ /* I2S_CONST */
/**@}*/ /* end of I2S register group */


/*---------------------- Interrupt Controller -------------------------*/
/**
    @addtogroup INT Interrupt Controller (INTR)
    Memory Mapped Structure for INT Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IRQ0SRC ~ IRQ31SRC</font><br><p> <font size="2">
Offset: 0x00 ~0x7C IRQ0~IRQ31 Interrupt Source Identity</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>INT_SRC</td><td><div style="word-wrap: break-word;"><b>Interrupt Source</b><br>
Define the interrupt sources for interrupt event.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t IRQSRC[32];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">NMI_SEL</font><br><p> <font size="2">
Offset: 0x80  NMI Source Interrupt Select Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[4:0]</td><td>NMI_SEL</td><td><div style="word-wrap: break-word;"><b>The NMI interrupt to Cortex-M0 can be selected from one of the interrupt[31:0]</b><br>
The NMI_SEL bit[4:0] used to select the NMI interrupt source<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t NMI_SEL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MCU_IRQ</font><br><p> <font size="2">
Offset: 0x84  MCU IRQ Number Identity Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>MCU_IRQ</td><td><div style="word-wrap: break-word;"><b>MCU IRQ Source Register</b><br>
The IRQ collects all the interrupts from the peripherals and generates the synchronous interrupt to Cortex-M0 core.<br>
There are two modes to generate interrupt to Cortex-M0 - the normal mode and test mode.<br>
In Normal mode (control by NMI_SEL register bit [7] = 0) The MCU_IRQ collects all interrupts from each peripheral<br>
and synchronizes  them and then interrupts  the Cortex-M0.<br>
In Test mode, all the interrupts from peripheral are blocked, and the interrupts sent to<br>
MCU are replaced by set the bit31~bit0.<br>
When the IRQ[n] is 0, setting IRQ[n] to 1 will generate an interrupt to Cortex-M0 NVIC[n].<br>
When the IRQ[n] is 1 (mean an interrupt is assert), setting 1 to the MCU_bit[n] will clear the interrupt and setting IRQ[n] 0 has no effect.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MCU_IRQ;

} INTR_T;

/**
    @addtogroup INT_CONST INT Bit Field Definition
    Constant Definitions for INT Controller
@{ */

#define INTR_IRQSRC_INT_SRC_Pos           (0)                                               /*!< INTR_T::IRQSRC: INT_SRC Position           */
#define INTR_IRQSRC_INT_SRC_Msk           (0xful << INTR_IRQ0SRC_INT_SRC_Pos)               /*!< INTR_T::IRQSRC: INT_SRC Mask               */

#define INTR_NMI_SEL_NMISEL_Pos           (0)                                               /*!< INTR_T::NMI_SEL: NMISEL Position           */
#define INTR_NMI_SEL_NMISEL_Msk           (0x1ful << INTR_NMI_SEL_NMISEL_Pos)               /*!< INTR_T::NMI_SEL: NMISEL Mask               */

#define INTR_MCU_IRQ_MCU_IRQ_Pos          (0)                                               /*!< INTR_T::MCU_IRQ: MCU_IRQ Position          */
#define INTR_MCU_IRQ_MCU_IRQ_Msk          (0xfffffffful << INTR_MCU_IRQ_MCU_IRQ_Pos)        /*!< INTR_T::MCU_IRQ: MCU_IRQ Mask              */

/**@}*/ /* INTR_CONST */
/**@}*/ /* end of INTR register group */


/*---------------------- LCD Controller -------------------------*/
/**
    @addtogroup LCD LCD Controller(LCD)
    Memory Mapped Structure for LCD Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTL</font><br><p> <font size="2">
Offset: 0x00  LCD Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>EN</td><td><div style="word-wrap: break-word;"><b>LCD Enable</b><br>
0 = LCD controller operation Disabled.<br>
1 = LCD controller operation Enabled.<br>
</div></td></tr><tr><td>
[3:1]</td><td>MUX</td><td><div style="word-wrap: break-word;"><b>Mux Select</b><br>
000 = Static.<br>
001 = 1/2 duty.<br>
010 = 1/3 duty.<br>
011 = 1/4 duty.<br>
100 = 1/5 duty.<br>
101 = 1/6 duty.<br>
110 = Reserved.<br>
111 = Reserved.<br>
Note : User does not need to set PD_H_MFP bit field, but only to set the MUX bit field to switch LCD_SEG0 and LCD_SEG1 to LCD_COM4 and LCD_COM5 for Nano110 and Nano130 series.<br>
</div></td></tr><tr><td>
[6:4]</td><td>FREQ</td><td><div style="word-wrap: break-word;"><b>LCD Frequency Selection</b><br>
000 = LCDCLK Divided by 32.<br>
001 = LCDCLK Divided by 64.<br>
010 = LCDCLK Divided by 96.<br>
011 = LCDCLK Divided by 128.<br>
100 = LCDCLK Divided by 192.<br>
101 = LCDCLK Divided by 256.<br>
110 = LCDCLK Divided by 384.<br>
111 = LCDCLK Divided by 512.<br>
</div></td></tr><tr><td>
[7]</td><td>BLINK</td><td><div style="word-wrap: break-word;"><b>LCD Blinking Enable</b><br>
0 = Blinking Disabled.<br>
1 = Blinking Enabled.<br>
</div></td></tr><tr><td>
[8]</td><td>PDDISP_EN</td><td><div style="word-wrap: break-word;"><b>Power Down Display Enable</b><br>
The LCD can be programmed to be displayed or not be displayed at power down state by PDDISP_EN setting.<br>
0 = LCD display Disabled ( LCD is put out) at power down state.<br>
1 = LCD display Enabled (LCD keeps the display) at power down state.<br>
</div></td></tr><tr><td>
[9]</td><td>PDINT_EN</td><td><div style="word-wrap: break-word;"><b>Power Down Interrupt Enable</b><br>
If the power down request is triggered from system management, LCD controller will execute the frame completely to avoid the DC component.<br>
When the frame is executed completely, the LCD power down interrupt signal is generated to inform system management that LCD controller is ready to enter power down state, if PDINT_EN is set to 1.<br>
Otherwise, if PDINT_EN is set to 0, the LCD power down interrupt signal is blocked and the interrupt is disabled to send to system management.<br>
0 = Power Down Interrupt Disabled.<br>
1 = Power Down Interrupt Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DISPCTL</font><br><p> <font size="2">
Offset: 0x04  LCD Display Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>CPUMP_EN</td><td><div style="word-wrap: break-word;"><b>Charge Pump Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[2:1]</td><td>BIAS_SEL</td><td><div style="word-wrap: break-word;"><b>Bias Selection</b><br>
00 = Static.<br>
01 = 1/2 Bias.<br>
10 = 1/3 Bias.<br>
11 = Reserved.<br>
</div></td></tr><tr><td>
[4]</td><td>IBRL_EN</td><td><div style="word-wrap: break-word;"><b>Internal Bias Reference Ladder Enable</b><br>
0 = Bias reference ladder Disabled.<br>
1 = Bias reference ladder Enabled.<br>
</div></td></tr><tr><td>
[6]</td><td>BV_SEL</td><td><div style="word-wrap: break-word;"><b>Bias Voltage Type Selection</b><br>
0 = C-Type bias mode. Bias voltage source from internal bias generator.<br>
1 = R-Type bias mode. Bias voltage source from external bias generator.<br>
Note: The external resistor ladder should be connected to the V1 pin, V2 pin, V3 pin and VSS.<br>
The VLCD pin should also be connected to VDD.<br>
</div></td></tr><tr><td>
[10:8]</td><td>CPUMP_VOL_SET</td><td><div style="word-wrap: break-word;"><b>Charge Pump Voltage Selection</b><br>
000 = 2.7V.<br>
001 = 2.8V.<br>
010 = 2.9V.<br>
011 = 3.0V.<br>
100 = 3.1V.<br>
101 = 3.2V.<br>
110 = 3.3V.<br>
111 = 3.4V.<br>
</div></td></tr><tr><td>
[13:11]</td><td>CPUMP_FREQ</td><td><div style="word-wrap: break-word;"><b>Charge Pump Frequency Selection</b><br>
000 = LCDCLK.<br>
001 = LCDCLK/2.<br>
010 = LCDCLK/4.<br>
011 = LCDCLK/8.<br>
100 = LCDCLK/16.<br>
101 = LCDCLK/32.<br>
110 = LCDCLK/64.<br>
111 = LCDCLK/128.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DISPCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MEM_0</font><br><p> <font size="2">
Offset: 0x08  LCD SEG3 ~ SEG0 data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>SEG_0_4x</td><td><div style="word-wrap: break-word;"><b>SEG_0_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[14:8]</td><td>SEG_1_4x</td><td><div style="word-wrap: break-word;"><b>SEG_1_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[21:16]</td><td>SEG_2_4x</td><td><div style="word-wrap: break-word;"><b>SEG_2_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[29:24]</td><td>SEG_3_4x</td><td><div style="word-wrap: break-word;"><b>SEG_3_4x DATA for COM (x = 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MEM_0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MEM_1</font><br><p> <font size="2">
Offset: 0x0C  LCD SEG7 ~ SEG4 data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>SEG_0_4x</td><td><div style="word-wrap: break-word;"><b>SEG_0_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[14:8]</td><td>SEG_1_4x</td><td><div style="word-wrap: break-word;"><b>SEG_1_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[21:16]</td><td>SEG_2_4x</td><td><div style="word-wrap: break-word;"><b>SEG_2_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[29:24]</td><td>SEG_3_4x</td><td><div style="word-wrap: break-word;"><b>SEG_3_4x DATA for COM (x = 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MEM_1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MEM_2</font><br><p> <font size="2">
Offset: 0x10  LCD SEG11 ~ SEG8 data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>SEG_0_4x</td><td><div style="word-wrap: break-word;"><b>SEG_0_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[14:8]</td><td>SEG_1_4x</td><td><div style="word-wrap: break-word;"><b>SEG_1_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[21:16]</td><td>SEG_2_4x</td><td><div style="word-wrap: break-word;"><b>SEG_2_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[29:24]</td><td>SEG_3_4x</td><td><div style="word-wrap: break-word;"><b>SEG_3_4x DATA for COM (x = 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MEM_2;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MEM_3</font><br><p> <font size="2">
Offset: 0x14  LCD SEG15 ~ SEG12 data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>SEG_0_4x</td><td><div style="word-wrap: break-word;"><b>SEG_0_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[14:8]</td><td>SEG_1_4x</td><td><div style="word-wrap: break-word;"><b>SEG_1_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[21:16]</td><td>SEG_2_4x</td><td><div style="word-wrap: break-word;"><b>SEG_2_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[29:24]</td><td>SEG_3_4x</td><td><div style="word-wrap: break-word;"><b>SEG_3_4x DATA for COM (x = 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MEM_3;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MEM_4</font><br><p> <font size="2">
Offset: 0x18  LCD SEG19 ~ SEG16 data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>SEG_0_4x</td><td><div style="word-wrap: break-word;"><b>SEG_0_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[14:8]</td><td>SEG_1_4x</td><td><div style="word-wrap: break-word;"><b>SEG_1_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[21:16]</td><td>SEG_2_4x</td><td><div style="word-wrap: break-word;"><b>SEG_2_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[29:24]</td><td>SEG_3_4x</td><td><div style="word-wrap: break-word;"><b>SEG_3_4x DATA for COM (x = 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MEM_4;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MEM_5</font><br><p> <font size="2">
Offset: 0x1C  LCD SEG23 ~ SEG20 data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>SEG_0_4x</td><td><div style="word-wrap: break-word;"><b>SEG_0_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[14:8]</td><td>SEG_1_4x</td><td><div style="word-wrap: break-word;"><b>SEG_1_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[21:16]</td><td>SEG_2_4x</td><td><div style="word-wrap: break-word;"><b>SEG_2_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[29:24]</td><td>SEG_3_4x</td><td><div style="word-wrap: break-word;"><b>SEG_3_4x DATA for COM (x = 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MEM_5;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MEM_6</font><br><p> <font size="2">
Offset: 0x20  LCD SEG27 ~ SEG24 data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>SEG_0_4x</td><td><div style="word-wrap: break-word;"><b>SEG_0_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[14:8]</td><td>SEG_1_4x</td><td><div style="word-wrap: break-word;"><b>SEG_1_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[21:16]</td><td>SEG_2_4x</td><td><div style="word-wrap: break-word;"><b>SEG_2_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[29:24]</td><td>SEG_3_4x</td><td><div style="word-wrap: break-word;"><b>SEG_3_4x DATA for COM (x = 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MEM_6;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MEM_7</font><br><p> <font size="2">
Offset: 0x24  LCD SEG31 ~ SEG28 data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>SEG_0_4x</td><td><div style="word-wrap: break-word;"><b>SEG_0_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[14:8]</td><td>SEG_1_4x</td><td><div style="word-wrap: break-word;"><b>SEG_1_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[21:16]</td><td>SEG_2_4x</td><td><div style="word-wrap: break-word;"><b>SEG_2_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[29:24]</td><td>SEG_3_4x</td><td><div style="word-wrap: break-word;"><b>SEG_3_4x DATA for COM (x = 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MEM_7;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MEM_8</font><br><p> <font size="2">
Offset: 0x28  LCD SEG35 ~ SEG32 data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>SEG_0_4x</td><td><div style="word-wrap: break-word;"><b>SEG_0_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[14:8]</td><td>SEG_1_4x</td><td><div style="word-wrap: break-word;"><b>SEG_1_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[21:16]</td><td>SEG_2_4x</td><td><div style="word-wrap: break-word;"><b>SEG_2_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[29:24]</td><td>SEG_3_4x</td><td><div style="word-wrap: break-word;"><b>SEG_3_4x DATA for COM (x = 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MEM_8;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MEM_9</font><br><p> <font size="2">
Offset: 0x2C  LCD SEG39 ~ SEG36 data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>SEG_0_4x</td><td><div style="word-wrap: break-word;"><b>SEG_0_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[14:8]</td><td>SEG_1_4x</td><td><div style="word-wrap: break-word;"><b>SEG_1_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[21:16]</td><td>SEG_2_4x</td><td><div style="word-wrap: break-word;"><b>SEG_2_4x DATA for COM(x= 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr><tr><td>
[29:24]</td><td>SEG_3_4x</td><td><div style="word-wrap: break-word;"><b>SEG_3_4x DATA for COM (x = 0 ~ 9)</b><br>
LCD display data<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MEM_9;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">FCR</font><br><p> <font size="2">
Offset: 0x30  LCD frame counter control register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>FCEN</td><td><div style="word-wrap: break-word;"><b>LCD Frame Counter Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>FCINTEN</td><td><div style="word-wrap: break-word;"><b>LCD Frame Counter Interrupt Enable</b><br>
0 = Frame counter interrupt Disabled.<br>
1 = Frame counter interrupt Enabled.<br>
</div></td></tr><tr><td>
[3:2]</td><td>PRESCL</td><td><div style="word-wrap: break-word;"><b>Frame Counter Pre-Scaler Value</b><br>
00 = CLKframe/1.<br>
01 = CLKframe/2.<br>
10 = CLKframe/4.<br>
11 = CLKframe/8.<br>
</div></td></tr><tr><td>
[9:4]</td><td>FCV</td><td><div style="word-wrap: break-word;"><b>Frame Counter Top Value</b><br>
These 6 bits contain the top value of the Frame counter.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t FCR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">FCSTS</font><br><p> <font size="2">
Offset: 0x34  LCD frame counter status</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>FCSTS</td><td><div style="word-wrap: break-word;"><b>LCD Frame Counter Status</b><br>
0 = Frame counter value does not reach FCV (Frame Count TOP value).<br>
1 = Frame counter value reaches FCV (Frame Count TOP value).<br>
If the FCINTEN is s enabled, the frame counter overflow Interrupt is generated.<br>
</div></td></tr><tr><td>
[1]</td><td>PDSTS</td><td><div style="word-wrap: break-word;"><b>Power-Down Interrupt Status</b><br>
0 = Inform system manager that LCD controller is not ready to enter power-down state until this bit becomes 1 if power down is set and one frame is not executed completely.<br>
1 = Inform system manager that LCD controller is ready to enter power-down state if power down is set and one frame is executed completely<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t FCSTS;

} LCD_T;

/**
    @addtogroup LCD_CONST LCD Bit Field Definition
    Constant Definitions for LCD Controller
@{ */

#define LCD_CTL_EN_Pos                   (0)                                               /*!< LCD_T::CTL: EN Position                   */
#define LCD_CTL_EN_Msk                   (0x1ul << LCD_CTL_EN_Pos)                         /*!< LCD_T::CTL: EN Mask                       */

#define LCD_CTL_MUX_Pos                  (1)                                               /*!< LCD_T::CTL: MUX Position                  */
#define LCD_CTL_MUX_Msk                  (0x7ul << LCD_CTL_MUX_Pos)                        /*!< LCD_T::CTL: MUX Mask                      */

#define LCD_CTL_FREQ_Pos                 (4)                                               /*!< LCD_T::CTL: FREQ Position                 */
#define LCD_CTL_FREQ_Msk                 (0x7ul << LCD_CTL_FREQ_Pos)                       /*!< LCD_T::CTL: FREQ Mask                     */

#define LCD_CTL_BLINK_Pos                (7)                                               /*!< LCD_T::CTL: BLINK Position                */
#define LCD_CTL_BLINK_Msk                (0x1ul << LCD_CTL_BLINK_Pos)                      /*!< LCD_T::CTL: BLINK Mask                    */

#define LCD_CTL_PDDISP_EN_Pos            (8)                                               /*!< LCD_T::CTL: PDDISP_EN Position            */
#define LCD_CTL_PDDISP_EN_Msk            (0x1ul << LCD_CTL_PDDISP_EN_Pos)                  /*!< LCD_T::CTL: PDDISP_EN Mask                */

#define LCD_CTL_PDINT_EN_Pos             (9)                                               /*!< LCD_T::CTL: PDINT_EN Position             */
#define LCD_CTL_PDINT_EN_Msk             (0x1ul << LCD_CTL_PDINT_EN_Pos)                   /*!< LCD_T::CTL: PDINT_EN Mask                 */

#define LCD_DISPCTL_CPUMP_EN_Pos         (0)                                               /*!< LCD_T::DISPCTL: CPUMP_EN Position         */
#define LCD_DISPCTL_CPUMP_EN_Msk         (0x1ul << LCD_DISPCTL_CPUMP_EN_Pos)               /*!< LCD_T::DISPCTL: CPUMP_EN Mask             */

#define LCD_DISPCTL_BIAS_SEL_Pos         (1)                                               /*!< LCD_T::DISPCTL: BIAS_SEL Position         */
#define LCD_DISPCTL_BIAS_SEL_Msk         (0x3ul << LCD_DISPCTL_BIAS_SEL_Pos)               /*!< LCD_T::DISPCTL: BIAS_SEL Mask             */

#define LCD_DISPCTL_IBRL_EN_Pos          (4)                                               /*!< LCD_T::DISPCTL: IBRL_EN Position          */
#define LCD_DISPCTL_IBRL_EN_Msk          (0x1ul << LCD_DISPCTL_IBRL_EN_Pos)                /*!< LCD_T::DISPCTL: IBRL_EN Mask              */

#define LCD_DISPCTL_BV_SEL_Pos           (6)                                               /*!< LCD_T::DISPCTL: BV_SEL Position           */
#define LCD_DISPCTL_BV_SEL_Msk           (0x1ul << LCD_DISPCTL_BV_SEL_Pos)                 /*!< LCD_T::DISPCTL: BV_SEL Mask               */

#define LCD_DISPCTL_CPUMP_VOL_SET_Pos    (8)                                               /*!< LCD_T::DISPCTL: CPUMP_VOL_SET Position    */
#define LCD_DISPCTL_CPUMP_VOL_SET_Msk    (0x7ul << LCD_DISPCTL_CPUMP_VOL_SET_Pos)          /*!< LCD_T::DISPCTL: CPUMP_VOL_SET Mask        */

#define LCD_DISPCTL_CPUMP_FREQ_Pos       (11)                                              /*!< LCD_T::DISPCTL: CPUMP_FREQ Position       */
#define LCD_DISPCTL_CPUMP_FREQ_Msk       (0x7ul << LCD_DISPCTL_CPUMP_FREQ_Pos)             /*!< LCD_T::DISPCTL: CPUMP_FREQ Mask           */

#define LCD_MEM_0_SEG_0_4x_Pos           (0)                                               /*!< LCD_T::MEM_0: SEG_0_4x Position           */
#define LCD_MEM_0_SEG_0_4x_Msk           (0x3ful << LCD_MEM_0_SEG_0_4x_Pos)                /*!< LCD_T::MEM_0: SEG_0_4x Mask               */

#define LCD_MEM_0_SEG_1_4x_Pos           (8)                                               /*!< LCD_T::MEM_0: SEG_1_4x Position           */
#define LCD_MEM_0_SEG_1_4x_Msk           (0x7ful << LCD_MEM_0_SEG_1_4x_Pos)                /*!< LCD_T::MEM_0: SEG_1_4x Mask               */

#define LCD_MEM_0_SEG_2_4x_Pos           (16)                                              /*!< LCD_T::MEM_0: SEG_2_4x Position           */
#define LCD_MEM_0_SEG_2_4x_Msk           (0x3ful << LCD_MEM_0_SEG_2_4x_Pos)                /*!< LCD_T::MEM_0: SEG_2_4x Mask               */

#define LCD_MEM_0_SEG_3_4x_Pos           (24)                                              /*!< LCD_T::MEM_0: SEG_3_4x Position           */
#define LCD_MEM_0_SEG_3_4x_Msk           (0x3ful << LCD_MEM_0_SEG_3_4x_Pos)                /*!< LCD_T::MEM_0: SEG_3_4x Mask               */

#define LCD_MEM_1_SEG_0_4x_Pos           (0)                                               /*!< LCD_T::MEM_1: SEG_0_4x Position           */
#define LCD_MEM_1_SEG_0_4x_Msk           (0x3ful << LCD_MEM_1_SEG_0_4x_Pos)                /*!< LCD_T::MEM_1: SEG_0_4x Mask               */

#define LCD_MEM_1_SEG_1_4x_Pos           (8)                                               /*!< LCD_T::MEM_1: SEG_1_4x Position           */
#define LCD_MEM_1_SEG_1_4x_Msk           (0x7ful << LCD_MEM_1_SEG_1_4x_Pos)                /*!< LCD_T::MEM_1: SEG_1_4x Mask               */

#define LCD_MEM_1_SEG_2_4x_Pos           (16)                                              /*!< LCD_T::MEM_1: SEG_2_4x Position           */
#define LCD_MEM_1_SEG_2_4x_Msk           (0x3ful << LCD_MEM_1_SEG_2_4x_Pos)                /*!< LCD_T::MEM_1: SEG_2_4x Mask               */

#define LCD_MEM_1_SEG_3_4x_Pos           (24)                                              /*!< LCD_T::MEM_1: SEG_3_4x Position           */
#define LCD_MEM_1_SEG_3_4x_Msk           (0x3ful << LCD_MEM_1_SEG_3_4x_Pos)                /*!< LCD_T::MEM_1: SEG_3_4x Mask               */

#define LCD_MEM_2_SEG_0_4x_Pos           (0)                                               /*!< LCD_T::MEM_2: SEG_0_4x Position           */
#define LCD_MEM_2_SEG_0_4x_Msk           (0x3ful << LCD_MEM_2_SEG_0_4x_Pos)                /*!< LCD_T::MEM_2: SEG_0_4x Mask               */

#define LCD_MEM_2_SEG_1_4x_Pos           (8)                                               /*!< LCD_T::MEM_2: SEG_1_4x Position           */
#define LCD_MEM_2_SEG_1_4x_Msk           (0x7ful << LCD_MEM_2_SEG_1_4x_Pos)                /*!< LCD_T::MEM_2: SEG_1_4x Mask               */

#define LCD_MEM_2_SEG_2_4x_Pos           (16)                                              /*!< LCD_T::MEM_2: SEG_2_4x Position           */
#define LCD_MEM_2_SEG_2_4x_Msk           (0x3ful << LCD_MEM_2_SEG_2_4x_Pos)                /*!< LCD_T::MEM_2: SEG_2_4x Mask               */

#define LCD_MEM_2_SEG_3_4x_Pos           (24)                                              /*!< LCD_T::MEM_2: SEG_3_4x Position           */
#define LCD_MEM_2_SEG_3_4x_Msk           (0x3ful << LCD_MEM_2_SEG_3_4x_Pos)                /*!< LCD_T::MEM_2: SEG_3_4x Mask               */

#define LCD_MEM_3_SEG_0_4x_Pos           (0)                                               /*!< LCD_T::MEM_3: SEG_0_4x Position           */
#define LCD_MEM_3_SEG_0_4x_Msk           (0x3ful << LCD_MEM_3_SEG_0_4x_Pos)                /*!< LCD_T::MEM_3: SEG_0_4x Mask               */

#define LCD_MEM_3_SEG_1_4x_Pos           (8)                                               /*!< LCD_T::MEM_3: SEG_1_4x Position           */
#define LCD_MEM_3_SEG_1_4x_Msk           (0x7ful << LCD_MEM_3_SEG_1_4x_Pos)                /*!< LCD_T::MEM_3: SEG_1_4x Mask               */

#define LCD_MEM_3_SEG_2_4x_Pos           (16)                                              /*!< LCD_T::MEM_3: SEG_2_4x Position           */
#define LCD_MEM_3_SEG_2_4x_Msk           (0x3ful << LCD_MEM_3_SEG_2_4x_Pos)                /*!< LCD_T::MEM_3: SEG_2_4x Mask               */

#define LCD_MEM_3_SEG_3_4x_Pos           (24)                                              /*!< LCD_T::MEM_3: SEG_3_4x Position           */
#define LCD_MEM_3_SEG_3_4x_Msk           (0x3ful << LCD_MEM_3_SEG_3_4x_Pos)                /*!< LCD_T::MEM_3: SEG_3_4x Mask               */

#define LCD_MEM_4_SEG_0_4x_Pos           (0)                                               /*!< LCD_T::MEM_4: SEG_0_4x Position           */
#define LCD_MEM_4_SEG_0_4x_Msk           (0x3ful << LCD_MEM_4_SEG_0_4x_Pos)                /*!< LCD_T::MEM_4: SEG_0_4x Mask               */

#define LCD_MEM_4_SEG_1_4x_Pos           (8)                                               /*!< LCD_T::MEM_4: SEG_1_4x Position           */
#define LCD_MEM_4_SEG_1_4x_Msk           (0x7ful << LCD_MEM_4_SEG_1_4x_Pos)                /*!< LCD_T::MEM_4: SEG_1_4x Mask               */

#define LCD_MEM_4_SEG_2_4x_Pos           (16)                                              /*!< LCD_T::MEM_4: SEG_2_4x Position           */
#define LCD_MEM_4_SEG_2_4x_Msk           (0x3ful << LCD_MEM_4_SEG_2_4x_Pos)                /*!< LCD_T::MEM_4: SEG_2_4x Mask               */

#define LCD_MEM_4_SEG_3_4x_Pos           (24)                                              /*!< LCD_T::MEM_4: SEG_3_4x Position           */
#define LCD_MEM_4_SEG_3_4x_Msk           (0x3ful << LCD_MEM_4_SEG_3_4x_Pos)                /*!< LCD_T::MEM_4: SEG_3_4x Mask               */

#define LCD_MEM_5_SEG_0_4x_Pos           (0)                                               /*!< LCD_T::MEM_5: SEG_0_4x Position           */
#define LCD_MEM_5_SEG_0_4x_Msk           (0x3ful << LCD_MEM_5_SEG_0_4x_Pos)                /*!< LCD_T::MEM_5: SEG_0_4x Mask               */

#define LCD_MEM_5_SEG_1_4x_Pos           (8)                                               /*!< LCD_T::MEM_5: SEG_1_4x Position           */
#define LCD_MEM_5_SEG_1_4x_Msk           (0x7ful << LCD_MEM_5_SEG_1_4x_Pos)                /*!< LCD_T::MEM_5: SEG_1_4x Mask               */

#define LCD_MEM_5_SEG_2_4x_Pos           (16)                                              /*!< LCD_T::MEM_5: SEG_2_4x Position           */
#define LCD_MEM_5_SEG_2_4x_Msk           (0x3ful << LCD_MEM_5_SEG_2_4x_Pos)                /*!< LCD_T::MEM_5: SEG_2_4x Mask               */

#define LCD_MEM_5_SEG_3_4x_Pos           (24)                                              /*!< LCD_T::MEM_5: SEG_3_4x Position           */
#define LCD_MEM_5_SEG_3_4x_Msk           (0x3ful << LCD_MEM_5_SEG_3_4x_Pos)                /*!< LCD_T::MEM_5: SEG_3_4x Mask               */

#define LCD_MEM_6_SEG_0_4x_Pos           (0)                                               /*!< LCD_T::MEM_6: SEG_0_4x Position           */
#define LCD_MEM_6_SEG_0_4x_Msk           (0x3ful << LCD_MEM_6_SEG_0_4x_Pos)                /*!< LCD_T::MEM_6: SEG_0_4x Mask               */

#define LCD_MEM_6_SEG_1_4x_Pos           (8)                                               /*!< LCD_T::MEM_6: SEG_1_4x Position           */
#define LCD_MEM_6_SEG_1_4x_Msk           (0x7ful << LCD_MEM_6_SEG_1_4x_Pos)                /*!< LCD_T::MEM_6: SEG_1_4x Mask               */

#define LCD_MEM_6_SEG_2_4x_Pos           (16)                                              /*!< LCD_T::MEM_6: SEG_2_4x Position           */
#define LCD_MEM_6_SEG_2_4x_Msk           (0x3ful << LCD_MEM_6_SEG_2_4x_Pos)                /*!< LCD_T::MEM_6: SEG_2_4x Mask               */

#define LCD_MEM_6_SEG_3_4x_Pos           (24)                                              /*!< LCD_T::MEM_6: SEG_3_4x Position           */
#define LCD_MEM_6_SEG_3_4x_Msk           (0x3ful << LCD_MEM_6_SEG_3_4x_Pos)                /*!< LCD_T::MEM_6: SEG_3_4x Mask               */

#define LCD_MEM_7_SEG_0_4x_Pos           (0)                                               /*!< LCD_T::MEM_7: SEG_0_4x Position           */
#define LCD_MEM_7_SEG_0_4x_Msk           (0x3ful << LCD_MEM_7_SEG_0_4x_Pos)                /*!< LCD_T::MEM_7: SEG_0_4x Mask               */

#define LCD_MEM_7_SEG_1_4x_Pos           (8)                                               /*!< LCD_T::MEM_7: SEG_1_4x Position           */
#define LCD_MEM_7_SEG_1_4x_Msk           (0x7ful << LCD_MEM_7_SEG_1_4x_Pos)                /*!< LCD_T::MEM_7: SEG_1_4x Mask               */

#define LCD_MEM_7_SEG_2_4x_Pos           (16)                                              /*!< LCD_T::MEM_7: SEG_2_4x Position           */
#define LCD_MEM_7_SEG_2_4x_Msk           (0x3ful << LCD_MEM_7_SEG_2_4x_Pos)                /*!< LCD_T::MEM_7: SEG_2_4x Mask               */

#define LCD_MEM_7_SEG_3_4x_Pos           (24)                                              /*!< LCD_T::MEM_7: SEG_3_4x Position           */
#define LCD_MEM_7_SEG_3_4x_Msk           (0x3ful << LCD_MEM_7_SEG_3_4x_Pos)                /*!< LCD_T::MEM_7: SEG_3_4x Mask               */

#define LCD_MEM_8_SEG_0_4x_Pos           (0)                                               /*!< LCD_T::MEM_8: SEG_0_4x Position           */
#define LCD_MEM_8_SEG_0_4x_Msk           (0x3ful << LCD_MEM_8_SEG_0_4x_Pos)                /*!< LCD_T::MEM_8: SEG_0_4x Mask               */

#define LCD_MEM_8_SEG_1_4x_Pos           (8)                                               /*!< LCD_T::MEM_8: SEG_1_4x Position           */
#define LCD_MEM_8_SEG_1_4x_Msk           (0x7ful << LCD_MEM_8_SEG_1_4x_Pos)                /*!< LCD_T::MEM_8: SEG_1_4x Mask               */

#define LCD_MEM_8_SEG_2_4x_Pos           (16)                                              /*!< LCD_T::MEM_8: SEG_2_4x Position           */
#define LCD_MEM_8_SEG_2_4x_Msk           (0x3ful << LCD_MEM_8_SEG_2_4x_Pos)                /*!< LCD_T::MEM_8: SEG_2_4x Mask               */

#define LCD_MEM_8_SEG_3_4x_Pos           (24)                                              /*!< LCD_T::MEM_8: SEG_3_4x Position           */
#define LCD_MEM_8_SEG_3_4x_Msk           (0x3ful << LCD_MEM_8_SEG_3_4x_Pos)                /*!< LCD_T::MEM_8: SEG_3_4x Mask               */

#define LCD_MEM_9_SEG_0_4x_Pos           (0)                                               /*!< LCD_T::MEM_9: SEG_0_4x Position           */
#define LCD_MEM_9_SEG_0_4x_Msk           (0x3ful << LCD_MEM_9_SEG_0_4x_Pos)                /*!< LCD_T::MEM_9: SEG_0_4x Mask               */

#define LCD_MEM_9_SEG_1_4x_Pos           (8)                                               /*!< LCD_T::MEM_9: SEG_1_4x Position           */
#define LCD_MEM_9_SEG_1_4x_Msk           (0x7ful << LCD_MEM_9_SEG_1_4x_Pos)                /*!< LCD_T::MEM_9: SEG_1_4x Mask               */

#define LCD_MEM_9_SEG_2_4x_Pos           (16)                                              /*!< LCD_T::MEM_9: SEG_2_4x Position           */
#define LCD_MEM_9_SEG_2_4x_Msk           (0x3ful << LCD_MEM_9_SEG_2_4x_Pos)                /*!< LCD_T::MEM_9: SEG_2_4x Mask               */

#define LCD_MEM_9_SEG_3_4x_Pos           (24)                                              /*!< LCD_T::MEM_9: SEG_3_4x Position           */
#define LCD_MEM_9_SEG_3_4x_Msk           (0x3ful << LCD_MEM_9_SEG_3_4x_Pos)                /*!< LCD_T::MEM_9: SEG_3_4x Mask               */

#define LCD_FCR_FCEN_Pos                 (0)                                               /*!< LCD_T::FCR: FCEN Position                 */
#define LCD_FCR_FCEN_Msk                 (0x1ul << LCD_FCR_FCEN_Pos)                       /*!< LCD_T::FCR: FCEN Mask                     */

#define LCD_FCR_FCINTEN_Pos              (1)                                               /*!< LCD_T::FCR: FCINTEN Position              */
#define LCD_FCR_FCINTEN_Msk              (0x1ul << LCD_FCR_FCINTEN_Pos)                    /*!< LCD_T::FCR: FCINTEN Mask                  */

#define LCD_FCR_PRESCL_Pos               (2)                                               /*!< LCD_T::FCR: PRESCL Position               */
#define LCD_FCR_PRESCL_Msk               (0x3ul << LCD_FCR_PRESCL_Pos)                     /*!< LCD_T::FCR: PRESCL Mask                   */

#define LCD_FCR_FCV_Pos                  (4)                                               /*!< LCD_T::FCR: FCV Position                  */
#define LCD_FCR_FCV_Msk                  (0x3ful << LCD_FCR_FCV_Pos)                       /*!< LCD_T::FCR: FCV Mask                      */

#define LCD_FCSTS_FCSTS_Pos              (0)                                               /*!< LCD_T::FCSTS: FCSTS Position              */
#define LCD_FCSTS_FCSTS_Msk              (0x1ul << LCD_FCSTS_FCSTS_Pos)                    /*!< LCD_T::FCSTS: FCSTS Mask                  */

#define LCD_FCSTS_PDSTS_Pos              (1)                                               /*!< LCD_T::FCSTS: PDSTS Position              */
#define LCD_FCSTS_PDSTS_Msk              (0x1ul << LCD_FCSTS_PDSTS_Pos)                    /*!< LCD_T::FCSTS: PDSTS Mask                  */

/**@}*/ /* LCD_CONST */
/**@}*/ /* end of LCD register group */


/*---------------------- Peripheral Direct Memory Access Controller -------------------------*/
/**
    @addtogroup DMA Direct Memory Access Controller(DMA)
    Memory Mapped Structure for DMA Controller
@{ */


typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTL</font><br><p> <font size="2">
Offset: 0x00  DMA CRC Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>CRCCEN</td><td><div style="word-wrap: break-word;"><b>CRC Channel Enable</b><br>
Setting this bit to 1 enables CRC's operation.<br>
When operating in CRC DMA mode (TRIG_EN = 1), if user clear this bit, the DMA operation will be continuous until all CRC DMA operation done, and the TRIG_EN bit will asserted until all CRC DMA operation done.<br>
But in this case, the CRC_DMAISR [BLKD_IF] flag will inactive, user can read CRC result by reading CRC_CHECKSUM register when TRIG_EN = 0.<br>
When operating in CRC DMA mode (TRIG_EN = 1), if user want to stop the transfer immediately, user can write 1 to CRC_RST bit to stop the transmission.<br>
</div></td></tr><tr><td>
[1]</td><td>CRC_RST</td><td><div style="word-wrap: break-word;"><b>CRC Engine Reset</b><br>
0 = Writing 0 to this bit has no effect.<br>
1 = Writing 1 to this bit will reset the internal CRC state machine and internal buffer.<br>
The contents of control register will not be cleared.<br>
This bit will be auto cleared after few clock cycles.<br>
Note: When operating in CPU PIO mode, setting this bit will reload the initial seed value<br>
</div></td></tr><tr><td>
[23]</td><td>TRIG_EN</td><td><div style="word-wrap: break-word;"><b>Trigger Enable</b><br>
0 = No effect.<br>
1 = CRC DMA data read or write transfer Enabled.<br>
Note1: If this bit assert that indicates the CRC engine operation in CRC DMA mode, so don't filled any data in CRC_WDATA register.<br>
Note2: When CRC DMA transfer completed, this bit will be cleared automatically.<br>
Note3: If the bus error occurs, all CRC DMA transfer will be stopped.<br>
Software must reset all DMA channel, and then trigger again.<br>
</div></td></tr><tr><td>
[24]</td><td>WDATA_RVS</td><td><div style="word-wrap: break-word;"><b>Write Data Order Reverse</b><br>
0 = No bit order reverse for CRC write data in.<br>
1 = Bit order reverse for CRC write data in (per byre).<br>
Note: If the write data is 0xAABBCCDD, the bit order reverse for CRC write data in is 0x55DD33BB<br>
</div></td></tr><tr><td>
[25]</td><td>CHECKSUM_RVS</td><td><div style="word-wrap: break-word;"><b>Checksum Reverse</b><br>
0 = No bit order reverse for CRC checksum.<br>
1 = Bit order reverse for CRC checksum.<br>
Note: If the checksum data is 0XDD7B0F2E, the bit order reverse for CRC checksum is 0x74F0DEBB<br>
</div></td></tr><tr><td>
[26]</td><td>WDATA_COM</td><td><div style="word-wrap: break-word;"><b>Write Data Complement</b><br>
0 = No bit order reverse for CRC write data in.<br>
1 = 1's complement for CRC write data in.<br>
</div></td></tr><tr><td>
[27]</td><td>CHECKSUM_COM</td><td><div style="word-wrap: break-word;"><b>Checksum Complement</b><br>
0 = No bit order reverse for CRC checksum.<br>
1 = 1's complement for CRC checksum.<br>
</div></td></tr><tr><td>
[29:28]</td><td>CPU_WDLEN</td><td><div style="word-wrap: break-word;"><b>CPU Write Data Length</b><br>
When operating in CPU PIO mode (CRCCEN= 1, TRIG_EN = 0), this field indicates the write data length.<br>
00 = The data length is 8-bit mode<br>
01 = The data length is 16-bit mode<br>
10 = The data length is 32-bit mode<br>
11 = Reserved<br>
Note1: This field is only used for CPU PIO mode.<br>
Note2: When the data length is 8-bit mode, the valid data is CRC_WDATA [7:0], and if the data length is 16 bit mode, the valid data is CRC_WDATA [15:0].<br>
</div></td></tr><tr><td>
[31:30]</td><td>CRC_MODE</td><td><div style="word-wrap: break-word;"><b>CRC Polynomial Mode</b><br>
00 = CRC-CCITT Polynomial Mode<br>
01 = CRC-8 Polynomial Mode<br>
10 = CRC-16 Polynomial Mode<br>
11 = CRC-32 Polynomial Mode<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DMASAR</font><br><p> <font size="2">
Offset: 0x04  DMA CRC Source Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>CRC_DMASAR</td><td><div style="word-wrap: break-word;"><b>CRC DMA Transfer Source Address Register</b><br>
This field indicates a 32-bit source address of CRC DMA.<br>
Note : The source address must be word alignment<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DMASAR;
    uint32_t RESERVE0[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DMABCR</font><br><p> <font size="2">
Offset: 0x0C  DMA CRC Transfer Byte Count Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CRC_DMABCR</td><td><div style="word-wrap: break-word;"><b>CRC DMA Transfer Byte Count Register</b><br>
This field indicates a 16-bit transfer byte count number of CRC DMA<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DMABCR;
    uint32_t RESERVE1[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DMACSAR</font><br><p> <font size="2">
Offset: 0x14  DMA CRC Current Source Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>CRC_DMACSAR</td><td><div style="word-wrap: break-word;"><b>CRC DMA Current Source Address Register (Read Only)</b><br>
This field indicates the source address where the CRC DMA transfer is just occurring.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t DMACSAR;
    uint32_t RESERVE2[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DMACBCR</font><br><p> <font size="2">
Offset: 0x1C  DMA CRC Current Transfer Byte Count Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CRC_DMACBCR</td><td><div style="word-wrap: break-word;"><b>CRC DMA Current Byte Count Register (Read Only)</b><br>
This field indicates the current remained byte count of CRC_DMA.<br>
Note: CRC_RST will clear this register value.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t DMACBCR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DMAIER</font><br><p> <font size="2">
Offset: 0x20  DMA CRC Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TABORT_IE</td><td><div style="word-wrap: break-word;"><b>CRC DMA Read/Write Target Abort Interrupt Enable</b><br>
0 = Target abort interrupt generation Disabled during CRC DMA transfer.<br>
1 = Target abort interrupt generation Enabled during CRC DMA transfer.<br>
</div></td></tr><tr><td>
[1]</td><td>BLKD_IE</td><td><div style="word-wrap: break-word;"><b>CRC DMA Transfer Done Interrupt Enable</b><br>
0 = Interrupt generator Disabled during CRC DMA transfer done.<br>
1 = Interrupt generator Enabled during CRC DMA transfer done.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DMAIER;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DMAISR</font><br><p> <font size="2">
Offset: 0x24  DMA CRC Interrupt Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TABORT_IF</td><td><div style="word-wrap: break-word;"><b>CRC DMA Read/Write Target Abort Interrupt Flag</b><br>
0 = No bus ERROR response received.<br>
1 = Bus ERROR response received.<br>
Software can write 1 to clear this bit to zero<br>
Note: The CRC_DMAISR [TABORT_IF] indicate bus master received ERROR response or not.<br>
If bus master received ERROR response, it means that target abort is happened.<br>
DMA will stop transfer and respond this event to software then go to IDLE state.<br>
When target abort occurred, software must reset DMA, and then transfer those data again.<br>
</div></td></tr><tr><td>
[1]</td><td>BLKD_IF</td><td><div style="word-wrap: break-word;"><b>Block Transfer Done Interrupt Flag</b><br>
This bit indicates that CRC DMA has finished all transfer.<br>
0 = Not finished yet.<br>
1 = Done.<br>
Software can write 1 to clear this bit to zero<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DMAISR;
    uint32_t RESERVE3[22];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">WDATA</font><br><p> <font size="2">
Offset: 0x80  DMA CRC Write Data Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>CRC_WDATA</td><td><div style="word-wrap: break-word;"><b>CRC Write Data Register</b><br>
When operating in CPU PIO (CRC_CTL [CRCCEN] = 1, CRC_CTL [TRIG_EN] = 0) mode, software can write data to this field to perform CRC operation;.<br>
When operating in CRC DMA mode (CRC_CTL [CRCCEN] = 1, CRC_CTL [TRIG_EN] = 0), this field will be used for DMA internal buffer.<br>
Note1: When operating in CRC DMA mode, so don't filled any data in this field.<br>
Note2:The CRC_CTL [WDATA_COM] and CRC_CTL [WDATA_RVS] bit setting will affected this field; For example, if WDATA_RVS = 1, if the write data in CRC_WDATA register is 0xAABBCCDD, the read data from CRC_WDATA register will be 0x55DD33BB<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t WDATA;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SEED</font><br><p> <font size="2">
Offset: 0x84  DMA CRC Seed Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>CRC_SEED</td><td><div style="word-wrap: break-word;"><b>CRC Seed Register</b><br>
This field indicates the CRC seed value.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SEED;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CHECKSUM</font><br><p> <font size="2">
Offset: 0x88  DMA CRC Check Sum Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>CRC_CHECKSUM</td><td><div style="word-wrap: break-word;"><b>CRC Checksum Register</b><br>
This field indicates the CRC checksum<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CHECKSUM;

} DMA_CRC_T;


typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">GCRCSR</font><br><p> <font size="2">
Offset: 0x00  DMA Global Control and Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[8]</td><td>CLK0_EN</td><td><div style="word-wrap: break-word;"><b>DMA Controller Channel 0 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[9]</td><td>CLK1_EN</td><td><div style="word-wrap: break-word;"><b>DMA Controller Channel 1 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[10]</td><td>CLK2_EN</td><td><div style="word-wrap: break-word;"><b>DMA Controller Channel 2 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[11]</td><td>CLK3_EN</td><td><div style="word-wrap: break-word;"><b>DMA Controller Channel 3 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[12]</td><td>CLK4_EN</td><td><div style="word-wrap: break-word;"><b>DMA Controller Channel 4 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[13]</td><td>CLK5_EN</td><td><div style="word-wrap: break-word;"><b>DMA Controller Channel 5 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[14]</td><td>CLK6_EN</td><td><div style="word-wrap: break-word;"><b>DMA Controller Channel 6 Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[24]</td><td>CRC_CLK_EN</td><td><div style="word-wrap: break-word;"><b>CRC Controller Clock Enable Control</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t GCRCSR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DSSR0</font><br><p> <font size="2">
Offset: 0x04  DMA Service Selection Control Register 0</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[12:8]</td><td>CH1_SEL</td><td><div style="word-wrap: break-word;"><b>Channel 1 Selection</b><br>
This filed defines which peripheral is connected to PDMA channel 1.<br>
Software can configure the peripheral by setting CH1_SEL.<br>
00000 = Connect to SPI0_TX.<br>
00001 = Connect to SPI1_TX.<br>
00010 = Connect to UART0_TX.<br>
00011 = Connect to UART1_TX.<br>
00100 = Connect to USB_TX.<br>
00101 = Connect to I2S_TX.<br>
00110 = Connect to DAC0_TX.<br>
00111 = Connect to DAC1_TX.<br>
01000 = Connect to SPI2_TX.<br>
01001 = Connect to TMR0.<br>
01010 = Connect to TMR1.<br>
01011 = Connect to TMR2.<br>
01100 = Connect to TMR3.<br>
10000 = Connect to SPI0_RX.<br>
10001 = Connect to SPI1_RX.<br>
10010 = Connect to UART0_RX.<br>
10011 = Connect to UART1_RX.<br>
10100 = Connect to USB_RX.<br>
10101 = Connect to I2S_RX.<br>
10110 = Connect to ADC.<br>
11000 = Connect to SPI2_RX.<br>
11001 = Connect to PWM0_CH0.<br>
11010 = Connect to PWM0_CH2.<br>
11011 = Connect to PWM1_CH0.<br>
11100 = Connect to PWM1_CH2.<br>
</div></td></tr><tr><td>
[20:16]</td><td>CH2_SEL</td><td><div style="word-wrap: break-word;"><b>Channel 2 Selection</b><br>
This filed defines which peripheral is connected to PDMA channel 2.<br>
Software can configure the peripheral setting by CH2_SEL.<br>
The channel configuration is the same as CH1_SEL field.<br>
Please refer to the explanation of CH1_SEL.<br>
</div></td></tr><tr><td>
[28:24]</td><td>CH3_SEL</td><td><div style="word-wrap: break-word;"><b>Channel 3 Selection</b><br>
This filed defines which peripheral is connected to PDMA channel 3.<br>
Software can configure the peripheral setting by CH3_SEL.<br>
The channel configuration is the same as CH1_SEL field.<br>
Please refer to the explanation of CH1_SEL.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DSSR0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DSSR1</font><br><p> <font size="2">
Offset: 0x08  DMA Service Selection Control Register 1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[4:0]</td><td>CH4_SEL</td><td><div style="word-wrap: break-word;"><b>Channel 4 Selection</b><br>
This filed defines which peripheral is connected to PDMA channel 4.<br>
Software can configure the peripheral by setting CH4_SEL.<br>
The channel configuration is the same as CH1_SEL field.<br>
Please refer to the explanation of CH1_SEL.<br>
</div></td></tr><tr><td>
[12:8]</td><td>CH5_SEL</td><td><div style="word-wrap: break-word;"><b>Channel 5 Selection</b><br>
This filed defines which peripheral is connected to PDMA channel 5.<br>
Software can configure the peripheral setting by CH5_SEL.<br>
The channel configuration is the same as CH1_SEL field.<br>
Please refer to the explanation of CH1_SEL.<br>
</div></td></tr><tr><td>
[20:16]</td><td>CH6_SEL</td><td><div style="word-wrap: break-word;"><b>Channel 6 Selection</b><br>
This filed defines which peripheral is connected to PDMA channel 6.<br>
Software can configure the peripheral setting by CH6_SEL.<br>
The channel configuration is the same as CH1_SEL field.<br>
Please refer to the explanation of CH1_SEL.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DSSR1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">GCRISR</font><br><p> <font size="2">
Offset: 0x0C  DMA Global Interrupt Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>INTR0</td><td><div style="word-wrap: break-word;"><b>Interrupt Pin Status Of Channel 0 (Read Only)</b><br>
This bit is the Interrupt pin status of DMA channel0.<br>
Note: This bit is read only<br>
</div></td></tr><tr><td>
[1]</td><td>INTR1</td><td><div style="word-wrap: break-word;"><b>Interrupt Pin Status Of Channel 1 (Read Only)</b><br>
This bit is the Interrupt pin status of DMA channel1.<br>
Note: This bit is read only<br>
</div></td></tr><tr><td>
[2]</td><td>INTR2</td><td><div style="word-wrap: break-word;"><b>Interrupt Pin Status Of Channel 2 (Read Only)</b><br>
This bit is the Interrupt pin status of DMA channel2.<br>
Note: This bit is read only<br>
</div></td></tr><tr><td>
[3]</td><td>INTR3</td><td><div style="word-wrap: break-word;"><b>Interrupt Pin Status Of Channel 3 (Read Only)</b><br>
This bit is the Interrupt pin status of DMA channel3.<br>
Note: This bit is read only<br>
</div></td></tr><tr><td>
[4]</td><td>INTR4</td><td><div style="word-wrap: break-word;"><b>Interrupt Pin Status Of Channel 4 (Read Only)</b><br>
This bit is the Interrupt pin status of DMA channel4.<br>
Note: This bit is read only<br>
</div></td></tr><tr><td>
[5]</td><td>INTR5</td><td><div style="word-wrap: break-word;"><b>Interrupt Pin Status Of Channel 5 (Read Only)</b><br>
This bit is the Interrupt pin status of DMA channel4.<br>
Note: This bit is read only<br>
</div></td></tr><tr><td>
[6]</td><td>INTR6</td><td><div style="word-wrap: break-word;"><b>Interrupt Pin Status Of Channel 6 (Read Only)</b><br>
This bit is the Interrupt pin status of DMA channel4.<br>
Note: This bit is read only<br>
</div></td></tr><tr><td>
[16]</td><td>CRC_INTR</td><td><div style="word-wrap: break-word;"><b>Interrupt Pin Status Of CRC Controller</b><br>
This bit is the Interrupt status of CRC controller<br>
Note: This bit is read only<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t GCRISR;

} DMA_GCR_T;


typedef struct {
/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CSR</font><br><p> <font size="2">
Offset: 0x00  PDMA Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>PDMACEN</td><td><div style="word-wrap: break-word;"><b>PDMA Channel Enable</b><br>
Setting this bit to "1" enables PDMA's operation.<br>
If this bit is cleared, PDMA will ignore all PDMA request and force Bus Master into IDLE state.<br>
Note: SW_RST will clear this bit.<br>
</div></td></tr><tr><td>
[1]</td><td>SW_RST</td><td><div style="word-wrap: break-word;"><b>Software Engine Reset</b><br>
0 = No effect.<br>
1 = Reset the internal state machine and pointers.<br>
The contents of control register will not be cleared.<br>
This bit will be auto cleared after few clock cycles.<br>
</div></td></tr><tr><td>
[3:2]</td><td>MODE_SEL</td><td><div style="word-wrap: break-word;"><b>PDMA Mode Select</b><br>
00 = Memory to Memory mode (Memory-to-Memory).<br>
01 = IP to Memory mode (APB-to-Memory)<br>
10 = Memory to IP mode (Memory-to-APB).<br>
11 = Reserved.<br>
</div></td></tr><tr><td>
[5:4]</td><td>SAD_SEL</td><td><div style="word-wrap: break-word;"><b>Transfer Source Address Direction Selection</b><br>
00 = Transfer Source address is incremented successively.<br>
01 = Reserved.<br>
10 = Transfer Source address is fixed (This feature can be used when data where transferred from a single source to multiple destinations).<br>
11 = Transfer Source address is wrap around (When the PDMA_CBCR is equal to zero, the PDMA_CSAR and PDMA_CBCR register will be updated by PDMA_SAR and PDMA_BCR automatically.<br>
PDMA will start another transfer without software trigger until PDMA_EN disabled.<br>
When the PDMA_EN is disabled, the PDMA will complete the active transfer but the remained data which in the PDMA_BUF will not transfer to destination address).<br>
</div></td></tr><tr><td>
[7:6]</td><td>DAD_SEL</td><td><div style="word-wrap: break-word;"><b>Transfer Destination Address Direction Selection</b><br>
00 = Transfer Destination address is incremented successively<br>
01 = Reserved.<br>
10 = Transfer Destination address is fixed (This feature can be used when data where transferred from multiple sources to a single destination)<br>
11 = Transfer Destination address is wrapped around (When the PDMA_CBCR is equal to zero, the PDMA_CDAR and PDMA_CBCR register will be updated by PDMA_DAR and PDMA_BCR automatically.<br>
PDMA will start another transfer without software trigger until PDMA_EN disabled.<br>
When the PDMA_EN is disabled, the PDMA will complete the active transfer but the remained data which in the PDMA_BUF will not transfer to destination address).<br>
</div></td></tr><tr><td>
[12]</td><td>TO_EN</td><td><div style="word-wrap: break-word;"><b>Time-Out Enable</b><br>
This bit will enable PDMA internal counter. While this counter counts to zero, the TO_IS will be set.<br>
0 = PDMA internal counter Disabled.<br>
1 = PDMA internal counter Enabled.<br>
</div></td></tr><tr><td>
[20:19]</td><td>APB_TWS</td><td><div style="word-wrap: break-word;"><b>Peripheral Transfer Width Selection</b><br>
00 = One word (32 bits) is transferred for every PDMA operation.<br>
01 = One byte (8 bits) is transferred for every PDMA operation.<br>
10 = One half-word (16 bits) is transferred for every PDMA operation.<br>
11 = Reserved.<br>
Note: This field is meaningful only when MODE_SEL is IP to Memory mode (APB-to-Memory) or Memory to IP mode (Memory-to-APB).<br>
</div></td></tr><tr><td>
[23]</td><td>TRIG_EN</td><td><div style="word-wrap: break-word;"><b>TRIG_EN</b><br>
0 = No effect.<br>
1 = PDMA data read or write transfer Enabled.<br>
Note1: When PDMA transfer completed, this bit will be cleared automatically.<br>
Note2: If the bus error occurs, all PDMA transfer will be stopped.<br>
Software must reset all PDMA channel, and then trig again.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CSR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SAR</font><br><p> <font size="2">
Offset: 0x04  PDMA Source Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>PDMA_SAR</td><td><div style="word-wrap: break-word;"><b>PDMA Transfer Source Address Register</b><br>
This field indicates a 32-bit source address of PDMA.<br>
Note: The source address must be word alignment.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SAR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DAR</font><br><p> <font size="2">
Offset: 0x08  PDMA Destination Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>PDMA_DAR</td><td><div style="word-wrap: break-word;"><b>PDMA Transfer Destination Address Register</b><br>
This field indicates a 32-bit destination address of PDMA.<br>
Note : The destination address must be word alignment<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DAR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">BCR</font><br><p> <font size="2">
Offset: 0x0C  PDMA Transfer Byte Count Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>PDMA_BCR</td><td><div style="word-wrap: break-word;"><b>PDMA Transfer Byte Count Register</b><br>
This field indicates a 16-bit transfer byte count of PDMA.<br>
Note: In Memory-to-memory (PDMA_CSR [MODE_SEL] = 00) mode, the transfer byte count must be word alignment.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t BCR;
    uint32_t RESERVE0[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CSAR</font><br><p> <font size="2">
Offset: 0x14  PDMA Current Source Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>PDMA_CSAR</td><td><div style="word-wrap: break-word;"><b>PDMA Current Source Address Register (Read Only)</b><br>
This field indicates the source address where the PDMA transfer is just occurring.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CSAR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CDAR</font><br><p> <font size="2">
Offset: 0x18  PDMA Current Destination Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>PDMA_CDAR</td><td><div style="word-wrap: break-word;"><b>PDMA Current Destination Address Register (Read Only)</b><br>
This field indicates the destination address where the PDMA transfer is just occurring.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CDAR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CBCR</font><br><p> <font size="2">
Offset: 0x1C  PDMA Current Transfer Byte Count Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[23:0]</td><td>PDMA_CBCR</td><td><div style="word-wrap: break-word;"><b>PDMA Current Byte Count Register (Read Only)</b><br>
This field indicates the current remained byte count of PDMA.<br>
Note: These fields will be changed when PDMA finish data transfer (data transfer to destination address),<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CBCR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IER</font><br><p> <font size="2">
Offset: 0x20  PDMA Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TABORT_IE</td><td><div style="word-wrap: break-word;"><b>PDMA Read/Write Target Abort Interrupt Enable</b><br>
0 = Target abort interrupt generation Disabled during PDMA transfer.<br>
1 = Target abort interrupt generation Enabled during PDMA transfer.<br>
</div></td></tr><tr><td>
[1]</td><td>TD_IE</td><td><div style="word-wrap: break-word;"><b>PDMA Transfer Done Interrupt Enable</b><br>
0 = Interrupt generator Disabled when PDMA transfer is done.<br>
1 = Interrupt generator Enabled when PDMA transfer is done.<br>
</div></td></tr><tr><td>
[5:2]</td><td>WRA_BCR_IE</td><td><div style="word-wrap: break-word;"><b>Wrap Around Byte Count Interrupt Enable</b><br>
0001 = Interrupt enable of PDMA_CBCR equals 0<br>
0100 = Interrupt enable of PDMA_CBCR equals 1/2 PDMA_BCR.<br>
</div></td></tr><tr><td>
[6]</td><td>TO_IE</td><td><div style="word-wrap: break-word;"><b>Time-Out Interrupt Enable</b><br>
0 = Time-out interrupt Disabled.<br>
1 = Time-out interrupt Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IER;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISR</font><br><p> <font size="2">
Offset: 0x24  PDMA Interrupt Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TABORT_IS</td><td><div style="word-wrap: break-word;"><b>PDMA Read/Write Target Abort Interrupt Status Flag</b><br>
0 = No bus ERROR response received.<br>
1 = Bus ERROR response received.<br>
Note1: This bit is cleared by writing "1" to itself.<br>
Note2: The PDMA_ISR [TABORT_IF] indicate bus master received ERROR response or not, if bus master received occur it means that target abort is happened.<br>
PDMA controller will stop transfer and respond this event to software then go to IDLE state.<br>
When target abort occurred, software must reset PDMA controller, and then transfer those data again.<br>
</div></td></tr><tr><td>
[1]</td><td>TD_IS</td><td><div style="word-wrap: break-word;"><b>Transfer Done Interrupt Status Flag</b><br>
This bit indicates that PDMA has finished all transfer.<br>
0 = Not finished yet.<br>
1 = Done.<br>
Note: This bit is cleared by writing "1" to itself.<br>
</div></td></tr><tr><td>
[5:2]</td><td>WRA_BCR_IS</td><td><div style="word-wrap: break-word;"><b>Wrap Around Transfer Byte Count Interrupt Status Flag</b><br>
WAR_)CR_IS [0] (xxx1) = PDMA_CBCR equal 0 flag.<br>
WAR_BCR_IS [2] (x1xx) = PDMA_CBCR equal 1/2 PDMA_BCR flag.<br>
Note: Each bit is cleared by writing "1" to itself.<br>
This field is only valid in wrap around mode.<br>
(PDMA_CSR[DAD_SEL] =11 or PDMA_CSR[SAD_SEL] =11).<br>
</div></td></tr><tr><td>
[6]</td><td>TO_IS</td><td><div style="word-wrap: break-word;"><b>Time-Out Interrupt Status Flag</b><br>
This flag indicated that PDMA has waited peripheral request for a period defined by PDMA_TCR.<br>
0 = No time-out flag.<br>
1 = Time-out flag.<br>
Note: This bit is cleared by writing "1" to itself.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TCR</font><br><p> <font size="2">
Offset: 0x28  PDMA Timer Counter Setting Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>PDMA_TCR</td><td><div style="word-wrap: break-word;"><b>PDMA Timer Count Setting Register</b><br>
Each PDMA channel contains an internal counter.<br>
The internal counter loads the value of PDAM_TCR and starts counting down when setting PDMA_CSRx [TO_EN] register.<br>
PDMA will request interrupt when this internal counter reaches zero and PDMA_IERx[TO_IE] is high.<br>
This internal counter will reload and start counting when completing each peripheral request service.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TCR;

} PDMA_T;



typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CSR</font><br><p> <font size="2">
Offset: 0x00  VDMA Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>VDMACEN</td><td><div style="word-wrap: break-word;"><b>VDMA Channel Enable</b><br>
Setting this bit to "1" enables VDMA's operation.<br>
If this bit is cleared, VDMA will ignore all VDMA request and force Bus Master into IDLE state.<br>
Note: SW_RST will clear this bit.<br>
</div></td></tr><tr><td>
[1]</td><td>SW_RST</td><td><div style="word-wrap: break-word;"><b>Software Engine Reset</b><br>
0 = No effect.<br>
1 = Reset the internal state machine and pointers.<br>
The contents of control register will not be cleared.<br>
This bit will be auto cleared after few clock cycles.<br>
</div></td></tr><tr><td>
[10]</td><td>STRIDE_EN</td><td><div style="word-wrap: break-word;"><b>Stride Mode Enable</b><br>
0 = Stride transfer mode Disabled.<br>
1 = Stride transfer mode Enabled.<br>
</div></td></tr><tr><td>
[11]</td><td>DIR_SEL</td><td><div style="word-wrap: break-word;"><b>Transfer Source/Destination Address Direction Select</b><br>
0 = Transfer address is incremented successively.<br>
1 = Transfer address is decremented successively.<br>
</div></td></tr><tr><td>
[23]</td><td>TRIG_EN</td><td><div style="word-wrap: break-word;"><b>TRIG_EN</b><br>
0 = No effect.<br>
1 = VDMA data read or write transfer Enabled.<br>
Note1: When VDMA transfer is completed, this bit will be cleared automatically.<br>
Note2: If the bus error occurs, all VDMA transfer will be stopped.<br>
Software must reset all VDMA channel, and then trig again.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CSR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SAR</font><br><p> <font size="2">
Offset: 0x04  VDMA Source Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>VDMA_SAR</td><td><div style="word-wrap: break-word;"><b>VDMA Transfer Source Address Register</b><br>
This field indicates a 32-bit source address of VDMA.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SAR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DAR</font><br><p> <font size="2">
Offset: 0x08  VDMA Destination Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>VDMA_DAR</td><td><div style="word-wrap: break-word;"><b>VDMA Transfer Destination Address Register</b><br>
This field indicates a 32-bit destination address of VDMA.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DAR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">BCR</font><br><p> <font size="2">
Offset: 0x0C  VDMA Transfer Byte Count Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>VDMA_BCR</td><td><div style="word-wrap: break-word;"><b>VDMA Transfer Byte Count Register</b><br>
This field indicates a 16-bit transfer byte count of VDMA.<br>
Note: In Stride Enable mode (VDMA_CSR [10] = "0"]), the transfer byte count (VDMA_BCR) must be an integer multiple of STBC (VDMA_SASOCR [31:16]).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t BCR;
    uint32_t RESERVE0[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CSAR</font><br><p> <font size="2">
Offset: 0x14  VDMA Current Source Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>VDMA_CSAR</td><td><div style="word-wrap: break-word;"><b>VDMA Current Source Address Register (Read Only)</b><br>
This field indicates the source address where the VDMA transfer is just occurring.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CSAR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CDAR</font><br><p> <font size="2">
Offset: 0x18  VDMA Current Destination Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>VDMA_CDAR</td><td><div style="word-wrap: break-word;"><b>VDMA Current Destination Address Register (Read Only)</b><br>
This field indicates the destination address where the VDMA transfer is just occurring.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CDAR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CBCR</font><br><p> <font size="2">
Offset: 0x1C  VDMA Current Transfer Byte Count Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>VDMA_CBCR</td><td><div style="word-wrap: break-word;"><b>VDMA Current Byte Count Register (Read Only)</b><br>
This field indicates the current remained byte count of VDMA.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CBCR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IER</font><br><p> <font size="2">
Offset: 0x20  VDMA Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TABORT_IE</td><td><div style="word-wrap: break-word;"><b>VDMA Read/Write Target Abort Interrupt Enable</b><br>
0 = Disabled target abort interrupt generation during VDMA transfer.<br>
1 = Enabled target abort interrupt generation during VDMA transfer.<br>
</div></td></tr><tr><td>
[1]</td><td>TD_IE</td><td><div style="word-wrap: break-word;"><b>VDMA Transfer Done Interrupt Enable</b><br>
0 = Disabled interrupt generator during VDMA transfer done.<br>
1 = Enabled interrupt generator during VDMA transfer done.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IER;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISR</font><br><p> <font size="2">
Offset: 0x24  VDMA Interrupt Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TABORT_IS</td><td><div style="word-wrap: break-word;"><b>VDMA Read/Write Target Abort Interrupt Status Flag</b><br>
0 = No bus ERROR response received.<br>
1 = Bus ERROR response received.<br>
Note1: This bit is cleared by writing "1" to itself.<br>
Note2: The VDMA_ISR [TABORT_IF] indicate bus master received ERROR response or not, if bus master received occur it means that target abort is happened.<br>
VDMA controller will stop transfer and respond this event to software then go to IDLE state.<br>
When target abort occurred, software must reset VDMA controller, and then transfer those data again.<br>
</div></td></tr><tr><td>
[1]</td><td>TD_IS</td><td><div style="word-wrap: break-word;"><b>Transfer Done Interrupt Status Flag</b><br>
This bit indicates that VDMA has finished all transfer.<br>
0 = Not finished yet.<br>
1 = Done.<br>
Note: This bit is cleared by writing "1" to itself.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISR;
    uint32_t RESERVE1[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SASOCR</font><br><p> <font size="2">
Offset: 0x2C  VDMA Source Address Stride Offset Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>SASTOBL</td><td><div style="word-wrap: break-word;"><b>VDMA Source Address Stride Offset Byte Length</b><br>
The 16-bit register defines the source address stride transfer offset count of each row.<br>
</div></td></tr><tr><td>
[31:16]</td><td>STBC</td><td><div style="word-wrap: break-word;"><b>VDMA Stride Transfer Byte Count</b><br>
The 16-bit register defines the stride transfer byte count of each row.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SASOCR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DASOCR</font><br><p> <font size="2">
Offset: 0x30  VDMA Destination Address Stride Offset Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>DASTOBL</td><td><div style="word-wrap: break-word;"><b>VDMA Destination Address Stride Offset Byte Length</b><br>
The 16-bit register defines the destination address stride transfer offset count of each row.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DASOCR;

} VDMA_T;


/**
    @addtogroup DMA_CRC_CONST DMA_CRC Bit Field Definition
    Constant Definitions for DMA_CRC Controller
@{ */

#define DMA_CRC_CTL_CRCCEN_Pos           (0)                                               /*!< DMA_CRC_T::CTL: CRCCEN Position           */
#define DMA_CRC_CTL_CRCCEN_Msk           (0x1ul << DMA_CRC_CTL_CRCCEN_Pos)                 /*!< DMA_CRC_T::CTL: CRCCEN Mask               */

#define DMA_CRC_CTL_CRC_RST_Pos          (1)                                               /*!< DMA_CRC_T::CTL: CRC_RST Position          */
#define DMA_CRC_CTL_CRC_RST_Msk          (0x1ul << DMA_CRC_CTL_CRC_RST_Pos)                /*!< DMA_CRC_T::CTL: CRC_RST Mask              */

#define DMA_CRC_CTL_TRIG_EN_Pos          (23)                                              /*!< DMA_CRC_T::CTL: TRIG_EN Position          */
#define DMA_CRC_CTL_TRIG_EN_Msk          (0x1ul << DMA_CRC_CTL_TRIG_EN_Pos)                /*!< DMA_CRC_T::CTL: TRIG_EN Mask              */

#define DMA_CRC_CTL_WDATA_RVS_Pos        (24)                                              /*!< DMA_CRC_T::CTL: WDATA_RVS Position        */
#define DMA_CRC_CTL_WDATA_RVS_Msk        (0x1ul << DMA_CRC_CTL_WDATA_RVS_Pos)              /*!< DMA_CRC_T::CTL: WDATA_RVS Mask            */

#define DMA_CRC_CTL_CHECKSUM_RVS_Pos     (25)                                              /*!< DMA_CRC_T::CTL: CHECKSUM_RVS Position     */
#define DMA_CRC_CTL_CHECKSUM_RVS_Msk     (0x1ul << DMA_CRC_CTL_CHECKSUM_RVS_Pos)           /*!< DMA_CRC_T::CTL: CHECKSUM_RVS Mask         */

#define DMA_CRC_CTL_WDATA_COM_Pos        (26)                                              /*!< DMA_CRC_T::CTL: WDATA_COM Position        */
#define DMA_CRC_CTL_WDATA_COM_Msk        (0x1ul << DMA_CRC_CTL_WDATA_COM_Pos)              /*!< DMA_CRC_T::CTL: WDATA_COM Mask            */

#define DMA_CRC_CTL_CHECKSUM_COM_Pos     (27)                                              /*!< DMA_CRC_T::CTL: CHECKSUM_COM Position     */
#define DMA_CRC_CTL_CHECKSUM_COM_Msk     (0x1ul << DMA_CRC_CTL_CHECKSUM_COM_Pos)           /*!< DMA_CRC_T::CTL: CHECKSUM_COM Mask         */

#define DMA_CRC_CTL_CPU_WDLEN_Pos        (28)                                              /*!< DMA_CRC_T::CTL: CPU_WDLEN Position        */
#define DMA_CRC_CTL_CPU_WDLEN_Msk        (0x3ul << DMA_CRC_CTL_CPU_WDLEN_Pos)              /*!< DMA_CRC_T::CTL: CPU_WDLEN Mask            */

#define DMA_CRC_CTL_CRC_MODE_Pos         (30)                                              /*!< DMA_CRC_T::CTL: CRC_MODE Position         */
#define DMA_CRC_CTL_CRC_MODE_Msk         (0x3ul << DMA_CRC_CTL_CRC_MODE_Pos)               /*!< DMA_CRC_T::CTL: CRC_MODE Mask             */

#define DMA_CRC_DMASAR_CRC_DMASAR_Pos    (0)                                               /*!< DMA_CRC_T::DMASAR: CRC_DMASAR Position    */
#define DMA_CRC_DMASAR_CRC_DMASAR_Msk    (0xfffffffful << DMA_CRC_DMASAR_CRC_DMASAR_Pos)   /*!< DMA_CRC_T::DMASAR: CRC_DMASAR Mask        */

#define DMA_CRC_DMABCR_CRC_DMABCR_Pos    (0)                                               /*!< DMA_CRC_T::DMABCR: CRC_DMABCR Position    */
#define DMA_CRC_DMABCR_CRC_DMABCR_Msk    (0xfffful << DMA_CRC_DMABCR_CRC_DMABCR_Pos)       /*!< DMA_CRC_T::DMABCR: CRC_DMABCR Mask        */

#define DMA_CRC_DMACSAR_CRC_DMACSAR_Pos  (0)                                               /*!< DMA_CRC_T::DMACSAR: CRC_DMACSAR Position  */
#define DMA_CRC_DMACSAR_CRC_DMACSAR_Msk  (0xfffffffful << DMA_CRC_DMACSAR_CRC_DMACSAR_Pos) /*!< DMA_CRC_T::DMACSAR: CRC_DMACSAR Mask      */

#define DMA_CRC_DMACBCR_CRC_DMACBCR_Pos  (0)                                               /*!< DMA_CRC_T::DMACBCR: CRC_DMACBCR Position  */
#define DMA_CRC_DMACBCR_CRC_DMACBCR_Msk  (0xfffful << DMA_CRC_DMACBCR_CRC_DMACBCR_Pos)     /*!< DMA_CRC_T::DMACBCR: CRC_DMACBCR Mask      */

#define DMA_CRC_DMAIER_TABORT_IE_Pos     (0)                                               /*!< DMA_CRC_T::DMAIER: TABORT_IE Position     */
#define DMA_CRC_DMAIER_TABORT_IE_Msk     (0x1ul << DMA_CRC_DMAIER_TABORT_IE_Pos)           /*!< DMA_CRC_T::DMAIER: TABORT_IE Mask         */

#define DMA_CRC_DMAIER_BLKD_IE_Pos       (1)                                               /*!< DMA_CRC_T::DMAIER: BLKD_IE Position       */
#define DMA_CRC_DMAIER_BLKD_IE_Msk       (0x1ul << DMA_CRC_DMAIER_BLKD_IE_Pos)             /*!< DMA_CRC_T::DMAIER: BLKD_IE Mask           */

#define DMA_CRC_DMAISR_TABORT_IF_Pos     (0)                                               /*!< DMA_CRC_T::DMAISR: TABORT_IF Position     */
#define DMA_CRC_DMAISR_TABORT_IF_Msk     (0x1ul << DMA_CRC_DMAISR_TABORT_IF_Pos)           /*!< DMA_CRC_T::DMAISR: TABORT_IF Mask         */

#define DMA_CRC_DMAISR_BLKD_IF_Pos       (1)                                               /*!< DMA_CRC_T::DMAISR: BLKD_IF Position       */
#define DMA_CRC_DMAISR_BLKD_IF_Msk       (0x1ul << DMA_CRC_DMAISR_BLKD_IF_Pos)             /*!< DMA_CRC_T::DMAISR: BLKD_IF Mask           */

#define DMA_CRC_WDATA_CRC_WDATA_Pos      (0)                                               /*!< DMA_CRC_T::WDATA: CRC_WDATA Position      */
#define DMA_CRC_WDATA_CRC_WDATA_Msk      (0xfffffffful << DMA_CRC_WDATA_CRC_WDATA_Pos)     /*!< DMA_CRC_T::WDATA: CRC_WDATA Mask          */

#define DMA_CRC_SEED_CRC_SEED_Pos        (0)                                               /*!< DMA_CRC_T::SEED: CRC_SEED Position        */
#define DMA_CRC_SEED_CRC_SEED_Msk        (0xfffffffful << DMA_CRC_SEED_CRC_SEED_Pos)       /*!< DMA_CRC_T::SEED: CRC_SEED Mask            */

#define DMA_CRC_CHECKSUM_CRC_CHECKSUM_Pos (0)                                              /*!< DMA_CRC_T::CHECKSUM: CRC_CHECKSUM Position*/
#define DMA_CRC_CHECKSUM_CRC_CHECKSUM_Msk (0xfffffffful << DMA_CRC_CHECKSUM_CRC_CHECKSUM_Pos) /*!< DMA_CRC_T::CHECKSUM: CRC_CHECKSUM Mask    */

/**@}*/ /* DMA_CRC_CONST */


/**
    @addtogroup DMA_GCR_CONST DMA_GCR Bit Field Definition
    Constant Definitions for DMA_GCR Controller
@{ */

#define DMA_GCR_GCRCSR_CLK0_EN_Pos       (8)                                               /*!< DMA_GCR_T::GCRCSR: CLK0_EN Position       */
#define DMA_GCR_GCRCSR_CLK0_EN_Msk       (0x1ul << DMA_GCR_GCRCSR_CLK0_EN_Pos)             /*!< DMA_GCR_T::GCRCSR: CLK0_EN Mask           */

#define DMA_GCR_GCRCSR_CLK1_EN_Pos       (9)                                               /*!< DMA_GCR_T::GCRCSR: CLK1_EN Position       */
#define DMA_GCR_GCRCSR_CLK1_EN_Msk       (0x1ul << DMA_GCR_GCRCSR_CLK1_EN_Pos)             /*!< DMA_GCR_T::GCRCSR: CLK1_EN Mask           */

#define DMA_GCR_GCRCSR_CLK2_EN_Pos       (10)                                              /*!< DMA_GCR_T::GCRCSR: CLK2_EN Position       */
#define DMA_GCR_GCRCSR_CLK2_EN_Msk       (0x1ul << DMA_GCR_GCRCSR_CLK2_EN_Pos)             /*!< DMA_GCR_T::GCRCSR: CLK2_EN Mask           */

#define DMA_GCR_GCRCSR_CLK3_EN_Pos       (11)                                              /*!< DMA_GCR_T::GCRCSR: CLK3_EN Position       */
#define DMA_GCR_GCRCSR_CLK3_EN_Msk       (0x1ul << DMA_GCR_GCRCSR_CLK3_EN_Pos)             /*!< DMA_GCR_T::GCRCSR: CLK3_EN Mask           */

#define DMA_GCR_GCRCSR_CLK4_EN_Pos       (12)                                              /*!< DMA_GCR_T::GCRCSR: CLK4_EN Position       */
#define DMA_GCR_GCRCSR_CLK4_EN_Msk       (0x1ul << DMA_GCR_GCRCSR_CLK4_EN_Pos)             /*!< DMA_GCR_T::GCRCSR: CLK4_EN Mask           */

#define DMA_GCR_GCRCSR_CLK5_EN_Pos       (13)                                              /*!< DMA_GCR_T::GCRCSR: CLK5_EN Position       */
#define DMA_GCR_GCRCSR_CLK5_EN_Msk       (0x1ul << DMA_GCR_GCRCSR_CLK5_EN_Pos)             /*!< DMA_GCR_T::GCRCSR: CLK5_EN Mask           */

#define DMA_GCR_GCRCSR_CLK6_EN_Pos       (14)                                              /*!< DMA_GCR_T::GCRCSR: CLK6_EN Position       */
#define DMA_GCR_GCRCSR_CLK6_EN_Msk       (0x1ul << DMA_GCR_GCRCSR_CLK6_EN_Pos)             /*!< DMA_GCR_T::GCRCSR: CLK6_EN Mask           */

#define DMA_GCR_GCRCSR_CRC_CLK_EN_Pos    (24)                                              /*!< DMA_GCR_T::GCRCSR: CRC_CLK_EN Position    */
#define DMA_GCR_GCRCSR_CRC_CLK_EN_Msk    (0x1ul << DMA_GCR_GCRCSR_CRC_CLK_EN_Pos)          /*!< DMA_GCR_T::GCRCSR: CRC_CLK_EN Mask        */

#define DMA_GCR_DSSR0_CH1_SEL_Pos        (8)                                               /*!< DMA_GCR_T::DSSR0: CH1_SEL Position        */
#define DMA_GCR_DSSR0_CH1_SEL_Msk        (0x1ful << DMA_GCR_DSSR0_CH1_SEL_Pos)             /*!< DMA_GCR_T::DSSR0: CH1_SEL Mask            */

#define DMA_GCR_DSSR0_CH2_SEL_Pos        (16)                                              /*!< DMA_GCR_T::DSSR0: CH2_SEL Position        */
#define DMA_GCR_DSSR0_CH2_SEL_Msk        (0x1ful << DMA_GCR_DSSR0_CH2_SEL_Pos)             /*!< DMA_GCR_T::DSSR0: CH2_SEL Mask            */

#define DMA_GCR_DSSR0_CH3_SEL_Pos        (24)                                              /*!< DMA_GCR_T::DSSR0: CH3_SEL Position        */
#define DMA_GCR_DSSR0_CH3_SEL_Msk        (0x1ful << DMA_GCR_DSSR0_CH3_SEL_Pos)             /*!< DMA_GCR_T::DSSR0: CH3_SEL Mask            */

#define DMA_GCR_DSSR1_CH4_SEL_Pos        (0)                                               /*!< DMA_GCR_T::DSSR1: CH4_SEL Position        */
#define DMA_GCR_DSSR1_CH4_SEL_Msk        (0x1ful << DMA_GCR_DSSR1_CH4_SEL_Pos)             /*!< DMA_GCR_T::DSSR1: CH4_SEL Mask            */

#define DMA_GCR_DSSR1_CH5_SEL_Pos        (8)                                               /*!< DMA_GCR_T::DSSR1: CH5_SEL Position        */
#define DMA_GCR_DSSR1_CH5_SEL_Msk        (0x1ful << DMA_GCR_DSSR1_CH5_SEL_Pos)             /*!< DMA_GCR_T::DSSR1: CH5_SEL Mask            */

#define DMA_GCR_DSSR1_CH6_SEL_Pos        (16)                                              /*!< DMA_GCR_T::DSSR1: CH6_SEL Position        */
#define DMA_GCR_DSSR1_CH6_SEL_Msk        (0x1ful << DMA_GCR_DSSR1_CH6_SEL_Pos)             /*!< DMA_GCR_T::DSSR1: CH6_SEL Mask            */

#define DMA_GCR_GCRISR_INTR0_Pos         (0)                                               /*!< DMA_GCR_T::GCRISR: INTR0 Position         */
#define DMA_GCR_GCRISR_INTR0_Msk         (0x1ul << DMA_GCR_GCRISR_INTR0_Pos)               /*!< DMA_GCR_T::GCRISR: INTR0 Mask             */

#define DMA_GCR_GCRISR_INTR1_Pos         (1)                                               /*!< DMA_GCR_T::GCRISR: INTR1 Position         */
#define DMA_GCR_GCRISR_INTR1_Msk         (0x1ul << DMA_GCR_GCRISR_INTR1_Pos)               /*!< DMA_GCR_T::GCRISR: INTR1 Mask             */

#define DMA_GCR_GCRISR_INTR2_Pos         (2)                                               /*!< DMA_GCR_T::GCRISR: INTR2 Position         */
#define DMA_GCR_GCRISR_INTR2_Msk         (0x1ul << DMA_GCR_GCRISR_INTR2_Pos)               /*!< DMA_GCR_T::GCRISR: INTR2 Mask             */

#define DMA_GCR_GCRISR_INTR3_Pos         (3)                                               /*!< DMA_GCR_T::GCRISR: INTR3 Position         */
#define DMA_GCR_GCRISR_INTR3_Msk         (0x1ul << DMA_GCR_GCRISR_INTR3_Pos)               /*!< DMA_GCR_T::GCRISR: INTR3 Mask             */

#define DMA_GCR_GCRISR_INTR4_Pos         (4)                                               /*!< DMA_GCR_T::GCRISR: INTR4 Position         */
#define DMA_GCR_GCRISR_INTR4_Msk         (0x1ul << DMA_GCR_GCRISR_INTR4_Pos)               /*!< DMA_GCR_T::GCRISR: INTR4 Mask             */

#define DMA_GCR_GCRISR_INTR5_Pos         (5)                                               /*!< DMA_GCR_T::GCRISR: INTR5 Position         */
#define DMA_GCR_GCRISR_INTR5_Msk         (0x1ul << DMA_GCR_GCRISR_INTR5_Pos)               /*!< DMA_GCR_T::GCRISR: INTR5 Mask             */

#define DMA_GCR_GCRISR_INTR6_Pos         (6)                                               /*!< DMA_GCR_T::GCRISR: INTR6 Position         */
#define DMA_GCR_GCRISR_INTR6_Msk         (0x1ul << DMA_GCR_GCRISR_INTR6_Pos)               /*!< DMA_GCR_T::GCRISR: INTR6 Mask             */

#define DMA_GCR_GCRISR_CRC_INTR_Pos      (16)                                              /*!< DMA_GCR_T::GCRISR: CRC_INTR Position      */
#define DMA_GCR_GCRISR_CRC_INTR_Msk      (0x1ul << DMA_GCR_GCRISR_CRC_INTR_Pos)            /*!< DMA_GCR_T::GCRISR: CRC_INTR Mask          */

/**@}*/ /* DMA_GCR_CONST */


/**
    @addtogroup PDMA_CONST PDMA Bit Field Definition
    Constant Definitions for PDMA Controller
@{ */

#define PDMA_CSR_PDMACEN_Pos             (0)                                               /*!< PDMA_T::CSR: PDMACEN Position             */
#define PDMA_CSR_PDMACEN_Msk             (0x1ul << PDMA_CSR_PDMACEN_Pos)                   /*!< PDMA_T::CSR: PDMACEN Mask                 */

#define PDMA_CSR_SW_RST_Pos              (1)                                               /*!< PDMA_T::CSR: SW_RST Position              */
#define PDMA_CSR_SW_RST_Msk              (0x1ul << PDMA_CSR_SW_RST_Pos)                    /*!< PDMA_T::CSR: SW_RST Mask                  */

#define PDMA_CSR_MODE_SEL_Pos            (2)                                               /*!< PDMA_T::CSR: MODE_SEL Position            */
#define PDMA_CSR_MODE_SEL_Msk            (0x3ul << PDMA_CSR_MODE_SEL_Pos)                  /*!< PDMA_T::CSR: MODE_SEL Mask                */

#define PDMA_CSR_SAD_SEL_Pos             (4)                                               /*!< PDMA_T::CSR: SAD_SEL Position             */
#define PDMA_CSR_SAD_SEL_Msk             (0x3ul << PDMA_CSR_SAD_SEL_Pos)                   /*!< PDMA_T::CSR: SAD_SEL Mask                 */

#define PDMA_CSR_DAD_SEL_Pos             (6)                                               /*!< PDMA_T::CSR: DAD_SEL Position             */
#define PDMA_CSR_DAD_SEL_Msk             (0x3ul << PDMA_CSR_DAD_SEL_Pos)                   /*!< PDMA_T::CSR: DAD_SEL Mask                 */

#define PDMA_CSR_TO_EN_Pos               (12)                                              /*!< PDMA_T::CSR: TO_EN Position               */
#define PDMA_CSR_TO_EN_Msk               (0x1ul << PDMA_CSR_TO_EN_Pos)                     /*!< PDMA_T::CSR: TO_EN Mask                   */

#define PDMA_CSR_APB_TWS_Pos             (19)                                              /*!< PDMA_T::CSR: APB_TWS Position             */
#define PDMA_CSR_APB_TWS_Msk             (0x3ul << PDMA_CSR_APB_TWS_Pos)                   /*!< PDMA_T::CSR: APB_TWS Mask                 */

#define PDMA_CSR_TRIG_EN_Pos             (23)                                              /*!< PDMA_T::CSR: TRIG_EN Position             */
#define PDMA_CSR_TRIG_EN_Msk             (0x1ul << PDMA_CSR_TRIG_EN_Pos)                   /*!< PDMA_T::CSR: TRIG_EN Mask                 */

#define PDMA_SAR_PDMA_SAR_Pos            (0)                                               /*!< PDMA_T::SAR: PDMA_SAR Position            */
#define PDMA_SAR_PDMA_SAR_Msk            (0xfffffffful << PDMA_SAR_PDMA_SAR_Pos)           /*!< PDMA_T::SAR: PDMA_SAR Mask                */

#define PDMA_DAR_PDMA_DAR_Pos            (0)                                               /*!< PDMA_T::DAR: PDMA_DAR Position            */
#define PDMA_DAR_PDMA_DAR_Msk            (0xfffffffful << PDMA_DAR_PDMA_DAR_Pos)           /*!< PDMA_T::DAR: PDMA_DAR Mask                */

#define PDMA_BCR_PDMA_BCR_Pos            (0)                                               /*!< PDMA_T::BCR: PDMA_BCR Position            */
#define PDMA_BCR_PDMA_BCR_Msk            (0xfffful << PDMA_BCR_PDMA_BCR_Pos)               /*!< PDMA_T::BCR: PDMA_BCR Mask                */

#define PDMA_CSAR_PDMA_CSAR_Pos          (0)                                               /*!< PDMA_T::CSAR: PDMA_CSAR Position          */
#define PDMA_CSAR_PDMA_CSAR_Msk          (0xfffffffful << PDMA_CSAR_PDMA_CSAR_Pos)         /*!< PDMA_T::CSAR: PDMA_CSAR Mask              */

#define PDMA_CDAR_PDMA_CDAR_Pos          (0)                                               /*!< PDMA_T::CDAR: PDMA_CDAR Position          */
#define PDMA_CDAR_PDMA_CDAR_Msk          (0xfffffffful << PDMA_CDAR_PDMA_CDAR_Pos)         /*!< PDMA_T::CDAR: PDMA_CDAR Mask              */

#define PDMA_CBCR_PDMA_CBCR_Pos          (0)                                               /*!< PDMA_T::CBCR: PDMA_CBCR Position          */
#define PDMA_CBCR_PDMA_CBCR_Msk          (0xfffffful << PDMA_CBCR_PDMA_CBCR_Pos)           /*!< PDMA_T::CBCR: PDMA_CBCR Mask              */

#define PDMA_IER_TABORT_IE_Pos           (0)                                               /*!< PDMA_T::IER: TABORT_IE Position           */
#define PDMA_IER_TABORT_IE_Msk           (0x1ul << PDMA_IER_TABORT_IE_Pos)                 /*!< PDMA_T::IER: TABORT_IE Mask               */

#define PDMA_IER_TD_IE_Pos               (1)                                               /*!< PDMA_T::IER: TD_IE Position               */
#define PDMA_IER_TD_IE_Msk               (0x1ul << PDMA_IER_TD_IE_Pos)                     /*!< PDMA_T::IER: TD_IE Mask                   */

#define PDMA_IER_WRA_BCR_IE_Pos          (2)                                               /*!< PDMA_T::IER: WRA_BCR_IE Position          */
#define PDMA_IER_WRA_BCR_IE_Msk          (0xful << PDMA_IER_WRA_BCR_IE_Pos)                /*!< PDMA_T::IER: WRA_BCR_IE Mask              */

#define PDMA_IER_TO_IE_Pos               (6)                                               /*!< PDMA_T::IER: TO_IE Position               */
#define PDMA_IER_TO_IE_Msk               (0x1ul << PDMA_IER_TO_IE_Pos)                     /*!< PDMA_T::IER: TO_IE Mask                   */

#define PDMA_ISR_TABORT_IS_Pos           (0)                                               /*!< PDMA_T::ISR: TABORT_IS Position           */
#define PDMA_ISR_TABORT_IS_Msk           (0x1ul << PDMA_ISR_TABORT_IS_Pos)                 /*!< PDMA_T::ISR: TABORT_IS Mask               */

#define PDMA_ISR_TD_IS_Pos               (1)                                               /*!< PDMA_T::ISR: TD_IS Position               */
#define PDMA_ISR_TD_IS_Msk               (0x1ul << PDMA_ISR_TD_IS_Pos)                     /*!< PDMA_T::ISR: TD_IS Mask                   */

#define PDMA_ISR_WRA_BCR_IS_Pos          (2)                                               /*!< PDMA_T::ISR: WRA_BCR_IS Position          */
#define PDMA_ISR_WRA_BCR_IS_Msk          (0xful << PDMA_ISR_WRA_BCR_IS_Pos)                /*!< PDMA_T::ISR: WRA_BCR_IS Mask              */

#define PDMA_ISR_TO_IS_Pos               (6)                                               /*!< PDMA_T::ISR: TO_IS Position               */
#define PDMA_ISR_TO_IS_Msk               (0x1ul << PDMA_ISR_TO_IS_Pos)                     /*!< PDMA_T::ISR: TO_IS Mask                   */

#define PDMA_TCR_PDMA_TCR_Pos            (0)                                               /*!< PDMA_T::TCR: PDMA_TCR Position            */
#define PDMA_TCR_PDMA_TCR_Msk            (0xfffful << PDMA_TCR_PDMA_TCR_Pos)               /*!< PDMA_T::TCR: PDMA_TCR Mask                */

/**@}*/ /* PDMA_CONST */


/**
    @addtogroup VDMA_CONST VDMA Bit Field Definition
    Constant Definitions for VDMA Controller
@{ */

#define VDMA_CSR_VDMACEN_Pos             (0)                                               /*!< VDMA_T::CSR: VDMACEN Position             */
#define VDMA_CSR_VDMACEN_Msk             (0x1ul << VDMA_CSR_VDMACEN_Pos)                   /*!< VDMA_T::CSR: VDMACEN Mask                 */

#define VDMA_CSR_SW_RST_Pos              (1)                                               /*!< VDMA_T::CSR: SW_RST Position              */
#define VDMA_CSR_SW_RST_Msk              (0x1ul << VDMA_CSR_SW_RST_Pos)                    /*!< VDMA_T::CSR: SW_RST Mask                  */

#define VDMA_CSR_STRIDE_EN_Pos           (10)                                              /*!< VDMA_T::CSR: STRIDE_EN Position           */
#define VDMA_CSR_STRIDE_EN_Msk           (0x1ul << VDMA_CSR_STRIDE_EN_Pos)                 /*!< VDMA_T::CSR: STRIDE_EN Mask               */

#define VDMA_CSR_DIR_SEL_Pos             (11)                                              /*!< VDMA_T::CSR: DIR_SEL Position             */
#define VDMA_CSR_DIR_SEL_Msk             (0x1ul << VDMA_CSR_DIR_SEL_Pos)                   /*!< VDMA_T::CSR: DIR_SEL Mask                 */

#define VDMA_CSR_TRIG_EN_Pos             (23)                                              /*!< VDMA_T::CSR: TRIG_EN Position             */
#define VDMA_CSR_TRIG_EN_Msk             (0x1ul << VDMA_CSR_TRIG_EN_Pos)                   /*!< VDMA_T::CSR: TRIG_EN Mask                 */

#define VDMA_SAR_VDMA_SAR_Pos            (0)                                               /*!< VDMA_T::SAR: VDMA_SAR Position            */
#define VDMA_SAR_VDMA_SAR_Msk            (0xfffffffful << VDMA_SAR_VDMA_SAR_Pos)           /*!< VDMA_T::SAR: VDMA_SAR Mask                */

#define VDMA_DAR_VDMA_DAR_Pos            (0)                                               /*!< VDMA_T::DAR: VDMA_DAR Position            */
#define VDMA_DAR_VDMA_DAR_Msk            (0xfffffffful << VDMA_DAR_VDMA_DAR_Pos)           /*!< VDMA_T::DAR: VDMA_DAR Mask                */

#define VDMA_BCR_VDMA_BCR_Pos            (0)                                               /*!< VDMA_T::BCR: VDMA_BCR Position            */
#define VDMA_BCR_VDMA_BCR_Msk            (0xfffful << VDMA_BCR_VDMA_BCR_Pos)               /*!< VDMA_T::BCR: VDMA_BCR Mask                */

#define VDMA_CSAR_VDMA_CSAR_Pos          (0)                                               /*!< VDMA_T::CSAR: VDMA_CSAR Position          */
#define VDMA_CSAR_VDMA_CSAR_Msk          (0xfffffffful << VDMA_CSAR_VDMA_CSAR_Pos)         /*!< VDMA_T::CSAR: VDMA_CSAR Mask              */

#define VDMA_CDAR_VDMA_CDAR_Pos          (0)                                               /*!< VDMA_T::CDAR: VDMA_CDAR Position          */
#define VDMA_CDAR_VDMA_CDAR_Msk          (0xfffffffful << VDMA_CDAR_VDMA_CDAR_Pos)         /*!< VDMA_T::CDAR: VDMA_CDAR Mask              */

#define VDMA_CBCR_VDMA_CBCR_Pos          (0)                                               /*!< VDMA_T::CBCR: VDMA_CBCR Position          */
#define VDMA_CBCR_VDMA_CBCR_Msk          (0xfffful << VDMA_CBCR_VDMA_CBCR_Pos)             /*!< VDMA_T::CBCR: VDMA_CBCR Mask              */

#define VDMA_IER_TABORT_IE_Pos           (0)                                               /*!< VDMA_T::IER: TABORT_IE Position           */
#define VDMA_IER_TABORT_IE_Msk           (0x1ul << VDMA_IER_TABORT_IE_Pos)                 /*!< VDMA_T::IER: TABORT_IE Mask               */

#define VDMA_IER_TD_IE_Pos               (1)                                               /*!< VDMA_T::IER: TD_IE Position               */
#define VDMA_IER_TD_IE_Msk               (0x1ul << VDMA_IER_TD_IE_Pos)                     /*!< VDMA_T::IER: TD_IE Mask                   */

#define VDMA_ISR_TABORT_IS_Pos           (0)                                               /*!< VDMA_T::ISR: TABORT_IS Position           */
#define VDMA_ISR_TABORT_IS_Msk           (0x1ul << VDMA_ISR_TABORT_IS_Pos)                 /*!< VDMA_T::ISR: TABORT_IS Mask               */

#define VDMA_ISR_TD_IS_Pos               (1)                                               /*!< VDMA_T::ISR: TD_IS Position               */
#define VDMA_ISR_TD_IS_Msk               (0x1ul << VDMA_ISR_TD_IS_Pos)                     /*!< VDMA_T::ISR: TD_IS Mask                   */

#define VDMA_SASOCR_SASTOBL_Pos          (0)                                               /*!< VDMA_T::SASOCR: SASTOBL Position          */
#define VDMA_SASOCR_SASTOBL_Msk          (0xfffful << VDMA_SASOCR_SASTOBL_Pos)             /*!< VDMA_T::SASOCR: SASTOBL Mask              */

#define VDMA_SASOCR_STBC_Pos             (16)                                              /*!< VDMA_T::SASOCR: STBC Position             */
#define VDMA_SASOCR_STBC_Msk             (0xfffful << VDMA_SASOCR_STBC_Pos)                /*!< VDMA_T::SASOCR: STBC Mask                 */

#define VDMA_DASOCR_DASTOBL_Pos          (0)                                               /*!< VDMA_T::DASOCR: DASTOBL Position          */
#define VDMA_DASOCR_DASTOBL_Msk          (0xfffful << VDMA_DASOCR_DASTOBL_Pos)             /*!< VDMA_T::DASOCR: DASTOBL Mask              */

/**@}*/ /* VDMA_CONST */

/**@}*/ /* end of DMA register group */


/*---------------------- Pulse Width Modulation Controller -------------------------*/
/**
    @addtogroup PWM Pulse Width Modulation Controller(PWM)
    Memory Mapped Structure for PWM Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PRES</font><br><p> <font size="2">
Offset: 0x00  PWM Prescaler Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>CP01</td><td><div style="word-wrap: break-word;"><b>Clock Prescaler 0 For PWM Timer 0 & 1</b><br>
Clock input is divided by (CP01 + 1) before it is fed to the counter 0 & 1<br>
If CP01 =0, the prescaler 0 output clock will be stopped. So PWM counter 0 and 1 will be stopped also.<br>
</div></td></tr><tr><td>
[15:8]</td><td>CP23</td><td><div style="word-wrap: break-word;"><b>Clock Prescaler 2 For PWM Timer 2 & 3</b><br>
Clock input is divided by (CP23 + 1) before it is fed to the counter 2 & 3<br>
If CP23=0, the prescaler 2 output clock will be stopped. So PWM counter2 and 3 will be stopped also.<br>
</div></td></tr><tr><td>
[23:16]</td><td>DZ01</td><td><div style="word-wrap: break-word;"><b>Dead Zone Interval Register For CH0 And CH1 Pair</b><br>
These 8 bits determine dead zone length.<br>
The unit time of dead zone length is received from clock selector 0.<br>
</div></td></tr><tr><td>
[31:24]</td><td>DZ23</td><td><div style="word-wrap: break-word;"><b>Dead Zone Interval Register For CH2 And CH3 Pair</b><br>
These 8 bits determine dead zone length.<br>
The unit time of dead zone length is received from clock selector 2.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PRES;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CLKSEL</font><br><p> <font size="2">
Offset: 0x04  PWM Clock Select Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>CLKSEL0</td><td><div style="word-wrap: break-word;"><b>Timer 0 Clock Source Selection</b><br>
Select clock input for timer 0.<br>
(Table is the same as CLKSEL3)<br>
</div></td></tr><tr><td>
[6:4]</td><td>CLKSEL1</td><td><div style="word-wrap: break-word;"><b>Timer 1 Clock Source Selection</b><br>
Select clock input for timer 1.<br>
(Table is the same as CLKSEL3)<br>
</div></td></tr><tr><td>
[10:8]</td><td>CLKSEL2</td><td><div style="word-wrap: break-word;"><b>Timer 2Clock Source Selection</b><br>
Select clock input for timer 2.<br>
(Table is the same as CLKSEL3)<br>
</div></td></tr><tr><td>
[14:12]</td><td>CLKSEL3</td><td><div style="word-wrap: break-word;"><b>Timer 3 Clock Source Selection</b><br>
Select clock input for timer 3.<br>
000 = Input Clock Divided by 2.<br>
001 = Input Clock Divided by 4.<br>
010 = Input Clock Divided by 8.<br>
011 = Input Clock Divided by 16.<br>
100 = Input Clock Divided by 1.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CLKSEL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTL</font><br><p> <font size="2">
Offset: 0x08  PWM Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>CH0EN</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 0 Enable/Disable Start Run</b><br>
0 = PWM-Timer 0 Running Stopped.<br>
1 = PWM-Timer 0 Start Run Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>CH0INV</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 0 Output Inverter ON/OFF</b><br>
0 = Inverter OFF.<br>
1 = Inverter ON.<br>
</div></td></tr><tr><td>
[3]</td><td>CH0MOD</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 0 Continuous/One-Shot Mode</b><br>
0 = One-Shot Mode.<br>
1 = Continuous Mode.<br>
Note: If there is a rising transition at this bit, it will cause CN and CM of PWM0_DUTY0 to be cleared.<br>
</div></td></tr><tr><td>
[4]</td><td>DZEN01</td><td><div style="word-wrap: break-word;"><b>Dead-Zone 0 Generator Enable/Disable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
Note: When Dead-Zone Generator is enabled, the pair of PWM0 and PWM1 becomes a complementary pair.<br>
</div></td></tr><tr><td>
[5]</td><td>DZEN23</td><td><div style="word-wrap: break-word;"><b>Dead-Zone 2 Generator Enable/Disable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
Note: When Dead-Zone Generator is enabled, the pair of PWM2 and PWM3 becomes a complementary pair.<br>
</div></td></tr><tr><td>
[8]</td><td>CH1EN</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 1 Enable/Disable Start Run</b><br>
0 = PWM-Timer 1 Running Stopped.<br>
1 = PWM-Timer 1 Start Run Enabled.<br>
</div></td></tr><tr><td>
[10]</td><td>CH1INV</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 1 Output Inverter ON/OFF</b><br>
0 = Inverter OFF.<br>
1 = Inverter ON.<br>
</div></td></tr><tr><td>
[11]</td><td>CH1MOD</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 1 Continuous/One-Shot Mode</b><br>
0 = One-Shot Mode.<br>
1 = Continuous Mode.<br>
Note: If there is a rising transition at this bit, it will cause CN and CM of PWM0_DUTY1 to be cleared.<br>
</div></td></tr><tr><td>
[16]</td><td>CH2EN</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 2 Enable/Disable Start Run</b><br>
0 = PWM-Timer 2 Running Stopped.<br>
1 = PWM-Timer 2 Start Run Enabled.<br>
</div></td></tr><tr><td>
[18]</td><td>CH2INV</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 2 Output Inverter ON/OFF</b><br>
0 = Inverter OFF.<br>
1 = Inverter ON.<br>
</div></td></tr><tr><td>
[19]</td><td>CH2MOD</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 2 Continuous/One-Shot Mode</b><br>
0 = One-Shot Mode.<br>
1 = Continuous Mode.<br>
Note: If there is a rising transition at this bit, it will cause CN and CM of PWM0_DUTY2 be cleared.<br>
</div></td></tr><tr><td>
[24]</td><td>CH3EN</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 3 Enable/Disable Start Run</b><br>
0 = PWM-Timer 3 Running Stopped.<br>
1 = PWM-Timer 3 Start Run Enabled.<br>
</div></td></tr><tr><td>
[26]</td><td>CH3INV</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 3 Output Inverter ON/OFF</b><br>
0 = Inverter OFF.<br>
1 = Inverter ON.<br>
</div></td></tr><tr><td>
[27]</td><td>CH3MOD</td><td><div style="word-wrap: break-word;"><b>PWM-Timer 3 Continuous/One-Shot Mode</b><br>
0 = One-Shot Mode.<br>
1 = Continuous Mode.<br>
Note: If there is a rising transition at this bit, it will cause CN and CM of PWM0_DUTY3 to be cleared.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">INTEN</font><br><p> <font size="2">
Offset: 0x0C  PWM Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TMIE0</td><td><div style="word-wrap: break-word;"><b>PWM Timer 0 Interrupt Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>TMIE1</td><td><div style="word-wrap: break-word;"><b>PWM Timer 1 Interrupt Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>TMIE2</td><td><div style="word-wrap: break-word;"><b>PWM Timer 2 Interrupt Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr><tr><td>
[3]</td><td>TMIE3</td><td><div style="word-wrap: break-word;"><b>PWM Timer 3 Interrupt Enable</b><br>
0 = Disabled.<br>
1 = Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t INTEN;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">INTSTS</font><br><p> <font size="2">
Offset: 0x10  PWM Interrupt Indication Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TMINT0</td><td><div style="word-wrap: break-word;"><b>PWM Timer 0 Interrupt Flag</b><br>
Flag is set by hardware when PWM0 down counter reaches zero, software can clear this bit by writing a one to it.<br>
</div></td></tr><tr><td>
[1]</td><td>TMINT1</td><td><div style="word-wrap: break-word;"><b>PWM Timer 1 Interrupt Flag</b><br>
Flag is set by hardware when PWM1 down counter reaches zero, software can clear this bit by writing a one to it.<br>
</div></td></tr><tr><td>
[2]</td><td>TMINT2</td><td><div style="word-wrap: break-word;"><b>PWM Timer 2 Interrupt Flag</b><br>
Flag is set by hardware when PWM2 down counter reaches zero, software can clear this bit by writing a one to it.<br>
</div></td></tr><tr><td>
[3]</td><td>TMINT3</td><td><div style="word-wrap: break-word;"><b>PWM Timer 3 Interrupt Flag</b><br>
Flag is set by hardware when PWM3 down counter reaches zero, software can clear this bit by writing a one to it.<br>
</div></td></tr><tr><td>
[4]</td><td>Duty0Syncflag</td><td><div style="word-wrap: break-word;"><b>Duty0 Synchronize Flag</b><br>
0 = Duty0 has been synchronized to ECLK domain.<br>
1 = Duty0 is synchronizing to ECLK domain.<br>
Note: software should check this flag when writing duty0, if this flag is set, and user ignore this flag and change duty0, the corresponding CNR and CMR may be wrong for one duty cycle<br>
</div></td></tr><tr><td>
[5]</td><td>Duty1Syncflag</td><td><div style="word-wrap: break-word;"><b>Duty1 Synchronize Flag</b><br>
0 = Duty1 has been synchronized to ECLK domain.<br>
1 = Duty1 is synchronizing to ECLK domain.<br>
Note: software should check this flag when writing duty1, if this flag is set, and user ignore this flag and change duty1, the corresponding CNR and CMR may be wrong for one duty cycle<br>
</div></td></tr><tr><td>
[6]</td><td>Duty2Syncflag</td><td><div style="word-wrap: break-word;"><b>Duty2 Synchronize Flag</b><br>
0 = Duty2 has been synchronized to ECLK domain.<br>
1 = Duty2 is synchronizing to ECLK domain.<br>
Note: software should check this flag when writing duty2, if this flag is set, and user ignore this flag and change duty2, the corresponding CNR and CMR may be wrong for one duty cycle<br>
</div></td></tr><tr><td>
[7]</td><td>Duty3Syncflag</td><td><div style="word-wrap: break-word;"><b>Duty3 Synchronize Flag</b><br>
0 = Duty3 has been synchronized to ECLK domain.<br>
1 = Duty3 is synchronizing to ECLK domain.<br>
Note: software should check this flag when writing duty3, if this flag is set, and user ignore this flag and change duty3, the corresponding CNR and CMR may be wrong for one duty cycle<br>
</div></td></tr><tr><td>
[8]</td><td>PresSyncFlag</td><td><div style="word-wrap: break-word;"><b>Prescale Synchronize Flag</b><br>
0 = Prescale has been synchronized to ECLK domain.<br>
1 = Prescale is synchronizing to ECLK domain.<br>
Note: software should check this flag when writing Prescale, if this flag is set, and user ignore this flag and change Prescale, the Prescale may be wrong for one prescale cycle<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t INTSTS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">OE</font><br><p> <font size="2">
Offset: 0x14  PWM Output Enable for PWM0~PWM3</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>CH0_OE</td><td><div style="word-wrap: break-word;"><b>PWM CH0 Output Enable Register</b><br>
0 = PWM CH0 output to pin Disabled.<br>
1 = PWM CH0 output to pin Enabled.<br>
Note: The corresponding GPI/O pin also must be switched to PWM function (refer to GPx_MFP)<br>
</div></td></tr><tr><td>
[1]</td><td>CH1_OE</td><td><div style="word-wrap: break-word;"><b>PWM CH1 Output Enable Register</b><br>
0 = PWM CH1 output to pin Disabled.<br>
1 = PWM CH1 output to pin Enabled.<br>
Note: The corresponding GPI/O pin also must be switched to PWM function (refer to GPx_MFP)<br>
</div></td></tr><tr><td>
[2]</td><td>CH2_OE</td><td><div style="word-wrap: break-word;"><b>PWM CH2 Output Enable Register</b><br>
0 = PWM CH2 output to pin Disabled.<br>
1 = PWM CH2 output to pin Enabled.<br>
Note: The corresponding GPI/O pin also must be switched to PWM function (refer to GPx_MFP)<br>
</div></td></tr><tr><td>
[3]</td><td>CH3_OE</td><td><div style="word-wrap: break-word;"><b>PWM CH3 Output Enable Register</b><br>
0 = PWM CH3 output to pin Disabled.<br>
1 = PWM CH3 output to pin Enabled.<br>
Note: The corresponding GPI/O pin also must be switched to PWM function (refer to GPx_MFP)<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t OE;
    uint32_t RESERVE0[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DUTY0</font><br><p> <font size="2">
Offset: 0x1C  PWM Counter/Comparator Register 0</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CN</td><td><div style="word-wrap: break-word;"><b>PWM Counter/Timer Loaded Value</b><br>
CN determines the PWM period.<br>
PWM frequency = PWMxy_CLK/(prescale+1)*(clock divider)/(CN+1); where xy, could be 01, 23, depends on selected PWM channel.<br>
Duty ratio = (CM+1)/(CN+1).<br>
CM >= CN: PWM output is always high.<br>
CM < CN: PWM low width = (CN-CM) unit; PWM high width = (CM+1) unit.<br>
CM = 0: PWM low width = (CN) unit; PWM high width = 1 unit.<br>
(Unit = one PWM clock cycle).<br>
Note:<br>
Any write to CN will take effect in next PWM cycle.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CM</td><td><div style="word-wrap: break-word;"><b>PWM Comparator Register</b><br>
CM determines the PWM duty.<br>
PWM frequency = PWMxy_CLK/(prescale+1)*(clock divider)/(CN+1); where xy, could be 01, 23, depending on the selected PWM channel.<br>
Duty ratio = (CM+1)/(CN+1).<br>
CM >= CN: PWM output is always high.<br>
CM < CN: PWM low width = (CN-CM) unit; PWM high width = (CM+1) unit.<br>
CM = 0: PWM low width = (CN) unit; PWM high width = 1 unit.<br>
(Unit = one PWM clock cycle).<br>
Note:<br>
Any write to CM will take effect in next PWM cycle.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DUTY0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DATA0</font><br><p> <font size="2">
Offset: 0x20  PWM Data Register 0</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>PWMx_DATAy15_0</td><td><div style="word-wrap: break-word;"><b>PWM Data Register</b><br>
User can monitor PWMx_DATAy to know the current value in 16-bit down count counter.<br>
</div></td></tr><tr><td>
[30:16]</td><td>PWMx_DATAy30_16</td><td><div style="word-wrap: break-word;"><b>PWM Data Register</b><br>
User can monitor PWMx_DATAy to know the current value in 32-bit down count counter<br>
Notes:This will be valid only for the corresponding cascade enable .bit is set<br>
</div></td></tr><tr><td>
[31]</td><td>sync</td><td><div style="word-wrap: break-word;"><b>Indicate That CNR Value Is Sync To PWM Counter</b><br>
0 = CNR value is sync to PWM counter.<br>
1 = CNR value is not sync to PWM counter.<br>
Note: when the corresponding cascade enable .bit is set is bit will not appear in the corresponding channel<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t DATA0;
    uint32_t RESERVE1[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DUTY1</font><br><p> <font size="2">
Offset: 0x28  PWM Counter/Comparator Register 1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CN</td><td><div style="word-wrap: break-word;"><b>PWM Counter/Timer Loaded Value</b><br>
CN determines the PWM period.<br>
PWM frequency = PWMxy_CLK/(prescale+1)*(clock divider)/(CN+1); where xy, could be 01, 23, depends on selected PWM channel.<br>
Duty ratio = (CM+1)/(CN+1).<br>
CM >= CN: PWM output is always high.<br>
CM < CN: PWM low width = (CN-CM) unit; PWM high width = (CM+1) unit.<br>
CM = 0: PWM low width = (CN) unit; PWM high width = 1 unit.<br>
(Unit = one PWM clock cycle).<br>
Note:<br>
Any write to CN will take effect in next PWM cycle.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CM</td><td><div style="word-wrap: break-word;"><b>PWM Comparator Register</b><br>
CM determines the PWM duty.<br>
PWM frequency = PWMxy_CLK/(prescale+1)*(clock divider)/(CN+1); where xy, could be 01, 23, depending on the selected PWM channel.<br>
Duty ratio = (CM+1)/(CN+1).<br>
CM >= CN: PWM output is always high.<br>
CM < CN: PWM low width = (CN-CM) unit; PWM high width = (CM+1) unit.<br>
CM = 0: PWM low width = (CN) unit; PWM high width = 1 unit.<br>
(Unit = one PWM clock cycle).<br>
Note:<br>
Any write to CM will take effect in next PWM cycle.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DUTY1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DATA1</font><br><p> <font size="2">
Offset: 0x2C  PWM Data Register 1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>PWMx_DATAy15_0</td><td><div style="word-wrap: break-word;"><b>PWM Data Register</b><br>
User can monitor PWMx_DATAy to know the current value in 16-bit down count counter.<br>
</div></td></tr><tr><td>
[30:16]</td><td>PWMx_DATAy30_16</td><td><div style="word-wrap: break-word;"><b>PWM Data Register</b><br>
User can monitor PWMx_DATAy to know the current value in 32-bit down count counter<br>
Notes:This will be valid only for the corresponding cascade enable .bit is set<br>
</div></td></tr><tr><td>
[31]</td><td>sync</td><td><div style="word-wrap: break-word;"><b>Indicate That CNR Value Is Sync To PWM Counter</b><br>
0 = CNR value is sync to PWM counter.<br>
1 = CNR value is not sync to PWM counter.<br>
Note: when the corresponding cascade enable .bit is set is bit will not appear in the corresponding channel<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t DATA1;
    uint32_t RESERVE2[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DUTY2</font><br><p> <font size="2">
Offset: 0x34  PWM Counter/Comparator Register 2</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CN</td><td><div style="word-wrap: break-word;"><b>PWM Counter/Timer Loaded Value</b><br>
CN determines the PWM period.<br>
PWM frequency = PWMxy_CLK/(prescale+1)*(clock divider)/(CN+1); where xy, could be 01, 23, depends on selected PWM channel.<br>
Duty ratio = (CM+1)/(CN+1).<br>
CM >= CN: PWM output is always high.<br>
CM < CN: PWM low width = (CN-CM) unit; PWM high width = (CM+1) unit.<br>
CM = 0: PWM low width = (CN) unit; PWM high width = 1 unit.<br>
(Unit = one PWM clock cycle).<br>
Note:<br>
Any write to CN will take effect in next PWM cycle.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CM</td><td><div style="word-wrap: break-word;"><b>PWM Comparator Register</b><br>
CM determines the PWM duty.<br>
PWM frequency = PWMxy_CLK/(prescale+1)*(clock divider)/(CN+1); where xy, could be 01, 23, depending on the selected PWM channel.<br>
Duty ratio = (CM+1)/(CN+1).<br>
CM >= CN: PWM output is always high.<br>
CM < CN: PWM low width = (CN-CM) unit; PWM high width = (CM+1) unit.<br>
CM = 0: PWM low width = (CN) unit; PWM high width = 1 unit.<br>
(Unit = one PWM clock cycle).<br>
Note:<br>
Any write to CM will take effect in next PWM cycle.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DUTY2;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DATA2</font><br><p> <font size="2">
Offset: 0x38  PWM Data Register 2</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>PWMx_DATAy15_0</td><td><div style="word-wrap: break-word;"><b>PWM Data Register</b><br>
User can monitor PWMx_DATAy to know the current value in 16-bit down count counter.<br>
</div></td></tr><tr><td>
[30:16]</td><td>PWMx_DATAy30_16</td><td><div style="word-wrap: break-word;"><b>PWM Data Register</b><br>
User can monitor PWMx_DATAy to know the current value in 32-bit down count counter<br>
Notes:This will be valid only for the corresponding cascade enable .bit is set<br>
</div></td></tr><tr><td>
[31]</td><td>sync</td><td><div style="word-wrap: break-word;"><b>Indicate That CNR Value Is Sync To PWM Counter</b><br>
0 = CNR value is sync to PWM counter.<br>
1 = CNR value is not sync to PWM counter.<br>
Note: when the corresponding cascade enable .bit is set is bit will not appear in the corresponding channel<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t DATA2;
    uint32_t RESERVE3[1];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DUTY3</font><br><p> <font size="2">
Offset: 0x40  PWM Counter/Comparator Register 3</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CN</td><td><div style="word-wrap: break-word;"><b>PWM Counter/Timer Loaded Value</b><br>
CN determines the PWM period.<br>
PWM frequency = PWMxy_CLK/(prescale+1)*(clock divider)/(CN+1); where xy, could be 01, 23, depends on selected PWM channel.<br>
Duty ratio = (CM+1)/(CN+1).<br>
CM >= CN: PWM output is always high.<br>
CM < CN: PWM low width = (CN-CM) unit; PWM high width = (CM+1) unit.<br>
CM = 0: PWM low width = (CN) unit; PWM high width = 1 unit.<br>
(Unit = one PWM clock cycle).<br>
Note: Any write to CN will take effect in next PWM cycle.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CM</td><td><div style="word-wrap: break-word;"><b>PWM Comparator Register</b><br>
CM determines the PWM duty.<br>
PWM frequency = PWMxy_CLK/(prescale+1)*(clock divider)/(CN+1); where xy, could be 01, 23, depending on the selected PWM channel.<br>
Duty ratio = (CM+1)/(CN+1).<br>
CM >= CN: PWM output is always high.<br>
CM < CN: PWM low width = (CN-CM) unit; PWM high width = (CM+1) unit.<br>
CM = 0: PWM low width = (CN) unit; PWM high width = 1 unit.<br>
(Unit = one PWM clock cycle).<br>
Note: Any write to CM will take effect in next PWM cycle.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DUTY3;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DATA3</font><br><p> <font size="2">
Offset: 0x44  PWM Data Register 3</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>PWMx_DATAy15_0</td><td><div style="word-wrap: break-word;"><b>PWM Data Register</b><br>
User can monitor PWMx_DATAy to know the current value in 16-bit down count counter.<br>
</div></td></tr><tr><td>
[30:16]</td><td>PWMx_DATAy30_16</td><td><div style="word-wrap: break-word;"><b>PWM Data Register</b><br>
User can monitor PWMx_DATAy to know the current value in 32-bit down count counter<br>
Notes:This will be valid only for the corresponding cascade enable .bit is set<br>
</div></td></tr><tr><td>
[31]</td><td>sync</td><td><div style="word-wrap: break-word;"><b>Indicate That CNR Value Is Sync To PWM Counter</b><br>
0 = CNR value is sync to PWM counter.<br>
1 = CNR value is not sync to PWM counter.<br>
Note: when the corresponding cascade enable .bit is set is bit will not appear in the corresponding channel<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t DATA3;
    uint32_t RESERVE4[3];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CAPCTL</font><br><p> <font size="2">
Offset: 0x54  Capture Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>INV0</td><td><div style="word-wrap: break-word;"><b>Channel 0 Inverter ON/OFF</b><br>
0 = Inverter OFF.<br>
1 = Inverter ON. Reverse the input signal from GPIO before fed to Capture timer<br>
</div></td></tr><tr><td>
[1]</td><td>CAPCH0EN</td><td><div style="word-wrap: break-word;"><b>Capture Channel 0 Transition Enable/Disable</b><br>
0 = Capture function on channel 0 Disabled.<br>
1 = Capture function on channel 0 Enabled.<br>
When Enabled, Capture latched the PWM-timer value and saved to PWM_CRL0 (Rising latch) and PWM_CFL0 (Falling latch).<br>
When Disabled, Capture does not update PWM_CRL0 and PWM_CFL0, and disable Channel 0 Interrupt.<br>
</div></td></tr><tr><td>
[2]</td><td>CAPCH0PADEN</td><td><div style="word-wrap: break-word;"><b>Capture Input Enable</b><br>
0 = OFF.<br>
1 = ON.<br>
</div></td></tr><tr><td>
[3]</td><td>CH0PDMAEN</td><td><div style="word-wrap: break-word;"><b>Channel 0 PDMA Enable</b><br>
0 = Channel 0 PDMA function Disabled.<br>
1 = Channel 0 PDMA function Enabled for the channel 0 captured data and transfer to memory.<br>
</div></td></tr><tr><td>
[5:4]</td><td>PDMACAPMOD0</td><td><div style="word-wrap: break-word;"><b>Select CRL0 Or CFL0 For PDMA Transfer</b><br>
00 = Reserved.<br>
01 = CRL0.<br>
10 = CFL0.<br>
11 = Both CRL0 and CFL0.<br>
</div></td></tr><tr><td>
[6]</td><td>CAPRELOADREN0</td><td><div style="word-wrap: break-word;"><b>Reload CNR0 When CH0 Capture Rising Event Comes</b><br>
0 = Rising capture reload for CH0 Disabled.<br>
1 = Rising capture reload for CH0 Enabled.<br>
</div></td></tr><tr><td>
[7]</td><td>CAPRELOADFEN0</td><td><div style="word-wrap: break-word;"><b>Reload CNR0 When CH0 Capture Falling Event Comes</b><br>
0 = Falling capture reload for CH0 Disabled.<br>
1 = Falling capture reload for CH0 Enabled.<br>
</div></td></tr><tr><td>
[8]</td><td>INV1</td><td><div style="word-wrap: break-word;"><b>Channel 1 Inverter ON/OFF</b><br>
0 = Inverter OFF.<br>
1 = Inverter ON. Reverse the input signal from GPIO before fed to Capture timer<br>
</div></td></tr><tr><td>
[9]</td><td>CAPCH1EN</td><td><div style="word-wrap: break-word;"><b>Capture Channel 1 Transition Enable/Disable</b><br>
0 = Capture function on channel 1 Disabled.<br>
1 = Capture function on channel 1 Enabled.<br>
When Enabled, Capture latched the PMW-counter and saved to PWM_CRL1 (Rising latch) and PWM_CFL1 (Falling latch).<br>
When Disabled, Capture does not update PWM_CRL1 and PWM_CFL1, and disable Channel 1 Interrupt.<br>
</div></td></tr><tr><td>
[10]</td><td>CAPCH1PADEN</td><td><div style="word-wrap: break-word;"><b>Capture Input Enable</b><br>
0 = OFF.<br>
1 = ON.<br>
</div></td></tr><tr><td>
[12]</td><td>CH0RFORDER</td><td><div style="word-wrap: break-word;"><b>Channel 0 capture order control</b><br>
Set this bit to determine whether the PWM_CRL0 or PWM_CFL0 is the first captured data transferred to memory through PDMA when PDMACAPMOD0 =2'b11.<br>
0 = PWM_CFL0 is the first captured data to memory.<br>
1 = PWM_CRL0 is the first captured data to memory.<br>
</div></td></tr><tr><td>
[13]</td><td>CH01CASK</td><td><div style="word-wrap: break-word;"><b>Cascade channel 0 and channel 1 PWM timer for capturing usage</b><br>
</div></td></tr><tr><td>
[14]</td><td>CAPRELOADREN1</td><td><div style="word-wrap: break-word;"><b>Reload CNR1 When CH1 Capture Rising Event Comes</b><br>
0 = Rising capture reload for CH1 Disabled.<br>
1 = Rising capture reload for CH1 Enabled.<br>
</div></td></tr><tr><td>
[15]</td><td>CAPRELOADFEN1</td><td><div style="word-wrap: break-word;"><b>Reload CNR1 When CH1 Capture Falling Event Coming</b><br>
0 = Capture falling reload for CH1 Disabled.<br>
1 = Capture falling reload for CH1 Enabled.<br>
</div></td></tr><tr><td>
[16]</td><td>INV2</td><td><div style="word-wrap: break-word;"><b>Channel 2 Inverter ON/OFF</b><br>
0 = Inverter OFF.<br>
1 = Inverter ON. Reverse the input signal from GPIO before fed to Capture timer<br>
</div></td></tr><tr><td>
[17]</td><td>CAPCH2EN</td><td><div style="word-wrap: break-word;"><b>Capture Channel 2 Transition Enable/Disable</b><br>
0 = Capture function on channel 2 Disabled.<br>
1 = Capture function on channel 2 Enabled.<br>
When Enabled, Capture latched the PWM-timer value and saved to PWM_CRL2 (Rising latch) and PWM_CFL2 (Falling latch).<br>
When Disabled, Capture does not update PWM_CRL2 and PWM_CFL2, and disable Channel 2 Interrupt.<br>
</div></td></tr><tr><td>
[18]</td><td>CAPCH2PADEN</td><td><div style="word-wrap: break-word;"><b>Capture Input Enable</b><br>
0 = OFF.<br>
1 = ON.<br>
</div></td></tr><tr><td>
[19]</td><td>CH2PDMAEN</td><td><div style="word-wrap: break-word;"><b>Channel 2 PDMA Enable</b><br>
0 = Channel 2 PDMA function Disabled.<br>
1 = Channel 2 PDMA function Enabled for the channel 2 captured data and transfer to memory.<br>
</div></td></tr><tr><td>
[21:20]</td><td>PDMACAPMOD2</td><td><div style="word-wrap: break-word;"><b>Select CRL2 Or CFL2 For PDMA Transfer</b><br>
00 = Reserved.<br>
01 = CRL2.<br>
10 = CFL2.<br>
11 = Both CRL2 and CFL2.<br>
</div></td></tr><tr><td>
[22]</td><td>CAPRELOADREN2</td><td><div style="word-wrap: break-word;"><b>Reload CNR2 When CH2 Capture Rising Event Coming</b><br>
0 = Rising capture reload for CH2 Disabled.<br>
1 = Rising capture reload for CH2 Enabled.<br>
</div></td></tr><tr><td>
[23]</td><td>CAPRELOADFEN2</td><td><div style="word-wrap: break-word;"><b>Reload CNR2 When CH2 Capture Failing Event Coming</b><br>
0 = Failing capture reload for CH2 Disabled.<br>
1 = Failing capture reload for CH2 Enabled.<br>
</div></td></tr><tr><td>
[24]</td><td>INV3</td><td><div style="word-wrap: break-word;"><b>Channel 3 Inverter ON/OFF</b><br>
0 = Inverter OFF.<br>
1 = Inverter ON. Reverse the input signal from GPIO before fed to Capture timer<br>
</div></td></tr><tr><td>
[25]</td><td>CAPCH3EN</td><td><div style="word-wrap: break-word;"><b>Capture Channel 3 Transition Enable/Disable</b><br>
0 = Capture function on channel 3 Disabled.<br>
1 = Capture function on channel 3 Enabled.<br>
When Enabled, Capture latched the PMW-timer and saved to PWM_CRL3 (Rising latch) and PWM_CFL3 (Falling latch).<br>
When Disabled, Capture does not update PWM_CRL3 and PWM_CFL3, and disable Channel 3 Interrupt.<br>
</div></td></tr><tr><td>
[26]</td><td>CAPCH3PADEN</td><td><div style="word-wrap: break-word;"><b>Capture Input Enable</b><br>
0 = OFF.<br>
1 = ON.<br>
</div></td></tr><tr><td>
[28]</td><td>CH2RFORDER</td><td><div style="word-wrap: break-word;"><b>Channel 0 capture order control</b><br>
Set this bit to determine whether the PWM_CRL2 or PWM_CFL2 is the first captured data transferred to memory through PDMA when PDMACAPMOD2 = 2'b11.<br>
0 = PWM_CFL2 is the first captured data to memory.<br>
1 = PWM_CRL2 is the first captured data to memory.<br>
</div></td></tr><tr><td>
[29]</td><td>CH23CASK</td><td><div style="word-wrap: break-word;"><b>Cascade channel 2 and channel 3 PWM counter for capturing usage</b><br>
</div></td></tr><tr><td>
[30]</td><td>CAPRELOADREN3</td><td><div style="word-wrap: break-word;"><b>Reload CNR3 When CH3 Rising Capture Event Comes</b><br>
0 = Rising capture reload for CH3 Disabled.<br>
1 = Rising capture reload for CH3 Enabled.<br>
</div></td></tr><tr><td>
[31]</td><td>CAPRELOADFEN3</td><td><div style="word-wrap: break-word;"><b>Reload CNR3 When CH3 Falling Capture Event Comes</b><br>
0 = Falling capture reload for CH3 Disabled.<br>
1 = Falling capture reload for CH3 Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CAPCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CAPINTEN</font><br><p> <font size="2">
Offset: 0x58  Capture interrupt enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>CRL_IE0</td><td><div style="word-wrap: break-word;"><b>Channel 0 Rising Latch Interrupt Enable ON/OFF</b><br>
0 = Rising latch interrupt Disabled.<br>
1 = Rising latch interrupt Enabled.<br>
When Enabled, if Capture detects Channel 0 has rising transition, Capture issues an Interrupt.<br>
</div></td></tr><tr><td>
[1]</td><td>CFL_IE0</td><td><div style="word-wrap: break-word;"><b>Channel 0 Falling Latch Interrupt Enable ON/OFF</b><br>
0 = Falling latch interrupt Disabled.<br>
1 = Falling latch interrupt Enabled.<br>
When Enabled, if Capture detects Channel 0 has falling transition, Capture issues an Interrupt.<br>
</div></td></tr><tr><td>
[8]</td><td>CRL_IE1</td><td><div style="word-wrap: break-word;"><b>Channel 1 Rising Latch Interrupt Enable</b><br>
0 = Rising latch interrupt Disabled.<br>
1 = Rising latch interrupt Enabled.<br>
When Enabled, if Capture detects Channel 1 has rising transition, Capture issues an Interrupt.<br>
</div></td></tr><tr><td>
[9]</td><td>CFL_IE1</td><td><div style="word-wrap: break-word;"><b>Channel 1 Falling Latch Interrupt Enable</b><br>
0 = Falling latch interrupt Disabled.<br>
1 = Falling latch interrupt Enabled.<br>
When Enabled, if Capture detects Channel 1 has falling transition, Capture issues an Interrupt.<br>
</div></td></tr><tr><td>
[16]</td><td>CRL_IE2</td><td><div style="word-wrap: break-word;"><b>Channel 2 Rising Latch Interrupt Enable ON/OFF</b><br>
0 = Rising latch interrupt Disabled.<br>
1 = Rising latch interrupt Enabled.<br>
When Enabled, if Capture detects Channel 2 has rising transition, Capture issues an Interrupt.<br>
</div></td></tr><tr><td>
[17]</td><td>CFL_IE2</td><td><div style="word-wrap: break-word;"><b>Channel 2 Falling Latch Interrupt Enable ON/OFF</b><br>
0 = Falling latch interrupt Disabled.<br>
1 = Falling latch interrupt Enabled.<br>
When Enabled, if Capture detects Channel 2 has falling transition, Capture issues an Interrupt.<br>
</div></td></tr><tr><td>
[24]</td><td>CRL_IE3</td><td><div style="word-wrap: break-word;"><b>Channel 3 Rising Latch Interrupt Enable ON/OFF</b><br>
0 = Rising latch interrupt Disabled.<br>
1 = Rising latch interrupt Enabled.<br>
When Enabled, if Capture detects Channel 3 has rising transition, Capture issues an Interrupt.<br>
</div></td></tr><tr><td>
[25]</td><td>CFL_IE3</td><td><div style="word-wrap: break-word;"><b>Channel 3 Falling Latch Interrupt Enable ON/OFF</b><br>
0 = Falling latch interrupt Disabled.<br>
1 = Falling latch interrupt Enabled.<br>
When Enabled, if Capture detects Channel 3 has falling transition, Capture issues an Interrupt.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CAPINTEN;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CAPINTSTS</font><br><p> <font size="2">
Offset: 0x5C  Capture Interrupt Indication Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>CAPIF0</td><td><div style="word-wrap: break-word;"><b>Capture0 Interrupt Indication Flag</b><br>
If channel 0 rising latch interrupt is enabled (CRL_IE0 =1), a rising transition occurs at input channel 0 will result in CAPIF0 to high; Similarly, a falling transition will cause CAPIF0 to be set high if channel 0 falling latch interrupt is enabled (CFL_IE0 =1).<br>
This flag is cleared by software with a write 1 on it.<br>
</div></td></tr><tr><td>
[1]</td><td>CRLI0</td><td><div style="word-wrap: break-word;"><b>PWM_CRL0 Latched Indicator Bit</b><br>
When input channel 0 has a rising transition, PWM0_CRL0 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing 1 to it.<br>
</div></td></tr><tr><td>
[2]</td><td>CFLRI0</td><td><div style="word-wrap: break-word;"><b>PWM_CFL0 Latched Indicator Bit</b><br>
When input channel 0 has a falling transition, PWM0_CFL0 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing 1 to it.<br>
</div></td></tr><tr><td>
[3]</td><td>CAPOVR0</td><td><div style="word-wrap: break-word;"><b>Capture Rising Flag Over Run For Channel 0</b><br>
This flag indicate CRL0 update faster than software reading it when it is set<br>
This bit will be cleared automatically when user clears CRLI0 bit 1 of PWM_CAPINTSTS.<br>
</div></td></tr><tr><td>
[4]</td><td>CAPOVF0</td><td><div style="word-wrap: break-word;"><b>Capture Falling Flag Over Run For Channel 0</b><br>
This flag indicate CFL0 update faster than software read it when it is set<br>
This bit will be cleared automatically when user clear CFLI0 bit 2 of PWM_CAPINTSTS<br>
</div></td></tr><tr><td>
[8]</td><td>CAPIF1</td><td><div style="word-wrap: break-word;"><b>Capture1 Interrupt Indication Flag</b><br>
If channel 1 rising latch interrupt is enabled (CRL_IE1 =1), a rising transition occurs at input channel 1 will result in CAPIF1 to high; Similarly, a falling transition will cause CAPIF1 to be set high if channel 1 falling latch interrupt is enabled (CFL_IE1 =1).<br>
This flag is cleared by software with a write 1 on it.<br>
</div></td></tr><tr><td>
[9]</td><td>CRLI1</td><td><div style="word-wrap: break-word;"><b>PWM_CRL1 Latched Indicator Bit</b><br>
When input channel 1 has a rising transition, PWM_CRL1 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing 1 to it.<br>
</div></td></tr><tr><td>
[10]</td><td>CFLI1</td><td><div style="word-wrap: break-word;"><b>PWM_CFL1 Latched Indicator Bit</b><br>
When input channel 1 has a falling transition, PWM_CFL1 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing 1 to it.<br>
</div></td></tr><tr><td>
[11]</td><td>CAPOVR1</td><td><div style="word-wrap: break-word;"><b>Capture Rising Flag Over Run For Channel 1</b><br>
This flag indicate CRL1 update faster than software reading it when it is set<br>
This bit will be cleared automatically when user clear CRLI1 bit 9 of PWM_CAPINTSTS<br>
</div></td></tr><tr><td>
[12]</td><td>CAPOVF1</td><td><div style="word-wrap: break-word;"><b>Capture Falling Flag Over Run For Channel 1</b><br>
This flag indicate CFL1 update faster than software reading it when it is set<br>
This bit will be cleared automatically when user clear CFLI1 bit 10 of PWM_CAPINTSTS<br>
</div></td></tr><tr><td>
[16]</td><td>CAPIF2</td><td><div style="word-wrap: break-word;"><b>Capture2 Interrupt Indication Flag</b><br>
If channel 2 rising latch interrupt is enabled (CRL_IE2=1), a rising transition occurs at input channel 2 will result in CAPIF2 to high; Similarly, a falling transition will cause CAPIF2 to be set high if channel 2 falling latch interrupt is enabled (CFL_IE2=1).<br>
This flag is cleared by software with a write 1 on it.<br>
</div></td></tr><tr><td>
[17]</td><td>CRLI2</td><td><div style="word-wrap: break-word;"><b>PWM_CRL2 Latched Indicator Bit</b><br>
When input channel 2 has a rising transition, PWM0_CRL2 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing 1 to it.<br>
</div></td></tr><tr><td>
[18]</td><td>CFLI2</td><td><div style="word-wrap: break-word;"><b>PWM_CFL2 Latched Indicator Bit</b><br>
When input channel 2 has a falling transition, PWM0_CFL2 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing 1 to it.<br>
</div></td></tr><tr><td>
[19]</td><td>CAPOVR2</td><td><div style="word-wrap: break-word;"><b>Capture Rising Flag Over Run For Channel 2</b><br>
This flag indicate CRL2 update faster than software reading it when it is set<br>
This bit will be cleared automatically when user clear CRLI2 bit 17 of PWM_CAPINTSTS<br>
</div></td></tr><tr><td>
[20]</td><td>CAPOVF2</td><td><div style="word-wrap: break-word;"><b>Capture Falling Flag Over Run For Channel 2</b><br>
This flag indicate CFL2 update faster than software reading it when it is set<br>
This bit will be cleared automatically when user clear CFLI2 bit 18 of PWM_CAPINTSTS<br>
</div></td></tr><tr><td>
[24]</td><td>CAPIF3</td><td><div style="word-wrap: break-word;"><b>Capture3 Interrupt Indication Flag</b><br>
If channel 3 rising latch interrupt is enabled (CRL_IE3 =1), a rising transition occurs at input channel 3 will result in CAPIF3 to high; Similarly, a falling transition will cause CAPIF3 to be set high if channel 3 falling latch interrupt is enabled (CFL_IE3=1).<br>
This flag is cleared by software with a write 1 on it.<br>
</div></td></tr><tr><td>
[25]</td><td>CRLI3</td><td><div style="word-wrap: break-word;"><b>PWM_CRL3 Latched Indicator Bit</b><br>
When input channel 3 has a rising transition, PWM_CRL3 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing 1 to it.<br>
</div></td></tr><tr><td>
[26]</td><td>CFLI3</td><td><div style="word-wrap: break-word;"><b>PWM_CFL3 Latched Indicator Bit</b><br>
When input channel 3 has a falling transition, PWM_CFL3 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing 1 to it.<br>
</div></td></tr><tr><td>
[27]</td><td>CAPOVR3</td><td><div style="word-wrap: break-word;"><b>Capture Rising Flag Over Run For Channel 3</b><br>
This flag indicate CRL3update faster than software reading it when it is set<br>
This bit will be cleared automatically when user clear CRLI3 bit 25 of PWM_CAPINTSTS<br>
</div></td></tr><tr><td>
[28]</td><td>CAPOVF3</td><td><div style="word-wrap: break-word;"><b>Capture Falling Flag Over Run For Channel 3</b><br>
This flag indicate CFL3 update faster than software reading it when it is set<br>
This bit will be cleared automatically when user clear CFLI3 bit 26 of PWM_CAPINTSTS<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CAPINTSTS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CRL0</font><br><p> <font size="2">
Offset: 0x60  Capture Rising Latch Register (Channel 0)</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CRL15_0</td><td><div style="word-wrap: break-word;"><b>Capture Rising Latch Register</b><br>
Latch the PWM counter when Channel 0/1/2/3 has rising transition.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CRL31_16</td><td><div style="word-wrap: break-word;"><b>Upper Half Word Of 32-Bit Capture Data When Cascade Enabled</b><br>
When cascade is enabled for capture channel 0, 2,the original 16 bit counter extend to 32 bit, and capture result CRL0 and CRL2 are also extend to 32 bit.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CRL0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CFL0</font><br><p> <font size="2">
Offset: 0x64  Capture Falling Latch Register (Channel 0)</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CFL15_0</td><td><div style="word-wrap: break-word;"><b>Capture Falling Latch Register</b><br>
Latch the PWM counter when Channel 01/2/3 has Falling transition.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CFL31_16</td><td><div style="word-wrap: break-word;"><b>Upper Half Word Of 32-Bit Capture Data When Cascade Enabled</b><br>
When cascade is enabled for capture channel 0, 2, the original 16 bit counter extend to 32 bit, and capture result CFL0 and CFL2 are also extend to 32 bit.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CFL0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CRL1</font><br><p> <font size="2">
Offset: 0x68  Capture Rising Latch Register (Channel 1)</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CRL15_0</td><td><div style="word-wrap: break-word;"><b>Capture Rising Latch Register</b><br>
Latch the PWM counter when Channel 0/1/2/3 has rising transition.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CRL31_16</td><td><div style="word-wrap: break-word;"><b>Upper Half Word Of 32-Bit Capture Data When Cascade Enabled</b><br>
When cascade is enabled for capture channel 0, 2,the original 16 bit counter extend to 32 bit, and capture result CRL0 and CRL2 are also extend to 32 bit.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CRL1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CFL1</font><br><p> <font size="2">
Offset: 0x6C  Capture Falling Latch Register (Channel 1)</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CFL15_0</td><td><div style="word-wrap: break-word;"><b>Capture Falling Latch Register</b><br>
Latch the PWM counter when Channel 01/2/3 has Falling transition.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CFL31_16</td><td><div style="word-wrap: break-word;"><b>Upper Half Word Of 32-Bit Capture Data When Cascade Enabled</b><br>
When cascade is enabled for capture channel 0, 2, the original 16 bit counter extend to 32 bit, and capture result CFL0 and CFL2 are also extend to 32 bit.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CFL1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CRL2</font><br><p> <font size="2">
Offset: 0x70  Capture Rising Latch Register (Channel 2)</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CRL15_0</td><td><div style="word-wrap: break-word;"><b>Capture Rising Latch Register</b><br>
Latch the PWM counter when Channel 0/1/2/3 has rising transition.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CRL31_16</td><td><div style="word-wrap: break-word;"><b>Upper Half Word Of 32-Bit Capture Data When Cascade Enabled</b><br>
When cascade is enabled for capture channel 0, 2,the original 16 bit counter extend to 32 bit, and capture result CRL0 and CRL2 are also extend to 32 bit.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CRL2;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CFL2</font><br><p> <font size="2">
Offset: 0x74  Capture Falling Latch Register (Channel 2)</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CFL15_0</td><td><div style="word-wrap: break-word;"><b>Capture Falling Latch Register</b><br>
Latch the PWM counter when Channel 01/2/3 has Falling transition.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CFL31_16</td><td><div style="word-wrap: break-word;"><b>Upper Half Word Of 32-Bit Capture Data When Cascade Enabled</b><br>
When cascade is enabled for capture channel 0, 2, the original 16 bit counter extend to 32 bit, and capture result CFL0 and CFL2 are also extend to 32 bit.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CFL2;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CRL3</font><br><p> <font size="2">
Offset: 0x78  Capture Rising Latch Register (Channel 3)</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CRL15_0</td><td><div style="word-wrap: break-word;"><b>Capture Rising Latch Register</b><br>
Latch the PWM counter when Channel 0/1/2/3 has rising transition.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CRL31_16</td><td><div style="word-wrap: break-word;"><b>Upper Half Word Of 32-Bit Capture Data When Cascade Enabled</b><br>
When cascade is enabled for capture channel 0, 2,the original 16 bit counter extend to 32 bit, and capture result CRL0 and CRL2 are also extend to 32 bit.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CRL3;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CFL3</font><br><p> <font size="2">
Offset: 0x7C  Capture Falling Latch Register (Channel 3)</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>CFL15_0</td><td><div style="word-wrap: break-word;"><b>Capture Falling Latch Register</b><br>
Latch the PWM counter when Channel 01/2/3 has Falling transition.<br>
</div></td></tr><tr><td>
[31:16]</td><td>CFL31_16</td><td><div style="word-wrap: break-word;"><b>Upper Half Word Of 32-Bit Capture Data When Cascade Enabled</b><br>
When cascade is enabled for capture channel 0, 2, the original 16 bit counter extend to 32 bit, and capture result CFL0 and CFL2 are also extend to 32 bit.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t CFL3;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PDMACH0</font><br><p> <font size="2">
Offset: 0x80  PDMA channel 0 captured data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>Captureddata7_0</td><td><div style="word-wrap: break-word;"><b>PDMACH0</b><br>
When CH01CASK is disabled, it is the capturing value(CFL0/CRL0) for channel 0<br>
When CH01CASK is enabled, It is the for the first byte of 32 bit capturing data for channel 0<br>
</div></td></tr><tr><td>
[15:8]</td><td>Captureddata15_8</td><td><div style="word-wrap: break-word;"><b>PDMACH0</b><br>
When CH01CASK is disabled, it is the capturing value(CFL0/CRL0) for channel 0<br>
When CH01CASK is enabled, It is the second byte of 32 bit capturing data for channel 0<br>
</div></td></tr><tr><td>
[23:16]</td><td>Captureddata23_16</td><td><div style="word-wrap: break-word;"><b>PDMACH0</b><br>
When CH01CASK is disabled, this byte is 0<br>
When CH01CASK is enabled, It is the third byte of 32 bit capturing data for channel 0<br>
</div></td></tr><tr><td>
[31:24]</td><td>Captureddata31_24</td><td><div style="word-wrap: break-word;"><b>PDMACH0</b><br>
When CH01CASK is disabled, this byte is 0<br>
When CH01CASK is enabled, It is the 4th byte of 32 bit capturing data for channel 0<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t PDMACH0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PDMACH2</font><br><p> <font size="2">
Offset: 0x84  PDMA channel 2 captured data</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>Captureddata7_0</td><td><div style="word-wrap: break-word;"><b>PDMACH0</b><br>
When CH23CASK is disabled, it is the capturing value(CFL2/CRL2) for channel 2<br>
When CH23CASK is enabled, It is the for the first byte of 32 bit capturing data for channel 2<br>
</div></td></tr><tr><td>
[15:8]</td><td>Captureddata15_8</td><td><div style="word-wrap: break-word;"><b>PDMACH0</b><br>
When CH23CASK is disabled, it is the capturing value(CFL2/CRL2) for channel 2<br>
When CH23CASK is enabled, It is the second byte of 32 bit capturing data for channel 2<br>
</div></td></tr><tr><td>
[23:16]</td><td>Captureddata23_16</td><td><div style="word-wrap: break-word;"><b>PDMACH0</b><br>
When CH23CASK is disabled, this byte is 0<br>
When CH23CASK is enabled, It is the third byte of 32 bit capturing data for channel 2<br>
</div></td></tr><tr><td>
[31:24]</td><td>Captureddata31_24</td><td><div style="word-wrap: break-word;"><b>PDMACH0</b><br>
When CH23CASK is disabled, this byte is 0<br>
When CH23CASK is enabled, It is the 4th byte of 32 bit capturing data for channel 2<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t PDMACH2;

} PWM_T;

/**
    @addtogroup PWM_CONST PWM Bit Field Definition
    Constant Definitions for PWM Controller
@{ */

#define PWM_PRES_CP01_Pos                (0)                                               /*!< PWM_T::PRES: CP01 Position                */
#define PWM_PRES_CP01_Msk                (0xfful << PWM_PRES_CP01_Pos)                     /*!< PWM_T::PRES: CP01 Mask                    */

#define PWM_PRES_CP23_Pos                (8)                                               /*!< PWM_T::PRES: CP23 Position                */
#define PWM_PRES_CP23_Msk                (0xfful << PWM_PRES_CP23_Pos)                     /*!< PWM_T::PRES: CP23 Mask                    */

#define PWM_PRES_DZ01_Pos                (16)                                              /*!< PWM_T::PRES: DZ01 Position                */
#define PWM_PRES_DZ01_Msk                (0xfful << PWM_PRES_DZ01_Pos)                     /*!< PWM_T::PRES: DZ01 Mask                    */

#define PWM_PRES_DZ23_Pos                (24)                                              /*!< PWM_T::PRES: DZ23 Position                */
#define PWM_PRES_DZ23_Msk                (0xfful << PWM_PRES_DZ23_Pos)                     /*!< PWM_T::PRES: DZ23 Mask                    */

#define PWM_CLKSEL_CLKSEL0_Pos           (0)                                               /*!< PWM_T::CLKSEL: CLKSEL0 Position           */
#define PWM_CLKSEL_CLKSEL0_Msk           (0x7ul << PWM_CLKSEL_CLKSEL0_Pos)                 /*!< PWM_T::CLKSEL: CLKSEL0 Mask               */

#define PWM_CLKSEL_CLKSEL1_Pos           (4)                                               /*!< PWM_T::CLKSEL: CLKSEL1 Position           */
#define PWM_CLKSEL_CLKSEL1_Msk           (0x7ul << PWM_CLKSEL_CLKSEL1_Pos)                 /*!< PWM_T::CLKSEL: CLKSEL1 Mask               */

#define PWM_CLKSEL_CLKSEL2_Pos           (8)                                               /*!< PWM_T::CLKSEL: CLKSEL2 Position           */
#define PWM_CLKSEL_CLKSEL2_Msk           (0x7ul << PWM_CLKSEL_CLKSEL2_Pos)                 /*!< PWM_T::CLKSEL: CLKSEL2 Mask               */

#define PWM_CLKSEL_CLKSEL3_Pos           (12)                                              /*!< PWM_T::CLKSEL: CLKSEL3 Position           */
#define PWM_CLKSEL_CLKSEL3_Msk           (0x7ul << PWM_CLKSEL_CLKSEL3_Pos)                 /*!< PWM_T::CLKSEL: CLKSEL3 Mask               */

#define PWM_CTL_CH0EN_Pos                (0)                                               /*!< PWM_T::CTL: CH0EN Position                */
#define PWM_CTL_CH0EN_Msk                (0x1ul << PWM_CTL_CH0EN_Pos)                      /*!< PWM_T::CTL: CH0EN Mask                    */

#define PWM_CTL_CH0INV_Pos               (2)                                               /*!< PWM_T::CTL: CH0INV Position               */
#define PWM_CTL_CH0INV_Msk               (0x1ul << PWM_CTL_CH0INV_Pos)                     /*!< PWM_T::CTL: CH0INV Mask                   */

#define PWM_CTL_CH0MOD_Pos               (3)                                               /*!< PWM_T::CTL: CH0MOD Position               */
#define PWM_CTL_CH0MOD_Msk               (0x1ul << PWM_CTL_CH0MOD_Pos)                     /*!< PWM_T::CTL: CH0MOD Mask                   */

#define PWM_CTL_DZEN01_Pos               (4)                                               /*!< PWM_T::CTL: DZEN01 Position               */
#define PWM_CTL_DZEN01_Msk               (0x1ul << PWM_CTL_DZEN01_Pos)                     /*!< PWM_T::CTL: DZEN01 Mask                   */

#define PWM_CTL_DZEN23_Pos               (5)                                               /*!< PWM_T::CTL: DZEN23 Position               */
#define PWM_CTL_DZEN23_Msk               (0x1ul << PWM_CTL_DZEN23_Pos)                     /*!< PWM_T::CTL: DZEN23 Mask                   */

#define PWM_CTL_CH1EN_Pos                (8)                                               /*!< PWM_T::CTL: CH1EN Position                */
#define PWM_CTL_CH1EN_Msk                (0x1ul << PWM_CTL_CH1EN_Pos)                      /*!< PWM_T::CTL: CH1EN Mask                    */

#define PWM_CTL_CH1INV_Pos               (10)                                              /*!< PWM_T::CTL: CH1INV Position               */
#define PWM_CTL_CH1INV_Msk               (0x1ul << PWM_CTL_CH1INV_Pos)                     /*!< PWM_T::CTL: CH1INV Mask                   */

#define PWM_CTL_CH1MOD_Pos               (11)                                              /*!< PWM_T::CTL: CH1MOD Position               */
#define PWM_CTL_CH1MOD_Msk               (0x1ul << PWM_CTL_CH1MOD_Pos)                     /*!< PWM_T::CTL: CH1MOD Mask                   */

#define PWM_CTL_CH2EN_Pos                (16)                                              /*!< PWM_T::CTL: CH2EN Position                */
#define PWM_CTL_CH2EN_Msk                (0x1ul << PWM_CTL_CH2EN_Pos)                      /*!< PWM_T::CTL: CH2EN Mask                    */

#define PWM_CTL_CH2INV_Pos               (18)                                              /*!< PWM_T::CTL: CH2INV Position               */
#define PWM_CTL_CH2INV_Msk               (0x1ul << PWM_CTL_CH2INV_Pos)                     /*!< PWM_T::CTL: CH2INV Mask                   */

#define PWM_CTL_CH2MOD_Pos               (19)                                              /*!< PWM_T::CTL: CH2MOD Position               */
#define PWM_CTL_CH2MOD_Msk               (0x1ul << PWM_CTL_CH2MOD_Pos)                     /*!< PWM_T::CTL: CH2MOD Mask                   */

#define PWM_CTL_CH3EN_Pos                (24)                                              /*!< PWM_T::CTL: CH3EN Position                */
#define PWM_CTL_CH3EN_Msk                (0x1ul << PWM_CTL_CH3EN_Pos)                      /*!< PWM_T::CTL: CH3EN Mask                    */

#define PWM_CTL_CH3INV_Pos               (26)                                              /*!< PWM_T::CTL: CH3INV Position               */
#define PWM_CTL_CH3INV_Msk               (0x1ul << PWM_CTL_CH3INV_Pos)                     /*!< PWM_T::CTL: CH3INV Mask                   */

#define PWM_CTL_CH3MOD_Pos               (27)                                              /*!< PWM_T::CTL: CH3MOD Position               */
#define PWM_CTL_CH3MOD_Msk               (0x1ul << PWM_CTL_CH3MOD_Pos)                     /*!< PWM_T::CTL: CH3MOD Mask                   */

#define PWM_INTEN_TMIE0_Pos              (0)                                               /*!< PWM_T::INTEN: TMIE0 Position              */
#define PWM_INTEN_TMIE0_Msk              (0x1ul << PWM_INTEN_TMIE0_Pos)                    /*!< PWM_T::INTEN: TMIE0 Mask                  */

#define PWM_INTEN_TMIE1_Pos              (1)                                               /*!< PWM_T::INTEN: TMIE1 Position              */
#define PWM_INTEN_TMIE1_Msk              (0x1ul << PWM_INTEN_TMIE1_Pos)                    /*!< PWM_T::INTEN: TMIE1 Mask                  */

#define PWM_INTEN_TMIE2_Pos              (2)                                               /*!< PWM_T::INTEN: TMIE2 Position              */
#define PWM_INTEN_TMIE2_Msk              (0x1ul << PWM_INTEN_TMIE2_Pos)                    /*!< PWM_T::INTEN: TMIE2 Mask                  */

#define PWM_INTEN_TMIE3_Pos              (3)                                               /*!< PWM_T::INTEN: TMIE3 Position              */
#define PWM_INTEN_TMIE3_Msk              (0x1ul << PWM_INTEN_TMIE3_Pos)                    /*!< PWM_T::INTEN: TMIE3 Mask                  */

#define PWM_INTSTS_TMINT0_Pos            (0)                                               /*!< PWM_T::INTSTS: TMINT0 Position            */
#define PWM_INTSTS_TMINT0_Msk            (0x1ul << PWM_INTSTS_TMINT0_Pos)                  /*!< PWM_T::INTSTS: TMINT0 Mask                */

#define PWM_INTSTS_TMINT1_Pos            (1)                                               /*!< PWM_T::INTSTS: TMINT1 Position            */
#define PWM_INTSTS_TMINT1_Msk            (0x1ul << PWM_INTSTS_TMINT1_Pos)                  /*!< PWM_T::INTSTS: TMINT1 Mask                */

#define PWM_INTSTS_TMINT2_Pos            (2)                                               /*!< PWM_T::INTSTS: TMINT2 Position            */
#define PWM_INTSTS_TMINT2_Msk            (0x1ul << PWM_INTSTS_TMINT2_Pos)                  /*!< PWM_T::INTSTS: TMINT2 Mask                */

#define PWM_INTSTS_TMINT3_Pos            (3)                                               /*!< PWM_T::INTSTS: TMINT3 Position            */
#define PWM_INTSTS_TMINT3_Msk            (0x1ul << PWM_INTSTS_TMINT3_Pos)                  /*!< PWM_T::INTSTS: TMINT3 Mask                */

#define PWM_INTSTS_DUTY0SYNC_Pos         (4)                                               /*!< PWM_T::INTSTS: DUTY0SYNC Position     */
#define PWM_INTSTS_DUTY0SYNC_Msk         (0x1ul << PWM_INTSTS_DUTY0SYNC_Pos)               /*!< PWM_T::INTSTS: DUTY0SYNC Mask         */

#define PWM_INTSTS_PRESSYNC_Pos          (8)                                               /*!< PWM_T::INTSTS: PRESSYNC Position      */
#define PWM_INTSTS_PRESSYNC_Msk          (0x1ul << PWM_INTSTS_PRESSYNC_Pos)                /*!< PWM_T::INTSTS: PRESSYNC Mask          */

#define PWM_OE_CH0_OE_Pos                (0)                                               /*!< PWM_T::OE: CH0_OE Position                */
#define PWM_OE_CH0_OE_Msk                (0x1ul << PWM_OE_CH0_OE_Pos)                      /*!< PWM_T::OE: CH0_OE Mask                    */

#define PWM_OE_CH1_OE_Pos                (1)                                               /*!< PWM_T::OE: CH1_OE Position                */
#define PWM_OE_CH1_OE_Msk                (0x1ul << PWM_OE_CH1_OE_Pos)                      /*!< PWM_T::OE: CH1_OE Mask                    */

#define PWM_OE_CH2_OE_Pos                (2)                                               /*!< PWM_T::OE: CH2_OE Position                */
#define PWM_OE_CH2_OE_Msk                (0x1ul << PWM_OE_CH2_OE_Pos)                      /*!< PWM_T::OE: CH2_OE Mask                    */

#define PWM_OE_CH3_OE_Pos                (3)                                               /*!< PWM_T::OE: CH3_OE Position                */
#define PWM_OE_CH3_OE_Msk                (0x1ul << PWM_OE_CH3_OE_Pos)                      /*!< PWM_T::OE: CH3_OE Mask                    */

#define PWM_DUTY_CN_Pos                 (0)                                               /*!< PWM_T::DUTY0: CN Position                 */
#define PWM_DUTY_CN_Msk                 (0xfffful << PWM_DUTY_CN_Pos)                    /*!< PWM_T::DUTY0: CN Mask                     */

#define PWM_DUTY_CM_Pos                 (16)                                              /*!< PWM_T::DUTY0: CM Position                 */
#define PWM_DUTY_CM_Msk                 (0xfffful << PWM_DUTY_CM_Pos)                    /*!< PWM_T::DUTY0: CM Mask                     */

#define PWM_DATA0_PWMx_DATAy15_0_Pos     (0)                                               /*!< PWM_T::DATA0: PWMx_DATAy15_0 Position     */
#define PWM_DATA0_PWMx_DATAy15_0_Msk     (0xfffful << PWM_DATA0_PWMx_DATAy15_0_Pos)        /*!< PWM_T::DATA0: PWMx_DATAy15_0 Mask         */

#define PWM_DATA0_PWMx_DATAy30_16_Pos    (16)                                              /*!< PWM_T::DATA0: PWMx_DATAy30_16 Position    */
#define PWM_DATA0_PWMx_DATAy30_16_Msk    (0x7ffful << PWM_DATA0_PWMx_DATAy30_16_Pos)       /*!< PWM_T::DATA0: PWMx_DATAy30_16 Mask        */

#define PWM_DATA0_sync_Pos               (31)                                              /*!< PWM_T::DATA0: sync Position               */
#define PWM_DATA0_sync_Msk               (0x1ul << PWM_DATA0_sync_Pos)                     /*!< PWM_T::DATA0: sync Mask                   */

#define PWM_DUTY1_CN_Pos                 (0)                                               /*!< PWM_T::DUTY1: CN Position                 */
#define PWM_DUTY1_CN_Msk                 (0xfffful << PWM_DUTY1_CN_Pos)                    /*!< PWM_T::DUTY1: CN Mask                     */

#define PWM_DUTY1_CM_Pos                 (16)                                              /*!< PWM_T::DUTY1: CM Position                 */
#define PWM_DUTY1_CM_Msk                 (0xfffful << PWM_DUTY1_CM_Pos)                    /*!< PWM_T::DUTY1: CM Mask                     */

#define PWM_DATA1_PWMx_DATAy15_0_Pos     (0)                                               /*!< PWM_T::DATA1: PWMx_DATAy15_0 Position     */
#define PWM_DATA1_PWMx_DATAy15_0_Msk     (0xfffful << PWM_DATA1_PWMx_DATAy15_0_Pos)        /*!< PWM_T::DATA1: PWMx_DATAy15_0 Mask         */

#define PWM_DATA1_PWMx_DATAy30_16_Pos    (16)                                              /*!< PWM_T::DATA1: PWMx_DATAy30_16 Position    */
#define PWM_DATA1_PWMx_DATAy30_16_Msk    (0x7ffful << PWM_DATA1_PWMx_DATAy30_16_Pos)       /*!< PWM_T::DATA1: PWMx_DATAy30_16 Mask        */

#define PWM_DATA1_sync_Pos               (31)                                              /*!< PWM_T::DATA1: sync Position               */
#define PWM_DATA1_sync_Msk               (0x1ul << PWM_DATA1_sync_Pos)                     /*!< PWM_T::DATA1: sync Mask                   */

#define PWM_DUTY2_CN_Pos                 (0)                                               /*!< PWM_T::DUTY2: CN Position                 */
#define PWM_DUTY2_CN_Msk                 (0xfffful << PWM_DUTY2_CN_Pos)                    /*!< PWM_T::DUTY2: CN Mask                     */

#define PWM_DUTY2_CM_Pos                 (16)                                              /*!< PWM_T::DUTY2: CM Position                 */
#define PWM_DUTY2_CM_Msk                 (0xfffful << PWM_DUTY2_CM_Pos)                    /*!< PWM_T::DUTY2: CM Mask                     */

#define PWM_DATA2_PWMx_DATAy15_0_Pos     (0)                                               /*!< PWM_T::DATA2: PWMx_DATAy15_0 Position     */
#define PWM_DATA2_PWMx_DATAy15_0_Msk     (0xfffful << PWM_DATA2_PWMx_DATAy15_0_Pos)        /*!< PWM_T::DATA2: PWMx_DATAy15_0 Mask         */

#define PWM_DATA2_PWMx_DATAy30_16_Pos    (16)                                              /*!< PWM_T::DATA2: PWMx_DATAy30_16 Position    */
#define PWM_DATA2_PWMx_DATAy30_16_Msk    (0x7ffful << PWM_DATA2_PWMx_DATAy30_16_Pos)       /*!< PWM_T::DATA2: PWMx_DATAy30_16 Mask        */

#define PWM_DATA2_sync_Pos               (31)                                              /*!< PWM_T::DATA2: sync Position               */
#define PWM_DATA2_sync_Msk               (0x1ul << PWM_DATA2_sync_Pos)                     /*!< PWM_T::DATA2: sync Mask                   */

#define PWM_DUTY3_CN_Pos                 (0)                                               /*!< PWM_T::DUTY3: CN Position                 */
#define PWM_DUTY3_CN_Msk                 (0xfffful << PWM_DUTY3_CN_Pos)                    /*!< PWM_T::DUTY3: CN Mask                     */

#define PWM_DUTY3_CM_Pos                 (16)                                              /*!< PWM_T::DUTY3: CM Position                 */
#define PWM_DUTY3_CM_Msk                 (0xfffful << PWM_DUTY3_CM_Pos)                    /*!< PWM_T::DUTY3: CM Mask                     */

#define PWM_DATA3_PWMx_DATAy15_0_Pos     (0)                                               /*!< PWM_T::DATA3: PWMx_DATAy15_0 Position     */
#define PWM_DATA3_PWMx_DATAy15_0_Msk     (0xfffful << PWM_DATA3_PWMx_DATAy15_0_Pos)        /*!< PWM_T::DATA3: PWMx_DATAy15_0 Mask         */

#define PWM_DATA3_PWMx_DATAy30_16_Pos    (16)                                              /*!< PWM_T::DATA3: PWMx_DATAy30_16 Position    */
#define PWM_DATA3_PWMx_DATAy30_16_Msk    (0x7ffful << PWM_DATA3_PWMx_DATAy30_16_Pos)       /*!< PWM_T::DATA3: PWMx_DATAy30_16 Mask        */

#define PWM_DATA3_sync_Pos               (31)                                              /*!< PWM_T::DATA3: sync Position               */
#define PWM_DATA3_sync_Msk               (0x1ul << PWM_DATA3_sync_Pos)                     /*!< PWM_T::DATA3: sync Mask                   */

#define PWM_CAPCTL_INV0_Pos              (0)                                               /*!< PWM_T::CAPCTL: INV0 Position              */
#define PWM_CAPCTL_INV0_Msk              (0x1ul << PWM_CAPCTL_INV0_Pos)                    /*!< PWM_T::CAPCTL: INV0 Mask                  */

#define PWM_CAPCTL_CAPCH0EN_Pos          (1)                                               /*!< PWM_T::CAPCTL: CAPCH0EN Position          */
#define PWM_CAPCTL_CAPCH0EN_Msk          (0x1ul << PWM_CAPCTL_CAPCH0EN_Pos)                /*!< PWM_T::CAPCTL: CAPCH0EN Mask              */

#define PWM_CAPCTL_CAPCH0PADEN_Pos       (2)                                               /*!< PWM_T::CAPCTL: CAPCH0PADEN Position       */
#define PWM_CAPCTL_CAPCH0PADEN_Msk       (0x1ul << PWM_CAPCTL_CAPCH0PADEN_Pos)             /*!< PWM_T::CAPCTL: CAPCH0PADEN Mask           */

#define PWM_CAPCTL_CH0PDMAEN_Pos         (3)                                               /*!< PWM_T::CAPCTL: CH0PDMAEN Position         */
#define PWM_CAPCTL_CH0PDMAEN_Msk         (0x1ul << PWM_CAPCTL_CH0PDMAEN_Pos)               /*!< PWM_T::CAPCTL: CH0PDMAEN Mask             */

#define PWM_CAPCTL_PDMACAPMOD0_Pos       (4)                                               /*!< PWM_T::CAPCTL: PDMACAPMOD0 Position       */
#define PWM_CAPCTL_PDMACAPMOD0_Msk       (0x3ul << PWM_CAPCTL_PDMACAPMOD0_Pos)             /*!< PWM_T::CAPCTL: PDMACAPMOD0 Mask           */

#define PWM_CAPCTL_CAPRELOADREN0_Pos     (6)                                               /*!< PWM_T::CAPCTL: CAPRELOADREN0 Position     */
#define PWM_CAPCTL_CAPRELOADREN0_Msk     (0x1ul << PWM_CAPCTL_CAPRELOADREN0_Pos)           /*!< PWM_T::CAPCTL: CAPRELOADREN0 Mask         */

#define PWM_CAPCTL_CAPRELOADFEN0_Pos     (7)                                               /*!< PWM_T::CAPCTL: CAPRELOADFEN0 Position     */
#define PWM_CAPCTL_CAPRELOADFEN0_Msk     (0x1ul << PWM_CAPCTL_CAPRELOADFEN0_Pos)           /*!< PWM_T::CAPCTL: CAPRELOADFEN0 Mask         */

#define PWM_CAPCTL_INV1_Pos              (8)                                               /*!< PWM_T::CAPCTL: INV1 Position              */
#define PWM_CAPCTL_INV1_Msk              (0x1ul << PWM_CAPCTL_INV1_Pos)                    /*!< PWM_T::CAPCTL: INV1 Mask                  */

#define PWM_CAPCTL_CAPCH1EN_Pos          (9)                                               /*!< PWM_T::CAPCTL: CAPCH1EN Position          */
#define PWM_CAPCTL_CAPCH1EN_Msk          (0x1ul << PWM_CAPCTL_CAPCH1EN_Pos)                /*!< PWM_T::CAPCTL: CAPCH1EN Mask              */

#define PWM_CAPCTL_CAPCH1PADEN_Pos       (10)                                              /*!< PWM_T::CAPCTL: CAPCH1PADEN Position       */
#define PWM_CAPCTL_CAPCH1PADEN_Msk       (0x1ul << PWM_CAPCTL_CAPCH1PADEN_Pos)             /*!< PWM_T::CAPCTL: CAPCH1PADEN Mask           */

#define PWM_CAPCTL_CH0RFORDER_Pos        (12)                                              /*!< PWM_T::CAPCTL: CH0RFORDER Position        */
#define PWM_CAPCTL_CH0RFORDER_Msk        (0x1ul << PWM_CAPCTL_CH0RFORDER_Pos)              /*!< PWM_T::CAPCTL: CH0RFORDER Mask            */

#define PWM_CAPCTL_CH01CASK_Pos          (13)                                              /*!< PWM_T::CAPCTL: CH01CASK Position          */
#define PWM_CAPCTL_CH01CASK_Msk          (0x1ul << PWM_CAPCTL_CH01CASK_Pos)                /*!< PWM_T::CAPCTL: CH01CASK Mask              */

#define PWM_CAPCTL_CAPRELOADREN1_Pos     (14)                                              /*!< PWM_T::CAPCTL: CAPRELOADREN1 Position     */
#define PWM_CAPCTL_CAPRELOADREN1_Msk     (0x1ul << PWM_CAPCTL_CAPRELOADREN1_Pos)           /*!< PWM_T::CAPCTL: CAPRELOADREN1 Mask         */

#define PWM_CAPCTL_CAPRELOADFEN1_Pos     (15)                                              /*!< PWM_T::CAPCTL: CAPRELOADFEN1 Position     */
#define PWM_CAPCTL_CAPRELOADFEN1_Msk     (0x1ul << PWM_CAPCTL_CAPRELOADFEN1_Pos)           /*!< PWM_T::CAPCTL: CAPRELOADFEN1 Mask         */

#define PWM_CAPCTL_INV2_Pos              (16)                                              /*!< PWM_T::CAPCTL: INV2 Position              */
#define PWM_CAPCTL_INV2_Msk              (0x1ul << PWM_CAPCTL_INV2_Pos)                    /*!< PWM_T::CAPCTL: INV2 Mask                  */

#define PWM_CAPCTL_CAPCH2EN_Pos          (17)                                              /*!< PWM_T::CAPCTL: CAPCH2EN Position          */
#define PWM_CAPCTL_CAPCH2EN_Msk          (0x1ul << PWM_CAPCTL_CAPCH2EN_Pos)                /*!< PWM_T::CAPCTL: CAPCH2EN Mask              */

#define PWM_CAPCTL_CAPCH2PADEN_Pos       (18)                                              /*!< PWM_T::CAPCTL: CAPCH2PADEN Position       */
#define PWM_CAPCTL_CAPCH2PADEN_Msk       (0x1ul << PWM_CAPCTL_CAPCH2PADEN_Pos)             /*!< PWM_T::CAPCTL: CAPCH2PADEN Mask           */

#define PWM_CAPCTL_CH2PDMAEN_Pos         (19)                                              /*!< PWM_T::CAPCTL: CH2PDMAEN Position         */
#define PWM_CAPCTL_CH2PDMAEN_Msk         (0x1ul << PWM_CAPCTL_CH2PDMAEN_Pos)               /*!< PWM_T::CAPCTL: CH2PDMAEN Mask             */

#define PWM_CAPCTL_PDMACAPMOD2_Pos       (20)                                              /*!< PWM_T::CAPCTL: PDMACAPMOD2 Position       */
#define PWM_CAPCTL_PDMACAPMOD2_Msk       (0x3ul << PWM_CAPCTL_PDMACAPMOD2_Pos)             /*!< PWM_T::CAPCTL: PDMACAPMOD2 Mask           */

#define PWM_CAPCTL_CAPRELOADREN2_Pos     (22)                                              /*!< PWM_T::CAPCTL: CAPRELOADREN2 Position     */
#define PWM_CAPCTL_CAPRELOADREN2_Msk     (0x1ul << PWM_CAPCTL_CAPRELOADREN2_Pos)           /*!< PWM_T::CAPCTL: CAPRELOADREN2 Mask         */

#define PWM_CAPCTL_CAPRELOADFEN2_Pos     (23)                                              /*!< PWM_T::CAPCTL: CAPRELOADFEN2 Position     */
#define PWM_CAPCTL_CAPRELOADFEN2_Msk     (0x1ul << PWM_CAPCTL_CAPRELOADFEN2_Pos)           /*!< PWM_T::CAPCTL: CAPRELOADFEN2 Mask         */

#define PWM_CAPCTL_INV3_Pos              (24)                                              /*!< PWM_T::CAPCTL: INV3 Position              */
#define PWM_CAPCTL_INV3_Msk              (0x1ul << PWM_CAPCTL_INV3_Pos)                    /*!< PWM_T::CAPCTL: INV3 Mask                  */

#define PWM_CAPCTL_CAPCH3EN_Pos          (25)                                              /*!< PWM_T::CAPCTL: CAPCH3EN Position          */
#define PWM_CAPCTL_CAPCH3EN_Msk          (0x1ul << PWM_CAPCTL_CAPCH3EN_Pos)                /*!< PWM_T::CAPCTL: CAPCH3EN Mask              */

#define PWM_CAPCTL_CAPCH3PADEN_Pos       (26)                                              /*!< PWM_T::CAPCTL: CAPCH3PADEN Position       */
#define PWM_CAPCTL_CAPCH3PADEN_Msk       (0x1ul << PWM_CAPCTL_CAPCH3PADEN_Pos)             /*!< PWM_T::CAPCTL: CAPCH3PADEN Mask           */

#define PWM_CAPCTL_CH2RFORDER_Pos        (28)                                              /*!< PWM_T::CAPCTL: CH2RFORDER Position        */
#define PWM_CAPCTL_CH2RFORDER_Msk        (0x1ul << PWM_CAPCTL_CH2RFORDER_Pos)              /*!< PWM_T::CAPCTL: CH2RFORDER Mask            */

#define PWM_CAPCTL_CH23CASK_Pos          (29)                                              /*!< PWM_T::CAPCTL: CH23CASK Position          */
#define PWM_CAPCTL_CH23CASK_Msk          (0x1ul << PWM_CAPCTL_CH23CASK_Pos)                /*!< PWM_T::CAPCTL: CH23CASK Mask              */

#define PWM_CAPCTL_CAPRELOADREN3_Pos     (30)                                              /*!< PWM_T::CAPCTL: CAPRELOADREN3 Position     */
#define PWM_CAPCTL_CAPRELOADREN3_Msk     (0x1ul << PWM_CAPCTL_CAPRELOADREN3_Pos)           /*!< PWM_T::CAPCTL: CAPRELOADREN3 Mask         */

#define PWM_CAPCTL_CAPRELOADFEN3_Pos     (31)                                              /*!< PWM_T::CAPCTL: CAPRELOADFEN3 Position     */
#define PWM_CAPCTL_CAPRELOADFEN3_Msk     (0x1ul << PWM_CAPCTL_CAPRELOADFEN3_Pos)           /*!< PWM_T::CAPCTL: CAPRELOADFEN3 Mask         */

#define PWM_CAPINTEN_CRL_IE0_Pos         (0)                                               /*!< PWM_T::CAPINTEN: CRL_IE0 Position         */
#define PWM_CAPINTEN_CRL_IE0_Msk         (0x1ul << PWM_CAPINTEN_CRL_IE0_Pos)               /*!< PWM_T::CAPINTEN: CRL_IE0 Mask             */

#define PWM_CAPINTEN_CFL_IE0_Pos         (1)                                               /*!< PWM_T::CAPINTEN: CFL_IE0 Position         */
#define PWM_CAPINTEN_CFL_IE0_Msk         (0x1ul << PWM_CAPINTEN_CFL_IE0_Pos)               /*!< PWM_T::CAPINTEN: CFL_IE0 Mask             */

#define PWM_CAPINTEN_CRL_IE1_Pos         (8)                                               /*!< PWM_T::CAPINTEN: CRL_IE1 Position         */
#define PWM_CAPINTEN_CRL_IE1_Msk         (0x1ul << PWM_CAPINTEN_CRL_IE1_Pos)               /*!< PWM_T::CAPINTEN: CRL_IE1 Mask             */

#define PWM_CAPINTEN_CFL_IE1_Pos         (9)                                               /*!< PWM_T::CAPINTEN: CFL_IE1 Position         */
#define PWM_CAPINTEN_CFL_IE1_Msk         (0x1ul << PWM_CAPINTEN_CFL_IE1_Pos)               /*!< PWM_T::CAPINTEN: CFL_IE1 Mask             */

#define PWM_CAPINTEN_CRL_IE2_Pos         (16)                                              /*!< PWM_T::CAPINTEN: CRL_IE2 Position         */
#define PWM_CAPINTEN_CRL_IE2_Msk         (0x1ul << PWM_CAPINTEN_CRL_IE2_Pos)               /*!< PWM_T::CAPINTEN: CRL_IE2 Mask             */

#define PWM_CAPINTEN_CFL_IE2_Pos         (17)                                              /*!< PWM_T::CAPINTEN: CFL_IE2 Position         */
#define PWM_CAPINTEN_CFL_IE2_Msk         (0x1ul << PWM_CAPINTEN_CFL_IE2_Pos)               /*!< PWM_T::CAPINTEN: CFL_IE2 Mask             */

#define PWM_CAPINTEN_CRL_IE3_Pos         (24)                                              /*!< PWM_T::CAPINTEN: CRL_IE3 Position         */
#define PWM_CAPINTEN_CRL_IE3_Msk         (0x1ul << PWM_CAPINTEN_CRL_IE3_Pos)               /*!< PWM_T::CAPINTEN: CRL_IE3 Mask             */

#define PWM_CAPINTEN_CFL_IE3_Pos         (25)                                              /*!< PWM_T::CAPINTEN: CFL_IE3 Position         */
#define PWM_CAPINTEN_CFL_IE3_Msk         (0x1ul << PWM_CAPINTEN_CFL_IE3_Pos)               /*!< PWM_T::CAPINTEN: CFL_IE3 Mask             */

#define PWM_CAPINTSTS_CAPIF0_Pos         (0)                                               /*!< PWM_T::CAPINTSTS: CAPIF0 Position         */
#define PWM_CAPINTSTS_CAPIF0_Msk         (0x1ul << PWM_CAPINTSTS_CAPIF0_Pos)               /*!< PWM_T::CAPINTSTS: CAPIF0 Mask             */

#define PWM_CAPINTSTS_CRLI0_Pos          (1)                                               /*!< PWM_T::CAPINTSTS: CRLI0 Position          */
#define PWM_CAPINTSTS_CRLI0_Msk          (0x1ul << PWM_CAPINTSTS_CRLI0_Pos)                /*!< PWM_T::CAPINTSTS: CRLI0 Mask              */

#define PWM_CAPINTSTS_CFLRI0_Pos         (2)                                               /*!< PWM_T::CAPINTSTS: CFLRI0 Position         */
#define PWM_CAPINTSTS_CFLRI0_Msk         (0x1ul << PWM_CAPINTSTS_CFLRI0_Pos)               /*!< PWM_T::CAPINTSTS: CFLRI0 Mask             */

#define PWM_CAPINTSTS_CAPOVR0_Pos        (3)                                               /*!< PWM_T::CAPINTSTS: CAPOVR0 Position        */
#define PWM_CAPINTSTS_CAPOVR0_Msk        (0x1ul << PWM_CAPINTSTS_CAPOVR0_Pos)              /*!< PWM_T::CAPINTSTS: CAPOVR0 Mask            */

#define PWM_CAPINTSTS_CAPOVF0_Pos        (4)                                               /*!< PWM_T::CAPINTSTS: CAPOVF0 Position        */
#define PWM_CAPINTSTS_CAPOVF0_Msk        (0x1ul << PWM_CAPINTSTS_CAPOVF0_Pos)              /*!< PWM_T::CAPINTSTS: CAPOVF0 Mask            */

#define PWM_CAPINTSTS_CAPIF1_Pos         (8)                                               /*!< PWM_T::CAPINTSTS: CAPIF1 Position         */
#define PWM_CAPINTSTS_CAPIF1_Msk         (0x1ul << PWM_CAPINTSTS_CAPIF1_Pos)               /*!< PWM_T::CAPINTSTS: CAPIF1 Mask             */

#define PWM_CAPINTSTS_CRLI1_Pos          (9)                                               /*!< PWM_T::CAPINTSTS: CRLI1 Position          */
#define PWM_CAPINTSTS_CRLI1_Msk          (0x1ul << PWM_CAPINTSTS_CRLI1_Pos)                /*!< PWM_T::CAPINTSTS: CRLI1 Mask              */

#define PWM_CAPINTSTS_CFLI1_Pos          (10)                                              /*!< PWM_T::CAPINTSTS: CFLI1 Position          */
#define PWM_CAPINTSTS_CFLI1_Msk          (0x1ul << PWM_CAPINTSTS_CFLI1_Pos)                /*!< PWM_T::CAPINTSTS: CFLI1 Mask              */

#define PWM_CAPINTSTS_CAPOVR1_Pos        (11)                                              /*!< PWM_T::CAPINTSTS: CAPOVR1 Position        */
#define PWM_CAPINTSTS_CAPOVR1_Msk        (0x1ul << PWM_CAPINTSTS_CAPOVR1_Pos)              /*!< PWM_T::CAPINTSTS: CAPOVR1 Mask            */

#define PWM_CAPINTSTS_CAPOVF1_Pos        (12)                                              /*!< PWM_T::CAPINTSTS: CAPOVF1 Position        */
#define PWM_CAPINTSTS_CAPOVF1_Msk        (0x1ul << PWM_CAPINTSTS_CAPOVF1_Pos)              /*!< PWM_T::CAPINTSTS: CAPOVF1 Mask            */

#define PWM_CAPINTSTS_CAPIF2_Pos         (16)                                              /*!< PWM_T::CAPINTSTS: CAPIF2 Position         */
#define PWM_CAPINTSTS_CAPIF2_Msk         (0x1ul << PWM_CAPINTSTS_CAPIF2_Pos)               /*!< PWM_T::CAPINTSTS: CAPIF2 Mask             */

#define PWM_CAPINTSTS_CRLI2_Pos          (17)                                              /*!< PWM_T::CAPINTSTS: CRLI2 Position          */
#define PWM_CAPINTSTS_CRLI2_Msk          (0x1ul << PWM_CAPINTSTS_CRLI2_Pos)                /*!< PWM_T::CAPINTSTS: CRLI2 Mask              */

#define PWM_CAPINTSTS_CFLI2_Pos          (18)                                              /*!< PWM_T::CAPINTSTS: CFLI2 Position          */
#define PWM_CAPINTSTS_CFLI2_Msk          (0x1ul << PWM_CAPINTSTS_CFLI2_Pos)                /*!< PWM_T::CAPINTSTS: CFLI2 Mask              */

#define PWM_CAPINTSTS_CAPOVR2_Pos        (19)                                              /*!< PWM_T::CAPINTSTS: CAPOVR2 Position        */
#define PWM_CAPINTSTS_CAPOVR2_Msk        (0x1ul << PWM_CAPINTSTS_CAPOVR2_Pos)              /*!< PWM_T::CAPINTSTS: CAPOVR2 Mask            */

#define PWM_CAPINTSTS_CAPOVF2_Pos        (20)                                              /*!< PWM_T::CAPINTSTS: CAPOVF2 Position        */
#define PWM_CAPINTSTS_CAPOVF2_Msk        (0x1ul << PWM_CAPINTSTS_CAPOVF2_Pos)              /*!< PWM_T::CAPINTSTS: CAPOVF2 Mask            */

#define PWM_CAPINTSTS_CAPIF3_Pos         (24)                                              /*!< PWM_T::CAPINTSTS: CAPIF3 Position         */
#define PWM_CAPINTSTS_CAPIF3_Msk         (0x1ul << PWM_CAPINTSTS_CAPIF3_Pos)               /*!< PWM_T::CAPINTSTS: CAPIF3 Mask             */

#define PWM_CAPINTSTS_CRLI3_Pos          (25)                                              /*!< PWM_T::CAPINTSTS: CRLI3 Position          */
#define PWM_CAPINTSTS_CRLI3_Msk          (0x1ul << PWM_CAPINTSTS_CRLI3_Pos)                /*!< PWM_T::CAPINTSTS: CRLI3 Mask              */

#define PWM_CAPINTSTS_CFLI3_Pos          (26)                                              /*!< PWM_T::CAPINTSTS: CFLI3 Position          */
#define PWM_CAPINTSTS_CFLI3_Msk          (0x1ul << PWM_CAPINTSTS_CFLI3_Pos)                /*!< PWM_T::CAPINTSTS: CFLI3 Mask              */

#define PWM_CAPINTSTS_CAPOVR3_Pos        (27)                                              /*!< PWM_T::CAPINTSTS: CAPOVR3 Position        */
#define PWM_CAPINTSTS_CAPOVR3_Msk        (0x1ul << PWM_CAPINTSTS_CAPOVR3_Pos)              /*!< PWM_T::CAPINTSTS: CAPOVR3 Mask            */

#define PWM_CAPINTSTS_CAPOVF3_Pos        (28)                                              /*!< PWM_T::CAPINTSTS: CAPOVF3 Position        */
#define PWM_CAPINTSTS_CAPOVF3_Msk        (0x1ul << PWM_CAPINTSTS_CAPOVF3_Pos)              /*!< PWM_T::CAPINTSTS: CAPOVF3 Mask            */

#define PWM_CRL0_CRL15_0_Pos             (0)                                               /*!< PWM_T::CRL0: CRL15_0 Position             */
#define PWM_CRL0_CRL15_0_Msk             (0xfffful << PWM_CRL0_CRL15_0_Pos)                /*!< PWM_T::CRL0: CRL15_0 Mask                 */

#define PWM_CRL0_CRL31_16_Pos            (16)                                              /*!< PWM_T::CRL0: CRL31_16 Position            */
#define PWM_CRL0_CRL31_16_Msk            (0xfffful << PWM_CRL0_CRL31_16_Pos)               /*!< PWM_T::CRL0: CRL31_16 Mask                */

#define PWM_CFL0_CFL15_0_Pos             (0)                                               /*!< PWM_T::CFL0: CFL15_0 Position             */
#define PWM_CFL0_CFL15_0_Msk             (0xfffful << PWM_CFL0_CFL15_0_Pos)                /*!< PWM_T::CFL0: CFL15_0 Mask                 */

#define PWM_CFL0_CFL31_16_Pos            (16)                                              /*!< PWM_T::CFL0: CFL31_16 Position            */
#define PWM_CFL0_CFL31_16_Msk            (0xfffful << PWM_CFL0_CFL31_16_Pos)               /*!< PWM_T::CFL0: CFL31_16 Mask                */

#define PWM_CRL1_CRL15_0_Pos             (0)                                               /*!< PWM_T::CRL1: CRL15_0 Position             */
#define PWM_CRL1_CRL15_0_Msk             (0xfffful << PWM_CRL1_CRL15_0_Pos)                /*!< PWM_T::CRL1: CRL15_0 Mask                 */

#define PWM_CRL1_CRL31_16_Pos            (16)                                              /*!< PWM_T::CRL1: CRL31_16 Position            */
#define PWM_CRL1_CRL31_16_Msk            (0xfffful << PWM_CRL1_CRL31_16_Pos)               /*!< PWM_T::CRL1: CRL31_16 Mask                */

#define PWM_CFL1_CFL15_0_Pos             (0)                                               /*!< PWM_T::CFL1: CFL15_0 Position             */
#define PWM_CFL1_CFL15_0_Msk             (0xfffful << PWM_CFL1_CFL15_0_Pos)                /*!< PWM_T::CFL1: CFL15_0 Mask                 */

#define PWM_CFL1_CFL31_16_Pos            (16)                                              /*!< PWM_T::CFL1: CFL31_16 Position            */
#define PWM_CFL1_CFL31_16_Msk            (0xfffful << PWM_CFL1_CFL31_16_Pos)               /*!< PWM_T::CFL1: CFL31_16 Mask                */

#define PWM_CRL2_CRL15_0_Pos             (0)                                               /*!< PWM_T::CRL2: CRL15_0 Position             */
#define PWM_CRL2_CRL15_0_Msk             (0xfffful << PWM_CRL2_CRL15_0_Pos)                /*!< PWM_T::CRL2: CRL15_0 Mask                 */

#define PWM_CRL2_CRL31_16_Pos            (16)                                              /*!< PWM_T::CRL2: CRL31_16 Position            */
#define PWM_CRL2_CRL31_16_Msk            (0xfffful << PWM_CRL2_CRL31_16_Pos)               /*!< PWM_T::CRL2: CRL31_16 Mask                */

#define PWM_CFL2_CFL15_0_Pos             (0)                                               /*!< PWM_T::CFL2: CFL15_0 Position             */
#define PWM_CFL2_CFL15_0_Msk             (0xfffful << PWM_CFL2_CFL15_0_Pos)                /*!< PWM_T::CFL2: CFL15_0 Mask                 */

#define PWM_CFL2_CFL31_16_Pos            (16)                                              /*!< PWM_T::CFL2: CFL31_16 Position            */
#define PWM_CFL2_CFL31_16_Msk            (0xfffful << PWM_CFL2_CFL31_16_Pos)               /*!< PWM_T::CFL2: CFL31_16 Mask                */

#define PWM_CRL3_CRL15_0_Pos             (0)                                               /*!< PWM_T::CRL3: CRL15_0 Position             */
#define PWM_CRL3_CRL15_0_Msk             (0xfffful << PWM_CRL3_CRL15_0_Pos)                /*!< PWM_T::CRL3: CRL15_0 Mask                 */

#define PWM_CRL3_CRL31_16_Pos            (16)                                              /*!< PWM_T::CRL3: CRL31_16 Position            */
#define PWM_CRL3_CRL31_16_Msk            (0xfffful << PWM_CRL3_CRL31_16_Pos)               /*!< PWM_T::CRL3: CRL31_16 Mask                */

#define PWM_CFL3_CFL15_0_Pos             (0)                                               /*!< PWM_T::CFL3: CFL15_0 Position             */
#define PWM_CFL3_CFL15_0_Msk             (0xfffful << PWM_CFL3_CFL15_0_Pos)                /*!< PWM_T::CFL3: CFL15_0 Mask                 */

#define PWM_CFL3_CFL31_16_Pos            (16)                                              /*!< PWM_T::CFL3: CFL31_16 Position            */
#define PWM_CFL3_CFL31_16_Msk            (0xfffful << PWM_CFL3_CFL31_16_Pos)               /*!< PWM_T::CFL3: CFL31_16 Mask                */

#define PWM_PDMACH0_Captureddata7_0_Pos  (0)                                               /*!< PWM_T::PDMACH0: Captureddata7_0 Position  */
#define PWM_PDMACH0_Captureddata7_0_Msk  (0xfful << PWM_PDMACH0_Captureddata7_0_Pos)       /*!< PWM_T::PDMACH0: Captureddata7_0 Mask      */

#define PWM_PDMACH0_Captureddata15_8_Pos (8)                                               /*!< PWM_T::PDMACH0: Captureddata15_8 Position */
#define PWM_PDMACH0_Captureddata15_8_Msk (0xfful << PWM_PDMACH0_Captureddata15_8_Pos)      /*!< PWM_T::PDMACH0: Captureddata15_8 Mask     */

#define PWM_PDMACH0_Captureddata23_16_Pos (16)                                             /*!< PWM_T::PDMACH0: Captureddata23_16 Position*/
#define PWM_PDMACH0_Captureddata23_16_Msk (0xfful << PWM_PDMACH0_Captureddata23_16_Pos)    /*!< PWM_T::PDMACH0: Captureddata23_16 Mask    */

#define PWM_PDMACH0_Captureddata31_24_Pos (24)                                             /*!< PWM_T::PDMACH0: Captureddata31_24 Position*/
#define PWM_PDMACH0_Captureddata31_24_Msk (0xfful << PWM_PDMACH0_Captureddata31_24_Pos)    /*!< PWM_T::PDMACH0: Captureddata31_24 Mask    */

#define PWM_PDMACH2_Captureddata7_0_Pos  (0)                                               /*!< PWM_T::PDMACH2: Captureddata7_0 Position  */
#define PWM_PDMACH2_Captureddata7_0_Msk  (0xfful << PWM_PDMACH2_Captureddata7_0_Pos)       /*!< PWM_T::PDMACH2: Captureddata7_0 Mask      */

#define PWM_PDMACH2_Captureddata15_8_Pos (8)                                               /*!< PWM_T::PDMACH2: Captureddata15_8 Position */
#define PWM_PDMACH2_Captureddata15_8_Msk (0xfful << PWM_PDMACH2_Captureddata15_8_Pos)      /*!< PWM_T::PDMACH2: Captureddata15_8 Mask     */

#define PWM_PDMACH2_Captureddata23_16_Pos (16)                                             /*!< PWM_T::PDMACH2: Captureddata23_16 Position*/
#define PWM_PDMACH2_Captureddata23_16_Msk (0xfful << PWM_PDMACH2_Captureddata23_16_Pos)    /*!< PWM_T::PDMACH2: Captureddata23_16 Mask    */

#define PWM_PDMACH2_Captureddata31_24_Pos (24)                                             /*!< PWM_T::PDMACH2: Captureddata31_24 Position*/
#define PWM_PDMACH2_Captureddata31_24_Msk (0xfful << PWM_PDMACH2_Captureddata31_24_Pos)    /*!< PWM_T::PDMACH2: Captureddata31_24 Mask    */

/**@}*/ /* PWM_CONST */
/**@}*/ /* end of PWM register group */


/*---------------------- Real Time Clock Controller -------------------------*/
/**
    @addtogroup RTC Real Time Clock Controller(RTC)
    Memory Mapped Structure for RTC Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">INIR</font><br><p> <font size="2">
Offset: 0x00  RTC Initiation Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>ACTIVE</td><td><div style="word-wrap: break-word;"><b>RTC Active Status (Read Only)</b><br>
0 = RTC is at reset state.<br>
1 = RTC is at normal active state.<br>
</div></td></tr><tr><td>
[31:1]</td><td>INIR</td><td><div style="word-wrap: break-word;"><b>RTC Initiation (Write Only)</b><br>
When RTC block is powered on, RTC is at reset state.<br>
User has to write a number (0x a5eb1357) to INIR to make RTC leaving reset state.<br>
Once the INIR is written as 0xa5eb1357, the RTC will be in un-reset state permanently.<br>
The INIR is a write-only field and read value will be always "0".<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO  uint32_t INIR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">AER</font><br><p> <font size="2">
Offset: 0x04  RTC Access Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>AER</td><td><div style="word-wrap: break-word;"><b>RTC Register Access Enable Password (Write Only)</b><br>
Enable RTC access after write 0xA965. Otherwise disable RTC access.<br>
</div></td></tr><tr><td>
[16]</td><td>ENF</td><td><div style="word-wrap: break-word;"><b>RTC Register Access Enable Flag (Read Only)</b><br>
1 = RTC register read/write Enabled.<br>
0 = RTC register read/write Disabled.<br>
This bit will be set after AER[15:0] register is load a 0xA965, and be cleared automatically 512 RTC clocks or AER[15:0] is not 0xA965.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO  uint32_t AER;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">FCR</font><br><p> <font size="2">
Offset: 0x08  RTC Frequency Compensation Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>FRACTION</td><td><div style="word-wrap: break-word;"><b>Fraction Part</b><br>
Formula = (fraction part of detected value) x 64.<br>
Note: Digit in FCR must be expressed as hexadecimal number.<br>
</div></td></tr><tr><td>
[11:8]</td><td>INTEGER</td><td><div style="word-wrap: break-word;"><b>Integer Part</b><br>
0000 = 32761.<br>
0001 = 32762.<br>
0010 = 32763.<br>
0011 = 32764.<br>
0100 = 32765.<br>
0101 = 32766.<br>
0110 = 32767.<br>
0111 = 32768.<br>
1000 = 32769.<br>
1001 = 32770.<br>
1010 = 32771.<br>
1011 = 32772.<br>
1100 = 32773.<br>
1101 = 32774.<br>
1110 = 32775.<br>
1111 = 32776.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t FCR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TLR</font><br><p> <font size="2">
Offset: 0x0C  Time Loading Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>1SEC</td><td><div style="word-wrap: break-word;"><b>1 Sec Time Digit (0~9)</b><br>
</div></td></tr><tr><td>
[6:4]</td><td>10SEC</td><td><div style="word-wrap: break-word;"><b>10 Sec Time Digit (0~5)</b><br>
</div></td></tr><tr><td>
[11:8]</td><td>1MIN</td><td><div style="word-wrap: break-word;"><b>1 Min Time Digit (0~9)</b><br>
</div></td></tr><tr><td>
[14:12]</td><td>10MIN</td><td><div style="word-wrap: break-word;"><b>10 Min Time Digit (0~5)</b><br>
</div></td></tr><tr><td>
[19:16]</td><td>1HR</td><td><div style="word-wrap: break-word;"><b>1 Hour Time Digit (0~9)</b><br>
</div></td></tr><tr><td>
[21:20]</td><td>10HR</td><td><div style="word-wrap: break-word;"><b>10 Hour Time Digit (0~2)</b><br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TLR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CLR</font><br><p> <font size="2">
Offset: 0x10  Calendar Loading Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>1DAY</td><td><div style="word-wrap: break-word;"><b>1 Day Calendar Digit (0~9)</b><br>
</div></td></tr><tr><td>
[5:4]</td><td>10DAY</td><td><div style="word-wrap: break-word;"><b>10 Day Calendar Digit (0~3)</b><br>
</div></td></tr><tr><td>
[11:8]</td><td>1MON</td><td><div style="word-wrap: break-word;"><b>1 Month Calendar Digit (0~9)</b><br>
</div></td></tr><tr><td>
[12]</td><td>10MON</td><td><div style="word-wrap: break-word;"><b>10 Month Calendar Digit (0~1)</b><br>
</div></td></tr><tr><td>
[19:16]</td><td>1YEAR</td><td><div style="word-wrap: break-word;"><b>1 Year Calendar Digit (0~9)</b><br>
</div></td></tr><tr><td>
[23:20]</td><td>10YEAR</td><td><div style="word-wrap: break-word;"><b>10 Year Calendar Digit (0~9)</b><br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CLR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TSSR</font><br><p> <font size="2">
Offset: 0x14  Time Scale Selection Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>24hr_12hr</td><td><div style="word-wrap: break-word;"><b>24-Hour / 12-Hour Mode Selection</b><br>
It indicates that TLR and TAR are in 24-hour mode or 12-hour mode<br>
0 = select 12-hour time scale with AM and PM indication.<br>
1 = select 24-hour time scale.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TSSR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DWR</font><br><p> <font size="2">
Offset: 0x18  Day of the Week Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>DWR</td><td><div style="word-wrap: break-word;"><b>Day Of The Week Register</b><br>
000 = Sunday.<br>
001 = Monday.<br>
010 = Tuesday.<br>
011 = Wednesday.<br>
100 = Thursday.<br>
101 = Friday.<br>
110 = Saturday.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DWR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TAR</font><br><p> <font size="2">
Offset: 0x1C  Time Alarm Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>1SEC</td><td><div style="word-wrap: break-word;"><b>1 Sec Time Digit of Alarm Setting (0~9)</b><br>
</div></td></tr><tr><td>
[6:4]</td><td>10SEC</td><td><div style="word-wrap: break-word;"><b>10 Sec Time Digit of Alarm Setting (0~5)</b><br>
</div></td></tr><tr><td>
[11:8]</td><td>1MIN</td><td><div style="word-wrap: break-word;"><b>1 Min Time Digit of Alarm Setting (0~9)</b><br>
</div></td></tr><tr><td>
[14:12]</td><td>10MIN</td><td><div style="word-wrap: break-word;"><b>10 Min Time Digit of Alarm Setting (0~5)</b><br>
</div></td></tr><tr><td>
[19:16]</td><td>1HR</td><td><div style="word-wrap: break-word;"><b>1 Hour Time Digit of Alarm Setting (0~9)</b><br>
</div></td></tr><tr><td>
[21:20]</td><td>10HR</td><td><div style="word-wrap: break-word;"><b>10 Hour Time Digit of Alarm Setting (0~2)</b><br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TAR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CAR</font><br><p> <font size="2">
Offset: 0x20  Calendar Alarm Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>1DAY</td><td><div style="word-wrap: break-word;"><b>1 Day Calendar Digit of Alarm Setting (0~9)</b><br>
</div></td></tr><tr><td>
[5:4]</td><td>10DAY</td><td><div style="word-wrap: break-word;"><b>10 Day Calendar Digit of Alarm Setting (0~3)</b><br>
</div></td></tr><tr><td>
[11:8]</td><td>1MON</td><td><div style="word-wrap: break-word;"><b>1 Month Calendar Digit of Alarm Setting (0~9)</b><br>
</div></td></tr><tr><td>
[12]</td><td>10MON</td><td><div style="word-wrap: break-word;"><b>10 Month Calendar Digit of Alarm Setting (0~1)</b><br>
</div></td></tr><tr><td>
[19:16]</td><td>1YEAR</td><td><div style="word-wrap: break-word;"><b>1 Year Calendar Digit of Alarm Setting (0~9)</b><br>
</div></td></tr><tr><td>
[23:20]</td><td>10YEAR</td><td><div style="word-wrap: break-word;"><b>10 Year Calendar Digit of Alarm Setting (0~9)</b><br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CAR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">LIR</font><br><p> <font size="2">
Offset: 0x24  Leap Year Indicator Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>LIR</td><td><div style="word-wrap: break-word;"><b>Leap Year Indication REGISTER (Read Only)</b><br>
0 = This year is not a leap year.<br>
1 = This year is leap year.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t LIR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RIER</font><br><p> <font size="2">
Offset: 0x28  RTC Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>AIER</td><td><div style="word-wrap: break-word;"><b>Alarm Interrupt Enable</b><br>
0 = RTC Alarm Interrupt is disabled.<br>
1 = RTC Alarm Interrupt is enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>TIER</td><td><div style="word-wrap: break-word;"><b>Time Tick Interrupt And Wake-Up By Tick Enable</b><br>
0 = RTC Time Tick Interrupt is disabled.<br>
1 = RTC Time Tick Interrupt is enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>SNOOPIER</td><td><div style="word-wrap: break-word;"><b>Snooper Pin Event Detection Interrupt Enable</b><br>
0 = Snooper Pin Event Detection Interrupt is disabled.<br>
1 = Snooper Pin Event Detection Interrupt is enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t RIER;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RIIR</font><br><p> <font size="2">
Offset: 0x2C  RTC Interrupt Indication Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>AIS</td><td><div style="word-wrap: break-word;"><b>RTC Alarm Interrupt Status</b><br>
RTC unit will set AIS to high once the RTC real time counters TLR and CLR reach the alarm setting time registers TAR and CAR.<br>
When this bit is set and AIER is also high, RTC will generate an interrupt to CPU.<br>
This bit is cleared by writing "1" to it through software.<br>
0 = RCT Alarm Interrupt condition never occurred.<br>
1 = RTC Alarm Interrupt is requested if RIER.AIER=1.<br>
</div></td></tr><tr><td>
[1]</td><td>TIS</td><td><div style="word-wrap: break-word;"><b>RTC Time Tick Interrupt Status</b><br>
RTC unit will set TIF to high periodically in the period selected by TTR[2:0].<br>
When this bit is set and TIER is also high, RTC will generate an interrupt to CPU.<br>
This bit is cleared by writing "1" to it through software.<br>
0 = RCT Time Tick Interrupt condition never occurred.<br>
1 = RTC Time Tick Interrupt is requested.<br>
</div></td></tr><tr><td>
[2]</td><td>SNOOPIF</td><td><div style="word-wrap: break-word;"><b>Snooper Pin Event Detection Interrupt Flag</b><br>
When SNOOPEN is high and an event defined by SNOOPEDGE detected in snooper pin, this flag will be set.<br>
While this bit is set and SNOOPIER is also high, RTC will generate an interrupt to CPU.<br>
Write "1" to clear this bit to "0".<br>
0 = Snooper pin event defined by SNOOPEDGE never detected.<br>
1 = Snooper pin event defined by SNOOPEDGE detected.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t RIIR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TTR</font><br><p> <font size="2">
Offset: 0x30  RTC Time Tick Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>TTR</td><td><div style="word-wrap: break-word;"><b>Time Tick Register</b><br>
The RTC time tick period for Periodic Time Tick Interrupt request.<br>
000 = 1 tick/second.<br>
001 = 1/2 tick/second.<br>
010 = 1/4 tick/second.<br>
011 = 1/8 tick/second.<br>
100 = 1/16 tick/second.<br>
101 = 1/32 tick/second.<br>
110 = 1/64 tick/second.<br>
111 = 1/128 tick/second.<br>
Note: This register can be read back after the RTC is active by AER.<br>
</div></td></tr><tr><td>
[3]</td><td>TWKE</td><td><div style="word-wrap: break-word;"><b>RTC Timer Wake-Up CPU Function Enable Bit</b><br>
If TWKE is set before CPU enters power-down mode, when a RTC Time Tick, CPU will be wakened up by RTC unit.<br>
0 = Time Tick wake-up CPU function Disabled.<br>
1 = Wake-up function Enabled so that CPU can be waken up from Power-down mode by Time Tick.<br>
Note: Tick timer setting follows the TTR description.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TTR;
    uint32_t RESERVE0[2];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SPRCTL</font><br><p> <font size="2">
Offset: 0x3C  RTC Spare Functional Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>SNOOPEN</td><td><div style="word-wrap: break-word;"><b>Snooper Pin Event Detection Enable</b><br>
This bit enables the snooper pin event detection.<br>
When this bit is set high and an event defined by SNOOPEDGE detected, the 20 spare registers will be cleared to "0" by hardware automatically.<br>
And, the SNOOPIF will also be set.<br>
In addition, RTC will also generate wake-up event to wake system up.<br>
0 = Snooper pin event detection function Disabled.<br>
1 = Snooper pin event detection function Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>SNOOPEDGE</td><td><div style="word-wrap: break-word;"><b>Snooper Active Edge Selection</b><br>
This bit defines which edge of snooper pin will generate a snooper pin detected event to clear the 20 spare registers.<br>
0 = Rising edge of snooper pin generates snooper pin detected event.<br>
1 = Falling edge of snooper pin generates snooper pin detected event.<br>
</div></td></tr><tr><td>
[7]</td><td>SPRRDY</td><td><div style="word-wrap: break-word;"><b>SPR Register Ready</b><br>
This bit indicates if the registers SPR0 ~ SPR19 are ready to read.<br>
After CPU writing registers SPR0 ~ SPR19, polling this bit to check if SP0 ~ SPR19 are updated done is necessary.<br>
This it is read only and any write to this bit won't take any effect.<br>
0 = SPR0 ~ SPR19 updating is in progress.<br>
1 = SPR0 ~ SPR19 are updated done and ready to read.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SPRCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SPR0 ~ 19</font><br><p> <font size="2">
Offset: 0x40 ~ 0x8C  RTC Spare Register 0 ~ 19</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>SPARE</td><td><div style="word-wrap: break-word;"><b>SPARE</b><br>
This field is used to store back-up information defined by software.<br>
This field will be cleared by hardware automatically once a snooper pin event is detected.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SPR[20];

} RTC_T;

/**
    @addtogroup RTC_CONST RTC Bit Field Definition
    Constant Definitions for RTC Controller
@{ */

#define RTC_INIR_ACTIVE_Pos              (0)                                               /*!< RTC_T::INIR: ACTIVE Position              */
#define RTC_INIR_ACTIVE_Msk              (0x1ul << RTC_INIR_ACTIVE_Pos)                    /*!< RTC_T::INIR: ACTIVE Mask                  */

#define RTC_INIR_INIR_Pos                (0)                                               /*!< RTC_T::INIR: INIR Position                */
#define RTC_INIR_INIR_Msk                (0xfffffffful << RTC_INIR_INIR_Pos)               /*!< RTC_T::INIR: INIR Mask                    */

#define RTC_AER_AER_Pos                  (0)                                               /*!< RTC_T::AER: AER Position                  */
#define RTC_AER_AER_Msk                  (0xfffful << RTC_AER_AER_Pos)                     /*!< RTC_T::AER: AER Mask                      */

#define RTC_AER_ENF_Pos                  (16)                                              /*!< RTC_T::AER: ENF Position                  */
#define RTC_AER_ENF_Msk                  (0x1ul << RTC_AER_ENF_Pos)                        /*!< RTC_T::AER: ENF Mask                      */

#define RTC_FCR_FRACTION_Pos             (0)                                               /*!< RTC_T::FCR: FRACTION Position             */
#define RTC_FCR_FRACTION_Msk             (0x3ful << RTC_FCR_FRACTION_Pos)                  /*!< RTC_T::FCR: FRACTION Mask                 */

#define RTC_FCR_INTEGER_Pos              (8)                                               /*!< RTC_T::FCR: INTEGER Position              */
#define RTC_FCR_INTEGER_Msk              (0xful << RTC_FCR_INTEGER_Pos)                    /*!< RTC_T::FCR: INTEGER Mask                  */

#define RTC_TLR_1SEC_Pos                 (0)                                               /*!< RTC_T::TLR: 1SEC Position                 */
#define RTC_TLR_1SEC_Msk                 (0xful << RTC_TLR_1SEC_Pos)                       /*!< RTC_T::TLR: 1SEC Mask                     */

#define RTC_TLR_10SEC_Pos                (4)                                               /*!< RTC_T::TLR: 10SEC Position                */
#define RTC_TLR_10SEC_Msk                (0x7ul << RTC_TLR_10SEC_Pos)                      /*!< RTC_T::TLR: 10SEC Mask                    */

#define RTC_TLR_1MIN_Pos                 (8)                                               /*!< RTC_T::TLR: 1MIN Position                 */
#define RTC_TLR_1MIN_Msk                 (0xful << RTC_TLR_1MIN_Pos)                       /*!< RTC_T::TLR: 1MIN Mask                     */

#define RTC_TLR_10MIN_Pos                (12)                                              /*!< RTC_T::TLR: 10MIN Position                */
#define RTC_TLR_10MIN_Msk                (0x7ul << RTC_TLR_10MIN_Pos)                      /*!< RTC_T::TLR: 10MIN Mask                    */

#define RTC_TLR_1HR_Pos                  (16)                                              /*!< RTC_T::TLR: 1HR Position                  */
#define RTC_TLR_1HR_Msk                  (0xful << RTC_TLR_1HR_Pos)                        /*!< RTC_T::TLR: 1HR Mask                      */

#define RTC_TLR_10HR_Pos                 (20)                                              /*!< RTC_T::TLR: 10HR Position                 */
#define RTC_TLR_10HR_Msk                 (0x3ul << RTC_TLR_10HR_Pos)                       /*!< RTC_T::TLR: 10HR Mask                     */

#define RTC_CLR_1DAY_Pos                 (0)                                               /*!< RTC_T::CLR: 1DAY Position                 */
#define RTC_CLR_1DAY_Msk                 (0xful << RTC_CLR_1DAY_Pos)                       /*!< RTC_T::CLR: 1DAY Mask                     */

#define RTC_CLR_10DAY_Pos                (4)                                               /*!< RTC_T::CLR: 10DAY Position                */
#define RTC_CLR_10DAY_Msk                (0x3ul << RTC_CLR_10DAY_Pos)                      /*!< RTC_T::CLR: 10DAY Mask                    */

#define RTC_CLR_1MON_Pos                 (8)                                               /*!< RTC_T::CLR: 1MON Position                 */
#define RTC_CLR_1MON_Msk                 (0xful << RTC_CLR_1MON_Pos)                       /*!< RTC_T::CLR: 1MON Mask                     */

#define RTC_CLR_10MON_Pos                (12)                                              /*!< RTC_T::CLR: 10MON Position                */
#define RTC_CLR_10MON_Msk                (0x1ul << RTC_CLR_10MON_Pos)                      /*!< RTC_T::CLR: 10MON Mask                    */

#define RTC_CLR_1YEAR_Pos                (16)                                              /*!< RTC_T::CLR: 1YEAR Position                */
#define RTC_CLR_1YEAR_Msk                (0xful << RTC_CLR_1YEAR_Pos)                      /*!< RTC_T::CLR: 1YEAR Mask                    */

#define RTC_CLR_10YEAR_Pos               (20)                                              /*!< RTC_T::CLR: 10YEAR Position               */
#define RTC_CLR_10YEAR_Msk               (0xful << RTC_CLR_10YEAR_Pos)                     /*!< RTC_T::CLR: 10YEAR Mask                   */

#define RTC_TSSR_24H_12H_Pos           (0)                                                 /*!< RTC_T::TSSR: 24hr_12hr Position           */
#define RTC_TSSR_24H_12H_Msk           (0x1ul << RTC_TSSR_24H_12H_Pos)                     /*!< RTC_T::TSSR: 24hr_12hr Mask               */

#define RTC_DWR_DWR_Pos                  (0)                                               /*!< RTC_T::DWR: DWR Position                  */
#define RTC_DWR_DWR_Msk                  (0x7ul << RTC_DWR_DWR_Pos)                        /*!< RTC_T::DWR: DWR Mask                      */

#define RTC_TAR_1SEC_Pos                 (0)                                               /*!< RTC_T::TAR: 1SEC Position                 */
#define RTC_TAR_1SEC_Msk                 (0xful << RTC_TAR_1SEC_Pos)                       /*!< RTC_T::TAR: 1SEC Mask                     */

#define RTC_TAR_10SEC_Pos                (4)                                               /*!< RTC_T::TAR: 10SEC Position                */
#define RTC_TAR_10SEC_Msk                (0x7ul << RTC_TAR_10SEC_Pos)                      /*!< RTC_T::TAR: 10SEC Mask                    */

#define RTC_TAR_1MIN_Pos                 (8)                                               /*!< RTC_T::TAR: 1MIN Position                 */
#define RTC_TAR_1MIN_Msk                 (0xful << RTC_TAR_1MIN_Pos)                       /*!< RTC_T::TAR: 1MIN Mask                     */

#define RTC_TAR_10MIN_Pos                (12)                                              /*!< RTC_T::TAR: 10MIN Position                */
#define RTC_TAR_10MIN_Msk                (0x7ul << RTC_TAR_10MIN_Pos)                      /*!< RTC_T::TAR: 10MIN Mask                    */

#define RTC_TAR_1HR_Pos                  (16)                                              /*!< RTC_T::TAR: 1HR Position                  */
#define RTC_TAR_1HR_Msk                  (0xful << RTC_TAR_1HR_Pos)                        /*!< RTC_T::TAR: 1HR Mask                      */

#define RTC_TAR_10HR_Pos                 (20)                                              /*!< RTC_T::TAR: 10HR Position                 */
#define RTC_TAR_10HR_Msk                 (0x3ul << RTC_TAR_10HR_Pos)                       /*!< RTC_T::TAR: 10HR Mask                     */

#define RTC_CAR_1DAY_Pos                 (0)                                               /*!< RTC_T::CAR: 1DAY Position                 */
#define RTC_CAR_1DAY_Msk                 (0xful << RTC_CAR_1DAY_Pos)                       /*!< RTC_T::CAR: 1DAY Mask                     */

#define RTC_CAR_10DAY_Pos                (4)                                               /*!< RTC_T::CAR: 10DAY Position                */
#define RTC_CAR_10DAY_Msk                (0x3ul << RTC_CAR_10DAY_Pos)                      /*!< RTC_T::CAR: 10DAY Mask                    */

#define RTC_CAR_1MON_Pos                 (8)                                               /*!< RTC_T::CAR: 1MON Position                 */
#define RTC_CAR_1MON_Msk                 (0xful << RTC_CAR_1MON_Pos)                       /*!< RTC_T::CAR: 1MON Mask                     */

#define RTC_CAR_10MON_Pos                (12)                                              /*!< RTC_T::CAR: 10MON Position                */
#define RTC_CAR_10MON_Msk                (0x1ul << RTC_CAR_10MON_Pos)                      /*!< RTC_T::CAR: 10MON Mask                    */

#define RTC_CAR_1YEAR_Pos                (16)                                              /*!< RTC_T::CAR: 1YEAR Position                */
#define RTC_CAR_1YEAR_Msk                (0xful << RTC_CAR_1YEAR_Pos)                      /*!< RTC_T::CAR: 1YEAR Mask                    */

#define RTC_CAR_10YEAR_Pos               (20)                                              /*!< RTC_T::CAR: 10YEAR Position               */
#define RTC_CAR_10YEAR_Msk               (0xful << RTC_CAR_10YEAR_Pos)                     /*!< RTC_T::CAR: 10YEAR Mask                   */

#define RTC_LIR_LIR_Pos                  (0)                                               /*!< RTC_T::LIR: LIR Position                  */
#define RTC_LIR_LIR_Msk                  (0x1ul << RTC_LIR_LIR_Pos)                        /*!< RTC_T::LIR: LIR Mask                      */

#define RTC_RIER_AIER_Pos                (0)                                               /*!< RTC_T::RIER: AIER Position                */
#define RTC_RIER_AIER_Msk                (0x1ul << RTC_RIER_AIER_Pos)                      /*!< RTC_T::RIER: AIER Mask                    */

#define RTC_RIER_TIER_Pos                (1)                                               /*!< RTC_T::RIER: TIER Position                */
#define RTC_RIER_TIER_Msk                (0x1ul << RTC_RIER_TIER_Pos)                      /*!< RTC_T::RIER: TIER Mask                    */

#define RTC_RIER_SNOOPIER_Pos            (2)                                               /*!< RTC_T::RIER: SNOOPIER Position            */
#define RTC_RIER_SNOOPIER_Msk            (0x1ul << RTC_RIER_SNOOPIER_Pos)                  /*!< RTC_T::RIER: SNOOPIER Mask                */

#define RTC_RIIR_AIF_Pos                 (0)                                               /*!< RTC_T::RIIR: AIF Position                 */
#define RTC_RIIR_AIF_Msk                 (0x1ul << RTC_RIIR_AIF_Pos)                       /*!< RTC_T::RIIR: AIF Mask                     */

#define RTC_RIIR_TIF_Pos                 (1)                                               /*!< RTC_T::RIIR: TIF Position                 */
#define RTC_RIIR_TIF_Msk                 (0x1ul << RTC_RIIR_TIF_Pos)                       /*!< RTC_T::RIIR: TIF Mask                     */

#define RTC_RIIR_SNOOPIF_Pos             (2)                                               /*!< RTC_T::RIIR: SNOOPIF Position             */
#define RTC_RIIR_SNOOPIF_Msk             (0x1ul << RTC_RIIR_SNOOPIF_Pos)                   /*!< RTC_T::RIIR: SNOOPIF Mask                 */

#define RTC_TTR_TTR_Pos                  (0)                                               /*!< RTC_T::TTR: TTR Position                  */
#define RTC_TTR_TTR_Msk                  (0x7ul << RTC_TTR_TTR_Pos)                        /*!< RTC_T::TTR: TTR Mask                      */

#define RTC_TTR_TWKE_Pos                 (3)                                               /*!< RTC_T::TTR: TWKE Position                 */
#define RTC_TTR_TWKE_Msk                 (0x1ul << RTC_TTR_TWKE_Pos)                       /*!< RTC_T::TTR: TWKE Mask                     */

#define RTC_SPRCTL_SNOOPEN_Pos           (0)                                               /*!< RTC_T::SPRCTL: SNOOPEN Position           */
#define RTC_SPRCTL_SNOOPEN_Msk           (0x1ul << RTC_SPRCTL_SNOOPEN_Pos)                 /*!< RTC_T::SPRCTL: SNOOPEN Mask               */

#define RTC_SPRCTL_SNOOPEDGE_Pos         (1)                                               /*!< RTC_T::SPRCTL: SNOOPEDGE Position         */
#define RTC_SPRCTL_SNOOPEDGE_Msk         (0x1ul << RTC_SPRCTL_SNOOPEDGE_Pos)               /*!< RTC_T::SPRCTL: SNOOPEDGE Mask             */

#define RTC_SPRCTL_SPRRDY_Pos            (7)                                               /*!< RTC_T::SPRCTL: SPRRDY Position            */
#define RTC_SPRCTL_SPRRDY_Msk            (0x1ul << RTC_SPRCTL_SPRRDY_Pos)                  /*!< RTC_T::SPRCTL: SPRRDY Mask                */

#define RTC_SPR0_SPARE_Pos               (0)                                               /*!< RTC_T::SPR0: SPARE Position               */
#define RTC_SPR0_SPARE_Msk               (0xfffffffful << RTC_SPR0_SPARE_Pos)              /*!< RTC_T::SPR0: SPARE Mask                   */

#define RTC_SPR1_SPARE_Pos               (0)                                               /*!< RTC_T::SPR1: SPARE Position               */
#define RTC_SPR1_SPARE_Msk               (0xfffffffful << RTC_SPR1_SPARE_Pos)              /*!< RTC_T::SPR1: SPARE Mask                   */

#define RTC_SPR2_SPARE_Pos               (0)                                               /*!< RTC_T::SPR2: SPARE Position               */
#define RTC_SPR2_SPARE_Msk               (0xfffffffful << RTC_SPR2_SPARE_Pos)              /*!< RTC_T::SPR2: SPARE Mask                   */

#define RTC_SPR3_SPARE_Pos               (0)                                               /*!< RTC_T::SPR3: SPARE Position               */
#define RTC_SPR3_SPARE_Msk               (0xfffffffful << RTC_SPR3_SPARE_Pos)              /*!< RTC_T::SPR3: SPARE Mask                   */

#define RTC_SPR4_SPARE_Pos               (0)                                               /*!< RTC_T::SPR4: SPARE Position               */
#define RTC_SPR4_SPARE_Msk               (0xfffffffful << RTC_SPR4_SPARE_Pos)              /*!< RTC_T::SPR4: SPARE Mask                   */

#define RTC_SPR5_SPARE_Pos               (0)                                               /*!< RTC_T::SPR5: SPARE Position               */
#define RTC_SPR5_SPARE_Msk               (0xfffffffful << RTC_SPR5_SPARE_Pos)              /*!< RTC_T::SPR5: SPARE Mask                   */

#define RTC_SPR6_SPARE_Pos               (0)                                               /*!< RTC_T::SPR6: SPARE Position               */
#define RTC_SPR6_SPARE_Msk               (0xfffffffful << RTC_SPR6_SPARE_Pos)              /*!< RTC_T::SPR6: SPARE Mask                   */

#define RTC_SPR7_SPARE_Pos               (0)                                               /*!< RTC_T::SPR7: SPARE Position               */
#define RTC_SPR7_SPARE_Msk               (0xfffffffful << RTC_SPR7_SPARE_Pos)              /*!< RTC_T::SPR7: SPARE Mask                   */

#define RTC_SPR8_SPARE_Pos               (0)                                               /*!< RTC_T::SPR8: SPARE Position               */
#define RTC_SPR8_SPARE_Msk               (0xfffffffful << RTC_SPR8_SPARE_Pos)              /*!< RTC_T::SPR8: SPARE Mask                   */

#define RTC_SPR9_SPARE_Pos               (0)                                               /*!< RTC_T::SPR9: SPARE Position               */
#define RTC_SPR9_SPARE_Msk               (0xfffffffful << RTC_SPR9_SPARE_Pos)              /*!< RTC_T::SPR9: SPARE Mask                   */

#define RTC_SPR10_SPARE_Pos              (0)                                               /*!< RTC_T::SPR10: SPARE Position              */
#define RTC_SPR10_SPARE_Msk              (0xfffffffful << RTC_SPR10_SPARE_Pos)             /*!< RTC_T::SPR10: SPARE Mask                  */

#define RTC_SPR11_SPARE_Pos              (0)                                               /*!< RTC_T::SPR11: SPARE Position              */
#define RTC_SPR11_SPARE_Msk              (0xfffffffful << RTC_SPR11_SPARE_Pos)             /*!< RTC_T::SPR11: SPARE Mask                  */

#define RTC_SPR12_SPARE_Pos              (0)                                               /*!< RTC_T::SPR12: SPARE Position              */
#define RTC_SPR12_SPARE_Msk              (0xfffffffful << RTC_SPR12_SPARE_Pos)             /*!< RTC_T::SPR12: SPARE Mask                  */

#define RTC_SPR13_SPARE_Pos              (0)                                               /*!< RTC_T::SPR13: SPARE Position              */
#define RTC_SPR13_SPARE_Msk              (0xfffffffful << RTC_SPR13_SPARE_Pos)             /*!< RTC_T::SPR13: SPARE Mask                  */

#define RTC_SPR14_SPARE_Pos              (0)                                               /*!< RTC_T::SPR14: SPARE Position              */
#define RTC_SPR14_SPARE_Msk              (0xfffffffful << RTC_SPR14_SPARE_Pos)             /*!< RTC_T::SPR14: SPARE Mask                  */

#define RTC_SPR15_SPARE_Pos              (0)                                               /*!< RTC_T::SPR15: SPARE Position              */
#define RTC_SPR15_SPARE_Msk              (0xfffffffful << RTC_SPR15_SPARE_Pos)             /*!< RTC_T::SPR15: SPARE Mask                  */

#define RTC_SPR16_SPARE_Pos              (0)                                               /*!< RTC_T::SPR16: SPARE Position              */
#define RTC_SPR16_SPARE_Msk              (0xfffffffful << RTC_SPR16_SPARE_Pos)             /*!< RTC_T::SPR16: SPARE Mask                  */

#define RTC_SPR17_SPARE_Pos              (0)                                               /*!< RTC_T::SPR17: SPARE Position              */
#define RTC_SPR17_SPARE_Msk              (0xfffffffful << RTC_SPR17_SPARE_Pos)             /*!< RTC_T::SPR17: SPARE Mask                  */

#define RTC_SPR18_SPARE_Pos              (0)                                               /*!< RTC_T::SPR18: SPARE Position              */
#define RTC_SPR18_SPARE_Msk              (0xfffffffful << RTC_SPR18_SPARE_Pos)             /*!< RTC_T::SPR18: SPARE Mask                  */

#define RTC_SPR19_SPARE_Pos              (0)                                               /*!< RTC_T::SPR19: SPARE Position              */
#define RTC_SPR19_SPARE_Msk              (0xfffffffful << RTC_SPR19_SPARE_Pos)             /*!< RTC_T::SPR19: SPARE Mask                  */

/**@}*/ /* RTC_CONST */
/**@}*/ /* end of RTC register group */


/*---------------------- Smart Card Host Interface Controller -------------------------*/
/**
    @addtogroup SC Smart Card Host Interface Controller(SC)
    Memory Mapped Structure for SC Controller
@{ */

typedef struct {


    union {
/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RBR</font><br><p> <font size="2">
Offset: 0x00  SC Receive Buffer Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>RBR</td><td><div style="word-wrap: break-word;"><b>Receiving Buffer</b><br>
By reading this register, the SC Controller will return an 8-bit data received from RX pin (LSB first).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

        __I  uint32_t  RBR;
/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">THR</font><br><p> <font size="2">
Offset: 0x00  SC Transmit Buffer Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>THR</td><td><div style="word-wrap: break-word;"><b>Transmit Buffer</b><br>
By writing to this register, the SC sends out an 8-bit data through the TX pin (LSB first).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

        __O  uint32_t  THR;
    };

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTL</font><br><p> <font size="2">
Offset: 0x04  SC Control Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>SC_CEN</td><td><div style="word-wrap: break-word;"><b>SC Engine Enable</b><br>
Set this bit to "1" to enable SC operation.<br>
If this bit is cleared, SC will force all transition to IDLE state.<br>
</div></td></tr><tr><td>
[1]</td><td>DIS_RX</td><td><div style="word-wrap: break-word;"><b>RX Transition Disable</b><br>
0 = Receiver Enabled.<br>
1 = Receiver Disabled.<br>
</div></td></tr><tr><td>
[2]</td><td>DIS_TX</td><td><div style="word-wrap: break-word;"><b>TX Transition Disable</b><br>
0 = Transceiver Enabled.<br>
1 = Transceiver Disabled.<br>
</div></td></tr><tr><td>
[3]</td><td>AUTO_CON_EN</td><td><div style="word-wrap: break-word;"><b>Auto Convention Enable</b><br>
0 = Auto-convention Disabled.<br>
1 = Auto-convention Enabled.<br>
When hardware receives TS in answer to reset state and the TS is direct convention, CON_SEL will be set to 00 automatically, otherwise if the TS is inverse convention, CON_SEL will be set to 11.<br>
If software enables auto convention function, the setting step must be done before Answer to Reset state and the first data must be 0x3B or 0x3F.<br>
After hardware received first data and stored it at buffer, hardware will decided the convention and change the SC_CTL[CON_SEL] register automatically.<br>
If the first data is not 0x3B or 0x3F, hardware will generate an interrupt INT_ACON_ERR(if SC_IER [ACON_ERR_IE = "1"] to CPU.<br>
</div></td></tr><tr><td>
[5:4]</td><td>CON_SEL</td><td><div style="word-wrap: break-word;"><b>Convention Selection</b><br>
00 = Direct convention.<br>
01 = Reserved.<br>
10 = Reserved.<br>
11 = Inverse convention.<br>
Note: If AUTO_CON_EN is enabled, this field must be ignored.<br>
</div></td></tr><tr><td>
[7:6]</td><td>RX_FTRI_LEV</td><td><div style="word-wrap: break-word;"><b>RX Buffer Trigger Level</b><br>
When the number of bytes in the receiving buffer equals the RX_FTRI_LEV, the RDA_IF will be set (if IER [RDA_IEN] is enabled, an interrupt will be generated).<br>
00 = INTR_RDA Trigger Level 1 byte.<br>
01 = INTR_RDA Trigger Level 2 bytes.<br>
10 = INTR_RDA Trigger Level 3 bytes.<br>
11 = Reserved.<br>
</div></td></tr><tr><td>
[12:8]</td><td>BGT</td><td><div style="word-wrap: break-word;"><b>Block Guard Time (BGT)</b><br>
This field indicates the counter for block guard time.<br>
According to ISO7816-3, in T=0 mode, software must fill 15 (real block guard time = 16) to this field and in T=1 mode software must fill 21 (real block guard time = 22) to it.<br>
In TX mode, hardware will auto hold off first character until BGT has elapsed regardless of the TX data.<br>
In RX mode, software can enable SC_ALTCTL [RX_BGT_EN] to detect the first coming character timing.<br>
If the incoming data timing less than BGT, an interrupt will be generated.<br>
Note: The real block guard time is BGT + 1.<br>
</div></td></tr><tr><td>
[14:13]</td><td>TMR_SEL</td><td><div style="word-wrap: break-word;"><b>Timer Selection</b><br>
00 = Disable all internal timer function.<br>
01 = Enable internal 24 bit timer.<br>
Software can configure it by setting SC_TMR0 [23:0].<br>
SC_TMR1 and SC_TMR2 will be ignored in this mode.<br>
10 = Enable internal 24 bit timer and 8 bit internal timer.<br>
Software can configure the 24 bit timer by setting SC_TMR0 [23:0] and configure the 8 bit timer by setting SC_TMR1 [7:0].<br>
SC_TMR2 will be ignored in this mode.<br>
11 = Enable internal 24 bit timer and two 8 bit timers.<br>
Software can configure them by setting SC_TMR0 [23:0], SC_TMR1 [7:0] and SC_TMR2 [7:0].<br>
</div></td></tr><tr><td>
[15]</td><td>SLEN</td><td><div style="word-wrap: break-word;"><b>Stop Bit Length</b><br>
This field indicates the length of stop bit.<br>
0 = The stop bit length is 2 ETU.<br>
1 = The stop bit length is 1 ETU.<br>
Note: The default stop bit length is 2.<br>
</div></td></tr><tr><td>
[18:16]</td><td>RX_ERETRY</td><td><div style="word-wrap: break-word;"><b>RX Error Retry Register</b><br>
This field indicates the maximum number of receiver retries that are allowed when parity error has occurred.<br>
Note1: The real maximum retry number is RX_ERETRY + 1, so 8 is the maximum retry number.<br>
Note2: This field can not be changed when RX_ERETRY_EN enabled.<br>
The change flow is to disable RX_ETRTRY_EN first and then fill new retry value.<br>
</div></td></tr><tr><td>
[19]</td><td>RX_ERETRY_EN</td><td><div style="word-wrap: break-word;"><b>RX Error Retry Enable Register</b><br>
This bit enables receiver retry function when parity error has occurred.<br>
0 = RX error retry function Disabled.<br>
1 = RX error retry function Enabled.<br>
Note: User must fill RX_ERETRY value before enabling this bit.<br>
</div></td></tr><tr><td>
[22:20]</td><td>TX_ERETRY</td><td><div style="word-wrap: break-word;"><b>TX Error Retry Register</b><br>
This field indicates the maximum number of transmitter retries that are allowed when parity error has occurred.<br>
Note1: The real retry number is TX_ERETRY + 1, so 8 is the maximum retry number.<br>
Note2: This field can not be changed when TX_ERETRY_EN enabled.<br>
The change flow is to disable TX_ETRTRY_EN first and then fill new retry value.<br>
</div></td></tr><tr><td>
[23]</td><td>TX_ERETRY_EN</td><td><div style="word-wrap: break-word;"><b>TX Error Retry Enable Register</b><br>
This bit enables transmitter retry function when parity error has occurred.<br>
0 = TX error retry function Disabled.<br>
1 = TX error retry function Enabled.<br>
Note: User must fill TX_ERETRY value before enabling this bit.<br>
</div></td></tr><tr><td>
[25:24]</td><td>CD_DEB_SEL</td><td><div style="word-wrap: break-word;"><b>Card Detect De-Bounce Select Register</b><br>
This field indicates the card detect de-bounce selection.<br>
This field indicates the card detect de-bounce selection.<br>
00 = De-bounce sample card insert once per 384 (128 * 3) engine clocks and de-bounce sample card removal once per 128 engine clocks.<br>
01 = De-bounce sample card insert once per 192 (64 * 3) engine clocks and de-bounce sample card removal once per 64 engine clocks.<br>
10 = De-bounce sample card insert once per 96 (32 * 3) engine clocks and de-bounce sample card removal once per 32 engine clocks.<br>
11 = De-bounce sample card insert once per 48 (16 * 3) engine clocks and de-bounce sample card removal once per 16 engine clocks.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ALTCTL</font><br><p> <font size="2">
Offset: 0x08  SC Alternate Control Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TX_RST</td><td><div style="word-wrap: break-word;"><b>TX Software Reset</b><br>
When TX_RST is set, all the bytes in the transmit buffer and TX internal state machine will be cleared.<br>
0 = No effect.<br>
1 = Reset the TX internal state machine and pointers.<br>
Note: This bit will be auto cleared and needs at least 3 SC engine clock cycles.<br>
</div></td></tr><tr><td>
[1]</td><td>RX_RST</td><td><div style="word-wrap: break-word;"><b>RX Software Reset</b><br>
When RX_RST is set, all the bytes in the receiver buffer and RX internal state machine will be cleared.<br>
0 = No effect.<br>
1 = Reset the RX internal state machine and pointers.<br>
Note: This bit will be auto cleared and needs at least 3 SC engine clock cycles.<br>
</div></td></tr><tr><td>
[2]</td><td>DACT_EN</td><td><div style="word-wrap: break-word;"><b>Deactivation Sequence Generator Enable</b><br>
This bit enables SC controller to initiate the card by deactivation sequence<br>
0 = No effect.<br>
1 = Deactivation sequence generator Enabled.<br>
Note1: When the deactivation sequence completed, this bit will be cleared automatically and the SC_ISR [INIT_IS] will be set to "1".<br>
Note2: This field will be cleared by TX_RST and RX_RST.<br>
So don't fill this bit, TX_RST, and RX_RST at the same time.<br>
Note3: If SC_CTL [SC_CEN] is not enabled, this filed can not be programmed.<br>
</div></td></tr><tr><td>
[3]</td><td>ACT_EN</td><td><div style="word-wrap: break-word;"><b>Activation Sequence Generator Enable</b><br>
This bit enables SC controller to initiate the card by activation sequence<br>
0 = No effect.<br>
1 = Activation sequence generator Enabled.<br>
Note1: When the activation sequence completed, this bit will be cleared automatically and the SC_IS [INIT_IS] will be set to "1".<br>
Note2: This field will be cleared by TX_RST and RX_RST, so don't fill this bit, TX_RST, and RX_RST at the same time.<br>
Note3: If SC_CTL [SC_CEN] is not enabled, this filed can not be programmed.<br>
</div></td></tr><tr><td>
[4]</td><td>WARST_EN</td><td><div style="word-wrap: break-word;"><b>Warm Reset Sequence Generator Enable</b><br>
This bit enables SC controller to initiate the card by warm reset sequence<br>
0 = No effect.<br>
1 = Warm reset sequence generator Enabled.<br>
Note1: When the warm reset sequence completed, this bit will be cleared automatically and the SC_ISR [INIT_IS] will be set to "1".<br>
Note2: This field will be cleared by TX_RST and RX_RST, so don't fill this bit, TX_RST, and RX_RST at the same time.<br>
Note3: If SC_CTL [SC_CEN] is not enabled, this filed can not be programmed.<br>
</div></td></tr><tr><td>
[5]</td><td>TMR0_SEN</td><td><div style="word-wrap: break-word;"><b>Internal Timer0 Start Enable</b><br>
This bit enables Timer0 to start counting.<br>
Software can fill "0" to stop it and set "1" to reload and count.<br>
0 = Stops counting.<br>
1 = Starts counting.<br>
Note1: This field is used for internal 24 bit timer when SC_CTL [TMR_SEL] = 01.<br>
Note2: If the operation mode is not in auto-reload mode (SC_TMR0 [26] = "0"), this bit will be auto-cleared by hardware.<br>
Note3: This field will be cleared by TX_RST and RX_RST.<br>
So don't fill this bit, TX_RST and RX_RST at the same time.<br>
Note4: If SC_CTL [SC_CEN] is not enabled, this filed can not be programmed.<br>
</div></td></tr><tr><td>
[6]</td><td>TMR1_SEN</td><td><div style="word-wrap: break-word;"><b>Internal Timer1 Start Enable</b><br>
This bit enables Timer "1" to start counting.<br>
Software can fill 0 to stop it and set "1" to reload and count.<br>
0 = Stops counting.<br>
1 = Starts counting.<br>
Note1: This field is used for internal 8-bit timer when SC_CTL [TMR_SEL] = 01 or 10.<br>
Don't filled TMR1_SEN when SC_CTL [TMR_SEL] = 00 or 11.<br>
Note2: If the operation mode is not in auto-reload mode (SC_TMR1 [26] = "0"), this bit will be auto-cleared by hardware.<br>
Note3: This field will be cleared by TX_RST and RX_RST, so don't fill this bit, TX_RST, and RX_RST at the same time.<br>
Note4: If SC_CTL [SC_CEN] is not enabled, this filed can not be programmed.<br>
</div></td></tr><tr><td>
[7]</td><td>TMR2_SEN</td><td><div style="word-wrap: break-word;"><b>Internal Timer2 Start Enable</b><br>
This bit enables Timer2 to start counting.<br>
Software can fill "0" to stop it and set "1" to reload and count.<br>
0 = Stops counting.<br>
1 = Starts counting.<br>
Note1: This field is used for internal 8-bit timer when SC_CTL [TMR_SEL] == 11.<br>
Don't filled TMR2_SEN when SC_CTL [TMR_SEL] == 00 or 01 or 10.<br>
Note2: If the operation mode is not in auto-reload mode (SC_TMR2 [26] = "0"), this bit will be auto-cleared by hardware.<br>
Note3: This field will be cleared by TX_RST and RX_RST.<br>
So don't fill this bit, TX_RST, and RX_RST at the same time.<br>
Note4: If SC_CTL [SC_CEN] is not enabled, this filed can not be programmed.<br>
</div></td></tr><tr><td>
[9:8]</td><td>INIT_SEL</td><td><div style="word-wrap: break-word;"><b>Initial Timing Selection</b><br>
This field indicates the timing of hardware initial state (activation or warm-reset or deactivation).<br>
</div></td></tr><tr><td>
[12]</td><td>RX_BGT_EN</td><td><div style="word-wrap: break-word;"><b>Receiver Block Guard Time Function Enable</b><br>
0 = Receiver block guard time function Disabled.<br>
1 = Receiver block guard time function Enabled.<br>
</div></td></tr><tr><td>
[13]</td><td>TMR0_ATV</td><td><div style="word-wrap: break-word;"><b>Internal Timer0 Active State (Read Only)</b><br>
This bit indicates the timer counter status of timer0.<br>
0 = Timer0 is not active.<br>
1 = Timer0 is active.<br>
</div></td></tr><tr><td>
[14]</td><td>TMR1_ATV</td><td><div style="word-wrap: break-word;"><b>Internal Timer1 Active State (Read Only)</b><br>
This bit indicates the timer counter status of timer1.<br>
0 = Timer1 is not active.<br>
1 = Timer1 is active.<br>
</div></td></tr><tr><td>
[15]</td><td>TMR2_ATV</td><td><div style="word-wrap: break-word;"><b>Internal Timer2 Active State (Read Only)</b><br>
This bit indicates the timer counter status of timer2.<br>
0 = Timer2 is not active.<br>
1 = Timer2 is active.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ALTCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">EGTR</font><br><p> <font size="2">
Offset: 0x0C  SC Extend Guard Time Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>EGT</td><td><div style="word-wrap: break-word;"><b>Extended Guard Time</b><br>
This field indicates the extended guard timer value.<br>
Note: The counter is ETU based and the real extended guard time is EGT.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t EGTR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RFTMR</font><br><p> <font size="2">
Offset: 0x10  SC Receive Buffer Time-Out Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[8:0]</td><td>RFTM</td><td><div style="word-wrap: break-word;"><b>SC Receiver Buffer Time-Out Register (ETU Based)</b><br>
The time-out counter resets and starts counting whenever the RX buffer received a new data word.<br>
Once the counter decrease to "1" and no new data is received or CPU does not read data by reading SC_RBR register, a receiver time-out interrupt INT_RTMR will be generated(if SC_IER[RTMR_IE] is high).<br>
Note1: The counter is ETU based and the real count value is RFTM + 1<br>
Note2: Fill all "0" to this field to disable this function.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t RFTMR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ETUCR</font><br><p> <font size="2">
Offset: 0x14  SC ETU Control Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[11:0]</td><td>ETU_RDIV</td><td><div style="word-wrap: break-word;"><b>ETU Rate Divider</b><br>
The field indicates the clock rate divider.<br>
The real ETU is ETU_RDIV + 1.<br>
Note1: Software can configure this field, but this field must be greater than 0x04.<br>
Note2: Software can configure this field, but if the error rate is equal to 2%, this field must be greater than 0x040.<br>
</div></td></tr><tr><td>
[15]</td><td>COMPEN_EN</td><td><div style="word-wrap: break-word;"><b>Compensation Mode Enable</b><br>
This bit enables clock compensation function.<br>
When this bit enabled, hardware will alternate between n clock cycles and (n-1) clock cycles, where n is the value to be written into the ETU_RDIV register.<br>
0 = Compensation function Disabled.<br>
1 = Compensation function Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ETUCR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IER</font><br><p> <font size="2">
Offset: 0x18  SC Interrupt Enable Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RDA_IE</td><td><div style="word-wrap: break-word;"><b>Receive Data Reach Interrupt Enable</b><br>
This field is used for received data reaching trigger level (SC_CTL [RX_FTRI_LEV]) interrupt enable.<br>
0 = INT_RDR Disabled.<br>
1 = INT_RDR Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>TBE_IE</td><td><div style="word-wrap: break-word;"><b>Transmit Buffer Empty Interrupt Enable</b><br>
This field is used for transmit buffer empty interrupt enable.<br>
0 = INT_THRE Disabled.<br>
1 = INT_THRE Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>TERR_IE</td><td><div style="word-wrap: break-word;"><b>Transfer Error Interrupt Enable</b><br>
This field is used for transfer error interrupt enable.<br>
The transfer error states is at SC_TRSR register which includes receiver break error (RX_EBR_F), frame error (RX_EFR_F), parity error (RX_EPA_F), receiver buffer overflow error (RX_OVER_F), transmit buffer overflow error (TX_OVER_F), receiver retry over limit error (RX_OVER_ERETRY) and transmitter retry over limit error (TX_OVER_ERETRY).<br>
0 = INT_TERR Disabled.<br>
1 = INT_TERR Enabled.<br>
</div></td></tr><tr><td>
[3]</td><td>TMR0_IE</td><td><div style="word-wrap: break-word;"><b>Timer0 Interrupt Enable</b><br>
This field is used for TMR0 interrupt enable.<br>
0 = INT_TMR0 Disabled.<br>
1 = INT_TMR0 Enabled.<br>
</div></td></tr><tr><td>
[4]</td><td>TMR1_IE</td><td><div style="word-wrap: break-word;"><b>Timer1 Interrupt Enable</b><br>
This field is used for TMR1 interrupt enable.<br>
0 = INT_TMR1 Disabled.<br>
1 = INT_TMR1 Enabled.<br>
</div></td></tr><tr><td>
[5]</td><td>TMR2_IE</td><td><div style="word-wrap: break-word;"><b>Timer2 Interrupt Enable</b><br>
This field is used for TMR2 interrupt enable.<br>
0 = INT_TMR2 Disabled.<br>
1 = INT_TMR2 Enabled.<br>
</div></td></tr><tr><td>
[6]</td><td>BGT_IE</td><td><div style="word-wrap: break-word;"><b>Block Guard Time Interrupt Enable</b><br>
This field is used for block guard time interrupt enable.<br>
0 = INT_BGT Disabled.<br>
1 = INT_BGT Enabled.<br>
</div></td></tr><tr><td>
[7]</td><td>CD_IE</td><td><div style="word-wrap: break-word;"><b>Card Detect Interrupt Enable</b><br>
This field is used for card detect interrupt enable.<br>
The card detect status register is SC_PINCSR [CD_CH] and SC_PINCSR[CD_CL].<br>
0 = INT_CD Disabled.<br>
1 = INT_CD Enabled.<br>
</div></td></tr><tr><td>
[8]</td><td>INIT_IE</td><td><div style="word-wrap: break-word;"><b>Initial End Interrupt Enable</b><br>
This field is used for activation (SC_ALTCTL [ACT_EN]), deactivation (SC_ALTCTL [DACT_EN]) and warm reset (SC_ALTCTL [WARST_EN]) sequence interrupt enable.<br>
0 = INT_INIT Disabled.<br>
1 = INT_INIT Enabled.<br>
</div></td></tr><tr><td>
[9]</td><td>RTMR_IE</td><td><div style="word-wrap: break-word;"><b>Receiver Buffer Time-Out Interrupt Enable</b><br>
This field is used for receiver buffer time-out interrupt enable.<br>
0 = INT_RTMR Disabled.<br>
1 = INT_RTMR Enabled.<br>
</div></td></tr><tr><td>
[10]</td><td>ACON_ERR_IE</td><td><div style="word-wrap: break-word;"><b>Auto Convention Error Interrupt Enable</b><br>
This field is used for auto convention error interrupt enable.<br>
0 = INT_ACON_ERR Disabled.<br>
1 = INT_ACON_ERR Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IER;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISR</font><br><p> <font size="2">
Offset: 0x1C  SC Interrupt Status Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RDA_IS</td><td><div style="word-wrap: break-word;"><b>Receive Data Reach Interrupt Status Flag (Read Only)</b><br>
This field is used for received data reaching trigger level (SC_CTL [RX_FTRI_LEV]) interrupt status flag.<br>
Note: This field is the status flag of received data reaching SC_CTL [RX_FTRI_LEV].<br>
If software reads data from SC_RBR and receiver pointer is less than SC_CTL [RX_FTRI_LEV], this bit will be cleared automatically.<br>
</div></td></tr><tr><td>
[1]</td><td>TBE_IS</td><td><div style="word-wrap: break-word;"><b>Transmit Buffer Empty Interrupt Status Flag (Read Only)</b><br>
This field is used for transmit buffer empty interrupt status flag.<br>
This bit is different with SC_TRSR [TX_EMPTY_F] flag and SC_TRSR [TX_ATV] flag; The TX_EMPTY_F will be set when the last byte data be read to shift register and TX_ATV flag indicates the transmitter is in active or not (the last data has been transmitted or not), but the TBE_IS may be set when the last byte data be read to shift register or the last data has been transmitted.<br>
When this bit assert, software can write 1~4 byte data to SC_THR register.<br>
Note: If software wants to clear this bit, software must write data to SC_THR register and then this bit will be cleared automatically.<br>
</div></td></tr><tr><td>
[2]</td><td>TERR_IS</td><td><div style="word-wrap: break-word;"><b>Transfer Error Interrupt Status Flag (Read Only)</b><br>
This field is used for transfer error interrupt status flag.<br>
The transfer error states is at SC_TRSR register which includes receiver break error (RX_EBR_F), frame error (RX_EFR_F), parity error (RX_EPA_F) and receiver buffer overflow error (RX_OVER_F), transmit buffer overflow error (TX_OVER_F), receiver retry over limit error (RX_OVER_ERETRY) and transmitter retry over limit error (TX_OVER_ERETRY).<br>
Note: This field is the status flag of SC_TRSR [RX_EBR_F], SC_TRSR [RX_EFR_F], SC_TRSR [RX_EPA_F], SC_TRSR [RX_OVER_F], SC_TRSR [TX_OVER_F], SC_TRSR [RX_OVER_ERETRY] or SC_TRSR [TX_OVER_ERETRY].<br>
So if software wants to clear this bit, software must write "1" to each field.<br>
</div></td></tr><tr><td>
[3]</td><td>TMR0_IS</td><td><div style="word-wrap: break-word;"><b>Timer0 Interrupt Status Flag (Read Only)</b><br>
This field is used for TMR0 interrupt status flag.<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[4]</td><td>TMR1_IS</td><td><div style="word-wrap: break-word;"><b>Timer1 Interrupt Status Flag (Read Only)</b><br>
This field is used for TMR1 interrupt status flag.<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[5]</td><td>TMR2_IS</td><td><div style="word-wrap: break-word;"><b>Timer2 Interrupt Status Flag (Read Only)</b><br>
This field is used for TMR2 interrupt status flag.<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[6]</td><td>BGT_IS</td><td><div style="word-wrap: break-word;"><b>Block Guard Time Interrupt Status Flag (Read Only)</b><br>
This field is used for block guard time interrupt status flag.<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[7]</td><td>CD_IS</td><td><div style="word-wrap: break-word;"><b>Card Detect Interrupt Status Flag (Read Only)</b><br>
This field is used for card detect interrupt status flag.<br>
The card detect status register is SC_PINCSR [CD_INS_F] and SC_PINCSR [CD_REM_F].<br>
Note: This field is the status flag of SC_PINCSR [CD_INS_F] or SC_PINCSR [CD_REM_F].<br>
So if software wants to clear this bit, software must write "1" to this field.<br>
</div></td></tr><tr><td>
[8]</td><td>INIT_IS</td><td><div style="word-wrap: break-word;"><b>Initial End Interrupt Status Flag (Read Only)</b><br>
This field is used for activation (SC_ALTCTL [ACT_EN]), deactivation (SC_ALTCTL [DACT_EN]) and warm reset (SC_ALTCTL [WARST_EN]) sequence interrupt status flag.<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[9]</td><td>RTMR_IS</td><td><div style="word-wrap: break-word;"><b>Receiver Buffer Time-Out Interrupt Status Flag (Read Only)</b><br>
This field is used for receiver buffer time-out interrupt status flag.<br>
Note: This field is the status flag of receiver buffer time-out state.<br>
If software wants to clear this bit, software must read the receiver buffer remaining data by reading SC_RBR register,.<br>
</div></td></tr><tr><td>
[10]</td><td>ACON_ERR_IS</td><td><div style="word-wrap: break-word;"><b>Auto Convention Error Interrupt Status Flag (Read Only)</b><br>
This field indicates auto convention sequence error.<br>
If the received TS at ATR state is not 0x3B or 0x3F, this bit will be set.<br>
Note: This bit is read only, but can be cleared by writing "1" to it.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TRSR</font><br><p> <font size="2">
Offset: 0x20  SC Transfer Status Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RX_OVER_F</td><td><div style="word-wrap: break-word;"><b>RX Overflow Error Status Flag (Read Only)</b><br>
This bit is set when RX buffer overflow.<br>
If the number of received bytes is greater than RX Buffer (SC_RBR) size, 4 bytes of SC, this bit will be set.<br>
Note1: This bit is read only, but it can be cleared by writing "1" to it.<br>
Note2: The overwrite data will be ignored.<br>
</div></td></tr><tr><td>
[1]</td><td>RX_EMPTY_F</td><td><div style="word-wrap: break-word;"><b>Receiver Buffer Empty Status Flag(Read Only)</b><br>
This bit indicates RX buffer empty or not.<br>
When the last byte of RX buffer has been read by CPU, hardware sets this bit high.<br>
It will be cleared when SC receives any new data.<br>
</div></td></tr><tr><td>
[2]</td><td>RX_FULL_F</td><td><div style="word-wrap: break-word;"><b>Receiver Buffer Full Status Flag (Read Only)</b><br>
This bit indicates RX buffer full or not.<br>
This bit is set when RX pointer is equal to 4, otherwise it is cleared by hardware.<br>
</div></td></tr><tr><td>
[4]</td><td>RX_EPA_F</td><td><div style="word-wrap: break-word;"><b>Receiver Parity Error Status Flag (Read Only)</b><br>
This bit is set to logic "1" whenever the received character does not have a valid "parity bit".<br>
Note1: This bit is read only, but it can be cleared by writing "1" to it.<br>
Note2: If CPU sets receiver retries function by setting SC_CTL [RX_ERETRY_EN] register, hardware will not set this flag.<br>
</div></td></tr><tr><td>
[5]</td><td>RX_EFR_F</td><td><div style="word-wrap: break-word;"><b>Receiver Frame Error Status Flag (Read Only)</b><br>
This bit is set to logic "1" whenever the received character does not have a valid "stop bit" (that is, the stop bit following the last data bit or parity bit is detected as a logic "0").<br>
Note1: This bit is read only, but can be cleared by writing "1" to it.<br>
Note2: If CPI sets receiver retries function by setting SC_CTL [RX_ERETRY_EN] register, hardware will not set this flag.<br>
</div></td></tr><tr><td>
[6]</td><td>RX_EBR_F</td><td><div style="word-wrap: break-word;"><b>Receiver Break Error Status Flag (Read Only)</b><br>
This bit is set to a logic "1" whenever the received data input (RX) held in the "spacing state" (logic "0") is longer than a full word transmission time (that is, the total time of "start bit" + data bits + parity + stop bits).<br>
Note1: This bit is read only, but it can be cleared by writing "1" to it.<br>
Note2: If CPU sets receiver retries function by setting SC_CTL [RX_ERETRY_EN] register, hardware will not set this flag.<br>
</div></td></tr><tr><td>
[8]</td><td>TX_OVER_F</td><td><div style="word-wrap: break-word;"><b>TX Overflow Error Interrupt Status Flag (Read Only)</b><br>
If TX buffer is full (TX_FULL_F = "1"), an additional write data to SC_THR will cause this bit to logic "1".<br>
Note1: This bit is read only, but it can be cleared by writing "1" to it.<br>
Note2: The additional write data will be ignored.<br>
</div></td></tr><tr><td>
[9]</td><td>TX_EMPTY_F</td><td><div style="word-wrap: break-word;"><b>Transmit Buffer Empty Status Flag (Read Only)</b><br>
This bit indicates TX buffer empty or not.<br>
When the last byte of TX buffer has been transferred to Transmitter Shift Register, hardware sets this bit high.<br>
It will be cleared when writing data into SC_THR (TX buffer not empty).<br>
</div></td></tr><tr><td>
[10]</td><td>TX_FULL_F</td><td><div style="word-wrap: break-word;"><b>Transmit Buffer Full Status Flag (Read Only)</b><br>
This bit indicates TX buffer full or not.<br>
This bit is set when TX pointer is equal to 4, otherwise is cleared by hardware.<br>
</div></td></tr><tr><td>
[18:16]</td><td>RX_POINT_F</td><td><div style="word-wrap: break-word;"><b>Receiver Buffer Pointer Status Flag (Read Only)</b><br>
This field indicates the RX buffer pointer status flag.<br>
When SC receives one byte from external device, RX_POINT_F increases one.<br>
When one byte of RX buffer is read by CPU, RX_POINT_F decreases one.<br>
</div></td></tr><tr><td>
[21]</td><td>RX_REERR</td><td><div style="word-wrap: break-word;"><b>Receiver Retry Error (Read Only)</b><br>
This bit is set by hardware when RX has any error and retries transfer.<br>
Note1: This bit is read only, but it can be cleared by writing "1" to it.<br>
Note2 This bit is a flag and can not generate any interrupt to CPU.<br>
Note3: If CPU enables receiver retry function by setting SC_CTL [RX_ERETRY_EN] register, the RX_EPA_F flag will be ignored (hardware will not set RX_EPA_F).<br>
</div></td></tr><tr><td>
[22]</td><td>RX_OVER_ERETRY</td><td><div style="word-wrap: break-word;"><b>Receiver Over Retry Error (Read Only)</b><br>
This bit is set by hardware when RX transfer error retry over retry number limit.<br>
Note1: This bit is read only, but it can be cleared by writing "1" to it.<br>
Note2: If CPU enables receiver retries function by setting SC_CTL [RX_ERETRY_EN] register, the RX_EPA_F flag will be ignored (hardware will not set RX_EPA_F).<br>
</div></td></tr><tr><td>
[23]</td><td>RX_ATV</td><td><div style="word-wrap: break-word;"><b>Receiver In Active Status Flag (Read Only)</b><br>
This bit is set by hardware when RX transfer is in active.<br>
This bit is cleared automatically when RX transfer is finished.<br>
</div></td></tr><tr><td>
[26:24]</td><td>TX_POINT_F</td><td><div style="word-wrap: break-word;"><b>Transmit Buffer Pointer Status Flag (Read Only)</b><br>
This field indicates the TX buffer pointer status flag.<br>
When CPU writes data into SC_THR, TX_POINT_F increases one.<br>
When one byte of TX Buffer is transferred to transmitter shift register, TX_POINT_F decreases one.<br>
</div></td></tr><tr><td>
[29]</td><td>TX_REERR</td><td><div style="word-wrap: break-word;"><b>Transmitter Retry Error (Read Only)</b><br>
This bit is set by hardware when transmitter re-transmits.<br>
Note1: This bit is read only, but it can be cleared by writing "1" to it.<br>
Note2 This bit is a flag and can not generate any interrupt to CPU.<br>
</div></td></tr><tr><td>
[30]</td><td>TX_OVER_ERETRY</td><td><div style="word-wrap: break-word;"><b>Transmitter Over Retry Error (Read Only)</b><br>
This bit is set by hardware when transmitter re-transmits over retry number limitation.<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[31]</td><td>TX_ATV</td><td><div style="word-wrap: break-word;"><b>Transmit In Active Status Flag (Read Only)</b><br>
This bit is set by hardware when TX transfer is in active or the last byte transmission has not completed.<br>
This bit is cleared automatically when TX transfer is finished and the STOP bit (include guard time) has been transmitted.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TRSR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PINCSR</font><br><p> <font size="2">
Offset: 0x24  SC Pin Control State Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>POW_EN</td><td><div style="word-wrap: break-word;"><b>SC_POW_EN Pin Signal</b><br>
This bit is the pin status of SC_POW_EN but user can drive SC_POW_EN pin to high or low by setting this bit.<br>
0 = Drive SC_POW_EN pin to low.<br>
1 = Drive SC_POW_EN pin to high.<br>
Note: When operation at activation, warm reset or deactivation mode, this bit will be changed automatically.<br>
So don't fill this field When operating in these modes.<br>
</div></td></tr><tr><td>
[1]</td><td>SC_RST</td><td><div style="word-wrap: break-word;"><b>SC_RST Pin Signal</b><br>
This bit is the pin status of SC_RST but user can drive SC_RST pin to high or low by setting this bit.<br>
0 = Drive SC_RST pin to low.<br>
1 = Drive SC_RST pin to high.<br>
Note: When operation at activation, warm reset or deactivation mode, this bit will be changed automatically.<br>
So don't fill this field When operating in these modes.<br>
</div></td></tr><tr><td>
[2]</td><td>CD_REM_F</td><td><div style="word-wrap: break-word;"><b>Card Detect Removal Status Of SC_CD Pin (Read Only)</b><br>
This bit is set whenever card has been removal.<br>
0 = No effect.<br>
1 = Card Removal.<br>
Note1: This bit is read only, but it can be cleared by writing "1" to it.<br>
Note2: Card detect engine will start after SC_CTL [SC_CEN] set.<br>
</div></td></tr><tr><td>
[3]</td><td>CD_INS_F</td><td><div style="word-wrap: break-word;"><b>Card Detect Insert Status Of SC_CD Pin (Read Only)</b><br>
This bit is set whenever card has been inserted.<br>
0 = No effect.<br>
1 = Card insert.<br>
Note1: This bit is read only, but it can be cleared by writing "1" to it.<br>
Note2: Card detect engine will start after SC_CTL [SC_CEN] set.<br>
</div></td></tr><tr><td>
[4]</td><td>CD_PIN_ST</td><td><div style="word-wrap: break-word;"><b>Card Detect Status Of SC_CD Pin Status (Read Only)</b><br>
This bit is the pin status flag of SC_CD<br>
0 = SC_CD pin state at low.<br>
1 = SC_CD pin state at high.<br>
</div></td></tr><tr><td>
[6]</td><td>CLK_KEEP</td><td><div style="word-wrap: break-word;"><b>SC Clock Enable</b><br>
0 = SC clock generation Disabled.<br>
1 = SC clock always keeps free running.<br>
Note: When operation at activation, warm reset or deactivation mode, this bit will be changed automatically.<br>
So don't fill this field when operation in these modes.<br>
</div></td></tr><tr><td>
[7]</td><td>ADAC_CD_EN</td><td><div style="word-wrap: break-word;"><b>Auto Deactivation When Card Removal</b><br>
0 = Auto deactivation Disabled when hardware detected the card is removal.<br>
1 = Auto deactivation Enabled when hardware detected the card is removal.<br>
Note1: When the card is removal, hardware will stop any process and then do deactivation sequence (if this bit be setting).<br>
If this process completes.<br>
Hardware will generate an interrupt INT_INIT to CPU.<br>
</div></td></tr><tr><td>
[8]</td><td>SC_OEN_ST</td><td><div style="word-wrap: break-word;"><b>SC Data Pin Output Enable Status (Read Only)</b><br>
0 = SC data output enable pin status is at low.<br>
1 = SC data output enable pin status is at high.<br>
</div></td></tr><tr><td>
[9]</td><td>SC_DATA_O</td><td><div style="word-wrap: break-word;"><b>Output Of SC Data Pin</b><br>
This bit is the pin status of SC data output but user can drive this pin to high or low by setting this bit.<br>
0 = Drive SC data output pin to low.<br>
1 = Drive SC data output pin to high.<br>
Note: When SC is at activation, warm re set or deactivation mode, this bit will be changed automatically.<br>
So don't fill this field when SC is in these modes.<br>
</div></td></tr><tr><td>
[10]</td><td>CD_LEV</td><td><div style="word-wrap: break-word;"><b>Card Detect Level</b><br>
0 = When hardware detects the card detect pin from high to low, it indicates a card is detected.<br>
1 = When hardware detects the card detect pin from low to high, it indicates a card is detected.<br>
Note: Software must select card detect level before Smart Card engine enable<br>
</div></td></tr><tr><td>
[11]</td><td>POW_INV</td><td><div style="word-wrap: break-word;"><b>SC_POW Pin Inverse</b><br>
This bit is used for inverse the SC_POW pin.<br>
There  are  four  kinds  of  combination  for  SC_POW  pin  setting  by  POW_INV  and<br>
POW_EN(SC_PINCSR[0]). POW_INV is bit 1 and POW_EN is bit 0 for SC_POW_Pin as<br>
high or low voltage selection.<br>
POW_INV is 0 and POW_EN is 0, than SC_POW Pin output 0.<br>
POW_INV is 0 and POW_EN is 1, than SC_POW Pin output 1.<br>
POW_INV is 1 and POW_EN is 0, than SC_POW Pin output 1.<br>
POW_INV is 1 and POW_EN is 1, than SC_POW Pin output 0.<br>
Note:  Software  must  select  POW_INV  before  Smart  Card  is  enabled  by  SC_CEN (SC_CTL[0])<br>
</div></td></tr><tr><td>
[16]</td><td>SC_DATA_I_ST</td><td><div style="word-wrap: break-word;"><b>SC Data Input Pin Status (Read Only)</b><br>
This bit is the pin status of SC_DATA_I<br>
0 = The SC_DATA_I pin is low.<br>
1 = The SC_DATA_I pin is high.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PINCSR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TMR0</font><br><p> <font size="2">
Offset: 0x28  SC Internal Timer Control Register 0.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[23:0]</td><td>CNT</td><td><div style="word-wrap: break-word;"><b>Timer 0 Counter Value Register (ETU Base)</b><br>
This field indicates the internal timer operation values.<br>
</div></td></tr><tr><td>
[27:24]</td><td>MODE</td><td><div style="word-wrap: break-word;"><b>Timer 0 Operation Mode Selection</b><br>
This field indicates the internal 24 bit timer operation selection.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TMR0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TMR1</font><br><p> <font size="2">
Offset: 0x2C  SC Internal Timer Control Register 1.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>CNT</td><td><div style="word-wrap: break-word;"><b>Timer 1 Counter Value Register (ETU Base)</b><br>
This field indicates the internal timer operation values.<br>
</div></td></tr><tr><td>
[27:24]</td><td>MODE</td><td><div style="word-wrap: break-word;"><b>Timer 1 Operation Mode Selection</b><br>
This field indicates the internal 8 bit timer operation selection.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TMR1;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TMR2</font><br><p> <font size="2">
Offset: 0x30  SC Internal Timer Control Register 2.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>CNT</td><td><div style="word-wrap: break-word;"><b>Timer 2 Counter Value Register (ETU Base)</b><br>
This field indicates the internal timer operation values.<br>
</div></td></tr><tr><td>
[27:24]</td><td>MODE</td><td><div style="word-wrap: break-word;"><b>Timer 2 Operation Mode Selection</b><br>
This field indicates the internal 8 bit timer operation selection.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TMR2;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">UACTL</font><br><p> <font size="2">
Offset: 0x34  SC UART Mode Control Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>UA_MODE_EN</td><td><div style="word-wrap: break-word;"><b>UART Mode Enable</b><br>
0 = Smart Card mode.<br>
1 = UART mode.<br>
Note1: When operating in UART mode, user must set SCx_CTL [CON_SEL] and SCx_CTL [AUTO_CON_EN] to "0".<br>
Note2: When operating in smart card mode, user must set SCx_UACTL [7:0] register to "0".<br>
Note3: When UART is enabled, hardware will generate a reset to reset internal buffer and internal state machine.<br>
</div></td></tr><tr><td>
[5:4]</td><td>DATA_LEN</td><td><div style="word-wrap: break-word;"><b>Data Length</b><br>
00 = 8 bits<br>
01 = 7 bits<br>
10 = 6 bits<br>
11 = 5 bits<br>
Note: In Smart Card mode, this field must be '00'<br>
</div></td></tr><tr><td>
[6]</td><td>PBDIS</td><td><div style="word-wrap: break-word;"><b>Parity Bit Disable</b><br>
0 = Parity bit is generated or checked between the "last data word bit" and "stop bit" of the serial data.<br>
1 = Parity bit is not generated (transmitting data) or checked (receiving data) during transfer.<br>
Note: In Smart Card mode, this field must be '0' (default setting is with parity bit)<br>
</div></td></tr><tr><td>
[7]</td><td>OPE</td><td><div style="word-wrap: break-word;"><b>Odd Parity Enable</b><br>
0 = Even number of logic 1's are transmitted or check the data word and parity bits in receiving mode.<br>
1 = Odd number of logic 1's are transmitted or check the data word and parity bits in receiving mode.<br>
Note: This bit has effect only when PBDIS bit is '0'.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t UACTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TDRA</font><br><p> <font size="2">
Offset: 0x38  SC Timer Current Data Register A.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[23:0]</td><td>TDR0</td><td><div style="word-wrap: break-word;"><b>Timer0 Current Data Register (Read Only)</b><br>
This field indicates the current count values of timer0.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t TDRA;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TDRB</font><br><p> <font size="2">
Offset: 0x3C  SC Timer Current Data Register B.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>TDR1</td><td><div style="word-wrap: break-word;"><b>Timer1 Current Data Register (Read Only)</b><br>
This field indicates the current count values of timer1.<br>
</div></td></tr><tr><td>
[15:8]</td><td>TDR2</td><td><div style="word-wrap: break-word;"><b>Timer2 Current Data Register (Read Only)</b><br>
This field indicates the current count values of timer2.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t TDRB;

} SC_T;

/**
    @addtogroup SC_CONST SC Bit Field Definition
    Constant Definitions for SC Controller
@{ */

#define SC_DAT_DAT_Pos                   (0)                                               /*!< SC_T::DAT: DAT Position                   */
#define SC_DAT_DAT_Msk                   (0xfful << SC_DAT_DAT_Pos)                        /*!< SC_T::DAT: DAT Mask                       */

#define SC_CTL_SC_CEN_Pos                (0)                                               /*!< SC_T::CTL: SC_CEN Position                */
#define SC_CTL_SC_CEN_Msk                (0x1ul << SC_CTL_SC_CEN_Pos)                      /*!< SC_T::CTL: SC_CEN Mask                    */

#define SC_CTL_DIS_RX_Pos                (1)                                               /*!< SC_T::CTL: DIS_RX Position                */
#define SC_CTL_DIS_RX_Msk                (0x1ul << SC_CTL_DIS_RX_Pos)                      /*!< SC_T::CTL: DIS_RX Mask                    */

#define SC_CTL_DIS_TX_Pos                (2)                                               /*!< SC_T::CTL: DIS_TX Position                */
#define SC_CTL_DIS_TX_Msk                (0x1ul << SC_CTL_DIS_TX_Pos)                      /*!< SC_T::CTL: DIS_TX Mask                    */

#define SC_CTL_AUTO_CON_EN_Pos           (3)                                               /*!< SC_T::CTL: AUTO_CON_EN Position           */
#define SC_CTL_AUTO_CON_EN_Msk           (0x1ul << SC_CTL_AUTO_CON_EN_Pos)                 /*!< SC_T::CTL: AUTO_CON_EN Mask               */

#define SC_CTL_CON_SEL_Pos               (4)                                               /*!< SC_T::CTL: CON_SEL Position               */
#define SC_CTL_CON_SEL_Msk               (0x3ul << SC_CTL_CON_SEL_Pos)                     /*!< SC_T::CTL: CON_SEL Mask                   */

#define SC_CTL_RX_FTRI_LEV_Pos           (6)                                               /*!< SC_T::CTL: RX_FTRI_LEV Position           */
#define SC_CTL_RX_FTRI_LEV_Msk           (0x3ul << SC_CTL_RX_FTRI_LEV_Pos)                 /*!< SC_T::CTL: RX_FTRI_LEV Mask               */

#define SC_CTL_BGT_Pos                   (8)                                               /*!< SC_T::CTL: BGT Position                   */
#define SC_CTL_BGT_Msk                   (0x1ful << SC_CTL_BGT_Pos)                        /*!< SC_T::CTL: BGT Mask                       */

#define SC_CTL_TMR_SEL_Pos               (13)                                              /*!< SC_T::CTL: TMR_SEL Position               */
#define SC_CTL_TMR_SEL_Msk               (0x3ul << SC_CTL_TMR_SEL_Pos)                     /*!< SC_T::CTL: TMR_SEL Mask                   */

#define SC_CTL_SLEN_Pos                  (15)                                              /*!< SC_T::CTL: SLEN Position                  */
#define SC_CTL_SLEN_Msk                  (0x1ul << SC_CTL_SLEN_Pos)                        /*!< SC_T::CTL: SLEN Mask                      */

#define SC_CTL_RX_ERETRY_Pos             (16)                                              /*!< SC_T::CTL: RX_ERETRY Position             */
#define SC_CTL_RX_ERETRY_Msk             (0x7ul << SC_CTL_RX_ERETRY_Pos)                   /*!< SC_T::CTL: RX_ERETRY Mask                 */

#define SC_CTL_RX_ERETRY_EN_Pos          (19)                                              /*!< SC_T::CTL: RX_ERETRY_EN Position          */
#define SC_CTL_RX_ERETRY_EN_Msk          (0x1ul << SC_CTL_RX_ERETRY_EN_Pos)                /*!< SC_T::CTL: RX_ERETRY_EN Mask              */

#define SC_CTL_TX_ERETRY_Pos             (20)                                              /*!< SC_T::CTL: TX_ERETRY Position             */
#define SC_CTL_TX_ERETRY_Msk             (0x7ul << SC_CTL_TX_ERETRY_Pos)                   /*!< SC_T::CTL: TX_ERETRY Mask                 */

#define SC_CTL_TX_ERETRY_EN_Pos          (23)                                              /*!< SC_T::CTL: TX_ERETRY_EN Position          */
#define SC_CTL_TX_ERETRY_EN_Msk          (0x1ul << SC_CTL_TX_ERETRY_EN_Pos)                /*!< SC_T::CTL: TX_ERETRY_EN Mask              */

#define SC_CTL_CD_DEB_SEL_Pos            (24)                                              /*!< SC_T::CTL: CD_DEB_SEL Position            */
#define SC_CTL_CD_DEB_SEL_Msk            (0x3ul << SC_CTL_CD_DEB_SEL_Pos)                  /*!< SC_T::CTL: CD_DEB_SEL Mask                */

#define SC_ALTCTL_TX_RST_Pos             (0)                                               /*!< SC_T::ALTCTL: TX_RST Position             */
#define SC_ALTCTL_TX_RST_Msk             (0x1ul << SC_ALTCTL_TX_RST_Pos)                   /*!< SC_T::ALTCTL: TX_RST Mask                 */

#define SC_ALTCTL_RX_RST_Pos             (1)                                               /*!< SC_T::ALTCTL: RX_RST Position             */
#define SC_ALTCTL_RX_RST_Msk             (0x1ul << SC_ALTCTL_RX_RST_Pos)                   /*!< SC_T::ALTCTL: RX_RST Mask                 */

#define SC_ALTCTL_DACT_EN_Pos            (2)                                               /*!< SC_T::ALTCTL: DACT_EN Position            */
#define SC_ALTCTL_DACT_EN_Msk            (0x1ul << SC_ALTCTL_DACT_EN_Pos)                  /*!< SC_T::ALTCTL: DACT_EN Mask                */

#define SC_ALTCTL_ACT_EN_Pos             (3)                                               /*!< SC_T::ALTCTL: ACT_EN Position             */
#define SC_ALTCTL_ACT_EN_Msk             (0x1ul << SC_ALTCTL_ACT_EN_Pos)                   /*!< SC_T::ALTCTL: ACT_EN Mask                 */

#define SC_ALTCTL_WARST_EN_Pos           (4)                                               /*!< SC_T::ALTCTL: WARST_EN Position           */
#define SC_ALTCTL_WARST_EN_Msk           (0x1ul << SC_ALTCTL_WARST_EN_Pos)                 /*!< SC_T::ALTCTL: WARST_EN Mask               */

#define SC_ALTCTL_TMR0_SEN_Pos           (5)                                               /*!< SC_T::ALTCTL: TMR0_SEN Position           */
#define SC_ALTCTL_TMR0_SEN_Msk           (0x1ul << SC_ALTCTL_TMR0_SEN_Pos)                 /*!< SC_T::ALTCTL: TMR0_SEN Mask               */

#define SC_ALTCTL_TMR1_SEN_Pos           (6)                                               /*!< SC_T::ALTCTL: TMR1_SEN Position           */
#define SC_ALTCTL_TMR1_SEN_Msk           (0x1ul << SC_ALTCTL_TMR1_SEN_Pos)                 /*!< SC_T::ALTCTL: TMR1_SEN Mask               */

#define SC_ALTCTL_TMR2_SEN_Pos           (7)                                               /*!< SC_T::ALTCTL: TMR2_SEN Position           */
#define SC_ALTCTL_TMR2_SEN_Msk           (0x1ul << SC_ALTCTL_TMR2_SEN_Pos)                 /*!< SC_T::ALTCTL: TMR2_SEN Mask               */

#define SC_ALTCTL_INIT_SEL_Pos           (8)                                               /*!< SC_T::ALTCTL: INIT_SEL Position           */
#define SC_ALTCTL_INIT_SEL_Msk           (0x3ul << SC_ALTCTL_INIT_SEL_Pos)                 /*!< SC_T::ALTCTL: INIT_SEL Mask               */

#define SC_ALTCTL_RX_BGT_EN_Pos          (12)                                              /*!< SC_T::ALTCTL: RX_BGT_EN Position          */
#define SC_ALTCTL_RX_BGT_EN_Msk          (0x1ul << SC_ALTCTL_RX_BGT_EN_Pos)                /*!< SC_T::ALTCTL: RX_BGT_EN Mask              */

#define SC_ALTCTL_TMR0_ATV_Pos           (13)                                              /*!< SC_T::ALTCTL: TMR0_ATV Position           */
#define SC_ALTCTL_TMR0_ATV_Msk           (0x1ul << SC_ALTCTL_TMR0_ATV_Pos)                 /*!< SC_T::ALTCTL: TMR0_ATV Mask               */

#define SC_ALTCTL_TMR1_ATV_Pos           (14)                                              /*!< SC_T::ALTCTL: TMR1_ATV Position           */
#define SC_ALTCTL_TMR1_ATV_Msk           (0x1ul << SC_ALTCTL_TMR1_ATV_Pos)                 /*!< SC_T::ALTCTL: TMR1_ATV Mask               */

#define SC_ALTCTL_TMR2_ATV_Pos           (15)                                              /*!< SC_T::ALTCTL: TMR2_ATV Position           */
#define SC_ALTCTL_TMR2_ATV_Msk           (0x1ul << SC_ALTCTL_TMR2_ATV_Pos)                 /*!< SC_T::ALTCTL: TMR2_ATV Mask               */

#define SC_EGTR_EGT_Pos                  (0)                                               /*!< SC_T::EGTR: EGT Position                  */
#define SC_EGTR_EGT_Msk                  (0xfful << SC_EGTR_EGT_Pos)                       /*!< SC_T::EGTR: EGT Mask                      */

#define SC_RFTMR_RFTM_Pos                (0)                                               /*!< SC_T::RFTMR: RFTM Position                */
#define SC_RFTMR_RFTM_Msk                (0x1fful << SC_RFTMR_RFTM_Pos)                    /*!< SC_T::RFTMR: RFTM Mask                    */

#define SC_ETUCR_ETU_RDIV_Pos            (0)                                               /*!< SC_T::ETUCR: ETU_RDIV Position            */
#define SC_ETUCR_ETU_RDIV_Msk            (0xffful << SC_ETUCR_ETU_RDIV_Pos)                /*!< SC_T::ETUCR: ETU_RDIV Mask                */

#define SC_ETUCR_COMPEN_EN_Pos           (15)                                              /*!< SC_T::ETUCR: COMPEN_EN Position           */
#define SC_ETUCR_COMPEN_EN_Msk           (0x1ul << SC_ETUCR_COMPEN_EN_Pos)                 /*!< SC_T::ETUCR: COMPEN_EN Mask               */

#define SC_IER_RDA_IE_Pos                (0)                                               /*!< SC_T::IER: RDA_IE Position                */
#define SC_IER_RDA_IE_Msk                (0x1ul << SC_IER_RDA_IE_Pos)                      /*!< SC_T::IER: RDA_IE Mask                    */

#define SC_IER_TBE_IE_Pos                (1)                                               /*!< SC_T::IER: TBE_IE Position                */
#define SC_IER_TBE_IE_Msk                (0x1ul << SC_IER_TBE_IE_Pos)                      /*!< SC_T::IER: TBE_IE Mask                    */

#define SC_IER_TERR_IE_Pos               (2)                                               /*!< SC_T::IER: TERR_IE Position               */
#define SC_IER_TERR_IE_Msk               (0x1ul << SC_IER_TERR_IE_Pos)                     /*!< SC_T::IER: TERR_IE Mask                   */

#define SC_IER_TMR0_IE_Pos               (3)                                               /*!< SC_T::IER: TMR0_IE Position               */
#define SC_IER_TMR0_IE_Msk               (0x1ul << SC_IER_TMR0_IE_Pos)                     /*!< SC_T::IER: TMR0_IE Mask                   */

#define SC_IER_TMR1_IE_Pos               (4)                                               /*!< SC_T::IER: TMR1_IE Position               */
#define SC_IER_TMR1_IE_Msk               (0x1ul << SC_IER_TMR1_IE_Pos)                     /*!< SC_T::IER: TMR1_IE Mask                   */

#define SC_IER_TMR2_IE_Pos               (5)                                               /*!< SC_T::IER: TMR2_IE Position               */
#define SC_IER_TMR2_IE_Msk               (0x1ul << SC_IER_TMR2_IE_Pos)                     /*!< SC_T::IER: TMR2_IE Mask                   */

#define SC_IER_BGT_IE_Pos                (6)                                               /*!< SC_T::IER: BGT_IE Position                */
#define SC_IER_BGT_IE_Msk                (0x1ul << SC_IER_BGT_IE_Pos)                      /*!< SC_T::IER: BGT_IE Mask                    */

#define SC_IER_CD_IE_Pos                 (7)                                               /*!< SC_T::IER: CD_IE Position                 */
#define SC_IER_CD_IE_Msk                 (0x1ul << SC_IER_CD_IE_Pos)                       /*!< SC_T::IER: CD_IE Mask                     */

#define SC_IER_INIT_IE_Pos               (8)                                               /*!< SC_T::IER: INIT_IE Position               */
#define SC_IER_INIT_IE_Msk               (0x1ul << SC_IER_INIT_IE_Pos)                     /*!< SC_T::IER: INIT_IE Mask                   */

#define SC_IER_RTMR_IE_Pos               (9)                                               /*!< SC_T::IER: RTMR_IE Position               */
#define SC_IER_RTMR_IE_Msk               (0x1ul << SC_IER_RTMR_IE_Pos)                     /*!< SC_T::IER: RTMR_IE Mask                   */

#define SC_IER_ACON_ERR_IE_Pos           (10)                                              /*!< SC_T::IER: ACON_ERR_IE Position           */
#define SC_IER_ACON_ERR_IE_Msk           (0x1ul << SC_IER_ACON_ERR_IE_Pos)                 /*!< SC_T::IER: ACON_ERR_IE Mask               */

#define SC_ISR_RDA_IS_Pos                (0)                                               /*!< SC_T::ISR: RDA_IS Position                */
#define SC_ISR_RDA_IS_Msk                (0x1ul << SC_ISR_RDA_IS_Pos)                      /*!< SC_T::ISR: RDA_IS Mask                    */

#define SC_ISR_TBE_IS_Pos                (1)                                               /*!< SC_T::ISR: TBE_IS Position                */
#define SC_ISR_TBE_IS_Msk                (0x1ul << SC_ISR_TBE_IS_Pos)                      /*!< SC_T::ISR: TBE_IS Mask                    */

#define SC_ISR_TERR_IS_Pos               (2)                                               /*!< SC_T::ISR: TERR_IS Position               */
#define SC_ISR_TERR_IS_Msk               (0x1ul << SC_ISR_TERR_IS_Pos)                     /*!< SC_T::ISR: TERR_IS Mask                   */

#define SC_ISR_TMR0_IS_Pos               (3)                                               /*!< SC_T::ISR: TMR0_IS Position               */
#define SC_ISR_TMR0_IS_Msk               (0x1ul << SC_ISR_TMR0_IS_Pos)                     /*!< SC_T::ISR: TMR0_IS Mask                   */

#define SC_ISR_TMR1_IS_Pos               (4)                                               /*!< SC_T::ISR: TMR1_IS Position               */
#define SC_ISR_TMR1_IS_Msk               (0x1ul << SC_ISR_TMR1_IS_Pos)                     /*!< SC_T::ISR: TMR1_IS Mask                   */

#define SC_ISR_TMR2_IS_Pos               (5)                                               /*!< SC_T::ISR: TMR2_IS Position               */
#define SC_ISR_TMR2_IS_Msk               (0x1ul << SC_ISR_TMR2_IS_Pos)                     /*!< SC_T::ISR: TMR2_IS Mask                   */

#define SC_ISR_BGT_IS_Pos                (6)                                               /*!< SC_T::ISR: BGT_IS Position                */
#define SC_ISR_BGT_IS_Msk                (0x1ul << SC_ISR_BGT_IS_Pos)                      /*!< SC_T::ISR: BGT_IS Mask                    */

#define SC_ISR_CD_IS_Pos                 (7)                                               /*!< SC_T::ISR: CD_IS Position                 */
#define SC_ISR_CD_IS_Msk                 (0x1ul << SC_ISR_CD_IS_Pos)                       /*!< SC_T::ISR: CD_IS Mask                     */

#define SC_ISR_INIT_IS_Pos               (8)                                               /*!< SC_T::ISR: INIT_IS Position               */
#define SC_ISR_INIT_IS_Msk               (0x1ul << SC_ISR_INIT_IS_Pos)                     /*!< SC_T::ISR: INIT_IS Mask                   */

#define SC_ISR_RTMR_IS_Pos               (9)                                               /*!< SC_T::ISR: RTMR_IS Position               */
#define SC_ISR_RTMR_IS_Msk               (0x1ul << SC_ISR_RTMR_IS_Pos)                     /*!< SC_T::ISR: RTMR_IS Mask                   */

#define SC_ISR_ACON_ERR_IS_Pos           (10)                                              /*!< SC_T::ISR: ACON_ERR_IS Position           */
#define SC_ISR_ACON_ERR_IS_Msk           (0x1ul << SC_ISR_ACON_ERR_IS_Pos)                 /*!< SC_T::ISR: ACON_ERR_IS Mask               */

#define SC_TRSR_RX_OVER_F_Pos            (0)                                               /*!< SC_T::TRSR: RX_OVER_F Position            */
#define SC_TRSR_RX_OVER_F_Msk            (0x1ul << SC_TRSR_RX_OVER_F_Pos)                  /*!< SC_T::TRSR: RX_OVER_F Mask                */

#define SC_TRSR_RX_EMPTY_F_Pos           (1)                                               /*!< SC_T::TRSR: RX_EMPTY_F Position           */
#define SC_TRSR_RX_EMPTY_F_Msk           (0x1ul << SC_TRSR_RX_EMPTY_F_Pos)                 /*!< SC_T::TRSR: RX_EMPTY_F Mask               */

#define SC_TRSR_RX_FULL_F_Pos            (2)                                               /*!< SC_T::TRSR: RX_FULL_F Position            */
#define SC_TRSR_RX_FULL_F_Msk            (0x1ul << SC_TRSR_RX_FULL_F_Pos)                  /*!< SC_T::TRSR: RX_FULL_F Mask                */

#define SC_TRSR_RX_EPA_F_Pos             (4)                                               /*!< SC_T::TRSR: RX_EPA_F Position             */
#define SC_TRSR_RX_EPA_F_Msk             (0x1ul << SC_TRSR_RX_EPA_F_Pos)                   /*!< SC_T::TRSR: RX_EPA_F Mask                 */

#define SC_TRSR_RX_EFR_F_Pos             (5)                                               /*!< SC_T::TRSR: RX_EFR_F Position             */
#define SC_TRSR_RX_EFR_F_Msk             (0x1ul << SC_TRSR_RX_EFR_F_Pos)                   /*!< SC_T::TRSR: RX_EFR_F Mask                 */

#define SC_TRSR_RX_EBR_F_Pos             (6)                                               /*!< SC_T::TRSR: RX_EBR_F Position             */
#define SC_TRSR_RX_EBR_F_Msk             (0x1ul << SC_TRSR_RX_EBR_F_Pos)                   /*!< SC_T::TRSR: RX_EBR_F Mask                 */

#define SC_TRSR_TX_OVER_F_Pos            (8)                                               /*!< SC_T::TRSR: TX_OVER_F Position            */
#define SC_TRSR_TX_OVER_F_Msk            (0x1ul << SC_TRSR_TX_OVER_F_Pos)                  /*!< SC_T::TRSR: TX_OVER_F Mask                */

#define SC_TRSR_TX_EMPTY_F_Pos           (9)                                               /*!< SC_T::TRSR: TX_EMPTY_F Position           */
#define SC_TRSR_TX_EMPTY_F_Msk           (0x1ul << SC_TRSR_TX_EMPTY_F_Pos)                 /*!< SC_T::TRSR: TX_EMPTY_F Mask               */

#define SC_TRSR_TX_FULL_F_Pos            (10)                                              /*!< SC_T::TRSR: TX_FULL_F Position            */
#define SC_TRSR_TX_FULL_F_Msk            (0x1ul << SC_TRSR_TX_FULL_F_Pos)                  /*!< SC_T::TRSR: TX_FULL_F Mask                */

#define SC_TRSR_RX_POINT_F_Pos           (16)                                              /*!< SC_T::TRSR: RX_POINT_F Position           */
#define SC_TRSR_RX_POINT_F_Msk           (0x7ul << SC_TRSR_RX_POINT_F_Pos)                 /*!< SC_T::TRSR: RX_POINT_F Mask               */

#define SC_TRSR_RX_REERR_Pos             (21)                                              /*!< SC_T::TRSR: RX_REERR Position             */
#define SC_TRSR_RX_REERR_Msk             (0x1ul << SC_TRSR_RX_REERR_Pos)                   /*!< SC_T::TRSR: RX_REERR Mask                 */

#define SC_TRSR_RX_OVER_ERETRY_Pos       (22)                                              /*!< SC_T::TRSR: RX_OVER_ERETRY Position       */
#define SC_TRSR_RX_OVER_ERETRY_Msk       (0x1ul << SC_TRSR_RX_OVER_ERETRY_Pos)             /*!< SC_T::TRSR: RX_OVER_ERETRY Mask           */

#define SC_TRSR_RX_ATV_Pos               (23)                                              /*!< SC_T::TRSR: RX_ATV Position               */
#define SC_TRSR_RX_ATV_Msk               (0x1ul << SC_TRSR_RX_ATV_Pos)                     /*!< SC_T::TRSR: RX_ATV Mask                   */

#define SC_TRSR_TX_POINT_F_Pos           (24)                                              /*!< SC_T::TRSR: TX_POINT_F Position           */
#define SC_TRSR_TX_POINT_F_Msk           (0x7ul << SC_TRSR_TX_POINT_F_Pos)                 /*!< SC_T::TRSR: TX_POINT_F Mask               */

#define SC_TRSR_TX_REERR_Pos             (29)                                              /*!< SC_T::TRSR: TX_REERR Position             */
#define SC_TRSR_TX_REERR_Msk             (0x1ul << SC_TRSR_TX_REERR_Pos)                   /*!< SC_T::TRSR: TX_REERR Mask                 */

#define SC_TRSR_TX_OVER_ERETRY_Pos       (30)                                              /*!< SC_T::TRSR: TX_OVER_ERETRY Position       */
#define SC_TRSR_TX_OVER_ERETRY_Msk       (0x1ul << SC_TRSR_TX_OVER_ERETRY_Pos)             /*!< SC_T::TRSR: TX_OVER_ERETRY Mask           */

#define SC_TRSR_TX_ATV_Pos               (31)                                              /*!< SC_T::TRSR: TX_ATV Position               */
#define SC_TRSR_TX_ATV_Msk               (0x1ul << SC_TRSR_TX_ATV_Pos)                     /*!< SC_T::TRSR: TX_ATV Mask                   */

#define SC_PINCSR_POW_EN_Pos             (0)                                               /*!< SC_T::PINCSR: POW_EN Position             */
#define SC_PINCSR_POW_EN_Msk             (0x1ul << SC_PINCSR_POW_EN_Pos)                   /*!< SC_T::PINCSR: POW_EN Mask                 */

#define SC_PINCSR_SC_RST_Pos             (1)                                               /*!< SC_T::PINCSR: SC_RST Position             */
#define SC_PINCSR_SC_RST_Msk             (0x1ul << SC_PINCSR_SC_RST_Pos)                   /*!< SC_T::PINCSR: SC_RST Mask                 */

#define SC_PINCSR_CD_REM_F_Pos           (2)                                               /*!< SC_T::PINCSR: CD_REM_F Position           */
#define SC_PINCSR_CD_REM_F_Msk           (0x1ul << SC_PINCSR_CD_REM_F_Pos)                 /*!< SC_T::PINCSR: CD_REM_F Mask               */

#define SC_PINCSR_CD_INS_F_Pos           (3)                                               /*!< SC_T::PINCSR: CD_INS_F Position           */
#define SC_PINCSR_CD_INS_F_Msk           (0x1ul << SC_PINCSR_CD_INS_F_Pos)                 /*!< SC_T::PINCSR: CD_INS_F Mask               */

#define SC_PINCSR_CD_PIN_ST_Pos          (4)                                               /*!< SC_T::PINCSR: CD_PIN_ST Position          */
#define SC_PINCSR_CD_PIN_ST_Msk          (0x1ul << SC_PINCSR_CD_PIN_ST_Pos)                /*!< SC_T::PINCSR: CD_PIN_ST Mask              */

#define SC_PINCSR_CLK_KEEP_Pos           (6)                                               /*!< SC_T::PINCSR: CLK_KEEP Position           */
#define SC_PINCSR_CLK_KEEP_Msk           (0x1ul << SC_PINCSR_CLK_KEEP_Pos)                 /*!< SC_T::PINCSR: CLK_KEEP Mask               */

#define SC_PINCSR_ADAC_CD_EN_Pos         (7)                                               /*!< SC_T::PINCSR: ADAC_CD_EN Position         */
#define SC_PINCSR_ADAC_CD_EN_Msk         (0x1ul << SC_PINCSR_ADAC_CD_EN_Pos)               /*!< SC_T::PINCSR: ADAC_CD_EN Mask             */

#define SC_PINCSR_SC_OEN_ST_Pos          (8)                                               /*!< SC_T::PINCSR: SC_OEN_ST Position          */
#define SC_PINCSR_SC_OEN_ST_Msk          (0x1ul << SC_PINCSR_SC_OEN_ST_Pos)                /*!< SC_T::PINCSR: SC_OEN_ST Mask              */

#define SC_PINCSR_SC_DATA_O_Pos          (9)                                               /*!< SC_T::PINCSR: SC_DATA_O Position          */
#define SC_PINCSR_SC_DATA_O_Msk          (0x1ul << SC_PINCSR_SC_DATA_O_Pos)                /*!< SC_T::PINCSR: SC_DATA_O Mask              */

#define SC_PINCSR_CD_LEV_Pos             (10)                                              /*!< SC_T::PINCSR: CD_LEV Position             */
#define SC_PINCSR_CD_LEV_Msk             (0x1ul << SC_PINCSR_CD_LEV_Pos)                   /*!< SC_T::PINCSR: CD_LEV Mask                 */

#define SC_PINCSR_POW_INV_Pos            (11)                                              /*!< SC_T::PINCSR: POW_INV Position            */
#define SC_PINCSR_POW_INV_Msk            (0x1ul << SC_PINCSR_POW_INV_Pos)                  /*!< SC_T::PINCSR: POW_INV Mask                */

#define SC_PINCSR_SC_DATA_I_ST_Pos       (16)                                              /*!< SC_T::PINCSR: SC_DATA_I_ST Position       */
#define SC_PINCSR_SC_DATA_I_ST_Msk       (0x1ul << SC_PINCSR_SC_DATA_I_ST_Pos)             /*!< SC_T::PINCSR: SC_DATA_I_ST Mask           */

#define SC_TMR0_CNT_Pos                  (0)                                               /*!< SC_T::TMR0: CNT Position                  */
#define SC_TMR0_CNT_Msk                  (0xfffffful << SC_TMR0_CNT_Pos)                   /*!< SC_T::TMR0: CNT Mask                      */

#define SC_TMR0_MODE_Pos                 (24)                                              /*!< SC_T::TMR0: MODE Position                 */
#define SC_TMR0_MODE_Msk                 (0xful << SC_TMR0_MODE_Pos)                       /*!< SC_T::TMR0: MODE Mask                     */

#define SC_TMR1_CNT_Pos                  (0)                                               /*!< SC_T::TMR1: CNT Position                  */
#define SC_TMR1_CNT_Msk                  (0xfful << SC_TMR1_CNT_Pos)                       /*!< SC_T::TMR1: CNT Mask                      */

#define SC_TMR1_MODE_Pos                 (24)                                              /*!< SC_T::TMR1: MODE Position                 */
#define SC_TMR1_MODE_Msk                 (0xful << SC_TMR1_MODE_Pos)                       /*!< SC_T::TMR1: MODE Mask                     */

#define SC_TMR2_CNT_Pos                  (0)                                               /*!< SC_T::TMR2: CNT Position                  */
#define SC_TMR2_CNT_Msk                  (0xfful << SC_TMR2_CNT_Pos)                       /*!< SC_T::TMR2: CNT Mask                      */

#define SC_TMR2_MODE_Pos                 (24)                                              /*!< SC_T::TMR2: MODE Position                 */
#define SC_TMR2_MODE_Msk                 (0xful << SC_TMR2_MODE_Pos)                       /*!< SC_T::TMR2: MODE Mask                     */

#define SC_UACTL_UA_MODE_EN_Pos          (0)                                               /*!< SC_T::UACTL: UA_MODE_EN Position          */
#define SC_UACTL_UA_MODE_EN_Msk          (0x1ul << SC_UACTL_UA_MODE_EN_Pos)                /*!< SC_T::UACTL: UA_MODE_EN Mask              */

#define SC_UACTL_DATA_LEN_Pos            (4)                                               /*!< SC_T::UACTL: DATA_LEN Position            */
#define SC_UACTL_DATA_LEN_Msk            (0x3ul << SC_UACTL_DATA_LEN_Pos)                  /*!< SC_T::UACTL: DATA_LEN Mask                */

#define SC_UACTL_PBDIS_Pos               (6)                                               /*!< SC_T::UACTL: PBDIS Position               */
#define SC_UACTL_PBDIS_Msk               (0x1ul << SC_UACTL_PBDIS_Pos)                     /*!< SC_T::UACTL: PBDIS Mask                   */

#define SC_UACTL_OPE_Pos                 (7)                                               /*!< SC_T::UACTL: OPE Position                 */
#define SC_UACTL_OPE_Msk                 (0x1ul << SC_UACTL_OPE_Pos)                       /*!< SC_T::UACTL: OPE Mask                     */

#define SC_TDRA_TDR0_Pos                 (0)                                               /*!< SC_T::TDRA: TDR0 Position                 */
#define SC_TDRA_TDR0_Msk                 (0xfffffful << SC_TDRA_TDR0_Pos)                  /*!< SC_T::TDRA: TDR0 Mask                     */

#define SC_TDRB_TDR1_Pos                 (0)                                               /*!< SC_T::TDRB: TDR1 Position                 */
#define SC_TDRB_TDR1_Msk                 (0xfful << SC_TDRB_TDR1_Pos)                      /*!< SC_T::TDRB: TDR1 Mask                     */

#define SC_TDRB_TDR2_Pos                 (8)                                               /*!< SC_T::TDRB: TDR2 Position                 */
#define SC_TDRB_TDR2_Msk                 (0xfful << SC_TDRB_TDR2_Pos)                      /*!< SC_T::TDRB: TDR2 Mask                     */

/**@}*/ /* SC_CONST */
/**@}*/ /* end of SC register group */


/*---------------------- Serial Peripheral Interface Controller -------------------------*/
/**
    @addtogroup SPI Serial Peripheral Interface Controller(SPI)
    Memory Mapped Structure for SPI Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTL</font><br><p> <font size="2">
Offset: 0x00  SPI Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>GO_BUSY</td><td><div style="word-wrap: break-word;"><b>SPI Transfer Control Bit And Busy Status</b><br>
0 = Writing this bit "0" will stop data transfer if SPI is transferring.<br>
1 = In Master mode, writing "1" to this bit will start the SPI data transfer; In Slave mode, writing '1' to this bit indicates that the salve is ready to communicate with a master.<br>
If the FIFO mode is disabled, during the data transfer, this bit keeps the value of '1'.<br>
As the transfer is finished, this bit will be cleared automatically.<br>
Software can read this bit to check if the SPI is in busy status.<br>
In FIFO mode, this bit will be controlled by hardware.<br>
Software should not modify this bit.<br>
In slave mode, this bit always returns 1 when software reads this register.<br>
In master mode, this bit reflects the busy or idle status of SPI.<br>
Note:<br>
1. When FIFO mode is disabled, all configurations should be set before writing "1" to the GO_BUSY bit in the SPI_CTL register.<br>
2. When FIFO bit is disabled and the software uses TX or RX PDMA function to transfer data, this bit will be cleared after the PDMA controller finishes the data transfer.<br>
</div></td></tr><tr><td>
[1]</td><td>RX_NEG</td><td><div style="word-wrap: break-word;"><b>Receive At Negative Edge</b><br>
0 = The received data is latched on the rising edge of SPI_SCLK.<br>
1 = The received data is latched on the falling edge of SPI_SCLK.<br>
</div></td></tr><tr><td>
[2]</td><td>TX_NEG</td><td><div style="word-wrap: break-word;"><b>Transmit At Negative Edge</b><br>
0 = The transmitted data output is changed on the rising edge of SPI_SCLK.<br>
1 = The transmitted data output is changed on the falling edge of SPI_SCLK.<br>
</div></td></tr><tr><td>
[7:3]</td><td>TX_BIT_LEN</td><td><div style="word-wrap: break-word;"><b>Transmit Bit Length</b><br>
This field specifies how many bits can be transmitted / received in one transaction.<br>
The minimum bit length is 8 bits and can be up to 32 bits.<br>
TX_BIT_LEN   Description<br>
01000        8 bits are transmitted in one transaction<br>
01001        9 bits are transmitted in one transaction<br>
------       ----------<br>
11111        31 bits are transmitted in one transaction<br>
00000        32 bits are transmitted in one transaction<br>
</div></td></tr><tr><td>
[10]</td><td>LSB</td><td><div style="word-wrap: break-word;"><b>Send LSB First</b><br>
0 = The MSB, which bit of transmit/receive register depends on the setting of TX_BITLEN, is transmitted/received first.<br>
1 = The LSB, bit 0 of the SPI_TX0/1, is sent first to the the SPI data output pin, and the first bit received from the SPI data input pin will be put in the LSB position of the SPI_RX register (SPI_RX0/1).<br>
</div></td></tr><tr><td>
[11]</td><td>CLKP</td><td><div style="word-wrap: break-word;"><b>Clock Polarity</b><br>
0 = The default level of SCLK is low in idle state.<br>
1 = The default level of SCLK is high in idle state.<br>
</div></td></tr><tr><td>
[15:12]</td><td>SP_CYCLE</td><td><div style="word-wrap: break-word;"><b>Suspend Interval (Master Only)</b><br>
These four bits provide configurable suspend interval between two successive transmit/receive transaction in a transfer.<br>
The suspend interval is from the last falling clock edge of the current transaction to the first rising clock edge of the successive transaction if CLKP = "0".<br>
If CLKP = "1", the interval is from the rising clock edge to the falling clock edge.<br>
The default value is 0x3. The desired suspend interval is obtained according to the following equation:<br>
(SP_CYCLE[3:0) + 0.5) * period of SPICLK<br>
Ex:<br>
SP_CYCLE = 0x0 ... 0.5 SPICLK clock cycle.<br>
SP_CYCLE = 0x1 ... 1.5 SPICLK clock cycle.<br>
......<br>
SP_CYCLE = 0xE ... 14.5 SPICLK clock cycle.<br>
SP_CYCLE = 0xF ... 15.5 SPICLK clock cycle.<br>
If the Variable Clock function is enabled, the minimum period of suspend interval (the transmit data in FIFO buffer is not empty) between the successive transaction is (6.5 + SP_CYCLE) * SPICLK clock cycle<br>
</div></td></tr><tr><td>
[17]</td><td>INTEN</td><td><div style="word-wrap: break-word;"><b>Interrupt Enable</b><br>
0 = SPI Interrupt Disabled.<br>
1 = SPI Interrupt Enabled.<br>
</div></td></tr><tr><td>
[18]</td><td>SLAVE</td><td><div style="word-wrap: break-word;"><b>Slave Mode</b><br>
0 = SPI controller set as Master mode.<br>
1 = SPI controller set as Slave mode.<br>
</div></td></tr><tr><td>
[19]</td><td>REORDER</td><td><div style="word-wrap: break-word;"><b>Byte Reorder Function Enable</b><br>
0 = Disable byte reorder function<br>
1 = Enable byte reorder function and insert a byte suspend interval among each byte.<br>
The setting of TX_BIT_LEN must be configured as 00b ( 32 bits/ word).<br>
The suspend interval is defined in SP_CYCLE.<br>
Note:<br>
1. The byte reorder function is only available if TX_BIT_LEN is defined as 16, 24, and 32 bits.<br>
2. In Slave mode with level-trigger configuration, if the byte suspend function is enabled, the slave select pin must be kept at active state during the successive four bytes transfer.<br>
3. The byte reorder function is not supported when the variable serial clock function or the dual I/O mode is enabled.<br>
</div></td></tr><tr><td>
[21]</td><td>FIFOM</td><td><div style="word-wrap: break-word;"><b>FIFO Mode Enable</b><br>
0 = Normal mode.<br>
1 = FIFO mode.<br>
Note:<br>
1. Before enabling FIFO mode, the other related settings should be set in advance.<br>
2. In Master mode, if the FIFO mode is enabled, the GO_BUSY bit will be set "1" automatically after the data was written into the 8-depth FIFO.<br>
The user can clear this FIFO bit after the transmit FIFO status is empty and the GO_BUSY back to 0.<br>
</div></td></tr><tr><td>
[22]</td><td>TWOB</td><td><div style="word-wrap: break-word;"><b>2-Bit Transfer Mode Active</b><br>
0 = 2-bit transfer mode Disabled.<br>
1 = 2-bit transfer mode Enabled.<br>
Note that when enabling TWOB, the serial transmitted 2-bits data are from SPI_TX1/0, and the received 2-bits data input are put into SPI_RX1/0.<br>
</div></td></tr><tr><td>
[23]</td><td>VARCLK_EN</td><td><div style="word-wrap: break-word;"><b>Variable Clock Enable</b><br>
0 = The serial clock output frequency is fixed and only decided by the value of DIVIDER1<br>
1 = The serial clock output frequency is variable.<br>
The output frequency is decided by the value of VARCLK (SPI_VARCLK), DIVIDER1, and DIVIDER2.<br>
Note: When this VARCLK_EN bit is set to 1, the setting of TX_BIT_LEN must be programmed as 0x10 (16-bit mode).<br>
</div></td></tr><tr><td>
[28]</td><td>DUAL_IO_DIR</td><td><div style="word-wrap: break-word;"><b>Dual IO Mode Direction</b><br>
0 = Date read in the Dual I/O Mode function.<br>
1 = Data write in the Dual I/O Mode function.<br>
</div></td></tr><tr><td>
[29]</td><td>DUAL_IO_EN</td><td><div style="word-wrap: break-word;"><b>Dual IO Mode Enable</b><br>
0 = Dual I/O Mode function Disabled.<br>
1 = Dual I/O Mode function Enabled.<br>
</div></td></tr><tr><td>
[31]</td><td>WKEUP_EN</td><td><div style="word-wrap: break-word;"><b>Wake-Up Enable</b><br>
0 = Wake-up function Disabled when the system enters Power-down mode.<br>
1 = Wake-up function Enabled.<br>
When the system enters Power-down mode, the system can be wake-up from the SPI controller when this bit is enabled and if there is any toggle in the SPICLK port.<br>
After the system wake-up, this bit must be cleared by user to disable the wake-up requirement.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">STATUS</font><br><p> <font size="2">
Offset: 0x04  SPI Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RX_EMPTY</td><td><div style="word-wrap: break-word;"><b>Received FIFO_EMPTY Status</b><br>
0 = Received data FIFO is not empty in the dual FIFO mode.<br>
1 = Received data FIFO is empty in the dual FIFO mode.<br>
</div></td></tr><tr><td>
[1]</td><td>RX_FULL</td><td><div style="word-wrap: break-word;"><b>Received FIFO_FULL Status</b><br>
0 = Received data FIFO is not full in dual FIFO mode.<br>
1 = Received data FIFO is full in the dual FIFO mode.<br>
</div></td></tr><tr><td>
[2]</td><td>TX_EMPTY</td><td><div style="word-wrap: break-word;"><b>Transmitted FIFO_EMPTY Status</b><br>
0 = Transmitted data FIFO is not empty in the dual FIFO mode.<br>
1 =Transmitted data FIFO is empty in the dual FIFO mode.<br>
</div></td></tr><tr><td>
[3]</td><td>TX_FULL</td><td><div style="word-wrap: break-word;"><b>Transmitted FIFO_FULL Status</b><br>
0 = Transmitted data FIFO is not full in the dual FIFO mode.<br>
1 = Transmitted data FIFO is full in the dual FIFO mode.<br>
</div></td></tr><tr><td>
[4]</td><td>LTRIG_FLAG</td><td><div style="word-wrap: break-word;"><b>Level Trigger Accomplish Flag </b><br>
In Slave mode, this bit indicates whether the received bit number meets the requirement or not after the current transaction done.<br>
0 = The transferred bit length of one transaction does not meet the specified requirement.<br>
1 = The transferred bit length meets the specified requirement which defined in TX_BIT_LEN.<br>
Note: This bit is READ only.<br>
As the software sets the GO_BUSY bit to 1, the LTRIG_FLAG will be cleared to 0 after 4 SPI engine clock periods plus 1 system clock period.<br>
In FIFO mode, this bit is unmeaning.<br>
</div></td></tr><tr><td>
[6]</td><td>SLV_START_INTSTS</td><td><div style="word-wrap: break-word;"><b>Slave Start Interrupt Status</b><br>
It is used to dedicate that the transfer has started in Slave mode with no slave select.<br>
0 = Slave started transfer no active.<br>
1 = Transfer has started in Slave mode with no slave select.<br>
It is auto clear by transfer done or writing one clear.<br>
</div></td></tr><tr><td>
[7]</td><td>INTSTS</td><td><div style="word-wrap: break-word;"><b>Interrupt Status</b><br>
0 = Transfer is not finished yet.<br>
1 = Transfer is done. The interrupt is requested when the INTEN bit is enabled.<br>
Note: This bit is read only, but can be cleared by writing "1" to this bit.<br>
</div></td></tr><tr><td>
[8]</td><td>RXINT_STS</td><td><div style="word-wrap: break-word;"><b>RX FIFO Threshold Interrupt Status (Read Only)</b><br>
0 = RX valid data counts small or equal than RXTHRESHOLD.<br>
1 = RX valid data counts bigger than RXTHRESHOLD.<br>
Note: If RXINT_EN = 1 and RX_INTSTS = 1, SPI will generate interrupt.<br>
</div></td></tr><tr><td>
[9]</td><td>RX_OVER_RUN</td><td><div style="word-wrap: break-word;"><b>RX FIFO Over Run Status</b><br>
If SPI receives data when RX FIFO is full, this bit will set to 1, and the received data will dropped.<br>
Note: This bit will be cleared by writing 1 to itself.<br>
</div></td></tr><tr><td>
[10]</td><td>TXINT_STS</td><td><div style="word-wrap: break-word;"><b>TX FIFO Threshold Interrupt Status (Read Only)</b><br>
0 = TX valid data counts bigger than TXTHRESHOLD.<br>
1 = TX valid data counts small or equal than TXTHRESHOLD.<br>
</div></td></tr><tr><td>
[12]</td><td>TIME_OUT_STS</td><td><div style="word-wrap: break-word;"><b>TIMEOUT Interrupt Flag</b><br>
0 = There is not timeout event on the received buffer.<br>
1 = RX FIFO is not empty and there is not be read over the 64 SPI_CLK period in master mode and over the 576 ECLK period in slave mode.<br>
When the received FIFO is read by user, the timeout status will be cleared automatically.<br>
Note: This bit will be cleared by writing 1 to itself.<br>
</div></td></tr><tr><td>
[19:16]</td><td>RX_FIFO_CNT</td><td><div style="word-wrap: break-word;"><b>Data counts in RX FIFO (Read Only)</b><br>
</div></td></tr><tr><td>
[23:20]</td><td>TX_FIFO_CNT</td><td><div style="word-wrap: break-word;"><b>Data counts in TX FIFO (Read Only)</b><br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t STATUS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CLKDIV</font><br><p> <font size="2">
Offset: 0x08  SPI Clock Divider Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>DIVIDER1</td><td><div style="word-wrap: break-word;"><b>Clock Divider 1 Register</b><br>
The value in this field is the 1th frequency divider of the PCLK to generate the serial clock of SPI_SCLK.<br>
The desired frequency is obtained according to the following equation:<br>
Where<br>
is the SPI engine clock source. It is defined in the CLK_SEL1.<br>
</div></td></tr><tr><td>
[23:16]</td><td>DIVIDER2</td><td><div style="word-wrap: break-word;"><b>Clock Divider 2 Register</b><br>
The value in this field is the 2nd frequency divider of the PCLK to generate the serial clock of SPI_SCLK.<br>
The desired frequency is obtained according to the following equation:<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CLKDIV;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">SSR</font><br><p> <font size="2">
Offset: 0x0C  SPI Slave Select Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[1:0]</td><td>SSR</td><td><div style="word-wrap: break-word;"><b>Slave Select Active Register (Master Only)</b><br>
If AUTOSS bit is cleared, writing "1" to SSR[0] bit sets the SPISS[0] line to an active state and writing "0" sets the line back to inactive state.(the same as SSR[1] for SPISS[1])<br>
If AUTOSS bit is set, writing "1" to any bit location of this field will select appropriate SPISS[1:0] line to be automatically driven to active state for the duration of the transaction, and will be driven to inactive state for the rest of the time.<br>
(The active level of SPISS[1:0] is specified in SS_LVL).<br>
Note:<br>
1. This interface can only drive one device/slave at a given time.<br>
Therefore, the slaves select of the selected device must be set to its active level before starting any read or write transfer.<br>
2. SPISS[0] is also defined as device/slave select input in Slave mode.<br>
And that the slave select input must be driven by edge active trigger which level depend on the SS_LVL setting, otherwise the SPI slave core will go into dead path until the edge active triggers again or reset the SPI core by software.<br>
</div></td></tr><tr><td>
[2]</td><td>SS_LVL</td><td><div style="word-wrap: break-word;"><b>Slave Select Active Level</b><br>
It defines the active level of device/slave select signal (SPISS[1:0]).<br>
0 = The SPI_SS slave select signal is active Low.<br>
1 = The SPI_SS slave select signal is active High.<br>
</div></td></tr><tr><td>
[3]</td><td>AUTOSS</td><td><div style="word-wrap: break-word;"><b>Automatic Slave Selection (Master Only)</b><br>
0 = If this bit is set as "0", slave select signals are asserted and de-asserted by setting and clearing related bits in SSR[1:0] register.<br>
1 = If this bit is set as "1", SPISS[1:0] signals are generated automatically.<br>
It means that device/slave select signal, which is set in SSR[1:0] register is asserted by the SPI controller when transmit/receive is started, and is de-asserted after each transaction is done.<br>
</div></td></tr><tr><td>
[4]</td><td>SS_LTRIG</td><td><div style="word-wrap: break-word;"><b>Slave Select Level Trigger</b><br>
0 = The input slave select signal is edge-trigger.<br>
1 = The slave select signal will be level-trigger.<br>
It depends on SS_LVL to decide the signal is active low or active high.<br>
</div></td></tr><tr><td>
[5]</td><td>NOSLVSEL</td><td><div style="word-wrap: break-word;"><b>No Slave Selected In Slave Mode</b><br>
This is used to ignore the slave select signal in Slave mode.<br>
The SPI controller can work on 3 wire interface including SPICLK, SPI_MISO, and SPI_MOSI when it is set as a slave device.<br>
0 = The controller is 4-wire bi-direction interface.<br>
1 = The controller is 3-wire bi-direction interface in Slave mode.<br>
When this bit is set as 1, the controller start to transmit/receive data after the GO_BUSY bit active and the serial clock input.<br>
Note: In no slave select signal mode, the SS_LTRIG, SPI_SSR[4], shall be set as "1".<br>
</div></td></tr><tr><td>
[8]</td><td>SLV_ABORT</td><td><div style="word-wrap: break-word;"><b>Abort In Slave Mode With No Slave Selected</b><br>
In normal operation, there is interrupt event when the received data meet the required bits which define in TX_BIT_LEN.<br>
If the received bits are less than the requirement and there is no more serial clock input over the time period which is defined by user in slave mode with no slave select, the user can set this bit to force the current transfer done and then the user can get a transfer done interrupt event.<br>
Note: It is auto cleared to "0" by hardware when the abort event is active.<br>
</div></td></tr><tr><td>
[9]</td><td>SSTA_INTEN</td><td><div style="word-wrap: break-word;"><b>Slave Start Interrupt Enable</b><br>
It is used to enable interrupt when the transfer has started in Slave mode with no slave select.<br>
If there is no transfer done interrupt over the time period which is defined by user after the transfer start, the user can set the SLV_ABORT bit to force the transfer done.<br>
0 = Tansfer start interrupt Disabled.<br>
1 = Transaction start interrupt Enabled.<br>
It is cleared when the current transfer done or the SLV_START_INTSTS bit cleared (write 1 clear).<br>
</div></td></tr><tr><td>
[16]</td><td>SS_INT_OPT</td><td><div style="word-wrap: break-word;"><b>Slave Select Interrupt Option</b><br>
It is used to enable the interrupt when the transfer has done in slave mode.<br>
0 = No any interrupt, even there is slave select inactive event.<br>
1 = There is interrupt event when the slave select is inactive.<br>
It is used to inform the user the transaction has finished and the slave select into the inactive state.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t SSR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RX0</font><br><p> <font size="2">
Offset: 0x10  SPI Receive Data FIFO Register 0</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>RDATA</td><td><div style="word-wrap: break-word;"><b>Receive Data FIFO Register</b><br>
The received data can be read on it.<br>
If the FIFO bit is set as 1, the user also checks the RX_EMPTY, SPI_STATUS[0], to check if there is any more received data or not.<br>
Note: These registers are read only.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t RX0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RX1</font><br><p> <font size="2">
Offset: 0x14  SPI Receive Data FIFO Register 1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>RDATA</td><td><div style="word-wrap: break-word;"><b>Receive Data FIFO Register</b><br>
The received data can be read on it.<br>
If the FIFO bit is set as 1, the user also checks the RX_EMPTY, SPI_STATUS[0], to check if there is any more received data or not.<br>
Note: These registers are read only.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t RX1;
    uint32_t RESERVE0[2];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TX0</font><br><p> <font size="2">
Offset: 0x20  SPI Transmit Data FIFO Register 0</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>TDATA</td><td><div style="word-wrap: break-word;"><b>Transmit Data FIFO Register</b><br>
The Data Transmit Registers hold the data to be transmitted in the next transfer.<br>
The number of valid bits depends on the setting of transmit bit length field of the SPI_CTL register.<br>
For example, if TX_BIT_LEN is set to 0x08, the bit SPI_TX[7:0] will be transmitted in next transfer.<br>
If TX_BIT_LEN is set to 0x00, the SPI controller will perform a 32-bit transfer.<br>
Note: When the SPI controller is configured as a slave device and the FIFO mode is disabled, if the SPI controller attempts to transmit data to a master, the software must update the transmit data register before setting the GO_BUSY bit to 1<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __O  uint32_t TX0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TX1</font><br><p> <font size="2">
Offset: 0x24  SPI Transmit Data FIFO Register 1</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>TDATA</td><td><div style="word-wrap: break-word;"><b>Transmit Data FIFO Register</b><br>
The Data Transmit Registers hold the data to be transmitted in the next transfer.<br>
The number of valid bits depends on the setting of transmit bit length field of the SPI_CTL register.<br>
For example, if TX_BIT_LEN is set to 0x08, the bit SPI_TX[7:0] will be transmitted in next transfer.<br>
If TX_BIT_LEN is set to 0x00, the SPI controller will perform a 32-bit transfer.<br>
Note: When the SPI controller is configured as a slave device and the FIFO mode is disabled, if the SPI controller attempts to transmit data to a master, the software must update the transmit data register before setting the GO_BUSY bit to 1<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __O  uint32_t TX1;
    uint32_t RESERVE1[3];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">VARCLK</font><br><p> <font size="2">
Offset: 0x34  SPI Variable Clock Pattern Flag Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>VARCLK</td><td><div style="word-wrap: break-word;"><b>Variable Clock Pattern Flag</b><br>
The value in this field is the frequency patterns of the SPICLK.<br>
If the bit pattern of VARCLK is '0', the output frequency of SPICLK is according the value of DIVIDER1.<br>
If the bit patterns of VARCLK are '1', the output frequency of SPICLK is according the value of DIVIDER2.<br>
Note: It is used for CLKP = 0 only.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t VARCLK;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DMA</font><br><p> <font size="2">
Offset: 0x38  SPI DMA Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TX_DMA_EN</td><td><div style="word-wrap: break-word;"><b>Transmit PDMA Enable (PDMA Writes Data To SPI)</b><br>
Set this bit to 1 will start the transmit PDMA process.<br>
SPI controller will issue request to PDMA controller automatically.<br>
If using PDMA mode to transfer data, remember not to set GO_BUSY bit of SPI_CNTRL register.<br>
The DMA controller inside SPI controller will set it automatically whenever necessary.<br>
Note:<br>
1. Two transaction need minimal 18 APB clock + 8 SPI serial clocks suspend interval in master mode for edge mode and 18 APB clock + 9.5 serial clocks for level mode.<br>
2. If the 2-bit function is enabled, the requirement timing shall append 18 APB clock based on the above clock period.<br>
Hardware will clear this bit to 0 automatically after PDMA transfer done. If FIFO mode not release, it should be remove.<br>
</div></td></tr><tr><td>
[1]</td><td>RX_DMA_EN</td><td><div style="word-wrap: break-word;"><b>Receiving PDMA Enable(PDMA Reads SPI Data To Memory)</b><br>
Set this bit to "1" will start the receive PDMA process.<br>
SPI controller will issue request to PDMA controller automatically when there is data written into the received buffer or the status of RX_EMPTY status is set to 0 in FIFO mode.<br>
If using the RX_PDMA mode to receive data but TX_DMA is disabled, the GO_BUSY bit shall be set by user.<br>
Hardware will clear this bit to 0 automatically after PDMA transfer done.<br>
In Slave mode and the FIFO bit is disabled, if the receive PDMA is enabled but the transmit PDMA is disabled, the minimal suspend interval between two successive transactions input is need to be larger than 9 SPI slave engine clock + 4 APB clock for edge mode and 9.5 SPI slave engine clock + 4 APB clock<br>
</div></td></tr><tr><td>
[2]</td><td>PDMA_RST</td><td><div style="word-wrap: break-word;"><b>PDMA Reset</b><br>
It is used to reset the SPI PDMA function into default state.<br>
0 = After reset PDMA function or in normal operation.<br>
1 = Reset PDMA function.<br>
Note: it is auto cleared to "0" after the reset function done.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t DMA;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">FFCTL</font><br><p> <font size="2">
Offset: 0x3C  SPI FIFO Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RX_CLR</td><td><div style="word-wrap: break-word;"><b>Receiving FIFO Counter Clear</b><br>
This bit is used to clear the receiver counter in FIFO Mode.<br>
This bit can be written "1" to clear the receiver counter and this bit will be cleared to "0" automatically after clearing receiving counter.<br>
After the clear operation, the flag of RX_EMPTY in SPI_STATUS[0] will be set to "1".<br>
</div></td></tr><tr><td>
[1]</td><td>TX_CLR</td><td><div style="word-wrap: break-word;"><b>Transmitting FIFO Counter Clear</b><br>
This bit is used to clear the transmit counter in FIFO Mode.<br>
This bit can be written "1" to clear the transmitting counter and this bit will be cleared to "0" automatically after clearing transmitting counter.<br>
After the clear operation, the flag of TX_EMPTY in SPI_STATUS[2] will be set to "1".<br>
</div></td></tr><tr><td>
[2]</td><td>RXINT_EN</td><td><div style="word-wrap: break-word;"><b>RX Threshold Interrupt Enable</b><br>
0 = Rx threshold interrupt Disabled.<br>
1 = RX threshold interrupt Enable.<br>
</div></td></tr><tr><td>
[3]</td><td>TXINT_EN</td><td><div style="word-wrap: break-word;"><b>TX Threshold Interrupt Enable</b><br>
0 = Tx threshold interrupt Disabled.<br>
1 = TX threshold interrupt Enable.<br>
</div></td></tr><tr><td>
[4]</td><td>RXOVINT_EN</td><td><div style="word-wrap: break-word;"><b>RX FIFO Over Run Interrupt Enable</b><br>
0 = RX FIFO over run interrupt Disabled.<br>
1 = RX FIFO over run interrupt Enabled.<br>
</div></td></tr><tr><td>
[7]</td><td>TIMEOUT_EN</td><td><div style="word-wrap: break-word;"><b>RX Read Timeout Function Enable</b><br>
0 = RX read Timeout function Disabled.<br>
1 = RX read Timeout function Enabled.<br>
</div></td></tr><tr><td>
[26:24]</td><td>RX_THRESHOLD</td><td><div style="word-wrap: break-word;"><b>Received FIFO Threshold</b><br>
3-bits register, value from 0 ~7.<br>
If RX valid data counts large than RXTHRESHOLD, RXINT_STS will set to 1, else RXINT_STS will set to 0.<br>
</div></td></tr><tr><td>
[30:28]</td><td>TX_THRESHOLD</td><td><div style="word-wrap: break-word;"><b>Transmit FIFO Threshold</b><br>
3-bit register, value from 0 ~7.<br>
If TX valid data counts small or equal than TXTHRESHOLD, TXINT_STS will set to 1, else TXINT_STS will set to 0<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t FFCTL;
} SPI_T;

/**
    @addtogroup SPI_CONST SPI Bit Field Definition
    Constant Definitions for SPI Controller
@{ */

#define SPI_CTL_GO_BUSY_Pos              (0)                                               /*!< SPI_T::CTL: GO_BUSY Position              */
#define SPI_CTL_GO_BUSY_Msk              (0x1ul << SPI_CTL_GO_BUSY_Pos)                    /*!< SPI_T::CTL: GO_BUSY Mask                  */

#define SPI_CTL_RX_NEG_Pos               (1)                                               /*!< SPI_T::CTL: RX_NEG Position               */
#define SPI_CTL_RX_NEG_Msk               (0x1ul << SPI_CTL_RX_NEG_Pos)                     /*!< SPI_T::CTL: RX_NEG Mask                   */

#define SPI_CTL_TX_NEG_Pos               (2)                                               /*!< SPI_T::CTL: TX_NEG Position               */
#define SPI_CTL_TX_NEG_Msk               (0x1ul << SPI_CTL_TX_NEG_Pos)                     /*!< SPI_T::CTL: TX_NEG Mask                   */

#define SPI_CTL_TX_BIT_LEN_Pos           (3)                                               /*!< SPI_T::CTL: TX_BIT_LEN Position           */
#define SPI_CTL_TX_BIT_LEN_Msk           (0x1ful << SPI_CTL_TX_BIT_LEN_Pos)                /*!< SPI_T::CTL: TX_BIT_LEN Mask               */

#define SPI_CTL_LSB_Pos                  (10)                                              /*!< SPI_T::CTL: LSB Position                  */
#define SPI_CTL_LSB_Msk                  (0x1ul << SPI_CTL_LSB_Pos)                        /*!< SPI_T::CTL: LSB Mask                      */

#define SPI_CTL_CLKP_Pos                 (11)                                              /*!< SPI_T::CTL: CLKP Position                 */
#define SPI_CTL_CLKP_Msk                 (0x1ul << SPI_CTL_CLKP_Pos)                       /*!< SPI_T::CTL: CLKP Mask                     */

#define SPI_CTL_SP_CYCLE_Pos             (12)                                              /*!< SPI_T::CTL: SP_CYCLE Position             */
#define SPI_CTL_SP_CYCLE_Msk             (0xful << SPI_CTL_SP_CYCLE_Pos)                   /*!< SPI_T::CTL: SP_CYCLE Mask                 */

#define SPI_CTL_INTEN_Pos                (17)                                              /*!< SPI_T::CTL: INTEN Position                */
#define SPI_CTL_INTEN_Msk                (0x1ul << SPI_CTL_INTEN_Pos)                      /*!< SPI_T::CTL: INTEN Mask                    */

#define SPI_CTL_SLAVE_Pos                (18)                                              /*!< SPI_T::CTL: SLAVE Position                */
#define SPI_CTL_SLAVE_Msk                (0x1ul << SPI_CTL_SLAVE_Pos)                      /*!< SPI_T::CTL: SLAVE Mask                    */

#define SPI_CTL_REORDER_Pos              (19)                                              /*!< SPI_T::CTL: REORDER Position              */
#define SPI_CTL_REORDER_Msk              (0x1ul << SPI_CTL_REORDER_Pos)                    /*!< SPI_T::CTL: REORDER Mask                  */

#define SPI_CTL_FIFOM_Pos                (21)                                              /*!< SPI_T::CTL: FIFOM Position                */
#define SPI_CTL_FIFOM_Msk                (0x1ul << SPI_CTL_FIFOM_Pos)                      /*!< SPI_T::CTL: FIFOM Mask                    */

#define SPI_CTL_TWOB_Pos                 (22)                                              /*!< SPI_T::CTL: TWOB Position                 */
#define SPI_CTL_TWOB_Msk                 (0x1ul << SPI_CTL_TWOB_Pos)                       /*!< SPI_T::CTL: TWOB Mask                     */

#define SPI_CTL_VARCLK_EN_Pos            (23)                                              /*!< SPI_T::CTL: VARCLK_EN Position            */
#define SPI_CTL_VARCLK_EN_Msk            (0x1ul << SPI_CTL_VARCLK_EN_Pos)                  /*!< SPI_T::CTL: VARCLK_EN Mask                */

#define SPI_CTL_DUAL_IO_DIR_Pos          (28)                                              /*!< SPI_T::CTL: DUAL_IO_DIR Position          */
#define SPI_CTL_DUAL_IO_DIR_Msk          (0x1ul << SPI_CTL_DUAL_IO_DIR_Pos)                /*!< SPI_T::CTL: DUAL_IO_DIR Mask              */

#define SPI_CTL_DUAL_IO_EN_Pos           (29)                                              /*!< SPI_T::CTL: DUAL_IO_EN Position           */
#define SPI_CTL_DUAL_IO_EN_Msk           (0x1ul << SPI_CTL_DUAL_IO_EN_Pos)                 /*!< SPI_T::CTL: DUAL_IO_EN Mask               */

#define SPI_CTL_WKEUP_EN_Pos             (31)                                              /*!< SPI_T::CTL: WKEUP_EN Position             */
#define SPI_CTL_WKEUP_EN_Msk             (0x1ul << SPI_CTL_WKEUP_EN_Pos)                   /*!< SPI_T::CTL: WKEUP_EN Mask                 */

#define SPI_STATUS_RX_EMPTY_Pos          (0)                                               /*!< SPI_T::STATUS: RX_EMPTY Position          */
#define SPI_STATUS_RX_EMPTY_Msk          (0x1ul << SPI_STATUS_RX_EMPTY_Pos)                /*!< SPI_T::STATUS: RX_EMPTY Mask              */

#define SPI_STATUS_RX_FULL_Pos           (1)                                               /*!< SPI_T::STATUS: RX_FULL Position           */
#define SPI_STATUS_RX_FULL_Msk           (0x1ul << SPI_STATUS_RX_FULL_Pos)                 /*!< SPI_T::STATUS: RX_FULL Mask               */

#define SPI_STATUS_TX_EMPTY_Pos          (2)                                               /*!< SPI_T::STATUS: TX_EMPTY Position          */
#define SPI_STATUS_TX_EMPTY_Msk          (0x1ul << SPI_STATUS_TX_EMPTY_Pos)                /*!< SPI_T::STATUS: TX_EMPTY Mask              */

#define SPI_STATUS_TX_FULL_Pos           (3)                                               /*!< SPI_T::STATUS: TX_FULL Position           */
#define SPI_STATUS_TX_FULL_Msk           (0x1ul << SPI_STATUS_TX_FULL_Pos)                 /*!< SPI_T::STATUS: TX_FULL Mask               */

#define SPI_STATUS_LTRIG_FLAG_Pos        (4)                                               /*!< SPI_T::STATUS: LTRIG_FLAG Position        */
#define SPI_STATUS_LTRIG_FLAG_Msk        (0x1ul << SPI_STATUS_LTRIG_FLAG_Pos)              /*!< SPI_T::STATUS: LTRIG_FLAG Mask            */

#define SPI_STATUS_SLV_START_INTSTS_Pos  (6)                                               /*!< SPI_T::STATUS: SLV_START_INTSTS Position  */
#define SPI_STATUS_SLV_START_INTSTS_Msk  (0x1ul << SPI_STATUS_SLV_START_INTSTS_Pos)        /*!< SPI_T::STATUS: SLV_START_INTSTS Mask      */

#define SPI_STATUS_INTSTS_Pos            (7)                                               /*!< SPI_T::STATUS: INTSTS Position            */
#define SPI_STATUS_INTSTS_Msk            (0x1ul << SPI_STATUS_INTSTS_Pos)                  /*!< SPI_T::STATUS: INTSTS Mask                */

#define SPI_STATUS_RXINT_STS_Pos         (8)                                               /*!< SPI_T::STATUS: RXINT_STS Position         */
#define SPI_STATUS_RXINT_STS_Msk         (0x1ul << SPI_STATUS_RXINT_STS_Pos)               /*!< SPI_T::STATUS: RXINT_STS Mask             */

#define SPI_STATUS_RX_OVER_RUN_Pos       (9)                                               /*!< SPI_T::STATUS: RX_OVER_RUN Position       */
#define SPI_STATUS_RX_OVER_RUN_Msk       (0x1ul << SPI_STATUS_RX_OVER_RUN_Pos)             /*!< SPI_T::STATUS: RX_OVER_RUN Mask           */

#define SPI_STATUS_TXINT_STS_Pos         (10)                                              /*!< SPI_T::STATUS: TXINT_STS Position         */
#define SPI_STATUS_TXINT_STS_Msk         (0x1ul << SPI_STATUS_TXINT_STS_Pos)               /*!< SPI_T::STATUS: TXINT_STS Mask             */

#define SPI_STATUS_TIME_OUT_STS_Pos      (12)                                              /*!< SPI_T::STATUS: TIME_OUT_STS Position      */
#define SPI_STATUS_TIME_OUT_STS_Msk      (0x1ul << SPI_STATUS_TIME_OUT_STS_Pos)            /*!< SPI_T::STATUS: TIME_OUT_STS Mask          */

#define SPI_STATUS_RX_FIFO_CNT_Pos       (16)                                              /*!< SPI_T::STATUS: RX_FIFO_CNT Position       */
#define SPI_STATUS_RX_FIFO_CNT_Msk       (0xful << SPI_STATUS_RX_FIFO_CNT_Pos)             /*!< SPI_T::STATUS: RX_FIFO_CNT Mask           */

#define SPI_STATUS_TX_FIFO_CNT_Pos       (20)                                              /*!< SPI_T::STATUS: TX_FIFO_CNT Position       */
#define SPI_STATUS_TX_FIFO_CNT_Msk       (0xful << SPI_STATUS_TX_FIFO_CNT_Pos)             /*!< SPI_T::STATUS: TX_FIFO_CNT Mask           */

#define SPI_CLKDIV_DIVIDER1_Pos          (0)                                               /*!< SPI_T::CLKDIV: DIVIDER1 Position          */
#define SPI_CLKDIV_DIVIDER1_Msk          (0xfful << SPI_CLKDIV_DIVIDER1_Pos)               /*!< SPI_T::CLKDIV: DIVIDER1 Mask              */

#define SPI_CLKDIV_DIVIDER2_Pos          (16)                                              /*!< SPI_T::CLKDIV: DIVIDER2 Position          */
#define SPI_CLKDIV_DIVIDER2_Msk          (0xfful << SPI_CLKDIV_DIVIDER2_Pos)               /*!< SPI_T::CLKDIV: DIVIDER2 Mask              */

#define SPI_SSR_SSR_Pos                  (0)                                               /*!< SPI_T::SSR: SSR Position                  */
#define SPI_SSR_SSR_Msk                  (0x3ul << SPI_SSR_SSR_Pos)                        /*!< SPI_T::SSR: SSR Mask                      */

#define SPI_SSR_SS_LVL_Pos               (2)                                               /*!< SPI_T::SSR: SS_LVL Position               */
#define SPI_SSR_SS_LVL_Msk               (0x1ul << SPI_SSR_SS_LVL_Pos)                     /*!< SPI_T::SSR: SS_LVL Mask                   */

#define SPI_SSR_AUTOSS_Pos               (3)                                               /*!< SPI_T::SSR: AUTOSS Position               */
#define SPI_SSR_AUTOSS_Msk               (0x1ul << SPI_SSR_AUTOSS_Pos)                     /*!< SPI_T::SSR: AUTOSS Mask                   */

#define SPI_SSR_SS_LTRIG_Pos             (4)                                               /*!< SPI_T::SSR: SS_LTRIG Position             */
#define SPI_SSR_SS_LTRIG_Msk             (0x1ul << SPI_SSR_SS_LTRIG_Pos)                   /*!< SPI_T::SSR: SS_LTRIG Mask                 */

#define SPI_SSR_NOSLVSEL_Pos             (5)                                               /*!< SPI_T::SSR: NOSLVSEL Position             */
#define SPI_SSR_NOSLVSEL_Msk             (0x1ul << SPI_SSR_NOSLVSEL_Pos)                   /*!< SPI_T::SSR: NOSLVSEL Mask                 */

#define SPI_SSR_SLV_ABORT_Pos            (8)                                               /*!< SPI_T::SSR: SLV_ABORT Position            */
#define SPI_SSR_SLV_ABORT_Msk            (0x1ul << SPI_SSR_SLV_ABORT_Pos)                  /*!< SPI_T::SSR: SLV_ABORT Mask                */

#define SPI_SSR_SSTA_INTEN_Pos           (9)                                               /*!< SPI_T::SSR: SSTA_INTEN Position           */
#define SPI_SSR_SSTA_INTEN_Msk           (0x1ul << SPI_SSR_SSTA_INTEN_Pos)                 /*!< SPI_T::SSR: SSTA_INTEN Mask               */

#define SPI_SSR_SS_INT_OPT_Pos           (16)                                              /*!< SPI_T::SSR: SS_INT_OPT Position           */
#define SPI_SSR_SS_INT_OPT_Msk           (0x1ul << SPI_SSR_SS_INT_OPT_Pos)                 /*!< SPI_T::SSR: SS_INT_OPT Mask               */

#define SPI_RX0_RDATA_Pos                (0)                                               /*!< SPI_T::RX0: RDATA Position                */
#define SPI_RX0_RDATA_Msk                (0xfffffffful << SPI_RX0_RDATA_Pos)               /*!< SPI_T::RX0: RDATA Mask                    */

#define SPI_RX1_RDATA_Pos                (0)                                               /*!< SPI_T::RX1: RDATA Position                */
#define SPI_RX1_RDATA_Msk                (0xfffffffful << SPI_RX1_RDATA_Pos)               /*!< SPI_T::RX1: RDATA Mask                    */

#define SPI_TX0_TDATA_Pos                (0)                                               /*!< SPI_T::TX0: TDATA Position                */
#define SPI_TX0_TDATA_Msk                (0xfffffffful << SPI_TX0_TDATA_Pos)               /*!< SPI_T::TX0: TDATA Mask                    */

#define SPI_TX1_TDATA_Pos                (0)                                               /*!< SPI_T::TX1: TDATA Position                */
#define SPI_TX1_TDATA_Msk                (0xfffffffful << SPI_TX1_TDATA_Pos)               /*!< SPI_T::TX1: TDATA Mask                    */

#define SPI_VARCLK_VARCLK_Pos            (0)                                               /*!< SPI_T::VARCLK: VARCLK Position            */
#define SPI_VARCLK_VARCLK_Msk            (0xfffffffful << SPI_VARCLK_VARCLK_Pos)           /*!< SPI_T::VARCLK: VARCLK Mask                */

#define SPI_DMA_TX_DMA_EN_Pos            (0)                                               /*!< SPI_T::DMA: TX_DMA_EN Position            */
#define SPI_DMA_TX_DMA_EN_Msk            (0x1ul << SPI_DMA_TX_DMA_EN_Pos)                  /*!< SPI_T::DMA: TX_DMA_EN Mask                */

#define SPI_DMA_RX_DMA_EN_Pos            (1)                                               /*!< SPI_T::DMA: RX_DMA_EN Position            */
#define SPI_DMA_RX_DMA_EN_Msk            (0x1ul << SPI_DMA_RX_DMA_EN_Pos)                  /*!< SPI_T::DMA: RX_DMA_EN Mask                */

#define SPI_DMA_PDMA_RST_Pos             (2)                                               /*!< SPI_T::DMA: PDMA_RST Position             */
#define SPI_DMA_PDMA_RST_Msk             (0x1ul << SPI_DMA_PDMA_RST_Pos)                   /*!< SPI_T::DMA: PDMA_RST Mask                 */

#define SPI_FFCTL_RX_CLR_Pos             (0)                                               /*!< SPI_T::FFCTL: RX_CLR Position             */
#define SPI_FFCTL_RX_CLR_Msk             (0x1ul << SPI_FFCTL_RX_CLR_Pos)                   /*!< SPI_T::FFCTL: RX_CLR Mask                 */

#define SPI_FFCTL_TX_CLR_Pos             (1)                                               /*!< SPI_T::FFCTL: TX_CLR Position             */
#define SPI_FFCTL_TX_CLR_Msk             (0x1ul << SPI_FFCTL_TX_CLR_Pos)                   /*!< SPI_T::FFCTL: TX_CLR Mask                 */

#define SPI_FFCTL_RX_INTEN_Pos           (2)                                               /*!< SPI_T::FFCTL: RX_INTEN Position           */
#define SPI_FFCTL_RX_INTEN_Msk           (0x1ul << SPI_FFCTL_RX_INTEN_Pos)                 /*!< SPI_T::FFCTL: RX_INTEN Mask               */

#define SPI_FFCTL_TX_INTEN_Pos           (3)                                               /*!< SPI_T::FFCTL: TX_INTEN Position           */
#define SPI_FFCTL_TX_INTEN_Msk           (0x1ul << SPI_FFCTL_TX_INTEN_Pos)                 /*!< SPI_T::FFCTL: TX_INTEN Mask               */

#define SPI_FFCTL_RXOVR_INTEN_Pos        (4)                                               /*!< SPI_T::FFCTL: RXOVR_INTEN Position        */
#define SPI_FFCTL_RXOVR_INTEN_Msk        (0x1ul << SPI_FFCTL_RXOVR_INTEN_Pos)              /*!< SPI_T::FFCTL: RXOVR_INTEN Mask            */

#define SPI_FFCTL_TIMEOUT_EN_Pos         (7)                                               /*!< SPI_T::FFCTL: TIMEOUT_EN Position         */
#define SPI_FFCTL_TIMEOUT_EN_Msk         (0x1ul << SPI_FFCTL_TIMEOUT_EN_Pos)               /*!< SPI_T::FFCTL: TIMEOUT_EN Mask             */

#define SPI_FFCTL_RX_THRESHOLD_Pos       (24)                                              /*!< SPI_T::FFCTL: RX_THRESHOLD Position       */
#define SPI_FFCTL_RX_THRESHOLD_Msk       (0x7ul << SPI_FFCTL_RX_THRESHOLD_Pos)             /*!< SPI_T::FFCTL: RX_THRESHOLD Mask           */

#define SPI_FFCTL_TX_THRESHOLD_Pos       (28)                                              /*!< SPI_T::FFCTL: TX_THRESHOLD Position       */
#define SPI_FFCTL_TX_THRESHOLD_Msk       (0x7ul << SPI_FFCTL_TX_THRESHOLD_Pos)             /*!< SPI_T::FFCTL: TX_THRESHOLD Mask           */

/**@}*/ /* SPI_CONST */
/**@}*/ /* end of SPI register group */


/*---------------------- Timer Controller -------------------------*/
/**
    @addtogroup TIMER Timer Controller(TIMER)
    Memory Mapped Structure for TIMER Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTL</font><br><p> <font size="2">
Offset: 0x00  Timer x Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TMR_EN</td><td><div style="word-wrap: break-word;"><b>Timer Counter Enable Bit</b><br>
0 = Stops/Suspends counting.<br>
1 = Starts counting.<br>
Note1: Set TMR_EN to 1 enables 24-bit counter keeps up counting from the last stop counting value.<br>
Note2: This bit is auto-cleared by hardware in one-shot mode (MODE_SEL [5:4] =2'b00) once the value of 24-bit up counter equals the TMRx_CMPR.<br>
</div></td></tr><tr><td>
[1]</td><td>SW_RST</td><td><div style="word-wrap: break-word;"><b>Software Reset</b><br>
Set this bit will reset the timer counter, pre-scale counter and also force TMR_CTL [TMR_EN] to 0.<br>
0 = No effect.<br>
1 = Reset Timer's pre-scale counter, internal 24-bit up-counter and TMR_CTL [TMR_EN] bit.<br>
Note: This bit will be auto cleared and takes at least 3 TMRx_CLK clock cycles.<br>
</div></td></tr><tr><td>
[2]</td><td>WAKE_EN</td><td><div style="word-wrap: break-word;"><b>Wake-Up Enable</b><br>
When WAKE_EN is set and the TMR_IS or TCAP_IS is set, the timer controller will generate a wake-up trigger event to CPU.<br>
0 = Wake-up trigger event Disabled.<br>
1 = Wake-up trigger event Enabled.<br>
</div></td></tr><tr><td>
[3]</td><td>DBGACK_EN</td><td><div style="word-wrap: break-word;"><b>ICE Debug Mode Acknowledge Ineffective Enable</b><br>
0 = ICE debug mode acknowledgement effects TIMER counting and TIMER counter will be held while ICE debug mode acknowledged.<br>
1 = ICE debug mode acknowledgement is ineffective and TIMER counter will keep going no matter ICE debug mode acknowledged or not.<br>
</div></td></tr><tr><td>
[5:4]</td><td>MODE_SEL</td><td><div style="word-wrap: break-word;"><b>Timer Operating Mode Select</b><br>
00 = The timer is operating in the one-shot mode.<br>
In this mode, the associated interrupt signal is generated (if TMR_IER [TMR_IE] is enabled) once the value of 24-bit up counter equals the TMRx_CMPR.<br>
And TMR_CTL [TMR_EN] is automatically cleared by hardware.<br>
01 = The timer is operating in the periodic mode.<br>
In this mode, the associated interrupt signal is generated periodically (if TMR_IER [TMR_IE] is enabled) while the value of 24-bit up counter equals the TMRx_CMPR.<br>
After that, the 24-bit counter will be reset and starts counting from zero again.<br>
10 = The timer is operating in the periodic mode with output toggling.<br>
In this mode, the associated interrupt signal is generated periodically (if TMR_IER [TMR_IE] is enabled) while the value of 24-bit up counter equals the TMRx_CMPR.<br>
After that, the 24-bit counter will be reset and starts counting from zero again.<br>
At the same time, timer controller will also toggle the output pin TMRx_TOG_OUT to its inverse level (from low to high or from high to low).<br>
Note: The default level of TMRx_TOG_OUT after reset is low.<br>
11 = The timer is operating in continuous counting mode.<br>
In this mode, the associated interrupt signal is generated when TMR_DR = TMR_CMPR (if TMR_IER [TMR_IE] is enabled).<br>
However, the 24-bit up-counter counts continuously without reset.<br>
</div></td></tr><tr><td>
[7]</td><td>TMR_ACT</td><td><div style="word-wrap: break-word;"><b>Timer Active Status Bit (Read Only)</b><br>
This bit indicates the timer counter status of timer.<br>
0 = Timer is not active.<br>
1 = Timer is in active.<br>
</div></td></tr><tr><td>
[8]</td><td>ADC_TEEN</td><td><div style="word-wrap: break-word;"><b>TMR_IS Or TCAP_IS Trigger ADC Enable</b><br>
This bit controls if TMR_IS or TCAP_IS could trigger ADC.<br>
When ADC_TEEN is set, TMR_IS is set and the CAP_TRG_EN is low, the timer controller will generate an internal trigger event to ADC controller.<br>
When ADC_TEEN is set, TCAP_IS is set and the CAP_TRG_EN is high, the timer controller will generate an internal trigger event to ADC controller.<br>
0 = TMR_IS or TCAP_IS trigger ADC Disabled.<br>
1 = TMR_IS or TCAP_IS trigger ADC Enabled.<br>
</div></td></tr><tr><td>
[9]</td><td>DAC_TEEN</td><td><div style="word-wrap: break-word;"><b>TMR_IS Or TCAP_IS Trigger DAC Enable</b><br>
This bit controls if TMR_IS or TCAP_IS could trigger DAC.<br>
When DAC_TEEN is set, TMR_IS is set and the CAP_TRG_EN is low, the timer controller will generate an internal trigger event to DAC controller.<br>
When DAC_TEEN is set, TCAP_IS is set and the CAP_TRG_EN is high, the timer controller will generate an internal trigger event to DAC controller.<br>
0 = TMR_IS or TCAP_IS trigger DAC Disabled.<br>
1 = TMR_IS or TCAP_IS trigger DAC Enabled.<br>
</div></td></tr><tr><td>
[10]</td><td>PDMA_TEEN</td><td><div style="word-wrap: break-word;"><b>TMR_IS Or TCAP_IS Trigger PDMA Enable</b><br>
This bit controls if TMR_IS or TCAP_IS could trigger PDMA.<br>
When PDMA_TEEN is set, TMR_IS is set and the CAP_TRG_EN is low, the timer controller will generate an internal trigger event to PDMA controller.<br>
When PDMA_TEEN is set, TCAP_IS is set and the CAP_TRG_EN is high, the timer controller will generate an internal trigger event to PDMA controller.<br>
0 = TMR_IS or TCAP_IS trigger PDMA Disabled.<br>
1 = TMR_IS or TCAP_IS trigger PDMA Enabled.<br>
</div></td></tr><tr><td>
[11]</td><td>CAP_TRG_EN</td><td><div style="word-wrap: break-word;"><b>TCAP_IS Trigger Mode Enable</b><br>
This bit controls if the TMR_IS or TCAP_IS is used to trigger PDMA, DAC and ADC while TMR_IS or TCAP_IS is set.<br>
If this bit is low and TMR_IS is set, timer will generate an internal trigger event to PDMA, DAC or ADC while related trigger enable bit (PDMA_TEEN, DAC_TEEN or ADC_TEEN) is also set.<br>
If this bit is set high and TCAP_IS is set, timer will generate an internal trigger event to PDMA, DAC or ADC while related trigger enable bit (PDMA_TEEN, DAC_TEEN or ADC_TEEN) is also set.<br>
0 = TMR_IS is used to trigger PDMA, DAC and ADC.<br>
1 = TCAP_IS is used to trigger PDMA, DAC and ADC.<br>
</div></td></tr><tr><td>
[12]</td><td>EVENT_EN</td><td><div style="word-wrap: break-word;"><b>Event Counting Mode Enable</b><br>
When EVENT_EN is set, the increase of 24-bit up-counting timer is controlled by external event pin.<br>
While the transition of external event pin matches the definition of EVENT_EDGE, the 24-bit up-counting timer increases by 1.<br>
Or, the 24-bit up-counting timer will keep its value unchanged.<br>
0 = Timer counting is not controlled by external event pin.<br>
1 = Timer counting is controlled by external event pin.<br>
Note: When EVENT_EN is enabled, user can not choose EXT_TMx(GPB) as clock source.<br>
However, the speed of chosen clock must 3 times greater than the speed of EXT_TMx(GPB).<br>
</div></td></tr><tr><td>
[13]</td><td>EVENT_EDGE</td><td><div style="word-wrap: break-word;"><b>Event Counting Mode Edge Selection</b><br>
This bit indicates which edge of external event pin enabling the timer to increase 1.<br>
0 = A falling edge of external event enabling the timer to increase 1.<br>
1 = A rising edge of external event enabling the timer to increase 1.<br>
</div></td></tr><tr><td>
[14]</td><td>EVNT_DEB_EN</td><td><div style="word-wrap: break-word;"><b>External Event De-Bounce Enable</b><br>
When EVNT_DEB_EN is set, the external event pin de-bounce circuit will be enabled to eliminate the bouncing of the signal.<br>
In de-bounce circuit the external event pin will be sampled 4 times by TMRx_CLK.<br>
0 = De-bounce circuit Disabled.<br>
1 = De-bounce circuit Enabled.<br>
Note: When EVENT_EN is enabled, enable this bit is recommended.<br>
And, while EVENT_EN is disabled, disable this bit is recommended to save power consumption.<br>
</div></td></tr><tr><td>
[16]</td><td>TCAP_EN</td><td><div style="word-wrap: break-word;"><b>Tcapture Pin Functional Enable</b><br>
This bit controls if the transition on Tcapture pin could be used as timer counter reset function or timer capture function.<br>
0 = The transition on Tcapture pin is ignored.<br>
1 = The transition on Tcapture pin will result in the capture or reset of 24-bit timer counter.<br>
Note: For TMRx_CTL, if INTR_TRG_EN is set, the TCAP_EN will be forced to low and the Tcapture pin transition is ignored.<br>
Note: For TMRx+1_CTL, if INTR_TRG_EN is set, the TCAP_EN will be forced to high.<br>
</div></td></tr><tr><td>
[17]</td><td>TCAP_MODE</td><td><div style="word-wrap: break-word;"><b>Tcapture Pin Function Mode Selection</b><br>
This bit indicates if the transition on Tcapture pin is used as timer counter reset function or timer capture function.<br>
0 = The transition on Tcapture pin is used as timer capture function.<br>
1 = The transition on Tcapture pin is used as timer counter reset function.<br>
Note: For TMRx+1_CTL, if INTR_TRG_EN is set, the TCAP_MODE will be forced to low.<br>
</div></td></tr><tr><td>
[19:18]</td><td>TCAP_EDGE</td><td><div style="word-wrap: break-word;"><b>Tcapture Pin Edge Detect Selection</b><br>
This field defines that active transition of Tcapture pin is for timer counter reset function or for timer capture function.<br>
For timer counter reset function and free-counting mode of timer capture function, the configurations are:<br>
00 = A falling edge (1 to 0 transition) on Tcapture pin is   an active transition.<br>
01 = A rising edge (0 to 1 transition) on Tcapture pin is   an active transition.<br>
10 = Both falling edge (1 to 0 transition) and rising edge   (0 to 1 transition) on Tcapture pin are active transitions.<br>
11 = Both falling edge (1 to 0 transition) and rising edge   (0 to 1 transition) on Tcapture pin are active transitions.<br>
For trigger-counting mode of timer capture function, the configurations are:<br>
00 = 1st falling edge on Tcapture pin triggers   24-bit timer to start counting, while 2nd falling edge   triggers 24-bit timer to stop counting.<br>
01 = 1st rising edge on Tcapture pin triggers   24-bit timer to start counting, while 2nd rising edge   triggers 24-bit timer to stop counting.<br>
10 = Falling edge on Tcapture pin triggers 24-bit timer to   start counting, while rising edge triggers 24-bit timer to stop counting.<br>
11 = Rising edge on Tcapture pin triggers 24-bit timer to   start counting, while falling edge triggers 24-bit timer to stop counting.<br>
Note: For TMRx+1_CTL, if INTR_TRG_EN is set, the TCAP_EDGE will be forced to 11.<br>
</div></td></tr><tr><td>
[20]</td><td>TCAP_CNT_MODE</td><td><div style="word-wrap: break-word;"><b>Timer Capture Counting Mode Selection</b><br>
This bit indicates the behavior of 24-bit up-counting timer while TCAP_EN is set to high.<br>
If this bit is 0, the free-counting mode, the behavior of 24-bit up-counting timer is defined by MODE_SEL field.<br>
When TCAP_EN is set, TCAP_MODE is 0, and the transition of Tcapture pin matches the TCAP_EDGE setting, the value of 24-bit up-counting timer will be saved into register TMRx_TCAPn.<br>
If this bit is 1, Trigger-counting mode, 24-bit up-counting timer will be not counting and keep its value at zero.<br>
When TCAP_EN is set, TCAP_MODE is 0, and once the transition of external pin matches the 1st transition of TCAP_EDGE setting, the 24-bit up-counting timer will start counting.<br>
And then if the transition of external pin matches the 2nd transition of TCAP_EDGE setting, the 24-bit up-counting timer will stop counting.<br>
And its value will be saved into register TMRx_TCAPn.<br>
0 = Capture with free-counting timer mode.<br>
1 = Capture with trigger-counting timer mode.<br>
Note: For TMRx+1_CTL, if INTR_TRG_EN is set, the CAP_CNT_MOD will be forced to high, the capture with Trigger-counting Timer mode.<br>
</div></td></tr><tr><td>
[22]</td><td>TCAP_DEB_EN</td><td><div style="word-wrap: break-word;"><b>Tcapture Pin De-Bounce Enable</b><br>
When CAP_DEB_EN is set, the Tcapture pin de-bounce circuit will be enabled to eliminate the bouncing of the signal.<br>
In de-bounce circuit the Tcapture pin signal will be sampled 4 times by TMRx_CLK.<br>
0 = De-bounce circuit Disabled.<br>
1 = De-bounce circuit Enabled.<br>
Note: When TCAP_EN is enabled, enable this bit is recommended.<br>
And, while TCAP_EN is disabled, disable this bit is recommended to save power consumption.<br>
</div></td></tr><tr><td>
[24]</td><td>INTR_TRG_EN</td><td><div style="word-wrap: break-word;"><b>Inter-Timer Trigger Mode Enable</b><br>
This bit controls if Inter-timer Trigger mode is enabled.<br>
If Inter-timer Trigger mode is enabled, the TMRx will be in counter mode and counting with external Clock Source or event.<br>
And, TMRx+1 will be in trigger-counting mode of capture function.<br>
0 = Inter-timer trigger mode Disabled.<br>
1 = Inter-timer trigger mode Enabled.<br>
Note: For TMRx+1_CTL, this bit is ignored and the read back value is always 1'b0.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PRECNT</font><br><p> <font size="2">
Offset: 0x04  Timer x Pre-Scale Counter Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>PRESCALE_CNT</td><td><div style="word-wrap: break-word;"><b>Pre-Scale Counter</b><br>
Clock input is divided by PRESCALE_CNT + 1 before it is fed to the counter.<br>
If PRESCALE_CNT =0, then there is no scaling.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PRECNT;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CMPR</font><br><p> <font size="2">
Offset: 0x08  Timer x Compare Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[23:0]</td><td>TMR_CMP</td><td><div style="word-wrap: break-word;"><b>Timer Compared Value</b><br>
TMR_CMP is a 24-bit compared register.<br>
When the internal 24-bit up-counter counts and its value is equal to TMR_CMP value, a Timer Interrupt is requested if the timer interrupt is enabled with TMR_IER [TMR_IE] is enabled.<br>
The TMR_CMP value defines the timer counting cycle time.<br>
Time-out period = (Period of timer clock input) * (8-bit PRESCALE_CNT + 1) * (24-bit TMR_CMP).<br>
Note1: Never write 0x0 or 0x1 in TMR_CMP, or the core will run into unknown state.<br>
Note2: No matter TMR_CTL [TMR_EN] is 0 or 1, whenever software write a new value into this register, TIMER will restart counting using this new value and abort previous count.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CMPR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IER</font><br><p> <font size="2">
Offset: 0x0C  Timer x Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TMR_IE</td><td><div style="word-wrap: break-word;"><b>Timer Interrupt Enable</b><br>
0 = Timer Interrupt Disabled.<br>
1 = Timer Interrupt Enabled.<br>
Note: If timer interrupt is enabled, the timer asserts its interrupt signal when the associated counter is equal to TMR_CMPR.<br>
</div></td></tr><tr><td>
[1]</td><td>TCAP_IE</td><td><div style="word-wrap: break-word;"><b>Timer Capture Function Interrupt Enable</b><br>
0 = Timer External Pin Function Interrupt Disabled.<br>
1 = Timer External Pin Function Interrupt Enabled.<br>
Note: If timer external pin function interrupt is enabled, the timer asserts its interrupt signal when the TCAP_EN is set and the transition of external pin matches the TCAP_EDGE setting<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IER;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISR</font><br><p> <font size="2">
Offset: 0x10  Timer x Interrupt Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>TMR_IS</td><td><div style="word-wrap: break-word;"><b>Timer Interrupt Status</b><br>
This bit indicates the interrupt status of Timer.<br>
This bit is set by hardware when the up counting value of internal 24-bit counter matches the timer compared value (TMR_CMPR).<br>
Write 1 to clear this bit to 0.<br>
If this bit is active and TMR_IE is enabled, Timer will trigger an interrupt to CPU.<br>
</div></td></tr><tr><td>
[1]</td><td>TCAP_IS</td><td><div style="word-wrap: break-word;"><b>Timer Capture Function Interrupt Status</b><br>
This bit indicates the external pin function interrupt status of Timer.<br>
This bit is set by hardware when TCAP_EN is set high, and the transition of external pin matches the TCAP_EDGE setting.<br>
Write 1 to clear this bit to zero.<br>
If this bit is active and TCAP_IE is enabled, Timer will trigger an interrupt to CPU.<br>
</div></td></tr><tr><td>
[4]</td><td>TMR_Wake_STS</td><td><div style="word-wrap: break-word;"><b>Timer Wake-Up Status</b><br>
If timer causes CPU wakes up from power-down mode, this bit will be set to high.<br>
It must be cleared by software with a write 1 to this bit.<br>
0 = Timer does not cause system wake-up.<br>
1 = Wakes system up from power-down mode by Timer timeout.<br>
</div></td></tr><tr><td>
[5]</td><td>NCAP_DET_STS</td><td><div style="word-wrap: break-word;"><b>New Capture Detected Status</b><br>
This status is to indicate there is a new incoming capture event detected before CPU clearing the TCAP_IS status.<br>
If the above condition occurred, the Timer will keep register TMRx_CAP unchanged and drop the new capture value.<br>
This bit is also cleared to 0 while TCAP_IS is cleared.<br>
0 = New incoming capture event didn't detect before CPU clearing TCAP_IS status.<br>
1 = New incoming capture event detected before CPU clearing TCAP_IS status.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">DR</font><br><p> <font size="2">
Offset: 0x14  Timer x Data Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[23:0]</td><td>TDR</td><td><div style="word-wrap: break-word;"><b>Timer Data Register</b><br>
User can read this register for internal 24-bit timer up-counter value.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t DR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TCAP</font><br><p> <font size="2">
Offset: 0x18  Timer x Capture Data Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[23:0]</td><td>CAP</td><td><div style="word-wrap: break-word;"><b>Timer Capture Data Register</b><br>
When TCAP_EN is set, TCAP_MODE is 0, and the transition of external pin matches the TCAP_EDGE setting, the value of 24-bit up-counting timer will be saved into register TMRx_TCAP.<br>
User can read this register for the counter value.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t TCAP;
} TIMER_T;


/**
    @addtogroup TMR_CONST TIMER Bit Field Definition
    Constant Definitions for TIMER Controller
@{ */

#define TIMER_CTL_TMR_EN_Pos             (0)                                               /*!< TIMER_T::CTL: TMR_EN Position             */
#define TIMER_CTL_TMR_EN_Msk             (0x1ul << TIMER_CTL_TMR_EN_Pos)                   /*!< TIMER_T::CTL: TMR_EN Mask                 */

#define TIMER_CTL_SW_RST_Pos             (1)                                               /*!< TIMER_T::CTL: SW_RST Position               */
#define TIMER_CTL_SW_RST_Msk             (0x1ul << TIMER_CTL_SW_RST_Pos)                   /*!< TIMER_T::CTL: SW_RST Mask                   */

#define TIMER_CTL_WAKE_EN_Pos            (2)                                               /*!< TIMER_T::CTL: WAKE_EN Position              */
#define TIMER_CTL_WAKE_EN_Msk            (0x1ul << TIMER_CTL_WAKE_EN_Pos)                  /*!< TIMER_T::CTL: WAKE_EN Mask                  */

#define TIMER_CTL_DBGACK_EN_Pos          (3)                                               /*!< TIMER_T::CTL: DBGACK_EN Position            */
#define TIMER_CTL_DBGACK_EN_Msk          (0x1ul << TIMER_CTL_DBGACK_EN_Pos)                /*!< TIMER_T::CTL: DBGACK_EN Mask                */

#define TIMER_CTL_MODE_SEL_Pos           (4)                                               /*!< TIMER_T::CTL: MODE_SEL Position             */
#define TIMER_CTL_MODE_SEL_Msk           (0x3ul << TIMER_CTL_MODE_SEL_Pos)                 /*!< TIMER_T::CTL: MODE_SEL Mask                 */

#define TIMER_CTL_TMR_ACT_Pos            (7)                                               /*!< TIMER_T::CTL: TMR_ACT Position            */
#define TIMER_CTL_TMR_ACT_Msk            (0x1ul << TIMER_CTL_TMR_ACT_Pos)                  /*!< TIMER_T::CTL: TMR_ACT Mask                */

#define TIMER_CTL_ADC_TEEN_Pos           (8)                                               /*!< TIMER_T::CTL: ADC_TEEN Position             */
#define TIMER_CTL_ADC_TEEN_Msk           (0x1ul << TIMER_CTL_ADC_TEEN_Pos)                 /*!< TIMER_T::CTL: ADC_TEEN Mask                 */

#define TIMER_CTL_DAC_TEEN_Pos           (9)                                               /*!< TIMER_T::CTL: DAC_TEEN Position             */
#define TIMER_CTL_DAC_TEEN_Msk           (0x1ul << TIMER_CTL_DAC_TEEN_Pos)                 /*!< TIMER_T::CTL: DAC_TEEN Mask                 */

#define TIMER_CTL_PDMA_TEEN_Pos          (10)                                              /*!< TIMER_T::CTL: PDMA_TEEN Position            */
#define TIMER_CTL_PDMA_TEEN_Msk          (0x1ul << TIMER_CTL_PDMA_TEEN_Pos)                /*!< TIMER_T::CTL: PDMA_TEEN Mask                */

#define TIMER_CTL_CAP_TRG_EN_Pos         (11)                                              /*!< TIMER_T::CTL: CAP_TRG_EN Position           */
#define TIMER_CTL_CAP_TRG_EN_Msk         (0x1ul << TIMER_CTL_CAP_TRG_EN_Pos)               /*!< TIMER_T::CTL: CAP_TRG_EN Mask               */

#define TIMER_CTL_EVENT_EN_Pos           (12)                                              /*!< TIMER_T::CTL: EVENT_EN Position             */
#define TIMER_CTL_EVENT_EN_Msk           (0x1ul << TIMER_CTL_EVENT_EN_Pos)                 /*!< TIMER_T::CTL: EVENT_EN Mask                 */

#define TIMER_CTL_EVENT_EDGE_Pos         (13)                                              /*!< TIMER_T::CTL: EVENT_EDGE Position           */
#define TIMER_CTL_EVENT_EDGE_Msk         (0x1ul << TIMER_CTL_EVENT_EDGE_Pos)               /*!< TIMER_T::CTL: EVENT_EDGE Mask               */

#define TIMER_CTL_EVNT_DEB_EN_Pos        (14)                                              /*!< TIMER_T::CTL: EVNT_DEB_EN Position          */
#define TIMER_CTL_EVNT_DEB_EN_Msk        (0x1ul << TIMER_CTL_EVNT_DEB_EN_Pos)              /*!< TIMER_T::CTL: EVNT_DEB_EN Mask              */

#define TIMER_CTL_TCAP_EN_Pos            (16)                                              /*!< TIMER_T::CTL: TCAP_EN Position              */
#define TIMER_CTL_TCAP_EN_Msk            (0x1ul << TIMER_CTL_TCAP_EN_Pos)                  /*!< TIMER_T::CTL: TCAP_EN Mask                  */

#define TIMER_CTL_TCAP_MODE_Pos          (17)                                              /*!< TIMER_T::CTL: TCAP_MODE Position            */
#define TIMER_CTL_TCAP_MODE_Msk          (0x1ul << TIMER_CTL_TCAP_MODE_Pos)                /*!< TIMER_T::CTL: TCAP_MODE Mask                */

#define TIMER_CTL_TCAP_EDGE_Pos          (18)                                              /*!< TIMER_T::CTL: TCAP_EDGE Position            */
#define TIMER_CTL_TCAP_EDGE_Msk          (0x3ul << TIMER_CTL_TCAP_EDGE_Pos)                /*!< TIMER_T::CTL: TCAP_EDGE Mask                */

#define TIMER_CTL_TCAP_CNT_MODE_Pos      (20)                                              /*!< TIMER_T::CTL: TCAP_CNT_MODE Position        */
#define TIMER_CTL_TCAP_CNT_MODE_Msk      (0x1ul << TIMER_CTL_TCAP_CNT_MODE_Pos)            /*!< TIMER_T::CTL: TCAP_CNT_MODE Mask            */

#define TIMER_CTL_TCAP_DEB_EN_Pos        (22)                                              /*!< TIMER_T::CTL: TCAP_DEB_EN Position          */
#define TIMER_CTL_TCAP_DEB_EN_Msk        (0x1ul << TIMER_CTL_TCAP_DEB_EN_Pos)              /*!< TIMER_T::CTL: TCAP_DEB_EN Mask              */

#define TIMER_CTL_INTR_TRG_EN_Pos        (24)                                              /*!< TIMER_T::CTL: INTR_TRG_EN Position          */
#define TIMER_CTL_INTR_TRG_EN_Msk        (0x1ul << TIMER_CTL_INTR_TRG_EN_Pos)              /*!< TIMER_T::CTL: INTR_TRG_EN Mask              */

#define TIMER_PRECNT_PRESCALE_CNT_Pos    (0)                                               /*!< TIMER_T::PRECNT: PRESCALE_CNT Position      */
#define TIMER_PRECNT_PRESCALE_CNT_Msk    (0xfful << TIMER_PRECNT_PRESCALE_CNT_Pos)         /*!< TIMER_T::PRECNT: PRESCALE_CNT Mask          */

#define TIMER_CMPR_TMR_CMP_Pos           (0)                                               /*!< TIMER_T::CMPR: TMR_CMP Position           */
#define TIMER_CMPR_TMR_CMP_Msk           (0xfffffful << TIMER_CMPR_TMR_CMP_Pos)            /*!< TIMER_T::CMPR: TMR_CMP Mask               */

#define TIMER_IER_TMR_IE_Pos             (0)                                               /*!< TIMER_T::IER: TMR_IE Position             */
#define TIMER_IER_TMR_IE_Msk             (0x1ul << TIMER_IER_TMR_IE_Pos)                   /*!< TIMER_T::IER: TMR_IE Mask                 */

#define TIMER_IER_TCAP_IE_Pos            (1)                                               /*!< TIMER_T::IER: TCAP_IE Position              */
#define TIMER_IER_TCAP_IE_Msk            (0x1ul << TIMER_IER_TCAP_IE_Pos)                  /*!< TIMER_T::IER: TCAP_IE Mask                  */

#define TIMER_ISR_TMR_IS_Pos             (0)                                               /*!< TIMER_T::ISR: TMR_IS Position             */
#define TIMER_ISR_TMR_IS_Msk             (0x1ul << TIMER_ISR_TMR_IS_Pos)                   /*!< TIMER_T::ISR: TMR_IS Mask                 */

#define TIMER_ISR_TCAP_IS_Pos            (1)                                               /*!< TIMER_T::ISR: TCAP_IS Position              */
#define TIMER_ISR_TCAP_IS_Msk            (0x1ul << TIMER_ISR_TCAP_IS_Pos)                  /*!< TIMER_T::ISR: TCAP_IS Mask                  */

#define TIMER_ISR_TMR_WAKE_STS_Pos       (4)                                               /*!< TIMER_T::ISR: TMR_WAKE_STS Position       */
#define TIMER_ISR_TMR_WAKE_STS_Msk       (0x1ul << TIMER_ISR_TMR_WAKE_STS_Pos)             /*!< TIMER_T::ISR: TMR_WAKE_STS Mask           */

#define TIMER_ISR_NCAP_DET_STS_Pos       (5)                                               /*!< TIMER_T::ISR: NCAP_DET_STS Position         */
#define TIMER_ISR_NCAP_DET_STS_Msk       (0x1ul << TIMER_ISR_NCAP_DET_STS_Pos)             /*!< TIMER_T::ISR: NCAP_DET_STS Mask             */

#define TIMER_DR_TDR_Pos                 (0)                                               /*!< TIMER_T::DR: TDR Position                   */
#define TIMER_DR_TDR_Msk                 (0xfffffful << TIMER_DR_TDR_Pos)                  /*!< TIMER_T::DR: TDR Mask                       */

#define TIMER_TCAP_CAP_Pos               (0)                                               /*!< TIMER_T::TCAP: CAP Position                 */
#define TIMER_TCAP_CAP_Msk               (0xfffffful << TIMER_TCAP_CAP_Pos)                /*!< TIMER_T::TCAP: CAP Mask                     */

/**@}*/ /* TMR_CONST */


/**@}*/ /* end of TMR register group */



/*---------------------- Universal Asynchronous Receiver/Transmitter Controller -------------------------*/
/**
    @addtogroup UART Universal Asynchronous Receiver/Transmitter Controller(UART)
    Memory Mapped Structure for UART Controller
@{ */

typedef struct {


    union {

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RBR</font><br><p> <font size="2">
Offset: 0x00  UART Receive Buffer Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>RBR</td><td><div style="word-wrap: break-word;"><b>Receiving Buffer</b><br>
By reading this register, the UART Controller will return an 8-bit data received from RX pin (LSB first).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

        __I  uint32_t  RBR;
        
        
/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">THR</font><br><p> <font size="2">
Offset: 0x00  UART Transmit Buffer Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7:0]</td><td>THR</td><td><div style="word-wrap: break-word;"><b>Transmit Buffer</b><br>
By writing to this register, the UART sends out an 8-bit data through the TX pin (LSB first).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

        __O  uint32_t  THR;
    };
    
/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTL</font><br><p> <font size="2">
Offset: 0x04  UART Control State Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RX_RST</td><td><div style="word-wrap: break-word;"><b>RX Software Reset</b><br>
When RX_RST is set, all the bytes in the receiving FIFO and RX internal state machine are cleared.<br>
0 = No effect.<br>
1 = Reset the RX internal state machine and pointers.<br>
Note: This bit will be auto cleared and take at least 3 UART engine clock cycles.<br>
</div></td></tr><tr><td>
[1]</td><td>TX_RST</td><td><div style="word-wrap: break-word;"><b>TX Software Reset</b><br>
When TX_RST is set, all the bytes in the transmitting FIFO and TX internal state machine are cleared.<br>
0 = No effect.<br>
1 = Reset the TX internal state machine and pointers.<br>
Note: This bit will be auto cleared and take at least 3 UART engine clock cycles.<br>
</div></td></tr><tr><td>
[2]</td><td>RX_DIS</td><td><div style="word-wrap: break-word;"><b>Receiver Disable Register</b><br>
The receiver is disabled or not (set "1" to disable receiver)<br>
0 = Receiver Enabled.<br>
1 = Receiver Disabled.<br>
Note1: When used for RS-485 NMM mode, user can set this bit to receive data before detecting address byte.<br>
Note2: In RS-485 AAD mode, this bit will be setting to "1" automatically.<br>
Note3: In RS-485 AUD mode and LIN "break + sync +PID" header mode, hardware will control data automatically, so don't fill any value to this bit.<br>
</div></td></tr><tr><td>
[3]</td><td>TX_DIS</td><td><div style="word-wrap: break-word;"><b>Transfer Disable Register</b><br>
The transceiver is disabled or not (set "1" to disable transceiver)<br>
0 = Transfer Enabled.<br>
1 = Transfer Disabled.<br>
</div></td></tr><tr><td>
[4]</td><td>AUTO_RTS_EN</td><td><div style="word-wrap: break-word;"><b>RTSn Auto-Flow Control Enable</b><br>
0 = RTSn auto-flow control. Disabled.<br>
1 = RTSn auto-flow control Enabled.<br>
Note: When RTSn auto-flow is enabled, if the number of bytes in the RX-FIFO equals the UART_FCR [RTS_Tri_Lev], the UART will reassert RTSn signal.<br>
</div></td></tr><tr><td>
[5]</td><td>AUTO_CTS_EN</td><td><div style="word-wrap: break-word;"><b>CTSn Auto-Flow Control Enable</b><br>
0 = CTSn auto-flow control. Disabled<br>
1 = CTSn auto-flow control Enabled.<br>
Note: When CTSn auto-flow is enabled, the UART will send data to external device when CTSn input assert (UART will not send data to device until CTSn is asserted).<br>
</div></td></tr><tr><td>
[6]</td><td>DMA_RX_EN</td><td><div style="word-wrap: break-word;"><b>RX DMA Enable</b><br>
This bit can enable or disable RX PDMA service.<br>
0 = RX PDMA service function Disabled.<br>
1 = RX PDMA service function Enabled.<br>
</div></td></tr><tr><td>
[7]</td><td>DMA_TX_EN</td><td><div style="word-wrap: break-word;"><b>TX DMA Enable</b><br>
This bit can enable or disable TX PDMA service.<br>
0 = TX PDMA service function Disabled.<br>
1 = TX PDMA service function Enabled.<br>
</div></td></tr><tr><td>
[8]</td><td>WAKE_CTS_EN</td><td><div style="word-wrap: break-word;"><b>CTSn Wake-Up Function Enable</b><br>
0 = CTSn wake-up system function Disabled.<br>
1 = Wake-up function Enabled when the system is in power-down mode, an external CTSn change will wake-up system from power-down mode.<br>
</div></td></tr><tr><td>
[9]</td><td>WAKE_DATA_EN</td><td><div style="word-wrap: break-word;"><b>Incoming Data Wake-Up Function Enable</b><br>
0 = Incoming data wake-up system Disabled.<br>
1 = Incoming data wake-up function Enabled when the system is in power-down mode, incoming data will wake-up system from power-down mode.<br>
Note: Hardware will clear this bit when the incoming data wake-up operation finishes and "system clock" work stable<br>
</div></td></tr><tr><td>
[12]</td><td>ABAUD_EN</td><td><div style="word-wrap: break-word;"><b>Auto-Baud Rate Detect Enable</b><br>
0 = Auto-baud rate detect function Disabled.<br>
1 = Auto-baud rate detect function Enabled.<br>
Note: When the auto-baud rate detect operation finishes, hardware will clear this bit and the associated interrupt (INT_ABAUD) will be generated (If UART_IER [ABAUD_IE] be enabled).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TLCTL</font><br><p> <font size="2">
Offset: 0x08  UART Transfer Line Control Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[1:0]</td><td>DATA_LEN</td><td><div style="word-wrap: break-word;"><b>Data Length</b><br>
00 = 5 bits.<br>
01 = 6 bits.<br>
10 = 7 bits.<br>
11 = 8 bits.<br>
</div></td></tr><tr><td>
[2]</td><td>NSB</td><td><div style="word-wrap: break-word;"><b>Number Of STOP Bit Length</b><br>
1 = 1.5 "STOP bit" is generated in the transmitted data when 5-bit word length is selected, and 2 STOP bit" is generated when 6, 7 and 8 bits data length is selected.<br>
0 = 1 " STOP bit" is generated in the transmitted data.<br>
</div></td></tr><tr><td>
[3]</td><td>PBE</td><td><div style="word-wrap: break-word;"><b>Parity Bit Enable</b><br>
1 = Parity bit is generated or checked bet"een the "last data"word "it" and "stop bit" of the serial data.<br>
0 = Parity bit is not generated (transmitting data) or checked (receiving data) during transfer.<br>
</div></td></tr><tr><td>
[4]</td><td>EPE</td><td><div style="word-wrap: break-word;"><b>Even Parity Enable</b><br>
1 = Even number of logic 1's are transmitted or check the data word and parity bits in receiving mode.<br>
0 = Odd number of logic 1's are transmitted or check the data word and parity bits in receiving mode.<br>
Note: This bit has effect only when PBE bit (parity bit enable) is set.<br>
</div></td></tr><tr><td>
[5]</td><td>SPE</td><td><div style="word-wrap: break-word;"><b>Stick Parity Enable</b><br>
1 = When bits PBE, EPE and SPE are set, the parity bit is transmitted and checked as "0".<br>
When PBE and SPE are set and EPE is cleared, the parity bit is transmitted and checked as "1".<br>
In RS-485 mode, PBE, EPE and SPE can control bit 9.<br>
0 = Stick parity Disabled.<br>
</div></td></tr><tr><td>
[6]</td><td>BCB</td><td><div style="word-wrap: break-word;"><b>Break Control Bit</b><br>
When this bit is set to logic "1", the serial data output (TX) is forced to the Spacing State (logic "0").<br>
This bit acts only on TX pin and has no effect on the transmitter logic.<br>
</div></td></tr><tr><td>
[9:8]</td><td>RFITL</td><td><div style="word-wrap: break-word;"><b>RX-FIFO Interrupt (INT_RDA) Trigger Level</b><br>
When the number of bytes in the receiving FIFO is equal to the RFITL then the RDA_IF will be set (if IER [RDA_IEN] is enabled, an interrupt will be generated)<br>
00 = INTR_RDA Trigger Level 1 byte.<br>
01 = INTR_RDA Trigger Level 4 byte.<br>
10 = INTR_RDA Trigger Level 8 byte.<br>
11 = INTR_RDA Trigger Level 14 byte.<br>
Note: When operating in IrDA mode or RS-485 mode, the RFITL must be set to "0".<br>
</div></td></tr><tr><td>
[13:12]</td><td>RTS_TRI_LEV</td><td><div style="word-wrap: break-word;"><b>RTSn Trigger Level (For Auto-Flow Control Use)</b><br>
00 = Trigger level 1 byte.<br>
01 = Trigger level 4 bytes.<br>
10 = Trigger level 8 bytes.<br>
11 = Trigger level 14 bytes.<br>
Note: This field is used for auto RTSn flow control.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TLCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IER</font><br><p> <font size="2">
Offset: 0x0C  UART Interrupt Enable Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RDA_IE</td><td><div style="word-wrap: break-word;"><b>Receive Data Available Interrupt Enable</b><br>
0 = INT_RDA Masked off.<br>
1 = INT_RDA Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>THRE_IE</td><td><div style="word-wrap: break-word;"><b>Transmit Holding Register Empty Interrupt Enable</b><br>
0 = INT_THRE Masked off.<br>
1 = INT_THRE Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>RLS_IE</td><td><div style="word-wrap: break-word;"><b>Receive Line Status Interrupt Enable</b><br>
0 = INT_RLS Masked off.<br>
1 = INT_RLS Enabled.<br>
</div></td></tr><tr><td>
[3]</td><td>MODEM_IE</td><td><div style="word-wrap: break-word;"><b>Modem Status Interrupt Enable</b><br>
0 = INT_MOS Masked off.<br>
1 = INT_MOS Enabled.<br>
</div></td></tr><tr><td>
[4]</td><td>RTO_IE</td><td><div style="word-wrap: break-word;"><b>RX Time-Out Interrupt Enable</b><br>
0 = INT_TOUT Masked off.<br>
1 = INT_TOUT Enabled.<br>
</div></td></tr><tr><td>
[5]</td><td>BUF_ERR_IE</td><td><div style="word-wrap: break-word;"><b>Buffer Error Interrupt Enable</b><br>
0 = INT_BUT_ERR Masked off.<br>
1 = INT_BUF_ERR Enabled.<br>
</div></td></tr><tr><td>
[6]</td><td>WAKE_IE</td><td><div style="word-wrap: break-word;"><b>Wake-Up Interrupt Enable</b><br>
0 = INT_WAKE Masked off.<br>
1 = INT_WAKE Enabled.<br>
</div></td></tr><tr><td>
[7]</td><td>ABAUD_IE</td><td><div style="word-wrap: break-word;"><b>Auto-Baud Rate Interrupt Enable</b><br>
0 = INT_ABAUD Masked off.<br>
1 = INT_ABAUD Enabled.<br>
</div></td></tr><tr><td>
[8]</td><td>LIN_IE</td><td><div style="word-wrap: break-word;"><b>LIN Interrupt Enable</b><br>
0 = INT_LIN Masked off.<br>
1 = INT_LIN Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IER;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISR</font><br><p> <font size="2">
Offset: 0x10  UART Interrupt Status Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RDA_IS</td><td><div style="word-wrap: break-word;"><b>Receive Data Available Interrupt Flag (Read Only)</b><br>
When the number of bytes in the RX-FIFO equals the RFITL then the RDA_IF will be set.<br>
If IER [RDA_IEN] is set then the RDA interrupt will be generated.<br>
Note: This bit is read only and it will be cleared when the number of unread bytes of RX-FIFO drops below the threshold level (RFITL).<br>
</div></td></tr><tr><td>
[1]</td><td>THRE_IS</td><td><div style="word-wrap: break-word;"><b>Transmit Holding Register Empty Interrupt Flag (Read Only)</b><br>
This bit is set when the last data of TX-FIFO is transferred to Transmitter Shift Register.<br>
If IER [THRE_IEN] is set that the THRE interrupt will be generated.<br>
Note: This bit is read only and it will be cleared when writing data into THR (TX-FIFO not empty).<br>
</div></td></tr><tr><td>
[2]</td><td>RLS_IS</td><td><div style="word-wrap: break-word;"><b>Receive Line Interrupt Status Flag (Read Only)</b><br>
This bit is set when the RX received data has parity error (UART_FSR [PE_F]), framing error (UART_FSR [FE_F]), break error (UART_FSR [BI_F]) or RS-485 detect address byte (UART_TRSR [RS-485_ADDET_F]).If IER [RLS_IEN] is set then the RLS interrupt will be generated.<br>
Note1: This bit is read only, but can be cleared by it by writing "1" to UART_FSR [BI_F], UART_FSR [FE_F], UART_FSR [PE_F] or UART_TRSR [RS-485_ADDET_F].<br>
Note2: This bit is cleared when all the BI_F, FE_F, PE_F and RS-485_ADDET_F are cleared.<br>
</div></td></tr><tr><td>
[3]</td><td>MODEM_IS</td><td><div style="word-wrap: break-word;"><b>MODEM Interrupt Status Flag (Read Only)</b><br>
This bit is set when the CTSn pin has state change (DCTSF = "1").<br>
If IER [MODEM_IEN] is set then the modem interrupt will be generated.<br>
Note: This bit is read only, but can be cleared by it by writing "1" to UART_MCSR [DCT_F].<br>
</div></td></tr><tr><td>
[4]</td><td>RTO_IS</td><td><div style="word-wrap: break-word;"><b>RX Time-Out Interrupt Status Flag (Read Only)</b><br>
This bit is set when the RX-FIFO is not empty and no activities occur in the RX-FIFO and the time-out counter equal to TOIC.<br>
If IER [Tout_IEN] is set then the tout interrupt will be generated.<br>
Note: This bit is read only and user can read UART_RBR (RX is in active) to clear it.<br>
</div></td></tr><tr><td>
[5]</td><td>BUF_ERR_IS</td><td><div style="word-wrap: break-word;"><b>Buffer Error Interrupt Status Flag (Read Only)</b><br>
This bit is set when the TX or RX-FIFO overflowed.<br>
When BUF_ERR_IS is set, the transfer maybe not correct.<br>
If IER [BUF_ER_IEN] is set then the buffer error interrupt will be generated.<br>
Note1: This bit is read only, but can be cleared by it by writing "1" to UART_FSR [TX_OVER_F] or UART_FSR [RX_OVER_F].<br>
Note2: This bit is cleared when both the TX_OVER_F and RX_OVER_F are cleared.<br>
</div></td></tr><tr><td>
[6]</td><td>WAKE_IS</td><td><div style="word-wrap: break-word;"><b>Wake-Up Interrupt Status Flag (Read Only)</b><br>
This bit is set in Power-down mode, the receiver received data or CTSn signal.<br>
If IER [WAKE_IE] is set then the wake-up interrupt will be generated.<br>
Note: This bit is read only, but can be cleared by it by writing "1" to it.<br>
</div></td></tr><tr><td>
[7]</td><td>ABAUD_IS</td><td><div style="word-wrap: break-word;"><b>Auto-Baud Rate Interrupt Status Flag (Read Only)</b><br>
This bit is set when auto-baud rate detection function finished or the auto-baud rate counter was overflow and if IER [ABAUD_IE] is set then the auto-baud rate interrupt will be generated.<br>
Note1: This bit is read only, but can be cleared by it by writing "1" to UART_TRSR [ABAUD_TOUT_F] or UART_TRSR [ABAUD_F].<br>
Note2: This bit is cleared when both the ABAUD_TOUT_F and ABAUD_F are cleared.<br>
</div></td></tr><tr><td>
[8]</td><td>LIN_IS</td><td><div style="word-wrap: break-word;"><b>LIN Interrupt Status Flag (Read Only)</b><br>
This bit is set when the LIN TX header transmitted, RX header received or the SIN does not equal SOUT and if IER [LIN_IE] is set then the LIN interrupt will be generated.<br>
Note1: This bit is read only, but can be cleared by it by writing "1" to UART_TRSR [BIT_ERR_F], UART_TRSR [BIT_TX_F] or UART_TRSR [LIN_RX_F].<br>
Note2: This bit is cleared when both the BIT_ERR_F, BIT_TX_F and LIN_RX_F are cleared.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO  uint32_t ISR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TRSR</font><br><p> <font size="2">
Offset: 0x14  UART Transfer State Status Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RS485_ADDET_F</td><td><div style="word-wrap: break-word;"><b>RS-485 Address Byte Detection Status Flag (Read Only)</b><br>
This bit is set to logic "1" and set UART_ALT_CTL [RS-485_ADD_EN] whenever in RS-485 mode the receiver detected any address byte character (bit 9 ='1') bit".<br>
This bit is reset whenever the CPU writes "1" to this bit.<br>
Note1: This field is used for RS-485 mode.<br>
Note2: This bit is read only, but can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[1]</td><td>ABAUD_F</td><td><div style="word-wrap: break-word;"><b>Auto-Baud Rate Interrupt (Read Only)</b><br>
This bit is set to logic "1" when auto-baud rate detect function finished.<br>
Note: This bit is read only, but can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[2]</td><td>ABAUD_TOUT_F</td><td><div style="word-wrap: break-word;"><b>Auto-Baud Rate Time-Out Interrupt (Read Only)</b><br>
This bit is set to logic "1" in Auto-baud Rate Detect mode and the baud rate counter is overflow.<br>
Note: This bit is read only, but can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[3]</td><td>LIN_TX_F</td><td><div style="word-wrap: break-word;"><b>LIN TX Interrupt Flag (Read Only)</b><br>
This bit is set to logic "1" when LIN transmitted header field.<br>
The header may be "break field" or "break field + sync field" or "break field + sync field + PID field", it can be choose by setting UART_ALT_CTL[LIN_HEAD_SEL] register.<br>
Note: This bit is read only, but can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[4]</td><td>LIN_RX_F</td><td><div style="word-wrap: break-word;"><b>LIN RX Interrupt Flag (Read Only)</b><br>
This bit is set to logic "1" when received LIN header field.<br>
The header may be "break field" or "break field + sync field" or "break field + sync field + PID field", and it can be choose by setting UART_ALT_CTL [LIN_HEAD_SEL] register.<br>
If the field includes "break field", when the receiver received break field then the LIN_RX_F will be set.<br>
The controller will receive next data and put it in FIFO.<br>
If the field includes "break field + sync field", hardware will wait for the flag LIN_RX_F in UART_TRSR to check RX received break field and sync field.<br>
If the break and sync field is received, hardware will set UART_TRSR [LIN_RX_F] flag, and if the break is received but the sync field does not equal 0x55, then hardware will set UART_TRSR [LIN_RX_F] and UART_TRSR [LIN_RX_SYNC_ERR_F] flag.<br>
The break and sync data (equals 0x55 or not) will not be stored in FIFO.<br>
If the field includes "break field + sync field + PID field", In this operation mode, hardware will control data automatically.<br>
Hardware will ignore any data until received break + sync (0x55) + PID value match the UART_ALT_CTL [ADDR_MATCH] value (break + sync + PID will not be stored in FIFO).<br>
When received break + sync (0x55) + PID value match the UART_ALT_CTL [ADDR_MATCH] value, hardware will set UART_TRSR [LIN_RX_F] and the following all data will be accepted and stored in the RX-FIFO until detect next break field.<br>
If the receiver received break + wrong sync (not equal 0x55) + PID value, hardware will set UART_TRSR [LIN_RX_F] and UART_TRSR [LIN_RX_SYNC_ERR_F] flag and the receiver will be disabled.<br>
If the receiver received break + sync (0x55) + wrong PID value, hardware will set UART_TRSR [LIN_RX_F] flag and the receiver will be disabled.<br>
Note: This bit is read only, but can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[5]</td><td>BIT_ERR_F</td><td><div style="word-wrap: break-word;"><b>Bit Error Detect Status Flag (Read Only)</b><br>
At TX transfer state, hardware will monitoring the bus state, if the input pin (SIN) state is not equal to the output pin (SOUT) state, BIT_ERR_F will be set.<br>
When occur bit error, hardware will generate an interrupt to CPU (INT_LIN).<br>
Note1: This bit is read only, but it can be cleared by writing "1" to it.<br>
Note2: This bit is only valid when enabling the bit error detection function (UART_ALT_CTL [BIT_ERR_EN] = "1").<br>
</div></td></tr><tr><td>
[8]</td><td>LIN_RX_SYNC_ERR_F</td><td><div style="word-wrap: break-word;"><b>LIN RX SYNC Error Flag (Read Only)</b><br>
This bit is set to logic "1" when LIN received incorrect SYNC field.<br>
User can choose the header by setting UART_ALT_CTL [LIN_HEAD_SEL] register.<br>
If the field includes "break field + sync field" and if the sync data does not equal 0x55, the LIN_RX_F and LIN_RX_SYNC_ERR_F will be set and the wrong sync data will be ignored.<br>
The controller will receive next data and put it in FIFO.<br>
If the field includes "break field + sync field + PID field" and if the sync data does not equal 0x55, the LIN_RX_F and LIN_RX_SYNC_ERR_F will be set and the wrong sync data will be ignored.<br>
The controller will receive next data and put it in FIFO.<br>
Note: This bit is read only, but can be cleared by writing "1" to LIN_RX_F.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO  uint32_t TRSR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">FSR</font><br><p> <font size="2">
Offset: 0x18  UART FIFO State Status Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>RX_OVER_F</td><td><div style="word-wrap: break-word;"><b>RX Overflow Error Status Flag (Read Only)</b><br>
This bit is set when RX-FIFO overflow.<br>
If the number of bytes of received data is greater than RX-FIFO (UART_RBR) size, 16 bytes of UART0/UART1, this bit will be set.<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[1]</td><td>RX_EMPTY_F</td><td><div style="word-wrap: break-word;"><b>Receiver FIFO Empty (Read Only)</b><br>
This bit initiate RX-FIFO empty or not.<br>
When the last byte of RX-FIFO has been read by CPU, hardware sets this bit high.<br>
It will be cleared when UART receives any new data.<br>
</div></td></tr><tr><td>
[2]</td><td>RX_FULL_F</td><td><div style="word-wrap: break-word;"><b>Receiver FIFO Full (Read Only)</b><br>
This bit initiates RX-FIFO full or not.<br>
This bit is set when RX_POINTER_F is equal to 16, otherwise is cleared by hardware.<br>
</div></td></tr><tr><td>
[4]</td><td>PE_F</td><td><div style="word-wrap: break-word;"><b>Parity Error State Status Flag (Read Only)</b><br>
This bit is set to logic "1" whenever the received character does not have a valid "parity bit", and it is reset whenever the CPU writes "1" to this bit.<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[5]</td><td>FE_F</td><td><div style="word-wrap: break-word;"><b>Framing Error Status Flag (Read Only)</b><br>
This bit is set to logic "1" whenever the received character does not have a valid "stop bit" (that is, the stop bit following the last data bit or parity bit is detected as a logic "0"), and it is reset whenever the CPU writes "1" to this bit.<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[6]</td><td>BI_F</td><td><div style="word-wrap: break-word;"><b>Break Status Flag (Read Only)</b><br>
This bit is set to a logic "1" whenever the received data input(RX) is held in the "spacing state" (logic "0") for longer than a full word transmission time (that is, the total time of "start bit" + data bits + parity + stop bits) and it is reset whenever the CPU writes "1" to this bit.<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[8]</td><td>TX_OVER_F</td><td><div style="word-wrap: break-word;"><b>TX Overflow Error Interrupt Status Flag (Read Only)</b><br>
If TX-FIFO (UART_THR) is full, an additional write to UART_THR will cause this bit to logic "1".<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[9]</td><td>TX_EMPTY_F</td><td><div style="word-wrap: break-word;"><b>Transmitter FIFO Empty (Read Only)</b><br>
This bit indicates TX-FIFO empty or not.<br>
When the last byte of TX-FIFO has been transferred to Transmitter Shift Register, hardware sets this bit high.<br>
It will be cleared when writing data into THR (TX-FIFO not empty).<br>
</div></td></tr><tr><td>
[10]</td><td>TX_FULL_F</td><td><div style="word-wrap: break-word;"><b>Transmitter FIFO Full (Read Only)</b><br>
This bit indicates TX-FIFO full or not.<br>
This bit is set when TX_POINTER_F is equal to 16, otherwise is cleared by hardware.<br>
</div></td></tr><tr><td>
[11]</td><td>TE_F</td><td><div style="word-wrap: break-word;"><b>Transmitter Empty Status Flag (Read Only)</b><br>
Bit is set by hardware when TX is inactive. (TX shift register does not have data)<br>
Bit is cleared automatically when TX-FIFO is transfer data to TX shift register or TX is empty but the transfer does not finish.<br>
</div></td></tr><tr><td>
[20:16]</td><td>RX_POINTER_F</td><td><div style="word-wrap: break-word;"><b>RX-FIFO Pointer (Read Only)</b><br>
This field indicates the RX-FIFO Buffer Pointer.<br>
When UART receives one byte from external device, RX_POINTER_F increases one.<br>
When one byte of RX-FIFO is read by CPU, RX_POINTER_F decreases one.<br>
</div></td></tr><tr><td>
[28:24]</td><td>TX_POINTER_F</td><td><div style="word-wrap: break-word;"><b>TX-FIFO Pointer (Read Only)</b><br>
This field indicates the TX-FIFO Buffer Pointer.<br>
When CPU writes one byte data into UART_THR, TX_POINTER_F increases one.<br>
When one byte of TX-FIFO is transferred to Transmitter Shift Register, TX_POINTER_F decreases one.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO  uint32_t FSR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MCSR</font><br><p> <font size="2">
Offset: 0x1C  UART Modem State Status Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>LEV_RTS</td><td><div style="word-wrap: break-word;"><b>RTSn Trigger Level</b><br>
This bit can change the RTSn trigger level.<br>
0 = low level triggered.<br>
1 = high level triggered.<br>
Note: In RS-485 AUD mode and RTS Auto-flow control mode, hardware will control the output RTS pin automatically, so the table indicates the default value.<br>
Note: The default setting in UART mode is LEV_RTS = "0" and RTS_ST = "1".<br>
</div></td></tr><tr><td>
[1]</td><td>RTS_ST</td><td><div style="word-wrap: break-word;"><b>RTSn Pin State (Read Only)</b><br>
This bit is the pin status of RTSn.<br>
</div></td></tr><tr><td>
[16]</td><td>LEV_CTS</td><td><div style="word-wrap: break-word;"><b>CTSn Trigger Level</b><br>
This bit can change the CTSn trigger level.<br>
0 = Low level triggered.<br>
1 = High level triggered.<br>
</div></td></tr><tr><td>
[17]</td><td>CTS_ST</td><td><div style="word-wrap: break-word;"><b>CTSn Pin Status (Read Only)</b><br>
This bit is the pin status of CTSn.<br>
</div></td></tr><tr><td>
[18]</td><td>DCT_F</td><td><div style="word-wrap: break-word;"><b>Detect CTSn State Change Status Flag (Read Only)</b><br>
This bit is set whenever CTSn input has change state, and it will generate Modem interrupt to CPU when UART_IER [Modem_IEN].<br>
Note: This bit is read only, but it can be cleared by writing "1" to it.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MCSR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">TMCTL</font><br><p> <font size="2">
Offset: 0x20  UART Time-Out Control State Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[8:0]</td><td>TOIC</td><td><div style="word-wrap: break-word;"><b>Time-Out Comparator</b><br>
The time-out counter resets and starts counting (the counting clock = baud rate) whenever the RX-FIFO receives a new data word.<br>
Once the content of time-out counter (TOUT_CNT) is equal to time-out interrupt comparator (TOIC), a receiver time-out interrupt (INT_TOUT) is generated if UART_IER [RTO_IEN].<br>
A new incoming data word or RX-FIFO empty clears INT_TOUT.<br>
Note1: Fill all "0" to this field indicates to disable this function.<br>
Note2: The real time-out value is TOIC + 1.<br>
Note3: The counting clock is baud rate clock.<br>
Note4: The UART data format is start bit + 8 data bits + parity bit + stop bit, although software can configure this field by any value but it is recommend to filled this field great than 0xA.<br>
</div></td></tr><tr><td>
[23:16]</td><td>DLY</td><td><div style="word-wrap: break-word;"><b>TX Delay Time Value</b><br>
This field is use to program the transfer delay time between the last stop bit leaving the TX-FIFO and the de-assertion of by setting UART_TMCTL [DLY] register.<br>
Note1: Fill all "0" to this field indicates to disable this function.<br>
Note2: The real delay value is DLY.<br>
Note3: The counting clock is baud rate clock.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t TMCTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">BAUD</font><br><p> <font size="2">
Offset: 0x24  UART Baud Rate Divisor Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[15:0]</td><td>BRD</td><td><div style="word-wrap: break-word;"><b>Baud Rate Divider</b><br>
</div></td></tr><tr><td>
[31]</td><td>DIV_16_EN</td><td><div style="word-wrap: break-word;"><b>Divider 16 Enable</b><br>
The BRD = Baud Rate Divider, and the baud rate equation is  Baud Rate = UART_CLK/ [16 * (BRD + 1)]; The default value of M is 16.<br>
0 = The equation of baud rate is UART_CLK / [ (BRD+1)].<br>
1 = The equation of baud rate is UART_CLK / [16 * (BRD+1)].<br>
Note: In IrDA mode, this bit must disable.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t BAUD;
    uint32_t RESERVE0[2];


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IRCR</font><br><p> <font size="2">
Offset: 0x30  UART IrDA Control Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[1]</td><td>TX_SELECT</td><td><div style="word-wrap: break-word;"><b>TX_SELECT</b><br>
0 = IrDA receiver Enabled.<br>
1 = IrDA transmitter Enabled.<br>
Note: In IrDA mode, the UART_BAUD [DIV_16_EN) register must be set (the baud equation must be Clock / 16 * (BRD)<br>
</div></td></tr><tr><td>
[5]</td><td>INV_TX</td><td><div style="word-wrap: break-word;"><b>INV_TX</b><br>
0 = No inversion.<br>
1 = Inverse TX output signal.<br>
</div></td></tr><tr><td>
[6]</td><td>INV_RX</td><td><div style="word-wrap: break-word;"><b>INV_RX</b><br>
0 = No inversion.<br>
1 = Inverse RX input signal.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IRCR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ALT_CTL</font><br><p> <font size="2">
Offset: 0x34  UART Alternate Control State Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>LIN_TX_BCNT</td><td><div style="word-wrap: break-word;"><b>LIN TX Break Field Count Register</b><br>
The field contains 3-bit LIN TX break field count.<br>
Note: The break field length is LIN_TX_BCNT + 8.<br>
</div></td></tr><tr><td>
[5:4]</td><td>LIN_HEAD_SEL</td><td><div style="word-wrap: break-word;"><b>LIN Header Selection</b><br>
00 = The LIN header includes "break field".<br>
01 = The LIN header includes "break field + sync field".<br>
10 = The LIN header includes "break field + sync field + PID field".<br>
11 = Reserved.<br>
</div></td></tr><tr><td>
[6]</td><td>LIN_RX_EN</td><td><div style="word-wrap: break-word;"><b>LIN RX Enable</b><br>
When LIN RX mode enabled and received a break field or sync field or PID field (Select by LIN_Header_SEL), the controller will generator a interrupt to CPU (INT_LIN)<br>
0 = LIN RX mode Disabled.<br>
1 = LIN RX mode Enabled.<br>
</div></td></tr><tr><td>
[7]</td><td>LIN_TX_EN</td><td><div style="word-wrap: break-word;"><b>LIN TX Header Trigger Enable</b><br>
0 = LIN TX Header Trigger Disabled.<br>
1 = LIN TX Header Trigger Enabled.<br>
Note1: When TX header field (break field or break and sync field or break, sync and PID field) transfer operation finished, this bit will be cleared automatically and generate a interrupt to CPU (INT_LIN).<br>
Note2: If user wants to receive transmit data, it recommended to enable LIN_RX_EN bit.<br>
</div></td></tr><tr><td>
[8]</td><td>Bit_ERR_EN</td><td><div style="word-wrap: break-word;"><b>Bit Error Detect Enable</b><br>
0 = Bit error detection function Disabled.<br>
1 = Bit error detection Enabled.<br>
Note: In LIN function mode, when bit error occurs, hardware will generate an interrupt to CPU (INT_LIN).<br>
</div></td></tr><tr><td>
[16]</td><td>RS485_NMM</td><td><div style="word-wrap: break-word;"><b>RS-485 Normal Multi-Drop Operation Mode (RS-485 NMM Mode)</b><br>
0 = RS-485 Normal Multi-drop Operation mode (NMM) Disabled.<br>
1 = RS-485 Normal Multi-drop Operation mode (NMM) Enabled.<br>
Note: It can't be active in RS-485_AAD Operation mode.<br>
</div></td></tr><tr><td>
[17]</td><td>RS485_AAD</td><td><div style="word-wrap: break-word;"><b>RS-485 Auto Address Detection Operation Mode (RS-485 AAD Mode)</b><br>
0 = RS-485 Auto Address Detection Operation mode (AAD) Disabled.<br>
1 = RS-485 Auto Address Detection Operation mode (AAD) Enabled.<br>
Note: It can't be active in RS-485_NMM Operation mode.<br>
</div></td></tr><tr><td>
[18]</td><td>RS485_AUD</td><td><div style="word-wrap: break-word;"><b>RS-485 Auto Direction Mode (RS-485 AUD Mode)</b><br>
0 = RS-485 Auto Direction mode (AUD) Disabled.<br>
1 = RS-485 Auto Direction mode (AUD) Enabled.<br>
Note: It can be active in RS-485_AAD or RS-485_NMM operation mode.<br>
</div></td></tr><tr><td>
[19]</td><td>RS485_ADD_EN</td><td><div style="word-wrap: break-word;"><b>RS-485 Address Detection Enable</b><br>
This bit is used to enable RS-485 hardware address detection mode.<br>
If hardware detects address byte, and then the controller will set UART_TRSR [RS485_ADDET_F] = "1".<br>
0 = Address detection mode Disabled.<br>
1 = Address detection mode Enabled.<br>
Note: This field is used for RS-485 any operation mode.<br>
</div></td></tr><tr><td>
[31:24]</td><td>ADDR_PID_MATCH</td><td><div style="word-wrap: break-word;"><b>Address / PID Match Value Register</b><br>
This field contains the RS-485 address match values in RS-485 Function mode.<br>
This field contains the LIN protected identifier field n LIN Function mode, software fills ID0~ID5 (ADDR_PID_MATCH [5:0]), hardware will calculate P0 and P1.<br>
Note: This field is used for RS-485 auto address detection mode or used for LIN protected identifier field (PID).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ALT_CTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">FUN_SEL</font><br><p> <font size="2">
Offset: 0x38  UART Function Select Register.</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[1:0]</td><td>FUN_SEL</td><td><div style="word-wrap: break-word;"><b>Function Select Enable</b><br>
00 = UART function mode.<br>
01 = LIN function mode.<br>
10 = IrDA Function.<br>
11 = RS-485 Function.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t FUN_SEL;

} UART_T;

/**
    @addtogroup UART_CONST UART Bit Field Definition
    Constant Definitions for UART Controller
@{ */

#define UART_DAT_DAT_Pos                 (0)                                               /*!< UART_T::DAT: DAT Position                 */
#define UART_DAT_DAT_Msk                 (0xfful << UART_DAT_DAT_Pos)                      /*!< UART_T::DAT: DAT Mask                     */

#define UART_CTL_RX_RST_Pos              (0)                                               /*!< UART_T::CTL: RX_RST Position              */
#define UART_CTL_RX_RST_Msk              (0x1ul << UART_CTL_RX_RST_Pos)                    /*!< UART_T::CTL: RX_RST Mask                  */

#define UART_CTL_TX_RST_Pos              (1)                                               /*!< UART_T::CTL: TX_RST Position              */
#define UART_CTL_TX_RST_Msk              (0x1ul << UART_CTL_TX_RST_Pos)                    /*!< UART_T::CTL: TX_RST Mask                  */

#define UART_CTL_RX_DIS_Pos              (2)                                               /*!< UART_T::CTL: RX_DIS Position              */
#define UART_CTL_RX_DIS_Msk              (0x1ul << UART_CTL_RX_DIS_Pos)                    /*!< UART_T::CTL: RX_DIS Mask                  */

#define UART_CTL_TX_DIS_Pos              (3)                                               /*!< UART_T::CTL: TX_DIS Position              */
#define UART_CTL_TX_DIS_Msk              (0x1ul << UART_CTL_TX_DIS_Pos)                    /*!< UART_T::CTL: TX_DIS Mask                  */

#define UART_CTL_AUTO_RTS_EN_Pos         (4)                                               /*!< UART_T::CTL: AUTO_RTS_EN Position         */
#define UART_CTL_AUTO_RTS_EN_Msk         (0x1ul << UART_CTL_AUTO_RTS_EN_Pos)               /*!< UART_T::CTL: AUTO_RTS_EN Mask             */

#define UART_CTL_AUTO_CTS_EN_Pos         (5)                                               /*!< UART_T::CTL: AUTO_CTS_EN Position         */
#define UART_CTL_AUTO_CTS_EN_Msk         (0x1ul << UART_CTL_AUTO_CTS_EN_Pos)               /*!< UART_T::CTL: AUTO_CTS_EN Mask             */

#define UART_CTL_DMA_RX_EN_Pos           (6)                                               /*!< UART_T::CTL: DMA_RX_EN Position           */
#define UART_CTL_DMA_RX_EN_Msk           (0x1ul << UART_CTL_DMA_RX_EN_Pos)                 /*!< UART_T::CTL: DMA_RX_EN Mask               */

#define UART_CTL_DMA_TX_EN_Pos           (7)                                               /*!< UART_T::CTL: DMA_TX_EN Position           */
#define UART_CTL_DMA_TX_EN_Msk           (0x1ul << UART_CTL_DMA_TX_EN_Pos)                 /*!< UART_T::CTL: DMA_TX_EN Mask               */

#define UART_CTL_WAKE_CTS_EN_Pos         (8)                                               /*!< UART_T::CTL: WAKE_CTS_EN Position         */
#define UART_CTL_WAKE_CTS_EN_Msk         (0x1ul << UART_CTL_WAKE_CTS_EN_Pos)               /*!< UART_T::CTL: WAKE_CTS_EN Mask             */

#define UART_CTL_WAKE_DATA_EN_Pos        (9)                                               /*!< UART_T::CTL: WAKE_DATA_EN Position        */
#define UART_CTL_WAKE_DATA_EN_Msk        (0x1ul << UART_CTL_WAKE_DATA_EN_Pos)              /*!< UART_T::CTL: WAKE_DATA_EN Mask            */

#define UART_CTL_ABAUD_EN_Pos            (12)                                              /*!< UART_T::CTL: ABAUD_EN Position            */
#define UART_CTL_ABAUD_EN_Msk            (0x1ul << UART_CTL_ABAUD_EN_Pos)                  /*!< UART_T::CTL: ABAUD_EN Mask                */

#define UART_TLCTL_DATA_LEN_Pos          (0)                                               /*!< UART_T::TLCTL: DATA_LEN Position          */
#define UART_TLCTL_DATA_LEN_Msk          (0x3ul << UART_TLCTL_DATA_LEN_Pos)                /*!< UART_T::TLCTL: DATA_LEN Mask              */

#define UART_TLCTL_NSB_Pos               (2)                                               /*!< UART_T::TLCTL: NSB Position               */
#define UART_TLCTL_NSB_Msk               (0x1ul << UART_TLCTL_NSB_Pos)                     /*!< UART_T::TLCTL: NSB Mask                   */

#define UART_TLCTL_PBE_Pos               (3)                                               /*!< UART_T::TLCTL: PBE Position               */
#define UART_TLCTL_PBE_Msk               (0x1ul << UART_TLCTL_PBE_Pos)                     /*!< UART_T::TLCTL: PBE Mask                   */

#define UART_TLCTL_EPE_Pos               (4)                                               /*!< UART_T::TLCTL: EPE Position               */
#define UART_TLCTL_EPE_Msk               (0x1ul << UART_TLCTL_EPE_Pos)                     /*!< UART_T::TLCTL: EPE Mask                   */

#define UART_TLCTL_SPE_Pos               (5)                                               /*!< UART_T::TLCTL: SPE Position               */
#define UART_TLCTL_SPE_Msk               (0x1ul << UART_TLCTL_SPE_Pos)                     /*!< UART_T::TLCTL: SPE Mask                   */

#define UART_TLCTL_BCB_Pos               (6)                                               /*!< UART_T::TLCTL: BCB Position               */
#define UART_TLCTL_BCB_Msk               (0x1ul << UART_TLCTL_BCB_Pos)                     /*!< UART_T::TLCTL: BCB Mask                   */

#define UART_TLCTL_RFITL_Pos             (8)                                               /*!< UART_T::TLCTL: RFITL Position             */
#define UART_TLCTL_RFITL_Msk             (0x3ul << UART_TLCTL_RFITL_Pos)                   /*!< UART_T::TLCTL: RFITL Mask                 */

#define UART_TLCTL_RTS_TRI_LEV_Pos       (12)                                              /*!< UART_T::TLCTL: RTS_TRI_LEV Position       */
#define UART_TLCTL_RTS_TRI_LEV_Msk       (0x3ul << UART_TLCTL_RTS_TRI_LEV_Pos)             /*!< UART_T::TLCTL: RTS_TRI_LEV Mask           */

#define UART_IER_RDA_IE_Pos              (0)                                               /*!< UART_T::IER: RDA_IE Position              */
#define UART_IER_RDA_IE_Msk              (0x1ul << UART_IER_RDA_IE_Pos)                    /*!< UART_T::IER: RDA_IE Mask                  */

#define UART_IER_THRE_IE_Pos             (1)                                               /*!< UART_T::IER: THRE_IE Position             */
#define UART_IER_THRE_IE_Msk             (0x1ul << UART_IER_THRE_IE_Pos)                   /*!< UART_T::IER: THRE_IE Mask                 */

#define UART_IER_RLS_IE_Pos              (2)                                               /*!< UART_T::IER: RLS_IE Position              */
#define UART_IER_RLS_IE_Msk              (0x1ul << UART_IER_RLS_IE_Pos)                    /*!< UART_T::IER: RLS_IE Mask                  */

#define UART_IER_MODEM_IE_Pos            (3)                                               /*!< UART_T::IER: MODEM_IE Position            */
#define UART_IER_MODEM_IE_Msk            (0x1ul << UART_IER_MODEM_IE_Pos)                  /*!< UART_T::IER: MODEM_IE Mask                */

#define UART_IER_RTO_IE_Pos              (4)                                               /*!< UART_T::IER: RTO_IE Position              */
#define UART_IER_RTO_IE_Msk              (0x1ul << UART_IER_RTO_IE_Pos)                    /*!< UART_T::IER: RTO_IE Mask                  */

#define UART_IER_BUF_ERR_IE_Pos          (5)                                               /*!< UART_T::IER: BUF_ERR_IE Position          */
#define UART_IER_BUF_ERR_IE_Msk          (0x1ul << UART_IER_BUF_ERR_IE_Pos)                /*!< UART_T::IER: BUF_ERR_IE Mask              */

#define UART_IER_WAKE_IE_Pos             (6)                                               /*!< UART_T::IER: WAKE_IE Position             */
#define UART_IER_WAKE_IE_Msk             (0x1ul << UART_IER_WAKE_IE_Pos)                   /*!< UART_T::IER: WAKE_IE Mask                 */

#define UART_IER_ABAUD_IE_Pos            (7)                                               /*!< UART_T::IER: ABAUD_IE Position            */
#define UART_IER_ABAUD_IE_Msk            (0x1ul << UART_IER_ABAUD_IE_Pos)                  /*!< UART_T::IER: ABAUD_IE Mask                */

#define UART_IER_LIN_IE_Pos              (8)                                               /*!< UART_T::IER: LIN_IE Position              */
#define UART_IER_LIN_IE_Msk              (0x1ul << UART_IER_LIN_IE_Pos)                    /*!< UART_T::IER: LIN_IE Mask                  */

#define UART_ISR_RDA_IS_Pos              (0)                                               /*!< UART_T::ISR: RDA_IS Position              */
#define UART_ISR_RDA_IS_Msk              (0x1ul << UART_ISR_RDA_IS_Pos)                    /*!< UART_T::ISR: RDA_IS Mask                  */

#define UART_ISR_THRE_IS_Pos             (1)                                               /*!< UART_T::ISR: THRE_IS Position             */
#define UART_ISR_THRE_IS_Msk             (0x1ul << UART_ISR_THRE_IS_Pos)                   /*!< UART_T::ISR: THRE_IS Mask                 */

#define UART_ISR_RLS_IS_Pos              (2)                                               /*!< UART_T::ISR: RLS_IS Position              */
#define UART_ISR_RLS_IS_Msk              (0x1ul << UART_ISR_RLS_IS_Pos)                    /*!< UART_T::ISR: RLS_IS Mask                  */

#define UART_ISR_MODEM_IS_Pos            (3)                                               /*!< UART_T::ISR: MODEM_IS Position            */
#define UART_ISR_MODEM_IS_Msk            (0x1ul << UART_ISR_MODEM_IS_Pos)                  /*!< UART_T::ISR: MODEM_IS Mask                */

#define UART_ISR_RTO_IS_Pos              (4)                                               /*!< UART_T::ISR: RTO_IS Position              */
#define UART_ISR_RTO_IS_Msk              (0x1ul << UART_ISR_RTO_IS_Pos)                    /*!< UART_T::ISR: RTO_IS Mask                  */

#define UART_ISR_BUF_ERR_IS_Pos          (5)                                               /*!< UART_T::ISR: BUF_ERR_IS Position          */
#define UART_ISR_BUF_ERR_IS_Msk          (0x1ul << UART_ISR_BUF_ERR_IS_Pos)                /*!< UART_T::ISR: BUF_ERR_IS Mask              */

#define UART_ISR_WAKE_IS_Pos             (6)                                               /*!< UART_T::ISR: WAKE_IS Position             */
#define UART_ISR_WAKE_IS_Msk             (0x1ul << UART_ISR_WAKE_IS_Pos)                   /*!< UART_T::ISR: WAKE_IS Mask                 */

#define UART_ISR_ABAUD_IS_Pos            (7)                                               /*!< UART_T::ISR: ABAUD_IS Position            */
#define UART_ISR_ABAUD_IS_Msk            (0x1ul << UART_ISR_ABAUD_IS_Pos)                  /*!< UART_T::ISR: ABAUD_IS Mask                */

#define UART_ISR_LIN_IS_Pos              (8)                                               /*!< UART_T::ISR: LIN_IS Position              */
#define UART_ISR_LIN_IS_Msk              (0x1ul << UART_ISR_LIN_IS_Pos)                    /*!< UART_T::ISR: LIN_IS Mask                  */

#define UART_TRSR_RS485_ADDET_F_Pos     (0)                                               /*!< UART_T::TRSR: RS485_ADDET_F Position     */
#define UART_TRSR_RS485_ADDET_F_Msk     (0x1ul << UART_TRSR_RS485_ADDET_F_Pos)           /*!< UART_T::TRSR: RS485_ADDET_F Mask         */

#define UART_TRSR_ABAUD_F_Pos            (1)                                               /*!< UART_T::TRSR: ABAUD_F Position            */
#define UART_TRSR_ABAUD_F_Msk            (0x1ul << UART_TRSR_ABAUD_F_Pos)                  /*!< UART_T::TRSR: ABAUD_F Mask                */

#define UART_TRSR_ABAUD_TOUT_F_Pos       (2)                                               /*!< UART_T::TRSR: ABAUD_TOUT_F Position       */
#define UART_TRSR_ABAUD_TOUT_F_Msk       (0x1ul << UART_TRSR_ABAUD_TOUT_F_Pos)             /*!< UART_T::TRSR: ABAUD_TOUT_F Mask           */

#define UART_TRSR_LIN_TX_F_Pos           (3)                                               /*!< UART_T::TRSR: LIN_TX_F Position           */
#define UART_TRSR_LIN_TX_F_Msk           (0x1ul << UART_TRSR_LIN_TX_F_Pos)                 /*!< UART_T::TRSR: LIN_TX_F Mask               */

#define UART_TRSR_LIN_RX_F_Pos           (4)                                               /*!< UART_T::TRSR: LIN_RX_F Position           */
#define UART_TRSR_LIN_RX_F_Msk           (0x1ul << UART_TRSR_LIN_RX_F_Pos)                 /*!< UART_T::TRSR: LIN_RX_F Mask               */

#define UART_TRSR_BIT_ERR_F_Pos          (5)                                               /*!< UART_T::TRSR: BIT_ERR_F Position          */
#define UART_TRSR_BIT_ERR_F_Msk          (0x1ul << UART_TRSR_BIT_ERR_F_Pos)                /*!< UART_T::TRSR: BIT_ERR_F Mask              */

#define UART_TRSR_LIN_RX_SYNC_ERR_F_Pos  (8)                                               /*!< UART_T::TRSR: LIN_RX_SYNC_ERR_F Position  */
#define UART_TRSR_LIN_RX_SYNC_ERR_F_Msk  (0x1ul << UART_TRSR_LIN_RX_SYNC_ERR_F_Pos)        /*!< UART_T::TRSR: LIN_RX_SYNC_ERR_F Mask      */

#define UART_FSR_RX_OVER_F_Pos           (0)                                               /*!< UART_T::FSR: RX_OVER_F Position           */
#define UART_FSR_RX_OVER_F_Msk           (0x1ul << UART_FSR_RX_OVER_F_Pos)                 /*!< UART_T::FSR: RX_OVER_F Mask               */

#define UART_FSR_RX_EMPTY_F_Pos          (1)                                               /*!< UART_T::FSR: RX_EMPTY_F Position          */
#define UART_FSR_RX_EMPTY_F_Msk          (0x1ul << UART_FSR_RX_EMPTY_F_Pos)                /*!< UART_T::FSR: RX_EMPTY_F Mask              */

#define UART_FSR_RX_FULL_F_Pos           (2)                                               /*!< UART_T::FSR: RX_FULL_F Position           */
#define UART_FSR_RX_FULL_F_Msk           (0x1ul << UART_FSR_RX_FULL_F_Pos)                 /*!< UART_T::FSR: RX_FULL_F Mask               */

#define UART_FSR_PE_F_Pos                (4)                                               /*!< UART_T::FSR: PE_F Position                */
#define UART_FSR_PE_F_Msk                (0x1ul << UART_FSR_PE_F_Pos)                      /*!< UART_T::FSR: PE_F Mask                    */

#define UART_FSR_FE_F_Pos                (5)                                               /*!< UART_T::FSR: FE_F Position                */
#define UART_FSR_FE_F_Msk                (0x1ul << UART_FSR_FE_F_Pos)                      /*!< UART_T::FSR: FE_F Mask                    */

#define UART_FSR_BI_F_Pos                (6)                                               /*!< UART_T::FSR: BI_F Position                */
#define UART_FSR_BI_F_Msk                (0x1ul << UART_FSR_BI_F_Pos)                      /*!< UART_T::FSR: BI_F Mask                    */

#define UART_FSR_TX_OVER_F_Pos           (8)                                               /*!< UART_T::FSR: TX_OVER_F Position           */
#define UART_FSR_TX_OVER_F_Msk           (0x1ul << UART_FSR_TX_OVER_F_Pos)                 /*!< UART_T::FSR: TX_OVER_F Mask               */

#define UART_FSR_TX_EMPTY_F_Pos          (9)                                               /*!< UART_T::FSR: TX_EMPTY_F Position          */
#define UART_FSR_TX_EMPTY_F_Msk          (0x1ul << UART_FSR_TX_EMPTY_F_Pos)                /*!< UART_T::FSR: TX_EMPTY_F Mask              */

#define UART_FSR_TX_FULL_F_Pos           (10)                                              /*!< UART_T::FSR: TX_FULL_F Position           */
#define UART_FSR_TX_FULL_F_Msk           (0x1ul << UART_FSR_TX_FULL_F_Pos)                 /*!< UART_T::FSR: TX_FULL_F Mask               */

#define UART_FSR_TE_F_Pos                (11)                                              /*!< UART_T::FSR: TE_F Position                */
#define UART_FSR_TE_F_Msk                (0x1ul << UART_FSR_TE_F_Pos)                      /*!< UART_T::FSR: TE_F Mask                    */

#define UART_FSR_RX_POINTER_F_Pos        (16)                                              /*!< UART_T::FSR: RX_POINTER_F Position        */
#define UART_FSR_RX_POINTER_F_Msk        (0x1ful << UART_FSR_RX_POINTER_F_Pos)             /*!< UART_T::FSR: RX_POINTER_F Mask            */

#define UART_FSR_TX_POINTER_F_Pos        (24)                                              /*!< UART_T::FSR: TX_POINTER_F Position        */
#define UART_FSR_TX_POINTER_F_Msk        (0x1ful << UART_FSR_TX_POINTER_F_Pos)             /*!< UART_T::FSR: TX_POINTER_F Mask            */

#define UART_MCSR_LEV_RTS_Pos            (0)                                               /*!< UART_T::MCSR: LEV_RTS Position            */
#define UART_MCSR_LEV_RTS_Msk            (0x1ul << UART_MCSR_LEV_RTS_Pos)                  /*!< UART_T::MCSR: LEV_RTS Mask                */

#define UART_MCSR_RTS_ST_Pos             (1)                                               /*!< UART_T::MCSR: RTS_ST Position             */
#define UART_MCSR_RTS_ST_Msk             (0x1ul << UART_MCSR_RTS_ST_Pos)                   /*!< UART_T::MCSR: RTS_ST Mask                 */

#define UART_MCSR_LEV_CTS_Pos            (16)                                              /*!< UART_T::MCSR: LEV_CTS Position            */
#define UART_MCSR_LEV_CTS_Msk            (0x1ul << UART_MCSR_LEV_CTS_Pos)                  /*!< UART_T::MCSR: LEV_CTS Mask                */

#define UART_MCSR_CTS_ST_Pos             (17)                                              /*!< UART_T::MCSR: CTS_ST Position             */
#define UART_MCSR_CTS_ST_Msk             (0x1ul << UART_MCSR_CTS_ST_Pos)                   /*!< UART_T::MCSR: CTS_ST Mask                 */

#define UART_MCSR_DCT_F_Pos              (18)                                              /*!< UART_T::MCSR: DCT_F Position              */
#define UART_MCSR_DCT_F_Msk              (0x1ul << UART_MCSR_DCT_F_Pos)                    /*!< UART_T::MCSR: DCT_F Mask                  */

#define UART_TMCTL_TOIC_Pos              (0)                                               /*!< UART_T::TMCTL: TOIC Position              */
#define UART_TMCTL_TOIC_Msk              (0x1fful << UART_TMCTL_TOIC_Pos)                  /*!< UART_T::TMCTL: TOIC Mask                  */

#define UART_TMCTL_DLY_Pos               (16)                                              /*!< UART_T::TMCTL: DLY Position               */
#define UART_TMCTL_DLY_Msk               (0xfful << UART_TMCTL_DLY_Pos)                    /*!< UART_T::TMCTL: DLY Mask                   */

#define UART_BAUD_BRD_Pos                (0)                                               /*!< UART_T::BAUD: BRD Position                */
#define UART_BAUD_BRD_Msk                (0xfffful << UART_BAUD_BRD_Pos)                   /*!< UART_T::BAUD: BRD Mask                    */

#define UART_BAUD_DIV_16_EN_Pos          (31)                                              /*!< UART_T::BAUD: DIV_16_EN Position          */
#define UART_BAUD_DIV_16_EN_Msk          (0x1ul << UART_BAUD_DIV_16_EN_Pos)                /*!< UART_T::BAUD: DIV_16_EN Mask              */

#define UART_IRCR_TX_SELECT_Pos          (1)                                               /*!< UART_T::IRCR: TX_SELECT Position          */
#define UART_IRCR_TX_SELECT_Msk          (0x1ul << UART_IRCR_TX_SELECT_Pos)                /*!< UART_T::IRCR: TX_SELECT Mask              */

#define UART_IRCR_INV_TX_Pos             (5)                                               /*!< UART_T::IRCR: INV_TX Position             */
#define UART_IRCR_INV_TX_Msk             (0x1ul << UART_IRCR_INV_TX_Pos)                   /*!< UART_T::IRCR: INV_TX Mask                 */

#define UART_IRCR_INV_RX_Pos             (6)                                               /*!< UART_T::IRCR: INV_RX Position             */
#define UART_IRCR_INV_RX_Msk             (0x1ul << UART_IRCR_INV_RX_Pos)                   /*!< UART_T::IRCR: INV_RX Mask                 */

#define UART_ALT_CTL_LIN_TX_BCNT_Pos     (0)                                               /*!< UART_T::ALT_CTL: LIN_TX_BCNT Position     */
#define UART_ALT_CTL_LIN_TX_BCNT_Msk     (0x7ul << UART_ALT_CTL_LIN_TX_BCNT_Pos)           /*!< UART_T::ALT_CTL: LIN_TX_BCNT Mask         */

#define UART_ALT_CTL_LIN_HEAD_SEL_Pos    (4)                                               /*!< UART_T::ALT_CTL: LIN_HEAD_SEL Position    */
#define UART_ALT_CTL_LIN_HEAD_SEL_Msk    (0x3ul << UART_ALT_CTL_LIN_HEAD_SEL_Pos)          /*!< UART_T::ALT_CTL: LIN_HEAD_SEL Mask        */

#define UART_ALT_CTL_LIN_RX_EN_Pos       (6)                                               /*!< UART_T::ALT_CTL: LIN_RX_EN Position       */
#define UART_ALT_CTL_LIN_RX_EN_Msk       (0x1ul << UART_ALT_CTL_LIN_RX_EN_Pos)             /*!< UART_T::ALT_CTL: LIN_RX_EN Mask           */

#define UART_ALT_CTL_LIN_TX_EN_Pos       (7)                                               /*!< UART_T::ALT_CTL: LIN_TX_EN Position       */
#define UART_ALT_CTL_LIN_TX_EN_Msk       (0x1ul << UART_ALT_CTL_LIN_TX_EN_Pos)             /*!< UART_T::ALT_CTL: LIN_TX_EN Mask           */

#define UART_ALT_CTL_Bit_ERR_EN_Pos      (8)                                               /*!< UART_T::ALT_CTL: Bit_ERR_EN Position      */
#define UART_ALT_CTL_Bit_ERR_EN_Msk      (0x1ul << UART_ALT_CTL_Bit_ERR_EN_Pos)            /*!< UART_T::ALT_CTL: Bit_ERR_EN Mask          */

#define UART_ALT_CTL_RS485_NMM_Pos      (16)                                              /*!< UART_T::ALT_CTL: RS485_NMM Position      */
#define UART_ALT_CTL_RS485_NMM_Msk      (0x1ul << UART_ALT_CTL_RS485_NMM_Pos)             /*!< UART_T::ALT_CTL: RS485_NMM Mask          */

#define UART_ALT_CTL_RS485_AAD_Pos      (17)                                              /*!< UART_T::ALT_CTL: RS485_AAD Position      */
#define UART_ALT_CTL_RS485_AAD_Msk      (0x1ul << UART_ALT_CTL_RS485_AAD_Pos)             /*!< UART_T::ALT_CTL: RS485_AAD Mask          */

#define UART_ALT_CTL_RS485_AUD_Pos      (18)                                              /*!< UART_T::ALT_CTL: RS485_AUD Position      */
#define UART_ALT_CTL_RS485_AUD_Msk      (0x1ul << UART_ALT_CTL_RS485_AUD_Pos)             /*!< UART_T::ALT_CTL: RS485_AUD Mask          */

#define UART_ALT_CTL_RS485_ADD_EN_Pos   (19)                                              /*!< UART_T::ALT_CTL: RS485_ADD_EN Position   */
#define UART_ALT_CTL_RS485_ADD_EN_Msk   (0x1ul << UART_ALT_CTL_RS485_ADD_EN_Pos)          /*!< UART_T::ALT_CTL: RS485_ADD_EN Mask       */

#define UART_ALT_CTL_ADDR_PID_MATCH_Pos  (24)                                              /*!< UART_T::ALT_CTL: ADDR_PID_MATCH Position  */
#define UART_ALT_CTL_ADDR_PID_MATCH_Msk  (0xfful << UART_ALT_CTL_ADDR_PID_MATCH_Pos)       /*!< UART_T::ALT_CTL: ADDR_PID_MATCH Mask      */

#define UART_FUN_SEL_FUN_SEL_Pos         (0)                                               /*!< UART_T::FUN_SEL: FUN_SEL Position         */
#define UART_FUN_SEL_FUN_SEL_Msk         (0x3ul << UART_FUN_SEL_FUN_SEL_Pos)               /*!< UART_T::FUN_SEL: FUN_SEL Mask             */

/**@}*/ /* UART_CONST */
/**@}*/ /* end of UART register group */


/*---------------------- USB Device Controller -------------------------*/
/**
    @addtogroup USBD USB Device Controller(USBD)
    Memory Mapped Structure for USBD Controller
@{ */

/**
  * @brief USBD endpoints register
  */
typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">BUFSEGx</font><br><p> <font size="2">
Offset: 0x20+x*0x10  Endpoint x Buffer Segmentation Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[8:3]</td><td>BUFSEG</td><td><div style="word-wrap: break-word;"><b>It Is Used To Define The Offset Address For Each Endpoint With The USB SRAM Starting Address Its physical address is USB_SRAM address + {BUFSEG[5:0], 000}; where the USB_SRAM = USB_BASE + 0x100h.</b><br>
Refer to the section 5.4.3.3 for the endpoint SRAM structure and its description.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t BUFSEG;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">MXPLDx</font><br><p> <font size="2">
Offset: 0x24+x*0x10  Endpoint x Maximal Payload Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[8:0]</td><td>MXPLD</td><td><div style="word-wrap: break-word;"><b>Maximal Payload</b><br>
It is used to define the length of data which is transmitted to host (IN token) or the actual length of data receiving from host (OUT token).<br>
It also used to indicate that the endpoint is ready to be transmitted in IN token or received in OUT token.<br>
(1). When the register is written by CPU,<br>
For IN token, the value of MXPLD is used to define the length of data to be transmitted and indicate the data buffer is ready.<br>
For OUT token, it means that the controller is ready to receive data from host and the value of MXPLD is the maximal data length comes from host.<br>
(2). When the register is read by CPU,<br>
For IN token, the value of MXPLD is indicated the length of data be transmitted to host<br>
For OUT token, the value of MXPLD is indicated the actual length of data receiving from host.<br>
Note: Once MXPLD is written, the data packets will be transmitted/received immediately after IN/OUT token arrived.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t MXPLD;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CFGx</font><br><p> <font size="2">
Offset: 0x28+x*0x10  Endpoint x Configuration Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[3:0]</td><td>EP_NUM</td><td><div style="word-wrap: break-word;"><b>Endpoint Number</b><br>
These bits are used to define the endpoint number of the current endpoint<br>
</div></td></tr><tr><td>
[4]</td><td>ISOCH</td><td><div style="word-wrap: break-word;"><b>Isochronous Endpoint</b><br>
This bit is used to set the endpoint as Isochronous endpoint, no handshake.<br>
</div></td></tr><tr><td>
[6:5]</td><td>EPMODE</td><td><div style="word-wrap: break-word;"><b>Endpoint Mode</b><br>
00 = Endpoint is disabled.<br>
01 = Out endpoint.<br>
10 = IN endpoint.<br>
11 = Undefined.<br>
</div></td></tr><tr><td>
[7]</td><td>DSQ_SYNC</td><td><div style="word-wrap: break-word;"><b>Data Sequence Synchronization</b><br>
0 = DATA0 PID.<br>
1 = DATA1 PID.<br>
It is used to specify the DATA0 or DATA1 PID in the current transaction.<br>
It will toggle automatically in IN token after host response ACK.<br>
In the other tokens, the user shall take care of it to confirm the right PID in its transaction.<br>
</div></td></tr><tr><td>
[8]</td><td>CSTALL</td><td><div style="word-wrap: break-word;"><b>Clear STALL Response</b><br>
0 = Disable the device to clear the STALL handshake in setup stage.<br>
1 = Clear the device to response STALL handshake in setup stage.<br>
</div></td></tr><tr><td>
[9]</td><td>SSTALL</td><td><div style="word-wrap: break-word;"><b>Set STALL Response</b><br>
0 = Disable the device to response STALL.<br>
1 = Set the device to respond STALL automatically.<br>
</div></td></tr><tr><td>
[15]</td><td>CLRRDY</td><td><div style="word-wrap: break-word;"><b>Clear Ready</b><br>
When the USBD_MXPLDx register is set by user, it means that the endpoint is ready to transmit or receive data.<br>
If the user wants to disable this transaction before the transaction start, users can set this bit to 1 to disable it and it is auto clear to 0.<br>
For IN token, write '1' to clear the IN token had ready to transmit the data to USB.<br>
For OUT token, write '1' to clear the OUT token had ready to receive the data from USB.<br>
This bit is write 1 only and is always 0 when it is read back.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CFG;
    uint32_t RESERVE;

} USBD_EP_T;

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTL</font><br><p> <font size="2">
Offset: 0x00  USB Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>USB_EN</td><td><div style="word-wrap: break-word;"><b>USB Function Enable</b><br>
0 = USB Disabled.<br>
1 = USB Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>PHY_EN</td><td><div style="word-wrap: break-word;"><b>PHY Transceiver Enable</b><br>
0 = PHY transceiver Disabled.<br>
1 = PHY transceiver Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>PWRDB</td><td><div style="word-wrap: break-word;"><b>Power Down PHY Transceiver, Low Active</b><br>
0 = Power-down related circuit of PHY transceiver.<br>
1 = Turn-on related circuit of PHY transceiver.<br>
</div></td></tr><tr><td>
[3]</td><td>DPPU_EN</td><td><div style="word-wrap: break-word;"><b>Pull-Up Resistor On USB_DP Enable</b><br>
0 = Pull-up resistor in USB_DP bus Disabled.<br>
1 = Pull-up resistor in USB_DP bus will be active.<br>
</div></td></tr><tr><td>
[4]</td><td>DRVSE0</td><td><div style="word-wrap: break-word;"><b>Force USB PHY Transceiver To Drive SE0 (Single Ended Zero)</b><br>
The Single Ended Zero is present when both lines (USB_DP, USB_DM) are being pulled low.<br>
0 = None.<br>
1 = Force USB PHY transceiver to drive SE0.<br>
The default value is "1".<br>
</div></td></tr><tr><td>
[8]</td><td>RWAKEUP</td><td><div style="word-wrap: break-word;"><b>Remote Wake-Up</b><br>
0 = Don't force USB bus to K state.<br>
1 = Force USB bus to K (USB_DP low, USB_DM: high) state, used for remote wake-up.<br>
</div></td></tr><tr><td>
[9]</td><td>WAKEUP_EN</td><td><div style="word-wrap: break-word;"><b>Wake-Up Function Enable</b><br>
0 = USB wake-up function Disabled.<br>
1 = USB wake-up function Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">BUSSTS</font><br><p> <font size="2">
Offset: 0x04  USB Bus Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>USBRST</td><td><div style="word-wrap: break-word;"><b>USB Reset Status</b><br>
1 = Bus reset when SE0 (single-ended 0) more than 2.5uS. It is read only.<br>
</div></td></tr><tr><td>
[1]</td><td>SUSPEND</td><td><div style="word-wrap: break-word;"><b>Suspend Status</b><br>
1 = Bus idle more than 3 ms, either cable is plugged off or host is sleeping. It is read only.<br>
</div></td></tr><tr><td>
[2]</td><td>RESUME</td><td><div style="word-wrap: break-word;"><b>Resume Status</b><br>
1 = Resume from suspend. It is read only.<br>
</div></td></tr><tr><td>
[3]</td><td>TIMEOUT</td><td><div style="word-wrap: break-word;"><b>Time-Out Flag</b><br>
1 = Bus no any response more than 18 bits time. It is read only.<br>
</div></td></tr><tr><td>
[4]</td><td>FLDET</td><td><div style="word-wrap: break-word;"><b>Device Floating Detection</b><br>
0 = The controller didn't attach into the USB.<br>
1 = When the controller is attached into the USB, this bit will be set as "1".<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t BUSSTS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">INTEN</font><br><p> <font size="2">
Offset: 0x08  Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>BUSEVT_IE</td><td><div style="word-wrap: break-word;"><b>Bus Event Interrupt Enable</b><br>
0 = BUS event interrupt Disabled.<br>
1 = BUS event interrupt Enabled.<br>
</div></td></tr><tr><td>
[1]</td><td>USBEVT_IE</td><td><div style="word-wrap: break-word;"><b>USB Event Interrupt Enable</b><br>
0 = USB event interrupt Disabled.<br>
1 = USB event interrupt Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>FLDET_IE</td><td><div style="word-wrap: break-word;"><b>Floating Detect Interrupt Enable</b><br>
0 = Floating detect Interrupt Disabled.<br>
1 = Floating detect Interrupt Enabled.<br>
</div></td></tr><tr><td>
[3]</td><td>WAKEUP_IE</td><td><div style="word-wrap: break-word;"><b>USB Wake-Up Interrupt Enable</b><br>
0 = Wake-up Interrupt Disabled.<br>
1 = Wake-up Interrupt Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t INTEN;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">INTSTS</font><br><p> <font size="2">
Offset: 0x0C  Interrupt Event Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>BUS_STS</td><td><div style="word-wrap: break-word;"><b>BUS Interrupt Status</b><br>
The BUS event means there is bus suspense or bus resume in the bus.<br>
This bit is used to indicate that there is one of events in the bus.<br>
0 = No BUS event is occurred.<br>
1 = BUS event occurred; check USB_BUSSTS [3:0] to know which kind of bus event was occurred, cleared by write "1" to USB_INTSTS [0].<br>
</div></td></tr><tr><td>
[1]</td><td>USB_STS</td><td><div style="word-wrap: break-word;"><b>USB Interrupt Status</b><br>
The USB event means that there is Setup Token, IN token, OUT ACK, ISO IN, or ISO OUT event in the bus.<br>
This bit is used to indicate that there is one of events in the bus.<br>
0 = No USB event is occurred.<br>
1 = USB event occurred, check EPSTS0~7[3:0] in USB_EPSTS [31:8] to know which kind of USB event was occurred, cleared by write "1" to USB_INTSTS [1] or USB_INTSTS[31] or EPEVT0~7.<br>
</div></td></tr><tr><td>
[2]</td><td>FLD_STS</td><td><div style="word-wrap: break-word;"><b>Floating Interrupt Status</b><br>
0 = There is not attached event in the USB.<br>
1 = There is attached event in the USB and it is cleared by write "1" to USB_INTSTS [2].<br>
</div></td></tr><tr><td>
[3]</td><td>WKEUP_STS</td><td><div style="word-wrap: break-word;"><b>Wake-Up Interrupt Status</b><br>
0 = No wake-up event is occurred.<br>
1 = Wake-up event occurred, cleared by write 1 to USB_INTSTS [3].<br>
</div></td></tr><tr><td>
[16]</td><td>EPEVT0</td><td><div style="word-wrap: break-word;"><b>USB Event Status On EP0</b><br>
0 = No event occurred in Endpoint 0.<br>
1 = USB event occurred on Endpoint 0, check USB_EPSTS[11:8] to know which kind of USB event was occurred, cleared by write "1" to USB_INTSTS [16] or USB_INTSTS [1].<br>
</div></td></tr><tr><td>
[17]</td><td>EPEVT1</td><td><div style="word-wrap: break-word;"><b>USB Event Status On EP1</b><br>
0 = No event occurred in Endpoint 1.<br>
1 = USB event occurred on Endpoint 1, check USB_EPSTS[15:12] to know which kind of USB event was occurred, cleared by write "1" to USB_INTSTS [17] or USB_INTSTS [1].<br>
</div></td></tr><tr><td>
[18]</td><td>EPEVT2</td><td><div style="word-wrap: break-word;"><b>USB Event Status On EP2</b><br>
0 = No event occurred in Endpoint 2.<br>
1 = USB event occurred on Endpoint 2, check USB_EPSTS[19:16] to know which kind of USB event was occurred, cleared by write "1" to USB_INTSTS [18] or USB_INTSTS [1].<br>
</div></td></tr><tr><td>
[19]</td><td>EPEVT3</td><td><div style="word-wrap: break-word;"><b>USB Event Status On EP3</b><br>
0 = No event occurred in Endpoint 3.<br>
1 = USB event occurred on Endpoint 3, check USB_EPSTS[23:20] to know which kind of USB event was occurred, cleared by write "1" to USB_INTSTS [19] or USB_INTSTS [1].<br>
</div></td></tr><tr><td>
[20]</td><td>EPEVT4</td><td><div style="word-wrap: break-word;"><b>USB Event Status On EP4</b><br>
0 = No event occurred in Endpoint 4.<br>
1 = USB event occurred on Endpoint 4, check USB_EPSTS[27:24] to know which kind of USB event was occurred, cleared by write "1" to USB_INTSTS [20] or USB_INTSTS [1].<br>
</div></td></tr><tr><td>
[21]</td><td>EPEVT5</td><td><div style="word-wrap: break-word;"><b>USB Event Status On EP5</b><br>
0 = No event occurred in Endpoint 5.<br>
1 = USB event occurred on Endpoint 5, check USB_EPSTS[31:28] to know which kind of USB event was occurred, cleared by write "1" to USB_INTSTS [21] or USB_INTSTS [1].<br>
</div></td></tr><tr><td>
[22]</td><td>EPEVT6</td><td><div style="word-wrap: break-word;"><b>USB Event Status On EP6</b><br>
0 = No event occurred in Endpoint 6.<br>
1 = USB event occurred on Endpoint 6, check USB_EPSTS2[2:0] to know which kind of USB event was occurred, cleared by write "1" to USB_INTSTS [22] or USB_INTSTS [1].<br>
</div></td></tr><tr><td>
[23]</td><td>EPEVT7</td><td><div style="word-wrap: break-word;"><b>USB Event Status On EP7</b><br>
0 = No event occurred in Endpoint 7.<br>
1 = USB event occurred on Endpoint 7, check USB_EPSTS2[6:4] to know which kind of USB event was occurred, cleared by write "1" to USB_INTSTS [23] or USB_INTSTS [1].<br>
</div></td></tr><tr><td>
[31]</td><td>SETUP</td><td><div style="word-wrap: break-word;"><b>Setup Event Status</b><br>
0 = No Setup event.<br>
1 = Setup event occurred, cleared by write "1" to USB_INTSTS[31].<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t INTSTS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">FADDR</font><br><p> <font size="2">
Offset: 0x10  Device 's Function Address Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[6:0]</td><td>FADDR</td><td><div style="word-wrap: break-word;"><b>USB device's function address</b><br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t FADDR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">EPSTS</font><br><p> <font size="2">
Offset: 0x14  Endpoint Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[7]</td><td>OVERRUN</td><td><div style="word-wrap: break-word;"><b>Overrun</b><br>
It means the received data is over the maximum payload number or not.<br>
0 = No overrun.<br>
1 = Out Data more than the Max Payload in MXPLD register or the Setup Data more than 8 Bytes.<br>
</div></td></tr><tr><td>
[11:8]</td><td>EPSTS0</td><td><div style="word-wrap: break-word;"><b>Endpoint 0 Bus Status</b><br>
These bits are used to show the current status of this endpoint.<br>
Definition is the same with EPSTS5(USB_EPSTS[27:24]).<br>
</div></td></tr><tr><td>
[15:12]</td><td>EPSTS1</td><td><div style="word-wrap: break-word;"><b>Endpoint 1 Bus Status</b><br>
These bits are used to show the current status of this endpoint.<br>
Definition is the same with EPSTS5(USB_EPSTS[27:24]).<br>
</div></td></tr><tr><td>
[19:16]</td><td>EPSTS2</td><td><div style="word-wrap: break-word;"><b>Endpoint 2 Bus Status</b><br>
These bits are used to show the current status of this endpoint.<br>
Definition is the same with EPSTS5(USB_EPSTS[27:24]).<br>
</div></td></tr><tr><td>
[23:20]</td><td>EPSTS3</td><td><div style="word-wrap: break-word;"><b>Endpoint 3 Bus Status</b><br>
These bits are used to show the current status of this endpoint.<br>
Definition is the same with EPSTS5(USB_EPSTS[27:24]).<br>
</div></td></tr><tr><td>
[27:24]</td><td>EPSTS4</td><td><div style="word-wrap: break-word;"><b>Endpoint 4 Bus Status</b><br>
These bits are used to show the current status of this endpoint.<br>
Definition is the same with EPSTS5(USB_EPSTS[27:24]).<br>
</div></td></tr><tr><td>
[31:28]</td><td>EPSTS5</td><td><div style="word-wrap: break-word;"><b>Endpoint 5 Bus Status</b><br>
These bits are used to show the current status of this endpoint.<br>
0000 = INACK.<br>
0001 = IN NAK (INTERNAL ONLY).<br>
0010 = OUT Packet Data0 ACK.<br>
0011 = Setup ACK<br>
0110 = OUT Packet Data1 ACK.<br>
0111 = Isochronous transfer end.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t EPSTS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">BUFSEG</font><br><p> <font size="2">
Offset: 0x18  Setup Token Buffer Segmentation Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[8:3]</td><td>BUFSEG</td><td><div style="word-wrap: break-word;"><b>This Register Is Used For Setup Token Only</b><br>
It is used to define the offset address for the Setup Token with the USB SRAM starting address.<br>
Its physical address is USB_SRAM address + {BUFSEG[5:0], 000} where the USB_SRAM = USB_BASE + 0x100h.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t BUFSEG;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">EPSTS2</font><br><p> <font size="2">
Offset: 0x1C  Endpoint Bus Status</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[2:0]</td><td>EPSTS6</td><td><div style="word-wrap: break-word;"><b>Endpoint 6 Bus Status</b><br>
These bits are used to show the current status of this endpoint.<br>
Definition is the same with EPSTS5(USB_EPSTS[27:24]).<br>
</div></td></tr><tr><td>
[6:4]</td><td>EPSTS7</td><td><div style="word-wrap: break-word;"><b>Endpoint 7 Bus Status</b><br>
These bits are used to show the current status of this endpoint.<br>
Definition is the same with EPSTS5(USB_EPSTS[27:24]).<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t EPSTS2;


    USBD_EP_T EP[8];

    uint32_t RESERVE0;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">PDMA</font><br><p> <font size="2">
Offset: 0xA4  USB PDMA Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>PDMA_RW</td><td><div style="word-wrap: break-word;"><b>PDMA_RW</b><br>
0 = The PDMA will read data from memory to USB buffer.<br>
1 = The PDMA will read data from USB buffer to memory.<br>
</div></td></tr><tr><td>
[1]</td><td>PDMA_TRG</td><td><div style="word-wrap: break-word;"><b>Active PDMA Function</b><br>
0 = The PDMA function is not active.<br>
1 = The PDMA function in USB is active.<br>
This bit will be automatically cleared after PDMA transfer done.<br>
</div></td></tr><tr><td>
[2]</td><td>BYTEM</td><td><div style="word-wrap: break-word;"><b>CPU Access USB SRAM Size Mode Select</b><br>
0 = Word Mode: The size of the transfer from CPU to USB SRAM is Word order.<br>
1 = Byte Mode: The size of the transfer from CPU to USB SRAM is Byte order.<br>
</div></td></tr><tr><td>
[3]</td><td>PDMA_RST</td><td><div style="word-wrap: break-word;"><b>PDMA Reset</b><br>
It is used to reset the USB PDMA function into default state.<br>
0 = No Reset PDMA Reset Disable.<br>
1 = Reset the PDMA function in this controller.<br>
Note: it is auto cleared to 0 after the reset function done.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t PDMA;

} USBD_T;

/**
    @addtogroup USBD_CONST USBD Bit Field Definition
    Constant Definitions for USBD Controller
@{ */

#define USBD_CTL_USB_EN_Pos              (0)                                               /*!< USBD_T::CTL: USB_EN Position              */
#define USBD_CTL_USB_EN_Msk              (0x1ul << USBD_CTL_USB_EN_Pos)                    /*!< USBD_T::CTL: USB_EN Mask                  */

#define USBD_CTL_PHY_EN_Pos              (1)                                               /*!< USBD_T::CTL: PHY_EN Position              */
#define USBD_CTL_PHY_EN_Msk              (0x1ul << USBD_CTL_PHY_EN_Pos)                    /*!< USBD_T::CTL: PHY_EN Mask                  */

#define USBD_CTL_PWRDB_Pos               (2)                                               /*!< USBD_T::CTL: PWRDB Position               */
#define USBD_CTL_PWRDB_Msk               (0x1ul << USBD_CTL_PWRDB_Pos)                     /*!< USBD_T::CTL: PWRDB Mask                   */

#define USBD_CTL_DPPU_EN_Pos             (3)                                               /*!< USBD_T::CTL: DPPU_EN Position             */
#define USBD_CTL_DPPU_EN_Msk             (0x1ul << USBD_CTL_DPPU_EN_Pos)                   /*!< USBD_T::CTL: DPPU_EN Mask                 */

#define USBD_CTL_DRVSE0_Pos              (4)                                               /*!< USBD_T::CTL: DRVSE0 Position              */
#define USBD_CTL_DRVSE0_Msk              (0x1ul << USBD_CTL_DRVSE0_Pos)                    /*!< USBD_T::CTL: DRVSE0 Mask                  */

#define USBD_CTL_RWAKEUP_Pos             (8)                                               /*!< USBD_T::CTL: RWAKEUP Position             */
#define USBD_CTL_RWAKEUP_Msk             (0x1ul << USBD_CTL_RWAKEUP_Pos)                   /*!< USBD_T::CTL: RWAKEUP Mask                 */

#define USBD_CTL_WAKEUP_EN_Pos           (9)                                               /*!< USBD_T::CTL: WAKEUP_EN Position           */
#define USBD_CTL_WAKEUP_EN_Msk           (0x1ul << USBD_CTL_WAKEUP_EN_Pos)                 /*!< USBD_T::CTL: WAKEUP_EN Mask               */

#define USBD_BUSSTS_USBRST_Pos           (0)                                               /*!< USBD_T::BUSSTS: USBRST Position           */
#define USBD_BUSSTS_USBRST_Msk           (0x1ul << USBD_BUSSTS_USBRST_Pos)                 /*!< USBD_T::BUSSTS: USBRST Mask               */

#define USBD_BUSSTS_SUSPEND_Pos          (1)                                               /*!< USBD_T::BUSSTS: SUSPEND Position          */
#define USBD_BUSSTS_SUSPEND_Msk          (0x1ul << USBD_BUSSTS_SUSPEND_Pos)                /*!< USBD_T::BUSSTS: SUSPEND Mask              */

#define USBD_BUSSTS_RESUME_Pos           (2)                                               /*!< USBD_T::BUSSTS: RESUME Position           */
#define USBD_BUSSTS_RESUME_Msk           (0x1ul << USBD_BUSSTS_RESUME_Pos)                 /*!< USBD_T::BUSSTS: RESUME Mask               */

#define USBD_BUSSTS_TIMEOUT_Pos          (3)                                               /*!< USBD_T::BUSSTS: TIMEOUT Position          */
#define USBD_BUSSTS_TIMEOUT_Msk          (0x1ul << USBD_BUSSTS_TIMEOUT_Pos)                /*!< USBD_T::BUSSTS: TIMEOUT Mask              */

#define USBD_BUSSTS_FLDET_Pos            (4)                                               /*!< USBD_T::BUSSTS: FLDET Position            */
#define USBD_BUSSTS_FLDET_Msk            (0x1ul << USBD_BUSSTS_FLDET_Pos)                  /*!< USBD_T::BUSSTS: FLDET Mask                */

#define USBD_INTEN_BUSEVT_IE_Pos         (0)                                               /*!< USBD_T::INTEN: BUSEVT_IE Position         */
#define USBD_INTEN_BUSEVT_IE_Msk         (0x1ul << USBD_INTEN_BUSEVT_IE_Pos)               /*!< USBD_T::INTEN: BUSEVT_IE Mask             */

#define USBD_INTEN_USBEVT_IE_Pos         (1)                                               /*!< USBD_T::INTEN: USBEVT_IE Position         */
#define USBD_INTEN_USBEVT_IE_Msk         (0x1ul << USBD_INTEN_USBEVT_IE_Pos)               /*!< USBD_T::INTEN: USBEVT_IE Mask             */

#define USBD_INTEN_FLDET_IE_Pos          (2)                                               /*!< USBD_T::INTEN: FLDET_IE Position          */
#define USBD_INTEN_FLDET_IE_Msk          (0x1ul << USBD_INTEN_FLDET_IE_Pos)                /*!< USBD_T::INTEN: FLDET_IE Mask              */

#define USBD_INTEN_WAKEUP_IE_Pos         (3)                                               /*!< USBD_T::INTEN: WAKEUP_IE Position         */
#define USBD_INTEN_WAKEUP_IE_Msk         (0x1ul << USBD_INTEN_WAKEUP_IE_Pos)               /*!< USBD_T::INTEN: WAKEUP_IE Mask             */

#define USBD_INTSTS_BUS_STS_Pos          (0)                                               /*!< USBD_T::INTSTS: BUS_STS Position          */
#define USBD_INTSTS_BUS_STS_Msk          (0x1ul << USBD_INTSTS_BUS_STS_Pos)                /*!< USBD_T::INTSTS: BUS_STS Mask              */

#define USBD_INTSTS_USB_STS_Pos          (1)                                               /*!< USBD_T::INTSTS: USB_STS Position          */
#define USBD_INTSTS_USB_STS_Msk          (0x1ul << USBD_INTSTS_USB_STS_Pos)                /*!< USBD_T::INTSTS: USB_STS Mask              */

#define USBD_INTSTS_FLD_STS_Pos          (2)                                               /*!< USBD_T::INTSTS: FLD_STS Position          */
#define USBD_INTSTS_FLD_STS_Msk          (0x1ul << USBD_INTSTS_FLD_STS_Pos)                /*!< USBD_T::INTSTS: FLD_STS Mask              */

#define USBD_INTSTS_WKEUP_STS_Pos        (3)                                               /*!< USBD_T::INTSTS: WKEUP_STS Position        */
#define USBD_INTSTS_WKEUP_STS_Msk        (0x1ul << USBD_INTSTS_WKEUP_STS_Pos)              /*!< USBD_T::INTSTS: WKEUP_STS Mask            */

#define USBD_INTSTS_EPEVT0_Pos           (16)                                              /*!< USBD_T::INTSTS: EPEVT0 Position           */
#define USBD_INTSTS_EPEVT0_Msk           (0x1ul << USBD_INTSTS_EPEVT0_Pos)                 /*!< USBD_T::INTSTS: EPEVT0 Mask               */

#define USBD_INTSTS_EPEVT1_Pos           (17)                                              /*!< USBD_T::INTSTS: EPEVT1 Position           */
#define USBD_INTSTS_EPEVT1_Msk           (0x1ul << USBD_INTSTS_EPEVT1_Pos)                 /*!< USBD_T::INTSTS: EPEVT1 Mask               */

#define USBD_INTSTS_EPEVT2_Pos           (18)                                              /*!< USBD_T::INTSTS: EPEVT2 Position           */
#define USBD_INTSTS_EPEVT2_Msk           (0x1ul << USBD_INTSTS_EPEVT2_Pos)                 /*!< USBD_T::INTSTS: EPEVT2 Mask               */

#define USBD_INTSTS_EPEVT3_Pos           (19)                                              /*!< USBD_T::INTSTS: EPEVT3 Position           */
#define USBD_INTSTS_EPEVT3_Msk           (0x1ul << USBD_INTSTS_EPEVT3_Pos)                 /*!< USBD_T::INTSTS: EPEVT3 Mask               */

#define USBD_INTSTS_EPEVT4_Pos           (20)                                              /*!< USBD_T::INTSTS: EPEVT4 Position           */
#define USBD_INTSTS_EPEVT4_Msk           (0x1ul << USBD_INTSTS_EPEVT4_Pos)                 /*!< USBD_T::INTSTS: EPEVT4 Mask               */

#define USBD_INTSTS_EPEVT5_Pos           (21)                                              /*!< USBD_T::INTSTS: EPEVT5 Position           */
#define USBD_INTSTS_EPEVT5_Msk           (0x1ul << USBD_INTSTS_EPEVT5_Pos)                 /*!< USBD_T::INTSTS: EPEVT5 Mask               */

#define USBD_INTSTS_EPEVT6_Pos           (22)                                              /*!< USBD_T::INTSTS: EPEVT6 Position           */
#define USBD_INTSTS_EPEVT6_Msk           (0x1ul << USBD_INTSTS_EPEVT6_Pos)                 /*!< USBD_T::INTSTS: EPEVT6 Mask               */

#define USBD_INTSTS_EPEVT7_Pos           (23)                                              /*!< USBD_T::INTSTS: EPEVT7 Position           */
#define USBD_INTSTS_EPEVT7_Msk           (0x1ul << USBD_INTSTS_EPEVT7_Pos)                 /*!< USBD_T::INTSTS: EPEVT7 Mask               */

#define USBD_INTSTS_SETUP_Pos            (31)                                              /*!< USBD_T::INTSTS: SETUP Position            */
#define USBD_INTSTS_SETUP_Msk            (0x1ul << USBD_INTSTS_SETUP_Pos)                  /*!< USBD_T::INTSTS: SETUP Mask                */

#define USBD_FADDR_FADDR_Pos             (0)                                               /*!< USBD_T::FADDR: FADDR Position             */
#define USBD_FADDR_FADDR_Msk             (0x7ful << USBD_FADDR_FADDR_Pos)                  /*!< USBD_T::FADDR: FADDR Mask                 */

#define USBD_EPSTS_OVERRUN_Pos           (7)                                               /*!< USBD_T::EPSTS: OVERRUN Position           */
#define USBD_EPSTS_OVERRUN_Msk           (0x1ul << USBD_EPSTS_OVERRUN_Pos)                 /*!< USBD_T::EPSTS: OVERRUN Mask               */

#define USBD_EPSTS_EPSTS0_Pos            (8)                                               /*!< USBD_T::EPSTS: EPSTS0 Position            */
#define USBD_EPSTS_EPSTS0_Msk            (0xful << USBD_EPSTS_EPSTS0_Pos)                  /*!< USBD_T::EPSTS: EPSTS0 Mask                */

#define USBD_EPSTS_EPSTS1_Pos            (12)                                              /*!< USBD_T::EPSTS: EPSTS1 Position            */
#define USBD_EPSTS_EPSTS1_Msk            (0xful << USBD_EPSTS_EPSTS1_Pos)                  /*!< USBD_T::EPSTS: EPSTS1 Mask                */

#define USBD_EPSTS_EPSTS2_Pos            (16)                                              /*!< USBD_T::EPSTS: EPSTS2 Position            */
#define USBD_EPSTS_EPSTS2_Msk            (0xful << USBD_EPSTS_EPSTS2_Pos)                  /*!< USBD_T::EPSTS: EPSTS2 Mask                */

#define USBD_EPSTS_EPSTS3_Pos            (20)                                              /*!< USBD_T::EPSTS: EPSTS3 Position            */
#define USBD_EPSTS_EPSTS3_Msk            (0xful << USBD_EPSTS_EPSTS3_Pos)                  /*!< USBD_T::EPSTS: EPSTS3 Mask                */

#define USBD_EPSTS_EPSTS4_Pos            (24)                                              /*!< USBD_T::EPSTS: EPSTS4 Position            */
#define USBD_EPSTS_EPSTS4_Msk            (0xful << USBD_EPSTS_EPSTS4_Pos)                  /*!< USBD_T::EPSTS: EPSTS4 Mask                */

#define USBD_EPSTS_EPSTS5_Pos            (28)                                              /*!< USBD_T::EPSTS: EPSTS5 Position            */
#define USBD_EPSTS_EPSTS5_Msk            (0xful << USBD_EPSTS_EPSTS5_Pos)                  /*!< USBD_T::EPSTS: EPSTS5 Mask                */

#define USBD_BUFSEG_BUFSEG_Pos           (3)                                               /*!< USBD_T::BUFSEG: BUFSEG Position           */
#define USBD_BUFSEG_BUFSEG_Msk           (0x3ful << USBD_BUFSEG_BUFSEG_Pos)                /*!< USBD_T::BUFSEG: BUFSEG Mask               */

#define USBD_EPSTS2_EPSTS6_Pos           (0)                                               /*!< USBD_T::EPSTS2: EPSTS6 Position           */
#define USBD_EPSTS2_EPSTS6_Msk           (0x7ul << USBD_EPSTS2_EPSTS6_Pos)                 /*!< USBD_T::EPSTS2: EPSTS6 Mask               */

#define USBD_EPSTS2_EPSTS7_Pos           (4)                                               /*!< USBD_T::EPSTS2: EPSTS7 Position           */
#define USBD_EPSTS2_EPSTS7_Msk           (0x7ul << USBD_EPSTS2_EPSTS7_Pos)                 /*!< USBD_T::EPSTS2: EPSTS7 Mask               */

#define USBD_BUFSEG_BUFSEG_Pos           (3)                                               /*!< USBD_T::BUFSEG: BUFSEG Position          */
#define USBD_BUFSEG_BUFSEG_Msk           (0x3ful << USBD_BUFSEG_BUFSEG_Pos)                /*!< USBD_T::BUFSEG: BUFSEG Mask              */

#define USBD_MXPLD_MXPLD_Pos             (0)                                               /*!< USBD_T::MXPLD: MXPLD Position            */
#define USBD_MXPLD_MXPLD_Msk             (0x1fful << USBD_MXPLD_MXPLD_Pos)                 /*!< USBD_T::MXPLD: MXPLD Mask                */

#define USBD_CFG_EP_NUM_Pos              (0)                                               /*!< USBD_T::CFG: EP_NUM Position             */
#define USBD_CFG_EP_NUM_Msk              (0xful << USBD_CFG_EP_NUM_Pos)                    /*!< USBD_T::CFG: EP_NUM Mask                 */

#define USBD_CFG_ISOCH_Pos               (4)                                               /*!< USBD_T::CFG: ISOCH Position              */
#define USBD_CFG_ISOCH_Msk               (0x1ul << USBD_CFG_ISOCH_Pos)                     /*!< USBD_T::CFG: ISOCH Mask                  */

#define USBD_CFG_EPMODE_Pos              (5)                                               /*!< USBD_T::CFG: EPMODE Position             */
#define USBD_CFG_EPMODE_Msk              (0x3ul << USBD_CFG_EPMODE_Pos)                    /*!< USBD_T::CFG: EPMODE Mask                 */

#define USBD_CFG_DSQ_SYNC_Pos            (7)                                               /*!< USBD_T::CFG: DSQ_SYNC Position           */
#define USBD_CFG_DSQ_SYNC_Msk            (0x1ul << USBD_CFG_DSQ_SYNC_Pos)                  /*!< USBD_T::CFG: DSQ_SYNC Mask               */

#define USBD_CFG_CSTALL_Pos              (8)                                               /*!< USBD_T::CFG: CSTALL Position             */
#define USBD_CFG_CSTALL_Msk              (0x1ul << USBD_CFG_CSTALL_Pos)                    /*!< USBD_T::CFG: CSTALL Mask                 */

#define USBD_CFG_SSTALL_Pos              (9)                                               /*!< USBD_T::CFG: SSTALL Position             */
#define USBD_CFG_SSTALL_Msk              (0x1ul << USBD_CFG_SSTALL_Pos)                    /*!< USBD_T::CFG: SSTALL Mask                 */

#define USBD_CFG_CLRRDY_Pos              (15)                                              /*!< USBD_T::CFG: CLRRDY Position             */
#define USBD_CFG_CLRRDY_Msk              (0x1ul << USBD_CFG_CLRRDY_Pos)                    /*!< USBD_T::CFG: CLRRDY Mask                 */

#define USBD_PDMA_PDMA_RW_Pos            (0)                                               /*!< USBD_T::PDMA: PDMA_RW Position            */
#define USBD_PDMA_PDMA_RW_Msk            (0x1ul << USBD_PDMA_PDMA_RW_Pos)                  /*!< USBD_T::PDMA: PDMA_RW Mask                */

#define USBD_PDMA_PDMA_TRG_Pos           (1)                                               /*!< USBD_T::PDMA: PDMA_TRG Position           */
#define USBD_PDMA_PDMA_TRG_Msk           (0x1ul << USBD_PDMA_PDMA_TRG_Pos)                 /*!< USBD_T::PDMA: PDMA_TRG Mask               */

#define USBD_PDMA_BYTEM_Pos              (2)                                               /*!< USBD_T::PDMA: BYTEM Position              */
#define USBD_PDMA_BYTEM_Msk              (0x1ul << USBD_PDMA_BYTEM_Pos)                    /*!< USBD_T::PDMA: BYTEM Mask                  */

#define USBD_PDMA_PDMA_RST_Pos           (3)                                               /*!< USBD_T::PDMA: PDMA_RST Position           */
#define USBD_PDMA_PDMA_RST_Msk           (0x1ul << USBD_PDMA_PDMA_RST_Pos)                 /*!< USBD_T::PDMA: PDMA_RST Mask               */

/**@}*/ /* USBD_CONST */
/**@}*/ /* end of USBD register group */


/*---------------------- Watch Dog Timer Controller -------------------------*/
/**
    @addtogroup WDT Watch Dog Timer Controller(WDT)
    Memory Mapped Structure for WDT Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CTL</font><br><p> <font size="2">
Offset: 0x00  Watchdog Timer Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>WTR</td><td><div style="word-wrap: break-word;"><b>Clear Watchdog Timer</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
Set this bit will clear the Watchdog timer.<br>
0 = No effect.<br>
1 = Reset the contents of the Watchdog timer.<br>
Note: This bit will be auto cleared after few clock cycles.<br>
</div></td></tr><tr><td>
[1]</td><td>WTRE</td><td><div style="word-wrap: break-word;"><b>Watchdog Timer Reset Function Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
Setting this bit will enable the Watchdog timer reset function.<br>
0 = Watchdog timer reset function Disabled.<br>
1 = Watchdog timer reset function Enabled.<br>
</div></td></tr><tr><td>
[2]</td><td>WTWKE</td><td><div style="word-wrap: break-word;"><b>Watchdog Timer Wake-Up Function Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Watchdog timer Wake-up CPU function Disabled.<br>
1 = Wake-up function Enabled so that Watchdog timer time-out can wake up CPU from power-down mode.<br>
</div></td></tr><tr><td>
[3]</td><td>WTE</td><td><div style="word-wrap: break-word;"><b>Watchdog Timer Enable</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
0 = Watchdog timer Disabled (this action will reset the internal counter).<br>
1 = Watchdog timer Enabled.<br>
</div></td></tr><tr><td>
[6:4]</td><td>WTIS</td><td><div style="word-wrap: break-word;"><b>Watchdog Timer Interval Selection</b><br>
This is a protected register. Please refer to open lock sequence to program it.<br>
These three bits select the time-out interval for the Watchdog timer.<br>
This count is free running counter.<br>
Please refer to the Table 5-16.<br>
</div></td></tr><tr><td>
[9:8]</td><td>WTRDSEL</td><td><div style="word-wrap: break-word;"><b>Watchdog Timer Reset Delay Select</b><br>
When watchdog timeout happened, software has a time named watchdog reset delay period to clear watchdog timer to prevent watchdog reset happened.<br>
Software can select a suitable value of watchdog reset delay period for different watchdog timeout period.<br>
00 = Watchdog reset delay period is 1026 watchdog clock<br>
01 = Watchdog reset delay period is 130 watchdog clock<br>
10 = Watchdog reset delay period is 18 watchdog clock<br>
11 = Watchdog reset delay period is 3 watchdog clock<br>
This register will be reset if watchdog reset happened<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CTL;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IER</font><br><p> <font size="2">
Offset: 0x04  Watchdog Timer Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>WDT_IE</td><td><div style="word-wrap: break-word;"><b>Watchdog Timer Interrupt Enable</b><br>
0 = Watchdog timer interrupt Disabled.<br>
1 = Watchdog timer interrupt Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IER;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">ISR</font><br><p> <font size="2">
Offset: 0x08  Watchdog Timer Interrupt Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>IS</td><td><div style="word-wrap: break-word;"><b>Watchdog Timer Interrupt Status</b><br>
If the Watchdog timer interrupt is enabled, then the hardware will set this bit to indicate that the Watchdog timer interrupt has occurred.<br>
If the Watchdog timer interrupt is not enabled, then this bit indicates that a time-out period has elapsed.<br>
0 = Watchdog timer interrupt did not occur.<br>
1 = Watchdog timer interrupt occurs.<br>
Note: This bit is read only, but can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[1]</td><td>RST_IS</td><td><div style="word-wrap: break-word;"><b>Watchdog Timer Reset Status</b><br>
When the Watchdog timer initiates a reset, the hardware will set this bit.<br>
This flag can be read by software to determine the source of reset.<br>
Software is responsible to clear it manually by writing "1" to it.<br>
If WTRE is disabled, then the Watchdog timer has no effect on this bit.<br>
0 = Watchdog timer reset did not occur.<br>
1 = Watchdog timer reset occurs.<br>
Note: This bit is read only, but can be cleared by writing "1" to it.<br>
</div></td></tr><tr><td>
[2]</td><td>WAKE_IS</td><td><div style="word-wrap: break-word;"><b>Watchdog Timer Wake-Up Status</b><br>
If Watchdog timer causes system to wake up from power-down mode, this bit will be set to high.<br>
It must be cleared by software with a write "1" to this bit.<br>
0 = Watchdog timer does not cause system wake-up.<br>
1 = Wake system up from power-down mode by Watchdog time-out.<br>
Note1: When system in power-down mode and watchdog time-out, hardware will set WDT_WAKE_IS and WDT_IS.<br>
Note2: After one engine clock, this bit can be cleared by writing "1" to it<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t ISR;

} WDT_T;

/**
    @addtogroup WDT_CONST WDT Bit Field Definition
    Constant Definitions for WDT Controller
@{ */

#define WDT_CTL_WTR_Pos                  (0)                                               /*!< WDT_T::CTL: WTR Position                  */
#define WDT_CTL_WTR_Msk                  (0x1ul << WDT_CTL_WTR_Pos)                        /*!< WDT_T::CTL: WTR Mask                      */

#define WDT_CTL_WTRE_Pos                 (1)                                               /*!< WDT_T::CTL: WTRE Position                 */
#define WDT_CTL_WTRE_Msk                 (0x1ul << WDT_CTL_WTRE_Pos)                       /*!< WDT_T::CTL: WTRE Mask                     */

#define WDT_CTL_WTWKE_Pos                (2)                                               /*!< WDT_T::CTL: WTWKE Position                */
#define WDT_CTL_WTWKE_Msk                (0x1ul << WDT_CTL_WTWKE_Pos)                      /*!< WDT_T::CTL: WTWKE Mask                    */

#define WDT_CTL_WTE_Pos                  (3)                                               /*!< WDT_T::CTL: WTE Position                  */
#define WDT_CTL_WTE_Msk                  (0x1ul << WDT_CTL_WTE_Pos)                        /*!< WDT_T::CTL: WTE Mask                      */

#define WDT_CTL_WTIS_Pos                 (4)                                               /*!< WDT_T::CTL: WTIS Position                 */
#define WDT_CTL_WTIS_Msk                 (0x7ul << WDT_CTL_WTIS_Pos)                       /*!< WDT_T::CTL: WTIS Mask                     */

#define WDT_CTL_WTRDSEL_Pos              (8)                                               /*!< WDT_T::CTL: WTRDSEL Position              */
#define WDT_CTL_WTRDSEL_Msk              (0x3ul << WDT_CTL_WTRDSEL_Pos)                    /*!< WDT_T::CTL: WTRDSEL Mask                  */

#define WDT_IER_IE_Pos                   (0)                                               /*!< WDT_T::IER: IE Position                   */
#define WDT_IER_IE_Msk                   (0x1ul << WDT_IER_IE_Pos)                         /*!< WDT_T::IER: IE Mask                       */

#define WDT_ISR_IS_Pos                   (0)                                               /*!< WDT_T::ISR: IS Position                   */
#define WDT_ISR_IS_Msk                   (0x1ul << WDT_ISR_IS_Pos)                         /*!< WDT_T::ISR: IS Mask                       */

#define WDT_ISR_RST_IS_Pos               (1)                                               /*!< WDT_T::ISR: RST_IS Position               */
#define WDT_ISR_RST_IS_Msk               (0x1ul << WDT_ISR_RST_IS_Pos)                     /*!< WDT_T::ISR: RST_IS Mask                   */

#define WDT_ISR_WAKE_IS_Pos              (2)                                               /*!< WDT_T::ISR: WAKE_IS Position              */
#define WDT_ISR_WAKE_IS_Msk              (0x1ul << WDT_ISR_WAKE_IS_Pos)                    /*!< WDT_T::ISR: WAKE_IS Mask                  */

/**@}*/ /* WDT_CONST */
/**@}*/ /* end of WDT register group */


/*---------------------- Window Watchdog Timer -------------------------*/
/**
    @addtogroup WWDT Window Watchdog Timer(WWDT)
    Memory Mapped Structure for WWDT Controller
@{ */

typedef struct {


/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">RLD</font><br><p> <font size="2">
Offset: 0x00  Window Watchdog Timer Reload Counter Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[31:0]</td><td>RLD</td><td><div style="word-wrap: break-word;"><b>Window Watchdog Timer Reload Counter Register</b><br>
Writing 0x00005AA5 to this register will reload the Window Watchdog Timer counter value to 0x3F.<br>
Note: SW only can write WWDTRLD when WWDT counter value between 0 and WINCMP.<br>
If SW writes WWDTRLD when WWDT counter value larger than WINCMP, WWDT will generate RESET signal.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __O  uint32_t RLD;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">CR</font><br><p> <font size="2">
Offset: 0x04  Window Watchdog Timer Control Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>WWDTEN</td><td><div style="word-wrap: break-word;"><b>Window Watchdog Enable</b><br>
Set this bit to enable Window Watchdog timer.<br>
0 = Window Watchdog timer function Disabled.<br>
1 = Window Watchdog timer function Enabled.<br>
</div></td></tr><tr><td>
[11:8]</td><td>PERIODSEL</td><td><div style="word-wrap: break-word;"><b>WWDT Pre-Scale Period Select</b><br>
These three bits select the pre-scale for the WWDT counter period.<br>
Please refer to Table 5-17<br>
</div></td></tr><tr><td>
[21:16]</td><td>WINCMP</td><td><div style="word-wrap: break-word;"><b>WWDT Window Compare Register</b><br>
Set this register to adjust the valid reload window.<br>
Note: SW only can write WWDTRLD when WWDT counter value between 0 and WINCMP.<br>
If SW writes WWDTRLD when WWDT counter value larger than WWCMP, WWDT will generate RESET signal.<br>
</div></td></tr><tr><td>
[31]</td><td>DBGEN</td><td><div style="word-wrap: break-word;"><b>WWDT Debug Enable</b><br>
0 = WWDT stopped count if system is in Debug mode.<br>
1 = WWDT still counted even system is in Debug mode.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t CR;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">IER</font><br><p> <font size="2">
Offset: 0x08  Window Watchdog Timer Interrupt Enable Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>WWDTIE</td><td><div style="word-wrap: break-word;"><b>WWDT Interrupt Enable</b><br>
Setting this bit will enable the Watchdog timer interrupt function.<br>
0 = Watchdog timer interrupt function Disabled.<br>
1 = Watchdog timer interrupt function Enabled.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t IER;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">STS</font><br><p> <font size="2">
Offset: 0x0C  Window Watchdog Timer Status Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[0]</td><td>IF</td><td><div style="word-wrap: break-word;"><b>WWDT Compare Match Interrupt Flag</b><br>
When WWCMP match the WWDT counter, then this bit is set to 1.<br>
This bit will be cleared by software write 1 to this bit.<br>
</div></td></tr><tr><td>
[1]</td><td>RF</td><td><div style="word-wrap: break-word;"><b>WWDT Reset Flag</b><br>
When WWDT counter down count to 0 or write WWDTRLD during WWDT counter larger than WINCMP, chip will be reset and this bit is set to 1.<br>
Software can write 1 to clear this bit to 0.<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __IO uint32_t STS;

/**

\htmlonly

<html><table class="fixed" border="1" style="border-collapse:collapse;" borderColor=black ><col width="75px" /><col width="125px" /><col width="700px" /><caption align="left"><font size="3">WWDTVAL</font><br><p> <font size="2">
Offset: 0x10  Window Watchdog Timer Counter Value Register</font></caption><thread><tr bgcolor="#8A0808" ><td><font color=white><b>Bits</b></font></td><td><font color=white><b>Field</b></font></td><td><font color=white><b>Descriptions</b></font></td></tr></thread><tbody>
<tr><td>
[5:0]</td><td>VAL</td><td><div style="word-wrap: break-word;"><b>WWDT Counter Value</b><br>
This register reflects the counter value of window watchdog. This register is read only<br>
</div></td></tr></tbody></table></html>

\endhtmlonly

*/

    __I  uint32_t VAL;

} WWDT_T;

/**
    @addtogroup WWDT_CONST WWDT Bit Field Definition
    Constant Definitions for WWDT Controller
@{ */

#define WWDT_RLD_WWDTRLD_Pos             (0)                                               /*!< WWDT_T::RLD: RLD Position             */
#define WWDT_RLD_WWDTRLD_Msk             (0xfffffffful << WWDT_RLD_RLD_Pos)                /*!< WWDT_T::RLD: RLD Mask                 */

#define WWDT_CR_WWDTEN_Pos               (0)                                               /*!< WWDT_T::CR: WWDTEN Position           */
#define WWDT_CR_WWDTEN_Msk               (0x1ul << WWDT_CR_WWDTEN_Pos)                     /*!< WWDT_T::CR: WWDTEN Mask               */

#define WWDT_CR_PERIODSEL_Pos            (8)                                               /*!< WWDT_T::CR: PERIODSEL Position        */
#define WWDT_CR_PERIODSEL_Msk            (0xful << WWDT_CR_PERIODSEL_Pos)                  /*!< WWDT_T::CR: PERIODSEL Mask            */

#define WWDT_CR_WINCMP_Pos               (16)                                              /*!< WWDT_T::CR: WINCMP Position           */
#define WWDT_CR_WINCMP_Msk               (0x3ful << WWDT_CR_WINCMP_Pos)                    /*!< WWDT_T::CR: WINCMP Mask               */

#define WWDT_CR_DBGEN_Pos                (31)                                              /*!< WWDT_T::CR: DBGEN Position            */
#define WWDT_CR_DBGEN_Msk                (0x1ul << WWDT_CR_DBGEN_Pos)                      /*!< WWDT_T::CR: DBGEN Mask                */

#define WWDT_IER_WWDTIE_Pos              (0)                                               /*!< WWDT_T::IER: WWDTIE Position          */
#define WWDT_IER_WWDTIE_Msk              (0x1ul << WWDT_IER_WWDTIE_Pos)                    /*!< WWDT_T::IER: WWDTIE Mask              */

#define WWDT_STS_IF_Pos                  (0)                                               /*!< WWDT_T::STS: IF Position              */
#define WWDT_STS_IF_Msk                  (0x1ul << WWDT_STS_IF_Pos)                        /*!< WWDT_T::STS: IF Mask                  */

#define WWDT_STS_RF_Pos                  (1)                                               /*!< WWDT_T::STS: RF Position              */
#define WWDT_STS_RF_Msk                  (0x1ul << WWDT_STS_RF_Pos)                        /*!< WWDT_T::STS: RF Mask                  */

#define WWDT_VAL_WWDTVAL_Pos             (0)                                               /*!< WWDT_T::VAL: WWDTVAL Position         */
#define WWDT_VAL_WWDTVAL_Msk             (0x3ful << WWDT_VAL_WWDTVAL_Pos)                  /*!< WWDT_T::VAL: WWDTVAL Mask             */

/**@}*/ /* WWDT_CONST */
/**@}*/ /* end of WWDT register group */




#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/** @addtogroup NANO100_PERIPHERAL_MEM_MAP NANO100 Peripheral Memory Map
  Memory Mapped Structure for NANO100 Series Peripheral
  @{
 */
/*!<Peripheral and SRAM base address */
#define FLASH_BASE            ((uint32_t)0x00000000)    ///< Flash base address
#define SRAM_BASE             ((uint32_t)0x20000000)    ///< SRAM base address
#define APB1PERIPH_BASE       ((uint32_t)0x40000000)    ///< APB1 base address
#define APB2PERIPH_BASE       ((uint32_t)0x40100000)    ///< APB2 base address
#define AHBPERIPH_BASE        ((uint32_t)0x50000000)    ///< AHB base address

/*!<Peripheral memory map */

#define WDT_BASE              (APB1PERIPH_BASE + 0x04000)    ///< WDT register base address
#define WWDT_BASE             (APB1PERIPH_BASE + 0x04100)    ///< WWDT register base address
#define RTC_BASE              (APB1PERIPH_BASE + 0x08000)    ///< RTC register base address
#define TIMER0_BASE           (APB1PERIPH_BASE + 0x10000)    ///< TIMER0 register base address
#define TIMER1_BASE           (APB1PERIPH_BASE + 0x10100)    ///< TIMER1 register base address
#define I2C0_BASE             (APB1PERIPH_BASE + 0x20000)    ///< I2C0 register base address
#define SPI0_BASE             (APB1PERIPH_BASE + 0x30000)    ///< SPI0 register base address
#define PWM0_BASE             (APB1PERIPH_BASE + 0x40000)    ///< PWM0 register base address
#define UART0_BASE            (APB1PERIPH_BASE + 0x50000)    ///< UART0 register base address
#define DAC_BASE              (APB1PERIPH_BASE + 0xA0000)    ///< DAC register base address
#define LCD_BASE              (APB1PERIPH_BASE + 0xB0000)    ///< LCD register base address
#define SPI2_BASE             (APB1PERIPH_BASE + 0xD0000)    ///< SPI2 register base address
#define ADC_BASE              (APB1PERIPH_BASE + 0xE0000)    ///< ADC register base address

#define TIMER2_BASE           (APB2PERIPH_BASE + 0x10000)    ///< TIMER2 register base address
#define TIMER3_BASE           (APB2PERIPH_BASE + 0x10100)    ///< TIMER3 register base address
#define SHADOW_BASE           (APB1PERIPH_BASE + 0x10200)    ///< GPIO shadow register base address
#define I2C1_BASE             (APB2PERIPH_BASE + 0x20000)    ///< I2C1 register base address
#define SPI1_BASE             (APB2PERIPH_BASE + 0x30000)    ///< SPI1 register base address
#define PWM1_BASE             (APB2PERIPH_BASE + 0x40000)    ///< PWM1 register base address
#define UART1_BASE            (APB2PERIPH_BASE + 0x50000)    ///< UART1 register base address
#define USBD_BASE             (APB1PERIPH_BASE + 0x60000)    ///< USBD register base address
#define SC0_BASE              (APB2PERIPH_BASE + 0x90000)    ///< SC0 register base address
#define I2S_BASE              (APB2PERIPH_BASE + 0xA0000)    ///< I2S register base address
#define SC1_BASE              (APB2PERIPH_BASE + 0xB0000)    ///< SC1 register base address
#define SC2_BASE              (APB2PERIPH_BASE + 0xC0000)    ///< SC2 register base address

#define SYS_BASE              (AHBPERIPH_BASE + 0x00000)     ///< SYS register base address
#define CLK_BASE              (AHBPERIPH_BASE + 0x00200)     ///< CLK register base address
#define INT_BASE              (AHBPERIPH_BASE + 0x00300)     ///< INT register base address
#define GPIOA_BASE            (AHBPERIPH_BASE + 0x04000)     ///< GPIO port A register base address
#define GPIOB_BASE            (AHBPERIPH_BASE + 0x04040)     ///< GPIO port B register base address
#define GPIOC_BASE            (AHBPERIPH_BASE + 0x04080)     ///< GPIO port C register base address
#define GPIOD_BASE            (AHBPERIPH_BASE + 0x040C0)     ///< GPIO port D register base address
#define GPIOE_BASE            (AHBPERIPH_BASE + 0x04100)     ///< GPIO port E register base address
#define GPIOF_BASE            (AHBPERIPH_BASE + 0x04140)     ///< GPIO port F register base address
#define GPIODBNCE_BASE        (AHBPERIPH_BASE + 0x04180)     ///< GPIO debounce register base address
#define GPIO_PIN_DATA_BASE    (AHBPERIPH_BASE + 0x04200)     ///< GPIO bit access register base address
#define VDMA_BASE             (AHBPERIPH_BASE + 0x08000)     ///< VDMA register base address
#define PDMA1_BASE            (AHBPERIPH_BASE + 0x08100)     ///< PDMA1 register base address
#define PDMA2_BASE            (AHBPERIPH_BASE + 0x08200)     ///< PDMA2 register base address
#define PDMA3_BASE            (AHBPERIPH_BASE + 0x08300)     ///< PDMA3 register base address
#define PDMA4_BASE            (AHBPERIPH_BASE + 0x08400)     ///< PDMA4 register base address
#define PDMA5_BASE            (AHBPERIPH_BASE + 0x08500)     ///< PDMA5 register base address
#define PDMA6_BASE            (AHBPERIPH_BASE + 0x08600)     ///< PDMA6 register base address
#define PDMACRC_BASE          (AHBPERIPH_BASE + 0x08E00)     ///< PDMA global control register base address
#define PDMAGCR_BASE          (AHBPERIPH_BASE + 0x08F00)     ///< PDMA CRC register base address
#define FMC_BASE              (AHBPERIPH_BASE + 0x0C000)     ///< FMC register base address
#define EBI_BASE              (AHBPERIPH_BASE + 0x10000)     ///< EBI register base address

/*@}*/ /* end of group NANO100_PERIPHERAL_MEM_MAP */


/** @addtogroup NANO100_PERIPHERAL_DECLARATION NANO100 Peripheral Declaration
  The Declaration of NANO100 Series Peripheral
  @{
 */
#define WDT                   ((WDT_T *) WDT_BASE)              ///< Pointer to WDT register structure
#define WWDT                  ((WWDT_T *) WWDT_BASE)            ///< Pointer to WWDT register structure
#define RTC                   ((RTC_T *) RTC_BASE)              ///< Pointer to RTC register structure
#define TIMER0                ((TIMER_T *) TIMER0_BASE)         ///< Pointer to TIMER0 register structure
#define TIMER1                ((TIMER_T *) TIMER1_BASE)         ///< Pointer to TIMER1 register structure
#define TIMER2                ((TIMER_T *) TIMER2_BASE)         ///< Pointer to TIMER2 register structure
#define TIMER3                ((TIMER_T *) TIMER3_BASE)         ///< Pointer to TIMER3 register structure
#define SHADOW                ((SHADOW_T *) SHADOW_BASE)        ///< Pointer to GPIO shadow register structure
#define I2C0                  ((I2C_T *) I2C0_BASE)             ///< Pointer to I2C0 register structure
#define I2C1                  ((I2C_T *) I2C1_BASE)             ///< Pointer to I2C1 register structure
#define SPI0                  ((SPI_T *) SPI0_BASE)             ///< Pointer to SPI0 register structure
#define SPI1                  ((SPI_T *) SPI1_BASE)             ///< Pointer to SPI1 register structure
#define SPI2                  ((SPI_T *) SPI2_BASE)             ///< Pointer to SPI2 register structure
#define PWM0                  ((PWM_T *) PWM0_BASE)             ///< Pointer to PWM0 register structure
#define PWM1                  ((PWM_T *) PWM1_BASE)             ///< Pointer to PWM1 register structure
#define UART0                 ((UART_T *) UART0_BASE)           ///< Pointer to UART0 register structure
#define UART1                 ((UART_T *) UART1_BASE)           ///< Pointer to UART1 register structure
#define LCD                   ((LCD_T *) LCD_BASE)              ///< Pointer to LCD register structure
#define ADC                   ((ADC_T *) ADC_BASE)              ///< Pointer to ADC register structure
#define SC0                   ((SC_T *) SC0_BASE)               ///< Pointer to SC0 register structure
#define SC1                   ((SC_T *) SC1_BASE)               ///< Pointer to SC1 register structure
#define SC2                   ((SC_T *) SC2_BASE)               ///< Pointer to SC2 register structure
#define USBD                  ((USBD_T *) USBD_BASE)            ///< Pointer to USBD register structure
#define I2S                   ((I2S_T *) I2S_BASE)              ///< Pointer to I2S register structure
#define DAC                   ((DAC_T *) DAC_BASE)              ///< Pointer to DAC register structure

#define SYS                   ((SYS_T *) SYS_BASE)              ///< Pointer to SYS register structure
#define CLK                   ((CLK_T *) CLK_BASE)              ///< Pointer to CLK register structure
#define INTR                  ((INTR_T *) INTID_BASE)           ///< Pointer to INTR register structure
#define PA                    ((GPIO_T *) GPIOA_BASE)           ///< Pointer to GPIO port A register structure
#define PB                    ((GPIO_T *) GPIOB_BASE)           ///< Pointer to GPIO port B register structure
#define PC                    ((GPIO_T *) GPIOC_BASE)           ///< Pointer to GPIO port C register structure
#define PD                    ((GPIO_T *) GPIOD_BASE)           ///< Pointer to GPIO port D register structure
#define PE                    ((GPIO_T *) GPIOE_BASE)           ///< Pointer to GPIO port E register structure
#define PF                    ((GPIO_T *) GPIOF_BASE)           ///< Pointer to GPIO port F register structure
#define GPIO                  ((GP_DB_T *) GPIODBNCE_BASE)      ///< Pointer to GPIO debounce register structure
#define VDMA                  ((VDMA_T *) VDMA_BASE)            ///< Pointer to VDMA register structure
#define PDMA1                 ((PDMA_T *) PDMA1_BASE)           ///< Pointer to PDMA1 register structure
#define PDMA2                 ((PDMA_T *) PDMA2_BASE)           ///< Pointer to PDMA2 register structure
#define PDMA3                 ((PDMA_T *) PDMA3_BASE)           ///< Pointer to PDMA3 register structure
#define PDMA4                 ((PDMA_T *) PDMA4_BASE)           ///< Pointer to PDMA4 register structure
#define PDMA5                 ((PDMA_T *) PDMA5_BASE)           ///< Pointer to PDMA5 register structure
#define PDMA6                 ((PDMA_T *) PDMA6_BASE)           ///< Pointer to PDMA6 register structure
#define PDMACRC               ((DMA_CRC_T *) PDMACRC_BASE)      ///< Pointer to PDMA CRC register structure
#define PDMAGCR               ((DMA_GCR_T *) PDMAGCR_BASE)      ///< Pointer to PDMA global control register structure
#define FMC                   ((FMC_T *) FMC_BASE)              ///< Pointer to FMC register structure
#define EBI                   ((EBI_T *) EBI_BASE)              ///< Pointer to EBI register structure

/*@}*/ /* end of group NANO100_PERIPHERAL_DECLARATION */

/*@}*/ /* end of group NANO100_Peripherals */

/** @addtogroup NANO100_IO_ROUTINE NANO100 I/O Routines
  The Declaration of NANO100 I/O Routines
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

/*@}*/ /* end of group NANO100_IO_ROUTINE */

/******************************************************************************/
/*                Legacy Constants                                            */
/******************************************************************************/
/** @addtogroup NANO100_legacy_Constants NANO100 Legacy Constants
  NANO100 Legacy Constants
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

/*@}*/ /* end of group NANO100_legacy_Constants */

/*@}*/ /* end of group NANO100_Definitions */

#ifdef __cplusplus
}
#endif


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "sys.h"
#include "clk.h"
#include "adc.h"
#include "dac.h"
#include "fmc.h"
#include "ebi.h"
#include "gpio.h"
#include "i2c.h"
#include "crc.h"
#include "pdma.h"
#include "pwm.h"
#include "rtc.h"
#include "sc.h"
#include "scuart.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include "usbd.h"
#include "wdt.h"
#include "wwdt.h"
#include "i2s.h"
#include "lcd.h"

#endif  // __NANO100SERIES_H__



