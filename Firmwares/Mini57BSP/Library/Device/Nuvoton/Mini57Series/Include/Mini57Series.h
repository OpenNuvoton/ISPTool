/**************************************************************************//**
 * @file     Mini57Series.h
 * @version  V3.0
 * $Revision: 6 $
 * $Date: 17/05/04 5:16p $
 * @brief    Mini57 Series Peripheral Access Layer Header File
 *
 * @note
 * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/



/**
  \mainpage Introduction
  *
  *
  * This user manual describes the usage of Mini57 Series MCU device driver
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
  * Copyright (C) 2017 Nuvoton Technology Corp. All rights reserved.
  */


/**
  * \page pg1 NuMicro Mini57 Series BSP Directory Structure
  * Please refer to Readme.pdf under BSP root directory for the BSP directory structure
  *
  * \page pg2 Revision History
  * Please refer to NuMicro Mini57 Series CMSIS BSP Revision History.pdf under BSP\\Document directory for the BSP Revision History
  *
*/



#ifndef __MINI57_H__
#define __MINI57_H__


#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Definitions Definitions for CMSIS
  This file defines all structures and symbols:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/


/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
*/

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */

typedef enum IRQn
{
    /******  Cortex-M0 Processor Exceptions Numbers ***************************************************/
    NonMaskableInt_IRQn         = -14,      /*!< 2 Non Maskable Interrupt                             */
    HardFault_IRQn              = -13,      /*!< 3 Cortex-M0 Hard Fault Interrupt                     */
    SVCall_IRQn                 = -5,       /*!< 11 Cortex-M0 SV Call Interrupt                       */
    PendSV_IRQn                 = -2,       /*!< 14 Cortex-M0 Pend SV Interrupt                       */
    SysTick_IRQn                = -1,       /*!< 15 Cortex-M0 System Tick Interrupt                   */

    /******  ARMIKMCU Swift specific Interrupt Numbers ************************************************/
    BOD_IRQn                    = 0,        /*!< Brown-Out Low Voltage Detected Interrupt             */
    WDT_IRQn                    = 1,        /*!< Watch Dog Timer Interrupt                            */
    USCI0_IRQn                  = 2,        /*!< USCI0 Interrupt                                      */
    USCI1_IRQn                  = 3,        /*!< USCI1 Interrupt                                      */
    GP_IRQn                     = 4,        /*!< GPIO_PA/PB/PC/PD Interrupt                           */
    EPWM_IRQn                   = 5,        /*!< EPWM interrupt                                       */
    BRAKE0_IRQn                 = 6,        /*!< EPWM brake interrupt from PWM0 or PWM_BRAKE pin      */
    BRAKE1_IRQn                 = 7,        /*!< EPWM brake interrupt from PWM1                       */
    BPWM0_IRQn                  = 8,        /*!< BPWM0 Interrupt                                      */
    BPWM1_IRQn                  = 9,        /*!< BPWM1 Interrupt                                      */
    ECAP_IRQn                   = 15,       /*!< Enhanced Input Capture Interrupt                     */
    CCAP_IRQn                   = 16,       /*!< Continues Input Capture Interrupt                    */
    HIRCTRIM_IRQn               = 21,       /*!< HIRC TRIM  Interrupt                                 */
    TMR0_IRQn                   = 22,       /*!< TIMER0 Interrupt                                     */
    TMR1_IRQn                   = 23,       /*!< TIMER1 Interrupt                                     */
    ACMP_IRQn                   = 26,       /*!< ACMP0/1 Interrupt                                    */
    PWRWU_IRQn                  = 28,       /*!< Chip wake-up from Power-down statue Interrupt        */
    EADC0_IRQn                  = 29,       /*!< EADC0 Interrupt                                      */
    EADC1_IRQn                  = 30,       /*!< EADC1 Interrupt                                      */
    EADCWCMP_IRQn               = 31        /*!< EADC Window Compare Interrupt                        */
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
#include "system_Mini57Series.h"        /* Mini57 System                                          */

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

  @{

*/


/*---------------------- Interrupt Source Controller -------------------------*/
/**
    @addtogroup INT Interrupt Source Controller (INT)
    Memory Mapped Structure for INT Controller
@{ */

typedef struct
{


    /**
     * @var INT_T::INT_NMICTL
     * Offset: 0x80  NMI Source Interrupt Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |NMISEL    |NMI Interrupt Source Selection
     * |        |          |The NMI interrupt to Cortex-M0 can be selected from one of the peripheral interrupt by setting NMTSEL.
     * |[8]     |NMISELEN  |NMI Interrupt Enable Control (Write Protected)
     * |        |          |0 = NMI interrupt Disabled.
     * |        |          |1 = NMI interrupt Enabled.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection
     * |        |          |Refer to the register SYS_REGLCTL at address SYS_BA+0x100.
     * @var INT_T::INT_IRQSTS
     * Offset: 0x84  MCU IRQ Number Identity Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |IRQ       |MCU IRQ Source Register
     * |        |          |The IRQ collects all the interrupts from the peripherals and generates the synchronous interrupt to Cortex-M0 core
     * |        |          |There is one mode to generate interrupt to Cortex-M0 - the normal mode.
     * |        |          |The IRQ collects all interrupts from each peripheral and synchronizes them then interrupts the Cortex-M0.
     * |        |          |When the IRQ[n] is 0, setting IRQ[n] to 1 will generate an interrupt to Cortex-M0 NVIC[n].
     * |        |          |When the IRQ[n] is 1 (mean an interrupt is assert), setting 1 to the MCU_bit[n] will clear the interrupt and setting IRQ[n] 0 has no effect.
     */
    __IO uint32_t INT_NMICTL;            /*!< [0x0080] NMI Source Interrupt Select Control Register                     */
    __IO uint32_t INT_IRQSTS;            /*!< [0x0084] MCU IRQ Number Identity Register                                 */

} INT_T;

/**
    @addtogroup INT_CONST INT Bit Field Definition
    Constant Definitions for INT Controller
@{ */

#define INT_NMICTL_NMISEL_Pos            (0)                                               /*!< INT_T::NMICTL: NMISEL Position         */
#define INT_NMICTL_NMISEL_Msk            (0x1ful << INT_NMICTL_NMISEL_Pos)                 /*!< INT_T::NMICTL: NMISEL Mask             */

#define INT_NMICTL_NMISELEN_Pos          (8)                                               /*!< INT_T::NMICTL: NMISELEN Position       */
#define INT_NMICTL_NMISELEN_Msk          (0x1ul << INT_NMICTL_NMISELEN_Pos)                /*!< INT_T::NMICTL: NMISELEN Mask           */

#define INT_IRQSTS_IRQ_Pos               (0)                                               /*!< INT_T::IRQSTS: IRQ Position            */
#define INT_IRQSTS_IRQ_Msk               (0xfffffffful << INT_IRQSTS_IRQ_Pos)              /*!< INT_T::IRQSTS: IRQ Mask                */

/**@}*/ /* INT_CONST */
/**@}*/ /* end of INT register group */


/*---------------------- System Manger Controller -------------------------*/
/**
    @addtogroup SYS System Manger Controller(SYS)
    Memory Mapped Structure for SYS Controller
@{ */

typedef struct
{


    /**
     * @var SYS_T::PDID
     * Offset: 0x00  Part Device Identification Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDID      |Part Device Identification Number (Read Only)
     * |        |          |This register reflects device part number code
     * |        |          |Software can read this register to identify which device is used.
     * @var SYS_T::RSTSTS
     * Offset: 0x04  System Reset Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PORF      |POR Reset Flag
     * |        |          |The POR reset flag is set by the "Reset Signal" from the Power-on Reset (POR) Controller or bit CHIPRST (SYS_IPRST0[0]) to indicate the previous reset source.
     * |        |          |0 = No reset from POR or CHIPRST.
     * |        |          |1 = Power-on Reset (POR) or CHIPRST had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[1]     |PINRF     |NRESET Pin Reset Flag
     * |        |          |The nRESET pin reset flag is set by the "Reset Signal" from the nRESET Pin to indicate the previous reset source.
     * |        |          |0 = No reset from nRESET pin.
     * |        |          |1 = Pin nRESET had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[2]     |WDTRF     |WDT Reset Flag
     * |        |          |The WDT reset flag is set by the "Reset Signal" from the Watchdog Timer or Window Watchdog Timer to indicate the previous reset source.
     * |        |          |0 = No reset from watchdog timer or window watchdog timer.
     * |        |          |1 = The watchdog timer or window watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note1: Write 1 to clear this bit to 0.
     * |        |          |Note2: Watchdog Timer register RSTF(WDT_CTL[2]) bit is set if the system has been reset by WDT time-out reset
     * |        |          |Window Watchdog Timer register WWDTRF(WWDT_STATUS[1]) bit is set if the system has been reset by WWDT time-out reset.
     * |[3]     |LVRF      |LVR Reset Flag
     * |        |          |The LVR reset flag is set by the "Reset Signal" from the Low Voltage Reset Controller to indicate the previous reset source.
     * |        |          |0 = No reset from LVR.
     * |        |          |1 = LVR controller had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[4]     |BODRF     |BOD Reset Flag
     * |        |          |The BOD reset flag is set by the "Reset Signal" from the Brown-Out Detector to indicate the previous reset source.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[5]     |SYSRF     |System Reset Flag
     * |        |          |The system reset flag is set by the "Reset Signal" from the Cortex-M0 Core to indicate the previous reset source.
     * |        |          |0 = No reset from Cortex-M0.
     * |        |          |1 = The Cortex-M0 had issued the reset signal to reset the system by writing 1 to the bit SYSRESETREQ(AIRCR[2], Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M0 core.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[7]     |CPURF     |CPU Reset Flag
     * |        |          |The CPU reset flag is set by hardware if software writes CPURST (SYS_IPRST0[1]) 1 to reset Cortex-M0 Core and Flash Memory Controller (FMC).
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = The Cortex-M0 Core and FMC are reset by software setting CPURST to 1.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * @var SYS_T::IPRST0
     * Offset: 0x08  Peripheral Reset Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIPRST   |Chip One-Shot Reset (Write Protect)
     * |        |          |Setting this bit will reset the whole chip, including Processor core and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |The CHIPRST is same as the POR reset, all the chip controllers is reset and the chip setting from flash are also reload.
     * |        |          |About the difference between CHIPRST and SYSRESETREQ(AIRCR[2]), please refer to section 6.2.2
     * |        |          |0 = Chip normal operation.
     * |        |          |1 = Chip one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[1]     |CPURST    |Processor Core One-Shot Reset (Write Protect)
     * |        |          |Setting this bit will only reset the processor core and Flash Memory Controller(FMC), and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |0 = Processor core normal operation.
     * |        |          |1 = Processor core one-shot reset.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::IPRST1
     * Offset: 0x0C  Peripheral Reset Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |GPIORST   |GPIO Controller Reset
     * |        |          |0 = GPIO controller normal operation.
     * |        |          |1 = GPIO controller reset.
     * |[2]     |TMR0RST   |Timer0 Controller Reset
     * |        |          |0 = Timer0 controller normal operation.
     * |        |          |1 = Timer0 controller reset.
     * |[3]     |TMR1RST   |Timer1 Controller Reset
     * |        |          |0 = Timer1 controller normal operation.
     * |        |          |1 = Timer1 controller reset.
     * |[8]     |CAPRST    |CAP Controller Reset
     * |        |          |0 = CAP controller normal operation.
     * |        |          |1 = CAP controller reset.
     * |[12]    |PGARST    |PGA Controller Reset
     * |        |          |0 = PGA controller normal operation.
     * |        |          |1 = PGA controller reset.
     * |[16]    |BPWMRST   |Basic PWM Controller Reset
     * |        |          |0 = BPWM controller normal operation.
     * |        |          |1 = BPWM controller reset.
     * |[20]    |EPWMRST   |Enhanced PWM Controller Reset
     * |        |          |0 = EPWM controller normal operation.
     * |        |          |1 = EPWM controller reset.
     * |[24]    |USCI0RST  |USCI0 Controller Reset
     * |        |          |0 = USCI0 controller normal operation.
     * |        |          |1 = USCI0 controller reset.
     * |[25]    |USCI1RST  |USCI1 Controller Reset
     * |        |          |0 = USCI1 controller normal operation.
     * |        |          |1 = USCI1 controller reset.
     * |[28]    |ADCRST    |EADC Controller Reset
     * |        |          |0 = EADC controller normal operation.
     * |        |          |1 = EADC controller reset.
     * |[30]    |ACMPRST   |ACMP Controller Reset
     * |        |          |0 = ACMP controller normal operation.
     * |        |          |1 = ACMP controller reset.
     * @var SYS_T::WAIT
     * Offset: 0x10  HCLK Wait State Cycle Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HCLKWS    |HCLK Wait State Cycle Control Bit
     * |        |          |This bit is used to enable/disable HCLK wait state when access Flash.
     * |        |          |0 = No wait state.
     * |        |          |1 = One wait state inserted when CPU access Flash.
     * |        |          |Note: When HCLK frequency is faster than 48MHz, insert one wait state is necessary.
     * @var SYS_T::BODCTL
     * Offset: 0x18  Brown-Out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODEN     |Brown-Out Detector Enable Bit (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CBODEN (CONFIG0 [18]).
     * |        |          |0 = Brown-out Detector function Disabled.
     * |        |          |1 = Brown-out Detector function Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3:1]   |BODVL     |Brown-Out Detector Threshold Voltage Selection (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CBOV (CONFIG0 [22:20]).
     * |        |          |000 = Brown-Out Detector threshold voltage is 2.0V.
     * |        |          |001 = Brown-Out Detector threshold voltage is 2.2V.
     * |        |          |010 = Brown-Out Detector threshold voltage is 2.4V.
     * |        |          |011 = Brown-Out Detector threshold voltage is 2.7V.
     * |        |          |100 = Brown-Out Detector threshold voltage is 3.0V.
     * |        |          |101 = Brown-Out Detector threshold voltage is 3.7V.
     * |        |          |110 = Brown-Out Detector threshold voltage is 4.0V.
     * |        |          |111 = Brown-Out Detector threshold voltage is 4.3V.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |BODRSTEN  |Brown-Out Reset Enable Bit (Write Protect)
     * |        |          |The default value is set by flash controller user configuration register CBORST(CONFIG0[19]) bit .
     * |        |          |0 = Brown-out "INTERRUPT" function Enabled.
     * |        |          |1 = Brown-out "RESET" function Enabled.
     * |        |          |Note1:
     * |        |          |While the Brown-out Detector function is enabled (BODEN high) and BOD reset function is enabled (BODRSTEN high), BOD will assert a signal to reset chip when the detected voltage is lower than the threshold (BODOUT high).
     * |        |          |While the BOD function is enabled (BODEN high) and BOD interrupt function is enabled (BODRSTEN low), BOD will assert an interrupt if BODOUT is high
     * |        |          |BOD interrupt will keep till to the BODEN set to 0
     * |        |          |BOD interrupt can be blocked by disabling the NVIC BOD interrupt or disabling BOD function (set BODEN low)
     * |        |          |BOD will wake CPU up when BODOUT is high in power-down mode.
     * |        |          |Note2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5]     |BODIF     |Brown-Out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage draft at VDD down through or up through the voltage of BODVL setting.
     * |        |          |1 = When Brown-out Detector detects the VDD is dropped down through the voltage of BODVL setting or the VDD is raised up through the voltage of BODVL setting, this bit is set to 1 and the brown-out interrupt is requested if brown-out interrupt is enabled.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[6]     |BODLPM    |Brown-Out Detector Low Power Mode (Write Protect)
     * |        |          |0 = BOD operate in normal mode (default).
     * |        |          |1 = BOD Low Power mode Enabled.
     * |        |          |Note1: The BOD consumes about 100uA in normal mode, the low power mode can reduce the current to about 1/10 but slow the BOD response.
     * |        |          |Note2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[7]     |BODOUT    |Brown-Out Detector Output Status
     * |        |          |0 = Brown-out Detector output status is 0.
     * |        |          |It means the detected voltage is higher than BODVL setting or BODEN is 0.
     * |        |          |1 = Brown-out Detector output status is 1.
     * |        |          |It means the detected voltage is lower than BODVL setting
     * |        |          |If the BODEN is 0, BOD function disabled , this bit always responds 0000.
     * |[15]    |LVREN     |Low Voltage Reset Enable Bit (Write Protect)
     * |        |          |The LVR function resets the chip when the input power voltage is lower than LVR circuit setting
     * |        |          |LVR function is enabled by default.
     * |        |          |0 = Low Voltage Reset function Disabled.
     * |        |          |1 = Low Voltage Reset function Enabled.
     * |        |          |Note1: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |        |          |Note2:
     * @var SYS_T::IVSCTL
     * Offset: 0x1C  Internal Voltage Source Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |VTEMPEN   |Temperature Sensor Enable Bit
     * |        |          |This bit is used to enable/disable temperature sensor function.
     * |        |          |0 = Temperature sensor function Disabled (default).
     * |        |          |1 = Temperature sensor function Enabled.
     * |        |          |Note: After this bit is set to 1, the value of temperature sensor output can be obtained from EADC conversion result
     * |        |          |Please refer to EADC function chapter for details.
     * @var SYS_T::PORCTL
     * Offset: 0x24  Power-On-Reset Controller Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |POROFF    |Power-On Reset Enable Bit (Write Protect)
     * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again
     * |        |          |User can disable internal POR circuit to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
     * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var SYS_T::GPA_MFP
     * Offset: 0x30  GPIOA Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PA0MFP    |PA.0 Multi-function Pin Selection
     * |[7:4]   |PA1MFP    |PA.1 Multi-function Pin Selection
     * |[11:8]  |PA2MFP    |PA.2 Multi-function Pin Selection
     * |[15:12] |PA3MFP    |PA.3 Multi-function Pin Selection
     * |[19:16] |PA4MFP    |PA.4 Multi-function Pin Selection
     * |[23:20] |PA5MFP    |PA.5 Multi-function Pin Selection
     * @var SYS_T::GPB_MFP
     * Offset: 0x34  GPIOB Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PB0MFP    |PB.0 Multi-function Pin Selection
     * |[7:4]   |PB1MFP    |PB.1 Multi-function Pin Selection
     * |[11:8]  |PB2MFP    |PB.2 Multi-function Pin Selection
     * |[15:12] |PB3MFP    |PB.3 Multi-function Pin Selection
     * |[19:16] |PB4MFP    |PB.4 Multi-function Pin Selection
     * @var SYS_T::GPC_MFP
     * Offset: 0x38  GPIOC Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |PC0MFP    |PC.0 Multi-function Pin Selection
     * |[7:4]   |PC1MFP    |PC.1 Multi-function Pin Selection
     * |[11:8]  |PC2MFP    |PC.2 Multi-function Pin Selection
     * |[15:12] |PC3MFP    |PC.3 Multi-function Pin Selection
     * |[19:16] |PC4MFP    |PC.4 Multi-function Pin Selection
     * @var SYS_T::GPD_MFP
     * Offset: 0x3C  GPIOD Multiple Function Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:4]   |PD1MFP    |PD.1 Multi-function Pin Selection
     * |[11:8]  |PD2MFP    |PD.2 Multi-function Pin Selection
     * |[15:12] |PD3MFP    |PD.3 Multi-function Pin Selection
     * |[19:16] |PD4MFP    |PD.4 Multi-function Pin Selection
     * |[23:20] |PD5MFP    |PD.5 Multi-function Pin Selection
     * |[27:24] |PD6MFP    |PD.6 Multi-function Pin Selection
     * @var SYS_T::IRCTCTL
     * Offset: 0x80  HIRC Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQSEL   |Trim Frequency Selection
     * |        |          |This field indicates the target frequency of 48 MHz internal high speed RC oscillator (HIRC) auto trim.
     * |        |          |0 = Disable HIRC auto trim function.
     * |        |          |1 = Enable HIRC auto trim function and trim HIRC to 4 MHz.
     * |[5:4]   |LOOPSEL   |Trim Calculation Loop Selection
     * |        |          |This field defines that trim value calculation is based on how many 32.768 kHz clock.
     * |        |          |00 = Trim value calculation is based on average difference in 4 32.768 kHz clock.
     * |        |          |01 = Trim value calculation is based on average difference in 8 32.768 kHz clock.
     * |        |          |10 = Trim value calculation is based on average difference in 16 32.768 kHz clock.
     * |        |          |11 = Trim value calculation is based on average difference in 32 32.768 kHz clock.
     * |        |          |Note: For example, if LOOPSEL is set as 00, auto trim circuit will calculate trim value based on the average frequency difference in 4 32.768 kHz clock.
     * |[7:6]   |RETRYCNT  |Trim Value Update Limitation Count
     * |        |          |This field defines that how many times the auto trim circuit will try to update the HIRC trim value before the frequency of HIRC locked.
     * |        |          |Once the HIRC locked, the internal trim value update counter will be reset.
     * |        |          |If the trim value update counter reached this limitation value and frequency of HIRC still doesn't lock, the auto trim operation will be disabled and FREQSEL will be cleared to 00.
     * |        |          |00 = Trim retry count limitation is 64 loops.
     * |        |          |01 = Trim retry count limitation is 128 loops.
     * |        |          |10 = Trim retry count limitation is 256 loops.
     * |        |          |11 = Trim retry count limitation is 512 loops.
     * @var SYS_T::IRCTIEN
     * Offset: 0x84  HIRC Trim Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TFAILIEN  |Trim Failure Interrupt Enable Bit
     * |        |          |This bit controls if an interrupt will be triggered while HIRC trim value update limitation count reached and HIRC frequency still not locked on target frequency set by FREQSEL(SYS_IRCTCTL[1:0]).
     * |        |          |If this bit is high and TFAILIF(SYS_IRCTSTS[1]) is set during auto trim operation, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached.
     * |        |          |0 = Disable TFAILIF(SYS_IRCTSTS[1]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable TFAILIF(SYS_IRCTSTS[1]) status to trigger an interrupt to CPU.
     * |[2]     |CLKEIEN   |Clock Error Interrupt Enable Bit
     * |        |          |This bit controls if CPU would get an interrupt while clock is inaccuracy during auto trim operation.
     * |        |          |If this bit is set to1, and CLKERRIF(SYS_IRCTSTS[2]) is set during auto trim operation, an interrupt will be triggered to notify the clock frequency is inaccuracy.
     * |        |          |0 = Disable CLKERRIF(SYS_IRCTSTS[2]) status to trigger an interrupt to CPU.
     * |        |          |1 = Enable CLKERRIF(SYS_IRCTSTS[2]) status to trigger an interrupt to CPU.
     * @var SYS_T::IRCTISTS
     * Offset: 0x88  HIRC Trim Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQLOCK  |HIRC Frequency Lock Status
     * |        |          |This bit indicates the HIRC frequency is locked.
     * |        |          |This is a status bit and doesn't trigger any interrupt.
     * |        |          |0 = The internal high-speed oscillator frequency doesn't lock at 48 MHz yet.
     * |        |          |1 = The internal high-speed oscillator frequency locked at 48 MHz.
     * |[1]     |TFAILIF   |Trim Failure Interrupt Status
     * |        |          |This bit indicates that HIRC trim value update limitation count reached and the HIRC clock frequency still doesn't be locked
     * |        |          |Once this bit is set, the auto trim operation stopped and FREQSEL(SYS_iRCTCTL[1:0]) will be cleared to 00 by hardware automatically.
     * |        |          |If this bit is set and TFAILIEN(SYS_IRCTIEN[1]) is high, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached
     * |        |          |Write 1 to clear this to 0.
     * |        |          |0 = Trim value update limitation count does not reach.
     * |        |          |1 = Trim value update limitation count reached and HIRC frequency still not locked.
     * |[2]     |CLKERRIF  |Clock Error Interrupt Status
     * |        |          |When the frequency of 32.768 kHz external low speed crystal oscillator (LXT) or 48 MHz internal high speed RC oscillator (HIRC) is shift larger to unreasonable value, this bit will be set and to be an indicate that clock frequency is inaccuracy
     * |        |          |Once this bit is set to 1, the auto trim operation stopped and FREQSEL(SYS_IRCTCL[1:0]) will be cleared to 00 by hardware automatically if CESTOPEN(SYS_IRCTCTL[8]) is set to 1.
     * |        |          |If this bit is set and CLKEIEN(SYS_IRCTIEN[2]) is high, an interrupt will be triggered to notify the clock frequency is inaccuracy
     * |        |          |Write 1 to clear this to 0.
     * |        |          |0 = Clock frequency is accuracy.
     * |        |          |1 = Clock frequency is inaccuracy.
     * @var SYS_T::REGLCTL
     * Offset: 0x100  Register Write-Protection Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |REGLCTL   |Register Write-Protection Disable Index (Read Only)
     * |        |          |0 = Write-protection Enabled for writing protected registers.
     * |        |          |Any write to the protected register is ignore.
     * |        |          |1 = Write-protection Disabled for writing protected registers.
     * |        |          |The Protected registers are:
     * |        |          |SYS_IPRST0
     * |        |          |SYS_IPRST0
     * |        |          |SYS_BODCTL
     * |        |          |LDOCR
     * |        |          |SYS_PORCTL
     * |        |          |CLK_PWRCTL
     * |        |          |CLK_APBCLK bit[0]
     * |        |          |CLK_CLKSEL0
     * |        |          |CLK_CLKSEL1 bit[1:0]
     * |        |          |NMI_SEL bit[8]
     * |        |          |FMC_ISPCTL
     * |        |          |FMC_ISPTRG
     * |        |          |WDT_CTL
     * |        |          |Note: The bits which are write-protected will be noted as" (Write Protect)" beside the
     * |        |          |description.
     * |[7:1]   |REGPROTDIS|Register Write-protection Code (Write Only)
     * |        |          |Some registers have write-protection function
     * |        |          |Writing these registers have to disable the protected function by writing the sequence value 0x59, 0x16, 0x88 to this field
     * |        |          |After this sequence is completed, the SYS_REGLCTL bit will be set to 1 and write-protection registers can be normal write.
     * @var SYS_T::TSOFFSET
     * Offset: 0x114  Temperature sensor offset Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |VTEMP0    |Temperature Sensor Offset Value
     * |        |          |This field reflects temperature sensor output voltage offset at 25oC.
     * |[27:16] |VTEMP1    |Temperature Sensor Offset Value
     * |        |          |This field reflects temperature sensor output voltage offset at 125oC.
     */
    __I  uint32_t PDID;                  /*!< [0x0000] Part Device Identification Number Register                       */
    __IO uint32_t RSTSTS;                /*!< [0x0004] System Reset Status Register                                     */
    __IO uint32_t IPRST0;                /*!< [0x0008] Peripheral Reset Control Register 0                              */
    __IO uint32_t IPRST1;                /*!< [0x000c] Peripheral Reset Control Register 1                              */
    __IO uint32_t WAIT;                  /*!< [0x0010] HCLK Wait State Cycle Control Register                           */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t BODCTL;                /*!< [0x0018] Brown-Out Detector Control Register                              */
    __IO uint32_t IVSCTL;                /*!< [0x001c] Internal Voltage Source Control Register                         */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t PORCTL;                /*!< [0x0024] Power-On-Reset Controller Register                               */
    __I  uint32_t RESERVE2[2];
    __IO uint32_t GPA_MFP;               /*!< [0x0030] GPIOA Multiple Function Control Register                         */
    __IO uint32_t GPB_MFP;               /*!< [0x0034] GPIOB Multiple Function Control Register                         */
    __IO uint32_t GPC_MFP;               /*!< [0x0038] GPIOC Multiple Function Control Register                         */
    __IO uint32_t GPD_MFP;               /*!< [0x003c] GPIOD Multiple Function Control Register                         */
    __I  uint32_t RESERVE3[16];
    __IO uint32_t IRCTCTL;               /*!< [0x0080] HIRC Trim Control Register                                       */
    __IO uint32_t IRCTIEN;               /*!< [0x0084] HIRC Trim Interrupt Enable Register                              */
    __IO uint32_t IRCTISTS;              /*!< [0x0088] HIRC Trim Interrupt Status Register                              */
    __I  uint32_t RESERVE4[29];
    __IO uint32_t REGLCTL;               /*!< [0x0100] Register Write-Protection Control Register                       */
    __I  uint32_t RESERVE5[4];
    __I  uint32_t TSOFFSET;              /*!< [0x0114] Temperature sensor offset Register                               */

} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */

#define SYS_PDID_PDID_Pos                (0)                                               /*!< SYS_T::PDID: PDID Position             */
#define SYS_PDID_PDID_Msk                (0xfffffffful << SYS_PDID_PDID_Pos)               /*!< SYS_T::PDID: PDID Mask                 */

#define SYS_RSTSTS_PORF_Pos              (0)                                               /*!< SYS_T::RSTSTS: PORF Position           */
#define SYS_RSTSTS_PORF_Msk              (0x1ul << SYS_RSTSTS_PORF_Pos)                    /*!< SYS_T::RSTSTS: PORF Mask               */

#define SYS_RSTSTS_PINRF_Pos             (1)                                               /*!< SYS_T::RSTSTS: PINRF Position          */
#define SYS_RSTSTS_PINRF_Msk             (0x1ul << SYS_RSTSTS_PINRF_Pos)                   /*!< SYS_T::RSTSTS: PINRF Mask              */

#define SYS_RSTSTS_WDTRF_Pos             (2)                                               /*!< SYS_T::RSTSTS: WDTRF Position          */
#define SYS_RSTSTS_WDTRF_Msk             (0x1ul << SYS_RSTSTS_WDTRF_Pos)                   /*!< SYS_T::RSTSTS: WDTRF Mask              */

#define SYS_RSTSTS_LVRF_Pos              (3)                                               /*!< SYS_T::RSTSTS: LVRF Position           */
#define SYS_RSTSTS_LVRF_Msk              (0x1ul << SYS_RSTSTS_LVRF_Pos)                    /*!< SYS_T::RSTSTS: LVRF Mask               */

#define SYS_RSTSTS_BODRF_Pos             (4)                                               /*!< SYS_T::RSTSTS: BODRF Position          */
#define SYS_RSTSTS_BODRF_Msk             (0x1ul << SYS_RSTSTS_BODRF_Pos)                   /*!< SYS_T::RSTSTS: BODRF Mask              */

#define SYS_RSTSTS_SYSRF_Pos             (5)                                               /*!< SYS_T::RSTSTS: SYSRF Position          */
#define SYS_RSTSTS_SYSRF_Msk             (0x1ul << SYS_RSTSTS_SYSRF_Pos)                   /*!< SYS_T::RSTSTS: SYSRF Mask              */

#define SYS_RSTSTS_CPURF_Pos             (7)                                               /*!< SYS_T::RSTSTS: CPURF Position          */
#define SYS_RSTSTS_CPURF_Msk             (0x1ul << SYS_RSTSTS_CPURF_Pos)                   /*!< SYS_T::RSTSTS: CPURF Mask              */

#define SYS_IPRST0_CHIPRST_Pos           (0)                                               /*!< SYS_T::IPRST0: CHIPRST Position        */
#define SYS_IPRST0_CHIPRST_Msk           (0x1ul << SYS_IPRST0_CHIPRST_Pos)                 /*!< SYS_T::IPRST0: CHIPRST Mask            */

#define SYS_IPRST0_CPURST_Pos            (1)                                               /*!< SYS_T::IPRST0: CPURST Position         */
#define SYS_IPRST0_CPURST_Msk            (0x1ul << SYS_IPRST0_CPURST_Pos)                  /*!< SYS_T::IPRST0: CPURST Mask             */

#define SYS_IPRST1_GPIORST_Pos           (1)                                               /*!< SYS_T::IPRST1: GPIORST Position        */
#define SYS_IPRST1_GPIORST_Msk           (0x1ul << SYS_IPRST1_GPIORST_Pos)                 /*!< SYS_T::IPRST1: GPIORST Mask            */

#define SYS_IPRST1_TMR0RST_Pos           (2)                                               /*!< SYS_T::IPRST1: TMR0RST Position        */
#define SYS_IPRST1_TMR0RST_Msk           (0x1ul << SYS_IPRST1_TMR0RST_Pos)                 /*!< SYS_T::IPRST1: TMR0RST Mask            */

#define SYS_IPRST1_TMR1RST_Pos           (3)                                               /*!< SYS_T::IPRST1: TMR1RST Position        */
#define SYS_IPRST1_TMR1RST_Msk           (0x1ul << SYS_IPRST1_TMR1RST_Pos)                 /*!< SYS_T::IPRST1: TMR1RST Mask            */

#define SYS_IPRST1_CAPRST_Pos            (8)                                               /*!< SYS_T::IPRST1: CAPRST Position         */
#define SYS_IPRST1_CAPRST_Msk            (0x1ul << SYS_IPRST1_CAPRST_Pos)                  /*!< SYS_T::IPRST1: CAPRST Mask             */

#define SYS_IPRST1_PGARST_Pos            (12)                                              /*!< SYS_T::IPRST1: PGARST Position         */
#define SYS_IPRST1_PGARST_Msk            (0x1ul << SYS_IPRST1_PGARST_Pos)                  /*!< SYS_T::IPRST1: PGARST Mask             */

#define SYS_IPRST1_BPWMRST_Pos           (16)                                              /*!< SYS_T::IPRST1: BPWMRST Position        */
#define SYS_IPRST1_BPWMRST_Msk           (0x1ul << SYS_IPRST1_BPWMRST_Pos)                 /*!< SYS_T::IPRST1: BPWMRST Mask            */

#define SYS_IPRST1_EPWMRST_Pos           (20)                                              /*!< SYS_T::IPRST1: EPWMRST Position        */
#define SYS_IPRST1_EPWMRST_Msk           (0x1ul << SYS_IPRST1_EPWMRST_Pos)                 /*!< SYS_T::IPRST1: EPWMRST Mask            */

#define SYS_IPRST1_USCI0RST_Pos          (24)                                              /*!< SYS_T::IPRST1: USCI0RST Position       */
#define SYS_IPRST1_USCI0RST_Msk          (0x1ul << SYS_IPRST1_USCI0RST_Pos)                /*!< SYS_T::IPRST1: USCI0RST Mask           */

#define SYS_IPRST1_USCI1RST_Pos          (25)                                              /*!< SYS_T::IPRST1: USCI1RST Position       */
#define SYS_IPRST1_USCI1RST_Msk          (0x1ul << SYS_IPRST1_USCI1RST_Pos)                /*!< SYS_T::IPRST1: USCI1RST Mask           */

#define SYS_IPRST1_ADCRST_Pos            (28)                                              /*!< SYS_T::IPRST1: ADCRST Position         */
#define SYS_IPRST1_ADCRST_Msk            (0x1ul << SYS_IPRST1_ADCRST_Pos)                  /*!< SYS_T::IPRST1: ADCRST Mask             */

#define SYS_IPRST1_ACMPRST_Pos           (30)                                              /*!< SYS_T::IPRST1: ACMPRST Position        */
#define SYS_IPRST1_ACMPRST_Msk           (0x1ul << SYS_IPRST1_ACMPRST_Pos)                 /*!< SYS_T::IPRST1: ACMPRST Mask            */

#define SYS_WAIT_HCLKWS_Pos              (0)                                               /*!< SYS_T::WAIT: HCLKWS Position           */
#define SYS_WAIT_HCLKWS_Msk              (0x1ul << SYS_WAIT_HCLKWS_Pos)                    /*!< SYS_T::WAIT: HCLKWS Mask               */

#define SYS_BODCTL_BODEN_Pos             (0)                                               /*!< SYS_T::BODCTL: BODEN Position          */
#define SYS_BODCTL_BODEN_Msk             (0x1ul << SYS_BODCTL_BODEN_Pos)                   /*!< SYS_T::BODCTL: BODEN Mask              */

#define SYS_BODCTL_BODVL_Pos             (1)                                               /*!< SYS_T::BODCTL: BODVL Position          */
#define SYS_BODCTL_BODVL_Msk             (0x7ul << SYS_BODCTL_BODVL_Pos)                   /*!< SYS_T::BODCTL: BODVL Mask              */

#define SYS_BODCTL_BODRSTEN_Pos          (4)                                               /*!< SYS_T::BODCTL: BODRSTEN Position       */
#define SYS_BODCTL_BODRSTEN_Msk          (0x1ul << SYS_BODCTL_BODRSTEN_Pos)                /*!< SYS_T::BODCTL: BODRSTEN Mask           */

#define SYS_BODCTL_BODIF_Pos             (5)                                               /*!< SYS_T::BODCTL: BODIF Position          */
#define SYS_BODCTL_BODIF_Msk             (0x1ul << SYS_BODCTL_BODIF_Pos)                   /*!< SYS_T::BODCTL: BODIF Mask              */

#define SYS_BODCTL_BODLPM_Pos            (6)                                               /*!< SYS_T::BODCTL: BODLPM Position         */
#define SYS_BODCTL_BODLPM_Msk            (0x1ul << SYS_BODCTL_BODLPM_Pos)                  /*!< SYS_T::BODCTL: BODLPM Mask             */

#define SYS_BODCTL_BODOUT_Pos            (7)                                               /*!< SYS_T::BODCTL: BODOUT Position         */
#define SYS_BODCTL_BODOUT_Msk            (0x1ul << SYS_BODCTL_BODOUT_Pos)                  /*!< SYS_T::BODCTL: BODOUT Mask             */

#define SYS_BODCTL_LVREN_Pos             (15)                                              /*!< SYS_T::BODCTL: LVREN Position          */
#define SYS_BODCTL_LVREN_Msk             (0x1ul << SYS_BODCTL_LVREN_Pos)                   /*!< SYS_T::BODCTL: LVREN Mask              */

#define SYS_IVSCTL_VTEMPEN_Pos           (0)                                               /*!< SYS_T::IVSCTL: VTEMPEN Position        */
#define SYS_IVSCTL_VTEMPEN_Msk           (0x1ul << SYS_IVSCTL_VTEMPEN_Pos)                 /*!< SYS_T::IVSCTL: VTEMPEN Mask            */

#define SYS_PORCTL_POROFF_Pos            (0)                                               /*!< SYS_T::PORCTL: POROFF Position         */
#define SYS_PORCTL_POROFF_Msk            (0xfffful << SYS_PORCTL_POROFF_Pos)               /*!< SYS_T::PORCTL: POROFF Mask             */

#define SYS_GPA_MFP_PA0MFP_Pos           (0)                                               /*!< SYS_T::GPA_MFP: PA0MFP Position        */
#define SYS_GPA_MFP_PA0MFP_Msk           (0xful << SYS_GPA_MFP_PA0MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA0MFP Mask            */

#define SYS_GPA_MFP_PA1MFP_Pos           (4)                                               /*!< SYS_T::GPA_MFP: PA1MFP Position        */
#define SYS_GPA_MFP_PA1MFP_Msk           (0xful << SYS_GPA_MFP_PA1MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA1MFP Mask            */

#define SYS_GPA_MFP_PA2MFP_Pos           (8)                                               /*!< SYS_T::GPA_MFP: PA2MFP Position        */
#define SYS_GPA_MFP_PA2MFP_Msk           (0xful << SYS_GPA_MFP_PA2MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA2MFP Mask            */

#define SYS_GPA_MFP_PA3MFP_Pos           (12)                                              /*!< SYS_T::GPA_MFP: PA3MFP Position        */
#define SYS_GPA_MFP_PA3MFP_Msk           (0xful << SYS_GPA_MFP_PA3MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA3MFP Mask            */

#define SYS_GPA_MFP_PA4MFP_Pos           (16)                                              /*!< SYS_T::GPA_MFP: PA4MFP Position        */
#define SYS_GPA_MFP_PA4MFP_Msk           (0xful << SYS_GPA_MFP_PA4MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA4MFP Mask            */

#define SYS_GPA_MFP_PA5MFP_Pos           (20)                                              /*!< SYS_T::GPA_MFP: PA5MFP Position        */
#define SYS_GPA_MFP_PA5MFP_Msk           (0xful << SYS_GPA_MFP_PA5MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA5MFP Mask            */

#define SYS_GPB_MFP_PB0MFP_Pos           (0)                                               /*!< SYS_T::GPB_MFP: PB0MFP Position        */
#define SYS_GPB_MFP_PB0MFP_Msk           (0xful << SYS_GPB_MFP_PB0MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB0MFP Mask            */

#define SYS_GPB_MFP_PB1MFP_Pos           (4)                                               /*!< SYS_T::GPB_MFP: PB1MFP Position        */
#define SYS_GPB_MFP_PB1MFP_Msk           (0xful << SYS_GPB_MFP_PB1MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB1MFP Mask            */

#define SYS_GPB_MFP_PB2MFP_Pos           (8)                                               /*!< SYS_T::GPB_MFP: PB2MFP Position        */
#define SYS_GPB_MFP_PB2MFP_Msk           (0xful << SYS_GPB_MFP_PB2MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB2MFP Mask            */

#define SYS_GPB_MFP_PB3MFP_Pos           (12)                                              /*!< SYS_T::GPB_MFP: PB3MFP Position        */
#define SYS_GPB_MFP_PB3MFP_Msk           (0xful << SYS_GPB_MFP_PB3MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB3MFP Mask            */

#define SYS_GPB_MFP_PB4MFP_Pos           (16)                                              /*!< SYS_T::GPB_MFP: PB4MFP Position        */
#define SYS_GPB_MFP_PB4MFP_Msk           (0xful << SYS_GPB_MFP_PB4MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB4MFP Mask            */

#define SYS_GPC_MFP_PC0MFP_Pos           (0)                                               /*!< SYS_T::GPC_MFP: PC0MFP Position        */
#define SYS_GPC_MFP_PC0MFP_Msk           (0xful << SYS_GPC_MFP_PC0MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC0MFP Mask            */

#define SYS_GPC_MFP_PC1MFP_Pos           (4)                                               /*!< SYS_T::GPC_MFP: PC1MFP Position        */
#define SYS_GPC_MFP_PC1MFP_Msk           (0xful << SYS_GPC_MFP_PC1MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC1MFP Mask            */

#define SYS_GPC_MFP_PC2MFP_Pos           (8)                                               /*!< SYS_T::GPC_MFP: PC2MFP Position        */
#define SYS_GPC_MFP_PC2MFP_Msk           (0xful << SYS_GPC_MFP_PC2MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC2MFP Mask            */

#define SYS_GPC_MFP_PC3MFP_Pos           (12)                                              /*!< SYS_T::GPC_MFP: PC3MFP Position        */
#define SYS_GPC_MFP_PC3MFP_Msk           (0xful << SYS_GPC_MFP_PC3MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC3MFP Mask            */

#define SYS_GPC_MFP_PC4MFP_Pos           (16)                                              /*!< SYS_T::GPC_MFP: PC4MFP Position        */
#define SYS_GPC_MFP_PC4MFP_Msk           (0xful << SYS_GPC_MFP_PC4MFP_Pos)                 /*!< SYS_T::GPC_MFP: PC4MFP Mask            */

#define SYS_GPD_MFP_PD1MFP_Pos           (4)                                               /*!< SYS_T::GPD_MFP: PD1MFP Position        */
#define SYS_GPD_MFP_PD1MFP_Msk           (0xful << SYS_GPD_MFP_PD1MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD1MFP Mask            */

#define SYS_GPD_MFP_PD2MFP_Pos           (8)                                               /*!< SYS_T::GPD_MFP: PD2MFP Position        */
#define SYS_GPD_MFP_PD2MFP_Msk           (0xful << SYS_GPD_MFP_PD2MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD2MFP Mask            */

#define SYS_GPD_MFP_PD3MFP_Pos           (12)                                               /*!< SYS_T::GPD_MFP: PD3MFP Position        */
#define SYS_GPD_MFP_PD3MFP_Msk           (0xful << SYS_GPD_MFP_PD3MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD3MFP Mask            */

#define SYS_GPD_MFP_PD4MFP_Pos           (16)                                               /*!< SYS_T::GPD_MFP: PD4MFP Position        */
#define SYS_GPD_MFP_PD4MFP_Msk           (0xful << SYS_GPD_MFP_PD4MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD4MFP Mask            */

#define SYS_GPD_MFP_PD5MFP_Pos           (20)                                               /*!< SYS_T::GPD_MFP: PD5MFP Position        */
#define SYS_GPD_MFP_PD5MFP_Msk           (0xful << SYS_GPD_MFP_PD5MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD5MFP Mask            */

#define SYS_GPD_MFP_PD6MFP_Pos           (24)                                               /*!< SYS_T::GPD_MFP: PD6MFP Position        */
#define SYS_GPD_MFP_PD6MFP_Msk           (0xful << SYS_GPD_MFP_PD6MFP_Pos)                 /*!< SYS_T::GPD_MFP: PD6MFP Mask            */

#define SYS_IRCTCTL_FREQSEL_Pos          (0)                                               /*!< SYS_T::IRCTCTL: FREQSEL Position       */
#define SYS_IRCTCTL_FREQSEL_Msk          (0x1ul << SYS_IRCTCTL_FREQSEL_Pos)                /*!< SYS_T::IRCTCTL: FREQSEL Mask           */

#define SYS_IRCTCTL_LOOPSEL_Pos          (4)                                               /*!< SYS_T::IRCTCTL: LOOPSEL Position       */
#define SYS_IRCTCTL_LOOPSEL_Msk          (0x3ul << SYS_IRCTCTL_LOOPSEL_Pos)                /*!< SYS_T::IRCTCTL: LOOPSEL Mask           */

#define SYS_IRCTCTL_RETRYCNT_Pos         (6)                                               /*!< SYS_T::IRCTCTL: RETRYCNT Position      */
#define SYS_IRCTCTL_RETRYCNT_Msk         (0x3ul << SYS_IRCTCTL_RETRYCNT_Pos)               /*!< SYS_T::IRCTCTL: RETRYCNT Mask          */

#define SYS_IRCTIEN_TFAILIEN_Pos         (1)                                               /*!< SYS_T::IRCTIEN: TFAILIEN Position      */
#define SYS_IRCTIEN_TFAILIEN_Msk         (0x1ul << SYS_IRCTIEN_TFAILIEN_Pos)               /*!< SYS_T::IRCTIEN: TFAILIEN Mask          */

#define SYS_IRCTIEN_CLKEIEN_Pos          (2)                                               /*!< SYS_T::IRCTIEN: CLKEIEN Position       */
#define SYS_IRCTIEN_CLKEIEN_Msk          (0x1ul << SYS_IRCTIEN_CLKEIEN_Pos)                /*!< SYS_T::IRCTIEN: CLKEIEN Mask           */

#define SYS_IRCTISTS_FREQLOCK_Pos        (0)                                               /*!< SYS_T::IRCTISTS: FREQLOCK Position     */
#define SYS_IRCTISTS_FREQLOCK_Msk        (0x1ul << SYS_IRCTISTS_FREQLOCK_Pos)              /*!< SYS_T::IRCTISTS: FREQLOCK Mask         */

#define SYS_IRCTISTS_TFAILIF_Pos         (1)                                               /*!< SYS_T::IRCTISTS: TFAILIF Position      */
#define SYS_IRCTISTS_TFAILIF_Msk         (0x1ul << SYS_IRCTISTS_TFAILIF_Pos)               /*!< SYS_T::IRCTISTS: TFAILIF Mask          */

#define SYS_IRCTISTS_CLKERRIF_Pos        (2)                                               /*!< SYS_T::IRCTISTS: CLKERRIF Position     */
#define SYS_IRCTISTS_CLKERRIF_Msk        (0x1ul << SYS_IRCTISTS_CLKERRIF_Pos)              /*!< SYS_T::IRCTISTS: CLKERRIF Mask         */

#define SYS_REGLCTL_REGLCTL_Pos          (0)                                               /*!< SYS_T::REGLCTL: REGLCTLT Position      */
#define SYS_REGLCTL_REGLCTL_Msk          (0x1ul << SYS_REGLCTL_REGLCTL_Pos)                /*!< SYS_T::REGLCTL: REGLCTL Mask           */

#define SYS_REGLCTL_REGPROTDIS_Pos       (0)                                               /*!< SYS_T::REGLCTL: REGPROTDIS Position    */
#define SYS_REGLCTL_REGPROTDIS_Msk       (0xfful << SYS_REGLCTL_REGPROTDIS_Pos)            /*!< SYS_T::REGLCTL: REGPROTDIS Mask        */


#define SYS_TSOFFSET_VTEMP0_Pos          (0)                                               /*!< SYS_T::TSOFFSET: VTEMP0 Position       */
#define SYS_TSOFFSET_VTEMP0_Msk          (0xffful << SYS_TSOFFSET_VTEMP0_Pos)              /*!< SYS_T::TSOFFSET: VTEMP0 Mask           */

#define SYS_TSOFFSET_VTEMP1_Pos          (16)                                              /*!< SYS_T::TSOFFSET: VTEMP1 Position       */
#define SYS_TSOFFSET_VTEMP1_Msk          (0xffful << SYS_TSOFFSET_VTEMP1_Pos)              /*!< SYS_T::TSOFFSET: VTEMP1 Mask           */

/**@}*/ /* SYS_CONST */
/**@}*/ /* end of SYS register group */


/*---------------------- System Clock Controller -------------------------*/
/**
    @addtogroup CLK System Clock Controller(CLK)
    Memory Mapped Structure for CLK Controller
@{ */

typedef struct
{


    /**
     * @var CLK_T::PWRCTL
     * Offset: 0x00  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |XTLEN     |XTL Enable Bit (Write Protect)
     * |        |          |These two bits are default set to "00" and the XT_IN and XT_OUT pins are GPIO.
     * |        |          |00 = XT_IN and XT_OUT are GPIO, disable both LXT & HXT (default).
     * |        |          |01 = HXT Enabled.
     * |        |          |10 = LXT Enabled.
     * |        |          |11 = XT_IN is external clock input pin, XT_OUT is GPIO.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |HIRCEN    |HIRC Enable Bit (Write Protect)
     * |        |          |0 = 48 MHz internal high speed RC oscillator (HIRC) Disabled.
     * |        |          |1 = 48 MHz internal high speed RC oscillator (HIRC) Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[3]     |LIRCEN    |LIRC Enable Bit (Write Protect)
     * |        |          |0 = 10 kHz internal low speed RC oscillator (LIRC) Disabled.
     * |        |          |1 = 10 kHz internal low speed RC oscillator (LIRC) Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4]     |PDWKDLY   |Enable The Wake-Up Delay Counter (Write Protect)
     * |        |          |When the chip wakes up from Power-down mode, the clock control will delay certain clock cycles to wait system clock stable.
     * |        |          |The delayed clock cycle is 4096 clock cycles when chip works at 4~24 MHz external high speed crystal oscillator (HXT), and 256 clock cycles when chip works at 48 MHz internal high speed RC oscillator (HIRC).
     * |        |          |0 = Clock cycles delay Disabled.
     * |        |          |1 = Clock cycles delay Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5]     |PDWKIEN   |Power-Down Mode Wake-Up Interrupt Enable Bit (Write Protect)
     * |        |          |0 = Power-down mode wake-up interrupt Disabled.
     * |        |          |1 = Power-down mode wake-up interrupt Enabled.
     * |        |          |Note1: The interrupt will occur when both PDWKIF and PDWKIEN are high.
     * |        |          |Note2: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[6]     |PDWKIF    |Power-Down Mode Wake-Up Interrupt Status
     * |        |          |Set by "Power-down wake-up event", it indicates that resume from "Power-down mode"
     * |        |          |The flag is set if the GPIO, USCI01, WDT, ACMP01, BOD, TMR01 wake-up occurred.
     * |        |          |Note1: Write 1 to clear the bit to 0.
     * |        |          |Note2: This bit works only if PDWKIEN (CLK_PWRCTL[5]) set to 1.
     * |[7]     |PDEN      |System Power-Down Enable (Write Protect)
     * |        |          |When this bit is set to 1, Power-down mode is enabled.
     * |        |          |When chip wakes up from Power-down mode, this bit is auto cleared
     * |        |          |Users need to set this bit again for next Power-down.
     * |        |          |In Power-down mode, HXT and the HIRC will be disabled in this mode, but LXT and LIRC are not controlled by Power-down mode.
     * |        |          |In Power-down mode, the system clocks are disabled, and ignored the clock source selection
     * |        |          |The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from LXT or LIRC.
     * |        |          |0 = Chip operating normally or chip in idle mode because of WFI/WFE command.
     * |        |          |1 = Chip enters Power-down mode when CPU sleep command WFI/WFE.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[9]     |PDLXT     |LXT Alive In Power-Down
     * |        |          |0 = LXT will be turned off automatically when chip into Power-Down.
     * |        |          |1 = If XTLEN[1:0] are 0x2, LXT keep active in Power-Down.
     * |[11:10] |HXTGAIN   |HXT Gain Control Bit (Write Protect)
     * |        |          |This is a protected register. Please refer to open lock sequence to program it.
     * |        |          |Gain control is used to enlarge the gain of crystal to make sure crystal work normally
     * |        |          |If gain control is enabled, crystal will consume more power than gain control off.
     * |        |          |11 = HXT frequency is higher than 16 MHz.
     * |        |          |10 = HXT frequency is from 12 MHz to 16 MHz.
     * |        |          |01 = HXT frequency is from 8 MHz to 12 MHz.
     * |        |          |00 = HXT frequency is lower than from 8 MHz.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::AHBCLK
     * Offset: 0x04  AHB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |ISPCKEN   |Flash ISP Controller Clock Enable Bit
     * |        |          |0 = Flash ISP peripheral clock Disabled.
     * |        |          |1 = Flash ISP peripheral clock Enabled.
     * |[4]     |HDIVCKEN  |Hardware Divider Controller Clock Enable Bit
     * |        |          |0 = HDIV peripheral clock Disabled.
     * |        |          |1 = HDIV peripheral clock Enabled.
     * @var CLK_T::APBCLK
     * Offset: 0x08  APB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDTCKEN   |Watchdog Timer Clock Enable Bit (Write Protect)
     * |        |          |0 = Watchdog timer clock Disabled.
     * |        |          |1 = Watchdog timer clock Enabled.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[2]     |TMR0CKEN  |Timer0 Clock Enable Bit
     * |        |          |0 = Timer0 clock Disabled.
     * |        |          |1 = Timer0 clock Enabled.
     * |[3]     |TMR1CKEN  |Timer1 Clock Enable Bit
     * |        |          |0 = Timer1 clock Disabled.
     * |        |          |1 = Timer1 clock Enabled.
     * |[6]     |CLKOCKEN  |CLKO Clock Enable Bit
     * |        |          |0 = CLKO clock Disabled.
     * |        |          |1 = CLKO clock Enabled.
     * |[8]     |ECAPCKEN  |Input Capture Clock Enable Bit
     * |        |          |0 = CAP clock Disabled.
     * |        |          |1 = CAP clock Enabled.
     * |[12]    |PGACKEN   |PGA Clock Enable Bit
     * |        |          |0 = PGA clock Disabled.
     * |        |          |1 = PGA clock Enabled.
     * |[20]    |EPWMCKEN  |Enhanced PWM Clock Enable Bit
     * |        |          |0 = EPWM channel 0/1 clock Disabled.
     * |        |          |1 = EPWM channel 0/1 clock Enabled.
     * |[23]    |BPWMCKEN  |Basic PWM Channel 0/1 Clock Enable Bit
     * |        |          |0 = BBPWM channel 0/1 clock Disabled.
     * |        |          |1 = BPWM channel 0/1 clock Enabled.
     * |[24]    |USCI0CKEN |USCI0 Clock Enable Bit
     * |        |          |0 = USCI0 clock Disabled.
     * |        |          |1 = USCI0 clock Enabled.
     * |[25]    |USCI1CKEN |USCI1 Clock Enable Bit
     * |        |          |0 = USCI1 clock Disabled.
     * |        |          |1 = USCI1 clock Enabled.
     * |[28]    |ADCCKEN   |Enhanced Analog-Digital-Converter (EADC) Clock Enable Bit
     * |        |          |0 = EADC clock Disabled.
     * |        |          |1 = EADC clock Enabled.
     * |[30]    |ACMPCKEN  |Analog Comparator Clock Enable Bit
     * |        |          |0 = Analog comparator clock Disabled.
     * |        |          |1 = Analog comparator clock Enabled.
     * @var CLK_T::CLKSEL0
     * Offset: 0x10  Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |HCLKSEL   |HCLK Clock Source Selection (Write Protect)
     * |        |          |Before clock switching, the related clock sources (both pre-select and new-select) must be turned on.
     * |        |          |00 = Clock source from HXT/LXT.
     * |        |          |01 = Clock source from LIRC.
     * |        |          |11= Clock source from HIRC.
     * |        |          |Other = Reserved.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[4:3]   |STCLKSEL  |Cortex-M0 SysTick Clock Source Selection (Write Protect)
     * |        |          |If SYST_CTL[2]=0, SysTick uses listed clock source below.
     * |        |          |00 = Clock source from HXT/LXT.
     * |        |          |01 = Clock source from (HXT or LXT)/2.
     * |        |          |10 = Clock source from HCLK/2.
     * |        |          |11 = Clock source from HIRC/2.
     * |        |          |Other = Reserved.
     * |        |          |Note: if SysTick clock source is not from HCLK (i.e
     * |        |          |SYST_CTL[2] = 0), SysTick clock source must less than or equal to HCLK/2.
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * @var CLK_T::CLKSEL1
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDTSEL    |Watchdog Timer Clock Source Selection (Write Protect)
     * |        |          |00 = Clock source from external crystal oscillator (HXT or LXT).
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source from HCLK0/2048.
     * |        |          |11 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |Note: This bit is write protected. Refer to the SYS_REGLCTL register.
     * |[5:4]   |ADCSEL    |EADC Peripheral Clock Source Selection
     * |        |          |00 = Clock source from external crystal oscillator (HXT or LXT).
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source is from HCLK.
     * |        |          |11 = Clock source from 48 MHz internal high speed RC oscillator (HIRC).
     * |[10:8]  |TMR0SEL   |TIMER0 Clock Source Selection
     * |        |          |000 = Clock source from external crystal oscillator (HXT or LXT).
     * |        |          |001 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |010 = Clock source from HCLK.
     * |        |          |011 = Clock source from external clock T0 pin.
     * |        |          |111 = Clock source from 48 MHz internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[14:12] |TMR1SEL   |TIMER1 Clock Source Selection
     * |        |          |000 = Clock source from external crystal oscillator (HXT or LXT).
     * |        |          |001 = Clock source from 10 kHz internal low speed RC oscillator (LIRC).
     * |        |          |010 = Clock source from HCLK.
     * |        |          |011 = Clock source from external clock T1 pin.
     * |        |          |111 = Clock source from 48 MHz internal high speed RC oscillator (HIRC).
     * |        |          |Others = Reserved.
     * |[31:30] |CLKOSEL   |Clock Divider Clock Source Selection
     * |        |          |00 = Clock source from external crystal oscillator (HXT or LXT).
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source from HCLK.
     * |        |          |11 = Clock source from 48 MHz internal high speed RC oscillator (HIRC).
     * @var CLK_T::CLKDIV
     * Offset: 0x20  Clock Divider Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLKDIV   |HCLK Clock Divide Number From HCLK Clock Source
     * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLKDIV + 1).
     * |[23:16] |ADCDIV    |EADC Clock Divide Number From EADC Clock Source
     * |        |          |EADC clock frequency = (EADC clock source frequency) / (ADCDIV + 1).
     * @var CLK_T::STATUS
     * Offset: 0x50  Clock Status Monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |XTLSTB    |XTL Clock Source Stable Flag (Read Only)
     * |        |          |0 = External crystal oscillator (HXT or LXT) clock is not stable or disabled.
     * |        |          |1 = External crystal oscillator (HXT or LXT) clock is stable and enabled.
     * |[3]     |LIRCSTB   |LIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 10 kHz internal low speed RC oscillator (LIRC) clock is not stable or disabled.
     * |        |          |1 = 10 kHz internal low speed RC oscillator (LIRC) clock is stable and enabled.
     * |[4]     |HIRCSTB   |HIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = 48 MHz internal high speed RC oscillator (HIRC) clock is not stable or disabled.
     * |        |          |1 = 48 MHz internal high speed RC oscillator (HIRC) clock is stable and enabled.
     * |[7]     |CLKSFAIL  |Clock Switching Fail Flag (Read Only)
     * |        |          |This bit is updated when software switches system clock source
     * |        |          |If switch target clock is stable, this bit will be set to 0
     * |        |          |If switch target clock is not stable, this bit will be set to 1.
     * |        |          |0 = Clock switching success.
     * |        |          |1 = Clock switching failure.
     * |        |          |Note: Write 1 to clear the bit to 0.
     * @var CLK_T::CLKOCTL
     * Offset: 0x60  Clock Output Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FREQSEL   |Clock Output Frequency Selection
     * |        |          |The formula of output frequency is
     * |        |          |Fout = Fin/2(N+1).
     * |        |          |Fin is the input clock frequency.
     * |        |          |Fout is the frequency of divider output clock.
     * |        |          |N is the 4-bit value of FREQSEL[3:0].
     * |[4]     |CLKOEN    |Clock Output Enable Bit
     * |        |          |0 = Clock Output function Disabled.
     * |        |          |1 = Clock Output function Enabled.
     * |[5]     |DIV1EN    |Clock Output Divide One Enable Bit
     * |        |          |0 = Clock Output will output clock with source frequency divided by FREQSEL.
     * |        |          |1 = Clock Output will output clock with source frequency.
     */
    __IO uint32_t PWRCTL;                /*!< [0x0000] System Power-down Control Register                               */
    __IO uint32_t AHBCLK;                /*!< [0x0004] AHB Devices Clock Enable Control Register                        */
    __IO uint32_t APBCLK;                /*!< [0x0008] APB Devices Clock Enable Control Register                        */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t CLKSEL0;               /*!< [0x0010] Clock Source Select Control Register 0                           */
    __IO uint32_t CLKSEL1;               /*!< [0x0014] Clock Source Select Control Register 1                           */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t CLKDIV;                /*!< [0x0020] Clock Divider Number Register                                    */
    __I  uint32_t RESERVE2[11];
    __I  uint32_t STATUS;                /*!< [0x0050] Clock Status Monitor Register                                    */
    __I  uint32_t RESERVE3[3];
    __IO uint32_t CLKOCTL;               /*!< [0x0060] Clock Output Control Register                                    */

} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_PWRCTL_XTLEN_Pos             (0)                                               /*!< CLK_T::PWRCTL: XTLEN Position          */
#define CLK_PWRCTL_XTLEN_Msk             (0x3ul << CLK_PWRCTL_XTLEN_Pos)                   /*!< CLK_T::PWRCTL: XTLEN Mask              */

#define CLK_PWRCTL_HIRCEN_Pos            (2)                                               /*!< CLK_T::PWRCTL: HIRCEN Position         */
#define CLK_PWRCTL_HIRCEN_Msk            (0x1ul << CLK_PWRCTL_HIRCEN_Pos)                  /*!< CLK_T::PWRCTL: HIRCEN Mask             */

#define CLK_PWRCTL_LIRCEN_Pos            (3)                                               /*!< CLK_T::PWRCTL: LIRCEN Position         */
#define CLK_PWRCTL_LIRCEN_Msk            (0x1ul << CLK_PWRCTL_LIRCEN_Pos)                  /*!< CLK_T::PWRCTL: LIRCEN Mask             */

#define CLK_PWRCTL_PDWKDLY_Pos           (4)                                               /*!< CLK_T::PWRCTL: PDWKDLY Position        */
#define CLK_PWRCTL_PDWKDLY_Msk           (0x1ul << CLK_PWRCTL_PDWKDLY_Pos)                 /*!< CLK_T::PWRCTL: PDWKDLY Mask            */

#define CLK_PWRCTL_PDWKIEN_Pos           (5)                                               /*!< CLK_T::PWRCTL: PDWKIEN Position        */
#define CLK_PWRCTL_PDWKIEN_Msk           (0x1ul << CLK_PWRCTL_PDWKIEN_Pos)                 /*!< CLK_T::PWRCTL: PDWKIEN Mask            */

#define CLK_PWRCTL_PDWKIF_Pos            (6)                                               /*!< CLK_T::PWRCTL: PDWKIF Position         */
#define CLK_PWRCTL_PDWKIF_Msk            (0x1ul << CLK_PWRCTL_PDWKIF_Pos)                  /*!< CLK_T::PWRCTL: PDWKIF Mask             */

#define CLK_PWRCTL_PDEN_Pos              (7)                                               /*!< CLK_T::PWRCTL: PDEN Position           */
#define CLK_PWRCTL_PDEN_Msk              (0x1ul << CLK_PWRCTL_PDEN_Pos)                    /*!< CLK_T::PWRCTL: PDEN Mask               */

#define CLK_PWRCTL_PDLXT_Pos             (9)                                               /*!< CLK_T::PWRCTL: PDLXT Position          */
#define CLK_PWRCTL_PDLXT_Msk             (0x1ul << CLK_PWRCTL_PDLXT_Pos)                   /*!< CLK_T::PWRCTL: PDLXT Mask              */

#define CLK_PWRCTL_HXTGAIN_Pos           (10)                                              /*!< CLK_T::PWRCTL: HXTGAIN Position        */
#define CLK_PWRCTL_HXTGAIN_Msk           (0x3ul << CLK_PWRCTL_HXTGAIN_Pos)                 /*!< CLK_T::PWRCTL: HXTGAIN Mask            */

#define CLK_AHBCLK_ISPCKEN_Pos           (2)                                               /*!< CLK_T::AHBCLK: ISPCKEN Position        */
#define CLK_AHBCLK_ISPCKEN_Msk           (0x1ul << CLK_AHBCLK_ISPCKEN_Pos)                 /*!< CLK_T::AHBCLK: ISPCKEN Mask            */

#define CLK_AHBCLK_HDIVCKEN_Pos          (4)                                               /*!< CLK_T::AHBCLK: HDIVCKEN Position       */
#define CLK_AHBCLK_HDIVCKEN_Msk          (0x1ul << CLK_AHBCLK_HDIVCKEN_Pos)                /*!< CLK_T::AHBCLK: HDIVCKEN Mask           */

#define CLK_APBCLK_WDTCKEN_Pos           (0)                                               /*!< CLK_T::APBCLK: WDTCKEN Position        */
#define CLK_APBCLK_WDTCKEN_Msk           (0x1ul << CLK_APBCLK_WDTCKEN_Pos)                 /*!< CLK_T::APBCLK: WDTCKEN Mask            */

#define CLK_APBCLK_TMR0CKEN_Pos          (2)                                               /*!< CLK_T::APBCLK: TMR0CKEN Position       */
#define CLK_APBCLK_TMR0CKEN_Msk          (0x1ul << CLK_APBCLK_TMR0CKEN_Pos)                /*!< CLK_T::APBCLK: TMR0CKEN Mask           */

#define CLK_APBCLK_TMR1CKEN_Pos          (3)                                               /*!< CLK_T::APBCLK: TMR1CKEN Position       */
#define CLK_APBCLK_TMR1CKEN_Msk          (0x1ul << CLK_APBCLK_TMR1CKEN_Pos)                /*!< CLK_T::APBCLK: TMR1CKEN Mask           */

#define CLK_APBCLK_CLKOCKEN_Pos          (6)                                               /*!< CLK_T::APBCLK: CLKOCKEN Position       */
#define CLK_APBCLK_CLKOCKEN_Msk          (0x1ul << CLK_APBCLK_CLKOCKEN_Pos)                /*!< CLK_T::APBCLK: CLKOCKEN Mask           */

#define CLK_APBCLK_ECAPCKEN_Pos          (8)                                               /*!< CLK_T::APBCLK: ECAPCKEN Position       */
#define CLK_APBCLK_ECAPCKEN_Msk          (0x1ul << CLK_APBCLK_ECAPCKEN_Pos)                /*!< CLK_T::APBCLK: ECAPCKEN Mask           */

#define CLK_APBCLK_PGACKEN_Pos           (12)                                              /*!< CLK_T::APBCLK: PGACKEN Position        */
#define CLK_APBCLK_PGACKEN_Msk           (0x1ul << CLK_APBCLK_PGACKEN_Pos)                 /*!< CLK_T::APBCLK: PGACKEN Mask            */

#define CLK_APBCLK_EPWMCKEN_Pos          (20)                                              /*!< CLK_T::APBCLK: EPWMCKEN Position       */
#define CLK_APBCLK_EPWMCKEN_Msk          (0x1ul << CLK_APBCLK_EPWMCKEN_Pos)                /*!< CLK_T::APBCLK: EPWMCKEN Mask           */

#define CLK_APBCLK_BPWMCKEN_Pos          (23)                                              /*!< CLK_T::APBCLK: BPWMCKEN Position       */
#define CLK_APBCLK_BPWMCKEN_Msk          (0x1ul << CLK_APBCLK_BPWMCKEN_Pos)                /*!< CLK_T::APBCLK: BPWMCKEN Mask           */

#define CLK_APBCLK_USCI0CKEN_Pos         (24)                                              /*!< CLK_T::APBCLK: USCI0CKEN Position      */
#define CLK_APBCLK_USCI0CKEN_Msk         (0x1ul << CLK_APBCLK_USCI0CKEN_Pos)               /*!< CLK_T::APBCLK: USCI0CKEN Mask          */

#define CLK_APBCLK_USCI1CKEN_Pos         (25)                                              /*!< CLK_T::APBCLK: USCI1CKEN Position      */
#define CLK_APBCLK_USCI1CKEN_Msk         (0x1ul << CLK_APBCLK_USCI1CKEN_Pos)               /*!< CLK_T::APBCLK: USCI1CKEN Mask          */

#define CLK_APBCLK_ADCCKEN_Pos           (28)                                              /*!< CLK_T::APBCLK: ADCCKEN Position        */
#define CLK_APBCLK_ADCCKEN_Msk           (0x1ul << CLK_APBCLK_ADCCKEN_Pos)                 /*!< CLK_T::APBCLK: ADCCKEN Mask            */

#define CLK_APBCLK_ACMPCKEN_Pos          (30)                                              /*!< CLK_T::APBCLK: ACMPCKEN Position       */
#define CLK_APBCLK_ACMPCKEN_Msk          (0x1ul << CLK_APBCLK_ACMPCKEN_Pos)                /*!< CLK_T::APBCLK: ACMPCKEN Mask           */

#define CLK_CLKSEL0_HCLKSEL_Pos          (0)                                               /*!< CLK_T::CLKSEL0: HCLKSEL Position       */
#define CLK_CLKSEL0_HCLKSEL_Msk          (0x3ul << CLK_CLKSEL0_HCLKSEL_Pos)                /*!< CLK_T::CLKSEL0: HCLKSEL Mask           */

#define CLK_CLKSEL0_STCLKSEL_Pos         (3)                                               /*!< CLK_T::CLKSEL0: STCLKSEL Position      */
#define CLK_CLKSEL0_STCLKSEL_Msk         (0x3ul << CLK_CLKSEL0_STCLKSEL_Pos)               /*!< CLK_T::CLKSEL0: STCLKSEL Mask          */

#define CLK_CLKSEL1_WDTSEL_Pos           (0)                                               /*!< CLK_T::CLKSEL1: WDTSEL Position        */
#define CLK_CLKSEL1_WDTSEL_Msk           (0x3ul << CLK_CLKSEL1_WDTSEL_Pos)                 /*!< CLK_T::CLKSEL1: WDTSEL Mask            */

#define CLK_CLKSEL1_ADCSEL_Pos           (4)                                               /*!< CLK_T::CLKSEL1: ADCSEL Position        */
#define CLK_CLKSEL1_ADCSEL_Msk           (0x3ul << CLK_CLKSEL1_ADCSEL_Pos)                 /*!< CLK_T::CLKSEL1: ADCSEL Mask            */

#define CLK_CLKSEL1_TMR0SEL_Pos          (8)                                               /*!< CLK_T::CLKSEL1: TMR0SEL Position       */
#define CLK_CLKSEL1_TMR0SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR0SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR0SEL Mask           */

#define CLK_CLKSEL1_TMR1SEL_Pos          (12)                                              /*!< CLK_T::CLKSEL1: TMR1SEL Position       */
#define CLK_CLKSEL1_TMR1SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR1SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR1SEL Mask           */

#define CLK_CLKSEL1_CLKOSEL_Pos          (30)                                              /*!< CLK_T::CLKSEL1: CLKOSEL Position       */
#define CLK_CLKSEL1_CLKOSEL_Msk          (0x3ul << CLK_CLKSEL1_CLKOSEL_Pos)                /*!< CLK_T::CLKSEL1: CLKOSEL Mask           */

#define CLK_CLKDIV_HCLKDIV_Pos           (0)                                               /*!< CLK_T::CLKDIV: HCLKDIV Position        */
#define CLK_CLKDIV_HCLKDIV_Msk           (0xful << CLK_CLKDIV_HCLKDIV_Pos)                 /*!< CLK_T::CLKDIV: HCLKDIV Mask            */

#define CLK_CLKDIV_ADCDIV_Pos            (16)                                              /*!< CLK_T::CLKDIV: ADCDIV Position         */
#define CLK_CLKDIV_ADCDIV_Msk            (0xfful << CLK_CLKDIV_ADCDIV_Pos)                 /*!< CLK_T::CLKDIV: ADCDIV Mask             */

#define CLK_STATUS_XTLSTB_Pos            (0)                                               /*!< CLK_T::STATUS: XTLSTB Position         */
#define CLK_STATUS_XTLSTB_Msk            (0x1ul << CLK_STATUS_XTLSTB_Pos)                  /*!< CLK_T::STATUS: XTLSTB Mask             */

#define CLK_STATUS_LIRCSTB_Pos           (3)                                               /*!< CLK_T::STATUS: LIRCSTB Position        */
#define CLK_STATUS_LIRCSTB_Msk           (0x1ul << CLK_STATUS_LIRCSTB_Pos)                 /*!< CLK_T::STATUS: LIRCSTB Mask            */

#define CLK_STATUS_HIRCSTB_Pos           (4)                                               /*!< CLK_T::STATUS: HIRCSTB Position        */
#define CLK_STATUS_HIRCSTB_Msk           (0x1ul << CLK_STATUS_HIRCSTB_Pos)                 /*!< CLK_T::STATUS: HIRCSTB Mask            */

#define CLK_STATUS_CLKSFAIL_Pos          (7)                                               /*!< CLK_T::STATUS: CLKSFAIL Position       */
#define CLK_STATUS_CLKSFAIL_Msk          (0x1ul << CLK_STATUS_CLKSFAIL_Pos)                /*!< CLK_T::STATUS: CLKSFAIL Mask           */

#define CLK_CLKOCTL_FREQSEL_Pos          (0)                                               /*!< CLK_T::CLKOCTL: FREQSEL Position       */
#define CLK_CLKOCTL_FREQSEL_Msk          (0xful << CLK_CLKOCTL_FREQSEL_Pos)                /*!< CLK_T::CLKOCTL: FREQSEL Mask           */

#define CLK_CLKOCTL_CLKOEN_Pos           (4)                                               /*!< CLK_T::CLKOCTL: CLKOEN Position        */
#define CLK_CLKOCTL_CLKOEN_Msk           (0x1ul << CLK_CLKOCTL_CLKOEN_Pos)                 /*!< CLK_T::CLKOCTL: CLKOEN Mask            */

#define CLK_CLKOCTL_DIV1EN_Pos           (5)                                               /*!< CLK_T::CLKOCTL: DIV1EN Position        */
#define CLK_CLKOCTL_DIV1EN_Msk           (0x1ul << CLK_CLKOCTL_DIV1EN_Pos)                 /*!< CLK_T::CLKOCTL: DIV1EN Mask            */

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */


/*---------------------- General Purpose Input/Output Controller -------------------------*/
/**
    @addtogroup GPIO General Purpose Input/Output Controller(GPIO)
    Memory Mapped Structure for GPIO Controller
@{ */

typedef struct
{


    /**
     * @var GPIO_T::MODE
     * Offset: 0x00  Port I/O Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |MODE0     |Port A-D I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: The initial value of this field is defined by CIOINI (CONFIG0 [10])
     * |        |          |If CIOINI is set to 0, the default value is 0xFFFF_FFFF and all pins will be quasi-bidirectional mode after chip powered on
     * |        |          |If CIOINI is set to 1, the default value is 0x0000_0000 and all pins will be  input mode after chip powered on.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3:2]   |MODE1     |Port A-D I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: The initial value of this field is defined by CIOINI (CONFIG0 [10])
     * |        |          |If CIOINI is set to 0, the default value is 0xFFFF_FFFF and all pins will be quasi-bidirectional mode after chip powered on
     * |        |          |If CIOINI is set to 1, the default value is 0x0000_0000 and all pins will be  input mode after chip powered on.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5:4]   |MODE2     |Port A-D I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: The initial value of this field is defined by CIOINI (CONFIG0 [10])
     * |        |          |If CIOINI is set to 0, the default value is 0xFFFF_FFFF and all pins will be quasi-bidirectional mode after chip powered on
     * |        |          |If CIOINI is set to 1, the default value is 0x0000_0000 and all pins will be  input mode after chip powered on.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[7:6]   |MODE3     |Port A-D I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: The initial value of this field is defined by CIOINI (CONFIG0 [10])
     * |        |          |If CIOINI is set to 0, the default value is 0xFFFF_FFFF and all pins will be quasi-bidirectional mode after chip powered on
     * |        |          |If CIOINI is set to 1, the default value is 0x0000_0000 and all pins will be  input mode after chip powered on.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[9:8]   |MODE4     |Port A-D I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: The initial value of this field is defined by CIOINI (CONFIG0 [10])
     * |        |          |If CIOINI is set to 0, the default value is 0xFFFF_FFFF and all pins will be quasi-bidirectional mode after chip powered on
     * |        |          |If CIOINI is set to 1, the default value is 0x0000_0000 and all pins will be  input mode after chip powered on.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[11:10] |MODE5     |Port A-D I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: The initial value of this field is defined by CIOINI (CONFIG0 [10])
     * |        |          |If CIOINI is set to 0, the default value is 0xFFFF_FFFF and all pins will be quasi-bidirectional mode after chip powered on
     * |        |          |If CIOINI is set to 1, the default value is 0x0000_0000 and all pins will be  input mode after chip powered on.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[13:12] |MODE6     |Port A-D I/O Pin[N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pins.
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: The initial value of this field is defined by CIOINI (CONFIG0 [10])
     * |        |          |If CIOINI is set to 0, the default value is 0xFFFF_FFFF and all pins will be quasi-bidirectional mode after chip powered on
     * |        |          |If CIOINI is set to 1, the default value is 0x0000_0000 and all pins will be  input mode after chip powered on.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::DINOFF
     * Offset: 0x04  Port Digital Input Path Disable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16]    |DINOFF0   |Port A-D Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[17]    |DINOFF1   |Port A-D Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[18]    |DINOFF2   |Port A-D Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[19]    |DINOFF3   |Port A-D Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[20]    |DINOFF4   |Port A-D Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[21]    |DINOFF5   |Port A-D Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[22]    |DINOFF6   |Port A-D Pin[N] Digital Input Path Disable Control
     * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled
     * |        |          |If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::DOUT
     * Offset: 0x08  Port Data Output Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DOUT0     |Port A-D Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[1]     |DOUT1     |Port A-D Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[2]     |DOUT2     |Port A-D Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3]     |DOUT3     |Port A-D Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[4]     |DOUT4     |Port A-D Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5]     |DOUT5     |Port A-D Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[6]     |DOUT6     |Port A-D Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::DATMSK
     * Offset: 0x0C  Port Data Output Write Mask
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DATMSK0   |Port A-D Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected
     * |        |          |If the write signal is masked, writing data to the protect bit is ignored.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[1]     |DATMSK1   |Port A-D Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected
     * |        |          |If the write signal is masked, writing data to the protect bit is ignored.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[2]     |DATMSK2   |Port A-D Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected
     * |        |          |If the write signal is masked, writing data to the protect bit is ignored.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3]     |DATMSK3   |Port A-D Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected
     * |        |          |If the write signal is masked, writing data to the protect bit is ignored.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[4]     |DATMSK4   |Port A-D Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected
     * |        |          |If the write signal is masked, writing data to the protect bit is ignored.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5]     |DATMSK5   |Port A-D Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected
     * |        |          |If the write signal is masked, writing data to the protect bit is ignored.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[6]     |DATMSK6   |Port A-D Pin[N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding DOUT (Px_DOUT[n]) bit
     * |        |          |When the DATMSK (Px_DATMSK[n]) bit is set to 1, the corresponding DOUT (Px_DOUT[n]) bit is protected
     * |        |          |If the write signal is masked, writing data to the protect bit is ignored.
     * |        |          |0 = Corresponding DOUT (Px_DOUT[n]) bit can be updated.
     * |        |          |1 = Corresponding DOUT (Px_DOUT[n]) bit protected.
     * |        |          |Note1: This function only protects the corresponding DOUT (Px_DOUT[n]) bit, and will not protect the corresponding PDIO (Pxn_PDIO[0]) bit.
     * |        |          |Note2:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::PIN
     * Offset: 0x10  Port Pin Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIN0      |Port A-D Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[1]     |PIN1      |Port A-D Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[2]     |PIN2      |Port A-D Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3]     |PIN3      |Port A-D Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[4]     |PIN4      |Port A-D Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5]     |PIN5      |Port A-D Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[6]     |PIN6      |Port A-D Pin[N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::DBEN
     * Offset: 0x14  Port De-Bounce Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DBEN0     |Port A-D Pin[N] Input Signal De-Bounce Enable Bit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[1]     |DBEN1     |Port A-D Pin[N] Input Signal De-Bounce Enable Biit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[2]     |DBEN2     |Port A-D Pin[N] Input Signal De-Bounce Enable Biit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3]     |DBEN3     |Port A-D Pin[N] Input Signal De-Bounce Enable Biit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[4]     |DBEN4     |Port A-D Pin[N] Input Signal De-Bounce Enable Biit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5]     |DBEN5     |Port A-D Pin[N] Input Signal De-Bounce Enable Biit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[6]     |DBEN6     |Port A-D Pin[N] Input Signal De-Bounce Enable Biit
     * |        |          |The DBEN[n] bit is used to enable the de-bounce function for each corresponding bit
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt
     * |        |          |The de-bounce clock source is controlled by DBCLKSRC (GPIO_DBCTL [4]), one de-bounce sample cycle period is controlled by DBCLKSEL (GPIO_DBCTL [3:0]).
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::INTTYPE
     * Offset: 0x18  Port Interrupt Trigger Type Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TYPE0     |Port A-D Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n])
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[1]     |TYPE1     |Port A-D Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n])
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[2]     |TYPE2     |Port A-D Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n])
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3]     |TYPE3     |Port A-D Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n])
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[4]     |TYPE4     |Port A-D Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n])
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5]     |TYPE5     |Port A-D Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n])
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[6]     |TYPE6     |Port A-D Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE (Px_INTTYPE[n]) bit is used to control the triggered interrupt is by level trigger or by edge trigger
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers RHIEN (Px_INTEN[n+16])/FLIEN (Px_INTEN[n])
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::INTEN
     * Offset: 0x1C  Port Interrupt Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FLIEN0    |Port A-D Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Biit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger(TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[1]     |FLIEN1    |Port A-D Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Biit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger(TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[2]     |FLIEN2    |Port A-D Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Biit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger(TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3]     |FLIEN3    |Port A-D Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Biit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger(TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[4]     |FLIEN4    |Port A-D Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Biit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger(TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5]     |FLIEN5    |Port A-D Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Biit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger(TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[6]     |FLIEN6    |Port A-D Pin[N] Falling Edge Or Low Level Interrupt Trigger Type Enable Biit
     * |        |          |The FLIEN (Px_INTEN[n]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the FLIEN (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge trigger(TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n level low or high to low interrupt Disabled.
     * |        |          |1 = Px.n level low or high to low interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[16]    |RHIEN0    |Port A-D Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[17]    |RHIEN1    |Port A-D Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[18]    |RHIEN2    |Port A-D Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[19]    |RHIEN3    |Port A-D Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[20]    |RHIEN4    |Port A-D Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[21]    |RHIEN5    |Port A-D Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[22]    |RHIEN6    |Port A-D Pin[N] Rising Edge Or High Level Interrupt Trigger Type Enable Bit
     * |        |          |The RHIEN (Px_INTEN[n+16]) bit is used to enable the interrupt for each of the corresponding input Px.n pin
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the RHIEN (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is level trigger (TYPE (Px_INTTYPE[n]) bit is set to 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (TYPE (Px_INTTYPE[n]) bit is set to 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::INTSRC
     * Offset: 0x20  Port Interrupt Source Flag
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTSRC0   |Port A-D Pin[N] Interrupt Source Flag
     * |        |          |Write Operation :
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation :
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[1]     |INTSRC1   |Port A-D Pin[N] Interrupt Source Flag
     * |        |          |Write Operation :
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation :
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[2]     |INTSRC2   |Port A-D Pin[N] Interrupt Source Flag
     * |        |          |Write Operation :
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation :
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3]     |INTSRC3   |Port A-D Pin[N] Interrupt Source Flag
     * |        |          |Write Operation :
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation :
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[4]     |INTSRC4   |Port A-D Pin[N] Interrupt Source Flag
     * |        |          |Write Operation :
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation :
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5]     |INTSRC5   |Port A-D Pin[N] Interrupt Source Flag
     * |        |          |Write Operation :
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation :
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[6]     |INTSRC6   |Port A-D Pin[N] Interrupt Source Flag
     * |        |          |Write Operation :
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read Operation :
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::SMTEN
     * Offset: 0x24  Port Input Schmitt Trigger Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SMTEN0    |Port A-D Pin[N] Input Schmitt Trigger Enable Bit
     * |        |          |0 = Px.n input schmitt trigger function Disabled.
     * |        |          |1 = Px.n input schmitt trigger function Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[1]     |SMTEN1    |Port A-D Pin[N] Input Schmitt Trigger Enable Bit
     * |        |          |0 = Px.n input schmitt trigger function Disabled.
     * |        |          |1 = Px.n input schmitt trigger function Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[2]     |SMTEN2    |Port A-D Pin[N] Input Schmitt Trigger Enable Bit
     * |        |          |0 = Px.n input schmitt trigger function Disabled.
     * |        |          |1 = Px.n input schmitt trigger function Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3]     |SMTEN3    |Port A-D Pin[N] Input Schmitt Trigger Enable Bit
     * |        |          |0 = Px.n input schmitt trigger function Disabled.
     * |        |          |1 = Px.n input schmitt trigger function Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[4]     |SMTEN4    |Port A-D Pin[N] Input Schmitt Trigger Enable Bit
     * |        |          |0 = Px.n input schmitt trigger function Disabled.
     * |        |          |1 = Px.n input schmitt trigger function Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5]     |SMTEN5    |Port A-D Pin[N] Input Schmitt Trigger Enable Bit
     * |        |          |0 = Px.n input schmitt trigger function Disabled.
     * |        |          |1 = Px.n input schmitt trigger function Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[6]     |SMTEN6    |Port A-D Pin[N] Input Schmitt Trigger Enable Bit
     * |        |          |0 = Px.n input schmitt trigger function Disabled.
     * |        |          |1 = Px.n input schmitt trigger function Enabled.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::SLEWCTL
     * Offset: 0x28  Port High Slew Rate Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HSREN0    |Port A-D Pin[n] High Slew Rate Control
     * |        |          |0 = Px.n output with basic slew rate.
     * |        |          |1 = Px.n output with higher slew rate.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[1]     |HSREN1    |Port A-D Pin[n] High Slew Rate Control
     * |        |          |0 = Px.n output with basic slew rate.
     * |        |          |1 = Px.n output with higher slew rate.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[2]     |HSREN2    |Port A-D Pin[n] High Slew Rate Control
     * |        |          |0 = Px.n output with basic slew rate.
     * |        |          |1 = Px.n output with higher slew rate.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3]     |HSREN3    |Port A-D Pin[n] High Slew Rate Control
     * |        |          |0 = Px.n output with basic slew rate.
     * |        |          |1 = Px.n output with higher slew rate.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[4]     |HSREN4    |Port A-D Pin[n] High Slew Rate Control
     * |        |          |0 = Px.n output with basic slew rate.
     * |        |          |1 = Px.n output with higher slew rate.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5]     |HSREN5    |Port A-D Pin[n] High Slew Rate Control
     * |        |          |0 = Px.n output with basic slew rate.
     * |        |          |1 = Px.n output with higher slew rate.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[6]     |HSREN6    |Port A-D Pin[n] High Slew Rate Control
     * |        |          |0 = Px.n output with basic slew rate.
     * |        |          |1 = Px.n output with higher slew rate.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::PLEN
     * Offset: 0x2C  Port Pull-Low Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PLEN0     |Port A-D Pull-Low Resistor Control
     * |        |          |0 = Pull-Low Resistor Disable.
     * |        |          |1 = Pull-Low Resistor Enable.
     * |        |          |Note: The initial value of PA_PLEN were defined by GPAn_RINI (CONFIG0[27:16]).
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[1]     |PLEN1     |Port A-D Pull-Low Resistor Control
     * |        |          |0 = Pull-Low Resistor Disable.
     * |        |          |1 = Pull-Low Resistor Enable.
     * |        |          |Note: The initial value of PA_PLEN were defined by GPAn_RINI (CONFIG0[27:16]).
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[2]     |PLEN2     |Port A-D Pull-Low Resistor Control
     * |        |          |0 = Pull-Low Resistor Disable.
     * |        |          |1 = Pull-Low Resistor Enable.
     * |        |          |Note: The initial value of PA_PLEN were defined by GPAn_RINI (CONFIG0[27:16]).
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3]     |PLEN3     |Port A-D Pull-Low Resistor Control
     * |        |          |0 = Pull-Low Resistor Disable.
     * |        |          |1 = Pull-Low Resistor Enable.
     * |        |          |Note: The initial value of PA_PLEN were defined by GPAn_RINI (CONFIG0[27:16]).
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[4]     |PLEN4     |Port A-D Pull-Low Resistor Control
     * |        |          |0 = Pull-Low Resistor Disable.
     * |        |          |1 = Pull-Low Resistor Enable.
     * |        |          |Note: The initial value of PA_PLEN were defined by GPAn_RINI (CONFIG0[27:16]).
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5]     |PLEN5     |Port A-D Pull-Low Resistor Control
     * |        |          |0 = Pull-Low Resistor Disable.
     * |        |          |1 = Pull-Low Resistor Enable.
     * |        |          |Note: The initial value of PA_PLEN were defined by GPAn_RINI (CONFIG0[27:16]).
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[6]     |PLEN6     |Port A-D Pull-Low Resistor Control
     * |        |          |0 = Pull-Low Resistor Disable.
     * |        |          |1 = Pull-Low Resistor Enable.
     * |        |          |Note: The initial value of PA_PLEN were defined by GPAn_RINI (CONFIG0[27:16]).
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * @var GPIO_T::PHEN
     * Offset: 0x30  Port Pull-High Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PHEN0     |Port A Pull-High Resistor Control
     * |        |          |0 = Pull-Low Resistor Enable.
     * |        |          |1 = Pull-Low Resistor Disable.
     * |        |          |Note: The initial value of PA_PHEN were defined by GPAn_RINI (CONFIG0[27:16])
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[1]     |PHEN1     |Port A Pull-High Resistor Control
     * |        |          |0 = Pull-Low Resistor Enable.
     * |        |          |1 = Pull-Low Resistor Disable.
     * |        |          |Note: The initial value of PA_PHEN were defined by GPAn_RINI (CONFIG0[27:16])
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[2]     |PHEN2     |Port A Pull-High Resistor Control
     * |        |          |0 = Pull-Low Resistor Enable.
     * |        |          |1 = Pull-Low Resistor Disable.
     * |        |          |Note: The initial value of PA_PHEN were defined by GPAn_RINI (CONFIG0[27:16])
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[3]     |PHEN3     |Port A Pull-High Resistor Control
     * |        |          |0 = Pull-Low Resistor Enable.
     * |        |          |1 = Pull-Low Resistor Disable.
     * |        |          |Note: The initial value of PA_PHEN were defined by GPAn_RINI (CONFIG0[27:16])
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[4]     |PHEN4     |Port A Pull-High Resistor Control
     * |        |          |0 = Pull-Low Resistor Enable.
     * |        |          |1 = Pull-Low Resistor Disable.
     * |        |          |Note: The initial value of PA_PHEN were defined by GPAn_RINI (CONFIG0[27:16])
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[5]     |PHEN5     |Port A Pull-High Resistor Control
     * |        |          |0 = Pull-Low Resistor Enable.
     * |        |          |1 = Pull-Low Resistor Disable.
     * |        |          |Note: The initial value of PA_PHEN were defined by GPAn_RINI (CONFIG0[27:16])
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     * |[6]     |PHEN6     |Port A Pull-High Resistor Control
     * |        |          |0 = Pull-Low Resistor Enable.
     * |        |          |1 = Pull-Low Resistor Disable.
     * |        |          |Note: The initial value of PA_PHEN were defined by GPAn_RINI (CONFIG0[27:16])
     * |        |          |Selected pins will be configured after chip powered on.
     * |        |          |Note:
     * |        |          |Max. n=5 for port A.
     * |        |          |Max. n=4 for port B.
     * |        |          |Max. n=4 for port C.
     * |        |          |Max. n=6 for port D. n=0 is reserved.
     */
    __IO uint32_t MODE;               /*!< Offset [0x0000] GPIO Port[A/B/C/D] I/O Mode Control                                         */
    __IO uint32_t DINOFF;             /*!< Offset [0x0004] GPIO Port[A/B/C/D] Digital Input Path Disable Control                       */
    __IO uint32_t DOUT;               /*!< Offset [0x0008] GPIO Port[A/B/C/D] Data Output Value                                        */
    __IO uint32_t DATMSK;             /*!< Offset [0x000c] GPIO Port[A/B/C/D] Data Output Write Mask                                   */
    __I  uint32_t PIN;                /*!< Offset [0x0010] GPIO Port[A/B/C/D] Pin Value                                                */
    __IO uint32_t DBEN;               /*!< Offset [0x0014] GPIO Port[A/B/C/D] De-Bounce Enable Control Register                        */
    __IO uint32_t INTTYPE;            /*!< Offset [0x0018] GPIO Port[A/B/C/D] Interrupt Trigger Type Control                           */
    __IO uint32_t INTEN;              /*!< Offset [0x001c] GPIO Port[A/B/C/D] Interrupt Enable Control Register                        */
    __IO uint32_t INTSRC;             /*!< Offset [0x0020] GPIO Port[A/B/C/D] Interrupt Source Flag                                    */
    __IO uint32_t SMTEN;              /*!< Offset [0x0024] GPIO Port[A/B/C/D] Input Schmitt Trigger Enable Register                    */
    __IO uint32_t SLEWCTL;            /*!< Offset [0x0028] GPIO Port[A/B/C/D] High Slew Rate Control Register                          */
    __IO uint32_t PLEN;               /*!< Offset [0x002c] GPIO Port[A/B/C/D] Pull-Low Control Register                                */
    __IO uint32_t PHEN;               /*!< Offset [0x0030] GPIO Port[A/B/C/D] Pull-High Control Register                               */
} GPIO_T;


typedef struct
{

    /**
     * @var GPIO_DB_T::GPIO_DBCTL
     * Offset: 0x440  Interrupt De-bounce Control Register
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
     * |        |          |1010 = Sample interrupt input once per 4*256 clocks.
     * |        |          |1011 = Sample interrupt input once per 8*256 clocks.
     * |        |          |1100 = Sample interrupt input once per 16*256 clocks.
     * |        |          |1101 = Sample interrupt input once per 32*256 clocks.
     * |        |          |1110 = Sample interrupt input once per 64*256 clocks.
     * |        |          |1111 = Sample interrupt input once per 128*256 clocks.
     * |[4]     |DBCLKSRC  |De-Bounce Counter Clock Source Selection
     * |        |          |0 = De-bounce counter clock source is the HCLK.
     * |        |          |1 = De-bounce counter clock source is the 10 kHz internal low speed RC oscillator (LIRC).
     * |[5]     |ICLKON    |Interrupt Clock On Mode
     * |        |          |0 = Edge detection circuit is active only if I/O pin corresponding RHIEN
     * |        |          |(Px_INTEN[n+16])/FLIEN (Px_INTEN[n]) bit is set to 1.
     * |        |          |1 = All I/O pins edge detection circuit is always active after reset.
     * |        |          |Note: It is recommended to disable this bit to save system power if no special application
     * |        |          |concern.
     */
    __IO uint32_t GPIO_DBCTL;
} GPIO_DB_T;


/**
    @addtogroup GPIO_CONST GPIO Bit Field Definition
    Constant Definitions for GPIO Controller
@{ */

#define GPIO_MODE_MODE0_Pos              (0)                                               /*!< GPIO_T::MODE: MODE0 Position           */
#define GPIO_MODE_MODE0_Msk              (0x3ul << GPIO_MODE_MODE0_Pos)                    /*!< GPIO_T::MODE: MODE0 Mask               */

#define GPIO_MODE_MODE1_Pos              (2)                                               /*!< GPIO_T::MODE: MODE1 Position           */
#define GPIO_MODE_MODE1_Msk              (0x3ul << GPIO_MODE_MODE1_Pos)                    /*!< GPIO_T::MODE: MODE1 Mask               */

#define GPIO_MODE_MODE2_Pos              (4)                                               /*!< GPIO_T::MODE: MODE2 Position           */
#define GPIO_MODE_MODE2_Msk              (0x3ul << GPIO_MODE_MODE2_Pos)                    /*!< GPIO_T::MODE: MODE2 Mask               */

#define GPIO_MODE_MODE3_Pos              (6)                                               /*!< GPIO_T::MODE: MODE3 Position           */
#define GPIO_MODE_MODE3_Msk              (0x3ul << GPIO_MODE_MODE3_Pos)                    /*!< GPIO_T::MODE: MODE3 Mask               */

#define GPIO_MODE_MODE4_Pos              (8)                                               /*!< GPIO_T::MODE: MODE4 Position           */
#define GPIO_MODE_MODE4_Msk              (0x3ul << GPIO_MODE_MODE4_Pos)                    /*!< GPIO_T::MODE: MODE4 Mask               */

#define GPIO_MODE_MODE5_Pos              (10)                                              /*!< GPIO_T::MODE: MODE5 Position           */
#define GPIO_MODE_MODE5_Msk              (0x3ul << GPIO_MODE_MODE5_Pos)                    /*!< GPIO_T::MODE: MODE5 Mask               */

#define GPIO_MODE_MODE6_Pos              (12)                                              /*!< GPIO_T::MODE: MODE6 Position           */
#define GPIO_MODE_MODE6_Msk              (0x3ul << GPIO_MODE_MODE6_Pos)                    /*!< GPIO_T::MODE: MODE6 Mask               */

#define GPIO_DINOFF_DINOFF0_Pos          (16)                                              /*!< GPIO_T::DINOFF: DINOFF0 Position       */
#define GPIO_DINOFF_DINOFF0_Msk          (0x1ul << GPIO_DINOFF_DINOFF0_Pos)                /*!< GPIO_T::DINOFF: DINOFF0 Mask           */

#define GPIO_DINOFF_DINOFF1_Pos          (17)                                              /*!< GPIO_T::DINOFF: DINOFF1 Position       */
#define GPIO_DINOFF_DINOFF1_Msk          (0x1ul << GPIO_DINOFF_DINOFF1_Pos)                /*!< GPIO_T::DINOFF: DINOFF1 Mask           */

#define GPIO_DINOFF_DINOFF2_Pos          (18)                                              /*!< GPIO_T::DINOFF: DINOFF2 Position       */
#define GPIO_DINOFF_DINOFF2_Msk          (0x1ul << GPIO_DINOFF_DINOFF2_Pos)                /*!< GPIO_T::DINOFF: DINOFF2 Mask           */

#define GPIO_DINOFF_DINOFF3_Pos          (19)                                              /*!< GPIO_T::DINOFF: DINOFF3 Position       */
#define GPIO_DINOFF_DINOFF3_Msk          (0x1ul << GPIO_DINOFF_DINOFF3_Pos)                /*!< GPIO_T::DINOFF: DINOFF3 Mask           */

#define GPIO_DINOFF_DINOFF4_Pos          (20)                                              /*!< GPIO_T::DINOFF: DINOFF4 Position       */
#define GPIO_DINOFF_DINOFF4_Msk          (0x1ul << GPIO_DINOFF_DINOFF4_Pos)                /*!< GPIO_T::DINOFF: DINOFF4 Mask           */

#define GPIO_DINOFF_DINOFF5_Pos          (21)                                              /*!< GPIO_T::DINOFF: DINOFF5 Position       */
#define GPIO_DINOFF_DINOFF5_Msk          (0x1ul << GPIO_DINOFF_DINOFF5_Pos)                /*!< GPIO_T::DINOFF: DINOFF5 Mask           */

#define GPIO_DINOFF_DINOFF6_Pos          (22)                                              /*!< GPIO_T::DINOFF: DINOFF6 Position       */
#define GPIO_DINOFF_DINOFF6_Msk          (0x1ul << GPIO_DINOFF_DINOFF6_Pos)                /*!< GPIO_T::DINOFF: DINOFF6 Mask           */

#define GPIO_DOUT_DOUT0_Pos              (0)                                               /*!< GPIO_T::DOUT: DOUT0 Position           */
#define GPIO_DOUT_DOUT0_Msk              (0x1ul << GPIO_DOUT_DOUT0_Pos)                    /*!< GPIO_T::DOUT: DOUT0 Mask               */

#define GPIO_DOUT_DOUT1_Pos              (1)                                               /*!< GPIO_T::DOUT: DOUT1 Position           */
#define GPIO_DOUT_DOUT1_Msk              (0x1ul << GPIO_DOUT_DOUT1_Pos)                    /*!< GPIO_T::DOUT: DOUT1 Mask               */

#define GPIO_DOUT_DOUT2_Pos              (2)                                               /*!< GPIO_T::DOUT: DOUT2 Position           */
#define GPIO_DOUT_DOUT2_Msk              (0x1ul << GPIO_DOUT_DOUT2_Pos)                    /*!< GPIO_T::DOUT: DOUT2 Mask               */

#define GPIO_DOUT_DOUT3_Pos              (3)                                               /*!< GPIO_T::DOUT: DOUT3 Position           */
#define GPIO_DOUT_DOUT3_Msk              (0x1ul << GPIO_DOUT_DOUT3_Pos)                    /*!< GPIO_T::DOUT: DOUT3 Mask               */

#define GPIO_DOUT_DOUT4_Pos              (4)                                               /*!< GPIO_T::DOUT: DOUT4 Position           */
#define GPIO_DOUT_DOUT4_Msk              (0x1ul << GPIO_DOUT_DOUT4_Pos)                    /*!< GPIO_T::DOUT: DOUT4 Mask               */

#define GPIO_DOUT_DOUT5_Pos              (5)                                               /*!< GPIO_T::DOUT: DOUT5 Position           */
#define GPIO_DOUT_DOUT5_Msk              (0x1ul << GPIO_DOUT_DOUT5_Pos)                    /*!< GPIO_T::DOUT: DOUT5 Mask               */

#define GPIO_DOUT_DOUT6_Pos              (6)                                               /*!< GPIO_T::DOUT: DOUT6 Position           */
#define GPIO_DOUT_DOUT6_Msk              (0x1ul << GPIO_DOUT_DOUT6_Pos)                    /*!< GPIO_T::DOUT: DOUT6 Mask               */

#define GPIO_DATMSK_DATMSK0_Pos          (0)                                               /*!< GPIO_T::DATMSK: DATMSK0 Position       */
#define GPIO_DATMSK_DATMSK0_Msk          (0x1ul << GPIO_DATMSK_DATMSK0_Pos)                /*!< GPIO_T::DATMSK: DATMSK0 Mask           */

#define GPIO_DATMSK_DATMSK1_Pos          (1)                                               /*!< GPIO_T::DATMSK: DATMSK1 Position       */
#define GPIO_DATMSK_DATMSK1_Msk          (0x1ul << GPIO_DATMSK_DATMSK1_Pos)                /*!< GPIO_T::DATMSK: DATMSK1 Mask           */

#define GPIO_DATMSK_DATMSK2_Pos          (2)                                               /*!< GPIO_T::DATMSK: DATMSK2 Position       */
#define GPIO_DATMSK_DATMSK2_Msk          (0x1ul << GPIO_DATMSK_DATMSK2_Pos)                /*!< GPIO_T::DATMSK: DATMSK2 Mask           */

#define GPIO_DATMSK_DATMSK3_Pos          (3)                                               /*!< GPIO_T::DATMSK: DATMSK3 Position       */
#define GPIO_DATMSK_DATMSK3_Msk          (0x1ul << GPIO_DATMSK_DATMSK3_Pos)                /*!< GPIO_T::DATMSK: DATMSK3 Mask           */

#define GPIO_DATMSK_DATMSK4_Pos          (4)                                               /*!< GPIO_T::DATMSK: DATMSK4 Position       */
#define GPIO_DATMSK_DATMSK4_Msk          (0x1ul << GPIO_DATMSK_DATMSK4_Pos)                /*!< GPIO_T::DATMSK: DATMSK4 Mask           */

#define GPIO_DATMSK_DATMSK5_Pos          (5)                                               /*!< GPIO_T::DATMSK: DATMSK5 Position       */
#define GPIO_DATMSK_DATMSK5_Msk          (0x1ul << GPIO_DATMSK_DATMSK5_Pos)                /*!< GPIO_T::DATMSK: DATMSK5 Mask           */

#define GPIO_DATMSK_DATMSK6_Pos          (6)                                               /*!< GPIO_T::DATMSK: DATMSK6 Position       */
#define GPIO_DATMSK_DATMSK6_Msk          (0x1ul << GPIO_DATMSK_DATMSK6_Pos)                /*!< GPIO_T::DATMSK: DATMSK6 Mask           */

#define GPIO_PIN_PIN0_Pos                (0)                                               /*!< GPIO_T::PIN: PIN0 Position             */
#define GPIO_PIN_PIN0_Msk                (0x1ul << GPIO_PIN_PIN0_Pos)                      /*!< GPIO_T::PIN: PIN0 Mask                 */

#define GPIO_PIN_PIN1_Pos                (1)                                               /*!< GPIO_T::PIN: PIN1 Position             */
#define GPIO_PIN_PIN1_Msk                (0x1ul << GPIO_PIN_PIN1_Pos)                      /*!< GPIO_T::PIN: PIN1 Mask                 */

#define GPIO_PIN_PIN2_Pos                (2)                                               /*!< GPIO_T::PIN: PIN2 Position             */
#define GPIO_PIN_PIN2_Msk                (0x1ul << GPIO_PIN_PIN2_Pos)                      /*!< GPIO_T::PIN: PIN2 Mask                 */

#define GPIO_PIN_PIN3_Pos                (3)                                               /*!< GPIO_T::PIN: PIN3 Position             */
#define GPIO_PIN_PIN3_Msk                (0x1ul << GPIO_PIN_PIN3_Pos)                      /*!< GPIO_T::PIN: PIN3 Mask                 */

#define GPIO_PIN_PIN4_Pos                (4)                                               /*!< GPIO_T::PIN: PIN4 Position             */
#define GPIO_PIN_PIN4_Msk                (0x1ul << GPIO_PIN_PIN4_Pos)                      /*!< GPIO_T::PIN: PIN4 Mask                 */

#define GPIO_PIN_PIN5_Pos                (5)                                               /*!< GPIO_T::PIN: PIN5 Position             */
#define GPIO_PIN_PIN5_Msk                (0x1ul << GPIO_PIN_PIN5_Pos)                      /*!< GPIO_T::PIN: PIN5 Mask                 */

#define GPIO_PIN_PIN6_Pos                (6)                                               /*!< GPIO_T::PIN: PIN6 Position             */
#define GPIO_PIN_PIN6_Msk                (0x1ul << GPIO_PIN_PIN6_Pos)                      /*!< GPIO_T::PIN: PIN6 Mask                 */

#define GPIO_DBEN_DBEN0_Pos              (0)                                               /*!< GPIO_T::DBEN: DBEN0 Position           */
#define GPIO_DBEN_DBEN0_Msk              (0x1ul << GPIO_DBEN_DBEN0_Pos)                    /*!< GPIO_T::DBEN: DBEN0 Mask               */

#define GPIO_DBEN_DBEN1_Pos              (1)                                               /*!< GPIO_T::DBEN: DBEN1 Position           */
#define GPIO_DBEN_DBEN1_Msk              (0x1ul << GPIO_DBEN_DBEN1_Pos)                    /*!< GPIO_T::DBEN: DBEN1 Mask               */

#define GPIO_DBEN_DBEN2_Pos              (2)                                               /*!< GPIO_T::DBEN: DBEN2 Position           */
#define GPIO_DBEN_DBEN2_Msk              (0x1ul << GPIO_DBEN_DBEN2_Pos)                    /*!< GPIO_T::DBEN: DBEN2 Mask               */

#define GPIO_DBEN_DBEN3_Pos              (3)                                               /*!< GPIO_T::DBEN: DBEN3 Position           */
#define GPIO_DBEN_DBEN3_Msk              (0x1ul << GPIO_DBEN_DBEN3_Pos)                    /*!< GPIO_T::DBEN: DBEN3 Mask               */

#define GPIO_DBEN_DBEN4_Pos              (4)                                               /*!< GPIO_T::DBEN: DBEN4 Position           */
#define GPIO_DBEN_DBEN4_Msk              (0x1ul << GPIO_DBEN_DBEN4_Pos)                    /*!< GPIO_T::DBEN: DBEN4 Mask               */

#define GPIO_DBEN_DBEN5_Pos              (5)                                               /*!< GPIO_T::DBEN: DBEN5 Position           */
#define GPIO_DBEN_DBEN5_Msk              (0x1ul << GPIO_DBEN_DBEN5_Pos)                    /*!< GPIO_T::DBEN: DBEN5 Mask               */

#define GPIO_DBEN_DBEN6_Pos              (6)                                               /*!< GPIO_T::DBEN: DBEN6 Position           */
#define GPIO_DBEN_DBEN6_Msk              (0x1ul << GPIO_DBEN_DBEN6_Pos)                    /*!< GPIO_T::DBEN: DBEN6 Mask               */

#define GPIO_INTTYPE_TYPE0_Pos           (0)                                               /*!< GPIO_T::INTTYPE: TYPE0 Position        */
#define GPIO_INTTYPE_TYPE0_Msk           (0x1ul << GPIO_INTTYPE_TYPE0_Pos)                 /*!< GPIO_T::INTTYPE: TYPE0 Mask            */

#define GPIO_INTTYPE_TYPE1_Pos           (1)                                               /*!< GPIO_T::INTTYPE: TYPE1 Position        */
#define GPIO_INTTYPE_TYPE1_Msk           (0x1ul << GPIO_INTTYPE_TYPE1_Pos)                 /*!< GPIO_T::INTTYPE: TYPE1 Mask            */

#define GPIO_INTTYPE_TYPE2_Pos           (2)                                               /*!< GPIO_T::INTTYPE: TYPE2 Position        */
#define GPIO_INTTYPE_TYPE2_Msk           (0x1ul << GPIO_INTTYPE_TYPE2_Pos)                 /*!< GPIO_T::INTTYPE: TYPE2 Mask            */

#define GPIO_INTTYPE_TYPE3_Pos           (3)                                               /*!< GPIO_T::INTTYPE: TYPE3 Position        */
#define GPIO_INTTYPE_TYPE3_Msk           (0x1ul << GPIO_INTTYPE_TYPE3_Pos)                 /*!< GPIO_T::INTTYPE: TYPE3 Mask            */

#define GPIO_INTTYPE_TYPE4_Pos           (4)                                               /*!< GPIO_T::INTTYPE: TYPE4 Position        */
#define GPIO_INTTYPE_TYPE4_Msk           (0x1ul << GPIO_INTTYPE_TYPE4_Pos)                 /*!< GPIO_T::INTTYPE: TYPE4 Mask            */

#define GPIO_INTTYPE_TYPE5_Pos           (5)                                               /*!< GPIO_T::INTTYPE: TYPE5 Position        */
#define GPIO_INTTYPE_TYPE5_Msk           (0x1ul << GPIO_INTTYPE_TYPE5_Pos)                 /*!< GPIO_T::INTTYPE: TYPE5 Mask            */

#define GPIO_INTTYPE_TYPE6_Pos           (6)                                               /*!< GPIO_T::INTTYPE: TYPE6 Position        */
#define GPIO_INTTYPE_TYPE6_Msk           (0x1ul << GPIO_INTTYPE_TYPE6_Pos)                 /*!< GPIO_T::INTTYPE: TYPE6 Mask            */

#define GPIO_INTEN_FLIEN0_Pos            (0)                                               /*!< GPIO_T::INTEN: FLIEN0 Position         */
#define GPIO_INTEN_FLIEN0_Msk            (0x1ul << GPIO_INTEN_FLIEN0_Pos)                  /*!< GPIO_T::INTEN: FLIEN0 Mask             */

#define GPIO_INTEN_FLIEN1_Pos            (1)                                               /*!< GPIO_T::INTEN: FLIEN1 Position         */
#define GPIO_INTEN_FLIEN1_Msk            (0x1ul << GPIO_INTEN_FLIEN1_Pos)                  /*!< GPIO_T::INTEN: FLIEN1 Mask             */

#define GPIO_INTEN_FLIEN2_Pos            (2)                                               /*!< GPIO_T::INTEN: FLIEN2 Position         */
#define GPIO_INTEN_FLIEN2_Msk            (0x1ul << GPIO_INTEN_FLIEN2_Pos)                  /*!< GPIO_T::INTEN: FLIEN2 Mask             */

#define GPIO_INTEN_FLIEN3_Pos            (3)                                               /*!< GPIO_T::INTEN: FLIEN3 Position         */
#define GPIO_INTEN_FLIEN3_Msk            (0x1ul << GPIO_INTEN_FLIEN3_Pos)                  /*!< GPIO_T::INTEN: FLIEN3 Mask             */

#define GPIO_INTEN_FLIEN4_Pos            (4)                                               /*!< GPIO_T::INTEN: FLIEN4 Position         */
#define GPIO_INTEN_FLIEN4_Msk            (0x1ul << GPIO_INTEN_FLIEN4_Pos)                  /*!< GPIO_T::INTEN: FLIEN4 Mask             */

#define GPIO_INTEN_FLIEN5_Pos            (5)                                               /*!< GPIO_T::INTEN: FLIEN5 Position         */
#define GPIO_INTEN_FLIEN5_Msk            (0x1ul << GPIO_INTEN_FLIEN5_Pos)                  /*!< GPIO_T::INTEN: FLIEN5 Mask             */

#define GPIO_INTEN_FLIEN6_Pos            (6)                                               /*!< GPIO_T::INTEN: FLIEN6 Position         */
#define GPIO_INTEN_FLIEN6_Msk            (0x1ul << GPIO_INTEN_FLIEN6_Pos)                  /*!< GPIO_T::INTEN: FLIEN6 Mask             */

#define GPIO_INTEN_RHIEN0_Pos            (16)                                              /*!< GPIO_T::INTEN: RHIEN0 Position         */
#define GPIO_INTEN_RHIEN0_Msk            (0x1ul << GPIO_INTEN_RHIEN0_Pos)                  /*!< GPIO_T::INTEN: RHIEN0 Mask             */

#define GPIO_INTEN_RHIEN1_Pos            (17)                                              /*!< GPIO_T::INTEN: RHIEN1 Position         */
#define GPIO_INTEN_RHIEN1_Msk            (0x1ul << GPIO_INTEN_RHIEN1_Pos)                  /*!< GPIO_T::INTEN: RHIEN1 Mask             */

#define GPIO_INTEN_RHIEN2_Pos            (18)                                              /*!< GPIO_T::INTEN: RHIEN2 Position         */
#define GPIO_INTEN_RHIEN2_Msk            (0x1ul << GPIO_INTEN_RHIEN2_Pos)                  /*!< GPIO_T::INTEN: RHIEN2 Mask             */

#define GPIO_INTEN_RHIEN3_Pos            (19)                                              /*!< GPIO_T::INTEN: RHIEN3 Position         */
#define GPIO_INTEN_RHIEN3_Msk            (0x1ul << GPIO_INTEN_RHIEN3_Pos)                  /*!< GPIO_T::INTEN: RHIEN3 Mask             */

#define GPIO_INTEN_RHIEN4_Pos            (20)                                              /*!< GPIO_T::INTEN: RHIEN4 Position         */
#define GPIO_INTEN_RHIEN4_Msk            (0x1ul << GPIO_INTEN_RHIEN4_Pos)                  /*!< GPIO_T::INTEN: RHIEN4 Mask             */

#define GPIO_INTEN_RHIEN5_Pos            (21)                                              /*!< GPIO_T::INTEN: RHIEN5 Position         */
#define GPIO_INTEN_RHIEN5_Msk            (0x1ul << GPIO_INTEN_RHIEN5_Pos)                  /*!< GPIO_T::INTEN: RHIEN5 Mask             */

#define GPIO_INTEN_RHIEN6_Pos            (22)                                              /*!< GPIO_T::INTEN: RHIEN6 Position         */
#define GPIO_INTEN_RHIEN6_Msk            (0x1ul << GPIO_INTEN_RHIEN6_Pos)                  /*!< GPIO_T::INTEN: RHIEN6 Mask             */

#define GPIO_INTSRC_INTSRC0_Pos          (0)                                               /*!< GPIO_T::INTSRC: INTSRC0 Position       */
#define GPIO_INTSRC_INTSRC0_Msk          (0x1ul << GPIO_INTSRC_INTSRC0_Pos)                /*!< GPIO_T::INTSRC: INTSRC0 Mask           */

#define GPIO_INTSRC_INTSRC1_Pos          (1)                                               /*!< GPIO_T::INTSRC: INTSRC1 Position       */
#define GPIO_INTSRC_INTSRC1_Msk          (0x1ul << GPIO_INTSRC_INTSRC1_Pos)                /*!< GPIO_T::INTSRC: INTSRC1 Mask           */

#define GPIO_INTSRC_INTSRC2_Pos          (2)                                               /*!< GPIO_T::INTSRC: INTSRC2 Position       */
#define GPIO_INTSRC_INTSRC2_Msk          (0x1ul << GPIO_INTSRC_INTSRC2_Pos)                /*!< GPIO_T::INTSRC: INTSRC2 Mask           */

#define GPIO_INTSRC_INTSRC3_Pos          (3)                                               /*!< GPIO_T::INTSRC: INTSRC3 Position       */
#define GPIO_INTSRC_INTSRC3_Msk          (0x1ul << GPIO_INTSRC_INTSRC3_Pos)                /*!< GPIO_T::INTSRC: INTSRC3 Mask           */

#define GPIO_INTSRC_INTSRC4_Pos          (4)                                               /*!< GPIO_T::INTSRC: INTSRC4 Position       */
#define GPIO_INTSRC_INTSRC4_Msk          (0x1ul << GPIO_INTSRC_INTSRC4_Pos)                /*!< GPIO_T::INTSRC: INTSRC4 Mask           */

#define GPIO_INTSRC_INTSRC5_Pos          (5)                                               /*!< GPIO_T::INTSRC: INTSRC5 Position       */
#define GPIO_INTSRC_INTSRC5_Msk          (0x1ul << GPIO_INTSRC_INTSRC5_Pos)                /*!< GPIO_T::INTSRC: INTSRC5 Mask           */

#define GPIO_INTSRC_INTSRC6_Pos          (6)                                               /*!< GPIO_T::INTSRC: INTSRC6 Position       */
#define GPIO_INTSRC_INTSRC6_Msk          (0x1ul << GPIO_INTSRC_INTSRC6_Pos)                /*!< GPIO_T::INTSRC: INTSRC6 Mask           */

#define GPIO_SMTEN_SMTEN0_Pos            (0)                                               /*!< GPIO_T::SMTEN: SMTEN0 Position         */
#define GPIO_SMTEN_SMTEN0_Msk            (0x1ul << GPIO_SMTEN_SMTEN0_Pos)                  /*!< GPIO_T::SMTEN: SMTEN0 Mask             */

#define GPIO_SMTEN_SMTEN1_Pos            (1)                                               /*!< GPIO_T::SMTEN: SMTEN1 Position         */
#define GPIO_SMTEN_SMTEN1_Msk            (0x1ul << GPIO_SMTEN_SMTEN1_Pos)                  /*!< GPIO_T::SMTEN: SMTEN1 Mask             */

#define GPIO_SMTEN_SMTEN2_Pos            (2)                                               /*!< GPIO_T::SMTEN: SMTEN2 Position         */
#define GPIO_SMTEN_SMTEN2_Msk            (0x1ul << GPIO_SMTEN_SMTEN2_Pos)                  /*!< GPIO_T::SMTEN: SMTEN2 Mask             */

#define GPIO_SMTEN_SMTEN3_Pos            (3)                                               /*!< GPIO_T::SMTEN: SMTEN3 Position         */
#define GPIO_SMTEN_SMTEN3_Msk            (0x1ul << GPIO_SMTEN_SMTEN3_Pos)                  /*!< GPIO_T::SMTEN: SMTEN3 Mask             */

#define GPIO_SMTEN_SMTEN4_Pos            (4)                                               /*!< GPIO_T::SMTEN: SMTEN4 Position         */
#define GPIO_SMTEN_SMTEN4_Msk            (0x1ul << GPIO_SMTEN_SMTEN4_Pos)                  /*!< GPIO_T::SMTEN: SMTEN4 Mask             */

#define GPIO_SMTEN_SMTEN5_Pos            (5)                                               /*!< GPIO_T::SMTEN: SMTEN5 Position         */
#define GPIO_SMTEN_SMTEN5_Msk            (0x1ul << GPIO_SMTEN_SMTEN5_Pos)                  /*!< GPIO_T::SMTEN: SMTEN5 Mask             */

#define GPIO_SMTEN_SMTEN6_Pos            (6)                                               /*!< GPIO_T::SMTEN: SMTEN6 Position         */
#define GPIO_SMTEN_SMTEN6_Msk            (0x1ul << GPIO_SMTEN_SMTEN6_Pos)                  /*!< GPIO_T::SMTEN: SMTEN6 Mask             */

#define GPIO_SLEWCTL_HSREN0_Pos          (0)                                               /*!< GPIO_T::SLEWCTL: HSREN0 Position          */
#define GPIO_SLEWCTL_HSREN0_Msk          (0x1ul << GPIO_SLEWCTL_HSREN0_Pos)                /*!< GPIO_T::SLEWCTL: HSREN0 Mask              */

#define GPIO_SLEWCTL_HSREN1_Pos          (1)                                               /*!< GPIO_T::SLEWCTL: HSREN1 Position          */
#define GPIO_SLEWCTL_HSREN1_Msk          (0x1ul << GPIO_SLEWCTL_HSREN1_Pos)                /*!< GPIO_T::SLEWCTL: HSREN1 Mask              */

#define GPIO_SLEWCTL_HSREN2_Pos          (2)                                               /*!< GPIO_T::SLEWCTL: HSREN2 Position          */
#define GPIO_SLEWCTL_HSREN2_Msk          (0x1ul << GPIO_SLEWCTL_HSREN2_Pos)                /*!< GPIO_T::SLEWCTL: HSREN2 Mask              */

#define GPIO_SLEWCTL_HSREN3_Pos          (3)                                               /*!< GPIO_T::SLEWCTL: HSREN3 Position          */
#define GPIO_SLEWCTL_HSREN3_Msk          (0x1ul << GPIO_SLEWCTL_HSREN3_Pos)                /*!< GPIO_T::SLEWCTL: HSREN3 Mask              */

#define GPIO_SLEWCTL_HSREN4_Pos          (4)                                               /*!< GPIO_T::SLEWCTL: HSREN4 Position          */
#define GPIO_SLEWCTL_HSREN4_Msk          (0x1ul << GPIO_SLEWCTL_HSREN4_Pos)                /*!< GPIO_T::SLEWCTL: HSREN4 Mask              */

#define GPIO_SLEWCTL_HSREN5_Pos          (5)                                               /*!< GPIO_T::SLEWCTL: HSREN5 Position          */
#define GPIO_SLEWCTL_HSREN5_Msk          (0x1ul << GPIO_SLEWCTL_HSREN5_Pos)                /*!< GPIO_T::SLEWCTL: HSREN5 Mask              */

#define GPIO_SLEWCTL_HSREN6_Pos          (6)                                               /*!< GPIO_T::SLEWCTL: HSREN6 Position          */
#define GPIO_SLEWCTL_HSREN6_Msk          (0x1ul << GPIO_SLEWCTL_HSREN6_Pos)                /*!< GPIO_T::SLEWCTL: HSREN6 Mask              */

#define GPIO_PLEN_PX_PLEN0_Pos           (0)                                               /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN0 Position */
#define GPIO_PLEN_PX_PLEN0_Msk           (0x1ul << GPIO_PLEN_PX_PLEN0_Pos)                /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN0 Mask     */

#define GPIO_PLEN_PX_PLEN1_Pos           (1)                                               /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN1 Position */
#define GPIO_PLEN_PX_PLEN1_Msk           (0x1ul << GPIO_PLEN_PX_PLEN1_Pos)                /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN1 Mask     */

#define GPIO_PLEN_PX_PLEN2_Pos           (2)                                               /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN2 Position */
#define GPIO_PLEN_PX_PLEN2_Msk           (0x1ul << GPIO_PLEN_PX_PLEN2_Pos)                /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN2 Mask     */

#define GPIO_PLEN_PX_PLEN3_Pos           (3)                                               /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN3 Position */
#define GPIO_PLEN_PX_PLEN3_Msk           (0x1ul << GPIO_PLEN_PX_PLEN3_Pos)                /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN3 Mask     */

#define GPIO_PLEN_PX_PLEN4_Pos           (4)                                               /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN4 Position */
#define GPIO_PLEN_PX_PLEN4_Msk           (0x1ul << GPIO_PLEN_PX_PLEN4_Pos)                /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN4 Mask     */

#define GPIO_PLEN_PX_PLEN5_Pos           (5)                                               /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN5 Position */
#define GPIO_PLEN_PX_PLEN5_Msk           (0x1ul << GPIO_PLEN_PX_PLEN5_Pos)                /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN5 Mask     */

#define GPIO_PLEN_PX_PLEN6_Pos           (6)                                               /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN6 Position */
#define GPIO_PLEN_PX_PLEN6_Msk           (0x1ul << GPIO_PLEN_PX_PLEN6_Pos)                /*!< GPIO_T::PLEN: PA/PB/PC/PD_PLEN6 Mask     */

#define GPIO_PHEN_PX_PHEN0_Pos           (0)                                               /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN0 Position */
#define GPIO_PHEN_PX_PHEN0_Msk           (0x1ul << GPIO_PHEN_PX_PHEN0_Pos)                /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN0 Mask     */

#define GPIO_PHEN_PX_PHEN1_Pos           (1)                                               /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN1 Position */
#define GPIO_PHEN_PX_PHEN1_Msk           (0x1ul << GPIO_PHEN_PX_PHEN1_Pos)                /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN1 Mask     */

#define GPIO_PHEN_PX_PHEN2_Pos           (2)                                               /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN2 Position */
#define GPIO_PHEN_PX_PHEN2_Msk           (0x1ul << GPIO_PHEN_PX_PHEN2_Pos)                /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN2 Mask     */

#define GPIO_PHEN_PX_PHEN3_Pos           (3)                                               /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN3 Position */
#define GPIO_PHEN_PX_PHEN3_Msk           (0x1ul << GPIO_PHEN_PX_PHEN3_Pos)                /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN3 Mask     */

#define GPIO_PHEN_PX_PHEN4_Pos           (4)                                               /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN4 Position */
#define GPIO_PHEN_PX_PHEN4_Msk           (0x1ul << GPIO_PHEN_PX_PHEN4_Pos)                /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN4 Mask     */

#define GPIO_PHEN_PX_PHEN5_Pos           (5)                                               /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN5 Position */
#define GPIO_PHEN_PX_PHEN5_Msk           (0x1ul << GPIO_PHEN_PX_PHEN5_Pos)                /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN5 Mask     */

#define GPIO_PHEN_PX_PHEN6_Pos           (6)                                               /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN6 Position */
#define GPIO_PHEN_PX_PHEN6_Msk           (0x1ul << GPIO_PHEN_PX_PHEN6_Pos)                /*!< GPIO_T::PHEN: PA/PB/PC/PD_PHEN6 Mask     */

#define GPIO_DBCTL_DBCLKSEL_Pos          (0)                                               /*!< GPIO_DB_T::GPIO_DBCTL: BCLKSEL Position   */
#define GPIO_DBCTL_DBCLKSEL_Msk          (0xful << GPIO_DBCTL_DBCLKSEL_Pos)                /*!< GPIO_DB_T::GPIO_DBCTL: BCLKSEL Mask       */

#define GPIO_DBCTL_DBCLKSRC_Pos          (4)                                               /*!< GPIO_DB_T::GPIO_DBCTL: BCLKSRC Position   */
#define GPIO_DBCTL_DBCLKSRC_Msk          (0x1ul << GPIO_DBCTL_DBCLKSRC_Pos)                /*!< GPIO_DB_T::GPIO_DBCTL: BCLKSRC Mask       */

#define GPIO_DBCTL_ICLKON_Pos            (5)                                               /*!< GPIO_DB_T::GPIO_DBCTL: ICLKON Position    */
#define GPIO_DBCTL_ICLKON_Msk            (0x1ul << GPIO_DBCTL_ICLKON_Pos)                  /*!< GPIO_DB_T::GPIO_DBCTL: ICLKON Mask        */

/**@}*/ /* GPIO_CONST */
/**@}*/ /* end of GPIO register group */


/*---------------------- Timer Controller -------------------------*/
/**
    @addtogroup TIMER Timer Controller(TIMER)
    Memory Mapped Structure for TMR Controller
@{ */

typedef struct
{


    /**
     * @var TIMER_T::CTL
     * Offset: 0x00  Timer Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PSC       |Prescale Counter
     * |        |          |Timer input clock source is divided by (PSC+1) before it is fed to the Timer up counter
     * |        |          |If this field is 0 (PSC = 0), then there is no scaling.
     * |[16]    |CNTDATEN  |Data Load Enable Control
     * |        |          |When CNTDATEN is set, CNT (TIMERx_CNT[23:0]) (Timer Data Register) will be updated continuously with the 24-bit up-timer value as the timer is counting.
     * |        |          |0 = Timer Data Register update Disabled.
     * |        |          |1 = Timer Data Register update Enabled while Timer counter is active.
     * |[17]    |CMPCTL    |TIMERx_CMP Mode Control
     * |        |          |0 = In One-shot or Periodic mode, when write new CMPDAT, timer counter will reset.
     * |        |          |1 = In One-shot or Periodic mode, when write new CMPDAT if new CMPDAT > CNT (TIMERx_CNT[23:0])(current counter) , timer counter keep counting and will not reset
     * |        |          |If new CMPDAT <= CNT(current counter) , timer counter will reset.
     * |[23]    |WKEN      |Wake-Up Enable
     * |        |          |When WKEN is set and the TIF or CAPIF is set, the timer controller will generator a wake-up trigger event to CPU.
     * |        |          |0 = Wake-up trigger event Disabled.
     * |        |          |1 = Wake-up trigger event Enabled.
     * |[24]    |EXTCNTEN  |Counter Mode Enable Control
     * |        |          |This bit is for external counting pin function enabled
     * |        |          |When timer is used as an event counter, this bit should be set to 1 and select HCLK as timer clock source
     * |        |          |Please refer to section "Event Counting Mode" for detail description.
     * |        |          |0 = External event counter mode Disabled.
     * |        |          |1 = External event counter mode Enabled.
     * |[25]    |ACTSTS    |Timer Active Status (Read Only)
     * |        |          |This bit indicates the 24-bit up counter status.
     * |        |          |0 = 24-bit up counter is not active.
     * |        |          |1 = 24-bit up counter is active.
     * |[26]    |RSTCNT    |Timer Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset 8-bit PSC counter, 24-bit up counter value and CEN bit if ACTSTS is 1.
     * |[28:27] |OPMODE    |Timer Operating Mode
     * |        |          |00 = The timer is operating in the One-shot OPMODE
     * |        |          |The associated interrupt signal is generated once (if INTEN is enabled) and CEN is automatically cleared by hardware.
     * |        |          |01 = The timer is operating in Periodic OPMODE
     * |        |          |The associated interrupt signal is generated periodically (if INTEN is enabled).
     * |        |          |10 = The timer is operating in Toggle OPMODE
     * |        |          |The interrupt signal is generated periodically (if INTEN is enabled)
     * |        |          |The associated signal (tout) is changing back and forth with 50% duty cycle.
     * |        |          |11 = The timer is operating in Continuous Counting mode
     * |        |          |The associated interrupt signal is generated when TIMERx_CNT = TIMERx_CMP (if INTEN is enabled)
     * |        |          |However, the 24-bit up-timer counts continuously
     * |        |          |Please refer to 6.12.5.2 for detailed description about Continuous Counting mode operation.
     * |[29]    |INTEN     |Interrupt Enable Control
     * |        |          |0 = Timer Interrupt function Disabled.
     * |        |          |1 = Timer Interrupt function Enabled.
     * |        |          |If this bit is enabled, when the timer interrupt flag (TIF) is set to 1, the timer interrupt signal is generated and inform to CPU.
     * |[30]    |CNTEN     |Timer Enable Control
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note1: In stop status, and then set CEN to 1 will enable the 24-bit up counter to keep  counting from the last stop counting value.
     * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (TIMERx_CTL[28:27] = 00) when the timer interrupt flag (TIF) is generated.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects TIMER counting.
     * |        |          |Timer counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |Timer counter will keep going no matter CPU is held by ICE or not.
     * @var TIMER_T::CMP
     * Offset: 0x04  Timer Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CMPDAT    |Timer Compared Value
     * |        |          |CMPDAT is a 24-bit compared value register
     * |        |          |When the internal 24-bit up counter value is equal to CMPDAT value, the TIF flag will set to 1.
     * |        |          |Time-out period = (Period of Timer clock source) * (8-bit PSC + 1) * (24-bit CMPDAT).
     * |        |          |Note1: Never write 0x0 or 0x1 in CMPDAT field, or the core will run into unknown state.
     * |        |          |Note2: When Timer is operating at Continuous Counting mode, the 24-bit up counter will keep counting continuously even if software writes a new value into CMPDAT field
     * |        |          |But if Timer is operating at other modes except Periodic mode on M05xxDN/DE, the 24-bit up counter will restart counting and using newest CMPDAT value to be the timer compared value if software writes a new value into CMPDAT field.
     * @var TIMER_T::INTSTS
     * Offset: 0x08  Timer Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Timer Interrupt Flag
     * |        |          |This bit indicates the interrupt flag status of Timer while CNT (TIMERx_CNT[23:0]) value reaches to CMPDAT value.
     * |        |          |0 = No effect.
     * |        |          |1 = CNT value matches the CMPDAT value.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[1]     |TWKF      |Timer Wake-Up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of Time.
     * |        |          |0 = Timer does not cause chip wake-up.
     * |        |          |1 = Chip wake-up from Idle or Power-down mode if Timer time-out interrupt signal generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var TIMER_T::CNT
     * Offset: 0x0C  Timer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CNT       |Timer Data Register
     * |        |          |If CNTDATEN is set to 1, CNT register value will be updated continuously to monitor 24-bit up counter value.
     * @var TIMER_T::CAP
     * Offset: 0x10  Timer Capture Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CAPDAT    |Timer Capture Data Register
     * |        |          |When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on ACMPOx matched the CAPEDGE (TIMERx_EXTCTL[2:1]) setting, CAPIF (TIMERx_EINTSTS[0]) will set to 1 and the current timer counter value CNT (TIMERx_CNT[23:0]) will be auto-loaded into this CAPDAT field.
     * @var TIMER_T::EXTCTL
     * Offset: 0x14  Timer Extended Event Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTPHASE  |Timer External Count Pin Phase Detect Selection
     * |        |          |This bit indicates the detection phase of TMx (x = 0~1) pin.
     * |        |          |0 = A falling edge of TMx (x = 0~1) pin will be counted.
     * |        |          |1 = A rising edge of TMx (x = 0~1) pin will be counted.
     * |[2:1]   |CAPEDGE   |Timer Capture Pin Edge Detection
     * |        |          |00 = A falling edge on ACMPOx will be detected.
     * |        |          |01 = A rising edge on ACMPO1 will be detected.
     * |        |          |10 = Either rising or falling edge on ACMPOx will be detected.
     * |        |          |11 = Reserved.
     * |[3]     |CAPEN     |Timer Capture Function Enable Bit
     * |        |          |This bit enables the Timer Capture Function
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[4]     |CAPFUNCS  |Capture Function Select Bit
     * |        |          |0 = Capture Mode Enabled.
     * |        |          |1 = Reset Mode Enabled.
     * |        |          |Note1: When CAPFUNCS is 0, transition on ACMPOx is using to save the 24-bit timer counter value to CAPDAT register.
     * |        |          |Note2: When CAPFUNCS is 1, transition on ACMPOx is using to reset the 24-bit timer counter value.
     * |[5]     |CAPIEN    |Timer Capture Interrupt Enable Bit
     * |        |          |0 = Timer Capture Interrupt Disabled.
     * |        |          |1 = Timer Capture Interrupt Enabled.
     * |        |          |Note: CAPIEN is used to enable timer capture interrupt
     * |        |          |If CAPIEN enabled, timer will generate an interrupt when CAPIF (TIMERx_EINTSTS[0]) is 1.
     * |        |          |For example, while CAPIEN = 1, CAPEN = 1, and CAPEDGE = 00, an 1 to 0 transition on the ACMPOx will cause the CAPIF to be set then the interrupt signal is generated and sent to NVIC to inform CPU.
     * |[7]     |ECNTDBEN  |Timer Counter Input Pin De-Bounce Enable Control
     * |        |          |0 = TMx (x = 0~1) pin de-bounce Disabled.
     * |        |          |1 = TMx (x = 0~1) pin de-bounce Enabled.
     * |        |          |If this bit is enabled, the edge detection of TMx (x = 0~1) pin is detected with de-bounce circuit.
     * |[8]     |CAPMODE   |Capture Mode Select Bit
     * |        |          |0 = Timer counter reset function or free-counting mode of timer capture function.
     * |        |          |1 = Trigger-counting mode of timer capture function.
     * @var TIMER_T::EINTSTS
     * Offset: 0x18  Timer Extended Event Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPIF     |Timer Capture Interrupt Flag
     * |        |          |This bit indicates the timer external capture interrupt flag status.
     * |        |          |0 = Timer Capture interrupt did not occur.
     * |        |          |1 = Timer Capture interrupt occurred.
     * |        |          |Note1: This bit is cleared by writing 1 to it.
     * |        |          |Note2: When CAPEN (TIMERx_EXTCTL[3]) bit is set, CAPFUNCS (TIMERx_EXTCTL[4]) bit is 0, and a transition on ACMPOx matched the CAPEDGE (TIMERx_EXTCTL[2:1]) setting, this bit will set to 1 by hardware.
     * |        |          |Note3: There is a new incoming capture event detected before CPU clearing the CAPIF status
     * |        |          |If the above condition occurred, the Timer will keep register TIMERx_CAP unchanged and drop the new capture value.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] Timer Control and Status Register                               */
    __IO uint32_t CMP;                   /*!< [0x0004] Timer Compare Register                                          */
    __IO uint32_t INTSTS;                /*!< [0x0008] Timer Interrupt Status Register                                 */
    __I  uint32_t CNT;                   /*!< [0x000c] Timer Data Register                                             */
    __I  uint32_t CAP;                   /*!< [0x0010] Timer Capture Data Register                                     */
    __IO uint32_t EXTCTL;                /*!< [0x0014] Timer Extended Event Control Register                           */
    __IO uint32_t EINTSTS;               /*!< [0x0018] Timer Extended Event Interrupt Status Register                  */
} TIMER_T;

typedef struct
{
    /**
    * @var TIMER_AC_T::CCAPCTL
    * Offset: 0x40  Timer Continuous Capture Control Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[0]     |CCAPEN    |Continuous Capture Enable
    * |        |          |This bit is to be enabled the continuous capture function.
    * |        |          |0 = Disable.
    * |        |          |1 = Enable.
    * |        |          |Note: This bit is cleared by hardware automatically when capture operation finish or writing 0 to it
    * |[1]     |INV       |Input Signal Inverse
    * |        |          |Invert the input signal which be captured.
    * |        |          |0 = None.
    * |        |          |1 = Inverse.
    * |[3:2]   |CNTSEL    |Capture Timer Selection
    * |        |          |Select the timer to continuous capture the input signal.
    * |        |          |00 = TIMER0.
    * |        |          |01 = TIMER1.
    * |        |          |10 = SysTick.
    * |        |          |11 = Reserved.
    * |[4]     |CAPCHSEL  |Capture Timer Channel Selection
    * |        |          |Select the channel to be the continuous capture event.
    * |        |          |0 = PD.2
    * |        |          |1 = PC.2
    * |[8]     |CAPR1F    |Capture Rising Edge 1 Flag
    * |        |          |First rising edge already captured, this bit will be set to 1.
    * |        |          |0 = None.
    * |        |          |1 = TACDR0 data is ready for read.
    * |        |          |Note: This bit is cleared by hardware automatically when write ACAP_EN to 1 or writing 1 to it
    * |[9]     |CAPF1F    |Capture Falling Edge 1 Flag
    * |        |          |First falling edge already captured, this bit will be set to 1.
    * |        |          |0 = None.
    * |        |          |1 = TACDR1 data is ready for read.
    * |        |          |Note: This bit is cleared by hardware automatically when write ACAP_EN to 1 or writing 1 to it
    * |[10]    |CAPR2F    |Capture Rising Edge 2 Flag
    * |        |          |Second rising edge already captured, this bit will be set to 1.
    * |        |          |0 = None.
    * |        |          |1 = TACDR2 data is ready for read.
    * |        |          |Note: This bit is cleared by hardware automatically when write ACAP_EN to 1 or writing 1 to it
    * |[11]    |CAPF2F    |Capture Falling Edge 2 Flag
    * |        |          |Second falling edge already captured, this bit will be set to 1
    * |        |          |0 = None.
    * |        |          |1 = TCAP0 or TCAP1 data is ready for read.
    * |        |          |When TMR_SEL=0, TCAP0 store the timer 0 counter value.
    * |        |          |When TMR_SEL=1, TCAP1 store the timer 1 counter value.
    * |        |          |Note: This bit is cleared by hardware automatically when write ACAP_EN to 1 or writing 1 to it
    * |[17:16] |CCAPIEN   |Capture Interrupt Enable
    * |        |          |00 = Interrupt disable.
    * |        |          |01 = Capture Rising Edge 1 and Falling Edge 1 interrupt enable.
    * |        |          |10 = Capture Rising Edge 1, Falling Edge 1 and Rising Edge 2 interrupt enable.
    * |        |          |11 = Capture Rising Edge 1, Falling edge 1, Rising Edge 2 and Falling Edge 2 interrupt enable.
    * @var TIMER_AC_T::CCAP
    * Offset: 0x44~0x50  Timer Continuous Capture Data Register
    * ---------------------------------------------------------------------------------------------------
    * |Bits    |Field     |Descriptions
    * | :----: | :----:   | :---- |
    * |[23:0]  |CAPDAT    |Timer Continuous Capture Data Register
    * |        |          |TIMER_CCAP0 store the timer count value of first rising edge
    * |        |          |TIMER_CCAP1 store the timer count value of first falling edge
    * |        |          |TIMER_CCAP2 store the timer count value of second rising edge
    * |        |          |TIMER_CCAP3 store the timer count value of second rising edge
    */
    __IO uint32_t CCAPCTL;               /*!< [0x0040] Timer Continuous Capture Control Register                        */
    __I  uint32_t CCAP[4];               /*!< [0x0044 ~ 0x0050] Timer Continuous Capture Data Register 0/1/2/3          */

} TIMER_AC_T;

/**
    @addtogroup TIMER_CONST TIMER Bit Field Definition
    Constant Definitions for TIMER Controller
@{ */

#define TIMER_CTL_PSC_Pos                  (0)                                               /*!< TIMER_T::CTL: PSC Position               */
#define TIMER_CTL_PSC_Msk                  (0xfful << TIMER_CTL_PSC_Pos)                     /*!< TIMER_T::CTL: PSC Mask                   */

#define TIMER_CTL_CNTDATEN_Pos             (16)                                              /*!< TIMER_T::CTL: CNTDATEN Position          */
#define TIMER_CTL_CNTDATEN_Msk             (0x1ul << TIMER_CTL_CNTDATEN_Pos)                 /*!< TIMER_T::CTL: CNTDATEN Mask              */

#define TIMER_CTL_CMPCTL_Pos               (17)                                              /*!< TIMER_T::CTL: CMPCTL Position            */
#define TIMER_CTL_CMPCTL_Msk               (0x1ul << TIMER_CTL_CMPCTL_Pos)                   /*!< TIMER_T::CTL: CMPCTL Mask                */

#define TIMER_CTL_WKEN_Pos                 (23)                                              /*!< TIMER_T::CTL: WKEN Position              */
#define TIMER_CTL_WKEN_Msk                 (0x1ul << TIMER_CTL_WKEN_Pos)                     /*!< TIMER_T::CTL: WKEN Mask                  */

#define TIMER_CTL_EXTCNTEN_Pos             (24)                                              /*!< TIMER_T::CTL: EXTCNTEN Position          */
#define TIMER_CTL_EXTCNTEN_Msk             (0x1ul << TIMER_CTL_EXTCNTEN_Pos)                 /*!< TIMER_T::CTL: EXTCNTEN Mask              */

#define TIMER_CTL_ACTSTS_Pos               (25)                                              /*!< TIMER_T::CTL: ACTSTS Position            */
#define TIMER_CTL_ACTSTS_Msk               (0x1ul << TIMER_CTL_ACTSTS_Pos)                   /*!< TIMER_T::CTL: ACTSTS Mask                */

#define TIMER_CTL_RSTCNT_Pos               (26)                                              /*!< TIMER_T::CTL: RSTCNT Position            */
#define TIMER_CTL_RSTCNT_Msk               (0x1ul << TIMER_CTL_RSTCNT_Pos)                   /*!< TIMER_T::CTL: RSTCNT Mask                */

#define TIMER_CTL_OPMODE_Pos               (27)                                              /*!< TIMER_T::CTL: OPMODE Position            */
#define TIMER_CTL_OPMODE_Msk               (0x3ul << TIMER_CTL_OPMODE_Pos)                   /*!< TIMER_T::CTL: OPMODE Mask                */

#define TIMER_CTL_INTEN_Pos                (29)                                              /*!< TIMER_T::CTL: INTEN Position             */
#define TIMER_CTL_INTEN_Msk                (0x1ul << TIMER_CTL_INTEN_Pos)                    /*!< TIMER_T::CTL: INTEN Mask                 */

#define TIMER_CTL_CNTEN_Pos                (30)                                              /*!< TIMER_T::CTL: CNTEN Position               */
#define TIMER_CTL_CNTEN_Msk                (0x1ul << TIMER_CTL_CNTEN_Pos)                    /*!< TIMER_T::CTL: CNTEN Mask                   */

#define TIMER_CTL_ICEDEBUG_Pos             (31)                                              /*!< TIMER_T::CTL: ICEDEBUG Position          */
#define TIMER_CTL_ICEDEBUG_Msk             (0x1ul << TIMER_CTL_ICEDEBUG_Pos)                 /*!< TIMER_T::CTL: ICEDEBUG Mask              */

#define TIMER_CMP_CMPDAT_Pos               (0)                                               /*!< TIMER_T::CMP: CMPDAT Position            */
#define TIMER_CMP_CMPDAT_Msk               (0xfffffful << TIMER_CMP_CMPDAT_Pos)              /*!< TIMER_T::CMP: CMPDAT Mask                */

#define TIMER_INTSTS_TIF_Pos               (0)                                               /*!< TIMER_T::INTSTS: TIF Position            */
#define TIMER_INTSTS_TIF_Msk               (0x1ul << TIMER_INTSTS_TIF_Pos)                   /*!< TIMER_T::INTSTS: TIF Mask                */

#define TIMER_INTSTS_TWKF_Pos              (1)                                               /*!< TIMER_T::INTSTS: TWKF Position           */
#define TIMER_INTSTS_TWKF_Msk              (0x1ul << TIMER_INTSTS_TWKF_Pos)                  /*!< TIMER_T::INTSTS: TWKF Mask               */

#define TIMER_CNT_CNT_Pos                  (0)                                               /*!< TIMER_T::CNT: CNT Position               */
#define TIMER_CNT_CNT_Msk                  (0xfffffful << TIMER_CNT_CNT_Pos)                 /*!< TIMER_T::CNT: CNT Mask                   */

#define TIMER_CAP_CAPDAT_Pos               (0)                                               /*!< TIMER_T::CAP: CAPDAT Position            */
#define TIMER_CAP_CAPDAT_Msk               (0xfffffful << TIMER_CAP_CAPDAT_Pos)              /*!< TIMER_T::CAP: CAPDAT Mask                */

#define TIMER_EXTCTL_CNTPHASE_Pos          (0)                                               /*!< TIMER_T::EXTCTL: CNTPHASE Position       */
#define TIMER_EXTCTL_CNTPHASE_Msk          (0x1ul << TIMER_EXTCTL_CNTPHASE_Pos)              /*!< TIMER_T::EXTCTL: CNTPHASE Mask           */

#define TIMER_EXTCTL_CAPEDGE_Pos           (1)                                               /*!< TIMER_T::EXTCTL: CAPEDGE Position        */
#define TIMER_EXTCTL_CAPEDGE_Msk           (0x3ul << TIMER_EXTCTL_CAPEDGE_Pos)               /*!< TIMER_T::EXTCTL: CAPEDGE Mask            */

#define TIMER_EXTCTL_CAPEN_Pos             (3)                                               /*!< TIMER_T::EXTCTL: CAPEN Position          */
#define TIMER_EXTCTL_CAPEN_Msk             (0x1ul << TIMER_EXTCTL_CAPEN_Pos)                 /*!< TIMER_T::EXTCTL: CAPEN Mask              */

#define TIMER_EXTCTL_CAPFUNCS_Pos          (4)                                               /*!< TIMER_T::EXTCTL: CAPFUNCS Position       */
#define TIMER_EXTCTL_CAPFUNCS_Msk          (0x1ul << TIMER_EXTCTL_CAPFUNCS_Pos)              /*!< TIMER_T::EXTCTL: CAPFUNCS Mask           */

#define TIMER_EXTCTL_CAPIEN_Pos            (5)                                               /*!< TIMER_T::EXTCTL: CAPIEN Position         */
#define TIMER_EXTCTL_CAPIEN_Msk            (0x1ul << TIMER_EXTCTL_CAPIEN_Pos)                /*!< TIMER_T::EXTCTL: CAPIEN Mask             */

#define TIMER_EXTCTL_ECNTDBEN_Pos          (7)                                               /*!< TIMER_T::EXTCTL: ECNTDBEN Position       */
#define TIMER_EXTCTL_ECNTDBEN_Msk          (0x1ul << TIMER_EXTCTL_ECNTDBEN_Pos)              /*!< TIMER_T::EXTCTL: ECNTDBEN Mask           */

#define TIMER_EXTCTL_CAPMODE_Pos           (8)                                               /*!< TIMER_T::EXTCTL: CAPMODE Position        */
#define TIMER_EXTCTL_CAPMODE_Msk           (0x1ul << TIMER_EXTCTL_CAPMODE_Pos)               /*!< TIMER_T::EXTCTL: CAPMODE Mask            */

#define TIMER_EINTSTS_CAPIF_Pos            (0)                                               /*!< TIMER_T::EINTSTS: CAPIF Position         */
#define TIMER_EINTSTS_CAPIF_Msk            (0x1ul << TIMER_EINTSTS_CAPIF_Pos)                  /*!< TIMER_T::EINTSTS: CAPIF Mask             */

#define TIMER_CCAPCTL_CCAPEN_Pos           (0)                                               /*!< TIMER_AC_T::CCAPCTL: CCAPEN Position        */
#define TIMER_CCAPCTL_CCAPEN_Msk           (0x1ul << TIMER_CCAPCTL_CCAPEN_Pos)               /*!< TIMER_AC_T::CCAPCTL: CCAPEN Mask            */

#define TIMER_CCAPCTL_INV_Pos              (1)                                               /*!< TIMER_AC_T::CCAPCTL: INV Position           */
#define TIMER_CCAPCTL_INV_Msk              (0x1ul << TIMER_CCAPCTL_INV_Pos)                  /*!< TIMER_AC_T::CCAPCTL: INV Mask               */

#define TIMER_CCAPCTL_CNTSEL_Pos           (2)                                               /*!< TIMER_AC_T::CCAPCTL: CNTSEL Position        */
#define TIMER_CCAPCTL_CNTSEL_Msk           (0x3ul << TIMER_CCAPCTL_CNTSEL_Pos)               /*!< TIMER_AC_T::CCAPCTL: CNTSEL Mask            */

#define TIMER_CCAPCTL_CAPCHSEL_Pos         (4)                                               /*!< TIMER_AC_T::CCAPCTL: CAPCHSEL Position      */
#define TIMER_CCAPCTL_CAPCHSEL_Msk         (0x1ul << TIMER_CCAPCTL_CAPCHSEL_Pos)             /*!< TIMER_AC_T::CCAPCTL: CAPCHSEL Mask          */

#define TIMER_CCAPCTL_CAPR1F_Pos           (8)                                               /*!< TIMER_AC_T::CCAPCTL: CAPR1F Position        */
#define TIMER_CCAPCTL_CAPR1F_Msk           (0x1ul << TIMER_CCAPCTL_CAPR1F_Pos)               /*!< TIMER_AC_T::CCAPCTL: CAPR1F Mask            */

#define TIMER_CCAPCTL_CAPF1F_Pos           (9)                                               /*!< TIMER_AC_T::CCAPCTL: CAPF1F Position        */
#define TIMER_CCAPCTL_CAPF1F_Msk           (0x1ul << TIMER_CCAPCTL_CAPF1F_Pos)               /*!< TIMER_AC_T::CCAPCTL: CAPF1F Mask            */

#define TIMER_CCAPCTL_CAPR2F_Pos           (10)                                              /*!< TIMER_AC_T::CCAPCTL: CAPR2F Position        */
#define TIMER_CCAPCTL_CAPR2F_Msk           (0x1ul << TIMER_CCAPCTL_CAPR2F_Pos)               /*!< TIMER_AC_T::CCAPCTL: CAPR2F Mask            */

#define TIMER_CCAPCTL_CAPF2F_Pos           (11)                                              /*!< TIMER_AC_T::CCAPCTL: CAPF2F Position        */
#define TIMER_CCAPCTL_CAPF2F_Msk           (0x1ul << TIMER_CCAPCTL_CAPF2F_Pos)               /*!< TIMER_AC_T::CCAPCTL: CAPF2F Mask            */

#define TIMER_CCAPCTL_CCAPIEN_Pos          (16)                                              /*!< TIMER_AC_T::CCAPCTL: CCAPIEN Position       */
#define TIMER_CCAPCTL_CCAPIEN_Msk          (0x3ul << TIMER_CCAPCTL_CCAPIEN_Pos)              /*!< TIMER_AC_T::CCAPCTL: CCAPIEN Mask           */

#define TIMER_CCAP_CAPDAT_Pos             (0)                                                /*!< TIMER_AC_T::CCAP: CAPDAT Position          */
#define TIMER_CCAP_CAPDAT_Msk             (0xfffffful << TIMER_CCAP_CAPDAT_Pos)              /*!< TIMER_AC_T::CCAP: CAPDAT Mask              */


/**@}*/ /* TIMER_CONST */
/**@}*/ /* end of TMR register group */


/*---------------------- Enhanced Input Capture Timer -------------------------*/
/**
    @addtogroup ECAP Enhanced Input Capture Timer(ECAP)
    Memory Mapped Structure for ECAP Controller
@{ */

typedef struct
{


    /**
     * @var ECAP_T::CNT
     * Offset: 0x00  Input Capture Counter
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CNT       |Input Capture Timer/Counter (24-Bit Up Counter)
     * |        |          |The input Capture Timer/Counter is a 24-bit up-counting counter
     * |        |          |The clock source for the counter is from the clock divider output which the CAP_CLK is software optionally divided by 1,4,16 or 32
     * @var ECAP_T::HLD
     * Offset: 0x04 ~ 0x0C Input Capture Counter Hold Register 0/1/2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |HOLD      |Input Capture Counter Hold Register
     * |        |          |When an active input capture channel detects a valid edge signal change, the CAPCNT value is latched into the corresponding holding register
     * |        |          |Each input channel has itself holding register named by CAP_HLDx where x is from 0 to 2 to indicate inputs from IC0 to IC2, respectively.
     * @var ECAP_T::CNTCMP
     * Offset: 0x10  Input Capture Counter Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CNTCMP    |Input Capture Counter Compare Register
     * |        |          |If the compare function is enabled (CMP_EN = 1), the compare register is loaded with the value that the compare function compares the capture counter (CAP_CNT) with.
     * |        |          |If the reload control is enabled (RLD_EN = 1), an overflow event or capture events will trigger the hardware to reload CAP_CNTCMP into CAP_CNT.
     * @var ECAP_T::CTL0
     * Offset: 0x14  Input Capture Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |NFCLKS    |Noise Filter Clock Pre-Divided Selection
     * |        |          |To determine the sampling frequency of the Noise Filter clock
     * |        |          |00 = CAPCLK.
     * |        |          |01 = CAPCLK / 2.
     * |        |          |10 = CAPCLK / 4.
     * |        |          |11 = CAPCLK / 16.
     * |[3]     |CAPNFDIS  |Disable Input Capture Noise Filter
     * |        |          |0 = Noise filter of Input Capture Enabled.
     * |        |          |1 = The noise filter of Input Capture Disabled.
     * |[4]     |IC0EN     |Enable Port Pin IC0 Input To Input Capture Unit
     * |        |          |0 = IC0 input to Input Capture Unit Disabled.
     * |        |          |1 = IC0 input to Input Capture Unit Enabled.
     * |[5]     |IC1EN     |Enable Port Pin IC1 Input To Input Capture Unit
     * |        |          |0 = IC1 input to Input Capture Unit Disabled.
     * |        |          |1 = IC1 input to Input Capture Unit Enabled.
     * |[6]     |IC2EN     |Enable Port Pin IC2 Input To Input Capture Unit
     * |        |          |0 = IC2 input to Input Capture Unit Disabled.
     * |        |          |1 = IC2 input to Input Capture Unit Enabled.
     * |[9:8]   |CAP0SEL   |CAP0 Input Source Selection Bit
     * |        |          |00 = CAP0 input is from port pin ECAP_P0.
     * |        |          |01 = CAP0 input is from signal ACMP0_O (Analog comparator 0 output).
     * |        |          |10 = CAP0 input is from signal ACMP1_O (Analog comparator 1 output).
     * |        |          |11 = CAP0 input is from signal ADC_CPR (EADC compare output).
     * |[11:10] |CAP1SEL   |CAP1 Input Source Selection Bit
     * |        |          |00 = CAP1 input is from port pin ECAP_P1.
     * |        |          |01 = CAP1 input is from signal ACMP0_O (Analog comparator 0 output).
     * |        |          |10 = CAP1 input is from signal ACMP1_O (Analog comparator 1 output).
     * |        |          |11 = CAP1 input is from signal ADC_CPR (EADC compare output).
     * |[13:12] |CAP2SEL   |CAP2 Input Source Selection Bit
     * |        |          |00 = CAP2 input is from port pin ECAP_P2.
     * |        |          |01 = CAP2 input is from signal ACMP0_O (Analog comparator 0 output).
     * |        |          |10 = CAP2 input is from signal ACMP1_O (Analog comparator 1 output).
     * |        |          |11 = CAP2 input is from signal ADC_CPR (EADC compare output).
     * |[16]    |CAPTF0IEN |Enable Input Capture Channel 0 Interrupt
     * |        |          |0 = Disabling flag CAPTF0 can trigger Input Capture interrupt.
     * |        |          |1 = Enabling flag CAPTF0 can trigger Input Capture interrupt.
     * |[17]    |CAPTF1IEN |Enable Input Capture Channel 1 Interrupt
     * |        |          |0 = Disabling flag CAPTF1 can trigger Input Capture interrupt.
     * |        |          |1 = Enabling flag CAPTF1 can trigger Input Capture interrupt.
     * |[18]    |CAPTF2IEN |Enable Input Capture Channel 2 Interrupt
     * |        |          |0 = Disabling flag CAPTF2 can trigger Input Capture interrupt.
     * |        |          |1 = Enabling flag CAPTF2 can trigger Input Capture interrupt.
     * |[20]    |CAPOVIEN  |Enable CAPOVF Trigger Input Capture Interrupt
     * |        |          |0 = Disabling flag CAPOVF can trigger Input Capture interrupt.
     * |        |          |1 = Enabling flag CAPOVF can trigger Input Capture interrupt.
     * |[21]    |CAPCMPIEN |Enable CAPCMPF Trigger Input Capture Interrupt
     * |        |          |0 = Disabling flag CAPCMPF can trigger Input Capture interrupt.
     * |        |          |1 = Enabling flag CAPCMPF can trigger Input Capture interrupt.
     * |[24]    |CPTST     |Input Capture Counter Start Bit
     * |        |          |Setting this bit to 1, the capture counter (ECAP_CNT) starts up-counting synchronously with capture clock input (CAP_CLK).
     * |        |          |0 = ECAP_CNT stop counting.
     * |        |          |1 = ECAP_CNT starts up-counting.
     * |[25]    |CMPCLR    |Input Capture Counter Clear by Compare-match Control Bit
     * |        |          |If this bit is set to 1, the capture counter (ECAP_CNT) will be cleared to 0 when the compare-match event (CAMCMPF = 1) occurs.
     * |        |          |0 = Compare-match event (CAMCMPF) can clear capture counter (ECAP_CNT) Disabled.
     * |        |          |1 = Compare-match event (CAMCMPF) can clear capture counter (ECAP_CNT) Enabled.
     * |[26]    |CPTCLR    |Input Capture Counter Clear by Capture Events Control Bit
     * |        |          |If this bit is set to 1, the capture counter (ECAP_CNT) will be cleared to 0 when any one of capture events (CAPTF0~3) occurs.
     * |        |          |0 = Capture events (CAPTF0~3) can clear capture counter (ECAP_CNT) Disabled.
     * |        |          |1 = Capture events (CAPTF0~3) can clear capture counter (ECAP_CNT) Enabled.
     * |[27]    |RLDEN     |The Reload Function Enable Bit
     * |        |          |Setting this bit to enable reload function
     * |        |          |If the reload control is enabled, an overflow event (CAPOVF) or capture events (CAPTFx) will trigger the hardware to reload ECAP_CNTCMP into ECAP_CNT.
     * |        |          |0 =Reload function Disabled.
     * |        |          |1 = Reload function Enabled.
     * |[28]    |CMPEN     |The Compare Function Enable Bit
     * |        |          |The compare function in input capture timer/counter is to compare the dynamic counting ECAP_CNT with the compare register ECAP_CNTCMP, if ECAP_CNT value reaches ECAP_CNTCMP, the flag CAPCMPF will be set.
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |[29]    |CAPEN     |Input Capture Timer/Counter Enable Bit
     * |        |          |0 = Input Capture function Disabled.
     * |        |          |1 = Input Capture function Enabled.
     * |[30]    |CAPPHGEN  |Input Capture Flag Trigger PWM Phase Change Function Enable Bit
     * |        |          |0 = CAPTF2 or CAPTF1 or CAPTF0 trigger PWM phase change function Disabled.
     * |        |          |1 = CAPTF2 or CAPTF1 or CAPTF0 trigger PWM phase change function Enabled.
     * @var ECAP_T::CTL1
     * Offset: 0x18  Input Capture Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |CAPEDG0   |Channel 0 Captured Edge Selection
     * |        |          |Input capture can detect falling edge change only, rising edge change only or one of both edge change
     * |        |          |00 = Detect rising edge.
     * |        |          |01 = Detect falling edge.
     * |        |          |1x = Detect either rising or falling edge.
     * |[3:2]   |CAPEDG1   |Channel 1 Captured Edge Selection
     * |        |          |Input capture can detect falling edge change only, rising edge change only or one of both edge change
     * |        |          |00 = Detect rising edge.
     * |        |          |01 = Detect falling edge.
     * |        |          |1x = Detect either rising or falling edge.
     * |[5:4]   |CAPEDG2   |Channel 2 Captured Edge Selection
     * |        |          |Input capture can detect falling edge change only, rising edge change only or one of both edge change
     * |        |          |00 = Detect rising edge.
     * |        |          |01 = Detect falling edge.
     * |        |          |1x = Detect either rising or falling edge.
     * |[10:8]  |CPRLDS    |ECAP_CNT Reload Trigger Source Selection
     * |        |          |If the reload function is enabled (RLDEN = 1), when a reload trigger event comes, the ECAP_CNT is reloaded with ECAP_CNTCMP.
     * |        |          |CPRLDS[2:0] determines the ECAP_CNT reload trigger source
     * |        |          |000 = CAPTF0.
     * |        |          |001 = CAPTF1.
     * |        |          |010 = CAPTF2.
     * |        |          |100 = CAPOVF.
     * |        |          |Other = Reserved.
     * |[14:12] |CAPDIV    |Capture Timer Clock Divide Selection
     * |        |          |The capture timer clock has a pre-divider with four divided options controlled by CAPDIV[2:0].
     * |        |          |000 = CAPCLK / 1.
     * |        |          |001 = CAPCLK / 4.
     * |        |          |010 = CAPCLK / 16.
     * |        |          |011 = CAPCLK / 32.
     * |        |          |100 = CAPCLK / 64.
     * |        |          |101 = CAPCLK / 96.
     * |        |          |110 = CAPCLK / 112.
     * |        |          |111 = CAPCLK / 128.
     * |[17:16] |CNTSRC    |Capture Timer/Counter Clock Source Select
     * |        |          |Select the capture timer/counter clock source
     * |        |          |00 = CAPCLK (Default).
     * |        |          |01 = CAP0.
     * |        |          |10 = CAP1.
     * |        |          |11 = CAP2.
     * @var ECAP_T::STS
     * Offset: 0x1C  Input Capture Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPTF0    |Input Capture Channel 0 Captured Flag
     * |        |          |When the input capture channel 0 detects a valid edge change at CAP0 input, it will set flag CAPTF0 to high.
     * |        |          |0 = No valid edge change is detected at CAP0 input.
     * |        |          |1 = A valid edge change is detected at CAP0 input.
     * |        |          |Note: This bit is only cleared by writing 1 to itself through software.
     * |[1]     |CAPTF1    |Input Capture Channel 1 Captured Flag
     * |        |          |When the input capture channel 1 detects a valid edge change at CAP1 input, it will set flag CAPTF1 to high.
     * |        |          |0 = No valid edge change is detected at CAP1 input.
     * |        |          |1 = A valid edge change is detected at CAP1 input.
     * |        |          |Note: This bit is only cleared by writing 1 to itself through software.
     * |[2]     |CAPTF2    |Input Capture Channel 2 Captured Flag
     * |        |          |When the input capture channel 2 detects a valid edge change at CAP2 input, it will set flag CAPTF2 to high.
     * |        |          |0 = No valid edge change is detected at CAP2 input.
     * |        |          |1 = A valid edge change is detected at CAP2 input.
     * |        |          |Note: This bit is only cleared by writing 1 to itself through software.
     * |[4]     |CAPCMPF   |Input Capture Compare-Match Flag
     * |        |          |If the input capture compare function is enabled, the flag is set by hardware while capture counter (CNT) up counts and reach to the CNTCMP value.
     * |        |          |0 = CNT does not match with CNTCMP value.
     * |        |          |1 = CNT counts to the same as CNTCMP value.
     * |        |          |Note: This bit is only cleared by writing 1 to itself through software.
     * |[5]     |CAPOVF    |Input Capture Counter Overflow Flag
     * |        |          |Flag is set by hardware when input capture up counter (CNT) overflows from 0x00FF_FFFF to 0.
     * |        |          |0 = No overflow occurs in CNT.
     * |        |          |1 = CNT overflows.
     * |        |          |Note: This bit is only cleared by writing 1 to itself through software.
     * |[8]     |ECAP0     |Input Capture Pin 0 Status
     * |        |          |Input capture pin 0 (ECAP_P0) status. It is read only.
     * |[9]     |ECAP1     |Input Capture Pin 1 Status
     * |        |          |Input capture pin 1 (ECAP_P1) status. It is read only.
     * |[10]    |ECAP2     |Input Capture Pin 2 Status
     * |        |          |Input capture pin 2 (ECAP_P2) status. It is read only.
     */
    __IO uint32_t CNT;                   /*!< [0x0000] Input Capture Counter                                            */
    __IO uint32_t HLD[3];                /*!< [0x0004 ~ 0x000c] Input Capture Counter Hold Register 0/1/2               */
    __IO uint32_t CNTCMP;                /*!< [0x0010] Input Capture Counter Compare Register                           */
    __IO uint32_t CTL0;                  /*!< [0x0014] Input Capture Control Register 0                                 */
    __IO uint32_t CTL1;                  /*!< [0x0018] Input Capture Control Register 1                                 */
    __IO uint32_t STS;                   /*!< [0x001c] Input Capture Status Register                                    */

} ECAP_T;

/**
    @addtogroup ECAP_CONST ECAP Bit Field Definition
    Constant Definitions for ECAP Controller
@{ */

#define ECAP_CNT_CNT_Pos                 (0)                                               /*!< ECAP_T::CNT: CNT Position              */
#define ECAP_CNT_CNT_Msk                 (0xfffffful << ECAP_CNT_CNT_Pos)                  /*!< ECAP_T::CNT: CNT Mask                  */

#define ECAP_HLD0_HOLD_Pos               (0)                                               /*!< ECAP_T::HLD0: HOLD Position            */
#define ECAP_HLD0_HOLD_Msk               (0xfffffful << ECAP_HLD0_HOLD_Pos)                /*!< ECAP_T::HLD0: HOLD Mask                */

#define ECAP_HLD1_HOLD_Pos               (0)                                               /*!< ECAP_T::HLD1: HOLD Position            */
#define ECAP_HLD1_HOLD_Msk               (0xfffffful << ECAP_HLD1_HOLD_Pos)                /*!< ECAP_T::HLD1: HOLD Mask                */

#define ECAP_HLD2_HOLD_Pos               (0)                                               /*!< ECAP_T::HLD2: HOLD Position            */
#define ECAP_HLD2_HOLD_Msk               (0xfffffful << ECAP_HLD2_HOLD_Pos)                /*!< ECAP_T::HLD2: HOLD Mask                */

#define ECAP_CNTCMP_CNTCMP_Pos           (0)                                               /*!< ECAP_T::CNTCMP: CNTCMP Position        */
#define ECAP_CNTCMP_CNTCMP_Msk           (0xfffffful << ECAP_CNTCMP_CNTCMP_Pos)            /*!< ECAP_T::CNTCMP: CNTCMP Mask            */

#define ECAP_CTL0_NFCLKS_Pos             (0)                                               /*!< ECAP_T::CTL0: NFCLKS Position          */
#define ECAP_CTL0_NFCLKS_Msk             (0x3ul << ECAP_CTL0_NFCLKS_Pos)                   /*!< ECAP_T::CTL0: NFCLKS Mask              */

#define ECAP_CTL0_CAPNFDIS_Pos           (3)                                               /*!< ECAP_T::CTL0: CAPNFDIS Position        */
#define ECAP_CTL0_CAPNFDIS_Msk           (0x1ul << ECAP_CTL0_CAPNFDIS_Pos)                 /*!< ECAP_T::CTL0: CAPNFDIS Mask            */

#define ECAP_CTL0_IC0EN_Pos              (4)                                               /*!< ECAP_T::CTL0: IC0EN Position           */
#define ECAP_CTL0_IC0EN_Msk              (0x1ul << ECAP_CTL0_IC0EN_Pos)                    /*!< ECAP_T::CTL0: IC0EN Mask               */

#define ECAP_CTL0_IC1EN_Pos              (5)                                               /*!< ECAP_T::CTL0: IC1EN Position           */
#define ECAP_CTL0_IC1EN_Msk              (0x1ul << ECAP_CTL0_IC1EN_Pos)                    /*!< ECAP_T::CTL0: IC1EN Mask               */

#define ECAP_CTL0_IC2EN_Pos              (6)                                               /*!< ECAP_T::CTL0: IC2EN Position           */
#define ECAP_CTL0_IC2EN_Msk              (0x1ul << ECAP_CTL0_IC2EN_Pos)                    /*!< ECAP_T::CTL0: IC2EN Mask               */

#define ECAP_CTL0_CAP0SEL_Pos            (8)                                               /*!< ECAP_T::CTL0: CAP0SEL Position         */
#define ECAP_CTL0_CAP0SEL_Msk            (0x3ul << ECAP_CTL0_CAP0SEL_Pos)                  /*!< ECAP_T::CTL0: CAP0SEL Mask             */

#define ECAP_CTL0_CAP1SEL_Pos            (10)                                              /*!< ECAP_T::CTL0: CAP1SEL Position         */
#define ECAP_CTL0_CAP1SEL_Msk            (0x3ul << ECAP_CTL0_CAP1SEL_Pos)                  /*!< ECAP_T::CTL0: CAP1SEL Mask             */

#define ECAP_CTL0_CAP2SEL_Pos            (12)                                              /*!< ECAP_T::CTL0: CAP2SEL Position         */
#define ECAP_CTL0_CAP2SEL_Msk            (0x3ul << ECAP_CTL0_CAP2SEL_Pos)                  /*!< ECAP_T::CTL0: CAP2SEL Mask             */

#define ECAP_CTL0_CAPTF0IEN_Pos          (16)                                              /*!< ECAP_T::CTL0: CAPTF0IEN Position       */
#define ECAP_CTL0_CAPTF0IEN_Msk          (0x1ul << ECAP_CTL0_CAPTF0IEN_Pos)                /*!< ECAP_T::CTL0: CAPTF0IEN Mask           */

#define ECAP_CTL0_CAPTF1IEN_Pos          (17)                                              /*!< ECAP_T::CTL0: CAPTF1IEN Position       */
#define ECAP_CTL0_CAPTF1IEN_Msk          (0x1ul << ECAP_CTL0_CAPTF1IEN_Pos)                /*!< ECAP_T::CTL0: CAPTF1IEN Mask           */

#define ECAP_CTL0_CAPTF2IEN_Pos          (18)                                              /*!< ECAP_T::CTL0: CAPTF2IEN Position       */
#define ECAP_CTL0_CAPTF2IEN_Msk          (0x1ul << ECAP_CTL0_CAPTF2IEN_Pos)                /*!< ECAP_T::CTL0: CAPTF2IEN Mask           */

#define ECAP_CTL0_CAPOVIEN_Pos           (20)                                              /*!< ECAP_T::CTL0: CAPOVIEN Position        */
#define ECAP_CTL0_CAPOVIEN_Msk           (0x1ul << ECAP_CTL0_CAPOVIEN_Pos)                 /*!< ECAP_T::CTL0: CAPOVIEN Mask            */

#define ECAP_CTL0_CAPCMPIEN_Pos          (21)                                              /*!< ECAP_T::CTL0: CAPCMPIEN Position       */
#define ECAP_CTL0_CAPCMPIEN_Msk          (0x1ul << ECAP_CTL0_CAPCMPIEN_Pos)                /*!< ECAP_T::CTL0: CAPCMPIEN Mask           */

#define ECAP_CTL0_CPTST_Pos              (24)                                              /*!< ECAP_T::CTL0: CPTST Position           */
#define ECAP_CTL0_CPTST_Msk              (0x1ul << ECAP_CTL0_CPTST_Pos)                    /*!< ECAP_T::CTL0: CPTST Mask               */

#define ECAP_CTL0_CMPCLR_Pos             (25)                                              /*!< ECAP_T::CTL0: CMPCLR Position          */
#define ECAP_CTL0_CMPCLR_Msk             (0x1ul << ECAP_CTL0_CMPCLR_Pos)                   /*!< ECAP_T::CTL0: CMPCLR Mask              */

#define ECAP_CTL0_CPTCLR_Pos             (26)                                              /*!< ECAP_T::CTL0: CPTCLR Position          */
#define ECAP_CTL0_CPTCLR_Msk             (0x1ul << ECAP_CTL0_CPTCLR_Pos)                   /*!< ECAP_T::CTL0: CPTCLR Mask              */

#define ECAP_CTL0_RLDEN_Pos              (27)                                              /*!< ECAP_T::CTL0: RLDEN Position           */
#define ECAP_CTL0_RLDEN_Msk              (0x1ul << ECAP_CTL0_RLDEN_Pos)                    /*!< ECAP_T::CTL0: RLDEN Mask               */

#define ECAP_CTL0_CMPEN_Pos              (28)                                              /*!< ECAP_T::CTL0: CMPEN Position           */
#define ECAP_CTL0_CMPEN_Msk              (0x1ul << ECAP_CTL0_CMPEN_Pos)                    /*!< ECAP_T::CTL0: CMPEN Mask               */

#define ECAP_CTL0_CAPEN_Pos              (29)                                              /*!< ECAP_T::CTL0: CAPEN Position           */
#define ECAP_CTL0_CAPEN_Msk              (0x1ul << ECAP_CTL0_CAPEN_Pos)                    /*!< ECAP_T::CTL0: CAPEN Mask               */

#define ECAP_CTL0_CAPPHGEN_Pos           (30)                                              /*!< ECAP_T::CTL0: CAPPHGEN Position        */
#define ECAP_CTL0_CAPPHGEN_Msk           (0x1ul << ECAP_CTL0_CAPPHGEN_Pos)                 /*!< ECAP_T::CTL0: CAPPHGEN Mask            */

#define ECAP_CTL1_CAPEDG0_Pos            (0)                                               /*!< ECAP_T::CTL1: CAPEDG0 Position         */
#define ECAP_CTL1_CAPEDG0_Msk            (0x3ul << ECAP_CTL1_CAPEDG0_Pos)                  /*!< ECAP_T::CTL1: CAPEDG0 Mask             */

#define ECAP_CTL1_CAPEDG1_Pos            (2)                                               /*!< ECAP_T::CTL1: CAPEDG1 Position         */
#define ECAP_CTL1_CAPEDG1_Msk            (0x3ul << ECAP_CTL1_CAPEDG1_Pos)                  /*!< ECAP_T::CTL1: CAPEDG1 Mask             */

#define ECAP_CTL1_CAPEDG2_Pos            (4)                                               /*!< ECAP_T::CTL1: CAPEDG2 Position         */
#define ECAP_CTL1_CAPEDG2_Msk            (0x3ul << ECAP_CTL1_CAPEDG2_Pos)                  /*!< ECAP_T::CTL1: CAPEDG2 Mask             */

#define ECAP_CTL1_CPRLDS_Pos             (8)                                               /*!< ECAP_T::CTL1: CPRLDS Position          */
#define ECAP_CTL1_CPRLDS_Msk             (0x7ul << ECAP_CTL1_CPRLDS_Pos)                   /*!< ECAP_T::CTL1: CPRLDS Mask              */

#define ECAP_CTL1_CAPDIV_Pos             (12)                                              /*!< ECAP_T::CTL1: CAPDIV Position          */
#define ECAP_CTL1_CAPDIV_Msk             (0x7ul << ECAP_CTL1_CAPDIV_Pos)                   /*!< ECAP_T::CTL1: CAPDIV Mask              */

#define ECAP_CTL1_CNTSRC_Pos             (16)                                              /*!< ECAP_T::CTL1: CNTSRC Position          */
#define ECAP_CTL1_CNTSRC_Msk             (0x3ul << ECAP_CTL1_CNTSRC_Pos)                   /*!< ECAP_T::CTL1: CNTSRC Mask              */

#define ECAP_STS_CAPTF0_Pos              (0)                                               /*!< ECAP_T::STS: CAPTF0 Position           */
#define ECAP_STS_CAPTF0_Msk              (0x1ul << ECAP_STS_CAPTF0_Pos)                    /*!< ECAP_T::STS: CAPTF0 Mask               */

#define ECAP_STS_CAPTF1_Pos              (1)                                               /*!< ECAP_T::STS: CAPTF1 Position           */
#define ECAP_STS_CAPTF1_Msk              (0x1ul << ECAP_STS_CAPTF1_Pos)                    /*!< ECAP_T::STS: CAPTF1 Mask               */

#define ECAP_STS_CAPTF2_Pos              (2)                                               /*!< ECAP_T::STS: CAPTF2 Position           */
#define ECAP_STS_CAPTF2_Msk              (0x1ul << ECAP_STS_CAPTF2_Pos)                    /*!< ECAP_T::STS: CAPTF2 Mask               */

#define ECAP_STS_CAPCMPF_Pos             (4)                                               /*!< ECAP_T::STS: CAPCMPF Position          */
#define ECAP_STS_CAPCMPF_Msk             (0x1ul << ECAP_STS_CAPCMPF_Pos)                   /*!< ECAP_T::STS: CAPCMPF Mask              */

#define ECAP_STS_CAPOVF_Pos              (5)                                               /*!< ECAP_T::STS: CAPOVF Position           */
#define ECAP_STS_CAPOVF_Msk              (0x1ul << ECAP_STS_CAPOVF_Pos)                    /*!< ECAP_T::STS: CAPOVF Mask               */

#define ECAP_STS_ECAP0_Pos               (8)                                               /*!< ECAP_T::STS: ECAP0 Position            */
#define ECAP_STS_ECAP0_Msk               (0x1ul << ECAP_STS_ECAP0_Pos)                     /*!< ECAP_T::STS: ECAP0 Mask                */

#define ECAP_STS_ECAP1_Pos               (9)                                               /*!< ECAP_T::STS: ECAP1 Position            */
#define ECAP_STS_ECAP1_Msk               (0x1ul << ECAP_STS_ECAP1_Pos)                     /*!< ECAP_T::STS: ECAP1 Mask                */

#define ECAP_STS_ECAP2_Pos               (10)                                              /*!< ECAP_T::STS: ECAP2 Position            */
#define ECAP_STS_ECAP2_Msk               (0x1ul << ECAP_STS_ECAP2_Pos)                     /*!< ECAP_T::STS: ECAP2 Mask                */

/**@}*/ /* ECAP_CONST */
/**@}*/ /* end of ECAP register group */


/*---------------------- Watch Dog Timer Controller -------------------------*/
/**
    @addtogroup WDT Watch Dog Timer Controller(WDT)
    Memory Mapped Structure for WDT Controller
@{ */

typedef struct
{


    /**
     * @var WDT_T::CTL
     * Offset: 0x00  Watchdog Timer Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RSTCNT    |Reset Watchdog Timer Up Counter (Write Protect)
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the internal 18-bit WDT up counter value.
     * |        |          |Note: This bit will be automatically cleared by hardware.
     * |[1]     |RSTEN     |Watchdog Timer Time-Out Reset Enable Control (Write Protect)
     * |        |          |Setting this bit will enable the WDT time-out reset function if the WDT up counter value has not been cleared after the specific WDT reset delay period (1024 * TWDT) expires.
     * |        |          |0 = WDT time-out reset function Disabled.
     * |        |          |1 = WDT time-out reset function Enabled.
     * |[2]     |RSTF      |Watchdog Timer Time-Out Reset Flag
     * |        |          |This bit indicates the system has been reset by WDT time-out reset or not.
     * |        |          |0 = WDT time-out reset did not occur.
     * |        |          |1 = WDT time-out reset occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[3]     |IF        |Watchdog Timer Time-Out Interrupt Flag
     * |        |          |This bit will be set to 1 while WDT up counter value reaches the selected WDT time-out interval.
     * |        |          |0 = WDT time-out interrupt did not occur.
     * |        |          |1 = WDT time-out interrupt occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[4]     |WKEN      |Watchdog Timer Time-Out Wake-Up Function Control (Write Protect)
     * |        |          |If this bit is set to 1, while IF is generated to 1 and INTEN enabled, the WDT time-out interrupt signal will generate a wake-up trigger event to chip.
     * |        |          |0 = Wake-up trigger event Disabled if WDT time-out interrupt signal generated.
     * |        |          |1 = Wake-up trigger event Enabled if WDT time-out interrupt signal generated.
     * |        |          |Note: Chip can be woken-up by WDT time-out interrupt signal generated only if WDT clock source is selected to 10 kHz oscillator.
     * |[5]     |WKF       |Watchdog Timer Time-Out Wake-Up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of WDT.
     * |        |          |0 = WDT does not cause chip wake-up.
     * |        |          |1 = Chip wake-up from Idle or Power-down mode if WDT time-out interrupt signal generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[6]     |INTEN     |Watchdog Timer Time-Out Interrupt Enable Control (Write Protect)
     * |        |          |If this bit is enabled, the WDT time-out interrupt signal is generated and inform to CPU.
     * |        |          |0 = WDT time-out interrupt Disabled.
     * |        |          |1 = WDT time-out interrupt Enabled.
     * |[7]     |WDTEN     |Watchdog Timer Enable Control (Write Protect)
     * |        |          |0 = WDT Disabled. (This action will reset the internal up counter value.)
     * |        |          |1 = WDT Enabled.
     * |[10:8]  |TOUTSEL   |Watchdog Timer Interval Selection
     * |        |          |These three bits select the time-out interval for the Watchdog Timer.
     * |        |          |000 = 24 * TWDT.
     * |        |          |001 = 26 * TWDT.
     * |        |          |010 = 28 * TWDT.
     * |        |          |011 = 210 * TWDT.
     * |        |          |100 = 212 * TWDT.
     * |        |          |101 = 214 * TWDT.
     * |        |          |110 = 216 * TWDT.
     * |        |          |111 = 218 * TWDT.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable Control (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects WDT counting.
     * |        |          |WDT up counter will be kept while CPU is hanging by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |WDT up counter will keep going no matter CPU is hanging by ICE or not.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] Watchdog Timer Control Register                                  */

} WDT_T;

/**
    @addtogroup WDT_CONST WDT Bit Field Definition
    Constant Definitions for WDT Controller
@{ */

#define WDT_CTL_RSTCNT_Pos               (0)                                               /*!< WDT_T::CTL: RSTCNT Position            */
#define WDT_CTL_RSTCNT_Msk               (0x1ul << WDT_CTL_RSTCNT_Pos)                     /*!< WDT_T::CTL: RSTCNT Mask                */

#define WDT_CTL_RSTEN_Pos                (1)                                               /*!< WDT_T::CTL: RSTEN Position             */
#define WDT_CTL_RSTEN_Msk                (0x1ul << WDT_CTL_RSTEN_Pos)                      /*!< WDT_T::CTL: RSTEN Mask                 */

#define WDT_CTL_RSTF_Pos                 (2)                                               /*!< WDT_T::CTL: RSTF Position              */
#define WDT_CTL_RSTF_Msk                 (0x1ul << WDT_CTL_RSTF_Pos)                       /*!< WDT_T::CTL: RSTF Mask                  */

#define WDT_CTL_IF_Pos                   (3)                                               /*!< WDT_T::CTL: IF Position                */
#define WDT_CTL_IF_Msk                   (0x1ul << WDT_CTL_IF_Pos)                         /*!< WDT_T::CTL: IF Mask                    */

#define WDT_CTL_WKEN_Pos                 (4)                                               /*!< WDT_T::CTL: WKEN Position              */
#define WDT_CTL_WKEN_Msk                 (0x1ul << WDT_CTL_WKEN_Pos)                       /*!< WDT_T::CTL: WKEN Mask                  */

#define WDT_CTL_WKF_Pos                  (5)                                               /*!< WDT_T::CTL: WKF Position               */
#define WDT_CTL_WKF_Msk                  (0x1ul << WDT_CTL_WKF_Pos)                        /*!< WDT_T::CTL: WKF Mask                   */

#define WDT_CTL_INTEN_Pos                (6)                                               /*!< WDT_T::CTL: INTEN Position             */
#define WDT_CTL_INTEN_Msk                (0x1ul << WDT_CTL_INTEN_Pos)                      /*!< WDT_T::CTL: INTEN Mask                 */

#define WDT_CTL_WDTEN_Pos                (7)                                               /*!< WDT_T::CTL: WDTEN Position             */
#define WDT_CTL_WDTEN_Msk                (0x1ul << WDT_CTL_WDTEN_Pos)                      /*!< WDT_T::CTL: WDTEN Mask                 */

#define WDT_CTL_TOUTSEL_Pos              (8)                                               /*!< WDT_T::CTL: TOUTSEL Position           */
#define WDT_CTL_TOUTSEL_Msk              (0x7ul << WDT_CTL_TOUTSEL_Pos)                    /*!< WDT_T::CTL: TOUTSEL Mask               */

#define WDT_CTL_ICEDEBUG_Pos             (31)                                              /*!< WDT_T::CTL: ICEDEBUG Position          */
#define WDT_CTL_ICEDEBUG_Msk             (0x1ul << WDT_CTL_ICEDEBUG_Pos)                   /*!< WDT_T::CTL: ICEDEBUG Mask              */

/**@}*/ /* WDT_CONST */
/**@}*/ /* end of WDT register group */


/*---------------------- UART Mode of USCI Controller -------------------------*/
/**
    @addtogroup UUART UART Mode of USCI Controller(UUART)
    Memory Mapped Structure for UUART Controller
@{ */

typedef struct
{


    /**
     * @var UUART_T::CTL
     * Offset: 0x00  USCI Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |FUNMODE   |Function Mode
     * |        |          |This bit field selects the protocol for this USCI controller
     * |        |          |Selecting a protocol that is not available or a reserved combination disables the USCI
     * |        |          |When switching between two protocols, the USCI has to be disabled before selecting a new protocol
     * |        |          |Simultaneously, the USCI will be reset when user write 000 to FUNMODE.
     * |        |          |000 = The USCI is disabled. All protocol related state machines are set to idle state.
     * |        |          |001 = The SPI protocol is selected.
     * |        |          |010 = The UART protocol is selected.
     * |        |          |100 = The I2C protocol is selected.
     * |        |          |Note: Other bit combinations are reserved.
     * @var UUART_T::INTEN
     * Offset: 0x04  USCI Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TXSTIEN   |Transmit Start Interrupt Enable Bit
     * |        |          |This bit enables the interrupt generation in case of a transmit start event.
     * |        |          |0 = The transmit start interrupt is disabled.
     * |        |          |1 = The transmit start interrupt is enabled.
     * |[2]     |TXENDIEN  |Transmit End Interrupt Enable Bit
     * |        |          |This bit enables the interrupt generation in case of a transmit finish event.
     * |        |          |0 = The transmit finish interrupt is disabled.
     * |        |          |1 = The transmit finish interrupt is enabled.
     * |[3]     |RXSTIEN   |Receive Start Interrupt Enable Bit
     * |        |          |This bit enables the interrupt generation in case of a receive start event.
     * |        |          |0 = The receive start interrupt is disabled.
     * |        |          |1 = The receive start interrupt is enabled.
     * |[4]     |RXENDIEN  |Receive End Interrupt Enable Bit
     * |        |          |This bit enables the interrupt generation in case of a receive finish event.
     * |        |          |0 = The receive end interrupt is disabled.
     * |        |          |1 = The receive end interrupt is enabled.
     * @var UUART_T::BRGEN
     * Offset: 0x08  USCI Baud Rate Generator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RCLKSEL   |Reference Clock Source Selection
     * |        |          |This bit selects the source signal of reference clock (fREF_CLK).
     * |        |          |0 = Peripheral device clock fPCLK.
     * |        |          |1 = Reserved.
     * |[1]     |PTCLKSEL  |Protocol Clock Source Selection
     * |        |          |This bit selects the source signal of protocol clock (fPROT_CLK).
     * |        |          |0 = Reference clock fREF_CLK.
     * |        |          |1 = fREF_CLK2 (its frequency is half of fREF_CLK).
     * |[3:2]   |SPCLKSEL  |Sample Clock Source Selection
     * |        |          |This bit field used for the clock source selection of a sample clock (fSAMP_CLK) for the protocol processor.
     * |        |          |00 = fSAMP_CLK = fDIV_CLK.
     * |        |          |01 = fSAMP_CLK = fPROT_CLK.
     * |        |          |10 = fSAMP_CLK = fSCLK.
     * |        |          |11 = fSAMP_CLK = fREF_CLK.
     * |[4]     |TMCNTEN   |Timing Measurement Counter Enable Bit
     * |        |          |This bit enables the 10-bit timing measurement counter.
     * |        |          |0 = Timing measurement counter is Disabled.
     * |        |          |1 = Timing measurement counter is Enabled.
     * |[5]     |TMCNTSRC  |Timing Measurement Counter Clock Source Selection
     * |        |          |0 = Timing measurement counter with fPROT_CLK.
     * |        |          |1 = Timing measurement counter with fDIV_CLK.
     * |[9:8]   |PDSCNT    |Pre-divider for Sample Counter
     * |        |          |This bit field defines the divide ratio of the clock division from sample clock fSAMP_CLK
     * |        |          |The divided frequency fPDS_CNT = fSAMP_CLK / (PDSCNT+1).
     * |[14:10] |DSCNT     |Denominator for Sample Counter
     * |        |          |This bit field defines the divide ratio of the sample clock fSAMP_CLK.
     * |        |          |The divided frequency fDS_CNT = fPDS_CNT / (DSCNT+1).
     * |        |          |Note: The maximum value of DSCNT is 0xF on UART mode and suggest to set over 4 to confirm the receiver data is sampled in right value
     * |[25:16] |CLKDIV    |Clock Divider
     * |        |          |This bit field defines the ratio between the protocol clock frequency fPROT_CLK and the clock divider frequency fDIV_CLK (fDIV_CLK = fPROT_CLK / (CLKDIV+1) ).
     * |        |          |Note: In UART function, it can be updated by hardware in the 4th falling edge of the input data 0x55 when the auto baud rate function (ABREN(UUART_PROTCTL[6])) is enabled
     * |        |          |The revised value is the average bit time between bit 5 and bit 6
     * |        |          |The user can use revised CLKDIV and new BRDETITV (UUART_PROTCTL[24:16]) to calculate the precise baud rate.
     * @var UUART_T::DATIN0
     * Offset: 0x10  USCI Input Data Signal Configuration Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SYNCSEL   |Input Signal   Synchronization Selection
     * |        |          |This bit   selects if the un-synchronized input signal (with optionally inverted) or the   synchronized (and optionally filtered) signal can be used as input for the   data shift unit.
     * |        |          |0 = The un-synchronized   signal can be taken as input for the data shift unit.
     * |        |          |1 = The   synchronized signal can be taken as input for the data shift unit.
     * |[2]     |ININV     |Input   Signal Inverse Selection
     * |        |          |This bit   defines the inverter enable of the input asynchronous signal.
     * |        |          |0 = The   un-synchronized input signal will not be inverted.
     * |        |          |1 = The   un-synchronized input signal will be inverted.
     * |[4:3]   |EDGEDET   |Input Signal   Edge Detection Mode
     * |        |          |This bit   field selects which edge actives the trigger event of input data signal.
     * |        |          |00 = The   trigger event activation is disabled.
     * |        |          |01 = A rising   edge activates the trigger event of input data signal.
     * |        |          |10 = A   falling edge activates the trigger event of input data signal.
     * |        |          |11 = Both   edges activate the trigger event of input data signal.
     * |        |          |Note: In UART function mode, it is suggested to   set this bit field as 10.
     * @var UUART_T::CTLIN0
     * Offset: 0x20  USCI Input Control Signal Configuration Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SYNCSEL   |Input   Synchronization Signal Selection
     * |        |          |This bit   selects if the un-synchronized input signal (with optionally inverted) or the   synchronized (and optionally filtered) signal can be used as input for the   data shift unit.
     * |        |          |0 = The   un-synchronized signal can be taken as input for the data shift unit.
     * |        |          |1 = The   synchronized signal can be taken as input for the data shift unit.
     * |[2]     |ININV     |Input   Signal Inverse Selection
     * |        |          |This bit   defines the inverter enable of the input asynchronous signal.
     * |        |          |0 = The   un-synchronized input signal will not be inverted.
     * |        |          |1 = The   un-synchronized input signal will be inverted.
     * @var UUART_T::CLKIN
     * Offset: 0x28  USCI Input Clock Signal Configuration Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SYNCSEL   |Input   Synchronization Signal Selection
     * |        |          |This bit   selects if the un-synchronized input signal or the synchronized (and   optionally filtered) signal can be used as input for the data shift unit.
     * |        |          |0 = The   un-synchronized signal can be taken as input for the data shift unit.
     * |        |          |1 = The   synchronized signal can be taken as input for the data shift unit.
     * @var UUART_T::LINECTL
     * Offset: 0x2C  USCI Line Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LSB       |LSB First Transmission Selection
     * |        |          |0 = The MSB, which bit of transmit/receive data buffer depends on the setting of DWIDTH, is transmitted/received first.
     * |        |          |1 = The LSB, the bit 0 of data buffer, will be transmitted/received first.
     * |[5]     |DATOINV   |Data Output Inverse Selection
     * |        |          |This bit defines the relation between the internal shift data value and the output data signal of USCIx_DAT1 pin.
     * |        |          |0 = The value of USCIx_DAT1 is equal to the data shift register.
     * |        |          |1 = The value of USCIx_DAT1 is the inversion of data shift register.
     * |[7]     |CTLOINV   |Control Signal Output Inverse Selection
     * |        |          |This bit defines the relation between the internal control signal and the output control signal.
     * |        |          |0 = No effect.
     * |        |          |1 = The control signal will be inverted before its output.
     * |        |          |Note: In UART protocol, the control signal means nRTS signal.
     * |[11:8]  |DWIDTH    |Word Length of Transmission
     * |        |          |This bit field defines the data word length (amount of bits) for reception and transmission
     * |        |          |The data word is always right-aligned in the data buffer
     * |        |          |USCI support word length from 4 to 16 bits.
     * |        |          |0x0: The data word contains 16 bits located at bit positions [15:0].
     * |        |          |0x1: Reserved.
     * |        |          |0x2: Reserved.
     * |        |          |0x3: Reserved.
     * |        |          |0x4: The data word contains 4 bits located at bit positions [3:0].
     * |        |          |0x5: The data word contains 5 bits located at bit positions [4:0].
     * |        |          |...
     * |        |          |0xF: The data word contains 15 bits located at bit positions [14:0].
     * |        |          |Note: In UART protocol, the length can be configured as 6~13 bits.
     * @var UUART_T::TXDAT
     * Offset: 0x30  USCI Transmit Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |TXDAT     |Transmit Data
     * |        |          |Software can use this bit field to write 16-bit transmit data for transmission.
     * @var UUART_T::RXDAT
     * Offset: 0x34  USCI Receive Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RXDAT     |Received Data
     * |        |          |This bit field monitors the received data which stored in receive data buffer.
     * |        |          |Note: RXDAT[15:13] indicate the same frame status of BREAK, FRMERR and PARITYERR (UUART_PROTSTS[7:5]).
     * @var UUART_T::BUFCTL
     * Offset: 0x38  USCI Transmit/Receive Buffer Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7]     |TXCLR     |Clear Transmit Buffer
     * |        |          |0 = No effect.
     * |        |          |1 = The transmit buffer is cleared (filling level is cleared and output pointer is set to input pointer value)
     * |        |          |Should only be used while the buffer is not taking part in data traffic.
     * |        |          |Note: It is cleared automatically after one PCLK cycle.
     * |[14]    |RXOVIEN   |Receive Buffer Overrun Error Interrupt Enable Control
     * |        |          |0 = Receive overrun interrupt Disabled.
     * |        |          |1 = Receive overrun interrupt Enabled.
     * |[15]    |RXCLR     |Clear Receive Buffer
     * |        |          |0 = No effect.
     * |        |          |1 = The receive buffer is cleared (filling level is cleared and output pointer is set to input pointer value)
     * |        |          |Should only be used while the buffer is not taking part in data traffic.
     * |        |          |Note: It is cleared automatically after one PCLK cycle.
     * |[16]    |TXRST     |Transmit Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the transmit-related counters, state machine, and the content of transmit shift register and data buffer.
     * |        |          |Note: It is cleared automatically after one PCLK cycle.
     * |[17]    |RXRST     |Receive Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the receive-related counters, state machine, and the content of receive shift register and data buffer.
     * |        |          |Note 1: It is cleared automatically after one PCLK cycle.
     * |        |          |Note 2: It is suggest to check the RXBUSY (UUART_PROTSTS[10]) before this bit will be set to 1.
     * @var UUART_T::BUFSTS
     * Offset: 0x3C  USCI Transmit/Receive Buffer Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXEMPTY   |Receive Buffer Empty Indicator
     * |        |          |0 = Receive buffer is not empty.
     * |        |          |1 = Receive buffer is empty.
     * |[1]     |RXFULL    |Receive Buffer Full Indicator
     * |        |          |0 = Receive buffer is not full.
     * |        |          |1 = Receive buffer is full.
     * |[3]     |RXOVIF    |Receive Buffer Over-run Error Interrupt Status
     * |        |          |This bit indicates that a receive buffer overrun error event has been detected
     * |        |          |If RXOVIEN (UUART_BUFCTL[14]) is enabled, the corresponding interrupt request is activated
     * |        |          |It is cleared by software writes 1 to this bit.
     * |        |          |0 = A receive buffer overrun error event has not been detected.
     * |        |          |1 = A receive buffer overrun error event has been detected.
     * |[8]     |TXEMPTY   |Transmit Buffer Empty Indicator
     * |        |          |0 = Transmit buffer is not empty.
     * |        |          |1 = Transmit buffer is empty.
     * |[9]     |TXFULL    |Transmit Buffer Full Indicator
     * |        |          |0 = Transmit buffer is not full.
     * |        |          |1 = Transmit buffer is full.
     * @var UUART_T::WKCTL
     * Offset: 0x54  USCI Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN      |Wake-up Enable Bit
     * |        |          |0 = Wake-up function Disabled.
     * |        |          |1 = Wake-up function Enabled.
     * |[2]     |PDBOPT    |Power Down Blocking Option
     * |        |          |0 = If user attempts to enter Power-down mode by executing WFI while the protocol is in transferring, MCU will stop the transfer and enter Power-down mode immediately.
     * |        |          |1 = If user attempts to enter Power-down mode by executing WFI while the protocol is in transferring, the on-going transfer will not be stopped and MCU will enter idle mode immediately.
     * @var UUART_T::WKSTS
     * Offset: 0x58  USCI Wake-up Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKF       |Wake-up Flag
     * |        |          |When chip is woken up from Power-down mode, this bit is set to 1
     * |        |          |Software can write 1 to clear this bit.
     * @var UUART_T::PROTCTL
     * Offset: 0x5C  USCI Protocol Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |STOPB     |Stop Bits
     * |        |          |This bit defines the number of stop bits in an UART frame.
     * |        |          |0 = The number of stop bits is 1.
     * |        |          |1 = The number of stop bits is 2.
     * |[1]     |PARITYEN  |Parity Enable Bit
     * |        |          |This bit defines the parity bit is enabled in an UART frame.
     * |        |          |0 = The parity bit Disabled.
     * |        |          |1 = The parity bit Enabled.
     * |[2]     |EVENPARITY|Even Parity Enable Bit
     * |        |          |0 = Odd number of logic 1's is transmitted and checked in each word.
     * |        |          |1 = Even number of logic 1's is transmitted and checked in each word.
     * |        |          |Note: This bit has effect only when PARITYEN is set.
     * |[6]     |ABREN     |Auto-baud Rate Detect Enable Bit
     * |        |          |0 = Auto-baud rate detect function Disabled.
     * |        |          |1 = Auto-baud rate detect function Enabled.
     * |        |          |Note: When the auto - baud rate detect operation finishes, hardware will clear this bit
     * |        |          |The associated interrupt ABRDETIF (USCI_PROTST[9]) will be generated (If ARBIEN (UUART_PROTIEN [1]) is enabled).
     * |[7]     |LINBRKEN     |LIN TX Break Mode Enable Control
     * |        |          |0 = LIN TX Break mode Disabled.
     * |        |          |1 = LIN TX Break mode Enabled.
     * |        |          |Note 1: When TX break field transfer operation finished, this bit will be cleared automatically.
     * |        |          |Note 2: 13-bit level 0 and 1-bit level 1 were sent out before the 1st data be transmitted.
     * |[8]     |LINRXEN     |LIN RX Duplex Mode Enable Control
     * |        |          |0 = LIN RX Duplex mode Disabled.
     * |        |          |1 = LIN RX Duplex mode Enabled. The LIN can be play as Slave to receive the LIN frame.
     * |        |          |Note: This bit is used to check the break duration for incoming data when the LIN operation is active.
     * |[9]     |DATWKEN   |Data Wake-up Mode Enable Bit
     * |        |          |0 = Data wake-up mode Disabled.
     * |        |          |1 = Data wake-up mode Enabled.
     * |[14:11] |WAKECNT   |Wake-up Counter
     * |        |          |These bits field indicate how many clock cycle selected by fPDS_CNT do the slave can get the 1st bit (start bit) when the device is wake-up from Power-down mode.
     * |[24:16] |BRDETITV  |Baud Rate Detection Interval
     * |        |          |This bit fields indicate how many clock cycle selected by TMCNTSRC (UUART_BRGEN [5]) does the slave calculates the baud rate in one bits
     * |        |          |The order of the bus shall be 1 and 0 step by step (e.g
     * |        |          |the input data pattern shall be 0x55)
     * |        |          |The user can read the value to know the current input baud rate of the bus whenever the ABRDETIF (UUART_PROTCTL[9]) is set.
     * |        |          |Note: This bit can be cleared to 0 by software writing '0' to the BRDETITV.
     * |[26]    |STICKEN   |Stick Parity Enable Bit
     * |        |          |0 = Stick parity Disabled.
     * |        |          |1 = Stick parity Enabled.
     * |        |          |Note: Refer to RS-485 Support section for detail information.
     * |[29]    |BCEN      |Transmit Break Control Enable Bit
     * |        |          |0 = Transmit Break Control Disabled.
     * |        |          |1 = Transmit Break Control Enabled.
     * |        |          |Note: When this bit is set to logic 1, the serial data output (TX) is forced to the Spacing State (logic 0)
     * |        |          |This bit acts only on TX line and has no effect on the transmitter logic.
     * |[31]    |PROTEN    |UART Protocol Enable Bit
     * |        |          |0 = UART Protocol Disabled.
     * |        |          |1 = UART Protocol Enabled.
     * @var UUART_T::PROTIEN
     * Offset: 0x60  USCI Protocol Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BRKIEN    |LIN Break Detected Interrupt Enable Control
     * |        |          |0 = The LIN break detected interrupt generation is Disabled.
     * |        |          |1 = The LIN break detected interrupt generation is Enabled.
     * |[1]     |ABRIEN    |Auto-baud Rate Interrupt Enable Bit
     * |        |          |0 = Auto-baud rate interrupt Disabled.
     * |        |          |1 = Auto-baud rate interrupt Enabled.
     * |[2]     |RLSIEN    |Receive Line Status Interrupt Enable Bit
     * |        |          |0 = Receive line status interrupt Disabled.
     * |        |          |1 = Receive line status interrupt Enabled.
     * |        |          |Note: UUART_PROTSTS[7:5] indicates the current interrupt event for receive line status interrupt.
     * @var UUART_T::PROTSTS
     * Offset: 0x64  USCI Protocol Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TXSTIF    |Transmit Start Interrupt Flag
     * |        |          |0 = A transmit start interrupt status has not occurred.
     * |        |          |1 = A transmit start interrupt status has occurred.
     * |        |          |Note 1: It is cleared by software writing one into this bit.
     * |        |          |Note 2: Used for user to load next transmit data when there is no data in transmit buffer.
     * |[2]     |TXENDIF   |Transmit End Interrupt Flag
     * |        |          |0 = A transmit end interrupt status has not occurred.
     * |        |          |1 = A transmit end interrupt status has occurred.
     * |        |          |Note: It is cleared by software writing one into this bit.
     * |[3]     |RXSTIF    |Receive Start Interrupt Flag
     * |        |          |0 = A receive start interrupt status has not occurred.
     * |        |          |1 = A receive start interrupt status has occurred.
     * |        |          |Note: It is cleared by software writing one into this bit.
     * |[4]     |RXENDIF   |Receive End Interrupt Flag
     * |        |          |0 = A receive finish interrupt status has not occurred.
     * |        |          |1 = A receive finish interrupt status has occurred.
     * |        |          |Note: It is cleared by software writing one into this bit.
     * |[5]     |PARITYERR |Parity Error Flag
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "parity bit".
     * |        |          |0 = No parity error is generated.
     * |        |          |1 = Parity error is generated.
     * |        |          |Note: This bit can be cleared by write "1" among the BREAK, FRMERR and PARITYERR bits.
     * |[6]     |FRMERR    |Framing Error Flag
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "stop bit" (that is, the stop bit following the last data bit or parity bit is detected as logic 0).
     * |        |          |0 = No framing error is generated.
     * |        |          |1 = Framing error is generated.
     * |        |          |Note: This bit can be cleared by write "1" among the BREAK, FRMERR and PARITYERR bits.
     * |[7]     |BREAK     |Break Flag
     * |        |          |This bit is set to logic 1 whenever the received data input (RX) is held in the "spacing state" (logic 0) for longer than a full word transmission time (that is, the total time of "start bit" + data bits + parity + stop bits).
     * |        |          |0 = No Break is generated.
     * |        |          |1 = Break is generated in the receiver bus.
     * |        |          |Note: This bit can be cleared by write "1" among the BREAK, FRMERR and PARITYERR bits.
     * |[8]     |BRKDETIF  |LIN Break Detected Interrupt Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received data input (RX) is held in the "spacing state" (logic 0) for longer than 12-bit transmission time in LIN mode function.
     * |        |          |0 = LIN Break is no detected.
     * |        |          |1 = LIN Break is detected.
     * |        |          |Note: This bit is read only, but can be cleared by writing '1' to it.
     * |[9]     |ABRDETIF  |Auto-baud Rate Interrupt Flag
     * |        |          |This bit is set when auto-baud rate detection is done among the falling edge of the input data
     * |        |          |If the ABRIEN (UUART_PROTCTL[6]) is set, the auto-baud rate interrupt will be generated
     * |        |          |This bit can be set 4 times when the input data pattern is 0x55 and it is cleared before the next falling edge of the input bus.
     * |        |          |0 = Auto-baud rate detect function is not done.
     * |        |          |1 = One Bit auto-baud rate detect function is done.
     * |        |          |Note: This bit can be cleared by writing "1" to it.
     * |[10]    |RXBUSY    |RX Bus Status Flag (Read Only)
     * |        |          |This bit indicates the busy status of the receiver.
     * |        |          |0 = The receiver is Idle.
     * |        |          |1 = The receiver is BUSY.
     * |[11]    |ABERRSTS  |Auto-baud Rate Error Status
     * |        |          |This bit is set when auto-baud rate detection counter overrun
     * |        |          |When the auto-baud rate counter overrun, the user shall revise the CLKDIV (UUART_BRGEN[25:16]) value and enable ABREN (UUART_PROTCTL[6]) to detect the correct baud rate again.
     * |        |          |0 = Auto-baud rate detect counter is not overrun.
     * |        |          |1 = Auto-baud rate detect counter is overrun.
     * |        |          |Note 1: This bit is set at the same time of ABRDETIF.
     * |        |          |Note 2: This bit can be cleared by writing "1" to ABRDETIF or ABERRSTS.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] USCI Control Register                                            */
    __IO uint32_t INTEN;                 /*!< [0x0004] USCI Interrupt Enable Register                                   */
    __IO uint32_t BRGEN;                 /*!< [0x0008] USCI Baud Rate Generator Register                                */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t DATIN0;                /*!< [0x0010] USCI Input Data Signal Configuration Register 0                  */
    __I  uint32_t RESERVE1[3];
    __IO uint32_t CTLIN0;                /*!< [0x0020] USCI Input Control Signal Configuration Register 0               */
    __I  uint32_t RESERVE2[1];
    __IO uint32_t CLKIN;                 /*!< [0x0028] USCI Input Clock Signal Configuration Register                   */
    __IO uint32_t LINECTL;               /*!< [0x002c] USCI Line Control Register                                       */
    __O  uint32_t TXDAT;                 /*!< [0x0030] USCI Transmit Data Register                                      */
    __I  uint32_t RXDAT;                 /*!< [0x0034] USCI Receive Data Register                                       */
    __IO uint32_t BUFCTL;                /*!< [0x0038] USCI Transmit/Receive Buffer Control Register                    */
    __IO  uint32_t BUFSTS;                /*!< [0x003c] USCI Transmit/Receive Buffer Status Register                     */
    __I  uint32_t RESERVE3[5];
    __IO uint32_t WKCTL;                 /*!< [0x0054] USCI Wake-up Control Register                                    */
    __IO uint32_t WKSTS;                 /*!< [0x0058] USCI Wake-up Status Register                                     */
    __IO uint32_t PROTCTL;               /*!< [0x005c] USCI Protocol Control Register                                   */
    __IO uint32_t PROTIEN;               /*!< [0x0060] USCI Protocol Interrupt Enable Register                          */
    __IO uint32_t PROTSTS;               /*!< [0x0064] USCI Protocol Status Register                                    */

} UUART_T;

/**
    @addtogroup UUART_CONST UUART Bit Field Definition
    Constant Definitions for UUART Controller
@{ */

#define UUART_CTL_FUNMODE_Pos            (0)                                               /*!< UUART_T::CTL: FUNMODE Position         */
#define UUART_CTL_FUNMODE_Msk            (0x7ul << UUART_CTL_FUNMODE_Pos)                  /*!< UUART_T::CTL: FUNMODE Mask             */

#define UUART_INTEN_TXSTIEN_Pos          (1)                                               /*!< UUART_T::INTEN: TXSTIEN Position       */
#define UUART_INTEN_TXSTIEN_Msk          (0x1ul << UUART_INTEN_TXSTIEN_Pos)                /*!< UUART_T::INTEN: TXSTIEN Mask           */

#define UUART_INTEN_TXENDIEN_Pos         (2)                                               /*!< UUART_T::INTEN: TXENDIEN Position      */
#define UUART_INTEN_TXENDIEN_Msk         (0x1ul << UUART_INTEN_TXENDIEN_Pos)               /*!< UUART_T::INTEN: TXENDIEN Mask          */

#define UUART_INTEN_RXSTIEN_Pos          (3)                                               /*!< UUART_T::INTEN: RXSTIEN Position       */
#define UUART_INTEN_RXSTIEN_Msk          (0x1ul << UUART_INTEN_RXSTIEN_Pos)                /*!< UUART_T::INTEN: RXSTIEN Mask           */

#define UUART_INTEN_RXENDIEN_Pos         (4)                                               /*!< UUART_T::INTEN: RXENDIEN Position      */
#define UUART_INTEN_RXENDIEN_Msk         (0x1ul << UUART_INTEN_RXENDIEN_Pos)               /*!< UUART_T::INTEN: RXENDIEN Mask          */

#define UUART_BRGEN_RCLKSEL_Pos          (0)                                               /*!< UUART_T::BRGEN: RCLKSEL Position       */
#define UUART_BRGEN_RCLKSEL_Msk          (0x1ul << UUART_BRGEN_RCLKSEL_Pos)                /*!< UUART_T::BRGEN: RCLKSEL Mask           */

#define UUART_BRGEN_PTCLKSEL_Pos         (1)                                               /*!< UUART_T::BRGEN: PTCLKSEL Position      */
#define UUART_BRGEN_PTCLKSEL_Msk         (0x1ul << UUART_BRGEN_PTCLKSEL_Pos)               /*!< UUART_T::BRGEN: PTCLKSEL Mask          */

#define UUART_BRGEN_SPCLKSEL_Pos         (2)                                               /*!< UUART_T::BRGEN: SPCLKSEL Position      */
#define UUART_BRGEN_SPCLKSEL_Msk         (0x3ul << UUART_BRGEN_SPCLKSEL_Pos)               /*!< UUART_T::BRGEN: SPCLKSEL Mask          */

#define UUART_BRGEN_TMCNTEN_Pos          (4)                                               /*!< UUART_T::BRGEN: TMCNTEN Position       */
#define UUART_BRGEN_TMCNTEN_Msk          (0x1ul << UUART_BRGEN_TMCNTEN_Pos)                /*!< UUART_T::BRGEN: TMCNTEN Mask           */

#define UUART_BRGEN_TMCNTSRC_Pos         (5)                                               /*!< UUART_T::BRGEN: TMCNTSRC Position      */
#define UUART_BRGEN_TMCNTSRC_Msk         (0x1ul << UUART_BRGEN_TMCNTSRC_Pos)               /*!< UUART_T::BRGEN: TMCNTSRC Mask          */

#define UUART_BRGEN_PDSCNT_Pos           (8)                                               /*!< UUART_T::BRGEN: PDSCNT Position        */
#define UUART_BRGEN_PDSCNT_Msk           (0x3ul << UUART_BRGEN_PDSCNT_Pos)                 /*!< UUART_T::BRGEN: PDSCNT Mask            */

#define UUART_BRGEN_DSCNT_Pos            (10)                                              /*!< UUART_T::BRGEN: DSCNT Position         */
#define UUART_BRGEN_DSCNT_Msk            (0x1ful << UUART_BRGEN_DSCNT_Pos)                 /*!< UUART_T::BRGEN: DSCNT Mask             */

#define UUART_BRGEN_CLKDIV_Pos           (16)                                              /*!< UUART_T::BRGEN: CLKDIV Position        */
#define UUART_BRGEN_CLKDIV_Msk           (0x3fful << UUART_BRGEN_CLKDIV_Pos)               /*!< UUART_T::BRGEN: CLKDIV Mask            */

#define UUART_DATIN0_SYNCSEL_Pos         (0)                                               /*!< UUART_T::DATIN0: SYNCSEL Position      */
#define UUART_DATIN0_SYNCSEL_Msk         (0x1ul << UUART_DATIN0_SYNCSEL_Pos)               /*!< UUART_T::DATIN0: SYNCSEL Mask          */

#define UUART_DATIN0_ININV_Pos           (2)                                               /*!< UUART_T::DATIN0: ININV Position        */
#define UUART_DATIN0_ININV_Msk           (0x1ul << UUART_DATIN0_ININV_Pos)                 /*!< UUART_T::DATIN0: ININV Mask            */

#define UUART_DATIN0_EDGEDET_Pos         (3)                                               /*!< UUART_T::DATIN0: EDGEDET Position      */
#define UUART_DATIN0_EDGEDET_Msk         (0x3ul << UUART_DATIN0_EDGEDET_Pos)               /*!< UUART_T::DATIN0: EDGEDET Mask          */

#define UUART_CTLIN0_SYNCSEL_Pos         (0)                                               /*!< UUART_T::CTLIN0: SYNCSEL Position      */
#define UUART_CTLIN0_SYNCSEL_Msk         (0x1ul << UUART_CTLIN0_SYNCSEL_Pos)               /*!< UUART_T::CTLIN0: SYNCSEL Mask          */

#define UUART_CTLIN0_ININV_Pos           (2)                                               /*!< UUART_T::CTLIN0: ININV Position        */
#define UUART_CTLIN0_ININV_Msk           (0x1ul << UUART_CTLIN0_ININV_Pos)                 /*!< UUART_T::CTLIN0: ININV Mask            */

#define UUART_CLKIN_SYNCSEL_Pos          (0)                                               /*!< UUART_T::CLKIN: SYNCSEL Position       */
#define UUART_CLKIN_SYNCSEL_Msk          (0x1ul << UUART_CLKIN_SYNCSEL_Pos)                /*!< UUART_T::CLKIN: SYNCSEL Mask           */

#define UUART_LINECTL_LSB_Pos            (0)                                               /*!< UUART_T::LINECTL: LSB Position         */
#define UUART_LINECTL_LSB_Msk            (0x1ul << UUART_LINECTL_LSB_Pos)                  /*!< UUART_T::LINECTL: LSB Mask             */

#define UUART_LINECTL_DATOINV_Pos        (5)                                               /*!< UUART_T::LINECTL: DATOINV Position     */
#define UUART_LINECTL_DATOINV_Msk        (0x1ul << UUART_LINECTL_DATOINV_Pos)              /*!< UUART_T::LINECTL: DATOINV Mask         */

#define UUART_LINECTL_CTLOINV_Pos        (7)                                               /*!< UUART_T::LINECTL: CTLOINV Position     */
#define UUART_LINECTL_CTLOINV_Msk        (0x1ul << UUART_LINECTL_CTLOINV_Pos)              /*!< UUART_T::LINECTL: CTLOINV Mask         */

#define UUART_LINECTL_DWIDTH_Pos         (8)                                               /*!< UUART_T::LINECTL: DWIDTH Position      */
#define UUART_LINECTL_DWIDTH_Msk         (0xful << UUART_LINECTL_DWIDTH_Pos)               /*!< UUART_T::LINECTL: DWIDTH Mask          */

#define UUART_TXDAT_TXDAT_Pos            (0)                                               /*!< UUART_T::TXDAT: TXDAT Position         */
#define UUART_TXDAT_TXDAT_Msk            (0xfffful << UUART_TXDAT_TXDAT_Pos)               /*!< UUART_T::TXDAT: TXDAT Mask             */

#define UUART_RXDAT_RXDAT_Pos            (0)                                               /*!< UUART_T::RXDAT: RXDAT Position         */
#define UUART_RXDAT_RXDAT_Msk            (0xfffful << UUART_RXDAT_RXDAT_Pos)               /*!< UUART_T::RXDAT: RXDAT Mask             */

#define UUART_BUFCTL_TXCLR_Pos           (7)                                               /*!< UUART_T::BUFCTL: TXCLR Position        */
#define UUART_BUFCTL_TXCLR_Msk           (0x1ul << UUART_BUFCTL_TXCLR_Pos)                 /*!< UUART_T::BUFCTL: TXCLR Mask            */

#define UUART_BUFCTL_RXOVIEN_Pos         (14)                                              /*!< UUART_T::BUFCTL: RXOVIEN Position      */
#define UUART_BUFCTL_RXOVIEN_Msk         (0x1ul << UUART_BUFCTL_RXOVIEN_Pos)               /*!< UUART_T::BUFCTL: RXOVIEN Mask          */

#define UUART_BUFCTL_RXCLR_Pos           (15)                                              /*!< UUART_T::BUFCTL: RXCLR Position        */
#define UUART_BUFCTL_RXCLR_Msk           (0x1ul << UUART_BUFCTL_RXCLR_Pos)                 /*!< UUART_T::BUFCTL: RXCLR Mask            */

#define UUART_BUFCTL_TXRST_Pos           (16)                                              /*!< UUART_T::BUFCTL: TXRST Position        */
#define UUART_BUFCTL_TXRST_Msk           (0x1ul << UUART_BUFCTL_TXRST_Pos)                 /*!< UUART_T::BUFCTL: TXRST Mask            */

#define UUART_BUFCTL_RXRST_Pos           (17)                                              /*!< UUART_T::BUFCTL: RXRST Position        */
#define UUART_BUFCTL_RXRST_Msk           (0x1ul << UUART_BUFCTL_RXRST_Pos)                 /*!< UUART_T::BUFCTL: RXRST Mask            */

#define UUART_BUFSTS_RXEMPTY_Pos         (0)                                               /*!< UUART_T::BUFSTS: RXEMPTY Position      */
#define UUART_BUFSTS_RXEMPTY_Msk         (0x1ul << UUART_BUFSTS_RXEMPTY_Pos)               /*!< UUART_T::BUFSTS: RXEMPTY Mask          */

#define UUART_BUFSTS_RXFULL_Pos          (1)                                               /*!< UUART_T::BUFSTS: RXFULL Position       */
#define UUART_BUFSTS_RXFULL_Msk          (0x1ul << UUART_BUFSTS_RXFULL_Pos)                /*!< UUART_T::BUFSTS: RXFULL Mask           */

#define UUART_BUFSTS_RXOVIF_Pos          (3)                                               /*!< UUART_T::BUFSTS: RXOVIF Position       */
#define UUART_BUFSTS_RXOVIF_Msk          (0x1ul << UUART_BUFSTS_RXOVIF_Pos)                /*!< UUART_T::BUFSTS: RXOVIF Mask           */

#define UUART_BUFSTS_TXEMPTY_Pos         (8)                                               /*!< UUART_T::BUFSTS: TXEMPTY Position      */
#define UUART_BUFSTS_TXEMPTY_Msk         (0x1ul << UUART_BUFSTS_TXEMPTY_Pos)               /*!< UUART_T::BUFSTS: TXEMPTY Mask          */

#define UUART_BUFSTS_TXFULL_Pos          (9)                                               /*!< UUART_T::BUFSTS: TXFULL Position       */
#define UUART_BUFSTS_TXFULL_Msk          (0x1ul << UUART_BUFSTS_TXFULL_Pos)                /*!< UUART_T::BUFSTS: TXFULL Mask           */

#define UUART_WKCTL_WKEN_Pos             (0)                                               /*!< UUART_T::WKCTL: WKEN Position          */
#define UUART_WKCTL_WKEN_Msk             (0x1ul << UUART_WKCTL_WKEN_Pos)                   /*!< UUART_T::WKCTL: WKEN Mask              */

#define UUART_WKCTL_PDBOPT_Pos           (2)                                               /*!< UUART_T::WKCTL: PDBOPT Position        */
#define UUART_WKCTL_PDBOPT_Msk           (0x1ul << UUART_WKCTL_PDBOPT_Pos)                 /*!< UUART_T::WKCTL: PDBOPT Mask            */

#define UUART_WKSTS_WKF_Pos              (0)                                               /*!< UUART_T::WKSTS: WKF Position           */
#define UUART_WKSTS_WKF_Msk              (0x1ul << UUART_WKSTS_WKF_Pos)                    /*!< UUART_T::WKSTS: WKF Mask               */

#define UUART_PROTCTL_STOPB_Pos          (0)                                               /*!< UUART_T::PROTCTL: STOPB Position       */
#define UUART_PROTCTL_STOPB_Msk          (0x1ul << UUART_PROTCTL_STOPB_Pos)                /*!< UUART_T::PROTCTL: STOPB Mask           */

#define UUART_PROTCTL_PARITYEN_Pos       (1)                                               /*!< UUART_T::PROTCTL: PARITYEN Position    */
#define UUART_PROTCTL_PARITYEN_Msk       (0x1ul << UUART_PROTCTL_PARITYEN_Pos)             /*!< UUART_T::PROTCTL: PARITYEN Mask        */

#define UUART_PROTCTL_EVENPARITY_Pos     (2)                                               /*!< UUART_T::PROTCTL: EVENPARITY Position  */
#define UUART_PROTCTL_EVENPARITY_Msk     (0x1ul << UUART_PROTCTL_EVENPARITY_Pos)           /*!< UUART_T::PROTCTL: EVENPARITY Mask      */

#define UUART_PROTCTL_ABREN_Pos          (6)                                               /*!< UUART_T::PROTCTL: ABREN Position       */
#define UUART_PROTCTL_ABREN_Msk          (0x1ul << UUART_PROTCTL_ABREN_Pos)                /*!< UUART_T::PROTCTL: ABREN Mask           */

#define UUART_PROTCTL_LINBRKEN_Pos      (7)                                               /*!< UUART_T::PROTCTL: LINBRKEN Position   */
#define UUART_PROTCTL_LINBRKEN_Msk      (0x1ul << UUART_PROTCTL_LINBRKEN_Pos)            /*!< UUART_T::PROTCTL: LINBRKEN Mask       */

#define UUART_PROTCTL_LINRXEN_Pos     (8)                                               /*!< UUART_T::PROTCTL: LINRXEN Position  */
#define UUART_PROTCTL_LINRXEN_Msk     (0x1ul << UUART_PROTCTL_LINRXEN_Pos)           /*!< UUART_T::PROTCTL: LINRXEN Mask      */

#define UUART_PROTCTL_DATWKEN_Pos        (9)                                               /*!< UUART_T::PROTCTL: DATWKEN Position     */
#define UUART_PROTCTL_DATWKEN_Msk        (0x1ul << UUART_PROTCTL_DATWKEN_Pos)              /*!< UUART_T::PROTCTL: DATWKEN Mask         */

#define UUART_PROTCTL_WAKECNT_Pos        (11)                                              /*!< UUART_T::PROTCTL: WAKECNT Position     */
#define UUART_PROTCTL_WAKECNT_Msk        (0xful << UUART_PROTCTL_WAKECNT_Pos)              /*!< UUART_T::PROTCTL: WAKECNT Mask         */

#define UUART_PROTCTL_BRDETITV_Pos       (16)                                              /*!< UUART_T::PROTCTL: BRDETITV Position    */
#define UUART_PROTCTL_BRDETITV_Msk       (0x1fful << UUART_PROTCTL_BRDETITV_Pos)           /*!< UUART_T::PROTCTL: BRDETITV Mask        */

#define UUART_PROTCTL_STICKEN_Pos        (26)                                              /*!< UUART_T::PROTCTL: STICKEN Position     */
#define UUART_PROTCTL_STICKEN_Msk        (0x1ul << UUART_PROTCTL_STICKEN_Pos)              /*!< UUART_T::PROTCTL: STICKEN Mask         */

#define UUART_PROTCTL_BCEN_Pos           (29)                                              /*!< UUART_T::PROTCTL: BCEN Position        */
#define UUART_PROTCTL_BCEN_Msk           (0x1ul << UUART_PROTCTL_BCEN_Pos)                 /*!< UUART_T::PROTCTL: BCEN Mask            */

#define UUART_PROTCTL_PROTEN_Pos         (31)                                              /*!< UUART_T::PROTCTL: PROTEN Position      */
#define UUART_PROTCTL_PROTEN_Msk         (0x1ul << UUART_PROTCTL_PROTEN_Pos)               /*!< UUART_T::PROTCTL: PROTEN Mask          */

#define UUART_PROTIEN_BRKIEN_Pos         (0)                                               /*!< UUART_T::PROTIEN: BRKIEN Position      */
#define UUART_PROTIEN_BRKIEN_Msk         (0x1ul << UUART_PROTIEN_BRKIEN_Pos)               /*!< UUART_T::PROTIEN: BRKIEN Mask          */

#define UUART_PROTIEN_ABRIEN_Pos         (1)                                               /*!< UUART_T::PROTIEN: ABRIEN Position      */
#define UUART_PROTIEN_ABRIEN_Msk         (0x1ul << UUART_PROTIEN_ABRIEN_Pos)               /*!< UUART_T::PROTIEN: ABRIEN Mask          */

#define UUART_PROTIEN_RLSIEN_Pos         (2)                                               /*!< UUART_T::PROTIEN: RLSIEN Position      */
#define UUART_PROTIEN_RLSIEN_Msk         (0x1ul << UUART_PROTIEN_RLSIEN_Pos)               /*!< UUART_T::PROTIEN: RLSIEN Mask          */

#define UUART_PROTSTS_TXSTIF_Pos         (1)                                               /*!< UUART_T::PROTSTS: TXSTIF Position      */
#define UUART_PROTSTS_TXSTIF_Msk         (0x1ul << UUART_PROTSTS_TXSTIF_Pos)               /*!< UUART_T::PROTSTS: TXSTIF Mask          */

#define UUART_PROTSTS_TXENDIF_Pos        (2)                                               /*!< UUART_T::PROTSTS: TXENDIF Position     */
#define UUART_PROTSTS_TXENDIF_Msk        (0x1ul << UUART_PROTSTS_TXENDIF_Pos)              /*!< UUART_T::PROTSTS: TXENDIF Mask         */

#define UUART_PROTSTS_RXSTIF_Pos         (3)                                               /*!< UUART_T::PROTSTS: RXSTIF Position      */
#define UUART_PROTSTS_RXSTIF_Msk         (0x1ul << UUART_PROTSTS_RXSTIF_Pos)               /*!< UUART_T::PROTSTS: RXSTIF Mask          */

#define UUART_PROTSTS_RXENDIF_Pos        (4)                                               /*!< UUART_T::PROTSTS: RXENDIF Position     */
#define UUART_PROTSTS_RXENDIF_Msk        (0x1ul << UUART_PROTSTS_RXENDIF_Pos)              /*!< UUART_T::PROTSTS: RXENDIF Mask         */

#define UUART_PROTSTS_PARITYERR_Pos      (5)                                               /*!< UUART_T::PROTSTS: PARITYERR Position   */
#define UUART_PROTSTS_PARITYERR_Msk      (0x1ul << UUART_PROTSTS_PARITYERR_Pos)            /*!< UUART_T::PROTSTS: PARITYERR Mask       */

#define UUART_PROTSTS_FRMERR_Pos         (6)                                               /*!< UUART_T::PROTSTS: FRMERR Position      */
#define UUART_PROTSTS_FRMERR_Msk         (0x1ul << UUART_PROTSTS_FRMERR_Pos)               /*!< UUART_T::PROTSTS: FRMERR Mask          */

#define UUART_PROTSTS_BREAK_Pos          (7)                                               /*!< UUART_T::PROTSTS: BREAK Position       */
#define UUART_PROTSTS_BREAK_Msk          (0x1ul << UUART_PROTSTS_BREAK_Pos)                /*!< UUART_T::PROTSTS: BREAK Mask           */

#define UUART_PROTSTS_BRKDETIF_Pos          (8)                                               /*!< UUART_T::PROTSTS: BRKDETIF Position       */
#define UUART_PROTSTS_BRKDETIF_Msk          (0x1ul << UUART_PROTSTS_BRKDETIF_Pos)                /*!< UUART_T::PROTSTS: BRKDETIF Mask           */

#define UUART_PROTSTS_ABRDETIF_Pos       (9)                                               /*!< UUART_T::PROTSTS: ABRDETIF Position    */
#define UUART_PROTSTS_ABRDETIF_Msk       (0x1ul << UUART_PROTSTS_ABRDETIF_Pos)             /*!< UUART_T::PROTSTS: ABRDETIF Mask        */

#define UUART_PROTSTS_RXBUSY_Pos         (10)                                              /*!< UUART_T::PROTSTS: RXBUSY Position      */
#define UUART_PROTSTS_RXBUSY_Msk         (0x1ul << UUART_PROTSTS_RXBUSY_Pos)               /*!< UUART_T::PROTSTS: RXBUSY Mask          */

#define UUART_PROTSTS_ABERRSTS_Pos       (11)                                              /*!< UUART_T::PROTSTS: ABERRSTS Position    */
#define UUART_PROTSTS_ABERRSTS_Msk       (0x1ul << UUART_PROTSTS_ABERRSTS_Pos)             /*!< UUART_T::PROTSTS: ABERRSTS Mask        */

/**@}*/ /* UUART_CONST */
/**@}*/ /* end of UUART register group */


/*---------------------- SPI Mode of USCI Controller -------------------------*/
/**
    @addtogroup USPI SPI Mode of USCI Controller(USPI)
    Memory Mapped Structure for USPI Controller
@{ */

typedef struct
{


    /**
     * @var USPI_T::CTL
     * Offset: 0x00  USCI Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |FUNMODE   |Function Mode
     * |        |          |This bit field selects the protocol for this USCI controller
     * |        |          |Selecting a protocol that is not available or a reserved combination disables the USCI
     * |        |          |When switching between two protocols, the USCI has to be disabled before selecting a new protocol
     * |        |          |Simultaneously, the USCI will be reset when user write 000 to FUNMODE.
     * |        |          |000 = The USCI is disabled. All protocol related state machines are set to idle state.
     * |        |          |001 = The SPI protocol is selected.
     * |        |          |010 = The UART protocol is selected.
     * |        |          |100 = The I2C protocol is selected.
     * |        |          |Note: Other bit combinations are reserved.
     * @var USPI_T::INTEN
     * Offset: 0x04  USCI Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TXSTIEN   |Transmit Start Interrupt Enable Bit
     * |        |          |This bit enables the interrupt generation in case of a transmit start event.
     * |        |          |0 = The transmit start interrupt is disabled.
     * |        |          |1 = The transmit start interrupt is enabled.
     * |[2]     |TXENDIEN  |Transmit End Interrupt Enable Bit
     * |        |          |This bit enables the interrupt generation in case of a transmit finish event.
     * |        |          |0 = The transmit finish interrupt is disabled.
     * |        |          |1 = The transmit finish interrupt is enabled.
     * |[3]     |RXSTIEN   |Receive Start Interrupt Enable Bit
     * |        |          |This bit enables the interrupt generation in case of a receive start event.
     * |        |          |0 = The receive start interrupt is disabled.
     * |        |          |1 = The receive start interrupt is enabled.
     * |[4]     |RXENDIEN  |Receive End Interrupt Enable Bit
     * |        |          |This bit enables the interrupt generation in case of a receive finish event.
     * |        |          |0 = The receive end interrupt is disabled.
     * |        |          |1 = The receive end interrupt is enabled.
     * @var USPI_T::BRGEN
     * Offset: 0x08  USCI Baud Rate Generator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RCLKSEL   |Reference Clock Source Selection
     * |        |          |This bit selects the source of reference clock (fREF_CLK).
     * |        |          |0 = Peripheral device clock fPCLK.
     * |        |          |1 = Reserved.
     * |[1]     |PTCLKSEL  |Protocol Clock Source Selection
     * |        |          |This bit selects the source of protocol clock (fPROT_CLK).
     * |        |          |0 = Reference clock fREF_CLK.
     * |        |          |1 = fREF_CLK2 (its frequency is half of fREF_CLK).
     * |[3:2]   |SPCLKSEL  |Sample Clock Source Selection
     * |        |          |This bit field used for the clock source selection of sample clock (fSAMP_CLK) for the protocol processor.
     * |        |          |00 = fDIV_CLK.
     * |        |          |01 = fPROT_CLK.
     * |        |          |10 = fSCLK.
     * |        |          |11 = fREF_CLK.
     * |[4]     |TMCNTEN   |Time Measurement Counter Enable Bit
     * |        |          |This bit enables the 10-bit timing measurement counter.
     * |        |          |0 = Time measurement counter is Disabled.
     * |        |          |1 = Time measurement counter is Enabled.
     * |[5]     |TMCNTSRC  |Time Measurement Counter Clock Source Selection
     * |        |          |0 = Time measurement counter with fPROT_CLK.
     * |        |          |1 = Time measurement counter with fDIV_CLK.
     * |[25:16] |CLKDIV    |Clock Divider
     * |        |          |This bit field defines the ratio between the protocol clock frequency fPROT_CLK and the clock divider frequency fDIV_CLK (fDIV_CLK = fPROT_CLK / (CLKDIV+1) ).
     * |        |          |Note: In UART function, it can be updated by hardware in the 4th falling edge of the input data 0x55 when the auto baud rate function (ABREN(USPI_PROTCTL[6])) is enabled
     * |        |          |The revised value is the average bit time between bit 5 and bit 6
     * |        |          |The user can use revised CLKDIV and new BRDETITV (USPI_PROTCTL[24:16]) to calculate the precise baud rate.
     * @var USPI_T::DATIN0
     * Offset: 0x10  USCI Input Data Signal Configuration Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SYNCSEL   |Input   Signal Synchronization Selection
     * |        |          |This bit   selects if the un-synchronized input signal (with optionally inverted) or the   synchronized (and optionally filtered) signal can be used as input for the   data shift unit.
     * |        |          |0 = The   un-synchronized signal can be taken as input for the data shift unit.
     * |        |          |1 = The   synchronized signal can be taken as input for the data shift unit.
     * |        |          |Note: In SPI protocol, we suggest this bit   should be set as 0.
     * |[2]     |ININV     |Input   Signal Inverse Selection
     * |        |          |This bit   defines the inverter enable of the input asynchronous signal.
     * |        |          |0 = The   un-synchronized input signal will not be inverted.
     * |        |          |1 = The   un-synchronized input signal will be inverted.
     * |        |          |Note: In SPI protocol, we suggest this bit   should be set as 0.
     * @var USPI_T::CTLIN0
     * Offset: 0x20  USCI Input Control Signal Configuration Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SYNCSEL   |Input   Synchronization Signal Selection
     * |        |          |This bit   selects if the un-synchronized input signal (with optionally inverted) or the   synchronized (and optionally filtered) signal can be used as input for the   data shift unit.
     * |        |          |0 = The   un-synchronized signal can be taken as input for the data shift unit.
     * |        |          |1 = The   synchronized signal can be taken as input for the data shift unit.
     * |        |          |Note: In SPI protocol, we suggest this bit   should be set as 0.
     * |[2]     |ININV     |Input   Signal Inverse Selection
     * |        |          |This bit   defines the inverter enable of the input asynchronous signal.
     * |        |          |0 = The   un-synchronized input signal will not be inverted.
     * |        |          |1 = The   un-synchronized input signal will be inverted.
     * @var USPI_T::CLKIN
     * Offset: 0x28  USCI Input Clock Signal Configuration Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SYNCSEL   |Input   Synchronization Signal Selection
     * |        |          |This bit   selects if the un-synchronized input signal or the synchronized (and   optionally filtered) signal can be used as input for the data shift unit.
     * |        |          |0 = The   un-synchronized signal can be taken as input for the data shift unit.
     * |        |          |1 = The   synchronized signal can be taken as input for the data shift unit.
     * |        |          |Note: In SPI protocol, we suggest this bit   should be set as 0.
     * @var USPI_T::LINECTL
     * Offset: 0x2C  USCI Line Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LSB       |LSB First Transmission Selection
     * |        |          |0 = The MSB, which bit of transmit/receive data buffer depends on the setting of DWIDTH, is transmitted/received first.
     * |        |          |1 = The LSB, the bit 0 of data buffer, will be transmitted/received first.
     * |[5]     |DATOINV   |Data Output Inverse Selection
     * |        |          |This bit defines the relation between the internal shift data value and the output data signal of USCIx_DAT0/1 pin.
     * |        |          |0 = Data output level is not inverted.
     * |        |          |1 = Data output level is inverted.
     * |[7]     |CTLOINV   |Control Signal Output Inverse Selection
     * |        |          |This bit defines the relation between the internal control signal and the output control signal.
     * |        |          |0 = No effect.
     * |        |          |1 = The control signal will be inverted before its output.
     * |        |          |Note: The control signal has different definitions in different protocol
     * |        |          |In SPI protocol, the control signal means slave select signal
     * |        |          |In UART protocol, the control signal means RTS signal.
     * |[11:8]  |DWIDTH    |Word Length of Transmission
     * |        |          |This bit field defines the data word length (amount of bits) for reception and transmission
     * |        |          |The data word is always right-aligned in the data buffer
     * |        |          |USCI support word length from 4 to 16 bits.
     * |        |          |0x0: The data word contains 16 bits located at bit positions [15:0].
     * |        |          |0x1: Reserved.
     * |        |          |0x2: Reserved.
     * |        |          |0x3: Reserved.
     * |        |          |0x4: The data word contains 4 bits located at bit positions [3:0].
     * |        |          |0x5: The data word contains 5 bits located at bit positions [4:0].
     * |        |          |...
     * |        |          |0xF: The data word contains 15 bits located at bit positions [14:0].
     * |        |          |Note: In UART protocol, the length can be configured as 6~13 bits.
     * @var USPI_T::TXDAT
     * Offset: 0x30  USCI Transmit Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |TXDAT     |Transmit Data
     * |        |          |Software can use this bit field to write 16-bit transmit data for transmission
     * |        |          |In order to avoid overwriting the transmit data, user have to check TXEMPTY (USPI_BUFSTS[8]) status before writing transmit data into this bit field.
     * @var USPI_T::RXDAT
     * Offset: 0x34  USCI Receive Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RXDAT     |Received Data
     * |        |          |This bit field monitors the received data which stored in receive data buffer.
     * |        |          |Note 1: In I2C protocol, RXDAT[12:8] indicate the different transmission conditions which defined in I2C.
     * |        |          |Note 2: In UART protocol, RXDAT[15:13] indicate the same frame status of BREAK, FRMERR and PARITYERR (USPI_PROTSTS[7:5]).
     * @var USPI_T::BUFCTL
     * Offset: 0x38  USCI Transmit/Receive Buffer Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6]     |TXUDRIEN  |Slave Transmit Under-run Interrupt Enable
     * |        |          |0 = Transmit under-run interrupt Disabled.
     * |        |          |1 = Transmit under-run interrupt Enabled.
     * |[7]     |TXCLR     |Clear Transmit Buffer
     * |        |          |0 = No effect.
     * |        |          |1 = The transmit buffer is cleared
     * |        |          |Should only be used while the buffer is not taking part in data traffic.
     * |        |          |Note: It is cleared automatically after one PCLK cycle.
     * |[14]    |RXOVIEN   |Receive Buffer Overrun Interrupt Enable Control
     * |        |          |0 = Receive overrun interrupt Disabled.
     * |        |          |1 = Receive overrun interrupt Enabled.
     * |[15]    |RXCLR     |Clear Receive Buffer
     * |        |          |0 = No effect.
     * |        |          |1 = The receive buffer is cleared
     * |        |          |Should only be used while the buffer is not taking part in data traffic.
     * |        |          |Note: It is cleared automatically after one PCLK cycle.
     * |[16]    |TXRST     |Transmit Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the transmit-related counters, state machine, and the content of transmit shift register and data buffer.
     * |        |          |Note: It is cleared automatically after one PCLK cycle.
     * |        |          |Note2: Write 1 to this bit will set the output data pin to zero if USPI_BUFCTL[5]=0.
     * |[17]    |RXRST     |Receive Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset the receive-related counters, state machine, and the content of receive shift register and data buffer.
     * |        |          |Note: It is cleared automatically after one PCLK cycle.
     * @var USPI_T::BUFSTS
     * Offset: 0x3C  USCI Transmit/Receive Buffer Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXEMPTY   |Receive Buffer Empty Indicator
     * |        |          |0 = Receive buffer is not empty.
     * |        |          |1 = Receive buffer is empty.
     * |[1]     |RXFULL    |Receive Buffer Full Indicator
     * |        |          |0 = Receive buffer is not full.
     * |        |          |1 = Receive buffer is full.
     * |[3]     |RXOVIF    |Receive Buffer Overrun Interrupt Status
     * |        |          |This bit indicates that a receive buffer overrun event has been detected
     * |        |          |If RXOVIEN (USPI_BUFCTL[14]) is enabled, the corresponding interrupt request is activated
     * |        |          |It is cleared by software writes 1 to this bit.
     * |        |          |0 = A receive buffer overrun event has not been detected.
     * |        |          |1 = A receive buffer overrun event has been detected.
     * |[8]     |TXEMPTY   |Transmit Buffer Empty Indicator
     * |        |          |0 = Transmit buffer is not empty.
     * |        |          |1 = Transmit buffer is empty and available for the next transmission datum.
     * |[9]     |TXFULL    |Transmit Buffer Full Indicator
     * |        |          |0 = Transmit buffer is not full.
     * |        |          |1 = Transmit buffer is full.
     * |[11]    |TXUDRIF   |Transmit Buffer Under-run Interrupt Status
     * |        |          |This bit indicates that a transmit buffer under-run event has been detected
     * |        |          |If enabled by TXUDRIEN (USPI_BUFCTL[6]), the corresponding interrupt request is activated
     * |        |          |It is cleared by software writes 1 to this bit
     * |        |          |0 = A transmit buffer under-run event has not been detected.
     * |        |          |1 = A transmit buffer under-run event has been detected.
     * @var USPI_T::WKCTL
     * Offset: 0x54  USCI Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN      |Wake-up Enable Bit
     * |        |          |0 = Wake-up function Disabled.
     * |        |          |1 = Wake-up function Enabled.
     * |[2]     |PDBOPT    |Power Down Blocking Option
     * |        |          |0 = If user attempts to enter Power-down mode by executing WFI while the protocol is in transferring, MCU will stop the transfer and enter Power-down mode immediately.
     * |        |          |1 = If user attempts to enter Power-down mode by executing WFI while the protocol is in transferring, the on-going transfer will not be stopped and MCU will enter idle mode immediately.
     * @var USPI_T::WKSTS
     * Offset: 0x58  USCI Wake-up Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKF       |Wake-up Flag
     * |        |          |When chip is woken up from Power-down mode, this bit is set to 1
     * |        |          |Software can write 1 to clear this bit.
     * @var USPI_T::PROTCTL
     * Offset: 0x5C  USCI Protocol Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SLAVE     |Slave Mode Selection
     * |        |          |0 = Master mode.
     * |        |          |1 = Slave mode.
     * |[1]     |SLV3WIRE  |Slave 3-wire Mode Selection (Slave Only)
     * |        |          |The SPI protocol can work with 3-wire interface (without slave select signal) in Slave mode.
     * |        |          |0 = 4-wire bi-direction interface.
     * |        |          |1 = 3-wire bi-direction interface.
     * |[2]     |SS        |Slave Select Control (Master Only)
     * |        |          |If AUTOSS bit is cleared, setting this bit to 1 will set the slave select signal to active state, and setting this bit to 0 will set the slave select signal back to inactive state.
     * |        |          |If the AUTOSS function is enabled (AUTOSS = 1), the setting value of this bit will not affect the current state of slave select signal.
     * |        |          |Note: In SPI protocol, the internal slave select signal is active high.
     * |[3]     |AUTOSS    |Automatic Slave Select Function Enable (Master Only)
     * |        |          |0 = Slave select signal will be controlled by the setting value of SS (USPI_PROTCTL[2]) bit.
     * |        |          |1 = Slave select signal will be generated automatically
     * |        |          |The slave select signal will be asserted by the SPI controller when transmit/receive is started, and will be de-asserted after each transmit/receive is finished.
     * |[7:6]   |SCLKMODE  |Serial Bus Clock Mode
     * |        |          |This bit field defines the SCLK idle status, data transmit, and data receive edge.
     * |        |          |MODE0 = The idle state of SPI clock is low level
     * |        |          |Data is transmitted with falling edge and received with rising edge.
     * |        |          |MODE1 = The idle state of SPI clock is low level
     * |        |          |Data is transmitted with rising edge and received with falling edge.
     * |        |          |MODE2 = The idle state of SPI clock is high level
     * |        |          |Data is transmitted with rising edge and received with falling edge.
     * |        |          |MODE3 = The idle state of SPI clock is high level
     * |        |          |Data is transmitted with falling edge and received with rising edge.
     * |[11:8]  |SUSPITV   |Suspend Interval (Master Only)
     * |        |          |This bit field provides the configurable suspend interval between two successive transmit/receive transaction in a transfer
     * |        |          |The definition of the suspend interval is the interval between the last clock edge of the preceding transaction word and the first clock edge of the following transaction word
     * |        |          |The default value is 0x3
     * |        |          |The period of the suspend interval is obtained according to the following equation.
     * |        |          |(SUSPITV[3:0] + 0.5) * period of SPI_CLK clock cycle
     * |        |          |Example:
     * |        |          |SUSPITV = 0x0 ... 0.5 SPI_CLK clock cycle.
     * |        |          |SUSPITV = 0x1 ... 1.5 SPI_CLK clock cycle.
     * |        |          |...
     * |        |          |SUSPITV = 0xE ... 14.5 SPI_CLK clock cycle.
     * |        |          |SUSPITV = 0xF ... 15.5 SPI_CLK clock cycle.
     * |[14:12] |TSMSEL    |Transmit Data Mode Selection
     * |        |          |This bit field describes how receive and transmit data is shifted in and out.
     * |        |          |TSMSEL = 000b: Full-duplex SPI.
     * |        |          |TSMSEL = 100b: Half-duplex SPI.
     * |        |          |Other values are reserved.
     * |        |          |Note: Changing the value of this bit field will produce the TXRST and RXRST to clear the TX/RX data buffer automatically.
     * |[25:16] |SLVTOCNT  |Slave Mode Time-out Period (Slave Only)
     * |        |          |In Slave mode, this bit field is used for Slave time-out period
     * |        |          |This bit field indicates how many clock periods (selected by TMCNTSRC, USPI_BRGEN[5]) between the two edges of input SCLK will assert the Slave time-out event
     * |        |          |Writing 0x0 into this bit field will disable the Slave time-out function.
     * |        |          |Example: Assume SLVTOCNT is 0x0A and TMCNTSRC (USPI_BRGEN[5]) is 1, it means the time-out event will occur if the state of SPI bus clock pin is not changed more than (10+1) periods of fDIV_CLK.
     * |[28]    |TXUDRPOL  |Transmit Under-run Data Polarity (for Slave)
     * |        |          |This bit defines the transmitting data level when no data is available for transferring.
     * |        |          |0 = The output data level is 0 if TX under-run event occurs.
     * |        |          |1 = The output data level is 1 if TX under-run event occurs.
     * |[31]    |PROTEN    |SPI Protocol Enable
     * |        |          |0 = SPI Protocol Disabled.
     * |        |          |1 = SPI Protocol Enabled.
     * @var USPI_T::PROTIEN
     * Offset: 0x60  USCI Protocol Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SSINAIEN  |Slave Select Inactive Interrupt Enable Control
     * |        |          |This bit enables/disables the generation of a slave select interrupt if the slave select changes to inactive.
     * |        |          |0 = Slave select inactive interrupt generation Disabled.
     * |        |          |1 = Slave select inactive interrupt generation Enabled.
     * |[1]     |SSACTIEN  |Slave Select Active Interrupt Enable Control
     * |        |          |This bit enables/disables the generation of a slave select interrupt if the slave select changes to active.
     * |        |          |0 = Slave select active interrupt generation Disabled.
     * |        |          |1 = Slave select active interrupt generation Enabled.
     * |[2]     |SLVTOIEN  |Slave Time-out Interrupt Enable Control
     * |        |          |In SPI protocol, this bit enables the interrupt generation in case of a Slave time-out event.
     * |        |          |0 = The Slave time-out interrupt Disabled.
     * |        |          |1 = The Slave time-out interrupt Enabled.
     * |[3]     |SLVBEIEN  |Slave Mode Bit Count Error Interrupt Enable Control
     * |        |          |If data transfer is terminated by slave time-out or slave select inactive event in Slave mode, so that the transmit/receive data bit count does not match the setting of DWIDTH (USPI_LINECTL[11:8])
     * |        |          |Bit count error event occurs.
     * |        |          |0 = The Slave mode bit count error interrupt Disabled.
     * |        |          |1 = The Slave mode bit count error interrupt Enabled.
     * @var USPI_T::PROTSTS
     * Offset: 0x64  USCI Protocol Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TXSTIF    |Transmit Start Interrupt Flag
     * |        |          |0 = Transmit start event does not occur.
     * |        |          |1 = Transmit start event occurs.
     * |        |          |Note: It is cleared by software writes 1 to this bit
     * |[2]     |TXENDIF   |Transmit End Interrupt Flag
     * |        |          |0 = Transmit end event does not occur.
     * |        |          |1 = Transmit end event occurs.
     * |        |          |Note: It is cleared by software writes 1 to this bit
     * |[3]     |RXSTIF    |Receive Start Interrupt Flag
     * |        |          |0 = Receive start event does not occur.
     * |        |          |1 = Receive start event occurs.
     * |        |          |Note: It is cleared by software writes 1 to this bit
     * |[4]     |RXENDIF   |Receive End Interrupt Flag
     * |        |          |0 = Receive end event does not occur.
     * |        |          |1 = Receive end event occurs.
     * |        |          |Note: It is cleared by software writes 1 to this bit
     * |[5]     |SLVTOIF   |Slave Time-out Interrupt Flag (for Slave Only)
     * |        |          |0 = Slave time-out event does not occur.
     * |        |          |1 = Slave time-out event occurs.
     * |        |          |Note: It is cleared by software writes 1 to this bit
     * |[6]     |SLVBEIF   |Slave Bit Count Error Interrupt Flag (for Slave Only)
     * |        |          |0 = Slave bit count error event does not occur.
     * |        |          |1 = Slave bit count error event occurs.
     * |        |          |Note: It is cleared by software writes 1 to this bit.
     * |[8]     |SSINAIF   |Slave Select Inactive Interrupt Flag (for Slave Only)
     * |        |          |This bit indicates that the internal slave select signal has changed to inactive
     * |        |          |It is cleared by software writes 1 to this bit
     * |        |          |0 = The slave select signal has not changed to inactive.
     * |        |          |1 = The slave select signal has changed to inactive.
     * |        |          |Note: The internal slave select signal is active high.
     * |[9]     |SSACTIF   |Slave Select Active Interrupt Flag (for Slave Only)
     * |        |          |This bit indicates that the internal slave select signal has changed to active
     * |        |          |It is cleared by software writes one to this bit
     * |        |          |0 = The slave select signal has not changed to active.
     * |        |          |1 = The slave select signal has changed to active.
     * |        |          |Note: The internal slave select signal is active high.
     * |[16]    |SSLINE    |Slave Select Line Bus Status (Read Only)
     * |        |          |This bit is only available in Slave mode
     * |        |          |It used to monitor the current status of the input slave select signal on the bus.
     * |        |          |0 = The slave select line status is 0.
     * |        |          |1 = The slave select line status is 1.
     * |[17]    |BUSY      |Busy Status (Read Only)
     * |        |          |0 = SPI is in idle state.
     * |        |          |1 = SPI is in busy state.
     * |        |          |The following listing are the bus busy conditions:
     * |        |          |a. USPI_PROTCTL[31] = 1 and the TXEMPTY = 0.
     * |        |          |b. For SPI Master mode, the TXEMPTY = 1 but the current transaction is not finished yet.
     * |        |          |c
     * |        |          |For SPI Slave mode, the USPI_PROTCTL[31] = 1 and there is serial clock input into the SPI core logic when slave select is active.
     * |        |          |d
     * |        |          |For SPI Slave mode, the USPI_PROTCTL[31] = 1 and the transmit buffer or transmit shift register is not empty even if the slave select is inactive.
     * |[18]    |SLVUDR    |Slave Mode Transmit Under-run Status (Read Only)
     * |        |          |In Slave mode, if there is no available transmit data in buffer while transmit data shift out caused by input serial bus clock, this status flag will be set to 1
     * |        |          |This bit indicates whether the current shift-out data of word transmission is switched to TXUDRPOL (USPI_PROTCTL[28]) or not.
     * |        |          |0 = Slave transmit under-run event does not occur.
     * |        |          |1 = Slave transmit under-run event occurs.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] USCI Control Register                                            */
    __IO uint32_t INTEN;                 /*!< [0x0004] USCI Interrupt Enable Register                                   */
    __IO uint32_t BRGEN;                 /*!< [0x0008] USCI Baud Rate Generator Register                                */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t DATIN0;                /*!< [0x0010] USCI Input Data Signal Configuration Register 0                  */
    __I  uint32_t RESERVE1[3];
    __IO uint32_t CTLIN0;                /*!< [0x0020] USCI Input Control Signal Configuration Register 0               */
    __I  uint32_t RESERVE2[1];
    __IO uint32_t CLKIN;                 /*!< [0x0028] USCI Input Clock Signal Configuration Register                   */
    __IO uint32_t LINECTL;               /*!< [0x002c] USCI Line Control Register                                       */
    __O  uint32_t TXDAT;                 /*!< [0x0030] USCI Transmit Data Register                                      */
    __I  uint32_t RXDAT;                 /*!< [0x0034] USCI Receive Data Register                                       */
    __IO uint32_t BUFCTL;                /*!< [0x0038] USCI Transmit/Receive Buffer Control Register                    */
    __IO  uint32_t BUFSTS;                /*!< [0x003c] USCI Transmit/Receive Buffer Status Register                     */
    __I  uint32_t RESERVE3[5];
    __IO uint32_t WKCTL;                 /*!< [0x0054] USCI Wake-up Control Register                                    */
    __IO uint32_t WKSTS;                 /*!< [0x0058] USCI Wake-up Status Register                                     */
    __IO uint32_t PROTCTL;               /*!< [0x005c] USCI Protocol Control Register                                   */
    __IO uint32_t PROTIEN;               /*!< [0x0060] USCI Protocol Interrupt Enable Register                          */
    __IO uint32_t PROTSTS;               /*!< [0x0064] USCI Protocol Status Register                                    */

} USPI_T;

/**
    @addtogroup USPI_CONST USPI Bit Field Definition
    Constant Definitions for USPI Controller
@{ */

#define USPI_CTL_FUNMODE_Pos             (0)                                               /*!< USPI_T::CTL: FUNMODE Position          */
#define USPI_CTL_FUNMODE_Msk             (0x7ul << USPI_CTL_FUNMODE_Pos)                   /*!< USPI_T::CTL: FUNMODE Mask              */

#define USPI_INTEN_TXSTIEN_Pos           (1)                                               /*!< USPI_T::INTEN: TXSTIEN Position        */
#define USPI_INTEN_TXSTIEN_Msk           (0x1ul << USPI_INTEN_TXSTIEN_Pos)                 /*!< USPI_T::INTEN: TXSTIEN Mask            */

#define USPI_INTEN_TXENDIEN_Pos          (2)                                               /*!< USPI_T::INTEN: TXENDIEN Position       */
#define USPI_INTEN_TXENDIEN_Msk          (0x1ul << USPI_INTEN_TXENDIEN_Pos)                /*!< USPI_T::INTEN: TXENDIEN Mask           */

#define USPI_INTEN_RXSTIEN_Pos           (3)                                               /*!< USPI_T::INTEN: RXSTIEN Position        */
#define USPI_INTEN_RXSTIEN_Msk           (0x1ul << USPI_INTEN_RXSTIEN_Pos)                 /*!< USPI_T::INTEN: RXSTIEN Mask            */

#define USPI_INTEN_RXENDIEN_Pos          (4)                                               /*!< USPI_T::INTEN: RXENDIEN Position       */
#define USPI_INTEN_RXENDIEN_Msk          (0x1ul << USPI_INTEN_RXENDIEN_Pos)                /*!< USPI_T::INTEN: RXENDIEN Mask           */

#define USPI_BRGEN_RCLKSEL_Pos           (0)                                               /*!< USPI_T::BRGEN: RCLKSEL Position        */
#define USPI_BRGEN_RCLKSEL_Msk           (0x1ul << USPI_BRGEN_RCLKSEL_Pos)                 /*!< USPI_T::BRGEN: RCLKSEL Mask            */

#define USPI_BRGEN_PTCLKSEL_Pos          (1)                                               /*!< USPI_T::BRGEN: PTCLKSEL Position       */
#define USPI_BRGEN_PTCLKSEL_Msk          (0x1ul << USPI_BRGEN_PTCLKSEL_Pos)                /*!< USPI_T::BRGEN: PTCLKSEL Mask           */

#define USPI_BRGEN_SPCLKSEL_Pos          (2)                                               /*!< USPI_T::BRGEN: SPCLKSEL Position       */
#define USPI_BRGEN_SPCLKSEL_Msk          (0x3ul << USPI_BRGEN_SPCLKSEL_Pos)                /*!< USPI_T::BRGEN: SPCLKSEL Mask           */

#define USPI_BRGEN_TMCNTEN_Pos           (4)                                               /*!< USPI_T::BRGEN: TMCNTEN Position        */
#define USPI_BRGEN_TMCNTEN_Msk           (0x1ul << USPI_BRGEN_TMCNTEN_Pos)                 /*!< USPI_T::BRGEN: TMCNTEN Mask            */

#define USPI_BRGEN_TMCNTSRC_Pos          (5)                                               /*!< USPI_T::BRGEN: TMCNTSRC Position       */
#define USPI_BRGEN_TMCNTSRC_Msk          (0x1ul << USPI_BRGEN_TMCNTSRC_Pos)                /*!< USPI_T::BRGEN: TMCNTSRC Mask           */

#define USPI_BRGEN_CLKDIV_Pos            (16)                                              /*!< USPI_T::BRGEN: CLKDIV Position         */
#define USPI_BRGEN_CLKDIV_Msk            (0x3fful << USPI_BRGEN_CLKDIV_Pos)                /*!< USPI_T::BRGEN: CLKDIV Mask             */

#define USPI_DATIN0_SYNCSEL_Pos          (0)                                               /*!< USPI_T::DATIN0: SYNCSEL Position       */
#define USPI_DATIN0_SYNCSEL_Msk          (0x1ul << USPI_DATIN0_SYNCSEL_Pos)                /*!< USPI_T::DATIN0: SYNCSEL Mask           */

#define USPI_DATIN0_ININV_Pos            (2)                                               /*!< USPI_T::DATIN0: ININV Position         */
#define USPI_DATIN0_ININV_Msk            (0x1ul << USPI_DATIN0_ININV_Pos)                  /*!< USPI_T::DATIN0: ININV Mask             */

#define USPI_CTLIN0_SYNCSEL_Pos          (0)                                               /*!< USPI_T::CTLIN0: SYNCSEL Position       */
#define USPI_CTLIN0_SYNCSEL_Msk          (0x1ul << USPI_CTLIN0_SYNCSEL_Pos)                /*!< USPI_T::CTLIN0: SYNCSEL Mask           */

#define USPI_CTLIN0_ININV_Pos            (2)                                               /*!< USPI_T::CTLIN0: ININV Position         */
#define USPI_CTLIN0_ININV_Msk            (0x1ul << USPI_CTLIN0_ININV_Pos)                  /*!< USPI_T::CTLIN0: ININV Mask             */

#define USPI_CLKIN_SYNCSEL_Pos           (0)                                               /*!< USPI_T::CLKIN: SYNCSEL Position        */
#define USPI_CLKIN_SYNCSEL_Msk           (0x1ul << USPI_CLKIN_SYNCSEL_Pos)                 /*!< USPI_T::CLKIN: SYNCSEL Mask            */

#define USPI_LINECTL_LSB_Pos             (0)                                               /*!< USPI_T::LINECTL: LSB Position          */
#define USPI_LINECTL_LSB_Msk             (0x1ul << USPI_LINECTL_LSB_Pos)                   /*!< USPI_T::LINECTL: LSB Mask              */

#define USPI_LINECTL_DATOINV_Pos         (5)                                               /*!< USPI_T::LINECTL: DATOINV Position      */
#define USPI_LINECTL_DATOINV_Msk         (0x1ul << USPI_LINECTL_DATOINV_Pos)               /*!< USPI_T::LINECTL: DATOINV Mask          */

#define USPI_LINECTL_CTLOINV_Pos         (7)                                               /*!< USPI_T::LINECTL: CTLOINV Position      */
#define USPI_LINECTL_CTLOINV_Msk         (0x1ul << USPI_LINECTL_CTLOINV_Pos)               /*!< USPI_T::LINECTL: CTLOINV Mask          */

#define USPI_LINECTL_DWIDTH_Pos          (8)                                               /*!< USPI_T::LINECTL: DWIDTH Position       */
#define USPI_LINECTL_DWIDTH_Msk          (0xful << USPI_LINECTL_DWIDTH_Pos)                /*!< USPI_T::LINECTL: DWIDTH Mask           */

#define USPI_TXDAT_TXDAT_Pos             (0)                                               /*!< USPI_T::TXDAT: TXDAT Position          */
#define USPI_TXDAT_TXDAT_Msk             (0xfffful << USPI_TXDAT_TXDAT_Pos)                /*!< USPI_T::TXDAT: TXDAT Mask              */

#define USPI_RXDAT_RXDAT_Pos             (0)                                               /*!< USPI_T::RXDAT: RXDAT Position          */
#define USPI_RXDAT_RXDAT_Msk             (0xfffful << USPI_RXDAT_RXDAT_Pos)                /*!< USPI_T::RXDAT: RXDAT Mask              */

#define USPI_BUFCTL_TXUDRIEN_Pos         (6)                                               /*!< USPI_T::BUFCTL: TXUDRIEN Position      */
#define USPI_BUFCTL_TXUDRIEN_Msk         (0x1ul << USPI_BUFCTL_TXUDRIEN_Pos)               /*!< USPI_T::BUFCTL: TXUDRIEN Mask          */

#define USPI_BUFCTL_TXCLR_Pos            (7)                                               /*!< USPI_T::BUFCTL: TXCLR Position         */
#define USPI_BUFCTL_TXCLR_Msk            (0x1ul << USPI_BUFCTL_TXCLR_Pos)                  /*!< USPI_T::BUFCTL: TXCLR Mask             */

#define USPI_BUFCTL_RXOVIEN_Pos          (14)                                              /*!< USPI_T::BUFCTL: RXOVIEN Position       */
#define USPI_BUFCTL_RXOVIEN_Msk          (0x1ul << USPI_BUFCTL_RXOVIEN_Pos)                /*!< USPI_T::BUFCTL: RXOVIEN Mask           */

#define USPI_BUFCTL_RXCLR_Pos            (15)                                              /*!< USPI_T::BUFCTL: RXCLR Position         */
#define USPI_BUFCTL_RXCLR_Msk            (0x1ul << USPI_BUFCTL_RXCLR_Pos)                  /*!< USPI_T::BUFCTL: RXCLR Mask             */

#define USPI_BUFCTL_TXRST_Pos            (16)                                              /*!< USPI_T::BUFCTL: TXRST Position         */
#define USPI_BUFCTL_TXRST_Msk            (0x1ul << USPI_BUFCTL_TXRST_Pos)                  /*!< USPI_T::BUFCTL: TXRST Mask             */

#define USPI_BUFCTL_RXRST_Pos            (17)                                              /*!< USPI_T::BUFCTL: RXRST Position         */
#define USPI_BUFCTL_RXRST_Msk            (0x1ul << USPI_BUFCTL_RXRST_Pos)                  /*!< USPI_T::BUFCTL: RXRST Mask             */

#define USPI_BUFSTS_RXEMPTY_Pos          (0)                                               /*!< USPI_T::BUFSTS: RXEMPTY Position       */
#define USPI_BUFSTS_RXEMPTY_Msk          (0x1ul << USPI_BUFSTS_RXEMPTY_Pos)                /*!< USPI_T::BUFSTS: RXEMPTY Mask           */

#define USPI_BUFSTS_RXFULL_Pos           (1)                                               /*!< USPI_T::BUFSTS: RXFULL Position        */
#define USPI_BUFSTS_RXFULL_Msk           (0x1ul << USPI_BUFSTS_RXFULL_Pos)                 /*!< USPI_T::BUFSTS: RXFULL Mask            */

#define USPI_BUFSTS_RXOVIF_Pos           (3)                                               /*!< USPI_T::BUFSTS: RXOVIF Position        */
#define USPI_BUFSTS_RXOVIF_Msk           (0x1ul << USPI_BUFSTS_RXOVIF_Pos)                 /*!< USPI_T::BUFSTS: RXOVIF Mask            */

#define USPI_BUFSTS_TXEMPTY_Pos          (8)                                               /*!< USPI_T::BUFSTS: TXEMPTY Position       */
#define USPI_BUFSTS_TXEMPTY_Msk          (0x1ul << USPI_BUFSTS_TXEMPTY_Pos)                /*!< USPI_T::BUFSTS: TXEMPTY Mask           */

#define USPI_BUFSTS_TXFULL_Pos           (9)                                               /*!< USPI_T::BUFSTS: TXFULL Position        */
#define USPI_BUFSTS_TXFULL_Msk           (0x1ul << USPI_BUFSTS_TXFULL_Pos)                 /*!< USPI_T::BUFSTS: TXFULL Mask            */

#define USPI_BUFSTS_TXUDRIF_Pos          (11)                                              /*!< USPI_T::BUFSTS: TXUDRIF Position       */
#define USPI_BUFSTS_TXUDRIF_Msk          (0x1ul << USPI_BUFSTS_TXUDRIF_Pos)                /*!< USPI_T::BUFSTS: TXUDRIF Mask           */

#define USPI_WKCTL_WKEN_Pos              (0)                                               /*!< USPI_T::WKCTL: WKEN Position           */
#define USPI_WKCTL_WKEN_Msk              (0x1ul << USPI_WKCTL_WKEN_Pos)                    /*!< USPI_T::WKCTL: WKEN Mask               */

#define USPI_WKCTL_PDBOPT_Pos            (2)                                               /*!< USPI_T::WKCTL: PDBOPT Position         */
#define USPI_WKCTL_PDBOPT_Msk            (0x1ul << USPI_WKCTL_PDBOPT_Pos)                  /*!< USPI_T::WKCTL: PDBOPT Mask             */

#define USPI_WKSTS_WKF_Pos               (0)                                               /*!< USPI_T::WKSTS: WKF Position            */
#define USPI_WKSTS_WKF_Msk               (0x1ul << USPI_WKSTS_WKF_Pos)                     /*!< USPI_T::WKSTS: WKF Mask                */

#define USPI_PROTCTL_SLAVE_Pos           (0)                                               /*!< USPI_T::PROTCTL: SLAVE Position        */
#define USPI_PROTCTL_SLAVE_Msk           (0x1ul << USPI_PROTCTL_SLAVE_Pos)                 /*!< USPI_T::PROTCTL: SLAVE Mask            */

#define USPI_PROTCTL_SLV3WIRE_Pos        (1)                                               /*!< USPI_T::PROTCTL: SLV3WIRE Position     */
#define USPI_PROTCTL_SLV3WIRE_Msk        (0x1ul << USPI_PROTCTL_SLV3WIRE_Pos)              /*!< USPI_T::PROTCTL: SLV3WIRE Mask         */

#define USPI_PROTCTL_SS_Pos              (2)                                               /*!< USPI_T::PROTCTL: SS Position           */
#define USPI_PROTCTL_SS_Msk              (0x1ul << USPI_PROTCTL_SS_Pos)                    /*!< USPI_T::PROTCTL: SS Mask               */

#define USPI_PROTCTL_AUTOSS_Pos          (3)                                               /*!< USPI_T::PROTCTL: AUTOSS Position       */
#define USPI_PROTCTL_AUTOSS_Msk          (0x1ul << USPI_PROTCTL_AUTOSS_Pos)                /*!< USPI_T::PROTCTL: AUTOSS Mask           */

#define USPI_PROTCTL_SCLKMODE_Pos        (6)                                               /*!< USPI_T::PROTCTL: SCLKMODE Position     */
#define USPI_PROTCTL_SCLKMODE_Msk        (0x3ul << USPI_PROTCTL_SCLKMODE_Pos)              /*!< USPI_T::PROTCTL: SCLKMODE Mask         */

#define USPI_PROTCTL_SUSPITV_Pos         (8)                                               /*!< USPI_T::PROTCTL: SUSPITV Position      */
#define USPI_PROTCTL_SUSPITV_Msk         (0xful << USPI_PROTCTL_SUSPITV_Pos)               /*!< USPI_T::PROTCTL: SUSPITV Mask          */

#define USPI_PROTCTL_TSMSEL_Pos          (12)                                              /*!< USPI_T::PROTCTL: TSMSEL Position       */
#define USPI_PROTCTL_TSMSEL_Msk          (0x7ul << USPI_PROTCTL_TSMSEL_Pos)                /*!< USPI_T::PROTCTL: TSMSEL Mask           */

#define USPI_PROTCTL_SLVTOCNT_Pos        (16)                                              /*!< USPI_T::PROTCTL: SLVTOCNT Position     */
#define USPI_PROTCTL_SLVTOCNT_Msk        (0x3fful << USPI_PROTCTL_SLVTOCNT_Pos)            /*!< USPI_T::PROTCTL: SLVTOCNT Mask         */

#define USPI_PROTCTL_TXUDRPOL_Pos        (28)                                              /*!< USPI_T::PROTCTL: TXUDRPOL Position     */
#define USPI_PROTCTL_TXUDRPOL_Msk        (0x1ul << USPI_PROTCTL_TXUDRPOL_Pos)              /*!< USPI_T::PROTCTL: TXUDRPOL Mask         */

#define USPI_PROTCTL_PROTEN_Pos          (31)                                              /*!< USPI_T::PROTCTL: PROTEN Position       */
#define USPI_PROTCTL_PROTEN_Msk          (0x1ul << USPI_PROTCTL_PROTEN_Pos)                /*!< USPI_T::PROTCTL: PROTEN Mask           */

#define USPI_PROTIEN_SSINAIEN_Pos        (0)                                               /*!< USPI_T::PROTIEN: SSINAIEN Position     */
#define USPI_PROTIEN_SSINAIEN_Msk        (0x1ul << USPI_PROTIEN_SSINAIEN_Pos)              /*!< USPI_T::PROTIEN: SSINAIEN Mask         */

#define USPI_PROTIEN_SSACTIEN_Pos        (1)                                               /*!< USPI_T::PROTIEN: SSACTIEN Position     */
#define USPI_PROTIEN_SSACTIEN_Msk        (0x1ul << USPI_PROTIEN_SSACTIEN_Pos)              /*!< USPI_T::PROTIEN: SSACTIEN Mask         */

#define USPI_PROTIEN_SLVTOIEN_Pos        (2)                                               /*!< USPI_T::PROTIEN: SLVTOIEN Position     */
#define USPI_PROTIEN_SLVTOIEN_Msk        (0x1ul << USPI_PROTIEN_SLVTOIEN_Pos)              /*!< USPI_T::PROTIEN: SLVTOIEN Mask         */

#define USPI_PROTIEN_SLVBEIEN_Pos        (3)                                               /*!< USPI_T::PROTIEN: SLVBEIEN Position     */
#define USPI_PROTIEN_SLVBEIEN_Msk        (0x1ul << USPI_PROTIEN_SLVBEIEN_Pos)              /*!< USPI_T::PROTIEN: SLVBEIEN Mask         */

#define USPI_PROTSTS_TXSTIF_Pos          (1)                                               /*!< USPI_T::PROTSTS: TXSTIF Position       */
#define USPI_PROTSTS_TXSTIF_Msk          (0x1ul << USPI_PROTSTS_TXSTIF_Pos)                /*!< USPI_T::PROTSTS: TXSTIF Mask           */

#define USPI_PROTSTS_TXENDIF_Pos         (2)                                               /*!< USPI_T::PROTSTS: TXENDIF Position      */
#define USPI_PROTSTS_TXENDIF_Msk         (0x1ul << USPI_PROTSTS_TXENDIF_Pos)               /*!< USPI_T::PROTSTS: TXENDIF Mask          */

#define USPI_PROTSTS_RXSTIF_Pos          (3)                                               /*!< USPI_T::PROTSTS: RXSTIF Position       */
#define USPI_PROTSTS_RXSTIF_Msk          (0x1ul << USPI_PROTSTS_RXSTIF_Pos)                /*!< USPI_T::PROTSTS: RXSTIF Mask           */

#define USPI_PROTSTS_RXENDIF_Pos         (4)                                               /*!< USPI_T::PROTSTS: RXENDIF Position      */
#define USPI_PROTSTS_RXENDIF_Msk         (0x1ul << USPI_PROTSTS_RXENDIF_Pos)               /*!< USPI_T::PROTSTS: RXENDIF Mask          */

#define USPI_PROTSTS_SLVTOIF_Pos         (5)                                               /*!< USPI_T::PROTSTS: SLVTOIF Position      */
#define USPI_PROTSTS_SLVTOIF_Msk         (0x1ul << USPI_PROTSTS_SLVTOIF_Pos)               /*!< USPI_T::PROTSTS: SLVTOIF Mask          */

#define USPI_PROTSTS_SLVBEIF_Pos         (6)                                               /*!< USPI_T::PROTSTS: SLVBEIF Position      */
#define USPI_PROTSTS_SLVBEIF_Msk         (0x1ul << USPI_PROTSTS_SLVBEIF_Pos)               /*!< USPI_T::PROTSTS: SLVBEIF Mask          */

#define USPI_PROTSTS_SSINAIF_Pos         (8)                                               /*!< USPI_T::PROTSTS: SSINAIF Position      */
#define USPI_PROTSTS_SSINAIF_Msk         (0x1ul << USPI_PROTSTS_SSINAIF_Pos)               /*!< USPI_T::PROTSTS: SSINAIF Mask          */

#define USPI_PROTSTS_SSACTIF_Pos         (9)                                               /*!< USPI_T::PROTSTS: SSACTIF Position      */
#define USPI_PROTSTS_SSACTIF_Msk         (0x1ul << USPI_PROTSTS_SSACTIF_Pos)               /*!< USPI_T::PROTSTS: SSACTIF Mask          */

#define USPI_PROTSTS_SSLINE_Pos          (16)                                              /*!< USPI_T::PROTSTS: SSLINE Position       */
#define USPI_PROTSTS_SSLINE_Msk          (0x1ul << USPI_PROTSTS_SSLINE_Pos)                /*!< USPI_T::PROTSTS: SSLINE Mask           */

#define USPI_PROTSTS_BUSY_Pos            (17)                                              /*!< USPI_T::PROTSTS: BUSY Position         */
#define USPI_PROTSTS_BUSY_Msk            (0x1ul << USPI_PROTSTS_BUSY_Pos)                  /*!< USPI_T::PROTSTS: BUSY Mask             */

#define USPI_PROTSTS_SLVUDR_Pos          (18)                                              /*!< USPI_T::PROTSTS: SLVUDR Position       */
#define USPI_PROTSTS_SLVUDR_Msk          (0x1ul << USPI_PROTSTS_SLVUDR_Pos)                /*!< USPI_T::PROTSTS: SLVUDR Mask           */

/**@}*/ /* USPI_CONST */
/**@}*/ /* end of USPI register group */


/*---------------------- I2C Mode of USCI Controller -------------------------*/
/**
    @addtogroup UI2C I2C Mode of USCI Controller(UI2C)
    Memory Mapped Structure for UI2C Controller
@{ */

typedef struct
{


    /**
     * @var UI2C_T::CTL
     * Offset: 0x00  USCI Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |FUNMODE   |Function Mode
     * |        |          |This bit field selects the protocol for this USCI controller
     * |        |          |Selecting a protocol that is not available or a reserved combination disables the USCI
     * |        |          |When switching between two protocols, the USCI has to be disabled before selecting a new protocol
     * |        |          |Simultaneously, the USCI will be reset when user write 000 to FUNMODE.
     * |        |          |000 = The USCI is disabled. All protocol related state machines are set to idle state.
     * |        |          |001 = The SPI protocol is selected.
     * |        |          |010 = The UART protocol is selected.
     * |        |          |100 = The I2C protocol is selected.
     * |        |          |Note: Other bit combinations are reserved.
     * @var UI2C_T::BRGEN
     * Offset: 0x08  USCI Baud Rate Generator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RCLKSEL   |Reference Clock Source Selection
     * |        |          |This bit selects the source signal of reference clock (fREF_CLK).
     * |        |          |0 = Peripheral device clock fPCLK.
     * |        |          |1 = Reserved.
     * |[1]     |PTCLKSEL  |Protocol Clock Source Selection
     * |        |          |This bit selects the source signal of protocol clock (fPROT_CLK).
     * |        |          |0 = Reference clock fREF_CLK.
     * |        |          |1 = fREF_CLK2 (its frequency is half of fREF_CLK).
     * |[3:2]   |SPCLKSEL  |Sample Clock Source Selection
     * |        |          |This bit field used for the clock source selection of a sample clock (fSAMP_CLK) for the protocol processor.
     * |        |          |00 = fSAMP_CLK = fDIV_CLK.
     * |        |          |01 = fSAMP_CLK = fPROT_CLK.
     * |        |          |10 = fSAMP_CLK = fSCLK.
     * |        |          |11 = fSAMP_CLK = fREF_CLK.
     * |[4]     |TMCNTEN   |Time Measurement Counter Enable Bit
     * |        |          |This bit enables the 10-bit timing measurement counter.
     * |        |          |0 = Time measurement counter is Disabled.
     * |        |          |1 = Time measurement counter is Enabled.
     * |[5]     |TMCNTSRC  |Time Measurement Counter Clock Source Selection
     * |        |          |0 = Time measurement counter with fPROT_CLK.
     * |        |          |1 = Time measurement counter with fDIV_CLK.
     * |[9:8]   |PDSCNT    |Pre-divider for Sample Counter
     * |        |          |This bit field defines the divide ratio of the clock division from sample clock fSAMP_CLK
     * |        |          |The divided frequency fPDS_CNT = fSAMP_CLK / (PDSCNT+1).
     * |[14:10] |DSCNT     |Denominator for Sample Counter
     * |        |          |This bit field defines the divide ratio of the sample clock fSAMP_CLK.
     * |        |          |The divided frequency fDS_CNT = fPDS_CNT / (DSCNT+1).
     * |        |          |Note: The maximum value of DSCNT is 0xF on UART mode and suggest to set over 4 to confirm the receiver data is sampled in right value
     * |[25:16] |CLKDIV    |Clock Divider
     * |        |          |This bit field defines the ratio between the protocol clock frequency fPROT_CLK and the clock divider frequency fDIV_CLK (fDIV_CLK = fPROT_CLK / (CLKDIV+1) ).
     * |        |          |Note: In UART function, it can be updated by hardware in the 4th falling edge of the input data 0x55 when the auto baud rate function (ABREN(UI2C_PROTCTL[6])) is enabled
     * |        |          |The revised value is the average bit time between bit 5 and bit 6
     * |        |          |The user can use revised CLKDIV and new BRDETITV (UI2C_PROTCTL[24:16]) to calculate the precise baud rate.
     * @var UI2C_T::LINECTL
     * Offset: 0x2C  USCI Line Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LSB       |LSB First Transmission Selection
     * |        |          |0 = The MSB, which bit of transmit/receive data buffer depends on the setting of DWIDTH, is transmitted/received first.
     * |        |          |1 = The LSB, the bit 0 of data buffer, will be transmitted/received first.
     * |[11:8]  |DWIDTH    |Word Length of Transmission
     * |        |          |This bit field defines the data word length (amount of bits) for reception and transmission
     * |        |          |The data word is always right-aligned in the data buffer
     * |        |          |USCI support word length from 4 to 16 bits.
     * |        |          |0x0: The data word contains 16 bits located at bit positions [15:0].
     * |        |          |0x1: Reserved.
     * |        |          |0x2: Reserved.
     * |        |          |0x3: Reserved.
     * |        |          |0x4: The data word contains 4 bits located at bit positions [3:0].
     * |        |          |0x5: The data word contains 5 bits located at bit positions [4:0].
     * |        |          |...
     * |        |          |0xF: The data word contains 15 bits located at bit positions [14:0].
     * |        |          |Note: In I2C protocol, the length must be configured as 8 bits.
     * @var UI2C_T::TXDAT
     * Offset: 0x30  USCI Transmit Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |TXDAT     |Transmit Data
     * |        |          |Software can use this bit field to write 16-bit transmit data for transmission.
     * @var UI2C_T::RXDAT
     * Offset: 0x34  USCI Receive Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RXDAT     |Received Data
     * |        |          |This bit field monitors the received data which stored in receive data buffer.
     * |        |          |Note 1: In I2C protocol, only use RXDAT[7:0]..
     * @var UI2C_T::DEVADDR0
     * Offset: 0x44  USCI Device Address Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |DEVADDR   |Device Address
     * |        |          |In I2C protocol, this bit field contains the programmed slave address
     * |        |          |If the first received address byte are 1111 0AAXb, the AA bits are compared to the bits DEVADDR[9:8] to check for address match, where the X is R/W bit
     * |        |          |Then the second address byte is also compared to DEVADDR[7:0].
     * |        |          |Note: The DEVADDR [9:7] must be set 3'b000 when I2C operating in 7-bit address mode.
     * @var UI2C_T::ADDRMSK0
     * Offset: 0x4C  USCI Device Address Mask Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |ADDRMSK   |USCI Device Address Mask
     * |        |          |0 = Mask Disabled (the received corresponding register bit should be exact the same as address register.).
     * |        |          |1 = Mask Enabled (the received corresponding address bit is don't care.).
     * |        |          |USCI support multiple address recognition with two address mask register
     * |        |          |When the bit in the address mask register is set to one, it means the received corresponding address bit is don't-care
     * |        |          |If the bit is set to zero, that means the received corresponding register bit should be exact the same as address register.
     * @var UI2C_T::WKCTL
     * Offset: 0x54  USCI Wake-up Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN      |Wake-up Enable Bit
     * |        |          |0 = Wake-up function Disabled.
     * |        |          |1 = Wake-up function Enabled.
     * |[1]     |WKADDREN  |Wake-up Address Match Enable Bit
     * |        |          |0 = The chip is woken up according data toggle.
     * |        |          |1 = The chip is woken up according address match.
     * @var UI2C_T::WKSTS
     * Offset: 0x58  USCI Wake-up Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKF       |Wake-up Flag
     * |        |          |When chip is woken up from Power-down mode, this bit is set to 1
     * |        |          |Software can write 1 to clear this bit.
     * @var UI2C_T::PROTCTL
     * Offset: 0x5C  USCI Protocol Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GCFUNC    |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[1]     |AA        |Assert Acknowledge Control
     * |        |          |When AA =1 prior to address or data received, an acknowledged (low level to SDA) will be returned during the acknowledge clock pulse on the SCL line when 1.) A slave is acknowledging the address sent from master, 2.) The receiver devices are acknowledging the data sent by transmitter
     * |        |          |When AA=0 prior to address or data received, a Not acknowledged (high level to SDA) will be returned during the acknowledge clock pulse on the SCL line.
     * |[2]     |STO       |I2C STOP Control
     * |        |          |In Master mode, setting STO to transmit a STOP condition to bus then I2C hardware will check the bus condition if a STOP condition is detected this bit will be cleared by hardware automatically
     * |        |          |In a slave mode, setting STO resets I2C hardware to the defined "not addressed" slave mode when bus error (UI2C_PROTSTS.ERRIF = 1).
     * |[3]     |STA       |I2C START Control
     * |        |          |Setting STA to logic 1 to enter Master mode, the I2C hardware sends a START or repeat START condition to bus when the bus is free.
     * |[4]     |ADDR10EN  |Address 10-bit Function Enable Bit
     * |        |          |0 = Address match 10 bit function is disabled.
     * |        |          |1 = Address match 10 bit function is enabled.
     * |[5]     |PTRG      |I2C Protocol Trigger
     * |        |          |When a new state is present in the UI2C_PROTSTS register, if the related interrupt enable bits are set, the I2C interrupt is requested
     * |        |          |It must write one by software to this bit after the related interrupt flags are set to 1 and the I2C protocol function will go ahead until the STOP is active or the PROTEN is disabled.
     * |        |          |0 = I2C's stretch disabled and the I2C protocol function will go ahead.
     * |        |          |1 = I2C's stretch active.
     * |[25:16] |TOCNT     |Time-out Clock Cycle
     * |        |          |This bit field indicates how many clock cycle selected by TMCNTSRC (UI2C_BRGEN [5]) when each interrupt flags are clear
     * |        |          |The time-out is enable when TOCNT bigger than 0.
     * |        |          |Note: The TMCNTSRC (UI2C_BRGEN [5]) must be set zero on I2C mode.
     * |[31]    |PROTEN    |I2C Protocol Enable Bit
     * |        |          |0 = I2C Protocol disable.
     * |        |          |1 = I2C Protocol enable.
     * @var UI2C_T::PROTIEN
     * Offset: 0x60  USCI Protocol Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TOIEN     |Time-out Interrupt Enable Control
     * |        |          |In I2C protocol, this bit enables the interrupt generation in case of a time-out event.
     * |        |          |0 = The time-out interrupt is disabled.
     * |        |          |1 = The time-out interrupt is enabled.
     * |[1]     |STARIEN   |Start Condition Received Interrupt Enable Control
     * |        |          |This bit enables the generation of a protocol interrupt if a start condition is detected.
     * |        |          |0 = The start condition interrupt is disabled.
     * |        |          |1 = The start condition interrupt is enabled.
     * |[2]     |STORIEN   |Stop Condition Received Interrupt Enable Control
     * |        |          |This bit enables the generation of a protocol interrupt if a stop condition is detected.
     * |        |          |0 = The stop condition interrupt is disabled.
     * |        |          |1 = The stop condition interrupt is enabled.
     * |[3]     |NACKIEN   |Non - Acknowledge Interrupt Enable Control
     * |        |          |This bit enables the generation of a protocol interrupt if a non - acknowledge is detected by a master.
     * |        |          |0 = The non - acknowledge interrupt is disabled.
     * |        |          |1 = The non - acknowledge interrupt is enabled.
     * |[4]     |ARBLOIEN  |Arbitration Lost Interrupt Enable Control
     * |        |          |This bit enables the generation of a protocol interrupt if an arbitration lost event is detected.
     * |        |          |0 = The arbitration lost interrupt is disabled.
     * |        |          |1 = The arbitration lost interrupt is enabled.
     * |[5]     |ERRIEN    |Error Interrupt Enable Control
     * |        |          |This bit enables the generation of a protocol interrupt if an I2C error condition is detected (indicated by ERR (UI2C_PROTSTS [16])).
     * |        |          |0 = The error interrupt is disabled.
     * |        |          |1 = The error interrupt is enabled.
     * |[6]     |ACKIEN    |Acknowledge Interrupt Enable Control
     * |        |          |This bit enables the generation of a protocol interrupt if an acknowledge is detected by a master.
     * |        |          |0 = The acknowledge interrupt is disabled.
     * |        |          |1 = The acknowledge interrupt is enabled.
     * @var UI2C_T::PROTSTS
     * Offset: 0x64  USCI Protocol Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5]     |TOIF      |Time-out Interrupt Flag
     * |        |          |0 = A time-out interrupt status has not occurred.
     * |        |          |1 = A time-out interrupt status has occurred.
     * |        |          |Note: It is cleared by software writing one into this bit
     * |[6]     |ONBUSY    |On Bus Busy
     * |        |          |Indicates that a communication is in progress on the bus
     * |        |          |It is set by hardware when a START condition is detected
     * |        |          |It is cleared by hardware when a STOP condition is detected
     * |        |          |0 = The bus is IDLE (both SCLK and SDA High).
     * |        |          |1 = The bus is busy.
     * |[8]     |STARIF    |Start Condition Received Interrupt Flag
     * |        |          |This bit indicates that a start condition or repeated start condition has been detected on master mode
     * |        |          |However, this bit also indicates that a repeated start condition has been detected on slave mode.
     * |        |          |A protocol interrupt can be generated if UI2C_PROTCTL.STARIEN = 1.
     * |        |          |0 = A start condition has not yet been detected.
     * |        |          |1 = A start condition has been detected.
     * |        |          |It is cleared by software writing one into this bit
     * |[9]     |STORIF    |Stop Condition Received Interrupt Flag
     * |        |          |This bit indicates that a stop condition has been detected on the I2C bus lines
     * |        |          |A protocol interrupt can be generated if UI2C_PROTCTL.STORIEN = 1.
     * |        |          |0 = A stop condition has not yet been detected.
     * |        |          |1 = A stop condition has been detected.
     * |        |          |It is cleared by software writing one into this bit
     * |[10]    |NACKIF    |Non - Acknowledge Received Interrupt Flag
     * |        |          |This bit indicates that a non - acknowledge has been received in master mode
     * |        |          |This bit is not set in slave mode
     * |        |          |A protocol interrupt can be generated if UI2C_PROTCTL.NACKIEN = 1.
     * |        |          |0 = A non - acknowledge has not been received.
     * |        |          |1 = A non - acknowledge has been received.
     * |        |          |It is cleared by software writing one into this bit
     * |[11]    |ARBLOIF   |Arbitration Lost Interrupt Flag
     * |        |          |This bit indicates that an arbitration has been lost
     * |        |          |A protocol interrupt can be generated if UI2C_PROTCTL.ARBLOIEN = 1.
     * |        |          |0 = An arbitration has not been lost.
     * |        |          |1 = An arbitration has been lost.
     * |        |          |It is cleared by software writing one into this bit
     * |[12]    |ERRIF     |Error Interrupt Flag
     * |        |          |This bit indicates that a Bus Error occurs when a START or STOP condition is present at an illegal position in the formation frame
     * |        |          |Example of illegal position are during the serial transfer of an address byte, a data byte or an acknowledge bit
     * |        |          |A protocol interrupt can be generated if UI2C_PROTCTL.ERRIEN = 1.
     * |        |          |0 = An I2C error has not been detected.
     * |        |          |1 = An I2C error has been detected.
     * |        |          |It is cleared by software writing one into this bit
     * |        |          |Note: This bit is set when slave mode, user must write one into STO register to the defined "not addressed" slave mode.
     * |[13]    |ACKIF     |Acknowledge Received Interrupt Flag
     * |        |          |This bit indicates that an acknowledge has been received in master mode
     * |        |          |This bit is not set in slave mode
     * |        |          |A protocol interrupt can be generated if UI2C_PROTCTL.ACKIEN = 1.
     * |        |          |0 = An acknowledge has not been received.
     * |        |          |1 = An acknowledge has been received.
     * |        |          |It is cleared by software writing one into this bit
     * |[14]    |SLASEL    |Slave Select Status
     * |        |          |This bit indicates that this device has been selected as slave.
     * |        |          |0 = The device is not selected as slave.
     * |        |          |1 = The device is selected as slave.
     * |        |          |Note: This bit has no interrupt signal, and it will be cleared automatically by hardware.
     * |[15]    |SLAREAD   |Slave Read Request Status
     * |        |          |This bit indicates that a slave read request has been detected.
     * |        |          |0 = A slave read request has not been detected.
     * |        |          |1 = A slave read request has been detected.
     * |        |          |Note: This bit has no interrupt signal, and it will be cleared automatically by hardware.
     * |[16]    |WKAKDONE  |Wakeup Address Frame Acknowledge Bit Done
     * |        |          |0 = The ACK bit cycle of address match frame isn't done.
     * |        |          |1 = The ACK bit cycle of address match frame is done in power-down.
     * |        |          |Note: This bit can't release when WKUPIF is set.
     * |[17]    |WRSTSWK   |Read/Write Status Bit in Address Wakeup Frame
     * |        |          |0 = Write command be record on the address match wakeup frame.
     * |        |          |1 = Read command be record on the address match wakeup frame.
     * |[18]    |BUSHANG   |Bus Hang-up
     * |        |          |This bit indicates bus hang-up status
     * |        |          |There is 4-bit counter count when SCL hold high and refer fSAMP_CLK
     * |        |          |The hang-up counter will count to overflow and set this bit when SDA is low
     * |        |          |The counter will be reset by falling edge of SCL signal.
     * |        |          |0 = The bus is normal status for transmission.
     * |        |          |1 = The bus is hang-up status for transmission.
     * |        |          |Note: This bit has no interrupt signal, and it will be cleared automatically by hardware.
     * |[19]    |ERRARBLO  |Error Arbitration Lost
     * |        |          |This bit indicates bus arbitration lost due to bigger noise which is can't be filtered by input processor
     * |        |          |The I2C can send start condition when ERRARBLO is set
     * |        |          |Thus this bit doesn't be cared on slave mode.
     * |        |          |0 = The bus is normal status for transmission.
     * |        |          |1 = The bus is error arbitration lost status for transmission.
     * |        |          |Note: This bit has no interrupt signal, and it will be cleared automatically by hardware.
     * @var UI2C_T::TMCTL
     * Offset: 0x8C  I2C Timing Configure Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |STCTL     |Setup Time Configure Control Register
     * |        |          |This field is used to generate a delay timing between SDA edge and SCL rising edge in transmission mode..
     * |        |          |The delay setup time is numbers of peripheral clock = STCTL x fPCLK.
     * |[11:6]  |HTCTL     |Hold Time Configure Control Register
     * |        |          |This field is used to generate the delay timing between SCL falling edge SDA edge in
     * |        |          |transmission mode.
     * |        |          |The delay hold time is numbers of peripheral clock = HTCTL x fPCLK.
     */
    __IO uint32_t CTL;                   /*!< [0x0000] USCI Control Register                                            */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t BRGEN;                 /*!< [0x0008] USCI Baud Rate Generator Register                                */
    __I  uint32_t RESERVE1[8];
    __IO uint32_t LINECTL;               /*!< [0x002c] USCI Line Control Register                                       */
    __O  uint32_t TXDAT;                 /*!< [0x0030] USCI Transmit Data Register                                      */
    __I  uint32_t RXDAT;                 /*!< [0x0034] USCI Receive Data Register                                       */
    __I  uint32_t RESERVE2[3];
    __IO uint32_t DEVADDR0;              /*!< [0x0044] USCI Device Address Register 0                                   */
    __I  uint32_t RESERVE3[1];
    __IO uint32_t ADDRMSK0;              /*!< [0x004c] USCI Device Address Mask Register 0                              */
    __I  uint32_t RESERVE4[1];
    __IO uint32_t WKCTL;                 /*!< [0x0054] USCI Wake-up Control Register                                    */
    __IO uint32_t WKSTS;                 /*!< [0x0058] USCI Wake-up Status Register                                     */
    __IO uint32_t PROTCTL;               /*!< [0x005c] USCI Protocol Control Register                                   */
    __IO uint32_t PROTIEN;               /*!< [0x0060] USCI Protocol Interrupt Enable Register                          */
    __IO uint32_t PROTSTS;               /*!< [0x0064] USCI Protocol Status Register                                    */
    __I  uint32_t RESERVE5[9];
    __IO uint32_t TMCTL;                 /*!< [0x008c] I2C Timing Configure Control Register                            */

} UI2C_T;

/**
    @addtogroup UI2C_CONST UI2C Bit Field Definition
    Constant Definitions for UI2C Controller
@{ */

#define UI2C_CTL_FUNMODE_Pos             (0)                                               /*!< UI2C_T::CTL: FUNMODE Position          */
#define UI2C_CTL_FUNMODE_Msk             (0x7ul << UI2C_CTL_FUNMODE_Pos)                   /*!< UI2C_T::CTL: FUNMODE Mask              */

#define UI2C_BRGEN_RCLKSEL_Pos           (0)                                               /*!< UI2C_T::BRGEN: RCLKSEL Position        */
#define UI2C_BRGEN_RCLKSEL_Msk           (0x1ul << UI2C_BRGEN_RCLKSEL_Pos)                 /*!< UI2C_T::BRGEN: RCLKSEL Mask            */

#define UI2C_BRGEN_PTCLKSEL_Pos          (1)                                               /*!< UI2C_T::BRGEN: PTCLKSEL Position       */
#define UI2C_BRGEN_PTCLKSEL_Msk          (0x1ul << UI2C_BRGEN_PTCLKSEL_Pos)                /*!< UI2C_T::BRGEN: PTCLKSEL Mask           */

#define UI2C_BRGEN_SPCLKSEL_Pos          (2)                                               /*!< UI2C_T::BRGEN: SPCLKSEL Position       */
#define UI2C_BRGEN_SPCLKSEL_Msk          (0x3ul << UI2C_BRGEN_SPCLKSEL_Pos)                /*!< UI2C_T::BRGEN: SPCLKSEL Mask           */

#define UI2C_BRGEN_TMCNTEN_Pos           (4)                                               /*!< UI2C_T::BRGEN: TMCNTEN Position        */
#define UI2C_BRGEN_TMCNTEN_Msk           (0x1ul << UI2C_BRGEN_TMCNTEN_Pos)                 /*!< UI2C_T::BRGEN: TMCNTEN Mask            */

#define UI2C_BRGEN_TMCNTSRC_Pos          (5)                                               /*!< UI2C_T::BRGEN: TMCNTSRC Position       */
#define UI2C_BRGEN_TMCNTSRC_Msk          (0x1ul << UI2C_BRGEN_TMCNTSRC_Pos)                /*!< UI2C_T::BRGEN: TMCNTSRC Mask           */

#define UI2C_BRGEN_PDSCNT_Pos            (8)                                               /*!< UI2C_T::BRGEN: PDSCNT Position         */
#define UI2C_BRGEN_PDSCNT_Msk            (0x3ul << UI2C_BRGEN_PDSCNT_Pos)                  /*!< UI2C_T::BRGEN: PDSCNT Mask             */

#define UI2C_BRGEN_DSCNT_Pos             (10)                                              /*!< UI2C_T::BRGEN: DSCNT Position          */
#define UI2C_BRGEN_DSCNT_Msk             (0x1ful << UI2C_BRGEN_DSCNT_Pos)                  /*!< UI2C_T::BRGEN: DSCNT Mask              */

#define UI2C_BRGEN_CLKDIV_Pos            (16)                                              /*!< UI2C_T::BRGEN: CLKDIV Position         */
#define UI2C_BRGEN_CLKDIV_Msk            (0x3fful << UI2C_BRGEN_CLKDIV_Pos)                /*!< UI2C_T::BRGEN: CLKDIV Mask             */

#define UI2C_LINECTL_LSB_Pos             (0)                                               /*!< UI2C_T::LINECTL: LSB Position          */
#define UI2C_LINECTL_LSB_Msk             (0x1ul << UI2C_LINECTL_LSB_Pos)                   /*!< UI2C_T::LINECTL: LSB Mask              */

#define UI2C_LINECTL_DWIDTH_Pos          (8)                                               /*!< UI2C_T::LINECTL: DWIDTH Position       */
#define UI2C_LINECTL_DWIDTH_Msk          (0xful << UI2C_LINECTL_DWIDTH_Pos)                /*!< UI2C_T::LINECTL: DWIDTH Mask           */

#define UI2C_TXDAT_TXDAT_Pos             (0)                                               /*!< UI2C_T::TXDAT: TXDAT Position          */
#define UI2C_TXDAT_TXDAT_Msk             (0xfffful << UI2C_TXDAT_TXDAT_Pos)                /*!< UI2C_T::TXDAT: TXDAT Mask              */

#define UI2C_RXDAT_RXDAT_Pos             (0)                                               /*!< UI2C_T::RXDAT: RXDAT Position          */
#define UI2C_RXDAT_RXDAT_Msk             (0xfffful << UI2C_RXDAT_RXDAT_Pos)                /*!< UI2C_T::RXDAT: RXDAT Mask              */

#define UI2C_DEVADDR0_DEVADDR_Pos        (0)                                               /*!< UI2C_T::DEVADDR0: DEVADDR Position     */
#define UI2C_DEVADDR0_DEVADDR_Msk        (0x3fful << UI2C_DEVADDR0_DEVADDR_Pos)            /*!< UI2C_T::DEVADDR0: DEVADDR Mask         */

#define UI2C_ADDRMSK0_ADDRMSK_Pos        (0)                                               /*!< UI2C_T::ADDRMSK0: ADDRMSK Position     */
#define UI2C_ADDRMSK0_ADDRMSK_Msk        (0x3fful << UI2C_ADDRMSK0_ADDRMSK_Pos)            /*!< UI2C_T::ADDRMSK0: ADDRMSK Mask         */

#define UI2C_WKCTL_WKEN_Pos              (0)                                               /*!< UI2C_T::WKCTL: WKEN Position           */
#define UI2C_WKCTL_WKEN_Msk              (0x1ul << UI2C_WKCTL_WKEN_Pos)                    /*!< UI2C_T::WKCTL: WKEN Mask               */

#define UI2C_WKCTL_WKADDREN_Pos          (1)                                               /*!< UI2C_T::WKCTL: WKADDREN Position       */
#define UI2C_WKCTL_WKADDREN_Msk          (0x1ul << UI2C_WKCTL_WKADDREN_Pos)                /*!< UI2C_T::WKCTL: WKADDREN Mask           */

#define UI2C_WKSTS_WKF_Pos               (0)                                               /*!< UI2C_T::WKSTS: WKF Position            */
#define UI2C_WKSTS_WKF_Msk               (0x1ul << UI2C_WKSTS_WKF_Pos)                     /*!< UI2C_T::WKSTS: WKF Mask                */

#define UI2C_PROTCTL_GCFUNC_Pos          (0)                                               /*!< UI2C_T::PROTCTL: GCFUNC Position       */
#define UI2C_PROTCTL_GCFUNC_Msk          (0x1ul << UI2C_PROTCTL_GCFUNC_Pos)                /*!< UI2C_T::PROTCTL: GCFUNC Mask           */

#define UI2C_PROTCTL_AA_Pos              (1)                                               /*!< UI2C_T::PROTCTL: AA Position           */
#define UI2C_PROTCTL_AA_Msk              (0x1ul << UI2C_PROTCTL_AA_Pos)                    /*!< UI2C_T::PROTCTL: AA Mask               */

#define UI2C_PROTCTL_STO_Pos             (2)                                               /*!< UI2C_T::PROTCTL: STO Position          */
#define UI2C_PROTCTL_STO_Msk             (0x1ul << UI2C_PROTCTL_STO_Pos)                   /*!< UI2C_T::PROTCTL: STO Mask              */

#define UI2C_PROTCTL_STA_Pos             (3)                                               /*!< UI2C_T::PROTCTL: STA Position          */
#define UI2C_PROTCTL_STA_Msk             (0x1ul << UI2C_PROTCTL_STA_Pos)                   /*!< UI2C_T::PROTCTL: STA Mask              */

#define UI2C_PROTCTL_ADDR10EN_Pos        (4)                                               /*!< UI2C_T::PROTCTL: ADDR10EN Position     */
#define UI2C_PROTCTL_ADDR10EN_Msk        (0x1ul << UI2C_PROTCTL_ADDR10EN_Pos)              /*!< UI2C_T::PROTCTL: ADDR10EN Mask         */

#define UI2C_PROTCTL_PTRG_Pos            (5)                                               /*!< UI2C_T::PROTCTL: PTRG Position         */
#define UI2C_PROTCTL_PTRG_Msk            (0x1ul << UI2C_PROTCTL_PTRG_Pos)                  /*!< UI2C_T::PROTCTL: PTRG Mask             */

#define UI2C_PROTCTL_TOCNT_Pos           (16)                                              /*!< UI2C_T::PROTCTL: TOCNT Position        */
#define UI2C_PROTCTL_TOCNT_Msk           (0x3fful << UI2C_PROTCTL_TOCNT_Pos)               /*!< UI2C_T::PROTCTL: TOCNT Mask            */

#define UI2C_PROTCTL_PROTEN_Pos          (31)                                              /*!< UI2C_T::PROTCTL: PROTEN Position       */
#define UI2C_PROTCTL_PROTEN_Msk          (0x1ul << UI2C_PROTCTL_PROTEN_Pos)                /*!< UI2C_T::PROTCTL: PROTEN Mask           */

#define UI2C_PROTIEN_TOIEN_Pos           (0)                                               /*!< UI2C_T::PROTIEN: TOIEN Position        */
#define UI2C_PROTIEN_TOIEN_Msk           (0x1ul << UI2C_PROTIEN_TOIEN_Pos)                 /*!< UI2C_T::PROTIEN: TOIEN Mask            */

#define UI2C_PROTIEN_STARIEN_Pos         (1)                                               /*!< UI2C_T::PROTIEN: STARIEN Position      */
#define UI2C_PROTIEN_STARIEN_Msk         (0x1ul << UI2C_PROTIEN_STARIEN_Pos)               /*!< UI2C_T::PROTIEN: STARIEN Mask          */

#define UI2C_PROTIEN_STORIEN_Pos         (2)                                               /*!< UI2C_T::PROTIEN: STORIEN Position      */
#define UI2C_PROTIEN_STORIEN_Msk         (0x1ul << UI2C_PROTIEN_STORIEN_Pos)               /*!< UI2C_T::PROTIEN: STORIEN Mask          */

#define UI2C_PROTIEN_NACKIEN_Pos         (3)                                               /*!< UI2C_T::PROTIEN: NACKIEN Position      */
#define UI2C_PROTIEN_NACKIEN_Msk         (0x1ul << UI2C_PROTIEN_NACKIEN_Pos)               /*!< UI2C_T::PROTIEN: NACKIEN Mask          */

#define UI2C_PROTIEN_ARBLOIEN_Pos        (4)                                               /*!< UI2C_T::PROTIEN: ARBLOIEN Position     */
#define UI2C_PROTIEN_ARBLOIEN_Msk        (0x1ul << UI2C_PROTIEN_ARBLOIEN_Pos)              /*!< UI2C_T::PROTIEN: ARBLOIEN Mask         */

#define UI2C_PROTIEN_ERRIEN_Pos          (5)                                               /*!< UI2C_T::PROTIEN: ERRIEN Position       */
#define UI2C_PROTIEN_ERRIEN_Msk          (0x1ul << UI2C_PROTIEN_ERRIEN_Pos)                /*!< UI2C_T::PROTIEN: ERRIEN Mask           */

#define UI2C_PROTIEN_ACKIEN_Pos          (6)                                               /*!< UI2C_T::PROTIEN: ACKIEN Position       */
#define UI2C_PROTIEN_ACKIEN_Msk          (0x1ul << UI2C_PROTIEN_ACKIEN_Pos)                /*!< UI2C_T::PROTIEN: ACKIEN Mask           */

#define UI2C_PROTSTS_TOIF_Pos            (5)                                               /*!< UI2C_T::PROTSTS: TOIF Position         */
#define UI2C_PROTSTS_TOIF_Msk            (0x1ul << UI2C_PROTSTS_TOIF_Pos)                  /*!< UI2C_T::PROTSTS: TOIF Mask             */

#define UI2C_PROTSTS_ONBUSY_Pos          (6)                                               /*!< UI2C_T::PROTSTS: ONBUSY Position       */
#define UI2C_PROTSTS_ONBUSY_Msk          (0x1ul << UI2C_PROTSTS_ONBUSY_Pos)                /*!< UI2C_T::PROTSTS: ONBUSY Mask           */

#define UI2C_PROTSTS_STARIF_Pos          (8)                                               /*!< UI2C_T::PROTSTS: STARIF Position       */
#define UI2C_PROTSTS_STARIF_Msk          (0x1ul << UI2C_PROTSTS_STARIF_Pos)                /*!< UI2C_T::PROTSTS: STARIF Mask           */

#define UI2C_PROTSTS_STORIF_Pos          (9)                                               /*!< UI2C_T::PROTSTS: STORIF Position       */
#define UI2C_PROTSTS_STORIF_Msk          (0x1ul << UI2C_PROTSTS_STORIF_Pos)                /*!< UI2C_T::PROTSTS: STORIF Mask           */

#define UI2C_PROTSTS_NACKIF_Pos          (10)                                              /*!< UI2C_T::PROTSTS: NACKIF Position       */
#define UI2C_PROTSTS_NACKIF_Msk          (0x1ul << UI2C_PROTSTS_NACKIF_Pos)                /*!< UI2C_T::PROTSTS: NACKIF Mask           */

#define UI2C_PROTSTS_ARBLOIF_Pos         (11)                                              /*!< UI2C_T::PROTSTS: ARBLOIF Position      */
#define UI2C_PROTSTS_ARBLOIF_Msk         (0x1ul << UI2C_PROTSTS_ARBLOIF_Pos)               /*!< UI2C_T::PROTSTS: ARBLOIF Mask          */

#define UI2C_PROTSTS_ERRIF_Pos           (12)                                              /*!< UI2C_T::PROTSTS: ERRIF Position        */
#define UI2C_PROTSTS_ERRIF_Msk           (0x1ul << UI2C_PROTSTS_ERRIF_Pos)                 /*!< UI2C_T::PROTSTS: ERRIF Mask            */

#define UI2C_PROTSTS_ACKIF_Pos           (13)                                              /*!< UI2C_T::PROTSTS: ACKIF Position        */
#define UI2C_PROTSTS_ACKIF_Msk           (0x1ul << UI2C_PROTSTS_ACKIF_Pos)                 /*!< UI2C_T::PROTSTS: ACKIF Mask            */

#define UI2C_PROTSTS_SLASEL_Pos          (14)                                              /*!< UI2C_T::PROTSTS: SLASEL Position       */
#define UI2C_PROTSTS_SLASEL_Msk          (0x1ul << UI2C_PROTSTS_SLASEL_Pos)                /*!< UI2C_T::PROTSTS: SLASEL Mask           */

#define UI2C_PROTSTS_SLAREAD_Pos         (15)                                              /*!< UI2C_T::PROTSTS: SLAREAD Position      */
#define UI2C_PROTSTS_SLAREAD_Msk         (0x1ul << UI2C_PROTSTS_SLAREAD_Pos)               /*!< UI2C_T::PROTSTS: SLAREAD Mask          */

#define UI2C_PROTSTS_WKAKDONE_Pos        (16)                                              /*!< UI2C_T::PROTSTS: WKAKDONE Position     */
#define UI2C_PROTSTS_WKAKDONE_Msk        (0x1ul << UI2C_PROTSTS_WKAKDONE_Pos)              /*!< UI2C_T::PROTSTS: WKAKDONE Mask         */

#define UI2C_PROTSTS_WRSTSWK_Pos         (17)                                              /*!< UI2C_T::PROTSTS: WRSTSWK Position      */
#define UI2C_PROTSTS_WRSTSWK_Msk         (0x1ul << UI2C_PROTSTS_WRSTSWK_Pos)               /*!< UI2C_T::PROTSTS: WRSTSWK Mask          */

#define UI2C_PROTSTS_BUSHANG_Pos         (18)                                              /*!< UI2C_T::PROTSTS: BUSHANG Position      */
#define UI2C_PROTSTS_BUSHANG_Msk         (0x1ul << UI2C_PROTSTS_BUSHANG_Pos)               /*!< UI2C_T::PROTSTS: BUSHANG Mask          */

#define UI2C_PROTSTS_ERRARBLO_Pos        (19)                                              /*!< UI2C_T::PROTSTS: ERRARBLO Position     */
#define UI2C_PROTSTS_ERRARBLO_Msk        (0x1ul << UI2C_PROTSTS_ERRARBLO_Pos)              /*!< UI2C_T::PROTSTS: ERRARBLO Mask         */

#define UI2C_TMCTL_STCTL_Pos             (0)                                               /*!< UI2C_T::TMCTL: STCTL Position          */
#define UI2C_TMCTL_STCTL_Msk             (0x3ful << UI2C_TMCTL_STCTL_Pos)                  /*!< UI2C_T::TMCTL: STCTL Mask              */

#define UI2C_TMCTL_HTCTL_Pos             (6)                                               /*!< UI2C_T::TMCTL: HTCTL Position          */
#define UI2C_TMCTL_HTCTL_Msk             (0x3ful << UI2C_TMCTL_HTCTL_Pos)                  /*!< UI2C_T::TMCTL: HTCTL Mask              */

/**@}*/ /* UI2C_CONST */
/**@}*/ /* end of UI2C register group */



/*---------------------- Hardware Divider -------------------------*/
/**
    @addtogroup HDIV Hardware Divider(HDIV)
    Memory Mapped Structure for HDIV Controller
@{ */

typedef struct
{


    /**
     * @var HDIV_T::DIVIDEND
     * Offset: 0x00  Dividend Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DIVIDEND  |Dividend Source
     * |        |          |This register is given the dividend of divider before calculation starting.
     * @var HDIV_T::DIVISOR
     * Offset: 0x04  Divisor Source Resister
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |DIVISOR   |Divisor Source
     * |        |          |This register is given the divisor of divider before calculation starts.
     * |        |          |Note: When this register is written, hardware divider will start calculate.
     * @var HDIV_T::QUOTIENT
     * Offset: 0x08  Quotient Result Resister
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |QUOTIENT  |Quotient Result
     * |        |          |This register holds the quotient result of divider after calculation complete.
     * @var HDIV_T::REM
     * Offset: 0x0C  Remainder Result Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |REM       |Remainder Result
     * |        |          |The remainder of hardware divider is 16-bit sign integer (REM[15:0]) with sign extension (REM[31:16]) to 32-bit integer.
     * @var HDIV_T::STATUS
     * Offset: 0x10  Divider Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |DIVBYZERO |Divisor Zero Warning
     * |        |          |0 = The divisor is not 0.
     * |        |          |1 = The divisor is 0.
     * |        |          |Note: The DIVBYZERO flag is used to indicate divide-by-zero situation and updated whenever HDIV_DIVISOR is written
     * |        |          |This register is read only
     */
    __IO uint32_t DIVIDEND;              /*!< [0x0000] Dividend Source Register                                         */
    __IO uint32_t DIVISOR;               /*!< [0x0004] Divisor Source Resister                                          */
    __IO uint32_t QUOTIENT;              /*!< [0x0008] Quotient Result Resister                                         */
    __IO uint32_t REM;                   /*!< [0x000c] Remainder Result Register                                        */
    __I  uint32_t STATUS;                /*!< [0x0010] Divider Status Register                                          */

} HDIV_T;

/**
    @addtogroup HDIV_CONST HDIV Bit Field Definition
    Constant Definitions for HDIV Controller
@{ */

#define HDIV_DIVIDEND_DIVIDEND_Pos       (0)                                               /*!< HDIV_T::DIVIDEND: DIVIDEND Position    */
#define HDIV_DIVIDEND_DIVIDEND_Msk       (0xfffffffful << HDIV_DIVIDEND_DIVIDEND_Pos)      /*!< HDIV_T::DIVIDEND: DIVIDEND Mask        */

#define HDIV_DIVISOR_DIVISOR_Pos         (0)                                               /*!< HDIV_T::DIVISOR: DIVISOR Position      */
#define HDIV_DIVISOR_DIVISOR_Msk         (0xfffful << HDIV_DIVISOR_DIVISOR_Pos)            /*!< HDIV_T::DIVISOR: DIVISOR Mask          */

#define HDIV_QUOTIENT_QUOTIENT_Pos       (0)                                               /*!< HDIV_T::QUOTIENT: QUOTIENT Position    */
#define HDIV_QUOTIENT_QUOTIENT_Msk       (0xfffffffful << HDIV_QUOTIENT_QUOTIENT_Pos)      /*!< HDIV_T::QUOTIENT: QUOTIENT Mask        */

#define HDIV_REM_REM_Pos                 (0)                                               /*!< HDIV_T::REM: REM Position              */
#define HDIV_REM_REM_Msk                 (0xfffffffful << HDIV_REM_REM_Pos)                /*!< HDIV_T::REM: REM Mask                  */

#define HDIV_STATUS_DIVBYZERO_Pos        (1)                                               /*!< HDIV_T::STATUS: DIVBYZERO Position     */
#define HDIV_STATUS_DIVBYZERO_Msk        (0x1ul << HDIV_STATUS_DIVBYZERO_Pos)              /*!< HDIV_T::STATUS: DIVBYZERO Mask         */

/**@}*/ /* HDIV_CONST */
/**@}*/ /* end of HDIV register group */


/*---------------------- Enhanced PWM Generator -------------------------*/
/**
    @addtogroup EPWM Enhanced PWM Generator(EPWM)
    Memory Mapped Structure for EPWM Controller
@{ */

typedef struct
{


    /**
     * @var EPWM_T::NPCTL
     * Offset: 0x00  EPWM Negative Polarity Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |NEGPOLAR0 |PWM0 Negative Polarity Control
     * |        |          |The register bit controls polarity/active state of real PWM output.
     * |        |          |0 = PWM output is active high.
     * |        |          |1 = PWM output is active low.
     * |[1]     |NEGPOLAR1 |PWM1 Negative Polarity Control
     * |        |          |The register bit controls polarity/active state of real PWM output.
     * |        |          |0 = PWM output is active high.
     * |        |          |1 = PWM output is active low.
     * |[2]     |NEGPOLAR2 |PWM2 Negative Polarity Control
     * |        |          |The register bit controls polarity/active state of real PWM output.
     * |        |          |0 = PWM output is active high.
     * |        |          |1 = PWM output is active low.
     * |[3]     |NEGPOLAR3 |PWM3 Negative Polarity Control
     * |        |          |The register bit controls polarity/active state of real PWM output.
     * |        |          |0 = PWM output is active high.
     * |        |          |1 = PWM output is active low.
     * |[4]     |NEGPOLAR4 |PWM4 Negative Polarity Control
     * |        |          |The register bit controls polarity/active state of real PWM output.
     * |        |          |0 = PWM output is active high.
     * |        |          |1 = PWM output is active low.
     * |[5]     |NEGPOLAR5 |PWM5 Negative Polarity Control
     * |        |          |The register bit controls polarity/active state of real PWM output.
     * |        |          |0 = PWM output is active high.
     * |        |          |1 = PWM output is active low.
     * @var EPWM_T::CLKDIV
     * Offset: 0x04  EPWM Clock Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CLKDIV    |EPWM Clock Divider (9 Step Divider)
     * |        |          |Select clock input for PWM timer
     * |        |          |0000 = 1 (HCLK / 2^0).
     * |        |          |0001 = 1/2 (HCLK / 2^1).
     * |        |          |0010 = 1/4 (HCLK / 2^2).
     * |        |          |0011 = 1/8 (HCLK / 2^3).
     * |        |          |0100 = 1/16 (HCLK / 2^4).
     * |        |          |0101 = 1/32 (HCLK / 2^5).
     * |        |          |0110 = 1/64 (HCLK / 2^6).
     * |        |          |0111 = 1/128 (HCLK / 2^7).
     * |        |          |1000 = 1/256 (HCLK / 2^8).
     * |        |          |1001~ 1111 = Reserved.
     * @var EPWM_T::CTL
     * Offset: 0x08  EPWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTEN0    |PWM-timer 0 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[1]     |CNTEN1    |PWM-timer 1 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[2]     |CNTEN2    |PWM-timer 2 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[3]     |CNTEN3    |PWM-timer 3 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[4]     |CNTEN4    |PWM-timer 4 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[5]     |CNTEN5    |PWM-timer 5 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[8]     |CNTMODE   |PWM-timer Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |[17:16] |HCUPDT    |Half Cycle Update Enable for Center-aligned Type
     * |        |          |00= Update PERIOD & CMP at pwm_counter = PERIOD (Period).
     * |        |          |01 = Update PERIOD & CMP at pwm_counter = 0.
     * |        |          |10 = Update PERIOD & CMP at half cycle (counter = 0 & PERIOD, both update).
     * |        |          |11 = Update PERIOD & CMP at pwm_counter = PERIOD (Period).
     * |[20]    |ASYMEN    |Asymmetric Mode in Center-aligned Type
     * |        |          |0 = Symmetric mode in center-aligned type.
     * |        |          |1 = Asymmetric mode in center-aligned type.
     * |[23]    |DBGTRIOFF |PWM Debug Mode Configuration Bit (Available in DEBUG Mode Only)
     * |        |          |0 = Safe mode: The timer is frozen and PWM outputs are shut down Safe state for the inverter
     * |        |          |The timer can still be re-started from where it stops.
     * |        |          |1 = Normal mode: The timer continues to operate normally May be dangerous in some cases since a constant duty cycle is applied to the inverter (no more interrupts serviced).
     * |[24]    |DTCNT01   |Dead-zone 0 Generator Enable/Disable (PWM0 and PWM1 Pair for PWM Group)
     * |        |          |0 = Dead-zone 0 Generator Disabled.
     * |        |          |1 = Dead-zone 0 Generator Enabled.
     * |        |          |Note: When the dead-zone generator is enabled, the pair of PWM0 and PWM1 becomes a complementary pair for PWM group.
     * |[25]    |DTCNT23   |Dead-zone 2 Generator Enable/Disable (PWM2 and PWM3 Pair for PWM Group)
     * |        |          |0 = Dead-zone 2 Generator Disabled.
     * |        |          |1 = Dead-zone 2 Generator Enabled.
     * |        |          |Note: When the dead-zone generator is enabled, the pair of PWM2 and PWM3 becomes a complementary pair for PWM group.
     * |[26]    |DTCNT45   |Dead-zone 4 Generator Enable/Disable (PWM4 and PWM5 Pair for PWM Group)
     * |        |          |0 = Dead-zone 4 Generator Disabled.
     * |        |          |1 = Dead-zone 4 Generator Enabled.
     * |        |          |Note: When the dead-zone generator is enabled, the pair of PWM4 and PWM5 becomes a complementary pair for PWM group.
     * |[27]    |CNTCLR    |Clear PWM Counter Control Bit
     * |        |          |0 = Do not clear PWM counter.
     * |        |          |1 = 16-bit PWM counter cleared to 0x000.
     * |        |          |Note: It is automatically cleared by hardware.
     * |[29:28] |MODE      |PWM Operating Mode Selection
     * |        |          |00 = Independent mode.
     * |        |          |01 = Complementary mode.
     * |        |          |10 = Synchronized mode.
     * |        |          |11 = Reserved.
     * |[30]    |GROUPEN   |Group Bit
     * |        |          |0 = The signals timing of PWM0, PWM2 and PWM4 are independent.
     * |        |          |1 = Unify the signals timing of PWM0, PWM2 and PWM4 in the same phase which is controlled by PWM0.
     * |[31]    |CNTTYPE   |PWM Aligned Type Selection Bit
     * |        |          |0 = Edge-aligned type.
     * |        |          |1 = Center-aligned type.
     * @var EPWM_T::PERIOD
     * Offset: 0x0C  EPWM Period Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD    |PWM Counter/Timer Loaded Value
     * |        |          |PERIODn determines the PWM Period.
     * |        |          |Edge-aligned mode: where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |PWM frequency = PWMxy_CLK[HCLK]/clock divider[EPWM_CLKDIV].
     * |        |          |PWM Clock Cycle = 1 / PWM Freq.
     * |        |          |Period = PWM Clock Cycle * (PERIOD+1).
     * |        |          |Duty = PWM Clock Cycle * CMPn.
     * |        |          |Duty ratio = CMPn/(PERIOD+1).
     * |        |          |CMPn >= PERIOD: PWM output is always high.
     * |        |          |CMPn < PERIOD: PWM low width = (PERIODn-CMPn) unit; PWM high width = (CMP) unit.
     * |        |          |CMPn = 0: PWM always output low.
     * |        |          |Center-aligned mode: where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |Period = 1/ (PWMxy_CLK[HCLK]/clock divider[EPWM_CLKDIV] /(2*(PERIOD+1)).
     * |        |          |Duty ratio = 2* (CMPn/(PERIOD+1)).
     * |        |          |CMPn >= PERIOD: PWM output is always high.
     * |        |          |CMPn < PERIOD: PWM low width = (PERIOD - CMPn) x 2 unit; PWM high width = CMP x 2 unit.
     * |        |          |CMPn = 0: PWM always low.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIODn will take effect in next PWM cycle.
     * @var EPWM_T::CMPDAT
     * Offset: 0x24 ~ 0x38  EPWM Comparator Register 0/1/2/3/4/5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP       |PWM Comparator Register
     * |        |          |CMP determines the PWM Duty.
     * |        |          |Edge-aligned mode: where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |Period = (PERIOD + 1) unit.
     * |        |          |Duty ratio = CMP / (PERIOD+1).
     * |        |          |CMP > PERIOD: PWM output is always high
     * |        |          |CMP <= PERIOD: PWM output high duty = (CMP) unit.
     * |        |          |CMP = 0: PWM always low.
     * |        |          |Center-aligned mode: where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |Period = 2 x (PERIOD + 1) unit.
     * |        |          |Duty ratio = 2 x (CMP / PERIOD).
     * |        |          |CMP > PERIOD: PWM output is always high
     * |        |          |CMP <= PERIOD: PWM output high duty = (2 x CMP) unit.
     * |        |          |CMP = 0: PWM always low.
     * |        |          |(Unit = One PWM clock cycle.......)
     * |        |          |Note: Any write to CMPn will take effect in next PWM cycle.
     * |[31:16] |CMPU      |PWM Comparator Register for UP Counter in Center-aligned Asymmetric Mode
     * |        |          |CMPU > PERIOD: @ up counter PWM output is keep to Max. duty.
     * |        |          |CMPU <= PERIOD: (CMPUn + CMPn) unit.
     * |        |          |CMP <= PERIOD: PWM output high duty = (CMP + CMPU) unit.
     * |        |          |Others: PWM output is always low
     * |        |          |(Unit = One PWM clock cycle).
     * @var EPWM_T::CNT
     * Offset: 0x3C  EPWM Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNT       |PWM Data Register
     * |        |          |User can monitor CNT to know the current value in 16-bit down counter.
     * |[31]    |CNTDIR    |PWM Counter (Up/Down) Direction
     * |        |          |0 = PWM counter is down count.
     * |        |          |1 = PWM counter is up count.
     * @var EPWM_T::INTEN
     * Offset: 0x54  EPWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIEN      |PWM Channel 0 Period Interrupt Enable Bit
     * |        |          |for Edge-aligned and Center-aligned
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM Period.
     * |[8]     |CMPUIEN0  |PWM Channel 0 UP Interrupt Enable Bit
     * |        |          |UP for Center-aligned only
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH0 PWM UP counter reaches EPWM_CMPDAT0.
     * |[9]     |CMPUIEN1  |PWM Channel 1 UP Interrupt Enable Bit
     * |        |          |UP for Center-aligned only
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH1 PWM UP counter reaches EPWM_CMPDAT1.
     * |[10]    |CMPUIEN2  |PWM Channel 2 UP Interrupt Enable Bit
     * |        |          |UP for Center-aligned only
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH2 PWM UP counter reaches EPWM_CMPDAT2.
     * |[11]    |CMPUIEN3  |PWM Channel 3 UP Interrupt Enable Bit
     * |        |          |UP for Center-aligned only
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH3 PWM UP counter reaches EPWM_CMPDAT3.
     * |[12]    |CMPUIEN4  |PWM Channel 4 UP Interrupt Enable Bit
     * |        |          |UP for Center-aligned only
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH4 PWM UP counter reaches EPWM_CMPDAT4.
     * |[13]    |CMPUIEN5  |PWM Channel 5 UP Interrupt Enable Bit
     * |        |          |UP for Center-aligned only
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH5 PWM UP counter reaches EPWM_CMPDAT5.
     * |[16]    |BRK0IEN   |Enable Fault Brake0 Interrupt Bit
     * |        |          |0 = Disabling flags BRK0IF to trigger PWM interrupt.
     * |        |          |1 = Enabling flags BRK0IF can trigger PWM interrupt.
     * |[17]    |BRK1IEN   |Enable Fault Brake1 Interrupt Bit
     * |        |          |0 = Disabling flags BRK1IF to trigger PWM interrupt.
     * |        |          |1 = Enabling flags BRK1IF can trigger PWM interrupt.
     * |[18]    |CIEN      |PWM Central Interrupt Enable Bit
     * |        |          |for Center-aligned only
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM Central.
     * |[24]    |CMPDIEN0  |PWM Channel 0 DOWN Interrupt Enable Bit
     * |        |          |DOWN for Edge-aligned and Center-aligned
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH0 PWM DOWN counter reaches EPWM_CMPDAT0.
     * |[25]    |CMPDIEN1  |PWM Channel 1 DOWN Interrupt Enable Bit
     * |        |          |DOWN for Edge-aligned and Center-aligned
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH1 PWM DOWN counter reaches EPWM_CMPDAT1.
     * |[26]    |CMPDIEN2  |PWM Channel 2 DOWN Interrupt Enable Bit
     * |        |          |DOWN for Edge-aligned and Center-aligned
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH2 PWM DOWN counter reaches EPWM_CMPDAT2.
     * |[27]    |CMPDIEN3  |PWM Channel 3 DOWN Interrupt Enable Bit
     * |        |          |DOWN for Edge-aligned and Center-aligned
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH3 PWM DOWN counter reaches EPWM_CMPDAT3.
     * |[28]    |CMPDIEN4  |PWM Channel 4 DOWN Interrupt Enable Bit
     * |        |          |DOWN for Edge-aligned and Center-aligned
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH4 PWM DOWN counter reaches EPWM_CMPDAT4.
     * |[29]    |CMPDIEN5  |PWM Channel 5 DOWN Interrupt Enable Bit
     * |        |          |DOWN for Edge-aligned and Center-aligned
     * |        |          |0 = Disable.
     * |        |          |1 = Enabled interruptwhen EPWM_CH5 PWM DOWN counter reaches EPWM_CMPDAT5.
     * @var EPWM_T::INTSTS
     * Offset: 0x58  EPWM Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIF       |PWM Channel 0 Period Interrupt Flag
     * |        |          |Edge-aligned mode:
     * |        |          |Flag is set by hardware when PWM down counter reaches zero point.
     * |        |          |Center-aligned mode:
     * |        |          |Flag is set by hardware when PWM down counter reaches zero point and then up counter reaches EPWM_PERIOD.
     * |        |          |Software can write 1 to clear this bit.
     * |[8]     |CMPUIF0   |PWM Channel 0 UP Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 0 PWM UP counter reaches PWM_CMPDAT0
     * |        |          |Software can write 1 to clear this bit.
     * |[9]     |CMPUIF1   |PWM Channel 1 UP Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 1 PWM UP counter reaches PWM_CMPDAT1
     * |        |          |Software can write 1 to clear this bit.
     * |[10]    |CMPUIF2   |PWM Channel 2 UP Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 2 PWM UP counter reaches PWM_CMPDAT2
     * |        |          |Software can write 1 to clear this bit.
     * |[11]    |CMPUIF3   |PWM Channel 3 UP Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 3 PWMUP counter reaches PWM_CMPDAT3
     * |        |          |Software can write 1 to clear this bit.
     * |[12]    |CMPUIF4   |PWM Channel 4 UP Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 4 PWM UP counter reaches PWM_CMPDAT4
     * |        |          |Software can write 1 to clear this bit.
     * |[13]    |CMPUIF5   |PWM Channel 5 UP Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 5 PWM UP counter reaches PWM_CMPDAT5
     * |        |          |Software can write 1 to clear this bit.
     * |[16]    |BRK0IF    |PWM Brake0 Flag
     * |        |          |0 = PWM Brake does not recognize a falling signal at BKP0.
     * |        |          |1 = When PWM Brake detects a falling signal at pin BKP0, this flag will be set to high.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[17]    |BRK1IF    |PWM Brake1 Flag
     * |        |          |0 = PWM Brake does not recognize a falling signal at BKP1.
     * |        |          |1 = When PWM Brake detects a falling signal at pin BKP1, this flag will be set to high.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[18]    |CIF       |PWM Channel 0 Central Interrupt Flag
     * |        |          |Flag is set by hardware when PWM down counter reaches zero point
     * |        |          |Software can write 1 to clear this bit.
     * |[24]    |CMPDIF0   |PWM Channel 0 DOWN Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 0 PWM DOWN counter reaches EPWM_CMPDAT0
     * |        |          |Software can write 1 to clear this bit.
     * |[25]    |CMPDIF1   |PWM Channel 1 DOWN Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 1 PWM DOWN counter reaches EPWM_CMPDAT1
     * |        |          |Software can write 1 to clear this bit.
     * |[26]    |CMPDIF2   |PWM Channel 2 DOWN Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 2 PWM DOWN counter reaches EPWM_CMPDAT2
     * |        |          |Software can write 1 to clear this bit.
     * |[27]    |CMPDIF3   |PWM Channel 3 DOWN Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 3 PWM DOWN counter reaches EPWM_CMPDAT3
     * |        |          |Software can write 1 to clear this bit.
     * |[28]    |CMPDIF4   |PWM Channel 4 DOWN Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 4 PWM DOWN counter reaches EPWM_CMPDAT4
     * |        |          |Software can write 1 to clear this bit.
     * |[29]    |CMPDIF5   |PWM Channel 5 DOWN Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 5 PWM DOWN counter reaches EPWM_CMPDAT5
     * |        |          |Software can write 1 to clear this bit.
     * @var EPWM_T::RESDLY
     * Offset: 0x5C  EPWM BRK Low Voltage Detect Resume Delay
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |DELAY     |PWM BRK Low Voltage Detect Resume Delay
     * |        |          |12 bits Down-Counter
     * @var EPWM_T::BRKCTL
     * Offset: 0x60  EPWM Fault Brake Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BRK0EN    |Brake0 Function Enable Bit
     * |        |          |0 = Brake0 detect function Disabled.
     * |        |          |1 = Brake0 detect function Enabled.
     * |[1]     |BRK1EN    |Brake1 Function Enable Bit
     * |        |          |0 = Brake1 detect function Disabled.
     * |        |          |1 = Brake1 detect function Enabled.
     * |[2]     |BRK0A0EN  |BRK0 Source From ACMP0 Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[3]     |BRK0A1EN  |BRK0 Source From ACMP1 Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[4]     |BK0ADCEN  |BRK0 Source From EADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[5]     |BRK0PEN   |BRK0 Source From External Pin Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[9]     |SWBRK     |Software Break
     * |        |          |0 = Disable Software break and back to normal PWM function.
     * |        |          |1 = Issue Software break.
     * |[10]    |BRK1A0EN  |BRK1 Source From ACMP0 Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[11]    |BRK1A1EN  |BRK1 Source From ACMP1 Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[12]    |BK1ADCEN  |BRK1 Source From EADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[13]    |BRK1PEN   |BRK1 Source From External Pin Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[14]    |LVDBKEN   |Low-level Detection Trigger PWM Brake Function 1 Enable Bit
     * |        |          |0 = Brake Function 1 triggered by Low-level detection Disabled.
     * |        |          |1 = Brake Function 1 triggered by Low-level detection Enabled.
     * |[15]    |LVDTYPE   |Low-level Detection Resume Type
     * |        |          |0 = Brake resume at BRK resume delay counter counting to 0.
     * |        |          |1 = Brake resume at period edge.
     * |[24]    |BKOD0     |PWM Channel 0 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[25]    |BKOD1     |PWM Channel 1 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[26]    |BKOD2     |PWM Channel 2 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[27]    |BKOD3     |PWM Channel 3 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[28]    |BKOD4     |PWM Channel 4 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[29]    |BKOD5     |PWM Channel 5 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[31]    |NFPEN     |Noise Filter for External Brake Input Pin (BRAKE) Enable Bit
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * @var EPWM_T::DTCTL
     * Offset: 0x64  EPWM Dead-zone Interval Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DTCNT01   |Dead-zone Interval Register for Pair of Channel0 and Channel1 (PWM0 and PWM1 Pair)
     * |        |          |These 8 bits determine dead-zone length.
     * |        |          |The unit time of dead-zone length is received from corresponding EPWM_CLKDIV bits.
     * |[15:8]  |DTCNT23   |Dead-zone Interval Register for Pair of Channel2 and Channel3 (PWM2 and PWM3 Pair)
     * |        |          |These 8 bits determine dead-zone length.
     * |        |          |The unit time of dead-zone length is received from corresponding EPWM_CLKDIV bits.
     * |[23:16] |DTCNT45   |Dead-zone Interval Register for Pair of Channel4 and Channel5 (PWM4 and PWM5 Pair)
     * |        |          |These 8 bits determine dead-zone length.
     * |        |          |The unit time of dead-zone length is received from corresponding EPWM_CLKDIV bits.
     * @var EPWM_T::PHCHG
     * Offset: 0x78  EPWM Phase Changed Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MSKDAT0   |Enable PWM0 Mask Data
     * |        |          |0 = PWM0 state is masked with zero.
     * |        |          |1 = PWM0 state is masked with one.
     * |[1]     |MSKDAT1   |Enable PWM1 Mask Data
     * |        |          |0 = PWM1 state is masked with zero.
     * |        |          |1 = PWM1 state is masked with one.
     * |[2]     |MSKDAT2   |Enable PWM2 Mask Data
     * |        |          |0 = PWM2 state is masked with zero.
     * |        |          |1 = PWM2 state is masked with one.
     * |[3]     |MSKDAT3   |Enable PWM3 Mask Data
     * |        |          |0 = PWM3 state is masked with zero.
     * |        |          |1 = PWM3 state is masked with one.
     * |[4]     |MSKDAT4   |Enable PWM4 Mask Data
     * |        |          |0 = PWM4 state is masked with zero.
     * |        |          |1 = PWM4 state is masked with one.
     * |[5]     |MSKDAT5   |Enable PWM5 Mask Data
     * |        |          |0 = PWM5 state is masked with zero.
     * |        |          |1 = PWM5 state is masked with one.
     * |[8]     |MSKEN0    |Enable PWM0 Mask Function
     * |        |          |0 = PWM0 Mask Function Disabled.
     * |        |          |1 = PWM0 Mask Function Enabled.
     * |[9]     |MSKEN1    |Enable PWM1 Mask Function
     * |        |          |0 = PWM1 Mask Function Disabled.
     * |        |          |1 = PWM1 Mask Function Enabled.
     * |[10]    |MSKEN2    |Enable PWM2 Mask Function
     * |        |          |0 = PWM2 Mask Function Disabled.
     * |        |          |1 = PWM2 Mask Function Enabled.
     * |[11]    |MSKEN3    |Enable PWM3 Mask Function
     * |        |          |0 = PWM3 Mask Function Disabled.
     * |        |          |1 = PWM3 Mask Function Enabled.
     * |[12]    |MSKEN4    |Enable PWM4 Mask Function
     * |        |          |0 = PWM4 Mask Function Disabled.
     * |        |          |1 = PWM4 Mask Function Enabled.
     * |[13]    |MSKEN5    |Enable PWM5 Mask Function
     * |        |          |0 = PWM5 Mask Function Disabled.
     * |        |          |1 = PWM5 Mask Function Enabled.
     * |[22:20] |TRGSEL    |Phase Change Trigger Selection
     * |        |          |Select the trigger condition to load PHCHG from PHCHG_NXT.
     * |        |          |When the trigger condition occurs it will load PHCHG_NOW with PHCHG_NXT.
     * |        |          |Phase Change: PWM outputs are masked according with the definition of MSKENn and MSKDATn in PHCHG_NOW.
     * |        |          |000 = Triggered by Timer0 event.
     * |        |          |001 = Triggered by Timer1 event.
     * |        |          |010 = Triggered by Timer2 event.
     * |        |          |011 = Triggered by HALLSTS (EPWM_PHCHGNXT[18:16]) matched hall sensor state.
     * |        |          |100 = Triggered by ACMP0 event.
     * |        |          |101 = Triggered by ACMP1 event.
     * |        |          |110 = Reserved.
     * |        |          |111 = Auto Phase Change Function Disabled.
     * |[25:24] |A0POSSEL  |Alternative Comparator 0 Positive Input Selection
     * |        |          |Select the positive input source of ACMP0.
     * |        |          |00 = Select ACMP0_P0 (PB.0) as the input of ACMP0.
     * |        |          |01 = Select ACMP0_P1 (PB.1) as the input of ACMP0.
     * |        |          |10 = Select ACMP0_P2 (PB.2) as the input of ACMP0.
     * |        |          |11 = Reserved.
     * |[27:26] |A1POSSEL  |Alternative Comparator 1 Positive Input Selection
     * |        |          |Select the positive input source of ACMP1.
     * |        |          |00 = Select ACMP1_P0 (PC.0) as the input of ACMP1.
     * |        |          |01 = Select ACMP1_P1 (PC.1) as the input of ACMP1.
     * |        |          |10 = Select ACMP1_P2 (PD.1) as the input of ACMP1.
     * |        |          |11 = Reserved.
     * |[28]    |ACMP0TEN  |ACMP0 Trigger Function Enable Bit
     * |        |          |0 = ACMP0 trigger PWM function Disabled.
     * |        |          |1 = ACMP0 trigger PWM function Enabled.
     * |[29]    |ACMP1TEN  |ACMP1 Trigger Function Enable Bit
     * |        |          |0 = ACMP1 trigger PWM function Disabled.
     * |        |          |1 = ACMP1 trigger PWM function Enabled.
     * @var EPWM_T::PHCHGNXT
     * Offset: 0x7C  EPWM Next Phase Change Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MSKDAT0   |Enable PWM0 Mask Data Preset Bit
     * |        |          |This bit will be load to bit MSKDAT0 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[1]     |MSKDAT1   |Enable PWM1 Mask Data Preset Bit
     * |        |          |This bit will be load to bit MSKDAT1 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[2]     |MSKDAT2   |Enable PWM2 Mask Data Preset Bit
     * |        |          |This bit will be load to bit MSKDAT2 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[3]     |MSKDAT3   |Enable PWM3 Mask Data Preset Bit
     * |        |          |This bit will be load to bit MSKDAT3 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[4]     |MSKDAT4   |Enable PWM4 Mask Data Preset Bit
     * |        |          |This bit will be load to bit MSKDAT4 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[5]     |MSKDAT5   |Enable PWM5 Mask Data Preset Bit
     * |        |          |This bit will be load to bit MSKDAT5 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[8]     |MSKEN0    |Enable PWM0 Mask Function Preset Bit
     * |        |          |This bit will be load to bit MSKEN0 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[9]     |MSKEN1    |Enable PWM1 Mask Function Preset Bit
     * |        |          |This bit will be load to bit MSKEN1 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[10]    |MSKEN2    |Enable PWM2 Mask Function Preset Bit
     * |        |          |This bit will be load to bit MSKEN2 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[11]    |MSKEN3    |Enable PWM3 Mask Function Preset Bit
     * |        |          |This bit will be load to bit MSKEN3 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[12]    |MSKEN4    |Enable PWM4 Mask Function Preset Bit
     * |        |          |This bit will be load to bit MSKEN4 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[13]    |MSKEN5    |Enable PWM5 Mask Function Preset Bit
     * |        |          |This bit will be load to bit MSKEN5 in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[18:16] |HALLSTS   |Predicted Next HALL State
     * |        |          |This bit field indicates the predicted hall state at next commutation.
     * |        |          |If TRGSEL (EPWM_PHCHG[22:20]) = 0x3,.
     * |        |          |the hardware will compare bits (CAP2, CAP1, CAP0) in timer 2 with HALLSTS [2:0] when any hall state change occurs.
     * |        |          |If the comparison is matched it will trigger phase change function.
     * |[22:20] |TRGSEL    |Phase Change Trigger Selection Preset Bits
     * |        |          |This bit field will be load to bit field TRGSEL in PHCHG_NOW when load trigger condition occurs.
     * |        |          |000 = Triggered by Timer0 event.
     * |        |          |001 = Triggered by Timer1 event.
     * |        |          |010 = Triggered by Timer2 event.
     * |        |          |011 = Triggered by HALLSTS (EPWM_PHCHGNXT[18:16]) matched hall sensor state.
     * |        |          |100 = Triggered by ACMP0 event.
     * |        |          |101 = Triggered by ACMP1 event.
     * |        |          |110 = Reserved.
     * |        |          |111 = Auto Phase Change Function Disabled.
     * |        |          |Refer to register EPWM_PHCHG for detailed definition.
     * |[25:24] |A0POSSEL  |Alternative Comparator 0 Positive Input Selection Preset Bits
     * |        |          |This bit field will be load to bit field A0POSSEL in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[27:26] |A1POSSEL  |Alternative Comparator 1 Positive Input Selection Preset Bits
     * |        |          |This bit field will be load to bit field A1POSSEL in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[28]    |ACMP0TEN  |ACMP0 Trigger Function Control Preset Bit
     * |        |          |This bit will be load to bit ACMP0TEN in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * |[29]    |ACMP1TEN  |ACMP1 Trigger Function Control Preset Bit
     * |        |          |This bit will be load to bit ACMP1TEN in PHCHG_NOW when load trigger condition occurs.
     * |        |          |Refer to register PHCHG_NOW for detailed definition.
     * @var EPWM_T::PHCHGALT
     * Offset: 0x80  EPWM Phase Change Alternative Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |POSCTL0   |Positive Input Control For ACMP0
     * |        |          |0 = The input of ACMP0 is controlled by ACMP_CTL0.
     * |        |          |1 = The input of ACMP0 is controlled by A0POSSEL in PHCHG_NOW register.
     * |        |          |Note: Register ACMP_CTL0 is describe in Comparator Controller chapter
     * |[1]     |POSCTL1   |Positive Input Control For ACMP1
     * |        |          |0 = The input of ACMP1 is controlled by ACMP_CTL1.
     * |        |          |1 = The input of ACMP1 is controlled by A1POSSEL in PHCHG_NOW register.
     * |        |          |Note: Register ACMP_CTL1 is describe in Comparator Controller chapter
     * @var EPWM_T::IFA
     * Offset: 0x84  EPWM Period Interrupt Accumulation Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IFAEN     |Enable Period Interrupt Accumulation Function
     * |        |          |0 = Period Interrupt Accumulation Disabled.
     * |        |          |1 = Period Interrupt Accumulation Enabled.
     * |[7:4]   |IFCNT     |Period Interrupt Accumulation Counter Value Setting Register (Write Only)
     * |        |          |16 step Down-Counter value setting register.
     * |        |          |When IFAEN is set, IFCNT value will load into IFDAT and decrase gradually.
     * |[15:12] |IFDAT     |Period Interrupt Down-counter Data Register (Read Only)
     * |        |          |When IFAEN is set, IFDAT will decrease when every PWM Interrupt flag is set,
     * |        |          |and when IFDAT reach to zero, the PWM interrupt will occurred and IFCNT will reload to IFDAT.
     */
    __IO uint32_t NPCTL;                 /*!< [0x0000] EPWM Negative Polarity Control Register                          */
    __IO uint32_t CLKDIV;                /*!< [0x0004] EPWM Clock Select Register                                       */
    __IO uint32_t CTL;                   /*!< [0x0008] EPWM Control Register                                            */
    __IO uint32_t PERIOD;                /*!< [0x000c] EPWM Period Counter Register                                     */
    __I  uint32_t RESERVE0[5];
    __IO uint32_t CMPDAT[6];             /*!< [0x0024 ~ 0x0038] EPWM Comparator Register 0/1/2/3/4/5                    */
    __I  uint32_t CNT;                   /*!< [0x003c] EPWM Data Register                                               */
    __I  uint32_t RESERVE1[5];
    __IO uint32_t INTEN;                 /*!< [0x0054] EPWM Interrupt Enable Register                                   */
    __IO uint32_t INTSTS;                /*!< [0x0058] EPWM Interrupt Status Register                                   */
    __IO uint32_t RESDLY;                /*!< [0x005c] EPWM BRK Low Voltage Detect Resume Delay                         */
    __IO uint32_t BRKCTL;                /*!< [0x0060] EPWM Fault Brake Control Register                                */
    __IO uint32_t DTCTL;                 /*!< [0x0064] EPWM Dead-zone Interval Register                                 */
    __I  uint32_t RESERVE2[4];
    __IO uint32_t PHCHG;                 /*!< [0x0078] EPWM Phase Changed Register                                      */
    __IO uint32_t PHCHGNXT;              /*!< [0x007c] EPWM Next Phase Change Register                                  */
    __IO uint32_t PHCHGALT;              /*!< [0x0080] EPWM Phase Change Alternative Control Register                   */
    __IO uint32_t IFA;                   /*!< [0x0084] EPWM Period Interrupt Accumulation Control Register              */

} EPWM_T;

/**
    @addtogroup EPWM_CONST EPWM Bit Field Definition
    Constant Definitions for EPWM Controller
@{ */

#define EPWM_NPCTL_NEGPOLAR0_Pos         (0)                                               /*!< EPWM_T::NPCTL: NEGPOLAR0 Position      */
#define EPWM_NPCTL_NEGPOLAR0_Msk         (0x1ul << EPWM_NPCTL_NEGPOLAR0_Pos)               /*!< EPWM_T::NPCTL: NEGPOLAR0 Mask          */

#define EPWM_NPCTL_NEGPOLAR1_Pos         (1)                                               /*!< EPWM_T::NPCTL: NEGPOLAR1 Position      */
#define EPWM_NPCTL_NEGPOLAR1_Msk         (0x1ul << EPWM_NPCTL_NEGPOLAR1_Pos)               /*!< EPWM_T::NPCTL: NEGPOLAR1 Mask          */

#define EPWM_NPCTL_NEGPOLAR2_Pos         (2)                                               /*!< EPWM_T::NPCTL: NEGPOLAR2 Position      */
#define EPWM_NPCTL_NEGPOLAR2_Msk         (0x1ul << EPWM_NPCTL_NEGPOLAR2_Pos)               /*!< EPWM_T::NPCTL: NEGPOLAR2 Mask          */

#define EPWM_NPCTL_NEGPOLAR3_Pos         (3)                                               /*!< EPWM_T::NPCTL: NEGPOLAR3 Position      */
#define EPWM_NPCTL_NEGPOLAR3_Msk         (0x1ul << EPWM_NPCTL_NEGPOLAR3_Pos)               /*!< EPWM_T::NPCTL: NEGPOLAR3 Mask          */

#define EPWM_NPCTL_NEGPOLAR4_Pos         (4)                                               /*!< EPWM_T::NPCTL: NEGPOLAR4 Position      */
#define EPWM_NPCTL_NEGPOLAR4_Msk         (0x1ul << EPWM_NPCTL_NEGPOLAR4_Pos)               /*!< EPWM_T::NPCTL: NEGPOLAR4 Mask          */

#define EPWM_NPCTL_NEGPOLAR5_Pos         (5)                                               /*!< EPWM_T::NPCTL: NEGPOLAR5 Position      */
#define EPWM_NPCTL_NEGPOLAR5_Msk         (0x1ul << EPWM_NPCTL_NEGPOLAR5_Pos)               /*!< EPWM_T::NPCTL: NEGPOLAR5 Mask          */

#define EPWM_CLKDIV_CLKDIV_Pos           (0)                                               /*!< EPWM_T::CLKDIV: CLKDIV Position        */
#define EPWM_CLKDIV_CLKDIV_Msk           (0xful << EPWM_CLKDIV_CLKDIV_Pos)                 /*!< EPWM_T::CLKDIV: CLKDIV Mask            */

#define EPWM_CTL_CNTEN0_Pos              (0)                                               /*!< EPWM_T::CTL: CNTEN0 Position           */
#define EPWM_CTL_CNTEN0_Msk              (0x1ul << EPWM_CTL_CNTEN0_Pos)                    /*!< EPWM_T::CTL: CNTEN0 Mask               */

#define EPWM_CTL_CNTEN1_Pos              (1)                                               /*!< EPWM_T::CTL: CNTEN1 Position           */
#define EPWM_CTL_CNTEN1_Msk              (0x1ul << EPWM_CTL_CNTEN1_Pos)                    /*!< EPWM_T::CTL: CNTEN1 Mask               */

#define EPWM_CTL_CNTEN2_Pos              (2)                                               /*!< EPWM_T::CTL: CNTEN2 Position           */
#define EPWM_CTL_CNTEN2_Msk              (0x1ul << EPWM_CTL_CNTEN2_Pos)                    /*!< EPWM_T::CTL: CNTEN2 Mask               */

#define EPWM_CTL_CNTEN3_Pos              (3)                                               /*!< EPWM_T::CTL: CNTEN3 Position           */
#define EPWM_CTL_CNTEN3_Msk              (0x1ul << EPWM_CTL_CNTEN3_Pos)                    /*!< EPWM_T::CTL: CNTEN3 Mask               */

#define EPWM_CTL_CNTEN4_Pos              (4)                                               /*!< EPWM_T::CTL: CNTEN4 Position           */
#define EPWM_CTL_CNTEN4_Msk              (0x1ul << EPWM_CTL_CNTEN4_Pos)                    /*!< EPWM_T::CTL: CNTEN4 Mask               */

#define EPWM_CTL_CNTEN5_Pos              (5)                                               /*!< EPWM_T::CTL: CNTEN5 Position           */
#define EPWM_CTL_CNTEN5_Msk              (0x1ul << EPWM_CTL_CNTEN5_Pos)                    /*!< EPWM_T::CTL: CNTEN5 Mask               */

#define EPWM_CTL_CNTMODE_Pos             (8)                                               /*!< EPWM_T::CTL: CNTMODE Position          */
#define EPWM_CTL_CNTMODE_Msk             (0x1ul << EPWM_CTL_CNTMODE_Pos)                   /*!< EPWM_T::CTL: CNTMODE Mask              */

#define EPWM_CTL_HCUPDT_Pos              (16)                                              /*!< EPWM_T::CTL: HCUPDT Position           */
#define EPWM_CTL_HCUPDT_Msk              (0x3ul << EPWM_CTL_HCUPDT_Pos)                    /*!< EPWM_T::CTL: HCUPDT Mask               */

#define EPWM_CTL_ASYMEN_Pos              (20)                                              /*!< EPWM_T::CTL: ASYMEN Position           */
#define EPWM_CTL_ASYMEN_Msk              (0x1ul << EPWM_CTL_ASYMEN_Pos)                    /*!< EPWM_T::CTL: ASYMEN Mask               */

#define EPWM_CTL_DBGTRIOFF_Pos           (23)                                              /*!< EPWM_T::CTL: DBGTRIOFF Position        */
#define EPWM_CTL_DBGTRIOFF_Msk           (0x1ul << EPWM_CTL_DBGTRIOFF_Pos)                 /*!< EPWM_T::CTL: DBGTRIOFF Mask            */

#define EPWM_CTL_DTCNT01_Pos             (24)                                              /*!< EPWM_T::CTL: DTCNT01 Position          */
#define EPWM_CTL_DTCNT01_Msk             (0x1ul << EPWM_CTL_DTCNT01_Pos)                   /*!< EPWM_T::CTL: DTCNT01 Mask              */

#define EPWM_CTL_DTCNT23_Pos             (25)                                              /*!< EPWM_T::CTL: DTCNT23 Position          */
#define EPWM_CTL_DTCNT23_Msk             (0x1ul << EPWM_CTL_DTCNT23_Pos)                   /*!< EPWM_T::CTL: DTCNT23 Mask              */

#define EPWM_CTL_DTCNT45_Pos             (26)                                              /*!< EPWM_T::CTL: DTCNT45 Position          */
#define EPWM_CTL_DTCNT45_Msk             (0x1ul << EPWM_CTL_DTCNT45_Pos)                   /*!< EPWM_T::CTL: DTCNT45 Mask              */

#define EPWM_CTL_CNTCLR_Pos              (27)                                              /*!< EPWM_T::CTL: CNTCLR Position           */
#define EPWM_CTL_CNTCLR_Msk              (0x1ul << EPWM_CTL_CNTCLR_Pos)                    /*!< EPWM_T::CTL: CNTCLR Mask               */

#define EPWM_CTL_MODE_Pos                (28)                                              /*!< EPWM_T::CTL: MODE Position             */
#define EPWM_CTL_MODE_Msk                (0x3ul << EPWM_CTL_MODE_Pos)                      /*!< EPWM_T::CTL: MODE Mask                 */

#define EPWM_CTL_GROUPEN_Pos             (30)                                              /*!< EPWM_T::CTL: GROUPEN Position          */
#define EPWM_CTL_GROUPEN_Msk             (0x1ul << EPWM_CTL_GROUPEN_Pos)                   /*!< EPWM_T::CTL: GROUPEN Mask              */

#define EPWM_CTL_CNTTYPE_Pos             (31)                                              /*!< EPWM_T::CTL: CNTTYPE Position          */
#define EPWM_CTL_CNTTYPE_Msk             (0x1ul << EPWM_CTL_CNTTYPE_Pos)                   /*!< EPWM_T::CTL: CNTTYPE Mask              */

#define EPWM_PERIOD_PERIOD_Pos           (0)                                               /*!< EPWM_T::PERIOD: PERIOD Position        */
#define EPWM_PERIOD_PERIOD_Msk           (0xfffful << EPWM_PERIOD_PERIOD_Pos)              /*!< EPWM_T::PERIOD: PERIOD Mask            */

#define EPWM_CMPDAT0_CMP_Pos             (0)                                               /*!< EPWM_T::CMPDAT0: CMP Position          */
#define EPWM_CMPDAT0_CMP_Msk             (0xfffful << EPWM_CMPDAT0_CMP_Pos)                /*!< EPWM_T::CMPDAT0: CMP Mask              */

#define EPWM_CMPDAT0_CMPU_Pos            (16)                                              /*!< EPWM_T::CMPDAT0: CMPU Position         */
#define EPWM_CMPDAT0_CMPU_Msk            (0xfffful << EPWM_CMPDAT0_CMPU_Pos)               /*!< EPWM_T::CMPDAT0: CMPU Mask             */

#define EPWM_CMPDAT1_CMP_Pos             (0)                                               /*!< EPWM_T::CMPDAT1: CMP Position          */
#define EPWM_CMPDAT1_CMP_Msk             (0xfffful << EPWM_CMPDAT1_CMP_Pos)                /*!< EPWM_T::CMPDAT1: CMP Mask              */

#define EPWM_CMPDAT1_CMPU_Pos            (16)                                              /*!< EPWM_T::CMPDAT1: CMPU Position         */
#define EPWM_CMPDAT1_CMPU_Msk            (0xfffful << EPWM_CMPDAT1_CMPU_Pos)               /*!< EPWM_T::CMPDAT1: CMPU Mask             */

#define EPWM_CMPDAT2_CMP_Pos             (0)                                               /*!< EPWM_T::CMPDAT2: CMP Position          */
#define EPWM_CMPDAT2_CMP_Msk             (0xfffful << EPWM_CMPDAT2_CMP_Pos)                /*!< EPWM_T::CMPDAT2: CMP Mask              */

#define EPWM_CMPDAT2_CMPU_Pos            (16)                                              /*!< EPWM_T::CMPDAT2: CMPU Position         */
#define EPWM_CMPDAT2_CMPU_Msk            (0xfffful << EPWM_CMPDAT2_CMPU_Pos)               /*!< EPWM_T::CMPDAT2: CMPU Mask             */

#define EPWM_CMPDAT3_CMP_Pos             (0)                                               /*!< EPWM_T::CMPDAT3: CMP Position          */
#define EPWM_CMPDAT3_CMP_Msk             (0xfffful << EPWM_CMPDAT3_CMP_Pos)                /*!< EPWM_T::CMPDAT3: CMP Mask              */

#define EPWM_CMPDAT3_CMPU_Pos            (16)                                              /*!< EPWM_T::CMPDAT3: CMPU Position         */
#define EPWM_CMPDAT3_CMPU_Msk            (0xfffful << EPWM_CMPDAT3_CMPU_Pos)               /*!< EPWM_T::CMPDAT3: CMPU Mask             */

#define EPWM_CMPDAT4_CMP_Pos             (0)                                               /*!< EPWM_T::CMPDAT4: CMP Position          */
#define EPWM_CMPDAT4_CMP_Msk             (0xfffful << EPWM_CMPDAT4_CMP_Pos)                /*!< EPWM_T::CMPDAT4: CMP Mask              */

#define EPWM_CMPDAT4_CMPU_Pos            (16)                                              /*!< EPWM_T::CMPDAT4: CMPU Position         */
#define EPWM_CMPDAT4_CMPU_Msk            (0xfffful << EPWM_CMPDAT4_CMPU_Pos)               /*!< EPWM_T::CMPDAT4: CMPU Mask             */

#define EPWM_CMPDAT5_CMP_Pos             (0)                                               /*!< EPWM_T::CMPDAT5: CMP Position          */
#define EPWM_CMPDAT5_CMP_Msk             (0xfffful << EPWM_CMPDAT5_CMP_Pos)                /*!< EPWM_T::CMPDAT5: CMP Mask              */

#define EPWM_CMPDAT5_CMPU_Pos            (16)                                              /*!< EPWM_T::CMPDAT5: CMPU Position         */
#define EPWM_CMPDAT5_CMPU_Msk            (0xfffful << EPWM_CMPDAT5_CMPU_Pos)               /*!< EPWM_T::CMPDAT5: CMPU Mask             */

#define EPWM_CNT_CNT_Pos                 (0)                                               /*!< EPWM_T::CNT: CNT Position              */
#define EPWM_CNT_CNT_Msk                 (0xfffful << EPWM_CNT_CNT_Pos)                    /*!< EPWM_T::CNT: CNT Mask                  */

#define EPWM_CNT_CNTDIR_Pos              (31)                                              /*!< EPWM_T::CNT: CNTDIR Position           */
#define EPWM_CNT_CNTDIR_Msk              (0x1ul << EPWM_CNT_CNTDIR_Pos)                    /*!< EPWM_T::CNT: CNTDIR Mask               */

#define EPWM_INTEN_PIEN_Pos              (0)                                               /*!< EPWM_T::INTEN: PIEN Position           */
#define EPWM_INTEN_PIEN_Msk              (0x1ul << EPWM_INTEN_PIEN_Pos)                    /*!< EPWM_T::INTEN: PIEN Mask               */

#define EPWM_INTEN_CMPUIEN0_Pos          (8)                                               /*!< EPWM_T::INTEN: CMPUIEN0 Position       */
#define EPWM_INTEN_CMPUIEN0_Msk          (0x1ul << EPWM_INTEN_CMPUIEN0_Pos)                /*!< EPWM_T::INTEN: CMPUIEN0 Mask           */

#define EPWM_INTEN_CMPUIEN1_Pos          (9)                                               /*!< EPWM_T::INTEN: CMPUIEN1 Position       */
#define EPWM_INTEN_CMPUIEN1_Msk          (0x1ul << EPWM_INTEN_CMPUIEN1_Pos)                /*!< EPWM_T::INTEN: CMPUIEN1 Mask           */

#define EPWM_INTEN_CMPUIEN2_Pos          (10)                                              /*!< EPWM_T::INTEN: CMPUIEN2 Position       */
#define EPWM_INTEN_CMPUIEN2_Msk          (0x1ul << EPWM_INTEN_CMPUIEN2_Pos)                /*!< EPWM_T::INTEN: CMPUIEN2 Mask           */

#define EPWM_INTEN_CMPUIEN3_Pos          (11)                                              /*!< EPWM_T::INTEN: CMPUIEN3 Position       */
#define EPWM_INTEN_CMPUIEN3_Msk          (0x1ul << EPWM_INTEN_CMPUIEN3_Pos)                /*!< EPWM_T::INTEN: CMPUIEN3 Mask           */

#define EPWM_INTEN_CMPUIEN4_Pos          (12)                                              /*!< EPWM_T::INTEN: CMPUIEN4 Position       */
#define EPWM_INTEN_CMPUIEN4_Msk          (0x1ul << EPWM_INTEN_CMPUIEN4_Pos)                /*!< EPWM_T::INTEN: CMPUIEN4 Mask           */

#define EPWM_INTEN_CMPUIEN5_Pos          (13)                                              /*!< EPWM_T::INTEN: CMPUIEN5 Position       */
#define EPWM_INTEN_CMPUIEN5_Msk          (0x1ul << EPWM_INTEN_CMPUIEN5_Pos)                /*!< EPWM_T::INTEN: CMPUIEN5 Mask           */

#define EPWM_INTEN_BRK0IEN_Pos           (16)                                              /*!< EPWM_T::INTEN: BRK0IEN Position        */
#define EPWM_INTEN_BRK0IEN_Msk           (0x1ul << EPWM_INTEN_BRK0IEN_Pos)                 /*!< EPWM_T::INTEN: BRK0IEN Mask            */

#define EPWM_INTEN_BRK1IEN_Pos           (17)                                              /*!< EPWM_T::INTEN: BRK1IEN Position        */
#define EPWM_INTEN_BRK1IEN_Msk           (0x1ul << EPWM_INTEN_BRK1IEN_Pos)                 /*!< EPWM_T::INTEN: BRK1IEN Mask            */

#define EPWM_INTEN_CIEN_Pos              (18)                                              /*!< EPWM_T::INTEN: CIEN Position           */
#define EPWM_INTEN_CIEN_Msk              (0x1ul << EPWM_INTEN_CIEN_Pos)                    /*!< EPWM_T::INTEN: CIEN Mask               */

#define EPWM_INTEN_CMPDIEN0_Pos          (24)                                              /*!< EPWM_T::INTEN: CMPDIEN0 Position       */
#define EPWM_INTEN_CMPDIEN0_Msk          (0x1ul << EPWM_INTEN_CMPDIEN0_Pos)                /*!< EPWM_T::INTEN: CMPDIEN0 Mask           */

#define EPWM_INTEN_CMPDIEN1_Pos          (25)                                              /*!< EPWM_T::INTEN: CMPDIEN1 Position       */
#define EPWM_INTEN_CMPDIEN1_Msk          (0x1ul << EPWM_INTEN_CMPDIEN1_Pos)                /*!< EPWM_T::INTEN: CMPDIEN1 Mask           */

#define EPWM_INTEN_CMPDIEN2_Pos          (26)                                              /*!< EPWM_T::INTEN: CMPDIEN2 Position       */
#define EPWM_INTEN_CMPDIEN2_Msk          (0x1ul << EPWM_INTEN_CMPDIEN2_Pos)                /*!< EPWM_T::INTEN: CMPDIEN2 Mask           */

#define EPWM_INTEN_CMPDIEN3_Pos          (27)                                              /*!< EPWM_T::INTEN: CMPDIEN3 Position       */
#define EPWM_INTEN_CMPDIEN3_Msk          (0x1ul << EPWM_INTEN_CMPDIEN3_Pos)                /*!< EPWM_T::INTEN: CMPDIEN3 Mask           */

#define EPWM_INTEN_CMPDIEN4_Pos          (28)                                              /*!< EPWM_T::INTEN: CMPDIEN4 Position       */
#define EPWM_INTEN_CMPDIEN4_Msk          (0x1ul << EPWM_INTEN_CMPDIEN4_Pos)                /*!< EPWM_T::INTEN: CMPDIEN4 Mask           */

#define EPWM_INTEN_CMPDIEN5_Pos          (29)                                              /*!< EPWM_T::INTEN: CMPDIEN5 Position       */
#define EPWM_INTEN_CMPDIEN5_Msk          (0x1ul << EPWM_INTEN_CMPDIEN5_Pos)                /*!< EPWM_T::INTEN: CMPDIEN5 Mask           */

#define EPWM_INTSTS_PIF_Pos              (0)                                               /*!< EPWM_T::INTSTS: PIF Position           */
#define EPWM_INTSTS_PIF_Msk              (0x1ul << EPWM_INTSTS_PIF_Pos)                    /*!< EPWM_T::INTSTS: PIF Mask               */

#define EPWM_INTSTS_CMPUIF0_Pos          (8)                                               /*!< EPWM_T::INTSTS: CMPUIF0 Position       */
#define EPWM_INTSTS_CMPUIF0_Msk          (0x1ul << EPWM_INTSTS_CMPUIF0_Pos)                /*!< EPWM_T::INTSTS: CMPUIF0 Mask           */

#define EPWM_INTSTS_CMPUIF1_Pos          (9)                                               /*!< EPWM_T::INTSTS: CMPUIF1 Position       */
#define EPWM_INTSTS_CMPUIF1_Msk          (0x1ul << EPWM_INTSTS_CMPUIF1_Pos)                /*!< EPWM_T::INTSTS: CMPUIF1 Mask           */

#define EPWM_INTSTS_CMPUIF2_Pos          (10)                                              /*!< EPWM_T::INTSTS: CMPUIF2 Position       */
#define EPWM_INTSTS_CMPUIF2_Msk          (0x1ul << EPWM_INTSTS_CMPUIF2_Pos)                /*!< EPWM_T::INTSTS: CMPUIF2 Mask           */

#define EPWM_INTSTS_CMPUIF3_Pos          (11)                                              /*!< EPWM_T::INTSTS: CMPUIF3 Position       */
#define EPWM_INTSTS_CMPUIF3_Msk          (0x1ul << EPWM_INTSTS_CMPUIF3_Pos)                /*!< EPWM_T::INTSTS: CMPUIF3 Mask           */

#define EPWM_INTSTS_CMPUIF4_Pos          (12)                                              /*!< EPWM_T::INTSTS: CMPUIF4 Position       */
#define EPWM_INTSTS_CMPUIF4_Msk          (0x1ul << EPWM_INTSTS_CMPUIF4_Pos)                /*!< EPWM_T::INTSTS: CMPUIF4 Mask           */

#define EPWM_INTSTS_CMPUIF5_Pos          (13)                                              /*!< EPWM_T::INTSTS: CMPUIF5 Position       */
#define EPWM_INTSTS_CMPUIF5_Msk          (0x1ul << EPWM_INTSTS_CMPUIF5_Pos)                /*!< EPWM_T::INTSTS: CMPUIF5 Mask           */

#define EPWM_INTSTS_BRK0IF_Pos           (16)                                              /*!< EPWM_T::INTSTS: BRK0IF Position        */
#define EPWM_INTSTS_BRK0IF_Msk           (0x1ul << EPWM_INTSTS_BRK0IF_Pos)                 /*!< EPWM_T::INTSTS: BRK0IF Mask            */

#define EPWM_INTSTS_BRK1IF_Pos           (17)                                              /*!< EPWM_T::INTSTS: BRK1IF Position        */
#define EPWM_INTSTS_BRK1IF_Msk           (0x1ul << EPWM_INTSTS_BRK1IF_Pos)                 /*!< EPWM_T::INTSTS: BRK1IF Mask            */

#define EPWM_INTSTS_CIF_Pos              (18)                                              /*!< EPWM_T::INTSTS: CIF Position           */
#define EPWM_INTSTS_CIF_Msk              (0x1ul << EPWM_INTSTS_CIF_Pos)                    /*!< EPWM_T::INTSTS: CIF Mask               */

#define EPWM_INTSTS_CMPDIF0_Pos          (24)                                              /*!< EPWM_T::INTSTS: CMPDIF0 Position       */
#define EPWM_INTSTS_CMPDIF0_Msk          (0x1ul << EPWM_INTSTS_CMPDIF0_Pos)                /*!< EPWM_T::INTSTS: CMPDIF0 Mask           */

#define EPWM_INTSTS_CMPDIF1_Pos          (25)                                              /*!< EPWM_T::INTSTS: CMPDIF1 Position       */
#define EPWM_INTSTS_CMPDIF1_Msk          (0x1ul << EPWM_INTSTS_CMPDIF1_Pos)                /*!< EPWM_T::INTSTS: CMPDIF1 Mask           */

#define EPWM_INTSTS_CMPDIF2_Pos          (26)                                              /*!< EPWM_T::INTSTS: CMPDIF2 Position       */
#define EPWM_INTSTS_CMPDIF2_Msk          (0x1ul << EPWM_INTSTS_CMPDIF2_Pos)                /*!< EPWM_T::INTSTS: CMPDIF2 Mask           */

#define EPWM_INTSTS_CMPDIF3_Pos          (27)                                              /*!< EPWM_T::INTSTS: CMPDIF3 Position       */
#define EPWM_INTSTS_CMPDIF3_Msk          (0x1ul << EPWM_INTSTS_CMPDIF3_Pos)                /*!< EPWM_T::INTSTS: CMPDIF3 Mask           */

#define EPWM_INTSTS_CMPDIF4_Pos          (28)                                              /*!< EPWM_T::INTSTS: CMPDIF4 Position       */
#define EPWM_INTSTS_CMPDIF4_Msk          (0x1ul << EPWM_INTSTS_CMPDIF4_Pos)                /*!< EPWM_T::INTSTS: CMPDIF4 Mask           */

#define EPWM_INTSTS_CMPDIF5_Pos          (29)                                              /*!< EPWM_T::INTSTS: CMPDIF5 Position       */
#define EPWM_INTSTS_CMPDIF5_Msk          (0x1ul << EPWM_INTSTS_CMPDIF5_Pos)                /*!< EPWM_T::INTSTS: CMPDIF5 Mask           */

#define EPWM_RESDLY_DELAY_Pos            (0)                                               /*!< EPWM_T::RESDLY: DELAY Position         */
#define EPWM_RESDLY_DELAY_Msk            (0xffful << EPWM_RESDLY_DELAY_Pos)                /*!< EPWM_T::RESDLY: DELAY Mask             */

#define EPWM_BRKCTL_BRK0EN_Pos           (0)                                               /*!< EPWM_T::BRKCTL: BRK0EN Position        */
#define EPWM_BRKCTL_BRK0EN_Msk           (0x1ul << EPWM_BRKCTL_BRK0EN_Pos)                 /*!< EPWM_T::BRKCTL: BRK0EN Mask            */

#define EPWM_BRKCTL_BRK1EN_Pos           (1)                                               /*!< EPWM_T::BRKCTL: BRK1EN Position        */
#define EPWM_BRKCTL_BRK1EN_Msk           (0x1ul << EPWM_BRKCTL_BRK1EN_Pos)                 /*!< EPWM_T::BRKCTL: BRK1EN Mask            */

#define EPWM_BRKCTL_BRK0A0EN_Pos         (2)                                               /*!< EPWM_T::BRKCTL: BRK0A0EN Position      */
#define EPWM_BRKCTL_BRK0A0EN_Msk         (0x1ul << EPWM_BRKCTL_BRK0A0EN_Pos)               /*!< EPWM_T::BRKCTL: BRK0A0EN Mask          */

#define EPWM_BRKCTL_BRK0A1EN_Pos         (3)                                               /*!< EPWM_T::BRKCTL: BRK0A1EN Position      */
#define EPWM_BRKCTL_BRK0A1EN_Msk         (0x1ul << EPWM_BRKCTL_BRK0A1EN_Pos)               /*!< EPWM_T::BRKCTL: BRK0A1EN Mask          */

#define EPWM_BRKCTL_BK0ADCEN_Pos         (4)                                               /*!< EPWM_T::BRKCTL: BK0ADCEN Position      */
#define EPWM_BRKCTL_BK0ADCEN_Msk         (0x1ul << EPWM_BRKCTL_BK0ADCEN_Pos)               /*!< EPWM_T::BRKCTL: BK0ADCEN Mask          */

#define EPWM_BRKCTL_BRK0PEN_Pos          (5)                                               /*!< EPWM_T::BRKCTL: BRK0PEN Position       */
#define EPWM_BRKCTL_BRK0PEN_Msk          (0x1ul << EPWM_BRKCTL_BRK0PEN_Pos)                /*!< EPWM_T::BRKCTL: BRK0PEN Mask           */

#define EPWM_BRKCTL_SWBRK_Pos            (9)                                               /*!< EPWM_T::BRKCTL: SWBRK Position         */
#define EPWM_BRKCTL_SWBRK_Msk            (0x1ul << EPWM_BRKCTL_SWBRK_Pos)                  /*!< EPWM_T::BRKCTL: SWBRK Mask             */

#define EPWM_BRKCTL_BRK1A0EN_Pos         (10)                                              /*!< EPWM_T::BRKCTL: BRK1A0EN Position      */
#define EPWM_BRKCTL_BRK1A0EN_Msk         (0x1ul << EPWM_BRKCTL_BRK1A0EN_Pos)               /*!< EPWM_T::BRKCTL: BRK1A0EN Mask          */

#define EPWM_BRKCTL_BRK1A1EN_Pos         (11)                                              /*!< EPWM_T::BRKCTL: BRK1A1EN Position      */
#define EPWM_BRKCTL_BRK1A1EN_Msk         (0x1ul << EPWM_BRKCTL_BRK1A1EN_Pos)               /*!< EPWM_T::BRKCTL: BRK1A1EN Mask          */

#define EPWM_BRKCTL_BK1ADCEN_Pos         (12)                                              /*!< EPWM_T::BRKCTL: BK1ADCEN Position      */
#define EPWM_BRKCTL_BK1ADCEN_Msk         (0x1ul << EPWM_BRKCTL_BK1ADCEN_Pos)               /*!< EPWM_T::BRKCTL: BK1ADCEN Mask          */

#define EPWM_BRKCTL_BRK1PEN_Pos          (13)                                              /*!< EPWM_T::BRKCTL: BRK1PEN Position       */
#define EPWM_BRKCTL_BRK1PEN_Msk          (0x1ul << EPWM_BRKCTL_BRK1PEN_Pos)                /*!< EPWM_T::BRKCTL: BRK1PEN Mask           */

#define EPWM_BRKCTL_LVDBKEN_Pos          (14)                                              /*!< EPWM_T::BRKCTL: LVDBKEN Position       */
#define EPWM_BRKCTL_LVDBKEN_Msk          (0x1ul << EPWM_BRKCTL_LVDBKEN_Pos)                /*!< EPWM_T::BRKCTL: LVDBKEN Mask           */

#define EPWM_BRKCTL_LVDTYPE_Pos          (15)                                              /*!< EPWM_T::BRKCTL: LVDTYPE Position       */
#define EPWM_BRKCTL_LVDTYPE_Msk          (0x1ul << EPWM_BRKCTL_LVDTYPE_Pos)                /*!< EPWM_T::BRKCTL: LVDTYPE Mask           */

#define EPWM_BRKCTL_BKOD0_Pos            (24)                                              /*!< EPWM_T::BRKCTL: BKOD0 Position         */
#define EPWM_BRKCTL_BKOD0_Msk            (0x1ul << EPWM_BRKCTL_BKOD0_Pos)                  /*!< EPWM_T::BRKCTL: BKOD0 Mask             */

#define EPWM_BRKCTL_BKOD1_Pos            (25)                                              /*!< EPWM_T::BRKCTL: BKOD1 Position         */
#define EPWM_BRKCTL_BKOD1_Msk            (0x1ul << EPWM_BRKCTL_BKOD1_Pos)                  /*!< EPWM_T::BRKCTL: BKOD1 Mask             */

#define EPWM_BRKCTL_BKOD2_Pos            (26)                                              /*!< EPWM_T::BRKCTL: BKOD2 Position         */
#define EPWM_BRKCTL_BKOD2_Msk            (0x1ul << EPWM_BRKCTL_BKOD2_Pos)                  /*!< EPWM_T::BRKCTL: BKOD2 Mask             */

#define EPWM_BRKCTL_BKOD3_Pos            (27)                                              /*!< EPWM_T::BRKCTL: BKOD3 Position         */
#define EPWM_BRKCTL_BKOD3_Msk            (0x1ul << EPWM_BRKCTL_BKOD3_Pos)                  /*!< EPWM_T::BRKCTL: BKOD3 Mask             */

#define EPWM_BRKCTL_BKOD4_Pos            (28)                                              /*!< EPWM_T::BRKCTL: BKOD4 Position         */
#define EPWM_BRKCTL_BKOD4_Msk            (0x1ul << EPWM_BRKCTL_BKOD4_Pos)                  /*!< EPWM_T::BRKCTL: BKOD4 Mask             */

#define EPWM_BRKCTL_BKOD5_Pos            (29)                                              /*!< EPWM_T::BRKCTL: BKOD5 Position         */
#define EPWM_BRKCTL_BKOD5_Msk            (0x1ul << EPWM_BRKCTL_BKOD5_Pos)                  /*!< EPWM_T::BRKCTL: BKOD5 Mask             */

#define EPWM_BRKCTL_NFPEN_Pos            (31)                                              /*!< EPWM_T::BRKCTL: NFPEN Position         */
#define EPWM_BRKCTL_NFPEN_Msk            (0x1ul << EPWM_BRKCTL_NFPEN_Pos)                  /*!< EPWM_T::BRKCTL: NFPEN Mask             */

#define EPWM_DTCTL_DTCNT01_Pos           (0)                                               /*!< EPWM_T::DTCTL: DTCNT01 Position        */
#define EPWM_DTCTL_DTCNT01_Msk           (0xfful << EPWM_DTCTL_DTCNT01_Pos)                /*!< EPWM_T::DTCTL: DTCNT01 Mask            */

#define EPWM_DTCTL_DTCNT23_Pos           (8)                                               /*!< EPWM_T::DTCTL: DTCNT23 Position        */
#define EPWM_DTCTL_DTCNT23_Msk           (0xfful << EPWM_DTCTL_DTCNT23_Pos)                /*!< EPWM_T::DTCTL: DTCNT23 Mask            */

#define EPWM_DTCTL_DTCNT45_Pos           (16)                                              /*!< EPWM_T::DTCTL: DTCNT45 Position        */
#define EPWM_DTCTL_DTCNT45_Msk           (0xfful << EPWM_DTCTL_DTCNT45_Pos)                /*!< EPWM_T::DTCTL: DTCNT45 Mask            */

#define EPWM_PHCHG_MSKDAT0_Pos           (0)                                               /*!< EPWM_T::PHCHG: MSKDAT0 Position        */
#define EPWM_PHCHG_MSKDAT0_Msk           (0x1ul << EPWM_PHCHG_MSKDAT0_Pos)                 /*!< EPWM_T::PHCHG: MSKDAT0 Mask            */

#define EPWM_PHCHG_MSKDAT1_Pos           (1)                                               /*!< EPWM_T::PHCHG: MSKDAT1 Position        */
#define EPWM_PHCHG_MSKDAT1_Msk           (0x1ul << EPWM_PHCHG_MSKDAT1_Pos)                 /*!< EPWM_T::PHCHG: MSKDAT1 Mask            */

#define EPWM_PHCHG_MSKDAT2_Pos           (2)                                               /*!< EPWM_T::PHCHG: MSKDAT2 Position        */
#define EPWM_PHCHG_MSKDAT2_Msk           (0x1ul << EPWM_PHCHG_MSKDAT2_Pos)                 /*!< EPWM_T::PHCHG: MSKDAT2 Mask            */

#define EPWM_PHCHG_MSKDAT3_Pos           (3)                                               /*!< EPWM_T::PHCHG: MSKDAT3 Position        */
#define EPWM_PHCHG_MSKDAT3_Msk           (0x1ul << EPWM_PHCHG_MSKDAT3_Pos)                 /*!< EPWM_T::PHCHG: MSKDAT3 Mask            */

#define EPWM_PHCHG_MSKDAT4_Pos           (4)                                               /*!< EPWM_T::PHCHG: MSKDAT4 Position        */
#define EPWM_PHCHG_MSKDAT4_Msk           (0x1ul << EPWM_PHCHG_MSKDAT4_Pos)                 /*!< EPWM_T::PHCHG: MSKDAT4 Mask            */

#define EPWM_PHCHG_MSKDAT5_Pos           (5)                                               /*!< EPWM_T::PHCHG: MSKDAT5 Position        */
#define EPWM_PHCHG_MSKDAT5_Msk           (0x1ul << EPWM_PHCHG_MSKDAT5_Pos)                 /*!< EPWM_T::PHCHG: MSKDAT5 Mask            */

#define EPWM_PHCHG_MSKEN0_Pos            (8)                                               /*!< EPWM_T::PHCHG: MSKEN0 Position         */
#define EPWM_PHCHG_MSKEN0_Msk            (0x1ul << EPWM_PHCHG_MSKEN0_Pos)                  /*!< EPWM_T::PHCHG: MSKEN0 Mask             */

#define EPWM_PHCHG_MSKEN1_Pos            (9)                                               /*!< EPWM_T::PHCHG: MSKEN1 Position         */
#define EPWM_PHCHG_MSKEN1_Msk            (0x1ul << EPWM_PHCHG_MSKEN1_Pos)                  /*!< EPWM_T::PHCHG: MSKEN1 Mask             */

#define EPWM_PHCHG_MSKEN2_Pos            (10)                                              /*!< EPWM_T::PHCHG: MSKEN2 Position         */
#define EPWM_PHCHG_MSKEN2_Msk            (0x1ul << EPWM_PHCHG_MSKEN2_Pos)                  /*!< EPWM_T::PHCHG: MSKEN2 Mask             */

#define EPWM_PHCHG_MSKEN3_Pos            (11)                                              /*!< EPWM_T::PHCHG: MSKEN3 Position         */
#define EPWM_PHCHG_MSKEN3_Msk            (0x1ul << EPWM_PHCHG_MSKEN3_Pos)                  /*!< EPWM_T::PHCHG: MSKEN3 Mask             */

#define EPWM_PHCHG_MSKEN4_Pos            (12)                                              /*!< EPWM_T::PHCHG: MSKEN4 Position         */
#define EPWM_PHCHG_MSKEN4_Msk            (0x1ul << EPWM_PHCHG_MSKEN4_Pos)                  /*!< EPWM_T::PHCHG: MSKEN4 Mask             */

#define EPWM_PHCHG_MSKEN5_Pos            (13)                                              /*!< EPWM_T::PHCHG: MSKEN5 Position         */
#define EPWM_PHCHG_MSKEN5_Msk            (0x1ul << EPWM_PHCHG_MSKEN5_Pos)                  /*!< EPWM_T::PHCHG: MSKEN5 Mask             */

#define EPWM_PHCHG_TRGSEL_Pos            (20)                                              /*!< EPWM_T::PHCHG: TRGSEL Position         */
#define EPWM_PHCHG_TRGSEL_Msk            (0x7ul << EPWM_PHCHG_TRGSEL_Pos)                  /*!< EPWM_T::PHCHG: TRGSEL Mask             */

#define EPWM_PHCHG_A0POSSEL_Pos          (24)                                              /*!< EPWM_T::PHCHG: A0POSSEL Position       */
#define EPWM_PHCHG_A0POSSEL_Msk          (0x3ul << EPWM_PHCHG_A0POSSEL_Pos)                /*!< EPWM_T::PHCHG: A0POSSEL Mask           */

#define EPWM_PHCHG_A1POSSEL_Pos          (26)                                              /*!< EPWM_T::PHCHG: A1POSSEL Position       */
#define EPWM_PHCHG_A1POSSEL_Msk          (0x3ul << EPWM_PHCHG_A1POSSEL_Pos)                /*!< EPWM_T::PHCHG: A1POSSEL Mask           */

#define EPWM_PHCHG_ACMP0TEN_Pos          (28)                                              /*!< EPWM_T::PHCHG: ACMP0TEN Position       */
#define EPWM_PHCHG_ACMP0TEN_Msk          (0x1ul << EPWM_PHCHG_ACMP0TEN_Pos)                /*!< EPWM_T::PHCHG: ACMP0TEN Mask           */

#define EPWM_PHCHG_ACMP1TEN_Pos          (29)                                              /*!< EPWM_T::PHCHG: ACMP1TEN Position       */
#define EPWM_PHCHG_ACMP1TEN_Msk          (0x1ul << EPWM_PHCHG_ACMP1TEN_Pos)                /*!< EPWM_T::PHCHG: ACMP1TEN Mask           */

#define EPWM_PHCHGNXT_MSKDAT0_Pos        (0)                                               /*!< EPWM_T::PHCHGNXT: MSKDAT0 Position     */
#define EPWM_PHCHGNXT_MSKDAT0_Msk        (0x1ul << EPWM_PHCHGNXT_MSKDAT0_Pos)              /*!< EPWM_T::PHCHGNXT: MSKDAT0 Mask         */

#define EPWM_PHCHGNXT_MSKDAT1_Pos        (1)                                               /*!< EPWM_T::PHCHGNXT: MSKDAT1 Position     */
#define EPWM_PHCHGNXT_MSKDAT1_Msk        (0x1ul << EPWM_PHCHGNXT_MSKDAT1_Pos)              /*!< EPWM_T::PHCHGNXT: MSKDAT1 Mask         */

#define EPWM_PHCHGNXT_MSKDAT2_Pos        (2)                                               /*!< EPWM_T::PHCHGNXT: MSKDAT2 Position     */
#define EPWM_PHCHGNXT_MSKDAT2_Msk        (0x1ul << EPWM_PHCHGNXT_MSKDAT2_Pos)              /*!< EPWM_T::PHCHGNXT: MSKDAT2 Mask         */

#define EPWM_PHCHGNXT_MSKDAT3_Pos        (3)                                               /*!< EPWM_T::PHCHGNXT: MSKDAT3 Position     */
#define EPWM_PHCHGNXT_MSKDAT3_Msk        (0x1ul << EPWM_PHCHGNXT_MSKDAT3_Pos)              /*!< EPWM_T::PHCHGNXT: MSKDAT3 Mask         */

#define EPWM_PHCHGNXT_MSKDAT4_Pos        (4)                                               /*!< EPWM_T::PHCHGNXT: MSKDAT4 Position     */
#define EPWM_PHCHGNXT_MSKDAT4_Msk        (0x1ul << EPWM_PHCHGNXT_MSKDAT4_Pos)              /*!< EPWM_T::PHCHGNXT: MSKDAT4 Mask         */

#define EPWM_PHCHGNXT_MSKDAT5_Pos        (5)                                               /*!< EPWM_T::PHCHGNXT: MSKDAT5 Position     */
#define EPWM_PHCHGNXT_MSKDAT5_Msk        (0x1ul << EPWM_PHCHGNXT_MSKDAT5_Pos)              /*!< EPWM_T::PHCHGNXT: MSKDAT5 Mask         */

#define EPWM_PHCHGNXT_MSKEN0_Pos         (8)                                               /*!< EPWM_T::PHCHGNXT: MSKEN0 Position      */
#define EPWM_PHCHGNXT_MSKEN0_Msk         (0x1ul << EPWM_PHCHGNXT_MSKEN0_Pos)               /*!< EPWM_T::PHCHGNXT: MSKEN0 Mask          */

#define EPWM_PHCHGNXT_MSKEN1_Pos         (9)                                               /*!< EPWM_T::PHCHGNXT: MSKEN1 Position      */
#define EPWM_PHCHGNXT_MSKEN1_Msk         (0x1ul << EPWM_PHCHGNXT_MSKEN1_Pos)               /*!< EPWM_T::PHCHGNXT: MSKEN1 Mask          */

#define EPWM_PHCHGNXT_MSKEN2_Pos         (10)                                              /*!< EPWM_T::PHCHGNXT: MSKEN2 Position      */
#define EPWM_PHCHGNXT_MSKEN2_Msk         (0x1ul << EPWM_PHCHGNXT_MSKEN2_Pos)               /*!< EPWM_T::PHCHGNXT: MSKEN2 Mask          */

#define EPWM_PHCHGNXT_MSKEN3_Pos         (11)                                              /*!< EPWM_T::PHCHGNXT: MSKEN3 Position      */
#define EPWM_PHCHGNXT_MSKEN3_Msk         (0x1ul << EPWM_PHCHGNXT_MSKEN3_Pos)               /*!< EPWM_T::PHCHGNXT: MSKEN3 Mask          */

#define EPWM_PHCHGNXT_MSKEN4_Pos         (12)                                              /*!< EPWM_T::PHCHGNXT: MSKEN4 Position      */
#define EPWM_PHCHGNXT_MSKEN4_Msk         (0x1ul << EPWM_PHCHGNXT_MSKEN4_Pos)               /*!< EPWM_T::PHCHGNXT: MSKEN4 Mask          */

#define EPWM_PHCHGNXT_MSKEN5_Pos         (13)                                              /*!< EPWM_T::PHCHGNXT: MSKEN5 Position      */
#define EPWM_PHCHGNXT_MSKEN5_Msk         (0x1ul << EPWM_PHCHGNXT_MSKEN5_Pos)               /*!< EPWM_T::PHCHGNXT: MSKEN5 Mask          */

#define EPWM_PHCHGNXT_HALLSTS_Pos        (16)                                              /*!< EPWM_T::PHCHGNXT: HALLSTS Position     */
#define EPWM_PHCHGNXT_HALLSTS_Msk        (0x7ul << EPWM_PHCHGNXT_HALLSTS_Pos)              /*!< EPWM_T::PHCHGNXT: HALLSTS Mask         */

#define EPWM_PHCHGNXT_TRGSEL_Pos         (20)                                              /*!< EPWM_T::PHCHGNXT: TRGSEL Position      */
#define EPWM_PHCHGNXT_TRGSEL_Msk         (0x7ul << EPWM_PHCHGNXT_TRGSEL_Pos)               /*!< EPWM_T::PHCHGNXT: TRGSEL Mask          */

#define EPWM_PHCHGNXT_A0POSSEL_Pos       (24)                                              /*!< EPWM_T::PHCHGNXT: A0POSSEL Position    */
#define EPWM_PHCHGNXT_A0POSSEL_Msk       (0x3ul << EPWM_PHCHGNXT_A0POSSEL_Pos)             /*!< EPWM_T::PHCHGNXT: A0POSSEL Mask        */

#define EPWM_PHCHGNXT_A1POSSEL_Pos       (26)                                              /*!< EPWM_T::PHCHGNXT: A1POSSEL Position    */
#define EPWM_PHCHGNXT_A1POSSEL_Msk       (0x3ul << EPWM_PHCHGNXT_A1POSSEL_Pos)             /*!< EPWM_T::PHCHGNXT: A1POSSEL Mask        */

#define EPWM_PHCHGNXT_ACMP0TEN_Pos       (28)                                              /*!< EPWM_T::PHCHGNXT: ACMP0TEN Position    */
#define EPWM_PHCHGNXT_ACMP0TEN_Msk       (0x1ul << EPWM_PHCHGNXT_ACMP0TEN_Pos)             /*!< EPWM_T::PHCHGNXT: ACMP0TEN Mask        */

#define EPWM_PHCHGNXT_ACMP1TEN_Pos       (29)                                              /*!< EPWM_T::PHCHGNXT: ACMP1TEN Position    */
#define EPWM_PHCHGNXT_ACMP1TEN_Msk       (0x1ul << EPWM_PHCHGNXT_ACMP1TEN_Pos)             /*!< EPWM_T::PHCHGNXT: ACMP1TEN Mask        */

#define EPWM_PHCHGALT_POSCTL0_Pos        (0)                                               /*!< EPWM_T::PHCHGALT: POSCTL0 Position     */
#define EPWM_PHCHGALT_POSCTL0_Msk        (0x1ul << EPWM_PHCHGALT_POSCTL0_Pos)              /*!< EPWM_T::PHCHGALT: POSCTL0 Mask         */

#define EPWM_PHCHGALT_POSCTL1_Pos        (1)                                               /*!< EPWM_T::PHCHGALT: POSCTL1 Position     */
#define EPWM_PHCHGALT_POSCTL1_Msk        (0x1ul << EPWM_PHCHGALT_POSCTL1_Pos)              /*!< EPWM_T::PHCHGALT: POSCTL1 Mask         */

#define EPWM_IFA_IFAEN_Pos               (0)                                               /*!< EPWM_T::IFA: IFAEN Position            */
#define EPWM_IFA_IFAEN_Msk               (0x1ul << EPWM_IFA_IFAEN_Pos)                     /*!< EPWM_T::IFA: IFAEN Mask                */

#define EPWM_IFA_IFCNT_Pos               (4)                                               /*!< EPWM_T::IFA: IFCNT Position            */
#define EPWM_IFA_IFCNT_Msk               (0xful << EPWM_IFA_IFCNT_Pos)                     /*!< EPWM_T::IFA: IFCNT Mask                */

#define EPWM_IFA_IFDAT_Pos               (12)                                              /*!< EPWM_T::IFA: IFDAT Position            */
#define EPWM_IFA_IFDAT_Msk               (0xful << EPWM_IFA_IFDAT_Pos)                     /*!< EPWM_T::IFA: IFDAT Mask                */

/**@}*/ /* EPWM_CONST */
/**@}*/ /* end of EPWM register group */


/*---------------------- Basic Pulse Width Modulation Controller -------------------------*/
/**
    @addtogroup BPWM Basic Pulse Width Modulation Controller(BPWM)
    Memory Mapped Structure for BPWM Controller
@{ */

typedef struct
{


    /**
     * @var BPWM_T::CLKPSC
     * Offset: 0x00  Basic PWM Pre-scalar Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CLKPSC01  |Clock Prescaler
     * |        |          |Clock input is divided by (CLKPSC01 + 1) before it is fed to the corresponding PWM-timer
     * |        |          |If CLKPSC01=0, then the clock prescaler 0 output clock will be stopped
     * |        |          |So corresponding PWM-timer will also be stopped.
     * |[23:16] |DTI01     |Dead-zone Interval for Pair of Channel 0 and Channel 1
     * |        |          |These 8-bit determine the Dead-zone length.
     * |        |          |The unit time of Dead-zone length = [(prescale+1)*(clock source divider)] / BPWM_CLK.
     * @var BPWM_T::CLKDIV
     * Offset: 0x04  Basic PWM Clock Source Divider Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CLKDIV0   |PWM Timer 0 Clock Source Divider Selection
     * |        |          |Select clock source divider for PWM timer 0.
     * |        |          |(Table is the same as CLKDIV1)
     * |[6:4]   |CLKDIV1   |PWM Timer 1 Clock Source Divider Selection
     * |        |          |Select clock source divider for PWM timer 1.
     * |        |          |000 = 1/2.
     * |        |          |001 = 1/4.
     * |        |          |010 = 1/8.
     * |        |          |011 = 1/16.
     * |        |          |100 = 1.
     * @var BPWM_T::CTL
     * Offset: 0x08  Basic PWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTEN0    |PWM-timer 0 Enable Bit
     * |        |          |0 = The corresponding PWM-Timer stops running.
     * |        |          |1 = The corresponding PWM-Timer starts running.
     * |[1]     |PINV0     |PWM-timer 0 Output Polar Inverse Enable Bit
     * |        |          |0 = PWM0 output polar inverse Disabled.
     * |        |          |1 = PWM0 output polar inverse Enabled.
     * |[2]     |CMPINV0   |PWM-timer 0 Output Inverter Enable Bit
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[3]     |CNTMODE0  |PWM-timer 0 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a transition at this bit, it will cause BPWM_PERIOD0 and BPWM_CMPDAT0 be cleared.
     * |[4]     |DTCNT01   |Dead-zone 0 Generator Enable Bit
     * |        |          |0 = Dead-zone 0 Generator Disabled.
     * |        |          |1 = Dead-zone 0 Generator Enabled.
     * |        |          |Note: When Dead-zone generator is enabled, the pair of PWM0 and PWM1 becomes a complementary pair.
     * |[8]     |CNTEN1    |PWM-timer 1 Enable Bit
     * |        |          |0 = Corresponding PWM-Timer Stopped.
     * |        |          |1 = Corresponding PWM-Timer Start Running.
     * |[9]     |PINV1     |PWM-timer 1 Output Polar Inverse Enable Bit
     * |        |          |1 = PWM1 output polar inverse Enabled.
     * |        |          |0 = PWM1 output polar inverse Disabled.
     * |[10]    |CMPINV1   |PWM-timer 1 Output Inverter Enable
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[11]    |CNTMODE1  |PWM-timer 1 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a transition at this bit, it will cause BPWM_PERIOD1 and BPWM_CMPDAT1 be cleared.
     * |[30]    |CNTTYPE01 |PWM01 Aligned Type Selection Bit
     * |        |          |0 = Edge-aligned type.
     * |        |          |1 = Center-aligned type.
     * @var BPWM_T::PERIOD0
     * Offset: 0x0C  Basic PWM Period Counter Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD    |Basic PWM Period Counter Register
     * |        |          |PERIOD data determines the PWM period.
     * |        |          |PWM frequency = BPWM_CLK/[(prescale+1)*(clock divider)*(PERIOD+1)].
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMP+1)/(PERIOD+1).
     * |        |          | CMP >= PERIOD: PWM output is always high.
     * |        |          | CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          | CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMP) + 1]/[2 x (PERIOD+1)].
     * |        |          | CMP > PERIOD: PWM output is always high.
     * |        |          | CMP <= PERIOD: PWM low width = 2 x (PERIOD-CMP) + 1 unit; PWM high width = (2 x CMP) + 1 unit.
     * |        |          | CMP = 0: PWM low width = 2 x PERIOD + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to PERIOD will take effect in next PWM cycle.
     * |        |          |Note: When PWM operating at Center-aligned type, PERIOD value should be set between 0x0000 to 0xFFFE
     * |        |          |If PERIOD equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When PERIOD value is set to 0, PWM output is always high.
     * @var BPWM_T::CMPDAT0
     * Offset: 0x10  Basic PWM Comparator Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP       |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |PWM frequency = BPWM_CLK/[(prescale+1)*(clock divider)*(PERIOD+1)].
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMP+1)/(PERIOD+1).
     * |        |          | CMP >= PERIOD: PWM output is always high.
     * |        |          | CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          | CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMP) + 1]/[2 x (PERIOD+1)].
     * |        |          | CMP > PERIOD: PWM output is always high.
     * |        |          | CMP <= PERIOD: PWM low width = 2 x (PERIOD-CMP) + 1 unit; PWM high width = (2 x CMP) + 1 unit.
     * |        |          | CMP = 0: PWM low width = 2 x PERIOD + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to PERIOD will take effect in next PWM cycle.
     * @var BPWM_T::CNT0
     * Offset: 0x14  Basic PWM Data Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNT       |PWM Data Register
     * |        |          |User can monitor CNT to know the current value in 16-bit counter.
     * @var BPWM_T::PERIOD1
     * Offset: 0x18  Basic PWM Period Counter Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD    |Basic PWM Period Counter Register
     * |        |          |PERIOD data determines the PWM period.
     * |        |          |PWM frequency = BPWM_CLK/[(prescale+1)*(clock divider)*(PERIOD+1)].
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMP+1)/(PERIOD+1).
     * |        |          | CMP >= PERIOD: PWM output is always high.
     * |        |          | CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          | CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMP) + 1]/[2 x (PERIOD+1)].
     * |        |          | CMP > PERIOD: PWM output is always high.
     * |        |          | CMP <= PERIOD: PWM low width = 2 x (PERIOD-CMP) + 1 unit; PWM high width = (2 x CMP) + 1 unit.
     * |        |          | CMP = 0: PWM low width = 2 x PERIOD + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to PERIOD will take effect in next PWM cycle.
     * |        |          |Note: When PWM operating at Center-aligned type, PERIOD value should be set between 0x0000 to 0xFFFE
     * |        |          |If PERIOD equal to 0xFFFF, the PWM will work unpredictable.
     * |        |          |Note: When PERIOD value is set to 0, PWM output is always high.
     * @var BPWM_T::CMPDAT1
     * Offset: 0x1C  Basic PWM Comparator Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP       |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |PWM frequency = BPWM_CLK/[(prescale+1)*(clock divider)*(PERIOD+1)].
     * |        |          |For Edge-aligned type:
     * |        |          | Duty ratio = (CMP+1)/(PERIOD+1).
     * |        |          | CMP >= PERIOD: PWM output is always high.
     * |        |          | CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          | CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
     * |        |          |For Center-aligned type:
     * |        |          | Duty ratio = [(2 x CMP) + 1]/[2 x (PERIOD+1)].
     * |        |          | CMP > PERIOD: PWM output is always high.
     * |        |          | CMP <= PERIOD: PWM low width = 2 x (PERIOD-CMP) + 1 unit; PWM high width = (2 x CMP) + 1 unit.
     * |        |          | CMP = 0: PWM low width = 2 x PERIOD + 1 unit; PWM high width = 1 unit.
     * |        |          |(Unit = one PWM clock cycle).
     * |        |          |Note: Any write to PERIOD will take effect in next PWM cycle.
     * @var BPWM_T::CNT1
     * Offset: 0x20  Basic PWM Data Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNT       |PWM Data Register
     * |        |          |User can monitor CNT to know the current value in 16-bit counter.
     * @var BPWM_T::INTEN
     * Offset: 0x40  Basic PWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIEN0     |BPWM Channel 0 Period Interrupt Enable Bit
     * |        |          |0 = BPWM Channel 0 Period Interrupt Disabled.
     * |        |          |1 = BPWM Channel 0 Period Interrupt Enabled.
     * |[1]     |PIEN1     |BPWM Channel 1 Period Interrupt Enable Bit
     * |        |          |0 = BPWM Channel 1 Period Interrupt Disabled.
     * |        |          |1 = BPWM Channel 1 Period Interrupt Enabled.
     * |[8]     |DIEN0     |BPWM Channel 0 Duty Interrupt Enable Bit
     * |        |          |0 = BPWM Channel 0 Duty Interrupt Disabled.
     * |        |          |1 = BPWM Channel 0 Duty Interrupt Enabled.
     * |[9]     |DIEN1     |BPWM Channel 1 Duty Interrupt Enable Bit
     * |        |          |0 = BPWM Channel 1 Duty Interrupt Disabled.
     * |        |          |1 = BPWM Channel 1 Duty Interrupt Enabled.
     * |[16]    |PINTTYPE  |BPWM Interrupt Period Type Selection Bit
     * |        |          |0 = PIFn will be set if BPWM counter underflow.
     * |        |          |1 = PIFn will be set if BPWM counter matches PERIODn register.
     * |        |          |Note: This bit is effective when BPWM in Center-aligned type only.
     * @var BPWM_T::INTSTS
     * Offset: 0x44  Basic PWM Interrupt Indication Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIF0      |BPWM Channel 0 Period Interrupt Status
     * |        |          |This bit is set by hardware when BPWM0 counter reaches the requirement of interrupt (depend on PINTTYPE bit of PWM_INTEN register), software can write 1 to clear this bit to 0.
     * |[1]     |PIF1      |BPWM Channel 1 Period Interrupt Status
     * |        |          |This bit is set by hardware when BPWM1 counter reaches the requirement of interrupt (depend on PINTTYPE bit of PWM_INTEN register), software can write 1 to clear this bit to 0.
     * |[8]     |DIF0      |BPWM Channel 0 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when channel 0 BPWM counter down count and reaches BPWM_CMPDAT 0, software can clear this bit by writing a one to it.
     * |        |          |Note: If CMP equal to PERIOD, this flag is not working in Edge-aligned type selection
     * |[9]     |DIF1      |BPWM Channel 1 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when channel 1 BPWM counter down count and reaches BPWM_CMPDAT 1, software can clear this bit by writing a one to it.
     * |        |          |Note: If CMP equal to PERIOD, this flag is not working in Edge-aligned type selection
     * @var BPWM_T::POEN
     * Offset: 0x7C  Basic PWM Output Enable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |POEN0     |Channel 0 Output Enable Register
     * |        |          |0 = BPWM channel 0 output to pin Disabled.
     * |        |          |1 = BPWM channel 0 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must also be switched to BPWM function
     * |[1]     |POEN1     |Channel 1 Output Enable Register
     * |        |          |0 = BPWM channel 1 output to pin Disabled.
     * |        |          |1 = BPWM channel 1 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must also be switched to BPWM function
     */
    __IO uint32_t CLKPSC;                /*!< [0x0000] Basic PWM Pre-scalar Register                                    */
    __IO uint32_t CLKDIV;                /*!< [0x0004] Basic PWM Clock Source Divider Select Register                   */
    __IO uint32_t CTL;                   /*!< [0x0008] Basic PWM Control Register                                       */
    __IO uint32_t PERIOD0;               /*!< [0x000c] Basic PWM Period Counter Register 0                              */
    __IO uint32_t CMPDAT0;               /*!< [0x0010] Basic PWM Comparator Register 0                                  */
    __I  uint32_t CNT0;                  /*!< [0x0014] Basic PWM Data Register 0                                        */
    __IO uint32_t PERIOD1;               /*!< [0x0018] Basic PWM Period Counter Register 1                              */
    __IO uint32_t CMPDAT1;               /*!< [0x001c] Basic PWM Comparator Register 1                                  */
    __I  uint32_t CNT1;                  /*!< [0x0020] Basic PWM Data Register 1                                        */
    __I  uint32_t RESERVE0[7];
    __IO uint32_t INTEN;                 /*!< [0x0040] Basic PWM Interrupt Enable Register                              */
    __IO uint32_t INTSTS;                /*!< [0x0044] Basic PWM Interrupt Indication Register                          */
    __I  uint32_t RESERVE1[13];
    __IO uint32_t POEN;                  /*!< [0x007c] Basic PWM Output Enable                                          */

} BPWM_T;

/**
    @addtogroup BPWM_CONST BPWM Bit Field Definition
    Constant Definitions for BPWM Controller
@{ */

#define BPWM_CLKPSC_CLKPSC01_Pos         (0)                                               /*!< BPWM_T::CLKPSC: CLKPSC01 Position      */
#define BPWM_CLKPSC_CLKPSC01_Msk         (0xfful << BPWM_CLKPSC_CLKPSC01_Pos)              /*!< BPWM_T::CLKPSC: CLKPSC01 Mask          */

#define BPWM_CLKPSC_DTI01_Pos            (16)                                              /*!< BPWM_T::CLKPSC: DTI01 Position         */
#define BPWM_CLKPSC_DTI01_Msk            (0xfful << BPWM_CLKPSC_DTI01_Pos)                 /*!< BPWM_T::CLKPSC: DTI01 Mask             */

#define BPWM_CLKDIV_CLKDIV0_Pos          (0)                                               /*!< BPWM_T::CLKDIV: CLKDIV0 Position       */
#define BPWM_CLKDIV_CLKDIV0_Msk          (0x7ul << BPWM_CLKDIV_CLKDIV0_Pos)                /*!< BPWM_T::CLKDIV: CLKDIV0 Mask           */

#define BPWM_CLKDIV_CLKDIV1_Pos          (4)                                               /*!< BPWM_T::CLKDIV: CLKDIV1 Position       */
#define BPWM_CLKDIV_CLKDIV1_Msk          (0x7ul << BPWM_CLKDIV_CLKDIV1_Pos)                /*!< BPWM_T::CLKDIV: CLKDIV1 Mask           */

#define BPWM_CTL_CNTEN0_Pos              (0)                                               /*!< BPWM_T::CTL: CNTEN0 Position           */
#define BPWM_CTL_CNTEN0_Msk              (0x1ul << BPWM_CTL_CNTEN0_Pos)                    /*!< BPWM_T::CTL: CNTEN0 Mask               */

#define BPWM_CTL_PINV0_Pos               (1)                                               /*!< BPWM_T::CTL: PINV0 Position            */
#define BPWM_CTL_PINV0_Msk               (0x1ul << BPWM_CTL_PINV0_Pos)                     /*!< BPWM_T::CTL: PINV0 Mask                */

#define BPWM_CTL_CMPINV0_Pos             (2)                                               /*!< BPWM_T::CTL: CMPINV0 Position          */
#define BPWM_CTL_CMPINV0_Msk             (0x1ul << BPWM_CTL_CMPINV0_Pos)                   /*!< BPWM_T::CTL: CMPINV0 Mask              */

#define BPWM_CTL_CNTMODE0_Pos            (3)                                               /*!< BPWM_T::CTL: CNTMODE0 Position         */
#define BPWM_CTL_CNTMODE0_Msk            (0x1ul << BPWM_CTL_CNTMODE0_Pos)                  /*!< BPWM_T::CTL: CNTMODE0 Mask             */

#define BPWM_CTL_DTCNT01_Pos             (4)                                               /*!< BPWM_T::CTL: DTCNT01 Position          */
#define BPWM_CTL_DTCNT01_Msk             (0x1ul << BPWM_CTL_DTCNT01_Pos)                   /*!< BPWM_T::CTL: DTCNT01 Mask              */

#define BPWM_CTL_CNTEN1_Pos              (8)                                               /*!< BPWM_T::CTL: CNTEN1 Position           */
#define BPWM_CTL_CNTEN1_Msk              (0x1ul << BPWM_CTL_CNTEN1_Pos)                    /*!< BPWM_T::CTL: CNTEN1 Mask               */

#define BPWM_CTL_PINV1_Pos               (9)                                               /*!< BPWM_T::CTL: PINV1 Position            */
#define BPWM_CTL_PINV1_Msk               (0x1ul << BPWM_CTL_PINV1_Pos)                     /*!< BPWM_T::CTL: PINV1 Mask                */

#define BPWM_CTL_CMPINV1_Pos             (10)                                              /*!< BPWM_T::CTL: CMPINV1 Position          */
#define BPWM_CTL_CMPINV1_Msk             (0x1ul << BPWM_CTL_CMPINV1_Pos)                   /*!< BPWM_T::CTL: CMPINV1 Mask              */

#define BPWM_CTL_CNTMODE1_Pos            (11)                                              /*!< BPWM_T::CTL: CNTMODE1 Position         */
#define BPWM_CTL_CNTMODE1_Msk            (0x1ul << BPWM_CTL_CNTMODE1_Pos)                  /*!< BPWM_T::CTL: CNTMODE1 Mask             */

#define BPWM_CTL_CNTTYPE01_Pos           (30)                                              /*!< BPWM_T::CTL: CNTTYPE01 Position        */
#define BPWM_CTL_CNTTYPE01_Msk           (0x1ul << BPWM_CTL_CNTTYPE01_Pos)                 /*!< BPWM_T::CTL: CNTTYPE01 Mask            */

#define BPWM_PERIOD0_PERIOD_Pos          (0)                                               /*!< BPWM_T::PERIOD0: PERIOD Position       */
#define BPWM_PERIOD0_PERIOD_Msk          (0xfffful << BPWM_PERIOD0_PERIOD_Pos)             /*!< BPWM_T::PERIOD0: PERIOD Mask           */

#define BPWM_CMPDAT0_CMP_Pos             (0)                                               /*!< BPWM_T::CMPDAT0: CMP Position          */
#define BPWM_CMPDAT0_CMP_Msk             (0xfffful << BPWM_CMPDAT0_CMP_Pos)                /*!< BPWM_T::CMPDAT0: CMP Mask              */

#define BPWM_CNT0_CNT_Pos                (0)                                               /*!< BPWM_T::CNT0: CNT Position             */
#define BPWM_CNT0_CNT_Msk                (0xfffful << BPWM_CNT0_CNT_Pos)                   /*!< BPWM_T::CNT0: CNT Mask                 */

#define BPWM_PERIOD1_PERIOD_Pos          (0)                                               /*!< BPWM_T::PERIOD1: PERIOD Position       */
#define BPWM_PERIOD1_PERIOD_Msk          (0xfffful << BPWM_PERIOD1_PERIOD_Pos)             /*!< BPWM_T::PERIOD1: PERIOD Mask           */

#define BPWM_CMPDAT1_CMP_Pos             (0)                                               /*!< BPWM_T::CMPDAT1: CMP Position          */
#define BPWM_CMPDAT1_CMP_Msk             (0xfffful << BPWM_CMPDAT1_CMP_Pos)                /*!< BPWM_T::CMPDAT1: CMP Mask              */

#define BPWM_CNT1_CNT_Pos                (0)                                               /*!< BPWM_T::CNT1: CNT Position             */
#define BPWM_CNT1_CNT_Msk                (0xfffful << BPWM_CNT1_CNT_Pos)                   /*!< BPWM_T::CNT1: CNT Mask                 */

#define BPWM_INTEN_PIEN0_Pos             (0)                                               /*!< BPWM_T::INTEN: PIEN0 Position          */
#define BPWM_INTEN_PIEN0_Msk             (0x1ul << BPWM_INTEN_PIEN0_Pos)                   /*!< BPWM_T::INTEN: PIEN0 Mask              */

#define BPWM_INTEN_PIEN1_Pos             (1)                                               /*!< BPWM_T::INTEN: PIEN1 Position          */
#define BPWM_INTEN_PIEN1_Msk             (0x1ul << BPWM_INTEN_PIEN1_Pos)                   /*!< BPWM_T::INTEN: PIEN1 Mask              */

#define BPWM_INTEN_DIEN0_Pos             (8)                                               /*!< BPWM_T::INTEN: DIEN0 Position          */
#define BPWM_INTEN_DIEN0_Msk             (0x1ul << BPWM_INTEN_DIEN0_Pos)                   /*!< BPWM_T::INTEN: DIEN0 Mask              */

#define BPWM_INTEN_DIEN1_Pos             (9)                                               /*!< BPWM_T::INTEN: DIEN1 Position          */
#define BPWM_INTEN_DIEN1_Msk             (0x1ul << BPWM_INTEN_DIEN1_Pos)                   /*!< BPWM_T::INTEN: DIEN1 Mask              */

#define BPWM_INTEN_PINTTYPE_Pos          (16)                                              /*!< BPWM_T::INTEN: PINTTYPE Position       */
#define BPWM_INTEN_PINTTYPE_Msk          (0x1ul << BPWM_INTEN_PINTTYPE_Pos)                /*!< BPWM_T::INTEN: PINTTYPE Mask           */

#define BPWM_INTSTS_PIF0_Pos             (0)                                               /*!< BPWM_T::INTSTS: PIF0 Position          */
#define BPWM_INTSTS_PIF0_Msk             (0x1ul << BPWM_INTSTS_PIF0_Pos)                   /*!< BPWM_T::INTSTS: PIF0 Mask              */

#define BPWM_INTSTS_PIF1_Pos             (1)                                               /*!< BPWM_T::INTSTS: PIF1 Position          */
#define BPWM_INTSTS_PIF1_Msk             (0x1ul << BPWM_INTSTS_PIF1_Pos)                   /*!< BPWM_T::INTSTS: PIF1 Mask              */

#define BPWM_INTSTS_DIF0_Pos             (8)                                               /*!< BPWM_T::INTSTS: DIF0 Position          */
#define BPWM_INTSTS_DIF0_Msk             (0x1ul << BPWM_INTSTS_DIF0_Pos)                   /*!< BPWM_T::INTSTS: DIF0 Mask              */

#define BPWM_INTSTS_DIF1_Pos             (9)                                               /*!< BPWM_T::INTSTS: DIF1 Position          */
#define BPWM_INTSTS_DIF1_Msk             (0x1ul << BPWM_INTSTS_DIF1_Pos)                   /*!< BPWM_T::INTSTS: DIF1 Mask              */

#define BPWM_POEN_POEN0_Pos              (0)                                               /*!< BPWM_T::POEN: POEN0 Position           */
#define BPWM_POEN_POEN0_Msk              (0x1ul << BPWM_POEN_POEN0_Pos)                    /*!< BPWM_T::POEN: POEN0 Mask               */

#define BPWM_POEN_POEN1_Pos              (1)                                               /*!< BPWM_T::POEN: POEN1 Position           */
#define BPWM_POEN_POEN1_Msk              (0x1ul << BPWM_POEN_POEN1_Pos)                    /*!< BPWM_T::POEN: POEN1 Mask               */

/**@}*/ /* BPWM_CONST */
/**@}*/ /* end of BPWM register group */


/*---------------------- Analog Comparator Controller -------------------------*/
/**
    @addtogroup ACMP Analog Comparator Controller(ACMP)
    Memory Mapped Structure for ACMP Controller
@{ */

typedef struct
{


    /**
     * @var ACMP_T::CTL
     * Offset: 0x00 Analog Comparator0 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPEN    |Comparator Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Comparator output needs to wait 2 us stable time after ACMPEN is set.
     * |[1]     |ACMPIE    |Comparator Interrupt Enable Bit
     * |        |          |0 = ACMP interrupt function Disabled.
     * |        |          |1 = ACMP interrupt function Enabled.
     * |        |          |Note1: Interrupt is generated if ACMPIE bit is set to "1" after ACMP conversion finished.
     * |        |          |Note2: ACMP interrupt will wake CPU up in power-down mode.
     * |[3:2]   |ACMPHYSEN |Comparator0 Hysteresis Enable (Only 20mV)
     * |        |          |00 = ACMP0 Hysteresis function Disabled (Default).
     * |        |          |01 = ACMP0 Hysteresis function at comparator 0 Enabled that the typical range is 20mV.
     * |        |          |10/11 = ACMP0 Hysteresis function Disabled.
     * |[5:4]   |EDGESEL   |Interrupt Flag Trigger Edge Detection
     * |        |          |00 = Disable.
     * |        |          |01 = Rising.
     * |        |          |10 = Falling.
     * |        |          |11 = Rising/Falling.
     * |[6]     |PBRKSEL   |ACMP to EPWM Brake Selection
     * |        |          |0 = ACMP Result direct output.
     * |        |          |1 = ACMP Delay Trigger Result output.
     * |[9:8]   |DLYTRGSEL |Analog Comparator Delay Trigger Mode Trigger Level Selection
     * |        |          |00 = Disable.
     * |        |          |01 = Rising.
     * |        |          |10 = Falling.
     * |        |          |11 = Rising/Falling.
     * |[11:10] |DLYTRGSOR |Analog Comparator Delay Trigger Mode Trigger Source Selection
     * |        |          |00 = PWM0.
     * |        |          |01 = PWM2.
     * |        |          |10 = PWM4.
     * |        |          |11 = Reserved.
     * |[12]    |DLYTRGEN  |Analog Comparator Delay Trigger Mode Enable
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * |[13]    |DLYTRGIE  |Analog Comparator Delay Trigger Mode Interrupt Enable
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * |[19]    |POLARITY  |Analog Comparator Polarity Control
     * |        |          |0 = Analog Comparator normal output.
     * |        |          |1 = Analog Comparator invert output.
     * |[21:20] |NFCLKS    |Noise Filter Clock Pre-divided Selection
     * |        |          |To determine the sampling frequency of the Noise Filter clock
     * |        |          |00 = PCLK.
     * |        |          |01 = PCLK / 2.
     * |        |          |10 = PCLK / 4.
     * |        |          |11 = PCLK / 16.
     * |[23]    |NFDIS     |Disable Comparator Noise Filter
     * |        |          |0 = Noise filter Enable.
     * |        |          |1 = Noise filter Disable.
     * |[25:24] |CPNSEL    |Comparator Negative Input Select
     * |        |          |00 = ACMP0_N (PB.4).
     * |        |          |01 = Band_Gap.
     * |        |          |10 = CRV.
     * |        |          |11 = Reserved.
     * |[30:28] |CPPSEL    |Comparator Positive Input Select
     * |        |          |000 = ACMP0_P0 (PB.0).
     * |        |          |001 = ACMP0_P1 (PB.1).
     * |        |          |010 = ACMP0_P2 (PB.2).
     * |        |          |011 = ACMP0_P3 (PC.1).
     * |        |          |100 = PGA_CMP.
     * |[31]    |PRESET    |Comparator Result Preset Value
     * |        |          |0 = 0 for preset value.
     * |        |          |1 = 1 for preset value.
     * @var ACMP_T::CTL
     * Offset: 0x04  Analog Comparator1 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPEN    |Comparator Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Comparator output needs to wait 2 us stable time after ACMPEN is set.
     * |[1]     |ACMPIE    |Comparator Interrupt Enable Bit
     * |        |          |0 = ACMP interrupt function Disabled.
     * |        |          |1 = ACMP interrupt function Enabled.
     * |        |          |Note1: Interrupt is generated if ACMPIE bit is set to ...1u201D after ACMP conversion finished.
     * |        |          |Note2: ACMP interrupt will wake CPU up in power-down mode.
     * |[3:2]   |ACMPHYSEN |Comparator1 Hysteresis Enable (Only 20mV)
     * |        |          |00 = ACMP1 Hysteresis function Disabled (Default).
     * |        |          |01 = ACMP1 Hysteresis function at comparator 0 Enabled that the typical range is 20mV.
     * |        |          |10/11 = ACMP1 Hysteresis function Disabled.
     * |[5:4]   |EDGESEL   |Interrupt Flag Trigger Edge Detection
     * |        |          |00 = Disable.
     * |        |          |01 = Rising.
     * |        |          |10 = Falling.
     * |        |          |11 = Rising/Falling.
     * |[6]     |PBRKSEL   |ACMP to EPWM Brake Selection
     * |        |          |0 = ACMP Result direct output.
     * |        |          |1 = ACMP Delay Trigger Result output.
     * |[9:8]   |DLYTRGSEL |Analog Comparator Delay Trigger Mode Trigger Level Selection
     * |        |          |00 = Disable.
     * |        |          |01 = Rising.
     * |        |          |10 = Falling.
     * |        |          |11 = Rising/Falling.
     * |[11:10] |DLYTRGSOR |Analog Comparator Delay Trigger Mode Trigger Source Selection
     * |        |          |00 = PWM0.
     * |        |          |01 = PWM2.
     * |        |          |10 = PWM4.
     * |        |          |11 = Reserved.
     * |[12]    |DLYTRGEN  |Analog Comparator Delay Trigger Mode Enable
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * |[13]    |DLYTRGIE  |Analog Comparator Delay Trigger Mode Interrupt Enable
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * |[19]    |POLARITY  |Analog Comparator Polarity Control
     * |        |          |0 = Analog Comparator normal output.
     * |        |          |1 = Analog Comparator invert output.
     * |[21:20] |NFCLKS    |Noise Filter Clock Pre-divided Selection
     * |        |          |To determine the sampling frequency of the Noise Filter clock
     * |        |          |00 = PCLK.
     * |        |          |01 = PCLK / 2.
     * |        |          |10 = PCLK / 4.
     * |        |          |11 = PCLK / 16.
     * |[23]    |NFDIS     |Disable Comparator Noise Filter
     * |        |          |0 = Noise filter Enable.
     * |        |          |1 = Noise filter Disable.
     * |[25:24] |CPNSEL    |Comparator Negative Input Select
     * |        |          |00 = ACMP1_N (PB.3).
     * |        |          |01 = Band_Gap.
     * |        |          |10 = CRV.
     * |        |          |11 = Reserved.
     * |[30:28] |CPPSEL    |Comparator Positive Input Select
     * |        |          |00 = ACMP1_P0 (PC.0).
     * |        |          |01 = ACMP1_P1 (PC.1).
     * |        |          |10 = ACMP1_P2 (PD.1).
     * |        |          |11 = PGA_CMP.
     * |[31]    |PRESET    |Comparator Result Preset Value
     * |        |          |0 = 0 for preset value.
     * |        |          |1 = 1 for preset value.
     * @var ACMP_T::STATUS
     * Offset: 0x08  Analog Comparator Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPF0    |Comparator0 Flag
     * |        |          |This bit is set by hardware whenever the comparator0 output changes state
     * |        |          |This will cause an interrupt if ACMPIE set.
     * |        |          |Write "1" to clear this bit to zero.
     * |[1]     |ACMPF1    |Comparator1 Flag
     * |        |          |This bit is set by hardware whenever the comparator1 output changes state
     * |        |          |This will cause an interrupt if ACMPIE set.
     * |        |          |Write "1" to clear this bit to zero.
     * |[2]     |ACMPO0    |Comparator0 Output
     * |        |          |Synchronized to the APB clock to allow reading by software
     * |        |          |Cleared when the comparator is disabled (ACMPEN = 0).
     * |[3]     |ACMPO1    |Comparator1 Output
     * |        |          |Synchronized to the APB clock to allow reading by software
     * |        |          |Cleared when the comparator is disabled (ACMPEN = 0).
     * |[4]     |DLYTRGF0  |Comparator0 Flag
     * |        |          |This bit is set by hardware whenever the comparator0 output changes state
     * |        |          |This will cause an interrupt if DLYTRGIEN set.
     * |        |          |Write "1" to clear this bit to zero.
     * |[5]     |DLYTRGF1  |Comparator1 Flag
     * |        |          |This bit is set by hardware whenever the comparator1 output changes state
     * |        |          |This will cause an interrupt if DLYTRGIEN set.
     * |        |          |Write "1" to clear this bit to zero.
     * |[6]     |DLYTRGO0  |Analog Comparator0 Delay Trigger Mode Comparator Output
     * |        |          |Synchronized to the APB clock to allow reading by software
     * |        |          |Cleared when the comparator is disabled (DLYTRGEN = 0).
     * |[7]     |DLYTRGO1  |Analog Comparator1 Delay Trigger Mode Comparator Output
     * |        |          |Synchronized to the APB clock to allow reading by software
     * |        |          |Cleared when the comparator is disabled (DLYTRGEN = 0).
     * @var ACMP_T::VREF
     * Offset: 0x0C  Analog Comparator Reference Voltage Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CRVCTL    |Comparator Reference Voltage Setting
     * |        |          |CRVS = AVDD x (1/6+CRV[3:0]/24).
     * @var ACMP_T::TRGDLY
     * Offset: 0x10  Analog Comparator Delay Trigger Mode Dleay Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8:0]   |DELAY     |Analog Comparator Delay Trigger Mode Dleay cycle
     */
    __IO uint32_t CTL[2];                /*!< [0x0000]/[0x0004] Analog Comparator0 Control Register                     */
    __IO uint32_t STATUS;                /*!< [0x0008] Analog Comparator Status Register                                */
    __IO uint32_t VREF;                  /*!< [0x000c] Analog Comparator Reference Voltage Control Register             */
    __IO uint32_t TRGDLY;                /*!< [0x0010] Analog Comparator Delay Trigger Mode Dleay Register              */

} ACMP_T;

/**
    @addtogroup ACMP_CONST ACMP Bit Field Definition
    Constant Definitions for ACMP Controller
@{ */

#define ACMP_CTL_ACMPEN_Pos              (0)                                               /*!< ACMP_T::CTL: ACMPEN Position           */
#define ACMP_CTL_ACMPEN_Msk              (0x1ul << ACMP_CTL_ACMPEN_Pos)                    /*!< ACMP_T::CTL: ACMPEN Mask               */

#define ACMP_CTL_ACMPIE_Pos              (1)                                               /*!< ACMP_T::CTL: ACMPIE Position           */
#define ACMP_CTL_ACMPIE_Msk              (0x1ul << ACMP_CTL_ACMPIE_Pos)                    /*!< ACMP_T::CTL: ACMPIE Mask               */

#define ACMP_CTL_ACMPHYSEN_Pos           (2)                                               /*!< ACMP_T::CTL: ACMPHYSEN Position        */
#define ACMP_CTL_ACMPHYSEN_Msk           (0x3ul << ACMP_CTL_ACMPHYSEN_Pos)                 /*!< ACMP_T::CTL: ACMPHYSEN Mask            */

#define ACMP_CTL_EDGESEL_Pos             (4)                                               /*!< ACMP_T::CTL: EDGESEL Position          */
#define ACMP_CTL_EDGESEL_Msk             (0x3ul << ACMP_CTL_EDGESEL_Pos)                   /*!< ACMP_T::CTL: EDGESEL Mask              */

#define ACMP_CTL_PBRKSEL_Pos             (6)                                               /*!< ACMP_T::CTL: PBRKSEL Position          */
#define ACMP_CTL_PBRKSEL_Msk             (0x1ul << ACMP_CTL_PBRKSEL_Pos)                   /*!< ACMP_T::CTL: PBRKSEL Mask              */

#define ACMP_CTL_DLYTRGSEL_Pos           (8)                                               /*!< ACMP_T::CTL: DLYTRGSEL Position        */
#define ACMP_CTL_DLYTRGSEL_Msk           (0x3ul << ACMP_CTL_DLYTRGSEL_Pos)                 /*!< ACMP_T::CTL: DLYTRGSEL Mask            */

#define ACMP_CTL_DLYTRGSOR_Pos           (10)                                              /*!< ACMP_T::CTL: DLYTRGSOR Position        */
#define ACMP_CTL_DLYTRGSOR_Msk           (0x3ul << ACMP_CTL_DLYTRGSOR_Pos)                 /*!< ACMP_T::CTL: DLYTRGSOR Mask            */

#define ACMP_CTL_DLYTRGEN_Pos            (12)                                              /*!< ACMP_T::CTL: DLYTRGEN Position         */
#define ACMP_CTL_DLYTRGEN_Msk            (0x1ul << ACMP_CTL_DLYTRGEN_Pos)                  /*!< ACMP_T::CTL: DLYTRGEN Mask             */

#define ACMP_CTL_DLYTRGIE_Pos            (13)                                              /*!< ACMP_T::CTL: DLYTRGIE Position         */
#define ACMP_CTL_DLYTRGIE_Msk            (0x1ul << ACMP_CTL_DLYTRGIE_Pos)                  /*!< ACMP_T::CTL: DLYTRGIE Mask             */

#define ACMP_CTL_POLARITY_Pos            (19)                                              /*!< ACMP_T::CTL: POLARITY Position         */
#define ACMP_CTL_POLARITY_Msk            (0x1ul << ACMP_CTL_POLARITY_Pos)                  /*!< ACMP_T::CTL: POLARITY Mask             */

#define ACMP_CTL_NFCLKS_Pos              (20)                                              /*!< ACMP_T::CTL: NFCLKS Position           */
#define ACMP_CTL_NFCLKS_Msk              (0x3ul << ACMP_CTL_NFCLKS_Pos)                    /*!< ACMP_T::CTL: NFCLKS Mask               */

#define ACMP_CTL_NFDIS_Pos               (23)                                              /*!< ACMP_T::CTL: NFDIS Position            */
#define ACMP_CTL_NFDIS_Msk               (0x1ul << ACMP_CTL_NFDIS_Pos)                     /*!< ACMP_T::CTL: NFDIS Mask                */

#define ACMP_CTL_CPNSEL_Pos              (24)                                              /*!< ACMP_T::CTL: CPNSEL Position           */
#define ACMP_CTL_CPNSEL_Msk              (0x3ul << ACMP_CTL_CPNSEL_Pos)                    /*!< ACMP_T::CTL: CPNSEL Mask               */

#define ACMP_CTL_CPPSEL_Pos              (28)                                              /*!< ACMP_T::CTL: CPPSEL Position           */
#define ACMP_CTL_CPPSEL_Msk              (0x7ul << ACMP_CTL_CPPSEL_Pos)                    /*!< ACMP_T::CTL: CPPSEL Mask               */

#define ACMP_CTL_PRESET_Pos              (31)                                              /*!< ACMP_T::CTL: PRESET Position           */
#define ACMP_CTL_PRESET_Msk              (0x1ul << ACMP_CTL_PRESET_Pos)                    /*!< ACMP_T::CTL: PRESET Mask               */


#define ACMP_STATUS_ACMPF0_Pos           (0)                                               /*!< ACMP_T::STATUS: ACMPF0 Position        */
#define ACMP_STATUS_ACMPF0_Msk           (0x1ul << ACMP_STATUS_ACMPF0_Pos)                 /*!< ACMP_T::STATUS: ACMPF0 Mask            */

#define ACMP_STATUS_ACMPF1_Pos           (1)                                               /*!< ACMP_T::STATUS: ACMPF1 Position        */
#define ACMP_STATUS_ACMPF1_Msk           (0x1ul << ACMP_STATUS_ACMPF1_Pos)                 /*!< ACMP_T::STATUS: ACMPF1 Mask            */

#define ACMP_STATUS_ACMPO0_Pos           (2)                                               /*!< ACMP_T::STATUS: ACMPO0 Position        */
#define ACMP_STATUS_ACMPO0_Msk           (0x1ul << ACMP_STATUS_ACMPO0_Pos)                 /*!< ACMP_T::STATUS: ACMPO0 Mask            */

#define ACMP_STATUS_ACMPO1_Pos           (3)                                               /*!< ACMP_T::STATUS: ACMPO1 Position        */
#define ACMP_STATUS_ACMPO1_Msk           (0x1ul << ACMP_STATUS_ACMPO1_Pos)                 /*!< ACMP_T::STATUS: ACMPO1 Mask            */

#define ACMP_STATUS_DLYTRGF0_Pos         (4)                                               /*!< ACMP_T::STATUS: DLYTRGF0 Position      */
#define ACMP_STATUS_DLYTRGF0_Msk         (0x1ul << ACMP_STATUS_DLYTRGF0_Pos)               /*!< ACMP_T::STATUS: DLYTRGF0 Mask          */

#define ACMP_STATUS_DLYTRGF1_Pos         (5)                                               /*!< ACMP_T::STATUS: DLYTRGF1 Position      */
#define ACMP_STATUS_DLYTRGF1_Msk         (0x1ul << ACMP_STATUS_DLYTRGF1_Pos)               /*!< ACMP_T::STATUS: DLYTRGF1 Mask          */

#define ACMP_STATUS_DLYTRGO0_Pos         (6)                                               /*!< ACMP_T::STATUS: DLYTRGO0 Position      */
#define ACMP_STATUS_DLYTRGO0_Msk         (0x1ul << ACMP_STATUS_DLYTRGO0_Pos)               /*!< ACMP_T::STATUS: DLYTRGO0 Mask          */

#define ACMP_STATUS_DLYTRGO1_Pos         (7)                                               /*!< ACMP_T::STATUS: DLYTRGO1 Position      */
#define ACMP_STATUS_DLYTRGO1_Msk         (0x1ul << ACMP_STATUS_DLYTRGO1_Pos)               /*!< ACMP_T::STATUS: DLYTRGO1 Mask          */

#define ACMP_VREF_CRVCTL_Pos             (0)                                               /*!< ACMP_T::VREF: CRVCTL Position          */
#define ACMP_VREF_CRVCTL_Msk             (0xful << ACMP_VREF_CRVCTL_Pos)                   /*!< ACMP_T::VREF: CRVCTL Mask              */

#define ACMP_TRGDLY_DELAY_Pos            (0)                                               /*!< ACMP_T::TRGDLY: DELAY Position         */
#define ACMP_TRGDLY_DELAY_Msk            (0x1fful << ACMP_TRGDLY_DELAY_Pos)                /*!< ACMP_T::TRGDLY: DELAY Mask             */

/**@}*/ /* ACMP_CONST */
/**@}*/ /* end of ACMP register group */


/*---------------------- Enhanced Analog to Digital Converter -------------------------*/
/**
    @addtogroup EADC Enhanced Analog to Digital Converter(EADC)
    Memory Mapped Structure for EADC Controller
@{ */

typedef struct
{


    /**
     * @var EADC_T::DAT
     * Offset: 0x00  EADC data register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |ADC0DAT0  |ADC0 Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[14]    |ADC0OV    |ADC0 over Run Flag
     * |        |          |0 = Data in ADC0DAT0[11:0] is recent conversion result.
     * |        |          |1 = Data in ADC0DAT0[11:0] overwritten.
     * |        |          |If converted data in ADC0DAT0[11:0] has not been read before
     * |        |          |the new conversion result is loaded to this register, OV is set to "1".
     * |        |          |It is cleared by hardware after the ADC_DAT0 register is read.
     * |[15]    |ADC0VALID |ADC0 Valid Flag
     * |        |          |0 = Data in ADC0DAT0[11:0] bits not valid.
     * |        |          |1 = Data in ADC0DAT0[11:0] bits valid.
     * |        |          |This bit is set to "1" when ADC conversion is completed
     * |        |          |and cleared by hardware after the ADC_DAT0 register is read.
     * |[27:16] |ADC1DAT0  |ADC1 Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[30]    |ADC1OV    |ADC1 over Run Flag
     * |        |          |0 = Data in ADC1DAT0[27:16] is recent conversion result.
     * |        |          |1 = Data in ADC1DAT0[27:16] overwritten.
     * |        |          |If converted data in ADC1DAT0[27:16] has not been read before
     * |        |          |the new conversion result is loaded to this register, OV is set to "1".
     * |        |          |It is cleared by hardware after the ADC_DAT0 register is read.
     * |[31]    |ADC1VALID |ADC1 Valid Flag
     * |        |          |0 = Data in ADC1DAT0[27:16] bits not valid.
     * |        |          |1 = Data in ADC1DAT0[27:16] bits valid.
     * |        |          |This bit is set to "1" when ADC conversion is completed
     * |        |          |and cleared by hardware after the ADC_DAT0 register is read.
     * @var EADC_T::DAT
     * Offset: 0x04  EADC Data Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |ADC0DAT1  |ADC0 Conversion Result for FIFO1
     * |        |          |This field contains conversion result of ADC.
     * |[14]    |ADC0OV    |ADC0Over Run Flag
     * |        |          |0 = Data in ADC0DAT1[11:0] is recent conversion result.
     * |        |          |1 = Data in ADC0DAT1[11:0]] overwritten.
     * |        |          |If converted data in ADC0DAT1[11:0] has not been read before
     * |        |          |the new conversion result is loaded to this register, OV is set to "1".
     * |        |          |It is cleared by hardware after the ADC_DAT1 register is read.
     * |[15]    |ADC0VALID |ADC0 Valid Flag
     * |        |          |0 = Data in ADC0DAT1[11:0] bits not valid.
     * |        |          |1 = Data in ADC0DAT1[11:0] bits valid.
     * |        |          |This bit is set to "1" when ADC conversion is completed
     * |        |          |and cleared by hardware after the ADC_DAT1 register is read.
     * |[27:16] |ADC1DAT1  |ADC1 Conversion Result for FIFO1
     * |        |          |This field contains conversion result of ADC.
     * |[30]    |ADC1OV    |ADC1 over Run Flag
     * |        |          |0 = Data in ADC1DAT1[27:16] is recent conversion result.
     * |        |          |1 = Data in ADC1DAT1[27:16] overwritten.
     * |        |          |If converted data in ADC1DAT1[27:16] has not been read before
     * |        |          |the new conversion result is loaded to this register, OV is set to "1".
     * |        |          |It is cleared by hardware after the ADC_DAT1 register is read.
     * |[31]    |ADC1VALID |ADC1 Valid Flag
     * |        |          |0 = Data in ADC1DAT1[27:16] bits not valid.
     * |        |          |1 = Data in ADC1DAT1[27:16] bits valid.
     * |        |          |This bit is set to "1" when ADC conversion is completed
     * |        |          |and cleared by hardware after the ADC_DAT1 register is read.
     * @var EADC_T::CTL
     * Offset: 0x20  EADC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCEN     |EADC Converter Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Before starting the A/D conversion function, this bit should be set to "1".
     * |        |          |Clear it to "0" to disable A/D converter analog circuit power consumption.
     * |[1]     |ADC0IEN   |ADC0 Interrupt Enable
     * |        |          |0 = ADC0 interrupt function Disabled.
     * |        |          |1 = ADC0 interrupt function Enabled.
     * |        |          |A/D conversion end interrupt request is generated if ADC0IEN bit is set to "1".
     * |[2]     |ADC0HWTRGEN|Hardware Trigger ADC Convertion Enable
     * |        |          |Enable or disable triggering of A/D conversion by Hardware (PWM, Timer, ADC self)
     * |        |          |0= Disabled.
     * |        |          |1= Enabled.
     * |[3]     |ADC0SWTRG |ADC0 Conversion Start
     * |        |          |0 = Conversion stopped and A/D converter entered idle state.
     * |        |          |1 = Conversion start.
     * |        |          |ADC0SWTRG bit can be set to "1" from two sources: software and external pin STADC.
     * |        |          |ADC0SWTRG will be cleared to "0" by hardware automatically.
     * |[5]     |ADCSS3R   |ADC Simultaneous Sequential 3 data at ADCMODE = 11
     * |        |          |0 = convert sequential is ADC0 -> ADC1 -> ADC0 -> ADC1, four datas at ADCMODE=11.
     * |        |          |1 = convert sequential is ADC0 -> ADC1 -> ADC0, three datas at ADCMODE=11.
     * |[7:6]   |ADCMODE   |ADC Conversion Mode
     * |        |          |00 = Independent simple; independent function and independent interrupt by themselves.
     * |        |          |01 = Independent 2SH; independent trigger function, ADC0 with ADC1 both convert finish then only generate interrupt ADC0IF.
     * |        |          |10 = Simultaneous Simple; simultaneous trigger function by ADC0, ADC0 with ADC1 both convert finish then generate interrupt ADC0IF.
     * |        |          |11 = Simultaneous Sequential; simultaneous trigger function by ADC0, this mode converts sequential is ADC0 -> ADC1 ->ADC0 -> ADC1 4 times, then generate interrupt ADC0IF.
     * |[9]     |ADC1IEN   |ADC1 Interrupt Enable Bit
     * |        |          |0 = ADC1 interrupt function Disabled.
     * |        |          |1 = ADC1 interrupt function Enabled.
     * |        |          |A/D conversion end interrupt request is generated if ADC1IEN bit is set to "1".
     * |[10]    |ADC1HWTRGEN|Hardware Trigger ADC Convertion Enable Bit
     * |        |          |Enable or disable triggering of A/D conversion by Hardware (PWM, Timer, ADC self)
     * |        |          |0= Hardware Trigger ADC Convertion Disabled.
     * |        |          |1= Hardware Trigger ADC Convertion Enabled.
     * |[11]    |ADC1SWTRG |ADC1 Conversion Start
     * |        |          |0 = Conversion stopped and A/D converter entered idle state.
     * |        |          |1 = Conversion start.
     * |        |          |ADC1SWTRG bit can be set to "1" from two sources: software and external pin STADC.
     * |        |          |ADC1SWTRG will be cleared to "0" by hardware automatically.
     * |[18:16] |ADC0CHSEL |ADC1 Channel Select
     * |        |          |000 = ADC0_CH0.
     * |        |          |001 = ADC0_CH1.
     * |        |          |010 = ADC0_CH2.
     * |        |          |011 = ADC0_CH3.
     * |        |          |100 = ADC0_CH4.
     * |        |          |101 = PGA_ADC.
     * |        |          |110 = BAND_GAP.
     * |        |          |111 = VSS.
     * |[22:20] |ADC0SEQSEL|ADC0 Sequential Input Pin Selection
     * |        |          |000 = ADC0_CH0.
     * |        |          |001 = ADC0_CH1.
     * |        |          |010 = ADC0_CH2.
     * |        |          |011 = ADC0_CH3.
     * |        |          |100 = ADC0_CH4.
     * |        |          |101 = PGA_ADC.
     * |        |          |110 = BAND_GAP.
     * |        |          |111 = VSS.
     * |[26:24] |ADC1CHSEL |ADC1 Channel Select
     * |        |          |000 = ADC1_CH0.
     * |        |          |001 = ADC1_CH1.
     * |        |          |010 = ADC1_CH2.
     * |        |          |011 = ADC0_CH0.
     * |        |          |100 = ADC0_CH4.
     * |        |          |101 = PGA_ADC.
     * |        |          |110 = Temp Sensor.
     * |        |          |111 = VSS.
     * |[30:28] |ADC1SEQSEL|ADC1 Sequential Input Pin Selection (Second Input)
     * |        |          |000 = ADC1_CH0.
     * |        |          |001 = ADC1_CH1.
     * |        |          |010 = ADC1_CH2.
     * |        |          |011 = ADC0_CH0.
     * |        |          |100 = ADC0_CH4.
     * |        |          |101 = PGA_ADC.
     * |        |          |110 = Temp Sensor.
     * |        |          |111 = VSS.
     * @var EADC_T::TRGSOR
     * Offset: 0x24  EADC Hardware Trigger Source Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |ADC0TRGSOR|ADC0 Trigger Source
     * |        |          |0000 = STADC.
     * |        |          |0001 = PWM0.
     * |        |          |0010 = PWM1.
     * |        |          |0011 = PWM2.
     * |        |          |0100 = PWM3.
     * |        |          |0101 = PWM4.
     * |        |          |0110 = PWM5.
     * |        |          |0111 = TMR0.
     * |        |          |1000 = TMR1.
     * |        |          |1001 = TMR2.
     * |        |          |1010 = ADC0IF.
     * |        |          |1011 = ADC1IF.
     * |        |          |1100~1111 = Reserved.
     * |[5:4]   |ADC0PWMTRGSEL|PWM Trigger Selection for ADC0
     * |        |          |00 = EPWM Signal Falling.
     * |        |          |01 = EPWM Counter Central.
     * |        |          |10 = EPWM signal Rising.
     * |        |          |11 = Period.
     * |[7:6]   |ADC0STADCSEL|ADC0 External Trigger Pin (STADC) Trigger Selection
     * |        |          |00 = Rising.
     * |        |          |01 = Falling.
     * |        |          |10 = Rising or Falling.
     * |        |          |11 = Reserved.
     * |[19:16] |ADC1TRGSOR|ADC1 Trigger Source
     * |        |          |0000 = STADC.
     * |        |          |0001 = PWM0.
     * |        |          |0010 = PWM1.
     * |        |          |0011 = PWM2.
     * |        |          |0100 = PWM3.
     * |        |          |0101 = PWM4.
     * |        |          |0110 = PWM5.
     * |        |          |0111 = TMR0.
     * |        |          |1000 = TMR1.
     * |        |          |1001 = TMR2.
     * |        |          |1010 = ADC0IF.
     * |        |          |1011 = ADC1IF.
     * |        |          |1100~1111 = Reserved.
     * |[21:20] |ADC1PWMTRGSEL|PWM Trigger Selection for ADC1
     * |        |          |00 = EPWM Signal Falling.
     * |        |          |01 = EPWM Counter Central.
     * |        |          |10 = EPWM signal Rising.
     * |        |          |11 = Period.
     * |[23:22] |ADC1STADCSEL|ADC1 External Trigger Pin (STADC) Trigger Selection
     * |        |          |00 = Rising.
     * |        |          |01 = Falling.
     * |        |          |10 = Rising or Falling.
     * |        |          |11 = Reserved.
     * @var EADC_T::TRGDLY
     * Offset: 0x28  EADC Trigger Delay Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ADC0DELAY |ADC0 Trigger Delay Timer
     * |        |          |Set this field will delay ADC start conversion time after ADCxTRGCTL trigger is coming. (x:0/1)
     * |        |          |delay time is (4 * ADC0DELAY) * system clock
     * |[23:16] |ADC1DELAY |ADC1 Trigger Delay Timer
     * |        |          |Set this field will delay ADC start conversion time after ADCxTRGCTL trigger is coming. (x:0/1)
     * |        |          |delay time is (4 * ADC1DELAY) * system clock
     * @var EADC_T::SMPCNT
     * Offset: 0x2C  EADC Sampling Time Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |ADCSMPCNT |ADC Sampling Counter
     * |        |          |ADC sampling counters are 6 ADC clock is suggestion
     * |        |          |0 = 1 * ADC Clock.
     * |        |          |1 = 2 * ADC Clock.
     * |        |          |2 = 3 * ADC Clock.
     * |        |          |3 = 4 * ADC Clock.
     * |        |          |4 = 5 * ADC Clock.
     * |        |          |5 = 6 * ADC Clock.
     * |        |          |6 = 7 * ADC Clock.
     * |        |          |7 = 8 * ADC Clock.
     * |        |          |8 = 16 * ADC Clock.
     * |        |          |9 = 32 * ADC Clock.
     * |        |          |10 = 64 * ADC Clock.
     * |        |          |11 = 128 * ADC Clock.
     * |        |          |12 = 256 * ADC Clock.
     * |        |          |13 = 512 * ADC Clock.
     * |        |          |14 = 1024 * ADC Clock.
     * |        |          |15 = 1024 * ADC Clock.
     * @var EADC_T::STATUS
     * Offset: 0x30  EADC Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADC0IF    |A/D Conversion End Flag
     * |        |          |A status flag that indicates the end of A/D conversion.
     * |        |          |ADF is set to "1" When A/D conversion ends.
     * |        |          |This flag can be cleared by writing "1" to itself.
     * |[1]     |ADC0OV    |Over Run Flag
     * |        |          |It is a mirror to OV bit in ADDR.
     * |[3]     |ADC0BUSY  |BUSY/IDLE
     * |        |          |0 = A/D converter is in idle state.
     * |        |          |1 = A/D converter is busy at conversion.
     * |        |          |This bit is mirror of as ADST bit in ADCR.
     * |[7:4]   |ADC0CH    |Current Conversion Channel
     * |        |          |This filed reflects the current conversion channel when ADC0BUSY =1.
     * |        |          |When ADC0BUSY =0, it shows the number of the next converted channel.
     * |        |          |It is read only.
     * |[8]     |ADC1IF    |ADC1 Conversion End Flag
     * |        |          |A status flag that indicates the end of A/D conversion.
     * |        |          |ADF is set to "1" When A/D conversion ends.
     * |        |          |This flag can be cleared by writing "1" to itself.
     * |[9]     |ADC1OV    |Over Run Flag
     * |        |          |It is a mirror to OV bit in ADDR.
     * |[11]    |ADC1BUSY  |BUSY/IDLE
     * |        |          |0 = A/D converter is in idle state.
     * |        |          |1 = A/D converter is busy at conversion.
     * |        |          |This bit is mirror of as ADST bit in ADCR.
     * |[15:12] |ADC1CH    |Current Conversion Channel
     * |        |          |This filed reflects the current conversion channel when ADC1BUSY =1.
     * |        |          |When ADC1BUSY =0, it shows the number of the next converted channel.
     * |        |          |It is read only.
     * |[16]    |WCMPIF    |Window Comparator Interrupt Flag
     * |        |          |When Windows Comparator has generat a result output, this bit is set to "1".
     * |        |          |Then it is cleared by writing "1" to ifself.
     * |        |          |0 = Conversion result in ADC_DAT1 does not meets the WCMPLOWDAT setting.
     * |        |          |1 = Conversion result in ADC_DAT1 meets the WCMPLOWDAT setting.
     * |[17]    |LOWFG     |Window Comparator Low Bound Flag
     * |        |          |When ADC conversion result low than the setting condition in Low Bound (WCMPLOWDAT), this bit is set to "1".
     * |        |          |Then it is cleared by writing "1" to ifself.
     * |        |          |0 = Conversion result in ADC_DAT1 does not meets the WCMPLOWDAT setting.
     * |        |          |1 = Conversion result in ADC_DAT1 meets the WCMPLOWDAT setting.
     * |[18]    |MIDFG     |Window Comparator Middle Bound Flag
     * |        |          |When ADC conversion result is between High Bound (WCMPHIGHDAT) and Low Bound (WCMPLOWDAT), this bit is set to "1".
     * |        |          |Then it is cleared by writing "1" to ifself.
     * |        |          |0 = Conversion result in ADC_DAT1 isn't between High Bound (WCMPHIGHDAT) and Low Bound (WCMPLOWDAT).
     * |        |          |1 = Conversion result in ADC_DAT1 is between High Bound (WCMPHIGHDAT) and Low Bound (WCMPLOWDAT).
     * |[19]    |HIGHFG    |Window Comparator High Bound Flag
     * |        |          |When ADC conversion result high than the setting condition in High Bound (WCMPHIGHDAT), this bit is set to "1".
     * |        |          |Then it is cleared by writing "1" to ifself.
     * |        |          |0 = Conversion result in ADC_DAT1 does not meets the WCMPHIGHDAT setting.
     * |        |          |1 = Conversion result in ADC_DAT1 meets the WCMPHIGHDAT setting.
     * @var EADC_T::WCMPCTL
     * Offset: 0x34  EADC Window Comparator Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WCMPEN    |Window Comparator Enable Bit
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * |[1]     |WCMPIEN   |Window Comparator Interrupt Enable Bit
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * |[4]     |WCMPLOWEN |Window Comparator Low Flag Enable Bit
     * |        |          |set ADC conversion result low than compare condition Low bound range
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * |[5]     |WCMPMIDEN |Window Comparator Middle Flag Enable Bit
     * |        |          |set ADC conversion result equal compare condition at Low and High bound range
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * |[6]     |WCMPHIGHEN|Window Comparator High Flag Enable Bit
     * |        |          |set ADC conversion result high than compare condition High bound range
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * |[7]     |WFLAGCTL  |Window Comparator Flag Control
     * |        |          |When the ADC conversion result matches the compare condition
     * |        |          |0 = auto-update.
     * |        |          |1 = none.
     * |[11:8]  |WCMPMCNT  |Window Compare Match Count
     * |        |          |When the ADC conversion result matches the compare condition
     * |        |          |defined by CMP Flag setting (CMPUPEN, CMPEQUEN, CMPLOWEN and WCFLAGCTL), the internal match counter will increase 1.
     * |        |          |When the internal counter reaches the value to (WCMPMCNT ), the CMPIF bit will be set.
     * |        |          |NOTE:If WCMPMCNT = 0,the counter would do 16 times.
     * @var EADC_T::WCMPDAT
     * Offset: 0x38  EADC Window Comparator Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |WCMPLOWDAT|Window Comparator Low Bound Data
     * |[27:16] |WCMPHIGHDAT|Window Comparator High Bound Data
     */
    __I  uint32_t DAT[2];                /*!< [0x0000 ~ 0x0004] EADC data register 0/1                                             */
    __I  uint32_t RESERVE0[6];
    __IO uint32_t CTL;                   /*!< [0x0020] ADC Control Register                                             */
    __IO uint32_t TRGSOR;                /*!< [0x0024] ADC Hardware Trigger Source Control Register                     */
    __IO uint32_t TRGDLY;                /*!< [0x0028] ADC Trigger Delay Control Register                               */
    __IO uint32_t SMPCNT;                /*!< [0x002c] ADC Sampling Time Counter Register                               */
    __IO uint32_t STATUS;                /*!< [0x0030] ADC Status Register                                              */
    __IO uint32_t WCMPCTL;               /*!< [0x0034] ADC Window Comparator Control Register                           */
    __IO uint32_t WCMPDAT;               /*!< [0x0038] ADC Window Comparator Data Register                              */

} EADC_T;

/**
    @addtogroup EADC_CONST EADC Bit Field Definition
    Constant Definitions for EADC Controller
@{ */

#define EADC_DAT0_ADC0DAT0_Pos            (0)                                               /*!< EADC_T::DAT0: ADC0DAT0 Position         */
#define EADC_DAT0_ADC0DAT0_Msk            (0xffful << EADC_DAT0_ADC0DAT0_Pos)               /*!< EADC_T::DAT0: ADC0DAT0 Mask             */

#define EADC_DAT0_ADC0OV_Pos              (14)                                              /*!< EADC_T::DAT0: ADC0OV Position           */
#define EADC_DAT0_ADC0OV_Msk              (0x1ul << EADC_DAT0_ADC0OV_Pos)                   /*!< EADC_T::DAT0: ADC0OV Mask               */

#define EADC_DAT0_ADC0VALID_Pos           (15)                                              /*!< EADC_T::DAT0: ADC0VALID Position        */
#define EADC_DAT0_ADC0VALID_Msk           (0x1ul << EADC_DAT0_ADC0VALID_Pos)                /*!< EADC_T::DAT0: ADC0VALID Mask            */

#define EADC_DAT0_ADC1DAT0_Pos            (16)                                              /*!< EADC_T::DAT0: ADC1DAT0 Position         */
#define EADC_DAT0_ADC1DAT0_Msk            (0xffful << EADC_DAT0_ADC1DAT0_Pos)               /*!< EADC_T::DAT0: ADC1DAT0 Mask             */

#define EADC_DAT0_ADC1OV_Pos              (30)                                              /*!< EADC_T::DAT0: ADC1OV Position           */
#define EADC_DAT0_ADC1OV_Msk              (0x1ul << EADC_DAT0_ADC1OV_Pos)                   /*!< EADC_T::DAT0: ADC1OV Mask               */

#define EADC_DAT0_ADC1VALID_Pos           (31)                                              /*!< EADC_T::DAT0: ADC1VALID Position        */
#define EADC_DAT0_ADC1VALID_Msk           (0x1ul << EADC_DAT0_ADC1VALID_Pos)                /*!< EADC_T::DAT0: ADC1VALID Mask            */

#define EADC_DAT1_ADC0DAT1_Pos            (0)                                               /*!< EADC_T::DAT1: ADC0DAT1 Position         */
#define EADC_DAT1_ADC0DAT1_Msk            (0xffful << EADC_DAT1_ADC0DAT1_Pos)               /*!< EADC_T::DAT1: ADC0DAT1 Mask             */

#define EADC_DAT1_ADC0OV_Pos              (14)                                              /*!< EADC_T::DAT1: ADC0OV Position           */
#define EADC_DAT1_ADC0OV_Msk              (0x1ul << EADC_DAT1_ADC0OV_Pos)                   /*!< EADC_T::DAT1: ADC0OV Mask               */

#define EADC_DAT1_ADC0VALID_Pos           (15)                                              /*!< EADC_T::DAT1: ADC0VALID Position        */
#define EADC_DAT1_ADC0VALID_Msk           (0x1ul << EADC_DAT1_ADC0VALID_Pos)                /*!< EADC_T::DAT1: ADC0VALID Mask            */

#define EADC_DAT1_ADC1DAT1_Pos            (16)                                              /*!< EADC_T::DAT1: ADC1DAT1 Position         */
#define EADC_DAT1_ADC1DAT1_Msk            (0xffful << EADC_DAT1_ADC1DAT1_Pos)               /*!< EADC_T::DAT1: ADC1DAT1 Mask             */

#define EADC_DAT1_ADC1OV_Pos              (30)                                              /*!< EADC_T::DAT1: ADC1OV Position           */
#define EADC_DAT1_ADC1OV_Msk              (0x1ul << EADC_DAT1_ADC1OV_Pos)                   /*!< EADC_T::DAT1: ADC1OV Mask               */

#define EADC_DAT1_ADC1VALID_Pos           (31)                                              /*!< EADC_T::DAT1: ADC1VALID Position        */
#define EADC_DAT1_ADC1VALID_Msk           (0x1ul << EADC_DAT1_ADC1VALID_Pos)                /*!< EADC_T::DAT1: ADC1VALID Mask            */

#define EADC_CTL_ADCEN_Pos                (0)                                               /*!< EADC_T::CTL: ADCEN Position             */
#define EADC_CTL_ADCEN_Msk                (0x1ul << EADC_CTL_ADCEN_Pos)                     /*!< EADC_T::CTL: ADCEN Mask                 */

#define EADC_CTL_ADC0IEN_Pos              (1)                                               /*!< EADC_T::CTL: ADC0IEN Position           */
#define EADC_CTL_ADC0IEN_Msk              (0x1ul << EADC_CTL_ADC0IEN_Pos)                   /*!< EADC_T::CTL: ADC0IEN Mask               */

#define EADC_CTL_ADC0HWTRGEN_Pos          (2)                                               /*!< EADC_T::CTL: ADC0HWTRGEN Position       */
#define EADC_CTL_ADC0HWTRGEN_Msk          (0x1ul << EADC_CTL_ADC0HWTRGEN_Pos)               /*!< EADC_T::CTL: ADC0HWTRGEN Mask           */

#define EADC_CTL_ADC0SWTRG_Pos            (3)                                               /*!< EADC_T::CTL: ADC0SWTRG Position         */
#define EADC_CTL_ADC0SWTRG_Msk            (0x1ul << EADC_CTL_ADC0SWTRG_Pos)                 /*!< EADC_T::CTL: ADC0SWTRG Mask             */

#define EADC_CTL_ADCSS3R_Pos              (5)                                               /*!< EADC_T::CTL: ADCSS3R Position           */
#define EADC_CTL_ADCSS3R_Msk              (0x1ul << EADC_CTL_ADCSS3R_Pos)                   /*!< EADC_T::CTL: ADCSS3R Mask               */

#define EADC_CTL_ADCMODE_Pos              (6)                                               /*!< EADC_T::CTL: ADCMODE Position           */
#define EADC_CTL_ADCMODE_Msk              (0x3ul << EADC_CTL_ADCMODE_Pos)                   /*!< EADC_T::CTL: ADCMODE Mask               */

#define EADC_CTL_ADC1IEN_Pos              (9)                                               /*!< EADC_T::CTL: ADC1IEN Position           */
#define EADC_CTL_ADC1IEN_Msk              (0x1ul << EADC_CTL_ADC1IEN_Pos)                   /*!< EADC_T::CTL: ADC1IEN Mask               */

#define EADC_CTL_ADC1HWTRGEN_Pos          (10)                                              /*!< EADC_T::CTL: ADC1HWTRGEN Position       */
#define EADC_CTL_ADC1HWTRGEN_Msk          (0x1ul << EADC_CTL_ADC1HWTRGEN_Pos)               /*!< EADC_T::CTL: ADC1HWTRGEN Mask           */

#define EADC_CTL_ADC1SWTRG_Pos            (11)                                              /*!< EADC_T::CTL: ADC1SWTRG Position         */
#define EADC_CTL_ADC1SWTRG_Msk            (0x1ul << EADC_CTL_ADC1SWTRG_Pos)                 /*!< EADC_T::CTL: ADC1SWTRG Mask             */

#define EADC_CTL_ADC0CHSEL_Pos            (16)                                              /*!< EADC_T::CTL: ADC0CHSEL Position         */
#define EADC_CTL_ADC0CHSEL_Msk            (0x7ul << EADC_CTL_ADC0CHSEL_Pos)                 /*!< EADC_T::CTL: ADC0CHSEL Mask             */

#define EADC_CTL_ADC0SEQSEL_Pos           (20)                                              /*!< EADC_T::CTL: ADC0SEQSEL Position        */
#define EADC_CTL_ADC0SEQSEL_Msk           (0x7ul << EADC_CTL_ADC0SEQSEL_Pos)                /*!< EADC_T::CTL: ADC0SEQSEL Mask            */

#define EADC_CTL_ADC1CHSEL_Pos            (24)                                              /*!< EADC_T::CTL: ADC1CHSEL Position         */
#define EADC_CTL_ADC1CHSEL_Msk            (0x7ul << EADC_CTL_ADC1CHSEL_Pos)                 /*!< EADC_T::CTL: ADC1CHSEL Mask             */

#define EADC_CTL_ADC1SEQSEL_Pos           (28)                                              /*!< EADC_T::CTL: ADC1SEQSEL Position        */
#define EADC_CTL_ADC1SEQSEL_Msk           (0x7ul << EADC_CTL_ADC1SEQSEL_Pos)                /*!< EADC_T::CTL: ADC1SEQSEL Mask            */

#define EADC_TRGSOR_ADC0TRGSOR_Pos        (0)                                               /*!< EADC_T::TRGSOR: ADC0TRGSOR Position     */
#define EADC_TRGSOR_ADC0TRGSOR_Msk        (0xful << EADC_TRGSOR_ADC0TRGSOR_Pos)             /*!< EADC_T::TRGSOR: ADC0TRGSOR Mask         */

#define EADC_TRGSOR_ADC0PWMTRGSEL_Pos     (4)                                               /*!< EADC_T::TRGSOR: ADC0PWMTRGSEL Position  */
#define EADC_TRGSOR_ADC0PWMTRGSEL_Msk     (0x3ul << EADC_TRGSOR_ADC0PWMTRGSEL_Pos)          /*!< EADC_T::TRGSOR: ADC0PWMTRGSEL Mask      */

#define EADC_TRGSOR_ADC0STADCSEL_Pos      (6)                                               /*!< EADC_T::TRGSOR: ADC0STADCSEL Position   */
#define EADC_TRGSOR_ADC0STADCSEL_Msk      (0x3ul << EADC_TRGSOR_ADC0STADCSEL_Pos)           /*!< EADC_T::TRGSOR: ADC0STADCSEL Mask       */

#define EADC_TRGSOR_ADC1TRGSOR_Pos        (16)                                              /*!< EADC_T::TRGSOR: ADC1TRGSOR Position     */
#define EADC_TRGSOR_ADC1TRGSOR_Msk        (0xful << EADC_TRGSOR_ADC1TRGSOR_Pos)             /*!< EADC_T::TRGSOR: ADC1TRGSOR Mask         */

#define EADC_TRGSOR_ADC1PWMTRGSEL_Pos     (20)                                              /*!< EADC_T::TRGSOR: ADC1PWMTRGSEL Position  */
#define EADC_TRGSOR_ADC1PWMTRGSEL_Msk     (0x3ul << EADC_TRGSOR_ADC1PWMTRGSEL_Pos)          /*!< EADC_T::TRGSOR: ADC1PWMTRGSEL Mask      */

#define EADC_TRGSOR_ADC1STADCSEL_Pos      (22)                                              /*!< EADC_T::TRGSOR: ADC1STADCSEL Position   */
#define EADC_TRGSOR_ADC1STADCSEL_Msk      (0x3ul << EADC_TRGSOR_ADC1STADCSEL_Pos)           /*!< EADC_T::TRGSOR: ADC1STADCSEL Mask       */

#define EADC_TRGDLY_ADC0DELAY_Pos         (0)                                               /*!< EADC_T::TRGDLY: ADC0DELAY Position      */
#define EADC_TRGDLY_ADC0DELAY_Msk         (0xfful << EADC_TRGDLY_ADC0DELAY_Pos)             /*!< EADC_T::TRGDLY: ADC0DELAY Mask          */

#define EADC_TRGDLY_ADC1DELAY_Pos         (16)                                              /*!< EADC_T::TRGDLY: ADC1DELAY Position      */
#define EADC_TRGDLY_ADC1DELAY_Msk         (0xfful << EADC_TRGDLY_ADC1DELAY_Pos)             /*!< EADC_T::TRGDLY: ADC1DELAY Mask          */

#define EADC_SMPCNT_ADCSMPCNT_Pos         (0)                                               /*!< EADC_T::SMPCNT: ADCSMPCNT Position      */
#define EADC_SMPCNT_ADCSMPCNT_Msk         (0xful << EADC_SMPCNT_ADCSMPCNT_Pos)              /*!< EADC_T::SMPCNT: ADCSMPCNT Mask          */

#define EADC_STATUS_ADC0F_Pos             (0)                                               /*!< EADC_T::STATUS: ADC0F Position          */
#define EADC_STATUS_ADC0F_Msk             (0x1ul << EADC_STATUS_ADC0F_Pos)                  /*!< EADC_T::STATUS: ADC0F Mask              */

#define EADC_STATUS_ADC0OV_Pos            (1)                                               /*!< EADC_T::STATUS: ADC0OV Position         */
#define EADC_STATUS_ADC0OV_Msk            (0x1ul << EADC_STATUS_ADC0OV_Pos)                 /*!< EADC_T::STATUS: ADC0OV Mask             */

#define EADC_STATUS_ADC0BUSY_Pos          (3)                                               /*!< EADC_T::STATUS: ADC0BUSY Position       */
#define EADC_STATUS_ADC0BUSY_Msk          (0x1ul << EADC_STATUS_ADC0BUSY_Pos)               /*!< EADC_T::STATUS: ADC0BUSY Mask           */

#define EADC_STATUS_ADC0CH_Pos            (4)                                               /*!< EADC_T::STATUS: ADC0CH Position         */
#define EADC_STATUS_ADC0CH_Msk            (0xful << EADC_STATUS_ADC0CH_Pos)                 /*!< EADC_T::STATUS: ADC0CH Mask             */

#define EADC_STATUS_ADC1F_Pos             (8)                                               /*!< EADC_T::STATUS: ADC1F Position          */
#define EADC_STATUS_ADC1F_Msk             (0x1ul << EADC_STATUS_ADC1F_Pos)                  /*!< EADC_T::STATUS: ADC1F Mask              */

#define EADC_STATUS_ADC1OV_Pos            (9)                                               /*!< EADC_T::STATUS: ADC1OV Position         */
#define EADC_STATUS_ADC1OV_Msk            (0x1ul << EADC_STATUS_ADC1OV_Pos)                 /*!< EADC_T::STATUS: ADC1OV Mask             */

#define EADC_STATUS_ADC1BUSY_Pos          (11)                                              /*!< EADC_T::STATUS: ADC1BUSY Position       */
#define EADC_STATUS_ADC1BUSY_Msk          (0x1ul << EADC_STATUS_ADC1BUSY_Pos)               /*!< EADC_T::STATUS: ADC1BUSY Mask           */

#define EADC_STATUS_ADC1CH_Pos            (12)                                              /*!< EADC_T::STATUS: ADC1CH Position         */
#define EADC_STATUS_ADC1CH_Msk            (0xful << EADC_STATUS_ADC1CH_Pos)                 /*!< EADC_T::STATUS: ADC1CH Mask             */

#define EADC_STATUS_WCMPIF_Pos            (16)                                              /*!< EADC_T::STATUS: WCMPIF Position         */
#define EADC_STATUS_WCMPIF_Msk            (0x1ul << EADC_STATUS_WCMPIF_Pos)                 /*!< EADC_T::STATUS: WCMPIF Mask             */

#define EADC_STATUS_LOWFG_Pos             (17)                                              /*!< EADC_T::STATUS: LOWFG Position          */
#define EADC_STATUS_LOWFG_Msk             (0x1ul << EADC_STATUS_LOWFG_Pos)                  /*!< EADC_T::STATUS: LOWFG Mask              */

#define EADC_STATUS_MIDFG_Pos             (18)                                              /*!< EADC_T::STATUS: MIDFG Position          */
#define EADC_STATUS_MIDFG_Msk             (0x1ul << EADC_STATUS_MIDFG_Pos)                  /*!< EADC_T::STATUS: MIDFG Mask              */

#define EADC_STATUS_HIGHFG_Pos            (19)                                              /*!< EADC_T::STATUS: HIGHFG Position         */
#define EADC_STATUS_HIGHFG_Msk            (0x1ul << EADC_STATUS_HIGHFG_Pos)                 /*!< EADC_T::STATUS: HIGHFG Mask             */

#define EADC_WCMPCTL_WCMPEN_Pos           (0)                                               /*!< EADC_T::WCMPCTL: WCMPEN Position        */
#define EADC_WCMPCTL_WCMPEN_Msk           (0x1ul << EADC_WCMPCTL_WCMPEN_Pos)                /*!< EADC_T::WCMPCTL: WCMPEN Mask            */

#define EADC_WCMPCTL_WCMPIEN_Pos          (1)                                               /*!< EADC_T::WCMPCTL: WCMPIEN Position       */
#define EADC_WCMPCTL_WCMPIEN_Msk          (0x1ul << EADC_WCMPCTL_WCMPIEN_Pos)               /*!< EADC_T::WCMPCTL: WCMPIEN Mask           */

#define EADC_WCMPCTL_WCMPLOWEN_Pos        (4)                                               /*!< EADC_T::WCMPCTL: WCMPLOWEN Position     */
#define EADC_WCMPCTL_WCMPLOWEN_Msk        (0x1ul << EADC_WCMPCTL_WCMPLOWEN_Pos)             /*!< EADC_T::WCMPCTL: WCMPLOWEN Mask         */

#define EADC_WCMPCTL_WCMPMIDEN_Pos        (5)                                               /*!< EADC_T::WCMPCTL: WCMPMIDEN Position     */
#define EADC_WCMPCTL_WCMPMIDEN_Msk        (0x1ul << EADC_WCMPCTL_WCMPMIDEN_Pos)             /*!< EADC_T::WCMPCTL: WCMPMIDEN Mask         */

#define EADC_WCMPCTL_WCMPHIGHEN_Pos         (6)                                             /*!< EADC_T::WCMPCTL: WCMPHIGHEN Position      */
#define EADC_WCMPCTL_WCMPHIGHEN_Msk         (0x1ul << EADC_WCMPCTL_WCMPHIGHEN_Pos)            /*!< EADC_T::WCMPCTL: WCMPHIGHEN Mask          */

#define EADC_WCMPCTL_WFLAGCTL_Pos          (7)                                               /*!< EADC_T::WCMPCTL: WFLAGCTL Position       */
#define EADC_WCMPCTL_WFLAGCTL_Msk          (0x1ul << EADC_WCMPCTL_WFLAGCTL_Pos)              /*!< EADC_T::WCMPCTL: WFLAGCTL Mask           */

#define EADC_WCMPCTL_WCMPMCNT_Pos         (8)                                                /*!< EADC_T::WCMPCTL: WCMPMCNT Position      */
#define EADC_WCMPCTL_WCMPMCNT_Msk         (0xful << EADC_WCMPCTL_WCMPMCNT_Pos)               /*!< EADC_T::WCMPCTL: WCMPMCNT Mask          */

#define EADC_WCMPDAT_WCMPLOWDAT_Pos        (0)                                               /*!< EADC_T::WCMPDAT: WCMPLOWDAT Position     */
#define EADC_WCMPDAT_WCMPLOWDAT_Msk        (0xffful << EADC_WCMPDAT_WCMPLOWDAT_Pos)          /*!< EADC_T::WCMPDAT: WCMPLOWDAT Mask         */

#define EADC_WCMPDAT_WCMPHIGHDAT_Pos         (16)                                              /*!< EADC_T::WCMPDAT: WCMPHIGHDAT Position      */
#define EADC_WCMPDAT_WCMPHIGHDAT_Msk         (0xffful << EADC_WCMPDAT_WCMPHIGHDAT_Pos)           /*!< EADC_T::WCMPDAT: WCMPHIGHDAT Mask          */

/**@}*/ /* EADC_CONST */
/**@}*/ /* end of EADC register group */


/*---------------------- Flash Memory Controller -------------------------*/
/**
    @addtogroup FMC Flash Memory Controller(FMC)
    Memory Mapped Structure for FMC Controller
@{ */

typedef struct
{


    /**
     * @var FMC_T::ISPCTL
     * Offset: 0x00  ISP Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPEN     |ISP Enable Control (Write Protect)
     * |        |          |Set this bit to enable ISP function.
     * |        |          |0 = ISP function Disabled.
     * |        |          |1 = ISP function Enabled.
     * |[1]     |BS        |Boot Select (Write Protect)
     * |        |          |Set/clear this bit to select next booting from LDROM/APROM, respectively
     * |        |          |This bit also functions as chip booting status flag, which can be used to check where chip booted from
     * |        |          |This bit is initiated with the inversed value of CBS in CONFIG0 after any reset is happened except CPU reset (CPURF is 1) or system reset (SYSRF) is happened.
     * |        |          |0 = Boot from APROM.
     * |        |          |1 = Boot from LDROM.
     * |[2]     |SPUEN     |SPROM Update Enable Control (Write Protect)
     * |        |          |0 = SPROM cannot be updated.
     * |        |          |1 = SPROM can be updated when the MCU runs in APROM.
     * |[3]     |APUEN     |APROM Update Enable Control (Write Protect)
     * |        |          |0 = APROM cannot be updated when chip runs in APROM.
     * |        |          |1 = APROM can be updated when chip runs in APROM.
     * |[4]     |CFGUEN    |CONFIG Update Enable Control (Write Protect)
     * |        |          |Writing this bit to 1 enables software to update CONFIG value by ISP register control procedure regardless of program code is running in APROM or LDROM.
     * |        |          |0 = ISP update User Configuration Disabled.
     * |        |          |1 = ISP update User Configuration Enabled.
     * |[5]     |LDUEN     |LDROM Update Enable Control (Write Protect)
     * |        |          |0 = LDROM cannot be updated.
     * |        |          |1 = LDROM can be updated when the MCU runs in APROM.
     * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself if APUEN is set to 0.
     * |        |          |(2) LDROM writes to itself if LDUEN is set to 0.
     * |        |          |(3) SPROM writes to itself if SPUEN is set to 0.
     * |        |          |(4) CONFIG is erased/programmed if CFGUEN is set to 0.
     * |        |          |(5) Destination address is illegal, such as over an available range.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * @var FMC_T::ISPADDR
     * Offset: 0x04  ISP Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPADR    |ISP Address
     * |        |          |The NuMicroTM Mini57 series supports word program only. ISPADR[1:0] must be kept 00 for ISP operation.
     * @var FMC_T::ISPDAT
     * Offset: 0x08  ISP Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT    |ISP Data
     * |        |          |Write data to this register before ISP program operation.
     * |        |          |Read data from this register after ISP read operation.
     * @var FMC_T::ISPCMD
     * Offset: 0x0C  ISP Command Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |CMD       |ISP Command
     * |        |          |ISP commands are shown below:
     * |        |          |0x00 = Read.
     * |        |          |0x04 = Read Unique ID.
     * |        |          |0x0B = Read Company ID (0xDA).
     * |        |          |0x0D = Read CRC32 Checksum Result After Calculating.
     * |        |          |0x21 = Program.
     * |        |          |0x22 = Page Erase.
     * |        |          |0x2D = Run Memory CRC32 Checksum Calculation.
     * |        |          |0x2E = Set Vector Page Re-Map.
     * @var FMC_T::ISPTRG
     * Offset: 0x10  ISP Trigger Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP Start Trigger (Write Protect)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP operation is progressed.
     * @var FMC_T::DFBA
     * Offset: 0x14  Data Flash Start Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DFBA      |Data Flash Base Address
     * |        |          |This register indicates Data Flash start address. It is a read only register.
     * |        |          |The Data Flash start address is defined by user
     * |        |          |Since on chip flash erase unit is 512 bytes, it is mandatory to keep bit 8-0 as 0.
     * @var FMC_T::ISPSTS
     * Offset: 0x40  ISP Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPBUSY   |ISP Start Trigger (Read Only)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP operation is progressed.
     * |        |          |Note: This bit is the same with FMC_ISPTRG bit 0.
     * |[2:1]   |CBS       |Config Boot Selection (Read Only)
     * |        |          |This is a mirror of CBS in CONFIG0.
     * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself if APUEN is set to 0.
     * |        |          |(2) LDROM writes to itself if LDUEN is set to 0.
     * |        |          |(3) SPROM writes to itself if SPUEN is set to 0.
     * |        |          |(4) CONFIG is erased/programmed if CFGUEN is set to 0.
     * |        |          |(5) Destination address is illegal, such as over an available range.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[20:9]  |VECMAP    |Vector Page Mapping Address (Read Only)
     * |        |          |The current flash address space 0x0000_0000~0x0000_01FF is mapping to address {VECMAP[11:0], 9'h000} ~ {VECMAP[11:0], 9'h1FF}.
     * |[31:29] |SCODE     |Security Code Active Flag
     * |        |          |This bit field set by hardware when detecting SPROM secured code is active at flash initiation, or software writes 1 to this bit to make secured code active; this bit is clear by SPROM page erase operation.
     * |        |          |000 = SPROM0/1/2 secured code are inactive.
     * |        |          |001 = SPROM0 secured code is active.
     * |        |          |010 = SPROM1 secured code is active.
     * |        |          |100 = SPROM2 secured code is active.
     * |        |          |111 = SPROM0/1/2 Secured code are active.
     * @var FMC_T::CRCSEED
     * Offset: 0x50  ISP CRC Seed Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CRCSEED   |CRC Seed Data
     * |        |          |This register was provided to be the initial value for CRC operation.
     * |        |          |Write data to this register before ISP CRC operation.
     * |        |          |Read data from this register after ISP CRC read operation.
     * @var FMC_T::CRCCV
     * Offset: 0x54  ISP CRC Current Value Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |CRCCV     |CRC Current Value
     * |        |          |This register provided current value of CRC during calculation.
     */
    __IO uint32_t ISPCTL;                /*!< [0x0000] ISP Control Register                                             */
    __IO uint32_t ISPADDR;               /*!< [0x0004] ISP Address Register                                             */
    __IO uint32_t ISPDAT;                /*!< [0x0008] ISP Data Register                                                */
    __IO uint32_t ISPCMD;                /*!< [0x000c] ISP Command Register                                             */
    __IO uint32_t ISPTRG;                /*!< [0x0010] ISP Trigger Register                                             */
    __I  uint32_t DFBA;                  /*!< [0x0014] Data Flash Start Address                                         */
    __I  uint32_t RESERVE0[10];
    __IO  uint32_t ISPSTS;                /*!< [0x0040] ISP Status Register                                              */
    __I  uint32_t RESERVE1[3];
    __IO uint32_t CRCSEED;               /*!< [0x0050] ISP CRC Seed Register                                            */
    __I  uint32_t CRCCV;                 /*!< [0x0054] ISP CRC Current Value Register                                   */

} FMC_T;

/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for FMC Controller
@{ */

#define FMC_ISPCTL_ISPEN_Pos             (0)                                               /*!< FMC_T::ISPCTL: ISPEN Position          */
#define FMC_ISPCTL_ISPEN_Msk             (0x1ul << FMC_ISPCTL_ISPEN_Pos)                   /*!< FMC_T::ISPCTL: ISPEN Mask              */

#define FMC_ISPCTL_BS_Pos                (1)                                               /*!< FMC_T::ISPCTL: BS Position             */
#define FMC_ISPCTL_BS_Msk                (0x1ul << FMC_ISPCTL_BS_Pos)                      /*!< FMC_T::ISPCTL: BS Mask                 */

#define FMC_ISPCTL_SPUEN_Pos             (2)                                               /*!< FMC_T::ISPCTL: SPUEN Position          */
#define FMC_ISPCTL_SPUEN_Msk             (0x1ul << FMC_ISPCTL_SPUEN_Pos)                   /*!< FMC_T::ISPCTL: SPUEN Mask              */

#define FMC_ISPCTL_APUEN_Pos             (3)                                               /*!< FMC_T::ISPCTL: APUEN Position          */
#define FMC_ISPCTL_APUEN_Msk             (0x1ul << FMC_ISPCTL_APUEN_Pos)                   /*!< FMC_T::ISPCTL: APUEN Mask              */

#define FMC_ISPCTL_CFGUEN_Pos            (4)                                               /*!< FMC_T::ISPCTL: CFGUEN Position         */
#define FMC_ISPCTL_CFGUEN_Msk            (0x1ul << FMC_ISPCTL_CFGUEN_Pos)                  /*!< FMC_T::ISPCTL: CFGUEN Mask             */

#define FMC_ISPCTL_LDUEN_Pos             (5)                                               /*!< FMC_T::ISPCTL: LDUEN Position          */
#define FMC_ISPCTL_LDUEN_Msk             (0x1ul << FMC_ISPCTL_LDUEN_Pos)                   /*!< FMC_T::ISPCTL: LDUEN Mask              */

#define FMC_ISPCTL_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPCTL: ISPFF Position          */
#define FMC_ISPCTL_ISPFF_Msk             (0x1ul << FMC_ISPCTL_ISPFF_Pos)                   /*!< FMC_T::ISPCTL: ISPFF Mask              */

#define FMC_ISPADDR_ISPADR_Pos           (0)                                               /*!< FMC_T::ISPADDR: ISPADR Position        */
#define FMC_ISPADDR_ISPADR_Msk           (0xfffffffful << FMC_ISPADDR_ISPADR_Pos)          /*!< FMC_T::ISPADDR: ISPADR Mask            */

#define FMC_ISPDAT_ISPDAT_Pos            (0)                                               /*!< FMC_T::ISPDAT: ISPDAT Position         */
#define FMC_ISPDAT_ISPDAT_Msk            (0xfffffffful << FMC_ISPDAT_ISPDAT_Pos)           /*!< FMC_T::ISPDAT: ISPDAT Mask             */

#define FMC_ISPCMD_CMD_Pos               (0)                                               /*!< FMC_T::ISPCMD: CMD Position            */
#define FMC_ISPCMD_CMD_Msk               (0x3ful << FMC_ISPCMD_CMD_Pos)                    /*!< FMC_T::ISPCMD: CMD Mask                */

#define FMC_ISPTRG_ISPGO_Pos             (0)                                               /*!< FMC_T::ISPTRG: ISPGO Position          */
#define FMC_ISPTRG_ISPGO_Msk             (0x1ul << FMC_ISPTRG_ISPGO_Pos)                   /*!< FMC_T::ISPTRG: ISPGO Mask              */

#define FMC_DFBA_DFBA_Pos                (0)                                               /*!< FMC_T::DFBA: DFBA Position             */
#define FMC_DFBA_DFBA_Msk                (0xfffffffful << FMC_DFBA_DFBA_Pos)               /*!< FMC_T::DFBA: DFBA Mask                 */

#define FMC_ISPSTS_ISPBUSY_Pos           (0)                                               /*!< FMC_T::ISPSTS: ISPBUSY Position        */
#define FMC_ISPSTS_ISPBUSY_Msk           (0x1ul << FMC_ISPSTS_ISPBUSY_Pos)                 /*!< FMC_T::ISPSTS: ISPBUSY Mask            */

#define FMC_ISPSTS_CBS_Pos               (1)                                               /*!< FMC_T::ISPSTS: CBS Position            */
#define FMC_ISPSTS_CBS_Msk               (0x3ul << FMC_ISPSTS_CBS_Pos)                     /*!< FMC_T::ISPSTS: CBS Mask                */

#define FMC_ISPSTS_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPSTS: ISPFF Position          */
#define FMC_ISPSTS_ISPFF_Msk             (0x1ul << FMC_ISPSTS_ISPFF_Pos)                   /*!< FMC_T::ISPSTS: ISPFF Mask              */

#define FMC_ISPSTS_VECMAP_Pos            (9)                                               /*!< FMC_T::ISPSTS: VECMAP Position         */
#define FMC_ISPSTS_VECMAP_Msk            (0xffful << FMC_ISPSTS_VECMAP_Pos)                /*!< FMC_T::ISPSTS: VECMAP Mask             */

#define FMC_ISPSTS_SCODE_Pos             (29)                                              /*!< FMC_T::ISPSTS: SCODE Position          */
#define FMC_ISPSTS_SCODE_Msk             (0x7ul << FMC_ISPSTS_SCODE_Pos)                   /*!< FMC_T::ISPSTS: SCODE Mask              */

#define FMC_CRCSEED_CRCSEED_Pos          (0)                                               /*!< FMC_T::CRCSEED: CRCSEED Position       */
#define FMC_CRCSEED_CRCSEED_Msk          (0xfffffffful << FMC_CRCSEED_CRCSEED_Pos)         /*!< FMC_T::CRCSEED: CRCSEED Mask           */

#define FMC_CRCCV_CRCCV_Pos              (0)                                               /*!< FMC_T::CRCCV: CRCCV Position           */
#define FMC_CRCCV_CRCCV_Msk              (0xfffffffful << FMC_CRCCV_CRCCV_Pos)             /*!< FMC_T::CRCCV: CRCCV Mask               */

/**@}*/ /* FMC_CONST */
/**@}*/ /* end of FMC register group */



/*---------------------- Programmable Gain Ampifier -------------------------*/
/**
    @addtogroup PGA Programmable Gain Ampifier(PGA)
    Memory Mapped Structure for PGA Controller
@{ */

typedef struct
{

    /**
     * @var PGA_T::CTL
     * Offset: 0x00  Programmable Gain Amplifier Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PGAEN     |Programmable Gain Amplifier Enable Bit
     * |        |          |0 = Programmable Gain Amplifier Disabled.
     * |        |          |1 = Programmable Gain Amplifier Enabled.
     * |        |          |Note: The PGA output needs to wait stable 20us after PGAEN is first set.
     * |[6:4]   |GAIN      |PGA Gain Selection
     * |        |          |000 = 2.
     * |        |          |001 = 3.
     * |        |          |010 = 4.
     * |        |          |011 = 7.
     * |        |          |100 = 9.
     * |        |          |101 = 11.
     * |        |          |110 = 13.
     * |        |          |111 = 1. (* See Note)
     * |        |          |Note: GAIN=111; PGA uses bypass pathl Software must write PGAEN to disable.
     */
    __IO uint32_t CTL;                  /*!< [0x0000] Programmable Gain Amplifier Control Register             */

} PGA_T;

/**
    @addtogroup PGA_CONST PGA Bit Field Definition
    Constant Definitions for PGA Controller
@{ */

#define PGA_CTL_PGAEN_Pos               (0)
#define PGA_CTL_PGAEN_Msk               (0x1ul << PGA_CTL_PGAEN_Pos)

#define PGA_CTL_GAIN_Pos                (4)
#define PGA_CTL_GAIN_Msk                (0x7ul << PGA_CTL_GAIN_Pos)

/**@}*/ /* PGA_CONST */
/**@}*/ /* end of PGA register group */


/**@}*/ /* end of REGISTER group */



/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup PERIPHERAL_MEM_MAP Peripheral Memory Map
  Memory Mapped Structure for Peripheral
  @{
 */
/* Peripheral and SRAM base address */
#define FLASH_BASE          ((     uint32_t)0x00000000)
#define SRAM_BASE           ((     uint32_t)0x20000000)
#define AHB_BASE            ((     uint32_t)0x50000000)
#define APB1_BASE           ((     uint32_t)0x40000000)
#define APB2_BASE           ((     uint32_t)0x40100000)

/* Peripheral memory map */
#define GCR_BASE             (AHB_BASE       + 0x00000)                  /*!< System Global Controller Base Address            */
#define CLK_BASE             (AHB_BASE       + 0x00200)                  /*!< System Clock Controller Base Address             */
#define INT_BASE             (AHB_BASE       + 0x00380)                  /*!< Interrupt Source Controller Base Address         */
#define GPIO_BASE            (AHB_BASE        + 0x4000)                  /*!< GPIO Base Address                                */
#define PA_BASE              (GPIO_BASE               )                  /*!< GPIO PORTA Base Address                          */
#define PB_BASE              (GPIO_BASE       + 0x0040)                  /*!< GPIO PORTB Base Address                          */
#define PC_BASE              (GPIO_BASE       + 0x0080)                  /*!< GPIO PORTC Base Address                          */
#define PD_BASE              (GPIO_BASE       + 0x00C0)                  /*!< GPIO PORTD Base Address                          */
#define GPIO_DBNCECON_BASE   (GPIO_BASE       + 0x0440)                  /*!< GPIO De-bounce Cycle Control Base Address        */
#define GPIO_PIN_DATA_BASE   (GPIO_BASE       + 0x0800)                  /*!< GPIO Pin Data Input/Output Control Base Address  */
#define FMC_BASE             (AHB_BASE       + 0x0C000)                 /*!< Flash Memory Base Address                         */
#define HDIV_BASE            (AHB_BASE       + 0x14000)                 /*!< Hardware Divider Base Address                     */
#define WDT_BASE             (APB1_BASE      +  0x4000)                 /*!< Watchdog Timer Base Address                       */
#define TIMER0_BASE          (APB1_BASE      + 0x10000)                 /*!< Timer0 Base Address                               */
#define TIMER1_BASE          (APB1_BASE      + 0x10020)                 /*!< Timer1 Base Address                               */
#define TIMER_AC_BASE        (APB1_BASE      + 0x10040)                 /*!< Timer Advance Capture Base Address                */
#define EPWM_BASE            (APB1_BASE      + 0x40000)                 /*!< Enhance PWM Base Address                          */
#define USCI0_BASE           (APB1_BASE      + 0x70000)                 /*!< USCI0 Base Address                                */
#define ACMP_BASE            (APB1_BASE      + 0xD0000)                 /*!< Analog Comparator 0/1 Base Address                */
#define EADC_BASE            (APB1_BASE      + 0xE0000)                 /*!< EADC Base Address                                 */
#define PGA_BASE             (APB1_BASE      + 0xF0000)                 /*!< PGA Base Address                                  */
#define BPWM_BASE            (APB2_BASE      + 0x40000)                 /*!< Basic PWM Base Address                            */
#define USCI1_BASE           (APB2_BASE      + 0x70000)                 /*!< USCI1 Base Address                                */
#define ECAP_BASE            (APB2_BASE      + 0xB0000)                 /*!< Enhanced Input Capture Timer Base Address         */


/*@}*/ /* end of group PERIPHERAL_MEM_MAP */


/******************************************************************************/
/*                         Peripheral Definitions                             */
/******************************************************************************/

/** @addtogroup PERIPHERAL Peripheral Definitions
  The Definitions of Peripheral
  @{
 */
#define SYS                 ((SYS_T *) GCR_BASE)                        /*!< System Global Controller Configuration Struct    */
#define CLK                 ((CLK_T *) CLK_BASE)                        /*!< System Clock Controller Configuration Struct     */
#define SYSINT              ((INT_T *) INT_BASE)                        /*!< Interrupt Source Controller Configuration Struct */
#define PA                  ((GPIO_T *) PA_BASE)                        /*!< GPIO PORTA Configuration Struct                        */
#define PB                  ((GPIO_T *) PB_BASE)                        /*!< GPIO PORTB Configuration Struct                        */
#define PC                  ((GPIO_T *) PC_BASE)                        /*!< GPIO PORTC Configuration Struct                        */
#define PD                  ((GPIO_T *) PD_BASE)                        /*!< GPIO PORTD Configuration Struct                        */
#define GPIO                ((GPIO_DB_T *) GPIO_DBNCECON_BASE)          /*!< Interrupt De-bounce Cycle Control Configuration Struct */
#define GPIO_DATA           ((GPIO_DATA_T *) GPIO_PIN_DATA_BASE)
#define FMC                 ((FMC_T *) FMC_BASE)                        /*!< FMC Configuration Struct                         */
#define HDIV                ((HDIV_T *) HDIV_BASE)                      /*!< HDIV Configuration Struct                        */
#define WDT                 ((WDT_T *) WDT_BASE)                        /*!< Watchdog Timer Configuration Struct              */
#define TIMER0              ((TIMER_T *) TIMER0_BASE)                   /*!< Timer0 Configuration Struct                      */
#define TIMER1              ((TIMER_T *) TIMER1_BASE)                   /*!< Timer1 Configuration Struct                      */
#define TIMERAC             ((TIMER_AC_T *) TIMER_AC_BASE)              /*!< Timer Adavnce Capture Configuration Struct       */
#define EPWM                ((EPWM_T *) EPWM_BASE)                      /*!< EPWM Configuration Struct                        */
#define ACMP                ((ACMP_T *) ACMP_BASE)                      /*!< ACMP Configuration Struct                        */
#define EADC                ((EADC_T *) EADC_BASE)                      /*!< EADC Configuration Struct                        */
#define PGA                 ((PGA_T *)  PGA_BASE)                       /*!< PGA Configuration Struct                         */
#define BPWM                ((BPWM_T *) BPWM_BASE)                      /*!< BPWM Configuration Struct                        */
#define ECAP                ((ECAP_T *) ECAP_BASE)
#define UI2C0                           ((UI2C_T *) USCI0_BASE)                     /*!< UI2C0 Configuration Struct                       */
#define UI2C1                           ((UI2C_T *) USCI1_BASE)                     /*!< UI2C1 Configuration Struct                       */
#define USPI0                           ((USPI_T *) USCI0_BASE)                     /*!< USPI0 Configuration Struct                       */
#define USPI1                           ((USPI_T *) USCI1_BASE)                     /*!< USPI1 Configuration Struct                       */
#define UUART0                      ((UUART_T *) USCI0_BASE)                    /*!< UUART0 Configuration Struct                      */
#define UUART1                      ((UUART_T *) USCI1_BASE)                    /*!< UUART1 Configuration Struct                      */

/*@}*/ /* end of group PERIPHERAL */

//=============================================================================
/** @addtogroup IO_ROUTINE I/O routines
  The Declaration of I/O routines
  @{
 */

#define UNLOCKREG(x)        do{*((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x59;*((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x16;*((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x88;}while(*((__IO uint32_t *)(GCR_BASE + 0x100))==0)
#define LOCKREG(x)          *((__IO uint32_t *)(GCR_BASE + 0x100)) = 0x00

#define REGCOPY(dest, src)  *((uint32_t *)&(dest)) = *((uint32_t *)&(src))
#define CLEAR(dest)         *((uint32_t *)&(dest)) = 0


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

#define ENABLE      1
#define DISABLE     0

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
#include "sys.h"
#include "ACMP.h"
#include "eadc.h"
#include "FMC.h"
#include "gpio.h"
#include "EPWM.h"
#include "BPWM.h"
#include "TIMER.h"
#include "WDT.h"
#include "clk.h"
#include "pga.h"
#include "ecap.h"
#include "usci_uart.h"
#include "hdiv.h"
#include "pga.h"
#include "usci_i2c.h"
#include "usci_spi.h"

#ifdef __cplusplus
}
#endif

#endif // End of __MINI57_H__


/*** (C) COPYRIGHT 2017 Nuvoton Technology Corp. ***/

