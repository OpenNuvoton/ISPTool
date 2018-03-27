/**************************************************************************//**
 * @file     ISD9000.h
 * @version  V1.0
 * $Revision: 1 $
 * $Date: 15/11/13 11:06a $
 * @brief    ISD9000 Series Peripheral Access Layer Header File
 *
 * @note
 *
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

/**
  \mainpage Introduction
  *
  *
  * This user manual describes the usage of ISD9000 Series MCU device driver
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

/**
  * \page PG_DIR Directory Structure
  * Please refer to Readme.pdf under BSP root directory for the BSP directory structure. 
  *
  *
  * \page PG_REV Revision History
  *
  *
  * <b>Revision 3.00.001</b>
  * \li Updated to support new API
*/

#ifndef __ISD9000_H__
#define __ISD9000_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
*/

/** @addtogroup ISD9000_CMSIS Device Definitions for CMSIS
  ISD9000 Interrupt Number Definition and Configurations for CMSIS
  @{
*/

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */

typedef	enum IRQn
{
/******	 Cortex-M0 Processor Exceptions	Numbers	***************************************************/
	NonMaskableInt_IRQn		= -14,		/*!< 2 Non Maskable	Interrupt							  */
	HardFault_IRQn			= -13,		/*!< 3 Cortex-M0 Hard Fault	Interrupt					  */
	SVCall_IRQn				= -5,		/*!< 11	Cortex-M0 SV Call Interrupt						  */
	PendSV_IRQn				= -2,		/*!< 14	Cortex-M0 Pend SV Interrupt						  */
	SysTick_IRQn			= -1,		/*!< 15	Cortex-M0 System Tick Interrupt					  */

/******	 ISD9000 specific Interrupt Numbers **********************************************************/
	WDT_IRQn				= 0,
	DPWM_IRQn               = 1,
	ADC_IRQn				= 2,
	SPIM_IRQn               = 4,
	TMR0_IRQn				= 5,
	TMR1_IRQn				= 6,
	TMR2_IRQn				= 7,
	GPAB_IRQn				= 8,
	SPI0_IRQn				= 9,
	PWM0_IRQn				= 10,
	PDMA_IRQn               = 11,
	TMRF_IRQn				= 12,
	RTC_IRQn				= 13,
	PWRWU_IRQn				= 14,
	PWM1_IRQn               = 15,
	MAC_IRQn                = 16,
	URT0_IRQn               = 17,
	BOD_IRQn                = 18
										/*!< maximum of	32 Interrupts are possible				  */
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
/*@}*/ /* end of group ISD9000_CMSIS */

#include "core_cm0.h"                   /* Cortex-M0 processor and core peripherals               */
#include "system_ISD9000.h"                /* ISD9000 System include file                           */

#if defined ( __CC_ARM   )
#pragma anon_unions
// IAR C compiler detected
#elif  ( defined (__ICCARM__) )
  #define __wfi       __WFI
  #ifndef __STATIC_INLINE
    #define __STATIC_INLINE  static inline
  #endif
/*
Usage of #define
  #define A(x)  T_##x
  #define B(x) #@x
  #define C(x) #x

  A(1)------>T_1
  B(1)------>'1'
  C(1)------>"1"
*/
  #define __quote(n)      #n
  #define __iar_align(n)  __quote(data_alignment=##n)
  #define __align(n)      _Pragma(__iar_align(n))
#elif ( defined(__GNUC__) )
// GNU C compiler detected
  #define __inline      inline
  #define __isb(n)      __ISB()
  #define __wfi         __WFI
  #define __weak	    __attribute__((weak))
  #define __align(n)    __attribute__ ((aligned (n))) 
#endif

  
/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup ISD9000_REGISTER ISD9000 Control Register
  @{
*/
  

/*---------------------- System Manger Controller -------------------------*/
/**
    @addtogroup SYS System Manger Controller(SYS)
    Memory Mapped Structure for SYS Controller
@{ */
 
typedef struct
{
    /**
     * PDID
     * ===================================================================================================
     * Offset: 0x00  Product Identifier Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |IMG2      |Product Identifier
     * |        |          |Data in MAP2 of information block are copied to this register after power on.
     * |        |          |MAP2 is used to store part number defined by Nuvoton..
     */
    __I  uint32_t PDID;                  

    /**
     * RSTSTS
     * ===================================================================================================
     * Offset: 0x04  System Reset Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PORF      |POR Reset Flag
     * |        |          |The POR reset flag is set by the "Reset Signal" from the Power-on Reset (POR) Controller to indicate the previous reset source.
     * |        |          |0 = No reset from POR.
     * |        |          |1 = Power-on Reset (POR) Controller had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0. 
     * |[1]     |PINRF     |nRESET Pin Reset Flag
     * |        |          |The nRESET pin reset flag is set by the "Reset Signal" from the nRESET Pin to indicate the previous reset source.
     * |        |          |0 = No reset from nRESET pin.
     * |        |          |1 = Pin nRESET had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0. 
     * |[2]     |WDTRF     |Reset Source From WDG
     * |        |          |The WDTRF flag is set if pervious reset source originates from the Watch-Dog module.
     * |        |          |0= No reset from Watch-Dog.
     * |        |          |1= The Watch-Dog module issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[3]     |LVRF      |LVR Reset Flag
     * |        |          |The LVR reset flag is set by the "Reset Signal" from the Low Voltage Reset Controller to indicate the previous reset source.
     * |        |          |0 = No reset from LVR.
     * |        |          |1 = LVR controller had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[4]     |BODRF     |BOD Reset Flag
     * |        |          |The BOD reset flag is set by the "Reset Signal" from the Brown Out Reset Controller to indicate the previous reset source.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = BOD controller had issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.	 
     * |[5]     |M0RF      |M0 Reset Flag
     * |        |          |The Cortex M0 reset flag is set by the SYSRESETREQ from the Cortex M0 to indicate the previous reset source.
     * |        |          |0 = No reset from M0 STSRESETREQ.
     * |        |          |1 = M0 STSRESETREQ issued a reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[6]     |PMURSTF   |Reset Source From PMU
     * |        |          |The PMURSTF flag is set by the reset signal from the PMU module to indicate the previous reset source.
     * |        |          |0= No reset from PMU.
     * |        |          |1= The PMU has issued the reset signal to reset the system.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * |[8]     |PINWK     |Wakeup from DPD From PIN
     * |        |          |The device was woken from Deep Power Down by a low transition on the WAKEUP in or RESETn pin.
     * |        |          |0 = No wakeup from PIN.
     * |        |          |1 = The device was issued a wakeup from DPD by a pin trasition.
     * |        |          |Note: Write 1 to this register to clear all wakeup flags
     * |[9]     |TIMERWK   |Wakeup from DPD From TIMER.
     * |        |          |The device was woken from Deep Power Down by count of 10kHz timer.
     * |        |          |0 = No wakeup from TIMER.
     * |        |          |1 = The device was issued a wakeup from DPD by a TIMER event.
     * |[10]    |PORWK     |Wakeup from DPD From POR.
     * |        |          |The device was woken from Deep Power Down by a Power On Reset.
     * |        |          |0 = No wakeup from POR.
     * |        |          |1 = The device was issued a wakeup from DPD by a POR.
     */
    __IO uint32_t RSTSTS;                

    /**
     * IPRST0
     * ===================================================================================================
     * Offset: 0x08  IP Reset Control Resister0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIPRST   |CHIP One Shot Reset
     * |        |          |Set this bit will reset the whole chip, this bit will automatically return to "0" after 2 clock cycles.
     * |        |          |CHIPRST is same as POR reset, all the chip modules are reset and the chip configuration settings from OTP are reloaded.
     * |        |          |0 = Normal.
     * |        |          |1 = Reset CHIP.
     * |[1]     |CPURST    |CPU Kernel One Shot Reset
     * |        |          |Setting this bit will reset the CPU kernel and OTP Memory Controller(OMC), this bit will automatically return to "0" after the 2 clock cycles
     * |        |          |0 = Normal.
     * |        |          |1 = Reset CPU.
     */
    __IO uint32_t IPRST0;                

    /**
     * IPRST1
     * ===================================================================================================
     * Offset: 0x0C  IP Reset Control Resister1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |GPIORST   |GPIO Controller Reset
     * |        |          |0 = Normal operation.
     * |        |          |1 = Reset.
     * |[2]     |TMR0RST   |Timer0 Controller Reset
     * |        |          |0 = Normal Operation.
     * |        |          |1 = Reset.
     * |[3]     |TMR1RST   |Timer1 Controller Reset
     * |        |          |0 = Normal Operation.
     * |        |          |1 = Reset.
     * |[4]     |TMR2RST   |Timer2 Controller Reset
     * |        |          |0 = Normal operation.
     * |        |          |1 = Reset.
     * |[6]     |TMRFRST   |TimerF Controller Reset
     * |        |          |0 = Normal operation.
     * |        |          |1 = Reset.
     * |[7]     |PDMARST   |PDMA Controller Reset
     * |        |          |0 = Normal operation.
     * |        |          |1 = Reset.
     * |[12]    |SPI0RST   |SPI0 Controller Reset
     * |        |          |0 = Normal Operation.
     * |        |          |1 = Reset.
     * |[13]    |SPIMRST   |SPIM Controller Reset
     * |        |          |0 = Normal Operation.
     * |        |          |1 = Reset.
     * |[20]    |PWM0RST   |PWM0 Controller Reset
     * |        |          |0 = Normal Operation.
     * |        |          |1 = Reset.
     * |[21]    |PWM1RST   |PWM1 Controller Reset
     * |        |          |0 = Normal Operation.
     * |        |          |1 = Reset.
     * |[28]    |ADCRST    |ADC Controller Reset
     * |        |          |0 = Normal Operation.
     * |        |          |1 = Reset.
     * |[29]    |DPWMRST   |DPWM Controller Reset
     * |        |          |0 = Normal Operation.
     * |        |          |1 = Reset.
     */
    __IO uint32_t IPRST1;                
         
		 uint32_t RESERVE0[2]; // 0x10, 0x14

    /**
     * BODCTL
     * ===================================================================================================
     * Offset: 0x18  Brown-Out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BOD_EN    |Brown-Out Detector Threshold Voltage Selection Extension (Initialized & Protected Bit)
	 * |        |          |The default value is set by flash controller as inverse of user configuration CBODEN bit (config0 [20]).
     * |        |          |0 = Brown-Out Detector function is disabled.
     * |        |          |1 = Brown-Out Detector function is enabled.
     * |[1]     |BOD_RSTEN |Brown-Out Detector Reset or Interrupt Bit (Initialized & Protected Bit)
	 * |        |          |The default value is set by flash controller as inverse of user configuration CBORST bit (config0 [21]).
     * |        |          |0 = Brown-Out Detector generate an interrupt.
     * |        |          |1 = Brown-Out Detector will reset chip.
     * |[5:2]   |BOD_LVL   |Brown-Out Detector Threshold Voltage Selection (Initialized & Protected Bit).
	 * |        |          |The default value is set by flash controller user configuration CBOV bit (config0 [25:22]).
     * |        |          |0 = Threshold voltage is 1.8V.
     * |        |          |1 = Threshold voltage is 1.9V.
     * |        |          |2 = Threshold voltage is 2.0V.
     * |        |          |3 = Threshold voltage is 2.1V.
     * |        |          |4 = Threshold voltage is 2.2V.
     * |        |          |5 = Threshold voltage is 2.4V.
     * |        |          |6 = Threshold voltage is 2.6V.
     * |        |          |7 = Threshold voltage is 2.8V.
     * |[6]     |BOD_HYS   |Brown-Out Detector Hysteresis (Initialized & Protected Bit)
	 * |        |          |The default value is set by flash controller user configuration CBOV[4] bit (config0 [26]).
     * |        |          |0 = No hysteresis on BOD detection.
     * |        |          |1 = BOD hysteresis enabled.
     * |[7]     |BOD_OUT   |Brown-Out Detector Output State 
     * |        |          |0 = Brown-out Detector status output is 0, the detected voltage is higher than BOD_VL setting.
     * |        |          |1 = Brown-out Detector status output is 1, the detected voltage is lower than BOD_VL setting.
     * |[8]     |BOD_INT   |Brown-Out Dectector Interrupt.
     * |        |          |1 = Indicates BOD_INT is active. 
     * |        |          |0 = Indicates BOD_INT isn't active.
	 * |        |          |Write 1 to clear.
     * |[16]    |LVR_EN    |Low Voltage Reset (LVR) Enable (Initialized & Protected Bit)
     * |        |          |The LVR function resets the chip when the input power voltage is lower than LVR trip point. 
	 * |        |          |Default value is set by flash controller as inverse of CLVR config0[27].
     * |        |          |0 = Disable LVR function.
     * |        |          |1 = Enable LVR function.
     * |[18:17] |LVR_FILTER|00 = LVR output will be filtered by 1 HCLK.(Default)
     * |        |          |01 = LVR output will be filtered by 2 HCLK
     * |        |          |10 = LVR output will be filtered by 8 HCLK
     * |        |          |11 = LVR output will be filtered by 15 HCLK
     */
    __IO uint32_t BODCTL;                
          
    /**
     * PORCTL
     * ===================================================================================================
     * Offset: 0x1C  Power-On-Reset Controller Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |POROFF    |Power-On Reset Enable Code (Write Protect)
     * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again.
     * |        |          |User can disable internal POR circuit to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field..
     * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including:
     * |        |          |nRESET, Watchdog, LVR reset, ICE reset command and the software-chip reset function. 
     * |[16]    |POROFFSTS |This bit is status bit of POR, it is read only.
     * |        |          |0 = POR is active now.
     * |        |          |1 = POR is non-active now while POROFF equals 0x5aa5
 */
    __IO uint32_t PORCTL;           
	
         uint32_t RESERVE1[4];

    /**
     * GPA_MFP
     * ===================================================================================================
     * Offset: 0x30  GPIO PA Multiple Alternate Functions and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PA0MFP    |0 = The GPIOA-0 is selected to the pin PA.0.
     * |        |          |1 = SPI0 2nd chip select output.
     * |[1]     |PA1MFP    |0 = The GPIOA-1 is selected to the pin PA.1.
     * |        |          |1 = SPI0 1st chip select output.
     * |[2]     |PA2MFP    |0 = The GPIOA-2 is selected to the pin PA.2.
     * |        |          |1 = SPI0 clock output.
     * |[3]     |PA3MFP    |0 = The GPIOA-3 is selected to the pin PA.3.
     * |        |          |1 = SPI0 data input.
     * |[4]     |PA4MFP    |0 = The GPIOA-4 is selected to the pin PA.4.
     * |        |          |1 = SPI0 data output.
     * |[5]     |PA5MFP    |0 = The GPIOA-5 is selected to the pin PA.5.
     * |        |          |1 = Timer0 counter external input.
     * |[10]    |PA10MFP   |0 = The GPIOA-10 is selected to the pin PA.10.
     * |        |          |1 = ADC input channel 2.
     * |[11]    |PA11MFP   |0 = The GPIOA-11 is selected to the pin PA.11.
     * |        |          |1 = ADC input channel 3.
     * |[12]    |PA12MFP   |0 = The GPIOA-12 is selected to the pin PA.12.
     * |        |          |1 = Mic. IN+ pin to Pre-Amp.
     * |[13]    |PA13MFP   |0 = The GPIOA-13 is selected to the pin PA.13.
     * |        |          |1 = Mic. IN- pin to Pre-Amp.
     * |[14]    |PA14MFP   |0 = The GPIOA-14 is selected to the pin PA.14.
     * |        |          |1 = MIC Bias pin.
     * |[15]    |PA15MFP   |0 = The GPIOA-15 is selected to the pin PA.15.
     * |        |          |1 = PGC Reference Voltage pin.
 */
    __IO uint32_t GPA_MFP;               

    /**
     * GPB_MFP
     * ===================================================================================================
     * Offset: 0x34  GPIO PB Multiple Alternate Functions and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |PB8MFP    |0 = The GPIOB-8 is selected to the pin PB.8.
     * |        |          |1 = PWM output pin 0.
     * |[9]     |PB9MFP    |0 = The GPIOB-9 is selected to the pin PB.9.
     * |        |          |1 = PWM output pin 1.
     * |[10]    |PB10MFP   |0 = The GPIOB-10 is selected to the pin PB.10.
     * |        |          |1 = PWM output pin 2.
     * |[11]    |PB11MFP   |0 = The GPIOB-11 is selected to the pin PB.11.
     * |        |          |1 = PWM output pin 3.
     * |[12]    |PB12MFP   |0 = The GPIOB-12 is selected to the pin PB.12.
     * |        |          |1 = PWM timer capture input.
     * |[13]    |PB13MFP   |0 = The GPIOB-13 is selected to the pin PB.13.
     * |        |          |1 = IR carrier output.
     * |[14]    |PB14MFP   |0 = The GPIOB-14 is selected to the pin PB.14.
     * |        |          |1 = Timer1 counter external input.
     * |[15]    |PB15MFP   |0 = The GPIOB-15 is selected to the pin PB.15.
     * |        |          |1 = BOD active signal output.
 */
    __IO uint32_t GPB_MFP;               

    /**
     * ICE_MFP
     * ===================================================================================================
     * Offset: 0x38  ICE Multi-Function-Pin Controller Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ICE_EN    |This bit will set ICE_CLK & ICE_DAT pins to be serial debug wires or PB.4/5
     * |        |          |0 = ICE_CLK and ICE_DAT will be assigned as PB.4 and PB.5, for general IO purpose
     * |        |          |1 = ICE_CLK and ICE_DAT will be set as ICE CLCOK/ ICE DIO, only for debugging purpose
     */
    __IO uint32_t ICE_MFP;      

         uint32_t RESERVE2; // 0x3c

    /**
	Fix Flag by CYFang
     */	
    __IO uint32_t GPIO_INTP;
	
    /**
	Fix Flag by CYFang
     */	
    __IO uint32_t GPA_PULL;
	
	       uint32_t RESERVE3; // 0x48

    /**
	Fix Flag by CYFang
     */	
    __IO uint32_t GPA_IEN;
	
	       uint32_t RESERVE4; // 0x50

    /**
	Fix Flag by CYFang
     */	
    __IO uint32_t GPB_PULL;

	       uint32_t RESERVE5; // 0x58
 
    /**
	Fix Flag by CYFang
     */	
    __IO uint32_t GPB_IEN;		 

	     uint32_t RESERVE6[36]; // 0x60, 0x64, 0x68, 0x6C, 0x70, 0x74, 0x78, 0x7C,
		                        // 0x80, 0x84, 0x88, 0x8C, 0x90, 0x94, 0x98, 0x9C,
		                        // 0xA0, 0xA4, 0xA8, 0xAC, 0xB0, 0xB4, 0xB8, 0xBC,
		                        // 0xC0, 0xC4, 0xC8, 0xCC, 0xD0, 0xD4, 0xD8, 0xDC,
		                        // 0xE0, 0xE4, 0xE8, 0xEC
							  
    /**
     * IMGMAP1
     * ===================================================================================================
     * Offset: 0xF0  MAP3 Data Image Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |IMG3      |Data Image of MAP3
     * |        |          |Data in MAP3 of information block are copied to this register after power on.
     */
    __I  uint32_t IMGMAP3;  

    /**
     * DEVICEID
     * ===================================================================================================
     * Offset: 0xF4  Device ID Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |DEVICEID  |Device ID Data
     * |        |          |This register provides specific read-only information for the Device ID
     */
    __I  uint32_t DEVICEID;              

    /**
     * IMGMAP1
     * ===================================================================================================
     * Offset: 0xF8  MAP0 Data Image Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |IMG0      |Data Image of MAP0
     * |        |          |Data in MAP0 of information block are copied to this register after power on.
     */
    __I  uint32_t IMGMAP0;   

    /**
     * IMGMAP1
     * ===================================================================================================
     * Offset: 0xFC  MAP1 Data Image Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |IMG1      |Data Image of MAP1
     * |        |          |Data in MAP1 of information block are copied to this register after power on.
     */
    __I  uint32_t IMGMAP1;               

    /**
     * REGLCTL
     * ===================================================================================================
     * Offset: 0x100  Register Lock Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |REGLCTL   |Protected Register Lock/Unlock Index (Read Only)
     * |        |          |0 = Protected registers are locked. Any write to the target register is ignored.
     * |        |          |1 = Protected registers are unlocked.
     * |        |          |The protected registers are:
     * |        |          |SYS_IPRST0 - address 0x5000_0008
     * |        |          |SYS_BODCTL - address 0x5000_0018
     * |        |          |SYS_PORCTL - address 0x5000_001C
     * |        |          |SYS_ICE_MFP - address 0x5000_0040
     * |        |          |PA_ADJ - address 0x5000_0120
     * |        |          |CLK_PWRCTL - address 0x5000_0200 (bit[6] is not protected for power wake-up interrupt clear)
     * |        |          |CLK_APBCLK bit[0] - address 0x5000_0208 (bit[0] is watch dog clock enable)
     * |        |          |CLK_CLKSEL0 - address 0x5000_0210 (for HCLK clock source select)
     * |        |          |CLK_CLKSEL1 - address 0x5000_0214 (bit[1:0] is watch dog clock selection)
     * |        |          |NMI_SEL[7]= IRQ_TM - address 0x5000_0380 bit[7].
     * |        |          |SPI0_RCLK - address 0x4003_0030
     * |        |          |WDT_CTL -- address 0x4000_4000
     * |[7:0]   |SYS_REGLCTL|Register Lock Control Code (Write Only)
     * |        |          |Some registers have write-protection function.
     * |        |          |Writing these registers have to disable the protected function by writing the sequence value "59h", "16h", "88h" to this field.
     * |        |          |After this sequence is completed, the REGLCTL bit will be set to 1 and write-protection registers can be normal write.
 */
    __IO  uint32_t REGLCTL;               
         
		 uint32_t RESERVE7[3]; // 0x104, 0x108, 0x10C
		 
    /**
	Fix Flag by CYFang
     */	
    __IO uint32_t OSCTRIM;
	
    /**
	Fix Flag by CYFang
     */	
    __IO uint32_t OSC10K;
	
    /**
	Fix Flag by CYFang
     */	
    __IO uint32_t OSC_TRIM[3];

} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */

#define SYS_PDID_IMG2_Pos                (0)                                               /*!< SYS PDID: IMG2 Position                */
#define SYS_PDID_IMG2_Msk                (0xfffful << SYS_PDID_IMG2_Pos)                   /*!< SYS PDID: IMG2 Mask                    */

#define SYS_RSTSTS_PORF_Pos              (0)                                               /*!< SYS RSTSTS: PORF Position              */
#define SYS_RSTSTS_PORF_Msk              (0x1ul << SYS_RSTSTS_PORF_Pos)                    /*!< SYS RSTSTS: PORF Mask                  */

#define SYS_RSTSTS_PINRF_Pos             (1)                                               /*!< SYS RSTSTS: PINRF Position             */
#define SYS_RSTSTS_PINRF_Msk             (0x1ul << SYS_RSTSTS_PINRF_Pos)                   /*!< SYS RSTSTS: PINRF Mask                 */

#define SYS_RSTSTS_WDTRF_Pos             (2)                                               /*!< SYS RSTSTS: WDTRF Position             */
#define SYS_RSTSTS_WDTRF_Msk             (0x1ul << SYS_RSTSTS_WDTRF_Pos)                   /*!< SYS RSTSTS: WDTRF Mask                 */

#define SYS_RSTSTS_LVRF_Pos              (3)                                               /*!< SYS RSTSTS: LVRF Position              */
#define SYS_RSTSTS_LVRF_Msk              (0x1ul << SYS_RSTSTS_LVRF_Pos)                    /*!< SYS RSTSTS: LVRF Mask                  */

#define SYS_RSTSTS_BODRF_Pos             (4)                                               /*!< SYS RSTSTS: BODRF Position             */
#define SYS_RSTSTS_BODRF_Msk             (0x1ul << SYS_RSTSTS_BODRF_Pos)                   /*!< SYS RSTSTS: BODRF Mask                 */

#define SYS_RSTSTS_M0RF_Pos              (5)                                               /*!< SYS RSTSTS: M0RF Position              */
#define SYS_RSTSTS_M0RF_Msk              (0x1ul << SYS_RSTSTS_M0RF_Pos)                    /*!< SYS RSTSTS: M0RF Mask                  */

#define SYS_RSTSTS_PMURSTF_Pos           (6)                                               /*!< SYS RSTSTS: PMURSTF Position           */
#define SYS_RSTSTS_PMURSTF_Msk           (0x1ul << SYS_RSTSTS_PMURSTF_Pos)                 /*!< SYS RSTSTS: PMURSTF Mask               */

#define SYS_RSTSTS_PINWK_Pos             (8)                                               /*!< SYS RSTSTS: PIN_WK Position            */
#define SYS_RSTSTS_PINWK_Msk             (0x1ul << SYS_RSTSTS_PINWK_Pos)                   /*!< SYS RSTSTS: PIN_WK Mask                */

#define SYS_RSTSTS_TIMERWK_Pos           (9)                                               /*!< SYS RSTSTS: TIMER_WK Position          */
#define SYS_RSTSTS_TIMERWK_Msk           (0x1ul << SYS_RSTSTS_TIMERWK_Pos)                 /*!< SYS RSTSTS: TIMER_WK Mask              */

#define SYS_RSTSTS_PORWK_Pos             (10)                                              /*!< SYS RSTSTS: POR_WK Position            */
#define SYS_RSTSTS_PORWK_Msk             (0x1ul << SYS_RSTSTS_PORWK_Pos)                   /*!< SYS RSTSTS: POR_WK Mask                */

#define SYS_IPRST0_CHIPRST_Pos           (0)                                               /*!< SYS IPRST0: CHIPRST Position           */
#define SYS_IPRST0_CHIPRST_Msk           (0x1ul << SYS_IPRST0_CHIPRST_Pos)                 /*!< SYS IPRST0: CHIPRST Mask               */

#define SYS_IPRST0_CPURST_Pos            (1)                                               /*!< SYS IPRST0: CPURST Position            */
#define SYS_IPRST0_CPURST_Msk            (0x1ul << SYS_IPRST0_CPURST_Pos)                  /*!< SYS IPRST0: CPURST Mask                */

#define SYS_IPRST1_GPIORST_Pos           (1)                                               /*!< SYS IPRST1: GPIORST Position           */
#define SYS_IPRST1_GPIORST_Msk           (0x1ul << SYS_IPRST1_GPIORST_Pos)                 /*!< SYS IPRST1: GPIORST Mask               */

#define SYS_IPRST1_TMR0RST_Pos           (2)                                               /*!< SYS IPRST1: TMR0RST Position           */
#define SYS_IPRST1_TMR0RST_Msk           (0x1ul << SYS_IPRST1_TMR0RST_Pos)                 /*!< SYS IPRST1: TMR0RST Mask               */

#define SYS_IPRST1_TMR1RST_Pos           (3)                                               /*!< SYS IPRST1: TMR1RST Position           */
#define SYS_IPRST1_TMR1RST_Msk           (0x1ul << SYS_IPRST1_TMR1RST_Pos)                 /*!< SYS IPRST1: TMR1RST Mask               */

#define SYS_IPRST1_TMR2RST_Pos           (4)                                               /*!< SYS IPRST1: TMR2RST Position           */
#define SYS_IPRST1_TMR2RST_Msk           (0x1ul << SYS_IPRST1_TMR2RST_Pos)                 /*!< SYS IPRST1: TMR2RST Mask               */

#define SYS_IPRST1_TMRFRST_Pos           (6)                                               /*!< SYS IPRST1: TMRFRST Position           */
#define SYS_IPRST1_TMRFRST_Msk           (0x1ul << SYS_IPRST1_TMRFRST_Pos)                 /*!< SYS IPRST1: TMRFRST Mask               */

#define SYS_IPRST1_PDMARST_Pos           (7)                                               /*!< SYS IPRST1: PDMARST Position           */
#define SYS_IPRST1_PDMARST_Msk           (0x1ul << SYS_IPRST1_PDMARST_Pos)                 /*!< SYS IPRST1: PDMARST Mask               */

#define SYS_IPRST1_SPI0RST_Pos           (12)                                              /*!< SYS IPRST1: SPI0RST Position           */
#define SYS_IPRST1_SPI0RST_Msk           (0x1ul << SYS_IPRST1_SPI0RST_Pos)                 /*!< SYS IPRST1: SPI0RST Mask               */

#define SYS_IPRST1_SPIMRST_Pos           (13)                                              /*!< SYS IPRST1: SPIMRST Position           */
#define SYS_IPRST1_SPIMRST_Msk           (0x1ul << SYS_IPRST1_SPIMRST_Pos)                 /*!< SYS IPRST1: SPIMRST Mask               */

#define SYS_IPRST1_PWM0RST_Pos           (20)                                              /*!< SYS IPRST1: PWM0RST Position           */
#define SYS_IPRST1_PWM0RST_Msk           (0x1ul << SYS_IPRST1_PWM0RST_Pos)                 /*!< SYS IPRST1: PWM0RST Mask               */

#define SYS_IPRST1_PWM1RST_Pos           (21)                                              /*!< SYS IPRST1: PWM1RST Position           */
#define SYS_IPRST1_PWM1RST_Msk           (0x1ul << SYS_IPRST1_PWM1RST_Pos)                 /*!< SYS IPRST1: PWM1RST Mask               */

#define SYS_IPRST1_ADCRST_Pos            (28)                                              /*!< SYS IPRST1: ADCRST Position            */
#define SYS_IPRST1_ADCRST_Msk            (0x1ul << SYS_IPRST1_ADCRST_Pos)                  /*!< SYS IPRST1: ADCRST Mask                */

#define SYS_IPRST1_DPWMRST_Pos           (29)                                              /*!< SYS IPRST1: DPWMRST Position           */
#define SYS_IPRST1_DPWMRST_Msk           (0x1ul << SYS_IPRST1_DPWMRST_Pos)                 /*!< SYS IPRST1: DPWMRST Mask               */

#define SYS_BODCTL_BOD_EN_Pos            (0)                                               /*!< SYS BODCTL: BOD_EN Position            */
#define SYS_BODCTL_BOD_EN_Msk            (0x1ul << SYS_BODCTL_BOD_EN_Pos)                  /*!< SYS BODCTL: BOD_EN Mask                */

#define SYS_BODCTL_BOD_RSTEN_Pos         (1)                                               /*!< SYS BODCTL: BOD_RSTEN Position         */
#define SYS_BODCTL_BOD_RSTEN_Msk         (0x1ul << SYS_BODCTL_BOD_RSTEN_Pos)               /*!< SYS BODCTL: BOD_RSTEN Mask             */

#define SYS_BODCTL_BOD_LVL_Pos           (2)                                               /*!< SYS BODCTL: BOD_LVL Position           */
#define SYS_BODCTL_BOD_LVL_Msk           (0xful << SYS_BODCTL_BOD_LVL_Pos)                 /*!< SYS BODCTL: BOD_LVL Mask               */

#define SYS_BODCTL_BOD_HYS_Pos           (6)                                               /*!< SYS BODCTL: BOD_HYS Position           */
#define SYS_BODCTL_BOD_HYS_Msk           (0x1ul << SYS_BODCTL_BOD_HYS_Pos)                 /*!< SYS BODCTL: BOD_HYS Mask               */

#define SYS_BODCTL_BOD_OUT_Pos           (7)                                               /*!< SYS BODCTL: BOD_OUT Position           */
#define SYS_BODCTL_BOD_OUT_Msk           (0x1ul << SYS_BODCTL_BOD_OUT_Pos)                 /*!< SYS BODCTL: BOD_OUT Mask               */

#define SYS_BODCTL_BOD_INT_Pos           (8)                                               /*!< SYS BODCTL: BOD_INT Position           */
#define SYS_BODCTL_BOD_INT_Msk           (0x1ul << SYS_BODCTL_BOD_INT_Pos)                 /*!< SYS BODCTL: BOD_INT Mask               */

#define SYS_BODCTL_LVR_EN_Pos            (16)                                              /*!< SYS BODCTL: LVR_EN Position            */
#define SYS_BODCTL_LVR_EN_Msk            (0x1ul << SYS_BODCTL_LVR_EN_Pos)                  /*!< SYS BODCTL: LVR_EN Mask                */

#define SYS_BODCTL_LVR_FILTER_Pos        (17)                                              /*!< SYS BODCTL: LVR_FILTER Position        */
#define SYS_BODCTL_LVR_FILTER_Msk        (0x3ul << SYS_BODCTL_LVR_FILTER_Pos)              /*!< SYS BODCTL: LVR_FILTER Mask            */

#define SYS_GPA_MFP_PA0MFP_Pos           (0)                                               /*!< SYS GPA_MFP: PA0MFP Position           */
#define SYS_GPA_MFP_PA0MFP_Msk           (0x3ul << SYS_GPA_MFP_PA0MFP_Pos)                 /*!< SYS GPA_MFP: PA0MFP Mask               */

#define SYS_GPA_MFP_PA1MFP_Pos           (2)                                               /*!< SYS GPA_MFP: PA1MFP Position           */
#define SYS_GPA_MFP_PA1MFP_Msk           (0x3ul << SYS_GPA_MFP_PA1MFP_Pos)                 /*!< SYS GPA_MFP: PA1MFP Mask               */

#define SYS_GPA_MFP_PA2MFP_Pos           (4)                                               /*!< SYS GPA_MFP: PA2MFP Position           */
#define SYS_GPA_MFP_PA2MFP_Msk           (0x3ul << SYS_GPA_MFP_PA2MFP_Pos)                 /*!< SYS GPA_MFP: PA2MFP Mask               */

#define SYS_GPA_MFP_PA3MFP_Pos           (6)                                               /*!< SYS GPA_MFP: PA3MFP Position           */
#define SYS_GPA_MFP_PA3MFP_Msk           (0x3ul << SYS_GPA_MFP_PA3MFP_Pos)                 /*!< SYS GPA_MFP: PA3MFP Mask               */

#define SYS_GPA_MFP_PA4MFP_Pos           (8)                                               /*!< SYS GPA_MFP: PA4MFP Position           */
#define SYS_GPA_MFP_PA4MFP_Msk           (0x3ul << SYS_GPA_MFP_PA4MFP_Pos)                 /*!< SYS GPA_MFP: PA4MFP Mask               */

#define SYS_GPA_MFP_PA5MFP_Pos           (10)                                              /*!< SYS GPA_MFP: PA5MFP Position           */
#define SYS_GPA_MFP_PA5MFP_Msk           (0x3ul << SYS_GPA_MFP_PA5MFP_Pos)                 /*!< SYS GPA_MFP: PA5MFP Mask               */

#define SYS_GPA_MFP_PA6MFP_Pos           (12)                                              /*!< SYS GPA_MFP: PA6MFP Position           */
#define SYS_GPA_MFP_PA6MFP_Msk           (0x3ul << SYS_GPA_MFP_PA6MFP_Pos)                 /*!< SYS GPA_MFP: PA6MFP Mask               */

#define SYS_GPA_MFP_PA7MFP_Pos           (14)                                              /*!< SYS GPA_MFP: PA7MFP Position           */
#define SYS_GPA_MFP_PA7MFP_Msk           (0x3ul << SYS_GPA_MFP_PA7MFP_Pos)                 /*!< SYS GPA_MFP: PA7MFP Mask               */

#define SYS_GPA_MFP_PA8MFP_Pos           (16)                                              /*!< SYS GPA_MFP: PA8MFP Position           */
#define SYS_GPA_MFP_PA8MFP_Msk           (0x3ul << SYS_GPA_MFP_PA8MFP_Pos)                 /*!< SYS GPA_MFP: PA8MFP Mask               */

#define SYS_GPA_MFP_PA9MFP_Pos           (18)                                              /*!< SYS GPA_MFP: PA9MFP Position           */
#define SYS_GPA_MFP_PA9MFP_Msk           (0x3ul << SYS_GPA_MFP_PA9MFP_Pos)                 /*!< SYS GPA_MFP: PA9MFP Mask               */

#define SYS_GPA_MFP_PA10MFP_Pos          (20)                                              /*!< SYS GPA_MFP: PA10MFP Position          */
#define SYS_GPA_MFP_PA10MFP_Msk          (0x3ul << SYS_GPA_MFP_PA10MFP_Pos)                /*!< SYS GPA_MFP: PA10MFP Mask              */

#define SYS_GPA_MFP_PA11MFP_Pos          (22)                                              /*!< SYS GPA_MFP: PA11MFP Position          */
#define SYS_GPA_MFP_PA11MFP_Msk          (0x3ul << SYS_GPA_MFP_PA11MFP_Pos)                /*!< SYS GPA_MFP: PA11MFP Mask              */

#define SYS_GPA_MFP_PA12MFP_Pos          (24)                                              /*!< SYS GPA_MFP: PA12MFP Position          */
#define SYS_GPA_MFP_PA12MFP_Msk          (0x3ul << SYS_GPA_MFP_PA12MFP_Pos)                /*!< SYS GPA_MFP: PA12MFP Mask              */

#define SYS_GPA_MFP_PA13MFP_Pos          (26)                                              /*!< SYS GPA_MFP: PA13MFP Position          */
#define SYS_GPA_MFP_PA13MFP_Msk          (0x3ul << SYS_GPA_MFP_PA13MFP_Pos)                /*!< SYS GPA_MFP: PA13MFP Mask              */

#define SYS_GPA_MFP_PA14MFP_Pos          (28)                                              /*!< SYS GPA_MFP: PA14MFP Position          */
#define SYS_GPA_MFP_PA14MFP_Msk          (0x3ul << SYS_GPA_MFP_PA14MFP_Pos)                /*!< SYS GPA_MFP: PA14MFP Mask              */

#define SYS_GPA_MFP_PA15MFP_Pos          (30)                                              /*!< SYS GPA_MFP: PA15MFP Position          */
#define SYS_GPA_MFP_PA15MFP_Msk          (0x3ul << SYS_GPA_MFP_PA15MFP_Pos)                /*!< SYS GPA_MFP: PA15MFP Mask              */

#define SYS_GPB_MFP_PB0MFP_Pos           (0)                                               /*!< SYS GPB_MFP: PB0MFP Position           */
#define SYS_GPB_MFP_PB0MFP_Msk           (0x3ul << SYS_GPB_MFP_PB0MFP_Pos)                 /*!< SYS GPB_MFP: PB0MFP Mask               */

#define SYS_GPB_MFP_PB1MFP_Pos           (2)                                               /*!< SYS GPB_MFP: PB1MFP Position           */
#define SYS_GPB_MFP_PB1MFP_Msk           (0x3ul << SYS_GPB_MFP_PB1MFP_Pos)                 /*!< SYS GPB_MFP: PB1MFP Mask               */

#define SYS_GPB_MFP_PB2MFP_Pos           (4)                                               /*!< SYS GPB_MFP: PB2MFP Position           */
#define SYS_GPB_MFP_PB2MFP_Msk           (0x3ul << SYS_GPB_MFP_PB2MFP_Pos)                 /*!< SYS GPB_MFP: PB2MFP Mask               */

#define SYS_GPB_MFP_PB3MFP_Pos           (6)                                               /*!< SYS GPB_MFP: PB3MFP Position           */
#define SYS_GPB_MFP_PB3MFP_Msk           (0x3ul << SYS_GPB_MFP_PB3MFP_Pos)                 /*!< SYS GPB_MFP: PB3MFP Mask               */

#define SYS_GPB_MFP_PB4MFP_Pos           (8)                                               /*!< SYS GPB_MFP: PB4MFP Position           */
#define SYS_GPB_MFP_PB4MFP_Msk           (0x3ul << SYS_GPB_MFP_PB4MFP_Pos)                 /*!< SYS GPB_MFP: PB4MFP Mask               */

#define SYS_GPB_MFP_PB5MFP_Pos           (10)                                              /*!< SYS GPB_MFP: PB5MFP Position           */
#define SYS_GPB_MFP_PB5MFP_Msk           (0x3ul << SYS_GPB_MFP_PB5MFP_Pos)                 /*!< SYS GPB_MFP: PB5MFP Mask               */

#define SYS_ICE_MFP_ICE_EN_Pos           (0)                                               /*!< SYS ICE_MFP: ICE_EN Position           */
#define SYS_ICE_MFP_ICE_EN_Msk           (0x1ul << SYS_ICE_MFP_ICE_EN_Pos)                 /*!< SYS ICE_MFP: ICE_EN Mask               */

/* 
Fix Flag by CYFang
*/

#define SYS_IMGMAP3_IMG3_Pos             (0)                                               /*!< SYS IMGMAP3: IMG3 Position             */
#define SYS_IMGMAP3_IMG3_Msk             (0xfffffffful << SYS_IMGMAP3_IMG3_Pos)            /*!< SYS IMGMAP3: IMG3 Mask                 */

#define SYS_DEVICEID_DEVICEID_Pos        (0)                                               /*!< SYS DEVICEID: DEVICEID Position        */
#define SYS_DEVICEID_DEVICEID_Msk        (0xfffful << SYS_DEVICEID_DEVICEID_Pos)           /*!< SYS DEVICEID: DEVICEID Mask            */

#define SYS_IMGMAP0_IMG0_Pos             (0)                                               /*!< SYS IMGMAP0: IMG0 Position             */
#define SYS_IMGMAP0_IMG0_Msk             (0xfffffffful << SYS_IMGMAP0_IMG0_Pos)            /*!< SYS IMGMAP0: IMG0 Mask                 */

#define SYS_IMGMAP1_IMG1_Pos             (0)                                               /*!< SYS IMGMAP1: IMG1 Position             */
#define SYS_IMGMAP1_IMG1_Msk             (0xfffffffful << SYS_IMGMAP1_IMG1_Pos)            /*!< SYS IMGMAP1: IMG1 Mask                 */

#define SYS_REGLCTL_REGLCTL_Pos          (0)                                               /*!< SYS REGLCTL: REGLCTL Position          */
#define SYS_REGLCTL_REGLCTL_Msk          (0x1ul << SYS_REGLCTL_REGLCTL_Pos)                /*!< SYS REGLCTL: REGLCTL Mask              */

#define SYS_REGLCTL_SYS_REGLCTL_Pos      (0)                                               /*!< SYS REGLCTL: SYS_REGLCTL Position      */
#define SYS_REGLCTL_SYS_REGLCTL_Msk      (0xfful << SYS_REGLCTL_SYS_REGLCTL_Pos)           /*!< SYS REGLCTL: SYS_REGLCTL Mask          */

/* 
Fix Flag by CYFang
*/

/**@}*/ /* SYS_CONST */
/**@}*/ /* end of SYS register group */
 
 
typedef struct
{

    /**
     * @var SYSINT_T::IRQSRC[32]
     * Offset: 0x00-0x7C  IRQn(n=0~31) Interrupt Source Identity Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |INT_SRC   |Interrupt Source Identity
     * |        |          |IRQ_SRC[0].0 - WDT INT
     * |        |          |IRQ_SRC[1].0 - DPWM INT
     * |        |          |IRQ_SRC[2].0 - ADC INT
     * |        |          |IRQ_SRC[3].0 - Reserved
     * |        |          |IRQ_SRC[4].0 - SPIM INT
     * |        |          |IRQ_SRC[5].0 - TIMER0 INT
     * |        |          |IRQ_SRC[6].0 - TIMER1 INT
     * |        |          |IRQ_SRC[7].0 - TIMER2 INT
     * |        |          |IRQ_SRC[8].0 - GPA INT
     * |        |          |IRQ_SRC[8].1 - GPB INT
     * |        |          |IRQ_SRC[9].0 - SPI0 INT
     * |        |          |IRQ_SRC[10].0 - PWM0 INT
     * |        |          |IRQ_SRC[11].0 - PDMA INT
     * |        |          |IRQ_SRC[12].0 - TIMERF INT
     * |        |          |IRQ_SRC[13].0 - RTC INT
     * |        |          |IRQ_SRC[14].0 - PWRWU INT
     * |        |          |IRQ_SRC[15].0 - PWM1 INT
     * |        |          |IRQ_SRC[16].0 - MAC INT
     * |        |          |IRQ_SRC[17].0 - UART0 INT
     * |        |          |IRQ_SRC[18].0 - BOD INT
     * @var SYSINT_T::NMISEL
     * Offset: 0x80  NMI Interrupt Source Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |NMI_SEL   |NMI interrupt source selection
     * |        |          |The NMI interrupt to Cortex-M0 can be selected from one of IRQ0~IRQ31 by setting NMI_SEL with IRQ number.
     * |        |          |The default NMI interrupt is assigned as IRQ0 interrupt if NMI is enabled by setting NMI_SEL[8].
     * |[7]     |IRQ_TM    |IRQ Test Mode (Write Protect)
	 * |        |          |This bit is the protected bit. To program this bit needs an open lock sequence, write "59h", "16h", "88h" to register SYS_REGLCTL to un-lock this bit.
     * |        |          |0 = The interrupt register MCU_IRQ operates in normal mode. The MCU_IRQ collects all the interrupts from the peripheral and generates interrupt to MCU.
     * |        |          |1 = All the interrupts from peripheral to MCU are blocked. The peripheral IRQ signals (0-15) are replaced by the value in the MCU_IRQ register.
     * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
     * @var SYSINT_T::MCUIRQ
     * Offset: 0x84  MCU Interrupt Request Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |MCU_IRQ   |MCU IRQ Source Register
     * |        |          |In Normal mode (NMI_SEL register bit [7] = 0) The device collects interrupts from each peripheral and synchronizes them to interrupt the Cortex-M0.
     * |        |          |In Test mode (NMI_SEL register bit [7] = 1), the interrupts from peripherals are blocked, and the interrupts are replaces by MCU_IRQ[31:0]. 
     * |        |          |When the MCU_IRQ[n] is 0:
     * |        |          |Write 0: Has no effect.
     * |        |          |Write 1: Generate an interrupt to Cortex_M0 NVIC[n].
     * |        |          |When the MCU_IRQ[n] is 1 (means an interrupt is assert):
     * |        |          |Write 0: Has no effect.
     * |        |          |Write 1: Clear the interrupt and MCU_IRQ[n].
     */
    __I  uint32_t IRQSRC[32];   /* Offset: 0x00-0x7C  IRQn(n=0~31) Interrupt Source Identity Register               */
    __IO uint32_t NMISEL;       /* Offset: 0x80  NMI Interrupt Source Select Control Register                       */
    __IO uint32_t MCUIRQ;       /* Offset: 0x84  MCU Interrupt Request Source Register                              */
    
} SYSINT_T;

/* INT IRQSRC Bit Field Definitions */
#define INT_IRQSRC_INT_SRC_Pos                  0                                   /*!< SYSINT_T::IRQSRC: INT_SRC Position */
#define INT_IRQSRC_INT_SRC_Msk                  (0xFul << INT_IRQSRC_INT_SRC_Pos)

/* INT NMI_SEL Bit Field Definitions */
#define INT_NMI_SEL_IRQ_TM_Pos                  7                                   /*!< SYSINT_T::NMISEL: IRQ_TM Position */
#define INT_NMI_SEL_IRQ_TM_Msk                  (1ul << INT_NMI_SEL_IRQ_TM_Pos)     /*!< SYSINT_T::NMISEL: IRQ_TM Mask */

#define INT_NMI_SEL_NMI_SEL_Pos                 0                                   /*!< SYSINT_T::NMISEL: NMI_SEL Position */
#define INT_NMI_SEL_NMI_SEL_Msk                 (0x1Ful << INT_NMI_SEL_NMI_SEL_Pos) /*!< SYSINT_T::NMISEL: NMI_SEL Mask */
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
     * PWRCTL
     * ===================================================================================================
     * Offset: 0x00  System Power Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |LXTEN     |External 32.768 kHz Crystal Enable Bit
     * |        |          |0 = disable (default)
     * |        |          |1 = enable
     * |[2]     |HIRCEN    |OSC49M Oscillator Enable Bit
     * |        |          |0 = disable
     * |        |          |1 = enable (default)
     * |[3]     |XTL32K_FILTER|Filter the XTL32K output clock
     * |        |          |0 = Disable, XTL32K output clock without filter.
     * |        |          |1 = Enable, XTL32K output clock will be filtered to avoid glitch occurs
     * |        |          |Note: High level of XTL32K must keep 112 HCLK for recognition valid, when this bit is enabled.
     * |[9]     |STOP      |Stop
     * |        |          |Reserved - do not set to '1'
     * |[10]    |DPDEN     |Deep Power Down (DPD) Bit
     * |        |          |Set to '1' and issue WFI/WFE instruction to enter DPD mode.
     * |[11]    |LIRCEN    |OSC10K Oscillator Enable Bit
     * |        |          |0 = disable
     * |        |          |1 = enable (default)
     * |[13:15] |VSET      |Adjusts the digital supply voltage. 
     * |        |          |Should be left as default 0.
     * |[16]    |WKPINEN   |Wakeup Pin Enabled Control
     * |        |          |Determines whether WAKEUP pin is enabled in DPD mode.
     * |        |          |0 = enabled
     * |        |          |1 = disabled	 
     * |[17]    |LIRCDPDEN |OSC16K Enabled Control
     * |        |          |Determines whether OSC16K is enabled in DPD mode.
     * |        |          |If OSC16K is disabled, device cannot wake from DPD with SELWKTMR delay.
     * |        |          |0 = enabled
     * |        |          |1 = disabled
     * |[18:19] |FLASHPWR  |Determine whether FLASH memory enters deep power down
     * |        |          |FLASH_PWR[0]: 1: flash enters deep powerdown upon DEEP_SLEEP.
     * |        |          |FLASH_PWR[1]: 1: flash enters deep powerdown upon STOP mode.
     * |        |          |If FLASH_PWR is selected for a power state mode, current consumption is reduced, but a 10us wakeup time must be added to the wakeup sequence. 
	 * |        |          |Trade-off is wakeup time for standby power.
     * |[20:23] |SELWKTMR  |Select Wakeup Timer
     * |        |          |SELWKTMR[0] = 1: WAKEUP after 128 OSC16K clocks (12.8 ms)
     * |        |          |SELWKTMR[1] = 1: WAKEUP after 256 OSC16K clocks (25.6 ms)
     * |        |          |SELWKTMR[2] = 1: WAKEUP after 512 OSC16K clocks (51.2 ms)
     * |        |          |SELWKTMR[3] = 1: WAKEUP after 1024 OSC16K clocks (102.4ms)
     * |[24]    |WKPINWKF  |Pin Wakeup Flag
     * |        |          |Read Only.
     * |        |          |This flag indicates that wakeup of device was requested with a high to low transition of the WAKEUP pin.
     * |        |          |Flag is cleared when DPD mode is entered.
     * |[25]    |TMRWKF    |Timer Wakeup Flag
     * |        |          |Read Only.
     * |        |          |This flag indicates that wakeup of device was requested with TIMER count of the 16Khz oscillator.
     * |        |          |Flag is cleared when DPD mode is entered.
     * |[26]    |PORWKF    |POI Wakeup Flag
     * |        |          |Read Only.
     * |        |          |This flag indicates that wakeup of device was requested with a power-on reset.
     * |        |          |Flag is cleared when DPD mode is entered.
     * |[27]    |TRIWK     |1: Tri-state the wakeup pin (GPIOB[4]) in DPD).
	 * |        |          |0: Add pullup to wakeup pin in DPD
     * |[28:31] |WKTMRSTS  |Current Wakeup Timer Setting
     * |        |          |Read-Only.
     * |        |          |Read back of the current WAKEUP timer setting.
     * |        |          |This value is updated with SELWKTMR upon entering DPD mode.
     */
    __IO uint32_t PWRCTL;   
	
    /**
     * AHBCLK
     * ===================================================================================================
     * Offset: 0x04  AHB Device Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PDMACKEN  |PDMA Controller Clock Enable Control
     * |        |          |0 = To disable the PDMA engine clock
     * |        |          |1 = To enable the PDMA engine clock.
     * |[1]     |SPIMCKEN  |SPIM Clock Enable Control
     * |        |          |0 = SPIM engine clock Disabled.
     * |        |          |1 = SPIM engine clock Enabled.     
	 * |[2]     |ISPCKEN   |Flash ISP Controller Clock Enable Control
     * |        |          |0 = To disable the Flash ISP engine clock.
     * |        |          |1 = To enable the Flash ISP engine clock.
     */
    __IO uint32_t AHBCLK;     
	
    /**
     * APBCLK0
     * ===================================================================================================
     * Offset: 0x08  APB Device Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDTCKEN   |Watchdog Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[1]     |RTCCKEN   |Real-Time-Clock APB Interface Clock Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[2]     |TMR0CKEN  |Timer0 Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[3]     |TMR1CKEN  |Timer1 Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[4]     |TMR2CKEN  |Timer2 Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[5]     |TMRFCKEN  |TimerF Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[12]    |SPI0CKEN  |SPI0 Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[20]    |PWM0CKEN  |PWM0 Block Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable     
     * |[21]    |PWM1CKEN  |PWM0 Block Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable  	 
     * |[28]    |ADCCKEN   |Audio Analog-Digital-Converter (ADC) Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable	 
	 * |[29]    |DPWMCKEN  |Differential PWM Speaker Driver Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
	 * |[30]    |DACCKEN   |DAC clock enable control.
     * |        |          |0=Disable
     * |        |          |1=Enable
     */
    __IO uint32_t APBCLK;   

    /**
     * DPDSTATE
     * ===================================================================================================
     * Offset: 0x0C  Deep Power Down State Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |DPDSTSWR  |DPD State Write
     * |        |          |To set the CLK_DPDSTATE register, write value to this register. Data is latched on next DPD event. 
     * |[8:15]  |DPDSTSRD  |DPD State Read Back
     * |        |          |Read back of CLK_DPDSTATE register. This register was preserved from last DPD event . 
 */
    __IO uint32_t DPDSTATE;              

    /**
     * CLKSEL0
     * ===================================================================================================
     * Offset: 0x10  Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:2]   |HCLKSEL   |HCLK Clock Source Select
     * |        |          |Ensure that related clock sources (pre-select and new-select) are enabled before updating register.
     * |        |          |These bits are protected, to write to bits first perform the unlock sequence (see Protected Register Lock Key Register (SYS_REGLCTL))
     * |        |          |000 or 111 = clock source from HIRC.
     * |        |          |001 = clock source from LXT.
     * |        |          |010 = clock source from LIRC.
     * |        |          |Others = reserved
     * |[3:4]   |STCLKSEL  |MCU Cortex_M0 SysTick Clock Source Select
     * |        |          |These bits are protected, to write to bits first perform the unlock sequence (see Protected Register Lock Key Register (SYS_REGLCTL))
     * |        |          |00 = clock source from HIRC
     * |        |          |01 = clock source from LIRC
     * |        |          |10 = clock source from LXT
     * |        |          |11 = clock source from HCLK
     * |        |          |Note that to use STCLKSEL as source of SysTic timer the CLKSRC bit of SYST_CSR must be set to 0.
     * |[6:7]   |OSCFSEL   |Fix Flag by CYFang
     * |[16:18] |FCLK_MUX_STATE|Fix Flag by CYFang
     */
    __IO uint32_t CLKSEL0;  
	
    /**
     * CLKSEL1
     * ===================================================================================================
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDTSEL    |Watchdog Timer Clock Source Selection (Write Protect)
     * |        |          |These bits are protected bits.
     * |        |          |To program these bits needs an open lock sequence, write "59h", "16h", "88h" to SYS_REGLCTL to un-lock these bits.
     * |        |          |Refer to the register SYS_REGLCTL at address SYS_BA+0x100..
     * |        |          |00 = Clock source from HCLK/2048.
     * |        |          |01 = Clock source from XTL_32K.
     * |        |          |10 = Clock source from PLL_FOUT.
     * |        |          |11 = Clock source from RC_23M.
     * |[3:2]   |ADCSEL    |ADC Clock Source Select
     * |        |          |00 = Clock source from PLL_FOUT.
     * |        |          |01 = Clock source from HCLK.
     * |        |          |1x = Clock source from RC_46M.
     * |[7]     |OSCTESTSEL|Fix Flag by CYFang
     * |[10:8]  |TMR0SEL   |Timer0 Clock Source Select
     * |        |          |000 = Clock source from HCLK.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from LIRC.
     * |        |          |011 = Clock source from external trigger.
     * |        |          |1xx = Clock source from HIRC.
     * |[14:12] |TMR1SEL   |Timer1 Clock Source Select
     * |        |          |000 = Clock source from HCLK.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from LIRC.
     * |        |          |011 = Clock source from external trigger.
     * |        |          |1xx = Clock source from HIRC.
     * |[18:16] |TMR2SEL   |Timer2 Clock Source Select
     * |        |          |000 = Clock source from HCLK.
     * |        |          |001 = Clock source from LXT.
     * |        |          |010 = Clock source from LIRC.
     * |        |          |011 = Clock source from external trigger.
     * |        |          |1xx = Clock source from HIRC.
     * |[22:20] |TMRFSEL   |TimerF Clock Source Select
     * |        |          |000 = Clock source from external LXT/32.
     * |        |          |001 = Clock source from external LXT/(4x32).
     * |        |          |010 = Clock source from external LIRC/32.
     * |        |          |011 = Clock source from external LIRC/(4x32).
     * |        |          |110 = Clock source from HIRC/32768.
     * |        |          |111 = Clock source from HIRC/(4x32768).
     * |        |          |Others = Equivalent with "000".
     * |[24]    |RTCSEL    |RTC Clock Source Select.
     * |        |          |0 = Clock source from LIRC.
     * |        |          |1 = Clock source from LXT.
     * |[29:28] |PWM0SEL   |PWM0 Timer Clock Source Select
     * |        |          |00 = Clock source from HCLK.
     * |        |          |01 = Clock source from LXT.
     * |        |          |10 = Clock source from LIRC.
     * |        |          |11 = Clock source from HIRC.
     * |[31:30] |PWM1SEL   |PWM1 Timer Clock Source Select
     * |        |          |00 = Clock source from HCLK.
     * |        |          |01 = Clock source from LXT.
     * |        |          |10 = Clock source from LIRC.
     * |        |          |11 = Clock source from HIRC.
     */
    __IO uint32_t CLKSEL1;  
	
    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x18  Clock Divider Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLKDIV   |HCLK Clock Divide Number From HCLK Clock Source
     * |        |          |The HCLK clock frequency = (HCLK clock source frequency) / (HCLKDIV + 1).
     * |[23:16] |ADCDIV    |ADC Clock Divide Number From ADC Clock Source
     * |        |          |The ADC clock frequency ADCLK = (ADC clock source frequency) / (ADCDIV + 1).
     * |        |          |The ADC engine clock must meet the constraint: ADCLK &pound; HCKL/2.
     */
    __IO uint32_t CLKDIV;   
         uint32_t RESERVE0[2];

    /**
     * PWRSTSF
     * ===================================================================================================
     * Offset: 0x24  Power State Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DSF       |Deep Sleep Flag
     * |        |          |This flag is set if core logic was placed in Deep Sleep mode. Write '1' to clear flag.
     * |[1]     |STOPF     |Stop Flag
     * |        |          |This flag is set if core logic was stopped but not powered down. Write '1' to clear flag.
     * |[2]     |SPDF      |Powered Down Flag
     * |        |          |This flag is set if core logic was powered down to Standby (SPD). Write '1' to clear flag.
 */
    __IO uint32_t PWRSTSF;               
	
 } CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */
 
#define CLK_PWRCTL_LXTEN_Pos             (1)                                               /*!< CLK PWRCTL: LXTEN Position         */
#define CLK_PWRCTL_LXTEN_Msk             (0x1ul << CLK_PWRCTL_LXTEN_Pos)                   /*!< CLK PWRCTL: LXTEN Mask             */

#define CLK_PWRCTL_HIRCEN_Pos            (2)                                               /*!< CLK PWRCTL: HIRCEN Position         */
#define CLK_PWRCTL_HIRCEN_Msk            (0x1ul << CLK_PWRCTL_HIRCEN_Pos)                  /*!< CLK PWRCTL: HIRCEN Mask             */

#define CLK_PWRCTL_XTL32K_FILTER_Pos     (3)                                               /*!< CLK PWRCTL: XTL32K_FILTER Position     */
#define CLK_PWRCTL_XTL32K_FILTER_Msk     (0x1ul << CLK_PWRCTL_XTL32K_FILTER_Pos)           /*!< CLK PWRCTL: XTL32K_FILTER Mask         */

#define CLK_PWRCTL_STOPEN_Pos              (9)                                               /*!< CLK PWRCTL: STOP Position              */
#define CLK_PWRCTL_STOPEN_Msk              (0x1ul << CLK_PWRCTL_STOPEN_Pos)                    /*!< CLK PWRCTL: STOP Mask                  */
 
#define CLK_PWRCTL_DPDEN_Pos             (10)                                              /*!< CLK PWRCTL: DPDEN Position             */
#define CLK_PWRCTL_DPDEN_Msk             (0x1ul << CLK_PWRCTL_DPDEN_Pos)                   /*!< CLK PWRCTL: DPDEN Mask                 */
 
#define CLK_PWRCTL_LIRCEN_Pos            (12)                                              /*!< CLK PWRCTL: LIRCEN Position            */
#define CLK_PWRCTL_LIRCEN_Msk            (0x1ul << CLK_PWRCTL_LIRCEN_Pos)                  /*!< CLK PWRCTL: LIRCEN Mask                */

#define CLK_PWRCTL_VSET_Pos              (13)                                              /*!< CLK PWRCTL: VSET Position            */
#define CLK_PWRCTL_VSET_Msk              (0x7ul << CLK_PWRCTL_VSET_Pos)                    /*!< CLK PWRCTL: VSET Mask                */

#define CLK_PWRCTL_WKPINEN_Pos           (16)                                              /*!< CLK PWRCTL: WKPINEN Position           */
#define CLK_PWRCTL_WKPINEN_Msk           (0x1ul << CLK_PWRCTL_WKPINEN_Pos)                 /*!< CLK PWRCTL: WKPINEN Mask               */
 
#define CLK_PWRCTL_LIRCDPDEN_Pos         (17)                                              /*!< CLK PWRCTL: LIRCDPDEN Position         */
#define CLK_PWRCTL_LIRCDPDEN_Msk         (0x1ul << CLK_PWRCTL_LIRCDPDEN_Pos)               /*!< CLK PWRCTL: LIRCDPDEN Mask             */ 

#define CLK_PWRCTL_FLASHPWR_Pos          (18)                                              /*!< CLK PWRCTL: FLASHPWR Position         */
#define CLK_PWRCTL_FLASHPWR_Msk          (0x3ul << CLK_PWRCTL_FLASHPWR_Pos)                /*!< CLK PWRCTL: FLASHPWR Mask             */ 

#define CLK_PWRCTL_SELWKTMR_Pos          (20)                                              /*!< CLK PWRCTL: SELWKTMR Position          */
#define CLK_PWRCTL_SELWKTMR_Msk          (0xful << CLK_PWRCTL_SELWKTMR_Pos)                /*!< CLK PWRCTL: SELWKTMR Mask              */

#define CLK_PWRCTL_WKPINWKF_Pos          (24)                                              /*!< CLK PWRCTL: WKPINWKF Position          */
#define CLK_PWRCTL_WKPINWKF_Msk          (0x1ul << CLK_PWRCTL_WKPINWKF_Pos)                /*!< CLK PWRCTL: WKPINWKF Mask              */

#define CLK_PWRCTL_TMRWKF_Pos            (25)                                              /*!< CLK PWRCTL: TMRWKF Position            */
#define CLK_PWRCTL_TMRWKF_Msk            (0x1ul << CLK_PWRCTL_TMRWKF_Pos)                  /*!< CLK PWRCTL: TMRWKF Mask                */

#define CLK_PWRCTL_PORWKF_Pos            (26)                                              /*!< CLK PWRCTL: PORWKF Position            */
#define CLK_PWRCTL_PORWKF_Msk            (0x1ul << CLK_PWRCTL_PORWKF_Pos)                  /*!< CLK PWRCTL: PORWKF Mask                */

#define CLK_PWRCTL_TRIWK_Pos             (27)                                              /*!< CLK PWRCTL: TRIWK Position            */
#define CLK_PWRCTL_TRIWK_Msk             (0x1ul << CLK_PWRCTL_TRIWK_Pos)                   /*!< CLK PWRCTL: TRIWK Mask                */

#define CLK_PWRCTL_WKTMRSTS_Pos          (28)                                              /*!< CLK PWRCTL: WKTMRSTS Position          */
#define CLK_PWRCTL_WKTMRSTS_Msk          (0xful << CLK_PWRCTL_WKTMRSTS_Pos)                /*!< CLK PWRCTL: WKTMRSTS Mask              */

#define CLK_AHBCLK_PDMACKEN_Pos          (0)                                               /*!< CLK AHBCLK: PDMACKEN Position          */
#define CLK_AHBCLK_PDMACKEN_Msk          (0x1ul << CLK_AHBCLK_PDMACKEN_Pos)                /*!< CLK AHBCLK: PDMACKEN Mask              */

#define CLK_AHBCLK_SPIMCKEN_Pos          (1)                                               /*!< CLK AHBCLK: SPIMCKEN Position            */
#define CLK_AHBCLK_SPIMCKEN_Msk          (0x1ul << CLK_AHBCLK_SPIMCKEN_Pos)                /*!< CLK AHBCLK: SPIMCKEN Mask                */

#define CLK_AHBCLK_ISPCKEN_Pos           (2)                                               /*!< CLK AHBCLK: ISPCKEN Position           */
#define CLK_AHBCLK_ISPCKEN_Msk           (0x1ul << CLK_AHBCLK_ISPCKEN_Pos)                 /*!< CLK AHBCLK: ISPCKEN Mask               */

#define CLK_AHBCLK_UART0CKEN_Pos         (3)                                               /*!< CLK AHBCLK: UART0CKEN Position           */
#define CLK_AHBCLK_UART0CKEN_Msk         (0x1ul << CLK_AHBCLK_UART0CKEN_Pos)               /*!< CLK AHBCLK: UART0CKEN Mask               */

#define CLK_APBCLK_WDTCKEN_Pos           (0)                                               /*!< CLK APBCLK: WDTCKEN Position          */
#define CLK_APBCLK_WDTCKEN_Msk           (0x1ul << CLK_APBCLK_WDTCKEN_Pos)                 /*!< CLK APBCLK: WDTCKEN Mask              */

#define CLK_APBCLK_RTCCKEN_Pos           (1)                                               /*!< CLK APBCLK: RTCCKEN Position          */
#define CLK_APBCLK_RTCCKEN_Msk           (0x1ul << CLK_APBCLK_RTCCKEN_Pos)                 /*!< CLK APBCLK: RTCCKEN Mask              */

#define CLK_APBCLK_TMR0CKEN_Pos          (2)                                               /*!< CLK APBCLK: TMR0CKEN Position         */
#define CLK_APBCLK_TMR0CKEN_Msk          (0x1ul << CLK_APBCLK_TMR0CKEN_Pos)                /*!< CLK APBCLK: TMR0CKEN Mask             */

#define CLK_APBCLK_TMR1CKEN_Pos          (3)                                               /*!< CLK APBCLK: TMR1CKEN Position         */
#define CLK_APBCLK_TMR1CKEN_Msk          (0x1ul << CLK_APBCLK_TMR1CKEN_Pos)                /*!< CLK APBCLK: TMR1CKEN Mask             */

#define CLK_APBCLK_TMR2CKEN_Pos          (4)                                               /*!< CLK APBCLK: TMR2CKEN Position         */
#define CLK_APBCLK_TMR2CKEN_Msk          (0x1ul << CLK_APBCLK_TMR2CKEN_Pos)                /*!< CLK APBCLK: TMR2CKEN Mask             */

#define CLK_APBCLK_TMRFCKEN_Pos          (5)                                               /*!< CLK APBCLK: TMRFCKEN Position         */
#define CLK_APBCLK_TMRFCKEN_Msk          (0x1ul << CLK_APBCLK_TMRFCKEN_Pos)                /*!< CLK APBCLK: TMRFCKEN Mask             */

#define CLK_APBCLK_SPI0CKEN_Pos          (12)                                              /*!< CLK APBCLK: SPI0CKEN Position         */
#define CLK_APBCLK_SPI0CKEN_Msk          (0x1ul << CLK_APBCLK_SPI0CKEN_Pos)                /*!< CLK APBCLK: SPI0CKEN Mask             */

#define CLK_APBCLK_PWM0CKEN_Pos          (20)                                              /*!< CLK APBCLK: PWM0CKEN Position         */
#define CLK_APBCLK_PWM0CKEN_Msk          (0x1ul << CLK_APBCLK_PWM0CKEN_Pos)                /*!< CLK APBCLK: PWM0CKEN Mask             */

#define CLK_APBCLK_PWM1CKEN_Pos          (21)                                              /*!< CLK APBCLK: PWM1CKEN Position         */
#define CLK_APBCLK_PWM1CKEN_Msk          (0x1ul << CLK_APBCLK_PWM1CKEN_Pos)                /*!< CLK APBCLK: PWM1CKEN Mask             */

#define CLK_APBCLK_ADCCKEN_Pos           (28)                                              /*!< CLK APBCLK: ADCCKEN Position          */
#define CLK_APBCLK_ADCCKEN_Msk           (0x1ul << CLK_APBCLK_ADCCKEN_Pos)                 /*!< CLK APBCLK: ADCCKEN Mask              */

#define CLK_APBCLK_DPWMCKEN_Pos          (29)                                              /*!< CLK APBCLK: DPWMCKEN Position         */
#define CLK_APBCLK_DPWMCKEN_Msk          (0x1ul << CLK_APBCLK_DPWMCKEN_Pos)                /*!< CLK APBCLK: DPWMCKEN Mask             */

#define CLK_APBCLK_DACCKEN_Pos           (30)                                              /*!< CLK APBCLK: DACCKEN Position          */
#define CLK_APBCLK_DACCKEN_Msk           (0x1ul << CLK_APBCLK_DACCKEN_Pos)                 /*!< CLK APBCLK: DACCKEN Mask              */

#define CLK_DPDSTATE_DPDSTSWR_Pos        (0)                                               /*!< CLK DPDSTATE: DPDSTSWR Position        */
#define CLK_DPDSTATE_DPDSTSWR_Msk        (0xfful << CLK_DPDSTATE_DPDSTSWR_Pos)             /*!< CLK DPDSTATE: DPDSTSWR Mask            */

#define CLK_DPDSTATE_DPDSTSRD_Pos        (8)                                               /*!< CLK DPDSTATE: DPDSTSRD Position        */
#define CLK_DPDSTATE_DPDSTSRD_Msk        (0xfful << CLK_DPDSTATE_DPDSTSRD_Pos)             /*!< CLK DPDSTATE: DPDSTSRD Mask            */

#define CLK_DPDSTATE_LDOVOLT_Pos         (24)                                               /*!< CLK DPDSTATE: DPDSTSWR Position        */
#define CLK_DPDSTATE_LDOVOLT_Msk         (0x7ul << CLK_DPDSTATE_LDOVOLT_Pos)               /*!< CLK DPDSTATE: DPDSTSWR Mask            */

#define CLK_DPDSTATE_LDOPD_Pos           (31)                                               /*!< CLK DPDSTATE: DPDSTSWR Position        */
#define CLK_DPDSTATE_LDOPD_Msk           (0x1ul << CLK_DPDSTATE_LDOPD_Pos)                  /*!< CLK DPDSTATE: DPDSTSWR Mask            */

#define CLK_CLKSEL0_HCLKSEL_Pos          (0)                                               /*!< CLK CLKSEL0: HCLKSEL Position          */
#define CLK_CLKSEL0_HCLKSEL_Msk          (0x7ul << CLK_CLKSEL0_HCLKSEL_Pos)                /*!< CLK CLKSEL0: HCLKSEL Mask              */

#define CLK_CLKSEL0_STCLKSEL_Pos         (3)                                               /*!< CLK CLKSEL0: STCLKSEL Position         */
#define CLK_CLKSEL0_STCLKSEL_Msk         (0x3ul << CLK_CLKSEL0_STCLKSEL_Pos)               /*!< CLK CLKSEL0: STCLKSEL Mask             */

#define CLK_CLKSEL0_OSCFSEL_Pos          (6)                                               /*!< CLK CLKSEL0: OSCFSEL Position          */
#define CLK_CLKSEL0_OSCFSEL_Msk          (0x3ul << CLK_CLKSEL0_OSCFSEL_Pos)                /*!< CLK CLKSEL0: OSCFSEL Mask              */

#define CLK_CLKSEL0_FCLK_MUX_STATE_Pos   (16)                                              /*!< CLK CLKSEL0: FCLK_MUX_STATE Position   */
#define CLK_CLKSEL0_FCLK_MUX_STATE_Msk   (0x7ul << CLK_CLKSEL0_FCLK_MUX_STATE_Pos)         /*!< CLK CLKSEL0: FCLK_MUX_STATE Mask       */

#define CLK_CLKSEL1_WDTSEL_Pos           (0)                                               /*!< CLK CLKSEL1: WDTSEL Position           */
#define CLK_CLKSEL1_WDTSEL_Msk           (0x3ul << CLK_CLKSEL1_WDTSEL_Pos)                 /*!< CLK CLKSEL1: WDTSEL Mask               */

#define CLK_CLKSEL1_ADCSEL_Pos           (2)                                               /*!< CLK CLKSEL1: ADCSEL Position           */
#define CLK_CLKSEL1_ADCSEL_Msk           (0x3ul << CLK_CLKSEL1_ADCSEL_Pos)                 /*!< CLK CLKSEL1: ADCSEL Mask               */

#define CLK_CLKSEL1_OSCTESTSEL_Pos       (7)                                               /*!< CLK CLKSEL1: OSCTESTSEL Position       */
#define CLK_CLKSEL1_OSCTESTSEL_Msk       (0x1ul << CLK_CLKSEL1_OSCTESTSEL_Pos)             /*!< CLK CLKSEL1: OSCTESTSEL Mask           */

#define CLK_CLKSEL1_TMR0SEL_Pos          (8)                                               /*!< CLK CLKSEL1: TMR0SEL Position          */
#define CLK_CLKSEL1_TMR0SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR0SEL_Pos)                /*!< CLK CLKSEL1: TMR0SEL Mask              */

#define CLK_CLKSEL1_TMR1SEL_Pos          (12)                                              /*!< CLK CLKSEL1: TMR1SEL Position          */
#define CLK_CLKSEL1_TMR1SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR1SEL_Pos)                /*!< CLK CLKSEL1: TMR1SEL Mask              */

#define CLK_CLKSEL1_TMR2SEL_Pos          (16)                                              /*!< CLK CLKSEL1: TMR2SEL Position          */
#define CLK_CLKSEL1_TMR2SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR2SEL_Pos)                /*!< CLK CLKSEL1: TMR2SEL Mask              */

#define CLK_CLKSEL1_TMRFSEL_Pos          (20)                                              /*!< CLK CLKSEL1: TMRFSEL Position          */
#define CLK_CLKSEL1_TMRFSEL_Msk          (0x7ul << CLK_CLKSEL1_TMRFSEL_Pos)                /*!< CLK CLKSEL1: TMRFSEL Mask              */

#define CLK_CLKSEL1_RTCSEL_Pos           (24)                                              /*!< CLK CLKSEL1: RTCSEL Position           */
#define CLK_CLKSEL1_RTCSEL_Msk           (0x1ul << CLK_CLKSEL1_RTCSEL_Pos)                 /*!< CLK CLKSEL1: RTCSEL Mask               */

#define CLK_CLKSEL1_PWM0SEL_Pos          (28)                                              /*!< CLK CLKSEL1: PWM0SEL Position          */
#define CLK_CLKSEL1_PWM0SEL_Msk          (0x3ul << CLK_CLKSEL1_PWM0SEL_Pos)                /*!< CLK CLKSEL1: PWM0SEL Mask              */

#define CLK_CLKSEL1_PWM1SEL_Pos          (30)                                              /*!< CLK CLKSEL1: PWM1SEL Position          */
#define CLK_CLKSEL1_PWM1SEL_Msk          (0x3ul << CLK_CLKSEL1_PWM1SEL_Pos)                /*!< CLK CLKSEL1: PWM1SEL Mask              */

#define CLK_CLKDIV_HCLKDIV_Pos           (0)                                               /*!< CLK CLKDIV: HCLKDIV Position           */
#define CLK_CLKDIV_HCLKDIV_Msk           (0xful << CLK_CLKDIV_HCLKDIV_Pos)                 /*!< CLK CLKDIV: HCLKDIV Mask               */

#define CLK_CLKDIV_ADCDIV_Pos            (16)                                              /*!< CLK CLKDIV: ADCDIV Position            */
#define CLK_CLKDIV_ADCDIV_Msk            (0xfful << CLK_CLKDIV_ADCDIV_Pos)                 /*!< CLK CLKDIV: ADCDIV Mask                */

#define CLK_PWRSTSF_DSF_Pos              (0)                                               /*!< CLK PWRSTSF: DSF Position              */
#define CLK_PWRSTSF_DSF_Msk              (0x1ul << CLK_PWRSTSF_DSF_Pos)                    /*!< CLK PWRSTSF: DSF Mask                  */

#define CLK_PWRSTSF_STOPF_Pos            (1)                                               /*!< CLK PWRSTSF: STOPF Position            */
#define CLK_PWRSTSF_STOPF_Msk            (0x1ul << CLK_PWRSTSF_STOPF_Pos)                  /*!< CLK PWRSTSF: STOPF Mask                */

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */ 
 

/*---------------------- Analog to Digital Converter -------------------------*/
/**
    @addtogroup ADC Analog to Digital Converter(ADC)
    Memory Mapped Structure for ADC Controller
@{ */
 
typedef struct
{
    /**
     * DAT0~7
     * ===================================================================================================
     * Offset: 0x00  A/D Data Register for the channel defined in CHSEQ0~7
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[11:0]  |RESULT    |A/D Conversion Result
     * |        |          |This field contains the 12-bit conversion result. Its data format is defined by ADCFM bit.
     * |[15:12] |EXTS      |Extension Bits Of RESULT for Different Data Format
     * |        |          |If ADCFM is "0", EXTS all are read as "0".
     * |        |          |If ADCFM is "1", EXTS all are read as bit RESULT[11].
     * |[16]    |OV        |Over Run Flag
     * |        |          |0 = Data in RESULT are recent conversion result.
     * |        |          |1 = Data in RESULT are overwritten.
     * |        |          |If converted data in RESULT[11:0] have not been read before new conversion result is loaded to this register, OV is set to 1.
     * |        |          |It is cleared by hardware after ADC_DAT register is read..
     * |[17]    |VALID     |Valid Flag
     * |        |          |0 = Data in RESULT are not valid.
     * |        |          |1 = Data in RESULT are valid.
     * |        |          |This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after ADC_DAT register is read.
     */
    __I  uint32_t DAT[8];                  

    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x20  A/D Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCEN     |A/D Converter Enable
     * |        |          |0 = Disable
     * |        |          |1 = Enable
     * |        |          |Before starting A/D conversion function, this bit should be set to 1.
     * |        |          |Clear it to 0 to disable A/D converter analog circuit power consumption..
     * |[1]     |ADCIE     |A/D Interrupt Enable
     * |        |          |0 = Disable A/D interrupt function
     * |        |          |1 = Enable A/D interrupt function
     * |        |          |A/D conversion end interrupt request is generated if ADCIE bit is set to 1.
     * |[3:2]   |OPMODE    |A/D Converter Operation Mode
     * |        |          |00 = Single conversion
     * |        |          |01 = Reserved
     * |        |          |10 = Single-cycle scan
     * |        |          |11 = Continuous scan
     * |        |          |Note 1: This field will be effective only when DS_EN field in this register is set as "0".
     * |        |          |When DS_EN is set as "1", ADC conversion will be forced to "continuous scan mode"
     * |        |          |Note 2: When changing the operation mode, software should disable SWTRG bit firstly.
     * |[4]     |PDMAEN    |PDMA Transfer Enable Bit 
     * |        |          |When A/D conversion is completed, the converted data is loaded into ADC_DATn (n: 0 ~ 7) register,
     * |        |          |user can enable this bit to generate a PDMA data transfer request. 
     * |        |          |0 = PDMA data transfer Disabled. 
     * |        |          |1 = PDMA data transfer Enabled. 
     * |[11]    |SWTRG     |A/D Conversion Start
     * |        |          |0 = Conversion is stopped and A/D converter enters idle state.
     * |        |          |1 = Start conversion.
     * |        |          |Note: SWTRG bit can be reset to 0 by software, or can be cleared to 0 by hardware automatically at the end of single mode and single-cycle scan mode on specified channel.
     * |        |          |In continuous scan mode, A/D conversion is continuously performed sequentially until software writes 0 to this bit or chip resets..
     * |[12]    |ADCFM     |Data Format Of ADC Conversion Result
     * |        |          |0 = Unsigned
     * |        |          |1 = 2'Complemet
     * |[17:16] |DSRATE    |Down Sample Rate
     * |        |          |00 = Down sample X2
     * |        |          |01 = Down sample X4
     * |        |          |10 = Down sample X8
     * |        |          |11 = Down sample X16
     * |[19]    |DSEN      |Down Sample Function Enable
     * |        |          |0 = Down sample function is disabled
     * |        |          |1 = Down sample function is enabled.
     * |        |          |When this field is set, ADC will be forced to continuous scan mode, no matter what is specified in field OPMODE (ADC_CTL[3:2]).
     * |[22:20] |HPFSEL    |High-pass Filter Frequency Selection:
     * |        |          |000 = Do not remove DC part
     * |        |          |001 = DC part is suppressed by -40dB, -3dB at 0.005 x Sampling Rate
     * |        |          |010 = DC part is suppressed by -40dB, -3dB at 0.010 x Sampling Rate
     * |        |          |011 = DC part is suppressed by -40dB, -3dB at 0.014 x Sampling Rate
     * |        |          |100 = DC part is suppressed by -40dB, -3dB at 0.019 x Sampling Rate
     * |        |          |101 = DC part is suppressed by -40dB, -3dB at 0.023 x Sampling Rate
     * |        |          |110 = DC part is suppressed by -40dB, -3dB at 0.027 x Sampling Rate
     * |        |          |111 = DC part is suppressed by -40dB, -3dB at 0.032 x Sampling Rate
     * |[23]    |HPEN      |High-pass Filter Enable
     * |        |          |0 = High-pass filter is disabled
     * |        |          |1 = High-pass filter is enabled (must in continuous scan mode)
     */
    __IO uint32_t CTL;                   

    /**
     * CHSEQ
     * ===================================================================================================
     * Offset: 0x24  A/D Channel Sequence Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CHSEQ0    |Select Channel N As The 1st Conversion In Scan Sequence
     * |        |          |If CHSEQ0[3]=0, one of the following channel is selected according to CHSEQ0[2:0].
     * |        |          |CHSEQ0
     * |        |          |Selected   channel to ADC input
     * |        |          |0000
     * |        |          |Band   Gap
     * |        |          |0001
     * |        |          |AVDD   (internal)
     * |        |          |0010
     * |        |          |Channel   2
     * |        |          |0011
     * |        |          |Channel   3
     * |        |          |0100
     * |        |          |Reserved
     * |        |          |0101
     * |        |          |GND
     * |        |          |0110
     * |        |          |PA_N
     * |        |          |0111
     * |        |          |PA_P
     * |        |          |If CHSEQ0[3] =1, this is dedicated to select pre-amplifier output as ADC input.
     * |        |          |For microphone application, the audio signal from microphone circuit is connected to the differential input pair of pre-amplifier, so programmer must set CHSEQ0[3]=1 in order to process the audio signal..
     * |        |          | For CHSEQ0[0] =0,
     * |        |          |Only CHSEQ0[2:1].={1,0} is meaningful.
     * |        |          |The output of pre-amplifier with differential input pair is selected as ADC input..
     * |        |          |CHSEQ0
     * |        |          |Selected   channel to ADC input
     * |        |          |1000
     * |        |          |Reserved
     * |        |          |1010
     * |        |          |Reserved
     * |        |          |1100
     * |        |          |Pre-amplifier   output
     * |        |          |1110
     * |        |          |None
     * |        |          | For CHSEQ0[0].=1,
     * |        |          | CHSEQ0 = 1xx1: No channel is selected, scan sequence is stopped.
     * |[7:4]   |CHSEQ1    |Select Channel N As The 2nd Conversion In Scan Sequence
     * |        |          |The definition of channel selection is the same as CHSEQ0.
     * |[11:8]  |CHSEQ2    |Select Channel N As The 3rd Conversion In Scan Sequence
     * |        |          |The definition of channel selection is the same as CHSEQ0.
     * |[15:12] |CHSEQ3    |Select Channel N As The 4th Conversion In Scan Sequence
     * |        |          |The definition of channel selection is the same as CHSEQ0.
     * |[19:16] |CHSEQ4    |Select Channel N As The 5th Conversion In Scan Sequence
     * |        |          |The definition of channel selection is the same as CHSEQ0.
     * |[23:20] |CHSEQ5    |Select Channel N As The 6th Conversion In Scan Sequence
     * |        |          |The definition of channel selection is the same as CHSEQ0.
     * |[27:24] |CHSEQ6    |Select Channel N As The 7th Conversion In Scan Sequence
     * |        |          |The definition of channel selection is the same as CHSEQ0.
     * |[31:28] |CHSEQ7    |Select Channel N As The 8th Conversion In Scan Sequence
     * |        |          |The definition of channel selection is the same as CHSEQ0.
     */
    __IO uint32_t CHSEQ;                 

    /**
     * CMP0~1
     * ===================================================================================================
     * Offset: 0x28  A/D Compare Register 0~1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCMPEN   |Compare Enable
     * |        |          |0 = Disable compare.
     * |        |          |1 = Enable compare.
     * |        |          |Set this bit to 1 to enable the comparison CMPDAT with specified channel conversion result when converted data is loaded into ADC_DAT register.
     * |[1]     |ADCMPIE   |Compare Interrupt Enable
     * |        |          |0 = Disable
     * |        |          |1 = Enable
     * |        |          |When converted data in RESULT is less (or greater) than the compare data CMPDAT, ADCMPF bit is asserted.
     * |        |          |If ADCMPIE is set to 1, a compare interrupt request is generated..
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = ADCMPFx bit is set if conversion result is less than CMPDAT.
     * |        |          |1 = ADCMPFx bit is set if conversion result is greater or equal to CMPDAT,
     * |[5:3]   |CMPCH     |Compare Channel Selection
     * |        |          |000 = Reserved.
     * |        |          |001 = Reserved
     * |        |          |010 = Channel 2 conversion result is selected to be compared.
     * |        |          |011 = Channel 3 conversion result is selected to be compared.
     * |        |          |100 = Reserved.
     * |        |          |101 =The conversion result of pre-amplifier output is selected to be compared.
     * |        |          |110 = Reserved
     * |        |          |111 = Reserved
     * |[11:8]  |CMPMCNT   |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the comparing condition, the internal match counter will increase 1.
     * |        |          |When the internal counter achieves the setting, (CMPMCNT+1) hardware will set the ADCMPF bit..
     * |[27:23] |CMPDAT    |Compare Data
     * |        |          |This field possessing the 5 MSB of 12-bit compare data, and 7 LSB are treated as "0", is used to compare with conversion result of specified channel.
     * |        |          |Software can use it to monitor the external analog input pin voltage transition in scan mode without imposing a load on software..
     * |        |          |The data format should be consistent with the setting of ADCFM bit.
     */
    __IO uint32_t CMP[2];                  

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x30  A/D Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADIF      |A/D Conversion End Flag
     * |        |          |A status flag that indicates the end of A/D conversion.
     * |        |          |ADIF is set to 1 under the following two conditions:
     * |        |          |1. When A/D conversion ends in single mode,
     * |        |          |2. When A/D conversion ends on all channels specified by channel sequence register in scan mode.
     * |        |          |And it is cleared when 1 is written.
     * |[1]     |ADCMPF0   |Compare Flag
     * |        |          |When the selected channel A/D conversion result meets setting conditions in ADC_CMP0, then this bit is set to 1.
     * |        |          |And it is cleared by write 1..
     * |        |          |0 = Converted result RESULT in ADC_DAT does not meet ADC_CMP0 setting.
     * |        |          |1 = Converted result RESULT in ADC_DAT meets ADC_CMP0 setting,
     * |[2]     |ADCMPF1   |Compare Flag
     * |        |          |When the selected channel A/D conversion result meets setting conditions in ADC_CMP1, then this bit is set to 1.
     * |        |          |And it is cleared by write 1..
     * |        |          |0 = Converted result RESULT in ADC_DAT does not meet ADC_CMP1 setting.
     * |        |          |1 = Converted result RESULT in ADC_DAT meets ADC_CMP1 setting,
     * |[3]     |BUSY      |BUSY/IDLE
     * |        |          |0 = A/D converter is in idle state.
     * |        |          |1 = A/D converter is busy at conversion.
     * |        |          |This bit is mirror of SWTRG bit in ADC_CTL.
     * |        |          |It is read only.
     * |[6:4]   |CHANNEL   |Current Conversion Channel
     * |        |          |This filed reflects current conversion channel when BUSY=1.
     * |        |          |When BUSY=0, it shows the next channel will be converted..
     * |        |          |It is read only.
     * |[15:8]  |VALID     |Data Valid Flag
     * |        |          |It is a mirror of VALID bit in ADC_DATn.
     * |[23:16] |OV        |Over Run Flag
     * |        |          |It is a mirror to OV bit in ADC_DATn.
     */
    __IO uint32_t STATUS;       
	
    /**
     * PDMACTL
     * ===================================================================================================
     * Offset: 0x34  ADC PDMA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |RESULT    |ADC PDMA transfer data
     * |        |          |If DW_EN is '0' and HPF_EN is '0', transfer SAR output to SRAM
     * |        |          |If DW_EN is '1' and HPF_EN is '0', transfer DW output to SRAM
     * |        |          |If  HPF_EN is '1', transfer HPF output to SRAM
	 */
    __IO uint32_t PDMACTL;       

         uint32_t RESERVE0[1];

    /**
     * PGCTL
     * ===================================================================================================
     * Offset: 0x3C  ADC Pre-amplifier Gain Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |OPMUTE    |Mute Control of First Stage Pre-Amp for Offset Bias Calibration
     * |        |          |When this bit set is as "1", two input end of first stage pre-amp will be shorted, and feedback resistor of this stage will be shorted.
     * |        |          |0 = Open
     * |        |          |1 = Short
     * |[3]     |GAIN_CHG  |Change Gain method of PGC
     * |        |          |0 = Load Boost & Post gain to PGC directly.
     * |        |          |1 = Load Boost & Post gain to PGC when zero cross occurs.
     * |[9]     |SAR_VREF  |0 = VREF=VDDA
     * |        |          |1 = VREF=MIC_BIAS
     * |[10]    |EN_PGA    |0 = Disable PGA
     * |        |          |1 = Enable PGA
     * |[11]    |PD_IBEN   |1 : Power down analog bias generation
     * |[13:12] |IBGEN_TRIM|Set to 0
     * |        |          |1 = Enable PGA
     * |[16]    |MICB_EN   |0 = Disable MIC_BIAS
     * |        |          |1 = Enable MIC_BIAS
     * |[18:17] |MICB_VSEL |Select MIC BIAS level
     * |        |          |0 = 0% of VCCA
     * |        |          |1 = 65% of VCCA
     * |        |          |2 = 70% of VCCA
     * |        |          |3 = 50% of VCCA
     * |[29:24] |POST_GAIN |Gain setting bits for the second stage of pre-amp. Gain start from 14dB till 34dB for 0.65dB per step.
     * |        |          |00000 = 14 dB
     * |        |          |00001 = 14.65 dB
     * |        |          |00010 = 15.3 dB
     * |        |          |00011 = 15.95 dB
     * |        |          |00100 = 16.6 dB
     * |        |          |00101 = 17.25 dB
     * |        |          |00110 = 17.9 dB
     * |        |          |00111 = 18.55 dB
     * |        |          |01000 = 19.2 dB
     * |        |          |01001 = 19.85 dB
     * |        |          |01010 = 20.5 dB
     * |        |          |01011 = 21.15 dB
     * |        |          |01100 = 21.8 dB
     * |        |          |01101 = 22.45 dB
     * |        |          |01110 = 23.1 dB
     * |        |          |01111 = 23.75 dB
     * |        |          |10000 = 24.4 dB
     * |        |          |11111 = 34.15 dB
     */
    __IO uint32_t PGCTL;  
	
    /**
     * VMID
     * ===================================================================================================
     * Offset: 0x40  ADC VMID Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PULLDWN   |1 = Pull down VMID reference to 0V.
     * |[1]     |PDLOWRES  |0 = Enable low resistance VMID reference
     * |        |          |1 = Disconnect low resistance VMID reference
     * |[2]     |PDHIRES   |0 = Enable high resistance VMID reference
     * |        |          |1 = Disconnect high resistance VMID reference. 
     */
    __IO uint32_t VMID;    

    /**
     * HWPARA
     * ===================================================================================================
     * Offset: 0x44  ADC H/W Parameter Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |SHCLK_N   |Specify the high level of ADC start signal.
     * |        |          |ADC start signal high level duration time = ADC_CLK x (SHCLK_N + 1).
     * |        |          |Note: Suggested and default value is 0.
     * |[14:8]  |CONV_N    |Specify ADC conversion clock number
     * |        |          |ADC Conversion clock number = (CONV_N + 1).
     * |        |          |CONV_N has to be equal to or great than 11.
     * |        |          |To update this field, programmer can only revise bit [14:8] and keep other bits the same as before.
     * |        |          |Note: CONV_N valid range is from 11~127
     */
    __IO uint32_t HWPARA;                

} ADC_T;

/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */

#define ADC_DAT_RESULT_Pos               (0)                                               /*!< ADC DAT: RESULT Position              */
#define ADC_DAT_RESULT_Msk               (0xffful << ADC_DAT_RESULT_Pos)                   /*!< ADC DAT: RESULT Mask                  */

#define ADC_DAT_EXTS_Pos                 (12)                                              /*!< ADC DAT: EXTS Position                */
#define ADC_DAT_EXTS_Msk                 (0xful << ADC_DAT_EXTS_Pos)                       /*!< ADC DAT: EXTS Mask                    */

#define ADC_DAT_OV_Pos                   (16)                                              /*!< ADC DAT: OV Position                  */
#define ADC_DAT_OV_Msk                   (0x1ul << ADC_DAT_OV_Pos)                         /*!< ADC DAT: OV Mask                      */
 
#define ADC_DAT_VALID_Pos                (17)                                              /*!< ADC DAT: VALID Position               */
#define ADC_DAT_VALID_Msk                (0x1ul << ADC_DAT_VALID_Pos)                      /*!< ADC DAT: VALID Mask                   */

#define ADC_CTL_ADCEN_Pos                (0)                                               /*!< ADC CTL: ADCEN Position                */
#define ADC_CTL_ADCEN_Msk                (0x1ul << ADC_CTL_ADCEN_Pos)                      /*!< ADC CTL: ADCEN Mask                    */

#define ADC_CTL_ADCIE_Pos                (1)                                               /*!< ADC CTL: ADCIE Position                */
#define ADC_CTL_ADCIE_Msk                (0x1ul << ADC_CTL_ADCIE_Pos)                      /*!< ADC CTL: ADCIE Mask                    */

#define ADC_CTL_OPMODE_Pos               (2)                                               /*!< ADC CTL: OPMODE Position               */
#define ADC_CTL_OPMODE_Msk               (0x3ul << ADC_CTL_OPMODE_Pos)                     /*!< ADC CTL: OPMODE Mask                   */

#define ADC_CTL_PDMAEN_Pos               (4)                                               /*!< ADC CTL: PDMAEN Position               */
#define ADC_CTL_PDMAEN_Msk               (0x1ul << ADC_CTL_PDMAEN_Pos)                     /*!< ADC CTL: PDMAEN Mask                   */

#define ADC_CTL_SWTRG_Pos                (11)                                              /*!< ADC CTL: SWTRG Position                */
#define ADC_CTL_SWTRG_Msk                (0x1ul << ADC_CTL_SWTRG_Pos)                      /*!< ADC CTL: SWTRG Mask                    */

#define ADC_CTL_ADCFM_Pos                (12)                                              /*!< ADC CTL: ADCFM Position                */
#define ADC_CTL_ADCFM_Msk                (0x1ul << ADC_CTL_ADCFM_Pos)                      /*!< ADC CTL: ADCFM Mask                    */

#define ADC_CTL_DS_RATE_Pos              (16)                                              /*!< ADC CTL: DS_RATE Position              */
#define ADC_CTL_DS_RATE_Msk              (0x3ul << ADC_CTL_DS_RATE_Pos)                    /*!< ADC CTL: DS_RATE Mask                  */

#define ADC_CTL_DS_1CH_Pos               (18)                                              /*!< ADC CTL: DS_1CH Position               */
#define ADC_CTL_DS_1CH_Msk               (0x1ul << ADC_CTL_DS_1CH_Pos)                     /*!< ADC CTL: DS_1CH Mask                   */

#define ADC_CTL_DS_EN_Pos                (19)                                              /*!< ADC CTL: DS_EN Position                */
#define ADC_CTL_DS_EN_Msk                (0x1ul << ADC_CTL_DS_EN_Pos)                      /*!< ADC CTL: DS_EN Mask                    */

#define ADC_CTL_HP_FSEL_Pos              (20)                                              /*!< ADC CTL: HP_FSEL Position              */
#define ADC_CTL_HP_FSEL_Msk              (0x7ul << ADC_CTL_HP_FSEL_Pos)                    /*!< ADC CTL: HP_FSEL Mask                  */

#define ADC_CTL_HP_EN_Pos                (23)                                              /*!< ADC CTL: HP_EN Position                */
#define ADC_CTL_HP_EN_Msk                (0x1ul << ADC_CTL_HP_EN_Pos)                      /*!< ADC CTL: HP_EN Mask                    */

#define ADC_CHSEQ_CHSEQ0_Pos             (0)                                               /*!< ADC CHSEQ: CHSEQ0 Position             */
#define ADC_CHSEQ_CHSEQ0_Msk             (0xful << ADC_CHSEQ_CHSEQ0_Pos)                   /*!< ADC CHSEQ: CHSEQ0 Mask                 */

#define ADC_CHSEQ_CHSEQ1_Pos             (4)                                               /*!< ADC CHSEQ: CHSEQ1 Position             */
#define ADC_CHSEQ_CHSEQ1_Msk             (0xful << ADC_CHSEQ_CHSEQ1_Pos)                   /*!< ADC CHSEQ: CHSEQ1 Mask                 */

#define ADC_CHSEQ_CHSEQ2_Pos             (8)                                               /*!< ADC CHSEQ: CHSEQ2 Position             */
#define ADC_CHSEQ_CHSEQ2_Msk             (0xful << ADC_CHSEQ_CHSEQ2_Pos)                   /*!< ADC CHSEQ: CHSEQ2 Mask                 */

#define ADC_CHSEQ_CHSEQ3_Pos             (12)                                              /*!< ADC CHSEQ: CHSEQ3 Position             */
#define ADC_CHSEQ_CHSEQ3_Msk             (0xful << ADC_CHSEQ_CHSEQ3_Pos)                   /*!< ADC CHSEQ: CHSEQ3 Mask                 */

#define ADC_CHSEQ_CHSEQ4_Pos             (16)                                              /*!< ADC CHSEQ: CHSEQ4 Position             */
#define ADC_CHSEQ_CHSEQ4_Msk             (0xful << ADC_CHSEQ_CHSEQ4_Pos)                   /*!< ADC CHSEQ: CHSEQ4 Mask                 */

#define ADC_CHSEQ_CHSEQ5_Pos             (20)                                              /*!< ADC CHSEQ: CHSEQ5 Position             */
#define ADC_CHSEQ_CHSEQ5_Msk             (0xful << ADC_CHSEQ_CHSEQ5_Pos)                   /*!< ADC CHSEQ: CHSEQ5 Mask                 */

#define ADC_CHSEQ_CHSEQ6_Pos             (24)                                              /*!< ADC CHSEQ: CHSEQ6 Position             */
#define ADC_CHSEQ_CHSEQ6_Msk             (0xful << ADC_CHSEQ_CHSEQ6_Pos)                   /*!< ADC CHSEQ: CHSEQ6 Mask                 */

#define ADC_CHSEQ_CHSEQ7_Pos             (28)                                              /*!< ADC CHSEQ: CHSEQ7 Position             */
#define ADC_CHSEQ_CHSEQ7_Msk             (0xful << ADC_CHSEQ_CHSEQ7_Pos)                   /*!< ADC CHSEQ: CHSEQ7 Mask                 */

#define ADC_CMP_ADCMPEN_Pos              (0)                                               /*!< ADC CMP: ADCMPEN Position             */
#define ADC_CMP_ADCMPEN_Msk              (0x1ul << ADC_CMP_ADCMPEN_Pos)                    /*!< ADC CMP: ADCMPEN Mask                 */

#define ADC_CMP_ADCMPIE_Pos              (1)                                               /*!< ADC CMP: ADCMPIE Position             */
#define ADC_CMP_ADCMPIE_Msk              (0x1ul << ADC_CMP_ADCMPIE_Pos)                    /*!< ADC CMP: ADCMPIE Mask                 */

#define ADC_CMP_CMPCOND_Pos              (2)                                               /*!< ADC CMP: CMPCOND Position             */
#define ADC_CMP_CMPCOND_Msk              (0x1ul << ADC_CMP_CMPCOND_Pos)                    /*!< ADC CMP: CMPCOND Mask                 */

#define ADC_CMP_CMPCH_Pos                (3)                                               /*!< ADC CMP: CMPCH Position               */
#define ADC_CMP_CMPCH_Msk                (0x7ul << ADC_CMP_CMPCH_Pos)                      /*!< ADC CMP: CMPCH Mask                   */

#define ADC_CMP_CMPMCNT_Pos              (8)                                               /*!< ADC CMP: CMPMCNT Position             */
#define ADC_CMP_CMPMCNT_Msk              (0xful << ADC_CMP_CMPMCNT_Pos)                    /*!< ADC CMP: CMPMCNT Mask                 */

#define ADC_CMP_CMPDAT_Pos               (23)                                              /*!< ADC CMP: CMPDAT Position              */
#define ADC_CMP_CMPDAT_Msk               (0x1ful << ADC_CMP_CMPDAT_Pos)                    /*!< ADC CMP: CMPDAT Mask                  */

#define ADC_STATUS_ADIF_Pos              (0)                                               /*!< ADC STATUS: ADIF Position              */
#define ADC_STATUS_ADIF_Msk              (0x1ul << ADC_STATUS_ADIF_Pos)                    /*!< ADC STATUS: ADIF Mask                  */

#define ADC_STATUS_ADCMPF0_Pos           (1)                                               /*!< ADC STATUS: ADCMPF0 Position           */
#define ADC_STATUS_ADCMPF0_Msk           (0x1ul << ADC_STATUS_ADCMPF0_Pos)                 /*!< ADC STATUS: ADCMPF0 Mask               */

#define ADC_STATUS_ADCMPF1_Pos           (2)                                               /*!< ADC STATUS: ADCMPF1 Position           */
#define ADC_STATUS_ADCMPF1_Msk           (0x1ul << ADC_STATUS_ADCMPF1_Pos)                 /*!< ADC STATUS: ADCMPF1 Mask               */

#define ADC_STATUS_BUSY_Pos              (3)                                               /*!< ADC STATUS: BUSY Position              */
#define ADC_STATUS_BUSY_Msk              (0x1ul << ADC_STATUS_BUSY_Pos)                    /*!< ADC STATUS: BUSY Mask                  */

#define ADC_STATUS_CHANNEL_Pos           (4)                                               /*!< ADC STATUS: CHANNEL Position           */
#define ADC_STATUS_CHANNEL_Msk           (0x7ul << ADC_STATUS_CHANNEL_Pos)                 /*!< ADC STATUS: CHANNEL Mask               */

#define ADC_STATUS_VALID_Pos             (8)                                               /*!< ADC STATUS: VALID Position             */
#define ADC_STATUS_VALID_Msk             (0xfful << ADC_STATUS_VALID_Pos)                  /*!< ADC STATUS: VALID Mask                 */

#define ADC_STATUS_OV_Pos                (16)                                              /*!< ADC STATUS: OV Position                */
#define ADC_STATUS_OV_Msk                (0xfful << ADC_STATUS_OV_Pos)                     /*!< ADC STATUS: OV Mask                    */

#define ADC_PDMACTL_RESULT_Pos           (0)                                               /*!< ADC PDMACTL: RESULT Position                */
#define ADC_PDMACTL_RESULT_Msk           (0xfffful << ADC_PDMACTL_RESULT_Pos)              /*!< ADC PDMACTL: RESULT Mask                    */

#define ADC_PGCTL_OPMUTE_Pos             (0)                                               /*!< ADC PGCTL: OPMUTE Position             */
#define ADC_PGCTL_OPMUTE_Msk             (0x1ul << ADC_PGCTL_OPMUTE_Pos)                   /*!< ADC PGCTL: OPMUTE Mask                 */

#define ADC_PGCTL_GAIN_CHG_Pos           (3)                                               /*!< ADC PGCTL: GAIN_CHG Position           */
#define ADC_PGCTL_GAIN_CHG_Msk           (0x1ul << ADC_PGCTL_GAIN_CHG_Pos)                 /*!< ADC PGCTL: GAIN_CHG Mask               */

#define ADC_PGCTL_SAR_VREF_Pos           (9)                                               /*!< ADC PGCTL: SAR_VREF Position           */
#define ADC_PGCTL_SAR_VREF_Msk           (0x1ul << ADC_PGCTL_SAR_VREF_Pos)                 /*!< ADC PGCTL: SAR_VREF Mask               */

#define ADC_PGCTL_EN_PGA_Pos             (10)                                              /*!< ADC PGCTL: EN_PGA Position           */
#define ADC_PGCTL_EN_PGA_Msk             (0x1ul << ADC_PGCTL_EN_PGA_Pos)                   /*!< ADC PGCTL: EN_PGA Mask               */

#define ADC_PGCTL_PD_IBEN_Pos            (11)                                              /*!< ADC PGCTL: PD_IBEN Position           */
#define ADC_PGCTL_PD_IBEN_Msk            (0x1ul << ADC_PGCTL_PD_IBEN_Pos)                  /*!< ADC PGCTL: PD_IBEN Mask               */

#define ADC_PGCTL_OS_Pos                 (12)                                              /*!< ADC PGCTL: OS Position           */
#define ADC_PGCTL_OS_Msk                 (0x3ul << ADC_PGCTL_OS_Pos)                       /*!< ADC PGCTL: OS Mask               */

#define ADC_PGCTL_MICB_EN_Pos            (16)                                              /*!< ADC PGCTL: MICB_EN Position           */
#define ADC_PGCTL_MICB_EN_Msk            (0x1ul << ADC_PGCTL_MICB_EN_Pos)                  /*!< ADC PGCTL: MICB_EN Mask               */

#define ADC_PGCTL_MICB_VSEL_Pos          (17)                                              /*!< ADC PGCTL: MICB_VSEL Position           */
#define ADC_PGCTL_MICB_VSEL_Msk          (0x3ul << ADC_PGCTL_MICB_VSEL_Pos)                /*!< ADC PGCTL: MICB_VSEL Mask               */

#define ADC_PGCTL_PGA_SEL_Pos            (24)                                              /*!< ADC PGCTL: PGA_SEL Position          */
#define ADC_PGCTL_PGA_SEL_Msk            (0x3ful << ADC_PGCTL_PGA_SEL_Pos)                 /*!< ADC PGCTL: PGA_SEL Mask              */

#define ADC_VMID_PULLDWN_Pos             (0)                                               /*!< ADC VMID: PULLDWN Position          */
#define ADC_VMID_PULLDWN_Msk             (0x1ul << ADC_VMID_PULLDWN_Pos)                   /*!< ADC VMID: PULLDWN Mask              */

#define ADC_VMID_PDLOWRES_Pos            (1)                                               /*!< ADC VMID: PDLOWRES Position          */
#define ADC_VMID_PDLOWRES_Msk            (0x1ul << ADC_VMID_PDLOWRES_Pos)                  /*!< ADC VMID: PDLOWRES Mask              */

#define ADC_VMID_PDHIRES_Pos             (2)                                               /*!< ADC VMID: PDHIRES Position          */
#define ADC_VMID_PDHIRES_Msk             (0x1ul << ADC_VMID_PDHIRES_Pos)                   /*!< ADC VMID: PDHIRES Mask              */

#define ADC_HWPARA_SHCLK_N_Pos           (0)                                               /*!< ADC HWPARA: SHCLK_N Position           */
#define ADC_HWPARA_SHCLK_N_Msk           (0x3ful << ADC_HWPARA_SHCLK_N_Pos)                /*!< ADC HWPARA: SHCLK_N Mask               */

#define ADC_HWPARA_CONV_N_Pos            (8)                                               /*!< ADC HWPARA: CONV_N Position            */
#define ADC_HWPARA_CONV_N_Msk            (0x7ful << ADC_HWPARA_CONV_N_Pos)                 /*!< ADC HWPARA: CONV_N Mask                */

/**@}*/ /* ADC_CONST */
/**@}*/ /* end of ADC register group */


/*---------------------- Timer Controller -------------------------*/
/**
    @addtogroup TMR Timer Controller(TMR)
    Memory Mapped Structure for TMR Controller
@{ */
 
typedef struct
{
    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  Timer Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |PSC       |Timer Clock Prescaler
     * |        |          |Clock input is divided by (PSC+1) before it is fed to the counter.
     * |        |          |If PSC = 0, then there is no scaling.
     * |[25]    |ACTSTS    |Timer Active Status Bit (Read Only)
     * |        |          |This bit indicates the counter status of Timer.
     * |        |          |0 = Timer is not active.
     * |        |          |1 = Timer is active.
     * |[26]    |RSTCNT    |Counter Reset Bit
     * |        |          |Set this bit will reset the Timer counter, pre-scale and also force CNTEN to 0.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset Timer's pre-scale counter, internal 16-bit up-counter and CNTEN bit.
     * |[27:28] |OPMODE    |Timer Operating Mode
     * |        |          |00 = The Timer is operating in the one-shot mode.
     * |        |          |The associated interrupt signal is generated once (if INTEN is 1) and CNTEN is automatically cleared by hardware.
     * |        |          |01 = The Timer is operating in the periodic mode.
     * |        |          |The associated interrupt signal is generated periodically (if INTEN is 1).
     * |        |          |10 = Reserved.
     * |        |          |11 = The Timer is operating in continuous counting mode.
     * |        |          |The associated interrupt signal is generated when TIMERx_CNT = TIMERx_CMP (if INTEN is 1); however, the 16-bit up-counter counts continuously without reset.
     * |[29]    |INTEN     |Interrupt Enable Bit
     * |        |          |0 = Disable TIMER Interrupt.
     * |        |          |1 = Enable TIMER Interrupt.
     * |        |          |If timer interrupt is enabled, the timer asserts its interrupt signal when the associated count is equal to TIMERx_CMP.
     * |[30]    |CNTEN     |Counter Enable Bit
     * |        |          |0 = Stop/Suspend counting.
     * |        |          |1 = Start counting.
     * |        |          |Note: This bit is auto-cleared by hardware in one-shot mode (OPMODE = 00b) when the associated timer interrupt is generated (INTEN = 1).
     * |[31]    |Reserved_ |Reserved.
     */
    __IO uint32_t CTL;                   

    /**
     * CMP
     * ===================================================================================================
     * Offset: 0x04  Timer Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CMPDAT    |Timer Comparison Value
     * |        |          |CMPDAT is a 16-bit comparison register.
     * |        |          |When the 16-bit up-counter is enabled and its value is equal to CMPDAT value, a Timer Interrupt is requested if the timer interrupt is enabled with TIMERx_CTL.INTEN = 1.
     * |        |          |Note 1: Never set CMPDAT to 0x000 or 0x001. Timer will not function correctly.
     * |        |          |Note 2: No matter CNTEN is 0 or 1, whenever software writes a new value into this register, TIMER will restart counting by using this new value and abort previous count.
     */
    __IO uint32_t CMP;                   

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x08  Timer Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Timer Interrupt Flag (Read Only)
     * |        |          |This bit indicates the interrupt status of Timer.
     * |        |          |TIF bit is set by hardware when the 16-bit counter matches the timer comparison value (CMPDAT).
     * |        |          |It is cleared by writing 1 to itself.
     * |        |          |0 = No effect.
     * |        |          |1 = CNT (TIMERx_CNT[15:0]) value matches the CMPDAT (TIMERx_CMP[15:0]) value. 
     */
    __IO  uint32_t INTSTS;                

    /**
     * CNT
     * ===================================================================================================
     * Offset: 0x0C  Timer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CNT       |Timer Data Register
     * |        |          |User can read this register for the current up-counter value while TIMERx_CTL.CNTEN is set to 1, 
     */
    __I  uint32_t CNT;
} TMR_T;

typedef struct
{
    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x00  TimerF Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TFIF      |TimerF Interrupt Flag
     * |        |          |This bit indicates the interrupt status of TimerF.
     * |        |          |TFIF bit is set by hardware when TimerF time out. It is cleared by writing 1 to this bit.
     * |        |          |0 = It indicates that TimerF does not time out yet.
     * |        |          |1 = It indicates that TimerF time out. The interrupt flag is set if TimerF interrupt was enabled.
     * |[1]     |TFIE      |TimerF Interrupt Enable
     * |        |          |0 = Disable TimerF Interrupt.
     * |        |          |1 = Enable TimerF Interrupt.
     */
    __IO uint32_t INTSTS;
} TMRF_T;

typedef struct
{
    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  IR Carrier Output Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IRCEN     |IR carrier output enable                                                                
     * |        |          |0 = Disable IR carrier output,                                                          
     * |        |          |1 = Enable IR carrier output. Timer1 time out will toggle the output state on IROUT pin.
     * |[1]     |NONCS     |Non-carrier state                    
     * |        |          |0 = IROUT keeps low when IRCEN is 0, .
     * |        |          |1 = IROUT keeps high when IRCEN is 0.
     */
         uint32_t CTL;
} TMRIR_T;

/**
    @addtogroup TMR_CONST TMR Bit Field Definition
    Constant Definitions for TMR Controller
@{ */

#define TMR_CTL_PSC_Pos                  (0)                                               /*!< TMR CTL: PSC Position                  */
#define TMR_CTL_PSC_Msk                  (0xfful << TMR_CTL_PSC_Pos)                       /*!< TMR CTL: PSC Mask                      */

#define TMR_CTL_ACTSTS_Pos               (25)                                              /*!< TMR CTL: ACTSTS Position               */
#define TMR_CTL_ACTSTS_Msk               (0x1ul << TMR_CTL_ACTSTS_Pos)                     /*!< TMR CTL: ACTSTS Mask                   */

#define TMR_CTL_RSTCNT_Pos               (26)                                              /*!< TMR CTL: RSTCNT Position               */
#define TMR_CTL_RSTCNT_Msk               (0x1ul << TMR_CTL_RSTCNT_Pos)                     /*!< TMR CTL: RSTCNT Mask                   */

#define TMR_CTL_OPMODE_Pos               (27)                                              /*!< TMR CTL: OPMODE Position               */
#define TMR_CTL_OPMODE_Msk               (0x3ul << TMR_CTL_OPMODE_Pos)                     /*!< TMR CTL: OPMODE Mask                   */

#define TMR_CTL_INTEN_Pos                (29)                                              /*!< TMR CTL: INTEN Position                */
#define TMR_CTL_INTEN_Msk                (0x1ul << TMR_CTL_INTEN_Pos)                      /*!< TMR CTL: INTEN Mask                    */

#define TMR_CTL_CNTEN_Pos                (30)                                              /*!< TMR CTL: CNTEN Position                */
#define TMR_CTL_CNTEN_Msk                (0x1ul << TMR_CTL_CNTEN_Pos)                      /*!< TMR CTL: CNTEN Mask                    */

#define TMR_CMP_CMPDAT_Pos               (0)                                               /*!< TMR CMP: CMPDAT Position               */
#define TMR_CMP_CMPDAT_Msk               (0xfffful << TMR_CMP_CMPDAT_Pos)                  /*!< TMR CMP: CMPDAT Mask                   */

#define TMR_INTSTS_TIF_Pos               (0)                                               /*!< TMR INTSTS: TIF Position               */
#define TMR_INTSTS_TIF_Msk               (0x1ul << TMR_INTSTS_TIF_Pos)                     /*!< TMR INTSTS: TIF Mask                   */

#define TMR_CNT_CNT_Pos                  (0)                                               /*!< TMR CNT: CNT Position                  */
#define TMR_CNT_CNT_Msk                  (0xfffful << TMR_CNT_CNT_Pos)                     /*!< TMR CNT: CNT Mask                      */

#define TMR_CTL_PSC_Pos                  (0)                                               /*!< TMR CTL: PSC Position                  */
#define TMR_CTL_PSC_Msk                  (0xfful << TMR_CTL_PSC_Pos)                       /*!< TMR CTL: PSC Mask                      */

#define TMR_CTL_ACTSTS_Pos               (25)                                              /*!< TMR CTL: ACTSTS Position               */
#define TMR_CTL_ACTSTS_Msk               (0x1ul << TMR_CTL_ACTSTS_Pos)                     /*!< TMR CTL: ACTSTS Mask                   */

#define TMR_CTL_RSTCNT_Pos               (26)                                              /*!< TMR CTL: RSTCNT Position               */
#define TMR_CTL_RSTCNT_Msk               (0x1ul << TMR_CTL_RSTCNT_Pos)                     /*!< TMR CTL: RSTCNT Mask                   */

#define TMR_CTL_OPMODE_Pos               (27)                                              /*!< TMR CTL: OPMODE Position               */
#define TMR_CTL_OPMODE_Msk               (0x3ul << TMR_CTL_OPMODE_Pos)                     /*!< TMR CTL: OPMODE Mask                   */

#define TMR_CTL_INTEN_Pos                (29)                                              /*!< TMR CTL: INTEN Position                */
#define TMR_CTL_INTEN_Msk                (0x1ul << TMR_CTL_INTEN_Pos)                      /*!< TMR CTL: INTEN Mask                    */

#define TMR_CTL_CNTEN_Pos                (30)                                              /*!< TMR CTL: CNTEN Position                */
#define TMR_CTL_CNTEN_Msk                (0x1ul << TMR_CTL_CNTEN_Pos)                      /*!< TMR CTL: CNTEN Mask                    */

#define TMR_CMP_CMPDAT_Pos               (0)                                               /*!< TMR CMP: CMPDAT Position               */
#define TMR_CMP_CMPDAT_Msk               (0xfffful << TMR_CMP_CMPDAT_Pos)                  /*!< TMR CMP: CMPDAT Mask                   */

#define TMR_INTSTS_TIF_Pos               (0)                                               /*!< TMR INTSTS: TIF Position               */
#define TMR_INTSTS_TIF_Msk               (0x1ul << TMR_INTSTS_TIF_Pos)                     /*!< TMR INTSTS: TIF Mask                   */

#define TMR_CNT_CNT_Pos                  (0)                                               /*!< TMR CNT: CNT Position                  */
#define TMR_CNT_CNT_Msk                  (0xfffful << TMR_CNT_CNT_Pos)                     /*!< TMR CNT: CNT Mask                      */

#define TMR_INTSTS_TFIF_Pos              (0)                                               /*!< TMR INTSTS: TFIF Position              */
#define TMR_INTSTS_TFIF_Msk              (0x1ul << TMR_INTSTS_TFIF_Pos)                    /*!< TMR INTSTS: TFIF Mask                  */

#define TMR_INTSTS_TFIE_Pos              (1)                                               /*!< TMR INTSTS: TFIE Position              */
#define TMR_INTSTS_TFIE_Msk              (0x1ul << TMR_INTSTS_TFIE_Pos)                    /*!< TMR INTSTS: TFIE Mask                  */

#define TMR_CTL_PSC_Pos                  (0)                                               /*!< TMR CTL: PSC Position                  */
#define TMR_CTL_PSC_Msk                  (0xfful << TMR_CTL_PSC_Pos)                       /*!< TMR CTL: PSC Mask                      */

#define TMR_CTL_ACTSTS_Pos               (25)                                              /*!< TMR CTL: ACTSTS Position               */
#define TMR_CTL_ACTSTS_Msk               (0x1ul << TMR_CTL_ACTSTS_Pos)                     /*!< TMR CTL: ACTSTS Mask                   */

#define TMR_CTL_RSTCNT_Pos               (26)                                              /*!< TMR CTL: RSTCNT Position               */
#define TMR_CTL_RSTCNT_Msk               (0x1ul << TMR_CTL_RSTCNT_Pos)                     /*!< TMR CTL: RSTCNT Mask                   */

#define TMR_CTL_OPMODE_Pos               (27)                                              /*!< TMR CTL: OPMODE Position               */
#define TMR_CTL_OPMODE_Msk               (0x3ul << TMR_CTL_OPMODE_Pos)                     /*!< TMR CTL: OPMODE Mask                   */

#define TMR_CTL_INTEN_Pos                (29)                                              /*!< TMR CTL: INTEN Position                */
#define TMR_CTL_INTEN_Msk                (0x1ul << TMR_CTL_INTEN_Pos)                      /*!< TMR CTL: INTEN Mask                    */

#define TMR_CTL_CNTEN_Pos                (30)                                              /*!< TMR CTL: CNTEN Position                */
#define TMR_CTL_CNTEN_Msk                (0x1ul << TMR_CTL_CNTEN_Pos)                      /*!< TMR CTL: CNTEN Mask                    */

#define TMR_CMP_CMPDAT_Pos               (0)                                               /*!< TMR CMP: CMPDAT Position               */
#define TMR_CMP_CMPDAT_Msk               (0xfffful << TMR_CMP_CMPDAT_Pos)                  /*!< TMR CMP: CMPDAT Mask                   */

#define TMR_INTSTS_TIF_Pos               (0)                                               /*!< TMR INTSTS: TIF Position               */
#define TMR_INTSTS_TIF_Msk               (0x1ul << TMR_INTSTS_TIF_Pos)                     /*!< TMR INTSTS: TIF Mask                   */

#define TMR_CNT_CNT_Pos                  (0)                                               /*!< TMR CNT: CNT Position                  */
#define TMR_CNT_CNT_Msk                  (0xfffful << TMR_CNT_CNT_Pos)                     /*!< TMR CNT: CNT Mask                      */

#define TMRF_INTSTS_TFIF_Pos             (0)                                               /*!< TMRF INTSTS: TIF Position               */
#define TMRF_INTSTS_TFIF_Msk             (0x1ul << TMRF_INTSTS_TFIF_Pos)                   /*!< TMRF INTSTS: TIF Mask                   */

#define TMRF_INTSTS_IFIE_Pos             (1)                                               /*!< TMRF INTSTS: TIE Position               */
#define TMRF_INTSTS_IFIE_Msk             (0x1ul << TMRF_INTSTS_IFIE_Pos)                   /*!< TMRF INTSTS: TIE Mask                   */

#define TMRIR_CTL_IRCEN_Pos              (0)                                               /*!< TMRFIR CTL: IRCEN Position               */
#define TMRIR_CTL_IRCEN_Msk              (0x1ul << TMRIR_CTL_IRCEN_Pos)                    /*!< TMRFIR CTL: IRCEN Mask                   */

#define TMRIR_CTL_NONCS_Pos              (1)                                               /*!< TMRFIR CTL: NONCS Position               */
#define TMRIR_CTL_NONCS_Msk              (0x1ul << TMRIR_CTL_NONCS_Pos)                    /*!< TMRFIR CTL: NONCS Mask                   */

/**@}*/ /* TMR_CONST */
/**@}*/ /* end of TMR register group */


/*---------------------- Universal Asynchronous Receiver/Transmitter Controller -------------------------*/
/**
    @addtogroup UART Universal Asynchronous Receiver/Transmitter Controller(UART)
    Memory Mapped Structure for UART Controller
@{ */
 
typedef struct
{


    /**
     * DAT
     * ===================================================================================================
     * Offset: 0x00  UART0 Receive/Transfer FIFO Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |DAT       |Receive FIFO Register
     * |        |          |Reading this register will return data from the receive data FIFO.
     * |        |          |By reading this register, the UART will return the 8-bit data received from Rx pin (LSB first).
 */
    __IO uint32_t DAT;                   

    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x04  UART0 Interrupt Enable Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RDAIEN    |Receive Data Available Interrupt Enable.
     * |        |          |0 = Mask off RDAINT
     * |        |          |1 = Enable RDAINT
     * |[1]     |THREIEN   |Transmit FIFO Register Empty Interrupt Enable
     * |        |          |0 = Mask off THERINT
     * |        |          |1 = Enable THERINT
     * |[2]     |RLSIEN    |Receive Line Status Interrupt Enable
     * |        |          |0 = Mask off RLSINT
     * |        |          |1 = Enable RLSINT
     * |[3]     |MODEMIEN  |Modem Status Interrupt Enable
     * |        |          |0 = Mask off MODEMINT
     * |        |          |1 = Enable MODEMINT
     * |[4]     |RXTOIEN   |Receive Time out Interrupt Enable
     * |        |          |0 = Mask off RXTOINT
     * |        |          |1 = Enable RXTOINT
     * |[5]     |BUFERRIEN |Buffer Error Interrupt Enable
     * |        |          |0 = Mask off BUFERRINT
     * |        |          |1 = Enable IBUFERRINT
     * |[8]     |LINIEN    |LIN RX Break Field Detected Interrupt Enable
     * |        |          |0 = Mask off Lin bus Rx break field interrupt.
     * |        |          |1 = Enable Lin bus Rx break field interrupt.
     * |[11]    |TOCNTEN   |Time-Out Counter Enable
     * |        |          |0 = Disable Time-out counter.
     * |        |          |1 = Enable.
     * |[12]    |ATORTSEN  |RTS Auto Flow Control Enable
     * |        |          |0 = Disable RTS auto flow control.
     * |        |          |1 = Enable.
     * |        |          |When RTS auto-flow is enabled, if the number of bytes in the Rx FIFO equals UART_FIFO.RTSTRGLV, the UART will de-assert the RTS signal.
     * |[13]    |ATOCTSEN  |CTS Auto Flow Control Enable
     * |        |          |0 = Disable CTS auto flow control.
     * |        |          |1 = Enable.
     * |        |          |When CTS auto-flow is enabled, the UART will send data to external device when CTS input is asserted (UART will not send data to device until CTS is asserted).
     * |[14]    |DMATXEN   |Transmit DMA Enable
     * |        |          |If enabled, the UART will request DMA service when space is available in transmit FIFO.
     * |[15]    |DMARXEN   |Receive DMA Enable
     * |        |          |If enabled, the UART will request DMA service when data is available in receive FIFO. 
 */
    __IO uint32_t INTEN;                 

    /**
     * FIFO
     * ===================================================================================================
     * Offset: 0x08  UART0 FIFO Control Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RXRST     |Receive FIFO Reset
     * |        |          |When RXRST is set, all the bytes in the receive FIFO are cleared and receive internal state machine is reset.
     * |        |          |0 = Writing 0 to this bit has no effect.
     * |        |          |1 = Writing 1 to this bit will reset the receiving internal state machine and pointers.
     * |        |          |Note: This bit will auto-clear after 3 UART engine clock cycles.
     * |[2]     |TXRST     |Transmit FIFO Reset
     * |        |          |When TXRST is set, all the bytes in the transmit FIFO are cleared and transmit internal state machine is reset.
     * |        |          |0 = Writing 0 to this bit has no effect.
     * |        |          |1 = Writing 1 to this bit will reset the transmitting internal state machine and pointers.
     * |        |          |Note: This bit will auto-clear after 3 UART engine clock cycles.
     * |[4:7]   |RFITL     |Receive FIFO Interrupt (RDAINT) Trigger Level
     * |        |          |When the number of bytes in the receive FIFO equals the RFITL then the RDAIF will be set and, if enabled, an RDAINT interrupt will generated.
     * |        |          |Value : INTR_RDA Trigger Level (Bytes)
     * |        |          |0 : 1
     * |        |          |1 : 4
     * |        |          |2 : 8
     * |[16:19] |RTSTRGLV  |RTS Trigger Level for Auto-flow Control
     * |        |          |Sets the FIFO trigger level when auto-flow control will de-assert RTS (request-to-send).
     * |        |          |Value : Trigger Level (Bytes)
     * |        |          |0 : 1
     * |        |          |1 : 4
     * |        |          |2 : 8
 */
    __IO uint32_t FIFO;                  

    /**
     * LINE
     * ===================================================================================================
     * Offset: 0x0C  UART0 Line Control Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |WLS       |Word Length Select
     * |        |          |0 (5bits), 1(6bits), 2(7bits), 3(8bits)
     * |[2]     |NSB       |Number of STOP bits
     * |        |          |0= One "STOP bit" is generated after the transmitted data
     * |        |          |1= Two "STOP bits" are generated when 6-, 7- and 8-bit word length is selected; One and a half "STOP bits" are generated in the transmitted data when 5-bit word length is selected
     * |[3]     |PBE       |Parity Bit Enable
     * |        |          |0 = Parity bit is not generated (transmit data) or checked (receive data) during transfer.
     * |        |          |1 = Parity bit is generated or checked between the "last data word bit" and "stop bit" of the serial data.
     * |[4]     |EPE       |Even Parity Enable
     * |        |          |0 = Odd number of logic 1's are transmitted or checked in the data word and parity bits.
     * |        |          |1 = Even number of logic 1's are transmitted or checked in the data word and parity bits.
     * |        |          |This bit has effect only when PBE (parity bit enable) is set.
     * |[5]     |SPE       |Stick Parity Enable
     * |        |          |0 = Disable stick parity
     * |        |          |1 = When bits PBE and SPE are set 'Stick Parity' is enabled.
     * |        |          |If EPE=0 the parity bit is transmitted and checked as always set, if EPE=1, the parity bit is transmitted and checked as always cleared.
     * |[6]     |BCB       |Break Control Bit
     * |        |          |When this bit is set to logic 1, the serial data output (Tx) is forced to the 'Space' state (logic 0).
     * |        |          |Normal condition is serial data output is 'Mark' state.
     * |        |          |This bit acts only on Tx and has no effect on the transmitter logic.
 */
    __IO uint32_t LINE;                  

    /**
     * MODEM
     * ===================================================================================================
     * Offset: 0x10  UART0 Modem Control Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RTS       |RTS (Request-To-Send) Signal
     * |        |          |If UART_INTEN.ATORTSEN = 0, this bit controls whether RTS pin is active or not.
     * |        |          |0 = Drive RTS inactive ( = ~RTSACTLV).
     * |        |          |1 = Drive RTS active ( = RTSACTLV).
     * |[4]     |LBMEN     |Loopback Mode Enable
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[9]     |RTSACTLV  |Request-to-Send (RTS) Active Trigger Level
     * |        |          |This bit can change the RTS trigger level.
     * |        |          |0= RTS is active low level.
     * |        |          |1= RTS is active high level
     * |[13]    |RTSSTS    |RTS Pin State (read only)
     * |        |          |This bit is the pin status of RTS.
 */
    __IO uint32_t MODEM;                 

    /**
     * MODEMSTS
     * ===================================================================================================
     * Offset: 0x14  UART0 Modem Status Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CTSDETF   |Detect CTS State Change Flag
     * |        |          |This bit is set whenever CTS input has state change.
     * |        |          |It will generate Modem interrupt to CPU when UART_INTEN.MODEMIEN = 1.
     * |        |          |NOTE: This bit is cleared by writing 1 to itself.
     * |[4]     |CTSSTS    |CTS Pin Status (read only)
     * |        |          |This bit is the pin status of CTS. 
     * |[8]     |CTSACTLV  |Clear-to-Send (CTS) Active Trigger Level
     * |        |          |This bit can change the CTS trigger level.
     * |        |          |0= CTS is active low level.
     * |        |          |1= CTS is active high level 
 */
    __IO uint32_t MODEMSTS;              

    /**
     * FIFOSTS
     * ===================================================================================================
     * Offset: 0x18  UART0 FIFO Status Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXOVIF    |Rx Overflow Error Interrupt Flag
     * |        |          |If the Rx FIFO ( UART_DAT) is full, and an additional byte is received by the UART, an overflow condition will occur and set this bit to logic 1.
     * |        |          |It will also generate a BUFERRIF event and interrupt if enabled.
     * |        |          |NOTE: This bit is cleared by writing 1 to itself.
     * |[4]     |PEF       |Parity Error Flag
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "parity bit", and is reset whenever the CPU writes 1 to this bit.
     * |[5]     |FEF       |Framing Error Flag
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "stop bit" (that is, the stop bit following the last data bit or parity bit is detected as a logic 0), and is reset whenever the CPU writes 1 to this bit.
     * |[6]     |BIF       |Break Interrupt Flag
     * |        |          |This bit is set to a logic 1 whenever the receive data input (Rx) is held in the "space" state (logic 0) for longer than a full word transmission time (that is, the total time of start bit + data bits + parity + stop bits).
     * |        |          |It is reset whenever the CPU writes 1 to this bit.
     * |[8:13]  |RXPTR     |Rx FIFO pointer (Read Only)
     * |        |          |This field returns the Rx FIFO buffer pointer.
     * |        |          |It is the number of bytes available for read in the Rx FIFO.
     * |        |          |When UART receives one byte from external device, RXPTR is incremented.
     * |        |          |When one byte of Rx FIFO is read by CPU, RXPTR is decremented.
     * |[14]    |RXEMPTY   |Receive FIFO Empty (Read Only)
     * |        |          |This bit indicates whether the Rx FIFO is empty or not.
     * |        |          |When the last byte of Rx FIFO has been read by CPU, hardware sets this bit high.
     * |        |          |It will be cleared when UART receives any new data.
     * |[15]    |RXFULL    |Receive FIFO Full (Read Only)
     * |        |          |This bit indicates whether the Rx FIFO is full or not.
     * |        |          |This bit is set when Rx FIFO is full; otherwise it is cleared by hardware.
     * |[16:21] |TXPTR     |Tx FIFO Pointer (Read Only)
     * |        |          |This field returns the Tx FIFO buffer pointer.
     * |        |          |When CPU writes a byte into the Tx FIFO, TXPTR is incremented.
     * |        |          |When a byte from Tx FIFO is transferred to the Transmit Shift Register, TXPTR is decremented.
     * |[22]    |TXEMPTY   |Transmit FIFO Empty (Read Only)
     * |        |          |This bit indicates whether the Tx FIFO is empty or not.
     * |        |          |When the last byte of Tx FIFO has been transferred to Transmitter Shift Register, hardware sets this bit high.
     * |        |          |It will be cleared after writing data to FIFO (Tx FIFO not empty).
     * |[23]    |TXFULL    |Transmit FIFO Full (Read Only)
     * |        |          |This bit indicates whether the Tx FIFO is full or not.
     * |        |          |This bit is set when Tx FIFO is full; otherwise it is cleared by hardware.
     * |        |          |TXFULL=0 indicates there is room to write more data to Tx FIFO.
     * |[24]    |TXOVIF    |Tx Overflow Error Interrupt Flag
     * |        |          |If the Tx FIFO ( UART_DAT) is full, an additional write to UART_DAT will cause an overflow condition and set this bit to logic 1.
     * |        |          |It will also generate a BUFERRIF event and interrupt if enabled.
     * |        |          |NOTE: This bit is cleared by writing 1 to itself.
     * |[28]    |TXEMPTYF  |Transmitter Empty (Read Only)
     * |        |          |Bit is set by hardware when Tx FIFO is empty and the STOP bit of the last byte has been transmitted.
     * |        |          |Bit is cleared automatically when Tx FIFO is not empty or the last byte transmission has not completed.
     * |        |          |NOTE: This bit is read only. 
 */
    __IO uint32_t FIFOSTS;               

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x1C  UART0 Interrupt Status Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RDAIF     |Receive Data Available Interrupt Flag (Read Only)
     * |        |          |When the number of bytes in the Rx FIFO equals UART_FIFO.RFITL then the RDAIF will be set.
     * |        |          |If UART_INTEN.RDAIEN is enabled, the RDA interrupt will be generated.
     * |        |          |NOTE: This bit is read only and it will be cleared when the number of unread bytes of Rx FIFO drops below the threshold level (RFITL).
     * |[1]     |THREIF    |Transmit Holding Register Empty Interrupt Flag (Read Only)
     * |        |          |This bit is set when the last data of Tx FIFO is transferred to Transmitter Shift Register.
     * |        |          |If UART_INTEN.THREIEN is enabled, the THRE interrupt will be generated.
     * |        |          |NOTE: This bit is read only and it will be cleared when writing data into the Tx FIFO.
     * |[2]     |RLSIF     |Receive Line Status Interrupt Flag (Read Only)
     * |        |          |This bit is set when the Rx receive data has a parity, framing or break error (at least one of, UART_FIFOSTS.BIF, UART_FIFOSTS.FEF and UART_FIFOSTS.PEF, is set).
     * |        |          |If UART_INTEN.RLSIEN is enabled, the RLS interrupt will be generated.
     * |        |          |NOTE: This bit is read only and reset to 0 when all bits of BIF, FEF and PEF are cleared.
     * |[3]     |MODENIF   |MODEM Interrupt Flag (Read Only)
     * |        |          |This bit is set when the CTS pin has changed state (UART_MODEMSTS.CTSDETF=1).
     * |        |          |If UART_INTEN.MODEMIEN is enabled, a CPU interrupt request will be generated.
     * |        |          |NOTE: This bit is read only and reset when bit UART_MODEMSTS.CTSDETF is cleared by a write 1.
     * |[4]     |RXTOIF    |Time Out Interrupt Flag (Read Only)
     * |        |          |This bit is set when the Rx FIFO is not empty and no activity occurs in the Rx FIFO and the time out counter equal to TOIC.
     * |        |          |If UART_INTEN.TOUT_IEN is enabled a CPU interrupt request will be generated.
     * |        |          |NOTE: This bit is read only and user can read FIFO to clear it.
     * |[5]     |BUFERRIF  |Buffer Error Interrupt Flag (Read Only)
     * |        |          |This bit is set when either the Tx or Rx FIFO overflows (UART_FIFOSTS.TXOVIF or UART_FIFOSTS.RXOVIF is set).
     * |        |          |When BUFERRIF is set, the serial transfer may be corrupted.
     * |        |          |If UART_INTEN.BUFERRIEN is enabled a CPU interrupt request will be generated.
     * |        |          |NOTE: This bit is cleared when both UART_FIFOSTS.TXOVIF and UART_FIFOSTS.RXOVIF are cleared. 
     * |[7]     |LINIF     |LIN Bus Rx Break Field Detected Flag
     * |        |          |This bit is set when LIN controller detects a break field. This bit is cleared by writing a 1.
     * |[8]     |RDAINT    |Receive Data Available Interrupt Indicator to Interrupt Controller
     * |        |          |Logical AND of UART_INTEN.RDAIEN and RDAIF.
     * |[9]     |THERINT   |Transmit Holding Register Empty Interrupt Indicator to Interrupt Controller
     * |        |          |Logical AND of UART_INTEN.THREIEN and THREIF.
     * |[10]    |RLSINT    |Receive Line Status Interrupt Indicator to Interrupt Controller
     * |        |          |Logical AND of UART_INTEN.RLSIEN and RLSIF.
     * |[11]    |MODEMINT  |MODEM Status Interrupt Indicator to Interrupt
     * |        |          |Logical AND of UART_INTEN.MODEMIEN and MODENIF.
     * |[12]    |RXTOINT   |Time Out Interrupt Indicator to Interrupt Controller
     * |        |          |Logical AND of UART_INTEN.RXTOIEN and RXTOIF.
     * |[13]    |BUFERRINT |Buffer Error Interrupt Indicator to Interrupt Controller
     * |        |          |Logical AND of UART_INTEN.BUFERRIEN and BUFERRIF.
     * |[15]    |LININT    |LIN Bus Rx Break Field Detected Interrupt Indicator to Interrupt Controller
     * |        |          |Logical AND of UART_INTEN.LINIEN and LINIF.
     * |[18]    |DRLSIF    |DMA MODE Receive Line Status Interrupt Flag (Read Only)
     * |        |          |This bit is set when the Rx receive data has a parity, framing or break error (at least one of, UART_FIFOSTS.BIF, UART_FIFOSTS.FEF and UART_FIFOSTS.PEF, is set).
     * |        |          |If UART_INTEN.RLSIEN is enabled, the RLS interrupt will be generated.
     * |        |          |NOTE: This bit is read only and reset to 0 when all bits of BIF, FEF and PEF are cleared.
     * |[19]    |DMODEMIF  |DMA MODE MODEM Interrupt Flag (Read Only)
     * |        |          |This bit is set when the CTS pin has changed state (UART_MODEMSTS.CTSDETF=1).
     * |        |          |If UART_INTEN.MODEMIEN is enabled, a CPU interrupt request will be generated.
     * |        |          |NOTE: This bit is read only and reset when bit UART_MODEMSTS.CTSDETF is cleared by a write 1.
     * |[20]    |DRXTOIF   |DMA MODE Time Out Interrupt Flag (Read Only)
     * |        |          |This bit is set when the Rx FIFO is not empty and no activity occurs in the Rx FIFO and the time out counter equal to TOIC.
     * |        |          |If UART_INTEN.TOUT_IEN is enabled a CPU interrupt request will be generated.
     * |        |          |NOTE: This bit is read only and user can read FIFO to clear it.
     * |[21]    |DBERRIF   |DMA MODE Buffer Error Interrupt Flag (Read Only)
     * |        |          |This bit is set when either the Tx or Rx FIFO overflows (UART_FIFOSTS.TXOVIF or UART_FIFOSTS.RXOVIF is set).
     * |        |          |When BUFERRIF is set, the serial transfer may be corrupted.
     * |        |          |If UART_INTEN.BUFERRIEN is enabled a CPU interrupt request will be generated.
     * |        |          |NOTE: This bit is cleared when both UART_FIFOSTS.TXOVIF and UART_FIFOSTS.RXOVIF are cleared. 
     * |[23]    |DLINIF    |DMA MODE LIN Bus Rx Break Field Detected Flag
     * |        |          |This bit is set when LIN controller detects a break field. This bit is cleared by writing a 1.
     * |[26]    |DRLSINT   |DMA MODE Receive Line Status Interrupt Indicator to Interrupt Controller
     * |        |          |Logical AND of UART_INTEN.DMARXEN or UART_INTEN.DMATXEN and DRLSIF.
     * |[27]    |DMODEMI   |DMA MODE MODEM Status Interrupt Indicator to Interrupt
     * |        |          |Logical AND of UART_INTEN.DMARXEN or UART_INTEN.DMATXEN and DMODENIF.
     * |[28]    |DRXTOINT  |DMA MODE Time Out Interrupt Indicator to Interrupt Controller
     * |        |          |Logical AND of UART_INTEN.DMARXEN or UART_INTEN.DMATXEN and DRXTOIF.
     * |[29]    |DBERRINT  |DMA MODE Buffer Error Interrupt Indicator to Interrupt Controller
     * |        |          |Logical AND of UART_INTEN.DMARXEN or UART_INTEN.DMATXEN and DBERRIF.
     * |[31]    |DLININT   |DMA MODE LIN Bus Rx Break Field Detected Interrupt Indicator to Interrupt Controller
     * |        |          |Logical AND of UART_INTEN.DMARXEN or UART_INTEN.DMATXEN and DLINIF.
 */
    __IO  uint32_t INTSTS;                

    /**
     * TOUT
     * ===================================================================================================
     * Offset: 0x20  UART0 Time Out Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:6]   |TOIC      |Time Out Interrupt Comparator
     * |        |          |The time out counter resets and starts counting whenever the Rx FIFO receives a new data word.
     * |        |          |Once the content of time out counter (TOUT_CNT) is equal to that of time out interrupt comparator (TOIC), a receiver time out interrupt (RXTOINT) is generated if UART_INTEN.RXTOIEN is set.
     * |        |          |A new incoming data word or RX FIFO empty clears RXTOIF.
     * |        |          |The period of the time out counter is the baud rate.
 */
    __IO uint32_t TOUT;                  

    /**
     * BAUD
     * ===================================================================================================
     * Offset: 0x24  UART0 Baud Rate Divisor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |BRD       |Baud Rate Divider
     * |        |          |Refer to Table 5-111 for more information.
     * |[24:27] |EDIVM1    |Divider X
     * |        |          |The baud rate divider M = EDIVM1+1.
     * |[28]    |BAUDM0    |Divider X equal 1
     * |        |          |0: M = EDIVM1+1, with restriction EDIVM1 >= 8.
     * |        |          |1: M = 1, with restriction BRD[15:0] >= 3.
     * |        |          |Refer to Table 5-111 for more information.
     * |[29]    |BAUDM1    |Divider X Enable
     * |        |          |The baud rate equation is:
     * |        |          |Baud Rate = UART_CLK / [ M * (BRD + 2) ] ; The default value of M is 16.
     * |        |          |0 = Disable divider X ( M = 16)
     * |        |          |1 = Enable divider X (M = EDIVM1+1, with EDIVM1 >= 8).
     * |        |          |Refer to Table 5-111 for more information.
     * |        |          |NOTE: When in IrDA mode, this bit must disabled.
 */
    __IO uint32_t BAUD;                  

    /**
     * IRDA
     * ===================================================================================================
     * Offset: 0x28  UART0 IrDA Control Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TXEN      |Transmit/Receive Selection
     * |        |          |0=Enable IrDA receiver.
     * |        |          |1= Enable IrDA transmitter.
     * |[2]     |LOOPBACK  |IrDA Loopback Test Mode
     * |        |          |Loopback Tx to Rx.
     * |[5]     |TXINV     |Transmit inversion enable
     * |        |          |0= No inversion
     * |        |          |1= Invert Tx output signal
     * |[6]     |RXINV     |Receive Inversion Enable
     * |        |          |0= No inversion
     * |        |          |1= Invert Rx input signal
 */
    __IO uint32_t IRDA;                  

    /**
     * ALTCTL
     * ===================================================================================================
     * Offset: 0x2C  UART0 LIN Control Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |BRKFL     |UART LIN Break Field Length Count
     * |        |          |This field indicates a 4-bit LIN Tx break field count.
     * |        |          |NOTE: This break field length is BRKFL + 2
     * |[6]     |LINRXEN   |LIN RX Enable
     * |        |          |0 = Disable LIN Rx mode.
     * |        |          |1 = Enable LIN Rx mode.
     * |[7]     |LINTXEN   |LIN TX Break Mode Enable
     * |        |          |0 = Disable LIN Tx Break Mode.
     * |        |          |1 = Enable LIN Tx Break Mode.
     * |        |          |NOTE: When Tx break field transfer operation finished, this bit will be cleared automatically.
 */
    __IO uint32_t ALTCTL;                

    /**
     * FUNCSEL
     * ===================================================================================================
     * Offset: 0x30  UART0 Function Select Register.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LINEN     |Enable LIN Function
     * |        |          |0 = UART Function.
     * |        |          |1 = Enable LIN Function.
     * |        |          |Note that IrDA and LIN functions are mutually exclusive: both cannot be active at same time.
     * |[1]     |IRDAEN    |Enable IrDA Function
     * |        |          |0 = UART Function.
     * |        |          |1 = Enable IrDA Function.
 */
    __IO uint32_t FUNCSEL;               

} UART_T;

/**
    @addtogroup UART_CONST UART Bit Field Definition
    Constant Definitions for UART Controller
@{ */

#define UART_DAT_DAT_Pos                 (0)                                               /*!< UART DAT: DAT Position                 */
#define UART_DAT_DAT_Msk                 (0xfful << UART_DAT_DAT_Pos)                      /*!< UART DAT: DAT Mask                     */

#define UART_INTEN_RDAIEN_Pos            (0)                                               /*!< UART INTEN: RDAIEN Position            */
#define UART_INTEN_RDAIEN_Msk            (0x1ul << UART_INTEN_RDAIEN_Pos)                  /*!< UART INTEN: RDAIEN Mask                */

#define UART_INTEN_THREIEN_Pos           (1)                                               /*!< UART INTEN: THREIEN Position           */
#define UART_INTEN_THREIEN_Msk           (0x1ul << UART_INTEN_THREIEN_Pos)                 /*!< UART INTEN: THREIEN Mask               */

#define UART_INTEN_RLSIEN_Pos            (2)                                               /*!< UART INTEN: RLSIEN Position            */
#define UART_INTEN_RLSIEN_Msk            (0x1ul << UART_INTEN_RLSIEN_Pos)                  /*!< UART INTEN: RLSIEN Mask                */

#define UART_INTEN_MODEMIEN_Pos          (3)                                               /*!< UART INTEN: MODEMIEN Position          */
#define UART_INTEN_MODEMIEN_Msk          (0x1ul << UART_INTEN_MODEMIEN_Pos)                /*!< UART INTEN: MODEMIEN Mask              */

#define UART_INTEN_RXTOIEN_Pos           (4)                                               /*!< UART INTEN: RXTOIEN Position           */
#define UART_INTEN_RXTOIEN_Msk           (0x1ul << UART_INTEN_RXTOIEN_Pos)                 /*!< UART INTEN: RXTOIEN Mask               */

#define UART_INTEN_BUFERRIEN_Pos         (5)                                               /*!< UART INTEN: BUFERRIEN Position         */
#define UART_INTEN_BUFERRIEN_Msk         (0x1ul << UART_INTEN_BUFERRIEN_Pos)               /*!< UART INTEN: BUFERRIEN Mask             */

#define UART_INTEN_LINIEN_Pos            (8)                                               /*!< UART INTEN: LINIEN Position            */
#define UART_INTEN_LINIEN_Msk            (0x1ul << UART_INTEN_LINIEN_Pos)                  /*!< UART INTEN: LINIEN Mask                */

#define UART_INTEN_TOCNTEN_Pos           (11)                                              /*!< UART INTEN: TOCNTEN Position           */
#define UART_INTEN_TOCNTEN_Msk           (0x1ul << UART_INTEN_TOCNTEN_Pos)                 /*!< UART INTEN: TOCNTEN Mask               */

#define UART_INTEN_ATORTSEN_Pos          (12)                                              /*!< UART INTEN: ATORTSEN Position          */
#define UART_INTEN_ATORTSEN_Msk          (0x1ul << UART_INTEN_ATORTSEN_Pos)                /*!< UART INTEN: ATORTSEN Mask              */

#define UART_INTEN_ATOCTSEN_Pos          (13)                                              /*!< UART INTEN: ATOCTSEN Position          */
#define UART_INTEN_ATOCTSEN_Msk          (0x1ul << UART_INTEN_ATOCTSEN_Pos)                /*!< UART INTEN: ATOCTSEN Mask              */

#define UART_INTEN_DMATXEN_Pos           (14)                                              /*!< UART INTEN: DMATXEN Position           */
#define UART_INTEN_DMATXEN_Msk           (0x1ul << UART_INTEN_DMATXEN_Pos)                 /*!< UART INTEN: DMATXEN Mask               */

#define UART_INTEN_DMARXEN_Pos           (15)                                              /*!< UART INTEN: DMARXEN Position           */
#define UART_INTEN_DMARXEN_Msk           (0x1ul << UART_INTEN_DMARXEN_Pos)                 /*!< UART INTEN: DMARXEN Mask               */

#define UART_FIFO_RXRST_Pos              (1)                                               /*!< UART FIFO: RXRST Position              */
#define UART_FIFO_RXRST_Msk              (0x1ul << UART_FIFO_RXRST_Pos)                    /*!< UART FIFO: RXRST Mask                  */

#define UART_FIFO_TXRST_Pos              (2)                                               /*!< UART FIFO: TXRST Position              */
#define UART_FIFO_TXRST_Msk              (0x1ul << UART_FIFO_TXRST_Pos)                    /*!< UART FIFO: TXRST Mask                  */

#define UART_FIFO_RFITL_Pos              (4)                                               /*!< UART FIFO: RFITL Position              */
#define UART_FIFO_RFITL_Msk              (0xful << UART_FIFO_RFITL_Pos)                    /*!< UART FIFO: RFITL Mask                  */

#define UART_FIFO_RTSTRGLV_Pos           (16)                                              /*!< UART FIFO: RTSTRGLV Position           */
#define UART_FIFO_RTSTRGLV_Msk           (0xful << UART_FIFO_RTSTRGLV_Pos)                 /*!< UART FIFO: RTSTRGLV Mask               */

#define UART_LINE_WLS_Pos                (0)                                               /*!< UART LINE: WLS Position                */
#define UART_LINE_WLS_Msk                (0x3ul << UART_LINE_WLS_Pos)                      /*!< UART LINE: WLS Mask                    */

#define UART_LINE_NSB_Pos                (2)                                               /*!< UART LINE: NSB Position                */
#define UART_LINE_NSB_Msk                (0x1ul << UART_LINE_NSB_Pos)                      /*!< UART LINE: NSB Mask                    */

#define UART_LINE_PBE_Pos                (3)                                               /*!< UART LINE: PBE Position                */
#define UART_LINE_PBE_Msk                (0x1ul << UART_LINE_PBE_Pos)                      /*!< UART LINE: PBE Mask                    */

#define UART_LINE_EPE_Pos                (4)                                               /*!< UART LINE: EPE Position                */
#define UART_LINE_EPE_Msk                (0x1ul << UART_LINE_EPE_Pos)                      /*!< UART LINE: EPE Mask                    */

#define UART_LINE_SPE_Pos                (5)                                               /*!< UART LINE: SPE Position                */
#define UART_LINE_SPE_Msk                (0x1ul << UART_LINE_SPE_Pos)                      /*!< UART LINE: SPE Mask                    */

#define UART_LINE_BCB_Pos                (6)                                               /*!< UART LINE: BCB Position                */
#define UART_LINE_BCB_Msk                (0x1ul << UART_LINE_BCB_Pos)                      /*!< UART LINE: BCB Mask                    */

#define UART_MODEM_RTS_Pos               (1)                                               /*!< UART MODEM: RTS Position               */
#define UART_MODEM_RTS_Msk               (0x1ul << UART_MODEM_RTS_Pos)                     /*!< UART MODEM: RTS Mask                   */

#define UART_MODEM_LBMEN_Pos             (4)                                               /*!< UART MODEM: LBMEN Position             */
#define UART_MODEM_LBMEN_Msk             (0x1ul << UART_MODEM_LBMEN_Pos)                   /*!< UART MODEM: LBMEN Mask                 */

#define UART_MODEM_RTSACTLV_Pos          (9)                                               /*!< UART MODEM: RTSACTLV Position          */
#define UART_MODEM_RTSACTLV_Msk          (0x1ul << UART_MODEM_RTSACTLV_Pos)                /*!< UART MODEM: RTSACTLV Mask              */

#define UART_MODEM_RTSSTS_Pos            (13)                                              /*!< UART MODEM: RTSSTS Position            */
#define UART_MODEM_RTSSTS_Msk            (0x1ul << UART_MODEM_RTSSTS_Pos)                  /*!< UART MODEM: RTSSTS Mask                */

#define UART_MODEMSTS_CTSDETF_Pos        (0)                                               /*!< UART MODEMSTS: CTSDETF Position        */
#define UART_MODEMSTS_CTSDETF_Msk        (0x1ul << UART_MODEMSTS_CTSDETF_Pos)              /*!< UART MODEMSTS: CTSDETF Mask            */

#define UART_MODEMSTS_CTSSTS_Pos         (4)                                               /*!< UART MODEMSTS: CTSSTS Position         */
#define UART_MODEMSTS_CTSSTS_Msk         (0x1ul << UART_MODEMSTS_CTSSTS_Pos)               /*!< UART MODEMSTS: CTSSTS Mask             */

#define UART_MODEMSTS_CTSACTLV_Pos       (8)                                               /*!< UART MODEMSTS: CTSACTLV Position       */
#define UART_MODEMSTS_CTSACTLV_Msk       (0x1ul << UART_MODEMSTS_CTSACTLV_Pos)             /*!< UART MODEMSTS: CTSACTLV Mask           */

#define UART_FIFOSTS_RXOVIF_Pos          (0)                                               /*!< UART FIFOSTS: RXOVIF Position          */
#define UART_FIFOSTS_RXOVIF_Msk          (0x1ul << UART_FIFOSTS_RXOVIF_Pos)                /*!< UART FIFOSTS: RXOVIF Mask              */

#define UART_FIFOSTS_PEF_Pos             (4)                                               /*!< UART FIFOSTS: PEF Position             */
#define UART_FIFOSTS_PEF_Msk             (0x1ul << UART_FIFOSTS_PEF_Pos)                   /*!< UART FIFOSTS: PEF Mask                 */

#define UART_FIFOSTS_FEF_Pos             (5)                                               /*!< UART FIFOSTS: FEF Position             */
#define UART_FIFOSTS_FEF_Msk             (0x1ul << UART_FIFOSTS_FEF_Pos)                   /*!< UART FIFOSTS: FEF Mask                 */

#define UART_FIFOSTS_BIF_Pos             (6)                                               /*!< UART FIFOSTS: BIF Position             */
#define UART_FIFOSTS_BIF_Msk             (0x1ul << UART_FIFOSTS_BIF_Pos)                   /*!< UART FIFOSTS: BIF Mask                 */

#define UART_FIFOSTS_RXPTR_Pos           (8)                                               /*!< UART FIFOSTS: RXPTR Position           */
#define UART_FIFOSTS_RXPTR_Msk           (0x3ful << UART_FIFOSTS_RXPTR_Pos)                /*!< UART FIFOSTS: RXPTR Mask               */

#define UART_FIFOSTS_RXEMPTY_Pos         (14)                                              /*!< UART FIFOSTS: RXEMPTY Position         */
#define UART_FIFOSTS_RXEMPTY_Msk         (0x1ul << UART_FIFOSTS_RXEMPTY_Pos)               /*!< UART FIFOSTS: RXEMPTY Mask             */

#define UART_FIFOSTS_RXFULL_Pos          (15)                                              /*!< UART FIFOSTS: RXFULL Position          */
#define UART_FIFOSTS_RXFULL_Msk          (0x1ul << UART_FIFOSTS_RXFULL_Pos)                /*!< UART FIFOSTS: RXFULL Mask              */

#define UART_FIFOSTS_TXPTR_Pos           (16)                                              /*!< UART FIFOSTS: TXPTR Position           */
#define UART_FIFOSTS_TXPTR_Msk           (0x3ful << UART_FIFOSTS_TXPTR_Pos)                /*!< UART FIFOSTS: TXPTR Mask               */

#define UART_FIFOSTS_TXEMPTY_Pos         (22)                                              /*!< UART FIFOSTS: TXEMPTY Position         */
#define UART_FIFOSTS_TXEMPTY_Msk         (0x1ul << UART_FIFOSTS_TXEMPTY_Pos)               /*!< UART FIFOSTS: TXEMPTY Mask             */

#define UART_FIFOSTS_TXFULL_Pos          (23)                                              /*!< UART FIFOSTS: TXFULL Position          */
#define UART_FIFOSTS_TXFULL_Msk          (0x1ul << UART_FIFOSTS_TXFULL_Pos)                /*!< UART FIFOSTS: TXFULL Mask              */

#define UART_FIFOSTS_TXOVIF_Pos          (24)                                              /*!< UART FIFOSTS: TXOVIF Position          */
#define UART_FIFOSTS_TXOVIF_Msk          (0x1ul << UART_FIFOSTS_TXOVIF_Pos)                /*!< UART FIFOSTS: TXOVIF Mask              */

#define UART_FIFOSTS_TXEMPTYF_Pos        (28)                                              /*!< UART FIFOSTS: TXEMPTYF Position        */
#define UART_FIFOSTS_TXEMPTYF_Msk        (0x1ul << UART_FIFOSTS_TXEMPTYF_Pos)              /*!< UART FIFOSTS: TXEMPTYF Mask            */

#define UART_INTSTS_RDAIF_Pos            (0)                                               /*!< UART INTSTS: RDAIF Position            */
#define UART_INTSTS_RDAIF_Msk            (0x1ul << UART_INTSTS_RDAIF_Pos)                  /*!< UART INTSTS: RDAIF Mask                */

#define UART_INTSTS_THREIF_Pos           (1)                                               /*!< UART INTSTS: THREIF Position           */
#define UART_INTSTS_THREIF_Msk           (0x1ul << UART_INTSTS_THREIF_Pos)                 /*!< UART INTSTS: THREIF Mask               */

#define UART_INTSTS_RLSIF_Pos            (2)                                               /*!< UART INTSTS: RLSIF Position            */
#define UART_INTSTS_RLSIF_Msk            (0x1ul << UART_INTSTS_RLSIF_Pos)                  /*!< UART INTSTS: RLSIF Mask                */

#define UART_INTSTS_MODENIF_Pos          (3)                                               /*!< UART INTSTS: MODENIF Position          */
#define UART_INTSTS_MODENIF_Msk          (0x1ul << UART_INTSTS_MODENIF_Pos)                /*!< UART INTSTS: MODENIF Mask              */

#define UART_INTSTS_RXTOIF_Pos           (4)                                               /*!< UART INTSTS: RXTOIF Position           */
#define UART_INTSTS_RXTOIF_Msk           (0x1ul << UART_INTSTS_RXTOIF_Pos)                 /*!< UART INTSTS: RXTOIF Mask               */

#define UART_INTSTS_BUFERRIF_Pos         (5)                                               /*!< UART INTSTS: BUFERRIF Position         */
#define UART_INTSTS_BUFERRIF_Msk         (0x1ul << UART_INTSTS_BUFERRIF_Pos)               /*!< UART INTSTS: BUFERRIF Mask             */

#define UART_INTSTS_LINIF_Pos            (7)                                               /*!< UART INTSTS: LINIF Position            */
#define UART_INTSTS_LINIF_Msk            (0x1ul << UART_INTSTS_LINIF_Pos)                  /*!< UART INTSTS: LINIF Mask                */

#define UART_INTSTS_RDAINT_Pos           (8)                                               /*!< UART INTSTS: RDAINT Position           */
#define UART_INTSTS_RDAINT_Msk           (0x1ul << UART_INTSTS_RDAINT_Pos)                 /*!< UART INTSTS: RDAINT Mask               */

#define UART_INTSTS_THERINT_Pos          (9)                                               /*!< UART INTSTS: THERINT Position          */
#define UART_INTSTS_THERINT_Msk          (0x1ul << UART_INTSTS_THERINT_Pos)                /*!< UART INTSTS: THERINT Mask              */

#define UART_INTSTS_RLSINT_Pos           (10)                                              /*!< UART INTSTS: RLSINT Position           */
#define UART_INTSTS_RLSINT_Msk           (0x1ul << UART_INTSTS_RLSINT_Pos)                 /*!< UART INTSTS: RLSINT Mask               */

#define UART_INTSTS_MODEMINT_Pos         (11)                                              /*!< UART INTSTS: MODEMINT Position         */
#define UART_INTSTS_MODEMINT_Msk         (0x1ul << UART_INTSTS_MODEMINT_Pos)               /*!< UART INTSTS: MODEMINT Mask             */

#define UART_INTSTS_RXTOINT_Pos          (12)                                              /*!< UART INTSTS: RXTOINT Position          */
#define UART_INTSTS_RXTOINT_Msk          (0x1ul << UART_INTSTS_RXTOINT_Pos)                /*!< UART INTSTS: RXTOINT Mask              */

#define UART_INTSTS_BUFERRINT_Pos        (13)                                              /*!< UART INTSTS: BUFERRINT Position        */
#define UART_INTSTS_BUFERRINT_Msk        (0x1ul << UART_INTSTS_BUFERRINT_Pos)              /*!< UART INTSTS: BUFERRINT Mask            */

#define UART_INTSTS_LININT_Pos           (15)                                              /*!< UART INTSTS: LININT Position           */
#define UART_INTSTS_LININT_Msk           (0x1ul << UART_INTSTS_LININT_Pos)                 /*!< UART INTSTS: LININT Mask               */

#define UART_INTSTS_DRLSIF_Pos           (18)                                              /*!< UART INTSTS: DRLSIF Position           */
#define UART_INTSTS_DRLSIF_Msk           (0x1ul << UART_INTSTS_DRLSIF_Pos)                 /*!< UART INTSTS: DRLSIF Mask               */

#define UART_INTSTS_DMODEMIF_Pos         (19)                                              /*!< UART INTSTS: DMODEMIF Position         */
#define UART_INTSTS_DMODEMIF_Msk         (0x1ul << UART_INTSTS_DMODEMIF_Pos)               /*!< UART INTSTS: DMODEMIF Mask             */

#define UART_INTSTS_DRXTOIF_Pos          (20)                                              /*!< UART INTSTS: DRXTOIF Position          */
#define UART_INTSTS_DRXTOIF_Msk          (0x1ul << UART_INTSTS_DRXTOIF_Pos)                /*!< UART INTSTS: DRXTOIF Mask              */

#define UART_INTSTS_DBERRIF_Pos          (21)                                              /*!< UART INTSTS: DBERRIF Position          */
#define UART_INTSTS_DBERRIF_Msk          (0x1ul << UART_INTSTS_DBERRIF_Pos)                /*!< UART INTSTS: DBERRIF Mask              */

#define UART_INTSTS_DLINIF_Pos           (23)                                              /*!< UART INTSTS: DLINIF Position           */
#define UART_INTSTS_DLINIF_Msk           (0x1ul << UART_INTSTS_DLINIF_Pos)                 /*!< UART INTSTS: DLINIF Mask               */

#define UART_INTSTS_DRLSINT_Pos          (26)                                              /*!< UART INTSTS: DRLSINT Position          */
#define UART_INTSTS_DRLSINT_Msk          (0x1ul << UART_INTSTS_DRLSINT_Pos)                /*!< UART INTSTS: DRLSINT Mask              */

#define UART_INTSTS_DMODEMI_Pos          (27)                                              /*!< UART INTSTS: DMODEMI Position          */
#define UART_INTSTS_DMODEMI_Msk          (0x1ul << UART_INTSTS_DMODEMI_Pos)                /*!< UART INTSTS: DMODEMI Mask              */

#define UART_INTSTS_DRXTOINT_Pos         (28)                                              /*!< UART INTSTS: DRXTOINT Position         */
#define UART_INTSTS_DRXTOINT_Msk         (0x1ul << UART_INTSTS_DRXTOINT_Pos)               /*!< UART INTSTS: DRXTOINT Mask             */

#define UART_INTSTS_DBERRINT_Pos         (29)                                              /*!< UART INTSTS: DBERRINT Position         */
#define UART_INTSTS_DBERRINT_Msk         (0x1ul << UART_INTSTS_DBERRINT_Pos)               /*!< UART INTSTS: DBERRINT Mask             */

#define UART_INTSTS_DLININT_Pos          (31)                                              /*!< UART INTSTS: DLININT Position          */
#define UART_INTSTS_DLININT_Msk          (0x1ul << UART_INTSTS_DLININT_Pos)                /*!< UART INTSTS: DLININT Mask              */

#define UART_TOUT_TOIC_Pos               (0)                                               /*!< UART TOUT: TOIC Position               */
#define UART_TOUT_TOIC_Msk               (0x7ful << UART_TOUT_TOIC_Pos)                    /*!< UART TOUT: TOIC Mask                   */

#define UART_BAUD_BRD_Pos                (0)                                               /*!< UART BAUD: BRD Position                */
#define UART_BAUD_BRD_Msk                (0xfffful << UART_BAUD_BRD_Pos)                   /*!< UART BAUD: BRD Mask                    */

#define UART_BAUD_EDIVM1_Pos             (24)                                              /*!< UART BAUD: EDIVM1 Position             */
#define UART_BAUD_EDIVM1_Msk             (0xful << UART_BAUD_EDIVM1_Pos)                   /*!< UART BAUD: EDIVM1 Mask                 */

#define UART_BAUD_BAUDM0_Pos             (28)                                              /*!< UART BAUD: BAUDM0 Position             */
#define UART_BAUD_BAUDM0_Msk             (0x1ul << UART_BAUD_BAUDM0_Pos)                   /*!< UART BAUD: BAUDM0 Mask                 */

#define UART_BAUD_BAUDM1_Pos             (29)                                              /*!< UART BAUD: BAUDM1 Position             */
#define UART_BAUD_BAUDM1_Msk             (0x1ul << UART_BAUD_BAUDM1_Pos)                   /*!< UART BAUD: BAUDM1 Mask                 */

#define UART_IRDA_TXEN_Pos               (1)                                               /*!< UART IRDA: TXEN Position               */
#define UART_IRDA_TXEN_Msk               (0x1ul << UART_IRDA_TXEN_Pos)                     /*!< UART IRDA: TXEN Mask                   */

#define UART_IRDA_LOOPBACK_Pos           (2)                                               /*!< UART IRDA: LOOPBACK Position           */
#define UART_IRDA_LOOPBACK_Msk           (0x1ul << UART_IRDA_LOOPBACK_Pos)                 /*!< UART IRDA: LOOPBACK Mask               */

#define UART_IRDA_TXINV_Pos              (5)                                               /*!< UART IRDA: TXINV Position              */
#define UART_IRDA_TXINV_Msk              (0x1ul << UART_IRDA_TXINV_Pos)                    /*!< UART IRDA: TXINV Mask                  */

#define UART_IRDA_RXINV_Pos              (6)                                               /*!< UART IRDA: RXINV Position              */
#define UART_IRDA_RXINV_Msk              (0x1ul << UART_IRDA_RXINV_Pos)                    /*!< UART IRDA: RXINV Mask                  */

#define UART_ALTCTL_BRKFL_Pos            (0)                                               /*!< UART ALTCTL: BRKFL Position            */
#define UART_ALTCTL_BRKFL_Msk            (0xful << UART_ALTCTL_BRKFL_Pos)                  /*!< UART ALTCTL: BRKFL Mask                */

#define UART_ALTCTL_LINRXEN_Pos          (6)                                               /*!< UART ALTCTL: LINRXEN Position          */
#define UART_ALTCTL_LINRXEN_Msk          (0x1ul << UART_ALTCTL_LINRXEN_Pos)                /*!< UART ALTCTL: LINRXEN Mask              */

#define UART_ALTCTL_LINTXEN_Pos          (7)                                               /*!< UART ALTCTL: LINTXEN Position          */
#define UART_ALTCTL_LINTXEN_Msk          (0x1ul << UART_ALTCTL_LINTXEN_Pos)                /*!< UART ALTCTL: LINTXEN Mask              */

#define UART_FUNCSEL_LINEN_Pos           (0)                                               /*!< UART FUNCSEL: LINEN Position           */
#define UART_FUNCSEL_LINEN_Msk           (0x1ul << UART_FUNCSEL_LINEN_Pos)                 /*!< UART FUNCSEL: LINEN Mask               */

#define UART_FUNCSEL_IRDAEN_Pos          (1)                                               /*!< UART FUNCSEL: IRDAEN Position          */
#define UART_FUNCSEL_IRDAEN_Msk          (0x1ul << UART_FUNCSEL_IRDAEN_Pos)                /*!< UART FUNCSEL: IRDAEN Mask              */

/**@}*/ /* UART_CONST */
/**@}*/ /* end of UART register group */

/*---------------------- Serial Peripheral Interface Controller -------------------------*/
/**
    @addtogroup SPI Serial Peripheral Interface Controller(SPI)
    Memory Mapped Structure for SPI Controller
@{ */
typedef struct
{
    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BUSY      |Go and Busy Status
     * |        |          |0 = Writing 0 to this bit has no effect.
     * |        |          |1 = Writing 1 to this bit starts the transfer.
     * |        |          |This bit remains set during the transfer and is automatically cleared after transfer finished.
     * |        |          |NOTE: All registers should be set before writing 1 to this BUSY bit.
     * |        |          |When a transfer is in progress, writing to any register of the SPI master/slave core has no effect.
     * |[1]     |RXNEG     |Receive At Negative Edge
     * |        |          |0 = The received data input signal is latched at the rising edge of SCLK.
     * |        |          |1 = The received data input signal is latched at the falling edge of SCLK.
     * |[2]     |TXNEG     |Transmit At Negative Edge
     * |        |          |0 = The transmitted data output signal is changed at the rising edge of SCLK.
     * |        |          |1 = The transmitted data output signal is changed at the falling edge of SCLK.
     * |[3:7]   |DWIDTH    |Transmit Bit Length
     * |        |          |This field specifies how many bits are transmitted in one transmit/receive.
     * |        |          |Up to 32 bits can be transmitted.
     * |        |          |DWIDTH = 0x01 --- 1 bit
     * |        |          |DWIDTH = 0x02 --- 2 bits
     * |        |          |----
     * |        |          |DWIDTH = 0x1f --- 31 bits
     * |        |          |DWIDTH = 0x00 --- 32 bits
     * |[8:9]   |TXCNT     |Transmit/Receive Word Numbers
     * |        |          |This field specifies how many transmit/receive word numbers should be executed in one transfer.
     * |        |          |00 = Only one transmit/receive word will be executed in one transfer.
     * |        |          |01 = Two successive transmit/receive word will be executed in one transfer.
     * |        |          |10 = Reserved.
     * |        |          |11 = Reserved.
     * |[10]    |LSB       |LSB First
     * |        |          |0 = The MSB is transmitted/received first (which bit in SPI_TX0/1 and SPI_RX0/1 register that is depends on the DWIDTH field).
     * |        |          |1 = The LSB is sent first on the line (bit 0 of SPI_TX0/1), and the first bit received from the line will be put in the LSB position in the Rx register (bit 0 of SPI_RX0/1).
     * |[11]    |CLKPOL    |Clock Polarity
     * |        |          |0 = SCLK idle low.
     * |        |          |1 = SCLK idle high.
     * |[12:15] |SUSPITV   |Suspend Interval (Master Only)
     * |        |          |These four bits provide configurable suspend interval between two successive transmit/receive transactions in a transfer.
     * |        |          |The suspend interval is from the last falling clock edge of the current transaction to the first rising clock edge of the successive transaction if CLKPOL = 0.
     * |        |          |If CLKPOL = 1, the interval is from the rising clock edge to the falling clock edge.
     * |        |          |The default value is 0x0.
     * |        |          |When TXCNT = 00b, setting this field has no effect on transfer except as determined by REORDER[0] setting.
     * |        |          |The suspend interval is determined according to the following equation:.
     * |        |          |(SUSPITV[3:0] + 2) * period of SCLK
     * |[16]    |UNITIF    |Interrupt Flag
     * |        |          |0 = Indicates the transfer is not finished yet.
     * |        |          |1 = Indicates that the transfer is complete. Interrupt is generated to CPU if enabled.
     * |        |          |NOTE: This bit is cleared by writing 1 to itself.
     * |[17]    |UNITIEN   |Interrupt Enable
     * |        |          |0 = Disable SPI Interrupt.
     * |        |          |1 = Enable SPI Interrupt to CPU.
     * |[18]    |SLAVE     |Master Slave Mode Control
     * |        |          |0 = Master mode.
     * |        |          |1 = Slave mode.
     * |[19]    |BYTEITV   |Insert Sleep Interval Between Bytes
     * |        |          |This function is only valid for 32bit transfers (DWIDTH = 0).
     * |        |          |If set then a pause of (SUSPITV+2) SCLK cycles is inserted between each byte transmitted.
     * |[20]    |REORDER   |Byte Endian Reorder Function
     * |        |          |This function changes the order of bytes sent/received to be least significant physical byte first. 
     * |[21]    |FIFOEN    |FIFO Mode
     * |        |          |0 = No FIFO present on transmit and receive buffer.
     * |        |          |1 = Enable FIFO on transmit and receive buffer.
     * |[23]    |VARCLKEN  |Variable Clock Enable (Master Only)
     * |        |          |0 = The serial clock output frequency is fixed and determined only by the value of DIVIDER0.
     * |        |          |1 = SCLK output frequency is variable.
     * |        |          |The output frequency is determined by the value of SPI_VARCLK, DIVIDER0, and DIVIDER1.
     * |        |          |Note that when enabled, the setting of DWIDTH must be programmed as 0x10 (16 bits mode)
     * |[24]    |RXEMPTY   |Receive FIFO Empty Status
     * |        |          |0 = The receive data FIFO is not empty.
     * |        |          |1 = The receive data FIFO is empty.
     * |[25]    |RXFULL    |Receive FIFO Full Status
     * |        |          |0 = The receive data FIFO is not full.
     * |        |          |1 = The receive data FIFO is full.
     * |[26]    |TXEMPTY   |Transmit FIFO Empty Status
     * |        |          |0 = The transmit data FIFO is not empty.
     * |        |          |1 = The transmit data FIFO is empty.
     * |[27]    |TXFULL    |Transmit FIFO Full Status
     * |        |          |0 = The transmit data FIFO is not full.
     * |        |          |1 = The transmit data FIFO is full.
     * |[28]    |PDMASSEN  |Enable DMA Automatic SS function
     * |        |          |When enabled, interface will automatically generate a SS signal for an entire PDMA access transaction.
     */
    __IO uint32_t CTL;                   

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x04  Clock Divider Register (Master Only)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |DIVIDER0  |Clock Divider Register (master only)
     * |        |          |The value in this field is the frequency division of the system clock, PCLK, to generate the serial clock on the output SCLK.
     * |        |          |The desired frequency is obtained according to the following equation:.
     * |        |          |Fsclk = Fpclk / ((DIVIDER0+1) * 2)
     * |        |          |In slave mode, the period of SPI clock driven by a master shall satisfy
     * |        |          |Fsclk < = (Fpclk / 5)
     * |        |          |In other words, the maximum frequency of SCLK clock is one fifth of the SPI peripheral clock.
     * |[16:31] |DIVIDER1  |Clock Divider 2 Register (master only)
     * |        |          |The value in this field is the 2nd frequency divider of the system clock, PCLK, to generate the serial clock on the output SCLK.
     * |        |          |The desired frequency is obtained according to the following equation:.
     * |        |          |Fsclk = Fpclk / ((DIVIDER1+1) * 2)
     */
    __IO uint32_t CLKDIV;                

    /**
     * SSCTL
     * ===================================================================================================
     * Offset: 0x08  Slave Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |SS        |Slave Select Register (Master only)
     * |        |          |If AUTOSS bit is cleared, writing 1 to any bit location of this field sets the proper SPISSx0/1 line to an active state and writing 0 sets the line back to inactive state.
     * |        |          |If AUTOSS bit is set, writing 1 to any bit location of this field will select appropriate SPISSx0/1 line to be automatically driven to active state for the duration of the transmit/receive, and will be driven to inactive state for the rest of the time.
     * |        |          |(The active level of SPISSx0/1 is specified in SSACTPOL).
     * |        |          |Note: SPISSx0 is always defined as device/slave select input signal in slave mode. 
     * |[2]     |SSACTPOL  |Slave Select Active Level
     * |        |          |It defines the active level of device/slave select signal (SPISSx0/1).
     * |        |          |0 = The slave select signal SPISSx0/1 is active at low-level/falling-edge.
     * |        |          |1 = The slave select signal SPISSx0/1 is active at high-level/rising-edge.
     * |[3]     |AUTOSS    |Automatic Slave Select (Master only)
     * |        |          |0 = If this bit is cleared, slave select signals are asserted and de-asserted by setting and clearing related bits in SPI_SSCTL[1:0] register.
     * |        |          |1 = If this bit is set, SPISSx0/1 signals are generated automatically.
     * |        |          |It means that device/slave select signal, which is set in SPI_SSCTL[1:0] register is asserted by the SPI controller when transmit/receive is started by setting BUSY, and is de-asserted after each transmit/receive is finished.
     * |[4]     |LVTRGEN   |Slave Select Level Trigger (Slave only)
     * |        |          |0= The input slave select signal is edge-trigger. This is the default value.
     * |        |          |1= The slave select signal will be level-trigger.
     * |        |          |It depends on SSACTPOL to decide the signal is active low or active high.
     * |[5]     |LVTRGSTS  |Level Trigger Flag
     * |        |          |When the LVTRGEN bit is set in slave mode, this bit can be read to indicate the received bit number is met the requirement or not.
     * |        |          |0=One of the received number and the received bit length doesn't meet the requirement in one transfer.
     * |        |          |1=The received number and received bits met the requirement which defines in TXCNT and DWIDTH among one transfer.
     * |        |          |Note: This bit is READ only
     */
    __IO uint32_t SSCTL;      
	
         uint32_t RESERVE0[1];

    /**
     * RX0
     * ===================================================================================================
     * Offset: 0x10  Data Receive Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |RX        |Data Receive Register
     * |        |          |The Data Receive Registers hold the value of received data of the last executed transfer.
     * |        |          |Valid bits depend on the transmit bit length field in the SPI_CTL register.
     * |        |          |For example, if DWIDTH is set to 0x08 and TXCNT is set to 0x0, bit Rx0[7:0] holds the received data.
     * |        |          |NOTE: The Data Receive Registers are read only registers. 
     */
    __I  uint32_t RX0;                   

    /**
     * RX1
     * ===================================================================================================
     * Offset: 0x14  Data Receive Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |RX        |Data Receive Register
     * |        |          |The Data Receive Registers hold the value of received data of the last executed transfer.
     * |        |          |Valid bits depend on the transmit bit length field in the SPI_CTL register.
     * |        |          |For example, if DWIDTH is set to 0x08 and TXCNT is set to 0x0, bit Rx0[7:0] holds the received data.
     * |        |          |NOTE: The Data Receive Registers are read only registers. 
     */
    __I  uint32_t RX1;      
	
         uint32_t RESERVE1[2];

    /**
     * TX0
     * ===================================================================================================
     * Offset: 0x20  Data Transmit Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |TX        |Data Transmit Register
     * |        |          |The Data Transmit Registers hold the data to be transmitted in the next transfer.
     * |        |          |Valid bits depend on the transmit bit length field in the SPI_CTL register.
     * |        |          |For example, if DWIDTH is set to 0x08 and the TXCNT is set to 0x0, the bit TX0[7:0] will be transmitted in next transfer.
     * |        |          |If DWIDTH is set to 0x00 and TXCNT is set to 0x1, the core will perform two 32-bit transmit/receive successive using the same setting (the order is TX0[31:0], TX1[31:0]).
     */
    __O  uint32_t TX0;                   

    /**
     * TX1
     * ===================================================================================================
     * Offset: 0x24  Data Transmit Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |TX        |Data Transmit Register
     * |        |          |The Data Transmit Registers hold the data to be transmitted in the next transfer.
     * |        |          |Valid bits depend on the transmit bit length field in the SPI_CTL register.
     * |        |          |For example, if DWIDTH is set to 0x08 and the TXCNT is set to 0x0, the bit TX0[7:0] will be transmitted in next transfer.
     * |        |          |If DWIDTH is set to 0x00 and TXCNT is set to 0x1, the core will perform two 32-bit transmit/receive successive using the same setting (the order is TX0[31:0], TX1[31:0]).
     */
    __O  uint32_t TX1; 
	
         uint32_t RESERVE2[3];

    /**
     * VARCLK
     * ===================================================================================================
     * Offset: 0x34  Variable Clock Pattern Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |VARCLK    |Variable Clock Pattern
     * |        |          |The value in this field is the frequency pattern of the SPI clock.
     * |        |          |If the bit field of VARCLK is '0', the output frequency of SCLK is given by the value of DIVIDER0.
     * |        |          |If the bit field of VARCLK is '1', the output frequency of SCLK is given by the value of DIVIDER1.
     * |        |          |Refer to register DIVIDER0.
     * |        |          |Refer to Variable Serial Clock Frequency paragraph for detailed description.
     * |        |          |Note: Used for CLKPOL = 0 only, 16 bit transmission.
     */
    __IO uint32_t VARCLK;                

    /**
     * PDMACTL
     * ===================================================================================================
     * Offset: 0x38  SPI DMA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TXPDMAEN  |Transmit DMA Start
     * |        |          |Set this bit to 1 will start the transmit DMA process.
     * |        |          |SPI module will issue request to DMA module automatically.
     * |        |          |If using DMA mode to transfer data, remember not to set BUSY bit of SPI_CTL register.
     * |        |          |The DMA controller inside SPI module will set it automatically whenever necessary.
     * |[1]     |RXPDMAEN  |Receive DMA Start
     * |        |          |Set this bit to 1 will start the receive DMA process.
     * |        |          |SPI module will issue request to DMA module automatically.
     */
    __IO uint32_t PDMACTL;               

} SPI_T;

/**
    @addtogroup SPI_CONST SPI Bit Field Definition
    Constant Definitions for SPI Controller
@{ */

#define SPI_CTL_BUSY_Pos                 (0)                                               /*!< SPI CTL: BUSY Position                 */
#define SPI_CTL_BUSY_Msk                 (0x1ul << SPI_CTL_BUSY_Pos)                       /*!< SPI CTL: BUSY Mask                     */

#define SPI_CTL_RXNEG_Pos                (1)                                               /*!< SPI CTL: RXNEG Position                */
#define SPI_CTL_RXNEG_Msk                (0x1ul << SPI_CTL_RXNEG_Pos)                      /*!< SPI CTL: RXNEG Mask                    */

#define SPI_CTL_TXNEG_Pos                (2)                                               /*!< SPI CTL: TXNEG Position                */
#define SPI_CTL_TXNEG_Msk                (0x1ul << SPI_CTL_TXNEG_Pos)                      /*!< SPI CTL: TXNEG Mask                    */

#define SPI_CTL_DWIDTH_Pos               (3)                                               /*!< SPI CTL: DWIDTH Position               */
#define SPI_CTL_DWIDTH_Msk               (0x1ful << SPI_CTL_DWIDTH_Pos)                    /*!< SPI CTL: DWIDTH Mask                   */

#define SPI_CTL_TX_NUM_Pos               (8)                                               /*!< SPI CTL: TX_NUM Position                */
#define SPI_CTL_TX_NUM_Msk               (0x3ul << SPI_CTL_TX_NUM_Pos)                      /*!< SPI CTL: TX_NUM Mask                    */

#define SPI_CTL_LSB_Pos                  (10)                                              /*!< SPI CTL: LSB Position                  */
#define SPI_CTL_LSB_Msk                  (0x1ul << SPI_CTL_LSB_Pos)                        /*!< SPI CTL: LSB Mask                      */

#define SPI_CTL_CLKP_Pos                 (11)                                              /*!< SPI CTL: CLKP Position               */
#define SPI_CTL_CLKP_Msk                 (0x1ul << SPI_CTL_CLKP_Pos)                       /*!< SPI CTL: CLKP Mask                   */

#define SPI_CTL_SUSPITV_Pos              (12)                                              /*!< SPI CTL: SUSPITV Position              */
#define SPI_CTL_SUSPITV_Msk              (0xful << SPI_CTL_SUSPITV_Pos)                    /*!< SPI CTL: SUSPITV Mask                  */

#define SPI_CTL_UNIT_INTSTS_Pos          (16)                                              /*!< SPI CTL: UNIT_INTSTS Position               */
#define SPI_CTL_UNIT_INTSTS_Msk          (0x1ul << SPI_CTL_UNIT_INTSTS_Pos)                /*!< SPI CTL: UNITIF Mask                   */

#define SPI_CTL_UNIT_INTEN_Pos           (17)                                              /*!< SPI CTL: UNITIEN Position              */
#define SPI_CTL_UNIT_INTEN_Msk           (0x1ul << SPI_CTL_UNIT_INTEN_Pos)                 /*!< SPI CTL: UNITIEN Mask                  */

#define SPI_CTL_SLAVE_Pos                (18)                                              /*!< SPI CTL: SLAVE Position                */
#define SPI_CTL_SLAVE_Msk                (0x1ul << SPI_CTL_SLAVE_Pos)                      /*!< SPI CTL: SLAVE Mask                    */

#define SPI_CTL_BYTEITV_Pos              (19)                                              /*!< SPI CTL: BYTEITV Position              */
#define SPI_CTL_BYTEITV_Msk              (0x1ul << SPI_CTL_BYTEITV_Pos)                    /*!< SPI CTL: BYTEITV Mask                  */

#define SPI_CTL_REORDER_Pos              (20)                                              /*!< SPI CTL: REORDER Position              */
#define SPI_CTL_REORDER_Msk              (0x1ul << SPI_CTL_REORDER_Pos)                    /*!< SPI CTL: REORDER Mask                  */

#define SPI_CTL_FIFOEN_Pos               (21)                                              /*!< SPI CTL: FIFOEN Position               */
#define SPI_CTL_FIFOEN_Msk               (0x1ul << SPI_CTL_FIFOEN_Pos)                     /*!< SPI CTL: FIFOEN Mask                   */

#define SPI_CTL_VARCLKEN_Pos             (23)                                              /*!< SPI CTL: VARCLKEN Position             */
#define SPI_CTL_VARCLKEN_Msk             (0x1ul << SPI_CTL_VARCLKEN_Pos)                   /*!< SPI CTL: VARCLKEN Mask                 */

#define SPI_CTL_RXEMPTY_Pos              (24)                                              /*!< SPI CTL: RXEMPTY Position              */
#define SPI_CTL_RXEMPTY_Msk              (0x1ul << SPI_CTL_RXEMPTY_Pos)                    /*!< SPI CTL: RXEMPTY Mask                  */

#define SPI_CTL_RXFULL_Pos               (25)                                              /*!< SPI CTL: RXFULL Position               */
#define SPI_CTL_RXFULL_Msk               (0x1ul << SPI_CTL_RXFULL_Pos)                     /*!< SPI CTL: RXFULL Mask                   */

#define SPI_CTL_TXEMPTY_Pos              (26)                                              /*!< SPI CTL: TXEMPTY Position              */
#define SPI_CTL_TXEMPTY_Msk              (0x1ul << SPI_CTL_TXEMPTY_Pos)                    /*!< SPI CTL: TXEMPTY Mask                  */

#define SPI_CTL_TXFULL_Pos               (27)                                              /*!< SPI CTL: TXFULL Position               */
#define SPI_CTL_TXFULL_Msk               (0x1ul << SPI_CTL_TXFULL_Pos)                     /*!< SPI CTL: TXFULL Mask                   */

#define SPI_CTL_PDMASSEN_Pos             (28)                                              /*!< SPI CTL: PDMASSEN Position             */
#define SPI_CTL_PDMASSEN_Msk             (0x1ul << SPI_CTL_PDMASSEN_Pos)                   /*!< SPI CTL: PDMASSEN Mask                 */

#define SPI_CLKDIV_DIVIDER0_Pos          (0)                                               /*!< SPI CLKDIV: DIVIDER0 Position          */
#define SPI_CLKDIV_DIVIDER0_Msk          (0xfffful << SPI_CLKDIV_DIVIDER0_Pos)             /*!< SPI CLKDIV: DIVIDER0 Mask              */

#define SPI_CLKDIV_DIVIDER1_Pos          (16)                                              /*!< SPI CLKDIV: DIVIDER1 Position          */
#define SPI_CLKDIV_DIVIDER1_Msk          (0xfffful << SPI_CLKDIV_DIVIDER1_Pos)             /*!< SPI CLKDIV: DIVIDER1 Mask              */

#define SPI_SSCTL_SS_Pos                 (0)                                               /*!< SPI SSCTL: SS Position                 */
#define SPI_SSCTL_SS_Msk                 (0x3ul << SPI_SSCTL_SS_Pos)                       /*!< SPI SSCTL: SS Mask                     */

#define SPI_SSCTL_SS_LVL_Pos             (2)                                               /*!< SPI SSCTL: SS_LVL Position           */
#define SPI_SSCTL_SS_LVL_Msk             (0x1ul << SPI_SSCTL_SS_LVL_Pos)                   /*!< SPI SSCTL: SS_LVL Mask               */

#define SPI_SSCTL_AUTOSS_Pos             (3)                                               /*!< SPI SSCTL: AUTOSS Position             */
#define SPI_SSCTL_AUTOSS_Msk             (0x1ul << SPI_SSCTL_AUTOSS_Pos)                   /*!< SPI SSCTL: AUTOSS Mask                 */

#define SPI_SSCTL_LVTRGEN_Pos            (4)                                               /*!< SPI SSCTL: LVTRGEN Position            */
#define SPI_SSCTL_LVTRGEN_Msk            (0x1ul << SPI_SSCTL_LVTRGEN_Pos)                  /*!< SPI SSCTL: LVTRGEN Mask                */

#define SPI_SSCTL_LVTRGSTS_Pos           (5)                                               /*!< SPI SSCTL: LVTRGSTS Position           */
#define SPI_SSCTL_LVTRGSTS_Msk           (0x1ul << SPI_SSCTL_LVTRGSTS_Pos)                 /*!< SPI SSCTL: LVTRGSTS Mask               */

#define SPI_RX0_RX_Pos                   (0)                                               /*!< SPI RX0: RX Position                   */
#define SPI_RX0_RX_Msk                   (0xfffffffful << SPI_RX0_RX_Pos)                  /*!< SPI RX0: RX Mask                       */

#define SPI_RX1_RX_Pos                   (0)                                               /*!< SPI RX1: RX Position                   */
#define SPI_RX1_RX_Msk                   (0xfffffffful << SPI_RX1_RX_Pos)                  /*!< SPI RX1: RX Mask                       */

#define SPI_TX0_TX_Pos                   (0)                                               /*!< SPI TX0: TX Position                   */
#define SPI_TX0_TX_Msk                   (0xfffffffful << SPI_TX0_TX_Pos)                  /*!< SPI TX0: TX Mask                       */

#define SPI_TX1_TX_Pos                   (0)                                               /*!< SPI TX1: TX Position                   */
#define SPI_TX1_TX_Msk                   (0xfffffffful << SPI_TX1_TX_Pos)                  /*!< SPI TX1: TX Mask                       */

#define SPI_VARCLK_VARCLK_Pos            (0)                                               /*!< SPI VARCLK: VARCLK Position            */
#define SPI_VARCLK_VARCLK_Msk            (0xfffffffful << SPI_VARCLK_VARCLK_Pos)           /*!< SPI VARCLK: VARCLK Mask                */

#define SPI_PDMACTL_TXPDMAEN_Pos         (0)                                               /*!< SPI PDMACTL: TXPDMAEN Position         */
#define SPI_PDMACTL_TXPDMAEN_Msk         (0x1ul << SPI_PDMACTL_TXPDMAEN_Pos)               /*!< SPI PDMACTL: TXPDMAEN Mask             */

#define SPI_PDMACTL_RXPDMAEN_Pos         (1)                                               /*!< SPI PDMACTL: RXPDMAEN Position         */
#define SPI_PDMACTL_RXPDMAEN_Msk         (0x1ul << SPI_PDMACTL_RXPDMAEN_Pos)               /*!< SPI PDMACTL: RXPDMAEN Mask             */

/**@}*/ /* SPI_CONST */
/**@}*/ /* end of SPI register group */


/*---------------------- SPI Master -------------------------*/
/**
    @addtogroup SPIM SPI Master(SPIM)
    Memory Mapped Structure for SPIM Controller
@{ */
 
typedef struct
{
    /**
     * CTL0
     * ===================================================================================================
     * Offset: 0x00  Control and Status Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXDMAEN   |RX DMA Enable Control Bit
     * |        |          |If set RXDMAEN to high, SPI interface will receive the data from slave automatically.
     * |        |          |0 = DMA Disabled.
     * |        |          |1 = DMA Enable.
     * |        |          |Note: Only support master mode.
     * |        |          |Note2: Before setting RXDMAEN, user must set PDMA register correctly first.
     * |[1]     |TXDMAEN   |TX DMA Enable Control Bit
     * |        |          |If set TXDMAEN to high, SPI interface will transfer the data to slave automatically.
     * |        |          |0 = DMA Disabled.
     * |        |          |1 = DMA Enable.
     * |        |          |Note: Only support master mode.
     * |        |          |Note2: Before setting RXDMAEN, user must set PDMA register correctly first.
     * |[6]     |IEN       |Interrupt Enable Control
     * |        |          |0 = SPIM Interrupt Disabled.
     * |        |          |1 = SPIM Interrupt Enabled.
     * |[7]     |IF        |Interrupt Flag
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Write 1 to clear.
     * |        |          |Read Operation:
     * |        |          |0 = The transfer has not finished yet.
     * |        |          |1 = The transfer has done.
     * |[12:8]  |DWIDTH    |Transmit/Receive Bit Length
     * |        |          |This field specifies how many bits are transmitted/received in one transmit/receive transaction.
     * |        |          |0x7 = 8 bits.
     * |        |          |0xF = 16 bits.
     * |        |          |0x17 = 24 bits.
     * |        |          |0x1F = 32 bits.
     * |        |          |Others = Incorrect transfer result.
     * |        |          |Note1: Only used for I/O mode.
     * |        |          |Note2: Only 8-, 16-, 24-, and 32-bit are allowed. Other bit length will result in incorrect transfer.
     * |[14:13] |BURSTNUM  |Transmit/Receive Burst Number
     * |        |          |This field specifies how many transmit/receive transactions should be executed continuously in one transfer.
     * |        |          |00 = Only one transmit/receive transaction will be executed in one transfer.
     * |        |          |01 = Two successive transmit/receive transactions will be executed in one transfer.
     * |        |          |10 = Three successive transmit/receive transactions will be executed in one transfer.
     * |        |          |11 = Four successive transmit/receive transactions will be executed in one transfer.
     * |        |          |Note: Only used for I/O Mode.
     * |[15]    |QDIODIR   |SPI Interface Direction Select For Quad/Dual Mode
     * |        |          |0 = Interface signals are input.
     * |        |          |1 = Interface signals are output.
     * |        |          |Note: Only used for I/O mode.
     * |[19:16] |SUSPITV   |Suspend Interval
     * |        |          |These four bits provide the configuration of suspend interval between two successive transmit/receive transactions in a transfer.
     * |        |          |The default value is 0x00.
     * |        |          |When BURSTNUM = 00, setting this field has no effect on transfer.
     * |        |          |The desired interval is obtained according to the following equation (from the last falling edge of current sclk to the first rising edge of next sclk):
     * |        |          |(SUSPITV+2)*period of SCLK
     * |        |          |0x0 = 2 SCLK clock cycles.
     * |        |          |0x1 = 3 SCLK clock cycles.
     * |        |          |......
     * |        |          |0xE = 16 SCLK clock cycles.
     * |        |          |0xF = 17 SCLK clock cycles.
     * |        |          |Note: Only used for I/O mode.
     * |[21:20] |BITMODE   |SPI Interface Bit Mode
     * |        |          |00 = Standard mode.
     * |        |          |01 = Dual mode.
     * |        |          |10 = Quad mode.
     * |        |          |11 = Reserved.
     * |        |          |Note: Only used for I/O mode.
     */
    __IO uint32_t CTL0;                  

    /**
     * CTL1
     * ===================================================================================================
     * Offset: 0x04  Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SPIMEN    |Go And Busy Status
     * |        |          |Write Operation:
     * |        |          |0 = No effect.
     * |        |          |1 = Start the transfer.
     * |        |          |This bit remains set during the transfer and is automatically cleared after transfer finishe.
     * |        |          |Read Operation:
     * |        |          |0 = The transfer has done.
     * |        |          |1 = The transfer has not finished yet.
     * |        |          |Note: All registers should be set before writing 1 to the SPIMEN bit.
     * |        |          |When a transfer is in progress, you should not write to any register of this periphera.
     * |[4]     |SS        |Slave Select Active Enable Control
     * |        |          |0 = SPIM_SS is in active level.
     * |        |          |1 = SPIM_SS is in inactive level.
     * |        |          |Note: This interface can only drive one device/slave at a given time.
     * |        |          |Therefore, the slave selects of the selected device must be set to its active level before starting any read or write transfe.
     * |[5]     |SSACTPOL  |Slave Select Active Level
     * |        |          |It defines the active level of device/slave select signal (SPIM_SS).
     * |        |          |0 = The SPIM_SS slave select signal is Active Low.
     * |        |          |1 = The SPIM_SS slave select signal is Active High.
     * |[14:12] |DLYSEL    |RX Sample Clock Source Delay Chain Select
     * |        |          |000 = Not Delay.
     * |        |          |001 = Select sample clock through 2 Delay Cell.
     * |        |          |010 = Select sample clock through 4 Delay Cell.
     * |        |          |011 = Select sample clock through 6 Delay Cell.
     * |        |          |...
     * |        |          |111 = Select sample clock through 14 Delay Cell.
     * |[31:16] |DIVIDER   |Clock Divider Register
     * |        |          |The value in this field is the frequency divider of the system clock to generate the serial clock on the output SPIM_CLK pin.
     * |        |          |The desired frequency is obtained according to the following equation:
     * |        |          |Note: When set DIVIDER to zero, the frequency of SPIM_CLK will be equal to the frequency of SYS_CLK.
     */
    __IO uint32_t CTL1;                                   

         uint32_t RESERVE0[2];

    /**
     * RX0
     * ===================================================================================================
     * Offset: 0x10  Data Receive Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register
     * |        |          |The Data Receive Registers hold the received data of the last executed transfer.
     * |        |          |Number of valid RX registers is specified in SPIM_CTL0[BURSTNUM].
     * |        |          |If BURSTNUM > 0, received data are held in the most significant RX register firs.
     * |        |          |Number of valid-bit is specified in SPIM_CTL0[DWIDTH].
     * |        |          |If DWIDTH is 16, 24, or 32, received data are held in the least significant byte of RX register firs.
     * |        |          |In a byte, received data are held in the most significant bit of RX register first.
     * |        |          |Example 1: If SPIM_CTL0[BURSTNUM] = 0x3 and SPIM_CTL1[DWIDTH] = 0x17, received data will be held in the order SPIM_RX3[23:0], SPIM_RX2[23:0], SPIM_RX1[23:0], SPIM_RX0[23:0].
     * |        |          |Example 2: If SPIM_CTL0[BURSTNUM = 0x0 and SPIM_CTL0[DWIDTH] = 0x17, received data will be held in the order SPIM_RX0[7:0], SPIM_RX0[15:8], SPIM_RX0[23:16].
     * |        |          |Example 3: If SPIM_CTL0[BURSTNUM = 0x0 and SPIM_CTL0[DWIDTH] = 0x07, received data will be held in the order SPIM_RX0[7], SPIM_RX0[6], ..., SPIM_RX0[0].
     */
    __I  uint32_t RX0;                   

    /**
     * RX1
     * ===================================================================================================
     * Offset: 0x14  Data Receive Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register
     * |        |          |The Data Receive Registers hold the received data of the last executed transfer.
     * |        |          |Number of valid RX registers is specified in SPIM_CTL0[BURSTNUM].
     * |        |          |If BURSTNUM > 0, received data are held in the most significant RX register firs.
     * |        |          |Number of valid-bit is specified in SPIM_CTL0[DWIDTH].
     * |        |          |If DWIDTH is 16, 24, or 32, received data are held in the least significant byte of RX register firs.
     * |        |          |In a byte, received data are held in the most significant bit of RX register first.
     * |        |          |Example 1: If SPIM_CTL0[BURSTNUM] = 0x3 and SPIM_CTL1[DWIDTH] = 0x17, received data will be held in the order SPIM_RX3[23:0], SPIM_RX2[23:0], SPIM_RX1[23:0], SPIM_RX0[23:0].
     * |        |          |Example 2: If SPIM_CTL0[BURSTNUM = 0x0 and SPIM_CTL0[DWIDTH] = 0x17, received data will be held in the order SPIM_RX0[7:0], SPIM_RX0[15:8], SPIM_RX0[23:16].
     * |        |          |Example 3: If SPIM_CTL0[BURSTNUM = 0x0 and SPIM_CTL0[DWIDTH] = 0x07, received data will be held in the order SPIM_RX0[7], SPIM_RX0[6], ..., SPIM_RX0[0].
     */
    __I  uint32_t RX1;                   

    /**
     * RX2
     * ===================================================================================================
     * Offset: 0x18  Data Receive Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register
     * |        |          |The Data Receive Registers hold the received data of the last executed transfer.
     * |        |          |Number of valid RX registers is specified in SPIM_CTL0[BURSTNUM].
     * |        |          |If BURSTNUM > 0, received data are held in the most significant RX register firs.
     * |        |          |Number of valid-bit is specified in SPIM_CTL0[DWIDTH].
     * |        |          |If DWIDTH is 16, 24, or 32, received data are held in the least significant byte of RX register firs.
     * |        |          |In a byte, received data are held in the most significant bit of RX register first.
     * |        |          |Example 1: If SPIM_CTL0[BURSTNUM] = 0x3 and SPIM_CTL1[DWIDTH] = 0x17, received data will be held in the order SPIM_RX3[23:0], SPIM_RX2[23:0], SPIM_RX1[23:0], SPIM_RX0[23:0].
     * |        |          |Example 2: If SPIM_CTL0[BURSTNUM = 0x0 and SPIM_CTL0[DWIDTH] = 0x17, received data will be held in the order SPIM_RX0[7:0], SPIM_RX0[15:8], SPIM_RX0[23:16].
     * |        |          |Example 3: If SPIM_CTL0[BURSTNUM = 0x0 and SPIM_CTL0[DWIDTH] = 0x07, received data will be held in the order SPIM_RX0[7], SPIM_RX0[6], ..., SPIM_RX0[0].
     */
    __I  uint32_t RX2;                   

    /**
     * RX3
     * ===================================================================================================
     * Offset: 0x1C  Data Receive Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register
     * |        |          |The Data Receive Registers hold the received data of the last executed transfer.
     * |        |          |Number of valid RX registers is specified in SPIM_CTL0[BURSTNUM].
     * |        |          |If BURSTNUM > 0, received data are held in the most significant RX register firs.
     * |        |          |Number of valid-bit is specified in SPIM_CTL0[DWIDTH].
     * |        |          |If DWIDTH is 16, 24, or 32, received data are held in the least significant byte of RX register firs.
     * |        |          |In a byte, received data are held in the most significant bit of RX register first.
     * |        |          |Example 1: If SPIM_CTL0[BURSTNUM] = 0x3 and SPIM_CTL1[DWIDTH] = 0x17, received data will be held in the order SPIM_RX3[23:0], SPIM_RX2[23:0], SPIM_RX1[23:0], SPIM_RX0[23:0].
     * |        |          |Example 2: If SPIM_CTL0[BURSTNUM = 0x0 and SPIM_CTL0[DWIDTH] = 0x17, received data will be held in the order SPIM_RX0[7:0], SPIM_RX0[15:8], SPIM_RX0[23:16].
     * |        |          |Example 3: If SPIM_CTL0[BURSTNUM = 0x0 and SPIM_CTL0[DWIDTH] = 0x07, received data will be held in the order SPIM_RX0[7], SPIM_RX0[6], ..., SPIM_RX0[0].
     */
    __I  uint32_t RX3;                   

    /**
     * TX0
     * ===================================================================================================
     * Offset: 0x20  Data Transmit Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The Data Transmit Registers hold the data to be transmitted in next transfer.
     * |        |          |Number of valid TX registers is specified in SPIM_CTL0[BURSTNUM].
     * |        |          |If BURSTNUM > 0, data are transmitted in the most significant TX register firs.
     * |        |          |Number of valid-bit is specified in SPIM_CTL0[DWIDTH].
     * |        |          |If DWIDTH is 16, 24, or 32, data are transmitted in the least significant byte of TX register firs.
     * |        |          |In a byte, data are transmitted in the most significant bit of TX register first.
     * |        |          |Example 1: If SPIM_CTL0[BURSTNUM] = 0x3 and SPIM_CTL1[DWIDTH] = 0x17, data will be transmitted in the order SPIM_TX3[23:0], SPIM_TX2[23:0], SPIM_TX1[23:0], SPIM_TX0[23:0] in next transfer.
     * |        |          |Example 2: If SPIM_CTL0[BURSTNUM] = 0x0 and SPIM_CTL0[DWIDTH] = 0x17, data will be transmitted in the order SPIM_TX0[7:0], SPIM_TX0[15:8], SPIM_TX0[23:16] in next transfer.
     * |        |          |Example 3: If SPIM_CTL0[BURSTNUM] = 0x0 and SPIM_CTL0[DWIDTH] = 0x07, data will be transmitted in the order SPIM_TX0[7], SPIM_TX0[6], ..., SPIM_TX0[0] in next transfer.
     */
    __IO uint32_t TX0;                   

    /**
     * TX1
     * ===================================================================================================
     * Offset: 0x24  Data Transmit Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The Data Transmit Registers hold the data to be transmitted in next transfer.
     * |        |          |Number of valid TX registers is specified in SPIM_CTL0[BURSTNUM].
     * |        |          |If BURSTNUM > 0, data are transmitted in the most significant TX register firs.
     * |        |          |Number of valid-bit is specified in SPIM_CTL0[DWIDTH].
     * |        |          |If DWIDTH is 16, 24, or 32, data are transmitted in the least significant byte of TX register firs.
     * |        |          |In a byte, data are transmitted in the most significant bit of TX register first.
     * |        |          |Example 1: If SPIM_CTL0[BURSTNUM] = 0x3 and SPIM_CTL1[DWIDTH] = 0x17, data will be transmitted in the order SPIM_TX3[23:0], SPIM_TX2[23:0], SPIM_TX1[23:0], SPIM_TX0[23:0] in next transfer.
     * |        |          |Example 2: If SPIM_CTL0[BURSTNUM] = 0x0 and SPIM_CTL0[DWIDTH] = 0x17, data will be transmitted in the order SPIM_TX0[7:0], SPIM_TX0[15:8], SPIM_TX0[23:16] in next transfer.
     * |        |          |Example 3: If SPIM_CTL0[BURSTNUM] = 0x0 and SPIM_CTL0[DWIDTH] = 0x07, data will be transmitted in the order SPIM_TX0[7], SPIM_TX0[6], ..., SPIM_TX0[0] in next transfer.
     */
    __IO uint32_t TX1;                   

    /**
     * TX2
     * ===================================================================================================
     * Offset: 0x28  Data Transmit Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The Data Transmit Registers hold the data to be transmitted in next transfer.
     * |        |          |Number of valid TX registers is specified in SPIM_CTL0[BURSTNUM].
     * |        |          |If BURSTNUM > 0, data are transmitted in the most significant TX register firs.
     * |        |          |Number of valid-bit is specified in SPIM_CTL0[DWIDTH].
     * |        |          |If DWIDTH is 16, 24, or 32, data are transmitted in the least significant byte of TX register firs.
     * |        |          |In a byte, data are transmitted in the most significant bit of TX register first.
     * |        |          |Example 1: If SPIM_CTL0[BURSTNUM] = 0x3 and SPIM_CTL1[DWIDTH] = 0x17, data will be transmitted in the order SPIM_TX3[23:0], SPIM_TX2[23:0], SPIM_TX1[23:0], SPIM_TX0[23:0] in next transfer.
     * |        |          |Example 2: If SPIM_CTL0[BURSTNUM] = 0x0 and SPIM_CTL0[DWIDTH] = 0x17, data will be transmitted in the order SPIM_TX0[7:0], SPIM_TX0[15:8], SPIM_TX0[23:16] in next transfer.
     * |        |          |Example 3: If SPIM_CTL0[BURSTNUM] = 0x0 and SPIM_CTL0[DWIDTH] = 0x07, data will be transmitted in the order SPIM_TX0[7], SPIM_TX0[6], ..., SPIM_TX0[0] in next transfer.
     */
    __IO uint32_t TX2;                   

    /**
     * TX3
     * ===================================================================================================
     * Offset: 0x2C  Data Transmit Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The Data Transmit Registers hold the data to be transmitted in next transfer.
     * |        |          |Number of valid TX registers is specified in SPIM_CTL0[BURSTNUM].
     * |        |          |If BURSTNUM > 0, data are transmitted in the most significant TX register firs.
     * |        |          |Number of valid-bit is specified in SPIM_CTL0[DWIDTH].
     * |        |          |If DWIDTH is 16, 24, or 32, data are transmitted in the least significant byte of TX register firs.
     * |        |          |In a byte, data are transmitted in the most significant bit of TX register first.
     * |        |          |Example 1: If SPIM_CTL0[BURSTNUM] = 0x3 and SPIM_CTL1[DWIDTH] = 0x17, data will be transmitted in the order SPIM_TX3[23:0], SPIM_TX2[23:0], SPIM_TX1[23:0], SPIM_TX0[23:0] in next transfer.
     * |        |          |Example 2: If SPIM_CTL0[BURSTNUM] = 0x0 and SPIM_CTL0[DWIDTH] = 0x17, data will be transmitted in the order SPIM_TX0[7:0], SPIM_TX0[15:8], SPIM_TX0[23:16] in next transfer.
     * |        |          |Example 3: If SPIM_CTL0[BURSTNUM] = 0x0 and SPIM_CTL0[DWIDTH] = 0x07, data will be transmitted in the order SPIM_TX0[7], SPIM_TX0[6], ..., SPIM_TX0[0] in next transfer.
     */
    __IO uint32_t TX3;                               

} SPIM_T;

/**
    @addtogroup SPIM_CONST SPIM Bit Field Definition
    Constant Definitions for SPIM Controller
@{ */

#define SPIM_CTL0_RXDMAEN_Pos            (0)                                               /*!< SPIM_T::CTL0: RXDMAEN Position            */
#define SPIM_CTL0_RXDMAEN_Msk            (0x1ul << SPIM_CTL0_RXDMAEN_Pos)                  /*!< SPIM_T::CTL0: RXDMAEN Mask                */

#define SPIM_CTL0_TXDMAEN_Pos            (1)                                               /*!< SPIM_T::CTL0: TXDMAEN Position              */
#define SPIM_CTL0_TXDMAEN_Msk            (0x1ul << SPIM_CTL0_TXDMAEN_Pos)                  /*!< SPIM_T::CTL0: TXDMAEN Mask                  */

#define SPIM_CTL0_IEN_Pos                (6)                                               /*!< SPIM_T::CTL0: IEN Position                */
#define SPIM_CTL0_IEN_Msk                (0x1ul << SPIM_CTL0_IEN_Pos)                      /*!< SPIM_T::CTL0: IEN Mask                    */

#define SPIM_CTL0_IF_Pos                 (7)                                               /*!< SPIM_T::CTL0: IF Position                 */
#define SPIM_CTL0_IF_Msk                 (0x1ul << SPIM_CTL0_IF_Pos)                       /*!< SPIM_T::CTL0: IF Mask                     */

#define SPIM_CTL0_DWIDTH_Pos             (8)                                               /*!< SPIM_T::CTL0: DWIDTH Position             */
#define SPIM_CTL0_DWIDTH_Msk             (0x1ful << SPIM_CTL0_DWIDTH_Pos)                  /*!< SPIM_T::CTL0: DWIDTH Mask                 */

#define SPIM_CTL0_BURSTNUM_Pos           (13)                                              /*!< SPIM_T::CTL0: BURSTNUM Position           */
#define SPIM_CTL0_BURSTNUM_Msk           (0x3ul << SPIM_CTL0_BURSTNUM_Pos)                 /*!< SPIM_T::CTL0: BURSTNUM Mask               */

#define SPIM_CTL0_QDIODIR_Pos            (15)                                              /*!< SPIM_T::CTL0: QDIODIR Position            */
#define SPIM_CTL0_QDIODIR_Msk            (0x1ul << SPIM_CTL0_QDIODIR_Pos)                  /*!< SPIM_T::CTL0: QDIODIR Mask                */

#define SPIM_CTL0_SUSPITV_Pos            (16)                                              /*!< SPIM_T::CTL0: SUSPITV Position            */
#define SPIM_CTL0_SUSPITV_Msk            (0xful << SPIM_CTL0_SUSPITV_Pos)                  /*!< SPIM_T::CTL0: SUSPITV Mask                */

#define SPIM_CTL0_BITMODE_Pos            (20)                                              /*!< SPIM_T::CTL0: BITMODE Position            */
#define SPIM_CTL0_BITMODE_Msk            (0x3ul << SPIM_CTL0_BITMODE_Pos)                  /*!< SPIM_T::CTL0: BITMODE Mask                */

#define SPIM_CTL1_SPIMEN_Pos             (0)                                               /*!< SPIM_T::CTL1: SPIMEN Position             */
#define SPIM_CTL1_SPIMEN_Msk             (0x1ul << SPIM_CTL1_SPIMEN_Pos)                   /*!< SPIM_T::CTL1: SPIMEN Mask                 */

#define SPIM_CTL1_SS_Pos                 (4)                                               /*!< SPIM_T::CTL1: SS Position                 */
#define SPIM_CTL1_SS_Msk                 (0x1ul << SPIM_CTL1_SS_Pos)                       /*!< SPIM_T::CTL1: SS Mask                     */

#define SPIM_CTL1_SSACTPOL_Pos           (5)                                               /*!< SPIM_T::CTL1: SSACTPOL Position           */
#define SPIM_CTL1_SSACTPOL_Msk           (0x1ul << SPIM_CTL1_SSACTPOL_Pos)                 /*!< SPIM_T::CTL1: SSACTPOL Mask               */

#define SPIM_CTL1_DLYSEL_Pos             (12)                                              /*!< SPIM_T::CTL1: DLYSEL Position         */
#define SPIM_CTL1_DLYSEL_Msk             (0x7ul << SPIM_CTL1_DLYSEL_Pos)                   /*!< SPIM_T::CTL1: DLYSEL Mask             */

#define SPIM_CTL1_DIVIDER_Pos            (16)                                              /*!< SPIM_T::CTL1: DIVIDER Position            */
#define SPIM_CTL1_DIVIDER_Msk            (0xfffful << SPIM_CTL1_DIVIDER_Pos)               /*!< SPIM_T::CTL1: DIVIDER Mask                */

#define SPIM_RX0_RX_Pos                  (0)                                               /*!< SPIM_T::RX0: RX Position                  */
#define SPIM_RX0_RX_Msk                  (0xfffffffful << SPIM_RX0_RX_Pos)                 /*!< SPIM_T::RX0: RX Mask                      */

#define SPIM_RX1_RX_Pos                  (0)                                               /*!< SPIM_T::RX1: RX Position                  */
#define SPIM_RX1_RX_Msk                  (0xfffffffful << SPIM_RX1_RX_Pos)                 /*!< SPIM_T::RX1: RX Mask                      */

#define SPIM_RX2_RX_Pos                  (0)                                               /*!< SPIM_T::RX2: RX Position                  */
#define SPIM_RX2_RX_Msk                  (0xfffffffful << SPIM_RX2_RX_Pos)                 /*!< SPIM_T::RX2: RX Mask                      */

#define SPIM_RX3_RX_Pos                  (0)                                               /*!< SPIM_T::RX3: RX Position                  */
#define SPIM_RX3_RX_Msk                  (0xfffffffful << SPIM_RX3_RX_Pos)                 /*!< SPIM_T::RX3: RX Mask                      */

#define SPIM_TX0_TX_Pos                  (0)                                               /*!< SPIM_T::TX0: TX Position                  */
#define SPIM_TX0_TX_Msk                  (0xfffffffful << SPIM_TX0_TX_Pos)                 /*!< SPIM_T::TX0: TX Mask                      */

#define SPIM_TX1_TX_Pos                  (0)                                               /*!< SPIM_T::TX1: TX Position                  */
#define SPIM_TX1_TX_Msk                  (0xfffffffful << SPIM_TX1_TX_Pos)                 /*!< SPIM_T::TX1: TX Mask                      */

#define SPIM_TX2_TX_Pos                  (0)                                               /*!< SPIM_T::TX2: TX Position                  */
#define SPIM_TX2_TX_Msk                  (0xfffffffful << SPIM_TX2_TX_Pos)                 /*!< SPIM_T::TX2: TX Mask                      */

#define SPIM_TX3_TX_Pos                  (0)                                               /*!< SPIM_T::TX3: TX Position                  */
#define SPIM_TX3_TX_Msk                  (0xfffffffful << SPIM_TX3_TX_Pos)                 /*!< SPIM_T::TX3: TX Mask                      */

/**@}*/ /* SPIM_CONST */
/**@}*/ /* end of SPIM register group */


/*---------------------- Audio Class D Speaker Driver -------------------------*/
/**
    @addtogroup DPWM Audio Class D Speaker Driver(DPWM)
    Memory Mapped Structure for DPWM Controller
@{ */
 
typedef struct
{
    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  DPWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:2]   |MODUFRQ   |DPWM Modulation Frequency
     * |        |          |This parameter controls the carrier modulation frequency of the PWM signal as a proportion of DPWM_CLK.
     * |        |          |MODUFRQ : DPWM_CLK Division : Frequency for DPWM_CLK = 98.304MHZ
     * |        |          |0 : 228 : 431158
     * |        |          |1 : 156 : 630154
     * |        |          |2 : 76 : 1293474
     * |        |          |3 : 52 : 1890462
     * |        |          |4 : 780 : 126031
     * |        |          |5 : 524 : 187603
     * |        |          |6 : 396 : 248242
     * |        |          |7 : 268 : 366806
     * |[3]     |DEADTIME  |DPWM Driver Deadtime Control
     * |        |          |Enabling this bit will insert an additional clock cycle deadtime into the switching of PMOS and NMOS driver transistors.
     * |[4:5]   |DITHEREN  |DPWM Signal Dither Control
     * |        |          |To prevent structured noise on PWM output due to DC offsets in the input signal it is possible to add random dither to the PWM signal.
     * |        |          |These bits control the dither:.
     * |        |          |0 = No dither.
     * |        |          |1 = +/- 1 bit dither
     * |        |          |3 = +/- 2 bit dither
     * |[6]     |DPWMEN    |DPWM Enable
     * |        |          |0= Disable DPWM, SPK pins are tri-state, CIC filter is reset, FIFO pointers are reset (FIFO data is not reset).
     * |        |          |1= Enable DPWM, SPK pins are enabled and driven, data is taken from FIFO.
     * |[7]     |CLIPIE    |Fix Flag by CYFang
	 * |[8]     |RXTHIE    |Fix Flag by CYFang
	 * |[9:12]  |RXTH      |Fix Flag by CYFang
	 * |[14]    |ZCIE      |Fix Flag by CYFang
	 * |[16]    |DAC_EN    |Fix Flag by CYFang
	 * |[17]    |DAC_EN_10BIT|Fix Flag by CYFang
	 * |[18]    |DAC_PD    |Fix Flag by CYFang
	 * |[19]    |DACBUFBYPASS|Fix Flag by CYFang
	 * |[20]    |DACBUF_PD |Fix Flag by CYFang
	 * |[21]    |DAC_INSEL |Fix Flag by CYFang
	 */
    __IO uint32_t CTL;                   

    /**
     * STS
     * ===================================================================================================
     * Offset: 0x04  DPWM FIFO Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FULL      |FIFO Full
     * |        |          |0 = FIFO is not full.
     * |        |          |1 = FIFO is full.
     * |[1]     |EMPTY     |FIFO Empty
     * |        |          |0= FIFO is not empty
     * |        |          |1= FIFO is empty
     * |[2]     |RXTHF     |DPWM FIFO threshold Interrupt Status.
     * |        |          |0= The valid data count within the DPWM FIFO buffer is larger than the setting value of RXTH.
     * |        |          |1= The valid data count within the transmit FIFO buffer is less than or equal to the setting value of RXTH.
     * |[2]     |RXTHF     |DPWM FIFO Pointer.
     * |        |          |The FULL bit and FIFO_POINTER[3:0] indicates the field that the valid data count within the DPWM FIFO buffer.
     * |        |          |The Maximum value shown in FIFO_POINTER is 15. 
	 * |        |          |When the using level of DPWM FIFO Buffer equal to 16, The FULL bit is set to 1.
     */
    __I  uint32_t STS;                   

    /**
     * DMACTL
     * ===================================================================================================
     * Offset: 0x08  DPWM PDMA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DMAEN     |Enable DPWM DMA Interface
     * |        |          |0= Disable PDMA. No requests will be made to PDMA controller.
     * |        |          |1= Enable PDMA. Block will request data from PDMA controller whenever FIFO is not empty.
     */
    __IO uint32_t DMACTL;                

    /**
     * DATA
     * ===================================================================================================
     * Offset: 0x0C  DPWM FIFO Input
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |INDATA    |DPWM FIFO Audio Data Input
     * |        |          |A write to this register pushes data onto the DPWM FIFO and increments the write pointer.
     * |        |          |This is the address that PDMA writes audio data to.
     */
    __O  uint32_t DATA;                  

    /**
     * ZOHDIV
     * ===================================================================================================
     * Offset: 0x10  DPWM Zero Order Hold Division Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |ZOHDIV    |DPWM Zero Order Hold, Down-Sampling Divisor
     * |        |          |The input sample rate of the DPWM is set by HCLK frequency and the divisor set in this register by the following formula:
     * |        |          |Fs = HCLK/ZOHDIV/64
     * |        |          |Valid range is 1 to 255.
     * |        |          |Default is 48, which gives a sample rate of 16kHz for a 49.152MHz (default) HCLK.
     * |[8:15]  |GAIN      |(GAIN[7:0]+1)/256.
     * |[16:21] |DAC_DIV   |DAC clock divider
     * |        |          |DAC data sample rate is set by this divider.
     * |        |          |Fs = (HCLK/ZOH_DIV)/(DAC_DIV+1)
     * |        |          |Note : by DAC_DIV == 0x0, Sample rate is decided by ZOH_DIV
     */
    __IO uint32_t ZOHDIV;                

} DPWM_T;

/**
    @addtogroup DPWM_CONST DPWM Bit Field Definition
    Constant Definitions for DPWM Controller
@{ */

#define DPWM_CTL_MODUFRQ_Pos             (0)                                               /*!< DPWM CTL: MODUFRQ Position             */
#define DPWM_CTL_MODUFRQ_Msk             (0x7ul << DPWM_CTL_MODUFRQ_Pos)                   /*!< DPWM CTL: MODUFRQ Mask                 */

#define DPWM_CTL_DEADTIME_Pos            (3)                                               /*!< DPWM CTL: DEADTIME Position            */
#define DPWM_CTL_DEADTIME_Msk            (0x1ul << DPWM_CTL_DEADTIME_Pos)                  /*!< DPWM CTL: DEADTIME Mask                */

#define DPWM_CTL_DITHEREN_Pos            (4)                                               /*!< DPWM CTL: DITHEREN Position            */
#define DPWM_CTL_DITHEREN_Msk            (0x3ul << DPWM_CTL_DITHEREN_Pos)                  /*!< DPWM CTL: DITHEREN Mask                */

#define DPWM_CTL_DPWMEN_Pos              (6)                                               /*!< DPWM CTL: DPWMEN Position              */
#define DPWM_CTL_DPWMEN_Msk              (0x1ul << DPWM_CTL_DPWMEN_Pos)                    /*!< DPWM CTL: DPWMEN Mask                  */

/*
Fix Flag by CYFang
*/

#define DPWM_STS_FULL_Pos                (0)                                               /*!< DPWM STS: FULL Position                */
#define DPWM_STS_FULL_Msk                (0x1ul << DPWM_STS_FULL_Pos)                      /*!< DPWM STS: FULL Mask                    */

#define DPWM_STS_EMPTY_Pos               (1)                                               /*!< DPWM STS: EMPTY Position               */
#define DPWM_STS_EMPTY_Msk               (0x1ul << DPWM_STS_EMPTY_Pos)                     /*!< DPWM STS: EMPTY Mask                   */

/*
Fix Flag by CYFang
*/

#define DPWM_DMACTL_DMAEN_Pos            (0)                                               /*!< DPWM DMACTL: DMAEN Position            */
#define DPWM_DMACTL_DMAEN_Msk            (0x1ul << DPWM_DMACTL_DMAEN_Pos)                  /*!< DPWM DMACTL: DMAEN Mask                */

#define DPWM_DATA_INDATA_Pos             (0)                                               /*!< DPWM DATA: INDATA Position             */
#define DPWM_DATA_INDATA_Msk             (0xfffful << DPWM_DATA_INDATA_Pos)                /*!< DPWM DATA: INDATA Mask                 */

#define DPWM_ZOHDIV_ZOHDIV_Pos           (0)                                               /*!< DPWM ZOHDIV: ZOHDIV Position           */
#define DPWM_ZOHDIV_ZOHDIV_Msk           (0xfful << DPWM_ZOHDIV_ZOHDIV_Pos)                /*!< DPWM ZOHDIV: ZOHDIV Mask               */

#define DPWM_ZOHDIV_GAIN_Pos             (8)                                               /*!< DPWM ZOHDIV: GAIN Position           */
#define DPWM_ZOHDIV_GAIN_Msk             (0xfful << DPWM_ZOHDIV_GAIN_Pos)                  /*!< DPWM ZOHDIV: GAIN Mask               */

#define DPWM_ZOHDIV_DACDIV_Pos           (16)                                               /*!< DPWM ZOHDIV: DACDIV Position           */
#define DPWM_ZOHDIV_DACDIV_Msk           (0x3ful << DPWM_ZOHDIV_DACDIV_Pos)                 /*!< DPWM ZOHDIV: DACDIV Mask               */

/**@}*/ /* DPWM_CONST */
/**@}*/ /* end of DPWM register group */


/*---------------------- Peripheral Direct Memory Access Controller -------------------------*/
/**
    @addtogroup PDMA Peripheral Direct Memory Access Controller(PDMA)
    Memory Mapped Structure for PDMA Controller
@{ */
 
typedef struct
{
    /**
     * DSCT0_CTL
     * ===================================================================================================
     * Offset: 0x00  PDMA Control Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHEN      |PDMA Channel Enable
     * |        |          |Setting this bit to 1 enables PDMA's operation.
     * |        |          |If this bit is cleared, PDMA will ignore all PDMA request and force Bus Master into IDLE state.
     * |        |          |Note: SWRST will clear this bit.
     * |[1]     |SWRST     |Software Engine Reset
     * |        |          |0 = Writing 0 to this bit has no effect.
     * |        |          |1 = Writing 1 to this bit will reset the internal state machine and pointers.
     * |        |          |The contents of the control register will not be cleared.
     * |        |          |This bit will auto clear after a few clock cycles.
     * |[2:3]   |MODESEL   |PDMA Mode Select
     * |        |          |This parameter selects to transfer direction of the PDMA channel. Possible values are:
     * |        |          |00 = Memory to Memory mode (SRAM-to-SRAM).
     * |        |          |01 = IP to Memory mode (APB-to-SRAM).
     * |        |          |10 = Memory to IP mode (SRAM-to-APB).
     * |[4:5]   |SASEL     |Source Address Select
     * |        |          |This parameter determines the behavior of the current source address register with each PDMA transfer.
     * |        |          |It can either be fixed, incremented or wrapped.
     * |        |          |00 = Transfer Source address is incremented.
     * |        |          |01 = RESERVED.
     * |        |          |10 = Transfer Source address is fixed
     * |        |          |11 = Transfer Source address is wrapped.
     * |        |          |When PDMA_CURBCCH (Current Byte Count) equals zero, the PDMA_CURSACH (Current Source Address) and PDMA_CURBCCH registers will be reloaded from the SAR (Source Address) and PDMA_TXBCCH (Byte Count) registers automatically and PDMA will start another transfer.
     * |        |          |Cycle continues until software sets PDMA_EN = 0.
     * |        |          |When PDMA_EN is disabled, the PDMA will complete the active transfer but the remaining data in the SBUF will not be transferred to the destination address.
     * |[6:7]   |DASEL     |Destination Address Select
     * |        |          |This parameter determines the behavior of the current destination address register with each PDMA transfer.
     * |        |          |It can either be fixed, incremented or wrapped.
     * |        |          |00 = Transfer Destination Address is incremented.
     * |        |          |01 = RESERVED.
     * |        |          |10 = Transfer Destination Address is fixed (Used when data transferred from multiple addresses to a single destination such as peripheral FIFO input).
     * |        |          |11 = Transfer Destination Address is wrapped.
     * |        |          |When PDMA_CURBCCH (Current Byte Count) equals zero, the PDMA_CURDACH (Current Destination Address) and PDMA_CURBCCH registers will be reloaded from the PDMA_DSCTn_ENDDA (Destination Address) and PDMA_TXBCCHn (Byte Count) registers automatically and PDMA will start another transfer.
     * |        |          |Cycle continues until software sets PDMA_EN=0.
     * |        |          |When PDMA_EN is disabled, the PDMA will complete the active transfer but the remaining data in the SBUF will not be transferred to the destination address.
     * |[12:15] |WAINTSEL  |Wrap Interrupt Select
     * |        |          |x1xx: If this bit is set, and wraparound mode is in operation a Wrap Interrupt can be generated when half each PDMA transfer is complete.
     * |        |          |For example if BYTECNT = 32 then an interrupt could be generated when 16 bytes were sent.
     * |        |          |xxx1: If this bit is set, and wraparound mode is in operation a Wrap Interrupt can be generated when each PDMA transfer is wrapped.
     * |        |          |For example if BYTECNT = 32 then an interrupt could be generated when 32 bytes were sent and PDMA wraps around.
     * |        |          |x1x1: Both half and w interrupts generated.
     * |[19:20] |TXWIDTH   |Peripheral Transfer Width Select
     * |        |          |This parameter determines the data width to be transferred each PDMA transfer operation.
     * |        |          |00 = One word (32 bits) is transferred for every PDMA operation.
     * |        |          |01 = One byte (8 bits) is transferred for every PDMA operation.
     * |        |          |10 = One half-word (16 bits) is transferred for every PDMA operation.
     * |        |          |11 = RESERVED.
     * |        |          |Note: This field is meaningful only when MODESEL is IP to Memory mode (APB-to-Memory) or Memory to IP mode (Memory-to-APB).
     * |[23]    |TXEN      |Trigger Enable - Start a PDMA operation
     * |        |          |0 = Write: no effect. Read: Idle/Finished.
     * |        |          |1 = Enable PDMA data read or write transfer.
     * |        |          |Note: When PDMA transfer completed, this bit will be cleared automatically.
     * |        |          |If a bus error occurs, all PDMA transfer will be stopped.
     * |        |          |Software must reset PDMA channel, and then trigger again.
 */
    __IO uint32_t DSCT_CTL;             

    /**
     * DSCT0_ENDSA
     * ===================================================================================================
     * Offset: 0x04  PDMA Transfer Source Address Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |ENDSA     |PDMA Transfer Source Address Register
     * |        |          |This register holds the initial Source Address of PDMA transfer.
     * |        |          |Note: The source address must be word aligned.
 */
    __IO uint32_t DSCT_ENDSA;           

    /**
     * DSCT0_ENDDA
     * ===================================================================================================
     * Offset: 0x08  PDMA Transfer Destination Address Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |ENDDA     |PDMA Transfer Destination Address Register
     * |        |          |This register holds the initial Destination Address of PDMA transfer.
     * |        |          |Note: The destination address must be word aligned.
 */
    __IO uint32_t DSCT_ENDDA;           

    /**
     * TXBCCH0
     * ===================================================================================================
     * Offset: 0x0C  PDMA Transfer Byte Count Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |BYTECNT   |PDMA Transfer Byte Count Register
     * |        |          |This register controls the transfer byte count of PDMA. Maximum value is 0xFFFF.
     * |        |          |Note: When in memory-to-memory (PDMA_TXBCCHn.MODESEL = 00b) mode, the transfer byte count must be word aligned, that is multiples of 4bytes.
 */
    __IO uint32_t TXBCCH;               

    /**
     * INLBPCH0
     * ===================================================================================================
     * Offset: 0x10  PDMA Internal Buffer Pointer Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |BURPTR    |PDMA Internal Buffer Pointer Register (Read Only)
     * |        |          |A PDMA transaction consists of two stages, a read from the source address and a write to the destination address.
     * |        |          |Internally this data is buffered in a 32bit register.
     * |        |          |If transaction width between the read and write transactions are different, this register tracks which byte/half-word of the internal buffer is being processed by the current transaction.
 */
    __I  uint32_t INLBPCH;              

    /**
     * CURSACH0
     * ===================================================================================================
     * Offset: 0x14  PDMA Current Source Address Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |CURSA     |PDMA Current Source Address Register (Read Only)
     * |        |          |This register returns the source address from which the PDMA transfer is occurring.
     * |        |          |This register is loaded from ENDSA when PDMA is triggered or when a wraparound occurs.
 */
    __I  uint32_t CURSACH;              

    /**
     * CURDACH0
     * ===================================================================================================
     * Offset: 0x18  PDMA Current Destination Address Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |CURDA     |PDMA Current Destination Address Register (Read Only)
     * |        |          |This register returns the destination address to which the PDMA transfer is occurring.
     * |        |          |This register is loaded from PDMA_DSCTn_ENDDA when PDMA is triggered or when a wraparound occurs.
 */
    __I  uint32_t CURDACH;              

    /**
     * CURBCCH0
     * ===================================================================================================
     * Offset: 0x1C  PDMA Current Byte Count Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CURBC     |PDMA Current Byte Count Register (Read Only)
     * |        |          |This field indicates the current remaining byte count of PDMA transfer.
     * |        |          |This register is initialized with BYTECNT register when PDMA is triggered or when a wraparound occurs.
 */
    __I  uint32_t CURBCCH;              

    /**
     * INTENCH0
     * ===================================================================================================
     * Offset: 0x20  PDMA Interrupt Enable Control Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TXABTIEN  |PDMA Read/Write Target Abort Interrupt Enable
     * |        |          |If enabled, the PDMA controller will generate and interrupt to the CPU whenever a PDMA transaction is aborted due to an error.
     * |        |          |If a transfer is aborted, PDMA channel must be reset to resume DMA operation.
     * |        |          |0 = Disable PDMA transfer target abort interrupt generation.
     * |        |          |1 = Enable PDMA transfer target abort interrupt generation.
     * |[1]     |TXOKIEN   |PDMA Transfer Done Interrupt Enable
     * |        |          |If enabled, the PDMA controller will generate and interrupt to the CPU when the requested PDMA transfer is complete.
     * |        |          |0 = Disable PDMA transfer done interrupt generation.
     * |        |          |1 = Enable PDMA transfer done interrupt generation.
     * |[2]     |WAINTEN   |Wraparound Interrupt Enable
     * |        |          |If enabled, and channel source or destination address is in wraparound mode, the PDMA controller will generate a WRAP interrupt to the CPU according to the setting of
     * |        |          |PDMA_DSCTn_CTL.WAINTSEL.
     * |        |          |This can be interrupts when the transaction has finished and has wrapped around and/or when the transaction is half way in progress.
     * |        |          |This allows the efficient implementation of circular buffers for DMA.
     * |        |          |0 = Disable Wraparound PDMA interrupt generation.
     * |        |          |1 = Enable Wraparound interrupt generation.
 */
    __IO uint32_t INTENCH;              

    /**
     * CH0IF
     * ===================================================================================================
     * Offset: 0x24  PDMA Interrupt Status Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TXABTIF   |PDMA Read/Write Target Abort Interrupt Flag
     * |        |          |This flag indicates a Target Abort interrupt condition has occurred.
     * |        |          |This condition can happen if attempt is made to read/write from invalid or non-existent memory space.
     * |        |          |It occurs when PDMA controller receives a bus error from AHB master.
     * |        |          |Upon occurrence PDMA will stop transfer and go to idle state.
     * |        |          |To resume, software must reset PDMA channel and initiate transfer again.
     * |        |          |0 = No bus ERROR response received.
     * |        |          |1 = Bus ERROR response received.
     * |        |          |NOTE: This bit is cleared by writing 1 to itself.
     * |[1]     |TXOKIF    |Block Transfer Done Interrupt Flag
     * |        |          |This bit indicates that PDMA block transfer complete interrupt has been generated.
     * |        |          |It is cleared by writing 1 to the bit.
     * |        |          |0 = Transfer ongoing or Idle.
     * |        |          |1 = Transfer Complete.
     * |[8:11]  |WAIF      |Wrap Around Transfer Byte Count Interrupt Flag
     * |        |          |These flags are set whenever the conditions for a wraparound interrupt (complete or half complete) are met.
     * |        |          |They are cleared by writing one to the bits.
     * |        |          |0001 = Current transfer finished flag (CURBC == 0).
     * |        |          |0100 = Current transfer half complete flag (CURBC == BYTECNT/2).
     * |[31]    |INTSTS    |Interrupt Pin Status (Read Only)
     * |        |          |This bit is the Interrupt pin status of PDMA channel.
 */
    __IO uint32_t CHIF;                 
} PDMA_T;

typedef struct
{
    /**
     * GLOCTL
     * ===================================================================================================
     * Offset: 0xF00  PDMA Global Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SWRST     |PDMA Software Reset
     * |        |          |0 = Writing 0 to this bit has no effect.
     * |        |          |1 = Writing 1 to this bit will reset the internal state machine and pointers.
     * |        |          |The contents of control register will not be cleared.
     * |        |          |This bit will auto clear after several clock cycles.
     * |        |          |Note: This bit can reset all channels (global reset).
     * |[8:11]  |CHCKEN    |PDMA Controller Channel Clock Enable Control
     * |        |          |To enable clock for channel n CHCKEN[n] must be set.
     * |        |          |CHCKEN[n] = 1: Enable Channel n clock
     * |        |          |CHCKEN[n] = 0: Disable Channel n clock
 */
    __IO uint32_t GLOCTL;                

    /**
     * SVCSEL
     * ===================================================================================================
     * Offset: 0xF04  PDMA Service Selection Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |SPIRXSEL  |PDMA SPI0 Receive Selection
     * |        |          |This field defines which PDMA channel is connected to SPI0 peripheral receive (PDMA source) request.
     * |[4:7]   |SPITXSEL  |PDMA SPI0 Transmit Selection
     * |        |          |This field defines which PDMA channel is connected to SPI0 peripheral transmit (PDMA destination) request.
     * |[8:11]  |ADCRXSEL  |PDMA ADC Receive Selection
     * |        |          |This field defines which PDMA channel is connected to ADC peripheral receive (PDMA source) request.
     * |[12:15] |DPWMTXSEL |PDMA DPWM Transmit Selection
     * |        |          |This field defines which PDMA channel is connected to DPWM peripheral transmit (PDMA destination) request.
     * |[16:19] |UARTRXSEL |PDMA UART0 Receive Selection
     * |        |          |This field defines which PDMA channel is connected to UART0 peripheral receive (PDMA source) request.
     * |[20:23] |UARTXSEL  |PDMA UART0 Transmit Selection
     * |        |          |This field defines which PDMA channel is connected to UART0 peripheral transmit (PDMA destination) request
     * |[24:27] |I2SRXSEL  |PDMA I2S Receive Selection
     * |        |          |This field defines which PDMA channel is connected to I2S peripheral receive (PDMA source) request.
     * |[28:31] |I2STXSEL  |PDMA I2S Transmit Selection
     * |        |          |This field defines which PDMA channel is connected to I2S peripheral transmit (PDMA destination) request
 */
    __IO uint32_t SVCSEL;                
         uint32_t RESERVE13[1];
    /**
     * GLOBALIF
     * ===================================================================================================
     * Offset: 0xF0C  PDMA Global Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |GLOBALIF  |Interrupt Pin Status (Read Only)
     * |        |          |GLOBALIF[n] is the interrupt status of PDMA channel n.
 */
    __I  uint32_t GLOBALIF;     
} PDMA_GCR_T;		
		
/**
    @addtogroup PDMA_CONST PDMA Bit Field Definition
    Constant Definitions for PDMA Controller
@{ */

#define PDMA_DSCT_CTL_CHEN_Pos          (0)                                               /*!< PDMA DSCT0_CTL: CHEN Position          */
#define PDMA_DSCT_CTL_CHEN_Msk          (0x1ul << PDMA_DSCT_CTL_CHEN_Pos)                /*!< PDMA DSCT0_CTL: CHEN Mask              */

#define PDMA_DSCT_CTL_SWRST_Pos         (1)                                               /*!< PDMA DSCT0_CTL: SWRST Position         */
#define PDMA_DSCT_CTL_SWRST_Msk         (0x1ul << PDMA_DSCT_CTL_SWRST_Pos)               /*!< PDMA DSCT0_CTL: SWRST Mask             */

#define PDMA_DSCT_CTL_MODESEL_Pos       (2)                                               /*!< PDMA DSCT0_CTL: MODESEL Position       */
#define PDMA_DSCT_CTL_MODESEL_Msk       (0x3ul << PDMA_DSCT_CTL_MODESEL_Pos)             /*!< PDMA DSCT0_CTL: MODESEL Mask           */

#define PDMA_DSCT_CTL_SASEL_Pos         (4)                                               /*!< PDMA DSCT0_CTL: SASEL Position         */
#define PDMA_DSCT_CTL_SASEL_Msk         (0x3ul << PDMA_DSCT_CTL_SASEL_Pos)               /*!< PDMA DSCT0_CTL: SASEL Mask             */

#define PDMA_DSCT_CTL_DASEL_Pos         (6)                                               /*!< PDMA DSCT0_CTL: DASEL Position         */
#define PDMA_DSCT_CTL_DASEL_Msk         (0x3ul << PDMA_DSCT_CTL_DASEL_Pos)               /*!< PDMA DSCT0_CTL: DASEL Mask             */

#define PDMA_DSCT_CTL_WAINTSEL_Pos      (12)                                              /*!< PDMA DSCT0_CTL: WAINTSEL Position      */
#define PDMA_DSCT_CTL_WAINTSEL_Msk      (0xful << PDMA_DSCT_CTL_WAINTSEL_Pos)            /*!< PDMA DSCT0_CTL: WAINTSEL Mask          */

#define PDMA_DSCT_CTL_TXWIDTH_Pos       (19)                                              /*!< PDMA DSCT0_CTL: TXWIDTH Position       */
#define PDMA_DSCT_CTL_TXWIDTH_Msk       (0x3ul << PDMA_DSCT_CTL_TXWIDTH_Pos)             /*!< PDMA DSCT0_CTL: TXWIDTH Mask           */

#define PDMA_DSCT_CTL_TXEN_Pos          (23)                                              /*!< PDMA DSCT0_CTL: TXEN Position          */
#define PDMA_DSCT_CTL_TXEN_Msk          (0x1ul << PDMA_DSCT_CTL_TXEN_Pos)                /*!< PDMA DSCT0_CTL: TXEN Mask              */

#define PDMA_DSCT_ENDSA_ENDSA_Pos       (0)                                               /*!< PDMA DSCT0_ENDSA: ENDSA Position       */
#define PDMA_DSCT_ENDSA_ENDSA_Msk       (0xfffffffful << PDMA_DSCT_ENDSA_ENDSA_Pos)      /*!< PDMA DSCT0_ENDSA: ENDSA Mask           */

#define PDMA_DSCT_ENDDA_ENDDA_Pos       (0)                                               /*!< PDMA DSCT0_ENDDA: ENDDA Position       */
#define PDMA_DSCT_ENDDA_ENDDA_Msk       (0xfffffffful << PDMA_DSCT_ENDDA_ENDDA_Pos)      /*!< PDMA DSCT0_ENDDA: ENDDA Mask           */

#define PDMA_TXBCCH_BYTECNT_Pos         (0)                                               /*!< PDMA TXBCCH0: BYTECNT Position         */
#define PDMA_TXBCCH_BYTECNT_Msk         (0xfffful << PDMA_TXBCCH_BYTECNT_Pos)            /*!< PDMA TXBCCH0: BYTECNT Mask             */

#define PDMA_INLBPCH_BURPTR_Pos         (0)                                               /*!< PDMA INLBPCH0: BURPTR Position         */
#define PDMA_INLBPCH_BURPTR_Msk         (0xful << PDMA_INLBPCH_BURPTR_Pos)               /*!< PDMA INLBPCH0: BURPTR Mask             */

#define PDMA_CURSACH_CURSA_Pos          (0)                                               /*!< PDMA CURSACH0: CURSA Position          */
#define PDMA_CURSACH_CURSA_Msk          (0xfffffffful << PDMA_CURSACH_CURSA_Pos)         /*!< PDMA CURSACH0: CURSA Mask              */

#define PDMA_CURDACH_CURDA_Pos          (0)                                               /*!< PDMA CURDACH0: CURDA Position          */
#define PDMA_CURDACH_CURDA_Msk          (0xfffffffful << PDMA_CURDACH_CURDA_Pos)         /*!< PDMA CURDACH0: CURDA Mask              */

#define PDMA_CURBCCH_CURBC_Pos          (0)                                               /*!< PDMA CURBCCH0: CURBC Position          */
#define PDMA_CURBCCH_CURBC_Msk          (0xfffful << PDMA_CURBCCH_CURBC_Pos)             /*!< PDMA CURBCCH0: CURBC Mask              */

#define PDMA_INTENCH_TXABTIEN_Pos       (0)                                               /*!< PDMA INTENCH0: TXABTIEN Position       */
#define PDMA_INTENCH_TXABTIEN_Msk       (0x1ul << PDMA_INTENCH_TXABTIEN_Pos)             /*!< PDMA INTENCH0: TXABTIEN Mask           */

#define PDMA_INTENCH_TXOKIEN_Pos        (1)                                               /*!< PDMA INTENCH0: TXOKIEN Position        */
#define PDMA_INTENCH_TXOKIEN_Msk        (0x1ul << PDMA_INTENCH_TXOKIEN_Pos)              /*!< PDMA INTENCH0: TXOKIEN Mask            */

#define PDMA_INTENCH_WAINTEN_Pos        (2)                                               /*!< PDMA INTENCH0: WAINTEN Position        */
#define PDMA_INTENCH_WAINTEN_Msk        (0x1ul << PDMA_INTENCH_WAINTEN_Pos)              /*!< PDMA INTENCH0: WAINTEN Mask            */

#define PDMA_CHIF_TXABTIF_Pos           (0)                                               /*!< PDMA CH0IF: TXABTIF Position           */
#define PDMA_CHIF_TXABTIF_Msk           (0x1ul << PDMA_CHIF_TXABTIF_Pos)                 /*!< PDMA CH0IF: TXABTIF Mask               */

#define PDMA_CHIF_TXOKIF_Pos            (1)                                               /*!< PDMA CH0IF: TXOKIF Position            */
#define PDMA_CHIF_TXOKIF_Msk            (0x1ul << PDMA_CHIF_TXOKIF_Pos)                  /*!< PDMA CH0IF: TXOKIF Mask                */

#define PDMA_CHIF_WAIF_Pos              (8)                                               /*!< PDMA CH0IF: WAIF Position              */
#define PDMA_CHIF_WAIF_Msk              (0xful << PDMA_CHIF_WAIF_Pos)                    /*!< PDMA CH0IF: WAIF Mask                  */

#define PDMA_CHIF_INTSTS_Pos            (31)                                              /*!< PDMA CH0IF: INTSTS Position            */
#define PDMA_CHIF_INTSTS_Msk            (0x1ul << PDMA_CHIF_INTSTS_Pos)                  /*!< PDMA CH0IF: INTSTS Mask                */

/* PDMA GLOCTL Bit Field Definitions */
#define PDMA_GLOCTL_SWRST_Pos            (0)                                               /*!< PDMA GLOCTL: SWRST Position            */
#define PDMA_GLOCTL_SWRST_Msk            (0x1ul << PDMA_GLOCTL_SWRST_Pos)                  /*!< PDMA GLOCTL: SWRST Mask                */

#define PDMA_GLOCTL_CHCKEN_Pos           (8)                                               /*!< PDMA GLOCTL: CHCKEN Position           */
#define PDMA_GLOCTL_CHCKEN_Msk           (0x3ul << PDMA_GLOCTL_CHCKEN_Pos)                 /*!< PDMA GLOCTL: CHCKEN Mask               */

#define PDMA_SVCSEL_SPIRXSEL_Pos         (0)                                               /*!< PDMA SVCSEL: SPIRXSEL Position         */
#define PDMA_SVCSEL_SPIRXSEL_Msk         (0x3ul << PDMA_SVCSEL_SPIRXSEL_Pos)               /*!< PDMA SVCSEL: SPIRXSEL Mask             */

#define PDMA_SVCSEL_SPITXSEL_Pos         (4)                                               /*!< PDMA SVCSEL: SPITXSEL Position         */
#define PDMA_SVCSEL_SPITXSEL_Msk         (0x3ul << PDMA_SVCSEL_SPITXSEL_Pos)               /*!< PDMA SVCSEL: SPITXSEL Mask             */

#define PDMA_SVCSEL_SPIMRXSEL_Pos        (8)                                               /*!< PDMA SVCSEL: SPIRXSEL Position         */
#define PDMA_SVCSEL_SPIMRXSEL_Msk        (0x3ul << PDMA_SVCSEL_SPIMRXSEL_Pos)               /*!< PDMA SVCSEL: SPIRXSEL Mask             */

#define PDMA_SVCSEL_SPIMTXSEL_Pos        (12)                                               /*!< PDMA SVCSEL: SPITXSEL Position         */
#define PDMA_SVCSEL_SPIMTXSEL_Msk        (0x3ul << PDMA_SVCSEL_SPIMTXSEL_Pos)               /*!< PDMA SVCSEL: SPITXSEL Mask             */

#define PDMA_SVCSEL_ADCRXSEL_Pos         (16)                                               /*!< PDMA SVCSEL: ADCRXSEL Position         */
#define PDMA_SVCSEL_ADCRXSEL_Msk         (0x3ul << PDMA_SVCSEL_ADCRXSEL_Pos)               /*!< PDMA SVCSEL: ADCRXSEL Mask             */

#define PDMA_SVCSEL_DPWMTXSEL_Pos        (20)                                              /*!< PDMA SVCSEL: DPWMTXSEL Position        */
#define PDMA_SVCSEL_DPWMTXSEL_Msk        (0x3ul << PDMA_SVCSEL_DPWMTXSEL_Pos)              /*!< PDMA SVCSEL: DPWMTXSEL Mask            */

#define PDMA_GLOBALIF_GLOBALIF_Pos       (0)                                               /*!< PDMA GLOBALIF: GLOBALIF Position       */
#define PDMA_GLOBALIF_GLOBALIF_Msk       (0x3ul << PDMA_GLOBALIF_GLOBALIF_Pos)             /*!< PDMA GLOBALIF: GLOBALIF Mask           */

/**@}*/ /* PDMA_CONST */
/**@}*/ /* end of PDMA register group */

/*---------------------- Watch Dog Timer Controller -------------------------*/
/**
    @addtogroup WDT Watch Dog Timer Controller(WDT)
    Memory Mapped Structure for WDT Controller
@{ */
 
typedef struct
{
    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  Watchdog Timer Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RSTCNT    |Clear Watchdog Timer
     * |        |          |Set this bit will clear the Watchdog timer.
     * |        |          |0 = Writing 0 to this bit has no effect
     * |        |          |1 = Reset the contents of the Watchdog timer
     * |        |          |NOTE: This bit will auto clear after few clock cycle
     * |[1]     |RSTEN     |Watchdog Timer Reset Enable
     * |        |          |Setting this bit will enable the Watchdog timer reset function.
     * |        |          |0 = Disable Watchdog timer reset function
     * |        |          |1= Enable Watchdog timer reset function
     * |[2]     |RSTF      |Watchdog Timer Reset Flag
     * |        |          |When the Watchdog timer initiates a reset, the hardware will set this bit.
     * |        |          |This flag can be read by software to determine the source of reset.
     * |        |          |Software is responsible to clear it manually by writing 1 to it.
     * |        |          |If RSTEN is disabled, then the Watchdog timer has no effect on this bit.
     * |        |          |0 = Watchdog timer reset has not occurred.
     * |        |          |1= Watchdog timer reset has occurred.
     * |        |          |NOTE: This bit is cleared by writing 1 to this bit.
     * |[3]     |IF        |Watchdog Timer Interrupt Flag
     * |        |          |If the Watchdog timer interrupt is enabled, then the hardware will set this bit to indicate that the Watchdog timer interrupt has occurred.
     * |        |          |If the Watchdog timer interrupt is not enabled, then this bit indicates that a timeout period has elapsed.
     * |        |          |0 = Watchdog timer interrupt has not occurred.
     * |        |          |1 = Watchdog timer interrupt has occurred.
     * |        |          |NOTE: This bit is cleared by writing 1 to this bit.
     * |[4]     |WKEN      |WDT Time-Out Wake-Up Function Control
     * |        |          |If this bit is set to 1, while WDT time-out interrupt flag IF (WDT_CTL[3]) is generated to 1 and interrupt enable bit INTEN (WDT_CTL[6]) is enabled, the WDT time-out interrupt signal will generate a wake-up trigger event to chip.
     * |        |          |0 = Wake-up trigger event Disabled if WDT time-out interrupt signal generated.
     * |        |          |1 = Wake-up trigger event Enabled if WDT time-out interrupt signal generated.
     * |[5]     |WKF       |WDT Time-Out Wake-Up Flag
     * |        |          |If WDT causes CPU wake up from sleep or power-down mode, this bit will be set to high.
     * |        |          |0 = WDT does not cause chip wake-up.
     * |        |          |1 = Chip wakes up from sleep or power-down mode by WDT time-out interrupt.
     * |        |          |Note: This bit is cleared by writing 1 to it. 
     * |[6]     |INTEN     |Watchdog Timer Interrupt Enable
     * |        |          |0 = Disable the Watchdog timer interrupt
     * |        |          |1 = Enable the Watchdog timer interrupt
     * |[7]     |WDTEN     |Watchdog Timer Enable
     * |        |          |0 = Disable the Watchdog timer (This action will reset the internal counter)
     * |        |          |1 = Enable the Watchdog timer
     * |[8:10]  |TOUTSEL   |Watchdog Timer Interval Select
     * |        |          |These three bits select the timeout interval for the Watchdog timer, a watchdog reset will occur 1024 clock cycles later if WDG not reset.
     * |        |          |The timeout is given by:.
     * |        |          |Interrupt Timeout = 2^(2xTOUTSEL+4) x WDT_CLK
     * |        |          |Reset Timeout = (2^(2xTOUTSEL+4) +1024) x WDT_CLK
     * |        |          |Where WDT_CLK is the period of the Watchdog Timer clock source.
 */
    __IO uint32_t CTL;                   

} WDT_T;

/**
    @addtogroup WDT_CONST WDT Bit Field Definition
    Constant Definitions for WDT Controller
@{ */

#define WDT_CTL_RSTCNT_Pos               (0)                                               /*!< WDT CTL: RSTCNT Position               */
#define WDT_CTL_RSTCNT_Msk               (0x1ul << WDT_CTL_RSTCNT_Pos)                     /*!< WDT CTL: RSTCNT Mask                   */

#define WDT_CTL_RSTEN_Pos                (1)                                               /*!< WDT CTL: RSTEN Position                */
#define WDT_CTL_RSTEN_Msk                (0x1ul << WDT_CTL_RSTEN_Pos)                      /*!< WDT CTL: RSTEN Mask                    */

#define WDT_CTL_RSTF_Pos                 (2)                                               /*!< WDT CTL: RSTF Position                 */
#define WDT_CTL_RSTF_Msk                 (0x1ul << WDT_CTL_RSTF_Pos)                       /*!< WDT CTL: RSTF Mask                     */

#define WDT_CTL_IF_Pos                   (3)                                               /*!< WDT CTL: IF Position                   */
#define WDT_CTL_IF_Msk                   (0x1ul << WDT_CTL_IF_Pos)                         /*!< WDT CTL: IF Mask                       */

#define WDT_CTL_WKEN_Pos                 (4)                                               /*!< WDT CTL: WKEN Position                 */
#define WDT_CTL_WKEN_Msk                 (0x1ul << WDT_CTL_WKEN_Pos)                       /*!< WDT CTL: WKEN Mask                     */

#define WDT_CTL_WKF_Pos                  (5)                                               /*!< WDT CTL: WKF Position                  */
#define WDT_CTL_WKF_Msk                  (0x1ul << WDT_CTL_WKF_Pos)                        /*!< WDT CTL: WKF Mask                      */

#define WDT_CTL_INTEN_Pos                (6)                                               /*!< WDT CTL: INTEN Position                */
#define WDT_CTL_INTEN_Msk                (0x1ul << WDT_CTL_INTEN_Pos)                      /*!< WDT CTL: INTEN Mask                    */

#define WDT_CTL_WDTEN_Pos                (7)                                               /*!< WDT CTL: WDTEN Position                */
#define WDT_CTL_WDTEN_Msk                (0x1ul << WDT_CTL_WDTEN_Pos)                      /*!< WDT CTL: WDTEN Mask                    */

#define WDT_CTL_TOUTSEL_Pos              (8)                                               /*!< WDT CTL: TOUTSEL Position              */
#define WDT_CTL_TOUTSEL_Msk              (0x7ul << WDT_CTL_TOUTSEL_Pos)                    /*!< WDT CTL: TOUTSEL Mask                  */

/**@}*/ /* WDT_CONST */
/**@}*/ /* end of WDT register group */

/*---------------------- Pulse Width Modulation Controller -------------------------*/
/**
    @addtogroup PWM Pulse Width Modulation Controller(PWM)
    Memory Mapped Structure for PWM Controller
@{ */
 
typedef struct
{
    /**
     * CLKPSC
     * ===================================================================================================
     * Offset: 0x00  PWM Prescaler Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |CLKPSC    |Clock Prescaler For PWM Timer
     * |        |          |Clock input is divided by (CLKPSC + 1)
     * |        |          |If CLKPSC = 0, then the prescaler output clock will be stopped.
     * |        |          |This implies PWM counter will also be stopped.
     * |[16:23] |DZI0      |Dead Zone Interval Register 0
     * |        |          |These 8 bits determine dead zone length.
     * |        |          |The unit time of dead zone length is that from clock selector.
     * |[24:31] |DZI1      |Dead Zone Interval Register 1
     * |        |          |These 8 bits determine dead zone length.
     * |        |          |The unit time of dead zone length is that from clock selector.
 */
    __IO uint32_t CLKPSC;                

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x04  PWM Clock Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:2]   |CLKDIV    |PWM Timer Clock Source Selection
     * |        |          |Value : Input clock divided by
     * |        |          |000 : 2
     * |        |          |001 : 4
     * |        |          |010 : 8
     * |        |          |011 : 16
     * |        |          |1xx : 1
 */
    __IO uint32_t CLKDIV;                

    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x08  PWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTEN     |PWM-Timer Enable
     * |        |          |0 = Stop PWM-Timer Running.
     * |        |          |1 = Enable PWM-Timer.
     * |[2]     |PINV      |PWM-Timer Output Inverter ON/OFF
     * |        |          |0 = Inverter OFF.
     * |        |          |1 = Inverter ON.
     * |[3]     |CNTMODE   |PWM-Timer Auto-Reload/One-Shot Mode
     * |        |          |0 = One-Shot Mode.
     * |        |          |1 = Auto-reload Mode.
     * |[4]     |DTEN0     |Dead-Zone 0 Generator Enable/Disable
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * |[5]     |DTEN1     |Dead-Zone 1 Generator Enable/Disable
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
 */
    __IO uint32_t CTL;                   

    /**
     * PERIOD
     * ===================================================================================================
     * Offset: 0x0C  PWM Period Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |PERIOD    |PWM Counter/Timer Reload Value
     * |        |          |PERIOD determines the PWM period.
     * |        |          |Note: One PWM cycle width = (PERIOD + 1).
 */
    __IO uint32_t PERIOD;                

    /**
     * CMPDAT0
     * ===================================================================================================
     * Offset: 0x10  PWM Comparator Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CMP       |PWM Comparator Register
     * |        |          |CMP determines the PWM duty cycle.
     * |        |          |Assumption: PWM output initial is high
     * |        |          |CMP > = PERIOD: PWM output is always high.
     * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
     * |        |          |Note1: Unit = one PWM clock cycle.
     * |        |          |Note2: Any write to CMP will take effect in next PWM cycle.
 */
    __IO uint32_t CMPDAT0;               

    /**
     * CNT
     * ===================================================================================================
     * Offset: 0x14  PWM Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CNT       |PWM Counter Register
     * |        |          |Reports the current value of the 16-bit down counter.
 */
    __I  uint32_t CNT;                   
         uint32_t RESERVE0[1];


    /**
     * CMPDAT1
     * ===================================================================================================
     * Offset: 0x1C  PWM Comparator Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CMP       |PWM Comparator Register
     * |        |          |CMP determines the PWM duty cycle.
     * |        |          |Assumption: PWM output initial is high
     * |        |          |CMP > = PERIOD: PWM output is always high.
     * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
     * |        |          |Note1: Unit = one PWM clock cycle.
     * |        |          |Note2: Any write to CMP will take effect in next PWM cycle.
 */
    __IO uint32_t CMPDAT1;               
         uint32_t RESERVE1[2];


    /**
     * CMPDAT2
     * ===================================================================================================
     * Offset: 0x28  PWM Comparator Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CMP       |PWM Comparator Register
     * |        |          |CMP determines the PWM duty cycle.
     * |        |          |Assumption: PWM output initial is high
     * |        |          |CMP > = PERIOD: PWM output is always high.
     * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
     * |        |          |Note1: Unit = one PWM clock cycle.
     * |        |          |Note2: Any write to CMP will take effect in next PWM cycle.
 */
    __IO uint32_t CMPDAT2;               
         uint32_t RESERVE2[2];


    /**
     * CMPDAT3
     * ===================================================================================================
     * Offset: 0x34  PWM Comparator Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CMP       |PWM Comparator Register
     * |        |          |CMP determines the PWM duty cycle.
     * |        |          |Assumption: PWM output initial is high
     * |        |          |CMP > = PERIOD: PWM output is always high.
     * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
     * |        |          |Note1: Unit = one PWM clock cycle.
     * |        |          |Note2: Any write to CMP will take effect in next PWM cycle.
 */
    __IO uint32_t CMPDAT3;               
         uint32_t RESERVE3[2];


    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x40  PWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIEN      |PWM Timer Interrupt Enable
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
 */
    __IO uint32_t INTEN;                 

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x44  PWM Interrupt Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIF       |PWM Timer Interrupt Flag
     * |        |          |Flag is set by hardware when PWM down counter reaches zero, software can clear this bit by writing '1' to it.
 */
    __IO uint32_t INTSTS;                
         uint32_t RESERVE4[2];


    /**
     * CAPCTL
     * ===================================================================================================
     * Offset: 0x50  Capture Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPINV    |Inverter ON/OFF
     * |        |          |0 = Inverter OFF.
     * |        |          |1 = Inverter ON. Reverse the input signal from GPIO before fed to Capture timer
     * |[1]     |CRLIEN    |Rising Latch Interrupt Enable ON/OFF
     * |        |          |0 = Disable rising latch interrupt.
     * |        |          |1 = Enable rising latch interrupt.
     * |        |          |When enabled, capture block generates an interrupt on rising edge of input.
     * |[2]     |CFLIEN    |Falling Latch Interrupt Enable ON/OFF
     * |        |          |0 = Disable falling latch interrupt.
     * |        |          |1 = Enable falling latch interrupt.
     * |        |          |When enabled, capture block generates an interrupt on falling edge of input.
     * |[3]     |CAPEN     |Capture Channel Input Transition Enable/Disable
     * |        |          |0 = Disable capture function.
     * |        |          |1 = Enable capture function.
     * |        |          |When enabled, Capture function latches the PMW-counter to RCAPDAT (Rising latch) and FCAPDAT (Falling latch) registers on input edge transition.
     * |        |          |When disabled, Capture function is inactive as is interrupt.
     * |[4]     |CAPIF     |Capture Interrupt Indication Flag
     * |        |          |When capture input has a falling/rising transition and falling/rising latch interrupt is enabled (CFLIEN = 1/CRLIEN = 1), CAPIF0 is set 1 by hardware.
     * |        |          |Software can clear this bit by writing a one to it.
     * |        |          |Note:
     * |        |          |If this bit is "1", PWM counter will not be reloaded when next capture interrupt occurs.
     * |[6]     |CRLIF     |PWM_RCAPDAT Latched Indicator Bit
     * |        |          |When input channel has a rising transition, PWM_RCAPDAT was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it.
     * |[7]     |CFLIF     |PWM_FCAPDAT Latched Indicator Bit
     * |        |          |When input channel has a falling transition, PWM_FCAPDAT was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it
 */
    __IO uint32_t CAPCTL;                
         uint32_t RESERVE5[1];


    /**
     * RCAPDAT
     * ===================================================================================================
     * Offset: 0x58  Capture Rising Latch Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |RCAPDAT   |Capture Rising Latch Register
     * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a rising edge of the input signal.
 */
    __I  uint32_t RCAPDAT;               

    /**
     * FCAPDAT
     * ===================================================================================================
     * Offset: 0x5C  Capture Falling Latch Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |FCAPDAT   |Capture Falling Latch Register
     * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a falling edge of the input signal.
 */
    __I  uint32_t FCAPDAT;               
         uint32_t RESERVE6[7];


    /**
     * PCEN
     * ===================================================================================================
     * Offset: 0x7C  PWM Output and Capture Input Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |POEN0     |PWM0 Output Enable Register
     * |        |          |0 = Disable PWM0 output to pin.
     * |        |          |1 = Enable PWM0 output to pin.
     * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP Table 5-8)
     * |[1]     |POEN1     |PWM1 Output Enable Register
     * |        |          |0 = Disable PWM1 output to pin.
     * |        |          |1 = Enable PWM1 output to pin.
     * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP Table 5-8)
     * |[2]     |POEN2     |PWM2 Output Enable Register
     * |        |          |0 = Disable PWM2 output to pin.
     * |        |          |1 = Enable PWM2 output to pin.
     * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP Table 5-8)
     * |[3]     |POEN3     |PWM3 Output Enable Register
     * |        |          |0 = Disable PWM3 output to pin.
     * |        |          |1 = Enable PWM3 output to pin.
     * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPB_MFP Table 5-8)
     * |[8]     |CAPINEN   |Capture Input Enable Register
     * |        |          |0 = OFF (PB.12 pin input disconnected from Capture block).
     * |        |          |1 = ON (PB.12 pin, if in PWM alternative function, will be configured as an input and fed to capture function).
 */
    __IO uint32_t PCEN;                  

} PWM_T;

/**
    @addtogroup PWM_CONST PWM Bit Field Definition
    Constant Definitions for PWM Controller
@{ */

#define PWM_CLKPSC_CLKPSC_Pos            (0)                                               /*!< PWM CLKPSC: CLKPSC Position            */
#define PWM_CLKPSC_CLKPSC_Msk            (0xfful << PWM_CLKPSC_CLKPSC_Pos)                 /*!< PWM CLKPSC: CLKPSC Mask                */

#define PWM_CLKPSC_DZI0_Pos              (16)                                              /*!< PWM CLKPSC: DZI0 Position              */
#define PWM_CLKPSC_DZI0_Msk              (0xfful << PWM_CLKPSC_DZI0_Pos)                   /*!< PWM CLKPSC: DZI0 Mask                  */

#define PWM_CLKPSC_DZI1_Pos              (24)                                              /*!< PWM CLKPSC: DZI1 Position              */
#define PWM_CLKPSC_DZI1_Msk              (0xfful << PWM_CLKPSC_DZI1_Pos)                   /*!< PWM CLKPSC: DZI1 Mask                  */

#define PWM_CLKDIV_CLKDIV_Pos           (0)                                                /*!< PWM CLKDIV: CLKDIV Position           */
#define PWM_CLKDIV_CLKDIV_Msk           (0x7ul << PWM_CLKDIV_CLKDIV_Pos)                   /*!< PWM CLKDIV: CLKDIV Mask               */

#define PWM_CTL_CNTEN_Pos                (0)                                               /*!< PWM CTL: CNTEN Position                */
#define PWM_CTL_CNTEN_Msk                (0x1ul << PWM_CTL_CNTEN_Pos)                      /*!< PWM CTL: CNTEN Mask                    */

#define PWM_CTL_PINV_Pos                 (2)                                               /*!< PWM CTL: PINV Position                 */
#define PWM_CTL_PINV_Msk                 (0x1ul << PWM_CTL_PINV_Pos)                       /*!< PWM CTL: PINV Mask                     */

#define PWM_CTL_CNTMODE_Pos              (3)                                               /*!< PWM CTL: CNTMODE Position              */
#define PWM_CTL_CNTMODE_Msk              (0x1ul << PWM_CTL_CNTMODE_Pos)                    /*!< PWM CTL: CNTMODE Mask                  */

#define PWM_CTL_DTEN0_Pos                (4)                                               /*!< PWM CTL: DTEN0 Position                */
#define PWM_CTL_DTEN0_Msk                (0x1ul << PWM_CTL_DTEN0_Pos)                      /*!< PWM CTL: DTEN0 Mask                    */

#define PWM_CTL_DTEN1_Pos                (5)                                               /*!< PWM CTL: DTEN1 Position                */
#define PWM_CTL_DTEN1_Msk                (0x1ul << PWM_CTL_DTEN1_Pos)                      /*!< PWM CTL: DTEN1 Mask                    */

#define PWM_PERIOD_PERIOD_Pos            (0)                                               /*!< PWM PERIOD: PERIOD Position            */
#define PWM_PERIOD_PERIOD_Msk            (0xfffful << PWM_PERIOD_PERIOD_Pos)               /*!< PWM PERIOD: PERIOD Mask                */

#define PWM_CMPDAT0_CMP_Pos              (0)                                               /*!< PWM CMPDAT0: CMP Position              */
#define PWM_CMPDAT0_CMP_Msk              (0xfffful << PWM_CMPDAT0_CMP_Pos)                 /*!< PWM CMPDAT0: CMP Mask                  */

#define PWM_CNT_CNT_Pos                  (0)                                               /*!< PWM CNT: CNT Position                  */
#define PWM_CNT_CNT_Msk                  (0xfffful << PWM_CNT_CNT_Pos)                     /*!< PWM CNT: CNT Mask                      */

#define PWM_CMPDAT1_CMP_Pos              (0)                                               /*!< PWM CMPDAT1: CMP Position              */
#define PWM_CMPDAT1_CMP_Msk              (0xfffful << PWM_CMPDAT1_CMP_Pos)                 /*!< PWM CMPDAT1: CMP Mask                  */

#define PWM_CMPDAT2_CMP_Pos              (0)                                               /*!< PWM CMPDAT2: CMP Position              */
#define PWM_CMPDAT2_CMP_Msk              (0xfffful << PWM_CMPDAT2_CMP_Pos)                 /*!< PWM CMPDAT2: CMP Mask                  */

#define PWM_CMPDAT3_CMP_Pos              (0)                                               /*!< PWM CMPDAT3: CMP Position              */
#define PWM_CMPDAT3_CMP_Msk              (0xfffful << PWM_CMPDAT3_CMP_Pos)                 /*!< PWM CMPDAT3: CMP Mask                  */

#define PWM_INTEN_PIEN_Pos               (0)                                               /*!< PWM INTEN: PIEN Position               */
#define PWM_INTEN_PIEN_Msk               (0x1ul << PWM_INTEN_PIEN_Pos)                     /*!< PWM INTEN: PIEN Mask                   */

#define PWM_INTSTS_PIF_Pos               (0)                                               /*!< PWM INTSTS: PIF Position               */
#define PWM_INTSTS_PIF_Msk               (0x1ul << PWM_INTSTS_PIF_Pos)                     /*!< PWM INTSTS: PIF Mask                   */

#define PWM_CAPCTL_CAPINV_Pos            (0)                                               /*!< PWM CAPCTL: CAPINV Position            */
#define PWM_CAPCTL_CAPINV_Msk            (0x1ul << PWM_CAPCTL_CAPINV_Pos)                  /*!< PWM CAPCTL: CAPINV Mask                */

#define PWM_CAPCTL_CRLIEN_Pos            (1)                                               /*!< PWM CAPCTL: CRLIEN Position            */
#define PWM_CAPCTL_CRLIEN_Msk            (0x1ul << PWM_CAPCTL_CRLIEN_Pos)                  /*!< PWM CAPCTL: CRLIEN Mask                */

#define PWM_CAPCTL_CFLIEN_Pos            (2)                                               /*!< PWM CAPCTL: CFLIEN Position            */
#define PWM_CAPCTL_CFLIEN_Msk            (0x1ul << PWM_CAPCTL_CFLIEN_Pos)                  /*!< PWM CAPCTL: CFLIEN Mask                */

#define PWM_CAPCTL_CAPEN_Pos             (3)                                               /*!< PWM CAPCTL: CAPEN Position             */
#define PWM_CAPCTL_CAPEN_Msk             (0x1ul << PWM_CAPCTL_CAPEN_Pos)                   /*!< PWM CAPCTL: CAPEN Mask                 */

#define PWM_CAPCTL_CAPIF_Pos             (4)                                               /*!< PWM CAPCTL: CAPIF Position             */
#define PWM_CAPCTL_CAPIF_Msk             (0x1ul << PWM_CAPCTL_CAPIF_Pos)                   /*!< PWM CAPCTL: CAPIF Mask                 */

#define PWM_CAPCTL_CRLIF_Pos             (6)                                               /*!< PWM CAPCTL: CRLIF Position             */
#define PWM_CAPCTL_CRLIF_Msk             (0x1ul << PWM_CAPCTL_CRLIF_Pos)                   /*!< PWM CAPCTL: CRLIF Mask                 */

#define PWM_CAPCTL_CFLIF_Pos             (7)                                               /*!< PWM CAPCTL: CFLIF Position             */
#define PWM_CAPCTL_CFLIF_Msk             (0x1ul << PWM_CAPCTL_CFLIF_Pos)                   /*!< PWM CAPCTL: CFLIF Mask                 */

#define PWM_RCAPDAT_RCAPDAT_Pos          (0)                                               /*!< PWM RCAPDAT: RCAPDAT Position          */
#define PWM_RCAPDAT_RCAPDAT_Msk          (0xfffful << PWM_RCAPDAT_RCAPDAT_Pos)             /*!< PWM RCAPDAT: RCAPDAT Mask              */

#define PWM_FCAPDAT_FCAPDAT_Pos          (0)                                               /*!< PWM FCAPDAT: FCAPDAT Position          */
#define PWM_FCAPDAT_FCAPDAT_Msk          (0xfffful << PWM_FCAPDAT_FCAPDAT_Pos)             /*!< PWM FCAPDAT: FCAPDAT Mask              */

#define PWM_PCEN_POEN0_Pos               (0)                                               /*!< PWM PCEN: POEN0 Position               */
#define PWM_PCEN_POEN0_Msk               (0x1ul << PWM_PCEN_POEN0_Pos)                     /*!< PWM PCEN: POEN0 Mask                   */

#define PWM_PCEN_POEN1_Pos               (1)                                               /*!< PWM PCEN: POEN1 Position               */
#define PWM_PCEN_POEN1_Msk               (0x1ul << PWM_PCEN_POEN1_Pos)                     /*!< PWM PCEN: POEN1 Mask                   */

#define PWM_PCEN_POEN2_Pos               (2)                                               /*!< PWM PCEN: POEN2 Position               */
#define PWM_PCEN_POEN2_Msk               (0x1ul << PWM_PCEN_POEN2_Pos)                     /*!< PWM PCEN: POEN2 Mask                   */

#define PWM_PCEN_POEN3_Pos               (3)                                               /*!< PWM PCEN: POEN3 Position               */
#define PWM_PCEN_POEN3_Msk               (0x1ul << PWM_PCEN_POEN3_Pos)                     /*!< PWM PCEN: POEN3 Mask                   */

#define PWM_PCEN_CAPINEN_Pos             (8)                                               /*!< PWM PCEN: CAPINEN Position             */
#define PWM_PCEN_CAPINEN_Msk             (0x1ul << PWM_PCEN_CAPINEN_Pos)                   /*!< PWM PCEN: CAPINEN Mask                 */

/**@}*/ /* PWM_CONST */
/**@}*/ /* end of PWM register group */


/*---------------------- General Purpose Input/Output Controller -------------------------*/
/**
    @addtogroup GPIO General Purpose Input/Output Controller(GPIO)
    Memory Mapped Structure for GPIO Controller
@{ */
 
typedef struct
{
    /**
     * MODE
     * ===================================================================================================
     * Offset: 0x00  GPIO Pin I/O Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |MODE0     |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[3:2]   |MODE1     |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[5:4]   |MODE2     |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[7:6]   |MODE3     |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[9:8]   |MODE4     |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[11:10] |MODE5     |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[13:12] |MODE6     |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[15:14] |MODE7     |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[17:16] |MODE8     |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[19:18] |MODE9     |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[21:20] |MODE10    |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[23:22] |MODE11    |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[25:24] |MODE12    |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[27:26] |MODE13    |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[29:28] |MODE14    |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
     * |[31:30] |MODE15    |Port [A/B] Pin[N] I/O Mode Control
     * |        |          |Each GPIO Px pin has four modes:
     * |        |          |00 = GPIO Px[n] pin is in INPUT mode.
     * |        |          |01 = GPIO Px[n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO Px[n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO Px[n] pin is in Quasi-bidirectional mode.
     * |        |          |Note: PB_MODE[7:0] are reserved to 0.
 */
    __IO uint32_t MODE;               
         uint32_t RESERVE0[1];


    /**
     * DOUT
     * ===================================================================================================
     * Offset: 0x08  GPIO Data Output Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |DOUT      |Port [A/B] Pin[N] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |        |          |Note: PB_DOUT[3:0] are reserved to 0.
 */
    __IO uint32_t DOUT;               
         uint32_t RESERVE1[1];


    /**
     * PIN
     * ===================================================================================================
     * Offset: 0x10  GPIO Pin Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PIN       |Port [A/B] Pin[N] Pin Values
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low..
     * |        |          |Note: PB_PIN[3:0] are reserved to 0.
 */
    __I  uint32_t PIN;                
         uint32_t RESERVE2[1];


    /**
     * INTTYPE
     * ===================================================================================================
     * Offset: 0x18  GPIO Interrupt Trigger Type
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |TYPE      |Port [A/B] Pin[N] Edge Or Level Detection Interrupt Trigger Type Control
     * |        |          |TYPE[n] is used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled by one HCLK clock to generate the interrupt.
     * |        |          |0 = Edge triggered interrupt.
     * |        |          |1 = Level triggered interrupt.
     * |        |          |Note: If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set, the setting is ignored and no interrupt will occur.
 */
    __IO uint32_t INTTYPE;            

    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x1C  GPIO Interrupt Enable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FLIEN0    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[1]     |FLIEN1    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[2]     |FLIEN2    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[3]     |FLIEN3    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[4]     |FLIEN4    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[5]     |FLIEN5    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[6]     |FLIEN6    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[7]     |FLIEN7    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[8]     |FLIEN8    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[9]     |FLIEN9    |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[10]    |FLIEN10   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[11]    |FLIEN11   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[12]    |FLIEN12   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[13]    |FLIEN13   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[14]    |FLIEN14   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[15]    |FLIEN15   |Port [A/B] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the FLIEN[n] (Px_INTEN[n]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at low level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from high to low.
     * |        |          |0 = Disable Px.n for low-level or high-to-low interrupt.
     * |        |          |1 = Enable Px.n for low-level or high-to-low interrupt.
     * |[16]    |RHIEN0    |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[17]    |RHIEN1    |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[18]    |RHIEN2    |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[19]    |RHIEN3    |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[20]    |RHIEN4    |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[21]    |RHIEN5    |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[22]    |RHIEN6    |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[23]    |RHIEN7    |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[24]    |RHIEN8    |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[25]    |RHIEN9    |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[26]    |RHIEN10   |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[27]    |RHIEN11   |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[28]    |RHIEN12   |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[29]    |RHIEN13   |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[30]    |RHIEN14   |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
     * |[31]    |RHIEN15   |Port [A/B] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high-level interrupt for each of the corresponding input Px.n pin.
     * |        |          |To set "1" also enables the pin wake-up function..
     * |        |          |When setting the RHIEN[n] (Px_INTEN[n+16]) bit to 1 :
     * |        |          |If the interrupt is configured as level trigger mode (TYPE[n] is set to 1), one interrupt will occur while the input Px.n state is at high level.
     * |        |          |If the interrupt is configured as edge trigger mode (TYPE[n] is set to 0), one interrupt will occur while he input Px.n state changes from low to high.
     * |        |          |0 = Disable Px.n for low-to-high or level-high interrupt.
     * |        |          |1 = Enable Px.n for low-to-high or level-high interrupt.
 */
    __IO uint32_t INTEN;              

    /**
     * INTSRC
     * ===================================================================================================
     * Offset: 0x20  GPIO Interrupt Source Flag
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |INTSRC    |Port [A/B] Interrupt Source Flag
     * |        |          |Read operation:
     * |        |          |0 = No interrupt from Px.n.
     * |        |          |1 = Px.n generated an interrupt.
     * |        |          |Write operation:
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
 */
    __IO uint32_t INTSRC;             
} GPIO_T;

/**
    @addtogroup GPIO_CONST GPIO Bit Field Definition
    Constant Definitions for GPIO Controller
@{ */

#define GPIO_MODE_MODE0_Pos              (0)                                               /*!< GPIO MODE: MODE0 Position              */
#define GPIO_MODE_MODE0_Msk              (0x3ul << GPIO_MODE_MODE0_Pos)                    /*!< GPIO MODE: MODE0 Mask                  */

#define GPIO_MODE_MODE1_Pos              (2)                                               /*!< GPIO MODE: MODE1 Position              */
#define GPIO_MODE_MODE1_Msk              (0x3ul << GPIO_MODE_MODE1_Pos)                    /*!< GPIO MODE: MODE1 Mask                  */

#define GPIO_MODE_MODE2_Pos              (4)                                               /*!< GPIO MODE: MODE2 Position              */
#define GPIO_MODE_MODE2_Msk              (0x3ul << GPIO_MODE_MODE2_Pos)                    /*!< GPIO MODE: MODE2 Mask                  */

#define GPIO_MODE_MODE3_Pos              (6)                                               /*!< GPIO MODE: MODE3 Position              */
#define GPIO_MODE_MODE3_Msk              (0x3ul << GPIO_MODE_MODE3_Pos)                    /*!< GPIO MODE: MODE3 Mask                  */

#define GPIO_MODE_MODE4_Pos              (8)                                               /*!< GPIO MODE: MODE4 Position              */
#define GPIO_MODE_MODE4_Msk              (0x3ul << GPIO_MODE_MODE4_Pos)                    /*!< GPIO MODE: MODE4 Mask                  */

#define GPIO_MODE_MODE5_Pos              (10)                                              /*!< GPIO MODE: MODE5 Position              */
#define GPIO_MODE_MODE5_Msk              (0x3ul << GPIO_MODE_MODE5_Pos)                    /*!< GPIO MODE: MODE5 Mask                  */

#define GPIO_MODE_MODE6_Pos              (12)                                              /*!< GPIO MODE: MODE6 Position              */
#define GPIO_MODE_MODE6_Msk              (0x3ul << GPIO_MODE_MODE6_Pos)                    /*!< GPIO MODE: MODE6 Mask                  */

#define GPIO_MODE_MODE7_Pos              (14)                                              /*!< GPIO MODE: MODE7 Position              */
#define GPIO_MODE_MODE7_Msk              (0x3ul << GPIO_MODE_MODE7_Pos)                    /*!< GPIO MODE: MODE7 Mask                  */

#define GPIO_MODE_MODE8_Pos              (16)                                              /*!< GPIO MODE: MODE8 Position              */
#define GPIO_MODE_MODE8_Msk              (0x3ul << GPIO_MODE_MODE8_Pos)                    /*!< GPIO MODE: MODE8 Mask                  */

#define GPIO_MODE_MODE9_Pos              (18)                                              /*!< GPIO MODE: MODE9 Position              */
#define GPIO_MODE_MODE9_Msk              (0x3ul << GPIO_MODE_MODE9_Pos)                    /*!< GPIO MODE: MODE9 Mask                  */

#define GPIO_MODE_MODE10_Pos             (20)                                              /*!< GPIO MODE: MODE10 Position             */
#define GPIO_MODE_MODE10_Msk             (0x3ul << GPIO_MODE_MODE10_Pos)                   /*!< GPIO MODE: MODE10 Mask                 */

#define GPIO_MODE_MODE11_Pos             (22)                                              /*!< GPIO MODE: MODE11 Position             */
#define GPIO_MODE_MODE11_Msk             (0x3ul << GPIO_MODE_MODE11_Pos)                   /*!< GPIO MODE: MODE11 Mask                 */

#define GPIO_MODE_MODE12_Pos             (24)                                              /*!< GPIO MODE: MODE12 Position             */
#define GPIO_MODE_MODE12_Msk             (0x3ul << GPIO_MODE_MODE12_Pos)                   /*!< GPIO MODE: MODE12 Mask                 */

#define GPIO_MODE_MODE13_Pos             (26)                                              /*!< GPIO MODE: MODE13 Position             */
#define GPIO_MODE_MODE13_Msk             (0x3ul << GPIO_MODE_MODE13_Pos)                   /*!< GPIO MODE: MODE13 Mask                 */

#define GPIO_MODE_MODE14_Pos             (28)                                              /*!< GPIO MODE: MODE14 Position             */
#define GPIO_MODE_MODE14_Msk             (0x3ul << GPIO_MODE_MODE14_Pos)                   /*!< GPIO MODE: MODE14 Mask                 */

#define GPIO_MODE_MODE15_Pos             (30)                                              /*!< GPIO MODE: MODE15 Position             */
#define GPIO_MODE_MODE15_Msk             (0x3ul << GPIO_MODE_MODE15_Pos)                   /*!< GPIO MODE: MODE15 Mask                 */

#define GPIO_DOUT_DOUT_Pos               (0)                                               /*!< GPIO DOUT: DOUT Position               */
#define GPIO_DOUT_DOUT_Msk               (0xfffful << GPIO_DOUT_DOUT_Pos)                  /*!< GPIO DOUT: DOUT Mask                   */

#define GPIO_PIN_PIN_Pos                 (0)                                               /*!< GPIO PIN: PIN Position                 */
#define GPIO_PIN_PIN_Msk                 (0xfffful << GPIO_PIN_PIN_Pos)                    /*!< GPIO PIN: PIN Mask                     */

#define GPIO_INTTYPE_TYPE_Pos            (0)                                               /*!< GPIO INTTYPE: TYPE Position            */
#define GPIO_INTTYPE_TYPE_Msk            (0xfffful << GPIO_INTTYPE_TYPE_Pos)               /*!< GPIO INTTYPE: TYPE Mask                */

#define GPIO_INTEN_FLIEN0_Pos            (0)                                               /*!< GPIO INTEN: FLIEN0 Position            */
#define GPIO_INTEN_FLIEN0_Msk            (0x1ul << GPIO_INTEN_FLIEN0_Pos)                  /*!< GPIO INTEN: FLIEN0 Mask                */

#define GPIO_INTEN_FLIEN1_Pos            (1)                                               /*!< GPIO INTEN: FLIEN1 Position            */
#define GPIO_INTEN_FLIEN1_Msk            (0x1ul << GPIO_INTEN_FLIEN1_Pos)                  /*!< GPIO INTEN: FLIEN1 Mask                */

#define GPIO_INTEN_FLIEN2_Pos            (2)                                               /*!< GPIO INTEN: FLIEN2 Position            */
#define GPIO_INTEN_FLIEN2_Msk            (0x1ul << GPIO_INTEN_FLIEN2_Pos)                  /*!< GPIO INTEN: FLIEN2 Mask                */

#define GPIO_INTEN_FLIEN3_Pos            (3)                                               /*!< GPIO INTEN: FLIEN3 Position            */
#define GPIO_INTEN_FLIEN3_Msk            (0x1ul << GPIO_INTEN_FLIEN3_Pos)                  /*!< GPIO INTEN: FLIEN3 Mask                */

#define GPIO_INTEN_FLIEN4_Pos            (4)                                               /*!< GPIO INTEN: FLIEN4 Position            */
#define GPIO_INTEN_FLIEN4_Msk            (0x1ul << GPIO_INTEN_FLIEN4_Pos)                  /*!< GPIO INTEN: FLIEN4 Mask                */

#define GPIO_INTEN_FLIEN5_Pos            (5)                                               /*!< GPIO INTEN: FLIEN5 Position            */
#define GPIO_INTEN_FLIEN5_Msk            (0x1ul << GPIO_INTEN_FLIEN5_Pos)                  /*!< GPIO INTEN: FLIEN5 Mask                */

#define GPIO_INTEN_FLIEN6_Pos            (6)                                               /*!< GPIO INTEN: FLIEN6 Position            */
#define GPIO_INTEN_FLIEN6_Msk            (0x1ul << GPIO_INTEN_FLIEN6_Pos)                  /*!< GPIO INTEN: FLIEN6 Mask                */

#define GPIO_INTEN_FLIEN7_Pos            (7)                                               /*!< GPIO INTEN: FLIEN7 Position            */
#define GPIO_INTEN_FLIEN7_Msk            (0x1ul << GPIO_INTEN_FLIEN7_Pos)                  /*!< GPIO INTEN: FLIEN7 Mask                */

#define GPIO_INTEN_FLIEN8_Pos            (8)                                               /*!< GPIO INTEN: FLIEN8 Position            */
#define GPIO_INTEN_FLIEN8_Msk            (0x1ul << GPIO_INTEN_FLIEN8_Pos)                  /*!< GPIO INTEN: FLIEN8 Mask                */

#define GPIO_INTEN_FLIEN9_Pos            (9)                                               /*!< GPIO INTEN: FLIEN9 Position            */
#define GPIO_INTEN_FLIEN9_Msk            (0x1ul << GPIO_INTEN_FLIEN9_Pos)                  /*!< GPIO INTEN: FLIEN9 Mask                */

#define GPIO_INTEN_FLIEN10_Pos           (10)                                              /*!< GPIO INTEN: FLIEN10 Position           */
#define GPIO_INTEN_FLIEN10_Msk           (0x1ul << GPIO_INTEN_FLIEN10_Pos)                 /*!< GPIO INTEN: FLIEN10 Mask               */

#define GPIO_INTEN_FLIEN11_Pos           (11)                                              /*!< GPIO INTEN: FLIEN11 Position           */
#define GPIO_INTEN_FLIEN11_Msk           (0x1ul << GPIO_INTEN_FLIEN11_Pos)                 /*!< GPIO INTEN: FLIEN11 Mask               */

#define GPIO_INTEN_FLIEN12_Pos           (12)                                              /*!< GPIO INTEN: FLIEN12 Position           */
#define GPIO_INTEN_FLIEN12_Msk           (0x1ul << GPIO_INTEN_FLIEN12_Pos)                 /*!< GPIO INTEN: FLIEN12 Mask               */

#define GPIO_INTEN_FLIEN13_Pos           (13)                                              /*!< GPIO INTEN: FLIEN13 Position           */
#define GPIO_INTEN_FLIEN13_Msk           (0x1ul << GPIO_INTEN_FLIEN13_Pos)                 /*!< GPIO INTEN: FLIEN13 Mask               */

#define GPIO_INTEN_FLIEN14_Pos           (14)                                              /*!< GPIO INTEN: FLIEN14 Position           */
#define GPIO_INTEN_FLIEN14_Msk           (0x1ul << GPIO_INTEN_FLIEN14_Pos)                 /*!< GPIO INTEN: FLIEN14 Mask               */

#define GPIO_INTEN_FLIEN15_Pos           (15)                                              /*!< GPIO INTEN: FLIEN15 Position           */
#define GPIO_INTEN_FLIEN15_Msk           (0x1ul << GPIO_INTEN_FLIEN15_Pos)                 /*!< GPIO INTEN: FLIEN15 Mask               */

#define GPIO_INTEN_RHIEN0_Pos            (16)                                              /*!< GPIO INTEN: RHIEN0 Position            */
#define GPIO_INTEN_RHIEN0_Msk            (0x1ul << GPIO_INTEN_RHIEN0_Pos)                  /*!< GPIO INTEN: RHIEN0 Mask                */

#define GPIO_INTEN_RHIEN1_Pos            (17)                                              /*!< GPIO INTEN: RHIEN1 Position            */
#define GPIO_INTEN_RHIEN1_Msk            (0x1ul << GPIO_INTEN_RHIEN1_Pos)                  /*!< GPIO INTEN: RHIEN1 Mask                */

#define GPIO_INTEN_RHIEN2_Pos            (18)                                              /*!< GPIO INTEN: RHIEN2 Position            */
#define GPIO_INTEN_RHIEN2_Msk            (0x1ul << GPIO_INTEN_RHIEN2_Pos)                  /*!< GPIO INTEN: RHIEN2 Mask                */

#define GPIO_INTEN_RHIEN3_Pos            (19)                                              /*!< GPIO INTEN: RHIEN3 Position            */
#define GPIO_INTEN_RHIEN3_Msk            (0x1ul << GPIO_INTEN_RHIEN3_Pos)                  /*!< GPIO INTEN: RHIEN3 Mask                */

#define GPIO_INTEN_RHIEN4_Pos            (20)                                              /*!< GPIO INTEN: RHIEN4 Position            */
#define GPIO_INTEN_RHIEN4_Msk            (0x1ul << GPIO_INTEN_RHIEN4_Pos)                  /*!< GPIO INTEN: RHIEN4 Mask                */

#define GPIO_INTEN_RHIEN5_Pos            (21)                                              /*!< GPIO INTEN: RHIEN5 Position            */
#define GPIO_INTEN_RHIEN5_Msk            (0x1ul << GPIO_INTEN_RHIEN5_Pos)                  /*!< GPIO INTEN: RHIEN5 Mask                */

#define GPIO_INTEN_RHIEN6_Pos            (22)                                              /*!< GPIO INTEN: RHIEN6 Position            */
#define GPIO_INTEN_RHIEN6_Msk            (0x1ul << GPIO_INTEN_RHIEN6_Pos)                  /*!< GPIO INTEN: RHIEN6 Mask                */

#define GPIO_INTEN_RHIEN7_Pos            (23)                                              /*!< GPIO INTEN: RHIEN7 Position            */
#define GPIO_INTEN_RHIEN7_Msk            (0x1ul << GPIO_INTEN_RHIEN7_Pos)                  /*!< GPIO INTEN: RHIEN7 Mask                */

#define GPIO_INTEN_RHIEN8_Pos            (24)                                              /*!< GPIO INTEN: RHIEN8 Position            */
#define GPIO_INTEN_RHIEN8_Msk            (0x1ul << GPIO_INTEN_RHIEN8_Pos)                  /*!< GPIO INTEN: RHIEN8 Mask                */

#define GPIO_INTEN_RHIEN9_Pos            (25)                                              /*!< GPIO INTEN: RHIEN9 Position            */
#define GPIO_INTEN_RHIEN9_Msk            (0x1ul << GPIO_INTEN_RHIEN9_Pos)                  /*!< GPIO INTEN: RHIEN9 Mask                */

#define GPIO_INTEN_RHIEN10_Pos           (26)                                              /*!< GPIO INTEN: RHIEN10 Position           */
#define GPIO_INTEN_RHIEN10_Msk           (0x1ul << GPIO_INTEN_RHIEN10_Pos)                 /*!< GPIO INTEN: RHIEN10 Mask               */

#define GPIO_INTEN_RHIEN11_Pos           (27)                                              /*!< GPIO INTEN: RHIEN11 Position           */
#define GPIO_INTEN_RHIEN11_Msk           (0x1ul << GPIO_INTEN_RHIEN11_Pos)                 /*!< GPIO INTEN: RHIEN11 Mask               */

#define GPIO_INTEN_RHIEN12_Pos           (28)                                              /*!< GPIO INTEN: RHIEN12 Position           */
#define GPIO_INTEN_RHIEN12_Msk           (0x1ul << GPIO_INTEN_RHIEN12_Pos)                 /*!< GPIO INTEN: RHIEN12 Mask               */

#define GPIO_INTEN_RHIEN13_Pos           (29)                                              /*!< GPIO INTEN: RHIEN13 Position           */
#define GPIO_INTEN_RHIEN13_Msk           (0x1ul << GPIO_INTEN_RHIEN13_Pos)                 /*!< GPIO INTEN: RHIEN13 Mask               */

#define GPIO_INTEN_RHIEN14_Pos           (30)                                              /*!< GPIO INTEN: RHIEN14 Position           */
#define GPIO_INTEN_RHIEN14_Msk           (0x1ul << GPIO_INTEN_RHIEN14_Pos)                 /*!< GPIO INTEN: RHIEN14 Mask               */

#define GPIO_INTEN_RHIEN15_Pos           (31)                                              /*!< GPIO INTEN: RHIEN15 Position           */
#define GPIO_INTEN_RHIEN15_Msk           (0x1ul << GPIO_INTEN_RHIEN15_Pos)                 /*!< GPIO INTEN: RHIEN15 Mask               */

#define GPIO_INTSRC_INTSRC_Pos           (0)                                               /*!< GPIO INTSRC: INTSRC Position           */
#define GPIO_INTSRC_INTSRC_Msk           (0xfffful << GPIO_INTSRC_INTSRC_Pos)              /*!< GPIO INTSRC: INTSRC Mask               */

/**@}*/ /* GPIO_CONST */
/**@}*/ /* end of GPIO register group */

/*---------------------- Real Time Clock Controller -------------------------*/
/**
    @addtogroup RTC Real Time Clock Controller(RTC)
    Memory Mapped Structure for RTC Controller
@{ */
 
typedef struct
{
    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  RTC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RTIF      |RTC Interrupt Flag
     * |        |          |If the RTC interrupt is enabled, then the hardware will set this bit to indicate that the RTC interrupt has occurred.
     * |        |          |If the RTC interrupt is not enabled, then this bit indicates that a timeout period has elapsed.
     * |        |          |0 = RTC interrupt does not occur.
     * |        |          |1 = RTC interrupt occurs.
     * |        |          |Note: This bit is cleared by writing 1 to this bit.
     * |[1]     |RTIE      |RTC Interrupt Enable
     * |        |          |0 = Disable the RTC interrupt.
     * |        |          |1 = Enable the RTC interrupt.
     * |[2]     |RTCE      |RTC Enable
     * |        |          |0 = Disable RTC function.
     * |        |          |1 = Enable RTC function.
     * |[3:4]   |RTIS      |RTC Timer Interval Select
     * |        |          |These two bits select the timeout interval for the RTC.
     * |        |          |00 = Time-out frequency is 0.25Hz,.
     * |        |          |01 = Time-out frequency is 2Hz,.
     * |        |          |10 = Time-out frequency is 8Hz,.
     * |        |          |11 = Time-out frequency is 32Hz.
 */
    __IO uint32_t CTL;                   

} RTC_T;

/**
    @addtogroup RTC_CONST RTC Bit Field Definition
    Constant Definitions for RTC Controller
@{ */

#define RTC_CTL_RTIF_Pos                 (0)                                               /*!< RTC CTL: RTIF Position                 */
#define RTC_CTL_RTIF_Msk                 (0x1ul << RTC_CTL_RTIF_Pos)                       /*!< RTC CTL: RTIF Mask                     */

#define RTC_CTL_RTIE_Pos                 (1)                                               /*!< RTC CTL: RTIE Position                 */
#define RTC_CTL_RTIE_Msk                 (0x1ul << RTC_CTL_RTIE_Pos)                       /*!< RTC CTL: RTIE Mask                     */

#define RTC_CTL_RTCE_Pos                 (2)                                               /*!< RTC CTL: RTCE Position                 */
#define RTC_CTL_RTCE_Msk                 (0x1ul << RTC_CTL_RTCE_Pos)                       /*!< RTC CTL: RTCE Mask                     */

#define RTC_CTL_RTIS_Pos                 (3)                                               /*!< RTC CTL: RTIS Position                 */
#define RTC_CTL_RTIS_Msk                 (0x7ul << RTC_CTL_RTIS_Pos)                       /*!< RTC CTL: RTIS Mask                     */

/**@}*/ /* RTC_CONST */
/**@}*/ /* end of RTC register group */

/*---------------------- Flash Memory Controller -------------------------*/
/**
    @addtogroup FMC Flash Memory Controller(FMC)
    Memory Mapped Structure for FMC Controller
@{ */
 
typedef struct
{
    /**
     * ISPCTL
     * ===================================================================================================
     * Offset: 0x00  ISP Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPEN     |ISP Enable
     * |        |          |0 = Disable ISP function
     * |        |          |1 = Enable ISP function
     * |[1]     |BS        |Boot Select
     * |        |          |0 = APROM
     * |        |          |1 = LDROM
     * |        |          |Modify this bit to select which ROM next boot is to occur.
     * |        |          |This bit also functions as MCU boot status flag, which can be used to check where MCU booted from.
     * |        |          |This bit is initialized after power-on reset with the inverse of CBS in Config0; It is not reset for any other reset event.
     * |[4]     |CFGUEN    |CONFIG Update Enable
     * |        |          |0 = Disable
     * |        |          |1 = Enable
     * |        |          |When enabled, ISP functions can access the CONFIG address space and modify device configuration area. 
     * |[5]     |LDUEN     |LDROM Update Enable
     * |        |          |LDROM update enable bit.
     * |        |          |0 = LDROM cannot be updated
     * |        |          |1 = LDROM can be updated when the MCU runs in APROM.
     * |[6]     |ISPFF     |ISP Fail Flag
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself.
     * |        |          |(2) LDROM writes to itself.
     * |        |          |(3) Destination address is illegal, such as over an available range.
     * |        |          |Write 1 to clear.
     * |[7]     |SWRST     |Software Reset
     * |        |          |Writing 1 to this bit will initiate a software reset. It is cleared by hardware after reset.
 */
    __IO uint32_t ISPCTL;                

    /**
     * ISPADDR
     * ===================================================================================================
     * Offset: 0x04  ISP Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |ISPADDR   |ISP Address Register
     * |        |          |This is the memory address register that a subsequent ISP command will access.
     * |        |          |ISP operation are carried out on 32bit words only, consequently ISPARD[1:0] must be 00b for correct ISP operation.
 */
    __IO uint32_t ISPADDR;               

    /**
     * ISPDAT
     * ===================================================================================================
     * Offset: 0x08  ISP Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |ISPDAT    |ISP Data Register
     * |        |          |Write data to this register before an ISP program operation.
     * |        |          |Read data from this register after an ISP read operation
 */
    __IO uint32_t ISPDAT;                

    /**
     * ISPCMD
     * ===================================================================================================
     * Offset: 0x0C  ISP Command Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:5]   |CMD       |ISP Command
     * |        |          |Operation Mode : CMD
     * |        |          |Standby : 0x3X
     * |        |          |Read : 0x00
     * |        |          |Program : 0x21
     * |        |          |Page Erase : 0x22
     * |        |          |Read CID : 0x0B
     * |        |          |Read DID : 0x0C
 */
    __IO uint32_t ISPCMD;                

    /**
     * ISPTRG
     * ===================================================================================================
     * Offset: 0x10  ISP Trigger Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP Start Trigger
     * |        |          |Write 1 to start ISP operation.
     * |        |          |This will be cleared to 0 by hardware automatically when ISP operation is finished.
     * |        |          |0 = ISP operation is finished
     * |        |          |1 = ISP is on going
     * |        |          |After triggering an ISP function M0 instruction pipeline should be flushed with a ISB instruction to guarantee data integrity.
     * |        |          |This is a protected register, user must first follow the unlock sequence (see Protected Register Lock Key Register (SYS_REGLCTL)) to gain access.
 */
    __IO uint32_t ISPTRG;                

    /**
     * DFBA
     * ===================================================================================================
     * Offset: 0x14  Data Flash Base Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |DFBA      |Data Flash Base Address
     * |        |          |This register reports the data flash starting address. It is a read only register.
     * |        |          |Data flash size is defined by user's configuration; register content is loaded from Config1 when chip is reset.
 */
    __I  uint32_t DFBA;                  

} FMC_T;

/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for FMC Controller
@{ */

#define FMC_ISPCTL_ISPEN_Pos             (0)                                               /*!< FMC ISPCTL: ISPEN Position             */
#define FMC_ISPCTL_ISPEN_Msk             (0x1ul << FMC_ISPCTL_ISPEN_Pos)                   /*!< FMC ISPCTL: ISPEN Mask                 */

#define FMC_ISPCTL_BS_Pos                (1)                                               /*!< FMC ISPCTL: BS Position                */
#define FMC_ISPCTL_BS_Msk                (0x1ul << FMC_ISPCTL_BS_Pos)                      /*!< FMC ISPCTL: BS Mask                    */

#define FMC_ISPCTL_CFGUEN_Pos            (4)                                               /*!< FMC ISPCTL: CFGUEN Position            */
#define FMC_ISPCTL_CFGUEN_Msk            (0x1ul << FMC_ISPCTL_CFGUEN_Pos)                  /*!< FMC ISPCTL: CFGUEN Mask                */

#define FMC_ISPCTL_LDUEN_Pos             (5)                                               /*!< FMC ISPCTL: LDUEN Position             */
#define FMC_ISPCTL_LDUEN_Msk             (0x1ul << FMC_ISPCTL_LDUEN_Pos)                   /*!< FMC ISPCTL: LDUEN Mask                 */

#define FMC_ISPCTL_ISPFF_Pos             (6)                                               /*!< FMC ISPCTL: ISPFF Position             */
#define FMC_ISPCTL_ISPFF_Msk             (0x1ul << FMC_ISPCTL_ISPFF_Pos)                   /*!< FMC ISPCTL: ISPFF Mask                 */

#define FMC_ISPCTL_SWRST_Pos             (7)                                               /*!< FMC ISPCTL: SWRST Position             */
#define FMC_ISPCTL_SWRST_Msk             (0x1ul << FMC_ISPCTL_SWRST_Pos)                   /*!< FMC ISPCTL: SWRST Mask                 */

#define FMC_ISPCTL_WAITCFG_Pos           (16)                                              /*!< FMC ISPCTL: SWRST Position             */
#define FMC_ISPCTL_WAITCFG_Msk           (0x7ul << FMC_ISPCTL_WAITCFG_Pos)                 /*!< FMC ISPCTL: SWRST Mask                 */

#define FMC_ISPCTL_CACHEDIS_Pos          (21)                                              /*!< FMC ISPCTL: SWRST Position             */
#define FMC_ISPCTL_CACHEDIS_Msk          (0x1ul << FMC_ISPCTL_CACHEDIS_Pos)                /*!< FMC ISPCTL: SWRST Mask                 */

#define FMC_ISPADDR_ISPADDR_Pos          (0)                                               /*!< FMC ISPADDR: ISPADDR Position          */
#define FMC_ISPADDR_ISPADDR_Msk          (0xfffffffful << FMC_ISPADDR_ISPADDR_Pos)         /*!< FMC ISPADDR: ISPADDR Mask              */

#define FMC_ISPDAT_ISPDAT_Pos            (0)                                               /*!< FMC ISPDAT: ISPDAT Position            */
#define FMC_ISPDAT_ISPDAT_Msk            (0xfffffffful << FMC_ISPDAT_ISPDAT_Pos)           /*!< FMC ISPDAT: ISPDAT Mask                */

#define FMC_ISPCMD_CMD_Pos               (0)                                               /*!< FMC ISPCMD: CMD Position               */
#define FMC_ISPCMD_CMD_Msk               (0x3ful << FMC_ISPCMD_CMD_Pos)                    /*!< FMC ISPCMD: CMD Mask                   */

#define FMC_ISPTRG_ISPGO_Pos             (0)                                               /*!< FMC ISPTRG: ISPGO Position             */
#define FMC_ISPTRG_ISPGO_Msk             (0x1ul << FMC_ISPTRG_ISPGO_Pos)                   /*!< FMC ISPTRG: ISPGO Mask                 */

#define FMC_DFBA_DFBA_Pos                (0)                                               /*!< FMC DFBA: DFBA Position                */
#define FMC_DFBA_DFBA_Msk                (0xfffffffful << FMC_DFBA_DFBA_Pos)               /*!< FMC DFBA: DFBA Mask                    */

/**@}*/ /* FMC_CONST */
/**@}*/ /* end of FMC register group */





/**@}*/ /* end of REGISTER group */
  
/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup ISD9000_PERIPHERAL_MEM_MAP ISD9000 Peripheral Memory Map
  Memory Mapped Structure for ISD9000 Series Peripheral
  @{
 */
/* Peripheral and SRAM base address */
#define FLASH_BASE          ((     uint32_t)0x00000000)
#define SRAM_BASE           ((     uint32_t)0x20000000)
#define AHB_BASE            ((     uint32_t)0x50000000)
#define APB1_BASE           ((     uint32_t)0x40000000)

/* Peripheral memory map */
/******************************************************************************/
/*						   Peripheral memory map							  */
/******************************************************************************/
#define WDT_BASE       	     (APB1_BASE      + 0x04000)
#define RTC_BASE             (APB1_BASE      + 0x08000)
#define TIMER0_BASE          (APB1_BASE      + 0x10000)
#define TIMER1_BASE          (APB1_BASE      + 0x10020)
#define	TIMERF_BASE          (APB1_BASE      + 0x10030)
#define	TIMERIR_BASE         (APB1_BASE      + 0x10034)
#define	TIMER2_BASE          (APB1_BASE      + 0x10040)
#define SPI0_BASE            (APB1_BASE      + 0x30000)
#define PWM0_BASE            (APB1_BASE      + 0x40000)
#define PWM1_BASE            (APB1_BASE      + 0x50000)
#define UART0_BASE           (APB1_BASE      + 0x60000)
#define DPWM_BASE            (APB1_BASE      + 0x70000)
#define	ADC_BASE             (APB1_BASE      + 0xE0000)

#define SYS_BASE             (AHB_BASE       + 0x00000)
#define CLK_BASE             (AHB_BASE       + 0x00200)
#define INT_BASE             (AHB_BASE       + 0x00300)
#define MAC_BASE             (AHB_BASE       + 0x00400)
#define GPIO_BASE            (AHB_BASE       + 0x04000)
#define GPIOA_BASE           (GPIO_BASE               )
#define GPIOB_BASE           (GPIO_BASE      + 0x00040)
#define GPIO_PIN_DATA_BASE   (GPIO_BASE      + 0x00800)                   /*!< GPIO Pin Data Input/Output Control Base Address     */
#define SPIM_BASE            (AHB_BASE       + 0x07000)
#define PDMA0_BASE           (AHB_BASE       + 0x09000)                 /*!< PDMA0 Base Address                               */
#define PDMA1_BASE           (AHB_BASE       + 0x09100)                 /*!< PDMA1 Base Address                               */
#define PDMA_GCR_BASE        (AHB_BASE       + 0x09F00)                 /*!< PDMA Grobal Base Address                         */
#define FMC_BASE             (AHB_BASE       + 0x0C000)

/*@}*/ /* end of group ISD9000_PERIPHERAL_MEM_MAP */
/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
/** @addtogroup ISD9000_PeripheralDecl ISD9000 Peripheral Declaration
    @{
*/
#define WDT                 ((WDT_T *) WDT_BASE)
#define RTC                 ((RTC_T *) RTC_BASE)
#define TIMER0              ((TMR_T *) TIMER0_BASE)
#define TIMER1              ((TMR_T *) TIMER1_BASE)
#define TIMERF              ((TMRF_T *) TIMERF_BASE)
#define TIMER_IR            ((TMRIR_T *) TIMERIR_BASE)
#define TIMER2              ((TMR_T *) TIMER2_BASE)
#define SPI0                ((SPI_T *) SPI0_BASE)
#define PWM0                ((PWM_T *) PWM0_BASE)
#define PWM1                ((PWM_T *) PWM1_BASE)
#define DPWM                ((DPWM_T *) DPWM_BASE)
#define ADC                 ((ADC_T*)  ADC_BASE)

#define SYS                 ((SYS_T *) SYS_BASE)
#define SYSINT              ((SYSINT_T *) INT_BASE)
#define CLK                 ((CLK_T *) CLK_BASE)
#define MAC                 ((MAC_T *) MAC_BASE)
#define PA                  ((GPIO_T *) GPIOA_BASE)
#define PB                  ((GPIO_T *) GPIOB_BASE)
#define SPIM                ((SPIM_T *) SPIM_BASE)
#define PDMA0               ((PDMA_T *) PDMA0_BASE)                     /*!< PDMA0 Configuration Struct                       */
#define PDMA1               ((PDMA_T *) PDMA1_BASE)                     /*!< PDMA1 Configuration Struct                       */
#define PDMA_GCR            ((PDMA_GCR_T *) PDMA_GCR_BASE)              /*!< PDMA Global Configuration Struct                 */
#define FMC                 ((FMC_T *) FMC_BASE)
#define UART0               ((UART_T *) UART0_BASE)

/*@}*/ /* end of group ISD9000_PeripheralDecl */

#define UNLOCKREG(x)        do{*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x59;*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x16;*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x88;}while(*((__IO uint32_t *)(SYS_BASE + 0x100))==0)
#define LOCKREG(x)          *((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x00

#define REGCOPY(dest, src)  *((uint32_t *)&(dest)) = *((uint32_t *)&(src))
#define CLEAR(dest)         *((uint32_t *)&(dest)) = 0


//=============================================================================
/** @addtogroup ISD9000_IO_ROUTINE ISD9000 I/O routines
  The Declaration of ISD9000 I/O routines
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

/*@}*/ /* end of group ISD9000_IO_ROUTINE */


/** @addtogroup ISD9000_legacy_Constants ISD9000 Legacy Constants
  ISD9000 Legacy Constants
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

#define _GET_BYTE0(u32Param)    ((u32Param & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define _GET_BYTE1(u32Param)    ((u32Param & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define _GET_BYTE2(u32Param)    ((u32Param & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define _GET_BYTE3(u32Param)    ((u32Param & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/*@}*/ /* end of group ISD9000_legacy_Constants */

/*@}*/ /* end of group ISD9000_Definitions */

#define __ISD9000_SERIES__ (0x90000032)
#define __CHIP_SERIES__ __ISD9000_SERIES__

/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "sys.h"
#include "clk.h"
#include "spi.h"
#include "dpwm.h"
#include "gpio.h"
#include "bod.h"
#include "timer.h"
#include "pwm.h"
#include "fmc.h"
#include "wdt.h"
#include "rtc.h"
#include "spim.h"
#include "pdma.h"
#include "adc.h"
#include "uart.h"
//#include "mac.h"

#endif	// __ISD9000_H__
