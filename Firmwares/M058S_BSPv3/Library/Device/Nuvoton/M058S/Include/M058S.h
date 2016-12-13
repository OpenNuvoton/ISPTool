/**************************************************************************//**
 * @file     M058S.h
 * @version  V3.0
 * $Revision: 48 $
 * $Date: 15/05/07 11:12a $
 * @brief    M058S Series Peripheral Access Layer Header File
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/

/**
  \mainpage Introduction
  *
  *
  * This user manual describes the usage of M051DN/DE Series MCU device driver
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
  * \page PG_REV Revision History
  *
  * <b>Revision 3.00.002</b>
  * \li Fix UART, I2C, SYS definition error in header file.
  * \li Fix reset entry pointer setting of IAR projects.
  * \li Add VECMAP relative API to FMC driver.
  * \li Add MCUIRQ control register to GCR_INT_T of header file.
  * \li Add CAPIC control register to PWM_T of header file.
  * \li Add ISPSTA control register to FMC_T of header file.
  *
  * <b>Revision 3.00.001</b>
  * \li Initial release.
*/

#ifndef __M058S_H__
#define __M058S_H__


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
    NonMaskableInt_IRQn       = -14,      /*!< 2 Non Maskable Interrupt                             */
    HardFault_IRQn            = -13,      /*!< 3 Cortex-M0 Hard Fault Interrupt                     */
    SVCall_IRQn               = -5,       /*!< 11 Cortex-M0 SV Call Interrupt                       */
    PendSV_IRQn               = -2,       /*!< 14 Cortex-M0 Pend SV Interrupt                       */
    SysTick_IRQn              = -1,       /*!< 15 Cortex-M0 System Tick Interrupt                   */

    /******  ARMIKMCU Swift specific Interrupt Numbers ************************************************/
    BOD_IRQn                  = 0,        /*!< Brown-Out Low Voltage Detected Interrupt             */
    WDT_IRQn                  = 1,        /*!< Watch Dog Timer Interrupt                            */
    EINT0_IRQn                = 2,        /*!< EINT0 Interrupt                                      */
    EINT1_IRQn                = 3,        /*!< EINT1 Interrupt                                      */
    GPIO_P0P1_IRQn            = 4,        /*!< GPIO_P0P1 Interrupt                                  */
    GPIO_P2P3P4_IRQn          = 5,        /*!< GPIO_P2P3P4 Interrupt                                */
    PWMA_IRQn                 = 6,        /*!< PWMA Interrupt                                       */
    TMR0_IRQn                 = 8,        /*!< TIMER0 Interrupt                                     */
    TMR1_IRQn                 = 9,        /*!< TIMER1 Interrupt                                     */
    TMR2_IRQn                 = 10,       /*!< TIMER2 Interrupt                                     */
    TMR3_IRQn                 = 11,       /*!< TIMER3 Interrupt                                     */
    UART0_IRQn                = 12,       /*!< UART0 Interrupt                                      */
    SPI0_IRQn                 = 14,       /*!< SPI0 Interrupt                                       */
    GPIO_P5_IRQn              = 16,       /*!< GPIO_P5 Interrupt                                    */
    GPIO_P6P7_IRQn            = 17,       /*!< GPIO_P6P7 Interrupt                                  */
    I2C0_IRQn                 = 18,       /*!< I2C0 Interrupt                                       */
    I2C1_IRQn                 = 19,       /*!< I2C1 Interrupt                                       */
    PWRWU_IRQn                = 28,       /*!< Power Down Wake Up Interrupt                         */
    ADC_IRQn                  = 29        /*!< ADC Interrupt                                        */

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


#include "core_cm0.h"                   /*!< Cortex-M0 processor and core peripherals             */
#include "system_M058S.h"          /*!< M058S System                                    */


#if defined ( __CC_ARM   )
#pragma anon_unions
#endif


/**
 * Initialize the system clock
 *
 * @param  none
 * @return none
 *
 * @brief  Setup the microcontroller system
 *         Initialize the PLL and update the SystemFrequency variable
 */
extern void SystemInit(void);


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

  @{

*/


/*----------------------------- ADC Controller -------------------------------*/
/** @addtogroup ADC Analog to Digital Converter(ADC)
  Memory Mapped Structure for ADC Controller
  @{
 */

typedef struct
{


/**
 * @var ADC_T::ADDR
 * Offset: 0x00-0x1C  ADC Data Register 0~7
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |RSLT      |A/D Conversion Result(Read Only)
 * |        |          |This field contains conversion result of ADC.
 * |[16]    |OVERRUN   |Over Run Flag(Read Only)
 * |        |          |0 = Data in RSLT is not overwrote.
 * |        |          |1 = Data in RSLT is overwrote.
 * |        |          |If converted data in RSLT has not been read before new conversion result is
 * |        |          |loaded to this register, OVERRUN is set to 1. It will be cleared by hardware after
 * |        |          |ADDR register is read.
 * |[17]    |VALID     |Valid Flag(Read Only)
 * |        |          |0 = Data in RSLT bits is not valid.
 * |        |          |1 = Data in RSLT bits is valid.
 * |        |          |This bit will be set to 1 when the conversion of the corresponding channel is
 * |        |          |completed. This bit will be cleared to 0 by hardware after ADDR register is read.
 * |        |          |Note: When ADC is converting, if user wants to monitor the VALID flag of a specified channel,
 * |        |          |user should poll the VALID bit of ADSR register instead of polling this bit.
 * @var ADC_T::ADCR
 * Offset: 0x20  A/D Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADEN      |A/D Converter Enable
 * |        |          |0 = A/D converter Disabled.
 * |        |          |1 = A/D converter Enabled.
 * |        |          |Note: Before starting A/D conversion function, this bit should be set to 1. Clear it
 * |        |          |to 0 to disable A/D converter analog circuit to save power consumption.
 * |[1]     |ADIE      |A/D Interrupt Enable Control
 * |        |          |A/D conversion end interrupt request is generated if ADIE bit is set to 1.
 * |        |          |0 = A/D interrupt function Disabled.
 * |        |          |1 = A/D interrupt function Enabled.
 * |[3:2]   |ADMD      |A/D Converter Operation Mode Control
 * |        |          |00 = Single conversion.
 * |        |          |01 = Burst conversion.
 * |        |          |10 = Single-cycle scan.
 * |        |          |11 = Continuous scan.
 * |        |          |Note1: When changing the operation mode, software should clear ADST bit firstly.
 * |        |          |Note2: In Burst mode, the A/D result data always at Data Register 0.
 * |[5:4]   |TRGS      |Hardware Trigger Source
 * |        |          |00 = A/D conversion is started by external STADC pin.
 * |        |          |11 = A/D conversion is started by PWM trigger.
 * |        |          |Others = Reserved
 * |        |          |Note: Software should clear TRGEN bit and ADST bit to 0 before changing TRGS.
 * |[7:6]   |TRGCOND   |External Trigger Condition
 * |        |          |These two bits decide external pin STADC trigger event is level or edge. The signal
 * |        |          |must be kept at stable state at least 8 PCLKs for level trigger and at least
 * |        |          |4 PCLKs for edge trigger.
 * |        |          |00 = Low level.
 * |        |          |01 = High level.
 * |        |          |10 = Falling edge.
 * |        |          |11 = Rising edge.
 * |[8]     |TRGEN     |External Trigger Enable
 * |        |          |Enable or disable triggering of A/D conversion by external STADC pin.
 * |        |          |If external trigger is enabled, the ADST bit can be set to 1 by the selected hardware trigger source.
 * |        |          |0= External trigger Disabled.
 * |        |          |1= External trigger Enabled.
 * |[10]    |DIFFEN    |A/D Differential Input Mode Enable
 * |        |          |0 = Single-end analog input mode.
 * |        |          |1 = Differential analog input mode.
 * |        |          |Differential input voltage (Vdiff) = Vplus - Vminus,
 * |        |          |where Vplus is the analog input; Vminus is the inverted analog input.
 * |        |          |The Vplus of differential input paired channel 0 is from ADC0 pin; Vminus is from ADC1 pin.
 * |        |          |The Vplus of differential input paired channel 1 is from ADC2 pin; Vminus is from ADC3 pin.
 * |        |          |The Vplus of differential input paired channel 2 is from ADC4 pin; Vminus is from ADC5 pin.
 * |        |          |The Vplus of differential input paired channel 3 is from ADC6 pin; Vminus is from ADC7 pin.
 * |        |          |Note: In Differential Input mode, only the even number of the two corresponding
 * |        |          |channels needs to be enabled in ADCHER register. The conversion result
 * |        |          |will be placed to the corresponding data register of the enabled channel.
 * |[11]    |ADST      |A/D Conversion Start
 * |        |          |ADST bit can be set to 1 from two sources: software or external pin STADC.
 * |        |          |ADST will be cleared to 0 by hardware automatically at the ends of Single mode
 * |        |          |and Single-cycle Scan mode. In Continuous Scan mode and Burst mode, A/D
 * |        |          |conversion is continuously performed until software writes 0 to this bit or chip is reset.
 * |        |          |0 = Conversion stops and A/D converter enters idle state.
 * |        |          |1 = Conversion starts.
 * |[31]    |DMOF      |Differential Input Mode Output Format
 * |        |          |0 = A/D Conversion result will be filled in RSLT at ADDRx registers with unsigned format (straight binary format).
 * |        |          |1 = A/D Conversion result will be filled in RSLT at ADDRx registers with 2's complement format.
 * @var ADC_T::ADCHER
 * Offset: 0x24  A/D Channel Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |CHEN      |Analog Input Channel Enable Control
 * |        |          |Set CHEN[7:0] to enable the corresponding analog input channel 7 ~ 0. If DIFFEN
 * |        |          |bit (ADCR[10]) is set to 1, only the even number channels need to be enabled.
 * |        |          |0 = ADC input channel Disabled.
 * |        |          |1 = ADC input channel Enabled.
 * |[9:8]   |PRESEL    |Analog Input Channel 7 Source Selection
 * |        |          |00 = External analog input.
 * |        |          |01 = Internal band-gap voltage.
 * |        |          |10 = Internal temperature sensor.
 * |        |          |11 = Reserved.
 * |        |          |Note: When the band-gap voltage is selected as the analog input source of ADC channel 7,
 * |        |          |ADC peripheral clock rate needs to be limited to lower than 300 kHz.
 * @var ADC_T::ADCMPR
 * Offset: 0x28,0x2C  A/D Compare Register 0 & 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CMPEN     |Compare Enable Control
 * |        |          |Set this bit to 1 to enable compare CMPD[11:0] with specified channel conversion
 * |        |          |result when converted data is loaded into ADDR register.
 * |        |          |0 = Compare function Disabled.
 * |        |          |1 = Compare function Enabled.
 * |[1]     |CMPIE     |Compare Interrupt Enable Control
 * |        |          |If the compare function is enabled and the compare condition matches the settings
 * |        |          |of CMPCOND and CMPMATCNT, CMPFx bit will be asserted, in the meanwhile, If CMPIE is set to
 * |        |          |1, a compare interrupt request is generated.
 * |        |          |0 = Compare function interrupt Disabled.
 * |        |          |1 = Compare function interrupt Enabled.
 * |[2]     |CMPCOND   |Compare Condition
 * |        |          |0= Set the compare condition as that when a 12-bit A/D conversion result is less
 * |        |          |than the 12-bit CMPD (ADCMPRx[27:16]), the internal match counter will
 * |        |          |increase one.
 * |        |          |1= Set the compare condition as that when a 12-bit A/D conversion result is
 * |        |          |greater than or equal to the 12-bit CMPD (ADCMPRx[27:16]), the internal
 * |        |          |match counter will increase one.
 * |        |          |Note: When the internal counter reaches to (CMPMATCNT +1), the CMPFx bit will be set.
 * |[5:3]   |CMPCH     |Compare Channel Selection
 * |        |          |000 = Channel 0 conversion result is selected to be compared.
 * |        |          |001 = Channel 1 conversion result is selected to be compared.
 * |        |          |010 = Channel 2 conversion result is selected to be compared.
 * |        |          |011 = Channel 3 conversion result is selected to be compared.
 * |        |          |100 = Channel 4 conversion result is selected to be compared.
 * |        |          |101 = Channel 5 conversion result is selected to be compared.
 * |        |          |110 = Channel 6 conversion result is selected to be compared.
 * |        |          |111 = Channel 7 conversion result is selected to be compared.
 * |[11:8]  |CMPMATCNT |Compare Match Count
 * |        |          |When the specified A/D channel analog conversion result matches the compare
 * |        |          |condition defined by CMPCOND[2], the internal match counter will increase 1.
 * |        |          |When the internal counter reaches the value to (CMPMATCNT +1), the CMPFx bit
 * |        |          |will be set.
 * |[27:16] |CMPD      |Comparison Data
 * |        |          |The 12 bits data is used to compare with conversion result of specified channel.
 * |        |          |Note: CMPD should be filled in unsigned format (straight binary format).
 * @var ADC_T::ADSR
 * Offset: 0x30  ADC Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADF       |A/D Conversion End Flag
 * |        |          |A status flag that indicates the end of A/D conversion. Software can write 1 to clear this bit.
 * |        |          |ADF is set to 1 at these two conditions:
 * |        |          |1. When A/D conversion ends in single mode
 * |        |          |2. When A/D conversion ends on all specified channels in Single-cycle Scan mode and Continuous Scan mode.
 * |        |          |3. When more than 4 samples in FIFO in Burst mode.
 * |[1]     |CMPF0     |Compare Flag 0
 * |        |          |When the A/D conversion result of the selected channel meets setting condition
 * |        |          |in ADCMPR0 then this bit is set to 1. This bit is cleared by writing 1 to it.
 * |        |          |0 = Conversion result in ADDR does not meet ADCMPR0 setting.
 * |        |          |1 = Conversion result in ADDR meets ADCMPR0 setting.
 * |[2]     |CMPF1     |Compare Flag 1
 * |        |          |When the A/D conversion result of the selected channel meets setting condition
 * |        |          |in ADCMPR1 then this bit is set to 1; it is cleared by writing 1 to self.
 * |        |          |0 = Conversion result in ADDR does not meet ADCMPR1 setting.
 * |        |          |1 = Conversion result in ADDR meets ADCMPR1 setting.
 * |[3]     |BUSY      |BUSY/IDLE
 * |        |          |This bit is a mirror of ADST bit in ADCR register. It is read only.
 * |        |          |0 = A/D converter is in idle state.
 * |        |          |1 = A/D converter is busy at conversion.
 * |[6:4]   |CHANNEL   |Current Conversion Channel
 * |        |          |When BUSY=1, this filed reflects current conversion channel. When BUSY=0, it
 * |        |          |shows the number of the next converted channel.
 * |        |          |It is read only.
 * |[15:8]  |VALID     |Data Valid flag(Read Only)
 * |        |          |It is a mirror of VALID bit in ADDRx register.
 * |        |          |When ADC is in Burst mode and any conversion result is valid, all bits of
 * |        |          |VALID[7:0] will be set to 1.
 * |[23:16] |OVERRUN   |Overrun flag(Read Only)
 * |        |          |It is a mirror to OVERRUN bit in ADDRx register.
 * |        |          |When ADC is in Burst mode and the FIFO is overrun, all bits of OVERRUN[7:0] will be set to 1.
 * @var ADC_T::ADTDCR
 * Offset: 0x44  ADC Trigger Delay Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |PTDT      |PWM Trigger Delay Time
 * |        |          |Set this field will delay ADC start conversion time after PWM trigger.
 * |        |          |PWM trigger delay time is (4 * PTDT) * system clock
 */

    __I  uint32_t ADDR[8];       /* Offset: 0x00-0x1C  ADC Data Register 0~7                                         */
    __IO uint32_t ADCR;          /* Offset: 0x20  A/D Control Register                                               */
    __IO uint32_t ADCHER;        /* Offset: 0x24  A/D Channel Enable Register                                        */
    __IO uint32_t ADCMPR[2];     /* Offset: 0x28,0x2C  A/D Compare Register 0 & 1                                    */
    __IO uint32_t ADSR;          /* Offset: 0x30  ADC Status Register                                                */
    __I  uint32_t RESERVED0[4]; 
    __IO uint32_t ADTDCR;        /* Offset: 0x44  ADC Trigger Delay Control Register                                 */

} ADC_T;




/** @addtogroup ADC_CONST ADC Bit Field Definition
  Constant Definitions for ADC Controller
  @{
 */
/* ADDR Bit Field Definitions */
#define ADC_ADDR_VALID_Pos      17                                       /*!< ADC_T::ADDR: VALID Position */
#define ADC_ADDR_VALID_Msk      (1ul << ADC_ADDR_VALID_Pos)              /*!< ADC_T::ADDR: VALID Mask */

#define ADC_ADDR_OVERRUN_Pos    16                                       /*!< ADC_T::ADDR: OVERRUN Position */
#define ADC_ADDR_OVERRUN_Msk    (1ul << ADC_ADDR_OVERRUN_Pos)            /*!< ADC_T::ADDR: OVERRUN Mask */

#define ADC_ADDR_RSLT_Pos       0                                        /*!< ADC_T::ADDR: RSLT Position */
#define ADC_ADDR_RSLT_Msk       (0xFFFFul << ADC_ADDR_RSLT_Pos)          /*!< ADC_T::ADDR: RSLT Mask */

/* ADCR Bit Field Definitions */
#define ADC_ADCR_DMOF_Pos       31                                       /*!< ADC_T::ADCR: DMOF Position */
#define ADC_ADCR_DMOF_Msk       (1ul << ADC_ADCR_DMOF_Pos)               /*!< ADC_T::ADCR: DMOF Mask */

#define ADC_ADCR_ADST_Pos       11                                       /*!< ADC_T::ADCR: ADST Position */
#define ADC_ADCR_ADST_Msk       (1ul << ADC_ADCR_ADST_Pos)               /*!< ADC_T::ADCR: ADST Mask */

#define ADC_ADCR_DIFFEN_Pos     10                                       /*!< ADC_T::ADCR: DIFFEN Position */
#define ADC_ADCR_DIFFEN_Msk     (1ul << ADC_ADCR_DIFFEN_Pos)             /*!< ADC_T::ADCR: DIFFEN Mask */

#define ADC_ADCR_TRGEN_Pos      8                                        /*!< ADC_T::ADCR: TRGEN Position */
#define ADC_ADCR_TRGEN_Msk      (1ul << ADC_ADCR_TRGEN_Pos)              /*!< ADC_T::ADCR: TRGEN Mask */

#define ADC_ADCR_TRGCOND_Pos    6                                        /*!< ADC_T::ADCR: TRGCOND Position */
#define ADC_ADCR_TRGCOND_Msk    (3ul << ADC_ADCR_TRGCOND_Pos)            /*!< ADC_T::ADCR: TRGCOND Mask */

#define ADC_ADCR_TRGS_Pos       4                                        /*!< ADC_T::ADCR: TRGS Position */
#define ADC_ADCR_TRGS_Msk       (3ul << ADC_ADCR_TRGS_Pos)               /*!< ADC_T::ADCR: TRGS Mask */

#define ADC_ADCR_ADMD_Pos       2                                        /*!< ADC_T::ADCR: ADMD Position */
#define ADC_ADCR_ADMD_Msk       (3ul << ADC_ADCR_ADMD_Pos)               /*!< ADC_T::ADCR: ADMD Mask */

#define ADC_ADCR_ADIE_Pos       1                                        /*!< ADC_T::ADCR: ADIE Position */
#define ADC_ADCR_ADIE_Msk       (1ul << ADC_ADCR_ADIE_Pos)               /*!< ADC_T::ADCR: ADIE Mask */

#define ADC_ADCR_ADEN_Pos       0                                        /*!< ADC_T::ADCR: ADEN Position */
#define ADC_ADCR_ADEN_Msk       (1ul << ADC_ADCR_ADEN_Pos)               /*!< ADC_T::ADCR: ADEN Mask */

/* ADCHER Bit Field Definitions */
#define ADC_ADCHER_PRESEL_Pos   8                                        /*!< ADC_T::ADCHER: PRESEL Position */
#define ADC_ADCHER_PRESEL_Msk   (3ul << ADC_ADCHER_PRESEL_Pos)           /*!< ADC_T::ADCHER: PRESEL Mask */

#define ADC_ADCHER_CHEN_Pos     0                                        /*!< ADC_T::ADCHER: CHEN Position */
#define ADC_ADCHER_CHEN_Msk     (0xFFul << ADC_ADCHER_CHEN_Pos)          /*!< ADC_T::ADCHER: CHEN Mask */

/* ADCMPR Bit Field Definitions */
#define ADC_ADCMPR_CMPD_Pos        16                                    /*!< ADC_T::ADCMPR: CMPD Position */
#define ADC_ADCMPR_CMPD_Msk        (0xFFFul << ADC_ADCMPR_CMPD_Pos)      /*!< ADC_T::ADCMPR: CMPD Mask */

#define ADC_ADCMPR_CMPMATCNT_Pos   8                                     /*!< ADC_T::ADCMPR: CMPMATCNT Position */
#define ADC_ADCMPR_CMPMATCNT_Msk   (0xFul << ADC_ADCMPR_CMPMATCNT_Pos)   /*!< ADC_T::ADCMPR: CMPMATCNT Mask */

#define ADC_ADCMPR_CMPCH_Pos       3                                     /*!< ADC_T::ADCMPR: CMPCH Position */
#define ADC_ADCMPR_CMPCH_Msk       (7ul << ADC_ADCMPR_CMPCH_Pos)         /*!< ADC_T::ADCMPR: CMPCH Mask */

#define ADC_ADCMPR_CMPCOND_Pos     2                                     /*!< ADC_T::ADCMPR: CMPCOND Position */
#define ADC_ADCMPR_CMPCOND_Msk     (1ul << ADC_ADCMPR_CMPCOND_Pos)       /*!< ADC_T::ADCMPR: CMPCOND Mask */

#define ADC_ADCMPR_CMPIE_Pos       1                                     /*!< ADC_T::ADCMPR: CMPIE Position */
#define ADC_ADCMPR_CMPIE_Msk       (1ul << ADC_ADCMPR_CMPIE_Pos)         /*!< ADC_T::ADCMPR: CMPIE Mask */

#define ADC_ADCMPR_CMPEN_Pos       0                                     /*!< ADC_T::ADCMPR: CMPEN Position */
#define ADC_ADCMPR_CMPEN_Msk       (1ul << ADC_ADCMPR_CMPEN_Pos)         /*!< ADC_T::ADCMPR: CMPEN Mask */

/* ADSR Bit Field Definitions */
#define ADC_ADSR_OVERRUN_Pos       16                                    /*!< ADC_T::ADSR: OVERRUN Position */
#define ADC_ADSR_OVERRUN_Msk       (0xFFul << ADC_ADSR_OVERRUN_Pos)      /*!< ADC_T::ADSR: OVERRUN Mask */

#define ADC_ADSR_VALID_Pos         8                                     /*!< ADC_T::ADSR: VALID Position */
#define ADC_ADSR_VALID_Msk         (0xFFul << ADC_ADSR_VALID_Pos)        /*!< ADC_T::ADSR: VALID Mask */

#define ADC_ADSR_CHANNEL_Pos       4                                     /*!< ADC_T::ADSR: CHANNEL Position */
#define ADC_ADSR_CHANNEL_Msk       (7ul << ADC_ADSR_CHANNEL_Pos)         /*!< ADC_T::ADSR: CHANNEL Mask */

#define ADC_ADSR_BUSY_Pos          3                                     /*!< ADC_T::ADSR: BUSY Position */
#define ADC_ADSR_BUSY_Msk          (1ul << ADC_ADSR_BUSY_Pos)            /*!< ADC_T::ADSR: BUSY Mask */

#define ADC_ADSR_CMPF1_Pos         2                                     /*!< ADC_T::ADSR: CMPF1 Position */
#define ADC_ADSR_CMPF1_Msk         (1ul << ADC_ADSR_CMPF1_Pos)           /*!< ADC_T::ADSR: CMPF1 Mask */

#define ADC_ADSR_CMPF0_Pos         1                                     /*!< ADC_T::ADSR: CMPF0 Position */
#define ADC_ADSR_CMPF0_Msk         (1ul << ADC_ADSR_CMPF0_Pos)           /*!< ADC_T::ADSR: CMPF0 Mask */

#define ADC_ADSR_ADF_Pos           0                                     /*!< ADC_T::ADSR: ADF Position */
#define ADC_ADSR_ADF_Msk           (1ul << ADC_ADSR_ADF_Pos)             /*!< ADC_T::ADSR: ADF Mask */

/* ADTDCR Bit Field Definitions */
#define ADC_ADTDCR_PTDT_Pos        0                                     /*!< ADC_T::ADTDCR: PTDT Position */
#define ADC_ADTDCR_PTDT_Msk        (0xFFul << ADC_ADTDCR_PTDT_Pos)       /*!< ADC_T::ADTDCR: PTDT Mask */
/**@}*/ /* ADC_CONST */
/**@}*/ /* ADC */




/*-------------------------- FLASH Memory Controller -------------------------*/

/** @addtogroup FMC Flash Memory Controller(FMC)
 *
 * Memory Mapped Structure for Flash Memory Controller
 * @{
 */

typedef struct
{


/**
 * @var FMC_T::ISPCON
 * Offset: 0x00  ISP Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPEN     |ISP Enable
 * |        |          |ISP function enable bit. Set this bit to enable ISP function.
 * |        |          |0 = Disable ISP function
 * |        |          |1 = Enable ISP function
 * |        |          |Note: This bit is write protected.
 * |[1]     |BS        |Boot Select
 * |        |          |Set/clear this bit to select next booting from LDROM/APROM,
 * |        |          |respectively. This bit also functions as MCU booting status flag, which can be used to check where
 * |        |          |MCU booted from. This bit is initiated with the inverted value of CBS in Config0 after power-
 * |        |          |on reset; It keeps the same value at other reset.
 * |        |          |0 = boot from APROM
 * |        |          |1 = boot from LDROM
 * |        |          |Note: This bit is write protected.
 * |[3]     |APUEN     |APROM Update Enable
 * |        |          |0 = APROM cannot be updated when the chip runs in APROM
 * |        |          |1 = APROM can be updated when the chip runs in APROM
 * |        |          |Note: This bit is write protected.
 * |[4]     |CFGUEN    |Config Update Enable
 * |        |          |Writing this bit to 1 enables s/w to update Config value by ISP procedure regardless of program
 * |        |          |code is running in APROM or LDROM.
 * |        |          |0 = Config update disable
 * |        |          |1 = Config update enable
 * |        |          |Note: This bit is write protected.
 * |[5]     |LDUEN     |LDROM Update Enable
 * |        |          |LDROM update enable bit.
 * |        |          |0 = LDROM cannot be updated
 * |        |          |1 = LDROM can be updated.
 * |        |          |Note: This bit is write protected.
 * |[6]     |ISPFF     |ISP Fail Flag
 * |        |          |(1) APROM writes to itself.
 * |        |          |(2) LDROM writes to itself.
 * |        |          |(3) CONFIG is erased/programmed when CFGUEN is set to 0
 * |        |          |(4) Destination address is illegal, such as over an available range.
 * |        |          |Note: Write 1 to clear this bit.
 * @var FMC_T::ISPADR
 * Offset: 0x04  ISP Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ISPADR    |ISP Address
 * |        |          |NuMicro M058S series equips with a maximum 16kx32 embedded flash, it supports word program
 * |        |          |only. ISPARD[1:0] must be kept 2'b00 for ISP operation.
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
 * |        |          |ISP command table is shown below:
 * |        |          |0x00 = Read
 * |        |          |0x21 = Program
 * |        |          |0x22 = Page Erase
 * @var FMC_T::ISPTRG
 * Offset: 0x10  IISP Trigger Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPGO     |ISP Start Trigger
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
 * |        |          |It is a read only register.
 * |        |          |The data flash size is 4kB and it start address is fixed at
 * |        |          |0x01F000 by hardware internally.
 * @var FMC_T::FATCON
 * Offset: 0x18  Flash Access Time Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[4]     |LFOM      |Low Frequency Optimization Mode
 * |        |          |When chip operation frequency is lower than 25MHz, chip can work efficiently by setting this bit to 1.
 * |        |          |0 = Insert wait state while Flash access discontinued address.
 * |        |          |1 = Flash access always no wait state (zero wait state)
 * |        |          |Note: This bit is write protected.
 * @var FMC_T::ISPSTA
 * Offset: 0x40  Flash Access Time Control Register
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
    __IO uint32_t ISPTRG;        /* Offset: 0x10  IISP Trigger Control Register                                      */
    __I  uint32_t DFBADR;        /* Offset: 0x14  Data Flash Base Address Register                                   */
    __IO uint32_t FATCON;        /* Offset: 0x18  Flash Access Time Control Register                                 */
    __I  uint32_t RESERVED[9];  
    __IO uint32_t ISPSTA;        /* Offset: 0x40  ISPSTA control register of FMC                                     */
    

} FMC_T;



/** @addtogroup FMC_CONST FMC Bit Field Definition
  Constant Definitions for FMC Controller
  @{
 */
/* FMC ISPCON Bit Field Definitions */
#define FMC_ISPCON_ISPFF_Pos                    6                                       /*!< FMC_T::ISPCON: ISPFF Position  */
#define FMC_ISPCON_ISPFF_Msk                    (1ul << FMC_ISPCON_ISPFF_Pos)           /*!< FMC_T::ISPCON: ISPFF Mask      */

#define FMC_ISPCON_LDUEN_Pos                    5                                       /*!< FMC_T::ISPCON: LDUEN Position  */
#define FMC_ISPCON_LDUEN_Msk                    (1ul << FMC_ISPCON_LDUEN_Pos)           /*!< FMC_T::ISPCON: LDUEN Mask      */

#define FMC_ISPCON_CFGUEN_Pos                   4                                       /*!< FMC_T::ISPCON: CFGUEN Position */
#define FMC_ISPCON_CFGUEN_Msk                   (1ul << FMC_ISPCON_CFGUEN_Pos)          /*!< FMC_T::ISPCON: CFGUEN Mask     */

#define FMC_ISPCON_APUEN_Pos                    3                                       /*!< FMC_T::ISPCON: APUEN Position  */
#define FMC_ISPCON_APUEN_Msk                    (1ul << FMC_ISPCON_APUEN_Pos)           /*!< FMC_T::ISPCON: APUEN Mask      */

#define FMC_ISPCON_BS_Pos                       1                                       /*!< FMC_T::ISPCON: BS Position     */
#define FMC_ISPCON_BS_Msk                       (1ul << FMC_ISPCON_BS_Pos)              /*!< FMC_T::ISPCON: BS Mask         */

#define FMC_ISPCON_ISPEN_Pos                    0                                       /*!< FMC_T::ISPCON: ISPEN Position  */
#define FMC_ISPCON_ISPEN_Msk                    (1ul << FMC_ISPCON_ISPEN_Pos)           /*!< FMC_T::ISPCON: ISPEN Mask      */

/* FMC ISPADR Bit Field Definitions */
#define FMC_ISPADR_ISPADR_Pos                   0                                       /*!< FMC_T::ISPADR: ISPADR Position */
#define FMC_ISPADR_ISPADR_Msk                   (0xFFFFFFFFul << FMC_ISPADR_ISPADR_Pos) /*!< FMC_T::ISPADR: ISPADR Mask     */

/* FMC ISPDAT Bit Field Definitions */
#define FMC_ISPDAT_ISPDAT_Pos                   0                                       /*!< FMC_T::ISPDAT: ISPDAT Position */
#define FMC_ISPDAT_ISPDAT_Msk                   (0xFFFFFFFFul << FMC_ISPDAT_ISPDAT_Pos) /*!< FMC_T::ISPDAT: ISPDAT Mask     */

/* FMC ISPCMD Bit Field Definitions */
#define FMC_ISPCMD_CMD_Pos                      0                                       /*!< FMC_T::ISPCMD: CMD Position    */
#define FMC_ISPCMD_CMD_Msk                      (0x7Ful << FMC_ISPCMD_CMD_Pos)          /*!< FMC_T::ISPCMD: CMD Mask        */

/* FMC ISPTRG Bit Field Definitions */
#define FMC_ISPTRG_ISPGO_Pos                    0                                       /*!< FMC_T::ISPTRG: ISPGO Position  */
#define FMC_ISPTRG_ISPGO_Msk                    (1ul << FMC_ISPTRG_ISPGO_Pos)           /*!< FMC_T::ISPTRG: ISPGO Mask      */

/* FMC DFBADR Bit Field Definitions */
#define FMC_DFBADR_DFBADR_Pos                   0                                       /*!< FMC_T::DFBADR: DFBADR Position */
#define FMC_DFBADR_DFBADR_Msk                   (0xFFFFFFFFul << FMC_DFBADR_DFBADR_Pos) /*!< FMC_T::DFBADR: DFBADR Mask     */

/* FMC FATCON Bit Field Definitions */
#define FMC_FATCON_LFOM_Pos                     4                                       /*!< FMC_T::FATCON: LFOM Position   */
#define FMC_FATCON_LFOM_Msk                     (1ul << FMC_FATCON_LFOM_Pos)            /*!< FMC_T::FATCON: LFOM Mask       */

/* FMC ISPSTA Bit Field Definitions */
#define FMC_ISPSTA_ISPGO_Pos                    0                                       /*!< FMC_T::ISPSTA: ISPGO Position */
#define FMC_ISPSTA_ISPGO_Msk                    (1ul << FMC_ISPSTA_ISPGO_Pos)           /*!< FMC_T::ISPSTA: ISPGO Mask */

#define FMC_ISPSTA_CBS_Pos                      1                                       /*!< FMC_T::ISPSTA: CBS Position */
#define FMC_ISPSTA_CBS_Msk                      (3ul << FMC_ISPSTA_CBS_Pos)             /*!< FMC_T::ISPSTA: CBS Mask */

#define FMC_ISPSTA_ISPFF_Pos                    6                                       /*!< FMC_T::ISPSTA: ISPFF Position */
#define FMC_ISPSTA_ISPFF_Msk                    (1ul << FMC_ISPSTA_ISPFF_Pos)           /*!< FMC_T::ISPSTA: ISPFF Mask */

#define FMC_ISPSTA_VECMAP_Pos                   9                                       /*!< FMC_T::ISPSTA: VECMAP Position */
#define FMC_ISPSTA_VECMAP_Msk                   (0xffful << FMC_ISPSTA_VECMAP_Pos)      /*!< FMC_T::ISPSTA: VECMAP Mask */
/**@}*/ /* FMC_CONST */
/**@}*/ /* FMC */

/*---------------------------- Global Controller -----------------------------*/

/** @addtogroup GCR System Global Controller(SYS)
  Memory Mapped Structure for System Global Controller
  @{
 */

typedef struct
{


/**
 * @var GCR_T::PDID
 * Offset: 0x00  Part Device Identification Number Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |PDID      |This register reflects device part number code. Software can read this register to identify which device is used.
 * @var GCR_T::RSTSRC
 * Offset: 0x04  System Reset Source Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RSTS_POR  |Power-on Reset Flag
 * |        |          |The RSTS_POR flag is set by the "Reset Signal" from the Power-on Reset (POR) controller or bit CHIP_RST (IPRSTC1[0]) to indicate the previous reset source.
 * |        |          |0 = No reset from POR or CHIP_RST (IPRSTC1[0]).
 * |        |          |1 = Power-on Reset (POR) or CHIP_RST (IPRSTC1[0]) had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[1]     |RSTS_RESET|Reset Pin Reset Flag
 * |        |          |The RSTS_RESET flag is set by the "Reset Signal" from the /RESET pin to indicate the previous reset source.
 * |        |          |0 = No reset from /RESET pin.
 * |        |          |1 = The Pin /RESET had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[2]     |RSTS_WDT  |Watchdog Reset Flag
 * |        |          |The RSTS_WDT flag is set by The "Reset Signal" from the Watchdog Timer to indicate the previous reset source.
 * |        |          |0 = No reset from watchdog timer.
 * |        |          |1 = The watchdog timer had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[3]     |RSTS_LVR  |Low Voltage Reset Flag
 * |        |          |The RSTS_LVR flag is set by the "Reset Signal" from the Low-Voltage-Reset Controller to indicate the previous reset source.
 * |        |          |0 = No reset from LVR.
 * |        |          |1 = The LVR controller had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[4]     |RSTS_BOD  |Brown-out Detector Reset Flag
 * |        |          |The RSTS_BOD flag is set by the "Reset Signal" from the Brown-out Detector to indicate the previous reset source.
 * |        |          |0 = No reset from BOD.
 * |        |          |1 = The BOD had issued the reset signal to reset the system.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[5]     |RSTS_MCU  |MCU Reset Flag
 * |        |          |The RSTS_MCU flag is set by the "Reset Signal" from the Cortex-M0 kernel to indicate the previous reset source.
 * |        |          |0 = No reset from Cortex-M0.
 * |        |          |1 = The Cortex-M0 had issued the reset signal to reset the system by writing 1 to bit SYSRESETREQ (AIRCR[2], Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M0 kernel.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[7]     |RSTS_CPU  |CPU Reset Flag
 * |        |          |The RSTS_CPU flag is set by hardware if software writes CPU_RST (IPRSTC1[1]) 1 to reset Cortex-M0 CPU kernel and flash. Memory Controller (FMC)
 * |        |          |0 = No reset from CPU.
 * |        |          |1 = Cortex-M0 CPU kernel and FMC are reset by software setting CPU_RST(IPRSTC1[1]) to 1.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * @var GCR_T::IPRSTC1
 * Offset: 0x08  Peripheral Reset Control Resister 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CHIP_RST  |Chip One-shot Reset (Write Protect)
 * |        |          |Setting this bit will reset the whole chip, including Cortex-M0 core and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
 * |        |          |The CHIP_RST is the same as the POR reset. All the chip controllers are reset and the chip setting from CONFIG0 are also reload.
 * |        |          |0 = Chip normal operation.
 * |        |          |1 = Chip one-shot reset.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[1]     |CPU_RST   |Cortex-M0 Core One-shot Reset (Write Protect)
 * |        |          |Setting this bit will only reset the Cortex-M0 core and Flash Memory Controller (FMC), and this bit will automatically return 0 after the two clock cycles.
 * |        |          |0 = Cortex-M0 core normal operation.
 * |        |          |1 = Cortex-M0 core one-shot reset.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * @var GCR_T::IPRSTC2
 * Offset: 0x0C  Peripheral Reset Control Resister 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |GPIO_RST  |GPIO Controller Reset
 * |        |          |0= GPIO controller normal operation
 * |        |          |1= GPIO controller reset
 * |[2]     |TMR0_RST  |Timer0 Controller Reset
 * |        |          |0= Timer0 controller normal operation
 * |        |          |1= Timer0 controller reset
 * |[3]     |TMR1_RST  |Timer1 Controller Reset
 * |        |          |0= Timer1 controller normal operation
 * |        |          |1= Timer1 controller reset
 * |[4]     |TMR2_RST  |Timer2 Controller Reset
 * |        |          |0= Timer2 controller normal operation
 * |        |          |1= Timer2 controller reset
 * |[5]     |TMR3_RST  |Timer3 Controller Reset
 * |        |          |0= Timer3 controller normal operation
 * |        |          |1= Timer3 controller reset
 * |[8]     |I2C0_RST  |I2C0 Controller Reset
 * |        |          |0= I2C0 controller normal operation
 * |        |          |1= I2C0 controller reset
 * |[9]     |I2C1_RST  |I2C1 Controller Reset
 * |        |          |0= I2C1 controller normal operation
 * |        |          |1= I2C1 controller reset
 * |[12]    |SPI0_RST  |SPI0 Controller Reset
 * |        |          |0= SPI0 controller normal operation
 * |        |          |1= SPI0 controller reset
 * |[16]    |UART0_RST |UART0 Controller Reset
 * |        |          |0= UART0 controller Normal operation
 * |        |          |1= UART0 controller reset
 * |[20]    |PWM03_RST |PWM0~3 Controller Reset
 * |        |          |0= PWM0~3 controller normal operation
 * |        |          |1= PWM0~3 controller reset
 * |[28]    |ADC_RST   |ADC Controller Reset
 * |        |          |0= ADC controller normal operation
 * |        |          |1= ADC controller reset
 * @var GCR_T::BODCR
 * Offset: 0x18  Brown-Out Detector Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BOD_EN    |Brown-out Detector Enable Control (Write Protect)
 * |        |          |The default value is set by flash controller user configuration register CBODEN (CONFIG0[23]) bit.
 * |        |          |0 = Brown-out Detector function Disabled.
 * |        |          |1 = Brown-out Detector function Enabled.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[2:1]   |BOD_VL    |Brown-out Detector Threshold Voltage Select (Write Protect)
 * |        |          |The default value is set by flash memory controller user configuration register CBOV (CONFIG0[22:21]) bit.
 * |        |          |00 = Brown-out voltage is 2.2V.
 * |        |          |01 = Brown-out voltage is 2.7V.
 * |        |          |10 = Brown-out voltage is 3.7V.
 * |        |          |11 = Brown-out voltage is 4.4V.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[3]     |BOD_RSTEN |Brown-out Reset Enable Control (Write Protect)
 * |        |          |0 = Brown-out "INTERRUPT" function Enabled.
 * |        |          |While the BOD function is enabled (BOD_EN high) and BOD interrupt function is enabled (BOD_RSTEN low), BOD will assert an interrupt if BOD_OUT is high.
 * |        |          |BOD interrupt will keep till to the BOD_EN set to 0. BOD interrupt can be blocked by disabling the NVIC BOD interrupt or disabling BOD function (set BOD_EN low).
 * |        |          |1 = Brown-out "RESET" function Enabled.
 * |        |          |Note1: While the Brown-out Detector function is enabled (BOD_EN high) and BOD reset function is enabled (BOD_RSTEN high), BOD will assert a signal to reset chip when the detected voltage is lower than the threshold (BOD_OUT high).
 * |        |          |Note2: The default value is set by flash controller user configuration register CBORST (CONFIG0[20]) bit.
 * |        |          |Note3: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[4]     |BOD_INTF  |Brown-Out Detector Interrupt Flag
 * |        |          |0 = Brown-out Detector does not detect any voltage draft at VDD down through or up through the voltage of BOD_VL setting.
 * |        |          |1 = When Brown-out Detector detects the VDD is dropped down through the voltage of BOD_VL setting or the VDD is raised up through the voltage of BOD_VL setting, this bit is set to 1 and the Brown-out interrupt is requested if Brown-out interrupt is enabled.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[5]     |BOD_LPM   |Brown Out Detector Low power Mode (Write Protect)
 * |        |          |1= Enable the BOD low power mode
 * |        |          |0= BOD operate in normal mode (default)
 * |        |          |The BOD consumes about 100uA in normal mode, the low power mode can reduce the current
 * |        |          |to about 1/10 but slow the BOD response.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[5]     |BOD_LPM   |Brown-Out Detector Low Power Mode (Write Protect)
 * |        |          |0 = BOD operated in Normal mode (default).
 * |        |          |1 = BOD Low Power mode Enabled.
 * |        |          |Note1: The BOD consumes about 100 uA in Normal mode, and the low power mode can reduce the current to about 1/10 but slow the BOD response.
 * |        |          |Note2: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[6]     |BOD_OUT   |Brown-Out Detector Output Status
 * |        |          |0 = Brown-out Detector output status is 0. It means the detected voltage is higher than BOD_VL setting or BOD_EN is 0.
 * |        |          |1 = Brown-out Detector output status is 1. It means the detected voltage is lower than BOD_VL setting. If the BOD_EN is 0, BOD function disabled, this bit always responds to 0.
 * |[7]     |LVR_EN    |Low Voltage Reset Enable Bit (Write Protect)
 * |        |          |The LVR function reset the chip when the input power voltage is lower than LVR circuit setting.
 * |        |          |LVR function is enabled by default.
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
 * |        |          |Note: After this bit is set to 1, the value of temperature sensor output can be obtained from the ADC conversion result.
 * |        |          |Please refer to the ADC chapter for detailed ADC conversion functional description.
 * @var GCR_T::PORCR
 * Offset: 0x24  Power-On-Reset Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field       |Descriptions
 * | :----: | :----:     | :---- |
 * |[15:0]  |POR_DIS_CODE|Power-on-Reset Enable Control (Write Protect)
 * |        |          |When powered on, the POR circuit generates a reset signal to reset the whole chip function, but noise on the power may cause the POR active again.
 * |        |          |User can disable internal POR circuit to avoid unpredictable noise to cause chip reset by writing 0x5AA5 to this field.
 * |        |          |The POR function will be active again when this field is set to another value or chip is reset by other reset source, including:
 * |        |          |/RESET, Watchdog, LVR reset, BOD reset, ICE reset command and the software-chip reset function.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * @var GCR_T::P0_MFP
 * Offset: 0x30  P0 Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |P0_MFP    |P0 Multiple Function Selection
 * |        |          |The pin function of P0 is depending on P0_MFP and P0_ALT.
 * |        |          |Refer to P0_ALT descriptions in detail.
 * |[8]     |P0_ALT0   |P0.0 Alternate Function Selection
 * |        |          |The pin function of P0.0 is depend on P0_MFP[0] and P0_ALT[0].
 * |        |          |P0_ALT[0] P0_MFP[0] = P0.0 Function
 * |        |          |00 = P0.0
 * |        |          |01 = Reserved
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[9]     |P0_ALT1   |P0.1 Alternate Function Selection
 * |        |          |The pin function of P0.1 is depend on P0_MFP[1] and P0_ALT[1].
 * |        |          |P0_ALT[1] P0_MFP[1] = P0.1 Function
 * |        |          |00 = P0.1
 * |        |          |01 = Reserved
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[10]    |P0_ALT2   |P0.2 Alternate Function Selection
 * |        |          |The pin function of P0.2 is depend on P0_MFP[2] and P0_ALT[2].
 * |        |          |P0_ALT[2] P0_MFP[2] = P0.2 Function
 * |        |          |00 = P0.2
 * |        |          |01 = Reserved
 * |        |          |10 = CTS(UART0)
 * |        |          |11 = TXD(UART0)
 * |[11]    |P0_ALT3   |P0.3 Alternate Function Selection
 * |        |          |The pin function of P0.3 is depend on P0_MFP[3] and P0_ALT[3].
 * |        |          |P0_ALT[3] P0_MFP[3] = P0.3 Function
 * |        |          |00 = P0.3
 * |        |          |01 = Reserved
 * |        |          |10 = RTS(UART0)
 * |        |          |11 = RXD(UART0)
 * |[12]    |P0_ALT4   |P0.4 Alternate Function Selection
 * |        |          |The pin function of P0.4 is depend on P0_MFP[4] and P0_ALT[4].
 * |        |          |P0_ALT[4] P0_MFP[4] = P0.4 Function
 * |        |          |00 = P0.4
 * |        |          |01 = Reserved
 * |        |          |10 = SPISS(SPI0)
 * |        |          |11 = Reserved
 * |[13]    |P0_ALT5   |P0.5 Alternate Function Selection
 * |        |          |The pin function of P0.5 is depend on P0_MFP[5] and P0_ALT[5].
 * |        |          |P0_ALT[5] P0_MFP[5] = P0.5 Function
 * |        |          |00 = P0.5
 * |        |          |01 = Reserved
 * |        |          |10 = MOSI(SPI0)
 * |        |          |11 = Reserved
 * |[14]    |P0_ALT6   |P0.6 Alternate Function Selection
 * |        |          |The pin function of P0.6 is depend on P0_MFP[6] and P0_ALT[6].
 * |        |          |P0_ALT[6] P0_MFP[6] = P0.6 Function
 * |        |          |00 = P0.6
 * |        |          |01 = Reserved
 * |        |          |10 = MISO(SPI0)
 * |        |          |11 = Reserved
 * |[15]    |P0_ALT7   |P0.7 Alternate Function Selection
 * |        |          |The pin function of P0.7 is depend on P0_MFP[7] and P0_ALT[7].
 * |        |          |P0_ALT[7] P0_MFP[7] = P0.7 Function
 * |        |          |00 = P0.7
 * |        |          |01 = Reserved
 * |        |          |10 = SPICLK(SPI0)
 * |        |          |11 = Reserved
 * |[23:16] |P0_TYPEn  |P0[7:0] Input Schmitt Trigger function Enable
 * |        |          |0 = P0[7:0] I/O input Schmitt Trigger function Disabled.
 * |        |          |1 = P0[7:0] I/O input Schmitt Trigger function Enabled.
 * @var GCR_T::P1_MFP
 * Offset: 0x34  P1 Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |P1_MFP    |P1 Multiple Function Selection
 * |        |          |The pin function of P1 is depending on P1_MFP and P1_ALT.
 * |        |          |Refer to P1_ALT descriptions in detail.
 * |[8]     |P1_ALT0   |P1.0 alternate function Selection
 * |        |          |The pin function of P1.0 is depend on P1_MFP[0] and P1_ALT[0].
 * |        |          |P1_ALT[0] P1_MFP[0] = P1.0 Function
 * |        |          |00 = P1.0
 * |        |          |01 = AIN0(ADC)
 * |        |          |10 = T2(Timer2)
 * |        |          |11 = Reserved
 * |[9]     |P1_ALT1   |P1.1 Alternate Function Selection
 * |        |          |The pin function of P1.1 is depend on P1_MFP[1] and P1_ALT[1].
 * |        |          |P1_ALT[1] P1_MFP[1] = P1.1 Function
 * |        |          |00 = P1.1
 * |        |          |01 = AIN1(ADC)
 * |        |          |10 = T3(Timer3)
 * |        |          |11 = Reserved
 * |[10]    |P1_ALT2   |P1.2 Alternate Function Selection
 * |        |          |The pin function of P1.2 is depend on P1_MFP[2] and P1_ALT[2].
 * |        |          |P1_ALT[2] P1_MFP[2] = P1.2 Function
 * |        |          |00 = P1.2
 * |        |          |01 = AIN2(ADC)
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[11]    |P1_ALT3   |P1.3 alternate function Selection
 * |        |          |The pin function of P1.3 is depend on P1_MFP[3] and P1_ALT[3].
 * |        |          |P1_ALT[3] P1_MFP[3] = P1.3 Function
 * |        |          |00 = P1.3
 * |        |          |01 = AIN3(ADC)
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[12]    |P1_ALT4   |P1.4 Alternate Function Selection
 * |        |          |The pin function of P1.4 is depend on P1_MFP[4] and P1_ALT[4].
 * |        |          |P1_ALT[4] P1_MFP[4] = P1.4 Function
 * |        |          |00 = P1.4
 * |        |          |01 = AIN4(ADC)
 * |        |          |10 = SPISS(SPI0)
 * |        |          |11 = Reserved
 * |[13]    |P1_ALT5   |P1.5 Alternate Function Selection
 * |        |          |The pin function of P1.5 is depend on P1_MFP[5] and P1_ALT[5].
 * |        |          |P1_ALT[5] P1_MFP[5] = P1.5 Function
 * |        |          |00 = P1.5
 * |        |          |01 = AIN5(ADC)
 * |        |          |10 = MOSI0(SPI0)
 * |        |          |11 = Reserved
 * |[14]    |P1_ALT6   |P1.6 Alternate Function Selection
 * |        |          |The pin function of P1.6 is depend on P1_MFP[6] and P1_ALT[6].
 * |        |          |P1_ALT[6] P1_MFP[6] = P1.6 Function
 * |        |          |00 = P1.6
 * |        |          |01 = AIN6(ADC)
 * |        |          |10 = MISO0(SPI0)
 * |        |          |11 = Reserved
 * |[15]    |P1_ALT7   |P1.7 Alternate Function Selection
 * |        |          |The pin function of P1.7 is depend on P1_MFP[7] and P1_ALT[7].
 * |        |          |P1_ALT[7] P1_MFP[7] = P1.7 Function
 * |        |          |00 = P1.7
 * |        |          |01 = AIN7(ADC)
 * |        |          |10 = SPICLK(SPI0)
 * |        |          |11 = Reserved
 * |[23:16] |P1_TYPEn  |P1[7:0] Input Schmitt Trigger function Enable
 * |        |          |0 = P1[7:0] I/O input Schmitt Trigger function Disabled.
 * |        |          |1 = P1[7:0] I/O input Schmitt Trigger function Enabled.
 * @var GCR_T::P2_MFP
 * Offset: 0x38  P2 Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |P2_MFP    |P2 Multiple Function Selection
 * |        |          |The pin function of P2 is depending on P2_MFP and P2_ALT.
 * |        |          |Refer to P2_ALT descriptions in detail.
 * |[8]     |P2_ALT0   |P2.0 Alternate Function Selection
 * |        |          |The pin function of P2.0 is depend on P2_MFP[0] and P2_ALT[0].
 * |        |          |P2_ALT[0] P2_MFP[0] = P2.0 Function
 * |        |          |00 = P2.0
 * |        |          |01 = Reserved
 * |        |          |10 = PWM0(PWM channel 0)
 * |        |          |11 = Reserved
 * |[9]     |P2_ALT1   |P2.1 Alternate Function Selection
 * |        |          |The pin function of P2.1 is depend on P2_MFP[1] and P2_ALT[1].
 * |        |          |P2_ALT[1] P2_MFP[1] = P2.1 Function
 * |        |          |00 = P2.1
 * |        |          |01 = Reserved
 * |        |          |10 = PWM1(PWM channel 1)
 * |        |          |11 = Reserved
 * |[10]    |P2_ALT2   |P2.2 Alternate Function Selection
 * |        |          |The pin function of P2.2 is depend on P2_MFP[2] and P2_ALT[2].
 * |        |          |P2_ALT[2] P2_MFP[2] = P2.2 Function
 * |        |          |00 = P2.2
 * |        |          |01 = Reserved
 * |        |          |10 = PWM2(PWM channel 2)
 * |        |          |11 = Reserved
 * |[11]    |P2_ALT3   |P2.3 Alternate Function Selection
 * |        |          |The pin function of P2.3 is depend on P2_MFP[3] and P2_ALT[3].
 * |        |          |P2_ALT[3] P2_MFP[3] = P2.3 Function
 * |        |          |00 = P2.3
 * |        |          |01 = Reserved
 * |        |          |10 = PWM3(PWM channel 3)
 * |        |          |11 = Reserved
 * |[12]    |P2_ALT4   |P2.4 Alternate Function Selection
 * |        |          |The pin function of P2.4 is depend on P2_MFP[4] and P2_ALT[4].
 * |        |          |P2_ALT[4] P2_MFP[4] = P0.4 Function
 * |        |          |00 = P2.4
 * |        |          |01 = Reserved
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[13]    |P2_ALT5   |P2.5 Alternate Function Selection
 * |        |          |The pin function of P2.5 is depend on P2_MFP[5] and P2_ALT[5].
 * |        |          |P2_ALT[5] P2_MFP[5] = P2.5 Function
 * |        |          |00 = P2.5
 * |        |          |01 = Reserved
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[14]    |P2_ALT6   |P2.6 Alternate Function Selection
 * |        |          |The pin function of P2.6 is depend on P2_MFP[6] and P2_ALT[6].
 * |        |          |P2_ALT[6] P2_MFP[6] = P2.6 Function
 * |        |          |00 = P2.6
 * |        |          |01 = Reserved
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[15]    |P2_ALT7   |P2.7 Alternate Function Selection
 * |        |          |The pin function of P2.7 is depend on P2_MFP[7] and P2_ALT[7].
 * |        |          |P2_ALT[7] P2_MFP[7] = P2.7 Function
 * |        |          |00 = P2.7
 * |        |          |01 = Reserved
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[23:16] |P2_TYPEn  |P2[7:0] Input Schmitt Trigger function Enable
 * |        |          |0 = P2[7:0] I/O input Schmitt Trigger function Disable.
 * |        |          |1 = P2[7:0] I/O input Schmitt Trigger function Enable.
 * @var GCR_T::P3_MFP
 * Offset: 0x3C  P3 Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |P3_MFP    |P3 Multiple Function Selection
 * |        |          |The pin function of P3 is depending on P3_MFP and P3_ALT.
 * |        |          |Refer to P3_ALT descriptions in detail.
 * |[8]     |P3_ALT0   |P3.0 Alternate Function Selection
 * |        |          |The pin function of P3.0 is depend on P3_MFP[0] and P3_ALT[0].
 * |        |          |P3_ALT[0] P3_MFP[0] = P3.0 Function
 * |        |          |00 = P3.0
 * |        |          |01 = RXD(UART0)
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[9]     |P3_ALT1   |P3.1 Alternate Function Selection
 * |        |          |The pin function of P3.1 is depend on P3_MFP[1] and P3_ALT[1].
 * |        |          |P3_ALT[1] P3_MFP[1] = P3.1 Function
 * |        |          |00 = P3.1
 * |        |          |01 = TXD(UART0)
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[10]    |P3_ALT2   |P3.2 Alternate Function Selection
 * |        |          |The pin function of P3.2 is depend on P3_MFP[2] and P3_ALT[2].
 * |        |          |P3_ALT[2] P3_MFP[2] = P3.2 Function
 * |        |          |00 = P3.2
 * |        |          |01 = nINT0
 * |        |          |10 = T0EX
 * |        |          |11 = Reserved
 * |[11]    |P3_ALT3   |P3.3 Alternate Function Selection
 * |        |          |The pin function of P3.3 is depend on P3_MFP[3] and P3_ALT[3].
 * |        |          |P3_ALT[3] P3_MFP[3] = P3.3 Function
 * |        |          |00 = P3.3
 * |        |          |01 = nINT1
 * |        |          |10 = Reserved
 * |        |          |11 = T1EX
 * |[12]    |P3_ALT4   |P3.4 Alternate Function Selection
 * |        |          |The pin function of P3.4 is depend on P3_MFP[4] and P3_ALT[4].
 * |        |          |P3_ALT[4] P3_MFP[4] = P3.4 Function
 * |        |          |00 = P3.4
 * |        |          |01 = T0(Timer0)
 * |        |          |10 = SDA0(I2C)
 * |        |          |11 = Reserved
 * |[13]    |P3_ALT5   |P3.5 Alternate Function Selection
 * |        |          |The pin function of P3.5 is depend on P3_MFP[5] and P3_ALT[5].
 * |        |          |P3_ALT[5] P3_MFP[5] = P3.5 Function
 * |        |          |00 = P3.5
 * |        |          |01 = T1(Timer1)
 * |        |          |10 = SCL0(I2C0)
 * |        |          |11 = CKO(Clock Driver Output)
 * |[14]    |P3_ALT6   |P3.6 Alternate Function Selection
 * |        |          |The pin function of P3.6 is depend on P3_MFP[6] and P3_ALT[6].
 * |        |          |P3_ALT[6] P3_MFP[6] = P3.6 Function
 * |        |          |00 = P3.6
 * |        |          |01 = Reserved
 * |        |          |10 = CKO(Clock Driver output)
 * |        |          |11 = Reserved
 * |[15]    |P3_ALT7   |P3.7 Alternate Function Selection
 * |        |          |The pin function of P3.7 is depend on P3_MFP[7] and P3_ALT[7].
 * |        |          |P3_ALT[7] P3_MFP[7] = P3.7 Function
 * |        |          |00 = P3.7
 * |        |          |01 = Reserved
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[23:16] |P3_TYPEn  |P3[7:0] Input Schmitt Trigger function Enable
 * |        |          |0 = P3[7:0] I/O input Schmitt Trigger function Disabled.
 * |        |          |1 = P3[7:0] I/O input Schmitt Trigger function Enabled.
 * @var GCR_T::P4_MFP
 * Offset: 0x40  P4 Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |P4_MFP    |P4 Multiple Function Selection
 * |        |          |The pin function of P4 is depending on P4_MFP and P4_ALT.
 * |        |          |Refer to P4_ALT descriptions in detail.
 * |[8]     |P4_ALT0   |P4.0 Alternate Function Selection
 * |        |          |The pin function of P4.0 is depend on P4_MFP[0] and P4_ALT[0].
 * |        |          |P4_ALT[0] P4_MFP[0] = P4.0 Function
 * |        |          |00 = P4.0
 * |        |          |01 = PWM0(PWM channel 0)
 * |        |          |10 = T2EX
 * |        |          |11 = Reserved
 * |[9]     |P4_ALT1   |P4.1 Alternate Function Selection
 * |        |          |The pin function of P4.1 is depend on P4_MFP[1] and P4_ALT[1].
 * |        |          |P4_ALT[1] P4_MFP[1] = P4.1 Function
 * |        |          |00 = P4.1
 * |        |          |01 = PWM1(PWM channel 0)
 * |        |          |10 = T3EX
 * |        |          |11 = Reserved
 * |[10]    |P4_ALT2   |P4.2 Alternate Function Selection
 * |        |          |The pin function of P4.2 is depend on P4_MFP[2] and P4_ALT[2].
 * |        |          |P4_ALT[2] P4_MFP[2] = P4.2 Function
 * |        |          |00 = P4.2
 * |        |          |01 = PWM2(PWM channel 2)
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[11]    |P4_ALT3   |P4.3 Alternate Function Selection
 * |        |          |The pin function of P4.3 is depend on P4_MFP[3] and P4_ALT[3].
 * |        |          |P4_ALT[3] P4_MFP[3] = P4.3 Function
 * |        |          |00 = P4.3
 * |        |          |01 = PWM3(PWM channel 2)
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[12]    |P4_ALT4   |P4.4 Alternate Function Selection
 * |        |          |The pin function of P4.4 is depend on P4_MFP[4] and P4_ALT[4].
 * |        |          |P4_ALT[4] P4_MFP[4] = P4.4 Function
 * |        |          |00 = P4.4
 * |        |          |01 = Reserved
 * |        |          |10 = SCL1(I2C1)
 * |        |          |11 = Reserved
 * |[13]    |P4_ALT5   |P4.5 Alternate Function Selection
 * |        |          |The pin function of P4.5 is depend on P4_MFP[5] and P4_ALT[5].
 * |        |          |P4_ALT[5] P4_MFP[5] = P4.5 Function
 * |        |          |00 = P4.5
 * |        |          |01 = Reserved
 * |        |          |10 = SDA1(I2C1)
 * |        |          |11 = Reserved
 * |[14]    |P4_ALT6   |P4.6 Alternate Function Selection
 * |        |          |The pin function of P4.6 is depend on P4_MFP[6] and P4_ALT[6].
 * |        |          |P4_ALT[6] P4_MFP[6] = P4.6 Function
 * |        |          |00 = P4.6
 * |        |          |01 = ICE_CLK(ICE)
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[15]    |P4_ALT7   |P4.7 Alternate Function Selection
 * |        |          |The pin function of P4.7 is depend on P4_MFP[7] and P4_ALT[7].
 * |        |          |P4_ALT[7] P4_MFP[7] = P4.7 Function
 * |        |          |00 = P4.7
 * |        |          |01 = ICE_DAT(ICE)
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[23:16] |P4_TYPEn  |P4[7:0] Input Schmitt Trigger function Enable
 * |        |          |0 = P4[7:0] I/O input Schmitt Trigger function Disabled.
 * |        |          |1 = P4[7:0] I/O input Schmitt Trigger function Enabled.
 * @var GCR_T::P5_MFP
 * Offset: 0x44  P5 Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |P5_MFP    |P5 Multiple Function Selection
 * |        |          |The pin function of P5 depends on P5_MFP and P5_ALT.
 * |        |          |Refer to P5_ALT for detailed description.
 * |[8]     |P5_ALT0   |P5.0 Alternate Function Selection
 * |        |          |The pin function of P5.0 depends on P5_MFP[0] and P5_ALT[0].
 * |        |          |P5_ALT[0] P5_MFP[0] = P5.0 function
 * |        |          |00 = P5.0
 * |        |          |01 = T0EX
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[9]     |P5_ALT1   |P5.1 Alternate Function Selection
 * |        |          |The pin function of P4.1 depends on P5_MFP[1] and P5_ALT[1].
 * |        |          |P5_ALT[1] P5_MFP[1] = P5.1 function
 * |        |          |00 = P5.1
 * |        |          |01 = T1EX
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[10]     |P5_ALT2  |P5.2 Alternate Function Selection
 * |        |          |The pin function of P5.2 depends on P5_MFP[2] and P5_ALT[2].
 * |        |          |P4_ALT[2] P4_MFP[2] = P4.2 function
 * |        |          |00 = P5.2
 * |        |          |01 = SDA0(I2C0)
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[11]     |P5_ALT3  |P5.3 Alternate Function Selection
 * |        |          |The pin function of P4.3 depends on P5_MFP[3] and P5_ALT[3].
 * |        |          |P5_ALT[3] P5_MFP[3] = P5.3 function
 * |        |          |00 = P5.3
 * |        |          |01 = SCL0(I2C0)
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 * |[23:16] |P5_TYPEn  |P5[7:0] input Schmitt Trigger function Enable
 * |        |          |0 = P5[7:0] I/O input Schmitt Trigger function Disabled.
 * |        |          |1 = P5[7:0] I/O input Schmitt Trigger function Enabled.
 * @var GCR_T::P6_MFP
 * Offset: 0x48  P6 Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:16] |P6_TYPEn  |P6[7:0] Input Schmitt Trigger Function Enable
 * |        |          |0 = P6[7:0] I/O input Schmitt Trigger function Disabled.
 * |        |          |1 = P6[7:0] I/O input Schmitt Trigger function Enabled.
 * @var GCR_T::P7_MFP
 * Offset: 0x4C  P7 Multiple Function and Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |P7_MFP[1:0]|P7 Multiple Function Selection
 * |        |          |The pin function of P7 depends on CONFIG0[27]
 * |        |          |0 = P7.0 and P7.1 pins are configured as GPIO pins.
 * |        |          |1 = P7.0 and P7.1 pins are configured as external 4~24MHz (high speed) crystal pins.
 * |[17:16] |P7_TYPEn  |P7[1:0] Input Schmitt Trigger function Enable Control
 * |        |          |0 = P7.0 and P7.1 input Schmitt Trigger function Disabled.
 * |        |          |1 = P7.0 and P7.1 input Schmitt Trigger function Enabled.
 * @var GCR_T::REGWRPROT
 * Offset: 0x100 Register Write-Protection Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |REGWRPROT |Register Write-Protected Code (Write Only)
 * |        |          |Some write-protected registers have to be disabled the protected function by writing the
 * |        |          |sequence value "59h", "16h", "88h" to this field. After this sequence is completed, the
 * |        |          |REGPROTDIS bit will be set to 1 and write-protected registers can be normal write.
 * |[0]     |REGPROTDIS|Register Write-Protected Disable index (Read only)
 * |        |          |0 = Protection is enabled for writing protected registers. Any write to the protected register is ignored.
 * |        |          |1 = Protection is disabled for writing protected registers.
 * |        |          |The Write-Protected registers list are below table:
 * |        |          |Registers Address Note
 * |        |          |IPRSTC1 0x5000_0008 None
 * |        |          |BODCR 0x5000_0018 None
 * |        |          |PORCR 0x5000_001C None
 * |        |          |PWRCON 0x5000_0200 bit[6] is not protected for power, wake-up interrupt clear
 * |        |          |APBCLK bit[0] 0x5000_0208 bit[0] is watch dog clock enable
 * |        |          |CLKSEL0 0x5000_0210 HCLK and CPU STCLK clock source select
 * |        |          |CLK_SEL1 bit[1:0] 0x5000_0214 Watch dog clock source select
 * |        |          |ISPCON 0x5000_C000 Flash ISP Control register
 * |        |          |ISPCON 0x5000_C010 ISP Trigger Control
 * |        |          |WTCR 0x4000_4000 None
 * |        |          |FATCON 0x5000_C018 None
 */

    __I  uint32_t PDID;          /* Offset: 0x00  Part Device Identification Number Register.                        */
    __IO uint32_t RSTSRC;        /* Offset: 0x04  System Reset Source Register                                       */
    __IO uint32_t IPRSTC1;       /* Offset: 0x08  Peripheral Reset Control Resister 1                                */
    __IO uint32_t IPRSTC2;       /* Offset: 0x0C  Peripheral Reset Control Resister 2                                */
    __I  uint32_t RESERVED0[2];
    __IO uint32_t BODCR;         /* Offset: 0x18  Brown-Out Detector Control Register                                */
    __IO uint32_t TEMPCR;        /* Offset: 0x1C  Temperature Sensor Control Register                                */
    __I  uint32_t RESERVED1;
    __IO uint32_t PORCR;         /* Offset: 0x24  Power-On-Reset Control Register                                    */
    __I  uint32_t RESERVED2[2];
    __IO uint32_t P0_MFP;        /* Offset: 0x30  P0 Multiple Function and Input Type Control Register               */
    __IO uint32_t P1_MFP;        /* Offset: 0x34  P1 Multiple Function and Input Type Control Register               */
    __IO uint32_t P2_MFP;        /* Offset: 0x38  P2 Multiple Function and Input Type Control Register               */
    __IO uint32_t P3_MFP;        /* Offset: 0x3C  P3 Multiple Function and Input Type Control Register               */
    __IO uint32_t P4_MFP;        /* Offset: 0x40  P4 Multiple Function and Input Type Control Register               */
    __IO uint32_t P5_MFP;        /* Offset: 0x44  P5 Multiple Function and Input Type Control Register               */
    __IO uint32_t P6_MFP;        /* Offset: 0x48  P6 Multiple Function and Input Type Control Register               */
    __IO uint32_t P7_MFP;        /* Offset: 0x4C  P7 Multiple Function and Input Type Control Register               */
    __I  uint32_t RESERVED3[44];
    __IO uint32_t REGWRPROT;     /* Offset: 0x100 Register Write-Protection Register                                 */

} GCR_T;




/** @addtogroup GCR_CONST GCR Bit Field Definition
  Constant Definitions for GCR Controller
  @{
 */

/* GCR RSTSRC Bit Field Definitions */
#define SYS_RSTSRC_RSTS_CPU_Pos                 7                                       /*!< GCR_T::RSTSRC: RSTS_CPU Position */
#define SYS_RSTSRC_RSTS_CPU_Msk                 (1ul << SYS_RSTSRC_RSTS_CPU_Pos)        /*!< GCR_T::RSTSRC: RSTS_CPU Mask */

#define SYS_RSTSRC_RSTS_MCU_Pos                 5                                       /*!< GCR_T::RSTSRC: RSTS_MCU Position */
#define SYS_RSTSRC_RSTS_MCU_Msk                 (1ul << SYS_RSTSRC_RSTS_MCU_Pos)        /*!< GCR_T::RSTSRC: RSTS_MCU Mask */

#define SYS_RSTSRC_RSTS_BOD_Pos                 4                                       /*!< GCR_T::RSTSRC: RSTS_BOD Position */
#define SYS_RSTSRC_RSTS_BOD_Msk                 (1ul << SYS_RSTSRC_RSTS_BOD_Pos)        /*!< GCR_T::RSTSRC: RSTS_BOD Mask */

#define SYS_RSTSRC_RSTS_LVR_Pos                 3                                       /*!< GCR_T::RSTSRC: RSTS_LVR Position */
#define SYS_RSTSRC_RSTS_LVR_Msk                 (1ul << SYS_RSTSRC_RSTS_LVR_Pos)        /*!< GCR_T::RSTSRC: RSTS_LVR Mask */

#define SYS_RSTSRC_RSTS_WDT_Pos                 2                                       /*!< GCR_T::RSTSRC: RSTS_WDT Position */
#define SYS_RSTSRC_RSTS_WDT_Msk                 (1ul << SYS_RSTSRC_RSTS_WDT_Pos)        /*!< GCR_T::RSTSRC: RSTS_WDT Mask */

#define SYS_RSTSRC_RSTS_RESET_Pos               1                                       /*!< GCR_T::RSTSRC: RSTS_RESET Position */
#define SYS_RSTSRC_RSTS_RESET_Msk               (1ul << SYS_RSTSRC_RSTS_RESET_Pos)      /*!< GCR_T::RSTSRC: RSTS_RESET Mask */

#define SYS_RSTSRC_RSTS_POR_Pos                 0                                       /*!< GCR_T::RSTSRC: RSTS_POR Position */
#define SYS_RSTSRC_RSTS_POR_Msk                 (1ul << SYS_RSTSRC_RSTS_POR_Pos)        /*!< GCR_T::RSTSRC: RSTS_POR Mask */

/* GCR IPRSTC1 Bit Field Definitions */
#define SYS_IPRSTC1_CPU_RST_Pos                 1                                       /*!< GCR_T::IPRSTC1: CPU_RST Position */
#define SYS_IPRSTC1_CPU_RST_Msk                 (1ul << SYS_IPRSTC1_CPU_RST_Pos)        /*!< GCR_T::IPRSTC1: CPU_RST Mask */

#define SYS_IPRSTC1_CHIP_RST_Pos                0                                       /*!< GCR_T::IPRSTC1: CHIP_RST Position */
#define SYS_IPRSTC1_CHIP_RST_Msk                (1ul << SYS_IPRSTC1_CHIP_RST_Pos)       /*!< GCR_T::IPRSTC1: CHIP_RST Mask */

/* GCR IPRSTC2 Bit Field Definitions */
#define SYS_IPRSTC2_ADC_RST_Pos                 28                                      /*!< GCR_T::IPRSTC2: ADC_RST Position */
#define SYS_IPRSTC2_ADC_RST_Msk                 (1ul << SYS_IPRSTC2_ADC_RST_Pos)        /*!< GCR_T::IPRSTC2: ADC_RST Mask */

#define SYS_IPRSTC2_PWM03_RST_Pos               20                                      /*!< GCR_T::IPRSTC2: PWM03_RST Position */
#define SYS_IPRSTC2_PWM03_RST_Msk               (1ul << SYS_IPRSTC2_PWM03_RST_Pos)      /*!< GCR_T::IPRSTC2: PWM03_RST Mask */

#define SYS_IPRSTC2_UART0_RST_Pos               16                                      /*!< GCR_T::IPRSTC2: UART0_RST Position */
#define SYS_IPRSTC2_UART0_RST_Msk               (1ul << SYS_IPRSTC2_UART0_RST_Pos)      /*!< GCR_T::IPRSTC2: UART0_RST Mask */

#define SYS_IPRSTC2_SPI0_RST_Pos                12                                      /*!< GCR_T::IPRSTC2: SPI0_RST Position */
#define SYS_IPRSTC2_SPI0_RST_Msk                (1ul << SYS_IPRSTC2_SPI0_RST_Pos)       /*!< GCR_T::IPRSTC2: SPI0_RST Mask */

#define SYS_IPRSTC2_I2C1_RST_Pos                9                                       /*!< GCR_T::IPRSTC2: I2C1_RST Position */
#define SYS_IPRSTC2_I2C1_RST_Msk                (1ul << SYS_IPRSTC2_I2C1_RST_Pos)       /*!< GCR_T::IPRSTC2: I2C1_RST Mask */

#define SYS_IPRSTC2_I2C0_RST_Pos                8                                       /*!< GCR_T::IPRSTC2: I2C0_RST Position */
#define SYS_IPRSTC2_I2C0_RST_Msk                (1ul << SYS_IPRSTC2_I2C0_RST_Pos)       /*!< GCR_T::IPRSTC2: I2C0_RST Mask */
#define SYS_IPRSTC2_I2C_RST_Pos                 8                                       /*!< GCR_T::IPRSTC2: I2C_RST Position */
#define SYS_IPRSTC2_I2C_RST_Msk                 (1ul << SYS_IPRSTC2_I2C_RST_Pos)        /*!< GCR_T::IPRSTC2: I2C_RST Mask */

#define SYS_IPRSTC2_TMR3_RST_Pos                5                                       /*!< GCR_T::IPRSTC2: TMR3_RST Position */
#define SYS_IPRSTC2_TMR3_RST_Msk                (1ul << SYS_IPRSTC2_TMR3_RST_Pos)       /*!< GCR_T::IPRSTC2: TMR3_RST Mask */

#define SYS_IPRSTC2_TMR2_RST_Pos                4                                       /*!< GCR_T::IPRSTC2: TMR2_RST Position */
#define SYS_IPRSTC2_TMR2_RST_Msk                (1ul << SYS_IPRSTC2_TMR2_RST_Pos)       /*!< GCR_T::IPRSTC2: TMR2_RST Mask */

#define SYS_IPRSTC2_TMR1_RST_Pos                3                                       /*!< GCR_T::IPRSTC2: TMR1_RST Position */
#define SYS_IPRSTC2_TMR1_RST_Msk                (1ul << SYS_IPRSTC2_TMR1_RST_Pos)       /*!< GCR_T::IPRSTC2: TMR1_RST Mask */

#define SYS_IPRSTC2_TMR0_RST_Pos                2                                       /*!< GCR_T::IPRSTC2: TMR0_RST Position */
#define SYS_IPRSTC2_TMR0_RST_Msk                (1ul << SYS_IPRSTC2_TMR0_RST_Pos)       /*!< GCR_T::IPRSTC2: TMR0_RST Mask */

#define SYS_IPRSTC2_GPIO_RST_Pos                1                                       /*!< GCR_T::IPRSTC2: GPIO_RST Position */
#define SYS_IPRSTC2_GPIO_RST_Msk                (1ul << SYS_IPRSTC2_GPIO_RST_Pos)       /*!< GCR_T::IPRSTC2: GPIO_RST Mask */

/* GCR BODCR Bit Field Definitions */
#define SYS_BODCR_LVR_EN_Pos                    7                                       /*!< GCR_T::BODCR: LVR_EN Position */
#define SYS_BODCR_LVR_EN_Msk                    (1ul << SYS_BODCR_LVR_EN_Pos)           /*!< GCR_T::BODCR: LVR_EN Mask */

#define SYS_BODCR_BOD_OUT_Pos                   6                                       /*!< GCR_T::BODCR: BOD_OUT Position */
#define SYS_BODCR_BOD_OUT_Msk                   (1ul << SYS_BODCR_BOD_OUT_Pos)          /*!< GCR_T::BODCR: BOD_OUT Mask */

#define SYS_BODCR_BOD_LPM_Pos                   5                                       /*!< GCR_T::BODCR: BOD_LPM Position */
#define SYS_BODCR_BOD_LPM_Msk                   (1ul << SYS_BODCR_BOD_LPM_Pos)          /*!< GCR_T::BODCR: BOD_LPM Mask */

#define SYS_BODCR_BOD_INTF_Pos                  4                                       /*!< GCR_T::BODCR: BOD_INTF Position */
#define SYS_BODCR_BOD_INTF_Msk                  (1ul << SYS_BODCR_BOD_INTF_Pos)         /*!< GCR_T::BODCR: BOD_INTF Mask */

#define SYS_BODCR_BOD_RSTEN_Pos                 3                                       /*!< GCR_T::BODCR: BOD_RSTEN Position */
#define SYS_BODCR_BOD_RSTEN_Msk                 (1ul << SYS_BODCR_BOD_RSTEN_Pos)        /*!< GCR_T::BODCR: BOD_RSTEN Mask */

#define SYS_BODCR_BOD_VL_Pos                    1                                       /*!< GCR_T::BODCR: BOD_VL Position */
#define SYS_BODCR_BOD_VL_Msk                    (3ul << SYS_BODCR_BOD_VL_Pos)           /*!< GCR_T::BODCR: BOD_VL Mask */

#define SYS_BODCR_BOD_EN_Pos                    0                                       /*!< GCR_T::BODCR: BOD_EN Position */
#define SYS_BODCR_BOD_EN_Msk                    (1ul << SYS_BODCR_BOD_EN_Pos)           /*!< GCR_T::BODCR: BOD_EN Mask */

/* GCR TEMPCR Bit Field Definitions */
#define SYS_TEMPCR_VTEMP_EN_Pos                 0                                       /*!< GCR_T::TEMPCR: VTEMP_EN Position */
#define SYS_TEMPCR_VTEMP_EN_Msk                 (1ul << SYS_TEMPCR_VTEMP_EN_Pos)        /*!< GCR_T::TEMPCR: VTEMP_EN Mask */

/* GCR PORCR Bit Field Definitions */
#define SYS_PORCR_POR_DIS_CODE_Pos              0                                       /*!< GCR_T::PORCR: POR_DIS_CODE Position */
#define SYS_PORCR_POR_DIS_CODE_Msk              (0xFFFFul << SYS_PORCR_POR_DIS_CODE_Pos)/*!< GCR_T::PORCR: POR_DIS_CODE Mask */

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

#define SYS_P1_MFP_P1_ALT_Pos                   8                                       /*!< GCR_T::P1_MFP: P1_ALT Position */
#define SYS_P1_MFP_P1_ALT_Msk                   (0xFFul << SYS_P1_MFP_P1_ALT_Pos)       /*!< GCR_T::P1_MFP: P1_ALT Mask */

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


/* GCR REGWRPROT Bit Field Definitions */
#define SYS_REGWRPROT_REGWRPROT_Pos             0                                       /*!< GCR_T::REGWRPROT: REGWRPROT Position */
#define SYS_REGWRPROT_REGWRPROT_Msk             (0xFFul << SYS_REGWRPROT_REGWRPROT_Pos) /*!< GCR_T::REGWRPROT: REGWRPROT Mask */

#define SYS_REGWRPROT_REGPROTDIS_Pos            0                                       /*!< GCR_T::REGWRPROT: REGPROTDIS Position */
#define SYS_REGWRPROT_REGPROTDIS_Msk            (1ul << SYS_REGWRPROT_REGPROTDIS_Pos)   /*!< GCR_T::REGWRPROT: REGPROTDIS Mask */
/**@}*/ /* GCR_CONST */



typedef struct
{


/**
 * @var GCR_INT_T::IRQSRC
 * Offset: 0x00-0x7C MCU IRQn(n=0~31) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |IRQSRC[0].0 - BOD INT
 * |        |          |IRQSRC[0].1 - 0
 * |        |          |IRQSRC[0].2 - 0
 * |        |          |IRQSRC[1].0 - WDT INT
 * |        |          |IRQSRC[1].1 - 0
 * |        |          |IRQSRC[1].2 - 0
 * |        |          |IRQSRC[2].0 - EINT0, external interrupt 0 from P3.2
 * |        |          |IRQSRC[2].1 - 0
 * |        |          |IRQSRC[2].2 - 0
 * |        |          |IRQSRC[3].0 - EINT1, external interrupt 1 from P3.3
 * |        |          |IRQSRC[3].1 - 0
 * |        |          |IRQSRC[3].2 - 0
 * |        |          |IRQSRC[4].0 - P0 INT
 * |        |          |IRQSRC[4].1 - P1 INT
 * |        |          |IRQSRC[4].2 - 0
 * |        |          |IRQSRC[5].0 - P2 INT
 * |        |          |IRQSRC[5].1 - P3 INT
 * |        |          |IRQSRC[5].2 - P4 INT
 * |        |          |IRQSRC[6].0 - PWM0 INT
 * |        |          |IRQSRC[6].1 - PWM1 INT
 * |        |          |IRQSRC[6].2 - PWM2 INT
 * |        |          |IRQSRC[6].3 - PWM3 INT
 * |        |          |IRQSRC[8].0 - TIMER0 INT
 * |        |          |IRQSRC[8].1 - 0
 * |        |          |IRQSRC[8].2 - 0
 * |        |          |IRQSRC[9].0 - TIMER1 INT
 * |        |          |IRQSRC[9].1 - 0
 * |        |          |IRQSRC[9].2 - 0
 * |        |          |IRQSRC[10].0 - TIMER2 INT
 * |        |          |IRQSRC[10].1 - 0
 * |        |          |IRQSRC[10].2 - 0
 * |        |          |IRQSRC[11].0 - TIMER3 INT
 * |        |          |IRQSRC[11].1 - 0
 * |        |          |IRQSRC[11].2 - 0
 * |        |          |IRQSRC[12].0 - UART0 INT
 * |        |          |IRQSRC[12].1 - 0
 * |        |          |IRQSRC[12].2 - 0
 * |        |          |IRQSRC[13].0 - 0
 * |        |          |IRQSRC[13].1 - 0
 * |        |          |IRQSRC[13].2 - 0
 * |        |          |IRQSRC[14].0 - SPI0 INT
 * |        |          |IRQSRC[14].1 - 0
 * |        |          |IRQSRC[14].2 - 0
 * |        |          |IRQSRC[15].0 - 0
 * |        |          |IRQSRC[15].1 - 0
 * |        |          |IRQSRC[15].2 - 0
 * |        |          |IRQSRC[16].0 - P5 INT
 * |        |          |IRQSRC[16].1 - 0
 * |        |          |IRQSRC[16].2 - 0
 * |        |          |IRQSRC[17].0 - P6 INT
 * |        |          |IRQSRC[17].1 - P7 INT
 * |        |          |IRQSRC[17].2 - 0
 * |        |          |IRQSRC[18].0 - I2C0 INT
 * |        |          |IRQSRC[18].1 - 0
 * |        |          |IRQSRC[18].2 - 0
 * |        |          |IRQSRC[19].0 - I2C1 INT
 * |        |          |IRQSRC[19].1 - 0
 * |        |          |IRQSRC[19].2 - 0
 * |        |          |IRQSRC[20].0 - CAP0 INT
 * |        |          |IRQSRC[20].1 - 0
 * |        |          |IRQSRC[20].2 - 0
 * |        |          |IRQSRC[21].0 - CAP1 INT
 * |        |          |IRQSRC[21].1 - 0
 * |        |          |IRQSRC[21].2 - 0
 * |        |          |IRQSRC[22].0 - CAP2 INT
 * |        |          |IRQSRC[22].1 - 0
 * |        |          |IRQSRC[22].2 - 0
 * |        |          |IRQSRC[23].0 - CAP3 INT
 * |        |          |IRQSRC[23].1 - 0
 * |        |          |IRQSRC[23].2 - 0
 * |        |          |IRQSRC[24].0 - 0
 * |        |          |IRQSRC[24].1 - 0
 * |        |          |IRQSRC[24].2 - 0
 * |        |          |IRQSRC[25].0 - 0
 * |        |          |IRQSRC[25].1 - 0
 * |        |          |IRQSRC[25].2 - 0
 * |        |          |IRQSRC[26].0 - 0
 * |        |          |IRQSRC[26].1 - 0
 * |        |          |IRQSRC[26].2 - 0
 * |        |          |IRQSRC[27].0 - 0
 * |        |          |IRQSRC[27].1 - 0
 * |        |          |IRQSRC[27].2 - 0
 * |        |          |IRQSRC[28].0 - Power Down Wake up INT
 * |        |          |IRQSRC[28].1 - 0
 * |        |          |IRQSRC[28].2 - 0
 * |        |          |IRQSRC[29].0 - ADC INT
 * |        |          |IRQSRC[29].1 - 0
 * |        |          |IRQSRC[29].2 - 0
 * |        |          |IRQSRC[30].0 - 0
 * |        |          |IRQSRC[30].1 - 0
 * |        |          |IRQSRC[30].2 - 0
 * |        |          |IRQSRC[31].0 - 0
 * |        |          |IRQSRC[31].1 - 0
 * |        |          |IRQSRC[31].2 - 0
 * @var GCR_INT_T::NMISEL
 * Offset: 0x80 NMI Interrupt Source Select Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[4:0]   |NMI_SEL   |NMI interrupt source selection
 * |        |          |The NMI interrupt to Cortex-M0 can be selected from one of the interrupt[31:0].
 * |        |          |The NMI_SEL bit[4:0] used to select the NMI interrupt source.
 * |[8]     |NMI_EN    |NMI interrupt enable (Write Protect)
 * |        |          |0 = NMI interrupt Disabled.
 * |        |          |1 = NMI interrupt Enabled.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * @var GCR_INT_T::MCUIRQ
 * Offset: 0x84 MCU Interrupt Request Source Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]   |MCU_IRQ  |MCU IRQ Source Register
 * |        |          |The MCU_IRQ collects all the interrupts from the peripherals and generates the synchronous interrupt to Cortex-M0 core. 
 * |        |          |There are two modes to generate interrupt to Cortex-M0, the normal mode and test mode.
 * |        |          |The MCU_IRQ collects all interrupts from each peripheral and synchronizes them then interrupts the Cortex-M0.
 * |        |          |When the MCU_IRQ[n] is ¡§0¡¨, set MCU_IRQ[n] ¡§1¡¨ will generate an interrupt to Cortex_M0 NVIC[n].
 * |        |          |When the MCU_IRQ[n] is ¡§1¡¨ (mean an interrupt is assert), set 1 to the MCU_IRQ[n] will clear the interrupt 
 * |        |          |and set MCU_IRQ[n] ¡§0¡¨ : no any effect.
 */

    __I  uint32_t IRQSRC[32];    /* Offset: 0x00-0x7C MCU IRQn(n=0~31) Interrupt Source Identity Register            */
    __IO uint32_t NMISEL;        /* Offset: 0x80 NMI Interrupt Source Select Control Register                        */
    __IO uint32_t MCUIRQ;        /* Offset: 0x84 MCU Interrupt Request Source Register                               */

} GCR_INT_T;



/** @addtogroup INT_CONST INT Bit Field Definition
  Constant Definitions for INT Controller
  @{
 */

/* INT IRQSRC Bit Field Definitions */
#define INT_IRQ_SRC_INT_SRC_Pos                 0                                       /*!< GCR_INT_T::IRQSRC: IRQ_SRC Position */
#define INT_IRQ_SRC_INT_SRC_Msk                 (0xFul << INT_IRQSRC_INTSRC_Pos)        /*!< GCR_INT_T::IRQSRC: IRQ_SRC Mask */

/* INT NMI_SEL Bit Field Definitions */
#define INT_NMI_SEL_NMI_EN_Pos                  8                                       /*!< GCR_INT_T::NMISEL: NMI_EN Position */
#define INT_NMI_SEL_NMI_EN_Msk                  (1ul << INT_NMI_SEL_NMI_EN_Pos)         /*!< GCR_INT_T::NMISEL: NMI_EN Mask */

#define INT_NMI_SEL_NMI_SEL_Pos                 0                                       /*!< GCR_INT_T::NMISEL: NMI_SEL Position */
#define INT_NMI_SEL_NMI_SEL_Msk                 (0x1Ful << INT_NMI_SEL_NMI_SEL_Pos)     /*!< GCR_INT_T::NMISEL: NMI_SEL Mask */
/**@}*/ /* INT_CONST */

/**@}*/ /* GCR */


/*--------------------- General Purpose Input and Output ---------------------*/
/** @addtogroup GPIO General Purpose Input and Output(GPIO)
  Memory Mapped Structure for General Purpose I/O
  @{
 */

typedef struct
{


/**
 * @var GPIO_T::PMD
 * Offset: 0x00  Port 0-7  I/O Mode Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2n+1 :2n]|PMDn    |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of Px pins.
 * |        |          |00 = Px [n] pin is in INPUT mode.
 * |        |          |01 = Px [n] pin is in OUTPUT mode.
 * |        |          |10 = Px [n] pin is in Open-Drain mode.
 * |        |          |11 = Px [n] pin is in Quasi-bidirectional mode.
 * |        |          |Note: x=0~7, n = 0~7.
 * @var GPIO_T::OFFD
 * Offset: 0x04 Port 0-7 Digital Input Path Disable Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:16] |OFFD      |Port 0-7 Pin [n] Digital Input Path Disable Control
 * |        |          |Each of these bits is used to control if the digital input path of corresponding Px.n pin is disabled. If input is analog signal, users can disable Px.n digital input path to avoid input current leakage.
 * |        |          |0 = Px.n digital input path Enabled.
 * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
 * |        |          |Note: x = 0~7, n = 0~7.
 * @var GPIO_T::DOUT
 * Offset: 0x08 Port 0-7 Data Output Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DOUT[n]   |Port 0-7 Pin [n] Output Value
 * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
 * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
 * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
 * |        |          |Note: x = 0~7, n = 0~7.
 * @var GPIO_T::DMASK
 * Offset: 0x0C Port0-7 Data Output Write Mask
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DMASK[n]  |Port 0-7 Pin [n] Data Output Write Mask
 * |        |          |These bits are used to protect the corresponding Px_DOUT[n] bit. When the DMASK[n] bit is set to 1, the corresponding Px_DOUT[n] bit is protected. If the write signal is masked, writing data to the protect bit is ignored.
 * |        |          |0 = Corresponding Px_DOUT[n] bit can be updated.
 * |        |          |1 = Corresponding Px_DOUT[n] bit protected.
 * |        |          |Note1: x = 0~7, n = 0~7.
 * |        |          |Note2: This function only protects the corresponding Px_DOUT[n] bit, and will not protect the corresponding Pxn_PDIO bit.
 * @var GPIO_T::PIN
 * Offset: 0x10 Port 0-7 Pin Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PIN[n]    |Port 0-7 Pin [n] Pin Value
 * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin. If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
 * |        |          |Note: x = 0~7, n = 0~7.
 * @var GPIO_T::DBEN
 * Offset: 0x14 Port 0-7 De-bounce Enable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DBEN[n]   |Port 0-7 Pin [n] Input Signal De-bounce Enable Control
 * |        |          |DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
 * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
 * |        |          |The de-bounce clock source is controlled by DBNCECON[4], one de-bounce sample cycle period is controlled by DBNCECON[3:0].
 * |        |          |0 = Px.n de-bounce function Disabled.
 * |        |          |1 = Px.n de-bounce function Enabled.
 * |        |          |The de-bounce function is valid only for edge triggered interrupt. If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
 * |        |          |Note1: x = 0~7, n = 0~7.
 * |        |          |Note2: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
 * @var GPIO_T::IMD
 * Offset: 0x18 Port 0-7 Interrupt Mode Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |IMD[n]    |Port 0-7 Pin [n] Edge or Level Detection Interrupt Mode Control
 * |        |          |IMD[n] bit is used to control the triggered interrupt is by level trigger or by edge trigger.
 * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
 * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
 * |        |          |0 = Edge trigger interrupt.
 * |        |          |1 = Level trigger interrupt.
 * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers Px_IEN. If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
 * |        |          |The de-bounce function is valid only for edge triggered interrupt. If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
 * |        |          |Note: x = 0~7, n = 0~7.
 * @var GPIO_T::IEN
 * Offset: 0x1C Port 0-7 Interrupt Enable Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:16] |IR_EN[n]  |Port 0-7 Pin [n] Interrupt Enable by Input Rising Edge or Input Level High
 * |        |          |IR_EN[n] bit is used to enable the interrupt for each of the corresponding input Px.n pin.
 * |        |          |Set bit to 1 also enable the pin wake-up function.
 * |        |          |When setting the IR_EN[n] bit to 1 :
 * |        |          |If the interrupt is level trigger (IMD[n] is 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
 * |        |          |If the interrupt is edge trigger (IMD[n] is 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
 * |        |          |0 = Px.n level high or low to high interrupt Disabled.
 * |        |          |1 = Px.n level high or low to high interrupt Enabled.
 * |        |          |Note: x = 0~7, n = 0~7.
 * |[7:0]   |IF_EN[n]  |Port 0-7 Pin [n] Interrupt Enable by Input Falling Edge or Input Level Low
 * |        |          |IF_EN[n] bit is used to enable the interrupt for each of the corresponding input Px.n pin.
 * |        |          |Set bit to 1 also enable the pin wake-up function.
 * |        |          |When setting the IF_EN[n] bit to 1 :
 * |        |          |If the interrupt is level trigger (IMD[n] is 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
 * |        |          |If the interrupt is edge trigger (IMD[n] is 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
 * |        |          |0 = Px.n level low or high to low interrupt Disabled.
 * |        |          |1 = Px.n level low or high to low interrupt Enabled.
 * |        |          |Note: x = 0~7, n = 0~7.
 * @var GPIO_T::ISRC
 * Offset: 0x20 Port 0-7 Interrupt Trigger Source
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ISRC[n]   |Port 0-7 Pin [n] Interrupt Source Flag
 * |        |          |Write :
 * |        |          |0 = No action.
 * |        |          |1 = Clear the corresponding pending interrupt.
 * |        |          |Read :
 * |        |          |0 = No interrupt at Px.n.
 * |        |          |1 = Px.n generates an interrupt.
 * |        |          |Note: x = 0~7, n = 0~7.
 */

    __IO uint32_t PMD;           /* Offset: 0x00  Port 0-7  I/O Mode Control                                         */
    __IO uint32_t OFFD;          /* Offset: 0x04 Port 0-7 Digital Input Path Disable Control                         */
    __IO uint32_t DOUT;          /* Offset: 0x08 Port 0-7 Data Output Value                                          */
    __IO uint32_t DMASK;         /* Offset: 0x0C Port0-7 Data Output Write Mask                                      */
    __IO uint32_t PIN;           /* Offset: 0x10 Port 0-7 Pin Value                                                  */
    __IO uint32_t DBEN;          /* Offset: 0x14 Port 0-7 De-bounce Enable                                           */
    __IO uint32_t IMD;           /* Offset: 0x18 Port 0-7 Interrupt Mode Control                                     */
    __IO uint32_t IEN;           /* Offset: 0x1C Port 0-7 Interrupt Enable Control                                   */
    __IO uint32_t ISRC;          /* Offset: 0x20 Port 0-7 Interrupt Trigger Source                                   */

} GPIO_T;




typedef struct
{


/**
 * @var GPIO_DBNCECON_T::DBNCECON
 * Offset: 0x180 Interrupt De-bounce Cycle Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |DBCLKSEL  |De-bounce Sampling Cycle Selection
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
 * |[4]     |DBCLKSRC  |De-bounce Counter Clock Source Selection
 * |        |          |0 = De-bounce counter clock source is the HCLK.
 * |        |          |1 = De-bounce counter clock source is the 10 kHz internal low speed oscillator.
 * |[5]     |ICLK_ON   |Interrupt Clock On Mode
 * |        |          |0 = Edge detection circuit is active only if I/O pin corresponding Px_IEN bit is set to 1.
 * |        |          |1 = All I/O pins edge detection circuit is always active after reset.
 * |        |          |Note: It is recommended to turn off this bit to save system power if no special application concern.
 */

    __IO uint32_t DBNCECON;      /* Offset: 0x180 Interrupt De-bounce Cycle Control                                  */

} GPIO_DBNCECON_T;




/** @addtogroup GPIO_CONST GPIO Bit Field Definition
  Constant Definitions for GPIO Controller
  @{
 */

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

/* GPIO ISRC Bit Field Definitions */
#define GPIO_ISRC_ISRC_Pos          0                                           /*!< GPIO_T::ISRC: ISRC Position */
#define GPIO_ISRC_ISRC_Msk          (0xFFul << GPIO_ISRC_ISRC_Pos)              /*!< GPIO_T::ISRC: ISRC Mask */

/* GPIO DBNCECON Bit Field Definitions */
#define GPIO_DBNCECON_ICLK_ON_Pos   5                                           /*!< GPIO_DBNCECON_T::DBNCECON: ICLK_ON  Position */
#define GPIO_DBNCECON_ICLK_ON_Msk   (1ul << GPIO_DBNCECON_ICLK_ON_Pos)          /*!< GPIO_DBNCECON_T::DBNCECON: ICLK_ON  Mask */

#define GPIO_DBNCECON_DBCLKSRC_Pos  4                                           /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSRC Position */
#define GPIO_DBNCECON_DBCLKSRC_Msk  (1ul << GPIO_DBNCECON_DBCLKSRC_Pos)         /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSRC Mask */

#define GPIO_DBNCECON_DBCLKSEL_Pos  0                                           /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSEL Position */
#define GPIO_DBNCECON_DBCLKSEL_Msk  (0xFul << GPIO_DBNCECON_DBCLKSEL_Pos)       /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSEL Mask */
/**@}*/ /* GPIO_CONST */
/**@}*/ /* GPIO */


/*------------------------------ I2C Controller ------------------------------*/

/** @addtogroup I2C I2C Controller(I2C)
  Memory Mapped Structure for I2C Serial Interface Controller
  @{
 */

typedef struct
{


/**
 * @var I2C_T::I2CON
 * Offset: 0x00  I2C Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2]     |AA        |Assert Acknowledge Control Bit.
 * |        |          |When AA=1 prior to address or data received, an acknowledged (low level to SDA) will be returned during
 * |        |          |the acknowledge clock pulse on the SCL line when 1.) A slave is acknowledging the address sent from
 * |        |          |master, 2.) The receiver devices are acknowledging the data sent by transmitter. When AA=0 prior to
 * |        |          |address or data received, a Not acknowledged (high level to SDA) will be returned during the acknowledge
 * |        |          |clock pulse on the SCL line.
 * |[3]     |SI        |I2C Interrupt Flag.
 * |        |          |When a new I2C state is present in the I2CSTATUS register, the SI flag is set by hardware, and if bit EI
 * |        |          |(I2CON [7]) is set, the I2C interrupt is requested. SI must be cleared by software.
 * |        |          |Note: Clear SI is by writing one to this bit.
 * |[4]     |STO       |I2C STOP Control Bit.
 * |        |          |In master mode, setting STO to transmit a STOP condition to bus then I2C hardware will check the bus
 * |        |          |condition if a STOP condition is detected bit will be cleared by hardware automatically.
 * |        |          |In a slave mode, setting STO resets I2C hardware to the defined "not addressed" slave mode. This means
 * |        |          |it is NO LONGER in the slave receiver mode to receive data from the master transmit device.
 * |[5]     |STA       |I2C START Control Bit.
 * |        |          |Setting STA to logic 1 to enter master mode, the I2C hardware sends a START or repeat START condition
 * |        |          |to bus when the bus is free.
 * |[6]     |ENS1      |I2C Controller Enable Bit
 * |        |          |Set to enable I2C serial function block. When ENS1=1 the I2C serial function enables. The multi-function
 * |        |          |pin function of SDA and SCL must set to I2C function first.
 * |        |          |0 = Disable
 * |        |          |1 = Enable
 * |[7]     |EI        |Enable Interrupt.
 * |        |          |0 = Disable I2C interrupt.
 * |        |          |1 = Enable I2C interrupt.
 * @var I2C_T::I2CADDR0
 * Offset: 0x04  I2C Slave Address Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |I2CADDR   |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode. In the slave mode, the seven
 * |        |          |most significant bits must be loaded with the MCU's own address. The I2C hardware will react if
 * |        |          |either of the address is matched.
 * @var I2C_T::I2CDAT
 * Offset: 0x08  I2C Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |I2CDAT    |I2C Data Register
 * |        |          |Bit[7:0] is located with the 8-bit transferred data of I2C serial port.
 * @var I2C_T::I2CSTATUS
 * Offset: 0x0C  I2C Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |I2CSTATUS |I2C Status Register
 * |        |          |The status register of I2C:
 * |        |          |The three least significant bits are always 0. The five most significant bits contain the status code.
 * |        |          |There are 26 possible status codes. When I2STATUS contains F8H, no serial interrupt is
 * |        |          |requested. All other I2STATUS values correspond to defined I2C states. When each of these
 * |        |          |states is entered, a status interrupt is requested (SI = 1). A valid status code is present in
 * |        |          |I2STATUS one machine cycle after SI is set by hardware and is still present one machine cycle
 * |        |          |after SI has been reset by software. In addition, states 00H stands for a Bus Error. A Bus Error
 * |        |          |occurs when a START or STOP condition is present at an illegal position in the formation frame.
 * |        |          |Example of illegal position are during the serial transfer of an address byte, a data byte or an
 * |        |          |acknowledge bit.
 * @var I2C_T::I2CLK
 * Offset: 0x10  I2C Clock Divided Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |I2CLK     |I2C Clock Divided Register
 * |        |          |The I2C clock rate bits: Data Baud Rate of I2C = PCLK /(4x(I2CLK+1)).
 * |        |          |Note: The minimum value of I2CLK is 4.
 * @var I2C_T::I2CTOC
 * Offset: 0x14  I2C Time-Out Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TIF       |Time-Out Flag.
 * |        |          |This bit is set by H/W when I2C time-out happened and it can interrupt CPU if I2C interrupt enable(EI) is set to 1.
 * |        |          |Note: Write 1 to clear this bit.
 * |[1]     |DIV4      |Time-out Counter Input Clock is Divided by 4
 * |        |          |When Enable, The time-out period is extend 4 times.
 * |        |          |0 = Disable
 * |        |          |1 = Enable
 * |[2]     |ENTI      |Time-out Counter is Enabled/Disable
 * |        |          |When Enable, the 14 bit time-out counter will start counting when SI is clear. Setting flag SI to high will
 * |        |          |reset counter and re-start up counting after SI is cleared.
 * |        |          |0 = Disable
 * |        |          |1 = Enable
 * @var I2C_T::I2CADDR1
 * Offset: 0x18  I2C Slave Address Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |I2CADDR   |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode. In the slave mode, the seven
 * |        |          |most significant bits must be loaded with the MCU's own address. The I2C hardware will react if
 * |        |          |either of the address is matched.
 * @var I2C_T::I2CADDR2
 * Offset: 0x1C  I2C Slave Address Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |I2CADDR   |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode. In the slave mode, the seven
 * |        |          |most significant bits must be loaded with the MCU's own address. The I2C hardware will react if
 * |        |          |either of the address is matched.
 * @var I2C_T::I2CADDR3
 * Offset: 0x20  I2C Slave Address Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |I2CADDR   |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode. In the slave mode, the seven
 * |        |          |most significant bits must be loaded with the MCU's own address. The I2C hardware will react if
 * |        |          |either of the address is matched.
 * @var I2C_T::I2CADM0
 * Offset: 0x24  I2C Slave Address Mask Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |I2ADMx    |I2C Address Mask Register
 * |        |          |0 = Mask disable (the received corresponding register bit should be exact the same as
 * |        |          |address register.)
 * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
 * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
 * |        |          |When the bit in the address mask register is set to one, it means the received
 * |        |          |corresponding address bit is don't-care. If the bit is set to zero, that means the received
 * |        |          |corresponding register bit should be exact the same as address register.
 * @var I2C_T::I2CADM1
 * Offset: 0x28  I2C Slave Address Mask Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |I2ADMx    |I2C Address Mask register
 * |        |          |0 = Mask disable (the received corresponding register bit should be exact the same as
 * |        |          |address register.)
 * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
 * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
 * |        |          |When the bit in the address mask register is set to one, it means the received
 * |        |          |corresponding address bit is don't-care. If the bit is set to zero, that means the received
 * |        |          |corresponding register bit should be exact the same as address register.
 * @var I2C_T::I2CADM2
 * Offset: 0x2C  I2C Slave Address Mask Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |I2ADMx    |I2C Address Mask register
 * |        |          |0 = Mask disable (the received corresponding register bit should be exact the same as
 * |        |          |address register.)
 * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
 * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
 * |        |          |When the bit in the address mask register is set to one, it means the received
 * |        |          |corresponding address bit is don't-care. If the bit is set to zero, that means the received
 * |        |          |corresponding register bit should be exact the same as address register.
 * @var I2C_T::I2CADM3
 * Offset: 0x30  I2C Slave Address Mask Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |I2ADM     |I2C Address Mask register
 * |        |          |0 = Mask disable (the received corresponding register bit should be exact the same as
 * |        |          |address register.)
 * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
 * |        |          |I2C bus controllers support multiple address recognition with four address mask register.
 * |        |          |When the bit in the address mask register is set to one, it means the received
 * |        |          |corresponding address bit is don't-care. If the bit is set to zero, that means the received
 * |        |          |corresponding register bit should be exact the same as address register.
 * @var I2C_T::I2CWKUPCON
 * Offset: 0x3C  I2C Wake-up Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WKUPEN    |I2C Wake-Up Function Enable
 * |        |          |0 = Disable I2C wake-up function.
 * |        |          |1 = Enable I2C wake-up function.
 * @var I2C_T::I2CWKUPSTS
 * Offset: 0x40  I2C Wake-up Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WKUPIF    |I2C Wake-up Interrupt Flag
 * |        |          |When chip is woken up from Power-down mode by I2C, this bit is set to 1.
 * |        |          |Note: Software can write 1 to clear this bit.
 */

    __IO uint32_t I2CON;         /* Offset: 0x00  I2C Control Register                                               */
    __IO uint32_t I2CADDR0;      /* Offset: 0x04  I2C Slave Address Register 0                                       */
    __IO uint32_t I2CDAT;        /* Offset: 0x08  I2C Data Register                                                  */
    __I  uint32_t I2CSTATUS;     /* Offset: 0x0C  I2C Status Register                                                */
    __IO uint32_t I2CLK;         /* Offset: 0x10  I2C Clock Divided Register                                         */
    __IO uint32_t I2CTOC;        /* Offset: 0x14  I2C Time-Out Control Register                                      */
    __IO uint32_t I2CADDR1;      /* Offset: 0x18  I2C Slave Address Register 1                                       */
    __IO uint32_t I2CADDR2;      /* Offset: 0x1C  I2C Slave Address Register 2                                       */
    __IO uint32_t I2CADDR3;      /* Offset: 0x20  I2C Slave Address Register 3                                       */
    __IO uint32_t I2CADM0;       /* Offset: 0x24  I2C Slave Address Mask Register 0                                  */
    __IO uint32_t I2CADM1;       /* Offset: 0x28  I2C Slave Address Mask Register 1                                  */
    __IO uint32_t I2CADM2;       /* Offset: 0x2C  I2C Slave Address Mask Register 2                                  */
    __IO uint32_t I2CADM3;       /* Offset: 0x30  I2C Slave Address Mask Register 3                                  */
    __I  uint32_t RESERVED0[2];
    __IO uint32_t I2CWKUPCON;    /* Offset: 0x3C  I2C Wake-up Control Register                                       */
    __IO uint32_t I2CWKUPSTS;    /* Offset: 0x40  I2C Wake-up Status Register                                        */

} I2C_T;



/** @addtogroup I2C_CONST I2C Bit Field Definition
  Constant Definitions for I2C Controller
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

/**@}*/ /* I2C_CONST */
/**@}*/ /* I2C */



/*----------------------------- PWM Controller -------------------------------*/

/** @addtogroup PWM PWM Controller(PWM)
  Memory Mapped Structure for PWM Generator and Capture Timer
  @{
 */

typedef struct
{


/**
 * @var PWM_T::PPR
 * Offset: 0x00  PWM Prescaler Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |CP01      |Clock prescalar 0
 * |        |          |Clock input is divided by (CP01 + 1) before it is fed to the corresponding PWM-Timer
 * |        |          |If CP01=0, then the clock prescalar 0 output clock will be stopped. So corresponding
 * |        |          |PWM-Timer will also be stopped.
 * |[15:8]  |CP23      |Clock prescalar 2
 * |        |          |Clock input is divided by (CP23 + 1) before it is fed to the corresponding PWM-Timer
 * |        |          |If CP23=0, then the clock prescalar 2 output clock will be stopped. So corresponding
 * |        |          |PWM-Timer will also be stopped.
 * |[23:16] |DZI01     |Dead zone interval register for pair of channel 0 and channel 1
 * |        |          |These 8 bits determine dead zone length.
 * |        |          |The unit time of Dead-zone length = [(prescale+1)*(clock source divider)]/ PWM01_CLK.
 * |[31:24] |DZI23     |Dead zone interval register for pair of channel 2 and channel 3
 * |        |          |These 8 bits determine dead zone length.
 * |        |          |The unit time of Dead-zone length = [(prescale+1)*(clock source divider)]/ PWM23_CLK.
 * @var PWM_T::CSR
 * Offset: 0x04  PWM Clock Source Divider Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |CSR0      |Timer 0 Clock Source Selection
 * |        |          |Select clock input for timer.
 * |        |          |(Table is the same as CSR3)
 * |[6:4]   |CSR1      |Timer 1 Clock Source Selection
 * |        |          |Select clock input for timer.
 * |        |          |(Table is the same as CSR3)
 * |[10:8]  |CSR2      |Timer 2 Clock Source Selection
 * |        |          |Select clock input for timer.
 * |        |          |(Table is the same as CSR3)
 * |[14:12] |CSR3      |Timer 3 Clock Source Selection
 * |        |          |Select clock input for timer.
 * |        |          |CSRx[14:12] = Input clock divider
 * |        |          |000 = 2
 * |        |          |001 = 4
 * |        |          |010 = 8
 * |        |          |011 = 16
 * |        |          |100 = 1
 * @var PWM_T::PCR
 * Offset: 0x08  PWM Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CH0EN     |PWM-Timer 0 Enable Control
 * |        |          |0 = The corresponding PWM-Timer stops running.
 * |        |          |1 = The corresponding PWM-Timer starts running.
 * |[1]     |CH0PINV   |PWM-Timer 0 Output Polar Inverse Enable Control
 * |        |          |0 = Polar Inverter Disabled.
 * |        |          |1 = Polar Inverter Enabled.
 * |[2]     |CH0INV    |PWM-Timer 0 Output Inverter Enable Control
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled.
 * |[3]     |CH0MOD    |PWM-Timer 0 Auto-reload/One-Shot Mode Control
 * |        |          |0 = One-shot Mode.
 * |        |          |1 = Auto-reload Mode.
 * |        |          |Note: If there is a transition at this bit, it will cause CNR0 and CMR0 be cleared.
 * |[4]     |DZEN01    |Dead-zone 0 Generator Enable Control
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |        |          |Note: When Dead-Zone Generator is enabled, the pair of PWM0 and PWM1 becomes a
 * |        |          |complementary pair for PWM group A.
 * |[5]     |DZEN23    |Dead-zone 2 Generator Enable
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |        |          |Note: When Dead-Zone Generator is enabled, the pair of PWM2 and PWM3 becomes a
 * |        |          |complementary pair for PWM group A.
 * |[8]     |CH1EN     |PWM-Timer 1 Enable
 * |        |          |0 = Corresponding PWM-Timer Stopped.
 * |        |          |1 = Corresponding PWM-Timer Start Running.
 * |[9]     |CH1PINV   |PWM-Timer 1 Output Polar Inverse Enable
 * |        |          |0 = Polar Inverter Disabled.
 * |        |          |1 = Polar Inverter Enabled.
 * |[10]    |CH1INV    |PWM-Timer 1 Output Inverter Enable
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled.
 * |[11]    |CH1MOD    |PWM-Timer 1 Auto-reload/One-Shot Mode
 * |        |          |0 = One-shot Mode.
 * |        |          |1 = Auto-reload Mode.
 * |        |          |Note: If there is a transition at this bit, it will cause CNR1 and CMR1 be cleared.
 * |[16]    |CH2EN     |PWM-Timer 2 Enable
 * |        |          |0 = Corresponding PWM-Timer Stopped.
 * |        |          |1 = Corresponding PWM-Timer Start Running.
 * |[17]    |CH2PINV   |PWM-Timer 2 Output Polar Inverse Enable
 * |        |          |0 = Polar Inverter Disabled.
 * |        |          |1 = Polar Inverter Enabled.
 * |[18]    |CH2INV    |PWM-Timer 2 Output Inverter Enable
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled.
 * |[19]    |CH2MOD    |PWM-Timer 2 Auto-reload/One-Shot Mode Control
 * |        |          |0 = One-shot Mode.
 * |        |          |1 = Auto-reload Mode.
 * |        |          |Note: If there is a transition at this bit, it will cause CNR2 and CMR2 be cleared.
 * |[24]    |CH3EN     |PWM-Timer 3 Enable
 * |        |          |0 = Corresponding PWM-Timer Stopped.
 * |        |          |1 = Corresponding PWM-Timer Start Running.
 * |[25]    |CH3PINV   |PWM-Timer 3 Output Polar Inverse Enable
 * |        |          |0 = Polar Inverter Disabled.
 * |        |          |1 = Polar Inverter Enabled.
 * |[26]    |CH3INV    |PWM-Timer 3 Output Inverter Enable
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled.
 * |[27]    |CH3MOD    |PWM-Timer 3 Auto-reload/One-Shot Mode Control
 * |        |          |0 = One-shot Mode.
 * |        |          |1 = Auto-reload Mode.
 * |        |          |Note: If there is a transition at this bit, it will cause CNR3 and CMR3 be cleared.
 * |[30]    |PWM01TYPE |PWM01 Aligned Type Selection Bit
 * |        |          |0 = Edge-aligned type.
 * |        |          |1 = Center-aligned type.
 * |[31]    |PWM23TYPE |PWM23 Aligned Type Selection Bit
 * |        |          |0 = Edge-aligned type.
 * |        |          |1 = Center-aligned type.
 * @var PWM_T::CNR0
 * Offset: 0x0C  PWM Counter Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNR       |PWM Counter/Timer Loaded Value
 * |        |          |CNR determines the PWM period.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
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
 * |        |          |Note1: Any write to CNR will take effect in next PWM cycle.
 * |        |          |Note2: When CNR value is set to 0, PWM output is always high.
 * |        |          |Note3: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to 0xFFFE.
 * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
 * @var PWM_T::CMR0
 * Offset: 0x10  PWM Comparator Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMR       |PWM Comparator Register
 * |        |          |CMR determines the PWM duty.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
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
 * @var PWM_T::PDR0
 * Offset: 0x14  PWM Data Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PDR       |PWM Data Register
 * |        |          |User can monitor PDR to know the current value in 16-bit down counter.
 * @var PWM_T::CNR1
 * Offset: 0x18  PWM Counter Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNR       |PWM Counter/Timer Loaded Value
 * |        |          |CNR determines the PWM period.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
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
 * |        |          |Note1: Any write to CNR will take effect in next PWM cycle.
 * |        |          |Note2: When CNR value is set to 0, PWM output is always high.
 * |        |          |Note3: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to 0xFFFE.
 * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
 * @var PWM_T::CMR1
 * Offset: 0x1C  PWM Comparator Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMR       |PWM Comparator Register
 * |        |          |CMR determines the PWM duty.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
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
 * @var PWM_T::PDR1
 * Offset: 0x20  PWM Data Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PDR       |PWM Data Register
 * |        |          |User can monitor PDR to know the current value in 16-bit down counter.
 * @var PWM_T::CNR2
 * Offset: 0x24  PWM Counter Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNR       |PWM Counter/Timer Loaded Value
 * |        |          |CNR determines the PWM period.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
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
 * |        |          |Note1: Any write to CNR will take effect in next PWM cycle.
 * |        |          |Note2: When CNR value is set to 0, PWM output is always high.
 * |        |          |Note3: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to 0xFFFE.
 * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
 * @var PWM_T::CMR2
 * Offset: 0x28  PWM Comparator Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMR       |PWM Comparator Register
 * |        |          |CMR determines the PWM duty.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
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
 * @var PWM_T::PDR2
 * Offset: 0x2C  PWM Data Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PDR       |PWM Data Register
 * |        |          |User can monitor PDR to know the current value in 16-bit down counter.
 * @var PWM_T::CNR3
 * Offset: 0x30  PWM Counter Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNR       |PWM Counter/Timer Loaded Value
 * |        |          |CNR determines the PWM period.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
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
 * |        |          |Note1: Any write to CNR will take effect in next PWM cycle.
 * |        |          |Note2: When CNR value is set to 0, PWM output is always high.
 * |        |          |Note3: When PWM operating at Center-aligned type, CNR value should be set between 0x0000 to 0xFFFE.
 * |        |          |If CNR equal to 0xFFFF, the PWM will work unpredictable.
 * @var PWM_T::CMR3
 * Offset: 0x34  PWM Comparator Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMR       |PWM Comparator Register
 * |        |          |CMR determines the PWM duty.
 * |        |          |PWM frequency = PWMxy_CLK/[(prescale+1)*(clock divider)*(CNR+1)]; where xy, could be 01 or 23, depends on selected PWM channel.
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
 * @var PWM_T::PDR3
 * Offset: 0x38  PWM Data Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PDR       |PWM Data Register
 * |        |          |User can monitor PDR to know the current value in 16-bit down counter.
 * @var PWM_T::PIER
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
 * |[8]     |PWMDIE0   |PWM Channel 0 Duty Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[9]     |PWMDIE1   |PWM Channel 1 Duty Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[10]    |PWMDIE2   |PWM Channel 2 Duty Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[11]    |PWMDIE3   |PWM Channel 3 Duty Interrupt Enable
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |[16]    |INT01TYPE |PWM01 Interrupt Period Type Selection
 * |        |          |0 = PWMIFx will be set if PWM counter underflow.
 * |        |          |PWM will trigger ADC to conversion when PWM counter underflow if correlate PWM trigger enable bit (PWMxTEN) is set to 1.
 * |        |          |1 = PWMIFx will be set if PWM counter matches CNRx register.
 * |        |          |PWM will trigger ADC to conversion when PWM counter matches CNRx register if correlate PWM trigger enable bit (PWMxTEN) is set to 1.
 * |        |          |Note: Setting INT01TYPE to 1 only works when PWM operating is in center-aligned type.
 * |[17]    |INT23TYPE |PWM23 Interrupt Period Type Selection
 * |        |          |0 = PWMIFx will be set if PWM counter underflow.
 * |        |          |PWM will trigger ADC to conversion when PWM counter underflow if correlate PWM trigger enable bit (PWMxTEN) is set to 1.
 * |        |          |1 = PWMIFx will be set if PWM counter matches CNRx register.
 * |        |          |PWM will trigger ADC to conversion when PWM counter matches CNRx register if correlate PWM trigger enable bit (PWMxTEN) is set to 1.
 * |        |          |Note: Setting INT23TYPE to 1 only works when PWM operating is in center-aligned type.
 * |[24]    |INT01DTYPE|PWM01 Duty Interrupt Type Selection
 * |        |          |0 = PWMDIFx will be set if PWM counter down count and matches CMRx register.
 * |        |          |PWM will trigger ADC to conversion when PWM counter down count and matches CMRx register if correlate PWM trigger enable bit (PWMxDTEN) is set to 1.
 * |        |          |1 = PWMDIFx will be set when PWM counter up count and matches CMRx register.
 * |        |          |PWM will trigger ADC to conversion when PWM counter up count and matches CMRx register if correlate PWM trigger enable bit (PWMxDTEN) is set to 1.
 * |        |          |Note: Set INT01DTYPE to 1 only work when PWM operating in center aligned type.
 * |[25]    |INT23DTYPE|PWM23 Duty Interrupt Type Selection
 * |        |          |0 = PWMDIFx will be set if PWM counter down count and matches CMRx register.
 * |        |          |PWM will trigger ADC to conversion when PWM counter down count and matches CMRx register if correlate PWM trigger enable bit (PWMxDTEN) is set to 1.
 * |        |          |1 = PWMDIFx will be set when PWM counter up count and matches CMRx register.
 * |        |          |PWM will trigger ADC to conversion when PWM counter up count and matches CMRx register if correlate PWM trigger enable bit (PWMxDTEN) is set to 1.
 * |        |          |Note: Set INT23DTYPE to 1 only work when PWM operating in center aligned type.
 * @var PWM_T::PIIR
 * Offset: 0x44  PWM Interrupt Indication Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PWMIF0    |PWM channel 0 Period Interrupt Status
 * |        |          |When INTTYPE01 = 0, this bit is set by hardware when channel 0 PWM counter equals 0.
 * |        |          |When INTTYPE01 = 1, this bit is set by hardware when channel 0 PWM counter equals CNR0.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[1]     |PWMIF1    |PWM channel 1 Period Interrupt Status
 * |        |          |When INTTYPE01 = 0, this bit is set by hardware when channel 1 PWM counter equals 0.
 * |        |          |When INTTYPE01 = 1, this bit is set by hardware when channel 1 PWM counter equals CNR1.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[2]     |PWMIF2    |PWM channel 2 Period Interrupt Status
 * |        |          |When INTTYPE23 = 0, this bit is set by hardware when channel 2 PWM counter equals 0.
 * |        |          |When INTTYPE23 = 1, this bit is set by hardware when channel 2 PWM counter equals CNR2.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[3]     |PWMIF3    |PWM Channel 3 Period Interrupt Status
 * |        |          |When INTTYPE23 = 0, this bit is set by hardware when channel 3 PWM counter equals 0.
 * |        |          |When INTTYPE23 = 1, this bit is set by hardware when channel 3 PWM counter equals CNR3.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[8]     |PWMDIF0   |PWM Channel 0 Duty Interrupt Status
 * |        |          |When INTDTYPE01 = 0, this bit is set by hardware when channel 0 PWM counter equals CMR0 on down-count.
 * |        |          |When INTDTYPE01 = 1, this bit is set by hardware when channel 0 PWM counter equals CMR0 on up-count.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[9]     |PWMDIF1   |PWM Channel 1 Duty Interrupt Status
 * |        |          |When INTDTYPE01 = 0, this bit is set by hardware when channel 1 PWM counter equals CMR1 on down-count.
 * |        |          |When INTDTYPE01 = 1, this bit is set by hardware when channel 1 PWM counter equals CMR1 on up-count.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[10]    |PWMDIF2   |PWM Channel 2 Duty Interrupt Status
 * |        |          |When INTDTYPE23 = 0, this bit is set by hardware when channel 2 PWM counter equals CMR2 on down-count.
 * |        |          |When INTDTYPE23 = 1, this bit is set by hardware when channel 2 PWM counter equals CMR2 on up-count.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[11]    |PWMDIF3   |PWM Channel 3 Duty Interrupt Status
 * |        |          |When INTDTYPE23 = 0, this bit is set by hardware when channel 3 PWM counter equals CMR3 on down-count.
 * |        |          |When INTDTYPE23 = 1, this bit is set by hardware when channel 3 PWM counter equals CMR3 on up-count.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * @var PWM_T::CCR0
 * Offset: 0x50  PWM Capture Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |INV0      |Capture Channel 0 Inverter Enable Control
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer.
 * |[1]     |CRL_IE0   |Channel 0 Rising Latch Interrupt Enable Control
 * |        |          |0 = Rising latch interrupt Disabled.
 * |        |          |1 = Rising latch interrupt Enabled.
 * |        |          |Note: When Enabled, if Capture detects PWM group channel 0 has rising transition, Capture will issue an Interrupt.
 * |[2]     |CFL_IE0   |Channel 0 Falling Latch Interrupt Enable Control
 * |        |          |0 = Falling latch interrupt Disabled.
 * |        |          |1 = Falling latch interrupt Enabled.
 * |        |          |Note: When Enabled, if Capture detects PWM group channel 0 has falling transition, and Capture will issue an Interrupt.
 * |[3]     |CAPCH0EN  |Channel 0 Capture Function Enable Control
 * |        |          |0 = Capture function on PWM group channel 0 Disabled.
 * |        |          |1 = Capture function on PWM group channel 0 Enabled.
 * |        |          |Note1: When Enabled, Capture latches the PWM-counter value and saved to CRLR (Rising latch) and CFLR (Falling latch).
 * |        |          |Note2: When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 0 Interrupt.
 * |[4]     |CAPIF0    |Channel 0 Capture Interrupt Indication Flag
 * |        |          |If PWM group channel 0 rising latch interrupt is enabled (CRL_IE0 = 1), a rising transition occurs at PWM group channel 0 will result in CAPIF0 to high;
 * |        |          |Similarly, a falling transition will cause CAPIF0 to be set high if PWM group channel 0 falling latch interrupt is enabled (CFL_IE0 = 1).
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[6]     |CRLRI0    |CRLR0 Latched Indicator
 * |        |          |When PWM group input channel 0 has a rising transition, CRLR0 was latched with the
 * |        |          |value of PWM down-counter and this bit is set by hardware.
 * |        |          |Note:  Write 1 to clear this bit to 0.
 * |[7]     |CFLRI0    |CFLR0 Latched Indicator
 * |        |          |When PWM group input channel 0 has a falling transition, CFLR0 was latched with the
 * |        |          |value of PWM down-counter and this bit is set by hardware.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[16]    |INV1      |Capture Channel 1 Inverter Enable Control
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer.
 * |[17]    |CRL_IE1   |Channel 1 Rising Latch Interrupt Enable Control
 * |        |          |0 = Rising latch interrupt Disabled.
 * |        |          |1 = Rising latch interrupt Enabled.
 * |        |          |Note: When Enabled, if Capture detects PWM group channel 1 has rising transition, Capture will issue an Interrupt.
 * |[18]    |CFL_IE1   |Channel 1 Falling Latch Interrupt Enable Control
 * |        |          |0 = Falling latch interrupt Disabled.
 * |        |          |1 = Falling latch interrupt Enabled.
 * |        |          |Note: When Enabled, if Capture detects PWM group channel 1 has falling transition, Capture will issue an Interrupt.
 * |[19]    |CAPCH1EN  |Channel 1 Capture Function Enable Control
 * |        |          |0 = Capture function on PWM group channel 1 Disabled.
 * |        |          |1 = Capture function on PWM group channel 1 Enabled.
 * |        |          |Note1: When Enabled, Capture latched the PWM-counter and saved to CRLR (Rising latch) and CFLR (Falling latch).
 * |        |          |Note2: When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 1 Interrupt.
 * |[20]    |CAPIF1    |Channel 1 Capture Interrupt Indication Flag
 * |        |          |If PWM group channel 1 rising latch interrupt is enabled (CRL_IE1 = 1), a rising transition occurs at PWM group channel 1 will result in CAPIF1 to high;
 * |        |          |Similarly, a falling transition will cause CAPIF1 to be set high if PWM group channel 1 falling latch interrupt is enabled (CFL_IE1 = 1).
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[22]    |CRLRI1    |CRLR1 Latched Indicator
 * |        |          |When PWM group input channel 1 has a rising transition, CRLR1 was latched with the
 * |        |          |value of PWM down-counter and this bit is set by hardware.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[23]    |CFLRI1    |CFLR1 Latched Indicator
 * |        |          |When PWM group input channel 1 has a falling transition, CFLR1 was latched with the
 * |        |          |value of PWM down-counter and this bit is set by hardware.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * @var PWM_T::CCR2
 * Offset: 0x54  PWM Capture Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |INV2      |Capture Channel 2 Inverter Enable Control
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer.
 * |[1]     |CRL_IE2   |Channel 2 Rising Latch Interrupt Enable Control
 * |        |          |0 = Rising latch interrupt Disabled.
 * |        |          |1 = Rising latch interrupt Enabled.
 * |        |          |Note: When Enabled, if Capture detects PWM group channel 2 has rising transition, Capture will issue an Interrupt.
 * |[2]     |CFL_IE2   |Channel 2 Falling Latch Interrupt Enable Control
 * |        |          |0 = Falling latch interrupt Disabled.
 * |        |          |1 = Falling latch interrupt Enabled.
 * |        |          |Note: When Enabled, if Capture detects PWM group channel 2 has falling transition, Capture will issue an Interrupt.
 * |[3]     |CAPCH2EN  |Channel 2 Capture Function Enable Control
 * |        |          |0 = Capture function on PWM group channel 2 Disabled.
 * |        |          |1 = Capture function on PWM group channel 2 Enabled.
 * |        |          |Note1: When Enabled, Capture latched the PWM-counter value and saved to CRLR (Rising latch) and CFLR (Falling latch).
 * |        |          |Note2: When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 2 Interrupt.
 * |[4]     |CAPIF2    |Channel 2 Capture Interrupt Indication Flag
 * |        |          |If PWM group channel 2 rising latch interrupt is enabled (CRL_IE2=1), a rising transition occurs at PWM group channel 2 will result in CAPIF2 to high;
 * |        |          |Similarly, a falling transition will cause CAPIF2 to be set high if PWM group channel 2 falling latch interrupt is enabled (CFL_IE2=1).
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[6]     |CRLRI2    |CRLR2 Latched Indicator
 * |        |          |When PWM group input channel 2 has a rising transition, CRLR2 was latched with the
 * |        |          |value of PWM down-counter and this bit is set by hardware.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[7]     |CFLRI2    |CFLR2 Latched Indicator
 * |        |          |When PWM group input channel 2 has a falling transition, CFLR2 was latched with the
 * |        |          |value of PWM down-counter and this bit is set by hardware.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[16]    |INV3      |Capture Channel 3 Inverter Enable Control
 * |        |          |0 = Inverter Disabled.
 * |        |          |1 = Inverter Enabled. Reverse the input signal from GPIO before fed to Capture timer.
 * |[17]    |CRL_IE3   |Channel 3 Rising Latch Interrupt Enable Control
 * |        |          |0 = Rising latch interrupt Disabled.
 * |        |          |1 = Rising latch interrupt Enabled.
 * |        |          |Note: When Enabled, if Capture detects PWM group channel 3 has rising transition, Capture will issue an Interrupt.
 * |[18]    |CFL_IE3   |Channel 3 Falling Latch Interrupt Enable Control
 * |        |          |0 = Falling latch interrupt Disabled.
 * |        |          |1 = Falling latch interrupt Enabled.
 * |        |          |Note: When Enabled, if Capture detects PWM group channel 3 has falling transition, Capture will issue an Interrupt.
 * |[19]    |CAPCH3EN  |Channel 3 Capture Function Enable Control
 * |        |          |0 = Capture function on PWM group channel 3 Disabled.
 * |        |          |1 = Capture function on PWM group channel 3 Enabled.
 * |        |          |Note1: When Enabled, Capture latched the PWM-counter and saved to CRLR (Rising latch) and CFLR (Falling latch).
 * |        |          |Note2: When Disabled, Capture does not update CRLR and CFLR, and disable PWM group channel 3 Interrupt.
 * |[20]    |CAPIF3    |Channel 3 Capture Interrupt Indication Flag
 * |        |          |If PWM group channel 3 rising latch interrupt is enabled (CRL_IE3=1), a rising transition occurs at PWM group channel 3 will result in CAPIF3 to high;
 * |        |          |Similarly, a falling transition will cause CAPIF3 to be set high if PWM group channel 3 falling latch interrupt is enabled (CFL_IE3=1).
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[22]    |CRLRI3    |CRLR3 Latched Indicator
 * |        |          |When PWM group input channel 3 has a rising transition, CRLR3 was latched with the
 * |        |          |value of PWM down-counter and this bit is set by hardware.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * |[23]    |CFLRI3    |CFLR3 Latched Indicator
 * |        |          |When PWM group input channel 3 has a falling transition, CFLR3 was latched with the
 * |        |          |value of PWM down-counter and this bit is set by hardware.
 * |        |          |Note: Write 1 to clear this bit to 0.
 * @var PWM_T::CRLR0
 * Offset: 0x58  PWM Capture Rising Latch Register (Channel 0)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CRLR      |Capture Rising Latch Register
 * |        |          |Latch the PWM counter value when Channel 0 has rising transition.
 * @var PWM_T::CFLR0
 * Offset: 0x5C  PWM Capture Falling Latch Register (Channel 0)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CFLR      |Capture Falling Latch Register
 * |        |          |Latch the PWM counter value when Channel 0 has Falling transition.
 * @var PWM_T::CRLR1
 * Offset: 0x60  PWM Capture Rising Latch Register (Channel 1)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CRLR      |Capture Rising Latch Register
 * |        |          |Latch the PWM counter value when Channel 1 has rising transition.
 * @var PWM_T::CFLR1
 * Offset: 0x64  PWM Capture Falling Latch Register (Channel 1)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CFLR      |Capture Falling Latch Register
 * |        |          |Latch the PWM counter value when Channel 1 has Falling transition.
 * @var PWM_T::CRLR2
 * Offset: 0x68  PWM Capture Rising Latch Register (Channel 2)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CRLR      |Capture Rising Latch Register
 * |        |          |Latch the PWM counter value when Channel 2 has rising transition.
 * @var PWM_T::CFLR2
 * Offset: 0x6C  PWM Capture Falling Latch Register (Channel 2)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CFLR      |Capture Falling Latch Register
 * |        |          |Latch the PWM counter value when Channel 2 has Falling transition.
 * @var PWM_T::CRLR3
 * Offset: 0x70  PWM Capture Rising Latch Register (Channel 3)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CRLR      |Capture Rising Latch Register
 * |        |          |Latch the PWM counter value when Channel 3 has rising transition.
 * @var PWM_T::CFLR3
 * Offset: 0x74  PWM Capture Falling Latch Register (Channel 3)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CFLR      |Capture Falling Latch Register
 * |        |          |Latch the PWM counter value when Channel 3 has Falling transition.
 * @var PWM_T::CAPENR
 * Offset: 0x78  PWM Capture Input 0~3 Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |CAPENR    |Capture Input Enable Register
 * |        |          |0 = Capture input Disabled. (PWMx multi-function pin input does not affect input capture function.)
 * |        |          |1 = Capture input Enabled. (PWMx multi-function pin input will affect its input capture function.)
 * |        |          |CAPENR
 * |        |          |Bit 3210 for PWM group A
 * |        |          |Bit xxx1 -> Capture channel 0 enabled. Capture input channel can be from P2.0 or
 * |        |          |P4.0. User can only select one of pins by setting multi-function pin register.
 * |        |          |Bit xx1x -> Capture channel 1 enabled. Capture input channel can be from P2.1 or
 * |        |          |P4.1. User can only select one of pins by setting multi-function pin register.
 * |        |          |Bit x1xx -> Capture channel 2 enabled. Capture input channel can be from P2.2 or
 * |        |          |P4.2. User can only select one of pins by setting multi-function pin register.
 * |        |          |Bit 1xxx -> Capture channel 3 enabled. Capture input channel can be from P2.3 or
 * |        |          |P4.3. User can only select one of pins by setting multi-function pin register.
 * @var PWM_T::POE
 * Offset: 0x7C  PWM Output Enable Register for Channel 0~3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PWM0      |PWM Channel 0 Output Enable Register
 * |        |          |0 = PWM channel 0 output to pin Disabled.
 * |        |          |1 = PWM channel 0 output to pin Enabled.
 * |        |          |Note: The corresponding GPIO pin must also be switched to PWM function.
 * |[1]     |PWM1      |PWM Channel 1 Output Enable Register
 * |        |          |0 = PWM channel 1 output to pin Disabled.
 * |        |          |1 = PWM channel 1 output to pin Enabled.
 * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function.
 * |[2]     |PWM2      |PWM Channel 2 Output Enable Register
 * |        |          |0 = PWM channel 2 output to pin Disabled.
 * |        |          |1 = PWM channel 2 output to pin Enabled.
 * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function.
 * |[3]     |PWM3      |PWM Channel 3 Output Enable Register
 * |        |          |0 = PWM channel 3 output to pin Disabled.
 * |        |          |1 = PWM channel 3 output to pin Enabled.
 * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function.
 * @var PWM_T::TCON
 * Offset: 0x80  PWM Trigger Control Register for Channel 0~3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PWM0PTEN  |Channel 0 PWM Period Trigger ADC Enable Control
 * |        |          |0 = PWM channel 0 period trigger ADC function Disabled.
 * |        |          |1 = PWM channel 0 period trigger ADC function Enabled.
 * |        |          |As PWM operating at edge-aligned type, enable this bit can make PWM trigger ADC to start conversion when PWM channel 0 counter down count to underflow.
 * |        |          |As PWM operating at center-aligned type, enable this bit can make PWM trigger ADC to start conversion
 * |        |          |when PWM channel 0 counter up count to (CNR0 + 1) or down count to underflow based on INT01PTYPE setting.
 * |[1]     |PWM1PTEN  |Channel 1 PWM Period Trigger ADC Enable Control
 * |        |          |0 = PWM channel 1 period trigger ADC function Disabled.
 * |        |          |1 = PWM channel 1 period trigger ADC function Enabled.
 * |        |          |As PWM operating at edge-aligned type, enable this bit can make PWM trigger ADC to start conversion when PWM channel 1 counter down count to underflow.
 * |        |          |As PWM operating at center-aligned type, enable this bit can make PWM trigger ADC to start conversion
 * |        |          |when PWM channel 1 counter up count to (CNR1 + 1) or down count to underflow based on INT01PTYPE setting.
 * |[2]     |PWM2PTEN  |Channel 2 PWM Period Trigger ADC Enable Control
 * |        |          |0 = PWM channel 2 period trigger ADC function Disabled.
 * |        |          |1 = PWM channel 2 period trigger ADC function Enabled.
 * |        |          |As PWM operating at edge-aligned type, enable this bit can make PWM trigger ADC to start conversion when PWM channel 2 counter down count to underflow.
 * |        |          |As PWM operating at center-aligned type, enable this bit can make PWM trigger ADC to start conversion
 * |        |          |when PWM channel 2 counter up count to (CNR2 + 1) or down count to underflow based on INT23PTYPE setting.
 * |[3]     |PWM3PTEN  |Channel 3 PWM Period Trigger ADC Enable Control
 * |        |          |0 = PWM channel 3 period trigger ADC function Disabled.
 * |        |          |1 = PWM channel 3 period trigger ADC function Enabled.
 * |        |          |As PWM operating at edge-aligned type, enable this bit can make PWM trigger ADC to start conversion when PWM channel 3 counter down count to underflow.
 * |        |          |As PWM operating at center-aligned type, enable this bit can make PWM trigger ADC to start conversion
 * |        |          |when PWM channel 3 counter up count to (CNR3 + 1) or down count to underflow based on INT23PTYPE setting.
 * |[8]     |PWM0DTEN  |Channel 0 PWM Duty Trigger ADC Enable Control
 * |        |          |0 = PWM channel 0 duty trigger ADC function Disabled.
 * |        |          |1 = PWM channel 0 duty trigger ADC function Enabled.
 * |        |          |As PWM operating at edge-aligned type, enable this bit can make PWM trigger ADC to start conversion when PWM channel 0 counter down count to match CMR0.
 * |        |          |As PWM operating at center-aligned type, enable this bit can make PWM trigger ADC to start conversion
 * |        |          |when PWM channel 0 counter up count or down count to match CMR0 based on INT01DTYPE setting.
 * |[9]     |PWM1DTEN  |Channel 1 PWM Duty Trigger ADC Enable Control
 * |        |          |0 = PWM channel 1 duty trigger ADC function Disabled.
 * |        |          |1 = PWM channel 1 duty trigger ADC function Enabled.
 * |        |          |As PWM operating at edge-aligned type, enable this bit can make PWM trigger ADC to start conversion when PWM channel 1 counter down count to match CMR1.
 * |        |          |As PWM operating at center-aligned type, enable this bit can make PWM trigger ADC to start conversion
 * |        |          |when PWM channel 1 counter up count or down count to match CMR1 based on INT01DTYPE setting.
 * |[10]    |PWM2DTEN  |Channel 2 PWM Duty Trigger ADC Enable Control
 * |        |          |0 = PWM channel 2 duty trigger ADC function Disabled.
 * |        |          |1 = PWM channel 2 duty trigger ADC function Enabled.
 * |        |          |As PWM operating at edge-aligned type, enable this bit can make PWM trigger ADC to start conversion when PWM channel 2 counter down count to match CMR2.
 * |        |          |As PWM operating at center-aligned type, enable this bit can make PWM trigger ADC to start conversion
 * |        |          |when PWM channel 2 counter up count or down count to match CMR2 based on INT23DTYPE setting.
 * |[11]    |PWM3DTEN  |Channel 3 PWM Duty Trigger ADC Enable Control
 * |        |          |0 = PWM channel 3 duty trigger ADC function Disabled.
 * |        |          |1 = PWM channel 3 duty trigger ADC function Enabled.
 * |        |          |As PWM operating at edge-aligned type, enable this bit can make PWM trigger ADC to start conversion when PWM channel 3 counter down count to match CMR3.
 * |        |          |As PWM operating at center-aligned type, enable this bit can make PWM trigger ADC to start conversion
 * |        |          |when PWM channel 3 counter up count or down count to match CMR3 based on INT23DTYPE setting.
 * @var PWM_T::TSTATUS
 * Offset: 0x84  PWM Trigger Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PWM0TF    |Channel 0 PWM Trigger ADC Flag
 * |        |          |This bit is set to 1 by hardware when PWM channel 0 trigger ADC condition
 * |        |          |matched. ADC will start conversion if ADC triggered source is selected by PWM
 * |        |          |while this bit is set to 1.
 * |[1]     |PWM1TF    |PWM Channel 1 Trigger ADC Flag
 * |        |          |This bit is set to 1 by hardware when PWM channel 1 trigger ADC condition
 * |        |          |matched. ADC will start conversion if ADC triggered source is selected by PWM
 * |        |          |while this bit is set to 1.
 * |[2]     |PWM2TF    |PWM Channel 2 Trigger ADC Flag
 * |        |          |This bit is set to 1 by hardware when PWM channel 2 trigger ADC condition
 * |        |          |matched. ADC will start conversion if ADC triggered source is selected by PWM
 * |        |          |while this bit is set to 1.
 * |[3]     |PWM3TF    |PWM Channel 3 Trigger ADC Flag
 * |        |          |This bit is set to 1 by hardware when PWM channel 3 trigger ADC condition
 * |        |          |matched. ADC will start conversion if ADC triggered source is selected by PWM
 * |        |          |while this bit is set to 1.
 * @var PWM_T::CAPIC
 * Offset: 0xA0  PWM Capture Interrupt Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CIENS     |PWM Capture interrupt exception Number select
 * |        |          |This bit is to enable PWM0-3 capture interrupts have independent interrupt exception number.
 * |        |          |1 = Enable PWM0-3 independent capture interrupt exception number
 * |        |          |    PWM0-3 capture interrupt exception number is 36-39.
 * |        |          |0 = Disable PWM0-3 independent capture interrupt exception number
 * |        |          |    PWM0-3 capture interrupts are merged into interrupt exception number 22.
 */

    __IO uint32_t PPR;           /* Offset: 0x00  PWM Prescaler Register                                             */
    __IO uint32_t CSR;           /* Offset: 0x04  PWM Clock Source Divider Select Register                           */
    __IO uint32_t PCR;           /* Offset: 0x08  PWM Control Register                                               */
    __IO uint32_t CNR0;          /* Offset: 0x0C  PWM Counter Register 0                                             */
    __IO uint32_t CMR0;          /* Offset: 0x10  PWM Comparator Register 0                                          */
    __I  uint32_t PDR0;          /* Offset: 0x14  PWM Data Register 0                                                */
    __IO uint32_t CNR1;          /* Offset: 0x18  PWM Counter Register 1                                             */
    __IO uint32_t CMR1;          /* Offset: 0x1C  PWM Comparator Register 1                                          */
    __I  uint32_t PDR1;          /* Offset: 0x20  PWM Data Register 1                                                */
    __IO uint32_t CNR2;          /* Offset: 0x24  PWM Counter Register 2                                             */
    __IO uint32_t CMR2;          /* Offset: 0x28  PWM Comparator Register 2                                          */
    __I  uint32_t PDR2;          /* Offset: 0x2C  PWM Data Register 2                                                */
    __IO uint32_t CNR3;          /* Offset: 0x30  PWM Counter Register 3                                             */
    __IO uint32_t CMR3;          /* Offset: 0x34  PWM Comparator Register 3                                          */
    __I  uint32_t PDR3;          /* Offset: 0x38  PWM Data Register 3                                                */
    __I  uint32_t RESERVE0;     
    __IO uint32_t PIER;          /* Offset: 0x40  PWM Interrupt Enable Register                                      */
    __IO uint32_t PIIR;          /* Offset: 0x44  PWM Interrupt Indication Register                                  */
    __I  uint32_t RESERVE1[2];  
    __IO uint32_t CCR0;          /* Offset: 0x50  PWM Capture Control Register 0                                     */
    __IO uint32_t CCR2;          /* Offset: 0x54  PWM Capture Control Register 2                                     */
    __IO uint32_t CRLR0;         /* Offset: 0x58  PWM Capture Rising Latch Register (Channel 0)                      */
    __IO uint32_t CFLR0;         /* Offset: 0x5C  PWM Capture Falling Latch Register (Channel 0)                     */
    __IO uint32_t CRLR1;         /* Offset: 0x60  PWM Capture Rising Latch Register (Channel 1)                      */
    __IO uint32_t CFLR1;         /* Offset: 0x64  PWM Capture Falling Latch Register (Channel 1)                     */
    __IO uint32_t CRLR2;         /* Offset: 0x68  PWM Capture Rising Latch Register (Channel 2)                      */
    __IO uint32_t CFLR2;         /* Offset: 0x6C  PWM Capture Falling Latch Register (Channel 2)                     */
    __IO uint32_t CRLR3;         /* Offset: 0x70  PWM Capture Rising Latch Register (Channel 3)                      */
    __IO uint32_t CFLR3;         /* Offset: 0x74  PWM Capture Falling Latch Register (Channel 3)                     */
    __IO uint32_t CAPENR;        /* Offset: 0x78  PWM Capture Input 0~3 Enable Register                              */
    __IO uint32_t POE;           /* Offset: 0x7C  PWM Output Enable Register for Channel 0~3                         */
    __IO uint32_t TCON;          /* Offset: 0x80  PWM Trigger Control Register for Channel 0~3                       */
    __IO uint32_t TSTATUS;       /* Offset: 0x84  PWM Trigger Status Register                                        */
    __I  uint32_t RESERVED2[6];
    __IO uint32_t CAPIC;         /* Offset: 0xA0  PWM Capture Interrupt Control Register                             */

} PWM_T;



/** @addtogroup PWM_CONST PWM Bit Field Definition
  Constant Definitions for PWM Controller
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
#define PWM_PCR_PWM23TYPE_Pos                   31                                  /*!< PWM_T::PCR: PWM23TYPE Position */
#define PWM_PCR_PWM23TYPE_Msk                   (1ul << PWM_PCR_PWM23TYPE_Pos)      /*!< PWM_T::PCR: PWM23TYPE Mask */

#define PWM_PCR_PWM01TYPE_Pos                   30                                  /*!< PWM_T::PCR: PWM01TYPE Position */
#define PWM_PCR_PWM01TYPE_Msk                   (1ul << PWM_PCR_PWM01TYPE_Pos)      /*!< PWM_T::PCR: PWM01TYPE Mask */

#define PWM_PCR_CH3MOD_Pos                      27                                  /*!< PWM_T::PCR: CH3MOD Position */
#define PWM_PCR_CH3MOD_Msk                      (1ul << PWM_PCR_CH3MOD_Pos)         /*!< PWM_T::PCR: CH3MOD Mask */

#define PWM_PCR_CH3INV_Pos                      26                                  /*!< PWM_T::PCR: CH3INV Position */
#define PWM_PCR_CH3INV_Msk                      (1ul << PWM_PCR_CH3INV_Pos)         /*!< PWM_T::PCR: CH3INV Mask */

#define PWM_PCR_CH3PINV_Pos                     25                                  /*!< PWM_T::PCR: CH3PINV Position */
#define PWM_PCR_CH3PINV_Msk                     (1ul << PWM_PCR_CH3PINV_Pos)        /*!< PWM_T::PCR: CH3PINV Mask */

#define PWM_PCR_CH3EN_Pos                       24                                  /*!< PWM_T::PCR: CH3EN Position */
#define PWM_PCR_CH3EN_Msk                       (1ul << PWM_PCR_CH3EN_Pos)          /*!< PWM_T::PCR: CH3EN Mask */

#define PWM_PCR_CH2MOD_Pos                      19                                  /*!< PWM_T::PCR: CH2MOD Position */
#define PWM_PCR_CH2MOD_Msk                      (1ul << PWM_PCR_CH2MOD_Pos)         /*!< PWM_T::PCR: CH2MOD Mask */

#define PWM_PCR_CH2INV_Pos                      18                                  /*!< PWM_T::PCR: CH2INV Position */
#define PWM_PCR_CH2INV_Msk                      (1ul << PWM_PCR_CH2INV_Pos)         /*!< PWM_T::PCR: CH2INV Mask */

#define PWM_PCR_CH2PINV_Pos                     17                                  /*!< PWM_T::PCR: CH2PINV Position */
#define PWM_PCR_CH2PINV_Msk                     (1ul << PWM_PCR_CH2PINV_Pos)        /*!< PWM_T::PCR: CH2PINV Mask */

#define PWM_PCR_CH2EN_Pos                       16                                  /*!< PWM_T::PCR: CH2EN Position */
#define PWM_PCR_CH2EN_Msk                       (1ul << PWM_PCR_CH2EN_Pos)          /*!< PWM_T::PCR: CH2EN Mask */

#define PWM_PCR_CH1MOD_Pos                      11                                  /*!< PWM_T::PCR: CH1MOD Position */
#define PWM_PCR_CH1MOD_Msk                      (1ul << PWM_PCR_CH1MOD_Pos)         /*!< PWM_T::PCR: CH1MOD Mask */

#define PWM_PCR_CH1INV_Pos                      10                                  /*!< PWM_T::PCR: CH1INV Position */
#define PWM_PCR_CH1INV_Msk                      (1ul << PWM_PCR_CH1INV_Pos)         /*!< PWM_T::PCR: CH1INV Mask */

#define PWM_PCR_CH1PINV_Pos                     9                                   /*!< PWM_T::PCR: CH1PINV Position */
#define PWM_PCR_CH1PINV_Msk                     (1ul << PWM_PCR_CH1PINV_Pos)        /*!< PWM_T::PCR: CH1PINV Mask */

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

#define PWM_PCR_CH0PINV_Pos                      1                                  /*!< PWM_T::PCR: CH0PINV Position */
#define PWM_PCR_CH0PINV_Msk                     (1ul << PWM_PCR_CH0PINV_Pos)        /*!< PWM_T::PCR: CH0PINV Mask */

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
#define PWM_PIER_INT23DTYPE_Pos                 25                                  /*!< PWM_T::PIER: INT23DTYPE Position */
#define PWM_PIER_INT23DTYPE_Msk                 (1ul << PWM_PIER_INT23DTYPE_Pos)    /*!< PWM_T::PIER: INT23DTYPE Mask */

#define PWM_PIER_INT01DTYPE_Pos                 24                                  /*!< PWM_T::PIER: INT01DTYPE Position */
#define PWM_PIER_INT01DTYPE_Msk                 (1ul << PWM_PIER_INT01DTYPE_Pos)    /*!< PWM_T::PIER: INT01DTYPE Mask */

#define PWM_PIER_INT23TYPE_Pos                  17                                  /*!< PWM_T::PIER: INT23TYPE Position */
#define PWM_PIER_INT23TYPE_Msk                  (1ul << PWM_PIER_INT23TYPE_Pos)     /*!< PWM_T::PIER: INT23TYPE Mask */

#define PWM_PIER_INT01TYPE_Pos                  16                                  /*!< PWM_T::PIER: INT01TYPE Position */
#define PWM_PIER_INT01TYPE_Msk                  (1ul << PWM_PIER_INT01TYPE_Pos)     /*!< PWM_T::PIER: INT01TYPE Mask */

#define PWM_PIER_PWMDIE3_Pos                    11                                  /*!< PWM_T::PIER: PWMDIE3 Position */
#define PWM_PIER_PWMDIE3_Msk                    (1ul << PWM_PIER_PWMDIE3_Pos)       /*!< PWM_T::PIER: PWMDIE3 Mask */

#define PWM_PIER_PWMDIE2_Pos                    10                                  /*!< PWM_T::PIER: PWMDIE2 Position */
#define PWM_PIER_PWMDIE2_Msk                    (1ul << PWM_PIER_PWMDIE2_Pos)       /*!< PWM_T::PIER: PWMDIE2 Mask */

#define PWM_PIER_PWMDIE1_Pos                    9                                   /*!< PWM_T::PIER: PWMDIE1 Position */
#define PWM_PIER_PWMDIE1_Msk                     (1ul << PWM_PIER_PWMDIE1_Pos)      /*!< PWM_T::PIER: PWMDIE1 Mask */

#define PWM_PIER_PWMDIE0_Pos                    8                                   /*!< PWM_T::PIER: PWMDIE0 Position */
#define PWM_PIER_PWMDIE0_Msk                    (1ul << PWM_PIER_PWMDIE0_Pos)       /*!< PWM_T::PIER: PWMDIE0 Mask */

#define PWM_PIER_PWMIE3_Pos                     3                                   /*!< PWM_T::PIER: PWMIE3 Position */
#define PWM_PIER_PWMIE3_Msk                     (1ul << PWM_PIER_PWMIE3_Pos)        /*!< PWM_T::PIER: PWMIE3 Mask */

#define PWM_PIER_PWMIE2_Pos                     2                                   /*!< PWM_T::PIER: PWMIE2 Position */
#define PWM_PIER_PWMIE2_Msk                     (1ul << PWM_PIER_PWMIE2_Pos)        /*!< PWM_T::PIER: PWMIE2 Mask */

#define PWM_PIER_PWMIE1_Pos                     1                                   /*!< PWM_T::PIER: PWMIE1 Position */
#define PWM_PIER_PWMIE1_Msk                     (1ul << PWM_PIER_PWMIE1_Pos)        /*!< PWM_T::PIER: PWMIE1 Mask */

#define PWM_PIER_PWMIE0_Pos                     0                                   /*!< PWM_T::PIER: PWMIE0 Position */
#define PWM_PIER_PWMIE0_Msk                     (1ul << PWM_PIER_PWMIE0_Pos)        /*!< PWM_T::PIER: PWMIE0 Mask */

/* PWM PIIR Bit Field Definitions */
#define PWM_PIIR_PWMDIF3_Pos                    11                                  /*!< PWM_T::PIIR: PWMDIF3 Position */
#define PWM_PIIR_PWMDIF3_Msk                    (1ul << PWM_PIIR_PWMDIF3_Pos)       /*!< PWM_T::PIIR: PWMDIF3 Mask */

#define PWM_PIIR_PWMDIF2_Pos                    10                                  /*!< PWM_T::PIIR: PWMDIF2 Position */
#define PWM_PIIR_PWMDIF2_Msk                    (1ul << PWM_PIIR_PWMDIF2_Pos)       /*!< PWM_T::PIIR: PWMDIF2 Mask */

#define PWM_PIIR_PWMDIF1_Pos                    9                                   /*!< PWM_T::PIIR: PWMDIF1 Position */
#define PWM_PIIR_PWMDIF1_Msk                    (1ul << PWM_PIIR_PWMDIF1_Pos)       /*!< PWM_T::PIIR: PWMDIF1 Mask */

#define PWM_PIIR_PWMDIF0_Pos                    8                                   /*!< PWM_T::PIIR: PWMDIF0 Position */
#define PWM_PIIR_PWMDIF0_Msk                    (1ul << PWM_PIIR_PWMDIF0_Pos)       /*!< PWM_T::PIIR: PWMDIF0 Mask */

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

/* PWM TCON Bit Field Definitions */
#define PWM_TCON_PWM3DTEN_Pos                   11                                  /*!< PWM_T::TCON: PWM3DTEN Position */
#define PWM_TCON_PWM3DTEN_Msk                   (1ul << PWM_TCON_PWM3DTEN_Pos)      /*!< PWM_T::TCON: PWM3DTEN Mask */

#define PWM_TCON_PWM2DTEN_Pos                   10                                  /*!< PWM_T::TCON: PWM2DTEN Position */
#define PWM_TCON_PWM2DTEN_Msk                   (1ul << PWM_TCON_PWM2DTEN_Pos)      /*!< PWM_T::TCON: PWM2DTEN Mask */

#define PWM_TCON_PWM1DTEN_Pos                   9                                   /*!< PWM_T::TCON: PWM1DTEN Position */
#define PWM_TCON_PWM1DTEN_Msk                   (1ul << PWM_TCON_PWM1DTEN_Pos)      /*!< PWM_T::TCON: PWM1DTEN Mask */

#define PWM_TCON_PWM0DTEN_Pos                   8                                   /*!< PWM_T::TCON: PWM0DTEN Position */
#define PWM_TCON_PWM0DTEN_Msk                   (1ul << PWM_TCON_PWM0DTEN_Pos)      /*!< PWM_T::TCON: PWM0DTEN Mask */

#define PWM_TCON_PWM3TEN_Pos                    3                                   /*!< PWM_T::TCON: PWM3TEN Position */
#define PWM_TCON_PWM3TEN_Msk                    (1ul << PWM_TCON_PWM3TEN_Pos)       /*!< PWM_T::TCON: PWM3TEN Mask */

#define PWM_TCON_PWM2TEN_Pos                    2                                   /*!< PWM_T::TCON: PWM2TEN Position */
#define PWM_TCON_PWM2TEN_Msk                    (1ul << PWM_TCON_PWM2TEN_Pos)       /*!< PWM_T::TCON: PWM2TEN Mask */

#define PWM_TCON_PWM1TEN_Pos                    1                                   /*!< PWM_T::TCON: PWM1TEN Position */
#define PWM_TCON_PWM1TEN_Msk                    (1ul << PWM_TCON_PWM1TEN_Pos)       /*!< PWM_T::TCON: PWM1TEN Mask */

#define PWM_TCON_PWM0TEN_Pos                    0                                   /*!< PWM_T::TCON: PWM0TEN Position */
#define PWM_TCON_PWM0TEN_Msk                    (1ul << PWM_TCON_PWM0TEN_Pos)       /*!< PWM_T::TCON: PWM0TEN Mask */

/* PWM TSTATUS Bit Field Definitions */
#define PWM_TSTATUS_PWM3TF_Pos                  3                                   /*!< PWM_T::TSTATUS: PWM3TF Position */
#define PWM_TSTATUS_PWM3TF_Msk                  (1ul << PWM_TSTATUS_PWM3TF_Pos)     /*!< PWM_T::TSTATUS: PWM3TF Mask */

#define PWM_TSTATUS_PWM2TF_Pos                  2                                   /*!< PWM_T::TSTATUS: PWM2TF Position */
#define PWM_TSTATUS_PWM2TF_Msk                  (1ul << PWM_TSTATUS_PWM2TF_Pos)     /*!< PWM_T::TSTATUS: PWM2TF Mask */

#define PWM_TSTATUS_PWM1TF_Pos                  1                                   /*!< PWM_T::TSTATUS: PWM1TF Position */
#define PWM_TSTATUS_PWM1TF_Msk                  (1ul << PWM_TSTATUS_PWM1TF_Pos)     /*!< PWM_T::TSTATUS: PWM1TF Mask */

#define PWM_TSTATUS_PWM0TF_Pos                  0                                   /*!< PWM_T::TSTATUS: PWM0TF Position */
#define PWM_TSTATUS_PWM0TF_Msk                  (1ul << PWM_TSTATUS_PWM0TF_Pos)     /*!< PWM_T::TSTATUS: PWM0TF Mask */

/**@}*/ /* PWM_CONST */
/**@}*/ /* PWM */




/*------------------------- SPI Interface Controller -------------------------*/
/** @addtogroup SPI SPI Controller(SPI)
  Memory Mapped Structure for SPI Controller
  @{
 */

typedef struct
{


/**
 * @var SPI_T::CNTRL
 * Offset: 0x00  SPI Control and Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GO_BUSY   |Go and Busy Status
 * |        |          |If the FIFO mode is disabled, during the data transfer, this bit keeps the value of 1. As
 * |        |          |the transfer is finished, this bit will be cleared automatically.
 * |        |          |In FIFO mode, this bit will be controlled by hardware. Software should not modify this bit.
 * |        |          |0 = Writing 0 to this bit to stop data transfer if SPI is transferring.
 * |        |          |1 = In master mode, writing 1 to this bit to start the SPI data transfer; in slave
 * |        |          |mode, writing 1 to this bit indicates that the slave is ready to communicate with a
 * |        |          |master.
 * |        |          |NOTE: All registers should be set before writing 1 to this GO_BUSY bit. The
 * |        |          |transfer result will be unpredictable if software changes related settings when
 * |        |          |GO_BUSY bit is 1.
 * |[1]     |RX_NEG    |Receive on Negative Edge
 * |        |          |0 = The received data input signal is latched on the rising edge of SPICLK.
 * |        |          |1 = The received data input signal is latched on the falling edge of SPICLK.
 * |[2]     |TX_NEG    |Transmit on Negative Edge
 * |        |          |0 = The transmitted data output signal is changed on the rising edge of SPICLK.
 * |        |          |1 = The transmitted data output signal is changed on the falling edge of SPICLK.
 * |[7:3]   |TX_BIT_LEN|Transmit Bit Length
 * |        |          |This field specifies how many bits are transmitted in one transaction. Up to 32 bits
 * |        |          |can be transmitted.
 * |        |          |TX_BIT_LEN = 0x08 ... 8 bits
 * |        |          |TX_BIT_LEN = 0x09 ... 9 bits
 * |        |          |......
 * |        |          |TX_BIT_LEN = 0x1f ... 31 bits
 * |        |          |TX_BIT_LEN = 0x00 .. 32 bits
 * |[10]    |LSB       |LSB First
 * |        |          |0 = The MSB is transmitted/received first (which bit in SPI_TX0 and SPI_RX0
 * |        |          |register that is depends on the TX_BIT_LEN field).
 * |        |          |1 = The LSB is sent first on the line (bit 0 of SPI_TX0), and the first bit received
 * |        |          |from the line will be put in the LSB position of the RX register (bit 0 of SPI_RX0).
 * |[11]    |CLKP      |Clock Polarity
 * |        |          |0 = SPICLK idle low.
 * |        |          |1 = SPICLK idle high.
 * |[15:12] |SP_CYCLE  |Suspend Interval (master only)
 * |        |          |These four bits provide configurable suspend interval between two successive
 * |        |          |transmit/receive transaction in a transfer. The definition of the suspend interval is the
 * |        |          |interval between the last clock edge of the preceding transaction word and the first
 * |        |          |clock edge of the following transaction word. The default value is 0x3. The period of the
 * |        |          |suspend interval is obtained according to the following equation:
 * |        |          |(SP_CYCLE[3:0] + 0.5)*period of SPI clock
 * |        |          |Example:
 * |        |          |SP_CYCLE = 0x0 ... 0.5 SPICLK clock cycle
 * |        |          |SP_CYCLE = 0x1 ... 1.5 SPICLK clock cycle
 * |        |          |......
 * |        |          |SP_CYCLE = 0xe ... 14.5 SPICLK clock cycle
 * |        |          |SP_CYCLE = 0xf ... 15.5 SPICLK clock cycle
 * |[16]    |IF        |Interrupt Flag
 * |        |          |0 = Indicates that the transfer does not finish yet.
 * |        |          |1 = Indicates that the SPI controller has finished one unit transfer.
 * |        |          |NOTE: This bit can be cleared by writing 1 to itself.
 * |[17]    |IE        |Interrupt Enable
 * |        |          |0 = Disable SPI unit transfer interrupt.
 * |        |          |1 = Enable SPI unit transfer interrupt.
 * |[18]    |SLAVE     |Slave Mode Enable Bit
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
 * Offset: 0x04  SPI Clock Divider Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DIVIDER   |Clock Divider Register
 * |        |          |The value in this field is the frequency divider for generating the SPI peripheral clock and the SPI bus clock of SPI master.
 * |        |          |The frequency is obtained according to the following equation.
 * |        |          |If the bit of BCn, SPI_CNTRL2[31], is set to 0,
 * |        |          |   SPI peripheral clock frequency = SPI peripheral clock source frequency / (DIVIDER + 1) / 2
 * |        |          |else if BCn is set to 1,
 * |        |          |   SPI peripheral clock frequency = SPI peripheral clock source frequency / (DIVIDER + 1)
 * |        |          |The SPI peripheral clock source is defined in the CLKSEL1 register.
 * @var SPI_T::SSR
 * Offset: 0x08  SPI Slave Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SSR       |Slave Select Register (master only)
 * |        |          |If AUTOSS bit is cleared, writing 1 to this bit sets the SPISS line to active state and
 * |        |          |writing 0 sets the line back to inactive state.
 * |        |          |If AUTOSS bit is set, writing 1 to this bit will select the SPISS line to be automatically
 * |        |          |driven to active state for the duration of the transmit/receive, and will be driven to
 * |        |          |inactive state for the rest of the time. The active level of SPISS is specified in SS_LVL.
 * |[2]     |SS_LVL    |Slave Select Active Level
 * |        |          |It defines the active level of slave select signal (SPISS).
 * |        |          |0 = The slave select signal SPISS is active at low-level/falling-edge.
 * |        |          |1 = The slave select signal SPISS is active at high-level/rising-edge.
 * |[3]     |AUTOSS    |Automatic Slave Select (master only)
 * |        |          |0 = If this bit is cleared, slave select signal will be asserted and de-asserted by setting
 * |        |          |and clearing SSR[0].
 * |        |          |1 = If this bit is set, SPISS signal is generated automatically. It means that slave
 * |        |          |select signal will be asserted by the SPI controller when transmit/receive is started by
 * |        |          |setting GO_BUSY, and is de-asserted after each transmit/receive is finished.
 * |[4]     |SS_LTRIG  |Slave Select Level Trigger (slave only)
 * |        |          |0: The input slave select signal is edge-trigger. This is default value.
 * |        |          |1: The slave select signal will be level-trigger. It depends on SS_LVL to decide the
 * |        |          |signal is active low or active high.
 * |[5]     |LTRIG_FLAG|Level Trigger Accomplish Flag
 * |        |          |In slave mode, this bit indicates whether the received bit number meets the requirement
 * |        |          |or not after the current transaction done.
 * |        |          |0 = The transferred bit length of one transaction does not meet the specified requirement.
 * |        |          |1 = The transferred bit length meets the specified requirement which defined in TX_BIT_LEN.
 * |        |          |Note: This bit is READ only
 * @var SPI_T::RX0
 * Offset: 0x10  Data Receive Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |RX        |Data Receive Register
 * |        |          |The data receive register holds the datum received from SPI data input pin. If the FIFO
 * |        |          |mode is disabled, the software can access the last received data by reading this
 * |        |          |register. If the FIFO bit is set as 1 and the RX_EMPTY bit, SPI_CNTRL[24] or SPI_STATUS[24],
 * |        |          |is not set to 1, the software can access the receive FIFO buffer by reading this register.
 * |        |          |NOTE: This is a read only register.
 * @var SPI_T::TX0
 * Offset: 0x20  Data Transmit Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |TX        |Data Transmit Register
 * |        |          |The Data Transmit Registers hold the data to be transmitted in the next transfer. The
 * |        |          |number of valid bits depends on the setting of transmit bit length field of the SPI_CNTRL register.
 * |        |          |For example, if TX_BIT_LEN is set to 0x08, the bit TX[7:0] will be transmitted in next
 * |        |          |transfer. If TX_BIT_LEN is set to 0x00, the SPI controller will perform a 32-bit transfer.
 * |        |          |Note: when the SPI controller is configured as a slave device and the FIFO mode is
 * |        |          |disabled, if the SPI controller attempts to transmit data to a master, the software must
 * |        |          |update the transmit data register before setting the GO_BUSY bit to 1.
 * @var SPI_T::CNTRL2
 * Offset: 0x3C  SPI Control and Status Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8]     |NOSLVSEL  |Slave 3-Wire Mode Enable Bit
 * |        |          |This is used to ignore the slave select signal in slave mode.
 * |        |          |The SPI controller can work with 3-wire interface including SPICLK, MISO, and MOSI.
 * |        |          |0 = 4-wire bi-direction interface.
 * |        |          |1 = 3-wire bi-direction interface.
 * |        |          |Note: In Slave 3-wire mode, the SS_LTRIG, SPI_SSR[4], shall be set as 1.
 * |[9]     |SLV_ABORT |Slave 3-Wire Mode Abort Control Bit
 * |        |          |In normal operation, there is an interrupt event when the received data meet the required bits which defined in TX_BIT_LEN.
 * |        |          |If the received bits are less than the requirement and there is no more serial clock input over the one transfer time in slave 3-wire mode,
 * |        |          |the user can set this bit to force the current transfer done and then the user can get a transfer done interrupt event.
 * |        |          |Note: It will be cleared to 0 automatically by hardware after the software sets this bit to 1.
 * |[10]    |SSTA_INTEN|Slave 3-Wire Mode Start Interrupt Enable
 * |        |          |It is used to enable interrupt when the transfer has started in slave 3-wire mode.
 * |        |          |If there is no transfer done interrupt over the time period which is defined by user after the transfer start,
 * |        |          |the user can set the SLV_ABORT bit to force the transfer done.
 * |        |          |0 = Disable the transfer start interrupt.
 * |        |          |1 = Enable the transaction start interrupt. It will be cleared to 0 as the current transfer is done or the SLV_START_INTSTS bit is cleared.
 * |[11]    |SLV_START_INTSTS|Slave 3-Wire Mode Start Interrupt Status
 * |        |          |It is used to dedicate that the transfer has started in slave 3-wire mode.
 * |        |          |0 = It indicates that the SPI transfer is not active.
 * |        |          |1 = It indicates that the transfer has started in slave 3-wire mode. It will be cleared to 0 as transfer done or by writing one to this bit.
 * |[16]    |SS_INT_OPT|Slave Select Inactive Interrupt Option
 * |        |          |This setting is only available if the SPI controller is configured as level trigger slave device.
 * |        |          |0 = As the slave select signal goes to inactive level, the IF bit will NOT be set to 1.
 * |        |          |1 = As the slave select signal goes to inactive level, the IF bit will be set to 1.
 * |[31]    |BCn       |SPI Engine Clock Backward Compatible Option
 * |        |          |0 = Backward compatible clock configuration.
 * |        |          |1 = The clock configuration is not backward compatible.
 * |        |          |Note: Refer to the description of SPI_DIVIDER register for details.
 * @var SPI_T::FIFO_CTL
 * Offset: 0x40  SPI FIFO Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RX_CLR    |Clear Receive FIFO Buffer
 * |        |          |0 = No effect.
 * |        |          |1 = Clear receive FIFO buffer. The RX_FULL flag will be cleared to 0 and the RX_EMPTY flag will be set to 1.
 * |        |          |This bit will be cleared to 0 by hardware after software sets it to 1.
 * |[1]     |TX_CLR    |Clear Transmit FIFO Buffer
 * |        |          |0 = No effect.
 * |        |          |1 = Clear transmit FIFO buffer. The TX_FULL flag will be cleared to 0 and the TX_EMPTY flag will be set to 1.
 * |        |          |This bit will be cleared to 0 by hardware after software sets it to 1.
 * |[2]     |RX_INTEN  |Receive Threshold Interrupt Enable
 * |        |          |0 = Disable receive threshold interrupt.
 * |        |          |1 = Enable receive threshold interrupt.
 * |[3]     |TX_INTEN  |Transmit Threshold Interrupt Enable
 * |        |          |0 = Disable transmit threshold interrupt.
 * |        |          |1 = Enable transmit threshold interrupt.
 * |[6]     |RXOV_INTEN|Receive FIFO Overrun Interrupt Enable
 * |        |          |0 = Disable Receive FIFO overrun interrupt.
 * |        |          |1 = Enable Receive FIFO overrun interrupt.
 * |[21]    |TIMEOUT_INTEN|Receive FIFO Time-out Interrupt Enable
 * |        |          |0 = Disable time-out interrupt.
 * |        |          |1 = Enable time-out interrupt.
 * |[25:24] |RX_THRESHOLD |Received FIFO Threshold
 * |        |          |If the valid data count of the receive FIFO buffer is larger than the RX_THRESHOLD setting,
 * |        |          |the RX_INTSTS bit will be set to 1, else the RX_INTSTS bit will be cleared to 0.
 * |[29:28] |RX_THRESHOLD |Transmit FIFO Threshold
 * |        |          |If the valid data count of the transmit FIFO buffer is less than or equal to the TX_THRESHOLD setting,
 * |        |          |the TX_INTSTS bit will be set to 1, else the TX_INTSTS bit will be cleared to 0.
 * @var SPI_T::STATUS
 * Offset: 0x44  SPI Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RX_INTSTS |Receive FIFO Threshold Interrupt Status (read only)
 * |        |          |0 = It indicates that the valid data count within the Rx FIFO buffer is smaller than or equal to the setting value of RX_THRESHOLD.
 * |        |          |1 = It indicates that the valid data count within the receive FIFO buffer is larger than the setting value of RX_THRESHOLD.
 * |        |          |Note: If RX_INTEN = 1 and RX_INTSTS = 1, the SPI controller will generate a SPI interrupt request.
 * |[2]     |RX_OVERRUN|Receive FIFO Overrun Status
 * |        |          |When the receive FIFO buffer is full, the follow-up data will be dropped and this bit will be set to 1.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[4]     |TX_INTSTS |Transmit FIFO Threshold Interrupt Status (read only)
 * |        |          |0 = It indicates that the valid data count within the transmit FIFO buffer is larger than the setting value of TX_THRESHOLD.
 * |        |          |1 = It indicates that the valid data count within the transmit FIFO buffer is less than or equal to the setting value of TX_THRESHOLD.
 * |        |          |Note: If TX_INTEN = 1 and TX_INTSTS = 1, the SPI controller will generate a SPI interrupt request.
 * |[11]    |SLV_START_INTSTS|Slave Start Interrupt Status
 * |        |          |It is used to dedicate that the transfer has started in slave 3-wire mode. It's a mutual mirror bit of SPI_CNTRL2[11].
 * |        |          |0 = It indicates that the transfer is not started.
 * |        |          |1 = It indicates that the transfer has started in slave 3-wire mode. It will be cleared as transfer done or by writing one to this bit.
 * |[15:12] |RX_FIFO_COUNT   |Receive FIFO Data Count (read only)
 * |        |          |Indicates the valid data count of receive FIFO buffer.
 * |[16]    |IF        |SPI Unit Transfer Interrupt Flag
 * |        |          |It's a mutual mirror bit of SPI_CNTRL[16].
 * |        |          |0 = It indicates that the transfer does not finish yet.
 * |        |          |1 = It indicates that the SPI controller has finished one unit transfer.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[20]    |TIMEOUT   |Time-out Interrupt Flag
 * |        |          |0 = No receive FIFO time-out event.
 * |        |          |1 = It indicates that the receive FIFO buffer is not empty and there is not be read over 64 SPI clock period in master
 * |        |          |mode and over 576 SPI engine clock period in slave mode. When the received FIFO buffer is read by software, the time-out
 * |        |          |status will be cleared automatically.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[24]    |RX_EMPTY  |Receive FIFO Buffer Empty Indicator (read only)
 * |        |          |It's a mutual mirror bit of SPI_CNTRL[24].
 * |        |          |0 = Indicates that the receive FIFO buffer is not empty
 * |        |          |1 = Indicates that the receive FIFO buffer is empty
 * |[25]    |RX_FULL   |Receive FIFO Buffer Full Indicator (read only)
 * |        |          |It's a mutual mirror bit of SPI_CNTRL[25].
 * |        |          |0 = Indicates that the receive FIFO buffer is not full
 * |        |          |1 = Indicates that the receive FIFO buffer is full
 * |[26]    |TX_EMPTY  |Transmit FIFO Buffer Empty Indicator (read only)
 * |        |          |It's a mutual mirror bit of SPI_CNTRL[26].
 * |        |          |0 = Indicates that the transmit FIFO buffer is not empty
 * |        |          |1 = Indicates that the transmit FIFO buffer is empty
 * |[27]    |TX_FULL   |Transmit FIFO Buffer Full Indicator (read only)
 * |        |          |It's a mutual mirror bit of SPI_CNTRL[27].
 * |        |          |0 = Indicates that the transmit FIFO buffer is not full
 * |        |          |1 = Indicates that the transmit FIFO buffer is full
 * |[31:28] |TX_FIFO_COUNT   |Transmit FIFO Data Count (read only)
 * |        |          |Indicates the valid data count of transmit FIFO buffer.
 */

    __IO uint32_t CNTRL;         /* Offset: 0x00  SPI Control and Status Register                                    */
    __IO uint32_t DIVIDER;       /* Offset: 0x04  SPI Clock Divider Register                                         */
    __IO uint32_t SSR;           /* Offset: 0x08  SPI Slave Select Register                                          */
    __I  uint32_t RESERVE0;     
    __I  uint32_t RX0;           /* Offset: 0x10  Data Receive Register                                              */
    __I  uint32_t RESERVE1[2];  
    __I  uint32_t RESERVE2;     
    __O  uint32_t TX0;           /* Offset: 0x20  Data Transmit Register                                             */
    __I  uint32_t RESERVE3[2];  
    __I  uint32_t RESERVE4;     
    __I  uint32_t RESERVE5;     
    __I  uint32_t RESERVE6;     
    __I  uint32_t RESERVE7;     
    __IO uint32_t CNTRL2;        /* Offset: 0x3C  SPI Control and Status Register 2                                  */
    __IO uint32_t FIFO_CTL;      /* Offset: 0x40  SPI FIFO Control Register                                          */
    __IO uint32_t STATUS;        /* Offset: 0x44  SPI Status Register                                                */

} SPI_T;



/** @addtogroup SPI_CONST SPI Bit Field Definition
  Constant Definitions for SPI Controller
  @{
 */

/* SPI_CNTRL Bit Field Definitions */
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
#define SPI_SSR_SSR_Msk            (1ul << SPI_SSR_SSR_Pos)          /*!< SPI_T::SSR: SSR Mask */

/* SPI_CNTRL2 Bit Field Definitions */
#define SPI_CNTRL2_BCn_Pos                31                                         /*!< SPI_T::CNTRL2: BCn Position */
#define SPI_CNTRL2_BCn_Msk                (1ul << SPI_CNTRL2_BCn_Pos)                /*!< SPI_T::CNTRL2: BCn Mask */

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

#define SPI_CNTRL2_DIV_ONE_Pos            0                                          /*!< SPI_T::CNTRL2: DIV_ONE Position */
#define SPI_CNTRL2_DIV_ONE_Msk            (1ul << SPI_CNTRL2_DIV_ONE_Pos)            /*!< SPI_T::CNTRL2: DIV_ONE Mask */

/* SPI_FIFO_CTL Bit Field Definitions */
#define SPI_FIFO_CTL_TX_THRESHOLD_Pos     28                                         /*!< SPI_T::FIFO_CTL: TX_THRESHOLD Position */
#define SPI_FIFO_CTL_TX_THRESHOLD_Msk     (3ul << SPI_FIFO_CTL_TX_THRESHOLD_Pos)     /*!< SPI_T::FIFO_CTL: TX_THRESHOLD Mask */

#define SPI_FIFO_CTL_RX_THRESHOLD_Pos     24                                         /*!< SPI_T::FIFO_CTL: RX_THRESHOLD Position */
#define SPI_FIFO_CTL_RX_THRESHOLD_Msk     (3ul << SPI_FIFO_CTL_RX_THRESHOLD_Pos)     /*!< SPI_T::FIFO_CTL: RX_THRESHOLD Mask */

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

/* SPI_STATUS Bit Field Definitions */
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
/**@}*/ /* SPI_CONST */
/**@}*/ /* SPI */




/*---------------------------- Clock Controller ------------------------------*/

/** @addtogroup CLK System Clock Controller(CLK)
  Memory Mapped Structure for System Clock Controller
  @{
 */


typedef struct
{


/**
 * @var CLK_T::PWRCON
 * Offset: 0x00  System Power Down Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field       |Descriptions
 * | :----: | :----:     | :---- |
 * |[0]     |XTL12M_EN   |External 4~24 MHz High Speed Crystal Enable (HXT) Control (Write Protect)
 * |        |            |The bit default value is set by flash controller user configuration register CONFIG0 [26:24].
 * |        |            |When the default clock source is from external 4~24 MHz high speed crystal, this bit is set to 1 automatically.
 * |        |            |0 = External 4~24 MHz high speed crystal oscillator (HXT) Disabled.
 * |        |            |1 = External 4~24 MHz high speed crystal oscillator (HXT) Enabled.
 * |        |            |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[2]     |OSC22M_EN   |Internal 22.1184 MHz High Speed Oscillator (HIRC) Enable Control (Write Protect)
 * |        |            |0 = Internal 22.1184 MHz high speed oscillator (HIRC) Disabled.
 * |        |            |1 = Internal 22.1184 MHz high speed oscillator (HIRC) Enabled.
 * |        |            |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[3]     |OSC10K_EN   |Internal 10 KHz Low Speed Oscillator (LIRC) Enable Control (Write Protect)
 * |        |            |0 = Internal 10 kHz low speed oscillator (LIRC) Disabled.
 * |        |            |1 = Internal 10 kHz low speed oscillator (LIRC) Enabled.
 * |        |            |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[4]     |PD_WU_DLY   |Wake-up Delay Counter Enable Control (Write Protect)
 * |        |            |When the chip wakes up from Power-down mode, the clock control will delay certain clock cycles to wait system clock stable.
 * |        |            |The delayed clock cycle is 4096 clock cycles when chip work at external 4~24 MHz high speed crystal, and 256 clock cycles when chip work at internal 22.1184 MHz high speed oscillator.
 * |        |            |0 = Clock cycles delay Disabled.
 * |        |            |1 = Clock cycles delay Enabled.
 * |        |            |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[5]     |PD_WU_INT_EN|Power-Down Mode Wake-Up Interrupt Enable Control (Write Protect)
 * |        |            |0 = Disabled.
 * |        |            |1 = Enabled.
 * |        |            |Note1: The interrupt will occur when both PD_WU_STS and PD_WU_INT_EN are high.
 * |        |            |Note2: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[6]     |PD_WU_STS   |Power-Down Mode Wake-Up Interrupt Status
 * |        |            |Set by "Power-down wake-up event", it indicates that resume from Power-down mode.
 * |        |            |The flag is set if the GPIO, UART, WDT or BOD wake-up occurred. Write 1 to clear the bit to 0.
 * |        |            |Note: This bit is working only if PD_WU_INT_EN (PWRCON[5]) set to 1.
 * |[7]     |PWR_DOWN_EN |System Power-down Enable Bit (Write Protect)
 * |        |            |When this bit is set to 1, Power-down mode is enabled and chip Power-down behavior will depend on the PD_WAIT_CPU bit.
 * |        |            |(a) If the PD_WAIT_CPU is 0, then the chip enters Power-down mode immediately after the PWR_DOWN_EN bit set.
 * |        |            |(b) If the PD_WAIT_CPU is 1, then the chip keeps active till CPU run WFI instruction.
 * |        |            |When chip wakes up from Power-down mode, this bit is cleared by hardware. User needs to set this bit again for next Power-down.
 * |        |            |In Power-down mode, 4~24 MHz external high speed crystal oscillator (HXT) and the 22.1184 MHz internal high speed oscillator (HIRC) will be disabled in this mode, and 10 kHz internal low speed RC oscillator (LIRC) are not controlled by Power-down mode.
 * |        |            |In Power- down mode, the PLL and system clock are disabled, and ignored the clock source selection. The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from 10 kHz internal low speed oscillator.
 * |        |            |0 = Chip operating normally or chip in Idle mode because of WFI command.
 * |        |            |1 = Chip enters Power-down mode instantly or waits CPU sleep command WFI.
 * |        |            |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[8]     |PD_WAIT_CPU|This Bit Control The Power-Down Entry Condition (Write Protect)
 * |        |            |0 = Chip enters Power-down mode when the PWR_DOWN_EN bit is set to 1.
 * |        |            |1 = Chip enters Power-down mode when the both PD_WAIT_CPU and PWR_DOWN_EN bits are set to 1 and CPU run WFI instruction.
 * |        |            |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * @var CLK_T::AHBCLK
 * Offset: 0x04  AHB Devices Clock Enable Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2]     |ISP_EN    |Flash ISP Controller Clock Enable Control
 * |        |          |0 = Flash ISP peripheral clock Disabled.
 * |        |          |1 = Flash ISP peripheral clock Enabled.
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
 * |[16]    |UART0_EN  |UART0 Clock Enable Control
 * |        |          |0 = UART0 clock Disabled.
 * |        |          |1 = UART0 clock Enabled.
 * |[20]    |PWM01_EN  |PWM_01 Clock Enable Control
 * |        |          |0 = PWM01 clock Disabled.
 * |        |          |1 = PWM01 clock Enabled.
 * |[21]    |PWM23_EN  |PWM_23 Clock Enable Control
 * |        |          |0 = PWM23 clock Disabled.
 * |        |          |1 = PWM23 clock Enabled.
 * |[28]    |ADC_EN    |Analog-Digital-Converter (ADC) Clock Enable Control
 * |        |          |0 = ADC clock Disabled.
 * |        |          |1 = ADC clock Enabled.
 * @var CLK_T::CLKSTATUS
 * Offset: 0x0C  Clock Status Monitor Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field      |Descriptions
 * | :----: | :----:    | :---- |
 * |[0]     |XTL12M_STB|External 4~24 MHz High Speed Crystal (HXT) Clock Source Stable Flag (Read Only)
 * |        |           |0 = External 4~24 MHz high speed crystal clock (HXT) is not stable or disabled.
 * |        |           |1 = External 4~24 MHz high speed crystal clock (HXT) is stable.
 * |[2]     |PLL_STB    |Internal PLL Clock Source Stable Flag (Read Only)
 * |        |           |0 = Internal PLL clock is not stable or disabled.
 * |        |           |1 = Internal PLL clock is stable.
 * |[3]     |OSC10K_STB|Internal 10 KHz Low Speed Oscillator (LIRC) Clock Source Stable Flag (Read Only)
 * |        |           |0 = Internal 10 kHz low speed oscillator clock (LIRC) is not stable or disabled.
 * |        |           |1 = Internal 10 kHz low speed oscillator clock (LIRC) is stable.
 * |[4]     |OSC22M_STB|Internal 22.1184 MHz High Speed Oscillator (HIRC) Clock Source Stable Flag (Read Only)
 * |        |           |0 = Internal 22.1184 MHz high speed oscillator (HIRC) clock is not stable or disabled.
 * |        |           |1 = Internal 22.1184 MHz high speed oscillator (HIRC) clock is stable.
 * |[7]     |CLK_SW_FAIL|Clock switch fail flag
 * |        |           |0 = Clock switching success.
 * |        |           |1 = Clock switching failure.
 * |        |           |This bit will be set when target switch clock source is not stable. Write 1 to clear this bit to zero.
 * @var CLK_T::CLKSEL0
 * Offset: 0x10  Clock Source Select Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |HCLK_S    |HCLK Clock Source Selection (Write Protect)
 * |        |          |000 = Clock source is from HXT.
 * |        |          |010 = Clock source is from PLL.
 * |        |          |011 = Clock source is from LIRC.
 * |        |          |111 = Clock source is from HIRC.
 * |        |          |Others = Reserved.
 * |        |          |Note1: Before clock switching, the related clock sources (both pre-select and new-select) must be turn-on and stable.
 * |        |          |Note2: The 3-bit default value is reloaded from the value of CFOSC (CONFIG0[26:24]) in user configuration register of Flash controller by any reset. Therefore the default value is either 000b or 111b.
 * |        |          |Note3: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[5:3]   |STCLK_S   |Cortex-M0 SysTick Clock Source Selection from Reference Clock (Write Protect)
 * |        |          |If SYST_CSR[2] = 1, SysTick clock source is from HCLK.
 * |        |          |If SYST_CSR[2] = 0, SysTick clock source is defined by below settings.
 * |        |          |000 = Clock source is from HXT.
 * |        |          |010 = Clock source is from HXT/2
 * |        |          |011 = Clock source is from HCLK/2.
 * |        |          |111 = Clock source is from HIRC/2.
 * |        |          |Others = Reserved.
 * |        |          |Note1: This bit is write protected bit. Refer to the REGWRPROT register.
 * |        |          |Note2: if the SysTick clock source is not from HCLK (i.e. SYST_CSR[2] = 0), SysTick clock source must less than or equal to HCLK/2.
 * @var CLK_T::CLKSEL1
 * Offset: 0x14  Clock Source Select Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |WDT_S     |Watchdog Timer Clock Source Selection (Write Protect)
 * |        |          |10 = Clock source is from HCLK/2048 clock.
 * |        |          |11 = Clock source is from LIRC.
 * |        |          |Note: This bit is write protected bit. Refer to the REGWRPROT register.
 * |[3:2]   |ADC_S     |ADC Peripheral Clock Source Selection
 * |        |          |00 = Clock source is from HXT.
 * |        |          |01 = Clock source is from PLL.
 * |        |          |10 = Clock source is from HCLK.
 * |        |          |11 = Clock source is from HIRC.
 * |[4]     |SPI0_S    |SPI0 clock Source Selection
 * |        |          |0 = Clock source is from PLL.
 * |        |          |1 = Clock source is from HCLK.
 * |[10:8]  |TMR0_S    |TIMER0 Clock Source Selection
 * |        |          |000 = Clock source is from HXT.
 * |        |          |010 = Clock source is from HCLK.
 * |        |          |011 = Clock source is from external trigger T0.
 * |        |          |101 = Clock source is from LIRC.
 * |        |          |111 = Clock source is from HIRC.
 * |[14:12] |TMR1_S    |TIMER1 Clock Source Selection
 * |        |          |000 = Clock source is from HXT.
 * |        |          |010 = Clock source is from HCLK.
 * |        |          |011 = Clock source is from external trigger T1.
 * |        |          |101 = Clock source is from LIRC.
 * |        |          |111 = Clock source is from HIRC.
 * |[18:16] |TMR2_S    |TIMER2 Clock Source Selection
 * |        |          |000 = Clock source is from HXT.
 * |        |          |010 = Clock source is from HCLK.
 * |        |          |011 = Clock source is from external trigger T2.
 * |        |          |101 = Clock source is from LIRC.
 * |        |          |111 = Clock source is from HIRC.
 * |[22:20] |TMR3_S    |TIMER3 Clock Source Selection
 * |        |          |000 = Clock source is from HXT.
 * |        |          |010 = Clock source is from HCLK.
 * |        |          |011 = Clock source is from external trigger T3.
 * |        |          |101 = Clock source is from LIRC.
 * |        |          |111 = Clock source is from HIRC.
 * |[25:24] |UART_S    |UART Clock Source Selection
 * |        |          |00 = Clock source is from HXT.
 * |        |          |01 = Clock source is from PLL.
 * |        |          |11 = Clock source is from HIRC.
 * |[29:28] |PWM01_S   |PWM0 and PWM1 Clock Source Selection
 * |        |          |PWM0 and PWM1 use the same clock source; both of them use the same prescaler.
 * |        |          |00 = Clock source is from HXT.
 * |        |          |01 = Clock source is from LIRC.
 * |        |          |10 = Clock source is from HCLK.
 * |        |          |11 = Clock source is from HIRC.
 * |[31:30] |PWM23_S   |PWM2 and PWM3 Clock Source Selection
 * |        |          |PWM2 and PWM3 use the same clock source; both of them use the same prescaler.
 * |        |          |00 = Clock source is from HXT.
 * |        |          |01 = Clock source is from LIRC.
 * |        |          |10 = Clock source is from HCLK.
 * |        |          |11 = Clock source is from HIRC.
 * @var CLK_T::CLKDIV
 * Offset: 0x18  Clock Divider Number Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |HCLK_N    |HCLK Clock Divide Number from HCLK Clock Source
 * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLK_N + 1).
 * |[11:8]  |UART_N    |UART Clock Divide Number from UART Clock Source
 * |        |          |UART clock frequency = (UART clock source frequency ) / (UART_N + 1).
 * |[23:16] |ADC_N     |ADC peripheral Clock Divide Number from ADC peripheral Clock Source
 * |        |          |ADC peripheral clock frequency = (ADC peripheral clock source frequency ) / (ADC_N + 1).
 * @var CLK_T::CLKSEL2
 * Offset: 0x1C  Clock Source Select Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:2]   |FRQDIV_S  |Clock Divider Clock Source Select
 * |        |          |00 = clock source from external crystal clock (4 ~ 24MHz)
 * |        |          |10 = clock source from HCLK
 * |        |          |11 = clock source from internal 22.1184 MHz oscillator clock
 * |[17:16] |WWDT_S    |Window Watchdog Timer Clock Source Selection
 * |        |          |10 = Clock source is from HCLK/2048 clock.
 * |        |          |11 = Clock source is from LIRC.
 * @var CLK_T::PLLCON
 * Offset: 0x20  PLL Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8:0]   |FB_DV     |PLL Feedback Divider Control Bits
 * |        |          |Refer to the formulas below the table.
 * |[13:9]  |IN_DV     |PLL Input Divider Control Bits
 * |        |          |Refer to the formulas below the table.
 * |[15:14] |OUT_DV    |PLL Output Divider Control Bits
 * |        |          |Refer to the formulas below the table.
 * |[16]    |PD        |Power-Down Mode
 * |        |          |If the PWR_DOWN_EN bit is set to 1 in PWRCON register, the PLL will enter Power-down mode too.
 * |        |          |0 = PLL is in Normal mode.
 * |        |          |1 = PLL is in Power-down mode (default).
 * |[17]    |BP        |PLL Bypass Control
 * |        |          |0 = PLL is in Normal mode (default).
 * |        |          |1 = PLL clock output is same as PLL source clock input.
 * |[18]    |OE        |PLL OE (FOUT Enable) Pin Control
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
 * |        |          |The formula of output frequency is Fout = Fin/2(N+1).
 * |        |          |Fin is the input clock frequency.
 * |        |          |Fout is the frequency of divider output clock.
 * |        |          |N is the 4-bit value of FSEL[3:0].
 * |[4]     |DIVIDER_EN|Frequency Divider Enable Bit
 * |        |          |0 = Frequency Divider function Disabled.
 * |        |          |1 = Frequency Divider function Enabled.
 */

    __IO uint32_t PWRCON;        /* Offset: 0x00  System Power Down Control Register                                 */
    __IO uint32_t AHBCLK;        /* Offset: 0x04  AHB Devices Clock Enable Control Register                          */
    __IO uint32_t APBCLK;        /* Offset: 0x08  APB Devices Clock Enable Control Register                          */
    __IO uint32_t CLKSTATUS;     /* Offset: 0x0C  Clock Status Monitor Register                                      */
    __IO uint32_t CLKSEL0;       /* Offset: 0x10  Clock Source Select Control Register 0                             */
    __IO uint32_t CLKSEL1;       /* Offset: 0x14  Clock Source Select Control Register 1                             */
    __IO uint32_t CLKDIV;        /* Offset: 0x18  Clock Divider Number Register                                      */
    __IO uint32_t CLKSEL2;       /* Offset: 0x1C  Clock Source Select Control Register 2                             */
    __IO uint32_t PLLCON;        /* Offset: 0x20  PLL Control Register                                               */
    __IO uint32_t FRQDIV;        /* Offset: 0x24  Frequency Divider Control Register                                 */

} CLK_T;



/** @addtogroup CLK_CONST CLK Bit Field Definition
  Constant Definitions for CLK Controller
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
#define CLK_PWRCON_IRC10K_EN_Pos             3                                    /*!< CLK_T::PWRCON: OSC10K_EN Position */
#define CLK_PWRCON_IRC10K_EN_Msk             (1ul << CLK_PWRCON_OSC10K_EN_Pos)    /*!< CLK_T::PWRCON: OSC10K_EN Mask */

#define CLK_PWRCON_OSC22M_EN_Pos             2                                    /*!< CLK_T::PWRCON: OSC22M_EN Position */
#define CLK_PWRCON_OSC22M_EN_Msk             (1ul << CLK_PWRCON_OSC22M_EN_Pos)    /*!< CLK_T::PWRCON: OSC22M_EN Mask */
#define CLK_PWRCON_IRC22M_EN_Pos             2                                    /*!< CLK_T::PWRCON: OSC22M_EN Position */
#define CLK_PWRCON_IRC22M_EN_Msk             (1ul << CLK_PWRCON_OSC22M_EN_Pos)    /*!< CLK_T::PWRCON: OSC22M_EN Mask */

#define CLK_PWRCON_XTL12M_EN_Pos             0                                    /*!< CLK_T::PWRCON: XTL12M_EN Position */
#define CLK_PWRCON_XTL12M_EN_Msk             (1ul << CLK_PWRCON_XTL12M_EN_Pos)    /*!< CLK_T::PWRCON: XTL12M_EN Mask */

/* CLK AHBCLK Bit Field Definitions */
#define CLK_AHBCLK_ISP_EN_Pos                2                                    /*!< CLK_T::AHBCLK: ISP_EN Position */
#define CLK_AHBCLK_ISP_EN_Msk                (1ul << CLK_AHBCLK_ISP_EN_Pos)       /*!< CLK_T::AHBCLK: ISP_EN Mask */

/* CLK APBCLK Bit Field Definitions */
#define CLK_APBCLK_ADC_EN_Pos                28                                   /*!< CLK_T::APBCLK: ADC_EN Position */
#define CLK_APBCLK_ADC_EN_Msk                (1ul << CLK_APBCLK_ADC_EN_Pos)       /*!< CLK_T::APBCLK: ADC_EN Mask */

#define CLK_APBCLK_PWM23_EN_Pos              21                                   /*!< CLK_T::APBCLK: PWM23_EN Position */
#define CLK_APBCLK_PWM23_EN_Msk              (1ul << CLK_APBCLK_PWM23_EN_Pos)     /*!< CLK_T::APBCLK: PWM23_EN Mask */

#define CLK_APBCLK_PWM01_EN_Pos              20                                   /*!< CLK_T::APBCLK: PWM01_EN Position */
#define CLK_APBCLK_PWM01_EN_Msk              (1ul << CLK_APBCLK_PWM01_EN_Pos)     /*!< CLK_T::APBCLK: PWM01_EN Mask */

#define CLK_APBCLK_UART0_EN_Pos              16                                   /*!< CLK_T::APBCLK: UART0_EN Position */
#define CLK_APBCLK_UART0_EN_Msk              (1ul << CLK_APBCLK_UART0_EN_Pos)     /*!< CLK_T::APBCLK: UART0_EN Mask */

#define CLK_APBCLK_SPI0_EN_Pos               12                                   /*!< CLK_T::APBCLK: SPI0_EN Position */
#define CLK_APBCLK_SPI0_EN_Msk               (1ul << CLK_APBCLK_SPI0_EN_Pos)      /*!< CLK_T::APBCLK: SPI0_EN Mask */

#define CLK_APBCLK_I2C1_EN_Pos               9                                    /*!< CLK_T::APBCLK: I2C1_EN Position */
#define CLK_APBCLK_I2C1_EN_Msk               (1ul << CLK_APBCLK_I2C1_EN_Pos)      /*!< CLK_T::APBCLK: I2C1_EN Mask */

#define CLK_APBCLK_I2C0_EN_Pos               8                                    /*!< CLK_T::APBCLK: I2C0_EN Position */
#define CLK_APBCLK_I2C0_EN_Msk               (1ul << CLK_APBCLK_I2C0_EN_Pos)      /*!< CLK_T::APBCLK: I2C0_EN Mask */
#define CLK_APBCLK_I2C_EN_Pos                8                                    /*!< CLK_T::APBCLK: I2C_EN Position */
#define CLK_APBCLK_I2C_EN_Msk                (1ul << CLK_APBCLK_I2C_EN_Pos)       /*!< CLK_T::APBCLK: I2C_EN Mask */

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
#define CLK_CLKSTATUS_CLK_SW_FAIL_Pos        7                                     /*!< CLK_T::CLKSTATUS: CLK_SW_FAIL Position */
#define CLK_CLKSTATUS_CLK_SW_FAIL_Msk        (1ul << CLK_CLKSTATUS_CLK_SW_FAIL_Pos)/*!< CLK_T::CLKSTATUS: CLK_SW_FAIL Mask */

#define CLK_CLKSTATUS_OSC22M_STB_Pos         4                                    /*!< CLK_T::CLKSTATUS: OSC22M_STB Position */
#define CLK_CLKSTATUS_OSC22M_STB_Msk         (1ul << CLK_CLKSTATUS_OSC22M_STB_Pos)/*!< CLK_T::CLKSTATUS: OSC22M_STB Mask */
#define CLK_CLKSTATUS_IRC22M_STB_Pos         4                                    /*!< CLK_T::CLKSTATUS: IRC22M_STB Position */
#define CLK_CLKSTATUS_IRC22M_STB_Msk         (1ul << CLK_CLKSTATUS_IRC22M_STB_Pos)/*!< CLK_T::CLKSTATUS: IRC22M_STB Mask */

#define CLK_CLKSTATUS_OSC10K_STB_Pos         3                                    /*!< CLK_T::CLKSTATUS: OSC10K_STB Position */
#define CLK_CLKSTATUS_OSC10K_STB_Msk         (1ul << CLK_CLKSTATUS_OSC10K_STB_Pos)/*!< CLK_T::CLKSTATUS: OSC10K_STB Mask */
#define CLK_CLKSTATUS_IRC10K_STB_Pos         3                                    /*!< CLK_T::CLKSTATUS: IRC10K_STB Position */
#define CLK_CLKSTATUS_IRC10K_STB_Msk         (1ul << CLK_CLKSTATUS_IRC10K_STB_Pos)/*!< CLK_T::CLKSTATUS: IRC10K_STB Mask */

#define CLK_CLKSTATUS_PLL_STB_Pos            2                                    /*!< CLK_T::CLKSTATUS: PLL_STB Position */
#define CLK_CLKSTATUS_PLL_STB_Msk            (1ul << CLK_CLKSTATUS_PLL_STB_Pos)   /*!< CLK_T::CLKSTATUS: PLL_STB Mask */

#define CLK_CLKSTATUS_XTL12M_STB_Pos         0                                    /*!< CLK_T::CLKSTATUS: XTL12M_STB Position */
#define CLK_CLKSTATUS_XTL12M_STB_Msk         (1ul << CLK_CLKSTATUS_XTL12M_STB_Pos)/*!< CLK_T::CLKSTATUS: XTL12M_STB Mask */

/* CLK CLKSEL0 Bit Field Definitions */
#define CLK_CLKSEL0_STCLK_S_Pos              3                                    /*!< CLK_T::CLKSEL0: STCLK_S Position */
#define CLK_CLKSEL0_STCLK_S_Msk              (7ul << CLK_CLKSEL0_STCLK_S_Pos)     /*!< CLK_T::CLKSEL0: STCLK_S Mask */

#define CLK_CLKSEL0_HCLK_S_Pos               0                                    /*!< CLK_T::CLKSEL0: HCLK_S Position */
#define CLK_CLKSEL0_HCLK_S_Msk               (7ul << CLK_CLKSEL0_HCLK_S_Pos)      /*!< CLK_T::CLKSEL0: HCLK_S Mask */

/* CLK CLKSEL1 Bit Field Definitions */
#define CLK_CLKSEL1_PWM23_S_Pos              30                                   /*!< CLK_T::CLKSEL1: PWM23_S Position */
#define CLK_CLKSEL1_PWM23_S_Msk              (3ul << CLK_CLKSEL1_PWM23_S_Pos)     /*!< CLK_T::CLKSEL1: PWM23_S Mask */

#define CLK_CLKSEL1_PWM01_S_Pos              28                                   /*!< CLK_T::CLKSEL1: PWM01_S Position */
#define CLK_CLKSEL1_PWM01_S_Msk              (3ul << CLK_CLKSEL1_PWM01_S_Pos)     /*!< CLK_T::CLKSEL1: PWM01_S Mask */

#define CLK_CLKSEL1_UART_S_Pos               24                                   /*!< CLK_T::CLKSEL1: UART_S Position */
#define CLK_CLKSEL1_UART_S_Msk               (3ul << CLK_CLKSEL1_UART_S_Pos)      /*!< CLK_T::CLKSEL1: UART_S Mask */

#define CLK_CLKSEL1_TMR3_S_Pos               20                                   /*!< CLK_T::CLKSEL1: TMR3_S Position */
#define CLK_CLKSEL1_TMR3_S_Msk               (7ul << CLK_CLKSEL1_TMR3_S_Pos)      /*!< CLK_T::CLKSEL1: TMR3_S Mask */

#define CLK_CLKSEL1_TMR2_S_Pos               16                                   /*!< CLK_T::CLKSEL1: TMR2_S Position */
#define CLK_CLKSEL1_TMR2_S_Msk               (7ul << CLK_CLKSEL1_TMR2_S_Pos)      /*!< CLK_T::CLKSEL1: TMR2_S Mask */

#define CLK_CLKSEL1_TMR1_S_Pos               12                                   /*!< CLK_T::CLKSEL1: TMR1_S Position */
#define CLK_CLKSEL1_TMR1_S_Msk               (7ul << CLK_CLKSEL1_TMR1_S_Pos)      /*!< CLK_T::CLKSEL1: TMR1_S Mask */

#define CLK_CLKSEL1_TMR0_S_Pos               8                                    /*!< CLK_T::CLKSEL1: TMR0_S Position */
#define CLK_CLKSEL1_TMR0_S_Msk               (7ul << CLK_CLKSEL1_TMR0_S_Pos)      /*!< CLK_T::CLKSEL1: TMR0_S Mask */

#define CLK_CLKSEL1_SPI0_S_Pos               4                                    /*!< CLK_T::CLKSEL1: SPI0_S Position */
#define CLK_CLKSEL1_SPI0_S_Msk               (1ul << CLK_CLKSEL1_SPI0_S_Pos)      /*!< CLK_T::CLKSEL1: SPI0_S Mask */

#define CLK_CLKSEL1_ADC_S_Pos                2                                    /*!< CLK_T::CLKSEL1: ADC_S Position */
#define CLK_CLKSEL1_ADC_S_Msk                (3ul << CLK_CLKSEL1_ADC_S_Pos)       /*!< CLK_T::CLKSEL1: ADC_S Mask */

#define CLK_CLKSEL1_WDT_S_Pos                0                                    /*!< CLK_T::CLKSEL1: WDT_S Position */
#define CLK_CLKSEL1_WDT_S_Msk                (3ul << CLK_CLKSEL1_WDT_S_Pos)       /*!< CLK_T::CLKSEL1: WDT_S Mask */

/* CLK CLKSEL2 Bit Field Definitions */
#define CLK_CLKSEL2_WWDT_S_Pos               16                                   /*!< CLK_T::CLKSEL2: WWDT_S Position */
#define CLK_CLKSEL2_WWDT_S_Msk               (3ul << CLK_CLKSEL2_WWDT_S_Pos)      /*!< CLK_T::CLKSEL2: WWDT_S Mask */

#define CLK_CLKSEL2_FRQDIV_S_Pos             2                                    /*!< CLK_T::CLKSEL2: FRQDIV_S Position */
#define CLK_CLKSEL2_FRQDIV_S_Msk             (3ul << CLK_CLKSEL2_FRQDIV_S_Pos)    /*!< CLK_T::CLKSEL2: FRQDIV_S Mask */

/* CLK CLKDIV Bit Field Definitions */
#define CLK_CLKDIV_ADC_N_Pos                 16                                   /*!< CLK_T::CLKDIV: ADC_N Position */
#define CLK_CLKDIV_ADC_N_Msk                 (0xFFul << CLK_CLKDIV_ADC_N_Pos)     /*!< CLK_T::CLKDIV: ADC_N Mask */

#define CLK_CLKDIV_UART_N_Pos                8                                    /*!< CLK_T::CLKDIV: UART_N Position */
#define CLK_CLKDIV_UART_N_Msk                (0xFul << CLK_CLKDIV_UART_N_Pos)     /*!< CLK_T::CLKDIV: UART_N Mask */

#define CLK_CLKDIV_HCLK_N_Pos                0                                    /*!< CLK_T::CLKDIV: HCLK_N Position */
#define CLK_CLKDIV_HCLK_N_Msk                (0xFul << CLK_CLKDIV_HCLK_N_Pos)     /*!< CLK_T::CLKDIV: HCLK_N Mask */

/* CLK PLLCON Bit Field Definitions */
#define CLK_PLLCON_PLL_SRC_Pos               19                                      /*!< CLK_T::PLLCON: PLL_SRC Position */
#define CLK_PLLCON_PLL_SRC_Msk               (1ul << CLK_PLLCON_PLL_SRC_Pos)      /*!< CLK_T::PLLCON: PLL_SRC Mask */

#define CLK_PLLCON_OE_Pos                    18                                   /*!< CLK_T::PLLCON: OE Position */
#define CLK_PLLCON_OE_Msk                    (1ul << CLK_PLLCON_OE_Pos)           /*!< CLK_T::PLLCON: OE Mask */

#define CLK_PLLCON_BP_Pos                    17                                   /*!< CLK_T::PLLCON: BP Position */
#define CLK_PLLCON_BP_Msk                    (1ul << CLK_PLLCON_BP_Pos)           /*!< CLK_T::PLLCON: BP Mask */

#define CLK_PLLCON_PD_Pos                    16                                   /*!< CLK_T::PLLCON: PD Position */
#define CLK_PLLCON_PD_Msk                    (1ul << CLK_PLLCON_PD_Pos)           /*!< CLK_T::PLLCON: PD Mask */

#define CLK_PLLCON_OUT_DV_Pos                14                                   /*!< CLK_T::PLLCON: OUT_DV Position */
#define CLK_PLLCON_OUT_DV_Msk                (3ul << CLK_PLLCON_OUT_DV_Pos)       /*!< CLK_T::PLLCON: OUT_DV Mask */

#define CLK_PLLCON_IN_DV_Pos                 9                                    /*!< CLK_T::PLLCON: IN_DV Position */
#define CLK_PLLCON_IN_DV_Msk                 (0x1Ful << CLK_PLLCON_IN_DV_Pos)     /*!< CLK_T::PLLCON: IN_DV Mask */

#define CLK_PLLCON_FB_DV_Pos                 0                                    /*!< CLK_T::PLLCON: FB_DV Position */
#define CLK_PLLCON_FB_DV_Msk                 (0x1FFul << CLK_PLLCON_FB_DV_Pos)    /*!< CLK_T::PLLCON: FB_DV Mask */

/* CLK FRQDIV Bit Field Definitions */
#define CLK_FRQDIV_DIVIDER_EN_Pos            4                                    /*!< CLK_T::FRQDIV: DIVIDER_EN Position */
#define CLK_FRQDIV_DIVIDER_EN_Msk            (1ul << CLK_FRQDIV_DIVIDER_EN_Pos)   /*!< CLK_T::FRQDIV: DIVIDER_EN Mask */

#define CLK_FRQDIV_FSEL_Pos                  0                                    /*!< CLK_T::FRQDIV: FSEL Position */
#define CLK_FRQDIV_FSEL_Msk                  (0xFul << CLK_FRQDIV_FSEL_Pos)       /*!< CLK_T::FRQDIV: FSEL Mask */
/**@}*/ /* CLK_CONST */
/**@}*/ /* CLK */





/*----------------------------- TIMER Controller -----------------------------*/
/** @addtogroup TIMER TIMER Controller(TIMER)
  Memory Mapped Structure for Timer Controller
  @{
 */

typedef struct
{


/**
 * @var TIMER_T::TCSR
 * Offset: 0x00  Timer Control Register
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
 * |[19]    |INTR_TRG_EN|Inter-Timer Trigger Mode Enable Control
 * |        |          |Setting this bit will enable the inter-timer trigger capture function.
 * |        |          |The Timer0/2 will be in counter mode and counting with external clock source or event.
 * |        |          |Also, Timer1/3 will be in trigger-counting mode of capture function.
 * |        |          |0 = Inter-Timer Trigger Capture mode Disabled.
 * |        |          |1 = Inter-Timer Trigger Capture mode Enabled.
 * |        |          |Note: For Timer1/3, this bit is ignored and the read back value is always 0.
 * |[20]    |PERIODIC_SEL|Periodic Mode Behavior Selection Enable
 * |        |          |0 = The behavior selection in periodic mode is Disabled.
 * |        |          |When user updates TCMP while timer is running in periodic mode, TDR will be reset to default value.
 * |        |          |1 = The behavior selection in periodic mode is Enabled.
 * |        |          |When user update TCMP while timer is running in periodic mode, the limitations as bellows list,
 * |        |          |If updated TCMP value > TDR, TCMP will be updated and TDR keep running continually.
 * |        |          |If updated TCMP value = TDR, timer time-out interrupt will be asserted immediately.
 * |        |          |If updated TCMP value < TDR, TDR will be reset to default value.
 * |[21]    |TOUT_SEL  |Toggle-output Pin Selection
 * |        |          |0 = Toggle-output pin is from Tx pin.
 * |        |          |1 = Toggle-output pin is from TxEx pin.
 * |[23]    |WAKE_EN   |Wake Up Function Enable Control
 * |        |          |0 = Wake-up trigger event Disabled.
 * |        |          |1 = Wake-up trigger event Enabled.
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
 * |[29]    |IE        |Interrupt Enable Bit
 * |        |          |0 = Timer Interrupt function Disabled.
 * |        |          |1 = Timer Interrupt function Enabled.
 * |        |          |If this bit is enabled, when the timer interrupt flag (TISR[0] TIF) is set to 1, the timer interrupt signal is generated and inform to CPU.
 * |[30]    |CEN       |Timer Enable Bit
 * |        |          |0 = Stops/Suspends counting.
 * |        |          |1 = Starts counting.
 * |        |          |Note1: In stop status, and then set CEN to 1 will enable the 24-bit up counter to keep counting from the last stop counting value.
 * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (TCSR [28:27] = 00) when the timer interrupt flag (TISR[0] TIF) is generated.
 * |[31]    |DBGACK_TMR|ICE Debug Mode Acknowledge Disable (Write Protect)
 * |        |          |0 = ICE debug mode acknowledgment effects TIMER counting.
 * |        |          |TIMER counter will be held while CPU is held by ICE.
 * |        |          |1 = ICE debug mode acknowledgment Disabled.
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
 * |        |          |Note2: When timer is operating at continuous counting mode, the 24-bit up counter will keep counting continuously even if user writes a new value into TCMP field.
 * |        |          |But if timer is operating at other modes, the 24-bit up counter will restart counting and using newest TCMP value to be the timer compared value if user writes a new value into TCMP field.
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
 * |        |          |If TDR_EN (TCSR[16]) is set to 1, TDR register will be updated continuously to monitor 24-bit up counter value.
 * @var TIMER_T::TCAP
 * Offset: 0x10  Timer Capture Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |TCAP      |Timer Capture Data Register
 * |        |          |When TEXIF flag is set to 1, the current TDR value will be auto-loaded into this TCAP filed immediately.
 * @var TIMER_T::TEXCON
 * Offset: 0x14  Timer External Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TX_PHASE  |Timer External Count Pin Phase Detect Selection
 * |        |          |This bit indicates the detection phase of Tx pin.
 * |        |          |0 = A falling edge of Tx pin will be counted.
 * |        |          |1 = A rising edge of Tx pin will be counted.
 * |[2:1]   |TEX_EDGE  |Timer External Capture Pin Edge Detect Selection
 * |        |          |00 = A 1 to 0 transition on TxEX pin will be detected.
 * |        |          |01 = A 0 to 1 transition on TxEX pin will be detected.
 * |        |          |10 = Either 1 to 0 or 0 to 1 transition on TxEX pin will be detected.
 * |        |          |11 = Reserved.
 * |[3]     |TEXEN     |Timer External Pin Function Enable
 * |        |          |This bit enables the RSTCAPSEL function on the TxEX pin.
 * |        |          |0 = RSTCAPSEL function of TxEX pin will be ignored.
 * |        |          |1 = RSTCAPSEL function of TxEX pin is active.
 * |[4]     |RSTCAPSEL |Timer External Reset Counter / Timer External Capture Mode Selection
 * |        |          |0 = Transition on TxEX pin is using to save the TDR value into TCAP value if TEXIF flag is set to 1.
 * |        |          |1 = Transition on TxEX pin is using to reset the 24-bit up counter.
 * |[5]     |TEXIEN    |Timer External Capture Interrupt Enable Control
 * |        |          |0 = TxEX pin detection Interrupt Disabled.
 * |        |          |1 = TxEX pin detection Interrupt Enabled.
 * |        |          |If TEXIEN enabled, Timer will raise an external capture interrupt signal and inform to CPU while TEXIF flag is set to 1.
 * |[6]     |TEXDB     |Timer External Capture Input Pin De-Bounce Enable Control
 * |        |          |0 = TxEX pin de-bounce Disabled.
 * |        |          |1 = TxEX pin de-bounce Enabled.
 * |        |          |If this bit is enabled, the edge detection of TxEX pin is detected with de-bounce circuit.
 * |[7]     |TCDB      |Timer External Counter Input Pin De-Bounce Enable Control
 * |        |          |0 = Tx pin de-bounce Disabled.
 * |        |          |1 = Tx pin de-bounce Enabled.
 * |        |          |If this bit is enabled, the edge detection of Tx pin is detected with de-bounce circuit.
 * @var TIMER_T::TEXISR
 * Offset: 0x18  Timer External Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TEXIF     |Timer External Capture Interrupt Flag
 * |        |          |This bit indicates the external capture interrupt flag status.
 * |        |          |When TEXEN enabled, TxEX pin selected as external capture function, and a transition on TxEX pin matched the TEX_EDGE setting, this flag will set to 1 by hardware.
 * |        |          |0 = TxEX pin interrupt did not occur.
 * |        |          |1 = TxEX pin interrupt occurred.
 * |        |          |Note: This bit is cleared by writing 1 to it.
 */

    __IO uint32_t TCSR;          /* Offset: 0x00  Timer Control Register                                             */
    __IO uint32_t TCMPR;         /* Offset: 0x04  Timer Compare Register                                             */
    __IO uint32_t TISR;          /* Offset: 0x08  Timer Interrupt Status Register                                    */
    __IO uint32_t TDR;           /* Offset: 0x0C  Timer Data Register                                                */
    __IO uint32_t TCAP;          /* Offset: 0x10  Timer Capture Data Register                                        */
    __IO uint32_t TEXCON;        /* Offset: 0x14  Timer External Control Register                                    */
    __IO uint32_t TEXISR;        /* Offset: 0x18  Timer External Interrupt Status Register                           */

} TIMER_T;



/** @addtogroup TMR_CONST Timer Bit Field Definition
  Constant Definitions for Timer Controller
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

#define TIMER_TCSR_WAKE_EN_Pos      23                                          /*!< TIMER_T::TCSR: WAKE_EN Position */
#define TIMER_TCSR_WAKE_EN_Msk      (1ul << TIMER_TCSR_WAKE_EN_Pos)             /*!< TIMER_T::TCSR: WAKE_EN Mask */

#define TIMER_TCSR_TOUT_SEL_Pos     21                                          /*!< TIMER_T::TCSR: TOUT_SEL Position */
#define TIMER_TCSR_TOUT_SEL_Msk     (1ul << TIMER_TCSR_TOUT_SEL_Pos)            /*!< TIMER_T::TCSR: TOUT_SEL Mask */

#define TIMER_TCSR_ERIODIC_SEL_Pos  20                                          /*!< TIMER_T::TCSR: PERIODIC_SEL Position */
#define TIMER_TCSR_ERIODIC_SEL_Msk  (1ul << TIMER_TCSR_ERIODIC_SEL_Pos)         /*!< TIMER_T::TCSR: PERIODIC_SEL Mask */

#define TIMER_TCSR_INTR_TRG_EN_Pos  19                                          /*!< TIMER_T::TCSR: INTR_TRG_EN Position */
#define TIMER_TCSR_INTR_TRG_EN_Msk  (1ul << TIMER_TCSR_INTR_TRG_EN_Pos)         /*!< TIMER_T::TCSR: INTR_TRG_EN Mask */

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
/**@}*/ /* TMR_CONST */
/**@}*/ /* TIMER */


/*------------------------- UART Interface Controller ------------------------*/

/** @addtogroup UART UART Controller(UART)
  Memory Mapped Structure for UART Serial Interface Controller
  @{
 */

typedef struct
{



/**
 * @var UART_T::DATA
 * Offset: 0x00 UART Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |THR       |UART Data Register
 * |        |          |By writing to this register, the UART will send out an 8-bit data through the Tx pin (LSB first).
 * |        |          |By reading this register, the UART will return the 8-bit data received from Rx pin (LSB first).
 * @var UART_T::THR
 * Offset: 0x00 UART Transmit Holding Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |THR       |Transmit Holding Register
 * |        |          |By writing to this register, the UART will send out an 8-bit data through the Tx pin (LSB first).
 * @var UART_T::RBR
 * Offset: 0x00  UART Receive Buffer Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |RBR       |Receive Buffer Register (Read Only)
 * |        |          |By reading this register, the UART will return the 8-bit data received from Rx pin (LSB first).
 * @var UART_T::IER
 * Offset: 0x04  UART Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RDA_IEN   |Receive Data Available Interrupt Enable
 * |        |          |0 = Receive data available interrupt Disabled.
 * |        |          |1 = Receive data available interrupt Enabled.
 * |[1]     |THRE_IEN  |Transmit Holding Register Empty Interrupt Enable
 * |        |          |0 = Transmit holding register empty interrupt Disabled.
 * |        |          |1 = Transmit holding register empty interrupt Enabled.
 * |[2]     |RLS_IEN   |Receive Line Status Interrupt Enable
 * |        |          |0 = Receive Line Status interrupt Disabled.
 * |        |          |1 = Receive Line Status interrupt Enabled.
 * |[3]     |MODEM_IEN |Modem Status Interrupt Enable
 * |        |          |0 = Modem status interrupt Disabled.
 * |        |          |1 = Modem status interrupt Enabled.
 * |[4]     |RTO_IEN   |RX Time-out Interrupt Enable
 * |        |          |0 = RX time-out interrupt Disabled.
 * |        |          |1 = RX time-out interrupt Enabled.
 * |[5]     |BUF_ERR_IEN|Buffer Error Interrupt Enable
 * |        |          |0 = Buffer error interrupt Disabled.
 * |        |          |1 = Buffer error interrupt Enabled.
 * |[6]     |WAKE_EN   |UART Wake-up Function Enable
 * |        |          |0 = UART wake-up function Disabled.
 * |        |          |1 = UART wake-up function Enabled, when chip is in Power-down mode, an external CTS change will wake up chip from Power-down mode.
 * |[8]     |LIN_RX_BRK_IEN|LIN RX Break Field Detected Interrupt Enable Control
 * |        |          |0 = LIN bus RX break filed interrupt Disabled.
 * |        |          |1 = LIN bus RX break filed interrupt Enabled.
 * |        |          |Note: This bit is used for LIN function mode.
 * |[11]    |TIME_OUT_EN|Time-out Counter Enable
 * |        |          |0 = Time-out counter Disabled.
 * |        |          |1 = Time-out counter Enabled.
 * |[12]    |AUTO_RTS_EN|RTS Auto Flow Control Enable
 * |        |          |0 = RTS auto flow control Disabled.
 * |        |          |1 = RTS auto flow control Enabled.
 * |        |          |Note: When RTS auto-flow is enabled, if the number of bytes in the RX FIFO is equal to the RTS_TRI_LEV (UA_FCR [19:16]), the UART will de-assert RTS signal.
 * |[13]    |AUTO_CTS_EN|CTS Auto Flow Control Enable
 * |        |          |0 = CTS auto flow control Disabled.
 * |        |          |1 = CTS auto flow control Enabled.
 * |        |          |Note: When CTS auto-flow is enabled, the UART will send data to external device when CTS input assert (UART will not send data to device until CTS is asserted).
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
 * |        |          |When the number of bytes in the receive FIFO equals the RFITL, the RDA_IF will be set (if RDA_IEN(UA_IER[0] enabled, and an interrupt will be generated).
 * |        |          |0000 = RX FIFO Interrupt Trigger Level is 1 byte.
 * |        |          |0001 = RX FIFO Interrupt Trigger Level is 4 bytes.
 * |        |          |0010 = RX FIFO Interrupt Trigger Level is 8 bytes.
 * |        |          |0011 = RX FIFO Interrupt Trigger Level is 14 bytes.
 * |[8]     |RX_DIS    |Receiver Disable
 * |        |          |The receiver is disabled or not (set 1 to disable receiver)
 * |        |          |0 = Receiver Enabled.
 * |        |          |1 = Receiver Disabled.
 * |        |          |Note: This field is used for RS-485 Normal Multi-drop mode. It should be programmed before RS485_NMM(UA_ALT_CSR [8]) is programmed.
 * |[19:16] |RTS_TRI_LEV|RTS Trigger Level For Auto-Flow Control Use
 * |        |          |0000 = RTS Trigger Level is 1 byte.
 * |        |          |0001 = RTS Trigger Level is 4 bytes.
 * |        |          |0010 = RTS Trigger Level is 8 bytes.
 * |        |          |0011 = RTS Trigger Level is 14 bytes.
 * |        |          |Note: This field is used for RTS auto-flow control.
 * @var UART_T::LCR
 * Offset: 0x0C  UART Line Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |WLS       |Word Length Select
 * |        |          |This field sets UART word length.
 * |        |          |00 = 5 bits.
 * |        |          |01 = 6 bits.
 * |        |          |10 = 7 bits.
 * |        |          |11 = 8 bits.
 * |[2]     |NSB       |Number of "STOP bit"
 * |        |          |0 = One "STOP bit" is generated in the transmitted data.
 * |        |          |1 = When select 5-bit word length, 1.5 "STOP bit" is generated in the transmitted data. When select 6-, 7- and 8-bit word length, 2 "STOP bit" is generated in the transmitted data.
 * |[3]     |PBE       |Parity Bit Enable
 * |        |          |0 = No parity bit.
 * |        |          |1 = Parity bit generated Enabled.
 * |        |          |Note : Parity bit is generated on each outgoing character and is checked on each incoming data.
 * |[4]     |EPE       |Even Parity Enable
 * |        |          |0 = Odd number of logic 1's is transmitted and checked in each word.
 * |        |          |1 = Even number of logic 1's is transmitted and checked in each word.
 * |        |          |Note: This bit has effect only when PBE (UA_LCR[3]) is set.
 * |[5]     |SPE       |Stick Parity Enable
 * |        |          |0 = Stick parity Disabled.
 * |        |          |1 = Stick parity Enabled.
 * |        |          |Note: If PBE (UA_LCR[3]) and EPE (UA_LCR[4]) are logic 1, the parity bit is transmitted and checked as logic 0. If PBE (UA_LCR[3]) is 1 and EPE (UA_LCR[4]) is 0 then the parity bit is transmitted and checked as 1.
 * |[6]     |BCB       |Break Control Bit
 * |        |          |0 = Break Control Disabled.
 * |        |          |1 = Break Control Enabled.
 * |        |          |Note: When this bit is set to logic 1, the serial data output (TX) is forced to the Spacing State (logic 0). This bit acts only on TX and has no effect on the transmitter logic.
 * @var UART_T::MCR
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
 * @var UART_T::MSR
 * Offset: 0x14  UART Modem Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |DCTSF     |Detect CTS State Change Flag (Read Only)
 * |        |          |This bit is set whenever CTS input has change state, and it will generate Modem interrupt to CPU when MODEM_IEN (UA_IER [3]) is set to 1.
 * |        |          |0 = CTS input has not change state.
 * |        |          |1 = CTS input has change state.
 * |        |          |Note: This bit is read only, but can be cleared by writing "1" to it.
 * |[4]     |CTS_ST    |CTS Pin Status (Read Only)
 * |        |          |This bit mirror from CTS pin input of voltage logic status.
 * |        |          |0 = CTS pin input is low level voltage logic state.
 * |        |          |1 = CTS pin input is high level voltage logic state.
 * |        |          |Note: This bit echoes when UART Controller peripheral clock is enabled, and CTS multi-function port is selected
 * |[8]     |LEV_CTS   |CTS Pin Active Level
 * |        |          |This bit defines the active level state of CTS pin input.
 * |        |          |0 = CTS pin input is high level active.
 * |        |          |1 = CTS pin input is low level active.
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
 * |        |          |Note1: This field is used for RS-485 function mode and RS485_ADD_EN (UA_ALT_CSR[15]) is set to 1 to enable Address detection mode .
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
 * |[13:8]  |RX_POINTER|RX FIFO Pointer
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
 * |        |          |When the number of bytes in the RX FIFO equals the RFITL then the RDA_IF(UA_ISR[0]) will be set.
 * |        |          |If RDA_IEN (UA_IER [0]) is enabled, the RDA interrupt will be generated.
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
 * |        |          |If RLS_IEN (UA_IER [2]) is enabled, the RLS interrupt will be generated.
 * |        |          |0 = No RLS interrupt flag is generated.
 * |        |          |1 = RLS interrupt flag is generated.
 * |        |          |Note1: In RS-485 function mode, this field is set include receiver detect and received address byte character (bit9 = '1') bit.
 * |        |          |At the same time, the bit of UA_FSR[RS485_ADD_DETF] is also set.
 * |        |          |Note2: This bit is read only and reset to 0 when all bits of BIF(UA_FSR[6]), FEF(UA_FSR[5]) and PEF(UA_FSR[4]) are cleared.
 * |        |          |Note3: In RS-485 function mode, this bit is read only and reset to 0 when all bits of BIF(UA_FSR[6]) , FEF(UA_FSR[5]) and PEF(UA_FSR[4]) and RS485_ADD_DETF (UA_FSR[3]) are cleared.
 * |[3]     |MODEM_IF  |MODEM Interrupt Flag (Read Only)
 * |        |          |This bit is set when the CTS pin has state change (DCTSF (UA_MSR[0]) = 1).
 * |        |          |If MODEM_IEN (UA_IER [3]) is enabled, the Modem interrupt will be generated.
 * |        |          |0 = No Modem interrupt flag is generated.
 * |        |          |1 = Modem interrupt flag is generated.
 * |        |          |Note: This bit is read only and reset to 0 when bit DCTSF is cleared by a write 1 on DCTSF(UA_MSR[0]).
 * |[4]     |TOUT_IF   |Time-Out Interrupt Flag (Read Only)
 * |        |          |This bit is set when the RX FIFO is not empty and no activities occurred in the RX FIFO and the time-out counter equal to TOIC.
 * |        |          |If TOUT_IEN (UA_IER [4]) is enabled, the Tout interrupt will be generated.
 * |        |          |0 = No Time-out interrupt flag is generated.
 * |        |          |1 = Time-out interrupt flag is generated.
 * |        |          |Note: This bit is read only and user can read UA_RBR (RX is in active) to clear it
 * |[5]     |BUF_ERR_IF|Buffer Error Interrupt Flag (Read Only)
 * |        |          |This bit is set when the TX FIFO or RX FIFO overflows (TX_OVER_IF (UA_FSR[24]) or RX_OVER_IF (UA_FSR[0]) is set).
 * |        |          |When BUF_ERR_IF (UA_ISR[5])is set, the transfer is not correct.
 * |        |          |If BUF_ERR_IEN (UA_IER [8]) is enabled, the buffer error interrupt will be generated.
 * |        |          |0 = No buffer error interrupt flag is generated.
 * |        |          |1 = Buffer error interrupt flag is generated.0 = No buffer error interrupt flag is generated.
 * |        |          |1 = Buffer error interrupt flag is generated.
 * |        |          |Note: This bit is read only and reset to 0 when all bits of TX_OVER_IF(UA_FSR[24]) and RX_OVER_IF(UA_FSR[0]) are cleared
 * |[7]     |LIN_RX_BREAK_IF|LIN Bus RX Break Field Detected Flag
 * |        |          |This bit is set when RX received LIN Break Field. If LIN_RX_BRK_IEN(UA_IER[8]) is enabled the LIN RX Break interrupt will be generated.
 * |        |          |0 = No LIN RX Break received.
 * |        |          |1 = LIN RX Break received.
 * |        |          |Note: This bit is cleared by writing 1 to it.
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
 * |[11]    |MODEM_INT |MODEM Status Interrupt Indicator (Read Only)
 * |        |          |This bit is set if MODEM_IEN(UA_IER[3] and MODEM_IF(UA_ISR[4]) are both set to 1
 * |        |          |0 = No Modem interrupt is generated.
 * |        |          |1 = Modem interrupt is generated.
 * |[12]    |TOUT_INT  |Time-Out Interrupt Indicator (Read Only)
 * |        |          |This bit is set if TOUT_IEN(UA_IER[4]) and TOUT_IF(UA_ISR[4]) are both set to 1.
 * |        |          |0 = No Tout interrupt is generated.
 * |        |          |1 = Tout interrupt is generated.
 * |[13]    |BUF_ERR_INT|Buffer Error Interrupt Indicator (Read Only)
 * |        |          |This bit is set if BUF_ERR_IEN(UA_IER[5] and BUF_ERR_IF(UA_ISR[5]) are both set to 1.
 * |        |          |0 = No buffer error interrupt is generated.
 * |        |          |1 = Buffer error interrupt is generated.
 * |[15]    |LIN_RX_BREAK_INT|LIN Bus RX Break Field Detected Interrupt Indicator (Read Only)
 * |        |          |This bit is set if LIN_RX_BRK_IEN(UA_IER[8]) and LIN_RX_BRK_IFLIN_IF(UA_ISR[7]) are both set to 1.
 * |        |          |0 = No LIN RX Break interrupt is generated.
 * |        |          |1 = LIN RX Break interrupt is generated.
 * @var UART_T::TOR
 * Offset: 0x20  UART Time-out Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |TOIC      |Time-Out Interrupt Comparator
 * |        |          |The time-out counter resets and starts counting (the counting clock = baud rate) whenever the RX FIFO receives a new data word.
 * |        |          |Once the content of time-out counter is equal to that of time-out interrupt comparator (TOIC (UA_TOR[7:0])), a receiver time-out interrupt (INT_TOUT) is generated if TOUT_IEN (UA_IER [4]) enabled.
 * |        |          |A new incoming data word or RX FIFO empty will clear TOUT_INT(UA_IER[9]).
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
 * |[28]    |DIV_X_ONE |Divider X Equal To 1
 * |        |          |0 = Divider M is X+1 (the equation of M = X+1, but DIVIDER_X[27:24] must >= 8).
 * |        |          |1 = Divider M is 1 (the equation of M = 1, but BRD [15:0] must >= 3).
 * |[29]    |DIV_X_EN  |Divider X Enable
 * |        |          |The BRD = Baud Rate Divider, and the baud rate equation is
 * |        |          |Baud Rate = Clock / [M * (BRD + 2)]; The default value of M is 16.
 * |        |          |0 = Divider X Disabled (the equation of M = 16).
 * |        |          |1 = Divider X Enabled (the equation of M = X+1, but DIVIDER_X [27:24] must >= 8).
 * |        |          |Note: In IrDA mode, this bit must disable.
 * @var UART_T::IRCR
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
 * @var UART_T::ALT_CSR
 * Offset: 0x2C  UART Alternate Control/Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |LIN_BKFL  |UART LIN Break Field Length
 * |        |          |This field indicates a 4-bit LIN TX break field count.
 * |        |          |Note1: This break field length is UA_LIN_BKFL + 1
 * |        |          |Note2: According to LIN spec, the reset value is 0xC (break field length = 13).
 * |[6]     |LIN_RX_EN |LIN RX Enable
 * |        |          |0 = LIN RX mode Disabled.
 * |        |          |1 = LIN RX mode Enabled.
 * |[7]     |LIN_TX_EN |LIN TX Break Mode Enable
 * |        |          |0 = LIN TX Break mode Disabled.
 * |        |          |1 = LIN TX Break mode Enabled.
 * |        |          |Note: When TX break field transfer operation finished, this bit will be cleared automatically.
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
 * |[31:24] |ADDR_MATCH|Address Match Value
 * |        |          |This field contains the RS-485 address match values.
 * |        |          |Note: This field is used for RS-485 auto address detection mode.
 * @var UART_T::FUN_SEL
 * Offset: 0x30  UART Function Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |FUN_SEL   |Function Select Enable
 * |        |          |00 = UART Function.
 * |        |          |01 = LIN Function.
 * |        |          |10 = IrDA Function.
 * |        |          |11 = RS-485 Function.
 */

    union {
    __IO uint32_t DATA;          /* Offset: 0x00 UART Data Register                                                  */
    __IO uint32_t THR;           /* Offset: 0x00 UART Transmit Holding Register                                      */
    __IO uint32_t RBR;           /* Offset: 0x00 UART Receive Buffer Register                                        */
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

} UART_T;



/** @addtogroup UART_CONST UART Bit Field Definition
  Constant Definitions for UART Controller
  @{
 */

/* UART THR Bit Field Definitions */
#define UART_THR_THR_Pos            0                                       /*!< UART_T::THR: THR Position  */
#define UART_THR_THR_Msk            (0xFul << UART_THR_THR_Pos)             /*!< UART_T::THR: THR Mask      */

/* UART RBR Bit Field Definitions */
#define UART_RBR_RBR_Pos            0                                       /*!< UART_T::RBR: RBR Position */
#define UART_RBR_RBR_Msk            (0xFul << UART_RBR_RBR_Pos)             /*!< UART_T::RBR: RBR Mask      */

/* UART IER Bit Field Definitions */
#define UART_IER_AUTO_CTS_EN_Pos    13                                      /*!< UART_T::IER: AUTO_CTS_EN Position      */
#define UART_IER_AUTO_CTS_EN_Msk    (1ul << UART_IER_AUTO_CTS_EN_Pos)       /*!< UART_T::IER: AUTO_CTS_EN Mask           */

#define UART_IER_AUTO_RTS_EN_Pos    12                                      /*!< UART_T::IER: AUTO_RTS_EN Position      */
#define UART_IER_AUTO_RTS_EN_Msk    (1ul << UART_IER_AUTO_RTS_EN_Pos)       /*!< UART_T::IER: AUTO_RTS_EN Mask           */

#define UART_IER_TIME_OUT_EN_Pos    11                                      /*!< UART_T::IER: TIME_OUT_EN Position      */
#define UART_IER_TIME_OUT_EN_Msk    (1ul << UART_IER_TIME_OUT_EN_Pos)       /*!< UART_T::IER: TIME_OUT_EN Mask           */

#define UART_IER_LIN_RX_BRK_IEN_Pos 8                                       /*!< UART_T::IER: LIN_RX_BRK_IEN Position   */
#define UART_IER_LIN_RX_BRK_IEN_Msk (1ul << UART_IER_LIN_RX_BRK_IEN_Pos)    /*!< UART_T::IER: LIN_RX_BRK_IEN Mask        */

#define UART_IER_WAKE_EN_Pos        6                                       /*!< UART_T::IER: WAKE_EN Position          */
#define UART_IER_WAKE_EN_Msk        (1ul << UART_IER_WAKE_EN_Pos)           /*!< UART_T::IER: WAKE_EN Mask               */

#define UART_IER_BUF_ERR_IEN_Pos    5                                       /*!< UART_T::IER: BUF_ERR_IEN Position      */
#define UART_IER_BUF_ERR_IEN_Msk    (1ul << UART_IER_BUF_ERR_IEN_Pos)       /*!< UART_T::IER: BUF_ERR_IEN Mask           */

#define UART_IER_RTO_IEN_Pos        4                                       /*!< UART_T::IER: RTO_IEN Position          */
#define UART_IER_RTO_IEN_Msk        (1ul << UART_IER_RTO_IEN_Pos)           /*!< UART_T::IER: RTO_IEN Mask               */

#define UART_IER_MODEM_IEN_Pos      3                                       /*!< UART_T::IER: MODEM_IEN Position        */
#define UART_IER_MODEM_IEN_Msk      (1ul << UART_IER_MODEM_IEN_Pos)         /*!< UART_T::IER: MODEM_IEN Mask             */

#define UART_IER_RLS_IEN_Pos        2                                       /*!< UART_T::IER: RLS_IEN Position          */
#define UART_IER_RLS_IEN_Msk        (1ul << UART_IER_RLS_IEN_Pos)           /*!< UART_T::IER: RLS_IEN Mask               */

#define UART_IER_THRE_IEN_Pos       1                                       /*!< UART_T::IER: THRE_IEN Position         */
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
#define UART_ISR_LIN_RX_BREAK_INT_Pos    15                                      /*!< UART_T::ISR: LIN_RX_BREAK_INT Position       */
#define UART_ISR_LIN_RX_BREAK_INT_Msk    (1ul << UART_ISR_LIN_RX_BREAK_INT_Pos)  /*!< UART_T::ISR: LIN_RX_BREAK_INT Mask           */

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

#define UART_ISR_LIN_RX_BREAK_IF_Pos 7                                      /*!< UART_T::ISR: LIN RX BREAK IF Position   */
#define UART_ISR_LIN_RX_BREAK_IF_Msk (1ul << UART_ISR_LIN_RX_BREAK_IF_Pos)  /*!< UART_T::ISR: LIN RX BREAK IF Mask       */

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
#define UART_FUN_SEL_FUN_SEL_Pos        0                                        /*!< UART_T::FUN_SEL: FUN_SEL Position       */
#define UART_FUN_SEL_FUN_SEL_Msk        (0x3ul << UART_FUN_SEL_FUN_SEL_Pos)      /*!< UART_T::FUN_SEL: FUN_SEL Mask           */
/**@}*/ /* UART_CONST */
/**@}*/ /* UART */



/*----------------------------- WDT Controller -----------------------------*/
/** @addtogroup WDT WDT Controller(WDT)
  Memory Mapped Structure for Watchdog Timer
  @{
 */

typedef struct
{


/**
 * @var WDT_T::WTCR
 * Offset: 0x00  Watchdog Timer Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WTR       |Clear Watchdog Timer
 * |        |          |Set this bit will clear the Watchdog timer.
 * |        |          |0= Writing 0 to this bit has no effect
 * |        |          |1= Reset the contents of the Watchdog timer
 * |        |          |NOTE: This bit will auto clear after few clock cycle
 * |[1]     |WTRE      |Watchdog Timer Reset Enable
 * |        |          |Setting this bit will enable the Watchdog timer reset function.
 * |        |          |0= Disable Watchdog timer reset function
 * |        |          |1= Enable Watchdog timer reset function
 * |[2]     |WTRF      |Watchdog Timer Reset Flag
 * |        |          |When the Watchdog timer initiates a reset, the hardware will set this bit. This flag can be read by
 * |        |          |software to determine the source of reset. Software is responsible to clear it manually by writing 1 to
 * |        |          |it. If WTRE is disabled, then the Watchdog timer has no effect on this bit.
 * |        |          |0= Watchdog timer reset does not occur
 * |        |          |1= Watchdog timer reset occurs
 * |        |          |NOTE: This bit is cleared by writing 1 to this bit.
 * |[3]     |WTIF      |Watchdog Timer Interrupt Flag
 * |        |          |If the Watchdog timer interrupt is enabled, then the hardware will set this bit to indicate that the
 * |        |          |Watchdog timer interrupt has occurred. If the Watchdog timer interrupt is not enabled, then this bit
 * |        |          |indicates that a timeout period has elapsed.
 * |        |          |0= Watchdog timer interrupt does not occur
 * |        |          |1= Watchdog timer interrupt occurs
 * |        |          |NOTE: Write 1 to clear this bit to zero.
 * |[4]     |WTWKE     |Watchdog Timer Wakeup Function Enable bit
 * |        |          |0 = Disable Watchdog timer Wakeup CPU function.
 * |        |          |1 = Enable the Wakeup function that Watchdog timer timeout can wake up CPU from power-down
 * |        |          |mode.
 * |[5]     |WTWKF     |Watchdog Timer Wakeup Flag
 * |        |          |If Watchdog timer causes CPU wakes up from power-down mode, this bit will be set to high. It must
 * |        |          |be cleared by software with a write 1 to this bit.
 * |        |          |0 = Watchdog timer does not cause CPU wakeup.
 * |        |          |1 = CPU wake up from sleep or power-down mode by Watchdog timeout.
 * |        |          |NOTE: Write 1 to clear this bit to zero.
 * |[6]     |WTIE      |Watchdog Timer Interrupt Enable
 * |        |          |0= Disable the Watchdog timer interrupt
 * |        |          |1= Enable the Watchdog timer interrupt
 * |[7]     |WTE       |Watchdog Timer Enable
 * |        |          |0= Disable the Watchdog timer (This action will reset the internal counter)
 * |        |          |1= Enable the Watchdog timer
 * |[10:8]  |WTIS      |Watchdog Timer Interval Select (write protection bit)
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

    __IO uint32_t WTCR;          /* Offset: 0x00  Watchdog Timer Control Register                                    */

} WDT_T;



/** @addtogroup WDT_CONST WDT Bit Field Definition
  Constant Definitions for WDT Controller
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
/**@}*/ /* WDT_CONST */
/**@}*/ /* WDT */


/*----------------------------- WWDT Controller -----------------------------*/
/** @addtogroup WWDT Window Watchdog Timer (WWDT)
  Register data structure of Window Watchdog Timer and relative constant definitions.
  @{
 */

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
 * |        |          | counter value between 0 and WINCMP. If user writes WWDTRLD when current WWDT
 * |        |          | counter value is larger than WINCMP, WWDT reset signal will generate immediately.
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



/** @addtogroup WWDT_CONST WWDT Bit Field Definition
  Constant Definitions for WWDT Controller
  @{
 */

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
#define WWDT_WWDTCVR_WWDTCVAL_Pos   0                                           /*!< WWDT_T::WWDTCVR: WWDTCVAL Position */
#define WWDT_WWDTCVR_WWDTCVAL_Msk   (0x3Ful << WWDT_WWDTCVR_WWDTCVAL_Pos)       /*!< WWDT_T::WWDTCVR: WWDTCVAL Mask */
/**@}*/ /* end of group WWDT_CONST */
/**@}*/ /* end of group WWDT */
/**@}*/ /* end of group REGISTER */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup PERIPHERAL_BASE Peripheral Memory Base
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
#define P5_BASE             (GPIO_BASE      + 0x02C0)                   /*!< GPIO P5 Base Address                                */
#define P6_BASE             (GPIO_BASE      + 0x0300)                   /*!< GPIO P6 Base Address                                */
#define P7_BASE             (GPIO_BASE      + 0x0340)                   /*!< GPIO P7 Base Address                                */
#define GPIO_DBNCECON_BASE  (GPIO_BASE      + 0x0180)                   /*!< GPIO De-bounce Cycle Control Base Address           */
#define GPIO_PIN_DATA_BASE  (GPIO_BASE      + 0x0200)                   /*!< GPIO Pin Data Input/Output Control Base Address     */
#define GPIO_PIN_DATA_BASE2 (GPIO_BASE      + 0x0380)                   /*!< GPIO Pin Data Input/Output Control Base Address for P5 ~ P7 */

#define UART0_BASE          (APB1_BASE      + 0x50000)                  /*!< UART0 Base Address                               */

#define TIMER0_BASE         (APB1_BASE      + 0x10000)                  /*!< Timer0 Base Address                              */
#define TIMER1_BASE         (APB1_BASE      + 0x10020)                  /*!< Timer1 Base Address                              */
#define TIMER2_BASE         (APB2_BASE      + 0x10000)                  /*!< Timer2 Base Address                              */
#define TIMER3_BASE         (APB2_BASE      + 0x10020)                  /*!< Timer3 Base Address                              */

#define WDT_BASE            (APB1_BASE      + 0x4000)                   /*!< Watch Dog Timer Base Address                     */

#define WWDT_BASE           (APB1_BASE      + 0x4100)                   /*!< Window Watch Dog Timer Base Address              */

#define SPI0_BASE           (APB1_BASE      + 0x30000)                  /*!< SPI0 Base Address                                */

#define I2C0_BASE           (APB1_BASE      + 0x20000)                  /*!< I2C0 Base Address                                */
#define I2C1_BASE           (APB2_BASE      + 0x20000)                  /*!< I2C1 Base Address                                */

#define ADC_BASE            (APB1_BASE      + 0xE0000)                  /*!< ADC Base Address                                 */

#define CLK_BASE            (AHB_BASE       + 0x00200)                  /*!< System Clock Controller Base Address             */

#define GCR_BASE            (AHB_BASE       + 0x00000)                  /*!< System Global Controller Base Address            */

#define INT_BASE            (AHB_BASE       + 0x00300)                  /*!< Interrupt Source Controller Base Address         */

#define FMC_BASE            (AHB_BASE       + 0x0C000)                  /*!< Flash Memory Controller Base Address             */

#define PWMA_BASE           (APB1_BASE      + 0x40000)                  /*!< PWMA Base Address                                */


/**@}*/ /* PERIPHERAL */

/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/

/** @addtogroup PMODULE Peripheral Pointer
  The Declaration of Peripheral Pointer
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
#define GPIO                ((GPIO_DBNCECON_T *) GPIO_DBNCECON_BASE)    /*!< Interrupt De-bounce Cycle Control Configuration Struct */

#define UART0               ((UART_T *) UART0_BASE)                     /*!< UART0 Configuration Struct                       */

#define TIMER0              ((TIMER_T *) TIMER0_BASE)                   /*!< TIMER0 Configuration Struct                      */
#define TIMER1              ((TIMER_T *) TIMER1_BASE)                   /*!< TIMER1 Configuration Struct                      */
#define TIMER2              ((TIMER_T *) TIMER2_BASE)                   /*!< TIMER2 Configuration Struct                      */
#define TIMER3              ((TIMER_T *) TIMER3_BASE)                   /*!< TIMER3 Configuration Struct                      */

#define WDT                 ((WDT_T *) WDT_BASE)                        /*!< Watch Dog Timer Configuration Struct             */

#define WWDT                ((WWDT_T *) WWDT_BASE)                      /*!< Window Watch Dog Timer Configuration Struct      */

#define SPI0                ((SPI_T *) SPI0_BASE)                       /*!< SPI0 Configuration Struct                        */

#define I2C0                ((I2C_T *) I2C0_BASE)                       /*!< I2C0 Configuration Struct                        */
#define I2C1                ((I2C_T *) I2C1_BASE)                       /*!< I2C1 Configuration Struct                        */

#define ADC                 ((ADC_T *) ADC_BASE)                        /*!< ADC Configuration Struct                         */

#define CLK                 ((CLK_T *) CLK_BASE)                        /*!< System Clock Controller Configuration Struct     */

#define SYS                 ((GCR_T *) GCR_BASE)                        /*!< System Global Controller Configuration Struct    */

#define SYSINT              ((GCR_INT_T *) INT_BASE)                    /*!< Interrupt Source Controller Configuration Struct */

#define FMC                 ((FMC_T *) FMC_BASE)                        /*!< Flash Memory Controller */

#define PWMA                ((PWM_T *) PWMA_BASE)                       /*!< PWMA Configuration Struct                        */


/**@}*/ /* end of group PMODULE */


//=============================================================================
typedef volatile unsigned char  vu8;
typedef volatile unsigned long  vu32;
typedef volatile unsigned short vu16;
#define M8(adr)  (*((vu8  *) (adr)))
#define M16(adr) (*((vu16 *) (adr)))
#define M32(adr) (*((vu32 *) (adr)))

#define outpw(port,value)   (*((volatile unsigned int *)(port))=(value))
#define inpw(port)          ((*((volatile unsigned int *)(port))))
#define outpb(port,value)   (*((volatile unsigned char *)(port))=(value))
#define inpb(port)          ((*((volatile unsigned char *)(port))))
#define outps(port,value)   (*((volatile unsigned short *)(port))=(value))
#define inps(port)          ((*((volatile unsigned short *)(port))))

#define outp32(port,value)  (*((volatile unsigned int *)(port))=(value))
#define inp32(port)         ((*((volatile unsigned int *)(port))))
#define outp8(port,value)   (*((volatile unsigned char *)(port))=(value))
#define inp8(port)          ((*((volatile unsigned char *)(port))))
#define outp16(port,value)  (*((volatile unsigned short *)(port))=(value))
#define inp16(port)         ((*((volatile unsigned short *)(port))))


#define E_SUCCESS   0
#ifndef NULL
#define NULL        0
#endif

#define TRUE        1
#define FALSE       0

#define ENABLE      1
#define DISABLE     0

/* Bit Mask Definitions */
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


/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "sys.h"
#include "clk.h"
#include "fmc.h"
#include "gpio.h"
#include "pwm.h"
#include "spi.h"
#include "timer.h"
#include "wdt.h"
#include "wwdt.h"
#include "uart.h"
#include "i2c.h"
#include "adc.h"
#endif

/* Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved. */



