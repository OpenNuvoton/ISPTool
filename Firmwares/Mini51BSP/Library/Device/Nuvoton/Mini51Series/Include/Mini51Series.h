/**************************************************************************//**
 * @file     Mini51Series.h
 * @version  V1.00
 * $Revision: 43 $
 * $Date: 16/06/08 1:55p $
 * @brief    Mini51 series peripheral access layer header file.
 *           This file contains all the peripheral register's definitions,
 *           bits definitions and memory mapping for NuMicro Mini51 series MCU.
 *
 * @note
 * Copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/**
   \mainpage NuMicro Mini51DE Driver Reference Guide
   *
   * <b>Introduction</b>
   *
   * This user manual describes the usage of Mini51DE Series MCU device driver
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
   * Copyright (C) 2013-2018 Nuvoton Technology Corp. All rights reserved.
   */
#ifndef __MINI51SERIES_H__
#define __MINI51SERIES_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup MINI51_Definitions MINI51 Definitions
  This file defines all structures and symbols for Mini51:
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
/** @addtogroup MINI51_CMSIS Device CMSIS Definitions
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

    /******  Mini51 specific Interrupt Numbers ***********************************************/

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

/*@}*/ /* end of group MINI51_CMSIS */


#include "core_cm0.h"                       /* Cortex-M0 processor and core peripherals           */
#include "system_Mini51Series.h"            /* Mini51 Series System include file                  */
#include <stdint.h>

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup MINI51_Peripherals MINI51 Peripherals
  MINI51 Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM  )
#pragma anon_unions
#endif



/*---------------------- Analog Comparator Controller -------------------------*/
/**
    @addtogroup ACMP Analog Comparator Controller(ACMP)
    Memory Mapped Structure for ACMP Controller
@{ */


typedef struct
{

    /**
     * @var ACMP_T::CMPCR
     * Offset: 0x00  Analog Comparator 0 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPEN    |Analog Comparator Enable Control
     * |        |          |0 = Analog Comparator Disabled.
     * |        |          |1 = Analog Comparator Enabled.
     * |        |          |Note: Analog comparator output needs to wait 2 us stable time after this bit is set.
     * |[1]     |ACMPIE    |Analog Comparator 0 Interrupt Enable Control
     * |        |          |0 = Interrupt function Disabled.
     * |        |          |1 = Interrupt function Enabled.
     * |[2]     |HYSEN     |Analog Comparator Hysteresis Enable Control
     * |        |          |0 = Hysteresis function Disabled.
     * |        |          |1 = Hysteresis function Enabled.
     * |[4]     |NEGSEL    |Analog Comparator Negative Input Selection
     * |        |          |0 = The source of the negative comparator input is from CPN pin.
     * |        |          |1 = The source of the negative comparator input is from internal band-gap voltage or comparator reference voltage.
     * |[8]     |RISING    |Analog Comparator Rising Edge Trigger Enable Control
     * |        |          |0 = Analog comparator rising edge trigger Disabled.
     * |        |          |1 = Analog comparator rising edge trigger PWM or Timer Enabled.
     * |        |          |Note: The bit is only effective while analog comparator triggers PWM or Timer.
     * |[9]     |FALLING   |Analog Comparator Falling Edge Trigger Enable Control
     * |        |          |0 = Analog comparator falling edge trigger Disabled.
     * |        |          |1 = Analog comparator falling edge trigger PWM or Timer Enabled.
     * |        |          |Note: The bit is only effective while analog comparator triggers PWM or Timer.
     * |[30:29] |CPPSEL    |Analog Comparator Positive Input Selection
     * @var ACMP_T::CMPSR
     * Offset: 0x08  Analog Comparator 0/1 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPF0    |Analog Comparator 0 Flag
     * |        |          |This bit is set by hardware whenever the comparator 0 output changes state.
     * |        |          |This will generate an interrupt if ACMPIE(ACMP_CR0[1]) = 1.
     * |        |          |0 = Analog comparator 0 output does not change.
     * |        |          |1 = Analog comparator 0 output changed.
     * |        |          |Note: Software can write 1 to clear this bit to 0.
     * |[1]     |ACMPF1    |Analog Comparator 1 Flag
     * |        |          |This bit is set by hardware whenever the comparator 1 output changes state.
     * |        |          |This will generate an interrupt if ACMPIE(ACMP_CR1[1]) = 1.
     * |        |          |0 = Analog comparator 1 output does not change.
     * |        |          |1 = Analog comparator 1 output changed.
     * |        |          |Note: Software can write 1 to clear this bit to 0.
     * |[2]     |ACMPO0    |Analog Comparator 0 Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the comparator 0 is disabled ACMPEN(ACMP_CR0[0]) = 0.
     * |        |          |0 = Analog comparator 0 outputs 0.
     * |        |          |1 = Analog comparator 0 outputs 1.
     * |[3]     |ACMPO1    |Analog Comparator 1 Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the comparator 1 is disabled ACMPEN(ACMP_CR1[0]) = 0.
     * |        |          |0 = Analog comparator 1 outputs 0.
     * |        |          |1 = Analog comparator 1 outputs 1.
     * @var ACMP_T::CMPRVCR
     * Offset: 0x0C  Analog Comparator Reference Voltage Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CRVS      |Comparator Reference Voltage Setting
     * |        |          |Comparator reference voltage = AVDD * (1 / 6 + CRVS[3:0] / 24).
     * |[7]     |OUT_SEL   |CRV Module Output Selection
     * |        |          |0 = Band-gap voltage.
     * |        |          |1 = Internal comparator reference voltage.
     */

    __IO uint32_t CMPCR[2];      /* Offset: 0x00 ~0x04 Analog Comparator 0 & 1Control Register                       */
    __IO uint32_t CMPSR;         /* Offset: 0x08  Analog Comparator 0/1 Status Register                              */
    __IO uint32_t CMPRVCR;       /* Offset: 0x0C  Analog Comparator Reference Voltage Control Register               */

} ACMP_T;



/**
    @addtogroup ACMP_CONST ACMP Bit Field Definition
    Constant Definitions for ACMP Controller
@{ */

#define ACMP_CMPCR_ACMPEN_Pos           (0)                                               /*!< ACMP_T::CMPCR: ACMPEN Position           */
#define ACMP_CMPCR_ACMPEN_Msk           (0x1ul << ACMP_CMPCR_ACMPEN_Pos)                  /*!< ACMP_T::CMPCR: ACMPEN Mask               */

#define ACMP_CMPCR_ACMPIE_Pos           (1)                                               /*!< ACMP_T::CMPCR: ACMPIE Position           */
#define ACMP_CMPCR_ACMPIE_Msk           (0x1ul << ACMP_CMPCR_ACMPIE_Pos)                  /*!< ACMP_T::CMPCR: ACMPIE Mask               */

#define ACMP_CMPCR_HYSEN_Pos            (2)                                               /*!< ACMP_T::CMPCR: HYSEN Position            */
#define ACMP_CMPCR_HYSEN_Msk            (0x1ul << ACMP_CMPCR_HYSEN_Pos)                   /*!< ACMP_T::CMPCR: HYSEN Mask                */

#define ACMP_CMPCR_NEGSEL_Pos           (4)                                               /*!< ACMP_T::CMPCR: NEGSEL Position           */
#define ACMP_CMPCR_NEGSEL_Msk           (0x1ul << ACMP_CMPCR_NEGSEL_Pos)                  /*!< ACMP_T::CMPCR: NEGSEL Mask               */

#define ACMP_CMPCR_RISING_Pos           (8)                                               /*!< ACMP_T::CMPCR: RISING Position           */
#define ACMP_CMPCR_RISING_Msk           (0x1ul << ACMP_CMPCR_RISING_Pos)                  /*!< ACMP_T::CMPCR: RISING Mask               */

#define ACMP_CMPCR_FALLING_Pos          (9)                                               /*!< ACMP_T::CMPCR: FALLING Position          */
#define ACMP_CMPCR_FALLING_Msk          (0x1ul << ACMP_CMPCR_FALLING_Pos)                 /*!< ACMP_T::CMPCR: FALLING Mask              */

#define ACMP_CMPCR_CPPSEL_Pos           (29)                                              /*!< ACMP_T::CMPCR: CPP0SEL Position          */
#define ACMP_CMPCR_CPPSEL_Msk           (0x3ul << ACMP_CMPCR_CPPSEL_Pos)                  /*!< ACMP_T::CMPCR: CPP0SEL Mask              */

#define ACMP_CMPSR_ACMPF0_Pos           (0)                                               /*!< ACMP_T::CMPSR: ACMPF0 Position            */
#define ACMP_CMPSR_ACMPF0_Msk           (0x1ul << ACMP_CMPSR_ACMPF0_Pos)                  /*!< ACMP_T::CMPSR: ACMPF0 Mask                */

#define ACMP_CMPSR_ACMPF1_Pos           (1)                                               /*!< ACMP_T::CMPSR: ACMPF1 Position            */
#define ACMP_CMPSR_ACMPF1_Msk           (0x1ul << ACMP_CMPSR_ACMPF1_Pos)                  /*!< ACMP_T::CMPSR: ACMPF1 Mask                */

#define ACMP_CMPSR_ACMPCO0_Pos          (2)                                               /*!< ACMP_T::CMPSR: ACMPO0 Position            */
#define ACMP_CMPSR_ACMPCO0_Msk          (0x1ul << ACMP_CMPSR_ACMPCO0_Pos)                 /*!< ACMP_T::CMPSR: ACMPO0 Mask                */

#define ACMP_CMPSR_ACMPCO1_Pos          (3)                                               /*!< ACMP_T::CMPSR: ACMPO1 Position            */
#define ACMP_CMPSR_ACMPCO1_Msk          (0x1ul << ACMP_CMPSR_ACMPCO1_Pos)                 /*!< ACMP_T::CMPSR: ACMPO1 Mask                */

#define ACMP_CMPRVCR_CRVS_Pos           (0)                                               /*!< ACMP_T::CMPRVCR: CRVS Position              */
#define ACMP_CMPRVCR_CRVS_Msk           (0xful << ACMP_CMPRVCR_CRVS_Pos)                  /*!< ACMP_T::CMPRVCR: CRVS Mask                  */

#define ACMP_CMPRVCR_OUT_SEL_Pos        (7)                                               /*!< ACMP_T::CMPRVCR: OUT_SEL Position           */
#define ACMP_CMPRVCR_OUT_SEL_Msk        (0x1ul << ACMP_CMPRVCR_OUT_SEL_Pos)               /*!< ACMP_T::CMPRVCR: OUT_SEL Mask               */

/**@}*/ /* ACMP_CONST */
/**@}*/ /* end of ACMP register group */


/*---------------------- Analog to Digital Converter -------------------------*/
/**
    @addtogroup ADC Analog to Digital Converter(ADC)
    Memory Mapped Structure for ADC Controller
@{ */


typedef struct
{
    /**
     * @var ADC_T::ADDR
     * Offset: 0x00  ADC Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |RSLT      |A/D Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OVERRUN   |Over Run Flag
     * |        |          |0 = Data in RSLT (ADDR[9:0])is recent conversion result.
     * |        |          |1 = Data in RSLT (ADDR[9:0])overwrote.
     * |        |          |If converted data in RSLT[9:0] has not been read before the new conversion result is loaded to this register, OVERRUN is set to 1.
     * |        |          |It is cleared by hardware after the ADDR register is read.
     * |[17]    |VALID     |Valid Flag
     * |        |          |0 = Data in RSLT (ADDR[9:0]) bits not valid.
     * |        |          |1 = Data in RSLT (ADDR[9:0]) bits valid.
     * |        |          |This bit is set to 1 when ADC conversion is completed and cleared by hardware after the ADDR register is read.
     * @var ADC_T::ADCR
     * Offset: 0x20  ADC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADEN      |A/D Converter Enable Control
     * |        |          |0 = A/D Converter Disabled.
     * |        |          |1 = A/D Converter Enabled.
     * |        |          |Note: Before starting A/D conversion function, this bit should be set to 1.
     * |        |          |Clear it to 0 to disable A/D converter analog circuit to save power consumption.
     * |[1]     |ADIE      |A/D Interrupt Enable Control
     * |        |          |A/D conversion end interrupt request is generated if ADIE bit is set to 1.
     * |        |          |0 = A/D interrupt function Disabled.
     * |        |          |1 = A/D interrupt function Enabled.
     * |[5:4]   |TRGS      |Hardware Trigger Source
     * |        |          |00 = A/D conversion is started by external STADC pin.
     * |        |          |11 = A/D conversion is started by PWM trigger.
     * |        |          |Others = Reserved.
     * |        |          |Note: Software should disable TRGEN and ADST before change TRGS.
     * |[6]     |TRGCOND   |External Trigger Condition
     * |        |          |This bit decides whether the external pin STADC trigger event is falling or raising edge.
     * |        |          |The signal must be kept at stable state at least 4 PCLKs at high and low state for edge trigger.
     * |        |          |0 = Falling edge.
     * |        |          |1 = Raising edge.
     * |[8]     |TRGEN     |External Trigger Enable Control
     * |        |          |Enable or disable triggering of A/D conversion by external STADC pin.
     * |        |          |If external trigger is enabled, the ADST bit can be set to 1 by the selected hardware trigger source.
     * |        |          |0= External trigger Disabled.
     * |        |          |1= External trigger Enabled.
     * |[11]    |ADST      |A/D Conversion Start
     * |        |          |ADST bit can be set to 1 from three sources: software or PWM trigger and external pin STADC.
     * |        |          |ADST will be cleared to 0 by hardware automatically after conversion complete.
     * |        |          |0 = Conversion stopped and A/D converter entered idle state.
     * |        |          |1 = Conversion start.
     * @var ADC_T::ADCHER
     * Offset: 0x24  ADC Channel Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHEN0     |Analog Input Channel 0 Enable Control
     * |        |          |0 = Channel 0 Disabled.
     * |        |          |1 = Channel 0 Enabled.
     * |        |          |Note: If software enables more than one channel, the channel with the smallest number will be selected and the other enabled channels will be ignored.
     * |[1]     |CHEN1     |Analog Input Channel 1 Enable Control
     * |        |          |0 = Channel 1 Disabled.
     * |        |          |1 = Channel 1 Enabled.
     * |[2]     |CHEN2     |Analog Input Channel 2 Enable Control
     * |        |          |0 = Channel 2 Disabled.
     * |        |          |1 = Channel 2 Enabled.
     * |[3]     |CHEN3     |Analog Input Channel 3 Enable Control
     * |        |          |0 = Channel 3 Disabled.
     * |        |          |1 = Channel 3 Enabled.
     * |[4]     |CHEN4     |Analog Input Channel 4 Enable Control
     * |        |          |0 = Channel 4 Disabled.
     * |        |          |1 = Channel 4 Enabled.
     * |[5]     |CHEN5     |Analog Input Channel 5 Enable Control
     * |        |          |0 = Channel 5 Disabled.
     * |        |          |1 = Channel 5 Enabled.
     * |[6]     |CHEN6     |Analog Input Channel 6 Enable Control
     * |        |          |0 = Channel 6 Disabled.
     * |        |          |1 = Channel 6 Enabled.
     * |[7]     |CHEN7     |Analog Input Channel 7 Enable Control
     * |        |          |0 = Channel 7 Disabled.
     * |        |          |1 = Channel 7 Enabled.
     * |[8]     |PRESEL    |Analog Input Channel 7 Selection
     * |        |          |0 = External analog input.
     * |        |          |1 = Internal band-gap voltage (VBG).
     * |        |          |Note: When software selects the band-gap voltage as the analog input source of ADC channel 7, the ADC clock rate needs to be limited to lower than 300 kHz.
     * @var ADC_T::ADCMPR
     * Offset: 0x28 ~ 0x2C ADC Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CMPEN     |Compare Enable Control
     * |        |          |Set 1 to this bit to enable comparing CMPD[9:0] with specified channel conversion results when converted data is loaded into the ADDR register.
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |[1]     |CMPIE     |Compare Interrupt Enable Control
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMATCNT, CMPFx bit will be asserted, in the meanwhile, if CMPIE is set to 1, a compare interrupt request is generated.
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = Set the compare condition as that when a 10-bit A/D conversion result is less than the 10-bit CMPD (ADCMPRx[25:16]), the internal match counter will increase one.
     * |        |          |1 = Set the compare condition as that when a 10-bit A/D conversion result is greater or equal to the 10-bit CMPD (ADCMPRx[25:16]), the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches the value to (CMPMATCNT +1), the CMPFx bit will be set.
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
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND, the internal match counter will increase 1.
     * |        |          |When the internal counter reaches the value to (CMPMATCNT +1), the CMPFx bit will be set.
     * |[25:16] |CMPD      |Comparison Data
     * |        |          |The 10-bit data is used to compare with conversion result of specified channel.
     * @var ADC_T::ADSR
     * Offset: 0x30  ADC Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADF       |A/D Conversion End Flag
     * |        |          |A status flag that indicates the end of A/D conversion. ADF is set to 1 When A/D conversion ends.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |[1]     |CMPF0     |Compare Flag 0
     * |        |          |When the selected channel A/D conversion result meets the setting condition in ADCMPR0, this bit is set to 1.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |        |          |0 = Conversion result in ADDR does not meet the ADCMPR0 setting.
     * |        |          |1 = Conversion result in ADDR meets the ADCMPR0 setting.
     * |[2]     |CMPF1     |Compare Flag 1
     * |        |          |When the selected channel A/D conversion result meets the setting condition in ADCMPR1, this bit is set to 1.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |        |          |0 = Conversion result in ADDR does not meet the ADCMPR1 setting.
     * |        |          |1 = Conversion result in ADDR meets the ADCMPR1 setting.
     * |[3]     |BUSY      |BUSY/IDLE (Read Only)
     * |        |          |This bit is mirror of as ADST bit in ADCR
     * |        |          |0 = A/D converter is in idle state.
     * |        |          |1 = A/D converter is busy at conversion.
     * |[6:4]   |CHANNEL   |Current Conversion Channel (Read Only)
     * |        |          |This filed reflects the current conversion channel when BUSY=1.
     * |        |          |When BUSY=0, it shows the number of the next converted channel.
     * |[8]     |VALID     |Data Valid Flag (Read Only)
     * |        |          |It is a mirror of VALID (ADDR[17]) bit in ADDR register.
     * |[16]    |OVERRUN   |Overrun Flag (Read Only)
     * |        |          |It is a mirror to OVERRUN (ADSR[16]) bit in ADDR register.
     * @var ADC_T::ADTDCR
     * Offset: 0x44  ADC Trigger Delay Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PTDT      |PWM Trigger Delay Timer
     * |        |          |Set this field will delay ADC start conversion time after PWM trigger.
     * |        |          |PWM trigger delay time is (4 * PTDT) * system clock.
     * @var ADC_T::ADSAMP
     * Offset: 0x48  ADC Sampling Time Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |ADSAMPCNT |ADC Sampling Counter
     * |        |          |If the ADC input is unstable, user can set this register to increase the sampling time to get a stable ADC input signal.
     * |        |          |The default sampling time is 1 ADC clock.
     * |        |          |The additional clock number will be inserted to lengthen the sampling clock.
     * |        |          |0000 = 0 additional ADC sample clock.
     * |        |          |0001 = 1 additional ADC sample clock.
     * |        |          |0010 = 2 additional ADC sample clock.
     * |        |          |0011 = 4 additional ADC sample clock.
     * |        |          |0100 = 8 additional ADC sample clock.
     * |        |          |0101 = 16 additional ADC sample clock.
     * |        |          |0110 = 32 additional ADC sample clock.
     * |        |          |0111 = 64 additional ADC sample clock.
     * |        |          |1000 = 128 additional ADC sample clock.
     * |        |          |1001 = 256 additional ADC sample clock.
     * |        |          |1010 = 512 additional ADC sample clock.
     * |        |          |1011 = 1024 additional ADC sample clock.
     * |        |          |1100 = 1024 additional ADC sample clock.
     * |        |          |1101 = 1024 additional ADC sample clock.
     * |        |          |1110 = 1024 additional ADC sample clock.
     * |        |          |1111 = 1024 additional ADC sample clock.
     */

    __I  uint32_t ADDR;          /* Offset: 0x00  ADC Data Register                                                  */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0[7];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t ADCR;          /* Offset: 0x20  ADC Control Register                                               */
    __IO uint32_t ADCHER;        /* Offset: 0x24  ADC Channel Enable Control Register                                */
    __IO uint32_t ADCMPR[2];     /* Offset: 0x28, 0x2C  A/D Compare Register 0 & 1                                   */
    __IO uint32_t ADSR;          /* Offset: 0x30  ADC Status Register                                                */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE1[4];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t ADTDCR;        /* Offset: 0x44  ADC Trigger Delay Control Register                                 */
    __IO uint32_t ADSAMP;        /* Offset: 0x48  ADC Sampling Time Counter Register                                 */

} ADC_T;



/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */

#define ADC_ADDR_RSLT_Pos                (0)                                               /*!< ADC_T::ADDR: RSLT Position                */
#define ADC_ADDR_RSLT_Msk                (0x3fful << ADC_ADDR_RSLT_Pos)                    /*!< ADC_T::ADDR: RSLT Mask                    */

#define ADC_ADDR_OVERRUN_Pos             (16)                                              /*!< ADC_T::ADDR: OVERRUN Position             */
#define ADC_ADDR_OVERRUN_Msk             (0x1ul << ADC_ADDR_OVERRUN_Pos)                   /*!< ADC_T::ADDR: OVERRUN Mask                 */

#define ADC_ADDR_VALID_Pos               (17)                                              /*!< ADC_T::ADDR: VALID Position               */
#define ADC_ADDR_VALID_Msk               (0x1ul << ADC_ADDR_VALID_Pos)                     /*!< ADC_T::ADDR: VALID Mask                   */

#define ADC_ADCR_ADEN_Pos                (0)                                               /*!< ADC_T::ADCR: ADEN Position                */
#define ADC_ADCR_ADEN_Msk                (0x1ul << ADC_ADCR_ADEN_Pos)                      /*!< ADC_T::ADCR: ADEN Mask                    */

#define ADC_ADCR_ADIE_Pos                (1)                                               /*!< ADC_T::ADCR: ADIE Position                */
#define ADC_ADCR_ADIE_Msk                (0x1ul << ADC_ADCR_ADIE_Pos)                      /*!< ADC_T::ADCR: ADIE Mask                    */

#define ADC_ADCR_TRGS_Pos                (4)                                               /*!< ADC_T::ADCR: TRGS Position                */
#define ADC_ADCR_TRGS_Msk                (0x3ul << ADC_ADCR_TRGS_Pos)                      /*!< ADC_T::ADCR: TRGS Mask                    */

#define ADC_ADCR_TRGCOND_Pos             (6)                                               /*!< ADC_T::ADCR: TRGCOND Position             */
#define ADC_ADCR_TRGCOND_Msk             (0x1ul << ADC_ADCR_TRGCOND_Pos)                   /*!< ADC_T::ADCR: TRGCOND Mask                 */

#define ADC_ADCR_TRGEN_Pos               (8)                                               /*!< ADC_T::ADCR: TRGEN Position               */
#define ADC_ADCR_TRGEN_Msk               (0x1ul << ADC_ADCR_TRGEN_Pos)                     /*!< ADC_T::ADCR: TRGEN Mask                   */

#define ADC_ADCR_ADST_Pos                (11)                                              /*!< ADC_T::ADCR: ADST Position                */
#define ADC_ADCR_ADST_Msk                (0x1ul << ADC_ADCR_ADST_Pos)                      /*!< ADC_T::ADCR: ADST Mask                    */

#define ADC_ADCHER_CHEN_Pos              (0)                                               /*!< ADC_T::ADCHER: CHEN Position              */
#define ADC_ADCHER_CHEN_Msk              (0xfful << ADC_ADCHER_CHEN_Pos)                   /*!< ADC_T::ADCHER: CHEN Mask                  */

#define ADC_ADCHER_CHEN0_Pos             (0)                                               /*!< ADC_T::ADCHER: CHEN0 Position             */
#define ADC_ADCHER_CHEN0_Msk             (0x1ul << ADC_ADCHER_CHEN0_Pos)                   /*!< ADC_T::ADCHER: CHEN0 Mask                 */

#define ADC_ADCHER_CHEN1_Pos             (1)                                               /*!< ADC_T::ADCHER: CHEN1 Position             */
#define ADC_ADCHER_CHEN1_Msk             (0x1ul << ADC_ADCHER_CHEN1_Pos)                   /*!< ADC_T::ADCHER: CHEN1 Mask                 */

#define ADC_ADCHER_CHEN2_Pos             (2)                                               /*!< ADC_T::ADCHER: CHEN2 Position             */
#define ADC_ADCHER_CHEN2_Msk             (0x1ul << ADC_ADCHER_CHEN2_Pos)                   /*!< ADC_T::ADCHER: CHEN2 Mask                 */

#define ADC_ADCHER_CHEN3_Pos             (3)                                               /*!< ADC_T::ADCHER: CHEN3 Position             */
#define ADC_ADCHER_CHEN3_Msk             (0x1ul << ADC_ADCHER_CHEN3_Pos)                   /*!< ADC_T::ADCHER: CHEN3 Mask                 */

#define ADC_ADCHER_CHEN4_Pos             (4)                                               /*!< ADC_T::ADCHER: CHEN4 Position             */
#define ADC_ADCHER_CHEN4_Msk             (0x1ul << ADC_ADCHER_CHEN4_Pos)                   /*!< ADC_T::ADCHER: CHEN4 Mask                 */

#define ADC_ADCHER_CHEN5_Pos             (5)                                               /*!< ADC_T::ADCHER: CHEN5 Position             */
#define ADC_ADCHER_CHEN5_Msk             (0x1ul << ADC_ADCHER_CHEN5_Pos)                   /*!< ADC_T::ADCHER: CHEN5 Mask                 */

#define ADC_ADCHER_CHEN6_Pos             (6)                                               /*!< ADC_T::ADCHER: CHEN6 Position             */
#define ADC_ADCHER_CHEN6_Msk             (0x1ul << ADC_ADCHER_CHEN6_Pos)                   /*!< ADC_T::ADCHER: CHEN6 Mask                 */

#define ADC_ADCHER_CHEN7_Pos             (7)                                               /*!< ADC_T::ADCHER: CHEN7 Position             */
#define ADC_ADCHER_CHEN7_Msk             (0x1ul << ADC_ADCHER_CHEN7_Pos)                   /*!< ADC_T::ADCHER: CHEN7 Mask                 */

#define ADC_ADCHER_PRESEL_Pos            (8)                                               /*!< ADC_T::ADCHER: PRESEL Position            */
#define ADC_ADCHER_PRESEL_Msk            (0x1ul << ADC_ADCHER_PRESEL_Pos)                  /*!< ADC_T::ADCHER: PRESEL Mask                */

#define ADC_ADCMPR_CMPEN_Pos             (0)                                               /*!< ADC_T::ADCMPR: CMPEN Position             */
#define ADC_ADCMPR_CMPEN_Msk             (0x1ul << ADC_ADCMPR_CMPEN_Pos)                   /*!< ADC_T::ADCMPR: CMPEN Mask                 */

#define ADC_ADCMPR_CMPIE_Pos             (1)                                               /*!< ADC_T::ADCMPR: CMPIE Position             */
#define ADC_ADCMPR_CMPIE_Msk             (0x1ul << ADC_ADCMPR_CMPIE_Pos)                   /*!< ADC_T::ADCMPR: CMPIE Mask                 */

#define ADC_ADCMPR_CMPCOND_Pos           (2)                                               /*!< ADC_T::ADCMPR: CMPCOND Position           */
#define ADC_ADCMPR_CMPCOND_Msk           (0x1ul << ADC_ADCMPR_CMPCOND_Pos)                 /*!< ADC_T::ADCMPR: CMPCOND Mask               */

#define ADC_ADCMPR_CMPCH_Pos             (3)                                               /*!< ADC_T::ADCMPR: CMPCH Position             */
#define ADC_ADCMPR_CMPCH_Msk             (0x7ul << ADC_ADCMPR_CMPCH_Pos)                   /*!< ADC_T::ADCMPR: CMPCH Mask                 */

#define ADC_ADCMPR_CMPMATCNT_Pos         (8)                                               /*!< ADC_T::ADCMPR: CMPMATCNT Position         */
#define ADC_ADCMPR_CMPMATCNT_Msk         (0xful << ADC_ADCMPR_CMPMATCNT_Pos)               /*!< ADC_T::ADCMPR: CMPMATCNT Mask             */

#define ADC_ADCMPR_CMPD_Pos              (16)                                              /*!< ADC_T::ADCMPR: CMPD Position              */
#define ADC_ADCMPR_CMPD_Msk              (0x3fful << ADC_ADCMPR_CMPD_Pos)                  /*!< ADC_T::ADCMPR: CMPD Mask                  */

#define ADC_ADSR_ADF_Pos                 (0)                                               /*!< ADC_T::ADSR: ADF Position                 */
#define ADC_ADSR_ADF_Msk                 (0x1ul << ADC_ADSR_ADF_Pos)                       /*!< ADC_T::ADSR: ADF Mask                     */

#define ADC_ADSR_CMPF0_Pos               (1)                                               /*!< ADC_T::ADSR: CMPF0 Position               */
#define ADC_ADSR_CMPF0_Msk               (0x1ul << ADC_ADSR_CMPF0_Pos)                     /*!< ADC_T::ADSR: CMPF0 Mask                   */

#define ADC_ADSR_CMPF1_Pos               (2)                                               /*!< ADC_T::ADSR: CMPF1 Position               */
#define ADC_ADSR_CMPF1_Msk               (0x1ul << ADC_ADSR_CMPF1_Pos)                     /*!< ADC_T::ADSR: CMPF1 Mask                   */

#define ADC_ADSR_BUSY_Pos                (3)                                               /*!< ADC_T::ADSR: BUSY Position                */
#define ADC_ADSR_BUSY_Msk                (0x1ul << ADC_ADSR_BUSY_Pos)                      /*!< ADC_T::ADSR: BUSY Mask                    */

#define ADC_ADSR_CHANNEL_Pos             (4)                                               /*!< ADC_T::ADSR: CHANNEL Position             */
#define ADC_ADSR_CHANNEL_Msk             (0x7ul << ADC_ADSR_CHANNEL_Pos)                   /*!< ADC_T::ADSR: CHANNEL Mask                 */

#define ADC_ADSR_VALID_Pos               (8)                                               /*!< ADC_T::ADSR: VALID Position               */
#define ADC_ADSR_VALID_Msk               (0x1ul << ADC_ADSR_VALID_Pos)                     /*!< ADC_T::ADSR: VALID Mask                   */

#define ADC_ADSR_OVERRUN_Pos             (16)                                              /*!< ADC_T::ADSR: OVERRUN Position             */
#define ADC_ADSR_OVERRUN_Msk             (0x1ul << ADC_ADSR_OVERRUN_Pos)                   /*!< ADC_T::ADSR: OVERRUN Mask                 */

#define ADC_ADTDCR_PTDT_Pos              (0)                                               /*!< ADC_T::ADTDCR: PTDT Position              */
#define ADC_ADTDCR_PTDT_Msk              (0xfful << ADC_ADTDCR_PTDT_Pos)                   /*!< ADC_T::ADTDCR: PTDT Mask                  */

#define ADC_ADSAMP_SAMPCNT_Pos           (0)                                               /*!< ADC_T::ADSAMP: ADSAMPCNT Position         */
#define ADC_ADSAMP_SAMPCNT_Msk           (0xful << ADC_ADSAMP_SAMPCNT_Pos)                 /*!< ADC_T::ADSAMP: ADSAMPCNT Mask             */

/**@}*/ /* ADC_CONST */
/**@}*/ /* end of ADC register group */


/*---------------------- System Clock Controller -------------------------*/
/**
    @addtogroup CLK System Clock Controller(CLK)
    Memory Mapped Structure for CLK Controller
@{ */


typedef struct
{

    /**
     * @var CLK_T::PWRCON
     * Offset: 0x00  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |XTLCLK_EN |External Crystal HXT Or LXT Enable Control (Write Protect)
     * |        |          |The default clock source is from HIRC.
     * |        |          |These two bits are default set to "00" and the XTAL1 and XTAL2 pins are GPIO.
     * |        |          |00 = XTAL1 and XTAL2 are GPIO, disable both LXT & HXT (default).
     * |        |          |01 = HXT Enabled.
     * |        |          |10 = LXT Enabled.
     * |        |          |11 = XTAL1 is external clock input pin, XTAL2 is GPIO.
     * |        |          |Note: To enable the external XTAL function, the P5_ALT[1:0] and P5_MFP[1:0] bits must also be set in P5_MFP.
     * |[2]     |OSC22M_EN |22.1184 MHz Internal High Speed RC Oscillator (HIRC) Enable Control (Write Protect)
     * |        |          |0 = 22.1184 MHz internal high speed RC oscillator (HIRC) Disabled.
     * |        |          |1 = 22.1184 MHz internal high speed RC oscillator (HIRC) Enabled.
     * |        |          |Note: The default of OSC22M_EN bit is 1.
     * |[3]     |OSC10K_EN |10 KHz Internal Low Speed RC Oscillator (LIRC) Enable Control (Write Protect)
     * |        |          |0 = 10 kHz internal low speed RC oscillator (LIRC) Disabled.
     * |        |          |1 = 10 kHz internal low speed RC oscillator (LIRC) Enabled.
     * |[4]     |PD_WU_DLY |Wake-up Delay Counter Enable Control (Write Protect)
     * |        |          |When the chip wakes up from Power-down mode, the clock control will delay certain clock cycles to wait system clock stable.
     * |        |          |The delayed clock cycle is 4096 clock cycles when chip work at 4~24 MHz external high speed crystal (HXT), 4096 clock cycles for 32.768 kHz external low speed crystal (LXT), and 16 clock cycles when chip works at 22.1184 MHz internal high speed RC oscillator (HIRC).
     * |        |          |0 = Clock cycles delay Disabled.
     * |        |          |1 = Clock cycles delay Enabled.
     * |[5]     |PD_WU_INT_EN|Power-down Mode Wake-up Interrupt Enable Control (Write Protect)
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: The interrupt will occur when both PD_WU_STS and PD_WU_INT_EN are high.
     * |[6]     |PD_WU_STS |Power-down Mode Wake-up Interrupt Status
     * |        |          |Set by "Power-down wake-up event", which indicates that resume from Power-down mode.
     * |        |          |The flag is set if the GPIO, UART, WDT, I2C, ACMP, Timer or BOD wake-up occurred.
     * |        |          |Note: This bit works only if PD_WU_INT_EN (PWRCON[5]) set to 1. Write 1 to clear the bit to 0.
     * |[7]     |PWR_DOWN_EN|System Power-down Enable Bit (Write Protect)
     * |        |          |When chip wakes up from Power-down mode, this bit is cleared by hardware.
     * |        |          |User needs to set this bit again for next Power-down.
     * |        |          |In Power-down mode, 4~24 MHz external high speed crystal oscillator (HXT), 32.768 kHz external low speed crystal oscillator (LXT), and the 22.1184 MHz internal high speed oscillator (HIRC) will be disabled in this mode, and 10 kHz internal low speed RC oscillator (LIRC) are not controlled by Power-down mode.
     * |        |          |In Power-down mode, the system clock is disabled, and ignored the clock source selection.
     * |        |          |The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from 10 kHz internal low speed oscillator.
     * |        |          |0 = Chip operating normally or chip in Idle mode because of WFI command.
     * |        |          |1 = Chip enters Power-down mode instantly or waits CPU sleep command WFI.
     * |[9]     |PD_32K    |Enable LXT In Power-down Mode
     * |        |          |This bit controls the crystal oscillator active or not in Power-down mode.
     * |        |          |0 = No effect to Power-down mode.
     * |        |          |1 = If XTLCLK_EN[1:0] = 10, LXT is still active in Power-down mode.
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
     * |        |          |Note: This bit is the protected bit, and programming it needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA + 0x100.
     * |[2]     |TMR0_EN   |Timer0 Clock Enable Control
     * |        |          |0 = Timer0 clock Disabled.
     * |        |          |1 = Timer0 clock Enabled.
     * |[3]     |TMR1_EN   |Timer1 Clock Enable Control
     * |        |          |0 = Timer1 clock Disabled.
     * |        |          |1 = Timer1 clock Enabled.
     * |[6]     |FDIV_EN   |Frequency Divider Output Clock Enable Control
     * |        |          |0 = FDIV clock Disabled.
     * |        |          |1 = FDIV clock Enabled.
     * |[8]     |I2C_EN    |I2C Clock Enable Control
     * |        |          |0 = I2C clock Disabled.
     * |        |          |1 = I2C clock Enabled.
     * |[12]    |SPI_EN    |SPI Peripheral Clock Enable Control
     * |        |          |0 = SPI peripheral clock Disabled.
     * |        |          |1 = SPI peripheral clock Enabled.
     * |[16]    |UART_EN   |UART Clock Enable Control
     * |        |          |0 = UART clock Disabled.
     * |        |          |1 = UART clock Enabled.
     * |[20]    |PWM01_EN  |PWM_01 Clock Enable Control
     * |        |          |0 = PWM01 clock Disabled.
     * |        |          |1 = PWM01 clock Enabled.
     * |[21]    |PWM23_EN  |PWM_23 Clock Enable Control
     * |        |          |0 = PWM23 clock Disabled.
     * |        |          |1 = PWM23 clock Enabled.
     * |[22]    |PWM45_EN  |PWM_45 Clock Enable Control
     * |        |          |0 = PWM45 clock Disabled.
     * |        |          |1 = PWM45 clock Enabled.
     * |[28]    |ADC_EN    |Analog-digital-converter (ADC) Clock Enable Control
     * |        |          |0 = ADC peripheral clock Disabled.
     * |        |          |1 = ADC peripheral clock Enabled.
     * |[30]    |ACMP_EN   |Analog Comparator Clock Enable Control
     * |        |          |0 = Analog Comparator clock Disabled.
     * |        |          |1 = Analog Comparator clock Enabled.
     * @var CLK_T::CLKSTATUS
     * Offset: 0x0C  Clock Status Monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |XTL_STB   |HXT Or LXT Clock Source Stable Flag
     * |        |          |0 = HXT or LXT clock is not stable or disabled.
     * |        |          |1 = HXT or LXT clock is stable.
     * |[3]     |OSC10K_STB|LIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = LIRC clock is not stable or disabled.
     * |        |          |1 = LIRC clock is stable.
     * |[4]     |OSC22M_STB|HIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = HIRC clock is not stable or disabled.
     * |        |          |1 = HIRC clock is stable.
     * |[7]     |CLK_SW_FAIL|Clock Switch Fail Flag
     * |        |          |0 = Clock switching success.
     * |        |          |1 = Clock switching failed.
     * |        |          |Note1: This bit is updated when software switches system clock source.
     * |        |          |If switch target clock is stable, this bit will be set to 0.
     * |        |          |If switch target clock is not stable, this bit will be set to 1.
     * |        |          |Note2: This bit is read only.
     * |        |          |After selected clock source is stable, hardware will switch system clock to selected clock automatically, and CLK_SE_FAIL will be cleared automatically by hardware.
     * @var CLK_T::CLKSEL0
     * Offset: 0x10  Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HCLK_S    |HCLK Clock Source Selection (Write Protect)
     * |        |          |000 = Clock source is from HXT or LXT.
     * |        |          |001 = Reserved.
     * |        |          |010 = Reserved.
     * |        |          |011 = Clock source is from LIRC.
     * |        |          |111 = Clock source is from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note1: Before clock switching, the related clock sources (both pre-select and new-select) must be turn-on and stable.
     * |        |          |Note2: These bits are protected bit, and programming them needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA + 0x100.
     * |        |          |Note3: To set PWRCON[1:0], select HXT or LXT crystal clock.
     * |[5:3]   |STCLK_S   |Cortex-M0 SysTick Clock Source Selection From Reference Clock (Write Protect)
     * |        |          |If SYST_CSR[2] = 1, SysTick clock source is from HCLK.
     * |        |          |If SYST_CSR[2] = 0, SysTick clock source is defined by below settings.
     * |        |          |000 = Clock source is from HXT or LXT.
     * |        |          |001 = Reserved.
     * |        |          |010 = Clock source is from HXT/2 or LXT/2.
     * |        |          |011 = Clock source is from HCLK/2.
     * |        |          |111 = Clock source is from HIRC /2.
     * |        |          |Others = Reserved.
     * |        |          |Note1: These bits are protected bit, and programming them needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA + 0x100.
     * |        |          |Note2: If the SysTick clock source is not from HCLK (i.e. SYST_CSR[2] = 0), SysTick clock source must less than or equal to HCLK/2.
     * |        |          |Note3: To set PWRCON[1:0], select HXT or LXT crystal clock.
     * @var CLK_T::CLKSEL1
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDT_S     |WDT CLK Clock Source Selection (Write Protect)
     * |        |          |00 = Clock source is from HXT or LXT.
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source is from HCLK/2048 clock.
     * |        |          |11 = Clock source is from LIRC.
     * |        |          |Note1: These bits are the protected bit, and programming them needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA + 0x100.
     * |        |          |Note2: To set PWRCON[1:0], select HXT or LXT crystal clock.
     * |[3:2]   |ADC_S     |ADC Peripheral Clock Source Selection
     * |        |          |00 = Clock source is from HXT or LXT.
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source is from HCLK.
     * |        |          |11 = Clock source is from HIRC.
     * |        |          |Note: To set PWRCON[1:0], select HXT or LXT crystal clock.
     * |[4]     |SPI_S     |SPI Clock Source Selection
     * |        |          |0 = Clock source is from HXT or LXT.
     * |        |          |1 = Clock source is from HCLK.
     * |        |          |Note: To set PWRCON[1:0], select HXT or LXT crystal clock.
     * |[10:8]  |TMR0_S    |TIMER0 Clock Source Selection
     * |        |          |000 = Clock source is from HXT or LXT.
     * |        |          |001 = Clock source is from LIRC.
     * |        |          |010 = Clock source is from HCLK.
     * |        |          |011 = Clock source is from external trigger.
     * |        |          |111 = Clock source is from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note: To set PWRCON[1:0], select HXT or LXT crystal clock.
     * |[14:12] |TMR1_S    |TIMER1 Clock Source Selection
     * |        |          |000 = Clock source is from HXT or LXT.
     * |        |          |001 = Clock source is from LIRC.
     * |        |          |010 = Clock source is from HCLK.
     * |        |          |011 = Clock source is from external trigger.
     * |        |          |111 = Clock source is from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note: To set PWRCON[1:0], select HXT or LXT crystal clock.
     * |[25:24] |UART_S    |UART Clock Source Selection
     * |        |          |00 = Clock source is from HXT or LXT.
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source is from HIRC.
     * |        |          |11 = Clock source is from HIRC .
     * |        |          |Note: To set PWRCON[1:0], select HXT or LXT crystal clock.
     * @var CLK_T::CLKDIV
     * Offset: 0x18  Clock Divider Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |HCLK_N    |HCLK Clock Divide Number From HCLK Clock Source
     * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLK_N + 1).
     * |[11:8]  |UART_N    |UART Clock Divide Number From UART Clock Source
     * |        |          |UART clock frequency = (UART clock source frequency) / (UART_N + 1).
     * |[23:16] |ADC_N     |ADC Peripheral Clock Divide Number From ADC Peripheral Clock Source
     * |        |          |ADC peripheral clock frequency = (ADC peripheral clock source frequency) / (ADC_N + 1).
     * @var CLK_T::CLKSEL2
     * Offset: 0x1C  Clock Source Select Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:2]   |FRQDIV_S  |Clock Divider Clock Source Selection
     * |        |          |00 = Clock source is from HXT or LXT.
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source is from HCLK.
     * |        |          |11 = Clock source is from HIRC.
     * |        |          |Note: To set PWRCON[1:0], select HXT or LXT crystal clock.
     * @var CLK_T::FRQDIV
     * Offset: 0x24  Frequency Divider Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |FSEL      |Divider Output Frequency Selection
     * |        |          |The formula of output frequency is
     * |        |          |Fout = Fin/2(N+1),.
     * |        |          |Fin is the input clock frequency.
     * |        |          |Fout is the frequency of divider output clock.
     * |        |          |N is the 4-bit value of FSEL[3:0].
     * |[4]     |DIVIDER_EN|Frequency Divider Enable Control
     * |        |          |0 = Frequency Divider Disabled.
     * |        |          |1 = Frequency Divider Enabled.
     * |[5]     |DIVIDER1  |Frequency Divider 1 Enable Control
     * |        |          |0 = Divider output frequency is depended on FSEL value.
     * |        |          |1 = Divider output frequency is the same as input clock frequency.
     */

    __IO uint32_t PWRCON;        /* Offset: 0x00  System Power-down Control Register                                 */
    __IO uint32_t AHBCLK;        /* Offset: 0x04  AHB Devices Clock Enable Control Register                          */
    __IO uint32_t APBCLK;        /* Offset: 0x08  APB Devices Clock Enable Control Register                          */
    __IO uint32_t CLKSTATUS;     /* Offset: 0x0C  Clock Status Monitor Register                                      */
    __IO uint32_t CLKSEL0;       /* Offset: 0x10  Clock Source Select Control Register 0                             */
    __IO uint32_t CLKSEL1;       /* Offset: 0x14  Clock Source Select Control Register 1                             */
    __IO uint32_t CLKDIV;        /* Offset: 0x18  Clock Divider Number Register                                      */
    __IO uint32_t CLKSEL2;       /* Offset: 0x1C  Clock Source Select Control Register 2                             */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0[1];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t FRQDIV;        /* Offset: 0x24  Frequency Divider Control Register                                 */

} CLK_T;



/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_PWRCON_XTLCLK_EN_Pos         (0)                                               /*!< CLK_T::PWRCON: XTLCLK_EN Position         */
#define CLK_PWRCON_XTLCLK_EN_Msk         (0x3ul << CLK_PWRCON_XTLCLK_EN_Pos)               /*!< CLK_T::PWRCON: XTLCLK_EN Mask             */

#define CLK_PWRCON_OSC22M_EN_Pos         (2)                                               /*!< CLK_T::PWRCON: OSC22M_EN Position         */
#define CLK_PWRCON_OSC22M_EN_Msk         (0x1ul << CLK_PWRCON_OSC22M_EN_Pos)               /*!< CLK_T::PWRCON: OSC22M_EN Mask             */
#define CLK_PWRCON_IRC22M_EN_Pos         (2)                                               /*!< CLK_T::PWRCON: OSC22M_EN Position         */
#define CLK_PWRCON_IRC22M_EN_Msk         (0x1ul << CLK_PWRCON_IRC22M_EN_Pos)               /*!< CLK_T::PWRCON: OSC22M_EN Mask             */
#define CLK_PWRCON_HIRC_EN_Pos           (2)                                               /*!< CLK_T::PWRCON: OSC22M_EN Position         */
#define CLK_PWRCON_HIRC_EN_Msk           (0x1ul << CLK_PWRCON_HIRC_EN_Pos)                 /*!< CLK_T::PWRCON: OSC22M_EN Mask             */

#define CLK_PWRCON_OSC10K_EN_Pos         (3)                                               /*!< CLK_T::PWRCON: OSC10K_EN Position         */
#define CLK_PWRCON_OSC10K_EN_Msk         (0x1ul << CLK_PWRCON_OSC10K_EN_Pos)               /*!< CLK_T::PWRCON: OSC10K_EN Mask             */
#define CLK_PWRCON_IRC10K_EN_Pos         (3)                                               /*!< CLK_T::PWRCON: OSC10K_EN Position         */
#define CLK_PWRCON_IRC10K_EN_Msk         (0x1ul << CLK_PWRCON_IRC10K_EN_Pos)               /*!< CLK_T::PWRCON: OSC10K_EN Mask             */
#define CLK_PWRCON_LIRC_EN_Pos           (3)                                               /*!< CLK_T::PWRCON: OSC10K_EN Position         */
#define CLK_PWRCON_LIRC_EN_Msk           (0x1ul << CLK_PWRCON_LIRC_EN_Pos)                 /*!< CLK_T::PWRCON: OSC10K_EN Mask             */

#define CLK_PWRCON_WU_DLY_Pos            (4)                                               /*!< CLK_T::PWRCON: PD_WU_DLY Position         */
#define CLK_PWRCON_WU_DLY_Msk            (0x1ul << CLK_PWRCON_WU_DLY_Pos)                  /*!< CLK_T::PWRCON: PD_WU_DLY Mask             */

#define CLK_PWRCON_WINT_EN_Pos           (5)                                               /*!< CLK_T::PWRCON: PD_WU_INT_EN Position      */
#define CLK_PWRCON_WINT_EN_Msk           (0x1ul << CLK_PWRCON_WINT_EN_Pos)                 /*!< CLK_T::PWRCON: PD_WU_INT_EN Mask          */

#define CLK_PWRCON_PD_WU_STS_Pos         (6)                                               /*!< CLK_T::PWRCON: PD_WU_STS Position         */
#define CLK_PWRCON_PD_WU_STS_Msk         (0x1ul << CLK_PWRCON_PD_WU_STS_Pos)               /*!< CLK_T::PWRCON: PD_WU_STS Mask             */

#define CLK_PWRCON_PWR_DOWN_EN_Pos       (7)                                               /*!< CLK_T::PWRCON: PWR_DOWN_EN Position       */
#define CLK_PWRCON_PWR_DOWN_EN_Msk       (0x1ul << CLK_PWRCON_PWR_DOWN_EN_Pos)             /*!< CLK_T::PWRCON: PWR_DOWN_EN Mask           */

#define CLK_PWRCON_PD_32K_Pos            (9)                                               /*!< CLK_T::PWRCON: PD_32K Position            */
#define CLK_PWRCON_PD_32K_Msk            (0x1ul << CLK_PWRCON_PD_32K_Pos)                  /*!< CLK_T::PWRCON: PD_32K Mask                */

#define CLK_AHBCLK_ISP_EN_Pos            (2)                                               /*!< CLK_T::AHBCLK: ISP_EN Position            */
#define CLK_AHBCLK_ISP_EN_Msk            (0x1ul << CLK_AHBCLK_ISP_EN_Pos)                  /*!< CLK_T::AHBCLK: ISP_EN Mask                */

#define CLK_APBCLK_WDT_EN_Pos            (0)                                               /*!< CLK_T::APBCLK: WDT_EN Position            */
#define CLK_APBCLK_WDT_EN_Msk            (0x1ul << CLK_APBCLK_WDT_EN_Pos)                  /*!< CLK_T::APBCLK: WDT_EN Mask                */

#define CLK_APBCLK_TMR0_EN_Pos           (2)                                               /*!< CLK_T::APBCLK: TMR0_EN Position           */
#define CLK_APBCLK_TMR0_EN_Msk           (0x1ul << CLK_APBCLK_TMR0_EN_Pos)                 /*!< CLK_T::APBCLK: TMR0_EN Mask               */

#define CLK_APBCLK_TMR1_EN_Pos           (3)                                               /*!< CLK_T::APBCLK: TMR1_EN Position           */
#define CLK_APBCLK_TMR1_EN_Msk           (0x1ul << CLK_APBCLK_TMR1_EN_Pos)                 /*!< CLK_T::APBCLK: TMR1_EN Mask               */

#define CLK_APBCLK_FDIV_EN_Pos           (6)                                               /*!< CLK_T::APBCLK: FDIV_EN Position           */
#define CLK_APBCLK_FDIV_EN_Msk           (0x1ul << CLK_APBCLK_FDIV_EN_Pos)                 /*!< CLK_T::APBCLK: FDIV_EN Mask               */

#define CLK_APBCLK_I2C_EN_Pos            (8)                                               /*!< CLK_T::APBCLK: I2C_EN Position            */
#define CLK_APBCLK_I2C_EN_Msk            (0x1ul << CLK_APBCLK_I2C_EN_Pos)                  /*!< CLK_T::APBCLK: I2C_EN Mask                */

#define CLK_APBCLK_SPI_EN_Pos            (12)                                              /*!< CLK_T::APBCLK: SPI_EN Position            */
#define CLK_APBCLK_SPI_EN_Msk            (0x1ul << CLK_APBCLK_SPI_EN_Pos)                  /*!< CLK_T::APBCLK: SPI_EN Mask                */

#define CLK_APBCLK_UART_EN_Pos           (16)                                              /*!< CLK_T::APBCLK: UART_EN Position           */
#define CLK_APBCLK_UART_EN_Msk           (0x1ul << CLK_APBCLK_UART_EN_Pos)                 /*!< CLK_T::APBCLK: UART_EN Mask               */

#define CLK_APBCLK_PWM01_EN_Pos          (20)                                              /*!< CLK_T::APBCLK: PWM01_EN Position          */
#define CLK_APBCLK_PWM01_EN_Msk          (0x1ul << CLK_APBCLK_PWM01_EN_Pos)                /*!< CLK_T::APBCLK: PWM01_EN Mask              */

#define CLK_APBCLK_PWM23_EN_Pos          (21)                                              /*!< CLK_T::APBCLK: PWM23_EN Position          */
#define CLK_APBCLK_PWM23_EN_Msk          (0x1ul << CLK_APBCLK_PWM23_EN_Pos)                /*!< CLK_T::APBCLK: PWM23_EN Mask              */

#define CLK_APBCLK_PWM45_EN_Pos          (22)                                              /*!< CLK_T::APBCLK: PWM45_EN Position          */
#define CLK_APBCLK_PWM45_EN_Msk          (0x1ul << CLK_APBCLK_PWM45_EN_Pos)                /*!< CLK_T::APBCLK: PWM45_EN Mask              */

#define CLK_APBCLK_ADC_EN_Pos            (28)                                              /*!< CLK_T::APBCLK: ADC_EN Position            */
#define CLK_APBCLK_ADC_EN_Msk            (0x1ul << CLK_APBCLK_ADC_EN_Pos)                  /*!< CLK_T::APBCLK: ADC_EN Mask                */

#define CLK_APBCLK_CMP_EN_Pos            (30)                                              /*!< CLK_T::APBCLK: ACMP_EN Position           */
#define CLK_APBCLK_CMP_EN_Msk            (0x1ul << CLK_APBCLK_CMP_EN_Pos)                  /*!< CLK_T::APBCLK: ACMP_EN Mask               */

#define CLK_CLKSTATUS_XTL_STB_Pos        (0)                                               /*!< CLK_T::CLKSTATUS: XTL_STB Position        */
#define CLK_CLKSTATUS_XTL_STB_Msk        (0x1ul << CLK_CLKSTATUS_XTL_STB_Pos)              /*!< CLK_T::CLKSTATUS: XTL_STB Mask            */
#define CLK_CLKSTATUS_HXT_STB_Pos        (0)                                               /*!< CLK_T::CLKSTATUS: XTL_STB Position        */
#define CLK_CLKSTATUS_HXT_STB_Msk        (0x1ul << CLK_CLKSTATUS_HXT_STB_Pos)              /*!< CLK_T::CLKSTATUS: XTL_STB Mask            */
#define CLK_CLKSTATUS_LXT_STB_Pos        (0)                                               /*!< CLK_T::CLKSTATUS: XTL_STB Position        */
#define CLK_CLKSTATUS_LXT_STB_Msk        (0x1ul << CLK_CLKSTATUS_LXT_STB_Pos)              /*!< CLK_T::CLKSTATUS: XTL_STB Mask            */

#define CLK_CLKSTATUS_OSC10K_STB_Pos     (3)                                               /*!< CLK_T::CLKSTATUS: OSC10K_STB Position     */
#define CLK_CLKSTATUS_OSC10K_STB_Msk     (0x1ul << CLK_CLKSTATUS_OSC10K_STB_Pos)           /*!< CLK_T::CLKSTATUS: OSC10K_STB Mask         */
#define CLK_CLKSTATUS_IRC10K_STB_Pos     (3)                                               /*!< CLK_T::CLKSTATUS: OSC10K_STB Position     */
#define CLK_CLKSTATUS_IRC10K_STB_Msk     (0x1ul << CLK_CLKSTATUS_OSC10K_STB_Pos)           /*!< CLK_T::CLKSTATUS: OSC10K_STB Mask         */
#define CLK_CLKSTATUS_LIRC_STB_Pos       (3)                                               /*!< CLK_T::CLKSTATUS: OSC10K_STB Position     */
#define CLK_CLKSTATUS_LIRC_STB_Msk       (0x1ul << CLK_CLKSTATUS_LIRC_STB_Pos)             /*!< CLK_T::CLKSTATUS: OSC10K_STB Mask         */

#define CLK_CLKSTATUS_OSC22M_STB_Pos     (4)                                               /*!< CLK_T::CLKSTATUS: OSC22M_STB Position     */
#define CLK_CLKSTATUS_OSC22M_STB_Msk     (0x1ul << CLK_CLKSTATUS_OSC22M_STB_Pos)           /*!< CLK_T::CLKSTATUS: OSC22M_STB Mask         */
#define CLK_CLKSTATUS_IRC22M_STB_Pos     (4)                                               /*!< CLK_T::CLKSTATUS: OSC22M_STB Position     */
#define CLK_CLKSTATUS_IRC22M_STB_Msk     (0x1ul << CLK_CLKSTATUS_OSC22M_STB_Pos)           /*!< CLK_T::CLKSTATUS: OSC22M_STB Mask         */
#define CLK_CLKSTATUS_HIRC_STB_Pos       (4)                                               /*!< CLK_T::CLKSTATUS: OSC22M_STB Position     */
#define CLK_CLKSTATUS_HIRC_STB_Msk       (0x1ul << CLK_CLKSTATUS_HIRC_STB_Pos)             /*!< CLK_T::CLKSTATUS: OSC22M_STB Mask         */

#define CLK_CLKSTATUS_CLK_SW_FAIL_Pos    (7)                                               /*!< CLK_T::CLKSTATUS: CLK_SW_FAIL Position    */
#define CLK_CLKSTATUS_CLK_SW_FAIL_Msk    (0x1ul << CLK_CLKSTATUS_CLK_SW_FAIL_Pos)          /*!< CLK_T::CLKSTATUS: CLK_SW_FAIL Mask        */

#define CLK_CLKSEL0_HCLK_S_Pos           (0)                                               /*!< CLK_T::CLKSEL0: HCLK_S Position           */
#define CLK_CLKSEL0_HCLK_S_Msk           (0x7ul << CLK_CLKSEL0_HCLK_S_Pos)                 /*!< CLK_T::CLKSEL0: HCLK_S Mask               */

#define CLK_CLKSEL0_STCLK_S_Pos          (3)                                               /*!< CLK_T::CLKSEL0: STCLK_S Position          */
#define CLK_CLKSEL0_STCLK_S_Msk          (0x7ul << CLK_CLKSEL0_STCLK_S_Pos)                /*!< CLK_T::CLKSEL0: STCLK_S Mask              */

#define CLK_CLKSEL1_WDT_S_Pos            (0)                                               /*!< CLK_T::CLKSEL1: WDT_S Position            */
#define CLK_CLKSEL1_WDT_S_Msk            (0x3ul << CLK_CLKSEL1_WDT_S_Pos)                  /*!< CLK_T::CLKSEL1: WDT_S Mask                */

#define CLK_CLKSEL1_ADC_S_Pos            (2)                                               /*!< CLK_T::CLKSEL1: ADC_S Position            */
#define CLK_CLKSEL1_ADC_S_Msk            (0x3ul << CLK_CLKSEL1_ADC_S_Pos)                  /*!< CLK_T::CLKSEL1: ADC_S Mask                */

#define CLK_CLKSEL1_SPI_S_Pos            (4)                                               /*!< CLK_T::CLKSEL1: SPI_S Position            */
#define CLK_CLKSEL1_SPI_S_Msk            (0x1ul << CLK_CLKSEL1_SPI_S_Pos)                  /*!< CLK_T::CLKSEL1: SPI_S Mask                */

#define CLK_CLKSEL1_TMR0_S_Pos           (8)                                               /*!< CLK_T::CLKSEL1: TMR0_S Position           */
#define CLK_CLKSEL1_TMR0_S_Msk           (0x7ul << CLK_CLKSEL1_TMR0_S_Pos)                 /*!< CLK_T::CLKSEL1: TMR0_S Mask               */

#define CLK_CLKSEL1_TMR1_S_Pos           (12)                                              /*!< CLK_T::CLKSEL1: TMR1_S Position           */
#define CLK_CLKSEL1_TMR1_S_Msk           (0x7ul << CLK_CLKSEL1_TMR1_S_Pos)                 /*!< CLK_T::CLKSEL1: TMR1_S Mask               */

#define CLK_CLKSEL1_UART_S_Pos           (24)                                              /*!< CLK_T::CLKSEL1: UART_S Position           */
#define CLK_CLKSEL1_UART_S_Msk           (0x3ul << CLK_CLKSEL1_UART_S_Pos)                 /*!< CLK_T::CLKSEL1: UART_S Mask               */

#define CLK_CLKDIV_HCLK_N_Pos            (0)                                               /*!< CLK_T::CLKDIV: HCLK_N Position            */
#define CLK_CLKDIV_HCLK_N_Msk            (0xful << CLK_CLKDIV_HCLK_N_Pos)                  /*!< CLK_T::CLKDIV: HCLK_N Mask                */

#define CLK_CLKDIV_UART_N_Pos            (8)                                               /*!< CLK_T::CLKDIV: UART_N Position            */
#define CLK_CLKDIV_UART_N_Msk            (0xful << CLK_CLKDIV_UART_N_Pos)                  /*!< CLK_T::CLKDIV: UART_N Mask                */

#define CLK_CLKDIV_ADC_N_Pos             (16)                                              /*!< CLK_T::CLKDIV: ADC_N Position             */
#define CLK_CLKDIV_ADC_N_Msk             (0xfful << CLK_CLKDIV_ADC_N_Pos)                  /*!< CLK_T::CLKDIV: ADC_N Mask                 */

#define CLK_CLKSEL2_FRQDIV_S_Pos         (2)                                               /*!< CLK_T::CLKSEL2: FRQDIV_S Position         */
#define CLK_CLKSEL2_FRQDIV_S_Msk         (0x3ul << CLK_CLKSEL2_FRQDIV_S_Pos)               /*!< CLK_T::CLKSEL2: FRQDIV_S Mask             */

#define CLK_FRQDIV_FSEL_Pos              (0)                                               /*!< CLK_T::FRQDIV: FSEL Position              */
#define CLK_FRQDIV_FSEL_Msk              (0xful << CLK_FRQDIV_FSEL_Pos)                    /*!< CLK_T::FRQDIV: FSEL Mask                  */

#define CLK_FRQDIV_DIVIDER_EN_Pos        (4)                                               /*!< CLK_T::FRQDIV: DIVIDER_EN Position        */
#define CLK_FRQDIV_DIVIDER_EN_Msk        (0x1ul << CLK_FRQDIV_DIVIDER_EN_Pos)              /*!< CLK_T::FRQDIV: DIVIDER_EN Mask            */

#define CLK_FRQDIV_DIVIDER1_Pos          (5)                                               /*!< CLK_T::FRQDIV: DIVIDER1 Position          */
#define CLK_FRQDIV_DIVIDER1_Msk          (0x1ul << CLK_FRQDIV_DIVIDER1_Pos)                /*!< CLK_T::FRQDIV: DIVIDER1 Mask              */

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */


/*---------------------- Flash Memory Controller -------------------------*/
/**
    @addtogroup FMC Flash Memory Controller(FMC)
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
     * |[0]     |ISPEN     |ISP Enable Control (Write Protect)
     * |        |          |Set this bit to enable ISP function.
     * |        |          |0 = ISP function Disabled.
     * |        |          |1 = ISP function Enabled.
     * |[1]     |BS        |Boot Select (Write Protect)
     * |        |          |Set/clear this bit to select next booting from LDROM/APROM, respectively.
     * |        |          |This bit also functions as chip booting status flag, which can be used to check where chip booted from.
     * |        |          |This bit is initiated with the inversed value of CBS in CONFIG0 after any reset is happened except CPU reset (RSTS_CPU is 1) or system reset (RSTS_SYS) is happened.
     * |        |          |0 = Boot from APROM.
     * |        |          |1 = Boot from LDROM.
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
     * |        |          |(1) APROM writes to itself if APUEN is set to 0 or CBS[0]=1.
     * |        |          |(2) LDROM writes to itself if LDUEN is set to 0 or CBS[0]=1.
     * |        |          |(3) User Configuration is erased/programmed when CFGUEN is 0.
     * |        |          |(4) Destination address is illegal, such as over an available range.
     * |        |          |Note: Write 1 to clear this bit to 0.
     * @var FMC_T::ISPADR
     * Offset: 0x04  ISP Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPADR    |ISP Address
     * |        |          |The NuMicro Mini51TM series supports word program only. ISPADR[1:0] must be kept 00 for ISP operation.
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
     * |[5:0]   |ISPCMD    |ISP Command
     * |        |          |ISP commands are shown below:
     * |        |          |0x00 = Read.
     * |        |          |0x04 = Read Unique ID.
     * |        |          |0x0B = Read Company ID (0xDA).
     * |        |          |0x21 = Program.
     * |        |          |0x22 = Page Erase.
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
     * @var FMC_T::DFBADR
     * Offset: 0x14  Data Flash Start Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DFBA      |Data Flash Base Address
     * |        |          |This register indicates Data Flash start address. It is a read only register.
     * |        |          |The Data Flash start address is defined by user.
     * |        |          |Since on chip flash erase unit is 512 bytes, it is mandatory to keep bit 8-0 as 0.
     * @var FMC_T::ISPSTA
     * Offset: 0x40  ISP Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP Start Trigger (Read Only)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP operation is progressed.
     * |        |          |Note: This bit is the same with ISPTRG bit 0.
     * |[2:1]   |CBS       |Config Boot Selection (Read Only)
     * |        |          |This is a mirror of CBS in CONFIG0.
     * |[6]     |ISPFF     |ISP Fail Flag (Write Protect)
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself if APUEN is set to 0 or CBS[0]=1.
     * |        |          |(2) LDROM writes to itself if LDUEN is set to 0 or CBS[0]=1.
     * |        |          |(3) User Configuration is erased/programmed when CFGUEN is 0.
     * |        |          |(4) Destination address is illegal, such as over an available range.
     * |        |          |Write 1 to clear.
     * |        |          |Note: This bit functions the same as ISPCON bit 6.
     * |[20:9]  |VECMAP    |Vector Page Mapping Address (Read Only)
     * |        |          |The current flash address space 0x0000_0000~0x0000_01FF is mapping to address {VECMAP[11:0], 9'h000} ~ {VECMAP[11:0], 9'h1FF}.
     */

    __IO uint32_t ISPCON;        /* Offset: 0x00  ISP Control Register                                               */
    __IO uint32_t ISPADR;        /* Offset: 0x04  ISP Address Register                                               */
    __IO uint32_t ISPDAT;        /* Offset: 0x08  ISP Data Register                                                  */
    __IO uint32_t ISPCMD;        /* Offset: 0x0C  ISP Command Register                                               */
    __IO uint32_t ISPTRG;        /* Offset: 0x10  ISP Trigger Register                                               */
    __I  uint32_t DFBADR;        /* Offset: 0x14  Data Flash Start Address                                           */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0[10];
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t ISPSTA;        /* Offset: 0x40  ISP Status Register                                                */

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

#define FMC_ISPCMD_ISPCMD_Pos            (0)                                               /*!< FMC_T::ISPCMD: ISPCMD Position            */
#define FMC_ISPCMD_ISPCMD_Msk            (0x3ful << FMC_ISPCMD_ISPCMD_Pos)                 /*!< FMC_T::ISPCMD: ISPCMD Mask                */

#define FMC_ISPTRG_ISPGO_Pos             (0)                                               /*!< FMC_T::ISPTRG: ISPGO Position             */
#define FMC_ISPTRG_ISPGO_Msk             (0x1ul << FMC_ISPTRG_ISPGO_Pos)                   /*!< FMC_T::ISPTRG: ISPGO Mask                 */

#define FMC_DFBADR_DFBA_Pos                (0)                                               /*!< FMC_T::DFBADR: DFBA Position                */
#define FMC_DFBADR_DFBA_Msk                (0xfffffffful << FMC_DFBA_DFBA_Pos)               /*!< FMC_T::DFBADR: DFBA Mask                    */

#define FMC_ISPSTA_ISPGO_Pos             (0)                                               /*!< FMC_T::ISPSTA: ISPGO Position             */
#define FMC_ISPSTA_ISPGO_Msk             (0x1ul << FMC_ISPSTA_ISPGO_Pos)                   /*!< FMC_T::ISPSTA: ISPGO Mask                 */

#define FMC_ISPSTA_CBS_Pos               (1)                                               /*!< FMC_T::ISPSTA: CBS Position               */
#define FMC_ISPSTA_CBS_Msk               (0x3ul << FMC_ISPSTA_CBS_Pos)                     /*!< FMC_T::ISPSTA: CBS Mask                   */

#define FMC_ISPSTA_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPSTA: ISPFF Position             */
#define FMC_ISPSTA_ISPFF_Msk             (0x1ul << FMC_ISPSTA_ISPFF_Pos)                   /*!< FMC_T::ISPSTA: ISPFF Mask                 */

#define FMC_ISPSTA_VECMAP_Pos            (9)                                               /*!< FMC_T::ISPSTA: VECMAP Position            */
#define FMC_ISPSTA_VECMAP_Msk            (0xffful << FMC_ISPSTA_VECMAP_Pos)                /*!< FMC_T::ISPSTA: VECMAP Mask                */

/**@}*/ /* FMC_CONST */
/**@}*/ /* end of FMC register group */


/*---------------------- System Global Control Registers -------------------------*/
/**
    @addtogroup GCR System Global Control Registers(GCR)
    Memory Mapped Structure for GCR Controller
@{ */


typedef struct
{

    /**
     * @var GCR_T::PDID
     * Offset: 0x00  Part Device Identification Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDID      |Product Device Identification Number
     * |        |          |This register reflects the device part number code.
     * |        |          |Software can read this register to identify which device is used.
     * |        |          |For example, the MINI51LDE PDID code is "0x20205100".
     * @var GCR_T::RSTSRC
     * Offset: 0x04  System Reset Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RSTS_POR  |Power-on Reset Flag
     * |        |          |The RSTS_POR flag is set by the "reset signal", which is from the Power-On Reset (POR) controller or bit CHIP_RST (IPRSTC1[0]), to indicate the previous reset source.
     * |        |          |0 = No reset from POR or CHIP_RST.
     * |        |          |1 = Power-on-Reset (POR) or CHIP_RST had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to 0.
     * |[1]     |RSTS_RESET|Reset Pin Reset Flag
     * |        |          |The RSTS_RESET flag is set by the "reset signal" from the /RESET pin to indicate the previous reset source.
     * |        |          |0 = No reset from pin /RESET pin.
     * |        |          |1 = The /RESET pin had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to 0.
     * |[2]     |RSTS_WDT  |Watchdog Reset Flag
     * |        |          |The RSTS_WDT flag is set by the "reset signal" from the Watchdog timer to indicate the previous reset source.
     * |        |          |0 = No reset from Watchdog timer.
     * |        |          |1 = The Watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to 0.
     * |[4]     |RSTS_BOD  |Brown-out Detector Reset Flag
     * |        |          |The RSTS_BOD flag is set by the "reset signal" from the Brown-out Detector to indicate the previous reset source.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to 0.
     * |[5]     |RSTS_MCU  |MCU Reset Flag
     * |        |          |The RSTS_MCU flag is set by the "reset signal" from the Cortex-M0 core to indicate the previous reset source.
     * |        |          |0 = No reset from Cortex-M0.
     * |        |          |1 = The Cortex-M0 had issued the reset signal to reset the system by writing 1 to bit SYSRESETREQ (AIRCR[2]), Application Interrupt and Reset Control Register, address = 0xE000ED0C in system control registers of Cortex-M0 core.
     * |        |          |Note: Software can write 1 to clear this bit to 0.
     * |[7]     |RSTS_CPU  |CPU Reset Flag
     * |        |          |The RSTS_CPU flag is set by hardware if software writes CPU_RST (IPRSTC1[1]) 1 to reset Cortex-M0 core and Flash memory controller (FMC).
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = Cortex-M0 core and FMC are reset by software setting CPU_RST to 1.
     * |        |          |Note: Software can write 1 to clear this bit to 0.
     * @var GCR_T::IPRSTC1
     * Offset: 0x08  Peripheral Reset Control Resister 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIP_RST  |CHIP One-shot Reset (Write Protect)
     * |        |          |Setting this bit will reset the CHIP, including CPU kernel and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |The CHIP_RST is the same as the POR reset, and all the chip module is reset and the chip settings from flash are also reloaded.
     * |        |          |0 = Chip normal operation.
     * |        |          |1 = CHIP one-shot reset.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA + 0x100.
     * |[1]     |CPU_RST   |CPU Kernel Reset
     * |        |          |Setting this bit will reset the CPU kernel, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |0 = CPU normal operation.
     * |        |          |1 = Reset CPU Kernel.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA + 0x100.
     * @var GCR_T::IPRSTC2
     * Offset: 0x0C  Peripheral Reset Control Resister 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |GPIO_RST  |GPIO (P0~P5) Controller Reset
     * |        |          |0 = GPIO module normal operation.
     * |        |          |1 = GPIO module reset.
     * |[2]     |TMR0_RST  |Timer0 Controller Reset
     * |        |          |0 = Timer0 module normal operation.
     * |        |          |1 = Timer0 module reset.
     * |[3]     |TMR1_RST  |Timer1 Controller Reset
     * |        |          |0 = Timer1 module normal operation.
     * |        |          |1 = Timer1 module reset.
     * |[8]     |I2C_RST   |I2C Controller Reset
     * |        |          |0 = I2C module normal operation.
     * |        |          |1 = I2C module reset.
     * |[12]    |SPI_RST   |SPI Controller Reset
     * |        |          |0 = SPI module normal operation.
     * |        |          |1 = SPI module reset.
     * |[16]    |UART_RST  |UART Controller Reset
     * |        |          |0 = UART module normal operation.
     * |        |          |1 = UART module reset.
     * |[20]    |PWM_RST   |PWM Controller Reset
     * |        |          |0 = PWM module normal operation.
     * |        |          |1 = PWM module reset.
     * |[22]    |ACMP_RST  |ACMP Controller Reset
     * |        |          |0 = ACMP module normal operation.
     * |        |          |1 = ACMP module reset.
     * |[28]    |ADC_RST   |ADC Controller Reset
     * |        |          |0 = ADC module normal operation.
     * |        |          |1 = ADC module reset.
     * @var GCR_T::BODCTL
     * Offset: 0x18  Brown-out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BOD_VL_EXT|Brown-out Detector Selection Extension (Initiated & Write-protected Bit)
     * |        |          |The default value is set by flash controller user configuration CBOVEXT bit (config0 [23]).
     * |        |          |If config0 bit[23] is set to 1, default value of BOD_VL_EXT is 0.
     * |        |          |If config0 bit[23] is set to 0, default value of BOD_VL_EXT is 1.
     * |        |          |0 = Brown-out detector threshold voltage is selected by the table defined in BOD_VL.
     * |        |          |1 = Brown-out detector threshold voltage is selected by the table defined below.
     * |        |          |11 = 4.4V
     * |        |          |10 = 3.7V
     * |        |          |01 = 2.7V
     * |        |          |00 = 2.2V
     * |[2:1]   |BOD_VL    |Brown-out Detector Threshold Voltage Selection (Initiated & Write-protected Bit)
     * |        |          |The default value is set by flash controller user configuration CBOV bit (config0 [22:21]).
     * |        |          |BOD_VL[1]
     * |        |          |BOD_VL[0]
     * |        |          |Brown-out   voltage
     * |        |          |11 = Disable
     * |        |          |10 = 3.7V
     * |        |          |01 = 2.7V
     * |        |          |00 = Reserved
     * |[3]     |BOD_RSTEN |Brown-out Reset Enable Control (Initiated And Write-protected Bit)
     * |        |          |0 = Brown-out "INTERRUPT" function Enabled; when the Brown-out Detector function is enable and the detected voltage is lower than the threshold, then assert a signal to interrupt the Cortex-M0 CPU.
     * |        |          |1 = Brown-out "RESET" function Enabled; when the Brown-out Detector function is enable and the detected voltage is lower than the threshold then assert a signal to reset the chip.
     * |        |          |The default value is set by flash controller user configuration register config0 bit[20].
     * |        |          |When the BOD_EN is enabled and the interrupt is asserted, the interrupt will be kept till the BOD_EN is set to 0.
     * |        |          |The interrupt for CPU can be blocked by disabling the NVIC in CPU for BOD interrupt or disable the interrupt source by disabling the BOD_EN and then re-enabling the BOD_EN function if the BOD function is required.
     * |[4]     |BOD_INTF  |Brown-out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage dropped at AVDD down through or up through the voltage of BOD_VL setting.
     * |        |          |1 = When Brown-out Detector detects the AVDD is dropped through the voltage of BOD_VL setting or the AVDD is raised up through the voltage of BOD_VL setting, this bit is set to 1 and the Brown-out interrupt is requested if Brown-out interrupt is enabled.
     * |[5]     |BOD_LPM   |Brown-out Detector Low Power Mode (Write Protect)
     * |        |          |0 = BOD operate in normal mode (default).
     * |        |          |1 = Enable the BOD low power mode.
     * |        |          |The BOD consumes about 100uA in normal mode, the low power mode can reduce the current to about 1uA but slow the BOD response.
     * |[6]     |BOD_OUT   |Brown-out Detector Output State
     * |        |          |0 = Brown-out Detector status output is 0, the detected voltage is higher than BOD_VL setting.
     * |        |          |1 = Brown-out Detector status output is 1, the detected voltage is lower than BOD_VL setting.
     * @var GCR_T::P0_MFP
     * Offset: 0x30  P0 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P0_MFP    |P0 Multiple Function Selection
     * |[8]     |P0_ALT0   |P0.0 Alternate Function Selection
     * |[9]     |P0_ALT1   |P0.1 Alternate Function Selection
     * |[12]    |P0_ALT4   |P0.4 Alternate Function Selection
     * |[13]    |P0_ALT5   |P0.5 Alternate Function Selection
     * |[14]    |P0_ALT6   |P0.6 Alternate Function Selection
     * |[15]    |P0_ALT7   |P0.7 Alternate Function Selection
     * |[23:16] |P0_TYPE   |P0[7:0] TTL Or Schmitt Trigger Function Enable Control
     * |        |          |0 = P0[7:0]Select I/O input as TTL function.
     * |        |          |1 = P0[7:0] Select I/O input as Schmitt Trigger function .
     * @var GCR_T::P1_MFP
     * Offset: 0x34  P1 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P1_MFP    |P1 Multiple Function Selection
     * |[8]     |P1_ALT0   |P1.0 Alternate Function Selection
     * |[10]    |P1_ALT2   |P1.2 Alternate Function Selection
     * |[11]    |P1_ALT3   |P1.3 Alternate Function Selection
     * |[12]    |P1_ALT4   |P1.4 Alternate Function Selection
     * |[13]    |P1_ALT5   |P1.5 Alternate Function Selection
     * |[23:16] |P1_TYPE   |P1[7:0] TTL Or Schmitt Trigger Function Enable Control
     * |        |          |0 = P1[7:0]Select I/O input as TTL function.
     * |        |          |1 = P1[7:0] Select I/O input as Schmitt Trigger function .
     * @var GCR_T::P2_MFP
     * Offset: 0x38  P2 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P2_MFP    |P2 Multiple Function Selection
     * |[10]    |P2_ALT2   |P2.2 Alternate Function Selection
     * |[11]    |P2_ALT3   |P2.3 Alternate Function Selection
     * |[12]    |P2_ALT4   |P2.4 Alternate Function Selection
     * |[13]    |P2_ALT5   |P2.5 Alternate Function Selection
     * |[14]    |P2_ALT6   |P2.6 Alternate Function Selection
     * |[23:16] |P2_TYPE   |P2[7:0] TTL Or Schmitt Trigger Function Enable Control
     * |        |          |0 = P2[7:0]Select I/O input as TTL function.
     * |        |          |1 = P2[7:0] Select I/O input as Schmitt Trigger function .
     * @var GCR_T::P3_MFP
     * Offset: 0x3C  P3 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P3_MFP    |P3 Multiple Function Selection
     * |[8]     |P3_ALT0   |P3.0 Alternate Function Selection
     * |[9]     |P3_ALT1   |P3.1 Alternate Function Selection
     * |[10]    |P3_ALT2   |P3.2 Alternate Function Selection
     * |[12]    |P3_ALT4   |P3.4 Alternate Function Selection
     * |[13]    |P3_ALT5   |P3.5 Alternate Function Selection
     * |[14]    |P3_ALT6   |P3.6 Alternate Function Selection
     * |[23:16] |P3_TYPE   |P3[7:0] TTL Or Schmitt Trigger Function Enable Control
     * |        |          |0 = P3[7:0]Select I/O input as TTL function.
     * |        |          |1 = P3[7:0] Select I/O input as Schmitt Trigger function .
     * |[24]    |P32CPP1   |P3.2 Alternate Function Selection Extension
     * @var GCR_T::P4_MFP
     * Offset: 0x40  P4 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P4_MFP    |P4 Multiple Function Selection
     * |[14]    |P4_ALT6   |P4.6 Alternate Function Selection
     * |[15]    |P4_ALT7   |P4.7 Alternate Function Selection
     * |[23:16] |P4_TYPE   |P4[7:0] TTL Or Schmitt Trigger Function Enable Control
     * |        |          |0 = P4[7:0]Select I/O input as TTL function.
     * |        |          |1 = P4[7:0] Select I/O input as Schmitt Trigger function .
     * @var GCR_T::P5_MFP
     * Offset: 0x44  P5 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |P5_MFP    |P5 Multiple Function Selection
     * |[8]     |P5_ALT0   |P5.0 Alternate Function Selection
     * |[9]     |P5_ALT1   |P5.1 Alternate Function Selection
     * |[10]    |P5_ALT2   |P5.2 Alternate Function Selection
     * |[11]    |P5_ALT3   |P5.3 Alternate Function Selection
     * |[12]    |P5_ALT4   |P5.4 Alternate Function Selection
     * |[13]    |P5_ALT5   |P5.5 Alternate Function Selection
     * |[23:16] |P5_TYPE   |P5[7:0] TTL Or Schmitt Trigger Function Enable Control
     * |        |          |0 = P5[7:0]Select I/O input as TTL function.
     * |        |          |1 = P5[7:0] Select I/O input as Schmitt Trigger function .
     * @var GCR_T::IRCTRIMCTL
     * Offset: 0x80  HIRC Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TRIM_SEL  |Trim Frequency Selection
     * |        |          |This bit is to enable the HIRC auto trim.
     * |        |          |When setting this bit to 1, the HIRC auto trim function will trim HIRC to 22.1184 MHz automatically based on the LXT reference clock.
     * |        |          |During auto trim operation, if LXT clock error is detected or trim retry limitation count reached, this field will be cleared to 0 automatically.
     * |        |          |0 = HIRC auto trim function Disabled.
     * |        |          |1 = HIRC auto trim function Enabled and HIRC trimmed to 22.1184 MHz.
     * |[5:4]   |TRIM_LOOP |Trim Calculation Loop
     * |        |          |This field defines that trim value calculation is based on how many LXT clocks in.
     * |        |          |For example, if TRIM_LOOP is set as 00, auto trim circuit will calculate trim value based on the average frequency difference in 4 LXT clock.
     * |        |          |00 = Trim value calculation is based on average difference in 4 LXT clocks.
     * |        |          |01 = Trim value calculation is based on average difference in 8 LXT clocks.
     * |        |          |10 = Trim value calculation is based on average difference in 16 LXT clocks.
     * |        |          |11 = Trim value calculation is based on average difference in 32 LXT clocks.
     * @var GCR_T::IRCTRIMIER
     * Offset: 0x84  HIRC Trim Interrupt Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TRIM_FAIL_IEN|Trim Failure Interrupt Enable Control
     * |        |          |This bit controls if an interrupt will be triggered while HIRC trim value update limitation count is reached and HIRC frequency is still not locked on target frequency set by TRIM_SEL.
     * |        |          |If this bit is high and TRIM_FAIL_INT is set during auto trim operation, an interrupt will be triggered to notify that HIRC trim value update limitation count is reached.
     * |        |          |0 = TRIM_FAIL_INT status Disabled to trigger an interrupt to CPU.
     * |        |          |1 = TRIM_FAIL_INT status Enabled to trigger an interrupt to CPU.
     * |[2]     |32K_ERR_IEN|LXT Clock Error Interrupt Enable Control
     * |        |          |This bit controls if CPU could get an interrupt while LXT clock is inaccurate during auto trim operation.
     * |        |          |If this bit is high, and 32K_ERR_INT is set during auto trim operation, an interrupt will be triggered to notify the LXT clock frequency is inaccurate.
     * |        |          |0 = 32K_ERR_INT status Disabled to trigger an interrupt to CPU.
     * |        |          |1 = 32K_ERR_INT status Enabled to trigger an interrupt to CPU.
     * @var GCR_T::IRCTRIMISR
     * Offset: 0x88  HIRC Trim Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQ_LOCK |HIRC Frequency Lock Status
     * |        |          |This bit indicates the HIRC frequency locked in 22.1184 MHz.
     * |        |          |This is a read only status bit and doesn't trigger any interrupt.
     * |[1]     |TRIM_FAIL_INT|Trim Failure Interrupt Status
     * |        |          |This bit indicates that HIRC trim value update limitation count reached and HIRC clock frequency still doesn't lock.
     * |        |          |Once this bit is set, the auto trim operation stopped and TRIM_SEL will be cleared to 0 by hardware automatically.
     * |        |          |If this bit is set and TRIM_FAIL_IEN is high, an interrupt will be triggered to notify that HIRC trim value update limitation count was reached.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |        |          |0 = Trim value update limitation count is not reached.
     * |        |          |1 = Trim value update limitation count is reached and HIRC frequency is still not locked.
     * |[2]     |32K_ERR_INT|LXT Clock Error Interrupt Status
     * |        |          |This bit indicates that LXT clock frequency is inaccuracy.
     * |        |          |Once this bit is set, the auto trim operation stopped and TRIM_SEL will be cleared to 0 by hardware automatically.
     * |        |          |If this bit is set and 32K_ERR_IEN is high, an interrupt will be triggered to notify the LXT clock frequency is inaccuracy.
     * |        |          |Software can write 1 to clear this bit to 0.
     * |        |          |0 = LXT clock frequency is accuracy.
     * |        |          |1 = LXT clock frequency is inaccuracy.
     * @var GCR_T::RegLockAddr
     * Offset: 0x100  Register Write-Protection Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RegUnLock |Register Write-protection Disable Index (Read Only)
     * |        |          |0 = Write-protection Enabled for writing protected registers.
     * |        |          |Any write to the protected register is ignored.
     * |        |          |1 = Write-protection Disabled for writing protected registers.
     * |        |          |The Protected registers are:
     * |        |          |IPRSTC1 (0x5000_0008)
     * |        |          |BODCTL (0x5000_0018)
     * |        |          |PWRCON (0x5000_0200), bit[6] is not protected for power wake-up interrupt clear
     * |        |          |APBCLK (0x5000_0208), bit[0] watchdog clock enable only
     * |        |          |CLKSEL0 (0x5000_0210)
     * |        |          |CLKSEL1 (0x5000_0214), bit[1:0] Watchdog clock source select only
     * |        |          |NMI_SEL (0x5000_0380), bit[8] NMI interrupt enable only
     * |        |          |ISPCON (0x5000_C000)
     * |        |          |ISPTRG (0x5000_C010)
     * |        |          |WTCR (0x4000_4000)
     * |        |          |Note: The bits which are write-protected will be noted as" (Write Protect)" beside the description.
     * |[7:0]   |REGWRPROT |Register Write-protection Code (Write Only)
     * |        |          |Some registers have write-protection function.
     * |        |          |Writing these registers have to disable the protected function by writing the sequence value 0x59, 0x16, 0x88 to this field.
     * |        |          |After this sequence is completed, the REGPROTDIS bit will be set to 1 and write-protection registers can be normal write.
     */

    __I  uint32_t PDID;          /* Offset: 0x00  Part Device Identification Number Register                         */
    __IO uint32_t RSTSRC;        /* Offset: 0x04  System Reset Source Register                                       */
    __IO uint32_t IPRSTC1;       /* Offset: 0x08  Peripheral Reset Control Resister 1                                */
    __IO uint32_t IPRSTC2;       /* Offset: 0x0C  Peripheral Reset Control Resister 2                                */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0[2];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t BODCTL;         /* Offset: 0x18  Brown-out Detector Control Register                                */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE1[5];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t P0_MFP;        /* Offset: 0x30  P0 Multiple Function and Input Type Control Register               */
    __IO uint32_t P1_MFP;        /* Offset: 0x34  P1 Multiple Function and Input Type Control Register               */
    __IO uint32_t P2_MFP;        /* Offset: 0x38  P2 Multiple Function and Input Type Control Register               */
    __IO uint32_t P3_MFP;        /* Offset: 0x3C  P3 Multiple Function and Input Type Control Register               */
    __IO uint32_t P4_MFP;        /* Offset: 0x40  P4 Multiple Function and Input Type Control Register               */
    __IO uint32_t P5_MFP;        /* Offset: 0x44  P5 Multiple Function and Input Type Control Register               */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE2[14];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t IRCTRIMCTL;    /* Offset: 0x80  HIRC Trim Control Register                                         */
    __IO uint32_t IRCTRIMIER;    /* Offset: 0x84  HIRC Trim Interrupt Enable Control Register                        */
    __IO uint32_t IRCTRIMISR;    /* Offset: 0x88  HIRC Trim Interrupt Status Register                                */
    /// @cond HIDDEN_SYMBOLS
    __IO uint32_t RESERVE3[29];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t RegLockAddr;   /* Offset: 0x100  Register Write-Protection Control Register                        */

} GCR_T;



/**
    @addtogroup GCR_CONST GCR Bit Field Definition
    Constant Definitions for GCR Controller
@{ */

#define SYS_PDID_PDID_Pos                (0)                                               /*!< GCR_T::PDID: PDID Position                */
#define SYS_PDID_PDID_Msk                (0xfffffffful << SYS_PDID_PDID_Pos)               /*!< GCR_T::PDID: PDID Mask                    */

#define SYS_RSTSRC_RSTS_POR_Pos          (0)                                               /*!< GCR_T::RSTSRC: RSTS_POR Position          */
#define SYS_RSTSRC_RSTS_POR_Msk          (0x1ul << SYS_RSTSRC_RSTS_POR_Pos)                /*!< GCR_T::RSTSRC: RSTS_POR Mask              */

#define SYS_RSTSRC_RSTS_RESET_Pos        (1)                                               /*!< GCR_T::RSTSRC: RSTS_RESET Position        */
#define SYS_RSTSRC_RSTS_RESET_Msk        (0x1ul << SYS_RSTSRC_RSTS_RESET_Pos)              /*!< GCR_T::RSTSRC: RSTS_RESET Mask            */

#define SYS_RSTSRC_RSTS_WDT_Pos          (2)                                               /*!< GCR_T::RSTSRC: RSTS_WDT Position          */
#define SYS_RSTSRC_RSTS_WDT_Msk          (0x1ul << SYS_RSTSRC_RSTS_WDT_Pos)                /*!< GCR_T::RSTSRC: RSTS_WDT Mask              */

#define SYS_RSTSRC_RSTS_BOD_Pos          (4)                                               /*!< GCR_T::RSTSRC: RSTS_BOD Position          */
#define SYS_RSTSRC_RSTS_BOD_Msk          (0x1ul << SYS_RSTSRC_RSTS_BOD_Pos)                /*!< GCR_T::RSTSRC: RSTS_BOD Mask              */

#define SYS_RSTSRC_RSTS_MCU_Pos          (5)                                               /*!< GCR_T::RSTSRC: RSTS_MCU Position          */
#define SYS_RSTSRC_RSTS_MCU_Msk          (0x1ul << SYS_RSTSRC_RSTS_MCU_Pos)                /*!< GCR_T::RSTSRC: RSTS_MCU Mask              */

#define SYS_RSTSRC_RSTS_CPU_Pos          (7)                                               /*!< GCR_T::RSTSRC: RSTS_CPU Position          */
#define SYS_RSTSRC_RSTS_CPU_Msk          (0x1ul << SYS_RSTSRC_RSTS_CPU_Pos)                /*!< GCR_T::RSTSRC: RSTS_CPU Mask              */

#define SYS_IPRSTC1_CHIP_RST_Pos         (0)                                               /*!< GCR_T::IPRSTC1: CHIP_RST Position         */
#define SYS_IPRSTC1_CHIP_RST_Msk         (0x1ul << SYS_IPRSTC1_CHIP_RST_Pos)               /*!< GCR_T::IPRSTC1: CHIP_RST Mask             */

#define SYS_IPRSTC1_CPU_RST_Pos          (1)                                               /*!< GCR_T::IPRSTC1: CPU_RST Position          */
#define SYS_IPRSTC1_CPU_RST_Msk          (0x1ul << SYS_IPRSTC1_CPU_RST_Pos)                /*!< GCR_T::IPRSTC1: CPU_RST Mask              */

#define SYS_IPRSTC2_GPIO_RST_Pos         (1)                                               /*!< GCR_T::IPRSTC2: GPIO_RST Position         */
#define SYS_IPRSTC2_GPIO_RST_Msk         (0x1ul << SYS_IPRSTC2_GPIO_RST_Pos)               /*!< GCR_T::IPRSTC2: GPIO_RST Mask             */

#define SYS_IPRSTC2_TMR0_RST_Pos         (2)                                               /*!< GCR_T::IPRSTC2: TMR0_RST Position         */
#define SYS_IPRSTC2_TMR0_RST_Msk         (0x1ul << SYS_IPRSTC2_TMR0_RST_Pos)               /*!< GCR_T::IPRSTC2: TMR0_RST Mask             */

#define SYS_IPRSTC2_TMR1_RST_Pos         (3)                                               /*!< GCR_T::IPRSTC2: TMR1_RST Position         */
#define SYS_IPRSTC2_TMR1_RST_Msk         (0x1ul << SYS_IPRSTC2_TMR1_RST_Pos)               /*!< GCR_T::IPRSTC2: TMR1_RST Mask             */

#define SYS_IPRSTC2_I2C_RST_Pos          (8)                                               /*!< GCR_T::IPRSTC2: I2C_RST Position          */
#define SYS_IPRSTC2_I2C_RST_Msk          (0x1ul << SYS_IPRSTC2_I2C_RST_Pos)                /*!< GCR_T::IPRSTC2: I2C_RST Mask              */

#define SYS_IPRSTC2_SPI_RST_Pos          (12)                                              /*!< GCR_T::IPRSTC2: SPI_RST Position          */
#define SYS_IPRSTC2_SPI_RST_Msk          (0x1ul << SYS_IPRSTC2_SPI_RST_Pos)                /*!< GCR_T::IPRSTC2: SPI_RST Mask              */

#define SYS_IPRSTC2_UART_RST_Pos         (16)                                              /*!< GCR_T::IPRSTC2: UART_RST Position         */
#define SYS_IPRSTC2_UART_RST_Msk         (0x1ul << SYS_IPRSTC2_UART_RST_Pos)               /*!< GCR_T::IPRSTC2: UART_RST Mask             */

#define SYS_IPRSTC2_PWM_RST_Pos          (20)                                              /*!< GCR_T::IPRSTC2: PWM_RST Position          */
#define SYS_IPRSTC2_PWM_RST_Msk          (0x1ul << SYS_IPRSTC2_PWM_RST_Pos)                /*!< GCR_T::IPRSTC2: PWM_RST Mask              */

#define SYS_IPRSTC2_ACMP_RST_Pos         (22)                                              /*!< GCR_T::IPRSTC2: ACMP_RST Position         */
#define SYS_IPRSTC2_ACMP_RST_Msk         (0x1ul << SYS_IPRSTC2_ACMP_RST_Pos)               /*!< GCR_T::IPRSTC2: ACMP_RST Mask             */

#define SYS_IPRSTC2_ADC_RST_Pos          (28)                                              /*!< GCR_T::IPRSTC2: ADC_RST Position          */
#define SYS_IPRSTC2_ADC_RST_Msk          (0x1ul << SYS_IPRSTC2_ADC_RST_Pos)                /*!< GCR_T::IPRSTC2: ADC_RST Mask              */

#define SYS_BODCR_BOD_VL_EXT_Pos         (0)                                               /*!< GCR_T::BODCTL: BOD_VL_EXT Position         */
#define SYS_BODCR_BOD_VL_EXT_Msk         (0x1ul << SYS_BODCR_BOD_VL_EXT_Pos)               /*!< GCR_T::BODCTL: BOD_VL_EXT Mask             */

#define SYS_BODCR_BOD_VL_Pos             (1)                                               /*!< GCR_T::BODCTL: BOD_VL Position             */
#define SYS_BODCR_BOD_VL_Msk             (0x3ul << SYS_BODCR_BOD_VL_Pos)                   /*!< GCR_T::BODCTL: BOD_VL Mask                 */

#define SYS_BODCR_BOD_RSTEN_Pos          (3)                                               /*!< GCR_T::BODCTL: BOD_RSTEN Position          */
#define SYS_BODCR_BOD_RSTEN_Msk          (0x1ul << SYS_BODCR_BOD_RSTEN_Pos)                /*!< GCR_T::BODCTL: BOD_RSTEN Mask              */

#define SYS_BODCR_BOD_INTF_Pos           (4)                                               /*!< GCR_T::BODCTL: BOD_INTF Position           */
#define SYS_BODCR_BOD_INTF_Msk           (0x1ul << SYS_BODCR_BOD_INTF_Pos)                 /*!< GCR_T::BODCTL: BOD_INTF Mask               */

#define SYS_BODCR_BOD_LPM_Pos            (5)                                               /*!< GCR_T::BODCTL: BOD_LPM Position            */
#define SYS_BODCR_BOD_LPM_Msk            (0x1ul << SYS_BODCR_BOD_LPM_Pos)                  /*!< GCR_T::BODCTL: BOD_LPM Mask                */

#define SYS_BODCR_BOD_OUT_Pos            (6)                                               /*!< GCR_T::BODCTL: BOD_OUT Position            */
#define SYS_BODCR_BOD_OUT_Msk            (0x1ul << SYS_BODCR_BOD_OUT_Pos)                  /*!< GCR_T::BODCTL: BOD_OUT Mask                */

#define SYS_P0_MFP_P0_MFP_Pos            (0)                                               /*!< GCR_T::P0_MFP: P0_MFP Position            */
#define SYS_P0_MFP_P0_MFP_Msk            (0xfful << SYS_P0_MFP_P0_MFP_Pos)                 /*!< GCR_T::P0_MFP: P0_MFP Mask                */

#define SYS_P0_MFP_P0_ALT_Pos            (8)                                               /*!< GCR_T::P0_MFP: P0_ALT Position            */
#define SYS_P0_MFP_P0_ALT_Msk            (0xfful << SYS_P0_MFP_P0_ALT_Pos)                 /*!< GCR_T::P0_MFP: P0_ALT Mask                */

#define SYS_P0_MFP_P0_ALT0_Pos           (8)                                               /*!< GCR_T::P0_MFP: P0_ALT0 Position           */
#define SYS_P0_MFP_P0_ALT0_Msk           (0x1ul << SYS_P0_MFP_P0_ALT0_Pos)                 /*!< GCR_T::P0_MFP: P0_ALT0 Mask               */

#define SYS_P0_MFP_P0_ALT1_Pos           (9)                                               /*!< GCR_T::P0_MFP: P0_ALT1 Position           */
#define SYS_P0_MFP_P0_ALT1_Msk           (0x1ul << SYS_P0_MFP_P0_ALT1_Pos)                 /*!< GCR_T::P0_MFP: P0_ALT1 Mask               */

#define SYS_P0_MFP_P0_ALT4_Pos           (12)                                              /*!< GCR_T::P0_MFP: P0_ALT4 Position           */
#define SYS_P0_MFP_P0_ALT4_Msk           (0x1ul << SYS_P0_MFP_P0_ALT4_Pos)                 /*!< GCR_T::P0_MFP: P0_ALT4 Mask               */

#define SYS_P0_MFP_P0_ALT5_Pos           (13)                                              /*!< GCR_T::P0_MFP: P0_ALT5 Position           */
#define SYS_P0_MFP_P0_ALT5_Msk           (0x1ul << SYS_P0_MFP_P0_ALT5_Pos)                 /*!< GCR_T::P0_MFP: P0_ALT5 Mask               */

#define SYS_P0_MFP_P0_ALT6_Pos           (14)                                              /*!< GCR_T::P0_MFP: P0_ALT6 Position           */
#define SYS_P0_MFP_P0_ALT6_Msk           (0x1ul << SYS_P0_MFP_P0_ALT6_Pos)                 /*!< GCR_T::P0_MFP: P0_ALT6 Mask               */

#define SYS_P0_MFP_P0_ALT7_Pos           (15)                                              /*!< GCR_T::P0_MFP: P0_ALT7 Position           */
#define SYS_P0_MFP_P0_ALT7_Msk           (0x1ul << SYS_P0_MFP_P0_ALT7_Pos)                 /*!< GCR_T::P0_MFP: P0_ALT7 Mask               */

#define SYS_P0_MFP_P0_TYPE_Pos           (16)                                              /*!< GCR_T::P0_MFP: P0_TYPE Position           */
#define SYS_P0_MFP_P0_TYPE_Msk           (0xfful << SYS_P0_MFP_P0_TYPE_Pos)                /*!< GCR_T::P0_MFP: P0_TYPE Mask               */

#define SYS_P1_MFP_P1_MFP_Pos            (0)                                               /*!< GCR_T::P1_MFP: P1_MFP Position            */
#define SYS_P1_MFP_P1_MFP_Msk            (0xfful << SYS_P1_MFP_P1_MFP_Pos)                 /*!< GCR_T::P1_MFP: P1_MFP Mask                */

#define SYS_P1_MFP_P1_ALT_Pos            (8)                                               /*!< GCR_T::P1_MFP: P1_ALT Position            */
#define SYS_P1_MFP_P1_ALT_Msk            (0xfful << SYS_P1_MFP_P1_ALT_Pos)                 /*!< GCR_T::P1_MFP: P1_ALT Mask                */

#define SYS_P1_MFP_P1_ALT0_Pos           (8)                                               /*!< GCR_T::P1_MFP: P1_ALT0 Position           */
#define SYS_P1_MFP_P1_ALT0_Msk           (0x1ul << SYS_P1_MFP_P1_ALT0_Pos)                 /*!< GCR_T::P1_MFP: P1_ALT0 Mask               */

#define SYS_P1_MFP_P1_ALT2_Pos           (10)                                              /*!< GCR_T::P1_MFP: P1_ALT2 Position           */
#define SYS_P1_MFP_P1_ALT2_Msk           (0x1ul << SYS_P1_MFP_P1_ALT2_Pos)                 /*!< GCR_T::P1_MFP: P1_ALT2 Mask               */

#define SYS_P1_MFP_P1_ALT3_Pos           (11)                                              /*!< GCR_T::P1_MFP: P1_ALT3 Position           */
#define SYS_P1_MFP_P1_ALT3_Msk           (0x1ul << SYS_P1_MFP_P1_ALT3_Pos)                 /*!< GCR_T::P1_MFP: P1_ALT3 Mask               */

#define SYS_P1_MFP_P1_ALT4_Pos           (12)                                              /*!< GCR_T::P1_MFP: P1_ALT4 Position           */
#define SYS_P1_MFP_P1_ALT4_Msk           (0x1ul << SYS_P1_MFP_P1_ALT4_Pos)                 /*!< GCR_T::P1_MFP: P1_ALT4 Mask               */

#define SYS_P1_MFP_P1_ALT5_Pos           (13)                                              /*!< GCR_T::P1_MFP: P1_ALT5 Position           */
#define SYS_P1_MFP_P1_ALT5_Msk           (0x1ul << SYS_P1_MFP_P1_ALT5_Pos)                 /*!< GCR_T::P1_MFP: P1_ALT5 Mask               */

#define SYS_P1_MFP_P1_TYPE_Pos           (16)                                              /*!< GCR_T::P1_MFP: P1_TYPE Position           */
#define SYS_P1_MFP_P1_TYPE_Msk           (0xfful << SYS_P1_MFP_P1_TYPE_Pos)                /*!< GCR_T::P1_MFP: P1_TYPE Mask               */

#define SYS_P2_MFP_P2_MFP_Pos            (0)                                               /*!< GCR_T::P2_MFP: P2_MFP Position            */
#define SYS_P2_MFP_P2_MFP_Msk            (0xfful << SYS_P2_MFP_P2_MFP_Pos)                 /*!< GCR_T::P2_MFP: P2_MFP Mask                */

#define SYS_P2_MFP_P2_ALT_Pos            (8)                                               /*!< GCR_T::P2_MFP: P2_ALT Position            */
#define SYS_P2_MFP_P2_ALT_Msk            (0xfful << SYS_P2_MFP_P2_ALT_Pos)                 /*!< GCR_T::P2_MFP: P2_ALT Mask                */

#define SYS_P2_MFP_P2_ALT2_Pos           (10)                                              /*!< GCR_T::P2_MFP: P2_ALT2 Position           */
#define SYS_P2_MFP_P2_ALT2_Msk           (0x1ul << SYS_P2_MFP_P2_ALT2_Pos)                 /*!< GCR_T::P2_MFP: P2_ALT2 Mask               */

#define SYS_P2_MFP_P2_ALT3_Pos           (11)                                              /*!< GCR_T::P2_MFP: P2_ALT3 Position           */
#define SYS_P2_MFP_P2_ALT3_Msk           (0x1ul << SYS_P2_MFP_P2_ALT3_Pos)                 /*!< GCR_T::P2_MFP: P2_ALT3 Mask               */

#define SYS_P2_MFP_P2_ALT4_Pos           (12)                                              /*!< GCR_T::P2_MFP: P2_ALT4 Position           */
#define SYS_P2_MFP_P2_ALT4_Msk           (0x1ul << SYS_P2_MFP_P2_ALT4_Pos)                 /*!< GCR_T::P2_MFP: P2_ALT4 Mask               */

#define SYS_P2_MFP_P2_ALT5_Pos           (13)                                              /*!< GCR_T::P2_MFP: P2_ALT5 Position           */
#define SYS_P2_MFP_P2_ALT5_Msk           (0x1ul << SYS_P2_MFP_P2_ALT5_Pos)                 /*!< GCR_T::P2_MFP: P2_ALT5 Mask               */

#define SYS_P2_MFP_P2_ALT6_Pos           (14)                                              /*!< GCR_T::P2_MFP: P2_ALT6 Position           */
#define SYS_P2_MFP_P2_ALT6_Msk           (0x1ul << SYS_P2_MFP_P2_ALT6_Pos)                 /*!< GCR_T::P2_MFP: P2_ALT6 Mask               */

#define SYS_P2_MFP_P2_TYPE_Pos           (16)                                              /*!< GCR_T::P2_MFP: P2_TYPE Position           */
#define SYS_P2_MFP_P2_TYPE_Msk           (0xfful << SYS_P2_MFP_P2_TYPE_Pos)                /*!< GCR_T::P2_MFP: P2_TYPE Mask               */

#define SYS_P3_MFP_P3_MFP_Pos            (0)                                               /*!< GCR_T::P3_MFP: P3_MFP Position            */
#define SYS_P3_MFP_P3_MFP_Msk            (0xfful << SYS_P3_MFP_P3_MFP_Pos)                 /*!< GCR_T::P3_MFP: P3_MFP Mask                */

#define SYS_P3_MFP_P3_ALT_Pos            (8)                                               /*!< GCR_T::P3_MFP: P3_ALT Position            */
#define SYS_P3_MFP_P3_ALT_Msk            (0xfful << SYS_P3_MFP_P3_ALT_Pos)                 /*!< GCR_T::P3_MFP: P3_ALT Mask                */

#define SYS_P3_MFP_P3_ALT0_Pos           (8)                                               /*!< GCR_T::P3_MFP: P3_ALT0 Position           */
#define SYS_P3_MFP_P3_ALT0_Msk           (0x1ul << SYS_P3_MFP_P3_ALT0_Pos)                 /*!< GCR_T::P3_MFP: P3_ALT0 Mask               */

#define SYS_P3_MFP_P3_ALT1_Pos           (9)                                               /*!< GCR_T::P3_MFP: P3_ALT1 Position           */
#define SYS_P3_MFP_P3_ALT1_Msk           (0x1ul << SYS_P3_MFP_P3_ALT1_Pos)                 /*!< GCR_T::P3_MFP: P3_ALT1 Mask               */

#define SYS_P3_MFP_P3_ALT2_Pos           (10)                                              /*!< GCR_T::P3_MFP: P3_ALT2 Position           */
#define SYS_P3_MFP_P3_ALT2_Msk           (0x1ul << SYS_P3_MFP_P3_ALT2_Pos)                 /*!< GCR_T::P3_MFP: P3_ALT2 Mask               */

#define SYS_P3_MFP_P3_ALT4_Pos           (12)                                              /*!< GCR_T::P3_MFP: P3_ALT4 Position           */
#define SYS_P3_MFP_P3_ALT4_Msk           (0x1ul << SYS_P3_MFP_P3_ALT4_Pos)                 /*!< GCR_T::P3_MFP: P3_ALT4 Mask               */

#define SYS_P3_MFP_P3_ALT5_Pos           (13)                                              /*!< GCR_T::P3_MFP: P3_ALT5 Position           */
#define SYS_P3_MFP_P3_ALT5_Msk           (0x1ul << SYS_P3_MFP_P3_ALT5_Pos)                 /*!< GCR_T::P3_MFP: P3_ALT5 Mask               */

#define SYS_P3_MFP_P3_ALT6_Pos           (14)                                              /*!< GCR_T::P3_MFP: P3_ALT6 Position           */
#define SYS_P3_MFP_P3_ALT6_Msk           (0x1ul << SYS_P3_MFP_P3_ALT6_Pos)                 /*!< GCR_T::P3_MFP: P3_ALT6 Mask               */

#define SYS_P3_MFP_P3_TYPE_Pos           (16)                                              /*!< GCR_T::P3_MFP: P3_TYPE Position           */
#define SYS_P3_MFP_P3_TYPE_Msk           (0xfful << SYS_P3_MFP_P3_TYPE_Pos)                /*!< GCR_T::P3_MFP: P3_TYPE Mask               */

#define SYS_P3_MFP_P32CPP1_Pos           (24)                                              /*!< GCR_T::P3_MFP: P32CPP1 Position           */
#define SYS_P3_MFP_P32CPP1_Msk           (0x1ul << SYS_P3_MFP_P32CPP1_Pos)                 /*!< GCR_T::P3_MFP: P32CPP1 Mask               */

#define SYS_P4_MFP_P4_MFP_Pos            (0)                                               /*!< GCR_T::P4_MFP: P4_MFP Position            */
#define SYS_P4_MFP_P4_MFP_Msk            (0xfful << SYS_P4_MFP_P4_MFP_Pos)                 /*!< GCR_T::P4_MFP: P4_MFP Mask                */

#define SYS_P4_MFP_P4_ALT_Pos            (8)                                               /*!< GCR_T::P4_MFP: P4_ALT Position           */
#define SYS_P4_MFP_P4_ALT_Msk            (0xfful << SYS_P4_MFP_P4_ALT_Pos)                 /*!< GCR_T::P4_MFP: P4_ALT Mask               */

#define SYS_P4_MFP_P4_ALT6_Pos           (14)                                              /*!< GCR_T::P4_MFP: P4_ALT6 Position           */
#define SYS_P4_MFP_P4_ALT6_Msk           (0x1ul << SYS_P4_MFP_P4_ALT6_Pos)                 /*!< GCR_T::P4_MFP: P4_ALT6 Mask               */

#define SYS_P4_MFP_P4_ALT7_Pos           (15)                                              /*!< GCR_T::P4_MFP: P4_ALT7 Position           */
#define SYS_P4_MFP_P4_ALT7_Msk           (0x1ul << SYS_P4_MFP_P4_ALT7_Pos)                 /*!< GCR_T::P4_MFP: P4_ALT7 Mask               */

#define SYS_P4_MFP_P4_TYPE_Pos           (16)                                              /*!< GCR_T::P4_MFP: P4_TYPE Position           */
#define SYS_P4_MFP_P4_TYPE_Msk           (0xfful << SYS_P4_MFP_P4_TYPE_Pos)                /*!< GCR_T::P4_MFP: P4_TYPE Mask               */

#define SYS_P5_MFP_P5_MFP_Pos            (0)                                               /*!< GCR_T::P5_MFP: P5_MFP Position            */
#define SYS_P5_MFP_P5_MFP_Msk            (0xfful << SYS_P5_MFP_P5_MFP_Pos)                 /*!< GCR_T::P5_MFP: P5_MFP Mask                */

#define SYS_P5_MFP_P5_ALT_Pos            (8)                                               /*!< GCR_T::P5_MFP: P5_ALT Position            */
#define SYS_P5_MFP_P5_ALT_Msk            (0xFFul << SYS_P5_MFP_P5_ALT_Pos)                 /*!< GCR_T::P5_MFP: P5_ALT Mask                */

#define SYS_P5_MFP_P5_ALT0_Pos           (8)                                               /*!< GCR_T::P5_MFP: P5_ALT0 Position           */
#define SYS_P5_MFP_P5_ALT0_Msk           (0x1ul << SYS_P5_MFP_P5_ALT0_Pos)                 /*!< GCR_T::P5_MFP: P5_ALT0 Mask               */

#define SYS_P5_MFP_P5_ALT1_Pos           (9)                                               /*!< GCR_T::P5_MFP: P5_ALT1 Position           */
#define SYS_P5_MFP_P5_ALT1_Msk           (0x1ul << SYS_P5_MFP_P5_ALT1_Pos)                 /*!< GCR_T::P5_MFP: P5_ALT1 Mask               */

#define SYS_P5_MFP_P5_ALT2_Pos           (10)                                              /*!< GCR_T::P5_MFP: P5_ALT2 Position           */
#define SYS_P5_MFP_P5_ALT2_Msk           (0x1ul << SYS_P5_MFP_P5_ALT2_Pos)                 /*!< GCR_T::P5_MFP: P5_ALT2 Mask               */

#define SYS_P5_MFP_P5_ALT3_Pos           (11)                                              /*!< GCR_T::P5_MFP: P5_ALT3 Position           */
#define SYS_P5_MFP_P5_ALT3_Msk           (0x1ul << SYS_P5_MFP_P5_ALT3_Pos)                 /*!< GCR_T::P5_MFP: P5_ALT3 Mask               */

#define SYS_P5_MFP_P5_ALT4_Pos           (12)                                              /*!< GCR_T::P5_MFP: P5_ALT4 Position           */
#define SYS_P5_MFP_P5_ALT4_Msk           (0x1ul << SYS_P5_MFP_P5_ALT4_Pos)                 /*!< GCR_T::P5_MFP: P5_ALT4 Mask               */

#define SYS_P5_MFP_P5_ALT5_Pos           (13)                                              /*!< GCR_T::P5_MFP: P5_ALT5 Position           */
#define SYS_P5_MFP_P5_ALT5_Msk           (0x1ul << SYS_P5_MFP_P5_ALT5_Pos)                 /*!< GCR_T::P5_MFP: P5_ALT5 Mask               */

#define SYS_P5_MFP_P5_TYPE_Pos           (16)                                              /*!< GCR_T::P5_MFP: P5_TYPE Position           */
#define SYS_P5_MFP_P5_TYPE_Msk           (0xfful << SYS_P5_MFP_P5_TYPE_Pos)                /*!< GCR_T::P5_MFP: P5_TYPE Mask               */

#define SYS_IRCTRIMCTL_TRIM_SEL_Pos      (0)                                               /*!< GCR_T::IRCTRIMCTL: TRIM_SEL Position      */
#define SYS_IRCTRIMCTL_TRIM_SEL_Msk      (0x1ul << SYS_IRCTRIMCTL_TRIM_SEL_Pos)            /*!< GCR_T::IRCTRIMCTL: TRIM_SEL Mask          */

#define SYS_IRCTRIMCTL_TRIM_LOOP_Pos     (4)                                               /*!< GCR_T::IRCTRIMCTL: TRIM_LOOP Position     */
#define SYS_IRCTRIMCTL_TRIM_LOOP_Msk     (0x3ul << SYS_IRCTRIMCTL_TRIM_LOOP_Pos)           /*!< GCR_T::IRCTRIMCTL: TRIM_LOOP Mask         */

#define SYS_IRCTRIMIEN_TRIM_FAIL_IEN_Pos (1)                                               /*!< GCR_T::IRCTRIMIER: TRIM_FAIL_IEN Position */
#define SYS_IRCTRIMIEN_TRIM_FAIL_IEN_Msk (0x1ul << SYS_IRCTRIMIEN_TRIM_FAIL_IEN_Pos)       /*!< GCR_T::IRCTRIMIER: TRIM_FAIL_IEN Mask     */

#define SYS_IRCTRIMIEN_32K_ERR_IEN_Pos   (2)                                               /*!< GCR_T::IRCTRIMIER: 32K_ERR_IEN Position   */
#define SYS_IRCTRIMIEN_32K_ERR_IEN_Msk   (0x1ul << SYS_IRCTRIMIEN_32K_ERR_IEN_Pos)         /*!< GCR_T::IRCTRIMIER: 32K_ERR_IEN Mask       */

#define SYS_IRCTRIMINT_FREQ_LOCK_Pos     (0)                                               /*!< GCR_T::IRCTRIMISR: FREQ_LOCK Position     */
#define SYS_IRCTRIMINT_FREQ_LOCK_Msk     (0x1ul << SYS_IRCTRIMINT_FREQ_LOCK_Pos)           /*!< GCR_T::IRCTRIMISR: FREQ_LOCK Mask         */

#define SYS_IRCTRIMINT_TRIM_FAIL_INT_Pos (1)                                               /*!< GCR_T::IRCTRIMISR: TRIM_FAIL_INT Position */
#define SYS_IRCTRIMINT_TRIM_FAIL_INT_Msk (0x1ul << SYS_IRCTRIMINT_TRIM_FAIL_INT_Pos)       /*!< GCR_T::IRCTRIMISR: TRIM_FAIL_INT Mask     */

#define SYS_IRCTRIMINT_32K_ERR_INT_Pos   (2)                                               /*!< GCR_T::IRCTRIMISR: 32K_ERR_INT Position   */
#define SYS_IRCTRIMINT_32K_ERR_INT_Msk   (0x1ul << SYS_IRCTRIMINT_32K_ERR_INT_Pos)         /*!< GCR_T::IRCTRIMISR: 32K_ERR_INT Mask       */

#define SYS_RegLockAddr_RegUnLock_Pos    0                                                 /*!< GCR_T::RegLockAddr: RegUnLock Position    */
#define SYS_RegLockAddr_RegUnLock_Msk    (0x1ul << SYS_RegLockAddr_RegUnLock_Pos)          /*!< GCR_T::RegLockAddr: RegUnLock Mask        */

/**@}*/ /* GCR_CONST */
/**@}*/ /* end of GCR register group */


/*---------------------- General Purpose Input/Output Controller -------------------------*/
/**
    @addtogroup GP General Purpose Input/Output Controller(GP)
    Memory Mapped Structure for GP Controller
@{ */


typedef struct
{

    /**
     * @var GPIO_T::PMD
     * Offset: 0x00  Px I/O Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |PMD0      |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin. Default mode is controlled by CIOINI (CONFIG0[10]).
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: x = 0~4, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_PMD[7:4] are reserved.
     * |        |          |P1_PMD[15:12], [3:2] are reserved.
     * |        |          |P2_PMD[15:14], [3:0] are reserved.
     * |        |          |P3_PMD[15:14], [7:6] are reserved.
     * |        |          |P4_PMD[11:0] are reserved.
     * |        |          |P5_PMD[15:12] are reserved.
     * |[3:2]   |PMD1      |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin. Default mode is controlled by CIOINI (CONFIG0[10]).
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: x = 0~4, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_PMD[7:4] are reserved.
     * |        |          |P1_PMD[15:12], [3:2] are reserved.
     * |        |          |P2_PMD[15:14], [3:0] are reserved.
     * |        |          |P3_PMD[15:14], [7:6] are reserved.
     * |        |          |P4_PMD[11:0] are reserved.
     * |        |          |P5_PMD[15:12] are reserved.
     * |[5:4]   |PMD2      |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin. Default mode is controlled by CIOINI (CONFIG0[10]).
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: x = 0~4, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_PMD[7:4] are reserved.
     * |        |          |P1_PMD[15:12], [3:2] are reserved.
     * |        |          |P2_PMD[15:14], [3:0] are reserved.
     * |        |          |P3_PMD[15:14], [7:6] are reserved.
     * |        |          |P4_PMD[11:0] are reserved.
     * |        |          |P5_PMD[15:12] are reserved.
     * |[7:6]   |PMD3      |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin. Default mode is controlled by CIOINI (CONFIG0[10]).
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: x = 0~4, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_PMD[7:4] are reserved.
     * |        |          |P1_PMD[15:12], [3:2] are reserved.
     * |        |          |P2_PMD[15:14], [3:0] are reserved.
     * |        |          |P3_PMD[15:14], [7:6] are reserved.
     * |        |          |P4_PMD[11:0] are reserved.
     * |        |          |P5_PMD[15:12] are reserved.
     * |[9:8]   |PMD4      |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin. Default mode is controlled by CIOINI (CONFIG0[10]).
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: x = 0~4, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_PMD[7:4] are reserved.
     * |        |          |P1_PMD[15:12], [3:2] are reserved.
     * |        |          |P2_PMD[15:14], [3:0] are reserved.
     * |        |          |P3_PMD[15:14], [7:6] are reserved.
     * |        |          |P4_PMD[11:0] are reserved.
     * |        |          |P5_PMD[15:12] are reserved.
     * |[11:10] |PMD5      |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin. Default mode is controlled by CIOINI (CONFIG0[10]).
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: x = 0~4, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_PMD[7:4] are reserved.
     * |        |          |P1_PMD[15:12], [3:2] are reserved.
     * |        |          |P2_PMD[15:14], [3:0] are reserved.
     * |        |          |P3_PMD[15:14], [7:6] are reserved.
     * |        |          |P4_PMD[11:0] are reserved.
     * |        |          |P5_PMD[15:12] are reserved.
     * |[13:12] |PMD6      |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin. Default mode is controlled by CIOINI (CONFIG0[10]).
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: x = 0~4, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_PMD[7:4] are reserved.
     * |        |          |P1_PMD[15:12], [3:2] are reserved.
     * |        |          |P2_PMD[15:14], [3:0] are reserved.
     * |        |          |P3_PMD[15:14], [7:6] are reserved.
     * |        |          |P4_PMD[11:0] are reserved.
     * |        |          |P5_PMD[15:12] are reserved.
     * |[15:14] |PMD7      |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin. Default mode is controlled by CIOINI (CONFIG0[10]).
     * |        |          |00 = Px.n is in Input mode.
     * |        |          |01 = Px.n is in Push-pull Output mode.
     * |        |          |10 = Px.n is in Open-drain Output mode.
     * |        |          |11 = Px.n is in Quasi-bidirectional mode.
     * |        |          |Note1: x = 0~4, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_PMD[7:4] are reserved.
     * |        |          |P1_PMD[15:12], [3:2] are reserved.
     * |        |          |P2_PMD[15:14], [3:0] are reserved.
     * |        |          |P3_PMD[15:14], [7:6] are reserved.
     * |        |          |P4_PMD[11:0] are reserved.
     * |        |          |P5_PMD[15:12] are reserved.
     * @var GPIO_T::OFFD
     * Offset: 0x04  Px Digital Input Path Disable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:16] |OFFD      |Port 0-5 Pin [N] Digital Input Path Disable Control
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note: x = 0~5, n = 0~7.
     * @var GPIO_T::DOUT
     * Offset: 0x08  Px Data Output Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DOUT      |Port 0-5 Pin [N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output and Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note1: x = 0~5, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_DOUT[3:2] are reserved.
     * |        |          |P1_DOUT[7:6], [1] are reserved.
     * |        |          |P2_DOUT[7], [1:0] are reserved.
     * |        |          |P3_DOUT[7], [3] are reserved.
     * |        |          |P4_DOUT[5:0] are reserved.
     * |        |          |P5_DOUT[7:6] are reserved.
     * @var GPIO_T::DMASK
     * Offset: 0x0C  Px Data Output Write Mask
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DMASK     |Port 0-5 Pin [N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding Px_DOUT[n] bit.
     * |        |          |When the DMASK[n] bit is set to 1, the corresponding Px_DOUT[n] bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignored.
     * |        |          |0 = Corresponding Px_DOUT[n] bit can be updated.
     * |        |          |1 = Corresponding Px_DOUT[n] bit is protected.
     * |        |          |Note1: x = 0~5, n = 0~7.
     * |        |          |Note2: This function only protects the corresponding Px_DOUT[n] bit, and will not protect the corresponding Pxn_PDIO bit.
     * |        |          |Note3:
     * |        |          |P0_DMASK[3:2] are reserved.
     * |        |          |P1_DMASK[7:6], [1] are reserved.
     * |        |          |P2_DMASK[7], [1:0] are reserved.
     * |        |          |P3_DMASK[7], [3] are reserved.
     * |        |          |P4_DMASK[5:0] are reserved.
     * |        |          |P5_DMASK[7:6] are reserved.
     * @var GPIO_T::PIN
     * Offset: 0x10  Px Pin Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PIN       |Port 0-5 Pin [N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note1: x = 0~5, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_PIN[3:2] are reserved.
     * |        |          |P1_PIN[7:6], [1] are reserved.
     * |        |          |P2_PIN[7], [1:0] are reserved.
     * |        |          |P3_PIN[7], [3] are reserved.
     * |        |          |P4_PIN[5:0] are reserved.
     * |        |          |P5_PIN[7:6] are reserved.
     * @var GPIO_T::DBEN
     * Offset: 0x14  Px De-bounce Enable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DBEN      |Port 0-5 Pin [N] Input Signal De-bounce Enable Control
     * |        |          |DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBNCECON[4], one de-bounce sample cycle period is controlled by DBNCECON[3:0].
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note1: x = 0~5, n = 0~7.
     * |        |          |Note2: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system woken up caused by the Px.n de-bounce function.
     * |        |          |Note3:
     * |        |          |P0_DBEN[3:2] are reserved.
     * |        |          |P1_DBEN[7:6], [1] are reserved.
     * |        |          |P2_DBEN[7], [1:0] are reserved.
     * |        |          |P3_DBEN[7], [3] are reserved.
     * |        |          |P4_DBEN[5:0] are reserved.
     * |        |          |P5_DBEN[7:6] are reserved.
     * @var GPIO_T::IMD
     * Offset: 0x18  Px Interrupt Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |IMD       |Port 0-5 Pin [N] Edge Or Level Detection Interrupt Mode Control
     * |        |          |IMD[n] bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If pin is set as the level trigger interrupt, only one level can be set on the registers Px_IEN.
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note1: x = 0~5, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_IMD[3:2] are reserved.
     * |        |          |P1_IMD[7:6], [1] are reserved.
     * |        |          |P2_IMD[7], [1:0] are reserved.
     * |        |          |P3_IMD[7], [3] are reserved.
     * |        |          |P4_IMD[5:0] are reserved.
     * |        |          |P5_IMD[7:6] are reserved.
     * @var GPIO_T::IEN
     * Offset: 0x1C  Px Interrupt Enable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IF_EN     |Port 0-5 Pin [N] Interrupt Enabled By Input Falling Edge Or Input Level Low
     * |        |          |IF_EN[n] is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the IF_EB[n] bit to 1:
     * |        |          |If the interrupt is level trigger (IMD[n] is 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge mode trigger (IMD[n] is 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n low level or high to low interrupt Disabled.
     * |        |          |1 = Px.n low level or high to low interrupt Enabled.
     * |        |          |Note1: x = 0~5, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_IEN[19:18], [3:2] are reserved.
     * |        |          |P1_IEN[23:22], [17], [7:6], [1] are reserved.
     * |        |          |P2_IEN[23], [17:16], [7], [1:0] are reserved.
     * |        |          |P3_IEN[23], [19], [7], [3] are reserved.
     * |        |          |P4_IEN[21:16], [5:0] are reserved.
     * |        |          |P5_IEN[23:22], [7:6] are reserved.
     * |[23:16] |IR_EN     |Port 0-5 Pin [N] Interrupt Enabled By Input Rising Edge Or Input Level High
     * |        |          |IR_EN[n] bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the IR_EN[n] bit to 1:
     * |        |          |If the interrupt is level trigger (IMD[n] is 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (IMD[n] is 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note: x = 0~5, n = 0~7.
     * @var GPIO_T::ISRC
     * Offset: 0x20  Px Interrupt Source Flag
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |ISRC      |Port 0-5 Pin [N] Interrupt Source Flag
     * |        |          |Write :
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read :
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note1: x = 0~5, n = 0~7.
     * |        |          |Note2:
     * |        |          |P0_ISRC[3:2] are reserved.
     * |        |          |P1_ISRC[7:6], [1] are reserved.
     * |        |          |P2_ISRC[7], [1:0] are reserved.
     * |        |          |P3_ISRC[7], [3] are reserved.
     * |        |          |P4_ISRC[5:0] are reserved.
     * |        |          |P5_ISRC[7:6] are reserved.
     */
    __IO uint32_t PMD;           /* Offset: 0x00  Px I/O Mode Control                                                */
    __IO uint32_t OFFD;          /* Offset: 0x04  Px Digital Input Path Disable Control                              */
    __IO uint32_t DOUT;          /* Offset: 0x08  Px Data Output Value                                               */
    __IO uint32_t DMASK;         /* Offset: 0x0C  Px Data Output Write Mask                                          */
    __I  uint32_t PIN;           /* Offset: 0x10  Px Pin Value                                                       */
    __IO uint32_t DBEN;          /* Offset: 0x14  Px De-bounce Enable Control                                        */
    __IO uint32_t IMD;           /* Offset: 0x18  Px Interrupt Mode Control                                          */
    __IO uint32_t IEN;           /* Offset: 0x1C  Px Interrupt Enable Control                                        */
    __IO uint32_t ISRC;          /* Offset: 0x20  Px Interrupt Source Flag                                           */
} GPIO_T;

/**
  * @brief GPIO debounce register map
  */
typedef struct
{
    /**
     * @var GPIO_DBNCECON_T::DBNCECON
     * Offset: 0x180  Interrupt De-bounce Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |DBCLKSEL  |De-bounce Sampling Cycle Selection
     * |        |          |0000 = Sample interrupt input once per 1 clock.
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
    __IO uint32_t DBNCECON;      /* Offset: 0x00  Interrupt De-bounce Control                                       */
} GPIO_DBNCECON_T;



/**
    @addtogroup GP_CONST GP Bit Field Definition
    Constant Definitions for GP Controller
@{ */

#define GPIO_PMD_PMD0_Pos                (0)                                               /*!< GPIO_T::PMD: PMD0 Position                  */
#define GPIO_PMD_PMD0_Msk                (0x3ul << GPIO_PMD_PMD0_Pos)                      /*!< GPIO_T::PMD: PMD0 Mask                      */

#define GPIO_PMD_PMD1_Pos                (2)                                               /*!< GPIO_T::PMD: PMD1 Position                  */
#define GPIO_PMD_PMD1_Msk                (0x3ul << GPIO_PMD_PMD1_Pos)                      /*!< GPIO_T::PMD: PMD1 Mask                      */

#define GPIO_PMD_PMD2_Pos                (4)                                               /*!< GPIO_T::PMD: PMD2 Position                  */
#define GPIO_PMD_PMD2_Msk                (0x3ul << GPIO_PMD_PMD2_Pos)                      /*!< GPIO_T::PMD: PMD2 Mask                      */

#define GPIO_PMD_PMD3_Pos                (6)                                               /*!< GPIO_T::PMD: PMD3 Position                  */
#define GPIO_PMD_PMD3_Msk                (0x3ul << GPIO_PMD_PMD3_Pos)                      /*!< GPIO_T::PMD: PMD3 Mask                      */

#define GPIO_PMD_PMD4_Pos                (8)                                               /*!< GPIO_T::PMD: PMD4 Position                  */
#define GPIO_PMD_PMD4_Msk                (0x3ul << GPIO_PMD_PMD4_Pos)                      /*!< GPIO_T::PMD: PMD4 Mask                      */

#define GPIO_PMD_PMD5_Pos                (10)                                              /*!< GPIO_T::PMD: PMD5 Position                  */
#define GPIO_PMD_PMD5_Msk                (0x3ul << GPIO_PMD_PMD5_Pos)                      /*!< GPIO_T::PMD: PMD5 Mask                      */

#define GPIO_PMD_PMD6_Pos                (12)                                              /*!< GPIO_T::PMD: PMD6 Position                  */
#define GPIO_PMD_PMD6_Msk                (0x3ul << GPIO_PMD_PMD6_Pos)                      /*!< GPIO_T::PMD: PMD6 Mask                      */

#define GPIO_PMD_PMD7_Pos                (14)                                              /*!< GPIO_T::PMD: PMD7 Position                  */
#define GPIO_PMD_PMD7_Msk                (0x3ul << GPIO_PMD_PMD7_Pos)                      /*!< GPIO_T::PMD: PMD7 Mask                      */

#define GPIO_OFFD_OFFD_Pos               (16)                                              /*!< GPIO_T::OFFD: OFFD Position                 */
#define GPIO_OFFD_OFFD_Msk               (0xfful << GPIO_OFFD_OFFD_Pos)                    /*!< GPIO_T::OFFD: OFFD Mask                     */

#define GPIO_DOUT_DOUT_Pos               (0)                                               /*!< GPIO_T::DOUT: DOUT Position                 */
#define GPIO_DOUT_DOUT_Msk               (0xfful << GPIO_DOUT_DOUT_Pos)                    /*!< GPIO_T::DOUT: DOUT Mask                     */

#define GPIO_DMASK_DMASK_Pos             (0)                                               /*!< GPIO_T::DMASK: DMASK Position               */
#define GPIO_DMASK_DMASK_Msk             (0xfful << GPIO_DMASK_DMASK_Pos)                  /*!< GPIO_T::DMASK: DMASK Mask                   */

#define GPIO_PIN_PIN_Pos                 (0)                                               /*!< GPIO_T::PIN: PIN Position                   */
#define GPIO_PIN_PIN_Msk                 (0xfful << GPIO_PIN_PIN_Pos)                      /*!< GPIO_T::PIN: PIN Mask                       */

#define GPIO_DBEN_DBEN_Pos               (0)                                               /*!< GPIO_T::DBEN: DBEN Position                 */
#define GPIO_DBEN_DBEN_Msk               (0xfful << GPIO_DBEN_DBEN_Pos)                    /*!< GPIO_T::DBEN: DBEN Mask                     */

#define GPIO_IMD_IMD_Pos                 (0)                                               /*!< GPIO_T::IMD: IMD Position                   */
#define GPIO_IMD_IMD_Msk                 (0xfful << GPIO_IMD_IMD_Pos)                      /*!< GPIO_T::IMD: IMD Mask                       */

#define GPIO_IEN_IF_EN_Pos               (0)                                               /*!< GPIO_T::IEN: IF_EN Position                 */
#define GPIO_IEN_IF_EN_Msk               (0x1ul << GPIO_IEN_IF_EN_Pos)                     /*!< GPIO_T::IEN: IF_EN Mask                     */

#define GPIO_IEN_IR_EN_Pos               (16)                                              /*!< GPIO_T::IEN: IR_EN Position                 */
#define GPIO_IEN_IR_EN_Msk               (0xfful << GPIO_IEN_IR_EN_Pos)                    /*!< GPIO_T::IEN: IR_EN Mask                     */

#define GPIO_ISRC_ISRC_Pos               (0)                                               /*!< GPIO_T::ISRC: ISRC Position                 */
#define GPIO_ISRC_ISRC_Msk               (0xfful << GPIO_ISRC_ISRC_Pos)                    /*!< GPIO_T::ISRC: ISRC Mask                     */

#define GPIO_DBNCECON_DBCLKSEL_Pos       (0)                                               /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSEL Position         */
#define GPIO_DBNCECON_DBCLKSEL_Msk       (0xful << GPIO_DBNCECON_DBCLKSEL_Pos)             /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSEL Mask             */

#define GPIO_DBNCECON_DBCLKSRC_Pos       (4)                                               /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSRC Position         */
#define GPIO_DBNCECON_DBCLKSRC_Msk       (0x1ul << GPIO_DBNCECON_DBCLKSRC_Pos)             /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSRC Mask             */

#define GPIO_DBNCECON_ICLK_ON_Pos        (5)                                               /*!< GPIO_DBNCECON_T::DBNCECON: ICLK_ON Position          */
#define GPIO_DBNCECON_ICLK_ON_Msk        (0x1ul << GPIO_DBNCECON_ICLK_ON_Pos)              /*!< GPIO_DBNCECON_T::DBNCECON: ICLK_ON Mask              */

/**@}*/ /* GP_CONST */
/**@}*/ /* end of GP register group */


/*---------------------- Inter-IC Bus Controller -------------------------*/
/**
    @addtogroup I2C Inter-IC Bus Controller(I2C)
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
     * |[2]     |AA        |Assert Acknowledge Control Bit
     * |        |          |When AA=1 is prior to address or data received, an acknowledged (low level to SDA) will be returned during the acknowledge clock pulse on the SCL line when 1.) A slave is acknowledging the address sent from master, 2.) The receiver devices are acknowledging the data sent by transmitter.
     * |        |          |When AA=0 prior to address or data received, a Not acknowledged (high level to SDA) will be returned during the acknowledge clock pulse on the SCL line.
     * |[3]     |SI        |I2C Interrupt Flag
     * |        |          |When a new I2C state is present in the I2CSTATUS register, the SI flag is set by hardware, and if bit EI (I2CON[7]) is set, the I2C interrupt is requested.
     * |        |          |SI must be cleared by software.
     * |        |          |Software can write 1 to clear this bit.
     * |[4]     |STO       |I2C STOP Control Bit
     * |        |          |In Master mode, setting STO to transmit a STOP condition to bus then I2C hardware will check the bus condition if a STOP condition is detected this bit will be cleared by hardware automatically.
     * |        |          |In Slave mode, setting STO resets I2C hardware to the defined "not addressed" Slave mode.
     * |        |          |This means it is NO LONGER in the Slave receiver mode to receive data from the master transmit device.
     * |[5]     |STA       |I2C START Control Bit
     * |        |          |Setting STA to logic 1 to enter Master mode.
     * |        |          |I2C hardware sends a START or repeats the START condition to bus when the bus is free.
     * |[6]     |ENS1      |I2C Controller Enable Control
     * |        |          |0 = I2C Controller Disabled.
     * |        |          |1 = I2C Controller Enabled.
     * |        |          |Set to enable I2C serial function controller.
     * |        |          |When ENS1=1 the I2C serial function enables.
     * |        |          |The function of multi-function pin must be set to I2C first.
     * |[7]     |EI        |Interrupt Enable Control
     * |        |          |0 = I2C interrupt Disabled.
     * |        |          |1 = I2C interrupt Enabled.
     * @var I2C_T::I2CADDR0
     * Offset: 0x04  I2C Slave Address Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Bits
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In Slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     * @var I2C_T::I2CDAT
     * Offset: 0x08  I2C DATA Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2CDAT    |I2C Data Bits
     * |        |          |Bit [7:0] is located with the 8-bit transferred data of the I2C serial port.
     * @var I2C_T::I2CSTATUS
     * Offset: 0x0C  I2C Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2CSTATUS |I2C Status Bits
     * |        |          |The three least significant bits are always 0.
     * |        |          |The five most significant bits contain the status code.
     * |        |          |There are 26 possible status codes.
     * |        |          |When I2CSTATUS contains F8H, no serial interrupt is requested.
     * |        |          |All the other I2CSTATUS values correspond to defined I2C states.
     * |        |          |When each of these states is entered, a status interrupt is requested (SI = 1).
     * |        |          |A valid status code is present in I2CSTATUS one cycle after SI is set by hardware and is still present one cycle after SI has been reset by software.
     * |        |          |In addition, the states 00H stands for a Bus Error.
     * |        |          |A Bus Error occurs when a START or STOP condition is present at an illegal position in the formation frame.
     * |        |          |Examples of illegal position are during the serial transfer of an address byte, a data byte or an acknowledge bit.
     * @var I2C_T::I2CLK
     * Offset: 0x10  I2C Clock Divided Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |I2CLK     |I2C Clock Divided Bits
     * |        |          |The I2C clock rate bits: Data Baud Rate of I2C = (system clock) / (4x (I2CLK+1)).
     * |        |          |Note: The minimum value of I2CLK is 4.
     * @var I2C_T::I2CTOC
     * Offset: 0x14  I2C Time-Out Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Time-out Flag
     * |        |          |This bit is set by hardware when I2C time-out happened and it can interrupt CPU if I2C interrupt enable bit (EI) is set to 1.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[1]     |DIV4      |Time-out Counter Input Clock Divided By 4
     * |        |          |0 = Time-out counter input clock divided by 4 Disabled.
     * |        |          |1 = Time-out counter input clock divided by 4 Enabled.
     * |        |          |Note: When enabled, the time-out period is extended 4 times.
     * |[2]     |ENTI      |Time-out Counter Enable Control
     * |        |          |0 = Time-out counter Disabled.
     * |        |          |1 = Time-out counter Enabled.
     * |        |          |Note: When the 14-bit time-out counter is enabled, it will start counting when SI is clear.
     * |        |          |Setting 1to the SI flag will reset counter and re-start up counting after SI is cleared.
     * @var I2C_T::I2CADDR1
     * Offset: 0x18  I2C Slave Address Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Bits
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In Slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     * @var I2C_T::I2CADDR2
     * Offset: 0x1C  I2C Slave Address Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Bits
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In Slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     * @var I2C_T::I2CADDR3
     * Offset: 0x20  I2C Slave Address Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |I2CADDR   |I2C Address Bits
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In Slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
     * @var I2C_T::I2CADM0
     * Offset: 0x24  I2C Slave Address Mask Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Bits
     * |        |          |0 = I2C address mask Disabled (the received corresponding register bit should be exactly the same as address register).
     * |        |          |1 = I2C address mask Enabled (the received corresponding address bit is "Don't care").
     * @var I2C_T::I2CADM1
     * Offset: 0x28  I2C Slave Address Mask Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Bits
     * |        |          |0 = I2C address mask Disabled (the received corresponding register bit should be exactly the same as address register).
     * |        |          |1 = I2C address mask Enabled (the received corresponding address bit is "Don't care").
     * @var I2C_T::I2CADM2
     * Offset: 0x2C  I2C Slave Address Mask Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Bits
     * |        |          |0 = I2C address mask Disabled (the received corresponding register bit should be exactly the same as address register).
     * |        |          |1 = I2C address mask Enabled (the received corresponding address bit is "Don't care").
     * @var I2C_T::I2CADM3
     * Offset: 0x30  I2C Slave Address Mask Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |I2CADM    |I2C Address Mask Bits
     * |        |          |0 = I2C address mask Disabled (the received corresponding register bit should be exactly the same as address register).
     * |        |          |1 = I2C address mask Enabled (the received corresponding address bit is "Don't care").
     * @var I2C_T::I2CON2
     * Offset: 0x3C  I2C Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WAKEUPEN  |Wake-up Enable Control
     * |        |          |0 = I2C wake-up function Disabled.
     * |        |          |1 = I2C wake-up function Enabled.
     * |        |          |The system can be wake-up by I2C bus when the system is set into power mode and the received data matched one of the addresses in Address Register.
     * |[1]     |TWOFF_EN  |TWO LEVEL FIFO Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Set to enable the two-level FIFO for I2C transmitted or received buffer.
     * |        |          |It is used to improve the performance of the I2C bus.
     * |        |          |If this bit is set = 1, the control bit of STA for repeat start or STO bit should be set after the current SI is clear.
     * |        |          |For example: if there are 4 data shall be transmitted and then stop it.
     * |        |          |The STO bit shall be set after the 3rd data's SI event being clear.
     * |        |          |In this time, the 4th data can be transmitted and the I2C stop after the 4th data transmission done.
     * |[2]     |NOSTRETCH |NO STRETCH The I2C BUS
     * |        |          |0 = The I2C SCL bus is stretched by hardware if the SI is not cleared in master mode.
     * |        |          |1 = The I2C SCL bus is not stretched by hardware if the SI is not cleared in master mode.
     * |[3]     |OVER_INTEN|I2C OVER RUN Interrupt Control Bit
     * |        |          |Setting OVER_INTEN to enable will send a interrupt to system when the TWOFF bit is enabled and there is over run event in received FIFO.
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[4]     |UNDER_INTEN|I2C UNDER RUN Interrupt Control Bit
     * |        |          |Setting UNDER_INTEN to enable will send a interrupt to system when the TWOFF bit is enabled and there is under run event happened in transmitted FIFO.
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * @var I2C_T::I2CSTATUS2
     * Offset: 0x40  I2C Status Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WAKEUP    |I2C Wake-up Interrupt Flag
     * |        |          |When chip is woken up from Power-Down mode by I2C, this bit is set to 1.
     * |        |          |Software can write 1 to clear this bit.
     * |[1]     |FULL      |I2C TWO LEVEL FIFO FULL
     * |        |          |This bit indicates TX FIFO full or not when the TWOFF_EN = 1.
     * |[2]     |EMPTY     |I2C TWO LEVEL FIFO EMPTY
     * |        |          |This bit indicates RX FIFO empty or not when the TWOFF_EN = 1.
     * |[3]     |OVERUN    |I2C OVER RUN Status Bit
     * |        |          |This bit indicates the received FIFO is over run when the TWOFF_EN = 1.
     * |[4]     |UNDERUN   |I2C UNDER RUN Status Bit
     * |        |          |This bit indicates the transmitted FIFO is under run when the TWOFF_EN = 1.
     */

    __IO uint32_t I2CON;         /* Offset: 0x00  I2C Control Register                                               */
    __IO uint32_t I2CADDR0;      /* Offset: 0x04  I2C Slave Address Register 0                                       */
    __IO uint32_t I2CDAT;        /* Offset: 0x08  I2C DATA Register                                                  */
    __I  uint32_t I2CSTATUS;     /* Offset: 0x0C  I2C Status Register                                                */
    __IO uint32_t I2CLK;         /* Offset: 0x10  I2C Clock Divided Register                                         */
    __IO uint32_t I2CTOC;        /* Offset: 0x14  I2C Time-Out Counter Register                                      */
    __IO uint32_t I2CADDR1;      /* Offset: 0x18  I2C Slave Address Register 1                                       */
    __IO uint32_t I2CADDR2;      /* Offset: 0x1C  I2C Slave Address Register 2                                       */
    __IO uint32_t I2CADDR3;      /* Offset: 0x20  I2C Slave Address Register 3                                       */
    __IO uint32_t I2CADM0;       /* Offset: 0x24  I2C Slave Address Mask Register 0                                  */
    __IO uint32_t I2CADM1;       /* Offset: 0x28  I2C Slave Address Mask Register 1                                  */
    __IO uint32_t I2CADM2;       /* Offset: 0x2C  I2C Slave Address Mask Register 2                                  */
    __IO uint32_t I2CADM3;       /* Offset: 0x30  I2C Slave Address Mask Register 3                                  */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0[2];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t I2CON2;        /* Offset: 0x3C  I2C Control Register 2                                             */
    __IO uint32_t I2CSTATUS2;    /* Offset: 0x40  I2C Status Register 2                                              */

} I2C_T;



/**
    @addtogroup I2C_CONST I2C Bit Field Definition
    Constant Definitions for I2C Controller
@{ */

#define I2C_I2CON_AA_Pos                 (2)                                               /*!< I2C_T::I2CON: AA Position                 */
#define I2C_I2CON_AA_Msk                 (0x1ul << I2C_I2CON_AA_Pos)                       /*!< I2C_T::I2CON: AA Mask                     */

#define I2C_I2CON_SI_Pos                 (3)                                               /*!< I2C_T::I2CON: SI Position                 */
#define I2C_I2CON_SI_Msk                 (0x1ul << I2C_I2CON_SI_Pos)                       /*!< I2C_T::I2CON: SI Mask                     */

#define I2C_I2CON_STO_Pos                (4)                                               /*!< I2C_T::I2CON: STO Position                */
#define I2C_I2CON_STO_Msk                (0x1ul << I2C_I2CON_STO_Pos)                      /*!< I2C_T::I2CON: STO Mask                    */

#define I2C_I2CON_STA_Pos                (5)                                               /*!< I2C_T::I2CON: STA Position                */
#define I2C_I2CON_STA_Msk                (0x1ul << I2C_I2CON_STA_Pos)                      /*!< I2C_T::I2CON: STA Mask                    */

#define I2C_I2CON_ENSI_Pos               (6)                                               /*!< I2C_T::I2CON: ENSI Position               */
#define I2C_I2CON_ENSI_Msk               (0x1ul << I2C_I2CON_ENSI_Pos)                     /*!< I2C_T::I2CON: ENSI Mask                   */

#define I2C_I2CON_EI_Pos                 (7)                                               /*!< I2C_T::I2CON: EI Position                 */
#define I2C_I2CON_EI_Msk                 (0x1ul << I2C_I2CON_EI_Pos)                       /*!< I2C_T::I2CON: EI Mask                     */

#define I2C_I2CADDR_GC_Pos               (0)                                               /*!< I2C_T::I2CADDR0: GC Position               */
#define I2C_I2CADDR_GC_Msk               (0x1ul << I2C_I2CADDR_GC_Pos)                     /*!< I2C_T::I2CADDR0: GC Mask                   */

#define I2C_I2CADDR_I2CADDR_Pos          (1)                                               /*!< I2C_T::I2CADDR0: I2CADDR Position          */
#define I2C_I2CADDR_I2CADDR_Msk          (0x7ful << I2C_I2CADDR_I2CADDR_Pos)               /*!< I2C_T::I2CADDR0: I2CADDR Mask              */

#define I2C_I2CDAT_I2CDAT_Pos            (0)                                               /*!< I2C_T::I2CDAT: I2CDAT Position            */
#define I2C_I2CDAT_I2CDAT_Msk            (0xfful << I2C_I2CDAT_I2CDAT_Pos)                 /*!< I2C_T::I2CDAT: I2CDAT Mask                */

#define I2C_I2CSTATUS_I2CSTATUS_Pos      (0)                                               /*!< I2C_T::I2CSTATUS: I2CSTATUS Position      */
#define I2C_I2CSTATUS_I2CSTATUS_Msk      (0xfful << I2C_I2CSTATUS_I2CSTATUS_Pos)           /*!< I2C_T::I2CSTATUS: I2CSTATUS Mask          */

#define I2C_I2CLK_I2CLK_Pos              (0)                                               /*!< I2C_T::I2CLK: I2CLK Position              */
#define I2C_I2CLK_I2CLK_Msk              (0xfful << I2C_I2CLK_I2CLK_Pos)                   /*!< I2C_T::I2CLK: I2CLK Mask                  */

#define I2C_I2CTOC_TIF_Pos               (0)                                               /*!< I2C_T::I2CTOC: TIF Position               */
#define I2C_I2CTOC_TIF_Msk               (0x1ul << I2C_I2CTOC_TIF_Pos)                     /*!< I2C_T::I2CTOC: TIF Mask                   */

#define I2C_I2CTOC_DIV4_Pos              (1)                                               /*!< I2C_T::I2CTOC: DIV4 Position              */
#define I2C_I2CTOC_DIV4_Msk              (0x1ul << I2C_I2CTOC_DIV4_Pos)                    /*!< I2C_T::I2CTOC: DIV4 Mask                  */

#define I2C_I2CTOC_ENTI_Pos              (2)                                               /*!< I2C_T::I2CTOC: ENTI Position              */
#define I2C_I2CTOC_ENTI_Msk              (0x1ul << I2C_I2CTOC_ENTI_Pos)                    /*!< I2C_T::I2CTOC: ENTI Mask                  */

#define I2C_I2CADM_I2CADM_Pos            (1)                                               /*!< I2C_T::I2CADM0: I2CADM Position            */
#define I2C_I2CADM_I2CADM_Msk            (0x7ful << I2C_I2CADM_I2CADM_Pos)                 /*!< I2C_T::I2CADM0: I2CADM Mask                */

#define I2C_I2CON2_WKUPEN_Pos            (0)                                               /*!< I2C_T::I2CON2: WKUPEN Position            */
#define I2C_I2CON2_WKUPEN_Msk            (0x1ul << I2C_I2CON2_WKUPEN_Pos)                  /*!< I2C_T::I2CON2: WKUPEN Mask                */

#define I2C_I2CON2_TWOFF_EN_Pos          (1)                                               /*!< I2C_T::I2CON2: TWOFF_EN Position          */
#define I2C_I2CON2_TWOFF_EN_Msk          (0x1ul << I2C_I2CON2_TWOFF_EN_Pos)                /*!< I2C_T::I2CON2: TWOFF_EN Mask              */

#define I2C_I2CON2_NOSTRETCH_Pos         (2)                                               /*!< I2C_T::I2CON2: NOSTRETCH Position         */
#define I2C_I2CON2_NOSTRETCH_Msk         (0x1ul << I2C_I2CON2_NOSTRETCH_Pos)               /*!< I2C_T::I2CON2: NOSTRETCH Mask             */

#define I2C_I2CON2_OVER_INTEN_Pos        (3)                                               /*!< I2C_T::I2CON2: OVER_INTEN Position        */
#define I2C_I2CON2_OVER_INTEN_Msk        (0x1ul << I2C_I2CON2_OVER_INTEN_Pos)              /*!< I2C_T::I2CON2: OVER_INTEN Mask            */

#define I2C_I2CON2_UNDER_INTEN_Pos       (4)                                               /*!< I2C_T::I2CON2: UNDER_INTEN Position       */
#define I2C_I2CON2_UNDER_INTEN_Msk       (0x1ul << I2C_I2CON2_UNDER_INTEN_Pos)             /*!< I2C_T::I2CON2: UNDER_INTEN Mask           */

#define I2C_I2CSTATUS2_WAKEUP_Pos        (0)                                               /*!< I2C_T::I2CSTATUS2: WAKEUP Position        */
#define I2C_I2CSTATUS2_WAKEUP_Msk        (0x1ul << I2C_I2CSTATUS2_WAKEUP_Pos)              /*!< I2C_T::I2CSTATUS2: WAKEUP Mask            */

#define I2C_I2CSTATUS2_FULL_Pos          (1)                                               /*!< I2C_T::I2CSTATUS2: FULL Position          */
#define I2C_I2CSTATUS2_FULL_Msk          (0x1ul << I2C_I2CSTATUS2_FULL_Pos)                /*!< I2C_T::I2CSTATUS2: FULL Mask              */

#define I2C_I2CSTATUS2_EMPTY_Pos         (2)                                               /*!< I2C_T::I2CSTATUS2: EMPTY Position         */
#define I2C_I2CSTATUS2_EMPTY_Msk         (0x1ul << I2C_I2CSTATUS2_EMPTY_Pos)               /*!< I2C_T::I2CSTATUS2: EMPTY Mask             */

#define I2C_I2CSTATUS2_OVERUN_Pos        (3)                                               /*!< I2C_T::I2CSTATUS2: OVERUN Position        */
#define I2C_I2CSTATUS2_OVERUN_Msk        (0x1ul << I2C_I2CSTATUS2_OVERUN_Pos)              /*!< I2C_T::I2CSTATUS2: OVERUN Mask            */

#define I2C_I2CSTATUS2_UNDERUN_Pos       (4)                                               /*!< I2C_T::I2CSTATUS2: UNDERUN Position       */
#define I2C_I2CSTATUS2_UNDERUN_Msk       (0x1ul << I2C_I2CSTATUS2_UNDERUN_Pos)             /*!< I2C_T::I2CSTATUS2: UNDERUN Mask           */

/**@}*/ /* I2C_CONST */
/**@}*/ /* end of I2C register group */


/*---------------------- INT Controller -------------------------*/
/**
    @addtogroup INT Interrupt Controller (INT)
    Memory Mapped Structure for INT Controller
@{ */



typedef struct
{

    /**
     * @var INT_T::SRC0
     * Offset: 0x00  IRQ0 (BOD) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC1
     * Offset: 0x04  IRQ1 (WDT) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC2
     * Offset: 0x08  IRQ2 (EINT0) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC3
     * Offset: 0x0C  IRQ3 (EINT1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC4
     * Offset: 0x10  IRQ4 (GP0/1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC5
     * Offset: 0x14  IRQ5 (GP2/3/4) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC6
     * Offset: 0x18  IRQ6 (PWM) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC7
     * Offset: 0x1C  IRQ7 (BRAKE) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC8
     * Offset: 0x20  IRQ8 (TMR0) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC9
     * Offset: 0x24  IRQ9 (TMR1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC12
     * Offset: 0x30  IRQ12 (UART) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC14
     * Offset: 0x38  IRQ14 (SPI) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC16
     * Offset: 0x40  IRQ16 (GP5) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC17
     * Offset: 0x44  IRQ17 (HIRC trim) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC18
     * Offset: 0x48  IRQ18 (I2C) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC25
     * Offset: 0x64  IRQ25 (ACMP) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC28
     * Offset: 0x70  IRQ28 (PWRWU) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::SRC29
     * Offset: 0x74  IRQ29 (ADC) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
     * @var INT_T::NMICON
     * Offset: 0x80  NMI Source Interrupt Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |NMI_SEL   |NMI Interrupt Source Selection
     * |        |          |The NMI interrupt to Cortex-M0 can be selected from one of the peripheral interrupt by setting NMI_SEL.
     * |[8]     |NMI_SEL_EN|NMI Interrupt Enable Control (Write Protect)
     * |        |          |0 = NMI interrupt Disabled.
     * |        |          |1 = NMI interrupt Enabled.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address GCR_BA+0x100.
     * @var INT_T::MCUIRQ
     * Offset: 0x84  MCU IRQ Number Identity Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |MCU_IRQ   |MCU IRQ Source
     * |        |          |The MCU_IRQ collects all the interrupts from the peripherals and generates the synchronous interrupt to Cortex-M0 core.
     * |        |          |This modes to generate interrupt to Cortex-M0 - the normal mode.
     * |        |          |The MCU_IRQ collects all interrupts from each peripheral and synchronizes them then interrupts the Cortex-M0.
     * |        |          |When the MCU_IRQ[n] is 0, setting MCU_IRQ[n] to 1 will generate an interrupt to Cortex-M0 NVIC[n].
     * |        |          |When the MCU_IRQ[n] is 1 (mean an interrupt is assert), setting 1 to the MCU_bit[n] will clear the interrupt and setting MCU_IRQ[n] 0 has no effect.
     */


    __I  uint32_t SRC0;           /* Offset: 0x00  IRQ0 (BOD) Interrupt Source Identity                               */
    __I  uint32_t SRC1;           /* Offset: 0x04  IRQ1 (WDT) Interrupt Source Identity                               */
    __I  uint32_t SRC2;           /* Offset: 0x08  IRQ2 (EINT0) Interrupt Source Identity                             */
    __I  uint32_t SRC3;           /* Offset: 0x0C  IRQ3 (EINT1) Interrupt Source Identity                             */
    __I  uint32_t SRC4;           /* Offset: 0x10  IRQ4 (GP0/1) Interrupt Source Identity                             */
    __I  uint32_t SRC5;           /* Offset: 0x14  IRQ5 (GP2/3/4) Interrupt Source Identity                           */
    __I  uint32_t SRC6;           /* Offset: 0x18  IRQ6 (PWM) Interrupt Source Identity                               */
    __I  uint32_t SRC7;           /* Offset: 0x1C  IRQ7 (BRAKE) Interrupt Source Identity                             */
    __I  uint32_t SRC8;           /* Offset: 0x20  IRQ8 (TMR0) Interrupt Source Identity                              */
    __I  uint32_t SRC9;           /* Offset: 0x24  IRQ9 (TMR1) Interrupt Source Identity                              */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED0[2];
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t SRC12;           /* Offset: 0x30  IRQ12 (UART) Interrupt Source Identity                            */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED1;
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t SRC14;           /* Offset: 0x38  IRQ14 (SPI) Interrupt Source Identity                             */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED2;
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t SRC16;           /* Offset: 0x40  IRQ16 (GP5) Interrupt Source Identity                             */
    __I  uint32_t SRC17;           /* Offset: 0x44  IRQ17 (HIRC trim) Interrupt Source Identity                       */
    __I  uint32_t SRC18;           /* Offset: 0x48  IRQ18 (I2C) Interrupt Source Identity                             */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED3[6];
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t SRC25;           /* Offset: 0x64  IRQ25 (ACMP) Interrupt Source Identity                            */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED4[2];
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t SRC28;           /* Offset: 0x70  IRQ28 (PWRWU) Interrupt Source Identity                           */
    __I  uint32_t SRC29;           /* Offset: 0x74  IRQ29 (ADC) Interrupt Source Identity                             */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVED5[2];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t NMICON;          /* Offset: 0x80  NMI Source Interrupt Select Control Register                      */
    __IO uint32_t MCUIRQ;          /* Offset: 0x84  MCU IRQ Number Identity Register                                  */

} INT_T;



/**
    @addtogroup INT_CONST INT Bit Field Definition
    Constant Definitions for INT Controller
@{ */

#define INT_SRC_INT_SRC_Pos              (0)                                               /*!< INT_T::SRC0: INT_SRC Position              */
#define INT_SRC_INT_SRC_Msk              (0x7ul << INT_SRC_INT_SRC_Pos)                    /*!< INT_T::SRC0: INT_SRC Mask                  */

#define INT_CON_NMI_SEL_Pos              (0)                                               /*!< INT_T::NMICON: NMI_SEL Position              */
#define INT_CON_NMI_SEL_Msk              (0x1ful << INT_CON_NMI_SEL_Pos)                   /*!< INT_T::NMICON: NMI_SEL Mask                  */

#define INT_CON_NMI_SEL_EN_Pos           (8)                                               /*!< INT_T::NMICON: NMI_SEL_EN Position           */
#define INT_CON_NMI_SEL_EN_Msk           (0x1ul << INT_CON_NMI_SEL_EN_Pos)                 /*!< INT_T::NMICON: NMI_SEL_EN Mask               */

#define INT_IRQ_MCU_IRQ_Pos              (0)                                               /*!< INT_T::MCUIRQ: MCU_IRQ Position              */
#define INT_IRQ_MCU_IRQ_Msk              (0xfffffffful << INT_IRQ_MCU_IRQ_Pos)             /*!< INT_T::MCUIRQ: MCU_IRQ Mask                  */

/**@}*/ /* INT_CONST */
/**@}*/ /* end of INT register group */


/*---------------------- Pulse Width Modulation Controller -------------------------*/
/**
    @addtogroup PWM Pulse Width Modulation Controller(PWM)
    Memory Mapped Structure for PWM Controller
@{ */


typedef struct
{

    /**
     * @var PWM_T::PPR
     * Offset: 0x00  PWM Pre-scale Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CP01      |Clock Prescaler 0 For PWM Counter 0 And 1
     * |        |          |Clock input is divided by (CP01 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CP01 = 0, the clock prescaler 0 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
     * |[15:8]  |CP23      |Clock Prescaler 2 For PWM Counter 2 And 3
     * |        |          |Clock input is divided by (CP23 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CP23 = 0, the clock prescaler 2 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
     * |[23:16] |CP45      |Clock Prescaler 4 For PWM Counter 4 And 5
     * |        |          |Clock input is divided by (CP45 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CP45 = 0, the clock prescaler 4 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
     * @var PWM_T::CSR
     * Offset: 0x04  PWM Clock Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CSR0      |Timer 0 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |(Table is the same as CSR5.)
     * |[6:4]   |CSR1      |Timer 1 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |(Table is the same as CSR5.)
     * |[10:8]  |CSR2      |Timer 2 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |(Table is the same as CSR5.)
     * |[14:12] |CSR3      |Timer 3 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |(Table is the same as CSR5.)
     * |[18:16] |CSR4      |Timer 4 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |(Table is the same as CSR5.)
     * |[22:20] |CSR5      |Timer 5 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |000 = Input Clock Divided by 2.
     * |        |          |001 = Input Clock Divided by 4.
     * |        |          |010 = Input Clock Divided by 8.
     * |        |          |011 = Input Clock Divided by 16.
     * |        |          |100 = Input Clock Divided by 1.
     * @var PWM_T::PCR
     * Offset: 0x08  PWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CH0EN     |PWM-timer 0 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[1]     |DB_MODE   |PWM Debug Mode Configuration Bit (Available In DEBUG Mode Only)
     * |        |          |0 = Safe mode: The timer is frozen and PWM outputs are shut down Safe state for the inverter.
     * |        |          |The timer can still be re-started from where it stops.
     * |        |          |1 = Normal mode: The timer continues to operate normally May be dangerous in some cases since a constant duty cycle is applied to the inverter (no more interrupts serviced).
     * |[2]     |CH0INV    |PWM-timer 0 Output Inverter Enable Control
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[3]     |CH0MOD    |PWM-timer 0 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause CNR0 and CMR0 cleared.
     * |[4]     |CH1EN     |PWM-timer 1 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[6]     |CH1INV    |PWM-timer 1 Output Inverter ON/OFF
     * |        |          |0 = Inverter OFF.
     * |        |          |1 = Inverter ON.
     * |[7]     |CH1MOD    |PWM-timer 1 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause CNR1 and CMR1 cleared.
     * |[8]     |CH2EN     |PWM-timer 2 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[10]    |CH2INV    |PWM-timer 2 Output Inverter Enable Control
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[11]    |CH2MOD    |PWM-timer 2 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause CNR2 and CMR2 cleared.
     * |[12]    |CH3EN     |PWM-timer 3 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[14]    |CH3INV    |PWM-timer 3 Output Inverter Enable Control
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[15]    |CH3MOD    |PWM-timer 3 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause CNR3 and CMR3 cleared.
     * |[16]    |CH4EN     |PWM-timer 4 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[18]    |CH4INV    |PWM-timer 4 Output Inverter Enable Control
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[19]    |CH4MOD    |PWM-timer 4 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause CNR4 and CMR4 cleared.
     * |[20]    |CH5EN     |PWM-timer 5 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[22]    |CH5INV    |PWM-timer 5 Output Inverter Enable Control
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[23]    |CH5MOD    |PWM-timer 5 Auto-reload/One-shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause CNR5 and CMR5 cleared.
     * |[24]    |DZEN01    |Dead-zone 0 Generator Enable Control (PWM0 And PWM1 Pair For PWM Group)
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: When the dead-zone generator is enabled, the pair of PWM0 and PWM1 becomes a complementary pair for PWM group.
     * |[25]    |DZEN23    |Dead-zone 2 Generator Enable Control (PWM2 And PWM3 Pair For PWM Group)
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: When the dead-zone generator is enabled, the pair of PWM2 and PWM3 becomes a complementary pair for PWM group.
     * |[26]    |DZEN45    |Dead-zone 4 Generator Enable Control (PWM4 And PWM5 Pair For PWM Group)
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: When the dead-zone generator is enabled, the pair of PWM4 and PWM5 becomes a complementary pair for PWM group.
     * |[27]    |CLRPWM    |Clear PWM Counter Control Bit
     * |        |          |0 = Do not clear PWM counter.
     * |        |          |1 = All 16-bit PWM counters cleared to 0x0000.
     * |        |          |Note: It is automatically cleared by hardware.
     * |[29:28] |PWMMOD    |PWM Operating Mode Selection
     * |        |          |00 = Independent mode.
     * |        |          |01 = Complementary mode.
     * |        |          |10 = Synchronized mode.
     * |        |          |11 = Reserved.
     * |[30]    |GRP       |Group Bit
     * |        |          |0 = The signals timing of all PWM channels are independent.
     * |        |          |1 = Unify the signals timing of PWM0, PWM2 and PWM4 in the same phase which is controlled by PWM0 and also unify the signals timing of PWM1, PWM3 and PWM5 in the same phase which is controlled by PWM1.
     * |[31]    |PWMTYPE   |PWM Aligned Type Selection Bit
     * |        |          |0 = Edge-aligned type.
     * |        |          |1 = Center-aligned type.
     * @var PWM_T::CNR
     * Offset: 0x0C ~ 0x20 PWM Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CNRn      |PWM Counter/Timer Loaded Value
     * |        |          |CNRn determines the PWM period. n = 0, 1..5.
     * |        |          |Edge-aligned mode:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/(CNRn+1); where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |Duty ratio = (CMRn+1)/(CNRn+1).
     * |        |          |CMRn >= CNRn: PWM output is always high.
     * |        |          |CMRn < CNRn: PWM low width = (CNRn-CMRn) unit; PWM high width = (CMR+1) unit.
     * |        |          |CMRn = 0: PWM low width = (CNRn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned mode:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*CNRn+1); where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |Duty ratio = (CNRn - CMRn)/(CNRn+1).
     * |        |          |CMRn >= CNRn: PWM output is always low.
     * |        |          |CMRn < CNRn: PWM low width = (CMRn + 1) * 2 unit; PWM high width = (CNRn - CMRn) * 2 unit.
     * |        |          |CMRn = 0: PWM low width = 2 unit; PWM high width = (CNRn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CNRn will take effect in next PWM cycle.
     * @var PWM_T::CMR
     * Offset: 0x24 ~ 0x38 PWM Comparator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMRn      |PWM Comparator Bits
     * |        |          |CMR determines the PWM duty. n = 0, 1..5.
     * |        |          |Edge-aligned mode:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/(CNRn+1); where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |Duty ratio = (CMRn+1)/(CNRn+1).
     * |        |          |CMRn >= CNRn: PWM output is always high.
     * |        |          |CMRn < CNRn: PWM low width = (CNRn-CMRn) unit; PWM high width = (CMR+1) unit.
     * |        |          |CMRn = 0: PWM low width = (CNRn) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned mode:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider)) /(2*CNRn+1); where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |Duty ratio = (CNRn - CMRn)/(CNRn+1).
     * |        |          |CMRn >= CNRn: PWM output is always low.
     * |        |          |CMRn < CNRn: PWM low width = (CMRn + 1) * 2 unit; PWM high width = (CNRn - CMRn) * 2 unit.
     * |        |          |CMRn = 0: PWM low width = 2 unit; PWM high width = (CNRn) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMRn will take effect in next PWM cycle.
     * @var PWM_T::PIER
     * Offset: 0x54  PWM Interrupt Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWMPIE0   |PWM Channel 0 Period Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[1]     |PWMPIE1   |PWM Channel 1 Period Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[2]     |PWMPIE2   |PWM Channel 2 Period Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[3]     |PWMPIE3   |PWM Channel 3 Period Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[4]     |PWMPIE4   |PWM Channel 4 Period Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[5]     |PWMPIE5   |PWM Channel 5 Period Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[8]     |PWMDIE0   |PWM Channel 0 Duty Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[9]     |PWMDIE1   |PWM Channel 1 Duty Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[10]    |PWMDIE2   |PWM Channel 2 Duty Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[11]    |PWMDIE3   |PWM Channel 3 Duty Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[12]    |PWMDIE4   |PWM Channel 4 Duty Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[13]    |PWMDIE5   |PWM Channel 5 Duty Interrupt Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[16]    |BRKIE     |Fault Brake0 And 1 Interrupt Enable Control
     * |        |          |0 = Disabling flags BKF0 and BKF1 to trigger PWM interrupt.
     * |        |          |1 = Enabling flags BKF0 and BKF1 can trigger PWM interrupt.
     * |[17]    |INT_TYPE  |PWM Interrupt Type Selection Bit
     * |        |          |0 = PWMPIFn will be set if PWM counter underflows.
     * |        |          |1 = PWMPIFn will be set if PWM counter matches CNRn register.
     * |        |          |Note: This bit is effective when PWM in central align mode only.
     * @var PWM_T::PIIR
     * Offset: 0x58  PWM Interrupt Indication Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWMPIF0   |PWM Channel 0 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM0 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[1]     |PWMPIF1   |PWM Channel 1 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM1 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[2]     |PWMPIF2   |PWM Channel 2 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM2 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[3]     |PWMPIF3   |PWM Channel 3 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM3 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[4]     |PWMPIF4   |PWM Channel 4 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM4 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[5]     |PWMPIF5   |PWM Channel 5 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM5 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[8]     |PWMDIF0   |PWM Channel 0 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 0 PWM counter reaches CMR0 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[9]     |PWMDIF1   |PWM Channel 1 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 1 PWM counter reaches CMR1 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[10]    |PWMDIF2   |PWM Channel 2 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 2 PWM counter reaches CMR2 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[11]    |PWMDIF3   |PWM Channel 3 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 3 PWM counter reaches CMR3 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[12]    |PWMDIF4   |PWM Channel 4 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 4 PWM counter reaches CMR4 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[13]    |PWMDIF5   |PWM Channel 5 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 5 PWM counter reaches CMR5 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[16]    |BKF0      |PWM Brake0 Flag
     * |        |          |0 = PWM Brake does not recognize a falling signal at BKP0.
     * |        |          |1 = When PWM Brake detects a falling signal at pin BKP0, this flag will be set to high.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[17]    |BKF1      |PWM Brake1 Flag
     * |        |          |0 = PWM Brake does not recognize a falling signal at BKP1.
     * |        |          |1 = When PWM Brake detects a falling signal at pin BKP1, this flag will be set to high.
     * |        |          |Note: Software can write 1 to clear this bit.
     * @var PWM_T::POE
     * Offset: 0x5C  PWM Output Enable for Channel 0~5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PWM0      |PWM Channel 0 Output Enable Control
     * |        |          |0 = PWM channel 0 output to pin Disabled.
     * |        |          |1 = PWM channel 0 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
     * |[1]     |PWM1      |PWM Channel 1 Output Enable Control
     * |        |          |0 = PWM channel 1 output to pin Disabled.
     * |        |          |1 = PWM channel 1 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
     * |[2]     |PWM2      |PWM Channel 2 Output Enable Control
     * |        |          |0 = PWM channel 2 output to pin Disabled.
     * |        |          |1 = PWM channel 2 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
     * |[3]     |PWM3      |PWM Channel 3 Output Enable Control
     * |        |          |0 = PWM channel 3 output to pin Disabled.
     * |        |          |1 = PWM channel 3 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
     * |[4]     |PWM4      |PWM Channel 4 Output Enable Control
     * |        |          |0 = PWM channel 4 output to pin Disabled.
     * |        |          |1 = PWM channel 4 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
     * |[5]     |PWM5      |PWM Channel 5 Output Enable Control
     * |        |          |0 = PWM channel 5 output to pin Disabled.
     * |        |          |1 = PWM channel 5 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
     * @var PWM_T::PFBCON
     * Offset: 0x60  PWM Fault Brake Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BKEN0     |Enable BKP0 Pin Trigger Fault Brake Function 0
     * |        |          |0 = Disabling BKP0 pin can trigger brake function 0 (EINT0 or CPO1).
     * |        |          |1 = Enabling a falling at BKP0 pin can trigger brake function 0.
     * |[1]     |BKEN1     |Enable BKP1 Pin Trigger Fault Brake Function 1
     * |        |          |0 = Disabling BKP1 pin can trigger brake function 1 (EINT1 or CPO0).
     * |        |          |1 = Enabling a falling at BKP1 pin can trigger brake function 1.
     * |[2]     |CPO0BKEN  |BKP1 Fault Brake Function Source Selection
     * |        |          |0 = EINT1 as one brake source in BKP1.
     * |        |          |1 = CPO0 as one brake source in BKP1.
     * |[3]     |CPO1BKEN  |BKP0 Fault Brake Function Source Selection
     * |        |          |0 = EINT0 as one brake source in BKP0.
     * |        |          |1 = CPO1 as one brake source in BKP0.
     * |[7]     |BKF       |PWM Fault Brake Event Flag (Write 1 Clear)
     * |        |          |0 = PWM output initial state when fault brake conditions asserted.
     * |        |          |1 = PWM output fault brake state when fault brake conditions asserted.
     * |        |          |Software can write 1 to clear this bit and must clear this bit before restart PWM counter.
     * |[24]    |PWMBKO0   |PWM Channel 0 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[25]    |PWMBKO1   |PWM Channel 1 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[26]    |PWMBKO2   |PWM Channel 2 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[27]    |PWMBKO3   |PWM Channel 3 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[28]    |PWMBKO4   |PWM Channel 4 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[29]    |PWMBKO5   |PWM Channel 5 Brake Output Select Bit
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[30]    |D6BKO6    |D6 Brake Output Select Bit
     * |        |          |0 = D6 output low when fault brake conditions asserted.
     * |        |          |1 = D6 output high when fault brake conditions asserted.
     * |[31]    |D7BKO7    |D7 Brake Output Select Bit
     * |        |          |0 = D7 output low when fault brake conditions asserted.
     * |        |          |1 = D7 output high when fault brake conditions asserted.
     * @var PWM_T::PDZIR
     * Offset: 0x64  PWM Dead-zone Interval Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DZI01     |Dead-zone Interval Register For Pair Of Channel0 And Channel1 (PWM0 And PWM1 Pair)
     * |        |          |These 8 bits determine dead-zone length.
     * |        |          |The unit time of dead-zone length is received from corresponding CSR bits.
     * |[15:8]  |DZI23     |Dead-zone Interval Register For Pair Of Channel2 And Channel3 (PWM2 And PWM3 Pair)
     * |        |          |These 8 bits determine dead-zone length.
     * |        |          |The unit time of dead-zone length is received from corresponding CSR bits.
     * |[23:16] |DZI45     |Dead-zone Interval Register For Pair Of Channel4 And Channel5 (PWM4 And PWM5 Pair)
     * |        |          |These 8 bits determine dead-zone length.
     * |        |          |The unit time of dead-zone length is received from corresponding CSR bits.
     * @var PWM_T::TRGCON0
     * Offset: 0x68  PWM Trigger Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CM0TRGREN |Enable PWM Trigger ADC Function While Channel0's Counter Matching CMR0 In Up-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[1]     |CNT0TRGEN |Enable PWM Trigger ADC Function While Channel0's Counter Matching CNR0
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[2]     |CM0TRGFEN |Enable PWM Trigger ADC Function While Channel0's Counter Matching CMR0 In Down-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[3]     |P0TRGEN   |Enable PWM Trigger ADC Function While Channel0's Counter Matching 0
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[8]     |CM1TRGREN |Enable PWM Trigger ADC Function While Channel1's Counter Matching CMR1 In Up-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[9]     |CNT1TRGEN |Enable PWM Trigger ADC Function While Channel1's Counter Matching CNR1
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[10]    |CM1TRGFEN |Enable PWM Trigger ADC Function While Channel1's Counter Matching CMR1 In Down-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[11]    |P1TRGEN   |Enable PWM Trigger ADC Function While Channel1's Counter Matching 0
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[16]    |CM2TRGREN |Enable PWM Trigger ADC Function While Channel2's Counter Matching CMR2 In Up-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[17]    |CNT2TRGEN |Enable PWM Trigger ADC Function While Channel2's Counter Matching CNR2
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[18]    |CM2TRGFEN |Enable PWM Trigger ADC Function While Channel2's Counter Matching CMR2 In Down-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[19]    |P2TRGEN   |Enable PWM Trigger ADC Function While Channel2's Counter Matching 0
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[24]    |CM3TRGREN |Enable PWM Trigger ADC Function While Channel3's Counter Matching CMR3 In Up-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[25]    |CNT3TRGEN |Enable PWM Trigger ADC Function While Channel3's Counter Matching CNR3
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[26]    |CM3TRGFEN |Enable PWM Trigger ADC Function While Channel3's Counter Matching CMR3 In Down-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[27]    |P3TRGEN   |Enable PWM Trigger ADC Function While Channel3's Counter Matching 0
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * @var PWM_T::TRGCON1
     * Offset: 0x6C  PWM Trigger Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CM4TRGREN |Enable PWM Trigger ADC Function While Channel4's Counter Matching CMR4 In Up-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[1]     |CNT4TRGEN |Enable PWM Trigger ADC Function While Channel4's Counter Matching CNR4
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[2]     |CM4TRGFEN |Enable PWM Trigger ADC Function While Channel4's Counter Matching CMR4 In Down-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[3]     |P4TRGEN   |Enable PWM Trigger ADC Function While Channel4's Counter Matching 0
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[8]     |CM5TRGREN |Enable PWM Trigger ADC Function While Channel5's Counter Matching CMR5 In Up-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[9]     |CNT5TRGEN |Enable PWM Trigger ADC Function While Channel5's Counter Matching CNR5
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[10]    |CM5TRGFEN |Enable PWM Trigger ADC Function While Channel5's Counter Matching CMR5 In Down-count Direction
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[11]    |P5TRGEN   |Enable PWM Trigger ADC Function While Channel5's Counter Matching 0
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * @var PWM_T::TRGSTS0
     * Offset: 0x70  PWM Trigger Status Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CMR0FLAG_R|ADC Trigger Flag By Counting Up To CMR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[1]     |CNT0FLAG  |ADC Trigger Flag By Counting To CNR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[2]     |CMR0FLAG_F|ADC Trigger Flag By Counting Down To CMR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[3]     |PERID0FLAG|ADC Trigger Flag By Period
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[8]     |CMR1FLAG_R|ADC Trigger Flag By Counting Up To CMR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[9]     |CNT1FLAG  |ADC Trigger Flag By Counting To CNR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[10]    |CMR1FLAG_F|ADC Trigger Flag By Counting Down To CMR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[11]    |PERID1FLAG|ADC Trigger Flag By Period
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[16]    |CMR2FLAG_R|ADC Trigger Flag By Counting Up To CMR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[17]    |CNT2FLAG  |ADC Trigger Flag By Counting To CNR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[18]    |CMR2FLAG_F|ADC Trigger Flag By Counting Down To CMR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[19]    |PERID2FLAG|ADC Trigger Flag By Period
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[24]    |CMR3FLAG_R|When Counter Counting Up To CMR, This Bit Will Be Set For Trigger ADC
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[25]    |CNT3FLAG  |When Counter Counting To CNR, This Bit Will Be Set For Trigger ADC
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[26]    |CMR3FLAG_F|When Counter Counting Down To CMR, This Bit Will Be Set For Trigger ADC
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[27]    |PERID3FLAG|When Counter Counting To Period, This Bit Will Be Set For Trigger ADC
     * |        |          |Note: Software can write 1 to clear this bit.
     * @var PWM_T::TRGSTS1
     * Offset: 0x74  PWM Trigger Status Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CMR4FLAG_R|ADC Trigger Flag By Counting Up To CMR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[1]     |CNT4FLAG  |ADC Trigger Flag By Counting To CNR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[2]     |CMR4FLAG_F|ADC Trigger Flag By Counting Down To CMR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[3]     |PERID4FLAG|ADC Trigger Flag By Period
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[8]     |CMR5FLAG_R|ADC Trigger Flag By Counting Up To CMR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[9]     |CNT5FLAG  |ADC Trigger Flag By Counting To CNR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[10]    |CMR5FLAG_F|ADC Trigger Flag By Counting Down To CMR
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[11]    |PERID5FLAG|ADC Trigger Flag By Period
     * |        |          |Note: Software can write 1 to clear this bit.
     * @var PWM_T::PHCHG
     * Offset: 0x78  Phase Change Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |D0        |D0: When PWM0 Is Zero, Channel 0's Output Waveform Is D0
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[1]     |D1        |D1: When PWM1 Is Zero, Channel 1's Output Waveform Is D1
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[2]     |D2        |D2: When PWM2 Is Zero, Channel 2's Output Waveform Is D2
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[3]     |D3        |D3: When PWM3 Is Zero, Channel 3's Output Waveform Is D3
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[4]     |D4        |D4: When PWM4 Is Zero, Channel 4's Output Waveform Is D4
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[5]     |D5        |D5: When PWM5 Is Zero, Channel 5's Output Waveform Is D5
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[6]     |D6        |D6: When MASK6 Is 1, Channel 6's Output Waveform Is D6
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[7]     |D7        |D7: When MASK7 Is 1, Channel 7's Output Waveform Is D7
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[8]     |PWM0      |PWM Channel 0 Output Enable Control
     * |        |          |0 = Output D0 specified in bit 0 of PHCHG register.
     * |        |          |1 = Output the original channel 0 waveform.
     * |[9]     |PWM1      |PWM Channel 1 Output Enable Control
     * |        |          |0 = Output D1 specified in bit 1 of PHCHG register.
     * |        |          |1 = Output the original channel 1 waveform.
     * |[10]    |PWM2      |PWM Channel 2 Output Enable Control
     * |        |          |0 = Output D2 specified in bit 2 of PHCHG register.
     * |        |          |1 = Output the original channel 2 waveform.
     * |[11]    |PWM3      |PWM Channel 3 Output Enable Control
     * |        |          |0 = Output D3 specified in bit 3 of PHCHG register.
     * |        |          |1 = Output the original channel 3 waveform.
     * |[12]    |PWM4      |PWM Channel 4 Output Enable Control
     * |        |          |0 = Output D4 specified in bit 4 of PHCHG register.
     * |        |          |1 = Output the original channel 4 waveform.
     * |[13]    |PWM5      |PWM Channel 5 Output Enable Control
     * |        |          |0 = Output D5 specified in bit 5 of PHCHG register.
     * |        |          |1 = Output the original channel 5 waveform.
     * |[14]    |ACCNT0    |Hardware Auto Clear CE0 When ACMP0 Trigger It
     * |        |          |0 = Enabled.
     * |        |          |1 = Disabled.
     * |[15]    |ACCNT1    |Hardware Auto Clear CE1 When ACMP1 Trigger It
     * |        |          |0 = Enabled.
     * |        |          |1 = Disabled.
     * |[16]    |CH01TOFF1 |Setting This Bit Will Force PWM0 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: only for PWM0,PWM1,PWM2,PWM3.
     * |[17]    |CH11TOFF1 |Setting This Bit Will Force PWM1 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[18]    |CH21TOFF1 |Setting This Bit Will Force PWM2 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[19]    |CH31TOFF1 |Setting This Bit Will Force PWM3 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[21:20] |CMP1SEL   |CMP1SEL
     * |        |          |Select the positive input source of ACMP1.
     * |        |          |00 = Select P3.1 as the input of ACMP1.
     * |        |          |01 = Select P3.2 as the input of ACMP1.
     * |        |          |10 = Select P3.3 as the input of ACMP1.
     * |        |          |11 = Select P3.4 as the input of ACMP1.
     * |[22]    |T1        |Timer1 Trigger PWM Function Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When this bit is set, timer1 time-out event will update PHCHG with PHCHG_NXT register.
     * |[23]    |CE1       |ACMP1 Trigger Function Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit will be auto cleared when ACMP1 trigger PWM if ACCNT1 is set.
     * |[24]    |CH01TOFF0 |Setting This Bit Will Force PWM0 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[25]    |CH11TOFF0 |Setting This Bit Will Force PWM1 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[26]    |CH21TOFF0 |Setting This Bit Will Force PWM2 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[27]    |CH31TOFF0 |Setting This Bit Will Force PWM3 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[29:28] |CMP0SEL   |CMP0SEL
     * |        |          |Select the positive input source of ACMP0.
     * |        |          |00 = Select P1.5 as the input of ACMP0.
     * |        |          |01 = Select P1.0 as the input of ACMP0.
     * |        |          |10 = Select P1.2 as the input of ACMP0.
     * |        |          |11 = Select P1.3 as the input of ACMP0.
     * |[30]    |T0        |Timer0 Trigger PWM Function Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When this bit is set, timer0 time-out event will update PHCHG with PHCHG_NXT register.
     * |[31]    |CE0       |ACMP0 Trigger Function Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit will be auto cleared when ACMP0 trigger PWM if ACCNT0 is set.
     * @var PWM_T::PHCHGNXT
     * Offset: 0x7C  Next Phase Change Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |D0        |D0: When PWM0 Is Zero, Channel 0's Output Waveform Is D0
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[1]     |D1        |D1: When PWM1 Is Zero, Channel 1's Output Waveform Is D1
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[2]     |D2        |D2: When PWM2 Is Zero, Channel 2's Output Waveform Is D2
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[3]     |D3        |D3: When PWM3 Is Zero, Channel 3's Output Waveform Is D3
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[4]     |D4        |D4: When PWM4 Is Zero, Channel 4's Output Waveform Is D4
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[5]     |D5        |D5: When PWM5 Is Zero, Channel 5's Output Waveform Is D5
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[6]     |D6        |D6: When MASK6 Is 1, Channel 6's Output Waveform Is D6
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[7]     |D7        |D7: When MASK7 Is 1, Channel 7's Output Waveform Is D7
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[8]     |PWM0      |PWM Channel 0 Output Enable Control
     * |        |          |0 = Output D0 specified in bit 0 of PHCHG register.
     * |        |          |1 = Output the original channel 0 waveform.
     * |[9]     |PWM1      |PWM Channel 1 Output Enable Control
     * |        |          |0 = Output D1 specified in bit 1 of PHCHG register.
     * |        |          |1 = Output the original channel 1 waveform.
     * |[10]    |PWM2      |PWM Channel 2 Output Enable Control
     * |        |          |0 = Output D2 specified in bit 2 of PHCHG register.
     * |        |          |1 = Output the original channel 2 waveform.
     * |[11]    |PWM3      |PWM Channel 3 Output Enable Control
     * |        |          |0 = Output D3 specified in bit 3 of PHCHG register.
     * |        |          |1 = Output the original channel 3 waveform.
     * |[12]    |PWM4      |PWM Channel 4 Output Enable Control
     * |        |          |0 = Output D4 specified in bit 4 of PHCHG register.
     * |        |          |1 = Output the original channel 4 waveform.
     * |[13]    |PWM5      |PWM Channel 5 Output Enable Control
     * |        |          |0 = Output D5 specified in bit 5 of PHCHG register.
     * |        |          |1 = Output the original channel 5 waveform.
     * |[14]    |ACCNT0    |Hardware Auto Clear CE0 When ACMP0 Trigger It
     * |        |          |0 = Enabled.
     * |        |          |1 = Disabled.
     * |[15]    |ACCNT1    |Hardware Auto Clear CE1 When ACMP1 Trigger It
     * |        |          |0 = Enabled.
     * |        |          |1 = Disabled.
     * |[16]    |CH01TOFF1 |Setting This Bit Will Force PWM0 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: only for PWM0,PWM1,PWM2,PWM3.
     * |[17]    |CH11TOFF1 |Setting This Bit Will Force PWM1 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[18]    |CH21TOFF1 |Setting This Bit Will Force PWM2 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[19]    |CH31TOFF1 |Setting This Bit Will Force PWM3 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[21:20] |CMP1SEL   |CMP1SEL
     * |        |          |Select the positive input source of ACMP1.
     * |        |          |00 = Select P3.1 as the input of ACMP1.
     * |        |          |01 = Select P3.2 as the input of ACMP1.
     * |        |          |10 = Select P3.3 as the input of ACMP1.
     * |        |          |11 = Select P3.4 as the input of ACMP1.
     * |[22]    |T1        |Timer1 Trigger PWM Function Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When this bit is set, timer1 time-out event will update PHCHG with PHCHG_NXT register.
     * |[23]    |CE1       |ACMP1 Trigger Function Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit will be auto cleared when ACMP1 trigger PWM if ACCNT1 is set.
     * |[24]    |CH01TOFF0 |Setting This Bit Will Force PWM0 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[25]    |CH11TOFF0 |Setting This Bit Will Force PWM1 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[26]    |CH21TOFF0 |Setting This Bit Will Force PWM2 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[27]    |CH31TOFF0 |Setting This Bit Will Force PWM3 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for PWM0, PWM1, PWM2, PWM3.
     * |[29:28] |CMP0SEL   |CMP0SEL
     * |        |          |Select the positive input source of ACMP0.
     * |        |          |00 = Select P1.5 as the input of ACMP0.
     * |        |          |01 = Select P1.0 as the input of ACMP0.
     * |        |          |10 = Select P1.2 as the input of ACMP0.
     * |        |          |11 = Select P1.3 as the input of ACMP0.
     * |[30]    |T0        |Timer0 Trigger PWM Function Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When this bit is set, timer0 time-out event will update PHCHG with PHCHG_NXT register.
     * |[31]    |CE0       |ACMP0 Trigger Function Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit will be auto cleared when ACMP0 trigger PWM if ACCNT0 is set.
     * @var PWM_T::PHCHGMASK
     * Offset: 0x80  Phase Change MASK Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6]     |MASK6     |MASK For D6
     * |        |          |0 = Original GPIO P0.1.
     * |        |          |1 = D6.
     * |[7]     |MASK7     |MASK For D7
     * |        |          |0 = Original GPIO P0.0.
     * |        |          |1 = D7.
     * |[8]     |CMPMASK0  |MASK For ACMP0
     * |        |          |0 = The input of ACMP is controlled by CMP0CR.
     * |        |          |1 = The input of ACMP is controlled by CMP0SEL of PHCHG register.
     * |        |          |Note: Register CMP0CR is describe in Comparator Controller chapter
     * |[9]     |CMPMASK1  |MASK For ACMP1
     * |        |          |0 = The input of ACMP is controlled by CMP1CR.
     * |        |          |1 = The input of ACMP is controlled by CMP1SEL of PHCHG register.
     * |        |          |Note: Register CMP1CR is describe in Comparator Controller chapter
     * @var PWM_T::INTACCUCTL
     * Offset: 0x84  Period Interrupt Accumulation Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTACCUEN0|Interrupt Accumulation Function Enable Control
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[7:4]   |PERIODCNT |Interrupt Accumulation Bits
     * |        |          |When INTACCUEN0 is set, PERIODCNT will decrease when every PWMPIF0 flag is set and when PERIODCNT reach to zero, the PWM0 interrupt will occurred and PERIODCNT will reload itself.
     */

    __IO uint32_t PPR;           /* Offset: 0x00  PWM Pre-scale Register                                             */
    __IO uint32_t CSR;           /* Offset: 0x04  PWM Clock Select Register                                          */
    __IO uint32_t PCR;           /* Offset: 0x08  PWM Control Register                                               */
    __IO uint32_t CNR[6];        /* Offset: 0x000C ~ 0x0020 PWM Counter Register 0 ~ 5                               */
    __IO uint32_t CMR[6];        /* Offset: 0x0024 ~ 0x0038 PWM Comparator Register 0 ~ 5                            */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0[6];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t PIER;          /* Offset: 0x54  PWM Interrupt Enable Control Register                              */
    __IO uint32_t PIIR;          /* Offset: 0x58  PWM Interrupt Indication Register                                  */
    __IO uint32_t POE;           /* Offset: 0x5C  PWM Output Enable for Channel 0~5                                  */
    __IO uint32_t PFBCON;        /* Offset: 0x60  PWM Fault Brake Control Register                                   */
    __IO uint32_t PDZIR;         /* Offset: 0x64  PWM Dead-zone Interval Register                                    */
    __IO uint32_t TRGCON0;       /* Offset: 0x68  PWM Trigger Control Register 0                                     */
    __IO uint32_t TRGCON1;       /* Offset: 0x6C  PWM Trigger Control Register 1                                     */
    __IO uint32_t TRGSTS0;       /* Offset: 0x70  PWM Trigger Status Register 0                                      */
    __IO uint32_t TRGSTS1;       /* Offset: 0x74  PWM Trigger Status Register 1                                      */
    __IO uint32_t PHCHG;         /* Offset: 0x78  Phase Change Register                                              */
    __IO uint32_t PHCHGNXT;      /* Offset: 0x7C  Next Phase Change Register                                         */
    __IO uint32_t PHCHGMASK;     /* Offset: 0x80  Phase Change MASK Register                                         */
    __IO uint32_t INTACCUCTL;    /* Offset: 0x84  Period Interrupt Accumulation Control Register                     */

} PWM_T;



/**
    @addtogroup PWM_CONST PWM Bit Field Definition
    Constant Definitions for PWM Controller
@{ */

#define PWM_PPR_CP01_Pos                 (0)                                               /*!< PWM_T::PPR: CP01 Position                 */
#define PWM_PPR_CP01_Msk                 (0xfful << PWM_PPR_CP01_Pos)                      /*!< PWM_T::PPR: CP01 Mask                     */

#define PWM_PPR_CP23_Pos                 (8)                                               /*!< PWM_T::PPR: CP23 Position                 */
#define PWM_PPR_CP23_Msk                 (0xfful << PWM_PPR_CP23_Pos)                      /*!< PWM_T::PPR: CP23 Mask                     */

#define PWM_PPR_CP45_Pos                 (16)                                              /*!< PWM_T::PPR: CP45 Position                 */
#define PWM_PPR_CP45_Msk                 (0xfful << PWM_PPR_CP45_Pos)                      /*!< PWM_T::PPR: CP45 Mask                     */

#define PWM_CSR_CSR0_Pos                 (0)                                               /*!< PWM_T::CSR: CSR0 Position                 */
#define PWM_CSR_CSR0_Msk                 (0x7ul << PWM_CSR_CSR0_Pos)                       /*!< PWM_T::CSR: CSR0 Mask                     */

#define PWM_CSR_CSR1_Pos                 (4)                                               /*!< PWM_T::CSR: CSR1 Position                 */
#define PWM_CSR_CSR1_Msk                 (0x7ul << PWM_CSR_CSR1_Pos)                       /*!< PWM_T::CSR: CSR1 Mask                     */

#define PWM_CSR_CSR2_Pos                 (8)                                               /*!< PWM_T::CSR: CSR2 Position                 */
#define PWM_CSR_CSR2_Msk                 (0x7ul << PWM_CSR_CSR2_Pos)                       /*!< PWM_T::CSR: CSR2 Mask                     */

#define PWM_CSR_CSR3_Pos                 (12)                                              /*!< PWM_T::CSR: CSR3 Position                 */
#define PWM_CSR_CSR3_Msk                 (0x7ul << PWM_CSR_CSR3_Pos)                       /*!< PWM_T::CSR: CSR3 Mask                     */

#define PWM_CSR_CSR4_Pos                 (16)                                              /*!< PWM_T::CSR: CSR4 Position                 */
#define PWM_CSR_CSR4_Msk                 (0x7ul << PWM_CSR_CSR4_Pos)                       /*!< PWM_T::CSR: CSR4 Mask                     */

#define PWM_CSR_CSR5_Pos                 (20)                                              /*!< PWM_T::CSR: CSR5 Position                 */
#define PWM_CSR_CSR5_Msk                 (0x7ul << PWM_CSR_CSR5_Pos)                       /*!< PWM_T::CSR: CSR5 Mask                     */

#define PWM_PCR_CH0EN_Pos                (0)                                               /*!< PWM_T::PCR: CH0EN Position                */
#define PWM_PCR_CH0EN_Msk                (0x1ul << PWM_PCR_CH0EN_Pos)                      /*!< PWM_T::PCR: CH0EN Mask                    */

#define PWM_PCR_DB_MOD_Pos               (1)                                               /*!< PWM_T::PCR: DB_MODE Position              */
#define PWM_PCR_DB_MOD_Msk               (0x1ul << PWM_PCR_DB_MOD_Pos)                     /*!< PWM_T::PCR: DB_MODE Mask                  */

#define PWM_PCR_CH0INV_Pos               (2)                                               /*!< PWM_T::PCR: CH0INV Position               */
#define PWM_PCR_CH0INV_Msk               (0x1ul << PWM_PCR_CH0INV_Pos)                     /*!< PWM_T::PCR: CH0INV Mask                   */

#define PWM_PCR_CH0MOD_Pos               (3)                                               /*!< PWM_T::PCR: CH0MOD Position               */
#define PWM_PCR_CH0MOD_Msk               (0x1ul << PWM_PCR_CH0MOD_Pos)                     /*!< PWM_T::PCR: CH0MOD Mask                   */

#define PWM_PCR_CH1EN_Pos                (4)                                               /*!< PWM_T::PCR: CH1EN Position                */
#define PWM_PCR_CH1EN_Msk                (0x1ul << PWM_PCR_CH1EN_Pos)                      /*!< PWM_T::PCR: CH1EN Mask                    */

#define PWM_PCR_CH1INV_Pos               (6)                                               /*!< PWM_T::PCR: CH1INV Position               */
#define PWM_PCR_CH1INV_Msk               (0x1ul << PWM_PCR_CH1INV_Pos)                     /*!< PWM_T::PCR: CH1INV Mask                   */

#define PWM_PCR_CH1MOD_Pos               (7)                                               /*!< PWM_T::PCR: CH1MOD Position               */
#define PWM_PCR_CH1MOD_Msk               (0x1ul << PWM_PCR_CH1MOD_Pos)                     /*!< PWM_T::PCR: CH1MOD Mask                   */

#define PWM_PCR_CH2EN_Pos                (8)                                               /*!< PWM_T::PCR: CH2EN Position                */
#define PWM_PCR_CH2EN_Msk                (0x1ul << PWM_PCR_CH2EN_Pos)                      /*!< PWM_T::PCR: CH2EN Mask                    */

#define PWM_PCR_CH2INV_Pos               (10)                                              /*!< PWM_T::PCR: CH2INV Position               */
#define PWM_PCR_CH2INV_Msk               (0x1ul << PWM_PCR_CH2INV_Pos)                     /*!< PWM_T::PCR: CH2INV Mask                   */

#define PWM_PCR_CH2MOD_Pos               (11)                                              /*!< PWM_T::PCR: CH2MOD Position               */
#define PWM_PCR_CH2MOD_Msk               (0x1ul << PWM_PCR_CH2MOD_Pos)                     /*!< PWM_T::PCR: CH2MOD Mask                   */

#define PWM_PCR_CH3EN_Pos                (12)                                              /*!< PWM_T::PCR: CH3EN Position                */
#define PWM_PCR_CH3EN_Msk                (0x1ul << PWM_PCR_CH3EN_Pos)                      /*!< PWM_T::PCR: CH3EN Mask                    */

#define PWM_PCR_CH3INV_Pos               (14)                                              /*!< PWM_T::PCR: CH3INV Position               */
#define PWM_PCR_CH3INV_Msk               (0x1ul << PWM_PCR_CH3INV_Pos)                     /*!< PWM_T::PCR: CH3INV Mask                   */

#define PWM_PCR_CH3MOD_Pos               (15)                                              /*!< PWM_T::PCR: CH3MOD Position               */
#define PWM_PCR_CH3MOD_Msk               (0x1ul << PWM_PCR_CH3MOD_Pos)                     /*!< PWM_T::PCR: CH3MOD Mask                   */

#define PWM_PCR_CH4EN_Pos                (16)                                              /*!< PWM_T::PCR: CH4EN Position                */
#define PWM_PCR_CH4EN_Msk                (0x1ul << PWM_PCR_CH4EN_Pos)                      /*!< PWM_T::PCR: CH4EN Mask                    */

#define PWM_PCR_CH4INV_Pos               (18)                                              /*!< PWM_T::PCR: CH4INV Position               */
#define PWM_PCR_CH4INV_Msk               (0x1ul << PWM_PCR_CH4INV_Pos)                     /*!< PWM_T::PCR: CH4INV Mask                   */

#define PWM_PCR_CH4MOD_Pos               (19)                                              /*!< PWM_T::PCR: CH4MOD Position               */
#define PWM_PCR_CH4MOD_Msk               (0x1ul << PWM_PCR_CH4MOD_Pos)                     /*!< PWM_T::PCR: CH4MOD Mask                   */

#define PWM_PCR_CH5EN_Pos                (20)                                              /*!< PWM_T::PCR: CH5EN Position                */
#define PWM_PCR_CH5EN_Msk                (0x1ul << PWM_PCR_CH5EN_Pos)                      /*!< PWM_T::PCR: CH5EN Mask                    */

#define PWM_PCR_CH5INV_Pos               (22)                                              /*!< PWM_T::PCR: CH5INV Position               */
#define PWM_PCR_CH5INV_Msk               (0x1ul << PWM_PCR_CH5INV_Pos)                     /*!< PWM_T::PCR: CH5INV Mask                   */

#define PWM_PCR_CH5MOD_Pos               (23)                                              /*!< PWM_T::PCR: CH5MOD Position               */
#define PWM_PCR_CH5MOD_Msk               (0x1ul << PWM_PCR_CH5MOD_Pos)                     /*!< PWM_T::PCR: CH5MOD Mask                   */

#define PWM_PCR_DZEN01_Pos               (24)                                              /*!< PWM_T::PCR: DZEN01 Position               */
#define PWM_PCR_DZEN01_Msk               (0x1ul << PWM_PCR_DZEN01_Pos)                     /*!< PWM_T::PCR: DZEN01 Mask                   */

#define PWM_PCR_DZEN23_Pos               (25)                                              /*!< PWM_T::PCR: DZEN23 Position               */
#define PWM_PCR_DZEN23_Msk               (0x1ul << PWM_PCR_DZEN23_Pos)                     /*!< PWM_T::PCR: DZEN23 Mask                   */

#define PWM_PCR_DZEN45_Pos               (26)                                              /*!< PWM_T::PCR: DZEN45 Position               */
#define PWM_PCR_DZEN45_Msk               (0x1ul << PWM_PCR_DZEN45_Pos)                     /*!< PWM_T::PCR: DZEN45 Mask                   */

#define PWM_PCR_CLRPWM_Pos               (27)                                              /*!< PWM_T::PCR: CLRPWM Position               */
#define PWM_PCR_CLRPWM_Msk               (0x1ul << PWM_PCR_CLRPWM_Pos)                     /*!< PWM_T::PCR: CLRPWM Mask                   */

#define PWM_PCR_PWMMOD_Pos               (28)                                              /*!< PWM_T::PCR: PWMMOD Position               */
#define PWM_PCR_PWMMOD_Msk               (0x3ul << PWM_PCR_PWMMOD_Pos)                     /*!< PWM_T::PCR: PWMMOD Mask                   */

#define PWM_PCR_GRP_Pos                  (30)                                              /*!< PWM_T::PCR: GRP Position                  */
#define PWM_PCR_GRP_Msk                  (0x1ul << PWM_PCR_GRP_Pos)                        /*!< PWM_T::PCR: GRP Mask                      */

#define PWM_PCR_PWMTYPE_Pos              (31)                                              /*!< PWM_T::PCR: PWMTYPE Position              */
#define PWM_PCR_PWMTYPE_Msk              (0x1ul << PWM_PCR_PWMTYPE_Pos)                    /*!< PWM_T::PCR: PWMTYPE Mask                  */

#define PWM_CNR_CNR_Pos                  0                                                 /*!< PWM CNR::CNR Position                     */
#define PWM_CNR_CNR_Msk                  (0xfffful << PWM_CNR_CNR_Pos)                     /*!< PWM CNR::CNR Mask                         */

#define PWM_CMR_CMR_Pos                  0                                                 /*!< PWM CMR::CMR Position                     */
#define PWM_CMR_CMR_Msk                  (0xfffful << PWM_CMR_CMR_Pos)                     /*!< PWM CMR::CMR Mask                         */

#define PWM_PIER_PWMPIE0_Pos             (0)                                               /*!< PWM_T::PIER: PWMPIE0 Position             */
#define PWM_PIER_PWMPIE0_Msk             (0x1ul << PWM_PIER_PWMPIE0_Pos)                   /*!< PWM_T::PIER: PWMPIE0 Mask                 */

#define PWM_PIER_PWMPIE1_Pos             (1)                                               /*!< PWM_T::PIER: PWMPIE1 Position             */
#define PWM_PIER_PWMPIE1_Msk             (0x1ul << PWM_PIER_PWMPIE1_Pos)                   /*!< PWM_T::PIER: PWMPIE1 Mask                 */

#define PWM_PIER_PWMPIE2_Pos             (2)                                               /*!< PWM_T::PIER: PWMPIE2 Position             */
#define PWM_PIER_PWMPIE2_Msk             (0x1ul << PWM_PIER_PWMPIE2_Pos)                   /*!< PWM_T::PIER: PWMPIE2 Mask                 */

#define PWM_PIER_PWMPIE3_Pos             (3)                                               /*!< PWM_T::PIER: PWMPIE3 Position             */
#define PWM_PIER_PWMPIE3_Msk             (0x1ul << PWM_PIER_PWMPIE3_Pos)                   /*!< PWM_T::PIER: PWMPIE3 Mask                 */

#define PWM_PIER_PWMPIE4_Pos             (4)                                               /*!< PWM_T::PIER: PWMPIE4 Position             */
#define PWM_PIER_PWMPIE4_Msk             (0x1ul << PWM_PIER_PWMPIE4_Pos)                   /*!< PWM_T::PIER: PWMPIE4 Mask                 */

#define PWM_PIER_PWMPIE5_Pos             (5)                                               /*!< PWM_T::PIER: PWMPIE5 Position             */
#define PWM_PIER_PWMPIE5_Msk             (0x1ul << PWM_PIER_PWMPIE5_Pos)                   /*!< PWM_T::PIER: PWMPIE5 Mask                 */

#define PWM_PIER_PWMDIE0_Pos             (8)                                               /*!< PWM_T::PIER: PWMDIE0 Position             */
#define PWM_PIER_PWMDIE0_Msk             (0x1ul << PWM_PIER_PWMDIE0_Pos)                   /*!< PWM_T::PIER: PWMDIE0 Mask                 */

#define PWM_PIER_PWMDIE1_Pos             (9)                                               /*!< PWM_T::PIER: PWMDIE1 Position             */
#define PWM_PIER_PWMDIE1_Msk             (0x1ul << PWM_PIER_PWMDIE1_Pos)                   /*!< PWM_T::PIER: PWMDIE1 Mask                 */

#define PWM_PIER_PWMDIE2_Pos             (10)                                              /*!< PWM_T::PIER: PWMDIE2 Position             */
#define PWM_PIER_PWMDIE2_Msk             (0x1ul << PWM_PIER_PWMDIE2_Pos)                   /*!< PWM_T::PIER: PWMDIE2 Mask                 */

#define PWM_PIER_PWMDIE3_Pos             (11)                                              /*!< PWM_T::PIER: PWMDIE3 Position             */
#define PWM_PIER_PWMDIE3_Msk             (0x1ul << PWM_PIER_PWMDIE3_Pos)                   /*!< PWM_T::PIER: PWMDIE3 Mask                 */

#define PWM_PIER_PWMDIE4_Pos             (12)                                              /*!< PWM_T::PIER: PWMDIE4 Position             */
#define PWM_PIER_PWMDIE4_Msk             (0x1ul << PWM_PIER_PWMDIE4_Pos)                   /*!< PWM_T::PIER: PWMDIE4 Mask                 */

#define PWM_PIER_PWMDIE5_Pos             (13)                                              /*!< PWM_T::PIER: PWMDIE5 Position             */
#define PWM_PIER_PWMDIE5_Msk             (0x1ul << PWM_PIER_PWMDIE5_Pos)                   /*!< PWM_T::PIER: PWMDIE5 Mask                 */

#define PWM_PIER_BRKIE_Pos               (16)                                              /*!< PWM_T::PIER: BRKIE Position               */
#define PWM_PIER_BRKIE_Msk               (0x1ul << PWM_PIER_BRKIE_Pos)                     /*!< PWM_T::PIER: BRKIE Mask                   */

#define PWM_PIER_INT_TYPE_Pos            (17)                                              /*!< PWM_T::PIER: INT_TYPE Position            */
#define PWM_PIER_INT_TYPE_Msk            (0x1ul << PWM_PIER_INT_TYPE_Pos)                  /*!< PWM_T::PIER: INT_TYPE Mask                */

#define PWM_PIIR_PWMPIF0_Pos             (0)                                               /*!< PWM_T::PIIR: PWMPIF0 Position             */
#define PWM_PIIR_PWMPIF0_Msk             (0x1ul << PWM_PIIR_PWMPIF0_Pos)                   /*!< PWM_T::PIIR: PWMPIF0 Mask                 */

#define PWM_PIIR_PWMPIF1_Pos             (1)                                               /*!< PWM_T::PIIR: PWMPIF1 Position             */
#define PWM_PIIR_PWMPIF1_Msk             (0x1ul << PWM_PIIR_PWMPIF1_Pos)                   /*!< PWM_T::PIIR: PWMPIF1 Mask                 */

#define PWM_PIIR_PWMPIF2_Pos             (2)                                               /*!< PWM_T::PIIR: PWMPIF2 Position             */
#define PWM_PIIR_PWMPIF2_Msk             (0x1ul << PWM_PIIR_PWMPIF2_Pos)                   /*!< PWM_T::PIIR: PWMPIF2 Mask                 */

#define PWM_PIIR_PWMPIF3_Pos             (3)                                               /*!< PWM_T::PIIR: PWMPIF3 Position             */
#define PWM_PIIR_PWMPIF3_Msk             (0x1ul << PWM_PIIR_PWMPIF3_Pos)                   /*!< PWM_T::PIIR: PWMPIF3 Mask                 */

#define PWM_PIIR_PWMPIF4_Pos             (4)                                               /*!< PWM_T::PIIR: PWMPIF4 Position             */
#define PWM_PIIR_PWMPIF4_Msk             (0x1ul << PWM_PIIR_PWMPIF4_Pos)                   /*!< PWM_T::PIIR: PWMPIF4 Mask                 */

#define PWM_PIIR_PWMPIF5_Pos             (5)                                               /*!< PWM_T::PIIR: PWMPIF5 Position             */
#define PWM_PIIR_PWMPIF5_Msk             (0x1ul << PWM_PIIR_PWMPIF5_Pos)                   /*!< PWM_T::PIIR: PWMPIF5 Mask                 */

#define PWM_PIIR_PWMDIF0_Pos             (8)                                               /*!< PWM_T::PIIR: PWMDIF0 Position             */
#define PWM_PIIR_PWMDIF0_Msk             (0x1ul << PWM_PIIR_PWMDIF0_Pos)                   /*!< PWM_T::PIIR: PWMDIF0 Mask                 */

#define PWM_PIIR_PWMDIF1_Pos             (9)                                               /*!< PWM_T::PIIR: PWMDIF1 Position             */
#define PWM_PIIR_PWMDIF1_Msk             (0x1ul << PWM_PIIR_PWMDIF1_Pos)                   /*!< PWM_T::PIIR: PWMDIF1 Mask                 */

#define PWM_PIIR_PWMDIF2_Pos             (10)                                              /*!< PWM_T::PIIR: PWMDIF2 Position             */
#define PWM_PIIR_PWMDIF2_Msk             (0x1ul << PWM_PIIR_PWMDIF2_Pos)                   /*!< PWM_T::PIIR: PWMDIF2 Mask                 */

#define PWM_PIIR_PWMDIF3_Pos             (11)                                              /*!< PWM_T::PIIR: PWMDIF3 Position             */
#define PWM_PIIR_PWMDIF3_Msk             (0x1ul << PWM_PIIR_PWMDIF3_Pos)                   /*!< PWM_T::PIIR: PWMDIF3 Mask                 */

#define PWM_PIIR_PWMDIF4_Pos             (12)                                              /*!< PWM_T::PIIR: PWMDIF4 Position             */
#define PWM_PIIR_PWMDIF4_Msk             (0x1ul << PWM_PIIR_PWMDIF4_Pos)                   /*!< PWM_T::PIIR: PWMDIF4 Mask                 */

#define PWM_PIIR_PWMDIF5_Pos             (13)                                              /*!< PWM_T::PIIR: PWMDIF5 Position             */
#define PWM_PIIR_PWMDIF5_Msk             (0x1ul << PWM_PIIR_PWMDIF5_Pos)                   /*!< PWM_T::PIIR: PWMDIF5 Mask                 */

#define PWM_PIIR_BKF0_Pos                (16)                                              /*!< PWM_T::PIIR: BKF0 Position                */
#define PWM_PIIR_BKF0_Msk                (0x1ul << PWM_PIIR_BKF0_Pos)                      /*!< PWM_T::PIIR: BKF0 Mask                    */

#define PWM_PIIR_BKF1_Pos                (17)                                              /*!< PWM_T::PIIR: BKF1 Position                */
#define PWM_PIIR_BKF1_Msk                (0x1ul << PWM_PIIR_BKF1_Pos)                      /*!< PWM_T::PIIR: BKF1 Mask                    */

#define PWM_POE_PWM0_Pos                 (0)                                               /*!< PWM_T::POE: PWM0 Position                 */
#define PWM_POE_PWM0_Msk                 (0x1ul << PWM_POE_PWM0_Pos)                       /*!< PWM_T::POE: PWM0 Mask                     */

#define PWM_POE_PWM1_Pos                 (1)                                               /*!< PWM_T::POE: PWM1 Position                 */
#define PWM_POE_PWM1_Msk                 (0x1ul << PWM_POE_PWM1_Pos)                       /*!< PWM_T::POE: PWM1 Mask                     */

#define PWM_POE_PWM2_Pos                 (2)                                               /*!< PWM_T::POE: PWM2 Position                 */
#define PWM_POE_PWM2_Msk                 (0x1ul << PWM_POE_PWM2_Pos)                       /*!< PWM_T::POE: PWM2 Mask                     */

#define PWM_POE_PWM3_Pos                 (3)                                               /*!< PWM_T::POE: PWM3 Position                 */
#define PWM_POE_PWM3_Msk                 (0x1ul << PWM_POE_PWM3_Pos)                       /*!< PWM_T::POE: PWM3 Mask                     */

#define PWM_POE_PWM4_Pos                 (4)                                               /*!< PWM_T::POE: PWM4 Position                 */
#define PWM_POE_PWM4_Msk                 (0x1ul << PWM_POE_PWM4_Pos)                       /*!< PWM_T::POE: PWM4 Mask                     */

#define PWM_POE_PWM5_Pos                 (5)                                               /*!< PWM_T::POE: PWM5 Position                 */
#define PWM_POE_PWM5_Msk                 (0x1ul << PWM_POE_PWM5_Pos)                       /*!< PWM_T::POE: PWM5 Mask                     */

#define PWM_PFBCON_BKEN0_Pos             (0)                                               /*!< PWM_T::PFBCON: BKEN0 Position             */
#define PWM_PFBCON_BKEN0_Msk             (0x1ul << PWM_PFBCON_BKEN0_Pos)                   /*!< PWM_T::PFBCON: BKEN0 Mask                 */

#define PWM_PFBCON_BKEN1_Pos             (1)                                               /*!< PWM_T::PFBCON: BKEN1 Position             */
#define PWM_PFBCON_BKEN1_Msk             (0x1ul << PWM_PFBCON_BKEN1_Pos)                   /*!< PWM_T::PFBCON: BKEN1 Mask                 */

#define PWM_PFBCON_CPO0BKEN_Pos          (2)                                               /*!< PWM_T::PFBCON: CPO0BKEN Position          */
#define PWM_PFBCON_CPO0BKEN_Msk          (0x1ul << PWM_PFBCON_CPO0BKEN_Pos)                /*!< PWM_T::PFBCON: CPO0BKEN Mask              */

#define PWM_PFBCON_CPO1BKEN_Pos          (3)                                               /*!< PWM_T::PFBCON: CPO1BKEN Position          */
#define PWM_PFBCON_CPO1BKEN_Msk          (0x1ul << PWM_PFBCON_CPO1BKEN_Pos)                /*!< PWM_T::PFBCON: CPO1BKEN Mask              */

#define PWM_PFBCON_BKF_Pos               (7)                                               /*!< PWM_T::PFBCON: BKF Position               */
#define PWM_PFBCON_BKF_Msk               (0x1ul << PWM_PFBCON_BKF_Pos)                     /*!< PWM_T::PFBCON: BKF Mask                   */

#define PWM_PFBCON_PWMBKO0_Pos           (24)                                              /*!< PWM_T::PFBCON: PWMBKO0 Position           */
#define PWM_PFBCON_PWMBKO0_Msk           (0x1ul << PWM_PFBCON_PWMBKO0_Pos)                 /*!< PWM_T::PFBCON: PWMBKO0 Mask               */

#define PWM_PFBCON_PWMBKO1_Pos           (25)                                              /*!< PWM_T::PFBCON: PWMBKO1 Position           */
#define PWM_PFBCON_PWMBKO1_Msk           (0x1ul << PWM_PFBCON_PWMBKO1_Pos)                 /*!< PWM_T::PFBCON: PWMBKO1 Mask               */

#define PWM_PFBCON_PWMBKO2_Pos           (26)                                              /*!< PWM_T::PFBCON: PWMBKO2 Position           */
#define PWM_PFBCON_PWMBKO2_Msk           (0x1ul << PWM_PFBCON_PWMBKO2_Pos)                 /*!< PWM_T::PFBCON: PWMBKO2 Mask               */

#define PWM_PFBCON_PWMBKO3_Pos           (27)                                              /*!< PWM_T::PFBCON: PWMBKO3 Position           */
#define PWM_PFBCON_PWMBKO3_Msk           (0x1ul << PWM_PFBCON_PWMBKO3_Pos)                 /*!< PWM_T::PFBCON: PWMBKO3 Mask               */

#define PWM_PFBCON_PWMBKO4_Pos           (28)                                              /*!< PWM_T::PFBCON: PWMBKO4 Position           */
#define PWM_PFBCON_PWMBKO4_Msk           (0x1ul << PWM_PFBCON_PWMBKO4_Pos)                 /*!< PWM_T::PFBCON: PWMBKO4 Mask               */

#define PWM_PFBCON_PWMBKO5_Pos           (29)                                              /*!< PWM_T::PFBCON: PWMBKO5 Position           */
#define PWM_PFBCON_PWMBKO5_Msk           (0x1ul << PWM_PFBCON_PWMBKO5_Pos)                 /*!< PWM_T::PFBCON: PWMBKO5 Mask               */

#define PWM_PFBCON_D6BKO6_Pos            (30)                                              /*!< PWM_T::PFBCON: D6BKO6 Position            */
#define PWM_PFBCON_D6BKO6_Msk            (0x1ul << PWM_PFBCON_D6BKO6_Pos)                  /*!< PWM_T::PFBCON: D6BKO6 Mask                */

#define PWM_PFBCON_D7BKO7_Pos            (31)                                              /*!< PWM_T::PFBCON: D7BKO7 Position            */
#define PWM_PFBCON_D7BKO7_Msk            (0x1ul << PWM_PFBCON_D7BKO7_Pos)                  /*!< PWM_T::PFBCON: D7BKO7 Mask                */

#define PWM_DZIR_DZI01_Pos               (0)                                               /*!< PWM_T::PDZIR: DZI01 Position              */
#define PWM_DZIR_DZI01_Msk               (0xfful << PWM_DZIR_DZI01_Pos)                    /*!< PWM_T::PDZIR: DZI01 Mask                  */

#define PWM_DZIR_DZI23_Pos               (8)                                               /*!< PWM_T::PDZIR: DZI23 Position              */
#define PWM_DZIR_DZI23_Msk               (0xfful << PWM_DZIR_DZI23_Pos)                    /*!< PWM_T::PDZIR: DZI23 Mask                  */

#define PWM_DZIR_DZI45_Pos               (16)                                              /*!< PWM_T::PDZIR: DZI45 Position              */
#define PWM_DZIR_DZI45_Msk               (0xfful << PWM_DZIR_DZI45_Pos)                    /*!< PWM_T::PDZIR: DZI45 Mask                  */

#define PWM_TRGCON0_CM0TRGREN_Pos        (0)                                               /*!< PWM_T::TRGCON0: CM0TRGREN Position        */
#define PWM_TRGCON0_CM0TRGREN_Msk        (0x1ul << PWM_TRGCON0_CM0TRGREN_Pos)              /*!< PWM_T::TRGCON0: CM0TRGREN Mask            */

#define PWM_TRGCON0_CNT0TRGEN_Pos        (1)                                               /*!< PWM_T::TRGCON0: CNT0TRGEN Position        */
#define PWM_TRGCON0_CNT0TRGEN_Msk        (0x1ul << PWM_TRGCON0_CNT0TRGEN_Pos)              /*!< PWM_T::TRGCON0: CNT0TRGEN Mask            */

#define PWM_TRGCON0_CM0TRGFEN_Pos        (2)                                               /*!< PWM_T::TRGCON0: CM0TRGFEN Position        */
#define PWM_TRGCON0_CM0TRGFEN_Msk        (0x1ul << PWM_TRGCON0_CM0TRGFEN_Pos)              /*!< PWM_T::TRGCON0: CM0TRGFEN Mask            */

#define PWM_TRGCON0_P0TRGEN_Pos          (3)                                               /*!< PWM_T::TRGCON0: P0TRGEN Position          */
#define PWM_TRGCON0_P0TRGEN_Msk          (0x1ul << PWM_TRGCON0_P0TRGEN_Pos)                /*!< PWM_T::TRGCON0: P0TRGEN Mask              */

#define PWM_TRGCON0_CM1TRGREN_Pos        (8)                                               /*!< PWM_T::TRGCON0: CM1TRGREN Position        */
#define PWM_TRGCON0_CM1TRGREN_Msk        (0x1ul << PWM_TRGCON0_CM1TRGREN_Pos)              /*!< PWM_T::TRGCON0: CM1TRGREN Mask            */

#define PWM_TRGCON0_CNT1TRGEN_Pos        (9)                                               /*!< PWM_T::TRGCON0: CNT1TRGEN Position        */
#define PWM_TRGCON0_CNT1TRGEN_Msk        (0x1ul << PWM_TRGCON0_CNT1TRGEN_Pos)              /*!< PWM_T::TRGCON0: CNT1TRGEN Mask            */

#define PWM_TRGCON0_CM1TRGFEN_Pos        (10)                                              /*!< PWM_T::TRGCON0: CM1TRGFEN Position        */
#define PWM_TRGCON0_CM1TRGFEN_Msk        (0x1ul << PWM_TRGCON0_CM1TRGFEN_Pos)              /*!< PWM_T::TRGCON0: CM1TRGFEN Mask            */

#define PWM_TRGCON0_P1TRGEN_Pos          (11)                                              /*!< PWM_T::TRGCON0: P1TRGEN Position          */
#define PWM_TRGCON0_P1TRGEN_Msk          (0x1ul << PWM_TRGCON0_P1TRGEN_Pos)                /*!< PWM_T::TRGCON0: P1TRGEN Mask              */

#define PWM_TRGCON0_CM2TRGREN_Pos        (16)                                              /*!< PWM_T::TRGCON0: CM2TRGREN Position        */
#define PWM_TRGCON0_CM2TRGREN_Msk        (0x1ul << PWM_TRGCON0_CM2TRGREN_Pos)              /*!< PWM_T::TRGCON0: CM2TRGREN Mask            */

#define PWM_TRGCON0_CNT2TRGEN_Pos        (17)                                              /*!< PWM_T::TRGCON0: CNT2TRGEN Position        */
#define PWM_TRGCON0_CNT2TRGEN_Msk        (0x1ul << PWM_TRGCON0_CNT2TRGEN_Pos)              /*!< PWM_T::TRGCON0: CNT2TRGEN Mask            */

#define PWM_TRGCON0_CM2TRGFEN_Pos        (18)                                              /*!< PWM_T::TRGCON0: CM2TRGFEN Position        */
#define PWM_TRGCON0_CM2TRGFEN_Msk        (0x1ul << PWM_TRGCON0_CM2TRGFEN_Pos)              /*!< PWM_T::TRGCON0: CM2TRGFEN Mask            */

#define PWM_TRGCON0_P2TRGEN_Pos          (19)                                              /*!< PWM_T::TRGCON0: P2TRGEN Position          */
#define PWM_TRGCON0_P2TRGEN_Msk          (0x1ul << PWM_TRGCON0_P2TRGEN_Pos)                /*!< PWM_T::TRGCON0: P2TRGEN Mask              */

#define PWM_TRGCON0_CM3TRGREN_Pos        (24)                                              /*!< PWM_T::TRGCON0: CM3TRGREN Position        */
#define PWM_TRGCON0_CM3TRGREN_Msk        (0x1ul << PWM_TRGCON0_CM3TRGREN_Pos)              /*!< PWM_T::TRGCON0: CM3TRGREN Mask            */

#define PWM_TRGCON0_CNT3TRGEN_Pos        (25)                                              /*!< PWM_T::TRGCON0: CNT3TRGEN Position        */
#define PWM_TRGCON0_CNT3TRGEN_Msk        (0x1ul << PWM_TRGCON0_CNT3TRGEN_Pos)              /*!< PWM_T::TRGCON0: CNT3TRGEN Mask            */

#define PWM_TRGCON0_CM3TRGFEN_Pos        (26)                                              /*!< PWM_T::TRGCON0: CM3TRGFEN Position        */
#define PWM_TRGCON0_CM3TRGFEN_Msk        (0x1ul << PWM_TRGCON0_CM3TRGFEN_Pos)              /*!< PWM_T::TRGCON0: CM3TRGFEN Mask            */

#define PWM_TRGCON0_P3TRGEN_Pos          (27)                                              /*!< PWM_T::TRGCON0: P3TRGEN Position          */
#define PWM_TRGCON0_P3TRGEN_Msk          (0x1ul << PWM_TRGCON0_P3TRGEN_Pos)                /*!< PWM_T::TRGCON0: P3TRGEN Mask              */

#define PWM_TRGCON1_CM4TRGREN_Pos        (0)                                               /*!< PWM_T::TRGCON1: CM4TRGREN Position        */
#define PWM_TRGCON1_CM4TRGREN_Msk        (0x1ul << PWM_TRGCON1_CM4TRGREN_Pos)              /*!< PWM_T::TRGCON1: CM4TRGREN Mask            */

#define PWM_TRGCON1_CNT4TRGEN_Pos        (1)                                               /*!< PWM_T::TRGCON1: CNT4TRGEN Position        */
#define PWM_TRGCON1_CNT4TRGEN_Msk        (0x1ul << PWM_TRGCON1_CNT4TRGEN_Pos)              /*!< PWM_T::TRGCON1: CNT4TRGEN Mask            */

#define PWM_TRGCON1_CM4TRGFEN_Pos        (2)                                               /*!< PWM_T::TRGCON1: CM4TRGFEN Position        */
#define PWM_TRGCON1_CM4TRGFEN_Msk        (0x1ul << PWM_TRGCON1_CM4TRGFEN_Pos)              /*!< PWM_T::TRGCON1: CM4TRGFEN Mask            */

#define PWM_TRGCON1_P4TRGEN_Pos          (3)                                               /*!< PWM_T::TRGCON1: P4TRGEN Position          */
#define PWM_TRGCON1_P4TRGEN_Msk          (0x1ul << PWM_TRGCON1_P4TRGEN_Pos)                /*!< PWM_T::TRGCON1: P4TRGEN Mask              */

#define PWM_TRGCON1_CM5TRGREN_Pos        (8)                                               /*!< PWM_T::TRGCON1: CM5TRGREN Position        */
#define PWM_TRGCON1_CM5TRGREN_Msk        (0x1ul << PWM_TRGCON1_CM5TRGREN_Pos)              /*!< PWM_T::TRGCON1: CM5TRGREN Mask            */

#define PWM_TRGCON1_CNT5TRGEN_Pos        (9)                                               /*!< PWM_T::TRGCON1: CNT5TRGEN Position        */
#define PWM_TRGCON1_CNT5TRGEN_Msk        (0x1ul << PWM_TRGCON1_CNT5TRGEN_Pos)              /*!< PWM_T::TRGCON1: CNT5TRGEN Mask            */

#define PWM_TRGCON1_CM5TRGFEN_Pos        (10)                                              /*!< PWM_T::TRGCON1: CM5TRGFEN Position        */
#define PWM_TRGCON1_CM5TRGFEN_Msk        (0x1ul << PWM_TRGCON1_CM5TRGFEN_Pos)              /*!< PWM_T::TRGCON1: CM5TRGFEN Mask            */

#define PWM_TRGCON1_P5TRGEN_Pos          (11)                                              /*!< PWM_T::TRGCON1: P5TRGEN Position          */
#define PWM_TRGCON1_P5TRGEN_Msk          (0x1ul << PWM_TRGCON1_P5TRGEN_Pos)                /*!< PWM_T::TRGCON1: P5TRGEN Mask              */

#define PWM_TRGSTS0_CMR0FLAG_R_Pos       (0)                                               /*!< PWM_T::TRGSTS0: CMR0FLAG_R Position       */
#define PWM_TRGSTS0_CMR0FLAG_R_Msk       (0x1ul << PWM_TRGSTS0_CMR0FLAG_R_Pos)             /*!< PWM_T::TRGSTS0: CMR0FLAG_R Mask           */

#define PWM_TRGSTS0_CNT0FLAG_Pos         (1)                                               /*!< PWM_T::TRGSTS0: CNT0FLAG Position         */
#define PWM_TRGSTS0_CNT0FLAG_Msk         (0x1ul << PWM_TRGSTS0_CNT0FLAG_Pos)               /*!< PWM_T::TRGSTS0: CNT0FLAG Mask             */

#define PWM_TRGSTS0_CMR0FLAG_F_Pos       (2)                                               /*!< PWM_T::TRGSTS0: CMR0FLAG_F Position       */
#define PWM_TRGSTS0_CMR0FLAG_F_Msk       (0x1ul << PWM_TRGSTS0_CMR0FLAG_F_Pos)             /*!< PWM_T::TRGSTS0: CMR0FLAG_F Mask           */

#define PWM_TRGSTS0_PERID0FLAG_Pos       (3)                                               /*!< PWM_T::TRGSTS0: PERID0FLAG Position       */
#define PWM_TRGSTS0_PERID0FLAG_Msk       (0x1ul << PWM_TRGSTS0_PERID0FLAG_Pos)             /*!< PWM_T::TRGSTS0: PERID0FLAG Mask           */

#define PWM_TRGSTS0_CMR1FLAG_R_Pos       (8)                                               /*!< PWM_T::TRGSTS0: CMR1FLAG_R Position       */
#define PWM_TRGSTS0_CMR1FLAG_R_Msk       (0x1ul << PWM_TRGSTS0_CMR1FLAG_R_Pos)             /*!< PWM_T::TRGSTS0: CMR1FLAG_R Mask           */

#define PWM_TRGSTS0_CNT1FLAG_Pos         (9)                                               /*!< PWM_T::TRGSTS0: CNT1FLAG Position         */
#define PWM_TRGSTS0_CNT1FLAG_Msk         (0x1ul << PWM_TRGSTS0_CNT1FLAG_Pos)               /*!< PWM_T::TRGSTS0: CNT1FLAG Mask             */

#define PWM_TRGSTS0_CMR1FLAG_F_Pos       (10)                                              /*!< PWM_T::TRGSTS0: CMR1FLAG_F Position       */
#define PWM_TRGSTS0_CMR1FLAG_F_Msk       (0x1ul << PWM_TRGSTS0_CMR1FLAG_F_Pos)             /*!< PWM_T::TRGSTS0: CMR1FLAG_F Mask           */

#define PWM_TRGSTS0_PERID1FLAG_Pos       (11)                                              /*!< PWM_T::TRGSTS0: PERID1FLAG Position       */
#define PWM_TRGSTS0_PERID1FLAG_Msk       (0x1ul << PWM_TRGSTS0_PERID1FLAG_Pos)             /*!< PWM_T::TRGSTS0: PERID1FLAG Mask           */

#define PWM_TRGSTS0_CMR2FLAG_R_Pos       (16)                                              /*!< PWM_T::TRGSTS0: CMR2FLAG_R Position       */
#define PWM_TRGSTS0_CMR2FLAG_R_Msk       (0x1ul << PWM_TRGSTS0_CMR2FLAG_R_Pos)             /*!< PWM_T::TRGSTS0: CMR2FLAG_R Mask           */

#define PWM_TRGSTS0_CNT2FLAG_Pos         (17)                                              /*!< PWM_T::TRGSTS0: CNT2FLAG Position         */
#define PWM_TRGSTS0_CNT2FLAG_Msk         (0x1ul << PWM_TRGSTS0_CNT2FLAG_Pos)               /*!< PWM_T::TRGSTS0: CNT2FLAG Mask             */

#define PWM_TRGSTS0_CMR2FLAG_F_Pos       (18)                                              /*!< PWM_T::TRGSTS0: CMR2FLAG_F Position       */
#define PWM_TRGSTS0_CMR2FLAG_F_Msk       (0x1ul << PWM_TRGSTS0_CMR2FLAG_F_Pos)             /*!< PWM_T::TRGSTS0: CMR2FLAG_F Mask           */

#define PWM_TRGSTS0_PERID2FLAG_Pos       (19)                                              /*!< PWM_T::TRGSTS0: PERID2FLAG Position       */
#define PWM_TRGSTS0_PERID2FLAG_Msk       (0x1ul << PWM_TRGSTS0_PERID2FLAG_Pos)             /*!< PWM_T::TRGSTS0: PERID2FLAG Mask           */

#define PWM_TRGSTS0_CMR3FLAG_R_Pos       (24)                                              /*!< PWM_T::TRGSTS0: CMR3FLAG_R Position       */
#define PWM_TRGSTS0_CMR3FLAG_R_Msk       (0x1ul << PWM_TRGSTS0_CMR3FLAG_R_Pos)             /*!< PWM_T::TRGSTS0: CMR3FLAG_R Mask           */

#define PWM_TRGSTS0_CNT3FLAG_Pos         (25)                                              /*!< PWM_T::TRGSTS0: CNT3FLAG Position         */
#define PWM_TRGSTS0_CNT3FLAG_Msk         (0x1ul << PWM_TRGSTS0_CNT3FLAG_Pos)               /*!< PWM_T::TRGSTS0: CNT3FLAG Mask             */

#define PWM_TRGSTS0_CMR3FLAG_F_Pos       (26)                                              /*!< PWM_T::TRGSTS0: CMR3FLAG_F Position       */
#define PWM_TRGSTS0_CMR3FLAG_F_Msk       (0x1ul << PWM_TRGSTS0_CMR3FLAG_F_Pos)             /*!< PWM_T::TRGSTS0: CMR3FLAG_F Mask           */

#define PWM_TRGSTS0_PERID3FLAG_Pos       (27)                                              /*!< PWM_T::TRGSTS0: PERID3FLAG Position       */
#define PWM_TRGSTS0_PERID3FLAG_Msk       (0x1ul << PWM_TRGSTS0_PERID3FLAG_Pos)             /*!< PWM_T::TRGSTS0: PERID3FLAG Mask           */

#define PWM_TRGSTS1_CMR4FLAG_R_Pos       (0)                                               /*!< PWM_T::TRGSTS1: CMR4FLAG_R Position       */
#define PWM_TRGSTS1_CMR4FLAG_R_Msk       (0x1ul << PWM_TRGSTS1_CMR4FLAG_R_Pos)             /*!< PWM_T::TRGSTS1: CMR4FLAG_R Mask           */

#define PWM_TRGSTS1_CNT4FLAG_Pos         (1)                                               /*!< PWM_T::TRGSTS1: CNT4FLAG Position         */
#define PWM_TRGSTS1_CNT4FLAG_Msk         (0x1ul << PWM_TRGSTS1_CNT4FLAG_Pos)               /*!< PWM_T::TRGSTS1: CNT4FLAG Mask             */

#define PWM_TRGSTS1_CMR4FLAG_F_Pos       (2)                                               /*!< PWM_T::TRGSTS1: CMR4FLAG_F Position       */
#define PWM_TRGSTS1_CMR4FLAG_F_Msk       (0x1ul << PWM_TRGSTS1_CMR4FLAG_F_Pos)             /*!< PWM_T::TRGSTS1: CMR4FLAG_F Mask           */

#define PWM_TRGSTS1_PERID4FLAG_Pos       (3)                                               /*!< PWM_T::TRGSTS1: PERID4FLAG Position       */
#define PWM_TRGSTS1_PERID4FLAG_Msk       (0x1ul << PWM_TRGSTS1_PERID4FLAG_Pos)             /*!< PWM_T::TRGSTS1: PERID4FLAG Mask           */

#define PWM_TRGSTS1_CMR5FLAG_R_Pos       (8)                                               /*!< PWM_T::TRGSTS1: CMR5FLAG_R Position       */
#define PWM_TRGSTS1_CMR5FLAG_R_Msk       (0x1ul << PWM_TRGSTS1_CMR5FLAG_R_Pos)             /*!< PWM_T::TRGSTS1: CMR5FLAG_R Mask           */

#define PWM_TRGSTS1_CNT5FLAG_Pos         (9)                                               /*!< PWM_T::TRGSTS1: CNT5FLAG Position         */
#define PWM_TRGSTS1_CNT5FLAG_Msk         (0x1ul << PWM_TRGSTS1_CNT5FLAG_Pos)               /*!< PWM_T::TRGSTS1: CNT5FLAG Mask             */

#define PWM_TRGSTS1_CMR5FLAG_F_Pos       (10)                                              /*!< PWM_T::TRGSTS1: CMR5FLAG_F Position       */
#define PWM_TRGSTS1_CMR5FLAG_F_Msk       (0x1ul << PWM_TRGSTS1_CMR5FLAG_F_Pos)             /*!< PWM_T::TRGSTS1: CMR5FLAG_F Mask           */

#define PWM_TRGSTS1_PERID5FLAG_Pos       (11)                                              /*!< PWM_T::TRGSTS1: PERID5FLAG Position       */
#define PWM_TRGSTS1_PERID5FLAG_Msk       (0x1ul << PWM_TRGSTS1_PERID5FLAG_Pos)             /*!< PWM_T::TRGSTS1: PERID5FLAG Mask           */

#define PWM_PHCHG_D0_Pos                 (0)                                               /*!< PWM_T::PHCHG: D0 Position                 */
#define PWM_PHCHG_D0_Msk                 (0x1ul << PWM_PHCHG_D0_Pos)                       /*!< PWM_T::PHCHG: D0 Mask                     */

#define PWM_PHCHG_D1_Pos                 (1)                                               /*!< PWM_T::PHCHG: D1 Position                 */
#define PWM_PHCHG_D1_Msk                 (0x1ul << PWM_PHCHG_D1_Pos)                       /*!< PWM_T::PHCHG: D1 Mask                     */

#define PWM_PHCHG_D2_Pos                 (2)                                               /*!< PWM_T::PHCHG: D2 Position                 */
#define PWM_PHCHG_D2_Msk                 (0x1ul << PWM_PHCHG_D2_Pos)                       /*!< PWM_T::PHCHG: D2 Mask                     */

#define PWM_PHCHG_D3_Pos                 (3)                                               /*!< PWM_T::PHCHG: D3 Position                 */
#define PWM_PHCHG_D3_Msk                 (0x1ul << PWM_PHCHG_D3_Pos)                       /*!< PWM_T::PHCHG: D3 Mask                     */

#define PWM_PHCHG_D4_Pos                 (4)                                               /*!< PWM_T::PHCHG: D4 Position                 */
#define PWM_PHCHG_D4_Msk                 (0x1ul << PWM_PHCHG_D4_Pos)                       /*!< PWM_T::PHCHG: D4 Mask                     */

#define PWM_PHCHG_D5_Pos                 (5)                                               /*!< PWM_T::PHCHG: D5 Position                 */
#define PWM_PHCHG_D5_Msk                 (0x1ul << PWM_PHCHG_D5_Pos)                       /*!< PWM_T::PHCHG: D5 Mask                     */

#define PWM_PHCHG_D6_Pos                 (6)                                               /*!< PWM_T::PHCHG: D6 Position                 */
#define PWM_PHCHG_D6_Msk                 (0x1ul << PWM_PHCHG_D6_Pos)                       /*!< PWM_T::PHCHG: D6 Mask                     */

#define PWM_PHCHG_D7_Pos                 (7)                                               /*!< PWM_T::PHCHG: D7 Position                 */
#define PWM_PHCHG_D7_Msk                 (0x1ul << PWM_PHCHG_D7_Pos)                       /*!< PWM_T::PHCHG: D7 Mask                     */

#define PWM_PHCHG_PWM0_Pos               (8)                                               /*!< PWM_T::PHCHG: PWM0 Position               */
#define PWM_PHCHG_PWM0_Msk               (0x1ul << PWM_PHCHG_PWM0_Pos)                     /*!< PWM_T::PHCHG: PWM0 Mask                   */

#define PWM_PHCHG_PWM1_Pos               (9)                                               /*!< PWM_T::PHCHG: PWM1 Position               */
#define PWM_PHCHG_PWM1_Msk               (0x1ul << PWM_PHCHG_PWM1_Pos)                     /*!< PWM_T::PHCHG: PWM1 Mask                   */

#define PWM_PHCHG_PWM2_Pos               (10)                                              /*!< PWM_T::PHCHG: PWM2 Position               */
#define PWM_PHCHG_PWM2_Msk               (0x1ul << PWM_PHCHG_PWM2_Pos)                     /*!< PWM_T::PHCHG: PWM2 Mask                   */

#define PWM_PHCHG_PWM3_Pos               (11)                                              /*!< PWM_T::PHCHG: PWM3 Position               */
#define PWM_PHCHG_PWM3_Msk               (0x1ul << PWM_PHCHG_PWM3_Pos)                     /*!< PWM_T::PHCHG: PWM3 Mask                   */

#define PWM_PHCHG_PWM4_Pos               (12)                                              /*!< PWM_T::PHCHG: PWM4 Position               */
#define PWM_PHCHG_PWM4_Msk               (0x1ul << PWM_PHCHG_PWM4_Pos)                     /*!< PWM_T::PHCHG: PWM4 Mask                   */

#define PWM_PHCHG_PWM5_Pos               (13)                                              /*!< PWM_T::PHCHG: PWM5 Position               */
#define PWM_PHCHG_PWM5_Msk               (0x1ul << PWM_PHCHG_PWM5_Pos)                     /*!< PWM_T::PHCHG: PWM5 Mask                   */

#define PWM_PHCHG_ACCNT0_Pos             (14)                                              /*!< PWM_T::PHCHG: ACCNT0 Position             */
#define PWM_PHCHG_ACCNT0_Msk             (0x1ul << PWM_PHCHG_ACCNT0_Pos)                   /*!< PWM_T::PHCHG: ACCNT0 Mask                 */

#define PWM_PHCHG_ACCNT1_Pos             (15)                                              /*!< PWM_T::PHCHG: ACCNT1 Position             */
#define PWM_PHCHG_ACCNT1_Msk             (0x1ul << PWM_PHCHG_ACCNT1_Pos)                   /*!< PWM_T::PHCHG: ACCNT1 Mask                 */

#define PWM_PHCHG_CH01TOFF1_Pos          (16)                                              /*!< PWM_T::PHCHG: CH01TOFF1 Position          */
#define PWM_PHCHG_CH01TOFF1_Msk          (0x1ul << PWM_PHCHG_CH01TOFF1_Pos)                /*!< PWM_T::PHCHG: CH01TOFF1 Mask              */

#define PWM_PHCHG_CH11TOFF1_Pos          (17)                                              /*!< PWM_T::PHCHG: CH11TOFF1 Position          */
#define PWM_PHCHG_CH11TOFF1_Msk          (0x1ul << PWM_PHCHG_CH11TOFF1_Pos)                /*!< PWM_T::PHCHG: CH11TOFF1 Mask              */

#define PWM_PHCHG_CH21TOFF1_Pos          (18)                                              /*!< PWM_T::PHCHG: CH21TOFF1 Position          */
#define PWM_PHCHG_CH21TOFF1_Msk          (0x1ul << PWM_PHCHG_CH21TOFF1_Pos)                /*!< PWM_T::PHCHG: CH21TOFF1 Mask              */

#define PWM_PHCHG_CH31TOFF1_Pos          (19)                                              /*!< PWM_T::PHCHG: CH31TOFF1 Position          */
#define PWM_PHCHG_CH31TOFF1_Msk          (0x1ul << PWM_PHCHG_CH31TOFF1_Pos)                /*!< PWM_T::PHCHG: CH31TOFF1 Mask              */

#define PWM_PHCHG_CMP1SEL_Pos            (20)                                              /*!< PWM_T::PHCHG: CMP1SEL Position            */
#define PWM_PHCHG_CMP1SEL_Msk            (0x3ul << PWM_PHCHG_CMP1SEL_Pos)                  /*!< PWM_T::PHCHG: CMP1SEL Mask                */

#define PWM_PHCHG_T1_Pos                 (22)                                              /*!< PWM_T::PHCHG: T1 Position                 */
#define PWM_PHCHG_T1_Msk                 (0x1ul << PWM_PHCHG_T1_Pos)                       /*!< PWM_T::PHCHG: T1 Mask                     */

#define PWM_PHCHG_CE1_Pos                (23)                                              /*!< PWM_T::PHCHG: CE1 Position                */
#define PWM_PHCHG_CE1_Msk                (0x1ul << PWM_PHCHG_CE1_Pos)                      /*!< PWM_T::PHCHG: CE1 Mask                    */

#define PWM_PHCHG_CH01TOFF0_Pos          (24)                                              /*!< PWM_T::PHCHG: CH01TOFF0 Position          */
#define PWM_PHCHG_CH01TOFF0_Msk          (0x1ul << PWM_PHCHG_CH01TOFF0_Pos)                /*!< PWM_T::PHCHG: CH01TOFF0 Mask              */

#define PWM_PHCHG_CH11TOFF0_Pos          (25)                                              /*!< PWM_T::PHCHG: CH11TOFF0 Position          */
#define PWM_PHCHG_CH11TOFF0_Msk          (0x1ul << PWM_PHCHG_CH11TOFF0_Pos)                /*!< PWM_T::PHCHG: CH11TOFF0 Mask              */

#define PWM_PHCHG_CH21TOFF0_Pos          (26)                                              /*!< PWM_T::PHCHG: CH21TOFF0 Position          */
#define PWM_PHCHG_CH21TOFF0_Msk          (0x1ul << PWM_PHCHG_CH21TOFF0_Pos)                /*!< PWM_T::PHCHG: CH21TOFF0 Mask              */

#define PWM_PHCHG_CH31TOFF0_Pos          (27)                                              /*!< PWM_T::PHCHG: CH31TOFF0 Position          */
#define PWM_PHCHG_CH31TOFF0_Msk          (0x1ul << PWM_PHCHG_CH31TOFF0_Pos)                /*!< PWM_T::PHCHG: CH31TOFF0 Mask              */

#define PWM_PHCHG_CMP0SEL_Pos            (28)                                              /*!< PWM_T::PHCHG: CMP0SEL Position            */
#define PWM_PHCHG_CMP0SEL_Msk            (0x3ul << PWM_PHCHG_CMP0SEL_Pos)                  /*!< PWM_T::PHCHG: CMP0SEL Mask                */

#define PWM_PHCHG_T0_Pos                 (30)                                              /*!< PWM_T::PHCHG: T0 Position                 */
#define PWM_PHCHG_T0_Msk                 (0x1ul << PWM_PHCHG_T0_Pos)                       /*!< PWM_T::PHCHG: T0 Mask                     */

#define PWM_PHCHG_CE0_Pos                (31)                                              /*!< PWM_T::PHCHG: CE0 Position                */
#define PWM_PHCHG_CE0_Msk                (0x1ul << PWM_PHCHG_CE0_Pos)                      /*!< PWM_T::PHCHG: CE0 Mask                    */

#define PWM_PHCHGNXT_D0_Pos              (0)                                               /*!< PWM_T::PHCHGNXT: D0 Position              */
#define PWM_PHCHGNXT_D0_Msk              (0x1ul << PWM_PHCHGNXT_D0_Pos)                    /*!< PWM_T::PHCHGNXT: D0 Mask                  */

#define PWM_PHCHGNXT_D1_Pos              (1)                                               /*!< PWM_T::PHCHGNXT: D1 Position              */
#define PWM_PHCHGNXT_D1_Msk              (0x1ul << PWM_PHCHGNXT_D1_Pos)                    /*!< PWM_T::PHCHGNXT: D1 Mask                  */

#define PWM_PHCHGNXT_D2_Pos              (2)                                               /*!< PWM_T::PHCHGNXT: D2 Position              */
#define PWM_PHCHGNXT_D2_Msk              (0x1ul << PWM_PHCHGNXT_D2_Pos)                    /*!< PWM_T::PHCHGNXT: D2 Mask                  */

#define PWM_PHCHGNXT_D3_Pos              (3)                                               /*!< PWM_T::PHCHGNXT: D3 Position              */
#define PWM_PHCHGNXT_D3_Msk              (0x1ul << PWM_PHCHGNXT_D3_Pos)                    /*!< PWM_T::PHCHGNXT: D3 Mask                  */

#define PWM_PHCHGNXT_D4_Pos              (4)                                               /*!< PWM_T::PHCHGNXT: D4 Position              */
#define PWM_PHCHGNXT_D4_Msk              (0x1ul << PWM_PHCHGNXT_D4_Pos)                    /*!< PWM_T::PHCHGNXT: D4 Mask                  */

#define PWM_PHCHGNXT_D5_Pos              (5)                                               /*!< PWM_T::PHCHGNXT: D5 Position              */
#define PWM_PHCHGNXT_D5_Msk              (0x1ul << PWM_PHCHGNXT_D5_Pos)                    /*!< PWM_T::PHCHGNXT: D5 Mask                  */

#define PWM_PHCHGNXT_D6_Pos              (6)                                               /*!< PWM_T::PHCHGNXT: D6 Position              */
#define PWM_PHCHGNXT_D6_Msk              (0x1ul << PWM_PHCHGNXT_D6_Pos)                    /*!< PWM_T::PHCHGNXT: D6 Mask                  */

#define PWM_PHCHGNXT_D7_Pos              (7)                                               /*!< PWM_T::PHCHGNXT: D7 Position              */
#define PWM_PHCHGNXT_D7_Msk              (0x1ul << PWM_PHCHGNXT_D7_Pos)                    /*!< PWM_T::PHCHGNXT: D7 Mask                  */

#define PWM_PHCHGNXT_PWM0_Pos            (8)                                               /*!< PWM_T::PHCHGNXT: PWM0 Position            */
#define PWM_PHCHGNXT_PWM0_Msk            (0x1ul << PWM_PHCHGNXT_PWM0_Pos)                  /*!< PWM_T::PHCHGNXT: PWM0 Mask                */

#define PWM_PHCHGNXT_PWM1_Pos            (9)                                               /*!< PWM_T::PHCHGNXT: PWM1 Position            */
#define PWM_PHCHGNXT_PWM1_Msk            (0x1ul << PWM_PHCHGNXT_PWM1_Pos)                  /*!< PWM_T::PHCHGNXT: PWM1 Mask                */

#define PWM_PHCHGNXT_PWM2_Pos            (10)                                              /*!< PWM_T::PHCHGNXT: PWM2 Position            */
#define PWM_PHCHGNXT_PWM2_Msk            (0x1ul << PWM_PHCHGNXT_PWM2_Pos)                  /*!< PWM_T::PHCHGNXT: PWM2 Mask                */

#define PWM_PHCHGNXT_PWM3_Pos            (11)                                              /*!< PWM_T::PHCHGNXT: PWM3 Position            */
#define PWM_PHCHGNXT_PWM3_Msk            (0x1ul << PWM_PHCHGNXT_PWM3_Pos)                  /*!< PWM_T::PHCHGNXT: PWM3 Mask                */

#define PWM_PHCHGNXT_PWM4_Pos            (12)                                              /*!< PWM_T::PHCHGNXT: PWM4 Position            */
#define PWM_PHCHGNXT_PWM4_Msk            (0x1ul << PWM_PHCHGNXT_PWM4_Pos)                  /*!< PWM_T::PHCHGNXT: PWM4 Mask                */

#define PWM_PHCHGNXT_PWM5_Pos            (13)                                              /*!< PWM_T::PHCHGNXT: PWM5 Position            */
#define PWM_PHCHGNXT_PWM5_Msk            (0x1ul << PWM_PHCHGNXT_PWM5_Pos)                  /*!< PWM_T::PHCHGNXT: PWM5 Mask                */

#define PWM_PHCHGNXT_ACCNT0_Pos          (14)                                              /*!< PWM_T::PHCHGNXT: ACCNT0 Position          */
#define PWM_PHCHGNXT_ACCNT0_Msk          (0x1ul << PWM_PHCHGNXT_ACCNT0_Pos)                /*!< PWM_T::PHCHGNXT: ACCNT0 Mask              */

#define PWM_PHCHGNXT_ACCNT1_Pos          (15)                                              /*!< PWM_T::PHCHGNXT: ACCNT1 Position          */
#define PWM_PHCHGNXT_ACCNT1_Msk          (0x1ul << PWM_PHCHGNXT_ACCNT1_Pos)                /*!< PWM_T::PHCHGNXT: ACCNT1 Mask              */

#define PWM_PHCHGNXT_CH01TOFF1_Pos       (16)                                              /*!< PWM_T::PHCHGNXT: CH01TOFF1 Position       */
#define PWM_PHCHGNXT_CH01TOFF1_Msk       (0x1ul << PWM_PHCHGNXT_CH01TOFF1_Pos)             /*!< PWM_T::PHCHGNXT: CH01TOFF1 Mask           */

#define PWM_PHCHGNXT_CH11TOFF1_Pos       (17)                                              /*!< PWM_T::PHCHGNXT: CH11TOFF1 Position       */
#define PWM_PHCHGNXT_CH11TOFF1_Msk       (0x1ul << PWM_PHCHGNXT_CH11TOFF1_Pos)             /*!< PWM_T::PHCHGNXT: CH11TOFF1 Mask           */

#define PWM_PHCHGNXT_CH21TOFF1_Pos       (18)                                              /*!< PWM_T::PHCHGNXT: CH21TOFF1 Position       */
#define PWM_PHCHGNXT_CH21TOFF1_Msk       (0x1ul << PWM_PHCHGNXT_CH21TOFF1_Pos)             /*!< PWM_T::PHCHGNXT: CH21TOFF1 Mask           */

#define PWM_PHCHGNXT_CH31TOFF1_Pos       (19)                                              /*!< PWM_T::PHCHGNXT: CH31TOFF1 Position       */
#define PWM_PHCHGNXT_CH31TOFF1_Msk       (0x1ul << PWM_PHCHGNXT_CH31TOFF1_Pos)             /*!< PWM_T::PHCHGNXT: CH31TOFF1 Mask           */

#define PWM_PHCHGNXT_CMP1SEL_Pos         (20)                                              /*!< PWM_T::PHCHGNXT: CMP1SEL Position         */
#define PWM_PHCHGNXT_CMP1SEL_Msk         (0x3ul << PWM_PHCHGNXT_CMP1SEL_Pos)               /*!< PWM_T::PHCHGNXT: CMP1SEL Mask             */

#define PWM_PHCHGNXT_T1_Pos              (22)                                              /*!< PWM_T::PHCHGNXT: T1 Position              */
#define PWM_PHCHGNXT_T1_Msk              (0x1ul << PWM_PHCHGNXT_T1_Pos)                    /*!< PWM_T::PHCHGNXT: T1 Mask                  */

#define PWM_PHCHGNXT_CE1_Pos             (23)                                              /*!< PWM_T::PHCHGNXT: CE1 Position             */
#define PWM_PHCHGNXT_CE1_Msk             (0x1ul << PWM_PHCHGNXT_CE1_Pos)                   /*!< PWM_T::PHCHGNXT: CE1 Mask                 */

#define PWM_PHCHGNXT_CH01TOFF0_Pos       (24)                                              /*!< PWM_T::PHCHGNXT: CH01TOFF0 Position       */
#define PWM_PHCHGNXT_CH01TOFF0_Msk       (0x1ul << PWM_PHCHGNXT_CH01TOFF0_Pos)             /*!< PWM_T::PHCHGNXT: CH01TOFF0 Mask           */

#define PWM_PHCHGNXT_CH11TOFF0_Pos       (25)                                              /*!< PWM_T::PHCHGNXT: CH11TOFF0 Position       */
#define PWM_PHCHGNXT_CH11TOFF0_Msk       (0x1ul << PWM_PHCHGNXT_CH11TOFF0_Pos)             /*!< PWM_T::PHCHGNXT: CH11TOFF0 Mask           */

#define PWM_PHCHGNXT_CH21TOFF0_Pos       (26)                                              /*!< PWM_T::PHCHGNXT: CH21TOFF0 Position       */
#define PWM_PHCHGNXT_CH21TOFF0_Msk       (0x1ul << PWM_PHCHGNXT_CH21TOFF0_Pos)             /*!< PWM_T::PHCHGNXT: CH21TOFF0 Mask           */

#define PWM_PHCHGNXT_CH31TOFF0_Pos       (27)                                              /*!< PWM_T::PHCHGNXT: CH31TOFF0 Position       */
#define PWM_PHCHGNXT_CH31TOFF0_Msk       (0x1ul << PWM_PHCHGNXT_CH31TOFF0_Pos)             /*!< PWM_T::PHCHGNXT: CH31TOFF0 Mask           */

#define PWM_PHCHGNXT_CMP0SEL_Pos         (28)                                              /*!< PWM_T::PHCHGNXT: CMP0SEL Position         */
#define PWM_PHCHGNXT_CMP0SEL_Msk         (0x3ul << PWM_PHCHGNXT_CMP0SEL_Pos)               /*!< PWM_T::PHCHGNXT: CMP0SEL Mask             */

#define PWM_PHCHGNXT_T0_Pos              (30)                                              /*!< PWM_T::PHCHGNXT: T0 Position              */
#define PWM_PHCHGNXT_T0_Msk              (0x1ul << PWM_PHCHGNXT_T0_Pos)                    /*!< PWM_T::PHCHGNXT: T0 Mask                  */

#define PWM_PHCHGNXT_CE0_Pos             (31)                                              /*!< PWM_T::PHCHGNXT: CE0 Position             */
#define PWM_PHCHGNXT_CE0_Msk             (0x1ul << PWM_PHCHGNXT_CE0_Pos)                   /*!< PWM_T::PHCHGNXT: CE0 Mask                 */

#define PWM_PHCHGMASK_MASK6_Pos          (6)                                               /*!< PWM_T::PHCHGMASK: MASK6 Position          */
#define PWM_PHCHGMASK_MASK6_Msk          (0x1ul << PWM_PHCHGMASK_MASK6_Pos)                /*!< PWM_T::PHCHGMASK: MASK6 Mask              */

#define PWM_PHCHGMASK_MASK7_Pos          (7)                                               /*!< PWM_T::PHCHGMASK: MASK7 Position          */
#define PWM_PHCHGMASK_MASK7_Msk          (0x1ul << PWM_PHCHGMASK_MASK7_Pos)                /*!< PWM_T::PHCHGMASK: MASK7 Mask              */

#define PWM_PHCHGMASK_CMPMASK_Pos        (8)                                               /*!< PWM_T::PHCHGMASK: CMPMASK Position        */
#define PWM_PHCHGMASK_CMPMASK_Msk        (0x3ul << PWM_PHCHGMASK_CMPMASK_Pos)              /*!< PWM_T::PHCHGMASK: CMPMASK Mask            */

#define PWM_PHCHGMASK_CMPMASK0_Pos       (8)                                               /*!< PWM_T::PHCHGMASK: CMPMASK0 Position       */
#define PWM_PHCHGMASK_CMPMASK0_Msk       (0x1ul << PWM_PHCHGMASK_CMPMASK0_Pos)             /*!< PWM_T::PHCHGMASK: CMPMASK0 Mask           */

#define PWM_PHCHGMASK_CMPMASK1_Pos       (9)                                               /*!< PWM_T::PHCHGMASK: CMPMASK1 Position       */
#define PWM_PHCHGMASK_CMPMASK1_Msk       (0x1ul << PWM_PHCHGMASK_CMPMASK1_Pos)             /*!< PWM_T::PHCHGMASK: CMPMASK1 Mask           */

#define PWM_INTACCUCTL_INTACCUEN0_Pos    (0)                                               /*!< PWM_T::INTACCUCTL: INTACCUEN0 Position    */
#define PWM_INTACCUCTL_INTACCUEN0_Msk    (0x1ul << PWM_INTACCUCTL_INTACCUEN0_Pos)          /*!< PWM_T::INTACCUCTL: INTACCUEN0 Mask        */

#define PWM_INTACCUCTL_PERIODCNT_Pos     (4)                                               /*!< PWM_T::INTACCUCTL: PERIODCNT Position     */
#define PWM_INTACCUCTL_PERIODCNT_Msk     (0xful << PWM_INTACCUCTL_PERIODCNT_Pos)           /*!< PWM_T::INTACCUCTL: PERIODCNT Mask         */

/**@}*/ /* PWM_CONST */
/**@}*/ /* end of PWM register group */


/*---------------------- Serial Peripheral Interface Controller -------------------------*/
/**
    @addtogroup SPI Serial Peripheral Interface Controller(SPI)
    Memory Mapped Structure for SPI Controller
@{ */


typedef struct
{

    /**
     * @var SPI_T::CNTRL
     * Offset: 0x00  SPI Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GO_BUSY   |SPI Transfer Control Bit And Busy Status
     * |        |          |If FIFO mode is enabled, this bit will be controlled by hardware and is Read only.
     * |        |          |If FIFO mode is disabled, during the data transfer, this bit keeps the value of 1.
     * |        |          |As the transfer is finished, this bit will be cleared automatically.
     * |        |          |0 = Writing 0 to this bit to stop data transfer if SPI is transferring.
     * |        |          |1 = In Master mode, writing 1 to this bit to start the SPI data transfer; in Slave mode, writing 1 to this bit indicates that the slave is ready to communicate with a master.
     * |        |          |Note 1: When FIFO mode is disabled, all configurations should be ready before writing 1 to the GO_BUSY bit.
     * |        |          |Note 2: In SPI Slave mode, if FIFO mode is disabled and the SPI bus clock is kept at idle state during a data transfer, the GO_BUSY bit will not be cleared to 0 when slave select signal goes to inactive state.
     * |[1]     |RX_NEG    |Receive On Negative Edge
     * |        |          |0 = The received data input signal latched on the Rising edge of SPICLK.
     * |        |          |1 = The received data input signal latched on the Falling edge of SPICLK.
     * |[2]     |TX_NEG    |Transmit On Negative Edge
     * |        |          |0 = The transmitted data output signal is driven on the Rising edge of SPICLK.
     * |        |          |1 = The transmitted data output signal is driven on the Falling edge of SPICLK.
     * |[7:3]   |TX_BIT_LEN|Transmit Bit Length
     * |        |          |This field specifies how many bits are transmitted in one transmit/receive.
     * |        |          |The minimum bit length is 8 bits and can up to 32 bits.
     * |        |          |TX_BIT_LEN = 0x08 ... 8 bits.
     * |        |          |TX_BIT_LEN = 0x09 ... 9 bits.
     * |        |          |.....
     * |        |          |TX_BIT_LEN = 0x1F ... 31 bits.
     * |        |          |TX_BIT_LEN = 0x00 ... 32 bits.
     * |[10]    |LSB       |LSB First
     * |        |          |0 = The MSB is transmitted/received first.
     * |        |          |1 = The LSB is transmitted/received first.
     * |[11]    |CLKP      |Clock Polarity
     * |        |          |0 = SPICLK idle low.
     * |        |          |1 = SPICLK idle high.
     * |[15:12] |SP_CYCLE  |Suspend Interval (Master Only)
     * |        |          |The four bits provide configurable suspend interval between two successive transactions in a transfer.
     * |        |          |The definition of the suspend interval is the interval between the last clock edge of the preceding transaction word and the first clock edge of the following transaction word.
     * |        |          |The default value is 0x3.
     * |        |          |The period of the suspend interval is obtained according to the following equation:
     * |        |          |(SP_CYCLE[3:0] + 0.5) * period of SPICLK clock cycle
     * |        |          |Example:
     * |        |          |SP_CYCLE = 0x0 ... 0.5 SPICLK clock cycle.
     * |        |          |SP_CYCLE = 0x1 ... 1.5 SPICLK clock cycle.
     * |        |          |.....
     * |        |          |SP_CYCLE = 0xE ... 14.5 SPICLK clock cycle.
     * |        |          |SP_CYCLE = 0xF ... 15.5 SPICLK clock cycle.
     * |[16]    |IF        |Unit-transfer Interrupt Flag
     * |        |          |0 = The transfer does not finish yet.
     * |        |          |1 = The SPI controller has finished one unit transfer.
     * |        |          |Note 1: This bit will be cleared by writing 1 to itself.
     * |        |          |Note 2: It's a mutual mirror bit of SPI_STATUS[16].
     * |[17]    |IE        |Unit-transfer Interrupt Enable Control
     * |        |          |0 = SPI unit-transfer interrupt Disabled.
     * |        |          |1 = SPI unit-transfer interrupt Enabled.
     * |[18]    |SLAVE     |Slave Mode Control
     * |        |          |0 = Master mode.
     * |        |          |1 = Slave mode.
     * |[19]    |REORDER   |Byte Reorder Function
     * |        |          |0 = Byte reorder function Disabled.
     * |        |          |1 = Byte reorder function Enabled.
     * |        |          |Note: This setting is only available if TX_BIT_LEN is defined as 16, 24, and 32 bits.
     * |[21]    |FIFO      |FIFO Mode Enable Control
     * |        |          |0 = FIFO Mode Disabled.
     * |        |          |1 = FIFO Mode Enabled.
     * |        |          |Note 1: Before enabling FIFO mode, the other related settings should be set in advance.
     * |        |          |Note 2: In Master mode, if the FIFO mode is enabled, the GO_BUSY bit will be set to 1 automatically after writing data into the 4-depth transmit FIFO.
     * |        |          |When all data stored at transmit FIFO buffer are transferred, the GO_BUSY bit will back to 0.
     * |[24]    |RX_EMPTY  |Receive FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = The receive FIFO buffer is not empty.
     * |        |          |1 = The receive FIFO buffer is empty.
     * |        |          |Note: It's a mutual mirror bit of SPI_CNTRL[24].
     * |[25]    |RX_FULL   |Receive FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 = The receive FIFO buffer is not full.
     * |        |          |1 = The receive FIFO buffer is full.
     * |        |          |Note: It's a mutual mirror bit of SPI_STATUS[25]
     * |[26]    |TX_EMPTY  |Transmit FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = The transmit FIFO buffer is not empty.
     * |        |          |1 = The transmit FIFO buffer is empty.
     * |        |          |Note: It's a mutual mirror bit of SPI_STAUTS[26].
     * |[27]    |TX_FULL   |Transmit FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 =The transmit FIFO buffer is not full.
     * |        |          |1 =The transmit FIFO buffer is full.
     * |        |          |Note: It's a mutual mirror bit of SPI_STATUS[27].
     * @var SPI_T::DIVIDER
     * Offset: 0x04  SPI Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DIVIDER   |Clock Divider Bits (Master Only)
     * |        |          |The value in this field is the frequency divider to determine the SPI peripheral clock frequency fspi, and the SPI master's bus clock frequency on the SPICLK output pin.
     * |        |          |The frequency is obtained according to the following equation:
     * |        |          |If the bit of BCn, SPI_CNTRL2[31], is set to 0.
     * |        |          |else if BCn is set to 1,
     * |        |          |where
     * |        |          |is the SPI peripheral clock source which is defined in the CLKSEL1 register.
     * @var SPI_T::SSR
     * Offset: 0x08  SPI Slave Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SSR       |Slave Select Control Bit (Master Only)
     * |        |          |If AUTOSS bit is 0,
     * |        |          |0 = Set the SPISS line to inactive state.
     * |        |          |1 = Set the proper SPISS line to active state.
     * |        |          |If AUTOSS bit is 1,
     * |        |          |0 = Keep the SPISS line at inactive state.
     * |        |          |1 = Select the SPISS line to be automatically driven to active state for the duration of transmission/reception, and will be driven to inactive state for the rest of the time.
     * |        |          |The active state of SPISS is specified in SS_LVL bit.
     * |[2]     |SS_LVL    |Slave Select Active Level (Slave Only)
     * |        |          |It defines the active status of slave select signal (SPISS).
     * |        |          |If SS_LTRIG bit is 1:
     * |        |          |0 = The slave select signal SPISS is active at Low-level.
     * |        |          |1 = The slave select signal SPISS is active at High-level.
     * |        |          |If SS_LTRIG bit is 0:
     * |        |          |0 = The slave select signal SPISS is active at Falling-edge.
     * |        |          |1 = The slave select signal SPISS is active at Rising-edge.
     * |[3]     |AUTOSS    |Automatic Slave Selection Function Enable Bit (Master Only)
     * |        |          |0 = SPISS pin signal will be asserted/de-asserted by setting /clearing SSR bit.
     * |        |          |1 = SPISS pin signal will be generated automatically, which means that slave select signal will be asserted by the SPI controller when transmit/receive is started by setting GO_BUSY, and will be de-asserted after each transmit/receive is finished.
     * |[4]     |SS_LTRIG  |Slave Select Level Trigger Enable Bit (Slave Only)
     * |        |          |0 = The input slave select signal is edge-trigger.
     * |        |          |1 = The input slave select signal is level-trigger.
     * |[5]     |LTRIG_FLAG|Level Trigger Flag (Read Only, Slave Only)
     * |        |          |When the SS_LTRIG bit is set in Slave mode, this bit can be read to indicate the received bit number is met the requirement or not.
     * |        |          |0 = The transaction number or the transferred bit length of one transaction does not meet the specified requirements.
     * |        |          |1 = The transaction number and the transferred bit length met the specified requirements which defined in TX_BIT_LEN.
     * @var SPI_T::RX
     * Offset: 0x10  SPI Data Receive Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Bits (Read Only)
     * |        |          |The Data Receive Registers hold the value of received data of the last executed transfer.
     * |        |          |Valid bits depend on the transmit bit length field in the SPI_CNTRL register.
     * |        |          |For example, if TX_BIT_LEN is set to 0x08, bit RX [7:0] holds the received data.
     * |        |          |The values of the other bits are unknown.
     * @var SPI_T::TX
     * Offset: 0x20  SPI Data Transmit Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Bits (Write Only)
     * |        |          |The Data Transmit Registers hold the data to be transmitted in the next transfer.
     * |        |          |Valid bits depend on the transmit bit length field in the CNTRL register.
     * |        |          |For example, if TX_BIT_LEN is set to 0x08, the bit TX [7:0] will be transmitted in next transfer.
     * @var SPI_T::CNTRL2
     * Offset: 0x3C  SPI Control and Status Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |NOSLVSEL  |Slave 3-wire Mode Enable Control (Slave Only)
     * |        |          |The SPI controller work with 3-wire interface including SPICLK, SPI_MISO, and SPI_MOSI
     * |        |          |0 = The controller is 4-wire bi-direction interface.
     * |        |          |1 = The controller is 3-wire bi-direction interface in Slave mode.
     * |        |          |The controller will be ready to transmit/receive data after the GO_BUSY bit is set to 1.
     * |        |          |Note: In Slave 3-wire mode, the SS_LTRIG bit (SPI_SSR[4]) shall be set as 1.
     * |[9]     |SLV_ABORT |Slave 3-wire Mode Abort Control Bit (Slave Only)
     * |        |          |In normal operation, there is an interrupt event when the number of received bits meets the requirement which defined in TX_BIT_LEN.
     * |        |          |If the number of received bits is less than the requirement and there is no more bus clock input over one transfer time in Slave 3-wire mode, user can set this bit to force the current transfer done and then user can get a unit transfer interrupt event.
     * |        |          |0 = No force the transfer done when the NOSLVSEL bit is set to 1.
     * |        |          |1 = Force the transfer done when the NOSLVSEL bit is set to 1.
     * |        |          |Note: This bit will be cleared to 0 automatically by hardware after it is set to 1 by software.
     * |[10]    |SSTA_INTEN|Slave 3-wire Mode Start Interrupt Enable Control (Slave Only)
     * |        |          |It is used to enable interrupt when the transfer has started in slave 3-wire mode.
     * |        |          |If there is no transfer done interrupt over the time period which is defined by user after the transfer start, user can set the SLV_ABORT bit to force the transfer done.
     * |        |          |0 = Transaction start interrupt Disabled.
     * |        |          |1 = Transaction start interrupt Enabled.
     * |        |          |Note: It will be cleared to 0 as the current transfer is done or the SLV_START_INTSTS bit is cleared to 0.
     * |[11]    |SLV_START_INTSTS|Slave 3-wire Mode Start Interrupt Status (Slave Only)
     * |        |          |This bit dedicates if a transaction has started in slave 3-wire mode.
     * |        |          |0 = Slave does not detect any SPI bus clock transfer since the SSTA_INTEN bit was set to 1.
     * |        |          |1 = The transfer has started in slave 3-wire mode.
     * |        |          |Note 1: It will be cleared automatically when a transaction is done or by writing 1 to this bit.
     * |        |          |Note 2: It is a mutual mirror bit of SPI_STATUS[11].
     * |[16]    |SS_INT_OPT|Slave Select Inactive Interrupt Option (Slave Only)
     * |        |          |0 = As the slave select signal goes to inactive level, the IF bit will NOT be set to 1.
     * |        |          |1 = As the slave select signal goes to inactive level, the IF bit will be set to 1.
     * |        |          |Note: This setting is only available if the SPI controller is configured as level trigger in slave device.
     * |[31]    |BCn       |Clock Configuration Backward Compatible Option
     * |        |          |0 = The clock configuration is backward compatible.
     * |        |          |1 = The clock configuration is not backward compatible.
     * |        |          |Note: Refer to the description of SPI_DIVIDER register for details.
     * @var SPI_T::FIFO_CTL
     * Offset: 0x40  SPI FIFO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RX_CLR    |Clear Receive FIFO Buffer
     * |        |          |0 = No effect.
     * |        |          |1 = Clear receive FIFO buffer.
     * |        |          |Note: This bit will be cleared to 0 by hardware after software sets it to 1 and the receive FIFO is cleared.
     * |[1]     |TX_CLR    |Clear Transmit FIFO Buffer
     * |        |          |0 = No effect.
     * |        |          |1 = Clear transmit FIFO buffer.
     * |        |          |Note: This bit will be cleared to 0 by hardware after software sets it to 1 and the transmit FIFO is cleared.
     * |[2]     |RX_INTEN  |Receive Threshold Interrupt Enable Control
     * |        |          |0 = Receive threshold interrupt Disabled.
     * |        |          |1 = Receive threshold interrupt Enabled.
     * |[3]     |TX_INTEN  |Transmit Threshold Interrupt Enable Control
     * |        |          |0 = Transmit threshold interrupt Disabled.
     * |        |          |1 = Transmit threshold interrupt Enabled.
     * |[6]     |RXOV_INTEN|Receive FIFO Overrun Interrupt Enable Control
     * |        |          |0 = Receive FIFO overrun interrupt Disabled.
     * |        |          |1 = Receive FIFO overrun interrupt Enabled.
     * |[21]    |TIMEOUT_INTEN|Receive FIFO Time-out Interrupt Enable Control
     * |        |          |0 = Time-out interrupt Disabled.
     * |        |          |1 = Time-out interrupt Enabled.
     * |[25:24] |RX_THRESHOLD|Received FIFO Threshold
     * |        |          |If the valid data count of the receive FIFO buffer is larger than the RX_THRESHOLD setting, the RX_INTSTS bit will be set to 1, else the RX_INTSTS bit will be cleared to 0.
     * |[29:28] |TX_THRESHOLD|Transmit FIFO Threshold
     * |        |          |If the valid data count of the transmit FIFO buffer is less than or equal to the TX_THRESHOLD setting, the TX_INTSTS bit will be set to 1, else the TX_INTSTS bit will be cleared to 0.
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
     * |        |          |0 = No overrun in receive FIFO.
     * |        |          |1 = Overrun in receive FIFO.
     * |        |          |Note: This bit will be cleared by writing 1 to itself.
     * |[4]     |TX_INTSTS |Transmit FIFO Threshold Interrupt Status (Read Only)
     * |        |          |0 = The valid data count within the transmit FIFO buffer is larger than the setting value of TX_THRESHOLD.
     * |        |          |1 = The valid data count within the transmit FIFO buffer is less than or equal to the setting value of TX_THRESHOLD.
     * |        |          |Note: If TX_INTEN = 1 and TX_INTSTS = 1, the SPI controller will generate a SPI interrupt request.
     * |[11]    |SLV_START_INTSTS|Slave Start Interrupt Status (Slave Only)
     * |        |          |It is used to dedicate that the transfer has started in slave 3-wire mode.
     * |        |          |0 = Slave does not detect any SPI bus clock transfer since the SSTA_INTEN bit was set to 1.
     * |        |          |1 = The transfer has started in slave 3-wire mode.
     * |        |          |Note 1: It will be cleared as transfer done or by writing one to this bit.
     * |        |          |Note 2: It's a mutual mirror bit of SPI_CNTRL2[11].
     * |[15:12] |RX_FIFO_COUNT|Receive FIFO Data Count (Read Only)
     * |        |          |Indicates the valid data count of receive FIFO buffer.
     * |[16]    |IF        |SPI Unit-transfer Interrupt Flag
     * |        |          |0 = The transfer does not finish yet.
     * |        |          |1 = The SPI controller has finished one unit transfer.
     * |        |          |Note 1: This bit will be cleared by writing 1 to itself.
     * |        |          |Note 2: It's a mutual mirror bit of SPI_CNTRL[16].
     * |[20]    |TIMEOUT   |Time-out Interrupt Flag
     * |        |          |0 = No receive FIFO time-out event.
     * |        |          |1 = The receive FIFO buffer is not empty and it does not be read over 64 SPI clock periods in Master mode or over 576 SPI peripheral clock periods in Slave mode.
     * |        |          |When the received FIFO buffer is read by software, the time-out status will be cleared automatically.
     * |        |          |Note: This bit will be cleared by writing 1 to itself.
     * |[24]    |RX_EMPTY  |Receive FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = The receive FIFO buffer is not empty.
     * |        |          |1 = The receive FIFO buffer is empty.
     * |        |          |Note: It's a mutual mirror bit of SPI_CNTRL[24].
     * |[25]    |RX_FULL   |Receive FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 = The receive FIFO buffer is not full.
     * |        |          |1 = The receive FIFO buffer is full.
     * |        |          |Note: It's a mutual mirror bit of SPI_CNTRL[25].
     * |[26]    |TX_EMPTY  |Transmit FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = The transmit FIFO buffer is not empty.
     * |        |          |1 = The transmit FIFO buffer is empty.
     * |        |          |Note: It's a mutual mirror bit of SPI_CNTRL[26].
     * |[27]    |TX_FULL   |Transmit FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 = The transmit FIFO buffer is not full.
     * |        |          |1 = The transmit FIFO buffer is full.
     * |        |          |Note: It's a mutual mirror bit of SPI_CNTRL[27].
     * |[31:28] |TX_FIFO_COUNT|Transmit FIFO Data Count (Read Only)
     * |        |          |Indicates the valid data count of transmit FIFO buffer.
     */

    __IO uint32_t CNTRL;         /* Offset: 0x00  SPI Control and Status Register                                    */
    __IO uint32_t DIVIDER;       /* Offset: 0x04  SPI Clock Divider Register                                         */
    __IO uint32_t SSR;           /* Offset: 0x08  SPI Slave Select Register                                          */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE0[1];
    /// @endcond //HIDDEN_SYMBOLS
    __I  uint32_t RX;            /* Offset: 0x10  SPI Data Receive Register                                          */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE1[3];
    /// @endcond //HIDDEN_SYMBOLS
    __O  uint32_t TX;            /* Offset: 0x20  SPI Data Transmit Register                                         */
    /// @cond HIDDEN_SYMBOLS
    __I  uint32_t RESERVE2[6];
    /// @endcond //HIDDEN_SYMBOLS
    __IO uint32_t CNTRL2;        /* Offset: 0x3C  SPI Control and Status Register 2                                  */
    __IO uint32_t FIFO_CTL;      /* Offset: 0x40  SPI FIFO Control Register                                          */
    __IO uint32_t STATUS;        /* Offset: 0x44  SPI Status Register                                                */

} SPI_T;



/**
    @addtogroup SPI_CONST SPI Bit Field Definition
    Constant Definitions for SPI Controller
@{ */

#define SPI_CNTRL_GO_BUSY_Pos            (0)                                               /*!< SPI_T::CNTRL: GO_BUSY Position            */
#define SPI_CNTRL_GO_BUSY_Msk            (0x1ul << SPI_CNTRL_GO_BUSY_Pos)                  /*!< SPI_T::CNTRL: GO_BUSY Mask                */

#define SPI_CNTRL_RX_NEG_Pos             (1)                                               /*!< SPI_T::CNTRL: RX_NEG Position             */
#define SPI_CNTRL_RX_NEG_Msk             (0x1ul << SPI_CNTRL_RX_NEG_Pos)                   /*!< SPI_T::CNTRL: RX_NEG Mask                 */

#define SPI_CNTRL_TX_NEG_Pos             (2)                                               /*!< SPI_T::CNTRL: TX_NEG Position             */
#define SPI_CNTRL_TX_NEG_Msk             (0x1ul << SPI_CNTRL_TX_NEG_Pos)                   /*!< SPI_T::CNTRL: TX_NEG Mask                 */

#define SPI_CNTRL_TX_BIT_LEN_Pos         (3)                                               /*!< SPI_T::CNTRL: TX_BIT_LEN Position         */
#define SPI_CNTRL_TX_BIT_LEN_Msk         (0x1ful << SPI_CNTRL_TX_BIT_LEN_Pos)              /*!< SPI_T::CNTRL: TX_BIT_LEN Mask             */

#define SPI_CNTRL_LSB_Pos                (10)                                              /*!< SPI_T::CNTRL: LSB Position                */
#define SPI_CNTRL_LSB_Msk                (0x1ul << SPI_CNTRL_LSB_Pos)                      /*!< SPI_T::CNTRL: LSB Mask                    */

#define SPI_CNTRL_CLKP_Pos               (11)                                              /*!< SPI_T::CNTRL: CLKP Position               */
#define SPI_CNTRL_CLKP_Msk               (0x1ul << SPI_CNTRL_CLKP_Pos)                     /*!< SPI_T::CNTRL: CLKP Mask                   */

#define SPI_CNTRL_SP_CYCLE_Pos           (12)                                              /*!< SPI_T::CNTRL: SP_CYCLE Position           */
#define SPI_CNTRL_SP_CYCLE_Msk           (0xful << SPI_CNTRL_SP_CYCLE_Pos)                 /*!< SPI_T::CNTRL: SP_CYCLE Mask               */

#define SPI_CNTRL_IF_Pos                 (16)                                              /*!< SPI_T::CNTRL: IF Position                 */
#define SPI_CNTRL_IF_Msk                 (0x1ul << SPI_CNTRL_IF_Pos)                       /*!< SPI_T::CNTRL: IF Mask                     */

#define SPI_CNTRL_IE_Pos                 (17)                                              /*!< SPI_T::CNTRL: IE Position                 */
#define SPI_CNTRL_IE_Msk                 (0x1ul << SPI_CNTRL_IE_Pos)                       /*!< SPI_T::CNTRL: IE Mask                     */

#define SPI_CNTRL_SLAVE_Pos              (18)                                              /*!< SPI_T::CNTRL: SLAVE Position              */
#define SPI_CNTRL_SLAVE_Msk              (0x1ul << SPI_CNTRL_SLAVE_Pos)                    /*!< SPI_T::CNTRL: SLAVE Mask                  */

#define SPI_CNTRL_REORDER_Pos            (19)                                              /*!< SPI_T::CNTRL: REORDER Position            */
#define SPI_CNTRL_REORDER_Msk            (0x1ul << SPI_CNTRL_REORDER_Pos)                  /*!< SPI_T::CNTRL: REORDER Mask                */

#define SPI_CNTRL_FIFO_Pos               (21)                                              /*!< SPI_T::CNTRL: FIFO Position               */
#define SPI_CNTRL_FIFO_Msk               (0x1ul << SPI_CNTRL_FIFO_Pos)                     /*!< SPI_T::CNTRL: FIFO Mask                   */

#define SPI_CNTRL_RX_EMPTY_Pos           (24)                                              /*!< SPI_T::CNTRL: RX_EMPTY Position           */
#define SPI_CNTRL_RX_EMPTY_Msk           (0x1ul << SPI_CNTRL_RX_EMPTY_Pos)                 /*!< SPI_T::CNTRL: RX_EMPTY Mask               */

#define SPI_CNTRL_RX_FULL_Pos            (25)                                              /*!< SPI_T::CNTRL: RX_FULL Position            */
#define SPI_CNTRL_RX_FULL_Msk            (0x1ul << SPI_CNTRL_RX_FULL_Pos)                  /*!< SPI_T::CNTRL: RX_FULL Mask                */

#define SPI_CNTRL_TX_EMPTY_Pos           (26)                                              /*!< SPI_T::CNTRL: TX_EMPTY Position           */
#define SPI_CNTRL_TX_EMPTY_Msk           (0x1ul << SPI_CNTRL_TX_EMPTY_Pos)                 /*!< SPI_T::CNTRL: TX_EMPTY Mask               */

#define SPI_CNTRL_TX_FULL_Pos            (27)                                              /*!< SPI_T::CNTRL: TX_FULL Position            */
#define SPI_CNTRL_TX_FULL_Msk            (0x1ul << SPI_CNTRL_TX_FULL_Pos)                  /*!< SPI_T::CNTRL: TX_FULL Mask                */

#define SPI_DIVIDER_DIVIDER_Pos          (0)                                               /*!< SPI_T::DIVIDER: DIVIDER Position          */
#define SPI_DIVIDER_DIVIDER_Msk          (0xfful << SPI_DIVIDER_DIVIDER_Pos)               /*!< SPI_T::DIVIDER: DIVIDER Mask              */

#define SPI_SSR_SSR_Pos                  (0)                                               /*!< SPI_T::SSR: SSR Position                  */
#define SPI_SSR_SSR_Msk                  (0x1ul << SPI_SSR_SSR_Pos)                        /*!< SPI_T::SSR: SSR Mask                      */

#define SPI_SSR_SS_LVL_Pos               (2)                                               /*!< SPI_T::SSR: SS_LVL Position               */
#define SPI_SSR_SS_LVL_Msk               (0x1ul << SPI_SSR_SS_LVL_Pos)                     /*!< SPI_T::SSR: SS_LVL Mask                   */

#define SPI_SSR_AUTOSS_Pos               (3)                                               /*!< SPI_T::SSR: AUTOSS Position               */
#define SPI_SSR_AUTOSS_Msk               (0x1ul << SPI_SSR_AUTOSS_Pos)                     /*!< SPI_T::SSR: AUTOSS Mask                   */

#define SPI_SSR_SS_LTRIG_Pos             (4)                                               /*!< SPI_T::SSR: SS_LTRIG Position             */
#define SPI_SSR_SS_LTRIG_Msk             (0x1ul << SPI_SSR_SS_LTRIG_Pos)                   /*!< SPI_T::SSR: SS_LTRIG Mask                 */

#define SPI_SSR_LTRIG_FLAG_Pos           (5)                                               /*!< SPI_T::SSR: LTRIG_FLAG Position           */
#define SPI_SSR_LTRIG_FLAG_Msk           (0x1ul << SPI_SSR_LTRIG_FLAG_Pos)                 /*!< SPI_T::SSR: LTRIG_FLAG Mask               */

#define SPI_RX_RX_Pos                    (0)                                               /*!< SPI_T::RX: RX Position                    */
#define SPI_RX_RX_Msk                    (0xfffffffful << SPI_RX_RX_Pos)                   /*!< SPI_T::RX: RX Mask                        */

#define SPI_TX_TX_Pos                    (0)                                               /*!< SPI_T::TX: TX Position                    */
#define SPI_TX_TX_Msk                    (0xfffffffful << SPI_TX_TX_Pos)                   /*!< SPI_T::TX: TX Mask                        */

#define SPI_CNTRL2_NOSLVSEL_Pos          (8)                                               /*!< SPI_T::CNTRL2: NOSLVSEL Position          */
#define SPI_CNTRL2_NOSLVSEL_Msk          (0x1ul << SPI_CNTRL2_NOSLVSEL_Pos)                /*!< SPI_T::CNTRL2: NOSLVSEL Mask              */

#define SPI_CNTRL2_SLV_ABORT_Pos         (9)                                               /*!< SPI_T::CNTRL2: SLV_ABORT Position         */
#define SPI_CNTRL2_SLV_ABORT_Msk         (0x1ul << SPI_CNTRL2_SLV_ABORT_Pos)               /*!< SPI_T::CNTRL2: SLV_ABORT Mask             */

#define SPI_CNTRL2_SSTA_INTEN_Pos        (10)                                              /*!< SPI_T::CNTRL2: SSTA_INTEN Position        */
#define SPI_CNTRL2_SSTA_INTEN_Msk        (0x1ul << SPI_CNTRL2_SSTA_INTEN_Pos)              /*!< SPI_T::CNTRL2: SSTA_INTEN Mask            */

#define SPI_CNTRL2_SLV_START_INTSTS_Pos  (11)                                              /*!< SPI_T::CNTRL2: SLV_START_INTSTS Position  */
#define SPI_CNTRL2_SLV_START_INTSTS_Msk  (0x1ul << SPI_CNTRL2_SLV_START_INTSTS_Pos)        /*!< SPI_T::CNTRL2: SLV_START_INTSTS Mask      */

#define SPI_CNTRL2_SS_INT_OPT_Pos        (16)                                              /*!< SPI_T::CNTRL2: SS_INT_OPT Position        */
#define SPI_CNTRL2_SS_INT_OPT_Msk        (0x1ul << SPI_CNTRL2_SS_INT_OPT_Pos)              /*!< SPI_T::CNTRL2: SS_INT_OPT Mask            */

#define SPI_CNTRL2_BCn_Pos               (31)                                              /*!< SPI_T::CNTRL2: BCn Position               */
#define SPI_CNTRL2_BCn_Msk               (0x1ul << SPI_CNTRL2_BCn_Pos)                     /*!< SPI_T::CNTRL2: BCn Mask                   */

#define SPI_FIFO_CTL_RX_CLR_Pos          (0)                                               /*!< SPI_T::FIFO_CTL: RX_CLR Position          */
#define SPI_FIFO_CTL_RX_CLR_Msk          (0x1ul << SPI_FIFO_CTL_RX_CLR_Pos)                /*!< SPI_T::FIFO_CTL: RX_CLR Mask              */

#define SPI_FIFO_CTL_TX_CLR_Pos          (1)                                               /*!< SPI_T::FIFO_CTL: TX_CLR Position          */
#define SPI_FIFO_CTL_TX_CLR_Msk          (0x1ul << SPI_FIFO_CTL_TX_CLR_Pos)                /*!< SPI_T::FIFO_CTL: TX_CLR Mask              */

#define SPI_FIFO_CTL_RX_INTEN_Pos        (2)                                               /*!< SPI_T::FIFO_CTL: RX_INTEN Position        */
#define SPI_FIFO_CTL_RX_INTEN_Msk        (0x1ul << SPI_FIFO_CTL_RX_INTEN_Pos)              /*!< SPI_T::FIFO_CTL: RX_INTEN Mask            */

#define SPI_FIFO_CTL_TX_INTEN_Pos        (3)                                               /*!< SPI_T::FIFO_CTL: TX_INTEN Position        */
#define SPI_FIFO_CTL_TX_INTEN_Msk        (0x1ul << SPI_FIFO_CTL_TX_INTEN_Pos)              /*!< SPI_T::FIFO_CTL: TX_INTEN Mask            */

#define SPI_FIFO_CTL_RXOV_INTEN_Pos      (6)                                               /*!< SPI_T::FIFO_CTL: RXOV_INTEN Position      */
#define SPI_FIFO_CTL_RXOV_INTEN_Msk      (0x1ul << SPI_FIFO_CTL_RXOV_INTEN_Pos)            /*!< SPI_T::FIFO_CTL: RXOV_INTEN Mask          */

#define SPI_FIFO_CTL_TIMEOUT_INTEN_Pos   (21)                                              /*!< SPI_T::FIFO_CTL: TIMEOUT_INTEN Position   */
#define SPI_FIFO_CTL_TIMEOUT_INTEN_Msk   (0x1ul << SPI_FIFO_CTL_TIMEOUT_INTEN_Pos)         /*!< SPI_T::FIFO_CTL: TIMEOUT_INTEN Mask       */

#define SPI_FIFO_CTL_RX_THRESHOLD_Pos    (24)                                              /*!< SPI_T::FIFO_CTL: RX_THRESHOLD Position    */
#define SPI_FIFO_CTL_RX_THRESHOLD_Msk    (0x3ul << SPI_FIFO_CTL_RX_THRESHOLD_Pos)          /*!< SPI_T::FIFO_CTL: RX_THRESHOLD Mask        */

#define SPI_FIFO_CTL_TX_THRESHOLD_Pos    (28)                                              /*!< SPI_T::FIFO_CTL: TX_THRESHOLD Position    */
#define SPI_FIFO_CTL_TX_THRESHOLD_Msk    (0x3ul << SPI_FIFO_CTL_TX_THRESHOLD_Pos)          /*!< SPI_T::FIFO_CTL: TX_THRESHOLD Mask        */

#define SPI_STATUS_RX_INTSTS_Pos         (0)                                               /*!< SPI_T::STATUS: RX_INTSTS Position         */
#define SPI_STATUS_RX_INTSTS_Msk         (0x1ul << SPI_STATUS_RX_INTSTS_Pos)               /*!< SPI_T::STATUS: RX_INTSTS Mask             */

#define SPI_STATUS_RX_OVERRUN_Pos        (2)                                               /*!< SPI_T::STATUS: RX_OVERRUN Position        */
#define SPI_STATUS_RX_OVERRUN_Msk        (0x1ul << SPI_STATUS_RX_OVERRUN_Pos)              /*!< SPI_T::STATUS: RX_OVERRUN Mask            */

#define SPI_STATUS_TX_INTSTS_Pos         (4)                                               /*!< SPI_T::STATUS: TX_INTSTS Position         */
#define SPI_STATUS_TX_INTSTS_Msk         (0x1ul << SPI_STATUS_TX_INTSTS_Pos)               /*!< SPI_T::STATUS: TX_INTSTS Mask             */

#define SPI_STATUS_SLV_START_INTSTS_Pos  (11)                                              /*!< SPI_T::STATUS: SLV_START_INTSTS Position  */
#define SPI_STATUS_SLV_START_INTSTS_Msk  (0x1ul << SPI_STATUS_SLV_START_INTSTS_Pos)        /*!< SPI_T::STATUS: SLV_START_INTSTS Mask      */

#define SPI_STATUS_RX_FIFO_COUNT_Pos     (12)                                              /*!< SPI_T::STATUS: RX_FIFO_COUNT Position     */
#define SPI_STATUS_RX_FIFO_COUNT_Msk     (0xful << SPI_STATUS_RX_FIFO_COUNT_Pos)           /*!< SPI_T::STATUS: RX_FIFO_COUNT Mask         */

#define SPI_STATUS_IF_Pos                (16)                                              /*!< SPI_T::STATUS: IF Position                */
#define SPI_STATUS_IF_Msk                (0x1ul << SPI_STATUS_IF_Pos)                      /*!< SPI_T::STATUS: IF Mask                    */

#define SPI_STATUS_TIMEOUT_Pos           (20)                                              /*!< SPI_T::STATUS: TIMEOUT Position           */
#define SPI_STATUS_TIMEOUT_Msk           (0x1ul << SPI_STATUS_TIMEOUT_Pos)                 /*!< SPI_T::STATUS: TIMEOUT Mask               */

#define SPI_STATUS_RX_EMPTY_Pos          (24)                                              /*!< SPI_T::STATUS: RX_EMPTY Position          */
#define SPI_STATUS_RX_EMPTY_Msk          (0x1ul << SPI_STATUS_RX_EMPTY_Pos)                /*!< SPI_T::STATUS: RX_EMPTY Mask              */

#define SPI_STATUS_RX_FULL_Pos           (25)                                              /*!< SPI_T::STATUS: RX_FULL Position           */
#define SPI_STATUS_RX_FULL_Msk           (0x1ul << SPI_STATUS_RX_FULL_Pos)                 /*!< SPI_T::STATUS: RX_FULL Mask               */

#define SPI_STATUS_TX_EMPTY_Pos          (26)                                              /*!< SPI_T::STATUS: TX_EMPTY Position          */
#define SPI_STATUS_TX_EMPTY_Msk          (0x1ul << SPI_STATUS_TX_EMPTY_Pos)                /*!< SPI_T::STATUS: TX_EMPTY Mask              */

#define SPI_STATUS_TX_FULL_Pos           (27)                                              /*!< SPI_T::STATUS: TX_FULL Position           */
#define SPI_STATUS_TX_FULL_Msk           (0x1ul << SPI_STATUS_TX_FULL_Pos)                 /*!< SPI_T::STATUS: TX_FULL Mask               */

#define SPI_STATUS_TX_FIFO_COUNT_Pos     (28)                                              /*!< SPI_T::STATUS: TX_FIFO_COUNT Position     */
#define SPI_STATUS_TX_FIFO_COUNT_Msk     (0xful << SPI_STATUS_TX_FIFO_COUNT_Pos)           /*!< SPI_T::STATUS: TX_FIFO_COUNT Mask         */

/**@}*/ /* SPI_CONST */
/**@}*/ /* end of SPI register group */


/*---------------------- Timer Controller -------------------------*/
/**
    @addtogroup TMR Timer Controller(TMR)
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
     * |        |          |When TDR_EN is set, TDR (Timer Data Register) will be updated continuously with the 24-bit up-timer value as the timer is counting.
     * |        |          |0 = Timer Data Register update Disabled.
     * |        |          |1 = Timer Data Register update Enabled while Timer counter is active.
     * |[17]    |PERIODIC_SEL|Periodic Mode Behavior Selection
     * |        |          |0 = In One-shot or Periodic mode, when write new TCMP, timer counter will reset.
     * |        |          |1 = In One-shot or Periodic mode, when write new TCMP if new TCMP > TDR(current counter) , timer counter keep counting and will not reset.
     * |        |          |If new TCMP <= TDR(current counter) , timer counter will reset.
     * |[18]    |TOUT_PIN  |Toggle Out Pin Selection
     * |        |          |When Timer is set to toggle mode,
     * |        |          |0 = Time0/1 toggle output pin is T0/T1 pin.
     * |        |          |1 = Time0/1 toggle output pin is T0EX/T1EX pin.
     * |[19]    |CAP_SRC   |Capture Pin Source Selection
     * |        |          |0 = Capture Function source is from TxEX pin.
     * |        |          |1 = Capture Function source is from ACMPx output signal.
     * |[23]    |WAKE_EN   |Wake-up Enable Control
     * |        |          |When WAKE_EN (UA_IER[6]) is set and the TIF or TEXIF (TEXISR[0]) is set, the timer controller will generator a wake-up trigger event to CPU.
     * |        |          |0 = Wake-up trigger event Disabled.
     * |        |          |1 = Wake-up trigger event Enabled.
     * |[24]    |CTB       |Counter Mode Enable Control
     * |        |          |This bit is for external counting pin function enabled.
     * |        |          |When timer is used as an event counter, this bit should be set to 1 and select HCLK as timer clock source.
     * |        |          |Please refer to section 6.12.5.3 for detail description.
     * |        |          |0 = External event counter mode Disabled.
     * |        |          |1 = External event counter mode Enabled.
     * |[25]    |CACT      |Timer Active Status (Read Only)
     * |        |          |This bit indicates the 24-bit up counter status.
     * |        |          |0 = 24-bit up counter is not active.
     * |        |          |1 = 24-bit up counter is active.
     * |[26]    |CRST      |Timer Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset 8-bit prescale counter, 24-bit up counter value and CEN bit if CACT is 1.
     * |[28:27] |MODE      |Timer Operating Mode
     * |        |          |00 = The timer is operating in the One-shot mode.
     * |        |          |The associated interrupt signal is generated once (if IE is enabled) and CEN is automatically cleared by hardware.
     * |        |          |01 = The timer is operating in Periodic mode.
     * |        |          |The associated interrupt signal is generated periodically (if IE is enabled).
     * |        |          |10 = The timer is operating in Toggle mode.
     * |        |          |The interrupt signal is generated periodically (if   IE is enabled).
     * |        |          |The associated signal (tout) is changing back and forth with 50% duty cycle.
     * |        |          |11 = The timer is operating in Continuous Counting mode.
     * |        |          |The associated interrupt signal is generated when TDR = TCMPR (if IE is enabled).
     * |        |          |However, the 24-bit up-timer counts continuously.
     * |        |          |Please refer to 6.12.5.2 for detailed description about Continuous Counting mode operation.
     * |[29]    |IE        |Interrupt Enable Control
     * |        |          |0 = Timer Interrupt function Disabled.
     * |        |          |1 = Timer Interrupt function Enabled.
     * |        |          |If this bit is enabled, when the timer interrupt flag (TIF) is set to 1, the timer interrupt signal is generated and inform to CPU.
     * |[30]    |CEN       |Timer Enable Control
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note1: In stop status, and then set CEN to 1 will enable the 24-bit up counter to keep  counting from the last stop counting value.
     * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (MODE (TCSRx[28:27]) = 00) when the timer interrupt flag (TIF) is generated.
     * |[31]    |DBGACK_TMR|ICE Debug Mode Acknowledge Disable Control (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects TIMER counting.
     * |        |          |Timer counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |Timer counter will keep going no matter CPU is held by ICE or not.
     * @var TIMER_T::TCMPR
     * Offset: 0x04  Timer Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |TCMP      |Timer Compared Value
     * |        |          |TCMP is a 24-bit compared value register.
     * |        |          |When the internal 24-bit up counter value is equal to TCMP value, the TIF flag will set to 1.
     * |        |          |Time-out period = (Period of Timer clock source) * (8-bit PRESCALE + 1) * (24-bit TCMP).
     * |        |          |Note1: Never write 0x0 or 0x1 in TCMP field, or the core will run into unknown state.
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
     * |[1]     |TWF       |Timer Wake-up Flag
     * |        |          |This bit indicates the interrupt wake-up flag status of Time.
     * |        |          |0 = Timer does not cause chip wake-up.
     * |        |          |1 = Chip wake-up from Idle or Power-down mode if Timer time-out interrupt signal generated.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * @var TIMER_T::TDR
     * Offset: 0x0C  Timer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |TDR       |Timer Data Register
     * |        |          |If TDR_EN (TCSRx[16]) is set to 1, TDR register value will be updated continuously to monitor 24-bit up counter value.
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
     * |        |          |This bit indicates the detection phase of Tx (x = 0~1) pin.
     * |        |          |0 = A falling edge of Tx (x = 0~1) pin will be counted.
     * |        |          |1 = A rising edge of Tx (x = 0~1) pin will be counted.
     * |[2:1]   |TEX_EDGE  |Timer External Pin Edge Detection
     * |        |          |00 = A 1 to 0 transition on TxEX (x = 0~1) will be detected.
     * |        |          |01 = A 0 to 1 transition on TxEX (x = 0~1) will be detected.
     * |        |          |10 = Either 1 to 0 or 0 to 1 transition on TxEX (x = 0~1) will be detected.
     * |        |          |11 = Reserved.
     * |[3]     |TEXEN     |Timer External Pin Function Enable Control
     * |        |          |This bit enables the RSTCAPSEL function on the TxEX (x = 0~1) pin.
     * |        |          |0 = RSTCAPSEL function of TxEX (x = 0~1) pin will be ignored.
     * |        |          |1 = RSTCAPSEL function of TxEX (x = 0~1) pin is active.
     * |[4]     |RSTCAPSEL |Timer External Reset Counter / Timer External Capture Mode Selection
     * |        |          |0 = Transition on TxEX (x = 0~1) pin is using to save the TDR value into TCAP value if TEXIF flag is set to 1.
     * |        |          |1 = Transition on TxEX (x = 0~1) pin is using to reset the 24-bit up counter.
     * |[5]     |TEXIEN    |Timer External Capture Interrupt Enable Control
     * |        |          |0 = TxEX (x = 0~1) pin detection Interrupt Disabled.
     * |        |          |1 = TxEX (x = 0~1) pin detection Interrupt Enabled.
     * |        |          |If TEXIEN enabled, Timer will raise an external capture interrupt signal and inform to CPU while TEXIF flag is set to 1.
     * |[6]     |TEXDB     |Timer External Capture Input Pin De-bounce Enable Control
     * |        |          |0 = TxEX (x = 0~1) pin de-bounce Disabled.
     * |        |          |1 = TxEX (x = 0~1) pin de-bounce Enabled.
     * |        |          |If this bit is enabled, the edge detection of TxEX (x = 0~1) pin is detected with de-bounce circuit.
     * |[7]     |TCDB      |Timer External Counter Input Pin De-bounce Enable Control
     * |        |          |0 = Tx (x = 0~1) pin de-bounce Disabled.
     * |        |          |1 = Tx (x = 0~1) pin de-bounce Enabled.
     * |        |          |If this bit is enabled, the edge detection of Tx (x = 0~1) pin is detected with de-bounce circuit.
     * |[8]     |CAP_MODE  |Capture Mode Selection
     * |        |          |0 = Timer counter reset function or free-counting mode of timer capture function.
     * |        |          |1 = Trigger-counting mode of timer capture function.
     * @var TIMER_T::TEXISR
     * Offset: 0x18  Timer External Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TEXIF     |Timer External Interrupt Flag
     * |        |          |This bit indicates the external capture interrupt flag status
     * |        |          |When TEXEN enabled, TxEX (x = 0, 1) pin selected as external capture function, and a transition on TxEX (x = 0, 1) pin matched the TEX_EDGE setting, this flag will set to 1 by hardware.
     * |        |          |0 = TxEX (x = 0, 1) pin interrupt did not occur.
     * |        |          |1 = TxEX (x = 0, 1) pin interrupt occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it
     */

    __IO uint32_t TCSR;         /* Offset: 0x00  Timer Control and Status Register                                 */
    __IO uint32_t TCMPR;        /* Offset: 0x04  Timer Compare Register                                            */
    __IO uint32_t TISR;         /* Offset: 0x08  Timer Interrupt Status Register                                   */
    __I  uint32_t TDR;          /* Offset: 0x0C  Timer Data Register                                               */
    __I  uint32_t TCAP;         /* Offset: 0x10  Timer Capture Data Register                                       */
    __IO uint32_t TEXCON;       /* Offset: 0x14  Timer External Control Register                                   */
    __IO uint32_t TEXISR;       /* Offset: 0x18  Timer External Interrupt Status Register                          */
} TIMER_T;



/**
    @addtogroup TMR_CONST TMR Bit Field Definition
    Constant Definitions for TMR Controller
@{ */

#define TIMER_TCSR_PRESCALE_Pos           (0)                                               /*!< TIMER_T::TCSR: PRESCALE Position           */
#define TIMER_TCSR_PRESCALE_Msk           (0xfful << TIMER_TCSR_PRESCALE_Pos)               /*!< TIMER_T::TCSR: PRESCALE Mask               */

#define TIMER_TCSR_TDR_EN_Pos             (16)                                              /*!< TIMER_T::TCSR: TDR_EN Position             */
#define TIMER_TCSR_TDR_EN_Msk             (0x1ul << TIMER_TCSR_TDR_EN_Pos)                  /*!< TIMER_T::TCSR: TDR_EN Mask                 */

#define TIMER_TCSR_PERIODIC_SEL_Pos       (17)                                              /*!< TIMER_T::TCSR: PERIODIC_SEL Position       */
#define TIMER_TCSR_PERIODIC_SEL_Msk       (0x1ul << TIMER_TCSR_PERIODIC_SEL_Pos)            /*!< TIMER_T::TCSR: PERIODIC_SEL Mask           */

#define TIMER_TCSR_TOGGLE_PIN_Pos         (18)                                              /*!< TIMER TCSR: TOGGLE_PIN Position            */
#define TIMER_TCSR_TOGGLE_PIN_Msk         (0x1ul << TIMER_TCSR_TOGGLE_PIN_Pos)              /*!< TIMER TCSR: TOGGLE_PIN Mask                */

#define TIMER_TCSR_TOUT_PIN_Pos           (18)                                              /*!< TIMER_T::TCSR: TOUT_PIN Position           */
#define TIMER_TCSR_TOUT_PIN_Msk           (0x1ul << TIMER_TCSR_TOUT_PIN_Pos)                /*!< TIMER_T::TCSR: TOUT_PIN Mask               */

#define TIMER_TCSR_CAP_SRC_Pos            (19)                                              /*!< TIMER_T::TCSR: CAP_SRC Position            */
#define TIMER_TCSR_CAP_SRC_Msk            (0x1ul << TIMER_TCSR_CAP_SRC_Pos)                 /*!< TIMER_T::TCSR: CAP_SRC Mask                */

#define TIMER_TCSR_WAKE_EN_Pos            (23)                                              /*!< TIMER_T::TCSR: WAKE_EN Position            */
#define TIMER_TCSR_WAKE_EN_Msk            (0x1ul << TIMER_TCSR_WAKE_EN_Pos)                 /*!< TIMER_T::TCSR: WAKE_EN Mask                */

#define TIMER_TCSR_CTB_Pos                (24)                                              /*!< TIMER_T::TCSR: CTB Position                */
#define TIMER_TCSR_CTB_Msk                (0x1ul << TIMER_TCSR_CTB_Pos)                     /*!< TIMER_T::TCSR: CTB Mask                    */

#define TIMER_TCSR_CACT_Pos               (25)                                              /*!< TIMER_T::TCSR: CACT Position               */
#define TIMER_TCSR_CACT_Msk               (0x1ul << TIMER_TCSR_CACT_Pos)                    /*!< TIMER_T::TCSR: CACT Mask                   */

#define TIMER_TCSR_CRST_Pos               (26)                                              /*!< TIMER_T::TCSR: CRST Position               */
#define TIMER_TCSR_CRST_Msk               (0x1ul << TIMER_TCSR_CRST_Pos)                    /*!< TIMER_T::TCSR: CRST Mask                   */

#define TIMER_TCSR_MODE_Pos               (27)                                              /*!< TIMER_T::TCSR: MODE Position               */
#define TIMER_TCSR_MODE_Msk               (0x3ul << TIMER_TCSR_MODE_Pos)                    /*!< TIMER_T::TCSR: MODE Mask                   */

#define TIMER_TCSR_IE_Pos                 (29)                                              /*!< TIMER_T::TCSR: IE Position                 */
#define TIMER_TCSR_IE_Msk                 (0x1ul << TIMER_TCSR_IE_Pos)                      /*!< TIMER_T::TCSR: IE Mask                     */

#define TIMER_TCSR_CEN_Pos                (30)                                              /*!< TIMER_T::TCSR: CEN Position                */
#define TIMER_TCSR_CEN_Msk                (0x1ul << TIMER_TCSR_CEN_Pos)                     /*!< TIMER_T::TCSR: CEN Mask                    */

#define TIMER_TCSR_DBGACK_TMR_Pos         (31)                                              /*!< TIMER_T::TCSR: DBGACK_TMR Position         */
#define TIMER_TCSR_DBGACK_TMR_Msk         (0x1ul << TIMER_TCSR_DBGACK_TMR_Pos)              /*!< TIMER_T::TCSR: DBGACK_TMR Mask             */

#define TIMER_TCMP_TCMP_Pos               (0)                                               /*!< TIMER_T::TCMPR: TCMP Position              */
#define TIMER_TCMP_TCMP_Msk               (0xfffffful << TIMER_TCMP_TCMP_Pos)               /*!< TIMER_T::TCMPR: TCMP Mask                  */

#define TIMER_TISR_TIF_Pos                (0)                                               /*!< TIMER_T::TISR: TIF Position                */
#define TIMER_TISR_TIF_Msk                (0x1ul << TIMER_TISR_TIF_Pos)                     /*!< TIMER_T::TISR: TIF Mask                    */

#define TIMER_TISR_TWF_Pos                (1)                                               /*!< TIMER_T::TISR: TWF Position                */
#define TIMER_TISR_TWF_Msk                (0x1ul << TIMER_TISR_TWF_Pos)                     /*!< TIMER_T::TISR: TWF Mask                    */

#define TIMER_TDR_TDR_Pos                 (0)                                               /*!< TIMER_T::TDR: TDR Position                 */
#define TIMER_TDR_TDR_Msk                 (0xfffffful << TIMER_TDR_TDR_Pos)                 /*!< TIMER_T::TDR: TDR Mask                     */

#define TIMER_TCAP_TCAP_Pos               (0)                                               /*!< TIMER_T::TCAP: TCAP Position               */
#define TIMER_TCAP_TCAP_Msk               (0xfffffful << TIMER_TCAP_TCAP_Pos)               /*!< TIMER_T::TCAP: TCAP Mask                   */

#define TIMER_TEXCON_TX_PHASE_Pos         (0)                                               /*!< TIMER_T::TEXCON: TX_PHASE Position         */
#define TIMER_TEXCON_TX_PHASE_Msk         (0x1ul << TIMER_TEXCON_TX_PHASE_Pos)              /*!< TIMER_T::TEXCON: TX_PHASE Mask             */

#define TIMER_TEXCON_TEX_EDGE_Pos         (1)                                               /*!< TIMER_T::TEXCON: TEX_EDGE Position         */
#define TIMER_TEXCON_TEX_EDGE_Msk         (0x3ul << TIMER_TEXCON_TEX_EDGE_Pos)              /*!< TIMER_T::TEXCON: TEX_EDGE Mask             */

#define TIMER_TEXCON_TEXEN_Pos            (3)                                               /*!< TIMER_T::TEXCON: TEXEN Position            */
#define TIMER_TEXCON_TEXEN_Msk            (0x1ul << TIMER_TEXCON_TEXEN_Pos)                 /*!< TIMER_T::TEXCON: TEXEN Mask                */

#define TIMER_TEXCON_RSTCAPSEL_Pos        (4)                                               /*!< TIMER_T::TEXCON: RSTCAPSEL Position        */
#define TIMER_TEXCON_RSTCAPSEL_Msk        (0x1ul << TIMER_TEXCON_RSTCAPSEL_Pos)             /*!< TIMER_T::TEXCON: RSTCAPSEL Mask            */

#define TIMER_TEXCON_TEXIEN_Pos           (5)                                               /*!< TIMER_T::TEXCON: TEXIEN Position           */
#define TIMER_TEXCON_TEXIEN_Msk           (0x1ul << TIMER_TEXCON_TEXIEN_Pos)                /*!< TIMER_T::TEXCON: TEXIEN Mask               */

#define TIMER_TEXCON_TEXDB_Pos            (6)                                               /*!< TIMER_T::TEXCON: TEXDB Position            */
#define TIMER_TEXCON_TEXDB_Msk            (0x1ul << TIMER_TEXCON_TEXDB_Pos)                 /*!< TIMER_T::TEXCON: TEXDB Mask                */

#define TIMER_TEXCON_TCDB_Pos             (7)                                               /*!< TIMER_T::TEXCON: TCDB Position             */
#define TIMER_TEXCON_TCDB_Msk             (0x1ul << TIMER_TEXCON_TCDB_Pos)                  /*!< TIMER_T::TEXCON: TCDB Mask                 */

#define TIMER_TEXCON_CAP_MODE_Pos         (8)                                               /*!< TIMER_T::TEXCON: CAP_MODE Position         */
#define TIMER_TEXCON_CAP_MODE_Msk         (0x1ul << TIMER_TEXCON_CAP_MODE_Pos)              /*!< TIMER_T::TEXCON: CAP_MODE Mask             */

#define TIMER_TEXISR_TEXIF_Pos            (0)                                               /*!< TIMER_T::TEXISR: TEXIF Position            */
#define TIMER_TEXISR_TEXIF_Msk            (0x1ul << TIMER_TEXISR_TEXIF_Pos)                 /*!< TIMER_T::TEXISR: TEXIF Mask                */

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
     * @var UART_T::RBR
     * Offset: 0x00  UART Receive Buffer Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |RBR       |Receive Buffer Bits (Read Only)
     * |        |          |By reading this register, the UART Controller will return an 8-bit data received from RX pin (LSB first).
     * @var UART_T::THR
     * Offset: 0x00  UART Transmit Holding Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |THR       |Transmit Holding Bits
     * |        |          |By writing to this register, the UART sends out an 8-bit data through the TX pin (LSB first).
     * @var UART_T::IER
     * Offset: 0x04  UART Interrupt Enable Control Register
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
     * |        |          |1 = RLS_INT Enabled.
     * |[3]     |MODEM_IEN |Modem Status Interrupt Enable Control
     * |        |          |0 = MODEM_INT Masked off.
     * |        |          |1 = MODEM_INT Enabled.
     * |[4]     |RTO_IEN   |RX Time-out Interrupt Enable Control
     * |        |          |0 = TOUT_INT Masked off.
     * |        |          |1 = TOUT_INT Enabled.
     * |[5]     |BUF_ERR_IEN|Buffer Error Interrupt Enable Control
     * |        |          |0 = INT_BUF_ERR Masked Disabled.
     * |        |          |1 = INT_BUF_ERR Enabled.
     * |[6]     |WAKE_EN   |Wake-up CPU Function Enable Control
     * |        |          |0 = UART wake-up function Disabled.
     * |        |          |1 = UART Wake-up function Enabled.
     * |        |          |Note: when the chip is in Power-down mode, an external CTS change will wake-up chip from Power-down mode.
     * |[11]    |TIME_OUT_EN|Time-out Counter Enable Control
     * |        |          |0 = Time-out counter Disabled.
     * |        |          |1 = Time-out counter Enabled.
     * |[12]    |AUTO_RTS_EN|RTS Auto Flow Control Enable Control
     * |        |          |0 = RTS auto flow control Disabled.
     * |        |          |1 = RTS auto flow control Enabled.
     * |        |          |Note: When RTS auto-flow is enabled, if the number of bytes in the RX FIFO equals the RTS_TRI_LEV (UA_FCR [19:16]), the UART will de-assert RTS signal.
     * |[13]    |AUTO_CTS_EN|CTS Auto Flow Control Enable Control
     * |        |          |0 = CTS auto flow control Disabled.
     * |        |          |1 = CTS auto flow control Enabled.
     * |        |          |Note: When CTS auto-flow is enabled, the UART will send data to external device when CTS input assert (UART will not send data to device until CTS is asserted).
     * @var UART_T::FCR
     * Offset: 0x08  UART FIFO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RFR       |RX Field Software Reset
     * |        |          |When RX_RST is set, all the byte in the receiver FIFO and RX internal state machine are cleared.
     * |        |          |0 = No effect.
     * |        |          |1 = The RX internal state machine and pointers reset.
     * |        |          |Note: This bit will auto clear needs at least 3 UART Controller peripheral clock cycles.
     * |[2]     |TFR       |TX Field Software Reset
     * |        |          |When TX_RST is set, all the byte in the transmit FIFO and TX internal state machine are cleared.
     * |        |          |0 = No effect.
     * |        |          |1 = The TX internal state machine and pointers reset.
     * |        |          |Note: This bit will auto clear needs at least 3 UART Controller peripheral clock cycles.
     * |[7:4]   |RFITL     |RX FIFO Interrupt (RDA_INT) Trigger Level
     * |        |          |When the number of bytes in the receive FIFO equals the RFITL then the RDA_IF will be set (if RDA_IEN in UA_IER register is enable, an interrupt will generated).
     * |        |          |0000 = RX FIFO Interrupt Trigger Level is 1 byte.
     * |        |          |0001 = RX FIFO Interrupt Trigger Level is 4 bytes.
     * |        |          |0010 = RX FIFO Interrupt Trigger Level is 8 bytes.
     * |        |          |0011 = RX FIFO Interrupt Trigger Level is 14 bytes.
     * |        |          |Other = Reserved.
     * |[8]     |RX_DIS    |Receiver Disable Control
     * |        |          |The receiver is disabled or not (setting 1 to disable the receiver).
     * |        |          |0 = Receiver Enabled.
     * |        |          |1 = Receiver Disabled.
     * |        |          |Note1: This field is only used for RS-485 Normal Multi-drop mode.
     * |        |          |It should be programmed firstly to avoid receiving unknown data before RS-485_NMM (UA_ALT_CSR [8]) is programmed.
     * |        |          |Note2: After RS-485 receives an address byte in RS-485 Normal Multi-drop mode, this bit (RX_DIS) will be cleared to "0" by hardware.
     * |[19:16] |RTS_TRI_LEV|RTS Trigger Level (For Auto-flow Control Use)
     * |        |          |0000 = RTS Trigger Level is 1 byte.
     * |        |          |0001 = RTS Trigger Level is 4 bytes.
     * |        |          |0010 = RTS Trigger Level is 8 bytes.
     * |        |          |0011 = RTS Trigger Level is 14 bytes.
     * |        |          |Other = Reserved.
     * |        |          |Note: This field is used for RTS auto-flow control.
     * @var UART_T::LCR
     * Offset: 0x0C  UART Line Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WLS       |Word Length Selection
     * |        |          |00 = Word length is 5-bit.
     * |        |          |01 = Word length is 6-bit.
     * |        |          |10 = Word length is 7-bit.
     * |        |          |11 = Word length is 8-bit.
     * |[2]     |NSB       |Number Of "STOP Bit"
     * |        |          |0 = One "STOP bit" is generated in the transmitted data.
     * |        |          |1 = When select 5-bit word length, 1.5 "STOP bit" is generated in the transmitted data.
     * |        |          |When select 6-, 7- and 8-bit word length, 2 "STOP bit" is generated in the transmitted data.
     * |[3]     |PBE       |Parity Bit Enable Control
     * |        |          |0 = No parity bit.
     * |        |          |1 = Parity bit is generated on each outgoing character and is checked on each incoming data.
     * |[4]     |EPE       |Even Parity Enable Control
     * |        |          |0 = Odd number of logic 1's is transmitted and checked in each word.
     * |        |          |1 = Even number of logic 1's is transmitted and checked in each word.
     * |        |          |This bit has effect only when PBE (UA_LCR[3]) is set.
     * |[5]     |SPE       |Stick Parity Enable Control
     * |        |          |0 = Stick parity Disabled.
     * |        |          |1 = If PBE (UA_LCR[3]) and EBE (UA_LCR[4]) are logic 1, the parity bit is transmitted and checked as logic 0.
     * |        |          |If PBE (UA_LCR[3]) is 1 and EBE (UA_LCR[4]) is 0 then the parity bit is transmitted and checked as 1.
     * |[6]     |BCB       |Break Control Bit
     * |        |          |When this bit is set to logic 1, the serial data output (TX) is forced to the Spacing State (logic 0).
     * |        |          |This bit acts only on TX and has no effect on the transmitter logic.
     * |        |          |0 = Break control Disabled.
     * |        |          |1 = Break control Enabled.
     * @var UART_T::MCR
     * Offset: 0x10  UART Modem Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RTS       |RTS (Request-to-send) Signal Control
     * |        |          |This bit is direct control internal RTS signal active or not, and then drive the RTS pin output with LEV_RTS bit configuration.
     * |        |          |0 = RTS signal is active.
     * |        |          |1 = RTS signal is inactive.
     * |        |          |Note1: This RTS signal control bit is not effective when RTS auto-flow control (AUTO_RTS_EN) is enabled in UART function mode.
     * |        |          |Note2: This RTS signal control bit is not effective when RS-485 auto direction mode (RS485_AUD) is enabled in RS-485 function mode.
     * |[9]     |LEV_RTS   |RTS Pin Active Level
     * |        |          |This bit defines the active level state of RTS pin output.
     * |        |          |0 = RTS pin output is high level active.
     * |        |          |1 = RTS pin output is low level active.
     * |        |          |Note1: Refer to and UART function mode.
     * |        |          |Note2: Refer to and for RS-485 function mode.
     * |[13]    |RTS_ST    |RTS Pin State (Read Only)
     * |        |          |This bit mirror from RTS pin output of voltage logic status.
     * |        |          |0 = RTS pin output is low level voltage logic state.
     * |        |          |1 = RTS pin output is high level voltage logic state.
     * @var UART_T::MSR
     * Offset: 0x14  UART Modem Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DCTSF     |Detect CTS State Change Flag
     * |        |          |This bit is set whenever CTS input has change state, and it will generate Modem interrupt to CPU when MODEM_IEN (UA_IER [3]) is set to 1.
     * |        |          |0 = CTS input has not change state.
     * |        |          |1 = CTS input has change state.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[4]     |CTS_ST    |CTS Pin Status (Read Only)
     * |        |          |This bit mirror from CTS pin input of voltage logic status.
     * |        |          |0 = CTS pin input is low level voltage logic state.
     * |        |          |1 = CTS pin input is high level voltage logic state.
     * |        |          |Note: This bit echoes when UART Controller peripheral clock is enabled, and CTS multi-function port is selected.
     * |[8]     |LEV_CTS   |CTS Pin Active Level
     * |        |          |This bit defines the active level state of CTS pin input.
     * |        |          |0 = CTS pin input is high level active.
     * |        |          |1 = CTS pin input is low level active.
     * |        |          |Note: Refer to
     * @var UART_T::FSR
     * Offset: 0x18  UART FIFO Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RX_OVER_IF|RX Overflow Error Interrupt Flag
     * |        |          |This bit is set when RX FIFO overflow.
     * |        |          |If the number of bytes of received data is greater than RX_FIFO (UA_RBR) size 16 bytes, this bit will be set.
     * |        |          |0 = RX FIFO is not overflow.
     * |        |          |1 = RX FIFO is overflow.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[3]     |RS485_ADD_DETF|RS-485 Address Byte Detection Flag
     * |        |          |This bit is set to 1 while RS485_ADD_EN (UA_ALT_CSR[15]) is set to 1 to enable Address detection mode and receive detect a data with an address bit (bit 9 = 1).
     * |        |          |Note1: This field is used for RS-485 function mode.
     * |        |          |Note2: This bit is cleared by writing 1 to it.
     * |[4]     |PEF       |Parity Error Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "parity bit".
     * |        |          |0 = No parity error is generated.
     * |        |          |1 = Parity error is generated.Note: This bit is read only, but can be cleared by writing '1' to it .
     * |[5]     |FEF       |Framing Error Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "stop bit" (that is, the stop bit follows the last data bit or parity bit is detected as as logic 0).
     * |        |          |0 = No framing error is generated.
     * |        |          |1 = Framing error is generated.
     * |        |          |Note: This bit is read only, but can be cleared by writing '1' to it .
     * |[6]     |BIF       |Break Interrupt Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received data input (RX) is held in the "spacing state" (logic 0) for longer than a full word transmission time (that is, the total time of "start bit" + data bits + parity + stop bits).
     * |        |          |0 = No Break interrupt is generated.
     * |        |          |1 = Break interrupt is generated.
     * |        |          |Note: This bit is read only, but software can write 1 to clear it.
     * |[13:8]  |RX_POINTER|RX FIFO Pointer (Read Only)
     * |        |          |This field indicates the RX FIFO Buffer Pointer.
     * |        |          |When UART receives one byte from external device, RX_POINTER increases one.
     * |        |          |When one byte of RX FIFO is read by CPU, RX_POINTER decreases one.
     * |        |          |The Maximum value shown in RX_POINTER is 15.
     * |        |          |When the using level of RX FIFO Buffer equal to 16, the RX_FULL bit is set to 1 and RX_POINTER will show 0.
     * |        |          |As one byte of RX FIFO is read by CPU, the RX_FULL bit is cleared to 0 and RX_POINTER will show 15.
     * |[14]    |RX_EMPTY  |Receiver FIFO Empty (Read Only)
     * |        |          |This bit initiate RX FIFO empty or not.
     * |        |          |0 = RX FIFO is not empty.
     * |        |          |1 = RX FIFO is empty.
     * |        |          |Note: When the last byte of RX FIFO has been read by CPU, hardware sets this bit high.
     * |        |          |It will be cleared when UART receives any new data.
     * |[15]    |RX_FULL   |Receiver FIFO Full (Read Only)
     * |        |          |This bit initiates RX FIFO full or not.
     * |        |          |0 = RX FIFO is not full.
     * |        |          |1 = RX FIFO is full.
     * |        |          |Note: This bit is set when the number of usage in RX FIFO Buffer is equal to 16, otherwise is cleared by hardware.
     * |[21:16] |TX_POINTER|TX FIFO Pointer (Read Only)
     * |        |          |This field indicates the TX FIFO Buffer Pointer.
     * |        |          |When CPU writes one byte into UA_THR, TX_POINTER increases one.
     * |        |          |When one byte of TX FIFO is transferred to Transmitter Shift Register, TX_POINTER decreases one.
     * |        |          |The Maximum value shown in TX_POINTER is 15.
     * |        |          |When the using level of TX FIFO Buffer equal to 16, the TX_FULL bit is set to 1 and TX_POINTER will show 0.
     * |        |          |As one byte of TX FIFO is transferred to Transmitter Shift Register, the TX_FULL bit is cleared to 0 and TX_POINTER will show 15.
     * |[22]    |TX_EMPTY  |Transmitter FIFO Empty (Read Only)
     * |        |          |This bit indicates TX FIFO is empty or not.
     * |        |          |0 = TX FIFO is not empty.
     * |        |          |1 = TX FIFO is empty.
     * |        |          |Note: When the last byte of TX FIFO has been transferred to Transmitter Shift Register, hardware sets this bit high.
     * |        |          |It will be cleared when writing data into THR (TX FIFO not empty).
     * |[23]    |TX_FULL   |Transmitter FIFO Full (Read Only)
     * |        |          |This bit indicates TX FIFO full or not.
     * |        |          |0 = TX FIFO is not full.
     * |        |          |1 = TX FIFO is full.
     * |        |          |Note: This bit is set when the number of usage in TX FIFO Buffer is equal to 16, otherwise is cleared by hardware.
     * |[24]    |TX_OVER_IF|TX Overflow Error Interrupt Flag
     * |        |          |If TX FIFO (UA_THR) is full, an additional write to UA_THR will cause this bit to  logic 1.
     * |        |          |0 = TX FIFO is not overflow.
     * |        |          |1 = TX FIFO is overflow.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[28]    |TE_FLAG   |Transmitter Empty Flag (Read Only)
     * |        |          |This bit is set by hardware when TX FIFO (UA_THR) is empty and the STOP bit of the last byte has been transmitted.
     * |        |          |0 = TX FIFO is not empty.
     * |        |          |1 = TX FIFO is empty and the STOP bit of the last byte has been transmitted.
     * |        |          |Note: This bit is cleared automatically when TX FIFO is not empty or the last byte transmission has not completed.
     * @var UART_T::ISR
     * Offset: 0x1C  UART Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RDA_IF    |Receive Data Available Interrupt Flag (Read Only)
     * |        |          |When the number of bytes in the RX FIFO equals the RFITL then the RDA_IF will be set.
     * |        |          |If RDA_IEN (UA_IER [0]) is enabled, the RDA interrupt will be generated.
     * |        |          |0 = No RDA interrupt flag is generated.
     * |        |          |1 = RDA interrupt flag is generated.
     * |        |          |Note: This bit is read only and it will be cleared when the number of unread bytes of RX FIFO drops below the threshold level (RFITL).
     * |[1]     |THRE_IF   |Transmit Holding Register Empty Interrupt Flag (Read Only)
     * |        |          |This bit is set when the last data of TX FIFO is transferred to Transmitter Shift Register.
     * |        |          |If THRE_IEN (UA_IER [1]) is enabled, the THRE interrupt will be generated.
     * |        |          |0 = No THRE interrupt flag is generated.
     * |        |          |1 = THRE interrupt flag is generated.
     * |        |          |Note: This bit is read only and it will be cleared when writing data into THR (TX FIFO not empty).
     * |[2]     |RLS_IF    |Receive Line Interrupt Flag (Read Only)
     * |        |          |This bit is set when the RX receive data have parity error, framing error or break error (at least one of 3 bits, BIF, FEF and PEF, is set).
     * |        |          |If RLS_IEN (UA_IER [2]) is enabled, the RLS interrupt will be generated.
     * |        |          |0 = No RLS interrupt flag is generated.
     * |        |          |1 = RLS interrupt flag is generated.
     * |        |          |Note1: In RS-485 function mode, this field is set including "receiver detect and received address byte character (bit 9 = 1) bit".
     * |        |          |At the same time, the bit of RS485_ADD_DETF (UA_FSR[3]) is also set.
     * |        |          |Note2: This bit is read only and reset to 0 when all bits of BIF, FEF, PEF and RS485_ADD_DETF are cleared.
     * |[3]     |MODEM_IF  |MODEM Interrupt Flag (Read Only)
     * |        |          |This bit is set when the CTS pin has state change (DCTSF = 1).
     * |        |          |If UA_IER [MODEM_IEN] is enabled, the Modem interrupt will be generated.
     * |        |          |0 = No Modem interrupt flag is generated.
     * |        |          |1 = Modem interrupt flag is generated.
     * |        |          |Note: This bit is read only and reset to 0 when bit DCTSF is cleared by a write 1 on DCTSF.
     * |[4]     |TOUT_IF   |Time-out Interrupt Flag (Read Only)
     * |        |          |This bit is set when the RX FIFO is not empty and no activities occurred in the RX FIFO and the time-out counter equal to TOIC.
     * |        |          |If RTO_IEN (UA_IER [4]) is enabled, the Tout interrupt will be generated.
     * |        |          |0 = No Time-out interrupt flag is generated.
     * |        |          |1 = Time-out interrupt flag is generated.
     * |        |          |Note: This bit is read only and user can read UA_RBR (RX is in active) to clear it.
     * |[5]     |BUF_ERR_IF|Buffer Error Interrupt Flag (Read Only)
     * |        |          |This bit is set when the TX/RX FIFO overflow flag (TX_OVER_IF or RX_OVER_IF) is set.
     * |        |          |When BUF_ERR_IF is set, the transfer is not correct.
     * |        |          |If BUF_ERR_IEN (UA_IER [5]) is enabled, the buffer error interrupt will be generated.
     * |        |          |0 = No buffer error interrupt flag is generated.
     * |        |          |1 = Buffer error interrupt flag is generated.
     * |        |          |Note: This bit is read only and reset to 0 when all bits of TX_OVER_IF and RX_OVER_IF are cleared.
     * |[8]     |RDA_INT   |Receive Data Available Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RDA_IEN and RDA_IF are both set to 1.
     * |        |          |This bit is set if RDA_IEN and RDA_IF are both set to 1.
     * |        |          |0 = No RDA interrupt is generated.
     * |        |          |1 = RDA interrupt is generated.
     * |[9]     |THRE_INT  |Transmit Holding Register Empty Interrupt Indicator (Read Only)
     * |        |          |This bit is set if THRE_IEN and THRE_IF are both set to 1.
     * |        |          |0 = No THRE interrupt is generated.
     * |        |          |1 = THRE interrupt is generated.
     * |[10]    |RLS_INT   |Receive Line Status Interrupt (Read Only)
     * |        |          |This bit is set if RLS_IEN and RLS_IF are both set to 1.
     * |        |          |0 = No RLS interrupt is generated.
     * |        |          |1 = RLS interrupt is generated.
     * |[11]    |MODEM_INT |MODEM Status Interrupt Indicator (Read Only)
     * |        |          |This bit is set if MODEM_IEN and MODEM_IF are both set to 1.
     * |        |          |0 = No Modem interrupt is generated.
     * |        |          |1 = Modem interrupt is generated.
     * |[12]    |TOUT_INT  |Time-out Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RTO_IEN and TOUT_IF are both set to 1.
     * |        |          |0 = No Time-out interrupt is generated.
     * |        |          |1 = Time-out interrupt is generated.
     * |[13]    |BUF_ERR_INT|Buffer Error Interrupt Indicator (Read Only)
     * |        |          |This bit is set if BUF_ERR_IEN and BUF_ERR_IF are both set to 1.
     * |        |          |0 = No buffer error interrupt is generated.
     * |        |          |1 = buffer error interrupt is generated.
     * @var UART_T::TOR
     * Offset: 0x20  UART Time-out Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TOIC      |Time-out Interrupt Comparator
     * |        |          |The time-out counter resets and starts counting (the counting clock = baud rate) whenever the RX FIFO receives a new data word.
     * |        |          |Once the content of time-out counter (TOUT_CNT) is equal to that of time-out interrupt comparator (TOIC), a receiver time-out interrupt (TOUT_INT) is generated if RTO_IEN (UA_IER [4]).
     * |        |          |A new incoming data word or RX FIFO empty clears TOUT_INT.
     * |        |          |In order to avoid receiver time-out interrupt generation immediately during one character is being received, TOIC value should be set between 40 and 255.
     * |        |          |So, for example, if TOIC is set with 40, the time-out interrupt is generated after four characters are not received when 1 stop bit and no parity check is set for UART transfer.
     * |[15:8]  |DLY       |TX Delay Time Value
     * |        |          |This field is used to program the transfer delay time between the last stop bit and next start bit.
     * @var UART_T::BAUD
     * Offset: 0x24  UART Baud Rate Divisor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |BRD       |Baud Rate Divider
     * |        |          |The field indicates the baud rate divider.
     * |[27:24] |DIVIDER_X |Divider X
     * |        |          |The baud rate divider M = X+1.
     * |[28]    |DIV_X_ONE |Divider X Equal 1
     * |        |          |0 = Divider M = X (the equation of M = X+1, but DIVIDER_X[27:24] must >= 8).
     * |        |          |1 = Divider M = 1 (the equation of M = 1, but BRD [15:0] must >= 8).
     * |        |          |Refer to section "UART Controller Baud Rate Generator" for more information.
     * |[29]    |DIV_X_EN  |Divider X Enable Control
     * |        |          |The BRD = Baud Rate Divider, and the baud rate equation is:  Baud Rate = Clock / [M * (BRD + 2)], The default value of M is 16.
     * |        |          |0 = Divider X Disabled (the equation of M = 16).
     * |        |          |1 = Divider X Enabled (the equation of M = X+1, but DIVIDER_X [27:24] must >= 8).
     * |        |          |Note: When in IrDA mode, this bit must be disabled.
     * @var UART_T::IRCR
     * Offset: 0x28  UART IrDA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TX_SELECT |TX_SELECT
     * |        |          |0 = IrDA receiver Enabled.
     * |        |          |1 = IrDA transmitter Enabled.
     * |[5]     |INV_TX    |INV_TX
     * |        |          |0 = No inversion.
     * |        |          |1 = Inverse TX output signal.
     * |[6]     |INV_RX    |INV_RX
     * |        |          |0 = No inversion.
     * |        |          |1 = Inverse RX input signal.
     * @var UART_T::ALT_CSR
     * Offset: 0x2C  UART Alternate Control/Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |RS485_NMM |RS-485 Normal Multi-drop Operation Mode (NMM) Control
     * |        |          |0 = RS-485 Normal Multi-drop Operation Mode (NMM) Disabled.
     * |        |          |1 = RS-485 Normal Multi-drop Operation Mode (NMM) Enabled.
     * |        |          |Note: This bit cannot be active with RS485_AAD operation mode.
     * |[9]     |RS485_AAD |RS-485 Auto Address Detection Operation Mode (AAD)
     * |        |          |0 = RS-485 Auto Address Detection Operation Mode (AAD) Disabled.
     * |        |          |1 = RS-485 Auto Address Detection Operation Mode (AAD) Enabled.
     * |        |          |Note: This bit cannot be active with RS485_NMM operation mode.
     * |[10]    |RS485_AUD |RS-485 Auto Direction Mode (AUD) Control
     * |        |          |0 = RS-485 Auto Address Detection Operation Mode (AAD) Disabled.
     * |        |          |1 = RS-485 Auto Address Detection Operation Mode (AAD) Enabled.
     * |        |          |Note: This bit cannot be active with RS485_NMM operation mode.
     * |[15]    |RS485_ADD_EN|RS-485 Address Detection Enable Control
     * |        |          |This bit is used to enable RS-485 Address Detection mode.
     * |        |          |0 = RS-485 address detection mode Disabled.
     * |        |          |1 = RS-485 address detection mode Enabled.
     * |        |          |Note: This field is used for RS-485 any operation mode.
     * |[31:24] |ADDR_MATCH|Address Match Value
     * |        |          |This field contains the RS-485 address match values.
     * |        |          |Note: This field is used for RS-485 auto address detection mode.
     * @var UART_T::FUN_SEL
     * Offset: 0x30  UART Function Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |FUN_SEL   |Function Selection
     * |        |          |00 = UART function mode.
     * |        |          |01 = Reserved.
     * |        |          |10 = IrDA function mode.
     * |        |          |11 = RS-485 function mode.
     */

    union
    {
        __I  uint32_t RBR;           /* Offset: 0x00  UART Receive Buffer Register                                       */
        __O  uint32_t THR;           /* Offset: 0x00  UART Transmit Holding Register                                     */
    };
    __IO uint32_t IER;           /* Offset: 0x04  UART Interrupt Enable Control Register                             */
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



/**
    @addtogroup UART_CONST UART Bit Field Definition
    Constant Definitions for UART Controller
@{ */

#define UART_RBR_RBR_Pos                 (0)                                               /*!< UART_T::RBR: RBR Position                 */
#define UART_RBR_RBR_Msk                 (0xfful << UART_RBR_RBR_Pos)                      /*!< UART_T::RBR: RBR Mask                     */

#define UART_THR_THR_Pos                 (0)                                               /*!< UART_T::THR: THR Position                 */
#define UART_THR_THR_Msk                 (0xfful << UART_THR_THR_Pos)                      /*!< UART_T::THR: THR Mask                     */

#define UART_IER_RDA_IEN_Pos             (0)                                               /*!< UART_T::IER: RDA_IEN Position             */
#define UART_IER_RDA_IEN_Msk             (0x1ul << UART_IER_RDA_IEN_Pos)                   /*!< UART_T::IER: RDA_IEN Mask                 */

#define UART_IER_THRE_IEN_Pos            (1)                                               /*!< UART_T::IER: THRE_IEN Position            */
#define UART_IER_THRE_IEN_Msk            (0x1ul << UART_IER_THRE_IEN_Pos)                  /*!< UART_T::IER: THRE_IEN Mask                */

#define UART_IER_RLS_IEN_Pos             (2)                                               /*!< UART_T::IER: RLS_IEN Position             */
#define UART_IER_RLS_IEN_Msk             (0x1ul << UART_IER_RLS_IEN_Pos)                   /*!< UART_T::IER: RLS_IEN Mask                 */

#define UART_IER_MODEM_IEN_Pos           (3)                                               /*!< UART_T::IER: MODEM_IEN Position           */
#define UART_IER_MODEM_IEN_Msk           (0x1ul << UART_IER_MODEM_IEN_Pos)                 /*!< UART_T::IER: MODEM_IEN Mask               */

#define UART_IER_RTO_IEN_Pos             (4)                                               /*!< UART_T::IER: RTO_IEN Position             */
#define UART_IER_RTO_IEN_Msk             (0x1ul << UART_IER_RTO_IEN_Pos)                   /*!< UART_T::IER: RTO_IEN Mask                 */

#define UART_IER_BUF_ERR_IEN_Pos         (5)                                               /*!< UART_T::IER: BUF_ERR_IEN Position         */
#define UART_IER_BUF_ERR_IEN_Msk         (0x1ul << UART_IER_BUF_ERR_IEN_Pos)               /*!< UART_T::IER: BUF_ERR_IEN Mask             */

#define UART_IER_WAKE_EN_Pos             (6)                                               /*!< UART_T::IER: WAKE_EN Position             */
#define UART_IER_WAKE_EN_Msk             (0x1ul << UART_IER_WAKE_EN_Pos)                   /*!< UART_T::IER: WAKE_EN Mask                 */

#define UART_IER_TIME_OUT_EN_Pos         (11)                                              /*!< UART_T::IER: TIME_OUT_EN Position         */
#define UART_IER_TIME_OUT_EN_Msk         (0x1ul << UART_IER_TIME_OUT_EN_Pos)               /*!< UART_T::IER: TIME_OUT_EN Mask             */

#define UART_IER_AUTO_RTS_EN_Pos         (12)                                              /*!< UART_T::IER: AUTO_RTS_EN Position         */
#define UART_IER_AUTO_RTS_EN_Msk         (0x1ul << UART_IER_AUTO_RTS_EN_Pos)               /*!< UART_T::IER: AUTO_RTS_EN Mask             */

#define UART_IER_AUTO_CTS_EN_Pos         (13)                                              /*!< UART_T::IER: AUTO_CTS_EN Position         */
#define UART_IER_AUTO_CTS_EN_Msk         (0x1ul << UART_IER_AUTO_CTS_EN_Pos)               /*!< UART_T::IER: AUTO_CTS_EN Mask             */

#define UART_FCR_RFR_Pos                 (1)                                               /*!< UART_T::FCR: RFR Position                 */
#define UART_FCR_RFR_Msk                 (0x1ul << UART_FCR_RFR_Pos)                       /*!< UART_T::FCR: RFR Mask                     */

#define UART_FCR_TFR_Pos                 (2)                                               /*!< UART_T::FCR: TFR Position                 */
#define UART_FCR_TFR_Msk                 (0x1ul << UART_FCR_TFR_Pos)                       /*!< UART_T::FCR: TFR Mask                     */

#define UART_FCR_RFITL_Pos               (4)                                               /*!< UART_T::FCR: RFITL Position               */
#define UART_FCR_RFITL_Msk               (0xful << UART_FCR_RFITL_Pos)                     /*!< UART_T::FCR: RFITL Mask                   */

#define UART_FCR_RX_DIS_Pos              (8)                                               /*!< UART_T::FCR: RX_DIS Position              */
#define UART_FCR_RX_DIS_Msk              (0x1ul << UART_FCR_RX_DIS_Pos)                    /*!< UART_T::FCR: RX_DIS Mask                  */

#define UART_FCR_RTS_TRI_LEV_Pos         (16)                                              /*!< UART_T::FCR: RTS_TRI_LEV Position         */
#define UART_FCR_RTS_TRI_LEV_Msk         (0xful << UART_FCR_RTS_TRI_LEV_Pos)               /*!< UART_T::FCR: RTS_TRI_LEV Mask             */

#define UART_LCR_WLS_Pos                 (0)                                               /*!< UART_T::LCR: WLS Position                 */
#define UART_LCR_WLS_Msk                 (0x3ul << UART_LCR_WLS_Pos)                       /*!< UART_T::LCR: WLS Mask                     */

#define UART_LCR_NSB_Pos                 (2)                                               /*!< UART_T::LCR: NSB Position                 */
#define UART_LCR_NSB_Msk                 (0x1ul << UART_LCR_NSB_Pos)                       /*!< UART_T::LCR: NSB Mask                     */

#define UART_LCR_PBE_Pos                 (3)                                               /*!< UART_T::LCR: PBE Position                 */
#define UART_LCR_PBE_Msk                 (0x1ul << UART_LCR_PBE_Pos)                       /*!< UART_T::LCR: PBE Mask                     */

#define UART_LCR_EPE_Pos                 (4)                                               /*!< UART_T::LCR: EPE Position                 */
#define UART_LCR_EPE_Msk                 (0x1ul << UART_LCR_EPE_Pos)                       /*!< UART_T::LCR: EPE Mask                     */

#define UART_LCR_SPE_Pos                 (5)                                               /*!< UART_T::LCR: SPE Position                 */
#define UART_LCR_SPE_Msk                 (0x1ul << UART_LCR_SPE_Pos)                       /*!< UART_T::LCR: SPE Mask                     */

#define UART_LCR_BCB_Pos                 (6)                                               /*!< UART_T::LCR: BCB Position                 */
#define UART_LCR_BCB_Msk                 (0x1ul << UART_LCR_BCB_Pos)                       /*!< UART_T::LCR: BCB Mask                     */

#define UART_MCR_RTS_Pos                 (1)                                               /*!< UART_T::MCR: RTS Position                */
#define UART_MCR_RTS_Msk                 (0x1ul << UART_MCR_RTS_Pos)                       /*!< UART_T::MCR: RTS Mask                    */

#define UART_MCR_LEV_RTS_Pos             (9)                                               /*!< UART_T::MCR: LEV_RTS Position             */
#define UART_MCR_LEV_RTS_Msk             (0x1ul << UART_MCR_LEV_RTS_Pos)                   /*!< UART_T::MCR: LEV_RTS Mask                 */

#define UART_MCR_RTS_ST_Pos              (13)                                              /*!< UART_T::MCR: RTS_ST Position              */
#define UART_MCR_RTS_ST_Msk              (0x1ul << UART_MCR_RTS_ST_Pos)                    /*!< UART_T::MCR: RTS_ST Mask                  */

#define UART_MSR_DCTSF_Pos               (0)                                               /*!< UART_T::MSR: DCTSF Position               */
#define UART_MSR_DCTSF_Msk               (0x1ul << UART_MSR_DCTSF_Pos)                     /*!< UART_T::MSR: DCTSF Mask                   */

#define UART_MSR_CTS_ST_Pos              (4)                                               /*!< UART_T::MSR: CTS_ST Position              */
#define UART_MSR_CTS_ST_Msk              (0x1ul << UART_MSR_CTS_ST_Pos)                    /*!< UART_T::MSR: CTS_ST Mask                  */

#define UART_MSR_LEV_CTS_Pos             (8)                                               /*!< UART_T::MSR: LEV_CTS Position             */
#define UART_MSR_LEV_CTS_Msk             (0x1ul << UART_MSR_LEV_CTS_Pos)                   /*!< UART_T::MSR: LEV_CTS Mask                 */

#define UART_FSR_RX_OVER_IF_Pos          (0)                                               /*!< UART_T::FSR: RX_OVER_IF Position          */
#define UART_FSR_RX_OVER_IF_Msk          (0x1ul << UART_FSR_RX_OVER_IF_Pos)                /*!< UART_T::FSR: RX_OVER_IF Mask              */

#define UART_FSR_RS485_ADD_DETF_Pos     (3)                                                /*!< UART_T::FSR: RS485_ADD_DETF Position     */
#define UART_FSR_RS485_ADD_DETF_Msk     (0x1ul << UART_FSR_RS485_ADD_DETF_Pos)             /*!< UART_T::FSR: RS485_ADD_DETF Mask         */

#define UART_FSR_PEF_Pos                 (4)                                               /*!< UART_T::FSR: PEF Position                 */
#define UART_FSR_PEF_Msk                 (0x1ul << UART_FSR_PEF_Pos)                       /*!< UART_T::FSR: PEF Mask                     */

#define UART_FSR_FEF_Pos                 (5)                                               /*!< UART_T::FSR: FEF Position                 */
#define UART_FSR_FEF_Msk                 (0x1ul << UART_FSR_FEF_Pos)                       /*!< UART_T::FSR: FEF Mask                     */

#define UART_FSR_BIF_Pos                 (6)                                               /*!< UART_T::FSR: BIF Position                 */
#define UART_FSR_BIF_Msk                 (0x1ul << UART_FSR_BIF_Pos)                       /*!< UART_T::FSR: BIF Mask                     */

#define UART_FSR_RX_POINTER_Pos          (8)                                               /*!< UART_T::FSR: RX_POINTER Position          */
#define UART_FSR_RX_POINTER_Msk          (0x3ful << UART_FSR_RX_POINTER_Pos)               /*!< UART_T::FSR: RX_POINTER Mask              */

#define UART_FSR_RX_EMPTY_Pos            (14)                                              /*!< UART_T::FSR: RX_EMPTY Position            */
#define UART_FSR_RX_EMPTY_Msk            (0x1ul << UART_FSR_RX_EMPTY_Pos)                  /*!< UART_T::FSR: RX_EMPTY Mask                */

#define UART_FSR_RX_FULL_Pos             (15)                                              /*!< UART_T::FSR: RX_FULL Position             */
#define UART_FSR_RX_FULL_Msk             (0x1ul << UART_FSR_RX_FULL_Pos)                   /*!< UART_T::FSR: RX_FULL Mask                 */

#define UART_FSR_TX_POINTER_Pos          (16)                                              /*!< UART_T::FSR: TX_POINTER Position          */
#define UART_FSR_TX_POINTER_Msk          (0x3ful << UART_FSR_TX_POINTER_Pos)               /*!< UART_T::FSR: TX_POINTER Mask              */

#define UART_FSR_TX_EMPTY_Pos            (22)                                              /*!< UART_T::FSR: TX_EMPTY Position            */
#define UART_FSR_TX_EMPTY_Msk            (0x1ul << UART_FSR_TX_EMPTY_Pos)                  /*!< UART_T::FSR: TX_EMPTY Mask                */

#define UART_FSR_TX_FULL_Pos             (23)                                              /*!< UART_T::FSR: TX_FULL Position             */
#define UART_FSR_TX_FULL_Msk             (0x1ul << UART_FSR_TX_FULL_Pos)                   /*!< UART_T::FSR: TX_FULL Mask                 */

#define UART_FSR_TX_OVER_IF_Pos          (24)                                              /*!< UART_T::FSR: TX_OVER_IF Position          */
#define UART_FSR_TX_OVER_IF_Msk          (0x1ul << UART_FSR_TX_OVER_IF_Pos)                /*!< UART_T::FSR: TX_OVER_IF Mask              */

#define UART_FSR_TE_FLAG_Pos             (28)                                              /*!< UART_T::FSR: TE_FLAG Position             */
#define UART_FSR_TE_FLAG_Msk             (0x1ul << UART_FSR_TE_FLAG_Pos)                   /*!< UART_T::FSR: TE_FLAG Mask                 */

#define UART_ISR_RDA_IF_Pos              (0)                                               /*!< UART_T::ISR: RDA_IF Position              */
#define UART_ISR_RDA_IF_Msk              (0x1ul << UART_ISR_RDA_IF_Pos)                    /*!< UART_T::ISR: RDA_IF Mask                  */

#define UART_ISR_THRE_IF_Pos             (1)                                               /*!< UART_T::ISR: THRE_IF Position             */
#define UART_ISR_THRE_IF_Msk             (0x1ul << UART_ISR_THRE_IF_Pos)                   /*!< UART_T::ISR: THRE_IF Mask                 */

#define UART_ISR_RLS_IF_Pos              (2)                                               /*!< UART_T::ISR: RLS_IF Position              */
#define UART_ISR_RLS_IF_Msk              (0x1ul << UART_ISR_RLS_IF_Pos)                    /*!< UART_T::ISR: RLS_IF Mask                  */

#define UART_ISR_MODEM_IF_Pos            (3)                                               /*!< UART_T::ISR: MODEM_IF Position            */
#define UART_ISR_MODEM_IF_Msk            (0x1ul << UART_ISR_MODEM_IF_Pos)                  /*!< UART_T::ISR: MODEM_IF Mask                */

#define UART_ISR_TOUT_IF_Pos             (4)                                               /*!< UART_T::ISR: TOUT_IF Position             */
#define UART_ISR_TOUT_IF_Msk             (0x1ul << UART_ISR_TOUT_IF_Pos)                   /*!< UART_T::ISR: TOUT_IF Mask                 */

#define UART_ISR_BUF_ERR_IF_Pos          (5)                                               /*!< UART_T::ISR: BUF_ERR_IF Position          */
#define UART_ISR_BUF_ERR_IF_Msk          (0x1ul << UART_ISR_BUF_ERR_IF_Pos)                /*!< UART_T::ISR: BUF_ERR_IF Mask              */

#define UART_ISR_RDA_INT_Pos             (8)                                               /*!< UART_T::ISR: RDA_INT Position             */
#define UART_ISR_RDA_INT_Msk             (0x1ul << UART_ISR_RDA_INT_Pos)                   /*!< UART_T::ISR: RDA_INT Mask                 */

#define UART_ISR_THRE_INT_Pos            (9)                                               /*!< UART_T::ISR: THRE_INT Position            */
#define UART_ISR_THRE_INT_Msk            (0x1ul << UART_ISR_THRE_INT_Pos)                  /*!< UART_T::ISR: THRE_INT Mask                */

#define UART_ISR_RLS_INT_Pos             (10)                                              /*!< UART_T::ISR: RLS_INT Position             */
#define UART_ISR_RLS_INT_Msk             (0x1ul << UART_ISR_RLS_INT_Pos)                   /*!< UART_T::ISR: RLS_INT Mask                 */

#define UART_ISR_MODEM_INT_Pos           (11)                                              /*!< UART_T::ISR: MODEM_INT Position           */
#define UART_ISR_MODEM_INT_Msk           (0x1ul << UART_ISR_MODEM_INT_Pos)                 /*!< UART_T::ISR: MODEM_INT Mask               */

#define UART_ISR_TOUT_INT_Pos            (12)                                              /*!< UART_T::ISR: TOUT_INT Position            */
#define UART_ISR_TOUT_INT_Msk            (0x1ul << UART_ISR_TOUT_INT_Pos)                  /*!< UART_T::ISR: TOUT_INT Mask                */

#define UART_ISR_BUF_ERR_INT_Pos         (13)                                              /*!< UART_T::ISR: BUF_ERR_INT Position         */
#define UART_ISR_BUF_ERR_INT_Msk         (0x1ul << UART_ISR_BUF_ERR_INT_Pos)               /*!< UART_T::ISR: BUF_ERR_INT Mask             */

#define UART_TOR_TOIC_Pos                (0)                                               /*!< UART_T::TOR: TOIC Position                */
#define UART_TOR_TOIC_Msk                (0xfful << UART_TOR_TOIC_Pos)                     /*!< UART_T::TOR: TOIC Mask                    */

#define UART_TOR_DLY_Pos                 (8)                                               /*!< UART_T::TOR: DLY Position                 */
#define UART_TOR_DLY_Msk                 (0xfful << UART_TOR_DLY_Pos)                      /*!< UART_T::TOR: DLY Mask                     */

#define UART_BAUD_BRD_Pos                (0)                                               /*!< UART_T::BAUD: BRD Position                */
#define UART_BAUD_BRD_Msk                (0xfffful << UART_BAUD_BRD_Pos)                   /*!< UART_T::BAUD: BRD Mask                    */

#define UART_BAUD_DIVIDER_X_Pos          (24)                                              /*!< UART_T::BAUD: DIVIDER_X Position          */
#define UART_BAUD_DIVIDER_X_Msk          (0xful << UART_BAUD_DIVIDER_X_Pos)                /*!< UART_T::BAUD: DIVIDER_X Mask              */

#define UART_BAUD_DIV_X_ONE_Pos          (28)                                              /*!< UART_T::BAUD: DIV_X_ONE Position          */
#define UART_BAUD_DIV_X_ONE_Msk          (0x1ul << UART_BAUD_DIV_X_ONE_Pos)                /*!< UART_T::BAUD: DIV_X_ONE Mask              */

#define UART_BAUD_DIV_X_EN_Pos           (29)                                              /*!< UART_T::BAUD: DIV_X_EN Position           */
#define UART_BAUD_DIV_X_EN_Msk           (0x1ul << UART_BAUD_DIV_X_EN_Pos)                 /*!< UART_T::BAUD: DIV_X_EN Mask               */

#define UART_IRCR_TX_SELECT_Pos          (1)                                               /*!< UART_T::IRCR: TX_SELECT Position          */
#define UART_IRCR_TX_SELECT_Msk          (0x1ul << UART_IRCR_TX_SELECT_Pos)                /*!< UART_T::IRCR: TX_SELECT Mask              */

#define UART_IRCR_INV_TX_Pos             (5)                                               /*!< UART_T::IRCR: INV_TX Position             */
#define UART_IRCR_INV_TX_Msk             (0x1ul << UART_IRCR_INV_TX_Pos)                   /*!< UART_T::IRCR: INV_TX Mask                 */

#define UART_IRCR_INV_RX_Pos             (6)                                               /*!< UART_T::IRCR: INV_RX Position             */
#define UART_IRCR_INV_RX_Msk             (0x1ul << UART_IRCR_INV_RX_Pos)                   /*!< UART_T::IRCR: INV_RX Mask                 */

#define UART_ALT_CSR_RS485_NMM_Pos       (8)                                               /*!< UART_T::ALT_CSR: RS485_NMM Position       */
#define UART_ALT_CSR_RS485_NMM_Msk       (0x1ul << UART_ALT_CSR_RS485_NMM_Pos)             /*!< UART_T::ALT_CSR: RS485_NMM Mask           */

#define UART_ALT_CSR_RS485_AAD_Pos       (9)                                               /*!< UART_T::ALT_CSR: RS485_AAD Position       */
#define UART_ALT_CSR_RS485_AAD_Msk       (0x1ul << UART_ALT_CSR_RS485_AAD_Pos)             /*!< UART_T::ALT_CSR: RS485_AAD Mask           */

#define UART_ALT_CSR_RS485_AUD_Pos       (10)                                              /*!< UART_T::ALT_CSR: RS485_AUD Position       */
#define UART_ALT_CSR_RS485_AUD_Msk       (0x1ul << UART_ALT_CSR_RS485_AUD_Pos)             /*!< UART_T::ALT_CSR: RS485_AUD Mask           */

#define UART_ALT_CSR_RS485_ADD_EN_Pos    (15)                                              /*!< UART_T::ALT_CSR: RS485_ADD_EN Position    */
#define UART_ALT_CSR_RS485_ADD_EN_Msk    (0x1ul << UART_ALT_CSR_RS485_ADD_EN_Pos)          /*!< UART_T::ALT_CSR: RS485_ADD_EN Mask        */

#define UART_ALT_CSR_ADDR_MATCH_Pos      (24)                                              /*!< UART_T::ALT_CSR: ADDR_MATCH Position      */
#define UART_ALT_CSR_ADDR_MATCH_Msk      (0xfful << UART_ALT_CSR_ADDR_MATCH_Pos)           /*!< UART_T::ALT_CSR: ADDR_MATCH Mask          */

#define UART_FUN_SEL_FUN_SEL_Pos         (0)                                               /*!< UART_T::FUN_SEL: FUN_SEL Position         */
#define UART_FUN_SEL_FUN_SEL_Msk         (0x3ul << UART_FUN_SEL_FUN_SEL_Pos)               /*!< UART_T::FUN_SEL: FUN_SEL Mask             */

/**@}*/ /* UART_CONST */
/**@}*/ /* end of UART register group */


/*---------------------- Watch Dog Timer Controller -------------------------*/
/**
    @addtogroup WDT Watch Dog Timer Controller(WDT)
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
     * |[1]     |WTRE      |Watchdog Timer Time-out Reset Enable Control (Write Protect)
     * |        |          |Setting this bit will enable the WDT time-out reset function if the WDT up counter value has not been cleared after the specific WDT reset delay period (1024 * TWDT) expires.
     * |        |          |0 = WDT time-out reset function Disabled.
     * |        |          |1 = WDT time-out reset function Enabled.
     * |[2]     |WTRF      |Watchdog Timer Time-out Reset Flag
     * |        |          |This bit indicates the system has been reset by WDT time-out reset or not.
     * |        |          |0 = WDT time-out reset did not occur.
     * |        |          |1 = WDT time-out reset occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[3]     |WTIF      |Watchdog Timer Time-out Interrupt Flag
     * |        |          |This bit will be set to 1 while WDT up counter value reaches the selected WDT time-out interval.
     * |        |          |0 = WDT time-out interrupt did not occur.
     * |        |          |1 = WDT time-out interrupt occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[4]     |WTWKE     |Watchdog Timer Time-out Wake-up Function Control (Write Protect)
     * |        |          |If this bit is set to 1, while WTIF is generated to 1 and WTIE enabled, the WDT time-out interrupt signal will generate a wake-up trigger event to chip.
     * |        |          |0 = Wake-up trigger event Disabled if WDT time-out interrupt signal generated.
     * |        |          |1 = Wake-up trigger event Enabled if WDT time-out interrupt signal generated.
     * |        |          |Note: Chip can be woken-up by WDT time-out interrupt signal generated only if WDT clock source is selected to 10 kHz oscillator.
     * |[5]     |WTWKF     |Watchdog Timer Time-out Wake-up Flag
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
     * |[10:8]  |WTIS      |Watchdog Timer Interval Selection
     * |        |          |These three bits select the time-out interval for the Watchdog Timer.
     * |        |          |000 = 24 * TWDT.
     * |        |          |001 = 26 * TWDT.
     * |        |          |010 = 28 * TWDT.
     * |        |          |011 = 210 * TWDT.
     * |        |          |100 = 212 * TWDT.
     * |        |          |101 = 214 * TWDT.
     * |        |          |110 = 216 * TWDT.
     * |        |          |111 = 218 * TWDT.
     * |[31]    |DBGACK_WDT|ICE Debug Mode Acknowledge Disable Control (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects WDT counting.
     * |        |          |WDT up counter will be kept while CPU is hanging by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |WDT up counter will keep going no matter CPU is hanging by ICE or not.
     */

    __IO uint32_t WTCR;          /* Offset: 0x00  Watchdog Timer Control Register                                    */

} WDT_T;



/**
    @addtogroup WDT_CONST WDT Bit Field Definition
    Constant Definitions for WDT Controller
@{ */

#define WDT_WTCR_WTR_Pos                 (0)                                               /*!< WDT_T::WTCR: WTR Position                 */
#define WDT_WTCR_WTR_Msk                 (0x1ul << WDT_WTCR_WTR_Pos)                       /*!< WDT_T::WTCR: WTR Mask                     */

#define WDT_WTCR_WTRE_Pos                (1)                                               /*!< WDT_T::WTCR: WTRE Position                */
#define WDT_WTCR_WTRE_Msk                (0x1ul << WDT_WTCR_WTRE_Pos)                      /*!< WDT_T::WTCR: WTRE Mask                    */

#define WDT_WTCR_WTRF_Pos                (2)                                               /*!< WDT_T::WTCR: WTRF Position                */
#define WDT_WTCR_WTRF_Msk                (0x1ul << WDT_WTCR_WTRF_Pos)                      /*!< WDT_T::WTCR: WTRF Mask                    */

#define WDT_WTCR_WTIF_Pos                (3)                                               /*!< WDT_T::WTCR: WTIF Position                */
#define WDT_WTCR_WTIF_Msk                (0x1ul << WDT_WTCR_WTIF_Pos)                      /*!< WDT_T::WTCR: WTIF Mask                    */

#define WDT_WTCR_WTWKE_Pos               (4)                                               /*!< WDT_T::WTCR: WTWKE Position               */
#define WDT_WTCR_WTWKE_Msk               (0x1ul << WDT_WTCR_WTWKE_Pos)                     /*!< WDT_T::WTCR: WTWKE Mask                   */

#define WDT_WTCR_WTWKF_Pos               (5)                                               /*!< WDT_T::WTCR: WTWKF Position               */
#define WDT_WTCR_WTWKF_Msk               (0x1ul << WDT_WTCR_WTWKF_Pos)                     /*!< WDT_T::WTCR: WTWKF Mask                   */

#define WDT_WTCR_WTIE_Pos                (6)                                               /*!< WDT_T::WTCR: WTIE Position                */
#define WDT_WTCR_WTIE_Msk                (0x1ul << WDT_WTCR_WTIE_Pos)                      /*!< WDT_T::WTCR: WTIE Mask                    */

#define WDT_WTCR_WTE_Pos                 (7)                                               /*!< WDT_T::WTCR: WTE Position                 */
#define WDT_WTCR_WTE_Msk                 (0x1ul << WDT_WTCR_WTE_Pos)                       /*!< WDT_T::WTCR: WTE Mask                     */

#define WDT_WTCR_WTIS_Pos                (8)                                               /*!< WDT_T::WTCR: WTIS Position                */
#define WDT_WTCR_WTIS_Msk                (0x7ul << WDT_WTCR_WTIS_Pos)                      /*!< WDT_T::WTCR: WTIS Mask                    */

#define WDT_WTCR_DBGACK_WDT_Pos          (31)                                              /*!< WDT_T::WTCR: DBGACK_WDT Position          */
#define WDT_WTCR_DBGACK_WDT_Msk          (0x1ul << WDT_WTCR_DBGACK_WDT_Pos)                /*!< WDT_T::WTCR: DBGACK_WDT Mask              */

/**@}*/ /* WDT_CONST */
/**@}*/ /* end of WDT register group */


#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/** @addtogroup MINI51_PERIPHERAL_MEM_MAP MINI51 Peripheral Memory Map
  Memory Mapped Structure for MINI51 Series Peripheral
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

#define GCR_BASE              (AHBPERIPH_BASE + 0x00000)    ///< GCR register base address
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

/*@}*/ /* end of group MINI51_PERIPHERAL_MEM_MAP */


/** @addtogroup MINI51_PERIPHERAL_DECLARATION MINI51 Peripheral Declaration
  The Declaration of MINI51 Series Peripheral
  @{
 */
#define WDT                   ((WDT_T *) WDT_BASE)              ///< Pointer to WDT register structure
#define TIMER0                ((TIMER_T *) TIMER0_BASE)         ///< Pointer to Timer 0 register structure
#define TIMER1                ((TIMER_T *) TIMER1_BASE)         ///< Pointer to Timer 1 register structure
#define I2C                   ((I2C_T *) I2C_BASE)              ///< Pointer to I2C register structure
#define I2C0                  ((I2C_T *) I2C_BASE)              ///< Pointer to I2C register structure
#define SPI                   ((SPI_T *) SPI_BASE)              ///< Pointer to SPI register structure
#define SPI0                  ((SPI_T *) SPI_BASE)              ///< Pointer to SPI register structure
#define PWM                   ((PWM_T *) PWM_BASE)              ///< Pointer to PWM register structure
#define UART                  ((UART_T *) UART_BASE)            ///< Pointer to UART register structure
#define UART0                 ((UART_T *) UART_BASE)            ///< Pointer to UART register structure
#define ADC                   ((ADC_T *) ADC_BASE)              ///< Pointer to ADC register structure
#define ACMP                  ((ACMP_T *) ACMP_BASE)            ///< Pointer to ACMP register structure

#define SYS                   ((GCR_T *) GCR_BASE)              ///< Pointer to SYS register structure
#define CLK                   ((CLK_T *) CLK_BASE)              ///< Pointer to CLK register structure
#define INT                   ((INT_T *) INT_BASE)              ///< Pointer to INT register structure
#define P0                    ((GPIO_T *) P0_BASE)              ///< Pointer to GPIO port 0 register structure
#define P1                    ((GPIO_T *) P1_BASE)              ///< Pointer to GPIO port 1 register structure 
#define P2                    ((GPIO_T *) P2_BASE)              ///< Pointer to GPIO port 2 register structure
#define P3                    ((GPIO_T *) P3_BASE)              ///< Pointer to GPIO port 3 register structure
#define P4                    ((GPIO_T *) P4_BASE)              ///< Pointer to GPIO port 4 register structure
#define P5                    ((GPIO_T *) P5_BASE)              ///< Pointer to GPIO port 5 register structure
#define GPIO                  ((GPIO_DBNCECON_T *) GPIO_DBNCECON_BASE)      ///< Pointer to GPIO de-bounce register structure
#define FMC                   ((FMC_T *) FMC_BASE)              ///< Pointer to FMC register structure

/*@}*/ /* end of group MINI51_PERIPHERAL_DECLARATION */
/*@}*/ /* end of group MINI51_Peripherals */

/** @addtogroup MINI51_IO_ROUTINE MINI51 I/O routines
  The Declaration of MINI51 I/O routines
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


/*@}*/ /* end of group MINI51_IO_ROUTINE */

/******************************************************************************/
/*                Legacy Constants                                            */
/******************************************************************************/
/** @addtogroup MINI51_legacy_Constants MINI51 Legacy Constants
  MINI51 Legacy Constants
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

/*@}*/ /* end of group MINI51_legacy_Constants */

/*@}*/ /* end of group MINI51_Definitions */

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

#endif  // __MINI51SERIES_H__

