/**************************************************************************//**
 * @file     Mini55Series.h
 * @version  V1.00
 * $Revision: 26 $
 * $Date: 15/07/17 3:25p $
 * @brief    Mini55 series peripheral access layer header file.
 *           This file contains all the peripheral register's definitions,
 *           bits definitions and memory mapping for NuMicro Mini55 series MCU.
 *
 * @note
 * Copyright (C) 2014~2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
/**
   \mainpage NuMicro Mini55 Driver Reference Guide
   *
   * <b>Introduction</b>
   *
   * This user manual describes the usage of Mini55 Series MCU device driver
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

#ifndef __MINI55SERIES_H__
#define __MINI55SERIES_H__

#ifdef __cplusplus
extern "C" {
#endif

/** @addtogroup Mini55_Definitions MINI55 Definitions
  This file defines all structures and symbols for Mini55:
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
/** @addtogroup MINI55_CMSIS Device CMSIS Definitions
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

    /******  Mini55 specific Interrupt Numbers ***********************************************/

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
    UART0_IRQn            = 12,     /*!< UART0 interrupt                                    */
    UART1_IRQn            = 13,     /*!< UART1 interrupt                                    */
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

/*@}*/ /* end of group MINI55_CMSIS */


#include "core_cm0.h"                       /* Cortex-M0 processor and core peripherals           */
#include "system_Mini55Series.h"            /* Mini55 Series System include file                  */
#include <stdint.h>

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup Mini55_Peripherals MINI55 Peripherals
  MINI55 Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM  )
#pragma anon_unions
#endif


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/



/*---------------------- Analog Comparator Controller -------------------------*/
/**
    @addtogroup ACMP Analog Comparator Controller(ACMP)
    Memory Mapped Structure for ACMP Controller
@{ */

typedef struct
{


    /**
     * CTL0, CTL1
     * ===================================================================================================
     * Offset: 0x00, 0x04  Analog Comparator 0, Analog Comparator 1 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPEN    |Analog Comparator Enable Control
     * |        |          |0 = Analog Comparator 0 Disabled.
     * |        |          |1 = Analog Comparator 1 Enabled.
     * |        |          |Note: Analog comparator output needs to wait 2 us stable time after this bit is set.
     * |[1]     |ACMPIE    |Analog Comparator Interrupt Enable Control
     * |        |          |0 = Interrupt function Disabled.
     * |        |          |1 = Interrupt function Enabled.
     * |[3:2]   |HYSSEL    |Analog Comparator Hysteresis Selection
     * |        |          |00 = CMP0 Hysteresis Disabled.
     * |        |          |01 = CMP0 Hysteresis typical range is 15mV.
     * |        |          |10 = CMP0 Hysteresis typical range is 90mV.
     * |        |          |11 = Same as 00.
     * |[4]     |NEGSEL    |Analog Comparator Negative Input Selection
     * |        |          |0 = The source of the negative comparator input is from CPN pin.
     * |        |          |1 = The source of the negative comparator input is from internal band-gap voltage or comparator reference voltage.
     * |[8]     |RTRGEN    |Analog Comparator Rising Edge Trigger Enable
     * |        |          |0 = Analog comparator rising edge trigger PWM or Timer enabled.
     * |        |          |1 = Analog comparator rising edge trigger disabled.
     * |        |          |Note: The bit is only effective while analog comparator triggers PWM or Timer.
     * |[9]     |FTRGEN    |Analog Comparator Falling Edge Trigger Enable
     * |        |          |0 = Analog comparator falling edge trigger PWM or Timer enabled.
     * |        |          |1 = Analog comparator falling edge trigger disabled.
     * |        |          |Note: The bit is only effective while analog comparator triggers PWM or Timer.
     * |[12]    |SMPTSEL   |Analog Comparator Speed Mode Selection
     * |        |          |0 = Slow mode.
     * |        |          |1 = Fast mode.
     * |[30:29] |POSSEL    |Analog Comparator Positive Input Selection
     * |        |          |00 = CPP is from P1.5 pin.
     * |        |          |01 = CPP is from P1.0 pin.
     * |        |          |10 = CPP is from P1.2 pin.
     * |        |          |11 = CPP is from P1.3 pin.
    */
    __IO uint32_t CTL[2];

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x08  Analog Comparator 0/1 Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPIF0   |Analog Comparator 0 Flag
     * |        |          |This bit is set by hardware whenever the comparator 0 output changes state.
     * |        |          |This will generate an interrupt if ACMPIE(ACMP_CTL0[1]) = 1.
     * |        |          |0 = Analog comparator 0 output does not change.
     * |        |          |1 = Analog comparator 0 output changed.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[1]     |ACMPIF1   |Analog Comparator 1 Flag
     * |        |          |This bit is set by hardware whenever the comparator 1 output changes state.
     * |        |          |This will generate an interrupt if ACMPIE(ACMP_CTL1[1]) = 1.
     * |        |          |0 = Analog comparator 1 output does not change.
     * |        |          |1 = Analog comparator 1 output changed.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[2]     |ACMPO0    |Analog Comparator 0 Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the comparator 0 is disabled ACMPEN(ACMP_CTL0[0]) = 0.
     * |        |          |0 = Analog comparator 0 outputs 0.
     * |        |          |1 = Analog comparator 0 outputs 1.
     * |[3]     |ACMPO1    |Analog Comparator 1 Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the comparator 1 is disabled ACMPEN(ACMP_CTL1[0]) = 0.
     * |        |          |0 = Analog comparator 1 outputs 0.
     * |        |          |1 = Analog comparator 1 outputs 1.
    */
    __IO uint32_t STATUS;

    /**
     * VREF
     * ===================================================================================================
     * Offset: 0x0C  Analog Comparator Reference Voltage Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |CRVCTL    |Internal Reference Selection
     * |        |          |Comparator reference voltage = AVDD * (1 / 6 + CRVCTL[3:0] / 24).
     * |[7]     |IREFSEL   |CRV Module Output Selection
     * |        |          |0 = Band-gap voltage.
     * |        |          |1 = Internal comparator reference voltage.
    */
    __IO uint32_t VREF;

} ACMP_T;

/**
    @addtogroup ACMP_CONST ACMP Bit Field Definition
    Constant Definitions for ACMP Controller
@{ */

#define ACMP_CTL_ACMPEN_Pos             (0)                                                /*!< ACMP_T::CTL: ACMPEN Position             */
#define ACMP_CTL_ACMPEN_Msk             (0x1ul << ACMP_CTL_ACMPEN_Pos)                     /*!< ACMP_T::CTL: ACMPEN Mask                 */

#define ACMP_CTL_ACMPIE_Pos             (1)                                                /*!< ACMP_T::CTL: ACMPIE Position             */
#define ACMP_CTL_ACMPIE_Msk             (0x1ul << ACMP_CTL_ACMPIE_Pos)                     /*!< ACMP_T::CTL: ACMPIE Mask                 */

#define ACMP_CTL_HYSSEL_Pos             (2)                                                /*!< ACMP_T::CTL: HYSSEL Position             */
#define ACMP_CTL_HYSSEL_Msk             (0x3ul << ACMP_CTL_HYSSEL_Pos)                     /*!< ACMP_T::CTL: HYSSEL Mask                 */

#define ACMP_CTL_NEGSEL_Pos             (4)                                                /*!< ACMP_T::CTL: NEGSEL Position             */
#define ACMP_CTL_NEGSEL_Msk             (0x1ul << ACMP_CTL_NEGSEL_Pos)                     /*!< ACMP_T::CTL: NEGSEL Mask                 */

#define ACMP_CTL_RTRGEN_Pos             (8)                                                /*!< ACMP_T::CTL: RTRGEN Position             */
#define ACMP_CTL_RTRGEN_Msk             (0x1ul << ACMP_CTL_RTRGEN_Pos)                     /*!< ACMP_T::CTL: RTRGEN Mask                 */

#define ACMP_CTL_FTRGEN_Pos             (9)                                                /*!< ACMP_T::CTL: FTRGEN Position             */
#define ACMP_CTL_FTRGEN_Msk             (0x1ul << ACMP_CTL_FTRGEN_Pos)                     /*!< ACMP_T::CTL: FTRGEN Mask                 */

#define ACMP_CTL_SMPTSEL_Pos            (12)                                               /*!< ACMP_T::CTL: SMPTSEL Position            */
#define ACMP_CTL_SMPTSEL_Msk            (0x1ul << ACMP_CTL_SMPTSEL_Pos)                    /*!< ACMP_T::CTL: SMPTSEL Mask                */

#define ACMP_CTL_POSSEL_Pos             (29)                                               /*!< ACMP_T::CTL: POSSEL Position             */
#define ACMP_CTL_POSSEL_Msk             (0x3ul << ACMP_CTL_POSSEL_Pos)                     /*!< ACMP_T::CTL: POSSEL Mask                 */

#define ACMP_STATUS_ACMPIF0_Pos          (0)                                               /*!< ACMP_T::STATUS: ACMPIF0 Position          */
#define ACMP_STATUS_ACMPIF0_Msk          (0x1ul << ACMP_STATUS_ACMPIF0_Pos)                /*!< ACMP_T::STATUS: ACMPIF0 Mask              */

#define ACMP_STATUS_ACMPIF1_Pos          (1)                                               /*!< ACMP_T::STATUS: ACMPIF1 Position          */
#define ACMP_STATUS_ACMPIF1_Msk          (0x1ul << ACMP_STATUS_ACMPIF1_Pos)                /*!< ACMP_T::STATUS: ACMPIF1 Mask              */

#define ACMP_STATUS_ACMPO0_Pos           (2)                                               /*!< ACMP_T::STATUS: ACMPO0 Position           */
#define ACMP_STATUS_ACMPO0_Msk           (0x1ul << ACMP_STATUS_ACMPO0_Pos)                 /*!< ACMP_T::STATUS: ACMPO0 Mask               */

#define ACMP_STATUS_ACMPO1_Pos           (3)                                               /*!< ACMP_T::STATUS: ACMPO1 Position           */
#define ACMP_STATUS_ACMPO1_Msk           (0x1ul << ACMP_STATUS_ACMPO1_Pos)                 /*!< ACMP_T::STATUS: ACMPO1 Mask               */

#define ACMP_VREF_CRVCTL_Pos             (0)                                               /*!< ACMP_T::VREF: CRVCTL Position             */
#define ACMP_VREF_CRVCTL_Msk             (0xful << ACMP_VREF_CRVCTL_Pos)                   /*!< ACMP_T::VREF: CRVCTL Mask                 */

#define ACMP_VREF_IREFSEL_Pos            (7)                                               /*!< ACMP_T::VREF: IREFSEL Position            */
#define ACMP_VREF_IREFSEL_Msk            (0x1ul << ACMP_VREF_IREFSEL_Pos)                  /*!< ACMP_T::VREF: IREFSEL Mask                */

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
     * DAT
     * ===================================================================================================
     * Offset: 0x00  ADC Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |RESULT    |A/D Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OV        |Over Run Flag
     * |        |          |0 = Data in RESULT[9:0] is recent conversion result.
     * |        |          |1 = Data in RESULT[9:0] overwrote.
     * |        |          |If converted data in RESULT[9:0] has not been read before the new conversion result is loaded to this register, OV is set to 1.
     * |        |          |It is cleared by hardware after the ADC_DAT register is read.
     * |[17]    |VALID     |Valid Flag
     * |        |          |0 = Data in RESULT[9:0] bits not valid.
     * |        |          |1 = Data in RESULT[9:0] bits valid.
     * |        |          |This bit is set to 1 when ADC conversion is completed and cleared by hardware after the ADC_DAT register is read.
    */
    __I  uint32_t DAT;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED0[7];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x20  ADC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCEN     |A/D Converter Enable
     * |        |          |0 = A/D Converter Disabled.
     * |        |          |1 = A/D Converter Enabled.
     * |        |          |Note: Before starting A/D conversion function, this bit should be set to 1.
     * |        |          |Clear it to 0 to disable A/D converter analog circuit to save power consumption.
     * |[1]     |ADCIEN    |A/D Interrupt Enable
     * |        |          |A/D conversion end interrupt request is generated if ADCIEN bit is set to 1.
     * |        |          |0 = A/D interrupt function Disabled.
     * |        |          |1 = A/D interrupt function Enabled.
     * |[5:4]   |HWTRGSEL  |Hardware Trigger Source
     * |        |          |00 = A/D conversion is started by external STADC pin.
     * |        |          |11 = A/D conversion is started by PWM trigger.
     * |        |          |Others = Reserved.
     * |        |          |Note: Software should disable HWTRGEN and SWTRG before change HWTRGSEL.
     * |[6]     |HWTRGCOND |External Trigger Condition
     * |        |          |This bit decides whether the external pin STADC trigger event is falling or raising edge.
     * |        |          |The signal must be kept at stable state at least 4 PCLKs at high and low state for edge trigger.
     * |        |          |0 = Falling edge.
     * |        |          |1 = Raising edge.
     * |[8]     |HWTRGEN   |External Trigger Enable
     * |        |          |Enable or disable triggering of A/D conversion by external STADC pin.
     * |        |          |If external trigger is enabled, the SWTRG bit can be set to 1 by the selected hardware trigger source.
     * |        |          |0= External trigger Disabled.
     * |        |          |1= External trigger Enabled.
     * |[11]    |SWTRG     |A/D Conversion Start
     * |        |          |SWTRG bit can be set to 1 from two sources: software and external pin STADC.
     * |        |          |SWTRG will be cleared to 0 by hardware automatically after conversion complete.
     * |        |          |0 = Conversion stopped and A/D converter entered idle state.
     * |        |          |1 = Conversion start.
     * |[12]    |VREFSEL   |Reference Voltage Selection Signal
     * |        |          |0 = Connect VDD5V to internal reference.
     * |        |          |1 = Connect VREF (AIN0) pin to internal reference.
    */
    __IO uint32_t CTL;

    /**
     * CHEN
     * ===================================================================================================
     * Offset: 0x24  ADC Channel Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHEN0     |Analog Input Channel 0 Enable
     * |        |          |0 = Channel 0 Disabled.
     * |        |          |1 = Channel 0 Enabled.
     * |        |          |Note: If software enables more than one channel, the channel with the smallest number will be selected and the other enabled channels will be ignored.
     * |[1]     |CHEN1     |Analog Input Channel 1 Enable
     * |        |          |0 = Channel 1 Disabled.
     * |        |          |1 = Channel 1 Enabled.
     * |[2]     |CHEN2     |Analog Input Channel 2 Enable
     * |        |          |0 = Channel 2 Disabled.
     * |        |          |1 = Channel 2 Enabled.
     * |[3]     |CHEN3     |Analog Input Channel 3 Enable
     * |        |          |0 = Channel 3 Disabled.
     * |        |          |1 = Channel 3 Enabled.
     * |[4]     |CHEN4     |Analog Input Channel 4 Enable
     * |        |          |0 = Channel 4 Disabled.
     * |        |          |1 = Channel 4 Enabled.
     * |[5]     |CHEN5     |Analog Input Channel 5 Enable
     * |        |          |0 = Channel 5 Disabled.
     * |        |          |1 = Channel 5 Enabled.
     * |[6]     |CHEN6     |Analog Input Channel 6 Enable
     * |        |          |0 = Channel 6 Disabled.
     * |        |          |1 = Channel 6 Enabled.
     * |[7]     |CHEN7     |Analog Input Channel 7 Enable
     * |        |          |0 = Channel 7 Disabled.
     * |        |          |1 = Channel 7 Enabled.
     * |[8]     |CH7SEL    |Analog Input Channel 7 Selection
     * |        |          |0 = External analog input.
     * |        |          |1 = Internal band-gap voltage (VBG).
     * |        |          |Note: When software selects the band-gap voltage as the analog input source of ADC channel 7, the ADC clock rate needs to be limited to lower than 300 kHz.
     * |[9]     |CHEN8     |Analog Input Channel 8 Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[10]    |CHEN9     |Analog Input Channel 9 Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[11]    |CHEN10    |Analog Input Channel 10 Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[12]    |CHEN11    |Analog Input Channel 11 Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[13]    |BGEN      |Band-Gap Voltage Measurement
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: User can set BGEN high to use ADC to measure Band-Gap voltage directly to instead of enabling PRESET and CHEN7.
    */
    __IO uint32_t CHEN;

    /**
     * CMP0,CMP1
     * ===================================================================================================
     * Offset: 0x28 ADC Compare Register 0, 0x2C  ADC Compare Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCMPEN   |Compare Enable
     * |        |          |Set 1 to this bit to enable comparing CMPDAT[9:0] with specified channel conversion results when converted data is loaded into the ADC_DAT register.
     * |        |          |0 = Compare function Disabled.
     * |        |          |1 = Compare function Enabled.
     * |[1]     |ADCMPIE   |Compare Interrupt Enable
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMCNT, ADCMPFx bit will be asserted, in the meanwhile, if CMPCOND is set to 1, a compare interrupt request is generated.
     * |        |          |0 = Compare function interrupt Disabled.
     * |        |          |1 = Compare function interrupt Enabled.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0 = Set the compare condition as that when a 10-bit A/D conversion result is less than the 10-bit CMPDAT (ADCMPRx[25:16]), the internal match counter will increase one.
     * |        |          |1 = Set the compare condition as that when a 10-bit A/D conversion result is greater or equal to the 10-bit CMPDAT (ADCMPRx[25:16]), the internal match counter will increase one.
     * |        |          |Note: When the internal counter reaches the value to (CMPMCNT +1), the ADCMPFx bit will be set.
     * |[6:3]   |CMPCH     |Compare Channel Selection
     * |        |          |0000 = Channel 0 conversion result is selected to be compared.
     * |        |          |0001 = Channel 1 conversion result is selected to be compared.
     * |        |          |0010 = Channel 2 conversion result is selected to be compared.
     * |        |          |0011 = Channel 3 conversion result is selected to be compared.
     * |        |          |0100 = Channel 4 conversion result is selected to be compared.
     * |        |          |0101 = Channel 5 conversion result is selected to be compared.
     * |        |          |0110 = Channel 6 conversion result is selected to be compared.
     * |        |          |0111 = Channel 7 conversion result is selected to be compared.
     * |        |          |1000 = Channel 8 conversion result is selected to be compared.
     * |        |          |1001 = Channel 9 conversion result is selected to be compared.
     * |        |          |1010 = Channel 10 conversion result is selected to be compared.
     * |        |          |1011 = Channel 11 conversion result is selected to be compared.
     * |        |          |1100 = band-gap voltage result is selected to be compared.
     * |[11:8]  |CMPMCNT   |Compare Match Count
     * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND[2], the internal match counter will increase 1.
     * |        |          |When the internal counter reaches the value to (CMPMCNT +1), the ADCMPFx bit will be set.
     * |[25:16] |CMPDAT    |Comparison Data
     * |        |          |The 10-bit data is used to compare with conversion result of specified channel.
    */
    __IO uint32_t CMP[2];

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x30  ADC Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADIF      |A/D Conversion End Flag
     * |        |          |A status flag that indicates the end of A/D conversion. ADIF is set to 1 When A/D conversion ends.
     * |        |          |Software can write 1 to clear this bit to zero.
     * |[1]     |ADCMPF0   |Compare Flag 0
     * |        |          |When the selected channel A/D conversion result meets the setting condition in ADC_CMP0, this bit is set to 1.
     * |        |          |Software can write 1 to clear this bit to zero.
     * |        |          |0 = Conversion result in ADC_DAT does not meet the ADC_CMP0 setting.
     * |        |          |1 = Conversion result in ADC_DAT meets the ADC_CMP0 setting.
     * |[2]     |ADCMPF1   |Compare Flag 1
     * |        |          |When the selected channel A/D conversion result meets the setting condition in ADC_CMP1, this bit is set to 1.
     * |        |          |Software can write 1 to clear this bit to zero.
     * |        |          |0 = Conversion result in ADC_DAT does not meet the ADC_CMP1 setting.
     * |        |          |1 = Conversion result in ADC_DAT meets the ADC_CMP1 setting.
     * |[3]     |BUSY      |BUSY/IDLE (Read Only)
     * |        |          |This bit is mirror of as SWTRG bit in ADC_CTL
     * |        |          |0 = A/D converter is in idle state.
     * |        |          |1 = A/D converter is busy at conversion.
     * |[6:4]   |CHANNEL   |Current Conversion Channel (Read Only)
     * |        |          |This filed reflects the current conversion channel when BUSY=1.
     * |        |          |When BUSY=0, it shows the number of the next converted channel.
     * |[8]     |VALID     |Data Valid Flag (Read Only)
     * |        |          |It is a mirror of VALID bit in ADC_DAT register.
     * |[16]    |OV        |OV Flag (Read Only)
     * |        |          |It is a mirror to OV bit in ADC_DAT register.
    */
    __IO uint32_t STATUS;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED1[4];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * TRGDLY
     * ===================================================================================================
     * Offset: 0x44  ADC Trigger Delay Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DELAY     |PWM Trigger Delay Timer
     * |        |          |Set this field will delay ADC start conversion time after PWM trigger.
     * |        |          |PWM trigger delay time is (4 * DELAY) * system clock.
    */
    __IO uint32_t TRGDLY;

    /**
     * EXTSMPT
     * ===================================================================================================
     * Offset: 0x48  ADC Sampling Time Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |EXTSMPT   |ADC Sampling Counter
     * |        |          |0000 = 0 ADC Clock
     * |        |          |0001 = 1 ADC Clock
     * |        |          |0010 = 2 ADC Clock
     * |        |          |0011 = 4 ADC Clock
     * |        |          |0100 = 5 ADC Clock
     * |        |          |0101 = 6 ADC Clock
     * |        |          |0110 = 7 ADC Clock
     * |        |          |0111 = 8 ADC Clock
     * |        |          |1000 = 16 ADC Clock
     * |        |          |1001 = 32 ADC Clock
     * |        |          |1010 = 64 ADC Clock
     * |        |          |1011 = 128 ADC Clock
     * |        |          |1100 = 256 ADC Clock
     * |        |          |1101 = 512 ADC Clock
     * |        |          |1110 = 1024 ADC Clock
     * |        |          |1111 = 1024 ADC Clock
     * |        |          |ADC sampling counters are 6 ADC clock is suggestion.
    */
    __IO uint32_t EXTSMPT;

    /**
     * SEQCTL
     * ===================================================================================================
     * Offset: 0x4C  ADC PWM Sequential Mode Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SEQEN     |ADC Sequential Mode Enable
     * |        |          |When ADC sequential mode is enabled, two of three ADC channels from 0 to 2 will automatically convert analog data in the sequence of channel [0, 1] or channel[1, 2] or channel[0, 2] defined by SEQ_MODE[1:0].
     * |        |          |0 = ADC sequential mode Disabled.
     * |        |          |1 = ADC sequential mode Enabled.
     * |[1]     |SEQTYPE   |ADC Sequential Mode Type
     * |        |          |0 = ADC delay time is only inserted before the first conversion.
     * |        |          |The second conversion starts immediately after the first conversion is completed.
     * |        |          |(for 2/3-shunt type).
     * |        |          |1 = ADC delay time is inserted before each conversion. (for 1-shunt type)
     * |[3:2]   |MODESEL   |ADC Sequential Mode Selection
     * |        |          |00 = Issue ADC_INT after Channel 0 then Channel 1 conversion finishes when SEQEN =1.
     * |        |          |01 = Issue ADC_INT after Channel 1 then Channel 2 conversion finishes when SEQEN =1.
     * |        |          |10 = Issue ADC_INT after Channel 0 then Channel 2 conversion finishes when SEQEN =1.
     * |        |          |11 = Reserved
     * |[9:8]   |TRG1TYPE  |ADC Sequential Mode Trigger1 Type
     * |        |          |00 = Rising of the selected PWM
     * |        |          |01 = center of the selected PWM
     * |        |          |10 = Falling of the selected PWM
     * |        |          |11 = Period of the selected PWM
     * |[11:10] |TRG1SRC   |ADC Sequential Mode Trigger1 Source
     * |        |          |00 = PWM0
     * |        |          |01 = PWM2
     * |        |          |10 = PWM4
     * |        |          |11 = Reserved
     * |[17:16] |TRG2TYPE  |ADC Sequential Mode Trigger2 Type
     * |        |          |00 = Rising of the selected PWM
     * |        |          |01 = center of the selected PWM
     * |        |          |10 = Falling of the selected PWM
     * |        |          |11 = Period of the selected PWM
     * |[19:18] |TRG2SRC   |ADC Sequential Mode Trigger2 Source
     * |        |          |00 = PWM0
     * |        |          |01 = PWM2
     * |        |          |10 = PWM4
     * |        |          |11 = Reserved
    */
    __IO uint32_t SEQCTL;

    /**
     * SEQDAT0
     * ===================================================================================================
     * Offset: 0x50  ADC PWM Sequential Mode Result Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |RESULT    |A/D PWM Sequential Mode Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OV        |Over Run Flag
     * |        |          |0 = Data in RESULT [9:0] is recent conversion result.
     * |        |          |1 = Data in RESULT [9:0] overwritten.
     * |        |          |If converted data in RESULT [9:0] has not been read before the new conversion result is loaded to this register, OV is set to 1.
     * |        |          |It is cleared by hardware after the ADC_DAT register is read.
     * |[17]    |VALID     |Valid Flag
     * |        |          |0 = Data in RESULT [9:0] bits not valid.
     * |        |          |1 = Data in RESULT. ADC_TRGDLY [9:0] bits valid.
     * |        |          |This bit is set to 1 when ADC conversion is completed and cleared by hardware after the ADC_DAT register is read.
    */
    __I  uint32_t SEQDAT0;

    /**
     * SEQDAT1
     * ===================================================================================================
     * Offset: 0x54  ADC PWM Sequential Mode Result Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[9:0]   |RESULT    |A/D PWM Sequential Mode Conversion Result
     * |        |          |This field contains conversion result of ADC.
     * |[16]    |OV        |Over Run Flag
     * |        |          |0 = Data in RESULT [9:0] is recent conversion result.
     * |        |          |1 = Data in RESULT [9:0] overwritten.
     * |        |          |If converted data in RESULT [9:0] has not been read before the new conversion result is loaded to this register, OV is set to 1.
     * |        |          |It is cleared by hardware after the ADC_DAT register is read.
     * |[17]    |VALID     |Valid Flag
     * |        |          |0 = Data in RESULT [9:0] bits not valid.
     * |        |          |1 = Data in RESULT. ADC_TRGDLY [9:0] bits valid.
     * |        |          |This bit is set to 1 when ADC conversion is completed and cleared by hardware after the ADC_DAT register is read.
    */
    __I  uint32_t SEQDAT1;

} ADC_T;

/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */

#define ADC_DAT_RESULT_Pos               (0)                                               /*!< ADC_T::DAT: RESULT Position               */
#define ADC_DAT_RESULT_Msk               (0x3fful << ADC_DAT_RESULT_Pos)                   /*!< ADC_T::DAT: RESULT Mask                   */

#define ADC_DAT_OV_Pos                   (16)                                              /*!< ADC_T::DAT: OV Position                   */
#define ADC_DAT_OV_Msk                   (0x1ul << ADC_DAT_OV_Pos)                         /*!< ADC_T::DAT: OV Mask                       */

#define ADC_DAT_VALID_Pos                (17)                                              /*!< ADC_T::DAT: VALID Position                */
#define ADC_DAT_VALID_Msk                (0x1ul << ADC_DAT_VALID_Pos)                      /*!< ADC_T::DAT: VALID Mask                    */

#define ADC_CTL_ADCEN_Pos                (0)                                               /*!< ADC_T::CTL: ADCEN Position                */
#define ADC_CTL_ADCEN_Msk                (0x1ul << ADC_CTL_ADCEN_Pos)                      /*!< ADC_T::CTL: ADCEN Mask                    */

#define ADC_CTL_ADCIEN_Pos               (1)                                               /*!< ADC_T::CTL: ADCIEN Position               */
#define ADC_CTL_ADCIEN_Msk               (0x1ul << ADC_CTL_ADCIEN_Pos)                     /*!< ADC_T::CTL: ADCIEN Mask                   */

#define ADC_CTL_HWTRGSEL_Pos             (4)                                               /*!< ADC_T::CTL: HWTRGSEL Position             */
#define ADC_CTL_HWTRGSEL_Msk             (0x3ul << ADC_CTL_HWTRGSEL_Pos)                   /*!< ADC_T::CTL: HWTRGSEL Mask                 */

#define ADC_CTL_HWTRGCOND_Pos            (6)                                               /*!< ADC_T::CTL: HWTRGCOND Position            */
#define ADC_CTL_HWTRGCOND_Msk            (0x1ul << ADC_CTL_HWTRGCOND_Pos)                  /*!< ADC_T::CTL: HWTRGCOND Mask                */

#define ADC_CTL_HWTRGEN_Pos              (8)                                               /*!< ADC_T::CTL: HWTRGEN Position              */
#define ADC_CTL_HWTRGEN_Msk              (0x1ul << ADC_CTL_HWTRGEN_Pos)                    /*!< ADC_T::CTL: HWTRGEN Mask                  */

#define ADC_CTL_SWTRG_Pos                (11)                                              /*!< ADC_T::CTL: SWTRG Position                */
#define ADC_CTL_SWTRG_Msk                (0x1ul << ADC_CTL_SWTRG_Pos)                      /*!< ADC_T::CTL: SWTRG Mask                    */

#define ADC_CTL_VREFSEL_Pos              (12)                                              /*!< ADC_T::CTL: VREFSEL Position              */
#define ADC_CTL_VREFSEL_Msk              (0x1ul << ADC_CTL_VREFSEL_Pos)                    /*!< ADC_T::CTL: VREFSEL Mask                  */

#define ADC_CHEN_CHEN0_Pos               (0)                                               /*!< ADC_T::CHEN: CHEN0 Position               */
#define ADC_CHEN_CHEN0_Msk               (0x1ul << ADC_CHEN_CHEN0_Pos)                     /*!< ADC_T::CHEN: CHEN0 Mask                   */

#define ADC_CHEN_CHEN1_Pos               (1)                                               /*!< ADC_T::CHEN: CHEN1 Position               */
#define ADC_CHEN_CHEN1_Msk               (0x1ul << ADC_CHEN_CHEN1_Pos)                     /*!< ADC_T::CHEN: CHEN1 Mask                   */

#define ADC_CHEN_CHEN2_Pos               (2)                                               /*!< ADC_T::CHEN: CHEN2 Position               */
#define ADC_CHEN_CHEN2_Msk               (0x1ul << ADC_CHEN_CHEN2_Pos)                     /*!< ADC_T::CHEN: CHEN2 Mask                   */

#define ADC_CHEN_CHEN3_Pos               (3)                                               /*!< ADC_T::CHEN: CHEN3 Position               */
#define ADC_CHEN_CHEN3_Msk               (0x1ul << ADC_CHEN_CHEN3_Pos)                     /*!< ADC_T::CHEN: CHEN3 Mask                   */

#define ADC_CHEN_CHEN4_Pos               (4)                                               /*!< ADC_T::CHEN: CHEN4 Position               */
#define ADC_CHEN_CHEN4_Msk               (0x1ul << ADC_CHEN_CHEN4_Pos)                     /*!< ADC_T::CHEN: CHEN4 Mask                   */

#define ADC_CHEN_CHEN5_Pos               (5)                                               /*!< ADC_T::CHEN: CHEN5 Position               */
#define ADC_CHEN_CHEN5_Msk               (0x1ul << ADC_CHEN_CHEN5_Pos)                     /*!< ADC_T::CHEN: CHEN5 Mask                   */

#define ADC_CHEN_CHEN6_Pos               (6)                                               /*!< ADC_T::CHEN: CHEN6 Position               */
#define ADC_CHEN_CHEN6_Msk               (0x1ul << ADC_CHEN_CHEN6_Pos)                     /*!< ADC_T::CHEN: CHEN6 Mask                   */

#define ADC_CHEN_CHEN7_Pos               (7)                                               /*!< ADC_T::CHEN: CHEN7 Position               */
#define ADC_CHEN_CHEN7_Msk               (0x1ul << ADC_CHEN_CHEN7_Pos)                     /*!< ADC_T::CHEN: CHEN7 Mask                   */

#define ADC_CHEN_CH7SEL_Pos              (8)                                               /*!< ADC_T::CHEN: CH7SEL Position              */
#define ADC_CHEN_CH7SEL_Msk              (0x1ul << ADC_CHEN_CH7SEL_Pos)                    /*!< ADC_T::CHEN: CH7SEL Mask                  */

#define ADC_CHEN_CHEN8_Pos               (9)                                               /*!< ADC_T::CHEN: CHEN8 Position               */
#define ADC_CHEN_CHEN8_Msk               (0x1ul << ADC_CHEN_CHEN8_Pos)                     /*!< ADC_T::CHEN: CHEN8 Mask                   */

#define ADC_CHEN_CHEN9_Pos               (10)                                              /*!< ADC_T::CHEN: CHEN9 Position               */
#define ADC_CHEN_CHEN9_Msk               (0x1ul << ADC_CHEN_CHEN9_Pos)                     /*!< ADC_T::CHEN: CHEN9 Mask                   */

#define ADC_CHEN_CHEN10_Pos              (11)                                              /*!< ADC_T::CHEN: CHEN10 Position              */
#define ADC_CHEN_CHEN10_Msk              (0x1ul << ADC_CHEN_CHEN10_Pos)                    /*!< ADC_T::CHEN: CHEN10 Mask                  */

#define ADC_CHEN_CHEN11_Pos              (12)                                              /*!< ADC_T::CHEN: CHEN11 Position              */
#define ADC_CHEN_CHEN11_Msk              (0x1ul << ADC_CHEN_CHEN11_Pos)                    /*!< ADC_T::CHEN: CHEN11 Mask                  */

#define ADC_CHEN_BGEN_Pos                (13)                                              /*!< ADC_T::CHEN: BGEN Position                */
#define ADC_CHEN_BGEN_Msk                (0x1ul << ADC_CHEN_BGEN_Pos)                      /*!< ADC_T::CHEN: BGEN Mask                    */

#define ADC_CMP_ADCMPEN_Pos              (0)                                               /*!< ADC_T::CMP: ADCMPEN Position             */
#define ADC_CMP_ADCMPEN_Msk              (0x1ul << ADC_CMP_ADCMPEN_Pos)                    /*!< ADC_T::CMP: ADCMPEN Mask                 */

#define ADC_CMP_ADCMPIE_Pos              (1)                                               /*!< ADC_T::CMP: ADCMPIE Position             */
#define ADC_CMP_ADCMPIE_Msk              (0x1ul << ADC_CMP_ADCMPIE_Pos)                    /*!< ADC_T::CMP: ADCMPIE Mask                 */

#define ADC_CMP_CMPCOND_Pos              (2)                                               /*!< ADC_T::CMP: CMPCOND Position             */
#define ADC_CMP_CMPCOND_Msk              (0x1ul << ADC_CMP_CMPCOND_Pos)                    /*!< ADC_T::CMP: CMPCOND Mask                 */

#define ADC_CMP_CMPCH_Pos                (3)                                               /*!< ADC_T::CMP: CMPCH Position               */
#define ADC_CMP_CMPCH_Msk                (0xful << ADC_CMP_CMPCH_Pos)                      /*!< ADC_T::CMP: CMPCH Mask                   */

#define ADC_CMP_CMPMCNT_Pos              (8)                                               /*!< ADC_T::CMP: CMPMCNT Position             */
#define ADC_CMP_CMPMCNT_Msk              (0xful << ADC_CMP_CMPMCNT_Pos)                    /*!< ADC_T::CMP: CMPMCNT Mask                 */

#define ADC_CMP_CMPDAT_Pos               (16)                                              /*!< ADC_T::CMP: CMPDAT Position              */
#define ADC_CMP_CMPDAT_Msk               (0x3fful << ADC_CMP_CMPDAT_Pos)                   /*!< ADC_T::CMP: CMPDAT Mask                  */

#define ADC_STATUS_ADIF_Pos              (0)                                               /*!< ADC_T::STATUS: ADIF Position              */
#define ADC_STATUS_ADIF_Msk              (0x1ul << ADC_STATUS_ADIF_Pos)                    /*!< ADC_T::STATUS: ADIF Mask                  */

#define ADC_STATUS_ADCMPF0_Pos           (1)                                               /*!< ADC_T::STATUS: ADCMPF0 Position           */
#define ADC_STATUS_ADCMPF0_Msk           (0x1ul << ADC_STATUS_ADCMPF0_Pos)                 /*!< ADC_T::STATUS: ADCMPF0 Mask               */

#define ADC_STATUS_ADCMPF1_Pos           (2)                                               /*!< ADC_T::STATUS: ADCMPF1 Position           */
#define ADC_STATUS_ADCMPF1_Msk           (0x1ul << ADC_STATUS_ADCMPF1_Pos)                 /*!< ADC_T::STATUS: ADCMPF1 Mask               */

#define ADC_STATUS_BUSY_Pos              (3)                                               /*!< ADC_T::STATUS: BUSY Position              */
#define ADC_STATUS_BUSY_Msk              (0x1ul << ADC_STATUS_BUSY_Pos)                    /*!< ADC_T::STATUS: BUSY Mask                  */

#define ADC_STATUS_CHANNEL_Pos           (4)                                               /*!< ADC_T::STATUS: CHANNEL Position           */
#define ADC_STATUS_CHANNEL_Msk           (0x7ul << ADC_STATUS_CHANNEL_Pos)                 /*!< ADC_T::STATUS: CHANNEL Mask               */

#define ADC_STATUS_VALID_Pos             (8)                                               /*!< ADC_T::STATUS: VALID Position             */
#define ADC_STATUS_VALID_Msk             (0x1ul << ADC_STATUS_VALID_Pos)                   /*!< ADC_T::STATUS: VALID Mask                 */

#define ADC_STATUS_OV_Pos                (16)                                              /*!< ADC_T::STATUS: OV Position                */
#define ADC_STATUS_OV_Msk                (0x1ul << ADC_STATUS_OV_Pos)                      /*!< ADC_T::STATUS: OV Mask                    */

#define ADC_TRGDLY_DELAY_Pos             (0)                                               /*!< ADC_T::TRGDLY: DELAY Position             */
#define ADC_TRGDLY_DELAY_Msk             (0xfful << ADC_TRGDLY_DELAY_Pos)                  /*!< ADC_T::TRGDLY: DELAY Mask                 */

#define ADC_EXTSMPT_EXTSMPT_Pos          (0)                                               /*!< ADC_T::EXTSMPT: EXTSMPT Position          */
#define ADC_EXTSMPT_EXTSMPT_Msk          (0xful << ADC_EXTSMPT_EXTSMPT_Pos)                /*!< ADC_T::EXTSMPT: EXTSMPT Mask              */

#define ADC_SEQCTL_SEQEN_Pos             (0)                                               /*!< ADC_T::SEQCTL: SEQEN Position             */
#define ADC_SEQCTL_SEQEN_Msk             (0x1ul << ADC_SEQCTL_SEQEN_Pos)                   /*!< ADC_T::SEQCTL: SEQEN Mask                 */

#define ADC_SEQCTL_SEQTYPE_Pos           (1)                                               /*!< ADC_T::SEQCTL: SEQTYPE Position           */
#define ADC_SEQCTL_SEQTYPE_Msk           (0x1ul << ADC_SEQCTL_SEQTYPE_Pos)                 /*!< ADC_T::SEQCTL: SEQTYPE Mask               */

#define ADC_SEQCTL_MODESEL_Pos           (2)                                               /*!< ADC_T::SEQCTL: MODESEL Position           */
#define ADC_SEQCTL_MODESEL_Msk           (0x3ul << ADC_SEQCTL_MODESEL_Pos)                 /*!< ADC_T::SEQCTL: MODESEL Mask               */

#define ADC_SEQCTL_TRG1TYPE_Pos          (8)                                               /*!< ADC_T::SEQCTL: TRG1TYPE Position          */
#define ADC_SEQCTL_TRG1TYPE_Msk          (0x3ul << ADC_SEQCTL_TRG1TYPE_Pos)                /*!< ADC_T::SEQCTL: TRG1TYPE Mask              */

#define ADC_SEQCTL_TRG1SRC_Pos           (10)                                              /*!< ADC_T::SEQCTL: TRG1SRC Position           */
#define ADC_SEQCTL_TRG1SRC_Msk           (0x3ul << ADC_SEQCTL_TRG1SRC_Pos)                 /*!< ADC_T::SEQCTL: TRG1SRC Mask               */

#define ADC_SEQCTL_TRG2TYPE_Pos          (16)                                              /*!< ADC_T::SEQCTL: TRG2TYPE Position          */
#define ADC_SEQCTL_TRG2TYPE_Msk          (0x3ul << ADC_SEQCTL_TRG2TYPE_Pos)                /*!< ADC_T::SEQCTL: TRG2TYPE Mask              */

#define ADC_SEQCTL_TRG2SRC_Pos           (18)                                              /*!< ADC_T::SEQCTL: TRG2SRC Position           */
#define ADC_SEQCTL_TRG2SRC_Msk           (0x3ul << ADC_SEQCTL_TRG2SRC_Pos)                 /*!< ADC_T::SEQCTL: TRG2SRC Mask               */

#define ADC_SEQDAT0_RESULT_Pos           (0)                                               /*!< ADC_T::SEQDAT0: RESULT Position           */
#define ADC_SEQDAT0_RESULT_Msk           (0x3fful << ADC_SEQDAT0_RESULT_Pos)               /*!< ADC_T::SEQDAT0: RESULT Mask               */

#define ADC_SEQDAT0_OV_Pos               (16)                                              /*!< ADC_T::SEQDAT0: OV Position               */
#define ADC_SEQDAT0_OV_Msk               (0x1ul << ADC_SEQDAT0_OV_Pos)                     /*!< ADC_T::SEQDAT0: OV Mask                   */

#define ADC_SEQDAT0_VALID_Pos            (17)                                              /*!< ADC_T::SEQDAT0: VALID Position            */
#define ADC_SEQDAT0_VALID_Msk            (0x1ul << ADC_SEQDAT0_VALID_Pos)                  /*!< ADC_T::SEQDAT0: VALID Mask                */

#define ADC_SEQDAT1_RESULT_Pos           (0)                                               /*!< ADC_T::SEQDAT1: RESULT Position           */
#define ADC_SEQDAT1_RESULT_Msk           (0x3fful << ADC_SEQDAT1_RESULT_Pos)               /*!< ADC_T::SEQDAT1: RESULT Mask               */

#define ADC_SEQDAT1_OV_Pos               (16)                                              /*!< ADC_T::SEQDAT1: OV Position               */
#define ADC_SEQDAT1_OV_Msk               (0x1ul << ADC_SEQDAT1_OV_Pos)                     /*!< ADC_T::SEQDAT1: OV Mask                   */

#define ADC_SEQDAT1_VALID_Pos            (17)                                              /*!< ADC_T::SEQDAT1: VALID Position            */
#define ADC_SEQDAT1_VALID_Msk            (0x1ul << ADC_SEQDAT1_VALID_Pos)                  /*!< ADC_T::SEQDAT1: VALID Mask                */

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
     * PWRCTL
     * ===================================================================================================
     * Offset: 0x00  System Power-down Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |XTLEN     |External HXT Or LXT Crystal Oscillator Enable Control (Write Protect)
     * |        |          |The default clock source is from HIRC.
     * |        |          |These two bits are default set to "00" and the XTAL1 and XTAL2 pins are GPIO.
     * |        |          |00 = XTAL1 and XTAL2 are GPIO, disable both LXT & HXT (default).
     * |        |          |01 = HXT Enabled.
     * |        |          |10 = LXT Enabled.
     * |        |          |11 = XTAL1 is external clock input pin, XTAL2 is GPIO.
     * |        |          |Note: To enable external XTAL function, P5_ALT[1:0] and P5_MFP[1:0] bits must also be set in P5_MFP.
     * |[2]     |HIRCEN    |Internal High Speed RC Oscillator (HIRC) Enable Control (Write Protect)
     * |        |          |0 = Internal high speed RC oscillator (HIRC) Disabled.
     * |        |          |1 = Internal high speed RC oscillator (HIRC) Enabled.
     * |        |          |Note: The default of HIRCEN bit is 1.
     * |[3]     |LIRCEN    |10 KHz Internal Low Speed RC Oscillator (LIRC) Enable Control (Write Protest)
     * |        |          |0 = 10 kHz internal low speed RC oscillator (LIRC) Disabled.
     * |        |          |1 = 10 kHz internal low speed RC oscillator (LIRC) Enabled.
     * |[4]     |PDWKDLY   |Wake-Up Delay Counter Enable Control (Write Protect)
     * |        |          |When the chip wakes up from Power-down mode, the clock control will delay certain clock cycles to wait system clock stable.
     * |        |          |The delayed clock cycle is 4096 clock cycles when chip work at 4~24 MHz external high speed crystal (HXT), 4096 clock cycles for 32.768 kHz external low speed crystal (LXT), and 16 clock cycles when chip works at 22.1184 MHz internal high speed RC oscillator (HIRC).
     * |        |          |0 = Clock cycles delay Disabled.
     * |        |          |1 = Clock cycles delay Enabled.
     * |[5]     |PDWKIEN   |Power-Down Mode Wake-Up Interrupt Enable Control (Write Protect)
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: The interrupt will occur when both PDWKIF and PDWKIEN are high.
     * |[6]     |PDWKIF    |Power-Down Mode Wake-Up Interrupt Status
     * |        |          |Set by "Power-down wake-up event", which indicates that resume from Power-down mode"
     * |        |          |The flag is set if the GPIO, UART, WDT, ACMP, Timer or BOD wake-up occurred.
     * |        |          |Note: This bit works only if PDWKIEN (CLK_PWRCTL[5]) set to 1. Write 1 to clear the bit to 0.
     * |[7]     |PDEN      |System Power-Down Enable Bit (Write Protect)
     * |        |          |When chip wakes up from Power-down mode, this bit is cleared by hardware.
     * |        |          |User needs to set this bit again for next Power-down.
     * |        |          |In Power-down mode, 4~24 MHz external high speed crystal oscillator (HXT), 32.768 kHz external low speed crystal oscillator (LXT), and the 22.1184 MHz internal high speed oscillator (HIRC) will be disabled in this mode, and 10 kHz internal low speed RC oscillator (LIRC) are not controlled by Power-down mode.
     * |        |          |In Power-down mode, the system clock is disabled, and ignored the clock source selection.
     * |        |          |The clocks of peripheral are not controlled by Power-down mode, if the peripheral clock source is from 10 kHz internal low speed oscillator.
     * |        |          |0 = Chip operating normally or chip in Idle mode because of WFI command.
     * |        |          |1 = Chip enters Power-down mode instantly or waits CPU sleep command WFI.
     * |[9]     |PDLXT     |Enable LXT In Power-Down Mode
     * |        |          |This bit controls the crystal oscillator active or not in Power-down mode.
     * |        |          |0 = No effect to Power-down mode.
     * |        |          |1 = If XTLEN[1:0] = 10, LXT is still active in Power-down mode.
     * |[11:10] |HXTGAIN   |HXT Gain Selection
     * |        |          |00 = Full gain for the frequency up to 24MHz.
     * |        |          |01 = 3/4 gain for the frequency up to 16MHz.
     * |        |          |10 = 1/2 gain for the frequency up to 12MHz.
     * |        |          |11 = 1/4 gain for the frequency up to 4MHz.
    */
    __IO uint32_t PWRCTL;

    /**
     * AHBCLK
     * ===================================================================================================
     * Offset: 0x04  AHB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |ISPCKEN   |Flash ISP Controller Clock Enable Control
     * |        |          |0 = Flash ISP peripheral clock Disabled.
     * |        |          |1 = Flash ISP peripheral clock Enabled.
         * |[4]     |HDIVCKEN  |Hardware Divider Clock Enable Control
     * |        |          |0 = Hardware Divider clock Disabled.
     * |        |          |1 = Hardware Divider clock Enabled.
    */
    __IO uint32_t AHBCLK;

    /**
     * APBCLK
     * ===================================================================================================
     * Offset: 0x08  APB Devices Clock Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WDTCKEN   |Watchdog Timer Clock Enable Control (Write Protect)
     * |        |          |0 = Watchdog Timer clock Disabled.
     * |        |          |1 = Watchdog Timer clock Enabled.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address SYS_BA + 0x100.
     * |[2]     |TMR0CKEN  |Timer0 Clock Enable Control
     * |        |          |0 = Timer0 clock Disabled.
     * |        |          |1 = Timer0 clock Enabled.
     * |[3]     |TMR1CKEN  |Timer1 Clock Enable Control
     * |        |          |0 = Timer1 clock Disabled.
     * |        |          |1 = Timer1 clock Enabled.
     * |[6]     |CLKOCKEN  |Frequency Divider Output Clock Enable Control
     * |        |          |0 = FDIV clock Disabled.
     * |        |          |1 = FDIV clock Enabled.
     * |[8]     |I2CCKEN   |I2C Clock Enable Control
     * |        |          |0 = I2C clock Disabled.
     * |        |          |1 = I2C clock Enabled.
     * |[12]    |SPICKEN   |SPI Peripheral Clock Enable Control
     * |        |          |0 = SPI peripheral clock Disabled.
     * |        |          |1 = SPI peripheral clock Enabled.
     * |[16]    |UART0CKEN |UART0 Clock Enable Control
     * |        |          |0 = UART0 clock Disabled.
     * |        |          |1 = UART0 clock Enabled.
     * |[17]    |UART1CKEN |UART1 Clock Enable Control
     * |        |          |0 = UART1 clock Disabled.
     * |        |          |1 = UART1 clock Enabled.
     * |[20]    |PWMCH01CKEN|PWM_01 Clock Enable Control
     * |        |          |0 = PWM01 clock Disabled.
     * |        |          |1 = PWM01 clock Enabled.
     * |[21]    |PWMCH23CKEN|PWM_23 Clock Enable Control
     * |        |          |0 = PWM23 clock Disabled.
     * |        |          |1 = PWM23 clock Enabled.
     * |[22]    |PWMCH45CKEN|PWM_45 Clock Enable Control
     * |        |          |0 = PWM45 clock Disabled.
     * |        |          |1 = PWM45 clock Enabled.
     * |[28]    |ADCCKEN   |Analog-Digital-Converter (ADC) Clock Enable Control
     * |        |          |0 = ADC peripheral clock Disabled.
     * |        |          |1 = ADC peripheral clock Enabled.
     * |[30]    |ACMPCKEN  |Analog Comparator Clock Enable Control
     * |        |          |0 = Analog Comparator clock Disabled.
     * |        |          |1 = Analog Comparator clock Enabled.
    */
    __IO uint32_t APBCLK;

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x0C  Clock Status Monitor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |XTLSTB    |HXT Or LXT Clock Source Stable Flag
     * |        |          |0 = HXT or LXT clock is not stable or disabled.
     * |        |          |1 = HXT or LXT clock is stable.
     * |[3]     |LIRCSTB   |LIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = LIRC clock is not stable or disabled.
     * |        |          |1 = LIRC clock is stable.
     * |[4]     |HIRCSTB   |HIRC Clock Source Stable Flag (Read Only)
     * |        |          |0 = HIRC clock is not stable or disabled.
     * |        |          |1 = HIRC clock is stable.
     * |[7]     |CLKSFAIL  |Clock Switch Fail Flag
     * |        |          |0 = Clock switching success.
     * |        |          |1 = Clock switching failed.
     * |        |          |Note1: This bit is updated when software switches system clock source.
     * |        |          |If switch target clock is stable, this bit will be set to 0.
     * |        |          |If switch target clock is not stable, this bit will be set to 1.
     * |        |          |Note2: This bit is read only.
     * |        |          |After selected clock source is stable, hardware will switch system clock to selected clock automatically, and CLKSFAIL will be cleared automatically by hardware.
    */
    __IO uint32_t STATUS;

    /**
     * CLKSEL0
     * ===================================================================================================
     * Offset: 0x10  Clock Source Select Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |HCLKSEL   |HCLK Clock Source Selection (Write Protect)
     * |        |          |000 = Clock source is from HXT or LXT.
     * |        |          |001 = Reserved.
     * |        |          |010 = Reserved.
     * |        |          |011 = Clock source is from LIRC.
     * |        |          |111 = Clock source is from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note1: Before clock switching, the related clock sources (both pre-select and new-select) must be turn-on and stable.
     * |        |          |Note2: These bits are protected bit, and programming them needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address SYS_BA + 0x100.
     * |        |          |Note3: To set CLK_PWRCTL[1:0] to select HXT or LXT crystal clock.
     * |[5:3]   |STCLKSEL  |Cortex-M0 SysTick Clock Source Selection From Reference Clock (Write Protect)
     * |        |          |If SYST_CSR[2] = 1, SysTick clock source is from HCLK.
     * |        |          |If SYST_CSR[2] = 0, SysTick clock source is defined by below settings.
     * |        |          |000 = Clock source is from HXT or LXT.
     * |        |          |001 = Reserved.
     * |        |          |010 = Clock source is from HXT/2 or LXT/2.
     * |        |          |011 = Clock source is from HCLK/2.
     * |        |          |111 = Clock source is from HIRC /2.
     * |        |          |Others = Reserved.
     * |        |          |Note1: These bits are protected bit, and programming them needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address SYS_BA + 0x100.
     * |        |          |Note2: If the SysTick clock source is not from HCLK (i.e.
     * |        |          |SYST_CSR[2] = 0), SysTick clock source must less than or equal to HCLK/2.
     * |        |          |Note3: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
    */
    __IO uint32_t CLKSEL0;

    /**
     * CLKSEL1
     * ===================================================================================================
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |WDTSEL    |WDT CLK Clock Source Selection (Write Protect)
     * |        |          |00 = Clock source is from HXT or LXT.
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source is from HCLK/2048 clock.
     * |        |          |11 = Clock source is from LIRC.
     * |        |          |Note1: These bits are the protected bit, and programming them needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address SYS_BA + 0x100.
     * |        |          |Note2: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
     * |[3:2]   |ADCSEL    |ADC Peripheral Clock Source Selection
     * |        |          |00 = Clock source is from HXT or LXT.
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source is from HCLK.
     * |        |          |11 = Clock source is from HIRC.
     * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
     * |[4]     |SPISEL    |SPI Clock Source Selection
     * |        |          |0 = Clock source is from HXT or LXT.
     * |        |          |1 = Clock source is from HCLK.
     * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
     * |[10:8]  |TMR0SEL   |TIMER0 Clock Source Selection
     * |        |          |000 = Clock source is from HXT or LXT.
     * |        |          |001 = Clock source is from LIRC.
     * |        |          |010 = Clock source is from HCLK.
     * |        |          |011 = Clock source is from external trigger.
     * |        |          |111 = Clock source is from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
     * |[14:12] |TMR1SEL   |TIMER1 Clock Source Selection
     * |        |          |000 = Clock source is from HXT or LXT.
     * |        |          |001 = Clock source is from LIRC.
     * |        |          |010 = Clock source is from HCLK.
     * |        |          |011 = Clock source is from external trigger.
     * |        |          |111 = Clock source is from HIRC.
     * |        |          |Others = Reserved.
     * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
     * |[25:24] |UART0SEL  |UART0 Clock Source Selection
     * |        |          |00 = Clock source is from HXT or LXT.
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source is from HIRC.
     * |        |          |11 = Clock source is from HIRC .
     * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
     * |[27:26] |UART1SEL  |UART1 Clock Source Selection
     * |        |          |00 = Clock source from HXT or LXT crystal clock.
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source from HIRC oscillator clock.
     * |        |          |11 = Reserved.
     * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
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
     * |        |          |HCLK clock frequency = (HCLK clock source frequency) / (HCLKDIV + 1).
     * |[11:8]  |UART0DIV  |UART0 Clock Divide Number From UART0 Clock Source
     * |        |          |UART0 clock frequency = (UART0 clock source frequency) / (UART_N + 1).
     * |[15:12] |UART1DIV  |UART1 Clock Divide Number From UART1 Clock Source
     * |        |          |UART1 clock frequency = (UART1 clock source frequency) / (UART_N + 1).
     * |[23:16] |ADCDIV    |ADC Peripheral Clock Divide Number From ADC Peripheral Clock Source
     * |        |          |ADC peripheral clock frequency = (ADC peripheral clock source frequency) / (ADCDIV + 1).
    */
    __IO uint32_t CLKDIV;

    /**
     * CLKSEL2
     * ===================================================================================================
     * Offset: 0x1C  Clock Source Select Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:2]   |FDIVSEL   |Clock Divider Clock Source Selection
     * |        |          |00 = Clock source is from HXT or LXT.
     * |        |          |01 = Reserved.
     * |        |          |10 = Clock source is from HCLK.
     * |        |          |11 = Clock source is from HIRC.
     * |        |          |Note: To set CLK_PWRCTL[1:0], select HXT or LXT crystal clock.
    */
    __IO uint32_t CLKSEL2;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED0[1];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * CLKOCTL
     * ===================================================================================================
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
     * |[4]     |CLKOEN    |Frequency Divider Enable Control
     * |        |          |0 = Frequency Divider Disabled.
     * |        |          |1 = Frequency Divider Enabled.
     * |[5]     |DIV1EN    |Frequency Divider 1 Enable Control
     * |        |          |0 = Divider output frequency is depended on FSEL value.
     * |        |          |1 = Divider output frequency is the same as input clock frequency.
    */
    __IO uint32_t CLKOCTL;

} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_PWRCTL_XTLEN_Pos             (0)                                               /*!< CLK_T::PWRCTL: XTLEN Position             */
#define CLK_PWRCTL_XTLEN_Msk             (0x3ul << CLK_PWRCTL_XTLEN_Pos)                   /*!< CLK_T::PWRCTL: XTLEN Mask                 */

#define CLK_PWRCTL_HIRCEN_Pos            (2)                                               /*!< CLK_T::PWRCTL: HIRCEN Position            */
#define CLK_PWRCTL_HIRCEN_Msk            (0x1ul << CLK_PWRCTL_HIRCEN_Pos)                  /*!< CLK_T::PWRCTL: HIRCEN Mask                */

#define CLK_PWRCTL_LIRCEN_Pos            (3)                                               /*!< CLK_T::PWRCTL: LIRCEN Position            */
#define CLK_PWRCTL_LIRCEN_Msk            (0x1ul << CLK_PWRCTL_LIRCEN_Pos)                  /*!< CLK_T::PWRCTL: LIRCEN Mask                */

#define CLK_PWRCTL_PDWKDLY_Pos           (4)                                               /*!< CLK_T::PWRCTL: PDWKDLY Position           */
#define CLK_PWRCTL_PDWKDLY_Msk           (0x1ul << CLK_PWRCTL_PDWKDLY_Pos)                 /*!< CLK_T::PWRCTL: PDWKDLY Mask               */

#define CLK_PWRCTL_PDWKIEN_Pos           (5)                                               /*!< CLK_T::PWRCTL: PDWKIEN Position           */
#define CLK_PWRCTL_PDWKIEN_Msk           (0x1ul << CLK_PWRCTL_PDWKIEN_Pos)                 /*!< CLK_T::PWRCTL: PDWKIEN Mask               */

#define CLK_PWRCTL_PDWKIF_Pos            (6)                                               /*!< CLK_T::PWRCTL: PDWKIF Position            */
#define CLK_PWRCTL_PDWKIF_Msk            (0x1ul << CLK_PWRCTL_PDWKIF_Pos)                  /*!< CLK_T::PWRCTL: PDWKIF Mask                */

#define CLK_PWRCTL_PDEN_Pos              (7)                                               /*!< CLK_T::PWRCTL: PDEN Position              */
#define CLK_PWRCTL_PDEN_Msk              (0x1ul << CLK_PWRCTL_PDEN_Pos)                    /*!< CLK_T::PWRCTL: PDEN Mask                  */

#define CLK_PWRCTL_PDLXT_Pos             (9)                                               /*!< CLK_T::PWRCTL: PDLXT Position             */
#define CLK_PWRCTL_PDLXT_Msk             (0x1ul << CLK_PWRCTL_PDLXT_Pos)                   /*!< CLK_T::PWRCTL: PDLXT Mask                 */

#define CLK_PWRCTL_HXTGAIN_Pos           (10)                                              /*!< CLK_T::PWRCTL: HXTGAIN Position           */
#define CLK_PWRCTL_HXTGAIN_Msk           (0x3ul << CLK_PWRCTL_HXTGAIN_Pos)                 /*!< CLK_T::PWRCTL: HXTGAIN Mask               */

#define CLK_AHBCLK_ISPCKEN_Pos           (2)                                               /*!< CLK_T::AHBCLK: ISPCKEN Position           */
#define CLK_AHBCLK_ISPCKEN_Msk           (0x1ul << CLK_AHBCLK_ISPCKEN_Pos)                 /*!< CLK_T::AHBCLK: ISPCKEN Mask               */

#define CLK_AHBCLK_HDIVCKEN_Pos          (4)                                               /*!< CLK_T::AHBCLK: HDIVCKEN Position          */
#define CLK_AHBCLK_HDIVCKEN_Msk          (0x1ul << CLK_AHBCLK_HDIVCKEN_Pos)                /*!< CLK_T::AHBCLK: HDIVCKEN Mask              */

#define CLK_APBCLK_WDTCKEN_Pos           (0)                                               /*!< CLK_T::APBCLK: WDTCKEN Position           */
#define CLK_APBCLK_WDTCKEN_Msk           (0x1ul << CLK_APBCLK_WDTCKEN_Pos)                 /*!< CLK_T::APBCLK: WDTCKEN Mask               */

#define CLK_APBCLK_TMR0CKEN_Pos          (2)                                               /*!< CLK_T::APBCLK: TMR0CKEN Position          */
#define CLK_APBCLK_TMR0CKEN_Msk          (0x1ul << CLK_APBCLK_TMR0CKEN_Pos)                /*!< CLK_T::APBCLK: TMR0CKEN Mask              */

#define CLK_APBCLK_TMR1CKEN_Pos          (3)                                               /*!< CLK_T::APBCLK: TMR1CKEN Position          */
#define CLK_APBCLK_TMR1CKEN_Msk          (0x1ul << CLK_APBCLK_TMR1CKEN_Pos)                /*!< CLK_T::APBCLK: TMR1CKEN Mask              */

#define CLK_APBCLK_CLKOCKEN_Pos          (6)                                               /*!< CLK_T::APBCLK: CLKOCKEN Position          */
#define CLK_APBCLK_CLKOCKEN_Msk          (0x1ul << CLK_APBCLK_CLKOCKEN_Pos)                /*!< CLK_T::APBCLK: CLKOCKEN Mask              */

#define CLK_APBCLK_I2CCKEN_Pos           (8)                                               /*!< CLK_T::APBCLK: I2CCKEN Position           */
#define CLK_APBCLK_I2CCKEN_Msk           (0x1ul << CLK_APBCLK_I2CCKEN_Pos)                 /*!< CLK_T::APBCLK: I2CCKEN Mask               */

#define CLK_APBCLK_SPICKEN_Pos           (12)                                              /*!< CLK_T::APBCLK: SPICKEN Position           */
#define CLK_APBCLK_SPICKEN_Msk           (0x1ul << CLK_APBCLK_SPICKEN_Pos)                 /*!< CLK_T::APBCLK: SPICKEN Mask               */

#define CLK_APBCLK_UART0CKEN_Pos         (16)                                              /*!< CLK_T::APBCLK: UART0CKEN Position         */
#define CLK_APBCLK_UART0CKEN_Msk         (0x1ul << CLK_APBCLK_UART0CKEN_Pos)               /*!< CLK_T::APBCLK: UART0CKEN Mask             */

#define CLK_APBCLK_UART1CKEN_Pos         (17)                                              /*!< CLK_T::APBCLK: UART1CKEN Position         */
#define CLK_APBCLK_UART1CKEN_Msk         (0x1ul << CLK_APBCLK_UART1CKEN_Pos)               /*!< CLK_T::APBCLK: UART1CKEN Mask             */

#define CLK_APBCLK_PWMCH01CKEN_Pos       (20)                                              /*!< CLK_T::APBCLK: PWMCH01CKEN Position       */
#define CLK_APBCLK_PWMCH01CKEN_Msk       (0x1ul << CLK_APBCLK_PWMCH01CKEN_Pos)             /*!< CLK_T::APBCLK: PWMCH01CKEN Mask           */

#define CLK_APBCLK_PWMCH23CKEN_Pos       (21)                                              /*!< CLK_T::APBCLK: PWMCH23CKEN Position       */
#define CLK_APBCLK_PWMCH23CKEN_Msk       (0x1ul << CLK_APBCLK_PWMCH23CKEN_Pos)             /*!< CLK_T::APBCLK: PWMCH23CKEN Mask           */

#define CLK_APBCLK_PWMCH45CKEN_Pos       (22)                                              /*!< CLK_T::APBCLK: PWMCH45CKEN Position       */
#define CLK_APBCLK_PWMCH45CKEN_Msk       (0x1ul << CLK_APBCLK_PWMCH45CKEN_Pos)             /*!< CLK_T::APBCLK: PWMCH45CKEN Mask           */

#define CLK_APBCLK_ADCCKEN_Pos           (28)                                              /*!< CLK_T::APBCLK: ADCCKEN Position           */
#define CLK_APBCLK_ADCCKEN_Msk           (0x1ul << CLK_APBCLK_ADCCKEN_Pos)                 /*!< CLK_T::APBCLK: ADCCKEN Mask               */

#define CLK_APBCLK_ACMPCKEN_Pos          (30)                                              /*!< CLK_T::APBCLK: ACMPCKEN Position          */
#define CLK_APBCLK_ACMPCKEN_Msk          (0x1ul << CLK_APBCLK_ACMPCKEN_Pos)                /*!< CLK_T::APBCLK: ACMPCKEN Mask              */

#define CLK_STATUS_XTLSTB_Pos            (0)                                               /*!< CLK_T::STATUS: XTLSTB Position            */
#define CLK_STATUS_XTLSTB_Msk            (0x1ul << CLK_STATUS_XTLSTB_Pos)                  /*!< CLK_T::STATUS: XTLSTB Mask                */

#define CLK_STATUS_LIRCSTB_Pos           (3)                                               /*!< CLK_T::STATUS: LIRCSTB Position           */
#define CLK_STATUS_LIRCSTB_Msk           (0x1ul << CLK_STATUS_LIRCSTB_Pos)                 /*!< CLK_T::STATUS: LIRCSTB Mask               */

#define CLK_STATUS_HIRCSTB_Pos           (4)                                               /*!< CLK_T::STATUS: HIRCSTB Position           */
#define CLK_STATUS_HIRCSTB_Msk           (0x1ul << CLK_STATUS_HIRCSTB_Pos)                 /*!< CLK_T::STATUS: HIRCSTB Mask               */

#define CLK_STATUS_CLKSFAIL_Pos          (7)                                               /*!< CLK_T::STATUS: CLKSFAIL Position          */
#define CLK_STATUS_CLKSFAIL_Msk          (0x1ul << CLK_STATUS_CLKSFAIL_Pos)                /*!< CLK_T::STATUS: CLKSFAIL Mask              */

#define CLK_CLKSEL0_HCLKSEL_Pos          (0)                                               /*!< CLK_T::CLKSEL0: HCLKSEL Position          */
#define CLK_CLKSEL0_HCLKSEL_Msk          (0x7ul << CLK_CLKSEL0_HCLKSEL_Pos)                /*!< CLK_T::CLKSEL0: HCLKSEL Mask              */

#define CLK_CLKSEL0_STCLKSEL_Pos         (3)                                               /*!< CLK_T::CLKSEL0: STCLKSEL Position         */
#define CLK_CLKSEL0_STCLKSEL_Msk         (0x7ul << CLK_CLKSEL0_STCLKSEL_Pos)               /*!< CLK_T::CLKSEL0: STCLKSEL Mask             */

#define CLK_CLKSEL1_WDTSEL_Pos           (0)                                               /*!< CLK_T::CLKSEL1: WDTSEL Position           */
#define CLK_CLKSEL1_WDTSEL_Msk           (0x3ul << CLK_CLKSEL1_WDTSEL_Pos)                 /*!< CLK_T::CLKSEL1: WDTSEL Mask               */

#define CLK_CLKSEL1_ADCSEL_Pos           (2)                                               /*!< CLK_T::CLKSEL1: ADCSEL Position           */
#define CLK_CLKSEL1_ADCSEL_Msk           (0x3ul << CLK_CLKSEL1_ADCSEL_Pos)                 /*!< CLK_T::CLKSEL1: ADCSEL Mask               */

#define CLK_CLKSEL1_SPISEL_Pos           (4)                                               /*!< CLK_T::CLKSEL1: SPISEL Position           */
#define CLK_CLKSEL1_SPISEL_Msk           (0x1ul << CLK_CLKSEL1_SPISEL_Pos)                 /*!< CLK_T::CLKSEL1: SPISEL Mask               */

#define CLK_CLKSEL1_TMR0SEL_Pos          (8)                                               /*!< CLK_T::CLKSEL1: TMR0SEL Position          */
#define CLK_CLKSEL1_TMR0SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR0SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR0SEL Mask              */

#define CLK_CLKSEL1_TMR1SEL_Pos          (12)                                              /*!< CLK_T::CLKSEL1: TMR1SEL Position          */
#define CLK_CLKSEL1_TMR1SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR1SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR1SEL Mask              */

#define CLK_CLKSEL1_UART0SEL_Pos         (24)                                              /*!< CLK_T::CLKSEL1: UART0SEL Position         */
#define CLK_CLKSEL1_UART0SEL_Msk         (0x3ul << CLK_CLKSEL1_UART0SEL_Pos)               /*!< CLK_T::CLKSEL1: UART0SEL Mask             */

#define CLK_CLKSEL1_UART1SEL_Pos         (26)                                              /*!< CLK_T::CLKSEL1: UART1SEL Position         */
#define CLK_CLKSEL1_UART1SEL_Msk         (0x3ul << CLK_CLKSEL1_UART1SEL_Pos)               /*!< CLK_T::CLKSEL1: UART1SEL Mask             */

#define CLK_CLKDIV_HCLKDIV_Pos           (0)                                               /*!< CLK_T::CLKDIV: HCLKDIV Position           */
#define CLK_CLKDIV_HCLKDIV_Msk           (0xful << CLK_CLKDIV_HCLKDIV_Pos)                 /*!< CLK_T::CLKDIV: HCLKDIV Mask               */

#define CLK_CLKDIV_UART0DIV_Pos          (8)                                               /*!< CLK_T::CLKDIV: UART0DIV Position          */
#define CLK_CLKDIV_UART0DIV_Msk          (0xful << CLK_CLKDIV_UART0DIV_Pos)                /*!< CLK_T::CLKDIV: UART0DIV Mask              */

#define CLK_CLKDIV_UART1DIV_Pos          (12)                                              /*!< CLK_T::CLKDIV: UART1DIV Position          */
#define CLK_CLKDIV_UART1DIV_Msk          (0xful << CLK_CLKDIV_UART1DIV_Pos)                /*!< CLK_T::CLKDIV: UART1DIV Mask              */

#define CLK_CLKDIV_ADCDIV_Pos            (16)                                              /*!< CLK_T::CLKDIV: ADCDIV Position            */
#define CLK_CLKDIV_ADCDIV_Msk            (0xfful << CLK_CLKDIV_ADCDIV_Pos)                 /*!< CLK_T::CLKDIV: ADCDIV Mask                */

#define CLK_CLKSEL2_FDIVSEL_Pos          (2)                                               /*!< CLK_T::CLKSEL2: FDIVSEL Position          */
#define CLK_CLKSEL2_FDIVSEL_Msk          (0x3ul << CLK_CLKSEL2_FDIVSEL_Pos)                /*!< CLK_T::CLKSEL2: FDIVSEL Mask              */

#define CLK_CLKOCTL_FSEL_Pos             (0)                                               /*!< CLK_T::CLKOCTL: FSEL Position             */
#define CLK_CLKOCTL_FSEL_Msk             (0xful << CLK_CLKOCTL_FSEL_Pos)                   /*!< CLK_T::CLKOCTL: FSEL Mask                 */

#define CLK_CLKOCTL_CLKOEN_Pos           (4)                                               /*!< CLK_T::CLKOCTL: CLKOEN Position           */
#define CLK_CLKOCTL_CLKOEN_Msk           (0x1ul << CLK_CLKOCTL_CLKOEN_Pos)                 /*!< CLK_T::CLKOCTL: CLKOEN Mask               */

#define CLK_CLKOCTL_DIV1EN_Pos           (5)                                               /*!< CLK_T::CLKOCTL: DIV1EN Position           */
#define CLK_CLKOCTL_DIV1EN_Msk           (0x1ul << CLK_CLKOCTL_DIV1EN_Pos)                 /*!< CLK_T::CLKOCTL: DIV1EN Mask               */

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
     * ISPCTL
     * ===================================================================================================
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
     * |        |          |This bit is initiated with the inversed value of CBS in CONFIG0 after any reset is happened except CPU reset (CPURF is 1) or system reset (SYSRF) is happened.
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
     * |        |          |(1) APROM writes to itself if APUEN is set to 0.
     * |        |          |(2) LDROM writes to itself.
     * |        |          |(3) Destination address is illegal, such as over an available range.
     * |        |          |Note: Write 1 to clear this bit to 0.
    */
    __IO uint32_t ISPCTL;

    /**
     * ISPADDR
     * ===================================================================================================
     * Offset: 0x04  ISP Address Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPADR    |ISP Address
     * |        |          |The NuMicro Mini55 series supports word program only. ISPADR[1:0] must be kept 00 for ISP operation.
    */
    __IO uint32_t ISPADDR;

    /**
     * ISPDAT
     * ===================================================================================================
     * Offset: 0x08  ISP Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |ISPDAT    |ISP Data
     * |        |          |Write data to this register before ISP program operation.
     * |        |          |Read data from this register after ISP read operation.
    */
    __IO uint32_t ISPDAT;

    /**
     * ISPCMD
     * ===================================================================================================
     * Offset: 0x0C  ISP Command Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[5:0]   |CMD       |ISP Command
     * |        |          |ISP commands are shown below:
     * |        |          |0x00 = Read.
     * |        |          |0x04 = Read Unique ID.
     * |        |          |0x0B = Read Company ID (0xDA).
     * |        |          |0x21 = Program.
     * |        |          |0x22 = Page Erase.
     * |        |          |0x2E = Set Vector Page Re-Map.
    */
    __IO uint32_t ISPCMD;

    /**
     * ISPTRG
     * ===================================================================================================
     * Offset: 0x10  ISP Trigger Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ISPGO     |ISP Start Trigger (Write Protect)
     * |        |          |Write 1 to start ISP operation and this bit will be cleared to 0 by hardware automatically when ISP operation is finished.
     * |        |          |0 = ISP operation is finished.
     * |        |          |1 = ISP operation is progressed.
    */
    __IO uint32_t ISPTRG;

    /**
     * DFBA
     * ===================================================================================================
     * Offset: 0x14  Data Flash Start Address
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DFBA      |Data Flash Base Address
     * |        |          |This register indicates data flash start address. It is a read only register.
     * |        |          |The data flash start address is defined by user.
     * |        |          |Since on chip flash erase unit is 512 bytes, it is mandatory to keep bit 8-0 as 0.
    */
    __I  uint32_t DFBA;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED0[10];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * ISPSTS
     * ===================================================================================================
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
     * |[6]     |ISPFF     |ISP Fail Flag (Write-Protection Bit)
     * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
     * |        |          |(1) APROM writes to itself if APUEN is set to 0.
     * |        |          |(2) LDROM writes to itself if LDUEN is set to 0.
     * |        |          |(3) CONFIG is erased/programmed if CFGUEN is set to 0.
     * |        |          |(4) Destination address is illegal, such as over an available range.
     * |        |          |Write 1 to clear.
     * |        |          |Note: This bit functions the same as FMC_ISPCTL bit 6.
     * |[20:9]  |VECMAP    |Vector Page Mapping Address (Read Only)
     * |        |          |The current flash address space 0x0000_0000~0x0000_01FF is mapping to address {VECMAP[11:0], 9'h000} ~ {VECMAP[11:0], 9'h1FF}.
    */
    __I  uint32_t ISPSTS;

} FMC_T;

/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for FMC Controller
@{ */

#define FMC_ISPCTL_ISPEN_Pos             (0)                                               /*!< FMC_T::ISPCTL: ISPEN Position             */
#define FMC_ISPCTL_ISPEN_Msk             (0x1ul << FMC_ISPCTL_ISPEN_Pos)                   /*!< FMC_T::ISPCTL: ISPEN Mask                 */

#define FMC_ISPCTL_BS_Pos                (1)                                               /*!< FMC_T::ISPCTL: BS Position                */
#define FMC_ISPCTL_BS_Msk                (0x1ul << FMC_ISPCTL_BS_Pos)                      /*!< FMC_T::ISPCTL: BS Mask                    */

#define FMC_ISPCTL_APUEN_Pos             (3)                                               /*!< FMC_T::ISPCTL: APUEN Position             */
#define FMC_ISPCTL_APUEN_Msk             (0x1ul << FMC_ISPCTL_APUEN_Pos)                   /*!< FMC_T::ISPCTL: APUEN Mask                 */

#define FMC_ISPCTL_CFGUEN_Pos            (4)                                               /*!< FMC_T::ISPCTL: CFGUEN Position            */
#define FMC_ISPCTL_CFGUEN_Msk            (0x1ul << FMC_ISPCTL_CFGUEN_Pos)                  /*!< FMC_T::ISPCTL: CFGUEN Mask                */

#define FMC_ISPCTL_LDUEN_Pos             (5)                                               /*!< FMC_T::ISPCTL: LDUEN Position             */
#define FMC_ISPCTL_LDUEN_Msk             (0x1ul << FMC_ISPCTL_LDUEN_Pos)                   /*!< FMC_T::ISPCTL: LDUEN Mask                 */

#define FMC_ISPCTL_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPCTL: ISPFF Position             */
#define FMC_ISPCTL_ISPFF_Msk             (0x1ul << FMC_ISPCTL_ISPFF_Pos)                   /*!< FMC_T::ISPCTL: ISPFF Mask                 */

#define FMC_ISPADDR_ISPADR_Pos           (0)                                               /*!< FMC_T::ISPADDR: ISPADR Position           */
#define FMC_ISPADDR_ISPADR_Msk           (0xfffffffful << FMC_ISPADDR_ISPADR_Pos)          /*!< FMC_T::ISPADDR: ISPADR Mask               */

#define FMC_ISPDAT_ISPDAT_Pos            (0)                                               /*!< FMC_T::ISPDAT: ISPDAT Position            */
#define FMC_ISPDAT_ISPDAT_Msk            (0xfffffffful << FMC_ISPDAT_ISPDAT_Pos)           /*!< FMC_T::ISPDAT: ISPDAT Mask                */

#define FMC_ISPCMD_CMD_Pos               (0)                                               /*!< FMC_T::ISPCMD: CMD Position               */
#define FMC_ISPCMD_CMD_Msk               (0x3ful << FMC_ISPCMD_CMD_Pos)                    /*!< FMC_T::ISPCMD: CMD Mask                   */

#define FMC_ISPTRG_ISPGO_Pos             (0)                                               /*!< FMC_T::ISPTRG: ISPGO Position             */
#define FMC_ISPTRG_ISPGO_Msk             (0x1ul << FMC_ISPTRG_ISPGO_Pos)                   /*!< FMC_T::ISPTRG: ISPGO Mask                 */

#define FMC_DFBA_DFBA_Pos                (0)                                               /*!< FMC_T::DFBA: DFBA Position                */
#define FMC_DFBA_DFBA_Msk                (0xfffffffful << FMC_DFBA_DFBA_Pos)               /*!< FMC_T::DFBA: DFBA Mask                    */

#define FMC_ISPSTS_ISPBUSY_Pos           (0)                                               /*!< FMC_T::ISPSTS: ISPBUSY Position           */
#define FMC_ISPSTS_ISPBUSY_Msk           (0x1ul << FMC_ISPSTS_ISPBUSY_Pos)                 /*!< FMC_T::ISPSTS: ISPBUSY Mask               */

#define FMC_ISPSTS_CBS_Pos               (1)                                               /*!< FMC_T::ISPSTS: CBS Position               */
#define FMC_ISPSTS_CBS_Msk               (0x3ul << FMC_ISPSTS_CBS_Pos)                     /*!< FMC_T::ISPSTS: CBS Mask                   */

#define FMC_ISPSTS_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPSTS: ISPFF Position             */
#define FMC_ISPSTS_ISPFF_Msk             (0x1ul << FMC_ISPSTS_ISPFF_Pos)                   /*!< FMC_T::ISPSTS: ISPFF Mask                 */

#define FMC_ISPSTS_VECMAP_Pos            (9)                                               /*!< FMC_T::ISPSTS: VECMAP Position            */
#define FMC_ISPSTS_VECMAP_Msk            (0xffful << FMC_ISPSTS_VECMAP_Pos)                /*!< FMC_T::ISPSTS: VECMAP Mask                */

/**@}*/ /* FMC_CONST */
/**@}*/ /* end of FMC register group */


/*---------------------- General Purpose Input/Output Controller -------------------------*/
/**
    @addtogroup GP General Purpose Input/Output Controller(GPIO)
    Memory Mapped Structure for GP Controller
@{ */

typedef struct
{


    /**
     * MODE
     * ===================================================================================================
     * Offset: 0x00  P0 I/O Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |MODE0     |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin.
     * |        |          |MODEn[1:0]
     * |        |          |Pin I/O   Mode (x = 0~5, n = 0~7).
     * |        |          |00 = Input mode.
     * |        |          |01 = Push-pull Output mode.
     * |        |          |10 = Open-Drain Output mode.
     * |        |          |11 = Quasi-bidirectional mode.
     * |[3:2]   |MODE1     |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin.
     * |        |          |MODEn[1:0]
     * |        |          |Pin I/O   Mode (x = 0~5, n = 0~7).
     * |        |          |00 = Input mode.
     * |        |          |01 = Push-pull Output mode.
     * |        |          |10 = Open-Drain Output mode.
     * |        |          |11 = Quasi-bidirectional mode.
     * |[5:4]   |MODE2     |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin.
     * |        |          |MODEn[1:0]
     * |        |          |Pin I/O   Mode (x = 0~5, n = 0~7).
     * |        |          |00 = Input mode.
     * |        |          |01 = Push-pull Output mode.
     * |        |          |10 = Open-Drain Output mode.
     * |        |          |11 = Quasi-bidirectional mode.
     * |[7:6]   |MODE3     |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin.
     * |        |          |MODEn[1:0]
     * |        |          |Pin I/O   Mode (x = 0~5, n = 0~7).
     * |        |          |00 = Input mode.
     * |        |          |01 = Push-pull Output mode.
     * |        |          |10 = Open-Drain Output mode.
     * |        |          |11 = Quasi-bidirectional mode.
     * |[9:8]   |MODE4     |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin.
     * |        |          |MODEn[1:0]
     * |        |          |Pin I/O   Mode (x = 0~5, n = 0~7).
     * |        |          |00 = Input mode.
     * |        |          |01 = Push-pull Output mode.
     * |        |          |10 = Open-Drain Output mode.
     * |        |          |11 = Quasi-bidirectional mode.
     * |[11:10] |MODE5     |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin.
     * |        |          |MODEn[1:0]
     * |        |          |Pin I/O   Mode (x = 0~5, n = 0~7).
     * |        |          |00 = Input mode.
     * |        |          |01 = Push-pull Output mode.
     * |        |          |10 = Open-Drain Output mode.
     * |        |          |11 = Quasi-bidirectional mode.
     * |[13:12] |MODE6     |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin.
     * |        |          |MODEn[1:0]
     * |        |          |Pin I/O   Mode (x = 0~5, n = 0~7).
     * |        |          |00 = Input mode.
     * |        |          |01 = Push-pull Output mode.
     * |        |          |10 = Open-Drain Output mode.
     * |        |          |11 = Quasi-bidirectional mode.
     * |[15:14] |MODE7     |Port 0-5 I/O Pin [N] Mode Control
     * |        |          |Determine each I/O mode of Px.n pin.
     * |        |          |MODEn[1:0]
     * |        |          |Pin I/O   Mode (x = 0~5, n = 0~7).
     * |        |          |00 = Input mode.
     * |        |          |01 = Push-pull Output mode.
     * |        |          |10 = Open-Drain Output mode.
     * |        |          |11 = Quasi-bidirectional mode.
    */
    __IO uint32_t MODE;

    /**
     * DINOFF
     * ===================================================================================================
     * Offset: 0x04  P0 Digital Input Path Disable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:16] |DINOFF    |Port 0-5 Pin [N] Digital Input Path Disable Control
     * |        |          |0 = Px.n digital input path Enabled.
     * |        |          |1 = Px.n digital input path Disabled (digital input tied to low).
     * |        |          |Note: x = 0~5, n = 0~7.
    */
    __IO uint32_t DINOFF;

    /**
     * DOUT
     * ===================================================================================================
     * Offset: 0x08  P0 Data Output Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DOUT      |Port 0-5 Pin [N] Output Value
     * |        |          |Each of these bits controls the status of a Px.n pin when the Px.n is configured as Push-pull output, Open-drain output and Quasi-bidirectional mode.
     * |        |          |0 = Px.n will drive Low if the Px.n pin is configured as Push-pull output, Open-drain output or Quasi-bidirectional mode.
     * |        |          |1 = Px.n will drive High if the Px.n pin is configured as Push-pull output or Quasi-bidirectional mode.
     * |        |          |Note: x = 0~5, n = 0~7.
    */
    __IO uint32_t DOUT;

    /**
     * DATMSK
     * ===================================================================================================
     * Offset: 0x0C  P0 Data Output Write Mask
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DATMSK    |Port 0-5 Pin [N] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding Px_DOUT[n] bit.
     * |        |          |When the DATMSK[n] bit is set to 1, the corresponding Px_DOUT[n] bit is protected.
     * |        |          |If the write signal is masked, writing data to the protect bit is ignored.
     * |        |          |0 = Corresponding Px_DOUT[n] bit can be updated.
     * |        |          |1 = Corresponding Px_DOUT[n] bit is protected.
     * |        |          |Note1: x = 0~5, n = 0~7.
     * |        |          |Note2: This function only protects the corresponding Px_DOUT[n] bit, and will not protect the corresponding Pxn_PDIO bit.
    */
    __IO uint32_t DATMSK;

    /**
     * PIN
     * ===================================================================================================
     * Offset: 0x10  P0 Pin Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PIN       |Port 0-5 Pin [N] Pin Value
     * |        |          |Each bit of the register reflects the actual status of the respective Px.n pin.
     * |        |          |If the bit is 1, it indicates the corresponding pin status is high; else the pin status is low.
     * |        |          |Note: x = 0~5, n = 0~7.
    */
    __I  uint32_t PIN;

    /**
     * DBEN
     * ===================================================================================================
     * Offset: 0x14  P0 De-bounce Enable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DBEN      |Port 0-5 Pin [N] Input Signal De-Bounce Enable Control
     * |        |          |DBEN[n] bit is used to enable the de-bounce function for each corresponding bit.
     * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the input signal transition is seen as the signal bounce and will not trigger the interrupt.
     * |        |          |The de-bounce clock source is controlled by DBNCECON[4], one de-bounce sample cycle period is controlled by DBNCECON[3:0].
     * |        |          |0 = Px.n de-bounce function Disabled.
     * |        |          |1 = Px.n de-bounce function Enabled.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note1: x = 0~5, n = 0~7.
     * |        |          |Note2: If Px.n pin is chosen as Power-down wake-up source, user should be disable the de-bounce function before entering Power-down mode to avoid the second interrupt event occurred after system waken up which caused by Px.n de-bounce function.
    */
    __IO uint32_t DBEN;

    /**
     * INTTYPE
     * ===================================================================================================
     * Offset: 0x18  P0 Interrupt Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |INTTYPE   |Port 0-5 Pin [N] Edge Or Level Detection Interrupt Mode Control
     * |        |          |INTTYPE[n] bit is used to control the triggered interrupt is by level trigger or by edge trigger.
     * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
     * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK clock and generates the interrupt.
     * |        |          |0 = Edge trigger interrupt.
     * |        |          |1 = Level trigger interrupt.
     * |        |          |If pin is set as the level trigger interrupt, only one level can be set on the registers Px_INTEN.
     * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
     * |        |          |The de-bounce function is valid only for edge triggered interrupt.
     * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
     * |        |          |Note: x = 0~5, n = 0~7.
    */
    __IO uint32_t INTTYPE;

    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x1C  P0 Interrupt Enable Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FLIEN     |Port 0-5 Pin [N] Interrupt Enable By Input Falling Edge Or Input Level Low
     * |        |          |FLIEN[n] is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the IF_EB[n] bit to 1:
     * |        |          |If the interrupt is level trigger (INTTYPE[n] is 1), the input Px.n pin will generate the interrupt while this pin state is at low level.
     * |        |          |If the interrupt is edge mode trigger (INTTYPE[n] is 0), the input Px.n pin will generate the interrupt while this pin state changed from high to low.
     * |        |          |0 = Px.n low level or high to low interrupt Disabled.
     * |        |          |1 = Px.n low level or high to low interrupt Enabled.
     * |        |          |Note: x = 0~5, n = 0~7.
     * |[23:16] |RHIEN     |Port 0-5 Pin [N] Interrupt Enable By Input Rising Edge Or Input Level High
     * |        |          |IR_EN[n] bit is used to enable the interrupt for each of the corresponding input Px.n pin.
     * |        |          |Set bit to 1 also enable the pin wake-up function.
     * |        |          |When setting the IR_EN[n] bit to 1:
     * |        |          |If the interrupt is level trigger (INTTYPE[n] is 1), the input Px.n pin will generate the interrupt while this pin state is at high level.
     * |        |          |If the interrupt is edge trigger (INTTYPE[n] is 0), the input Px.n pin will generate the interrupt while this pin state changed from low to high.
     * |        |          |0 = Px.n level high or low to high interrupt Disabled.
     * |        |          |1 = Px.n level high or low to high interrupt Enabled.
     * |        |          |Note: x = 0~5, n = 0~7.
    */
    __IO uint32_t INTEN;

    /**
     * INTSRC
     * ===================================================================================================
     * Offset: 0x20  P0 Interrupt Source Flag
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |INTSRC    |Port 0-5 Pin [N] Interrupt Source Flag
     * |        |          |Write :
     * |        |          |0 = No action.
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |Read :
     * |        |          |0 = No interrupt at Px.n.
     * |        |          |1 = Px.n generates an interrupt.
     * |        |          |Note: x = 0~5, n = 0~7.
    */
    __IO uint32_t INTSRC;

} GPIO_T;



typedef struct
{

    /**
     * DBCTL
     * ===================================================================================================
     * Offset: 0x180  Interrupt De-bounce Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[3:0]   |DBCLKSEL  |De-Bounce Sampling Cycle Selection
     * |        |          |0000 = Sample interrupt input once per 1 clock.
     * |        |          |0001 = Sample interrupt input once per 2 clocks.
     * |        |          |0010 = Sample interrupt input once per 4 clocks.
     * |        |          |0011 = Sample interrupt input once per 8 clocks.
     * |        |          |0100 = Sample interrupt input once per 16 clocks.
     * |        |          |0101 = Sample interrupt input once per 32 clocks.
     * |        |          |0110 = Sample interrupt input once per 64 clocks.
     * |        |          |0111 = Sample interrupt input once per 128 clocks.
     * |        |          |1000 = Sample interrupt input once per 256 clocks.
     * |        |          |1001 = Sample interrupt input once per 512 clocks.
     * |        |          |1010 = Sample interrupt input once per 1024 clocks.
     * |        |          |1011 = Sample interrupt input once per 2048 clocks.
     * |        |          |1100 = Sample interrupt input once per 4096 clocks.
     * |        |          |1101 = Sample interrupt input once per 8192 clocks.
     * |        |          |1110 = Sample interrupt input once per 16384 clocks.
     * |        |          |1111 = Sample interrupt input once per 32768 clocks.
     * |[4]     |DBCLKSRC  |De-Bounce Counter Clock Source Selection
     * |        |          |0 = De-bounce counter clock source is the HCLK.
     * |        |          |1 = De-bounce counter clock source is the 10 kHz internal low speed oscillator.
     * |[5]     |ICLKON    |Interrupt Clock On Mode
     * |        |          |0 = Edge detection circuit is active only if I/O pin corresponding Px_INTEN bit is set to 1.
     * |        |          |1 = All I/O pins edge detection circuit is always active after reset.
     * |        |          |Note: It is recommended to turn off this bit to save system power if no special application concern.
    */
    __IO uint32_t DBCTL;

} GPIO_DB_T;


/**
    @addtogroup GP_CONST GP Bit Field Definition
    Constant Definitions for GP Controller
@{ */

#define GP_MODE_MODE0_Pos                (0)                                               /*!< GPIO_T::MODE: MODE0 Position                */
#define GP_MODE_MODE0_Msk                (0x3ul << GP_MODE_MODE0_Pos)                      /*!< GPIO_T::MODE: MODE0 Mask                    */

#define GP_MODE_MODE1_Pos                (2)                                               /*!< GPIO_T::MODE: MODE1 Position                */
#define GP_MODE_MODE1_Msk                (0x3ul << GP_MODE_MODE1_Pos)                      /*!< GPIO_T::MODE: MODE1 Mask                    */

#define GP_MODE_MODE2_Pos                (4)                                               /*!< GPIO_T::MODE: MODE2 Position                */
#define GP_MODE_MODE2_Msk                (0x3ul << GP_MODE_MODE2_Pos)                      /*!< GPIO_T::MODE: MODE2 Mask                    */

#define GP_MODE_MODE3_Pos                (6)                                               /*!< GPIO_T::MODE: MODE3 Position                */
#define GP_MODE_MODE3_Msk                (0x3ul << GP_MODE_MODE3_Pos)                      /*!< GPIO_T::MODE: MODE3 Mask                    */

#define GP_MODE_MODE4_Pos                (8)                                               /*!< GPIO_T::MODE: MODE4 Position                */
#define GP_MODE_MODE4_Msk                (0x3ul << GP_MODE_MODE4_Pos)                      /*!< GPIO_T::MODE: MODE4 Mask                    */

#define GP_MODE_MODE5_Pos                (10)                                              /*!< GPIO_T::MODE: MODE5 Position                */
#define GP_MODE_MODE5_Msk                (0x3ul << GP_MODE_MODE5_Pos)                      /*!< GPIO_T::MODE: MODE5 Mask                    */

#define GP_DINOFF_DINOFF0_Pos            (16)                                              /*!< GPIO_T::DINOFF: DINOFF0 Position             */
#define GP_DINOFF_DINOFF0_Msk            (0x1ul << GP_DINOFF_DINOFF0_Pos)                  /*!< GPIO_T::DINOFF: DINOFF0 Mask                 */

#define GP_DINOFF_DINOFF1_Pos            (17)                                              /*!< GPIO_T::DINOFF: DINOFF1 Position             */
#define GP_DINOFF_DINOFF1_Msk            (0x1ul << GP_DINOFF_DINOFF1_Pos)                  /*!< GPIO_T::DINOFF: DINOFF1 Mask                 */

#define GP_DINOFF_DINOFF2_Pos            (18)                                              /*!< GPIO_T::DINOFF: DINOFF2 Position             */
#define GP_DINOFF_DINOFF2_Msk            (0x1ul << GP_DINOFF_DINOFF2_Pos)                  /*!< GPIO_T::DINOFF: DINOFF2 Mask                 */

#define GP_DINOFF_DINOFF3_Pos            (19)                                              /*!< GPIO_T::DINOFF: DINOFF3 Position             */
#define GP_DINOFF_DINOFF3_Msk            (0x1ul << GP_DINOFF_DINOFF3_Pos)                  /*!< GPIO_T::DINOFF: DINOFF3 Mask                 */

#define GP_DINOFF_DINOFF4_Pos            (20)                                              /*!< GPIO_T::DINOFF: DINOFF4 Position             */
#define GP_DINOFF_DINOFF4_Msk            (0x1ul << GP_DINOFF_DINOFF4_Pos)                  /*!< GPIO_T::DINOFF: DINOFF4 Mask                 */

#define GP_DINOFF_DINOFF5_Pos            (21)                                              /*!< GPIO_T::DINOFF: DINOFF5 Position             */
#define GP_DINOFF_DINOFF5_Msk            (0x1ul << GP_DINOFF_DINOFF5_Pos)                  /*!< GPIO_T::DINOFF: DINOFF5 Mask                 */

#define GP_DINOFF_DINOFF6_Pos            (22)                                              /*!< GPIO_T::DINOFF: DINOFF6 Position             */
#define GP_DINOFF_DINOFF6_Msk            (0x1ul << GP_DINOFF_DINOFF6_Pos)                  /*!< GPIO_T::DINOFF: DINOFF6 Mask                 */

#define GP_DINOFF_DINOFF7_Pos            (23)                                              /*!< GPIO_T::DINOFF: DINOFF7 Position             */
#define GP_DINOFF_DINOFF7_Msk            (0x1ul << GP_DINOFF_DINOFF7_Pos)                  /*!< GPIO_T::DINOFF: DINOFF7 Mask                 */

#define GP_DOUT_DOUT0_Pos                (0)                                               /*!< GPIO_T::DOUT: DOUT0 Position                 */
#define GP_DOUT_DOUT0_Msk                (0x1ul << GP_DOUT_DOUT0_Pos)                      /*!< GPIO_T::DOUT: DOUT0 Mask                     */

#define GP_DOUT_DOUT1_Pos                (1)                                               /*!< GPIO_T::DOUT: DOUT1 Position                 */
#define GP_DOUT_DOUT1_Msk                (0x1ul << GP_DOUT_DOUT1_Pos)                      /*!< GPIO_T::DOUT: DOUT1 Mask                     */

#define GP_DOUT_DOUT2_Pos                (2)                                               /*!< GPIO_T::DOUT: DOUT2 Position                 */
#define GP_DOUT_DOUT2_Msk                (0x1ul << GP_DOUT_DOUT2_Pos)                      /*!< GPIO_T::DOUT: DOUT2 Mask                     */

#define GP_DOUT_DOUT3_Pos                (3)                                               /*!< GPIO_T::DOUT: DOUT3 Position                 */
#define GP_DOUT_DOUT3_Msk                (0x1ul << GP_DOUT_DOUT3_Pos)                      /*!< GPIO_T::DOUT: DOUT3 Mask                     */

#define GP_DOUT_DOUT4_Pos                (4)                                               /*!< GPIO_T::DOUT: DOUT4 Position                 */
#define GP_DOUT_DOUT4_Msk                (0x1ul << GP_DOUT_DOUT4_Pos)                      /*!< GPIO_T::DOUT: DOUT4 Mask                     */

#define GP_DOUT_DOUT5_Pos                (5)                                               /*!< GPIO_T::DOUT: DOUT5 Position                 */
#define GP_DOUT_DOUT5_Msk                (0x1ul << GP_DOUT_DOUT5_Pos)                      /*!< GPIO_T::DOUT: DOUT5 Mask                     */

#define GP_DOUT_DOUT6_Pos                (6)                                               /*!< GPIO_T::DOUT: DOUT6 Position                 */
#define GP_DOUT_DOUT6_Msk                (0x1ul << GP_DOUT_DOUT6_Pos)                      /*!< GPIO_T::DOUT: DOUT6 Mask                     */

#define GP_DOUT_DOUT7_Pos                (7)                                               /*!< GPIO_T::DOUT: DOUT7 Position                 */
#define GP_DOUT_DOUT7_Msk                (0x1ul << GP_DOUT_DOUT7_Pos)                      /*!< GPIO_T::DOUT: DOUT7 Mask                     */

#define GP_DATMSK_DATMSK0_Pos            (0)                                               /*!< GPIO_T::DATMSK: DATMSK0 Position             */
#define GP_DATMSK_DATMSK0_Msk            (0x1ul << GP_DATMSK_DATMSK0_Pos)                  /*!< GPIO_T::DATMSK: DATMSK0 Mask                 */

#define GP_DATMSK_DATMSK1_Pos            (1)                                               /*!< GPIO_T::DATMSK: DATMSK1 Position             */
#define GP_DATMSK_DATMSK1_Msk            (0x1ul << GP_DATMSK_DATMSK1_Pos)                  /*!< GPIO_T::DATMSK: DATMSK1 Mask                 */

#define GP_DATMSK_DATMSK2_Pos            (2)                                               /*!< GPIO_T::DATMSK: DATMSK2 Position             */
#define GP_DATMSK_DATMSK2_Msk            (0x1ul << GP_DATMSK_DATMSK2_Pos)                  /*!< GPIO_T::DATMSK: DATMSK2 Mask                 */

#define GP_DATMSK_DATMSK3_Pos            (3)                                               /*!< GPIO_T::DATMSK: DATMSK3 Position             */
#define GP_DATMSK_DATMSK3_Msk            (0x1ul << GP_DATMSK_DATMSK3_Pos)                  /*!< GPIO_T::DATMSK: DATMSK3 Mask                 */

#define GP_DATMSK_DATMSK4_Pos            (4)                                               /*!< GPIO_T::DATMSK: DATMSK4 Position             */
#define GP_DATMSK_DATMSK4_Msk            (0x1ul << GP_DATMSK_DATMSK4_Pos)                  /*!< GPIO_T::DATMSK: DATMSK4 Mask                 */

#define GP_DATMSK_DATMSK5_Pos            (5)                                               /*!< GPIO_T::DATMSK: DATMSK5 Position             */
#define GP_DATMSK_DATMSK5_Msk            (0x1ul << GP_DATMSK_DATMSK5_Pos)                  /*!< GPIO_T::DATMSK: DATMSK5 Mask                 */

#define GP_DATMSK_DATMSK6_Pos            (6)                                               /*!< GPIO_T::DATMSK: DATMSK6 Position             */
#define GP_DATMSK_DATMSK6_Msk            (0x1ul << GP_DATMSK_DATMSK6_Pos)                  /*!< GPIO_T::DATMSK: DATMSK6 Mask                 */

#define GP_DATMSK_DATMSK7_Pos            (7)                                               /*!< GPIO_T::DATMSK: DATMSK7 Position             */
#define GP_DATMSK_DATMSK7_Msk            (0x1ul << GP_DATMSK_DATMSK7_Pos)                  /*!< GPIO_T::DATMSK: DATMSK7 Mask                 */

#define GP_PIN_PIN0_Pos                  (0)                                               /*!< GPIO_T::PIN: PIN0 Position                  */
#define GP_PIN_PIN0_Msk                  (0x1ul << GP_PIN_PIN0_Pos)                        /*!< GPIO_T::PIN: PIN0 Mask                      */

#define GP_PIN_PIN1_Pos                  (1)                                               /*!< GPIO_T::PIN: PIN1 Position                  */
#define GP_PIN_PIN1_Msk                  (0x1ul << GP_PIN_PIN1_Pos)                        /*!< GPIO_T::PIN: PIN1 Mask                      */

#define GP_PIN_PIN2_Pos                  (2)                                               /*!< GPIO_T::PIN: PIN2 Position                  */
#define GP_PIN_PIN2_Msk                  (0x1ul << GP_PIN_PIN2_Pos)                        /*!< GPIO_T::PIN: PIN2 Mask                      */

#define GP_PIN_PIN3_Pos                  (3)                                               /*!< GPIO_T::PIN: PIN3 Position                  */
#define GP_PIN_PIN3_Msk                  (0x1ul << GP_PIN_PIN3_Pos)                        /*!< GPIO_T::PIN: PIN3 Mask                      */

#define GP_PIN_PIN4_Pos                  (4)                                               /*!< GPIO_T::PIN: PIN4 Position                  */
#define GP_PIN_PIN4_Msk                  (0x1ul << GP_PIN_PIN4_Pos)                        /*!< GPIO_T::PIN: PIN4 Mask                      */

#define GP_PIN_PIN5_Pos                  (5)                                               /*!< GPIO_T::PIN: PIN5 Position                  */
#define GP_PIN_PIN5_Msk                  (0x1ul << GP_PIN_PIN5_Pos)                        /*!< GPIO_T::PIN: PIN5 Mask                      */

#define GP_PIN_PIN6_Pos                  (6)                                               /*!< GPIO_T::PIN: PIN6 Position                  */
#define GP_PIN_PIN6_Msk                  (0x1ul << GP_PIN_PIN6_Pos)                        /*!< GPIO_T::PIN: PIN6 Mask                      */

#define GP_PIN_PIN7_Pos                  (7)                                               /*!< GPIO_T::PIN: PIN7 Position                  */
#define GP_PIN_PIN7_Msk                  (0x1ul << GP_PIN_PIN7_Pos)                        /*!< GPIO_T::PIN: PIN7 Mask                      */

#define GP_DBEN_DBEN0_Pos                (0)                                               /*!< GPIO_T::DBEN: DBEN0 Position                */
#define GP_DBEN_DBEN0_Msk                (0x1ul << GP_DBEN_DBEN0_Pos)                      /*!< GPIO_T::DBEN: DBEN0 Mask                    */

#define GP_DBEN_DBEN1_Pos                (1)                                               /*!< GPIO_T::DBEN: DBEN1 Position                */
#define GP_DBEN_DBEN1_Msk                (0x1ul << GP_DBEN_DBEN1_Pos)                      /*!< GPIO_T::DBEN: DBEN1 Mask                    */

#define GP_DBEN_DBEN2_Pos                (2)                                               /*!< GPIO_T::DBEN: DBEN2 Position                */
#define GP_DBEN_DBEN2_Msk                (0x1ul << GP_DBEN_DBEN2_Pos)                      /*!< GPIO_T::DBEN: DBEN2 Mask                    */

#define GP_DBEN_DBEN3_Pos                (3)                                               /*!< GPIO_T::DBEN: DBEN3 Position                */
#define GP_DBEN_DBEN3_Msk                (0x1ul << GP_DBEN_DBEN3_Pos)                      /*!< GPIO_T::DBEN: DBEN3 Mask                    */

#define GP_DBEN_DBEN4_Pos                (4)                                               /*!< GPIO_T::DBEN: DBEN4 Position                */
#define GP_DBEN_DBEN4_Msk                (0x1ul << GP_DBEN_DBEN4_Pos)                      /*!< GPIO_T::DBEN: DBEN4 Mask                    */

#define GP_DBEN_DBEN5_Pos                (5)                                               /*!< GPIO_T::DBEN: DBEN5 Position                */
#define GP_DBEN_DBEN5_Msk                (0x1ul << GP_DBEN_DBEN5_Pos)                      /*!< GPIO_T::DBEN: DBEN5 Mask                    */

#define GP_DBEN_DBEN6_Pos                (6)                                               /*!< GPIO_T::DBEN: DBEN6 Position                */
#define GP_DBEN_DBEN6_Msk                (0x1ul << GP_DBEN_DBEN6_Pos)                      /*!< GPIO_T::DBEN: DBEN6 Mask                    */

#define GP_DBEN_DBEN7_Pos                (7)                                               /*!< GPIO_T::DBEN: DBEN7 Position                */
#define GP_DBEN_DBEN7_Msk                (0x1ul << GP_DBEN_DBEN7_Pos)                      /*!< GPIO_T::DBEN: DBEN7 Mask                    */

#define GP_INTTYPE_TYPE0_Pos             (0)                                               /*!< GPIO_T::INTTYPE: TYPE0 Position             */
#define GP_INTTYPE_TYPE0_Msk             (0x1ul << GP_INTTYPE_TYPE0_Pos)                   /*!< GPIO_T::INTTYPE: TYPE0 Mask                 */

#define GP_INTTYPE_TYPE1_Pos             (1)                                               /*!< GPIO_T::INTTYPE: TYPE1 Position             */
#define GP_INTTYPE_TYPE1_Msk             (0x1ul << GP_INTTYPE_TYPE1_Pos)                   /*!< GPIO_T::INTTYPE: TYPE1 Mask                 */

#define GP_INTTYPE_TYPE2_Pos             (2)                                               /*!< GPIO_T::INTTYPE: TYPE2 Position             */
#define GP_INTTYPE_TYPE2_Msk             (0x1ul << GP_INTTYPE_TYPE2_Pos)                   /*!< GPIO_T::INTTYPE: TYPE2 Mask                 */

#define GP_INTTYPE_TYPE3_Pos             (3)                                               /*!< GPIO_T::INTTYPE: TYPE3 Position             */
#define GP_INTTYPE_TYPE3_Msk             (0x1ul << GP_INTTYPE_TYPE3_Pos)                   /*!< GPIO_T::INTTYPE: TYPE3 Mask                 */

#define GP_INTTYPE_TYPE4_Pos             (4)                                               /*!< GPIO_T::INTTYPE: TYPE4 Position             */
#define GP_INTTYPE_TYPE4_Msk             (0x1ul << GP_INTTYPE_TYPE4_Pos)                   /*!< GPIO_T::INTTYPE: TYPE4 Mask                 */

#define GP_INTTYPE_TYPE5_Pos             (5)                                               /*!< GPIO_T::INTTYPE: TYPE5 Position             */
#define GP_INTTYPE_TYPE5_Msk             (0x1ul << GP_INTTYPE_TYPE5_Pos)                   /*!< GPIO_T::INTTYPE: TYPE5 Mask                 */

#define GP_INTTYPE_TYPE6_Pos             (6)                                               /*!< GPIO_T::INTTYPE: TYPE6 Position             */
#define GP_INTTYPE_TYPE6_Msk             (0x1ul << GP_INTTYPE_TYPE6_Pos)                   /*!< GPIO_T::INTTYPE: TYPE6 Mask                 */

#define GP_INTTYPE_TYPE7_Pos             (7)                                               /*!< GPIO_T::INTTYPE: TYPE7 Position             */
#define GP_INTTYPE_TYPE7_Msk             (0x1ul << GP_INTTYPE_TYPE7_Pos)                   /*!< GPIO_T::INTTYPE: TYPE7 Mask                 */

#define GP_INTEN_FLIEN0_Pos              (0)                                               /*!< GPIO_T::INTEN: FLIEN0 Position              */
#define GP_INTEN_FLIEN0_Msk              (0x1ul << GP_INTEN_FLIEN0_Pos)                    /*!< GPIO_T::INTEN: FLIEN0 Mask                  */

#define GP_INTEN_FLIEN1_Pos              (1)                                               /*!< GPIO_T::INTEN: FLIEN1 Position              */
#define GP_INTEN_FLIEN1_Msk              (0x1ul << GP_INTEN_FLIEN1_Pos)                    /*!< GPIO_T::INTEN: FLIEN1 Mask                  */

#define GP_INTEN_FLIEN2_Pos              (2)                                               /*!< GPIO_T::INTEN: FLIEN2 Position              */
#define GP_INTEN_FLIEN2_Msk              (0x1ul << GP_INTEN_FLIEN2_Pos)                    /*!< GPIO_T::INTEN: FLIEN2 Mask                  */

#define GP_INTEN_FLIEN3_Pos              (3)                                               /*!< GPIO_T::INTEN: FLIEN3 Position              */
#define GP_INTEN_FLIEN3_Msk              (0x1ul << GP_INTEN_FLIEN3_Pos)                    /*!< GPIO_T::INTEN: FLIEN3 Mask                  */

#define GP_INTEN_FLIEN4_Pos              (4)                                               /*!< GPIO_T::INTEN: FLIEN4 Position              */
#define GP_INTEN_FLIEN4_Msk              (0x1ul << GP_INTEN_FLIEN4_Pos)                    /*!< GPIO_T::INTEN: FLIEN4 Mask                  */

#define GP_INTEN_FLIEN5_Pos              (5)                                               /*!< GPIO_T::INTEN: FLIEN5 Position              */
#define GP_INTEN_FLIEN5_Msk              (0x1ul << GP_INTEN_FLIEN5_Pos)                    /*!< GPIO_T::INTEN: FLIEN5 Mask                  */

#define GP_INTEN_FLIEN6_Pos              (6)                                               /*!< GPIO_T::INTEN: FLIEN6 Position              */
#define GP_INTEN_FLIEN6_Msk              (0x1ul << GP_INTEN_FLIEN6_Pos)                    /*!< GPIO_T::INTEN: FLIEN6 Mask                  */

#define GP_INTEN_FLIEN7_Pos              (7)                                               /*!< GPIO_T::INTEN: FLIEN7 Position              */
#define GP_INTEN_FLIEN7_Msk              (0x1ul << GP_INTEN_FLIEN7_Pos)                    /*!< GPIO_T::INTEN: FLIEN7 Mask                  */

#define GP_INTEN_RHIEN0_Pos              (16)                                              /*!< GPIO_T::INTEN: RHIEN0 Position              */
#define GP_INTEN_RHIEN0_Msk              (0x1ul << GP_INTEN_RHIEN0_Pos)                    /*!< GPIO_T::INTEN: RHIEN0 Mask                  */

#define GP_INTEN_RHIEN1_Pos              (17)                                              /*!< GPIO_T::INTEN: RHIEN1 Position              */
#define GP_INTEN_RHIEN1_Msk              (0x1ul << GP_INTEN_RHIEN1_Pos)                    /*!< GPIO_T::INTEN: RHIEN1 Mask                  */

#define GP_INTEN_RHIEN2_Pos              (18)                                              /*!< GPIO_T::INTEN: RHIEN2 Position              */
#define GP_INTEN_RHIEN2_Msk              (0x1ul << GP_INTEN_RHIEN2_Pos)                    /*!< GPIO_T::INTEN: RHIEN2 Mask                  */

#define GP_INTEN_RHIEN3_Pos              (19)                                              /*!< GPIO_T::INTEN: RHIEN3 Position              */
#define GP_INTEN_RHIEN3_Msk              (0x1ul << GP_INTEN_RHIEN3_Pos)                    /*!< GPIO_T::INTEN: RHIEN3 Mask                  */

#define GP_INTEN_RHIEN4_Pos              (20)                                              /*!< GPIO_T::INTEN: RHIEN4 Position              */
#define GP_INTEN_RHIEN4_Msk              (0x1ul << GP_INTEN_RHIEN4_Pos)                    /*!< GPIO_T::INTEN: RHIEN4 Mask                  */

#define GP_INTEN_RHIEN5_Pos              (21)                                              /*!< GPIO_T::INTEN: RHIEN5 Position              */
#define GP_INTEN_RHIEN5_Msk              (0x1ul << GP_INTEN_RHIEN5_Pos)                    /*!< GPIO_T::INTEN: RHIEN5 Mask                  */

#define GP_INTEN_RHIEN6_Pos              (22)                                              /*!< GPIO_T::INTEN: RHIEN6 Position              */
#define GP_INTEN_RHIEN6_Msk              (0x1ul << GP_INTEN_RHIEN6_Pos)                    /*!< GPIO_T::INTEN: RHIEN6 Mask                  */

#define GP_INTEN_RHIEN7_Pos              (23)                                              /*!< GPIO_T::INTEN: RHIEN7 Position              */
#define GP_INTEN_RHIEN7_Msk              (0x1ul << GP_INTEN_RHIEN7_Pos)                    /*!< GPIO_T::INTEN: RHIEN7 Mask                  */

#define GP_INTSRC_INTSRC0_Pos            (0)                                               /*!< GPIO_T::INTSRC: INTSRC0 Position            */
#define GP_INTSRC_INTSRC0_Msk            (0x1ul << GP_INTSRC_INTSRC0_Pos)                  /*!< GPIO_T::INTSRC: INTSRC0 Mask                */

#define GP_INTSRC_INTSRC1_Pos            (1)                                               /*!< GPIO_T::INTSRC: INTSRC1 Position            */
#define GP_INTSRC_INTSRC1_Msk            (0x1ul << GP_INTSRC_INTSRC1_Pos)                  /*!< GPIO_T::INTSRC: INTSRC1 Mask                */

#define GP_INTSRC_INTSRC2_Pos            (2)                                               /*!< GPIO_T::INTSRC: INTSRC2 Position            */
#define GP_INTSRC_INTSRC2_Msk            (0x1ul << GP_INTSRC_INTSRC2_Pos)                  /*!< GPIO_T::INTSRC: INTSRC2 Mask                */

#define GP_INTSRC_INTSRC3_Pos            (3)                                               /*!< GPIO_T::INTSRC: INTSRC3 Position            */
#define GP_INTSRC_INTSRC3_Msk            (0x1ul << GP_INTSRC_INTSRC3_Pos)                  /*!< GPIO_T::INTSRC: INTSRC3 Mask                */

#define GP_INTSRC_INTSRC4_Pos            (4)                                               /*!< GPIO_T::INTSRC: INTSRC4 Position            */
#define GP_INTSRC_INTSRC4_Msk            (0x1ul << GP_INTSRC_INTSRC4_Pos)                  /*!< GPIO_T::INTSRC: INTSRC4 Mask                */

#define GP_INTSRC_INTSRC5_Pos            (5)                                               /*!< GPIO_T::INTSRC: INTSRC5 Position            */
#define GP_INTSRC_INTSRC5_Msk            (0x1ul << GP_INTSRC_INTSRC5_Pos)                  /*!< GPIO_T::INTSRC: INTSRC5 Mask                */

#define GP_INTSRC_INTSRC6_Pos            (6)                                               /*!< GPIO_T::INTSRC: INTSRC6 Position            */
#define GP_INTSRC_INTSRC6_Msk            (0x1ul << GP_INTSRC_INTSRC6_Pos)                  /*!< GPIO_T::INTSRC: INTSRC6 Mask                */

#define GP_INTSRC_INTSRC7_Pos            (7)                                               /*!< GPIO_T::INTSRC: INTSRC7 Position            */
#define GP_INTSRC_INTSRC7_Msk            (0x1ul << GP_INTSRC_INTSRC7_Pos)                  /*!< GPIO_T::INTSRC: INTSRC7 Mask                */

#define GP_DBCTL_DBCLKSEL_Pos            (0)                                               /*!< GPIO_DB_T::DBCTL: DBCLKSEL Position            */
#define GP_DBCTL_DBCLKSEL_Msk            (0xful << GP_DBCTL_DBCLKSEL_Pos)                  /*!< GPIO_DB_T::DBCTL: DBCLKSEL Mask                */

#define GP_DBCTL_DBCLKSRC_Pos            (4)                                               /*!< GPIO_DB_T::DBCTL: DBCLKSRC Position            */
#define GP_DBCTL_DBCLKSRC_Msk            (0x1ul << GP_DBCTL_DBCLKSRC_Pos)                  /*!< GPIO_DB_T::DBCTL: DBCLKSRC Mask                */

#define GP_DBCTL_ICLKON_Pos              (5)                                               /*!< GPIO_DB_T::DBCTL: ICLKON Position              */
#define GP_DBCTL_ICLKON_Msk              (0x1ul << GP_DBCTL_ICLKON_Pos)                    /*!< GPIO_DB_T::DBCTL: ICLKON Mask                  */


/**@}*/ /* GP_CONST */
/**@}*/ /* end of GP register group */


/*---------------------- Hardware Divider -------------------------*/
/**
    @addtogroup HDIV Hardware Divider(HDIV)
    Memory Mapped Structure for HDIV Controller
@{ */

typedef struct
{


    /**
     * DIVIDEND
     * ===================================================================================================
     * Offset: 0x00  Dividend Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |DIVIDEND  |Dividend Source
     * |        |          |This register is given the dividend of divider before calculation starting.
    */
    __IO uint32_t DIVIDEND;

    /**
     * DIVISOR
     * ===================================================================================================
     * Offset: 0x04  Divisor Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |DIVISOR   |Divisor Source
     * |        |          |This register is given the divisor of divider before calculation starts.
     * |        |          |Note: When this register is written, hardware divider will start calculate.
    */
    __IO uint32_t DIVISOR;

    /**
     * QUOTIENT
     * ===================================================================================================
     * Offset: 0x08  Quotient Result Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |QUOTIENT  |Quotient Result
     * |        |          |This register holds the quotient result of divider after calculation complete.
    */
    __IO uint32_t QUOTIENT;

    /**
     * REM
     * ===================================================================================================
     * Offset: 0x0C  Remainder Result Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |REM       |Remainder Result
     * |        |          |The remainder of hardware divider is 16-bit sign integer (REM[15:0]) with sign extension (REM[31:16]) to 32-bit integer.
    */
    __IO uint32_t REM;

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x10  Divider Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |DIVBYZERO |Divisor Zero Warning
     * |        |          |0 = The divisor is not 0.
     * |        |          |1 = The divisor is 0.
     * |        |          |Note: The DIVBYZERO flag is used to indicate divide-by-zero situation and updated whenever HDIV_DIVISOR is written.
     * |        |          |This register is read only.
    */
    __I  uint32_t STATUS;

} HDIV_T;

/**
    @addtogroup HDIV_CONST HDIV Bit Field Definition
    Constant Definitions for HDIV Controller
@{ */

#define HDIV_DIVIDEND_DIVIDEND_Pos       (0)                                               /*!< HDIV_T::DIVIDEND: DIVIDEND Position       */
#define HDIV_DIVIDEND_DIVIDEND_Msk       (0xfffffffful << HDIV_DIVIDEND_DIVIDEND_Pos)      /*!< HDIV_T::DIVIDEND: DIVIDEND Mask           */

#define HDIV_DIVISOR_DIVISOR_Pos         (0)                                               /*!< HDIV_T::DIVISOR: DIVISOR Position         */
#define HDIV_DIVISOR_DIVISOR_Msk         (0xfffful << HDIV_DIVISOR_DIVISOR_Pos)            /*!< HDIV_T::DIVISOR: DIVISOR Mask             */

#define HDIV_QUOTIENT_QUOTIENT_Pos       (0)                                               /*!< HDIV_T::QUOTIENT: QUOTIENT Position       */
#define HDIV_QUOTIENT_QUOTIENT_Msk       (0xfffffffful << HDIV_QUOTIENT_QUOTIENT_Pos)      /*!< HDIV_T::QUOTIENT: QUOTIENT Mask           */

#define HDIV_REM_REM_Pos                 (0)                                               /*!< HDIV_T::REM: REM Position                 */
#define HDIV_REM_REM_Msk                 (0xfffffffful << HDIV_REM_REM_Pos)                /*!< HDIV_T::REM: REM Mask                     */

#define HDIV_STATUS_DIVBYZERO_Pos        (1)                                               /*!< HDIV_T::STATUS: DIVBYZERO Position        */
#define HDIV_STATUS_DIVBYZERO_Msk        (0x1ul << HDIV_STATUS_DIVBYZERO_Pos)              /*!< HDIV_T::STATUS: DIVBYZERO Mask            */

/**@}*/ /* HDIV_CONST */
/**@}*/ /* end of HDIV register group */


/*---------------------- Inter-IC Bus Controller -------------------------*/
/**
    @addtogroup I2C Inter-IC Bus Controller(I2C)
    Memory Mapped Structure for I2C Controller
@{ */

typedef struct
{


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  I2C Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2]     |AA        |Assert Acknowledge Control Bit
     * |        |          |When AA=1 is prior to address or data received, an acknowledged (low level to SDA) will be returned during the acknowledge clock pulse on the SCL line when 1.) A slave is acknowledging the address sent from master, 2.) The receiver devices are acknowledging the data sent by transmitter.
     * |        |          |When AA=0 prior to address or data received, a Not acknowledged (high level to SDA) will be returned during the acknowledge clock pulse on the SCL line.
     * |[3]     |SI        |I2C Interrupt Flag
     * |        |          |When a new I2C state is present in the I2C_STATUS register, the SI flag is set by hardware, and if bit INTEN (I2C_CTL[7]) is set, the I2C interrupt is requested.
     * |        |          |SI must be cleared by software.
     * |        |          |Software can write 1 to clear this bit.
     * |[4]     |STO       |I2C STOP Control Bit
     * |        |          |In Master mode, setting STO to transmit a STOP condition to bus then I2C hardware will check the bus condition if a STOP condition is detected this bit will be cleared by hardware automatically.
     * |        |          |In Slave mode, setting STO resets I2C hardware to the defined "not addressed" Slave mode.
     * |        |          |This means it is NO LONGER in the Slave receiver mode to receive data from the master transmit device.
     * |[5]     |STA       |I2C START Control Bit
     * |        |          |Setting STA to logic 1 to enter Master mode.
     * |        |          |I2C hardware sends a START or repeats the START condition to bus when the bus is free.
     * |[6]     |I2CEN     |I2C Controller Enable Bit
     * |        |          |0 = I2C Controller Disabled.
     * |        |          |1 = I2C Controller Enabled.
     * |        |          |Set to enable I2C serial function controller.
     * |        |          |When I2CEN=1 the I2C serial function enables.
     * |        |          |The function of multi-function pin must be set to I2C first.
     * |[7]     |INTEN     |Enable Interrupt
     * |        |          |0 = I2C interrupt Disabled.
     * |        |          |1 = I2C interrupt Enabled.
    */
    __IO uint32_t CTL;

    /**
     * ADDR0
     * ===================================================================================================
     * Offset: 0x04  I2C Slave Address Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |ADDR      |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In Slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
    */
    __IO uint32_t ADDR0;

    /**
     * DAT
     * ===================================================================================================
     * Offset: 0x08  I2C Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DAT       |I2C Data Register
     * |        |          |Bit [7:0] is located with the 8-bit transferred data of the I2C serial port.
    */
    __IO uint32_t DAT;

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x0C  I2C Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |STATUS    |I2C Status Register
     * |        |          |The three least significant bits are always 0.
     * |        |          |The five most significant bits contain the status code.
     * |        |          |There are 26 possible status codes.
     * |        |          |When STATUS contains F8H, no serial interrupt is requested.
     * |        |          |All the other STATUS values correspond to defined I2C states.
     * |        |          |When each of these states is entered, a status interrupt is requested (SI = 1).
     * |        |          |A valid status code is present in STATUS one cycle after SI is set by hardware and is still present one cycle after SI has been reset by software.
     * |        |          |In addition, the states 00H stands for a Bus Error.
     * |        |          |A Bus Error occurs when a START or STOP condition is present at an illegal position in the formation frame.
     * |        |          |Examples of illegal position are during the serial transfer of an address byte, a data byte or an acknowledge bit.
    */
    __I  uint32_t STATUS;

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x10  I2C Clock Divided Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DIVIDER   |I2C Clock Divided Register
     * |        |          |The I2C clock rate bits: Data Baud Rate of I2C = (system clock) / (4x (DIVIDER +1)).
     * |        |          |Note: The minimum value of DIVIDER is 4.
    */
    __IO uint32_t CLKDIV;

    /**
     * TOCTL
     * ===================================================================================================
     * Offset: 0x14  I2C Time-Out Counter Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TOIF      |Time-Out Flag
     * |        |          |This bit is set by hardware when I2C time-out happened and it can interrupt CPU if I2C interrupt enable bit (INTEN) is set to 1.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[1]     |TOCDIV4   |Time-Out Counter Input Clock Divided By 4
     * |        |          |0 = Time-out counter input clock divided by 4 Disabled.
     * |        |          |1 = Time-out counter input clock divided by 4 Enabled.
     * |        |          |Note: When enabled, the time-out period is extended 4 times.
     * |[2]     |TOCEN     |Time-Out Counter Enabled
     * |        |          |0 = Time-out counter Disabled.
     * |        |          |1 = Time-out counter Enabled.
     * |        |          |Note: When the 14-bit time-out counter is enabled, it will start counting when SI is clear.
     * |        |          |Setting 1to the SI flag will reset counter and re-start up counting after SI is cleared.
    */
    __IO uint32_t TOCTL;

    /**
     * ADDR1
     * ===================================================================================================
     * Offset: 0x18  I2C Slave Address Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |ADDR      |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In Slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
    */
    __IO uint32_t ADDR1;

    /**
     * ADDR2
     * ===================================================================================================
     * Offset: 0x1C  I2C Slave Address Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |ADDR      |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In Slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
    */
    __IO uint32_t ADDR2;

    /**
     * ADDR3
     * ===================================================================================================
     * Offset: 0x20  I2C Slave Address Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = General Call Function Disabled.
     * |        |          |1 = General Call Function Enabled.
     * |[7:1]   |ADDR      |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in Master mode.
     * |        |          |In Slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if either of the address is matched.
    */
    __IO uint32_t ADDR3;

    /**
     * ADDRMSK0
     * ===================================================================================================
     * Offset: 0x24  I2C Slave Address Mask Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |ADDRMSK   |I2C Address Mask Register
     * |        |          |0 = I2C address mask Disabled (the received corresponding register bit should be exactly the same as address register).
     * |        |          |1 = I2C address mask Enabled (the received corresponding address bit is "Don't care").
    */
    __IO uint32_t ADDRMSK0;

    /**
     * ADDRMSK1
     * ===================================================================================================
     * Offset: 0x28  I2C Slave Address Mask Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |ADDRMSK   |I2C Address Mask Register
     * |        |          |0 = I2C address mask Disabled (the received corresponding register bit should be exactly the same as address register).
     * |        |          |1 = I2C address mask Enabled (the received corresponding address bit is "Don't care").
    */
    __IO uint32_t ADDRMSK1;

    /**
     * ADDRMSK2
     * ===================================================================================================
     * Offset: 0x2C  I2C Slave Address Mask Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |ADDRMSK   |I2C Address Mask Register
     * |        |          |0 = I2C address mask Disabled (the received corresponding register bit should be exactly the same as address register).
     * |        |          |1 = I2C address mask Enabled (the received corresponding address bit is "Don't care").
    */
    __IO uint32_t ADDRMSK2;

    /**
     * ADDRMSK3
     * ===================================================================================================
     * Offset: 0x30  I2C Slave Address Mask Register 3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:1]   |ADDRMSK   |I2C Address Mask Register
     * |        |          |0 = I2C address mask Disabled (the received corresponding register bit should be exactly the same as address register).
     * |        |          |1 = I2C address mask Enabled (the received corresponding address bit is "Don't care").
    */
    __IO uint32_t ADDRMSK3;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED0[2];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * CTL1
     * ===================================================================================================
     * Offset: 0x3C  I2C Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKEN      |Wake-Up Enable
     * |        |          |0 = I2C wake up function Disabled.
     * |        |          |1 = I2C wake up function Enabled.
     * |        |          |The system can be wake up by I2C bus when the system is set into power mode and the received data matched one of the addresses in Address Register.
     * |[1]     |FIFOEN    |FIFO Mode Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Set to enable the two-level FIFO for I2C transmitted or received buffer.
     * |        |          |It is used to improve the performance of the I2C bus.
     * |        |          |If this bit is set = 1, the control bit of STA for repeat start or STO bit should be set after the current SI is clear.
     * |        |          |For example: if there are 4 data shall be transmitted and then stop it.
     * |        |          |The STO bit shall be set after the 3rd data's SI event being clear.
     * |        |          |In this time, the 4th data can be transmitted and the I2C stop after the 4th data transmission done.
     * |[2]     |NSTRETCH  |NO STRETCH The I2C BUS
     * |        |          |0 = The I2C SCL bus is stretched by hardware if the SI is not cleared in master mode.
     * |        |          |1 = The I2C SCL bus is not stretched by hardware if the SI is not cleared in master mode.
     * |[3]     |OVIEN     |I2C OVER RUN Interrupt Control Bit
     * |        |          |Setting OVIEN to enable will send a interrupt to system when the TWOFF bit is enabled and there is over run event in received FIFO.
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[4]     |URIEN     |I2C UNDER RUN Interrupt Control Bit
     * |        |          |Setting URIEN to enable will send a interrupt to system when the TWOFF bit is enabled and there is under run event happened in transmitted FIFO.
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
    */
    __IO uint32_t CTL1;

    /**
     * STATUS1
     * ===================================================================================================
     * Offset: 0x40  I2C Status Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKIF      |I2C Wake-Up Interrupt Flag
     * |        |          |When chip is woken up from Power-Down mode by I2C, this bit is set to 1.
     * |        |          |Software can write 1 to clear this bit.
     * |[1]     |FULL      |I2C TWO LEVEL FIFO FULL
     * |        |          |This bit indicates TX FIFO full or not when the FIFOEN = 1.
     * |[2]     |EMPTY     |I2C TWO LEVEL FIFO EMPTY
     * |        |          |This bit indicates RX FIFO empty or not when the FIFOEN = 1.
     * |[3]     |OVIF      |I2C OVER RUN Status Bit
     * |        |          |This bit indicates the received FIFO is over run when the FIFOEN = 1.
     * |[4]     |URIF      |I2C UNDER RUN Status Bit
     * |        |          |This bit indicates the transmitted FIFO is under run when the FIFOEN = 1.
    */
    __IO uint32_t STATUS1;

} I2C_T;

/**
    @addtogroup I2C_CONST I2C Bit Field Definition
    Constant Definitions for I2C Controller
@{ */

#define I2C_CTL_AA_Pos                   (2)                                               /*!< I2C_T::CTL: AA Position                   */
#define I2C_CTL_AA_Msk                   (0x1ul << I2C_CTL_AA_Pos)                         /*!< I2C_T::CTL: AA Mask                       */

#define I2C_CTL_SI_Pos                   (3)                                               /*!< I2C_T::CTL: SI Position                   */
#define I2C_CTL_SI_Msk                   (0x1ul << I2C_CTL_SI_Pos)                         /*!< I2C_T::CTL: SI Mask                       */

#define I2C_CTL_STO_Pos                  (4)                                               /*!< I2C_T::CTL: STO Position                  */
#define I2C_CTL_STO_Msk                  (0x1ul << I2C_CTL_STO_Pos)                        /*!< I2C_T::CTL: STO Mask                      */

#define I2C_CTL_STA_Pos                  (5)                                               /*!< I2C_T::CTL: STA Position                  */
#define I2C_CTL_STA_Msk                  (0x1ul << I2C_CTL_STA_Pos)                        /*!< I2C_T::CTL: STA Mask                      */

#define I2C_CTL_I2CEN_Pos                (6)                                               /*!< I2C_T::CTL: I2CEN Position                */
#define I2C_CTL_I2CEN_Msk                (0x1ul << I2C_CTL_I2CEN_Pos)                      /*!< I2C_T::CTL: I2CEN Mask                    */

#define I2C_CTL_INTEN_Pos                (7)                                               /*!< I2C_T::CTL: INTEN Position                */
#define I2C_CTL_INTEN_Msk                (0x1ul << I2C_CTL_INTEN_Pos)                      /*!< I2C_T::CTL: INTEN Mask                    */

#define I2C_ADDR0_GC_Pos                 (0)                                               /*!< I2C_T::ADDR0: GC Position                 */
#define I2C_ADDR0_GC_Msk                 (0x1ul << I2C_ADDR0_GC_Pos)                       /*!< I2C_T::ADDR0: GC Mask                     */

#define I2C_ADDR0_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR0: ADDR Position               */
#define I2C_ADDR0_ADDR_Msk               (0x7ful << I2C_ADDR0_ADDR_Pos)                    /*!< I2C_T::ADDR0: ADDR Mask                   */

#define I2C_DAT_DAT_Pos                  (0)                                               /*!< I2C_T::DAT: DAT Position                  */
#define I2C_DAT_DAT_Msk                  (0xfful << I2C_DAT_DAT_Pos)                       /*!< I2C_T::DAT: DAT Mask                      */

#define I2C_STATUS_STATUS_Pos            (0)                                               /*!< I2C_T::STATUS: STATUS Position            */
#define I2C_STATUS_STATUS_Msk            (0xfful << I2C_STATUS_STATUS_Pos)                 /*!< I2C_T::STATUS: STATUS Mask                */

#define I2C_CLKDIV_DIVIDER_Pos           (0)                                               /*!< I2C_T::CLKDIV: DIVIDER Position           */
#define I2C_CLKDIV_DIVIDER_Msk           (0xfful << I2C_CLKDIV_DIVIDER_Pos)                /*!< I2C_T::CLKDIV: DIVIDER Mask               */

#define I2C_TOCTL_TOIF_Pos               (0)                                               /*!< I2C_T::TOCTL: TOIF Position               */
#define I2C_TOCTL_TOIF_Msk               (0x1ul << I2C_TOCTL_TOIF_Pos)                     /*!< I2C_T::TOCTL: TOIF Mask                   */

#define I2C_TOCTL_TOCDIV4_Pos            (1)                                               /*!< I2C_T::TOCTL: TOCDIV4 Position            */
#define I2C_TOCTL_TOCDIV4_Msk            (0x1ul << I2C_TOCTL_TOCDIV4_Pos)                  /*!< I2C_T::TOCTL: TOCDIV4 Mask                */

#define I2C_TOCTL_TOCEN_Pos              (2)                                               /*!< I2C_T::TOCTL: TOCEN Position              */
#define I2C_TOCTL_TOCEN_Msk              (0x1ul << I2C_TOCTL_TOCEN_Pos)                    /*!< I2C_T::TOCTL: TOCEN Mask                  */

#define I2C_ADDR1_GC_Pos                 (0)                                               /*!< I2C_T::ADDR1: GC Position                 */
#define I2C_ADDR1_GC_Msk                 (0x1ul << I2C_ADDR1_GC_Pos)                       /*!< I2C_T::ADDR1: GC Mask                     */

#define I2C_ADDR1_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR1: ADDR Position               */
#define I2C_ADDR1_ADDR_Msk               (0x7ful << I2C_ADDR1_ADDR_Pos)                    /*!< I2C_T::ADDR1: ADDR Mask                   */

#define I2C_ADDR2_GC_Pos                 (0)                                               /*!< I2C_T::ADDR2: GC Position                 */
#define I2C_ADDR2_GC_Msk                 (0x1ul << I2C_ADDR2_GC_Pos)                       /*!< I2C_T::ADDR2: GC Mask                     */

#define I2C_ADDR2_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR2: ADDR Position               */
#define I2C_ADDR2_ADDR_Msk               (0x7ful << I2C_ADDR2_ADDR_Pos)                    /*!< I2C_T::ADDR2: ADDR Mask                   */

#define I2C_ADDR3_GC_Pos                 (0)                                               /*!< I2C_T::ADDR3: GC Position                 */
#define I2C_ADDR3_GC_Msk                 (0x1ul << I2C_ADDR3_GC_Pos)                       /*!< I2C_T::ADDR3: GC Mask                     */

#define I2C_ADDR3_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR3: ADDR Position               */
#define I2C_ADDR3_ADDR_Msk               (0x7ful << I2C_ADDR3_ADDR_Pos)                    /*!< I2C_T::ADDR3: ADDR Mask                   */

#define I2C_ADDRMSK0_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK0: ADDRMSK Position         */
#define I2C_ADDRMSK0_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK0_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK0: ADDRMSK Mask             */

#define I2C_ADDRMSK1_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK1: ADDRMSK Position         */
#define I2C_ADDRMSK1_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK1_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK1: ADDRMSK Mask             */

#define I2C_ADDRMSK2_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK2: ADDRMSK Position         */
#define I2C_ADDRMSK2_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK2_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK2: ADDRMSK Mask             */

#define I2C_ADDRMSK3_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK3: ADDRMSK Position         */
#define I2C_ADDRMSK3_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK3_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK3: ADDRMSK Mask             */

#define I2C_CTL1_WKEN_Pos                (0)                                               /*!< I2C_T::CTL1: WKEN Position                */
#define I2C_CTL1_WKEN_Msk                (0x1ul << I2C_CTL1_WKEN_Pos)                      /*!< I2C_T::CTL1: WKEN Mask                    */

#define I2C_CTL1_FIFOEN_Pos              (1)                                               /*!< I2C_T::CTL1: FIFOEN Position              */
#define I2C_CTL1_FIFOEN_Msk              (0x1ul << I2C_CTL1_FIFOEN_Pos)                    /*!< I2C_T::CTL1: FIFOEN Mask                  */

#define I2C_CTL1_NSTRETCH_Pos            (2)                                               /*!< I2C_T::CTL1: NSTRETCH Position            */
#define I2C_CTL1_NSTRETCH_Msk            (0x1ul << I2C_CTL1_NSTRETCH_Pos)                  /*!< I2C_T::CTL1: NSTRETCH Mask                */

#define I2C_CTL1_OVIEN_Pos               (3)                                               /*!< I2C_T::CTL1: OVIEN Position               */
#define I2C_CTL1_OVIEN_Msk               (0x1ul << I2C_CTL1_OVIEN_Pos)                     /*!< I2C_T::CTL1: OVIEN Mask                   */

#define I2C_CTL1_URIEN_Pos               (4)                                               /*!< I2C_T::CTL1: URIEN Position               */
#define I2C_CTL1_URIEN_Msk               (0x1ul << I2C_CTL1_URIEN_Pos)                     /*!< I2C_T::CTL1: URIEN Mask                   */

#define I2C_STATUS1_WKIF_Pos             (0)                                               /*!< I2C_T::STATUS1: WKIF Position             */
#define I2C_STATUS1_WKIF_Msk             (0x1ul << I2C_STATUS1_WKIF_Pos)                   /*!< I2C_T::STATUS1: WKIF Mask                 */

#define I2C_STATUS1_FULL_Pos             (1)                                               /*!< I2C_T::STATUS1: FULL Position             */
#define I2C_STATUS1_FULL_Msk             (0x1ul << I2C_STATUS1_FULL_Pos)                   /*!< I2C_T::STATUS1: FULL Mask                 */

#define I2C_STATUS1_EMPTY_Pos            (2)                                               /*!< I2C_T::STATUS1: EMPTY Position            */
#define I2C_STATUS1_EMPTY_Msk            (0x1ul << I2C_STATUS1_EMPTY_Pos)                  /*!< I2C_T::STATUS1: EMPTY Mask                */

#define I2C_STATUS1_OVIF_Pos             (3)                                               /*!< I2C_T::STATUS1: OVIF Position             */
#define I2C_STATUS1_OVIF_Msk             (0x1ul << I2C_STATUS1_OVIF_Pos)                   /*!< I2C_T::STATUS1: OVIF Mask                 */

#define I2C_STATUS1_URIF_Pos             (4)                                               /*!< I2C_T::STATUS1: URIF Position             */
#define I2C_STATUS1_URIF_Msk             (0x1ul << I2C_STATUS1_URIF_Pos)                   /*!< I2C_T::STATUS1: URIF Mask                 */

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
     * IRQ0SRC
     * ===================================================================================================
     * Offset: 0x00  IRQ0 (BOD) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ0SRC;

    /**
     * IRQ1SRC
     * ===================================================================================================
     * Offset: 0x04  IRQ1 (WDT) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ1SRC;

    /**
     * IRQ2SRC
     * ===================================================================================================
     * Offset: 0x08  IRQ2 (EINT0) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ2SRC;

    /**
     * IRQ3SRC
     * ===================================================================================================
     * Offset: 0x0C  IRQ3 (EINT1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ3SRC;

    /**
     * IRQ4SRC
     * ===================================================================================================
     * Offset: 0x10  IRQ4 (GP0/1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ4SRC;

    /**
     * IRQ5SRC
     * ===================================================================================================
     * Offset: 0x14  IRQ5 (GP2/3/4) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ5SRC;

    /**
     * IRQ6SRC
     * ===================================================================================================
     * Offset: 0x18  IRQ6 (PWM) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ6SRC;

    /**
     * IRQ7SRC
     * ===================================================================================================
     * Offset: 0x1C  IRQ7 (BRAKE) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ7SRC;

    /**
     * IRQ8SRC
     * ===================================================================================================
     * Offset: 0x20  IRQ8 (TMR0) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ8SRC;

    /**
     * IRQ9SRC
     * ===================================================================================================
     * Offset: 0x24  IRQ9 (TMR1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ9SRC;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED0[2];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * IRQ12SRC
     * ===================================================================================================
     * Offset: 0x30  IRQ12 (UART0) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ12SRC;

    /**
     * IRQ13SRC
     * ===================================================================================================
     * Offset: 0x34  IRQ13 (UART1) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ13SRC;

    /**
     * IRQ14SRC
     * ===================================================================================================
     * Offset: 0x38  IRQ14 (SPI) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ14SRC;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED1[1];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * IRQ16SRC
     * ===================================================================================================
     * Offset: 0x40  IRQ16 (GP5) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ16SRC;

    /**
     * IRQ17SRC
     * ===================================================================================================
     * Offset: 0x44  IRQ17 (HFIRC trim) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ17SRC;

    /**
     * IRQ18SRC
     * ===================================================================================================
     * Offset: 0x48  IRQ18 (I2C) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
    */
    __I  uint32_t IRQ18SRC;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED2[6];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * IRQ25SRC
     * ===================================================================================================
     * Offset: 0x64  IRQ25 (ACMP) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ25SRC;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED3[2];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * IRQ28SRC
     * ===================================================================================================
     * Offset: 0x70  IRQ28 (PWRWU) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ28SRC;

    /**
     * IRQ29SRC
     * ===================================================================================================
     * Offset: 0x74  IRQ29 (ADC) Interrupt Source Identity
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |INT_SRC   |Interrupt Source
     * |        |          |Define the interrupt sources for interrupt event.
    */
    __I  uint32_t IRQ29SRC;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED4[2];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * NMICTL
     * ===================================================================================================
     * Offset: 0x80  NMI Source Interrupt Select Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[4:0]   |NMTSEL    |NMI Interrupt Source Selection
     * |        |          |The NMI interrupt to Cortex-M0 can be selected from one of the peripheral interrupt by setting NMTSEL.
     * |[8]     |NMISELEN  |NMI Interrupt Enable Control (Write Protected)
     * |        |          |0 = NMI interrupt Disabled.
     * |        |          |1 = NMI interrupt Enabled.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGWRPROT at address SYS_BA+0x100.
    */
    __IO uint32_t NMICTL;

    /**
     * IRQSTS
     * ===================================================================================================
     * Offset: 0x84  MCU IRQ Number Identity Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |IRQ       |MCU IRQ Source Register
     * |        |          |The IRQ collects all the interrupts from the peripherals and generates the synchronous interrupt to Cortex-M0 core.
     * |        |          |There are two modes to generate interrupt to Cortex-M0 - the normal mode and test mode.
     * |        |          |The IRQ collects all interrupts from each peripheral and synchronizes them then interrupts the Cortex-M0.
     * |        |          |When the IRQ[n] is 0, setting IRQ[n] to 1 will generate an interrupt to Cortex-M0 NVIC[n].
     * |        |          |When the IRQ[n] is 1 (mean an interrupt is assert), setting 1 to the MCU_bit[n] will clear the interrupt and setting IRQ[n] 0 has no effect.
    */
    __IO uint32_t IRQSTS;

} INTR_T;

/**
    @addtogroup INT_CONST INT Bit Field Definition
    Constant Definitions for INT Controller
@{ */

#define INT_IRQ0SRC_INT_SRC_Pos          (0)                                               /*!< INTR_T::IRQ0SRC: INT_SRC Position          */
#define INT_IRQ0SRC_INT_SRC_Msk          (0x7ul << INT_IRQ0SRC_INT_SRC_Pos)                /*!< INTR_T::IRQ0SRC: INT_SRC Mask              */

#define INT_IRQ1SRC_INT_SRC_Pos          (0)                                               /*!< INTR_T::IRQ1SRC: INT_SRC Position          */
#define INT_IRQ1SRC_INT_SRC_Msk          (0x7ul << INT_IRQ1SRC_INT_SRC_Pos)                /*!< INTR_T::IRQ1SRC: INT_SRC Mask              */

#define INT_IRQ2SRC_INT_SRC_Pos          (0)                                               /*!< INTR_T::IRQ2SRC: INT_SRC Position          */
#define INT_IRQ2SRC_INT_SRC_Msk          (0x7ul << INT_IRQ2SRC_INT_SRC_Pos)                /*!< INTR_T::IRQ2SRC: INT_SRC Mask              */

#define INT_IRQ3SRC_INT_SRC_Pos          (0)                                               /*!< INTR_T::IRQ3SRC: INT_SRC Position          */
#define INT_IRQ3SRC_INT_SRC_Msk          (0x7ul << INT_IRQ3SRC_INT_SRC_Pos)                /*!< INTR_T::IRQ3SRC: INT_SRC Mask              */

#define INT_IRQ4SRC_INT_SRC_Pos          (0)                                               /*!< INTR_T::IRQ4SRC: INT_SRC Position          */
#define INT_IRQ4SRC_INT_SRC_Msk          (0x7ul << INT_IRQ4SRC_INT_SRC_Pos)                /*!< INTR_T::IRQ4SRC: INT_SRC Mask              */

#define INT_IRQ5SRC_INT_SRC_Pos          (0)                                               /*!< INTR_T::IRQ5SRC: INT_SRC Position          */
#define INT_IRQ5SRC_INT_SRC_Msk          (0x7ul << INT_IRQ5SRC_INT_SRC_Pos)                /*!< INTR_T::IRQ5SRC: INT_SRC Mask              */

#define INT_IRQ6SRC_INT_SRC_Pos          (0)                                               /*!< INTR_T::IRQ6SRC: INT_SRC Position          */
#define INT_IRQ6SRC_INT_SRC_Msk          (0x7ul << INT_IRQ6SRC_INT_SRC_Pos)                /*!< INTR_T::IRQ6SRC: INT_SRC Mask              */

#define INT_IRQ7SRC_INT_SRC_Pos          (0)                                               /*!< INTR_T::IRQ7SRC: INT_SRC Position          */
#define INT_IRQ7SRC_INT_SRC_Msk          (0x7ul << INT_IRQ7SRC_INT_SRC_Pos)                /*!< INTR_T::IRQ7SRC: INT_SRC Mask              */

#define INT_IRQ8SRC_INT_SRC_Pos          (0)                                               /*!< INTR_T::IRQ8SRC: INT_SRC Position          */
#define INT_IRQ8SRC_INT_SRC_Msk          (0x7ul << INT_IRQ8SRC_INT_SRC_Pos)                /*!< INTR_T::IRQ8SRC: INT_SRC Mask              */

#define INT_IRQ9SRC_INT_SRC_Pos          (0)                                               /*!< INTR_T::IRQ9SRC: INT_SRC Position          */
#define INT_IRQ9SRC_INT_SRC_Msk          (0x7ul << INT_IRQ9SRC_INT_SRC_Pos)                /*!< INTR_T::IRQ9SRC: INT_SRC Mask              */

#define INT_IRQ12SRC_INT_SRC_Pos         (0)                                               /*!< INTR_T::IRQ12SRC: INT_SRC Position         */
#define INT_IRQ12SRC_INT_SRC_Msk         (0x7ul << INT_IRQ12SRC_INT_SRC_Pos)               /*!< INTR_T::IRQ12SRC: INT_SRC Mask             */

#define INT_IRQ13SRC_INT_SRC_Pos         (0)                                               /*!< INTR_T::IRQ13SRC: INT_SRC Position         */
#define INT_IRQ13SRC_INT_SRC_Msk         (0x7ul << INT_IRQ13SRC_INT_SRC_Pos)               /*!< INTR_T::IRQ13SRC: INT_SRC Mask             */

#define INT_IRQ14SRC_INT_SRC_Pos         (0)                                               /*!< INTR_T::IRQ14SRC: INT_SRC Position         */
#define INT_IRQ14SRC_INT_SRC_Msk         (0x7ul << INT_IRQ14SRC_INT_SRC_Pos)               /*!< INTR_T::IRQ14SRC: INT_SRC Mask             */

#define INT_IRQ16SRC_INT_SRC_Pos         (0)                                               /*!< INTR_T::IRQ16SRC: INT_SRC Position         */
#define INT_IRQ16SRC_INT_SRC_Msk         (0x7ul << INT_IRQ16SRC_INT_SRC_Pos)               /*!< INTR_T::IRQ16SRC: INT_SRC Mask             */

#define INT_IRQ17SRC_INT_SRC_Pos         (0)                                               /*!< INTR_T::IRQ17SRC: INT_SRC Position         */
#define INT_IRQ17SRC_INT_SRC_Msk         (0x7ul << INT_IRQ17SRC_INT_SRC_Pos)               /*!< INTR_T::IRQ17SRC: INT_SRC Mask             */

#define INT_IRQ18SRC_INT_SRC_Pos         (0)                                               /*!< INTR_T::IRQ18SRC: INT_SRC Position         */
#define INT_IRQ18SRC_INT_SRC_Msk         (0x7ul << INT_IRQ18SRC_INT_SRC_Pos)               /*!< INTR_T::IRQ18SRC: INT_SRC Mask             */

#define INT_IRQ25SRC_INT_SRC_Pos         (0)                                               /*!< INTR_T::IRQ25SRC: INT_SRC Position         */
#define INT_IRQ25SRC_INT_SRC_Msk         (0x7ul << INT_IRQ25SRC_INT_SRC_Pos)               /*!< INTR_T::IRQ25SRC: INT_SRC Mask             */

#define INT_IRQ28SRC_INT_SRC_Pos         (0)                                               /*!< INTR_T::IRQ28SRC: INT_SRC Position         */
#define INT_IRQ28SRC_INT_SRC_Msk         (0x7ul << INT_IRQ28SRC_INT_SRC_Pos)               /*!< INTR_T::IRQ28SRC: INT_SRC Mask             */

#define INT_IRQ29SRC_INT_SRC_Pos         (0)                                               /*!< INTR_T::IRQ29SRC: INT_SRC Position         */
#define INT_IRQ29SRC_INT_SRC_Msk         (0x7ul << INT_IRQ29SRC_INT_SRC_Pos)               /*!< INTR_T::IRQ29SRC: INT_SRC Mask             */

#define INT_NMICTL_NMTSEL_Pos            (0)                                               /*!< INTR_T::NMICTL: NMTSEL Position            */
#define INT_NMICTL_NMTSEL_Msk            (0x1ful << INT_NMICTL_NMTSEL_Pos)                 /*!< INTR_T::NMICTL: NMTSEL Mask                */

#define INT_NMICTL_NMISELEN_Pos          (8)                                               /*!< INTR_T::NMICTL: NMISELEN Position          */
#define INT_NMICTL_NMISELEN_Msk          (0x1ul << INT_NMICTL_NMISELEN_Pos)                /*!< INTR_T::NMICTL: NMISELEN Mask              */

#define INT_IRQSTS_IRQ_Pos               (0)                                               /*!< INTR_T::IRQSTS: IRQ Position               */
#define INT_IRQSTS_IRQ_Msk               (0xfffffffful << INT_IRQSTS_IRQ_Pos)              /*!< INTR_T::IRQSTS: IRQ Mask                   */

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
     * CLKPSC
     * ===================================================================================================
     * Offset: 0x00  PWM Pre-scale Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |CLKPSC01  |Clock Prescaler 0 For PWM Counter 0 And 1
     * |        |          |Clock input is divided by (CLKPSC01 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CLKPSC01 = 0, the clock prescaler 0 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
     * |[15:8]  |CLKPSC23  |Clock Prescaler 2 For PWM Counter 2 And 3
     * |        |          |Clock input is divided by (CLKPSC23 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CLKPSC23 = 0, the clock prescaler 2 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
     * |[23:16] |CLKPSC45  |Clock Prescaler 4 For PWM Counter 4 And 5
     * |        |          |Clock input is divided by (CLKPSC45 + 1) before it is fed to the corresponding PWM counter.
     * |        |          |If CLKPSC45 = 0, the clock prescaler 4 output clock will be stopped.
     * |        |          |So the corresponding PWM counter will also be stopped.
    */
    __IO uint32_t CLKPSC;

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x04  PWM Clock Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[2:0]   |CLKDIV0   |Timer 0 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |(Table is the same as CLKDIV5.)
     * |[6:4]   |CLKDIV1   |Timer 1 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |(Table is the same as CLKDIV5.)
     * |[10:8]  |CLKDIV2   |Timer 2 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |(Table is the same as CLKDIV5.)
     * |[14:12] |CLKDIV3   |Timer 3 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |(Table is the same as CLKDIV5.)
     * |[18:16] |CLKDIV4   |Timer 4 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |(Table is the same as CLKDIV5.)
     * |[22:20] |CLKDIV5   |Timer 5 Clock Source Selection
     * |        |          |Select clock input for PWM timer.
     * |        |          |000 = 2 clock input/CLKPSC45/2
     * |        |          |001 = 4 clock input/CLKPSC45/4
     * |        |          |010 = 8 clock input/CLKPSC45/8
     * |        |          |011 = 16 clock input/CLKPSC45/16
     * |        |          |100 = 1 clock input/CLKPSC45/1
     * |        |          |101 = Clock input
     * |        |          |110,111 = Reserved
    */
    __IO uint32_t CLKDIV;

    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x08  PWM Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTEN0    |PWM-Timer 0 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[1]     |DBGTRIOFF |PWM Debug Mode Configuration Bit (Available In DEBUG Mode Only)
     * |        |          |0 = Safe mode: The timer is frozen and PWM outputs are shut down Safe state for the inverter.
     * |        |          |The timer can still be re-started from where it stops.
     * |        |          |1 = Normal mode: The timer continues to operate normally May be dangerous in some cases since a constant duty cycle is applied to the inverter (no more interrupts serviced).
     * |[2]     |PINV0     |PWM-Timer 0 Output Inverter Enabled/Disabled
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[3]     |CNTMODE0  |PWM-Timer 0 Auto-Reload/One-Shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PWM_PERIOD0 and PWM_CMPDAT0 cleared.
     * |[4]     |CNTEN1    |PWM-Timer 1 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[5]     |HCUPDT    |Half Cycle Update Enable For center-Aligned Type
     * |        |          |0 = disable half cycle update PERIOD & CMP.
     * |        |          |1 = enable half cycle update PERIOD & CMP.
     * |[6]     |PINV1     |PWM-Timer 1 Output Inverter Enabled/Disabled
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[7]     |CNTMODE1  |PWM-Timer 1 Auto-Reload/One-Shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PWM_PERIOD1 and PWM_CMPDAT1 cleared.
     * |[8]     |CNTEN2    |PWM-Timer 2 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[10]    |PINV2     |PWM-Timer 2 Output Inverter Enabled/Disabled
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[11]    |CNTMODE2  |PWM-Timer 2 Auto-Reload/One-Shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PWM_PERIOD2 and PWM_CMPDAT2 cleared.
     * |[12]    |CNTEN3    |PWM-Timer 3 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[14]    |PINV3     |PWM-Timer 3 Output Inverter Enabled/Disabled
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[15]    |CNTMODE3  |PWM-Timer 3 Auto-Reload/One-Shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PWM_PERIOD3 and PWM_CMPDAT3 cleared.
     * |[16]    |CNTEN4    |PWM-Timer 4 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[18]    |PINV4     |PWM-Timer 4 Output Inverter Enabled/Disabled
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[19]    |CNTMODE4  |PWM-Timer 4 Auto-Reload/One-Shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PWM_PERIOD4 and PWM_CMPDAT4 cleared.
     * |[20]    |CNTEN5    |PWM-Timer 5 Enable/Disable Start Run
     * |        |          |0 = Corresponding PWM-timer running Stopped.
     * |        |          |1 = Corresponding PWM-timer start run Enabled.
     * |[21]    |ASYMEN    |Asymmetric Mode In Center-Aligned Type
     * |        |          |0 = symmetric mode in center-aligned type.
     * |        |          |1 = asymmetric mode in center-aligned type.
     * |[22]    |PINV5     |PWM-Timer 5 Output Inverter Enabled/Disabled
     * |        |          |0 = Inverter Disabled.
     * |        |          |1 = Inverter Enabled.
     * |[23]    |CNTMODE5  |PWM-Timer 5 Auto-Reload/One-Shot Mode
     * |        |          |0 = One-shot mode.
     * |        |          |1 = Auto-reload mode.
     * |        |          |Note: If there is a rising transition at this bit, it will cause PWM_PERIOD5 and PWM_CMPDAT5 cleared.
     * |[24]    |DTCNT01   |Dead-Zone 0 Generator Enable/Disable (PWM0 And PWM1 Pair For PWM Group)
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: When the dead-zone generator is enabled, the pair of PWM0 and PWM1 becomes a complementary pair for PWM group.
     * |[25]    |DTCNT23   |Dead-Zone 2 Generator Enable/Disable (PWM2 And PWM3 Pair For PWM Group)
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: When the dead-zone generator is enabled, the pair of PWM2 and PWM3 becomes a complementary pair for PWM group.
     * |[26]    |DTCNT45   |Dead-Zone 4 Generator Enable/Disable (PWM4 And PWM5 Pair For PWM Group)
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: When the dead-zone generator is enabled, the pair of PWM4 and PWM5 becomes a complementary pair for PWM group.
     * |[27]    |CNTCLR    |Clear PWM Counter Control Bit
     * |        |          |0 = Do not clear PWM counter.
     * |        |          |1 = All 16-bit PWM counters cleared to 0x0000.
     * |        |          |Note: It is automatically cleared by hardware.
     * |[29:28] |MODE      |PWM Operating Mode Selection
     * |        |          |00 = Independent mode
     * |        |          |01 = Complementary mode
     * |        |          |10 = Synchronized mode
     * |        |          |11 = Reserved
     * |[30]    |GROUPEN   |Group Bit
     * |        |          |0 = The signals timing of all PWM channels are independent.
     * |        |          |1 = Unify the signals timing of PWM0, PWM2 and PWM4 in the same phase which is controlled by PWM0 and also unify the signals timing of PWM1, PWM3 and PWM5 in the same phase which is controlled by PWM1.
     * |[31]    |CNTTYPE   |PWM Aligned Type Selection Bit
     * |        |          |0 = Edge-aligned type.
     * |        |          |1 = Center-aligned type.
    */
    __IO uint32_t CTL;

    /**
     * PERIOD
     * ===================================================================================================
     * Offset: 0x0C ~ 0x20  PWM Counter Register 0 ~ 5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |PERIOD    |PWM Counter/Timer Loaded Value
     * |        |          |PERIOD determines the PWM period.
     * |        |          |Edge-aligned mode:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/(PERIOD+1); where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |Duty ratio = (CMP+1)/(PERIOD+1).
     * |        |          |CMP >= PERIOD: PWM output is always high.
     * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned mode:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/ (2*PERIOD+1); where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |Duty ratio = (PERIOD - CMP)/(PERIOD+1).
     * |        |          |CMP >= PERIOD: PWM output is always low.
     * |        |          |CMP < PERIOD: PWM low width = (CMP + 1) * 2 unit; PWM high width = (PERIOD - CMP) * 2 unit.
     * |        |          |CMP = 0: PWM low width = 2 unit; PWM high width = (PERIOD) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to PERIOD will take effect in next PWM cycle.
    */
    __IO uint32_t PERIOD[6];

    /**
     * CMPDAT
     * ===================================================================================================
     * Offset: 0x24 ~ 0x38  PWM Comparator Register 0 ~ 5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |CMP       |PWM Comparator Register
     * |        |          |CMP determines the PWM duty.
     * |        |          |Edge-aligned mode:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider))/(PERIOD+1); where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |Duty ratio = (CMP+1)/(PERIOD+1).
     * |        |          |CMP >= PERIOD: PWM output is always high.
     * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
     * |        |          |Center-aligned mode:
     * |        |          |PWM frequency = HCLK/((prescale+1)*(clock divider)) /(2*PERIODn+1); where xy, could be 01, 23, 45 depending on the selected PWM channel.
     * |        |          |Duty ratio = (PERIOD - CMP)/(PERIODn+1).
     * |        |          |CMP >= PERIOD: PWM output is always low.
     * |        |          |CMP < PERIOD: PWM low width = (CMP + 1) * 2 unit; PWM high width = (PERIOD - CMP) * 2 unit.
     * |        |          |CMP = 0: PWM low width = 2 unit; PWM high width = (PERIOD) * 2 unit.
     * |        |          |(Unit = One PWM clock cycle).
     * |        |          |Note: Any write to CMP will take effect in next PWM cycle.
     * |[31:16] |CMPD      |PWM Comparator Register For Down Counter In Center-Aligned Asymmetric Mode
     * |        |          |CMPD >= PERIOD: up counter PWM output is always low.
     * |        |          |CMPD >= PERIOD: down counter PWM output is always low.
     * |        |          |Others: PWM output is always high
    */
    __IO uint32_t CMPDAT[6];
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED0[6];
    /// @endcond //HIDDEN_SYMBOLS

    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x54  PWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ZIEN0     |PWM Channel 0 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[1]     |ZIEN1     |PWM Channel 1 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[2]     |ZIEN2     |PWM Channel 2 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[3]     |ZIEN3     |PWM Channel 3 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[4]     |ZIEN4     |PWM Channel 4 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[5]     |ZIEN5     |PWM Channel 5 Period Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[8]     |CMPDIEN0  |PWM Channel 0 Duty Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[9]     |CMPDIEN1  |PWM Channel 1 Duty Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[10]    |CMPDIEN2  |PWM Channel 2 Duty Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[11]    |CMPDIEN3  |PWM Channel 3 Duty Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[12]    |CMPDIEN4  |PWM Channel 4 Duty Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[13]    |CMPDIEN5  |PWM Channel 5 Duty Interrupt Enable
     * |        |          |0 = Disabled. Rising for edge aligned mode. Falling for center aligned mode.
     * |        |          |1 = Enabled.
     * |[16]    |BRKIEN    |Enable Fault Brake0 And 1 Interrupt
     * |        |          |0 = Disabling flags BRKIF0 and BRKIF1 to trigger PWM interrupt.
     * |        |          |1 = Enabling flags BRKIF0 and BRKIF1 can trigger PWM interrupt.
     * |[17]    |PINTTYPE  |PWM Period Interrupt Type Selection
     * |        |          |0 = ZIFn will be set if PWM counter underflows.
     * |        |          |1 = ZIFn will be set if PWM counter matches PERIODn register.
     * |        |          |Note: This bit is effective when PWM in central align mode only.
     * |[18]    |PIEN0     |PWM Channel 0 Central Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[19]    |PIEN1     |PWM Channel 1 Central Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[20]    |PIEN2     |PWM Channel 2 Central Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[21]    |PIEN3     |PWM Channel 3 Central Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[22]    |PIEN4     |PWM Channel 4 Central Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[23]    |PIEN5     |PWM Channel 5 Central Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[24]    |CMPUIEN0  |PWM Channel 0 Rising Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[25]    |CMPUIEN1  |PWM Channel 1 Rising Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[26]    |CMPUIEN2  |PWM Channel 2 Rising Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[27]    |CMPUIEN3  |PWM Channel 3 Rising Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[28]    |CMPUIEN4  |PWM Channel 4 Rising Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[29]    |CMPUIEN5  |PWM Channel 5 Rising Interrupt Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
    */
    __IO uint32_t INTEN;

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x58  PWM Interrupt Indication Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ZIF0      |PWM Channel 0 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM0 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[1]     |ZIF1      |PWM Channel 1 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM1 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[2]     |ZIF2      |PWM Channel 2 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM2 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[3]     |ZIF3      |PWM Channel 3 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM3 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[4]     |ZIF4      |PWM Channel 4 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM4 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[5]     |ZIF5      |PWM Channel 5 Period Interrupt Flag
     * |        |          |Flag is set by hardware when PWM5 down counter reaches zero.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[8]     |CMPDIF0   |PWM Channel 0 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 0 PWM counter reaches PWM_CMPDAT0 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[9]     |CMPDIF1   |PWM Channel 1 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 1 PWM counter reaches PWM_CMPDAT1 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[10]    |CMPDIF2   |PWM Channel 2 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 2 PWM counter reaches PWM_CMPDAT2 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[11]    |CMPDIF3   |PWM Channel 3 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 3 PWM counter reaches PWM_CMPDAT3 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[12]    |CMPDIF4   |PWM Channel 4 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 4 PWM counter reaches PWM_CMPDAT4 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[13]    |CMPDIF5   |PWM Channel 5 Duty Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 5 PWM counter reaches PWM_CMPDAT5 in down-count direction.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[16]    |BRKIF0    |PWM Brake0 Flag
     * |        |          |0 = PWM Brake does not recognize a falling signal at BKP0.
     * |        |          |1 = When PWM Brake detects a falling signal at pin BKP0, this flag will be set to high.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[17]    |BRKIF1    |PWM Brake1 Flag
     * |        |          |0 = PWM Brake does not recognize a falling signal at BKP1.
     * |        |          |1 = When PWM Brake detects a falling signal at pin BKP1, this flag will be set to high.
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[18]    |PIF0      |PWM Channel 0 Center Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 0 PWM rise counter reaches CNT0.
     * |        |          |Software can write 1 to clear this bit.
     * |[19]    |PIF1      |PWM Channel 1 Center Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 1 PWM rise counter reaches CNT1.
     * |        |          |Software can write 1 to clear this bit.
     * |[20]    |PIF2      |PWM Channel 2 Center Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 2 PWM rise counter reaches CNT2.
     * |        |          |Software can write 1 to clear this bit.
     * |[21]    |PIF3      |PWM Channel 3 Center Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 3 PWM rise counter reaches CNT3.
     * |        |          |Software can write 1 to clear this bit.
     * |[22]    |PIF4      |PWM Channel 4 Center Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 4 PWM rise counter reaches CNT4.
     * |        |          |Software can write 1 to clear this bit.
     * |[23]    |PIF5      |PWM Channel 5 Center Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 5 PWM rise counter reaches CNT5.
     * |        |          |Software can write 1 to clear this bit.
     * |[24]    |CMPUIF0   |PWM Channel 1 Rise Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 0 PWM rise counter reaches PWM_CMPDAT0.
     * |        |          |Software can write 1 to clear this bit.
     * |[25]    |CMPUIF1   |PWM Channel 1 Rise Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 1 PWM rise counter reaches PWM_CMPDAT1.
     * |        |          |Software can write 1 to clear this bit.
     * |[26]    |CMPUIF2   |PWM Channel 2 Rise Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 2 PWM rise counter reaches PWM_CMPDAT2.
     * |        |          |Software can write 1 to clear this bit.
     * |[27]    |CMPUIF3   |PWM Channel 3 Rise Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 3 PWM rise counter reaches PWM_CMPDAT3.
     * |        |          |Software can write 1 to clear this bit.
     * |[28]    |CMPUIF4   |PWM Channel 4 Rise Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 4 PWM rise counter reaches PWM_CMPDAT4.
     * |        |          |Software can write 1 to clear this bit.
     * |[29]    |CMPUIF5   |PWM Channel 5 Rise Interrupt Flag
     * |        |          |Flag is set by hardware when a channel 5 PWM rise counter reaches
     * |        |          |PWM_CNT0
     * |        |          |5. Software can write 1 to clear this bit.
    */
    __IO uint32_t INTSTS;

    /**
     * POEN
     * ===================================================================================================
     * Offset: 0x5C  PWM Output Enable for Channel 0~5
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |POEN0     |PWM Channel 0 Output Enable Register
     * |        |          |0 = PWM channel 0 output to pin Disabled.
     * |        |          |1 = PWM channel 0 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
     * |[1]     |POEN1     |PWM Channel 1 Output Enable Register
     * |        |          |0 = PWM channel 1 output to pin Disabled.
     * |        |          |1 = PWM channel 1 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
     * |[2]     |POEN2     |PWM Channel 2 Output Enable Register
     * |        |          |0 = PWM channel 2 output to pin Disabled.
     * |        |          |1 = PWM channel 2 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
     * |[3]     |POEN3     |PWM Channel 3 Output Enable Register
     * |        |          |0 = PWM channel 3 output to pin Disabled.
     * |        |          |1 = PWM channel 3 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
     * |[4]     |POEN4     |PWM Channel 4 Output Enable Register
     * |        |          |0 = PWM channel 4 output to pin Disabled.
     * |        |          |1 = PWM channel 4 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
     * |[5]     |POEN5     |PWM Channel 5 Output Enable Register
     * |        |          |0 = PWM channel 5 output to pin Disabled.
     * |        |          |1 = PWM channel 5 output to pin Enabled.
     * |        |          |Note: The corresponding GPIO pin must be switched to PWM function.
    */
    __IO uint32_t POEN;

    /**
     * BRKCTL
     * ===================================================================================================
     * Offset: 0x60  PWM Fault Brake Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BRK0EN    |Enable BKP0 Pin Trigger Fault Brake Function 0
     * |        |          |0 = Disabling BKP0 pin can trigger brake function 0 (EINT0 or CPO1).
     * |        |          |1 = Enabling a falling at BKP0 pin can trigger brake function 0.
     * |[1]     |BRK1EN    |Enable BKP1 Pin Trigger Fault Brake Function 1
     * |        |          |0 = Disabling BKP1 pin can trigger brake function 1 (EINT1 or CPO0).
     * |        |          |1 = Enabling a falling at BKP1 pin can trigger brake function 1.
     * |[2]     |BRK0SEL   |BKP1 Fault Brake Function Source Selection
     * |        |          |0 = EINT1 as one brake source in BKP1.
     * |        |          |1 = CPO0 as one brake source in BKP1.
     * |[3]     |BRK1SEL   |BKP0 Fault Brake Function Source Selection
     * |        |          |0 = EINT0 as one brake source in BKP0.
     * |        |          |1 = CPO1 as one brake source in BKP0.
     * |[7]     |BRKSTS    |PWM Fault Brake Event Flag (Write 1 Clear)
     * |        |          |0 = PWM output initial state when fault brake conditions asserted.
     * |        |          |1 = PWM output fault brake state when fault brake conditions asserted.
     * |        |          |Software can write 1 to clear this bit and must clear this bit before restart PWM counter.
     * |[8]     |BRKACT    |PWM Brake Type
     * |        |          |0 = PWM counter stop when brake is asserted.
     * |        |          |1 = PWM counter keep going when brake is asserted.
     * |[9]     |SWBRK     |Software Brake
     * |        |          |0 = Disable PWM Software brake and back to normal PWM function.
     * |        |          |1 = Assert PWM Brake immediately.
     * |[24]    |BKOD0     |PWM Channel 0 Brake Output Select Register
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[25]    |BKOD1     |PWM Channel 1 Brake Output Select Register
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[26]    |BKOD2     |PWM Channel 2 Brake Output Select Register
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[27]    |BKOD3     |PWM Channel 3 Brake Output Select Register
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[28]    |BKOD4     |PWM Channel 4 Brake Output Select Register
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[29]    |BKOD5     |PWM Channel 5 Brake Output Select Register
     * |        |          |0 = PWM output low when fault brake conditions asserted.
     * |        |          |1 = PWM output high when fault brake conditions asserted.
     * |[30]    |D6BKOD    |D6 Brake Output Select Register
     * |        |          |0 = D6 output low when fault brake conditions asserted.
     * |        |          |1 = D6 output high when fault brake conditions asserted.
     * |[31]    |D7BKOD    |D7 Brake Output Select Register
     * |        |          |0 = D7 output low when fault brake conditions asserted.
     * |        |          |1 = D7 output high when fault brake conditions asserted.
    */
    __IO uint32_t BRKCTL;

    /**
     * DTCTL
     * ===================================================================================================
     * Offset: 0x64  PWM Dead-zone Interval Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DTCNT01   |Dead-Zone Interval Register For Pair Of Channel0 And Channel1 (PWM0 And PWM1 Pair)
     * |        |          |These 8 bits determine dead-zone length.
     * |        |          |The unit time of dead-zone length is received from corresponding CLKDIV bits.
     * |[15:8]  |DTCNT23   |Dead-Zone Interval Register For Pair Of Channel2 And Channel3 (PWM2 And PWM3 Pair)
     * |        |          |These 8 bits determine dead-zone length.
     * |        |          |The unit time of dead-zone length is received from corresponding CLKDIV bits.
     * |[23:16] |DTCNT45   |Dead-Zone Interval Register For Pair Of Channel4 And Channel5 (PWM4 And PWM5 Pair)
     * |        |          |These 8 bits determine dead-zone length.
     * |        |          |The unit time of dead-zone length is received from corresponding CLKDIV bits.
    */
    __IO uint32_t DTCTL;

    /**
     * ADCTCTL0
     * ===================================================================================================
     * Offset: 0x68  PWM Trigger Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGEN0  |PWM Channel 0 Compare Up Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[1]     |CPTRGEN0  |PWM Channel 0 Center Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[2]     |CDTRGEN0  |PWM Channel 0 Compare Down Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[3]     |ZPTRGEN0  |PWM Channel 0 Zero Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[8]     |CUTRGEN1  |PWM Channel 1 Compare Up Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[9]     |CPTRGEN1  |PWM Channel 1 Center Point Trigger ADC Enable Bit 0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[10]    |CDTRGEN1  |PWM Channel 1 Compare Down Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[11]    |ZPTRGEN1  |PWM Channel 1 Zero Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[16]    |CUTRGEN2  |PWM Channel 2 Compare Up Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[17]    |CPTRGEN2  |PWM Channel 2 Center Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[18]    |CDTRGEN2  |PWM Channel 2 Compare Down Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[19]    |ZPTRGEN2  |PWM Channel 2 Zero Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[24]    |CUTRGEN3  |PWM Channel 3 Compare Up Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[25]    |CPTRGEN3  |PWM Channel 3 Center Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[26]    |CDTRGEN3  |PWM Channel 3 Compare Down Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[27]    |ZPTRGEN3  |PWM Channel 3 Zero Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
    */
    __IO uint32_t ADCTCTL0;

    /**
     * ADCTCTL1
     * ===================================================================================================
     * Offset: 0x6C  PWM Trigger Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGEN4  |PWM Channel 4 Compare Up Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[1]     |CPTRGEN4  |PWM Channel 4 Center Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[2]     |CDTRGEN4  |PWM Channel 4 Compare Down Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[3]     |ZPTRGEN4  |PWM Channel 4 Zero Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[8]     |CUTRGEN5  |PWM Channel 5 Compare Up Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[9]     |CPTRGEN5  |PWM Channel 5 Center Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is only valid for PWM in center aligned mode.
     * |        |          |When PWM is in edged aligned mode, setting this bit is meaningless and will not take any effect.
     * |[10]    |CDTRGEN5  |PWM Channel 5 Compare Down Count Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
     * |[11]    |ZPTRGEN5  |PWM Channel 5 Zero Point Trigger ADC Enable Bit
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit is valid for both center aligned mode and edged aligned mode.
    */
    __IO uint32_t ADCTCTL1;

    /**
     * ADCTSTS0
     * ===================================================================================================
     * Offset: 0x70  PWM Trigger Status Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGF0   |PWM Channel 0 Zero Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[1]     |CPTRGF0   |PWM Channel 0 Compare Down Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[2]     |CDTRGF0   |PWM Channel 0 Center Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[3]     |ZPTRGF0   |PWM Channel 0 Compare Up Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[8]     |CUTRGF1   |PWM Channel 1 Zero Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[9]     |CPTRGF1   |PWM Channel 1 Compare Down Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[10]    |CDTRGF1   |PWM Channel 1 Center Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[11]    |ZPTRGF1   |PWM Channel 1 Compare Up Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[16]    |CUTRGF2   |PWM Channel 3 Zero Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[17]    |CPTRGF2   |PWM Channel 2 Compare Down Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[18]    |CDTRGF2   |PWM Channel 2 Center Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[19]    |ZPTRGF2   |PWM Channel 2 Compare Up Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[24]    |CUTRGF3   |PWM Channel 3 Zero Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[25]    |CPTRGF3   |PWM Channel 3 Compare Down Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[26]    |CDTRGF3   |PWM Channel 3 Center Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[27]    |ZPTRGF3   |PWM Channel 3 Compare Up Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
    */
    __IO uint32_t ADCTSTS0;

    /**
     * ADCTSTS1
     * ===================================================================================================
     * Offset: 0x74  PWM Trigger Status Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CUTRGF4   |PWM Channel 4 Zero Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[1]     |CPTRGF4   |PWM Channel 4 Compare Down Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[2]     |CDTRGF4   |PWM Channel 4 Center Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[3]     |ZPTRGF4   |PWM Channel 4 Compare Up Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[8]     |CUTRGF5   |PWM Channel 5 Zero Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[9]     |CPTRGF5   |PWM Channel 5 Compare Down Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[10]    |CDTRGF5   |PWM Channel 5 Center Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
     * |[11]    |ZPTRGF5   |PWM Channel 5 Compare Up Count Point Trigger ADC Flag
     * |        |          |Note: Software can write 1 to clear this bit.
    */
    __IO uint32_t ADCTSTS1;

    /**
     * PHCHG
     * ===================================================================================================
     * Offset: 0x78  Phase Changed Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MSKDAT0   |MSKDAT0: When MSKEN0 Is Zero, Channel 0's Output Waveform Is MSKDAT0
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[1]     |MSKDAT1   |MSKDAT1: When MSKEN1 Is Zero, Channel 1's Output Waveform Is MSKDAT1
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[2]     |MSKDAT2   |MSKDAT2: When MSKEN2 Is Zero, Channel 2's Output Waveform Is MSKDAT2
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[3]     |MSKDAT3   |MSKDAT3: When MSKEN3 Is Zero, Channel 3's Output Waveform Is MSKDAT3
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[4]     |MSKDAT4   |MSKDAT4: When MSKEN4 Is Zero, Channel 4's Output Waveform Is MSKDAT4
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[5]     |MSKDAT5   |MSKDAT5: When MSKEN5 Is Zero, Channel 5's Output Waveform Is MSKDAT5
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[6]     |MSKDAT6   |MSKDAT6: When MSKEN6 Is 1, Channel 6's Output Waveform Is MSKDAT6
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[7]     |MSKDAT7   |MSKDAT7: When MSKEN7 Is 1, Channel 7's Output Waveform Is MSKDAT7
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[8]     |MSKEN0    |MSKEN Channel 0 Output Enable Control
     * |        |          |0 = Output the original channel 0 waveform.
     * |        |          |1 = Output MSKDAT0 specified in bit 0 of PWM_PHCHG register.
     * |[9]     |MSKEN1    |MSKEN Channel 1 Output Enable Control
     * |        |          |0 = Output the original channel 1 waveform.
     * |        |          |1 = Output MSKDAT1 specified in bit 1 of PWM_PHCHG register.
     * |[10]    |MSKEN2    |MSKEN Channel 2 Output Enable Control
     * |        |          |0 = Output the original channel 2 waveform.
     * |        |          |1 = Output MSKDAT2 specified in bit 2 of PWM_PHCHG register.
     * |[11]    |MSKEN3    |MSKEN Channel 3 Output Enable Control
     * |        |          |0 = Output the original channel 3 waveform.
     * |        |          |1 = Output MSKDAT3 specified in bit 3 of PWM_PHCHG register.
     * |[12]    |MSKEN4    |MSKEN Channel 4 Output Enable Control
     * |        |          |0 = Output the original channel 4 waveform.
     * |        |          |1 = Output MSKDAT4 specified in bit 4 of PWM_PHCHG register.
     * |[13]    |MSKEN5    |MSKEN Channel 5 Output Enable Control
     * |        |          |0 = Output the original channel 5 waveform.
     * |        |          |1 = Output MSKDAT5 specified in bit 5 of PWM_PHCHG register.
     * |[14]    |AUTOCLR0  |Hardware Auto Clear ACMP0TEN When ACMP0 Trigger It
     * |        |          |0 = Enabled.
     * |        |          |1 = Disabled.
     * |[15]    |AUTOCLR1  |Hardware Auto Clear ACMP1TEN When ACMP1 Trigger It
     * |        |          |0 = Enabled.
     * |        |          |1 = Disabled.
     * |[16]    |OFFEN01   |Setting This Bit Will Force MSKEN0 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[17]    |OFFEN11   |Setting This Bit Will Force MSKEN1 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[18]    |OFFEN21   |Setting This Bit Will Force MSKEN2 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[19]    |OFFEN31   |Setting This Bit Will Force MSKEN3 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[21:20] |A1POSSEL  |A1POSSEL
     * |        |          |Select the positive input source of ACMP1.
     * |        |          |00 = Select P3.1 as the input of ACMP1
     * |        |          |01 = Select P3.2 as the input of ACMP1
     * |        |          |10 = Select P3.3 as the input of ACMP1
     * |        |          |11 = Select P3.4 as the input of ACMP1
     * |[22]    |TMR1TEN   |Enable Timer1 Trigger PWM Function
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When this bit is set, timer1 time-out event will update PWM_PHCHG with PWM_PHCHG_NXT register.
     * |[23]    |ACMP1TEN  |Enable ACMP1 Trigger Function
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit will be auto cleared when ACMP1 trigger PWM if AUTOCLR1 is set.
     * |[24]    |OFFEN00   |Setting This Bit Will Force MSKEN0 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[25]    |OFFEN10   |Setting This Bit Will Force MSKEN1 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[26]    |OFFEN20   |Setting This Bit Will Force MSKEN2 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[27]    |OFFEN30   |Setting This Bit Will Force MSKEN3 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[29:28] |A0POSSEL  |A0POSSEL
     * |        |          |Select the positive input source of ACMP0.
     * |        |          |00 = Select P1.5 as the input of ACMP0
     * |        |          |01 = Select P1.0 as the input of ACMP0
     * |        |          |10 = Select P1.2 as the input of ACMP0
     * |        |          |11 = Select P1.3 as the input of ACMP0
     * |[30]    |T0        |Enable Timer0 Trigger PWM Function
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When this bit is set, timer0 time-out event will update PWM_PHCHG with PWM_PHCHG_NXT register.
     * |[31]    |ACMP0TEN  |Enable ACMP0 Trigger Function
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit will be auto cleared when ACMP0 trigger PWM if AUTOCLR0 is set.
    */
    __IO uint32_t PHCHG;

    /**
     * PHCHGNXT
     * ===================================================================================================
     * Offset: 0x7C  Next Phase Change Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MSKDAT0   |MSKDAT0: When MSKEN0 Is Zero, Channel 0's Output Waveform Is MSKDAT0
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[1]     |MSKDAT1   |MSKDAT1: When MSKEN1 Is Zero, Channel 1's Output Waveform Is MSKDAT1
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[2]     |MSKDAT2   |MSKDAT2: When MSKEN2 Is Zero, Channel 2's Output Waveform Is MSKDAT2
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[3]     |MSKDAT3   |MSKDAT3: When MSKEN3 Is Zero, Channel 3's Output Waveform Is MSKDAT3
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[4]     |MSKDAT4   |MSKDAT4: When MSKEN4 Is Zero, Channel 4's Output Waveform Is MSKDAT4
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[5]     |MSKDAT5   |MSKDAT5: When MSKEN5 Is Zero, Channel 5's Output Waveform Is MSKDAT5
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[6]     |MSKDAT6   |MSKDAT6: When MSKEN6 Is 1, Channel 6's Output Waveform Is MSKDAT6
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[7]     |MSKDAT7   |MSKDAT7: When MSKEN7 Is 1, Channel 7's Output Waveform Is MSKDAT7
     * |        |          |0 = Output low.
     * |        |          |1 = Output high.
     * |[8]     |MSKEN0    |MSKEN Channel 0 Output Enable Control
     * |        |          |0 = Output the original channel 0 waveform.
     * |        |          |1 = Output D0 specified in bit 0 of PWM_PHCHG register.
     * |[9]     |MSKEN1    |MSKEN Channel 1 Output Enable Control
     * |        |          |0 = Output the original channel 1 waveform.
     * |        |          |1 = Output MSKDAT1 specified in bit 1 of PWM_PHCHG register.
     * |[10]    |MSKEN2    |MSKEN Channel 2 Output Enable Control
     * |        |          |0 = Output the original channel 2 waveform.
     * |        |          |1 = Output MSKDAT2 specified in bit 2 of PWM_PHCHG register.
     * |[11]    |MSKEN3    |MSKEN Channel 3 Output Enable Control
     * |        |          |0 = Output the original channel 3 waveform.
     * |        |          |1 = Output MSKDAT3 specified in bit 3 of PWM_PHCHG register.
     * |[12]    |MSKEN4    |MSKEN Channel 4 Output Enable Control
     * |        |          |0 = Output the original channel 4 waveform.
     * |        |          |1 = Output MSKDAT4 specified in bit 4 of PWM_PHCHG register.
     * |[13]    |MSKEN5    |MSKEN Channel 5 Output Enable Control
     * |        |          |0 = Output the original channel 5 waveform.
     * |        |          |1 = Output MSKDAT5 specified in bit 5 of PWM_PHCHG register.
     * |[14]    |AUTOCLR0  |Hardware Auto Clear ACMP0TEN When ACMP0 Trigger It
     * |        |          |0 = Enabled.
     * |        |          |1 = Disabled.
     * |[15]    |AUTOCLR1  |Hardware Auto Clear ACMP1TEN When ACMP1 Trigger It
     * |        |          |0 = Enabled.
     * |        |          |1 = Disabled.
     * |[16]    |OFFEN01   |Setting This Bit Will Force MSKEN0 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[17]    |OFFEN11   |Setting This Bit Will Force MSKEN1 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[18]    |OFFEN21   |Setting This Bit Will Force MSKEN2 To Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[19]    |OFFEN31   |Setting This Bit Will Force MSKEN3to Output Low Lasting For At Most One Period Cycle As Long As ACMP1 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[21:20] |A1POSSEL  |A1POSSEL
     * |        |          |Select the positive input source of ACMP1.
     * |        |          |00 = Select P3.1 as the input of ACMP1
     * |        |          |01 = Select P3.2 as the input of ACMP1
     * |        |          |10 = Select P3.3 as the input of ACMP1
     * |        |          |11 = Select P3.4 as the input of ACMP1
     * |[22]    |TMR1TEN   |Enable Timer1 Trigger PWM Function
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When this bit is set, timer1 time-out event will update PWM_PHCHG with PWM_PHCHG_NXT register.
     * |[23]    |ACMP1TEN  |Enable ACMP1 Trigger Function
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit will be auto cleared when ACMP1 trigger PWM if AUTOCLR1 is set.
     * |[24]    |OFFEN00   |Setting This Bit Will Force MSKEN0 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[25]    |OFFEN10   |Setting This Bit Will Force MSKEN1 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[26]    |OFFEN20   |Setting This Bit Will Force MSKEN2 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[27]    |OFFEN30   |Setting This Bit Will Force MSKEN3 To Output Low Lasting For At Most One Period Cycle As Long As ACMP0 Trigger It; This Feature Is Usually In Step Motor Application
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: Only for MSKEN0, MSKEN3, MSKEN2, MSKEN3.
     * |[29:28] |A0POSSEL  |A0POSSEL
     * |        |          |Select the positive input source of ACMP0.
     * |        |          |00 = Select P1.5 as the input of ACMP0
     * |        |          |01 = Select P1.0 as the input of ACMP0
     * |        |          |10 = Select P1.2 as the input of ACMP0
     * |        |          |11 = Select P1.3 as the input of ACMP0
     * |[30]    |TMR0TEN   |Enable Timer0 Trigger PWM Function
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |When this bit is set, timer0 time-out event will update PWM_PHCHG with PWM_PHCHG_NXT register.
     * |[31]    |ACMP0TEN  |Enable ACMP0 Trigger Function
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |        |          |Note: This bit will be auto cleared when ACMP0 trigger PWM if AUTOCLR0 is set.
    */
    __IO uint32_t PHCHGNXT;

    /**
     * PHCHGMSK
     * ===================================================================================================
     * Offset: 0x80  Phase Change MASK Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6]     |MASKEND6  |MASK For D6
     * |        |          |0 = Original GPIO P0.1.
     * |        |          |1 = D6.
     * |[7]     |MASKEND7  |MASK For D7
     * |        |          |0 = Original GPIO P0.0.
     * |        |          |1 = D7.
     * |[8]     |POSCTL0   |ACMP0 Positive Input Selection Control
     * |        |          |0 = The input of ACMP0 is controlled by CMP0CR.
     * |        |          |1 = The input of ACMP0 is controlled by A0POSSEL of PWM_PHCHG register.
     * |        |          |Note: Register CMP0CR is describe in Comparator Controller chapter
     * |[9]     |POSCTL1   |ACMP1 Positive Input Selection Control
     * |        |          |0 = The input of ACMP1 is controlled by CMP1CR.
     * |        |          |1 = The input of ACMP1 is controlled by A1POSSEL of PWM_PHCHG register.
     * |        |          |Note: Register CMP1CR is describe in Comparator Controller chapter
    */
    __IO uint32_t PHCHGMSK;

    /**
     * IFA
     * ===================================================================================================
     * Offset: 0x84  Period Interrupt Accumulation Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |IFAEN     |Interrupt Accumulation Enable
     * |        |          |0 = Disabled.
     * |        |          |1 = Enabled.
     * |[7:4]   |IFCNT     |Interrupt Accumulation Count
     * |        |          |When IFCNT is set, IFCNT will decrease when every ZIF0 flag is set and when IFCNT reach to zero, the PWM0 interrupt will occurred and IFCNT will reload itself.
    */
    __IO uint32_t IFA;

} PWM_T;

/**
    @addtogroup PWM_CONST PWM Bit Field Definition
    Constant Definitions for PWM Controller
@{ */

#define PWM_CLKPSC_CLKPSC01_Pos          (0)                                               /*!< PWM_T::CLKPSC: CLKPSC01 Position          */
#define PWM_CLKPSC_CLKPSC01_Msk          (0xfful << PWM_CLKPSC_CLKPSC01_Pos)               /*!< PWM_T::CLKPSC: CLKPSC01 Mask              */

#define PWM_CLKPSC_CLKPSC23_Pos          (8)                                               /*!< PWM_T::CLKPSC: CLKPSC23 Position          */
#define PWM_CLKPSC_CLKPSC23_Msk          (0xfful << PWM_CLKPSC_CLKPSC23_Pos)               /*!< PWM_T::CLKPSC: CLKPSC23 Mask              */

#define PWM_CLKPSC_CLKPSC45_Pos          (16)                                              /*!< PWM_T::CLKPSC: CLKPSC45 Position          */
#define PWM_CLKPSC_CLKPSC45_Msk          (0xfful << PWM_CLKPSC_CLKPSC45_Pos)               /*!< PWM_T::CLKPSC: CLKPSC45 Mask              */

#define PWM_CLKDIV_CLKDIV0_Pos           (0)                                               /*!< PWM_T::CLKDIV: CLKDIV0 Position           */
#define PWM_CLKDIV_CLKDIV0_Msk           (0x7ul << PWM_CLKDIV_CLKDIV0_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV0 Mask               */

#define PWM_CLKDIV_CLKDIV1_Pos           (4)                                               /*!< PWM_T::CLKDIV: CLKDIV1 Position           */
#define PWM_CLKDIV_CLKDIV1_Msk           (0x7ul << PWM_CLKDIV_CLKDIV1_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV1 Mask               */

#define PWM_CLKDIV_CLKDIV2_Pos           (8)                                               /*!< PWM_T::CLKDIV: CLKDIV2 Position           */
#define PWM_CLKDIV_CLKDIV2_Msk           (0x7ul << PWM_CLKDIV_CLKDIV2_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV2 Mask               */

#define PWM_CLKDIV_CLKDIV3_Pos           (12)                                              /*!< PWM_T::CLKDIV: CLKDIV3 Position           */
#define PWM_CLKDIV_CLKDIV3_Msk           (0x7ul << PWM_CLKDIV_CLKDIV3_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV3 Mask               */

#define PWM_CLKDIV_CLKDIV4_Pos           (16)                                              /*!< PWM_T::CLKDIV: CLKDIV4 Position           */
#define PWM_CLKDIV_CLKDIV4_Msk           (0x7ul << PWM_CLKDIV_CLKDIV4_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV4 Mask               */

#define PWM_CLKDIV_CLKDIV5_Pos           (20)                                              /*!< PWM_T::CLKDIV: CLKDIV5 Position           */
#define PWM_CLKDIV_CLKDIV5_Msk           (0x7ul << PWM_CLKDIV_CLKDIV5_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV5 Mask               */

#define PWM_CTL_CNTEN0_Pos               (0)                                               /*!< PWM_T::CTL: CNTEN0 Position               */
#define PWM_CTL_CNTEN0_Msk               (0x1ul << PWM_CTL_CNTEN0_Pos)                     /*!< PWM_T::CTL: CNTEN0 Mask                   */

#define PWM_CTL_DBGTRIOFF_Pos            (1)                                               /*!< PWM_T::CTL: DBGTRIOFF Position            */
#define PWM_CTL_DBGTRIOFF_Msk            (0x1ul << PWM_CTL_DBGTRIOFF_Pos)                  /*!< PWM_T::CTL: DBGTRIOFF Mask                */

#define PWM_CTL_PINV0_Pos                (2)                                               /*!< PWM_T::CTL: PINV0 Position                */
#define PWM_CTL_PINV0_Msk                (0x1ul << PWM_CTL_PINV0_Pos)                      /*!< PWM_T::CTL: PINV0 Mask                    */

#define PWM_CTL_CNTMODE0_Pos             (3)                                               /*!< PWM_T::CTL: CNTMODE0 Position             */
#define PWM_CTL_CNTMODE0_Msk             (0x1ul << PWM_CTL_CNTMODE0_Pos)                   /*!< PWM_T::CTL: CNTMODE0 Mask                 */

#define PWM_CTL_CNTEN1_Pos               (4)                                               /*!< PWM_T::CTL: CNTEN1 Position               */
#define PWM_CTL_CNTEN1_Msk               (0x1ul << PWM_CTL_CNTEN1_Pos)                     /*!< PWM_T::CTL: CNTEN1 Mask                   */

#define PWM_CTL_HCUPDT_Pos               (5)                                               /*!< PWM_T::CTL: HCUPDT Position               */
#define PWM_CTL_HCUPDT_Msk               (0x1ul << PWM_CTL_HCUPDT_Pos)                     /*!< PWM_T::CTL: HCUPDT Mask                   */

#define PWM_CTL_PINV1_Pos                (6)                                               /*!< PWM_T::CTL: PINV1 Position              */
#define PWM_CTL_PINV1_Msk                (0x1ul << PWM_CTL_PINV1_Pos)                      /*!< PWM_T::CTL: PINV1 Mask                  */

#define PWM_CTL_CNTMODE1_Pos             (7)                                               /*!< PWM_T::CTL: CNTMODE1 Position             */
#define PWM_CTL_CNTMODE1_Msk             (0x1ul << PWM_CTL_CNTMODE1_Pos)                   /*!< PWM_T::CTL: CNTMODE1 Mask                 */

#define PWM_CTL_CNTEN2_Pos               (8)                                               /*!< PWM_T::CTL: CNTEN2 Position               */
#define PWM_CTL_CNTEN2_Msk               (0x1ul << PWM_CTL_CNTEN2_Pos)                     /*!< PWM_T::CTL: CNTEN2 Mask                   */

#define PWM_CTL_PINV2_Pos                (10)                                              /*!< PWM_T::CTL: PINV2 Position              */
#define PWM_CTL_PINV2_Msk                (0x1ul << PWM_CTL_PINV2_Pos)                      /*!< PWM_T::CTL: PINV2 Mask                  */

#define PWM_CTL_CNTMODE2_Pos             (11)                                              /*!< PWM_T::CTL: CNTMODE2 Position             */
#define PWM_CTL_CNTMODE2_Msk             (0x1ul << PWM_CTL_CNTMODE2_Pos)                   /*!< PWM_T::CTL: CNTMODE2 Mask                 */

#define PWM_CTL_CNTEN3_Pos               (12)                                              /*!< PWM_T::CTL: CNTEN3 Position               */
#define PWM_CTL_CNTEN3_Msk               (0x1ul << PWM_CTL_CNTEN3_Pos)                     /*!< PWM_T::CTL: CNTEN3 Mask                   */

#define PWM_CTL_PINV3_Pos                (14)                                              /*!< PWM_T::CTL: PINV3 Position              */
#define PWM_CTL_PINV3_Msk                (0x1ul << PWM_CTL_PINV3_Pos)                      /*!< PWM_T::CTL: PINV3 Mask                  */

#define PWM_CTL_CNTMODE3_Pos             (15)                                              /*!< PWM_T::CTL: CNTMODE3 Position             */
#define PWM_CTL_CNTMODE3_Msk             (0x1ul << PWM_CTL_CNTMODE3_Pos)                   /*!< PWM_T::CTL: CNTMODE3 Mask                 */

#define PWM_CTL_CNTEN4_Pos               (16)                                              /*!< PWM_T::CTL: CNTEN4 Position               */
#define PWM_CTL_CNTEN4_Msk               (0x1ul << PWM_CTL_CNTEN4_Pos)                     /*!< PWM_T::CTL: CNTEN4 Mask                   */

#define PWM_CTL_PINV4_Pos                (18)                                              /*!< PWM_T::CTL: PINV4 Position              */
#define PWM_CTL_PINV4_Msk                (0x1ul << PWM_CTL_PINV4_Pos)                      /*!< PWM_T::CTL: PINV4 Mask                  */

#define PWM_CTL_CNTMODE4_Pos             (19)                                              /*!< PWM_T::CTL: CNTMODE4 Position             */
#define PWM_CTL_CNTMODE4_Msk             (0x1ul << PWM_CTL_CNTMODE4_Pos)                   /*!< PWM_T::CTL: CNTMODE4 Mask                 */

#define PWM_CTL_CNTEN5_Pos               (20)                                              /*!< PWM_T::CTL: CNTEN5 Position               */
#define PWM_CTL_CNTEN5_Msk               (0x1ul << PWM_CTL_CNTEN5_Pos)                     /*!< PWM_T::CTL: CNTEN5 Mask                   */

#define PWM_CTL_ASYMEN_Pos               (21)                                              /*!< PWM_T::CTL: ASYMEN Position               */
#define PWM_CTL_ASYMEN_Msk               (0x1ul << PWM_CTL_ASYMEN_Pos)                     /*!< PWM_T::CTL: ASYMEN Mask                   */

#define PWM_CTL_PINV5_Pos                (22)                                              /*!< PWM_T::CTL: PINV5 Position              */
#define PWM_CTL_PINV5_Msk                (0x1ul << PWM_CTL_PINV5_Pos)                      /*!< PWM_T::CTL: PINV5 Mask                  */

#define PWM_CTL_CNTMODE5_Pos             (23)                                              /*!< PWM_T::CTL: CNTMODE5 Position             */
#define PWM_CTL_CNTMODE5_Msk             (0x1ul << PWM_CTL_CNTMODE5_Pos)                   /*!< PWM_T::CTL: CNTMODE5 Mask                 */

#define PWM_CTL_DTCNT01_Pos              (24)                                              /*!< PWM_T::CTL: DTCNT01 Position              */
#define PWM_CTL_DTCNT01_Msk              (0x1ul << PWM_CTL_DTCNT01_Pos)                    /*!< PWM_T::CTL: DTCNT01 Mask                  */

#define PWM_CTL_DTCNT23_Pos              (25)                                              /*!< PWM_T::CTL: DTCNT23 Position              */
#define PWM_CTL_DTCNT23_Msk              (0x1ul << PWM_CTL_DTCNT23_Pos)                    /*!< PWM_T::CTL: DTCNT23 Mask                  */

#define PWM_CTL_DTCNT45_Pos              (26)                                              /*!< PWM_T::CTL: DTCNT45 Position              */
#define PWM_CTL_DTCNT45_Msk              (0x1ul << PWM_CTL_DTCNT45_Pos)                    /*!< PWM_T::CTL: DTCNT45 Mask                  */

#define PWM_CTL_CNTCLR_Pos               (27)                                              /*!< PWM_T::CTL: CNTCLR Position               */
#define PWM_CTL_CNTCLR_Msk               (0x1ul << PWM_CTL_CNTCLR_Pos)                     /*!< PWM_T::CTL: CNTCLR Mask                   */

#define PWM_CTL_MODE_Pos                 (28)                                              /*!< PWM_T::CTL: MODE Position                 */
#define PWM_CTL_MODE_Msk                 (0x3ul << PWM_CTL_MODE_Pos)                       /*!< PWM_T::CTL: MODE Mask                     */

#define PWM_CTL_GROUPEN_Pos              (30)                                              /*!< PWM_T::CTL: GROUPEN Position              */
#define PWM_CTL_GROUPEN_Msk              (0x1ul << PWM_CTL_GROUPEN_Pos)                    /*!< PWM_T::CTL: GROUPEN Mask                  */

#define PWM_CTL_CNTTYPE_Pos              (31)                                              /*!< PWM_T::CTL: CNTTYPE Position              */
#define PWM_CTL_CNTTYPE_Msk              (0x1ul << PWM_CTL_CNTTYPE_Pos)                    /*!< PWM_T::CTL: CNTTYPE Mask                  */

#define PWM_PERIOD_PERIOD_Pos            (0)                                               /*!< PWM_T::PERIOD: PERIOD Position           */
#define PWM_PERIOD_PERIOD_Msk            (0xfffful << PWM_PERIOD_PERIOD_Pos)               /*!< PWM_T::PERIOD: PERIOD Mask               */

#define PWM_CMPDAT_CMP_Pos               (0)                                               /*!< PWM_T::CMPDAT: CMP Position              */
#define PWM_CMPDAT_CMP_Msk               (0xfffful << PWM_CMPDAT_CMP_Pos)                  /*!< PWM_T::CMPDAT: CMP Mask                  */

#define PWM_CMPDAT_CMPD_Pos              (16)                                              /*!< PWM_T::CMPDAT: CMPD Position             */
#define PWM_CMPDAT_CMPD_Msk              (0xfffful << PWM_CMPDAT_CMPD_Pos)                 /*!< PWM_T::CMPDAT: CMPD Mask                 */

#define PWM_INTEN_ZIEN0_Pos              (0)                                               /*!< PWM_T::INTEN: ZIEN0 Position              */
#define PWM_INTEN_ZIEN0_Msk              (0x1ul << PWM_INTEN_ZIEN0_Pos)                    /*!< PWM_T::INTEN: ZIEN0 Mask                  */

#define PWM_INTEN_ZIEN1_Pos              (1)                                               /*!< PWM_T::INTEN: ZIEN1 Position              */
#define PWM_INTEN_ZIEN1_Msk              (0x1ul << PWM_INTEN_ZIEN1_Pos)                    /*!< PWM_T::INTEN: ZIEN1 Mask                  */

#define PWM_INTEN_ZIEN2_Pos              (2)                                               /*!< PWM_T::INTEN: ZIEN2 Position              */
#define PWM_INTEN_ZIEN2_Msk              (0x1ul << PWM_INTEN_ZIEN2_Pos)                    /*!< PWM_T::INTEN: ZIEN2 Mask                  */

#define PWM_INTEN_ZIEN3_Pos              (3)                                               /*!< PWM_T::INTEN: ZIEN3 Position              */
#define PWM_INTEN_ZIEN3_Msk              (0x1ul << PWM_INTEN_ZIEN3_Pos)                    /*!< PWM_T::INTEN: ZIEN3 Mask                  */

#define PWM_INTEN_ZIEN4_Pos              (4)                                               /*!< PWM_T::INTEN: ZIEN4 Position              */
#define PWM_INTEN_ZIEN4_Msk              (0x1ul << PWM_INTEN_ZIEN4_Pos)                    /*!< PWM_T::INTEN: ZIEN4 Mask                  */

#define PWM_INTEN_ZIEN5_Pos              (5)                                               /*!< PWM_T::INTEN: ZIEN5 Position              */
#define PWM_INTEN_ZIEN5_Msk              (0x1ul << PWM_INTEN_ZIEN5_Pos)                    /*!< PWM_T::INTEN: ZIEN5 Mask                  */

#define PWM_INTEN_CMPDIEN0_Pos           (8)                                               /*!< PWM_T::INTEN: CMPDIEN0 Position           */
#define PWM_INTEN_CMPDIEN0_Msk           (0x1ul << PWM_INTEN_CMPDIEN0_Pos)                 /*!< PWM_T::INTEN: CMPDIEN0 Mask               */

#define PWM_INTEN_CMPDIEN1_Pos           (9)                                               /*!< PWM_T::INTEN: CMPDIEN1 Position           */
#define PWM_INTEN_CMPDIEN1_Msk           (0x1ul << PWM_INTEN_CMPDIEN1_Pos)                 /*!< PWM_T::INTEN: CMPDIEN1 Mask               */

#define PWM_INTEN_CMPDIEN2_Pos           (10)                                              /*!< PWM_T::INTEN: CMPDIEN2 Position           */
#define PWM_INTEN_CMPDIEN2_Msk           (0x1ul << PWM_INTEN_CMPDIEN2_Pos)                 /*!< PWM_T::INTEN: CMPDIEN2 Mask               */

#define PWM_INTEN_CMPDIEN3_Pos           (11)                                              /*!< PWM_T::INTEN: CMPDIEN3 Position           */
#define PWM_INTEN_CMPDIEN3_Msk           (0x1ul << PWM_INTEN_CMPDIEN3_Pos)                 /*!< PWM_T::INTEN: CMPDIEN3 Mask               */

#define PWM_INTEN_CMPDIEN4_Pos           (12)                                              /*!< PWM_T::INTEN: CMPDIEN4 Position           */
#define PWM_INTEN_CMPDIEN4_Msk           (0x1ul << PWM_INTEN_CMPDIEN4_Pos)                 /*!< PWM_T::INTEN: CMPDIEN4 Mask               */

#define PWM_INTEN_CMPDIEN5_Pos           (13)                                              /*!< PWM_T::INTEN: CMPDIEN5 Position           */
#define PWM_INTEN_CMPDIEN5_Msk           (0x1ul << PWM_INTEN_CMPDIEN5_Pos)                 /*!< PWM_T::INTEN: CMPDIEN5 Mask               */

#define PWM_INTEN_BRKIEN_Pos             (16)                                              /*!< PWM_T::INTEN: BRKIEN Position             */
#define PWM_INTEN_BRKIEN_Msk             (0x1ul << PWM_INTEN_BRKIEN_Pos)                   /*!< PWM_T::INTEN: BRKIEN Mask                 */

#define PWM_INTEN_PINTTYPE_Pos           (17)                                              /*!< PWM_T::INTEN: PINTTYPE Position           */
#define PWM_INTEN_PINTTYPE_Msk           (0x1ul << PWM_INTEN_PINTTYPE_Pos)                 /*!< PWM_T::INTEN: PINTTYPE Mask               */

#define PWM_INTEN_PIEN0_Pos              (18)                                              /*!< PWM_T::INTEN: PIEN0 Position              */
#define PWM_INTEN_PIEN0_Msk              (0x1ul << PWM_INTEN_PIEN0_Pos)                    /*!< PWM_T::INTEN: PIEN0 Mask                  */

#define PWM_INTEN_PIEN1_Pos              (19)                                              /*!< PWM_T::INTEN: PIEN1 Position              */
#define PWM_INTEN_PIEN1_Msk              (0x1ul << PWM_INTEN_PIEN1_Pos)                    /*!< PWM_T::INTEN: PIEN1 Mask                  */

#define PWM_INTEN_PIEN2_Pos              (20)                                              /*!< PWM_T::INTEN: PIEN2 Position              */
#define PWM_INTEN_PIEN2_Msk              (0x1ul << PWM_INTEN_PIEN2_Pos)                    /*!< PWM_T::INTEN: PIEN2 Mask                  */

#define PWM_INTEN_PIEN3_Pos              (21)                                              /*!< PWM_T::INTEN: PIEN3 Position              */
#define PWM_INTEN_PIEN3_Msk              (0x1ul << PWM_INTEN_PIEN3_Pos)                    /*!< PWM_T::INTEN: PIEN3 Mask                  */

#define PWM_INTEN_PIEN4_Pos              (22)                                              /*!< PWM_T::INTEN: PIEN4 Position              */
#define PWM_INTEN_PIEN4_Msk              (0x1ul << PWM_INTEN_PIEN4_Pos)                    /*!< PWM_T::INTEN: PIEN4 Mask                  */

#define PWM_INTEN_PIEN5_Pos              (23)                                              /*!< PWM_T::INTEN: PIEN5 Position              */
#define PWM_INTEN_PIEN5_Msk              (0x1ul << PWM_INTEN_PIEN5_Pos)                    /*!< PWM_T::INTEN: PIEN5 Mask                  */

#define PWM_INTEN_CMPUIEN0_Pos           (24)                                              /*!< PWM_T::INTEN: CMPUIEN0 Position           */
#define PWM_INTEN_CMPUIEN0_Msk           (0x1ul << PWM_INTEN_CMPUIEN0_Pos)                 /*!< PWM_T::INTEN: CMPUIEN0 Mask               */

#define PWM_INTEN_CMPUIEN1_Pos           (25)                                              /*!< PWM_T::INTEN: CMPUIEN1 Position           */
#define PWM_INTEN_CMPUIEN1_Msk           (0x1ul << PWM_INTEN_CMPUIEN1_Pos)                 /*!< PWM_T::INTEN: CMPUIEN1 Mask               */

#define PWM_INTEN_CMPUIEN2_Pos           (26)                                              /*!< PWM_T::INTEN: CMPUIEN2 Position           */
#define PWM_INTEN_CMPUIEN2_Msk           (0x1ul << PWM_INTEN_CMPUIEN2_Pos)                 /*!< PWM_T::INTEN: CMPUIEN2 Mask               */

#define PWM_INTEN_CMPUIEN3_Pos           (27)                                              /*!< PWM_T::INTEN: CMPUIEN3 Position           */
#define PWM_INTEN_CMPUIEN3_Msk           (0x1ul << PWM_INTEN_CMPUIEN3_Pos)                 /*!< PWM_T::INTEN: CMPUIEN3 Mask               */

#define PWM_INTEN_CMPUIEN4_Pos           (28)                                              /*!< PWM_T::INTEN: CMPUIEN4 Position           */
#define PWM_INTEN_CMPUIEN4_Msk           (0x1ul << PWM_INTEN_CMPUIEN4_Pos)                 /*!< PWM_T::INTEN: CMPUIEN4 Mask               */

#define PWM_INTEN_CMPUIEN5_Pos           (29)                                              /*!< PWM_T::INTEN: CMPUIEN5 Position           */
#define PWM_INTEN_CMPUIEN5_Msk           (0x1ul << PWM_INTEN_CMPUIEN5_Pos)                 /*!< PWM_T::INTEN: CMPUIEN5 Mask               */

#define PWM_INTSTS_ZIF0_Pos              (0)                                               /*!< PWM_T::INTSTS: ZIF0 Position              */
#define PWM_INTSTS_ZIF0_Msk              (0x1ul << PWM_INTSTS_ZIF0_Pos)                    /*!< PWM_T::INTSTS: ZIF0 Mask                  */

#define PWM_INTSTS_ZIF1_Pos              (1)                                               /*!< PWM_T::INTSTS: ZIF1 Position              */
#define PWM_INTSTS_ZIF1_Msk              (0x1ul << PWM_INTSTS_ZIF1_Pos)                    /*!< PWM_T::INTSTS: ZIF1 Mask                  */

#define PWM_INTSTS_ZIF2_Pos              (2)                                               /*!< PWM_T::INTSTS: ZIF2 Position              */
#define PWM_INTSTS_ZIF2_Msk              (0x1ul << PWM_INTSTS_ZIF2_Pos)                    /*!< PWM_T::INTSTS: ZIF2 Mask                  */

#define PWM_INTSTS_ZIF3_Pos              (3)                                               /*!< PWM_T::INTSTS: ZIF3 Position              */
#define PWM_INTSTS_ZIF3_Msk              (0x1ul << PWM_INTSTS_ZIF3_Pos)                    /*!< PWM_T::INTSTS: ZIF3 Mask                  */

#define PWM_INTSTS_ZIF4_Pos              (4)                                               /*!< PWM_T::INTSTS: ZIF4 Position              */
#define PWM_INTSTS_ZIF4_Msk              (0x1ul << PWM_INTSTS_ZIF4_Pos)                    /*!< PWM_T::INTSTS: ZIF4 Mask                  */

#define PWM_INTSTS_ZIF5_Pos              (5)                                               /*!< PWM_T::INTSTS: ZIF5 Position              */
#define PWM_INTSTS_ZIF5_Msk              (0x1ul << PWM_INTSTS_ZIF5_Pos)                    /*!< PWM_T::INTSTS: ZIF5 Mask                  */

#define PWM_INTSTS_CMPDIF0_Pos           (8)                                               /*!< PWM_T::INTSTS: CMPDIF0 Position           */
#define PWM_INTSTS_CMPDIF0_Msk           (0x1ul << PWM_INTSTS_CMPDIF0_Pos)                 /*!< PWM_T::INTSTS: CMPDIF0 Mask               */

#define PWM_INTSTS_CMPDIF1_Pos           (9)                                               /*!< PWM_T::INTSTS: CMPDIF1 Position           */
#define PWM_INTSTS_CMPDIF1_Msk           (0x1ul << PWM_INTSTS_CMPDIF1_Pos)                 /*!< PWM_T::INTSTS: CMPDIF1 Mask               */

#define PWM_INTSTS_CMPDIF2_Pos           (10)                                              /*!< PWM_T::INTSTS: CMPDIF2 Position           */
#define PWM_INTSTS_CMPDIF2_Msk           (0x1ul << PWM_INTSTS_CMPDIF2_Pos)                 /*!< PWM_T::INTSTS: CMPDIF2 Mask               */

#define PWM_INTSTS_CMPDIF3_Pos           (11)                                              /*!< PWM_T::INTSTS: CMPDIF3 Position           */
#define PWM_INTSTS_CMPDIF3_Msk           (0x1ul << PWM_INTSTS_CMPDIF3_Pos)                 /*!< PWM_T::INTSTS: CMPDIF3 Mask               */

#define PWM_INTSTS_CMPDIF4_Pos           (12)                                              /*!< PWM_T::INTSTS: CMPDIF4 Position           */
#define PWM_INTSTS_CMPDIF4_Msk           (0x1ul << PWM_INTSTS_CMPDIF4_Pos)                 /*!< PWM_T::INTSTS: CMPDIF4 Mask               */

#define PWM_INTSTS_CMPDIF5_Pos           (13)                                              /*!< PWM_T::INTSTS: CMPDIF5 Position           */
#define PWM_INTSTS_CMPDIF5_Msk           (0x1ul << PWM_INTSTS_CMPDIF5_Pos)                 /*!< PWM_T::INTSTS: CMPDIF5 Mask               */

#define PWM_INTSTS_BRKIF0_Pos            (16)                                              /*!< PWM_T::INTSTS: BRKIF0 Position            */
#define PWM_INTSTS_BRKIF0_Msk            (0x1ul << PWM_INTSTS_BRKIF0_Pos)                  /*!< PWM_T::INTSTS: BRKIF0 Mask                */

#define PWM_INTSTS_BRKIF1_Pos            (17)                                              /*!< PWM_T::INTSTS: BRKIF1 Position            */
#define PWM_INTSTS_BRKIF1_Msk            (0x1ul << PWM_INTSTS_BRKIF1_Pos)                  /*!< PWM_T::INTSTS: BRKIF1 Mask                */

#define PWM_INTSTS_PIF0_Pos              (18)                                              /*!< PWM_T::INTSTS: PIF0 Position              */
#define PWM_INTSTS_PIF0_Msk              (0x1ul << PWM_INTSTS_PIF0_Pos)                    /*!< PWM_T::INTSTS: PIF0 Mask                  */

#define PWM_INTSTS_PIF1_Pos              (19)                                              /*!< PWM_T::INTSTS: PIF1 Position              */
#define PWM_INTSTS_PIF1_Msk              (0x1ul << PWM_INTSTS_PIF1_Pos)                    /*!< PWM_T::INTSTS: PIF1 Mask                  */

#define PWM_INTSTS_PIF2_Pos              (20)                                              /*!< PWM_T::INTSTS: PIF2 Position              */
#define PWM_INTSTS_PIF2_Msk              (0x1ul << PWM_INTSTS_PIF2_Pos)                    /*!< PWM_T::INTSTS: PIF2 Mask                  */

#define PWM_INTSTS_PIF3_Pos              (21)                                              /*!< PWM_T::INTSTS: PIF3 Position              */
#define PWM_INTSTS_PIF3_Msk              (0x1ul << PWM_INTSTS_PIF3_Pos)                    /*!< PWM_T::INTSTS: PIF3 Mask                  */

#define PWM_INTSTS_PIF4_Pos              (22)                                              /*!< PWM_T::INTSTS: PIF4 Position              */
#define PWM_INTSTS_PIF4_Msk              (0x1ul << PWM_INTSTS_PIF4_Pos)                    /*!< PWM_T::INTSTS: PIF4 Mask                  */

#define PWM_INTSTS_PIF5_Pos              (23)                                              /*!< PWM_T::INTSTS: PIF5 Position              */
#define PWM_INTSTS_PIF5_Msk              (0x1ul << PWM_INTSTS_PIF5_Pos)                    /*!< PWM_T::INTSTS: PIF5 Mask                  */

#define PWM_INTSTS_CMPUIF0_Pos           (24)                                              /*!< PWM_T::INTSTS: CMPUIF0 Position           */
#define PWM_INTSTS_CMPUIF0_Msk           (0x1ul << PWM_INTSTS_CMPUIF0_Pos)                 /*!< PWM_T::INTSTS: CMPUIF0 Mask               */

#define PWM_INTSTS_CMPUIF1_Pos           (25)                                              /*!< PWM_T::INTSTS: CMPUIF1 Position           */
#define PWM_INTSTS_CMPUIF1_Msk           (0x1ul << PWM_INTSTS_CMPUIF1_Pos)                 /*!< PWM_T::INTSTS: CMPUIF1 Mask               */

#define PWM_INTSTS_CMPUIF2_Pos           (26)                                              /*!< PWM_T::INTSTS: CMPUIF2 Position           */
#define PWM_INTSTS_CMPUIF2_Msk           (0x1ul << PWM_INTSTS_CMPUIF2_Pos)                 /*!< PWM_T::INTSTS: CMPUIF2 Mask               */

#define PWM_INTSTS_CMPUIF3_Pos           (27)                                              /*!< PWM_T::INTSTS: CMPUIF3 Position           */
#define PWM_INTSTS_CMPUIF3_Msk           (0x1ul << PWM_INTSTS_CMPUIF3_Pos)                 /*!< PWM_T::INTSTS: CMPUIF3 Mask               */

#define PWM_INTSTS_CMPUIF4_Pos           (28)                                              /*!< PWM_T::INTSTS: CMPUIF4 Position           */
#define PWM_INTSTS_CMPUIF4_Msk           (0x1ul << PWM_INTSTS_CMPUIF4_Pos)                 /*!< PWM_T::INTSTS: CMPUIF4 Mask               */

#define PWM_INTSTS_CMPUIF5_Pos           (29)                                              /*!< PWM_T::INTSTS: CMPUIF5 Position           */
#define PWM_INTSTS_CMPUIF5_Msk           (0x1ul << PWM_INTSTS_CMPUIF5_Pos)                 /*!< PWM_T::INTSTS: CMPUIF5 Mask               */

#define PWM_POEN_POEN0_Pos               (0)                                               /*!< PWM_T::POEN: POEN0 Position               */
#define PWM_POEN_POEN0_Msk               (0x1ul << PWM_POEN_POEN0_Pos)                     /*!< PWM_T::POEN: POEN0 Mask                   */

#define PWM_POEN_POEN1_Pos               (1)                                               /*!< PWM_T::POEN: POEN1 Position               */
#define PWM_POEN_POEN1_Msk               (0x1ul << PWM_POEN_POEN1_Pos)                     /*!< PWM_T::POEN: POEN1 Mask                   */

#define PWM_POEN_POEN2_Pos               (2)                                               /*!< PWM_T::POEN: POEN2 Position               */
#define PWM_POEN_POEN2_Msk               (0x1ul << PWM_POEN_POEN2_Pos)                     /*!< PWM_T::POEN: POEN2 Mask                   */

#define PWM_POEN_POEN3_Pos               (3)                                               /*!< PWM_T::POEN: POEN3 Position               */
#define PWM_POEN_POEN3_Msk               (0x1ul << PWM_POEN_POEN3_Pos)                     /*!< PWM_T::POEN: POEN3 Mask                   */

#define PWM_POEN_POEN4_Pos               (4)                                               /*!< PWM_T::POEN: POEN4 Position               */
#define PWM_POEN_POEN4_Msk               (0x1ul << PWM_POEN_POEN4_Pos)                     /*!< PWM_T::POEN: POEN4 Mask                   */

#define PWM_POEN_POEN5_Pos               (5)                                               /*!< PWM_T::POEN: POEN5 Position               */
#define PWM_POEN_POEN5_Msk               (0x1ul << PWM_POEN_POEN5_Pos)                     /*!< PWM_T::POEN: POEN5 Mask                   */

#define PWM_BRKCTL_BRK0EN_Pos            (0)                                               /*!< PWM_T::BRKCTL: BRK0EN Position            */
#define PWM_BRKCTL_BRK0EN_Msk            (0x1ul << PWM_BRKCTL_BRK0EN_Pos)                  /*!< PWM_T::BRKCTL: BRK0EN Mask                */

#define PWM_BRKCTL_BRK1EN_Pos            (1)                                               /*!< PWM_T::BRKCTL: BRK1EN Position            */
#define PWM_BRKCTL_BRK1EN_Msk            (0x1ul << PWM_BRKCTL_BRK1EN_Pos)                  /*!< PWM_T::BRKCTL: BRK1EN Mask                */

#define PWM_BRKCTL_BRK0SEL_Pos           (2)                                               /*!< PWM_T::BRKCTL: BRK0SEL Position           */
#define PWM_BRKCTL_BRK0SEL_Msk           (0x1ul << PWM_BRKCTL_BRK0SEL_Pos)                 /*!< PWM_T::BRKCTL: BRK0SEL Mask               */

#define PWM_BRKCTL_BRK1SEL_Pos           (3)                                               /*!< PWM_T::BRKCTL: BRK1SEL Position           */
#define PWM_BRKCTL_BRK1SEL_Msk           (0x1ul << PWM_BRKCTL_BRK1SEL_Pos)                 /*!< PWM_T::BRKCTL: BRK1SEL Mask               */

#define PWM_BRKCTL_BRKSTS_Pos            (7)                                               /*!< PWM_T::BRKCTL: BRKSTS Position            */
#define PWM_BRKCTL_BRKSTS_Msk            (0x1ul << PWM_BRKCTL_BRKSTS_Pos)                  /*!< PWM_T::BRKCTL: BRKSTS Mask                */

#define PWM_BRKCTL_BRKACT_Pos            (8)                                               /*!< PWM_T::BRKCTL: BRKACT Position            */
#define PWM_BRKCTL_BRKACT_Msk            (0x1ul << PWM_BRKCTL_BRKACT_Pos)                  /*!< PWM_T::BRKCTL: BRKACT Mask                */

#define PWM_BRKCTL_SWBRK_Pos             (9)                                               /*!< PWM_T::BRKCTL: SWBRK Position             */
#define PWM_BRKCTL_SWBRK_Msk             (0x1ul << PWM_BRKCTL_SWBRK_Pos)                   /*!< PWM_T::BRKCTL: SWBRK Mask                 */

#define PWM_BRKCTL_BKOD0_Pos             (24)                                              /*!< PWM_T::BRKCTL: BKOD0 Position             */
#define PWM_BRKCTL_BKOD0_Msk             (0x1ul << PWM_BRKCTL_BKOD0_Pos)                   /*!< PWM_T::BRKCTL: BKOD0 Mask                 */

#define PWM_BRKCTL_BKOD1_Pos             (25)                                              /*!< PWM_T::BRKCTL: BKOD1 Position             */
#define PWM_BRKCTL_BKOD1_Msk             (0x1ul << PWM_BRKCTL_BKOD1_Pos)                   /*!< PWM_T::BRKCTL: BKOD1 Mask                 */

#define PWM_BRKCTL_BKOD2_Pos             (26)                                              /*!< PWM_T::BRKCTL: BKOD2 Position             */
#define PWM_BRKCTL_BKOD2_Msk             (0x1ul << PWM_BRKCTL_BKOD2_Pos)                   /*!< PWM_T::BRKCTL: BKOD2 Mask                 */

#define PWM_BRKCTL_BKOD3_Pos             (27)                                              /*!< PWM_T::BRKCTL: BKOD3 Position             */
#define PWM_BRKCTL_BKOD3_Msk             (0x1ul << PWM_BRKCTL_BKOD3_Pos)                   /*!< PWM_T::BRKCTL: BKOD3 Mask                 */

#define PWM_BRKCTL_BKOD4_Pos             (28)                                              /*!< PWM_T::BRKCTL: BKOD4 Position             */
#define PWM_BRKCTL_BKOD4_Msk             (0x1ul << PWM_BRKCTL_BKOD4_Pos)                   /*!< PWM_T::BRKCTL: BKOD4 Mask                 */

#define PWM_BRKCTL_BKOD5_Pos             (29)                                              /*!< PWM_T::BRKCTL: BKOD5 Position             */
#define PWM_BRKCTL_BKOD5_Msk             (0x1ul << PWM_BRKCTL_BKOD5_Pos)                   /*!< PWM_T::BRKCTL: BKOD5 Mask                 */

#define PWM_BRKCTL_D6BKOD_Pos            (30)                                              /*!< PWM_T::BRKCTL: D6BKOD Position            */
#define PWM_BRKCTL_D6BKOD_Msk            (0x1ul << PWM_BRKCTL_D6BKOD_Pos)                  /*!< PWM_T::BRKCTL: D6BKOD Mask                */

#define PWM_BRKCTL_D7BKOD_Pos            (31)                                              /*!< PWM_T::BRKCTL: D7BKOD Position            */
#define PWM_BRKCTL_D7BKOD_Msk            (0x1ul << PWM_BRKCTL_D7BKOD_Pos)                  /*!< PWM_T::BRKCTL: D7BKOD Mask                */

#define PWM_DTCTL_DTCNT01_Pos            (0)                                               /*!< PWM_T::DTCTL: DTCNT01 Position            */
#define PWM_DTCTL_DTCNT01_Msk            (0xfful << PWM_DTCTL_DTCNT01_Pos)                 /*!< PWM_T::DTCTL: DTCNT01 Mask                */

#define PWM_DTCTL_DTCNT23_Pos            (8)                                               /*!< PWM_T::DTCTL: DTCNT23 Position            */
#define PWM_DTCTL_DTCNT23_Msk            (0xfful << PWM_DTCTL_DTCNT23_Pos)                 /*!< PWM_T::DTCTL: DTCNT23 Mask                */

#define PWM_DTCTL_DTCNT45_Pos            (16)                                              /*!< PWM_T::DTCTL: DTCNT45 Position            */
#define PWM_DTCTL_DTCNT45_Msk            (0xfful << PWM_DTCTL_DTCNT45_Pos)                 /*!< PWM_T::DTCTL: DTCNT45 Mask                */

#define PWM_ADCTCTL0_CUTRGEN0_Pos        (0)                                               /*!< PWM_T::ADCTCTL0: CUTRGEN0 Position        */
#define PWM_ADCTCTL0_CUTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN0 Mask            */

#define PWM_ADCTCTL0_CPTRGEN0_Pos        (1)                                               /*!< PWM_T::ADCTCTL0: CPTRGEN0 Position        */
#define PWM_ADCTCTL0_CPTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN0 Mask            */

#define PWM_ADCTCTL0_CDTRGEN0_Pos        (2)                                               /*!< PWM_T::ADCTCTL0: CDTRGEN0 Position        */
#define PWM_ADCTCTL0_CDTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN0 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN0_Pos        (3)                                               /*!< PWM_T::ADCTCTL0: ZPTRGEN0 Position        */
#define PWM_ADCTCTL0_ZPTRGEN0_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN0_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN0 Mask            */

#define PWM_ADCTCTL0_CUTRGEN1_Pos        (8)                                               /*!< PWM_T::ADCTCTL0: CUTRGEN1 Position        */
#define PWM_ADCTCTL0_CUTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN1 Mask            */

#define PWM_ADCTCTL0_CPTRGEN1_Pos        (9)                                               /*!< PWM_T::ADCTCTL0: CPTRGEN1 Position        */
#define PWM_ADCTCTL0_CPTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN1 Mask            */

#define PWM_ADCTCTL0_CDTRGEN1_Pos        (10)                                              /*!< PWM_T::ADCTCTL0: CDTRGEN1 Position        */
#define PWM_ADCTCTL0_CDTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN1 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN1_Pos        (11)                                              /*!< PWM_T::ADCTCTL0: ZPTRGEN1 Position        */
#define PWM_ADCTCTL0_ZPTRGEN1_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN1_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN1 Mask            */

#define PWM_ADCTCTL0_CUTRGEN2_Pos        (16)                                              /*!< PWM_T::ADCTCTL0: CUTRGEN2 Position        */
#define PWM_ADCTCTL0_CUTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN2 Mask            */

#define PWM_ADCTCTL0_CPTRGEN2_Pos        (17)                                              /*!< PWM_T::ADCTCTL0: CPTRGEN2 Position        */
#define PWM_ADCTCTL0_CPTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN2 Mask            */

#define PWM_ADCTCTL0_CDTRGEN2_Pos        (18)                                              /*!< PWM_T::ADCTCTL0: CDTRGEN2 Position        */
#define PWM_ADCTCTL0_CDTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN2 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN2_Pos        (19)                                              /*!< PWM_T::ADCTCTL0: ZPTRGEN2 Position        */
#define PWM_ADCTCTL0_ZPTRGEN2_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN2_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN2 Mask            */

#define PWM_ADCTCTL0_CUTRGEN3_Pos        (24)                                              /*!< PWM_T::ADCTCTL0: CUTRGEN3 Position        */
#define PWM_ADCTCTL0_CUTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_CUTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: CUTRGEN3 Mask            */

#define PWM_ADCTCTL0_CPTRGEN3_Pos        (25)                                              /*!< PWM_T::ADCTCTL0: CPTRGEN3 Position        */
#define PWM_ADCTCTL0_CPTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_CPTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: CPTRGEN3 Mask            */

#define PWM_ADCTCTL0_CDTRGEN3_Pos        (26)                                              /*!< PWM_T::ADCTCTL0: CDTRGEN3 Position        */
#define PWM_ADCTCTL0_CDTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_CDTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: CDTRGEN3 Mask            */

#define PWM_ADCTCTL0_ZPTRGEN3_Pos        (27)                                              /*!< PWM_T::ADCTCTL0: ZPTRGEN3 Position        */
#define PWM_ADCTCTL0_ZPTRGEN3_Msk        (0x1ul << PWM_ADCTCTL0_ZPTRGEN3_Pos)              /*!< PWM_T::ADCTCTL0: ZPTRGEN3 Mask            */

#define PWM_ADCTCTL1_CUTRGEN4_Pos        (0)                                               /*!< PWM_T::ADCTCTL1: CUTRGEN4 Position        */
#define PWM_ADCTCTL1_CUTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_CUTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: CUTRGEN4 Mask            */

#define PWM_ADCTCTL1_CPTRGEN4_Pos        (1)                                               /*!< PWM_T::ADCTCTL1: CPTRGEN4 Position        */
#define PWM_ADCTCTL1_CPTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_CPTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: CPTRGEN4 Mask            */

#define PWM_ADCTCTL1_CDTRGEN4_Pos        (2)                                               /*!< PWM_T::ADCTCTL1: CDTRGEN4 Position        */
#define PWM_ADCTCTL1_CDTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_CDTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: CDTRGEN4 Mask            */

#define PWM_ADCTCTL1_ZPTRGEN4_Pos        (3)                                               /*!< PWM_T::ADCTCTL1: ZPTRGEN4 Position        */
#define PWM_ADCTCTL1_ZPTRGEN4_Msk        (0x1ul << PWM_ADCTCTL1_ZPTRGEN4_Pos)              /*!< PWM_T::ADCTCTL1: ZPTRGEN4 Mask            */

#define PWM_ADCTCTL1_CUTRGEN5_Pos        (8)                                               /*!< PWM_T::ADCTCTL1: CUTRGEN5 Position        */
#define PWM_ADCTCTL1_CUTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_CUTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: CUTRGEN5 Mask            */

#define PWM_ADCTCTL1_CPTRGEN5_Pos        (9)                                               /*!< PWM_T::ADCTCTL1: CPTRGEN5 Position        */
#define PWM_ADCTCTL1_CPTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_CPTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: CPTRGEN5 Mask            */

#define PWM_ADCTCTL1_CDTRGEN5_Pos        (10)                                              /*!< PWM_T::ADCTCTL1: CDTRGEN5 Position        */
#define PWM_ADCTCTL1_CDTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_CDTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: CDTRGEN5 Mask            */

#define PWM_ADCTCTL1_ZPTRGEN5_Pos        (11)                                              /*!< PWM_T::ADCTCTL1: ZPTRGEN5 Position        */
#define PWM_ADCTCTL1_ZPTRGEN5_Msk        (0x1ul << PWM_ADCTCTL1_ZPTRGEN5_Pos)              /*!< PWM_T::ADCTCTL1: ZPTRGEN5 Mask            */

#define PWM_ADCTSTS0_CUTRGF0_Pos         (0)                                               /*!< PWM_T::ADCTSTS0: CUTRGF0 Position         */
#define PWM_ADCTSTS0_CUTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF0 Mask             */

#define PWM_ADCTSTS0_CPTRGF0_Pos         (1)                                               /*!< PWM_T::ADCTSTS0: CPTRGF0 Position         */
#define PWM_ADCTSTS0_CPTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF0 Mask             */

#define PWM_ADCTSTS0_CDTRGF0_Pos         (2)                                               /*!< PWM_T::ADCTSTS0: CDTRGF0 Position         */
#define PWM_ADCTSTS0_CDTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF0 Mask             */

#define PWM_ADCTSTS0_ZPTRGF0_Pos         (3)                                               /*!< PWM_T::ADCTSTS0: ZPTRGF0 Position         */
#define PWM_ADCTSTS0_ZPTRGF0_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF0_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF0 Mask             */

#define PWM_ADCTSTS0_CUTRGF1_Pos         (8)                                               /*!< PWM_T::ADCTSTS0: CUTRGF1 Position         */
#define PWM_ADCTSTS0_CUTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF1 Mask             */

#define PWM_ADCTSTS0_CPTRGF1_Pos         (9)                                               /*!< PWM_T::ADCTSTS0: CPTRGF1 Position         */
#define PWM_ADCTSTS0_CPTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF1 Mask             */

#define PWM_ADCTSTS0_CDTRGF1_Pos         (10)                                              /*!< PWM_T::ADCTSTS0: CDTRGF1 Position         */
#define PWM_ADCTSTS0_CDTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF1 Mask             */

#define PWM_ADCTSTS0_ZPTRGF1_Pos         (11)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF1 Position         */
#define PWM_ADCTSTS0_ZPTRGF1_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF1_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF1 Mask             */

#define PWM_ADCTSTS0_CUTRGF2_Pos         (16)                                              /*!< PWM_T::ADCTSTS0: CUTRGF2 Position         */
#define PWM_ADCTSTS0_CUTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF2 Mask             */

#define PWM_ADCTSTS0_CPTRGF2_Pos         (17)                                              /*!< PWM_T::ADCTSTS0: CPTRGF2 Position         */
#define PWM_ADCTSTS0_CPTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF2 Mask             */

#define PWM_ADCTSTS0_CDTRGF2_Pos         (18)                                              /*!< PWM_T::ADCTSTS0: CDTRGF2 Position         */
#define PWM_ADCTSTS0_CDTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF2 Mask             */

#define PWM_ADCTSTS0_ZPTRGF2_Pos         (19)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF2 Position         */
#define PWM_ADCTSTS0_ZPTRGF2_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF2_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF2 Mask             */

#define PWM_ADCTSTS0_CUTRGF3_Pos         (24)                                              /*!< PWM_T::ADCTSTS0: CUTRGF3 Position         */
#define PWM_ADCTSTS0_CUTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_CUTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: CUTRGF3 Mask             */

#define PWM_ADCTSTS0_CPTRGF3_Pos         (25)                                              /*!< PWM_T::ADCTSTS0: CPTRGF3 Position         */
#define PWM_ADCTSTS0_CPTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_CPTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: CPTRGF3 Mask             */

#define PWM_ADCTSTS0_CDTRGF3_Pos         (26)                                              /*!< PWM_T::ADCTSTS0: CDTRGF3 Position         */
#define PWM_ADCTSTS0_CDTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_CDTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: CDTRGF3 Mask             */

#define PWM_ADCTSTS0_ZPTRGF3_Pos         (27)                                              /*!< PWM_T::ADCTSTS0: ZPTRGF3 Position         */
#define PWM_ADCTSTS0_ZPTRGF3_Msk         (0x1ul << PWM_ADCTSTS0_ZPTRGF3_Pos)               /*!< PWM_T::ADCTSTS0: ZPTRGF3 Mask             */

#define PWM_ADCTSTS1_CUTRGF4_Pos         (0)                                               /*!< PWM_T::ADCTSTS1: CUTRGF4 Position         */
#define PWM_ADCTSTS1_CUTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_CUTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: CUTRGF4 Mask             */

#define PWM_ADCTSTS1_CPTRGF4_Pos         (1)                                               /*!< PWM_T::ADCTSTS1: CPTRGF4 Position         */
#define PWM_ADCTSTS1_CPTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_CPTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: CPTRGF4 Mask             */

#define PWM_ADCTSTS1_CDTRGF4_Pos         (2)                                               /*!< PWM_T::ADCTSTS1: CDTRGF4 Position         */
#define PWM_ADCTSTS1_CDTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_CDTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: CDTRGF4 Mask             */

#define PWM_ADCTSTS1_ZPTRGF4_Pos         (3)                                               /*!< PWM_T::ADCTSTS1: ZPTRGF4 Position         */
#define PWM_ADCTSTS1_ZPTRGF4_Msk         (0x1ul << PWM_ADCTSTS1_ZPTRGF4_Pos)               /*!< PWM_T::ADCTSTS1: ZPTRGF4 Mask             */

#define PWM_ADCTSTS1_CUTRGF5_Pos         (8)                                               /*!< PWM_T::ADCTSTS1: CUTRGF5 Position         */
#define PWM_ADCTSTS1_CUTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_CUTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: CUTRGF5 Mask             */

#define PWM_ADCTSTS1_CPTRGF5_Pos         (9)                                               /*!< PWM_T::ADCTSTS1: CPTRGF5 Position         */
#define PWM_ADCTSTS1_CPTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_CPTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: CPTRGF5 Mask             */

#define PWM_ADCTSTS1_CDTRGF5_Pos         (10)                                              /*!< PWM_T::ADCTSTS1: CDTRGF5 Position         */
#define PWM_ADCTSTS1_CDTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_CDTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: CDTRGF5 Mask             */

#define PWM_ADCTSTS1_ZPTRGF5_Pos         (11)                                              /*!< PWM_T::ADCTSTS1: ZPTRGF5 Position         */
#define PWM_ADCTSTS1_ZPTRGF5_Msk         (0x1ul << PWM_ADCTSTS1_ZPTRGF5_Pos)               /*!< PWM_T::ADCTSTS1: ZPTRGF5 Mask             */

#define PWM_PHCHG_MSKDAT0_Pos            (0)                                               /*!< PWM_T::PHCHG: MSKDAT0 Position            */
#define PWM_PHCHG_MSKDAT0_Msk            (0x1ul << PWM_PHCHG_MSKDAT0_Pos)                  /*!< PWM_T::PHCHG: MSKDAT0 Mask                */

#define PWM_PHCHG_MSKDAT1_Pos            (1)                                               /*!< PWM_T::PHCHG: MSKDAT1 Position            */
#define PWM_PHCHG_MSKDAT1_Msk            (0x1ul << PWM_PHCHG_MSKDAT1_Pos)                  /*!< PWM_T::PHCHG: MSKDAT1 Mask                */

#define PWM_PHCHG_MSKDAT2_Pos            (2)                                               /*!< PWM_T::PHCHG: MSKDAT2 Position            */
#define PWM_PHCHG_MSKDAT2_Msk            (0x1ul << PWM_PHCHG_MSKDAT2_Pos)                  /*!< PWM_T::PHCHG: MSKDAT2 Mask                */

#define PWM_PHCHG_MSKDAT3_Pos            (3)                                               /*!< PWM_T::PHCHG: MSKDAT3 Position            */
#define PWM_PHCHG_MSKDAT3_Msk            (0x1ul << PWM_PHCHG_MSKDAT3_Pos)                  /*!< PWM_T::PHCHG: MSKDAT3 Mask                */

#define PWM_PHCHG_MSKDAT4_Pos            (4)                                               /*!< PWM_T::PHCHG: MSKDAT4 Position            */
#define PWM_PHCHG_MSKDAT4_Msk            (0x1ul << PWM_PHCHG_MSKDAT4_Pos)                  /*!< PWM_T::PHCHG: MSKDAT4 Mask                */

#define PWM_PHCHG_MSKDAT5_Pos            (5)                                               /*!< PWM_T::PHCHG: MSKDAT5 Position            */
#define PWM_PHCHG_MSKDAT5_Msk            (0x1ul << PWM_PHCHG_MSKDAT5_Pos)                  /*!< PWM_T::PHCHG: MSKDAT5 Mask                */

#define PWM_PHCHG_MSKDAT6_Pos            (6)                                               /*!< PWM_T::PHCHG: MSKDAT6 Position            */
#define PWM_PHCHG_MSKDAT6_Msk            (0x1ul << PWM_PHCHG_MSKDAT6_Pos)                  /*!< PWM_T::PHCHG: MSKDAT6 Mask                */

#define PWM_PHCHG_MSKDAT7_Pos            (7)                                               /*!< PWM_T::PHCHG: MSKDAT7 Position            */
#define PWM_PHCHG_MSKDAT7_Msk            (0x1ul << PWM_PHCHG_MSKDAT7_Pos)                  /*!< PWM_T::PHCHG: MSKDAT7 Mask                */

#define PWM_PHCHG_MSKEN0_Pos             (8)                                               /*!< PWM_T::PHCHG: MSKEN0 Position             */
#define PWM_PHCHG_MSKEN0_Msk             (0x1ul << PWM_PHCHG_MSKEN0_Pos)                   /*!< PWM_T::PHCHG: MSKEN0 Mask                 */

#define PWM_PHCHG_MSKEN1_Pos             (9)                                               /*!< PWM_T::PHCHG: MSKEN1 Position             */
#define PWM_PHCHG_MSKEN1_Msk             (0x1ul << PWM_PHCHG_MSKEN1_Pos)                   /*!< PWM_T::PHCHG: MSKEN1 Mask                 */

#define PWM_PHCHG_MSKEN2_Pos             (10)                                              /*!< PWM_T::PHCHG: MSKEN2 Position             */
#define PWM_PHCHG_MSKEN2_Msk             (0x1ul << PWM_PHCHG_MSKEN2_Pos)                   /*!< PWM_T::PHCHG: MSKEN2 Mask                 */

#define PWM_PHCHG_MSKEN3_Pos             (11)                                              /*!< PWM_T::PHCHG: MSKEN3 Position             */
#define PWM_PHCHG_MSKEN3_Msk             (0x1ul << PWM_PHCHG_MSKEN3_Pos)                   /*!< PWM_T::PHCHG: MSKEN3 Mask                 */

#define PWM_PHCHG_MSKEN4_Pos             (12)                                              /*!< PWM_T::PHCHG: MSKEN4 Position             */
#define PWM_PHCHG_MSKEN4_Msk             (0x1ul << PWM_PHCHG_MSKEN4_Pos)                   /*!< PWM_T::PHCHG: MSKEN4 Mask                 */

#define PWM_PHCHG_MSKEN5_Pos             (13)                                              /*!< PWM_T::PHCHG: MSKEN5 Position             */
#define PWM_PHCHG_MSKEN5_Msk             (0x1ul << PWM_PHCHG_MSKEN5_Pos)                   /*!< PWM_T::PHCHG: MSKEN5 Mask                 */

#define PWM_PHCHG_AUTOCLR0_Pos           (14)                                              /*!< PWM_T::PHCHG: AUTOCLR0 Position           */
#define PWM_PHCHG_AUTOCLR0_Msk           (0x1ul << PWM_PHCHG_AUTOCLR0_Pos)                 /*!< PWM_T::PHCHG: AUTOCLR0 Mask               */

#define PWM_PHCHG_AUTOCLR1_Pos           (15)                                              /*!< PWM_T::PHCHG: AUTOCLR1 Position           */
#define PWM_PHCHG_AUTOCLR1_Msk           (0x1ul << PWM_PHCHG_AUTOCLR1_Pos)                 /*!< PWM_T::PHCHG: AUTOCLR1 Mask               */

#define PWM_PHCHG_OFFEN01_Pos            (16)                                              /*!< PWM_T::PHCHG: OFFEN01 Position            */
#define PWM_PHCHG_OFFEN01_Msk            (0x1ul << PWM_PHCHG_OFFEN01_Pos)                  /*!< PWM_T::PHCHG: OFFEN01 Mask                */

#define PWM_PHCHG_OFFEN11_Pos            (17)                                              /*!< PWM_T::PHCHG: OFFEN11 Position            */
#define PWM_PHCHG_OFFEN11_Msk            (0x1ul << PWM_PHCHG_OFFEN11_Pos)                  /*!< PWM_T::PHCHG: OFFEN11 Mask                */

#define PWM_PHCHG_OFFEN21_Pos            (18)                                              /*!< PWM_T::PHCHG: OFFEN21 Position            */
#define PWM_PHCHG_OFFEN21_Msk            (0x1ul << PWM_PHCHG_OFFEN21_Pos)                  /*!< PWM_T::PHCHG: OFFEN21 Mask                */

#define PWM_PHCHG_OFFEN31_Pos            (19)                                              /*!< PWM_T::PHCHG: OFFEN31 Position            */
#define PWM_PHCHG_OFFEN31_Msk            (0x1ul << PWM_PHCHG_OFFEN31_Pos)                  /*!< PWM_T::PHCHG: OFFEN31 Mask                */

#define PWM_PHCHG_A1POSSEL_Pos           (20)                                              /*!< PWM_T::PHCHG: A1POSSEL Position           */
#define PWM_PHCHG_A1POSSEL_Msk           (0x3ul << PWM_PHCHG_A1POSSEL_Pos)                 /*!< PWM_T::PHCHG: A1POSSEL Mask               */

#define PWM_PHCHG_TMR1TEN_Pos            (22)                                              /*!< PWM_T::PHCHG: TMR1TEN Position            */
#define PWM_PHCHG_TMR1TEN_Msk            (0x1ul << PWM_PHCHG_TMR1TEN_Pos)                  /*!< PWM_T::PHCHG: TMR1TEN Mask                */

#define PWM_PHCHG_ACMP1TEN_Pos           (23)                                              /*!< PWM_T::PHCHG: ACMP1TEN Position           */
#define PWM_PHCHG_ACMP1TEN_Msk           (0x1ul << PWM_PHCHG_ACMP1TEN_Pos)                 /*!< PWM_T::PHCHG: ACMP1TEN Mask               */

#define PWM_PHCHG_OFFEN00_Pos            (24)                                              /*!< PWM_T::PHCHG: OFFEN00 Position            */
#define PWM_PHCHG_OFFEN00_Msk            (0x1ul << PWM_PHCHG_OFFEN00_Pos)                  /*!< PWM_T::PHCHG: OFFEN00 Mask                */

#define PWM_PHCHG_OFFEN10_Pos            (25)                                              /*!< PWM_T::PHCHG: OFFEN10 Position            */
#define PWM_PHCHG_OFFEN10_Msk            (0x1ul << PWM_PHCHG_OFFEN10_Pos)                  /*!< PWM_T::PHCHG: OFFEN10 Mask                */

#define PWM_PHCHG_OFFEN20_Pos            (26)                                              /*!< PWM_T::PHCHG: OFFEN20 Position            */
#define PWM_PHCHG_OFFEN20_Msk            (0x1ul << PWM_PHCHG_OFFEN20_Pos)                  /*!< PWM_T::PHCHG: OFFEN20 Mask                */

#define PWM_PHCHG_OFFEN30_Pos            (27)                                              /*!< PWM_T::PHCHG: OFFEN30 Position            */
#define PWM_PHCHG_OFFEN30_Msk            (0x1ul << PWM_PHCHG_OFFEN30_Pos)                  /*!< PWM_T::PHCHG: OFFEN30 Mask                */

#define PWM_PHCHG_A0POSSEL_Pos           (28)                                              /*!< PWM_T::PHCHG: A0POSSEL Position           */
#define PWM_PHCHG_A0POSSEL_Msk           (0x3ul << PWM_PHCHG_A0POSSEL_Pos)                 /*!< PWM_T::PHCHG: A0POSSEL Mask               */

#define PWM_PHCHG_T0_Pos                 (30)                                              /*!< PWM_T::PHCHG: T0 Position                 */
#define PWM_PHCHG_T0_Msk                 (0x1ul << PWM_PHCHG_T0_Pos)                       /*!< PWM_T::PHCHG: T0 Mask                     */

#define PWM_PHCHG_ACMP0TEN_Pos           (31)                                              /*!< PWM_T::PHCHG: ACMP0TEN Position           */
#define PWM_PHCHG_ACMP0TEN_Msk           (0x1ul << PWM_PHCHG_ACMP0TEN_Pos)                 /*!< PWM_T::PHCHG: ACMP0TEN Mask               */

#define PWM_PHCHGNXT_MSKDAT0_Pos         (0)                                               /*!< PWM_T::PHCHGNXT: MSKDAT0 Position         */
#define PWM_PHCHGNXT_MSKDAT0_Msk         (0x1ul << PWM_PHCHGNXT_MSKDAT0_Pos)               /*!< PWM_T::PHCHGNXT: MSKDAT0 Mask             */

#define PWM_PHCHGNXT_MSKDAT1_Pos         (1)                                               /*!< PWM_T::PHCHGNXT: MSKDAT1 Position         */
#define PWM_PHCHGNXT_MSKDAT1_Msk         (0x1ul << PWM_PHCHGNXT_MSKDAT1_Pos)               /*!< PWM_T::PHCHGNXT: MSKDAT1 Mask             */

#define PWM_PHCHGNXT_MSKDAT2_Pos         (2)                                               /*!< PWM_T::PHCHGNXT: MSKDAT2 Position         */
#define PWM_PHCHGNXT_MSKDAT2_Msk         (0x1ul << PWM_PHCHGNXT_MSKDAT2_Pos)               /*!< PWM_T::PHCHGNXT: MSKDAT2 Mask             */

#define PWM_PHCHGNXT_MSKDAT3_Pos         (3)                                               /*!< PWM_T::PHCHGNXT: MSKDAT3 Position         */
#define PWM_PHCHGNXT_MSKDAT3_Msk         (0x1ul << PWM_PHCHGNXT_MSKDAT3_Pos)               /*!< PWM_T::PHCHGNXT: MSKDAT3 Mask             */

#define PWM_PHCHGNXT_MSKDAT4_Pos         (4)                                               /*!< PWM_T::PHCHGNXT: MSKDAT4 Position         */
#define PWM_PHCHGNXT_MSKDAT4_Msk         (0x1ul << PWM_PHCHGNXT_MSKDAT4_Pos)               /*!< PWM_T::PHCHGNXT: MSKDAT4 Mask             */

#define PWM_PHCHGNXT_MSKDAT5_Pos         (5)                                               /*!< PWM_T::PHCHGNXT: MSKDAT5 Position         */
#define PWM_PHCHGNXT_MSKDAT5_Msk         (0x1ul << PWM_PHCHGNXT_MSKDAT5_Pos)               /*!< PWM_T::PHCHGNXT: MSKDAT5 Mask             */

#define PWM_PHCHGNXT_MSKDAT6_Pos         (6)                                               /*!< PWM_T::PHCHGNXT: MSKDAT6 Position         */
#define PWM_PHCHGNXT_MSKDAT6_Msk         (0x1ul << PWM_PHCHGNXT_MSKDAT6_Pos)               /*!< PWM_T::PHCHGNXT: MSKDAT6 Mask             */

#define PWM_PHCHGNXT_MSKDAT7_Pos         (7)                                               /*!< PWM_T::PHCHGNXT: MSKDAT7 Position         */
#define PWM_PHCHGNXT_MSKDAT7_Msk         (0x1ul << PWM_PHCHGNXT_MSKDAT7_Pos)               /*!< PWM_T::PHCHGNXT: MSKDAT7 Mask             */

#define PWM_PHCHGNXT_MSKEN0_Pos          (8)                                               /*!< PWM_T::PHCHGNXT: MSKEN0 Position          */
#define PWM_PHCHGNXT_MSKEN0_Msk          (0x1ul << PWM_PHCHGNXT_MSKEN0_Pos)                /*!< PWM_T::PHCHGNXT: MSKEN0 Mask              */

#define PWM_PHCHGNXT_MSKEN1_Pos          (9)                                               /*!< PWM_T::PHCHGNXT: MSKEN1 Position          */
#define PWM_PHCHGNXT_MSKEN1_Msk          (0x1ul << PWM_PHCHGNXT_MSKEN1_Pos)                /*!< PWM_T::PHCHGNXT: MSKEN1 Mask              */

#define PWM_PHCHGNXT_MSKEN2_Pos          (10)                                              /*!< PWM_T::PHCHGNXT: MSKEN2 Position          */
#define PWM_PHCHGNXT_MSKEN2_Msk          (0x1ul << PWM_PHCHGNXT_MSKEN2_Pos)                /*!< PWM_T::PHCHGNXT: MSKEN2 Mask              */

#define PWM_PHCHGNXT_MSKEN3_Pos          (11)                                              /*!< PWM_T::PHCHGNXT: MSKEN3 Position          */
#define PWM_PHCHGNXT_MSKEN3_Msk          (0x1ul << PWM_PHCHGNXT_MSKEN3_Pos)                /*!< PWM_T::PHCHGNXT: MSKEN3 Mask              */

#define PWM_PHCHGNXT_MSKEN4_Pos          (12)                                              /*!< PWM_T::PHCHGNXT: MSKEN4 Position          */
#define PWM_PHCHGNXT_MSKEN4_Msk          (0x1ul << PWM_PHCHGNXT_MSKEN4_Pos)                /*!< PWM_T::PHCHGNXT: MSKEN4 Mask              */

#define PWM_PHCHGNXT_MSKEN5_Pos          (13)                                              /*!< PWM_T::PHCHGNXT: MSKEN5 Position          */
#define PWM_PHCHGNXT_MSKEN5_Msk          (0x1ul << PWM_PHCHGNXT_MSKEN5_Pos)                /*!< PWM_T::PHCHGNXT: MSKEN5 Mask              */

#define PWM_PHCHGNXT_AUTOCLR0_Pos        (14)                                              /*!< PWM_T::PHCHGNXT: AUTOCLR0 Position        */
#define PWM_PHCHGNXT_AUTOCLR0_Msk        (0x1ul << PWM_PHCHGNXT_AUTOCLR0_Pos)              /*!< PWM_T::PHCHGNXT: AUTOCLR0 Mask            */

#define PWM_PHCHGNXT_AUTOCLR1_Pos        (15)                                              /*!< PWM_T::PHCHGNXT: AUTOCLR1 Position        */
#define PWM_PHCHGNXT_AUTOCLR1_Msk        (0x1ul << PWM_PHCHGNXT_AUTOCLR1_Pos)              /*!< PWM_T::PHCHGNXT: AUTOCLR1 Mask            */

#define PWM_PHCHGNXT_OFFEN01_Pos         (16)                                              /*!< PWM_T::PHCHGNXT: OFFEN01 Position         */
#define PWM_PHCHGNXT_OFFEN01_Msk         (0x1ul << PWM_PHCHGNXT_OFFEN01_Pos)               /*!< PWM_T::PHCHGNXT: OFFEN01 Mask             */

#define PWM_PHCHGNXT_OFFEN11_Pos         (17)                                              /*!< PWM_T::PHCHGNXT: OFFEN11 Position         */
#define PWM_PHCHGNXT_OFFEN11_Msk         (0x1ul << PWM_PHCHGNXT_OFFEN11_Pos)               /*!< PWM_T::PHCHGNXT: OFFEN11 Mask             */

#define PWM_PHCHGNXT_OFFEN21_Pos         (18)                                              /*!< PWM_T::PHCHGNXT: OFFEN21 Position         */
#define PWM_PHCHGNXT_OFFEN21_Msk         (0x1ul << PWM_PHCHGNXT_OFFEN21_Pos)               /*!< PWM_T::PHCHGNXT: OFFEN21 Mask             */

#define PWM_PHCHGNXT_OFFEN31_Pos         (19)                                              /*!< PWM_T::PHCHGNXT: OFFEN31 Position         */
#define PWM_PHCHGNXT_OFFEN31_Msk         (0x1ul << PWM_PHCHGNXT_OFFEN31_Pos)               /*!< PWM_T::PHCHGNXT: OFFEN31 Mask             */

#define PWM_PHCHGNXT_A1POSSEL_Pos        (20)                                              /*!< PWM_T::PHCHGNXT: A1POSSEL Position        */
#define PWM_PHCHGNXT_A1POSSEL_Msk        (0x3ul << PWM_PHCHGNXT_A1POSSEL_Pos)              /*!< PWM_T::PHCHGNXT: A1POSSEL Mask            */

#define PWM_PHCHGNXT_TMR1TEN_Pos         (22)                                              /*!< PWM_T::PHCHGNXT: TMR1TEN Position         */
#define PWM_PHCHGNXT_TMR1TEN_Msk         (0x1ul << PWM_PHCHGNXT_TMR1TEN_Pos)               /*!< PWM_T::PHCHGNXT: TMR1TEN Mask             */

#define PWM_PHCHGNXT_ACMP1TEN_Pos        (23)                                              /*!< PWM_T::PHCHGNXT: ACMP1TEN Position        */
#define PWM_PHCHGNXT_ACMP1TEN_Msk        (0x1ul << PWM_PHCHGNXT_ACMP1TEN_Pos)              /*!< PWM_T::PHCHGNXT: ACMP1TEN Mask            */

#define PWM_PHCHGNXT_OFFEN00_Pos         (24)                                              /*!< PWM_T::PHCHGNXT: OFFEN00 Position         */
#define PWM_PHCHGNXT_OFFEN00_Msk         (0x1ul << PWM_PHCHGNXT_OFFEN00_Pos)               /*!< PWM_T::PHCHGNXT: OFFEN00 Mask             */

#define PWM_PHCHGNXT_OFFEN10_Pos         (25)                                              /*!< PWM_T::PHCHGNXT: OFFEN10 Position         */
#define PWM_PHCHGNXT_OFFEN10_Msk         (0x1ul << PWM_PHCHGNXT_OFFEN10_Pos)               /*!< PWM_T::PHCHGNXT: OFFEN10 Mask             */

#define PWM_PHCHGNXT_OFFEN20_Pos         (26)                                              /*!< PWM_T::PHCHGNXT: OFFEN20 Position         */
#define PWM_PHCHGNXT_OFFEN20_Msk         (0x1ul << PWM_PHCHGNXT_OFFEN20_Pos)               /*!< PWM_T::PHCHGNXT: OFFEN20 Mask             */

#define PWM_PHCHGNXT_OFFEN30_Pos         (27)                                              /*!< PWM_T::PHCHGNXT: OFFEN30 Position         */
#define PWM_PHCHGNXT_OFFEN30_Msk         (0x1ul << PWM_PHCHGNXT_OFFEN30_Pos)               /*!< PWM_T::PHCHGNXT: OFFEN30 Mask             */

#define PWM_PHCHGNXT_A0POSSEL_Pos        (28)                                              /*!< PWM_T::PHCHGNXT: A0POSSEL Position        */
#define PWM_PHCHGNXT_A0POSSEL_Msk        (0x3ul << PWM_PHCHGNXT_A0POSSEL_Pos)              /*!< PWM_T::PHCHGNXT: A0POSSEL Mask            */

#define PWM_PHCHGNXT_TMR0TEN_Pos         (30)                                              /*!< PWM_T::PHCHGNXT: TMR0TEN Position         */
#define PWM_PHCHGNXT_TMR0TEN_Msk         (0x1ul << PWM_PHCHGNXT_TMR0TEN_Pos)               /*!< PWM_T::PHCHGNXT: TMR0TEN Mask             */

#define PWM_PHCHGNXT_ACMP0TEN_Pos        (31)                                              /*!< PWM_T::PHCHGNXT: ACMP0TEN Position        */
#define PWM_PHCHGNXT_ACMP0TEN_Msk        (0x1ul << PWM_PHCHGNXT_ACMP0TEN_Pos)              /*!< PWM_T::PHCHGNXT: ACMP0TEN Mask            */

#define PWM_PHCHGMSK_MASKEND6_Pos        (6)                                               /*!< PWM_T::PHCHGMSK: MASKEND6 Position        */
#define PWM_PHCHGMSK_MASKEND6_Msk        (0x1ul << PWM_PHCHGMSK_MASKEND6_Pos)              /*!< PWM_T::PHCHGMSK: MASKEND6 Mask            */

#define PWM_PHCHGMSK_MASKEND7_Pos        (7)                                               /*!< PWM_T::PHCHGMSK: MASKEND7 Position        */
#define PWM_PHCHGMSK_MASKEND7_Msk        (0x1ul << PWM_PHCHGMSK_MASKEND7_Pos)              /*!< PWM_T::PHCHGMSK: MASKEND7 Mask            */

#define PWM_PHCHGMSK_POSCTL0_Pos         (8)                                               /*!< PWM_T::PHCHGMSK: POSCTL0 Position         */
#define PWM_PHCHGMSK_POSCTL0_Msk         (0x1ul << PWM_PHCHGMSK_POSCTL0_Pos)               /*!< PWM_T::PHCHGMSK: POSCTL0 Mask             */

#define PWM_PHCHGMSK_POSCTL1_Pos         (9)                                               /*!< PWM_T::PHCHGMSK: POSCTL1 Position         */
#define PWM_PHCHGMSK_POSCTL1_Msk         (0x1ul << PWM_PHCHGMSK_POSCTL1_Pos)               /*!< PWM_T::PHCHGMSK: POSCTL1 Mask             */

#define PWM_IFA_IFAEN_Pos                (0)                                               /*!< PWM_T::IFA: IFAEN Position                */
#define PWM_IFA_IFAEN_Msk                (0x1ul << PWM_IFA_IFAEN_Pos)                      /*!< PWM_T::IFA: IFAEN Mask                    */

#define PWM_IFA_IFCNT_Pos                (4)                                               /*!< PWM_T::IFA: IFCNT Position                */
#define PWM_IFA_IFCNT_Msk                (0xful << PWM_IFA_IFCNT_Pos)                      /*!< PWM_T::IFA: IFCNT Mask                    */

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
     * CTL
     * ===================================================================================================
     * Offset: 0x00  SPI Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SPIEN     |SPI Transfer Control Bit And Busy Status
     * |        |          |If FIFO mode is enabled, this bit will be controlled by hardware and its Read only.
     * |        |          |If FIFO mode is disabled, during the data transfer, this bit keeps the value of 1.
     * |        |          |As the transfer is finished, this bit will be cleared automatically.
     * |        |          |0 = Writing 0 to this bit to stop data transfer if SPI is transferring.
     * |        |          |1 = In Master mode, writing 1 to this bit to start the SPI data transfer; in Slave mode, writing 1 to this bit indicates that the slave is ready to communicate with a master.
     * |        |          |Note 1: When FIFO mode is disabled, all configurations should be ready before writing 1 to the SPIEN bit.
     * |        |          |Note 2: In SPI Slave mode, if FIFO mode is disabled and the SPI bus clock is kept at idle state during a data transfer, the SPIEN bit will not be cleared to 0 when slave select signal goes to inactive state.
     * |[1]     |RXNEG     |Receive On Negative Edge
     * |        |          |0 = The received data input signal latched on the Rising edge of SPICLK.
     * |        |          |1 = The received data input signal latched on the Falling edge of SPICLK.
     * |[2]     |TXNEG     |Transmit On Negative Edge
     * |        |          |0 = The transmitted data output signal is driven on the Rising edge of SPICLK.
     * |        |          |1 = The transmitted data output signal is driven on the Falling edge of SPICLK.
     * |[7:3]   |DWIDTH    |Transmit Bit Length
     * |        |          |This field specifies how many bits are transmitted in one transmit/receive.
     * |        |          |The minimum bit length is 8 bits and can up to 32 bits.
     * |        |          |DWIDTH = 0x08 ... 8 bits.
     * |        |          |DWIDTH = 0x09 ... 9 bits.
     * |        |          |......
     * |        |          |DWIDTH = 0x1F ... 31 bits.
     * |        |          |DWIDTH = 0x00 ... 32 bits.
     * |[10]    |LSB       |LSB First
     * |        |          |0 = The MSB is transmitted/received first.
     * |        |          |1 = The LSB is transmitted/received first.
     * |[11]    |CLKPOL    |Clock Polarity
     * |        |          |0 = SPICLK idle low.
     * |        |          |1 = SPICLK idle high.
     * |[15:12] |SUSPITV   |Suspend Interval (Master Only)
     * |        |          |The four bits provide configurable suspend interval between two successive transactions in a transfer.
     * |        |          |The definition of the suspend interval is the interval between the last clock edge of the preceding transaction word and the first clock edge of the following transaction word.
     * |        |          |The default value is 0x3.
     * |        |          |The period of the suspend interval is obtained according to the following equation:
     * |        |          |(SUSPITV + 0.5) * period of SPICLK clock cycle
     * |        |          |Example:
     * |        |          |SUSPITV = 0x0 ... 0.5 SPICLK clock cycle.
     * |        |          |SUSPITV = 0x1 ... 1.5 SPICLK clock cycle.
     * |        |          |......
     * |        |          |SUSPITV = 0xE ... 14.5 SPICLK clock cycle.
     * |        |          |SUSPITV = 0xF ... 15.5 SPICLK clock cycle.
     * |[16]    |UNITIF    |Unit-Transfer Interrupt Flag
     * |        |          |0 = The transfer does not finish yet.
     * |        |          |1 = The SPI controller has finished one unit transfer.
     * |        |          |Note 1: This bit will be cleared by writing 1 to itself.
     * |        |          |Note 2: . It's a mutual mirror bit of SPI_STATUS[16].
     * |[17]    |UNITIEN   |Unit-Transfer Interrupt Enable Control
     * |        |          |0 = SPI unit-transfer interrupt Disabled.
     * |        |          |1 = SPI unit-transfer interrupt Enabled.
     * |[18]    |SLAVE     |Slave Mode Control
     * |        |          |0 = Master mode.
     * |        |          |1 = Slave mode.
     * |[19]    |REORDER   |Byte Reorder Function
     * |        |          |0 = Byte reorder function Disabled.
     * |        |          |1 = Byte reorder function Enabled.
     * |        |          |Note: This setting is only available if DWIDTH is defined as 16, 24, and 32 bits.
     * |[21]    |FIFOEN    |FIFO Mode Enable Control
     * |        |          |0 = FIFO Mode Disabled.
     * |        |          |1 = FIFO Mode Enabled.
     * |        |          |Note 1: Before enabling FIFO mode, the other related settings should be set in advance.
     * |        |          |Note 2: In Master mode, if the FIFO mode is enabled, the SPIEN bit will be set to 1 automatically after writing data into the 4-depth transmit FIFO.
     * |        |          |When all data stored at transmit FIFO buffer are transferred, the SPIEN bit will back to 0.
     * |[24]    |RXEMPTY   |Receive FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = The receive FIFO buffer is not empty.
     * |        |          |1 = The receive FIFO buffer is empty.
     * |        |          |Note: It's a mutual mirror bit of SPI_CTL[24].
     * |[25]    |RXFULL    |Receive FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 = The receive FIFO buffer is not full.
     * |        |          |1 = The receive FIFO buffer is full.
     * |        |          |Note: It's a mutual mirror bit of SPI_STATUS[25]
     * |[26]    |TXEMPTY   |Transmit FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = The transmit FIFO buffer is not empty.
     * |        |          |1 = The transmit FIFO buffer is empty.
     * |        |          |Note: It's a mutual mirror bit of SPI_STAUTS[26].
     * |[27]    |TXFULL    |Transmit FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 = The transmit FIFO buffer is not full.
     * |        |          |1 = The transmit FIFO buffer is full.
     * |        |          |Note: It's a mutual mirror bit of SPI_STATUS[27].
    */
    __IO uint32_t CTL;

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x04  SPI Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DIVIDER   |Clock Divider Register (Master Only)
     * |        |          |The value in this field is the frequency divider to determine the SPI peripheral clock frequency fspi, and the SPI master's bus clock frequency on the SPICLK output pin.
     * |        |          |The frequency is obtained according to the following equation:
     * |        |          |If the bit of DIVMOD, SPI_SLVCTL[31], is set to 0.
     * |        |          |else if DIVMOD is set to 1,
     * |        |          |where
     * |        |          |is the SPI peripheral clock source which is defined in the CLKSEL1 register.
    */
    __IO uint32_t CLKDIV;

    /**
     * SSCTL
     * ===================================================================================================
     * Offset: 0x08  SPI Slave Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SS        |Slave Select Control Bits (Master Only)
     * |        |          |If AUTOSS bit is 0,
     * |        |          |0 = Set the SPISS line to inactive state.
     * |        |          |1 = Set the proper SPISS line to active state.
     * |        |          |If AUTOSS bit is 1,
     * |        |          |0 = Keep the SPISS line at inactive state.
     * |        |          |1 = Select the SPISS line to be automatically driven to active state for the duration of transmission/reception, and will be driven to inactive state for the rest of the time.
     * |        |          |The active state of SPISS is specified in SSACTPOL bit.
     * |[2]     |SSACTPOL  |Slave Select Active Level (Slave Only)
     * |        |          |It defines the active status of slave select signal (SPISS).
     * |        |          |If SSLTEN bit is 1:
     * |        |          |0 = The slave select signal SPISS is active at low-level.
     * |        |          |1 = The slave select signal SPISS is active at high-level.
     * |        |          |If SSLTEN bit is 0:
     * |        |          |0 = The slave select signal SPISS is active at Falling-edge.
     * |        |          |1 = The slave select signal SPISS is active at Rising-edge.
     * |[3]     |AUTOSS    |Automatic Slave Selection Function Enable Bit (Master Only)
     * |        |          |0 = SPISS pin signal will be asserted/de-asserted by setting /clearing SS bit.
     * |        |          |1 = SPISS pin signal will be generated automatically, which means that slave select signal will be asserted by the SPI controller when transmit/receive is started by setting SPIEN, and will be de-asserted after each transmit/receive is finished.
     * |[4]     |SSLTEN    |Slave Select Level Trigger Enable Bit (Slave Only)
     * |        |          |0 = The input slave select signal is edge-trigger.
     * |        |          |1 = The input slave select signal is level-trigger.
     * |[5]     |LTF       |Level Trigger Flag (Read Only, Slave Only)
     * |        |          |When the SSLTEN bit is set in Slave mode, this bit can be read to indicate the received bit number is met the requirement or not.
     * |        |          |0 = The transaction number or the transferred bit length of one transaction does not meet the specified requirements.
     * |        |          |1 = The transaction number and the transferred bit length met the specified requirements which defined in DWIDTH.
    */
    __IO uint32_t SSCTL;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED0[1];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * RX
     * ===================================================================================================
     * Offset: 0x10  SPI Data Receive Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |RX        |Data Receive Register (Read Only)
     * |        |          |The Data Receive Registers hold the value of received data of the last executed transfer.
     * |        |          |Valid bits depend on the transmit bit length field in the SPI_CTL register.
     * |        |          |For example, if DWIDTH is set to 0x08, bit RX0[7:0] holds the received data.
     * |        |          |The values of the other bits are unknown.
     * |        |          |The Data Receive Registers are read-only registers.
    */
    __I  uint32_t RX;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED1[3];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * TX
     * ===================================================================================================
     * Offset: 0x20  SPI Data Transmit Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |TX        |Data Transmit Register
     * |        |          |The Data Transmit Registers hold the data to be transmitted in the next transfer.
     * |        |          |Valid bits depend on the transmit bit length field in the CNTRL register.
     * |        |          |For example, if DWIDTH is set to 0x08, the bit TX0[7:0] will be transmitted in next transfer.
    */
    __O  uint32_t TX;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED2[6];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * SLVCTL
     * ===================================================================================================
     * Offset: 0x3C  SPI Slave Mode Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |SLV3WIRE  |Slave 3-Wire Mode Enable Bit (Slave Only)
     * |        |          |The SPI controller work with 3-wire interface including SPICLK, SPI_MISO, and SPI_MOSI.
     * |        |          |0 = The controller is 4-wire bi-direction interface.
     * |        |          |1 = The controller is 3-wire bi-direction interface in Slave mode.
     * |        |          |The controller will be ready to transmit/receive data after the SPIEN bit is set to 1.
     * |        |          |Note: In Slave 3-wire mode, the SSLTEN bit (SPI_SSCTL[4]) shall be set as 1.
     * |[9]     |SLVABT    |Slave 3-Wire Mode Abort Control Bit (Slave Only)
     * |        |          |In normal operation, there is an interrupt event when the number of received bits meets the requirement which defined in DWIDTH.
     * |        |          |If the number of received bits is less than the requirement and there is no more bus clock input over one transfer time in Slave 3-wire mode, user can set this bit to force the current transfer done and then user can get a unit transfer interrupt event.
     * |        |          |0 = No force the transfer done when the SLV3WIRE bit is set to 1.
     * |        |          |1 = Force the transfer done when the SLV3WIRE bit is set to 1.
     * |        |          |Note: This bit will be cleared to 0 automatically by hardware after it is set to 1 by software.
     * |[10]    |SLVSTIEN  |Slave 3-Wire Mode Start Interrupt Enable (Slave Only)
     * |        |          |It is used to enable interrupt when the transfer has started in Slave 3-wire mode.
     * |        |          |If there is no transfer done interrupt over the time period which is defined by user after the transfer start, user can set the SLV3WIRE bit to force the transfer done.
     * |        |          |0 = Transaction start interrupt Disabled.
     * |        |          |1 = Transaction start interrupt Enabled.
     * |        |          |Note 1: It will be cleared to 0 as the current transfer is done or the SLVSTIF bit is cleared to 0.
     * |[11]    |SLVSTIF   |Slave 3-Wire Mode Start Interrupt Status (Slave Only)
     * |        |          |This bit dedicates if a transaction has started in Slave 3-wire mode.
     * |        |          |0 = Slave does not detect any SPI bus clock transfer since the SLVSTIEN bit was set to 1.
     * |        |          |1 = The transfer has started in Slave 3-wire mode.
     * |        |          |Note 1: It will be cleared automatically when a transaction is done or by writing 1 to this bit.
     * |        |          |Note 2: It is a mutual mirror bit of SPI_STATUS[11].
     * |[16]    |SSINAIEN  |Slave Select Inactive Interrupt Option (Slave Only)
     * |        |          |0 = As the slave select signal goes to inactive level, the UNITIF bit will NOT be set to 1.
     * |        |          |1 = As the slave select signal goes to inactive level, the UNITIF bit will be set to 1.
     * |        |          |Note: This setting is only available if the SPI controller is configured as level trigger in slave device.
     * |[31]    |DIVMOD    |Clock Configuration Backward Compatible Option
     * |        |          |0 = The clock configuration is backward compatible.
     * |        |          |1 = The clock configuration is not backward compatible.
     * |        |          |Note: Refer to the description of SPI_CLKDIV register for details.
    */
    __IO uint32_t SLVCTL;

    /**
     * FIFOCTL
     * ===================================================================================================
     * Offset: 0x40  SPI FIFO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXRST     |Clear Receive FIFO Buffer
     * |        |          |0 = No effect.
     * |        |          |1 = Clear receive FIFO buffer.
     * |        |          |Note: This bit will be cleared to 0 by hardware after software sets it to 1 and the receive FIFO is cleared.
     * |[1]     |TXRST     |Clear Transmit FIFO Buffer
     * |        |          |0 = No effect.
     * |        |          |1 = Clear transmit FIFO buffer.
     * |        |          |Note: This bit will be cleared to 0 by hardware after software sets it to 1 and the transmit FIFO is cleared.
     * |[2]     |RXTHIEN   |Receive Threshold Interrupt Enable Control
     * |        |          |0 = Receive threshold interrupt Disabled.
     * |        |          |1 = Receive threshold interrupt Enabled.
     * |[3]     |TXTHIEN   |Transmit Threshold Interrupt Enable
     * |        |          |0 = Transmit threshold interrupt Disabled.
     * |        |          |1 = Transmit threshold interrupt Enabled.
     * |[6]     |RXOVIEN   |Receive FIFO Overrun Interrupt Enable Control
     * |        |          |0 = Receive FIFO overrun interrupt Disabled.
     * |        |          |1 = Receive FIFO overrun interrupt Enabled.
     * |[21]    |RXTOIEN   |Receive FIFO Time-Out Interrupt Enable Control
     * |        |          |0 = Time-out interrupt Disabled.
     * |        |          |1 = Time-out interrupt Enabled.
     * |[25:24] |RXTH      |Received FIFO Threshold
     * |        |          |If the valid data count of the receive FIFO buffer is larger than the RXTH setting, the RXTHIF bit will be set to 1, else the RXTHIF bit will be cleared to 0.
     * |[29:28] |TXTH      |Transmit FIFO Threshold
     * |        |          |If the valid data count of the transmit FIFO buffer is less than or equal to the TXTH setting, the TXTHIF bit will be set to 1, else the TXTHIF bit will be cleared to 0.
    */
    __IO uint32_t FIFOCTL;

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x44  SPI Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXTHIF    |Receive FIFO Threshold Interrupt Status (Read Only)
     * |        |          |0 = The valid data count within the Rx FIFO buffer is smaller than or equal to the setting value of RXTH.
     * |        |          |1 = The valid data count within the receive FIFO buffer is larger than the setting value of RXTH.
     * |        |          |Note: If RXTHIEN = 1 and RXTHIF = 1, the SPI controller will generate a SPI interrupt request.
     * |[2]     |RXOVIF    |Receive FIFO Overrun Status
     * |        |          |When the receive FIFO buffer is full, the follow-up data will be dropped and this bit will be set to 1.
     * |        |          |0 = No overrun in receive FIFO.
     * |        |          |1 = Overrun in receive FIFO.
     * |        |          |Note: This bit will be cleared by writing 1 to itself.
     * |[4]     |TXTHIF    |Transmit FIFO Threshold Interrupt Status (Read Only)
     * |        |          |0 = The valid data count within the transmit FIFO buffer is larger than the setting value of TXTH.
     * |        |          |1 = The valid data count within the transmit FIFO buffer is less than or equal to the setting value of TXTH.
     * |        |          |Note: If TXTHIEN = 1 and TXTHIF = 1, the SPI controller will generate a SPI interrupt request.
     * |[11]    |SLVSTIF   |Slave Start Interrupt Status (Slave Only)
     * |        |          |It is used to dedicate that the transfer has started in slave 3-wire mode.
     * |        |          |0 = Slave does not detect any SPI bus clock transfer since the SSTA_INTEN bit was set to 1.
     * |        |          |The transfer is not started.
     * |        |          |1 = The transfer has started in Slave 3-wire mode.
     * |        |          |Note 1: It will be cleared as transfer done or by writing one to this bit.
     * |        |          |Note 2: It's a mutual mirror bit of SPI_SLVCTL[11].
     * |[15:12] |RXCNT     |Receive FIFO Data Count (Read Only)
     * |        |          |Indicates the valid data count of receive FIFO buffer.
     * |[16]    |UNITIF    |SPI Unit-Transfer Interrupt Flag
     * |        |          |0 = The transfer does not finish yet.
     * |        |          |1 = The SPI controller has finished one unit transfer.
     * |        |          |Note 1: This bit will be cleared by writing 1 to itself.
     * |        |          |Note 2: It's a mutual mirror bit of SPI_CTL[16].
     * |[20]    |SLVTOIF   |Time-Out Interrupt Flag
     * |        |          |0 = No receive FIFO time-out event.
     * |        |          |1 = The receive FIFO buffer is not empty and it does not be read over 64 SPI clock periods in Master mode or over 576 SPI peripheral clock periods in Slave mode.
     * |        |          |When the received FIFO buffer is read by software, the time-out status will be cleared automatically.
     * |        |          |Note: This bit will be cleared by writing 1 to itself.
     * |[24]    |RXEMPTY   |Receive FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = The receive FIFO buffer is not empty.
     * |        |          |1 = The receive FIFO buffer is empty.
     * |        |          |Note: It's a mutual mirror bit of SPI_CTL[24].
     * |[25]    |RXFULL    |Receive FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 = The receive FIFO buffer is not full.
     * |        |          |1 = The receive FIFO buffer is full.
     * |        |          |Note: It's a mutual mirror bit of SPI_CTL[25].
     * |[26]    |TXEMPTY   |Transmit FIFO Buffer Empty Indicator (Read Only)
     * |        |          |0 = The transmit FIFO buffer is not empty.
     * |        |          |1 = The transmit FIFO buffer is empty.
     * |        |          |Note: It's a mutual mirror bit of SPI_CTL[26].
     * |[27]    |TXFULL    |Transmit FIFO Buffer Full Indicator (Read Only)
     * |        |          |0 = The transmit FIFO buffer is not full.
     * |        |          |1 = The transmit FIFO buffer is full.
     * |        |          |Note: It's a mutual mirror bit of SPI_CTL[27].
     * |[31:28] |TXCNT     |Transmit FIFO Data Count (Read Only)
     * |        |          |Indicates the valid data count of transmit FIFO buffer.
    */
    __IO  uint32_t STATUS;

} SPI_T;

/**
    @addtogroup SPI_CONST SPI Bit Field Definition
    Constant Definitions for SPI Controller
@{ */

#define SPI_CTL_SPIEN_Pos                (0)                                               /*!< SPI_T::CTL: SPIEN Position                */
#define SPI_CTL_SPIEN_Msk                (0x1ul << SPI_CTL_SPIEN_Pos)                      /*!< SPI_T::CTL: SPIEN Mask                    */

#define SPI_CTL_RXNEG_Pos                (1)                                               /*!< SPI_T::CTL: RXNEG Position                */
#define SPI_CTL_RXNEG_Msk                (0x1ul << SPI_CTL_RXNEG_Pos)                      /*!< SPI_T::CTL: RXNEG Mask                    */

#define SPI_CTL_TXNEG_Pos                (2)                                               /*!< SPI_T::CTL: TXNEG Position                */
#define SPI_CTL_TXNEG_Msk                (0x1ul << SPI_CTL_TXNEG_Pos)                      /*!< SPI_T::CTL: TXNEG Mask                    */

#define SPI_CTL_DWIDTH_Pos               (3)                                               /*!< SPI_T::CTL: DWIDTH Position               */
#define SPI_CTL_DWIDTH_Msk               (0x1ful << SPI_CTL_DWIDTH_Pos)                    /*!< SPI_T::CTL: DWIDTH Mask                   */

#define SPI_CTL_LSB_Pos                  (10)                                              /*!< SPI_T::CTL: LSB Position                  */
#define SPI_CTL_LSB_Msk                  (0x1ul << SPI_CTL_LSB_Pos)                        /*!< SPI_T::CTL: LSB Mask                      */

#define SPI_CTL_CLKPOL_Pos               (11)                                              /*!< SPI_T::CTL: CLKPOL Position               */
#define SPI_CTL_CLKPOL_Msk               (0x1ul << SPI_CTL_CLKPOL_Pos)                     /*!< SPI_T::CTL: CLKPOL Mask                   */

#define SPI_CTL_SUSPITV_Pos              (12)                                              /*!< SPI_T::CTL: SUSPITV Position              */
#define SPI_CTL_SUSPITV_Msk              (0xful << SPI_CTL_SUSPITV_Pos)                    /*!< SPI_T::CTL: SUSPITV Mask                  */

#define SPI_CTL_UNITIF_Pos               (16)                                              /*!< SPI_T::CTL: UNITIF Position               */
#define SPI_CTL_UNITIF_Msk               (0x1ul << SPI_CTL_UNITIF_Pos)                     /*!< SPI_T::CTL: UNITIF Mask                   */

#define SPI_CTL_UNITIEN_Pos              (17)                                              /*!< SPI_T::CTL: UNITIEN Position              */
#define SPI_CTL_UNITIEN_Msk              (0x1ul << SPI_CTL_UNITIEN_Pos)                    /*!< SPI_T::CTL: UNITIEN Mask                  */

#define SPI_CTL_SLAVE_Pos                (18)                                              /*!< SPI_T::CTL: SLAVE Position                */
#define SPI_CTL_SLAVE_Msk                (0x1ul << SPI_CTL_SLAVE_Pos)                      /*!< SPI_T::CTL: SLAVE Mask                    */

#define SPI_CTL_REORDER_Pos              (19)                                              /*!< SPI_T::CTL: REORDER Position              */
#define SPI_CTL_REORDER_Msk              (0x1ul << SPI_CTL_REORDER_Pos)                    /*!< SPI_T::CTL: REORDER Mask                  */

#define SPI_CTL_FIFOEN_Pos               (21)                                              /*!< SPI_T::CTL: FIFOEN Position               */
#define SPI_CTL_FIFOEN_Msk               (0x1ul << SPI_CTL_FIFOEN_Pos)                     /*!< SPI_T::CTL: FIFOEN Mask                   */

#define SPI_CTL_RXEMPTY_Pos              (24)                                              /*!< SPI_T::CTL: RXEMPTY Position              */
#define SPI_CTL_RXEMPTY_Msk              (0x1ul << SPI_CTL_RXEMPTY_Pos)                    /*!< SPI_T::CTL: RXEMPTY Mask                  */

#define SPI_CTL_RXFULL_Pos               (25)                                              /*!< SPI_T::CTL: RXFULL Position               */
#define SPI_CTL_RXFULL_Msk               (0x1ul << SPI_CTL_RXFULL_Pos)                     /*!< SPI_T::CTL: RXFULL Mask                   */

#define SPI_CTL_TXEMPTY_Pos              (26)                                              /*!< SPI_T::CTL: TXEMPTY Position              */
#define SPI_CTL_TXEMPTY_Msk              (0x1ul << SPI_CTL_TXEMPTY_Pos)                    /*!< SPI_T::CTL: TXEMPTY Mask                  */

#define SPI_CTL_TXFULL_Pos               (27)                                              /*!< SPI_T::CTL: TXFULL Position               */
#define SPI_CTL_TXFULL_Msk               (0x1ul << SPI_CTL_TXFULL_Pos)                     /*!< SPI_T::CTL: TXFULL Mask                   */

#define SPI_CLKDIV_DIVIDER_Pos           (0)                                               /*!< SPI_T::CLKDIV: DIVIDER Position           */
#define SPI_CLKDIV_DIVIDER_Msk           (0xfful << SPI_CLKDIV_DIVIDER_Pos)                /*!< SPI_T::CLKDIV: DIVIDER Mask               */

#define SPI_SSCTL_SS_Pos                 (0)                                               /*!< SPI_T::SSCTL: SS Position                 */
#define SPI_SSCTL_SS_Msk                 (0x1ul << SPI_SSCTL_SS_Pos)                       /*!< SPI_T::SSCTL: SS Mask                     */

#define SPI_SSCTL_SSACTPOL_Pos           (2)                                               /*!< SPI_T::SSCTL: SSACTPOL Position           */
#define SPI_SSCTL_SSACTPOL_Msk           (0x1ul << SPI_SSCTL_SSACTPOL_Pos)                 /*!< SPI_T::SSCTL: SSACTPOL Mask               */

#define SPI_SSCTL_AUTOSS_Pos             (3)                                               /*!< SPI_T::SSCTL: AUTOSS Position             */
#define SPI_SSCTL_AUTOSS_Msk             (0x1ul << SPI_SSCTL_AUTOSS_Pos)                   /*!< SPI_T::SSCTL: AUTOSS Mask                 */

#define SPI_SSCTL_SSLTEN_Pos             (4)                                               /*!< SPI_T::SSCTL: SSLTEN Position             */
#define SPI_SSCTL_SSLTEN_Msk             (0x1ul << SPI_SSCTL_SSLTEN_Pos)                   /*!< SPI_T::SSCTL: SSLTEN Mask                 */

#define SPI_SSCTL_LTF_Pos                (5)                                               /*!< SPI_T::SSCTL: LTF Position                */
#define SPI_SSCTL_LTF_Msk                (0x1ul << SPI_SSCTL_LTF_Pos)                      /*!< SPI_T::SSCTL: LTF Mask                    */

#define SPI_RX_RX_Pos                    (0)                                               /*!< SPI_T::RX: RX Position                    */
#define SPI_RX_RX_Msk                    (0xfffffffful << SPI_RX_RX_Pos)                   /*!< SPI_T::RX: RX Mask                        */

#define SPI_TX_TX_Pos                    (0)                                               /*!< SPI_T::TX: TX Position                    */
#define SPI_TX_TX_Msk                    (0xfffffffful << SPI_TX_TX_Pos)                   /*!< SPI_T::TX: TX Mask                        */

#define SPI_SLVCTL_SLV3WIRE_Pos          (8)                                               /*!< SPI_T::SLVCTL: SLV3WIRE Position          */
#define SPI_SLVCTL_SLV3WIRE_Msk          (0x1ul << SPI_SLVCTL_SLV3WIRE_Pos)                /*!< SPI_T::SLVCTL: SLV3WIRE Mask              */

#define SPI_SLVCTL_SLVABT_Pos            (9)                                               /*!< SPI_T::SLVCTL: SLVABT Position            */
#define SPI_SLVCTL_SLVABT_Msk            (0x1ul << SPI_SLVCTL_SLVABT_Pos)                  /*!< SPI_T::SLVCTL: SLVABT Mask                */

#define SPI_SLVCTL_SLVSTIEN_Pos          (10)                                              /*!< SPI_T::SLVCTL: SLVSTIEN Position          */
#define SPI_SLVCTL_SLVSTIEN_Msk          (0x1ul << SPI_SLVCTL_SLVSTIEN_Pos)                /*!< SPI_T::SLVCTL: SLVSTIEN Mask              */

#define SPI_SLVCTL_SLVSTIF_Pos           (11)                                              /*!< SPI_T::SLVCTL: SLVSTIF Position           */
#define SPI_SLVCTL_SLVSTIF_Msk           (0x1ul << SPI_SLVCTL_SLVSTIF_Pos)                 /*!< SPI_T::SLVCTL: SLVSTIF Mask               */

#define SPI_SLVCTL_SSINAIEN_Pos          (16)                                              /*!< SPI_T::SLVCTL: SSINAIEN Position          */
#define SPI_SLVCTL_SSINAIEN_Msk          (0x1ul << SPI_SLVCTL_SSINAIEN_Pos)                /*!< SPI_T::SLVCTL: SSINAIEN Mask              */

#define SPI_SLVCTL_DIVMOD_Pos            (31)                                              /*!< SPI_T::SLVCTL: DIVMOD Position            */
#define SPI_SLVCTL_DIVMOD_Msk            (0x1ul << SPI_SLVCTL_DIVMOD_Pos)                  /*!< SPI_T::SLVCTL: DIVMOD Mask                */

#define SPI_FIFOCTL_RXRST_Pos            (0)                                               /*!< SPI_T::FIFOCTL: RXRST Position            */
#define SPI_FIFOCTL_RXRST_Msk            (0x1ul << SPI_FIFOCTL_RXRST_Pos)                  /*!< SPI_T::FIFOCTL: RXRST Mask                */

#define SPI_FIFOCTL_TXRST_Pos            (1)                                               /*!< SPI_T::FIFOCTL: TXRST Position            */
#define SPI_FIFOCTL_TXRST_Msk            (0x1ul << SPI_FIFOCTL_TXRST_Pos)                  /*!< SPI_T::FIFOCTL: TXRST Mask                */

#define SPI_FIFOCTL_RXTHIEN_Pos          (2)                                               /*!< SPI_T::FIFOCTL: RXTHIEN Position          */
#define SPI_FIFOCTL_RXTHIEN_Msk          (0x1ul << SPI_FIFOCTL_RXTHIEN_Pos)                /*!< SPI_T::FIFOCTL: RXTHIEN Mask              */

#define SPI_FIFOCTL_TXTHIEN_Pos          (3)                                               /*!< SPI_T::FIFOCTL: TXTHIEN Position          */
#define SPI_FIFOCTL_TXTHIEN_Msk          (0x1ul << SPI_FIFOCTL_TXTHIEN_Pos)                /*!< SPI_T::FIFOCTL: TXTHIEN Mask              */

#define SPI_FIFOCTL_RXOVIEN_Pos          (6)                                               /*!< SPI_T::FIFOCTL: RXOVIEN Position          */
#define SPI_FIFOCTL_RXOVIEN_Msk          (0x1ul << SPI_FIFOCTL_RXOVIEN_Pos)                /*!< SPI_T::FIFOCTL: RXOVIEN Mask              */

#define SPI_FIFOCTL_RXTOIEN_Pos          (21)                                              /*!< SPI_T::FIFOCTL: RXTOIEN Position          */
#define SPI_FIFOCTL_RXTOIEN_Msk          (0x1ul << SPI_FIFOCTL_RXTOIEN_Pos)                /*!< SPI_T::FIFOCTL: RXTOIEN Mask              */

#define SPI_FIFOCTL_RXTH_Pos             (24)                                              /*!< SPI_T::FIFOCTL: RXTH Position             */
#define SPI_FIFOCTL_RXTH_Msk             (0x3ul << SPI_FIFOCTL_RXTH_Pos)                   /*!< SPI_T::FIFOCTL: RXTH Mask                 */

#define SPI_FIFOCTL_TXTH_Pos             (28)                                              /*!< SPI_T::FIFOCTL: TXTH Position             */
#define SPI_FIFOCTL_TXTH_Msk             (0x3ul << SPI_FIFOCTL_TXTH_Pos)                   /*!< SPI_T::FIFOCTL: TXTH Mask                 */

#define SPI_STATUS_RXTHIF_Pos            (0)                                               /*!< SPI_T::STATUS: RXTHIF Position            */
#define SPI_STATUS_RXTHIF_Msk            (0x1ul << SPI_STATUS_RXTHIF_Pos)                  /*!< SPI_T::STATUS: RXTHIF Mask                */

#define SPI_STATUS_RXOVIF_Pos            (2)                                               /*!< SPI_T::STATUS: RXOVIF Position            */
#define SPI_STATUS_RXOVIF_Msk            (0x1ul << SPI_STATUS_RXOVIF_Pos)                  /*!< SPI_T::STATUS: RXOVIF Mask                */

#define SPI_STATUS_TXTHIF_Pos            (4)                                               /*!< SPI_T::STATUS: TXTHIF Position            */
#define SPI_STATUS_TXTHIF_Msk            (0x1ul << SPI_STATUS_TXTHIF_Pos)                  /*!< SPI_T::STATUS: TXTHIF Mask                */

#define SPI_STATUS_SLVSTIF_Pos           (11)                                              /*!< SPI_T::STATUS: SLVSTIF Position           */
#define SPI_STATUS_SLVSTIF_Msk           (0x1ul << SPI_STATUS_SLVSTIF_Pos)                 /*!< SPI_T::STATUS: SLVSTIF Mask               */

#define SPI_STATUS_RXCNT_Pos             (12)                                              /*!< SPI_T::STATUS: RXCNT Position             */
#define SPI_STATUS_RXCNT_Msk             (0xful << SPI_STATUS_RXCNT_Pos)                   /*!< SPI_T::STATUS: RXCNT Mask                 */

#define SPI_STATUS_UNITIF_Pos            (16)                                              /*!< SPI_T::STATUS: UNITIF Position            */
#define SPI_STATUS_UNITIF_Msk            (0x1ul << SPI_STATUS_UNITIF_Pos)                  /*!< SPI_T::STATUS: UNITIF Mask                */

#define SPI_STATUS_SLVTOIF_Pos           (20)                                              /*!< SPI_T::STATUS: SLVTOIF Position           */
#define SPI_STATUS_SLVTOIF_Msk           (0x1ul << SPI_STATUS_SLVTOIF_Pos)                 /*!< SPI_T::STATUS: SLVTOIF Mask               */

#define SPI_STATUS_RXEMPTY_Pos           (24)                                              /*!< SPI_T::STATUS: RXEMPTY Position           */
#define SPI_STATUS_RXEMPTY_Msk           (0x1ul << SPI_STATUS_RXEMPTY_Pos)                 /*!< SPI_T::STATUS: RXEMPTY Mask               */

#define SPI_STATUS_RXFULL_Pos            (25)                                              /*!< SPI_T::STATUS: RXFULL Position            */
#define SPI_STATUS_RXFULL_Msk            (0x1ul << SPI_STATUS_RXFULL_Pos)                  /*!< SPI_T::STATUS: RXFULL Mask                */

#define SPI_STATUS_TXEMPTY_Pos           (26)                                              /*!< SPI_T::STATUS: TXEMPTY Position           */
#define SPI_STATUS_TXEMPTY_Msk           (0x1ul << SPI_STATUS_TXEMPTY_Pos)                 /*!< SPI_T::STATUS: TXEMPTY Mask               */

#define SPI_STATUS_TXFULL_Pos            (27)                                              /*!< SPI_T::STATUS: TXFULL Position            */
#define SPI_STATUS_TXFULL_Msk            (0x1ul << SPI_STATUS_TXFULL_Pos)                  /*!< SPI_T::STATUS: TXFULL Mask                */

#define SPI_STATUS_TXCNT_Pos             (28)                                              /*!< SPI_T::STATUS: TXCNT Position             */
#define SPI_STATUS_TXCNT_Msk             (0xful << SPI_STATUS_TXCNT_Pos)                   /*!< SPI_T::STATUS: TXCNT Mask                 */

/**@}*/ /* SPI_CONST */
/**@}*/ /* end of SPI register group */


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
     * Offset: 0x00  Part Device Identification Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[31:0]  |PDID      |Product Device Identification Number
     * |        |          |This register reflects the device part number code.
     * |        |          |Software can read this register to identify which device is used.
    */
    __I  uint32_t PDID;

    /**
     * RSTSTS
     * ===================================================================================================
     * Offset: 0x04  System Reset Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PORF      |Power-On Reset Flag
     * |        |          |The PORF flag is set by the "reset signal", which is from the Power-On Reset (POR) controller or bit CHIPRST (SYS_IPRST0[0]), to indicate the previous reset source.
     * |        |          |0 = No reset from POR or CHIPRST.
     * |        |          |1 = The Power-on-Reset (POR) or CHIPRST had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[1]     |PINRF     |Reset Pin Reset Flag
     * |        |          |The PINRF flag is set by the "reset signal" from the /RESET pin to indicate the previous reset source.
     * |        |          |0 = No reset from pin /RESET pin.
     * |        |          |1 = The /RESET pin had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[2]     |WDTRF     |Watchdog Reset Flag
     * |        |          |The RSTS_WDT flag is set by the "reset signal" from the Watchdog timer to indicate the previous reset source.
     * |        |          |0 = No reset from Watchdog timer.
     * |        |          |1 = The Watchdog timer had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[4]     |BODRF     |Brown-Out Detector Reset Flag
     * |        |          |The BODRF flag is set by the "reset signal" from the Brown-out Detector to indicate the previous reset source.
     * |        |          |0 = No reset from BOD.
     * |        |          |1 = The BOD had issued the reset signal to reset the system.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[5]     |SYSRF     |MCU Reset Flag
     * |        |          |The SYSRF flag is set by the "reset signal" from the Cortex-M0 core to indicate the previous reset source.
     * |        |          |0 = No reset from Cortex-M0.
     * |        |          |1 = The Cortex-M0 had issued the reset signal to reset the system by writing 1 to bit SYSRESETREQ (AIRCR[2]), Application Interrupt and Reset Control Register, address = 0xE000ED0C) in system control registers of Cortex-M0 core.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
     * |[7]     |CPURF     |CPU Reset Flag
     * |        |          |The CPURF flag is set by hardware if software writes CPURST (SYS_IPRST0[1]) 1 to reset Cortex-M0 core and Flash memory controller (FMC).
     * |        |          |0 = No reset from CPU.
     * |        |          |1 = Cortex-M0 core and FMC are reset by software setting CPURST to 1.
     * |        |          |Note: Software can write 1 to clear this bit to zero.
    */
    __IO uint32_t RSTSTS;

    /**
     * IPRST0
     * ===================================================================================================
     * Offset: 0x08  Peripheral Reset Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIPRST   |CHIP One-Shot Reset (Write Protect)
     * |        |          |Setting this bit will reset the CHIP, including CPU kernel and all peripherals, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |The CHIPRST is the same as the POR reset, and all the chip module is reset and the chip settings from flash are also reload.
     * |        |          |0 = Chip normal operation.
     * |        |          |1 = CHIP one-shot reset.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGLCTL at address SYS_BA + 0x100.
     * |[1]     |CPURST    |CPU Kernel One Shot Reset
     * |        |          |Setting this bit will reset the CPU kernel, and this bit will automatically return to 0 after the 2 clock cycles.
     * |        |          |0 = Normal.
     * |        |          |1 = Reset CPU.
     * |        |          |Note: This bit is the protected bit, and programming it needs to write 0x59, 0x16, and 0x88 to address 0x5000_0100 to disable register protection.
     * |        |          |Refer to the register REGLCTL at address SYS_BA + 0x100.
     * |[2]     |CPUWS     |CPU Wait-State Control For Flash Memory Access
     * |        |          |0: Insert one wait-state when access Flash
     * |        |          |1: Non-insert wait-state when access Flash
     * |        |          |Note: When HCLK frequency is faster than 44MHz, insert one wait state is necessary.
    */
    __IO uint32_t IPRST0;

    /**
     * IPRST1
     * ===================================================================================================
     * Offset: 0x0C  Peripheral Reset Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |GPIORST   |GPIO (P0~P5) Controller Reset
     * |        |          |0 = GPIO normal operation.
     * |        |          |1 = GPIO reset.
     * |[2]     |TMR0RST   |Timer0 Controller Reset
     * |        |          |0 = Timer0 normal operation.
     * |        |          |1 = Timer0 block reset.
     * |[3]     |TMR1RST   |Timer1 Controller Reset
     * |        |          |0 = Timer1 normal operation.
     * |        |          |1 = Timer1 block reset.
     * |[8]     |I2C_RST   |I2C Controller Reset
     * |        |          |0 = I2C normal operation.
     * |        |          |1 = I2C block reset.
     * |[12]    |SPIRST    |SPI Controller Reset
     * |        |          |0 = SPI block normal operation.
     * |        |          |1 = SPI block reset.
     * |[16]    |UART0RST  |UART0 Controller Reset
     * |        |          |0 = UART0 normal operation.
     * |        |          |1 = UART0 block reset.
     * |[17]    |UART1RST  |UART1 Controller Reset
     * |        |          |0 = UART1 normal operation.
     * |        |          |1 = UART1 block reset.
     * |[20]    |PWMRST    |PWM Controller Reset
     * |        |          |0 = PWM block normal operation.
     * |        |          |1 = PWM block reset.
     * |[22]    |ACMPRST   |ACMP Controller Reset
     * |        |          |0 = ACMP block normal operation.
     * |        |          |1 = ACMP block reset.
     * |[28]    |ADCRST    |ADC Controller Reset
     * |        |          |0 = ADC block normal operation.
     * |        |          |1 = ADC block reset.
    */
    __IO uint32_t IPRST1;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED0[2];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * BODCTL
     * ===================================================================================================
     * Offset: 0x18  Brown-out Detector Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BODEN     |Brown-Out Detector Selection Extension (Initiated & Write-Protected Bit)
     * |        |          |The default value is set by flash controller user configuration register config0 bit[23].
     * |        |          |If config0 bit[23] is set to 1, default value of BODEN is 0.
     * |        |          |If config0 bit[23] is set to 0, default value of BODEN is 1.
     * |        |          |0 = Brown-out detector threshold voltage is selected by the table defined in BODVL[1:0].
     * |        |          |1 = Brown-out detector threshold voltage is selected by BODVL[2:0] defined as below.
     * |        |          |BODVL[2:0]
     * |        |          |BOD Threshold Voltage
     * |        |          |000 = 2.2V
     * |        |          |001 = 2.7V
     * |        |          |010 = 3.7V
     * |        |          |011 = 4.4V
     * |        |          |100 = 1.7V
     * |        |          |101 = 2.0V
     * |        |          |110 = 2.4V
     * |        |          |111 = 3.0V
     * |[2:1]   |BODVL1_0  |Brown-Out Detector Threshold Voltage Selection (Initiated & Write-Protected Bit)
     * |        |          |The default value is set by flash controller user configuration register config0 bit[22:21].
     * |        |          |When BODEN = 0, BOD threshold voltage setting (BODVL[1:0]) table as below, BODVL[2] has no effect.
     * |        |          |00 = Reserved
     * |        |          |01 = 2.7V
     * |        |          |10 = 3.7V
     * |        |          |11 = Disable 2.7V and 3.7V
     * |[3]     |BODRSTEN  |Brown-Out Reset Enable (Initiated And Write-Protected Bit)
     * |        |          |0 = Brown-out "INTERRUPT" function Enabled; when the Brown-out Detector function is enable and the detected voltage is lower than the threshold, then assert a signal to interrupt the Cortex-M0 CPU.
     * |        |          |1 = Brown-out "RESET" function Enabled; when the Brown-out Detector function is enable and the detected voltage is lower than the threshold then assert a signal to reset the chip.
     * |        |          |The default value is set by flash controller user configuration register config0 bit[20].
     * |        |          |When the BOREN is enabled and the interrupt is asserted, the interrupt will be kept till the BOREN is set to 0.
     * |        |          |The interrupt for CPU can be blocked by disabling the NVIC in CPU for BOD interrupt or disable the interrupt source by disabling the BOREN and then re-enabling the BOREN function if the BOD function is required.
     * |[4]     |BODIF     |Brown-Out Detector Interrupt Flag
     * |        |          |0 = Brown-out Detector does not detect any voltage draft at VDD down through or up through the voltage of BODVL setting.
     * |        |          |1 = When Brown-out Detector detects the VDD is dropped through the voltage of BODVL setting or the VDD is raised up through the voltage of BODVL setting, this bit is set to 1 and the Brown-out interrupt is requested if Brown-out interrupt is enabled.
     * |[5]     |BODLPM    |Brown-Out Detector Low Power Mode (Write-Protected)
     * |        |          |0 = BOD operate in normal mode (default).
     * |        |          |1 = Enable the BOD low power mode.
     * |        |          |The BOD consumes about 100uA in normal mode, the low power mode can reduce the current to about 1/10 but slow the BOD response.
     * |[6]     |BODOUT    |Brown-Out Detector Output State
     * |        |          |0 = Brown-out Detector status output is 0, the detected voltage is higher than BODVL setting.
     * |        |          |1 = Brown-out Detector status output is 1, the detected voltage is lower than BODVL setting.
     * |[7]     |BODVL2    |Brown-Out Detector Threshold Voltage Selection (Initiated & Write-Protected Bit)
     * |        |          |The default value is set by flash controller user configuration register config0 bit[19].
     * |[8]     |BOREN     |Brown-Out Reset Enable
     * |        |          |The bit will enable BOR reset function. When VDD5V lower than 1.7v BOR will reset whole chip.
     * |        |          |0: Disable
     * |        |          |1: Enable
    */
    __IO uint32_t BODCTL;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED1[5];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * P0_MFP
     * ===================================================================================================
     * Offset: 0x30  P0 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |MFP       |P0 Multiple Function Selection
     * |        |          |The pin function of P0 depends on P0_MFP and ALT.
     * |[8:15]  |ALT       |P0 Alternate Function Selection
     * |        |          |The pin function of P0 depends on P0_MFP and ALT.
     * |[23:16] |TYPE      |P0[7:0] Input Schmitt Trigger Function Enable
     * |        |          |0 = P0[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P0[7:0] I/O input Schmitt Trigger function Enabled.
     * |[31:24] |HS        |P0[7:0] Slew Rate Control
     * |        |          |0 = P0[7:0] Low slew rate output, 16MHz available.
     * |        |          |1 = P0[7:0] High slew rate output, 24MHz available.
    */
    __IO uint32_t P0_MFP;

    /**
     * P1_MFP
     * ===================================================================================================
     * Offset: 0x34  P1 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |MFP       |P1 Multiple Function Selection
     * |        |          |The pin function of P1 depends on MFP and ALT.
     * |        |          |Refer to P1_ALT Description for details.
     * |[8:15]  |ALT       |P1 Alternate Function Selection
     * |        |          |The pin function of P1 depends on MFP and ALT.
     * |[23:16] |TYPE      |P1[7:0] Input Schmitt Trigger Function Enable
     * |        |          |0 = P1[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P1[7:0] I/O input Schmitt Trigger function Enabled.
     * |[31:24] |HS        |P1[7:0] Slew Rate Control
     * |        |          |0 = P1[7:0] Low slew rate output, 16MHz available.
     * |        |          |1 = P1[7:0] High slew rate output, 24MHz available.
    */
    __IO uint32_t P1_MFP;

    /**
     * P2_MFP
     * ===================================================================================================
     * Offset: 0x38  P2 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |MFP       |P2 Multiple Function Selection
     * |        |          |The pin function of P2 depends on P2_MFP and ALT.
     * |        |          |Refer to ALT Description for details.
     * |[8:15]  |ALT       |P2 Alternate Function Selection
     * |        |          |The pin function of P2 depends on MFP and ALT.
     * |[23:16] |TYPE      |P2[7:0] Input Schmitt Trigger Function Enable
     * |        |          |0 = P2[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P2[7:0] I/O input Schmitt Trigger function Enabled.
     * |[31:24] |HS        |P2[7:0] Slew Rate Control
     * |        |          |0 = P2[7:0] Low slew rate output, 16MHz available.
     * |        |          |1 = P2[7:0] High slew rate output, 24MHz available.
    */
    __IO uint32_t P2_MFP;

    /**
     * P3_MFP
     * ===================================================================================================
     * Offset: 0x3C  P3 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |MFP       |P3 Multiple Function Selection
     * |        |          |The pin function of P3 depends on MFP and ALT.
     * |        |          |Refer to ALT Description for details.
     * |[8:15]  |ALT       |P3 Alternate Function Selection
     * |        |          |The pin function of P3 depends on MFP and ALT.
     * |[23:16] |TYPE      |P3[7:0] Input Schmitt Trigger Function Enable
     * |        |          |0 = P3[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P3[7:0] I/O input Schmitt Trigger function Enabled.
     * |[24]    |P32CTL    |P3.2 Alternate Function Selection Extension
     * |        |          |0 = P3.2 is set by ALT[2] and MFP[2].
     * |        |          |1 = P3.2 is set to CPP1 of ACMP1.
     * |[31:25] |HS        |P3[6:0] Slew Rate Control
     * |        |          |0 = P3[6:0] Low slew rate output, 16MHz available.
     * |        |          |1 = P3[6:0] High slew rate output, 24MHz available.
    */
    __IO uint32_t P3_MFP;

    /**
     * P4_MFP
     * ===================================================================================================
     * Offset: 0x40  P4 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |MFP       |P4 Multiple Function Selection
     * |        |          |The pin function of P4 depends on MFP and P4_ALT.
     * |        |          |Refer to ALT Description for details.
     * |[8:15]  |ALT       |P4 Alternate Function Selection
     * |        |          |The pin function of P4 depends on MFP and ALT.
     * |[23:16] |TYPE      |P4[7:0] Input Schmitt Trigger Function Enable
     * |        |          |0 = P4[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P4[7:0] I/O input Schmitt Trigger function Enabled.
     * |[31:24] |HS        |P4[7:0] Slew Rate Control
     * |        |          |0 = P4[7:0] Low slew rate output, 16MHz available.
     * |        |          |1 = P4[7:0] High slew rate output, 24MHz available.
    */
    __IO uint32_t P4_MFP;

    /**
     * P5_MFP
     * ===================================================================================================
     * Offset: 0x44  P5 Multiple Function and Input Type Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |MFP       |P5 Multiple Function Selection
     * |        |          |The pin function of P5 depends on MFP and ALT.
     * |        |          |Refer to ALT Description for details.
     * |[8:15]  |ALT       |P5 Alternate Function Selection
     * |        |          |The pin function of P5 depends on MFP and ALT.
     * |[23:16] |TYPE      |P5[7:0] Input Schmitt Trigger Function Enable
     * |        |          |0 = P5[7:0] I/O input Schmitt Trigger function Disabled.
     * |        |          |1 = P5[7:0] I/O input Schmitt Trigger function Enabled.
     * |[31:24] |HS        |P5[7:0] Slew Rate Control
     * |        |          |0 = P5[7:0] Low slew rate output, 16MHz available.
     * |        |          |1 = P5[7:0] High slew rate output, 24MHz available.
    */
    __IO uint32_t P5_MFP;

    /**
     * EINT0SEL
     * ===================================================================================================
     * Offset: 0x48  PIN selection
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |SEL       |INT0 SEL GPB3
     * |        |          |0 = INT0 source is P3.2.
     * |        |          |1 = INT0 source is P1.3.
    */
    __IO uint32_t EINT0SEL;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED2[13];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * IRCTCTL
     * ===================================================================================================
     * Offset: 0x80  HFIRC Trim Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQSEL   |Trim Frequency Selection
     * |        |          |This bit is to enable the HFIRC auto trim.
     * |        |          |When setting this bit to 1, the HFIRC auto trim function will trim HFIRC to 22.1184 MHz automatically based on the LXT reference clock.
     * |        |          |During auto trim operation, if LXT clock error is detected or trim retry limitation count reached, this field will be cleared to 0 automatically.
     * |        |          |0 = HFIRC auto trim function Disabled.
     * |        |          |1 = HFIRC auto trim function Enabled and HFIRC trimmed to 22.1184 MHz.
     * |[5:4]   |LOOPSEL   |Trim Calculation Loop
     * |        |          |This field defines trim value calculation based on the number of LXT clock.
     * |        |          |For example, if LOOPSEL is set as "00", auto trim circuit will calculate trim value based on the average frequency difference in 4 LXT clock.
     * |        |          |This field also defines how many times the auto trim circuit will try to update the HFIRC trim value before the frequency of HFIRC is locked.
     * |        |          |Once the HFIRC is locked, the internal trim value update counter will be reset.
     * |        |          |If the trim value update counter reaches this limitation value and frequency of HFIRC is still not locked, the auto trim operation will be disabled and FREQSEL will be cleared to 0.
     * |        |          |00 = Trim value calculation is based on average difference in 4 LXT clock and trim retry count limitation is 64.
     * |        |          |01 = Trim value calculation is based on average difference in 8 LXT clock and trim retry count limitation is 128.
     * |        |          |10 = Trim value calculation is based on average difference in 16 LXT clock and trim retry count limitation is 256.
     * |        |          |11 = Trim value calculation is based on average difference in 32 LXT clock and trim retry count limitation is 512.
    */
    __IO uint32_t IRCTCTL;

    /**
     * IRCTIEN
     * ===================================================================================================
     * Offset: 0x84  HFIRC Trim Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TFAILIEN  |Trim Failure Interrupt Enable
     * |        |          |This bit controls if an interrupt will be triggered while HFIRC trim value update limitation count is reached and HFIRC frequency is still not locked on target frequency set by FREQSEL.
     * |        |          |If this bit is high and TFAILIF is set during auto trim operation, an interrupt will be triggered to notify that HFIRC trim value update limitation count is reached.
     * |        |          |0 = TFAILIF status Disabled to trigger an interrupt to CPU.
     * |        |          |1 = TFAILIF status Enabled to trigger an interrupt to CPU.
     * |[2]     |CLKEIEN   |LXT Clock Error Interrupt Enable
     * |        |          |This bit controls if CPU could get an interrupt while LXT clock is inaccurate during auto trim operation.
     * |        |          |If this bit is high, and CLKERRIF is set during auto trim operation, an interrupt will be triggered to notify the LXT clock frequency is inaccurate.
     * |        |          |0 = CLKERRIF status Disabled to trigger an interrupt to CPU.
     * |        |          |1 = CLKERRIF status Enabled to trigger an interrupt to CPU.
    */
    __IO uint32_t IRCTIEN;

    /**
     * IRCTISTS
     * ===================================================================================================
     * Offset: 0x88  HFIRC Trim Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FREQLOCK  |HFIRC Frequency Lock Status
     * |        |          |This bit indicates the HFIRC frequency locked in 22.1184 MHz.
     * |        |          |This is a read only status bit and doesn't trigger any interrupt.
     * |[1]     |TFAILIF   |Trim Failure Interrupt Status
     * |        |          |This bit indicates that HFIRC trim value update limitation count reached and HFIRC clock frequency still doesn't lock.
     * |        |          |Once this bit is set, the auto trim operation stopped and FREQSEL will be cleared to 0 by hardware automatically.
     * |        |          |If this bit is set and TFAILIEN is high, an interrupt will be triggered to notify that HFIRC trim value update limitation count was reached.
     * |        |          |Software can write 1 to clear this bit to zero.
     * |        |          |0 = Trim value update limitation count is not reached.
     * |        |          |1 = Trim value update limitation count is reached and HFIRC frequency is still not locked.
     * |[2]     |CLKERRIF  |LXT Clock Error Interrupt Status
     * |        |          |This bit indicates that LXT clock frequency is inaccuracy.
     * |        |          |Once this bit is set, the auto trim operation stopped and FREQSEL will be cleared to 0 by hardware automatically.
     * |        |          |If this bit is set and CLKEIEN is high, an interrupt will be triggered to notify the LXT clock frequency is inaccuracy.
     * |        |          |Software can write 1 to clear this bit to zero.
     * |        |          |0 = LXT clock frequency is accuracy.
     * |        |          |1 = LXT clock frequency is inaccuracy.
    */
    __IO uint32_t IRCTISTS;
    /// @cond HIDDEN_SYMBOLS
    uint32_t RESERVED3[29];
    /// @endcond //HIDDEN_SYMBOLS


    /**
     * REGLCTL
     * ===================================================================================================
     * Offset: 0x100  Register Write-Protection Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |REGWRPROT |Register Write-Protection Disable Index (Read Only)
     * |        |          |0 = Write-protection Enabled for writing protected registers.
     * |        |          |Any write to the protected register is ignored.
     * |        |          |1 = Write-protection Disabled for writing protected registers.
     * |        |          |The Protected registers are:
     * |        |          |SYS_IPRST0, SYS_BODCTL, SYS_LDOCR, SYS_PORCTL, CLK_PWRCTL, CLK_APBCLK, CLK_CLKSEL0, CLK_CLKSEL1, NMI_SEL, FMC_ISPCTL, FMC_ISPTRG, WDT_CTL
     * |        |          |Note: The bits which are write-protected will be noted as" (Write Protect)" beside the description.
     * |[7:0]   |REGPROTDIS|Register Write-Protection Code (Write Only)
     * |        |          |Some registers have write-protection function.
     * |        |          |Writing these registers have to disable the protected function by writing the sequence value 0x59, 0x16, 0x88 to this field.
     * |        |          |After this sequence is completed, the REGWRPROT bit will be set to 1 and write-protection registers can be normal write.
    */
    __IO  uint32_t REGLCTL;

} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */

#define SYS_PDID_PDID_Pos                (0)                                               /*!< SYS_T::PDID: PDID Position                */
#define SYS_PDID_PDID_Msk                (0xfffffffful << SYS_PDID_PDID_Pos)               /*!< SYS_T::PDID: PDID Mask                    */

#define SYS_RSTSTS_PORF_Pos              (0)                                               /*!< SYS_T::RSTSTS: PORF Position              */
#define SYS_RSTSTS_PORF_Msk              (0x1ul << SYS_RSTSTS_PORF_Pos)                    /*!< SYS_T::RSTSTS: PORF Mask                  */

#define SYS_RSTSTS_PINRF_Pos             (1)                                               /*!< SYS_T::RSTSTS: PINRF Position             */
#define SYS_RSTSTS_PINRF_Msk             (0x1ul << SYS_RSTSTS_PINRF_Pos)                   /*!< SYS_T::RSTSTS: PINRF Mask                 */

#define SYS_RSTSTS_WDTRF_Pos             (2)                                               /*!< SYS_T::RSTSTS: WDTRF Position             */
#define SYS_RSTSTS_WDTRF_Msk             (0x1ul << SYS_RSTSTS_WDTRF_Pos)                   /*!< SYS_T::RSTSTS: WDTRF Mask                 */

#define SYS_RSTSTS_BODRF_Pos             (4)                                               /*!< SYS_T::RSTSTS: BODRF Position             */
#define SYS_RSTSTS_BODRF_Msk             (0x1ul << SYS_RSTSTS_BODRF_Pos)                   /*!< SYS_T::RSTSTS: BODRF Mask                 */

#define SYS_RSTSTS_SYSRF_Pos             (5)                                               /*!< SYS_T::RSTSTS: SYSRF Position             */
#define SYS_RSTSTS_SYSRF_Msk             (0x1ul << SYS_RSTSTS_SYSRF_Pos)                   /*!< SYS_T::RSTSTS: SYSRF Mask                 */

#define SYS_RSTSTS_CPURF_Pos             (7)                                               /*!< SYS_T::RSTSTS: CPURF Position             */
#define SYS_RSTSTS_CPURF_Msk             (0x1ul << SYS_RSTSTS_CPURF_Pos)                   /*!< SYS_T::RSTSTS: CPURF Mask                 */

#define SYS_IPRST0_CHIPRST_Pos           (0)                                               /*!< SYS_T::IPRST0: CHIPRST Position           */
#define SYS_IPRST0_CHIPRST_Msk           (0x1ul << SYS_IPRST0_CHIPRST_Pos)                 /*!< SYS_T::IPRST0: CHIPRST Mask               */

#define SYS_IPRST0_CPURST_Pos            (1)                                               /*!< SYS_T::IPRST0: CPURST Position            */
#define SYS_IPRST0_CPURST_Msk            (0x1ul << SYS_IPRST0_CPURST_Pos)                  /*!< SYS_T::IPRST0: CPURST Mask                */

#define SYS_IPRST0_CPUWS_Pos             (2)                                               /*!< SYS_T::IPRST0: CPUWS Position             */
#define SYS_IPRST0_CPUWS_Msk             (0x1ul << SYS_IPRST0_CPUWS_Pos)                   /*!< SYS_T::IPRST0: CPUWS Mask                 */

#define SYS_IPRST1_GPIORST_Pos           (1)                                               /*!< SYS_T::IPRST1: GPIORST Position           */
#define SYS_IPRST1_GPIORST_Msk           (0x1ul << SYS_IPRST1_GPIORST_Pos)                 /*!< SYS_T::IPRST1: GPIORST Mask               */

#define SYS_IPRST1_TMR0RST_Pos           (2)                                               /*!< SYS_T::IPRST1: TMR0RST Position           */
#define SYS_IPRST1_TMR0RST_Msk           (0x1ul << SYS_IPRST1_TMR0RST_Pos)                 /*!< SYS_T::IPRST1: TMR0RST Mask               */

#define SYS_IPRST1_TMR1RST_Pos           (3)                                               /*!< SYS_T::IPRST1: TMR1RST Position           */
#define SYS_IPRST1_TMR1RST_Msk           (0x1ul << SYS_IPRST1_TMR1RST_Pos)                 /*!< SYS_T::IPRST1: TMR1RST Mask               */

#define SYS_IPRST1_I2C_RST_Pos           (8)                                               /*!< SYS_T::IPRST1: I2C_RST Position           */
#define SYS_IPRST1_I2C_RST_Msk           (0x1ul << SYS_IPRST1_I2C_RST_Pos)                 /*!< SYS_T::IPRST1: I2C_RST Mask               */

#define SYS_IPRST1_SPIRST_Pos            (12)                                              /*!< SYS_T::IPRST1: SPIRST Position            */
#define SYS_IPRST1_SPIRST_Msk            (0x1ul << SYS_IPRST1_SPIRST_Pos)                  /*!< SYS_T::IPRST1: SPIRST Mask                */

#define SYS_IPRST1_UART0RST_Pos          (16)                                              /*!< SYS_T::IPRST1: UART0RST Position          */
#define SYS_IPRST1_UART0RST_Msk          (0x1ul << SYS_IPRST1_UART0RST_Pos)                /*!< SYS_T::IPRST1: UART0RST Mask              */

#define SYS_IPRST1_UART1RST_Pos          (17)                                              /*!< SYS_T::IPRST1: UART1RST Position          */
#define SYS_IPRST1_UART1RST_Msk          (0x1ul << SYS_IPRST1_UART1RST_Pos)                /*!< SYS_T::IPRST1: UART1RST Mask              */

#define SYS_IPRST1_PWMRST_Pos            (20)                                              /*!< SYS_T::IPRST1: PWMRST Position            */
#define SYS_IPRST1_PWMRST_Msk            (0x1ul << SYS_IPRST1_PWMRST_Pos)                  /*!< SYS_T::IPRST1: PWMRST Mask                */

#define SYS_IPRST1_ACMPRST_Pos           (22)                                              /*!< SYS_T::IPRST1: ACMPRST Position           */
#define SYS_IPRST1_ACMPRST_Msk           (0x1ul << SYS_IPRST1_ACMPRST_Pos)                 /*!< SYS_T::IPRST1: ACMPRST Mask               */

#define SYS_IPRST1_ADCRST_Pos            (28)                                              /*!< SYS_T::IPRST1: ADCRST Position            */
#define SYS_IPRST1_ADCRST_Msk            (0x1ul << SYS_IPRST1_ADCRST_Pos)                  /*!< SYS_T::IPRST1: ADCRST Mask                */

#define SYS_BODCTL_BODEN_Pos             (0)                                               /*!< SYS_T::BODCTL: BODEN Position             */
#define SYS_BODCTL_BODEN_Msk             (0x1ul << SYS_BODCTL_BODEN_Pos)                   /*!< SYS_T::BODCTL: BODEN Mask                 */

#define SYS_BODCTL_BODVL1_0_Pos          (1)                                               /*!< SYS_T::BODCTL: BODVL1_0 Position          */
#define SYS_BODCTL_BODVL1_0_Msk          (0x3ul << SYS_BODCTL_BODVL1_0_Pos)                /*!< SYS_T::BODCTL: BODVL1_0 Mask              */

#define SYS_BODCTL_BODRSTEN_Pos          (3)                                               /*!< SYS_T::BODCTL: BODRSTEN Position          */
#define SYS_BODCTL_BODRSTEN_Msk          (0x1ul << SYS_BODCTL_BODRSTEN_Pos)                /*!< SYS_T::BODCTL: BODRSTEN Mask              */

#define SYS_BODCTL_BODIF_Pos             (4)                                               /*!< SYS_T::BODCTL: BODIF Position             */
#define SYS_BODCTL_BODIF_Msk             (0x1ul << SYS_BODCTL_BODIF_Pos)                   /*!< SYS_T::BODCTL: BODIF Mask                 */

#define SYS_BODCTL_BODLPM_Pos            (5)                                               /*!< SYS_T::BODCTL: BODLPM Position            */
#define SYS_BODCTL_BODLPM_Msk            (0x1ul << SYS_BODCTL_BODLPM_Pos)                  /*!< SYS_T::BODCTL: BODLPM Mask                */

#define SYS_BODCTL_BODOUT_Pos            (6)                                               /*!< SYS_T::BODCTL: BODOUT Position            */
#define SYS_BODCTL_BODOUT_Msk            (0x1ul << SYS_BODCTL_BODOUT_Pos)                  /*!< SYS_T::BODCTL: BODOUT Mask                */

#define SYS_BODCTL_BODVL2_Pos            (7)                                               /*!< SYS_T::BODCTL: BODVL2 Position            */
#define SYS_BODCTL_BODVL2_Msk            (0x1ul << SYS_BODCTL_BODVL2_Pos)                  /*!< SYS_T::BODCTL: BODVL2 Mask                */

#define SYS_BODCTL_BOREN_Pos             (8)                                               /*!< SYS_T::BODCTL: BOREN Position             */
#define SYS_BODCTL_BOREN_Msk             (0x1ul << SYS_BODCTL_BOREN_Pos)                   /*!< SYS_T::BODCTL: BOREN Mask                 */

#define SYS_P0_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P0_MFP: MFP Position               */
#define SYS_P0_MFP_MFP_Msk               (0xfful << SYS_P0_MFP_MFP_Pos)                    /*!< SYS_T::P0_MFP: MFP Mask                   */

#define SYS_P0_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P0_MFP: ALT0 Position              */
#define SYS_P0_MFP_ALT0_Msk              (0x1ul << SYS_P0_MFP_ALT0_Pos)                    /*!< SYS_T::P0_MFP: ALT0 Mask                  */

#define SYS_P0_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P0_MFP: ALT1 Position              */
#define SYS_P0_MFP_ALT1_Msk              (0x1ul << SYS_P0_MFP_ALT1_Pos)                    /*!< SYS_T::P0_MFP: ALT1 Mask                  */

#define SYS_P0_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P0_MFP: ALT2 Position              */
#define SYS_P0_MFP_ALT2_Msk              (0x1ul << SYS_P0_MFP_ALT2_Pos)                    /*!< SYS_T::P0_MFP: ALT2 Mask                  */

#define SYS_P0_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P0_MFP: ALT4 Position              */
#define SYS_P0_MFP_ALT4_Msk              (0x1ul << SYS_P0_MFP_ALT4_Pos)                    /*!< SYS_T::P0_MFP: ALT4 Mask                  */

#define SYS_P0_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P0_MFP: ALT5 Position              */
#define SYS_P0_MFP_ALT5_Msk              (0x1ul << SYS_P0_MFP_ALT5_Pos)                    /*!< SYS_T::P0_MFP: ALT5 Mask                  */

#define SYS_P0_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P0_MFP: ALT6 Position              */
#define SYS_P0_MFP_ALT6_Msk              (0x1ul << SYS_P0_MFP_ALT6_Pos)                    /*!< SYS_T::P0_MFP: ALT6 Mask                  */

#define SYS_P0_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P0_MFP: ALT7 Position              */
#define SYS_P0_MFP_ALT7_Msk              (0x1ul << SYS_P0_MFP_ALT7_Pos)                    /*!< SYS_T::P0_MFP: ALT7 Mask                  */

#define SYS_P0_MFP_TYPE_Pos              (16)                                              /*!< SYS_T::P0_MFP: TYPE Position              */
#define SYS_P0_MFP_TYPE_Msk              (0xfful << SYS_P0_MFP_TYPE_Pos)                   /*!< SYS_T::P0_MFP: TYPE Mask                  */

#define SYS_P0_MFP_HS_Pos                (24)                                              /*!< SYS_T::P0_MFP: HS Position                */
#define SYS_P0_MFP_HS_Msk                (0xfful << SYS_P0_MFP_HS_Pos)                     /*!< SYS_T::P0_MFP: HS Mask                    */

#define SYS_P1_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P1_MFP: MFP Position               */
#define SYS_P1_MFP_MFP_Msk               (0xfful << SYS_P1_MFP_MFP_Pos)                    /*!< SYS_T::P1_MFP: MFP Mask                   */

#define SYS_P1_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P1_MFP: ALT0 Position              */
#define SYS_P1_MFP_ALT0_Msk              (0x1ul << SYS_P1_MFP_ALT0_Pos)                    /*!< SYS_T::P1_MFP: ALT0 Mask                  */

#define SYS_P1_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P1_MFP: ALT2 Position              */
#define SYS_P1_MFP_ALT2_Msk              (0x1ul << SYS_P1_MFP_ALT2_Pos)                    /*!< SYS_T::P1_MFP: ALT2 Mask                  */

#define SYS_P1_MFP_ALT3_Pos              (11)                                              /*!< SYS_T::P1_MFP: ALT3 Position              */
#define SYS_P1_MFP_ALT3_Msk              (0x1ul << SYS_P1_MFP_ALT3_Pos)                    /*!< SYS_T::P1_MFP: ALT3 Mask                  */

#define SYS_P1_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P1_MFP: ALT4 Position              */
#define SYS_P1_MFP_ALT4_Msk              (0x1ul << SYS_P1_MFP_ALT4_Pos)                    /*!< SYS_T::P1_MFP: ALT4 Mask                  */

#define SYS_P1_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P1_MFP: ALT5 Position              */
#define SYS_P1_MFP_ALT5_Msk              (0x1ul << SYS_P1_MFP_ALT5_Pos)                    /*!< SYS_T::P1_MFP: ALT5 Mask                  */

#define SYS_P1_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P1_MFP: ALT6 Position              */
#define SYS_P1_MFP_ALT6_Msk              (0x1ul << SYS_P1_MFP_ALT6_Pos)                    /*!< SYS_T::P1_MFP: ALT6 Mask                  */

#define SYS_P1_MFP_TYPE_Pos              (16)                                              /*!< SYS_T::P1_MFP: TYPE Position              */
#define SYS_P1_MFP_TYPE_Msk              (0xfful << SYS_P1_MFP_TYPE_Pos)                   /*!< SYS_T::P1_MFP: TYPE Mask                  */

#define SYS_P1_MFP_HS_Pos                (24)                                              /*!< SYS_T::P1_MFP: HS Position                */
#define SYS_P1_MFP_HS_Msk                (0xfful << SYS_P1_MFP_HS_Pos)                     /*!< SYS_T::P1_MFP: HS Mask                    */

#define SYS_P2_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P2_MFP: MFP Position               */
#define SYS_P2_MFP_MFP_Msk               (0xfful << SYS_P2_MFP_MFP_Pos)                    /*!< SYS_T::P2_MFP: MFP Mask                   */

#define SYS_P2_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P2_MFP: ALT2 Position              */
#define SYS_P2_MFP_ALT2_Msk              (0x1ul << SYS_P2_MFP_ALT2_Pos)                    /*!< SYS_T::P2_MFP: ALT2 Mask                  */

#define SYS_P2_MFP_ALT3_Pos              (11)                                              /*!< SYS_T::P2_MFP: ALT3 Position              */
#define SYS_P2_MFP_ALT3_Msk              (0x1ul << SYS_P2_MFP_ALT3_Pos)                    /*!< SYS_T::P2_MFP: ALT3 Mask                  */

#define SYS_P2_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P2_MFP: ALT4 Position              */
#define SYS_P2_MFP_ALT4_Msk              (0x1ul << SYS_P2_MFP_ALT4_Pos)                    /*!< SYS_T::P2_MFP: ALT4 Mask                  */

#define SYS_P2_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P2_MFP: ALT5 Position              */
#define SYS_P2_MFP_ALT5_Msk              (0x1ul << SYS_P2_MFP_ALT5_Pos)                    /*!< SYS_T::P2_MFP: ALT5 Mask                  */

#define SYS_P2_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P2_MFP: ALT6 Position              */
#define SYS_P2_MFP_ALT6_Msk              (0x1ul << SYS_P2_MFP_ALT6_Pos)                    /*!< SYS_T::P2_MFP: ALT6 Mask                  */

#define SYS_P2_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P2_MFP: ALT7 Position              */
#define SYS_P2_MFP_ALT7_Msk              (0x1ul << SYS_P2_MFP_ALT7_Pos)                    /*!< SYS_T::P2_MFP: ALT7 Mask                  */

#define SYS_P2_MFP_TYPE_Pos              (16)                                              /*!< SYS_T::P2_MFP: TYPE Position              */
#define SYS_P2_MFP_TYPE_Msk              (0xfful << SYS_P2_MFP_TYPE_Pos)                   /*!< SYS_T::P2_MFP: TYPE Mask                  */

#define SYS_P2_MFP_HS_Pos                (24)                                              /*!< SYS_T::P2_MFP: HS Position                */
#define SYS_P2_MFP_HS_Msk                (0xfful << SYS_P2_MFP_HS_Pos)                     /*!< SYS_T::P2_MFP: HS Mask                    */

#define SYS_P3_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P3_MFP: MFP Position               */
#define SYS_P3_MFP_MFP_Msk               (0xfful << SYS_P3_MFP_MFP_Pos)                    /*!< SYS_T::P3_MFP: MFP Mask                   */

#define SYS_P3_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P3_MFP: ALT0 Position              */
#define SYS_P3_MFP_ALT0_Msk              (0x1ul << SYS_P3_MFP_ALT0_Pos)                    /*!< SYS_T::P3_MFP: ALT0 Mask                  */

#define SYS_P3_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P3_MFP: ALT1 Position              */
#define SYS_P3_MFP_ALT1_Msk              (0x1ul << SYS_P3_MFP_ALT1_Pos)                    /*!< SYS_T::P3_MFP: ALT1 Mask                  */

#define SYS_P3_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P3_MFP: ALT2 Position              */
#define SYS_P3_MFP_ALT2_Msk              (0x1ul << SYS_P3_MFP_ALT2_Pos)                    /*!< SYS_T::P3_MFP: ALT2 Mask                  */

#define SYS_P3_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P3_MFP: ALT4 Position              */
#define SYS_P3_MFP_ALT4_Msk              (0x1ul << SYS_P3_MFP_ALT4_Pos)                    /*!< SYS_T::P3_MFP: ALT4 Mask                  */

#define SYS_P3_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P3_MFP: ALT5 Position              */
#define SYS_P3_MFP_ALT5_Msk              (0x1ul << SYS_P3_MFP_ALT5_Pos)                    /*!< SYS_T::P3_MFP: ALT5 Mask                  */

#define SYS_P3_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P3_MFP: ALT6 Position              */
#define SYS_P3_MFP_ALT6_Msk              (0x1ul << SYS_P3_MFP_ALT6_Pos)                    /*!< SYS_T::P3_MFP: ALT6 Mask                  */

#define SYS_P3_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P3_MFP: ALT7 Position              */
#define SYS_P3_MFP_ALT7_Msk              (0x1ul << SYS_P3_MFP_ALT7_Pos)                    /*!< SYS_T::P3_MFP: ALT7 Mask                  */

#define SYS_P3_MFP_TYPE_Pos              (16)                                              /*!< SYS_T::P3_MFP: TYPE Position              */
#define SYS_P3_MFP_TYPE_Msk              (0xfful << SYS_P3_MFP_TYPE_Pos)                   /*!< SYS_T::P3_MFP: TYPE Mask                  */

#define SYS_P3_MFP_P32CTL_Pos            (24)                                              /*!< SYS_T::P3_MFP: P32CTL Position            */
#define SYS_P3_MFP_P32CTL_Msk            (0x1ul << SYS_P3_MFP_P32CTL_Pos)                  /*!< SYS_T::P3_MFP: P32CTL Mask                */

#define SYS_P3_MFP_HS_Pos                (25)                                              /*!< SYS_T::P3_MFP: HS Position                */
#define SYS_P3_MFP_HS_Msk                (0x7ful << SYS_P3_MFP_HS_Pos)                     /*!< SYS_T::P3_MFP: HS Mask                    */

#define SYS_P4_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P4_MFP: MFP Position               */
#define SYS_P4_MFP_MFP_Msk               (0xfful << SYS_P4_MFP_MFP_Pos)                    /*!< SYS_T::P4_MFP: MFP Mask                   */

#define SYS_P4_MFP_ALT6_Pos              (14)                                              /*!< SYS_T::P4_MFP: ALT6 Position              */
#define SYS_P4_MFP_ALT6_Msk              (0x1ul << SYS_P4_MFP_ALT6_Pos)                    /*!< SYS_T::P4_MFP: ALT6 Mask                  */

#define SYS_P4_MFP_ALT7_Pos              (15)                                              /*!< SYS_T::P4_MFP: ALT7 Position              */
#define SYS_P4_MFP_ALT7_Msk              (0x1ul << SYS_P4_MFP_ALT7_Pos)                    /*!< SYS_T::P4_MFP: ALT7 Mask                  */

#define SYS_P4_MFP_TYPE_Pos              (16)                                              /*!< SYS_T::P4_MFP: TYPE Position              */
#define SYS_P4_MFP_TYPE_Msk              (0xfful << SYS_P4_MFP_TYPE_Pos)                   /*!< SYS_T::P4_MFP: TYPE Mask                  */

#define SYS_P4_MFP_HS_Pos                (24)                                              /*!< SYS_T::P4_MFP: HS Position                */
#define SYS_P4_MFP_HS_Msk                (0xfful << SYS_P4_MFP_HS_Pos)                     /*!< SYS_T::P4_MFP: HS Mask                    */

#define SYS_P5_MFP_MFP_Pos               (0)                                               /*!< SYS_T::P5_MFP: MFP Position               */
#define SYS_P5_MFP_MFP_Msk               (0xfful << SYS_P5_MFP_MFP_Pos)                    /*!< SYS_T::P5_MFP: MFP Mask                   */

#define SYS_P5_MFP_ALT0_Pos              (8)                                               /*!< SYS_T::P5_MFP: ALT0 Position              */
#define SYS_P5_MFP_ALT0_Msk              (0x1ul << SYS_P5_MFP_ALT0_Pos)                    /*!< SYS_T::P5_MFP: ALT0 Mask                  */

#define SYS_P5_MFP_ALT1_Pos              (9)                                               /*!< SYS_T::P5_MFP: ALT1 Position              */
#define SYS_P5_MFP_ALT1_Msk              (0x1ul << SYS_P5_MFP_ALT1_Pos)                    /*!< SYS_T::P5_MFP: ALT1 Mask                  */

#define SYS_P5_MFP_ALT2_Pos              (10)                                              /*!< SYS_T::P5_MFP: ALT2 Position              */
#define SYS_P5_MFP_ALT2_Msk              (0x1ul << SYS_P5_MFP_ALT2_Pos)                    /*!< SYS_T::P5_MFP: ALT2 Mask                  */

#define SYS_P5_MFP_ALT3_Pos              (11)                                              /*!< SYS_T::P5_MFP: ALT3 Position              */
#define SYS_P5_MFP_ALT3_Msk              (0x1ul << SYS_P5_MFP_ALT3_Pos)                    /*!< SYS_T::P5_MFP: ALT3 Mask                  */

#define SYS_P5_MFP_ALT4_Pos              (12)                                              /*!< SYS_T::P5_MFP: ALT4 Position              */
#define SYS_P5_MFP_ALT4_Msk              (0x1ul << SYS_P5_MFP_ALT4_Pos)                    /*!< SYS_T::P5_MFP: ALT4 Mask                  */

#define SYS_P5_MFP_ALT5_Pos              (13)                                              /*!< SYS_T::P5_MFP: ALT5 Position              */
#define SYS_P5_MFP_ALT5_Msk              (0x1ul << SYS_P5_MFP_ALT5_Pos)                    /*!< SYS_T::P5_MFP: ALT5 Mask                  */

#define SYS_P5_MFP_TYPE_Pos              (16)                                              /*!< SYS_T::P5_MFP: TYPE Position              */
#define SYS_P5_MFP_TYPE_Msk              (0xfful << SYS_P5_MFP_TYPE_Pos)                   /*!< SYS_T::P5_MFP: TYPE Mask                  */

#define SYS_P5_MFP_HS_Pos                (24)                                              /*!< SYS_T::P5_MFP: HS Position                */
#define SYS_P5_MFP_HS_Msk                (0xfful << SYS_P5_MFP_HS_Pos)                     /*!< SYS_T::P5_MFP: HS Mask                    */

#define SYS_EINT0SEL_SEL_Pos             (0)                                               /*!< SYS_T::EINT0SEL: SEL Position             */
#define SYS_EINT0SEL_SEL_Msk             (0x1ul << SYS_EINT0SEL_SEL_Pos)                   /*!< SYS_T::EINT0SEL: SEL Mask                 */

#define SYS_IRCTCTL_FREQSEL_Pos          (0)                                               /*!< SYS_T::IRCTCTL: FREQSEL Position          */
#define SYS_IRCTCTL_FREQSEL_Msk          (0x1ul << SYS_IRCTCTL_FREQSEL_Pos)                /*!< SYS_T::IRCTCTL: FREQSEL Mask              */

#define SYS_IRCTCTL_LOOPSEL_Pos          (4)                                               /*!< SYS_T::IRCTCTL: LOOPSEL Position          */
#define SYS_IRCTCTL_LOOPSEL_Msk          (0x3ul << SYS_IRCTCTL_LOOPSEL_Pos)                /*!< SYS_T::IRCTCTL: LOOPSEL Mask              */

#define SYS_IRCTIEN_TFAILIEN_Pos         (1)                                               /*!< SYS_T::IRCTIEN: TFAILIEN Position         */
#define SYS_IRCTIEN_TFAILIEN_Msk         (0x1ul << SYS_IRCTIEN_TFAILIEN_Pos)               /*!< SYS_T::IRCTIEN: TFAILIEN Mask             */

#define SYS_IRCTIEN_CLKEIEN_Pos          (2)                                               /*!< SYS_T::IRCTIEN: CLKEIEN Position          */
#define SYS_IRCTIEN_CLKEIEN_Msk          (0x1ul << SYS_IRCTIEN_CLKEIEN_Pos)                /*!< SYS_T::IRCTIEN: CLKEIEN Mask              */

#define SYS_IRCTISTS_FREQLOCK_Pos        (0)                                               /*!< SYS_T::IRCTISTS: FREQLOCK Position        */
#define SYS_IRCTISTS_FREQLOCK_Msk        (0x1ul << SYS_IRCTISTS_FREQLOCK_Pos)              /*!< SYS_T::IRCTISTS: FREQLOCK Mask            */

#define SYS_IRCTISTS_TFAILIF_Pos         (1)                                               /*!< SYS_T::IRCTISTS: TFAILIF Position         */
#define SYS_IRCTISTS_TFAILIF_Msk         (0x1ul << SYS_IRCTISTS_TFAILIF_Pos)               /*!< SYS_T::IRCTISTS: TFAILIF Mask             */

#define SYS_IRCTISTS_CLKERRIF_Pos        (2)                                               /*!< SYS_T::IRCTISTS: CLKERRIF Position        */
#define SYS_IRCTISTS_CLKERRIF_Msk        (0x1ul << SYS_IRCTISTS_CLKERRIF_Pos)              /*!< SYS_T::IRCTISTS: CLKERRIF Mask            */

#define SYS_REGLCTL_REGWRPROT_Pos        (0)                                               /*!< SYS_T::REGLCTL: REGWRPROT Position        */
#define SYS_REGLCTL_REGWRPROT_Msk        (0x1ul << SYS_REGLCTL_REGWRPROT_Pos)              /*!< SYS_T::REGLCTL: REGWRPROT Mask            */

#define SYS_REGLCTL_REGPROTDIS_Pos       (0)                                               /*!< SYS_T::REGLCTL: REGPROTDIS Position       */
#define SYS_REGLCTL_REGPROTDIS_Msk       (0x1ul << SYS_REGLCTL_REGPROTDIS_Pos)            /*!< SYS_T::REGLCTL: REGPROTDIS Mask           */

/**@}*/ /* SYS_CONST */
/**@}*/ /* end of SYS register group */


/*---------------------- Timer Controller -------------------------*/
/**
    @addtogroup TIMER Timer Controller(TIMER)
    Memory Mapped Structure for TiMER Controller
@{ */

typedef struct
{


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  Timer0 Control and Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |PSC       |Prescale Counter
     * |        |          |Timer input clock source is divided by (PSC+1) before it is fed to the Timer up counter.
     * |        |          |If this field is 0 (PSC = 0), then there is no scaling.
     * |[16]    |CNTDATEN  |Data Load Enable Control
     * |        |          |When CNTDATEN is set, CNT (TIMERx_CNT[23:0]) (Timer Data Register) will be updated continuously with the 24-bit up-timer value as the timer is counting.
     * |        |          |0 = Timer Data Register update Disabled.
     * |        |          |1 = Timer Data Register update Enabled while Timer counter is active.
     * |[17]    |CMPCTL    |TIMERx_CMP Mode Control
     * |        |          |0 = In One-shot or Periodic mode, when write new CMPDAT, timer counter will reset.
     * |        |          |1 = In One-shot or Periodic mode, when write new CMPDAT if new CMPDAT > CNT (TIMERx_CNT[23:0])(current counter) , timer counter keep counting and will not reset.
     * |        |          |If new CMPDAT <= CNT(current counter) , timer counter will reset.
     * |[18]    |TGLPINSEL |Toggle Out Pin Selection
     * |        |          |When Timer is set to toggle mode,
     * |        |          |0 = Time0/1 toggle output pin is T0/T1 pin.
     * |        |          |1 = Time0/1 toggle output pin is T0EX/T1EX pin.
     * |[19]    |CAPPINSEL |Capture Pin Source Selection
     * |        |          |0 = Capture Function source is from TxEX pin.
     * |        |          |1 = Capture Function source is from ACMPx output signal.
     * |[23]    |WKEN      |Wake-Up Enable
     * |        |          |When WKEN is set and the TIF or CAPIF is set, the timer controller will generator a wake-up trigger event to CPU.
     * |        |          |0 = Wake-up trigger event Disabled.
     * |        |          |1 = Wake-up trigger event Enabled.
     * |[24]    |EXTCNTEN  |Counter Mode Enable Control
     * |        |          |This bit is for external counting pin function enabled.
     * |        |          |When timer is used as an event counter, this bit should be set to 1 and select HCLK as timer clock source.
     * |        |          |Please refer to section 6.12.5.3 for detail description.
     * |        |          |0 = External event counter mode Disabled.
     * |        |          |1 = External event counter mode Enabled.
     * |[25]    |ACTSTS    |Timer Active Status (Read Only)
     * |        |          |This bit indicates the 24-bit up counter status.
     * |        |          |0 = 24-bit up counter is not active.
     * |        |          |1 = 24-bit up counter is active.
     * |[26]    |RSTCNT    |Timer Reset
     * |        |          |0 = No effect.
     * |        |          |1 = Reset 8-bit PSC counter, 24-bit up counter value and CNTEN bit if ACTSTS is 1.
     * |[28:27] |OPMODE    |Timer Operating Mode
     * |        |          |00 = The timer is operating in the One-shot OPMODE.
     * |        |          |The associated interrupt signal is generated once (if INTEN is enabled) and CNTEN is automatically cleared by hardware.
     * |        |          |01 = The timer is operating in Periodic OPMODE.
     * |        |          |The associated interrupt signal is generated periodically (if INTEN is enabled).
     * |        |          |10 = The timer is operating in Toggle OPMODE.
     * |        |          |The interrupt signal is generated periodically (if INTEN is enabled).
     * |        |          |The associated signal (tout) is changing back and forth with 50% duty cycle.
     * |        |          |11 = The timer is operating in Continuous Counting mode.
     * |        |          |The associated interrupt signal is generated when TIMERx_CNT = TIMERx_CMP (if INTEN is enabled).
     * |        |          |However, the 24-bit up-timer counts continuously.
     * |        |          |Please refer to 6.12.5.2 for detailed description about Continuous Counting mode operation.
     * |[29]    |INTEN     |Interrupt Enable Control
     * |        |          |0 = Timer Interrupt function Disabled.
     * |        |          |1 = Timer Interrupt function Enabled.
     * |        |          |If this bit is enabled, when the timer interrupt flag (TIF) is set to 1, the timer interrupt signal is generated and inform to CPU.
     * |[30]    |CNTEN     |Timer Enable Control
     * |        |          |0 = Stops/Suspends counting.
     * |        |          |1 = Starts counting.
     * |        |          |Note1: In stop status, and then set CNTEN to 1 will enable the 24-bit up counter to keep  counting from the last stop counting value.
     * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (TIMERx_CTL[28:27] = 00) when the timer interrupt flag (TIF) is generated.
     * |[31]    |ICEDEBUG  |ICE Debug Mode Acknowledge Disable (Write Protect)
     * |        |          |0 = ICE debug mode acknowledgement effects TIMER counting.
     * |        |          |Timer counter will be held while CPU is held by ICE.
     * |        |          |1 = ICE debug mode acknowledgement Disabled.
     * |        |          |Timer counter will keep going no matter CPU is held by ICE or not.
    */
    __IO uint32_t CTL;

    /**
     * CMP
     * ===================================================================================================
     * Offset: 0x04  Timer0 Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CMPDAT    |Timer Compared Value
     * |        |          |CMPDAT is a 24-bit compared value register.
     * |        |          |When the internal 24-bit up counter value is equal to CMPDAT value, the TIF flag will set to 1.
     * |        |          |Time-out period = (Period of Timer clock source) * (8-bit PSC + 1) * (24-bit CMPDAT).
     * |        |          |Note1: Never write 0x0 or 0x1 in CMPDAT field, or the core will run into unknown state.
     * |        |          |Note2: When Timer is operating at Continuous Counting mode, the 24-bit up counter will keep counting continuously even if software writes a new value into CMPDAT field.
     * |        |          |But if Timer is operating at other modes except Periodic mode on M05xxDN/DE, the 24-bit up counter will restart counting and using newest CMPDAT value to be the timer compared value if software writes a new value into CMPDAT field.
    */
    __IO uint32_t CMP;

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x08  Timer0 Interrupt Status Register
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
    */
    __IO uint32_t INTSTS;

    /**
     * CNT
     * ===================================================================================================
     * Offset: 0x0C  Timer0 Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CNT       |Timer Data Register
     * |        |          |If CNTDATEN is set to 1, CNT register value will be updated continuously to monitor 24-bit up counter value.
    */
    __I  uint32_t CNT;

    /**
     * CAP
     * ===================================================================================================
     * Offset: 0x10  Timer0 Capture Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CAPDAT    |Timer Capture Data Register
     * |        |          |When CAPIF flag is set to 1, the current CNT (TIMERx_CNT[23:0]) value will be auto-loaded into this TCAP filed immediately.
     * |        |          |Note: When CCAPEN is 1, TIMER0_CAP will store the timer0 counter value for second falling edge with TMRSEL=0.
     * |        |          |TIMER1_CAP will store the timer1 counter value for second falling edge with TMRSEL=1.
    */
    __I  uint32_t CAP;

    /**
     * EXTCTL
     * ===================================================================================================
     * Offset: 0x14  Timer0 External Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CNTPHASE  |Timer External Count Pin Phase Detect Selection
     * |        |          |This bit indicates the detection phase of Tx (x = 0~1) pin.
     * |        |          |0 = A falling edge of Tx (x = 0~1) pin will be counted.
     * |        |          |1 = A rising edge of Tx (x = 0~1) pin will be counted.
     * |[2:1]   |CAPEDGE   |Timer External Pin Edge Detection
     * |        |          |00 = A 1 to 0 transition on TxEX (x = 0~1) will be detected.
     * |        |          |01 = A 0 to 1 transition on TxEX (x = 0~1) will be detected.
     * |        |          |10 = Either 1 to 0 or 0 to 1 transition on TxEX (x = 0~1) will be detected.
     * |        |          |11 = Reserved.
     * |[3]     |CAPEN     |Timer External Pin Function Enable
     * |        |          |This bit enables the CAPFUNCS function on the TxEX (x = 0~1) pin.
     * |        |          |0 = CAPFUNCS function of TxEX (x = 0~1) pin will be ignored.
     * |        |          |1 = CAPFUNCS function of TxEX (x = 0~1) pin is active.
     * |[4]     |CAPFUNCS  |Timer External Reset Counter / Timer External Capture Mode Selection
     * |        |          |0 = Transition on TxEX (x = 0~1) pin is using to save the CNT (TIMERx_CNT[23:0]) value into TCAP value if CAPIF flag is set to 1.
     * |        |          |1 = Transition on TxEX (x = 0~1) pin is using to reset the 24-bit up counter.
     * |[5]     |CAPIEN    |Timer External Capture Interrupt Enable Control
     * |        |          |0 = TxEX (x = 0~1) pin detection Interrupt Disabled.
     * |        |          |1 = TxEX (x = 0~1) pin detection Interrupt Enabled.
     * |        |          |If CAPIEN enabled, Timer will raise an external capture interrupt signal and inform to CPU while CAPIF flag is set to 1.
     * |[6]     |CAPDBEN   |Timer External Capture Input Pin De-Bounce Enable Control
     * |        |          |0 = TxEX (x = 0~1) pin de-bounce Disabled.
     * |        |          |1 = TxEX (x = 0~1) pin de-bounce Enabled.
     * |        |          |If this bit is enabled, the edge detection of TxEX (x = 0~1) pin is detected with de-bounce circuit.
     * |[7]     |ECNTDBEN  |Timer External Counter Input Pin De-Bounce Enable Control
     * |        |          |0 = Tx (x = 0~1) pin de-bounce Disabled.
     * |        |          |1 = Tx (x = 0~1) pin de-bounce Enabled.
     * |        |          |If this bit is enabled, the edge detection of Tx (x = 0~1) pin is detected with de-bounce circuit.
     * |[8]     |CAPMODE   |Capture Mode Selection
     * |        |          |0 = Timer counter reset function or free-counting mode of timer capture function.
     * |        |          |1 = Trigger-counting mode of timer capture function.
    */
    __IO uint32_t EXTCTL;

    /**
     * EINTSTS
     * ===================================================================================================
     * Offset: 0x18  Timer0 External Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPIF     |Timer External Interrupt Flag
     * |        |          |This bit indicates the external capture interrupt flag status
     * |        |          |When CAPEN enabled, TxEX (x = 0~1) pin selected as external capture function, and a transition on TxEX (x = 0~1) pin matched the CAPEDGE setting, this flag will set to 1 by hardware.
     * |        |          |0 = TxEX (x = 0~1) pin interrupt did not occur.
     * |        |          |1 = TxEX (x = 0~1) pin interrupt occurred.
     * |        |          |Note: This bit is cleared by writing 1 to it
    */
    __IO uint32_t EINTSTS;

} TIMER_T;

typedef struct
{
    /**
     * CCAPCTL
     * ===================================================================================================
     * Offset: 0x00  Timer Continuous Capture Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CCAPEN    |Continuous Capture Enable
     * |        |          |This bit enables the advanced capture function.
     * |        |          |0 = Disable.
     * |        |          |1 = Enable.
     * |        |          |Note: This bit is cleared by H/W automatically when capture operation finish or writing 0 to it
     * |[1]     |INV       |Input Signal Inverse
     * |        |          |Invert the input signal which be captured.
     * |        |          |0 = None.
     * |        |          |1 = Inverse.
     * |[2]     |TMRSEL    |Capture Timer Selection
     * |        |          |Select the timer to capture the input signal.
     * |        |          |0 = Timer 0.
     * |        |          |1 = Timer 1.
     * |[5:3]   |CAPCHSEL  |Capture Channel Selection
     * |        |          |Select the input channel to be captured.
     * |        |          |000 = P0.0.
     * |        |          |001 = P0.4.
     * |        |          |010 = P0.5.
     * |        |          |011 = P0.6.
     * |        |          |100 = P0.7.
     * |        |          |101 = P5.2.
     * |        |          |110 = P3.0.
     * |        |          |111 = P3.1.
     * |[8]     |CAPR1F    |Capture Rising Edge 1 Flag
     * |        |          |First rising edge already captured, this bit will be set to 1.
     * |        |          |0 = None.
     * |        |          |1 =TIMER_CCAP0 data is ready for read.
     * |        |          |Note: This bit is cleared by H/W automatically when write CCAPEN to 1 or writing 1 to it
     * |[9]     |CAPF1F    |Capture Falling Edge 1 Flag
     * |        |          |First falling edge already captured, this bit will be set to 1.
     * |        |          |0 = None.
     * |        |          |1 =TIMER_CCAP1 data is ready for read.
     * |        |          |Note: This bit is cleared by H/W automatically when write CCAPEN to 1 or writing 1 to it
     * |[10]    |CAPR2F    |Capture Rising Edge 2 Flag
     * |        |          |Second rising edge already captured, this bit will be set to 1.
     * |        |          |0 = None.
     * |        |          |1 =TIMER_CCAP2 data is ready for read.
     * |        |          |Note: This bit is cleared by H/W automatically when write CCAPEN to 1 or writing 1 to it
     * |[11]    |CAPF2F    |Capture Falling Edge 2 Flag
     * |        |          |Second falling edge already captured, this bit will be set to 1.
     * |        |          |0 = None.
     * |        |          |1 = TIMER0_CAP or TIMER1_CAP data is ready for read.
     * |        |          |When TMRSEL=0, TIMER0_CAP store the timer 0 counter value.
     * |        |          |TMRSEL=1, TIMER1_CAP store the timer 1 counter value.
     * |        |          |Note: This bit is cleared by H/W automatically when write CCAPEN to 1 or writing 1 to it
    */
    __IO uint32_t CCAPCTL;

    /**
     * CCAP0
     * ===================================================================================================
     * Offset: 0x04 ~0x0C Timer Continuous Capture Data Register 0~2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[23:0]  |CAPDAT    |Timer Continuous Capture Data Register X
     * |        |          |TIMER_CCAP0 store the timer count value of first rising edge.
     * |        |          |TIMER_CCAP1 store the timer count value of first falling edge.
     * |        |          |TIMER_CCAP2 store the timer count value of second rising edge.
    */
    __I  uint32_t CCAP[3];
} TIMER_AC_T;
/**
    @addtogroup TIMER_CONST TIMER Bit Field Definition
    Constant Definitions for TIMER Controller
@{ */

#define TIMER_CTL_PSC_Pos                  (0)                                               /*!< TIMER_T::CTL: PSC Position                  */
#define TIMER_CTL_PSC_Msk                  (0xfful << TIMER_CTL_PSC_Pos)                     /*!< TIMER_T::CTL: PSC Mask                      */

#define TIMER_CTL_CNTDATEN_Pos             (16)                                              /*!< TIMER_T::CTL: CNTDATEN Position             */
#define TIMER_CTL_CNTDATEN_Msk             (0x1ul << TIMER_CTL_CNTDATEN_Pos)                 /*!< TIMER_T::CTL: CNTDATEN Mask                 */

#define TIMER_CTL_CMPCTL_Pos               (17)                                              /*!< TIMER_T::CTL: CMPCTL Position               */
#define TIMER_CTL_CMPCTL_Msk               (0x1ul << TIMER_CTL_CMPCTL_Pos)                   /*!< TIMER_T::CTL: CMPCTL Mask                   */

#define TIMER_CTL_TGLPINSEL_Pos            (18)                                              /*!< TIMER_T::CTL: TGLPINSEL Position            */
#define TIMER_CTL_TGLPINSEL_Msk            (0x1ul << TIMER_CTL_TGLPINSEL_Pos)                /*!< TIMER_T::CTL: TGLPINSEL Mask                */

#define TIMER_CTL_CAPPINSEL_Pos            (19)                                              /*!< TIMER_T::CTL: CAPPINSEL Position            */
#define TIMER_CTL_CAPPINSEL_Msk            (0x1ul << TIMER_CTL_CAPPINSEL_Pos)                /*!< TIMER_T::CTL: CAPPINSEL Mask                */

#define TIMER_CTL_WKEN_Pos                 (23)                                              /*!< TIMER_T::CTL: WKEN Position                 */
#define TIMER_CTL_WKEN_Msk                 (0x1ul << TIMER_CTL_WKEN_Pos)                     /*!< TIMER_T::CTL: WKEN Mask                     */

#define TIMER_CTL_EXTCNTEN_Pos             (24)                                              /*!< TIMER_T::CTL: EXTCNTEN Position             */
#define TIMER_CTL_EXTCNTEN_Msk             (0x1ul << TIMER_CTL_EXTCNTEN_Pos)                 /*!< TIMER_T::CTL: EXTCNTEN Mask                 */

#define TIMER_CTL_ACTSTS_Pos               (25)                                              /*!< TIMER_T::CTL: ACTSTS Position               */
#define TIMER_CTL_ACTSTS_Msk               (0x1ul << TIMER_CTL_ACTSTS_Pos)                   /*!< TIMER_T::CTL: ACTSTS Mask                   */

#define TIMER_CTL_RSTCNT_Pos               (26)                                              /*!< TIMER_T::CTL: RSTCNT Position               */
#define TIMER_CTL_RSTCNT_Msk               (0x1ul << TIMER_CTL_RSTCNT_Pos)                   /*!< TIMER_T::CTL: RSTCNT Mask                   */

#define TIMER_CTL_OPMODE_Pos               (27)                                              /*!< TIMER_T::CTL: OPMODE Position               */
#define TIMER_CTL_OPMODE_Msk               (0x3ul << TIMER_CTL_OPMODE_Pos)                   /*!< TIMER_T::CTL: OPMODE Mask                   */

#define TIMER_CTL_INTEN_Pos                (29)                                              /*!< TIMER_T::CTL: INTEN Position                */
#define TIMER_CTL_INTEN_Msk                (0x1ul << TIMER_CTL_INTEN_Pos)                    /*!< TIMER_T::CTL: INTEN Mask                    */

#define TIMER_CTL_CNTEN_Pos                (30)                                              /*!< TIMER_T::CTL: CNTEN Position                */
#define TIMER_CTL_CNTEN_Msk                (0x1ul << TIMER_CTL_CNTEN_Pos)                    /*!< TIMER_T::CTL: CNTEN Mask                    */

#define TIMER_CTL_ICEDEBUG_Pos             (31)                                              /*!< TIMER_T::CTL: ICEDEBUG Position             */
#define TIMER_CTL_ICEDEBUG_Msk             (0x1ul << TIMER_CTL_ICEDEBUG_Pos)                 /*!< TIMER_T::CTL: ICEDEBUG Mask                 */

#define TIMER_CMP_CMPDAT_Pos               (0)                                               /*!< TIMER_T::CMP: CMPDAT Position               */
#define TIMER_CMP_CMPDAT_Msk               (0xfffffful << TIMER_CMP_CMPDAT_Pos)              /*!< TIMER_T::CMP: CMPDAT Mask                   */

#define TIMER_INTSTS_TIF_Pos               (0)                                               /*!< TIMER_T::INTSTS: TIF Position               */
#define TIMER_INTSTS_TIF_Msk               (0x1ul << TIMER_INTSTS_TIF_Pos)                   /*!< TIMER_T::INTSTS: TIF Mask                   */

#define TIMER_INTSTS_TWKF_Pos              (1)                                               /*!< TIMER_T::INTSTS: TWKF Position              */
#define TIMER_INTSTS_TWKF_Msk              (0x1ul << TIMER_INTSTS_TWKF_Pos)                  /*!< TIMER_T::INTSTS: TWKF Mask                  */

#define TIMER_CNT_CNT_Pos                  (0)                                               /*!< TIMER_T::CNT: CNT Position                  */
#define TIMER_CNT_CNT_Msk                  (0xfffffful << TIMER_CNT_CNT_Pos)                 /*!< TIMER_T::CNT: CNT Mask                      */

#define TIMER_CAP_CAPDAT_Pos               (0)                                               /*!< TIMER_T::CAP: CAPDAT Position               */
#define TIMER_CAP_CAPDAT_Msk               (0xfffffful << TIMER_CAP_CAPDAT_Pos)              /*!< TIMER_T::CAP: CAPDAT Mask                   */

#define TIMER_EXTCTL_CNTPHASE_Pos          (0)                                               /*!< TIMER_T::EXTCTL: CNTPHASE Position          */
#define TIMER_EXTCTL_CNTPHASE_Msk          (0x1ul << TIMER_EXTCTL_CNTPHASE_Pos)              /*!< TIMER_T::EXTCTL: CNTPHASE Mask              */

#define TIMER_EXTCTL_CAPEDGE_Pos           (1)                                               /*!< TIMER_T::EXTCTL: CAPEDGE Position           */
#define TIMER_EXTCTL_CAPEDGE_Msk           (0x3ul << TIMER_EXTCTL_CAPEDGE_Pos)               /*!< TIMER_T::EXTCTL: CAPEDGE Mask               */

#define TIMER_EXTCTL_CAPEN_Pos             (3)                                               /*!< TIMER_T::EXTCTL: CAPEN Position             */
#define TIMER_EXTCTL_CAPEN_Msk             (0x1ul << TIMER_EXTCTL_CAPEN_Pos)                 /*!< TIMER_T::EXTCTL: CAPEN Mask                 */

#define TIMER_EXTCTL_CAPFUNCS_Pos          (4)                                               /*!< TIMER_T::EXTCTL: CAPFUNCS Position          */
#define TIMER_EXTCTL_CAPFUNCS_Msk          (0x1ul << TIMER_EXTCTL_CAPFUNCS_Pos)              /*!< TIMER_T::EXTCTL: CAPFUNCS Mask              */

#define TIMER_EXTCTL_CAPIEN_Pos            (5)                                               /*!< TIMER_T::EXTCTL: CAPIEN Position            */
#define TIMER_EXTCTL_CAPIEN_Msk            (0x1ul << TIMER_EXTCTL_CAPIEN_Pos)                /*!< TIMER_T::EXTCTL: CAPIEN Mask                */

#define TIMER_EXTCTL_CAPDBEN_Pos           (6)                                               /*!< TIMER_T::EXTCTL: CAPDBEN Position           */
#define TIMER_EXTCTL_CAPDBEN_Msk           (0x1ul << TIMER_EXTCTL_CAPDBEN_Pos)               /*!< TIMER_T::EXTCTL: CAPDBEN Mask               */

#define TIMER_EXTCTL_ECNTDBEN_Pos          (7)                                               /*!< TIMER_T::EXTCTL: ECNTDBEN Position          */
#define TIMER_EXTCTL_ECNTDBEN_Msk          (0x1ul << TIMER_EXTCTL_ECNTDBEN_Pos)              /*!< TIMER_T::EXTCTL: ECNTDBEN Mask              */

#define TIMER_EXTCTL_CAPMODE_Pos           (8)                                               /*!< TIMER_T::EXTCTL: CAPMODE Position           */
#define TIMER_EXTCTL_CAPMODE_Msk           (0x1ul << TIMER_EXTCTL_CAPMODE_Pos)               /*!< TIMER_T::EXTCTL: CAPMODE Mask               */

#define TIMER_EINTSTS_CAPIF_Pos            (0)                                               /*!< TIMER_T::EINTSTS: CAPIF Position            */
#define TIMER_EINTSTS_CAPIF_Msk            (0x1ul << TIMER_EINTSTS_CAPIF_Pos)                /*!< TIMER_T::EINTSTS: CAPIF Mask                */

#define TIMER_CCAPCTL_CCAPEN_Pos           (0)                                               /*!< TIMER_AC_T::CCAPCTL: CCAPEN Position           */
#define TIMER_CCAPCTL_CCAPEN_Msk           (0x1ul << TIMER_CCAPCTL_CCAPEN_Pos)               /*!< TIMER_AC_T::CCAPCTL: CCAPEN Mask               */

#define TIMER_CCAPCTL_INV_Pos              (1)                                               /*!< TIMER_AC_T::CCAPCTL: INV Position              */
#define TIMER_CCAPCTL_INV_Msk              (0x1ul << TIMER_CCAPCTL_INV_Pos)                  /*!< TIMER_AC_T::CCAPCTL: INV Mask                  */

#define TIMER_CCAPCTL_TMRSEL_Pos           (2)                                               /*!< TIMER_AC_T::CCAPCTL: TMRSEL Position           */
#define TIMER_CCAPCTL_TMRSEL_Msk           (0x1ul << TIMER_CCAPCTL_TMRSEL_Pos)               /*!< TMR CCAPCTL: TMRSEL Mask               */

#define TIMER_CCAPCTL_CAPCHSEL_Pos         (3)                                               /*!< TIMER_AC_T::CCAPCTL: CAPCHSEL Position         */
#define TIMER_CCAPCTL_CAPCHSEL_Msk         (0x7ul << TIMER_CCAPCTL_CAPCHSEL_Pos)             /*!< TIMER_AC_T::CCAPCTL: CAPCHSEL Mask             */

#define TIMER_CCAPCTL_CAPR1F_Pos           (8)                                               /*!< TIMER_AC_T::CCAPCTL: CAPR1F Position           */
#define TIMER_CCAPCTL_CAPR1F_Msk           (0x1ul << TIMER_CCAPCTL_CAPR1F_Pos)               /*!< TIMER_AC_T::CCAPCTL: CAPR1F Mask               */

#define TIMER_CCAPCTL_CAPF1F_Pos           (9)                                               /*!< TIMER_AC_T::CCAPCTL: CAPF1F Position           */
#define TIMER_CCAPCTL_CAPF1F_Msk           (0x1ul << TIMER_CCAPCTL_CAPF1F_Pos)               /*!< TIMER_AC_T::CCAPCTL: CAPF1F Mask               */

#define TIMER_CCAPCTL_CAPR2F_Pos           (10)                                              /*!< TIMER_AC_T::CCAPCTL: CAPR2F Position           */
#define TIMER_CCAPCTL_CAPR2F_Msk           (0x1ul << TIMER_CCAPCTL_CAPR2F_Pos)               /*!< TIMER_AC_T::CCAPCTL: CAPR2F Mask               */

#define TIMER_CCAPCTL_CAPF2F_Pos           (11)                                              /*!< TIMER_AC_T::CCAPCTL: CAPF2F Position           */
#define TIMER_CCAPCTL_CAPF2F_Msk           (0x1ul << TIMER_CCAPCTL_CAPF2F_Pos)               /*!< TIMER_AC_T::CCAPCTL: CAPF2F Mask               */

#define TIMER_CCAP_CAPDAT_Pos              (0)                                               /*!< TIMER_AC_T::CCAP: CAPDAT Position             */
#define TIMER_CCAP_CAPDAT_Msk              (0xfffffful << TIMER_CCAP_CAPDAT_Pos)             /*!< TIMER_AC_T::CCAP: CAPDAT Mask                 */


/**@}*/ /* TIMER_CONST */
/**@}*/ /* end of TIMER register group */


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
     * Offset: 0x00  UART Receive/Transmit Buffer Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |DAT       |Receiving/Transmit Buffer
     * |        |          |Write Operation:
     * |        |          |By writing to this register, the UART sends out an 8-bit data through the TX pin (LSB first).
     * |        |          |By reading this register, the UART Controller will return an 8-bit data received from RX pin (LSB first).
    */
    __IO uint32_t DAT;

    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x04  UART Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RDAIEN    |Receive Data Available Interrupt Enable Control
     * |        |          |0 = RDAINT Masked off.
     * |        |          |1 = RDAINT Enabled.
     * |[1]     |THREIEN   |Transmit Holding Register Empty Interrupt Enable Control
     * |        |          |0 = THREINT Masked off.
     * |        |          |1 = THREINT Enabled.
     * |[2]     |RLSIEN    |Receive Line Status Interrupt Enable Control
     * |        |          |0 = RLSINT Masked off.
     * |        |          |1 = RLSINT Enabled.
     * |[3]     |MODEMIEN  |Modem Status Interrupt Enable Control
     * |        |          |0 = MODEMINT Masked off.
     * |        |          |1 = MODEMINT Enabled.
     * |[4]     |RXTOIEN   |RX Time-Out Interrupt Enable Control
     * |        |          |0 = RXTOINT Masked off.
     * |        |          |1 = RXTOINT Enabled.
     * |[5]     |BUFERRIEN |Buffer Error Interrupt Enable Control
     * |        |          |0 = BUFERRINT Masked Disabled.
     * |        |          |1 = BUFERRINT Enabled.
     * |[6]     |WKCTSIEN  |Wake-Up CPU Function Enable Control
     * |        |          |0 = UART wake-up function Disabled.
     * |        |          |1 = UART Wake-up function Enabled.
     * |        |          |Note: when the chip is in Power-down mode, an external CTS change will wake-up chip from Power-down mode.
     * |[11]    |TOCNTEN   |Time-Out Counter Enable Control
     * |        |          |0 = Time-out counter Disabled.
     * |        |          |1 = Time-out counter Enabled.
     * |[12]    |ATORTSEN  |RTS Auto Flow Control Enable Control
     * |        |          |0 = RTS auto flow control Disabled.
     * |        |          |1 = RTS auto flow control Enabled.
     * |        |          |Note: When RTS auto-flow is enabled, if the number of bytes in the RX FIFO equals the RTSTRGLV (UART_FIFO [19:16]), the UART will de-assert RTS signal.
     * |[13]    |ATOCTSEN  |CTS Auto Flow Control Enable Control
     * |        |          |0 = CTS auto flow control Disabled.
     * |        |          |1 = CTS auto flow control Enabled.
     * |        |          |Note: When CTS auto-flow is enabled, the UART will send data to external device when CTS input assert (UART will not send data to device until CTS is asserted).
    */
    __IO uint32_t INTEN;

    /**
     * FIFO
     * ===================================================================================================
     * Offset: 0x08  UART FIFO Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RXRST     |RX Field Software Reset
     * |        |          |When RX_RST is set, all the byte in the receiver FIFO and RX internal state machine are cleared.
     * |        |          |0 = No effect.
     * |        |          |1 = The RX internal state machine and pointers reset.
     * |        |          |Note: This bit will auto clear needs at least 3 UART Controller peripheral clock cycles.
     * |[2]     |TXRST     |TX Field Software Reset
     * |        |          |When TX_RST is set, all the byte in the transmit FIFO and TX internal state machine are cleared.
     * |        |          |0 = No effect.
     * |        |          |1 = The TX internal state machine and pointers reset.
     * |        |          |Note: This bit will auto clear needs at least 3 UART Controller peripheral clock cycles.
     * |[7:4]   |RFITL     |RX FIFO Interrupt (RDAINT) Trigger Level
     * |        |          |When the number of bytes in the receive FIFO equals the RFITL then the RDAIF will be set (if RDAIEN in UART_INTEN register is enable, an interrupt will generated).
     * |        |          |0000 = RX FIFO Interrupt Trigger Level is 1 byte.
     * |        |          |0001 = RX FIFO Interrupt Trigger Level is 4 bytes.
     * |        |          |0010 = RX FIFO Interrupt Trigger Level is 8 bytes.
     * |        |          |0011 = RX FIFO Interrupt Trigger Level is 14 bytes.
     * |[8]     |RXOFF     |Receiver Disable Register
     * |        |          |The receiver is disabled or not (setting 1 to disable the receiver).
     * |        |          |0 = Receiver Enabled.
     * |        |          |1 = Receiver Disabled.
     * |        |          |Note: This field is used for RS-485 Normal Multi-drop mode.
     * |        |          |It should be programmed before RS-485_NMM (UART_ALTCTL [8]) is programmed.
     * |[19:16] |RTSTRGLV  |RTS Trigger Level (for Auto-flow Control Use)
     * |        |          |0000 = RTS Trigger Level is 1 byte.
     * |        |          |0001 = RTS Trigger Level is 4 bytes.
     * |        |          |0010 = RTS Trigger Level is 8 bytes.
     * |        |          |0011 = RTS Trigger Level is 14 bytes.
     * |        |          |Others = Reserved
     * |        |          |Note: This field is used for RTS auto-flow control.
    */
    __IO uint32_t FIFO;

    /**
     * LINE
     * ===================================================================================================
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
     * |        |          |When select 6-, 7- and 8-bti word length, 2 "STOP bit" is generated in the transmitted data.
     * |[3]     |PBE       |Parity Bit Enable Control
     * |        |          |0 = No parity bit.
     * |        |          |1 = Parity bit is generated on each outgoing character and is checked on each incoming data.
     * |[4]     |EPE       |Even Parity Enable Control
     * |        |          |0 = Odd number of logic 1's is transmitted and checked in each word.
     * |        |          |1 = Even number of logic 1's is transmitted and checked in each word.
     * |        |          |This bit has effect only when PBE (UART_LINE[3]) is set.
     * |[5]     |SPE       |Stick Parity Enable Control
     * |        |          |0 = Stick parity Disabled.
     * |        |          |1 = If PBE (UART_LINE[3]) and EBE (UART_LINE[4]) are logic 1, the parity bit is transmitted and checked as logic 0.
     * |        |          |If PBE (UART_LINE[3]) is 1 and EBE (UART_LINE[4]) is 0 then the parity bit is transmitted and checked as 1.
     * |[6]     |BCB       |Break Control Bit
     * |        |          |When this bit is set to logic 1, the serial data output (TX) is forced to the Spacing State (logic 0).
     * |        |          |This bit acts only on TX and has no effect on the transmitter logic.
     * |        |          |0 = Break control Disabled.
     * |        |          |1 = Break control Enabled.
    */
    __IO uint32_t LINE;

    /**
     * MODEM
     * ===================================================================================================
     * Offset: 0x10  UART Modem Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |RTS       |RTS (Request-To-Send) Signal Control
     * |        |          |This bit is direct control internal RTS signal active or not, and then drive the RTS pin output with RTSACTLV bit configuration.
     * |        |          |0 = RTS signal is active.
     * |        |          |1 = RTS signal is inactive.
     * |        |          |Note1: This RTS signal control bit is not effective when RTS auto-flow control is enabled in UART function mode.
     * |        |          |Note2: This RTS signal control bit is not effective when RS-485 auto direction mode (AUD) is enabled in RS-485 function mode.
     * |[9]     |RTSACTLV  |RTS Pin Active Level
     * |        |          |This bit defines the active level state of RTS pin output.
     * |        |          |0 = RTS pin output is high level active.
     * |        |          |1 = RTS pin output is low level active.
     * |[13]    |RTSSTS    |RTS Pin State (Read Only)
     * |        |          |This bit mirror from RTS pin output of voltage logic status.
     * |        |          |0 = RTS pin output is low level voltage logic state.
     * |        |          |1 = RTS pin output is high level voltage logic state.
    */
    __IO uint32_t MODEM;

    /**
     * MODEMSTS
     * ===================================================================================================
     * Offset: 0x14  UART Modem Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CTSDETF   |Detect CTS State Change Flag
     * |        |          |This bit is set whenever CTS input has change state, and it will generate Modem interrupt to CPU when MODEMIEN (UART_INTEN [3]) is set to 1.
     * |        |          |0 = CTS input has not change state.
     * |        |          |1 = CTS input has change state.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[4]     |CTSSTS    |CTS Pin Status (Read Only)
     * |        |          |This bit mirror from CTS pin input of voltage logic status.
     * |        |          |0 = CTS pin input is low level voltage logic state.
     * |        |          |1 = CTS pin input is high level voltage logic state.
     * |        |          |Note: This bit echoes when UART Controller peripheral clock is enabled, and CTS multi-function port is selected.
     * |[8]     |CTSACTLV  |CTS Pin Active Level
     * |        |          |This bit defines the active level state of CTS pin input.
     * |        |          |0 = CTS pin input is high level active.
     * |        |          |1 = CTS pin input is low level active.
     * |        |          |Note: Refer to Figure 6.14-10
    */
    __IO uint32_t MODEMSTS;

    /**
     * FIFOSTS
     * ===================================================================================================
     * Offset: 0x18  UART FIFO Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXOVIF    |RX Overflow Error Interrupt Flag
     * |        |          |This bit is set when RX FIFO overflow.
     * |        |          |If the number of bytes of received data is greater than RX_FIFO (UART_DAT) size, 16 bytes this bit will be set.
     * |        |          |0 = RX FIFO is not overflow.
     * |        |          |1 = RX FIFO is overflow.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[3]     |ADDRDETF  |RS-485 Address Byte Detection Flag
     * |        |          |This bit is set to 1 while ADDRDEN (UART_ALTCTL[15]) is set to 1 to enable Address detection mode and receive detect a data with an address bit (bit 9 = 1).
     * |        |          |Note1: This field is used for RS-485 function mode.
     * |        |          |Note2: This bit is cleared by writing 1 to it.
     * |[4]     |PEF       |Parity Error Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "parity bit".
     * |        |          |0 = No parity error is generated.
     * |        |          |1 = Parity error is generated.Note: This bit is read only, but can be cleared by writing '1' to it .
     * |[5]     |FEF       |Framing Error Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "stop bit" (that is, the stop bit follows the last data bit or parity bit is detected as a logic 0), 0 = No framing error is generated.
     * |        |          |1 = Framing error is generated.
     * |        |          |Note: This bit is read only, but can be cleared by writing '1' to it .
     * |[6]     |BIF       |Break Interrupt Flag (Read Only)
     * |        |          |This bit is set to logic 1 whenever the received data input (RX) is held in the "spacing state" (logic 0) for longer than a full word transmission time (that is, the total time of "start bit" + data bits + parity + stop bits).
     * |        |          |0 = No Break interrupt is generated.
     * |        |          |1 = Break interrupt is generated.
     * |        |          |Note: This bit is read only, but software can write 1 to clear it.
     * |[13:8]  |RXPTR     |RX FIFO Pointer (Read Only)
     * |        |          |This field indicates the RX FIFO Buffer Pointer.
     * |        |          |When UART receives one byte from external device, RXPTR increases one.
     * |        |          |When one byte of RX FIFO is read by CPU, RXPTR decreases one.
     * |        |          |The Maximum value shown in RXPTR is 15.
     * |        |          |When the using level of RX FIFO Buffer equal to 16, the RXFULL bit is set to 1 and RXPTR will show 0.
     * |        |          |As one byte of RX FIFO is read by CPU, the RXFULL bit is cleared to 0 and RXPTR will show 15.
     * |[14]    |RXEMPTY   |Receiver FIFO Empty (Read Only)
     * |        |          |This bit initiate RX FIFO empty or not.
     * |        |          |0 = RX FIFO is not empty.
     * |        |          |1 = RX FIFO is empty.
     * |        |          |Note: When the last byte of RX FIFO has been read by CPU, hardware sets this bit high.
     * |        |          |It will be cleared when UART receives any new data.
     * |[15]    |RXFULL    |Receiver FIFO Full (Read Only)
     * |        |          |This bit initiates RX FIFO full or not.
     * |        |          |0 = RX FIFO is not full.
     * |        |          |1 = RX FIFO is full.
     * |        |          |Note: This bit is set when the number of usage in RX FIFO Buffer is equal to 16, otherwise is cleared by hardware.
     * |[21:16] |TXPTR     |TX FIFO Pointer (Read Only)
     * |        |          |This field indicates the TX FIFO Buffer Pointer.
     * |        |          |When CPU writes one byte into UART_DAT, TXPTR increases one.
     * |        |          |When one byte of TX FIFO is transferred to Transmitter Shift Register, TXPTR decreases one.
     * |        |          |The Maximum value shown in TXPTR is 15.
     * |        |          |When the using level of TX FIFO Buffer equal to 16, the TXFULL bit is set to 1 and TXPTR will show 0.
     * |        |          |As one byte of TX FIFO is transferred to Transmitter Shift Register, the TXFULL bit is cleared to 0 and TXPTR will show 15.
     * |[22]    |TXEMPTY   |Transmitter FIFO Empty (Read Only)
     * |        |          |This bit indicates TX FIFO empty or not.
     * |        |          |0 = TX FIFO is not empty.
     * |        |          |1 = TX FIFO is empty.
     * |        |          |Note: When the last byte of TX FIFO has been transferred to Transmitter Shift Register, hardware sets this bit high.
     * |        |          |It will be cleared when writing data into DAT (TX FIFO not empty).
     * |[23]    |TXFULL    |Transmitter FIFO Full (Read Only)
     * |        |          |This bit indicates TX FIFO full or not.
     * |        |          |0 = TX FIFO is not full.
     * |        |          |1 = TX FIFO is full.
     * |        |          |Note: This bit is set when the number of usage in TX FIFO Buffer is equal to 16, otherwise is cleared by hardware.
     * |[24]    |TXOVIF    |TX Overflow Error Interrupt Flag
     * |        |          |If TX FIFO (UART_DAT) is full, an additional write to UART_DAT will cause this bit to  logic 1.
     * |        |          |0 = TX FIFO is not overflow.
     * |        |          |1 = TX FIFO is overflow.
     * |        |          |Note: This bit is cleared by writing 1 to it.
     * |[28]    |TXEMPTYF  |Transmitter Empty Flag (Read Only)
     * |        |          |This bit is set by hardware when TX FIFO (UART_DAT) is empty and the STOP bit of the last byte has been transmitted.
     * |        |          |0 = TX FIFO is not empty.
     * |        |          |1 = TX FIFO is empty.
     * |        |          |Note: This bit is cleared automatically when TX FIFO is not empty or the last byte transmission has not completed.
    */
    __IO uint32_t FIFOSTS;

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x1C  UART Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RDAIF     |Receive Data Available Interrupt Flag (Read Only)
     * |        |          |When the number of bytes in the RX FIFO equals the RFITL then the RDAIF will be set.
     * |        |          |If RDAIEN (UART_INTEN [0]) is enabled, the RDA interrupt will be generated.
     * |        |          |0 = No RDA interrupt flag is generated.
     * |        |          |1 = RDA interrupt flag is generated.
     * |        |          |Note: This bit is read only and it will be cleared when the number of unread bytes of RX FIFO drops below the threshold level (RFITL).
     * |[1]     |THREIF    |Transmit Holding Register Empty Interrupt Flag (Read Only)
     * |        |          |This bit is set when the last data of TX FIFO is transferred to Transmitter Shift Register.
     * |        |          |If THREIEN (UART_INTEN [1]) is enabled, the THRE interrupt will be generated.
     * |        |          |0 = No THRE interrupt flag is generated.
     * |        |          |1 = THRE interrupt flag is generated.
     * |        |          |Note: This bit is read only and it will be cleared when writing data into DAT (TX FIFO not empty).
     * |[2]     |RLSIF     |Receive Line Interrupt Flag (Read Only)
     * |        |          |This bit is set when the RX receive data have parity error, framing error or break error (at least one of 3 bits, BIF, FEF and PEF, is set).
     * |        |          |If RLSIEN (UART_INTEN [2]) is enabled, the RLS interrupt will be generated.
     * |        |          |0 = No RLS interrupt flag is generated.
     * |        |          |1 = RLS interrupt flag is generated.
     * |        |          |Note1: In RS-485 function mode, this field is set including "receiver detect and received address byte character (bit 9 = 1) bit&quot.
     * |        |          |At the same time, the bit of ADDRDETF (UART_FIFOSTS[3]) is also set.
     * |        |          |Note2: This bit is read only and reset to 0 when all bits of BIF, FEF, PEF and ADDRDETF are cleared.
     * |[3]     |MODENIF   |MODEM Interrupt Flag (Read Only)
     * |        |          |This bit is set when the CTS pin has state change (CTSDETF = 1).
     * |        |          |If UART_INTEN [MODEMIEN] is enabled, the Modem interrupt will be generated.
     * |        |          |0 = No Modem interrupt flag is generated.
     * |        |          |1 = Modem interrupt flag is generated.
     * |        |          |Note: This bit is read only and reset to 0 when bit CTSDETF is cleared by a write 1 on CTSDETF.
     * |[4]     |RXTOIF    |Time-Out Interrupt Flag (Read Only)
     * |        |          |This bit is set when the RX FIFO is not empty and no activities occurred in the RX FIFO and the time-out counter equal to TOIC.
     * |        |          |If RXTOIEN (UART_INTEN [4]) is enabled, the Tout interrupt will be generated.
     * |        |          |0 = No Time-out interrupt flag is generated.
     * |        |          |1 = Time-out interrupt flag is generated.
     * |        |          |Note: This bit is read only and user can read UART_DAT (RX is in active) to clear it.
     * |[5]     |BUFERRIF  |Buffer Error Interrupt Flag (Read Only)
     * |        |          |This bit is set when the TX/RX FIFO overflow flag (TXOVIF or RXOVIF) is set.
     * |        |          |When BUFERRIF is set, the transfer is not correct.
     * |        |          |If BUFERRIEN (UART_INTEN [5]) is enabled, the buffer error interrupt will be generated.
     * |        |          |0 = No buffer error interrupt flag is generated.
     * |        |          |1 = Buffer error interrupt flag is generated.
     * |        |          |Note: This bit is read only and reset to 0 when all bits of TXOVIF and RXOVIF are cleared.
     * |[8]     |RDAINT    |Receive Data Available Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RDAIEN and RDAIF are both set to 1.
     * |        |          |This bit is set if RDAIEN and RDAIF are both set to 1.
     * |        |          |0 = No RDA interrupt is generated.
     * |        |          |1 = RDA interrupt is generated.
     * |[9]     |THREINT   |Transmit Holding Register Empty Interrupt Indicator (Read Only)
     * |        |          |This bit is set if THREIEN and THREIF are both set to 1.
     * |        |          |0 = No THRE interrupt is generated.
     * |        |          |1 = THRE interrupt is generated.
     * |[10]    |RLSINT    |Receive Line Status Interrupt (Read Only)
     * |        |          |This bit is set if RLSIEN and RLSIF are both set to 1.
     * |        |          |0 = No RLS interrupt is generated.
     * |        |          |1 = RLS interrupt is generated.
     * |[11]    |MODEMINT  |MODEM Status Interrupt Indicator (Read Only)
     * |        |          |This bit is set if MODEMIEN and MODENIF are both set to 1.
     * |        |          |0 = No Modem interrupt is generated.
     * |        |          |1 = Modem interrupt is generated.
     * |[12]    |RXTOINT   |Time-Out Interrupt Indicator (Read Only)
     * |        |          |This bit is set if RXTOIEN and RXTOIF are both set to 1.
     * |        |          |0 = No Time-out interrupt is generated.
     * |        |          |1 = Time-out interrupt is generated.
     * |[13]    |BUFERRINT |Buffer Error Interrupt Indicator (Read Only)
     * |        |          |This bit is set if BUFERRIEN and BUFERRIF are both set to 1.
     * |        |          |0 = No buffer error interrupt is generated.
     * |        |          |1 = buffer error interrupt is generated.
    */
    __IO  uint32_t INTSTS;

    /**
     * TOUT
     * ===================================================================================================
     * Offset: 0x20  UART Time-out Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[7:0]   |TOIC      |Time-Out Interrupt Comparator
     * |        |          |The time-out counter resets and starts counting (the counting clock = baud rate) whenever the RX FIFO receives a new data word.
     * |        |          |Once the content of time-out counter (TOUT_CNT) is equal to that of time-out interrupt comparator (TOIC), a receiver time-out interrupt (RXTOINT) is generated if RXTOIEN (UART_INTEN [4]).
     * |        |          |A new incoming data word or RX FIFO empty clears RXTOINT.
     * |        |          |In order to avoid receiver time-out interrupt generation immediately during one character is being received, TOIC value should be set between 40 and 255.
     * |        |          |So, for example, if TOIC is set with 40, the time-out interrupt is generated after four characters are not received when 1 stop bit and no parity check is set for UART transfer.
     * |[15:8]  |DLY       |TX Delay Time Value
     * |        |          |This field is used to program the transfer delay time between the last stop bit and next start bit.
    */
    __IO uint32_t TOUT;

    /**
     * BAUD
     * ===================================================================================================
     * Offset: 0x24  UART Baud Rate Divisor Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[15:0]  |BRD       |Baud Rate Divider
     * |        |          |The field indicates the baud rate divider.
     * |[27:24] |EDIVM1    |Divider X
     * |        |          |The baud rate divider M = X+1.
     * |[28]    |BAUDM0    |Divider X Equal 1
     * |        |          |0 = Divider M = X (the equation of M = X+1, but EDIVM1[27:24] must >= 8).
     * |        |          |1 = Divider M = 1 (the equation of M = 1, but BRD [15:0] must >= 8).
     * |        |          |UART Controller Baud Rate Generator Refer to section "UART Controller Baud Rate Generator" for more information.
     * |[29]    |BAUDM1    |Divider X Enable
     * |        |          |The BRD = Baud Rate Divider, and the baud rate equation is:  Baud Rate = Clock / [M * (BRD + 2)], The default value of M is 16.
     * |        |          |0 = Divider X Disabled (the equation of M = 16).
     * |        |          |1 = Divider X Enabled (the equation of M = X+1, but EDIVM1 [27:24] must >= 8).
     * |        |          |Refer to the table below for more information.
     * |        |          |Note: When in IrDA mode, this bit must be disabled.
    */
    __IO uint32_t BAUD;

    /**
     * IRDA
     * ===================================================================================================
     * Offset: 0x28  UART IrDA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |TXEN      |TXEN
     * |        |          |0 = IrDA receiver Enabled.
     * |        |          |1 = IrDA transmitter Enabled.
     * |[5]     |TXINV     |TXINV
     * |        |          |0 = No inversion.
     * |        |          |1 = Inverse TX output signal.
     * |[6]     |RXINV     |RXINV
     * |        |          |0 = No inversion.
     * |        |          |1 = Inverse RX input signal.
    */
    __IO uint32_t IRDA;

    /**
     * ALTCTL
     * ===================================================================================================
     * Offset: 0x2C  UART Alternate Control/Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[8]     |RS485NMM  |RS-485 Normal Multi-Drop Operation Mode (NMM) Control
     * |        |          |0 = RS-485 Normal Multi-drop Operation Mode (NMM) Disabled.
     * |        |          |1 = RS-485 Normal Multi-drop Operation Mode (NMM) Enabled.
     * |        |          |Note: It cannot be active with RS485AAD operation mode.
     * |[9]     |RS485AAD  |RS-485 Auto Address Detection Operation Mode (AAD)
     * |        |          |0 = RS-485 Auto Address Detection Operation Mode (AAD) Disabled.
     * |        |          |1 = RS-485 Auto Address Detection Operation Mode (AAD) Enabled.
     * |        |          |Note: It cannot be active with RS485NMM operation mode.
     * |[10]    |RS485AUD  |RS-485 Auto Direction Mode (AUD) Control
     * |        |          |0 = RS-485 Auto Direction Mode (AUD) Disabled.
     * |        |          |1 = RS-485 Auto Direction Mode (AUD) Enabled.
     * |        |          |Note: It can be active with RS485ADD or RS485NMM operation mode.
     * |[15]    |ADDRDEN   |RS-485 Address Detection Enable Control
     * |        |          |This bit is used to enable RS-485 Address Detection mode.
     * |        |          |0 = RS-485 address detection mode Disabled.
     * |        |          |1 = RS-485 address detection mode Enabled.
     * |        |          |Note: This field is used for RS-485 any operation mode.
     * |[31:24] |ADDRMV    |Address Match Value Register
     * |        |          |This field contains the RS-485 address match values.
     * |        |          |Note: This field is used for RS-485 auto address detection mode.
    */
    __IO uint32_t ALTCTL;

    /**
     * FUNCSEL
     * ===================================================================================================
     * Offset: 0x30  UART Function Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1:0]   |FUNCSEL   |Function Select
     * |        |          |00 = UART function mode.
     * |        |          |01 = Reserved.
     * |        |          |10 = IrDA function mode.
     * |        |          |11 = RS-485 function mode.
    */
    __IO uint32_t FUNCSEL;

} UART_T;

/**
    @addtogroup UART_CONST UART Bit Field Definition
    Constant Definitions for UART Controller
@{ */

#define UART_DAT_DAT_Pos                 (0)                                               /*!< UART_T::DAT: DAT Position                 */
#define UART_DAT_DAT_Msk                 (0xfful << UART_DAT_DAT_Pos)                      /*!< UART_T::DAT: DAT Mask                     */

#define UART_INTEN_RDAIEN_Pos            (0)                                               /*!< UART_T::INTEN: RDAIEN Position            */
#define UART_INTEN_RDAIEN_Msk            (0x1ul << UART_INTEN_RDAIEN_Pos)                  /*!< UART_T::INTEN: RDAIEN Mask                */

#define UART_INTEN_THREIEN_Pos           (1)                                               /*!< UART_T::INTEN: THREIEN Position           */
#define UART_INTEN_THREIEN_Msk           (0x1ul << UART_INTEN_THREIEN_Pos)                 /*!< UART_T::INTEN: THREIEN Mask               */

#define UART_INTEN_RLSIEN_Pos            (2)                                               /*!< UART_T::INTEN: RLSIEN Position            */
#define UART_INTEN_RLSIEN_Msk            (0x1ul << UART_INTEN_RLSIEN_Pos)                  /*!< UART_T::INTEN: RLSIEN Mask                */

#define UART_INTEN_MODEMIEN_Pos          (3)                                               /*!< UART_T::INTEN: MODEMIEN Position          */
#define UART_INTEN_MODEMIEN_Msk          (0x1ul << UART_INTEN_MODEMIEN_Pos)                /*!< UART_T::INTEN: MODEMIEN Mask              */

#define UART_INTEN_RXTOIEN_Pos           (4)                                               /*!< UART_T::INTEN: RXTOIEN Position           */
#define UART_INTEN_RXTOIEN_Msk           (0x1ul << UART_INTEN_RXTOIEN_Pos)                 /*!< UART_T::INTEN: RXTOIEN Mask               */

#define UART_INTEN_BUFERRIEN_Pos         (5)                                               /*!< UART_T::INTEN: BUFERRIEN Position         */
#define UART_INTEN_BUFERRIEN_Msk         (0x1ul << UART_INTEN_BUFERRIEN_Pos)               /*!< UART_T::INTEN: BUFERRIEN Mask             */

#define UART_INTEN_WKCTSIEN_Pos          (6)                                               /*!< UART_T::INTEN: WKCTSIEN Position          */
#define UART_INTEN_WKCTSIEN_Msk          (0x1ul << UART_INTEN_WKCTSIEN_Pos)                /*!< UART_T::INTEN: WKCTSIEN Mask              */

#define UART_INTEN_TOCNTEN_Pos           (11)                                              /*!< UART_T::INTEN: TOCNTEN Position           */
#define UART_INTEN_TOCNTEN_Msk           (0x1ul << UART_INTEN_TOCNTEN_Pos)                 /*!< UART_T::INTEN: TOCNTEN Mask               */

#define UART_INTEN_ATORTSEN_Pos          (12)                                              /*!< UART_T::INTEN: ATORTSEN Position          */
#define UART_INTEN_ATORTSEN_Msk          (0x1ul << UART_INTEN_ATORTSEN_Pos)                /*!< UART_T::INTEN: ATORTSEN Mask              */

#define UART_INTEN_ATOCTSEN_Pos          (13)                                              /*!< UART_T::INTEN: ATOCTSEN Position          */
#define UART_INTEN_ATOCTSEN_Msk          (0x1ul << UART_INTEN_ATOCTSEN_Pos)                /*!< UART_T::INTEN: ATOCTSEN Mask              */

#define UART_FIFO_RXRST_Pos              (1)                                               /*!< UART_T::FIFO: RXRST Position              */
#define UART_FIFO_RXRST_Msk              (0x1ul << UART_FIFO_RXRST_Pos)                    /*!< UART_T::FIFO: RXRST Mask                  */

#define UART_FIFO_TXRST_Pos              (2)                                               /*!< UART_T::FIFO: TXRST Position              */
#define UART_FIFO_TXRST_Msk              (0x1ul << UART_FIFO_TXRST_Pos)                    /*!< UART_T::FIFO: TXRST Mask                  */

#define UART_FIFO_RFITL_Pos              (4)                                               /*!< UART_T::FIFO: RFITL Position              */
#define UART_FIFO_RFITL_Msk              (0xful << UART_FIFO_RFITL_Pos)                    /*!< UART_T::FIFO: RFITL Mask                  */

#define UART_FIFO_RXOFF_Pos              (8)                                               /*!< UART_T::FIFO: RXOFF Position              */
#define UART_FIFO_RXOFF_Msk              (0x1ul << UART_FIFO_RXOFF_Pos)                    /*!< UART_T::FIFO: RXOFF Mask                  */

#define UART_FIFO_RTSTRGLV_Pos           (16)                                              /*!< UART_T::FIFO: RTSTRGLV Position           */
#define UART_FIFO_RTSTRGLV_Msk           (0xful << UART_FIFO_RTSTRGLV_Pos)                 /*!< UART_T::FIFO: RTSTRGLV Mask               */

#define UART_LINE_WLS_Pos                (0)                                               /*!< UART_T::LINE: WLS Position                */
#define UART_LINE_WLS_Msk                (0x3ul << UART_LINE_WLS_Pos)                      /*!< UART_T::LINE: WLS Mask                    */

#define UART_LINE_NSB_Pos                (2)                                               /*!< UART_T::LINE: NSB Position                */
#define UART_LINE_NSB_Msk                (0x1ul << UART_LINE_NSB_Pos)                      /*!< UART_T::LINE: NSB Mask                    */

#define UART_LINE_PBE_Pos                (3)                                               /*!< UART_T::LINE: PBE Position                */
#define UART_LINE_PBE_Msk                (0x1ul << UART_LINE_PBE_Pos)                      /*!< UART_T::LINE: PBE Mask                    */

#define UART_LINE_EPE_Pos                (4)                                               /*!< UART_T::LINE: EPE Position                */
#define UART_LINE_EPE_Msk                (0x1ul << UART_LINE_EPE_Pos)                      /*!< UART_T::LINE: EPE Mask                    */

#define UART_LINE_SPE_Pos                (5)                                               /*!< UART_T::LINE: SPE Position                */
#define UART_LINE_SPE_Msk                (0x1ul << UART_LINE_SPE_Pos)                      /*!< UART_T::LINE: SPE Mask                    */

#define UART_LINE_BCB_Pos                (6)                                               /*!< UART_T::LINE: BCB Position                */
#define UART_LINE_BCB_Msk                (0x1ul << UART_LINE_BCB_Pos)                      /*!< UART_T::LINE: BCB Mask                    */

#define UART_MODEM_RTS_Pos               (1)                                               /*!< UART_T::MODEM: RTS Position               */
#define UART_MODEM_RTS_Msk               (0x1ul << UART_MODEM_RTS_Pos)                     /*!< UART_T::MODEM: RTS Mask                   */

#define UART_MODEM_RTSACTLV_Pos          (9)                                               /*!< UART_T::MODEM: RTSACTLV Position          */
#define UART_MODEM_RTSACTLV_Msk          (0x1ul << UART_MODEM_RTSACTLV_Pos)                /*!< UART_T::MODEM: RTSACTLV Mask              */

#define UART_MODEM_RTSSTS_Pos            (13)                                              /*!< UART_T::MODEM: RTSSTS Position            */
#define UART_MODEM_RTSSTS_Msk            (0x1ul << UART_MODEM_RTSSTS_Pos)                  /*!< UART_T::MODEM: RTSSTS Mask                */

#define UART_MODEMSTS_CTSDETF_Pos        (0)                                               /*!< UART_T::MODEMSTS: CTSDETF Position        */
#define UART_MODEMSTS_CTSDETF_Msk        (0x1ul << UART_MODEMSTS_CTSDETF_Pos)              /*!< UART_T::MODEMSTS: CTSDETF Mask            */

#define UART_MODEMSTS_CTSSTS_Pos         (4)                                               /*!< UART_T::MODEMSTS: CTSSTS Position         */
#define UART_MODEMSTS_CTSSTS_Msk         (0x1ul << UART_MODEMSTS_CTSSTS_Pos)               /*!< UART_T::MODEMSTS: CTSSTS Mask             */

#define UART_MODEMSTS_CTSACTLV_Pos       (8)                                               /*!< UART_T::MODEMSTS: CTSACTLV Position       */
#define UART_MODEMSTS_CTSACTLV_Msk       (0x1ul << UART_MODEMSTS_CTSACTLV_Pos)             /*!< UART_T::MODEMSTS: CTSACTLV Mask           */

#define UART_FIFOSTS_RXOVIF_Pos          (0)                                               /*!< UART_T::FIFOSTS: RXOVIF Position          */
#define UART_FIFOSTS_RXOVIF_Msk          (0x1ul << UART_FIFOSTS_RXOVIF_Pos)                /*!< UART_T::FIFOSTS: RXOVIF Mask              */

#define UART_FIFOSTS_ADDRDETF_Pos        (3)                                               /*!< UART_T::FIFOSTS: ADDRDETF Position        */
#define UART_FIFOSTS_ADDRDETF_Msk        (0x1ul << UART_FIFOSTS_ADDRDETF_Pos)              /*!< UART_T::FIFOSTS: ADDRDETF Mask            */

#define UART_FIFOSTS_PEF_Pos             (4)                                               /*!< UART_T::FIFOSTS: PEF Position             */
#define UART_FIFOSTS_PEF_Msk             (0x1ul << UART_FIFOSTS_PEF_Pos)                   /*!< UART_T::FIFOSTS: PEF Mask                 */

#define UART_FIFOSTS_FEF_Pos             (5)                                               /*!< UART_T::FIFOSTS: FEF Position             */
#define UART_FIFOSTS_FEF_Msk             (0x1ul << UART_FIFOSTS_FEF_Pos)                   /*!< UART_T::FIFOSTS: FEF Mask                 */

#define UART_FIFOSTS_BIF_Pos             (6)                                               /*!< UART_T::FIFOSTS: BIF Position             */
#define UART_FIFOSTS_BIF_Msk             (0x1ul << UART_FIFOSTS_BIF_Pos)                   /*!< UART_T::FIFOSTS: BIF Mask                 */

#define UART_FIFOSTS_RXPTR_Pos           (8)                                               /*!< UART_T::FIFOSTS: RXPTR Position           */
#define UART_FIFOSTS_RXPTR_Msk           (0x3ful << UART_FIFOSTS_RXPTR_Pos)                /*!< UART_T::FIFOSTS: RXPTR Mask               */

#define UART_FIFOSTS_RXEMPTY_Pos         (14)                                              /*!< UART_T::FIFOSTS: RXEMPTY Position         */
#define UART_FIFOSTS_RXEMPTY_Msk         (0x1ul << UART_FIFOSTS_RXEMPTY_Pos)               /*!< UART_T::FIFOSTS: RXEMPTY Mask             */

#define UART_FIFOSTS_RXFULL_Pos          (15)                                              /*!< UART_T::FIFOSTS: RXFULL Position          */
#define UART_FIFOSTS_RXFULL_Msk          (0x1ul << UART_FIFOSTS_RXFULL_Pos)                /*!< UART_T::FIFOSTS: RXFULL Mask              */

#define UART_FIFOSTS_TXPTR_Pos           (16)                                              /*!< UART_T::FIFOSTS: TXPTR Position           */
#define UART_FIFOSTS_TXPTR_Msk           (0x3ful << UART_FIFOSTS_TXPTR_Pos)                /*!< UART_T::FIFOSTS: TXPTR Mask               */

#define UART_FIFOSTS_TXEMPTY_Pos         (22)                                              /*!< UART_T::FIFOSTS: TXEMPTY Position         */
#define UART_FIFOSTS_TXEMPTY_Msk         (0x1ul << UART_FIFOSTS_TXEMPTY_Pos)               /*!< UART_T::FIFOSTS: TXEMPTY Mask             */

#define UART_FIFOSTS_TXFULL_Pos          (23)                                              /*!< UART_T::FIFOSTS: TXFULL Position          */
#define UART_FIFOSTS_TXFULL_Msk          (0x1ul << UART_FIFOSTS_TXFULL_Pos)                /*!< UART_T::FIFOSTS: TXFULL Mask              */

#define UART_FIFOSTS_TXOVIF_Pos          (24)                                              /*!< UART_T::FIFOSTS: TXOVIF Position          */
#define UART_FIFOSTS_TXOVIF_Msk          (0x1ul << UART_FIFOSTS_TXOVIF_Pos)                /*!< UART_T::FIFOSTS: TXOVIF Mask              */

#define UART_FIFOSTS_TXEMPTYF_Pos        (28)                                              /*!< UART_T::FIFOSTS: TXEMPTYF Position        */
#define UART_FIFOSTS_TXEMPTYF_Msk        (0x1ul << UART_FIFOSTS_TXEMPTYF_Pos)              /*!< UART_T::FIFOSTS: TXEMPTYF Mask            */

#define UART_INTSTS_RDAIF_Pos            (0)                                               /*!< UART_T::INTSTS: RDAIF Position            */
#define UART_INTSTS_RDAIF_Msk            (0x1ul << UART_INTSTS_RDAIF_Pos)                  /*!< UART_T::INTSTS: RDAIF Mask                */

#define UART_INTSTS_THREIF_Pos           (1)                                               /*!< UART_T::INTSTS: THREIF Position           */
#define UART_INTSTS_THREIF_Msk           (0x1ul << UART_INTSTS_THREIF_Pos)                 /*!< UART_T::INTSTS: THREIF Mask               */

#define UART_INTSTS_RLSIF_Pos            (2)                                               /*!< UART_T::INTSTS: RLSIF Position            */
#define UART_INTSTS_RLSIF_Msk            (0x1ul << UART_INTSTS_RLSIF_Pos)                  /*!< UART_T::INTSTS: RLSIF Mask                */

#define UART_INTSTS_MODEMIF_Pos          (3)                                               /*!< UART_T::INTSTS: MODENIF Position          */
#define UART_INTSTS_MODEMIF_Msk          (0x1ul << UART_INTSTS_MODEMIF_Pos)                /*!< UART_T::INTSTS: MODENIF Mask              */

#define UART_INTSTS_RXTOIF_Pos           (4)                                               /*!< UART_T::INTSTS: RXTOIF Position           */
#define UART_INTSTS_RXTOIF_Msk           (0x1ul << UART_INTSTS_RXTOIF_Pos)                 /*!< UART_T::INTSTS: RXTOIF Mask               */

#define UART_INTSTS_BUFERRIF_Pos         (5)                                               /*!< UART_T::INTSTS: BUFERRIF Position         */
#define UART_INTSTS_BUFERRIF_Msk         (0x1ul << UART_INTSTS_BUFERRIF_Pos)               /*!< UART_T::INTSTS: BUFERRIF Mask             */

#define UART_INTSTS_RDAINT_Pos           (8)                                               /*!< UART_T::INTSTS: RDAINT Position           */
#define UART_INTSTS_RDAINT_Msk           (0x1ul << UART_INTSTS_RDAINT_Pos)                 /*!< UART_T::INTSTS: RDAINT Mask               */

#define UART_INTSTS_THREINT_Pos          (9)                                               /*!< UART_T::INTSTS: THREINT Position          */
#define UART_INTSTS_THREINT_Msk          (0x1ul << UART_INTSTS_THREINT_Pos)                /*!< UART_T::INTSTS: THREINT Mask              */

#define UART_INTSTS_RLSINT_Pos           (10)                                              /*!< UART_T::INTSTS: RLSINT Position           */
#define UART_INTSTS_RLSINT_Msk           (0x1ul << UART_INTSTS_RLSINT_Pos)                 /*!< UART_T::INTSTS: RLSINT Mask               */

#define UART_INTSTS_MODEMINT_Pos         (11)                                              /*!< UART_T::INTSTS: MODEMINT Position         */
#define UART_INTSTS_MODEMINT_Msk         (0x1ul << UART_INTSTS_MODEMINT_Pos)               /*!< UART_T::INTSTS: MODEMINT Mask             */

#define UART_INTSTS_RXTOINT_Pos          (12)                                              /*!< UART_T::INTSTS: RXTOINT Position          */
#define UART_INTSTS_RXTOINT_Msk          (0x1ul << UART_INTSTS_RXTOINT_Pos)                /*!< UART_T::INTSTS: RXTOINT Mask              */

#define UART_INTSTS_BUFERRINT_Pos        (13)                                              /*!< UART_T::INTSTS: BUFERRINT Position        */
#define UART_INTSTS_BUFERRINT_Msk        (0x1ul << UART_INTSTS_BUFERRINT_Pos)              /*!< UART_T::INTSTS: BUFERRINT Mask            */

#define UART_TOUT_TOIC_Pos               (0)                                               /*!< UART_T::TOUT: TOIC Position               */
#define UART_TOUT_TOIC_Msk               (0xfful << UART_TOUT_TOIC_Pos)                    /*!< UART_T::TOUT: TOIC Mask                   */

#define UART_TOUT_DLY_Pos                (8)                                               /*!< UART_T::TOUT: DLY Position                */
#define UART_TOUT_DLY_Msk                (0xfful << UART_TOUT_DLY_Pos)                     /*!< UART_T::TOUT: DLY Mask                    */

#define UART_BAUD_BRD_Pos                (0)                                               /*!< UART_T::BAUD: BRD Position                */
#define UART_BAUD_BRD_Msk                (0xfffful << UART_BAUD_BRD_Pos)                   /*!< UART_T::BAUD: BRD Mask                    */

#define UART_BAUD_EDIVM1_Pos             (24)                                              /*!< UART_T::BAUD: EDIVM1 Position             */
#define UART_BAUD_EDIVM1_Msk             (0xful << UART_BAUD_EDIVM1_Pos)                   /*!< UART_T::BAUD: EDIVM1 Mask                 */

#define UART_BAUD_BAUDM0_Pos             (28)                                              /*!< UART_T::BAUD: BAUDM0 Position             */
#define UART_BAUD_BAUDM0_Msk             (0x1ul << UART_BAUD_BAUDM0_Pos)                   /*!< UART_T::BAUD: BAUDM0 Mask                 */

#define UART_BAUD_BAUDM1_Pos             (29)                                              /*!< UART_T::BAUD: BAUDM1 Position             */
#define UART_BAUD_BAUDM1_Msk             (0x1ul << UART_BAUD_BAUDM1_Pos)                   /*!< UART_T::BAUD: BAUDM1 Mask                 */

#define UART_IRDA_TXEN_Pos               (1)                                               /*!< UART_T::IRDA: TXEN Position               */
#define UART_IRDA_TXEN_Msk               (0x1ul << UART_IRDA_TXEN_Pos)                     /*!< UART_T::IRDA: TXEN Mask                   */

#define UART_IRDA_TXINV_Pos              (5)                                               /*!< UART_T::IRDA: TXINV Position              */
#define UART_IRDA_TXINV_Msk              (0x1ul << UART_IRDA_TXINV_Pos)                    /*!< UART_T::IRDA: TXINV Mask                  */

#define UART_IRDA_RXINV_Pos              (6)                                               /*!< UART_T::IRDA: RXINV Position              */
#define UART_IRDA_RXINV_Msk              (0x1ul << UART_IRDA_RXINV_Pos)                    /*!< UART_T::IRDA: RXINV Mask                  */

#define UART_ALTCTL_RS485NMM_Pos         (8)                                               /*!< UART_T::ALTCTL: RS485NMM Position         */
#define UART_ALTCTL_RS485NMM_Msk         (0x1ul << UART_ALTCTL_RS485NMM_Pos)               /*!< UART_T::ALTCTL: RS485NMM Mask             */

#define UART_ALTCTL_RS485AAD_Pos         (9)                                               /*!< UART_T::ALTCTL: RS485AAD Position         */
#define UART_ALTCTL_RS485AAD_Msk         (0x1ul << UART_ALTCTL_RS485AAD_Pos)               /*!< UART_T::ALTCTL: RS485AAD Mask             */

#define UART_ALTCTL_RS485AUD_Pos         (10)                                              /*!< UART_T::ALTCTL: RS485AUD Position         */
#define UART_ALTCTL_RS485AUD_Msk         (0x1ul << UART_ALTCTL_RS485AUD_Pos)               /*!< UART_T::ALTCTL: RS485AUD Mask             */

#define UART_ALTCTL_ADDRDEN_Pos          (15)                                              /*!< UART_T::ALTCTL: ADDRDEN Position          */
#define UART_ALTCTL_ADDRDEN_Msk          (0x1ul << UART_ALTCTL_ADDRDEN_Pos)                /*!< UART_T::ALTCTL: ADDRDEN Mask              */

#define UART_ALTCTL_ADDRMV_Pos           (24)                                              /*!< UART_T::ALTCTL: ADDRMV Position           */
#define UART_ALTCTL_ADDRMV_Msk           (0xfful << UART_ALTCTL_ADDRMV_Pos)                /*!< UART_T::ALTCTL: ADDRMV Mask               */

#define UART_FUNCSEL_FUNCSEL_Pos         (0)                                               /*!< UART_T::FUNCSEL: FUNCSEL Position         */
#define UART_FUNCSEL_FUNCSEL_Msk         (0x3ul << UART_FUNCSEL_FUNCSEL_Pos)               /*!< UART_T::FUNCSEL: FUNCSEL Mask             */

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
     * CTL
     * ===================================================================================================
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
    __IO uint32_t CTL;

} WDT_T;

/**
    @addtogroup WDT_CONST WDT Bit Field Definition
    Constant Definitions for WDT Controller
@{ */

#define WDT_CTL_RSTCNT_Pos               (0)                                               /*!< WDT_T::CTL: RSTCNT Position               */
#define WDT_CTL_RSTCNT_Msk               (0x1ul << WDT_CTL_RSTCNT_Pos)                     /*!< WDT_T::CTL: RSTCNT Mask                   */

#define WDT_CTL_RSTEN_Pos                (1)                                               /*!< WDT_T::CTL: RSTEN Position                */
#define WDT_CTL_RSTEN_Msk                (0x1ul << WDT_CTL_RSTEN_Pos)                      /*!< WDT_T::CTL: RSTEN Mask                    */

#define WDT_CTL_RSTF_Pos                 (2)                                               /*!< WDT_T::CTL: RSTF Position                 */
#define WDT_CTL_RSTF_Msk                 (0x1ul << WDT_CTL_RSTF_Pos)                       /*!< WDT_T::CTL: RSTF Mask                     */

#define WDT_CTL_IF_Pos                   (3)                                               /*!< WDT_T::CTL: IF Position                   */
#define WDT_CTL_IF_Msk                   (0x1ul << WDT_CTL_IF_Pos)                         /*!< WDT_T::CTL: IF Mask                       */

#define WDT_CTL_WKEN_Pos                 (4)                                               /*!< WDT_T::CTL: WKEN Position                 */
#define WDT_CTL_WKEN_Msk                 (0x1ul << WDT_CTL_WKEN_Pos)                       /*!< WDT_T::CTL: WKEN Mask                     */

#define WDT_CTL_WKF_Pos                  (5)                                               /*!< WDT_T::CTL: WKF Position                  */
#define WDT_CTL_WKF_Msk                  (0x1ul << WDT_CTL_WKF_Pos)                        /*!< WDT_T::CTL: WKF Mask                      */

#define WDT_CTL_INTEN_Pos                (6)                                               /*!< WDT_T::CTL: INTEN Position                */
#define WDT_CTL_INTEN_Msk                (0x1ul << WDT_CTL_INTEN_Pos)                      /*!< WDT_T::CTL: INTEN Mask                    */

#define WDT_CTL_WDTEN_Pos                (7)                                               /*!< WDT_T::CTL: WDTEN Position                */
#define WDT_CTL_WDTEN_Msk                (0x1ul << WDT_CTL_WDTEN_Pos)                      /*!< WDT_T::CTL: WDTEN Mask                    */

#define WDT_CTL_TOUTSEL_Pos              (8)                                               /*!< WDT_T::CTL: TOUTSEL Position              */
#define WDT_CTL_TOUTSEL_Msk              (0x7ul << WDT_CTL_TOUTSEL_Pos)                    /*!< WDT_T::CTL: TOUTSEL Mask                  */

#define WDT_CTL_ICEDEBUG_Pos             (31)                                              /*!< WDT_T::CTL: ICEDEBUG Position             */
#define WDT_CTL_ICEDEBUG_Msk             (0x1ul << WDT_CTL_ICEDEBUG_Pos)                   /*!< WDT_T::CTL: ICEDEBUG Mask                 */

/**@}*/ /* WDT_CONST */
/**@}*/ /* end of WDT register group */


#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/** @addtogroup Mini55_PERIPHERAL_MEM_MAP MINI55 Peripheral Memory Map
  Memory Mapped Structure for MINI55 Series Peripheral
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
#define TIMER_AC_BASE         (APB1PERIPH_BASE + 0x10040)    ///< TIMER Advance Capture register base address
#define I2C_BASE              (APB1PERIPH_BASE + 0x20000)    ///< I2C register base address
#define SPI_BASE              (APB1PERIPH_BASE + 0x30000)    ///< SPI register base address
#define PWM_BASE              (APB1PERIPH_BASE + 0x40000)    ///< PWM register base address
#define UART0_BASE            (APB1PERIPH_BASE + 0x50000)    ///< UART0 register base address
#define UART1_BASE            (APB2PERIPH_BASE + 0x50000)    ///< UART1 register base address
#define ACMP_BASE             (APB1PERIPH_BASE + 0xD0000)    ///< ACMP register base address
#define ADC_BASE              (APB1PERIPH_BASE + 0xE0000)    ///< ADC register base address

#define SYS_BASE              (AHBPERIPH_BASE + 0x00000)    ///< SYS register base address
#define CLK_BASE              (AHBPERIPH_BASE + 0x00200)    ///< CLK register base address
#define INTR_BASE             (AHBPERIPH_BASE + 0x00300)    ///< INT register base address
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
#define HDIV_BASE             (AHBPERIPH_BASE + 0x14000)    ///< HDIV register base address

/*@}*/ /* end of group Mini55_PERIPHERAL_MEM_MAP */


/** @addtogroup Mini55_PERIPHERAL_DECLARATION MINI55 Peripheral Declaration
  The Declaration of MINI55 Series Peripheral
  @{
 */
#define WDT                   ((WDT_T *) WDT_BASE)              ///< Pointer to WDT register structure
#define TIMER0                ((TIMER_T *) TIMER0_BASE)         ///< Pointer to Timer 0 register structure
#define TIMER1                ((TIMER_T *) TIMER1_BASE)         ///< Pointer to Timer 1 register structure
#define TIMERAC               ((TIMER_AC_T *) TIMER_AC_BASE)    ///< Pointer to Timer Advance Capture register structure
#define I2C                   ((I2C_T *) I2C_BASE)              ///< Pointer to I2C register structure
#define I2C0                  ((I2C_T *) I2C_BASE)              ///< Pointer to I2C register structure
#define SPI                   ((SPI_T *) SPI_BASE)              ///< Pointer to SPI register structure
#define SPI0                  ((SPI_T *) SPI_BASE)              ///< Pointer to SPI register structure
#define PWM                   ((PWM_T *) PWM_BASE)              ///< Pointer to PWM register structure
#define UART                  ((UART_T *) UART0_BASE)           ///< Pointer to UART0 register structure (compatible with Mini51)
#define UART0                 ((UART_T *) UART0_BASE)           ///< Pointer to UART0 register structure
#define UART1                 ((UART_T *) UART1_BASE)           ///< Pointer to UART1 register structure
#define ADC                   ((ADC_T *) ADC_BASE)              ///< Pointer to ADC register structure
#define ACMP                  ((ACMP_T *) ACMP_BASE)            ///< Pointer to ACMP register structure

#define SYS                   ((SYS_T *) SYS_BASE)              ///< Pointer to SYS register structure
#define CLK                   ((CLK_T *) CLK_BASE)              ///< Pointer to CLK register structure
#define INTR                  ((INTR_T *) INTR_BASE)            ///< Pointer to INT register structure
#define P0                    ((GPIO_T *) P0_BASE)              ///< Pointer to GPIO port 0 register structure
#define P1                    ((GPIO_T *) P1_BASE)              ///< Pointer to GPIO port 1 register structure
#define P2                    ((GPIO_T *) P2_BASE)              ///< Pointer to GPIO port 2 register structure
#define P3                    ((GPIO_T *) P3_BASE)              ///< Pointer to GPIO port 3 register structure
#define P4                    ((GPIO_T *) P4_BASE)              ///< Pointer to GPIO port 4 register structure
#define P5                    ((GPIO_T *) P5_BASE)              ///< Pointer to GPIO port 5 register structure
#define GPIO                  ((GPIO_DB_T *) GPIO_DBNCECON_BASE)  ///< Pointer to GPIO de-bounce register structure
#define FMC                   ((FMC_T *) FMC_BASE)              ///< Pointer to FMC register structure
#define HDIV                  ((HDIV_T *) HDIV_BASE)            ///< Pointer to HDIV register structure

/*@}*/ /* end of group Mini55_PERIPHERAL_DECLARATION */
/*@}*/ /* end of group Mini55_Peripherals */

/** @addtogroup Mini55_IO_ROUTINE MINI55 I/O Routines
  The Declaration of MINI55 I/O Routines
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


/*@}*/ /* end of group Mini55_IO_ROUTINE */

/******************************************************************************/
/*                Legacy Constants                                            */
/******************************************************************************/
/** @addtogroup Mini55_legacy_Constants MINI55 Legacy Constants
  MINI55 Legacy Constants
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

/*@}*/ /* end of group Mini55_legacy_Constants */

/*@}*/ /* end of group Mini55_Definitions */

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
#include "hdiv.h"
#include "i2c.h"
#include "pwm.h"
#include "spi.h"
#include "timer.h"
#include "uart.h"
#include "wdt.h"

#endif  // __MINI55SERIES_H__

