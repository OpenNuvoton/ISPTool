/**************************************************************************//**
 * @file     ISD9100.h
 * @version  V3.0
 * $Revision: 32 $
 * $Date: 14/07/11 11:06a $
 * @brief    ISD9100 Series Peripheral Access Layer Header File
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/



/**
  \mainpage Introduction
  *
  *
  * This user manual describes the usage of ISD9100 Series MCU device driver
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
  * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
  */

/**
  * \page PG_DIR Directory Structure
  * 
  * Please refer to Readme.pdf under BSP root directory for the BSP directory structure. 
  *
  * \page PG_REV Revision History
  *
  *
  * <b>Revision 3.00.001</b>
  * \li Updated to support new API
*/

#ifndef __ISD9100_H__
#define __ISD9100_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
*/

/** @addtogroup ISD93XX_CMSIS Device Definitions for CMSIS
  ISD93XX Interrupt Number Definition and Configurations for CMSIS
  @{
*/

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */

typedef enum IRQn
    {
        /******  Cortex-M0 Processor Exceptions Numbers *************************************************/
        NonMaskableInt_IRQn       = -14,    /*!< 2 Non Maskable Interrupt                               */
        HardFault_IRQn		      = -13,    /*!< 3 Cortex-M0 Hard Fault Interrupt                   */
        SVCall_IRQn               = -5,     /*!< 11 Cortex-M0 SV Call Interrupt                         */
        PendSV_IRQn               = -2,     /*!< 14 Cortex-M0 Pend SV Interrupt                         */
        SysTick_IRQn              = -1,     /*!< 15 Cortex-M0 System Tick Interrupt                     */
        /******  ARMIKMCU Swift specific Interrupt Numbers **********************************************/
        BOD_IRQn                  = 0,      /*!< 16 Brown Out Detector Device Interrupt               */
        WDT_IRQn                  = 1,      /*!< 17 Watchdog Timer Device Interrupt                   */
        EINT0_IRQn                = 2,      /*!< 18 External Interrupt PB0 Interrupt                  */
        EINT1_IRQn                = 3,      /*!< 19 External Interrupt PB1 Interrupt                  */
        GPAB_IRQn                 = 4,      /*!< 20 GPIO Interrupt                                    */
        ALC_IRQn                  = 5,      /*!< 21 ALC Interrupt                                     */
        PWM0_IRQn                 = 6,      /*!< 22 PWM 0 Peripheral Device Interrupt                 */
        IRQ7n                     = 7,
        TMR0_IRQn                 = 8,      /*!< 24 Timer 0 Interrupt                                 */
        TMR1_IRQn                 = 9,      /*!< 25 Timer 1 Interrupt                                 */
        IRQ10n                    = 10,
        IRQ11n                    = 11,
        UART0_IRQn                = 12,     /*!< 28 UART Device Interrupt                            */
        IRQ13n                    = 13,
        SPI0_IRQn                 = 14,     /*!< 30 SPI Interface Interrupt                          */
        IRQ15n                    = 15,
        IRQ16n                    = 16,
        IRQ17n                    = 17,
        I2C0_IRQn                 = 18,     /*!< 34 I2C Interface Interrupt                          */
        IRQ19n                    = 19,
        IRQ20n                    = 20,
        TALARM_IRQn               = 21,     /*!< 37 Temperature Alaram Interrupt                     */
        IRQ22n                    = 22,
        IRQ23n                    = 23,
        IRQ24n                    = 24,
        ACMP_IRQn                 = 25,     /*!< 41 Analog Comparator Interrupt                      */
        PDMA_IRQn                 = 26,     /*!< 42 Peripheral DMA Interrupt                         */
        I2S0_IRQn                  = 27,    /*!< 43 I2S Interface Device Interrupt                   */
        CAPS_IRQn                 = 28,     /*!< 44 CapSense Device Interrupt                        */
        ADC_IRQn                  = 29,     /*!< 45 Audio ADC Device Interrupt                       */
        IRQ30n                    = 30,
        RTC_IRQn                  = 31      /*!< 47 Real Time Clock Interrupt                        */

        /*!< maximum of 32 Interrupts are possible                */
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
/*@}*/ /* end of group ISD9100_CMSIS */

#include "core_cm0.h"                   /* Cortex-M0 processor and core peripherals               */
#include "system_ISD9100.h"              /* ISD9xx System include file                             */

#if defined ( __CC_ARM   )
  #pragma anon_unions
#elif  ( defined (__ICCARM__) )
// IAR C compiler detected
  #define __wfi       __WFI
  #ifndef __STATIC_INLINE
    #define __STATIC_INLINE  static inline
  #endif
/*
Usage of #define
  #define A(x)  T_##x
  #define B¡]x) #@x
  #define C¡]x) #x

  A(1)------>T_1
  B(1)------>'1'
  C(1)------>"1"
*/
  #define __quote(n)      #n
  #define __iar_align(n)  __quote(data_alignment=##n)
  #define __align(n)      _Pragma(__iar_align(n))
#endif


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

  @{

*/


/*---------------------- Analog Comparator Controller -------------------------*/
/**
    @addtogroup ACMP Analog Comparator Controller(ACMP)
    Memory Mapped Structure for ACMP Controller
@{ */
 
typedef struct
{


    /**
     * CTL0
     * ===================================================================================================
     * Offset: 0x00  Analog Comparator 0 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPEN    |Comparator Enable
     * |        |          |0 = Disable
     * |        |          |1 = Enable
     * |[1]     |ACMPIE    |CMP0 Interrupt Enable
     * |        |          |0 = Disable CMP0 interrupt function
     * |        |          |1 = Enable CMP0 interrupt function
     * |[4]     |NEGSEL    |Comparator0 Negative Input Select
     * |        |          |0 = VBG, Bandgap reference voltage = 1.2V
     * |        |          |1 = VMID reference voltage = VCCA/2
 */
    __IO uint32_t CTL0;                  

    /**
     * CTL1
     * ===================================================================================================
     * Offset: 0x04  Analog Comparator 1 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPEN    |Comparator Enable
     * |        |          |0 = Disable
     * |        |          |1 = Enable
     * |[1]     |ACMPIE    |CMP1 Interrupt Enable
     * |        |          |0 = Disable CMP1interrupt function
     * |        |          |1 = Enable CMP1 interrupt function
     * |[4]     |NEGSEL    |Comparator1 Negative Input Select
     * |        |          |0 = GPIOB[7]
     * |        |          |1 = VBG, Bandgap reference voltage = 1.2V 
 */
    __IO uint32_t CTL1;                  

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x08  Comparator Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ACMPIF0   |Compare 0 Flag
     * |        |          |This bit is set by hardware whenever the comparator output changes state.
     * |        |          |This bit will cause a hardware interrupt if enabled.
     * |        |          |This bit is cleared by writing 1 to itself.
     * |[1]     |ACMPIF1   |Compare 1 Flag
     * |        |          |This bit is set by hardware whenever the comparator output changes state.
     * |        |          |This bit will cause a hardware interrupt if enabled.
     * |        |          |This bit is cleared by writing 1 to itself.
     * |[2]     |ACMPO0    |Comparator0 Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the comparator is disabled (CMP0EN = 0).
     * |[3]     |ACMPO1    |Comparator1 Output
     * |        |          |Synchronized to the APB clock to allow reading by software.
     * |        |          |Cleared when the comparator is disabled (CMP1EN = 0).
 */
    __IO uint32_t STATUS;                

    /**
     * POSSEL
     * ===================================================================================================
     * Offset: 0x0C  Comparator Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:2]   |POSSEL    |Comparator0 GPIO Selection
     * |        |          |GPIOB[POSSEL] is the active analog GPIO input selected to Comparator 0 positive input. 
 */
    __IO uint32_t POSSEL;                

} ACMP_T;

/**
    @addtogroup ACMP_CONST ACMP Bit Field Definition
    Constant Definitions for ACMP Controller
@{ */

#define ACMP_CTL0_ACMPEN_Pos             (0)                                               /*!< ACMP CTL0: ACMPEN Position             */
#define ACMP_CTL0_ACMPEN_Msk             (0x1ul << ACMP_CTL0_ACMPEN_Pos)                   /*!< ACMP CTL0: ACMPEN Mask                 */

#define ACMP_CTL0_ACMPIE_Pos             (1)                                               /*!< ACMP CTL0: ACMPIE Position             */
#define ACMP_CTL0_ACMPIE_Msk             (0x1ul << ACMP_CTL0_ACMPIE_Pos)                   /*!< ACMP CTL0: ACMPIE Mask                 */

#define ACMP_CTL0_NEGSEL_Pos             (4)                                               /*!< ACMP CTL0: NEGSEL Position             */
#define ACMP_CTL0_NEGSEL_Msk             (0x1ul << ACMP_CTL0_NEGSEL_Pos)                   /*!< ACMP CTL0: NEGSEL Mask                 */

#define ACMP_CTL1_ACMPEN_Pos             (0)                                               /*!< ACMP CTL1: ACMPEN Position             */
#define ACMP_CTL1_ACMPEN_Msk             (0x1ul << ACMP_CTL1_ACMPEN_Pos)                   /*!< ACMP CTL1: ACMPEN Mask                 */

#define ACMP_CTL1_ACMPIE_Pos             (1)                                               /*!< ACMP CTL1: ACMPIE Position             */
#define ACMP_CTL1_ACMPIE_Msk             (0x1ul << ACMP_CTL1_ACMPIE_Pos)                   /*!< ACMP CTL1: ACMPIE Mask                 */

#define ACMP_CTL1_NEGSEL_Pos             (4)                                               /*!< ACMP CTL1: NEGSEL Position             */
#define ACMP_CTL1_NEGSEL_Msk             (0x1ul << ACMP_CTL1_NEGSEL_Pos)                   /*!< ACMP CTL1: NEGSEL Mask                 */

#define ACMP_STATUS_ACMPIF0_Pos          (0)                                               /*!< ACMP STATUS: ACMPIF0 Position          */
#define ACMP_STATUS_ACMPIF0_Msk          (0x1ul << ACMP_STATUS_ACMPIF0_Pos)                /*!< ACMP STATUS: ACMPIF0 Mask              */

#define ACMP_STATUS_ACMPIF1_Pos          (1)                                               /*!< ACMP STATUS: ACMPIF1 Position          */
#define ACMP_STATUS_ACMPIF1_Msk          (0x1ul << ACMP_STATUS_ACMPIF1_Pos)                /*!< ACMP STATUS: ACMPIF1 Mask              */

#define ACMP_STATUS_ACMPO0_Pos           (2)                                               /*!< ACMP STATUS: ACMPO0 Position           */
#define ACMP_STATUS_ACMPO0_Msk           (0x1ul << ACMP_STATUS_ACMPO0_Pos)                 /*!< ACMP STATUS: ACMPO0 Mask               */

#define ACMP_STATUS_ACMPO1_Pos           (3)                                               /*!< ACMP STATUS: ACMPO1 Position           */
#define ACMP_STATUS_ACMPO1_Msk           (0x1ul << ACMP_STATUS_ACMPO1_Pos)                 /*!< ACMP STATUS: ACMPO1 Mask               */

#define ACMP_POSSEL_POSSEL_Pos           (0)                                               /*!< ACMP POSSEL: POSSEL Position           */
#define ACMP_POSSEL_POSSEL_Msk           (0x7ul << ACMP_POSSEL_POSSEL_Pos)                 /*!< ACMP POSSEL: POSSEL Mask               */

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
     * Offset: 0x00  ADC FIFO Data Out.
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |RESULT    |ADC Audio Data FIFO Read
     * |        |          |A read of this register will read data from the audio FIFO and increment the read pointer.
     * |        |          |A read past empty will repeat the last data.
     * |        |          |Can be used with FIFOINTLV interrupt to determine if valid data is present in FIFO.
 */
    __I  uint32_t DAT;                   

    /**
     * CHEN
     * ===================================================================================================
     * Offset: 0x04  ADC Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHEN      |ADC Enable
     * |        |          |0 = Conversion stopped and ADC is reset including FIFO pointers.
     * |        |          |1 = ADC Conversion enabled.
 */
    __IO uint32_t CHEN;                  

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x08  ADC Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |CLKDIV    |ADC Clock Divider
     * |        |          |This register determines the clock division ration between the incoming ADC_CLK (= HCLK by default) and the Delta-Sigma sampling clock of the ADC.
     * |        |          |This together with the over-sampling ratio (OSR) determines the audio sample rate of the converter.
     * |        |          |CLKDIV should be set to give a SD_CLK frequency in the range of 1.024-6.144MHz.
     * |        |          |CLKDIV must be greater than 2.
     * |        |          |SD_CLK frequency = HCLK / CLKDIV
 */
    __IO uint32_t CLKDIV;                

    /**
     * DCICTL
     * ===================================================================================================
     * Offset: 0x0C  ADC Decimation Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |OVSPLRAT  |Decimation Over-Sampling Ratio
     * |        |          |This term determines the over-sampling ratio of the decimation filter. Valid values are:
     * |        |          |0: OVSPLRAT = 64
     * |        |          |1: OVSPLRAT = 128
     * |        |          |2: OVSPLRAT = 192
     * |        |          |3: OVSPLRAT = 384
     * |[16:19] |GAIN      |CIC Filter Additional Gain
     * |        |          |This should normally remain default 0.
     * |        |          |Can be set to non-zero values to provide additional digital gain from the decimation filter.
     * |        |          |An additional gain is applied to signal of GAIN/2.
 */
    __IO uint32_t DCICTL;                

    /**
     * INTCTL
     * ===================================================================================================
     * Offset: 0x10  ADC Interrupt Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:2]   |FIFOINTLV |FIFO Interrupt Level
     * |        |          |Determines at what level the ADC FIFO will generate a servicing interrupt to the CPU.
     * |        |          |Interrupt will be generated when number of words present in ADC FIFO is > FIFOINTLV.
     * |[31]    |INTEN     |Interrupt Enable
     * |        |          |If set to '1' an interrupt is generated whenever FIFO level exceeds that set in FIFOINTLV.
 */
    __IO uint32_t INTCTL;                

    /**
     * PDMACTL
     * ===================================================================================================
     * Offset: 0x14  ADC PDMA Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXDMAEN   |Enable ADC PDMA Receive Channel
     * |        |          |Enable ADC PDMA. If set, then ADC will request PDMA service when data is available.
 */
    __IO uint32_t PDMACTL;               

    /**
     * CMP0
     * ===================================================================================================
     * Offset: 0x18  ADC Comparator 0 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCMPEN   |Compare Enable
     * |        |          |0 = Disable compare.
     * |        |          |1 = Enable compare.
     * |        |          |Set this bit to 1 to enable compare CMPDAT with FIFO data output.
     * |[1]     |ADCMPIE   |Compare Interrupt Enable
     * |        |          |0 = Disable compare function interrupt.
     * |        |          |1 = Enable compare function interrupt.
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMCNT, CMPFLAG bit will be asserted, if ADCMPIE is set to 1, a compare interrupt request is generated.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0= Set the compare condition that result is less than CMPDAT
     * |        |          |1= Set the compare condition that result is greater or equal to CMPDAT
     * |        |          |Note: When the internal counter reaches the value (CMPMCNT +1), the CMPFLAG bit will be set.
     * |[7]     |CMPFLAG   |Compare Flag
     * |        |          |When the conversion result meets condition in ADCMPR0 this bit is set to 1.
     * |        |          |It is cleared by writing 1 to self.
     * |[8:11]  |CMPMCNT   |Compare Match Count
     * |        |          |When the A/D FIFO result matches the compare condition defined by CMPCOND, the internal match counter will increase by 1.
     * |        |          |When the internal counter reaches the value to (CMPMCNT +1), the CMPFLAG bit will be set.
     * |[16:31] |CMPDAT    |Comparison Data
     * |        |          |16 bit value to compare to FIFO output word.
 */
    __IO uint32_t CMP0;                  

    /**
     * CMP1
     * ===================================================================================================
     * Offset: 0x1C  ADC Comparator 1 Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ADCMPEN   |Compare Enable
     * |        |          |0 = Disable compare.
     * |        |          |1 = Enable compare.
     * |        |          |Set this bit to 1 to enable compare CMPDAT with FIFO data output.
     * |[1]     |ADCMPIE   |Compare Interrupt Enable
     * |        |          |0 = Disable compare function interrupt.
     * |        |          |1 = Enable compare function interrupt.
     * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMCNT, CMPFLAG bit will be asserted, if ADCMPIE is set to 1, a compare interrupt request is generated.
     * |[2]     |CMPCOND   |Compare Condition
     * |        |          |0= Set the compare condition that result is less than CMPDAT
     * |        |          |1= Set the compare condition that result is greater or equal to CMPDAT
     * |        |          |Note: When the internal counter reaches the value (CMPMCNT +1), the CMPFLAG bit will be set.
     * |[7]     |CMPFLAG   |Compare Flag
     * |        |          |When the conversion result meets condition in ADCMPR0 this bit is set to 1.
     * |        |          |It is cleared by writing 1 to self.
     * |[8:11]  |CMPMCNT   |Compare Match Count
     * |        |          |When the A/D FIFO result matches the compare condition defined by CMPCOND, the internal match counter will increase by 1.
     * |        |          |When the internal counter reaches the value to (CMPMCNT +1), the CMPFLAG bit will be set.
     * |[16:31] |CMPDAT    |Comparison Data
     * |        |          |16 bit value to compare to FIFO output word.
 */
    __IO uint32_t CMP1;                  

} ADC_T;

/**
    @addtogroup ADC_CONST ADC Bit Field Definition
    Constant Definitions for ADC Controller
@{ */

#define ADC_DAT_RESULT_Pos               (0)                                               /*!< ADC DAT: RESULT Position               */
#define ADC_DAT_RESULT_Msk               (0xfffful << ADC_DAT_RESULT_Pos)                  /*!< ADC DAT: RESULT Mask                   */

#define ADC_CHEN_CHEN_Pos                (0)                                               /*!< ADC CHEN: CHEN Position                */
#define ADC_CHEN_CHEN_Msk                (0x1ul << ADC_CHEN_CHEN_Pos)                      /*!< ADC CHEN: CHEN Mask                    */

#define ADC_CLKDIV_CLKDIV_Pos            (0)                                               /*!< ADC CLKDIV: CLKDIV Position            */
#define ADC_CLKDIV_CLKDIV_Msk            (0xfful << ADC_CLKDIV_CLKDIV_Pos)                 /*!< ADC CLKDIV: CLKDIV Mask                */

#define ADC_DCICTL_OVSPLRAT_Pos          (0)                                               /*!< ADC DCICTL: OVSPLRAT Position          */
#define ADC_DCICTL_OVSPLRAT_Msk          (0xful << ADC_DCICTL_OVSPLRAT_Pos)                /*!< ADC DCICTL: OVSPLRAT Mask              */

#define ADC_DCICTL_GAIN_Pos              (16)                                              /*!< ADC DCICTL: GAIN Position              */
#define ADC_DCICTL_GAIN_Msk              (0xful << ADC_DCICTL_GAIN_Pos)                    /*!< ADC DCICTL: GAIN Mask                  */

#define ADC_INTCTL_FIFOINTLV_Pos         (0)                                               /*!< ADC INTCTL: FIFOINTLV Position         */
#define ADC_INTCTL_FIFOINTLV_Msk         (0x7ul << ADC_INTCTL_FIFOINTLV_Pos)               /*!< ADC INTCTL: FIFOINTLV Mask             */

#define ADC_INTCTL_INTEN_Pos             (31)                                              /*!< ADC INTCTL: INTEN Position             */
#define ADC_INTCTL_INTEN_Msk             (0x1ul << ADC_INTCTL_INTEN_Pos)                   /*!< ADC INTCTL: INTEN Mask                 */

#define ADC_PDMACTL_RXDMAEN_Pos          (0)                                               /*!< ADC PDMACTL: RXDMAEN Position          */
#define ADC_PDMACTL_RXDMAEN_Msk          (0x1ul << ADC_PDMACTL_RXDMAEN_Pos)                /*!< ADC PDMACTL: RXDMAEN Mask              */

#define ADC_CMP0_ADCMPEN_Pos             (0)                                               /*!< ADC CMP0: ADCMPEN Position             */
#define ADC_CMP0_ADCMPEN_Msk             (0x1ul << ADC_CMP0_ADCMPEN_Pos)                   /*!< ADC CMP0: ADCMPEN Mask                 */

#define ADC_CMP0_ADCMPIE_Pos             (1)                                               /*!< ADC CMP0: ADCMPIE Position             */
#define ADC_CMP0_ADCMPIE_Msk             (0x1ul << ADC_CMP0_ADCMPIE_Pos)                   /*!< ADC CMP0: ADCMPIE Mask                 */

#define ADC_CMP0_CMPCOND_Pos             (2)                                               /*!< ADC CMP0: CMPCOND Position             */
#define ADC_CMP0_CMPCOND_Msk             (0x1ul << ADC_CMP0_CMPCOND_Pos)                   /*!< ADC CMP0: CMPCOND Mask                 */

#define ADC_CMP0_CMPFLAG_Pos             (7)                                               /*!< ADC CMP0: CMPFLAG Position             */
#define ADC_CMP0_CMPFLAG_Msk             (0x1ul << ADC_CMP0_CMPFLAG_Pos)                   /*!< ADC CMP0: CMPFLAG Mask                 */

#define ADC_CMP0_CMPMCNT_Pos             (8)                                               /*!< ADC CMP0: CMPMCNT Position             */
#define ADC_CMP0_CMPMCNT_Msk             (0xful << ADC_CMP0_CMPMCNT_Pos)                   /*!< ADC CMP0: CMPMCNT Mask                 */

#define ADC_CMP0_CMPDAT_Pos              (16)                                              /*!< ADC CMP0: CMPDAT Position              */
#define ADC_CMP0_CMPDAT_Msk              (0xfffful << ADC_CMP0_CMPDAT_Pos)                 /*!< ADC CMP0: CMPDAT Mask                  */

#define ADC_CMP1_ADCMPEN_Pos             (0)                                               /*!< ADC CMP1: ADCMPEN Position             */
#define ADC_CMP1_ADCMPEN_Msk             (0x1ul << ADC_CMP1_ADCMPEN_Pos)                   /*!< ADC CMP1: ADCMPEN Mask                 */

#define ADC_CMP1_ADCMPIE_Pos             (1)                                               /*!< ADC CMP1: ADCMPIE Position             */
#define ADC_CMP1_ADCMPIE_Msk             (0x1ul << ADC_CMP1_ADCMPIE_Pos)                   /*!< ADC CMP1: ADCMPIE Mask                 */

#define ADC_CMP1_CMPCOND_Pos             (2)                                               /*!< ADC CMP1: CMPCOND Position             */
#define ADC_CMP1_CMPCOND_Msk             (0x1ul << ADC_CMP1_CMPCOND_Pos)                   /*!< ADC CMP1: CMPCOND Mask                 */

#define ADC_CMP1_CMPFLAG_Pos             (7)                                               /*!< ADC CMP1: CMPFLAG Position             */
#define ADC_CMP1_CMPFLAG_Msk             (0x1ul << ADC_CMP1_CMPFLAG_Pos)                   /*!< ADC CMP1: CMPFLAG Mask                 */

#define ADC_CMP1_CMPMCNT_Pos             (8)                                               /*!< ADC CMP1: CMPMCNT Position             */
#define ADC_CMP1_CMPMCNT_Msk             (0xful << ADC_CMP1_CMPMCNT_Pos)                   /*!< ADC CMP1: CMPMCNT Mask                 */

#define ADC_CMP1_CMPDAT_Pos              (16)                                              /*!< ADC CMP1: CMPDAT Position              */
#define ADC_CMP1_CMPDAT_Msk              (0xfffful << ADC_CMP1_CMPDAT_Pos)                 /*!< ADC CMP1: CMPDAT Mask                  */

/**@}*/ /* ADC_CONST */
/**@}*/ /* end of ADC register group */


/*---------------------- Automatic Level Control -------------------------*/
/**
    @addtogroup ALC Automatic Level Control(ALC)
    Memory Mapped Structure for ALC Controller
@{ */
 
typedef struct
{


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  ALC Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:2]   |NGTHBST   |Noise Gate Threshold
     * |        |          |Boost disabled: Threshold = (-81+6xNGTHBST) dB
     * |        |          |Boost enabled: Threshold = (-87+6xNGTHBST) dB
     * |[3]     |NGEN      |Noise Gate Enable
     * |        |          |0 = Noise gate disabled
     * |        |          |1 = Noise gate enabled
     * |[4:7]   |ATKSEL    |ALC Attack Time
     * |        |          |(Value: 0~10)
     * |        |          |When MODESEL = 0, Range: 500us to 512ms
     * |        |          |When MODESEL = 1,Range: 125us to 128ms (Both ALC time doubles with every step)
     * |[8:11]  |DECAYSEL  |ALC Decay Time
     * |        |          |(Value: 0~10)
     * |        |          |When MODESEL = 0, Range: 125us to 128ms
     * |        |          |When MODESEL = 1, Range: 31us to 32ms (time doubles with every step)
     * |[12]    |MODESEL   |ALC Mode
     * |        |          |0 = ALC normal operation mode
     * |        |          |1 = ALC limiter mode
     * |[13:16] |TARGETLV  |ALC Target Level
     * |        |          |0 = -28.5 dB
     * |        |          |1 = -27 dB
     * |        |          |2 = -25.5 dB
     * |        |          |3 = -24 dB
     * |        |          |4 = -22.5 dB
     * |        |          |5 = -21 dB
     * |        |          |6 = -19.5 dB
     * |        |          |7 = -18 dB
     * |        |          |8 = -16.5 dB
     * |        |          |9 = -15 dB
     * |        |          |10 = -13.5 dB
     * |        |          |11 = -12 dB
     * |        |          |12 = -10.5 dB
     * |        |          |13 = -9 dB
     * |        |          |14 = -7.5 dB
     * |        |          |15 = -6 dB
     * |[17:20] |HOLDTIME  |ALC Hold Time
     * |        |          |(Value: 0~10). Hold Time = (2^HOLDTIME) ms
     * |[21]    |ZCEN      |ALC Zero Crossing
     * |        |          |0 = zero crossing disabled
     * |        |          |1 = zero crossing enabled
     * |[22:24] |MINGAIN   |ALC Minimum Gain
     * |        |          |0 = -12 dB
     * |        |          |1 = -6 dB
     * |        |          |2 = 0 dB
     * |        |          |3 = 6 dB
     * |        |          |4 = 12 dB
     * |        |          |5 = 18 dB
     * |        |          |6 = 24 dB
     * |        |          |7 = 30 dB
     * |[25:27] |MAXGAIN   |ALC Maximum Gain
     * |        |          |0 = -6.75 dB
     * |        |          |1 = -0.75 dB
     * |        |          |2 = +5.25 dB
     * |        |          |3 = +11.25 dB
     * |        |          |4 = +17.25 dB
     * |        |          |5 = +23.25 dB
     * |        |          |6 = +29.25 dB
     * |        |          |7 = +35.25 dB
     * |[28]    |ALCEN     |ALC select
     * |        |          |0 = ALC disabled (default)
     * |        |          |1 = ALC enabled
     * |[29]    |NGPKSEL   |ALC noise gate peak detector select
     * |        |          |0 = use peak-to-peak value for noise gate threshold determination (default)
     * |        |          |1 = use absolute peak value for noise gate threshold determination
     * |[30]    |PKSEL     |ALC gain peak detector select
     * |        |          |0 = use absolute peak value for ALC training (default)
     * |        |          |1 = use peak-to-peak value for ALC training
     * |[31]    |PKLIMEN   |ALC peak limiter enable
     * |        |          |0 = enable fast decrement when signal exceeds 87.5% of full scale (default)
     * |        |          |1 = disable fast decrement when signal exceeds 87.5% of full scale
 */
    __IO uint32_t CTL;                   

    /**
     * STS
     * ===================================================================================================
     * Offset: 0x04  ALC status register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CLIPFLAG  |Clipping Flag
     * |        |          |Asserted when signal level is detected to be above 87.5% of full scale
     * |[1]     |NOISEF    |Noise Flag
     * |        |          |Asserted when signal level is detected to be below NGTHBST
     * |[2:10]  |P2PVAL    |Peak-To-Peak Value
     * |        |          |9 MSBs of measured peak-to-peak value
     * |[11:18] |PEAKVAL   |Peak Value
     * |        |          |9 MSBs of measured absolute peak value
 */
    __I  uint32_t STS;                   

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x08  ALC interrupt register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTFLAG   |ALC interrupt flag
     * |        |          |This interrupt flag asserts whenever the interrupt is enabled and the PGA gain is updated, either through an ALC change with the ALC enabled or through a PGA gain write with the ALC disabled.
     * |        |          |Write a 1 to this register to clear.
 */
    __IO uint32_t INTSTS;                

    /**
     * INTCTL
     * ===================================================================================================
     * Offset: 0x0C  ALC interrupt enable register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTEN     |ALC Interrupt Enable
     * |        |          |0 = ALC INT disabled
     * |        |          |1 = ALC INT enabled
 */
    __IO uint32_t INTCTL;                

} ALC_T;

/**
    @addtogroup ALC_CONST ALC Bit Field Definition
    Constant Definitions for ALC Controller
@{ */

#define ALC_CTL_NGTHBST_Pos              (0)                                               /*!< ALC CTL: NGTHBST Position              */
#define ALC_CTL_NGTHBST_Msk              (0x7ul << ALC_CTL_NGTHBST_Pos)                    /*!< ALC CTL: NGTHBST Mask                  */

#define ALC_CTL_NGEN_Pos                 (3)                                               /*!< ALC CTL: NGEN Position                 */
#define ALC_CTL_NGEN_Msk                 (0x1ul << ALC_CTL_NGEN_Pos)                       /*!< ALC CTL: NGEN Mask                     */

#define ALC_CTL_ATKSEL_Pos               (4)                                               /*!< ALC CTL: ATKSEL Position               */
#define ALC_CTL_ATKSEL_Msk               (0xful << ALC_CTL_ATKSEL_Pos)                     /*!< ALC CTL: ATKSEL Mask                   */

#define ALC_CTL_DECAYSEL_Pos             (8)                                               /*!< ALC CTL: DECAYSEL Position             */
#define ALC_CTL_DECAYSEL_Msk             (0xful << ALC_CTL_DECAYSEL_Pos)                   /*!< ALC CTL: DECAYSEL Mask                 */

#define ALC_CTL_MODESEL_Pos              (12)                                              /*!< ALC CTL: MODESEL Position              */
#define ALC_CTL_MODESEL_Msk              (0x1ul << ALC_CTL_MODESEL_Pos)                    /*!< ALC CTL: MODESEL Mask                  */

#define ALC_CTL_TARGETLV_Pos             (13)                                              /*!< ALC CTL: TARGETLV Position             */
#define ALC_CTL_TARGETLV_Msk             (0xful << ALC_CTL_TARGETLV_Pos)                   /*!< ALC CTL: TARGETLV Mask                 */

#define ALC_CTL_HOLDTIME_Pos             (17)                                              /*!< ALC CTL: HOLDTIME Position             */
#define ALC_CTL_HOLDTIME_Msk             (0xful << ALC_CTL_HOLDTIME_Pos)                   /*!< ALC CTL: HOLDTIME Mask                 */

#define ALC_CTL_ZCEN_Pos                 (21)                                              /*!< ALC CTL: ZCEN Position                 */
#define ALC_CTL_ZCEN_Msk                 (0x1ul << ALC_CTL_ZCEN_Pos)                       /*!< ALC CTL: ZCEN Mask                     */

#define ALC_CTL_MINGAIN_Pos              (22)                                              /*!< ALC CTL: MINGAIN Position              */
#define ALC_CTL_MINGAIN_Msk              (0x7ul << ALC_CTL_MINGAIN_Pos)                    /*!< ALC CTL: MINGAIN Mask                  */

#define ALC_CTL_MAXGAIN_Pos              (25)                                              /*!< ALC CTL: MAXGAIN Position              */
#define ALC_CTL_MAXGAIN_Msk              (0x7ul << ALC_CTL_MAXGAIN_Pos)                    /*!< ALC CTL: MAXGAIN Mask                  */

#define ALC_CTL_ALCEN_Pos                (28)                                              /*!< ALC CTL: ALCEN Position                */
#define ALC_CTL_ALCEN_Msk                (0x1ul << ALC_CTL_ALCEN_Pos)                      /*!< ALC CTL: ALCEN Mask                    */

#define ALC_CTL_NGPKSEL_Pos              (29)                                              /*!< ALC CTL: NGPKSEL Position              */
#define ALC_CTL_NGPKSEL_Msk              (0x1ul << ALC_CTL_NGPKSEL_Pos)                    /*!< ALC CTL: NGPKSEL Mask                  */

#define ALC_CTL_PKSEL_Pos                (30)                                              /*!< ALC CTL: PKSEL Position                */
#define ALC_CTL_PKSEL_Msk                (0x1ul << ALC_CTL_PKSEL_Pos)                      /*!< ALC CTL: PKSEL Mask                    */

#define ALC_CTL_PKLIMEN_Pos              (31)                                              /*!< ALC CTL: PKLIMEN Position              */
#define ALC_CTL_PKLIMEN_Msk              (0x1ul << ALC_CTL_PKLIMEN_Pos)                    /*!< ALC CTL: PKLIMEN Mask                  */

#define ALC_STS_CLIPFLAG_Pos             (0)                                               /*!< ALC STS: CLIPFLAG Position             */
#define ALC_STS_CLIPFLAG_Msk             (0x1ul << ALC_STS_CLIPFLAG_Pos)                   /*!< ALC STS: CLIPFLAG Mask                 */

#define ALC_STS_NOISEF_Pos               (1)                                               /*!< ALC STS: NOISEF Position               */
#define ALC_STS_NOISEF_Msk               (0x1ul << ALC_STS_NOISEF_Pos)                     /*!< ALC STS: NOISEF Mask                   */

#define ALC_STS_P2PVAL_Pos               (2)                                               /*!< ALC STS: P2PVAL Position               */
#define ALC_STS_P2PVAL_Msk               (0x1fful << ALC_STS_P2PVAL_Pos)                   /*!< ALC STS: P2PVAL Mask                   */

#define ALC_STS_PEAKVAL_Pos              (11)                                              /*!< ALC STS: PEAKVAL Position              */
#define ALC_STS_PEAKVAL_Msk              (0xfful << ALC_STS_PEAKVAL_Pos)                   /*!< ALC STS: PEAKVAL Mask                  */

#define ALC_INTSTS_INTFLAG_Pos           (0)                                               /*!< ALC INTSTS: INTFLAG Position           */
#define ALC_INTSTS_INTFLAG_Msk           (0x1ul << ALC_INTSTS_INTFLAG_Pos)                 /*!< ALC INTSTS: INTFLAG Mask               */

#define ALC_INTCTL_INTEN_Pos             (0)                                               /*!< ALC INTCTL: INTEN Position             */
#define ALC_INTCTL_INTEN_Msk             (0x1ul << ALC_INTCTL_INTEN_Pos)                   /*!< ALC INTCTL: INTEN Mask                 */

/**@}*/ /* ALC_CONST */
/**@}*/ /* end of ALC register group */


/*---------------------- Analog Functional Blocks -------------------------*/
/**
    @addtogroup ANA Analog Functional Blocks(ANA)
    Memory Mapped Structure for ANA Controller
@{ */
 
typedef struct
{


    /**
     * VMID
     * ===================================================================================================
     * Offset: 0x00  VMID Reference Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PULLDOWN  |VMID Pulldown
     * |        |          |0= Release VMID pin for reference operation.
     * |        |          |1= Pull VMID pin to ground. Default power down and reset condition.
     * |[1]     |PDLORES   |Power Down Low (4.8kOhm) Resistance Reference
     * |        |          |0= Connect the Low Resistance reference to VMID.
     * |        |          |Use this setting for fast power up of VMID.
     * |        |          |Can be turned off after 50ms to save power.
     * |        |          |1= The Low Resistance reference is disconnected from VMID. Default power down and reset condition.
     * |[2]     |PDHIRES   |Power Down High (360kOhm) Resistance Reference
     * |        |          |0= Connect the High Resistance reference to VMID. Use this setting for minimum power consumption.
     * |        |          |1= The High Resistance reference is disconnected from VMID. Default power down and reset condition.
 */
    __IO uint32_t VMID;                  
         uint32_t RESERVE0[1];


    /**
     * CURCTL0
     * ===================================================================================================
     * Offset: 0x08  Current Source Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |CURSRCEN  |Enable Current Source to GPIOB[x]
     * |        |          |Individually enable current source to GPIOB pins. Each GPIOB pin has a separate current source.
     * |        |          |0 = Disable
     * |        |          |1 = Enable current source to pin GPIOB[x]
     * |[8:9]   |VALSEL    |Current Source Value
     * |        |          |Select master current for source generation
     * |        |          |0= 0.5 uA
     * |        |          |1= 1 uA
     * |        |          |2= 2.5 uA
     * |        |          |3= 5 uA
 */
    __IO uint32_t CURCTL0;               
         uint32_t RESERVE1[5];


    /**
     * LDOSEL
     * ===================================================================================================
     * Offset: 0x20  LDO Voltage Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |LDOSEL    |Select LDO Output Voltage
     * |        |          |Note that maximum I/O pad operation speed only specified for voltage >2.4V.
     * |        |          |0= 3.0V
     * |        |          |1= 1.8V
     * |        |          |2= 2.4V
     * |        |          |3= 3.3V
 */
    __IO uint32_t LDOSEL;                

    /**
     * LDOPD
     * ===================================================================================================
     * Offset: 0x24  LDO Power Down Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PD        |Power Down LDO
     * |        |          |When powered down no current delivered to VD33.
     * |        |          |0= Enable LDO
     * |        |          |1= Power Down.
     * |[1]     |DISCHAR   |Discharge
     * |        |          |0 = No load on VD33
     * |        |          |1 = Switch discharge resistor to VD33.
 */
    __IO uint32_t LDOPD;                 

    /**
     * MICBSEL
     * ===================================================================================================
     * Offset: 0x28  Microphone Bias Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |VOLSEL    |Select Microphone Bias Voltage
     * |        |          |MICBMODE = 0
     * |        |          |0: 90% VCCA
     * |        |          |1: 65% VCCA
     * |        |          |2: 75% VCCA
     * |        |          |3: 50% VCCA
     * |        |          |MICBMODE = 1
     * |        |          |0: 2.4V
     * |        |          |1: 1.7V
     * |        |          |2: 2.0V
     * |        |          |3: 1.3V
     * |[2]     |REFSEL    |Select Reference Source For MICBIAS Generator
     * |        |          |VMID provides superior noise performance for MICBIAS generation and should be used unless fixed voltage is absolutely necessary, then noise performance can be sacrificed and bandgap voltage used as reference.
     * |        |          |0= VMID = VCCA/2 is reference source.
     * |        |          |1= VBG (bandgap voltage reference) is reference source.
 */
    __IO uint32_t MICBSEL;               

    /**
     * MICBEN
     * ===================================================================================================
     * Offset: 0x2C  Microphone Bias Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |MICBEN    |Enable Microphone Bias Generator
     * |        |          |0 = Powered Down.
     * |        |          |1 = Enabled.
 */
    __IO uint32_t MICBEN;                
         uint32_t RESERVE2[8];


    /**
     * MUXCTL
     * ===================================================================================================
     * Offset: 0x50  Analog Multiplexer Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |NEGINSEL  |Selects Connection Of GPIOB[7:0] To PGA_INN, Negative Input Of PGA
     * |        |          |If NEGINSEL[n] = 1 then GPIOB[n] is connected to PGA_INN.
     * |[8:11]  |POSINSEL  |Selects Connection Of GPIOB[7,5,3,1] To PGA_INP, Positive Input Of PGA
     * |        |          |1000b: GPIOB[7] connected to PGA_INP
     * |        |          |0100b: GPIOB[5] connected to PGA_INP
     * |        |          |0010b: GPIOB[3] connected to PGA_INP
     * |        |          |0001b: GPIOB[1] connected to PGA_INP
     * |[12]    |PTATCUR   |Select PTAT Current
     * |        |          |I_PTAT, to PGA_INN, negative input to PGA, for temperature measurement.
     * |[13]    |PGAINSEL  |Select MICP/MICN To PGA Inputs
     * |[14]    |MUXEN     |Enable The Analog Multiplexer
     * |        |          |0 = All channels disabled
     * |        |          |1 = Selection determined by register setting.
 */
    __IO uint32_t MUXCTL;                
         uint32_t RESERVE3[3];


    /**
     * PGACTL
     * ===================================================================================================
     * Offset: 0x60  PGA Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |REFSEL    |Select Reference For Analog Path
     * |        |          |Signal path is normally referenced to VMID (VCCA/2).
     * |        |          |To use an absolute reference this can be set to VBG = 1.2V.
     * |        |          |0 = Select VMID voltage as analog ground reference.
     * |        |          |1 = Select Bandgap voltage as analog ground reference. 
     * |[1]     |PUPGA     |Power Up Control For PGA Amplifier
     * |        |          |This amplifier must be powered up for signal path operation.
     * |        |          |0 = Power Down.
     * |        |          |1 = Power up.
     * |[2]     |PUBOOST   |Power Up Control For Boost Stage Amplifier
     * |        |          |This amplifier must be powered up for signal path operation.
     * |        |          |0 = Power Down.
     * |        |          |1 = Power up.
     * |[3]     |BSTGAIN   |Boost Stage Gain Setting
     * |        |          |0 = Gain = 0dB.
     * |        |          |1 = Gain = 26dB
 */
    __IO uint32_t PGACTL;                

    /**
     * SIGCTL
     * ===================================================================================================
     * Offset: 0x64  Signal Path Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PUZCDCMP  |Power Up And Enable Control For Zero Cross Detect Comparator
     * |        |          |When enabled PGA gain settings will only be updated when ADC input signal crosses zero signal threshold.
     * |        |          |To operate ZCD the ALC peripheral clock (CLK_APBCLK0.BFALCKEN) must also be enabled and BIQ_CTL.DLCOEFF = 1 to allow ZCD clocks to be generated.
     * |        |          |0 = Power down.
     * |        |          |1 = Power up and enable zero cross detection. 
     * |[1]     |PUBUFPGA  |Power Up Control For PGA Reference Buffer
     * |        |          |This block must be powered up for signal path operation.
     * |        |          |0 = Power down.
     * |        |          |1 = Power up.
     * |[2]     |PUBUFADC  |Power Up Control For ADC Reference Buffer
     * |        |          |This block must be powered up for signal path operation.
     * |        |          |0 = Power down.
     * |        |          |1 = Power up.
     * |[3]     |PUCURB    |Power Up Control For Current Bias Generation
     * |        |          |This block must be powered up for signal path operation.
     * |        |          |0 = Power down.
     * |        |          |1 = Power up.
     * |[4]     |PUADCOP   |Power Up ADC Sigma-Delta Modulator
     * |        |          |This block must be powered up for ADC operation.
     * |        |          |0 = Power down.
     * |        |          |1 = Power up.
     * |[5]     |MUTEPGA   |PGA Mute Control
     * |        |          |0 = Normal.
     * |        |          |1 = Signal Muted.
     * |[6]     |MUTEBST   |Boost Stage Mute Control
     * |        |          |0 = Normal.
     * |        |          |1 = Signal Muted. 
 */
    __IO uint32_t SIGCTL;                

    /**
     * PGAGAIN
     * ===================================================================================================
     * Offset: 0x68  PGA Gain Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:5]   |GAINSET   |Select The PGA Gain Setting
     * |        |          |From -12dB to 35.25dB in 0.75dB step size.
     * |        |          |0x00 is lowest gain setting at -12dB and 0x3F is largest gain at 35.25dB.
     * |[8:13]  |GAINREAD  |Current PGA Gain
     * |        |          |Read Only. May be different from GAIN register when AGC is enabled and is controlling the PGA gain.
 */
    __IO uint32_t PGAGAIN;               
         uint32_t RESERVE4[6];


    /**
     * TRIM
     * ===================================================================================================
     * Offset: 0x84  Oscillator Trim Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |OSCTRIM   |Oscillator Trim
     * |        |          |Reads current oscillator trim setting. Read Only.
     * |[8:15]  |COARSE    |COARSE
     * |        |          |Current coarse range setting of the oscillator. Read Only
     * |[16:23] |SUPERFINE |Superfine
     * |        |          |The superfine trim setting is an 8bit signed integer.
     * |        |          |It adjusts the master oscillator by dithering the FINE trim setting between the current setting and one setting above (values 1,127) or below (values -1, -128) the current trim setting.
     * |        |          |Each step effectively moves the frequency 1/128th of the full FINE trim step size.
 */
    __IO uint32_t TRIM;                  
         uint32_t RESERVE5[1];


    /**
     * CAPSCTL
     * ===================================================================================================
     * Offset: 0x8C  Capacitive Touch Sensing Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |LOWTIME   |Output Low Time
     * |        |          |Number of PCLK cycles to discharge external capacitor.
     * |        |          |0=1cycle
     * |        |          |1=2cycles
     * |        |          |2=8cycles
     * |        |          |3=16cycles
     * |[2:4]   |CYCLECNT  |Number of Relaxation Cycles
     * |        |          |Peripheral performs 2^(CYCLECNT) relaxation cycles before generating interrupt.
     * |[5]     |CLKMODE   |Reference Clock Mode
     * |        |          |0 = Capacitive Touch Sensing Mode.
     * |        |          |1 = Circuit is in Reference clock generation mode.
     * |[8:15]  |CLKDIV    |Reference Clock Divider
     * |        |          |Circuit can be used to generate a reference clock output of SDCLK/2/(CLKDIV+1) instead of a Capacitive Touch Sensing reset signal.
     * |[29]    |RSTCNT    |Reset Count
     * |        |          |0: Release/Activate ANA_CAPSCNT
     * |        |          |1: Set high to reset ANA_CAPSCNT.
     * |[30]    |INTEN     |Interrupt Enable
     * |        |          |0 = Disable/Reset CAPS_IRQ interrupt.
     * |        |          |1 = Enable CAPS_IRQ interrupt.
     * |[31]    |CAPSEN    |Enable
     * |        |          |0 = Disable/Reset block.
     * |        |          |1 = Enable Block.
 */
    __IO uint32_t CAPSCTL;               

    /**
     * CAPSCNT
     * ===================================================================================================
     * Offset: 0x90  Capacitive Touch Sensing Count Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:23]  |CAPSCNT   |Counter Read Back Value Of Capacitive Touch Sensing Block
 */
    __I  uint32_t CAPSCNT;               

    /**
     * FQMMCTL
     * ===================================================================================================
     * Offset: 0x94  Frequency Measurement Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |CLKSEL    |Reference Clock Source
     * |        |          |00b: OSC16K,
     * |        |          |01b: OSC32K (default),
     * |        |          |1xb: I2S_WS - can be GPIOA[4,8,12] according to SYS_GPA_MFP register, configure I2S in SLAVE mode to enable.
     * |[2]     |MMSTS     |Measurement Done
     * |        |          |0 = Measurement Ongoing.
     * |        |          |1 = Measurement Complete.
     * |[16:23] |CYCLESEL  |Frequency Measurement Cycles
     * |        |          |Number of reference clock periods plus one to measure target clock (PCLK).
     * |        |          |For example if reference clock is OSC32K (T is 30.5175us), set CYCLESEL to 7, then measurement period would be 30.5175*(7+1), 244.1us.
     * |[31]    |FQMMEN    |FQMMEN
     * |        |          |0 = Disable/Reset block.
     * |        |          |1 = Start Frequency Measurement.
 */
    __IO uint32_t FQMMCTL;               

    /**
     * FQMMCNT
     * ===================================================================================================
     * Offset: 0x98  Frequency Measurement Count Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |FQMMCNT   |Frequency Measurement Count
     * |        |          |When MMSTS = 1 and G0 = 1, this is number of PCLK periods counted for frequency measurement.
     * |        |          |The frequency will be PCLK = FQMMCNT * Fref /(CYCLESEL+1) Hz
     * |        |          |Maximum resolution of measurement is Fref /(CYCLESEL+1)*2 Hz
 */
    __I  uint32_t FQMMCNT;               

} ANA_T;

/**
    @addtogroup ANA_CONST ANA Bit Field Definition
    Constant Definitions for ANA Controller
@{ */

#define ANA_VMID_PULLDOWN_Pos            (0)                                               /*!< ANA VMID: PULLDOWN Position            */
#define ANA_VMID_PULLDOWN_Msk            (0x1ul << ANA_VMID_PULLDOWN_Pos)                  /*!< ANA VMID: PULLDOWN Mask                */

#define ANA_VMID_PDLORES_Pos             (1)                                               /*!< ANA VMID: PDLORES Position             */
#define ANA_VMID_PDLORES_Msk             (0x1ul << ANA_VMID_PDLORES_Pos)                   /*!< ANA VMID: PDLORES Mask                 */

#define ANA_VMID_PDHIRES_Pos             (2)                                               /*!< ANA VMID: PDHIRES Position             */
#define ANA_VMID_PDHIRES_Msk             (0x1ul << ANA_VMID_PDHIRES_Pos)                   /*!< ANA VMID: PDHIRES Mask                 */

#define ANA_CURCTL0_CURSRCEN_Pos         (0)                                               /*!< ANA CURCTL0: CURSRCEN Position         */
#define ANA_CURCTL0_CURSRCEN_Msk         (0xfful << ANA_CURCTL0_CURSRCEN_Pos)              /*!< ANA CURCTL0: CURSRCEN Mask             */

#define ANA_CURCTL0_VALSEL_Pos           (8)                                               /*!< ANA CURCTL0: VALSEL Position           */
#define ANA_CURCTL0_VALSEL_Msk           (0x3ul << ANA_CURCTL0_VALSEL_Pos)                 /*!< ANA CURCTL0: VALSEL Mask               */

#define ANA_LDOSEL_LDOSEL_Pos            (0)                                               /*!< ANA LDOSEL: LDOSEL Position            */
#define ANA_LDOSEL_LDOSEL_Msk            (0x3ul << ANA_LDOSEL_LDOSEL_Pos)                  /*!< ANA LDOSEL: LDOSEL Mask                */

#define ANA_LDOPD_PD_Pos                 (0)                                               /*!< ANA LDOPD: PD Position                 */
#define ANA_LDOPD_PD_Msk                 (0x1ul << ANA_LDOPD_PD_Pos)                       /*!< ANA LDOPD: PD Mask                     */

#define ANA_LDOPD_DISCHAR_Pos            (1)                                               /*!< ANA LDOPD: DISCHAR Position            */
#define ANA_LDOPD_DISCHAR_Msk            (0x1ul << ANA_LDOPD_DISCHAR_Pos)                  /*!< ANA LDOPD: DISCHAR Mask                */

#define ANA_MICBSEL_VOLSEL_Pos           (0)                                               /*!< ANA MICBSEL: VOLSEL Position           */
#define ANA_MICBSEL_VOLSEL_Msk           (0x3ul << ANA_MICBSEL_VOLSEL_Pos)                 /*!< ANA MICBSEL: VOLSEL Mask               */

#define ANA_MICBSEL_REFSEL_Pos           (2)                                               /*!< ANA MICBSEL: REFSEL Position           */
#define ANA_MICBSEL_REFSEL_Msk           (0x1ul << ANA_MICBSEL_REFSEL_Pos)                 /*!< ANA MICBSEL: REFSEL Mask               */

#define ANA_MICBEN_MICBEN_Pos            (0)                                               /*!< ANA MICBEN: MICBEN Position            */
#define ANA_MICBEN_MICBEN_Msk            (0x1ul << ANA_MICBEN_MICBEN_Pos)                  /*!< ANA MICBEN: MICBEN Mask                */

#define ANA_MUXCTL_NEGINSEL_Pos          (0)                                               /*!< ANA MUXCTL: NEGINSEL Position          */
#define ANA_MUXCTL_NEGINSEL_Msk          (0xfful << ANA_MUXCTL_NEGINSEL_Pos)               /*!< ANA MUXCTL: NEGINSEL Mask              */

#define ANA_MUXCTL_POSINSEL_Pos          (8)                                               /*!< ANA MUXCTL: POSINSEL Position          */
#define ANA_MUXCTL_POSINSEL_Msk          (0xful << ANA_MUXCTL_POSINSEL_Pos)                /*!< ANA MUXCTL: POSINSEL Mask              */

#define ANA_MUXCTL_PTATCUR_Pos           (12)                                              /*!< ANA MUXCTL: PTATCUR Position           */
#define ANA_MUXCTL_PTATCUR_Msk           (0x1ul << ANA_MUXCTL_PTATCUR_Pos)                 /*!< ANA MUXCTL: PTATCUR Mask               */

#define ANA_MUXCTL_PGAINSEL_Pos          (13)                                              /*!< ANA MUXCTL: PGAINSEL Position          */
#define ANA_MUXCTL_PGAINSEL_Msk          (0x1ul << ANA_MUXCTL_PGAINSEL_Pos)                /*!< ANA MUXCTL: PGAINSEL Mask              */

#define ANA_MUXCTL_MUXEN_Pos             (14)                                              /*!< ANA MUXCTL: MUXEN Position             */
#define ANA_MUXCTL_MUXEN_Msk             (0x1ul << ANA_MUXCTL_MUXEN_Pos)                   /*!< ANA MUXCTL: MUXEN Mask                 */

#define ANA_PGACTL_REFSEL_Pos            (0)                                               /*!< ANA PGACTL: REFSEL Position            */
#define ANA_PGACTL_REFSEL_Msk            (0x1ul << ANA_PGACTL_REFSEL_Pos)                  /*!< ANA PGACTL: REFSEL Mask                */

#define ANA_PGACTL_PUPGA_Pos             (1)                                               /*!< ANA PGACTL: PUPGA Position             */
#define ANA_PGACTL_PUPGA_Msk             (0x1ul << ANA_PGACTL_PUPGA_Pos)                   /*!< ANA PGACTL: PUPGA Mask                 */

#define ANA_PGACTL_PUBOOST_Pos           (2)                                               /*!< ANA PGACTL: PUBOOST Position           */
#define ANA_PGACTL_PUBOOST_Msk           (0x1ul << ANA_PGACTL_PUBOOST_Pos)                 /*!< ANA PGACTL: PUBOOST Mask               */

#define ANA_PGACTL_BSTGAIN_Pos           (3)                                               /*!< ANA PGACTL: BSTGAIN Position           */
#define ANA_PGACTL_BSTGAIN_Msk           (0x1ul << ANA_PGACTL_BSTGAIN_Pos)                 /*!< ANA PGACTL: BSTGAIN Mask               */

#define ANA_SIGCTL_PUZCDCMP_Pos          (0)                                               /*!< ANA SIGCTL: PUZCDCMP Position          */
#define ANA_SIGCTL_PUZCDCMP_Msk          (0x1ul << ANA_SIGCTL_PUZCDCMP_Pos)                /*!< ANA SIGCTL: PUZCDCMP Mask              */

#define ANA_SIGCTL_PUBUFPGA_Pos          (1)                                               /*!< ANA SIGCTL: PUBUFPGA Position          */
#define ANA_SIGCTL_PUBUFPGA_Msk          (0x1ul << ANA_SIGCTL_PUBUFPGA_Pos)                /*!< ANA SIGCTL: PUBUFPGA Mask              */

#define ANA_SIGCTL_PUBUFADC_Pos          (2)                                               /*!< ANA SIGCTL: PUBUFADC Position          */
#define ANA_SIGCTL_PUBUFADC_Msk          (0x1ul << ANA_SIGCTL_PUBUFADC_Pos)                /*!< ANA SIGCTL: PUBUFADC Mask              */

#define ANA_SIGCTL_PUCURB_Pos            (3)                                               /*!< ANA SIGCTL: PUCURB Position            */
#define ANA_SIGCTL_PUCURB_Msk            (0x1ul << ANA_SIGCTL_PUCURB_Pos)                  /*!< ANA SIGCTL: PUCURB Mask                */

#define ANA_SIGCTL_PUADCOP_Pos           (4)                                               /*!< ANA SIGCTL: PUADCOP Position           */
#define ANA_SIGCTL_PUADCOP_Msk           (0x1ul << ANA_SIGCTL_PUADCOP_Pos)                 /*!< ANA SIGCTL: PUADCOP Mask               */

#define ANA_SIGCTL_MUTEPGA_Pos           (5)                                               /*!< ANA SIGCTL: MUTEPGA Position           */
#define ANA_SIGCTL_MUTEPGA_Msk           (0x1ul << ANA_SIGCTL_MUTEPGA_Pos)                 /*!< ANA SIGCTL: MUTEPGA Mask               */

#define ANA_SIGCTL_MUTEBST_Pos           (6)                                               /*!< ANA SIGCTL: MUTEBST Position           */
#define ANA_SIGCTL_MUTEBST_Msk           (0x1ul << ANA_SIGCTL_MUTEBST_Pos)                 /*!< ANA SIGCTL: MUTEBST Mask               */

#define ANA_PGAGAIN_GAINSET_Pos          (0)                                               /*!< ANA PGAGAIN: GAINSET Position          */
#define ANA_PGAGAIN_GAINSET_Msk          (0x3ful << ANA_PGAGAIN_GAINSET_Pos)               /*!< ANA PGAGAIN: GAINSET Mask              */

#define ANA_PGAGAIN_GAINREAD_Pos         (8)                                               /*!< ANA PGAGAIN: GAINREAD Position         */
#define ANA_PGAGAIN_GAINREAD_Msk         (0x3ful << ANA_PGAGAIN_GAINREAD_Pos)              /*!< ANA PGAGAIN: GAINREAD Mask             */

#define ANA_TRIM_OSCTRIM_Pos             (0)                                               /*!< ANA TRIM: OSCTRIM Position             */
#define ANA_TRIM_OSCTRIM_Msk             (0xfful << ANA_TRIM_OSCTRIM_Pos)                  /*!< ANA TRIM: OSCTRIM Mask                 */

#define ANA_TRIM_COARSE_Pos              (8)                                               /*!< ANA TRIM: COARSE Position              */
#define ANA_TRIM_COARSE_Msk              (0xfful << ANA_TRIM_COARSE_Pos)                   /*!< ANA TRIM: COARSE Mask                  */

#define ANA_TRIM_SUPERFINE_Pos           (16)                                              /*!< ANA TRIM: SUPERFINE Position           */
#define ANA_TRIM_SUPERFINE_Msk           (0xfful << ANA_TRIM_SUPERFINE_Pos)                /*!< ANA TRIM: SUPERFINE Mask               */

#define ANA_CAPSCTL_LOWTIME_Pos          (0)                                               /*!< ANA CAPSCTL: LOWTIME Position          */
#define ANA_CAPSCTL_LOWTIME_Msk          (0x3ul << ANA_CAPSCTL_LOWTIME_Pos)                /*!< ANA CAPSCTL: LOWTIME Mask              */

#define ANA_CAPSCTL_CYCLECNT_Pos         (2)                                               /*!< ANA CAPSCTL: CYCLECNT Position         */
#define ANA_CAPSCTL_CYCLECNT_Msk         (0x7ul << ANA_CAPSCTL_CYCLECNT_Pos)               /*!< ANA CAPSCTL: CYCLECNT Mask             */

#define ANA_CAPSCTL_CLKMODE_Pos          (5)                                               /*!< ANA CAPSCTL: CLKMODE Position          */
#define ANA_CAPSCTL_CLKMODE_Msk          (0x1ul << ANA_CAPSCTL_CLKMODE_Pos)                /*!< ANA CAPSCTL: CLKMODE Mask              */

#define ANA_CAPSCTL_CLKDIV_Pos           (8)                                               /*!< ANA CAPSCTL: CLKDIV Position           */
#define ANA_CAPSCTL_CLKDIV_Msk           (0xfful << ANA_CAPSCTL_CLKDIV_Pos)                /*!< ANA CAPSCTL: CLKDIV Mask               */

#define ANA_CAPSCTL_RSTCNT_Pos           (29)                                              /*!< ANA CAPSCTL: RSTCNT Position           */
#define ANA_CAPSCTL_RSTCNT_Msk           (0x1ul << ANA_CAPSCTL_RSTCNT_Pos)                 /*!< ANA CAPSCTL: RSTCNT Mask               */

#define ANA_CAPSCTL_INTEN_Pos            (30)                                              /*!< ANA CAPSCTL: INTEN Position            */
#define ANA_CAPSCTL_INTEN_Msk            (0x1ul << ANA_CAPSCTL_INTEN_Pos)                  /*!< ANA CAPSCTL: INTEN Mask                */

#define ANA_CAPSCTL_CAPSEN_Pos           (31)                                              /*!< ANA CAPSCTL: CAPSEN Position           */
#define ANA_CAPSCTL_CAPSEN_Msk           (0x1ul << ANA_CAPSCTL_CAPSEN_Pos)                 /*!< ANA CAPSCTL: CAPSEN Mask               */

#define ANA_CAPSCNT_CAPSCNT_Pos          (0)                                               /*!< ANA CAPSCNT: CAPSCNT Position          */
#define ANA_CAPSCNT_CAPSCNT_Msk          (0xfffffful << ANA_CAPSCNT_CAPSCNT_Pos)           /*!< ANA CAPSCNT: CAPSCNT Mask              */

#define ANA_FQMMCTL_CLKSEL_Pos           (0)                                               /*!< ANA FQMMCTL: CLKSEL Position           */
#define ANA_FQMMCTL_CLKSEL_Msk           (0x3ul << ANA_FQMMCTL_CLKSEL_Pos)                 /*!< ANA FQMMCTL: CLKSEL Mask               */

#define ANA_FQMMCTL_MMSTS_Pos            (2)                                               /*!< ANA FQMMCTL: MMSTS Position            */
#define ANA_FQMMCTL_MMSTS_Msk            (0x1ul << ANA_FQMMCTL_MMSTS_Pos)                  /*!< ANA FQMMCTL: MMSTS Mask                */

#define ANA_FQMMCTL_CYCLESEL_Pos         (16)                                              /*!< ANA FQMMCTL: CYCLESEL Position         */
#define ANA_FQMMCTL_CYCLESEL_Msk         (0xfful << ANA_FQMMCTL_CYCLESEL_Pos)              /*!< ANA FQMMCTL: CYCLESEL Mask             */

#define ANA_FQMMCTL_FQMMEN_Pos           (31)                                              /*!< ANA FQMMCTL: FQMMEN Position           */
#define ANA_FQMMCTL_FQMMEN_Msk           (0x1ul << ANA_FQMMCTL_FQMMEN_Pos)                 /*!< ANA FQMMCTL: FQMMEN Mask               */

#define ANA_FQMMCNT_FQMMCNT_Pos          (0)                                               /*!< ANA FQMMCNT: FQMMCNT Position          */
#define ANA_FQMMCNT_FQMMCNT_Msk          (0xfffful << ANA_FQMMCNT_FQMMCNT_Pos)             /*!< ANA FQMMCNT: FQMMCNT Mask              */

/**@}*/ /* ANA_CONST */
/**@}*/ /* end of ANA register group */


/*---------------------- Biquad Filter -------------------------*/
/**
    @addtogroup BIQ Biquad Filter(BIQ)
    Memory Mapped Structure for BIQ Controller
@{ */
 
typedef struct
{


    /**
     * COEFF0
     * ===================================================================================================
     * Offset: 0x00  Coefficient b0 In H(z) Transfer Function
(3.16 format) - 1st stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF0;                

    /**
     * COEFF1
     * ===================================================================================================
     * Offset: 0x04  Coefficient b1 In H(z) Transfer Function
(3.16 format) - 1st stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF1;                

    /**
     * COEFF2
     * ===================================================================================================
     * Offset: 0x08  Coefficient b2 In H(z) Transfer Function
(3.16 format) - 1st stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF2;                

    /**
     * COEFF3
     * ===================================================================================================
     * Offset: 0x0C  Coefficient a1 In H(z) Transfer Function
(3.16 format) - 1st stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF3;                

    /**
     * COEFF4
     * ===================================================================================================
     * Offset: 0x10  Coefficient a2 In H(z) Transfer Function
(3.16 format) - 1st stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF4;                

    /**
     * COEFF5
     * ===================================================================================================
     * Offset: 0x14  Coefficient b0 In H(z) Transfer Function
(3.16 format) - 2nd stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF5;                

    /**
     * COEFF6
     * ===================================================================================================
     * Offset: 0x18  Coefficient b1 In H(z) Transfer Function
(3.16 format) - 2nd stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF6;                

    /**
     * COEFF7
     * ===================================================================================================
     * Offset: 0x1C  Coefficient b2 In H(z) Transfer Function
(3.16 format) - 2nd stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF7;                

    /**
     * COEFF8
     * ===================================================================================================
     * Offset: 0x20  Coefficient a1 In H(z) Transfer Function
(3.16 format) - 2nd stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF8;                

    /**
     * COEFF9
     * ===================================================================================================
     * Offset: 0x24  Coefficient a2 In H(z) Transfer Function
(3.16 format) - 2nd stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF9;                

    /**
     * COEFF10
     * ===================================================================================================
     * Offset: 0x28  Coefficient b0 In H(z) Transfer Function
(3.16 format) - 3rd stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF10;               

    /**
     * COEFF11
     * ===================================================================================================
     * Offset: 0x2C  Coefficient b1 In H(z) Transfer Function
(3.16 format) - 3rd stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF11;               

    /**
     * COEFF12
     * ===================================================================================================
     * Offset: 0x30  Coefficient b2 In H(z) Transfer Function
(3.16 format) - 3rd stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF12;               

    /**
     * COEFF13
     * ===================================================================================================
     * Offset: 0x34  Coefficient a1 In H(z) Transfer Function
(3.16 format) - 3rd stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF13;               

    /**
     * COEFF14
     * ===================================================================================================
     * Offset: 0x38  Coefficient a2 In H(z) Transfer Function
(3.16 format) - 3rd stage BIQ Coefficients
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |COEFFDAT  |Coefficient Data
 */
    __IO uint32_t COEFF14;               
         uint32_t RESERVE0[1];


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x40  BIQ Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |BIQEN     |BIQ Filter Start To Run
     * |        |          |0 = BIQ filter is not processing
     * |        |          |1 = BIQ filter is on.
     * |[1]     |PATHSEL   |AC Path Selection For BIQ
     * |        |          |0 = used in ADC path
     * |        |          |1 = used in DPWM path
     * |[2]     |PRGCOEFF  |Programming Mode Coefficient Control Bit
     * |        |          |0 = Coefficient RAM is in normal mode.
     * |        |          |1 = coefficient RAM is under programming mode.
     * |        |          |This bit must be turned off when BIQEN in on.
     * |[3]     |DLCOEFF   |Move BIQ Out Of Reset State
     * |        |          |0 = BIQ filter is in reset state.
     * |        |          |1 = When this bit is on, the default coefficients will be downloaded to the coefficient ram automatically in 32 internal system clocks.
     * |        |          |Processor must delay enough time before changing the coefficients or turn the BIQ on.
     * |[4:6]   |DPWMPUSR  |DPWM Path Up Sample Rate (From SRDIV Result)
     * |        |          |This register is only used when PATHSEL is set to 1.
     * |        |          |The operating sample rate for the biquad filter will be.
     * |        |          |(DPWMPUSR+1)*HCLK/(SRDIV+1).
     * |        |          |Default value for this register is 3.
     * |[16:28] |SRDIV     |Sample Rate Divider
     * |        |          |This register is used to program the operating sampling rate of the biquad filter.
     * |        |          |The sample rate is defined as.
     * |        |          |HCLK/(SRDIV+1).
     * |        |          |Default to 3071 so the sampling rate is 16K when HCLK is 49.152MHz.
 */
    __IO uint32_t CTL;                   

} BIQ_T;

/**
    @addtogroup BIQ_CONST BIQ Bit Field Definition
    Constant Definitions for BIQ Controller
@{ */

#define BIQ_COEFF0_COEFFDAT_Pos          (0)                                               /*!< BIQ COEFF0: COEFFDAT Position          */
#define BIQ_COEFF0_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF0_COEFFDAT_Pos)         /*!< BIQ COEFF0: COEFFDAT Mask              */

#define BIQ_COEFF1_COEFFDAT_Pos          (0)                                               /*!< BIQ COEFF1: COEFFDAT Position          */
#define BIQ_COEFF1_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF1_COEFFDAT_Pos)         /*!< BIQ COEFF1: COEFFDAT Mask              */

#define BIQ_COEFF2_COEFFDAT_Pos          (0)                                               /*!< BIQ COEFF2: COEFFDAT Position          */
#define BIQ_COEFF2_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF2_COEFFDAT_Pos)         /*!< BIQ COEFF2: COEFFDAT Mask              */

#define BIQ_COEFF3_COEFFDAT_Pos          (0)                                               /*!< BIQ COEFF3: COEFFDAT Position          */
#define BIQ_COEFF3_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF3_COEFFDAT_Pos)         /*!< BIQ COEFF3: COEFFDAT Mask              */

#define BIQ_COEFF4_COEFFDAT_Pos          (0)                                               /*!< BIQ COEFF4: COEFFDAT Position          */
#define BIQ_COEFF4_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF4_COEFFDAT_Pos)         /*!< BIQ COEFF4: COEFFDAT Mask              */

#define BIQ_COEFF5_COEFFDAT_Pos          (0)                                               /*!< BIQ COEFF5: COEFFDAT Position          */
#define BIQ_COEFF5_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF5_COEFFDAT_Pos)         /*!< BIQ COEFF5: COEFFDAT Mask              */

#define BIQ_COEFF6_COEFFDAT_Pos          (0)                                               /*!< BIQ COEFF6: COEFFDAT Position          */
#define BIQ_COEFF6_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF6_COEFFDAT_Pos)         /*!< BIQ COEFF6: COEFFDAT Mask              */

#define BIQ_COEFF7_COEFFDAT_Pos          (0)                                               /*!< BIQ COEFF7: COEFFDAT Position          */
#define BIQ_COEFF7_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF7_COEFFDAT_Pos)         /*!< BIQ COEFF7: COEFFDAT Mask              */

#define BIQ_COEFF8_COEFFDAT_Pos          (0)                                               /*!< BIQ COEFF8: COEFFDAT Position          */
#define BIQ_COEFF8_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF8_COEFFDAT_Pos)         /*!< BIQ COEFF8: COEFFDAT Mask              */

#define BIQ_COEFF9_COEFFDAT_Pos          (0)                                               /*!< BIQ COEFF9: COEFFDAT Position          */
#define BIQ_COEFF9_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF9_COEFFDAT_Pos)         /*!< BIQ COEFF9: COEFFDAT Mask              */

#define BIQ_COEFF10_COEFFDAT_Pos         (0)                                               /*!< BIQ COEFF10: COEFFDAT Position         */
#define BIQ_COEFF10_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF10_COEFFDAT_Pos)        /*!< BIQ COEFF10: COEFFDAT Mask             */

#define BIQ_COEFF11_COEFFDAT_Pos         (0)                                               /*!< BIQ COEFF11: COEFFDAT Position         */
#define BIQ_COEFF11_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF11_COEFFDAT_Pos)        /*!< BIQ COEFF11: COEFFDAT Mask             */

#define BIQ_COEFF12_COEFFDAT_Pos         (0)                                               /*!< BIQ COEFF12: COEFFDAT Position         */
#define BIQ_COEFF12_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF12_COEFFDAT_Pos)        /*!< BIQ COEFF12: COEFFDAT Mask             */

#define BIQ_COEFF13_COEFFDAT_Pos         (0)                                               /*!< BIQ COEFF13: COEFFDAT Position         */
#define BIQ_COEFF13_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF13_COEFFDAT_Pos)        /*!< BIQ COEFF13: COEFFDAT Mask             */

#define BIQ_COEFF14_COEFFDAT_Pos         (0)                                               /*!< BIQ COEFF14: COEFFDAT Position         */
#define BIQ_COEFF14_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF14_COEFFDAT_Pos)        /*!< BIQ COEFF14: COEFFDAT Mask             */

#define BIQ_CTL_BIQEN_Pos                (0)                                               /*!< BIQ CTL: BIQEN Position                */
#define BIQ_CTL_BIQEN_Msk                (0x1ul << BIQ_CTL_BIQEN_Pos)                      /*!< BIQ CTL: BIQEN Mask                    */

#define BIQ_CTL_PATHSEL_Pos              (1)                                               /*!< BIQ CTL: PATHSEL Position              */
#define BIQ_CTL_PATHSEL_Msk              (0x1ul << BIQ_CTL_PATHSEL_Pos)                    /*!< BIQ CTL: PATHSEL Mask                  */

#define BIQ_CTL_PRGCOEFF_Pos             (2)                                               /*!< BIQ CTL: PRGCOEFF Position             */
#define BIQ_CTL_PRGCOEFF_Msk             (0x1ul << BIQ_CTL_PRGCOEFF_Pos)                   /*!< BIQ CTL: PRGCOEFF Mask                 */

#define BIQ_CTL_DLCOEFF_Pos              (3)                                               /*!< BIQ CTL: DLCOEFF Position              */
#define BIQ_CTL_DLCOEFF_Msk              (0x1ul << BIQ_CTL_DLCOEFF_Pos)                    /*!< BIQ CTL: DLCOEFF Mask                  */

#define BIQ_CTL_DPWMPUSR_Pos             (4)                                               /*!< BIQ CTL: DPWMPUSR Position             */
#define BIQ_CTL_DPWMPUSR_Msk             (0x7ul << BIQ_CTL_DPWMPUSR_Pos)                   /*!< BIQ CTL: DPWMPUSR Mask                 */

#define BIQ_CTL_SRDIV_Pos                (16)                                              /*!< BIQ CTL: SRDIV Position                */
#define BIQ_CTL_SRDIV_Msk                (0x1ffful << BIQ_CTL_SRDIV_Pos)                   /*!< BIQ CTL: SRDIV Mask                    */

/**@}*/ /* BIQ_CONST */
/**@}*/ /* end of BIQ register group */


/*---------------------- Brownout Detection and Temperature Alarm -------------------------*/
/**
    @addtogroup BODTALM Brownout Detection and Temperature Alarm(BODTALM)
    Memory Mapped Structure for BODTALM Controller
@{ */
 
typedef struct
{


    /**
     * BODSEL
     * ===================================================================================================
     * Offset: 0x00  Brown Out Detector Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:2]   |BODVL     |BOD Voltage Level
     * |        |          |111b = 4.6V
     * |        |          |110b = 3.0V
     * |        |          |101b = 2.8V
     * |        |          |100b = 2.625V
     * |        |          |011b = 2.5V
     * |        |          |010b = 2.4V
     * |        |          |001b = 2.2V
     * |        |          |000b = 2.1V
     * |[3]     |BODHYS    |BOD Hysteresis
     * |        |          |0= Hysteresis Disabled.
     * |        |          |1= Enable Hysteresis of BOD detection.
 */
    __IO uint32_t BODSEL;                

    /**
     * BODCTL
     * ===================================================================================================
     * Offset: 0x04  Brown Out Detector Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |BODEN     |BOD Enable
     * |        |          |1xb = Enable continuous BOD detection.
     * |        |          |01b = Enable time multiplexed BOD detection. See BODTALM_BODDTMR register.
     * |        |          |00b = Disable BOD Detection.
     * |[2]     |BODINTEN  |BOD Interrupt Enable
     * |        |          |0= Disable BOD Interrupt.
     * |        |          |1= Enable BOD Interrupt.
     * |[3]     |BODIF     |Current Status Of Interrupt
     * |        |          |Latched whenever a BOD event occurs and BODINTEN = 1. Write '1' to clear.
     * |[4]     |BODOUT    |Output of BOD Detection Block
     * |        |          |This signal can be monitored to determine the current state of the BOD comparator.
     * |        |          |Read '1' implies that VCC is less than BODVL.
 */
    __IO uint32_t BODCTL;                

    /**
     * TALMSEL
     * ===================================================================================================
     * Offset: 0x08  Temperature Alarm Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |TALMVL    |Temperature Alarm Sense Level
     * |        |          |0000:105C
     * |        |          |0001:115C
     * |        |          |0010:125C
     * |        |          |0100:135C
     * |        |          |1000:145C
 */
    __IO uint32_t TALMSEL;               

    /**
     * TALMCTL
     * ===================================================================================================
     * Offset: 0x0C  Temperature Alarm Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TALMEN    |TALARM Enable
     * |        |          |0 = Disable TALARM Detection
     * |        |          |1 = Enable TALARM Detection
     * |[1]     |TALMOUT   |Output of TALARM Block
     * |        |          |Can be polled to determine whether TALARM active (be 1).
     * |[2]     |TALMIEN   |TALARM Interrupt Enable
     * |        |          |0 = Disable TALARM Interrupt
     * |        |          |1 = Enable TALARM Interrupt
     * |[3]     |TALMIF    |Current status of interrupt
     * |        |          |Latched whenever a Temperature Sense event occurs and IE = 1. Write '1' to clear.
 */
    __IO uint32_t TALMCTL;               

    /**
     * BODDTMR
     * ===================================================================================================
     * Offset: 0x10  Brown Out Detector Timer Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |DURTOFF   |Time BOD Detector Is Off
     * |        |          |(DURTOFF+1)*100us . Minimum value is 7. (default is 99.6ms)
     * |[16:19] |DURTON    |Time BOD Detector Is Active
     * |        |          |(DURTON+1) * 100us. Minimum value is 1. (default is 400us)
 */
    __IO uint32_t BODDTMR;               

} BODTALM_T;

/**
    @addtogroup BODTALM_CONST BODTALM Bit Field Definition
    Constant Definitions for BODTALM Controller
@{ */

#define BODTALM_BODSEL_BODVL_Pos         (0)                                               /*!< BODTALM BODSEL: BODVL Position         */
#define BODTALM_BODSEL_BODVL_Msk         (0x7ul << BODTALM_BODSEL_BODVL_Pos)               /*!< BODTALM BODSEL: BODVL Mask             */

#define BODTALM_BODSEL_BODHYS_Pos        (3)                                               /*!< BODTALM BODSEL: BODHYS Position        */
#define BODTALM_BODSEL_BODHYS_Msk        (0x1ul << BODTALM_BODSEL_BODHYS_Pos)              /*!< BODTALM BODSEL: BODHYS Mask            */

#define BODTALM_BODCTL_BODEN_Pos         (0)                                               /*!< BODTALM BODCTL: BODEN Position         */
#define BODTALM_BODCTL_BODEN_Msk         (0x3ul << BODTALM_BODCTL_BODEN_Pos)               /*!< BODTALM BODCTL: BODEN Mask             */

#define BODTALM_BODCTL_BODINTEN_Pos      (2)                                               /*!< BODTALM BODCTL: BODINTEN Position      */
#define BODTALM_BODCTL_BODINTEN_Msk      (0x1ul << BODTALM_BODCTL_BODINTEN_Pos)            /*!< BODTALM BODCTL: BODINTEN Mask          */

#define BODTALM_BODCTL_BODIF_Pos         (3)                                               /*!< BODTALM BODCTL: BODIF Position         */
#define BODTALM_BODCTL_BODIF_Msk         (0x1ul << BODTALM_BODCTL_BODIF_Pos)               /*!< BODTALM BODCTL: BODIF Mask             */

#define BODTALM_BODCTL_BODOUT_Pos        (4)                                               /*!< BODTALM BODCTL: BODOUT Position        */
#define BODTALM_BODCTL_BODOUT_Msk        (0x1ul << BODTALM_BODCTL_BODOUT_Pos)              /*!< BODTALM BODCTL: BODOUT Mask            */

#define BODTALM_TALMSEL_TALMVL_Pos       (0)                                               /*!< BODTALM TALMSEL: TALMVL Position       */
#define BODTALM_TALMSEL_TALMVL_Msk       (0xful << BODTALM_TALMSEL_TALMVL_Pos)             /*!< BODTALM TALMSEL: TALMVL Mask           */

#define BODTALM_TALMCTL_TALMEN_Pos       (0)                                               /*!< BODTALM TALMCTL: TALMEN Position       */
#define BODTALM_TALMCTL_TALMEN_Msk       (0x1ul << BODTALM_TALMCTL_TALMEN_Pos)             /*!< BODTALM TALMCTL: TALMEN Mask           */

#define BODTALM_TALMCTL_TALMOUT_Pos      (1)                                               /*!< BODTALM TALMCTL: TALMOUT Position      */
#define BODTALM_TALMCTL_TALMOUT_Msk      (0x1ul << BODTALM_TALMCTL_TALMOUT_Pos)            /*!< BODTALM TALMCTL: TALMOUT Mask          */

#define BODTALM_TALMCTL_TALMIEN_Pos      (2)                                               /*!< BODTALM TALMCTL: TALMIEN Position      */
#define BODTALM_TALMCTL_TALMIEN_Msk      (0x1ul << BODTALM_TALMCTL_TALMIEN_Pos)            /*!< BODTALM TALMCTL: TALMIEN Mask          */

#define BODTALM_TALMCTL_TALMIF_Pos       (3)                                               /*!< BODTALM TALMCTL: TALMIF Position       */
#define BODTALM_TALMCTL_TALMIF_Msk       (0x1ul << BODTALM_TALMCTL_TALMIF_Pos)             /*!< BODTALM TALMCTL: TALMIF Mask           */

#define BODTALM_BODDTMR_DURTOFF_Pos      (0)                                               /*!< BODTALM BODDTMR: DURTOFF Position      */
#define BODTALM_BODDTMR_DURTOFF_Msk      (0xfffful << BODTALM_BODDTMR_DURTOFF_Pos)         /*!< BODTALM BODDTMR: DURTOFF Mask          */

#define BODTALM_BODDTMR_DURTON_Pos       (16)                                              /*!< BODTALM BODDTMR: DURTON Position       */
#define BODTALM_BODDTMR_DURTON_Msk       (0xful << BODTALM_BODDTMR_DURTON_Pos)             /*!< BODTALM BODDTMR: DURTON Mask           */

/**@}*/ /* BODTALM_CONST */
/**@}*/ /* end of BODTALM register group */


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
     * |[3]     |LIRCEN    |OSC16K Oscillator Enable Bit
     * |        |          |0 = disable
     * |        |          |1 = enable (default)
     * |[9]     |STOP      |Stop
     * |        |          |Reserved - do not set to '1'
     * |[10]    |SPDEN     |Standby Power Down (SPD) Bit
     * |        |          |Set to '1' and issue WFI/WFE instruction to enter SPD mode.
     * |[11]    |DPDEN     |Deep Power Down (DPD) Bit
     * |        |          |Set to '1' and issue WFI/WFE instruction to enter DPD mode.
     * |[16]    |WKPINEN   |Wakeup Pin Enabled Control
     * |        |          |Determines whether WAKEUP pin is enabled in DPD mode.
     * |        |          |0 = enabled
     * |        |          |1 = disabled
     * |[17]    |LIRCDPDEN |OSC16K Enabled Control
     * |        |          |Determines whether OSC16K is enabled in DPD mode.
     * |        |          |If OSC16K is disabled, device cannot wake from DPD with SELWKTMR delay.
     * |        |          |0 = enabled
     * |        |          |1 = disabled
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
     * |[0]     |HCLKEN    |CPU Clock Enable (HCLK)
     * |        |          |Must be left as '1' for normal operation.
     * |[1]     |PDMACKEN  |PDMA Controller Clock Enable Control
     * |        |          |0 = To disable the PDMA engine clock
     * |        |          |1 = To enable the PDMA engine clock.
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
     * |[4]     |WDTCKEN   |Watchdog Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[5]     |RTCCKEN   |Real-Time-Clock APB Interface Clock Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[6]     |TMR0CKEN  |Timer0 Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[7]     |TMR1CKEN  |Timer1 Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[8]     |I2C0CKEN  |I2C0 Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[12]    |SPI0CKEN  |SPI0 Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[13]    |DPWMCKEN  |Differential PWM Speaker Driver Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[16]    |UARTCKEN  |UART0 Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[18]    |BFALCKEN  |Biquad Filter And Automatic Level Control Block Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[19]    |CRCCKEN   |Cyclic Redundancy Check Block Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[20]    |PWM0CH01CKEN|PWM0 CH0 and CH1 Block Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[22]    |ACMPCKEN  |Analog Comparator Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[26]    |SBRAMCKEN |Standby RAM Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[28]    |ADCCKEN   |Audio Analog-Digital-Converter (ADC) Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[29]    |I2S0CKEN  |I2S Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[30]    |ANACKEN   |Analog Block Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
 */
    __IO uint32_t APBCLK0;               

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
     * |        |          |000 = clock source from internal OSC48M oscillator.
     * |        |          |001 = clock source from external 32kHz crystal clock
     * |        |          |010 = clock source from internal 16 kHz oscillator clock
     * |        |          |Others = reserved
     * |[3:5]   |STCLKSEL  |MCU Cortex_M0 SysTick Clock Source Select
     * |        |          |These bits are protected, to write to bits first perform the unlock sequence (see Protected Register Lock Key Register (SYS_REGLCTL))
     * |        |          |000 = clock source from 16 kHz internal clock
     * |        |          |001 = clock source from external 32kHz crystal clock
     * |        |          |010 = clock source from 16 kHz internal oscillator divided by 2
     * |        |          |011 = clock source from OSC49M internal oscillator divided by 2
     * |        |          |1xx = clock source from HCLK / 2 (Default)
     * |        |          |Note that to use STCLKSEL as source of SysTic timer the CLKSRC bit of SYST_CSR must be set to 0.
     * |[6]     |HIRCFSEL  |OSC48M Frequency Select
     * |        |          |Determines which trim setting to use for OSC48M internal oscillator.
     * |        |          |Oscillator is factory trimmed within 1% to:.
     * |        |          |0= 49.152MHz (Default)
     * |        |          |1= 32.768MHz
 */
    __IO uint32_t CLKSEL0;               

    /**
     * CLKSEL1
     * ===================================================================================================
     * Offset: 0x14  Clock Source Select Control Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |WDTSEL    |WDT CLK Clock Source Select
     * |        |          |00 = clock source from internal OSC48M oscillator clock
     * |        |          |01 = clock source from external 32kHz crystal clock
     * |        |          |10 = clock source from HCLK/2048 clock
     * |        |          |11 = clock source from internal 16 kHz oscillator clock
     * |[4]     |DPWMCKSEL |Differential Speaker Driver PWM Clock Source Select
     * |        |          |0 = OSC48M clock
     * |        |          |1 = 2x OSC48M clock
     * |[8:10]  |TMR0SEL   |TIMER0 Clock Source Select
     * |        |          |000 = clock source from internal 16 kHz oscillator
     * |        |          |001 = clock source from external 32kHz crystal clock
     * |        |          |010 = clock source from HCLK
     * |        |          |011 = clock source from external pin (GPIOA[14])
     * |        |          |1xx = clock source from internal OSC48M oscillator clock
     * |[12:14] |TMR1SEL   |TIMER1 Clock Source Select
     * |        |          |000 = clock source from internal 16 kHz oscillator
     * |        |          |001 = clock source from external 32kHz crystal clock
     * |        |          |010 = clock source from HCLK
     * |        |          |011 = clock source from external pin (GPIOA[15])
     * |        |          |1xx = clock source from internal OSC48M oscillator clock
     * |[28:29] |PWM0CH01CKSEL|PWM0 CH0 and CH1 Clock Source Select
     * |        |          |PWM0 and PWM1 uses the same clock source, and prescaler
     * |        |          |00 = clock source from internal 16 kHz oscillator
     * |        |          |01 = clock source from external 32kHz crystal clock
     * |        |          |10 = clock source from HCLK
     * |        |          |11 = clock source from internal OSC48M oscillator clock
 */
    __IO uint32_t CLKSEL1;               

    /**
     * CLKDIV0
     * ===================================================================================================
     * Offset: 0x18  Clock Divider Number Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |HCLKDIV   |HCLK Clock Divide Number From HCLK Clock Source
     * |        |          |The HCLK clock frequency = (HCLK clock source frequency) / (HCLKDIV + 1)
     * |[8:11]  |UARTDIV   |UART Clock Divide Number From UART Clock Source
     * |        |          |The UART clock frequency = (UART clock source frequency ) / (UARTDIV + 1)
     * |[16:23] |ADCDIV    |ADC Clock Divide Number From ADC Clock Source
     * |        |          |The ADC clock frequency = (ADC clock source frequency ) / (ADCDIV + 1)
 */
    __IO uint32_t CLKDIV0;               

    /**
     * CLKSEL2
     * ===================================================================================================
     * Offset: 0x1C  Clock Source Select Control Register 2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |I2S0SEL   |I2S Clock Source Select
     * |        |          |00 = clock source from internal 16 kHz oscillator
     * |        |          |01 = clock source from external 32kHz crystal clock
     * |        |          |10 = clock source from HCLK
     * |        |          |11 = clock source from internal OSC48M oscillator clock
 */
    __IO uint32_t CLKSEL2;               

    /**
     * SLEEPCTL
     * ===================================================================================================
     * Offset: 0x20  Sleep Clock Source Select  Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |HCLKEN    |CPU Clock Sleep Enable (HCLK)
     * |        |          |Must be left as '1' for normal operation.
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[1]     |PDMACKEN  |PDMA Controller Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[2]     |ISPCKEN   |Flash ISP Controller Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[4]     |WDTCKEN   |Watchdog Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[5]     |RTCCKEN   |Real-Time- Sleep Clock APB Interface Clock Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[6]     |TMR0CKEN  |Timer0 Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[7]     |TMR1CKEN  |Timer1 Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[8]     |I2C0CKEN  |I2C0 Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[12]    |SPI0CKEN  |SPI0 Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[13]    |DPWMCKEN  |Differential PWM Speaker Driver Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[16]    |UARTCKEN  |UART0 Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[18]    |BFALCKEN  |Biquad filter/ALC block Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[19]    |CRCCKEN   |Cyclic Redundancy Check Sleep Block Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[20]    |PWM0CH01CKEN|PWM0 CH0 and CH1 Block Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[22]    |ACMPCKEN  |Analog Comparator Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[26]    |SBRAMCKEN |Standby RAM Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[28]    |ADCCKEN   |Audio Analog-Digital-Converter (ADC) Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[29]    |I2S0CKEN  |I2S Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
     * |[30]    |ANACKEN   |Analog Block Sleep Clock Enable Control
     * |        |          |0=Disable
     * |        |          |1=Enable
 */
    __IO uint32_t SLEEPCTL;              

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

    /**
     * DBGPD
     * ===================================================================================================
     * Offset: 0x28  Debug Port Power Down Disable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DISPDREQ  |Disable Power Down
     * |        |          |0 = Enable power down requests.
     * |        |          |1 = Disable power down requests.
     * |[6]     |ICECLKST  |ICE_CLK Pin State
     * |        |          |Read Only. Current state of ICE_CLK pin.
     * |[7]     |ICEDATST  |ICE_DAT Pin State
     * |        |          |Read Only. Current state of ICE_DAT pin.
 */
    __IO uint32_t DBGPD;                 

} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_PWRCTL_LXTEN_Pos             (1)                                               /*!< CLK PWRCTL: LXTEN Position             */
#define CLK_PWRCTL_LXTEN_Msk             (0x1ul << CLK_PWRCTL_LXTEN_Pos)                   /*!< CLK PWRCTL: LXTEN Mask                 */

#define CLK_PWRCTL_HIRCEN_Pos            (2)                                               /*!< CLK PWRCTL: HIRCEN Position            */
#define CLK_PWRCTL_HIRCEN_Msk            (0x1ul << CLK_PWRCTL_HIRCEN_Pos)                  /*!< CLK PWRCTL: HIRCEN Mask                */

#define CLK_PWRCTL_LIRCEN_Pos            (3)                                               /*!< CLK PWRCTL: LIRCEN Position            */
#define CLK_PWRCTL_LIRCEN_Msk            (0x1ul << CLK_PWRCTL_LIRCEN_Pos)                  /*!< CLK PWRCTL: LIRCEN Mask                */

#define CLK_PWRCTL_STOP_Pos              (9)                                               /*!< CLK PWRCTL: STOP Position              */
#define CLK_PWRCTL_STOP_Msk              (0x1ul << CLK_PWRCTL_STOP_Pos)                    /*!< CLK PWRCTL: STOP Mask                  */

#define CLK_PWRCTL_SPDEN_Pos             (10)                                              /*!< CLK PWRCTL: SPDEN Position             */
#define CLK_PWRCTL_SPDEN_Msk             (0x1ul << CLK_PWRCTL_SPDEN_Pos)                   /*!< CLK PWRCTL: SPDEN Mask                 */

#define CLK_PWRCTL_DPDEN_Pos             (11)                                              /*!< CLK PWRCTL: DPDEN Position             */
#define CLK_PWRCTL_DPDEN_Msk             (0x1ul << CLK_PWRCTL_DPDEN_Pos)                   /*!< CLK PWRCTL: DPDEN Mask                 */

#define CLK_PWRCTL_WKPINEN_Pos           (16)                                              /*!< CLK PWRCTL: WKPINEN Position           */
#define CLK_PWRCTL_WKPINEN_Msk           (0x1ul << CLK_PWRCTL_WKPINEN_Pos)                 /*!< CLK PWRCTL: WKPINEN Mask               */

#define CLK_PWRCTL_LIRCDPDEN_Pos         (17)                                              /*!< CLK PWRCTL: LIRCDPDEN Position         */
#define CLK_PWRCTL_LIRCDPDEN_Msk         (0x1ul << CLK_PWRCTL_LIRCDPDEN_Pos)               /*!< CLK PWRCTL: LIRCDPDEN Mask             */

#define CLK_PWRCTL_SELWKTMR_Pos          (20)                                              /*!< CLK PWRCTL: SELWKTMR Position          */
#define CLK_PWRCTL_SELWKTMR_Msk          (0xful << CLK_PWRCTL_SELWKTMR_Pos)                /*!< CLK PWRCTL: SELWKTMR Mask              */

#define CLK_PWRCTL_WKPINWKF_Pos          (24)                                              /*!< CLK PWRCTL: WKPINWKF Position          */
#define CLK_PWRCTL_WKPINWKF_Msk          (0x1ul << CLK_PWRCTL_WKPINWKF_Pos)                /*!< CLK PWRCTL: WKPINWKF Mask              */

#define CLK_PWRCTL_TMRWKF_Pos            (25)                                              /*!< CLK PWRCTL: TMRWKF Position            */
#define CLK_PWRCTL_TMRWKF_Msk            (0x1ul << CLK_PWRCTL_TMRWKF_Pos)                  /*!< CLK PWRCTL: TMRWKF Mask                */

#define CLK_PWRCTL_PORWKF_Pos            (26)                                              /*!< CLK PWRCTL: PORWKF Position            */
#define CLK_PWRCTL_PORWKF_Msk            (0x1ul << CLK_PWRCTL_PORWKF_Pos)                  /*!< CLK PWRCTL: PORWKF Mask                */

#define CLK_PWRCTL_WKTMRSTS_Pos          (28)                                              /*!< CLK PWRCTL: WKTMRSTS Position          */
#define CLK_PWRCTL_WKTMRSTS_Msk          (0xful << CLK_PWRCTL_WKTMRSTS_Pos)                /*!< CLK PWRCTL: WKTMRSTS Mask              */

#define CLK_AHBCLK_HCLKEN_Pos            (0)                                               /*!< CLK AHBCLK: HCLKEN Position            */
#define CLK_AHBCLK_HCLKEN_Msk            (0x1ul << CLK_AHBCLK_HCLKEN_Pos)                  /*!< CLK AHBCLK: HCLKEN Mask                */

#define CLK_AHBCLK_PDMACKEN_Pos          (1)                                               /*!< CLK AHBCLK: PDMACKEN Position          */
#define CLK_AHBCLK_PDMACKEN_Msk          (0x1ul << CLK_AHBCLK_PDMACKEN_Pos)                /*!< CLK AHBCLK: PDMACKEN Mask              */

#define CLK_AHBCLK_ISPCKEN_Pos           (2)                                               /*!< CLK AHBCLK: ISPCKEN Position           */
#define CLK_AHBCLK_ISPCKEN_Msk           (0x1ul << CLK_AHBCLK_ISPCKEN_Pos)                 /*!< CLK AHBCLK: ISPCKEN Mask               */

#define CLK_APBCLK0_WDTCKEN_Pos          (4)                                               /*!< CLK APBCLK0: WDTCKEN Position          */
#define CLK_APBCLK0_WDTCKEN_Msk          (0x1ul << CLK_APBCLK0_WDTCKEN_Pos)                /*!< CLK APBCLK0: WDTCKEN Mask              */

#define CLK_APBCLK0_RTCCKEN_Pos          (5)                                               /*!< CLK APBCLK0: RTCCKEN Position          */
#define CLK_APBCLK0_RTCCKEN_Msk          (0x1ul << CLK_APBCLK0_RTCCKEN_Pos)                /*!< CLK APBCLK0: RTCCKEN Mask              */

#define CLK_APBCLK0_TMR0CKEN_Pos         (6)                                               /*!< CLK APBCLK0: TMR0CKEN Position         */
#define CLK_APBCLK0_TMR0CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR0CKEN_Pos)               /*!< CLK APBCLK0: TMR0CKEN Mask             */

#define CLK_APBCLK0_TMR1CKEN_Pos         (7)                                               /*!< CLK APBCLK0: TMR1CKEN Position         */
#define CLK_APBCLK0_TMR1CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR1CKEN_Pos)               /*!< CLK APBCLK0: TMR1CKEN Mask             */

#define CLK_APBCLK0_I2C0CKEN_Pos         (8)                                               /*!< CLK APBCLK0: I2C0CKEN Position         */
#define CLK_APBCLK0_I2C0CKEN_Msk         (0x1ul << CLK_APBCLK0_I2C0CKEN_Pos)               /*!< CLK APBCLK0: I2C0CKEN Mask             */

#define CLK_APBCLK0_SPI0CKEN_Pos         (12)                                              /*!< CLK APBCLK0: SPI0CKEN Position         */
#define CLK_APBCLK0_SPI0CKEN_Msk         (0x1ul << CLK_APBCLK0_SPI0CKEN_Pos)               /*!< CLK APBCLK0: SPI0CKEN Mask             */

#define CLK_APBCLK0_DPWMCKEN_Pos         (13)                                              /*!< CLK APBCLK0: DPWMCKEN Position         */
#define CLK_APBCLK0_DPWMCKEN_Msk         (0x1ul << CLK_APBCLK0_DPWMCKEN_Pos)               /*!< CLK APBCLK0: DPWMCKEN Mask             */

#define CLK_APBCLK0_UARTCKEN_Pos         (16)                                              /*!< CLK APBCLK0: UARTCKEN Position         */
#define CLK_APBCLK0_UARTCKEN_Msk         (0x1ul << CLK_APBCLK0_UARTCKEN_Pos)               /*!< CLK APBCLK0: UARTCKEN Mask             */

#define CLK_APBCLK0_BFALCKEN_Pos         (18)                                              /*!< CLK APBCLK0: BFALCKEN Position         */
#define CLK_APBCLK0_BFALCKEN_Msk         (0x1ul << CLK_APBCLK0_BFALCKEN_Pos)               /*!< CLK APBCLK0: BFALCKEN Mask             */

#define CLK_APBCLK0_CRCCKEN_Pos          (19)                                              /*!< CLK APBCLK0: CRCCKEN Position          */
#define CLK_APBCLK0_CRCCKEN_Msk          (0x1ul << CLK_APBCLK0_CRCCKEN_Pos)                /*!< CLK APBCLK0: CRCCKEN Mask              */

#define CLK_APBCLK0_PWM0CH01CKEN_Pos     (20)                                              /*!< CLK APBCLK0: PWM0CH01CKEN Position     */
#define CLK_APBCLK0_PWM0CH01CKEN_Msk     (0x1ul << CLK_APBCLK0_PWM0CH01CKEN_Pos)           /*!< CLK APBCLK0: PWM0CH01CKEN Mask         */

#define CLK_APBCLK0_ACMPCKEN_Pos         (22)                                              /*!< CLK APBCLK0: ACMPCKEN Position         */
#define CLK_APBCLK0_ACMPCKEN_Msk         (0x1ul << CLK_APBCLK0_ACMPCKEN_Pos)               /*!< CLK APBCLK0: ACMPCKEN Mask             */

#define CLK_APBCLK0_SBRAMCKEN_Pos        (26)                                              /*!< CLK APBCLK0: SBRAMCKEN Position        */
#define CLK_APBCLK0_SBRAMCKEN_Msk        (0x1ul << CLK_APBCLK0_SBRAMCKEN_Pos)              /*!< CLK APBCLK0: SBRAMCKEN Mask            */

#define CLK_APBCLK0_ADCCKEN_Pos          (28)                                              /*!< CLK APBCLK0: ADCCKEN Position          */
#define CLK_APBCLK0_ADCCKEN_Msk          (0x1ul << CLK_APBCLK0_ADCCKEN_Pos)                /*!< CLK APBCLK0: ADCCKEN Mask              */

#define CLK_APBCLK0_I2S0CKEN_Pos         (29)                                              /*!< CLK APBCLK0: I2S0CKEN Position         */
#define CLK_APBCLK0_I2S0CKEN_Msk         (0x1ul << CLK_APBCLK0_I2S0CKEN_Pos)               /*!< CLK APBCLK0: I2S0CKEN Mask             */

#define CLK_APBCLK0_ANACKEN_Pos          (30)                                              /*!< CLK APBCLK0: ANACKEN Position          */
#define CLK_APBCLK0_ANACKEN_Msk          (0x1ul << CLK_APBCLK0_ANACKEN_Pos)                /*!< CLK APBCLK0: ANACKEN Mask              */

#define CLK_DPDSTATE_DPDSTSWR_Pos        (0)                                               /*!< CLK DPDSTATE: DPDSTSWR Position        */
#define CLK_DPDSTATE_DPDSTSWR_Msk        (0xfful << CLK_DPDSTATE_DPDSTSWR_Pos)             /*!< CLK DPDSTATE: DPDSTSWR Mask            */

#define CLK_DPDSTATE_DPDSTSRD_Pos        (8)                                               /*!< CLK DPDSTATE: DPDSTSRD Position        */
#define CLK_DPDSTATE_DPDSTSRD_Msk        (0xfful << CLK_DPDSTATE_DPDSTSRD_Pos)             /*!< CLK DPDSTATE: DPDSTSRD Mask            */

#define CLK_CLKSEL0_HCLKSEL_Pos          (0)                                               /*!< CLK CLKSEL0: HCLKSEL Position          */
#define CLK_CLKSEL0_HCLKSEL_Msk          (0x7ul << CLK_CLKSEL0_HCLKSEL_Pos)                /*!< CLK CLKSEL0: HCLKSEL Mask              */

#define CLK_CLKSEL0_STCLKSEL_Pos         (3)                                               /*!< CLK CLKSEL0: STCLKSEL Position         */
#define CLK_CLKSEL0_STCLKSEL_Msk         (0x7ul << CLK_CLKSEL0_STCLKSEL_Pos)               /*!< CLK CLKSEL0: STCLKSEL Mask             */

#define CLK_CLKSEL0_HIRCFSEL_Pos         (6)                                               /*!< CLK CLKSEL0: HIRCFSEL Position         */
#define CLK_CLKSEL0_HIRCFSEL_Msk         (0x1ul << CLK_CLKSEL0_HIRCFSEL_Pos)               /*!< CLK CLKSEL0: HIRCFSEL Mask             */

#define CLK_CLKSEL1_WDTSEL_Pos           (0)                                               /*!< CLK CLKSEL1: WDTSEL Position           */
#define CLK_CLKSEL1_WDTSEL_Msk           (0x3ul << CLK_CLKSEL1_WDTSEL_Pos)                 /*!< CLK CLKSEL1: WDTSEL Mask               */

#define CLK_CLKSEL1_DPWMCKSEL_Pos        (4)                                               /*!< CLK CLKSEL1: DPWMCKSEL Position        */
#define CLK_CLKSEL1_DPWMCKSEL_Msk        (0x1ul << CLK_CLKSEL1_DPWMCKSEL_Pos)              /*!< CLK CLKSEL1: DPWMCKSEL Mask            */

#define CLK_CLKSEL1_TMR0SEL_Pos          (8)                                               /*!< CLK CLKSEL1: TMR0SEL Position          */
#define CLK_CLKSEL1_TMR0SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR0SEL_Pos)                /*!< CLK CLKSEL1: TMR0SEL Mask              */

#define CLK_CLKSEL1_TMR1SEL_Pos          (12)                                              /*!< CLK CLKSEL1: TMR1SEL Position          */
#define CLK_CLKSEL1_TMR1SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR1SEL_Pos)                /*!< CLK CLKSEL1: TMR1SEL Mask              */

#define CLK_CLKSEL1_PWM0CH01CKSEL_Pos    (28)                                              /*!< CLK CLKSEL1: PWM0CH01CKSEL Position    */
#define CLK_CLKSEL1_PWM0CH01CKSEL_Msk    (0x3ul << CLK_CLKSEL1_PWM0CH01CKSEL_Pos)          /*!< CLK CLKSEL1: PWM0CH01CKSEL Mask        */

#define CLK_CLKDIV0_HCLKDIV_Pos          (0)                                               /*!< CLK CLKDIV0: HCLKDIV Position          */
#define CLK_CLKDIV0_HCLKDIV_Msk          (0xful << CLK_CLKDIV0_HCLKDIV_Pos)                /*!< CLK CLKDIV0: HCLKDIV Mask              */

#define CLK_CLKDIV0_UARTDIV_Pos          (8)                                               /*!< CLK CLKDIV0: UARTDIV Position          */
#define CLK_CLKDIV0_UARTDIV_Msk          (0xful << CLK_CLKDIV0_UARTDIV_Pos)                /*!< CLK CLKDIV0: UARTDIV Mask              */

#define CLK_CLKDIV0_ADCDIV_Pos           (16)                                              /*!< CLK CLKDIV0: ADCDIV Position           */
#define CLK_CLKDIV0_ADCDIV_Msk           (0xfful << CLK_CLKDIV0_ADCDIV_Pos)                /*!< CLK CLKDIV0: ADCDIV Mask               */

#define CLK_CLKSEL2_I2S0SEL_Pos          (0)                                               /*!< CLK CLKSEL2: I2S0SEL Position          */
#define CLK_CLKSEL2_I2S0SEL_Msk          (0x3ul << CLK_CLKSEL2_I2S0SEL_Pos)                /*!< CLK CLKSEL2: I2S0SEL Mask              */

#define CLK_SLEEPCTL_HCLKEN_Pos          (0)                                               /*!< CLK SLEEPCTL: HCLKEN Position          */
#define CLK_SLEEPCTL_HCLKEN_Msk          (0x1ul << CLK_SLEEPCTL_HCLKEN_Pos)                /*!< CLK SLEEPCTL: HCLKEN Mask              */

#define CLK_SLEEPCTL_PDMACKEN_Pos        (1)                                               /*!< CLK SLEEPCTL: PDMACKEN Position        */
#define CLK_SLEEPCTL_PDMACKEN_Msk        (0x1ul << CLK_SLEEPCTL_PDMACKEN_Pos)              /*!< CLK SLEEPCTL: PDMACKEN Mask            */

#define CLK_SLEEPCTL_ISPCKEN_Pos         (2)                                               /*!< CLK SLEEPCTL: ISPCKEN Position         */
#define CLK_SLEEPCTL_ISPCKEN_Msk         (0x1ul << CLK_SLEEPCTL_ISPCKEN_Pos)               /*!< CLK SLEEPCTL: ISPCKEN Mask             */

#define CLK_SLEEPCTL_WDTCKEN_Pos         (4)                                               /*!< CLK SLEEPCTL: WDTCKEN Position         */
#define CLK_SLEEPCTL_WDTCKEN_Msk         (0x1ul << CLK_SLEEPCTL_WDTCKEN_Pos)               /*!< CLK SLEEPCTL: WDTCKEN Mask             */

#define CLK_SLEEPCTL_RTCCKEN_Pos         (5)                                               /*!< CLK SLEEPCTL: RTCCKEN Position         */
#define CLK_SLEEPCTL_RTCCKEN_Msk         (0x1ul << CLK_SLEEPCTL_RTCCKEN_Pos)               /*!< CLK SLEEPCTL: RTCCKEN Mask             */

#define CLK_SLEEPCTL_TMR0CKEN_Pos        (6)                                               /*!< CLK SLEEPCTL: TMR0CKEN Position        */
#define CLK_SLEEPCTL_TMR0CKEN_Msk        (0x1ul << CLK_SLEEPCTL_TMR0CKEN_Pos)              /*!< CLK SLEEPCTL: TMR0CKEN Mask            */

#define CLK_SLEEPCTL_TMR1CKEN_Pos        (7)                                               /*!< CLK SLEEPCTL: TMR1CKEN Position        */
#define CLK_SLEEPCTL_TMR1CKEN_Msk        (0x1ul << CLK_SLEEPCTL_TMR1CKEN_Pos)              /*!< CLK SLEEPCTL: TMR1CKEN Mask            */

#define CLK_SLEEPCTL_I2C0CKEN_Pos        (8)                                               /*!< CLK SLEEPCTL: I2C0CKEN Position        */
#define CLK_SLEEPCTL_I2C0CKEN_Msk        (0x1ul << CLK_SLEEPCTL_I2C0CKEN_Pos)              /*!< CLK SLEEPCTL: I2C0CKEN Mask            */

#define CLK_SLEEPCTL_SPI0CKEN_Pos        (12)                                              /*!< CLK SLEEPCTL: SPI0CKEN Position        */
#define CLK_SLEEPCTL_SPI0CKEN_Msk        (0x1ul << CLK_SLEEPCTL_SPI0CKEN_Pos)              /*!< CLK SLEEPCTL: SPI0CKEN Mask            */

#define CLK_SLEEPCTL_DPWMCKEN_Pos        (13)                                              /*!< CLK SLEEPCTL: DPWMCKEN Position        */
#define CLK_SLEEPCTL_DPWMCKEN_Msk        (0x1ul << CLK_SLEEPCTL_DPWMCKEN_Pos)              /*!< CLK SLEEPCTL: DPWMCKEN Mask            */

#define CLK_SLEEPCTL_UARTCKEN_Pos        (16)                                              /*!< CLK SLEEPCTL: UARTCKEN Position        */
#define CLK_SLEEPCTL_UARTCKEN_Msk        (0x1ul << CLK_SLEEPCTL_UARTCKEN_Pos)              /*!< CLK SLEEPCTL: UARTCKEN Mask            */

#define CLK_SLEEPCTL_BFALCKEN_Pos        (18)                                              /*!< CLK SLEEPCTL: BFALCKEN Position        */
#define CLK_SLEEPCTL_BFALCKEN_Msk        (0x1ul << CLK_SLEEPCTL_BFALCKEN_Pos)              /*!< CLK SLEEPCTL: BFALCKEN Mask            */

#define CLK_SLEEPCTL_CRCCKEN_Pos         (19)                                              /*!< CLK SLEEPCTL: CRCCKEN Position         */
#define CLK_SLEEPCTL_CRCCKEN_Msk         (0x1ul << CLK_SLEEPCTL_CRCCKEN_Pos)               /*!< CLK SLEEPCTL: CRCCKEN Mask             */

#define CLK_SLEEPCTL_PWM0CH01CKEN_Pos    (20)                                              /*!< CLK SLEEPCTL: PWM0CH01CKEN Position    */
#define CLK_SLEEPCTL_PWM0CH01CKEN_Msk    (0x1ul << CLK_SLEEPCTL_PWM0CH01CKEN_Pos)          /*!< CLK SLEEPCTL: PWM0CH01CKEN Mask        */

#define CLK_SLEEPCTL_ACMPCKEN_Pos        (22)                                              /*!< CLK SLEEPCTL: ACMPCKEN Position        */
#define CLK_SLEEPCTL_ACMPCKEN_Msk        (0x1ul << CLK_SLEEPCTL_ACMPCKEN_Pos)              /*!< CLK SLEEPCTL: ACMPCKEN Mask            */

#define CLK_SLEEPCTL_SBRAMCKEN_Pos       (26)                                              /*!< CLK SLEEPCTL: SBRAMCKEN Position       */
#define CLK_SLEEPCTL_SBRAMCKEN_Msk       (0x1ul << CLK_SLEEPCTL_SBRAMCKEN_Pos)             /*!< CLK SLEEPCTL: SBRAMCKEN Mask           */

#define CLK_SLEEPCTL_ADCCKEN_Pos         (28)                                              /*!< CLK SLEEPCTL: ADCCKEN Position         */
#define CLK_SLEEPCTL_ADCCKEN_Msk         (0x1ul << CLK_SLEEPCTL_ADCCKEN_Pos)               /*!< CLK SLEEPCTL: ADCCKEN Mask             */

#define CLK_SLEEPCTL_I2S0CKEN_Pos        (29)                                              /*!< CLK SLEEPCTL: I2S0CKEN Position        */
#define CLK_SLEEPCTL_I2S0CKEN_Msk        (0x1ul << CLK_SLEEPCTL_I2S0CKEN_Pos)              /*!< CLK SLEEPCTL: I2S0CKEN Mask            */

#define CLK_SLEEPCTL_ANACKEN_Pos         (30)                                              /*!< CLK SLEEPCTL: ANACKEN Position         */
#define CLK_SLEEPCTL_ANACKEN_Msk         (0x1ul << CLK_SLEEPCTL_ANACKEN_Pos)               /*!< CLK SLEEPCTL: ANACKEN Mask             */

#define CLK_PWRSTSF_DSF_Pos              (0)                                               /*!< CLK PWRSTSF: DSF Position              */
#define CLK_PWRSTSF_DSF_Msk              (0x1ul << CLK_PWRSTSF_DSF_Pos)                    /*!< CLK PWRSTSF: DSF Mask                  */

#define CLK_PWRSTSF_STOPF_Pos            (1)                                               /*!< CLK PWRSTSF: STOPF Position            */
#define CLK_PWRSTSF_STOPF_Msk            (0x1ul << CLK_PWRSTSF_STOPF_Pos)                  /*!< CLK PWRSTSF: STOPF Mask                */

#define CLK_PWRSTSF_SPDF_Pos             (2)                                               /*!< CLK PWRSTSF: SPDF Position             */
#define CLK_PWRSTSF_SPDF_Msk             (0x1ul << CLK_PWRSTSF_SPDF_Pos)                   /*!< CLK PWRSTSF: SPDF Mask                 */

#define CLK_DBGPD_DISPDREQ_Pos           (0)                                               /*!< CLK DBGPD: DISPDREQ Position           */
#define CLK_DBGPD_DISPDREQ_Msk           (0x1ul << CLK_DBGPD_DISPDREQ_Pos)                 /*!< CLK DBGPD: DISPDREQ Mask               */

#define CLK_DBGPD_ICECLKST_Pos           (6)                                               /*!< CLK DBGPD: ICECLKST Position           */
#define CLK_DBGPD_ICECLKST_Msk           (0x1ul << CLK_DBGPD_ICECLKST_Pos)                 /*!< CLK DBGPD: ICECLKST Mask               */

#define CLK_DBGPD_ICEDATST_Pos           (7)                                               /*!< CLK DBGPD: ICEDATST Position           */
#define CLK_DBGPD_ICEDATST_Msk           (0x1ul << CLK_DBGPD_ICEDATST_Pos)                 /*!< CLK DBGPD: ICEDATST Mask               */

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */


/*---------------------- Cyclic Redundancy Check Controller -------------------------*/
/**
    @addtogroup CRC Cyclic Redundancy Check Controller(CRC)
    Memory Mapped Structure for CRC Controller
@{ */
 
typedef struct
{


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  CRC Enable Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:8]   |PKTLEN    |CRC Packet Length
     * |        |          |Indicates number of bytes of CRC input to process.
     * |        |          |CRC calculation will stop once input number of bytes = PKTLEN+1.
     * |        |          |Maximum packet size is 512 bytes, for PKTLEN = 511.
     * |        |          |Writing any value to this register will flush all previous calculations and restart a new CRC calculation.
     * |[16]    |MODE      |CRC LSB mode
     * |        |          |Determines whether CRC Generator processes input words (32bit/4Bytes) LSB (least significant byte) first or MSB (most significant byte) first.
     * |        |          |0 = CRC input is MSB first (default).
     * |        |          |1 = CRC input is LSB first.
     * |        |          |For example if MODE = 1, and 0x01020304 is written to CRC_DAT, bytes will be processed in order 0x04, 0x03, 0x02, 0x01.
     * |        |          |If MODE = 0, then order would be 0x01, 0x02, 0x3, 0x04.
     * |        |          |Writing any value to this register will flush all previous calculations and restart a new CRC calculation.
 */
    __IO uint32_t CTL;                   

    /**
     * DAT
     * ===================================================================================================
     * Offset: 0x04  CRC Input Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |DATA      |CRC Input
     * |        |          |The string of bytes to perform CRC calculation on.
     * |        |          |When MODE = 0, CRC performs calculation byte by byte in the order DATA[31:24], DATA[23:16], DATA[15:8], DATA[7:0].
     * |        |          |When MODE = 1, CRC performs calculation byte by byte in the order DATA[7:0], DATA[15:8], DATA[23:16], DATA[31:24].
     * |        |          |If number of input bytes exceeds CRC Packet Length (CRC_CTL[8:0]+1), any additional input bytes will be ignored.
     * |        |          |The CRC generator takes four clock cycles to process the CRC input.
     * |        |          |Software must ensure that at least four clock cycles occur between writes of CRC_DAT.
     * |        |          |Compiled assembly language can be examined to ensure this requirement is met.
 */
    __IO uint32_t DAT;                   

    /**
     * CHECKSUM
     * ===================================================================================================
     * Offset: 0x08  CRC Output Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CHECKSUM  |CRC Output
     * |        |          |The result of CRC computation.
     * |        |          |The result is valid four clock cycles after last CRC_DAT input data is written to CRC generator.
 */
    __I  uint32_t CHECKSUM;              

} CRC_T;

/**
    @addtogroup CRC_CONST CRC Bit Field Definition
    Constant Definitions for CRC Controller
@{ */

#define CRC_CTL_PKTLEN_Pos               (0)                                               /*!< CRC CTL: PKTLEN Position               */
#define CRC_CTL_PKTLEN_Msk               (0x1fful << CRC_CTL_PKTLEN_Pos)                   /*!< CRC CTL: PKTLEN Mask                   */

#define CRC_CTL_MODE_Pos                 (16)                                              /*!< CRC CTL: MODE Position                 */
#define CRC_CTL_MODE_Msk                 (0x1ul << CRC_CTL_MODE_Pos)                       /*!< CRC CTL: MODE Mask                     */

#define CRC_DAT_DATA_Pos                 (0)                                               /*!< CRC DAT: DATA Position                 */
#define CRC_DAT_DATA_Msk                 (0xfffffffful << CRC_DAT_DATA_Pos)                /*!< CRC DAT: DATA Mask                     */

#define CRC_CHECKSUM_CHECKSUM_Pos        (0)                                               /*!< CRC CHECKSUM: CHECKSUM Position        */
#define CRC_CHECKSUM_CHECKSUM_Msk        (0xfffful << CRC_CHECKSUM_CHECKSUM_Pos)           /*!< CRC CHECKSUM: CHECKSUM Mask            */

/**@}*/ /* CRC_CONST */
/**@}*/ /* end of CRC register group */


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

#define DPWM_STS_FULL_Pos                (0)                                               /*!< DPWM STS: FULL Position                */
#define DPWM_STS_FULL_Msk                (0x1ul << DPWM_STS_FULL_Pos)                      /*!< DPWM STS: FULL Mask                    */

#define DPWM_STS_EMPTY_Pos               (1)                                               /*!< DPWM STS: EMPTY Position               */
#define DPWM_STS_EMPTY_Msk               (0x1ul << DPWM_STS_EMPTY_Pos)                     /*!< DPWM STS: EMPTY Mask                   */

#define DPWM_DMACTL_DMAEN_Pos            (0)                                               /*!< DPWM DMACTL: DMAEN Position            */
#define DPWM_DMACTL_DMAEN_Msk            (0x1ul << DPWM_DMACTL_DMAEN_Pos)                  /*!< DPWM DMACTL: DMAEN Mask                */

#define DPWM_DATA_INDATA_Pos             (0)                                               /*!< DPWM DATA: INDATA Position             */
#define DPWM_DATA_INDATA_Msk             (0xfffful << DPWM_DATA_INDATA_Pos)                /*!< DPWM DATA: INDATA Mask                 */

#define DPWM_ZOHDIV_ZOHDIV_Pos           (0)                                               /*!< DPWM ZOHDIV: ZOHDIV Position           */
#define DPWM_ZOHDIV_ZOHDIV_Msk           (0xfful << DPWM_ZOHDIV_ZOHDIV_Pos)                /*!< DPWM ZOHDIV: ZOHDIV Mask               */

/**@}*/ /* DPWM_CONST */
/**@}*/ /* end of DPWM register group */


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
     * |[0:1]   |MODE0     |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[2:3]   |MODE1     |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[4:5]   |MODE2     |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[6:7]   |MODE3     |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[8:9]   |MODE4     |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[10:11] |MODE5     |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[12:13] |MODE6     |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[14:15] |MODE7     |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[16:17] |MODE8     |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[18:19] |MODE9     |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[20:21] |MODE10    |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[22:23] |MODE11    |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[24:25] |MODE12    |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[26:27] |MODE13    |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[28:29] |MODE14    |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
     * |[30:31] |MODE15    |GPIOx I/O Pin[n] Mode Control
     * |        |          |Determine each I/O type of GPIOx pins.
     * |        |          |00 = GPIO port [n] pin is in INPUT mode.
     * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
     * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
     * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 */
    __IO uint32_t MODE;               

    /**
     * DINOFF
     * ===================================================================================================
     * Offset: 0x04  GPIO Pin Digital Input Disable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16]    |DINOFF16  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[17]    |DINOFF17  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[18]    |DINOFF18  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[19]    |DINOFF19  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[20]    |DINOFF20  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[21]    |DINOFF21  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[22]    |DINOFF22  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[23]    |DINOFF23  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[24]    |DINOFF24  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[25]    |DINOFF25  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[26]    |DINOFF26  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[27]    |DINOFF27  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[28]    |DINOFF28  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[29]    |DINOFF29  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[30]    |DINOFF30  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
     * |[31]    |DINOFF31  |GPIOx Pin[n] OFF Digital Input Path Enable
     * |        |          |0 = Enable IO digital input path (Default)
     * |        |          |1 = Disable IO digital input path (low leakage mode)
 */
    __IO uint32_t DINOFF;             

    /**
     * DOUT
     * ===================================================================================================
     * Offset: 0x08  GPIO Data Output Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DOUT0     |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[1]     |DOUT1     |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[2]     |DOUT2     |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[3]     |DOUT3     |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[4]     |DOUT4     |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[5]     |DOUT5     |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[6]     |DOUT6     |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[7]     |DOUT7     |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[8]     |DOUT8     |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[9]     |DOUT9     |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[10]    |DOUT10    |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[11]    |DOUT11    |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[12]    |DOUT12    |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[13]    |DOUT13    |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[14]    |DOUT14    |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
     * |[15]    |DOUT15    |GPIOx Pin[n] Output Value
     * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
     * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
     * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
 */
    __IO uint32_t DOUT;               

    /**
     * DATMSK
     * ===================================================================================================
     * Offset: 0x0C  GPIO Data Output Write Mask
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DATMSK0   |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[1]     |DATMSK1   |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[2]     |DATMSK2   |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[3]     |DATMSK3   |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[4]     |DATMSK4   |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[5]     |DATMSK5   |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[6]     |DATMSK6   |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[7]     |DATMSK7   |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[8]     |DATMSK8   |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[9]     |DATMSK9   |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[10]    |DATMSK10  |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[11]    |DATMSK11  |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[12]    |DATMSK12  |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[13]    |DATMSK13  |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[14]    |DATMSK14  |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
     * |[15]    |DATMSK15  |Port [A/B] Data Output Write Mask
     * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n].
     * |        |          |When set the DATMSK bit[n] to "1", the corresponding DOUTn bit is write-protected.
     * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated
     * |        |          |1 = The corresponding Px_DOUT[n] bit is read only
 */
    __IO uint32_t DATMSK;             

    /**
     * PIN
     * ===================================================================================================
     * Offset: 0x10  GPIO Pin Value
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIN0      |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[1]     |PIN1      |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[2]     |PIN2      |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[3]     |PIN3      |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[4]     |PIN4      |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[5]     |PIN5      |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[6]     |PIN6      |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[7]     |PIN7      |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[8]     |PIN8      |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[9]     |PIN9      |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[10]    |PIN10     |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[11]    |PIN11     |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[12]    |PIN12     |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[13]    |PIN13     |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[14]    |PIN14     |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
     * |[15]    |PIN15     |Port [A/B] Pin Values
     * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
 */
    __I  uint32_t PIN;                

    /**
     * DBEN
     * ===================================================================================================
     * Offset: 0x14  GPIO De-bounce Enable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |DBEN0     |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[1]     |DBEN1     |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[2]     |DBEN2     |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[3]     |DBEN3     |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[4]     |DBEN4     |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[5]     |DBEN5     |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[6]     |DBEN6     |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[7]     |DBEN7     |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[8]     |DBEN8     |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[9]     |DBEN9     |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[10]    |DBEN10    |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[11]    |DBEN11    |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[12]    |DBEN12    |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[13]    |DBEN13    |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[14]    |DBEN14    |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
     * |[15]    |DBEN15    |Port [A/B] Input Signal De-bounce Enable
     * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit.
     * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods.
     * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
     * |        |          |The DBEN[n] is used for "edge-trigger" interrupt only; it is ignored for "level trigger" interrupt
     * |        |          |0 = The bit[n] de-bounce function is disabled
     * |        |          |1 = The bit[n] de-bounce function is enabled 
 */
    __IO uint32_t DBEN;               

    /**
     * INTTYPE
     * ===================================================================================================
     * Offset: 0x18  GPIO Interrupt Mode Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TYPE0     |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[1]     |TYPE1     |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[2]     |TYPE2     |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[3]     |TYPE3     |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[4]     |TYPE4     |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[5]     |TYPE5     |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[6]     |TYPE6     |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[7]     |TYPE7     |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[8]     |TYPE8     |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[9]     |TYPE9     |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[10]    |TYPE10    |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[11]    |TYPE11    |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[12]    |TYPE12    |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[13]    |TYPE13    |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[14]    |TYPE14    |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
     * |[15]    |TYPE15    |Port [A/B] Edge Or Level Detection Interrupt Control
     * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered.
     * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register.
     * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt.
     * |        |          |0 = Edge triggered interrupt
     * |        |          |1 = Level triggered interrupt
     * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register.
     * |        |          |If both levels are set no interrupt will occur.
 */
    __IO uint32_t INTTYPE;            

    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x1C  GPIO Interrupt Enable
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |FLIEN0    |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[1]     |FLIEN1    |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[2]     |FLIEN2    |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[3]     |FLIEN3    |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[4]     |FLIEN4    |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[5]     |FLIEN5    |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[6]     |FLIEN6    |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[7]     |FLIEN7    |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[8]     |FLIEN8    |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[9]     |FLIEN9    |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[10]    |FLIEN10   |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[11]    |FLIEN11   |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[12]    |FLIEN12   |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[13]    |FLIEN13   |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[14]    |FLIEN14   |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[15]    |FLIEN15   |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
     * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "low" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "high-to-low" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt
     * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt 
     * |[16]    |RHIEN0    |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[17]    |RHIEN1    |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[18]    |RHIEN2    |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[19]    |RHIEN3    |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[20]    |RHIEN4    |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[21]    |RHIEN5    |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[22]    |RHIEN6    |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[23]    |RHIEN7    |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[24]    |RHIEN8    |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[25]    |RHIEN9    |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[26]    |RHIEN10   |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[27]    |RHIEN11   |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[28]    |RHIEN12   |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[29]    |RHIEN13   |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[30]    |RHIEN14   |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
     * |[31]    |RHIEN15   |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
     * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins.
     * |        |          |It also enables the pin wakeup function.
     * |        |          |If the interrupt is configured in level trigger mode, a level "high" will generate an interrupt.
     * |        |          |If the interrupt is configured in edge trigger mode, a state change from "low-to-high" will generate an interrupt.
     * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
     * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
     * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt 
 */
    __IO uint32_t INTEN;              

    /**
     * INTSRC
     * ===================================================================================================
     * Offset: 0x20  GPIO Interrupt Trigger Source Indicator
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |INTSRC0   |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[1]     |INTSRC1   |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[2]     |INTSRC2   |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[3]     |INTSRC3   |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[4]     |INTSRC4   |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[5]     |INTSRC5   |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[6]     |INTSRC6   |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[7]     |INTSRC7   |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[8]     |INTSRC8   |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[9]     |INTSRC9   |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[10]    |INTSRC10  |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[11]    |INTSRC11  |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[12]    |INTSRC12  |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[13]    |INTSRC13  |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[14]    |INTSRC14  |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
     * |[15]    |INTSRC15  |Port [A/B] Interrupt Trigger Source Indicator
     * |        |          |Read :
     * |        |          |1 = Indicates GPIOx[n] generated an interrupt
     * |        |          |0 = No interrupt from GPIOx[n]
     * |        |          |Write :
     * |        |          |1 = Clear the corresponding pending interrupt.
     * |        |          |0 = No action
 */
    __IO uint32_t INTSRC;                     

} GPIO_T;


typedef struct { 
    /**
     * GPIO_DBCTL
     * ===================================================================================================
     * Offset: 0x000  Interrupt De-bounce Control
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |DBCLKSEL  |De-bounce Sampling Cycle Selection.
     * |        |          |For edge level interrupt GPIO state is sampled every 2^(DBCLKSEL) de-bounce clocks.
     * |        |          |For example if DBCLKSRC = 6, then interrupt is sampled every 2^6 = 64 de-bounce clocks.
     * |        |          |If DBCLKSRC is 16KHz oscillator this would be a 64ms de-bounce.
     * |[4]     |DBCLKSRC  |De-bounce Counter Clock Source Select
     * |        |          |0 = De-bounce counter clock source is HCLK
     * |        |          |1 = De-bounce counter clock source is the internal 16 kHz clock
     * |[5]     |ICLKON    |Interrupt Clock On Mode
     * |        |          |Set this bit "0" will gate the clock to the interrupt generation circuit if the GPIOx[n] interrupt is disabled.
     * |        |          |0 = disable the clock if the GPIOx[n] interrupt is disabled
     * |        |          |1 = Interrupt generation clock always active.
 */
    __IO uint32_t DBCTL;  
} GPIO_DB_T; 

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

#define GPIO_DINOFF_DINOFF16_Pos         (16)                                              /*!< GPIO DINOFF: DINOFF16 Position         */
#define GPIO_DINOFF_DINOFF16_Msk         (0x1ul << GPIO_DINOFF_DINOFF16_Pos)               /*!< GPIO DINOFF: DINOFF16 Mask             */

#define GPIO_DINOFF_DINOFF17_Pos         (17)                                              /*!< GPIO DINOFF: DINOFF17 Position         */
#define GPIO_DINOFF_DINOFF17_Msk         (0x1ul << GPIO_DINOFF_DINOFF17_Pos)               /*!< GPIO DINOFF: DINOFF17 Mask             */

#define GPIO_DINOFF_DINOFF18_Pos         (18)                                              /*!< GPIO DINOFF: DINOFF18 Position         */
#define GPIO_DINOFF_DINOFF18_Msk         (0x1ul << GPIO_DINOFF_DINOFF18_Pos)               /*!< GPIO DINOFF: DINOFF18 Mask             */

#define GPIO_DINOFF_DINOFF19_Pos         (19)                                              /*!< GPIO DINOFF: DINOFF19 Position         */
#define GPIO_DINOFF_DINOFF19_Msk         (0x1ul << GPIO_DINOFF_DINOFF19_Pos)               /*!< GPIO DINOFF: DINOFF19 Mask             */

#define GPIO_DINOFF_DINOFF20_Pos         (20)                                              /*!< GPIO DINOFF: DINOFF20 Position         */
#define GPIO_DINOFF_DINOFF20_Msk         (0x1ul << GPIO_DINOFF_DINOFF20_Pos)               /*!< GPIO DINOFF: DINOFF20 Mask             */

#define GPIO_DINOFF_DINOFF21_Pos         (21)                                              /*!< GPIO DINOFF: DINOFF21 Position         */
#define GPIO_DINOFF_DINOFF21_Msk         (0x1ul << GPIO_DINOFF_DINOFF21_Pos)               /*!< GPIO DINOFF: DINOFF21 Mask             */

#define GPIO_DINOFF_DINOFF22_Pos         (22)                                              /*!< GPIO DINOFF: DINOFF22 Position         */
#define GPIO_DINOFF_DINOFF22_Msk         (0x1ul << GPIO_DINOFF_DINOFF22_Pos)               /*!< GPIO DINOFF: DINOFF22 Mask             */

#define GPIO_DINOFF_DINOFF23_Pos         (23)                                              /*!< GPIO DINOFF: DINOFF23 Position         */
#define GPIO_DINOFF_DINOFF23_Msk         (0x1ul << GPIO_DINOFF_DINOFF23_Pos)               /*!< GPIO DINOFF: DINOFF23 Mask             */

#define GPIO_DINOFF_DINOFF24_Pos         (24)                                              /*!< GPIO DINOFF: DINOFF24 Position         */
#define GPIO_DINOFF_DINOFF24_Msk         (0x1ul << GPIO_DINOFF_DINOFF24_Pos)               /*!< GPIO DINOFF: DINOFF24 Mask             */

#define GPIO_DINOFF_DINOFF25_Pos         (25)                                              /*!< GPIO DINOFF: DINOFF25 Position         */
#define GPIO_DINOFF_DINOFF25_Msk         (0x1ul << GPIO_DINOFF_DINOFF25_Pos)               /*!< GPIO DINOFF: DINOFF25 Mask             */

#define GPIO_DINOFF_DINOFF26_Pos         (26)                                              /*!< GPIO DINOFF: DINOFF26 Position         */
#define GPIO_DINOFF_DINOFF26_Msk         (0x1ul << GPIO_DINOFF_DINOFF26_Pos)               /*!< GPIO DINOFF: DINOFF26 Mask             */

#define GPIO_DINOFF_DINOFF27_Pos         (27)                                              /*!< GPIO DINOFF: DINOFF27 Position         */
#define GPIO_DINOFF_DINOFF27_Msk         (0x1ul << GPIO_DINOFF_DINOFF27_Pos)               /*!< GPIO DINOFF: DINOFF27 Mask             */

#define GPIO_DINOFF_DINOFF28_Pos         (28)                                              /*!< GPIO DINOFF: DINOFF28 Position         */
#define GPIO_DINOFF_DINOFF28_Msk         (0x1ul << GPIO_DINOFF_DINOFF28_Pos)               /*!< GPIO DINOFF: DINOFF28 Mask             */

#define GPIO_DINOFF_DINOFF29_Pos         (29)                                              /*!< GPIO DINOFF: DINOFF29 Position         */
#define GPIO_DINOFF_DINOFF29_Msk         (0x1ul << GPIO_DINOFF_DINOFF29_Pos)               /*!< GPIO DINOFF: DINOFF29 Mask             */

#define GPIO_DINOFF_DINOFF30_Pos         (30)                                              /*!< GPIO DINOFF: DINOFF30 Position         */
#define GPIO_DINOFF_DINOFF30_Msk         (0x1ul << GPIO_DINOFF_DINOFF30_Pos)               /*!< GPIO DINOFF: DINOFF30 Mask             */

#define GPIO_DINOFF_DINOFF31_Pos         (31)                                              /*!< GPIO DINOFF: DINOFF31 Position         */
#define GPIO_DINOFF_DINOFF31_Msk         (0x1ul << GPIO_DINOFF_DINOFF31_Pos)               /*!< GPIO DINOFF: DINOFF31 Mask             */

#define GPIO_DOUT_DOUT0_Pos              (0)                                               /*!< GPIO DOUT: DOUT0 Position              */
#define GPIO_DOUT_DOUT0_Msk              (0x1ul << GPIO_DOUT_DOUT0_Pos)                    /*!< GPIO DOUT: DOUT0 Mask                  */

#define GPIO_DOUT_DOUT1_Pos              (1)                                               /*!< GPIO DOUT: DOUT1 Position              */
#define GPIO_DOUT_DOUT1_Msk              (0x1ul << GPIO_DOUT_DOUT1_Pos)                    /*!< GPIO DOUT: DOUT1 Mask                  */

#define GPIO_DOUT_DOUT2_Pos              (2)                                               /*!< GPIO DOUT: DOUT2 Position              */
#define GPIO_DOUT_DOUT2_Msk              (0x1ul << GPIO_DOUT_DOUT2_Pos)                    /*!< GPIO DOUT: DOUT2 Mask                  */

#define GPIO_DOUT_DOUT3_Pos              (3)                                               /*!< GPIO DOUT: DOUT3 Position              */
#define GPIO_DOUT_DOUT3_Msk              (0x1ul << GPIO_DOUT_DOUT3_Pos)                    /*!< GPIO DOUT: DOUT3 Mask                  */

#define GPIO_DOUT_DOUT4_Pos              (4)                                               /*!< GPIO DOUT: DOUT4 Position              */
#define GPIO_DOUT_DOUT4_Msk              (0x1ul << GPIO_DOUT_DOUT4_Pos)                    /*!< GPIO DOUT: DOUT4 Mask                  */

#define GPIO_DOUT_DOUT5_Pos              (5)                                               /*!< GPIO DOUT: DOUT5 Position              */
#define GPIO_DOUT_DOUT5_Msk              (0x1ul << GPIO_DOUT_DOUT5_Pos)                    /*!< GPIO DOUT: DOUT5 Mask                  */

#define GPIO_DOUT_DOUT6_Pos              (6)                                               /*!< GPIO DOUT: DOUT6 Position              */
#define GPIO_DOUT_DOUT6_Msk              (0x1ul << GPIO_DOUT_DOUT6_Pos)                    /*!< GPIO DOUT: DOUT6 Mask                  */

#define GPIO_DOUT_DOUT7_Pos              (7)                                               /*!< GPIO DOUT: DOUT7 Position              */
#define GPIO_DOUT_DOUT7_Msk              (0x1ul << GPIO_DOUT_DOUT7_Pos)                    /*!< GPIO DOUT: DOUT7 Mask                  */

#define GPIO_DOUT_DOUT8_Pos              (8)                                               /*!< GPIO DOUT: DOUT8 Position              */
#define GPIO_DOUT_DOUT8_Msk              (0x1ul << GPIO_DOUT_DOUT8_Pos)                    /*!< GPIO DOUT: DOUT8 Mask                  */

#define GPIO_DOUT_DOUT9_Pos              (9)                                               /*!< GPIO DOUT: DOUT9 Position              */
#define GPIO_DOUT_DOUT9_Msk              (0x1ul << GPIO_DOUT_DOUT9_Pos)                    /*!< GPIO DOUT: DOUT9 Mask                  */

#define GPIO_DOUT_DOUT10_Pos             (10)                                              /*!< GPIO DOUT: DOUT10 Position             */
#define GPIO_DOUT_DOUT10_Msk             (0x1ul << GPIO_DOUT_DOUT10_Pos)                   /*!< GPIO DOUT: DOUT10 Mask                 */

#define GPIO_DOUT_DOUT11_Pos             (11)                                              /*!< GPIO DOUT: DOUT11 Position             */
#define GPIO_DOUT_DOUT11_Msk             (0x1ul << GPIO_DOUT_DOUT11_Pos)                   /*!< GPIO DOUT: DOUT11 Mask                 */

#define GPIO_DOUT_DOUT12_Pos             (12)                                              /*!< GPIO DOUT: DOUT12 Position             */
#define GPIO_DOUT_DOUT12_Msk             (0x1ul << GPIO_DOUT_DOUT12_Pos)                   /*!< GPIO DOUT: DOUT12 Mask                 */

#define GPIO_DOUT_DOUT13_Pos             (13)                                              /*!< GPIO DOUT: DOUT13 Position             */
#define GPIO_DOUT_DOUT13_Msk             (0x1ul << GPIO_DOUT_DOUT13_Pos)                   /*!< GPIO DOUT: DOUT13 Mask                 */

#define GPIO_DOUT_DOUT14_Pos             (14)                                              /*!< GPIO DOUT: DOUT14 Position             */
#define GPIO_DOUT_DOUT14_Msk             (0x1ul << GPIO_DOUT_DOUT14_Pos)                   /*!< GPIO DOUT: DOUT14 Mask                 */

#define GPIO_DOUT_DOUT15_Pos             (15)                                              /*!< GPIO DOUT: DOUT15 Position             */
#define GPIO_DOUT_DOUT15_Msk             (0x1ul << GPIO_DOUT_DOUT15_Pos)                   /*!< GPIO DOUT: DOUT15 Mask                 */

#define GPIO_DATMSK_DATMSK0_Pos          (0)                                               /*!< GPIO DATMSK: DATMSK0 Position          */
#define GPIO_DATMSK_DATMSK0_Msk          (0x1ul << GPIO_DATMSK_DATMSK0_Pos)                /*!< GPIO DATMSK: DATMSK0 Mask              */

#define GPIO_DATMSK_DATMSK1_Pos          (1)                                               /*!< GPIO DATMSK: DATMSK1 Position          */
#define GPIO_DATMSK_DATMSK1_Msk          (0x1ul << GPIO_DATMSK_DATMSK1_Pos)                /*!< GPIO DATMSK: DATMSK1 Mask              */

#define GPIO_DATMSK_DATMSK2_Pos          (2)                                               /*!< GPIO DATMSK: DATMSK2 Position          */
#define GPIO_DATMSK_DATMSK2_Msk          (0x1ul << GPIO_DATMSK_DATMSK2_Pos)                /*!< GPIO DATMSK: DATMSK2 Mask              */

#define GPIO_DATMSK_DATMSK3_Pos          (3)                                               /*!< GPIO DATMSK: DATMSK3 Position          */
#define GPIO_DATMSK_DATMSK3_Msk          (0x1ul << GPIO_DATMSK_DATMSK3_Pos)                /*!< GPIO DATMSK: DATMSK3 Mask              */

#define GPIO_DATMSK_DATMSK4_Pos          (4)                                               /*!< GPIO DATMSK: DATMSK4 Position          */
#define GPIO_DATMSK_DATMSK4_Msk          (0x1ul << GPIO_DATMSK_DATMSK4_Pos)                /*!< GPIO DATMSK: DATMSK4 Mask              */

#define GPIO_DATMSK_DATMSK5_Pos          (5)                                               /*!< GPIO DATMSK: DATMSK5 Position          */
#define GPIO_DATMSK_DATMSK5_Msk          (0x1ul << GPIO_DATMSK_DATMSK5_Pos)                /*!< GPIO DATMSK: DATMSK5 Mask              */

#define GPIO_DATMSK_DATMSK6_Pos          (6)                                               /*!< GPIO DATMSK: DATMSK6 Position          */
#define GPIO_DATMSK_DATMSK6_Msk          (0x1ul << GPIO_DATMSK_DATMSK6_Pos)                /*!< GPIO DATMSK: DATMSK6 Mask              */

#define GPIO_DATMSK_DATMSK7_Pos          (7)                                               /*!< GPIO DATMSK: DATMSK7 Position          */
#define GPIO_DATMSK_DATMSK7_Msk          (0x1ul << GPIO_DATMSK_DATMSK7_Pos)                /*!< GPIO DATMSK: DATMSK7 Mask              */

#define GPIO_DATMSK_DATMSK8_Pos          (8)                                               /*!< GPIO DATMSK: DATMSK8 Position          */
#define GPIO_DATMSK_DATMSK8_Msk          (0x1ul << GPIO_DATMSK_DATMSK8_Pos)                /*!< GPIO DATMSK: DATMSK8 Mask              */

#define GPIO_DATMSK_DATMSK9_Pos          (9)                                               /*!< GPIO DATMSK: DATMSK9 Position          */
#define GPIO_DATMSK_DATMSK9_Msk          (0x1ul << GPIO_DATMSK_DATMSK9_Pos)                /*!< GPIO DATMSK: DATMSK9 Mask              */

#define GPIO_DATMSK_DATMSK10_Pos         (10)                                              /*!< GPIO DATMSK: DATMSK10 Position         */
#define GPIO_DATMSK_DATMSK10_Msk         (0x1ul << GPIO_DATMSK_DATMSK10_Pos)               /*!< GPIO DATMSK: DATMSK10 Mask             */

#define GPIO_DATMSK_DATMSK11_Pos         (11)                                              /*!< GPIO DATMSK: DATMSK11 Position         */
#define GPIO_DATMSK_DATMSK11_Msk         (0x1ul << GPIO_DATMSK_DATMSK11_Pos)               /*!< GPIO DATMSK: DATMSK11 Mask             */

#define GPIO_DATMSK_DATMSK12_Pos         (12)                                              /*!< GPIO DATMSK: DATMSK12 Position         */
#define GPIO_DATMSK_DATMSK12_Msk         (0x1ul << GPIO_DATMSK_DATMSK12_Pos)               /*!< GPIO DATMSK: DATMSK12 Mask             */

#define GPIO_DATMSK_DATMSK13_Pos         (13)                                              /*!< GPIO DATMSK: DATMSK13 Position         */
#define GPIO_DATMSK_DATMSK13_Msk         (0x1ul << GPIO_DATMSK_DATMSK13_Pos)               /*!< GPIO DATMSK: DATMSK13 Mask             */

#define GPIO_DATMSK_DATMSK14_Pos         (14)                                              /*!< GPIO DATMSK: DATMSK14 Position         */
#define GPIO_DATMSK_DATMSK14_Msk         (0x1ul << GPIO_DATMSK_DATMSK14_Pos)               /*!< GPIO DATMSK: DATMSK14 Mask             */

#define GPIO_DATMSK_DATMSK15_Pos         (15)                                              /*!< GPIO DATMSK: DATMSK15 Position         */
#define GPIO_DATMSK_DATMSK15_Msk         (0x1ul << GPIO_DATMSK_DATMSK15_Pos)               /*!< GPIO DATMSK: DATMSK15 Mask             */

#define GPIO_PIN_PIN0_Pos                (0)                                               /*!< GPIO PIN: PIN0 Position                */
#define GPIO_PIN_PIN0_Msk                (0x1ul << GPIO_PIN_PIN0_Pos)                      /*!< GPIO PIN: PIN0 Mask                    */

#define GPIO_PIN_PIN1_Pos                (1)                                               /*!< GPIO PIN: PIN1 Position                */
#define GPIO_PIN_PIN1_Msk                (0x1ul << GPIO_PIN_PIN1_Pos)                      /*!< GPIO PIN: PIN1 Mask                    */

#define GPIO_PIN_PIN2_Pos                (2)                                               /*!< GPIO PIN: PIN2 Position                */
#define GPIO_PIN_PIN2_Msk                (0x1ul << GPIO_PIN_PIN2_Pos)                      /*!< GPIO PIN: PIN2 Mask                    */

#define GPIO_PIN_PIN3_Pos                (3)                                               /*!< GPIO PIN: PIN3 Position                */
#define GPIO_PIN_PIN3_Msk                (0x1ul << GPIO_PIN_PIN3_Pos)                      /*!< GPIO PIN: PIN3 Mask                    */

#define GPIO_PIN_PIN4_Pos                (4)                                               /*!< GPIO PIN: PIN4 Position                */
#define GPIO_PIN_PIN4_Msk                (0x1ul << GPIO_PIN_PIN4_Pos)                      /*!< GPIO PIN: PIN4 Mask                    */

#define GPIO_PIN_PIN5_Pos                (5)                                               /*!< GPIO PIN: PIN5 Position                */
#define GPIO_PIN_PIN5_Msk                (0x1ul << GPIO_PIN_PIN5_Pos)                      /*!< GPIO PIN: PIN5 Mask                    */

#define GPIO_PIN_PIN6_Pos                (6)                                               /*!< GPIO PIN: PIN6 Position                */
#define GPIO_PIN_PIN6_Msk                (0x1ul << GPIO_PIN_PIN6_Pos)                      /*!< GPIO PIN: PIN6 Mask                    */

#define GPIO_PIN_PIN7_Pos                (7)                                               /*!< GPIO PIN: PIN7 Position                */
#define GPIO_PIN_PIN7_Msk                (0x1ul << GPIO_PIN_PIN7_Pos)                      /*!< GPIO PIN: PIN7 Mask                    */

#define GPIO_PIN_PIN8_Pos                (8)                                               /*!< GPIO PIN: PIN8 Position                */
#define GPIO_PIN_PIN8_Msk                (0x1ul << GPIO_PIN_PIN8_Pos)                      /*!< GPIO PIN: PIN8 Mask                    */

#define GPIO_PIN_PIN9_Pos                (9)                                               /*!< GPIO PIN: PIN9 Position                */
#define GPIO_PIN_PIN9_Msk                (0x1ul << GPIO_PIN_PIN9_Pos)                      /*!< GPIO PIN: PIN9 Mask                    */

#define GPIO_PIN_PIN10_Pos               (10)                                              /*!< GPIO PIN: PIN10 Position               */
#define GPIO_PIN_PIN10_Msk               (0x1ul << GPIO_PIN_PIN10_Pos)                     /*!< GPIO PIN: PIN10 Mask                   */

#define GPIO_PIN_PIN11_Pos               (11)                                              /*!< GPIO PIN: PIN11 Position               */
#define GPIO_PIN_PIN11_Msk               (0x1ul << GPIO_PIN_PIN11_Pos)                     /*!< GPIO PIN: PIN11 Mask                   */

#define GPIO_PIN_PIN12_Pos               (12)                                              /*!< GPIO PIN: PIN12 Position               */
#define GPIO_PIN_PIN12_Msk               (0x1ul << GPIO_PIN_PIN12_Pos)                     /*!< GPIO PIN: PIN12 Mask                   */

#define GPIO_PIN_PIN13_Pos               (13)                                              /*!< GPIO PIN: PIN13 Position               */
#define GPIO_PIN_PIN13_Msk               (0x1ul << GPIO_PIN_PIN13_Pos)                     /*!< GPIO PIN: PIN13 Mask                   */

#define GPIO_PIN_PIN14_Pos               (14)                                              /*!< GPIO PIN: PIN14 Position               */
#define GPIO_PIN_PIN14_Msk               (0x1ul << GPIO_PIN_PIN14_Pos)                     /*!< GPIO PIN: PIN14 Mask                   */

#define GPIO_PIN_PIN15_Pos               (15)                                              /*!< GPIO PIN: PIN15 Position               */
#define GPIO_PIN_PIN15_Msk               (0x1ul << GPIO_PIN_PIN15_Pos)                     /*!< GPIO PIN: PIN15 Mask                   */

#define GPIO_DBEN_DBEN0_Pos              (0)                                               /*!< GPIO DBEN: DBEN0 Position              */
#define GPIO_DBEN_DBEN0_Msk              (0x1ul << GPIO_DBEN_DBEN0_Pos)                    /*!< GPIO DBEN: DBEN0 Mask                  */

#define GPIO_DBEN_DBEN1_Pos              (1)                                               /*!< GPIO DBEN: DBEN1 Position              */
#define GPIO_DBEN_DBEN1_Msk              (0x1ul << GPIO_DBEN_DBEN1_Pos)                    /*!< GPIO DBEN: DBEN1 Mask                  */

#define GPIO_DBEN_DBEN2_Pos              (2)                                               /*!< GPIO DBEN: DBEN2 Position              */
#define GPIO_DBEN_DBEN2_Msk              (0x1ul << GPIO_DBEN_DBEN2_Pos)                    /*!< GPIO DBEN: DBEN2 Mask                  */

#define GPIO_DBEN_DBEN3_Pos              (3)                                               /*!< GPIO DBEN: DBEN3 Position              */
#define GPIO_DBEN_DBEN3_Msk              (0x1ul << GPIO_DBEN_DBEN3_Pos)                    /*!< GPIO DBEN: DBEN3 Mask                  */

#define GPIO_DBEN_DBEN4_Pos              (4)                                               /*!< GPIO DBEN: DBEN4 Position              */
#define GPIO_DBEN_DBEN4_Msk              (0x1ul << GPIO_DBEN_DBEN4_Pos)                    /*!< GPIO DBEN: DBEN4 Mask                  */

#define GPIO_DBEN_DBEN5_Pos              (5)                                               /*!< GPIO DBEN: DBEN5 Position              */
#define GPIO_DBEN_DBEN5_Msk              (0x1ul << GPIO_DBEN_DBEN5_Pos)                    /*!< GPIO DBEN: DBEN5 Mask                  */

#define GPIO_DBEN_DBEN6_Pos              (6)                                               /*!< GPIO DBEN: DBEN6 Position              */
#define GPIO_DBEN_DBEN6_Msk              (0x1ul << GPIO_DBEN_DBEN6_Pos)                    /*!< GPIO DBEN: DBEN6 Mask                  */

#define GPIO_DBEN_DBEN7_Pos              (7)                                               /*!< GPIO DBEN: DBEN7 Position              */
#define GPIO_DBEN_DBEN7_Msk              (0x1ul << GPIO_DBEN_DBEN7_Pos)                    /*!< GPIO DBEN: DBEN7 Mask                  */

#define GPIO_DBEN_DBEN8_Pos              (8)                                               /*!< GPIO DBEN: DBEN8 Position              */
#define GPIO_DBEN_DBEN8_Msk              (0x1ul << GPIO_DBEN_DBEN8_Pos)                    /*!< GPIO DBEN: DBEN8 Mask                  */

#define GPIO_DBEN_DBEN9_Pos              (9)                                               /*!< GPIO DBEN: DBEN9 Position              */
#define GPIO_DBEN_DBEN9_Msk              (0x1ul << GPIO_DBEN_DBEN9_Pos)                    /*!< GPIO DBEN: DBEN9 Mask                  */

#define GPIO_DBEN_DBEN10_Pos             (10)                                              /*!< GPIO DBEN: DBEN10 Position             */
#define GPIO_DBEN_DBEN10_Msk             (0x1ul << GPIO_DBEN_DBEN10_Pos)                   /*!< GPIO DBEN: DBEN10 Mask                 */

#define GPIO_DBEN_DBEN11_Pos             (11)                                              /*!< GPIO DBEN: DBEN11 Position             */
#define GPIO_DBEN_DBEN11_Msk             (0x1ul << GPIO_DBEN_DBEN11_Pos)                   /*!< GPIO DBEN: DBEN11 Mask                 */

#define GPIO_DBEN_DBEN12_Pos             (12)                                              /*!< GPIO DBEN: DBEN12 Position             */
#define GPIO_DBEN_DBEN12_Msk             (0x1ul << GPIO_DBEN_DBEN12_Pos)                   /*!< GPIO DBEN: DBEN12 Mask                 */

#define GPIO_DBEN_DBEN13_Pos             (13)                                              /*!< GPIO DBEN: DBEN13 Position             */
#define GPIO_DBEN_DBEN13_Msk             (0x1ul << GPIO_DBEN_DBEN13_Pos)                   /*!< GPIO DBEN: DBEN13 Mask                 */

#define GPIO_DBEN_DBEN14_Pos             (14)                                              /*!< GPIO DBEN: DBEN14 Position             */
#define GPIO_DBEN_DBEN14_Msk             (0x1ul << GPIO_DBEN_DBEN14_Pos)                   /*!< GPIO DBEN: DBEN14 Mask                 */

#define GPIO_DBEN_DBEN15_Pos             (15)                                              /*!< GPIO DBEN: DBEN15 Position             */
#define GPIO_DBEN_DBEN15_Msk             (0x1ul << GPIO_DBEN_DBEN15_Pos)                   /*!< GPIO DBEN: DBEN15 Mask                 */

#define GPIO_INTTYPE_TYPE0_Pos           (0)                                               /*!< GPIO INTTYPE: TYPE0 Position           */
#define GPIO_INTTYPE_TYPE0_Msk           (0x1ul << GPIO_INTTYPE_TYPE0_Pos)                 /*!< GPIO INTTYPE: TYPE0 Mask               */

#define GPIO_INTTYPE_TYPE1_Pos           (1)                                               /*!< GPIO INTTYPE: TYPE1 Position           */
#define GPIO_INTTYPE_TYPE1_Msk           (0x1ul << GPIO_INTTYPE_TYPE1_Pos)                 /*!< GPIO INTTYPE: TYPE1 Mask               */

#define GPIO_INTTYPE_TYPE2_Pos           (2)                                               /*!< GPIO INTTYPE: TYPE2 Position           */
#define GPIO_INTTYPE_TYPE2_Msk           (0x1ul << GPIO_INTTYPE_TYPE2_Pos)                 /*!< GPIO INTTYPE: TYPE2 Mask               */

#define GPIO_INTTYPE_TYPE3_Pos           (3)                                               /*!< GPIO INTTYPE: TYPE3 Position           */
#define GPIO_INTTYPE_TYPE3_Msk           (0x1ul << GPIO_INTTYPE_TYPE3_Pos)                 /*!< GPIO INTTYPE: TYPE3 Mask               */

#define GPIO_INTTYPE_TYPE4_Pos           (4)                                               /*!< GPIO INTTYPE: TYPE4 Position           */
#define GPIO_INTTYPE_TYPE4_Msk           (0x1ul << GPIO_INTTYPE_TYPE4_Pos)                 /*!< GPIO INTTYPE: TYPE4 Mask               */

#define GPIO_INTTYPE_TYPE5_Pos           (5)                                               /*!< GPIO INTTYPE: TYPE5 Position           */
#define GPIO_INTTYPE_TYPE5_Msk           (0x1ul << GPIO_INTTYPE_TYPE5_Pos)                 /*!< GPIO INTTYPE: TYPE5 Mask               */

#define GPIO_INTTYPE_TYPE6_Pos           (6)                                               /*!< GPIO INTTYPE: TYPE6 Position           */
#define GPIO_INTTYPE_TYPE6_Msk           (0x1ul << GPIO_INTTYPE_TYPE6_Pos)                 /*!< GPIO INTTYPE: TYPE6 Mask               */

#define GPIO_INTTYPE_TYPE7_Pos           (7)                                               /*!< GPIO INTTYPE: TYPE7 Position           */
#define GPIO_INTTYPE_TYPE7_Msk           (0x1ul << GPIO_INTTYPE_TYPE7_Pos)                 /*!< GPIO INTTYPE: TYPE7 Mask               */

#define GPIO_INTTYPE_TYPE8_Pos           (8)                                               /*!< GPIO INTTYPE: TYPE8 Position           */
#define GPIO_INTTYPE_TYPE8_Msk           (0x1ul << GPIO_INTTYPE_TYPE8_Pos)                 /*!< GPIO INTTYPE: TYPE8 Mask               */

#define GPIO_INTTYPE_TYPE9_Pos           (9)                                               /*!< GPIO INTTYPE: TYPE9 Position           */
#define GPIO_INTTYPE_TYPE9_Msk           (0x1ul << GPIO_INTTYPE_TYPE9_Pos)                 /*!< GPIO INTTYPE: TYPE9 Mask               */

#define GPIO_INTTYPE_TYPE10_Pos          (10)                                              /*!< GPIO INTTYPE: TYPE10 Position          */
#define GPIO_INTTYPE_TYPE10_Msk          (0x1ul << GPIO_INTTYPE_TYPE10_Pos)                /*!< GPIO INTTYPE: TYPE10 Mask              */

#define GPIO_INTTYPE_TYPE11_Pos          (11)                                              /*!< GPIO INTTYPE: TYPE11 Position          */
#define GPIO_INTTYPE_TYPE11_Msk          (0x1ul << GPIO_INTTYPE_TYPE11_Pos)                /*!< GPIO INTTYPE: TYPE11 Mask              */

#define GPIO_INTTYPE_TYPE12_Pos          (12)                                              /*!< GPIO INTTYPE: TYPE12 Position          */
#define GPIO_INTTYPE_TYPE12_Msk          (0x1ul << GPIO_INTTYPE_TYPE12_Pos)                /*!< GPIO INTTYPE: TYPE12 Mask              */

#define GPIO_INTTYPE_TYPE13_Pos          (13)                                              /*!< GPIO INTTYPE: TYPE13 Position          */
#define GPIO_INTTYPE_TYPE13_Msk          (0x1ul << GPIO_INTTYPE_TYPE13_Pos)                /*!< GPIO INTTYPE: TYPE13 Mask              */

#define GPIO_INTTYPE_TYPE14_Pos          (14)                                              /*!< GPIO INTTYPE: TYPE14 Position          */
#define GPIO_INTTYPE_TYPE14_Msk          (0x1ul << GPIO_INTTYPE_TYPE14_Pos)                /*!< GPIO INTTYPE: TYPE14 Mask              */

#define GPIO_INTTYPE_TYPE15_Pos          (15)                                              /*!< GPIO INTTYPE: TYPE15 Position          */
#define GPIO_INTTYPE_TYPE15_Msk          (0x1ul << GPIO_INTTYPE_TYPE15_Pos)                /*!< GPIO INTTYPE: TYPE15 Mask              */

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

#define GPIO_INTSRC_INTSRC0_Pos          (0)                                               /*!< GPIO INTSRC: INTSRC0 Position          */
#define GPIO_INTSRC_INTSRC0_Msk          (0x1ul << GPIO_INTSRC_INTSRC0_Pos)                /*!< GPIO INTSRC: INTSRC0 Mask              */

#define GPIO_INTSRC_INTSRC1_Pos          (1)                                               /*!< GPIO INTSRC: INTSRC1 Position          */
#define GPIO_INTSRC_INTSRC1_Msk          (0x1ul << GPIO_INTSRC_INTSRC1_Pos)                /*!< GPIO INTSRC: INTSRC1 Mask              */

#define GPIO_INTSRC_INTSRC2_Pos          (2)                                               /*!< GPIO INTSRC: INTSRC2 Position          */
#define GPIO_INTSRC_INTSRC2_Msk          (0x1ul << GPIO_INTSRC_INTSRC2_Pos)                /*!< GPIO INTSRC: INTSRC2 Mask              */

#define GPIO_INTSRC_INTSRC3_Pos          (3)                                               /*!< GPIO INTSRC: INTSRC3 Position          */
#define GPIO_INTSRC_INTSRC3_Msk          (0x1ul << GPIO_INTSRC_INTSRC3_Pos)                /*!< GPIO INTSRC: INTSRC3 Mask              */

#define GPIO_INTSRC_INTSRC4_Pos          (4)                                               /*!< GPIO INTSRC: INTSRC4 Position          */
#define GPIO_INTSRC_INTSRC4_Msk          (0x1ul << GPIO_INTSRC_INTSRC4_Pos)                /*!< GPIO INTSRC: INTSRC4 Mask              */

#define GPIO_INTSRC_INTSRC5_Pos          (5)                                               /*!< GPIO INTSRC: INTSRC5 Position          */
#define GPIO_INTSRC_INTSRC5_Msk          (0x1ul << GPIO_INTSRC_INTSRC5_Pos)                /*!< GPIO INTSRC: INTSRC5 Mask              */

#define GPIO_INTSRC_INTSRC6_Pos          (6)                                               /*!< GPIO INTSRC: INTSRC6 Position          */
#define GPIO_INTSRC_INTSRC6_Msk          (0x1ul << GPIO_INTSRC_INTSRC6_Pos)                /*!< GPIO INTSRC: INTSRC6 Mask              */

#define GPIO_INTSRC_INTSRC7_Pos          (7)                                               /*!< GPIO INTSRC: INTSRC7 Position          */
#define GPIO_INTSRC_INTSRC7_Msk          (0x1ul << GPIO_INTSRC_INTSRC7_Pos)                /*!< GPIO INTSRC: INTSRC7 Mask              */

#define GPIO_INTSRC_INTSRC8_Pos          (8)                                               /*!< GPIO INTSRC: INTSRC8 Position          */
#define GPIO_INTSRC_INTSRC8_Msk          (0x1ul << GPIO_INTSRC_INTSRC8_Pos)                /*!< GPIO INTSRC: INTSRC8 Mask              */

#define GPIO_INTSRC_INTSRC9_Pos          (9)                                               /*!< GPIO INTSRC: INTSRC9 Position          */
#define GPIO_INTSRC_INTSRC9_Msk          (0x1ul << GPIO_INTSRC_INTSRC9_Pos)                /*!< GPIO INTSRC: INTSRC9 Mask              */

#define GPIO_INTSRC_INTSRC10_Pos         (10)                                              /*!< GPIO INTSRC: INTSRC10 Position         */
#define GPIO_INTSRC_INTSRC10_Msk         (0x1ul << GPIO_INTSRC_INTSRC10_Pos)               /*!< GPIO INTSRC: INTSRC10 Mask             */

#define GPIO_INTSRC_INTSRC11_Pos         (11)                                              /*!< GPIO INTSRC: INTSRC11 Position         */
#define GPIO_INTSRC_INTSRC11_Msk         (0x1ul << GPIO_INTSRC_INTSRC11_Pos)               /*!< GPIO INTSRC: INTSRC11 Mask             */

#define GPIO_INTSRC_INTSRC12_Pos         (12)                                              /*!< GPIO INTSRC: INTSRC12 Position         */
#define GPIO_INTSRC_INTSRC12_Msk         (0x1ul << GPIO_INTSRC_INTSRC12_Pos)               /*!< GPIO INTSRC: INTSRC12 Mask             */

#define GPIO_INTSRC_INTSRC13_Pos         (13)                                              /*!< GPIO INTSRC: INTSRC13 Position         */
#define GPIO_INTSRC_INTSRC13_Msk         (0x1ul << GPIO_INTSRC_INTSRC13_Pos)               /*!< GPIO INTSRC: INTSRC13 Mask             */

#define GPIO_INTSRC_INTSRC14_Pos         (14)                                              /*!< GPIO INTSRC: INTSRC14 Position         */
#define GPIO_INTSRC_INTSRC14_Msk         (0x1ul << GPIO_INTSRC_INTSRC14_Pos)               /*!< GPIO INTSRC: INTSRC14 Mask             */

#define GPIO_INTSRC_INTSRC15_Pos         (15)                                              /*!< GPIO INTSRC: INTSRC15 Position         */
#define GPIO_INTSRC_INTSRC15_Msk         (0x1ul << GPIO_INTSRC_INTSRC15_Pos)               /*!< GPIO INTSRC: INTSRC15 Mask             */

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

#define GPIO_DINOFF_DINOFF16_Pos         (16)                                              /*!< GPIO DINOFF: DINOFF16 Position         */
#define GPIO_DINOFF_DINOFF16_Msk         (0x1ul << GPIO_DINOFF_DINOFF16_Pos)               /*!< GPIO DINOFF: DINOFF16 Mask             */

#define GPIO_DINOFF_DINOFF17_Pos         (17)                                              /*!< GPIO DINOFF: DINOFF17 Position         */
#define GPIO_DINOFF_DINOFF17_Msk         (0x1ul << GPIO_DINOFF_DINOFF17_Pos)               /*!< GPIO DINOFF: DINOFF17 Mask             */

#define GPIO_DINOFF_DINOFF18_Pos         (18)                                              /*!< GPIO DINOFF: DINOFF18 Position         */
#define GPIO_DINOFF_DINOFF18_Msk         (0x1ul << GPIO_DINOFF_DINOFF18_Pos)               /*!< GPIO DINOFF: DINOFF18 Mask             */

#define GPIO_DINOFF_DINOFF19_Pos         (19)                                              /*!< GPIO DINOFF: DINOFF19 Position         */
#define GPIO_DINOFF_DINOFF19_Msk         (0x1ul << GPIO_DINOFF_DINOFF19_Pos)               /*!< GPIO DINOFF: DINOFF19 Mask             */

#define GPIO_DINOFF_DINOFF20_Pos         (20)                                              /*!< GPIO DINOFF: DINOFF20 Position         */
#define GPIO_DINOFF_DINOFF20_Msk         (0x1ul << GPIO_DINOFF_DINOFF20_Pos)               /*!< GPIO DINOFF: DINOFF20 Mask             */

#define GPIO_DINOFF_DINOFF21_Pos         (21)                                              /*!< GPIO DINOFF: DINOFF21 Position         */
#define GPIO_DINOFF_DINOFF21_Msk         (0x1ul << GPIO_DINOFF_DINOFF21_Pos)               /*!< GPIO DINOFF: DINOFF21 Mask             */

#define GPIO_DINOFF_DINOFF22_Pos         (22)                                              /*!< GPIO DINOFF: DINOFF22 Position         */
#define GPIO_DINOFF_DINOFF22_Msk         (0x1ul << GPIO_DINOFF_DINOFF22_Pos)               /*!< GPIO DINOFF: DINOFF22 Mask             */

#define GPIO_DINOFF_DINOFF23_Pos         (23)                                              /*!< GPIO DINOFF: DINOFF23 Position         */
#define GPIO_DINOFF_DINOFF23_Msk         (0x1ul << GPIO_DINOFF_DINOFF23_Pos)               /*!< GPIO DINOFF: DINOFF23 Mask             */

#define GPIO_DINOFF_DINOFF24_Pos         (24)                                              /*!< GPIO DINOFF: DINOFF24 Position         */
#define GPIO_DINOFF_DINOFF24_Msk         (0x1ul << GPIO_DINOFF_DINOFF24_Pos)               /*!< GPIO DINOFF: DINOFF24 Mask             */

#define GPIO_DINOFF_DINOFF25_Pos         (25)                                              /*!< GPIO DINOFF: DINOFF25 Position         */
#define GPIO_DINOFF_DINOFF25_Msk         (0x1ul << GPIO_DINOFF_DINOFF25_Pos)               /*!< GPIO DINOFF: DINOFF25 Mask             */

#define GPIO_DINOFF_DINOFF26_Pos         (26)                                              /*!< GPIO DINOFF: DINOFF26 Position         */
#define GPIO_DINOFF_DINOFF26_Msk         (0x1ul << GPIO_DINOFF_DINOFF26_Pos)               /*!< GPIO DINOFF: DINOFF26 Mask             */

#define GPIO_DINOFF_DINOFF27_Pos         (27)                                              /*!< GPIO DINOFF: DINOFF27 Position         */
#define GPIO_DINOFF_DINOFF27_Msk         (0x1ul << GPIO_DINOFF_DINOFF27_Pos)               /*!< GPIO DINOFF: DINOFF27 Mask             */

#define GPIO_DINOFF_DINOFF28_Pos         (28)                                              /*!< GPIO DINOFF: DINOFF28 Position         */
#define GPIO_DINOFF_DINOFF28_Msk         (0x1ul << GPIO_DINOFF_DINOFF28_Pos)               /*!< GPIO DINOFF: DINOFF28 Mask             */

#define GPIO_DINOFF_DINOFF29_Pos         (29)                                              /*!< GPIO DINOFF: DINOFF29 Position         */
#define GPIO_DINOFF_DINOFF29_Msk         (0x1ul << GPIO_DINOFF_DINOFF29_Pos)               /*!< GPIO DINOFF: DINOFF29 Mask             */

#define GPIO_DINOFF_DINOFF30_Pos         (30)                                              /*!< GPIO DINOFF: DINOFF30 Position         */
#define GPIO_DINOFF_DINOFF30_Msk         (0x1ul << GPIO_DINOFF_DINOFF30_Pos)               /*!< GPIO DINOFF: DINOFF30 Mask             */

#define GPIO_DINOFF_DINOFF31_Pos         (31)                                              /*!< GPIO DINOFF: DINOFF31 Position         */
#define GPIO_DINOFF_DINOFF31_Msk         (0x1ul << GPIO_DINOFF_DINOFF31_Pos)               /*!< GPIO DINOFF: DINOFF31 Mask             */

#define GPIO_DOUT_DOUT0_Pos              (0)                                               /*!< GPIO DOUT: DOUT0 Position              */
#define GPIO_DOUT_DOUT0_Msk              (0x1ul << GPIO_DOUT_DOUT0_Pos)                    /*!< GPIO DOUT: DOUT0 Mask                  */

#define GPIO_DOUT_DOUT1_Pos              (1)                                               /*!< GPIO DOUT: DOUT1 Position              */
#define GPIO_DOUT_DOUT1_Msk              (0x1ul << GPIO_DOUT_DOUT1_Pos)                    /*!< GPIO DOUT: DOUT1 Mask                  */

#define GPIO_DOUT_DOUT2_Pos              (2)                                               /*!< GPIO DOUT: DOUT2 Position              */
#define GPIO_DOUT_DOUT2_Msk              (0x1ul << GPIO_DOUT_DOUT2_Pos)                    /*!< GPIO DOUT: DOUT2 Mask                  */

#define GPIO_DOUT_DOUT3_Pos              (3)                                               /*!< GPIO DOUT: DOUT3 Position              */
#define GPIO_DOUT_DOUT3_Msk              (0x1ul << GPIO_DOUT_DOUT3_Pos)                    /*!< GPIO DOUT: DOUT3 Mask                  */

#define GPIO_DOUT_DOUT4_Pos              (4)                                               /*!< GPIO DOUT: DOUT4 Position              */
#define GPIO_DOUT_DOUT4_Msk              (0x1ul << GPIO_DOUT_DOUT4_Pos)                    /*!< GPIO DOUT: DOUT4 Mask                  */

#define GPIO_DOUT_DOUT5_Pos              (5)                                               /*!< GPIO DOUT: DOUT5 Position              */
#define GPIO_DOUT_DOUT5_Msk              (0x1ul << GPIO_DOUT_DOUT5_Pos)                    /*!< GPIO DOUT: DOUT5 Mask                  */

#define GPIO_DOUT_DOUT6_Pos              (6)                                               /*!< GPIO DOUT: DOUT6 Position              */
#define GPIO_DOUT_DOUT6_Msk              (0x1ul << GPIO_DOUT_DOUT6_Pos)                    /*!< GPIO DOUT: DOUT6 Mask                  */

#define GPIO_DOUT_DOUT7_Pos              (7)                                               /*!< GPIO DOUT: DOUT7 Position              */
#define GPIO_DOUT_DOUT7_Msk              (0x1ul << GPIO_DOUT_DOUT7_Pos)                    /*!< GPIO DOUT: DOUT7 Mask                  */

#define GPIO_DOUT_DOUT8_Pos              (8)                                               /*!< GPIO DOUT: DOUT8 Position              */
#define GPIO_DOUT_DOUT8_Msk              (0x1ul << GPIO_DOUT_DOUT8_Pos)                    /*!< GPIO DOUT: DOUT8 Mask                  */

#define GPIO_DOUT_DOUT9_Pos              (9)                                               /*!< GPIO DOUT: DOUT9 Position              */
#define GPIO_DOUT_DOUT9_Msk              (0x1ul << GPIO_DOUT_DOUT9_Pos)                    /*!< GPIO DOUT: DOUT9 Mask                  */

#define GPIO_DOUT_DOUT10_Pos             (10)                                              /*!< GPIO DOUT: DOUT10 Position             */
#define GPIO_DOUT_DOUT10_Msk             (0x1ul << GPIO_DOUT_DOUT10_Pos)                   /*!< GPIO DOUT: DOUT10 Mask                 */

#define GPIO_DOUT_DOUT11_Pos             (11)                                              /*!< GPIO DOUT: DOUT11 Position             */
#define GPIO_DOUT_DOUT11_Msk             (0x1ul << GPIO_DOUT_DOUT11_Pos)                   /*!< GPIO DOUT: DOUT11 Mask                 */

#define GPIO_DOUT_DOUT12_Pos             (12)                                              /*!< GPIO DOUT: DOUT12 Position             */
#define GPIO_DOUT_DOUT12_Msk             (0x1ul << GPIO_DOUT_DOUT12_Pos)                   /*!< GPIO DOUT: DOUT12 Mask                 */

#define GPIO_DOUT_DOUT13_Pos             (13)                                              /*!< GPIO DOUT: DOUT13 Position             */
#define GPIO_DOUT_DOUT13_Msk             (0x1ul << GPIO_DOUT_DOUT13_Pos)                   /*!< GPIO DOUT: DOUT13 Mask                 */

#define GPIO_DOUT_DOUT14_Pos             (14)                                              /*!< GPIO DOUT: DOUT14 Position             */
#define GPIO_DOUT_DOUT14_Msk             (0x1ul << GPIO_DOUT_DOUT14_Pos)                   /*!< GPIO DOUT: DOUT14 Mask                 */

#define GPIO_DOUT_DOUT15_Pos             (15)                                              /*!< GPIO DOUT: DOUT15 Position             */
#define GPIO_DOUT_DOUT15_Msk             (0x1ul << GPIO_DOUT_DOUT15_Pos)                   /*!< GPIO DOUT: DOUT15 Mask                 */

#define GPIO_DATMSK_DATMSK0_Pos          (0)                                               /*!< GPIO DATMSK: DATMSK0 Position          */
#define GPIO_DATMSK_DATMSK0_Msk          (0x1ul << GPIO_DATMSK_DATMSK0_Pos)                /*!< GPIO DATMSK: DATMSK0 Mask              */

#define GPIO_DATMSK_DATMSK1_Pos          (1)                                               /*!< GPIO DATMSK: DATMSK1 Position          */
#define GPIO_DATMSK_DATMSK1_Msk          (0x1ul << GPIO_DATMSK_DATMSK1_Pos)                /*!< GPIO DATMSK: DATMSK1 Mask              */

#define GPIO_DATMSK_DATMSK2_Pos          (2)                                               /*!< GPIO DATMSK: DATMSK2 Position          */
#define GPIO_DATMSK_DATMSK2_Msk          (0x1ul << GPIO_DATMSK_DATMSK2_Pos)                /*!< GPIO DATMSK: DATMSK2 Mask              */

#define GPIO_DATMSK_DATMSK3_Pos          (3)                                               /*!< GPIO DATMSK: DATMSK3 Position          */
#define GPIO_DATMSK_DATMSK3_Msk          (0x1ul << GPIO_DATMSK_DATMSK3_Pos)                /*!< GPIO DATMSK: DATMSK3 Mask              */

#define GPIO_DATMSK_DATMSK4_Pos          (4)                                               /*!< GPIO DATMSK: DATMSK4 Position          */
#define GPIO_DATMSK_DATMSK4_Msk          (0x1ul << GPIO_DATMSK_DATMSK4_Pos)                /*!< GPIO DATMSK: DATMSK4 Mask              */

#define GPIO_DATMSK_DATMSK5_Pos          (5)                                               /*!< GPIO DATMSK: DATMSK5 Position          */
#define GPIO_DATMSK_DATMSK5_Msk          (0x1ul << GPIO_DATMSK_DATMSK5_Pos)                /*!< GPIO DATMSK: DATMSK5 Mask              */

#define GPIO_DATMSK_DATMSK6_Pos          (6)                                               /*!< GPIO DATMSK: DATMSK6 Position          */
#define GPIO_DATMSK_DATMSK6_Msk          (0x1ul << GPIO_DATMSK_DATMSK6_Pos)                /*!< GPIO DATMSK: DATMSK6 Mask              */

#define GPIO_DATMSK_DATMSK7_Pos          (7)                                               /*!< GPIO DATMSK: DATMSK7 Position          */
#define GPIO_DATMSK_DATMSK7_Msk          (0x1ul << GPIO_DATMSK_DATMSK7_Pos)                /*!< GPIO DATMSK: DATMSK7 Mask              */

#define GPIO_DATMSK_DATMSK8_Pos          (8)                                               /*!< GPIO DATMSK: DATMSK8 Position          */
#define GPIO_DATMSK_DATMSK8_Msk          (0x1ul << GPIO_DATMSK_DATMSK8_Pos)                /*!< GPIO DATMSK: DATMSK8 Mask              */

#define GPIO_DATMSK_DATMSK9_Pos          (9)                                               /*!< GPIO DATMSK: DATMSK9 Position          */
#define GPIO_DATMSK_DATMSK9_Msk          (0x1ul << GPIO_DATMSK_DATMSK9_Pos)                /*!< GPIO DATMSK: DATMSK9 Mask              */

#define GPIO_DATMSK_DATMSK10_Pos         (10)                                              /*!< GPIO DATMSK: DATMSK10 Position         */
#define GPIO_DATMSK_DATMSK10_Msk         (0x1ul << GPIO_DATMSK_DATMSK10_Pos)               /*!< GPIO DATMSK: DATMSK10 Mask             */

#define GPIO_DATMSK_DATMSK11_Pos         (11)                                              /*!< GPIO DATMSK: DATMSK11 Position         */
#define GPIO_DATMSK_DATMSK11_Msk         (0x1ul << GPIO_DATMSK_DATMSK11_Pos)               /*!< GPIO DATMSK: DATMSK11 Mask             */

#define GPIO_DATMSK_DATMSK12_Pos         (12)                                              /*!< GPIO DATMSK: DATMSK12 Position         */
#define GPIO_DATMSK_DATMSK12_Msk         (0x1ul << GPIO_DATMSK_DATMSK12_Pos)               /*!< GPIO DATMSK: DATMSK12 Mask             */

#define GPIO_DATMSK_DATMSK13_Pos         (13)                                              /*!< GPIO DATMSK: DATMSK13 Position         */
#define GPIO_DATMSK_DATMSK13_Msk         (0x1ul << GPIO_DATMSK_DATMSK13_Pos)               /*!< GPIO DATMSK: DATMSK13 Mask             */

#define GPIO_DATMSK_DATMSK14_Pos         (14)                                              /*!< GPIO DATMSK: DATMSK14 Position         */
#define GPIO_DATMSK_DATMSK14_Msk         (0x1ul << GPIO_DATMSK_DATMSK14_Pos)               /*!< GPIO DATMSK: DATMSK14 Mask             */

#define GPIO_DATMSK_DATMSK15_Pos         (15)                                              /*!< GPIO DATMSK: DATMSK15 Position         */
#define GPIO_DATMSK_DATMSK15_Msk         (0x1ul << GPIO_DATMSK_DATMSK15_Pos)               /*!< GPIO DATMSK: DATMSK15 Mask             */

#define GPIO_PIN_PIN0_Pos                (0)                                               /*!< GPIO PIN: PIN0 Position                */
#define GPIO_PIN_PIN0_Msk                (0x1ul << GPIO_PIN_PIN0_Pos)                      /*!< GPIO PIN: PIN0 Mask                    */

#define GPIO_PIN_PIN1_Pos                (1)                                               /*!< GPIO PIN: PIN1 Position                */
#define GPIO_PIN_PIN1_Msk                (0x1ul << GPIO_PIN_PIN1_Pos)                      /*!< GPIO PIN: PIN1 Mask                    */

#define GPIO_PIN_PIN2_Pos                (2)                                               /*!< GPIO PIN: PIN2 Position                */
#define GPIO_PIN_PIN2_Msk                (0x1ul << GPIO_PIN_PIN2_Pos)                      /*!< GPIO PIN: PIN2 Mask                    */

#define GPIO_PIN_PIN3_Pos                (3)                                               /*!< GPIO PIN: PIN3 Position                */
#define GPIO_PIN_PIN3_Msk                (0x1ul << GPIO_PIN_PIN3_Pos)                      /*!< GPIO PIN: PIN3 Mask                    */

#define GPIO_PIN_PIN4_Pos                (4)                                               /*!< GPIO PIN: PIN4 Position                */
#define GPIO_PIN_PIN4_Msk                (0x1ul << GPIO_PIN_PIN4_Pos)                      /*!< GPIO PIN: PIN4 Mask                    */

#define GPIO_PIN_PIN5_Pos                (5)                                               /*!< GPIO PIN: PIN5 Position                */
#define GPIO_PIN_PIN5_Msk                (0x1ul << GPIO_PIN_PIN5_Pos)                      /*!< GPIO PIN: PIN5 Mask                    */

#define GPIO_PIN_PIN6_Pos                (6)                                               /*!< GPIO PIN: PIN6 Position                */
#define GPIO_PIN_PIN6_Msk                (0x1ul << GPIO_PIN_PIN6_Pos)                      /*!< GPIO PIN: PIN6 Mask                    */

#define GPIO_PIN_PIN7_Pos                (7)                                               /*!< GPIO PIN: PIN7 Position                */
#define GPIO_PIN_PIN7_Msk                (0x1ul << GPIO_PIN_PIN7_Pos)                      /*!< GPIO PIN: PIN7 Mask                    */

#define GPIO_PIN_PIN8_Pos                (8)                                               /*!< GPIO PIN: PIN8 Position                */
#define GPIO_PIN_PIN8_Msk                (0x1ul << GPIO_PIN_PIN8_Pos)                      /*!< GPIO PIN: PIN8 Mask                    */

#define GPIO_PIN_PIN9_Pos                (9)                                               /*!< GPIO PIN: PIN9 Position                */
#define GPIO_PIN_PIN9_Msk                (0x1ul << GPIO_PIN_PIN9_Pos)                      /*!< GPIO PIN: PIN9 Mask                    */

#define GPIO_PIN_PIN10_Pos               (10)                                              /*!< GPIO PIN: PIN10 Position               */
#define GPIO_PIN_PIN10_Msk               (0x1ul << GPIO_PIN_PIN10_Pos)                     /*!< GPIO PIN: PIN10 Mask                   */

#define GPIO_PIN_PIN11_Pos               (11)                                              /*!< GPIO PIN: PIN11 Position               */
#define GPIO_PIN_PIN11_Msk               (0x1ul << GPIO_PIN_PIN11_Pos)                     /*!< GPIO PIN: PIN11 Mask                   */

#define GPIO_PIN_PIN12_Pos               (12)                                              /*!< GPIO PIN: PIN12 Position               */
#define GPIO_PIN_PIN12_Msk               (0x1ul << GPIO_PIN_PIN12_Pos)                     /*!< GPIO PIN: PIN12 Mask                   */

#define GPIO_PIN_PIN13_Pos               (13)                                              /*!< GPIO PIN: PIN13 Position               */
#define GPIO_PIN_PIN13_Msk               (0x1ul << GPIO_PIN_PIN13_Pos)                     /*!< GPIO PIN: PIN13 Mask                   */

#define GPIO_PIN_PIN14_Pos               (14)                                              /*!< GPIO PIN: PIN14 Position               */
#define GPIO_PIN_PIN14_Msk               (0x1ul << GPIO_PIN_PIN14_Pos)                     /*!< GPIO PIN: PIN14 Mask                   */

#define GPIO_PIN_PIN15_Pos               (15)                                              /*!< GPIO PIN: PIN15 Position               */
#define GPIO_PIN_PIN15_Msk               (0x1ul << GPIO_PIN_PIN15_Pos)                     /*!< GPIO PIN: PIN15 Mask                   */

#define GPIO_DBEN_DBEN0_Pos              (0)                                               /*!< GPIO DBEN: DBEN0 Position              */
#define GPIO_DBEN_DBEN0_Msk              (0x1ul << GPIO_DBEN_DBEN0_Pos)                    /*!< GPIO DBEN: DBEN0 Mask                  */

#define GPIO_DBEN_DBEN1_Pos              (1)                                               /*!< GPIO DBEN: DBEN1 Position              */
#define GPIO_DBEN_DBEN1_Msk              (0x1ul << GPIO_DBEN_DBEN1_Pos)                    /*!< GPIO DBEN: DBEN1 Mask                  */

#define GPIO_DBEN_DBEN2_Pos              (2)                                               /*!< GPIO DBEN: DBEN2 Position              */
#define GPIO_DBEN_DBEN2_Msk              (0x1ul << GPIO_DBEN_DBEN2_Pos)                    /*!< GPIO DBEN: DBEN2 Mask                  */

#define GPIO_DBEN_DBEN3_Pos              (3)                                               /*!< GPIO DBEN: DBEN3 Position              */
#define GPIO_DBEN_DBEN3_Msk              (0x1ul << GPIO_DBEN_DBEN3_Pos)                    /*!< GPIO DBEN: DBEN3 Mask                  */

#define GPIO_DBEN_DBEN4_Pos              (4)                                               /*!< GPIO DBEN: DBEN4 Position              */
#define GPIO_DBEN_DBEN4_Msk              (0x1ul << GPIO_DBEN_DBEN4_Pos)                    /*!< GPIO DBEN: DBEN4 Mask                  */

#define GPIO_DBEN_DBEN5_Pos              (5)                                               /*!< GPIO DBEN: DBEN5 Position              */
#define GPIO_DBEN_DBEN5_Msk              (0x1ul << GPIO_DBEN_DBEN5_Pos)                    /*!< GPIO DBEN: DBEN5 Mask                  */

#define GPIO_DBEN_DBEN6_Pos              (6)                                               /*!< GPIO DBEN: DBEN6 Position              */
#define GPIO_DBEN_DBEN6_Msk              (0x1ul << GPIO_DBEN_DBEN6_Pos)                    /*!< GPIO DBEN: DBEN6 Mask                  */

#define GPIO_DBEN_DBEN7_Pos              (7)                                               /*!< GPIO DBEN: DBEN7 Position              */
#define GPIO_DBEN_DBEN7_Msk              (0x1ul << GPIO_DBEN_DBEN7_Pos)                    /*!< GPIO DBEN: DBEN7 Mask                  */

#define GPIO_DBEN_DBEN8_Pos              (8)                                               /*!< GPIO DBEN: DBEN8 Position              */
#define GPIO_DBEN_DBEN8_Msk              (0x1ul << GPIO_DBEN_DBEN8_Pos)                    /*!< GPIO DBEN: DBEN8 Mask                  */

#define GPIO_DBEN_DBEN9_Pos              (9)                                               /*!< GPIO DBEN: DBEN9 Position              */
#define GPIO_DBEN_DBEN9_Msk              (0x1ul << GPIO_DBEN_DBEN9_Pos)                    /*!< GPIO DBEN: DBEN9 Mask                  */

#define GPIO_DBEN_DBEN10_Pos             (10)                                              /*!< GPIO DBEN: DBEN10 Position             */
#define GPIO_DBEN_DBEN10_Msk             (0x1ul << GPIO_DBEN_DBEN10_Pos)                   /*!< GPIO DBEN: DBEN10 Mask                 */

#define GPIO_DBEN_DBEN11_Pos             (11)                                              /*!< GPIO DBEN: DBEN11 Position             */
#define GPIO_DBEN_DBEN11_Msk             (0x1ul << GPIO_DBEN_DBEN11_Pos)                   /*!< GPIO DBEN: DBEN11 Mask                 */

#define GPIO_DBEN_DBEN12_Pos             (12)                                              /*!< GPIO DBEN: DBEN12 Position             */
#define GPIO_DBEN_DBEN12_Msk             (0x1ul << GPIO_DBEN_DBEN12_Pos)                   /*!< GPIO DBEN: DBEN12 Mask                 */

#define GPIO_DBEN_DBEN13_Pos             (13)                                              /*!< GPIO DBEN: DBEN13 Position             */
#define GPIO_DBEN_DBEN13_Msk             (0x1ul << GPIO_DBEN_DBEN13_Pos)                   /*!< GPIO DBEN: DBEN13 Mask                 */

#define GPIO_DBEN_DBEN14_Pos             (14)                                              /*!< GPIO DBEN: DBEN14 Position             */
#define GPIO_DBEN_DBEN14_Msk             (0x1ul << GPIO_DBEN_DBEN14_Pos)                   /*!< GPIO DBEN: DBEN14 Mask                 */

#define GPIO_DBEN_DBEN15_Pos             (15)                                              /*!< GPIO DBEN: DBEN15 Position             */
#define GPIO_DBEN_DBEN15_Msk             (0x1ul << GPIO_DBEN_DBEN15_Pos)                   /*!< GPIO DBEN: DBEN15 Mask                 */

#define GPIO_INTTYPE_TYPE0_Pos           (0)                                               /*!< GPIO INTTYPE: TYPE0 Position           */
#define GPIO_INTTYPE_TYPE0_Msk           (0x1ul << GPIO_INTTYPE_TYPE0_Pos)                 /*!< GPIO INTTYPE: TYPE0 Mask               */

#define GPIO_INTTYPE_TYPE1_Pos           (1)                                               /*!< GPIO INTTYPE: TYPE1 Position           */
#define GPIO_INTTYPE_TYPE1_Msk           (0x1ul << GPIO_INTTYPE_TYPE1_Pos)                 /*!< GPIO INTTYPE: TYPE1 Mask               */

#define GPIO_INTTYPE_TYPE2_Pos           (2)                                               /*!< GPIO INTTYPE: TYPE2 Position           */
#define GPIO_INTTYPE_TYPE2_Msk           (0x1ul << GPIO_INTTYPE_TYPE2_Pos)                 /*!< GPIO INTTYPE: TYPE2 Mask               */

#define GPIO_INTTYPE_TYPE3_Pos           (3)                                               /*!< GPIO INTTYPE: TYPE3 Position           */
#define GPIO_INTTYPE_TYPE3_Msk           (0x1ul << GPIO_INTTYPE_TYPE3_Pos)                 /*!< GPIO INTTYPE: TYPE3 Mask               */

#define GPIO_INTTYPE_TYPE4_Pos           (4)                                               /*!< GPIO INTTYPE: TYPE4 Position           */
#define GPIO_INTTYPE_TYPE4_Msk           (0x1ul << GPIO_INTTYPE_TYPE4_Pos)                 /*!< GPIO INTTYPE: TYPE4 Mask               */

#define GPIO_INTTYPE_TYPE5_Pos           (5)                                               /*!< GPIO INTTYPE: TYPE5 Position           */
#define GPIO_INTTYPE_TYPE5_Msk           (0x1ul << GPIO_INTTYPE_TYPE5_Pos)                 /*!< GPIO INTTYPE: TYPE5 Mask               */

#define GPIO_INTTYPE_TYPE6_Pos           (6)                                               /*!< GPIO INTTYPE: TYPE6 Position           */
#define GPIO_INTTYPE_TYPE6_Msk           (0x1ul << GPIO_INTTYPE_TYPE6_Pos)                 /*!< GPIO INTTYPE: TYPE6 Mask               */

#define GPIO_INTTYPE_TYPE7_Pos           (7)                                               /*!< GPIO INTTYPE: TYPE7 Position           */
#define GPIO_INTTYPE_TYPE7_Msk           (0x1ul << GPIO_INTTYPE_TYPE7_Pos)                 /*!< GPIO INTTYPE: TYPE7 Mask               */

#define GPIO_INTTYPE_TYPE8_Pos           (8)                                               /*!< GPIO INTTYPE: TYPE8 Position           */
#define GPIO_INTTYPE_TYPE8_Msk           (0x1ul << GPIO_INTTYPE_TYPE8_Pos)                 /*!< GPIO INTTYPE: TYPE8 Mask               */

#define GPIO_INTTYPE_TYPE9_Pos           (9)                                               /*!< GPIO INTTYPE: TYPE9 Position           */
#define GPIO_INTTYPE_TYPE9_Msk           (0x1ul << GPIO_INTTYPE_TYPE9_Pos)                 /*!< GPIO INTTYPE: TYPE9 Mask               */

#define GPIO_INTTYPE_TYPE10_Pos          (10)                                              /*!< GPIO INTTYPE: TYPE10 Position          */
#define GPIO_INTTYPE_TYPE10_Msk          (0x1ul << GPIO_INTTYPE_TYPE10_Pos)                /*!< GPIO INTTYPE: TYPE10 Mask              */

#define GPIO_INTTYPE_TYPE11_Pos          (11)                                              /*!< GPIO INTTYPE: TYPE11 Position          */
#define GPIO_INTTYPE_TYPE11_Msk          (0x1ul << GPIO_INTTYPE_TYPE11_Pos)                /*!< GPIO INTTYPE: TYPE11 Mask              */

#define GPIO_INTTYPE_TYPE12_Pos          (12)                                              /*!< GPIO INTTYPE: TYPE12 Position          */
#define GPIO_INTTYPE_TYPE12_Msk          (0x1ul << GPIO_INTTYPE_TYPE12_Pos)                /*!< GPIO INTTYPE: TYPE12 Mask              */

#define GPIO_INTTYPE_TYPE13_Pos          (13)                                              /*!< GPIO INTTYPE: TYPE13 Position          */
#define GPIO_INTTYPE_TYPE13_Msk          (0x1ul << GPIO_INTTYPE_TYPE13_Pos)                /*!< GPIO INTTYPE: TYPE13 Mask              */

#define GPIO_INTTYPE_TYPE14_Pos          (14)                                              /*!< GPIO INTTYPE: TYPE14 Position          */
#define GPIO_INTTYPE_TYPE14_Msk          (0x1ul << GPIO_INTTYPE_TYPE14_Pos)                /*!< GPIO INTTYPE: TYPE14 Mask              */

#define GPIO_INTTYPE_TYPE15_Pos          (15)                                              /*!< GPIO INTTYPE: TYPE15 Position          */
#define GPIO_INTTYPE_TYPE15_Msk          (0x1ul << GPIO_INTTYPE_TYPE15_Pos)                /*!< GPIO INTTYPE: TYPE15 Mask              */

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

#define GPIO_INTSRC_INTSRC0_Pos          (0)                                               /*!< GPIO INTSRC: INTSRC0 Position          */
#define GPIO_INTSRC_INTSRC0_Msk          (0x1ul << GPIO_INTSRC_INTSRC0_Pos)                /*!< GPIO INTSRC: INTSRC0 Mask              */

#define GPIO_INTSRC_INTSRC1_Pos          (1)                                               /*!< GPIO INTSRC: INTSRC1 Position          */
#define GPIO_INTSRC_INTSRC1_Msk          (0x1ul << GPIO_INTSRC_INTSRC1_Pos)                /*!< GPIO INTSRC: INTSRC1 Mask              */

#define GPIO_INTSRC_INTSRC2_Pos          (2)                                               /*!< GPIO INTSRC: INTSRC2 Position          */
#define GPIO_INTSRC_INTSRC2_Msk          (0x1ul << GPIO_INTSRC_INTSRC2_Pos)                /*!< GPIO INTSRC: INTSRC2 Mask              */

#define GPIO_INTSRC_INTSRC3_Pos          (3)                                               /*!< GPIO INTSRC: INTSRC3 Position          */
#define GPIO_INTSRC_INTSRC3_Msk          (0x1ul << GPIO_INTSRC_INTSRC3_Pos)                /*!< GPIO INTSRC: INTSRC3 Mask              */

#define GPIO_INTSRC_INTSRC4_Pos          (4)                                               /*!< GPIO INTSRC: INTSRC4 Position          */
#define GPIO_INTSRC_INTSRC4_Msk          (0x1ul << GPIO_INTSRC_INTSRC4_Pos)                /*!< GPIO INTSRC: INTSRC4 Mask              */

#define GPIO_INTSRC_INTSRC5_Pos          (5)                                               /*!< GPIO INTSRC: INTSRC5 Position          */
#define GPIO_INTSRC_INTSRC5_Msk          (0x1ul << GPIO_INTSRC_INTSRC5_Pos)                /*!< GPIO INTSRC: INTSRC5 Mask              */

#define GPIO_INTSRC_INTSRC6_Pos          (6)                                               /*!< GPIO INTSRC: INTSRC6 Position          */
#define GPIO_INTSRC_INTSRC6_Msk          (0x1ul << GPIO_INTSRC_INTSRC6_Pos)                /*!< GPIO INTSRC: INTSRC6 Mask              */

#define GPIO_INTSRC_INTSRC7_Pos          (7)                                               /*!< GPIO INTSRC: INTSRC7 Position          */
#define GPIO_INTSRC_INTSRC7_Msk          (0x1ul << GPIO_INTSRC_INTSRC7_Pos)                /*!< GPIO INTSRC: INTSRC7 Mask              */

#define GPIO_INTSRC_INTSRC8_Pos          (8)                                               /*!< GPIO INTSRC: INTSRC8 Position          */
#define GPIO_INTSRC_INTSRC8_Msk          (0x1ul << GPIO_INTSRC_INTSRC8_Pos)                /*!< GPIO INTSRC: INTSRC8 Mask              */

#define GPIO_INTSRC_INTSRC9_Pos          (9)                                               /*!< GPIO INTSRC: INTSRC9 Position          */
#define GPIO_INTSRC_INTSRC9_Msk          (0x1ul << GPIO_INTSRC_INTSRC9_Pos)                /*!< GPIO INTSRC: INTSRC9 Mask              */

#define GPIO_INTSRC_INTSRC10_Pos         (10)                                              /*!< GPIO INTSRC: INTSRC10 Position         */
#define GPIO_INTSRC_INTSRC10_Msk         (0x1ul << GPIO_INTSRC_INTSRC10_Pos)               /*!< GPIO INTSRC: INTSRC10 Mask             */

#define GPIO_INTSRC_INTSRC11_Pos         (11)                                              /*!< GPIO INTSRC: INTSRC11 Position         */
#define GPIO_INTSRC_INTSRC11_Msk         (0x1ul << GPIO_INTSRC_INTSRC11_Pos)               /*!< GPIO INTSRC: INTSRC11 Mask             */

#define GPIO_INTSRC_INTSRC12_Pos         (12)                                              /*!< GPIO INTSRC: INTSRC12 Position         */
#define GPIO_INTSRC_INTSRC12_Msk         (0x1ul << GPIO_INTSRC_INTSRC12_Pos)               /*!< GPIO INTSRC: INTSRC12 Mask             */

#define GPIO_INTSRC_INTSRC13_Pos         (13)                                              /*!< GPIO INTSRC: INTSRC13 Position         */
#define GPIO_INTSRC_INTSRC13_Msk         (0x1ul << GPIO_INTSRC_INTSRC13_Pos)               /*!< GPIO INTSRC: INTSRC13 Mask             */

#define GPIO_INTSRC_INTSRC14_Pos         (14)                                              /*!< GPIO INTSRC: INTSRC14 Position         */
#define GPIO_INTSRC_INTSRC14_Msk         (0x1ul << GPIO_INTSRC_INTSRC14_Pos)               /*!< GPIO INTSRC: INTSRC14 Mask             */

#define GPIO_INTSRC_INTSRC15_Pos         (15)                                              /*!< GPIO INTSRC: INTSRC15 Position         */
#define GPIO_INTSRC_INTSRC15_Msk         (0x1ul << GPIO_INTSRC_INTSRC15_Pos)               /*!< GPIO INTSRC: INTSRC15 Mask             */

#define GPIO_DBCTL_DBCLKSEL_Pos          (0)                                               /*!< GPIO DBCTL: DBCLKSEL Position          */
#define GPIO_DBCTL_DBCLKSEL_Msk          (0xful << GPIO_DBCTL_DBCLKSEL_Pos)                /*!< GPIO DBCTL: DBCLKSEL Mask              */

#define GPIO_DBCTL_DBCLKSRC_Pos          (4)                                               /*!< GPIO DBCTL: DBCLKSRC Position          */
#define GPIO_DBCTL_DBCLKSRC_Msk          (0x1ul << GPIO_DBCTL_DBCLKSRC_Pos)                /*!< GPIO DBCTL: DBCLKSRC Mask              */

#define GPIO_DBCTL_ICLKON_Pos            (5)                                               /*!< GPIO DBCTL: ICLKON Position            */
#define GPIO_DBCTL_ICLKON_Msk            (0x1ul << GPIO_DBCTL_ICLKON_Pos)                  /*!< GPIO DBCTL: ICLKON Mask                */

/**@}*/ /* GPIO_CONST */
/**@}*/ /* end of GPIO register group */


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
     * |        |          |When AA=1 prior to address or data received, an acknowledge (ACK - low level to SDA) will be returned during the acknowledge clock pulse on the SCL line when:
     * |        |          |1. A slave is acknowledging the address sent from master,
     * |        |          |2. The receiver devices are acknowledging the data sent by transmitter.
     * |        |          |When AA = 0 prior to address or data received, a Not acknowledged (high level to SDA) will be returned during the acknowledge clock pulse on the SCL line
     * |[3]     |SI        |I2C Interrupt Flag
     * |        |          |When a new SIO state is present in the I2C_STATUS register, the SI flag is set by hardware, and if bit EI ( I2C_CTL[7]) is set, the I2C interrupt is requested.
     * |        |          |SI must be cleared by software.
     * |        |          |Clear SI is by writing one to this bit.
     * |[4]     |STO       |I2C STOP Control Bit
     * |        |          |In master mode, set STO to transmit a STOP condition to bus.
     * |        |          |I2C hardware will check the bus condition, when a STOP condition is detected this bit will be cleared by hardware automatically.
     * |        |          |In slave mode, setting STO resets I2C hardware to the defined "not addressed" slave mode.
     * |        |          |This means it is NO LONGER in the slave receiver mode able receive data from the master transmit device.
     * |[5]     |STA       |I2C START Control Bit
     * |        |          |Setting STA to logic 1 will enter master mode, the I2C hardware sends a START or repeat START condition to bus when the bus is free.
     * |[6]     |I2CEN     |I2C Controller Enable Bit
     * |        |          |0 = Disable
     * |        |          |1 = Enable
     * |        |          |Set to enable I2C serial function block. 
     * |[7]     |INTEN     |Enable Interrupt
     * |        |          |0 = Disable interrupt.
     * |        |          |1 = Enable interrupt CPU.
 */
    __IO uint32_t CTL;                   

    /**
     * ADDR0
     * ===================================================================================================
     * Offset: 0x04  I2C Slave address Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = Disable General Call Function.
     * |        |          |1 = Enable General Call Function.
     * |[1:7]   |ADDR      |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if any of the addresses are matched.
 */
    __IO uint32_t ADDR0;                 

    /**
     * DAT
     * ===================================================================================================
     * Offset: 0x08  I2C DATA Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |DAT       |I2C Data Register
     * |        |          |During master or slave transmit mode, data to be transmitted is written to this register.
     * |        |          |During master or slave receive mode, data that has been received may be read from this register.
 */
    __IO uint32_t DAT;                   

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x0C  I2C Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |STATUS    |I2C Status Register
     * |        |          |The status register of I2C:
     * |        |          |The three least significant bits are always 0.
     * |        |          |The five most significant bits contain the status code.
     * |        |          |There are 26 possible status codes.
     * |        |          |When STATUS contains F8H, no serial interrupt is requested.
     * |        |          |All other STATUS values correspond to defined I2C states.
     * |        |          |When each of these states is entered, a status interrupt is requested (SI = 1).
     * |        |          |A valid status code is present in STATUS one PCLK cycle after SI is set by hardware and is still present one PCLK cycle after SI has been reset by software.
     * |        |          |In addition, states 00H stands for a Bus Error.
     * |        |          |A Bus Error occurs when a START or STOP condition is present at an illegal position in the frame.
     * |        |          |Example of illegal position are during the serial transfer of an address byte, a data byte or an acknowledge bit.
 */
    __I  uint32_t STATUS;                

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x10  I2C clock divided Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |DIVIDER   |I2C Clock Divided Register
     * |        |          |The I2C clock rate bits: Data Baud Rate of I2C = PCLK /(4x( CLK+1)).
 */
    __IO uint32_t CLKDIV;                

    /**
     * TOCTL
     * ===================================================================================================
     * Offset: 0x14  I2C Time out control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TOIF      |Time-Out Flag
     * |        |          |0 = No time-out.
     * |        |          |1 = Time-out flag is set by H/W. It can interrupt CPU. Write 1 to clear.
     * |[1]     |TOCDIV4   |Time-Out Counter Input Clock Divide By 4
     * |        |          |0 = Disable
     * |        |          |1 = Enable
     * |        |          |When enabled, the time-out clock is PCLK/4.
     * |[2]     |TOCEN     |Time-out Counter Control Bit
     * |        |          |0 = Disable
     * |        |          |1 = Enable
     * |        |          |When enabled, the 14 bit time-out counter will start counting when SI is clear.
     * |        |          |Setting flag SI to high will reset counter and re-start up counting after SI is cleared.
 */
    __IO uint32_t TOCTL;                 

    /**
     * ADDR1
     * ===================================================================================================
     * Offset: 0x18  I2C Slave address Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = Disable General Call Function.
     * |        |          |1 = Enable General Call Function.
     * |[1:7]   |ADDR      |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if any of the addresses are matched.
 */
    __IO uint32_t ADDR1;                 

    /**
     * ADDR2
     * ===================================================================================================
     * Offset: 0x1C  I2C Slave address Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = Disable General Call Function.
     * |        |          |1 = Enable General Call Function.
     * |[1:7]   |ADDR      |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if any of the addresses are matched.
 */
    __IO uint32_t ADDR2;                 

    /**
     * ADDR3
     * ===================================================================================================
     * Offset: 0x20  I2C Slave address Register3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |GC        |General Call Function
     * |        |          |0 = Disable General Call Function.
     * |        |          |1 = Enable General Call Function.
     * |[1:7]   |ADDR      |I2C Address Register
     * |        |          |The content of this register is irrelevant when I2C is in master mode.
     * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCU's own address.
     * |        |          |The I2C hardware will react if any of the addresses are matched.
 */
    __IO uint32_t ADDR3;                 

    /**
     * ADDRMSK0
     * ===================================================================================================
     * Offset: 0x24  I2C Slave address Mask Register0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |ADDRMSK1  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[2]     |ADDRMSK2  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[3]     |ADDRMSK3  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[4]     |ADDRMSK4  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[5]     |ADDRMSK5  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[6]     |ADDRMSK6  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[7]     |ADDRMSK7  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
 */
    __IO uint32_t ADDRMSK0;              

    /**
     * ADDRMSK1
     * ===================================================================================================
     * Offset: 0x28  I2C Slave address Mask Register1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |ADDRMSK1  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[2]     |ADDRMSK2  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[3]     |ADDRMSK3  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[4]     |ADDRMSK4  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[5]     |ADDRMSK5  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[6]     |ADDRMSK6  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[7]     |ADDRMSK7  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
 */
    __IO uint32_t ADDRMSK1;              

    /**
     * ADDRMSK2
     * ===================================================================================================
     * Offset: 0x2C  I2C Slave address Mask Register2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |ADDRMSK1  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[2]     |ADDRMSK2  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[3]     |ADDRMSK3  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[4]     |ADDRMSK4  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[5]     |ADDRMSK5  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[6]     |ADDRMSK6  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[7]     |ADDRMSK7  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
 */
    __IO uint32_t ADDRMSK2;              

    /**
     * ADDRMSK3
     * ===================================================================================================
     * Offset: 0x30  I2C Slave address Mask Register3
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[1]     |ADDRMSK1  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[2]     |ADDRMSK2  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[3]     |ADDRMSK3  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[4]     |ADDRMSK4  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[5]     |ADDRMSK5  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[6]     |ADDRMSK6  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
     * |[7]     |ADDRMSK7  |I2C Address Mask register
     * |        |          |0 = Mask disable.
     * |        |          |1 = Mask enable (the received corresponding address bit is don't care.)
     * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers.
     * |        |          |Bits in this field mask the I2C_ADDRx registers.
     * |        |          |masking bits from the address comparison.
 */
    __IO uint32_t ADDRMSK3;              

} I2C_T;

/**
    @addtogroup I2C_CONST I2C Bit Field Definition
    Constant Definitions for I2C Controller
@{ */

#define I2C_CTL_AA_Pos                   (2)                                               /*!< I2C CTL: AA Position                   */
#define I2C_CTL_AA_Msk                   (0x1ul << I2C_CTL_AA_Pos)                         /*!< I2C CTL: AA Mask                       */

#define I2C_CTL_SI_Pos                   (3)                                               /*!< I2C CTL: SI Position                   */
#define I2C_CTL_SI_Msk                   (0x1ul << I2C_CTL_SI_Pos)                         /*!< I2C CTL: SI Mask                       */

#define I2C_CTL_STO_Pos                  (4)                                               /*!< I2C CTL: STO Position                  */
#define I2C_CTL_STO_Msk                  (0x1ul << I2C_CTL_STO_Pos)                        /*!< I2C CTL: STO Mask                      */

#define I2C_CTL_STA_Pos                  (5)                                               /*!< I2C CTL: STA Position                  */
#define I2C_CTL_STA_Msk                  (0x1ul << I2C_CTL_STA_Pos)                        /*!< I2C CTL: STA Mask                      */

#define I2C_CTL_I2CEN_Pos                (6)                                               /*!< I2C CTL: I2CEN Position                */
#define I2C_CTL_I2CEN_Msk                (0x1ul << I2C_CTL_I2CEN_Pos)                      /*!< I2C CTL: I2CEN Mask                    */

#define I2C_CTL_INTEN_Pos                (7)                                               /*!< I2C CTL: INTEN Position                */
#define I2C_CTL_INTEN_Msk                (0x1ul << I2C_CTL_INTEN_Pos)                      /*!< I2C CTL: INTEN Mask                    */

#define I2C_ADDR0_GC_Pos                 (0)                                               /*!< I2C ADDR0: GC Position                 */
#define I2C_ADDR0_GC_Msk                 (0x1ul << I2C_ADDR0_GC_Pos)                       /*!< I2C ADDR0: GC Mask                     */

#define I2C_ADDR0_ADDR_Pos               (1)                                               /*!< I2C ADDR0: ADDR Position               */
#define I2C_ADDR0_ADDR_Msk               (0x7ful << I2C_ADDR0_ADDR_Pos)                    /*!< I2C ADDR0: ADDR Mask                   */

#define I2C_DAT_DAT_Pos                  (0)                                               /*!< I2C DAT: DAT Position                  */
#define I2C_DAT_DAT_Msk                  (0xfful << I2C_DAT_DAT_Pos)                       /*!< I2C DAT: DAT Mask                      */

#define I2C_STATUS_STATUS_Pos            (0)                                               /*!< I2C STATUS: STATUS Position            */
#define I2C_STATUS_STATUS_Msk            (0xfful << I2C_STATUS_STATUS_Pos)                 /*!< I2C STATUS: STATUS Mask                */

#define I2C_CLKDIV_DIVIDER_Pos           (0)                                               /*!< I2C CLKDIV: DIVIDER Position           */
#define I2C_CLKDIV_DIVIDER_Msk           (0xfful << I2C_CLKDIV_DIVIDER_Pos)                /*!< I2C CLKDIV: DIVIDER Mask               */

#define I2C_TOCTL_TOIF_Pos               (0)                                               /*!< I2C TOCTL: TOIF Position               */
#define I2C_TOCTL_TOIF_Msk               (0x1ul << I2C_TOCTL_TOIF_Pos)                     /*!< I2C TOCTL: TOIF Mask                   */

#define I2C_TOCTL_TOCDIV4_Pos            (1)                                               /*!< I2C TOCTL: TOCDIV4 Position            */
#define I2C_TOCTL_TOCDIV4_Msk            (0x1ul << I2C_TOCTL_TOCDIV4_Pos)                  /*!< I2C TOCTL: TOCDIV4 Mask                */

#define I2C_TOCTL_TOCEN_Pos              (2)                                               /*!< I2C TOCTL: TOCEN Position              */
#define I2C_TOCTL_TOCEN_Msk              (0x1ul << I2C_TOCTL_TOCEN_Pos)                    /*!< I2C TOCTL: TOCEN Mask                  */

#define I2C_ADDR1_GC_Pos                 (0)                                               /*!< I2C ADDR1: GC Position                 */
#define I2C_ADDR1_GC_Msk                 (0x1ul << I2C_ADDR1_GC_Pos)                       /*!< I2C ADDR1: GC Mask                     */

#define I2C_ADDR1_ADDR_Pos               (1)                                               /*!< I2C ADDR1: ADDR Position               */
#define I2C_ADDR1_ADDR_Msk               (0x7ful << I2C_ADDR1_ADDR_Pos)                    /*!< I2C ADDR1: ADDR Mask                   */

#define I2C_ADDR2_GC_Pos                 (0)                                               /*!< I2C ADDR2: GC Position                 */
#define I2C_ADDR2_GC_Msk                 (0x1ul << I2C_ADDR2_GC_Pos)                       /*!< I2C ADDR2: GC Mask                     */

#define I2C_ADDR2_ADDR_Pos               (1)                                               /*!< I2C ADDR2: ADDR Position               */
#define I2C_ADDR2_ADDR_Msk               (0x7ful << I2C_ADDR2_ADDR_Pos)                    /*!< I2C ADDR2: ADDR Mask                   */

#define I2C_ADDR3_GC_Pos                 (0)                                               /*!< I2C ADDR3: GC Position                 */
#define I2C_ADDR3_GC_Msk                 (0x1ul << I2C_ADDR3_GC_Pos)                       /*!< I2C ADDR3: GC Mask                     */

#define I2C_ADDR3_ADDR_Pos               (1)                                               /*!< I2C ADDR3: ADDR Position               */
#define I2C_ADDR3_ADDR_Msk               (0x7ful << I2C_ADDR3_ADDR_Pos)                    /*!< I2C ADDR3: ADDR Mask                   */

#define I2C_ADDRMSK0_ADDRMSK1_Pos        (1)                                               /*!< I2C ADDRMSK0: ADDRMSK1 Position        */
#define I2C_ADDRMSK0_ADDRMSK1_Msk        (0x1ul << I2C_ADDRMSK0_ADDRMSK1_Pos)              /*!< I2C ADDRMSK0: ADDRMSK1 Mask            */

#define I2C_ADDRMSK0_ADDRMSK2_Pos        (2)                                               /*!< I2C ADDRMSK0: ADDRMSK2 Position        */
#define I2C_ADDRMSK0_ADDRMSK2_Msk        (0x1ul << I2C_ADDRMSK0_ADDRMSK2_Pos)              /*!< I2C ADDRMSK0: ADDRMSK2 Mask            */

#define I2C_ADDRMSK0_ADDRMSK3_Pos        (3)                                               /*!< I2C ADDRMSK0: ADDRMSK3 Position        */
#define I2C_ADDRMSK0_ADDRMSK3_Msk        (0x1ul << I2C_ADDRMSK0_ADDRMSK3_Pos)              /*!< I2C ADDRMSK0: ADDRMSK3 Mask            */

#define I2C_ADDRMSK0_ADDRMSK4_Pos        (4)                                               /*!< I2C ADDRMSK0: ADDRMSK4 Position        */
#define I2C_ADDRMSK0_ADDRMSK4_Msk        (0x1ul << I2C_ADDRMSK0_ADDRMSK4_Pos)              /*!< I2C ADDRMSK0: ADDRMSK4 Mask            */

#define I2C_ADDRMSK0_ADDRMSK5_Pos        (5)                                               /*!< I2C ADDRMSK0: ADDRMSK5 Position        */
#define I2C_ADDRMSK0_ADDRMSK5_Msk        (0x1ul << I2C_ADDRMSK0_ADDRMSK5_Pos)              /*!< I2C ADDRMSK0: ADDRMSK5 Mask            */

#define I2C_ADDRMSK0_ADDRMSK6_Pos        (6)                                               /*!< I2C ADDRMSK0: ADDRMSK6 Position        */
#define I2C_ADDRMSK0_ADDRMSK6_Msk        (0x1ul << I2C_ADDRMSK0_ADDRMSK6_Pos)              /*!< I2C ADDRMSK0: ADDRMSK6 Mask            */

#define I2C_ADDRMSK0_ADDRMSK7_Pos        (7)                                               /*!< I2C ADDRMSK0: ADDRMSK7 Position        */
#define I2C_ADDRMSK0_ADDRMSK7_Msk        (0x1ul << I2C_ADDRMSK0_ADDRMSK7_Pos)              /*!< I2C ADDRMSK0: ADDRMSK7 Mask            */

#define I2C_ADDRMSK1_ADDRMSK1_Pos        (1)                                               /*!< I2C ADDRMSK1: ADDRMSK1 Position        */
#define I2C_ADDRMSK1_ADDRMSK1_Msk        (0x1ul << I2C_ADDRMSK1_ADDRMSK1_Pos)              /*!< I2C ADDRMSK1: ADDRMSK1 Mask            */

#define I2C_ADDRMSK1_ADDRMSK2_Pos        (2)                                               /*!< I2C ADDRMSK1: ADDRMSK2 Position        */
#define I2C_ADDRMSK1_ADDRMSK2_Msk        (0x1ul << I2C_ADDRMSK1_ADDRMSK2_Pos)              /*!< I2C ADDRMSK1: ADDRMSK2 Mask            */

#define I2C_ADDRMSK1_ADDRMSK3_Pos        (3)                                               /*!< I2C ADDRMSK1: ADDRMSK3 Position        */
#define I2C_ADDRMSK1_ADDRMSK3_Msk        (0x1ul << I2C_ADDRMSK1_ADDRMSK3_Pos)              /*!< I2C ADDRMSK1: ADDRMSK3 Mask            */

#define I2C_ADDRMSK1_ADDRMSK4_Pos        (4)                                               /*!< I2C ADDRMSK1: ADDRMSK4 Position        */
#define I2C_ADDRMSK1_ADDRMSK4_Msk        (0x1ul << I2C_ADDRMSK1_ADDRMSK4_Pos)              /*!< I2C ADDRMSK1: ADDRMSK4 Mask            */

#define I2C_ADDRMSK1_ADDRMSK5_Pos        (5)                                               /*!< I2C ADDRMSK1: ADDRMSK5 Position        */
#define I2C_ADDRMSK1_ADDRMSK5_Msk        (0x1ul << I2C_ADDRMSK1_ADDRMSK5_Pos)              /*!< I2C ADDRMSK1: ADDRMSK5 Mask            */

#define I2C_ADDRMSK1_ADDRMSK6_Pos        (6)                                               /*!< I2C ADDRMSK1: ADDRMSK6 Position        */
#define I2C_ADDRMSK1_ADDRMSK6_Msk        (0x1ul << I2C_ADDRMSK1_ADDRMSK6_Pos)              /*!< I2C ADDRMSK1: ADDRMSK6 Mask            */

#define I2C_ADDRMSK1_ADDRMSK7_Pos        (7)                                               /*!< I2C ADDRMSK1: ADDRMSK7 Position        */
#define I2C_ADDRMSK1_ADDRMSK7_Msk        (0x1ul << I2C_ADDRMSK1_ADDRMSK7_Pos)              /*!< I2C ADDRMSK1: ADDRMSK7 Mask            */

#define I2C_ADDRMSK2_ADDRMSK1_Pos        (1)                                               /*!< I2C ADDRMSK2: ADDRMSK1 Position        */
#define I2C_ADDRMSK2_ADDRMSK1_Msk        (0x1ul << I2C_ADDRMSK2_ADDRMSK1_Pos)              /*!< I2C ADDRMSK2: ADDRMSK1 Mask            */

#define I2C_ADDRMSK2_ADDRMSK2_Pos        (2)                                               /*!< I2C ADDRMSK2: ADDRMSK2 Position        */
#define I2C_ADDRMSK2_ADDRMSK2_Msk        (0x1ul << I2C_ADDRMSK2_ADDRMSK2_Pos)              /*!< I2C ADDRMSK2: ADDRMSK2 Mask            */

#define I2C_ADDRMSK2_ADDRMSK3_Pos        (3)                                               /*!< I2C ADDRMSK2: ADDRMSK3 Position        */
#define I2C_ADDRMSK2_ADDRMSK3_Msk        (0x1ul << I2C_ADDRMSK2_ADDRMSK3_Pos)              /*!< I2C ADDRMSK2: ADDRMSK3 Mask            */

#define I2C_ADDRMSK2_ADDRMSK4_Pos        (4)                                               /*!< I2C ADDRMSK2: ADDRMSK4 Position        */
#define I2C_ADDRMSK2_ADDRMSK4_Msk        (0x1ul << I2C_ADDRMSK2_ADDRMSK4_Pos)              /*!< I2C ADDRMSK2: ADDRMSK4 Mask            */

#define I2C_ADDRMSK2_ADDRMSK5_Pos        (5)                                               /*!< I2C ADDRMSK2: ADDRMSK5 Position        */
#define I2C_ADDRMSK2_ADDRMSK5_Msk        (0x1ul << I2C_ADDRMSK2_ADDRMSK5_Pos)              /*!< I2C ADDRMSK2: ADDRMSK5 Mask            */

#define I2C_ADDRMSK2_ADDRMSK6_Pos        (6)                                               /*!< I2C ADDRMSK2: ADDRMSK6 Position        */
#define I2C_ADDRMSK2_ADDRMSK6_Msk        (0x1ul << I2C_ADDRMSK2_ADDRMSK6_Pos)              /*!< I2C ADDRMSK2: ADDRMSK6 Mask            */

#define I2C_ADDRMSK2_ADDRMSK7_Pos        (7)                                               /*!< I2C ADDRMSK2: ADDRMSK7 Position        */
#define I2C_ADDRMSK2_ADDRMSK7_Msk        (0x1ul << I2C_ADDRMSK2_ADDRMSK7_Pos)              /*!< I2C ADDRMSK2: ADDRMSK7 Mask            */

#define I2C_ADDRMSK3_ADDRMSK1_Pos        (1)                                               /*!< I2C ADDRMSK3: ADDRMSK1 Position        */
#define I2C_ADDRMSK3_ADDRMSK1_Msk        (0x1ul << I2C_ADDRMSK3_ADDRMSK1_Pos)              /*!< I2C ADDRMSK3: ADDRMSK1 Mask            */

#define I2C_ADDRMSK3_ADDRMSK2_Pos        (2)                                               /*!< I2C ADDRMSK3: ADDRMSK2 Position        */
#define I2C_ADDRMSK3_ADDRMSK2_Msk        (0x1ul << I2C_ADDRMSK3_ADDRMSK2_Pos)              /*!< I2C ADDRMSK3: ADDRMSK2 Mask            */

#define I2C_ADDRMSK3_ADDRMSK3_Pos        (3)                                               /*!< I2C ADDRMSK3: ADDRMSK3 Position        */
#define I2C_ADDRMSK3_ADDRMSK3_Msk        (0x1ul << I2C_ADDRMSK3_ADDRMSK3_Pos)              /*!< I2C ADDRMSK3: ADDRMSK3 Mask            */

#define I2C_ADDRMSK3_ADDRMSK4_Pos        (4)                                               /*!< I2C ADDRMSK3: ADDRMSK4 Position        */
#define I2C_ADDRMSK3_ADDRMSK4_Msk        (0x1ul << I2C_ADDRMSK3_ADDRMSK4_Pos)              /*!< I2C ADDRMSK3: ADDRMSK4 Mask            */

#define I2C_ADDRMSK3_ADDRMSK5_Pos        (5)                                               /*!< I2C ADDRMSK3: ADDRMSK5 Position        */
#define I2C_ADDRMSK3_ADDRMSK5_Msk        (0x1ul << I2C_ADDRMSK3_ADDRMSK5_Pos)              /*!< I2C ADDRMSK3: ADDRMSK5 Mask            */

#define I2C_ADDRMSK3_ADDRMSK6_Pos        (6)                                               /*!< I2C ADDRMSK3: ADDRMSK6 Position        */
#define I2C_ADDRMSK3_ADDRMSK6_Msk        (0x1ul << I2C_ADDRMSK3_ADDRMSK6_Pos)              /*!< I2C ADDRMSK3: ADDRMSK6 Mask            */

#define I2C_ADDRMSK3_ADDRMSK7_Pos        (7)                                               /*!< I2C ADDRMSK3: ADDRMSK7 Position        */
#define I2C_ADDRMSK3_ADDRMSK7_Msk        (0x1ul << I2C_ADDRMSK3_ADDRMSK7_Pos)              /*!< I2C ADDRMSK3: ADDRMSK7 Mask            */

/**@}*/ /* I2C_CONST */
/**@}*/ /* end of I2C register group */


/*---------------------- I2S Interface Controller -------------------------*/
/**
    @addtogroup I2S I2S Interface Controller(I2S)
    Memory Mapped Structure for I2S Controller
@{ */
 
typedef struct
{


    /**
     * CTL
     * ===================================================================================================
     * Offset: 0x00  I2S Control Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2SEN     |Enable I2S Controller
     * |        |          |0 = Disable
     * |        |          |1 = Enable
     * |[1]     |TXEN      |Transmit Enable
     * |        |          |0 = Disable data transmit
     * |        |          |1 = Enable data transmit 
     * |[2]     |RXEN      |Receive Enable
     * |        |          |0 = Disable data receive
     * |        |          |1 = Enable data receive 
     * |[3]     |MUTE      |Transmit Mute Enable
     * |        |          |0 = Transmit data is shifted from FIFO
     * |        |          |1= Transmit channel zero
     * |[4:5]   |WDWIDTH   |Word Width
     * |        |          |This parameter sets the word width of audio data.
     * |        |          |See Figure 5-63 for details of how data is formatted in transmit and receive FIFO.
     * |        |          |00 = data is 8 bit
     * |        |          |01 = data is 16 bit
     * |        |          |10 = data is 24 bit
     * |        |          |11 = data is 32 bit
     * |[6]     |MONO      |Monaural data
     * |        |          |This parameter sets whether mono or stereo data is processed.
     * |        |          |See Figure 5-63 for details of how data is formatted in transmit and receive FIFO.
     * |        |          |0 = Data is stereo format
     * |        |          |1 = Data is monaural format
     * |[7]     |FORMAT    |Data format
     * |        |          |0 = I2S data format
     * |        |          |1 = MSB justified data format
     * |        |          |See Figure 5-61 and Figure 5-62 for timing differences.
     * |[8]     |SLAVE     |Slave Mode
     * |        |          |I2S can operate as a master or slave.
     * |        |          |For master mode, I2S_BCLK and I2S_FS pins are outputs and send bit clock and frame sync from ISD9100.
     * |        |          |In slave mode, I2S_BCLK and I2S_FS pins are inputs and bit clock and frame sync are received from external audio device.
     * |        |          |0 = Master mode
     * |        |          |1 = Slave mode 
     * |[9:11]  |TXTH      |Transmit FIFO Threshold Level
     * |        |          |If remaining data words in transmit FIFO less than or equal to the threshold level then TXTHI flag is set.
     * |        |          |Threshold = TXTH words remaining in transmit FIFO
     * |[12:14] |RXTH      |Receive FIFO Threshold Level
     * |        |          |When received data word(s) in buffer is equal or higher than threshold level then RXTHI flag is set.
     * |        |          |Threshold = RXTH+1 words of data in receive FIFO.
     * |[15]    |MCLKEN    |Master Clock Enable
     * |        |          |The ISD9100 can generate a master clock signal to an external audio CODEC to synchronize the audio devices.
     * |        |          |If audio devices are not synchronous, then data will be periodically corrupted.
     * |        |          |Software needs to implement a way to drop/repeat or interpolate samples in a jitter buffer if devices are not synchronized.
     * |        |          |The master clock frequency is determined by the I2S_CLKDIV.MCLKDIV.
     * |        |          |register.
     * |        |          |0 = Disable master clock
     * |        |          |1 = Enable master clock
     * |[16]    |RZCEN     |Right Channel Zero Cross Detect Enable
     * |        |          |If this bit is set to 1, when right channel data sign bit changes, or data bits are all zero, the RZCIF flag in I2S_STATUS register will be set to 1.
     * |        |          |0 = Disable right channel zero cross detect
     * |        |          |1 = Enable right channel zero cross detect
     * |[17]    |LZCEN     |Left Channel Zero Cross Detect Enable
     * |        |          |If this bit is set to 1, when left channel data sign bit changes, or data bits are all zero, the LZCIF flag in I2S_STATUS register will be set to 1.
     * |        |          |0 = Disable left channel zero cross detect
     * |        |          |1 = Enable left channel zero cross detect
     * |[18]    |TXCLR     |Clear Transmit FIFO
     * |        |          |Write 1 to clear transmitting FIFO, internal pointer is reset to FIFO start point, and TXTH returns to zero and transmit FIFO becomes empty.
     * |        |          |Data in transmit FIFO is not changed.
     * |        |          |This bit is cleared by hardware automatically when clear operation complete.
     * |[19]    |RXCLR     |Clear Receive FIFO
     * |        |          |Write 1 to clear receiving FIFO, internal pointer is reset to FIFO start point, and RXTH returns to zero and receive FIFO becomes empty.
     * |        |          |This bit is cleared by hardware automatically when clear operation complete.
     * |[20]    |TXPDMAEN  |Enable Transmit DMA
     * |        |          |When TX DMA is enables, I2S request DMA to transfer data from SRAM to transmit FIFO if FIFO is not full.
     * |        |          |0 = Disable TX DMA
     * |        |          |1 = Enable TX DMA
     * |[21]    |RXPDMAEN  |Enable Receive DMA
     * |        |          |When RX DMA is enabled, I2S requests DMA to transfer data from receive FIFO to SRAM if FIFO is not empty.
     * |        |          |0 = Disable RX DMA
     * |        |          |1 = Enable RX DMA
 */
    __IO uint32_t CTL;                   

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x04  I2S Clock Divider Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:2]   |MCLKDIV   |Master Clock Divider
     * |        |          |ISD9160 can generate a master clock to synchronously drive an external audio device.
     * |        |          |If MCLKDIV is set to 0, MCLK is the same as I2S_CLK clock input, otherwise MCLK frequency is given by:.
     * |        |          |F(MCLK) = F(I2S_CLK) / (2xMCLKDIV)
     * |        |          |Or,
     * |        |          |MCLKDIV = F(I2S_CLK) / (2 x F(MCLK))
     * |        |          |If the desired MCLK frequency is 254Fs and Fs = 16kHz then MCLKDIV = 6
     * |[8:15]  |BCLKDIV   |Bit Clock Divider
     * |        |          |If I2S operates in master mode, bit clock is provided by ISD9100.
     * |        |          |Software can program these bits to generate bit clock frequency for the desired sample rate.
     * |        |          |For sample rate Fs, the desired bit clock frequency is:
     * |        |          |F(BCLK) = Fs x Word_width_in_bytes x 16
     * |        |          |For example if Fs = 16kHz, and word width is 2-bytes (16bit) then desired bit clock frequency is 512kHz.
     * |        |          |The bit clock frequency is given by:
     * |        |          |F(BCLK) = F(I2S_CLK) / 2x(BCLKDIV+1)
     * |        |          |Or,
     * |        |          |BCLKDIV = F(I2S_CLK) / (2 x F(BCLK)) -1
     * |        |          |So if F(I2S_CLK) = HCLK = 49.152MHz, desired F(BCLK) = 512kHzthen BCLKDIV = 47
 */
    __IO uint32_t CLKDIV;                

    /**
     * IEN
     * ===================================================================================================
     * Offset: 0x08  I2S Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |RXUDIEN   |Receive FIFO Underflow Interrupt Enable
     * |        |          |If software read receive FIFO when it is empty then RXUDIF flag in I2SSTATUS register is set to 1.
     * |        |          |0 = Disable interrupt
     * |        |          |1 = Enable interrupt
     * |[1]     |RXOVIEN   |Receive FIFO Overflow Interrupt Enable
     * |        |          |0 = Disable interrupt
     * |        |          |1 = Enable interrupt
     * |[2]     |RXTHIEN   |Receive FIFO Threshold Level Interrupt
     * |        |          |Interrupt occurs if this bit is set to 1 and data words in receive FIFO is greater than or equal to RXTH[2:0].
     * |        |          |0 = Disable interrupt
     * |        |          |1 = Enable interrupt
     * |[8]     |TXUDIEN   |Transmit FIFO Underflow Interrupt Enable
     * |        |          |Interrupt occur if this bit is set to 1 and transmit FIFO underflow flag is set to 1.
     * |        |          |0 = Disable interrupt
     * |        |          |1 = Enable interrupt
     * |[9]     |TXOVIEN   |Transmit FIFO Overflow Interrupt Enable
     * |        |          |Interrupt occurs if this bit is set to 1 and transmit FIFO overflow flag is set to 1
     * |        |          |0 = Disable interrupt
     * |        |          |1 = Enable interrupt
     * |[10]    |TXTHIEN   |Transmit FIFO Threshold Level Interrupt Enable
     * |        |          |Interrupt occurs if this bit is set to 1 and data words in transmit FIFO is less than TXTH[2:0].
     * |        |          |0 = Disable interrupt
     * |        |          |1 = Enable interrupt
     * |[11]    |RZCIEN    |Right Channel Zero Cross Interrupt Enable
     * |        |          |Interrupt will occur if this bit is set to 1 and right channel has zero cross event
     * |        |          |0 = Disable interrupt
     * |        |          |1 = Enable interrupt
     * |[12]    |LZCIEN    |Left Channel Zero Cross Interrupt Enable
     * |        |          |Interrupt will occur if this bit is set to 1 and left channel has zero cross event
     * |        |          |0 = Disable interrupt
     * |        |          |1 = Enable interrupt
 */
    __IO uint32_t IEN;                   

    /**
     * STATUS
     * ===================================================================================================
     * Offset: 0x0C  I2S Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |I2SIF     |I2S Interrupt (Read Only)
     * |        |          |This bit is set if any enabled I2S interrupt is active.
     * |        |          |0 = No I2S interrupt
     * |        |          |1 = I2S interrupt active
     * |[1]     |RXIF      |I2S Receive Interrupt (Read Only)
     * |        |          |This indicates that there is an active receive interrupt source.
     * |        |          |This could be RXOVIF, RXUDIF or RXTHIF if corresponding interrupt enable bits are active.
     * |        |          |To clear interrupt the corresponding source(s) must be cleared.
     * |        |          |0 = No receive interrupt
     * |        |          |1 = Receive interrupt occurred
     * |[2]     |TXIF      |I2S Transmit Interrupt (Read Only)
     * |        |          |This indicates that there is an active transmit interrupt source.
     * |        |          |This could be TXOVIF, TXUDIF, TXTHIF, LZCIF or RZCIF if corresponding interrupt enable bits are active.
     * |        |          |To clear interrupt the corresponding source(s) must be cleared.
     * |        |          |0 = No transmit interrupt
     * |        |          |1 = Transmit interrupt occurred.
     * |[3]     |RIGHT     |Right Channel Active (Read Only)
     * |        |          |This bit indicates current data being transmitted/received belongs to right channel
     * |        |          |0 = Left channel
     * |        |          |1 = Right channel
     * |[8]     |RXUDIF    |Receive FIFO Underflow Flag (Write '1' to clear)
     * |        |          |This flag is set if attempt is made to read receive FIFO while it is empty.
     * |        |          |0 = No underflow
     * |        |          |1 = Underflow
     * |[9]     |RXOVIF    |Receive FIFO Overflow Flag (Write '1' to clear)
     * |        |          |This flag is set if I2S controller writes to receive FIFO when it is full. Audio data is lost.
     * |        |          |0 = No overflow
     * |        |          |1 = Overflow
     * |[10]    |RXTHIF    |Receive FIFO Threshold Flag (Read Only)
     * |        |          |When data word(s) in receive FIFO is greater than or equal to threshold value set in RXTH[2:0] the RXTHIF bit becomes to 1.
     * |        |          |It remains set until receive FIFO level is less than RXTH[2:0].
     * |        |          |It is cleared by reading I2S_RX until threshold satisfied.
     * |        |          |0 = Data word(s) in FIFO is less than threshold level
     * |        |          |1 = Data word(s) in FIFO is greater than or equal to threshold level
     * |[11]    |RXFULL    |Receive FIFO full (Read Only)
     * |        |          |This bit is set when receive FIFO is full.
     * |        |          |0 = Not full.
     * |        |          |1 = Full.
     * |[12]    |RXEMPTY   |Receive FIFO empty (Read Only)
     * |        |          |This is set when receive FIFO is empty.
     * |        |          |0 = Not empty
     * |        |          |1 = Empty
     * |[16]    |TXUDIF    |Transmit FIFO underflow flag (Write '1' to clear)
     * |        |          |This flag is set if I2S controller requests data when transmit FIFO is empty.
     * |        |          |0 = No underflow
     * |        |          |1 = Underflow
     * |[17]    |TXOVIF    |Transmit FIFO Overflow Flag (Write '1' to clear)
     * |        |          |This flag is set if data is written to transmit FIFO when it is full.
     * |        |          |0 = No overflow
     * |        |          |1 = Overflow
     * |[18]    |TXTHIF    |Transmit FIFO Threshold Flag (Read Only)
     * |        |          |When data word(s) in transmit FIFO is less than or equal to the threshold value set in TXTH[2:0] the TXTHIF bit becomes to 1.
     * |        |          |It remains set until transmit FIFO level is greater than TXTH[2:0].
     * |        |          |Cleared by writing to I2S_TX register until threshold exceeded.
     * |        |          |0 = Data word(s) in FIFO is greater than threshold level
     * |        |          |1 = Data word(s) in FIFO is less than or equal to threshold level
     * |[19]    |TXFULL    |Transmit FIFO Full (Read Only)
     * |        |          |This bit is set when transmit FIFO is full.
     * |        |          |0 = Not full.
     * |        |          |1 = Full.
     * |[20]    |TXEMPTY   |Transmit FIFO Empty (Read Only)
     * |        |          |This is set when transmit FIFO is empty.
     * |        |          |0 = Not empty
     * |        |          |1 = Empty
     * |[21]    |TXBUSY    |Transmit Busy (Read Only)
     * |        |          |This bit is cleared when all data in transmit FIFO and Tx shift register is shifted out.
     * |        |          |It is set when first data is loaded to Tx shift register.
     * |        |          |0 = Transmit shift register is empty
     * |        |          |1 = Transmit shift register is busy
     * |[22]    |RZCIF     |Right channel zero cross flag (write '1' to clear, or clear RZCEN)
     * |        |          |0 = No zero cross
     * |        |          |1 = Right channel zero cross is detected
     * |[23]    |LZCIF     |Left channel zero cross flag (write '1' to clear, or clear LZCEN)
     * |        |          |0 = No zero cross detected.
     * |        |          |1 = Left channel zero cross is detected
     * |[24:27] |RXCNT     |Receive FIFO level (Read Only)
     * |        |          |RXCNT = number of words in receive FIFO.
     * |[28:31] |TXCNT     |Transmit FIFO level (Read Only)
     * |        |          |TXCNT = number of words in transmit FIFO.
 */
    __I  uint32_t STATUS;                

    /**
     * TX
     * ===================================================================================================
     * Offset: 0x10  I2S Transmit FIFO Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |TX        |Transmit FIFO Register (Write Only)
     * |        |          |A write to this register pushes data onto the transmit FIFO.
     * |        |          |The transmit FIFO is eight words deep.
     * |        |          |The number of words currently in the FIFO can be determined by reading I2S_STATUS.TXCNT.
 */
    __O  uint32_t TX;                    

    /**
     * RX
     * ===================================================================================================
     * Offset: 0x14  I2S Receive FIFO Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |RX        |Receive FIFO Register (Read Only)
     * |        |          |A read of this register will pop data from the receive FIFO.
     * |        |          |The receive FIFO is eight words deep.
     * |        |          |The number of words currently in the FIFO can be determined by reading I2S_STATUS.RXCNT.
 */
    __I  uint32_t RX;                    

} I2S_T;

/**
    @addtogroup I2S_CONST I2S Bit Field Definition
    Constant Definitions for I2S Controller
@{ */

#define I2S_CTL_I2SEN_Pos                (0)                                               /*!< I2S CTL: I2SEN Position                */
#define I2S_CTL_I2SEN_Msk                (0x1ul << I2S_CTL_I2SEN_Pos)                      /*!< I2S CTL: I2SEN Mask                    */

#define I2S_CTL_TXEN_Pos                 (1)                                               /*!< I2S CTL: TXEN Position                 */
#define I2S_CTL_TXEN_Msk                 (0x1ul << I2S_CTL_TXEN_Pos)                       /*!< I2S CTL: TXEN Mask                     */

#define I2S_CTL_RXEN_Pos                 (2)                                               /*!< I2S CTL: RXEN Position                 */
#define I2S_CTL_RXEN_Msk                 (0x1ul << I2S_CTL_RXEN_Pos)                       /*!< I2S CTL: RXEN Mask                     */

#define I2S_CTL_MUTE_Pos                 (3)                                               /*!< I2S CTL: MUTE Position                 */
#define I2S_CTL_MUTE_Msk                 (0x1ul << I2S_CTL_MUTE_Pos)                       /*!< I2S CTL: MUTE Mask                     */

#define I2S_CTL_WDWIDTH_Pos              (4)                                               /*!< I2S CTL: WDWIDTH Position              */
#define I2S_CTL_WDWIDTH_Msk              (0x3ul << I2S_CTL_WDWIDTH_Pos)                    /*!< I2S CTL: WDWIDTH Mask                  */

#define I2S_CTL_MONO_Pos                 (6)                                               /*!< I2S CTL: MONO Position                 */
#define I2S_CTL_MONO_Msk                 (0x1ul << I2S_CTL_MONO_Pos)                       /*!< I2S CTL: MONO Mask                     */

#define I2S_CTL_FORMAT_Pos               (7)                                               /*!< I2S CTL: FORMAT Position               */
#define I2S_CTL_FORMAT_Msk               (0x1ul << I2S_CTL_FORMAT_Pos)                     /*!< I2S CTL: FORMAT Mask                   */

#define I2S_CTL_SLAVE_Pos                (8)                                               /*!< I2S CTL: SLAVE Position                */
#define I2S_CTL_SLAVE_Msk                (0x1ul << I2S_CTL_SLAVE_Pos)                      /*!< I2S CTL: SLAVE Mask                    */

#define I2S_CTL_TXTH_Pos                 (9)                                               /*!< I2S CTL: TXTH Position                 */
#define I2S_CTL_TXTH_Msk                 (0x7ul << I2S_CTL_TXTH_Pos)                       /*!< I2S CTL: TXTH Mask                     */

#define I2S_CTL_RXTH_Pos                 (12)                                              /*!< I2S CTL: RXTH Position                 */
#define I2S_CTL_RXTH_Msk                 (0x7ul << I2S_CTL_RXTH_Pos)                       /*!< I2S CTL: RXTH Mask                     */

#define I2S_CTL_MCLKEN_Pos               (15)                                              /*!< I2S CTL: MCLKEN Position               */
#define I2S_CTL_MCLKEN_Msk               (0x1ul << I2S_CTL_MCLKEN_Pos)                     /*!< I2S CTL: MCLKEN Mask                   */

#define I2S_CTL_RZCEN_Pos                (16)                                              /*!< I2S CTL: RZCEN Position                */
#define I2S_CTL_RZCEN_Msk                (0x1ul << I2S_CTL_RZCEN_Pos)                      /*!< I2S CTL: RZCEN Mask                    */

#define I2S_CTL_LZCEN_Pos                (17)                                              /*!< I2S CTL: LZCEN Position                */
#define I2S_CTL_LZCEN_Msk                (0x1ul << I2S_CTL_LZCEN_Pos)                      /*!< I2S CTL: LZCEN Mask                    */

#define I2S_CTL_TXCLR_Pos                (18)                                              /*!< I2S CTL: TXCLR Position                */
#define I2S_CTL_TXCLR_Msk                (0x1ul << I2S_CTL_TXCLR_Pos)                      /*!< I2S CTL: TXCLR Mask                    */

#define I2S_CTL_RXCLR_Pos                (19)                                              /*!< I2S CTL: RXCLR Position                */
#define I2S_CTL_RXCLR_Msk                (0x1ul << I2S_CTL_RXCLR_Pos)                      /*!< I2S CTL: RXCLR Mask                    */

#define I2S_CTL_TXPDMAEN_Pos             (20)                                              /*!< I2S CTL: TXPDMAEN Position             */
#define I2S_CTL_TXPDMAEN_Msk             (0x1ul << I2S_CTL_TXPDMAEN_Pos)                   /*!< I2S CTL: TXPDMAEN Mask                 */

#define I2S_CTL_RXPDMAEN_Pos             (21)                                              /*!< I2S CTL: RXPDMAEN Position             */
#define I2S_CTL_RXPDMAEN_Msk             (0x1ul << I2S_CTL_RXPDMAEN_Pos)                   /*!< I2S CTL: RXPDMAEN Mask                 */

#define I2S_CLKDIV_MCLKDIV_Pos           (0)                                               /*!< I2S CLKDIV: MCLKDIV Position           */
#define I2S_CLKDIV_MCLKDIV_Msk           (0x7ul << I2S_CLKDIV_MCLKDIV_Pos)                 /*!< I2S CLKDIV: MCLKDIV Mask               */

#define I2S_CLKDIV_BCLKDIV_Pos           (8)                                               /*!< I2S CLKDIV: BCLKDIV Position           */
#define I2S_CLKDIV_BCLKDIV_Msk           (0xfful << I2S_CLKDIV_BCLKDIV_Pos)                /*!< I2S CLKDIV: BCLKDIV Mask               */

#define I2S_IEN_RXUDIEN_Pos              (0)                                               /*!< I2S IEN: RXUDIEN Position              */
#define I2S_IEN_RXUDIEN_Msk              (0x1ul << I2S_IEN_RXUDIEN_Pos)                    /*!< I2S IEN: RXUDIEN Mask                  */

#define I2S_IEN_RXOVIEN_Pos              (1)                                               /*!< I2S IEN: RXOVIEN Position              */
#define I2S_IEN_RXOVIEN_Msk              (0x1ul << I2S_IEN_RXOVIEN_Pos)                    /*!< I2S IEN: RXOVIEN Mask                  */

#define I2S_IEN_RXTHIEN_Pos              (2)                                               /*!< I2S IEN: RXTHIEN Position              */
#define I2S_IEN_RXTHIEN_Msk              (0x1ul << I2S_IEN_RXTHIEN_Pos)                    /*!< I2S IEN: RXTHIEN Mask                  */

#define I2S_IEN_TXUDIEN_Pos              (8)                                               /*!< I2S IEN: TXUDIEN Position              */
#define I2S_IEN_TXUDIEN_Msk              (0x1ul << I2S_IEN_TXUDIEN_Pos)                    /*!< I2S IEN: TXUDIEN Mask                  */

#define I2S_IEN_TXOVIEN_Pos              (9)                                               /*!< I2S IEN: TXOVIEN Position              */
#define I2S_IEN_TXOVIEN_Msk              (0x1ul << I2S_IEN_TXOVIEN_Pos)                    /*!< I2S IEN: TXOVIEN Mask                  */

#define I2S_IEN_TXTHIEN_Pos              (10)                                              /*!< I2S IEN: TXTHIEN Position              */
#define I2S_IEN_TXTHIEN_Msk              (0x1ul << I2S_IEN_TXTHIEN_Pos)                    /*!< I2S IEN: TXTHIEN Mask                  */

#define I2S_IEN_RZCIEN_Pos               (11)                                              /*!< I2S IEN: RZCIEN Position               */
#define I2S_IEN_RZCIEN_Msk               (0x1ul << I2S_IEN_RZCIEN_Pos)                     /*!< I2S IEN: RZCIEN Mask                   */

#define I2S_IEN_LZCIEN_Pos               (12)                                              /*!< I2S IEN: LZCIEN Position               */
#define I2S_IEN_LZCIEN_Msk               (0x1ul << I2S_IEN_LZCIEN_Pos)                     /*!< I2S IEN: LZCIEN Mask                   */

#define I2S_STATUS_I2SIF_Pos             (0)                                               /*!< I2S STATUS: I2SIF Position             */
#define I2S_STATUS_I2SIF_Msk             (0x1ul << I2S_STATUS_I2SIF_Pos)                   /*!< I2S STATUS: I2SIF Mask                 */

#define I2S_STATUS_RXIF_Pos              (1)                                               /*!< I2S STATUS: RXIF Position              */
#define I2S_STATUS_RXIF_Msk              (0x1ul << I2S_STATUS_RXIF_Pos)                    /*!< I2S STATUS: RXIF Mask                  */

#define I2S_STATUS_TXIF_Pos              (2)                                               /*!< I2S STATUS: TXIF Position              */
#define I2S_STATUS_TXIF_Msk              (0x1ul << I2S_STATUS_TXIF_Pos)                    /*!< I2S STATUS: TXIF Mask                  */

#define I2S_STATUS_RIGHT_Pos             (3)                                               /*!< I2S STATUS: RIGHT Position             */
#define I2S_STATUS_RIGHT_Msk             (0x1ul << I2S_STATUS_RIGHT_Pos)                   /*!< I2S STATUS: RIGHT Mask                 */

#define I2S_STATUS_RXUDIF_Pos            (8)                                               /*!< I2S STATUS: RXUDIF Position            */
#define I2S_STATUS_RXUDIF_Msk            (0x1ul << I2S_STATUS_RXUDIF_Pos)                  /*!< I2S STATUS: RXUDIF Mask                */

#define I2S_STATUS_RXOVIF_Pos            (9)                                               /*!< I2S STATUS: RXOVIF Position            */
#define I2S_STATUS_RXOVIF_Msk            (0x1ul << I2S_STATUS_RXOVIF_Pos)                  /*!< I2S STATUS: RXOVIF Mask                */

#define I2S_STATUS_RXTHIF_Pos            (10)                                              /*!< I2S STATUS: RXTHIF Position            */
#define I2S_STATUS_RXTHIF_Msk            (0x1ul << I2S_STATUS_RXTHIF_Pos)                  /*!< I2S STATUS: RXTHIF Mask                */

#define I2S_STATUS_RXFULL_Pos            (11)                                              /*!< I2S STATUS: RXFULL Position            */
#define I2S_STATUS_RXFULL_Msk            (0x1ul << I2S_STATUS_RXFULL_Pos)                  /*!< I2S STATUS: RXFULL Mask                */

#define I2S_STATUS_RXEMPTY_Pos           (12)                                              /*!< I2S STATUS: RXEMPTY Position           */
#define I2S_STATUS_RXEMPTY_Msk           (0x1ul << I2S_STATUS_RXEMPTY_Pos)                 /*!< I2S STATUS: RXEMPTY Mask               */

#define I2S_STATUS_TXUDIF_Pos            (16)                                              /*!< I2S STATUS: TXUDIF Position            */
#define I2S_STATUS_TXUDIF_Msk            (0x1ul << I2S_STATUS_TXUDIF_Pos)                  /*!< I2S STATUS: TXUDIF Mask                */

#define I2S_STATUS_TXOVIF_Pos            (17)                                              /*!< I2S STATUS: TXOVIF Position            */
#define I2S_STATUS_TXOVIF_Msk            (0x1ul << I2S_STATUS_TXOVIF_Pos)                  /*!< I2S STATUS: TXOVIF Mask                */

#define I2S_STATUS_TXTHIF_Pos            (18)                                              /*!< I2S STATUS: TXTHIF Position            */
#define I2S_STATUS_TXTHIF_Msk            (0x1ul << I2S_STATUS_TXTHIF_Pos)                  /*!< I2S STATUS: TXTHIF Mask                */

#define I2S_STATUS_TXFULL_Pos            (19)                                              /*!< I2S STATUS: TXFULL Position            */
#define I2S_STATUS_TXFULL_Msk            (0x1ul << I2S_STATUS_TXFULL_Pos)                  /*!< I2S STATUS: TXFULL Mask                */

#define I2S_STATUS_TXEMPTY_Pos           (20)                                              /*!< I2S STATUS: TXEMPTY Position           */
#define I2S_STATUS_TXEMPTY_Msk           (0x1ul << I2S_STATUS_TXEMPTY_Pos)                 /*!< I2S STATUS: TXEMPTY Mask               */

#define I2S_STATUS_TXBUSY_Pos            (21)                                              /*!< I2S STATUS: TXBUSY Position            */
#define I2S_STATUS_TXBUSY_Msk            (0x1ul << I2S_STATUS_TXBUSY_Pos)                  /*!< I2S STATUS: TXBUSY Mask                */

#define I2S_STATUS_RZCIF_Pos             (22)                                              /*!< I2S STATUS: RZCIF Position             */
#define I2S_STATUS_RZCIF_Msk             (0x1ul << I2S_STATUS_RZCIF_Pos)                   /*!< I2S STATUS: RZCIF Mask                 */

#define I2S_STATUS_LZCIF_Pos             (23)                                              /*!< I2S STATUS: LZCIF Position             */
#define I2S_STATUS_LZCIF_Msk             (0x1ul << I2S_STATUS_LZCIF_Pos)                   /*!< I2S STATUS: LZCIF Mask                 */

#define I2S_STATUS_RXCNT_Pos             (24)                                              /*!< I2S STATUS: RXCNT Position             */
#define I2S_STATUS_RXCNT_Msk             (0xful << I2S_STATUS_RXCNT_Pos)                   /*!< I2S STATUS: RXCNT Mask                 */

#define I2S_STATUS_TXCNT_Pos             (28)                                              /*!< I2S STATUS: TXCNT Position             */
#define I2S_STATUS_TXCNT_Msk             (0xful << I2S_STATUS_TXCNT_Pos)                   /*!< I2S STATUS: TXCNT Mask                 */

#define I2S_TX_TX_Pos                    (0)                                               /*!< I2S TX: TX Position                    */
#define I2S_TX_TX_Msk                    (0xfffffffful << I2S_TX_TX_Pos)                   /*!< I2S TX: TX Mask                        */

#define I2S_RX_RX_Pos                    (0)                                               /*!< I2S RX: RX Position                    */
#define I2S_RX_RX_Msk                    (0xfffffffful << I2S_RX_RX_Pos)                   /*!< I2S RX: RX Mask                        */

/**@}*/ /* I2S_CONST */
/**@}*/ /* end of I2S register group */


/*---------------------- Peripheral Direct Memory Access Controller -------------------------*/
/**
    @addtogroup PDMA Peripheral Direct Memory Access Controller(PDMA)
    Memory Mapped Structure for PDMA Controller
@{ */
 
typedef struct
{
    /**
     * DSCT_CTL
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
     * |        |          |01 = Reserved.
     * |        |          |10 = Transfer Source address is fixed
     * |        |          |11 = Transfer Source address is wrapped.
     * |        |          |When PDMA_CURBCCHn (Current Byte Count) equals zero, the PDMA_CURSACHn (Current Source Address) and PDMA_CURBCCHn registers will be reloaded from the PDMA_DSCTn_ENDSA (Source Address) and PDMA_TXBCCHn (Byte Count) registers automatically and PDMA will start another transfer.
     * |        |          |Cycle continues until software sets PDMACKEN = 0.
     * |        |          |When PDMACKEN is disabled, the PDMA will complete the active transfer but the remaining data in the SBUF will not be transferred to the destination address.
     * |[6:7]   |DASEL     |Destination Address Select
     * |        |          |This parameter determines the behavior of the current destination address register with each PDMA transfer.
     * |        |          |It can either be fixed, incremented or wrapped.
     * |        |          |00 = Transfer Destination Address is incremented.
     * |        |          |01 = Reserved.
     * |        |          |10 = Transfer Destination Address is fixed (Used when data transferred from multiple addresses to a single destination such as peripheral FIFO input).
     * |        |          |11 = Transfer Destination Address is wrapped.
     * |        |          |When PDMA_CURBCCHn (Current Byte Count) equals zero, the PDMA_CURDACHn (Current Destination Address) and PDMA_CURBCCHn registers will be reloaded from the PDMA_DSCTn_ENDDA (Destination Address) and PDMA_TXBCCHn (Byte Count) registers automatically and PDMA will start another transfer.
     * |        |          |Cycle continues until software sets PDMACKEN=0.
     * |        |          |When PDMACKEN is disabled, the PDMA will complete the active transfer but the remaining data in the SBUF will not be transferred to the destination address.
     * |[12:15] |WAINTSEL  |Wrap Interrupt Select
     * |        |          |x1xx: If this bit is set, and wraparound mode is in operation a Wrap Interrupt can be generated when half each PDMA transfer is complete.
     * |        |          |For example if BYTECNT = 32 then an interrupt could be generated when 16 bytes were sent.
     * |        |          |xxx1: If this bit is set, and wraparound mode is in operation a Wrap Interrupt can be generated when each PDMA transfer is wrapped.
     * |        |          |For example if BYTECNT = 32 then an interrupt could be generated when 32 bytes were sent and PDMA wraps around.
     * |        |          |x1x1: Both half and w interrupts generated.
     * |[19:20] |TWIDTH    |Peripheral Transfer Width Select
     * |        |          |This parameter determines the data width to be transferred each PDMA transfer operation.
     * |        |          |00 = One word (32 bits) is transferred for every PDMA operation.
     * |        |          |01 = One byte (8 bits) is transferred for every PDMA operation.
     * |        |          |10 = One half-word (16 bits) is transferred for every PDMA operation.
     * |        |          |11 = Reserved.
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
     * DSCT_ENDSA
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
     * DSCT_ENDDA
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
     * TXBCCH
     * ===================================================================================================
     * Offset: 0x0C  PDMA Transfer Byte Count Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |BYTECNT   |PDMA Transfer Byte Count Register
     * |        |          |This register controls the transfer byte count of PDMA. Maximum value is 0xFFFF.
     * |        |          |Note: When in memory-to-memory (PDMA_DSCTn_CTL.MODESEL = 00b) mode, the transfer byte count must be word aligned, that is multiples of 4bytes.
 */
    __IO uint32_t TXBCCH;               

    /**
     * INLBPCH
     * ===================================================================================================
     * Offset: 0x10  PDMA Internal Buffer Pointer Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |BUFPTR    |PDMA Internal Buffer Pointer Register (Read Only)
     * |        |          |A PDMA transaction consists of two stages, a read from the source address and a write to the destination address.
     * |        |          |Internally this data is buffered in a 32bit register.
     * |        |          |If transaction width between the read and write transactions are different, this register tracks which byte/half-word of the internal buffer is being processed by the current transaction.
 */
    __I  uint32_t INLBPCH0;              

    /**
     * CURSACH
     * ===================================================================================================
     * Offset: 0x14  PDMA Current Source Address Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:31]  |CURSA     |PDMA Current Source Address Register (Read Only)
     * |        |          |This register returns the source address from which the PDMA transfer is occurring.
     * |        |          |This register is loaded from PDMA_DSCTn_ENDSA when PDMA is triggered or when a wraparound occurs.
 */
    __I  uint32_t CURSACH;              

    /**
     * CURDACH
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
     * CURBCCH
     * ===================================================================================================
     * Offset: 0x1C  PDMA Current Byte Count Register of Channel 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CURBC     |PDMA Current Byte Count Register (Read Only)
     * |        |          |This field indicates the current remaining byte count of PDMA transfer.
     * |        |          |This register is initialized with PDMA_TXBCCHn register when PDMA is triggered or when a wraparound occurs.
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
     * |        |          |If enabled, and channel source or destination address is in wraparound mode, the PDMA controller will generate a WRAP interrupt to the CPU according to the setting of PDMA_DSCTn_CTL.WAINTSEL.
     * |        |          |This can be interrupts when the transaction has finished and has wrapped around and/or when the transaction is half way in progress.
     * |        |          |This allows the efficient implementation of circular buffers for DMA.
     * |        |          |0 = Disable Wraparound PDMA interrupt generation.
     * |        |          |1 = Enable Wraparound interrupt generation.
 */
    __IO uint32_t INTENCH;              

    /**
     * CHIF
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
     * |        |          |0001 = Current transfer finished flag (CURBC aaaaaa 0).
     * |        |          |0100 = Current transfer half complete flag (CURBC aaaaaa BYTECNT/2).
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
     * Offset: 0x000  PDMA Global Control Register
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
     * Offset: 0x004  PDMA Service Selection Control Register
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
         uint32_t RESERVE4[1];


    /**
     * GLOBALIF
     * ===================================================================================================
     * Offset: 0x00C  PDMA Global Interrupt Status Register
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

#define PDMA_DSCT_CTL_TXWIDTH_Pos        (19)                                              /*!< PDMA DSCT0_CTL: TWIDTH Position        */
#define PDMA_DSCT_CTL_TXWIDTH_Msk        (0x3ul << PDMA_DSCT_CTL_TXWIDTH_Pos)              /*!< PDMA DSCT0_CTL: TWIDTH Mask            */

#define PDMA_DSCT_CTL_TXEN_Pos          (23)                                              /*!< PDMA DSCT0_CTL: TXEN Position          */
#define PDMA_DSCT_CTL_TXEN_Msk          (0x1ul << PDMA_DSCT_CTL_TXEN_Pos)                /*!< PDMA DSCT0_CTL: TXEN Mask              */

#define PDMA_DSCT_ENDSA_ENDSA_Pos       (0)                                               /*!< PDMA DSCT0_ENDSA: ENDSA Position       */
#define PDMA_DSCT_ENDSA_ENDSA_Msk       (0xfffffffful << PDMA_DSCT_ENDSA_ENDSA_Pos)      /*!< PDMA DSCT0_ENDSA: ENDSA Mask           */

#define PDMA_DSCT_ENDDA_ENDDA_Pos       (0)                                               /*!< PDMA DSCT0_ENDDA: ENDDA Position       */
#define PDMA_DSCT_ENDDA_ENDDA_Msk       (0xfffffffful << PDMA_DSCT_ENDDA_ENDDA_Pos)      /*!< PDMA DSCT0_ENDDA: ENDDA Mask           */

#define PDMA_TXBCCH_BYTECNT_Pos         (0)                                               /*!< PDMA TXBCCH0: BYTECNT Position         */
#define PDMA_TXBCCH_BYTECNT_Msk         (0xfffful << PDMA_TXBCCH_BYTECNT_Pos)            /*!< PDMA TXBCCH0: BYTECNT Mask             */

#define PDMA_INLBPCH_BUFPTR_Pos         (0)                                               /*!< PDMA INLBPCH0: BUFPTR Position         */
#define PDMA_INLBPCH_BUFPTR_Msk         (0xful << PDMA_INLBPCH_BUFPTR_Pos)               /*!< PDMA INLBPCH0: BUFPTR Mask             */

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

#define PDMA_GLOCTL_SWRST_Pos            (0)                                               /*!< PDMA GLOCTL: SWRST Position            */
#define PDMA_GLOCTL_SWRST_Msk            (0x1ul << PDMA_GLOCTL_SWRST_Pos)                  /*!< PDMA GLOCTL: SWRST Mask                */

#define PDMA_GLOCTL_CHCKEN_Pos           (8)                                               /*!< PDMA GLOCTL: CHCKEN Position           */
#define PDMA_GLOCTL_CHCKEN_Msk           (0xful << PDMA_GLOCTL_CHCKEN_Pos)                 /*!< PDMA GLOCTL: CHCKEN Mask               */

#define PDMA_SVCSEL_SPIRXSEL_Pos         (0)                                               /*!< PDMA SVCSEL: SPIRXSEL Position         */
#define PDMA_SVCSEL_SPIRXSEL_Msk         (0xful << PDMA_SVCSEL_SPIRXSEL_Pos)               /*!< PDMA SVCSEL: SPIRXSEL Mask             */

#define PDMA_SVCSEL_SPITXSEL_Pos         (4)                                               /*!< PDMA SVCSEL: SPITXSEL Position         */
#define PDMA_SVCSEL_SPITXSEL_Msk         (0xful << PDMA_SVCSEL_SPITXSEL_Pos)               /*!< PDMA SVCSEL: SPITXSEL Mask             */

#define PDMA_SVCSEL_ADCRXSEL_Pos         (8)                                               /*!< PDMA SVCSEL: ADCRXSEL Position         */
#define PDMA_SVCSEL_ADCRXSEL_Msk         (0xful << PDMA_SVCSEL_ADCRXSEL_Pos)               /*!< PDMA SVCSEL: ADCRXSEL Mask             */

#define PDMA_SVCSEL_DPWMTXSEL_Pos        (12)                                              /*!< PDMA SVCSEL: DPWMTXSEL Position        */
#define PDMA_SVCSEL_DPWMTXSEL_Msk        (0xful << PDMA_SVCSEL_DPWMTXSEL_Pos)              /*!< PDMA SVCSEL: DPWMTXSEL Mask            */

#define PDMA_SVCSEL_UARTRXSEL_Pos        (16)                                              /*!< PDMA SVCSEL: UARTRXSEL Position        */
#define PDMA_SVCSEL_UARTRXSEL_Msk        (0xful << PDMA_SVCSEL_UARTRXSEL_Pos)              /*!< PDMA SVCSEL: UARTRXSEL Mask            */

#define PDMA_SVCSEL_UARTXSEL_Pos         (20)                                              /*!< PDMA SVCSEL: UARTXSEL Position         */
#define PDMA_SVCSEL_UARTXSEL_Msk         (0xful << PDMA_SVCSEL_UARTXSEL_Pos)               /*!< PDMA SVCSEL: UARTXSEL Mask             */

#define PDMA_SVCSEL_I2SRXSEL_Pos         (24)                                              /*!< PDMA SVCSEL: I2SRXSEL Position         */
#define PDMA_SVCSEL_I2SRXSEL_Msk         (0xful << PDMA_SVCSEL_I2SRXSEL_Pos)               /*!< PDMA SVCSEL: I2SRXSEL Mask             */

#define PDMA_SVCSEL_I2STXSEL_Pos         (28)                                              /*!< PDMA SVCSEL: I2STXSEL Position         */
#define PDMA_SVCSEL_I2STXSEL_Msk         (0xful << PDMA_SVCSEL_I2STXSEL_Pos)               /*!< PDMA SVCSEL: I2STXSEL Mask             */

#define PDMA_GLOBALIF_GLOBALIF_Pos       (0)                                               /*!< PDMA GLOBALIF: GLOBALIF Position       */
#define PDMA_GLOBALIF_GLOBALIF_Msk       (0xful << PDMA_GLOBALIF_GLOBALIF_Pos)             /*!< PDMA GLOBALIF: GLOBALIF Mask           */

/**@}*/ /* PDMA_CONST */
/**@}*/ /* end of PDMA register group */


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
     * |[0:7]   |CLKPSC01  |Clock Pre-scaler For Pair Of PWM CH0 and CH1
     * |        |          |Clock input is divided by (CLKPSC01 + 1).
     * |        |          |If CLKPSC01 = 0, then the pre-scaler output clock will be stopped.
     * |        |          |This implies PWM counter 0 and 1 will also be stopped.
     * |[16:23] |DTCNT01   |Dead Zone Interval Register For Pair Of PWM CH0 and CH1
     * |        |          |These 8 bits determine dead zone length.
     * |        |          |The unit time of dead zone length is that from clock selector 0.
 */
    __IO uint32_t CLKPSC;                

    /**
     * CLKDIV
     * ===================================================================================================
     * Offset: 0x04  PWM Clock Select Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:2]   |CLKDIV0   |Timer 0 Clock Source Selection
     * |        |          |(Table is as CLKDIV1)
     * |[4:6]   |CLKDIV1   |Timer 1 Clock Source Selection
     * |        |          |Value : Input clock divided by
     * |        |          |0 : 2
     * |        |          |1 : 4
     * |        |          |2 : 8
     * |        |          |3 : 16
     * |        |          |4 : 1
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
     * |        |          |0 = Stop PWM-Timer 0 Running
     * |        |          |1 = Enable PWM-Timer 0 Start/Run
     * |[2]     |CH0INV    |PWM-Timer 0 Output Inverter ON/OFF
     * |        |          |0 = Inverter OFF
     * |        |          |1 = Inverter ON
     * |[3]     |CH0MOD    |PWM-Timer 0 Auto-reload/One-Shot Mode
     * |        |          |0 = One-Shot Mode
     * |        |          |1 = Auto-reload Mode
     * |        |          |Note: A rising transition of this bit will cause PWM_PERIOD0 and PWM_CMPDAT0 to be cleared.
     * |[4]     |DTEN01    |Dead-Zone 0 Generator Enable/Disable
     * |        |          |0 = Disable
     * |        |          |1 = Enable
     * |        |          |Note: When Dead-Zone Generator is enabled, the pair of PWM CH0 and CH1 become a complementary pair.
     * |[8]     |CNTEN1    |PWM-Timer 1 Enable/Disable Start Run
     * |        |          |0 = Stop PWM-Timer 1
     * |        |          |1 = Enable PWM-Timer 1 Start/Run
     * |[10]    |PINV1     |PWM-Timer 1 Output Inverter ON/OFF
     * |        |          |0 = Inverter OFF
     * |        |          |1 = Inverter ON
     * |[11]    |CNTMODE1  |PWM-Timer 1 Auto-reload/One-Shot Mode
     * |        |          |0 = One-Shot Mode
     * |        |          |1 = Auto-load Mode
     * |        |          |Note: A rising transition of this bit will cause PWM_PERIOD1 and PWM_CMPDAT1 to be cleared.
 */
    __IO uint32_t CTL;                   

    /**
     * PERIOD0
     * ===================================================================================================
     * Offset: 0x0C  PWM Counter Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |PERIOD    |PWM Counter/Timer Reload Value
     * |        |          |PERIOD determines the PWM period.
     * |        |          |PWM frequency = PWM01_CLK/(prescale+1)*(clock divider)/(PERIOD+1)
     * |        |          |Duty ratio = (CMP+1)/(PERIOD+1).
     * |        |          |CMP > = PERIOD: PWM output is always high.
     * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit
     * |        |          |(Unit = one PWM clock cycle)
     * |        |          |Note:
     * |        |          |Any write to PERIOD will take effect in next PWM cycle.
 */
    __IO uint32_t PERIOD0;               

    /**
     * CMPDAT0
     * ===================================================================================================
     * Offset: 0x10  PWM Comparator Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CMP       |PWM Comparator Register
     * |        |          |CMP determines the PWM duty cycle.
     * |        |          |PWM frequency = PWM01_CLK/(prescale+1)*(clock divider)/(PERIOD+1)
     * |        |          |Duty Cycle = (CMP+1)/(PERIOD+1).
     * |        |          |CMP > = PERIOD: PWM output is always high.
     * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit
     * |        |          |(Unit = one PWM clock cycle)
     * |        |          |Note: Any write to CMP will take effect in next PWM cycle.
 */
    __IO uint32_t CMPDAT0;               

    /**
     * CNT0
     * ===================================================================================================
     * Offset: 0x14  PWM Data Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CNT       |PWM Data Register
     * |        |          |Reports the current value of the 16-bit down counter.
 */
    __I  uint32_t CNT0;                  

    /**
     * PERIOD1
     * ===================================================================================================
     * Offset: 0x18  PWM Counter Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |PERIOD    |PWM Counter/Timer Reload Value
     * |        |          |PERIOD determines the PWM period.
     * |        |          |PWM frequency = PWM01_CLK/(prescale+1)*(clock divider)/(PERIOD+1)
     * |        |          |Duty ratio = (CMP+1)/(PERIOD+1).
     * |        |          |CMP > = PERIOD: PWM output is always high.
     * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit
     * |        |          |(Unit = one PWM clock cycle)
     * |        |          |Note:
     * |        |          |Any write to PERIOD will take effect in next PWM cycle.
 */
    __IO uint32_t PERIOD1;               

    /**
     * CMPDAT1
     * ===================================================================================================
     * Offset: 0x1C  PWM Comparator Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CMP       |PWM Comparator Register
     * |        |          |CMP determines the PWM duty cycle.
     * |        |          |PWM frequency = PWM01_CLK/(prescale+1)*(clock divider)/(PERIOD+1)
     * |        |          |Duty Cycle = (CMP+1)/(PERIOD+1).
     * |        |          |CMP > = PERIOD: PWM output is always high.
     * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
     * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit
     * |        |          |(Unit = one PWM clock cycle)
     * |        |          |Note: Any write to CMP will take effect in next PWM cycle.
 */
    __IO uint32_t CMPDAT1;               

    /**
     * CNT1
     * ===================================================================================================
     * Offset: 0x20  PWM Data Register 1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |CNT       |PWM Data Register
     * |        |          |Reports the current value of the 16-bit down counter.
 */
    __I  uint32_t CNT1;                  
         uint32_t RESERVE0[7];


    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x40  PWM Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIEN0     |PWM Timer 0 Interrupt Enable
     * |        |          |0 = Disable
     * |        |          |1 = Enable 
     * |[1]     |PIEN1     |PWM Timer 1 Interrupt Enable
     * |        |          |0 = Disable
     * |        |          |1 = Enable
 */
    __IO uint32_t INTEN;                 

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x44  PWM Interrupt Flag Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |PIF0      |PWM Timer 0 Interrupt Flag
     * |        |          |Flag is set by hardware when PWM CH0 down counter reaches zero, software can clear this bit by writing '1' to it.
     * |[1]     |PIF1      |PWM Timer 1 Interrupt Flag
     * |        |          |Flag is set by hardware when PWM CH1 down counter reaches zero, software can clear this bit by writing '1' to it.
 */
    __IO uint32_t INTSTS;                
         uint32_t RESERVE1[2];


    /**
     * CAPCTL01
     * ===================================================================================================
     * Offset: 0x50  Capture Control Register 0
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CAPINV0   |Channel 0 Inverter ON/OFF
     * |        |          |0 = Inverter OFF
     * |        |          |1 = Inverter ON. Reverse the input signal from GPIO before Capture timer
     * |[1]     |CRLIEN0   |Channel 0 Rising Latch Interrupt Enable ON/OFF
     * |        |          |0 = Disable rising latch interrupt
     * |        |          |1 = Enable rising latch interrupt.
     * |        |          |When enabled, capture block generates an interrupt on rising edge of input.
     * |[2]     |CFLIEN0   |Channel 0 Falling Latch Interrupt Enable ON/OFF
     * |        |          |0 = Disable falling latch interrupt
     * |        |          |1 = Enable falling latch interrupt.
     * |        |          |When enabled, capture block generates an interrupt on falling edge of input.
     * |[3]     |CAPEN0    |Capture Channel 0 transition Enable/Disable
     * |        |          |0 = Disable capture function on channel 0
     * |        |          |1 = Enable capture function on channel 0.
     * |        |          |When enabled, Capture function latches the PMW-counter to RCAPDAT (Rising latch) and FCAPDAT (Falling latch) registers on input edge transition.
     * |        |          |When disabled, Capture function is inactive as is interrupt.
     * |[4]     |CAPIF0    |Capture0 Interrupt Indication Flag
     * |        |          |If channel 0 rising latch interrupt is enabled (CRLIEN0 = 1), a rising transition at input channel 0 will result in CAPIF0 to high; Similarly, a falling transition will cause CAPIF0 to be set high if channel 0 falling latch interrupt is enabled (CFLIEN0 = 1).
     * |        |          |This flag is cleared by software writing a '1' to it.
     * |[6]     |CRLIF0    |PWM_RCAPDAT0 Latched Indicator Bit
     * |        |          |When input channel 0 has a rising transition, PWM_RCAPDAT0 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it.
     * |[7]     |CFLIF0    |PWM_FCAPDAT0 Latched Indicator Bit
     * |        |          |When input channel 0 has a falling transition, PWM_FCAPDAT0 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it
     * |[16]    |CAPINV1   |Channel 1 Inverter ON/OFF
     * |        |          |0 = Inverter OFF
     * |        |          |1 = Inverter ON. Reverse the input signal from GPIO before Capture timer
     * |[17]    |CRLIEN1   |Channel 1 Rising Latch Interrupt Enable
     * |        |          |0 = Disable rising edge latch interrupt
     * |        |          |1 = Enable rising edge latch interrupt.
     * |        |          |When enabled, capture block generates an interrupt on rising edge of input.
     * |[18]    |CFLIEN1   |Channel 1 Falling Latch Interrupt Enable
     * |        |          |0 = Disable falling edge latch interrupt
     * |        |          |1 = Enable falling edge latch interrupt.
     * |        |          |When enabled, capture block generates an interrupt on falling edge of input.
     * |[19]    |CAPEN1    |Capture Channel 1 Transition Enable/Disable
     * |        |          |0 = Disable capture function on channel 1
     * |        |          |1 = Enable capture function on channel 1.
     * |        |          |When enabled, Capture function latches the PMW-counter to RCAPDAT (Rising latch) and FCAPDAT (Falling latch) registers on input edge transition.
     * |        |          |When disabled, Capture function is inactive as is interrupt.
     * |[20]    |CAPIF1    |Capture1 Interrupt Indication Flag
     * |        |          |If channel 1 rising latch interrupt is enabled (CRLIEN1 = 1), a rising transition at input channel 1 will result in CAPIF1 to high; Similarly, a falling transition will cause CAPIF1 to be set high if channel 1 falling latch interrupt is enabled (CFLIEN1 = 1).
     * |        |          |This flag is cleared by software writing a '1' to it.
     * |[22]    |CRLIF1    |PWM_RCAPDAT1 Latched Indicator Bit
     * |        |          |When input channel 1 has a rising transition, PWM_RCAPDAT1 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it.
     * |[23]    |CFLIF1    |PWM_FCAPDAT1 Latched Indicator Bit
     * |        |          |When input channel 1 has a falling transition, PWM_FCAPDAT1 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it
 */
    __IO uint32_t CAPCTL01;              
         uint32_t RESERVE2[1];


    /**
     * RCAPDAT0
     * ===================================================================================================
     * Offset: 0x58  Capture Rising Latch Register (Channel 0)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |RCAPDAT   |Capture Rising Latch Register
     * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a rising edge of the input signal.
 */
    __I  uint32_t RCAPDAT0;              

    /**
     * FCAPDAT0
     * ===================================================================================================
     * Offset: 0x5C  Capture Falling Latch Register (Channel 0)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |FCAPDAT   |Capture Falling Latch Register
     * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a falling edge of the input signal.
 */
    __I  uint32_t FCAPDAT0;              

    /**
     * RCAPDAT1
     * ===================================================================================================
     * Offset: 0x60  Capture Rising Latch Register (Channel 1)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |RCAPDAT   |Capture Rising Latch Register
     * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a rising edge of the input signal.
 */
    __I  uint32_t RCAPDAT1;              

    /**
     * FCAPDAT1
     * ===================================================================================================
     * Offset: 0x64  Capture Falling Latch Register (Channel 1)
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |FCAPDAT   |Capture Falling Latch Register
     * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a falling edge of the input signal.
 */
    __I  uint32_t FCAPDAT1;              
         uint32_t RESERVE3[4];


    /**
     * CAPINEN
     * ===================================================================================================
     * Offset: 0x78  Capture Input Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |CAPINEN   |Capture Input Enable Register
     * |        |          |0 : OFF (PA[13:12] pin input disconnected from Capture block)
     * |        |          |1 : ON (PA[13:12] pin, if in PWM alternative function, will be configured as an input and fed to capture function)
     * |        |          |CAPINEN[1:0]
     * |        |          |Bit 10
     * |        |          |Bit x1 : Capture channel 0 is from PA [12]
     * |        |          |Bit 1x : Capture channel 1 is from PA [13] 
 */
    __IO uint32_t CAPINEN;               

    /**
     * POEN
     * ===================================================================================================
     * Offset: 0x7C  PWM Output Enable Register for PWM CH0~1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |POEN0     |PWM CH0 Output Enable Register
     * |        |          |0 = Disable PWM CH0 output to pin.
     * |        |          |1 = Enable PWM CH0 output to pin.
     * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP Table 5-7)
     * |[1]     |POEN1     |PWM CH1 Output Enable Register
     * |        |          |0 = Disable PWM CH1 output to pin.
     * |        |          |1 = Enable PWM CH1 output to pin.
     * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP Table 5-7)
 */
    __IO uint32_t POEN;                  

} PWM_T;

/**
    @addtogroup PWM_CONST PWM Bit Field Definition
    Constant Definitions for PWM Controller
@{ */

#define PWM_CLKPSC_CLKPSC01_Pos          (0)                                               /*!< PWM CLKPSC: CLKPSC01 Position          */
#define PWM_CLKPSC_CLKPSC01_Msk          (0xfful << PWM_CLKPSC_CLKPSC01_Pos)               /*!< PWM CLKPSC: CLKPSC01 Mask              */

#define PWM_CLKPSC_DTCNT01_Pos           (16)                                              /*!< PWM CLKPSC: DTCNT01 Position           */
#define PWM_CLKPSC_DTCNT01_Msk           (0xfful << PWM_CLKPSC_DTCNT01_Pos)                /*!< PWM CLKPSC: DTCNT01 Mask               */

#define PWM_CLKDIV_CLKDIV0_Pos           (0)                                               /*!< PWM CLKDIV: CLKDIV0 Position           */
#define PWM_CLKDIV_CLKDIV0_Msk           (0x7ul << PWM_CLKDIV_CLKDIV0_Pos)                 /*!< PWM CLKDIV: CLKDIV0 Mask               */

#define PWM_CLKDIV_CLKDIV1_Pos           (4)                                               /*!< PWM CLKDIV: CLKDIV1 Position           */
#define PWM_CLKDIV_CLKDIV1_Msk           (0x7ul << PWM_CLKDIV_CLKDIV1_Pos)                 /*!< PWM CLKDIV: CLKDIV1 Mask               */

#define PWM_CTL_CNTEN0_Pos               (0)                                               /*!< PWM CTL: CNTEN0 Position               */
#define PWM_CTL_CNTEN0_Msk               (0x1ul << PWM_CTL_CNTEN0_Pos)                     /*!< PWM CTL: CNTEN0 Mask                   */

#define PWM_CTL_PINV0_Pos          	     (2)                                               /*!< PWM CTL: PINV0 Position               */
#define PWM_CTL_PINV0_Msk           	   (0x1ul << PWM_CTL_PINV0_Pos)       	             /*!< PWM CTL: PINV0 Mask                   */

#define PWM_CTL_CNTMODE0_Pos             (3)                                               /*!< PWM CTL: CNTMODE0 Position               */
#define PWM_CTL_CNTMODE0_Msk             (0x1ul << PWM_CTL_CNTMODE0_Pos)                   /*!< PWM CTL: CNTMODE0 Mask                   */

#define PWM_CTL_DTEN01_Pos               (4)                                               /*!< PWM CTL: DTEN01 Position               */
#define PWM_CTL_DTEN01_Msk               (0x1ul << PWM_CTL_DTEN01_Pos)                     /*!< PWM CTL: DTEN01 Mask                   */

#define PWM_CTL_CNTEN1_Pos               (8)                                               /*!< PWM CTL: CNTEN1 Position               */
#define PWM_CTL_CNTEN1_Msk               (0x1ul << PWM_CTL_CNTEN1_Pos)                     /*!< PWM CTL: CNTEN1 Mask                   */

#define PWM_CTL_PINV1_Pos                (10)                                              /*!< PWM CTL: PINV1 Position                */
#define PWM_CTL_PINV1_Msk                (0x1ul << PWM_CTL_PINV1_Pos)                      /*!< PWM CTL: PINV1 Mask                    */

#define PWM_CTL_CNTMODE1_Pos             (11)                                              /*!< PWM CTL: CNTMODE1 Position             */
#define PWM_CTL_CNTMODE1_Msk             (0x1ul << PWM_CTL_CNTMODE1_Pos)                   /*!< PWM CTL: CNTMODE1 Mask                 */

#define PWM_PERIOD0_PERIOD_Pos           (0)                                               /*!< PWM PERIOD0: PERIOD Position           */
#define PWM_PERIOD0_PERIOD_Msk           (0xfffful << PWM_PERIOD0_PERIOD_Pos)              /*!< PWM PERIOD0: PERIOD Mask               */

#define PWM_CMPDAT0_CMP_Pos              (0)                                               /*!< PWM CMPDAT0: CMP Position              */
#define PWM_CMPDAT0_CMP_Msk              (0xfffful << PWM_CMPDAT0_CMP_Pos)                 /*!< PWM CMPDAT0: CMP Mask                  */

#define PWM_CNT0_CNT_Pos                 (0)                                               /*!< PWM CNT0: CNT Position                 */
#define PWM_CNT0_CNT_Msk                 (0xfffful << PWM_CNT0_CNT_Pos)                    /*!< PWM CNT0: CNT Mask                     */

#define PWM_PERIOD1_PERIOD_Pos           (0)                                               /*!< PWM PERIOD1: PERIOD Position           */
#define PWM_PERIOD1_PERIOD_Msk           (0xfffful << PWM_PERIOD1_PERIOD_Pos)              /*!< PWM PERIOD1: PERIOD Mask               */

#define PWM_CMPDAT1_CMP_Pos              (0)                                               /*!< PWM CMPDAT1: CMP Position              */
#define PWM_CMPDAT1_CMP_Msk              (0xfffful << PWM_CMPDAT1_CMP_Pos)                 /*!< PWM CMPDAT1: CMP Mask                  */

#define PWM_CNT1_CNT_Pos                 (0)                                               /*!< PWM CNT1: CNT Position                 */
#define PWM_CNT1_CNT_Msk                 (0xfffful << PWM_CNT1_CNT_Pos)                    /*!< PWM CNT1: CNT Mask                     */

#define PWM_INTEN_PIEN0_Pos              (0)                                               /*!< PWM INTEN: PIEN0 Position              */
#define PWM_INTEN_PIEN0_Msk              (0x1ul << PWM_INTEN_PIEN0_Pos)                    /*!< PWM INTEN: PIEN0 Mask                  */

#define PWM_INTEN_PIEN1_Pos              (1)                                               /*!< PWM INTEN: PIEN1 Position              */
#define PWM_INTEN_PIEN1_Msk              (0x1ul << PWM_INTEN_PIEN1_Pos)                    /*!< PWM INTEN: PIEN1 Mask                  */

#define PWM_INTSTS_PIF0_Pos              (0)                                               /*!< PWM INTSTS: PIF0 Position              */
#define PWM_INTSTS_PIF0_Msk              (0x1ul << PWM_INTSTS_PIF0_Pos)                    /*!< PWM INTSTS: PIF0 Mask                  */

#define PWM_INTSTS_PIF1_Pos              (1)                                               /*!< PWM INTSTS: PIF1 Position              */
#define PWM_INTSTS_PIF1_Msk              (0x1ul << PWM_INTSTS_PIF1_Pos)                    /*!< PWM INTSTS: PIF1 Mask                  */

#define PWM_CAPCTL01_CAPINV0_Pos         (0)                                               /*!< PWM CAPCTL01: CAPINV0 Position         */
#define PWM_CAPCTL01_CAPINV0_Msk         (0x1ul << PWM_CAPCTL01_CAPINV0_Pos)               /*!< PWM CAPCTL01: CAPINV0 Mask             */

#define PWM_CAPCTL01_CRLIEN0_Pos         (1)                                               /*!< PWM CAPCTL01: CRLIEN0 Position         */
#define PWM_CAPCTL01_CRLIEN0_Msk         (0x1ul << PWM_CAPCTL01_CRLIEN0_Pos)               /*!< PWM CAPCTL01: CRLIEN0 Mask             */

#define PWM_CAPCTL01_CFLIEN0_Pos         (2)                                               /*!< PWM CAPCTL01: CFLIEN0 Position         */
#define PWM_CAPCTL01_CFLIEN0_Msk         (0x1ul << PWM_CAPCTL01_CFLIEN0_Pos)               /*!< PWM CAPCTL01: CFLIEN0 Mask             */

#define PWM_CAPCTL01_CAPEN0_Pos          (3)                                               /*!< PWM CAPCTL01: CAPEN0 Position          */
#define PWM_CAPCTL01_CAPEN0_Msk          (0x1ul << PWM_CAPCTL01_CAPEN0_Pos)                /*!< PWM CAPCTL01: CAPEN0 Mask              */

#define PWM_CAPCTL01_CAPIF0_Pos          (4)                                               /*!< PWM CAPCTL01: CAPIF0 Position          */
#define PWM_CAPCTL01_CAPIF0_Msk          (0x1ul << PWM_CAPCTL01_CAPIF0_Pos)                /*!< PWM CAPCTL01: CAPIF0 Mask              */

#define PWM_CAPCTL01_CRLIF0_Pos          (6)                                               /*!< PWM CAPCTL01: CRLIF0 Position          */
#define PWM_CAPCTL01_CRLIF0_Msk          (0x1ul << PWM_CAPCTL01_CRLIF0_Pos)                /*!< PWM CAPCTL01: CRLIF0 Mask              */

#define PWM_CAPCTL01_CFLIF0_Pos          (7)                                               /*!< PWM CAPCTL01: CFLIF0 Position          */
#define PWM_CAPCTL01_CFLIF0_Msk          (0x1ul << PWM_CAPCTL01_CFLIF0_Pos)                /*!< PWM CAPCTL01: CFLIF0 Mask              */

#define PWM_CAPCTL01_CAPINV1_Pos         (16)                                              /*!< PWM CAPCTL01: CAPINV1 Position         */
#define PWM_CAPCTL01_CAPINV1_Msk         (0x1ul << PWM_CAPCTL01_CAPINV1_Pos)               /*!< PWM CAPCTL01: CAPINV1 Mask             */

#define PWM_CAPCTL01_CRLIEN1_Pos         (17)                                              /*!< PWM CAPCTL01: CRLIEN1 Position         */
#define PWM_CAPCTL01_CRLIEN1_Msk         (0x1ul << PWM_CAPCTL01_CRLIEN1_Pos)               /*!< PWM CAPCTL01: CRLIEN1 Mask             */

#define PWM_CAPCTL01_CFLIEN1_Pos         (18)                                              /*!< PWM CAPCTL01: CFLIEN1 Position         */
#define PWM_CAPCTL01_CFLIEN1_Msk         (0x1ul << PWM_CAPCTL01_CFLIEN1_Pos)               /*!< PWM CAPCTL01: CFLIEN1 Mask             */

#define PWM_CAPCTL01_CAPEN1_Pos          (19)                                              /*!< PWM CAPCTL01: CAPEN1 Position          */
#define PWM_CAPCTL01_CAPEN1_Msk          (0x1ul << PWM_CAPCTL01_CAPEN1_Pos)                /*!< PWM CAPCTL01: CAPEN1 Mask              */

#define PWM_CAPCTL01_CAPIF1_Pos          (20)                                              /*!< PWM CAPCTL01: CAPIF1 Position          */
#define PWM_CAPCTL01_CAPIF1_Msk          (0x1ul << PWM_CAPCTL01_CAPIF1_Pos)                /*!< PWM CAPCTL01: CAPIF1 Mask              */

#define PWM_CAPCTL01_CRLIF1_Pos          (22)                                              /*!< PWM CAPCTL01: CRLIF1 Position          */
#define PWM_CAPCTL01_CRLIF1_Msk          (0x1ul << PWM_CAPCTL01_CRLIF1_Pos)                /*!< PWM CAPCTL01: CRLIF1 Mask              */

#define PWM_CAPCTL01_CFLIF1_Pos          (23)                                              /*!< PWM CAPCTL01: CFLIF1 Position          */
#define PWM_CAPCTL01_CFLIF1_Msk          (0x1ul << PWM_CAPCTL01_CFLIF1_Pos)                /*!< PWM CAPCTL01: CFLIF1 Mask              */

#define PWM_RCAPDAT0_RCAPDAT_Pos         (0)                                               /*!< PWM RCAPDAT0: RCAPDAT Position         */
#define PWM_RCAPDAT0_RCAPDAT_Msk         (0xfffful << PWM_RCAPDAT0_RCAPDAT_Pos)            /*!< PWM RCAPDAT0: RCAPDAT Mask             */

#define PWM_FCAPDAT0_FCAPDAT_Pos         (0)                                               /*!< PWM FCAPDAT0: FCAPDAT Position         */
#define PWM_FCAPDAT0_FCAPDAT_Msk         (0xfffful << PWM_FCAPDAT0_FCAPDAT_Pos)            /*!< PWM FCAPDAT0: FCAPDAT Mask             */

#define PWM_RCAPDAT1_RCAPDAT_Pos         (0)                                               /*!< PWM RCAPDAT1: RCAPDAT Position         */
#define PWM_RCAPDAT1_RCAPDAT_Msk         (0xfffful << PWM_RCAPDAT1_RCAPDAT_Pos)            /*!< PWM RCAPDAT1: RCAPDAT Mask             */

#define PWM_FCAPDAT1_FCAPDAT_Pos         (0)                                               /*!< PWM FCAPDAT1: FCAPDAT Position         */
#define PWM_FCAPDAT1_FCAPDAT_Msk         (0xfffful << PWM_FCAPDAT1_FCAPDAT_Pos)            /*!< PWM FCAPDAT1: FCAPDAT Mask             */

#define PWM_CAPINEN_CAPINEN_Pos          (0)                                               /*!< PWM CAPINEN: CAPINEN Position          */
#define PWM_CAPINEN_CAPINEN_Msk          (0x3ul << PWM_CAPINEN_CAPINEN_Pos)                /*!< PWM CAPINEN: CAPINEN Mask              */

#define PWM_POEN_POEN0_Pos               (0)                                               /*!< PWM POEN: POEN0 Position               */
#define PWM_POEN_POEN0_Msk               (0x1ul << PWM_POEN_POEN0_Pos)                     /*!< PWM POEN: POEN0 Mask                   */

#define PWM_POEN_POEN1_Pos               (1)                                               /*!< PWM POEN: POEN1 Position               */
#define PWM_POEN_POEN1_Msk               (0x1ul << PWM_POEN_POEN1_Pos)                     /*!< PWM POEN: POEN1 Mask                   */

/**@}*/ /* PWM_CONST */
/**@}*/ /* end of PWM register group */


/*---------------------- Real Time Clock Controller -------------------------*/
/**
    @addtogroup RTC Real Time Clock Controller(RTC)
    Memory Mapped Structure for RTC Controller
@{ */
 
typedef struct
{


    /**
     * INIT
     * ===================================================================================================
     * Offset: 0x00  RTC Initialization Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ATVSTS    |RTC Active Status (Read only)
     * |        |          |0: RTC is in reset state
     * |        |          |1: RTC is in normal active state.
     * |[1:31]  |INIT      |RTC Initialization
     * |        |          |After a power-on reset (POR) RTC block should be initialized by writing 0xA5EB1357 to INIT.
     * |        |          |This will force a hardware reset then release all logic and counters.
 */
    __IO  uint32_t INIT;                  

    /**
     * RWEN
     * ===================================================================================================
     * Offset: 0x04  RTC Access Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:15]  |RWEN      |RTC Register Access Enable Password (Write only)
     * |        |          |0xA965 = Enable RTC access
     * |        |          |Others = Disable RTC access 
     * |[16]    |RWENF     |RTC Register Access Enable Flag (Read only)
     * |        |          |1 = RTC register read/write enable.
     * |        |          |0 = RTC register read/write disable
     * |        |          |This bit will be set after RWEN[15:0] register is set to 0xA965, it will clear automatically in 512 RTC clock cycles or RWEN[15:0] ! = 0xA965.
     * |        |          |The effect of RTC_RWEN.RWENF on access to each register is given Table 5-72.
     * |        |          |Table 5-72 RTC_RWEN.RWENF Register Access Effect.
     * |        |          |Register : RWENF = 1 : RWENF = 0
     * |        |          |RTC_INIT : R/W : R/W
     * |        |          |RTC_FREQADJ : R/W : -
     * |        |          |RTC_TIME : R/W : R
     * |        |          |RTC_CAL : R/W : R
     * |        |          |RTC_CLKFMT : R/W : R/W
     * |        |          |RTC_WEEKDAY : R/W : R
     * |        |          |RTC_TALM : R/W : -
     * |        |          |RTC_CALM : R/W : -
     * |        |          |RTC_LEAPYEAR : R : R
     * |        |          |RTC_INTEN : R/W : R/W
     * |        |          |RTC_INTSTS : R/W : R/W
     * |        |          |RTC_TICK : R/W : -
 */
    __O  uint32_t RWEN;                  

    /**
     * FREQADJ
     * ===================================================================================================
     * Offset: 0x08  RTC Frequency Compensation Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:5]   |FRACTION  |Fractional Part
     * |        |          |Formula = (fraction part of detected value) x 60
     * |        |          |Refer to 5.8.4.4 for the examples.
     * |[8:11]  |INTEGER   |Integer Part
     * |        |          |Register should contain the value (INT(Factual) - 32761)
     * |        |          |Ex: Integer part of detected value = 32772,
     * |        |          | RTC_FREQADJ.INTEGER = 32772-32761 = 11 (1011b)
     * |        |          |The range between 32761 and 32776
 */
    __IO uint32_t FREQADJ;               

    /**
     * TIME
     * ===================================================================================================
     * Offset: 0x0C  Time Load Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |SEC       |1 Sec Time Digit (0~9)
     * |[4:6]   |TENSEC    |10 Sec Time Digit (0~5)
     * |[8:11]  |MIN       |1 Min Time Digit (0~9)
     * |[12:14] |TENMIN    |10 Min Time Digit (0~5)
     * |[16:19] |HR        |1 Hour Time Digit (0~9)
     * |[20:21] |TENHR     |10 Hour Time Digit (0~3)
 */
    __IO uint32_t TIME;                  

    /**
     * CAL
     * ===================================================================================================
     * Offset: 0x10  Calendar Load Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |DAY       |1-Day Calendar Digit (0~9)
     * |[4:5]   |TENDAY    |10-Day Calendar Digit (0~3)
     * |[8:11]  |MON       |1-Month Calendar Digit (0~9)
     * |[12]    |TENMON    |10-Month Calendar Digit (0~1)
     * |[16:19] |YEAR      |1-Year Calendar Digit (0~9)
     * |[20:23] |TENYEAR   |10-Year Calendar Digit (0~9)
 */
    __IO uint32_t CAL;                   

    /**
     * CLKFMT
     * ===================================================================================================
     * Offset: 0x14  Time Scale Selection Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |24HEN     |24-Hour / 12-Hour Mode Selection
     * |        |          |Determines whether RTC_TIME and RTC_TALM are in 24-hour mode or 12-hour mode
     * |        |          |1 = select 24-hour time scale
     * |        |          |0 = select 12-hour time scale with AM and PM indication
     * |        |          |The range of 24-hour time scale is between 0 and 23.
     * |        |          |12-hour time scale:
     * |        |          |01(AM01), 02(AM02), 03(AM03), 04(AM04), 05(AM05), 06(AM06)
     * |        |          |07(AM07), 08(AM08), 09(AM09), 10(AM10), 11(AM11), 12(AM12)
     * |        |          |21(PM01), 22(PM02), 23(PM03), 24(PM04), 25(PM05), 26(PM06)
     * |        |          |27(PM07), 28(PM08), 29(PM09), 30(PM10), 31(PM11), 32(PM12)
 */
    __IO uint32_t CLKFMT;                

    /**
     * WEEKDAY
     * ===================================================================================================
     * Offset: 0x18  Day of the Week Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:2]   |WEEKDAY   |Day of the Week Register
     * |        |          |0 (Sunday), 1 (Monday), 2 (Tuesday), 3 (Wednesday)
     * |        |          |4 (Thursday), 5 (Friday), 6 (Saturday)
 */
    __IO uint32_t WEEKDAY;               

    /**
     * TALM
     * ===================================================================================================
     * Offset: 0x1C  Time Alarm Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |SEC       |1 Sec Time Digit of Alarm Setting (0~9)
     * |[4:6]   |TENSEC    |10 Sec Time Digit of Alarm Setting (0~5)
     * |[8:11]  |MIN       |1 Min Time Digit of Alarm Setting (0~9)
     * |[12:14] |TENMIN    |10 Min Time Digit of Alarm Setting (0~5)
     * |[16:19] |HR        |1 Hour Time Digit of Alarm Setting (0~9)
     * |[20:21] |TENHR     |10 Hour Time Digit of Alarm Setting (0~3)
 */
    __IO uint32_t TALM;                  

    /**
     * CALM
     * ===================================================================================================
     * Offset: 0x20  Calendar Alarm Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:3]   |DAY       |1-Day Calendar Digit of Alarm Setting (0~9)
     * |[4:5]   |TENDAY    |10-Day Calendar Digit of Alarm Setting (0~3)
     * |[8:11]  |MON       |1-Month Calendar Digit of Alarm Setting (0~9)
     * |[12]    |TENMON    |10-Month Calendar Digit of Alarm Setting (0~1)
     * |[16:19] |YEAR      |1-Year Calendar Digit of Alarm Setting (0~9)
     * |[20:23] |TENYEAR   |10-Year Calendar Digit of Alarm Setting (0~9)
 */
    __IO uint32_t CALM;                  

    /**
     * LEAPYEAR
     * ===================================================================================================
     * Offset: 0x24  Leap year Indicator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |LEAPYEAR  |Leap Year Indication Register (read only)
     * |        |          |0 = Current year is not a leap year
     * |        |          |1 = Current year is leap year
 */
    __I  uint32_t LEAPYEAR;              

    /**
     * INTEN
     * ===================================================================================================
     * Offset: 0x28  RTC Interrupt Enable Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ALMIEN    |Alarm Interrupt Enable
     * |        |          |0 = RTC Alarm Interrupt is disabled
     * |        |          |1 = RTC Alarm Interrupt is enabled
     * |[1]     |TICKIEN   |Time-Tick Interrupt and Wakeup-by-Tick Enable
     * |        |          |0 = RTC Time-Tick Interrupt is disabled.
     * |        |          |1 = RTC Time-Tick Interrupt is enabled.
 */
    __IO uint32_t INTEN;                 

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x2C  RTC Interrupt Indicator Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |ALMIF     |RTC Alarm Interrupt Flag
     * |        |          |When RTC Alarm Interrupt is enabled (RTC_INTEN.ALMIEN=1), RTC unit will set AIF to high once the RTC real time counters RTC_TIME and RTC_CAL reach the alarm setting time registers RTC_TALM and RTC_CALM.
     * |        |          |This bit cleared/acknowledged by writing 1 to it.
     * |        |          |0= Indicates no Alarm Interrupt condition.
     * |        |          |1= Indicates RTC Alarm Interrupt generated.
     * |[1]     |TICKIF    |RTC Time-Tick Interrupt Flag
     * |        |          |When RTC Time-Tick Interrupt is enabled (RTC_INTEN.TICKIEN=1), RTC unit will set TIF high at the rate selected by RTC_TICK[2:0].
     * |        |          |This bit cleared/acknowledged by writing 1 to it.
     * |        |          |0= Indicates no Time-Tick Interrupt condition.
     * |        |          |1= Indicates RTC Time-Tick Interrupt generated.
 */
    __IO uint32_t INTSTS;                

    /**
     * TICK
     * ===================================================================================================
     * Offset: 0x30  RTC Time Tick Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:2]   |TICKSEL   |Time Tick Register
     * |        |          |The RTC time tick period for Periodic Time-Tick Interrupt request.
     * |        |          |Time Tick (second) : 1 / (2^TICKSEL)
     * |        |          |Note: This register can be read back after the RTC is active.
     * |[3]     |TWKEN     |RTC Timer Wakeup CPU Function Enable Bit
     * |        |          |If TWKEN is set before CPU is in power-down mode, when a RTC Time-Tick or Alarm Match occurs, CPU will wake up.
     * |        |          |0= Disable Wakeup CPU function.
     * |        |          |1= Enable the Wakeup function.
 */
    __IO uint32_t TICK;                  

} RTC_T;

/**
    @addtogroup RTC_CONST RTC Bit Field Definition
    Constant Definitions for RTC Controller
@{ */

#define RTC_INIT_ATVSTS_Pos              (0)                                               /*!< RTC INIT: ATVSTS Position              */
#define RTC_INIT_ATVSTS_Msk              (0x1ul << RTC_INIT_ATVSTS_Pos)                    /*!< RTC INIT: ATVSTS Mask                  */

#define RTC_INIT_INIT_Pos                (1)                                               /*!< RTC INIT: INIT Position                */
#define RTC_INIT_INIT_Msk                (0x7ffffffful << RTC_INIT_INIT_Pos)               /*!< RTC INIT: INIT Mask                    */

#define RTC_RWEN_RWEN_Pos                (0)                                               /*!< RTC RWEN: RWEN Position                */
#define RTC_RWEN_RWEN_Msk                (0xfffful << RTC_RWEN_RWEN_Pos)                   /*!< RTC RWEN: RWEN Mask                    */

#define RTC_RWEN_RWENF_Pos               (16)                                              /*!< RTC RWEN: RWENF Position               */
#define RTC_RWEN_RWENF_Msk               (0x1ul << RTC_RWEN_RWENF_Pos)                     /*!< RTC RWEN: RWENF Mask                   */

#define RTC_FREQADJ_FRACTION_Pos         (0)                                               /*!< RTC FREQADJ: FRACTION Position         */
#define RTC_FREQADJ_FRACTION_Msk         (0x3ful << RTC_FREQADJ_FRACTION_Pos)              /*!< RTC FREQADJ: FRACTION Mask             */

#define RTC_FREQADJ_INTEGER_Pos          (8)                                               /*!< RTC FREQADJ: INTEGER Position          */
#define RTC_FREQADJ_INTEGER_Msk          (0xful << RTC_FREQADJ_INTEGER_Pos)                /*!< RTC FREQADJ: INTEGER Mask              */

#define RTC_TIME_SEC_Pos                 (0)                                               /*!< RTC TIME: SEC Position                 */
#define RTC_TIME_SEC_Msk                 (0xful << RTC_TIME_SEC_Pos)                       /*!< RTC TIME: SEC Mask                     */

#define RTC_TIME_TENSEC_Pos              (4)                                               /*!< RTC TIME: TENSEC Position              */
#define RTC_TIME_TENSEC_Msk              (0x7ul << RTC_TIME_TENSEC_Pos)                    /*!< RTC TIME: TENSEC Mask                  */

#define RTC_TIME_MIN_Pos                 (8)                                               /*!< RTC TIME: MIN Position                 */
#define RTC_TIME_MIN_Msk                 (0xful << RTC_TIME_MIN_Pos)                       /*!< RTC TIME: MIN Mask                     */

#define RTC_TIME_TENMIN_Pos              (12)                                              /*!< RTC TIME: TENMIN Position              */
#define RTC_TIME_TENMIN_Msk              (0x7ul << RTC_TIME_TENMIN_Pos)                    /*!< RTC TIME: TENMIN Mask                  */

#define RTC_TIME_HR_Pos                  (16)                                              /*!< RTC TIME: HR Position                  */
#define RTC_TIME_HR_Msk                  (0xful << RTC_TIME_HR_Pos)                        /*!< RTC TIME: HR Mask                      */

#define RTC_TIME_TENHR_Pos               (20)                                              /*!< RTC TIME: TENHR Position               */
#define RTC_TIME_TENHR_Msk               (0x3ul << RTC_TIME_TENHR_Pos)                     /*!< RTC TIME: TENHR Mask                   */

#define RTC_CAL_DAY_Pos                  (0)                                               /*!< RTC CAL: DAY Position                  */
#define RTC_CAL_DAY_Msk                  (0xful << RTC_CAL_DAY_Pos)                        /*!< RTC CAL: DAY Mask                      */

#define RTC_CAL_TENDAY_Pos               (4)                                               /*!< RTC CAL: TENDAY Position               */
#define RTC_CAL_TENDAY_Msk               (0x3ul << RTC_CAL_TENDAY_Pos)                     /*!< RTC CAL: TENDAY Mask                   */

#define RTC_CAL_MON_Pos                  (8)                                               /*!< RTC CAL: MON Position                  */
#define RTC_CAL_MON_Msk                  (0xful << RTC_CAL_MON_Pos)                        /*!< RTC CAL: MON Mask                      */

#define RTC_CAL_TENMON_Pos               (12)                                              /*!< RTC CAL: TENMON Position               */
#define RTC_CAL_TENMON_Msk               (0x1ul << RTC_CAL_TENMON_Pos)                     /*!< RTC CAL: TENMON Mask                   */

#define RTC_CAL_YEAR_Pos                 (16)                                              /*!< RTC CAL: YEAR Position                 */
#define RTC_CAL_YEAR_Msk                 (0xful << RTC_CAL_YEAR_Pos)                       /*!< RTC CAL: YEAR Mask                     */

#define RTC_CAL_TENYEAR_Pos              (20)                                              /*!< RTC CAL: TENYEAR Position              */
#define RTC_CAL_TENYEAR_Msk              (0xful << RTC_CAL_TENYEAR_Pos)                    /*!< RTC CAL: TENYEAR Mask                  */

#define RTC_CLKFMT_24HEN_Pos             (0)                                               /*!< RTC CLKFMT: 24HEN Position             */
#define RTC_CLKFMT_24HEN_Msk             (0x1ul << RTC_CLKFMT_24HEN_Pos)                   /*!< RTC CLKFMT: 24HEN Mask                 */

#define RTC_WEEKDAY_WEEKDAY_Pos          (0)                                               /*!< RTC WEEKDAY: WEEKDAY Position          */
#define RTC_WEEKDAY_WEEKDAY_Msk          (0x7ul << RTC_WEEKDAY_WEEKDAY_Pos)                /*!< RTC WEEKDAY: WEEKDAY Mask              */

#define RTC_TALM_SEC_Pos                 (0)                                               /*!< RTC TALM: SEC Position                 */
#define RTC_TALM_SEC_Msk                 (0xful << RTC_TALM_SEC_Pos)                       /*!< RTC TALM: SEC Mask                     */

#define RTC_TALM_TENSEC_Pos              (4)                                               /*!< RTC TALM: TENSEC Position              */
#define RTC_TALM_TENSEC_Msk              (0x7ul << RTC_TALM_TENSEC_Pos)                    /*!< RTC TALM: TENSEC Mask                  */

#define RTC_TALM_MIN_Pos                 (8)                                               /*!< RTC TALM: MIN Position                 */
#define RTC_TALM_MIN_Msk                 (0xful << RTC_TALM_MIN_Pos)                       /*!< RTC TALM: MIN Mask                     */

#define RTC_TALM_TENMIN_Pos              (12)                                              /*!< RTC TALM: TENMIN Position              */
#define RTC_TALM_TENMIN_Msk              (0x7ul << RTC_TALM_TENMIN_Pos)                    /*!< RTC TALM: TENMIN Mask                  */

#define RTC_TALM_HR_Pos                  (16)                                              /*!< RTC TALM: HR Position                  */
#define RTC_TALM_HR_Msk                  (0xful << RTC_TALM_HR_Pos)                        /*!< RTC TALM: HR Mask                      */

#define RTC_TALM_TENHR_Pos               (20)                                              /*!< RTC TALM: TENHR Position               */
#define RTC_TALM_TENHR_Msk               (0x3ul << RTC_TALM_TENHR_Pos)                     /*!< RTC TALM: TENHR Mask                   */

#define RTC_CALM_DAY_Pos                 (0)                                               /*!< RTC CALM: DAY Position                 */
#define RTC_CALM_DAY_Msk                 (0xful << RTC_CALM_DAY_Pos)                       /*!< RTC CALM: DAY Mask                     */

#define RTC_CALM_TENDAY_Pos              (4)                                               /*!< RTC CALM: TENDAY Position              */
#define RTC_CALM_TENDAY_Msk              (0x3ul << RTC_CALM_TENDAY_Pos)                    /*!< RTC CALM: TENDAY Mask                  */

#define RTC_CALM_MON_Pos                 (8)                                               /*!< RTC CALM: MON Position                 */
#define RTC_CALM_MON_Msk                 (0xful << RTC_CALM_MON_Pos)                       /*!< RTC CALM: MON Mask                     */

#define RTC_CALM_TENMON_Pos              (12)                                              /*!< RTC CALM: TENMON Position              */
#define RTC_CALM_TENMON_Msk              (0x1ul << RTC_CALM_TENMON_Pos)                    /*!< RTC CALM: TENMON Mask                  */

#define RTC_CALM_YEAR_Pos                (16)                                              /*!< RTC CALM: YEAR Position                */
#define RTC_CALM_YEAR_Msk                (0xful << RTC_CALM_YEAR_Pos)                      /*!< RTC CALM: YEAR Mask                    */

#define RTC_CALM_TENYEAR_Pos             (20)                                              /*!< RTC CALM: TENYEAR Position             */
#define RTC_CALM_TENYEAR_Msk             (0xful << RTC_CALM_TENYEAR_Pos)                   /*!< RTC CALM: TENYEAR Mask                 */

#define RTC_LEAPYEAR_LEAPYEAR_Pos        (0)                                               /*!< RTC LEAPYEAR: LEAPYEAR Position        */
#define RTC_LEAPYEAR_LEAPYEAR_Msk        (0x1ul << RTC_LEAPYEAR_LEAPYEAR_Pos)              /*!< RTC LEAPYEAR: LEAPYEAR Mask            */

#define RTC_INTEN_ALMIEN_Pos             (0)                                               /*!< RTC INTEN: ALMIEN Position             */
#define RTC_INTEN_ALMIEN_Msk             (0x1ul << RTC_INTEN_ALMIEN_Pos)                   /*!< RTC INTEN: ALMIEN Mask                 */

#define RTC_INTEN_TICKIEN_Pos            (1)                                               /*!< RTC INTEN: TICKIEN Position            */
#define RTC_INTEN_TICKIEN_Msk            (0x1ul << RTC_INTEN_TICKIEN_Pos)                  /*!< RTC INTEN: TICKIEN Mask                */

#define RTC_INTSTS_ALMIF_Pos             (0)                                               /*!< RTC INTSTS: ALMIF Position             */
#define RTC_INTSTS_ALMIF_Msk             (0x1ul << RTC_INTSTS_ALMIF_Pos)                   /*!< RTC INTSTS: ALMIF Mask                 */

#define RTC_INTSTS_TICKIF_Pos            (1)                                               /*!< RTC INTSTS: TICKIF Position            */
#define RTC_INTSTS_TICKIF_Msk            (0x1ul << RTC_INTSTS_TICKIF_Pos)                  /*!< RTC INTSTS: TICKIF Mask                */

#define RTC_TICK_TICKSEL_Pos             (0)                                               /*!< RTC TICK: TICKSEL Position             */
#define RTC_TICK_TICKSEL_Msk             (0x7ul << RTC_TICK_TICKSEL_Pos)                   /*!< RTC TICK: TICKSEL Mask                 */

#define RTC_TICK_TWKEN_Pos               (3)                                               /*!< RTC TICK: TWKEN Position               */
#define RTC_TICK_TWKEN_Msk               (0x1ul << RTC_TICK_TWKEN_Pos)                     /*!< RTC TICK: TWKEN Mask                   */

/**@}*/ /* RTC_CONST */
/**@}*/ /* end of RTC register group */


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
     * |[1]     |RXNET     |Receive At Negative Edge
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
     * |[22]    |TWOBIT    |Two Bits Transfer Mode
     * |        |          |0 = Disable two-bit transfer mode.
     * |        |          |1 = Enable two-bit transfer mode.
     * |        |          |Note that when enabled in master mode, MOSI0 data comes from SPI_TX0 and MOSI1 data from SPI_TX1.
     * |        |          |Likewise SPI_RX0 receives bit stream from MISO0 and SPI_RX1 from MISO1.
     * |        |          |Note that when enabled, the setting of TXCNT must be programmed as 0x00.
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
#define SPI_CTL_CLKP_Msk                 (0x1ul << SPI_CTL_CLKPOL_Pos)                     /*!< SPI CTL: CLKP Mask                   */

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

#define SPI_CTL_TWOBIT_Pos               (22)                                              /*!< SPI CTL: TWOBIT Position               */
#define SPI_CTL_TWOBIT_Msk               (0x1ul << SPI_CTL_TWOBIT_Pos)                     /*!< SPI CTL: TWOBIT Mask                   */

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
     * |[0:31]  |CORERSTF  |Chip Part Number Identifier
 */
         uint32_t PDID;


    /**
     * RSTSTS
     * ===================================================================================================
     * Offset: 0x04  System Reset Source Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CORERSTF  |Reset Source From CORE
     * |        |          |The CORERSTF flag is set if the core has been reset.
     * |        |          |Possible sources of reset are a Power-On Reset (POR), RESETn Pin Reset or PMU reset.
     * |        |          |0= No reset from CORE
     * |        |          |1= Core was reset by hardware block.
     * |        |          |This bit is cleared by writing 1 to itself.
     * |[2]     |WDTRF     |Reset Source From WDT
     * |        |          |The WDTRF flag is set if pervious reset source originates from the Watch-Dog module.
     * |        |          |0= No reset from Watch-Dog
     * |        |          |1= The Watch-Dog module issued the reset signal to reset the system.
     * |        |          |This bit is cleared by writing 1 to itself.
     * |[5]     |SYSRF     |Reset Source From MCU
     * |        |          |The SYSRF flag is set if the previous reset source originates from the Cortex_M0 kernel.
     * |        |          |0= No reset from MCU
     * |        |          |1= The Cortex_M0 MCU issued a reset signal to reset the system by software writing 1 to bit SYSCTL_AIRCTL.SYSRESTREQ, Application Interrupt and Reset Control Register) in system control registers of Cortex_M0 kernel.
     * |        |          |This bit is cleared by writing 1 to itself.
     * |[6]     |PMURSTF   |Reset Source From PMU
     * |        |          |The PMURSTF flag is set if the PMU.
     * |        |          |0= No reset from PMU
     * |        |          |1= PMU reset the system from a power down/standby event.
     * |        |          |This bit is cleared by writing 1 to itself.
     * |[7]     |CPURF     |Reset Source From CPU
     * |        |          |The CPURF flag is set by hardware if software writes SYS_IPRST0.CPURST with a "1" to reset Cortex-M0 CPU kernel and Flash memory controller (FMC).
     * |        |          |0= No reset from CPU
     * |        |          |1= The Cortex-M0 CPU kernel and FMC has been reset by software setting CPURST to 1.
     * |        |          |This bit is cleared by writing 1 to itself.
 */
    __IO uint32_t RSTSTS;                

    /**
     * IPRST0
     * ===================================================================================================
     * Offset: 0x08  IP Reset Control Resister1
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |CHIPRST   |CHIP One Shot Reset
     * |        |          |Set this bit will reset the whole chip, this bit will automatically return to "0" after the 2 clock cycles.
     * |        |          |CHIPRST has same behavior as POR reset, all the chip modules are reset and the chip configuration settings from flash are reloaded.
     * |        |          |This bit is a protected bit, to program first issue the unlock sequence (see Protected Register Lock Key Register (SYS_REGLCTL))
     * |        |          |0= Normal
     * |        |          |1= Reset CHIP
     * |[1]     |CPURST    |CPU Kernel One Shot Reset
     * |        |          |Setting this bit will reset the CPU kernel and Flash Memory Controller(FMC), this bit will automatically return to "0" after the 2 clock cycles
     * |        |          |This bit is a protected bit, to program first issue the unlock sequence (see Protected Register Lock Key Register (SYS_REGLCTL))
     * |        |          |0= Normal
     * |        |          |1= Reset CPU
     * |[2]     |PDMARST   |PDMA Controller Reset
     * |        |          |Set "1" will generate a reset signal to the PDMA Block.
     * |        |          |User needs to set this bit to "0" to release from the reset state.
     * |        |          |0= Normal operation
     * |        |          |1= PDMA IP reset
 */
    __IO uint32_t IPRST0;                

    /**
     * IPRST1
     * ===================================================================================================
     * Offset: 0x0C  IP Reset Control Resister2
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[6]     |TMR0RST   |Timer0 Controller Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[7]     |TMR1RST   |Timer1 Controller Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[8]     |I2C0RST   |I2C0 Controller Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[12]    |SPI0RST   |SPI0 Controller Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[13]    |DPWMRST   |DPWM Speaker Driver Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[16]    |UART0RST  |UART0 Controller Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[18]    |BIQRST    |Biquad Filter Block Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[19]    |CRCRST    |CRC Generation Block Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[20]    |PWM0RST   |PWM0 controller Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[22]    |ACMPRST   |Analog Comparator Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[28]    |EADCRST   |ADC Controller Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[29]    |I2S0RST   |I2S Controller Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
     * |[30]    |ANARST    |Analog Block Control Reset
     * |        |          |0 = Normal Operation
     * |        |          |1 = Reset
 */
    __IO uint32_t IPRST1;                
         uint32_t RESERVE1[8];


    /**
     * PASMTEN
     * ===================================================================================================
     * Offset: 0x30  GPIOA input type control register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16]    |SMTEN16   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[17]    |SMTEN17   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[18]    |SMTEN18   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[19]    |SMTEN19   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[20]    |SMTEN20   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[21]    |SMTEN21   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[22]    |SMTEN22   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[23]    |SMTEN23   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[24]    |SMTEN24   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[25]    |SMTEN25   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[26]    |SMTEN26   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[27]    |SMTEN27   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[28]    |SMTEN28   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[29]    |SMTEN29   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[30]    |SMTEN30   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
     * |[31]    |SMTEN31   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0 = GPIOA[15:0] I/O input Schmitt Trigger disabled
     * |        |          |1 = GPIOA[15:0] I/O input Schmitt Trigger enabled
 */
    __IO uint32_t PASMTEN;               

    /**
     * PBSMTEN
     * ===================================================================================================
     * Offset: 0x34  GPIOB input type control register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[16]    |SMTEN16   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger disabled
     * |        |          |1= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger enabled
     * |[17]    |SMTEN17   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger disabled
     * |        |          |1= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger enabled
     * |[18]    |SMTEN18   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger disabled
     * |        |          |1= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger enabled
     * |[19]    |SMTEN19   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger disabled
     * |        |          |1= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger enabled
     * |[20]    |SMTEN20   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger disabled
     * |        |          |1= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger enabled
     * |[21]    |SMTEN21   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger disabled
     * |        |          |1= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger enabled
     * |[22]    |SMTEN22   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger disabled
     * |        |          |1= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger enabled
     * |[23]    |SMTEN23   |Schmitt Trigger
     * |        |          |This register controls whether the GPIO input buffer Schmitt trigger is enabled.
     * |        |          |0= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger disabled
     * |        |          |1= GPIOB(port 0 ~ port 7) I/O input Schmitt Trigger enabled
 */
    __IO uint32_t PBSMTEN;               

    /**
     * GPA_MFP
     * ===================================================================================================
     * Offset: 0x38  GPIOA multiple function control register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |PA0MFP    |Alternate Function Setting For PA0MFP
     * |        |          |00 = GPIO
     * |        |          |01 = SPI_MOSI0
     * |        |          |10 = MCLK
     * |[2:3]   |PA1MFP    |Alternate Function Setting For PA1MFP
     * |        |          |00 = GPIO
     * |        |          |01 = SPI_SCLK
     * |        |          |10 = I2C_SCL
     * |[4:5]   |PA2MFP    |Alternate Function Setting For PA2MFP
     * |        |          |00 = GPIO
     * |        |          |01 = SPI_SSB0
     * |[6:7]   |PA3MFP    |Alternate Function Setting For PA3MFP
     * |        |          |00 = GPIO
     * |        |          |01 = SPI_MISO0
     * |        |          |10 = I2C_SDA
     * |[8:9]   |PA4MFP    |Alternate Function Setting For PA4MFP
     * |        |          |00 = GPIO
     * |        |          |01 = I2S_FS
     * |[10:11] |PA5MFP    |Alternate Function Setting For PA5MFP
     * |        |          |00 = GPIO
     * |        |          |01 = I2S_BCLK
     * |[12:13] |PA6MFP    |Alternate Function Setting For PA6MFP
     * |        |          |00 = GPIO
     * |        |          |01 = I2S_SDI
     * |[14:15] |PA7MFP    |Alternate Function Setting For PA7MFP
     * |        |          |00 = GPIO
     * |        |          |01 = I2S_SDO
     * |[16:17] |PA8MFP    |Alternate Function Setting For PA8MFP
     * |        |          |00 = GPIO
     * |        |          |01 = UART_TX
     * |        |          |10 = I2S_FS
     * |[18:19] |PA9MFP    |Alternate Function Setting For PA0MFP
     * |        |          |00 = GPIO
     * |        |          |01 = UART_RX
     * |        |          |10 = I2S_BCLK
     * |[20:21] |PA10MFP   |Alternate Function Setting For PA10MFP
     * |        |          |00 = GPIO
     * |        |          |01 = I2C_SDA
     * |        |          |10 = I2S_SDI
     * |        |          |11 = UART_RTSn
     * |[22:23] |PA11MFP   |Alternate Function Setting For PA11MFP
     * |        |          |00 = GPIO
     * |        |          |01 = I2C_SCL
     * |        |          |10 = I2S_SDO
     * |        |          |11 = UART_CTSn
     * |[24:25] |PA12MFP   |Alternate Function Setting For PA12MFP
     * |        |          |00 = GPIO
     * |        |          |01 = PWM0CH0
     * |        |          |10 = SPKP
     * |        |          |11 = I2S_FS
     * |[26:27] |PA13MFP   |Alternate Function Setting For PA13MFP
     * |        |          |00 = GPIO
     * |        |          |01 = PWM0CH1
     * |        |          |10 = SPKM
     * |        |          |11 = I2S_BCLK
     * |[28:29] |PA14MFP   |Alternate Function Setting For PA14MFP
     * |        |          |00 = GPIO
     * |        |          |01 = TM0
     * |        |          |10 = SDCLK
     * |        |          |11 = SDCLKn
     * |[30:31] |PA15MFP   |Alternate Function Setting For PA15MFP
     * |        |          |00 = GPIO
     * |        |          |01 = TM1
     * |        |          |10 = SDIN
 */
    __IO uint32_t GPA_MFP;               

    /**
     * GPB_MFP
     * ===================================================================================================
     * Offset: 0x3C  GPIOB multiple function control register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:1]   |PB0MFP    |Alternate Function Setting For PB0MFP
     * |        |          |00 = GPIO
     * |        |          |01 = SPI_SSB1
     * |        |          |10 = CMP0
     * |        |          |11 = SPI_SSB0
     * |[2:3]   |PB1MFP    |Alternate Function Setting For PB1MFP
     * |        |          |00 = GPIO
     * |        |          |01 = MCLK
     * |        |          |10 = CMP1
     * |        |          |11 = SPI_SSB1
     * |[4:5]   |PB2MFP    |Alternate Function Setting For PB2MFP
     * |        |          |00 = GPIO
     * |        |          |01 = I2C_SCL
     * |        |          |10 = CMP2
     * |        |          |11 = SPI_SCLK
     * |[6:7]   |PB3MFP    |Alternate Function Setting For PB3MFP
     * |        |          |00 = GPIO
     * |        |          |01 = I2C_SDA
     * |        |          |10 = CMP3
     * |        |          |11 = SPI_MISO0
     * |[8:9]   |PB4MFP    |Alternate Function Setting For PB4MFP
     * |        |          |00 = GPIO
     * |        |          |01 = PWM0CH0_INV(PWM0 channel 0 invert output pin)
     * |        |          |10 = CMP4
     * |        |          |11 = SPI_MOSI0
     * |[10:11] |PB5MFP    |Alternate Function Setting For PB5MFP
     * |        |          |00 = GPIO
     * |        |          |01 = PWM0CH1_INV(PWM0 channel 1 invert output pin)
     * |        |          |10 = CMP5
     * |        |          |11 = SPI_MISO1
     * |[12:13] |PB6MFP    |Alternate Function Setting For PB6MFP
     * |        |          |00 = GPIO
     * |        |          |01 = I2S_SDI
     * |        |          |10 = CMP6
     * |        |          |11 = SPI_MOSI1
     * |[14:15] |PB7MFP    |Alternate Function Setting For PB7MFP
     * |        |          |00 = GPIO
     * |        |          |01 = I2S_SDO
     * |        |          |10 = CMP7
 */
    __IO uint32_t GPB_MFP;               
         uint32_t RESERVE2[5];


    /**
     * WKCTL
     * ===================================================================================================
     * Offset: 0x54  WAKEUP pin control register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |WKDIN     |State Of Wakeup Pin
     * |        |          |Read only.
     * |[1]     |WKPUEN    |Wakeup Pin Pull-up Control
     * |        |          |This signal is latched in deep power down and preserved.
     * |        |          |0 = pull-up enable
     * |        |          |1 = tristate (default)
     * |[2]     |WKOENB    |Wakeup Pin Output Enable Bar
     * |        |          |0 = drive WKDOUT to pin
     * |        |          |1 = tristate (default)
     * |[3]     |WKDOUT    |Wakeup Output State
     * |        |          |0 = drive Low if the corresponding output mode bit is set (default)
     * |        |          |1 = drive High if the corresponding output mode bit is set
 */
    __IO uint32_t WKCTL;                 
         uint32_t RESERVE3[42];


    /**
     * REGLCTL
     * ===================================================================================================
     * Offset: 0x100  Register Lock Key Address register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |REGLCTL   |Protected Register Unlock Register
     * |        |          |0 = Protected registers are locked. Any write to the target register is ignored.
     * |        |          |1 = Protected registers are unlocked
 */
    __IO uint32_t REGLCTL;               
         uint32_t RESERVE4[3];


    /**
     * IRCTCTL
     * ===================================================================================================
     * Offset: 0x110  Oscillator Frequency Adjustment control register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:7]   |FREQ0SEL  |8 Bit Trim For Oscillator
     * |        |          |FREQ0SEL [7:5] are 8 coarse trim ranges which overlap in frequency.
     * |        |          |FREQ0SEL [4:0] are 32 fine trim steps of approximately 0.5% resolution.
     * |[8]     |RGE0SEL   |Range Bit For Oscillator
     * |        |          |0 = high range
     * |        |          |1 = low range
     * |[16:23] |FREQ1SEL  |8 Bit Trim For Oscillator
     * |        |          |FREQ1SEL [7:5] are 8 coarse trim ranges which overlap in frequency.
     * |        |          |FREQ1SEL [4:0] are 32 fine trim steps of approximately 0.5% resolution.
     * |[24]    |RGE1SEL   |Range Bit For Oscillator
     * |        |          |0 = high range
     * |        |          |1 = low range
 */
    __IO uint32_t IRCTCTL;               

} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */

#define SYS_RSTSTS_CORERSTF_Pos          (0)                                               /*!< SYS RSTSTS: CORERSTF Position          */
#define SYS_RSTSTS_CORERSTF_Msk          (0x1ul << SYS_RSTSTS_CORERSTF_Pos)                /*!< SYS RSTSTS: CORERSTF Mask              */

#define SYS_RSTSTS_WDTRF_Pos             (2)                                               /*!< SYS RSTSTS: WDTRF Position             */
#define SYS_RSTSTS_WDTRF_Msk             (0x1ul << SYS_RSTSTS_WDTRF_Pos)                   /*!< SYS RSTSTS: WDTRF Mask                 */

#define SYS_RSTSTS_SYSRF_Pos             (5)                                               /*!< SYS RSTSTS: SYSRF Position             */
#define SYS_RSTSTS_SYSRF_Msk             (0x1ul << SYS_RSTSTS_SYSRF_Pos)                   /*!< SYS RSTSTS: SYSRF Mask                 */

#define SYS_RSTSTS_PMURSTF_Pos           (6)                                               /*!< SYS RSTSTS: PMURSTF Position           */
#define SYS_RSTSTS_PMURSTF_Msk           (0x1ul << SYS_RSTSTS_PMURSTF_Pos)                 /*!< SYS RSTSTS: PMURSTF Mask               */

#define SYS_RSTSTS_CPURF_Pos             (7)                                               /*!< SYS RSTSTS: CPURF Position             */
#define SYS_RSTSTS_CPURF_Msk             (0x1ul << SYS_RSTSTS_CPURF_Pos)                   /*!< SYS RSTSTS: CPURF Mask                 */

#define SYS_IPRST0_CHIPRST_Pos           (0)                                               /*!< SYS IPRST0: CHIPRST Position           */
#define SYS_IPRST0_CHIPRST_Msk           (0x1ul << SYS_IPRST0_CHIPRST_Pos)                 /*!< SYS IPRST0: CHIPRST Mask               */

#define SYS_IPRST0_CPURST_Pos            (1)                                               /*!< SYS IPRST0: CPURST Position            */
#define SYS_IPRST0_CPURST_Msk            (0x1ul << SYS_IPRST0_CPURST_Pos)                  /*!< SYS IPRST0: CPURST Mask                */

#define SYS_IPRST0_PDMARST_Pos           (2)                                               /*!< SYS IPRST0: PDMARST Position           */
#define SYS_IPRST0_PDMARST_Msk           (0x1ul << SYS_IPRST0_PDMARST_Pos)                 /*!< SYS IPRST0: PDMARST Mask               */

#define SYS_IPRST1_TMR0RST_Pos           (6)                                               /*!< SYS IPRST1: TMR0RST Position           */
#define SYS_IPRST1_TMR0RST_Msk           (0x1ul << SYS_IPRST1_TMR0RST_Pos)                 /*!< SYS IPRST1: TMR0RST Mask               */

#define SYS_IPRST1_TMR1RST_Pos           (7)                                               /*!< SYS IPRST1: TMR1RST Position           */
#define SYS_IPRST1_TMR1RST_Msk           (0x1ul << SYS_IPRST1_TMR1RST_Pos)                 /*!< SYS IPRST1: TMR1RST Mask               */

#define SYS_IPRST1_I2C0RST_Pos           (8)                                               /*!< SYS IPRST1: I2C0RST Position           */
#define SYS_IPRST1_I2C0RST_Msk           (0x1ul << SYS_IPRST1_I2C0RST_Pos)                 /*!< SYS IPRST1: I2C0RST Mask               */

#define SYS_IPRST1_SPI0RST_Pos           (12)                                              /*!< SYS IPRST1: SPI0RST Position           */
#define SYS_IPRST1_SPI0RST_Msk           (0x1ul << SYS_IPRST1_SPI0RST_Pos)                 /*!< SYS IPRST1: SPI0RST Mask               */

#define SYS_IPRST1_DPWMRST_Pos           (13)                                              /*!< SYS IPRST1: DPWMRST Position           */
#define SYS_IPRST1_DPWMRST_Msk           (0x1ul << SYS_IPRST1_DPWMRST_Pos)                 /*!< SYS IPRST1: DPWMRST Mask               */

#define SYS_IPRST1_UART0RST_Pos          (16)                                              /*!< SYS IPRST1: UART0RST Position          */
#define SYS_IPRST1_UART0RST_Msk          (0x1ul << SYS_IPRST1_UART0RST_Pos)                /*!< SYS IPRST1: UART0RST Mask              */

#define SYS_IPRST1_BIQRST_Pos            (18)                                              /*!< SYS IPRST1: BIQRST Position            */
#define SYS_IPRST1_BIQRST_Msk            (0x1ul << SYS_IPRST1_BIQRST_Pos)                  /*!< SYS IPRST1: BIQRST Mask                */

#define SYS_IPRST1_CRCRST_Pos            (19)                                              /*!< SYS IPRST1: CRCRST Position            */
#define SYS_IPRST1_CRCRST_Msk            (0x1ul << SYS_IPRST1_CRCRST_Pos)                  /*!< SYS IPRST1: CRCRST Mask                */

#define SYS_IPRST1_PWM0RST_Pos           (20)                                              /*!< SYS IPRST1: PWM0RST Position           */
#define SYS_IPRST1_PWM0RST_Msk           (0x1ul << SYS_IPRST1_PWM0RST_Pos)                 /*!< SYS IPRST1: PWM0RST Mask               */

#define SYS_IPRST1_ACMPRST_Pos           (22)                                              /*!< SYS IPRST1: ACMPRST Position           */
#define SYS_IPRST1_ACMPRST_Msk           (0x1ul << SYS_IPRST1_ACMPRST_Pos)                 /*!< SYS IPRST1: ACMPRST Mask               */

#define SYS_IPRST1_EADCRST_Pos           (28)                                              /*!< SYS IPRST1: EADCRST Position           */
#define SYS_IPRST1_EADCRST_Msk           (0x1ul << SYS_IPRST1_EADCRST_Pos)                 /*!< SYS IPRST1: EADCRST Mask               */

#define SYS_IPRST1_I2S0RST_Pos           (29)                                              /*!< SYS IPRST1: I2S0RST Position           */
#define SYS_IPRST1_I2S0RST_Msk           (0x1ul << SYS_IPRST1_I2S0RST_Pos)                 /*!< SYS IPRST1: I2S0RST Mask               */

#define SYS_IPRST1_ANARST_Pos            (30)                                              /*!< SYS IPRST1: ANARST Position            */
#define SYS_IPRST1_ANARST_Msk            (0x1ul << SYS_IPRST1_ANARST_Pos)                  /*!< SYS IPRST1: ANARST Mask                */

#define SYS_PASMTEN_SMTEN16_Pos          (16)                                              /*!< SYS PASMTEN: SMTEN16 Position          */
#define SYS_PASMTEN_SMTEN16_Msk          (0x1ul << SYS_PASMTEN_SMTEN16_Pos)                /*!< SYS PASMTEN: SMTEN16 Mask              */

#define SYS_PASMTEN_SMTEN17_Pos          (17)                                              /*!< SYS PASMTEN: SMTEN17 Position          */
#define SYS_PASMTEN_SMTEN17_Msk          (0x1ul << SYS_PASMTEN_SMTEN17_Pos)                /*!< SYS PASMTEN: SMTEN17 Mask              */

#define SYS_PASMTEN_SMTEN18_Pos          (18)                                              /*!< SYS PASMTEN: SMTEN18 Position          */
#define SYS_PASMTEN_SMTEN18_Msk          (0x1ul << SYS_PASMTEN_SMTEN18_Pos)                /*!< SYS PASMTEN: SMTEN18 Mask              */

#define SYS_PASMTEN_SMTEN19_Pos          (19)                                              /*!< SYS PASMTEN: SMTEN19 Position          */
#define SYS_PASMTEN_SMTEN19_Msk          (0x1ul << SYS_PASMTEN_SMTEN19_Pos)                /*!< SYS PASMTEN: SMTEN19 Mask              */

#define SYS_PASMTEN_SMTEN20_Pos          (20)                                              /*!< SYS PASMTEN: SMTEN20 Position          */
#define SYS_PASMTEN_SMTEN20_Msk          (0x1ul << SYS_PASMTEN_SMTEN20_Pos)                /*!< SYS PASMTEN: SMTEN20 Mask              */

#define SYS_PASMTEN_SMTEN21_Pos          (21)                                              /*!< SYS PASMTEN: SMTEN21 Position          */
#define SYS_PASMTEN_SMTEN21_Msk          (0x1ul << SYS_PASMTEN_SMTEN21_Pos)                /*!< SYS PASMTEN: SMTEN21 Mask              */

#define SYS_PASMTEN_SMTEN22_Pos          (22)                                              /*!< SYS PASMTEN: SMTEN22 Position          */
#define SYS_PASMTEN_SMTEN22_Msk          (0x1ul << SYS_PASMTEN_SMTEN22_Pos)                /*!< SYS PASMTEN: SMTEN22 Mask              */

#define SYS_PASMTEN_SMTEN23_Pos          (23)                                              /*!< SYS PASMTEN: SMTEN23 Position          */
#define SYS_PASMTEN_SMTEN23_Msk          (0x1ul << SYS_PASMTEN_SMTEN23_Pos)                /*!< SYS PASMTEN: SMTEN23 Mask              */

#define SYS_PASMTEN_SMTEN24_Pos          (24)                                              /*!< SYS PASMTEN: SMTEN24 Position          */
#define SYS_PASMTEN_SMTEN24_Msk          (0x1ul << SYS_PASMTEN_SMTEN24_Pos)                /*!< SYS PASMTEN: SMTEN24 Mask              */

#define SYS_PASMTEN_SMTEN25_Pos          (25)                                              /*!< SYS PASMTEN: SMTEN25 Position          */
#define SYS_PASMTEN_SMTEN25_Msk          (0x1ul << SYS_PASMTEN_SMTEN25_Pos)                /*!< SYS PASMTEN: SMTEN25 Mask              */

#define SYS_PASMTEN_SMTEN26_Pos          (26)                                              /*!< SYS PASMTEN: SMTEN26 Position          */
#define SYS_PASMTEN_SMTEN26_Msk          (0x1ul << SYS_PASMTEN_SMTEN26_Pos)                /*!< SYS PASMTEN: SMTEN26 Mask              */

#define SYS_PASMTEN_SMTEN27_Pos          (27)                                              /*!< SYS PASMTEN: SMTEN27 Position          */
#define SYS_PASMTEN_SMTEN27_Msk          (0x1ul << SYS_PASMTEN_SMTEN27_Pos)                /*!< SYS PASMTEN: SMTEN27 Mask              */

#define SYS_PASMTEN_SMTEN28_Pos          (28)                                              /*!< SYS PASMTEN: SMTEN28 Position          */
#define SYS_PASMTEN_SMTEN28_Msk          (0x1ul << SYS_PASMTEN_SMTEN28_Pos)                /*!< SYS PASMTEN: SMTEN28 Mask              */

#define SYS_PASMTEN_SMTEN29_Pos          (29)                                              /*!< SYS PASMTEN: SMTEN29 Position          */
#define SYS_PASMTEN_SMTEN29_Msk          (0x1ul << SYS_PASMTEN_SMTEN29_Pos)                /*!< SYS PASMTEN: SMTEN29 Mask              */

#define SYS_PASMTEN_SMTEN30_Pos          (30)                                              /*!< SYS PASMTEN: SMTEN30 Position          */
#define SYS_PASMTEN_SMTEN30_Msk          (0x1ul << SYS_PASMTEN_SMTEN30_Pos)                /*!< SYS PASMTEN: SMTEN30 Mask              */

#define SYS_PASMTEN_SMTEN31_Pos          (31)                                              /*!< SYS PASMTEN: SMTEN31 Position          */
#define SYS_PASMTEN_SMTEN31_Msk          (0x1ul << SYS_PASMTEN_SMTEN31_Pos)                /*!< SYS PASMTEN: SMTEN31 Mask              */

#define SYS_PBSMTEN_SMTEN16_Pos          (16)                                              /*!< SYS PBSMTEN: SMTEN16 Position          */
#define SYS_PBSMTEN_SMTEN16_Msk          (0x1ul << SYS_PBSMTEN_SMTEN16_Pos)                /*!< SYS PBSMTEN: SMTEN16 Mask              */

#define SYS_PBSMTEN_SMTEN17_Pos          (17)                                              /*!< SYS PBSMTEN: SMTEN17 Position          */
#define SYS_PBSMTEN_SMTEN17_Msk          (0x1ul << SYS_PBSMTEN_SMTEN17_Pos)                /*!< SYS PBSMTEN: SMTEN17 Mask              */

#define SYS_PBSMTEN_SMTEN18_Pos          (18)                                              /*!< SYS PBSMTEN: SMTEN18 Position          */
#define SYS_PBSMTEN_SMTEN18_Msk          (0x1ul << SYS_PBSMTEN_SMTEN18_Pos)                /*!< SYS PBSMTEN: SMTEN18 Mask              */

#define SYS_PBSMTEN_SMTEN19_Pos          (19)                                              /*!< SYS PBSMTEN: SMTEN19 Position          */
#define SYS_PBSMTEN_SMTEN19_Msk          (0x1ul << SYS_PBSMTEN_SMTEN19_Pos)                /*!< SYS PBSMTEN: SMTEN19 Mask              */

#define SYS_PBSMTEN_SMTEN20_Pos          (20)                                              /*!< SYS PBSMTEN: SMTEN20 Position          */
#define SYS_PBSMTEN_SMTEN20_Msk          (0x1ul << SYS_PBSMTEN_SMTEN20_Pos)                /*!< SYS PBSMTEN: SMTEN20 Mask              */

#define SYS_PBSMTEN_SMTEN21_Pos          (21)                                              /*!< SYS PBSMTEN: SMTEN21 Position          */
#define SYS_PBSMTEN_SMTEN21_Msk          (0x1ul << SYS_PBSMTEN_SMTEN21_Pos)                /*!< SYS PBSMTEN: SMTEN21 Mask              */

#define SYS_PBSMTEN_SMTEN22_Pos          (22)                                              /*!< SYS PBSMTEN: SMTEN22 Position          */
#define SYS_PBSMTEN_SMTEN22_Msk          (0x1ul << SYS_PBSMTEN_SMTEN22_Pos)                /*!< SYS PBSMTEN: SMTEN22 Mask              */

#define SYS_PBSMTEN_SMTEN23_Pos          (23)                                              /*!< SYS PBSMTEN: SMTEN23 Position          */
#define SYS_PBSMTEN_SMTEN23_Msk          (0x1ul << SYS_PBSMTEN_SMTEN23_Pos)                /*!< SYS PBSMTEN: SMTEN23 Mask              */

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

#define SYS_GPB_MFP_PB6MFP_Pos           (12)                                              /*!< SYS GPB_MFP: PB6MFP Position           */
#define SYS_GPB_MFP_PB6MFP_Msk           (0x3ul << SYS_GPB_MFP_PB6MFP_Pos)                 /*!< SYS GPB_MFP: PB6MFP Mask               */

#define SYS_GPB_MFP_PB7MFP_Pos           (14)                                              /*!< SYS GPB_MFP: PB7MFP Position           */
#define SYS_GPB_MFP_PB7MFP_Msk           (0x3ul << SYS_GPB_MFP_PB7MFP_Pos)                 /*!< SYS GPB_MFP: PB7MFP Mask               */

#define SYS_WKCTL_WKDIN_Pos              (0)                                               /*!< SYS WKCTL: WKDIN Position              */
#define SYS_WKCTL_WKDIN_Msk              (0x1ul << SYS_WKCTL_WKDIN_Pos)                    /*!< SYS WKCTL: WKDIN Mask                  */

#define SYS_WKCTL_WKPUEN_Pos             (1)                                               /*!< SYS WKCTL: WKPUEN Position             */
#define SYS_WKCTL_WKPUEN_Msk             (0x1ul << SYS_WKCTL_WKPUEN_Pos)                   /*!< SYS WKCTL: WKPUEN Mask                 */

#define SYS_WKCTL_WKOENB_Pos             (2)                                               /*!< SYS WKCTL: WKOENB Position             */
#define SYS_WKCTL_WKOENB_Msk             (0x1ul << SYS_WKCTL_WKOENB_Pos)                   /*!< SYS WKCTL: WKOENB Mask                 */

#define SYS_WKCTL_WKDOUT_Pos             (3)                                               /*!< SYS WKCTL: WKDOUT Position             */
#define SYS_WKCTL_WKDOUT_Msk             (0x1ul << SYS_WKCTL_WKDOUT_Pos)                   /*!< SYS WKCTL: WKDOUT Mask                 */

#define SYS_REGLCTL_REGLCTL_Pos          (0)                                               /*!< SYS REGLCTL: REGLCTL Position          */
#define SYS_REGLCTL_REGLCTL_Msk          (0x1ul << SYS_REGLCTL_REGLCTL_Pos)                /*!< SYS REGLCTL: REGLCTL Mask              */

#define SYS_IRCTCTL_FREQ0SEL_Pos         (0)                                               /*!< SYS IRCTCTL: FREQ0SEL Position         */
#define SYS_IRCTCTL_FREQ0SEL_Msk         (0xfful << SYS_IRCTCTL_FREQ0SEL_Pos)              /*!< SYS IRCTCTL: FREQ0SEL Mask             */

#define SYS_IRCTCTL_RGE0SEL_Pos          (8)                                               /*!< SYS IRCTCTL: RGE0SEL Position          */
#define SYS_IRCTCTL_RGE0SEL_Msk          (0x1ul << SYS_IRCTCTL_RGE0SEL_Pos)                /*!< SYS IRCTCTL: RGE0SEL Mask              */

#define SYS_IRCTCTL_FREQ1SEL_Pos         (16)                                              /*!< SYS IRCTCTL: FREQ1SEL Position         */
#define SYS_IRCTCTL_FREQ1SEL_Msk         (0xfful << SYS_IRCTCTL_FREQ1SEL_Pos)              /*!< SYS IRCTCTL: FREQ1SEL Mask             */

#define SYS_IRCTCTL_RGE1SEL_Pos          (24)                                              /*!< SYS IRCTCTL: RGE1SEL Position          */
#define SYS_IRCTCTL_RGE1SEL_Msk          (0x1ul << SYS_IRCTCTL_RGE1SEL_Pos)                /*!< SYS IRCTCTL: RGE1SEL Mask              */

/**@}*/ /* SYS_CONST */
/**@}*/ /* end of SYS register group */


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
     * |[0:7]   |PSC       |Pre-scale Counter
     * |        |          |Clock input is divided by PSC+1 before it is fed to the counter. If PSC = 0, then there is no scaling.
     * |[16]    |CNTDATEN  |Data Latch Enable
     * |        |          |When CNTDATEN is set, TIMERx_CNT (Timer Data Register) will be updated continuously with the 24-bit up-counter value as the timer is counting.
     * |        |          |1 = Timer Data Register update enable.
     * |        |          |0 = Timer Data Register update disable. 
     * |[25]    |ACTSTS    |Timer Active Status Bit (Read only)
     * |        |          |This bit indicates the counter status of timer.
     * |        |          |0 = Timer is not active.
     * |        |          |1 = Timer is active.
     * |[26]    |RSTCNT    |Counter Reset Bit
     * |        |          |Set this bit will reset the timer counter, prescale and also force CNTEN to 0.
     * |        |          |0 = No effect.
     * |        |          |1 = Reset Timer's prescale counter, internal 24-bit up-counter and CNTEN bit.
     * |[27:28] |OPMODE    |Timer Operating Mode
     * |        |          |0 = The timer is operating in the one-shot mode.
     * |        |          |The associated interrupt signal is generated once (if INTEN is enabled) and CNTEN is automatically cleared by hardware.
     * |        |          |1 = The timer is operating in the periodic mode.
     * |        |          |The associated interrupt signal is generated periodically (if INTEN is enabled).
     * |        |          |2 = Reserved.
     * |        |          |3 = The timer is operating in continuous counting mode.
     * |        |          |The associated interrupt signal is generated when CNT = CMPDAT (if INTEN is enabled); however, the 24-bit up-counter counts continuously without reset.
     * |[29]    |INTEN     |Interrupt Enable Bit
     * |        |          |0 = Disable TIMER Interrupt.
     * |        |          |1 = Enable TIMER Interrupt.
     * |        |          |If timer interrupt is enabled, the timer asserts its interrupt signal when the count is equal to TIMERx_CMP.
     * |[30]    |CNTEN     |Counter Enable Bit
     * |        |          |0 = Stops/Suspends counting
     * |        |          |1 = Starts counting
     * |        |          |Note1: Setting CNTEN = 1 enables 24-bit counter. It continues count from last value.
     * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (OPMODE = 00b) when the timer interrupt is generated (INTEN = 1b).
 */
    __IO uint32_t CTL;                   

    /**
     * CMP
     * ===================================================================================================
     * Offset: 0x04  Timer Compare Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:24]  |CMPDAT    |Timer Comparison Value
     * |        |          |CMPDAT is a 24-bit comparison register.
     * |        |          |When the 24-bit up-counter is enabled and its value is equal to CMPDAT value, a Timer Interrupt is requested if the timer interrupt is enabled with TIMERx_CTL.INTEN = 1.
     * |        |          |The CMPDAT value defines the timer cycle time.
     * |        |          |Time out period = (Period of timer clock input) * (8-bit PSC + 1) * (24-bit CMPDAT)
     * |        |          |NOTE1: Never set CMPDAT to 0x000 or 0x001. Timer will not function correctly.
     * |        |          |NOTE2: Regardless of CNTEN state, whenever a new value is written to this register, TIMER will restart counting using this new value and abort previous count.
 */
    __IO uint32_t CMP;                   

    /**
     * INTSTS
     * ===================================================================================================
     * Offset: 0x08  Timer Interrupt Status Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0]     |TIF       |Timer Interrupt Flag
     * |        |          |This bit indicates the interrupt status of Timer.
     * |        |          |TIF bit is set by hardware when the 24-bit counter matches the timer comparison value (CMPDAT).
     * |        |          |It is cleared by writing 1.
 */
    __IO uint32_t INTSTS;                

    /**
     * CNT
     * ===================================================================================================
     * Offset: 0x0C  Timer Data Register
     * ---------------------------------------------------------------------------------------------------
     * |Bits    |Field     |Descriptions
     * | :----: | :----:   | :---- |
     * |[0:23]  |CNT       |Timer Data Register
     * |        |          |When TIMERx_CTL.CNTDATEN is set to 1, the internal 24-bit timer up-counter value will be latched into CNT.
     * |        |          |User can read this register for the up-counter value.
 */
    __IO uint32_t CNT;                   

} TMR_T;

/**
    @addtogroup TMR_CONST TMR Bit Field Definition
    Constant Definitions for TMR Controller
@{ */

#define TMR_CTL_PSC_Pos                  (0)                                               /*!< TMR CTL: PSC Position                  */
#define TMR_CTL_PSC_Msk                  (0xfful << TMR_CTL_PSC_Pos)                       /*!< TMR CTL: PSC Mask                      */

#define TMR_CTL_CNTDATEN_Pos             (16)                                              /*!< TMR CTL: CNTDATEN Position             */
#define TMR_CTL_CNTDATEN_Msk             (0x1ul << TMR_CTL_CNTDATEN_Pos)                   /*!< TMR CTL: CNTDATEN Mask                 */

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
#define TMR_CMP_CMPDAT_Msk               (0x1fffffful << TMR_CMP_CMPDAT_Pos)               /*!< TMR CMP: CMPDAT Mask                   */

#define TMR_INTSTS_TIF_Pos               (0)                                               /*!< TMR INTSTS: TIF Position               */
#define TMR_INTSTS_TIF_Msk               (0x1ul << TMR_INTSTS_TIF_Pos)                     /*!< TMR INTSTS: TIF Mask                   */

#define TMR_CNT_CNT_Pos                  (0)                                               /*!< TMR CNT: CNT Position                  */
#define TMR_CNT_CNT_Msk                  (0xfffffful << TMR_CNT_CNT_Pos)                   /*!< TMR CNT: CNT Mask                      */

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
     * |[6]     |INTEN     |Watchdog Timer Interrupt Enable
     * |        |          |0 = Disable the Watchdog timer interrupt
     * |        |          |1 = Enable the Watchdog timer interrupt
     * |[7]     |WDTEN     |Watchdog Timer Enable
     * |        |          |0 = Disable the Watchdog timer (This action will reset the internal counter)
     * |        |          |1 = Enable the Watchdog timer
     * |[8:10]  |TOUTSEL   |Watchdog Timer Interval Select
     * |        |          |These three bits select the timeout interval for the Watchdog timer, a watchdog reset will occur 1024 clock cycles later if WDG not reset.
     * |        |          |The timeout is given by:.
     * |        |          |Interrupt Timeout = 2^(2xWTIS+4) x WDT_CLK
     * |        |          |Reset Timeout = (2^(2xWTIS+4) +1024) x WDT_CLK
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

#define WDT_CTL_INTEN_Pos                (6)                                               /*!< WDT CTL: INTEN Position                */
#define WDT_CTL_INTEN_Msk                (0x1ul << WDT_CTL_INTEN_Pos)                      /*!< WDT CTL: INTEN Mask                    */

#define WDT_CTL_WDTEN_Pos                (7)                                               /*!< WDT CTL: WDTEN Position                */
#define WDT_CTL_WDTEN_Msk                (0x1ul << WDT_CTL_WDTEN_Pos)                      /*!< WDT CTL: WDTEN Mask                    */

#define WDT_CTL_TOUTSEL_Pos              (8)                                               /*!< WDT CTL: TOUTSEL Position              */
#define WDT_CTL_TOUTSEL_Msk              (0x7ul << WDT_CTL_TOUTSEL_Pos)                    /*!< WDT CTL: TOUTSEL Mask                  */

/**@}*/ /* WDT_CONST */
/**@}*/ /* end of WDT register group */

/**@}*/ /* end of SBRAM register group */
typedef struct
{
    __IO uint32_t	D[64];
} SBRAM_T;
/**@}*/ /* end of REGISTER group */


/**@}*/ /* end of REGISTER group */

/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/** @addtogroup ISD9100_PERIPHERAL_MEM_MAP ISD9100 Peripheral Memory Map
  Memory Mapped Structure for ISD9100 Series Peripheral
  @{
 */
/* Peripheral and SRAM base address */
#define FLASH_BASE          ((     uint32_t)0x00000000)
#define SRAM_BASE           ((     uint32_t)0x20000000)
#define AHB_BASE            ((     uint32_t)0x50000000)
#define APB1_BASE           ((     uint32_t)0x40000000)

/* Peripheral memory map */

#define WDT_BASE       	     (APB1_BASE      + 0x04000)
#define RTC_BASE             (APB1_BASE      + 0x08000)
#define TIMER0_BASE          (APB1_BASE      + 0x10000)
#define TIMER1_BASE          (APB1_BASE      + 0x10020)
#define I2C0_BASE            (APB1_BASE      + 0x20000)
#define SPI0_BASE            (APB1_BASE      + 0x30000)
#define PWM0_BASE            (APB1_BASE      + 0x40000)
#define UART0_BASE           (APB1_BASE      + 0x50000)
#define DPWM_BASE            (APB1_BASE      + 0x70000)
#define ANA_BASE             (APB1_BASE      + 0x80000)
#define BODTALM_BASE         (APB1_BASE      + 0x84000)
#define CRC_BASE             (APB1_BASE      + 0x90000)
#define I2S0_BASE            (APB1_BASE      + 0xA0000)
#define BIQ_BASE             (APB1_BASE      + 0xB0000)
#define ALC_BASE             (APB1_BASE      + 0xB0048)
#define ACMP_BASE            (APB1_BASE      + 0xD0000)
#define ADC_BASE             (APB1_BASE      + 0xE0000)
#define SBRAM_BASE           (APB1_BASE      + 0xF0000)

#define SYS_BASE             (AHB_BASE       + 0x00000)
#define CLK_BASE             (AHB_BASE       + 0x00200)
#define INT_BASE             (AHB_BASE       + 0x00300)
#define GPIO_BASE            (AHB_BASE       + 0x04000)
#define GPIOA_BASE           (GPIO_BASE               )
#define GPIOB_BASE           (GPIO_BASE      + 0x00040)
#define GPIO_DBNCECON_BASE   (GPIO_BASE      + 0x00180)

#define PDMA0_BASE           (AHB_BASE       + 0x08000)                 /*!< PDMA0 Base Address                               */
#define PDMA1_BASE           (AHB_BASE       + 0x08100)                 /*!< PDMA1 Base Address                               */
#define PDMA2_BASE           (AHB_BASE       + 0x08200)                 /*!< PDMA2 Base Address                               */
#define PDMA3_BASE           (AHB_BASE       + 0x08300)                 /*!< PDMA3 Base Address                               */

#define PDMA_GCR_BASE        (AHB_BASE       + 0x08F00)                 /*!< PDMA Grobal Base Address                         */



#define FMC_BASE             (AHB_BASE       + 0x0C000)

/*@}*/ /* end of group ISD9100_PERIPHERAL_MEM_MAP */
/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
/** @addtogroup ISD9100_PeripheralDecl ISD9100 Peripheral Declaration
    @{
*/
#define PA                  ((GPIO_T *) GPIOA_BASE)
#define PB                  ((GPIO_T *) GPIOB_BASE)
#define GPIO                ((GPIO_DB_T *) GPIO_DBNCECON_BASE)

#define UART0               ((UART_T *) UART0_BASE)
#define DPWM                ((DPWM_T *) DPWM_BASE)

#define TIMER0              ((TMR_T *) TIMER0_BASE)
#define TIMER1              ((TMR_T *) TIMER1_BASE)

#define WDT                 ((WDT_T *) WDT_BASE)

#define SPI0                ((SPI_T *) SPI0_BASE)

#define I2C0                ((I2C_T *) I2C0_BASE)

#define I2S0                ((I2S_T *) I2S0_BASE)

#define RTC                 ((RTC_T *) RTC_BASE)

#define ADC                 ((ADC_T*) ADC_BASE)

#define ACMP                ((ACMP_T *) ACMP_BASE)
#define ANA					((ANA_T *) ANA_BASE)
#define BODTALM             ((BODTALM_T *) BODTALM_BASE)

#define BIQ                 ((BIQ_T *) BIQ_BASE)
#define ALC                 ((ALC_T *) ALC_BASE)
#define SBRAM				((SBRAM_T *) SBRAM_BASE)

#define CLK                 ((CLK_T *) CLK_BASE)
#define SYS                 ((SYS_T *) SYS_BASE)

#define FMC                 ((FMC_T *) FMC_BASE)

#define PDMA0               ((PDMA_T *) PDMA0_BASE)                     /*!< PDMA0 Configuration Struct                       */
#define PDMA1               ((PDMA_T *) PDMA1_BASE)                     /*!< PDMA1 Configuration Struct                       */
#define PDMA2               ((PDMA_T *) PDMA2_BASE)                     /*!< PDMA2 Configuration Struct                       */
#define PDMA3               ((PDMA_T *) PDMA3_BASE)                     /*!< PDMA3 Configuration Struct                       */

#define PDMA_GCR            ((PDMA_GCR_T *) PDMA_GCR_BASE)              /*!< PDMA Global Configuration Struct                 */

#define CRC                 ((CRC_T *) CRC_BASE)                        /*!< CRC Configuration Struct                         */

#define PWM0                ((PWM_T *) PWM0_BASE)

/*@}*/ /* end of group ISD9100_PeripheralDecl */

#define UNLOCKREG(x)        do{*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x59;*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x16;*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x88;}while(*((__IO uint32_t *)(SYS_BASE + 0x100))==0)
#define LOCKREG(x)          *((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x00

#define REGCOPY(dest, src)  *((uint32_t *)&(dest)) = *((uint32_t *)&(src))
#define CLEAR(dest)         *((uint32_t *)&(dest)) = 0

//=============================================================================
/** @addtogroup ISD9100_IO_ROUTINE ISD9100 I/O routines
  The Declaration of ISD9100 I/O routines
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

/*@}*/ /* end of group ISD9100_IO_ROUTINE */




/** @addtogroup ISD9100_legacy_Constants ISD9100 Legacy Constants
  ISD9100 Legacy Constants
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

#define _GET_BYTE0(u32Param)    ((u32Param & BYTE0_Msk)      )  /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define _GET_BYTE1(u32Param)    ((u32Param & BYTE1_Msk) >>  8)  /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define _GET_BYTE2(u32Param)    ((u32Param & BYTE2_Msk) >> 16)  /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define _GET_BYTE3(u32Param)    ((u32Param & BYTE3_Msk) >> 24)  /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/*@}*/ /* end of group ISD9100_legacy_Constants */

/*@}*/ /* end of group ISD9100_Definitions */

#define __ISD9100_SERIES__  (0x91000000)
#define __CHIP_SERIES__     __ISD9100_SERIES__

/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "acmp.h"
#include "adc.h"
#include "biq.h"
#include "bod.h"
#include "capsense.h"
#include "clk.h"
#include "crc.h"
#include "dpwm.h"
#include "fmc.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "osc.h"
#include "pdma.h"
#include "pwm.h"
#include "rtc.h"
#include "spi.h"
#include "sys.h"
#include "talarm.h"
#include "timer.h"
#include "uart.h"
#include "wdt.h"


#endif	// __ISD9100_H__

