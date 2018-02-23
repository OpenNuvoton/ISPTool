/**************************************************************************//**
 * @file     I91200.h
 * @version  V1.0
 * $Revision: 1 $
 * $Date: 16/11/21 11:06a $
 * @brief    i91200 Series Peripheral Access Layer Header File
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#ifndef __I91200_H__
#define __I91200_H__

/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
*/

/** @addtogroup I91200_CMSIS Device Definitions for CMSIS
  I91200 Interrupt Number Definition and Configurations for CMSIS
  @{
*/

/**
 * @details  Interrupt Number Definition. The maximum of 32 Specific Interrupts are possible.
 */
 
typedef	enum IRQn
{
    /* Cortex-M0 Processor Exceptions Numbers                                                     */
    Reset_IRQn              = -15,      /*!< 1 Reset Vector, invoked on Power up and warm reset   */
    NonMaskableInt_IRQn		= -14,		/*!< 2 Non Maskable	Interrupt							  */
    HardFault_IRQn			= -13,		/*!< 3 Cortex-M0 Hard Fault	Interrupt					  */
    SVCall_IRQn				= -5,		/*!< 11	Cortex-M0 SV Call Interrupt						  */
    PendSV_IRQn				= -2,		/*!< 14	Cortex-M0 Pend SV Interrupt						  */
    SysTick_IRQn			= -1,		/*!< 15	Cortex-M0 System Tick Interrupt					  */

    /* I91200 specific Interrupt Numbers */
    BOD_IRQn                = 0,
    WDT_IRQn                = 1,
    EINT0_IRQn              = 2,
    EINT1_IRQn              = 3,
    GPAB_IRQn               = 4,
    ALC_IRQn                = 5,
    PWM0_IRQn               = 6,
    TMR0_IRQn               = 8,
    TMR1_IRQn               = 9, 
	UART1_IRQn              = 11,	
    UART0_IRQn              = 12,    
    SPI1_IRQn               = 13,
	SPI0_IRQn               = 14,
    DPWM_IRQn               = 15,
    I2C0_IRQn               = 18,
    CMP_IRQn             	= 21,
	SARADC_IRQn				= 25,
    PDMA_IRQn               = 26,
    I2S0_IRQn               = 27,
    CAPS_IRQn               = 28,
    SDADC_IRQn              = 29,
    RTC_IRQn                = 31,
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
/*@}*/ /* end of group I91200_CMSIS */

#include "core_cm0.h"                   /* Cortex-M0 processor and core peripherals               */
#include "system_I91200.h"              /* I91200 System include file                           */

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
// GNU C compiler detected
#elif ( defined(__GNUC__) )
    #define __inline      inline
    #define __isb(n)      __ISB()
    #define __wfi         __WFI
    #define __weak	      __attribute__((weak))
    #define __align(n)    __attribute__ ((aligned (n))) 
#endif

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/

/** @addtogroup REGISTER Control Register

  @{

*/


/*---------------------- Automatic Level Control -------------------------*/
/**
    @addtogroup ALC Automatic Level Control(ALC)
    Memory Mapped Structure for ALC Controller
@{ */
 
typedef struct
{

/**
 * @var ALC_T::CTL
 * Offset: 0x00  ALC Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |NGTHBST   |Noise Gate Threshold
 * |        |          |000 --- -39dB
 * |        |          |001--- -45dB
 * |        |          |010---- -51dB
 * |        |          |011 --- -57dB
 * |        |          |100 --- -63dB
 * |        |          |101 --- -69dB
 * |        |          |110 --- -75dB
 * |        |          |111 --- -81dB
 * |[3]     |NGEN      |Noise Gate Enable
 * |        |          |0 = Noise gate disabled.
 * |        |          |1 = Noise gate enabled.
 * |[7:4]   |ATKSEL    |ALC Attack Time
 * |        |          |(Value: 0~10)
 * |        |          |When MODESEL = 0, Range: 125us to 128ms.
 * |        |          |When MODESEL = 1, Range: 31us to 32ms (time doubles with every step).
 * |[11:8]  |DECAYSEL  |ALC Decay Time
 * |        |          |(Value: 0~10)
 * |        |          |When MODESEL = 0, Range: 500us to 512ms.
 * |        |          |When MODESEL = 1,Range: 125us to 128ms (Both ALC time doubles with every step).
 * |[12]    |MODESEL   |ALC Mode
 * |        |          |0 = ALC normal operation mode.
 * |        |          |1 = ALC limiter mode.
 * |[16:13] |TARGETLV  |ALC Target Level
 * |        |          |0 = -28.5 dB.
 * |        |          |1 = -27 dB.
 * |        |          |2 = -25.5 dB.
 * |        |          |3 = -24 dB.
 * |        |          |4 = -22.5 dB.
 * |        |          |5 = -21 dB.
 * |        |          |6 = -19.5 dB.
 * |        |          |7 = -18 dB.
 * |        |          |8 = -16.5 dB.
 * |        |          |9 = -15 dB.
 * |        |          |10 = -13.5 dB.
 * |        |          |11 = -12 dB.
 * |        |          |12 = -10.5 dB.
 * |        |          |13 = -9 dB.
 * |        |          |14 = -7.5 dB.
 * |        |          |15 = -6 dB.
 * |[20:17] |HOLDTIME  |ALC Hold Time
 * |        |          |(Value: 0~10). Hold Time = (2^HOLDTIME) ms.
 * |[21]    |ALCRANGESEL|ALC Target range selection
 * |        |          |0 = ALC target range -28.5~ -6dB
 * |        |          |1 = ALC target range -22.5 ~-1.5dB 
 * |[24:22] |MINGAIN   |ALC Minimum Gain
 * |        |          |0 = -12 dB.
 * |        |          |1 = -6 dB.
 * |        |          |2 = 0 dB.
 * |        |          |3 = 6 dB.
 * |        |          |4 = 12 dB.
 * |        |          |5 = 18 dB.
 * |        |          |6 = 24 dB.
 * |        |          |7 = 30 dB.
 * |[27:25] |MAXGAIN   |ALC Maximum Gain
 * |        |          |0 = -6.75 dB.
 * |        |          |1 = -0.75 dB.
 * |        |          |2 = +5.25 dB.
 * |        |          |3 = +11.25 dB.
 * |        |          |4 = +17.25 dB.
 * |        |          |5 = +23.25 dB.
 * |        |          |6 = +29.25 dB.
 * |        |          |7 = +35.25 dB.
 * |[28]    |ALCEN     |ALC Select
 * |        |          |0 = ALC disabled (default).
 * |        |          |1 = ALC enabled.
 * |[29]    |NGPKSEL   |ALC Noise Gate Peak Detector Select
 * |        |          |0 = use peak-to-peak value for noise gate threshold determination (default).
 * |        |          |1 = use absolute peak value for noise gate threshold determination.
 * |[30]    |PKSEL     |ALC Gain Peak Detector Select
 * |        |          |0 = use absolute peak value for ALC training (default).
 * |        |          |1 = use peak-to-peak value for ALC training.
 * |[31]    |PKLIMEN   |ALC Peak Limiter Enable
 * |        |          |Default is "0",
 * |        |          |Please set as "1"
 * @var ALC_T::GAIN
 * Offset: 0x04  ALC GAIN Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |INITGAIN  |ALC Initial Gain
 * |        |          |Set ALC initial gain.
 * |        |          |Selects the PGA gain setting from -12dB to 35.25dB in 0.75dB step size
 * |        |          |0x00 is lowest gain setting at -12dB and 0x3F is largest gain at 35.25dB
 * |[6]     |ZCEN      |ALC Zero Crossing Enable
 * |        |          |0 = zero crossing disabled.
 * |        |          |1 = zero crossing enabled when update gain.
 * |[7]     |INITGAINEN|ALC Update Initial Gain
 * |        |          |0 = ALC PGA GAIN load automatic calculating gain.
 * |        |          |1 = ALC PGA GAIN load ALCINIT_GAIN
 * |[31:16] |PKLIMIT   |ALC Peak Limiter Threshold
 * |        |          |Full scale - 0x7fff
 * |        |          |Default value is 0x6fdc - 87.5% of full scale
 * @var ALC_T::STS
 * Offset: 0x08  ALC Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CLIPF     |Clipping Flag
 * |        |          |Asserted when signal level is detected to be above 87.5% of full scale
 * |[1]     |NOISEF    |Noise Flag
 * |        |          |Asserted when signal level is detected to be below NGTHBST
 * |[10:2]  |P2PVAL    |Peak-to-peak Value
 * |        |          |9 MSBs of measured peak-to-peak value
 * |[19:11] |PEAKVAL   |Peak Value
 * |        |          |9 MSBs of measured absolute peak value
 * |[25:20] |ALCGAIN   |ALC GAIN
 * |        |          |Current ADC gain setting
 * @var ALC_T::INTCTL
 * Offset: 0x0C  ALC Interrupt Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PLMTIE    |ALC Peak limiting Interrupt enable control
 * |[1]     |NGIE      |ALC noise gating interrupt enable control
 * |[2]     |GINCIE    |GAIN Increase interrupt enable control
 * |[3]     |GDECIE    |GAIN Decrease interrupt enable control
 * |[4]     |GMAXIE    |GAIN more than maximum GAIN interrupt enable control
 * |[5]     |GMINIE    |GAIN less than minimum GAIN interrupt enable control
 * |[8]     |PLMTIF    |ALC Peak limiting Interrupt flag
 * |[9]     |NGIF      |ALC noise gating interrupt flag
 * |[10]    |GINCIF    |GAIN Increase interrupt flag
 * |[11]    |GDECIF    |GAIN Decrease interrupt flag
 * |[12]    |GMAXIF    |GAIN more than maximum GAIN interrupt flag.
 * |[13]    |GMINIF    |GAIN less than minimum GAIN interrupt flag.
 * |[31]    |ALCIF     |ALC Interrupt flag
 * |        |          |This interrupt flag asserts whenever the interrupt is enabled and the PGA gain is updated, either through an ALC change with the ALC enabled or through a PGA gain write with the ALC disabled.
 * |        |          |Write a 1 to this register to clear.
 */
    __IO uint32_t CTL;                   /*!< [0x0000] ALC Control Register                                             */
    __IO uint32_t GAIN;                  /*!< [0x0004] ALC GAIN Control Register                                        */
    __IO uint32_t STS;                   /*!< [0x0008] ALC Status Register                                              */
    __IO uint32_t INTCTL;                /*!< [0x000c] ALC Interrupt Control Register                                   */

} ALC_T;

/**
    @addtogroup ALC_CONST ALC Bit Field Definition
    Constant Definitions for ALC Controller
@{ */

#define ALC_CTL_NGTHBST_Pos              (0)                                               /*!< ALC_T::CTL: NGTHBST Position           */
#define ALC_CTL_NGTHBST_Msk              (0x7ul << ALC_CTL_NGTHBST_Pos)                    /*!< ALC_T::CTL: NGTHBST Mask               */

#define ALC_CTL_NGEN_Pos                 (3)                                               /*!< ALC_T::CTL: NGEN Position              */
#define ALC_CTL_NGEN_Msk                 (0x1ul << ALC_CTL_NGEN_Pos)                       /*!< ALC_T::CTL: NGEN Mask                  */

#define ALC_CTL_ATKSEL_Pos               (4)                                               /*!< ALC_T::CTL: ATKSEL Position            */
#define ALC_CTL_ATKSEL_Msk               (0xful << ALC_CTL_ATKSEL_Pos)                     /*!< ALC_T::CTL: ATKSEL Mask                */

#define ALC_CTL_DECAYSEL_Pos             (8)                                               /*!< ALC_T::CTL: DECAYSEL Position          */
#define ALC_CTL_DECAYSEL_Msk             (0xful << ALC_CTL_DECAYSEL_Pos)                   /*!< ALC_T::CTL: DECAYSEL Mask              */

#define ALC_CTL_MODESEL_Pos              (12)                                              /*!< ALC_T::CTL: MODESEL Position           */
#define ALC_CTL_MODESEL_Msk              (0x1ul << ALC_CTL_MODESEL_Pos)                    /*!< ALC_T::CTL: MODESEL Mask               */

#define ALC_CTL_TARGETLV_Pos             (13)                                              /*!< ALC_T::CTL: TARGETLV Position          */
#define ALC_CTL_TARGETLV_Msk             (0xful << ALC_CTL_TARGETLV_Pos)                   /*!< ALC_T::CTL: TARGETLV Mask              */

#define ALC_CTL_HOLDTIME_Pos             (17)                                              /*!< ALC_T::CTL: HOLDTIME Position          */
#define ALC_CTL_HOLDTIME_Msk             (0xful << ALC_CTL_HOLDTIME_Pos)                   /*!< ALC_T::CTL: HOLDTIME Mask              */

#define ALC_CTL_ALCRANGESEL_Pos          (21)                                              /*!< ALC_T::CTL: ALCRANGESEL Position       */
#define ALC_CTL_ALCRANGESEL_Msk          (0x1ul << ALC_CTL_ALCRANGESEL_Pos)                /*!< ALC_T::CTL: ALCRANGESEL Mask           */

#define ALC_CTL_MINGAIN_Pos              (22)                                              /*!< ALC_T::CTL: MINGAIN Position           */
#define ALC_CTL_MINGAIN_Msk              (0x7ul << ALC_CTL_MINGAIN_Pos)                    /*!< ALC_T::CTL: MINGAIN Mask               */

#define ALC_CTL_MAXGAIN_Pos              (25)                                              /*!< ALC_T::CTL: MAXGAIN Position           */
#define ALC_CTL_MAXGAIN_Msk              (0x7ul << ALC_CTL_MAXGAIN_Pos)                    /*!< ALC_T::CTL: MAXGAIN Mask               */

#define ALC_CTL_ALCEN_Pos                (28)                                              /*!< ALC_T::CTL: ALCEN Position             */
#define ALC_CTL_ALCEN_Msk                (0x1ul << ALC_CTL_ALCEN_Pos)                      /*!< ALC_T::CTL: ALCEN Mask                 */

#define ALC_CTL_NGPKSEL_Pos              (29)                                              /*!< ALC_T::CTL: NGPKSEL Position           */
#define ALC_CTL_NGPKSEL_Msk              (0x1ul << ALC_CTL_NGPKSEL_Pos)                    /*!< ALC_T::CTL: NGPKSEL Mask               */

#define ALC_CTL_PKSEL_Pos                (30)                                              /*!< ALC_T::CTL: PKSEL Position             */
#define ALC_CTL_PKSEL_Msk                (0x1ul << ALC_CTL_PKSEL_Pos)                      /*!< ALC_T::CTL: PKSEL Mask                 */

#define ALC_CTL_PKLIMEN_Pos              (31)                                              /*!< ALC_T::CTL: PKLIMEN Position           */
#define ALC_CTL_PKLIMEN_Msk              (0x1ul << ALC_CTL_PKLIMEN_Pos)                    /*!< ALC_T::CTL: PKLIMEN Mask               */

#define ALC_GAIN_INITGAIN_Pos            (0)                                               /*!< ALC_T::GAIN: INITGAIN Position         */
#define ALC_GAIN_INITGAIN_Msk            (0x3ful << ALC_GAIN_INITGAIN_Pos)                 /*!< ALC_T::GAIN: INITGAIN Mask             */

#define ALC_GAIN_ZCEN_Pos                (6)                                               /*!< ALC_T::GAIN: ZCEN Position             */
#define ALC_GAIN_ZCEN_Msk                (0x1ul << ALC_GAIN_ZCEN_Pos)                      /*!< ALC_T::GAIN: ZCEN Mask                 */

#define ALC_GAIN_INITGAINEN_Pos          (7)                                               /*!< ALC_T::GAIN: INITGAINEN Position       */
#define ALC_GAIN_INITGAINEN_Msk          (0x1ul << ALC_GAIN_INITGAINEN_Pos)                /*!< ALC_T::GAIN: INITGAINEN Mask           */

#define ALC_GAIN_PKLIMIT_Pos             (16)                                              /*!< ALC_T::GAIN: PKLIMIT Position          */
#define ALC_GAIN_PKLIMIT_Msk             (0xfffful << ALC_GAIN_PKLIMIT_Pos)                /*!< ALC_T::GAIN: PKLIMIT Mask              */

#define ALC_STS_CLIPF_Pos                (0)                                               /*!< ALC_T::STS: CLIPF Position             */
#define ALC_STS_CLIPF_Msk                (0x1ul << ALC_STS_CLIPF_Pos)                      /*!< ALC_T::STS: CLIPF Mask                 */

#define ALC_STS_NOISEF_Pos               (1)                                               /*!< ALC_T::STS: NOISEF Position            */
#define ALC_STS_NOISEF_Msk               (0x1ul << ALC_STS_NOISEF_Pos)                     /*!< ALC_T::STS: NOISEF Mask                */

#define ALC_STS_P2PVAL_Pos               (2)                                               /*!< ALC_T::STS: P2PVAL Position            */
#define ALC_STS_P2PVAL_Msk               (0x1fful << ALC_STS_P2PVAL_Pos)                   /*!< ALC_T::STS: P2PVAL Mask                */

#define ALC_STS_PEAKVAL_Pos              (11)                                              /*!< ALC_T::STS: PEAKVAL Position           */
#define ALC_STS_PEAKVAL_Msk              (0x1fful << ALC_STS_PEAKVAL_Pos)                  /*!< ALC_T::STS: PEAKVAL Mask               */

#define ALC_STS_ALCGAIN_Pos              (20)                                              /*!< ALC_T::STS: ALCGAIN Position           */
#define ALC_STS_ALCGAIN_Msk              (0x3ful << ALC_STS_ALCGAIN_Pos)                   /*!< ALC_T::STS: ALCGAIN Mask               */

#define ALC_INTCTL_PLMTIE_Pos            (0)                                               /*!< ALC_T::INTCTL: PLMTIE Position         */
#define ALC_INTCTL_PLMTIE_Msk            (0x1ul << ALC_INTCTL_PLMTIE_Pos)                  /*!< ALC_T::INTCTL: PLMTIE Mask             */

#define ALC_INTCTL_NGIE_Pos              (1)                                               /*!< ALC_T::INTCTL: NGIE Position           */
#define ALC_INTCTL_NGIE_Msk              (0x1ul << ALC_INTCTL_NGIE_Pos)                    /*!< ALC_T::INTCTL: NGIE Mask               */

#define ALC_INTCTL_GINCIE_Pos            (2)                                               /*!< ALC_T::INTCTL: GINCIE Position         */
#define ALC_INTCTL_GINCIE_Msk            (0x1ul << ALC_INTCTL_GINCIE_Pos)                  /*!< ALC_T::INTCTL: GINCIE Mask             */

#define ALC_INTCTL_GDECIE_Pos            (3)                                               /*!< ALC_T::INTCTL: GDECIE Position         */
#define ALC_INTCTL_GDECIE_Msk            (0x1ul << ALC_INTCTL_GDECIE_Pos)                  /*!< ALC_T::INTCTL: GDECIE Mask             */

#define ALC_INTCTL_GMAXIE_Pos            (4)                                               /*!< ALC_T::INTCTL: GMAXIE Position         */
#define ALC_INTCTL_GMAXIE_Msk            (0x1ul << ALC_INTCTL_GMAXIE_Pos)                  /*!< ALC_T::INTCTL: GMAXIE Mask             */

#define ALC_INTCTL_GMINIE_Pos            (5)                                               /*!< ALC_T::INTCTL: GMINIE Position         */
#define ALC_INTCTL_GMINIE_Msk            (0x1ul << ALC_INTCTL_GMINIE_Pos)                  /*!< ALC_T::INTCTL: GMINIE Mask             */

#define ALC_INTCTL_PLMTIF_Pos            (8)                                               /*!< ALC_T::INTCTL: PLMTIF Position         */
#define ALC_INTCTL_PLMTIF_Msk            (0x1ul << ALC_INTCTL_PLMTIF_Pos)                  /*!< ALC_T::INTCTL: PLMTIF Mask             */

#define ALC_INTCTL_NGIF_Pos              (9)                                               /*!< ALC_T::INTCTL: NGIF Position           */
#define ALC_INTCTL_NGIF_Msk              (0x1ul << ALC_INTCTL_NGIF_Pos)                    /*!< ALC_T::INTCTL: NGIF Mask               */

#define ALC_INTCTL_GINCIF_Pos            (10)                                              /*!< ALC_T::INTCTL: GINCIF Position         */
#define ALC_INTCTL_GINCIF_Msk            (0x1ul << ALC_INTCTL_GINCIF_Pos)                  /*!< ALC_T::INTCTL: GINCIF Mask             */

#define ALC_INTCTL_GDECIF_Pos            (11)                                              /*!< ALC_T::INTCTL: GDECIF Position         */
#define ALC_INTCTL_GDECIF_Msk            (0x1ul << ALC_INTCTL_GDECIF_Pos)                  /*!< ALC_T::INTCTL: GDECIF Mask             */

#define ALC_INTCTL_GMAXIF_Pos            (12)                                              /*!< ALC_T::INTCTL: GMAXIF Position         */
#define ALC_INTCTL_GMAXIF_Msk            (0x1ul << ALC_INTCTL_GMAXIF_Pos)                  /*!< ALC_T::INTCTL: GMAXIF Mask             */

#define ALC_INTCTL_GMINIF_Pos            (13)                                              /*!< ALC_T::INTCTL: GMINIF Position         */
#define ALC_INTCTL_GMINIF_Msk            (0x1ul << ALC_INTCTL_GMINIF_Pos)                  /*!< ALC_T::INTCTL: GMINIF Mask             */

#define ALC_INTCTL_ALCIF_Pos             (31)                                              /*!< ALC_T::INTCTL: ALCIF Position          */
#define ALC_INTCTL_ALCIF_Msk             (0x1ul << ALC_INTCTL_ALCIF_Pos)                   /*!< ALC_T::INTCTL: ALCIF Mask              */

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
 * @var ANA_T::VMID
 * Offset: 0x00  VMID Reference Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PULLDOWN  |VMID Pulldown
 * |        |          |0= Release VMID pin for reference operation.
 * |        |          |1= Pull VMID pin to ground. Default power down and reset condition.
 * |[1]     |PDLORES   |Power Down Low (4.8kOhm) Resistance Reference
 * |        |          |0= Connect the Low Resistance reference to VMID
 * |        |          |Use this setting for fast power up of VMID
 * |        |          |Can be turned off after 50ms to save power.
 * |        |          |1= The Low Resistance reference is disconnected from VMID. Default power down and reset condition.
 * |[2]     |PDHIRES   |Power Down High (360kOhm) Resistance Reference
 * |        |          |0= Connect the High Resistance reference to VMID. Use this setting for minimum power consumption.
 * |        |          |1= The High Resistance reference is disconnected from VMID. Default power down and reset condition.
 * @var ANA_T::LDOSEL
 * Offset: 0x20  LDO Voltage Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |LDOSEL    |Select LDO Output Voltage
 * |        |          |Note that maximum I/O pad operation speed only specified for voltage >2.4V.
 * |        |          |0= 1.8V
 * |        |          |1= 2.4V
 * |        |          |2= 2.5V
 * |        |          |3= 2.7V
 * |        |          |4=3.0V
 * |        |          |5=3.3V
 * |        |          |6=1.5V
 * |        |          |7=1.7V
 * @var ANA_T::LDOPD
 * Offset: 0x24  LDO Power Down Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PD        |Power Down LDO
 * |        |          |When powered down no current delivered to VD33.
 * |        |          |0= Enable LDO.
 * |        |          |1= Power Down.
 * |[1]     |DISCHAR   |Discharge
 * |        |          |0 = Donu2019t discharge VD33.
 * |        |          |1 = Switch discharge resistor to VD33.
 * @var ANA_T::MICBSEL
 * Offset: 0x28  Microphone Bias Voltage Level Selection
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |LVL       |LVL controls the voltage output of the MICBIAS generator, voltages are encoded as following:  0 - 1.5V 1 - 1.8V 2 - 1.95V 3 - 2.1V 4 - 2.25V 5 - 2.4V 6 - 2.55V  7 - 2.7
 * |        |          |Note: MICBIAS voltage should be at least 300mV lower than VCCA.
 * @var ANA_T::MICBEN
 * Offset: 0x2C  Microphone Bias Enable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PD        |Power Down Microphone Bias
 * |        |          |0= Enable Microphone Bias
 * |        |          |1= Power Down Microphone Bias
 * |        |          |Note: MICBIAS output needs VMID enable together.
 * @var ANA_T::TRIM
 * Offset: 0x84  Oscillator Trim Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |OSCTRIM   |Oscillator Trim
 * |        |          |Reads current oscillator trim setting. Read Only.
 * |[15:8]  |COARSE    |COARSE
 * |        |          |Current COARSE range setting of the oscillator. Read Only
 * @var ANA_T::FQMMCTL
 * Offset: 0x94  Frequency Measurement Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |CLKSEL    |Reference Clock Source
 * |        |          |00b: OSC10k,
 * |        |          |01b: OSC32K (default),
 * |        |          |1xb: I2S_WS u2013 can be GPIOA[4,8,12] according to SYS_GPA_MFP register, configure I2S in SLAVE mode to enable.
 * |[2]     |MMSTS     |Measurement Done
 * |        |          |0 = Measurement Ongoing.
 * |        |          |1 = Measurement Complete.
 * |[23:16] |CYCLESEL  |Frequency Measurement Cycles
 * |        |          |Number of reference clock periods plus one to measure target clock (PCLK)
 * |        |          |For example if reference clock is OSC32K (T is 30.5175us), set CYCLESEL to 7, then measurement period would be 30.5175*(7+1), 244.1us.
 * |[31]    |FQMMEN    |FQMMEN
 * |        |          |0 = Disable/Reset block.
 * |        |          |1 = Start Frequency Measurement.
 * @var ANA_T::FQMMCNT
 * Offset: 0x98  Frequency Measurement Count Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |FQMMCNT   |Frequency Measurement Count
 * |        |          |When MMSTS = 1 and FQMMEN = 1, this is number of PCLK periods counted for frequency measurement.
 * |        |          |The frequency will be PCLK = FQMMCNT * Fref /(CYCLESEL+1) Hz.
 * |        |          |Maximum resolution of measurement is Fref /(CYCLESEL+1)*2 Hz
 * @var ANA_T::FQMMCYC
 * Offset: 0x9C  Frequency Measurement Cycle Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |FQMMCYC   |Frequency Measurement Cycles
 * |        |          |Number of reference clock periods plus one to measure target clock (PCLK)
 * |        |          |For example if reference clock is OSC32K (T = 30.5175us), FQMMCYC = 7, then measurement period would be 30.5175*(7+1) = 244.1us
 * |        |          |This address access same register as ANA_FQMMCTL but allows access to more bits of register.
 */
    __IO uint32_t VMID;                  /*!< [0x0000] VMID Reference Control Register                                  */
    __I  uint32_t RESERVE0[7];
    __IO uint32_t LDOSEL;                /*!< [0x0020] LDO Voltage Select Register                                      */
    __IO uint32_t LDOPD;                 /*!< [0x0024] LDO Power Down Register                                          */
    __IO uint32_t MICBSEL;               /*!< [0x0028] Microphone Bias Voltage Level Selection                          */
    __IO uint32_t MICBEN;                /*!< [0x002c] Microphone Bias Enable                                           */
    __I  uint32_t RESERVE1[21];
    __IO uint32_t TRIM;                  /*!< [0x0084] Oscillator Trim Register                                         */
    __I  uint32_t RESERVE2[3];
    __IO uint32_t FQMMCTL;               /*!< [0x0094] Frequency Measurement Control Register                           */
    __I  uint32_t FQMMCNT;               /*!< [0x0098] Frequency Measurement Count Register                             */
    __IO uint32_t FQMMCYC;               /*!< [0x009c] Frequency Measurement Cycle Register                             */

} ANA_T;

/**
    @addtogroup ANA_CONST ANA Bit Field Definition
    Constant Definitions for ANA Controller
@{ */

#define ANA_VMID_PULLDOWN_Pos            (0)                                               /*!< ANA_T::VMID: PULLDOWN Position         */
#define ANA_VMID_PULLDOWN_Msk            (0x1ul << ANA_VMID_PULLDOWN_Pos)                  /*!< ANA_T::VMID: PULLDOWN Mask             */

#define ANA_VMID_PDLORES_Pos             (1)                                               /*!< ANA_T::VMID: PDLORES Position          */
#define ANA_VMID_PDLORES_Msk             (0x1ul << ANA_VMID_PDLORES_Pos)                   /*!< ANA_T::VMID: PDLORES Mask              */

#define ANA_VMID_PDHIRES_Pos             (2)                                               /*!< ANA_T::VMID: PDHIRES Position          */
#define ANA_VMID_PDHIRES_Msk             (0x1ul << ANA_VMID_PDHIRES_Pos)                   /*!< ANA_T::VMID: PDHIRES Mask              */

#define ANA_LDOSEL_LDOSEL_Pos            (0)                                               /*!< ANA_T::LDOSEL: LDOSEL Position         */
#define ANA_LDOSEL_LDOSEL_Msk            (0x7ul << ANA_LDOSEL_LDOSEL_Pos)                  /*!< ANA_T::LDOSEL: LDOSEL Mask             */

#define ANA_LDOPD_PD_Pos                 (0)                                               /*!< ANA_T::LDOPD: PD Position              */
#define ANA_LDOPD_PD_Msk                 (0x1ul << ANA_LDOPD_PD_Pos)                       /*!< ANA_T::LDOPD: PD Mask                  */

#define ANA_LDOPD_DISCHAR_Pos            (1)                                               /*!< ANA_T::LDOPD: DISCHAR Position         */
#define ANA_LDOPD_DISCHAR_Msk            (0x1ul << ANA_LDOPD_DISCHAR_Pos)                  /*!< ANA_T::LDOPD: DISCHAR Mask             */

#define ANA_MICBSEL_LVL_Pos              (0)                                               /*!< ANA_T::MICBSEL: LVL Position           */
#define ANA_MICBSEL_LVL_Msk              (0x7ul << ANA_MICBSEL_LVL_Pos)                    /*!< ANA_T::MICBSEL: LVL Mask               */

#define ANA_MICBEN_PD_Pos                (0)                                               /*!< ANA_T::MICBEN: PD Position             */
#define ANA_MICBEN_PD_Msk                (0x1ul << ANA_MICBEN_PD_Pos)                      /*!< ANA_T::MICBEN: PD Mask                 */

#define ANA_TRIM_OSCTRIM_Pos             (0)                                               /*!< ANA_T::TRIM: OSCTRIM Position          */
#define ANA_TRIM_OSCTRIM_Msk             (0xfful << ANA_TRIM_OSCTRIM_Pos)                  /*!< ANA_T::TRIM: OSCTRIM Mask              */

#define ANA_TRIM_COARSE_Pos              (8)                                               /*!< ANA_T::TRIM: COARSE Position           */
#define ANA_TRIM_COARSE_Msk              (0xfful << ANA_TRIM_COARSE_Pos)                   /*!< ANA_T::TRIM: COARSE Mask               */

#define ANA_FQMMCTL_CLKSEL_Pos           (0)                                               /*!< ANA_T::FQMMCTL: CLKSEL Position        */
#define ANA_FQMMCTL_CLKSEL_Msk           (0x3ul << ANA_FQMMCTL_CLKSEL_Pos)                 /*!< ANA_T::FQMMCTL: CLKSEL Mask            */

#define ANA_FQMMCTL_MMSTS_Pos            (2)                                               /*!< ANA_T::FQMMCTL: MMSTS Position         */
#define ANA_FQMMCTL_MMSTS_Msk            (0x1ul << ANA_FQMMCTL_MMSTS_Pos)                  /*!< ANA_T::FQMMCTL: MMSTS Mask             */

#define ANA_FQMMCTL_CYCLESEL_Pos         (16)                                              /*!< ANA_T::FQMMCTL: CYCLESEL Position      */
#define ANA_FQMMCTL_CYCLESEL_Msk         (0xfful << ANA_FQMMCTL_CYCLESEL_Pos)              /*!< ANA_T::FQMMCTL: CYCLESEL Mask          */

#define ANA_FQMMCTL_FQMMEN_Pos           (31)                                              /*!< ANA_T::FQMMCTL: FQMMEN Position        */
#define ANA_FQMMCTL_FQMMEN_Msk           (0x1ul << ANA_FQMMCTL_FQMMEN_Pos)                 /*!< ANA_T::FQMMCTL: FQMMEN Mask            */

#define ANA_FQMMCNT_FQMMCNT_Pos          (0)                                               /*!< ANA_T::FQMMCNT: FQMMCNT Position       */
#define ANA_FQMMCNT_FQMMCNT_Msk          (0xfffful << ANA_FQMMCNT_FQMMCNT_Pos)             /*!< ANA_T::FQMMCNT: FQMMCNT Mask           */

#define ANA_FQMMCYC_FQMMCYC_Pos          (0)                                               /*!< ANA_T::FQMMCYC: FQMMCYC Position       */
#define ANA_FQMMCYC_FQMMCYC_Msk          (0xfffffful << ANA_FQMMCYC_FQMMCYC_Pos)           /*!< ANA_T::FQMMCYC: FQMMCYC Mask           */

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
 * @var BIQ_T::COEFF0
 * Offset: 0x00  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF1
 * Offset: 0x04  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF2
 * Offset: 0x08  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF3
 * Offset: 0x0C  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF4
 * Offset: 0x10  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF5
 * Offset: 0x14  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF6
 * Offset: 0x18  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF7
 * Offset: 0x1C  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF8
 * Offset: 0x20  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF9
 * Offset: 0x24  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF10
 * Offset: 0x28  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF11
 * Offset: 0x2C  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF12
 * Offset: 0x30  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF13
 * Offset: 0x34  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF14
 * Offset: 0x38  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF15
 * Offset: 0x3C  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF16
 * Offset: 0x40  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF17
 * Offset: 0x44  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF18
 * Offset: 0x48  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF19
 * Offset: 0x4C  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF20
 * Offset: 0x50  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF21
 * Offset: 0x54  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF22
 * Offset: 0x58  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF23
 * Offset: 0x5C  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF24
 * Offset: 0x60  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF25
 * Offset: 0x64  Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF26
 * Offset: 0x68  Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF27
 * Offset: 0x6C  Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF28
 * Offset: 0x70  Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::COEFF29
 * Offset: 0x74  Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |COEFFDAT  |Coefficient Data
 * @var BIQ_T::CTL
 * Offset: 0x80  BIQ Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BIQEN     |BIQ Filter Start to Run
 * |        |          |0 = BIQ filter is not processing.
 * |        |          |1 = BIQ filter is on.
 * |[1]     |HPFON     |High Pass Filter On
 * |        |          |0 = disable high pass filter.
 * |        |          |1 = enable high pass filter.
 * |        |          |Note :
 * |        |          |If this register is on, BIQ only 5 stage left.for user.
 * |        |          |SDADC path sixth stage coefficient is for HPF filter coefficient.
 * |        |          |DPWM path first stage coefficient is for HPF filter coefficient. 
 * |[2]     |PATHSEL   |AC Path Selection for BIQ
 * |        |          |0 = used in SDADC path.
 * |        |          |1 = used in DPWM path.
 * |[3]     |DLCOEFF   |Move BIQ Out of Reset State
 * |        |          |0 = BIQ filter is in reset state.
 * |        |          |1 = When this bit is on, the default coefficients will be downloaded to the coefficient ram automatically in 32 internal system clocks
 * |        |          |Processor must delay enough time before changing the coefficients or turn the BIQ on.
 * |[6:4]   |SDADCWNSR |SDADC Down Sample
 * |        |          |001--- 1x (no down sample)
 * |        |          |010 --- 2x
 * |        |          |011 --- 3x
 * |        |          |100 --- 4x
 * |        |          |11 0--- 6x
 * |        |          |Others reserved
 * |[7]     |PRGCOEFF  |Programming Mode Coefficient Control Bit
 * |        |          |0 = Coefficient RAM is in normal mode.
 * |        |          |1 = coefficient RAM is under programming mode.
 * |        |          |This bit must be turned off when BIQEN is on.
 * |[10:8]  |DPWMPUSR  |DPWM Path Up Sample Rate (From SRDIV Result)
 * |        |          |0001 --- up 1x ( no up sample)
 * |        |          |0010 --- up 2x
 * |        |          |0011 --- up 3x
 * |        |          |0100 --- up 4x
 * |        |          |0110 --- up 6x
 * |        |          |Others reserved
 * |[11]    |STAGE     |BIQ Stage Number Control
 * |        |          |0 = 6 stage.
 * |        |          |1 = 5 stage.
 * |[28:16] |SRDIV     |SR Divider
 * @var BIQ_T::STS
 * Offset: 0x84  BIQ Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BISTEN    |RAM BIST testing Enable for internal use
 * |[1]     |BISTFAILED|RAM BIST testing FAILED indicator for internal use
 * |[2]     |BISTDONE  |RAM BIST testing DONE flag for internal use
 * |[31]    |RAMINITF  |Coefficient Ram Initial Default Done Flag
 * |        |          |0 = initial default value done.
 * |        |          |1 = still working on.
 */
    __IO uint32_t COEFF0;                /*!< [0x0000] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients */
    __IO uint32_t COEFF1;                /*!< [0x0004] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients */
    __IO uint32_t COEFF2;                /*!< [0x0008] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients */
    __IO uint32_t COEFF3;                /*!< [0x000c] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients */
    __IO uint32_t COEFF4;                /*!< [0x0010] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 1st Stage BIQ Coefficients */
    __IO uint32_t COEFF5;                /*!< [0x0014] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients */
    __IO uint32_t COEFF6;                /*!< [0x0018] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients */
    __IO uint32_t COEFF7;                /*!< [0x001c] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients */
    __IO uint32_t COEFF8;                /*!< [0x0020] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients */
    __IO uint32_t COEFF9;                /*!< [0x0024] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 2nd Stage BIQ Coefficients */
    __IO uint32_t COEFF10;               /*!< [0x0028] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients */
    __IO uint32_t COEFF11;               /*!< [0x002c] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients */
    __IO uint32_t COEFF12;               /*!< [0x0030] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients */
    __IO uint32_t COEFF13;               /*!< [0x0034] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients */
    __IO uint32_t COEFF14;               /*!< [0x0038] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 3rd Stage BIQ Coefficients */
    __IO uint32_t COEFF15;               /*!< [0x003c] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients */
    __IO uint32_t COEFF16;               /*!< [0x0040] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients */
    __IO uint32_t COEFF17;               /*!< [0x0044] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients */
    __IO uint32_t COEFF18;               /*!< [0x0048] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients */
    __IO uint32_t COEFF19;               /*!< [0x004c] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 4st Stage BIQ Coefficients */
    __IO uint32_t COEFF20;               /*!< [0x0050] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients */
    __IO uint32_t COEFF21;               /*!< [0x0054] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients */
    __IO uint32_t COEFF22;               /*!< [0x0058] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients */
    __IO uint32_t COEFF23;               /*!< [0x005c] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients */
    __IO uint32_t COEFF24;               /*!< [0x0060] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 5nd Stage BIQ Coefficients */
    __IO uint32_t COEFF25;               /*!< [0x0064] Coefficient B0 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients */
    __IO uint32_t COEFF26;               /*!< [0x0068] Coefficient B1 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients */
    __IO uint32_t COEFF27;               /*!< [0x006c] Coefficient B2 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients */
    __IO uint32_t COEFF28;               /*!< [0x0070] Coefficient A1 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients */
    __IO uint32_t COEFF29;               /*!< [0x0074] Coefficient A2 in H(z) Transfer Function
(3.16 Format) - 6rd Stage BIQ Coefficients */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t CTL;                   /*!< [0x0080] BIQ Control Register                                             */
    __IO uint32_t STS;                   /*!< [0x0084] BIQ Status Register                                              */

} BIQ_T;

/**
    @addtogroup BIQ_CONST BIQ Bit Field Definition
    Constant Definitions for BIQ Controller
@{ */

#define BIQ_COEFF0_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF0: COEFFDAT Position       */
#define BIQ_COEFF0_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF0_COEFFDAT_Pos)         /*!< BIQ_T::COEFF0: COEFFDAT Mask           */

#define BIQ_COEFF1_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF1: COEFFDAT Position       */
#define BIQ_COEFF1_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF1_COEFFDAT_Pos)         /*!< BIQ_T::COEFF1: COEFFDAT Mask           */

#define BIQ_COEFF2_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF2: COEFFDAT Position       */
#define BIQ_COEFF2_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF2_COEFFDAT_Pos)         /*!< BIQ_T::COEFF2: COEFFDAT Mask           */

#define BIQ_COEFF3_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF3: COEFFDAT Position       */
#define BIQ_COEFF3_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF3_COEFFDAT_Pos)         /*!< BIQ_T::COEFF3: COEFFDAT Mask           */

#define BIQ_COEFF4_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF4: COEFFDAT Position       */
#define BIQ_COEFF4_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF4_COEFFDAT_Pos)         /*!< BIQ_T::COEFF4: COEFFDAT Mask           */

#define BIQ_COEFF5_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF5: COEFFDAT Position       */
#define BIQ_COEFF5_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF5_COEFFDAT_Pos)         /*!< BIQ_T::COEFF5: COEFFDAT Mask           */

#define BIQ_COEFF6_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF6: COEFFDAT Position       */
#define BIQ_COEFF6_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF6_COEFFDAT_Pos)         /*!< BIQ_T::COEFF6: COEFFDAT Mask           */

#define BIQ_COEFF7_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF7: COEFFDAT Position       */
#define BIQ_COEFF7_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF7_COEFFDAT_Pos)         /*!< BIQ_T::COEFF7: COEFFDAT Mask           */

#define BIQ_COEFF8_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF8: COEFFDAT Position       */
#define BIQ_COEFF8_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF8_COEFFDAT_Pos)         /*!< BIQ_T::COEFF8: COEFFDAT Mask           */

#define BIQ_COEFF9_COEFFDAT_Pos          (0)                                               /*!< BIQ_T::COEFF9: COEFFDAT Position       */
#define BIQ_COEFF9_COEFFDAT_Msk          (0xfffffffful << BIQ_COEFF9_COEFFDAT_Pos)         /*!< BIQ_T::COEFF9: COEFFDAT Mask           */

#define BIQ_COEFF10_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF10: COEFFDAT Position      */
#define BIQ_COEFF10_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF10_COEFFDAT_Pos)        /*!< BIQ_T::COEFF10: COEFFDAT Mask          */

#define BIQ_COEFF11_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF11: COEFFDAT Position      */
#define BIQ_COEFF11_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF11_COEFFDAT_Pos)        /*!< BIQ_T::COEFF11: COEFFDAT Mask          */

#define BIQ_COEFF12_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF12: COEFFDAT Position      */
#define BIQ_COEFF12_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF12_COEFFDAT_Pos)        /*!< BIQ_T::COEFF12: COEFFDAT Mask          */

#define BIQ_COEFF13_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF13: COEFFDAT Position      */
#define BIQ_COEFF13_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF13_COEFFDAT_Pos)        /*!< BIQ_T::COEFF13: COEFFDAT Mask          */

#define BIQ_COEFF14_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF14: COEFFDAT Position      */
#define BIQ_COEFF14_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF14_COEFFDAT_Pos)        /*!< BIQ_T::COEFF14: COEFFDAT Mask          */

#define BIQ_COEFF15_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF15: COEFFDAT Position      */
#define BIQ_COEFF15_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF15_COEFFDAT_Pos)        /*!< BIQ_T::COEFF15: COEFFDAT Mask          */

#define BIQ_COEFF16_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF16: COEFFDAT Position      */
#define BIQ_COEFF16_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF16_COEFFDAT_Pos)        /*!< BIQ_T::COEFF16: COEFFDAT Mask          */

#define BIQ_COEFF17_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF17: COEFFDAT Position      */
#define BIQ_COEFF17_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF17_COEFFDAT_Pos)        /*!< BIQ_T::COEFF17: COEFFDAT Mask          */

#define BIQ_COEFF18_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF18: COEFFDAT Position      */
#define BIQ_COEFF18_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF18_COEFFDAT_Pos)        /*!< BIQ_T::COEFF18: COEFFDAT Mask          */

#define BIQ_COEFF19_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF19: COEFFDAT Position      */
#define BIQ_COEFF19_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF19_COEFFDAT_Pos)        /*!< BIQ_T::COEFF19: COEFFDAT Mask          */

#define BIQ_COEFF20_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF20: COEFFDAT Position      */
#define BIQ_COEFF20_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF20_COEFFDAT_Pos)        /*!< BIQ_T::COEFF20: COEFFDAT Mask          */

#define BIQ_COEFF21_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF21: COEFFDAT Position      */
#define BIQ_COEFF21_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF21_COEFFDAT_Pos)        /*!< BIQ_T::COEFF21: COEFFDAT Mask          */

#define BIQ_COEFF22_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF22: COEFFDAT Position      */
#define BIQ_COEFF22_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF22_COEFFDAT_Pos)        /*!< BIQ_T::COEFF22: COEFFDAT Mask          */

#define BIQ_COEFF23_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF23: COEFFDAT Position      */
#define BIQ_COEFF23_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF23_COEFFDAT_Pos)        /*!< BIQ_T::COEFF23: COEFFDAT Mask          */

#define BIQ_COEFF24_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF24: COEFFDAT Position      */
#define BIQ_COEFF24_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF24_COEFFDAT_Pos)        /*!< BIQ_T::COEFF24: COEFFDAT Mask          */

#define BIQ_COEFF25_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF25: COEFFDAT Position      */
#define BIQ_COEFF25_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF25_COEFFDAT_Pos)        /*!< BIQ_T::COEFF25: COEFFDAT Mask          */

#define BIQ_COEFF26_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF26: COEFFDAT Position      */
#define BIQ_COEFF26_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF26_COEFFDAT_Pos)        /*!< BIQ_T::COEFF26: COEFFDAT Mask          */

#define BIQ_COEFF27_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF27: COEFFDAT Position      */
#define BIQ_COEFF27_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF27_COEFFDAT_Pos)        /*!< BIQ_T::COEFF27: COEFFDAT Mask          */

#define BIQ_COEFF28_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF28: COEFFDAT Position      */
#define BIQ_COEFF28_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF28_COEFFDAT_Pos)        /*!< BIQ_T::COEFF28: COEFFDAT Mask          */

#define BIQ_COEFF29_COEFFDAT_Pos         (0)                                               /*!< BIQ_T::COEFF29: COEFFDAT Position      */
#define BIQ_COEFF29_COEFFDAT_Msk         (0xfffffffful << BIQ_COEFF29_COEFFDAT_Pos)        /*!< BIQ_T::COEFF29: COEFFDAT Mask          */

#define BIQ_CTL_BIQEN_Pos                (0)                                               /*!< BIQ_T::CTL: BIQEN Position             */
#define BIQ_CTL_BIQEN_Msk                (0x1ul << BIQ_CTL_BIQEN_Pos)                      /*!< BIQ_T::CTL: BIQEN Mask                 */

#define BIQ_CTL_HPFON_Pos                (1)                                               /*!< BIQ_T::CTL: HPFON Position             */
#define BIQ_CTL_HPFON_Msk                (0x1ul << BIQ_CTL_HPFON_Pos)                      /*!< BIQ_T::CTL: HPFON Mask                 */

#define BIQ_CTL_PATHSEL_Pos              (2)                                               /*!< BIQ_T::CTL: PATHSEL Position           */
#define BIQ_CTL_PATHSEL_Msk              (0x1ul << BIQ_CTL_PATHSEL_Pos)                    /*!< BIQ_T::CTL: PATHSEL Mask               */

#define BIQ_CTL_DLCOEFF_Pos              (3)                                               /*!< BIQ_T::CTL: DLCOEFF Position           */
#define BIQ_CTL_DLCOEFF_Msk              (0x1ul << BIQ_CTL_DLCOEFF_Pos)                    /*!< BIQ_T::CTL: DLCOEFF Mask               */

#define BIQ_CTL_SDADCWNSR_Pos            (4)                                               /*!< BIQ_T::CTL: SDADCWNSR Position         */
#define BIQ_CTL_SDADCWNSR_Msk            (0x7ul << BIQ_CTL_SDADCWNSR_Pos)                  /*!< BIQ_T::CTL: SDADCWNSR Mask             */

#define BIQ_CTL_PRGCOEFF_Pos             (7)                                               /*!< BIQ_T::CTL: PRGCOEFF Position          */
#define BIQ_CTL_PRGCOEFF_Msk             (0x1ul << BIQ_CTL_PRGCOEFF_Pos)                   /*!< BIQ_T::CTL: PRGCOEFF Mask              */

#define BIQ_CTL_DPWMPUSR_Pos             (8)                                               /*!< BIQ_T::CTL: DPWMPUSR Position          */
#define BIQ_CTL_DPWMPUSR_Msk             (0x7ul << BIQ_CTL_DPWMPUSR_Pos)                   /*!< BIQ_T::CTL: DPWMPUSR Mask              */

#define BIQ_CTL_STAGE_Pos                (11)                                              /*!< BIQ_T::CTL: STAGE Position             */
#define BIQ_CTL_STAGE_Msk                (0x1ul << BIQ_CTL_STAGE_Pos)                      /*!< BIQ_T::CTL: STAGE Mask                 */

#define BIQ_CTL_SRDIV_Pos                (16)                                              /*!< BIQ_T::CTL: SRDIV Position             */
#define BIQ_CTL_SRDIV_Msk                (0x1ffful << BIQ_CTL_SRDIV_Pos)                   /*!< BIQ_T::CTL: SRDIV Mask                 */

#define BIQ_STS_BISTEN_Pos               (0)                                               /*!< BIQ_T::STS: BISTEN Position            */
#define BIQ_STS_BISTEN_Msk               (0x1ul << BIQ_STS_BISTEN_Pos)                     /*!< BIQ_T::STS: BISTEN Mask                */

#define BIQ_STS_BISTFAILED_Pos           (1)                                               /*!< BIQ_T::STS: BISTFAILED Position        */
#define BIQ_STS_BISTFAILED_Msk           (0x1ul << BIQ_STS_BISTFAILED_Pos)                 /*!< BIQ_T::STS: BISTFAILED Mask            */

#define BIQ_STS_BISTDONE_Pos             (2)                                               /*!< BIQ_T::STS: BISTDONE Position          */
#define BIQ_STS_BISTDONE_Msk             (0x1ul << BIQ_STS_BISTDONE_Pos)                   /*!< BIQ_T::STS: BISTDONE Mask              */

#define BIQ_STS_RAMINITF_Pos             (31)                                              /*!< BIQ_T::STS: RAMINITF Position          */
#define BIQ_STS_RAMINITF_Msk             (0x1ul << BIQ_STS_RAMINITF_Pos)                   /*!< BIQ_T::STS: RAMINITF Mask              */

/**@}*/ /* BIQ_CONST */
/**@}*/ /* end of BIQ register group */


/*---------------------- Brown Out Detector -------------------------*/
/**
    @addtogroup BOD Brown Out Detector(BOD)
    Memory Mapped Structure for BOD Controller
@{ */
 
typedef struct
{


/**
 * @var BOD_T::BODSEL
 * Offset: 0x00  Brown Out Detector Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |BODVL     |BOD Voltage Level
 * |        |          |1111b = 4.6V.
 * |        |          |1110b = 4.2V.
 * |        |          |1101b = 3.9V.
 * |        |          |1100b = 3.7V.
 * |        |          |1011b = 3.6V.
 * |        |          |1010b = 3.4V.
 * |        |          |1001b = 3.1V.
 * |        |          |1000b = 3.0V.
 * |        |          |0111b= 2.8V
 * |        |          |0110b = 2.6V.
 * |        |          |0101b = 2.4V.
 * |        |          |0100b = 2.2V.
 * |        |          |0011b = 2.1V.
 * |        |          |0010b = 2.0V.
 * |        |          |0001b = 1.9V.
 * |        |          |0000b = 1.8V.
 * |[4]     |BODHYS    |BOD Hysteresis
 * |        |          |0 = Hysteresis Disabled.
 * |        |          |1 = Enable Hysteresis of BOD detection.
 * @var BOD_T::BODCTL
 * Offset: 0x04  Brown Out Detector Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |BODEN     |BOD Enable
 * |        |          |1xb = Enable continuous BOD detection.
 * |        |          |01b = Enable time multiplexed BOD detection. See BOD_BODDTMR register.
 * |        |          |00b = Disable BOD Detection.
 * |[2]     |BODINTEN  |BOD Interrupt Enable
 * |        |          |0= Disable BOD Interrupt.
 * |        |          |1= Enable BOD Interrupt.
 * |[3]     |BODIF     |Current Status of Interrupt
 * |        |          |Latched whenever a BOD event occurs and IE=1. Write '1' to clear.
 * |[4]     |BODOUT    |Output of BOD Detection Block
 * |        |          |This signal can be monitored to determine the current state of the BOD comparator
 * |        |          |BODOUT = 1 implies that VCC is less than BODVL.
 * |[5]     |BODRST    |BOD Reset
 * |        |          |1: Reset device when BOD is triggered.
 * |[16]    |LVR_EN    |Low Voltage Reset (LVR) Enable (Initialized & Protected Bit)
 * |        |          |The LVR function resets the chip when the input power voltage is lower than LVR trip point
 * |        |          |Default value is set by flash controller as inverse of CLVR config0[27].
 * |        |          |0 = Disable LVR function.
 * |        |          |1 = Enable LVR function.
 * |[18:17] |LVR_FILTER|00 = LVR output will be filtered by 1 HCLK.
 * |        |          |01 = LVR output will be filtered by 2 HCLK
 * |        |          |10 = LVR output will be filtered by 8 HCLK
 * |        |          |11 = LVR output will be filtered by 15 HCLK
 * |        |          |Default value is 00.
 * @var BOD_T::BODDTMR
 * Offset: 0x10  Brown Out Detector Timer Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DURTOFF   |Time BOD Detector Is Off
 * |        |          |(DURTOFF+1)*100us . Minimum value is 7. (default is 99.6ms)
 * |[19:16] |DURTON    |Time BOD Detector Is Active
 * |        |          |(DURTON+1) * 100us. Minimum value is 1. (default is 400us)
 */
    __IO uint32_t BODSEL;                /*!< [0x0000] Brown Out Detector Select Register                               */
    __IO uint32_t BODCTL;                /*!< [0x0004] Brown Out Detector Enable Register                               */
    __I  uint32_t RESERVE0[2];
    __IO uint32_t BODDTMR;               /*!< [0x0010] Brown Out Detector Timer Register                                */

} BOD_T;

/**
    @addtogroup BOD_CONST BOD Bit Field Definition
    Constant Definitions for BOD Controller
@{ */

#define BOD_BODSEL_BODVL_Pos             (0)                                               /*!< BOD_T::BODSEL: BODVL Position          */
#define BOD_BODSEL_BODVL_Msk             (0xful << BOD_BODSEL_BODVL_Pos)                   /*!< BOD_T::BODSEL: BODVL Mask              */

#define BOD_BODSEL_BODHYS_Pos            (4)                                               /*!< BOD_T::BODSEL: BODHYS Position         */
#define BOD_BODSEL_BODHYS_Msk            (0x1ul << BOD_BODSEL_BODHYS_Pos)                  /*!< BOD_T::BODSEL: BODHYS Mask             */

#define BOD_BODCTL_BODEN_Pos             (0)                                               /*!< BOD_T::BODCTL: BODEN Position          */
#define BOD_BODCTL_BODEN_Msk             (0x3ul << BOD_BODCTL_BODEN_Pos)                   /*!< BOD_T::BODCTL: BODEN Mask              */

#define BOD_BODCTL_BODINTEN_Pos          (2)                                               /*!< BOD_T::BODCTL: BODINTEN Position       */
#define BOD_BODCTL_BODINTEN_Msk          (0x1ul << BOD_BODCTL_BODINTEN_Pos)                /*!< BOD_T::BODCTL: BODINTEN Mask           */

#define BOD_BODCTL_BODIF_Pos             (3)                                               /*!< BOD_T::BODCTL: BODIF Position          */
#define BOD_BODCTL_BODIF_Msk             (0x1ul << BOD_BODCTL_BODIF_Pos)                   /*!< BOD_T::BODCTL: BODIF Mask              */

#define BOD_BODCTL_BODOUT_Pos            (4)                                               /*!< BOD_T::BODCTL: BODOUT Position         */
#define BOD_BODCTL_BODOUT_Msk            (0x1ul << BOD_BODCTL_BODOUT_Pos)                  /*!< BOD_T::BODCTL: BODOUT Mask             */

#define BOD_BODCTL_BODRST_Pos            (5)                                               /*!< BOD_T::BODCTL: BODRST Position         */
#define BOD_BODCTL_BODRST_Msk            (0x1ul << BOD_BODCTL_BODRST_Pos)                  /*!< BOD_T::BODCTL: BODRST Mask             */

#define BOD_BODCTL_LVR_EN_Pos            (16)                                              /*!< BOD_T::BODCTL: LVR_EN Position         */
#define BOD_BODCTL_LVR_EN_Msk            (0x1ul << BOD_BODCTL_LVR_EN_Pos)                  /*!< BOD_T::BODCTL: LVR_EN Mask             */

#define BOD_BODCTL_LVR_FILTER_Pos        (17)                                              /*!< BOD_T::BODCTL: LVR_FILTER Position     */
#define BOD_BODCTL_LVR_FILTER_Msk        (0x3ul << BOD_BODCTL_LVR_FILTER_Pos)              /*!< BOD_T::BODCTL: LVR_FILTER Mask         */

#define BOD_BODDTMR_DURTOFF_Pos          (0)                                               /*!< BOD_T::BODDTMR: DURTOFF Position       */
#define BOD_BODDTMR_DURTOFF_Msk          (0xfffful << BOD_BODDTMR_DURTOFF_Pos)             /*!< BOD_T::BODDTMR: DURTOFF Mask           */

#define BOD_BODDTMR_DURTON_Pos           (16)                                              /*!< BOD_T::BODDTMR: DURTON Position        */
#define BOD_BODDTMR_DURTON_Msk           (0xful << BOD_BODDTMR_DURTON_Pos)                 /*!< BOD_T::BODDTMR: DURTON Mask            */

/**@}*/ /* BOD_CONST */
/**@}*/ /* end of BOD register group */


/*---------------------- System Clock Controller -------------------------*/
/**
    @addtogroup CLK System Clock Controller(CLK)
    Memory Mapped Structure for CLK Controller
@{ */
 
typedef struct
{

/**
 * @var CLK_T::PWRCTL
 * Offset: 0x00  System Power Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |LXTEN     |External 32.768 KHz Crystal Enable Bit
 * |        |          |0 = disable (default).
 * |        |          |1 = enable.
 * |[2]     |HIRCEN    |HIRC Oscillator Enable Bit
 * |        |          |0 = disable.
 * |        |          |1 = enable (default).
 * |[3]     |LIRCEN    |LIRC Oscillator Enable Bit
 * |        |          |0 = disable.
 * |        |          |1 = enable (default).
 * |[4]     |HXTEN     |HXT Oscillator Enable Bit
 * |        |          |0 = disable (default).
 * |        |          |1 = enable.
 * |[9]     |STOPEN    |Stop
 * |        |          |Set to '1' and issue WFI/WFE instruction to enter STOP mode.
 * |[10]    |SPDEN     |Standby Power Down (SPD) Bit
 * |        |          |Set to '1' and issue WFI/WFE instruction to enter SPD mode.
 * |[11]    |DPDEN     |Deep Power Down (DPD) Bit
 * |        |          |Set to '1' and issue WFI/WFE instruction to enter DPD mode.
 * |[12]    |HOLDIO    |When entering SPD mode, IO state is automatically held If this bit is set to '1' then this sate upon resuming full power mode will be hold until the RELEASE_IO bit is written '1'.
 * |[13]    |RELEASEIO |Write '1' to this bit to release IO state after exiting SPD if hold request was made with the HOLD_IO bit.
 * |[14]    |IOSTATE   |'1': IO held from SPD '0': IO released.
 * |[16]    |WKPINEN   |Wakeup Pin Enabled Control
 * |        |          |Determines whether WAKEUP pin is enabled in DPD mode.
 * |        |          |0 = enabled.
 * |        |          |1 = disabled.
 * |[17]    |LIRCDPDEN |OSC10k Enabled Control
 * |        |          |Determines whether OSC10k is enabled in DPD mode
 * |        |          |If OSC10k is disabled, device cannot wake from DPD with SELWKTMR delay.
 * |        |          |0 = enabled.
 * |        |          |1 = disabled.
 * |[19:18] |FLASHEN   |Flash ROM Control Enable/Disable
 * |        |          |Bit [19]: for Stop mode operation
 * |        |          |Bit [18]: for Sleep mode operation
 * |        |          |1: Turn off flash
 * |        |          |0: Normal
 * |        |          |Note: It takes 10us to turn on the flash to normal
 * |[24]    |WKPINWKF  |Pin Wakeup Flag
 * |        |          |Read Only
 * |        |          |This flag indicates that wakeup of device was requested with a high to low transition of the WAKEUP pin
 * |        |          |Flag is cleared when DPD mode is entered.
 * |[25]    |TMRWKF    |Timer Wakeup Flag
 * |        |          |Read Only
 * |        |          |This flag indicates that wakeup of device was requested with TIMER count of the 10khz oscillator
 * |        |          |Flag is cleared when DPD mode is entered.
 * |[26]    |PORWKF    |POR Wakeup Flag
 * |        |          |Read Only
 * |        |          |This flag indicates that wakeup of device was requested with a power-on reset
 * |        |          |Flag is cleared when DPD mode is entered.
 * |[27]    |WKPUEN    |Wakeup Pin Pull-up Control
 * |        |          |This signal is latched in deep power down and preserved.
 * |        |          |0 = pull-up enable.
 * |        |          |1 = tri-state (default).
 * @var CLK_T::AHBCLK
 * Offset: 0x04  AHB Device Clock Enable Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |HCLKEN    |CPU Clock Enable (HCLK)
 * |        |          |Must be left as 1 for normal operation.
 * |[1]     |PDMACKEN  |PDMA Controller Clock Enable Control
 * |        |          |0 = To disable the PDMA engine clock.
 * |        |          |1 = To enable the PDMA engine clock.
 * |[2]     |ISPCKEN   |Flash ISP Controller Clock Enable Control
 * |        |          |0 = To disable the Flash ISP engine clock.
 * |        |          |1 = To enable the Flash ISP engine clock.
 * @var CLK_T::APBCLK0
 * Offset: 0x08  APB Device Clock Enable Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |WDTCKEN   |Watchdog Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[5]     |RTCCKEN   |Real-time-clock APB Interface Clock Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[6]     |TMR0CKEN  |Timer0 Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[7]     |TMR1CKEN  |Timer1 Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[8]     |I2C0CKEN  |I2C0 Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[11]    |SPI1CKEN  |SPI1 Clock Enable Control
 * |        |          |0=Disable
 * |        |          |1=Enable
 * |[12]    |SPI0CKEN  |SPI0 Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[13]    |DPWMCKEN  |Differential PWM Speaker Driver Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[15]    |UART1CKEN |UART1 Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[16]    |UART0CKEN |UART0 Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[17]    |SARADCKEN |SAR Analog-digital-converter (ADC) Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[18]    |BIQALCKEN |BIQ and ALC Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[20]    |PWM0CH01CKEN|PWM0CH0 and PWM0CH1 Block Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[21]    |PWM0CH23CKEN|PWM0CH2 and PWM0CH3 Block Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[28]    |SDADCCKEN |Delta-Sigma Analog-digital-converter (ADC) Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[29]    |I2S0CKEN  |I2S Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[30]    |ANACKEN   |Analog Block Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * @var CLK_T::DPDSTATE
 * Offset: 0x0C  Deep Power Down State Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DPDSTSWR  |DPD State Write Register
 * |        |          |Read back of CLK_DPDSTATE register. This register was preserved from last DPD event . 
 * |[15:8]  |DPDSTSRD  |DPD State Read Back
 * |        |          |Read back of CLK_DPDSTATE register. This register was preserved from last DPD event . 
 * @var CLK_T::CLKSEL0
 * Offset: 0x10  Clock Source Select Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |HCLKSEL   |HCLK Clock Source Select
 * |        |          |Ensure that related clock sources (pre-select and new-select) are enabled before updating register.
 * |        |          |These bits are protected, to write to bits first perform the unlock sequence.
 * |        |          |000 = clock source from HIRC. (deafult)
 * |        |          |001 = clock source from LXT.
 * |        |          |010 = clock source from LIRC.
 * |        |          |011 = clock source from HXT.
 * |        |          |Others = Reserved.
 * |[5:3]   |STCLKSEL  |MCU Cortex_M0 SYST Clock Source Select
 * |        |          |These bits are protected, to write to bits first perform the unlock sequence.
 * |        |          |000 = clock source from LIRC.
 * |        |          |001 = clock source from LXT.
 * |        |          |010 = clock source from LIRC divided by 2.
 * |        |          |011 = clock source from HIRC divided by 2.
 * |        |          |1xx = clock source from HCLKu00F72 (Default).
 * |        |          |Note that to use STCLKSEL as source of SysTic timer the CLKSRC bit of SYST_CSR must be set to 0.
 * |[7:6]   |HIRCFSEL  |High Frequency RC Oscilltor Frequency Select Register.
 * |        |          |These bits are protected, to write to bits first perform the unlock sequence.
 * |        |          |00 = Trim for 49.152MHz selected.
 * |        |          |01 = Trim for 32.768MHz selected.
 * |        |          |10 = Trim for reserved.
 * @var CLK_T::CLKSEL1
 * Offset: 0x14  Clock Source Select Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |WDTSEL    |WDT Clock Source Select
 * |        |          |00 = clock source from HIRC.
 * |        |          |01 = clock source from LXT.
 * |        |          |10 = clock source from HCLK/2048 clock.
 * |        |          |11 = clock source from LIRC.(default)
 * |[3:2]   |SDADCSEL  |SD ADC Clock Source Select (output is MCLK after clock enable)
 * |        |          |00 = clock source from HCLK/(SDADCDIV+1). (default)
 * |        |          |01 = clock source from HXT
 * |        |          |1x = Reserved.
 * |[5:4]   |DPWMSEL   |Differential Speaker Driver PWM Clock Source Select
 * |        |          |00 = clock source from HCLK/(DPWMDIV+1). (default)
 * |        |          |01 = clock source from HXT
 * |        |          |1x = Reserved.
 * |[10:8]  |TMR0SEL   |TIMER0 Clock Source Select
 * |        |          |000 = clock source from LIRC.
 * |        |          |001 = clock source from LXT.
 * |        |          |010 = clock source from HXT.
 * |        |          |011 = clock source from external pin (GPIOA[10]).
 * |        |          |1xx = clock source from internal HCLK.(default)
 * |[14:12] |TMR1SEL   |TIMER1 Clock Source Select
 * |        |          |000 = clock source from LIRC.
 * |        |          |001 = clock source from LXT.
 * |        |          |010 = clock source from HXT.
 * |        |          |011 = clock source from external pin (GPIOA[11]).
 * |        |          |1xx = clock source from HCLK.(default)
 * |[25:24] |SARADCSEL |SAR ADC Clock Source Select
 * |        |          |00 = clock source from HCLK (default)
 * |        |          |01 = clock source from LIRC.
 * |        |          |10 = clock source from HIRC.
 * |        |          |11 = clock source from LXT.
 * |[29:28] |PWM0CH01SEL|PWM0CH01 Clock Source Select
 * |        |          |PWM0 CH0 and CH1 uses the same clock source, and pre-scaler
 * |        |          |00 = clock source from LIRC.
 * |        |          |01 = clock source from LXT.
 * |        |          |10 = clock source from HCLK.
 * |        |          |11 = clock source from HIRC.(default)
 * |[31:30] |PWM0CH23SEL|PWM0CH23 Clock Source Select
 * |        |          |PWM0 CH2 and CH3 uses the same clock source, and pre-scaler
 * |        |          |00 = clock source from LIRC.
 * |        |          |01 = clock source from LXT.
 * |        |          |10 = clock source from HCLK.
 * |        |          |11 = clock source from HIRC.(default)
 * @var CLK_T::CLKDIV0
 * Offset: 0x18  Clock Divider Number Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |HCLKDIV   |HCLK Clock Divide Number From HCLK Clock Source
 * |        |          |The HCLK clock frequency = (System clock source frequency) / (HCLKDIV + 1).
 * |[7:4]   |BIQDIV    |BIQ Clock Divide Number From HCLK Clock Source
 * |        |          |The HCLK clock frequency = (System clock source frequency) / (BIQDIV + 1).
 * |        |          |Suggestion: BIQ_N =1, if HCLK is 49MHz.
 * |[11:8]  |UARTDIV   |UART Clock Divide Number From UART Clock Source
 * |        |          |The UART clock frequency = (UART clock source frequency ) / (UARTDIV + 1).
 * |[15:12] |DPWMDIV   |DPWM Clock Divide Number From HCLK Clock Source
 * |        |          |The HCLK clock frequency = (System clock source frequency) / (DPWMDIV + 1).
 * |[23:16] |SDADCDIV  |SDADC Clock Divide Number From ADC Clock Source
 * |        |          |The SDADC clock frequency = (SDADC clock source frequency ) / (SDADCDIV + 1)
 * |[31:24] |SARADCDIV |SARADC Clock Divide Number From ADC Clock Source
 * |        |          |The SARADC clock frequency = (ADC clock source frequency ) / (SARADCDIV + 1).
 * @var CLK_T::CLKSEL2
 * Offset: 0x1C  Clock Source Select Control Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |I2S0SEL   |I2S0 Clock Source Select
 * |        |          |00 = clock source from internal 10kHz oscillator.(default)
 * |        |          |01 = clock source from external 32kHz crystal clock.
 * |        |          |10 = clock source from HCLK
 * |        |          |11 = clock source from HIRC.
 * |[11:8]  |UART1DIV  |UART1 Clock Divide Number From UART Clock Source
 * |        |          |The UART clock frequency = (UART clock source frequency ) / (UART1DIV + 1).
 * @var CLK_T::SLEEPCTL
 * Offset: 0x20  Sleep Clock Source Select  Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |HCLKCKEN  |CPU Clock Sleep Enable (HCLK)
 * |        |          |Must be left as '1' for normal operation.
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[1]     |PDMACKEN  |PDMA Controller Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[2]     |ISPCKEN   |Flash ISP Controller Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[4]     |WDTCKEN   |Watchdog Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[5]     |RTCCKEN   |Real-time- Sleep Clock APB Interface Clock Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[6]     |TMR0CKEN  |Timer0 Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[7]     |TMR1CKEN  |Timer1 Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[8]     |I2C0CKEN  |I2C0 Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[11]    |SPI1CHEN  |SPI1 Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[12]    |SPI0CKEN  |SPI0 Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[13]    |DPWMCKEN  |Differential PWM Speaker Driver Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[15]    |UART1CKEN |UART1 Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[16]    |UART0CKEN |UART0 Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[17]    |SARADCCKEN|SARADC Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[18]    |BIQALCKEN |BIQ and ALCSleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[20]    |PWM0CH01CKEN|PWM0CH0 and PWM0CH1 Block Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[21]    |PWM0CH23CKEN|PWM0CH2 and PWM0CH3 Block Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[28]    |SDADCCKEN |Delta-Sigma Analog-digital-converter (ADC) Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[29]    |I2SCKEN   |I2S Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[30]    |ANACKEN   |Analog Block Sleep Clock Enable Control
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * @var CLK_T::PWRSTSF
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
 * @var CLK_T::DBGPD
 * Offset: 0x28  Debug Port Power Down Disable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |DISPDREQ  |Disable Power Down
 * |        |          |0 = Enable power down requests.
 * |        |          |1 = Disable power down requests.
 * |[6]     |ICECLKST  |ICECLKST Pin State
 * |        |          |Read Only. Current state of ICE_CLK pin.
 * |[7]     |ICEDATST  |ICEDATST Pin State
 * |        |          |Read Only. Current state of ICE_DAT pin.
 * @var CLK_T::WAKE10K
 * Offset: 0x2C  Deep Power Down 10K Wakeup Timer
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[13:0]  |SELWKTMR  |Select Wakeup Timer
 * |        |          |WAKEUP after 64* (SELWKTMR+1) OSC10k clocks (6.4 * (SELWKTMR+1) ms)
 * |[29:16] |WKTMRSTS  |Current Wakeup Timer Setting
 * |        |          |Read-Only
 * |        |          |Read back of the current WAKEUP timer setting
 * |        |          |This value is updated with SELWKTMR upon entering DPD mode.
 * |[31]    |WAKE10KEN |Enable WAKE from DPD on 10kHz timer
 */
    __IO uint32_t PWRCTL;                /*!< [0x0000] System Power Control Register                                    */
    __IO uint32_t AHBCLK;                /*!< [0x0004] AHB Device Clock Enable Control Register                         */
    __IO uint32_t APBCLK0;               /*!< [0x0008] APB Device Clock Enable Control Register                         */
    __IO uint32_t DPDSTATE;              /*!< [0x000c] Deep Power Down State Register                                   */
    __IO uint32_t CLKSEL0;               /*!< [0x0010] Clock Source Select Control Register 0                           */
    __IO uint32_t CLKSEL1;               /*!< [0x0014] Clock Source Select Control Register 1                           */
    __IO uint32_t CLKDIV0;               /*!< [0x0018] Clock Divider Number Register                                    */
    __IO uint32_t CLKSEL2;               /*!< [0x001c] Clock Source Select Control Register 2                           */
    __IO uint32_t SLEEPCTL;              /*!< [0x0020] Sleep Clock Source Select  Register                              */
    __IO uint32_t PWRSTSF;               /*!< [0x0024] Power State Flag Register                                        */
    __IO uint32_t DBGPD;                 /*!< [0x0028] Debug Port Power Down Disable Register                           */
    __IO uint32_t WAKE10K;               /*!< [0x002c] Deep Power Down 10K Wakeup Timer                                 */

} CLK_T;

/**
    @addtogroup CLK_CONST CLK Bit Field Definition
    Constant Definitions for CLK Controller
@{ */

#define CLK_PWRCTL_LXTEN_Pos             (1)                                               /*!< CLK_T::PWRCTL: LXTEN Position          */
#define CLK_PWRCTL_LXTEN_Msk             (0x1ul << CLK_PWRCTL_LXTEN_Pos)                   /*!< CLK_T::PWRCTL: LXTEN Mask              */

#define CLK_PWRCTL_HIRCEN_Pos            (2)                                               /*!< CLK_T::PWRCTL: HIRCEN Position         */
#define CLK_PWRCTL_HIRCEN_Msk            (0x1ul << CLK_PWRCTL_HIRCEN_Pos)                  /*!< CLK_T::PWRCTL: HIRCEN Mask             */

#define CLK_PWRCTL_LIRCEN_Pos            (3)                                               /*!< CLK_T::PWRCTL: LIRCEN Position         */
#define CLK_PWRCTL_LIRCEN_Msk            (0x1ul << CLK_PWRCTL_LIRCEN_Pos)                  /*!< CLK_T::PWRCTL: LIRCEN Mask             */

#define CLK_PWRCTL_HXTEN_Pos             (4)                                               /*!< CLK_T::PWRCTL: HXTEN Position          */
#define CLK_PWRCTL_HXTEN_Msk             (0x1ul << CLK_PWRCTL_HXTEN_Pos)                   /*!< CLK_T::PWRCTL: HXTEN Mask              */

#define CLK_PWRCTL_STOPEN_Pos            (9)                                               /*!< CLK_T::PWRCTL: STOPEN Position         */
#define CLK_PWRCTL_STOPEN_Msk            (0x1ul << CLK_PWRCTL_STOPEN_Pos)                  /*!< CLK_T::PWRCTL: STOPEN Mask             */

#define CLK_PWRCTL_SPDEN_Pos             (10)                                              /*!< CLK_T::PWRCTL: SPDEN Position          */
#define CLK_PWRCTL_SPDEN_Msk             (0x1ul << CLK_PWRCTL_SPDEN_Pos)                   /*!< CLK_T::PWRCTL: SPDEN Mask              */

#define CLK_PWRCTL_DPDEN_Pos             (11)                                              /*!< CLK_T::PWRCTL: DPDEN Position          */
#define CLK_PWRCTL_DPDEN_Msk             (0x1ul << CLK_PWRCTL_DPDEN_Pos)                   /*!< CLK_T::PWRCTL: DPDEN Mask              */

#define CLK_PWRCTL_HOLDIO_Pos            (12)                                              /*!< CLK_T::PWRCTL: HOLDIO Position         */
#define CLK_PWRCTL_HOLDIO_Msk            (0x1ul << CLK_PWRCTL_HOLDIO_Pos)                  /*!< CLK_T::PWRCTL: HOLDIO Mask             */

#define CLK_PWRCTL_RELEASEIO_Pos         (13)                                              /*!< CLK_T::PWRCTL: RELEASEIO Position      */
#define CLK_PWRCTL_RELEASEIO_Msk         (0x1ul << CLK_PWRCTL_RELEASEIO_Pos)               /*!< CLK_T::PWRCTL: RELEASEIO Mask          */

#define CLK_PWRCTL_IOSTATE_Pos           (14)                                              /*!< CLK_T::PWRCTL: IOSTATE Position        */
#define CLK_PWRCTL_IOSTATE_Msk           (0x1ul << CLK_PWRCTL_IOSTATE_Pos)                 /*!< CLK_T::PWRCTL: IOSTATE Mask            */

#define CLK_PWRCTL_WKPINEN_Pos           (16)                                              /*!< CLK_T::PWRCTL: WKPINEN Position        */
#define CLK_PWRCTL_WKPINEN_Msk           (0x1ul << CLK_PWRCTL_WKPINEN_Pos)                 /*!< CLK_T::PWRCTL: WKPINEN Mask            */

#define CLK_PWRCTL_LIRCDPDEN_Pos         (17)                                              /*!< CLK_T::PWRCTL: LIRCDPDEN Position      */
#define CLK_PWRCTL_LIRCDPDEN_Msk         (0x1ul << CLK_PWRCTL_LIRCDPDEN_Pos)               /*!< CLK_T::PWRCTL: LIRCDPDEN Mask          */

#define CLK_PWRCTL_FLASHEN_Pos           (18)                                              /*!< CLK_T::PWRCTL: FLASHEN Position        */
#define CLK_PWRCTL_FLASHEN_Msk           (0x3ul << CLK_PWRCTL_FLASHEN_Pos)                 /*!< CLK_T::PWRCTL: FLASHEN Mask            */

#define CLK_PWRCTL_WKPINWKF_Pos          (24)                                              /*!< CLK_T::PWRCTL: WKPINWKF Position       */
#define CLK_PWRCTL_WKPINWKF_Msk          (0x1ul << CLK_PWRCTL_WKPINWKF_Pos)                /*!< CLK_T::PWRCTL: WKPINWKF Mask           */

#define CLK_PWRCTL_TMRWKF_Pos            (25)                                              /*!< CLK_T::PWRCTL: TMRWKF Position         */
#define CLK_PWRCTL_TMRWKF_Msk            (0x1ul << CLK_PWRCTL_TMRWKF_Pos)                  /*!< CLK_T::PWRCTL: TMRWKF Mask             */

#define CLK_PWRCTL_PORWKF_Pos            (26)                                              /*!< CLK_T::PWRCTL: PORWKF Position         */
#define CLK_PWRCTL_PORWKF_Msk            (0x1ul << CLK_PWRCTL_PORWKF_Pos)                  /*!< CLK_T::PWRCTL: PORWKF Mask             */

#define CLK_PWRCTL_WKPUEN_Pos            (27)                                              /*!< CLK_T::PWRCTL: WKPUEN Position         */
#define CLK_PWRCTL_WKPUEN_Msk            (0x1ul << CLK_PWRCTL_WKPUEN_Pos)                  /*!< CLK_T::PWRCTL: WKPUEN Mask             */

#define CLK_AHBCLK_HCLKEN_Pos            (0)                                               /*!< CLK_T::AHBCLK: HCLKEN Position         */
#define CLK_AHBCLK_HCLKEN_Msk            (0x1ul << CLK_AHBCLK_HCLKEN_Pos)                  /*!< CLK_T::AHBCLK: HCLKEN Mask             */

#define CLK_AHBCLK_PDMACKEN_Pos          (1)                                               /*!< CLK_T::AHBCLK: PDMACKEN Position       */
#define CLK_AHBCLK_PDMACKEN_Msk          (0x1ul << CLK_AHBCLK_PDMACKEN_Pos)                /*!< CLK_T::AHBCLK: PDMACKEN Mask           */

#define CLK_AHBCLK_ISPCKEN_Pos           (2)                                               /*!< CLK_T::AHBCLK: ISPCKEN Position        */
#define CLK_AHBCLK_ISPCKEN_Msk           (0x1ul << CLK_AHBCLK_ISPCKEN_Pos)                 /*!< CLK_T::AHBCLK: ISPCKEN Mask            */

#define CLK_APBCLK0_WDTCKEN_Pos          (0)                                               /*!< CLK_T::APBCLK0: WDTCKEN Position       */
#define CLK_APBCLK0_WDTCKEN_Msk          (0x1ul << CLK_APBCLK0_WDTCKEN_Pos)                /*!< CLK_T::APBCLK0: WDTCKEN Mask           */

#define CLK_APBCLK0_RTCCKEN_Pos          (5)                                               /*!< CLK_T::APBCLK0: RTCCKEN Position       */
#define CLK_APBCLK0_RTCCKEN_Msk          (0x1ul << CLK_APBCLK0_RTCCKEN_Pos)                /*!< CLK_T::APBCLK0: RTCCKEN Mask           */

#define CLK_APBCLK0_TMR0CKEN_Pos         (6)                                               /*!< CLK_T::APBCLK0: TMR0CKEN Position      */
#define CLK_APBCLK0_TMR0CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR0CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR0CKEN Mask          */

#define CLK_APBCLK0_TMR1CKEN_Pos         (7)                                               /*!< CLK_T::APBCLK0: TMR1CKEN Position      */
#define CLK_APBCLK0_TMR1CKEN_Msk         (0x1ul << CLK_APBCLK0_TMR1CKEN_Pos)               /*!< CLK_T::APBCLK0: TMR1CKEN Mask          */

#define CLK_APBCLK0_I2C0CKEN_Pos         (8)                                               /*!< CLK_T::APBCLK0: I2C0CKEN Position      */
#define CLK_APBCLK0_I2C0CKEN_Msk         (0x1ul << CLK_APBCLK0_I2C0CKEN_Pos)               /*!< CLK_T::APBCLK0: I2C0CKEN Mask          */

#define CLK_APBCLK0_SPI1CKEN_Pos         (11)                                              /*!< CLK_T::APBCLK0: SPI1CKEN Position      */
#define CLK_APBCLK0_SPI1CKEN_Msk         (0x1ul << CLK_APBCLK0_SPI1CKEN_Pos)               /*!< CLK_T::APBCLK0: SPI1CKEN Mask          */

#define CLK_APBCLK0_SPI0CKEN_Pos         (12)                                              /*!< CLK_T::APBCLK0: SPI0CKEN Position      */
#define CLK_APBCLK0_SPI0CKEN_Msk         (0x1ul << CLK_APBCLK0_SPI0CKEN_Pos)               /*!< CLK_T::APBCLK0: SPI0CKEN Mask          */

#define CLK_APBCLK0_DPWMCKEN_Pos         (13)                                              /*!< CLK_T::APBCLK0: DPWMCKEN Position      */
#define CLK_APBCLK0_DPWMCKEN_Msk         (0x1ul << CLK_APBCLK0_DPWMCKEN_Pos)               /*!< CLK_T::APBCLK0: DPWMCKEN Mask          */

#define CLK_APBCLK0_UART1CKEN_Pos        (15)                                              /*!< CLK_T::APBCLK0: UART1CKEN Position     */
#define CLK_APBCLK0_UART1CKEN_Msk        (0x1ul << CLK_APBCLK0_UART1CKEN_Pos)              /*!< CLK_T::APBCLK0: UART1CKEN Mask         */

#define CLK_APBCLK0_UART0CKEN_Pos        (16)                                              /*!< CLK_T::APBCLK0: UART0CKEN Position     */
#define CLK_APBCLK0_UART0CKEN_Msk        (0x1ul << CLK_APBCLK0_UART0CKEN_Pos)              /*!< CLK_T::APBCLK0: UART0CKEN Mask         */

#define CLK_APBCLK0_SARADCKEN_Pos        (17)                                              /*!< CLK_T::APBCLK0: SARADCKEN Position     */
#define CLK_APBCLK0_SARADCKEN_Msk        (0x1ul << CLK_APBCLK0_SARADCKEN_Pos)              /*!< CLK_T::APBCLK0: SARADCKEN Mask         */

#define CLK_APBCLK0_BIQALCKEN_Pos        (18)                                              /*!< CLK_T::APBCLK0: BIQALCKEN Position     */
#define CLK_APBCLK0_BIQALCKEN_Msk        (0x1ul << CLK_APBCLK0_BIQALCKEN_Pos)              /*!< CLK_T::APBCLK0: BIQALCKEN Mask         */

#define CLK_APBCLK0_PWM0CH01CKEN_Pos     (20)                                              /*!< CLK_T::APBCLK0: PWM0CH01CKEN Position  */
#define CLK_APBCLK0_PWM0CH01CKEN_Msk     (0x1ul << CLK_APBCLK0_PWM0CH01CKEN_Pos)           /*!< CLK_T::APBCLK0: PWM0CH01CKEN Mask      */

#define CLK_APBCLK0_PWM0CH23CKEN_Pos     (21)                                              /*!< CLK_T::APBCLK0: PWM0CH23CKEN Position  */
#define CLK_APBCLK0_PWM0CH23CKEN_Msk     (0x1ul << CLK_APBCLK0_PWM0CH23CKEN_Pos)           /*!< CLK_T::APBCLK0: PWM0CH23CKEN Mask      */

#define CLK_APBCLK0_SDADCCKEN_Pos        (28)                                              /*!< CLK_T::APBCLK0: SDADCCKEN Position     */
#define CLK_APBCLK0_SDADCCKEN_Msk        (0x1ul << CLK_APBCLK0_SDADCCKEN_Pos)              /*!< CLK_T::APBCLK0: SDADCCKEN Mask         */

#define CLK_APBCLK0_I2S0CKEN_Pos         (29)                                              /*!< CLK_T::APBCLK0: I2S0CKEN Position      */
#define CLK_APBCLK0_I2S0CKEN_Msk         (0x1ul << CLK_APBCLK0_I2S0CKEN_Pos)               /*!< CLK_T::APBCLK0: I2S0CKEN Mask          */

#define CLK_APBCLK0_ANACKEN_Pos          (30)                                              /*!< CLK_T::APBCLK0: ANACKEN Position       */
#define CLK_APBCLK0_ANACKEN_Msk          (0x1ul << CLK_APBCLK0_ANACKEN_Pos)                /*!< CLK_T::APBCLK0: ANACKEN Mask           */

#define CLK_DPDSTATE_DPDSTSWR_Pos        (0)                                               /*!< CLK_T::DPDSTATE: DPDSTSWR Position     */
#define CLK_DPDSTATE_DPDSTSWR_Msk        (0xfful << CLK_DPDSTATE_DPDSTSWR_Pos)             /*!< CLK_T::DPDSTATE: DPDSTSWR Mask         */

#define CLK_DPDSTATE_DPDSTSRD_Pos        (8)                                               /*!< CLK_T::DPDSTATE: DPDSTSRD Position     */
#define CLK_DPDSTATE_DPDSTSRD_Msk        (0xfful << CLK_DPDSTATE_DPDSTSRD_Pos)             /*!< CLK_T::DPDSTATE: DPDSTSRD Mask         */

#define CLK_CLKSEL0_HCLKSEL_Pos          (0)                                               /*!< CLK_T::CLKSEL0: HCLKSEL Position       */
#define CLK_CLKSEL0_HCLKSEL_Msk          (0x7ul << CLK_CLKSEL0_HCLKSEL_Pos)                /*!< CLK_T::CLKSEL0: HCLKSEL Mask           */

#define CLK_CLKSEL0_STCLKSEL_Pos         (3)                                               /*!< CLK_T::CLKSEL0: STCLKSEL Position      */
#define CLK_CLKSEL0_STCLKSEL_Msk         (0x7ul << CLK_CLKSEL0_STCLKSEL_Pos)               /*!< CLK_T::CLKSEL0: STCLKSEL Mask          */

#define CLK_CLKSEL0_HIRCFSEL_Pos         (6)                                               /*!< CLK_T::CLKSEL0: HIRCFSEL Position      */
#define CLK_CLKSEL0_HIRCFSEL_Msk         (0x3ul << CLK_CLKSEL0_HIRCFSEL_Pos)               /*!< CLK_T::CLKSEL0: HIRCFSEL Mask          */

#define CLK_CLKSEL1_WDTSEL_Pos           (0)                                               /*!< CLK_T::CLKSEL1: WDTSEL Position        */
#define CLK_CLKSEL1_WDTSEL_Msk           (0x3ul << CLK_CLKSEL1_WDTSEL_Pos)                 /*!< CLK_T::CLKSEL1: WDTSEL Mask            */

#define CLK_CLKSEL1_SDADCSEL_Pos         (2)                                               /*!< CLK_T::CLKSEL1: SDADCSEL Position      */
#define CLK_CLKSEL1_SDADCSEL_Msk         (0x3ul << CLK_CLKSEL1_SDADCSEL_Pos)               /*!< CLK_T::CLKSEL1: SDADCSEL Mask          */

#define CLK_CLKSEL1_DPWMSEL_Pos          (4)                                               /*!< CLK_T::CLKSEL1: DPWMSEL Position       */
#define CLK_CLKSEL1_DPWMSEL_Msk          (0x3ul << CLK_CLKSEL1_DPWMSEL_Pos)                /*!< CLK_T::CLKSEL1: DPWMSEL Mask           */

#define CLK_CLKSEL1_TMR0SEL_Pos          (8)                                               /*!< CLK_T::CLKSEL1: TMR0SEL Position       */
#define CLK_CLKSEL1_TMR0SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR0SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR0SEL Mask           */

#define CLK_CLKSEL1_TMR1SEL_Pos          (12)                                              /*!< CLK_T::CLKSEL1: TMR1SEL Position       */
#define CLK_CLKSEL1_TMR1SEL_Msk          (0x7ul << CLK_CLKSEL1_TMR1SEL_Pos)                /*!< CLK_T::CLKSEL1: TMR1SEL Mask           */

#define CLK_CLKSEL1_SARADCSEL_Pos        (24)                                              /*!< CLK_T::CLKSEL1: SARADCSEL Position     */
#define CLK_CLKSEL1_SARADCSEL_Msk        (0x3ul << CLK_CLKSEL1_SARADCSEL_Pos)              /*!< CLK_T::CLKSEL1: SARADCSEL Mask         */

#define CLK_CLKSEL1_PWM0CH01SEL_Pos      (28)                                              /*!< CLK_T::CLKSEL1: PWM0CH01SEL Position   */
#define CLK_CLKSEL1_PWM0CH01SEL_Msk      (0x3ul << CLK_CLKSEL1_PWM0CH01SEL_Pos)            /*!< CLK_T::CLKSEL1: PWM0CH01SEL Mask       */

#define CLK_CLKSEL1_PWM0CH23SEL_Pos      (30)                                              /*!< CLK_T::CLKSEL1: PWM0CH23SEL Position   */
#define CLK_CLKSEL1_PWM0CH23SEL_Msk      (0x3ul << CLK_CLKSEL1_PWM0CH23SEL_Pos)            /*!< CLK_T::CLKSEL1: PWM0CH23SEL Mask       */

#define CLK_CLKDIV0_HCLKDIV_Pos          (0)                                               /*!< CLK_T::CLKDIV0: HCLKDIV Position       */
#define CLK_CLKDIV0_HCLKDIV_Msk          (0xful << CLK_CLKDIV0_HCLKDIV_Pos)                /*!< CLK_T::CLKDIV0: HCLKDIV Mask           */

#define CLK_CLKDIV0_BIQDIV_Pos           (4)                                               /*!< CLK_T::CLKDIV0: BIQDIV Position        */
#define CLK_CLKDIV0_BIQDIV_Msk           (0xful << CLK_CLKDIV0_BIQDIV_Pos)                 /*!< CLK_T::CLKDIV0: BIQDIV Mask            */

#define CLK_CLKDIV0_UART0DIV_Pos         (8)                                               /*!< CLK_T::CLKDIV0: UART0DIV Position       */
#define CLK_CLKDIV0_UART0DIV_Msk         (0xful << CLK_CLKDIV0_UART0DIV_Pos)               /*!< CLK_T::CLKDIV0: UART0DIV Mask           */

#define CLK_CLKDIV0_DPWMDIV_Pos          (12)                                              /*!< CLK_T::CLKDIV0: DPWMDIV Position       */
#define CLK_CLKDIV0_DPWMDIV_Msk          (0xful << CLK_CLKDIV0_DPWMDIV_Pos)                /*!< CLK_T::CLKDIV0: DPWMDIV Mask           */

#define CLK_CLKDIV0_SDADCDIV_Pos         (16)                                              /*!< CLK_T::CLKDIV0: SDADCDIV Position      */
#define CLK_CLKDIV0_SDADCDIV_Msk         (0xfful << CLK_CLKDIV0_SDADCDIV_Pos)              /*!< CLK_T::CLKDIV0: SDADCDIV Mask          */

#define CLK_CLKDIV0_SARADCDIV_Pos        (24)                                              /*!< CLK_T::CLKDIV0: SARADCDIV Position     */
#define CLK_CLKDIV0_SARADCDIV_Msk        (0xfful << CLK_CLKDIV0_SARADCDIV_Pos)             /*!< CLK_T::CLKDIV0: SARADCDIV Mask         */

#define CLK_CLKSEL2_I2S0SEL_Pos          (0)                                               /*!< CLK_T::CLKSEL2: I2S0SEL Position       */
#define CLK_CLKSEL2_I2S0SEL_Msk          (0x3ul << CLK_CLKSEL2_I2S0SEL_Pos)                /*!< CLK_T::CLKSEL2: I2S0SEL Mask           */

#define CLK_CLKSEL2_UART1DIV_Pos         (8)                                               /*!< CLK_T::CLKSEL2: UART1DIV Position      */
#define CLK_CLKSEL2_UART1DIV_Msk         (0xful << CLK_CLKSEL2_UART1DIV_Pos)               /*!< CLK_T::CLKSEL2: UART1DIV Mask          */

#define CLK_SLEEPCTL_HCLKCKEN_Pos        (0)                                               /*!< CLK_T::SLEEPCTL: HCLKCKEN Position     */
#define CLK_SLEEPCTL_HCLKCKEN_Msk        (0x1ul << CLK_SLEEPCTL_HCLKCKEN_Pos)              /*!< CLK_T::SLEEPCTL: HCLKCKEN Mask         */

#define CLK_SLEEPCTL_PDMACKEN_Pos        (1)                                               /*!< CLK_T::SLEEPCTL: PDMACKEN Position     */
#define CLK_SLEEPCTL_PDMACKEN_Msk        (0x1ul << CLK_SLEEPCTL_PDMACKEN_Pos)              /*!< CLK_T::SLEEPCTL: PDMACKEN Mask         */

#define CLK_SLEEPCTL_ISPCKEN_Pos         (2)                                               /*!< CLK_T::SLEEPCTL: ISPCKEN Position      */
#define CLK_SLEEPCTL_ISPCKEN_Msk         (0x1ul << CLK_SLEEPCTL_ISPCKEN_Pos)               /*!< CLK_T::SLEEPCTL: ISPCKEN Mask          */

#define CLK_SLEEPCTL_WDTCKEN_Pos         (4)                                               /*!< CLK_T::SLEEPCTL: WDTCKEN Position      */
#define CLK_SLEEPCTL_WDTCKEN_Msk         (0x1ul << CLK_SLEEPCTL_WDTCKEN_Pos)               /*!< CLK_T::SLEEPCTL: WDTCKEN Mask          */

#define CLK_SLEEPCTL_RTCCKEN_Pos         (5)                                               /*!< CLK_T::SLEEPCTL: RTCCKEN Position      */
#define CLK_SLEEPCTL_RTCCKEN_Msk         (0x1ul << CLK_SLEEPCTL_RTCCKEN_Pos)               /*!< CLK_T::SLEEPCTL: RTCCKEN Mask          */

#define CLK_SLEEPCTL_TMR0CKEN_Pos        (6)                                               /*!< CLK_T::SLEEPCTL: TMR0CKEN Position     */
#define CLK_SLEEPCTL_TMR0CKEN_Msk        (0x1ul << CLK_SLEEPCTL_TMR0CKEN_Pos)              /*!< CLK_T::SLEEPCTL: TMR0CKEN Mask         */

#define CLK_SLEEPCTL_TMR1CKEN_Pos        (7)                                               /*!< CLK_T::SLEEPCTL: TMR1CKEN Position     */
#define CLK_SLEEPCTL_TMR1CKEN_Msk        (0x1ul << CLK_SLEEPCTL_TMR1CKEN_Pos)              /*!< CLK_T::SLEEPCTL: TMR1CKEN Mask         */

#define CLK_SLEEPCTL_I2C0CKEN_Pos        (8)                                               /*!< CLK_T::SLEEPCTL: I2C0CKEN Position     */
#define CLK_SLEEPCTL_I2C0CKEN_Msk        (0x1ul << CLK_SLEEPCTL_I2C0CKEN_Pos)              /*!< CLK_T::SLEEPCTL: I2C0CKEN Mask         */

#define CLK_SLEEPCTL_SPI1CHEN_Pos        (11)                                              /*!< CLK_T::SLEEPCTL: SPI1CHEN Position     */
#define CLK_SLEEPCTL_SPI1CHEN_Msk        (0x1ul << CLK_SLEEPCTL_SPI1CHEN_Pos)              /*!< CLK_T::SLEEPCTL: SPI1CHEN Mask         */

#define CLK_SLEEPCTL_SPI0CKEN_Pos        (12)                                              /*!< CLK_T::SLEEPCTL: SPI0CKEN Position     */
#define CLK_SLEEPCTL_SPI0CKEN_Msk        (0x1ul << CLK_SLEEPCTL_SPI0CKEN_Pos)              /*!< CLK_T::SLEEPCTL: SPI0CKEN Mask         */

#define CLK_SLEEPCTL_DPWMCKEN_Pos        (13)                                              /*!< CLK_T::SLEEPCTL: DPWMCKEN Position     */
#define CLK_SLEEPCTL_DPWMCKEN_Msk        (0x1ul << CLK_SLEEPCTL_DPWMCKEN_Pos)              /*!< CLK_T::SLEEPCTL: DPWMCKEN Mask         */

#define CLK_SLEEPCTL_UART1CKEN_Pos       (15)                                              /*!< CLK_T::SLEEPCTL: UART1CKEN Position    */
#define CLK_SLEEPCTL_UART1CKEN_Msk       (0x1ul << CLK_SLEEPCTL_UART1CKEN_Pos)             /*!< CLK_T::SLEEPCTL: UART1CKEN Mask        */

#define CLK_SLEEPCTL_UART0CKEN_Pos       (16)                                              /*!< CLK_T::SLEEPCTL: UART0CKEN Position    */
#define CLK_SLEEPCTL_UART0CKEN_Msk       (0x1ul << CLK_SLEEPCTL_UART0CKEN_Pos)             /*!< CLK_T::SLEEPCTL: UART0CKEN Mask        */

#define CLK_SLEEPCTL_SARADCCKEN_Pos      (17)                                              /*!< CLK_T::SLEEPCTL: SARADCCKEN Position   */
#define CLK_SLEEPCTL_SARADCCKEN_Msk      (0x1ul << CLK_SLEEPCTL_SARADCCKEN_Pos)            /*!< CLK_T::SLEEPCTL: SARADCCKEN Mask       */

#define CLK_SLEEPCTL_BIQALCKEN_Pos       (18)                                              /*!< CLK_T::SLEEPCTL: BIQALCKEN Position    */
#define CLK_SLEEPCTL_BIQALCKEN_Msk       (0x1ul << CLK_SLEEPCTL_BIQALCKEN_Pos)             /*!< CLK_T::SLEEPCTL: BIQALCKEN Mask        */

#define CLK_SLEEPCTL_PWM0CH01CKEN_Pos    (20)                                              /*!< CLK_T::SLEEPCTL: PWM0CH01CKEN Position */
#define CLK_SLEEPCTL_PWM0CH01CKEN_Msk    (0x1ul << CLK_SLEEPCTL_PWM0CH01CKEN_Pos)          /*!< CLK_T::SLEEPCTL: PWM0CH01CKEN Mask     */

#define CLK_SLEEPCTL_PWM0CH23CKEN_Pos    (21)                                              /*!< CLK_T::SLEEPCTL: PWM0CH23CKEN Position */
#define CLK_SLEEPCTL_PWM0CH23CKEN_Msk    (0x1ul << CLK_SLEEPCTL_PWM0CH23CKEN_Pos)          /*!< CLK_T::SLEEPCTL: PWM0CH23CKEN Mask     */

#define CLK_SLEEPCTL_SDADCCKEN_Pos       (28)                                              /*!< CLK_T::SLEEPCTL: SDADCCKEN Position    */
#define CLK_SLEEPCTL_SDADCCKEN_Msk       (0x1ul << CLK_SLEEPCTL_SDADCCKEN_Pos)             /*!< CLK_T::SLEEPCTL: SDADCCKEN Mask        */

#define CLK_SLEEPCTL_I2SCKEN_Pos         (29)                                              /*!< CLK_T::SLEEPCTL: I2SCKEN Position      */
#define CLK_SLEEPCTL_I2SCKEN_Msk         (0x1ul << CLK_SLEEPCTL_I2SCKEN_Pos)               /*!< CLK_T::SLEEPCTL: I2SCKEN Mask          */

#define CLK_SLEEPCTL_ANACKEN_Pos         (30)                                              /*!< CLK_T::SLEEPCTL: ANACKEN Position      */
#define CLK_SLEEPCTL_ANACKEN_Msk         (0x1ul << CLK_SLEEPCTL_ANACKEN_Pos)               /*!< CLK_T::SLEEPCTL: ANACKEN Mask          */

#define CLK_PWRSTSF_DSF_Pos              (0)                                               /*!< CLK_T::PWRSTSF: DSF Position           */
#define CLK_PWRSTSF_DSF_Msk              (0x1ul << CLK_PWRSTSF_DSF_Pos)                    /*!< CLK_T::PWRSTSF: DSF Mask               */

#define CLK_PWRSTSF_STOPF_Pos            (1)                                               /*!< CLK_T::PWRSTSF: STOPF Position         */
#define CLK_PWRSTSF_STOPF_Msk            (0x1ul << CLK_PWRSTSF_STOPF_Pos)                  /*!< CLK_T::PWRSTSF: STOPF Mask             */

#define CLK_PWRSTSF_SPDF_Pos             (2)                                               /*!< CLK_T::PWRSTSF: SPDF Position          */
#define CLK_PWRSTSF_SPDF_Msk             (0x1ul << CLK_PWRSTSF_SPDF_Pos)                   /*!< CLK_T::PWRSTSF: SPDF Mask              */

#define CLK_DBGPD_DISPDREQ_Pos           (0)                                               /*!< CLK_T::DBGPD: DISPDREQ Position        */
#define CLK_DBGPD_DISPDREQ_Msk           (0x1ul << CLK_DBGPD_DISPDREQ_Pos)                 /*!< CLK_T::DBGPD: DISPDREQ Mask            */

#define CLK_DBGPD_ICECLKST_Pos           (6)                                               /*!< CLK_T::DBGPD: ICECLKST Position        */
#define CLK_DBGPD_ICECLKST_Msk           (0x1ul << CLK_DBGPD_ICECLKST_Pos)                 /*!< CLK_T::DBGPD: ICECLKST Mask            */

#define CLK_DBGPD_ICEDATST_Pos           (7)                                               /*!< CLK_T::DBGPD: ICEDATST Position        */
#define CLK_DBGPD_ICEDATST_Msk           (0x1ul << CLK_DBGPD_ICEDATST_Pos)                 /*!< CLK_T::DBGPD: ICEDATST Mask            */

#define CLK_WAKE10K_SELWKTMR_Pos         (0)                                               /*!< CLK_T::WAKE10K: SELWKTMR Position      */
#define CLK_WAKE10K_SELWKTMR_Msk         (0x3ffful << CLK_WAKE10K_SELWKTMR_Pos)            /*!< CLK_T::WAKE10K: SELWKTMR Mask          */

#define CLK_WAKE10K_WKTMRSTS_Pos         (16)                                              /*!< CLK_T::WAKE10K: WKTMRSTS Position      */
#define CLK_WAKE10K_WKTMRSTS_Msk         (0x3ffful << CLK_WAKE10K_WKTMRSTS_Pos)            /*!< CLK_T::WAKE10K: WKTMRSTS Mask          */

#define CLK_WAKE10K_WAKE10KEN_Pos        (31)                                              /*!< CLK_T::WAKE10K: WAKE10KEN Position     */
#define CLK_WAKE10K_WAKE10KEN_Msk        (0x1ul << CLK_WAKE10K_WAKE10KEN_Pos)              /*!< CLK_T::WAKE10K: WAKE10KEN Mask         */

/**@}*/ /* CLK_CONST */
/**@}*/ /* end of CLK register group */


/*---------------------- CAP SENSE Controller ----------------------------------------*/
/**
    @addtogroup CSCAN CAP SENSE Controller(CSCAN)
    Memory Mapped Structure for CSCAN Controller
@{ */
 
typedef struct
{


/**
 * @var CSCAN_T::CTRL
 * Offset: 0x00  CSCAN Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |SEL       |CSCAN Select
 * |        |          |In single mode selects the channel (GPIOB[15:0]) to perform measurement on.
 * |[17:16] |CURRENT   |CSCAN Oscillator current
 * |        |          |Controls the analog bais current of the capacitive relaxation oscillator.
 * |        |          |0:300nA 1:450nA 2:600nA 3:1200nA
 * |[20]    |INT_EN    |CSCAN Enable Interrupt
 * |        |          |0 = Interrupt disabled.
 * |        |          |1 = Interrupt enabled.
 * |[21]    |SLOW_CLK  |CSCAN Slow Clock
 * |        |          |0 = Timebase clock is HIRC.
 * |        |          |1 = Timebase clock is LIRC (XTAL32K_EN = 0) or XTAL (XTAL32K_EN = 1)
 * |        |          |**Notes:
 * |        |          |In low speed mode, for CYCLE_CNT <5, the minimum frequency of oscillation of a CAPSENSE GPIO must be > Fclk/2
 * |        |          |Where Fclk is the frequency of LXT or LIRC depending which is selected as reference.
 * |[22]    |MODE0     |CSCAN Mode0
 * |        |          |0 = Single shot Capacitive sense
 * |        |          |1 = Scans each channel set in SCAN_MASK and stores in RAM.
 * |[23]    |MODE1     |CSCAN Mode1
 * |        |          |0 = Interrupt when scan finished
 * |        |          |1 = Interrupt when DUR_CNT delay occurs.
 * |[27:24] |DUR_CNT   |CSCAN Duration Count
 * |        |          |This counter is used to set a wakeup time after a capacitive sensing scan is complete
 * |        |          |It is in units of low frewquency clock period (either LXT or LIRC clock) and gives delay of 160, 320, 480,640, 800, 960, 1120, 1280, 1440,1600, 1920, 2240, 2560, 2880,3200 3840 periods for settings 0,..,15.
 * |[30]    |EN        |CSCAN Enable
 * |        |          |Write 1 to start. Reset by hardware when operation finished.
 * |[31]    |PD        |Power Down
 * |        |          |0: Enable analog circuit
 * |        |          |1: Power down analog circuit and block.
 * @var CSCAN_T::CYCCNT
 * Offset: 0x04  CSCAN Cycle Count  Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |CYCLE_CNT |CSCAN Cycle Count
 * |        |          |Number of cycles to time a CapSense even over 4 bit value decoded to 1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 768, 1024, 1536, 2048, 2560, 3072
 * |        |          |CYCLE_CNT
 * |        |          |0x0
 * |        |          |0x1
 * |        |          |0x2
 * |        |          |0x3
 * |        |          |0x4
 * |        |          |0x5
 * |        |          |0x6
 * |        |          |0x7
 * |        |          |Cycles
 * |        |          |1
 * |        |          |2
 * |        |          |4
 * |        |          |8
 * |        |          |16
 * |        |          |32
 * |        |          |64
 * |        |          |128
 * |        |          |CYCLE_CNT
 * |        |          |0x8
 * |        |          |0x9
 * |        |          |0xA
 * |        |          |0xB
 * |        |          |0xC
 * |        |          |0xD
 * |        |          |0xE
 * |        |          |0xF
 * |        |          |Cycles
 * |        |          |256
 * |        |          |512
 * |        |          |768
 * |        |          |1024
 * |        |          |1536
 * |        |          |2048
 * |        |          |2560
 * |        |          |3072
 * |        |          |Notes:
 * |        |          |u00B7 It is recommended to use CYCLE_CNT >= 5 (32 cycles or more) when SLOW_CLK = 1
 * |        |          |Inappropriate CSCAN_CTRL.CURRENT settings with short scan cyle (CYCLE_CNT <= 4) might disrupt Capsense Interrupt and power mode wake up.
 * |        |          |u00B7 It is recommended to check result of CYCLE_CNT > 2 (4 cycles ) when SLOW_CLK = 0 to avoid overflow the scan counter.
 * |[31:16] |MASK      |Scan Mask Register
 * |        |          |If MASK[n] is set then GPIOB[n] is included in scan of capacitive sensing.
 * @var CSCAN_T::COUNT
 * Offset: 0x08  CSCAN Count Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |COUNT     |CSCAN Count
 * |        |          |Count result of single scan.
 * @var CSCAN_T::INT
 * Offset: 0x0C  CSCAN Interrupt Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |INT       |CSCAN Interrupt active
 * |        |          |Write u20181u2019 to clear.
 * @var CSCAN_T::AGPIO
 * Offset: 0x10  CSCAN Analog GPIO Function Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |AGPIO     |CSCAN AGPIO
 * |        |          |If bit set to 1 then corresponding GPIOB[n] is forced to an analog mode where digital input, output and pullup is disabled
 * |        |          |Can be used to set pad into analog mode for CapSensing, SAR ADC and OPAMP functions.
 * @var CSCAN_T::OPACTL
 * Offset: 0x14  Operational Amplifier Control  Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |A0EN      |OPA0 Enable or Disable Control Bit
 * |        |          |0 = disable.
 * |        |          |1 = enable.
 * |[1]     |A0OEN     |OPA0 Output Enable or Disable Control Bit
 * |        |          |0 = disable.
 * |        |          |1 = enable.
 * |[2]     |A0NS      |A0N Pin to OPA0 Inverting Input Control Bit
 * |        |          |0 = no connection.
 * |        |          |1 = from A0N pin.
 * |[3]     |A0PS      |A0P Pin to OPA0 Non-inverting Input Control Bit
 * |        |          |0 = no connection.
 * |        |          |1 = from A0P pin.
 * |[5:4]   |A0PSEL    |OPA0 Non-inverting Input Selection Bit
 * |        |          |00 = no connection.
 * |        |          |01 = from VH1 (0.9u00D7VDDA).
 * |        |          |10 = from VM (0.5u00D7VDDA).
 * |        |          |11 =: from VL1 (0.1u00D7VDDA).
 * |[6]     |A0O2N     |OPA0 Output to OPA0 Inverting Input Control Bit
 * |        |          |0 = disable.
 * |        |          |1 = enable.
 * |[7]     |A0X       |Operational amplifier 0 output; positive logic This bit is read only
 * |[8]     |A1EN      |OPA1 Enable or Disable Control Bit
 * |        |          |0 = disable.
 * |        |          |1 = enable.
 * |[9]     |A1OEN     |OPA1 Output Enable or Disable Control Bit
 * |        |          |0 = disable.
 * |        |          |1 = enable.
 * |[10]    |A1NS      |A1N Pin to OPA1 Inverting Input Control Bit
 * |        |          |0 = no connection.
 * |        |          |1 = from A0N pin.
 * |[11]    |A1PS      |A1P Pin to OPA1 Non-inverting Input Control Bit
 * |        |          |0 = no connection.
 * |        |          |1 = from A0P pin.
 * |[13:12] |A1PSEL    |OPA1 Non-inverting Input Selection Bit
 * |        |          |00 = no connection.
 * |        |          |01 = from VH1 (0.9u00D7VDDA).
 * |        |          |10 = from VM (0.5u00D7VDDA ).
 * |        |          |11 =: from VL1 (0.1u00D7VDDA).
 * |[14]    |A102N     |OPA1 Output to OPA1 Inverting Input Control Bit
 * |        |          |0 = disable.
 * |        |          |1 = enable.
 * |[15]    |A1X       |Operational amplifier 1 output; positive logic This bit is read only
 * |[18:16] |PGA       |OPA1 Gain Control Bits
 * |        |          |000 = 1.
 * |        |          |001 = 8.
 * |        |          |010 = 16.
 * |        |          |011 = 24.
 * |        |          |100 = 32.
 * |        |          |101 = 40.
 * |        |          |110 = 48.
 * |        |          |111 = 56.
 * |[19]    |PGAEN     |OPA1 PGA Gain Enable Control Bits
 * |        |          |0 = disable.
 * |        |          |1 = Enable.
 * |[20]    |VREFEN    |Enable OPA and Comparator Reference Voltage Generator
 * |        |          |0 = disable.
 * |        |          |1 = enable.
 * |[22]    |A0O2A1P   |OPA0 Output to OPA1 Non-inverting Input Control Bit
 * |        |          |0 = disable.
 * |        |          |1 = enable.
 * |[23]    |A0O2A1N   |OPA0 Output to OPA0 Inverting Input Control Bit
 * |        |          |0 = disable.
 * |        |          |1 = enable.
 * |[24]    |LPWREN    |Enable Opamps in STOP/SPD modes
 * |        |          |0 = disable.
 * |        |          |1 = enable.
 * |[25]    |A0O2CIN   |OPA0 output to comparator input control bit
 * |        |          |0= disable
 * |        |          |1= enable
 * |[26]    |A1O2CIN   |OPA1 output to comparator input control bit
 * |        |          |0= disable
 * |        |          |1= enable
 * @var CSCAN_T::CMPCTL
 * Offset: 0x18  Comparator Control  Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CMP1EN    |Comparator 1 enable or disable control
 * |        |          |0= disable
 * |        |          |1= enable
 * |[1]     |C1NSEL    |Comparator 1 inverting input control
 * |        |          |0= from VH0
 * |        |          |1= from C1N pin
 * |[2]     |C1OUTEN   |Comparator 1 output pin control bit
 * |        |          |0= disable
 * |        |          |1= enable
 * |[3]     |C1INTEN_  |Comparator 1 interrupt control
 * |        |          |0= disable
 * |        |          |1= enable
 * |[4]     |CMP_INT   |Comparator Interrupt.
 * |        |          |Set by harddware.
 * |        |          |Write 1 to clear.
 * |[7]     |CNPSEL_   |Comparator non-inverting input control
 * |        |          |0= from OPA output
 * |        |          |1= from CNP pin
 * |[8]     |CMP2EN_   |Comparator 2 enable or disable control
 * |        |          |0= disable
 * |        |          |1= enable
 * |[9]     |C2PSEL_   |Comparator 2 inverting input control
 * |        |          |0= from VL0
 * |        |          |1= from C2P pin
 * |[10]    |C2OUTEN   |Comparator 2 output pin control bit
 * |        |          |0= disable
 * |        |          |1= enable
 * |[11]    |C2INTEN_  |Comparator 2 interrupt control
 * |        |          |0= disable
 * |        |          |1= enable
 * |[15:14] |CMPES     |Interrupt edge control bits
 * |        |          |00=disable
 * |        |          |01= rising edge trigger
 * |        |          |10= falling edge trigger
 * |        |          |11= dual edge trigger
 * |[16]    |C1OUT     |Comparator 1 Output.
 * |        |          |Real time readback of comparator 1.
 * |[17]    |C2OUT     |Comparator 2 Output.
 * |        |          |Real time readback of comparator 2.
 * |[24]    |LPWREN    |Comparator Low power mode enable
 * |        |          |If u20181u2019 comparator will remain enabled in STOP/SPD power modes.
 */
    __IO uint32_t CTRL;                  /*!< [0x0000] CSCAN Control Register                                           */
    __IO uint32_t CYCCNT;                /*!< [0x0004] CSCAN Cycle Count  Control Register                              */
    __IO uint32_t COUNT;                 /*!< [0x0008] CSCAN Count Status Register                                      */
    __IO uint32_t INTSTS;                /*!< [0x000c] CSCAN Interrupt Register                                         */
    __IO uint32_t AGPIO;                 /*!< [0x0010] CSCAN Analog GPIO Function Register                              */
    __IO uint32_t OPACTL;                /*!< [0x0014] Operational Amplifier Control  Register                          */
    __IO uint32_t CMPCTL;                /*!< [0x0018] Comparator Control  Register                                     */

} CSCAN_T;

/**
    @addtogroup CSCAN_CONST CSCAN Bit Field Definition
    Constant Definitions for CSCAN Controller
@{ */

#define CSCAN_CTRL_SEL_Pos               (0)                                               /*!< CSCAN_T::CTRL: SEL Position            */
#define CSCAN_CTRL_SEL_Msk               (0xfffful << CSCAN_CTRL_SEL_Pos)                  /*!< CSCAN_T::CTRL: SEL Mask                */

#define CSCAN_CTRL_CURRENT_Pos           (16)                                              /*!< CSCAN_T::CTRL: CURRENT Position        */
#define CSCAN_CTRL_CURRENT_Msk           (0x3ul << CSCAN_CTRL_CURRENT_Pos)                 /*!< CSCAN_T::CTRL: CURRENT Mask            */

#define CSCAN_CTRL_INT_EN_Pos            (20)                                              /*!< CSCAN_T::CTRL: INT_EN Position         */
#define CSCAN_CTRL_INT_EN_Msk            (0x1ul << CSCAN_CTRL_INT_EN_Pos)                  /*!< CSCAN_T::CTRL: INT_EN Mask             */

#define CSCAN_CTRL_SLOW_CLK_Pos          (21)                                              /*!< CSCAN_T::CTRL: SLOW_CLK Position       */
#define CSCAN_CTRL_SLOW_CLK_Msk          (0x1ul << CSCAN_CTRL_SLOW_CLK_Pos)                /*!< CSCAN_T::CTRL: SLOW_CLK Mask           */

#define CSCAN_CTRL_MODE0_Pos             (22)                                              /*!< CSCAN_T::CTRL: MODE0 Position          */
#define CSCAN_CTRL_MODE0_Msk             (0x1ul << CSCAN_CTRL_MODE0_Pos)                   /*!< CSCAN_T::CTRL: MODE0 Mask              */

#define CSCAN_CTRL_MODE1_Pos             (23)                                              /*!< CSCAN_T::CTRL: MODE1 Position          */
#define CSCAN_CTRL_MODE1_Msk             (0x1ul << CSCAN_CTRL_MODE1_Pos)                   /*!< CSCAN_T::CTRL: MODE1 Mask              */

#define CSCAN_CTRL_DUR_CNT_Pos           (24)                                              /*!< CSCAN_T::CTRL: DUR_CNT Position        */
#define CSCAN_CTRL_DUR_CNT_Msk           (0xful << CSCAN_CTRL_DUR_CNT_Pos)                 /*!< CSCAN_T::CTRL: DUR_CNT Mask            */

#define CSCAN_CTRL_EN_Pos                (30)                                              /*!< CSCAN_T::CTRL: EN Position             */
#define CSCAN_CTRL_EN_Msk                (0x1ul << CSCAN_CTRL_EN_Pos)                      /*!< CSCAN_T::CTRL: EN Mask                 */

#define CSCAN_CTRL_PD_Pos                (31)                                              /*!< CSCAN_T::CTRL: PD Position             */
#define CSCAN_CTRL_PD_Msk                (0x1ul << CSCAN_CTRL_PD_Pos)                      /*!< CSCAN_T::CTRL: PD Mask                 */

#define CSCAN_CYCCNT_CYCLE_CNT_Pos       (0)                                               /*!< CSCAN_T::CYCCNT: CYCLE_CNT Position    */
#define CSCAN_CYCCNT_CYCLE_CNT_Msk       (0xful << CSCAN_CYCCNT_CYCLE_CNT_Pos)             /*!< CSCAN_T::CYCCNT: CYCLE_CNT Mask        */

#define CSCAN_CYCCNT_MASK_Pos            (16)                                              /*!< CSCAN_T::CYCCNT: MASK Position         */
#define CSCAN_CYCCNT_MASK_Msk            (0xfffful << CSCAN_CYCCNT_MASK_Pos)               /*!< CSCAN_T::CYCCNT: MASK Mask             */

#define CSCAN_COUNT_COUNT_Pos            (0)                                               /*!< CSCAN_T::COUNT: COUNT Position         */
#define CSCAN_COUNT_COUNT_Msk            (0xfffful << CSCAN_COUNT_COUNT_Pos)               /*!< CSCAN_T::COUNT: COUNT Mask             */

#define CSCAN_INTSTS_INT_Pos             (0)                                               /*!< CSCAN_T::INTSTS: INT Position          */
#define CSCAN_INTSTS_INT_Msk             (0x1ul << CSCAN_INTSTS_INT_Pos)                   /*!< CSCAN_T::INTSTS: INT Mask              */

#define CSCAN_AGPIO_AGPIO_Pos            (0)                                               /*!< CSCAN_T::AGPIO: AGPIO Position         */
#define CSCAN_AGPIO_AGPIO_Msk            (0xfffful << CSCAN_AGPIO_AGPIO_Pos)               /*!< CSCAN_T::AGPIO: AGPIO Mask             */

#define CSCAN_OPACTL_A0EN_Pos            (0)                                               /*!< CSCAN_T::OPACTL: A0EN Position         */
#define CSCAN_OPACTL_A0EN_Msk            (0x1ul << CSCAN_OPACTL_A0EN_Pos)                  /*!< CSCAN_T::OPACTL: A0EN Mask             */

#define CSCAN_OPACTL_A0OEN_Pos           (1)                                               /*!< CSCAN_T::OPACTL: A0OEN Position        */
#define CSCAN_OPACTL_A0OEN_Msk           (0x1ul << CSCAN_OPACTL_A0OEN_Pos)                 /*!< CSCAN_T::OPACTL: A0OEN Mask            */

#define CSCAN_OPACTL_A0NS_Pos            (2)                                               /*!< CSCAN_T::OPACTL: A0NS Position         */
#define CSCAN_OPACTL_A0NS_Msk            (0x1ul << CSCAN_OPACTL_A0NS_Pos)                  /*!< CSCAN_T::OPACTL: A0NS Mask             */

#define CSCAN_OPACTL_A0PS_Pos            (3)                                               /*!< CSCAN_T::OPACTL: A0PS Position         */
#define CSCAN_OPACTL_A0PS_Msk            (0x1ul << CSCAN_OPACTL_A0PS_Pos)                  /*!< CSCAN_T::OPACTL: A0PS Mask             */

#define CSCAN_OPACTL_A0PSEL_Pos          (4)                                               /*!< CSCAN_T::OPACTL: A0PSEL Position       */
#define CSCAN_OPACTL_A0PSEL_Msk          (0x3ul << CSCAN_OPACTL_A0PSEL_Pos)                /*!< CSCAN_T::OPACTL: A0PSEL Mask           */

#define CSCAN_OPACTL_A0O2N_Pos           (6)                                               /*!< CSCAN_T::OPACTL: A0O2N Position        */
#define CSCAN_OPACTL_A0O2N_Msk           (0x1ul << CSCAN_OPACTL_A0O2N_Pos)                 /*!< CSCAN_T::OPACTL: A0O2N Mask            */

#define CSCAN_OPACTL_A0X_Pos             (7)                                               /*!< CSCAN_T::OPACTL: A0X Position          */
#define CSCAN_OPACTL_A0X_Msk             (0x1ul << CSCAN_OPACTL_A0X_Pos)                   /*!< CSCAN_T::OPACTL: A0X Mask              */

#define CSCAN_OPACTL_A1EN_Pos            (8)                                               /*!< CSCAN_T::OPACTL: A1EN Position         */
#define CSCAN_OPACTL_A1EN_Msk            (0x1ul << CSCAN_OPACTL_A1EN_Pos)                  /*!< CSCAN_T::OPACTL: A1EN Mask             */

#define CSCAN_OPACTL_A1OEN_Pos           (9)                                               /*!< CSCAN_T::OPACTL: A1OEN Position        */
#define CSCAN_OPACTL_A1OEN_Msk           (0x1ul << CSCAN_OPACTL_A1OEN_Pos)                 /*!< CSCAN_T::OPACTL: A1OEN Mask            */

#define CSCAN_OPACTL_A1NS_Pos            (10)                                              /*!< CSCAN_T::OPACTL: A1NS Position         */
#define CSCAN_OPACTL_A1NS_Msk            (0x1ul << CSCAN_OPACTL_A1NS_Pos)                  /*!< CSCAN_T::OPACTL: A1NS Mask             */

#define CSCAN_OPACTL_A1PS_Pos            (11)                                              /*!< CSCAN_T::OPACTL: A1PS Position         */
#define CSCAN_OPACTL_A1PS_Msk            (0x1ul << CSCAN_OPACTL_A1PS_Pos)                  /*!< CSCAN_T::OPACTL: A1PS Mask             */

#define CSCAN_OPACTL_A1PSEL_Pos          (12)                                              /*!< CSCAN_T::OPACTL: A1PSEL Position       */
#define CSCAN_OPACTL_A1PSEL_Msk          (0x3ul << CSCAN_OPACTL_A1PSEL_Pos)                /*!< CSCAN_T::OPACTL: A1PSEL Mask           */

#define CSCAN_OPACTL_A1O2N_Pos           (14)                                              /*!< CSCAN_T::OPACTL: A1O2N Position        */
#define CSCAN_OPACTL_A1O2N_Msk           (0x1ul << CSCAN_OPACTL_A1O2N_Pos)                 /*!< CSCAN_T::OPACTL: A1O2N Mask            */

#define CSCAN_OPACTL_A1X_Pos             (15)                                              /*!< CSCAN_T::OPACTL: A1X Position          */
#define CSCAN_OPACTL_A1X_Msk             (0x1ul << CSCAN_OPACTL_A1X_Pos)                   /*!< CSCAN_T::OPACTL: A1X Mask              */

#define CSCAN_OPACTL_PGA_Pos             (16)                                              /*!< CSCAN_T::OPACTL: PGA Position          */
#define CSCAN_OPACTL_PGA_Msk             (0x7ul << CSCAN_OPACTL_PGA_Pos)                   /*!< CSCAN_T::OPACTL: PGA Mask              */

#define CSCAN_OPACTL_PGAEN_Pos           (19)                                              /*!< CSCAN_T::OPACTL: PGAEN Position        */
#define CSCAN_OPACTL_PGAEN_Msk           (0x1ul << CSCAN_OPACTL_PGAEN_Pos)                 /*!< CSCAN_T::OPACTL: PGAEN Mask            */

#define CSCAN_OPACTL_VREFEN_Pos          (20)                                              /*!< CSCAN_T::OPACTL: VREFEN Position       */
#define CSCAN_OPACTL_VREFEN_Msk          (0x1ul << CSCAN_OPACTL_VREFEN_Pos)                /*!< CSCAN_T::OPACTL: VREFEN Mask           */

#define CSCAN_OPACTL_A0O2A1P_Pos         (22)                                              /*!< CSCAN_T::OPACTL: A0O2A1P Position      */
#define CSCAN_OPACTL_A0O2A1P_Msk         (0x1ul << CSCAN_OPACTL_A0O2A1P_Pos)               /*!< CSCAN_T::OPACTL: A0O2A1P Mask          */

#define CSCAN_OPACTL_A0O2A1N_Pos         (23)                                              /*!< CSCAN_T::OPACTL: A0O2A1N Position      */
#define CSCAN_OPACTL_A0O2A1N_Msk         (0x1ul << CSCAN_OPACTL_A0O2A1N_Pos)               /*!< CSCAN_T::OPACTL: A0O2A1N Mask          */

#define CSCAN_OPACTL_LPWREN_Pos          (24)                                              /*!< CSCAN_T::OPACTL: LPWREN Position       */
#define CSCAN_OPACTL_LPWREN_Msk          (0x1ul << CSCAN_OPACTL_LPWREN_Pos)                /*!< CSCAN_T::OPACTL: LPWREN Mask           */

#define CSCAN_OPACTL_A0O2CIN_Pos         (25)                                              /*!< CSCAN_T::OPACTL: A0O2CIN Position      */
#define CSCAN_OPACTL_A0O2CIN_Msk         (0x1ul << CSCAN_OPACTL_A0O2CIN_Pos)               /*!< CSCAN_T::OPACTL: A0O2CIN Mask          */

#define CSCAN_OPACTL_A1O2CIN_Pos         (26)                                              /*!< CSCAN_T::OPACTL: A1O2CIN Position      */
#define CSCAN_OPACTL_A1O2CIN_Msk         (0x1ul << CSCAN_OPACTL_A1O2CIN_Pos)               /*!< CSCAN_T::OPACTL: A1O2CIN Mask          */

#define CSCAN_CMPCTL_CMP1EN_Pos          (0)                                               /*!< CSCAN_T::CMPCTL: CMP1EN Position       */
#define CSCAN_CMPCTL_CMP1EN_Msk          (0x1ul << CSCAN_CMPCTL_CMP1EN_Pos)                /*!< CSCAN_T::CMPCTL: CMP1EN Mask           */

#define CSCAN_CMPCTL_C1NSEL_Pos          (1)                                               /*!< CSCAN_T::CMPCTL: C1NSEL Position       */
#define CSCAN_CMPCTL_C1NSEL_Msk          (0x1ul << CSCAN_CMPCTL_C1NSEL_Pos)                /*!< CSCAN_T::CMPCTL: C1NSEL Mask           */

#define CSCAN_CMPCTL_C1OUTEN_Pos         (2)                                               /*!< CSCAN_T::CMPCTL: C1OUTEN Position      */
#define CSCAN_CMPCTL_C1OUTEN_Msk         (0x1ul << CSCAN_CMPCTL_C1OUTEN_Pos)               /*!< CSCAN_T::CMPCTL: C1OUTEN Mask          */

#define CSCAN_CMPCTL_C1INTEN_Pos         (3)                                               /*!< CSCAN_T::CMPCTL: C1INTEN_ Position     */
#define CSCAN_CMPCTL_C1INTEN_Msk         (0x1ul << CSCAN_CMPCTL_C1INTEN_Pos)               /*!< CSCAN_T::CMPCTL: C1INTEN_ Mask         */

#define CSCAN_CMPCTL_CMP_INT_Pos         (4)                                               /*!< CSCAN_T::CMPCTL: CMP_INT Position      */
#define CSCAN_CMPCTL_CMP_INT_Msk         (0x1ul << CSCAN_CMPCTL_CMP_INT_Pos)               /*!< CSCAN_T::CMPCTL: CMP_INT Mask          */

#define CSCAN_CMPCTL_CNPSEL_Pos          (7)                                               /*!< CSCAN_T::CMPCTL: CNPSEL_ Position      */
#define CSCAN_CMPCTL_CNPSEL_Msk          (0x1ul << CSCAN_CMPCTL_CNPSEL_Pos)                /*!< CSCAN_T::CMPCTL: CNPSEL_ Mask          */

#define CSCAN_CMPCTL_CMP2EN_Pos          (8)                                               /*!< CSCAN_T::CMPCTL: CMP2EN_ Position      */
#define CSCAN_CMPCTL_CMP2EN_Msk          (0x1ul << CSCAN_CMPCTL_CMP2EN_Pos)                /*!< CSCAN_T::CMPCTL: CMP2EN_ Mask          */

#define CSCAN_CMPCTL_C2PSEL_Pos          (9)                                               /*!< CSCAN_T::CMPCTL: C2PSEL_ Position      */
#define CSCAN_CMPCTL_C2PSEL_Msk          (0x1ul << CSCAN_CMPCTL_C2PSEL_Pos)                /*!< CSCAN_T::CMPCTL: C2PSEL_ Mask          */

#define CSCAN_CMPCTL_C2OUTEN_Pos         (10)                                              /*!< CSCAN_T::CMPCTL: C2OUTEN Position      */
#define CSCAN_CMPCTL_C2OUTEN_Msk         (0x1ul << CSCAN_CMPCTL_C2OUTEN_Pos)               /*!< CSCAN_T::CMPCTL: C2OUTEN Mask          */

#define CSCAN_CMPCTL_C2INTEN_Pos         (11)                                              /*!< CSCAN_T::CMPCTL: C2INTEN_ Position     */
#define CSCAN_CMPCTL_C2INTEN_Msk         (0x1ul << CSCAN_CMPCTL_C2INTEN_Pos)               /*!< CSCAN_T::CMPCTL: C2INTEN_ Mask         */

#define CSCAN_CMPCTL_CMPES_Pos           (14)                                              /*!< CSCAN_T::CMPCTL: CMPES Position        */
#define CSCAN_CMPCTL_CMPES_Msk           (0x3ul << CSCAN_CMPCTL_CMPES_Pos)                 /*!< CSCAN_T::CMPCTL: CMPES Mask            */

#define CSCAN_CMPCTL_C1OUT_Pos           (16)                                              /*!< CSCAN_T::CMPCTL: C1OUT Position        */
#define CSCAN_CMPCTL_C1OUT_Msk           (0x1ul << CSCAN_CMPCTL_C1OUT_Pos)                 /*!< CSCAN_T::CMPCTL: C1OUT Mask            */

#define CSCAN_CMPCTL_C2OUT_Pos           (17)                                              /*!< CSCAN_T::CMPCTL: C2OUT Position        */
#define CSCAN_CMPCTL_C2OUT_Msk           (0x1ul << CSCAN_CMPCTL_C2OUT_Pos)                 /*!< CSCAN_T::CMPCTL: C2OUT Mask            */

#define CSCAN_CMPCTL_LPWREN_Pos          (24)                                              /*!< CSCAN_T::CMPCTL: LPWREN Position       */
#define CSCAN_CMPCTL_LPWREN_Msk          (0x1ul << CSCAN_CMPCTL_LPWREN_Pos)                /*!< CSCAN_T::CMPCTL: LPWREN Mask           */

/**@}*/ /* CSCAN_CONST */
/**@}*/ /* end of CSCAN register group */


/*---------------------- Audio Class D Speaker Driver -------------------------*/
/**
    @addtogroup DPWM Audio Class D Speaker Driver(DPWM)
    Memory Mapped Structure for DPWM Controller
@{ */
 
typedef struct
{


/**
 * @var DPWM_T::CTL
 * Offset: 0x00  DPWM Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |FIFOWIDTH |DPWM FIFO DATA WIDTH SELETION From PDMA
 * |        |          |00 = PDMA MSB 24bits PWDATA[31:8]
 * |        |          |01 = PDMA 16 bits PWDATA[15:0]
 * |        |          |10 = PDMA 8bits PWDATA[7:0]
 * |        |          |11 = PDMA 24bits PWDATA[23:0]
 * |[3]     |DEADTIME  |DPWM Driver DEADTIME Control.
 * |        |          |Enabling this bit will insert an additional clock cycle deadtime into the switching of PMOS and NMOS driver transistors.
 * |[6]     |DPWMEN    |DPWM Enable
 * |        |          |0 = Disable DPWM.
 * |        |          |1 = Enable DPWM
 * |[7]     |DWPMDRVEN |DPWM Driver Enable
 * |        |          |0 = Disable DPWM Driver.
 * |        |          |1 = Enable DPWM Diver.
 * |[11]    |RXTHIE    |DPWM FIFO Threshold Interrupt
 * |        |          |0 = DPWM FIFO threshold interrupt Disabled
 * |        |          |1 = DPWM FIFO threshold interrupt Enabled.
 * |[15:12] |RXTH      |DPWM FIFO Threshold
 * |        |          |If the valid data count of the DPWM FIFO buffer is less than or equal to RXTH setting, the RXTHIF bit will set to 1, else the RXTHIF bit will be cleared to 0.
 * @var DPWM_T::STS
 * Offset: 0x04  DPWM DATA FIFO Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FULL      |FIFO Full
 * |        |          |0 = FIFO is not full.
 * |        |          |1 = FIFO is full.
 * |[1]     |EMPTY     |FIFO Empty
 * |        |          |0 = FIFO is not empty.
 * |        |          |1 = FIFO is empty.
 * |[2]     |RXTHIF    |DPWM FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count within the DPWM FIFO buffer is larger than the setting value of RXTH.
 * |        |          |1 = The valid data count within the transmit FIFO buffer is less than or equal to the setting value of RXTH
 * |[7:4]   |FIFOPTR   |DPWM FIFO Pointer (Read Only)
 * |        |          |The FULL bit and FIFOPOINTER indicates the field that the valid data count within the DPWM FIFO buffer.
 * |        |          |The Maximum value shown in FIFO_POINTER is 15
 * |        |          |When the using level of DPWM FIFO Buffer equal to 16, The FULL bit is set to 1.
 * |[31]    |BISTEN    |BIST Enable
 * |        |          |0 = disable DPWM FIFO BIST testing.
 * |        |          |1 = enable DPWM FIFO BIST testing.
 * |        |          | DPWM FIFO can be testing by Cortex-M0
 * |        |          |Internal use
 * @var DPWM_T::DMACTL
 * Offset: 0x08  DPWM PDMA Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |DMAEN     |Enable DPWM DMA Interface
 * |        |          |0 = Disable PDMA. No requests will be made to PDMA controller.
 * |        |          |1 = Enable PDMA. Block will request data from PDMA controller whenever FIFO is not empty.
 * @var DPWM_T::DATA
 * Offset: 0x0C  DPWM DATA FIFO Input
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |INDATA    |DPWM FIFO Audio Data Input
 * |        |          |A write to this register pushes data onto the DPWM FIFO and increments the write pointer
 * |        |          |This is the address that PDMA writes audio data to.
 * @var DPWM_T::ZOHDIV
 * Offset: 0x10  DPWM Zero Order Hold Division Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |ZOHDIV    |DPWM Zero Order Hold, Down-sampling Divisor
 * |        |          |The input sample rate of the DPWM is set by DPWM_CLK frequency and the divisor set in this register by the following formula:
 * |        |          |Fs = DPWM_CLK /(ZOHdiv *64* BIQ_CTL.DPWMPUSR).
 * |        |          |Default is 6, which gives a sample rate of 16kHz up-sample 256 for a 24.576MHz DPWM_CLK and BIQ_CTL.DPWMPUSR is 4.
 * |        |          |ZOH_DIV must be greater than 2
 */
    __IO uint32_t CTL;                   /*!< [0x0000] DPWM Control Register                                            */
    __I  uint32_t STS;                   /*!< [0x0004] DPWM DATA FIFO Status Register                                   */
    __IO uint32_t DMACTL;                /*!< [0x0008] DPWM PDMA Control Register                                       */
    __O  uint32_t DATA;                  /*!< [0x000c] DPWM DATA FIFO Input                                             */
    __IO uint32_t ZOHDIV;                /*!< [0x0010] DPWM Zero Order Hold Division Register                           */

} DPWM_T;

/**
    @addtogroup DPWM_CONST DPWM Bit Field Definition
    Constant Definitions for DPWM Controller
@{ */

#define DPWM_CTL_FIFOWIDTH_Pos           (0)                                               /*!< DPWM_T::CTL: FIFOWIDTH Position        */
#define DPWM_CTL_FIFOWIDTH_Msk           (0x3ul << DPWM_CTL_FIFOWIDTH_Pos)                 /*!< DPWM_T::CTL: FIFOWIDTH Mask            */

#define DPWM_CTL_DEADTIME_Pos            (3)                                               /*!< DPWM_T::CTL: DEADTIME Position         */
#define DPWM_CTL_DEADTIME_Msk            (0x1ul << DPWM_CTL_DEADTIME_Pos)                  /*!< DPWM_T::CTL: DEADTIME Mask             */

#define DPWM_CTL_DPWMEN_Pos              (6)                                               /*!< DPWM_T::CTL: DPWMEN Position           */
#define DPWM_CTL_DPWMEN_Msk              (0x1ul << DPWM_CTL_DPWMEN_Pos)                    /*!< DPWM_T::CTL: DPWMEN Mask               */

#define DPWM_CTL_DWPMDRVEN_Pos           (7)                                               /*!< DPWM_T::CTL: DWPMDRVEN Position        */
#define DPWM_CTL_DWPMDRVEN_Msk           (0x1ul << DPWM_CTL_DWPMDRVEN_Pos)                 /*!< DPWM_T::CTL: DWPMDRVEN Mask            */

#define DPWM_CTL_RXTHIE_Pos              (11)                                              /*!< DPWM_T::CTL: RXTHIE Position           */
#define DPWM_CTL_RXTHIE_Msk              (0x1ul << DPWM_CTL_RXTHIE_Pos)                    /*!< DPWM_T::CTL: RXTHIE Mask               */

#define DPWM_CTL_RXTH_Pos                (12)                                              /*!< DPWM_T::CTL: RXTH Position             */
#define DPWM_CTL_RXTH_Msk                (0xful << DPWM_CTL_RXTH_Pos)                      /*!< DPWM_T::CTL: RXTH Mask                 */

#define DPWM_STS_FULL_Pos                (0)                                               /*!< DPWM_T::STS: FULL Position             */
#define DPWM_STS_FULL_Msk                (0x1ul << DPWM_STS_FULL_Pos)                      /*!< DPWM_T::STS: FULL Mask                 */

#define DPWM_STS_EMPTY_Pos               (1)                                               /*!< DPWM_T::STS: EMPTY Position            */
#define DPWM_STS_EMPTY_Msk               (0x1ul << DPWM_STS_EMPTY_Pos)                     /*!< DPWM_T::STS: EMPTY Mask                */

#define DPWM_STS_RXTHIF_Pos              (2)                                               /*!< DPWM_T::STS: RXTHIF Position           */
#define DPWM_STS_RXTHIF_Msk              (0x1ul << DPWM_STS_RXTHIF_Pos)                    /*!< DPWM_T::STS: RXTHIF Mask               */

#define DPWM_STS_FIFOPTR_Pos             (4)                                               /*!< DPWM_T::STS: FIFOPTR Position          */
#define DPWM_STS_FIFOPTR_Msk             (0xful << DPWM_STS_FIFOPTR_Pos)                   /*!< DPWM_T::STS: FIFOPTR Mask              */

#define DPWM_STS_BISTEN_Pos              (31)                                              /*!< DPWM_T::STS: BISTEN Position           */
#define DPWM_STS_BISTEN_Msk              (0x1ul << DPWM_STS_BISTEN_Pos)                    /*!< DPWM_T::STS: BISTEN Mask               */

#define DPWM_DMACTL_DMAEN_Pos            (0)                                               /*!< DPWM_T::DMACTL: DMAEN Position         */
#define DPWM_DMACTL_DMAEN_Msk            (0x1ul << DPWM_DMACTL_DMAEN_Pos)                  /*!< DPWM_T::DMACTL: DMAEN Mask             */

#define DPWM_DATA_INDATA_Pos             (0)                                               /*!< DPWM_T::DATA: INDATA Position          */
#define DPWM_DATA_INDATA_Msk             (0xfffffffful << DPWM_DATA_INDATA_Pos)            /*!< DPWM_T::DATA: INDATA Mask              */

#define DPWM_ZOHDIV_ZOHDIV_Pos           (0)                                               /*!< DPWM_T::ZOHDIV: ZOHDIV Position        */
#define DPWM_ZOHDIV_ZOHDIV_Msk           (0xfful << DPWM_ZOHDIV_ZOHDIV_Pos)                /*!< DPWM_T::ZOHDIV: ZOHDIV Mask            */

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
 * @var FMC_T::ISPCTL
 * Offset: 0x00  ISP Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPEN     |ISP Enable
 * |        |          |0 = Disable ISP function.
 * |        |          |1 = Enable ISP function.
 * |[1]     |BS        |Boot Select
 * |        |          |0 = APROM.
 * |        |          |1 = LDROM.
 * |        |          |Modify this bit to select which ROM next boot is to occur
 * |        |          |This bit also functions as MCU boot status flag, which can be used to check where MCU booted from
 * |        |          |This bit is initialized after power-on reset with the inverse of CBS in Config0; It is not reset for any other reset event.
 * |[3]     |APUWEN    |APU Write Enable
 * |        |          |1 = APROM write to itself.
 * |        |          |0 = APROM canu2019t write itself. ISPFF with u201C1u201D
 * |[4]     |CFGUEN    |CONFIG Update Enable
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |        |          |When enabled, ISP functions can access the CONFIG address space and modify device configuration area. 
 * |[5]     |LDUEN     |LDROM Update Enable
 * |        |          |LDROM update enable bit.
 * |        |          |0 = LDROM cannot be updated.
 * |        |          |1 = LDROM can be updated when the MCU runs in APROM.
 * |[6]     |ISPFF     |ISP Fail Flag
 * |        |          |This bit is set by hardware when a triggered ISP meets any of the following conditions:
 * |        |          |(1) APROM writes to itself.
 * |        |          |(2) LDROM writes to itself.
 * |        |          |(3) Destination address is illegal, such as over an available range.
 * |        |          |Write 1 to clear.
 * |[21]    |CACHEDIS  |Cache Disable
 * |        |          |When set to 1, caching of flash memory reads is disabled.
 * @var FMC_T::ISPADDR
 * Offset: 0x04  ISP Address Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ISPADDR   |ISP Address Register
 * |        |          |This is the memory address register that a subsequent ISP command will access
 * |        |          |ISP operation are carried out on 32bit words only, consequently ISPADDR [1:0] must be 00b for correct ISP operation
 * @var FMC_T::ISPDAT
 * Offset: 0x08  ISP Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ISPDAT    |ISP Data Register
 * |        |          |Write data to this register before an ISP program operation.
 * |        |          |Read data from this register after an ISP read operation
 * @var FMC_T::ISPCMD
 * Offset: 0x0C  ISP Command Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |CMD       |ISP Command
 * |        |          |Operation Mode : CMD
 * |        |          |Standby : 0x3X
 * |        |          |Read : 0x00
 * |        |          |Program : 0x21
 * |        |          |Page Erase : 0x22
 * |        |          |Read CID : 0x0B
 * |        |          |Read DID : 0x0C
 * @var FMC_T::ISPTRG
 * Offset: 0x10  ISP Trigger Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ISPGO     |ISP Start Trigger
 * |        |          |Write 1 to start ISP operation
 * |        |          |This will be cleared to 0 by hardware automatically when ISP operation is finished.
 * |        |          |0 = ISP operation is finished.
 * |        |          |1 = ISP is on going.
 * |        |          |After triggering an ISP function M0 instruction pipeline should be flushed with a ISB instruction to guarantee data integrity.
 * |        |          |This is a protected register, user must first follow the unlock sequence to gain access.
 * @var FMC_T::DFBA
 * Offset: 0x14  Data Flash Base Address
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |DFBA      |Data Flash Base Address
 * |        |          |This register reports the data flash starting address. It is a read only register.
 * |        |          |Data flash size is defined by user configuration; register content is loaded from Config1 when chip is reset.
 */
    __IO uint32_t ISPCTL;                /*!< [0x0000] ISP Control Register                                             */
    __IO uint32_t ISPADDR;               /*!< [0x0004] ISP Address Register                                             */
    __IO uint32_t ISPDAT;                /*!< [0x0008] ISP Data Register                                                */
    __IO uint32_t ISPCMD;                /*!< [0x000c] ISP Command Register                                             */
    __IO uint32_t ISPTRG;                /*!< [0x0010] ISP Trigger Control Register                                     */
    __I  uint32_t DFBA;                  /*!< [0x0014] Data Flash Base Address                                          */

} FMC_T;

/**
    @addtogroup FMC_CONST FMC Bit Field Definition
    Constant Definitions for FMC Controller
@{ */

#define FMC_ISPCTL_ISPEN_Pos             (0)                                               /*!< FMC_T::ISPCTL: ISPEN Position          */
#define FMC_ISPCTL_ISPEN_Msk             (0x1ul << FMC_ISPCTL_ISPEN_Pos)                   /*!< FMC_T::ISPCTL: ISPEN Mask              */

#define FMC_ISPCTL_BS_Pos                (1)                                               /*!< FMC_T::ISPCTL: BS Position             */
#define FMC_ISPCTL_BS_Msk                (0x1ul << FMC_ISPCTL_BS_Pos)                      /*!< FMC_T::ISPCTL: BS Mask                 */

#define FMC_ISPCTL_APUWEN_Pos            (3)                                               /*!< FMC_T::ISPCTL: APUWEN Position         */
#define FMC_ISPCTL_APUWEN_Msk            (0x1ul << FMC_ISPCTL_APUWEN_Pos)                  /*!< FMC_T::ISPCTL: APUWEN Mask             */

#define FMC_ISPCTL_CFGUEN_Pos            (4)                                               /*!< FMC_T::ISPCTL: CFGUEN Position         */
#define FMC_ISPCTL_CFGUEN_Msk            (0x1ul << FMC_ISPCTL_CFGUEN_Pos)                  /*!< FMC_T::ISPCTL: CFGUEN Mask             */

#define FMC_ISPCTL_LDUEN_Pos             (5)                                               /*!< FMC_T::ISPCTL: LDUEN Position          */
#define FMC_ISPCTL_LDUEN_Msk             (0x1ul << FMC_ISPCTL_LDUEN_Pos)                   /*!< FMC_T::ISPCTL: LDUEN Mask              */

#define FMC_ISPCTL_ISPFF_Pos             (6)                                               /*!< FMC_T::ISPCTL: ISPFF Position          */
#define FMC_ISPCTL_ISPFF_Msk             (0x1ul << FMC_ISPCTL_ISPFF_Pos)                   /*!< FMC_T::ISPCTL: ISPFF Mask              */

#define FMC_ISPCTL_CACHEDIS_Pos          (21)                                              /*!< FMC_T::ISPCTL: CACHEDIS Position       */
#define FMC_ISPCTL_CACHEDIS_Msk          (0x1ul << FMC_ISPCTL_CACHEDIS_Pos)                /*!< FMC_T::ISPCTL: CACHEDIS Mask           */

#define FMC_ISPADDR_ISPADDR_Pos          (0)                                               /*!< FMC_T::ISPADDR: ISPADDR Position       */
#define FMC_ISPADDR_ISPADDR_Msk          (0xfffffffful << FMC_ISPADDR_ISPADDR_Pos)         /*!< FMC_T::ISPADDR: ISPADDR Mask           */

#define FMC_ISPDAT_ISPDAT_Pos            (0)                                               /*!< FMC_T::ISPDAT: ISPDAT Position         */
#define FMC_ISPDAT_ISPDAT_Msk            (0xfffffffful << FMC_ISPDAT_ISPDAT_Pos)           /*!< FMC_T::ISPDAT: ISPDAT Mask             */

#define FMC_ISPCMD_CMD_Pos               (0)                                               /*!< FMC_T::ISPCMD: CMD Position            */
#define FMC_ISPCMD_CMD_Msk               (0x3ful << FMC_ISPCMD_CMD_Pos)                    /*!< FMC_T::ISPCMD: CMD Mask                */

#define FMC_ISPTRG_ISPGO_Pos             (0)                                               /*!< FMC_T::ISPTRG: ISPGO Position          */
#define FMC_ISPTRG_ISPGO_Msk             (0x1ul << FMC_ISPTRG_ISPGO_Pos)                   /*!< FMC_T::ISPTRG: ISPGO Mask              */

#define FMC_DFBA_DFBA_Pos                (0)                                               /*!< FMC_T::DFBA: DFBA Position             */
#define FMC_DFBA_DFBA_Msk                (0xfffffffful << FMC_DFBA_DFBA_Pos)               /*!< FMC_T::DFBA: DFBA Mask                 */

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
 * @var GPIO_T::MODE
 * Offset: 0x00  GPIO Port A Pin I/O Mode Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |MODE0     |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[3:2]   |MODE1     |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[5:4]   |MODE2     |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[7:6]   |MODE3     |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[9:8]   |MODE4     |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[11:10] |MODE5     |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[13:12] |MODE6     |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[15:14] |MODE7     |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[17:16] |MODE8     |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[19:18] |MODE9     |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[21:20] |MODE10    |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[23:22] |MODE11    |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[25:24] |MODE12    |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[27:26] |MODE13    |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[29:28] |MODE14    |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |[31:30] |MODE15    |Px I/O Pin[n] Mode Control
 * |        |          |Determine each I/O type of GPIOx pins
 * |        |          |00 = GPIO port [n] pin is in INPUT mode.
 * |        |          |01 = GPIO port [n] pin is in OUTPUT mode.
 * |        |          |10 = GPIO port [n] pin is in Open-Drain mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * @var GPIO_T::DINOFF
 * Offset: 0x04  GPIO Port A Pin Input Disable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:16] |DINOFF    |GPIOx Pin[n] OFF Digital Input Path Enable
 * |        |          |0 = Enable IO digital input path (Default).
 * |        |          |1 = Disable IO digital input path (low leakage mode).
 * @var GPIO_T::DOUT
 * Offset: 0x08  GPIO Port A Data Output Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DOUT      |Px Pin[n] Output Value
 * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as output, open-drain or quasi-bidirectional mode.
 * |        |          |1 = GPIO port [A/B] Pin[n] will drive High if the corresponding output mode bit is set.
 * |        |          |0 = GPIO port [A/B] Pin[n] will drive Low if the corresponding output mode bit is set.
 * @var GPIO_T::DATMSK
 * Offset: 0x0C  GPIO Port A Data Output Write Mask
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DATMSK    |Port [A/B] Data Output Write Mask
 * |        |          |These bits are used to protect the corresponding register of Px_DOUT bit[n]
 * |        |          |When set the DATMSK bit[n] to u201C1u201D, the corresponding DOUTn bit is writing protected.
 * |        |          |0 = The corresponding Px_DOUT[n] bit can be updated.
 * |        |          |1 = The corresponding Px_DOUT[n] bit is read only.
 * @var GPIO_T::PIN
 * Offset: 0x10  GPIO Port A Pin Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PIN       |Port [A/B] Pin Values
 * |        |          |The value read from each of these bit reflects the actual status of the respective GPIO pin
 * @var GPIO_T::DBEN
 * Offset: 0x14  GPIO Port A De-bounce Enable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |DBEN      |Port [A/B] De-bounce Enable Control
 * |        |          |DBEN[n]used to enable the de-bounce function for each corresponding bit
 * |        |          |For an edge triggered interrupt to be generated, input signal must be valid for two consecutive de-bounce periods
 * |        |          |The de-bounce time is controlled by the GPIO_DBCTL register.
 * |        |          |The DBEN[n] is used for u201Cedge-triggeru201D interrupt only; it is ignored for u201Clevel triggeru201D interrupt
 * |        |          |0 = The bit[n] de-bounce function is disabled.
 * |        |          |1 = The bit[n] de-bounce function is enabled. 
 * @var GPIO_T::INTTYPE
 * Offset: 0x18  GPIO Port A Interrupt Trigger Type
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |TYPE      |Port [A/B] Edge or Level Detection Interrupt Trigger Type
 * |        |          |TYPE[n] used to control whether the interrupt mode is level triggered or edge triggered
 * |        |          |If the interrupt mode is edge triggered, edge de-bounce is controlled by the DBEN register
 * |        |          |If the interrupt mode is level triggered, the input source is sampled each clock to generate an interrupt
 * |        |          |0 = Edge triggered interrupt.
 * |        |          |1 = Level triggered interrupt.
 * |        |          |If level triggered interrupt is selected, then only one level can be selected in the Px_INTEN register
 * |        |          |If both levels are set no interrupt will occur
 * @var GPIO_T::INTEN
 * Offset: 0x1C  GPIO Port A Interrupt Enable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |FLIEN     |Port [A/B] Interrupt Enable by Input Falling Edge or Input Level Low
 * |        |          |FLIEN[n] is used to enable the falling/low interrupt for each of the corresponding GPIO pins
 * |        |          |It also enables the pin wakeup function.
 * |        |          |If the interrupt is configured in level trigger mode, a level u201Clowu201D will generate an interrupt.
 * |        |          |If the interrupt is configured in edge trigger mode, a state change from u201Chigh-to-lowu201D will generate an interrupt.
 * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
 * |        |          |0 = Disable GPIOx[n] for low-level or high-to-low interrupt.
 * |        |          |1 = Enable GPIOx[n] for low-level or high-to-low interrupt.
 * |[31:16] |RHIEN     |Port [A/B] Interrupt Enable by Input Rising Edge or Input Level High
 * |        |          |RHIEN[n] is used to enable the rising/high interrupt for each of the corresponding GPIO pins
 * |        |          |It also enables the pin wakeup function.
 * |        |          |If the interrupt is configured in level trigger mode, a level u201Chighu201D will generate an interrupt.
 * |        |          |If the interrupt is configured in edge trigger mode, a state change from u201Clow-to-highu201D will generate an interrupt.
 * |        |          |GPB.0 and GPB.1 trigger individual IRQ vectors (IRQ2/IRQ3) while remaining GPIO trigger a single interrupt vector IRQ4.
 * |        |          |0 = Disable GPIOx[n] for level-high or low-to-high interrupt.
 * |        |          |1 = Enable GPIOx[n] for level-high or low-to-high interrupt.
 * @var GPIO_T::INTSRC
 * Offset: 0x20  GPIO Port A Interrupt Source Flag
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |INTSRC    |Port [A/B] Interrupt Source Flag
 * |        |          |Read :
 * |        |          |1 = Indicates GPIOx[n] generated an interrupt.
 * |        |          |0 = No interrupt from GPIOx[n].
 * |        |          |Write :
 * |        |          |1 = Clear the corresponding pending interrupt.
 * |        |          |0 = No action.
 */
    __IO uint32_t MODE;               /*!< [0x0000] GPIO Port A Pin I/O Mode Control                                 */
    __IO uint32_t DINOFF;             /*!< [0x0004] GPIO Port A Pin Input Disable                                    */
    __IO uint32_t DOUT;               /*!< [0x0008] GPIO Port A Data Output Value                                    */
    __IO uint32_t DATMSK;             /*!< [0x000c] GPIO Port A Data Output Write Mask                               */
    __I  uint32_t PIN;                /*!< [0x0010] GPIO Port A Pin Value                                            */
    __IO uint32_t DBEN;               /*!< [0x0014] GPIO Port A De-bounce Enable                                     */
    __IO uint32_t INTTYPE;            /*!< [0x0018] GPIO Port A Interrupt Trigger Type                               */
    __IO uint32_t INTEN;              /*!< [0x001c] GPIO Port A Interrupt Enable                                     */
    __IO uint32_t INTSRC;             /*!< [0x0020] GPIO Port A Interrupt Source Flag                                */

} GPIO_T;

typedef struct {              
/**
* @var GPIO_DB_T::DBCTL
* Offset: 0x180  Interrupt De-bounce Control
* ---------------------------------------------------------------------------------------------------
* |Bits    |Field     |Descriptions
* | :----: | :----:   | :---- |
* |[3:0]   |DBCLKSEL  |De-bounce Sampling Cycle Selection
* |        |          |For edge level interrupt GPIO state is sampled every 2^(DBCLKSEL) de-bounce clocks
* |        |          |For example if DBCLKSRC = 6, then interrupt is sampled every 2^6 = 64 de-bounce clocks
* |        |          |If DBCLKSRC is 10kHz oscillator this would be a 64ms de-bounce.
* |[4]     |DBCLKSRC  |De-bounce Counter Clock Source Select
* |        |          |0 = De-bounce counter clock source is HCLK.
* |        |          |1 = De-bounce counter clock source is the internal 16 kHz clock.
* |[5]     |ICLKON    |Interrupt Clock on Mode
* |        |          |Set this bit u201C0u201D will gate the clock to the interrupt generation circuit if the GPIOx[n] interrupt is disabled.
* |        |          |0 = disable the clock if the GPIOx[n] interrupt is disabled.
* |        |          |1 = Interrupt generation clock always active.
*/
	__IO uint32_t DBCTL;                 /*!< [0x0180] Interrupt De-bounce Control                                      */
	
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

#define GPIO_MODE_MODE7_Pos              (14)                                              /*!< GPIO_T::MODE: MODE7 Position           */
#define GPIO_MODE_MODE7_Msk              (0x3ul << GPIO_MODE_MODE7_Pos)                    /*!< GPIO_T::MODE: MODE7 Mask               */

#define GPIO_MODE_MODE8_Pos              (16)                                              /*!< GPIO_T::MODE: MODE8 Position           */
#define GPIO_MODE_MODE8_Msk              (0x3ul << GPIO_MODE_MODE8_Pos)                    /*!< GPIO_T::MODE: MODE8 Mask               */

#define GPIO_MODE_MODE9_Pos              (18)                                              /*!< GPIO_T::MODE: MODE9 Position           */
#define GPIO_MODE_MODE9_Msk              (0x3ul << GPIO_MODE_MODE9_Pos)                    /*!< GPIO_T::MODE: MODE9 Mask               */

#define GPIO_MODE_MODE10_Pos             (20)                                              /*!< GPIO_T::MODE: MODE10 Position          */
#define GPIO_MODE_MODE10_Msk             (0x3ul << GPIO_MODE_MODE10_Pos)                   /*!< GPIO_T::MODE: MODE10 Mask              */

#define GPIO_MODE_MODE11_Pos             (22)                                              /*!< GPIO_T::MODE: MODE11 Position          */
#define GPIO_MODE_MODE11_Msk             (0x3ul << GPIO_MODE_MODE11_Pos)                   /*!< GPIO_T::MODE: MODE11 Mask              */

#define GPIO_MODE_MODE12_Pos             (24)                                              /*!< GPIO_T::MODE: MODE12 Position          */
#define GPIO_MODE_MODE12_Msk             (0x3ul << GPIO_MODE_MODE12_Pos)                   /*!< GPIO_T::MODE: MODE12 Mask              */

#define GPIO_MODE_MODE13_Pos             (26)                                              /*!< GPIO_T::MODE: MODE13 Position          */
#define GPIO_MODE_MODE13_Msk             (0x3ul << GPIO_MODE_MODE13_Pos)                   /*!< GPIO_T::MODE: MODE13 Mask              */

#define GPIO_MODE_MODE14_Pos             (28)                                              /*!< GPIO_T::MODE: MODE14 Position          */
#define GPIO_MODE_MODE14_Msk             (0x3ul << GPIO_MODE_MODE14_Pos)                   /*!< GPIO_T::MODE: MODE14 Mask              */

#define GPIO_MODE_MODE15_Pos             (30)                                              /*!< GPIO_T::MODE: MODE15 Position          */
#define GPIO_MODE_MODE15_Msk             (0x3ul << GPIO_MODE_MODE15_Pos)                   /*!< GPIO_T::MODE: MODE15 Mask              */

#define GPIO_DINOFF_DINOFF_Pos           (16)                                              /*!< GPIO_T::DINOFF: DINOFF Position        */
#define GPIO_DINOFF_DINOFF_Msk           (0xfffful << GPIO_DINOFF_DINOFF_Pos)              /*!< GPIO_T::DINOFF: DINOFF Mask            */

#define GPIO_DOUT_DOUT_Pos               (0)                                               /*!< GPIO_T::DOUT: DOUT Position            */
#define GPIO_DOUT_DOUT_Msk               (0xfffful << GPIO_DOUT_DOUT_Pos)                  /*!< GPIO_T::DOUT: DOUT Mask                */

#define GPIO_DATMSK_DATMSK_Pos           (0)                                               /*!< GPIO_T::DATMSK: DATMSK Position        */
#define GPIO_DATMSK_DATMSK_Msk           (0xfffful << GPIO_DATMSK_DATMSK_Pos)              /*!< GPIO_T::DATMSK: DATMSK Mask            */

#define GPIO_PIN_PIN_Pos                 (0)                                               /*!< GPIO_T::PIN: PIN Position              */
#define GPIO_PIN_PIN_Msk                 (0xfffful << GPIO_PIN_PIN_Pos)                    /*!< GPIO_T::PIN: PIN Mask                  */

#define GPIO_DBEN_DBEN_Pos               (0)                                               /*!< GPIO_T::DBEN: DBEN Position            */
#define GPIO_DBEN_DBEN_Msk               (0xfffful << GPIO_DBEN_DBEN_Pos)                  /*!< GPIO_T::DBEN: DBEN Mask                */

#define GPIO_INTTYPE_TYPE_Pos            (0)                                               /*!< GPIO_T::INTTYPE: TYPE Position         */
#define GPIO_INTTYPE_TYPE_Msk            (0xfffful << GPIO_INTTYPE_TYPE_Pos)               /*!< GPIO_T::INTTYPE: TYPE Mask             */

#define GPIO_INTEN_FLIEN_Pos             (0)                                               /*!< GPIO_T::INTEN: FLIEN Position          */
#define GPIO_INTEN_FLIEN_Msk             (0xfffful << GPIO_INTEN_FLIEN_Pos)                /*!< GPIO_T::INTEN: FLIEN Mask              */

#define GPIO_INTEN_RHIEN_Pos             (16)                                              /*!< GPIO_T::INTEN: RHIEN Position          */
#define GPIO_INTEN_RHIEN_Msk             (0xfffful << GPIO_INTEN_RHIEN_Pos)                /*!< GPIO_T::INTEN: RHIEN Mask              */

#define GPIO_INTSRC_INTSRC_Pos           (0)                                               /*!< GPIO_T::INTSRC: INTSRC Position        */
#define GPIO_INTSRC_INTSRC_Msk           (0xfffful << GPIO_INTSRC_INTSRC_Pos)              /*!< GPIO_T::INTSRC: INTSRC Mask            */

#define GPIO_DBCTL_DBCLKSEL_Pos          (0)                                               /*!< GPIO_T::DBCTL: DBCLKSEL Position       */
#define GPIO_DBCTL_DBCLKSEL_Msk          (0xful << GPIO_DBCTL_DBCLKSEL_Pos)                /*!< GPIO_T::DBCTL: DBCLKSEL Mask           */

#define GPIO_DBCTL_DBCLKSRC_Pos          (4)                                               /*!< GPIO_T::DBCTL: DBCLKSRC Position       */
#define GPIO_DBCTL_DBCLKSRC_Msk          (0x1ul << GPIO_DBCTL_DBCLKSRC_Pos)                /*!< GPIO_T::DBCTL: DBCLKSRC Mask           */

#define GPIO_DBCTL_ICLKON_Pos            (5)                                               /*!< GPIO_T::DBCTL: ICLKON Position         */
#define GPIO_DBCTL_ICLKON_Msk            (0x1ul << GPIO_DBCTL_ICLKON_Pos)                  /*!< GPIO_T::DBCTL: ICLKON Mask             */

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
 * @var I2C_T::CTL
 * Offset: 0x00  I2C Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2]     |AA        |Assert Acknowledge Control Bit
 * |        |          |When AA=1 prior to address or data received, an acknowledge (ACK - low level to SDA) will be returned during the acknowledge clock pulse on the SCL line when:.
 * |        |          |1. A slave is acknowledging the address sent from master,
 * |        |          |2. The receiver devices are acknowledging the data sent by transmitter.
 * |        |          |When AA = 0 prior to address or data received, a Not acknowledged (high level to SDA) will be returned during the acknowledge clock pulse on the SCL line
 * |[3]     |SI        |I2C Interrupt Flag
 * |        |          |When a new SIO state is present in the I2C_STATUS register, the SI flag is set by hardware, and if bit INTEN (I2C_CTL[7]) is set, the I2C interrupt is requested
 * |        |          |SI must be cleared by software
 * |        |          |Clear SI is by writing one to this bit.
 * |[4]     |STO       |I2C STOP Control Bit
 * |        |          |In master mode, set STO to transmit a STOP condition to bus
 * |        |          |I2C hardware will check the bus condition, when a STOP condition is detected this bit will be cleared by hardware automatically
 * |        |          |In slave mode, setting STO resets I2C hardware to the defined u201Cnot addressedu201D slave mode
 * |        |          |This means it is NO LONGER in the slave receiver mode able receive data from the master transmit device.
 * |[5]     |STA       |I2C START Control Bit
 * |        |          |Setting STA to logic 1 will enter master mode, the I2C hardware sends a START or repeat START condition to bus when the bus is free.
 * |[6]     |I2CEN     |I2C Controller Enable Bit
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |        |          |Set to enable I2C serial function block. 
 * |[7]     |INTEN     |Enable Interrupt
 * |        |          |0 = Disable interrupt.
 * |        |          |1 = Enable interrupt CPU.
 * @var I2C_T::ADDR0
 * Offset: 0x04  I2C Slave Address Register0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |ADDR      |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCUu2019s own address
 * |        |          |The I2C hardware will react if any of the addresses are matched.
 * @var I2C_T::DAT
 * Offset: 0x08  I2C DATA Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DAT       |I2C Data Register
 * |        |          |During master or slave transmit mode, data to be transmitted is written to this register
 * |        |          |During master or slave receive mode, data that has been received may be read from this register.
 * @var I2C_T::STATUS
 * Offset: 0x0C  I2C Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |STATUS    |I2C Status Register
 * |        |          |The status register of I2C:
 * |        |          |The three least significant bits are always 0
 * |        |          |The five most significant bits contain the status code
 * |        |          |There are 26 possible status codes
 * |        |          |When I2C_STATUS contains F8H, no serial interrupt is requested
 * |        |          |All other I2C_STATUS values correspond to defined I2C states
 * |        |          |When each of these states is entered, a status interrupt is requested (SI = 1)
 * |        |          |A valid status code is present in I2C_STATUS one PCLK cycle after SI is set by hardware and is still present one PCLK cycle after SI has been reset by software
 * |        |          |In addition, states 00H stands for a Bus Error
 * |        |          |A Bus Error occurs when a START or STOP condition is present at an illegal position in the frame
 * |        |          |Example of illegal position are during the serial transfer of an address byte, a data byte or an acknowledge bit.
 * @var I2C_T::CLKDIV
 * Offset: 0x10  I2C Clock Divided Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DIVIDER   |I2C Clock Divided Register
 * |        |          |The I2C clock rate bits: Data Baud Rate of I2C = PCLK /(4x(I2C_CLKDIV+1)).
 * @var I2C_T::TOCTL
 * Offset: 0x14  I2C Time Out Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TOIF      |Time-out Flag
 * |        |          |0 = No time-out.
 * |        |          |1 = Time-out flag is set by H/W. It can interrupt CPU. Write 1 to clear..
 * |[1]     |TOCDIV4   |Time-out Counter Input Clock Divide by 4
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |        |          |When enabled, the time-out clock is PCLK/4.
 * |[2]     |TOCEN     |Time-out Counter Control Bit
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |        |          |When enabled, the 14 bit time-out counter will start counting when SI is clear
 * |        |          |Setting flag SI to high will reset counter and re-start up counting after SI is cleared.
 * @var I2C_T::ADDR1
 * Offset: 0x18  I2C Slave Address Register1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |ADDR      |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCUu2019s own address
 * |        |          |The I2C hardware will react if any of the addresses are matched.
 * @var I2C_T::ADDR2
 * Offset: 0x1C  I2C Slave Address Register2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |ADDR      |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCUu2019s own address
 * |        |          |The I2C hardware will react if any of the addresses are matched.
 * @var I2C_T::ADDR3
 * Offset: 0x20  I2C Slave Address Register3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |GC        |General Call Function
 * |        |          |0 = Disable General Call Function.
 * |        |          |1 = Enable General Call Function.
 * |[7:1]   |ADDR      |I2C Address Register
 * |        |          |The content of this register is irrelevant when I2C is in master mode
 * |        |          |In the slave mode, the seven most significant bits must be loaded with the MCUu2019s own address
 * |        |          |The I2C hardware will react if any of the addresses are matched.
 * @var I2C_T::ADDRMSK0
 * Offset: 0x24  I2C Slave Address Mask Register0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |ADDRMSK   |I2C Address Mask Register
 * |        |          |0 = Mask disable.
 * |        |          |1 = Mask enable (the received corresponding address bit is donu2019t care.).
 * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers
 * |        |          |Bits in this field mask the ADDRx registers masking bits from the address comparison.
 * @var I2C_T::ADDRMSK1
 * Offset: 0x28  I2C Slave Address Mask Register1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |ADDRMSK   |I2C Address Mask Register
 * |        |          |0 = Mask disable.
 * |        |          |1 = Mask enable (the received corresponding address bit is donu2019t care.).
 * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers
 * |        |          |Bits in this field mask the ADDRx registers masking bits from the address comparison.
 * @var I2C_T::ADDRMSK2
 * Offset: 0x2C  I2C Slave Address Mask Register2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |ADDRMSK   |I2C Address Mask Register
 * |        |          |0 = Mask disable.
 * |        |          |1 = Mask enable (the received corresponding address bit is donu2019t care.).
 * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers
 * |        |          |Bits in this field mask the ADDRx registers masking bits from the address comparison.
 * @var I2C_T::ADDRMSK3
 * Offset: 0x30  I2C Slave Address Mask Register3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:1]   |ADDRMSK   |I2C Address Mask Register
 * |        |          |0 = Mask disable.
 * |        |          |1 = Mask enable (the received corresponding address bit is donu2019t care.).
 * |        |          |I2C bus controllers support multiple-address recognition with four address mask registers
 * |        |          |Bits in this field mask the ADDRx registers masking bits from the address comparison.
 */
    __IO uint32_t CTL;                   /*!< [0x0000] I2C Control Register                                             */
    __IO uint32_t ADDR0;                 /*!< [0x0004] I2C Slave Address Register0                                      */
    __IO uint32_t DAT;                   /*!< [0x0008] I2C DATA Register                                                */
    __I  uint32_t STATUS;                /*!< [0x000c] I2C Status Register                                              */
    __IO uint32_t CLKDIV;                /*!< [0x0010] I2C Clock Divided Register                                       */
    __IO uint32_t TOCTL;                 /*!< [0x0014] I2C Time Out Control Register                                    */
    __IO uint32_t ADDR1;                 /*!< [0x0018] I2C Slave Address Register1                                      */
    __IO uint32_t ADDR2;                 /*!< [0x001c] I2C Slave Address Register2                                      */
    __IO uint32_t ADDR3;                 /*!< [0x0020] I2C Slave Address Register3                                      */
    __IO uint32_t ADDRMSK0;              /*!< [0x0024] I2C Slave Address Mask Register0                                 */
    __IO uint32_t ADDRMSK1;              /*!< [0x0028] I2C Slave Address Mask Register1                                 */
    __IO uint32_t ADDRMSK2;              /*!< [0x002c] I2C Slave Address Mask Register2                                 */
    __IO uint32_t ADDRMSK3;              /*!< [0x0030] I2C Slave Address Mask Register3                                 */

} I2C_T;

/**
    @addtogroup I2C_CONST I2C Bit Field Definition
    Constant Definitions for I2C Controller
@{ */

#define I2C_CTL_AA_Pos                   (2)                                               /*!< I2C_T::CTL: AA Position                */
#define I2C_CTL_AA_Msk                   (0x1ul << I2C_CTL_AA_Pos)                         /*!< I2C_T::CTL: AA Mask                    */

#define I2C_CTL_SI_Pos                   (3)                                               /*!< I2C_T::CTL: SI Position                */
#define I2C_CTL_SI_Msk                   (0x1ul << I2C_CTL_SI_Pos)                         /*!< I2C_T::CTL: SI Mask                    */

#define I2C_CTL_STO_Pos                  (4)                                               /*!< I2C_T::CTL: STO Position               */
#define I2C_CTL_STO_Msk                  (0x1ul << I2C_CTL_STO_Pos)                        /*!< I2C_T::CTL: STO Mask                   */

#define I2C_CTL_STA_Pos                  (5)                                               /*!< I2C_T::CTL: STA Position               */
#define I2C_CTL_STA_Msk                  (0x1ul << I2C_CTL_STA_Pos)                        /*!< I2C_T::CTL: STA Mask                   */

#define I2C_CTL_I2CEN_Pos                (6)                                               /*!< I2C_T::CTL: I2CEN Position             */
#define I2C_CTL_I2CEN_Msk                (0x1ul << I2C_CTL_I2CEN_Pos)                      /*!< I2C_T::CTL: I2CEN Mask                 */

#define I2C_CTL_INTEN_Pos                (7)                                               /*!< I2C_T::CTL: INTEN Position             */
#define I2C_CTL_INTEN_Msk                (0x1ul << I2C_CTL_INTEN_Pos)                      /*!< I2C_T::CTL: INTEN Mask                 */

#define I2C_ADDR0_GC_Pos                 (0)                                               /*!< I2C_T::ADDR0: GC Position              */
#define I2C_ADDR0_GC_Msk                 (0x1ul << I2C_ADDR0_GC_Pos)                       /*!< I2C_T::ADDR0: GC Mask                  */

#define I2C_ADDR0_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR0: ADDR Position            */
#define I2C_ADDR0_ADDR_Msk               (0x7ful << I2C_ADDR0_ADDR_Pos)                    /*!< I2C_T::ADDR0: ADDR Mask                */

#define I2C_DAT_DAT_Pos                  (0)                                               /*!< I2C_T::DAT: DAT Position               */
#define I2C_DAT_DAT_Msk                  (0xfful << I2C_DAT_DAT_Pos)                       /*!< I2C_T::DAT: DAT Mask                   */

#define I2C_STATUS_STATUS_Pos            (0)                                               /*!< I2C_T::STATUS: STATUS Position         */
#define I2C_STATUS_STATUS_Msk            (0xfful << I2C_STATUS_STATUS_Pos)                 /*!< I2C_T::STATUS: STATUS Mask             */

#define I2C_CLKDIV_DIVIDER_Pos           (0)                                               /*!< I2C_T::CLKDIV: DIVIDER Position        */
#define I2C_CLKDIV_DIVIDER_Msk           (0xfful << I2C_CLKDIV_DIVIDER_Pos)                /*!< I2C_T::CLKDIV: DIVIDER Mask            */

#define I2C_TOCTL_TOIF_Pos               (0)                                               /*!< I2C_T::TOCTL: TOIF Position            */
#define I2C_TOCTL_TOIF_Msk               (0x1ul << I2C_TOCTL_TOIF_Pos)                     /*!< I2C_T::TOCTL: TOIF Mask                */

#define I2C_TOCTL_TOCDIV4_Pos            (1)                                               /*!< I2C_T::TOCTL: TOCDIV4 Position         */
#define I2C_TOCTL_TOCDIV4_Msk            (0x1ul << I2C_TOCTL_TOCDIV4_Pos)                  /*!< I2C_T::TOCTL: TOCDIV4 Mask             */

#define I2C_TOCTL_TOCEN_Pos              (2)                                               /*!< I2C_T::TOCTL: TOCEN Position           */
#define I2C_TOCTL_TOCEN_Msk              (0x1ul << I2C_TOCTL_TOCEN_Pos)                    /*!< I2C_T::TOCTL: TOCEN Mask               */

#define I2C_ADDR1_GC_Pos                 (0)                                               /*!< I2C_T::ADDR1: GC Position              */
#define I2C_ADDR1_GC_Msk                 (0x1ul << I2C_ADDR1_GC_Pos)                       /*!< I2C_T::ADDR1: GC Mask                  */

#define I2C_ADDR1_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR1: ADDR Position            */
#define I2C_ADDR1_ADDR_Msk               (0x7ful << I2C_ADDR1_ADDR_Pos)                    /*!< I2C_T::ADDR1: ADDR Mask                */

#define I2C_ADDR2_GC_Pos                 (0)                                               /*!< I2C_T::ADDR2: GC Position              */
#define I2C_ADDR2_GC_Msk                 (0x1ul << I2C_ADDR2_GC_Pos)                       /*!< I2C_T::ADDR2: GC Mask                  */

#define I2C_ADDR2_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR2: ADDR Position            */
#define I2C_ADDR2_ADDR_Msk               (0x7ful << I2C_ADDR2_ADDR_Pos)                    /*!< I2C_T::ADDR2: ADDR Mask                */

#define I2C_ADDR3_GC_Pos                 (0)                                               /*!< I2C_T::ADDR3: GC Position              */
#define I2C_ADDR3_GC_Msk                 (0x1ul << I2C_ADDR3_GC_Pos)                       /*!< I2C_T::ADDR3: GC Mask                  */

#define I2C_ADDR3_ADDR_Pos               (1)                                               /*!< I2C_T::ADDR3: ADDR Position            */
#define I2C_ADDR3_ADDR_Msk               (0x7ful << I2C_ADDR3_ADDR_Pos)                    /*!< I2C_T::ADDR3: ADDR Mask                */

#define I2C_ADDRMSK0_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK0: ADDRMSK Position      */
#define I2C_ADDRMSK0_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK0_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK0: ADDRMSK Mask          */

#define I2C_ADDRMSK1_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK1: ADDRMSK Position      */
#define I2C_ADDRMSK1_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK1_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK1: ADDRMSK Mask          */

#define I2C_ADDRMSK2_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK2: ADDRMSK Position      */
#define I2C_ADDRMSK2_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK2_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK2: ADDRMSK Mask          */

#define I2C_ADDRMSK3_ADDRMSK_Pos         (1)                                               /*!< I2C_T::ADDRMSK3: ADDRMSK Position      */
#define I2C_ADDRMSK3_ADDRMSK_Msk         (0x7ful << I2C_ADDRMSK3_ADDRMSK_Pos)              /*!< I2C_T::ADDRMSK3: ADDRMSK Mask          */

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
 * @var I2S_T::CTL
 * Offset: 0x00  I2S Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |I2SEN     |Enable I2S Controller
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |[1]     |TXEN      |Transmit Enable
 * |        |          |0 = Disable data transmit.
 * |        |          |1 = Enable data transmit. 
 * |[2]     |RXEN      |Receive Enable
 * |        |          |0 = Disable data receive.
 * |        |          |1 = Enable data receive. 
 * |[3]     |MUTE      |Transmit Mute Enable
 * |        |          |0 = Transmit data is shifted from FIFO.
 * |        |          |1= Transmit channel zero.
 * |[5:4]   |WDWIDTH   |Word Width
 * |        |          |This parameter sets the word width of audio data
 * |        |          |See Figure 5-81 FIFO contents for various I2S modes for details of how data is formatted in transmit and receive FIFO.
 * |        |          |00 = data is 8 bit.
 * |        |          |01 = data is 16 bit.
 * |        |          |10 = data is 24 bit.
 * |        |          |11 = data is 32 bit.
 * |[6]     |MONO      |Monaural Data
 * |        |          |This parameter sets whether mono or stereo data is processed
 * |        |          |See Figure 5-81 FIFO contents for various I2S modes for details of how data is formatted in transmit and receive FIFO.
 * |        |          |0 = Data is stereo format.
 * |        |          |1 = Data is monaural format.
 * |[7]     |FORMAT    |Data Format
 * |        |          |0 = I2S data format.
 * |        |          |1 = MSB justified data format.
 * |        |          |See Figure 5-79 I2S Bus Timing Diagram (Format =0) and Figure 5-80 MSB Justified Timing Diagram (Format=1) for timing differences.
 * |[8]     |SLAVE     |Slave Mode
 * |        |          |I2S can operate as a master or slave
 * |        |          |For master mode, I2S_BCLK and I2S_FS pins are outputs and send bit clock and frame sync from I91200
 * |        |          |In slave mode, I2S_BCLK and I2S_FS pins are inputs and bit clock and frame sync are received from external audio device.
 * |        |          |0 = Master mode.
 * |        |          |1 = Slave mode. 
 * |[11:9]  |TXTH      |Transmit FIFO Threshold Level
 * |        |          |If remaining data words in transmit FIFO less than or equal to the threshold level then TXTHI flag is set.
 * |        |          |Threshold = TXTH words remaining in transmit FIFO.
 * |[14:12] |RXTH      |Receive FIFO Threshold Level
 * |        |          |When received data word(s) in buffer is equal or higher than threshold level then RXTHI flag is set.
 * |        |          |Threshold = RXTH+1 words of data in receive FIFO.
 * |[15]    |MCLKEN    |Master Clock Enable
 * |        |          |The I91200can generate a master clock signal to an external audio CODEC to synchronize the audio devices
 * |        |          |If audio devices are not synchronous, then data will be periodically corrupted
 * |        |          |Software needs to implement a way to drop/repeat or interpolate samples in a jitter buffer if devices are not synchronized
 * |        |          |The master clock frequency is determined by the I2S_CLKDIV.MCLKDIV[2:0] register.
 * |        |          |0 = Disable master clock.
 * |        |          |1 = Enable master clock.
 * |[16]    |RZCEN     |Right Channel Zero Cross Detect Enable
 * |        |          |If this bit is set to 1, when right channel data sign bit changes, or data bits are all zero, the RZCIF flag in I2S_STATUS register will be set to 1.
 * |        |          |0 = Disable right channel zero cross detect.
 * |        |          |1 = Enable right channel zero cross detect.
 * |[17]    |LZCEN     |Left Channel Zero Cross Detect Enable
 * |        |          |If this bit is set to 1, when left channel data sign bit changes, or data bits are all zero, the LZCIF flag in I2S_STATUS register will be set to 1.
 * |        |          |0 = Disable left channel zero cross detect.
 * |        |          |1 = Enable left channel zero cross detect.
 * |[18]    |TXCLR     |Clear Transmit FIFO
 * |        |          |Write 1 to clear transmit FIFO, internal pointer is reset to FIFO start point, and I2S_STATUS.TXCNT[3:0] returns to zero and transmit FIFO becomes empty
 * |        |          |Data in transmit FIFO is not changed.
 * |        |          |This bit is cleared by hardware automatically when clear operation complete.
 * |[19]    |RXCLR     |Clear Receive FIFO
 * |        |          |Write 1 to clear receive FIFO, internal pointer is reset to FIFO start point, and I2S_STATUS.RXCNT[3:0] returns to zero and receive FIFO becomes empty.
 * |        |          |This bit is cleared by hardware automatically when clear operation complete.
 * |[20]    |TXPDMAEN  |Enable Transmit DMA
 * |        |          |When TX DMA is enables, I2S request DMA to transfer data from SRAM to transmit FIFO if FIFO is not full.
 * |        |          |0 = Disable TX DMA.
 * |        |          |1 = Enable TX DMA.
 * |[21]    |RXPDMAEN  |Enable Receive DMA
 * |        |          |When RX DMA is enabled, I2S requests DMA to transfer data from receive FIFO to SRAM if FIFO is not empty.
 * |        |          |0 = Disable RX DMA.
 * |        |          |1 = Enable RX DMA.
 * @var I2S_T::CLKDIV
 * Offset: 0x04  I2S Clock Divider Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |MCLKDIV   |Master Clock Divider
 * |        |          |I91200can generate a master clock to synchronously drive an external audio device
 * |        |          |If MCLKDIV is set to 0, MCLK is the same as I2S_CLKDIV clock input, otherwise MCLK frequency is given by:
 * |        |          |FMCLK = F I2S_CLKDIV / (2 x MCLKDIV).
 * |        |          |Or,
 * |        |          |MCLKDIV = F I2S_CLKDIV / (2 x FMCLK).
 * |        |          |If the desired MCLK frequency is 4.092MHz (= 256Fs) and Fs = 16kHz then MCLKDIV = 6 @ F I2S_CLKDIV = 49.152MHz.
 * |[15:8]  |BCLKDIV   |Bit Clock Divider
 * |        |          |If I2S operates in master mode, bit clock is provided by I91200
 * |        |          |Software can program these bits to generate bit clock frequency for the desired sample rate.
 * |        |          |For sample rate Fs, the desired bit clock frequency is:
 * |        |          |FBCLK = Fs x Word_width_in_bytes x 16.
 * |        |          |For example if Fs = 16kHz, and word width is 2-bytes (16bit) then desired bit clock frequency is 512kHz.
 * |        |          |The bit clock frequency is given by:
 * |        |          |FBCLK = FI2S_CLKDIV / (2x (BCLKDIV+1)).
 * |        |          |Or,
 * |        |          |BCLKDIV = F I2S_CLKDIV / (2 x FBCLK) -1.
 * |        |          |So if F I2S_CLKDIV = HCLK = 49.152MHz, desired FBCLK = 512kHzthen BCLKDIV = 47.
 * @var I2S_T::IEN
 * Offset: 0x08  I2S Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RXUDIEN   |Receive FIFO Underflow Interrupt Enable
 * |        |          |If software read receive FIFO when it is empty then RXUDIF flag in I2SSTATUS register is set to 1.
 * |        |          |0 = Disable interrupt.
 * |        |          |1 = Enable interrupt.
 * |[1]     |RXOVIEN   |Receive FIFO Overflow Interrupt Enable
 * |        |          |0 = Disable interrupt.
 * |        |          |1 = Enable interrupt.
 * |[2]     |RXTHIEN   |Receive FIFO Threshold Level Interrupt
 * |        |          |Interrupt occurs if this bit is set to 1 and data words in receive FIFO is greater than or equal to RXTH[2:0].
 * |        |          |0 = Disable interrupt.
 * |        |          |1 = Enable interrupt.
 * |[8]     |TXUDIEN   |Transmit FIFO Underflow Interrupt Enable
 * |        |          |Interrupt occur if this bit is set to 1 and transmit FIFO underflow flag is set to 1.
 * |        |          |0 = Disable interrupt.
 * |        |          |1 = Enable interrupt.
 * |[9]     |TXOVIEN   |Transmit FIFO Overflow Interrupt Enable
 * |        |          |Interrupt occurs if this bit is set to 1 and transmit FIFO overflow flag is set to 1
 * |        |          |0 = Disable interrupt.
 * |        |          |1 = Enable interrupt.
 * |[10]    |TXTHIEN   |Transmit FIFO Threshold Level Interrupt Enable
 * |        |          |Interrupt occurs if this bit is set to 1 and data words in transmit FIFO is less than TXTH[2:0].
 * |        |          |0 = Disable interrupt.
 * |        |          |1 = Enable interrupt.
 * |[11]    |RZCIEN    |Right Channel Zero Cross Interrupt Enable
 * |        |          |Interrupt will occur if this bit is set to 1 and right channel has zero cross event
 * |        |          |0 = Disable interrupt.
 * |        |          |1 = Enable interrupt.
 * |[12]    |LZCIEN    |Left Channel Zero Cross Interrupt Enable
 * |        |          |Interrupt will occur if this bit is set to 1 and left channel has zero cross event
 * |        |          |0 = Disable interrupt.
 * |        |          |1 = Enable interrupt.
 * @var I2S_T::STATUS
 * Offset: 0x0C  I2S Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |I2SIF     |I2S Interrupt (Read Only)
 * |        |          |This bit is set if any enabled I2S interrupt is active.
 * |        |          |0 = No I2S interrupt.
 * |        |          |1 = I2S interrupt active.
 * |[1]     |RXIF      |I2S Receive Interrupt (Read Only)
 * |        |          |This indicates that there is an active receive interrupt source
 * |        |          |This could be RXOVIF, RXUDIF or RXTHIF if corresponding interrupt enable bits are active
 * |        |          |To clear interrupt the corresponding source(s) must be cleared.
 * |        |          |0 = No receive interrupt.
 * |        |          |1 = Receive interrupt occurred.
 * |[2]     |TXIF      |I2S Transmit Interrupt (Read Only)
 * |        |          |This indicates that there is an active transmit interrupt source
 * |        |          |This could be TXOVIF, TXUDIF, TXTHIF, LZCIF or RZCIF if corresponding interrupt enable bits are active
 * |        |          |To clear interrupt the corresponding source(s) must be cleared.
 * |        |          |0 = No transmit interrupt.
 * |        |          |1 = Transmit interrupt occurred.
 * |[3]     |RIGHT     |Right Channel Active (Read Only)
 * |        |          |This bit indicates current data being transmitted/received belongs to right channel
 * |        |          |0 = Left channel.
 * |        |          |1 = Right channel.
 * |[8]     |RXUDIF    |Receive FIFO Underflow Flag (Write u20181u2019 to Clear)
 * |        |          |This flag is set if attempt is made to read receive FIFO while it is empty.
 * |        |          |0 = No underflow.
 * |        |          |1 = Underflow.
 * |[9]     |RXOVIF    |Receive FIFO Overflow Flag (Write u20181u2019 to Clear)
 * |        |          |This flag is set if I2S controller writes to receive FIFO when it is full. Audio data is lost.
 * |        |          |0 = No overflow.
 * |        |          |1 = Overflow.
 * |[10]    |RXTHIF    |Receive FIFO Threshold Flag (Read Only)
 * |        |          |When data word(s) in receive FIFO is greater than or equal to threshold value set in RXTH[2:0] the RXTHIF bit becomes to 1
 * |        |          |It remains set until receive FIFO level is less than RXTH[2:0]
 * |        |          |It is cleared by reading I2S_RX until threshold satisfied.
 * |        |          |0 = Data word(s) in FIFO is less than threshold level.
 * |        |          |1 = Data word(s) in FIFO is greater than or equal to threshold level.
 * |[11]    |RXFULL    |Receive FIFO Full (Read Only)
 * |        |          |This bit is set when receive FIFO is full.
 * |        |          |0 = Not full.
 * |        |          |1 = Full.
 * |[12]    |RXEMPTY   |Receive FIFO Empty (Read Only)
 * |        |          |This is set when receive FIFO is empty.
 * |        |          |0 = Not empty.
 * |        |          |1 = Empty.
 * |[16]    |TXUDIF    |Transmit FIFO Underflow Flag (Write u20181u2019 to Clear)
 * |        |          |This flag is set if I2S controller requests data when transmit FIFO is empty.
 * |        |          |0 = No underflow.
 * |        |          |1 = Underflow.
 * |[17]    |TXOVIF    |Transmit FIFO Overflow Flag (Write u20181u2019 to Clear)
 * |        |          |This flag is set if data is written to transmit FIFO when it is full.
 * |        |          |0 = No overflow.
 * |        |          |1 = Overflow.
 * |[18]    |TXTHIF    |Transmit FIFO Threshold Flag (Read Only)
 * |        |          |When data word(s) in transmit FIFO is less than or equal to the threshold value set in TXTH[2:0] the TXTHIF bit becomes to 1
 * |        |          |It remains set until transmit FIFO level is greater than TXTH[2:0]
 * |        |          |Cleared by writing to I2S_TX register until threshold exceeded.
 * |        |          |0 = Data word(s) in FIFO is greater than threshold level.
 * |        |          |1 = Data word(s) in FIFO is less than or equal to threshold level.
 * |[19]    |TXFULL    |Transmit FIFO Full (Read Only)
 * |        |          |This bit is set when transmit FIFO is full.
 * |        |          |0 = Not full.
 * |        |          |1 = Full.
 * |[20]    |TXEMPTY   |Transmit FIFO Empty (Read Only)
 * |        |          |This is set when transmit FIFO is empty.
 * |        |          |0 = Not empty.
 * |        |          |1 = Empty.
 * |[21]    |TXBUSY    |Transmit Busy (Read Only)
 * |        |          |This bit is cleared when all data in transmit FIFO and Tx shift register is shifted out
 * |        |          |It is set when first data is loaded to Tx shift register.
 * |        |          |0 = Transmit shift register is empty.
 * |        |          |1 = Transmit shift register is busy.
 * |[22]    |RZCIF     |Right Channel Zero Cross Flag (Write u20181u2019 to Clear, or Clear RZCEN)
 * |        |          |0 = No zero cross.
 * |        |          |1 = Right channel zero cross is detected.
 * |[23]    |LZCIF     |Left Channel Zero Cross Flag (Write u20181u2019 to Clear, or Clear LZCEN)
 * |        |          |0 = No zero cross detected.
 * |        |          |1 = Left channel zero cross is detected.
 * |[27:24] |RXCNT     |Receive FIFO Level (Read Only)
 * |        |          |RXCNT = number of words in receive FIFO.
 * |[31:28] |TXCNT     |Transmit FIFO Level (Read Only)
 * |        |          |TXCNT = number of words in transmit FIFO.
 * @var I2S_T::TX
 * Offset: 0x10  I2S Transmit FIFO Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |TX        |Transmit FIFO Register (Write Only)
 * |        |          |A write to this register pushes data onto the transmit FIFO
 * |        |          |The transmit FIFO is eight words deep
 * |        |          |The number of words currently in the FIFO can be determined by reading I2S_STATUS.TXCNT
 * @var I2S_T::RX
 * Offset: 0x14  I2S Receive FIFO Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |RX        |Receive FIFO Register (Read Only)
 * |        |          |A read of this register will pop data from the receive FIFO
 * |        |          |The receive FIFO is eight words deep
 * |        |          |The number of words currently in the FIFO can be determined by reading I2S_STATUS.RXCNT
 */
    __IO uint32_t CTL;                   /*!< [0x0000] I2S Control Register                                             */
    __IO uint32_t CLKDIV;                   /*!< [0x0004] I2S Clock Divider Register                                       */
    __IO uint32_t IEN;                   /*!< [0x0008] I2S Interrupt Enable Register                                    */
    __IO uint32_t STATUS;                /*!< [0x000c] I2S Status Register                                              */
    __O  uint32_t TX;                    /*!< [0x0010] I2S Transmit FIFO Register                                       */
    __I  uint32_t RX;                    /*!< [0x0014] I2S Receive FIFO Register                                        */

} I2S_T;

/**
    @addtogroup I2S_CONST I2S Bit Field Definition
    Constant Definitions for I2S Controller
@{ */

#define I2S_CTL_I2SEN_Pos                (0)                                               /*!< I2S_T::CTL: I2SEN Position             */
#define I2S_CTL_I2SEN_Msk                (0x1ul << I2S_CTL_I2SEN_Pos)                      /*!< I2S_T::CTL: I2SEN Mask                 */

#define I2S_CTL_TXEN_Pos                 (1)                                               /*!< I2S_T::CTL: TXEN Position              */
#define I2S_CTL_TXEN_Msk                 (0x1ul << I2S_CTL_TXEN_Pos)                       /*!< I2S_T::CTL: TXEN Mask                  */

#define I2S_CTL_RXEN_Pos                 (2)                                               /*!< I2S_T::CTL: RXEN Position              */
#define I2S_CTL_RXEN_Msk                 (0x1ul << I2S_CTL_RXEN_Pos)                       /*!< I2S_T::CTL: RXEN Mask                  */

#define I2S_CTL_MUTE_Pos                 (3)                                               /*!< I2S_T::CTL: MUTE Position              */
#define I2S_CTL_MUTE_Msk                 (0x1ul << I2S_CTL_MUTE_Pos)                       /*!< I2S_T::CTL: MUTE Mask                  */

#define I2S_CTL_WDWIDTH_Pos              (4)                                               /*!< I2S_T::CTL: WDWIDTH Position           */
#define I2S_CTL_WDWIDTH_Msk              (0x3ul << I2S_CTL_WDWIDTH_Pos)                    /*!< I2S_T::CTL: WDWIDTH Mask               */

#define I2S_CTL_MONO_Pos                 (6)                                               /*!< I2S_T::CTL: MONO Position              */
#define I2S_CTL_MONO_Msk                 (0x1ul << I2S_CTL_MONO_Pos)                       /*!< I2S_T::CTL: MONO Mask                  */

#define I2S_CTL_FORMAT_Pos               (7)                                               /*!< I2S_T::CTL: FORMAT Position            */
#define I2S_CTL_FORMAT_Msk               (0x1ul << I2S_CTL_FORMAT_Pos)                     /*!< I2S_T::CTL: FORMAT Mask                */

#define I2S_CTL_SLAVE_Pos                (8)                                               /*!< I2S_T::CTL: SLAVE Position             */
#define I2S_CTL_SLAVE_Msk                (0x1ul << I2S_CTL_SLAVE_Pos)                      /*!< I2S_T::CTL: SLAVE Mask                 */

#define I2S_CTL_TXTH_Pos                 (9)                                               /*!< I2S_T::CTL: TXTH Position              */
#define I2S_CTL_TXTH_Msk                 (0x7ul << I2S_CTL_TXTH_Pos)                       /*!< I2S_T::CTL: TXTH Mask                  */

#define I2S_CTL_RXTH_Pos                 (12)                                              /*!< I2S_T::CTL: RXTH Position              */
#define I2S_CTL_RXTH_Msk                 (0x7ul << I2S_CTL_RXTH_Pos)                       /*!< I2S_T::CTL: RXTH Mask                  */

#define I2S_CTL_MCLKEN_Pos               (15)                                              /*!< I2S_T::CTL: MCLKEN Position            */
#define I2S_CTL_MCLKEN_Msk               (0x1ul << I2S_CTL_MCLKEN_Pos)                     /*!< I2S_T::CTL: MCLKEN Mask                */

#define I2S_CTL_RZCEN_Pos                (16)                                              /*!< I2S_T::CTL: RZCEN Position             */
#define I2S_CTL_RZCEN_Msk                (0x1ul << I2S_CTL_RZCEN_Pos)                      /*!< I2S_T::CTL: RZCEN Mask                 */

#define I2S_CTL_LZCEN_Pos                (17)                                              /*!< I2S_T::CTL: LZCEN Position             */
#define I2S_CTL_LZCEN_Msk                (0x1ul << I2S_CTL_LZCEN_Pos)                      /*!< I2S_T::CTL: LZCEN Mask                 */

#define I2S_CTL_TXCLR_Pos                (18)                                              /*!< I2S_T::CTL: TXCLR Position             */
#define I2S_CTL_TXCLR_Msk                (0x1ul << I2S_CTL_TXCLR_Pos)                      /*!< I2S_T::CTL: TXCLR Mask                 */

#define I2S_CTL_RXCLR_Pos                (19)                                              /*!< I2S_T::CTL: RXCLR Position             */
#define I2S_CTL_RXCLR_Msk                (0x1ul << I2S_CTL_RXCLR_Pos)                      /*!< I2S_T::CTL: RXCLR Mask                 */

#define I2S_CTL_TXPDMAEN_Pos             (20)                                              /*!< I2S_T::CTL: TXPDMAEN Position          */
#define I2S_CTL_TXPDMAEN_Msk             (0x1ul << I2S_CTL_TXPDMAEN_Pos)                   /*!< I2S_T::CTL: TXPDMAEN Mask              */

#define I2S_CTL_RXPDMAEN_Pos             (21)                                              /*!< I2S_T::CTL: RXPDMAEN Position          */
#define I2S_CTL_RXPDMAEN_Msk             (0x1ul << I2S_CTL_RXPDMAEN_Pos)                   /*!< I2S_T::CTL: RXPDMAEN Mask              */

#define I2S_CLKDIV_MCLKDIV_Pos           (0)                                               /*!< I2S_T::CLKDIV: MCLKDIV Position           */
#define I2S_CLKDIV_MCLKDIV_Msk           (0x7ul << I2S_CLKDIV_MCLKDIV_Pos)                 /*!< I2S_T::CLKDIV: MCLKDIV Mask               */

#define I2S_CLKDIV_BCLKDIV_Pos           (8)                                               /*!< I2S_T::CLKDIV: BCLKDIV Position           */
#define I2S_CLKDIV_BCLKDIV_Msk           (0xfful << I2S_CLKDIV_BCLKDIV_Pos)                /*!< I2S_T::CLKDIV: BCLKDIV Mask               */

#define I2S_IEN_RXUDIEN_Pos              (0)                                               /*!< I2S_T::IEN: RXUDIEN Position           */
#define I2S_IEN_RXUDIEN_Msk              (0x1ul << I2S_IEN_RXUDIEN_Pos)                    /*!< I2S_T::IEN: RXUDIEN Mask               */

#define I2S_IEN_RXOVIEN_Pos              (1)                                               /*!< I2S_T::IEN: RXOVIEN Position           */
#define I2S_IEN_RXOVIEN_Msk              (0x1ul << I2S_IEN_RXOVIEN_Pos)                    /*!< I2S_T::IEN: RXOVIEN Mask               */

#define I2S_IEN_RXTHIEN_Pos              (2)                                               /*!< I2S_T::IEN: RXTHIEN Position           */
#define I2S_IEN_RXTHIEN_Msk              (0x1ul << I2S_IEN_RXTHIEN_Pos)                    /*!< I2S_T::IEN: RXTHIEN Mask               */

#define I2S_IEN_TXUDIEN_Pos              (8)                                               /*!< I2S_T::IEN: TXUDIEN Position           */
#define I2S_IEN_TXUDIEN_Msk              (0x1ul << I2S_IEN_TXUDIEN_Pos)                    /*!< I2S_T::IEN: TXUDIEN Mask               */

#define I2S_IEN_TXOVIEN_Pos              (9)                                               /*!< I2S_T::IEN: TXOVIEN Position           */
#define I2S_IEN_TXOVIEN_Msk              (0x1ul << I2S_IEN_TXOVIEN_Pos)                    /*!< I2S_T::IEN: TXOVIEN Mask               */

#define I2S_IEN_TXTHIEN_Pos              (10)                                              /*!< I2S_T::IEN: TXTHIEN Position           */
#define I2S_IEN_TXTHIEN_Msk              (0x1ul << I2S_IEN_TXTHIEN_Pos)                    /*!< I2S_T::IEN: TXTHIEN Mask               */

#define I2S_IEN_RZCIEN_Pos               (11)                                              /*!< I2S_T::IEN: RZCIEN Position            */
#define I2S_IEN_RZCIEN_Msk               (0x1ul << I2S_IEN_RZCIEN_Pos)                     /*!< I2S_T::IEN: RZCIEN Mask                */

#define I2S_IEN_LZCIEN_Pos               (12)                                              /*!< I2S_T::IEN: LZCIEN Position            */
#define I2S_IEN_LZCIEN_Msk               (0x1ul << I2S_IEN_LZCIEN_Pos)                     /*!< I2S_T::IEN: LZCIEN Mask                */

#define I2S_STATUS_I2SIF_Pos             (0)                                               /*!< I2S_T::STATUS: I2SIF Position          */
#define I2S_STATUS_I2SIF_Msk             (0x1ul << I2S_STATUS_I2SIF_Pos)                   /*!< I2S_T::STATUS: I2SIF Mask              */

#define I2S_STATUS_RXIF_Pos              (1)                                               /*!< I2S_T::STATUS: RXIF Position           */
#define I2S_STATUS_RXIF_Msk              (0x1ul << I2S_STATUS_RXIF_Pos)                    /*!< I2S_T::STATUS: RXIF Mask               */

#define I2S_STATUS_TXIF_Pos              (2)                                               /*!< I2S_T::STATUS: TXIF Position           */
#define I2S_STATUS_TXIF_Msk              (0x1ul << I2S_STATUS_TXIF_Pos)                    /*!< I2S_T::STATUS: TXIF Mask               */

#define I2S_STATUS_RIGHT_Pos             (3)                                               /*!< I2S_T::STATUS: RIGHT Position          */
#define I2S_STATUS_RIGHT_Msk             (0x1ul << I2S_STATUS_RIGHT_Pos)                   /*!< I2S_T::STATUS: RIGHT Mask              */

#define I2S_STATUS_RXUDIF_Pos            (8)                                               /*!< I2S_T::STATUS: RXUDIF Position         */
#define I2S_STATUS_RXUDIF_Msk            (0x1ul << I2S_STATUS_RXUDIF_Pos)                  /*!< I2S_T::STATUS: RXUDIF Mask             */

#define I2S_STATUS_RXOVIF_Pos            (9)                                               /*!< I2S_T::STATUS: RXOVIF Position         */
#define I2S_STATUS_RXOVIF_Msk            (0x1ul << I2S_STATUS_RXOVIF_Pos)                  /*!< I2S_T::STATUS: RXOVIF Mask             */

#define I2S_STATUS_RXTHIF_Pos            (10)                                              /*!< I2S_T::STATUS: RXTHIF Position         */
#define I2S_STATUS_RXTHIF_Msk            (0x1ul << I2S_STATUS_RXTHIF_Pos)                  /*!< I2S_T::STATUS: RXTHIF Mask             */

#define I2S_STATUS_RXFULL_Pos            (11)                                              /*!< I2S_T::STATUS: RXFULL Position         */
#define I2S_STATUS_RXFULL_Msk            (0x1ul << I2S_STATUS_RXFULL_Pos)                  /*!< I2S_T::STATUS: RXFULL Mask             */

#define I2S_STATUS_RXEMPTY_Pos           (12)                                              /*!< I2S_T::STATUS: RXEMPTY Position        */
#define I2S_STATUS_RXEMPTY_Msk           (0x1ul << I2S_STATUS_RXEMPTY_Pos)                 /*!< I2S_T::STATUS: RXEMPTY Mask            */

#define I2S_STATUS_TXUDIF_Pos            (16)                                              /*!< I2S_T::STATUS: TXUDIF Position         */
#define I2S_STATUS_TXUDIF_Msk            (0x1ul << I2S_STATUS_TXUDIF_Pos)                  /*!< I2S_T::STATUS: TXUDIF Mask             */

#define I2S_STATUS_TXOVIF_Pos            (17)                                              /*!< I2S_T::STATUS: TXOVIF Position         */
#define I2S_STATUS_TXOVIF_Msk            (0x1ul << I2S_STATUS_TXOVIF_Pos)                  /*!< I2S_T::STATUS: TXOVIF Mask             */

#define I2S_STATUS_TXTHIF_Pos            (18)                                              /*!< I2S_T::STATUS: TXTHIF Position         */
#define I2S_STATUS_TXTHIF_Msk            (0x1ul << I2S_STATUS_TXTHIF_Pos)                  /*!< I2S_T::STATUS: TXTHIF Mask             */

#define I2S_STATUS_TXFULL_Pos            (19)                                              /*!< I2S_T::STATUS: TXFULL Position         */
#define I2S_STATUS_TXFULL_Msk            (0x1ul << I2S_STATUS_TXFULL_Pos)                  /*!< I2S_T::STATUS: TXFULL Mask             */

#define I2S_STATUS_TXEMPTY_Pos           (20)                                              /*!< I2S_T::STATUS: TXEMPTY Position        */
#define I2S_STATUS_TXEMPTY_Msk           (0x1ul << I2S_STATUS_TXEMPTY_Pos)                 /*!< I2S_T::STATUS: TXEMPTY Mask            */

#define I2S_STATUS_TXBUSY_Pos            (21)                                              /*!< I2S_T::STATUS: TXBUSY Position         */
#define I2S_STATUS_TXBUSY_Msk            (0x1ul << I2S_STATUS_TXBUSY_Pos)                  /*!< I2S_T::STATUS: TXBUSY Mask             */

#define I2S_STATUS_RZCIF_Pos             (22)                                              /*!< I2S_T::STATUS: RZCIF Position          */
#define I2S_STATUS_RZCIF_Msk             (0x1ul << I2S_STATUS_RZCIF_Pos)                   /*!< I2S_T::STATUS: RZCIF Mask              */

#define I2S_STATUS_LZCIF_Pos             (23)                                              /*!< I2S_T::STATUS: LZCIF Position          */
#define I2S_STATUS_LZCIF_Msk             (0x1ul << I2S_STATUS_LZCIF_Pos)                   /*!< I2S_T::STATUS: LZCIF Mask              */

#define I2S_STATUS_RXCNT_Pos             (24)                                              /*!< I2S_T::STATUS: RXCNT Position          */
#define I2S_STATUS_RXCNT_Msk             (0xful << I2S_STATUS_RXCNT_Pos)                   /*!< I2S_T::STATUS: RXCNT Mask              */

#define I2S_STATUS_TXCNT_Pos             (28)                                              /*!< I2S_T::STATUS: TXCNT Position          */
#define I2S_STATUS_TXCNT_Msk             (0xful << I2S_STATUS_TXCNT_Pos)                   /*!< I2S_T::STATUS: TXCNT Mask              */

#define I2S_TX_TX_Pos                    (0)                                               /*!< I2S_T::TX: TX Position                 */
#define I2S_TX_TX_Msk                    (0xfffffffful << I2S_TX_TX_Pos)                   /*!< I2S_T::TX: TX Mask                     */

#define I2S_RX_RX_Pos                    (0)                                               /*!< I2S_T::RX: RX Position                 */
#define I2S_RX_RX_Msk                    (0xfffffffful << I2S_RX_RX_Pos)                   /*!< I2S_T::RX: RX Mask                     */

/**@}*/ /* I2S_CONST */
/**@}*/ /* end of I2S register group */


/*---------------------- Interrupt Source Register -------------------------*/
/**
    @addtogroup INT Interrupt Source Register(INT)
    Memory Mapped Structure for INT Controller
@{ */
 
typedef struct
{


/**
 * @var INT_T::IRQ0_SRC
 * Offset: 0x00  IRQ0 (BOD) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: BOD_INT
 * @var INT_T::IRQ1_SRC
 * Offset: 0x04  IRQ1 (WDT) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: WDT_INT
 * @var INT_T::IRQ2_SRC
 * Offset: 0x08  IRQ2 (EINT0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: INT0_INT
 * @var INT_T::IRQ3_SRC
 * Offset: 0x0C  IRQ3 (EINT1) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: INT0_INT
 * @var INT_T::IRQ4_SRC
 * Offset: 0x10  IRQ4 (GPA/B) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: GPB_INT
 * |        |          |Bit0: GPA_INT
 * @var INT_T::IRQ5_SRC
 * Offset: 0x14  IRQ5 (ALC) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: ALC_INT
 * @var INT_T::IRQ6_SRC
 * Offset: 0x18  IRQ6 (PWM0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: PWM_INT
 * @var INT_T::IRQ7_SRC
 * Offset: 0x1C  IRQ7 (Reserved) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var INT_T::IRQ8_SRC
 * Offset: 0x20  IRQ8 (TMR0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: TMR0_INT
 * @var INT_T::IRQ9_SRC
 * Offset: 0x24  IRQ9 (TMR1) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: TMR1_INT
 * @var INT_T::IRQ10_SRC
 * Offset: 0x28  IRQ10 (Reserved) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var INT_T::IRQ11_SRC
 * Offset: 0x2C  IRQ11 (UART1) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INTSRC    |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: UART1 INT
 * @var INT_T::IRQ12_SRC
 * Offset: 0x30  IRQ12 (UART0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: UART0_INT
 * @var INT_T::IRQ13_SRC
 * Offset: 0x34  IRQ13 (SPI1) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: SPI1 INT
 * @var INT_T::IRQ14_SRC
 * Offset: 0x38  IRQ14 (SPI0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: SPI0_INT
 * @var INT_T::IRQ15_SRC
 * Offset: 0x3C  IRQ15 (DPWM) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1:
 * |        |          |Bit0: DPWM INT
 * @var INT_T::IRQ16_SRC
 * Offset: 0x40  IRQ16 (Reserved) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var INT_T::IRQ17_SRC
 * Offset: 0x44  IRQ17 (Reserved) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var INT_T::IRQ18_SRC
 * Offset: 0x48  IRQ18 (I2C0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: I2C0_INT
 * @var INT_T::IRQ19_SRC
 * Offset: 0x4C  IRQ19 (Reserved) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var INT_T::IRQ20_SRC
 * Offset: 0x50  IRQ20 (Reserved) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var INT_T::IRQ21_SRC
 * Offset: 0x54  IRQ21 (CMP) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: CMP INT
 * @var INT_T::IRQ22_SRC
 * Offset: 0x58  IRQ22 (MAC ) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: MAC INT
 * @var INT_T::IRQ23_SRC
 * Offset: 0x5C  IRQ23 (Reserved) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var INT_T::IRQ24_SRC
 * Offset: 0x60  IRQ24 (Reserved) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var INT_T::IRQ25_SRC
 * Offset: 0x64  IRQ25 (SARADC) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: SARADC INT
 * @var INT_T::IRQ26_SRC
 * Offset: 0x68  IRQ26 (PDMA) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: PDMA_INT
 * @var INT_T::IRQ27_SRC
 * Offset: 0x6C  IRQ27 (I2S0) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: I2S_INT
 * @var INT_T::IRQ28_SRC
 * Offset: 0x70  IRQ28 (CAPS) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: CAPS_INT
 * @var INT_T::IRQ29_SRC
 * Offset: 0x74  IRQ29 (ADC) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: ADC_INT
 * @var INT_T::IRQ30_SRC
 * Offset: 0x78  IRQ30 (Reserved) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var INT_T::IRQ31_SRC
 * Offset: 0x7C  IRQ31 (RTC) Interrupt Source Identity Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |INT_SRC   |Interrupt Source Identity
 * |        |          |Bit2: 0
 * |        |          |Bit1: 0
 * |        |          |Bit0: RTC_INT
 * @var INT_T::NMI_SEL
 * Offset: 0x80  NMI Source Interrupt Select Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[4:0]   |NMI_SEL   |NMI Source Interrupt Select
 * |        |          |The NMI interrupt to Cortex-M0 can be selected from one of the interrupt[31:0]
 * |        |          |The NMI_SEL bit[4:0] used to select the NMI interrupt source
 * |[7]     |IRQ_TM    |IRQ Test Mode
 * |        |          |If set to 1 then peripheral IRQ signals (0-31) are replaced by the value in the MCU_IRQ register
 * |        |          |This is a protected register to program first issue the unlock sequence.
 * @var INT_T::MCU_IRQ
 * Offset: 0x84  MCU IRQ Number Identify Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BOD       |IRQ0 (BOD) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[1]     |WDT       |IRQ1 (WDT) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[2]     |EINT0     |IRQ2 (EINT0) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[3]     |EINT1     |IRQ3 (EINT1) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[4]     |GPAB      |IRQ4 (GPA/B) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[5]     |ALC       |IRQ5 (ALC) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[6]     |PWM       |IRQ6 (PWM0) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[8]     |TMR0      |IRQ8 (TMR0) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[9]     |TMR1      |RQ9 (TMR1) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[11]    |UART1     |IRQ11 (UART1) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[12]    |UART0     |IRQ12 (UART0) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[13]    |SPI1      |IRQ13 (SPI1) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[14]    |SPI0      |IRQ14 (SPI0) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[15]    |DPWM      |IRQ15 (DPWM) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[18]    |I2C       |IRQ18 (I2C0) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[21]    |CMP       |IRQ21 (CMP) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[22]    |MAC       |IRQ22 (MAC ) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[25]    |SARADC    |IRQ25 (SARADC) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[26]    |PDMA      |IRQ26 (PDMA) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[27]    |I2S       |IRQ27 (I2S0) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[28]    |CAPS      |IRQ28 (CAPS) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[29]    |SDADC     |IRQ29 (SDADC) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 * |[31]    |RTC       |IRQ31 (RTC) Interrupt Source Identity Register
 * |        |          |0: No effect.
 * |        |          |1: clear the interrupt
 */
    __I  uint32_t IRQ0_SRC;              /*!< [0x0000] IRQ0 (BOD) Interrupt Source Identity Register                    */
    __I  uint32_t IRQ1_SRC;              /*!< [0x0004] IRQ1 (WDT) Interrupt Source Identity Register                    */
    __I  uint32_t IRQ2_SRC;              /*!< [0x0008] IRQ2 (EINT0) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ3_SRC;              /*!< [0x000c] IRQ3 (EINT1) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ4_SRC;              /*!< [0x0010] IRQ4 (GPA/B) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ5_SRC;              /*!< [0x0014] IRQ5 (ALC) Interrupt Source Identity Register                    */
    __I  uint32_t IRQ6_SRC;              /*!< [0x0018] IRQ6 (PWM0) Interrupt Source Identity Register                   */
    __IO uint32_t IRQ7_SRC;              /*!< [0x001c] IRQ7 (Reserved) Interrupt Source Identity Register               */
    __I  uint32_t IRQ8_SRC;              /*!< [0x0020] IRQ8 (TMR0) Interrupt Source Identity Register                   */
    __I  uint32_t IRQ9_SRC;              /*!< [0x0024] IRQ9 (TMR1) Interrupt Source Identity Register                   */
    __IO uint32_t IRQ10_SRC;             /*!< [0x0028] IRQ10 (Reserved) Interrupt Source Identity Register              */
    __I  uint32_t IRQ11_SRC;             /*!< [0x002c] IRQ11 (UART1) Interrupt Source Identity Register                 */
    __I  uint32_t IRQ12_SRC;             /*!< [0x0030] IRQ12 (UART0) Interrupt Source Identity Register                 */
    __I  uint32_t IRQ13_SRC;             /*!< [0x0034] IRQ13 (SPI1) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ14_SRC;             /*!< [0x0038] IRQ14 (SPI0) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ15_SRC;             /*!< [0x003c] IRQ15 (DPWM) Interrupt Source Identity Register                  */
    __IO uint32_t IRQ16_SRC;             /*!< [0x0040] IRQ16 (Reserved) Interrupt Source Identity Register              */
    __IO uint32_t IRQ17_SRC;             /*!< [0x0044] IRQ17 (Reserved) Interrupt Source Identity Register              */
    __I  uint32_t IRQ18_SRC;             /*!< [0x0048] IRQ18 (I2C0) Interrupt Source Identity Register                  */
    __IO uint32_t IRQ19_SRC;             /*!< [0x004c] IRQ19 (Reserved) Interrupt Source Identity Register              */
    __IO uint32_t IRQ20_SRC;             /*!< [0x0050] IRQ20 (Reserved) Interrupt Source Identity Register              */
    __I  uint32_t IRQ21_SRC;             /*!< [0x0054] IRQ21 (CMP) Interrupt Source Identity Register                   */
    __I  uint32_t IRQ22_SRC;             /*!< [0x0058] IRQ22 (MAC ) Interrupt Source Identity Register                  */
    __IO uint32_t IRQ23_SRC;             /*!< [0x005c] IRQ23 (Reserved) Interrupt Source Identity Register              */
    __IO uint32_t IRQ24_SRC;             /*!< [0x0060] IRQ24 (Reserved) Interrupt Source Identity Register              */
    __I  uint32_t IRQ25_SRC;             /*!< [0x0064] IRQ25 (SARADC) Interrupt Source Identity Register                */
    __I  uint32_t IRQ26_SRC;             /*!< [0x0068] IRQ26 (PDMA) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ27_SRC;             /*!< [0x006c] IRQ27 (I2S0) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ28_SRC;             /*!< [0x0070] IRQ28 (CAPS) Interrupt Source Identity Register                  */
    __I  uint32_t IRQ29_SRC;             /*!< [0x0074] IRQ29 (ADC) Interrupt Source Identity Register                   */
    __IO uint32_t IRQ30_SRC;             /*!< [0x0078] IRQ30 (Reserved) Interrupt Source Identity Register              */
    __I  uint32_t IRQ31_SRC;             /*!< [0x007c] IRQ31 (RTC) Interrupt Source Identity Register                   */
    __IO uint32_t NMI_SEL;               /*!< [0x0080] NMI Source Interrupt Select Control Register                     */
    __IO uint32_t MCU_IRQ;               /*!< [0x0084] MCU IRQ Number Identify Register                                 */

} INT_T;

/**
    @addtogroup INT_CONST INT Bit Field Definition
    Constant Definitions for INT Controller
@{ */

#define INT_IRQ0SRC_INTSRC_Pos           (0)                                               /*!< INT_T::IRQ0SRC: INTSRC Position        */
#define INT_IRQ0SRC_INTSRC_Msk           (0x7ul << INT_IRQ0SRC_INTSRC_Pos)                 /*!< INT_T::IRQ0SRC: INTSRC Mask            */

#define INT_IRQ1SRC_INTSRC_Pos           (0)                                               /*!< INT_T::IRQ1SRC: INTSRC Position        */
#define INT_IRQ1SRC_INTSRC_Msk           (0x7ul << INT_IRQ1SRC_INTSRC_Pos)                 /*!< INT_T::IRQ1SRC: INTSRC Mask            */

#define INT_IRQ2SRC_INTSRC_Pos           (0)                                               /*!< INT_T::IRQ2SRC: INTSRC Position        */
#define INT_IRQ2SRC_INTSRC_Msk           (0x7ul << INT_IRQ2SRC_INTSRC_Pos)                 /*!< INT_T::IRQ2SRC: INTSRC Mask            */

#define INT_IRQ3SRC_INTSRC_Pos           (0)                                               /*!< INT_T::IRQ3SRC: INTSRC Position        */
#define INT_IRQ3SRC_INTSRC_Msk           (0x7ul << INT_IRQ3SRC_INTSRC_Pos)                 /*!< INT_T::IRQ3SRC: INTSRC Mask            */

#define INT_IRQ4SRC_INTSRC_Pos           (0)                                               /*!< INT_T::IRQ4SRC: INTSRC Position        */
#define INT_IRQ4SRC_INTSRC_Msk           (0x7ul << INT_IRQ4SRC_INTSRC_Pos)                 /*!< INT_T::IRQ4SRC: INTSRC Mask            */

#define INT_IRQ5SRC_INTSRC_Pos           (0)                                               /*!< INT_T::IRQ5SRC: INTSRC Position        */
#define INT_IRQ5SRC_INTSRC_Msk           (0x7ul << INT_IRQ5SRC_INTSRC_Pos)                 /*!< INT_T::IRQ5SRC: INTSRC Mask            */

#define INT_IRQ6SRC_INTSRC_Pos           (0)                                               /*!< INT_T::IRQ6SRC: INTSRC Position        */
#define INT_IRQ6SRC_INTSRC_Msk           (0xful << INT_IRQ6SRC_INTSRC_Pos)                 /*!< INT_T::IRQ6SRC: INTSRC Mask            */

#define INT_IRQ8SRC_INTSRC_Pos           (0)                                               /*!< INT_T::IRQ8SRC: INTSRC Position        */
#define INT_IRQ8SRC_INTSRC_Msk           (0x7ul << INT_IRQ8SRC_INTSRC_Pos)                 /*!< INT_T::IRQ8SRC: INTSRC Mask            */

#define INT_IRQ9SRC_INTSRC_Pos           (0)                                               /*!< INT_T::IRQ9SRC: INTSRC Position        */
#define INT_IRQ9SRC_INTSRC_Msk           (0x7ul << INT_IRQ9SRC_INTSRC_Pos)                 /*!< INT_T::IRQ9SRC: INTSRC Mask            */

#define INT_IRQ11SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ11SRC: INTSRC Position       */
#define INT_IRQ11SRC_INTSRC_Msk          (0x7ul << INT_IRQ11SRC_INTSRC_Pos)                /*!< INT_T::IRQ11SRC: INTSRC Mask           */

#define INT_IRQ12SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ12SRC: INTSRC Position       */
#define INT_IRQ12SRC_INTSRC_Msk          (0x7ul << INT_IRQ12SRC_INTSRC_Pos)                /*!< INT_T::IRQ12SRC: INTSRC Mask           */

#define INT_IRQ13SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ13SRC: INTSRC Position       */
#define INT_IRQ13SRC_INTSRC_Msk          (0x7ul << INT_IRQ13SRC_INTSRC_Pos)                /*!< INT_T::IRQ13SRC: INTSRC Mask           */

#define INT_IRQ14SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ14SRC: INTSRC Position       */
#define INT_IRQ14SRC_INTSRC_Msk          (0x7ul << INT_IRQ14SRC_INTSRC_Pos)                /*!< INT_T::IRQ14SRC: INTSRC Mask           */

#define INT_IRQ15SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ15SRC: INTSRC Position       */
#define INT_IRQ15SRC_INTSRC_Msk          (0x7ul << INT_IRQ15SRC_INTSRC_Pos)                /*!< INT_T::IRQ15SRC: INTSRC Mask           */

#define INT_IRQ18SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ18SRC: INTSRC Position       */
#define INT_IRQ18SRC_INTSRC_Msk          (0x7ul << INT_IRQ18SRC_INTSRC_Pos)                /*!< INT_T::IRQ18SRC: INTSRC Mask           */

#define INT_IRQ21SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ21SRC: INTSRC Position       */
#define INT_IRQ21SRC_INTSRC_Msk          (0x7ul << INT_IRQ21SRC_INTSRC_Pos)                /*!< INT_T::IRQ21SRC: INTSRC Mask           */

#define INT_IRQ22SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ22SRC: INTSRC Position       */
#define INT_IRQ22SRC_INTSRC_Msk          (0x7ul << INT_IRQ22SRC_INTSRC_Pos)                /*!< INT_T::IRQ22SRC: INTSRC Mask           */

#define INT_IRQ25SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ25SRC: INTSRC Position       */
#define INT_IRQ25SRC_INTSRC_Msk          (0x7ul << INT_IRQ25SRC_INTSRC_Pos)                /*!< INT_T::IRQ25SRC: INTSRC Mask           */

#define INT_IRQ26SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ26SRC: INTSRC Position       */
#define INT_IRQ26SRC_INTSRC_Msk          (0x7ul << INT_IRQ26SRC_INTSRC_Pos)                /*!< INT_T::IRQ26SRC: INTSRC Mask           */

#define INT_IRQ27SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ27SRC: INTSRC Position       */
#define INT_IRQ27SRC_INTSRC_Msk          (0x7ul << INT_IRQ27SRC_INTSRC_Pos)                /*!< INT_T::IRQ27SRC: INTSRC Mask           */

#define INT_IRQ28SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ28SRC: INTSRC Position       */
#define INT_IRQ28SRC_INTSRC_Msk          (0x7ul << INT_IRQ28SRC_INTSRC_Pos)                /*!< INT_T::IRQ28SRC: INTSRC Mask           */

#define INT_IRQ29SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ29SRC: INTSRC Position       */
#define INT_IRQ29SRC_INTSRC_Msk          (0x7ul << INT_IRQ29SRC_INTSRC_Pos)                /*!< INT_T::IRQ29SRC: INTSRC Mask           */

#define INT_IRQ31SRC_INTSRC_Pos          (0)                                               /*!< INT_T::IRQ31SRC: INTSRC Position       */
#define INT_IRQ31SRC_INTSRC_Msk          (0x7ul << INT_IRQ31SRC_INTSRC_Pos)                /*!< INT_T::IRQ31SRC: INTSRC Mask           */

#define INT_SEL_NMI_SEL_Pos              (0)                                               /*!< INT_T::SEL: NMI_SEL Position           */
#define INT_SEL_NMI_SEL_Msk              (0x1ful << INT_SEL_NMI_SEL_Pos)                   /*!< INT_T::SEL: NMI_SEL Mask               */

#define INT_SEL_IRQ_TM_Pos               (7)                                               /*!< INT_T::SEL: IRQ_TM Position            */
#define INT_SEL_IRQ_TM_Msk               (0x1ul << INT_SEL_IRQ_TM_Pos)                     /*!< INT_T::SEL: IRQ_TM Mask                */

#define INT_IRQ_BOD_Pos                  (0)                                               /*!< INT_T::IRQ: BOD Position               */
#define INT_IRQ_BOD_Msk                  (0x1ul << INT_IRQ_BOD_Pos)                        /*!< INT_T::IRQ: BOD Mask                   */

#define INT_IRQ_WDT_Pos                  (1)                                               /*!< INT_T::IRQ: WDT Position               */
#define INT_IRQ_WDT_Msk                  (0x1ul << INT_IRQ_WDT_Pos)                        /*!< INT_T::IRQ: WDT Mask                   */

#define INT_IRQ_EINT0_Pos                (2)                                               /*!< INT_T::IRQ: EINT0 Position             */
#define INT_IRQ_EINT0_Msk                (0x1ul << INT_IRQ_EINT0_Pos)                      /*!< INT_T::IRQ: EINT0 Mask                 */

#define INT_IRQ_EINT1_Pos                (3)                                               /*!< INT_T::IRQ: EINT1 Position             */
#define INT_IRQ_EINT1_Msk                (0x1ul << INT_IRQ_EINT1_Pos)                      /*!< INT_T::IRQ: EINT1 Mask                 */

#define INT_IRQ_GPAB_Pos                 (4)                                               /*!< INT_T::IRQ: GPAB Position              */
#define INT_IRQ_GPAB_Msk                 (0x1ul << INT_IRQ_GPAB_Pos)                       /*!< INT_T::IRQ: GPAB Mask                  */

#define INT_IRQ_ALC_Pos                  (5)                                               /*!< INT_T::IRQ: ALC Position               */
#define INT_IRQ_ALC_Msk                  (0x1ul << INT_IRQ_ALC_Pos)                        /*!< INT_T::IRQ: ALC Mask                   */

#define INT_IRQ_PWM_Pos                  (6)                                               /*!< INT_T::IRQ: PWM Position               */
#define INT_IRQ_PWM_Msk                  (0x1ul << INT_IRQ_PWM_Pos)                        /*!< INT_T::IRQ: PWM Mask                   */

#define INT_IRQ_TMR0_Pos                 (8)                                               /*!< INT_T::IRQ: TMR0 Position              */
#define INT_IRQ_TMR0_Msk                 (0x1ul << INT_IRQ_TMR0_Pos)                       /*!< INT_T::IRQ: TMR0 Mask                  */

#define INT_IRQ_TMR1_Pos                 (9)                                               /*!< INT_T::IRQ: TMR1 Position              */
#define INT_IRQ_TMR1_Msk                 (0x1ul << INT_IRQ_TMR1_Pos)                       /*!< INT_T::IRQ: TMR1 Mask                  */

#define INT_IRQ_UART1_Pos                (11)                                              /*!< INT_T::IRQ: UART1 Position             */
#define INT_IRQ_UART1_Msk                (0x1ul << INT_IRQ_UART1_Pos)                      /*!< INT_T::IRQ: UART1 Mask                 */

#define INT_IRQ_UART0_Pos                (12)                                              /*!< INT_T::IRQ: UART0 Position             */
#define INT_IRQ_UART0_Msk                (0x1ul << INT_IRQ_UART0_Pos)                      /*!< INT_T::IRQ: UART0 Mask                 */

#define INT_IRQ_SPI1_Pos                 (13)                                              /*!< INT_T::IRQ: SPI1 Position              */
#define INT_IRQ_SPI1_Msk                 (0x1ul << INT_IRQ_SPI1_Pos)                       /*!< INT_T::IRQ: SPI1 Mask                  */

#define INT_IRQ_SPI0_Pos                 (14)                                              /*!< INT_T::IRQ: SPI0 Position              */
#define INT_IRQ_SPI0_Msk                 (0x1ul << INT_IRQ_SPI0_Pos)                       /*!< INT_T::IRQ: SPI0 Mask                  */

#define INT_IRQ_DPWM_Pos                 (15)                                              /*!< INT_T::IRQ: DPWM Position              */
#define INT_IRQ_DPWM_Msk                 (0x1ul << INT_IRQ_DPWM_Pos)                       /*!< INT_T::IRQ: DPWM Mask                  */

#define INT_IRQ_I2C_Pos                  (18)                                              /*!< INT_T::IRQ: I2C Position               */
#define INT_IRQ_I2C_Msk                  (0x1ul << INT_IRQ_I2C_Pos)                        /*!< INT_T::IRQ: I2C Mask                   */

#define INT_IRQ_CMP_Pos                  (21)                                              /*!< INT_T::IRQ: CMP Position               */
#define INT_IRQ_CMP_Msk                  (0x1ul << INT_IRQ_CMP_Pos)                        /*!< INT_T::IRQ: CMP Mask                   */

#define INT_IRQ_MAC_Pos                  (22)                                              /*!< INT_T::IRQ: MAC Position               */
#define INT_IRQ_MAC_Msk                  (0x1ul << INT_IRQ_MAC_Pos)                        /*!< INT_T::IRQ: MAC Mask                   */

#define INT_IRQ_SARADC_Pos               (25)                                              /*!< INT_T::IRQ: SARADC Position            */
#define INT_IRQ_SARADC_Msk               (0x1ul << INT_IRQ_SARADC_Pos)                     /*!< INT_T::IRQ: SARADC Mask                */

#define INT_IRQ_PDMA_Pos                 (26)                                              /*!< INT_T::IRQ: PDMA Position              */
#define INT_IRQ_PDMA_Msk                 (0x1ul << INT_IRQ_PDMA_Pos)                       /*!< INT_T::IRQ: PDMA Mask                  */

#define INT_IRQ_I2S_Pos                  (27)                                              /*!< INT_T::IRQ: I2S Position               */
#define INT_IRQ_I2S_Msk                  (0x1ul << INT_IRQ_I2S_Pos)                        /*!< INT_T::IRQ: I2S Mask                   */

#define INT_IRQ_CAPS_Pos                 (28)                                              /*!< INT_T::IRQ: CAPS Position              */
#define INT_IRQ_CAPS_Msk                 (0x1ul << INT_IRQ_CAPS_Pos)                       /*!< INT_T::IRQ: CAPS Mask                  */

#define INT_IRQ_SDADC_Pos                (29)                                              /*!< INT_T::IRQ: SDADC Position             */
#define INT_IRQ_SDADC_Msk                (0x1ul << INT_IRQ_SDADC_Pos)                      /*!< INT_T::IRQ: SDADC Mask                 */

#define INT_IRQ_RTC_Pos                  (31)                                              /*!< INT_T::IRQ: RTC Position               */
#define INT_IRQ_RTC_Msk                  (0x1ul << INT_IRQ_RTC_Pos)                        /*!< INT_T::IRQ: RTC Mask                   */

/**@}*/ /* INT_CONST */
/**@}*/ /* end of INT register group */


/*---------------------- Peripheral Direct Memory Access Controller -------------------------*/
/**
    @addtogroup PDMA Peripheral Direct Memory Access Controller(PDMA)
    Memory Mapped Structure for PDMA Controller
@{ */
 
typedef struct
{


/**
 * @var PDMAC_T::GCTL
 * Offset: 0x00  PDMA Global Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SWRST     |PDMA Software Reset
 * |        |          |0 = Writing 0 to this bit has no effect.
 * |        |          |1 = Writing 1 to this bit will reset the internal state machine and pointers
 * |        |          |The contents of control register will not be cleared
 * |        |          |This bit will auto clear after several clock cycles.
 * |        |          |Note: This bit can reset all channels (global reset).
 * |[8]     |CH0CKEN   |PDMA Controller Channel 0 Clock Enable Control
 * |        |          |1: Enable Channel 0 clock.
 * |        |          |0: Disable Channel 0 clock.
 * |[9]     |CH1CKEN   |PDMA Controller Channel 1 Clock Enable Control
 * |        |          |1: Enable Channel 1 clock.
 * |        |          |0: Disable Channel 1 clock.
 * |[10]    |CH2CKEN   |PDMA Controller Channel 2 Clock Enable Control
 * |        |          |1: Enable Channel 2 clock.
 * |        |          |0: Disable Channel 2 clock.
 * |[11]    |CH3CKEN   |PDMA Controller Channel 3 Clock Enable Control
 * |        |          |1: Enable Channel 3 clock.
 * |        |          |0: Disable Channel 3 clock.
 * @var PDMAC_T::SVCSEL0
 * Offset: 0x04  PDMA Service Selection Control Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |SPI0RXSEL |PDMA SPI0 Receive Selection
 * |        |          |This field defines which PDMA channel is connected to SPI0 peripheral receive (PDMA source) request.
 * |[7:4]   |SPI0TXSEL |PDMA SPI0 Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to SPI0 peripheral transmit (PDMA destination) request.
 * |[11:8]  |SDADCRXSEL|PDMA SDADC Receive Selection
 * |        |          |This field defines which PDMA channel is connected to SDADC peripheral receive (PDMA source) request.
 * |[15:12] |DPWMTXSEL |PDMA DPWM Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to DPWM peripheral transmit (PDMA destination) request.
 * |[19:16] |UART0RXSEL|PDMA UART0 Receive Selection
 * |        |          |This field defines which PDMA channel is connected to UART0 peripheral receive (PDMA source) request.
 * |[23:20] |UART0XSEL |PDMA UART0 Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to UART0 peripheral transmit (PDMA destination) request
 * |[27:24] |I2SRXSEL  |PDMA I2S Receive Selection
 * |        |          |This field defines which PDMA channel is connected to I2S peripheral receive (PDMA source) request.
 * |[31:28] |I2STXSEL  |PDMA I2S Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to I2S peripheral transmit (PDMA destination) request
 * @var PDMAC_T::SVCSEL1
 * Offset: 0x08  PDMA Service Selection Control Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |UART1RXSEL|PDMA UART1 Receive Selection
 * |        |          |This field defines which PDMA channel is connected to UART1 peripheral receive (PDMA source) request.
 * |[7:4]   |UART1TXSEL|PDMA UART1 Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to UART1 peripheral transmit (PDMA destination) request
 * |[11:8]  |SPI1RXSEL |PDMA SPI1 Receive Selection
 * |        |          |This field defines which PDMA channel is connected to SPI1 peripheral receive (PDMA source) request.
 * |[15:12] |SPI1TXSEL |PDMA SPI1 Transmit Selection
 * |        |          |This field defines which PDMA channel is connected to SPI1 peripheral transmit (PDMA destination) request.
 * |[19:16] |SARADCRXSEL|PDMA SARADC Receive Selection
 * |        |          |This field defines which PDMA channel is connected to SARADC peripheral receive (PDMA source) request.
 * @var PDMAC_T::GINTSTS
 * Offset: 0x0C  PDMA Global Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CH0INTSTS |Interrupt Pin Status of Channel 0 (Read Only)
 * |        |          |This bit is the interrupt pin status of PDMA channel 0.
 * |[1]     |CH1INTSTS |Interrupt Pin Status of Channel 1 (Read Only)
 * |        |          |This bit is the interrupt pin status of PDMA channel 1.
 * |[2]     |CH2INTSTS |Interrupt Pin Status of Channel 2 (Read Only)
 * |        |          |This bit is the interrupt pin status of PDMA channel 2.
 * |[3]     |CH3INTSTS |Interrupt Pin Status of Channel 3 (Read Only)
 * |        |          |This bit is the interrupt pin status of PDMA channel 3.
 */
    __IO uint32_t GCTL;                  /*!< [0x0000] PDMA Global Control Register                                     */
    __IO uint32_t SVCSEL0;               /*!< [0x0004] PDMA Service Selection Control Register 0                        */
    __IO uint32_t SVCSEL1;               /*!< [0x0008] PDMA Service Selection Control Register 1                        */
    __I  uint32_t GINTSTS;               /*!< [0x000c] PDMA Global Interrupt Status Register                            */

} PDMAC_T;

/**
    @addtogroup PDMA_CONST PDMA Bit Field Definition
    Constant Definitions for PDMA Controller
@{ */

#define PDMA_GCTL_SWRST_Pos              (0)                                               /*!< PDMAC_T::GCTL: SWRST Position           */
#define PDMA_GCTL_SWRST_Msk              (0x1ul << PDMA_GCTL_SWRST_Pos)                    /*!< PDMAC_T::GCTL: SWRST Mask               */

#define PDMA_GCTL_CH0CKEN_Pos            (8)                                               /*!< PDMAC_T::GCTL: CH0CKEN Position         */
#define PDMA_GCTL_CH0CKEN_Msk            (0x1ul << PDMA_GCTL_CH0CKEN_Pos)                  /*!< PDMAC_T::GCTL: CH0CKEN Mask             */

#define PDMA_GCTL_CH1CKEN_Pos            (9)                                               /*!< PDMAC_T::GCTL: CH1CKEN Position         */
#define PDMA_GCTL_CH1CKEN_Msk            (0x1ul << PDMA_GCTL_CH1CKEN_Pos)                  /*!< PDMAC_T::GCTL: CH1CKEN Mask             */

#define PDMA_GCTL_CH2CKEN_Pos            (10)                                              /*!< PDMAC_T::GCTL: CH2CKEN Position         */
#define PDMA_GCTL_CH2CKEN_Msk            (0x1ul << PDMA_GCTL_CH2CKEN_Pos)                  /*!< PDMAC_T::GCTL: CH2CKEN Mask             */

#define PDMA_GCTL_CH3CKEN_Pos            (11)                                              /*!< PDMAC_T::GCTL: CH3CKEN Position         */
#define PDMA_GCTL_CH3CKEN_Msk            (0x1ul << PDMA_GCTL_CH3CKEN_Pos)                  /*!< PDMAC_T::GCTL: CH3CKEN Mask             */

#define PDMA_SVCSEL0_SPI0RXSEL_Pos       (0)                                               /*!< PDMAC_T::SVCSEL0: SPI0RXSEL Position    */
#define PDMA_SVCSEL0_SPI0RXSEL_Msk       (0xful << PDMA_SVCSEL0_SPI0RXSEL_Pos)             /*!< PDMAC_T::SVCSEL0: SPI0RXSEL Mask        */

#define PDMA_SVCSEL0_SPI0TXSEL_Pos       (4)                                               /*!< PDMAC_T::SVCSEL0: SPI0TXSEL Position    */
#define PDMA_SVCSEL0_SPI0TXSEL_Msk       (0xful << PDMA_SVCSEL0_SPI0TXSEL_Pos)             /*!< PDMAC_T::SVCSEL0: SPI0TXSEL Mask        */

#define PDMA_SVCSEL0_SDADCRXSEL_Pos      (8)                                               /*!< PDMAC_T::SVCSEL0: SDADCRXSEL Position   */
#define PDMA_SVCSEL0_SDADCRXSEL_Msk      (0xful << PDMA_SVCSEL0_SDADCRXSEL_Pos)            /*!< PDMAC_T::SVCSEL0: SDADCRXSEL Mask       */

#define PDMA_SVCSEL0_DPWMTXSEL_Pos       (12)                                              /*!< PDMAC_T::SVCSEL0: DPWMTXSEL Position    */
#define PDMA_SVCSEL0_DPWMTXSEL_Msk       (0xful << PDMA_SVCSEL0_DPWMTXSEL_Pos)             /*!< PDMAC_T::SVCSEL0: DPWMTXSEL Mask        */

#define PDMA_SVCSEL0_UART0RXSEL_Pos      (16)                                              /*!< PDMAC_T::SVCSEL0: UART0RXSEL Position   */
#define PDMA_SVCSEL0_UART0RXSEL_Msk      (0xful << PDMA_SVCSEL0_UART0RXSEL_Pos)            /*!< PDMAC_T::SVCSEL0: UART0RXSEL Mask       */

#define PDMA_SVCSEL0_UART0XSEL_Pos       (20)                                              /*!< PDMAC_T::SVCSEL0: UART0XSEL Position    */
#define PDMA_SVCSEL0_UART0XSEL_Msk       (0xful << PDMA_SVCSEL0_UART0XSEL_Pos)             /*!< PDMAC_T::SVCSEL0: UART0XSEL Mask        */

#define PDMA_SVCSEL0_I2SRXSEL_Pos        (24)                                              /*!< PDMAC_T::SVCSEL0: I2SRXSEL Position     */
#define PDMA_SVCSEL0_I2SRXSEL_Msk        (0xful << PDMA_SVCSEL0_I2SRXSEL_Pos)              /*!< PDMAC_T::SVCSEL0: I2SRXSEL Mask         */

#define PDMA_SVCSEL0_I2STXSEL_Pos        (28)                                              /*!< PDMAC_T::SVCSEL0: I2STXSEL Position     */
#define PDMA_SVCSEL0_I2STXSEL_Msk        (0xful << PDMA_SVCSEL0_I2STXSEL_Pos)              /*!< PDMAC_T::SVCSEL0: I2STXSEL Mask         */

#define PDMA_SVCSEL1_UART1RXSEL_Pos      (0)                                               /*!< PDMAC_T::SVCSEL1: UART1RXSEL Position   */
#define PDMA_SVCSEL1_UART1RXSEL_Msk      (0xful << PDMA_SVCSEL1_UART1RXSEL_Pos)            /*!< PDMAC_T::SVCSEL1: UART1RXSEL Mask       */

#define PDMA_SVCSEL1_UART1TXSEL_Pos      (4)                                               /*!< PDMAC_T::SVCSEL1: UART1TXSEL Position    */
#define PDMA_SVCSEL1_UART1TXSEL_Msk      (0xful << PDMA_SVCSEL1_UART1TXSEL_Pos)            /*!< PDMAC_T::SVCSEL1: UART1TXSEL Mask        */

#define PDMA_SVCSEL1_SPI1RXSEL_Pos       (8)                                               /*!< PDMAC_T::SVCSEL1: SPI1RXSEL Position    */
#define PDMA_SVCSEL1_SPI1RXSEL_Msk       (0xful << PDMA_SVCSEL1_SPI1RXSEL_Pos)             /*!< PDMAC_T::SVCSEL1: SPI1RXSEL Mask        */

#define PDMA_SVCSEL1_SPI1TXSEL_Pos       (12)                                              /*!< PDMAC_T::SVCSEL1: SPI1TXSEL Position    */
#define PDMA_SVCSEL1_SPI1TXSEL_Msk       (0xful << PDMA_SVCSEL1_SPI1TXSEL_Pos)             /*!< PDMAC_T::SVCSEL1: SPI1TXSEL Mask        */

#define PDMA_SVCSEL1_SARADCRXSEL_Pos     (16)                                              /*!< PDMAC_T::SVCSEL1: SARADCRXSEL Position  */
#define PDMA_SVCSEL1_SARADCRXSEL_Msk     (0xful << PDMA_SVCSEL1_SARADCRXSEL_Pos)           /*!< PDMAC_T::SVCSEL1: SARADCRXSEL Mask      */

#define PDMA_GINTSTS_CH0INTSTS_Pos       (0)                                               /*!< PDMAC_T::GINTSTS: CH0INTSTS Position    */
#define PDMA_GINTSTS_CH0INTSTS_Msk       (0x1ul << PDMA_GINTSTS_CH0INTSTS_Pos)             /*!< PDMAC_T::GINTSTS: CH0INTSTS Mask        */

#define PDMA_GINTSTS_CH1INTSTS_Pos       (1)                                               /*!< PDMAC_T::GINTSTS: CH1INTSTS Position    */
#define PDMA_GINTSTS_CH1INTSTS_Msk       (0x1ul << PDMA_GINTSTS_CH1INTSTS_Pos)             /*!< PDMAC_T::GINTSTS: CH1INTSTS Mask        */

#define PDMA_GINTSTS_CH2INTSTS_Pos       (2)                                               /*!< PDMAC_T::GINTSTS: CH2INTSTS Position    */
#define PDMA_GINTSTS_CH2INTSTS_Msk       (0x1ul << PDMA_GINTSTS_CH2INTSTS_Pos)             /*!< PDMAC_T::GINTSTS: CH2INTSTS Mask        */

#define PDMA_GINTSTS_CH3INTSTS_Pos       (3)                                               /*!< PDMAC_T::GINTSTS: CH3INTSTS Position    */
#define PDMA_GINTSTS_CH3INTSTS_Msk       (0x1ul << PDMA_GINTSTS_CH3INTSTS_Pos)             /*!< PDMAC_T::GINTSTS: CH3INTSTS Mask        */

/**@}*/ /* PDMAC_CONST */
/**@}*/ /* end of PDMAC register group */


/*---------------------- Peripheral Direct Memory Access Controller -------------------------*/
/**
    @addtogroup PDMA Peripheral Direct Memory Access Controller(PDMA)
    Memory Mapped Structure for PDMA Controller
@{ */
 
typedef struct
{


/**
 * @var PDMA_T::CTL
 * Offset: 0x00  PDMA Control Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CHEN      |PDMA Channel Enable
 * |        |          |Setting this bit to 1 enables PDMAu2019s operation
 * |        |          |If this bit is cleared, PDMA will ignore all PDMA request and force Bus Master into IDLE state.
 * |        |          |Note: SWRST will clear this bit.
 * |[1]     |SWRST     |Software Engine Reset
 * |        |          |0 = Writing 0 to this bit has no effect.
 * |        |          |1 = Writing 1 to this bit will reset the internal state machine and pointers
 * |        |          |The contents of the control register will not be cleared
 * |        |          |This bit will auto clear after a few clock cycles.
 * |[3:2]   |MODESEL   |PDMA Mode Select
 * |        |          |This parameter selects to transfer direction of the PDMA channel. Possible values are:
 * |        |          |00 = Memory to Memory mode (SRAM-to-SRAM).
 * |        |          |01 = IP to Memory mode (APB-to-SRAM).
 * |        |          |10 = Memory to IP mode (SRAM-to-APB).
 * |[5:4]   |SASEL     |Source Address Select
 * |        |          |This parameter determines the behavior of the current source address register with each PDMA transfer
 * |        |          |It can either be fixed, incremented or wrapped.
 * |        |          |00 = Transfer Source address is incremented.
 * |        |          |01 = Reserved.
 * |        |          |10 = Transfer Source address is fixed.
 * |        |          |11 = Transfer Source address is wrapped.
 * |        |          |When PDMA_CURTXCNTn (Current Byte Count) equals zero, the PDMA_CURSADDRn (Current Source Address) and PDMA_CURTXCNTn registers will be reloaded from the PDMA_SADDRn (Source Address) and PDMA_TXCNT (Byte Count) registers automatically and PDMA will start another transfer.
 * |        |          |Cycle continues until software sets PDMA_CTLn.CHEN = 0.
 * |        |          |When PDMA_CTLn.CHEN is disabled, the PDMA will complete the active transfer but the remaining data in the SBUF will not be transferred to the destination address.
 * |[7:6]   |DASEL     |Destination Address Select
 * |        |          |This parameter determines the behavior of the current destination address register with each PDMA transfer
 * |        |          |It can either be fixed, incremented or wrapped.
 * |        |          |00 = Transfer Destination Address is incremented.
 * |        |          |01 = Reserved.
 * |        |          |10 = Transfer Destination Address is fixed (Used when data transferred from multiple addresses to a single destination such as peripheral FIFO input).
 * |        |          |11 = Transfer Destination Address is wrapped.
 * |        |          |When PDMA_CURTXCNTn (Current Byte Count) equals zero, the PDMA_CURDADDR (Current Destination Address) and PDMA_CURTXCNTn registers will be reloaded from the PDMA_DADDRn (Destination Address) and PDMA_TXCNTn (Byte Count) registers automatically and PDMA will start another transfer.
 * |        |          |Cycle continues until software sets PDMA_CTLn.CHEN =0.
 * |        |          |When PDMA_CTLn.CHEN is disabled, the PDMA will complete the active transfer but the remaining data in the SBUF will not be transferred to the destination address.
 * |[15:12] |WAINTSEL  |Wrap Interrupt Select
 * |        |          |x1xx: If this bit is set, and wraparound mode is in operation a Wrap Interrupt can be generated when half each PDMA transfer is complete
 * |        |          |For example if PDMA_TXCNTn = 32 then an interrupt could be generated when 16 bytes were sent.
 * |        |          |xxx1: If this bit is set, and wraparound mode is in operation a Wrap Interrupt can be generated when each PDMA transfer is wrapped
 * |        |          |For example if PDMA_TXCNTn = 32 then an interrupt could be generated when 32 bytes were sent and PDMA wraps around.
 * |        |          |x1x1: Both half and w interrupts generated.
 * |[20:19] |TXWIDTH   |Peripheral Transfer Width Select
 * |        |          |This parameter determines the data width to be transferred each PDMA transfer operation.
 * |        |          |00 = One word (32 bits) is transferred for every PDMA operation.
 * |        |          |01 = One byte (8 bits) is transferred for every PDMA operation.
 * |        |          |10 = One half-word (16 bits) is transferred for every PDMA operation.
 * |        |          |11 = Reserved.
 * |        |          |Note: This field is meaningful only when MODESEL is IP to Memory mode (APB-to-Memory) or Memory to IP mode (Memory-to-APB).
 * |[23]    |TXEN      |Trigger Enable u2013 Start a PDMA Operation
 * |        |          |0 = Write: no effect. Read: Idle/Finished.
 * |        |          |1 = Enable PDMA data read or write transfer.
 * |        |          |Note: When PDMA transfer completed, this bit will be cleared automatically.
 * |        |          |If a bus error occurs, all PDMA transfer will be stopped
 * |        |          |Software must reset PDMA channel, and then trigger again.
 * @var PDMA_T::SADDR
 * Offset: 0x04  PDMA Transfer Source Address Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ADDR      |PDMA Transfer Source Address Register
 * |        |          |This register holds the initial Source Address of PDMA transfer.
 * |        |          |Note: The source address must be word aligned.
 * @var PDMA_T::DADDR
 * Offset: 0x08  PDMA Transfer Destination Address Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ADDR      |PDMA Transfer Destination Address Register
 * |        |          |This register holds the initial Destination Address of PDMA transfer.
 * |        |          |Note: The destination address must be word aligned.
 * @var PDMA_T::TXCNT
 * Offset: 0x0C  PDMA Transfer Byte Count Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNT       |PDMA Transfer Byte Count Register
 * |        |          |This register controls the transfer byte count of PDMA. Maximum value is 0xFFFF.
 * |        |          |Note: When in memory-to-memory (TXBCCHn.MODESEL = 00b) mode, the transfer byte count must be word aligned, that is multiples of 4bytes.
 * @var PDMA_T::INTPNT
 * Offset: 0x10  PDMA Internal Buffer Pointer Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |POINTER   |PDMA Internal Buffer Pointer Register (Read Only)
 * |        |          |A PDMA transaction consists of two stages, a read from the source address and a write to the destination address
 * |        |          |Internally this data is buffered in a 32bit register
 * |        |          |If transaction width between the read and write transactions are different, this register tracks which byte/half-word of the internal buffer is being processed by the current transaction.
 * @var PDMA_T::CURSADDR
 * Offset: 0x14  PDMA Current Source Address Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ADDR      |PDMA Current Source Address Register (Read Only)
 * |        |          |This register returns the source address from which the PDMA transfer is occurring.
 * |        |          |This register is loaded from PDMA_SADDRn when PDMA is triggered or when a wraparound occurs.
 * @var PDMA_T::CURDADDR
 * Offset: 0x18  PDMA Current Destination Address Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |ADDR      |PDMA Current Destination Address Register (Read Only)
 * |        |          |This register returns the destination address to which the PDMA transfer is occurring.
 * |        |          |This register is loaded from PDMA_DADDRn when PDMA is triggered or when a wraparound occurs.
 * @var PDMA_T::CURTXCNT
 * Offset: 0x1C  PDMA Current Transfer Byte Count Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNT       |PDMA Current Byte Count Register (Read Only)
 * |        |          |This field indicates the current remaining byte count of PDMA transfer
 * |        |          |This register is initialized with CNT register when PDMA is triggered or when a wraparound occurs
 * @var PDMA_T::INTEN
 * Offset: 0x20  PDMA Interrupt Enable Control Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ABTIEN    |PDMA Read/Write Target Abort Interrupt Enable
 * |        |          |If enabled, the PDMA controller will generate and interrupt to the CPU whenever a PDMA transaction is aborted due to an error
 * |        |          |If a transfer is aborted, PDMA channel must be reset to resume DMA operation.
 * |        |          |0 = Disable PDMA transfer target abort interrupt generation.
 * |        |          |1 = Enable PDMA transfer target abort interrupt generation.
 * |[1]     |TXIEN     |PDMA Transfer Done Interrupt Enable
 * |        |          |If enabled, the PDMA controller will generate and interrupt to the CPU when the requested PDMA transfer is complete.
 * |        |          |0 = Disable PDMA transfer done interrupt generation.
 * |        |          |1 = Enable PDMA transfer done interrupt generation.
 * |[2]     |WRAPIEN   |Wraparound Interrupt Enable
 * |        |          |If enabled, and channel source or destination address is in wraparound mode, the PDMA controller will generate a WRAP interrupt to the CPU according to the setting of
 * |        |          |PDMA_DSCTn_CTL.WAINTSEL
 * |        |          |This can be interrupts when the transaction has finished and has wrapped around and/or when the transaction is half way in progress
 * |        |          |This allows the efficient implementation of circular buffers for DMA.
 * |        |          |0 = Disable Wraparound PDMA interrupt generation.
 * |        |          |1 = Enable Wraparound interrupt generation.
 * @var PDMA_T::INTSTS
 * Offset: 0x24  PDMA Interrupt Status Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ABTIF     |PDMA Read/Write Target Abort Interrupt Flag
 * |        |          |This flag indicates a Target Abort interrupt condition has occurred
 * |        |          |This condition can happen if attempt is made to read/write from invalid or non-existent memory space
 * |        |          |It occurs when PDMA controller receives a bus error from AHB master
 * |        |          |Upon occurrence PDMA will stop transfer and go to idle state
 * |        |          |To resume, software must reset PDMA channel and initiate transfer again.
 * |        |          |0 = No bus ERROR response received.
 * |        |          |1 = Bus ERROR response received.
 * |        |          |NOTE: This bit is cleared by writing 1 to itself.
 * |[1]     |TXIF      |Block Transfer Done Interrupt Flag
 * |        |          |This bit indicates that PDMA block transfer complete interrupt has been generated
 * |        |          |It is cleared by writing 1 to the bit.
 * |        |          |0 = Transfer ongoing or Idle.
 * |        |          |1 = Transfer Complete.
 * |[11:8]  |WRAPIF    |Wrap Around Transfer Byte Count Interrupt Flag
 * |        |          |These flags are set whenever the conditions for a wraparound interrupt (complete or half complete) are met
 * |        |          |They are cleared by writing one to the bits.
 * |        |          |0001 = Current transfer finished flag (PDMA_CURTXCNT == 0).
 * |        |          |0100 = Current transfer half complete flag (PDMA_CURTXCNT == PDMA_TXCNT /2).
 * |[31]    |INTSTS    |Interrupt Pin Status (Read Only)
 * |        |          |This bit is the Interrupt pin status of PDMA channel.
 * @var PDMA_T::SPAN
 * Offset: 0x34  PDMA Span Increment Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |SPAN      |Span Increment Register
 * |        |          |This is a signed number in range [-128,127] for use in spanned address mode
 * |        |          |If destination or source addressing mode is set as spanned, then this number is added to the address register each transfer
 * |        |          |The size of the transfer is determined by the APB_TW setting
 * |        |          |Note that span increment must be a multiple of the transfer width otherwise a memory addressing HardFault will occur
 * |        |          |Also SPAN may be a negative number.
 * @var PDMA_T::CURSPAN
 * Offset: 0x38  PDMA Current Span Increment Register of Channel n
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |SPAN      |Current Span Increment Register
 * |        |          |This is a signed read only register for use in spanned address mode
 * |        |          |It provides the current address offset from SADDR or DADDR if either is set to span mode.
 */
    __IO uint32_t CTL;                  /*!< [0x0000] PDMA Control Register of Channel n                               */
    __IO uint32_t SADDR;                /*!< [0x0004] PDMA Transfer Source Address Register of Channel n               */
    __IO uint32_t DADDR;                /*!< [0x0008] PDMA Transfer Destination Address Register of Channel n          */
    __IO uint32_t TXCNT;                /*!< [0x000c] PDMA Transfer Byte Count Register of Channel n                   */
    __I  uint32_t INTPNT;               /*!< [0x0010] PDMA Internal Buffer Pointer Register of Channel n               */
    __I  uint32_t CURSADDR;             /*!< [0x0014] PDMA Current Source Address Register of Channel n                */
    __I  uint32_t CURDADDR;             /*!< [0x0018] PDMA Current Destination Address Register of Channel n           */
    __I  uint32_t CURTXCNT;             /*!< [0x001c] PDMA Current Transfer Byte Count Register of Channel n           */
    __IO uint32_t INTEN;                /*!< [0x0020] PDMA Interrupt Enable Control Register of Channel n              */
    __IO uint32_t INTSTS;               /*!< [0x0024] PDMA Interrupt Status Register of Channel n                      */
    __I  uint32_t RESERVE0[3];
    __I  uint32_t SPAN;                 /*!< [0x0034] PDMA Span Increment Register of Channel n                        */
    __IO uint32_t CURSPAN;              /*!< [0x0038] PDMA Current Span Increment Register of Channel n                */

} PDMA_T;

/**
    @addtogroup PDMA_CONST PDMA Bit Field Definition
    Constant Definitions for PDMA Controller
@{ */

#define PDMA_CTL_CHEN_Pos               (0)                                               /*!< PDMA_T::CTL: CHEN Position            */
#define PDMA_CTL_CHEN_Msk               (0x1ul << PDMA_CTL_CHEN_Pos)                     /*!< PDMA_T::CTL: CHEN Mask                */

#define PDMA_CTL_SWRST_Pos              (1)                                               /*!< PDMA_T::CTL: SWRST Position           */
#define PDMA_CTL_SWRST_Msk              (0x1ul << PDMA_CTL_SWRST_Pos)                    /*!< PDMA_T::CTL: SWRST Mask               */

#define PDMA_CTL_MODESEL_Pos            (2)                                               /*!< PDMA_T::CTL: MODESEL Position         */
#define PDMA_CTL_MODESEL_Msk            (0x3ul << PDMA_CTL_MODESEL_Pos)                  /*!< PDMA_T::CTL: MODESEL Mask             */

#define PDMA_CTL_SASEL_Pos              (4)                                               /*!< PDMA_T::CTL: SASEL Position           */
#define PDMA_CTL_SASEL_Msk              (0x3ul << PDMA_CTL_SASEL_Pos)                    /*!< PDMA_T::CTL: SASEL Mask               */

#define PDMA_CTL_DASEL_Pos              (6)                                               /*!< PDMA_T::CTL: DASEL Position           */
#define PDMA_CTL_DASEL_Msk              (0x3ul << PDMA_CTL_DASEL_Pos)                    /*!< PDMA_T::CTL: DASEL Mask               */

#define PDMA_CTL_WAINTSEL_Pos           (12)                                              /*!< PDMA_T::CTL: WAINTSEL Position        */
#define PDMA_CTL_WAINTSEL_Msk           (0xful << PDMA_CTL_WAINTSEL_Pos)                 /*!< PDMA_T::CTL: WAINTSEL Mask            */

#define PDMA_CTL_TXWIDTH_Pos            (19)                                              /*!< PDMA_T::CTL: TXWIDTH Position         */
#define PDMA_CTL_TXWIDTH_Msk            (0x3ul << PDMA_CTL_TXWIDTH_Pos)                  /*!< PDMA_T::CTL: TXWIDTH Mask             */

#define PDMA_CTL_TXEN_Pos               (23)                                              /*!< PDMA_T::CTL: TXEN Position            */
#define PDMA_CTL_TXEN_Msk               (0x1ul << PDMA_CTL_TXEN_Pos)                     /*!< PDMA_T::CTL: TXEN Mask                */

#define PDMA_SADDR_ADDR_Pos             (0)                                               /*!< PDMA_T::SADDR: ADDR Position          */
#define PDMA_SADDR_ADDR_Msk             (0xfffffffful << PDMA_SADDR_ADDR_Pos)            /*!< PDMA_T::SADDR: ADDR Mask              */

#define PDMA_DADDR_ADDR_Pos             (0)                                               /*!< PDMA_T::DADDR: ADDR Position          */
#define PDMA_DADDR_ADDR_Msk             (0xfffffffful << PDMA_DADDR_ADDR_Pos)            /*!< PDMA_T::DADDR: ADDR Mask              */

#define PDMA_TXCNT_CNT_Pos              (0)                                               /*!< PDMA_T::TXCNT: CNT Position           */
#define PDMA_TXCNT_CNT_Msk              (0xfffful << PDMA_TXCNT_CNT_Pos)                 /*!< PDMA_T::TXCNT: CNT Mask               */

#define PDMA_INTPNT_POINTER_Pos         (0)                                               /*!< PDMA_T::INTPNT: POINTER Position      */
#define PDMA_INTPNT_POINTER_Msk         (0xful << PDMA_INTPNT_POINTER_Pos)               /*!< PDMA_T::INTPNT: POINTER Mask          */

#define PDMA_CURSADDR_ADDR_Pos          (0)                                               /*!< PDMA_T::CURSADDR: ADDR Position       */
#define PDMA_CURSADDR_ADDR_Msk          (0xfffffffful << PDMA_CURSADDR_ADDR_Pos)         /*!< PDMA_T::CURSADDR: ADDR Mask           */

#define PDMA_CURDADDR_ADDR_Pos          (0)                                               /*!< PDMA_T::CURDADDR: ADDR Position       */
#define PDMA_CURDADDR_ADDR_Msk          (0xfffffffful << PDMA_CURDADDR_ADDR_Pos)         /*!< PDMA_T::CURDADDR: ADDR Mask           */

#define PDMA_CURTXCNT_CNT_Pos           (0)                                               /*!< PDMA_T::CURTXCNT: CNT Position        */
#define PDMA_CURTXCNT_CNT_Msk           (0xfffful << PDMA_CURTXCNT_CNT_Pos)              /*!< PDMA_T::CURTXCNT: CNT Mask            */

#define PDMA_INTEN_ABTIEN_Pos           (0)                                               /*!< PDMA_T::INTEN: ABTIEN Position        */
#define PDMA_INTEN_ABTIEN_Msk           (0x1ul << PDMA_INTEN_ABTIEN_Pos)                 /*!< PDMA_T::INTEN: ABTIEN Mask            */

#define PDMA_INTEN_TXIEN_Pos            (1)                                               /*!< PDMA_T::INTEN: TXIEN Position         */
#define PDMA_INTEN_TXIEN_Msk            (0x1ul << PDMA_INTEN_TXIEN_Pos)                  /*!< PDMA_T::INTEN: TXIEN Mask             */

#define PDMA_INTEN_WRAPIEN_Pos          (2)                                               /*!< PDMA_T::INTEN: WRAPIEN Position       */
#define PDMA_INTEN_WRAPIEN_Msk          (0x1ul << PDMA_INTEN_WRAPIEN_Pos)                /*!< PDMA_T::INTEN: WRAPIEN Mask           */

#define PDMA_INTSTS_ABTIF_Pos           (0)                                               /*!< PDMA_T::INTSTS: ABTIF Position        */
#define PDMA_INTSTS_ABTIF_Msk           (0x1ul << PDMA_INTSTS_ABTIF_Pos)                 /*!< PDMA_T::INTSTS: ABTIF Mask            */

#define PDMA_INTSTS_TXIF_Pos            (1)                                               /*!< PDMA_T::INTSTS: TXIF Position         */
#define PDMA_INTSTS_TXIF_Msk            (0x1ul << PDMA_INTSTS_TXIF_Pos)                  /*!< PDMA_T::INTSTS: TXIF Mask             */

#define PDMA_INTSTS_WRAPIF_Pos          (8)                                               /*!< PDMA_T::INTSTS: WRAPIF Position       */
#define PDMA_INTSTS_WRAPIF_Msk          (0xful << PDMA_INTSTS_WRAPIF_Pos)                /*!< PDMA_T::INTSTS: WRAPIF Mask           */

#define PDMA_INTSTS_INTSTS_Pos          (31)                                              /*!< PDMA_T::INTSTS: INTSTS Position       */
#define PDMA_INTSTS_INTSTS_Msk          (0x1ul << PDMA_INTSTS_INTSTS_Pos)                /*!< PDMA_T::INTSTS: INTSTS Mask           */

#define PDMA_SPAN_SPAN_Pos              (0)                                               /*!< PDMA_T::SPAN: SPAN Position           */
#define PDMA_SPAN_SPAN_Msk              (0xfful << PDMA_SPAN_SPAN_Pos)                   /*!< PDMA_T::SPAN: SPAN Mask               */

#define PDMA_CURSPAN_SPAN_Pos           (0)                                               /*!< PDMA_T::CURSPAN: SPAN Position        */
#define PDMA_CURSPAN_SPAN_Msk           (0xfful << PDMA_CURSPAN_SPAN_Pos)                /*!< PDMA_T::CURSPAN: SPAN Mask            */

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
 * @var PWM_T::CLKPSC
 * Offset: 0x00  PWM Prescaler Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |CLKPSC01  |Clock Pre-scaler Pair of PWM0CH0 and PWM0CH1
 * |        |          |Clock input is divided by (CLKPSC01 + 1)
 * |        |          |If CLKPSC01 = 0, then the pre-scaler output clock will be stopped.
 * |        |          |This implies PWM counter 0 and 1 will also be stopped.
 * |[15:8]  |CLKPSC23  |Clock Pre-scaler for Pair of PWM0CH2 and PWM0CH3
 * |        |          |Clock input is divided by (CLKPSC23 + 1)
 * |        |          |If CLKPSC23 = 0, then the pre-scaler output clock will be stopped.
 * |        |          |This implies PWM counter 2 and 3 will also be stopped.
 * |[23:16] |DTCNT01   |Dead Zone Interval Register for Pair of PWM0CH0 and PWM0CH1
 * |        |          |These 8 bits determine dead zone length.
 * |        |          |The unit time of dead zone length is that from clock selector 0.
 * |[31:24] |DTCNT23   |Dead Zone Interval Register for Pair of PWM0CH2 and PWM0CH3
 * |        |          |These 8 bits determine dead zone length.
 * |        |          |The unit time of dead zone length is that from clock selector 0.
 * @var PWM_T::CLKDIV
 * Offset: 0x04  PWM Clock Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |CLKDIV0   |Timer 0 Clock Source Selection
 * |        |          |Value : Input clock divided by
 * |        |          |0 : 2
 * |        |          |1 : 4
 * |        |          |2 : 8
 * |        |          |3 : 16
 * |        |          |4 : 1
 * |[6:4]   |CLKDIV1   |Timer 1 Clock Source Selection
 * |        |          |(Table is as CLKDIV0)
 * |[10:8]  |CLKDIV2   |Timer 2 Clock Source Selection
 * |        |          |(Table is as CLKDIV0)
 * |[14:12] |CLKDIV3   |Timer 3 Clock Source Selection
 * |        |          |(Table is as CLKDIV0)
 * @var PWM_T::CTL
 * Offset: 0x08  PWM Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CNTEN0    |PWM-timer 0 Enable/Disable Start Run
 * |        |          |0 = Stop PWM-Timer 0 Running.
 * |        |          |1 = Enable PWM-Timer 0 Start/Run.
 * |[2]     |PINV0     |PWM-timer 0 Output Inverter ON/OFF
 * |        |          |0 = Inverter OFF.
 * |        |          |1 = Inverter ON.
 * |[3]     |CNTMODE0  |PWM-timer 0 Auto-reload/One-shot Mode
 * |        |          |0 = One-Shot Mode.
 * |        |          |1 = Auto-reload Mode.
 * |        |          |Note: A rising transition of this bit will cause PWM_PERIOD0 and PWM_CMPDAT0 to be cleared.
 * |[4]     |DTEN01    |Dead-zone 01 Generator Enable/Disable Pair of PWM0CH0 and PWM0CH1
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |        |          |Note: When Dead-Zone Generator is enabled, the pair of PWM0CH0 and PWM0CH1 become a complementary pair.
 * |[5]     |DTEN23    |Dead-zone 23 Generator Enable/Disable Pair of PWM0CH2 and PWM0CH3
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |        |          |Note: When Dead-Zone Generator is enabled, the pair of PWM0CH2 and PWM0CH3 become a complementary pair.
 * |[8]     |CNTEN1    |PWM-timer 1 Enable/Disable Start Run
 * |        |          |0 = Stop PWM-Timer 1.
 * |        |          |1 = Enable PWM-Timer 1 Start/Run.
 * |[10]    |PINV1     |PWM-timer 1 Output Inverter ON/OFF
 * |        |          |0 = Inverter OFF.
 * |        |          |1 = Inverter ON.
 * |[11]    |CNTMODE1  |PWM-timer 1 Auto-reload/One-shot Mode
 * |        |          |0 = One-Shot Mode.
 * |        |          |1 = Auto-load Mode.
 * |        |          |Note: A rising transition of this bit will cause PWM_PERIOD1 and PWM_CMPDAT1 to be cleared.
 * |[16]    |CNTEN2    |PWM-timer 2 Enable/Disable Start Run
 * |        |          |0 = Stop PWM-Timer 2.
 * |        |          |1 = Enable PWM-Timer 2 Start/Run.
 * |[18]    |PINV2     |PWM-timer 2 Output Inverter ON/OFF
 * |        |          |0 = Inverter OFF.
 * |        |          |1 = Inverter ON.
 * |[19]    |CNTMODE2  |PWM-timer 2 Auto-reload/One-shot Mode
 * |        |          |0 = One-Shot Mode.
 * |        |          |1 = Auto-load Mode.
 * |        |          |Note: A rising transition of this bit will cause PWM_PERIOD2 and PWM_CMPDAT2 to be cleared.
 * |[24]    |CNTEN3    |PWM-timer 3 Enable/Disable Start Run
 * |        |          |0 = Stop PWM-Timer 3.
 * |        |          |1 = Enable PWM-Timer 3 Start/Run.
 * |[26]    |PINV3     |PWM-timer 3 Output Inverter ON/OFF
 * |        |          |0 = Inverter OFF.
 * |        |          |1 = Inverter ON.
 * |[27]    |CNTMODE3  |PWM-timer 3 Auto-reload/One-shot Mode
 * |        |          |0 = One-Shot Mode.
 * |        |          |1 = Auto-load Mode.
 * |        |          |Note: A rising transition of this bit will cause PWM_PERIOD3 and PWM_CMPDAT3 to be cleared.
 * @var PWM_T::PERIOD0
 * Offset: 0x0C  PWM Counter Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PERIOD    |PWM Counter/Timer Reload Value
 * |        |          |PERIOD determines the PWM period.
 * |        |          |PWM frequency = PWM0CHx_CLK/(prescale+1)*(clock divider)/(PERIOD+1);.
 * |        |          |Duty ratio = (CMP+1)/(PERIOD+1).
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note:
 * |        |          |Any write to PERIOD will take effect in next PWM cycle.
 * @var PWM_T::CMPDAT0
 * Offset: 0x10  PWM Comparator Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMP       |PWM Comparator Register
 * |        |          |CMP determines the PWM duty cycle.
 * |        |          |PWM frequency = PWM0CHx_CLK/(prescale+1)*(clock divider)/(PERIOD+1);.
 * |        |          |Duty Cycle = (CMP+1)/(PERIOD+1).
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CMP will take effect in next PWM cycle.
 * @var PWM_T::CNT0
 * Offset: 0x14  PWM Data Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNT       |PWM Data Register
 * |        |          |Reports the current value of the 16-bit down counter.
 * @var PWM_T::PERIOD1
 * Offset: 0x18  PWM Counter Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PERIOD    |PWM Counter/Timer Reload Value
 * |        |          |PERIOD determines the PWM period.
 * |        |          |PWM frequency = PWM0CHx_CLK/(prescale+1)*(clock divider)/(PERIOD+1);.
 * |        |          |Duty ratio = (CMP+1)/(PERIOD+1).
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note:
 * |        |          |Any write to PERIOD will take effect in next PWM cycle.
 * @var PWM_T::CMPDAT1
 * Offset: 0x1C  PWM Comparator Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMP       |PWM Comparator Register
 * |        |          |CMP determines the PWM duty cycle.
 * |        |          |PWM frequency = PWM0CHx_CLK/(prescale+1)*(clock divider)/(PERIOD+1);.
 * |        |          |Duty Cycle = (CMP+1)/(PERIOD+1).
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CMP will take effect in next PWM cycle.
 * @var PWM_T::CNT1
 * Offset: 0x20  PWM Data Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNT       |PWM Data Register
 * |        |          |Reports the current value of the 16-bit down counter.
 * @var PWM_T::PERIOD2
 * Offset: 0x24  PWM Counter Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PERIOD    |PWM Counter/Timer Reload Value
 * |        |          |PERIOD determines the PWM period.
 * |        |          |PWM frequency = PWM0CHx_CLK/(prescale+1)*(clock divider)/(PERIOD+1);.
 * |        |          |Duty ratio = (CMP+1)/(PERIOD+1).
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note:
 * |        |          |Any write to PERIOD will take effect in next PWM cycle.
 * @var PWM_T::CMPDAT2
 * Offset: 0x28  PWM Comparator Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMP       |PWM Comparator Register
 * |        |          |CMP determines the PWM duty cycle.
 * |        |          |PWM frequency = PWM0CHx_CLK/(prescale+1)*(clock divider)/(PERIOD+1);.
 * |        |          |Duty Cycle = (CMP+1)/(PERIOD+1).
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CMP will take effect in next PWM cycle.
 * @var PWM_T::CNT2
 * Offset: 0x2C  PWM Data Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNT       |PWM Data Register
 * |        |          |Reports the current value of the 16-bit down counter.
 * @var PWM_T::PERIOD3
 * Offset: 0x30  PWM Counter Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |PERIOD    |PWM Counter/Timer Reload Value
 * |        |          |PERIOD determines the PWM period.
 * |        |          |PWM frequency = PWM0CHx_CLK/(prescale+1)*(clock divider)/(PERIOD+1);.
 * |        |          |Duty ratio = (CMP+1)/(PERIOD+1).
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note:
 * |        |          |Any write to PERIOD will take effect in next PWM cycle.
 * @var PWM_T::CMPDAT3
 * Offset: 0x34  PWM Comparator Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CMP       |PWM Comparator Register
 * |        |          |CMP determines the PWM duty cycle.
 * |        |          |PWM frequency = PWM0CHx_CLK/(prescale+1)*(clock divider)/(PERIOD+1);.
 * |        |          |Duty Cycle = (CMP+1)/(PERIOD+1).
 * |        |          |CMP > = PERIOD: PWM output is always high.
 * |        |          |CMP < PERIOD: PWM low width = (PERIOD-CMP) unit; PWM high width = (CMP+1) unit.
 * |        |          |CMP = 0: PWM low width = (PERIOD) unit; PWM high width = 1 unit.
 * |        |          |(Unit = one PWM clock cycle).
 * |        |          |Note: Any write to CMP will take effect in next PWM cycle.
 * @var PWM_T::CNT3
 * Offset: 0x38  PWM Data Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CNT       |PWM Data Register
 * |        |          |Reports the current value of the 16-bit down counter.
 * @var PWM_T::INTEN
 * Offset: 0x40  PWM Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PIEN0     |PWM Timer 0 Interrupt Enable
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |[1]     |PIEN1     |PWM Timer 1 Interrupt Enable
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |[2]     |PIEN2     |PWM Timer 2 Interrupt Enable
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * |[3]     |PIEN3     |PWM Timer 3 Interrupt Enable
 * |        |          |0 = Disable.
 * |        |          |1 = Enable.
 * @var PWM_T::INTSTS
 * Offset: 0x44  PWM Interrupt Flag Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PIF0      |PWM Timer 0 Interrupt Flag
 * |        |          |Flag is set by hardware when PWM0CH0 down counter reaches zero, software can clear this bit by writing u20181u2019 to it.
 * |[1]     |PIF1      |PWM Timer 1 Interrupt Flag
 * |        |          |Flag is set by hardware when PWM0CH1 down counter reaches zero, software can clear this bit by writing u20181u2019 to it.
 * |[2]     |PIF2      |PWM Timer 2 Interrupt Flag
 * |        |          |Flag is set by hardware when PWM0CH2 down counter reaches zero, software can clear this bit by writing u20181u2019 to it.
 * |[3]     |PIF3      |PWM Timer 3 Interrupt Flag
 * |        |          |Flag is set by hardware when PWM0CH3 down counter reaches zero, software can clear this bit by writing u20181u2019 to it.
 * @var PWM_T::CAPCTL01
 * Offset: 0x50  Capture Control Register for Pair of PWM0CH0 and PWM0CH1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CAPINV0   |Channel 0 Inverter ON/OFF
 * |        |          |0 = Inverter OFF.
 * |        |          |1 = Inverter ON. Reverse the input signal from GPIO before Capture timer
 * |[1]     |CRLIEN0   |Channel 0 Rising Latch Interrupt Enable ON/OFF
 * |        |          |0 = Disable rising latch interrupt.
 * |        |          |1 = Enable rising latch interrupt.
 * |        |          |When enabled, capture block generates an interrupt on rising edge of input.
 * |[2]     |CFLIEN0   |Channel 0 Falling Latch Interrupt Enable ON/OFF
 * |        |          |0 = Disable falling latch interrupt.
 * |        |          |1 = Enable falling latch interrupt.
 * |        |          |When enabled, capture block generates an interrupt on falling edge of input.
 * |[3]     |CAPEN0    |Capture Channel 0 Transition Enable/Disable
 * |        |          |0 = Disable capture function on channel 0.
 * |        |          |1 = Enable capture function on channel 0.
 * |        |          |When enabled, Capture function latches the PMW-counter to RCAPDAT (Rising latch) and FCAPDAT (Falling latch) registers on input edge transition.
 * |        |          |When disabled, Capture function is inactive as is interrupt.
 * |[4]     |CAPIF0    |Capture0 Interrupt Indication Flag
 * |        |          |If channel 0 rising latch interrupt is enabled (CRLIEN0 = 1), a rising transition at input channel 0 will result in CAPIF0 to high; Similarly, a falling transition will cause CAPIF0 to be set high if channel 0 falling latch interrupt is enabled (CFLIEN0 = 1)
 * |        |          |This flag is cleared by software writing a u20181u2019 to it.
 * |[6]     |CRLIF0    |PWM_RCAPDAT0 Latched Indicator Bit
 * |        |          |When input channel 0 has a rising transition, PWM_RCAPDAT0 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it.
 * |[7]     |CFLIF0    |PWM_FCAPDAT0 Latched Indicator Bit
 * |        |          |When input channel 0 has a falling transition, PWM_FCAPDAT0 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it
 * |[16]    |CAPINV1   |Channel 1 Inverter ON/OFF
 * |        |          |0 = Inverter OFF.
 * |        |          |1 = Inverter ON. Reverse the input signal from GPIO before Capture timer
 * |[17]    |CRLIEN1   |Channel 1 Rising Latch Interrupt Enable
 * |        |          |0 = Disable rising edge latch interrupt.
 * |        |          |1 = Enable rising edge latch interrupt.
 * |        |          |When enabled, capture block generates an interrupt on rising edge of input.
 * |[18]    |CFLIEN1   |Channel 1 Falling Latch Interrupt Enable
 * |        |          |0 = Disable falling edge latch interrupt.
 * |        |          |1 = Enable falling edge latch interrupt.
 * |        |          |When enabled, capture block generates an interrupt on falling edge of input.
 * |[19]    |CAPEN1    |Capture Channel 1 Transition Enable/Disable
 * |        |          |0 = Disable capture function on channel 1.
 * |        |          |1 = Enable capture function on channel 1.
 * |        |          |When enabled, Capture function latches the PMW-counter to RCAPDAT (Rising latch) and FCAPDAT (Falling latch) registers on input edge transition.
 * |        |          |When disabled, Capture function is inactive as is interrupt.
 * |[20]    |CAPIF1    |Capture1 Interrupt Indication Flag
 * |        |          |If channel 1 rising latch interrupt is enabled (CRLIEN1 = 1), a rising transition at input channel 1 will result in CAPIF1 to high; Similarly, a falling transition will cause CAPIF1 to be set high if channel 1 falling latch interrupt is enabled (CFLIEN1 = 1)
 * |        |          |This flag is cleared by software writing a u20181u2019 to it.
 * |[22]    |CRLIF1    |PWM_RCAPDAT1 Latched Indicator Bit
 * |        |          |When input channel 1 has a rising transition, PWM_RCAPDAT1 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it.
 * |[23]    |CFLIF1    |PWM_FCAPDAT1 Latched Indicator Bit
 * |        |          |When input channel 1 has a falling transition, PWM_FCAPDAT1 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it
 * @var PWM_T::CAPCTL23
 * Offset: 0x54  Capture Control Register for Pair of PWM0CH2 and PWM0CH3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CAPINV2   |Channel 2 Inverter ON/OFF
 * |        |          |0 = Inverter OFF.
 * |        |          |1 = Inverter ON. Reverse the input signal from GPIO before Capture timer
 * |[1]     |CRLIEN2   |Channel 2 Rising Latch Interrupt Enable ON/OFF
 * |        |          |0 = Disable rising latch interrupt.
 * |        |          |1 = Enable rising latch interrupt.
 * |        |          |When enabled, capture block generates an interrupt on rising edge of input.
 * |[2]     |CFLIEN2   |Channel 2 Falling Latch Interrupt Enable ON/OFF
 * |        |          |0 = Disable falling latch interrupt.
 * |        |          |1 = Enable falling latch interrupt.
 * |        |          |When enabled, capture block generates an interrupt on falling edge of input.
 * |[3]     |CAPEN2    |Capture Channel 2 Transition Enable/Disable
 * |        |          |0 = Disable capture function on channel 0.
 * |        |          |1 = Enable capture function on channel 0.
 * |        |          |When enabled, Capture function latches the PMW-counter to RCAPDAT (Rising latch) and FCAPDAT (Falling latch) registers on input edge transition.
 * |        |          |When disabled, Capture function is inactive as is interrupt.
 * |[4]     |CAPIF2    |Capture2 Interrupt Indication Flag
 * |        |          |If channel 2 rising latch interrupt is enabled (CRLIEN2 = 1), a rising transition at input channel 2 will result in CAPIF2 to high; Similarly, a falling transition will cause CAPIF2 to be set high if channel 2 falling latch interrupt is enabled (CFLIEN2 = 1)
 * |        |          |This flag is cleared by software writing a u20181u2019 to it.
 * |[6]     |CRLIF2    |PWM_RCAPDAT2 Latched Indicator Bit
 * |        |          |When input channel 2 has a rising transition, PWM_RCAPDAT2 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it.
 * |[7]     |CFLIF2    |PWM_FCAPDAT2 Latched Indicator Bit
 * |        |          |When input channel 2 has a falling transition, PWM_FCAPDAT2 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it
 * |[16]    |CAPINV3   |Channel 3 Inverter ON/OFF
 * |        |          |0 = Inverter OFF.
 * |        |          |1 = Inverter ON. Reverse the input signal from GPIO before Capture timer
 * |[17]    |CRLIEN3   |Channel 3 Rising Latch Interrupt Enable
 * |        |          |0 = Disable rising edge latch interrupt.
 * |        |          |1 = Enable rising edge latch interrupt.
 * |        |          |When enabled, capture block generates an interrupt on rising edge of input.
 * |[18]    |CFLIEN3   |Channel 3 Falling Latch Interrupt Enable
 * |        |          |0 = Disable falling edge latch interrupt.
 * |        |          |1 = Enable falling edge latch interrupt.
 * |        |          |When enabled, capture block generates an interrupt on falling edge of input.
 * |[19]    |CAPEN3    |Capture Channel 3 Transition Enable/Disable
 * |        |          |0 = Disable capture function on channel 1.
 * |        |          |1 = Enable capture function on channel 1.
 * |        |          |When enabled, Capture function latches the PMW-counter to RCAPDAT (Rising latch) and FCAPDAT (Falling latch) registers on input edge transition.
 * |        |          |When disabled, Capture function is inactive as is interrupt.
 * |[20]    |CAPIF3    |Capture3 Interrupt Indication Flag
 * |        |          |If channel 3 rising latch interrupt is enabled (CRLIEN3 = 1), a rising transition at input channel 3 will result in CAPIF3 to high; Similarly, a falling transition will cause CAPIF3 to be set high if channel 3 falling latch interrupt is enabled (CFLIEN3 = 1)
 * |        |          |This flag is cleared by software writing a u20181u2019 to it.
 * |[22]    |CRLIF3    |PWM_RCAPDAT3 Latched Indicator Bit
 * |        |          |When input channel 3 has a rising transition, PWM_RCAPDAT3 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it.
 * |[23]    |CFLIF3    |PWM_FCAPDAT3 Latched Indicator Bit
 * |        |          |When input channel 3 has a falling transition, PWM_FCAPDAT3 was latched with the value of PWM down-counter and this bit is set by hardware, software can clear this bit by writing a zero to it
 * @var PWM_T::RCAPDAT0
 * Offset: 0x58  Capture Rising Latch Register (Channel 0)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |RCAPDAT   |Capture Rising Latch Register
 * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a rising edge of the input signal.
 * @var PWM_T::FCAPDAT0
 * Offset: 0x5C  Capture Falling Latch Register (Channel 0)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |FCAPDAT   |Capture Falling Latch Register
 * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a falling edge of the input signal.
 * @var PWM_T::RCAPDAT1
 * Offset: 0x60  Capture Rising Latch Register (Channel 1)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |RCAPDAT   |Capture Rising Latch Register
 * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a rising edge of the input signal.
 * @var PWM_T::FCAPDAT1
 * Offset: 0x64  Capture Falling Latch Register (Channel 1)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |FCAPDAT   |Capture Falling Latch Register
 * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a falling edge of the input signal.
 * @var PWM_T::RCAPDAT2
 * Offset: 0x68  Capture Rising Latch Register (Channel 2)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |RCAPDAT   |Capture Rising Latch Register
 * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a rising edge of the input signal.
 * @var PWM_T::FCAPDAT2
 * Offset: 0x6C  Capture Falling Latch Register (Channel 2)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |FCAPDAT   |Capture Falling Latch Register
 * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a falling edge of the input signal.
 * @var PWM_T::RCAPDAT3
 * Offset: 0x70  Capture Rising Latch Register (Channel 3)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |RCAPDAT   |Capture Rising Latch Register
 * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a rising edge of the input signal.
 * @var PWM_T::FCAPDAT3
 * Offset: 0x74  Capture Falling Latch Register (Channel 3)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |FCAPDAT   |Capture Falling Latch Register
 * |        |          |In Capture mode, this register is latched with the value of the PWM counter on a falling edge of the input signal.
 * @var PWM_T::CAPINEN
 * Offset: 0x78  Capture Input Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |CAPINEN   |Capture Input Enable Register
 * |        |          |0 : OFF (GPA[13:12], GPB[15:14] pin input disconnected from Capture block)
 * |        |          |1 : ON (GPA[13:12] , GPB[15:14] pin, if in PWM alternative function, will be configured as an input and fed to capture function)
 * |        |          |CAPINEN[3:0]
 * |        |          |Bit [3][2][1][0]
 * |        |          |Bit xxx1 : Capture channel 0 is from GPA [12]
 * |        |          |Bit xx1x : Capture channel 1 is from GPA [13]
 * |        |          |Bit x1xx : Capture channel 2 is from GPB [14]
 * |        |          |Bit 1xxx : Capture channel 3 is from GPB [15] 
 * @var PWM_T::POEN
 * Offset: 0x7C  PWM0 Output Enable Register for CH0~CH3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |POEN0     |PWM0CH0 Output Enable Register
 * |        |          |0 = Disable PWM0CH0 output to pin.
 * |        |          |1 = Enable PWM0CH 0 output to pin.
 * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP)
 * |[1]     |POEN1     |PWM0CH1 Output Enable Register
 * |        |          |0 = Disable PWM0CH1 output to pin.
 * |        |          |1 = Enable PWM0CH1 output to pin.
 * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP)
 * |[2]     |POEN2     |PWM0CH2 Output Enable Register
 * |        |          |0 = Disable PWM0CH2 output to pin.
 * |        |          |1 = Enable PWM0CH2 output to pin.
 * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP)
 * |[3]     |POEN3     |PWM0CH3 Output Enable Register
 * |        |          |0 = Disable PWM0CH3 output to pin.
 * |        |          |1 = Enable PWM0CH3 output to pin.
 * |        |          |Note: The corresponding GPIO pin also must be switched to PWM function (refer to SYS_GPA_MFP)
 */
    __IO uint32_t CLKPSC;                /*!< [0x0000] PWM Prescaler Register                                           */
    __IO uint32_t CLKDIV;                /*!< [0x0004] PWM Clock Select Register                                        */
    __IO uint32_t CTL;                   /*!< [0x0008] PWM Control Register                                             */
    __IO uint32_t PERIOD0;               /*!< [0x000c] PWM Counter Register 0                                           */
    __IO uint32_t CMPDAT0;               /*!< [0x0010] PWM Comparator Register 0                                        */
    __I  uint32_t CNT0;                  /*!< [0x0014] PWM Data Register 0                                              */
    __IO uint32_t PERIOD1;               /*!< [0x0018] PWM Counter Register 1                                           */
    __IO uint32_t CMPDAT1;               /*!< [0x001c] PWM Comparator Register 1                                        */
    __I  uint32_t CNT1;                  /*!< [0x0020] PWM Data Register 1                                              */
    __IO uint32_t PERIOD2;               /*!< [0x0024] PWM Counter Register 2                                           */
    __IO uint32_t CMPDAT2;               /*!< [0x0028] PWM Comparator Register 2                                        */
    __I  uint32_t CNT2;                  /*!< [0x002c] PWM Data Register 2                                              */
    __IO uint32_t PERIOD3;               /*!< [0x0030] PWM Counter Register 3                                           */
    __IO uint32_t CMPDAT3;               /*!< [0x0034] PWM Comparator Register 3                                        */
    __I  uint32_t CNT3;                  /*!< [0x0038] PWM Data Register 3                                              */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t INTEN;                 /*!< [0x0040] PWM Interrupt Enable Register                                    */
    __IO uint32_t INTSTS;                /*!< [0x0044] PWM Interrupt Flag Register                                      */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t CAPCTL01;              /*!< [0x0050] Capture Control Register for Pair of PWM0CH0 and PWM0CH1         */
    __IO uint32_t CAPCTL23;              /*!< [0x0054] Capture Control Register for Pair of PWM0CH2 and PWM0CH3         */
    __I  uint32_t RCAPDAT0;              /*!< [0x0058] Capture Rising Latch Register (Channel 0)                        */
    __I  uint32_t FCAPDAT0;              /*!< [0x005c] Capture Falling Latch Register (Channel 0)                       */
    __I  uint32_t RCAPDAT1;              /*!< [0x0060] Capture Rising Latch Register (Channel 1)                        */
    __I  uint32_t FCAPDAT1;              /*!< [0x0064] Capture Falling Latch Register (Channel 1)                       */
    __I  uint32_t RCAPDAT2;              /*!< [0x0068] Capture Rising Latch Register (Channel 2)                        */
    __I  uint32_t FCAPDAT2;              /*!< [0x006c] Capture Falling Latch Register (Channel 2)                       */
    __I  uint32_t RCAPDAT3;              /*!< [0x0070] Capture Rising Latch Register (Channel 3)                        */
    __I  uint32_t FCAPDAT3;              /*!< [0x0074] Capture Falling Latch Register (Channel 3)                       */
    __IO uint32_t CAPINEN;               /*!< [0x0078] Capture Input Enable Register                                    */
    __IO uint32_t POEN;                  /*!< [0x007c] PWM0 Output Enable Register for CH0~CH3                          */

} PWM_T;

/**
    @addtogroup PWM_CONST PWM Bit Field Definition
    Constant Definitions for PWM Controller
@{ */

#define PWM_CLKPSC_CLKPSC01_Pos          (0)                                               /*!< PWM_T::CLKPSC: CLKPSC01 Position       */
#define PWM_CLKPSC_CLKPSC01_Msk          (0xfful << PWM_CLKPSC_CLKPSC01_Pos)               /*!< PWM_T::CLKPSC: CLKPSC01 Mask           */

#define PWM_CLKPSC_CLKPSC23_Pos          (8)                                               /*!< PWM_T::CLKPSC: CLKPSC23 Position       */
#define PWM_CLKPSC_CLKPSC23_Msk          (0xfful << PWM_CLKPSC_CLKPSC23_Pos)               /*!< PWM_T::CLKPSC: CLKPSC23 Mask           */

#define PWM_CLKPSC_DTCNT01_Pos           (16)                                              /*!< PWM_T::CLKPSC: DTCNT01 Position        */
#define PWM_CLKPSC_DTCNT01_Msk           (0xfful << PWM_CLKPSC_DTCNT01_Pos)                /*!< PWM_T::CLKPSC: DTCNT01 Mask            */

#define PWM_CLKPSC_DTCNT23_Pos           (24)                                              /*!< PWM_T::CLKPSC: DTCNT23 Position        */
#define PWM_CLKPSC_DTCNT23_Msk           (0xfful << PWM_CLKPSC_DTCNT23_Pos)                /*!< PWM_T::CLKPSC: DTCNT23 Mask            */

#define PWM_CLKDIV_CLKDIV0_Pos           (0)                                               /*!< PWM_T::CLKDIV: CLKDIV0 Position        */
#define PWM_CLKDIV_CLKDIV0_Msk           (0x7ul << PWM_CLKDIV_CLKDIV0_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV0 Mask            */

#define PWM_CLKDIV_CLKDIV1_Pos           (4)                                               /*!< PWM_T::CLKDIV: CLKDIV1 Position        */
#define PWM_CLKDIV_CLKDIV1_Msk           (0x7ul << PWM_CLKDIV_CLKDIV1_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV1 Mask            */

#define PWM_CLKDIV_CLKDIV2_Pos           (8)                                               /*!< PWM_T::CLKDIV: CLKDIV2 Position        */
#define PWM_CLKDIV_CLKDIV2_Msk           (0x7ul << PWM_CLKDIV_CLKDIV2_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV2 Mask            */

#define PWM_CLKDIV_CLKDIV3_Pos           (12)                                              /*!< PWM_T::CLKDIV: CLKDIV3 Position        */
#define PWM_CLKDIV_CLKDIV3_Msk           (0x7ul << PWM_CLKDIV_CLKDIV3_Pos)                 /*!< PWM_T::CLKDIV: CLKDIV3 Mask            */

#define PWM_CTL_CNTEN0_Pos               (0)                                               /*!< PWM_T::CTL: CNTEN0 Position            */
#define PWM_CTL_CNTEN0_Msk               (0x1ul << PWM_CTL_CNTEN0_Pos)                     /*!< PWM_T::CTL: CNTEN0 Mask                */

#define PWM_CTL_PINV0_Pos                (2)                                               /*!< PWM_T::CTL: PINV0 Position             */
#define PWM_CTL_PINV0_Msk                (0x1ul << PWM_CTL_PINV0_Pos)                      /*!< PWM_T::CTL: PINV0 Mask                 */

#define PWM_CTL_CNTMODE0_Pos             (3)                                               /*!< PWM_T::CTL: CNTMODE0 Position          */
#define PWM_CTL_CNTMODE0_Msk             (0x1ul << PWM_CTL_CNTMODE0_Pos)                   /*!< PWM_T::CTL: CNTMODE0 Mask              */

#define PWM_CTL_DTEN01_Pos               (4)                                               /*!< PWM_T::CTL: DTEN01 Position            */
#define PWM_CTL_DTEN01_Msk               (0x1ul << PWM_CTL_DTEN01_Pos)                     /*!< PWM_T::CTL: DTEN01 Mask                */

#define PWM_CTL_DTEN23_Pos               (5)                                               /*!< PWM_T::CTL: DTEN23 Position            */
#define PWM_CTL_DTEN23_Msk               (0x1ul << PWM_CTL_DTEN23_Pos)                     /*!< PWM_T::CTL: DTEN23 Mask                */

#define PWM_CTL_CNTEN1_Pos               (8)                                               /*!< PWM_T::CTL: CNTEN1 Position            */
#define PWM_CTL_CNTEN1_Msk               (0x1ul << PWM_CTL_CNTEN1_Pos)                     /*!< PWM_T::CTL: CNTEN1 Mask                */

#define PWM_CTL_PINV1_Pos                (10)                                              /*!< PWM_T::CTL: PINV1 Position             */
#define PWM_CTL_PINV1_Msk                (0x1ul << PWM_CTL_PINV1_Pos)                      /*!< PWM_T::CTL: PINV1 Mask                 */

#define PWM_CTL_CNTMODE1_Pos             (11)                                              /*!< PWM_T::CTL: CNTMODE1 Position          */
#define PWM_CTL_CNTMODE1_Msk             (0x1ul << PWM_CTL_CNTMODE1_Pos)                   /*!< PWM_T::CTL: CNTMODE1 Mask              */

#define PWM_CTL_CNTEN2_Pos               (16)                                              /*!< PWM_T::CTL: CNTEN2 Position            */
#define PWM_CTL_CNTEN2_Msk               (0x1ul << PWM_CTL_CNTEN2_Pos)                     /*!< PWM_T::CTL: CNTEN2 Mask                */

#define PWM_CTL_PINV2_Pos                (18)                                              /*!< PWM_T::CTL: PINV2 Position             */
#define PWM_CTL_PINV2_Msk                (0x1ul << PWM_CTL_PINV2_Pos)                      /*!< PWM_T::CTL: PINV2 Mask                 */

#define PWM_CTL_CNTMODE2_Pos             (19)                                              /*!< PWM_T::CTL: CNTMODE2 Position          */
#define PWM_CTL_CNTMODE2_Msk             (0x1ul << PWM_CTL_CNTMODE2_Pos)                   /*!< PWM_T::CTL: CNTMODE2 Mask              */

#define PWM_CTL_CNTEN3_Pos               (24)                                              /*!< PWM_T::CTL: CNTEN3 Position            */
#define PWM_CTL_CNTEN3_Msk               (0x1ul << PWM_CTL_CNTEN3_Pos)                     /*!< PWM_T::CTL: CNTEN3 Mask                */

#define PWM_CTL_PINV3_Pos                (26)                                              /*!< PWM_T::CTL: PINV3 Position             */
#define PWM_CTL_PINV3_Msk                (0x1ul << PWM_CTL_PINV3_Pos)                      /*!< PWM_T::CTL: PINV3 Mask                 */

#define PWM_CTL_CNTMODE3_Pos             (27)                                              /*!< PWM_T::CTL: CNTMODE3 Position          */
#define PWM_CTL_CNTMODE3_Msk             (0x1ul << PWM_CTL_CNTMODE3_Pos)                   /*!< PWM_T::CTL: CNTMODE3 Mask              */

#define PWM_PERIOD0_PERIOD_Pos           (0)                                               /*!< PWM_T::PERIOD0: PERIOD Position        */
#define PWM_PERIOD0_PERIOD_Msk           (0xfffful << PWM_PERIOD0_PERIOD_Pos)              /*!< PWM_T::PERIOD0: PERIOD Mask            */

#define PWM_CMPDAT0_CMP_Pos              (0)                                               /*!< PWM_T::CMPDAT0: CMP Position           */
#define PWM_CMPDAT0_CMP_Msk              (0xfffful << PWM_CMPDAT0_CMP_Pos)                 /*!< PWM_T::CMPDAT0: CMP Mask               */

#define PWM_CNT0_CNT_Pos                 (0)                                               /*!< PWM_T::CNT0: CNT Position              */
#define PWM_CNT0_CNT_Msk                 (0xfffful << PWM_CNT0_CNT_Pos)                    /*!< PWM_T::CNT0: CNT Mask                  */

#define PWM_PERIOD1_PERIOD_Pos           (0)                                               /*!< PWM_T::PERIOD1: PERIOD Position        */
#define PWM_PERIOD1_PERIOD_Msk           (0xfffful << PWM_PERIOD1_PERIOD_Pos)              /*!< PWM_T::PERIOD1: PERIOD Mask            */

#define PWM_CMPDAT1_CMP_Pos              (0)                                               /*!< PWM_T::CMPDAT1: CMP Position           */
#define PWM_CMPDAT1_CMP_Msk              (0xfffful << PWM_CMPDAT1_CMP_Pos)                 /*!< PWM_T::CMPDAT1: CMP Mask               */

#define PWM_CNT1_CNT_Pos                 (0)                                               /*!< PWM_T::CNT1: CNT Position              */
#define PWM_CNT1_CNT_Msk                 (0xfffful << PWM_CNT1_CNT_Pos)                    /*!< PWM_T::CNT1: CNT Mask                  */

#define PWM_PERIOD2_PERIOD_Pos           (0)                                               /*!< PWM_T::PERIOD2: PERIOD Position        */
#define PWM_PERIOD2_PERIOD_Msk           (0xfffful << PWM_PERIOD2_PERIOD_Pos)              /*!< PWM_T::PERIOD2: PERIOD Mask            */

#define PWM_CMPDAT2_CMP_Pos              (0)                                               /*!< PWM_T::CMPDAT2: CMP Position           */
#define PWM_CMPDAT2_CMP_Msk              (0xfffful << PWM_CMPDAT2_CMP_Pos)                 /*!< PWM_T::CMPDAT2: CMP Mask               */

#define PWM_CNT2_CNT_Pos                 (0)                                               /*!< PWM_T::CNT2: CNT Position              */
#define PWM_CNT2_CNT_Msk                 (0xfffful << PWM_CNT2_CNT_Pos)                    /*!< PWM_T::CNT2: CNT Mask                  */

#define PWM_PERIOD3_PERIOD_Pos           (0)                                               /*!< PWM_T::PERIOD3: PERIOD Position        */
#define PWM_PERIOD3_PERIOD_Msk           (0xfffful << PWM_PERIOD3_PERIOD_Pos)              /*!< PWM_T::PERIOD3: PERIOD Mask            */

#define PWM_CMPDAT3_CMP_Pos              (0)                                               /*!< PWM_T::CMPDAT3: CMP Position           */
#define PWM_CMPDAT3_CMP_Msk              (0xfffful << PWM_CMPDAT3_CMP_Pos)                 /*!< PWM_T::CMPDAT3: CMP Mask               */

#define PWM_CNT3_CNT_Pos                 (0)                                               /*!< PWM_T::CNT3: CNT Position              */
#define PWM_CNT3_CNT_Msk                 (0xfffful << PWM_CNT3_CNT_Pos)                    /*!< PWM_T::CNT3: CNT Mask                  */

#define PWM_INTEN_PIEN0_Pos              (0)                                               /*!< PWM_T::INTEN: PIEN0 Position           */
#define PWM_INTEN_PIEN0_Msk              (0x1ul << PWM_INTEN_PIEN0_Pos)                    /*!< PWM_T::INTEN: PIEN0 Mask               */

#define PWM_INTEN_PIEN1_Pos              (1)                                               /*!< PWM_T::INTEN: PIEN1 Position           */
#define PWM_INTEN_PIEN1_Msk              (0x1ul << PWM_INTEN_PIEN1_Pos)                    /*!< PWM_T::INTEN: PIEN1 Mask               */

#define PWM_INTEN_PIEN2_Pos              (2)                                               /*!< PWM_T::INTEN: PIEN2 Position           */
#define PWM_INTEN_PIEN2_Msk              (0x1ul << PWM_INTEN_PIEN2_Pos)                    /*!< PWM_T::INTEN: PIEN2 Mask               */

#define PWM_INTEN_PIEN3_Pos              (3)                                               /*!< PWM_T::INTEN: PIEN3 Position           */
#define PWM_INTEN_PIEN3_Msk              (0x1ul << PWM_INTEN_PIEN3_Pos)                    /*!< PWM_T::INTEN: PIEN3 Mask               */

#define PWM_INTSTS_PIF0_Pos              (0)                                               /*!< PWM_T::INTSTS: PIF0 Position           */
#define PWM_INTSTS_PIF0_Msk              (0x1ul << PWM_INTSTS_PIF0_Pos)                    /*!< PWM_T::INTSTS: PIF0 Mask               */

#define PWM_INTSTS_PIF1_Pos              (1)                                               /*!< PWM_T::INTSTS: PIF1 Position           */
#define PWM_INTSTS_PIF1_Msk              (0x1ul << PWM_INTSTS_PIF1_Pos)                    /*!< PWM_T::INTSTS: PIF1 Mask               */

#define PWM_INTSTS_PIF2_Pos              (2)                                               /*!< PWM_T::INTSTS: PIF2 Position           */
#define PWM_INTSTS_PIF2_Msk              (0x1ul << PWM_INTSTS_PIF2_Pos)                    /*!< PWM_T::INTSTS: PIF2 Mask               */

#define PWM_INTSTS_PIF3_Pos              (3)                                               /*!< PWM_T::INTSTS: PIF3 Position           */
#define PWM_INTSTS_PIF3_Msk              (0x1ul << PWM_INTSTS_PIF3_Pos)                    /*!< PWM_T::INTSTS: PIF3 Mask               */

#define PWM_CAPCTL01_CAPINV0_Pos         (0)                                               /*!< PWM_T::CAPCTL01: CAPINV0 Position      */
#define PWM_CAPCTL01_CAPINV0_Msk         (0x1ul << PWM_CAPCTL01_CAPINV0_Pos)               /*!< PWM_T::CAPCTL01: CAPINV0 Mask          */

#define PWM_CAPCTL01_CRLIEN0_Pos         (1)                                               /*!< PWM_T::CAPCTL01: CRLIEN0 Position      */
#define PWM_CAPCTL01_CRLIEN0_Msk         (0x1ul << PWM_CAPCTL01_CRLIEN0_Pos)               /*!< PWM_T::CAPCTL01: CRLIEN0 Mask          */

#define PWM_CAPCTL01_CFLIEN0_Pos         (2)                                               /*!< PWM_T::CAPCTL01: CFLIEN0 Position      */
#define PWM_CAPCTL01_CFLIEN0_Msk         (0x1ul << PWM_CAPCTL01_CFLIEN0_Pos)               /*!< PWM_T::CAPCTL01: CFLIEN0 Mask          */

#define PWM_CAPCTL01_CAPEN0_Pos          (3)                                               /*!< PWM_T::CAPCTL01: CAPEN0 Position       */
#define PWM_CAPCTL01_CAPEN0_Msk          (0x1ul << PWM_CAPCTL01_CAPEN0_Pos)                /*!< PWM_T::CAPCTL01: CAPEN0 Mask           */

#define PWM_CAPCTL01_CAPIF0_Pos          (4)                                               /*!< PWM_T::CAPCTL01: CAPIF0 Position       */
#define PWM_CAPCTL01_CAPIF0_Msk          (0x1ul << PWM_CAPCTL01_CAPIF0_Pos)                /*!< PWM_T::CAPCTL01: CAPIF0 Mask           */

#define PWM_CAPCTL01_CRLIF0_Pos          (6)                                               /*!< PWM_T::CAPCTL01: CRLIF0 Position       */
#define PWM_CAPCTL01_CRLIF0_Msk          (0x1ul << PWM_CAPCTL01_CRLIF0_Pos)                /*!< PWM_T::CAPCTL01: CRLIF0 Mask           */

#define PWM_CAPCTL01_CFLIF0_Pos          (7)                                               /*!< PWM_T::CAPCTL01: CFLIF0 Position       */
#define PWM_CAPCTL01_CFLIF0_Msk          (0x1ul << PWM_CAPCTL01_CFLIF0_Pos)                /*!< PWM_T::CAPCTL01: CFLIF0 Mask           */

#define PWM_CAPCTL01_CAPINV1_Pos         (16)                                              /*!< PWM_T::CAPCTL01: CAPINV1 Position      */
#define PWM_CAPCTL01_CAPINV1_Msk         (0x1ul << PWM_CAPCTL01_CAPINV1_Pos)               /*!< PWM_T::CAPCTL01: CAPINV1 Mask          */

#define PWM_CAPCTL01_CRLIEN1_Pos         (17)                                              /*!< PWM_T::CAPCTL01: CRLIEN1 Position      */
#define PWM_CAPCTL01_CRLIEN1_Msk         (0x1ul << PWM_CAPCTL01_CRLIEN1_Pos)               /*!< PWM_T::CAPCTL01: CRLIEN1 Mask          */

#define PWM_CAPCTL01_CFLIEN1_Pos         (18)                                              /*!< PWM_T::CAPCTL01: CFLIEN1 Position      */
#define PWM_CAPCTL01_CFLIEN1_Msk         (0x1ul << PWM_CAPCTL01_CFLIEN1_Pos)               /*!< PWM_T::CAPCTL01: CFLIEN1 Mask          */

#define PWM_CAPCTL01_CAPEN1_Pos          (19)                                              /*!< PWM_T::CAPCTL01: CAPEN1 Position       */
#define PWM_CAPCTL01_CAPEN1_Msk          (0x1ul << PWM_CAPCTL01_CAPEN1_Pos)                /*!< PWM_T::CAPCTL01: CAPEN1 Mask           */

#define PWM_CAPCTL01_CAPIF1_Pos          (20)                                              /*!< PWM_T::CAPCTL01: CAPIF1 Position       */
#define PWM_CAPCTL01_CAPIF1_Msk          (0x1ul << PWM_CAPCTL01_CAPIF1_Pos)                /*!< PWM_T::CAPCTL01: CAPIF1 Mask           */

#define PWM_CAPCTL01_CRLIF1_Pos          (22)                                              /*!< PWM_T::CAPCTL01: CRLIF1 Position       */
#define PWM_CAPCTL01_CRLIF1_Msk          (0x1ul << PWM_CAPCTL01_CRLIF1_Pos)                /*!< PWM_T::CAPCTL01: CRLIF1 Mask           */

#define PWM_CAPCTL01_CFLIF1_Pos          (23)                                              /*!< PWM_T::CAPCTL01: CFLIF1 Position       */
#define PWM_CAPCTL01_CFLIF1_Msk          (0x1ul << PWM_CAPCTL01_CFLIF1_Pos)                /*!< PWM_T::CAPCTL01: CFLIF1 Mask           */

#define PWM_CAPCTL23_CAPINV2_Pos         (0)                                               /*!< PWM_T::CAPCTL23: CAPINV2 Position      */
#define PWM_CAPCTL23_CAPINV2_Msk         (0x1ul << PWM_CAPCTL23_CAPINV2_Pos)               /*!< PWM_T::CAPCTL23: CAPINV2 Mask          */

#define PWM_CAPCTL23_CRLIEN2_Pos         (1)                                               /*!< PWM_T::CAPCTL23: CRLIEN2 Position      */
#define PWM_CAPCTL23_CRLIEN2_Msk         (0x1ul << PWM_CAPCTL23_CRLIEN2_Pos)               /*!< PWM_T::CAPCTL23: CRLIEN2 Mask          */

#define PWM_CAPCTL23_CFLIEN2_Pos         (2)                                               /*!< PWM_T::CAPCTL23: CFLIEN2 Position      */
#define PWM_CAPCTL23_CFLIEN2_Msk         (0x1ul << PWM_CAPCTL23_CFLIEN2_Pos)               /*!< PWM_T::CAPCTL23: CFLIEN2 Mask          */

#define PWM_CAPCTL23_CAPEN2_Pos          (3)                                               /*!< PWM_T::CAPCTL23: CAPEN2 Position       */
#define PWM_CAPCTL23_CAPEN2_Msk          (0x1ul << PWM_CAPCTL23_CAPEN2_Pos)                /*!< PWM_T::CAPCTL23: CAPEN2 Mask           */

#define PWM_CAPCTL23_CAPIF2_Pos          (4)                                               /*!< PWM_T::CAPCTL23: CAPIF2 Position       */
#define PWM_CAPCTL23_CAPIF2_Msk          (0x1ul << PWM_CAPCTL23_CAPIF2_Pos)                /*!< PWM_T::CAPCTL23: CAPIF2 Mask           */

#define PWM_CAPCTL23_CRLIF2_Pos          (6)                                               /*!< PWM_T::CAPCTL23: CRLIF2 Position       */
#define PWM_CAPCTL23_CRLIF2_Msk          (0x1ul << PWM_CAPCTL23_CRLIF2_Pos)                /*!< PWM_T::CAPCTL23: CRLIF2 Mask           */

#define PWM_CAPCTL23_CFLIF2_Pos          (7)                                               /*!< PWM_T::CAPCTL23: CFLIF2 Position       */
#define PWM_CAPCTL23_CFLIF2_Msk          (0x1ul << PWM_CAPCTL23_CFLIF2_Pos)                /*!< PWM_T::CAPCTL23: CFLIF2 Mask           */

#define PWM_CAPCTL23_CAPINV3_Pos         (16)                                              /*!< PWM_T::CAPCTL23: CAPINV3 Position      */
#define PWM_CAPCTL23_CAPINV3_Msk         (0x1ul << PWM_CAPCTL23_CAPINV3_Pos)               /*!< PWM_T::CAPCTL23: CAPINV3 Mask          */

#define PWM_CAPCTL23_CRLIEN3_Pos         (17)                                              /*!< PWM_T::CAPCTL23: CRLIEN3 Position      */
#define PWM_CAPCTL23_CRLIEN3_Msk         (0x1ul << PWM_CAPCTL23_CRLIEN3_Pos)               /*!< PWM_T::CAPCTL23: CRLIEN3 Mask          */

#define PWM_CAPCTL23_CFLIEN3_Pos         (18)                                              /*!< PWM_T::CAPCTL23: CFLIEN3 Position      */
#define PWM_CAPCTL23_CFLIEN3_Msk         (0x1ul << PWM_CAPCTL23_CFLIEN3_Pos)               /*!< PWM_T::CAPCTL23: CFLIEN3 Mask          */

#define PWM_CAPCTL23_CAPEN3_Pos          (19)                                              /*!< PWM_T::CAPCTL23: CAPEN3 Position       */
#define PWM_CAPCTL23_CAPEN3_Msk          (0x1ul << PWM_CAPCTL23_CAPEN3_Pos)                /*!< PWM_T::CAPCTL23: CAPEN3 Mask           */

#define PWM_CAPCTL23_CAPIF3_Pos          (20)                                              /*!< PWM_T::CAPCTL23: CAPIF3 Position       */
#define PWM_CAPCTL23_CAPIF3_Msk          (0x1ul << PWM_CAPCTL23_CAPIF3_Pos)                /*!< PWM_T::CAPCTL23: CAPIF3 Mask           */

#define PWM_CAPCTL23_CRLIF3_Pos          (22)                                              /*!< PWM_T::CAPCTL23: CRLIF3 Position       */
#define PWM_CAPCTL23_CRLIF3_Msk          (0x1ul << PWM_CAPCTL23_CRLIF3_Pos)                /*!< PWM_T::CAPCTL23: CRLIF3 Mask           */

#define PWM_CAPCTL23_CFLIF3_Pos          (23)                                              /*!< PWM_T::CAPCTL23: CFLIF3 Position       */
#define PWM_CAPCTL23_CFLIF3_Msk          (0x1ul << PWM_CAPCTL23_CFLIF3_Pos)                /*!< PWM_T::CAPCTL23: CFLIF3 Mask           */

#define PWM_RCAPDAT0_RCAPDAT_Pos         (0)                                               /*!< PWM_T::RCAPDAT0: RCAPDAT Position      */
#define PWM_RCAPDAT0_RCAPDAT_Msk         (0xfffful << PWM_RCAPDAT0_RCAPDAT_Pos)            /*!< PWM_T::RCAPDAT0: RCAPDAT Mask          */

#define PWM_FCAPDAT0_FCAPDAT_Pos         (0)                                               /*!< PWM_T::FCAPDAT0: FCAPDAT Position      */
#define PWM_FCAPDAT0_FCAPDAT_Msk         (0xfffful << PWM_FCAPDAT0_FCAPDAT_Pos)            /*!< PWM_T::FCAPDAT0: FCAPDAT Mask          */

#define PWM_RCAPDAT1_RCAPDAT_Pos         (0)                                               /*!< PWM_T::RCAPDAT1: RCAPDAT Position      */
#define PWM_RCAPDAT1_RCAPDAT_Msk         (0xfffful << PWM_RCAPDAT1_RCAPDAT_Pos)            /*!< PWM_T::RCAPDAT1: RCAPDAT Mask          */

#define PWM_FCAPDAT1_FCAPDAT_Pos         (0)                                               /*!< PWM_T::FCAPDAT1: FCAPDAT Position      */
#define PWM_FCAPDAT1_FCAPDAT_Msk         (0xfffful << PWM_FCAPDAT1_FCAPDAT_Pos)            /*!< PWM_T::FCAPDAT1: FCAPDAT Mask          */

#define PWM_RCAPDAT2_RCAPDAT_Pos         (0)                                               /*!< PWM_T::RCAPDAT2: RCAPDAT Position      */
#define PWM_RCAPDAT2_RCAPDAT_Msk         (0xfffful << PWM_RCAPDAT2_RCAPDAT_Pos)            /*!< PWM_T::RCAPDAT2: RCAPDAT Mask          */

#define PWM_FCAPDAT2_FCAPDAT_Pos         (0)                                               /*!< PWM_T::FCAPDAT2: FCAPDAT Position      */
#define PWM_FCAPDAT2_FCAPDAT_Msk         (0xfffful << PWM_FCAPDAT2_FCAPDAT_Pos)            /*!< PWM_T::FCAPDAT2: FCAPDAT Mask          */

#define PWM_RCAPDAT3_RCAPDAT_Pos         (0)                                               /*!< PWM_T::RCAPDAT3: RCAPDAT Position      */
#define PWM_RCAPDAT3_RCAPDAT_Msk         (0xfffful << PWM_RCAPDAT3_RCAPDAT_Pos)            /*!< PWM_T::RCAPDAT3: RCAPDAT Mask          */

#define PWM_FCAPDAT3_FCAPDAT_Pos         (0)                                               /*!< PWM_T::FCAPDAT3: FCAPDAT Position      */
#define PWM_FCAPDAT3_FCAPDAT_Msk         (0xfffful << PWM_FCAPDAT3_FCAPDAT_Pos)            /*!< PWM_T::FCAPDAT3: FCAPDAT Mask          */

#define PWM_CAPINEN_CAPINEN_Pos          (0)                                               /*!< PWM_T::CAPINEN: CAPINEN Position       */
#define PWM_CAPINEN_CAPINEN_Msk          (0xful << PWM_CAPINEN_CAPINEN_Pos)                /*!< PWM_T::CAPINEN: CAPINEN Mask           */

#define PWM_POEN_POEN0_Pos               (0)                                               /*!< PWM_T::POEN: POEN0 Position            */
#define PWM_POEN_POEN0_Msk               (0x1ul << PWM_POEN_POEN0_Pos)                     /*!< PWM_T::POEN: POEN0 Mask                */

#define PWM_POEN_POEN1_Pos               (1)                                               /*!< PWM_T::POEN: POEN1 Position            */
#define PWM_POEN_POEN1_Msk               (0x1ul << PWM_POEN_POEN1_Pos)                     /*!< PWM_T::POEN: POEN1 Mask                */

#define PWM_POEN_POEN2_Pos               (2)                                               /*!< PWM_T::POEN: POEN2 Position            */
#define PWM_POEN_POEN2_Msk               (0x1ul << PWM_POEN_POEN2_Pos)                     /*!< PWM_T::POEN: POEN2 Mask                */

#define PWM_POEN_POEN3_Pos               (3)                                               /*!< PWM_T::POEN: POEN3 Position            */
#define PWM_POEN_POEN3_Msk               (0x1ul << PWM_POEN_POEN3_Pos)                     /*!< PWM_T::POEN: POEN3 Mask                */

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
 * @var RTC_T::INIT
 * Offset: 0x00  RTC Initialization Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ATVSTS    |RTC Active Status (Read Only)
 * |        |          |0: RTC is in reset state
 * |        |          |1: RTC is in normal active state.
 * |[31:1]  |INIT      |RTC Initialization
 * |        |          |After a power-on reset (POR) RTC block should be initialized by writing 0xA5EB1357 to INIT
 * |        |          |This will force a hardware reset then release all logic and counters
 * @var RTC_T::RWEN
 * Offset: 0x04  RTC Access Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |RWEN      |RTC Register Access Enable Password (Write Only)
 * |        |          |0xA965 = Enable RTC acces..s
 * |        |          |Others = Disable RTC acces..s 
 * |[16]    |RWENF     |RTC Register Access Enable Flag (Read Only)
 * |        |          |1 = RTC register read/write enable.
 * |        |          |0 = RTC register read/write disable.
 * |        |          |This bit will be set after RWEN[15:0] register is set to 0xA965, it will clear automatically in 512 RTC clock cycles or RWEN[15:0] ! = 0xA965
 * |        |          |The effect of RTC_RWEN.RWENF is as the below.
 * |        |          |Table 5-79 RTC_RWEN.RWENF Register Access Effect.
 * |        |          |Register : RWENF = 1 : RWENF = 0.
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
 * @var RTC_T::FREQADJ
 * Offset: 0x08  RTC Frequency Compensation Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[5:0]   |FRACTION  |Fractional Part
 * |        |          |Formula = (fraction part of detected value) x 60.
 * |        |          |Refer to Table 5-76 RTC Frequency Compensation Example for the examples.
 * |[11:8]  |INTEGER   |Integer Part
 * |        |          |Register should contain the value (INT(Factual) u2013 32761)
 * |        |          |Ex: Integer part of detected value = 32772,.
 * |        |          | RTC_FREQADJ.INTEGER = 32772-32761 = 11 (..1011b)
 * |        |          |The range between 32761 and 32776
 * @var RTC_T::TIME
 * Offset: 0x0C  Time Load Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |SEC       |1 Sec Time Digit (0~9)
 * |[6:4]   |TENSEC    |10 Sec Time Digit (0~5)
 * |[11:8]  |MIN       |1 Min Time Digit (0~9)
 * |[14:12] |TENMIN    |10 Min Time Digit (0~5)
 * |[19:16] |HR        |1 Hour Time Digit (0~9)
 * |[21:20] |TENHR     |10 Hour Time Digit (0~3)
 * @var RTC_T::CAL
 * Offset: 0x10  Calendar Load Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |DAY       |1-Day Calendar Digit (0~9)
 * |[5:4]   |TENDAY    |10-Day Calendar Digit (0~3)
 * |[11:8]  |MON       |1-Month Calendar Digit (0~9)
 * |[12]    |TENMON    |10-Month Calendar Digit (0~1)
 * |[19:16] |YEAR      |1-Year Calendar Digit (0~9)
 * |[23:20] |TENYEAR   |10-Year Calendar Digit (0~9)
 * @var RTC_T::CLKFMT
 * Offset: 0x14  Time Scale Selection Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |24HEN     |24-hour / 12-hour Mode Selection
 * |        |          |Determines whether RTC_TIME and RTC_TALM are in 24-hour mode or 12-hour mode
 * |        |          |1 = select 24-hour time scale.
 * |        |          |0 = select 12-hour time scale with AM and PM indication.
 * |        |          |The range of 24-hour time scale is between 0 and 23.
 * |        |          |12-hour time scale:
 * |        |          |01(AM01), 02(AM02), 03(AM03), 04(AM04), 05(AM05), 06(AM06)
 * |        |          |07(AM07), 08(AM08), 09(AM09), 10(AM10), 11(AM11), 12(AM12)
 * |        |          |21(PM01), 22(PM02), 23(PM03), 24(PM04), 25(PM05), 26(PM06)
 * |        |          |27(PM07), 28(PM08), 29(PM09), 30(PM10), 31(PM11), 32(PM12)
 * @var RTC_T::WEEKDAY
 * Offset: 0x18  Day of the Week Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |WEEKDAY   |Day of the Week Register
 * |        |          |0 (Sunday), 1 (Monday), 2 (Tuesday), 3 (Wednesday)
 * |        |          |4 (Thursday), 5 (Friday), 6 (Saturday)
 * @var RTC_T::TALM
 * Offset: 0x1C  Time Alarm Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |SEC       |1 Sec Time Digit of Alarm Setting (0~9)
 * |[6:4]   |TENSEC    |10 Sec Time Digit of Alarm Setting (0~5)
 * |[11:8]  |MIN       |1 Min Time Digit of Alarm Setting (0~9)
 * |[14:12] |TENMIN    |10 Min Time Digit of Alarm Setting (0~5)
 * |[19:16] |HR        |1 Hour Time Digit of Alarm Setting (0~9)
 * |[21:20] |TENHR     |10 Hour Time Digit of Alarm Setting (0~3)2
 * @var RTC_T::CALM
 * Offset: 0x20  Calendar Alarm Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |DAY       |1-Day Calendar Digit of Alarm Setting (0~9)
 * |[5:4]   |TENDAY    |10-Day Calendar Digit of Alarm Setting (0~3)
 * |[11:8]  |MON       |1-Month Calendar Digit of Alarm Setting (0~9)
 * |[12]    |TENMON    |10-Month Calendar Digit of Alarm Setting (0~1)
 * |[19:16] |YEAR      |1-Year Calendar Digit of Alarm Setting (0~9)
 * |[23:20] |TENYEAR   |10-Year Calendar Digit of Alarm Setting (0~9)
 * @var RTC_T::LEAPYEAR
 * Offset: 0x24  Leap Year Indicator Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |LEAPYEAR  |Leap Year Indication Register (Read Only)
 * |        |          |0 = Current year is not a leap year.
 * |        |          |1 = Current year is leap year.
 * @var RTC_T::INTEN
 * Offset: 0x28  RTC Interrupt Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ALMIEN    |Alarm Interrupt Enable
 * |        |          |1 = RTC Alarm Interrupt is enabled.
 * |        |          |0 = RTC Alarm Interrupt is disabled.
 * |[1]     |TICKIEN   |Time-tick Interrupt and Wakeup-by-tick Enable
 * |        |          |1 = RTC Time-Tick Interrupt is enabled.
 * |        |          |0 = RTC Time-Tick Interrupt is disabled.
 * @var RTC_T::INTSTS
 * Offset: 0x2C  RTC Interrupt Indicator Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ALMIF     |RTC Alarm Interrupt Flag
 * |        |          |When RTC Alarm Interrupt is enabled (RTC_INTEN.ALMIF=1), RTC unit will set ALMIF to high once the RTC real time counters RTC_TIME and RTC_CAL reach the alarm setting time registers RTC_TALM and RTC_CALM
 * |        |          |This bit cleared/acknowledged by writing 1 to it.
 * |        |          |0= Indicates no Alarm Interrupt condition.
 * |        |          |1= Indicates RTC Alarm Interrupt generated.
 * |[1]     |TICKIF    |RTC Time-tick Interrupt Flag
 * |        |          |When RTC Time-Tick Interrupt is enabled (RTC_INTEN.TICKIF=1), RTC unit will set TIF high at the rate selected by RTC_TICK[2:0]
 * |        |          |This bit cleared/acknowledged by writing 1 to it.
 * |        |          |0= Indicates no Time-Tick Interrupt condition.
 * |        |          |1= Indicates RTC Time-Tick Interrupt generated.
 * @var RTC_T::TICK
 * Offset: 0x30  RTC Time Tick Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2:0]   |TICKSEL   |Time Tick Period Select
 * |        |          |The RTC time tick period for Periodic Time-Tick Interrupt request.
 * |        |          |Time Tick (second) : 1 / (2^TTR)
 * |        |          |Note: This register can be read back after the RTC is active.
 * |[3]     |TWKEN     |RTC Timer Wakeup CPU Function Enable Bit
 * |        |          |If TWKE is set before CPU is in power-down mode, when a RTC Time-Tick or Alarm Match occurs, CPU will wake up.
 * |        |          |0= Disable Wakeup CPU function.
 * |        |          |1= Enable the Wakeup function.
 */
    __IO uint32_t INIT;                  /*!< [0x0000] RTC Initialization Register                                      */
    __IO uint32_t RWEN;                  /*!< [0x0004] RTC Access Enable Register                                       */
    __IO uint32_t FREQADJ;               /*!< [0x0008] RTC Frequency Compensation Register                              */
    __IO uint32_t TIME;                  /*!< [0x000c] Time Load Register                                               */
    __IO uint32_t CAL;                   /*!< [0x0010] Calendar Load Register                                           */
    __IO uint32_t CLKFMT;                /*!< [0x0014] Time Scale Selection Register                                    */
    __IO uint32_t WEEKDAY;               /*!< [0x0018] Day of the Week Register                                         */
    __IO uint32_t TALM;                  /*!< [0x001c] Time Alarm Register                                              */
    __IO uint32_t CALM;                  /*!< [0x0020] Calendar Alarm Register                                          */
    __I  uint32_t LEAPYEAR;              /*!< [0x0024] Leap Year Indicator Register                                     */
    __IO uint32_t INTEN;                 /*!< [0x0028] RTC Interrupt Enable Register                                    */
    __IO uint32_t INTSTS;                /*!< [0x002c] RTC Interrupt Indicator Register                                 */
    __IO uint32_t TICK;                  /*!< [0x0030] RTC Time Tick Register                                           */

} RTC_T;

/**
    @addtogroup RTC_CONST RTC Bit Field Definition
    Constant Definitions for RTC Controller
@{ */

#define RTC_INIT_ATVSTS_Pos              (0)                                               /*!< RTC_T::INIT: ATVSTS Position           */
#define RTC_INIT_ATVSTS_Msk              (0x1ul << RTC_INIT_ATVSTS_Pos)                    /*!< RTC_T::INIT: ATVSTS Mask               */

#define RTC_INIT_INIT_Pos                (1)                                               /*!< RTC_T::INIT: INIT Position             */
#define RTC_INIT_INIT_Msk                (0x7ffffffful << RTC_INIT_INIT_Pos)               /*!< RTC_T::INIT: INIT Mask                 */

#define RTC_RWEN_RWEN_Pos                (0)                                               /*!< RTC_T::RWEN: RWEN Position             */
#define RTC_RWEN_RWEN_Msk                (0xfffful << RTC_RWEN_RWEN_Pos)                   /*!< RTC_T::RWEN: RWEN Mask                 */

#define RTC_RWEN_RWENF_Pos               (16)                                              /*!< RTC_T::RWEN: RWENF Position            */
#define RTC_RWEN_RWENF_Msk               (0x1ul << RTC_RWEN_RWENF_Pos)                     /*!< RTC_T::RWEN: RWENF Mask                */

#define RTC_FREQADJ_FRACTION_Pos         (0)                                               /*!< RTC_T::FREQADJ: FRACTION Position      */
#define RTC_FREQADJ_FRACTION_Msk         (0x3ful << RTC_FREQADJ_FRACTION_Pos)              /*!< RTC_T::FREQADJ: FRACTION Mask          */

#define RTC_FREQADJ_INTEGER_Pos          (8)                                               /*!< RTC_T::FREQADJ: INTEGER Position       */
#define RTC_FREQADJ_INTEGER_Msk          (0xful << RTC_FREQADJ_INTEGER_Pos)                /*!< RTC_T::FREQADJ: INTEGER Mask           */

#define RTC_TIME_SEC_Pos                 (0)                                               /*!< RTC_T::TIME: SEC Position              */
#define RTC_TIME_SEC_Msk                 (0xful << RTC_TIME_SEC_Pos)                       /*!< RTC_T::TIME: SEC Mask                  */

#define RTC_TIME_TENSEC_Pos              (4)                                               /*!< RTC_T::TIME: TENSEC Position           */
#define RTC_TIME_TENSEC_Msk              (0x7ul << RTC_TIME_TENSEC_Pos)                    /*!< RTC_T::TIME: TENSEC Mask               */

#define RTC_TIME_MIN_Pos                 (8)                                               /*!< RTC_T::TIME: MIN Position              */
#define RTC_TIME_MIN_Msk                 (0xful << RTC_TIME_MIN_Pos)                       /*!< RTC_T::TIME: MIN Mask                  */

#define RTC_TIME_TENMIN_Pos              (12)                                              /*!< RTC_T::TIME: TENMIN Position           */
#define RTC_TIME_TENMIN_Msk              (0x7ul << RTC_TIME_TENMIN_Pos)                    /*!< RTC_T::TIME: TENMIN Mask               */

#define RTC_TIME_HR_Pos                  (16)                                              /*!< RTC_T::TIME: HR Position               */
#define RTC_TIME_HR_Msk                  (0xful << RTC_TIME_HR_Pos)                        /*!< RTC_T::TIME: HR Mask                   */

#define RTC_TIME_TENHR_Pos               (20)                                              /*!< RTC_T::TIME: TENHR Position            */
#define RTC_TIME_TENHR_Msk               (0x3ul << RTC_TIME_TENHR_Pos)                     /*!< RTC_T::TIME: TENHR Mask                */

#define RTC_CAL_DAY_Pos                  (0)                                               /*!< RTC_T::CAL: DAY Position               */
#define RTC_CAL_DAY_Msk                  (0xful << RTC_CAL_DAY_Pos)                        /*!< RTC_T::CAL: DAY Mask                   */

#define RTC_CAL_TENDAY_Pos               (4)                                               /*!< RTC_T::CAL: TENDAY Position            */
#define RTC_CAL_TENDAY_Msk               (0x3ul << RTC_CAL_TENDAY_Pos)                     /*!< RTC_T::CAL: TENDAY Mask                */

#define RTC_CAL_MON_Pos                  (8)                                               /*!< RTC_T::CAL: MON Position               */
#define RTC_CAL_MON_Msk                  (0xful << RTC_CAL_MON_Pos)                        /*!< RTC_T::CAL: MON Mask                   */

#define RTC_CAL_TENMON_Pos               (12)                                              /*!< RTC_T::CAL: TENMON Position            */
#define RTC_CAL_TENMON_Msk               (0x1ul << RTC_CAL_TENMON_Pos)                     /*!< RTC_T::CAL: TENMON Mask                */

#define RTC_CAL_YEAR_Pos                 (16)                                              /*!< RTC_T::CAL: YEAR Position              */
#define RTC_CAL_YEAR_Msk                 (0xful << RTC_CAL_YEAR_Pos)                       /*!< RTC_T::CAL: YEAR Mask                  */

#define RTC_CAL_TENYEAR_Pos              (20)                                              /*!< RTC_T::CAL: TENYEAR Position           */
#define RTC_CAL_TENYEAR_Msk              (0xful << RTC_CAL_TENYEAR_Pos)                    /*!< RTC_T::CAL: TENYEAR Mask               */

#define RTC_CLKFMT_24HEN_Pos             (0)                                               /*!< RTC_T::CLKFMT: 24HEN Position          */
#define RTC_CLKFMT_24HEN_Msk             (0x1ul << RTC_CLKFMT_24HEN_Pos)                   /*!< RTC_T::CLKFMT: 24HEN Mask              */

#define RTC_WEEKDAY_WEEKDAY_Pos          (0)                                               /*!< RTC_T::WEEKDAY: WEEKDAY Position       */
#define RTC_WEEKDAY_WEEKDAY_Msk          (0x7ul << RTC_WEEKDAY_WEEKDAY_Pos)                /*!< RTC_T::WEEKDAY: WEEKDAY Mask           */

#define RTC_TALM_SEC_Pos                 (0)                                               /*!< RTC_T::TALM: SEC Position              */
#define RTC_TALM_SEC_Msk                 (0xful << RTC_TALM_SEC_Pos)                       /*!< RTC_T::TALM: SEC Mask                  */

#define RTC_TALM_TENSEC_Pos              (4)                                               /*!< RTC_T::TALM: TENSEC Position           */
#define RTC_TALM_TENSEC_Msk              (0x7ul << RTC_TALM_TENSEC_Pos)                    /*!< RTC_T::TALM: TENSEC Mask               */

#define RTC_TALM_MIN_Pos                 (8)                                               /*!< RTC_T::TALM: MIN Position              */
#define RTC_TALM_MIN_Msk                 (0xful << RTC_TALM_MIN_Pos)                       /*!< RTC_T::TALM: MIN Mask                  */

#define RTC_TALM_TENMIN_Pos              (12)                                              /*!< RTC_T::TALM: TENMIN Position           */
#define RTC_TALM_TENMIN_Msk              (0x7ul << RTC_TALM_TENMIN_Pos)                    /*!< RTC_T::TALM: TENMIN Mask               */

#define RTC_TALM_HR_Pos                  (16)                                              /*!< RTC_T::TALM: HR Position               */
#define RTC_TALM_HR_Msk                  (0xful << RTC_TALM_HR_Pos)                        /*!< RTC_T::TALM: HR Mask                   */

#define RTC_TALM_TENHR_Pos               (20)                                              /*!< RTC_T::TALM: TENHR Position            */
#define RTC_TALM_TENHR_Msk               (0x3ul << RTC_TALM_TENHR_Pos)                     /*!< RTC_T::TALM: TENHR Mask                */

#define RTC_CALM_DAY_Pos                 (0)                                               /*!< RTC_T::CALM: DAY Position              */
#define RTC_CALM_DAY_Msk                 (0xful << RTC_CALM_DAY_Pos)                       /*!< RTC_T::CALM: DAY Mask                  */

#define RTC_CALM_TENDAY_Pos              (4)                                               /*!< RTC_T::CALM: TENDAY Position           */
#define RTC_CALM_TENDAY_Msk              (0x3ul << RTC_CALM_TENDAY_Pos)                    /*!< RTC_T::CALM: TENDAY Mask               */

#define RTC_CALM_MON_Pos                 (8)                                               /*!< RTC_T::CALM: MON Position              */
#define RTC_CALM_MON_Msk                 (0xful << RTC_CALM_MON_Pos)                       /*!< RTC_T::CALM: MON Mask                  */

#define RTC_CALM_TENMON_Pos              (12)                                              /*!< RTC_T::CALM: TENMON Position           */
#define RTC_CALM_TENMON_Msk              (0x1ul << RTC_CALM_TENMON_Pos)                    /*!< RTC_T::CALM: TENMON Mask               */

#define RTC_CALM_YEAR_Pos                (16)                                              /*!< RTC_T::CALM: YEAR Position             */
#define RTC_CALM_YEAR_Msk                (0xful << RTC_CALM_YEAR_Pos)                      /*!< RTC_T::CALM: YEAR Mask                 */

#define RTC_CALM_TENYEAR_Pos             (20)                                              /*!< RTC_T::CALM: TENYEAR Position          */
#define RTC_CALM_TENYEAR_Msk             (0xful << RTC_CALM_TENYEAR_Pos)                   /*!< RTC_T::CALM: TENYEAR Mask              */

#define RTC_LEAPYEAR_LEAPYEAR_Pos        (0)                                               /*!< RTC_T::LEAPYEAR: LEAPYEAR Position     */
#define RTC_LEAPYEAR_LEAPYEAR_Msk        (0x1ul << RTC_LEAPYEAR_LEAPYEAR_Pos)              /*!< RTC_T::LEAPYEAR: LEAPYEAR Mask         */

#define RTC_INTEN_ALMIEN_Pos             (0)                                               /*!< RTC_T::INTEN: ALMIEN Position          */
#define RTC_INTEN_ALMIEN_Msk             (0x1ul << RTC_INTEN_ALMIEN_Pos)                   /*!< RTC_T::INTEN: ALMIEN Mask              */

#define RTC_INTEN_TICKIEN_Pos            (1)                                               /*!< RTC_T::INTEN: TICKIEN Position         */
#define RTC_INTEN_TICKIEN_Msk            (0x1ul << RTC_INTEN_TICKIEN_Pos)                  /*!< RTC_T::INTEN: TICKIEN Mask             */

#define RTC_INTSTS_ALMIF_Pos             (0)                                               /*!< RTC_T::INTSTS: ALMIF Position          */
#define RTC_INTSTS_ALMIF_Msk             (0x1ul << RTC_INTSTS_ALMIF_Pos)                   /*!< RTC_T::INTSTS: ALMIF Mask              */

#define RTC_INTSTS_TICKIF_Pos            (1)                                               /*!< RTC_T::INTSTS: TICKIF Position         */
#define RTC_INTSTS_TICKIF_Msk            (0x1ul << RTC_INTSTS_TICKIF_Pos)                  /*!< RTC_T::INTSTS: TICKIF Mask             */

#define RTC_TICK_TICKSEL_Pos             (0)                                               /*!< RTC_T::TICK: TICKSEL Position          */
#define RTC_TICK_TICKSEL_Msk             (0x7ul << RTC_TICK_TICKSEL_Pos)                   /*!< RTC_T::TICK: TICKSEL Mask              */

#define RTC_TICK_TWKEN_Pos               (3)                                               /*!< RTC_T::TICK: TWKEN Position            */
#define RTC_TICK_TWKEN_Msk               (0x1ul << RTC_TICK_TWKEN_Pos)                     /*!< RTC_T::TICK: TWKEN Mask                */

/**@}*/ /* RTC_CONST */
/**@}*/ /* end of RTC register group */


/*---------------------- Successive Approximation Analog-to-Digital Convertor -------------------------*/
/**
    @addtogroup SARADC Successive Approximation Analog-to-Digital Convertor(SARADC)
    Memory Mapped Structure for SARADC Controller
@{ */
 
typedef struct
{


/**
 * @var SARADC_T::DAT
 * Offset: 0x00  SAR ADC Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[11:0]  |RESULT    |A/D Conversion Result
 * |        |          |This field contains conversion result of SARADC.
 * |        |          |12-bit SARADC conversion result with unsigned format.
 * |[16]    |OV        |Overrun Flag (Read Only)
 * |        |          |0 = Data in RESULT[11:0] is recent conversion result.
 * |        |          |1 = Data in RESULT[11:0] is overwritten.
 * |        |          |Note: If converted data in RESULT[11:0] has not been read before new conversion result is loaded to this register, OV is set to 1 and previous conversion result is gone
 * |        |          |It is cleared by hardware after DAT register is read.
 * |[17]    |VALID     |Valid Flag (Read Only)
 * |        |          |0 = Data in RESULT[11:0] bits is not valid.
 * |        |          |1 = Data in RESULT[11:0] bits is valid.
 * |        |          |Note: This bit is set to 1 when corresponding channel analog input conversion is completed and cleared by hardware after DAT register is read.
 * @var SARADC_T::STATUS
 * Offset: 0x40  SAR ADC Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADEF      |A/D Conversion End Flag
 * |        |          |A status flag that indicates the end of A/D conversion.
 * |        |          |ADEF is set to 1 at these two conditions:
 * |        |          |1. When A/D conversion ends in Single mode.
 * |        |          |2. When A/D conversion ends on all specified channels in Scan mode.
 * |        |          |Note: This bit can be cleared by writing u20181u2019 to it.
 * |[1]     |ADCMPF0   |Compare Flag
 * |        |          |When the selected channel A/D conversion result meets setting condition in SARADC_CMP0 then this bit is set to 1
 * |        |          |And it is cleared by writing 1 to self.
 * |        |          |0 = Conversion result in DAT register does not meet CMP0 register.
 * |        |          |1 = Conversion result in DAT register meets CMP0 register.
 * |[2]     |ADCMPF1   |Compare Flag
 * |        |          |When the selected channel A/D conversion result meets setting condition in SARADC_CMP1 then this bit is set to 1
 * |        |          |And it is cleared by writing 1 to self.
 * |        |          |0 = Conversion result in DAT register does not meet CMP1 register.
 * |        |          |1 = Conversion result in DAT register meets CMP1 register.
 * |[3]     |BUSY      |BUSY/IDLE (Read Only)
 * |        |          |0 = A/D converter is in idle state.
 * |        |          |1 = A/D converter is busy at conversion.
 * |        |          |This bit is mirror of as SWTRG bit in CTL.
 * |[7:4]   |CHANNEL   |Current Conversion Channel (Read Only)
 * |        |          |This field reflects the current conversion channel when BUSY = 1
 * |        |          |When BUSY = 0, it shows the number of the next converted channel.
 * |[23:8]  |VALID     |Data Valid Flag (Read Only)
 * |        |          |It is a mirror of VALID bit in DATx.
 * @var SARADC_T::PDMADAT
 * Offset: 0x50  SAR ADC PDMA Current Transfer Data
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[17:0]  |DATA      |SAR ADC PDMA Current Transfer Data Register (Read Only)
 * |        |          |When PDMA transferring, read this register can monitor current PDMA transfer data.
 * |        |          |Current PDMA transfer data is the content of DAT0 ~ DAT11.
 * |        |          |If PDMA word length = 32, data including Reserved bits, OV bit and VALID bit is moved
 * |        |          |If PDMA word length = 16, only AD conversion result is moved.
 * @var SARADC_T::ACTL
 * Offset: 0x5C  SAR ADC Analog Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SAR_SE_MODE|SE mode selection
 * |        |          |Have to be 1 0 = SARADC in differential mode
 * |        |          |1 = SARADC in single ended mode
 * |[18]    |SAR_VREF  |VREF selection
 * |        |          |0 -- select VCCA as VREF
 * |        |          |1 -- select MICBIAS as VREF
 * @var SARADC_T::CTL
 * Offset: 0x60  SAR ADC Control  Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADCEN     |A/D Converter Enable Bit
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |        |          |Before starting A/D conversion function, this bit should be set to 1
 * |        |          |Clear it to 0 to disable A/D converter analog circuit for saving power consumption.
 * |[1]     |ADCIE     |A/D Interrupt Enable Bit
 * |        |          |0 = A/D interrupt function Disabled.
 * |        |          |1 = A/D interrupt function Enabled.
 * |        |          |A/D conversion end interrupt request is generated if ADCIE bit is set to 1.
 * |[3:2]   |OPMODE    |A/D Converter Operation Mode
 * |        |          |00 = Single conversion.
 * |        |          |01 = Reserved.
 * |        |          |10 = Single-cycle scan.
 * |        |          |11 = Continuous scan.
 * |        |          |When changing the operation mode, software should disable SWTRG bit firstly.
 * |[5:4]   |HWTRGSEL  |Hardware Trigger Source Selection
 * |        |          |00 = A/D conversion is started by external STADC pin.
 * |        |          |Others = Reserved.
 * |        |          |Software should disable TRGEN and SWTRG before change HWTRGSEL.
 * |[7:6]   |HWTRGCOND |External Trigger Condition
 * |        |          |These two bits decide external pin STADC trigger event is level or edge
 * |        |          |The signal must be kept at stable state at least 8 PCLKs for level trigger and 4 PCLKs at high and low state for edge trigger.
 * |        |          |00 = Low level.
 * |        |          |01 = High level.
 * |        |          |10 = Falling edge.
 * |        |          |11 = Rising edge.
 * |[8]     |HWTRGEN   |Hardware Trigger Enable Bit
 * |        |          |Enable or disable triggering of A/D conversion by hardware (external STADC pin or PWM Center-aligned trigger).
 * |        |          |0 = Disabled.
 * |        |          |1 = Enabled.
 * |        |          |SARADC hardware trigger function is only supported in single-cycle scan mode.
 * |        |          |If hardware trigger mode, the SWTRG bit can be set to 1 by the selected hardware trigger source.
 * |[9]     |PDMAEN    |PDMA Transfer Enable Bit
 * |        |          |0 = PDMA data transfer Disabled.
 * |        |          |1 = PDMA data transfer in DAT 0~11 Enabled.
 * |        |          |When A/D conversion is completed, the converted data is loaded into DAT 0~11, software can enable this bit to generate a PDMA data transfer request.
 * |        |          |When PDMA=1, software must set ADCIE=0 to disable interrupt.
 * |[11]    |SWTRG     |A/D Conversion Start
 * |        |          |0 = Conversion stops and A/D converter enter idle state.
 * |        |          |1 = Conversion starts.
 * |        |          |SWTRG bit can be set to 1 from three sources: software, external pin STADC
 * |        |          |SWTRG will be cleared to 0 by hardware automatically at the ends of single mode and single-cycle scan mode
 * |        |          |In continuous scan mode, A/D conversion is continuously performed until software writes 0 to this bit or chip reset.
 * @var SARADC_T::CHEN
 * Offset: 0x64  SAR ADC Channel Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CHEN      |Analog Input Channel Enable Bit
 * |        |          |Set CHEN[11:0] to enable the corresponding analog input channel 11 ~ 0.
 * |        |          |0 = SARADC input channel Disabled.
 * |        |          |1 = SARADC input channel Enabled.
 * |        |          |Note: Keep 0 for [15:12]
 * @var SARADC_T::CMP
 * Offset: 0x68  SAR ADC Compare Register 0
 * Offset: 0x6C  SAR ADC Compare Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ADCMPEN   |Compare Enable Bit
 * |        |          |0 = Compare function Disabled.
 * |        |          |1 = Compare function Enabled.
 * |        |          |Note: Set this bit to 1 to enable SARADC controller to compare CMPDAT[11:0] with specified channel conversion result when converted data is loaded into DAT register.
 * |[1]     |ADCMPIE   |Compare Interrupt Enable Bit
 * |        |          |0 = Compare function interrupt Disabled.
 * |        |          |1 = Compare function interrupt Enabled.
 * |        |          |Note: If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMCNT, ADCMPF bit will be asserted, in the meanwhile, if ADCMPIE is set to 1, a compare interrupt request is generated.
 * |[2]     |CMPCOND   |Compare Condition
 * |        |          |0 = Set the compare condition as that when a 12-bit A/D conversion result is less than the 12-bit CMPDAT (CMPx[27:16]), the internal match counter will increase one.
 * |        |          |1 = Set the compare condition as that when a 12-bit A/D conversion result is greater or equal to the 12-bit CMPD (CMPx[27:16]), the internal match counter will increase one.
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
 * |        |          |1100 = Reserved.
 * |        |          |1101 = Reserved.
 * |        |          |1110 = Reserved.
 * |        |          |1111 = Reserved
 * |[11:8]  |CMPMCNT   |Compare Match Count
 * |        |          |When the specified A/D channel analog conversion result matches the compare condition defined by CMPCOND[2], the internal match counter will increase 1
 * |        |          |When the internal counter reaches the value to (CMPMCNT +1), the ADCMPFx bit will be set.
 * |[27:16] |CMPDAT    |Comparison Data
 * |        |          |The 12-bit data is used to compare with conversion result of specified channel.
 * |        |          |When ADCFMbit is set to 0, SARADC comparator compares CMPDAT with conversion result with unsigned format
 * |        |          |CMPDAT should be filled in unsigned format.
 * |        |          |When ADCFMbit is set to 1, SARADC comparator compares CMPDAT with conversion result with 2u2019complement format
 * |        |          |CMPDAT should be filled in 2u2019complement format.
 */
    __I  uint32_t DAT[12];				 /*!< [0x0000] SAR ADC Data Register 0~11                                       */
    __I  uint32_t RESERVE0[4];
    __IO uint32_t STATUS;                /*!< [0x0040] SAR ADC Status Register                                          */
    __I  uint32_t RESERVE1[3];
    __I  uint32_t PDMADAT;               /*!< [0x0050] SAR ADC PDMA Current Transfer Data                               */
    __I  uint32_t RESERVE2[2];
    __IO uint32_t ACTL;                  /*!< [0x005c] SAR ADC Analog Control Register                                  */
    __IO uint32_t CTL;                   /*!< [0x0060] SAR ADC Control  Register                                        */
    __IO uint32_t CHEN;                  /*!< [0x0064] SAR ADC Channel Enable Register                                  */
    __IO uint32_t CMP[2];                /*!< [0x0068] SAR ADC Compare Register 0~1                                     */

} SARADC_T;

/**
    @addtogroup SARADC_CONST SARADC Bit Field Definition
    Constant Definitions for SARADC Controller
@{ */

#define SARADC_DAT_RESULT_Pos            (0)                                               /*!< SARADC_T::DAT: RESULT Position         */
#define SARADC_DAT_RESULT_Msk            (0xffful << SARADC_DAT_RESULT_Pos)                /*!< SARADC_T::DAT: RESULT Mask             */

#define SARADC_DAT_OV_Pos                (16)                                              /*!< SARADC_T::DAT: OV Position             */
#define SARADC_DAT_OV_Msk                (0x1ul << SARADC_DAT_OV_Pos)                      /*!< SARADC_T::DAT: OV Mask                 */

#define SARADC_DAT_VALID_Pos             (17)                                              /*!< SARADC_T::DAT: VALID Position          */
#define SARADC_DAT_VALID_Msk             (0x1ul << SARADC_DAT_VALID_Pos)                   /*!< SARADC_T::DAT: VALID Mask              */

#define SARADC_STATUS_ADEF_Pos           (0)                                               /*!< SARADC_T::STATUS: ADEF Position        */
#define SARADC_STATUS_ADEF_Msk           (0x1ul << SARADC_STATUS_ADEF_Pos)                 /*!< SARADC_T::STATUS: ADEF Mask            */

#define SARADC_STATUS_ADCMPF0_Pos        (1)                                               /*!< SARADC_T::STATUS: ADCMPF0 Position     */
#define SARADC_STATUS_ADCMPF0_Msk        (0x1ul << SARADC_STATUS_ADCMPF0_Pos)              /*!< SARADC_T::STATUS: ADCMPF0 Mask         */

#define SARADC_STATUS_ADCMPF1_Pos        (2)                                               /*!< SARADC_T::STATUS: ADCMPF1 Position     */
#define SARADC_STATUS_ADCMPF1_Msk        (0x1ul << SARADC_STATUS_ADCMPF1_Pos)              /*!< SARADC_T::STATUS: ADCMPF1 Mask         */

#define SARADC_STATUS_BUSY_Pos           (3)                                               /*!< SARADC_T::STATUS: BUSY Position        */
#define SARADC_STATUS_BUSY_Msk           (0x1ul << SARADC_STATUS_BUSY_Pos)                 /*!< SARADC_T::STATUS: BUSY Mask            */

#define SARADC_STATUS_CHANNEL_Pos        (4)                                               /*!< SARADC_T::STATUS: CHANNEL Position     */
#define SARADC_STATUS_CHANNEL_Msk        (0xful << SARADC_STATUS_CHANNEL_Pos)              /*!< SARADC_T::STATUS: CHANNEL Mask         */

#define SARADC_STATUS_VALID_Pos          (8)                                               /*!< SARADC_T::STATUS: VALID Position       */
#define SARADC_STATUS_VALID_Msk          (0xfffful << SARADC_STATUS_VALID_Pos)             /*!< SARADC_T::STATUS: VALID Mask           */

#define SARADC_PDMADAT_DATA_Pos          (0)                                               /*!< SARADC_T::PDMADAT: DATA Position       */
#define SARADC_PDMADAT_DATA_Msk          (0x3fffful << SARADC_PDMADAT_DATA_Pos)            /*!< SARADC_T::PDMADAT: DATA Mask           */

#define SARADC_ACTL_SAR_SE_MODE_Pos      (0)                                               /*!< SARADC_T::ACTL: SAR_SE_MODE Position   */
#define SARADC_ACTL_SAR_SE_MODE_Msk      (0x1ul << SARADC_ACTL_SAR_SE_MODE_Pos)            /*!< SARADC_T::ACTL: SAR_SE_MODE Mask       */

#define SARADC_ACTL_SAR_VREF_Pos         (18)                                              /*!< SARADC_T::ACTL: SAR_VREF Position      */
#define SARADC_ACTL_SAR_VREF_Msk         (0x1ul << SARADC_ACTL_SAR_VREF_Pos)               /*!< SARADC_T::ACTL: SAR_VREF Mask          */

#define SARADC_CTL_ADCEN_Pos             (0)                                               /*!< SARADC_T::CTL: ADCEN Position          */
#define SARADC_CTL_ADCEN_Msk             (0x1ul << SARADC_CTL_ADCEN_Pos)                   /*!< SARADC_T::CTL: ADCEN Mask              */

#define SARADC_CTL_ADCIE_Pos             (1)                                               /*!< SARADC_T::CTL: ADCIE Position          */
#define SARADC_CTL_ADCIE_Msk             (0x1ul << SARADC_CTL_ADCIE_Pos)                   /*!< SARADC_T::CTL: ADCIE Mask              */

#define SARADC_CTL_OPMODE_Pos            (2)                                               /*!< SARADC_T::CTL: OPMODE Position         */
#define SARADC_CTL_OPMODE_Msk            (0x3ul << SARADC_CTL_OPMODE_Pos)                  /*!< SARADC_T::CTL: OPMODE Mask             */

#define SARADC_CTL_HWTRGSEL_Pos          (4)                                               /*!< SARADC_T::CTL: HWTRGSEL Position       */
#define SARADC_CTL_HWTRGSEL_Msk          (0x3ul << SARADC_CTL_HWTRGSEL_Pos)                /*!< SARADC_T::CTL: HWTRGSEL Mask           */

#define SARADC_CTL_HWTRGCOND_Pos         (6)                                               /*!< SARADC_T::CTL: HWTRGCOND Position      */
#define SARADC_CTL_HWTRGCOND_Msk         (0x3ul << SARADC_CTL_HWTRGCOND_Pos)               /*!< SARADC_T::CTL: HWTRGCOND Mask          */

#define SARADC_CTL_HWTRGEN_Pos           (8)                                               /*!< SARADC_T::CTL: HWTRGEN Position        */
#define SARADC_CTL_HWTRGEN_Msk           (0x1ul << SARADC_CTL_HWTRGEN_Pos)                 /*!< SARADC_T::CTL: HWTRGEN Mask            */

#define SARADC_CTL_PDMAEN_Pos            (9)                                               /*!< SARADC_T::CTL: PDMAEN Position         */
#define SARADC_CTL_PDMAEN_Msk            (0x1ul << SARADC_CTL_PDMAEN_Pos)                  /*!< SARADC_T::CTL: PDMAEN Mask             */

#define SARADC_CTL_SWTRG_Pos             (11)                                              /*!< SARADC_T::CTL: SWTRG Position          */
#define SARADC_CTL_SWTRG_Msk             (0x1ul << SARADC_CTL_SWTRG_Pos)                   /*!< SARADC_T::CTL: SWTRG Mask              */

#define SARADC_CHEN_CHEN_Pos             (0)                                               /*!< SARADC_T::CHEN: CHEN Position          */
#define SARADC_CHEN_CHEN_Msk             (0xfffful << SARADC_CHEN_CHEN_Pos)                /*!< SARADC_T::CHEN: CHEN Mask              */

#define SARADC_CMP_ADCMPEN_Pos           (0)                                               /*!< SARADC_T::CMP: ADCMPEN Position        */
#define SARADC_CMP_ADCMPEN_Msk           (0x1ul << SARADC_CMP_ADCMPEN_Pos)                 /*!< SARADC_T::CMP: ADCMPEN Mask            */

#define SARADC_CMP_ADCMPIE_Pos           (1)                                               /*!< SARADC_T::CMP: ADCMPIE Position        */
#define SARADC_CMP_ADCMPIE_Msk           (0x1ul << SARADC_CMP_ADCMPIE_Pos)                 /*!< SARADC_T::CMP: ADCMPIE Mask            */

#define SARADC_CMP_CMPCOND_Pos           (2)                                               /*!< SARADC_T::CMP: CMPCOND Position        */
#define SARADC_CMP_CMPCOND_Msk           (0x1ul << SARADC_CMP_CMPCOND_Pos)                 /*!< SARADC_T::CMP: CMPCOND Mask            */

#define SARADC_CMP_CMPCH_Pos             (3)                                               /*!< SARADC_T::CMP: CMPCH Position          */
#define SARADC_CMP_CMPCH_Msk             (0xful << SARADC_CMP_CMPCH_Pos)                   /*!< SARADC_T::CMP: CMPCH Mask              */

#define SARADC_CMP_CMPMCNT_Pos           (8)                                               /*!< SARADC_T::CMP: CMPMCNT Position        */
#define SARADC_CMP_CMPMCNT_Msk           (0xful << SARADC_CMP_CMPMCNT_Pos)                 /*!< SARADC_T::CMP: CMPMCNT Mask            */

#define SARADC_CMP_CMPDAT_Pos            (16)                                              /*!< SARADC_T::CMP: CMPDAT Position         */
#define SARADC_CMP_CMPDAT_Msk            (0xffful << SARADC_CMP_CMPDAT_Pos)                /*!< SARADC_T::CMP: CMPDAT Mask             */

/**@}*/ /* SARADC_CONST */
/**@}*/ /* end of SARADC register group */


/*---------------------- Sigma- Delta Analog-to-Digital Converter -------------------------*/
/**
    @addtogroup SDADC Sigma- Delta Analog-to-Digital Converter(SDADC)
    Memory Mapped Structure for SDADC Controller
@{ */
 
typedef struct
{


/**
 * @var SDADC_T::DAT
 * Offset: 0x00  SD ADC FIFO Data Read Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |RESULT    |Delta-Sigma ADC DATA FIFO Read
 * |        |          |A read of this register will read data from the audio FIFO and increment the read pointer
 * |        |          |A read past empty will repeat the last data
 * |        |          |Can be used with SDADC_FIFOSTS.THIF to determine if valid data is present in FIFO.
 * @var SDADC_T::EN
 * Offset: 0x04  SD ADC Enable  Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SDADCEN   |SDADC   Enable
 * |        |          |1 = ADC   Conversion enabled.
 * |        |          |0 = Conversion   stopped and ADC is reset including FIFO pointers.
 * |[1]     |DINEDGE   | ADC data input clock edge selection
 * |        |          | 1 = ADC clock positive edge latch
 * |        |          |0 = ADC clock negetive edge latch
 * |[2]     |DINBYPS   |ADC data input bypass (internal debug)
 * |        |          |1 = analog 5bits to FIFO for testing
 * |        |          |0 = normal mode
 * @var SDADC_T::CLKDIV
 * Offset: 0x08  SD ADC Clock Divider Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |CLKDIV    |SD_CLK Clock Divider
 * |        |          |SDADC internal clock divider
 * |        |          |CLKDIV should be set to give a SD_CLK frequency in the range of 1.024-6.144MHz
 * |        |          |(Refer to 7.1.4.2.)
 * |        |          |CLKDIV must be greater than and equal 2.
 * |        |          |CLKDIV = SD_CLK/Sample Rate/Down Sample Rate
 * @var SDADC_T::CTL
 * Offset: 0x0C  SD ADC Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |DSRATE    |Down Sampling Ratio
 * |        |          |0 = reserved
 * |        |          |1 = down sample X 16
 * |        |          |2 = down sample X 32
 * |        |          |3 = down sample X 64
 * |[3:2]   |FIFOBITS  |FIFO Data Bits Selection
 * |        |          |0 = 32 bits
 * |        |          |1 = 16 bits
 * |        |          |2 = 8 bits
 * |        |          |3 = 24 bits
 * |[6:4]   |FIFOTH    |FIFO Threshold:
 * |        |          |Determines at what level the ADC FIFO will generate a interrupt.
 * |        |          |Interrupt will be generated when number of words present in ADC FIFO is >
 * |        |          |FIFOTH.
 * |[7]     |FIFOTHIE  |FIFO Threshold Interrupt Enable
 * |        |          |0 = disable interrupt whenever FIFO level exceeds that set in FIFOTH.
 * |        |          |1 = enable interrupt whenever FIFO level exceeds that set in FIFOTH. 
 * |[8]     |DMICEN    |Digital MIC Enable
 * |        |          |1 = turn digital MIC function input from GPIO.
 * |        |          |0 = keep SDADC function.
 * |[11]    |RATESEL   |Sample Rate Selection
 * |        |          |1 = choose BSRATE for BS
 * |        |          |0 = choose DSRATE for SDADC 
 * @var SDADC_T::FIFOSTS
 * Offset: 0x10  SD ADC FIFO Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |FULL      |FIFO Full
 * |        |          |1 = FIFO is full.
 * |        |          |0 = FIFO is not full.
 * |[1]     |EMPTY     |FIFO Empty
 * |        |          |1= FIFO is empty.
 * |        |          |0= FIFO is not empty.
 * |[2]     |THIF      |ADC FIFO Threshold Interrupt Status (Read Only)
 * |        |          |1 = The valid data count within the SDADC FIFO buffer is larger than or equal the setting value of FIFOTH.
 * |        |          |0 = The valid data count within the transmit FIFO buffer is less than to the setting value of FIFOTH. 
 * |[7:4]   |POINTER   |SDADC FIFO Pointer (Read Only)
 * |        |          |The FULL bit and POINTER[3:0] indicates the field that the valid data count within the SDADC FIFO buffer.
 * |        |          |The Maximum value shown in POINTER is 15
 * |        |          |When the using level of SDADC FIFO Buffer equal to 16, The FULL bit is set to 1.
 * |[31]    |BISTEN    |BIST Enable
 * |        |          |1 = Enable SDADC FIFO BIST testing SDADC FIFO can be testing by Cortex-M0
 * |        |          |0 = Disable SDADC FIFO BIST testing
 * |        |          |Internal use
 * @var SDADC_T::PDMACTL
 * Offset: 0x14  SD ADC PDMA Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PDMAEN    |Enable SDADC PDMA Receive Channel
 * |        |          |1 = Enable SDADC PDMA.
 * |        |          |0 = Disable SDADC PDMA.
 * @var SDADC_T::CMPR
 * Offset: 0x18  SD ADC Comparator Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |CMPIE     |Compare Interrupt Enable
 * |        |          |1 = Enable compare function interrupt.
 * |        |          |0 = Disable compare function interrupt.
 * |        |          |If the compare function is enabled and the compare condition matches the setting of CMPCOND and CMPMATCNT, CMPF bit will be asserted, if CMPIE is set to 1, a compare interrupt request is generated.
 * |[2]     |CMPCOND   |Compare Condition
 * |        |          |1= Set the compare condition that result is greater or equal to CMPD
 * |        |          |0= Set the compare condition that result is less than CMPD
 * |        |          |Note: When the internal counter reaches the value (CMPMATCNT +1), the CMPF bit will be set.
 * |[3]     |CMPF      |Compare Flag
 * |        |          |When the conversion result meets condition in CMPCOND and CMPMATCNT this bit is set to 1
 * |        |          |It is cleared by writing 1 to self.
 * |[7:4]   |CMPMATCNT |Compare Match Count
 * |        |          |When the A/D FIFO result matches the compare condition defined by CMPCOND, the internal match counter will increase by 1
 * |        |          |When the internal counter reaches the value to (CMPMATCNT +1), the CMPF bit will be set.
 * |[30:8]  |CMPD      |Comparison Data
 * |        |          |23 bit value to compare to FIFO output word.
 * |[31]    |CMPOEN    |Compare Match output FIFO zero
 * |        |          |1 = compare match then FIFO out zero
 * |        |          |0 = FIFO data keep original one.
 * @var SDADC_T::SDCHOP
 * Offset: 0x20  Sigma Delta Analog Block Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |PD        |SDADC Power Down
 * |        |          |0 = SDADC power on
 * |        |          |1 = SDADC power off
 * |[2:1]   |BIAS      |SDADC Bias Current Selection
 * |        |          |00 = 1
 * |        |          |01 = 0.75
 * |        |          |10 = 0.5
 * |        |          |11 = 1.25
 * |[4]     |PGA_PU    |Power up PGA
 * |        |          |0 = disable
 * |        |          |1 = enable
 * |[5]     |PGA_MUTE  |Mute control signal
 * |        |          |0 = disable
 * |        |          |1 = enable
 * |[8:6]   |PGA_MODE  |PGA mode selection
 * |        |          |PGA_MODE[0] = Disable anti-aliasing filter adjust
 * |        |          |PGA_MODE [1] = Short the inputs
 * |        |          |PGA_MODE[2] = Noise reduction enable.
 * |        |          |1 = Enable
 * |        |          |0 = Disable
 * |[11:9]  |PGA_IBCTR |Trim PGA Current
 * |        |          |0=default
 * |[12]    |PGA_IBLOOP|Trim PGA current
 * |        |          |1=default
 * |[13]    |PGA_GAIN  |PGA Gain (default=0)
 * |        |          |PGA_GAIN
 * |        |          |PGA_HZMODE=0
 * |        |          |PGA_HZMODE=1
 * |        |          |0
 * |        |          |0dB
 * |        |          |6dB
 * |        |          |1
 * |        |          |6dB
 * |        |          |12dB
 * |[14]    |PGA_DISCH |Charge inputs selected by PGA_ACDC[1:0] to VREF
 * |        |          |1 = Enable
 * |        |          |0 = Disable
 * |[15]    |PGA_CMLCK |Common mode Threshold lock adjust enable
 * |        |          |0 = Enable
 * |        |          |1 = Disable
 * |[17:16] |PGA_CMLCKADJ|Common mode Threshold lock adjust. Action takes effect when PGA_CMLCK=0
 * |        |          |00---0.98 (default)
 * |        |          |01---0.96
 * |        |          |10---1.01
 * |        |          |11---1.04
 * |        |          |default=00
 * |[18]    |PGA_CLASSA|Enable Class A mode of operation
 * |        |          |0 = Class AB
 * |        |          |1 = Class A (default)
 * |[19]    |PGA_TRIMOBC|Trim current in output driver
 * |        |          |0=disable
 * |        |          |1=enable (default)
 * |[20]    |PGA_HZMODE|Select input impedance
 * |        |          |0 = 12k Ohm input impedance
 * |        |          |1 = 500k Ohm input impedance (default)
 * |[22:21] |PGA_ADCDC |Action takes effect when PGA_DISCH=1
 * |        |          |Bit[21]: ACDC_CTRL[0] charges INP to VREF
 * |        |          |Bit[22]: ACDC_CTRL[1] charges INN to VREF
 * |        |          |00=Default
 * |[31:30] |AUDIOPATHSEL|Audio Path Selection, Connect SDADC input to
 * |        |          |00 = PGA (default)
 * |        |          |01 = MICN and MICP pins (bypass PGA)
 * |        |          |10 = Reserved
 * |        |          |11 = Reserved
 */
    __I  uint32_t DAT;                   /*!< [0x0000] SD ADC FIFO Data Read Register                                   */
    __IO uint32_t EN;                    /*!< [0x0004] SD ADC Enable  Register                                          */
    __IO uint32_t CLKDIV;                /*!< [0x0008] SD ADC Clock Divider Register                                    */
    __IO uint32_t CTL;                   /*!< [0x000c] SD ADC Control Register                                          */
    __IO uint32_t FIFOSTS;               /*!< [0x0010] SD ADC FIFO Status Register                                      */
    __IO uint32_t PDMACTL;               /*!< [0x0014] SD ADC PDMA Control Register                                     */
    __IO uint32_t CMPR[2];               /*!< [0x0018] SD ADC Comparator 0,1 Control Register                           */
    __IO uint32_t SDCHOP;                /*!< [0x0020] Sigma Delta Analog Block Control Register                        */

} SDADC_T;

/**
    @addtogroup SDADC_CONST SDADC Bit Field Definition
    Constant Definitions for SDADC Controller
@{ */

#define SDADC_DAT_RESULT_Pos             (0)                                               /*!< SDADC_T::DAT: RESULT Position          */
#define SDADC_DAT_RESULT_Msk             (0xfffful << SDADC_DAT_RESULT_Pos)                /*!< SDADC_T::DAT: RESULT Mask              */

#define SDADC_EN_SDADCEN_Pos             (0)                                               /*!< SDADC_T::EN: SDADCEN Position          */
#define SDADC_EN_SDADCEN_Msk             (0x1ul << SDADC_EN_SDADCEN_Pos)                   /*!< SDADC_T::EN: SDADCEN Mask              */

#define SDADC_EN_DINEDGE_Pos             (1)                                               /*!< SDADC_T::EN: DINEDGE Position          */
#define SDADC_EN_DINEDGE_Msk             (0x1ul << SDADC_EN_DINEDGE_Pos)                   /*!< SDADC_T::EN: DINEDGE Mask              */

#define SDADC_EN_DINBYPS_Pos             (2)                                               /*!< SDADC_T::EN: DINBYPS Position          */
#define SDADC_EN_DINBYPS_Msk             (0x1ul << SDADC_EN_DINBYPS_Pos)                   /*!< SDADC_T::EN: DINBYPS Mask              */

#define SDADC_CLKDIV_CLKDIV_Pos          (0)                                               /*!< SDADC_T::CLKDIV: CLKDIV Position       */
#define SDADC_CLKDIV_CLKDIV_Msk          (0xfful << SDADC_CLKDIV_CLKDIV_Pos)               /*!< SDADC_T::CLKDIV: CLKDIV Mask           */

#define SDADC_CTL_DSRATE_Pos             (0)                                               /*!< SDADC_T::CTL: DSRATE Position          */
#define SDADC_CTL_DSRATE_Msk             (0x3ul << SDADC_CTL_DSRATE_Pos)                   /*!< SDADC_T::CTL: DSRATE Mask              */

#define SDADC_CTL_FIFOBITS_Pos           (2)                                               /*!< SDADC_T::CTL: FIFOBITS Position        */
#define SDADC_CTL_FIFOBITS_Msk           (0x3ul << SDADC_CTL_FIFOBITS_Pos)                 /*!< SDADC_T::CTL: FIFOBITS Mask            */

#define SDADC_CTL_FIFOTH_Pos             (4)                                               /*!< SDADC_T::CTL: FIFOTH Position          */
#define SDADC_CTL_FIFOTH_Msk             (0x7ul << SDADC_CTL_FIFOTH_Pos)                   /*!< SDADC_T::CTL: FIFOTH Mask              */

#define SDADC_CTL_FIFOTHIE_Pos           (7)                                               /*!< SDADC_T::CTL: FIFOTHIE Position        */
#define SDADC_CTL_FIFOTHIE_Msk           (0x1ul << SDADC_CTL_FIFOTHIE_Pos)                 /*!< SDADC_T::CTL: FIFOTHIE Mask            */

#define SDADC_CTL_DMICEN_Pos             (8)                                               /*!< SDADC_T::CTL: DMICEN Position          */
#define SDADC_CTL_DMICEN_Msk             (0x1ul << SDADC_CTL_DMICEN_Pos)                   /*!< SDADC_T::CTL: DMICEN Mask              */

#define SDADC_CTL_RATESEL_Pos            (11)                                              /*!< SDADC_T::CTL: RATESEL Position         */
#define SDADC_CTL_RATESEL_Msk            (0x1ul << SDADC_CTL_RATESEL_Pos)                  /*!< SDADC_T::CTL: RATESEL Mask             */

#define SDADC_FIFOSTS_FULL_Pos           (0)                                               /*!< SDADC_T::FIFOSTS: FULL Position        */
#define SDADC_FIFOSTS_FULL_Msk           (0x1ul << SDADC_FIFOSTS_FULL_Pos)                 /*!< SDADC_T::FIFOSTS: FULL Mask            */

#define SDADC_FIFOSTS_EMPTY_Pos          (1)                                               /*!< SDADC_T::FIFOSTS: EMPTY Position       */
#define SDADC_FIFOSTS_EMPTY_Msk          (0x1ul << SDADC_FIFOSTS_EMPTY_Pos)                /*!< SDADC_T::FIFOSTS: EMPTY Mask           */

#define SDADC_FIFOSTS_THIF_Pos           (2)                                               /*!< SDADC_T::FIFOSTS: THIF Position        */
#define SDADC_FIFOSTS_THIF_Msk           (0x1ul << SDADC_FIFOSTS_THIF_Pos)                 /*!< SDADC_T::FIFOSTS: THIF Mask            */

#define SDADC_FIFOSTS_POINTER_Pos        (4)                                               /*!< SDADC_T::FIFOSTS: POINTER Position     */
#define SDADC_FIFOSTS_POINTER_Msk        (0xful << SDADC_FIFOSTS_POINTER_Pos)              /*!< SDADC_T::FIFOSTS: POINTER Mask         */

#define SDADC_FIFOSTS_BISTEN_Pos         (31)                                              /*!< SDADC_T::FIFOSTS: BISTEN Position      */
#define SDADC_FIFOSTS_BISTEN_Msk         (0x1ul << SDADC_FIFOSTS_BISTEN_Pos)               /*!< SDADC_T::FIFOSTS: BISTEN Mask          */

#define SDADC_PDMACTL_PDMAEN_Pos         (0)                                               /*!< SDADC_T::PDMACTL: PDMAEN Position      */
#define SDADC_PDMACTL_PDMAEN_Msk         (0x1ul << SDADC_PDMACTL_PDMAEN_Pos)               /*!< SDADC_T::PDMACTL: PDMAEN Mask          */

#define SDADC_CMPR_CMPIE_Pos             (1)                                               /*!< SDADC_T::CMPR: CMPIE Position          */
#define SDADC_CMPR_CMPIE_Msk             (0x1ul << SDADC_CMPR_CMPIE_Pos)                   /*!< SDADC_T::CMPR: CMPIE Mask              */

#define SDADC_CMPR_CMPCOND_Pos           (2)                                               /*!< SDADC_T::CMPR: CMPCOND Position        */
#define SDADC_CMPR_CMPCOND_Msk           (0x1ul << SDADC_CMPR_CMPCOND_Pos)                 /*!< SDADC_T::CMPR: CMPCOND Mask            */

#define SDADC_CMPR_CMPF_Pos              (3)                                               /*!< SDADC_T::CMPR: CMPF Position           */
#define SDADC_CMPR_CMPF_Msk              (0x1ul << SDADC_CMPR_CMPF_Pos)                    /*!< SDADC_T::CMPR: CMPF Mask               */

#define SDADC_CMPR_CMPMATCNT_Pos         (4)                                               /*!< SDADC_T::CMPR: CMPMATCNT Position      */
#define SDADC_CMPR_CMPMATCNT_Msk         (0xful << SDADC_CMPR_CMPMATCNT_Pos)               /*!< SDADC_T::CMPR: CMPMATCNT Mask          */

#define SDADC_CMPR_CMPD_Pos              (8)                                               /*!< SDADC_T::CMPR: CMPD Position           */
#define SDADC_CMPR_CMPD_Msk              (0x7ffffful << SDADC_CMPR_CMPD_Pos)               /*!< SDADC_T::CMPR: CMPD Mask               */

#define SDADC_CMPR_CMPOEN_Pos            (31)                                              /*!< SDADC_T::CMPR: CMPOEN Position         */
#define SDADC_CMPR_CMPOEN_Msk            (0x1ul << SDADC_CMPR_CMPOEN_Pos)                  /*!< SDADC_T::CMPR: CMPOEN Mask             */

#define SDADC_SDCHOP_PD_Pos              (0)                                               /*!< SDADC_T::SDCHOP: PD Position           */
#define SDADC_SDCHOP_PD_Msk              (0x1ul << SDADC_SDCHOP_PD_Pos)                    /*!< SDADC_T::SDCHOP: PD Mask               */

#define SDADC_SDCHOP_BIAS_Pos            (1)                                               /*!< SDADC_T::SDCHOP: BIAS Position         */
#define SDADC_SDCHOP_BIAS_Msk            (0x3ul << SDADC_SDCHOP_BIAS_Pos)                  /*!< SDADC_T::SDCHOP: BIAS Mask             */

#define SDADC_SDCHOP_PGA_PU_Pos          (4)                                               /*!< SDADC_T::SDCHOP: PGA_PU Position       */
#define SDADC_SDCHOP_PGA_PU_Msk          (0x1ul << SDADC_SDCHOP_PGA_PU_Pos)                /*!< SDADC_T::SDCHOP: PGA_PU Mask           */

#define SDADC_SDCHOP_PGA_MUTE_Pos        (5)                                               /*!< SDADC_T::SDCHOP: PGA_MUTE Position     */
#define SDADC_SDCHOP_PGA_MUTE_Msk        (0x1ul << SDADC_SDCHOP_PGA_MUTE_Pos)              /*!< SDADC_T::SDCHOP: PGA_MUTE Mask         */

#define SDADC_SDCHOP_PGA_MODE_Pos        (6)                                               /*!< SDADC_T::SDCHOP: PGA_MODE Position     */
#define SDADC_SDCHOP_PGA_MODE_Msk        (0x7ul << SDADC_SDCHOP_PGA_MODE_Pos)              /*!< SDADC_T::SDCHOP: PGA_MODE Mask         */

#define SDADC_SDCHOP_PGA_IBCTR_Pos       (9)                                               /*!< SDADC_T::SDCHOP: PGA_IBCTR Position    */
#define SDADC_SDCHOP_PGA_IBCTR_Msk       (0x7ul << SDADC_SDCHOP_PGA_IBCTR_Pos)             /*!< SDADC_T::SDCHOP: PGA_IBCTR Mask        */

#define SDADC_SDCHOP_PGA_IBLOOP_Pos      (12)                                              /*!< SDADC_T::SDCHOP: PGA_IBLOOP Position   */
#define SDADC_SDCHOP_PGA_IBLOOP_Msk      (0x1ul << SDADC_SDCHOP_PGA_IBLOOP_Pos)            /*!< SDADC_T::SDCHOP: PGA_IBLOOP Mask       */

#define SDADC_SDCHOP_PGA_GAIN_Pos        (13)                                              /*!< SDADC_T::SDCHOP: PGA_GAIN Position     */
#define SDADC_SDCHOP_PGA_GAIN_Msk        (0x1ul << SDADC_SDCHOP_PGA_GAIN_Pos)              /*!< SDADC_T::SDCHOP: PGA_GAIN Mask         */

#define SDADC_SDCHOP_PGA_DISCH_Pos       (14)                                              /*!< SDADC_T::SDCHOP: PGA_DISCH Position    */
#define SDADC_SDCHOP_PGA_DISCH_Msk       (0x1ul << SDADC_SDCHOP_PGA_DISCH_Pos)             /*!< SDADC_T::SDCHOP: PGA_DISCH Mask        */

#define SDADC_SDCHOP_PGA_CMLCK_Pos       (15)                                              /*!< SDADC_T::SDCHOP: PGA_CMLCK Position    */
#define SDADC_SDCHOP_PGA_CMLCK_Msk       (0x1ul << SDADC_SDCHOP_PGA_CMLCK_Pos)             /*!< SDADC_T::SDCHOP: PGA_CMLCK Mask        */

#define SDADC_SDCHOP_PGA_CMLCKADJ_Pos    (16)                                              /*!< SDADC_T::SDCHOP: PGA_CMLCKADJ Position */
#define SDADC_SDCHOP_PGA_CMLCKADJ_Msk    (0x3ul << SDADC_SDCHOP_PGA_CMLCKADJ_Pos)          /*!< SDADC_T::SDCHOP: PGA_CMLCKADJ Mask     */

#define SDADC_SDCHOP_PGA_CLASSA_Pos      (18)                                              /*!< SDADC_T::SDCHOP: PGA_CLASSA Position   */
#define SDADC_SDCHOP_PGA_CLASSA_Msk      (0x1ul << SDADC_SDCHOP_PGA_CLASSA_Pos)            /*!< SDADC_T::SDCHOP: PGA_CLASSA Mask       */

#define SDADC_SDCHOP_PGA_TRIMOBC_Pos     (19)                                              /*!< SDADC_T::SDCHOP: PGA_TRIMOBC Position  */
#define SDADC_SDCHOP_PGA_TRIMOBC_Msk     (0x1ul << SDADC_SDCHOP_PGA_TRIMOBC_Pos)           /*!< SDADC_T::SDCHOP: PGA_TRIMOBC Mask      */

#define SDADC_SDCHOP_PGA_HZMODE_Pos      (20)                                              /*!< SDADC_T::SDCHOP: PGA_HZMODE Position   */
#define SDADC_SDCHOP_PGA_HZMODE_Msk      (0x1ul << SDADC_SDCHOP_PGA_HZMODE_Pos)            /*!< SDADC_T::SDCHOP: PGA_HZMODE Mask       */

#define SDADC_SDCHOP_PGA_ADCDC_Pos       (21)                                              /*!< SDADC_T::SDCHOP: PGA_ADCDC Position    */
#define SDADC_SDCHOP_PGA_ADCDC_Msk       (0x3ul << SDADC_SDCHOP_PGA_ADCDC_Pos)             /*!< SDADC_T::SDCHOP: PGA_ADCDC Mask        */

#define SDADC_SDCHOP_AUDIOPATHSEL_Pos    (30)                                              /*!< SDADC_T::SDCHOP: AUDIOPATHSEL Position */
#define SDADC_SDCHOP_AUDIOPATHSEL_Msk    (0x3ul << SDADC_SDCHOP_AUDIOPATHSEL_Pos)          /*!< SDADC_T::SDCHOP: AUDIOPATHSEL Mask     */

/**@}*/ /* SDADC_CONST */
/**@}*/ /* end of SDADC register group */


/*---------------------- Serial Peripheral Interface Controller -------------------------*/
/**
    @addtogroup SPI0 Serial Peripheral Interface Controller(SPI0)
    Memory Mapped Structure for SPI0 Controller
@{ */
 
typedef struct
{


/**
 * @var SPI0_T::CTL
 * Offset: 0x00  Control and Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SPIEN     |SPI Transfer Enable
 * |        |          |0 = Disable SPI Transfer.
 * |        |          |1 = Enable SPI Transfer.
 * |        |          |In Master mode, the transfer will start when there is data in the FIFO buffer after this is set to 1
 * |        |          |In Slave mode, the device is ready to receive data when this bit is set to 1.
 * |        |          |Note:
 * |        |          |All configuration should be set before writing 1 to this SPIEN bit
 * |        |          |(e.g.: TXNEG, RXNEG, DWIDTH, LSB, CLKP, and so on).
 * |[1]     |RXNEG     |Receive at Negative Edge
 * |        |          |0 = The received data input signal is latched at the rising edge of SCLK.
 * |        |          |1 = The received data input signal is latched at the falling edge of SCLK.
 * |[2]     |TXNEG     |Transmit at Negative Edge
 * |        |          |0 = The transmitted data output signal is changed at the rising edge of SCLK.
 * |        |          |1 = The transmitted data output signal is changed at the falling edge of SCLK.
 * |[3]     |CLKPOL    |Clock Polarity
 * |        |          |0 = SCLK idle low.
 * |        |          |1 = SCLK idle high.
 * |[7:4]   |SUSPITV   |Suspend Interval (Master Only)
 * |        |          |The four bits provide configurable suspend interval between two successive transmit/receive transactions in a transfer
 * |        |          |The definition of the suspend interval is the interval between the last clock edge of the preceding transaction word and the first clock edge of the following transaction word
 * |        |          |The default value is 0x3
 * |        |          |The period of the suspend interval is obtained according to the following equation
 * |        |          |SUSPITV is available for standard SPI transactions, it must be set to 0 for DUAL and QUAD mode transactions.
 * |        |          |(SUSPITV[3:0] + 0.5) * period of SPICLK clock cycle
 * |        |          |Example:
 * |        |          |SUSPITV = 0x0 ... 0.5 SPICLK clock cycle.
 * |        |          |SUSPITV = 0x1 ... 1.5 SPICLK clock cycle.
 * |        |          |....
 * |        |          |SUSPITV = 0xE ... 14.5 SPICLK clock cycle.
 * |        |          |SUSPITV = 0xF ... 15.5 SPICLK clock cycle.
 * |        |          |Note:
 * |        |          |For DUAL and QUAD transactions with SUSPITV must be set to 0.
 * |[12:8]  |DWIDTH    |DWIDTH u2013 Data Word Bit Length
 * |        |          |This field specifies how many bits are transmitted in one transmit/receive
 * |        |          |Up to 32 bits can be transmitted.
 * |        |          |DWIDTH = 0x01 ... 1 bit.
 * |        |          |DWIDTH = 0x02 ... 2 bits.
 * |        |          |......
 * |        |          |DWIDTH = 0x1f ... 31 bits.
 * |        |          |DWIDTH = 0x00 ... 32 bits.
 * |[13]    |LSB       |LSB First
 * |        |          |0 = The MSB is transmitted/received first (which bit in TX and RX FIFO depends on the DWIDTH field).
 * |        |          |1 = The LSB is sent first on the line (bit 0 of TX FIFO]), and the first bit received from the line will be put in the LSB position in the SPIn_RX FIFO (bit 0 SPIn_RX).
 * |        |          |Note:
 * |        |          |For DUAL and QUAD transactions with LSB must be set to 0.
 * |[16]    |TWOBIT    |Two Bits Transfer Mode
 * |        |          |0 = Disable two-bit transfer mode.
 * |        |          |1 = Enable two-bit transfer mode.
 * |        |          |When 2-bit mode is enabled, the first serial transmitted bit data is from the first FIFO buffer data, and the 2nd serial transmitted bit data is from the second FIFO buffer data
 * |        |          |As the same as transmitted function, the first received bit data is stored into the first FIFO buffer and the 2nd received bit data is stored into the second FIFO buffer at the same time.
 * |[17]    |UNITIEN   |Unit Transfer Interrupt Enable
 * |        |          |0 = Disable SPI Unit Transfer Interrupt.
 * |        |          |1 = Enable SPI Unit Transfer Interrupt to CPU.
 * |[18]    |SLAVE     |Master Slave Mode Control
 * |        |          |0 = Master mode.
 * |        |          |1 = Slave mode.
 * |[19]    |REORDER   |Byte Reorder Function Enable
 * |        |          |0 = Byte reorder function Disabled.
 * |        |          |1 = Byte reorder function Enabled
 * |        |          |A byte suspend interval will be inserted between each byte
 * |        |          |The period of the byte suspend interval depends on the setting of SUSPITV.
 * |        |          |Note:
 * |        |          |Byte reorder function is only available if DWIDTH is defined as 16, 24, and 32 bits.
 * |        |          |REORDER is only available for Receive mode in DUAL and QUAD transactions.
 * |        |          |For DUAL and QUAD transactions with REORDER, SUSPITV must be set to 0.
 * |[20]    |QDIODIR   |Quad or Dual I/O Mode Direction Control
 * |        |          |0 = Quad or Dual Input mode.
 * |        |          |1 = Quad or Dual Output mode.
 * |[21]    |DUALIOEN  |Dual I/O Mode Enable
 * |        |          |0 = Dual I/O mode Disabled.
 * |        |          |1 = Dual I/O mode Enabled.
 * |[22]    |QUADIOEN  |Quad I/O Mode Enable
 * |        |          |0 = Quad I/O mode Disabled.
 * |        |          |1 = Quad I/O mode Enabled.
 * |[23]    |RXTCNTEN  |DMA Receive Transaction Count Enable
 * |        |          |0 = Disable function.
 * |        |          |1 = Enable transaction counter for DMA receive only mode
 * |        |          |SPI will perform the number of transfers specified in the SPI_RXTSNCNT register, allowing the SPI interface to read ahead of DMA controller.
 * |[24]    |RXMODEEN  |FIFO Receive Mode Enable
 * |        |          |0 = Disable function.
 * |        |          |1 = Enable FIFO receive mode
 * |        |          |In this mode SPI transactions will be continuously performed while RXFULL is not active
 * |        |          |To stop transactions, set RXMODEEN to 0.
 * @var SPI0_T::CLKDIV
 * Offset: 0x04  Clock Divider Register (Master Only)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DIVIDER   |Clock Divider Register
 * |        |          |The value in this field is the frequency divider for generating the SPI engine clock,Fspi_sclk, and the SPI serial clock of SPI master
 * |        |          |The frequency is obtained according to the following equation.
 * |        |          |Fspi_sclk = Fspi_clockSRC / (DIVIDER+1).
 * |        |          |where
 * |        |          |Fspi_clockSRC is the SPI engine clock source, which is defined in the clock control, CLKSEL1 register.
 * @var SPI0_T::SSCTL
 * Offset: 0x08  Slave Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |SS        |Slave Select Control Bits (Master Only)
 * |        |          |If AUTOSS bit is cleared, writing 1 to any bit of this field sets the proper SPISSx0/1 line to an active state and writing 0 sets the line back to inactive state.
 * |        |          |If the AUTOSS bit is set, writing 0 to any bit location of this field will keep the corresponding SPI_SS0/1 line at inactive state; writing 1 to any bit location of this field will select appropriate SPI_SS0/1 line to be automatically driven to active state for the duration of the transmit/receive, and will be driven to inactive state for the rest of the time
 * |        |          |The active state of SPI_SS0/1 is specified in SSACTPOL.
 * |        |          |Note: SPI_SS0 is defined as the slave select input in Slave mode.
 * |[2]     |SSACTPOL  |Slave Select Active Level
 * |        |          |This bit defines the active status of slave select signal (SPI_SS0/1).
 * |        |          |0 = The slave select signal SPI_SS0/1 is active on low-level/falling-edge.
 * |        |          |1 = The slave select signal SPI_SS0/1 is active on high-level/rising-edge.
 * |[3]     |AUTOSS    |Automatic Slave Select Function Enable (Master Only)
 * |        |          |0 = If this bit is cleared, slave select signals will be asserted/de-asserted by setting/clearing the corresponding bits of SPI_SSCTL[1:0].
 * |        |          |1 = If this bit is set, SPI_SS0/1 signals will be generated automatically
 * |        |          |It means that device/slave select signal, which is set in SPI_SSCTL[1:0], will be asserted by the SPI controller when transmit/receive is started, and will be de-asserted after each transmit/receive is finished.
 * |[4]     |SLV3WIRE  |Slave 3-wire Mode Enable
 * |        |          |This is used to ignore the slave select signal in Slave mode
 * |        |          |The SPI controller can work with 3-wire interface consisting of SPI_CLK, SPI_MISO, and SPI_MOSI.
 * |        |          |0 = 4-wire bi-directional interface.
 * |        |          |1 = 3-wire bi-directional interface.
 * |[5]     |SLVTOIEN  |Slave Mode Time-out Interrupt Enable
 * |        |          |0 = Slave mode time-out interrupt Disabled.
 * |        |          |1 = Slave mode time-out interrupt Enabled.
 * |[6]     |SLVTORST  |Slave Mode Time-out FIFO Clear
 * |        |          |0 = Function disabled.
 * |        |          |1 = Both the FIFO clear function, TXRST and RXRST, are activated automatically when there is a slave mode time-out event.
 * |[8]     |SLVBCEIEN |Slave Mode Error 0 Interrupt Enable
 * |        |          |0 = Slave mode error 0 interrupt Disable.
 * |        |          |1 = Slave mode error 0 interrupt Enable.
 * |[9]     |SLVUDRIEN |Slave Mode Error 1 Interrupt Enable
 * |        |          |0 = Slave mode error 1 interrupt Disable.
 * |        |          |1 = Slave mode error 1 interrupt Enable.
 * |[12]    |SSACTIEN  |Slave Select Active Interrupt Enable
 * |        |          |0 = Slave select active interrupt Disable.
 * |        |          |1 = Slave select active interrupt Enable.
 * |[13]    |SSINAIEN  |Slave Select Inactive Interrupt Enable
 * |        |          |0 = Slave select inactive interrupt Disable.
 * |        |          |1 = Slave select inactive interrupt Enable.
 * |[31:16] |SLVTOCNT  |Slave Mode Time-out Period
 * |        |          |In Slave mode, these bits indicate the time out period when there is serial clock input during slave select active
 * |        |          |The clock source of the time out counter is Slave engine clock
 * |        |          |If the value is 0, it indicates the slave mode time-out function is disabled.
 * @var SPI0_T::PDMACTL
 * Offset: 0x0C  SPI0 PDMA Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TXPDMAEN  |Transmit DMA Enable
 * |        |          |Setting this bit to 1 will start the transmit PDMA process
 * |        |          |SPI controller will issue request to PDMA controller automatically
 * |        |          |Hardware will clear this bit to 0 automatically after PDMA transfer done.
 * |[1]     |RXPDMAEN  |Receive PDMA Enable
 * |        |          |Setting this bit to 1 will start the receive PDMA process
 * |        |          |The SPI controller will issue request to PDMA controller automatically when the SPI receive buffer is not empty
 * |        |          |This bit will be cleared to 0 by hardware automatically after PDMA transfer is done.
 * |[2]     |PDMARST   |PDMA Reset
 * |        |          |0 = No effect.
 * |        |          |1 = Reset the PDMA control logic of the SPI controller. This bit will be cleared to 0 automatically.
 * @var SPI0_T::FIFOCTL
 * Offset: 0x10  FIFO Control/Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RXRST     |Clear Receive FIFO Buffer
 * |        |          |0 = No effect.
 * |        |          |1 = Clear receive FIFO buffer
 * |        |          |The RXFULL bit will be cleared to 0 and the RXEMPTY bit will be set to 1
 * |        |          |This bit will be cleared to 0 by hardware about 3 system clocks + 3 SPI engine clock after it is set to 1.
 * |        |          |Note: If there is slave receive time out event, the RXRST will be set 1 when the SPI_SSCTL.SLVTORST, is enabled.
 * |[1]     |TXRST     |Clear Transmit FIFO Buffer
 * |        |          |0 = No effect.
 * |        |          |1 = Clear transmit FIFO buffer
 * |        |          |The TXFULL bit will be cleared to 0 and the TXEMPTY bit will be set to 1
 * |        |          |This bit will be cleared to 0 by hardware about 3 system clocks + 3 SPI engine clock after it is set to 1.
 * |        |          |Note: If there is slave receive time out event, the TXRST will be set 1 when the SPI_SSCTL.SLVTORST, is enabled.
 * |[2]     |RXTHIEN   |Receive FIFO Threshold Interrupt Enable
 * |        |          |0 = RX FIFO threshold interrupt Disabled.
 * |        |          |1 = RX FIFO threshold interrupt Enabled.
 * |[3]     |TXTHIEN   |Transmit FIFO Threshold Interrupt Enable
 * |        |          |0 = TX FIFO threshold interrupt Disabled.
 * |        |          |1 = TX FIFO threshold interrupt Enabled.
 * |[4]     |RXTOIEN   |Slave Receive Time-out Interrupt Enable
 * |        |          |0 = Receive time-out interrupt Disabled.
 * |        |          |1 = Receive time-out interrupt Enabled.
 * |[5]     |RXOVIEN   |Receive FIFO Overrun Interrupt Enable
 * |        |          |0 = Receive FIFO overrun interrupt Disabled.
 * |        |          |1 = Receive FIFO overrun interrupt Enabled.
 * |[6]     |TXUDFPOL  |Transmit Under-run Data Out
 * |        |          |0 = The SPI data out is 0 if there is transmit under-run event in Slave mode.
 * |        |          |1 = The SPI data out is 1 if there is transmit under-run event in Slave mode.
 * |        |          |Note: The under run event is active after the serial clock input and the hardware synchronous, so that the first 1~3 bit (depending on the relation between system clock and the engine clock) data out will be the last transaction data.
 * |        |          |Note: If the frequency of system clock approach the engine clock, they may be a 3-bit time to report the transmit under-run data out.
 * |[7]     |TXUDFIEN  |Slave Transmit Under Run Interrupt Enable
 * |        |          |0 = Slave Transmit FIFO under-run interrupt Disabled.
 * |        |          |1 = Slave Transmit FIFO under-run interrupt Enabled.
 * |[25:24] |RXTH      |Receive FIFO Threshold
 * |        |          |If the valid data count of the receive FIFO buffer is larger than the RXTH setting, the RXTHIF bit will be set to 1, else the RXTHIF bit will be cleared to 0.
 * |        |          |00: 1 word will transmit
 * |        |          |01: 2 word will transmit
 * |        |          |10: 3 word will transmit
 * |        |          |11: 4 word will transmit
 * |[29:28] |TXTH      |Transmit FIFO Threshold
 * |        |          |If the valid data count of the transmit FIFO buffer is less than or equal to the TXTH setting, the TXTHIF bit will be set to 1, else the TXTHIF bit will be cleared to 0.
 * |        |          |00: 1 word will transmit
 * |        |          |01: 2 word will transmit
 * |        |          |10: 3 word will transmit
 * |        |          |11: 4 word will transmit
 * @var SPI0_T::STATUS
 * Offset: 0x14  Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |BUSY      |SPI Unit Bus Status (Read Only)
 * |        |          |0 = No transaction in the SPI bus.
 * |        |          |1 = SPI controller unit is in busy state.
 * |        |          |The following listing are the bus busy conditions:
 * |        |          |SPIEN = 1 and the TXEMPTY = 0.
 * |        |          |For SPI Master, the TXEMPTY = 1 but the current transaction is not finished yet.
 * |        |          |For SPI Slave receive mode, the SPIEN = 1 and there is serial clock input into the SPI core logic when slave select is active.
 * |        |          |For SPI Slave transmit mode, the SPIEN = 1 and the transmit buffer is not empty in SPI core logic event if the slave select is inactive.
 * |[1]     |UNITIF    |Unit Transfer Interrupt Status
 * |        |          |0 = No transaction has been finished since this bit was cleared to 0.
 * |        |          |1 = SPI controller has finished one unit transfer.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[2]     |SSACTIF   |Slave Select Active Interrupt Status
 * |        |          |0 = Slave select active interrupt is clear or not occur.
 * |        |          |1 = Slave select active interrupt event has occur.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[3]     |SSINAIF   |Slave Select Inactive Interrupt Status
 * |        |          |0 = Slave select inactive interrupt is clear or not occur.
 * |        |          |1 = Slave select inactive interrupt event has occur.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[4]     |SSLINE    |Slave Select Line Bus Status (Read Only)
 * |        |          |0 = Indicates the slave select line bus status is 0.
 * |        |          |1 = Indicates the slave select line bus status is 1.
 * |        |          |Note: If SPI_SSCTL.SSACTPOL is set 0, and the SSLINE is 1, the SPI slave select is in inactive status.
 * |[5]     |SLVTOIF   |Slave Time-out Interrupt Status (Read Only)
 * |        |          |When the Slave Select is active and the value of SLVTOCNT is not 0 and the serial clock input, the slave time-out counter in SPI controller logic will be start
 * |        |          |When the value of time-out counter greater or equal than the value of SPI_SSCTL.SLVTOCNT, during before one transaction done, the slave time-out interrupt event will active.
 * |        |          |0 = Slave time-out is not active.
 * |        |          |1 = Slave time-out is active.
 * |        |          |Note: If the DWIDTH is set 16, one transaction is equal 16 bits serial clock period.
 * |[6]     |SLVBEIF   |Slave Mode Error 0 Interrupt Status (Read Only)
 * |        |          |In Slave mode, there is bit counter mismatch with DWIDTH when the slave select line goes to inactive state.
 * |        |          |0 = No Slave mode error 0 event.
 * |        |          |1 = Slave mode error 0 occurs.
 * |        |          |Note: If the slave select active but there is no any serial clock input, the SLVBEIF also active when the slave select goes to inactive state.
 * |[7]     |SLVURIF   |Slave Mode Error 1 Interrupt Status (Read Only)
 * |        |          |In Slave mode, transmit under-run occurs when the slave select line goes to inactive state.
 * |        |          |0 = No Slave mode error 1 event.
 * |        |          |1 = Slave mode error 1 occurs.
 * |[8]     |RXEMPTY   |Receive FIFO Buffer Empty Indicator (Read Only)
 * |        |          |0 = Receive FIFO buffer is not empty.
 * |        |          |1 = Receive FIFO buffer is empty.
 * |[9]     |RXFULL    |Receive FIFO Buffer Full Indicator (Read Only)
 * |        |          |0 = Receive FIFO buffer is not full.
 * |        |          |1 = Receive FIFO buffer is full.
 * |[10]    |RXTHIF    |Receive FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count within the Rx FIFO buffer is smaller than or equal to the setting value of RXTH.
 * |        |          |1 = The valid data count within the receive FIFO buffer is larger than the setting value of RXTH.
 * |        |          |Note: If RXTHIEN = 1 and RXTHIF = 1, the SPI controller will generate a SPI interrupt request.
 * |[11]    |RXOVIF    |Receive FIFO Overrun Status
 * |        |          |When the receive FIFO buffer is full, the follow-up data will be dropped and this bit will be set to 1.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[12]    |RXTOIF    |Receive Time-out Interrupt Status
 * |        |          |0 = No receive FIFO time-out event.
 * |        |          |1 = Receive FIFO buffer is not empty and no read operation on receive FIFO buffer over 64 SPI clock period in Master mode or over 576 SPI engine clock period in Slave mode
 * |        |          |When the received FIFO buffer is read by software, the time-out status will be cleared automatically.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[15]    |SPIENSTS  |SPI Enable Bit Status (Read Only)
 * |        |          |0 = Indicate the transmit control bit is disabled.
 * |        |          |1 = Indicate the transfer control bit is active.
 * |        |          |Note: The clock source of SPI controller logic is engine clock, it is asynchronous with the system clock
 * |        |          |In order to make sure the function is disabled in SPI controller logic, this bit indicates the real status of SPIEN in SPI controller logic for user.
 * |[16]    |TXEMPTY   |Transmit FIFO Buffer Empty Indicator (Read Only)
 * |        |          |0 = Transmit FIFO buffer is not empty.
 * |        |          |1 = Transmit FIFO buffer is empty.
 * |[17]    |TXFULL    |Transmit FIFO Buffer Full Indicator (Read Only)
 * |        |          |0 = Transmit FIFO buffer is not full.
 * |        |          |1 = Transmit FIFO buffer is full.
 * |[18]    |TXTHIF    |Transmit FIFO Threshold Interrupt Status (Read Only)
 * |        |          |0 = The valid data count of the transmit FIFO buffer is larger than the setting value of TXTH.
 * |        |          |1 = The valid data count of the transmit FIFO buffer is less than or equal to the setting value of TXTH.
 * |        |          |Note: If TXTHIEN = 1 and TXTHIF = 1, the SPI controller will generate a SPI interrupt request.
 * |[19]    |TXUFIF    |Slave Transmit FIFO Under-run Interrupt Status (Read Only)
 * |        |          |When the transmit FIFO buffer is empty and further serial clock pulses occur, data transmitted will be the value of the last transmitted bit and this under-run bit will be set.
 * |        |          |Note: This bit will be cleared by writing 1 to itself.
 * |[23]    |TXRXRST   |FIFO CLR Status (Read Only)
 * |        |          |0 = Done the FIFO buffer clear function of TXRST and RXRST.
 * |        |          |1 = Doing the FIFO buffer clear function of TXRST or RXRST.
 * |        |          |Note: Both the TXRST, RXRST, need 3 system clock + 3 engine clocks, the status of this bit allows the user to monitor whether the clear function is busy or done.
 * |[27:24] |RXCNT     |Receive FIFO Data Count (Read Only)
 * |        |          |This bit field indicates the valid data count of receive FIFO buffer.
 * |[31:28] |TXCNT     |Transmit FIFO Data Count (Read Only)
 * |        |          |This bit field indicates the valid data count of transmit FIFO buffer.
 * @var SPI0_T::RXTSNCNT
 * Offset: 0x18  Receive Transaction Count Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[16:0]  |RXTSNCNT  |DMA Receive Transaction Count
 * |        |          |When using DMA to receive SPI data without transmitting data, this register can be used in conjunction with the control bit SPI_CTL.RXTCNTEN to set number of transactions to perform
 * |        |          |Without this, the SPI interface will only initiate a transaction when it receives a request from the DMA system, resulting in a lower achievable data rate.
 * @var SPI0_T::TX
 * Offset: 0x20  FIFO Data Transmit Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |TX        |Data Transmit Register
 * |        |          |A write to the data transmit register pushes data onto into the 8-level transmit FIFO buffer
 * |        |          |The number of valid bits depends on the setting of transmit bit width field of the SPI_CTL register.
 * |        |          |For example, if DWIDTH is set to 0x08, the bits TX[7:0] will be transmitted
 * |        |          |If DWIDTH is set to 0, the SPI controller will perform a 32-bit transfer.
 * @var SPI0_T::RX
 * Offset: 0x30  FIFO Data Receive Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |RX        |Data Receive Register
 * |        |          |A read from this register pops data from the 8-level receive FIFO
 * |        |          |Valid data is present if the SPI_STATUS
 * |        |          |RXEMPTY bit is not set to 1
 * |        |          |This is a read-only register
 * @var SPI0_T::VERNUM
 * Offset: 0x50  IP Version Number Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 */
    __IO uint32_t CTL;                   /*!< [0x0000] Control and Status Register                                      */
    __IO uint32_t CLKDIV;                /*!< [0x0004] Clock Divider Register (Master Only)                             */
    __IO uint32_t SSCTL;                 /*!< [0x0008] Slave Select Register                                            */
    __IO uint32_t PDMACTL;               /*!< [0x000c] SPI0 PDMA Control Register                                        */
    __IO uint32_t FIFOCTL;               /*!< [0x0010] FIFO Control/Status Register                                     */
    __IO uint32_t STATUS;                /*!< [0x0014] Status Register                                                  */
    __IO uint32_t RXTSNCNT;              /*!< [0x0018] Receive Transaction Count Register                               */
    __I  uint32_t RESERVE0[1];
    __O  uint32_t TX;                    /*!< [0x0020] FIFO Data Transmit Register                                      */
    __I  uint32_t RESERVE1[3];
    __I  uint32_t RX;                    /*!< [0x0030] FIFO Data Receive Register                                       */
    __I  uint32_t RESERVE2[7];
    __IO uint32_t VERNUM;                /*!< [0x0050] IP Version Number Register                                       */

} SPI0_T;

/**
    @addtogroup SPI0_CONST SPI0 Bit Field Definition
    Constant Definitions for SPI0 Controller
@{ */

#define SPI0_CTL_SPIEN_Pos                (0)                                               /*!< SPI0_T::CTL: SPIEN Position             */
#define SPI0_CTL_SPIEN_Msk                (0x1ul << SPI0_CTL_SPIEN_Pos)                      /*!< SPI0_T::CTL: SPIEN Mask                 */

#define SPI0_CTL_RXNEG_Pos                (1)                                               /*!< SPI0_T::CTL: RXNEG Position             */
#define SPI0_CTL_RXNEG_Msk                (0x1ul << SPI0_CTL_RXNEG_Pos)                      /*!< SPI0_T::CTL: RXNEG Mask                 */

#define SPI0_CTL_TXNEG_Pos                (2)                                               /*!< SPI0_T::CTL: TXNEG Position             */
#define SPI0_CTL_TXNEG_Msk                (0x1ul << SPI0_CTL_TXNEG_Pos)                      /*!< SPI0_T::CTL: TXNEG Mask                 */

#define SPI0_CTL_CLKPOL_Pos               (3)                                               /*!< SPI0_T::CTL: CLKPOL Position            */
#define SPI0_CTL_CLKPOL_Msk               (0x1ul << SPI0_CTL_CLKPOL_Pos)                     /*!< SPI0_T::CTL: CLKPOL Mask                */

#define SPI0_CTL_SUSPITV_Pos              (4)                                               /*!< SPI0_T::CTL: SUSPITV Position           */
#define SPI0_CTL_SUSPITV_Msk              (0xful << SPI0_CTL_SUSPITV_Pos)                    /*!< SPI0_T::CTL: SUSPITV Mask               */

#define SPI0_CTL_DWIDTH_Pos               (8)                                               /*!< SPI0_T::CTL: DWIDTH Position            */
#define SPI0_CTL_DWIDTH_Msk               (0x1ful << SPI0_CTL_DWIDTH_Pos)                    /*!< SPI0_T::CTL: DWIDTH Mask                */

#define SPI0_CTL_LSB_Pos                  (13)                                              /*!< SPI0_T::CTL: LSB Position               */
#define SPI0_CTL_LSB_Msk                  (0x1ul << SPI0_CTL_LSB_Pos)                        /*!< SPI0_T::CTL: LSB Mask                   */

#define SPI0_CTL_TWOBIT_Pos               (16)                                              /*!< SPI0_T::CTL: TWOBIT Position            */
#define SPI0_CTL_TWOBIT_Msk               (0x1ul << SPI0_CTL_TWOBIT_Pos)                     /*!< SPI0_T::CTL: TWOBIT Mask                */

#define SPI0_CTL_UNITIEN_Pos              (17)                                              /*!< SPI0_T::CTL: UNITIEN Position           */
#define SPI0_CTL_UNITIEN_Msk              (0x1ul << SPI0_CTL_UNITIEN_Pos)                    /*!< SPI0_T::CTL: UNITIEN Mask               */

#define SPI0_CTL_SLAVE_Pos                (18)                                              /*!< SPI0_T::CTL: SLAVE Position             */
#define SPI0_CTL_SLAVE_Msk                (0x1ul << SPI0_CTL_SLAVE_Pos)                      /*!< SPI0_T::CTL: SLAVE Mask                 */

#define SPI0_CTL_REORDER_Pos              (19)                                              /*!< SPI0_T::CTL: REORDER Position           */
#define SPI0_CTL_REORDER_Msk              (0x1ul << SPI0_CTL_REORDER_Pos)                    /*!< SPI0_T::CTL: REORDER Mask               */

#define SPI0_CTL_QDIODIR_Pos              (20)                                              /*!< SPI0_T::CTL: QDIODIR Position           */
#define SPI0_CTL_QDIODIR_Msk              (0x1ul << SPI0_CTL_QDIODIR_Pos)                    /*!< SPI0_T::CTL: QDIODIR Mask               */

#define SPI0_CTL_DUALIOEN_Pos             (21)                                              /*!< SPI0_T::CTL: DUALIOEN Position          */
#define SPI0_CTL_DUALIOEN_Msk             (0x1ul << SPI0_CTL_DUALIOEN_Pos)                   /*!< SPI0_T::CTL: DUALIOEN Mask              */

#define SPI0_CTL_QUADIOEN_Pos             (22)                                              /*!< SPI0_T::CTL: QUADIOEN Position          */
#define SPI0_CTL_QUADIOEN_Msk             (0x1ul << SPI0_CTL_QUADIOEN_Pos)                   /*!< SPI0_T::CTL: QUADIOEN Mask              */

#define SPI0_CTL_RXTCNTEN_Pos             (23)                                              /*!< SPI0_T::CTL: RXTCNTEN Position          */
#define SPI0_CTL_RXTCNTEN_Msk             (0x1ul << SPI0_CTL_RXTCNTEN_Pos)                   /*!< SPI0_T::CTL: RXTCNTEN Mask              */

#define SPI0_CTL_RXMODEEN_Pos             (24)                                              /*!< SPI0_T::CTL: RXMODEEN Position          */
#define SPI0_CTL_RXMODEEN_Msk             (0x1ul << SPI0_CTL_RXMODEEN_Pos)                   /*!< SPI0_T::CTL: RXMODEEN Mask              */

#define SPI0_CLKDIV_DIVIDER_Pos           (0)                                               /*!< SPI0_T::CLKDIV: DIVIDER Position        */
#define SPI0_CLKDIV_DIVIDER_Msk           (0xfful << SPI0_CLKDIV_DIVIDER_Pos)                /*!< SPI0_T::CLKDIV: DIVIDER Mask            */

#define SPI0_SSCTL_SS_Pos                 (0)                                               /*!< SPI0_T::SSCTL: SS Position              */
#define SPI0_SSCTL_SS_Msk                 (0x3ul << SPI0_SSCTL_SS_Pos)                       /*!< SPI0_T::SSCTL: SS Mask                  */

#define SPI0_SSCTL_SSACTPOL_Pos           (2)                                               /*!< SPI0_T::SSCTL: SSACTPOL Position        */
#define SPI0_SSCTL_SSACTPOL_Msk           (0x1ul << SPI0_SSCTL_SSACTPOL_Pos)                 /*!< SPI0_T::SSCTL: SSACTPOL Mask            */

#define SPI0_SSCTL_AUTOSS_Pos             (3)                                               /*!< SPI0_T::SSCTL: AUTOSS Position          */
#define SPI0_SSCTL_AUTOSS_Msk             (0x1ul << SPI0_SSCTL_AUTOSS_Pos)                   /*!< SPI0_T::SSCTL: AUTOSS Mask              */

#define SPI0_SSCTL_SLV3WIRE_Pos           (4)                                               /*!< SPI0_T::SSCTL: SLV3WIRE Position        */
#define SPI0_SSCTL_SLV3WIRE_Msk           (0x1ul << SPI0_SSCTL_SLV3WIRE_Pos)                 /*!< SPI0_T::SSCTL: SLV3WIRE Mask            */

#define SPI0_SSCTL_SLVTOIEN_Pos           (5)                                               /*!< SPI0_T::SSCTL: SLVTOIEN Position        */
#define SPI0_SSCTL_SLVTOIEN_Msk           (0x1ul << SPI0_SSCTL_SLVTOIEN_Pos)                 /*!< SPI0_T::SSCTL: SLVTOIEN Mask            */

#define SPI0_SSCTL_SLVTORST_Pos           (6)                                               /*!< SPI0_T::SSCTL: SLVTORST Position        */
#define SPI0_SSCTL_SLVTORST_Msk           (0x1ul << SPI0_SSCTL_SLVTORST_Pos)                 /*!< SPI0_T::SSCTL: SLVTORST Mask            */

#define SPI0_SSCTL_SLVBCEIEN_Pos          (8)                                               /*!< SPI0_T::SSCTL: SLVBCEIEN Position       */
#define SPI0_SSCTL_SLVBCEIEN_Msk          (0x1ul << SPI0_SSCTL_SLVBCEIEN_Pos)                /*!< SPI0_T::SSCTL: SLVBCEIEN Mask           */

#define SPI0_SSCTL_SLVUDRIEN_Pos          (9)                                               /*!< SPI0_T::SSCTL: SLVUDRIEN Position       */
#define SPI0_SSCTL_SLVUDRIEN_Msk          (0x1ul << SPI0_SSCTL_SLVUDRIEN_Pos)                /*!< SPI0_T::SSCTL: SLVUDRIEN Mask           */

#define SPI0_SSCTL_SSACTIEN_Pos           (12)                                              /*!< SPI0_T::SSCTL: SSACTIEN Position        */
#define SPI0_SSCTL_SSACTIEN_Msk           (0x1ul << SPI0_SSCTL_SSACTIEN_Pos)                 /*!< SPI0_T::SSCTL: SSACTIEN Mask            */

#define SPI0_SSCTL_SSINAIEN_Pos           (13)                                              /*!< SPI0_T::SSCTL: SSINAIEN Position        */
#define SPI0_SSCTL_SSINAIEN_Msk           (0x1ul << SPI0_SSCTL_SSINAIEN_Pos)                 /*!< SPI0_T::SSCTL: SSINAIEN Mask            */

#define SPI0_SSCTL_SLVTOCNT_Pos           (16)                                              /*!< SPI0_T::SSCTL: SLVTOCNT Position        */
#define SPI0_SSCTL_SLVTOCNT_Msk           (0xfffful << SPI0_SSCTL_SLVTOCNT_Pos)              /*!< SPI0_T::SSCTL: SLVTOCNT Mask            */

#define SPI0_PDMACTL_TXPDMAEN_Pos         (0)                                               /*!< SPI0_T::PDMACTL: TXPDMAEN Position      */
#define SPI0_PDMACTL_TXPDMAEN_Msk         (0x1ul << SPI0_PDMACTL_TXPDMAEN_Pos)               /*!< SPI0_T::PDMACTL: TXPDMAEN Mask          */

#define SPI0_PDMACTL_RXPDMAEN_Pos         (1)                                               /*!< SPI0_T::PDMACTL: RXPDMAEN Position      */
#define SPI0_PDMACTL_RXPDMAEN_Msk         (0x1ul << SPI0_PDMACTL_RXPDMAEN_Pos)               /*!< SPI0_T::PDMACTL: RXPDMAEN Mask          */

#define SPI0_PDMACTL_PDMARST_Pos          (2)                                               /*!< SPI0_T::PDMACTL: PDMARST Position       */
#define SPI0_PDMACTL_PDMARST_Msk          (0x1ul << SPI0_PDMACTL_PDMARST_Pos)                /*!< SPI0_T::PDMACTL: PDMARST Mask           */

#define SPI0_FIFOCTL_RXRST_Pos            (0)                                               /*!< SPI0_T::FIFOCTL: RXRST Position         */
#define SPI0_FIFOCTL_RXRST_Msk            (0x1ul << SPI0_FIFOCTL_RXRST_Pos)                  /*!< SPI0_T::FIFOCTL: RXRST Mask             */

#define SPI0_FIFOCTL_TXRST_Pos            (1)                                               /*!< SPI0_T::FIFOCTL: TXRST Position         */
#define SPI0_FIFOCTL_TXRST_Msk            (0x1ul << SPI0_FIFOCTL_TXRST_Pos)                  /*!< SPI0_T::FIFOCTL: TXRST Mask             */

#define SPI0_FIFOCTL_RXTHIEN_Pos          (2)                                               /*!< SPI0_T::FIFOCTL: RXTHIEN Position       */
#define SPI0_FIFOCTL_RXTHIEN_Msk          (0x1ul << SPI0_FIFOCTL_RXTHIEN_Pos)                /*!< SPI0_T::FIFOCTL: RXTHIEN Mask           */

#define SPI0_FIFOCTL_TXTHIEN_Pos          (3)                                               /*!< SPI0_T::FIFOCTL: TXTHIEN Position       */
#define SPI0_FIFOCTL_TXTHIEN_Msk          (0x1ul << SPI0_FIFOCTL_TXTHIEN_Pos)                /*!< SPI0_T::FIFOCTL: TXTHIEN Mask           */

#define SPI0_FIFOCTL_RXTOIEN_Pos          (4)                                               /*!< SPI0_T::FIFOCTL: RXTOIEN Position       */
#define SPI0_FIFOCTL_RXTOIEN_Msk          (0x1ul << SPI0_FIFOCTL_RXTOIEN_Pos)                /*!< SPI0_T::FIFOCTL: RXTOIEN Mask           */

#define SPI0_FIFOCTL_RXOVIEN_Pos          (5)                                               /*!< SPI0_T::FIFOCTL: RXOVIEN Position       */
#define SPI0_FIFOCTL_RXOVIEN_Msk          (0x1ul << SPI0_FIFOCTL_RXOVIEN_Pos)                /*!< SPI0_T::FIFOCTL: RXOVIEN Mask           */

#define SPI0_FIFOCTL_TXUDFPOL_Pos         (6)                                               /*!< SPI0_T::FIFOCTL: TXUDFPOL Position      */
#define SPI0_FIFOCTL_TXUDFPOL_Msk         (0x1ul << SPI0_FIFOCTL_TXUDFPOL_Pos)               /*!< SPI0_T::FIFOCTL: TXUDFPOL Mask          */

#define SPI0_FIFOCTL_TXUDFIEN_Pos         (7)                                               /*!< SPI0_T::FIFOCTL: TXUDFIEN Position      */
#define SPI0_FIFOCTL_TXUDFIEN_Msk         (0x1ul << SPI0_FIFOCTL_TXUDFIEN_Pos)               /*!< SPI0_T::FIFOCTL: TXUDFIEN Mask          */

#define SPI0_FIFOCTL_RXTH_Pos             (24)                                              /*!< SPI0_T::FIFOCTL: RXTH Position          */
#define SPI0_FIFOCTL_RXTH_Msk             (0x3ul << SPI0_FIFOCTL_RXTH_Pos)                   /*!< SPI0_T::FIFOCTL: RXTH Mask              */

#define SPI0_FIFOCTL_TXTH_Pos             (28)                                              /*!< SPI0_T::FIFOCTL: TXTH Position          */
#define SPI0_FIFOCTL_TXTH_Msk             (0x3ul << SPI0_FIFOCTL_TXTH_Pos)                   /*!< SPI0_T::FIFOCTL: TXTH Mask              */

#define SPI0_STATUS_BUSY_Pos              (0)                                               /*!< SPI0_T::STATUS: BUSY Position           */
#define SPI0_STATUS_BUSY_Msk              (0x1ul << SPI0_STATUS_BUSY_Pos)                    /*!< SPI0_T::STATUS: BUSY Mask               */

#define SPI0_STATUS_UNITIF_Pos            (1)                                               /*!< SPI0_T::STATUS: UNITIF Position         */
#define SPI0_STATUS_UNITIF_Msk            (0x1ul << SPI0_STATUS_UNITIF_Pos)                  /*!< SPI0_T::STATUS: UNITIF Mask             */

#define SPI0_STATUS_SSACTIF_Pos           (2)                                               /*!< SPI0_T::STATUS: SSACTIF Position        */
#define SPI0_STATUS_SSACTIF_Msk           (0x1ul << SPI0_STATUS_SSACTIF_Pos)                 /*!< SPI0_T::STATUS: SSACTIF Mask            */

#define SPI0_STATUS_SSINAIF_Pos           (3)                                               /*!< SPI0_T::STATUS: SSINAIF Position        */
#define SPI0_STATUS_SSINAIF_Msk           (0x1ul << SPI0_STATUS_SSINAIF_Pos)                 /*!< SPI0_T::STATUS: SSINAIF Mask            */

#define SPI0_STATUS_SSLINE_Pos            (4)                                               /*!< SPI0_T::STATUS: SSLINE Position         */
#define SPI0_STATUS_SSLINE_Msk            (0x1ul << SPI0_STATUS_SSLINE_Pos)                  /*!< SPI0_T::STATUS: SSLINE Mask             */

#define SPI0_STATUS_SLVTOIF_Pos           (5)                                               /*!< SPI0_T::STATUS: SLVTOIF Position        */
#define SPI0_STATUS_SLVTOIF_Msk           (0x1ul << SPI0_STATUS_SLVTOIF_Pos)                 /*!< SPI0_T::STATUS: SLVTOIF Mask            */

#define SPI0_STATUS_SLVBEIF_Pos           (6)                                               /*!< SPI0_T::STATUS: SLVBEIF Position        */
#define SPI0_STATUS_SLVBEIF_Msk           (0x1ul << SPI0_STATUS_SLVBEIF_Pos)                 /*!< SPI0_T::STATUS: SLVBEIF Mask            */

#define SPI0_STATUS_SLVURIF_Pos           (7)                                               /*!< SPI0_T::STATUS: SLVURIF Position        */
#define SPI0_STATUS_SLVURIF_Msk           (0x1ul << SPI0_STATUS_SLVURIF_Pos)                 /*!< SPI0_T::STATUS: SLVURIF Mask            */

#define SPI0_STATUS_RXEMPTY_Pos           (8)                                               /*!< SPI0_T::STATUS: RXEMPTY Position        */
#define SPI0_STATUS_RXEMPTY_Msk           (0x1ul << SPI0_STATUS_RXEMPTY_Pos)                 /*!< SPI0_T::STATUS: RXEMPTY Mask            */

#define SPI0_STATUS_RXFULL_Pos            (9)                                               /*!< SPI0_T::STATUS: RXFULL Position         */
#define SPI0_STATUS_RXFULL_Msk            (0x1ul << SPI0_STATUS_RXFULL_Pos)                  /*!< SPI0_T::STATUS: RXFULL Mask             */

#define SPI0_STATUS_RXTHIF_Pos            (10)                                              /*!< SPI0_T::STATUS: RXTHIF Position         */
#define SPI0_STATUS_RXTHIF_Msk            (0x1ul << SPI0_STATUS_RXTHIF_Pos)                  /*!< SPI0_T::STATUS: RXTHIF Mask             */

#define SPI0_STATUS_RXOVIF_Pos            (11)                                              /*!< SPI0_T::STATUS: RXOVIF Position         */
#define SPI0_STATUS_RXOVIF_Msk            (0x1ul << SPI0_STATUS_RXOVIF_Pos)                  /*!< SPI0_T::STATUS: RXOVIF Mask             */

#define SPI0_STATUS_RXTOIF_Pos            (12)                                              /*!< SPI0_T::STATUS: RXTOIF Position         */
#define SPI0_STATUS_RXTOIF_Msk            (0x1ul << SPI0_STATUS_RXTOIF_Pos)                  /*!< SPI0_T::STATUS: RXTOIF Mask             */

#define SPI0_STATUS_SPIENSTS_Pos          (15)                                              /*!< SPI0_T::STATUS: SPIENSTS Position       */
#define SPI0_STATUS_SPIENSTS_Msk          (0x1ul << SPI0_STATUS_SPIENSTS_Pos)                /*!< SPI0_T::STATUS: SPIENSTS Mask           */

#define SPI0_STATUS_TXEMPTY_Pos           (16)                                              /*!< SPI0_T::STATUS: TXEMPTY Position        */
#define SPI0_STATUS_TXEMPTY_Msk           (0x1ul << SPI0_STATUS_TXEMPTY_Pos)                 /*!< SPI0_T::STATUS: TXEMPTY Mask            */

#define SPI0_STATUS_TXFULL_Pos            (17)                                              /*!< SPI0_T::STATUS: TXFULL Position         */
#define SPI0_STATUS_TXFULL_Msk            (0x1ul << SPI0_STATUS_TXFULL_Pos)                  /*!< SPI0_T::STATUS: TXFULL Mask             */

#define SPI0_STATUS_TXTHIF_Pos            (18)                                              /*!< SPI0_T::STATUS: TXTHIF Position         */
#define SPI0_STATUS_TXTHIF_Msk            (0x1ul << SPI0_STATUS_TXTHIF_Pos)                  /*!< SPI0_T::STATUS: TXTHIF Mask             */

#define SPI0_STATUS_TXUFIF_Pos            (19)                                              /*!< SPI0_T::STATUS: TXUFIF Position         */
#define SPI0_STATUS_TXUFIF_Msk            (0x1ul << SPI0_STATUS_TXUFIF_Pos)                  /*!< SPI0_T::STATUS: TXUFIF Mask             */

#define SPI0_STATUS_TXRXRST_Pos           (23)                                              /*!< SPI0_T::STATUS: TXRXRST Position        */
#define SPI0_STATUS_TXRXRST_Msk           (0x1ul << SPI0_STATUS_TXRXRST_Pos)                 /*!< SPI0_T::STATUS: TXRXRST Mask            */

#define SPI0_STATUS_RXCNT_Pos             (24)                                              /*!< SPI0_T::STATUS: RXCNT Position          */
#define SPI0_STATUS_RXCNT_Msk             (0xful << SPI0_STATUS_RXCNT_Pos)                   /*!< SPI0_T::STATUS: RXCNT Mask              */

#define SPI0_STATUS_TXCNT_Pos             (28)                                              /*!< SPI0_T::STATUS: TXCNT Position          */
#define SPI0_STATUS_TXCNT_Msk             (0xful << SPI0_STATUS_TXCNT_Pos)                   /*!< SPI0_T::STATUS: TXCNT Mask              */

#define SPI0_RXTSNCNT_RXTSNCNT_Pos        (0)                                               /*!< SPI0_T::RXTSNCNT: RXTSNCNT Position     */
#define SPI0_RXTSNCNT_RXTSNCNT_Msk        (0x1fffful << SPI0_RXTSNCNT_RXTSNCNT_Pos)          /*!< SPI0_T::RXTSNCNT: RXTSNCNT Mask         */

#define SPI0_TX_TX_Pos                    (0)                                               /*!< SPI0_T::TX: TX Position                 */
#define SPI0_TX_TX_Msk                    (0xfffffffful << SPI0_TX_TX_Pos)                   /*!< SPI0_T::TX: TX Mask                     */

#define SPI0_RX_RX_Pos                    (0)                                               /*!< SPI0_T::RX: RX Position                 */
#define SPI0_RX_RX_Msk                    (0xfffffffful << SPI0_RX_RX_Pos)                   /*!< SPI0_T::RX: RX Mask                     */

/**@}*/ /* SPI0_CONST */
/**@}*/ /* end of SPI0 register group */


/*---------------------- Serial 1 Peripheral Interface Controller -------------------------*/
/**
    @addtogroup SPI1 Serial 1 Peripheral Interface Controller(SPI1)
    Memory Mapped Structure for SPI1 Controller
@{ */
 
typedef struct
{


/**
 * @var SPI1_T::CTL
 * Offset: 0x00  Control and Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |EN        |Go and Busy Status
 * |        |          |0 = Writing 0 to this bit has no effect.
 * |        |          |1 = Writing 1 to this bit starts the transfer
 * |        |          |This bit remains set during the transfer and is automatically cleared after transfer finished.
 * |        |          |NOTE: All registers should be set before writing 1 to this EN bit
 * |        |          |When a transfer is in progress, writing to any register of the SPI master/slave core has no effect.
 * |[1]     |RXNEG     |Receive At Negative Edge
 * |        |          |0 = The received data input signal is latched at the rising edge of SCLK.
 * |        |          |1 = The received data input signal is latched at the falling edge of SCLK.
 * |[2]     |TXNEG     |Transmit At Negative Edge
 * |        |          |0 = The transmitted data output signal is changed at the rising edge of SCLK.
 * |        |          |1 = The transmitted data output signal is changed at the falling edge of SCLK.
 * |[7:3]   |TXBITLEN  |Transmit Bit Length
 * |        |          |This field specifies how many bits are transmitted in one transmit/receive
 * |        |          |Up to 32 bits can be transmitted.
 * |        |          |00001 = 1 bit
 * |        |          |00010 = 2 bit
 * |        |          |00011 = 3 bit
 * |        |          |00100 = 4 bit
 * |        |          |00101 = 5 bit
 * |        |          |00110 = 6 bit
 * |        |          |00111 = 7 bit
 * |        |          |01000 = 8 bit
 * |        |          |01001 = 9 bit
 * |        |          |01010 = 10 bit
 * |        |          |01011 = 11 bit
 * |        |          |01100 = 12 bit
 * |        |          |01101 = 13 bit
 * |        |          |01110 = 14 bit
 * |        |          |01111 = 15 bit
 * |        |          |10000 = 16 bit
 * |        |          |10001 = 17 bit
 * |        |          |10010 = 18 bit
 * |        |          |10011 = 19 bit
 * |        |          |10100 = 20 bit
 * |        |          |10101 = 21 bit
 * |        |          |10110 = 22 bit
 * |        |          |10111 = 23 bit
 * |        |          |11000 = 24 bit
 * |        |          |11001 = 25 bit
 * |        |          |11010 = 26 bit
 * |        |          |11011 = 27 bit
 * |        |          |11100 = 28 bit
 * |        |          |11101 = 29 bit
 * |        |          |11110 = 30 bit
 * |        |          |11111 = 31 bit
 * |        |          |00000 = 32 bit
 * |[9:8]   |TXNUM     |Transmit/Receive Word Numbers
 * |        |          |This field specifies how many transmit/receive word numbers should be executed in one transfer.
 * |        |          |00 = Only one transmit/receive word will be executed in one transfer.
 * |        |          |01 = Two successive transmit/receive word will be executed in one transfer.
 * |        |          |10 = Reserved.
 * |        |          |11 = Reserved.
 * |[10]    |LSB       |LSB First
 * |        |          |0 = The MSB is transmitted/received first (which bit in SPI1_TX0/1 and SPI1_RX0/1 register that is depends on the TXBITLEN field).
 * |        |          |1 = The LSB is sent first on the line (bit 0 of SPI1_TX0/1), and the first bit received from the line will be put in the LSB position in the Rx register (bit 0 of SPI1_RX0/1).
 * |[11]    |CLKP      |Clock Polarity
 * |        |          |0 = SCLK idle low.
 * |        |          |1 = SCLK idle high.
 * |[15:12] |SLEEP     |Suspend Interval (Master Only)
 * |        |          |These four bits provide configurable suspend interval between two successive transmit/receive transactions in a transfer
 * |        |          |The suspend interval is from the last falling clock edge of the current transaction to the first rising clock edge of the successive transaction if CLKP = 0
 * |        |          |If CLKP = 1, the interval is from the rising clock edge to the falling clock edge
 * |        |          |The default value is 0x0
 * |        |          |When TXNUM = 00b, setting this field has no effect on transfer except as determined by REORDER[0] setting
 * |        |          |The suspend interval is determined according to the following equation:
 * |        |          |(SLEEP[3:0] + 2) * period of SCLK
 * |[16]    |IF        |Interrupt Flag
 * |        |          |0 = Indicates the transfer is not finished yet.
 * |        |          |1 = Indicates that the transfer is complete. Interrupt is generated to CPU if enabled.
 * |        |          |NOTE: This bit is cleared by writing 1 to itself.
 * |[17]    |IE        |Interrupt Enable
 * |        |          |0 = Disable SPI Interrupt.
 * |        |          |1 = Enable SPI Interrupt to CPU.
 * |[18]    |SLAVE     |Master Slave Mode Control
 * |        |          |0 = Master mode.
 * |        |          |1 = Slave mode.
 * |[19]    |BYTESLEEP |Insert Sleep interval between Bytes
 * |        |          |This function is only valid for 32bit transfers (TXBITLEN=0)
 * |        |          |If set then a pause of (SLEEP+2) SCLK cycles is inserted between each byte transmitted.
 * |[20]    |BYTEENDIAN|Byte Endian Reorder Function
 * |        |          |This function changes the order of bytes sent/received to be least significant physical byte first. 
 * |[21]    |FIFO      |FIFO Mode
 * |        |          |0 = No FIFO present on transmit and receive buffer.
 * |        |          |1 = Enable FIFO on transmit and receive buffer.
 * |[22]    |TWOB      |Two Bits Transfer Mode
 * |        |          |1 = Enable two-bit transfer mode.
 * |        |          |0 = Disable two-bit transfer mode.
 * |        |          |Note that when enabled in master mode, MOSI data comes from SPI1_TX0 and MOSI data from SPI1_TX1
 * |        |          |Likewise SPI1_RX0 receives bit stream from MISO0 and SPI1_RX1 from MISO1
 * |        |          |Note that when enabled, the setting of TXNUM must be programmed as 0x00
 * |[23]    |VARCLKEN  |Variable Clock Enable (Master Only)
 * |        |          |1 = SCLK output frequency is variable
 * |        |          |The output frequency is determined by the value of VARCLK, DIVIDER, and DIVIDER2.
 * |        |          |0 = The serial clock output frequency is fixed and determined only by the value of DIVIDER.
 * |        |          |Note that when enabled, the setting of TXBITLEN must be programmed as 0x10 (16 bits mode)
 * |[24]    |RXEMPTY   |Receive FIFO EMPTY STATUS
 * |        |          |1 = The receive data FIFO is empty.
 * |        |          |0 = The receive data FIFO is not empty.
 * |[25]    |RXFULL    |Receive FIFO FULL STATUS
 * |        |          |1 = The receive data FIFO is full.
 * |        |          |0 = The receive data FIFO is not full.
 * |[26]    |TXEMPTY   |Transmit FIFO EMPTY STATUS
 * |        |          |1 = The transmit data FIFO is empty.
 * |        |          |0 = The transmit data FIFO is not empty.
 * |[27]    |TXFULL    |Transmit FIFO FULL STATUS
 * |        |          |1 = The transmit data FIFO is full.
 * |        |          |0 = The transmit data FIFO is not full.
 * |[28]    |DMABURST  |Enable DMA Automatic SS function.
 * |        |          |When enabled, interface will automatically generate a SS signal for an entire PDMA access transaction.
 * @var SPI1_T::CLKDIV
 * Offset: 0x04  Clock Divider Register (Master Only)
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |CLKDIV0   |Clock Divider Register (master only)
 * |        |          |The value in this field is the frequency division of the system clock, PCLK, to generate the serial clock on the output SCLK
 * |        |          |The desired frequency is obtained according to the following equation:
 * |        |          |In slave mode, the period of SPI clock driven by a master shall satisfy
 * |        |          |In other words, the maximum frequency of SCLK clock is one fifth of the SPI peripheral clock.
 * |[31:16] |CLKDIV1   |Clock Divider 2 Register (master only)
 * |        |          |The value in this field is the 2nd frequency divider of the system clock, PCLK, to generate the serial clock on the output SCLK
 * |        |          |The desired frequency is obtained according to the following equation:
 * @var SPI1_T::SSCTL
 * Offset: 0x08  Slave Select Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SSR       |Slave Select Register (Master only)
 * |        |          |If ASS bit is cleared, writing 1 to any bit location of this field sets the proper SPISSx0/1 line to an active state and writing 0 sets the line back to inactive state.
 * |        |          |If ASS bit is set, writing 1 to any bit location of this field will select appropriate SPISS line to be automatically driven to active state for the duration of the transmit/receive, and will be driven to inactive state for the rest of the time
 * |        |          |(The active level of SPISSx0/1 is specified in SSLVL).
 * |        |          |Note: SPISS is always defined as device/slave select input signal in slave mode. 
 * |[2]     |SSLVL     |Slave Select Active Level
 * |        |          |It defines the active level of device/slave select signal (SPISSx0/1).
 * |        |          |0 = The slave select signal SPISSx0/1 is active at low-level/falling-edge.
 * |        |          |1 = The slave select signal SPISSx0/1 is active at high-level/rising-edge.
 * |[3]     |ASS       |Automatic Slave Select (Master only)
 * |        |          |0 = If this bit is cleared, slave select signals are asserted and de-asserted by setting and clearing related bits in SSCTL register.
 * |        |          |1 = If this bit is set, SPISS signals are generated automatically
 * |        |          |It means that device/slave select signal, which is set in SSCTL register is asserted by the SPI controller when transmit/receive is started by setting EN, and is de-asserted after each transmit/receive is finished.
 * |[4]     |SSLTRIG   |Slave Select Level Trigger (Slave only)
 * |        |          |0 = The input slave select signal is edge-trigger. This is the default value.
 * |        |          |1 = The slave select signal will be level-trigger
 * |        |          |It depends on SSLVL to decide the signal is active low or active high.
 * |[5]     |LTRIGFLAG |Level Trigger Flag
 * |        |          |When the SSLTRIG bit is set in slave mode, this bit can be read to indicate the received bit number is met the requirement or not.
 * |        |          |1 = The received number and received bits met the requirement which defines in TXNUM and TXBITLEN among one transfer.
 * |        |          |0 = One of the received number and the received bit length doesn't meet the requirement in one transfer.
 * |        |          |Note: This bit is READ only
 * @var SPI1_T::RX0
 * Offset: 0x10  Data Receive Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |RX        |Data Receive Register
 * |        |          |The Data Receive Registers hold the value of received data of the last executed transfer
 * |        |          |Valid bits depend on the transmit bit length field in the SPI1_CTL register
 * |        |          |For example, if TXBITLEN is set to 0x08 and TXNUM is set to 0x0, bit SPI1_RX0[7:0] holds the received data.
 * |        |          |NOTE: The Data Receive Registers are read only registers.
 * @var SPI1_T::RX1
 * Offset: 0x14  Data Receive Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |RX        |Data Receive Register
 * |        |          |The Data Receive Registers hold the value of received data of the last executed transfer
 * |        |          |Valid bits depend on the transmit bit length field in the SPI1_CTL register
 * |        |          |For example, if TXBITLEN is set to 0x08 and TXNUM is set to 0x0, bit SPI1_RX0[7:0] holds the received data.
 * |        |          |NOTE: The Data Receive Registers are read only registers. 
 * @var SPI1_T::TX0
 * Offset: 0x20  Data Transmit Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |TX        |Data Transmit Register
 * |        |          |The Data Transmit Registers hold the data to be transmitted in the next transfer
 * |        |          |Valid bits depend on the transmit bit length field in the SPI1_CTL register
 * |        |          |For example, if TXBITLEN is set to 0x08 and the TXNUM is set to 0x0, the bit SPI1_TX0[7:0] will be transmitted in next transfer
 * |        |          |If TXBITLEN is set to 0x00 and TXNUM is set to 0x1, the core will perform two 32-bit transmit/receive successive using the same setting (the order is SPI1_TX0[31:0], SPI1_TX1[31:0]).
 * @var SPI1_T::TX1
 * Offset: 0x24  Data Transmit Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |TX        |Data Transmit Register
 * |        |          |The Data Transmit Registers hold the data to be transmitted in the next transfer
 * |        |          |Valid bits depend on the transmit bit length field in the SPI1_CTL register
 * |        |          |For example, if TXBITLEN is set to 0x08 and the TXNUM is set to 0x0, the bit SPI1_TX0[7:0] will be transmitted in next transfer
 * |        |          |If TXBITLEN is set to 0x00 and TXNUM is set to 0x1, the core will perform two 32-bit transmit/receive successive using the same setting (the order is SPI1_TX0[31:0], SPI1_TX1[31:0]).
 * @var SPI1_T::VARCLK
 * Offset: 0x34  Variable Clock Pattern Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |VARCLK    |Variable Clock Pattern
 * |        |          |The value in this field is the frequency pattern of the SPI clock
 * |        |          |If the bit field of VARCLK is u20180u2019, the output frequency of SCLK is given by the value of DIVIDER
 * |        |          |If the bit field of VARCLK is u20181u2019, the output frequency of SCLK is given by the value of CLKDIV1
 * |        |          |Refer to register CLKDIV0.
 * |        |          |Refer to Figure 5-62 Variable Serial Clock Frequency paragraph for detailed description.
 * |        |          |Note: Used for CLKP = 0 only, 16 bit transmission.
 * @var SPI1_T::PDMACTL
 * Offset: 0x38  SPI PDMA Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TXMDAEN   |Transmit DMA Start
 * |        |          |1 = Enable
 * |        |          |0 = Disable
 * |        |          |Set this bit to 1 will start the transmit DMA process
 * |        |          |SPI module will issue request to DMA module automatically.
 * |        |          |If using DMA mode to transfer data, remember not to set EN bit of SPI_CTL register
 * |        |          |The DMA controller inside SPI module will set it automatically whenever necessary.
 * |[1]     |RXMDAEN   |Receive DMA Start
 * |        |          |1 = Enable
 * |        |          |0 = Disable
 * |        |          |Set this bit to 1 will start the receive DMA process
 * |        |          |SPI module will issue request to DMA module automatically
 */
    __IO uint32_t CTL;                   /*!< [0x0000] Control and Status Register                                      */
    __IO uint32_t CLKDIV;                /*!< [0x0004] Clock Divider Register (Master Only)                             */
    __IO uint32_t SSCTL;                 /*!< [0x0008] Slave Select Register                                            */
    __I  uint32_t RESERVE0[1];
    __I  uint32_t RX0;                   /*!< [0x0010] Data Receive Register 0                                          */
    __I  uint32_t RX1;                   /*!< [0x0014] Data Receive Register 1                                          */
    __I  uint32_t RESERVE1[2];
    __O  uint32_t TX0;                   /*!< [0x0020] Data Transmit Register 0                                         */
    __O  uint32_t TX1;                   /*!< [0x0024] Data Transmit Register 1                                         */
    __I  uint32_t RESERVE2[3];
    __IO uint32_t VARCLK;                /*!< [0x0034] Variable Clock Pattern Register                                  */
    __IO uint32_t PDMACTL;               /*!< [0x0038] SPI PDMA Control Register                                        */

} SPI1_T;

/**
    @addtogroup SPI1_CONST SPI1 Bit Field Definition
    Constant Definitions for SPI1 Controller
@{ */

#define SPI1_CTL_EN_Pos                  (0)                                               /*!< SPI1_T::CTL: EN Position               */
#define SPI1_CTL_EN_Msk                  (0x1ul << SPI1_CTL_EN_Pos)                        /*!< SPI1_T::CTL: EN Mask                   */

#define SPI1_CTL_RXNEG_Pos               (1)                                               /*!< SPI1_T::CTL: RXNEG Position            */
#define SPI1_CTL_RXNEG_Msk               (0x1ul << SPI1_CTL_RXNEG_Pos)                     /*!< SPI1_T::CTL: RXNEG Mask                */

#define SPI1_CTL_TXNEG_Pos               (2)                                               /*!< SPI1_T::CTL: TXNEG Position            */
#define SPI1_CTL_TXNEG_Msk               (0x1ul << SPI1_CTL_TXNEG_Pos)                     /*!< SPI1_T::CTL: TXNEG Mask                */

#define SPI1_CTL_TXBITLEN_Pos            (3)                                               /*!< SPI1_T::CTL: TXBITLEN Position         */
#define SPI1_CTL_TXBITLEN_Msk            (0x1ful << SPI1_CTL_TXBITLEN_Pos)                 /*!< SPI1_T::CTL: TXBITLEN Mask             */

#define SPI1_CTL_TXNUM_Pos               (8)                                               /*!< SPI1_T::CTL: TXNUM Position            */
#define SPI1_CTL_TXNUM_Msk               (0x3ul << SPI1_CTL_TXNUM_Pos)                     /*!< SPI1_T::CTL: TXNUM Mask                */

#define SPI1_CTL_LSB_Pos                 (10)                                              /*!< SPI1_T::CTL: LSB Position              */
#define SPI1_CTL_LSB_Msk                 (0x1ul << SPI1_CTL_LSB_Pos)                       /*!< SPI1_T::CTL: LSB Mask                  */

#define SPI1_CTL_CLKP_Pos                (11)                                              /*!< SPI1_T::CTL: CLKP Position             */
#define SPI1_CTL_CLKP_Msk                (0x1ul << SPI1_CTL_CLKP_Pos)                      /*!< SPI1_T::CTL: CLKP Mask                 */

#define SPI1_CTL_SLEEP_Pos               (12)                                              /*!< SPI1_T::CTL: SLEEP Position            */
#define SPI1_CTL_SLEEP_Msk               (0xful << SPI1_CTL_SLEEP_Pos)                     /*!< SPI1_T::CTL: SLEEP Mask                */

#define SPI1_CTL_IF_Pos                  (16)                                              /*!< SPI1_T::CTL: IF Position               */
#define SPI1_CTL_IF_Msk                  (0x1ul << SPI1_CTL_IF_Pos)                        /*!< SPI1_T::CTL: IF Mask                   */

#define SPI1_CTL_IE_Pos                  (17)                                              /*!< SPI1_T::CTL: IE Position               */
#define SPI1_CTL_IE_Msk                  (0x1ul << SPI1_CTL_IE_Pos)                        /*!< SPI1_T::CTL: IE Mask                   */

#define SPI1_CTL_SLAVE_Pos               (18)                                              /*!< SPI1_T::CTL: SLAVE Position            */
#define SPI1_CTL_SLAVE_Msk               (0x1ul << SPI1_CTL_SLAVE_Pos)                     /*!< SPI1_T::CTL: SLAVE Mask                */

#define SPI1_CTL_BYTESLEEP_Pos           (19)                                              /*!< SPI1_T::CTL: BYTESLEEP Position        */
#define SPI1_CTL_BYTESLEEP_Msk           (0x1ul << SPI1_CTL_BYTESLEEP_Pos)                 /*!< SPI1_T::CTL: BYTESLEEP Mask            */

#define SPI1_CTL_BYTEENDIAN_Pos          (20)                                              /*!< SPI1_T::CTL: BYTEENDIAN Position       */
#define SPI1_CTL_BYTEENDIAN_Msk          (0x1ul << SPI1_CTL_BYTEENDIAN_Pos)                /*!< SPI1_T::CTL: BYTEENDIAN Mask           */

#define SPI1_CTL_FIFO_Pos                (21)                                              /*!< SPI1_T::CTL: FIFO Position             */
#define SPI1_CTL_FIFO_Msk                (0x1ul << SPI1_CTL_FIFO_Pos)                      /*!< SPI1_T::CTL: FIFO Mask                 */

#define SPI1_CTL_TWOB_Pos                (22)                                              /*!< SPI1_T::CTL: TWOB Position             */
#define SPI1_CTL_TWOB_Msk                (0x1ul << SPI1_CTL_TWOB_Pos)                      /*!< SPI1_T::CTL: TWOB Mask                 */

#define SPI1_CTL_VARCLKEN_Pos            (23)                                              /*!< SPI1_T::CTL: VARCLKEN Position         */
#define SPI1_CTL_VARCLKEN_Msk            (0x1ul << SPI1_CTL_VARCLKEN_Pos)                  /*!< SPI1_T::CTL: VARCLKEN Mask             */

#define SPI1_CTL_RXEMPTY_Pos             (24)                                              /*!< SPI1_T::CTL: RXEMPTY Position          */
#define SPI1_CTL_RXEMPTY_Msk             (0x1ul << SPI1_CTL_RXEMPTY_Pos)                   /*!< SPI1_T::CTL: RXEMPTY Mask              */

#define SPI1_CTL_RXFULL_Pos              (25)                                              /*!< SPI1_T::CTL: RXFULL Position           */
#define SPI1_CTL_RXFULL_Msk              (0x1ul << SPI1_CTL_RXFULL_Pos)                    /*!< SPI1_T::CTL: RXFULL Mask               */

#define SPI1_CTL_TXEMPTY_Pos             (26)                                              /*!< SPI1_T::CTL: TXEMPTY Position          */
#define SPI1_CTL_TXEMPTY_Msk             (0x1ul << SPI1_CTL_TXEMPTY_Pos)                   /*!< SPI1_T::CTL: TXEMPTY Mask              */

#define SPI1_CTL_TXFULL_Pos              (27)                                              /*!< SPI1_T::CTL: TXFULL Position           */
#define SPI1_CTL_TXFULL_Msk              (0x1ul << SPI1_CTL_TXFULL_Pos)                    /*!< SPI1_T::CTL: TXFULL Mask               */

#define SPI1_CTL_DMABURST_Pos            (28)                                              /*!< SPI1_T::CTL: DMABURST Position         */
#define SPI1_CTL_DMABURST_Msk            (0x1ul << SPI1_CTL_DMABURST_Pos)                  /*!< SPI1_T::CTL: DMABURST Mask             */

#define SPI1_CLKDIV_CLKDIV0_Pos          (0)                                               /*!< SPI1_T::CLKDIV: CLKDIV0 Position       */
#define SPI1_CLKDIV_CLKDIV0_Msk          (0xfffful << SPI1_CLKDIV_CLKDIV0_Pos)             /*!< SPI1_T::CLKDIV: CLKDIV0 Mask           */

#define SPI1_CLKDIV_CLKDIV1_Pos          (16)                                              /*!< SPI1_T::CLKDIV: CLKDIV1 Position       */
#define SPI1_CLKDIV_CLKDIV1_Msk          (0xfffful << SPI1_CLKDIV_CLKDIV1_Pos)             /*!< SPI1_T::CLKDIV: CLKDIV1 Mask           */

#define SPI1_SSCTL_SSR_Pos               (0)                                               /*!< SPI1_T::SSCTL: SSR Position            */
#define SPI1_SSCTL_SSR_Msk               (0x1ul << SPI1_SSCTL_SSR_Pos)                     /*!< SPI1_T::SSCTL: SSR Mask                */

#define SPI1_SSCTL_SSLVL_Pos             (2)                                               /*!< SPI1_T::SSCTL: SSLVL Position          */
#define SPI1_SSCTL_SSLVL_Msk             (0x1ul << SPI1_SSCTL_SSLVL_Pos)                   /*!< SPI1_T::SSCTL: SSLVL Mask              */

#define SPI1_SSCTL_ASS_Pos               (3)                                               /*!< SPI1_T::SSCTL: ASS Position            */
#define SPI1_SSCTL_ASS_Msk               (0x1ul << SPI1_SSCTL_ASS_Pos)                     /*!< SPI1_T::SSCTL: ASS Mask                */

#define SPI1_SSCTL_SSLTRIG_Pos           (4)                                               /*!< SPI1_T::SSCTL: SSLTRIG Position        */
#define SPI1_SSCTL_SSLTRIG_Msk           (0x1ul << SPI1_SSCTL_SSLTRIG_Pos)                 /*!< SPI1_T::SSCTL: SSLTRIG Mask            */

#define SPI1_SSCTL_LTRIGFLAG_Pos         (5)                                               /*!< SPI1_T::SSCTL: LTRIGFLAG Position      */
#define SPI1_SSCTL_LTRIGFLAG_Msk         (0x1ul << SPI1_SSCTL_LTRIGFLAG_Pos)               /*!< SPI1_T::SSCTL: LTRIGFLAG Mask          */

#define SPI1_RX0_RX_Pos                  (0)                                               /*!< SPI1_T::RX0: RX Position               */
#define SPI1_RX0_RX_Msk                  (0xfffffffful << SPI1_RX0_RX_Pos)                 /*!< SPI1_T::RX0: RX Mask                   */

#define SPI1_RX1_RX_Pos                  (0)                                               /*!< SPI1_T::RX1: RX Position               */
#define SPI1_RX1_RX_Msk                  (0xfffffffful << SPI1_RX1_RX_Pos)                 /*!< SPI1_T::RX1: RX Mask                   */

#define SPI1_TX0_TX_Pos                  (0)                                               /*!< SPI1_T::TX0: TX Position               */
#define SPI1_TX0_TX_Msk                  (0xfffffffful << SPI1_TX0_TX_Pos)                 /*!< SPI1_T::TX0: TX Mask                   */

#define SPI1_TX1_TX_Pos                  (0)                                               /*!< SPI1_T::TX1: TX Position               */
#define SPI1_TX1_TX_Msk                  (0xfffffffful << SPI1_TX1_TX_Pos)                 /*!< SPI1_T::TX1: TX Mask                   */

#define SPI1_VARCLK_VARCLK_Pos           (0)                                               /*!< SPI1_T::VARCLK: VARCLK Position        */
#define SPI1_VARCLK_VARCLK_Msk           (0xfffffffful << SPI1_VARCLK_VARCLK_Pos)          /*!< SPI1_T::VARCLK: VARCLK Mask            */

#define SPI1_PDMACTL_TXMDAEN_Pos         (0)                                               /*!< SPI1_T::PDMACTL: TXMDAEN Position      */
#define SPI1_PDMACTL_TXMDAEN_Msk         (0x1ul << SPI1_PDMACTL_TXMDAEN_Pos)               /*!< SPI1_T::PDMACTL: TXMDAEN Mask          */

#define SPI1_PDMACTL_RXMDAEN_Pos         (1)                                               /*!< SPI1_T::PDMACTL: RXMDAEN Position      */
#define SPI1_PDMACTL_RXMDAEN_Msk         (0x1ul << SPI1_PDMACTL_RXMDAEN_Pos)               /*!< SPI1_T::PDMACTL: RXMDAEN Mask          */

/**@}*/ /* SPI1_CONST */
/**@}*/ /* end of SPI1 register group */


/*---------------------- System Manger Controller -------------------------*/
/**
    @addtogroup SYS System Manger Controller(SYS)
    Memory Mapped Structure for SYS Controller
@{ */
 
typedef struct
{


/**
 * @var SYS_T::PDID
 * Offset: 0x00  Product ID
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:0]  |PDID      |Product Identifier
 * |        |          |Chip identifier for I91200 series.
 * @var SYS_T::RSTSTS
 * Offset: 0x04  System Reset Source Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CORERSTF  |Reset Source From CORE
 * |        |          |The CORERSTF flag is set if the core has been reset
 * |        |          |Possible sources of reset are a Power-On Reset (POR), RESETn Pin Reset or PMU reset.
 * |        |          |0 = No reset from CORE.
 * |        |          |1 = Core was reset by hardware block.
 * |        |          |This bit is cleared by writing 1 to itself.
 * |[1]     |PADRF     |The RSTS_PAD Flag Is If Pervious Reset Source Originates From the /RESET Pin
 * |        |          |0 = No reset from Pin /RESET.
 * |        |          |1 = Pin /RESET had issued the reset signal to reset the system.
 * |        |          |This bit is cleared by writing 1 to itself.
 * |[2]     |WDTRF     |Reset Source From WDT
 * |        |          |The WDTRF flag is set if pervious reset source originates from the Watch-Dog module.
 * |        |          |0 = No reset from Watch-Dog.
 * |        |          |1 = The Watch-Dog module issued the reset signal to reset the system.
 * |        |          |This bit is cleared by writing 1 to itself.
 * |[3]     |LVRF      |Low Voltage Reset Flag
 * |        |          |The LVRF flag is set if pervious reset source originates from the LVR module.
 * |        |          |0 = No reset from LVR
 * |        |          |1 = The LVR module issued the reset signal to reset the system.
 * |        |          |This bit is cleared by writing 1 to itself.
 * |[5]     |SYSRF     |Reset Source From MCU
 * |        |          |The SYSRF flag is set if the previous reset source originates from the Cortex_M0 kernel.
 * |        |          |0= No reset from MCU.
 * |        |          |1= The Cortex_M0 MCU issued a reset signal to reset the system by software writing 1 to bit SYSRESTREQ(SYSINFO_AIRCTL[2], Application Interrupt and Reset Control Register) in system control registers of Cortex_M0 kernel.
 * |        |          |This bit is cleared by writing 1 to itself.
 * |[6]     |PMURSTF   |Reset Source From PMU
 * |        |          |The PMURSTF flag is set if the PMU.
 * |        |          |0= No reset from PMU.
 * |        |          |1= PMU reset the system from a power down/standby event.
 * |        |          |This bit is cleared by writing 1 to itself.
 * |[7]     |CPURF     |Reset Source From CPU
 * |        |          |The CPURF flag is set by hardware if software writes CPURST (SYS_IPRST0[1]) with a u201C1u201D to reset Cortex-M0 CPU kernel and Flash memory controller (FMC).
 * |        |          |0= No reset from CPU.
 * |        |          |1= The Cortex-M0 CPU kernel and FMC has been reset by software setting CPURST to 1.
 * |        |          |This bit is cleared by writing 1 to itself.
 * |[8]     |WKRSTF    |Wakeup Pin Reset Flag
 * |        |          |The WKRSTF flag is set by hardware if device has powered up from deep power down (DPD) due to action of the WAKEUP pin.
 * |        |          |0=No detected.
 * |        |          |1= A power on was triggered by WAKEUP pin.
 * |        |          |This bit is cleared by writing 1 to itself
 * |        |          |Writing 1 to this bit will clear bits PORF, DPDRSTF, and WKRSTF
 * |[9]     |DPDRSTF   |Deep Power Down Reset Flag
 * |        |          |The DPDRSTF flag is set by hardware if device has powered up due to the DPD timer function.
 * |        |          |0=No detected.
 * |        |          |1= A power on was triggered by DPD timer.
 * |        |          |This bit is cleared by writing 1 to itself
 * |        |          |Writing 1 to this bit will clear bits PORF, DPDRSTF, and WKRSTF
 * |[10]    |PORF      |Power on Reset Flag
 * |        |          |The PORF flag is set by hardware if device has powered up from a power on reset condition or standby power down.
 * |        |          |0=No detected.
 * |        |          |1= A power on Reset has occurred.
 * |        |          |This bit is cleared by writing 1 to itself
 * |        |          |Writing 1 to this bit will clear bits PORF, DPDRSTF, and WKRSTF
 * @var SYS_T::IPRST0
 * Offset: 0x08  IP Reset Control Resister0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CHIPRST   |CHIP One Shot Reset
 * |        |          |Set this bit will reset the whole chip, this bit will automatically return to u201C0u201D after the 2 clock cycles.
 * |        |          |CHIPRST has same behavior as POR reset, all the chip modules are reset and the chip configuration settings from flash are reloaded.
 * |        |          |This bit is a protected bit, to program first issue the unlock sequence
 * |        |          |0= Normal.
 * |        |          |1= Reset CHIP.
 * |[1]     |CPURST    |CPU Kernel One Shot Reset
 * |        |          |Setting this bit will reset the CPU kernel and Flash Memory Controller(FMC), this bit will automatically return to u201C0u201D after the 2 clock cycles
 * |        |          |This bit is a protected bit, to program first issue the unlock sequence
 * |        |          |0= Normal.
 * |        |          |1= Reset CPU.
 * |[2]     |PDMARST   |PDMA Controller Reset
 * |        |          |Set u201C1u201D will generate a reset signal to the PDMA Block
 * |        |          |User needs to set this bit to u201C0u201D to release from the reset state
 * |        |          |0= Normal operation.
 * |        |          |1= PDMA IP reset.
 * @var SYS_T::IPRST1
 * Offset: 0x0C  IP Reset Control Resister1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[6]     |TMR0RST   |Timer0 Controller Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[7]     |TMR1RST   |Timer1 Controller Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[8]     |I2C0RST   |I2C0 Controller Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[11]    |SPI1RST   |SPI1 Controller Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[12]    |SPI0RST   |SPI0 Controller Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[13]    |DPWMRST   |DPWM Speaker Driver Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[16]    |UART0RST  |UART0 Controller Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[17]    |UART1RST  |UART1 Controller Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[18]    |BIQRST    |Biquad Filter Block Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[20]    |PWM0RST   |PWM0 Controller Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[27]    |SARADCRST |SAR ADC Controller Reset,
 * |        |          |0 =Normal Operation.
 * |        |          |1 = Reset,.
 * |[28]    |SDADCRST  |SDADC Controller Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[29]    |I2S0RST   |I2S Controller Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * |[30]    |ANARST    |Analog Block Control Reset
 * |        |          |0=Normal Operation.
 * |        |          |1=Reset.
 * @var SYS_T::GPSMTEN
 * Offset: 0x30  GPIOA/B Input Type Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[16]    |SSGPAG0   |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOA 3/2/1/0 input Schmitt Trigger enabled.
 * |        |          |0 = GPIOA 3/2/1/0 input CMOS enabled.
 * |[17]    |HSSGPAG0  |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOA 3/2/1/0 Output high slew rate.
 * |        |          |0 = GPIOA 3/2/1/0 Output low slew rate.
 * |[18]    |SSGPAG1   |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOA 7/6/5/4 input Schmitt Trigger enabled.
 * |        |          |0 = GPIOA 7/6/5/4 input CMOS enabled.
 * |[19]    |HSSGPAG1  |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOA 7/6/5/4 Output high slew rate.
 * |        |          |0 = GPIOA 7/6/5/4 Output low slew rate.
 * |[20]    |SSGPAG2   |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOA 11/10/9/8 input Schmitt Trigger enabled.
 * |        |          |0 = GPIOA 11/10/9/8 input CMOS enabled.
 * |[21]    |HSSGPAG2  |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOA 11/10/9/8 Output high slew rate.
 * |        |          |0 = GPIOA 11/10/9/8 Output low slew rate.
 * |[22]    |SSGPAG3   |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOA 15/14/13/12 input Schmitt Trigger enabled.
 * |        |          |0 = GPIOA 15/14/13/12 input CMOS enabled.
 * |[23]    |HSSGPAG3  |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOA 15/14/13/12 Output high slew rate.
 * |        |          |0 = GPIOA 15/14/13/12 Output low slew rate.
 * |[24]    |SSGPBG0   |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOB 3/2/1/0 input Schmitt Trigger enabled.
 * |        |          |0 = GPIOB 3/2/1/0 input CMOS enabled.
 * |[25]    |HSSGPBG0  |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOB 3/2/1/0 Output high slew rate.
 * |        |          |0 = GPIOB 3/2/1/0 Output low slew rate.
 * |[26]    |SSGPBG1   |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOB 7/6/5/4 input Schmitt Trigger enabled.
 * |        |          |0 = GPIOB 7/6/5/4 input CMOS enabled.
 * |[27]    |HSSGPBG1  |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOB 7/6/5/4 Output high slew rate.
 * |        |          |0 = GPIOB 7/6/5/4 Output low slew rate.
 * |[28]    |SSGPBG2   |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOB 11/10/9/8 input Schmitt Trigger enabled.
 * |        |          |0 = GPIOB 11/10/9/8 input CMOS enabled.
 * |[29]    |HSSGPBG2  |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOB 11/10/9/8 Output high slew rate.
 * |        |          |0 = GPIOB 11/10/9/8 Output low slew rate.
 * |[30]    |SSGPBG3   |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOB 15/14/13/12 input Schmitt Trigger enabled.
 * |        |          |0 = GPIOB 15/14/13/12 input CMOS enabled.
 * |[31]    |HSSGPBG3  |this Register Controls Whether the GPIO Input Buffer Schmitt Trigger Is Enabled and Whether High or Low Slew Rate Is Selected for Output Dr.
 * |        |          |Each bit controls a group of four GPIO pins
 * |        |          |1 = GPIOB 15/14/13/12 Output high slew rate.
 * |        |          |0 = GPIOB 15/14/13/12 Output low slew rate.
 * @var SYS_T::GPA_MFP
 * Offset: 0x38  GPIOA Multiple Alternate Functions Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |PA0MFP    |Alternate Function Setting for PA0MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI0_MISO1.
 * |        |          |11 = I2S0_FS.
 * |[3:2]   |PA1MFP    |Alternate Function Setting for PA1MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI0_MOSI0.
 * |        |          |11 = I2S0_BCLK.
 * |[5:4]   |PA2MFP    |Alternate Function Setting for PA2MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI0_SCLK0.
 * |        |          |10 = DMIC_DAT.
 * |        |          |11 = I2S0_SDI
 * |[7:6]   |PA3MFP    |Alternate Function Setting for PA3MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI0_SSB0.
 * |        |          |10 = SARADC_TRIG.
 * |        |          |11 = I2S0_SDO.
 * |[9:8]   |PA4MFP    |Alternate Function Setting for PA4MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI0_MISO0.
 * |        |          |10 = UART0_TX.
 * |        |          |11 = SPI1_MOSI
 * |[11:10] |PA5MFP    |Alternate Function Setting for PA5MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI0_MOSI1.
 * |        |          |10 = UART0_RX.
 * |        |          |11 = SPI1_SCLK.
 * |[13:12] |PA6MFP    |Alternate Function Setting for PA6MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = UART0_TX.
 * |        |          |10 = I2C0_SDA.
 * |        |          |11 = SPI1_SSB.
 * |[15:14] |PA7MFP    |Alternate Function Setting for PA7MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = UART0_RX.
 * |        |          |10 = I2C0_SCL.
 * |        |          |11 = SPI1_MISO.
 * |[17:16] |PA8MFP    |Alternate Function Setting for PA8MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = I2C0_SDA.
 * |        |          |10 = UART1_TX.
 * |        |          |11 = UART0_RTSn.
 * |[19:18] |PA9MFP    |Alternate Function Setting for PA9MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = I2C0_SCL.
 * |        |          |10 = UART1_RX.
 * |        |          |11 = UART0_CTSn.
 * |[21:20] |PA10MFP   |Alternate Function Setting for PA10MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = PWM0CH0.
 * |        |          |10 = TM0.
 * |        |          |11 = DPWM_P.
 * |[23:22] |PA11MFP   |Alternate Function Setting for PA11MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = PWM0CH1.
 * |        |          |10 = TM1.
 * |        |          |11 = DPWM_M.
 * |[25:24] |PA12MFP   |Alternate Function Setting for PA12MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = PWM0CH2.
 * |        |          |10 = X12MI.
 * |        |          |11 = I2C0_SDA.
 * |[27:26] |PA13MFP   |Alternate Function Setting for PA13MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = PWM0CH3.
 * |        |          |10 = X12MO.
 * |        |          |11 = I2C0_SCL.
 * |[29:28] |PA14MFP   |Alternate Function Setting for PA14MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = UART1_TX.
 * |        |          |10 = DMIC_CLK.
 * |        |          |11 = X32KI.
 * |[31:30] |PA15MFP   |Alternate Function Setting for PA15MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = UART1_RX.
 * |        |          |10 = MCLK.
 * |        |          |11 = X32KO.
 * @var SYS_T::GPB_MFP
 * Offset: 0x3C  GPIOB Multiple Alternate Functions Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |PB0MFP    |Alternate Function Setting for PB0MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI1_MOSI.
 * |[3:2]   |PB1MFP    |Alternate Function Setting for PB1MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI1_SCLK.
 * |[5:4]   |PB2MFP    |Alternate Function Setting for PB2MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI1_SSB.
 * |[7:6]   |PB3MFP    |Alternate Function Setting for PB3MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI1_MISO.
 * |[9:8]   |PB4MFP    |Alternate Function Setting for PB4MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = I2S0_FS.
 * |[11:10] |PB5MFP    |Alternate Function Setting for PB5MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = I2S0_BCLK.
 * |[13:12] |PB6MFP    |Alternate Function Setting for PB6MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = I2S0_SDI.
 * |[15:14] |PB7MFP    |Alternate Function Setting for PB7MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = I2S0_SDO.
 * |[17:16] |PB8MFP    |Alternate Function Setting for PB8MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = I2C0_SDA
 * |        |          |10 = I2S0_FS.
 * |        |          |11 = UART1_RSTn.
 * |[19:18] |PB9MFP    |Alternate Function Setting for PB9MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = I2C0_SCL.
 * |        |          |10 = I2S0_BCLK.
 * |        |          |11 = UART1_CTsn.
 * |[21:20] |PB10MFP   |Alternate Function Setting for PB10MFP
 * |        |          |00 = GPIO.
 * |        |          |10 = I2S0_SDI.
 * |        |          |11 = UART1_TX.
 * |[23:22] |PB11MFP   |Alternate Function Setting for PB11MFP
 * |        |          |00 = GPIO.
 * |        |          |10 = I2S0_SDO.
 * |        |          |11 = UART1_RX.
 * |[25:24] |PB12MFP   |Alternate Function Setting for PB12MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SP0_MISO1.
 * |        |          |10 = SPI1_MOSI.
 * |        |          |11 = DMIC_DAT.
 * |[27:26] |PB13MFP   |Alternate Function Setting for PB13MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI0_MOSI0.
 * |        |          |10 = SPI1_SCLK.
 * |        |          |11 = SARADC_TRIG.
 * |[29:28] |PB14MFP   |Alternate Function Setting for PB14MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI0_SCLK0.
 * |        |          |10 = SPI1_SSB.
 * |        |          |11 = DMIC_CLK.
 * |[31:30] |PB15MFP   |Alternate Function Setting for PB15MFP
 * |        |          |00 = GPIO.
 * |        |          |01 = SPI0_SSB0
 * |        |          |10 = SPI1_MISO.
 * |        |          |11 = MCLK.
 * @var SYS_T::WKCTL
 * Offset: 0x54  WAKEUP pin control register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * @var SYS_T::REGLCTL
 * Offset: 0x100  Register Lock Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |REGLCTL   |Protected Register Unlock Register
 * |        |          |0 = Protected registers are locked. Any write to the target register is ignored.
 * |        |          |1 = Protected registers are unlocked.
 * @var SYS_T::IRCTCTL
 * Offset: 0x110  Oscillator Frequency Adjustment Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[9:0]   |FREQ      |Current oscillator frequency trim value. (based on CLK_CLKSEL0.HIRCFSEL)
 * |[15]    |RANGE     |1: Low Frequency mode of oscillator active (2MHz).
 * |        |          |0: High frequency mode (20-50MHz)
 * @var SYS_T::OSC10KTRIM
 * Offset: 0x114  10kHz Oscillator (LIRC) Trim Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[22:0]  |TRIM      |23bit trim for LIRC.
 * |[31]    |TRMCLK    |Must be toggled to( from 0 => 1 => 0) load a new OSC10K_TRIM
 * @var SYS_T::OSCTRIM0
 * Offset: 0x118  Internal Oscillator Trim Register 0
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |TRIM      |16bit sign extended representation of 10bit trim.
 * |[31]    |EN2MHZ    |1: Low Frequency mode of oscillator active (2MHz).
 * |        |          |0: High frequency mode (20-50MHz)
 * @var SYS_T::OSCTRIM1
 * Offset: 0x11C  Internal Oscillator Trim Register 1
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |TRIM      |16bit sign extended representation of 10bit trim.
 * |[31]    |EN2MHZ    |1: Low Frequency mode of oscillator active (2MHz).
 * |        |          |0: High frequency mode (20-50MHz)
 * @var SYS_T::OSCTRIM2
 * Offset: 0x120  Internal Oscillator Trim Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |TRIM      |16bit sign extended representation of 10bit trim.
 * |[31]    |EN2MHZ    |1: Low Frequency mode of oscillator active (2MHz).
 * |        |          |0: High frequency mode (20-50MHz)
 * @var SYS_T::XTALTRIM
 * Offset: 0x124  External Crystal Oscillator Trim Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[9]     |LOWPWR    |1: low power mode. 0: normal mode.
 * |[16]    |SELXT     |HXT select external clock
 * |        |          |0: Disable
 * |        |          |1: Enable
 * |[25:24] |XGS       |HXT Gain Select
 * @var SYS_T::Reserved
 * Offset: 0x128  System Reserved, Keep POR Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 */
    __I  uint32_t PDID;                  /*!< [0x0000] Product ID                                                       */
    __IO uint32_t RSTSTS;                /*!< [0x0004] System Reset Source Register                                     */
    __IO uint32_t IPRST0;                /*!< [0x0008] IP Reset Control Resister0                                       */
    __IO uint32_t IPRST1;                /*!< [0x000c] IP Reset Control Resister1                                       */
    __I  uint32_t RESERVE0[8];
    __IO uint32_t GPSMTEN;               /*!< [0x0030] GPIOA/B Input Type Control Register                              */
    __I  uint32_t RESERVE1[1];
    __IO uint32_t GPA_MFP;               /*!< [0x0038] GPIOA Multiple Alternate Functions Control Register              */
    __IO uint32_t GPB_MFP;               /*!< [0x003c] GPIOB Multiple Alternate Functions Control Register              */
    __I  uint32_t RESERVE2[5];
    __IO uint32_t WKCTL;                 /*!< [0x0054] WAKEUP pin control register                                      */
    __I  uint32_t RESERVE3[42];
    __IO uint32_t REGLCTL;               /*!< [0x0100] Register Lock Control                                            */
    __I  uint32_t RESERVE4[3];
    __IO uint32_t IRCTCTL;               /*!< [0x0110] Oscillator Frequency Adjustment Control Register                 */
    __IO uint32_t OSC10KTRIM;            /*!< [0x0114] 10kHz Oscillator (LIRC) Trim Register                            */
    __IO uint32_t OSCTRIM0;              /*!< [0x0118] Internal Oscillator Trim Register 0                              */
    __IO uint32_t OSCTRIM1;              /*!< [0x011c] Internal Oscillator Trim Register 1                              */
    __IO uint32_t OSCTRIM2;              /*!< [0x0120] Internal Oscillator Trim Register 2                              */
    __IO uint32_t XTALTRIM;              /*!< [0x0124] External Crystal Oscillator Trim Register                        */
    __IO uint32_t Reserved;              /*!< [0x0128] System Reserved, Keep POR Value                                  */

} SYS_T;

/**
    @addtogroup SYS_CONST SYS Bit Field Definition
    Constant Definitions for SYS Controller
@{ */

#define SYS_PDID_PDID_Pos                (0)                                               /*!< SYS_T::PDID: PDID Position             */
#define SYS_PDID_PDID_Msk                (0xfffffffful << SYS_PDID_PDID_Pos)               /*!< SYS_T::PDID: PDID Mask                 */

#define SYS_RSTSTS_CORERSTF_Pos          (0)                                               /*!< SYS_T::RSTSTS: CORERSTF Position       */
#define SYS_RSTSTS_CORERSTF_Msk          (0x1ul << SYS_RSTSTS_CORERSTF_Pos)                /*!< SYS_T::RSTSTS: CORERSTF Mask           */

#define SYS_RSTSTS_PADRF_Pos             (1)                                               /*!< SYS_T::RSTSTS: PADRF Position          */
#define SYS_RSTSTS_PADRF_Msk             (0x1ul << SYS_RSTSTS_PADRF_Pos)                   /*!< SYS_T::RSTSTS: PADRF Mask              */

#define SYS_RSTSTS_WDTRF_Pos             (2)                                               /*!< SYS_T::RSTSTS: WDTRF Position          */
#define SYS_RSTSTS_WDTRF_Msk             (0x1ul << SYS_RSTSTS_WDTRF_Pos)                   /*!< SYS_T::RSTSTS: WDTRF Mask              */

#define SYS_RSTSTS_LVRF_Pos              (3)                                               /*!< SYS_T::RSTSTS: LVRF Position           */
#define SYS_RSTSTS_LVRF_Msk              (0x1ul << SYS_RSTSTS_LVRF_Pos)                    /*!< SYS_T::RSTSTS: LVRF Mask               */

#define SYS_RSTSTS_SYSRF_Pos             (5)                                               /*!< SYS_T::RSTSTS: SYSRF Position          */
#define SYS_RSTSTS_SYSRF_Msk             (0x1ul << SYS_RSTSTS_SYSRF_Pos)                   /*!< SYS_T::RSTSTS: SYSRF Mask              */

#define SYS_RSTSTS_PMURSTF_Pos           (6)                                               /*!< SYS_T::RSTSTS: PMURSTF Position        */
#define SYS_RSTSTS_PMURSTF_Msk           (0x1ul << SYS_RSTSTS_PMURSTF_Pos)                 /*!< SYS_T::RSTSTS: PMURSTF Mask            */

#define SYS_RSTSTS_CPURF_Pos             (7)                                               /*!< SYS_T::RSTSTS: CPURF Position          */
#define SYS_RSTSTS_CPURF_Msk             (0x1ul << SYS_RSTSTS_CPURF_Pos)                   /*!< SYS_T::RSTSTS: CPURF Mask              */

#define SYS_RSTSTS_WKRSTF_Pos            (8)                                               /*!< SYS_T::RSTSTS: WKRSTF Position         */
#define SYS_RSTSTS_WKRSTF_Msk            (0x1ul << SYS_RSTSTS_WKRSTF_Pos)                  /*!< SYS_T::RSTSTS: WKRSTF Mask             */

#define SYS_RSTSTS_DPDRSTF_Pos           (9)                                               /*!< SYS_T::RSTSTS: DPDRSTF Position        */
#define SYS_RSTSTS_DPDRSTF_Msk           (0x1ul << SYS_RSTSTS_DPDRSTF_Pos)                 /*!< SYS_T::RSTSTS: DPDRSTF Mask            */

#define SYS_RSTSTS_PORF_Pos              (10)                                              /*!< SYS_T::RSTSTS: PORF Position           */
#define SYS_RSTSTS_PORF_Msk              (0x1ul << SYS_RSTSTS_PORF_Pos)                    /*!< SYS_T::RSTSTS: PORF Mask               */

#define SYS_IPRST0_CHIPRST_Pos           (0)                                               /*!< SYS_T::IPRST0: CHIPRST Position        */
#define SYS_IPRST0_CHIPRST_Msk           (0x1ul << SYS_IPRST0_CHIPRST_Pos)                 /*!< SYS_T::IPRST0: CHIPRST Mask            */

#define SYS_IPRST0_CPURST_Pos            (1)                                               /*!< SYS_T::IPRST0: CPURST Position         */
#define SYS_IPRST0_CPURST_Msk            (0x1ul << SYS_IPRST0_CPURST_Pos)                  /*!< SYS_T::IPRST0: CPURST Mask             */

#define SYS_IPRST0_PDMARST_Pos           (2)                                               /*!< SYS_T::IPRST0: PDMARST Position        */
#define SYS_IPRST0_PDMARST_Msk           (0x1ul << SYS_IPRST0_PDMARST_Pos)                 /*!< SYS_T::IPRST0: PDMARST Mask            */

#define SYS_IPRST1_TMR0RST_Pos           (6)                                               /*!< SYS_T::IPRST1: TMR0RST Position        */
#define SYS_IPRST1_TMR0RST_Msk           (0x1ul << SYS_IPRST1_TMR0RST_Pos)                 /*!< SYS_T::IPRST1: TMR0RST Mask            */

#define SYS_IPRST1_TMR1RST_Pos           (7)                                               /*!< SYS_T::IPRST1: TMR1RST Position        */
#define SYS_IPRST1_TMR1RST_Msk           (0x1ul << SYS_IPRST1_TMR1RST_Pos)                 /*!< SYS_T::IPRST1: TMR1RST Mask            */

#define SYS_IPRST1_I2C0RST_Pos           (8)                                               /*!< SYS_T::IPRST1: I2C0RST Position        */
#define SYS_IPRST1_I2C0RST_Msk           (0x1ul << SYS_IPRST1_I2C0RST_Pos)                 /*!< SYS_T::IPRST1: I2C0RST Mask            */

#define SYS_IPRST1_SPI1RST_Pos           (11)                                              /*!< SYS_T::IPRST1: SPI1RST Position        */
#define SYS_IPRST1_SPI1RST_Msk           (0x1ul << SYS_IPRST1_SPI1RST_Pos)                 /*!< SYS_T::IPRST1: SPI1RST Mask            */

#define SYS_IPRST1_SPI0RST_Pos           (12)                                              /*!< SYS_T::IPRST1: SPI0RST Position        */
#define SYS_IPRST1_SPI0RST_Msk           (0x1ul << SYS_IPRST1_SPI0RST_Pos)                 /*!< SYS_T::IPRST1: SPI0RST Mask            */

#define SYS_IPRST1_DPWMRST_Pos           (13)                                              /*!< SYS_T::IPRST1: DPWMRST Position        */
#define SYS_IPRST1_DPWMRST_Msk           (0x1ul << SYS_IPRST1_DPWMRST_Pos)                 /*!< SYS_T::IPRST1: DPWMRST Mask            */

#define SYS_IPRST1_UART0RST_Pos          (16)                                              /*!< SYS_T::IPRST1: UART0RST Position       */
#define SYS_IPRST1_UART0RST_Msk          (0x1ul << SYS_IPRST1_UART0RST_Pos)                /*!< SYS_T::IPRST1: UART0RST Mask           */

#define SYS_IPRST1_UART1RST_Pos          (17)                                              /*!< SYS_T::IPRST1: UART1RST Position       */
#define SYS_IPRST1_UART1RST_Msk          (0x1ul << SYS_IPRST1_UART1RST_Pos)                /*!< SYS_T::IPRST1: UART1RST Mask           */

#define SYS_IPRST1_BIQRST_Pos            (18)                                              /*!< SYS_T::IPRST1: BIQRST Position         */
#define SYS_IPRST1_BIQRST_Msk            (0x1ul << SYS_IPRST1_BIQRST_Pos)                  /*!< SYS_T::IPRST1: BIQRST Mask             */

#define SYS_IPRST1_PWM0RST_Pos           (20)                                              /*!< SYS_T::IPRST1: PWM0RST Position        */
#define SYS_IPRST1_PWM0RST_Msk           (0x1ul << SYS_IPRST1_PWM0RST_Pos)                 /*!< SYS_T::IPRST1: PWM0RST Mask            */

#define SYS_IPRST1_SARADCRST_Pos         (27)                                              /*!< SYS_T::IPRST1: SARADCRST Position      */
#define SYS_IPRST1_SARADCRST_Msk         (0x1ul << SYS_IPRST1_SARADCRST_Pos)               /*!< SYS_T::IPRST1: SARADCRST Mask          */

#define SYS_IPRST1_SDADCRST_Pos          (28)                                              /*!< SYS_T::IPRST1: SDADCRST Position       */
#define SYS_IPRST1_SDADCRST_Msk          (0x1ul << SYS_IPRST1_SDADCRST_Pos)                /*!< SYS_T::IPRST1: SDADCRST Mask           */

#define SYS_IPRST1_I2S0RST_Pos           (29)                                              /*!< SYS_T::IPRST1: I2S0RST Position        */
#define SYS_IPRST1_I2S0RST_Msk           (0x1ul << SYS_IPRST1_I2S0RST_Pos)                 /*!< SYS_T::IPRST1: I2S0RST Mask            */

#define SYS_IPRST1_ANARST_Pos            (30)                                              /*!< SYS_T::IPRST1: ANARST Position         */
#define SYS_IPRST1_ANARST_Msk            (0x1ul << SYS_IPRST1_ANARST_Pos)                  /*!< SYS_T::IPRST1: ANARST Mask             */

#define SYS_GPSMTEN_SSGPAG0_Pos          (16)                                              /*!< SYS_T::GPSMTEN: SSGPAG0 Position       */
#define SYS_GPSMTEN_SSGPAG0_Msk          (0x1ul << SYS_GPSMTEN_SSGPAG0_Pos)                /*!< SYS_T::GPSMTEN: SSGPAG0 Mask           */

#define SYS_GPSMTEN_HSSGPAG0_Pos         (17)                                              /*!< SYS_T::GPSMTEN: HSSGPAG0 Position      */
#define SYS_GPSMTEN_HSSGPAG0_Msk         (0x1ul << SYS_GPSMTEN_HSSGPAG0_Pos)               /*!< SYS_T::GPSMTEN: HSSGPAG0 Mask          */

#define SYS_GPSMTEN_SSGPAG1_Pos          (18)                                              /*!< SYS_T::GPSMTEN: SSGPAG1 Position       */
#define SYS_GPSMTEN_SSGPAG1_Msk          (0x1ul << SYS_GPSMTEN_SSGPAG1_Pos)                /*!< SYS_T::GPSMTEN: SSGPAG1 Mask           */

#define SYS_GPSMTEN_HSSGPAG1_Pos         (19)                                              /*!< SYS_T::GPSMTEN: HSSGPAG1 Position      */
#define SYS_GPSMTEN_HSSGPAG1_Msk         (0x1ul << SYS_GPSMTEN_HSSGPAG1_Pos)               /*!< SYS_T::GPSMTEN: HSSGPAG1 Mask          */

#define SYS_GPSMTEN_SSGPAG2_Pos          (20)                                              /*!< SYS_T::GPSMTEN: SSGPAG2 Position       */
#define SYS_GPSMTEN_SSGPAG2_Msk          (0x1ul << SYS_GPSMTEN_SSGPAG2_Pos)                /*!< SYS_T::GPSMTEN: SSGPAG2 Mask           */

#define SYS_GPSMTEN_HSSGPAG2_Pos         (21)                                              /*!< SYS_T::GPSMTEN: HSSGPAG2 Position      */
#define SYS_GPSMTEN_HSSGPAG2_Msk         (0x1ul << SYS_GPSMTEN_HSSGPAG2_Pos)               /*!< SYS_T::GPSMTEN: HSSGPAG2 Mask          */

#define SYS_GPSMTEN_SSGPAG3_Pos          (22)                                              /*!< SYS_T::GPSMTEN: SSGPAG3 Position       */
#define SYS_GPSMTEN_SSGPAG3_Msk          (0x1ul << SYS_GPSMTEN_SSGPAG3_Pos)                /*!< SYS_T::GPSMTEN: SSGPAG3 Mask           */

#define SYS_GPSMTEN_HSSGPAG3_Pos         (23)                                              /*!< SYS_T::GPSMTEN: HSSGPAG3 Position      */
#define SYS_GPSMTEN_HSSGPAG3_Msk         (0x1ul << SYS_GPSMTEN_HSSGPAG3_Pos)               /*!< SYS_T::GPSMTEN: HSSGPAG3 Mask          */

#define SYS_GPSMTEN_SSGPBG0_Pos          (24)                                              /*!< SYS_T::GPSMTEN: SSGPBG0 Position       */
#define SYS_GPSMTEN_SSGPBG0_Msk          (0x1ul << SYS_GPSMTEN_SSGPBG0_Pos)                /*!< SYS_T::GPSMTEN: SSGPBG0 Mask           */

#define SYS_GPSMTEN_HSSGPBG0_Pos         (25)                                              /*!< SYS_T::GPSMTEN: HSSGPBG0 Position      */
#define SYS_GPSMTEN_HSSGPBG0_Msk         (0x1ul << SYS_GPSMTEN_HSSGPBG0_Pos)               /*!< SYS_T::GPSMTEN: HSSGPBG0 Mask          */

#define SYS_GPSMTEN_SSGPBG1_Pos          (26)                                              /*!< SYS_T::GPSMTEN: SSGPBG1 Position       */
#define SYS_GPSMTEN_SSGPBG1_Msk          (0x1ul << SYS_GPSMTEN_SSGPBG1_Pos)                /*!< SYS_T::GPSMTEN: SSGPBG1 Mask           */

#define SYS_GPSMTEN_HSSGPBG1_Pos         (27)                                              /*!< SYS_T::GPSMTEN: HSSGPBG1 Position      */
#define SYS_GPSMTEN_HSSGPBG1_Msk         (0x1ul << SYS_GPSMTEN_HSSGPBG1_Pos)               /*!< SYS_T::GPSMTEN: HSSGPBG1 Mask          */

#define SYS_GPSMTEN_SSGPBG2_Pos          (28)                                              /*!< SYS_T::GPSMTEN: SSGPBG2 Position       */
#define SYS_GPSMTEN_SSGPBG2_Msk          (0x1ul << SYS_GPSMTEN_SSGPBG2_Pos)                /*!< SYS_T::GPSMTEN: SSGPBG2 Mask           */

#define SYS_GPSMTEN_HSSGPBG2_Pos         (29)                                              /*!< SYS_T::GPSMTEN: HSSGPBG2 Position      */
#define SYS_GPSMTEN_HSSGPBG2_Msk         (0x1ul << SYS_GPSMTEN_HSSGPBG2_Pos)               /*!< SYS_T::GPSMTEN: HSSGPBG2 Mask          */

#define SYS_GPSMTEN_SSGPBG3_Pos          (30)                                              /*!< SYS_T::GPSMTEN: SSGPBG3 Position       */
#define SYS_GPSMTEN_SSGPBG3_Msk          (0x1ul << SYS_GPSMTEN_SSGPBG3_Pos)                /*!< SYS_T::GPSMTEN: SSGPBG3 Mask           */

#define SYS_GPSMTEN_HSSGPBG3_Pos         (31)                                              /*!< SYS_T::GPSMTEN: HSSGPBG3 Position      */
#define SYS_GPSMTEN_HSSGPBG3_Msk         (0x1ul << SYS_GPSMTEN_HSSGPBG3_Pos)               /*!< SYS_T::GPSMTEN: HSSGPBG3 Mask          */

#define SYS_GPA_MFP_PA0MFP_Pos           (0)                                               /*!< SYS_T::GPA_MFP: PA0MFP Position        */
#define SYS_GPA_MFP_PA0MFP_Msk           (0x3ul << SYS_GPA_MFP_PA0MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA0MFP Mask            */

#define SYS_GPA_MFP_PA1MFP_Pos           (2)                                               /*!< SYS_T::GPA_MFP: PA1MFP Position        */
#define SYS_GPA_MFP_PA1MFP_Msk           (0x3ul << SYS_GPA_MFP_PA1MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA1MFP Mask            */

#define SYS_GPA_MFP_PA2MFP_Pos           (4)                                               /*!< SYS_T::GPA_MFP: PA2MFP Position        */
#define SYS_GPA_MFP_PA2MFP_Msk           (0x3ul << SYS_GPA_MFP_PA2MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA2MFP Mask            */

#define SYS_GPA_MFP_PA3MFP_Pos           (6)                                               /*!< SYS_T::GPA_MFP: PA3MFP Position        */
#define SYS_GPA_MFP_PA3MFP_Msk           (0x3ul << SYS_GPA_MFP_PA3MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA3MFP Mask            */

#define SYS_GPA_MFP_PA4MFP_Pos           (8)                                               /*!< SYS_T::GPA_MFP: PA4MFP Position        */
#define SYS_GPA_MFP_PA4MFP_Msk           (0x3ul << SYS_GPA_MFP_PA4MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA4MFP Mask            */

#define SYS_GPA_MFP_PA5MFP_Pos           (10)                                              /*!< SYS_T::GPA_MFP: PA5MFP Position        */
#define SYS_GPA_MFP_PA5MFP_Msk           (0x3ul << SYS_GPA_MFP_PA5MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA5MFP Mask            */

#define SYS_GPA_MFP_PA6MFP_Pos           (12)                                              /*!< SYS_T::GPA_MFP: PA6MFP Position        */
#define SYS_GPA_MFP_PA6MFP_Msk           (0x3ul << SYS_GPA_MFP_PA6MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA6MFP Mask            */

#define SYS_GPA_MFP_PA7MFP_Pos           (14)                                              /*!< SYS_T::GPA_MFP: PA7MFP Position        */
#define SYS_GPA_MFP_PA7MFP_Msk           (0x3ul << SYS_GPA_MFP_PA7MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA7MFP Mask            */

#define SYS_GPA_MFP_PA8MFP_Pos           (16)                                              /*!< SYS_T::GPA_MFP: PA8MFP Position        */
#define SYS_GPA_MFP_PA8MFP_Msk           (0x3ul << SYS_GPA_MFP_PA8MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA8MFP Mask            */

#define SYS_GPA_MFP_PA9MFP_Pos           (18)                                              /*!< SYS_T::GPA_MFP: PA9MFP Position        */
#define SYS_GPA_MFP_PA9MFP_Msk           (0x3ul << SYS_GPA_MFP_PA9MFP_Pos)                 /*!< SYS_T::GPA_MFP: PA9MFP Mask            */

#define SYS_GPA_MFP_PA10MFP_Pos          (20)                                              /*!< SYS_T::GPA_MFP: PA10MFP Position       */
#define SYS_GPA_MFP_PA10MFP_Msk          (0x3ul << SYS_GPA_MFP_PA10MFP_Pos)                /*!< SYS_T::GPA_MFP: PA10MFP Mask           */

#define SYS_GPA_MFP_PA11MFP_Pos          (22)                                              /*!< SYS_T::GPA_MFP: PA11MFP Position       */
#define SYS_GPA_MFP_PA11MFP_Msk          (0x3ul << SYS_GPA_MFP_PA11MFP_Pos)                /*!< SYS_T::GPA_MFP: PA11MFP Mask           */

#define SYS_GPA_MFP_PA12MFP_Pos          (24)                                              /*!< SYS_T::GPA_MFP: PA12MFP Position       */
#define SYS_GPA_MFP_PA12MFP_Msk          (0x3ul << SYS_GPA_MFP_PA12MFP_Pos)                /*!< SYS_T::GPA_MFP: PA12MFP Mask           */

#define SYS_GPA_MFP_PA13MFP_Pos          (26)                                              /*!< SYS_T::GPA_MFP: PA13MFP Position       */
#define SYS_GPA_MFP_PA13MFP_Msk          (0x3ul << SYS_GPA_MFP_PA13MFP_Pos)                /*!< SYS_T::GPA_MFP: PA13MFP Mask           */

#define SYS_GPA_MFP_PA14MFP_Pos          (28)                                              /*!< SYS_T::GPA_MFP: PA14MFP Position       */
#define SYS_GPA_MFP_PA14MFP_Msk          (0x3ul << SYS_GPA_MFP_PA14MFP_Pos)                /*!< SYS_T::GPA_MFP: PA14MFP Mask           */

#define SYS_GPA_MFP_PA15MFP_Pos          (30)                                              /*!< SYS_T::GPA_MFP: PA15MFP Position       */
#define SYS_GPA_MFP_PA15MFP_Msk          (0x3ul << SYS_GPA_MFP_PA15MFP_Pos)                /*!< SYS_T::GPA_MFP: PA15MFP Mask           */

#define SYS_GPB_MFP_PB0MFP_Pos           (0)                                               /*!< SYS_T::GPB_MFP: PB0MFP Position        */
#define SYS_GPB_MFP_PB0MFP_Msk           (0x3ul << SYS_GPB_MFP_PB0MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB0MFP Mask            */

#define SYS_GPB_MFP_PB1MFP_Pos           (2)                                               /*!< SYS_T::GPB_MFP: PB1MFP Position        */
#define SYS_GPB_MFP_PB1MFP_Msk           (0x3ul << SYS_GPB_MFP_PB1MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB1MFP Mask            */

#define SYS_GPB_MFP_PB2MFP_Pos           (4)                                               /*!< SYS_T::GPB_MFP: PB2MFP Position        */
#define SYS_GPB_MFP_PB2MFP_Msk           (0x3ul << SYS_GPB_MFP_PB2MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB2MFP Mask            */

#define SYS_GPB_MFP_PB3MFP_Pos           (6)                                               /*!< SYS_T::GPB_MFP: PB3MFP Position        */
#define SYS_GPB_MFP_PB3MFP_Msk           (0x3ul << SYS_GPB_MFP_PB3MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB3MFP Mask            */

#define SYS_GPB_MFP_PB4MFP_Pos           (8)                                               /*!< SYS_T::GPB_MFP: PB4MFP Position        */
#define SYS_GPB_MFP_PB4MFP_Msk           (0x3ul << SYS_GPB_MFP_PB4MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB4MFP Mask            */

#define SYS_GPB_MFP_PB5MFP_Pos           (10)                                              /*!< SYS_T::GPB_MFP: PB5MFP Position        */
#define SYS_GPB_MFP_PB5MFP_Msk           (0x3ul << SYS_GPB_MFP_PB5MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB5MFP Mask            */

#define SYS_GPB_MFP_PB6MFP_Pos           (12)                                              /*!< SYS_T::GPB_MFP: PB6MFP Position        */
#define SYS_GPB_MFP_PB6MFP_Msk           (0x3ul << SYS_GPB_MFP_PB6MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB6MFP Mask            */

#define SYS_GPB_MFP_PB7MFP_Pos           (14)                                              /*!< SYS_T::GPB_MFP: PB7MFP Position        */
#define SYS_GPB_MFP_PB7MFP_Msk           (0x3ul << SYS_GPB_MFP_PB7MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB7MFP Mask            */

#define SYS_GPB_MFP_PB8MFP_Pos           (16)                                              /*!< SYS_T::GPB_MFP: PB8MFP Position        */
#define SYS_GPB_MFP_PB8MFP_Msk           (0x3ul << SYS_GPB_MFP_PB8MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB8MFP Mask            */

#define SYS_GPB_MFP_PB9MFP_Pos           (18)                                              /*!< SYS_T::GPB_MFP: PB9MFP Position        */
#define SYS_GPB_MFP_PB9MFP_Msk           (0x3ul << SYS_GPB_MFP_PB9MFP_Pos)                 /*!< SYS_T::GPB_MFP: PB9MFP Mask            */

#define SYS_GPB_MFP_PB10MFP_Pos          (20)                                              /*!< SYS_T::GPB_MFP: PB10MFP Position       */
#define SYS_GPB_MFP_PB10MFP_Msk          (0x3ul << SYS_GPB_MFP_PB10MFP_Pos)                /*!< SYS_T::GPB_MFP: PB10MFP Mask           */

#define SYS_GPB_MFP_PB11MFP_Pos          (22)                                              /*!< SYS_T::GPB_MFP: PB11MFP Position       */
#define SYS_GPB_MFP_PB11MFP_Msk          (0x3ul << SYS_GPB_MFP_PB11MFP_Pos)                /*!< SYS_T::GPB_MFP: PB11MFP Mask           */

#define SYS_GPB_MFP_PB12MFP_Pos          (24)                                              /*!< SYS_T::GPB_MFP: PB12MFP Position       */
#define SYS_GPB_MFP_PB12MFP_Msk          (0x3ul << SYS_GPB_MFP_PB12MFP_Pos)                /*!< SYS_T::GPB_MFP: PB12MFP Mask           */

#define SYS_GPB_MFP_PB13MFP_Pos          (26)                                              /*!< SYS_T::GPB_MFP: PB13MFP Position       */
#define SYS_GPB_MFP_PB13MFP_Msk          (0x3ul << SYS_GPB_MFP_PB13MFP_Pos)                /*!< SYS_T::GPB_MFP: PB13MFP Mask           */

#define SYS_GPB_MFP_PB14MFP_Pos          (28)                                              /*!< SYS_T::GPB_MFP: PB14MFP Position       */
#define SYS_GPB_MFP_PB14MFP_Msk          (0x3ul << SYS_GPB_MFP_PB14MFP_Pos)                /*!< SYS_T::GPB_MFP: PB14MFP Mask           */

#define SYS_GPB_MFP_PB15MFP_Pos          (30)                                              /*!< SYS_T::GPB_MFP: PB15MFP Position       */
#define SYS_GPB_MFP_PB15MFP_Msk          (0x3ul << SYS_GPB_MFP_PB15MFP_Pos)                /*!< SYS_T::GPB_MFP: PB15MFP Mask           */

#define SYS_REGLCTL_REGLCTL_Pos          (0)                                               /*!< SYS_T::REGLCTL: REGLCTL Position       */
#define SYS_REGLCTL_REGLCTL_Msk          (0x1ul << SYS_REGLCTL_REGLCTL_Pos)                /*!< SYS_T::REGLCTL: REGLCTL Mask           */

#define SYS_IRCTCTL_FREQ_Pos             (0)                                               /*!< SYS_T::IRCTCTL: FREQ Position          */
#define SYS_IRCTCTL_FREQ_Msk             (0x3fful << SYS_IRCTCTL_FREQ_Pos)                 /*!< SYS_T::IRCTCTL: FREQ Mask              */

#define SYS_IRCTCTL_RANGE_Pos            (15)                                              /*!< SYS_T::IRCTCTL: RANGE Position         */
#define SYS_IRCTCTL_RANGE_Msk            (0x1ul << SYS_IRCTCTL_RANGE_Pos)                  /*!< SYS_T::IRCTCTL: RANGE Mask             */

#define SYS_OSC10KTRIM_TRIM_Pos          (0)                                               /*!< SYS_T::OSC10KTRIM: TRIM Position       */
#define SYS_OSC10KTRIM_TRIM_Msk          (0x7ffffful << SYS_OSC10KTRIM_TRIM_Pos)           /*!< SYS_T::OSC10KTRIM: TRIM Mask           */

#define SYS_OSC10KTRIM_TRMCLK_Pos        (31)                                              /*!< SYS_T::OSC10KTRIM: TRMCLK Position     */
#define SYS_OSC10KTRIM_TRMCLK_Msk        (0x1ul << SYS_OSC10KTRIM_TRMCLK_Pos)              /*!< SYS_T::OSC10KTRIM: TRMCLK Mask         */

#define SYS_OSCTRIM0_TRIM_Pos            (0)                                               /*!< SYS_T::OSCTRIM0: TRIM Position         */
#define SYS_OSCTRIM0_TRIM_Msk            (0xfffful << SYS_OSCTRIM0_TRIM_Pos)               /*!< SYS_T::OSCTRIM0: TRIM Mask             */

#define SYS_OSCTRIM0_EN2MHZ_Pos          (31)                                              /*!< SYS_T::OSCTRIM0: EN2MHZ Position       */
#define SYS_OSCTRIM0_EN2MHZ_Msk          (0x1ul << SYS_OSCTRIM0_EN2MHZ_Pos)                /*!< SYS_T::OSCTRIM0: EN2MHZ Mask           */

#define SYS_OSCTRIM1_TRIM_Pos            (0)                                               /*!< SYS_T::OSCTRIM1: TRIM Position         */
#define SYS_OSCTRIM1_TRIM_Msk            (0xfffful << SYS_OSCTRIM1_TRIM_Pos)               /*!< SYS_T::OSCTRIM1: TRIM Mask             */

#define SYS_OSCTRIM1_EN2MHZ_Pos          (31)                                              /*!< SYS_T::OSCTRIM1: EN2MHZ Position       */
#define SYS_OSCTRIM1_EN2MHZ_Msk          (0x1ul << SYS_OSCTRIM1_EN2MHZ_Pos)                /*!< SYS_T::OSCTRIM1: EN2MHZ Mask           */

#define SYS_OSCTRIM2_TRIM_Pos            (0)                                               /*!< SYS_T::OSCTRIM2: TRIM Position         */
#define SYS_OSCTRIM2_TRIM_Msk            (0xfffful << SYS_OSCTRIM2_TRIM_Pos)               /*!< SYS_T::OSCTRIM2: TRIM Mask             */

#define SYS_OSCTRIM2_EN2MHZ_Pos          (31)                                              /*!< SYS_T::OSCTRIM2: EN2MHZ Position       */
#define SYS_OSCTRIM2_EN2MHZ_Msk          (0x1ul << SYS_OSCTRIM2_EN2MHZ_Pos)                /*!< SYS_T::OSCTRIM2: EN2MHZ Mask           */

#define SYS_XTALTRIM_LOWPWR_Pos          (9)                                               /*!< SYS_T::XTALTRIM: LOWPWR Position       */
#define SYS_XTALTRIM_LOWPWR_Msk          (0x1ul << SYS_XTALTRIM_LOWPWR_Pos)                /*!< SYS_T::XTALTRIM: LOWPWR Mask           */

#define SYS_XTALTRIM_SELXT_Pos           (16)                                              /*!< SYS_T::XTALTRIM: SELXT Position        */
#define SYS_XTALTRIM_SELXT_Msk           (0x1ul << SYS_XTALTRIM_SELXT_Pos)                 /*!< SYS_T::XTALTRIM: SELXT Mask            */

#define SYS_XTALTRIM_XGS_Pos             (24)                                              /*!< SYS_T::XTALTRIM: XGS Position          */
#define SYS_XTALTRIM_XGS_Msk             (0x3ul << SYS_XTALTRIM_XGS_Pos)                   /*!< SYS_T::XTALTRIM: XGS Mask              */

/**@}*/ /* SYS_CONST */
/**@}*/ /* end of SYS register group */


/*---------------------- System Control Registers -------------------------*/
/**
    @addtogroup SYSINFO System Control Register(SYSINFO)
    Memory Mapped Structure for SYSINFO Controller
@{ */
 
typedef struct
{


/**
 * @var SYSINFO_T::CPUID
 * Offset: 0x00  CPUID Base Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |REVISION  |Revision
 * |        |          |Reads as 0x0
 * |[15:4]  |PARTNO    |Part Number
 * |        |          |Reads as 0xC20.
 * |[19:16] |PART      |ARMv6-m Parts
 * |        |          |Reads as 0xC for ARMv6-M parts
 * |[31:24] |IMPCODE   |Implementer Code Assigned by ARM
 * |        |          |ARM = 0x41.
 * @var SYSINFO_T::ICSR
 * Offset: 0x04  Interrupt Control State Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[8:0]   |VTACT     |Vector Active
 * |        |          |0: Thread mode
 * |        |          |Value > 1: the exception number for the current executing exception.
 * |[20:12] |VTPEND    |Vector Pending
 * |        |          |Indicates the exception number for the highest priority pending exception
 * |        |          |The pending state includes the effect of memory-mapped enable and mask registers
 * |        |          |It does not include the PRIMASK special-purpose register qualifier
 * |        |          |A value of zero indicates no pending exceptions.
 * |[22]    |ISRPEND   |ISR Pending
 * |        |          |Indicates if an external configurable (NVIC generated) interrupt is pending.
 * |[23]    |ISRPREEM  |ISR Preemptive
 * |        |          |If set, a pending exception will be serviced on exit from the debug halt state.
 * |[25]    |PSTKICLR  |Clear a Pending SYST
 * |        |          |Write 1 to clear a pending SYST.
 * |[26]    |PSTKISET  |Set a Pending SYST
 * |        |          |Reads back with current state (1 if Pending, 0 if not).
 * |[27]    |PPSVICLR  |Clear a Pending PendSV Interrupt
 * |        |          |Write 1 to clear a pending PendSV interrupt.
 * |[28]    |PPSVISET  |Set a Pending PendSV Interrupt
 * |        |          |This is normally used to request a context switch
 * |        |          |Reads back with current state (1 if Pending, 0 if not).
 * |[31]    |NMIPNSET  |NMI Pending Set Control
 * |        |          |Setting this bit will activate an NMI
 * |        |          |Since NMI is the highest priority exception, it will activate as soon as it is registered
 * |        |          |Reads back with current state (1 if Pending, 0 if not).
 * @var SYSINFO_T::AIRCTL
 * Offset: 0x0C  Application Interrupt and Reset Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |CLRACTVT  |Clear All Active Vector
 * |        |          |Clears all active state information for fixed and configurable exceptions.
 * |        |          |0= do not clear state information.
 * |        |          |1= clear state information.
 * |        |          |The effect of writing a 1 to this bit if the processor is not halted in Debug, is UNPREDICTABLE.
 * |[2]     |SRSTREQ   |System Reset Request
 * |        |          |0 =do not request a reset.
 * |        |          |1 =request reset.
 * |        |          |Writing 1 to this bit asserts a signal to request a reset by the external system.
 * |[15]    |ENDIANES  |Endianness
 * |        |          |Read Only. Reads 0 indicating little endian machine.
 * |[31:16] |VTKEY     |Vector Key
 * |        |          |The value 0x05FA must be written to this register, otherwise
 * |        |          |a write to register is UNPREDICTABLE.
 * @var SYSINFO_T::SCR
 * Offset: 0x10  System Control Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |SLPONEXC  |Sleep on Exception
 * |        |          |When set to 1, the core can enter a sleep state on an exception return to Thread mode
 * |        |          |This is the mode and exception level entered at reset, the base level of execution
 * |        |          |Setting this bit to 1 enables an interrupt driven application to avoid returning to an empty main application.
 * |[2]     |SLPDEEP   |Controls Whether the Processor Uses Sleep or Deep Sleep As Its Low Power Mode
 * |        |          |0 = sleep.
 * |        |          |1 = deep sleep.
 * |        |          |The SLPDEEP flag is also used in conjunction with CLK_PWRCTL register to enter deeper power-down states than purely core sleep states.
 * |[4]     |SEVNONPN  |Send Event on Pending Bit
 * |        |          |0 = only enabled interrupts or events can wake-up the processor, disabled interrupts are excluded.
 * |        |          |1 = enabled events and all interrupts, including disabled interrupts, can wake-up the processor.
 * |        |          |When enabled, interrupt transitions from Inactive to Pending are included in the list of wakeup events for the WFE instruction.
 * |        |          |When an event or interrupt enters pending state, the event signal wakes up the processor from WFE
 * |        |          |If the processor is not waiting for an event, the event is registered and affects the next WFE.
 * |        |          |The processor also wakes up on execution of an SEV instruction.
 * @var SYSINFO_T::SHPR2
 * Offset: 0x1C  System Handler Priority Register 2
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:30] |PRI11     |Priority of System Handler 11 u2013 SVCall
 * |        |          |u201C0u201D denotes the highest priority and u201C3u201D denotes lowest priority
 * @var SYSINFO_T::SHPR3
 * Offset: 0x20  System Handler Priority Register 3
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:22] |PRI14     |Priority of System Handler 14 u2013 PendSV
 * |        |          |u201C0u201D denotes the highest priority and u201C3u201D denotes lowest priority
 * |[31:30] |PRI15     |Priority of System Handler 15 u2013 SYST
 * |        |          |u201C0u201D denotes the highest priority and u201C3u201D denotes lowest priority
 */
    __I  uint32_t CPUID;                 /*!< [0x0000] CPUID Base Register                                              */
    __IO uint32_t ICSR;                  /*!< [0x0004] Interrupt Control State Register                                 */
    __I  uint32_t RESERVE0[1];
    __IO uint32_t AIRCTL;                /*!< [0x000c] Application Interrupt and Reset Control Register                 */
    __IO uint32_t SCR;                   /*!< [0x0010] System Control Register                                          */
    __I  uint32_t RESERVE1[2];
    __IO uint32_t SHPR2;                 /*!< [0x001c] System Handler Priority Register 2                               */
    __IO uint32_t SHPR3;                 /*!< [0x0020] System Handler Priority Register 3                               */

} SYSINFO_T;

/**
    @addtogroup SYSINFO_CONST SYSINFO Bit Field Definition
    Constant Definitions for SYSINFO Controller
@{ */

#define SYSINFO_CPUID_REVISION_Pos       (0)                                               /*!< SYSINFO_T::CPUID: REVISION Position    */
#define SYSINFO_CPUID_REVISION_Msk       (0xful << SYSINFO_CPUID_REVISION_Pos)             /*!< SYSINFO_T::CPUID: REVISION Mask        */

#define SYSINFO_CPUID_PARTNO_Pos         (4)                                               /*!< SYSINFO_T::CPUID: PARTNO Position      */
#define SYSINFO_CPUID_PARTNO_Msk         (0xffful << SYSINFO_CPUID_PARTNO_Pos)             /*!< SYSINFO_T::CPUID: PARTNO Mask          */

#define SYSINFO_CPUID_PART_Pos           (16)                                              /*!< SYSINFO_T::CPUID: PART Position        */
#define SYSINFO_CPUID_PART_Msk           (0xful << SYSINFO_CPUID_PART_Pos)                 /*!< SYSINFO_T::CPUID: PART Mask            */

#define SYSINFO_CPUID_IMPCODE_Pos        (24)                                              /*!< SYSINFO_T::CPUID: IMPCODE Position     */
#define SYSINFO_CPUID_IMPCODE_Msk        (0xfful << SYSINFO_CPUID_IMPCODE_Pos)             /*!< SYSINFO_T::CPUID: IMPCODE Mask         */

#define SYSINFO_ICSR_VTACT_Pos           (0)                                               /*!< SYSINFO_T::ICSR: VTACT Position        */
#define SYSINFO_ICSR_VTACT_Msk           (0x1fful << SYSINFO_ICSR_VTACT_Pos)               /*!< SYSINFO_T::ICSR: VTACT Mask            */

#define SYSINFO_ICSR_VTPEND_Pos          (12)                                              /*!< SYSINFO_T::ICSR: VTPEND Position       */
#define SYSINFO_ICSR_VTPEND_Msk          (0x1fful << SYSINFO_ICSR_VTPEND_Pos)              /*!< SYSINFO_T::ICSR: VTPEND Mask           */

#define SYSINFO_ICSR_ISRPEND_Pos         (22)                                              /*!< SYSINFO_T::ICSR: ISRPEND Position      */
#define SYSINFO_ICSR_ISRPEND_Msk         (0x1ul << SYSINFO_ICSR_ISRPEND_Pos)               /*!< SYSINFO_T::ICSR: ISRPEND Mask          */

#define SYSINFO_ICSR_ISRPREEM_Pos        (23)                                              /*!< SYSINFO_T::ICSR: ISRPREEM Position     */
#define SYSINFO_ICSR_ISRPREEM_Msk        (0x1ul << SYSINFO_ICSR_ISRPREEM_Pos)              /*!< SYSINFO_T::ICSR: ISRPREEM Mask         */

#define SYSINFO_ICSR_PSTKICLR_Pos        (25)                                              /*!< SYSINFO_T::ICSR: PSTKICLR Position     */
#define SYSINFO_ICSR_PSTKICLR_Msk        (0x1ul << SYSINFO_ICSR_PSTKICLR_Pos)              /*!< SYSINFO_T::ICSR: PSTKICLR Mask         */

#define SYSINFO_ICSR_PSTKISET_Pos        (26)                                              /*!< SYSINFO_T::ICSR: PSTKISET Position     */
#define SYSINFO_ICSR_PSTKISET_Msk        (0x1ul << SYSINFO_ICSR_PSTKISET_Pos)              /*!< SYSINFO_T::ICSR: PSTKISET Mask         */

#define SYSINFO_ICSR_PPSVICLR_Pos        (27)                                              /*!< SYSINFO_T::ICSR: PPSVICLR Position     */
#define SYSINFO_ICSR_PPSVICLR_Msk        (0x1ul << SYSINFO_ICSR_PPSVICLR_Pos)              /*!< SYSINFO_T::ICSR: PPSVICLR Mask         */

#define SYSINFO_ICSR_PPSVISET_Pos        (28)                                              /*!< SYSINFO_T::ICSR: PPSVISET Position     */
#define SYSINFO_ICSR_PPSVISET_Msk        (0x1ul << SYSINFO_ICSR_PPSVISET_Pos)              /*!< SYSINFO_T::ICSR: PPSVISET Mask         */

#define SYSINFO_ICSR_NMIPNSET_Pos        (31)                                              /*!< SYSINFO_T::ICSR: NMIPNSET Position     */
#define SYSINFO_ICSR_NMIPNSET_Msk        (0x1ul << SYSINFO_ICSR_NMIPNSET_Pos)              /*!< SYSINFO_T::ICSR: NMIPNSET Mask         */

#define SYSINFO_AIRCTL_CLRACTVT_Pos      (1)                                               /*!< SYSINFO_T::AIRCTL: CLRACTVT Position   */
#define SYSINFO_AIRCTL_CLRACTVT_Msk      (0x1ul << SYSINFO_AIRCTL_CLRACTVT_Pos)            /*!< SYSINFO_T::AIRCTL: CLRACTVT Mask       */

#define SYSINFO_AIRCTL_SRSTREQ_Pos       (2)                                               /*!< SYSINFO_T::AIRCTL: SRSTREQ Position    */
#define SYSINFO_AIRCTL_SRSTREQ_Msk       (0x1ul << SYSINFO_AIRCTL_SRSTREQ_Pos)             /*!< SYSINFO_T::AIRCTL: SRSTREQ Mask        */

#define SYSINFO_AIRCTL_ENDIANES_Pos      (15)                                              /*!< SYSINFO_T::AIRCTL: ENDIANES Position   */
#define SYSINFO_AIRCTL_ENDIANES_Msk      (0x1ul << SYSINFO_AIRCTL_ENDIANES_Pos)            /*!< SYSINFO_T::AIRCTL: ENDIANES Mask       */

#define SYSINFO_AIRCTL_VTKEY_Pos         (16)                                              /*!< SYSINFO_T::AIRCTL: VTKEY Position      */
#define SYSINFO_AIRCTL_VTKEY_Msk         (0xfffful << SYSINFO_AIRCTL_VTKEY_Pos)            /*!< SYSINFO_T::AIRCTL: VTKEY Mask          */

#define SYSINFO_SCR_SLPONEXC_Pos         (1)                                               /*!< SYSINFO_T::SCR: SLPONEXC Position      */
#define SYSINFO_SCR_SLPONEXC_Msk         (0x1ul << SYSINFO_SCR_SLPONEXC_Pos)               /*!< SYSINFO_T::SCR: SLPONEXC Mask          */

#define SYSINFO_SCR_SLPDEEP_Pos          (2)                                               /*!< SYSINFO_T::SCR: SLPDEEP Position       */
#define SYSINFO_SCR_SLPDEEP_Msk          (0x1ul << SYSINFO_SCR_SLPDEEP_Pos)                /*!< SYSINFO_T::SCR: SLPDEEP Mask           */

#define SYSINFO_SCR_SEVNONPN_Pos         (4)                                               /*!< SYSINFO_T::SCR: SEVNONPN Position      */
#define SYSINFO_SCR_SEVNONPN_Msk         (0x1ul << SYSINFO_SCR_SEVNONPN_Pos)               /*!< SYSINFO_T::SCR: SEVNONPN Mask          */

#define SYSINFO_SHPR2_PRI11_Pos          (30)                                              /*!< SYSINFO_T::SHPR2: PRI11 Position       */
#define SYSINFO_SHPR2_PRI11_Msk          (0x3ul << SYSINFO_SHPR2_PRI11_Pos)                /*!< SYSINFO_T::SHPR2: PRI11 Mask           */

#define SYSINFO_SHPR3_PRI14_Pos          (22)                                              /*!< SYSINFO_T::SHPR3: PRI14 Position       */
#define SYSINFO_SHPR3_PRI14_Msk          (0x3ul << SYSINFO_SHPR3_PRI14_Pos)                /*!< SYSINFO_T::SHPR3: PRI14 Mask           */

#define SYSINFO_SHPR3_PRI15_Pos          (30)                                              /*!< SYSINFO_T::SHPR3: PRI15 Position       */
#define SYSINFO_SHPR3_PRI15_Msk          (0x3ul << SYSINFO_SHPR3_PRI15_Pos)                /*!< SYSINFO_T::SHPR3: PRI15 Mask           */

/**@}*/ /* SYSINFO_CONST */
/**@}*/ /* end of SYSINFO register group */


/*---------------------- System Timer -------------------------*/
/**
    @addtogroup SYSTICK System Timer(SYSTICK)
    Memory Mapped Structure for SYSTICK Controller
@{ */
 
typedef struct
{


/**
 * @var SYSTICK_T::CSR
 * Offset: 0x10  SysTick Control and Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |ENABLE    |ENABLE
 * |        |          |0 = The counter is disabled
 * |        |          |1 = The counter will operate in a multi-shot manner.
 * |[1]     |TICKINT   |Enables SysTick Exception Request
 * |        |          |0 = Counting down to 0 does not cause the SysTick exception to be pended
 * |        |          |Software can use COUNTFLAG to determine if a count to zero has occurred.
 * |        |          |1 = Counting down to 0 will cause SysTick exception to be pended
 * |        |          |Clearing the SysTick Current Value register by a register write in software will not cause SysTick to be pended.
 * |[2]     |CLKSRC    |Clock Source
 * |        |          |0= Core clock unused.
 * |        |          |1= Core clock used for SysTick, this bit will read as 1 and ignore writes.
 * |[16]    |COUNTFLAG |Count Flag
 * |        |          |Returns 1 if timer counted to 0 since last time this register was read.
 * |        |          |0= Cleared on read or by a write to the Current Value register.
 * |        |          |1= Set by a count transition from 1 to 0.
 * @var SYSTICK_T::RVR
 * Offset: 0x14  SysTick Reload Value Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |RELOAD    |SysTick Reload
 * |        |          |Value to load into the Current Value register when the counter reaches 0.
 * |        |          |To generate a multi-shot timer with a period of N processor clock cycles, use a RELOAD value of N-1
 * |        |          |For example, if the SysTick interrupt is required every 200 clock pulses, set RELOAD to 199.
 * @var SYSTICK_T::CVR
 * Offset: 0x18  SysTick Current Value Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |CURRENT   |Current Counter Value
 * |        |          |This is the value of the counter at the time it is sampled
 * |        |          |The counter does not provide read-modify-write protection
 * |        |          |The register is write-clear
 * |        |          |A software write of any value will clear the register to 0 and also clear the COUNTFLAG bit.
 */
    __I  uint32_t RESERVE0[4];
    __IO uint32_t CSR;                   /*!< [0x0010] SysTick Control and Status Register                              */
    __IO uint32_t RVR;                   /*!< [0x0014] SysTick Reload Value Register                                    */
    __IO uint32_t CVR;                   /*!< [0x0018] SysTick Current Value Register                                   */

} SYSTICK_T;

/**
    @addtogroup SYSTICK_CONST SYSTICK Bit Field Definition
    Constant Definitions for SYSTICK Controller
@{ */

#define SYSTICK_CSR_ENABLE_Pos           (0)                                               /*!< SYSTICK_T::CSR: ENABLE Position        */
#define SYSTICK_CSR_ENABLE_Msk           (0x1ul << SYSTICK_CSR_ENABLE_Pos)                 /*!< SYSTICK_T::CSR: ENABLE Mask            */

#define SYSTICK_CSR_TICKINT_Pos          (1)                                               /*!< SYSTICK_T::CSR: TICKINT Position       */
#define SYSTICK_CSR_TICKINT_Msk          (0x1ul << SYSTICK_CSR_TICKINT_Pos)                /*!< SYSTICK_T::CSR: TICKINT Mask           */

#define SYSTICK_CSR_CLKSRC_Pos           (2)                                               /*!< SYSTICK_T::CSR: CLKSRC Position        */
#define SYSTICK_CSR_CLKSRC_Msk           (0x1ul << SYSTICK_CSR_CLKSRC_Pos)                 /*!< SYSTICK_T::CSR: CLKSRC Mask            */

#define SYSTICK_CSR_COUNTFLAG_Pos        (16)                                              /*!< SYSTICK_T::CSR: COUNTFLAG Position     */
#define SYSTICK_CSR_COUNTFLAG_Msk        (0x1ul << SYSTICK_CSR_COUNTFLAG_Pos)              /*!< SYSTICK_T::CSR: COUNTFLAG Mask         */

#define SYSTICK_RVR_RELOAD_Pos           (0)                                               /*!< SYSTICK_T::RVR: RELOAD Position        */
#define SYSTICK_RVR_RELOAD_Msk           (0xfffffful << SYSTICK_RVR_RELOAD_Pos)            /*!< SYSTICK_T::RVR: RELOAD Mask            */

#define SYSTICK_CVR_CURRENT_Pos          (0)                                               /*!< SYSTICK_T::CVR: CURRENT Position       */
#define SYSTICK_CVR_CURRENT_Msk          (0xfffffful << SYSTICK_CVR_CURRENT_Pos)           /*!< SYSTICK_T::CVR: CURRENT Mask           */

/**@}*/ /* SYSTICK_CONST */
/**@}*/ /* end of SYSTICK register group */


/*---------------------- Timer Controller -------------------------*/
/**
    @addtogroup TMR Timer Controller(TMR)
    Memory Mapped Structure for TMR Controller
@{ */
 
typedef struct
{


/**
 * @var TMR_T::CTL
 * Offset: 0x00  Timer Control and Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |PSC       |Pre-scale Counter
 * |        |          |Clock input is divided by PSC+1 before it is fed to the counter. If PSC = 0, then there is no scaling.
 * |[16]    |CNTDATEN  |Data Latch Enable
 * |        |          |When CNTDATEN is set, TIMERx_CNT (Timer Data Register) will be updated continuously with the 24-bit up-counter value as the timer is counting.
 * |        |          |1 = Timer Data Register update enable.
 * |        |          |0 = Timer Data Register update disable. 
 * |[25]    |ACTSTS    |Timer Active Status Bit (Read Only)
 * |        |          |This bit indicates the counter status of timer.
 * |        |          |0 = Timer is not active.
 * |        |          |1 = Timer is active.
 * |[26]    |RSTCNT    |Counter Reset Bit
 * |        |          |Set this bit will reset the timer counter, pre-scale and also force CNTEN to 0.
 * |        |          |0 = No effect.
 * |        |          |1 = Reset Timeru2019s pre-scale counter, internal 24-bit up-counter and CNTEN bit.
 * |[28:27] |OPMODE    |Timer Operating Mode
 * |        |          |0 = The timer is operating in the one-shot mode
 * |        |          |The associated interrupt signal is generated once (if INTEN is enabled) and CNTEN is automatically cleared by hardware.
 * |        |          |1 = The timer is operating in the periodic mode
 * |        |          |The associated interrupt signal is generated periodically (if INTEN is enabled).
 * |        |          |2 = Reserved.
 * |        |          |3 = The timer is operating in continuous counting mode
 * |        |          |The associated interrupt signal is generated when CNT = TIMERx_CMP (if INTEN is enabled); however, the 24-bit up-counter counts continuously without reset.
 * |[29]    |INTEN     |Interrupt Enable Bit
 * |        |          |0 = Disable TIMER Interrupt.
 * |        |          |1 = Enable TIMER Interrupt.
 * |        |          |If timer interrupt is enabled, the timer asserts its interrupt signal when the count is equal to TIMERx_CMP.
 * |[30]    |CNTEN     |Counter Enable Bit
 * |        |          |0 = Stops/Suspends counting.
 * |        |          |1 = Starts counting.
 * |        |          |Note1: Setting CNTEN = 1 enables 24-bit counter. It continues count from last value.
 * |        |          |Note2: This bit is auto-cleared by hardware in one-shot mode (OPMODE = 00b) when the timer interrupt is generated (INTEN = 1b).
 * @var TMR_T::CMP
 * Offset: 0x04  Timer Compare Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[24:0]  |CMPDAT    |Timer Comparison Value
 * |        |          |CMPDAT is a 24-bit comparison register
 * |        |          |When the 24-bit up-counter is enabled and its value is equal to CMPDAT value, a Timer Interrupt is requested if the timer interrupt is enabled with TIMERx_CTL.INTEN = 1
 * |        |          |The CMPDAT value defines the timer cycle time.
 * |        |          |Time out period = (Period of timer clock input) * (8-bit PSC + 1) * (24-bit CMPDAT).
 * |        |          |NOTE1: Never set CMPDAT to 0x000 or 0x001. Timer will not function correctly.
 * |        |          |NOTE2: Regardless of CEN state, whenever a new value is written to this register, TIMER will restart counting using this new value and abort previous count.
 * @var TMR_T::INTSTS
 * Offset: 0x08  Timer Interrupt Status Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |TIF       |Timer Interrupt Flag
 * |        |          |This bit indicates the interrupt status of Timer.
 * |        |          |TIF bit is set by hardware when the 24-bit counter matches the timer comparison value (CMPDAT)
 * |        |          |It is cleared by writing 1.
 * @var TMR_T::CNT
 * Offset: 0x0C  Timer Data Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |CNT       |Timer Data Register
 * |        |          |When TIMERx_CTL.CNTDATEN is set to 1, the internal 24-bit timer up-counter value will be latched into CNT
 * |        |          |User can read this register for the up-counter value.
 */
    __IO uint32_t CTL;                   /*!< [0x0000] Timer Control and Status Register                                */
    __IO uint32_t CMP;                   /*!< [0x0004] Timer Compare Register                                           */
    __IO uint32_t INTSTS;                /*!< [0x0008] Timer Interrupt Status Register                                  */
    __IO uint32_t CNT;                   /*!< [0x000c] Timer Data Register                                              */

} TMR_T;

/**
    @addtogroup TMR_CONST TMR Bit Field Definition
    Constant Definitions for TMR Controller
@{ */

#define TMR_CTL_PSC_Pos                  (0)                                               /*!< TMR_T::CTL: PSC Position               */
#define TMR_CTL_PSC_Msk                  (0xfful << TMR_CTL_PSC_Pos)                       /*!< TMR_T::CTL: PSC Mask                   */

#define TMR_CTL_CNTDATEN_Pos             (16)                                              /*!< TMR_T::CTL: CNTDATEN Position          */
#define TMR_CTL_CNTDATEN_Msk             (0x1ul << TMR_CTL_CNTDATEN_Pos)                   /*!< TMR_T::CTL: CNTDATEN Mask              */

#define TMR_CTL_ACTSTS_Pos               (25)                                              /*!< TMR_T::CTL: ACTSTS Position            */
#define TMR_CTL_ACTSTS_Msk               (0x1ul << TMR_CTL_ACTSTS_Pos)                     /*!< TMR_T::CTL: ACTSTS Mask                */

#define TMR_CTL_RSTCNT_Pos               (26)                                              /*!< TMR_T::CTL: RSTCNT Position            */
#define TMR_CTL_RSTCNT_Msk               (0x1ul << TMR_CTL_RSTCNT_Pos)                     /*!< TMR_T::CTL: RSTCNT Mask                */

#define TMR_CTL_OPMODE_Pos               (27)                                              /*!< TMR_T::CTL: OPMODE Position            */
#define TMR_CTL_OPMODE_Msk               (0x3ul << TMR_CTL_OPMODE_Pos)                     /*!< TMR_T::CTL: OPMODE Mask                */

#define TMR_CTL_INTEN_Pos                (29)                                              /*!< TMR_T::CTL: INTEN Position             */
#define TMR_CTL_INTEN_Msk                (0x1ul << TMR_CTL_INTEN_Pos)                      /*!< TMR_T::CTL: INTEN Mask                 */

#define TMR_CTL_CNTEN_Pos                (30)                                              /*!< TMR_T::CTL: CNTEN Position             */
#define TMR_CTL_CNTEN_Msk                (0x1ul << TMR_CTL_CNTEN_Pos)                      /*!< TMR_T::CTL: CNTEN Mask                 */

#define TMR_CMP_CMPDAT_Pos               (0)                                               /*!< TMR_T::CMP: CMPDAT Position            */
#define TMR_CMP_CMPDAT_Msk               (0x1fffffful << TMR_CMP_CMPDAT_Pos)               /*!< TMR_T::CMP: CMPDAT Mask                */

#define TMR_INTSTS_TIF_Pos               (0)                                               /*!< TMR_T::INTSTS: TIF Position            */
#define TMR_INTSTS_TIF_Msk               (0x1ul << TMR_INTSTS_TIF_Pos)                     /*!< TMR_T::INTSTS: TIF Mask                */

#define TMR_CNT_CNT_Pos                  (0)                                               /*!< TMR_T::CNT: CNT Position               */
#define TMR_CNT_CNT_Msk                  (0xfffffful << TMR_CNT_CNT_Pos)                   /*!< TMR_T::CNT: CNT Mask                   */

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
 * @var UART_T::DAT
 * Offset: 0x00  UART Receive/Transfer FIFO Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[7:0]   |DAT       |Receive FIFO Register
 * |        |          |Reading this register will return data from the receive data FIFO
 * |        |          |By reading this register, the UART will return the 8-bit data received from Rx pin (LSB first).
 * @var UART_T::INTEN
 * Offset: 0x04  UART Interrupt Enable Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RDAIEN    |Receive Data Available Interrupt Enable
 * |        |          |0 = Mask off RDAINT.
 * |        |          |1 = Enable RDAINT.
 * |[1]     |THREIEN   |Transmit FIFO Register Empty Interrupt Enable
 * |        |          |0 = Mask off THERINT.
 * |        |          |1 = Enable THERINT.
 * |[2]     |RLSIEN    |Receive Line Status Interrupt Enable
 * |        |          |0 = Mask off RLSINT.
 * |        |          |1 = Enable RLSINT.
 * |[3]     |MODEMIEN  |Modem Status Interrupt Enable
 * |        |          |0 = Mask off MODEMINT.
 * |        |          |1 = Enable MODEMINT.
 * |[4]     |RXTOIEN   |Receive Time Out Interrupt Enable
 * |        |          |0 = Mask off RXTOINT.
 * |        |          |1 = Enable RXTOINT.
 * |[5]     |BUFERRIEN |Buffer Error Interrupt Enable
 * |        |          |0 = Mask off BUFERRINT.
 * |        |          |1 = Enable IBUFERRINT.
 * |[8]     |LINIEN    |LIN RX Break Field Detected Interrupt Enable
 * |        |          |0 = Mask off Lin bus Rx break field interrupt.
 * |        |          |1 = Enable Lin bus Rx break field interrupt.
 * |[11]    |TOCNTEN   |Time-out Counter Enable
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
 * @var UART_T::FIFO
 * Offset: 0x08  UART FIFO Control Register.
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
 * |        |          |1 = Writing 1 to this bit will reset the transmit internal state machine and pointers.
 * |        |          |Note: This bit will auto-clear after 3 UART engine clock cycles.
 * |[7:4]   |RFITL     |Receive FIFO Interrupt (RDAINT) Trigger Level
 * |        |          |When the number of bytes in the receive FIFO equals the RFITL then the RDAIF will be set and, if enabled, an RDAINT interrupt will generated.
 * |        |          |Value : INTR_RDA Trigger Level (Bytes)
 * |        |          |0 : 1
 * |        |          |1 : 4[TV1]
 * |        |          |[TV1]Not designed for 8 bytes u2013 removed from spec
 * |[19:16] |RTSTRGLV  |RTS Trigger Level for Auto-flow Control
 * |        |          |Sets the FIFO trigger level when auto-flow control will de-assert RTS (request-to-send).
 * |        |          |Value : Trigger Level (Bytes)
 * |        |          |0 : 1
 * |        |          |1 : 4
 * |        |          |2 : 8
 * @var UART_T::LINE
 * Offset: 0x0C  UART Line Control Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1:0]   |WLS       |Word Length Select
 * |        |          |0 (5bits), 1(6bits), 2(7bits), 3(8bits)
 * |[2]     |NSB       |Number of STOP Bits
 * |        |          |0= One u201CSTOP bitu201D is generated after the transmitted data.
 * |        |          |1= Two u201CSTOP bitsu201D are generated when 6-, 7- and 8-bit word length is selected; One and a half u201CSTOP bitsu201D are generated in the transmitted data when 5-bit word length is selected
 * |[3]     |PBE       |Parity Bit Enable
 * |        |          |0 = Parity bit is not generated (transmit data) or checked (receive data) during transfer.
 * |        |          |1 = Parity bit is generated or checked between the "last data word bit" and "stop bit" of the serial data.
 * |[4]     |EPE       |Even Parity Enable
 * |        |          |0 = Odd number of logic 1u2019s are transmitted or checked in the data word and parity bits.
 * |        |          |1 = Even number of logic 1u2019s are transmitted or checked in the data word and parity bits.
 * |        |          |This bit has effect only when PBE (parity bit enable) is set.
 * |[5]     |SPE       |Stick Parity Enable
 * |        |          |0 = Disable stick parity.
 * |        |          |1 = When bits PBE and SPE are set u2018Stick Parityu2019 is enabled
 * |        |          |If EPE=0 the parity bit is transmitted and checked as always set, if EPE=1, the parity bit is transmitted and checked as always cleared
 * |[6]     |BCB       |Break Control Bit
 * |        |          |When this bit is set to logic 1, the serial data output (Tx) is forced to the u2018Spaceu2019 state (logic 0)
 * |        |          |Normal condition is serial data output is u2018Marku2019 state
 * |        |          |This bit acts only on Tx and has no effect on the transmitter logic.
 * @var UART_T::MODEM
 * Offset: 0x10  UART Modem Control Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |RTS       |RTS (Request-to-send) Signal
 * |        |          |If UART_INTEN.ATORTSEN = 0, this bit controls whether RTS pin is active or not.
 * |        |          |0 = Drive RTS inactive ( = ~RTSACTLV).
 * |        |          |1 = Drive RTS active ( = RTSACTLV).
 * |[4]     |LBMEN     |Loopback Mode Enable
 * |        |          |0=Disable.
 * |        |          |1=Enable.
 * |[9]     |RTSACTLV  |Request-to-send (RTS) Active Trigger Level
 * |        |          |This bit can change the RTS trigger level.
 * |        |          |0= RTS is active low level.
 * |        |          |1= RTS is active high level.
 * |[13]    |RTSSTS    |RTS Pin State (Read Only)
 * |        |          |This bit is the pin status of RTS.
 * @var UART_T::MODEMSTS
 * Offset: 0x14  UART Modem Status Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |CTSDETF   |Detect CTS State Change Flag
 * |        |          |This bit is set whenever CTS input has state change
 * |        |          |It will generate Modem interrupt to CPU when UART_INTEN.MODEMIEN = 1.
 * |        |          |NOTE: This bit is cleared by writing 1 to itself.
 * |[4]     |CTSSTS    |CTS Pin Status (Read Only)
 * |        |          |This bit is the pin status of CTS. 
 * |[8]     |CTSACTLV  |Clear-to-send (CTS) Active Trigger Level
 * |        |          |This bit can change the CTS trigger level.
 * |        |          |0= CTS is active low level.
 * |        |          |1= CTS is active high level. 
 * @var UART_T::FIFOSTS
 * Offset: 0x18  UART FIFO Status Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RXOVIF    |Rx Overflow Error Interrupt Flag
 * |        |          |If the Rx FIFO (UART_DAT) is full, and an additional byte is received by the UART, an overflow condition will occur and set this bit to logic 1
 * |        |          |It will also generate a BUFERRIF event and interrupt if enabled.
 * |        |          |NOTE: This bit is cleared by writing 1 to itself.
 * |[4]     |PEF       |Parity Error Flag
 * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "parity bit", and is reset whenever the CPU writes 1 to this bit.
 * |[5]     |FEF       |Framing Error Flag
 * |        |          |This bit is set to logic 1 whenever the received character does not have a valid "stop bit" (that is, the stop bit following the last data bit or parity bit is detected as a logic 0), and is reset whenever the CPU writes 1 to this bit.
 * |[6]     |BIF       |Break Interrupt Flag
 * |        |          |This bit is set to a logic 1 whenever the receive data input (Rx) is held in the "spaceu201D state (logic 0) for longer than a full word transmission time (that is, the total time of start bit + data bits + parity + stop bits)
 * |        |          |It is reset whenever the CPU writes 1 to this bit.
 * |[13:8]  |RXPTR     |Rx FIFO Pointer (Read Only)
 * |        |          |This field returns the Rx FIFO buffer pointer
 * |        |          |It is the number of bytes available for read in the Rx FIFO
 * |        |          |When UART receives one byte from external device, RXPTR is incremented
 * |        |          |When one byte of Rx FIFO is read by CPU, RXPTR is decremented
 * |[14]    |RXEMPTY   |Receive FIFO Empty (Read Only)
 * |        |          |This bit indicates whether the Rx FIFO is empty or not.
 * |        |          |When the last byte of Rx FIFO has been read by CPU, hardware sets this bit high
 * |        |          |It will be cleared when UART receives any new data.
 * |[15]    |RXFULL    |Receive FIFO Full (Read Only)
 * |        |          |This bit indicates whether the Rx FIFO is full or not.
 * |        |          |This bit is set when Rx FIFO is full; otherwise it is cleared by hardware.
 * |[21:16] |TXPTR     |Tx FIFO Pointer (Read Only)
 * |        |          |This field returns the Tx FIFO buffer pointer
 * |        |          |When CPU writes a byte into the Tx FIFO, TXPTR is incremented
 * |        |          |When a byte from Tx FIFO is transferred to the Transmit Shift Register, TXPTR is decremented.
 * |[22]    |TXEMPTY   |Transmit FIFO Empty (Read Only)
 * |        |          |This bit indicates whether the Tx FIFO is empty or not.
 * |        |          |When the last byte of Tx FIFO has been transferred to Transmitter Shift Register, hardware sets this bit high
 * |        |          |It will be cleared after writing data to FIFO (Tx FIFO not empty).
 * |[23]    |TXFULL    |Transmit FIFO Full (Read Only)
 * |        |          |This bit indicates whether the Tx FIFO is full or not.
 * |        |          |This bit is set when Tx FIFO is full; otherwise it is cleared by hardware
 * |        |          |TXFULL=0 indicates there is room to write more data to Tx FIFO.
 * |[24]    |TXOVIF    |Tx Overflow Error Interrupt Flag
 * |        |          |If the Tx FIFO (UART_DAT) is full, an additional write to UART_DAT will cause an overflow condition and set this bit to logic 1
 * |        |          |It will also generate a BUFERRIF event and interrupt if enabled.
 * |        |          |NOTE: This bit is cleared by writing 1 to itself.
 * |[28]    |TXEMPTYF  |Transmitter Empty (Read Only)
 * |        |          |Bit is set by hardware when Tx FIFO is empty and the STOP bit of the last byte has been transmitted.
 * |        |          |Bit is cleared automatically when Tx FIFO is not empty or the last byte transmission has not completed.
 * |        |          |NOTE: This bit is read only. 
 * @var UART_T::INTSTS
 * Offset: 0x1C  UART Interrupt Status Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |RDAIF     |Receive Data Available Interrupt Flag (Read Only)
 * |        |          |When the number of bytes in the Rx FIFO equals UART_FIFO.RFITL then the RDAIF will be set
 * |        |          |If UART_INTEN.RDAIEN is enabled, the RDA interrupt will be generated.
 * |        |          |NOTE: This bit is read only and it will be cleared when the number of unread bytes of Rx FIFO drops below the threshold level (RFITL).
 * |[1]     |THREIF    |Transmit Holding Register Empty Interrupt Flag (Read Only)
 * |        |          |This bit is set when the last data of Tx FIFO is transferred to Transmitter Shift Register
 * |        |          |If UART_INTEN.THREIEN is enabled, the THRE interrupt will be generated.
 * |        |          |NOTE: This bit is read only and it will be cleared when writing data into the Tx FIFO.
 * |[2]     |RLSIF     |Receive Line Status Interrupt Flag (Read Only)
 * |        |          |This bit is set when the Rx receive data has a parity, framing or break error (at least one of, UART_FIFOSTS.BIF, UART_FIFOSTS.FEF and UART_FIFOSTS.PEF, is set)
 * |        |          |If UART_INTEN.RLSIEN is enabled, the RLS interrupt will be generated.
 * |        |          |NOTE: This bit is read only and reset to 0 when all bits of BIF, FEF and PEF are cleared.
 * |[3]     |MODENIF   |MODEM Interrupt Flag (Read Only)
 * |        |          |This bit is set when the CTS pin has changed state (UART_MODEMSTS.CTSDETF=1)
 * |        |          |If UART_INTEN.MODEMIEN is enabled, a CPU interrupt request will be generated.
 * |        |          |NOTE: This bit is read only and reset when bit UART_MODEMSTS.CTSDETF is cleared by a write 1.
 * |[4]     |RXTOIF    |Time Out Interrupt Flag (Read Only)
 * |        |          |This bit is set when the Rx FIFO is not empty and no activity occurs in the Rx FIFO and the time out counter equal to TOIC
 * |        |          |If UART_INTEN.TOUT_IEN is enabled a CPU interrupt request will be generated.
 * |        |          |NOTE: This bit is read only and user can read FIFO to clear it.
 * |[5]     |BUFERRIF  |Buffer Error Interrupt Flag (Read Only)
 * |        |          |This bit is set when either the Tx or Rx FIFO overflows (UART_FIFOSTS.TXOVIF or UART_FIFOSTS.RXOVIF is set)
 * |        |          |When BUFERRIF is set, the serial transfer may be corrupted
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
 * |        |          |This bit is set when the Rx receive data has a parity, framing or break error (at least one of, UART_FIFOSTS.BIF, UART_FIFOSTS.FEF and UART_FIFOSTS.PEF, is set)
 * |        |          |If UART_INTEN.RLSIEN is enabled, the RLS interrupt will be generated.
 * |        |          |NOTE: This bit is read only and reset to 0 when all bits of BIF, FEF and PEF are cleared.
 * |[19]    |DMODEMIF  |DMA MODE MODEM Interrupt Flag (Read Only)
 * |        |          |This bit is set when the CTS pin has changed state (UART_MODEMSTS.CTSDETF=1)
 * |        |          |If UART_INTEN.MODEMIEN is enabled, a CPU interrupt request will be generated.
 * |        |          |NOTE: This bit is read only and reset when bit UART_MODEMSTS.CTSDETF is cleared by a write 1.
 * |[20]    |DRXTOIF   |DMA MODE Time Out Interrupt Flag (Read Only)
 * |        |          |This bit is set when the Rx FIFO is not empty and no activity occurs in the Rx FIFO and the time out counter equal to TOIC
 * |        |          |If UART_INTEN.TOUT_IEN is enabled a CPU interrupt request will be generated.
 * |        |          |NOTE: This bit is read only and user can read FIFO to clear it.
 * |[21]    |DBERRIF   |DMA MODE Buffer Error Interrupt Flag (Read Only)
 * |        |          |This bit is set when either the Tx or Rx FIFO overflows (UART_FIFOSTS.TXOVIF or UART_FIFOSTS.RXOVIF is set)
 * |        |          |When BUFERRIF is set, the serial transfer may be corrupted
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
 * @var UART_T::TOUT
 * Offset: 0x20  UART Time Out Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[6:0]   |TOIC      |Time Out Interrupt Comparator
 * |        |          |The time out counter resets and starts counting whenever the Rx FIFO receives a new data word
 * |        |          |Once the content of time out counter is equal to that of time out interrupt comparator (TOIC), a receiver time out interrupt (RXTOINT) is generated if UART_INTEN.RXTOIEN is set
 * |        |          |A new incoming data word or RX FIFO empty clears RXTOIF
 * |        |          |The period of the time out counter is the baud rate.
 * @var UART_T::BAUD
 * Offset: 0x24  UART Baud Rate Divisor Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[15:0]  |BRD       |Baud Rate Divider
 * |        |          |Refer to Table 5-116 UART Baud Rate Setting Table for more information.
 * |[27:24] |EDIVM1    |Divider x
 * |        |          |The baud rate divider M = EDIVM1+1.
 * |[28]    |BAUDM0    |Divider X Equal 1
 * |        |          |0: M = EDIVM1+1, with restriction EDIVM1 >= 8.
 * |        |          |1: M = 1, with restriction BRD[15:0] >= 3.
 * |        |          |Refer to Table 5-116 UART Baud Rate Setting Table for more information.
 * |[29]    |BAUDM1    |Divider X Enable
 * |        |          |The baud rate equation is:  Baud Rate = UART_CLK / [ M * (BRD + 2) ] ; The default value of M is 16.
 * |        |          |0 = Disable divider X ( M = 16).
 * |        |          |1 = Enable divider X (M = EDIVM1+1, with EDIVM1 >= 8).
 * |        |          |Refer to Table 5-116 UART Baud Rate Setting Table for more information.
 * |        |          |NOTE: When in IrDA mode, this bit must disabled.
 * @var UART_T::IRDA
 * Offset: 0x28  UART IrDA Control Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[1]     |TXEN      |Transmit/Receive Selection
 * |        |          |0=Enable IrDA receiver.
 * |        |          |1= Enable IrDA transmitter.
 * |[2]     |LOOPBACK  |IrDA Loopback Test Mode
 * |        |          |Loopback Tx to Rx.
 * |[5]     |TXINV     |Transmit Inversion Enable
 * |        |          |0= No inversion.
 * |        |          |1= Invert Tx output signal.
 * |[6]     |RXINV     |Receive Inversion Enable
 * |        |          |0= No inversion.
 * |        |          |1= Invert Rx input signal.
 * @var UART_T::ALTCTL
 * Offset: 0x2C  UART LIN Control Register.
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |BRKFL     |UART LIN Break Field Length Count
 * |        |          |This field indicates a 4-bit LIN Tx break field count.
 * |        |          |NOTE: This break field length is BRKFL + 2
 * |[6]     |LINRXEN   |LIN RX Enable
 * |        |          |0 = Disable LIN Rx mode.
 * |        |          |1 = Enable LIN Rx mode.
 * |[7]     |LINTXEN   |LIN TX Break Mode Enable
 * |        |          |0 = Disable LIN Tx Break Mode.
 * |        |          |1 = Enable LIN Tx Break Mode.
 * |        |          |NOTE: When Tx break field transfer operation finished, this bit will be cleared automatically.
 * @var UART_T::FUNCSEL
 * Offset: 0x30  UART Function Select Register.
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
    __IO uint32_t DAT;                   /*!< [0x0000] UART Receive/Transfer FIFO Register.                             */
    __IO uint32_t INTEN;                 /*!< [0x0004] UART Interrupt Enable Register.                                  */
    __IO uint32_t FIFO;                  /*!< [0x0008] UART FIFO Control Register.                                      */
    __IO uint32_t LINE;                  /*!< [0x000c] UART Line Control Register.                                      */
    __IO uint32_t MODEM;                 /*!< [0x0010] UART Modem Control Register.                                     */
    __IO uint32_t MODEMSTS;              /*!< [0x0014] UART Modem Status Register.                                      */
    __IO uint32_t FIFOSTS;               /*!< [0x0018] UART FIFO Status Register.                                       */
    __IO uint32_t INTSTS;                /*!< [0x001c] UART Interrupt Status Register.                                  */
    __IO uint32_t TOUT;                  /*!< [0x0020] UART Time Out Register                                           */
    __IO uint32_t BAUD;                  /*!< [0x0024] UART Baud Rate Divisor Register                                  */
    __IO uint32_t IRDA;                  /*!< [0x0028] UART IrDA Control Register.                                      */
    __IO uint32_t ALTCTL;                /*!< [0x002c] UART LIN Control Register.                                       */
    __IO uint32_t FUNCSEL;               /*!< [0x0030] UART Function Select Register.                                   */

} UART_T;

/**
    @addtogroup UART_CONST UART Bit Field Definition
    Constant Definitions for UART Controller
@{ */

#define UART_DAT_DAT_Pos                 (0)                                               /*!< UART_T::DAT: DAT Position              */
#define UART_DAT_DAT_Msk                 (0xfful << UART_DAT_DAT_Pos)                      /*!< UART_T::DAT: DAT Mask                  */

#define UART_INTEN_RDAIEN_Pos            (0)                                               /*!< UART_T::INTEN: RDAIEN Position         */
#define UART_INTEN_RDAIEN_Msk            (0x1ul << UART_INTEN_RDAIEN_Pos)                  /*!< UART_T::INTEN: RDAIEN Mask             */

#define UART_INTEN_THREIEN_Pos           (1)                                               /*!< UART_T::INTEN: THREIEN Position        */
#define UART_INTEN_THREIEN_Msk           (0x1ul << UART_INTEN_THREIEN_Pos)                 /*!< UART_T::INTEN: THREIEN Mask            */

#define UART_INTEN_RLSIEN_Pos            (2)                                               /*!< UART_T::INTEN: RLSIEN Position         */
#define UART_INTEN_RLSIEN_Msk            (0x1ul << UART_INTEN_RLSIEN_Pos)                  /*!< UART_T::INTEN: RLSIEN Mask             */

#define UART_INTEN_MODEMIEN_Pos          (3)                                               /*!< UART_T::INTEN: MODEMIEN Position       */
#define UART_INTEN_MODEMIEN_Msk          (0x1ul << UART_INTEN_MODEMIEN_Pos)                /*!< UART_T::INTEN: MODEMIEN Mask           */

#define UART_INTEN_RXTOIEN_Pos           (4)                                               /*!< UART_T::INTEN: RXTOIEN Position        */
#define UART_INTEN_RXTOIEN_Msk           (0x1ul << UART_INTEN_RXTOIEN_Pos)                 /*!< UART_T::INTEN: RXTOIEN Mask            */

#define UART_INTEN_BUFERRIEN_Pos         (5)                                               /*!< UART_T::INTEN: BUFERRIEN Position      */
#define UART_INTEN_BUFERRIEN_Msk         (0x1ul << UART_INTEN_BUFERRIEN_Pos)               /*!< UART_T::INTEN: BUFERRIEN Mask          */

#define UART_INTEN_LINIEN_Pos            (8)                                               /*!< UART_T::INTEN: LINIEN Position         */
#define UART_INTEN_LINIEN_Msk            (0x1ul << UART_INTEN_LINIEN_Pos)                  /*!< UART_T::INTEN: LINIEN Mask             */

#define UART_INTEN_TOCNTEN_Pos           (11)                                              /*!< UART_T::INTEN: TOCNTEN Position        */
#define UART_INTEN_TOCNTEN_Msk           (0x1ul << UART_INTEN_TOCNTEN_Pos)                 /*!< UART_T::INTEN: TOCNTEN Mask            */

#define UART_INTEN_ATORTSEN_Pos          (12)                                              /*!< UART_T::INTEN: ATORTSEN Position       */
#define UART_INTEN_ATORTSEN_Msk          (0x1ul << UART_INTEN_ATORTSEN_Pos)                /*!< UART_T::INTEN: ATORTSEN Mask           */

#define UART_INTEN_ATOCTSEN_Pos          (13)                                              /*!< UART_T::INTEN: ATOCTSEN Position       */
#define UART_INTEN_ATOCTSEN_Msk          (0x1ul << UART_INTEN_ATOCTSEN_Pos)                /*!< UART_T::INTEN: ATOCTSEN Mask           */

#define UART_INTEN_DMATXEN_Pos           (14)                                              /*!< UART_T::INTEN: DMATXEN Position        */
#define UART_INTEN_DMATXEN_Msk           (0x1ul << UART_INTEN_DMATXEN_Pos)                 /*!< UART_T::INTEN: DMATXEN Mask            */

#define UART_INTEN_DMARXEN_Pos           (15)                                              /*!< UART_T::INTEN: DMARXEN Position        */
#define UART_INTEN_DMARXEN_Msk           (0x1ul << UART_INTEN_DMARXEN_Pos)                 /*!< UART_T::INTEN: DMARXEN Mask            */

#define UART_FIFO_RXRST_Pos              (1)                                               /*!< UART_T::FIFO: RXRST Position           */
#define UART_FIFO_RXRST_Msk              (0x1ul << UART_FIFO_RXRST_Pos)                    /*!< UART_T::FIFO: RXRST Mask               */

#define UART_FIFO_TXRST_Pos              (2)                                               /*!< UART_T::FIFO: TXRST Position           */
#define UART_FIFO_TXRST_Msk              (0x1ul << UART_FIFO_TXRST_Pos)                    /*!< UART_T::FIFO: TXRST Mask               */

#define UART_FIFO_RFITL_Pos              (4)                                               /*!< UART_T::FIFO: RFITL Position           */
#define UART_FIFO_RFITL_Msk              (0xful << UART_FIFO_RFITL_Pos)                    /*!< UART_T::FIFO: RFITL Mask               */

#define UART_FIFO_RTSTRGLV_Pos           (16)                                              /*!< UART_T::FIFO: RTSTRGLV Position        */
#define UART_FIFO_RTSTRGLV_Msk           (0xful << UART_FIFO_RTSTRGLV_Pos)                 /*!< UART_T::FIFO: RTSTRGLV Mask            */

#define UART_LINE_WLS_Pos                (0)                                               /*!< UART_T::LINE: WLS Position             */
#define UART_LINE_WLS_Msk                (0x3ul << UART_LINE_WLS_Pos)                      /*!< UART_T::LINE: WLS Mask                 */

#define UART_LINE_NSB_Pos                (2)                                               /*!< UART_T::LINE: NSB Position             */
#define UART_LINE_NSB_Msk                (0x1ul << UART_LINE_NSB_Pos)                      /*!< UART_T::LINE: NSB Mask                 */

#define UART_LINE_PBE_Pos                (3)                                               /*!< UART_T::LINE: PBE Position             */
#define UART_LINE_PBE_Msk                (0x1ul << UART_LINE_PBE_Pos)                      /*!< UART_T::LINE: PBE Mask                 */

#define UART_LINE_EPE_Pos                (4)                                               /*!< UART_T::LINE: EPE Position             */
#define UART_LINE_EPE_Msk                (0x1ul << UART_LINE_EPE_Pos)                      /*!< UART_T::LINE: EPE Mask                 */

#define UART_LINE_SPE_Pos                (5)                                               /*!< UART_T::LINE: SPE Position             */
#define UART_LINE_SPE_Msk                (0x1ul << UART_LINE_SPE_Pos)                      /*!< UART_T::LINE: SPE Mask                 */

#define UART_LINE_BCB_Pos                (6)                                               /*!< UART_T::LINE: BCB Position             */
#define UART_LINE_BCB_Msk                (0x1ul << UART_LINE_BCB_Pos)                      /*!< UART_T::LINE: BCB Mask                 */

#define UART_MODEM_RTS_Pos               (1)                                               /*!< UART_T::MODEM: RTS Position            */
#define UART_MODEM_RTS_Msk               (0x1ul << UART_MODEM_RTS_Pos)                     /*!< UART_T::MODEM: RTS Mask                */

#define UART_MODEM_LBMEN_Pos             (4)                                               /*!< UART_T::MODEM: LBMEN Position          */
#define UART_MODEM_LBMEN_Msk             (0x1ul << UART_MODEM_LBMEN_Pos)                   /*!< UART_T::MODEM: LBMEN Mask              */

#define UART_MODEM_RTSACTLV_Pos          (9)                                               /*!< UART_T::MODEM: RTSACTLV Position       */
#define UART_MODEM_RTSACTLV_Msk          (0x1ul << UART_MODEM_RTSACTLV_Pos)                /*!< UART_T::MODEM: RTSACTLV Mask           */

#define UART_MODEM_RTSSTS_Pos            (13)                                              /*!< UART_T::MODEM: RTSSTS Position         */
#define UART_MODEM_RTSSTS_Msk            (0x1ul << UART_MODEM_RTSSTS_Pos)                  /*!< UART_T::MODEM: RTSSTS Mask             */

#define UART_MODEMSTS_CTSDETF_Pos        (0)                                               /*!< UART_T::MODEMSTS: CTSDETF Position     */
#define UART_MODEMSTS_CTSDETF_Msk        (0x1ul << UART_MODEMSTS_CTSDETF_Pos)              /*!< UART_T::MODEMSTS: CTSDETF Mask         */

#define UART_MODEMSTS_CTSSTS_Pos         (4)                                               /*!< UART_T::MODEMSTS: CTSSTS Position      */
#define UART_MODEMSTS_CTSSTS_Msk         (0x1ul << UART_MODEMSTS_CTSSTS_Pos)               /*!< UART_T::MODEMSTS: CTSSTS Mask          */

#define UART_MODEMSTS_CTSACTLV_Pos       (8)                                               /*!< UART_T::MODEMSTS: CTSACTLV Position    */
#define UART_MODEMSTS_CTSACTLV_Msk       (0x1ul << UART_MODEMSTS_CTSACTLV_Pos)             /*!< UART_T::MODEMSTS: CTSACTLV Mask        */

#define UART_FIFOSTS_RXOVIF_Pos          (0)                                               /*!< UART_T::FIFOSTS: RXOVIF Position       */
#define UART_FIFOSTS_RXOVIF_Msk          (0x1ul << UART_FIFOSTS_RXOVIF_Pos)                /*!< UART_T::FIFOSTS: RXOVIF Mask           */

#define UART_FIFOSTS_PEF_Pos             (4)                                               /*!< UART_T::FIFOSTS: PEF Position          */
#define UART_FIFOSTS_PEF_Msk             (0x1ul << UART_FIFOSTS_PEF_Pos)                   /*!< UART_T::FIFOSTS: PEF Mask              */

#define UART_FIFOSTS_FEF_Pos             (5)                                               /*!< UART_T::FIFOSTS: FEF Position          */
#define UART_FIFOSTS_FEF_Msk             (0x1ul << UART_FIFOSTS_FEF_Pos)                   /*!< UART_T::FIFOSTS: FEF Mask              */

#define UART_FIFOSTS_BIF_Pos             (6)                                               /*!< UART_T::FIFOSTS: BIF Position          */
#define UART_FIFOSTS_BIF_Msk             (0x1ul << UART_FIFOSTS_BIF_Pos)                   /*!< UART_T::FIFOSTS: BIF Mask              */

#define UART_FIFOSTS_RXPTR_Pos           (8)                                               /*!< UART_T::FIFOSTS: RXPTR Position        */
#define UART_FIFOSTS_RXPTR_Msk           (0x3ful << UART_FIFOSTS_RXPTR_Pos)                /*!< UART_T::FIFOSTS: RXPTR Mask            */

#define UART_FIFOSTS_RXEMPTY_Pos         (14)                                              /*!< UART_T::FIFOSTS: RXEMPTY Position      */
#define UART_FIFOSTS_RXEMPTY_Msk         (0x1ul << UART_FIFOSTS_RXEMPTY_Pos)               /*!< UART_T::FIFOSTS: RXEMPTY Mask          */

#define UART_FIFOSTS_RXFULL_Pos          (15)                                              /*!< UART_T::FIFOSTS: RXFULL Position       */
#define UART_FIFOSTS_RXFULL_Msk          (0x1ul << UART_FIFOSTS_RXFULL_Pos)                /*!< UART_T::FIFOSTS: RXFULL Mask           */

#define UART_FIFOSTS_TXPTR_Pos           (16)                                              /*!< UART_T::FIFOSTS: TXPTR Position        */
#define UART_FIFOSTS_TXPTR_Msk           (0x3ful << UART_FIFOSTS_TXPTR_Pos)                /*!< UART_T::FIFOSTS: TXPTR Mask            */

#define UART_FIFOSTS_TXEMPTY_Pos         (22)                                              /*!< UART_T::FIFOSTS: TXEMPTY Position      */
#define UART_FIFOSTS_TXEMPTY_Msk         (0x1ul << UART_FIFOSTS_TXEMPTY_Pos)               /*!< UART_T::FIFOSTS: TXEMPTY Mask          */

#define UART_FIFOSTS_TXFULL_Pos          (23)                                              /*!< UART_T::FIFOSTS: TXFULL Position       */
#define UART_FIFOSTS_TXFULL_Msk          (0x1ul << UART_FIFOSTS_TXFULL_Pos)                /*!< UART_T::FIFOSTS: TXFULL Mask           */

#define UART_FIFOSTS_TXOVIF_Pos          (24)                                              /*!< UART_T::FIFOSTS: TXOVIF Position       */
#define UART_FIFOSTS_TXOVIF_Msk          (0x1ul << UART_FIFOSTS_TXOVIF_Pos)                /*!< UART_T::FIFOSTS: TXOVIF Mask           */

#define UART_FIFOSTS_TXEMPTYF_Pos        (28)                                              /*!< UART_T::FIFOSTS: TXEMPTYF Position     */
#define UART_FIFOSTS_TXEMPTYF_Msk        (0x1ul << UART_FIFOSTS_TXEMPTYF_Pos)              /*!< UART_T::FIFOSTS: TXEMPTYF Mask         */

#define UART_INTSTS_RDAIF_Pos            (0)                                               /*!< UART_T::INTSTS: RDAIF Position         */
#define UART_INTSTS_RDAIF_Msk            (0x1ul << UART_INTSTS_RDAIF_Pos)                  /*!< UART_T::INTSTS: RDAIF Mask             */

#define UART_INTSTS_THREIF_Pos           (1)                                               /*!< UART_T::INTSTS: THREIF Position        */
#define UART_INTSTS_THREIF_Msk           (0x1ul << UART_INTSTS_THREIF_Pos)                 /*!< UART_T::INTSTS: THREIF Mask            */

#define UART_INTSTS_RLSIF_Pos            (2)                                               /*!< UART_T::INTSTS: RLSIF Position         */
#define UART_INTSTS_RLSIF_Msk            (0x1ul << UART_INTSTS_RLSIF_Pos)                  /*!< UART_T::INTSTS: RLSIF Mask             */

#define UART_INTSTS_MODENIF_Pos          (3)                                               /*!< UART_T::INTSTS: MODENIF Position       */
#define UART_INTSTS_MODENIF_Msk          (0x1ul << UART_INTSTS_MODENIF_Pos)                /*!< UART_T::INTSTS: MODENIF Mask           */

#define UART_INTSTS_RXTOIF_Pos           (4)                                               /*!< UART_T::INTSTS: RXTOIF Position        */
#define UART_INTSTS_RXTOIF_Msk           (0x1ul << UART_INTSTS_RXTOIF_Pos)                 /*!< UART_T::INTSTS: RXTOIF Mask            */

#define UART_INTSTS_BUFERRIF_Pos         (5)                                               /*!< UART_T::INTSTS: BUFERRIF Position      */
#define UART_INTSTS_BUFERRIF_Msk         (0x1ul << UART_INTSTS_BUFERRIF_Pos)               /*!< UART_T::INTSTS: BUFERRIF Mask          */

#define UART_INTSTS_LINIF_Pos            (7)                                               /*!< UART_T::INTSTS: LINIF Position         */
#define UART_INTSTS_LINIF_Msk            (0x1ul << UART_INTSTS_LINIF_Pos)                  /*!< UART_T::INTSTS: LINIF Mask             */

#define UART_INTSTS_RDAINT_Pos           (8)                                               /*!< UART_T::INTSTS: RDAINT Position        */
#define UART_INTSTS_RDAINT_Msk           (0x1ul << UART_INTSTS_RDAINT_Pos)                 /*!< UART_T::INTSTS: RDAINT Mask            */

#define UART_INTSTS_THERINT_Pos          (9)                                               /*!< UART_T::INTSTS: THERINT Position       */
#define UART_INTSTS_THERINT_Msk          (0x1ul << UART_INTSTS_THERINT_Pos)                /*!< UART_T::INTSTS: THERINT Mask           */

#define UART_INTSTS_RLSINT_Pos           (10)                                              /*!< UART_T::INTSTS: RLSINT Position        */
#define UART_INTSTS_RLSINT_Msk           (0x1ul << UART_INTSTS_RLSINT_Pos)                 /*!< UART_T::INTSTS: RLSINT Mask            */

#define UART_INTSTS_MODEMINT_Pos         (11)                                              /*!< UART_T::INTSTS: MODEMINT Position      */
#define UART_INTSTS_MODEMINT_Msk         (0x1ul << UART_INTSTS_MODEMINT_Pos)               /*!< UART_T::INTSTS: MODEMINT Mask          */

#define UART_INTSTS_RXTOINT_Pos          (12)                                              /*!< UART_T::INTSTS: RXTOINT Position       */
#define UART_INTSTS_RXTOINT_Msk          (0x1ul << UART_INTSTS_RXTOINT_Pos)                /*!< UART_T::INTSTS: RXTOINT Mask           */

#define UART_INTSTS_BUFERRINT_Pos        (13)                                              /*!< UART_T::INTSTS: BUFERRINT Position     */
#define UART_INTSTS_BUFERRINT_Msk        (0x1ul << UART_INTSTS_BUFERRINT_Pos)              /*!< UART_T::INTSTS: BUFERRINT Mask         */

#define UART_INTSTS_LININT_Pos           (15)                                              /*!< UART_T::INTSTS: LININT Position        */
#define UART_INTSTS_LININT_Msk           (0x1ul << UART_INTSTS_LININT_Pos)                 /*!< UART_T::INTSTS: LININT Mask            */

#define UART_INTSTS_DRLSIF_Pos           (18)                                              /*!< UART_T::INTSTS: DRLSIF Position        */
#define UART_INTSTS_DRLSIF_Msk           (0x1ul << UART_INTSTS_DRLSIF_Pos)                 /*!< UART_T::INTSTS: DRLSIF Mask            */

#define UART_INTSTS_DMODEMIF_Pos         (19)                                              /*!< UART_T::INTSTS: DMODEMIF Position      */
#define UART_INTSTS_DMODEMIF_Msk         (0x1ul << UART_INTSTS_DMODEMIF_Pos)               /*!< UART_T::INTSTS: DMODEMIF Mask          */

#define UART_INTSTS_DRXTOIF_Pos          (20)                                              /*!< UART_T::INTSTS: DRXTOIF Position       */
#define UART_INTSTS_DRXTOIF_Msk          (0x1ul << UART_INTSTS_DRXTOIF_Pos)                /*!< UART_T::INTSTS: DRXTOIF Mask           */

#define UART_INTSTS_DBERRIF_Pos          (21)                                              /*!< UART_T::INTSTS: DBERRIF Position       */
#define UART_INTSTS_DBERRIF_Msk          (0x1ul << UART_INTSTS_DBERRIF_Pos)                /*!< UART_T::INTSTS: DBERRIF Mask           */

#define UART_INTSTS_DLINIF_Pos           (23)                                              /*!< UART_T::INTSTS: DLINIF Position        */
#define UART_INTSTS_DLINIF_Msk           (0x1ul << UART_INTSTS_DLINIF_Pos)                 /*!< UART_T::INTSTS: DLINIF Mask            */

#define UART_INTSTS_DRLSINT_Pos          (26)                                              /*!< UART_T::INTSTS: DRLSINT Position       */
#define UART_INTSTS_DRLSINT_Msk          (0x1ul << UART_INTSTS_DRLSINT_Pos)                /*!< UART_T::INTSTS: DRLSINT Mask           */

#define UART_INTSTS_DMODEMI_Pos          (27)                                              /*!< UART_T::INTSTS: DMODEMI Position       */
#define UART_INTSTS_DMODEMI_Msk          (0x1ul << UART_INTSTS_DMODEMI_Pos)                /*!< UART_T::INTSTS: DMODEMI Mask           */

#define UART_INTSTS_DRXTOINT_Pos         (28)                                              /*!< UART_T::INTSTS: DRXTOINT Position      */
#define UART_INTSTS_DRXTOINT_Msk         (0x1ul << UART_INTSTS_DRXTOINT_Pos)               /*!< UART_T::INTSTS: DRXTOINT Mask          */

#define UART_INTSTS_DBERRINT_Pos         (29)                                              /*!< UART_T::INTSTS: DBERRINT Position      */
#define UART_INTSTS_DBERRINT_Msk         (0x1ul << UART_INTSTS_DBERRINT_Pos)               /*!< UART_T::INTSTS: DBERRINT Mask          */

#define UART_INTSTS_DLININT_Pos          (31)                                              /*!< UART_T::INTSTS: DLININT Position       */
#define UART_INTSTS_DLININT_Msk          (0x1ul << UART_INTSTS_DLININT_Pos)                /*!< UART_T::INTSTS: DLININT Mask           */

#define UART_TOUT_TOIC_Pos               (0)                                               /*!< UART_T::TOUT: TOIC Position            */
#define UART_TOUT_TOIC_Msk               (0x7ful << UART_TOUT_TOIC_Pos)                    /*!< UART_T::TOUT: TOIC Mask                */

#define UART_BAUD_BRD_Pos                (0)                                               /*!< UART_T::BAUD: BRD Position             */
#define UART_BAUD_BRD_Msk                (0xfffful << UART_BAUD_BRD_Pos)                   /*!< UART_T::BAUD: BRD Mask                 */

#define UART_BAUD_EDIVM1_Pos             (24)                                              /*!< UART_T::BAUD: EDIVM1 Position          */
#define UART_BAUD_EDIVM1_Msk             (0xful << UART_BAUD_EDIVM1_Pos)                   /*!< UART_T::BAUD: EDIVM1 Mask              */

#define UART_BAUD_BAUDM0_Pos             (28)                                              /*!< UART_T::BAUD: BAUDM0 Position          */
#define UART_BAUD_BAUDM0_Msk             (0x1ul << UART_BAUD_BAUDM0_Pos)                   /*!< UART_T::BAUD: BAUDM0 Mask              */

#define UART_BAUD_BAUDM1_Pos             (29)                                              /*!< UART_T::BAUD: BAUDM1 Position          */
#define UART_BAUD_BAUDM1_Msk             (0x1ul << UART_BAUD_BAUDM1_Pos)                   /*!< UART_T::BAUD: BAUDM1 Mask              */

#define UART_IRDA_TXEN_Pos               (1)                                               /*!< UART_T::IRDA: TXEN Position            */
#define UART_IRDA_TXEN_Msk               (0x1ul << UART_IRDA_TXEN_Pos)                     /*!< UART_T::IRDA: TXEN Mask                */

#define UART_IRDA_LOOPBACK_Pos           (2)                                               /*!< UART_T::IRDA: LOOPBACK Position        */
#define UART_IRDA_LOOPBACK_Msk           (0x1ul << UART_IRDA_LOOPBACK_Pos)                 /*!< UART_T::IRDA: LOOPBACK Mask            */

#define UART_IRDA_TXINV_Pos              (5)                                               /*!< UART_T::IRDA: TXINV Position           */
#define UART_IRDA_TXINV_Msk              (0x1ul << UART_IRDA_TXINV_Pos)                    /*!< UART_T::IRDA: TXINV Mask               */

#define UART_IRDA_RXINV_Pos              (6)                                               /*!< UART_T::IRDA: RXINV Position           */
#define UART_IRDA_RXINV_Msk              (0x1ul << UART_IRDA_RXINV_Pos)                    /*!< UART_T::IRDA: RXINV Mask               */

#define UART_ALTCTL_BRKFL_Pos            (0)                                               /*!< UART_T::ALTCTL: BRKFL Position         */
#define UART_ALTCTL_BRKFL_Msk            (0xful << UART_ALTCTL_BRKFL_Pos)                  /*!< UART_T::ALTCTL: BRKFL Mask             */

#define UART_ALTCTL_LINRXEN_Pos          (6)                                               /*!< UART_T::ALTCTL: LINRXEN Position       */
#define UART_ALTCTL_LINRXEN_Msk          (0x1ul << UART_ALTCTL_LINRXEN_Pos)                /*!< UART_T::ALTCTL: LINRXEN Mask           */

#define UART_ALTCTL_LINTXEN_Pos          (7)                                               /*!< UART_T::ALTCTL: LINTXEN Position       */
#define UART_ALTCTL_LINTXEN_Msk          (0x1ul << UART_ALTCTL_LINTXEN_Pos)                /*!< UART_T::ALTCTL: LINTXEN Mask           */

#define UART_FUNCSEL_LINEN_Pos           (0)                                               /*!< UART_T::FUNCSEL: LINEN Position        */
#define UART_FUNCSEL_LINEN_Msk           (0x1ul << UART_FUNCSEL_LINEN_Pos)                 /*!< UART_T::FUNCSEL: LINEN Mask            */

#define UART_FUNCSEL_IRDAEN_Pos          (1)                                               /*!< UART_T::FUNCSEL: IRDAEN Position       */
#define UART_FUNCSEL_IRDAEN_Msk          (0x1ul << UART_FUNCSEL_IRDAEN_Pos)                /*!< UART_T::FUNCSEL: IRDAEN Mask           */

/**@}*/ /* UART_CONST */
/**@}*/ /* end of UART register group */


/*---------------------- Volume Control Enable Register -------------------------*/
/**
    @addtogroup VOLCTRL Volume Control Enable Register(VOLCTRL)
    Memory Mapped Structure for VOLCTRL Controller
@{ */
 
typedef struct
{


/**
 * @var VOLCTRL_T::EN
 * Offset: 0x00  Volume Control Enable Register
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[0]     |SDADCVOLEN|Delta-Sigma ADC Signal Volume Control Enable
 * |        |          |0 =   bypass the volume control function.
 * |        |          |1 =   enable the volume control function.
 * |[1]     |DPWMVOLEN |DPWM Audio Signal Volume Control Enable
 * |        |          |0 = bypass the volume control function.
 * |        |          |1 = enable the volume control function.
 * |[2]     |SDADCZCEN |Delta-Sigma ADC Signal Volume Zero Crossing Enable
 * |        |          |0 = disable zero crossing update gain.
 * |        |          |1 = enable Zero crossing update gain.
 * |[3]     |DPWMZCEN  |DPWM Audio Signal Volume Zero Crossing Enable
 * |        |          |0 = disable zero crossing update gain.
 * |        |          |1 = enable Zero crossing update gain.
 * @var VOLCTRL_T::ADCVAL
 * Offset: 0x04  ADC Volume  Control Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |VALUE     |Delta-Sigma ADC Signal Volume Control Value
 * |        |          |Format   <6,18>, gain range from -108.3dB to 36.1dB
 * |        |          |0x00_0001 ---   -108.3dB
 * |        |          |u2026
 * |        |          |0x04_0000   ---- 0dB(default)
 * |        |          |u2026
 * |        |          |0xCC_CCCC ----   34.1dB
 * |        |          |Others   reserved
 * |        |          |Volume db = 20*log10(VALUE)
 * @var VOLCTRL_T::DPWMVAL
 * Offset: 0x08  DPWM Volume Control Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[23:0]  |VALUE     |DPWM Audio Signal Volume Control Value
 * |        |          |Format   <6,18>. Gain range from 108.3dB to 36.1dB
 * |        |          |0x00_0001 ----   -108.3dB
 * |        |          |u2026
 * |        |          |0x40_0000   ---- 0dB (default)
 * |        |          |u2026
 * |        |          |0xFF_FFFF   ---- 36.1dB
 * |        |          |Volume db = 20*log10(VALUE)
 */
    __IO uint32_t EN;                    /*!< [0x0000] Volume Control Enable Register                                   */
    __IO uint32_t ADCVAL;                /*!< [0x0004] ADC Volume  Control Value                                        */
    __IO uint32_t DPWMVAL;               /*!< [0x0008] DPWM Volume Control Value                                        */

} VOLCTRL_T;

/**
    @addtogroup VOLCTRL_CONST VOLCTRL Bit Field Definition
    Constant Definitions for VOLCTRL Controller
@{ */

#define VOLCTRL_EN_SDADCVOLEN_Pos        (0)                                               /*!< VOLCTRL_T::EN: SDADCVOLEN Position     */
#define VOLCTRL_EN_SDADCVOLEN_Msk        (0x1ul << VOLCTRL_EN_SDADCVOLEN_Pos)              /*!< VOLCTRL_T::EN: SDADCVOLEN Mask         */

#define VOLCTRL_EN_DPWMVOLEN_Pos         (1)                                               /*!< VOLCTRL_T::EN: DPWMVOLEN Position      */
#define VOLCTRL_EN_DPWMVOLEN_Msk         (0x1ul << VOLCTRL_EN_DPWMVOLEN_Pos)               /*!< VOLCTRL_T::EN: DPWMVOLEN Mask          */

#define VOLCTRL_EN_SDADCZCEN_Pos         (2)                                               /*!< VOLCTRL_T::EN: SDADCZCEN Position      */
#define VOLCTRL_EN_SDADCZCEN_Msk         (0x1ul << VOLCTRL_EN_SDADCZCEN_Pos)               /*!< VOLCTRL_T::EN: SDADCZCEN Mask          */

#define VOLCTRL_EN_DPWMZCEN_Pos          (3)                                               /*!< VOLCTRL_T::EN: DPWMZCEN Position       */
#define VOLCTRL_EN_DPWMZCEN_Msk          (0x1ul << VOLCTRL_EN_DPWMZCEN_Pos)                /*!< VOLCTRL_T::EN: DPWMZCEN Mask           */

#define VOLCTRL_ADCVAL_VALUE_Pos         (0)                                               /*!< VOLCTRL_T::ADCVAL: VALUE Position      */
#define VOLCTRL_ADCVAL_VALUE_Msk         (0xfffffful << VOLCTRL_ADCVAL_VALUE_Pos)          /*!< VOLCTRL_T::ADCVAL: VALUE Mask          */

#define VOLCTRL_DPWMVAL_VALUE_Pos        (0)                                               /*!< VOLCTRL_T::DPWMVAL: VALUE Position     */
#define VOLCTRL_DPWMVAL_VALUE_Msk        (0xfffffful << VOLCTRL_DPWMVAL_VALUE_Pos)         /*!< VOLCTRL_T::DPWMVAL: VALUE Mask         */

/**@}*/ /* VOLCTRL_CONST */
/**@}*/ /* end of VOLCTRL register group */


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
 * |[0]     |RSTCNT    |Clear Watchdog Timer
 * |        |          |Set this bit will clear the Watchdog timer.
 * |        |          |0 = Writing 0 to this bit has no effect.
 * |        |          |1 = Reset the contents of the Watchdog timer.
 * |        |          |NOTE: This bit will auto clear after few clock cycle
 * |[1]     |RSTEN     |Watchdog Timer Reset Enable
 * |        |          |Setting this bit will enable the Watchdog timer reset function.
 * |        |          |0 = Disable Watchdog timer reset function.
 * |        |          |1= Enable Watchdog timer reset function.
 * |[2]     |RSTF      |Watchdog Timer Reset Flag
 * |        |          |When the Watchdog timer initiates a reset, the hardware will set this bit
 * |        |          |This flag can be read by software to determine the source of reset
 * |        |          |Software is responsible to clear it manually by writing 1 to it
 * |        |          |If RSTEN is disabled, then the Watchdog timer has no effect on this bit.
 * |        |          |0 = Watchdog timer reset has not occurred.
 * |        |          |1= Watchdog timer reset has occurred.
 * |        |          |NOTE: This bit is cleared by writing 1 to this bit.
 * |[3]     |IF        |Watchdog Timer Interrupt Flag
 * |        |          |If the Watchdog timer interrupt is enabled, then the hardware will set this bit to indicate that the Watchdog timer interrupt has occurred
 * |        |          |If the Watchdog timer interrupt is not enabled, then this bit indicates that a timeout period has elapsed.
 * |        |          |0 = Watchdog timer interrupt has not occurred.
 * |        |          |1 = Watchdog timer interrupt has occurred.
 * |        |          |NOTE: This bit is cleared by writing 1 to this bit.
 * |[6]     |INTEN     |Watchdog Timer Interrupt Enable
 * |        |          |0 = Disable the Watchdog timer interrupt.
 * |        |          |1 = Enable the Watchdog timer interrupt.
 * |[7]     |WDTEN     |Watchdog Timer Enable
 * |        |          |0 = Disable the Watchdog timer (This action will reset the internal counter).
 * |        |          |1 = Enable the Watchdog timer.
 * |[10:8]  |TOUTSEL   |Watchdog Timer Interval Select
 * |        |          |These three bits select the timeout interval for the Watchdog timer, a watchdog reset will occur 1024 clock cycles later if WDG not reset
 * |        |          |The timeout is given by:
 * |        |          |Interrupt Timeout = 2^(2xTOUTSEL+4) x WDT_CLK.
 * |        |          |Reset Timeout = (2^(2xTOUTSEL+4) +1024) x WDT_CLK.
 * |        |          |Where WDT_CLK is the period of the Watchdog Timer clock source.
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

#define WDT_CTL_INTEN_Pos                (6)                                               /*!< WDT_T::CTL: INTEN Position             */
#define WDT_CTL_INTEN_Msk                (0x1ul << WDT_CTL_INTEN_Pos)                      /*!< WDT_T::CTL: INTEN Mask                 */

#define WDT_CTL_WDTEN_Pos                (7)                                               /*!< WDT_T::CTL: WDTEN Position             */
#define WDT_CTL_WDTEN_Msk                (0x1ul << WDT_CTL_WDTEN_Pos)                      /*!< WDT_T::CTL: WDTEN Mask                 */

#define WDT_CTL_TOUTSEL_Pos              (8)                                               /*!< WDT_T::CTL: TOUTSEL Position           */
#define WDT_CTL_TOUTSEL_Msk              (0x7ul << WDT_CTL_TOUTSEL_Pos)                    /*!< WDT_T::CTL: TOUTSEL Mask               */

/**@}*/ /* WDT_CONST */
/**@}*/ /* end of WDT register group */


/*---------------------- Standby RAM Block Address Space -------------------------*/
/**
    @addtogroup Standby RAM Block Address Space(SBRAM)
    Memory Mapped Structure for SBRAM 
@{ */
 
typedef struct
{
    __IO uint32_t D[64];          
	
} SBRAM_T;

/**@}*/ /* end of REGISTER group */

/* ================================================================================ */
/* ================              Peripheral memory map             ================ */
/* ================================================================================ */

#define APB1_BASE                       0x40000000UL
#define SYSTICK_BASE                    0xE000E000UL
#define SCS0_BASE                       0xE000E100UL
#define SYSINFO_BASE                    0xE000ED00UL
#define SYS_BASE                        0x50000000UL
#define CLK_BASE                        0x50000200UL
#define INT_BASE                        0x50000300UL
#define GPIO_BASE                       0x50004000UL
#define GPIOB_BASE                      0x50004040UL
#define GPIO_DBNCECON_BASE              0x50004180UL
#define PDMA0_BASE                      0x50008000UL
#define PDMA1_BASE                      0x50008100UL
#define PDMA2_BASE                      0x50008200UL
#define PDMA3_BASE                      0x50008300UL
#define PDMA4_BASE                      0x50008400UL
#define PDMAC_BASE                      0x50008F00UL
#define FMC_BASE                        0x5000C000UL
#define WDT_BASE                        0x40004000UL
#define RTC_BASE                        0x40008000UL
#define TMR0_BASE                       0x40010000UL
#define TMR1_BASE                       0x40010020UL
#define I2C_BASE                        0x40020000UL
#define I2C0_BASE                       0x40020000UL
#define SPI0_BASE                       0x40030000UL
#define SPI1_BASE                       0x40038000UL
#define PWM_BASE                        0x40040000UL
#define UART0_BASE                      0x40050000UL
#define UART1_BASE                      0x40058000UL
#define SARADC_BASE                     0x40060000UL
#define DPWM_BASE                       0x40070000UL
#define ANA_BASE                        0x40080000UL
#define BOD_BASE                        0x40084000UL
#define OPA_BASE                        0x40090000UL
#define I2S_BASE                        0x400A0000UL
#define BIQ_BASE                        0x400B0000UL
#define ALC_BASE                        0x400B0090UL
#define VOLCTRL_BASE                    0x400B00A0UL
#define CSCAN_BASE                      0x400D0000UL
#define SDADC_BASE                      0x400E0000UL
#define SBRAM_BASE                      0x400F0000UL 

/* ================================================================================ */
/* ================             Peripheral declaration             ================ */
/* ================================================================================ */

#define SYS                             ((SYS_T                *) SYS_BASE)
#define SYSTICK                         ((SYSTICK_T            *) SYSTICK_BASE)
#define SCS                             ((SCS0_T               *) SCS0_BASE)
#define INT                             ((INT_T                *) INT_BASE)
#define SYSINFO                         ((SYSINFO_T            *) SYSINFO_BASE)
#define CLK                             ((CLK_T                *) CLK_BASE)
#define GPIO                            ((GPIO_DB_T            *) GPIO_DBNCECON_BASE)
#define PA                              ((GPIO_T               *) GPIO_BASE)
#define PB                              ((GPIO_T               *) GPIOB_BASE)
#define BOD                             ((BOD_T                *) BOD_BASE)
#define I2C                             ((I2C_T                *) I2C_BASE)
#define I2C0                            ((I2C_T                *) I2C_BASE)
#define PWM                             ((PWM_T                *) PWM_BASE)
#define PWM0                            ((PWM_T                *) PWM_BASE)
#define RTC                             ((RTC_T                *) RTC_BASE)
#define SPI0                            ((SPI0_T               *) SPI0_BASE)
#define SPI1                            ((SPI1_T               *) SPI1_BASE)
#define TMR0                            ((TMR_T                *) TMR0_BASE)
#define TMR1                            ((TMR_T                *) TMR1_BASE)
#define WDT                             ((WDT_T                *) WDT_BASE)
#define UART0                           ((UART_T               *) UART0_BASE)
#define UART1                           ((UART_T               *) UART1_BASE)
#define I2S                             ((I2S_T                *) I2S_BASE)
#define PDMA0                           ((PDMA_T               *) PDMA0_BASE)
#define PDMA1                           ((PDMA_T               *) PDMA1_BASE)
#define PDMA2                           ((PDMA_T               *) PDMA2_BASE)
#define PDMA3                           ((PDMA_T               *) PDMA3_BASE)
#define PDMA4                           ((PDMA_T               *) PDMA4_BASE)
#define PDMAC                           ((PDMAC_T              *) PDMAC_BASE)
#define MAC                             ((MAC_T                *) MAC_BASE)
#define VOLCTRL                         ((VOLCTRL_T            *) VOLCTRL_BASE)
#define FMC                             ((FMC_T                *) FMC_BASE)
#define SDADC                           ((SDADC_T              *) SDADC_BASE)
#define DPWM                            ((DPWM_T               *) DPWM_BASE)
#define ANA                             ((ANA_T                *) ANA_BASE)
#define ALC                             ((ALC_T                *) ALC_BASE)
#define BIQ                             ((BIQ_T                *) BIQ_BASE)
#define SARADC                          ((SARADC_T             *) SARADC_BASE)
#define OPA                             ((OPA_T                *) OPA_BASE)
#define CSCAN				            ((CSCAN_T              *) CSCAN_BASE)
#define SBRAM				            ((SBRAM_T              *) SBRAM_BASE)

/** @} */ /* End of group Device_Peripheral_Registers */
/** @} */ /* End of group I91200_Target_Specification */
/** @} */ /* End of group (null) */


#define UNLOCKREG(x)        do{*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x59;*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x16;*((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x88;}while(*((__IO uint32_t *)(SYS_BASE + 0x100))==0)
#define LOCKREG(x)          *((__IO uint32_t *)(SYS_BASE + 0x100)) = 0x00

#define REGCOPY(dest, src)  *((uint32_t *)&(dest)) = *((uint32_t *)&(src))
#define CLEAR(dest)         *((uint32_t *)&(dest)) = 0


//=============================================================================
/** @addtogroup I91200_IO_ROUTINE I91200 I/O routines
  The Declaration of I91200 I/O routines
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

/*@}*/ /* end of group I91200_IO_ROUTINE */


/** @addtogroup I91200_legacy_Constants I91200 Legacy Constants
  I91200 Legacy Constants
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

/*@}*/ /* end of group I91200_legacy_Constants */

/*@}*/ /* end of group I91200_Definitions */

#define __I91200_SERIES__  (0x91200000)
#define __CHIP_SERIES__    __I91200_SERIES__

/******************************************************************************/
/*                         Peripheral header files                            */
/******************************************************************************/
#include "biq.h"
#include "bod.h"
#include "capsense.h"
#include "clk.h"
#include "dpwm.h"
#include "fmc.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "pdma.h"
#include "pwm.h"
#include "rtc.h"
#include "saradc.h"
#include "sdadc.h"
#include "spi0.h"
#include "spi1.h"
#include "sys.h"
#include "timer.h"
#include "uart.h"
#include "volctrl.h"
#include "wdt.h"

#endif 
