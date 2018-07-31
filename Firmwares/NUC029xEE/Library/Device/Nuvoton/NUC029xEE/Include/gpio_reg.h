/**************************************************************************//**
 * @file     gpio_reg.h
 * @version  V1.00
 * @brief    GPIO register definition header file
 *
 * @copyright (C) 2018 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __GPIO_REG_H__
#define __GPIO_REG_H__

/*-------------------------------- Device Specific Peripheral registers structures ---------------------*/
/** @addtogroup REGISTER Control Register
  Peripheral Control Registers
  @{
 */


/*---------------------- General Purpose Input/Output Controller -------------------------*/
/**
    @addtogroup GPIO General Purpose Input/Output Controller (GPIO)
    Memory Mapped Structure for GPIO Controller
@{ */



typedef struct
{


/**
 * @var GPIO_T::PMD
 * Offset: 0x00  GPIO Port [A/B/C/D/E/F] Pin I/O Mode Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[2n+1:2n]|PMDn     |GPIOx I/O Pin[n] Mode Control
 * |        |          |Determine each I/O mode of GPIOx pins.
 * |        |          |00 = GPIO port [n] pin is in Input mode.
 * |        |          |01 = GPIO port [n] pin is in Push-pull Output mode.
 * |        |          |10 = GPIO port [n] pin is in Open-drain Output mode.
 * |        |          |11 = GPIO port [n] pin is in Quasi-bidirectional mode.
 * |        |          |Note1: Max. n = 3 for GPIOF; Max. n = 15 for GPIOA/GPIOB/GPIOC/GPIOD/GPIOE.
 * |        |          |Note2:
 * |        |          |The initial value of this field is defined by CIOINI (Config0[10]).
 * |        |          |If CIOINI is set to 1, the default value is 0xFFFF_FFFF and all pins will be Quasi-bidirectional
 * |        |          |mode after chip is powered on.
 * |        |          |If CIOINI is cleared to 0, the default value is 0x0000_0000 and all pins will be input only mode
 * |        |          |after chip is powered on.
 * @var GPIO_T::OFFD
 * Offset: 0x04  GPIO Port [A/B/C/D/E/F] Pin Digital Input Path Disable Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[31:16] |OFFD      |GPIOx Pin[n] Digital Input Path Disable Control
 * |        |          |Each of these bits is used to control if the digital input path of corresponding GPIO pin is
 * |        |          |disabled.
 * |        |          |If input is analog signal, users can disable GPIO digital input path to avoid current leakage.
 * |        |          |0 = I/O digital input path Enabled.
 * |        |          |1 = I/O digital input path Disabled (digital input tied to low). 
 * |        |          |Note: Max. n = 3 for GPIOF; Max. n = 15 for GPIOA/GPIOB/GPIOC/GPIOD/GPIOE.
 * @var GPIO_T::DOUT
 * Offset: 0x08  GPIO Port [A/B/C/D/E/F] Data Output Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[n]     |DOUTn     |GPIOx Pin[n] Output Value
 * |        |          |Each of these bits controls the status of a GPIO pin when the GPIO pin is configured as
 * |        |          |Push-pull output, open-drain output or quasi-bidirectional mode.
 * |        |          |0 = GPIO port [A/B/C/D/E/F] Pin[n] will drive Low if the GPIO pin is configured as Push-pull
 * |        |          |output, Open-drain output or Quasi-bidirectional mode.
 * |        |          |1 = GPIO port [A/B/C/D/E/F] Pin[n] will drive High if the GPIO pin is configured as Push-pull
 * |        |          |output or Quasi-bidirectional mode.
 * |        |          |Note: Max. n = 3 for GPIOF; Max. n = 15 for GPIOA/GPIOB/GPIOC/GPIOD/GPIOE. 
 * @var GPIO_T::DMASK
 * Offset: 0x0C  GPIO Port [A/B/C/D/E/F] Data Output Write Mask
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[n]     |DMASKn    |Port [A/B/C/D/E/F] Data Output Write Mask
 * |        |          |These bits are used to protect the corresponding register of GPIOx_DOUT bit[n].
 * |        |          |When the DMASK bit[n] is set to 1, the corresponding GPIOx_DOUT[n] bit is protected.
 * |        |          |If the write signal is masked, write data to the protect bit is ignored.
 * |        |          |0 = Corresponding GPIOx_DOUT[n] bit can be updated.
 * |        |          |1 = Corresponding GPIOx_DOUT[n] bit protected.
 * |        |          |Note1: This function only protects the corresponding GPIOx_DOUT[n] bit, and will not protect the
 * |        |          |corresponding bit control register (PAn_PDIO, PBn_PDIO, PCn_PDIO, PDn_PDIO, PEn_PDIO and
 * |        |          |PFn_PDIO). 
 * |        |          |Note2: Max. n = 3 for GPIOF; Max. n = 15 for GPIOA/GPIOB/GPIOC/GPIOD/GPIOE.
 * @var GPIO_T::PIN
 * Offset: 0x10  GPIO Port [A/B/C/D/E/F] Pin Value
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[n]     |PINn      |Port [A/B/C/D/E/F] Pin Values
 * |        |          |Each bit of the register reflects the actual status of the respective GPIO pin.
 * |        |          |If the bit is 1, it indicates the corresponding pin status is high, else the pin status is low.
 * |        |          |Note: Max. n = 3 for GPIOF; Max. n = 15 for GPIOA/GPIOB/GPIOC/GPIOD/GPIOE. 
 * @var GPIO_T::DBEN
 * Offset: 0x14  GPIO Port [A/B/C/D/E/F] De-bounce Enable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[n]     |DBENn     |Port [A/B/C/D/E/F] Input Signal De-Bounce Enable
 * |        |          |DBEN[n] is used to enable the de-bounce function for each corresponding bit.
 * |        |          |If the input signal pulse width cannot be sampled by continuous two de-bounce sample cycle, the
 * |        |          |input signal transition is seen as the signal bounce and will not trigger the interrupt.
 * |        |          |The de-bounce clock source is controlled by DBNCECON[4], one de-bounce sample cycle period is
 * |        |          |controlled by DBNCECON[3:0].
 * |        |          |0 = Bit[n] de-bounce function Disabled.
 * |        |          |1 = Bit[n] de-bounce function Enabled.
 * |        |          |The de-bounce function is valid only for edge triggered interrupt.
 * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
 * |        |          |Note: Max. n = 3 for GPIOF; Max. n = 15 for GPIOA/GPIOB/GPIOC/GPIOD/GPIOE.
 * @var GPIO_T::IMD
 * Offset: 0x18  GPIO Port [A/B/C/D/E/F] Interrupt Mode Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[n]     |IMDn      |Port [A/B/C/D/E/F] Edge Or Level Detection Interrupt Control
 * |        |          |IMD[n] is used to control the interrupt is by level trigger or by edge trigger.
 * |        |          |If the interrupt is by edge trigger, the trigger source can be controlled by de-bounce.
 * |        |          |If the interrupt is by level trigger, the input source is sampled by one HCLK.
 * |        |          |clock and generates the interrupt.
 * |        |          |0 = Edge trigger interrupt.
 * |        |          |1 = Level trigger interrupt.
 * |        |          |If the pin is set as the level trigger interrupt, only one level can be set on the registers
 * |        |          |GPIOx_IEN.
 * |        |          |If both levels to trigger interrupt are set, the setting is ignored and no interrupt will occur.
 * |        |          |The de-bounce function is valid only for edge triggered interrupt.
 * |        |          |If the interrupt mode is level triggered, the de-bounce enable bit is ignored.
 * |        |          |Note: Max. n = 3 for GPIOF; Max. n = 15 for GPIOA/GPIOB/GPIOC/GPIOD/GPIOE. 
 * @var GPIO_T::IEN
 * Offset: 0x1C  GPIO Port [A/B/C/D/E/F] Interrupt Enable
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[n]     |IF_ENn    |Port [A/B/C/D/E/F] Interrupt Enable By Input Falling Edge Or Input Level Low
 * |        |          |IF_EN[n] is used to enable the interrupt for each of the corresponding input GPIO_PIN[n].
 * |        |          |Set bit to 1 also enable the pin wake-up function.
 * |        |          |When setting the IF_EN[n] bit to 1:
 * |        |          |If the interrupt is level trigger, the input PIN[n] state at level "low" will generate the
 * |        |          |interrupt.
 * |        |          |If the interrupt is edge trigger, the input PIN[n] state change from "high-to-low" will generate
 * |        |          |the interrupt.
 * |        |          |0 = PIN[n] state low-level or high-to-low change interrupt Disabled.
 * |        |          |1 = PIN[n] state low-level or high-to-low change interrupt Enabled.
 * |        |          |Note: Max. n = 3 for GPIOF; Max. n = 15 for GPIOA/GPIOB/GPIOC/GPIOD/GPIOE.
 * |[n+16]  |IR_ENn    |Port [A/B/C/D/E/F] Interrupt Enable By Input Rising Edge Or Input Level High
 * |        |          |IR_EN[n] used to enable the interrupt for each of the corresponding input GPIO_PIN[n].
 * |        |          |Set bit to 1 also enable the pin wake-up function.
 * |        |          |When setting the IR_EN[n] bit to 1:
 * |        |          |If the interrupt is level trigger, the input PIN[n] state at level "high" will generate the
 * |        |          |interrupt.
 * |        |          |If the interrupt is edge trigger, the input PIN[n] state change from "low-to-high" will generate
 * |        |          |the interrupt.
 * |        |          |0 = PIN[n] level-high or low-to-high interrupt Disabled.
 * |        |          |1 = PIN[n] level-high or low-to-high interrupt Enabled.
 * |        |          |Note: Max. n = 3 for GPIOF; Max. n = 15 for GPIOA/GPIOB/GPIOC/GPIOD/GPIOE.
 * @var GPIO_T::ISRC
 * Offset: 0x20  GPIO Port [A/B/C/D/E/F] Interrupt Source Flag
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[n]     |ISRCn     |Port [A/B/C/D/E/F] Interrupt Source Flag
 * |        |          |Read :
 * |        |          |0 = No interrupt at GPIOx[n].
 * |        |          |1 = GPIOx[n] generates an interrupt.
 * |        |          |Write :
 * |        |          |0= No action.
 * |        |          |1= Clear the corresponding pending interrupt. 
 * |        |          |Note: Max. n = 3 for GPIOF; Max. n = 15 for GPIOA/GPIOB/GPIOC/GPIOD/GPIOE. 
 */

    __IO uint32_t PMD;           /* Offset: 0x00  GPIO Port [A/B/C/D/E/F] Pin I/O Mode Control                       */
    __IO uint32_t OFFD;          /* Offset: 0x04  GPIO Port [A/B/C/D/E/F] Pin Digital Input Path Disable Control     */
    __IO uint32_t DOUT;          /* Offset: 0x08  GPIO Port [A/B/C/D/E/F] Data Output Value                          */
    __IO uint32_t DMASK;         /* Offset: 0x0C  GPIO Port [A/B/C/D/E/F] Data Output Write Mask                     */
    __I  uint32_t PIN;           /* Offset: 0x10  GPIO Port [A/B/C/D/E/F] Pin Value                                  */
    __IO uint32_t DBEN;          /* Offset: 0x14  GPIO Port [A/B/C/D/E/F] De-bounce Enable                           */
    __IO uint32_t IMD;           /* Offset: 0x18  GPIO Port [A/B/C/D/E/F] Interrupt Mode Control                     */
    __IO uint32_t IEN;           /* Offset: 0x1C  GPIO Port [A/B/C/D/E/F] Interrupt Enable                           */
    __IO uint32_t ISRC;          /* Offset: 0x20  GPIO Port [A/B/C/D/E/F] Interrupt Source Flag                      */

} GPIO_T;





typedef struct
{


/**
 * @var GPIO_DBNCECON_T::DBNCECON
 * Offset: 0x180  External Interrupt De-bounce Control
 * ---------------------------------------------------------------------------------------------------
 * |Bits    |Field     |Descriptions
 * | :----: | :----:   | :---- |
 * |[3:0]   |DBCLKSEL  |De-Bounce Sampling Cycle Selection
 * |        |          |0000 = Sample interrupt input once per 1 clocks
 * |        |          |0001 = Sample interrupt input once per 2 clocks
 * |        |          |0010 = Sample interrupt input once per 4 clocks
 * |        |          |0011 = Sample interrupt input once per 8 clocks
 * |        |          |0100 = Sample interrupt input once per 16 clocks
 * |        |          |0101 = Sample interrupt input once per 32 clocks
 * |        |          |0110 = Sample interrupt input once per 64 clocks
 * |        |          |0111 = Sample interrupt input once per 128 clocks
 * |        |          |1000 = Sample interrupt input once per 256 clocks
 * |        |          |1001 = Sample interrupt input once per 2*256 clocks
 * |        |          |1010 = Sample interrupt input once per 4*256clocks
 * |        |          |1011 = Sample interrupt input once per 8*256 clocks
 * |        |          |1100 = Sample interrupt input once per 16*256 clocks
 * |        |          |1101 = Sample interrupt input once per 32*256 clocks
 * |        |          |1110 = Sample interrupt input once per 64*256 clocks
 * |        |          |1111 = Sample interrupt input once per 128*256 clocks
 * |        |          |Sample   interrupt input once per 128*256 clocks
 * |[4]     |DBCLKSRC  |De-Bounce Counter Clock Source Selection
 * |        |          |0 = De-bounce counter clock source is the HCLK.
 * |        |          |1 = De-bounce counter clock source is the internal 10 kHz low speed oscillator.
 * |[5]     |ICLK_ON   |Interrupt Clock On Mode
 * |        |          |0 = Edge detection circuit is active only if I/O pin corresponding GPIOx_IEN bit is set to 1.
 * |        |          |1 = All I/O pins edge detection circuit is always active after reset.
 * |        |          |It is recommended to turn off this bit to save system power if no special application concern.
 */

    __IO uint32_t DBNCECON;      /* Offset: 0x180  External Interrupt De-bounce Control                              */

} GPIO_DBNCECON_T;


/**
    @addtogroup GPIO_CONST GPIO Bit Field Definition
    Constant Definitions for GPIO Controller
@{ */


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
#define GPIO_DBNCECON_ICLK_ON_Pos   5                                           /*!< GPIO_DBNCECON_T::DBNCECON: ICLK_ON  Position */
#define GPIO_DBNCECON_ICLK_ON_Msk   (1ul << GPIO_DBNCECON_ICLK_ON_Pos)          /*!< GPIO_DBNCECON_T::DBNCECON: ICLK_ON  Mask */

#define GPIO_DBNCECON_DBCLKSRC_Pos  4                                           /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSRC Position */
#define GPIO_DBNCECON_DBCLKSRC_Msk  (1ul << GPIO_DBNCECON_DBCLKSRC_Pos)         /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSRC Mask */

#define GPIO_DBNCECON_DBCLKSEL_Pos  0                                           /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSEL Position */
#define GPIO_DBNCECON_DBCLKSEL_Msk  (0xFul << GPIO_DBNCECON_DBCLKSEL_Pos)       /*!< GPIO_DBNCECON_T::DBNCECON: DBCLKSEL Mask */
/*@}*/ /* end of group GPIO_CONST */
/*@}*/ /* end of group GPIO */
/**@}*/ /* end of REGISTER group */



#endif /* __GPIO_REG_H__ */
