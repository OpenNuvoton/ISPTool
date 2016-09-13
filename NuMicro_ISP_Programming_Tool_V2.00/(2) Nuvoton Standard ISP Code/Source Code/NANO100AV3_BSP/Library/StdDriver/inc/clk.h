/**************************************************************************//**
 * @file     clk.h
 * @version  V1.00
 * $Revision: 10 $
 * $Date: 15/06/05 11:13a $
 * @brief    Nano100 series CLK driver header file
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __CLK_H__
#define __CLK_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup NANO100_Device_Driver NANO100 Device Driver
  @{
*/

/** @addtogroup NANO100_CLK_Driver CLK Driver
  @{
*/

/** @addtogroup NANO100_CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/


#define FREQ_128MHZ       128000000
#define FREQ_120MHZ       120000000
#define FREQ_48MHZ         48000000
#define FREQ_42MHZ         42000000
#define FREQ_32MHZ         32000000
#define FREQ_24MHZ         24000000
#define FREQ_12MHZ         12000000

/********************* Bit definition of AHBCLK register **********************/
#define CLK_AHBCLK_GPIO_EN        (0x1UL<<CLK_AHBCLK_GPIO_EN_Pos)      /*!<GPIO clock enable */
#define CLK_AHBCLK_DMA_EN         (0x1UL<<CLK_AHBCLK_DMA_EN_Pos)       /*!<DMA clock enable */
#define CLK_AHBCLK_ISP_EN         (0x1UL<<CLK_AHBCLK_ISP_EN_Pos)       /*!<Flash ISP controller clock enable */
#define CLK_AHBCLK_EBI_EN         (0x1UL<<CLK_AHBCLK_EBI_EN_Pos)       /*!<EBI clock enable */
#define CLK_AHBCLK_SRAM_EN        (0x1UL<<CLK_AHBCLK_SRAM_EN_Pos)      /*!<SRAM Controller Clock Enable */
#define CLK_AHBCLK_TICK_EN        (0x1UL<<CLK_AHBCLK_TICK_EN_Pos)      /*!<System Tick Clock Enable */

/********************* Bit definition of APBCLK register **********************/
#define CLK_APBCLK_WDT_EN         (0x1UL<<CLK_APBCLK_WDT_EN_Pos)       /*!<Watchdog clock enable */
#define CLK_APBCLK_RTC_EN         (0x1UL<<CLK_APBCLK_RTC_EN_Pos)       /*!<RTC clock enable */
#define CLK_APBCLK_TMR0_EN        (0x1UL<<CLK_APBCLK_TMR0_EN_Pos)      /*!<Timer 0 clock enable */
#define CLK_APBCLK_TMR1_EN        (0x1UL<<CLK_APBCLK_TMR1_EN_Pos)      /*!<Timer 1 clock enable */
#define CLK_APBCLK_TMR2_EN        (0x1UL<<CLK_APBCLK_TMR2_EN_Pos)      /*!<Timer 2 clock enable */
#define CLK_APBCLK_TMR3_EN        (0x1UL<<CLK_APBCLK_TMR3_EN_Pos)      /*!<Timer 3 clock enable */
#define CLK_APBCLK_FDIV_EN        (0x1UL<<CLK_APBCLK_FDIV_EN_Pos)      /*!<Frequency Divider Output clock enable */
#define CLK_APBCLK_I2C0_EN        (0x1UL<<CLK_APBCLK_FDIV_EN_Pos)      /*!<I2C 0 clock enable */
#define CLK_APBCLK_I2C1_EN        (0x1UL<<CLK_APBCLK_I2C1_EN_Pos)      /*!<I2C 1 clock enable */
#define CLK_APBCLK_SPI0_EN        (0x1UL<<CLK_APBCLK_SPI0_EN_Pos)      /*!<SPI 0 clock enable */
#define CLK_APBCLK_SPI1_EN        (0x1UL<<CLK_APBCLK_SPI1_EN_Pos)      /*!<SPI 1 clock enable */
#define CLK_APBCLK_SPI2_EN        (0x1UL<<CLK_APBCLK_SPI2_EN_Pos)      /*!<SPI 2 clock enable */
#define CLK_APBCLK_UART0_EN       (0x1UL<<CLK_APBCLK_UART0_EN_Pos)     /*!<UART 0 clock enable */
#define CLK_APBCLK_UART1_EN       (0x1UL<<CLK_APBCLK_UART1_EN_Pos)     /*!<UART 1 clock enable */
#define CLK_APBCLK_PWM0_CH01_EN   (0x1UL<<CLK_APBCLK_PWM0_CH01_EN_Pos) /*!<PWM0 Channel 0 and Channel 1 Clock Enable Control */
#define CLK_APBCLK_PWM0_CH23_EN   (0x1UL<<CLK_APBCLK_PWM0_CH23_EN_Pos) /*!<PWM0 Channel 2 and Channel 3 Clock Enable Control */
#define CLK_APBCLK_PWM1_CH01_EN   (0x1UL<<CLK_APBCLK_PWM1_CH01_EN_Pos) /*!<PWM1 Channel 0 and Channel 1 Clock Enable Control */
#define CLK_APBCLK_PWM1_CH23_EN   (0x1UL<<CLK_APBCLK_PWM1_CH23_EN_Pos) /*!<PWM1 Channel 2 and Channel 3 Clock Enable Control */
#define CLK_APBCLK_USBD_EN        (0x1UL<<CLK_APBCLK_USBD_EN_Pos)      /*!<USB device clock enable */
#define CLK_APBCLK_ADC_EN         (0x1UL<<CLK_APBCLK_ADC_EN_Pos)       /*!<ADC clock enable */
#define CLK_APBCLK_I2S_EN         (0x1UL<<CLK_APBCLK_I2S_EN_Pos)       /*!<I2S clock enable */
#define CLK_APBCLK_SC0_EN         (0x1UL<<CLK_APBCLK_SC0_EN_Pos)       /*!<SmartCard 0 Clock Enable Control */
#define CLK_APBCLK_SC1_EN         (0x1UL<<CLK_APBCLK_SC1_EN_Pos)       /*!<SmartCard 1 Clock Enable Control */

/********************* Bit definition of CLKSTATUS register **********************/
#define CLK_CLKSTATUS_HXT_STB     (0x1UL<<CLK_CLKSTATUS_HXT_STB_Pos)       /*!<External high speed crystal clock source stable flag */
#define CLK_CLKSTATUS_LXT_STB     (0x1UL<<CLK_CLKSTATUS_LXT_STB_Pos)       /*!<External low speed crystal clock source stable flag */
#define CLK_CLKSTATUS_PLL_STB     (0x1UL<<CLK_CLKSTATUS_PLL_STB_Pos)       /*!<Internal PLL clock source stable flag */
#define CLK_CLKSTATUS_LIRC_STB    (0x1UL<<CLK_CLKSTATUS_LIRC_STB_Pos)      /*!<Internal low speed oscillator clock source stable flag */
#define CLK_CLKSTATUS_HIRC_STB    (0x1UL<<CLK_CLKSTATUS_HIRC_STB_Pos)      /*!<Internal high speed oscillator clock source stable flag */
#define CLK_CLKSTATUS_CLK_SW_FAIL (0x1UL<<CLK_CLKSTATUS_CLK_SW_FAIL_Pos)   /*!<Clock switch fail flag */


/********************* Bit definition of CLKSEL0 register **********************/
#define CLK_CLKSEL0_HCLK_S_HXT    (0UL<<CLK_CLKSEL0_HCLK_S_Pos)     /*!<Select HCLK clock source from high speed crystal */
#define CLK_CLKSEL0_HCLK_S_LXT    (1UL<<CLK_CLKSEL0_HCLK_S_Pos)     /*!<Select HCLK clock source from low speed crystal */
#define CLK_CLKSEL0_HCLK_S_PLL    (2UL<<CLK_CLKSEL0_HCLK_S_Pos)     /*!<Select HCLK clock source from PLL */
#define CLK_CLKSEL0_HCLK_S_LIRC   (3UL<<CLK_CLKSEL0_HCLK_S_Pos)     /*!<Select HCLK clock source from low speed oscillator */
#define CLK_CLKSEL0_HCLK_S_HIRC   (7UL<<CLK_CLKSEL0_HCLK_S_Pos)     /*!<Select HCLK clock source from high speed oscillator */

/********************* Bit definition of CLKSEL1 register **********************/

#define CLK_CLKSEL1_TMR1_S_HXT    (0x0UL<<CLK_CLKSEL1_TMR1_S_Pos)     /*!<Select TMR1 clock source from high speed crystal */
#define CLK_CLKSEL1_TMR1_S_LXT    (0x1UL<<CLK_CLKSEL1_TMR1_S_Pos)     /*!<Select TMR1 clock source from low speed crystal */
#define CLK_CLKSEL1_TMR1_S_LIRC   (0x2UL<<CLK_CLKSEL1_TMR1_S_Pos)     /*!<Select TMR1 clock source from low speed oscillator  */
#define CLK_CLKSEL1_TMR1_S_EXT    (0x3UL<<CLK_CLKSEL1_TMR1_S_Pos)     /*!<Select TMR1 clock source from external trigger */
#define CLK_CLKSEL1_TMR1_S_HIRC   (0x4UL<<CLK_CLKSEL1_TMR1_S_Pos)     /*!<Select TMR1 clock source from high speed oscillator */

#define CLK_CLKSEL1_TMR0_S_HXT    (0x0UL<<CLK_CLKSEL1_TMR0_S_Pos)     /*!<Select TMR0 clock source from high speed crystal */
#define CLK_CLKSEL1_TMR0_S_LXT    (0x1UL<<CLK_CLKSEL1_TMR0_S_Pos)     /*!<Select TMR0 clock source from low speed crystal */
#define CLK_CLKSEL1_TMR0_S_LIRC   (0x2UL<<CLK_CLKSEL1_TMR0_S_Pos)     /*!<Select TMR0 clock source from low speed oscillator */
#define CLK_CLKSEL1_TMR0_S_EXT    (0x3UL<<CLK_CLKSEL1_TMR0_S_Pos)     /*!<Select TMR0 clock source from external trigger */
#define CLK_CLKSEL1_TMR0_S_HIRC   (0x4UL<<CLK_CLKSEL1_TMR0_S_Pos)     /*!<Select TMR0 clock source from high speed oscillator */

#define CLK_CLKSEL1_PWM0_CH01_S_HXT   (0x0UL<<CLK_CLKSEL1_PWM0_CH01_S_Pos)  /*!<Select PWM0_CH01 clock source from high speed crystal */
#define CLK_CLKSEL1_PWM0_CH01_S_LXT   (0x1UL<<CLK_CLKSEL1_PWM0_CH01_S_Pos)  /*!<Select PWM0_CH01 clock source from low speed crystal */
#define CLK_CLKSEL1_PWM0_CH01_S_HCLK  (0x2UL<<CLK_CLKSEL1_PWM0_CH01_S_Pos)  /*!<Select PWM0_CH01 clock source from HCLK */
#define CLK_CLKSEL1_PWM0_CH01_S_HIRC  (0x3UL<<CLK_CLKSEL1_PWM0_CH01_S_Pos)  /*!<Select PWM0_CH01 clock source from high speed oscillator */

#define CLK_CLKSEL1_PWM0_CH23_S_HXT   (0x0UL<<CLK_CLKSEL1_PWM0_CH23_S_Pos)  /*!<Select PWM0_CH23 clock source from high speed crystal */
#define CLK_CLKSEL1_PWM0_CH23_S_LXT   (0x1UL<<CLK_CLKSEL1_PWM0_CH23_S_Pos)  /*!<Select PWM0_CH23 clock source from low speed crystal */
#define CLK_CLKSEL1_PWM0_CH23_S_HCLK  (0x2UL<<CLK_CLKSEL1_PWM0_CH23_S_Pos)  /*!<Select PWM0_CH23 clock source from HCLK */
#define CLK_CLKSEL1_PWM0_CH23_S_HIRC  (0x3UL<<CLK_CLKSEL1_PWM0_CH23_S_Pos)  /*!<Select PWM0_CH23 clock source from high speed oscillator */

#define CLK_CLKSEL1_ADC_S_HXT     (0x0UL<<CLK_CLKSEL1_ADC_S_Pos)      /*!<Select ADC clock source from high speed crystal */
#define CLK_CLKSEL1_ADC_S_LXT     (0x1UL<<CLK_CLKSEL1_ADC_S_Pos)      /*!<Select ADC clock source from low speed crystal */
#define CLK_CLKSEL1_ADC_S_PLL     (0x2UL<<CLK_CLKSEL1_ADC_S_Pos)      /*!<Select ADC clock source from PLL */
#define CLK_CLKSEL1_ADC_S_HIRC    (0x3UL<<CLK_CLKSEL1_ADC_S_Pos)      /*!<Select ADC clock source from high speed oscillator */

#define CLK_CLKSEL1_UART_S_HXT    (0x0UL<<CLK_CLKSEL1_UART_S_Pos)     /*!<Select UART clock source from high speed crystal */
#define CLK_CLKSEL1_UART_S_LXT    (0x1UL<<CLK_CLKSEL1_UART_S_Pos)     /*!<Select UART clock source from low speed crystal */
#define CLK_CLKSEL1_UART_S_PLL    (0x2UL<<CLK_CLKSEL1_UART_S_Pos)     /*!<Select UART clock source from PLL */
#define CLK_CLKSEL1_UART_S_HIRC   (0x3UL<<CLK_CLKSEL1_UART_S_Pos)     /*!<Select UART clock source from high speed oscillator */

/********************* Bit definition of CLKSEL2 register **********************/
#define CLK_CLKSEL2_SC_S_HXT      (0x0UL<<CLK_CLKSEL2_SC_S_Pos)       /*!<Select SmartCard clock source from HXT */
#define CLK_CLKSEL2_SC_S_PLL      (0x1UL<<CLK_CLKSEL2_SC_S_Pos)       /*!<Select smartCard clock source from PLL */
#define CLK_CLKSEL2_SC_S_HIRC     (0x2UL<<CLK_CLKSEL2_SC_S_Pos)       /*!<Select SmartCard clock source from HIRC */

#define CLK_CLKSEL2_I2S_S_HXT      (0x0UL<<CLK_CLKSEL2_I2S_S_Pos)       /*!<Select I2S clock source from HXT */
#define CLK_CLKSEL2_I2S_S_PLL      (0x1UL<<CLK_CLKSEL2_I2S_S_Pos)       /*!<Select I2S clock source from PLL */
#define CLK_CLKSEL2_I2S_S_HIRC     (0x2UL<<CLK_CLKSEL2_I2S_S_Pos)       /*!<Select I2S clock source from HIRC */

#define CLK_CLKSEL2_TMR3_S_HXT    (0x0UL<<CLK_CLKSEL2_TMR3_S_Pos)     /*!<Select TMR3 clock source from high speed crystal */
#define CLK_CLKSEL2_TMR3_S_LXT    (0x1UL<<CLK_CLKSEL2_TMR3_S_Pos)     /*!<Select TMR3 clock source from low speed crystal */
#define CLK_CLKSEL2_TMR3_S_LIRC   (0x2UL<<CLK_CLKSEL2_TMR3_S_Pos)     /*!<Select TMR3 clock source from low speed oscillator  */
#define CLK_CLKSEL2_TMR3_S_EXT    (0x3UL<<CLK_CLKSEL2_TMR3_S_Pos)     /*!<Select TMR3 clock source from external trigger */
#define CLK_CLKSEL2_TMR3_S_HIRC   (0x4UL<<CLK_CLKSEL2_TMR3_S_Pos)     /*!<Select TMR3 clock source from high speed oscillator */

#define CLK_CLKSEL2_TMR2_S_HXT    (0x0UL<<CLK_CLKSEL2_TMR2_S_Pos)     /*!<Select TMR2 clock source from high speed crystal */
#define CLK_CLKSEL2_TMR2_S_LXT    (0x1UL<<CLK_CLKSEL2_TMR2_S_Pos)     /*!<Select TMR2 clock source from low speed crystal */
#define CLK_CLKSEL2_TMR2_S_LIRC   (0x2UL<<CLK_CLKSEL2_TMR2_S_Pos)     /*!<Select TMR2 clock source from low speed oscillator */
#define CLK_CLKSEL2_TMR2_S_EXT    (0x3UL<<CLK_CLKSEL2_TMR2_S_Pos)     /*!<Select TMR2 clock source from external trigger */
#define CLK_CLKSEL2_TMR2_S_HIRC   (0x4UL<<CLK_CLKSEL2_TMR2_S_Pos)     /*!<Select TMR2 clock source from high speed oscillator */

#define CLK_CLKSEL2_PWM1_CH01_S_HXT   (0x0UL<<CLK_CLKSEL2_PWM1_CH01_S_Pos)  /*!<Select PWM1_CH01 clock source from high speed crystal */
#define CLK_CLKSEL2_PWM1_CH01_S_LXT   (0x1UL<<CLK_CLKSEL2_PWM1_CH01_S_Pos)  /*!<Select PWM1_CH01 clock source from low speed crystal */
#define CLK_CLKSEL2_PWM1_CH01_S_HCLK  (0x2UL<<CLK_CLKSEL2_PWM1_CH01_S_Pos)  /*!<Select PWM1_CH01 clock source from HCLK */
#define CLK_CLKSEL2_PWM1_CH01_S_HIRC  (0x3UL<<CLK_CLKSEL2_PWM1_CH01_S_Pos)  /*!<Select PWM1_CH01 clock source from high speed oscillator */

#define CLK_CLKSEL2_PWM1_CH23_S_HXT   (0x0UL<<CLK_CLKSEL2_PWM1_CH23_S_Pos)  /*!<Select PWM1_CH23 clock source from high speed crystal */
#define CLK_CLKSEL2_PWM1_CH23_S_LXT   (0x1UL<<CLK_CLKSEL2_PWM1_CH23_S_Pos)  /*!<Select PWM1_CH23 clock source from low speed crystal */
#define CLK_CLKSEL2_PWM1_CH23_S_HCLK  (0x2UL<<CLK_CLKSEL2_PWM1_CH23_S_Pos)  /*!<Select PWM1_CH23 clock source from HCLK */
#define CLK_CLKSEL2_PWM1_CH23_S_HIRC  (0x3UL<<CLK_CLKSEL2_PWM1_CH23_S_Pos)  /*!<Select PWM1_CH23 clock source from high speed oscillator */

#define CLK_CLKSEL2_FRQDIV_S_HXT      (0x0UL<<CLK_CLKSEL2_FRQDIV_S_Pos)     /*!<Select FRQDIV clock source from HXT */
#define CLK_CLKSEL2_FRQDIV_S_LXT      (0x1UL<<CLK_CLKSEL2_FRQDIV_S_Pos)     /*!<Select FRQDIV clock source from LXT */
#define CLK_CLKSEL2_FRQDIV_S_HCLK     (0x2UL<<CLK_CLKSEL2_FRQDIV_S_Pos)     /*!<Select FRQDIV clock source from HCLK */
#define CLK_CLKSEL2_FRQDIV_S_HIRC     (0x3UL<<CLK_CLKSEL2_FRQDIV_S_Pos)     /*!<Select FRQDIV clock source from HIRC */

/********************* Bit definition of CLKDIV0 register **********************/
#define CLK_HCLK_CLK_DIVIDER(x)     (((x-1)<< CLK_CLKDIV0_HCLK_N_Pos) & CLK_CLKDIV0_HCLK_N_Msk)  /*!< CLKDIV0 Setting for HCLK clock divider. It could be 1~16 */
#define CLK_USB_CLK_DIVIDER(x)      (((x-1)<< CLK_CLKDIV0_USB_N_Pos) & CLK_CLKDIV0_USB_N_Msk)    /*!< CLKDIV0 Setting for HCLK clock divider. It could be 1~16 */
#define CLK_UART_CLK_DIVIDER(x)     (((x-1)<< CLK_CLKDIV0_UART_N_Pos) & CLK_CLKDIV0_UART_N_Msk)  /*!< CLKDIV0 Setting for UART clock divider. It could be 1~16 */
#define CLK_ADC_CLK_DIVIDER(x)      (((x-1)<< CLK_CLKDIV0_ADC_N_Pos)  & CLK_CLKDIV0_ADC_N_Msk)   /*!< CLKDIV0 Setting for ADC clock divider. It could be 1~256 */
#define CLK_SC0_CLK_DIVIDER(x)      (((x-1)<< CLK_CLKDIV0_SC0_N_Pos)  & CLK_CLKDIV0_SC0_N_Msk)   /*!< CLKDIV0 Setting for SmartCard0 clock divider. It could be 1~16 */
#define CLK_I2S_CLK_DIVIDER(x)      (((x-1)<< CLK_CLKDIV0_I2S_N_Pos)  & CLK_CLKDIV0_I2S_N_Msk)   /*!< CLKDIV0 Setting for I2S clock divider. It could be 1~16 */

/********************* Bit definition of CLKDIV1 register **********************/
#define CLK_SC1_CLK_DIVIDER(x)      (((x-1)<< CLK_CLKDIV1_SC1_N_Pos ) & CLK_CLKDIV1_SC1_N_Msk)   /*!< CLKDIV1 Setting for SmartCard1 clock divider. It could be 1~16 */


/********************* Bit definition of SysTick register **********************/
#define CLK_CLKSEL0_STCLKSEL_HCLK         (1)     /*!< Setting systick clock source as external HCLK */ 
#define CLK_CLKSEL0_STCLKSEL_HCLK_DIV8    (2)     /*!< Setting systick clock source as external HCLK/8 */ 


/********************* Bit definition of PLLCTL register **********************/
#define PLL_IN_12M_OUT_42M_HXT    0x0318
#define PLL_IN_12M_OUT_48M_HXT    0x0320
#define PLL_IN_12M_OUT_84M_HXT    0x0218
#define PLL_IN_12M_OUT_96M_HXT    0x0220
#define PLL_IN_12M_OUT_120M_HXT   0x0108

#define PLL_IN_12M_OUT_42M_HIRC    0x2318
#define PLL_IN_12M_OUT_48M_HIRC    0x2320
#define PLL_IN_12M_OUT_84M_HIRC    0x2218
#define PLL_IN_12M_OUT_96M_HIRC    0x2220
#define PLL_IN_12M_OUT_120M_HIRC   0x2108

#define CLK_PLLCTL_PLL_SRC_HIRC   (0x1UL<<CLK_PLLCTL_PLL_SRC_Pos)    /*!<PLL clock source from high speed oscillator */
#define CLK_PLLCTL_PLL_SRC_HXT    (0x0UL<<CLK_PLLCTL_PLL_SRC_Pos)    /*!<PLL clock source from high speed crystal */

/*---------------------------------------------------------------------------------------------------------*/
/*  MODULE constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define MODULE_APBCLK(x)                   ((x >>31) & 0x1)    /*!< Calculate APBCLK offset on MODULE index */
#define MODULE_CLKSEL(x)                   ((x >>29) & 0x3)    /*!< Calculate CLKSEL offset on MODULE index */
#define MODULE_CLKSEL_Msk(x)               ((x >>25) & 0xf)    /*!< Calculate CLKSEL mask offset on MODULE index */
#define MODULE_CLKSEL_Pos(x)               ((x >>20) & 0x1f)   /*!< Calculate CLKSEL position offset on MODULE index */
#define MODULE_CLKDIV(x)                   ((x >>18) & 0x3)    /*!< Calculate APBCLK CLKDIV on MODULE index */
#define MODULE_CLKDIV_Msk(x)               ((x >>10) & 0xff)   /*!< Calculate CLKDIV mask offset on MODULE index */
#define MODULE_CLKDIV_Pos(x)               ((x >>5 ) & 0x1f)   /*!< Calculate CLKDIV position offset on MODULE index */
#define MODULE_IP_EN_Pos(x)                ((x >>0 ) & 0x1f)   /*!< Calculate APBCLK offset on MODULE index */
#define MODULE_NoMsk                       0x0                 /*!< Not mask on MODULE index */
#define NA                                 MODULE_NoMsk        /*!< Not Available */

#define MODULE_APBCLK_ENC(x)        (((x) & 0x01) << 31)   /*!< MODULE index, 0x0:AHBCLK, 0x1:APBCLK */
#define MODULE_CLKSEL_ENC(x)        (((x) & 0x03) << 29)   /*!< CLKSEL offset on MODULE index, 0x0:CLKSEL0, 0x1:CLKSEL1 0x3 CLKSEL2*/
#define MODULE_CLKSEL_Msk_ENC(x)    (((x) & 0x0f) << 25)   /*!< CLKSEL mask offset on MODULE index */
#define MODULE_CLKSEL_Pos_ENC(x)    (((x) & 0x1f) << 20)   /*!< CLKSEL position offset on MODULE index */
#define MODULE_CLKDIV_ENC(x)        (((x) & 0x03) << 18)   /*!< APBCLK CLKDIV on MODULE index, 0x0:CLKDIV */
#define MODULE_CLKDIV_Msk_ENC(x)    (((x) & 0xff) << 10)   /*!< CLKDIV mask offset on MODULE index */
#define MODULE_CLKDIV_Pos_ENC(x)    (((x) & 0x1f) <<  5)   /*!< CLKDIV position offset on MODULE index */
#define MODULE_IP_EN_Pos_ENC(x)     (((x) & 0x1f) <<  0)   /*!< APBCLK offset on MODULE index */
/*-------------------------------------------------------------------------------------------------------------------------------*/
/*   APBCLK(1) | CLKSEL(2) | CLKSEL_Msk(4) |  CLKSEL_Pos(5) | CLKDIV(2) | CLKDIV_Msk(8) |  CLKDIV_Pos(5)  |  IP_EN_Pos(5)        */
/*-------------------------------------------------------------------------------------------------------------------------------*/
#define TICK_MODULE      ((0UL<<31)|(3<<29)|(MODULE_NoMsk<<25)|( 0<<20)|(1<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_TICK_EN_Pos     ) /*!< TICK Module */
#define SRAM_MODULE      ((0UL<<31)|(3<<29)|(MODULE_NoMsk<<25)|( 0<<20)|(1<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_SRAM_EN_Pos     ) /*!< SRAM Module */
#define EBI_MODULE       ((0UL<<31)|(3<<29)|(MODULE_NoMsk<<25)|( 0<<20)|(1<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_EBI_EN_Pos      ) /*!< EBI Module */
#define ISP_MODULE       ((0UL<<31)|(3<<29)|(MODULE_NoMsk<<25)|( 0<<20)|(1<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_ISP_EN_Pos      ) /*!< ISP Module */
#define DMA_MODULE       ((0UL<<31)|(3<<29)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_DMA_EN_Pos      ) /*!< DMA Module */
#define GPIO_MODULE      ((0UL<<31)|(3<<29)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_GPIO_EN_Pos     ) /*!< GPIO Module */

#define SC1_MODULE       ((1UL<<31)|(2<<29)|(3<<25)           |(18<<20)|(1<<18)|(0xF<<10)         |( 0<<5)|CLK_APBCLK_SC1_EN_Pos      ) /*!< SmartCard1 Module */
#define SC0_MODULE       ((1UL<<31)|(2<<29)|(3<<25)           |(18<<20)|(0<<18)|(0xF<<10)         |(28<<5)|CLK_APBCLK_SC0_EN_Pos      ) /*!< SmartCard0 Module */
#define I2S_MODULE       ((1UL<<31)|(2<<29)|(3<<25)           |(16<<20)|(0<<18)|(0xF<<10)         |(12<<5)|CLK_APBCLK_I2S_EN_Pos      ) /*!< I2S Module */
#define ADC_MODULE       ((1UL<<31)|(1<<29)|(3<<25)           |( 2<<20)|(0<<18)|(0xFF<<10)        |(16<<5)|CLK_APBCLK_ADC_EN_Pos      ) /*!< ADC Module */
#define USBD_MODULE      ((1UL<<31)|(1<<29)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(0xF<<10)         |( 4<<5)|CLK_APBCLK_USBD_EN_Pos     ) /*!< USBD Module */
#define PWM1_CH23_MODULE ((1UL<<31)|(2<<29)|(3<<25)           |( 6<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_PWM1_CH23_EN_Pos) /*!< PWM1 Channel2 and Channel3 Module */
#define PWM1_CH01_MODULE ((1UL<<31)|(2<<29)|(3<<25)           |( 4<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_PWM1_CH01_EN_Pos) /*!< PWM1 Channel0 and Channel1 Module */
#define PWM0_CH23_MODULE ((1UL<<31)|(1<<29)|(3<<25)           |( 6<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_PWM0_CH23_EN_Pos) /*!< PWM0 Channel2 and Channel3 Module */
#define PWM0_CH01_MODULE ((1UL<<31)|(1<<29)|(3<<25)           |( 4<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_PWM0_CH01_EN_Pos) /*!< PWM0 Channel0 and Channel1 Module */
#define UART1_MODULE     ((1UL<<31)|(1<<29)|(3<<25)           |( 0<<20)|(0<<18)|(0xF<<10)         |( 8<<5)|CLK_APBCLK_UART1_EN_Pos    ) /*!< UART1 Module */
#define UART0_MODULE     ((1UL<<31)|(1<<29)|(3<<25)           |( 0<<20)|(0<<18)|(0xF<<10)         |( 8<<5)|CLK_APBCLK_UART0_EN_Pos    ) /*!< UART0 Module */
#define SPI2_MODULE      ((1UL<<31)|(2<<29)|(MODULE_NoMsk<<25)|(20<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_SPI2_EN_Pos     ) /*!< SPI0 Module */
#define SPI1_MODULE      ((1UL<<31)|(2<<29)|(MODULE_NoMsk<<25)|(21<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_SPI1_EN_Pos     ) /*!< SPI1 Module */
#define SPI0_MODULE      ((1UL<<31)|(2<<29)|(MODULE_NoMsk<<25)|(20<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_SPI0_EN_Pos     ) /*!< SPI0 Module */
#define I2C1_MODULE      ((1UL<<31)|(0<<29)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_I2C1_EN_Pos     ) /*!< I2C1 Module */
#define I2C0_MODULE      ((1UL<<31)|(0<<29)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_I2C0_EN_Pos     ) /*!< I2C0 Module */
#define FDIV_MODULE      ((1UL<<31)|(2<<29)|(3<<25)           |( 2<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_FDIV_EN_Pos     ) /*!< Frequency Divider0 Output Module */
#define TMR3_MODULE      ((1UL<<31)|(2<<29)|(7<<25)           |(12<<20)|(1<<18)|(0xF<<10)         |(20<<5)|CLK_APBCLK_TMR3_EN_Pos     ) /*!< Timer3 Module */
#define TMR2_MODULE      ((1UL<<31)|(2<<29)|(7<<25)           |( 8<<20)|(1<<18)|(0xF<<10)         |(16<<5)|CLK_APBCLK_TMR2_EN_Pos     ) /*!< Timer2 Module */
#define TMR1_MODULE      ((1UL<<31)|(1<<29)|(7<<25)           |(12<<20)|(1<<18)|(0xF<<10)         |(12<<5)|CLK_APBCLK_TMR1_EN_Pos     ) /*!< Timer1 Module */
#define TMR0_MODULE      ((1UL<<31)|(1<<29)|(7<<25)           |( 8<<20)|(1<<18)|(0xF<<10)         |( 8<<5)|CLK_APBCLK_TMR0_EN_Pos     ) /*!< Timer0 Module */
#define RTC_MODULE       ((1UL<<31)|(3<<29)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_RTC_EN_Pos      ) /*!< Real-Time-Clock Module */
#define WDT_MODULE       ((1UL<<31)|(3<<29)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK_WDT_EN_Pos      ) /*!< Watchdog Timer Module */
/*@}*/ /* end of group NANO100_CLK_EXPORTED_CONSTANTS */


/** @addtogroup NANO100_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/
void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_GetPLLClockFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
uint32_t CLK_EnablePLL(uint32_t u32PllClkSrc, uint32_t u32PllFreq);
void CLK_DisablePLL(void);
void CLK_SysTickDelay(uint32_t us);
void CLK_EnableSysTick(uint32_t u32ClkSrc, uint32_t u32Count);
void CLK_DisableSysTick(void);
uint32_t CLK_WaitClockReady(uint32_t u32ClkMask);

/*@}*/ /* end of group NANO100_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO100_CLK_Driver */

/*@}*/ /* end of group NANO100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__CLK_H__

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
