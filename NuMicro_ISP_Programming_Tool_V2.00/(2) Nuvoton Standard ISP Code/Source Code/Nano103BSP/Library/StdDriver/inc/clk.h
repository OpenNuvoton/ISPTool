/**************************************************************************//**
 * @file     clk.h
 * @version  V1.00
 * $Revision: 13 $
 * $Date: 16/02/24 9:23a $
 * @brief    NANO103 series CLK driver header file
 *
 * @note
 * Copyright (C) 2015 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __CLK_H__
#define __CLK_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup NANO103_Device_Driver NANO103 Device Driver
  @{
*/

/** @addtogroup NANO103_CLK_Driver CLK Driver
  @{
*/

/** @addtogroup NANO103_CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/


#define FREQ_36MHZ       36000000
#define FREQ_16MHZ       16000000

/********************* Bit definition of PWRCTL register **********************/
#define CLK_PWRCTL_HXT_EN         ((uint32_t)0x00000001)      /*!<Enable high speed crystal */
#define CLK_PWRCTL_LXT_EN         ((uint32_t)0x00000002)      /*!<Enable low speed crystal */
#define CLK_PWRCTL_HIRC0_EN       ((uint32_t)0x00000004)      /*!<Enable internal high speed oscillator 0 */
#define CLK_PWRCTL_LIRC_EN        ((uint32_t)0x00000008)      /*!<Enable internal low speed oscillator */
#define CLK_PWRCTL_DELY_EN        ((uint32_t)0x00000010)      /*!<Enable the wake-up delay counter */
#define CLK_PWRCTL_WAKEINT_EN     ((uint32_t)0x00000020)      /*!<Enable the wake-up interrupt */
#define CLK_PWRCTL_PWRDOWN_EN     ((uint32_t)0x00000040)      /*!<Power down enable bit */
#define CLK_PWRCTL_HXT_HXTSLTYP   ((uint32_t)0x00000100)      /*!<High frequency crystal loop back path Enabled */
#define CLK_PWRCTL_HIRC1_EN       ((uint32_t)0x01000000)      /*!<Enable internal high speed oscillator 1 */
#define CLK_PWRCTL_MIRC_EN        ((uint32_t)0x02000000)      /*!<Enable internal medium speed oscillator */
#define CLK_PWRCTL_HXT_SELXT      ((uint32_t)0x00000100)      /*!<High frequency crystal loop back path Enabled */

#define CLK_PWRCTL_HXT_GAIN_4M       ((uint32_t)0x00000000)   /*!<High frequency crystal Gain control is lower than from 4 MHz */
#define CLK_PWRCTL_HXT_GAIN_4M_8M    ((uint32_t)0x00000400)   /*!<High frequency crystal Gain control is from 4 MHz to 8 MHz */
#define CLK_PWRCTL_HXT_GAIN_8M_12M   ((uint32_t)0x00000800)   /*!<High frequency crystal Gain control is from 8 MHz to 12 MHz */
#define CLK_PWRCTL_HXT_GAIN_12M_16M  ((uint32_t)0x00000C00)   /*!<High frequency crystal Gain control is from 12 MHz to 16 MHz */
#define CLK_PWRCTL_HXT_GAIN_16M_24M  ((uint32_t)0x00001000)   /*!<High frequency crystal Gain control is from 16 MHz to 24 MHz */
#define CLK_PWRCTL_HXT_GAIN_24M_32M  ((uint32_t)0x00001400)   /*!<High frequency crystal Gain control is from 24 MHz to 32 MHz */
#define CLK_PWRCTL_HXT_GAIN_32M_36M  ((uint32_t)0x00001800)   /*!<High frequency crystal Gain control is from 32 MHz to 36 MHz */
#define CLK_PWRCTL_HXT_GAIN_36M      ((uint32_t)0x00001C00)   /*!<High frequency crystal Gain control is higher than 36 MHz */

/********************* Bit definition of AHBCLK register **********************/
#define CLK_AHBCLK_GPIO_EN        ((uint32_t)0x00000001)      /*!<GPIO clock enable */
#define CLK_AHBCLK_DMA_EN         ((uint32_t)0x00000002)      /*!<DMA clock enable */
#define CLK_AHBCLK_ISP_EN         ((uint32_t)0x00000004)      /*!<Flash ISP controller clock enable */
#define CLK_AHBCLK_SRAM_EN        ((uint32_t)0x00000010)      /*!<SRAM Controller Clock Enable */
#define CLK_AHBCLK_TICK_EN        ((uint32_t)0x00000020)      /*!<System Tick Clock Enable */

/********************* Bit definition of APBCLK register **********************/
#define CLK_APBCLK_WDT_EN         ((uint32_t)0x00000001)      /*!<Watchdog clock enable */
#define CLK_APBCLK_RTC_EN         ((uint32_t)0x00000002)      /*!<RTC clock enable */
#define CLK_APBCLK_TMR0_EN        ((uint32_t)0x00000004)      /*!<Timer 0 clock enable */
#define CLK_APBCLK_TMR1_EN        ((uint32_t)0x00000008)      /*!<Timer 1 clock enable */
#define CLK_APBCLK_TMR2_EN        ((uint32_t)0x00000010)      /*!<Timer 2 clock enable */
#define CLK_APBCLK_TMR3_EN        ((uint32_t)0x00000020)      /*!<Timer 3 clock enable */
#define CLK_APBCLK_CLKOC_EN       ((uint32_t)0x00000040)      /*!<Frequency Divider Output clock enable */
#define CLK_APBCLK_I2C0_EN        ((uint32_t)0x00000100)      /*!<I2C 0 clock enable */
#define CLK_APBCLK_I2C1_EN        ((uint32_t)0x00000200)      /*!<I2C 1 clock enable */
#define CLK_APBCLK_ACMP0_EN       ((uint32_t)0x00000800)      /*!<ACMP0 clock Enable Control */
#define CLK_APBCLK_SPI0_EN        ((uint32_t)0x00001000)      /*!<SPI 0 clock enable */
#define CLK_APBCLK_SPI1_EN        ((uint32_t)0x00002000)      /*!<SPI 1 clock enable */
#define CLK_APBCLK_SPI2_EN        ((uint32_t)0x00004000)      /*!<SPI 2 clock enable */
#define CLK_APBCLK_SPI3_EN        ((uint32_t)0x00008000)      /*!<SPI 3 clock enable */
#define CLK_APBCLK_UART0_EN       ((uint32_t)0x00010000)      /*!<UART 0 clock enable */
#define CLK_APBCLK_UART1_EN       ((uint32_t)0x00020000)      /*!<UART 1 clock enable */
#define CLK_APBCLK_PWM0_EN        ((uint32_t)0x00100000)      /*!<PWM0 clock Enable Control */
#define CLK_APBCLK_ADC_EN         ((uint32_t)0x10000000)      /*!<ADC clock enable */
#define CLK_APBCLK_SC0_EN         ((uint32_t)0x40000000)      /*!<SmartCard 0 Clock Enable Control */
#define CLK_APBCLK_SC1_EN         ((uint32_t)0x80000000)      /*!<SmartCard 1 Clock Enable Control */

/********************* Bit definition of STATUS register **********************/
#define CLK_CLKSTATUS_HXT_STB     ((uint32_t)0x00000001)      /*!<External high speed crystal clock source stable flag */
#define CLK_CLKSTATUS_LXT_STB     ((uint32_t)0x00000002)      /*!<External low speed crystal clock source stable flag */
#define CLK_CLKSTATUS_PLL_STB     ((uint32_t)0x00000004)      /*!<Internal PLL clock source stable flag */
#define CLK_CLKSTATUS_LIRC_STB    ((uint32_t)0x00000008)      /*!<Internal low speed oscillator clock source stable flag */
#define CLK_CLKSTATUS_HIRC0_STB   ((uint32_t)0x00000010)      /*!<Internal high speed oscillator 0 clock source stable flag */
#define CLK_CLKSTATUS_HIRC1_STB   ((uint32_t)0x00000020)      /*!<Internal high speed oscillator 1 clock source stable flag */
#define CLK_CLKSTATUS_MIRC_STB    ((uint32_t)0x00000040)      /*!<Internal medium speed oscillator clock source stable flag */
#define CLK_CLKSTATUS_CLK_SW_FAIL ((uint32_t)0x00000080)      /*!<Clock switch fail flag */

/********************* Bit definition of PLLCTL register **********************/
#define CLK_PLLCTL_PD             ((uint32_t)0x00010000)      /*!<PLL Power down mode */
#define CLK_PLLCTL_PLL_SRC_HXT    ((uint32_t)(0x00000000))    /*!< For PLL clock source is HXT */
#define CLK_PLLCTL_PLL_SRC_HIRC   ((uint32_t)(0x00020000))    /*!< For PLL clock source is HIRC */
#define CLK_PLLCTL_PLL_SRC_MIRC   ((uint32_t)(0x00040000))    /*!< For PLL clock source is MIRC */

#define CLK_PLL_SRC_N(x)          (((x)-1)<<8)                   /*!< PLL Input Source Divider */
#define CLK_PLL_MLP(x)            ((x)<<0)                       /*!< PLL Multiple */ 
#if (__HXT == 12000000)
#define CLK_PLLCTL_36MHz_HXT   (CLK_PLLCTL_PLL_SRC_HXT  | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(36)) /*!< Predefined PLLCTL setting for 36MHz PLL output with 12MHz X'tal */
#define CLK_PLLCTL_32MHz_HXT   (CLK_PLLCTL_PLL_SRC_HXT  | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(32)) /*!< Predefined PLLCTL setting for 32MHz PLL output with 12MHz X'tal */
#define CLK_PLLCTL_28MHz_HXT   (CLK_PLLCTL_PLL_SRC_HXT  | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(28)) /*!< Predefined PLLCTL setting for 28MHz PLL output with 12MHz X'tal */
#define CLK_PLLCTL_24MHz_HXT   (CLK_PLLCTL_PLL_SRC_HXT  | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(24)) /*!< Predefined PLLCTL setting for 24MHz PLL output with 12MHz X'tal */
#define CLK_PLLCTL_22MHz_HXT   (CLK_PLLCTL_PLL_SRC_HXT  | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(22)) /*!< Predefined PLLCTL setting for 22MHz PLL output with 12MHz X'tal */
#define CLK_PLLCTL_16MHz_HXT   (CLK_PLLCTL_PLL_SRC_HXT  | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(16)) /*!< Predefined PLLCTL setting for 16MHz PLL output with 12MHz X'tal */
#else
# error "The PLL pre-definitions are only valid when external crystal is 12MHz"
#endif
#define CLK_PLLCTL_36MHz_HIRC0  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(36)) /*!< Predefined PLLCTL setting for 36MHz PLL output with 12MHz HIRC0 */
#define CLK_PLLCTL_32MHz_HIRC0  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(32)) /*!< Predefined PLLCTL setting for 32MHz PLL output with 12MHz HIRC0 */
#define CLK_PLLCTL_28MHz_HIRC0  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(28)) /*!< Predefined PLLCTL setting for 28MHz PLL output with 12MHz HIRC0 */
#define CLK_PLLCTL_24MHz_HIRC0  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(24)) /*!< Predefined PLLCTL setting for 24MHz PLL output with 12MHz HIRC0 */
#define CLK_PLLCTL_22MHz_HIRC0  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(22)) /*!< Predefined PLLCTL setting for 22MHz PLL output with 12MHz HIRC0 */
#define CLK_PLLCTL_16MHz_HIRC0  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(12) | CLK_PLL_MLP(16)) /*!< Predefined PLLCTL setting for 16MHz PLL output with 12MHz HIRC0 */

#define CLK_PLLCTL_36MHz_HIRC1  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(36) | CLK_PLL_MLP(36)) /*!< Predefined PLLCTL setting for 36MHz PLL output with 36MHz HIRC1 */
#define CLK_PLLCTL_32MHz_HIRC1  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(36) | CLK_PLL_MLP(32)) /*!< Predefined PLLCTL setting for 32MHz PLL output with 36MHz HIRC1 */
#define CLK_PLLCTL_28MHz_HIRC1  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(36) | CLK_PLL_MLP(28)) /*!< Predefined PLLCTL setting for 28MHz PLL output with 36MHz HIRC1 */
#define CLK_PLLCTL_24MHz_HIRC1  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(36) | CLK_PLL_MLP(24)) /*!< Predefined PLLCTL setting for 24MHz PLL output with 36MHz HIRC1 */
#define CLK_PLLCTL_22MHz_HIRC1  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(36) | CLK_PLL_MLP(22)) /*!< Predefined PLLCTL setting for 22MHz PLL output with 36MHz HIRC1 */
#define CLK_PLLCTL_16MHz_HIRC1  (CLK_PLLCTL_PLL_SRC_HIRC | CLK_PLL_SRC_N(36) | CLK_PLL_MLP(16)) /*!< Predefined PLLCTL setting for 16MHz PLL output with 36MHz HIRC1 */

#define CLK_PLLCTL_36MHz_MIRC  (CLK_PLLCTL_PLL_SRC_MIRC | CLK_PLL_SRC_N(4) | CLK_PLL_MLP(36))  /*!< Predefined PLLCTL setting for 36MHz PLL output with 4MHz MIRC */
#define CLK_PLLCTL_32MHz_MIRC  (CLK_PLLCTL_PLL_SRC_MIRC | CLK_PLL_SRC_N(4) | CLK_PLL_MLP(32))  /*!< Predefined PLLCTL setting for 32MHz PLL output with 4MHz MIRC */
#define CLK_PLLCTL_28MHz_MIRC  (CLK_PLLCTL_PLL_SRC_MIRC | CLK_PLL_SRC_N(4) | CLK_PLL_MLP(28))  /*!< Predefined PLLCTL setting for 28MHz PLL output with 4MHz MIRC */
#define CLK_PLLCTL_24MHz_MIRC  (CLK_PLLCTL_PLL_SRC_MIRC | CLK_PLL_SRC_N(4) | CLK_PLL_MLP(24))  /*!< Predefined PLLCTL setting for 24MHz PLL output with 4MHz MIRC */
#define CLK_PLLCTL_22MHz_MIRC  (CLK_PLLCTL_PLL_SRC_MIRC | CLK_PLL_SRC_N(4) | CLK_PLL_MLP(22))  /*!< Predefined PLLCTL setting for 22MHz PLL output with 4MHz MIRC */
#define CLK_PLLCTL_16MHz_MIRC  (CLK_PLLCTL_PLL_SRC_MIRC | CLK_PLL_SRC_N(4) | CLK_PLL_MLP(16))  /*!< Predefined PLLCTL setting for 16MHz PLL output with 4MHz MIRC */

/********************* Bit definition of CLKSEL0 register **********************/
#define CLK_CLKSEL0_HCLKSEL_HXT          (0x0UL<<CLK_CLKSEL0_HCLKSEL_Pos)         /*!<Select HCLK clock source from high speed crystal */
#define CLK_CLKSEL0_HCLKSEL_LXT          (0x1UL<<CLK_CLKSEL0_HCLKSEL_Pos)         /*!<Select HCLK clock source from low speed crystal */
#define CLK_CLKSEL0_HCLKSEL_PLL          (0x2UL<<CLK_CLKSEL0_HCLKSEL_Pos)         /*!<Select HCLK clock source from PLL */ 
#define CLK_CLKSEL0_HCLKSEL_LIRC         (0x3UL<<CLK_CLKSEL0_HCLKSEL_Pos)         /*!<Select HCLK clock source from low speed oscillator */
#define CLK_CLKSEL0_HCLKSEL_HIRC         (0x4UL<<CLK_CLKSEL0_HCLKSEL_Pos)         /*!<Select HCLK clock source from high speed oscillator */
#define CLK_CLKSEL0_HCLKSEL_HIRC0        (0x4UL<<CLK_CLKSEL0_HCLKSEL_Pos)         /*!<Select HCLK clock source from high speed oscillator */
#define CLK_CLKSEL0_HCLKSEL_HIRC1        (0xCUL<<CLK_CLKSEL0_HCLKSEL_Pos)         /*!<Select HCLK clock source from high speed oscillator */
#define CLK_CLKSEL0_HCLKSEL_MIRC         (0x5UL<<CLK_CLKSEL0_HCLKSEL_Pos)         /*!<Select HCLK clock source from medium speed oscillator */
#define CLK_CLKSEL0_ISPSEL_HIRC          (0x0UL<<CLK_CLKSEL0_ISPSEL_Pos)          /*!<Select ISP clock source from high speed oscillator */
#define CLK_CLKSEL0_ISPSEL_MIRC          (0x1UL<<CLK_CLKSEL0_ISPSEL_Pos)          /*!<Select ISP clock source from medium speed oscillator */
/********************* Bit definition of CLKSEL1 register **********************/
#define CLK_CLKSEL1_UART0SEL_HXT         (0x0UL<<CLK_CLKSEL1_UART0SEL_Pos)        /*!<Select UART0 clock source from high speed crystal */
#define CLK_CLKSEL1_UART0SEL_LXT         (0x1UL<<CLK_CLKSEL1_UART0SEL_Pos)        /*!<Select UART0 clock source from low speed crystal */
#define CLK_CLKSEL1_UART0SEL_PLL         (0x2UL<<CLK_CLKSEL1_UART0SEL_Pos)        /*!<Select UART0 clock source from PLL */
#define CLK_CLKSEL1_UART0SEL_HIRC        (0x3UL<<CLK_CLKSEL1_UART0SEL_Pos)        /*!<Select UART0 clock source from high speed oscillator */
#define CLK_CLKSEL1_UART0SEL_MIRC        (0x4UL<<CLK_CLKSEL1_UART0SEL_Pos)        /*!<Select UART0 clock source from medium speed oscillator */
#define CLK_CLKSEL1_PWM0SEL_PLL          (0x0UL<<CLK_CLKSEL1_PWM0SEL_Pos)         /*!<Select PWM0 clock source from PLL */ 
#define CLK_CLKSEL1_PWM0SEL_PCLK0        (0x1UL<<CLK_CLKSEL1_PWM0SEL_Pos)         /*!<Select PWM0 clock source from PCLK0 */
#define CLK_CLKSEL1_TMR0SEL_HXT          (0x0UL<<CLK_CLKSEL1_TMR0SEL_Pos)         /*!<Select TMR0 clock source from high speed crystal */
#define CLK_CLKSEL1_TMR0SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR0SEL_Pos)         /*!<Select TMR0 clock source from low speed crystal */
#define CLK_CLKSEL1_TMR0SEL_LIRC         (0x2UL<<CLK_CLKSEL1_TMR0SEL_Pos)         /*!<Select TMR0 clock source from low speed oscillator */
#define CLK_CLKSEL1_TMR0SEL_HIRC         (0x4UL<<CLK_CLKSEL1_TMR0SEL_Pos)         /*!<Select TMR0 clock source from high speed oscillator */
#define CLK_CLKSEL1_TMR0SEL_MIRC         (0x5UL<<CLK_CLKSEL1_TMR0SEL_Pos)         /*!<Select TMR0 clock source from medium speed oscillator */
#define CLK_CLKSEL1_TMR0SEL_EXT          (0x3UL<<CLK_CLKSEL1_TMR0SEL_Pos)         /*!<Select TMR0 clock source from external trigger */
#define CLK_CLKSEL1_TMR0SEL_HCLK         (0x6UL<<CLK_CLKSEL1_TMR0SEL_Pos)         /*!<Select TMR0 clock source from HCLK */
#define CLK_CLKSEL1_TMR1SEL_HXT          (0x0UL<<CLK_CLKSEL1_TMR1SEL_Pos)         /*!<Select TMR1 clock source from high speed crystal */
#define CLK_CLKSEL1_TMR1SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR1SEL_Pos)         /*!<Select TMR1 clock source from low speed crystal */
#define CLK_CLKSEL1_TMR1SEL_LIRC         (0x2UL<<CLK_CLKSEL1_TMR1SEL_Pos)         /*!<Select TMR1 clock source from low speed oscillator */
#define CLK_CLKSEL1_TMR1SEL_HIRC         (0x4UL<<CLK_CLKSEL1_TMR1SEL_Pos)         /*!<Select TMR1 clock source from high speed oscillator */
#define CLK_CLKSEL1_TMR1SEL_MIRC         (0x5UL<<CLK_CLKSEL1_TMR1SEL_Pos)         /*!<Select TMR1 clock source from medium speed oscillator */
#define CLK_CLKSEL1_TMR1SEL_EXT          (0x3UL<<CLK_CLKSEL1_TMR1SEL_Pos)         /*!<Select TMR1 clock source from external trigger */
#define CLK_CLKSEL1_TMR1SEL_HCLK         (0x6UL<<CLK_CLKSEL1_TMR1SEL_Pos)         /*!<Select TMR1 clock source from HCLK */
#define CLK_CLKSEL1_ADCSEL_HXT           (0x0UL<<CLK_CLKSEL1_ADCSEL_Pos)          /*!<Select ADC  clock source from high speed crystal */
#define CLK_CLKSEL1_ADCSEL_LXT           (0x1UL<<CLK_CLKSEL1_ADCSEL_Pos)          /*!<Select ADC  clock source from low speed crystal */
#define CLK_CLKSEL1_ADCSEL_PLL           (0x2UL<<CLK_CLKSEL1_ADCSEL_Pos)          /*!<Select ADC  clock source from PLL */  
#define CLK_CLKSEL1_ADCSEL_HIRC          (0x3UL<<CLK_CLKSEL1_ADCSEL_Pos)          /*!<Select ADC  clock source from high speed oscillator   */
#define CLK_CLKSEL1_ADCSEL_MIRC          (0x4UL<<CLK_CLKSEL1_ADCSEL_Pos)          /*!<Select ADC  clock source from medium speed oscillator */
#define CLK_CLKSEL1_ADCSEL_HCLK          (0x5UL<<CLK_CLKSEL1_ADCSEL_Pos)          /*!<Select ADC  clock source from HCLK */ 
#define CLK_CLKSEL1_SPI0SEL_HXT          (0x2UL<<CLK_CLKSEL1_SPI0SEL_Pos)         /*!<Select SPI0 clock source from high speed crystal */
#define CLK_CLKSEL1_SPI0SEL_PLL          (0x0UL<<CLK_CLKSEL1_SPI0SEL_Pos)         /*!<Select SPI0 clock source from PLL */ 
#define CLK_CLKSEL1_SPI0SEL_HIRC         (0x3UL<<CLK_CLKSEL1_SPI0SEL_Pos)         /*!<Select SPI0 clock source from high speed oscillator */
#define CLK_CLKSEL1_SPI0SEL_HCLK         (0x1UL<<CLK_CLKSEL1_SPI0SEL_Pos)         /*!<Select SPI0 clock source from HCLK */
#define CLK_CLKSEL1_SPI2SEL_HXT          (0x2UL<<CLK_CLKSEL1_SPI2SEL_Pos)         /*!<Select SPI2 clock source from high speed crystal */
#define CLK_CLKSEL1_SPI2SEL_PLL          (0x0UL<<CLK_CLKSEL1_SPI2SEL_Pos)         /*!<Select SPI2 clock source from PLL */ 
#define CLK_CLKSEL1_SPI2SEL_HIRC         (0x3UL<<CLK_CLKSEL1_SPI2SEL_Pos)         /*!<Select SPI2 clock source from high speed oscillator */
#define CLK_CLKSEL1_SPI2SEL_HCLK         (0x1UL<<CLK_CLKSEL1_SPI2SEL_Pos)         /*!<Select SPI2 clock source from HCLK */
#define CLK_CLKSEL1_WDTSEL_LXT           (0x1UL<<CLK_CLKSEL1_WDTSEL_Pos)          /*!<Select WDT  clock source from low speed crystal */
#define CLK_CLKSEL1_WDTSEL_LIRC          (0x3UL<<CLK_CLKSEL1_WDTSEL_Pos)          /*!<Select WDT  clock source from low speed oscillator */
#define CLK_CLKSEL1_WDTSEL_HCLKDIV2048   (0x2UL<<CLK_CLKSEL1_WDTSEL_Pos)          /*!<Select WDT  clock source from HCLK/2048 */
#define CLK_CLKSEL1_WWDTSEL_LIRC         (0x3UL<<CLK_CLKSEL1_WWDTSEL_Pos)         /*!<Select WWDT clock source from low speed oscillator */
#define CLK_CLKSEL1_WWDTSEL_HCLKDIV2048  (0x2UL<<CLK_CLKSEL1_WWDTSEL_Pos)         /*!<Select WWDT clock source from HCLK/2048 */
/********************* Bit definition of CLKSEL2 register **********************/
#define CLK_CLKSEL2_UART1SEL_HXT         (0x0UL<<CLK_CLKSEL2_UART1SEL_Pos)        /*!<Select UART1 clock source from high speed crystal */
#define CLK_CLKSEL2_UART1SEL_LXT         (0x1UL<<CLK_CLKSEL2_UART1SEL_Pos)        /*!<Select UART1 clock source from low speed crystal  */
#define CLK_CLKSEL2_UART1SEL_PLL         (0x2UL<<CLK_CLKSEL2_UART1SEL_Pos)        /*!<Select UART1 clock source from PLL */
#define CLK_CLKSEL2_UART1SEL_HIRC        (0x3UL<<CLK_CLKSEL2_UART1SEL_Pos)        /*!<Select UART1 clock source from high speed oscillator   */
#define CLK_CLKSEL2_UART1SEL_MIRC        (0x4UL<<CLK_CLKSEL2_UART1SEL_Pos)        /*!<Select UART1 clock source from medium speed oscillator */
#define CLK_CLKSEL2_CLKOSEL_HXT          (0x0UL<<CLK_CLKSEL2_CLKOSEL_Pos)         /*!<Select CLKO clock source from high speed crystal */
#define CLK_CLKSEL2_CLKOSEL_LXT          (0x1UL<<CLK_CLKSEL2_CLKOSEL_Pos)         /*!<Select CLKO clock source from low speed crystal  */
#define CLK_CLKSEL2_CLKOSEL_HCLK         (0x2UL<<CLK_CLKSEL2_CLKOSEL_Pos)         /*!<Select CLKO clock source from HCLK */
#define CLK_CLKSEL2_CLKOSEL_HIRC         (0x3UL<<CLK_CLKSEL2_CLKOSEL_Pos)         /*!<Select CLKO clock source from high speed oscillator */
#define CLK_CLKSEL2_CLKOSEL_MIRC         (0x4UL<<CLK_CLKSEL2_CLKOSEL_Pos)         /*!<Select CLKO clock source from medium speed oscillator */
#define CLK_CLKSEL2_TMR2SEL_HXT          (0x0UL<<CLK_CLKSEL2_TMR2SEL_Pos)         /*!<Select TMR2 clock source from high speed crystal */
#define CLK_CLKSEL2_TMR2SEL_LXT          (0x1UL<<CLK_CLKSEL2_TMR2SEL_Pos)         /*!<Select TMR2 clock source from low speed crystal  */
#define CLK_CLKSEL2_TMR2SEL_LIRC         (0x2UL<<CLK_CLKSEL2_TMR2SEL_Pos)         /*!<Select TMR2 clock source from low speed oscillator */
#define CLK_CLKSEL2_TMR2SEL_HIRC         (0x4UL<<CLK_CLKSEL2_TMR2SEL_Pos)         /*!<Select TMR2 clock source from high speed oscillator */
#define CLK_CLKSEL2_TMR2SEL_MIRC         (0x5UL<<CLK_CLKSEL2_TMR2SEL_Pos)         /*!<Select TMR2 clock source from medium speed oscillator */
#define CLK_CLKSEL2_TMR2SEL_EXT          (0x3UL<<CLK_CLKSEL2_TMR2SEL_Pos)         /*!<Select TMR2 clock source from external trigger */
#define CLK_CLKSEL2_TMR2SEL_HCLK         (0x6UL<<CLK_CLKSEL2_TMR2SEL_Pos)         /*!<Select TMR2 clock source from HCLK */
#define CLK_CLKSEL2_TMR3SEL_HXT          (0x0UL<<CLK_CLKSEL2_TMR3SEL_Pos)         /*!<Select TMR3 clock source from high speed crystal */
#define CLK_CLKSEL2_TMR3SEL_LXT          (0x1UL<<CLK_CLKSEL2_TMR3SEL_Pos)         /*!<Select TMR3 clock source from low speed crystal  */
#define CLK_CLKSEL2_TMR3SEL_LIRC         (0x2UL<<CLK_CLKSEL2_TMR3SEL_Pos)         /*!<Select TMR3 clock source from low speed oscillator    */
#define CLK_CLKSEL2_TMR3SEL_HIRC         (0x4UL<<CLK_CLKSEL2_TMR3SEL_Pos)         /*!<Select TMR3 clock source from high speed oscillator   */
#define CLK_CLKSEL2_TMR3SEL_MIRC         (0x5UL<<CLK_CLKSEL2_TMR3SEL_Pos)         /*!<Select TMR3 clock source from medium speed oscillator */
#define CLK_CLKSEL2_TMR3SEL_EXT          (0x3UL<<CLK_CLKSEL2_TMR3SEL_Pos)         /*!<Select TMR3 clock source from external trigger  */
#define CLK_CLKSEL2_TMR3SEL_HCLK         (0x6UL<<CLK_CLKSEL2_TMR3SEL_Pos)         /*!<Select TMR3 clock source from HCLK */
#define CLK_CLKSEL2_SC0SEL_HXT           (0x0UL<<CLK_CLKSEL2_SC0SEL_Pos)          /*!<Select SC0 clock source from high speed crystal */
#define CLK_CLKSEL2_SC0SEL_PLL           (0x1UL<<CLK_CLKSEL2_SC0SEL_Pos)          /*!<Select SC0 clock source from PLL */  
#define CLK_CLKSEL2_SC0SEL_HIRC          (0x2UL<<CLK_CLKSEL2_SC0SEL_Pos)          /*!<Select SC0 clock source from high speed oscillator   */
#define CLK_CLKSEL2_SC0SEL_MIRC          (0x3UL<<CLK_CLKSEL2_SC0SEL_Pos)          /*!<Select SC0 clock source from medium speed oscillator */
#define CLK_CLKSEL2_SC0SEL_HCLK          (0x4UL<<CLK_CLKSEL2_SC0SEL_Pos)          /*!<Select SC0 clock source from HCLK */ 
#define CLK_CLKSEL2_SC1SEL_HXT           (0x0UL<<CLK_CLKSEL2_SC1SEL_Pos)          /*!<Select SC1 clock source from high speed crystal      */
#define CLK_CLKSEL2_SC1SEL_PLL           (0x1UL<<CLK_CLKSEL2_SC1SEL_Pos)          /*!<Select SC1 clock source from PLL */  
#define CLK_CLKSEL2_SC1SEL_HIRC          (0x2UL<<CLK_CLKSEL2_SC1SEL_Pos)          /*!<Select SC1 clock source from high speed oscillator   */
#define CLK_CLKSEL2_SC1SEL_MIRC          (0x3UL<<CLK_CLKSEL2_SC1SEL_Pos)          /*!<Select SC1 clock source from medium speed oscillator */
#define CLK_CLKSEL2_SC1SEL_HCLK          (0x4UL<<CLK_CLKSEL2_SC1SEL_Pos)          /*!<Select SC1 clock source from HCLK */ 
#define CLK_CLKSEL2_SPI1SEL_HXT          (0x2UL<<CLK_CLKSEL2_SPI1SEL_Pos)         /*!<Select SPI1 clock source from high speed crystal */
#define CLK_CLKSEL2_SPI1SEL_PLL          (0x0UL<<CLK_CLKSEL2_SPI1SEL_Pos)         /*!<Select SPI1 clock source from PLL */ 
#define CLK_CLKSEL2_SPI1SEL_HIRC         (0x3UL<<CLK_CLKSEL2_SPI1SEL_Pos)         /*!<Select SPI1 clock source from high speed oscillator */
#define CLK_CLKSEL2_SPI1SEL_HCLK         (0x1UL<<CLK_CLKSEL2_SPI1SEL_Pos)         /*!<Select SPI1 clock source from HCLK */
#define CLK_CLKSEL2_SPI3SEL_HXT          (0x2UL<<CLK_CLKSEL2_SPI3SEL_Pos)         /*!<Select SPI3 clock source from high speed crystal */
#define CLK_CLKSEL2_SPI3SEL_PLL          (0x0UL<<CLK_CLKSEL2_SPI3SEL_Pos)         /*!<Select SPI3 clock source from PLL */ 
#define CLK_CLKSEL2_SPI3SEL_HIRC         (0x3UL<<CLK_CLKSEL2_SPI3SEL_Pos)         /*!<Select SPI3 clock source from high speed oscillator */
#define CLK_CLKSEL2_SPI3SEL_HCLK         (0x1UL<<CLK_CLKSEL2_SPI3SEL_Pos)         /*!<Select SPI3 clock source from HCLK */

/********************* Bit definition of APBDIV register **********************/
#define CLK_APB0DIV_HCLK                 (0x0UL<<CLK_APBDIV_APB0DIV_Pos)          /*!<Select PCLK0 clock source from HCLK */
#define CLK_APB0DIV_1_2HCLK              (0x1UL<<CLK_APBDIV_APB0DIV_Pos)          /*!<Select PCLK0 clock source from 1/2HCLK */
#define CLK_APB0DIV_1_4HCLK              (0x2UL<<CLK_APBDIV_APB0DIV_Pos)          /*!<Select PCLK0 clock source from 1/4HCLK */
#define CLK_APB0DIV_1_8HCLK              (0x3UL<<CLK_APBDIV_APB0DIV_Pos)          /*!<Select PCLK0 clock source from 1/8HCLK */
#define CLK_APB0DIV_1_16HCLK             (0x4UL<<CLK_APBDIV_APB0DIV_Pos)          /*!<Select PCLK0 clock source from 1/16HCLK */
#define CLK_APB1DIV_HCLK                 (0x0UL<<CLK_APBDIV_APB1DIV_Pos)          /*!<Select PCLK1 clock source from HCLK */
#define CLK_APB1DIV_1_2HCLK              (0x1UL<<CLK_APBDIV_APB1DIV_Pos)          /*!<Select PCLK1 clock source from 1/2HCLK */
#define CLK_APB1DIV_1_4HCLK              (0x2UL<<CLK_APBDIV_APB1DIV_Pos)          /*!<Select PCLK1 clock source from 1/4HCLK */
#define CLK_APB1DIV_1_8HCLK              (0x3UL<<CLK_APBDIV_APB1DIV_Pos)          /*!<Select PCLK1 clock source from 1/8HCLK */
#define CLK_APB1DIV_1_16HCLK             (0x4UL<<CLK_APBDIV_APB1DIV_Pos)          /*!<Select PCLK1 clock source from 1/16HCLK */

/********************* Bit definition of CLKDIV0/CLKDIV1 register **********************/
#define CLK_HCLK_CLK_DIVIDER(x)        ((((uint32_t)x-1)<<CLK_CLKDIV0_HCLKDIV_Pos) & CLK_CLKDIV0_HCLKDIV_Msk)          /* CLKDIV0 Setting for HCLK clock divider. It could be 1~16*/
#define CLK_UART0_CLK_DIVIDER(x)       ((((uint32_t)x-1)<<CLK_CLKDIV0_UART0DIV_Pos)& CLK_CLKDIV0_UART0DIV_Msk)         /* CLKDIV0 Setting for UART0 clock divider. It could be 1~16*/
#define CLK_TMR0_CLK_DIVIDER(x)        ((((uint32_t)x-1)<<CLK_CLKDIV1_TMR0DIV_Pos) & CLK_CLKDIV1_TMR0DIV_Msk)          /* CLKDIV1 Setting for TMR0 clock divider. It could be 1~16*/
#define CLK_TMR1_CLK_DIVIDER(x)        ((((uint32_t)x-1)<<CLK_CLKDIV1_TMR1DIV_Pos) & CLK_CLKDIV1_TMR1DIV_Msk)          /* CLKDIV1 Setting for TMR1 clock divider. It could be 1~16*/
#define CLK_ADC_CLK_DIVIDER(x)         ((((uint32_t)x-1)<<CLK_CLKDIV0_ADCDIV_Pos)  & CLK_CLKDIV0_ADCDIV_Msk)           /* CLKDIV0 Setting for ADC clock divider. It could be 1~256*/
#define CLK_UART1_CLK_DIVIDER(x)       ((((uint32_t)x-1)<<CLK_CLKDIV0_UART1DIV_Pos)& CLK_CLKDIV0_UART1DIV_Msk)         /* CLKDIV0 Setting for UART1 clock divider. It could be 1~16*/
#define CLK_TMR2_CLK_DIVIDER(x)        ((((uint32_t)x-1)<<CLK_CLKDIV1_TMR2DIV_Pos) & CLK_CLKDIV1_TMR2DIV_Msk)          /* CLKDIV1 Setting for TMR2 clock divider. It could be 1~16*/
#define CLK_TMR3_CLK_DIVIDER(x)        ((((uint32_t)x-1)<<CLK_CLKDIV1_TMR3DIV_Pos) & CLK_CLKDIV1_TMR3DIV_Msk)          /* CLKDIV1 Setting for TMR3 clock divider. It could be 1~16*/
#define CLK_SC0_CLK_DIVIDER(x)         ((((uint32_t)x-1)<<CLK_CLKDIV0_SC0DIV_Pos)  & CLK_CLKDIV0_SC0DIV_Msk)           /* CLKDIV0 Setting for SC0 clock divider. It could be 1~16*/
#define CLK_SC1_CLK_DIVIDER(x)         ((((uint32_t)x-1)<<CLK_CLKDIV1_SC1DIV_Pos)  & CLK_CLKDIV1_SC1DIV_Msk)           /* CLKDIV1 Setting for SC1 clock divider. It could be 1~16*/

/********************* Bit definition of SysTick register **********************/
#define CLK_CLKSEL0_STCLKSEL_HCLK         (1)     /*!< Setting systick clock source as external HCLK */ 
#define CLK_CLKSEL0_STCLKSEL_HCLK_DIV8    (2)     /*!< Setting systick clock source as external HCLK/8 */ 

/********************* Bit definition of CLKOCTL register **********************/
#define CLK_CLKO_EN           ((uint32_t)0x00000010)    /*!<Frequency divider enable bit */

/********************* Bit definition of WK_INTSTS register **********************/
#define CLK_WK_INTSTS_IS      ((uint32_t)0x00000001)    /*!<Wake-up Interrupt Status in chip Power-down Mode */


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
/*   AHBCLK/APBCLK(1) | CLKSEL(2) | CLKSEL_Msk(4) |  CLKSEL_Pos(5) | CLKDIV(2) | CLKDIV_Msk(8) |  CLKDIV_Pos(5)  |  IP_EN_Pos(5) */
/*-------------------------------------------------------------------------------------------------------------------------------*/
#define GPIO_MODULE      (( 0UL<<31)|( 3<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 0<<0)) /*!< GPIO Module  \hideinitializer */
#define PDMA_MODULE      (( 0UL<<31)|( 3<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 1<<0)) /*!< PDMA Module  \hideinitializer */
#define ISP_MODULE       (( 0UL<<31)|( 0<<29)|(            1<<25)|( 4<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 2<<0)) /*!< ISP Module  \hideinitializer */
#define SRAM_MODULE      (( 0UL<<31)|( 3<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 4<<0)) /*!< SRAM Module  \hideinitializer */
#define STC_MODULE       (( 0UL<<31)|( 3<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 5<<0)) /*!< STC Module  \hideinitializer */
#define WDT_MODULE       (( 1UL<<31)|( 1<<29)|(            3<<25)|(28<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 0<<0)) /*!< WDT Module  \hideinitializer */
#define WWDT_MODULE      (( 1UL<<31)|( 1<<29)|(            3<<25)|(30<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 0<<0)) /*!< WWDT Module  \hideinitializer */
#define RTC_MODULE       (( 1UL<<31)|( 3<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 1<<0)) /*!< RTC Module  \hideinitializer */
#define TMR0_MODULE      (( 1UL<<31)|( 1<<29)|(            7<<25)|( 8<<20)|( 1<<18)|(          0xF<<10)|( 8<<5)|( 2<<0)) /*!< TMR0 Module  \hideinitializer */
#define TMR1_MODULE      (( 1UL<<31)|( 1<<29)|(            7<<25)|(12<<20)|( 1<<18)|(          0xF<<10)|(12<<5)|( 3<<0)) /*!< TMR1 Module  \hideinitializer */
#define TMR2_MODULE      (( 1UL<<31)|( 2<<29)|(            7<<25)|( 8<<20)|( 1<<18)|(          0xF<<10)|(16<<5)|( 4<<0)) /*!< TMR2 Module  \hideinitializer */
#define TMR3_MODULE      (( 1UL<<31)|( 2<<29)|(            7<<25)|(12<<20)|( 1<<18)|(          0xF<<10)|(20<<5)|( 5<<0)) /*!< TMR3 Module  \hideinitializer */
#define CLKO_MODULE      (( 1UL<<31)|( 2<<29)|(            7<<25)|( 4<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 6<<0)) /*!< CLKO Module  \hideinitializer */
#define I2C0_MODULE      (( 1UL<<31)|( 3<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 8<<0)) /*!< I2C0 Module  \hideinitializer */
#define I2C1_MODULE      (( 1UL<<31)|( 3<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|( 9<<0)) /*!< I2C1 Module  \hideinitializer */
#define ACMP0_MODULE     (( 1UL<<31)|( 3<<29)|( MODULE_NoMsk<<25)|( 0<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(11<<0)) /*!< ACMP0 Module  \hideinitializer */
#define SPI0_MODULE      (( 1UL<<31)|( 1<<29)|(            3<<25)|(24<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(12<<0)) /*!< SPI0 Module  \hideinitializer */
#define SPI1_MODULE      (( 1UL<<31)|( 2<<29)|(            3<<25)|(24<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(13<<0)) /*!< SPI1 Module  \hideinitializer */
#define SPI2_MODULE      (( 1UL<<31)|( 1<<29)|(            3<<25)|(26<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(14<<0)) /*!< SPI2 Module  \hideinitializer */
#define SPI3_MODULE      (( 1UL<<31)|( 2<<29)|(            3<<25)|(26<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(15<<0)) /*!< SPI3 Module  \hideinitializer */
#define UART0_MODULE     (( 1UL<<31)|( 1<<29)|(            7<<25)|( 0<<20)|( 0<<18)|(          0xF<<10)|( 8<<5)|(16<<0)) /*!< UART0 Module  \hideinitializer */
#define UART1_MODULE     (( 1UL<<31)|( 2<<29)|(            7<<25)|( 0<<20)|( 0<<18)|(          0xF<<10)|(12<<5)|(17<<0)) /*!< UART1 Module  \hideinitializer */
#define PWM0_MODULE      (( 1UL<<31)|( 1<<29)|(            1<<25)|( 4<<20)|( 3<<18)|( MODULE_NoMsk<<10)|( 0<<5)|(20<<0)) /*!< PWM0 Module  \hideinitializer */
#define ADC_MODULE       (( 1UL<<31)|( 1<<29)|(            7<<25)|(19<<20)|( 0<<18)|(         0xFF<<10)|(16<<5)|(28<<0)) /*!< ADC Module  \hideinitializer */
#define SC0_MODULE       (( 1UL<<31)|( 2<<29)|(            7<<25)|(16<<20)|( 0<<18)|(          0xF<<10)|(28<<5)|(30<<0)) /*!< SC0 Module  \hideinitializer */
#define SC1_MODULE       (( 1UL<<31)|( 2<<29)|(            7<<25)|(20<<20)|( 1<<18)|(          0xF<<10)|( 0<<5)|(31<<0)) /*!< SC1 Module  \hideinitializer */

/*@}*/ /* end of group NANO103_CLK_EXPORTED_CONSTANTS */


/** @addtogroup NANO103_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/
void CLK_DisableCKO(void);
void CLK_EnableCKO(uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32ClkDivBy1En);
void CLK_PowerDown(void);
void CLK_Idle(void);
uint32_t CLK_GetHXTFreq(void);
uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetPCLK0Freq(void);
uint32_t CLK_GetPCLK1Freq(void);
uint32_t CLK_GetCPUFreq(void);
uint32_t CLK_GetPLLClockFreq(void);
uint32_t CLK_SetCoreClock(uint32_t u32Hclk);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetPCLK0(uint32_t u32ClkDiv);
void CLK_SetPCLK1(uint32_t u32ClkDiv);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
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

/*@}*/ /* end of group NANO103_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group NANO103_CLK_Driver */

/*@}*/ /* end of group NANO103_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__CLK_H__

/*** (C) COPYRIGHT 2015 Nuvoton Technology Corp. ***/
