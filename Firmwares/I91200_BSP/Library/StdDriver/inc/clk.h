/**************************************************************************//**
 * @file     clk.h
 * @version  V1.01
 * $Revision: 6$
 * $Date: 16/12/09 2:35p $
 * @brief    I91200 series CLK driver header file
 *
 * @note
 * Copyright (C) Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __CLK_H__
#define __CLK_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup I91200_Device_Driver I91200 Device Driver
  @{
*/

/** @addtogroup I91200_CLK_Driver CLK Driver
  @{
*/

/** @addtogroup I91200_CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  PWRCON constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PWRCTL_LXT_EN       (CLK_PWRCTL_LXTEN_Msk)  	/*!< Enable 32.768 kHz External Low Speed Crystal (LXT) */
#define CLK_PWRCTL_HIRC_EN      (CLK_PWRCTL_HIRCEN_Msk)		/*!< Enable 49 MHz Internal High Speed RC Oscillator (HIRC) */
#define CLK_PWRCTL_LIRC_EN      (CLK_PWRCTL_LIRCEN_Msk)		/*!< Enable 10 kHz Internal Low Speed RC Oscillator (LIRC) */
#define CLK_PWRCTL_HXT_EN       (CLK_PWRCTL_HXTEN_Msk) 		/*!< Enable 12 MHz External high Speed Crystal (HXT) */

#define CLK_DPDWAKEUP_PINOSC10K   0      /*!< DPD wake up from WAKEUP pin and OSC 10k */  
#define CLK_DPDWAKEUP_PIN         1      /*!< DPD wake up from WAKEUP pin */
#define CLK_DPDWAKEUP_OSC10K      2      /*!< DPD wake up from OSC 10k */
#define CLK_DPDWAKEUP_POR         3      /*!< DPD wake up from POR event trigger */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL0 constant definitions.  (Write-protection)                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL0_HCLKSEL_HIRC         (0x00UL<<CLK_CLKSEL0_HCLKSEL_Pos) /*!< Setting clock source as internal 48MHz RC clock  \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_LXT          (0x01UL<<CLK_CLKSEL0_HCLKSEL_Pos) /*!< Setting clock source as external XTAL 32.768KHz \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_LIRC         (0x02UL<<CLK_CLKSEL0_HCLKSEL_Pos) /*!< Setting clock source as internal 10KHz RC clock  \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_HXT          (0x03UL<<CLK_CLKSEL0_HCLKSEL_Pos) /*!< Setting clock source as external XTAL 12MHz  \hideinitializer */

#define CLK_CLKSEL0_STCLKSEL_LIRC        (0x00UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Setting clock source as internal 10KHz RC clock  \hideinitializer */
#define CLK_CLKSEL0_STCLKSEL_LXT         (0x01UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Setting clock source as external XTAL 32.768KHz \hideinitializer */
#define CLK_CLKSEL0_STCLKSEL_LIRC_DIV2   (0x02UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Setting clock source as internal 10KHz RC clock/2  \hideinitializer */
#define CLK_CLKSEL0_STCLKSEL_HIRC_DIV2   (0x03UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Setting clock source as internal 48MHz RC clock/2 \hideinitializer */
#define CLK_CLKSEL0_STCLKSEL_HCLK_DIV2   (0x07UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Setting clock source as HCLK/2  \hideinitializer */

#define CLK_CLKSEL0_HIRCFSEL_48M         (0x00UL<<CLK_CLKSEL0_HIRCFSEL_Pos)  /*!< 49.152MHz RC trim clock for OSC48M(default)  \hideinitializer */
#define CLK_CLKSEL0_HIRCFSEL_32M         (0x01UL<<CLK_CLKSEL0_HIRCFSEL_Pos)  /*!< 32.768MHz RC trim clock for OSC48M  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL1 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL1_WDTSEL_HIRC          (0x0UL<<CLK_CLKSEL1_WDTSEL_Pos)       /*!< Setting WDT clock source as internal 48MHz RC clock  \hideinitializer */
#define CLK_CLKSEL1_WDTSEL_LXT           (0x1UL<<CLK_CLKSEL1_WDTSEL_Pos)       /*!< Setting WDT clock source as external XTAL 32.768KHz \hideinitializer */
#define CLK_CLKSEL1_WDTSEL_HCLK_DIV2048  (0x2UL<<CLK_CLKSEL1_WDTSEL_Pos)       /*!< Setting WDT clock source as HCLK/2048  \hideinitializer */
#define CLK_CLKSEL1_WDTSEL_LIRC          (0x3UL<<CLK_CLKSEL1_WDTSEL_Pos)       /*!< Setting WDT clock source as internal 10KHz RC clock  \hideinitializer */

#define CLK_CLKSEL1_SDADCSEL_HCLK        (0x0UL<<CLK_CLKSEL1_SDADCSEL_Pos)     /*!< Setting Delta-Sigma ADC clock source as HCLK/(SDADCDIV+1)  \hideinitializer */
#define CLK_CLKSEL1_SDADCSEL_HXT         (0x1UL<<CLK_CLKSEL1_SDADCSEL_Pos)     /*!< Setting Delta-Sigma ADC clock source as external XTAL 12MHz  \hideinitializer */

#define CLK_CLKSEL1_DPWMSEL_HCLK         (0x0UL<<CLK_CLKSEL1_DPWMSEL_Pos)    /*!< Setting DPWM clock source as HCLK/(DPWMDIV+1)  \hideinitializer */
#define CLK_CLKSEL1_DPWMSEL_HXT     	 (0x1UL<<CLK_CLKSEL1_DPWMSEL_Pos)    /*!< Setting DPWM clock source as external XTAL 12MHz  \hideinitializer */

#define CLK_CLKSEL1_TMR0SEL_LIRC         (0x0UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as internal 10KHz RC clock  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as external XTAL 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_HXT          (0x2UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as external XTAL 12MHz  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_EXT          (0x3UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as external trigger (GPIOA[10])  \hideinitializer */    
#define CLK_CLKSEL1_TMR0SEL_HCLK         (0x7UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as HCLK  \hideinitializer */

#define CLK_CLKSEL1_TMR1SEL_LIRC         (0x0UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as internal 10KHz RC clock  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as external XTAL 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_HXT          (0x2UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as external XTAL 12MHz  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_EXT          (0x3UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as external trigger (GPIOA[11])  \hideinitializer */    
#define CLK_CLKSEL1_TMR1SEL_HCLK         (0x7UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as HCLK  \hideinitializer */

#define CLK_CLKSEL1_SARADCSEL_HCLK       (0x0UL<<CLK_CLKSEL1_SARADCSEL_Pos)     /*!< Setting SAR ADC clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_SARADCSEL_LIRC       (0x1UL<<CLK_CLKSEL1_SARADCSEL_Pos)     /*!< Setting SAR ADC clock source as internal 16KHz RC clock  \hideinitializer */
#define CLK_CLKSEL1_SARADCSEL_HIRC       (0x2UL<<CLK_CLKSEL1_SARADCSEL_Pos)     /*!< Setting SAR ADC clock source as internal 48MHz RC clock  \hideinitializer */
#define CLK_CLKSEL1_SARADCSEL_LXT        (0x3UL<<CLK_CLKSEL1_SARADCSEL_Pos)     /*!< Setting SAR ADC clock source as external XTAL 32.768KHz  \hideinitializer */

#define CLK_CLKSEL1_PWM0CH01SEL_LIRC     (0x0UL<<CLK_CLKSEL1_PWM0CH01SEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as internal 10KHz RC clock  \hideinitializer */
#define CLK_CLKSEL1_PWM0CH01SEL_LXT      (0x1UL<<CLK_CLKSEL1_PWM0CH01SEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as external XTAL 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_PWM0CH01SEL_HCLK     (0x2UL<<CLK_CLKSEL1_PWM0CH01SEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_PWM0CH01SEL_HIRC     (0x3UL<<CLK_CLKSEL1_PWM0CH01SEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as internal 48MHz RC clock  \hideinitializer */

#define CLK_CLKSEL1_PWM0CH23SEL_LIRC     (0x0UL<<CLK_CLKSEL1_PWM0CH23SEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as internal 10KHz RC clock  \hideinitializer */
#define CLK_CLKSEL1_PWM0CH23SEL_LXT      (0x1UL<<CLK_CLKSEL1_PWM0CH23SEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as external XTAL 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_PWM0CH23SEL_HCLK     (0x2UL<<CLK_CLKSEL1_PWM0CH23SEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_PWM0CH23SEL_HIRC     (0x3UL<<CLK_CLKSEL1_PWM0CH23SEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as internal 48MHz RC clock  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL2 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL2_I2S0SEL_LIRC         (0x0UL<<CLK_CLKSEL2_I2S0SEL_Pos)      /*!< Setting I2S0 clock source as internal 10KHz RC clock  \hideinitializer */
#define CLK_CLKSEL2_I2S0SEL_LXT          (0x1UL<<CLK_CLKSEL2_I2S0SEL_Pos)      /*!< Setting I2S0 clock source as external XTAL 32.768KHz  \hideinitializer */
#define CLK_CLKSEL2_I2S0SEL_HCLK         (0x2UL<<CLK_CLKSEL2_I2S0SEL_Pos)      /*!< Setting I2S0 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL2_I2S0SEL_HIRC         (0x3UL<<CLK_CLKSEL2_I2S0SEL_Pos)      /*!< Setting I2S0 clock source as internal 48MHz RC clock  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKDIV0 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKDIV0_HCLK(x)    (((x)-1) << CLK_CLKDIV0_HCLKDIV_Pos)    /*!< CLKDIV Setting for HCLK clock divider. It could be 1~16  \hideinitializer */
#define CLK_CLKDIV0_BIQ(x)     (((x)-1) << CLK_CLKDIV0_BIQDIV_Pos)     /*!< CLKDIV Setting for BIQ filter clock divider. It could be 1~16  \hideinitializer */
#define CLK_CLKDIV0_UART0(x)   (((x)-1) << CLK_CLKDIV0_UART0DIV_Pos)    /*!< CLKDIV Setting for UART0 clock divider. It could be 1~16  \hideinitializer */
#define CLK_CLKDIV0_UART1(x)   (((x)-1) << CLK_CLKSEL2_UART1DIV_Pos)   /*!< CLKDIV Setting for UART0 clock divider. It could be 1~16  \hideinitializer */
#define CLK_CLKDIV0_DPWM(x)    (((x)-1) << CLK_CLKDIV0_DPWMDIV_Pos)    /*!< CLKDIV Setting for DPWM clock divider. It could be 1~16  \hideinitializer */
#define CLK_CLKDIV0_SDADC(x)   (((x)-1) << CLK_CLKDIV0_SDADCDIV_Pos)   /*!< CLKDIV Setting for Sigma-Delta ADC clock divider. It could be 1~256  \hideinitializer */
#define CLK_CLKDIV0_SARADC(x)  (((x)-1) << CLK_CLKDIV0_SARADCDIV_Pos)  /*!< CLKDIV Setting for SAR ADC clock divider. It could be 1~256  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  PWRSTSF constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PWRSTSF_SPDF		(CLK_PWRSTSF_SPDF_Msk)		/*!< Powered Down Flag (Standby (SPD))  \hideinitializer */
#define CLK_PWRSTSF_STOPF		(CLK_PWRSTSF_STOPF_Msk)		/*!< Stop Flag  \hideinitializer */
#define CLK_PWRSTSF_DSF			(CLK_PWRSTSF_DSF_Msk)		/*!< Deep Sleep Flag  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LDOSEL constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_LDOSEL_1_8V			(0x0ul<<ANA_LDOSEL_LDOSEL_Pos)		/*!< Select LDO Output 1.8 Voltage  \hideinitializer */
#define CLK_LDOSEL_2_4V			(0x1ul<<ANA_LDOSEL_LDOSEL_Pos)		/*!< Select LDO Output 2.4 Voltage  \hideinitializer */
#define CLK_LDOSEL_2_5V			(0x2ul<<ANA_LDOSEL_LDOSEL_Pos)	  	/*!< Select LDO Output 2.5 Voltage  \hideinitializer */
#define CLK_LDOSEL_2_7V			(0x3ul<<ANA_LDOSEL_LDOSEL_Pos)		/*!< Select LDO Output 2.7 Voltage  \hideinitializer */
#define CLK_LDOSEL_3_0V			(0x4ul<<ANA_LDOSEL_LDOSEL_Pos)		/*!< Select LDO Output 3.0 Voltage  \hideinitializer */
#define CLK_LDOSEL_3_3V			(0x5ul<<ANA_LDOSEL_LDOSEL_Pos)		/*!< Select LDO Output 3.3 Voltage  \hideinitializer */
#define CLK_LDOSEL_1_5V			(0x6ul<<ANA_LDOSEL_LDOSEL_Pos)	  	/*!< Select LDO Output 1.5 Voltage  \hideinitializer */
#define CLK_LDOSEL_1_7V			(0x7ul<<ANA_LDOSEL_LDOSEL_Pos)		/*!< Select LDO Output 1.7 Voltage  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/* VMID Constant Definitions                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_VMID_HIRES_CONNECT      (0x0ul << ANA_VMID_PDHIRES_Pos)     /*!< Connect the High Resistance reference to VMID */
#define CLK_VMID_HIRES_DISCONNECT   (0x1ul << ANA_VMID_PDHIRES_Pos)     /*!< The High Resistance reference is disconnected from VMID */
#define CLK_VMID_LORES_CONNECT      (0x0ul << ANA_VMID_PDLORES_Pos)     /*!< Connect the Low Resistance reference to VMID */
#define CLK_VMID_LORES_DISCONNECT   (0x1ul << ANA_VMID_PDLORES_Pos)     /*!< The Low Resistance reference is disconnected from VMID */

/*---------------------------------------------------------------------------------------------------------*/
/*  MODULE constant definitions.                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
#define MODULE_AHPBCLK(x)                  ((x >>30) & 0x3)    /*!< Calculate AHBCLK/APBCLK offset on MODULE index  \hideinitializer */
#define MODULE_CLKSEL(x)                   ((x >>28) & 0x3)    /*!< Calculate CLKSEL0~1 register offset by MODULE index  \hideinitializer */
#define MODULE_CLKSEL_Msk(x)               ((x >>25) & 0x7)    /*!< Calculate mask bits of CLKSEL0~1 by MODULE index  \hideinitializer */
#define MODULE_CLKSEL_Pos(x)               ((x >>20) & 0x1f)   /*!< Calculate mask bits offset of CLKSEL0~1 by MODULE index  \hideinitializer */
#define MODULE_CLKDIV(x)                   ((x >>18) & 0x3)    /*!< Calculate APBCLK CLKDIV by MODULE index  \hideinitializer */
#define MODULE_CLKDIV_Msk(x)               ((x >>10) & 0xff)   /*!< Calculate mask bits of CLKDIV by MODULE index  \hideinitializer */
#define MODULE_CLKDIV_Pos(x)               ((x >>5 ) & 0x1f)   /*!< Calculate ask bits offset of CLKDIV by MODULE index  \hideinitializer */
#define MODULE_IP_EN_Pos(x)                ((x >>0 ) & 0x1f)   /*!< Calculate APBCLK enabled offset by MODULE index  \hideinitializer */
#define MODULE_NoMsk                       0x0                 /*!< Not mask by MODULE index  \hideinitializer */
/*--------------------------------------------------------------------------------------------------------------------------------------*/
/*   AHBCLK/APBCLK(2) | CLKSEL(2) | CLKSEL_Msk(3) |    CLKSEL_Pos(5)    | CLKDIV(2) | CLKDIV_Msk(8) |     CLKDIV_Pos(5)  |  IP_EN_Pos(5)*/
/*--------------------------------------------------------------------------------------------------------------------------------------*/
#define PDMA_MODULE      ((0UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_PDMACKEN_Pos)    /*!< PDMA Module  \hideinitializer */
#define ISP_MODULE       ((0UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_AHBCLK_ISPCKEN_Pos)     /*!< ISP Module  \hideinitializer */
#define WDT_MODULE       ((1UL<<30)|(1<<28)|(           3<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_WDTCKEN_Pos)    /*!< Watchdog Timer Module  \hideinitializer */
#define RTC_MODULE       ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_RTCCKEN_Pos)    /*!< RTC Module  \hideinitializer */
#define TMR0_MODULE      ((1UL<<30)|(1<<28)|(           7<<25)|( 8<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_TMR0CKEN_Pos)   /*!< Timer0 Module  \hideinitializer */
#define TMR1_MODULE      ((1UL<<30)|(1<<28)|(           7<<25)|(12<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_TMR1CKEN_Pos)   /*!< Timer1 Module  \hideinitializer */
#define I2C0_MODULE      ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_I2C0CKEN_Pos)   /*!< I2C0 Module  \hideinitializer */
#define SPI0_MODULE      ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_SPI0CKEN_Pos)   /*!< SPI0 Module  \hideinitializer */
#define SPI1_MODULE      ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_SPI1CKEN_Pos)   /*!< SPI1 Module  \hideinitializer */
#define DPWM_MODULE      ((1UL<<30)|(1<<28)|(           1<<25)|( 4<<20)|(0<<18)|(         0xF<<10)|( 12<<5)|CLK_APBCLK0_DPWMCKEN_Pos)   /*!< DPWM Module  \hideinitializer */
#define UART0_MODULE     ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(         0xF<<10)|( 8<<5)|CLK_APBCLK0_UART0CKEN_Pos)   /*!< UART0 Module  \hideinitializer */
#define UART1_MODULE     ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(1<<18)|(         0xF<<10)|( 8<<5)|CLK_APBCLK0_UART1CKEN_Pos)   /*!< UART1 Module  \hideinitializer */
#define SARADC_MODULE    ((1UL<<30)|(1<<28)|(           3<<25)|(24<<20)|(0<<18)|(         0xFF<<10)|(24<<5)|CLK_APBCLK0_SARADCKEN_Pos)   /*!< SAR ADC Module  \hideinitializer */
#define BFAL_MODULE      ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(         0xF<<10)|( 4<<5)|CLK_APBCLK0_BIQALCKEN_Pos)   /*!< BIQ And ALC Module  \hideinitializer */
#define PWM0CH01_MODULE  ((1UL<<30)|(1<<28)|(           3<<25)|(28<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_PWM0CH01CKEN_Pos) /*!< PWM0CH01 Module  \hideinitializer */
#define PWM0CH23_MODULE  ((1UL<<30)|(1<<28)|(           3<<25)|(30<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_PWM0CH23CKEN_Pos) /*!< PWM0CH23 Module  \hideinitializer */
#define SDADC_MODULE     ((1UL<<30)|(1<<28)|(           1<<25)|( 2<<20)|(0<<18)|(        0xFF<<10)|(16<<5)|CLK_APBCLK0_SDADCCKEN_Pos)    /*!< Sigma-Delta ADC Module  \hideinitializer */
#define I2S0_MODULE      ((1UL<<30)|(3<<28)|(           3<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_I2S0CKEN_Pos)   /*!< I2S0 Module  \hideinitializer */
#define ANA_MODULE       ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_ANACKEN_Pos)   /*!< Analog Block Module  \hideinitializer */

/*@}*/ /* end of group I91200_CLK_EXPORTED_CONSTANTS */


/** @addtogroup I91200_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
  @{
*/

/**
  * @brief     Get power down state.
  * @param[in] clk Base address of CLK module.
  * @param[in] u8Flag Power state flag.
  *            - \ref CLK_PWRSTSF_SPDF
  *            - \ref CLK_PWRSTSF_STOPF
  *            - \ref CLK_PWRSTSF_DSF
  * @return    1: power down flag assert, 0: opposite.
  */
#define CLK_GET_POWERDOWNFLAG(clk, u8Flag)		(clk->PWRSTSF&u8Flag)

/**
  * @brief     Clear power down state.
  * @param[in] clk Base address of CLK module.
  * @param[in] u8Flag Power state flag.
  *            - \ref CLK_PWRSTSF_SPDF
  *            - \ref CLK_PWRSTSF_STOPF
  *            - \ref CLK_PWRSTSF_DSF
  * @return    None.
  */
#define CLK_CLEAR_POWERDOWNFLAG(clk, u8Flag)	(clk->PWRSTSF |= u8Flag)

/**
  * @brief      Enable VMID reference voltage
  * @param      clk Base address of CLK module
  * @param      Hires is High Resistance reference to VMID
  *             - \ref CLK_VMID_HIRES_CONNECT
  *             - \ref CLK_VMID_HIRES_DISCONNECT
  * @param      Lores is Low Resistance reference to VMID
  *             - \ref CLK_VMID_LORES_CONNECT
  *             - \ref CLK_VMID_LORES_DISCONNECT
  * @return     None
  * @details    The VMID needs to be enabled for operation before using the SDADC/SARADC, PGA or other analog blocks.
  */
#define CLK_ENABLE_VMID(clk, \
                        Hires, \
						Lores)  (ANA->VMID = (0x0|Hires|Lores))

/**
  * @brief      Disable VMID reference voltage
  * @param      clk Base address of CLK module
  * @return     None
  */
#define CLK_DISABLE_VMID(clk)	(ANA->VMID |= 0x7)

uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetHIRCFreq(void);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32HIRCType, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_SysTickDelay(uint32_t us);
void CLK_DeepPowerDown(uint32_t u32DPDWakeupMode, uint32_t u32Reserved);
void CLK_StandbyPowerDown(uint8_t u8IOHold);
void CLK_Stop(uint8_t u8FlashOnOff);
void CLK_DeepSleep(void);
void CLK_Sleep(void);
void CLK_EnableLDO( uint32_t u32LDOSel );
void CLK_DisableLDO(void);

/*@}*/ /* end of group I91200_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group I91200_CLK_Driver */

/*@}*/ /* end of group I91200_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__CLK_H__

/*** (C) COPYRIGHT Nuvoton Technology Corp. ***/
