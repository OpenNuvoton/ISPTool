/**************************************************************************//**
 * @file     clk.h
 * @version  V1.00
 * $Revision: 6$
 * $Date: 14/07/17 2:35p $
 * @brief    ISD9100 series CLK driver header file
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#ifndef __CLK_H__
#define __CLK_H__

#ifdef __cplusplus
extern "C"
{
#endif


/** @addtogroup ISD9100_Device_Driver ISD9100 Device Driver
  @{
*/

/** @addtogroup ISD9100_CLK_Driver CLK Driver
  @{
*/

/** @addtogroup ISD9100_CLK_EXPORTED_CONSTANTS CLK Exported Constants
  @{
*/

/*---------------------------------------------------------------------------------------------------------*/
/*  PWRCON constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PWRCON_LXT_EN       (0x1ul << CLK_PWRCTL_LXTEN_Pos) /*!< Enable 32.768 kHz External Low Speed Crystal (LXT) */
#define CLK_PWRCON_HIRC_EN      (0x1ul << CLK_PWRCTL_HIRCEN_Pos) /*!< Enable 49 MHz Internal High Speed RC Oscillator (HIRC) */
#define CLK_PWRCON_LIRC_EN      (0x1ul << CLK_PWRCTL_LIRCEN_Pos) /*!< Enable 16 kHz Internal Low Speed RC Oscillator (LIRC) */

#define CLK_DPDWAKEUP_PINOSC16K   0      /*!< DPD wake up from WAKEUP pin and OSC 16k */  
#define CLK_DPDWAKEUP_PIN         1      /*!< DPD wake up from WAKEUP pin */
#define CLK_DPDWAKEUP_OSC16K      2      /*!< DPD wake up from OSC 16k */
#define CLK_DPDWAKEUP_POR         3      /*!< DPD wake up from POR event trigger */

#define CLK_DPDWAKETIME_12ms    (0x1ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 128 OSC16K clocks (12.8 ms) */
#define CLK_DPDWAKETIME_25ms    (0x2ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 128 OSC16K clocks (25.6 ms) */
#define CLK_DPDWAKETIME_50ms    (0x4ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 128 OSC16K clocks (51.2 ms) */
#define CLK_DPDWAKETIME_100ms   (0x8ul << CLK_PWRCTL_SELWKTMR_Pos)    /*!< WAKEUP after 128 OSC16K clocks (102.4 ms) */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL0 constant definitions.  (Write-protection)                                                      */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL0_HCLKSEL_HIRC        (0x00UL<<CLK_CLKSEL0_HCLKSEL_Pos) /*!< Setting clock source as internal 48MHz RC clock  \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_LXT         (0x01UL<<CLK_CLKSEL0_HCLKSEL_Pos) /*!< Setting clock source as external XTAL 32.768KHz \hideinitializer */
#define CLK_CLKSEL0_HCLKSEL_LIRC        (0x02UL<<CLK_CLKSEL0_HCLKSEL_Pos) /*!< Setting clock source as internal 16KHz RC clock  \hideinitializer */

#define CLK_CLKSEL0_STCLKSEL_LIRC        (0x00UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Setting clock source as internal 16KHz RC clock  \hideinitializer */
#define CLK_CLKSEL0_STCLKSEL_LXT         (0x01UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Setting clock source as external XTAL 32.768KHz \hideinitializer */
#define CLK_CLKSEL0_STCLKSEL_LIRC_DIV2   (0x02UL<<CLK_CLKSEL0_STCLKSEL_Pos)  /*!< Setting clock source as external 16KHz/2  \hideinitializer */
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
#define CLK_CLKSEL1_WDTSEL_LIRC          (0x3UL<<CLK_CLKSEL1_WDTSEL_Pos)       /*!< Setting WDT clock source as internal 16KHz RC clock  \hideinitializer */

#define CLK_CLKSEL1_DPWMSEL_HIRC         (0x0UL<<CLK_CLKSEL1_DPWMCKSEL_Pos)    /*!< Setting DPWM clock source as internal 48MHz RC clock  \hideinitializer */
#define CLK_CLKSEL1_DPWMSEL_HIRC2X       (0x1UL<<CLK_CLKSEL1_DPWMCKSEL_Pos)    /*!< Setting DPWM clock source as internal 2X 48MHz RC clock  \hideinitializer */

#define CLK_CLKSEL1_TMR0SEL_LIRC         (0x0UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as internal 16KHz RC clock  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as external XTAL 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_HCLK         (0x2UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_TMR0SEL_EXT          (0x3UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as external trigger (GPIOA[14])  \hideinitializer */    
#define CLK_CLKSEL1_TMR0SEL_HIRC         (0x7UL<<CLK_CLKSEL1_TMR0SEL_Pos)      /*!< Setting Timer 0 clock source as internal 48MHz RC clock  \hideinitializer */

#define CLK_CLKSEL1_TMR1SEL_LIRC         (0x0UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as internal 16KHz RC clock  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_LXT          (0x1UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as external XTAL 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_HCLK         (0x2UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_TMR1SEL_EXT          (0x3UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as external trigger (GPIOA[15])  \hideinitializer */    
#define CLK_CLKSEL1_TMR1SEL_HIRC         (0x7UL<<CLK_CLKSEL1_TMR1SEL_Pos)      /*!< Setting Timer 1 clock source as internal 48MHz RC clock  \hideinitializer */

#define CLK_CLKSEL1_PWM0CH01SEL_LIRC     (0x0UL<<CLK_CLKSEL1_PWM0CH01CKSEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as internal 16KHz RC clock  \hideinitializer */
#define CLK_CLKSEL1_PWM0CH01SEL_LXT      (0x1UL<<CLK_CLKSEL1_PWM0CH01CKSEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as external XTAL 32.768KHz  \hideinitializer */
#define CLK_CLKSEL1_PWM0CH01SEL_HCLK     (0x2UL<<CLK_CLKSEL1_PWM0CH01CKSEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL1_PWM0CH01SEL_HIRC     (0x3UL<<CLK_CLKSEL1_PWM0CH01CKSEL_Pos)   /*!< Setting PWM0 and PWM1 clock source as internal 48MHz RC clock  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKSEL2 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKSEL2_I2S0SEL_LIRC         (0x0UL<<CLK_CLKSEL2_I2S0SEL_Pos)      /*!< Setting I2S0 clock source as internal 16KHz RC clock  \hideinitializer */
#define CLK_CLKSEL2_I2S0SEL_LXT          (0x1UL<<CLK_CLKSEL2_I2S0SEL_Pos)      /*!< Setting I2S0 clock source as external XTAL 32.768KHz  \hideinitializer */
#define CLK_CLKSEL2_I2S0SEL_HCLK         (0x2UL<<CLK_CLKSEL2_I2S0SEL_Pos)      /*!< Setting I2S0 clock source as HCLK  \hideinitializer */
#define CLK_CLKSEL2_I2S0SEL_HIRC         (0x3UL<<CLK_CLKSEL2_I2S0SEL_Pos)      /*!< Setting I2S0 clock source as internal 48MHz RC clock  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  CLKDIV0 constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_CLKDIV0_HCLK(x)    (((x)-1) << CLK_CLKDIV0_HCLKDIV_Pos)    /*!< CLKDIV Setting for HCLK clock divider. It could be 1~16  \hideinitializer */
#define CLK_CLKDIV0_UART(x)    (((x)-1) << CLK_CLKDIV0_UARTDIV_Pos)    /*!< CLKDIV Setting for UR clock divider. It could be 1~16  \hideinitializer */
#define CLK_CLKDIV0_ADC(x)     (((x)-1) << CLK_CLKDIV0_ADCDIV_Pos)     /*!< CLKDIV Setting for ADC clock divider. It could be 1~256  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  PWRSTSF constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_PWRSTSF_SPDF		(CLK_PWRSTSF_SPDF_Msk)		/*!< Powered Down Flag (Standby (SPD))  \hideinitializer */
#define CLK_PWRSTSF_STOPF		(CLK_PWRSTSF_STOPF_Msk)		/*!< Stop Flag  \hideinitializer */
#define CLK_PWRSTSF_DSF			(CLK_PWRSTSF_DSF_Msk)		  /*!< Deep Sleep Flag  \hideinitializer */

/*---------------------------------------------------------------------------------------------------------*/
/*  LDOSEL constant definitions.                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define CLK_LDOSEL_3_0V			(0x0ul<<ANA_LDOSEL_LDOSEL_Pos)		/*!< Select LDO Output 3.0 Voltage  \hideinitializer */
#define CLK_LDOSEL_1_8V			(0x1ul<<ANA_LDOSEL_LDOSEL_Pos)		/*!< Select LDO Output 1.8 Voltage  \hideinitializer */
#define CLK_LDOSEL_2_4V			(0x2ul<<ANA_LDOSEL_LDOSEL_Pos)	  	/*!< Select LDO Output 2.4 Voltage  \hideinitializer */
#define CLK_LDOSEL_3_3V			(0x3ul<<ANA_LDOSEL_LDOSEL_Pos)		/*!< Select LDO Output 3.3 Voltage  \hideinitializer */

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
#define DPWM_MODULE      ((1UL<<30)|(1<<28)|(           1<<25)|( 4<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_DPWMCKEN_Pos)   /*!< DPWM Module  \hideinitializer */
#define UART_MODULE      ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(         0xF<<10)|( 8<<5)|CLK_APBCLK0_UARTCKEN_Pos)   /*!< UART Module  \hideinitializer */
#define BFAL_MODULE      ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_BFALCKEN_Pos)   /*!< BIQ And ALC Module  \hideinitializer */
#define CRC_MODULE       ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_CRCCKEN_Pos)    /*!< CRC Module  \hideinitializer */
#define PWM0_MODULE      ((1UL<<30)|(1<<28)|(           3<<25)|( 28<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_PWM0CH01CKEN_Pos) /*!< PWM0CH01 Module  \hideinitializer */
#define ACMP_MODULE      ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_ACMPCKEN_Pos)   /*!< ACMP Module  \hideinitializer */
#define SBRAM_MODULE     ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_SBRAMCKEN_Pos)   /*!< Standby RAM Module  \hideinitializer */
#define ADC_MODULE       ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(        0xFF<<10)|(16<<5)|CLK_APBCLK0_ADCCKEN_Pos)    /*!< ADC Module  \hideinitializer */
#define I2S0_MODULE      ((1UL<<30)|(3<<28)|(           3<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_I2S0CKEN_Pos)   /*!< I2S0 Module  \hideinitializer */
#define ANA_MODULE       ((1UL<<30)|(0<<28)|(MODULE_NoMsk<<25)|( 0<<20)|(0<<18)|(MODULE_NoMsk<<10)|( 0<<5)|CLK_APBCLK0_ANACKEN_Pos)   /*!< Analog Block Module  \hideinitializer */

/*@}*/ /* end of group ISD9100_CLK_EXPORTED_CONSTANTS */


/** @addtogroup ISD9100_CLK_EXPORTED_FUNCTIONS CLK Exported Functions
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

uint32_t CLK_GetLXTFreq(void);
uint32_t CLK_GetHCLKFreq(void);
uint32_t CLK_GetHIRCFreq(void);
void CLK_DisableModuleClock(uint32_t u32ModuleIdx);
void CLK_DisableXtalRC(uint32_t u32ClkMask);
void CLK_EnableModuleClock(uint32_t u32ModuleIdx);
void CLK_EnableXtalRC(uint32_t u32ClkMask);
void CLK_SetModuleClock(uint32_t u32ModuleIdx, uint32_t u32ClkSrc, uint32_t u32ClkDiv);
void CLK_SetHCLK(uint32_t u32ClkSrc, uint32_t u32HIRCType, uint32_t u32ClkDiv);
void CLK_SetSysTickClockSrc(uint32_t u32ClkSrc);
void CLK_SysTickDelay(uint32_t us);
void CLK_DeepPowerDown(uint32_t u32DPDWakeupMode, uint32_t u32TimerSel);
void CLK_StandbyPowerDown(void);
void CLK_DeepSleep(void);
void CLK_Sleep(void);
void CLK_EnableLDO( uint32_t u32LDOSel );
void CLK_DisableLDO(void);

/*@}*/ /* end of group ISD9100_CLK_EXPORTED_FUNCTIONS */

/*@}*/ /* end of group ISD9100_CLK_Driver */

/*@}*/ /* end of group ISD9100_Device_Driver */

#ifdef __cplusplus
}
#endif

#endif //__CLK_H__

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
